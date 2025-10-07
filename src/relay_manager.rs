//! # Relay Manager - Network Topology and Forwarding Decisions
//!
//! This module implements the relay management system that maintains network topology
//! knowledge and makes intelligent forwarding decisions for messages in the mesh network.
//! It tracks connection quality between nodes and determines optimal relay paths.
//!
//! ## Architecture
//!
//! The RelayManager maintains:
//! - **Connection Matrix**: NxN grid tracking link quality between all known node pairs
//! - **Wait Pool**: Pending messages awaiting responses (request-response pattern)
//! - **Echo System**: Active probing mechanism to discover and measure connections
//! - **Scoring Algorithm**: Multi-factor evaluation for relay decisions
//!
//! ## Key Components
//!
//! - **Connection Matrix**: Stores link quality (0-63) and dirty flags for each node pair
//!   - Lower 6 bits: link quality value (0 = no connection, 63 = excellent)
//!   - Upper 2 bits: dirty counter for aging and cleanup
//!
//! - **Echo Protocol**: Periodic broadcast-response mechanism for topology discovery
//!   - Nodes broadcast echo requests at configurable intervals
//!   - Neighbors respond with echo replies containing connection quality data
//!   - Gathering phase collects responses within a timeout window
//!
//! - **Relay Decision Logic**: Multi-factor scoring to determine if/how to forward messages
//!   - Factors: hop count, destination awareness, connection quality, message type
//!   - Result: None (don't relay), SendMessage (relay), or AlreadyHaveMessage (duplicate)
//!
//! ## Connection Quality
//!
//! Link quality is measured using RSSI (Received Signal Strength Indicator) values:
//! - Range: 0-63 (6-bit value)
//! - Updated dynamically as packets are received
//! - Used to prefer stronger relay paths
//! - Aged out using dirty flags to remove stale connections
//!
//! ## Design Considerations
//!
//! - Connection matrix size is compile-time configurable (CONNECTION_MATRIX_SIZE)
//! - Echo intervals and timeouts are runtime configurable
//! - Scoring algorithm balances hop count minimization with connection quality
//! - Wait pool enables request-response pattern tracking
//! - Dirty flag mechanism prevents unbounded memory of stale connections

use core::cmp::{max, min};
use embassy_time::{Duration, Instant};
use log::log;

use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

use crate::ECHO_RESPONSES_WAIT_POOL_SIZE;
use crate::MessageProcessingResult;
use crate::RadioPacket;
use crate::ScoringMatrix;
use crate::wait_pool::WaitPool;
use crate::{MessageType, RadioMessage};

/// Type alias for the connection quality matrix
///
/// A 2D array tracking link quality between all node pairs in the network.
/// However we use this data structure also in wait_pool.rs it belongs here logically
/// The matrix is indexed by node position (not node ID), where:
/// - `ConnectionMatrix[i][j]` = link quality from node i to node j
/// - Lower 6 bits: quality value (0-63)
/// - Upper 2 bits: dirty counter for aging stale connections
///
/// # Size
/// The matrix size is fixed at compile-time by `CONNECTION_MATRIX_SIZE` (100 nodes),
/// limiting the maximum number of nodes that can be tracked simultaneously.
///
/// # Usage
/// Used throughout the relay manager and wait pool for network topology tracking.
pub(crate) type ConnectionMatrix = [[u8; crate::CONNECTION_MATRIX_SIZE]; crate::CONNECTION_MATRIX_SIZE];

/// Type alias for a single row in the connection quality matrix
///
/// Represents the connection quality values from one node to all other nodes.
/// However we use this data structure also in wait_pool.rs it belongs here logically
/// Each element stores:
/// - Lower 6 bits: quality value (0-63)
/// - Upper 2 bits: dirty counter for aging stale connections
///
/// # Size
/// The row size is fixed at compile-time by `CONNECTION_MATRIX_SIZE` (100 nodes),
/// matching the number of nodes tracked in the connection matrix.
///
/// # Usage
/// Used when passing individual node connection data, particularly in relay
/// scoring calculations where a sender's connections are evaluated.
pub(crate) type ConnectionMatrixRow = [u8; crate::CONNECTION_MATRIX_SIZE];

// Bitmask helpers for connection matrix cells

/// Bitmask for dirty counter bits in connection matrix cells
///
/// Upper 2 bits of each connection quality byte are used as a counter
/// to track how long a connection has been inactive, enabling cleanup.
const DIRTY_MASK: u8 = 0b1100_0000;

/// Bitmask for quality value in connection matrix cells
///
/// Lower 6 bits store the actual link quality (0-63 scale).
const QUALITY_MASK: u8 = 0b0011_1111;

/// Bit shift amount to access dirty counter
const DIRTY_SHIFT: u8 = 6;

///check to ensure MAX_DIRTY_COUNT fits within the dirty bits
const _: () = assert!(MAX_DIRTY_COUNT <= (DIRTY_MASK >> DIRTY_SHIFT), "MAX_DIRTY_COUNT exceeds dirty bits capacity");

/// Maximum dirty count before connection is reset to zero
///
/// When a connection reaches this dirty count without updates, it's
/// considered stale and removed from the matrix.
const MAX_DIRTY_COUNT: u8 = 3;

/// Result of evaluating whether to relay a message
///
/// Returned by the relay manager to indicate the decision on forwarding
/// a received message.
/// the default use-case for this item is to report back relayed messages, so the smaller variants are not a real problem here.
#[allow(clippy::large_enum_variant)]
pub(crate) enum RelayResult {
    /// Do not relay this message
    None,

    /// Relay this message to the network
    SendMessage(RadioMessage),

    /// Message was already processed (duplicate)
    AlreadyHaveMessage,
}

/// Creates an empty connection quality array
///
/// Helper function to initialize connection arrays with zeros.
fn empty_connections() -> ConnectionMatrixRow {
    [0; crate::CONNECTION_MATRIX_SIZE]
}

/// A queued echo response awaiting transmission
///
/// Echo responses are not sent immediately when an echo request is received.
/// Instead, they are queued with a random delay to avoid network congestion
/// when many nodes simultaneously respond to the same echo request.
///
/// # Fields
///
/// * `send_time` - Scheduled time to send this echo response
/// * `target_node_id` - Node ID to send the echo response to
/// * `link_quality` - Measured link quality value to report (0-63)
///
/// # Timing Strategy
///
/// Each echo response is given a random delay up to `echo_gathering_timeout * 30` seconds.
/// This staggers responses across time, preventing burst traffic that could overwhelm
/// the network when many nodes receive the same echo request broadcast.
#[derive(Clone, Copy, Debug)]
struct EchoResponseWaitPoolItem {
    /// Scheduled time when this echo response should be sent
    send_time: Instant,

    /// Target node ID to send the echo response to
    target_node_id: u32,

    /// Link quality value to report in the echo response (0-63)
    link_quality: u8,
}

impl EchoResponseWaitPoolItem {
    /// Creates a new echo response wait pool item
    ///
    /// # Arguments
    ///
    /// * `send_time` - When to send this echo response
    /// * `target_node_id` - Node to send the response to
    /// * `link_quality` - Link quality value to report
    fn with(send_time: Instant, target_node_id: u32, link_quality: u8) -> Self {
        Self {
            send_time,
            target_node_id,
            link_quality,
        }
    }
}

/// Relay Manager - Network Topology Tracking and Forwarding Coordinator
///
/// Manages the connection quality matrix, echo protocol, and makes intelligent
/// relay decisions for message forwarding in the mesh network.
///
/// # Structure
///
/// - **Connection Matrix**: NxN grid tracking link quality between all known nodes
/// - **Wait Pool**: Pending messages awaiting relay evaluation
/// - **Echo System**: Active network probing to discover and measure connections
///
/// # Generic Parameters
///
/// * `CONNECTION_MATRIX_SIZE` - Maximum number of nodes to track in the matrix
/// * `WAIT_POOL_SIZE` - Maximum number of messages pending relay decision
pub(crate) struct RelayManager<const CONNECTION_MATRIX_SIZE: usize, const WAIT_POOL_SIZE: usize> {
    /// Connection quality matrix (NxN)
    ///
    /// Matrix[i][j] stores the link quality from node i to node j.
    /// Lower 6 bits: quality (0-63), upper 2 bits: dirty counter.
    connection_matrix: ConnectionMatrix,

    /// Node IDs corresponding to matrix indices
    ///
    /// Maps matrix index to actual node ID in the network.
    connection_matrix_nodes: [u32; CONNECTION_MATRIX_SIZE],

    /// Number of known nodes in the network
    connected_nodes_count: usize,

    /// Wait pool for messages pending relay decisions
    wait_pool: WaitPool<WAIT_POOL_SIZE, CONNECTION_MATRIX_SIZE>,

    /// Buffered echo responses waiting to be sent
    ///
    /// Each slot holds an optional echo response scheduled for future transmission.
    /// Echo responses are delayed randomly to prevent network congestion.
    echo_responses_wait_pool: [Option<EchoResponseWaitPoolItem>; ECHO_RESPONSES_WAIT_POOL_SIZE],

    /// Next scheduled time to send an echo request
    next_echo_request_time: Instant,

    /// End time for current echo gathering phase (if active)
    echo_gathering_end_time: Option<Instant>,

    /// Minimum seconds between echo broadcasts
    echo_request_minimal_interval: u32,

    /// Target number of timeouts between echoes for the visible parts of the network (adaptive)
    echo_messages_target_interval: u8,

    /// Seconds to wait for echo responses after broadcasting request
    echo_gathering_timeout: u8,

    /// This node's unique network identifier
    own_node_id: u32,

    /// Random number generator for timing jitter
    rng: WyRand,
}

impl<const CONNECTION_MATRIX_SIZE: usize, const WAIT_POOL_SIZE: usize> RelayManager<CONNECTION_MATRIX_SIZE, WAIT_POOL_SIZE> {
    /// Creates a new RelayManager
    ///
    /// Initializes the connection matrix with this node as the first entry
    /// and schedules the first echo request.
    ///
    /// # Arguments
    ///
    /// * `echo_request_minimal_interval` - Minimum seconds between echo broadcasts
    /// * `echo_messages_target_interval` - Target messages received between echoes
    /// * `echo_gathering_timeout` - Seconds to collect echo responses
    /// * `wait_position_delay` - Base delay for relay position calculation
    /// * `scoring_matrix` - Configuration for relay scoring
    /// * `own_node_id` - This node's unique identifier
    /// * `rng_seed` - Seed for random number generation
    pub(crate) fn with(
        echo_request_minimal_interval: u32,
        echo_messages_target_interval: u8,
        echo_gathering_timeout: u8,
        wait_position_delay: u8,
        scoring_matrix: ScoringMatrix,
        own_node_id: u32,
        rng_seed: u64,
    ) -> Self {
        let mut rng = WyRand::seed_from_u64(rng_seed);
        let mut result = RelayManager {
            connection_matrix: [[0; crate::CONNECTION_MATRIX_SIZE]; crate::CONNECTION_MATRIX_SIZE],
            connection_matrix_nodes: [0; CONNECTION_MATRIX_SIZE],
            connected_nodes_count: 1, // Start with one node (own node)

            wait_pool: WaitPool::with(wait_position_delay as u64, scoring_matrix, rng.next_u64()),
            echo_responses_wait_pool: [None; ECHO_RESPONSES_WAIT_POOL_SIZE],
            next_echo_request_time: Instant::now() + Duration::from_secs(rng.next_u64() % echo_request_minimal_interval as u64),
            echo_gathering_end_time: None,
            echo_request_minimal_interval,
            echo_messages_target_interval,
            echo_gathering_timeout,
            own_node_id,
            rng,
        };
        result.connection_matrix_nodes[0] = own_node_id; // Initialize the first node as own_node_id
        result.connection_matrix[0][0] = 63; // Self-connection with max quality
        result
    }

    /// Calculates the next timeout instant for timed tasks
    ///
    /// Determines the earliest time when a timed event needs to be processed,
    /// considering all pending activities: echo requests, echo gathering, wait
    /// pool activations, and queued echo responses.
    ///
    /// # Returns
    /// The earliest `Instant` when a timeout-based action is required
    ///
    /// # Used By
    /// The main loop uses this to set Embassy timer durations for efficient
    /// async waiting without busy polling.
    pub(crate) fn calculate_next_timeout(&self) -> Instant {
        let mut min_timeout = self.next_echo_request_time;
        if let Some(timeout) = self.echo_gathering_end_time {
            min_timeout = min(min_timeout, timeout);
        }
        if let Some(timeout) = self.wait_pool.next_activation_time() {
            min_timeout = min(min_timeout, timeout);
        }

        for i in 0..self.echo_responses_wait_pool.len() {
            if let Some(item) = &self.echo_responses_wait_pool[i] {
                min_timeout = min(min_timeout, item.send_time);
            }
        }
        min_timeout
    }

    /// Adds an echo response to the wait pool with random delay
    ///
    /// Queues an echo response to be sent after a random delay. This staggers
    /// echo replies to avoid network congestion when many nodes respond to
    /// the same echo request simultaneously.
    ///
    /// # Arguments
    /// * `node_id` - Target node ID to send the echo response to
    /// * `quality` - Link quality value to report in the echo
    ///
    /// # Returns
    /// * `None` - Echo was queued successfully
    /// * `Some(RadioMessage)` - Pool was full, returns displaced echo to send immediately
    ///
    /// # Behavior
    /// - Adds random delay up to (echo_gathering_timeout * 30) seconds
    /// - If pool is full, displaces the echo with the earliest send time (because we send the oldest one immediately)
    /// - Displaced echo is returned for immediate transmission
    fn add_echo_response(&mut self, node_id: u32, quality: u8) -> Option<RadioMessage> {
        let send_time = Instant::now() + Duration::from_secs(self.rng.next_u64() % (self.echo_gathering_timeout as u64 * 60 / 2)); // Random delay up to half of echo_gathering_timeout in seconds
        for slot in self.echo_responses_wait_pool.iter_mut() {
            if slot.is_none() {
                *slot = Some(EchoResponseWaitPoolItem::with(send_time, node_id, quality));
                return None;
            }
        }

        // Pool full, replace the item with the earliest send time
        // Because the code runs on single-threaded executor we can assume every slot is filled
        let mut next_index = 0;
        let mut next_end_time = Instant::now();
        for i in 0..self.echo_responses_wait_pool.len() {
            if let Some(item) = &self.echo_responses_wait_pool[i] {
                if i == 0 || item.send_time < next_end_time {
                    next_end_time = item.send_time;
                    next_index = i;
                }
            }
        }
        let mut result = None;
        log!(log::Level::Debug, "Echo responses wait pool full, replacing oldest entry.");
        if let Some(old_item) = &self.echo_responses_wait_pool[next_index] {
            result = Some(RadioMessage::echo_with(self.own_node_id, old_item.target_node_id, old_item.link_quality));
        }
        self.echo_responses_wait_pool[next_index] = Some(EchoResponseWaitPoolItem::with(send_time, node_id, quality));

        result
    }

    /// Processes time-based tasks and returns messages to send
    ///
    /// Checks all time-based activities and generates messages when timeouts
    /// expire. This includes echo requests, echo results, wait pool activations,
    /// and queued echo responses.
    ///
    /// # Returns
    /// * `RelayResult::SendMessage` - A message is ready to be transmitted
    /// * `RelayResult::None` - No time-based action is due yet
    ///
    /// # Tasks Processed
    /// 1. **Echo Request**: Broadcasts topology discovery at configured intervals
    /// 2. **Echo Result**: Sends aggregated connection matrix after gathering phase
    /// 3. **Wait Pool**: Activates messages whose relay delay has expired
    /// 4. **Echo Responses**: Sends queued echo replies when their delay expires
    ///
    /// # Timing
    /// - Echo requests: Adaptive interval based on network size
    /// - Echo results: After echo_gathering_timeout window
    /// - Wait pool: Per-message calculated delay based on relay position
    /// - Echo responses: Random delay up to gathering timeout

    #[allow(dead_code)]
    const _ASSERT_CONNECTION_MATRIX_SQUARE_FITS_U32: () = assert!(
        CONNECTION_MATRIX_SIZE * CONNECTION_MATRIX_SIZE <= u32::MAX as usize,
        "CONNECTION_MATRIX_SIZE^2 must fit in u32"
    );

    pub(crate) fn process_timed_tasks(&mut self) -> RelayResult {
        if Instant::now() >= self.next_echo_request_time {
            log::debug!("Sending echo request");
            // Calculate adaptive echo request interval based on network size. This calculation will never overflow as CONNECTION_MATRIX_SIZE is limited to small numbers (hardware limitation, because of memory size). We don't use checks for optimal performance.
            // Echo request + Echo responses + Echo results
            let message_count = if self.connected_nodes_count == 0 {
                1 // minimum
            } else {
                max(
                    2 * self.connected_nodes_count + self.connected_nodes_count * (self.connected_nodes_count - 1),
                    1,
                )
            };
            let echo_request_interval = self.rng.next_u32()
                % (2 * max(
                    self.echo_messages_target_interval as u32 * message_count as u32,
                    self.echo_request_minimal_interval,
                ))
                + self.rng.next_u32() % (self.echo_gathering_timeout as u32 * 60);

            self.echo_gathering_end_time = Some(Instant::now() + Duration::from_secs(self.echo_gathering_timeout as u64 * 60)); //multiply by 60 to convert minutes to seconds

            // Send an echo request to all connected nodes
            let echo_request = RadioMessage::request_echo_with(self.own_node_id);
            self.next_echo_request_time = Instant::now() + Duration::from_secs(echo_request_interval as u64);
            return RelayResult::SendMessage(echo_request);
        }

        if Instant::now() >= self.echo_gathering_end_time.unwrap_or(Instant::MAX) {
            log::debug!("Echo gathering timeout passed, sending echo results");
            // If echo gathering timeout has passed, reset the echo gathering end time
            self.echo_gathering_end_time = None;
            // Send an echo result message with the current connection matrix
            let mut echo_result = RadioMessage::echo_result_with(self.own_node_id);
            for i in 1..self.connected_nodes_count {
                let neighbor_node = self.connection_matrix_nodes[i];
                let send_link_matrix_item = self.connection_matrix[0][i]; // Get the link quality from the connection matrix
                let receive_link_matrix_item = self.connection_matrix[i][0]; // Get the link quality from the connection matrix
                let send_link_quality = send_link_matrix_item & QUALITY_MASK; // Get the link quality from the connection matrix
                let receive_link_quality = receive_link_matrix_item & QUALITY_MASK; // Get the link quality from the connection matrix
                if (send_link_matrix_item & DIRTY_MASK == 0 || receive_link_matrix_item & DIRTY_MASK == 0)
                    && (send_link_quality != 0 || receive_link_quality != 0)
                    && echo_result
                        .add_echo_result_item(neighbor_node, send_link_quality, receive_link_quality)
                        .is_err()
                {
                    log::warn!("Echo result message full, cannot add more items");
                    break;
                }
            }
            return RelayResult::SendMessage(echo_result);
        }

        // Process wait pool items
        if let Some(mut message) = self.wait_pool.get_next_activated_message() {
            message.set_sender_node_id(self.own_node_id);
            if let Some(sequence) = message.sequence() {
                log::debug!("Sending relayed message: sequence: {}", sequence);
            }
            return RelayResult::SendMessage(message);
        }

        for i in 0..self.echo_responses_wait_pool.len() {
            if let Some(item) = &self.echo_responses_wait_pool[i] {
                if item.send_time <= Instant::now() {
                    let response = RadioMessage::echo_with(self.own_node_id, item.target_node_id, item.link_quality);

                    self.echo_responses_wait_pool[i] = None; // Clear the slot
                    return RelayResult::SendMessage(response);
                }
            }
        }

        RelayResult::None
    }

    /// Updates connection matrix based on received radio packet
    ///
    /// Records the link quality from the sender to this node and updates the
    /// wait pool with the sender's connection information. This is called for
    /// every received packet to maintain fresh connection data.
    ///
    /// # Arguments
    /// * `packet` - The received radio packet
    /// * `link_quality` - Measured link quality of the received packet (0-63)
    ///
    /// # Updates
    /// - Connection matrix: sender -> own node link quality
    /// - Wait pool: Provides sender's connection vector for relay scoring
    pub(crate) fn process_received_packet(&mut self, packet: &RadioPacket, link_quality: u8) {
        //Linear search is fine here, as CONNECTION_MATRIX_SIZE is limited to small numbers (hardware limitation, because of memory size). We don't use more complex data structures for optimal performance.
        let sender_index = self.connection_matrix_nodes.iter().position(|&id| id == packet.sender_node_id()).unwrap_or(0);
        let sender_connections = if sender_index > 0 {
            self.connection_matrix[sender_index][0] = link_quality;
            self.connection_matrix[sender_index]
        } else {
            empty_connections()
        };

        self.wait_pool
            .update_with_received_packet(packet, &self.connection_matrix, &self.connection_matrix[0], &sender_connections);
    }

    /// Processes a received message and updates network state
    ///
    /// Main entry point for handling received messages. Updates the connection
    /// matrix, processes protocol messages (echo, echo result), and checks for
    /// duplicates in the wait pool.
    ///
    /// # Arguments
    /// * `message` - The received radio message
    /// * `last_link_quality` - Link quality of the last received packet (0-63)
    ///
    /// # Returns
    /// * `RelayResult::SendMessage` - Generated response (e.g., echo reply)
    /// * `RelayResult::AlreadyHaveMessage` - Message is duplicate, already in wait pool
    /// * `RelayResult::None` - Message processed, no immediate action
    ///
    /// # Processing Steps
    /// 1. Add sender to connection matrix if unknown
    /// 2. Handle echo protocol messages (request, response, result)
    /// 3. Update connection matrix with link quality data
    /// 4. Check for duplicates in wait pool
    /// 5. Return appropriate result
    ///
    /// # Connection Matrix Management
    /// - New nodes are added when space available
    /// - When full, replaces lowest quality connection
    /// - Notifies wait pool when nodes are replaced
    pub(crate) fn process_received_message(&mut self, message: &RadioMessage, last_link_quality: u8) -> RelayResult {
        // find the connection matrix index for the sender node
        // Linear search is fine here, as CONNECTION_MATRIX_SIZE is limited to small numbers (hardware limitation, because of memory size). We don't use more complex data structures for optimal performance.
        let mut sender_index_opt = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id());
        if sender_index_opt.is_none() {
            log::trace!("Adding new sender node to connection matrix: {}", message.sender_node_id());
            // If the sender node is not in the connection matrix, add it
            if self.connected_nodes_count < CONNECTION_MATRIX_SIZE {
                sender_index_opt = Some(self.connected_nodes_count);
                // Initialize self-connection only; defer link qualities to echo traffic
                self.connection_matrix[self.connected_nodes_count][self.connected_nodes_count] = 63;
                self.connection_matrix_nodes[self.connected_nodes_count] = message.sender_node_id();
                self.connected_nodes_count += 1;
            } else {
                log::trace!("Connection matrix full, finding lowest quality node to replace");
                //find the node with the lowest link quality and replace it
                let mut lowest_quality_index = CONNECTION_MATRIX_SIZE;
                let mut lowest_quality = 255;
                for i in 1..CONNECTION_MATRIX_SIZE {
                    let value = self.connection_matrix[i][0] & QUALITY_MASK; // Get the link quality from the connection matrix
                    if value < lowest_quality || lowest_quality_index == CONNECTION_MATRIX_SIZE {
                        lowest_quality = value;
                        lowest_quality_index = i;
                    }
                }

                // If we found a node with lower link quality, replace it
                if lowest_quality < last_link_quality {
                    log::trace!(
                        "Replacing node {} with new node {} in connection matrix",
                        self.connection_matrix_nodes[lowest_quality_index],
                        message.sender_node_id()
                    );
                    sender_index_opt = Some(lowest_quality_index);
                    // Notify wait pool that a connection matrix item has changed
                    self.wait_pool.connection_matrix_item_changed(lowest_quality_index);
                    self.connection_matrix_nodes[lowest_quality_index] = message.sender_node_id();
                    self.connection_matrix[lowest_quality_index] = empty_connections();
                    self.connection_matrix[lowest_quality_index][lowest_quality_index] = 63;
                }
            }
        }

        let sender_index = if let Some(index) = sender_index_opt {
            index
        } else {
            //drop the message if we cannot find a place for it
            return RelayResult::None;
        };

        // If message is an echo_request increment dirty counter in connection matrix
        if message.message_type() == MessageType::RequestEcho as u8 {
            //set the dirty flag on connection matrix or zero the connection matrix item if it is already set
            for i in 0..CONNECTION_MATRIX_SIZE {
                // Skip the diagonal (sender to itself)
                if i == sender_index {
                    continue;
                }
                let value = self.connection_matrix[sender_index][i];
                // get the upper 2 bits of the value as a small counter
                let mut counter = (value & DIRTY_MASK) >> DIRTY_SHIFT;
                counter = counter.saturating_add(1);
                // If counter exceeds 3, zero the connection matrix cell
                if counter > MAX_DIRTY_COUNT {
                    self.connection_matrix[sender_index][i] = 0;
                } else {
                    // preserve existing link quality bits, update only the dirty counter
                    self.connection_matrix[sender_index][i] = (counter << DIRTY_SHIFT) | (value & QUALITY_MASK);
                }
            }

            // Do not overwrite the cell here; only mark dirty above. Quality is preserved per tests.
            //send a reply echo message
            //let echo_response = RadioMessage::new_echo(self.own_node_id, message.sender_node_id(), last_link_quality);
            let result_opt = self.add_echo_response(message.sender_node_id(), last_link_quality);

            if let Some(result) = result_opt {
                return RelayResult::SendMessage(result);
            }

            return RelayResult::None;
        }

        // If message is an echo_response
        if message.message_type() == MessageType::Echo as u8 {
            if let Some((target_node, link_quality)) = message.get_echo_data() {
                let target_index_opt = self.connection_matrix_nodes.iter().position(|&id| id == target_node);
                if let Some(target_index) = target_index_opt {
                    // Update the connection matrix with the link quality
                    // Record last link quality from sender to self (column 0)
                    self.connection_matrix[sender_index][0] = last_link_quality;
                    // Record echo-reported link from sender -> target (row: sender, col: target)
                    self.connection_matrix[sender_index][target_index] = link_quality;
                }
            }
        }

        //If message is an echo_result
        if message.message_type() == MessageType::EchoResult as u8 {
            if let Some(iterator) = message.get_echo_result_data_iterator() {
                for echo_result_item in iterator {
                    let mut target_index_opt = self.connection_matrix_nodes.iter().position(|&id| id == echo_result_item.neighbor_node);
                    if target_index_opt.is_none() {
                        //add to connection matrix if there is space
                        if self.connected_nodes_count < CONNECTION_MATRIX_SIZE {
                            target_index_opt = Some(self.connected_nodes_count);
                            // Initialize self-connection only; defer link qualities to echo traffic
                            self.connection_matrix[self.connected_nodes_count][self.connected_nodes_count] = 63;
                            self.connection_matrix_nodes[self.connected_nodes_count] = echo_result_item.neighbor_node;
                            self.connected_nodes_count += 1;
                        }
                    }
                    let send_link_quality = echo_result_item.send_link_quality;
                    let receive_link_quality = echo_result_item.receive_link_quality;
                    if let Some(target_index) = target_index_opt {
                        // Update the connection matrix with the link quality
                        self.connection_matrix[sender_index][target_index] = send_link_quality;
                        self.connection_matrix[target_index][sender_index] = receive_link_quality;
                    }
                }
            }
        }

        if (message.message_type() == MessageType::AddBlock as u8
            || message.message_type() == MessageType::AddTransaction as u8
            || message.message_type() == MessageType::Support as u8
            || message.message_type() == MessageType::RequestFullBlock as u8
            || message.message_type() == MessageType::RequestNewMempoolItem as u8)
            && self.wait_pool.contains_message_or_reply(message)
        {
            self.wait_pool.update_message(
                message,
                &self.connection_matrix,
                &self.connection_matrix[0],
                &self.connection_matrix[sender_index],
            );
            return RelayResult::AlreadyHaveMessage;
        }

        RelayResult::None
    }

    /// Processes message handling results from application layer
    ///
    /// Handles the outcome of message processing by the application, adding
    /// appropriate messages to the wait pool for relay or response. This is
    /// the bridge between application logic and relay decisions.
    ///
    /// # Arguments
    /// * `result` - The processing result from the application layer
    ///
    /// # Handled Results
    /// - **RequestedBlockNotFound**: Add request for missing block to wait pool
    /// - **RequestedBlockFound**: Add found block to wait pool for relay
    /// - **RequestedBlockPartsFound**: Add block parts response for specific requestor
    /// - **NewBlockAdded**: Relay newly validated block to network
    /// - **NewTransactionAdded**: Relay new transaction to network
    /// - **SendReplyTransaction**: Send transaction as reply to mempool request
    /// - **NewSupportAdded**: Relay support signature to network
    ///
    /// # Wait Pool Behavior
    /// - Messages from application get priority (no sender connections)
    /// - Sender connections used for received messages to score relay position
    /// - Specific requestor targeting for block part responses
    pub(crate) fn process_processing_result(&mut self, result: MessageProcessingResult) {
        match result {
            MessageProcessingResult::RequestedBlockNotFound(sequence) => {
                self.wait_pool.add_or_update_message(
                    RadioMessage::request_full_block_with(self.own_node_id, sequence),
                    &self.connection_matrix,
                    &self.connection_matrix[0],
                    &empty_connections(),
                    None,
                );
            }
            MessageProcessingResult::RequestedBlockFound(message) => {
                self.wait_pool
                    .add_or_update_message(message, &self.connection_matrix, &self.connection_matrix[0], &empty_connections(), None);
            }
            MessageProcessingResult::RequestedBlockPartsFound(message, requestor_node) => {
                let requestor_index = self.connection_matrix_nodes.iter().position(|&id| id == requestor_node).unwrap_or(0);
                if requestor_index > 0 {
                    self.wait_pool.add_or_update_message(
                        message,
                        &self.connection_matrix,
                        &self.connection_matrix[0],
                        &empty_connections(),
                        Some(requestor_index),
                    );
                }
            }
            MessageProcessingResult::NewBlockAdded(message) => {
                //find sender in nodes connection list
                //Linear search is fine here, as CONNECTION_MATRIX_SIZE is limited to small numbers (hardware limitation, because of memory size). We don't use more complex data structures for optimal performance.
                let sender_index = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id()).unwrap_or(0);
                let sender_connections = if sender_index > 0 {
                    self.connection_matrix[sender_index]
                } else {
                    empty_connections()
                };

                // Add the new block to the wait pool

                self.wait_pool
                    .add_or_update_message(message, &self.connection_matrix, &self.connection_matrix[0], &sender_connections, None);
            }
            MessageProcessingResult::NewTransactionAdded(message) => {
                //Linear search is fine here, as CONNECTION_MATRIX_SIZE is limited to small numbers (hardware limitation, because of memory size). We don't use more complex data structures for optimal performance.
                let sender_index = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id()).unwrap_or(0);
                let sender_connections = if sender_index > 0 {
                    self.connection_matrix[sender_index]
                } else {
                    empty_connections()
                };

                self.wait_pool
                    .add_or_update_message(message, &self.connection_matrix, &self.connection_matrix[0], &sender_connections, None);
            }
            MessageProcessingResult::SendReplyTransaction(message) => {
                self.wait_pool
                    .add_or_update_message(message, &self.connection_matrix, &self.connection_matrix[0], &empty_connections(), None);
            }
            MessageProcessingResult::NewSupportAdded(message) => {
                //Linear search is fine here, as CONNECTION_MATRIX_SIZE is limited to small numbers (hardware limitation, because of memory size). We don't use more complex data structures for optimal performance.
                let sender_index = self.connection_matrix_nodes.iter().position(|&id| id == message.sender_node_id()).unwrap_or(0);
                let sender_connections = if sender_index > 0 {
                    self.connection_matrix[sender_index]
                } else {
                    empty_connections()
                };

                self.wait_pool
                    .add_or_update_message(message, &self.connection_matrix, &self.connection_matrix[0], &sender_connections, None);
            }
            _ => { /* Ignore other results */ }
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use crate::{MessageProcessingResult, RadioMessage};

    const OWN_ID: u32 = 1;

    fn test_scoring_matrix() -> ScoringMatrix {
        // Simple matrix with uniform weights so scores are > 0
        ScoringMatrix::new([[1, 1, 1, 1]; 4], 16, 48, 0)
    }

    fn new_manager<const N: usize, const W: usize>() -> RelayManager<N, W> {
        RelayManager::with(1, 1, 1, 1, test_scoring_matrix(), OWN_ID, 42)
    }

    #[test]
    fn echo_request_increments_dirty_and_sends_reply() {
        const N: usize = 8;
        const W: usize = 4;
        let mut rm = new_manager::<N, W>();
        let sender_id = 2u32;
        let last_lq = 10u8;

        let req = RadioMessage::request_echo_with(sender_id);
        let res = rm.process_received_message(&req, last_lq);

        // With echo pooling, no immediate echo is sent
        match res {
            RelayResult::None => {}
            other => panic!("Expected None (pooled echo), got: {:?}", core::mem::discriminant(&other)),
        }

        // Sender should be inserted at index 1 (index 0 is own node)
        let sender_index = 1usize;
        // Dirty bit should be incremented and quality preserved (zero initially)
        assert_eq!(rm.connection_matrix[sender_index][0] & DIRTY_MASK, 1 << DIRTY_SHIFT);
        assert_eq!(rm.connection_matrix[sender_index][0] & QUALITY_MASK, 0);

        // An echo response should be queued in the wait pool
        let has_pooled_echo = rm
            .echo_responses_wait_pool
            .iter()
            .flatten()
            .any(|item| item.target_node_id == sender_id && item.link_quality == last_lq);
        assert!(has_pooled_echo, "expected an echo response to be queued");
    }

    #[test]
    fn echo_updates_matrix_for_known_target() {
        const N: usize = 8;
        const W: usize = 4;
        let mut rm = new_manager::<N, W>();
        // Manually register sender and target nodes
        let sender_id = 2u32;
        let target_id = 42u32;
        rm.connection_matrix_nodes[1] = sender_id;
        rm.connection_matrix_nodes[2] = target_id;
        rm.connection_matrix[1][1] = 63;
        rm.connection_matrix[2][2] = 63;
        rm.connected_nodes_count = 3;

        // Echo from sender -> target with link quality
        let link_q = 15u8;
        let echo = RadioMessage::echo_with(sender_id, target_id, link_q);
        let _ = rm.process_received_message(&echo, link_q);

        assert_eq!(rm.connection_matrix[1][2] & QUALITY_MASK, link_q);
    }

    #[test]
    fn echo_result_updates_both_directions() {
        const N: usize = 8;
        const W: usize = 4;
        let mut rm = new_manager::<N, W>();

        // Add sender at index 1
        let sender_id = 2u32;
        let _ = rm.process_received_message(&RadioMessage::request_echo_with(sender_id), 10);

        // Add neighbor at index 2
        let neighbor_id = 77u32;
        rm.connection_matrix_nodes[2] = neighbor_id;
        rm.connected_nodes_count = 3;

        // Build echo result with one item
        let mut er = RadioMessage::echo_result_with(sender_id);
        er.add_echo_result_item(neighbor_id, 20, 21).unwrap();
        let _ = rm.process_received_message(&er, 0);

        assert_eq!(rm.connection_matrix[1][2] & QUALITY_MASK, 20);
        assert_eq!(rm.connection_matrix[2][1] & QUALITY_MASK, 21);
    }

    #[test]
    fn duplicate_in_wait_pool_returns_already_have_message() {
        const N: usize = 8;
        const W: usize = 4;
        let mut rm = new_manager::<N, W>();

        let msg1 = RadioMessage::add_block_with(3, 5, &[9, 9, 9]);
        rm.process_processing_result(MessageProcessingResult::NewBlockAdded(msg1));

        // Receiving the same (logically equal) message should return AlreadyHaveMessage
        let msg2 = RadioMessage::add_block_with(3, 5, &[9, 9, 9]);
        let res = rm.process_received_message(&msg2, 0);
        match res {
            RelayResult::AlreadyHaveMessage => {}
            other => panic!("Expected AlreadyHaveMessage, got: {:?}", core::mem::discriminant(&other)),
        }
    }

    // ============================================================================
    // Connection Matrix Type Alias Tests
    // ============================================================================

    #[test]
    fn test_connection_matrix_initialization() {
        let matrix: ConnectionMatrix = [[0; crate::CONNECTION_MATRIX_SIZE]; crate::CONNECTION_MATRIX_SIZE];

        assert_eq!(matrix.len(), crate::CONNECTION_MATRIX_SIZE);
        assert_eq!(matrix[0].len(), crate::CONNECTION_MATRIX_SIZE);

        for row in matrix.iter() {
            for &cell in row.iter() {
                assert_eq!(cell, 0);
            }
        }
    }

    #[test]
    fn test_connection_matrix_row_type() {
        let matrix: ConnectionMatrix = [[42; crate::CONNECTION_MATRIX_SIZE]; crate::CONNECTION_MATRIX_SIZE];
        let row: &ConnectionMatrixRow = &matrix[0];

        assert_eq!(row.len(), crate::CONNECTION_MATRIX_SIZE);
        assert_eq!(row[0], 42);
        assert_eq!(row[crate::CONNECTION_MATRIX_SIZE - 1], 42);
    }

    #[test]
    fn test_connection_matrix_quality_and_dirty_bits() {
        let mut matrix: ConnectionMatrix = [[0; crate::CONNECTION_MATRIX_SIZE]; crate::CONNECTION_MATRIX_SIZE];

        const QUALITY_MASK: u8 = 0b0011_1111;
        const DIRTY_MASK: u8 = 0b1100_0000;
        const DIRTY_SHIFT: u8 = 6;

        let quality = 45u8;
        let dirty = 2u8;
        matrix[0][1] = (dirty << DIRTY_SHIFT) | quality;

        assert_eq!(matrix[0][1] & QUALITY_MASK, quality);
        assert_eq!((matrix[0][1] & DIRTY_MASK) >> DIRTY_SHIFT, dirty);
    }

    #[test]
    fn test_connection_matrix_full_size() {
        let matrix: ConnectionMatrix = [[255; crate::CONNECTION_MATRIX_SIZE]; crate::CONNECTION_MATRIX_SIZE];

        for i in 0..crate::CONNECTION_MATRIX_SIZE {
            for j in 0..crate::CONNECTION_MATRIX_SIZE {
                assert_eq!(matrix[i][j], 255);
            }
        }
    }
}

//! Wait Pool - Timed Message Relay Queue with Position-Based Scheduling
//!
//! The wait pool manages messages awaiting relay transmission, using network topology
//! information to intelligently schedule when each node should relay a message. This
//! prevents network flooding while ensuring good coverage.
//!
//! # Overview
//!
//! When a node receives a message that should be relayed, it's added to the wait pool
//! with a calculated activation time. The activation time depends on:
//! - The node's position in the network relative to other potential relays
//! - The quality of connections to other nodes
//! - A scoring matrix that prioritizes well-connected nodes
//!
//! # Relay Position Algorithm
//!
//! 1. Calculate a relay score based on network topology and connection quality
//! 2. Compare score against other nodes that have likely received the message
//! 3. Assign a position (0 = best relay candidate, higher = less optimal)
//! 4. Schedule activation time = position × delay + random jitter
//!
//! This approach naturally prioritizes well-connected nodes to relay first, while
//! allowing less-connected nodes to relay if better candidates don't respond.
//!
//! # Message Updates
//!
//! If additional packets for the same message arrive, the wait pool updates its
//! network knowledge and recalculates the relay position, potentially changing
//! the activation time.
//!
//! # Capacity Management
//!
//! When the pool is full, new messages replace existing ones only if they have
//! a higher relay score, ensuring the best relay candidates are prioritized.

use core::cmp::max;
use embassy_time::{Duration, Instant};
use rand_core::{RngCore, SeedableRng};
use rand_wyrand::WyRand;

use crate::relay_manager::{ConnectionMatrix, ConnectionMatrixRow};
use crate::{RadioMessage, RadioPacket, ScoringMatrix};

/// Bitmask to extract link quality from connection matrix cells
///
/// Connection matrix cells store link quality in the lower 6 bits (0-63 range).
/// Upper bits are reserved for flags.
const QUALITY_MASK: u8 = 0b0011_1111;

/// Categorizes a link quality value into one of four categories
///
/// Used by the scoring algorithm to map continuous quality values to discrete
/// categories for lookup in the scoring matrix.
///
/// # Arguments
///
/// * `value` - The connection matrix cell value (quality + flags)
/// * `poor_limit` - Threshold between Poor and Fair (typically 16)
/// * `excellent_limit` - Threshold between Fair and Excellent (typically 48)
///
/// # Returns
///
/// * `0` - Zero (no connection)
/// * `1` - Poor (0 < quality < poor_limit)
/// * `2` - Fair (poor_limit ≤ quality < excellent_limit)
/// * `3` - Excellent (quality ≥ excellent_limit)
fn calc_category(value: u8, poor_limit: u8, excellent_limit: u8) -> u8 {
    let quality = value & QUALITY_MASK;
    if quality == 0 {
        0
    } else if quality < poor_limit {
        1
    } else if quality < excellent_limit {
        2
    } else {
        3
    }
}

/// A message awaiting relay with its network topology information
///
/// Stores a message that should be relayed along with:
/// - When it should be transmitted (activation time)
/// - Network topology as known at the time (connection quality to all nodes)
/// - Optional requestor node (for targeted relay decisions)
///
/// The activation time is recalculated when new network information arrives,
/// allowing the node to adjust its relay priority dynamically.
pub(crate) struct WaitPoolItem<const CONNECTION_MATRIX_SIZE: usize> {
    /// The message to be relayed
    message: RadioMessage,

    /// When this message should be transmitted
    activation_time: Instant,

    /// Network topology snapshot: connection quality to each node as understood
    /// when the message was received
    message_connections: ConnectionMatrixRow,

    /// Index of the node that requested this message (if any)
    ///
    /// Used for messages like RequestBlockPart where only the requestor
    /// should receive the relay, not the entire network.
    requestor_index: Option<usize>,
}

#[allow(dead_code)]
const _ASSERT_CONNECTION_MATRIX_SQUARE_FITS_U32: () = assert!(
    crate::CONNECTION_MATRIX_SIZE * 255 <= u32::MAX as usize,
    "CONNECTION_MATRIX_SIZE*MAX score size must fit in u32"
);

impl<const CONNECTION_MATRIX_SIZE: usize> WaitPoolItem<CONNECTION_MATRIX_SIZE> {
    /// Calculates the relay score for this message
    ///
    /// The score represents how suitable this node is to relay the message,
    /// based on connection quality and network topology. Higher scores indicate
    /// better relay candidates.
    ///
    /// # Scoring Logic
    ///
    /// If there's a specific requestor, only that connection is scored.
    /// Otherwise, all connections are scored and summed.
    ///
    /// For each connection:
    /// 1. Categorize the message sender's connection quality to that node
    /// 2. Categorize this node's connection quality to that node
    /// 3. Look up the score in the scoring matrix [sender_category][own_category]
    ///
    /// # Arguments
    ///
    /// * `own_connections` - This node's connection quality to all nodes
    /// * `scoring_matrix` - Configuration defining category thresholds and scores
    ///
    /// # Returns
    ///
    /// Total relay score (higher = better relay candidate)
    pub(crate) fn calculate_score(&self, own_connections: &ConnectionMatrixRow, scoring_matrix: &ScoringMatrix) -> u32 {
        let mut score: u32 = 0;
        if let Some(requestor_index) = self.requestor_index {
            let network_category = calc_category(
                self.message_connections[requestor_index],
                scoring_matrix.poor_limit,
                scoring_matrix.excellent_limit,
            );
            let own_category = calc_category(own_connections[requestor_index], scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
            //because of the limited number of nodes, the score won't overflow u32
            score += scoring_matrix.matrix[network_category as usize][own_category as usize] as u32;
        } else {
            for (network_quality, own_quality) in self.message_connections.iter().zip(own_connections.iter()).take(CONNECTION_MATRIX_SIZE) {
                let network_category = calc_category(*network_quality, scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
                let own_category = calc_category(*own_quality, scoring_matrix.poor_limit, scoring_matrix.excellent_limit);
                score += scoring_matrix.matrix[network_category as usize][own_category as usize] as u32;
            }
        }
        score
    }

    /// Calculates this node's position among potential relays
    ///
    /// Determines how many other nodes in the network are better relay candidates
    /// for this message. This position is used to calculate the relay delay:
    /// nodes with better positions relay sooner.
    ///
    /// # Algorithm
    ///
    /// For each node in the network (except ourselves):
    /// 1. Check if that node likely received the message (connection quality > poor_limit)
    /// 2. Calculate that node's relay score for this message
    /// 3. If their score is higher than ours, increment our position
    ///
    /// # Arguments
    ///
    /// * `item_connections` - Message sender's connection quality to all nodes
    /// * `own_connections` - This node's connection quality to all nodes
    /// * `connection_matrix` - Full network topology (all nodes' connections)
    /// * `scoring_matrix` - Configuration for relay scoring
    ///
    /// # Returns
    ///
    /// Position in relay queue (0 = best candidate, higher = worse candidate)
    fn calculate_own_position(
        &self,
        item_connections: &ConnectionMatrixRow,
        own_connections: &ConnectionMatrixRow,
        connection_matrix: &ConnectionMatrix,
        scoring_matrix: &ScoringMatrix,
    ) -> u64 {
        let own_score = self.calculate_score(own_connections, scoring_matrix);
        let mut own_position = 0;
        for i in 1..CONNECTION_MATRIX_SIZE {
            if item_connections[i] > scoring_matrix.poor_limit {
                let score = self.calculate_score(&connection_matrix[i], scoring_matrix);
                if score > own_score {
                    own_position += 1;
                }
            }
        }

        own_position
    }
}

/// Timed queue for messages awaiting relay transmission
///
/// Manages a fixed-size pool of messages that should be relayed to the network.
/// Each message has an activation time calculated based on network topology,
/// ensuring well-connected nodes relay first while preventing network floods.
///
/// # Type Parameters
///
/// * `WAIT_POOL_SIZE` - Maximum number of messages that can be queued
/// * `CONNECTION_MATRIX_SIZE` - Size of the network (max nodes)
///
/// # Relay Scheduling
///
/// Messages are scheduled based on:
/// 1. Network topology analysis (who else likely received the message)
/// 2. Relay score calculation (how well-connected this node is)
/// 3. Position-based delay (better candidates relay sooner)
/// 4. Random jitter (prevents simultaneous transmissions)
///
/// # Dynamic Updates
///
/// When additional packets arrive for messages already in the pool, the network
/// topology information is updated and activation times are recalculated.
pub(crate) struct WaitPool<const WAIT_POOL_SIZE: usize, const CONNECTION_MATRIX_SIZE: usize> {
    /// Array of optional messages awaiting relay
    items: [Option<WaitPoolItem<CONNECTION_MATRIX_SIZE>>; WAIT_POOL_SIZE],

    /// Base delay per position in relay queue (seconds)
    relay_position_delay: u64,

    /// Configuration for relay score calculation
    scoring_matrix: ScoringMatrix,

    /// Random number generator for jitter
    rng: WyRand,
}

impl<const WAIT_POOL_SIZE: usize, const CONNECTION_MATRIX_SIZE: usize> WaitPool<WAIT_POOL_SIZE, CONNECTION_MATRIX_SIZE> {
    /// Creates a new empty wait pool
    ///
    /// # Arguments
    ///
    /// * `relay_position_delay` - Seconds of delay per position in relay queue
    /// * `scoring_matrix` - Configuration for relay scoring algorithm
    /// * `rng_seed` - Seed for random number generation (for jitter)
    ///
    /// # Returns
    ///
    /// A new wait pool with no messages
    pub fn with(relay_position_delay: u64, scoring_matrix: ScoringMatrix, rng_seed: u64) -> Self {
        Self {
            items: [const { None }; WAIT_POOL_SIZE],
            relay_position_delay,
            scoring_matrix,
            rng: WyRand::seed_from_u64(rng_seed),
        }
    }

    /// Checks if the pool contains a message or its reply
    ///
    /// Used to detect if a message is already queued for relay, preventing
    /// duplicate entries. Also checks for reply messages (e.g., if we're
    /// waiting to relay a RequestFullBlock, an AddBlock reply counts as a match).
    ///
    /// # Arguments
    ///
    /// * `message` - The message to search for
    ///
    /// # Returns
    ///
    /// `true` if the message or its reply is in the pool, `false` otherwise
    pub(crate) fn contains_message_or_reply(&self, message: &RadioMessage) -> bool {
        self.items
            .iter()
            .any(|item| item.as_ref().is_some_and(|i| &i.message == message || i.message.is_reply_to(message)))
    }

    /// Updates an existing message's network topology and recalculates activation time
    ///
    /// When additional packets arrive for a message already in the pool, this method
    /// updates the network knowledge and recalculates the relay position and timing.
    ///
    /// # Network Knowledge Update
    ///
    /// For each node, takes the maximum connection quality seen so far (from either
    /// the original packet or this new packet). This builds a progressively more
    /// complete picture of network topology.
    ///
    /// # Score-Based Removal
    ///
    /// If the updated score falls below the relay threshold, the message is removed
    /// from the pool (another nodes with better position already relayaed the message).
    ///
    /// # Arguments
    ///
    /// * `message` - The message to update
    /// * `connection_matrix` - Full network topology
    /// * `own_connections` - This node's connection quality to all nodes
    /// * `sender_connections` - New sender's connection quality to all nodes
    ///
    /// # Returns
    ///
    /// `true` if the message was found and updated, `false` if not found
    pub(crate) fn update_message(
        &mut self,
        message: &RadioMessage,
        connection_matrix: &ConnectionMatrix,
        own_connections: &ConnectionMatrixRow,
        sender_connections: &ConnectionMatrixRow,
    ) -> bool {
        for item_opt in self.items.iter_mut() {
            if let Some(item) = item_opt {
                if item.message == *message {
                    //log!(log::Level::Debug, "[{:?}] updating(add or) waitpool item", self.own_node_id);
                    for (message_conn, sender_conn) in item.message_connections.iter_mut().zip(sender_connections.iter()).take(CONNECTION_MATRIX_SIZE) {
                        *message_conn = max(*message_conn, sender_conn & QUALITY_MASK);
                    }

                    if item.calculate_score(own_connections, &self.scoring_matrix) < self.scoring_matrix.relay_score_limit as u32 {
                        //  log!(log::Level::Debug, "Message removed from wait pool");
                        *item_opt = None;
                        return true;
                    }
                    let position = item.calculate_own_position(&item.message_connections, own_connections, connection_matrix, &self.scoring_matrix);

                    item.activation_time = Instant::now()
                        + Duration::from_secs(position * self.relay_position_delay)
                        + Duration::from_millis(self.rng.next_u64() % (self.relay_position_delay * 1000 / 2));
                    return true;
                }
            }
        }
        false
    }

    /// Adds a new message to the pool or updates it if already present
    ///
    /// This is the primary method for adding messages to the wait pool. It handles:
    /// 1. Updating existing messages (if already in pool)
    /// 2. Creating new entries with calculated activation times
    /// 3. Capacity management (replacing low-score messages when full)
    ///
    /// # Relay Score Filtering
    ///
    /// Messages with scores below `relay_score_limit` are not added, as they
    /// indicate other nodes are much better positioned to relay.
    ///
    /// # Activation Time Calculation
    ///
    /// ```text
    /// activation_time = now + (position × relay_position_delay) + random_jitter
    /// ```
    ///
    /// Where:
    /// - `position` = number of nodes better positioned to relay
    /// - `relay_position_delay` = seconds per position (typically 1-3 seconds)
    /// - `random_jitter` = 0 to relay_position_delay seconds (for new messages)
    ///
    /// # Capacity Management
    ///
    /// When the pool is full:
    /// 1. Find the message with the lowest relay score
    /// 2. If the new message has a higher score, replace the lowest
    /// 3. Otherwise, discard the new message
    ///
    /// # Arguments
    ///
    /// * `message` - The message to add or update
    /// * `connection_matrix` - Full network topology
    /// * `own_connections` - This node's connection quality to all nodes
    /// * `sender_connections` - Message sender's connection quality to all nodes
    /// * `requestor_index` - Optional index of the requesting node (for targeted relays)
    pub(crate) fn add_or_update_message(
        &mut self,
        message: RadioMessage,
        connection_matrix: &ConnectionMatrix,
        own_connections: &ConnectionMatrixRow,
        sender_connections: &ConnectionMatrixRow,
        requestor_index: Option<usize>,
    ) {
        //first update existing item if present
        if self.update_message(&message, connection_matrix, own_connections, sender_connections) {
            return;
        }

        //create a new item
        let mut new_item = WaitPoolItem {
            message,
            activation_time: Instant::now(),
            message_connections: [0; crate::CONNECTION_MATRIX_SIZE],
            requestor_index,
        };

        for (message_conn, sender_conn) in new_item
            .message_connections
            .iter_mut()
            .zip(sender_connections.iter())
            .take(CONNECTION_MATRIX_SIZE)
        {
            *message_conn = sender_conn & QUALITY_MASK;
        }

        if new_item.calculate_score(own_connections, &self.scoring_matrix) < self.scoring_matrix.relay_score_limit as u32 {
            return;
        }

        let position = new_item.calculate_own_position(&new_item.message_connections, own_connections, connection_matrix, &self.scoring_matrix);

        new_item.activation_time = Instant::now()
            + Duration::from_secs(position * self.relay_position_delay)
            + Duration::from_millis(self.rng.next_u64() % (self.relay_position_delay * 1000));

        //find an empty spot in the waitpool

        for item_opt in self.items.iter_mut() {
            if item_opt.is_none() {
                *item_opt = Some(new_item);
                return;
            }
        }

        //no empty spot, find the item with the lowest score and replace if the new item has a higher score
        let new_score = new_item.calculate_score(own_connections, &self.scoring_matrix);
        let mut min_score = new_score;
        let mut min_index: usize = WAIT_POOL_SIZE; // Invalid index
        for (i, item_opt) in self.items.iter().enumerate() {
            if let Some(item) = item_opt {
                let score = item.calculate_score(own_connections, &self.scoring_matrix);
                if score < min_score {
                    min_score = score;
                    min_index = i;
                }
            }
        }

        if min_index < WAIT_POOL_SIZE {
            self.items[min_index] = Some(new_item);
        }
    }

    /// Updates network topology for messages based on a received packet
    ///
    /// When a packet arrives, it may match messages already in the wait pool
    /// (e.g., another packet from the same multi-packet message). This method
    /// updates the network knowledge for matching messages.
    ///
    /// # Matching Criteria
    ///
    /// A packet matches a wait pool item if:
    /// 1. The item has a requestor (is a targeted relay)
    /// 2. Message type matches
    /// 3. Sequence number matches (if both have sequence numbers)
    /// 4. Payload checksum matches (if both have checksums)
    ///
    /// # Update Behavior
    ///
    /// Same as `update_message`: updates network topology, recalculates score,
    /// removes if score too low, recalculates activation time.
    ///
    /// # Arguments
    ///
    /// * `packet` - The received packet
    /// * `connection_matrix` - Full network topology
    /// * `own_connections` - This node's connection quality to all nodes
    /// * `sender_connections` - Packet sender's connection quality to all nodes
    pub(crate) fn update_with_received_packet(
        &mut self,
        packet: &RadioPacket,
        connection_matrix: &ConnectionMatrix,
        own_connections: &ConnectionMatrixRow,
        sender_connections: &ConnectionMatrixRow,
    ) {
        if let (Some(packet_sequence), Some(packet_payload_checksum)) = (packet.sequence(), packet.payload_checksum()) {
            for item_opt in self.items.iter_mut() {
                if let Some(item) = item_opt {
                    if item.requestor_index.is_some() && item.message.message_type() == packet.message_type() {
                        if let (Some(message_sequence), Some(message_payload_checksum)) = (item.message.sequence(), item.message.payload_checksum()) {
                            if message_sequence == packet_sequence && message_payload_checksum == packet_payload_checksum {
                                for (message_conn, sender_conn) in
                                    item.message_connections.iter_mut().zip(sender_connections.iter()).take(CONNECTION_MATRIX_SIZE)
                                {
                                    *message_conn = max(*message_conn, sender_conn & QUALITY_MASK);
                                }

                                if item.calculate_score(own_connections, &self.scoring_matrix) < self.scoring_matrix.relay_score_limit as u32 {
                                    //  log!(log::Level::Debug, "Message removed from wait pool");
                                    *item_opt = None;
                                    return;
                                }
                                let position = item.calculate_own_position(&item.message_connections, own_connections, connection_matrix, &self.scoring_matrix);

                                item.activation_time = Instant::now()
                                    + Duration::from_secs(position * self.relay_position_delay)
                                    + Duration::from_millis(self.rng.next_u64() % (self.relay_position_delay * 1000 / 2));
                                return;
                            }
                        }
                    }
                }
            }
        }
    }

    /// Returns the earliest activation time among all pooled messages
    ///
    /// Used to determine when to wake up and check for messages ready to relay.
    ///
    /// # Returns
    ///
    /// - `Some(Instant)` - Time of the next message to activate
    /// - `None` - Pool is empty
    pub(crate) fn next_activation_time(&self) -> Option<Instant> {
        self.items.iter().filter_map(|item| item.as_ref()).map(|item| item.activation_time).min()
    }

    /// Retrieves and removes the next message that's ready for relay
    ///
    /// Checks if any message's activation time has been reached. If so, removes
    /// the first ready message from the pool and returns it for transmission.
    ///
    /// # Behavior
    ///
    /// - Only returns a message if its activation time ≤ current time
    /// - Returns the first ready message found (no prioritization among ready messages)
    /// - Removes the returned message from the pool
    /// - Other ready messages will be returned on subsequent calls
    ///
    /// # Returns
    ///
    /// - `Some(RadioMessage)` - A message ready for relay transmission
    /// - `None` - No messages are ready yet, or pool is empty
    pub(crate) fn get_next_activated_message(&mut self) -> Option<RadioMessage> {
        if let Some(next_activation_time) = self.next_activation_time() {
            if Instant::now() >= next_activation_time {
                // Process the first item that is ready, other items will be processed in the next iteration (no prioritization)
                for item_opt in self.items.iter_mut() {
                    if let Some(item) = item_opt {
                        if item.activation_time <= Instant::now() {
                            // Extract the message by taking it out and replacing with None
                            if let Some(item) = core::mem::take(item_opt) {
                                let message = item.message;
                                return Some(message);
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Updates the pool when a connection matrix entry changes
    ///
    /// When a node's connection status changes (e.g., connection lost, quality changed),
    /// this method updates all affected wait pool items by clearing the connection
    /// quality for that node.
    ///
    /// # Special Handling for Requestor-Targeted Messages
    ///
    /// If a wait pool item has a requestor (targeted relay), and that requestor's
    /// connection is the one that changed, the entire item is removed. This is because
    /// targeted relays are only meaningful if we can reach the requestor.
    ///
    /// # Arguments
    ///
    /// * `index` - Index of the node whose connection changed
    pub(crate) fn connection_matrix_item_changed(&mut self, index: usize) {
        for item_opt in self.items.iter_mut() {
            if let Some(item) = item_opt {
                if let Some(requestor_idx) = item.requestor_index {
                    if requestor_idx == index {
                        // If the requestor's connection is dropped, remove the item
                        *item_opt = None;
                        continue;
                    }
                }
                item.message_connections[index] = 0;
            }
        }
    }
}

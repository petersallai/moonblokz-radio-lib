//! # RX Handler - Reception and Packet Assembly
//!
//! This module implements the receive-side message processing pipeline for the radio
//! communication system. It handles incoming packets, assembles multi-packet messages,
//! deduplicates received messages, and coordinates message processing with relay decisions.
//!
//! ## Architecture
//!
//! The RX handler operates as an async task that:
//! - Receives raw packets from the radio device through a channel
//! - Buffers and assembles multi-packet messages
//! - Maintains a duplicate detection cache
//! - Processes complete messages for local handling or relay
//! - Integrates with the RelayManager for forwarding decisions
//! - Coordinates with WaitPool for request-response matching
//!
//! ## Key Components
//!
//! - **PacketBuffer**: Stores incoming packets with timestamps for multi-packet message assembly
//! - **LastReceivedMessages**: Circular buffer cache to prevent duplicate message processing
//! - **RelayManager Integration**: Coordinates topology-aware message forwarding
//! - **Message Assembly**: Reconstructs RadioMessages from fragmented RadioPackets
//!
//! ## Processing Flow
//!
//! 1. Receive packet from radio device
//! 2. Update RX state to prevent transmission conflicts
//! 3. Check for duplicate packets/messages
//! 4. Buffer single-packet messages or assemble multi-packet messages
//! 5. Process complete messages (local handling or relay decision)
//! 6. Forward to appropriate queues based on message type and relay decision

// Embassy executor task macro generates code that needs many parameters for flexibility
#![allow(clippy::too_many_arguments)]
//!
//! ## Design Considerations
//!
//! - Duplicate detection uses sequence numbers and payload checksums
//! - Multi-packet assembly has configurable buffer size and timeout
//! - RX state signaling prevents transmit/receive collisions
//! - Integration with WaitPool enables request-response pattern matching

use crate::relay_manager::RelayResult;
use crate::{
    IncomingMessageItem, MessageProcessingResult, MessageType, RxState, ScoringMatrix, CONNECTION_MATRIX_SIZE, INCOMING_PACKET_BUFFER_SIZE,
    LAST_RECEIVED_MESSAGE_BUFFER_SIZE, MAX_NODE_COUNT, RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE, WAIT_POOL_SIZE,
};
use embassy_futures::select::{select4, Either4};
use embassy_sync::channel::TrySendError;
use embassy_time::{Instant, Timer};
use log::{log, Level};
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

use crate::{
    relay_manager::RelayManager, IncomingMessageQueueSender, OutgoingMessageQueueSender, ProcessResultQueueReceiver, RadioMessage, RadioPacket,
    RxPacketQueueReceiver, RxStateQueueSender,
};

/// Buffered packet with arrival timestamp for assembly tracking
///
/// Used to store packets temporarily while multi-packet messages are being
/// assembled. The arrival time helps detect and clean up stale packets.
struct PacketBufferItem {
    /// The received radio packet
    packet: RadioPacket,

    /// Time when this packet was received
    arrival_time: Instant,
}

/// Sentinel value indicating an empty slot in the packet check buffer
const PACKET_CHECK_BUFFER_EMPTY_VALUE: u8 = 255;

/// Record of a received message for duplicate detection
///
/// Stores minimal identifying information about a processed message to
/// enable fast duplicate checking without storing entire payloads.
struct LastReceivedMessage {
    /// Message type byte
    message_type: u8,

    /// Message sequence number
    sequence: u32,

    /// CRC32C checksum of the payload
    payload_checksum: u32,
}

/// Circular buffer cache of recently received messages for duplicate detection
///
/// Maintains a fixed-size history of processed messages to quickly identify
/// and reject duplicates. Uses a circular buffer to automatically age out
/// old entries.
struct LastReceivedMessages {
    /// Fixed-size array of optional message records
    messages: [Option<LastReceivedMessage>; LAST_RECEIVED_MESSAGE_BUFFER_SIZE],

    /// Current insertion position in the circular buffer
    circular_index: usize,
}

impl LastReceivedMessages {
    /// Creates a new empty duplicate detection cache
    const fn new() -> Self {
        Self {
            messages: [const { None }; LAST_RECEIVED_MESSAGE_BUFFER_SIZE],
            circular_index: 0,
        }
    }

    /// Adds a received message to the duplicate detection cache
    ///
    /// Extracts identifying information from the message and stores it.
    /// Only messages with sequence numbers and checksums are tracked.
    ///
    /// # Arguments
    /// * `message` - The received message to record
    fn add_message(&mut self, message: &RadioMessage) {
        if let (Some(sequence), Some(payload_checksum)) = (message.sequence(), message.payload_checksum()) {
            self.add_message_with_parameters(message.message_type(), sequence, payload_checksum);
        }
    }

    /// Adds a message record with explicit parameters
    ///
    /// Inserts a new entry into the circular buffer, overwriting the oldest
    /// entry when the buffer is full.
    ///
    /// # Arguments
    /// * `message_type` - Type of the message
    /// * `sequence` - Sequence number
    /// * `payload_checksum` - CRC32C checksum of payload
    fn add_message_with_parameters(&mut self, message_type: u8, sequence: u32, payload_checksum: u32) {
        self.messages[self.circular_index] = Some(LastReceivedMessage {
            message_type,
            sequence,
            payload_checksum,
        });
        self.circular_index = (self.circular_index + 1) % LAST_RECEIVED_MESSAGE_BUFFER_SIZE;
    }

    /// Checks if a packet matches any recently received message
    ///
    /// Used for duplicate detection. Compares the packet's identifying
    /// information against the cache of recently processed messages.
    ///
    /// # Arguments
    /// * `packet` - The packet to check
    ///
    /// # Returns
    /// true if this packet/message was already received, false otherwise
    fn contains_message(&self, packet: &RadioPacket) -> bool {
        if let (Some(sequence), Some(payload_checksum)) = (packet.sequence(), packet.payload_checksum()) {
            //perform linear search - the buffer is small so this is efficient enough
            for entry in self.messages.iter().flatten() {
                if entry.message_type == packet.message_type() && entry.sequence == sequence && entry.payload_checksum == payload_checksum {
                    return true;
                }
            }
        }
        false
    }
}

/// RX Handler Task - Packet Reception and Message Assembly
///
/// This async task processes received packets, assembles multi-packet messages,
/// detects duplicates, and coordinates with the relay manager for forwarding decisions.
///
/// # Responsibilities
///
/// 1. **Packet Reception**: Receives packets from the radio device queue
/// 2. **Duplicate Detection**: Checks packets against recently received cache
/// 3. **Message Assembly**: Reconstructs multi-packet messages from individual packets
/// 4. **Processing Coordination**: Sends messages to application for processing
/// 5. **Relay Management**: Works with RelayManager for network topology and forwarding
/// 6. **Echo Protocol**: Handles echo requests, responses, and result aggregation
///
/// # Parameters
///
/// * `incoming_message_queue_sender` - Sends assembled messages to application
/// * `outgoing_message_queue_sender` - Queues messages for relay/transmission
/// * `rx_packet_queue_receiver` - Receives packets from radio device
/// * `rx_state_queue_sender` - Signals RX state to TX scheduler
/// * `process_result_queue_receiver` - Receives processing results from application
/// * `echo_request_minimal_interval` - Minimum time between echo requests (seconds)
/// * `echo_messages_target_interval` - Target number of messages between echoes
/// * `echo_gathering_timeout` - Time to collect echo responses (seconds)
/// * `relay_position_delay` - Base delay for relay position calculation (seconds)
/// * `scoring_matrix` - Configuration for relay scoring decisions
/// * `retry_interval_for_missing_packets` - Retry interval for incomplete messages (seconds)
/// * `own_node_id` - This node's unique network identifier
/// * `rng_seed` - Seed for random number generation
#[embassy_executor::task(pool_size = MAX_NODE_COUNT)]
pub(crate) async fn rx_handler_task(
    incoming_message_queue_sender: IncomingMessageQueueSender,
    outgoing_message_queue_sender: OutgoingMessageQueueSender,
    rx_packet_queue_receiver: RxPacketQueueReceiver,
    rx_state_queue_sender: RxStateQueueSender,
    process_result_queue_receiver: ProcessResultQueueReceiver,
    echo_request_minimal_interval: u32,
    echo_messages_target_interval: u8,
    echo_gathering_timeout: u8,
    relay_position_delay: u8,
    scoring_matrix: ScoringMatrix,
    retry_interval_for_missing_packets: u8,
    own_node_id: u32,
    rng_seed: u64,
) -> ! {
    let mut packet_buffer: [Option<PacketBufferItem>; INCOMING_PACKET_BUFFER_SIZE] = [const { None }; INCOMING_PACKET_BUFFER_SIZE];
    let mut packet_check_buffer: [u8; INCOMING_PACKET_BUFFER_SIZE] = [PACKET_CHECK_BUFFER_EMPTY_VALUE; INCOMING_PACKET_BUFFER_SIZE];
    let mut last_received_messages = LastReceivedMessages::new();

    let mut rng = WyRand::seed_from_u64(rng_seed);
    let mut relay_manager = RelayManager::<CONNECTION_MATRIX_SIZE, WAIT_POOL_SIZE>::with(
        echo_request_minimal_interval,
        echo_messages_target_interval,
        echo_gathering_timeout,
        relay_position_delay,
        scoring_matrix,
        own_node_id,
        rng.next_u64(),
    );

    let mut next_missing_packet_check = Instant::now() + embassy_time::Duration::from_secs(retry_interval_for_missing_packets as u64);
    log!(log::Level::Info, "[{}] RX handler task started", own_node_id);
    loop {
        match select4(
            rx_packet_queue_receiver.receive(),
            process_result_queue_receiver.receive(),
            Timer::at(relay_manager.calculate_next_timeout()),
            Timer::at(next_missing_packet_check),
        )
        .await
        {
            Either4::First(received_packet) => {
                let rx_packet = received_packet.packet;
                if rx_packet.total_packet_count() == 1 {
                    log::trace!(
                        "[{}] Received single-packet message: type: {}, sender: {}, length: {}",
                        own_node_id,
                        rx_packet.message_type(),
                        rx_packet.sender_node_id(),
                        rx_packet.length
                    );
                    let radio_message = RadioMessage::from_single_packet(rx_packet);
                    process_message(
                        radio_message,
                        received_packet.link_quality,
                        outgoing_message_queue_sender,
                        incoming_message_queue_sender,
                        &mut relay_manager,
                        own_node_id,
                    );
                } else if rx_packet.total_packet_count() == 0 {
                    log!(Level::Warn, "[{}] Received empty packet, skipping", own_node_id);
                    continue;
                } else {
                    // Handle multi-packet message
                    log::trace!(
                        "[{}] Received multi-packet message: type: {}, sender: {}, length: {}, packet {}/{}",
                        own_node_id,
                        rx_packet.message_type(),
                        rx_packet.sender_node_id(),
                        rx_packet.length,
                        rx_packet.packet_index() + 1,
                        rx_packet.total_packet_count()
                    );
                    let packet_index = rx_packet.packet_index() as usize;
                    let total_packet_count = rx_packet.total_packet_count() as usize;

                    if total_packet_count > INCOMING_PACKET_BUFFER_SIZE {
                        log!(
                            Level::Warn,
                            "[{}] Total packet count {} exceeds buffer capacity {}. Dropping message.",
                            own_node_id,
                            total_packet_count,
                            INCOMING_PACKET_BUFFER_SIZE
                        );
                        continue;
                    }

                    if packet_index == total_packet_count - 1 {
                        if rx_state_queue_sender.try_send(RxState::PacketedRxEnded).is_err() {
                            log::warn!("[{}] Failed to send PacketedRxEnded to rx_state_queue. The queue is full.", own_node_id);
                        }
                    } else if rx_state_queue_sender
                        .try_send(RxState::PacketedRxInProgress(packet_index as u8, total_packet_count as u8))
                        .is_err()
                    {
                        log::warn!("[{}] Failed to send PacketedRxInProgress to rx_state_queue. The queue is full.", own_node_id);
                    }

                    if packet_index >= total_packet_count {
                        if let Some(sequence) = rx_packet.sequence() {
                            log!(
                                Level::Warn,
                                "[{}] Packet index out of bounds: sender: {}, sequence: {}, index: {}/{}",
                                own_node_id,
                                rx_packet.sender_node_id(),
                                sequence,
                                packet_index + 1,
                                total_packet_count
                            );
                        } else {
                            log!(
                                Level::Warn,
                                "[{}] Packet index out of bounds: sender: {}, index: {}/{}",
                                own_node_id,
                                rx_packet.sender_node_id(),
                                packet_index + 1,
                                total_packet_count
                            );
                        }
                        continue;
                    }

                    relay_manager.process_received_packet(&rx_packet, received_packet.link_quality);

                    let mut already_received = false;
                    let mut first_from_message = true;
                    let mut empty_index = PACKET_CHECK_BUFFER_EMPTY_VALUE;
                    // Header bytes are the first RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE bytes
                    let mut packet_header: [u8; RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE] = [0u8; RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE];
                    // Copy first RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE bytes
                    packet_header.copy_from_slice(&rx_packet.data[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]);

                    for (i, packet) in packet_buffer.iter().enumerate().take(INCOMING_PACKET_BUFFER_SIZE) {
                        if let Some(packet) = packet {
                            if packet.packet.same_message(&packet_header) {
                                if packet.packet.packet_index() == packet_index as u8 {
                                    // This packet is already in the buffer, skip it
                                    already_received = true;
                                    break;
                                }
                                //we found an other packet from the same message
                                first_from_message = false;
                            }
                        } else {
                            // Found an empty slot in the buffer
                            if empty_index == PACKET_CHECK_BUFFER_EMPTY_VALUE {
                                empty_index = i as u8;
                            }
                        }
                    }

                    //if we already have this packet we can skip it
                    if already_received {
                        log::trace!("[{}] Packet already received, skipping.", own_node_id);
                        continue;
                    }

                    if first_from_message {
                        log::trace!("[{}] First packet from new message received.", own_node_id);
                        if last_received_messages.contains_message(&rx_packet) {
                            log!(
                                Level::Trace,
                                "[{}] Already have full message in last received messages cache, skipping.",
                                own_node_id
                            );
                            //if we already have the full message (in the last received messages list), we can skip it.
                            continue;
                        } else {
                            //we inititate a check to find if we have the content of this message
                            if let (Some(sequence), Some(payload_checksum)) = (rx_packet.sequence(), rx_packet.payload_checksum()) {
                                if incoming_message_queue_sender
                                    .try_send(IncomingMessageItem::CheckIfAlreadyHaveMessage(
                                        rx_packet.message_type(),
                                        sequence,
                                        payload_checksum,
                                    ))
                                    .is_err()
                                {
                                    log!(
                                        Level::Warn,
                                        "[{}] Failed to send CheckIfAlreadyHaveMessage to incoming_message_queue. The queue is full. Dropping check for message: messagetype: {}, sequence: {}, payload_checksum: {}",
                                        own_node_id,
                                        rx_packet.message_type(),
                                        sequence,
                                        payload_checksum,
                                    );
                                }
                            }
                        }
                    }

                    //if we can't find an empty slot, we delete the message with the oldest last packet
                    if empty_index == PACKET_CHECK_BUFFER_EMPTY_VALUE {
                        log::trace!("[{}] No empty slot found, removing oldest packet.", own_node_id);
                        let mut oldest_index: u8 = 0;
                        let mut oldest_time = Instant::now() + embassy_time::Duration::from_secs(60);
                        // Track oldest message by its first RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE bytes
                        let mut oldest_packet_header: [u8; RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE] = [0u8; RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE];

                        for (i, packet) in packet_buffer.iter().enumerate().take(INCOMING_PACKET_BUFFER_SIZE) {
                            if let Some(packet) = packet {
                                if packet.arrival_time < oldest_time {
                                    oldest_time = packet.arrival_time;
                                    oldest_index = i as u8;
                                    // Copy first RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE bytes
                                    oldest_packet_header.copy_from_slice(&packet.packet.data[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]);
                                }
                            }
                        }

                        // Remove all packets for the oldest message
                        for packet in packet_buffer.iter_mut().take(INCOMING_PACKET_BUFFER_SIZE) {
                            if let Some(p) = packet {
                                // Compare first RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE bytes
                                if p.packet.data[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE] == oldest_packet_header {
                                    *packet = None;
                                }
                            }
                        }

                        empty_index = oldest_index;
                    }

                    let last_sender = rx_packet.sender_node_id();

                    // Store the packet in the buffer
                    packet_buffer[empty_index as usize] = Some(PacketBufferItem {
                        packet: rx_packet,
                        arrival_time: Instant::now(),
                    });

                    // Check if we have all packets for this message
                    packet_check_buffer.fill(PACKET_CHECK_BUFFER_EMPTY_VALUE);
                    for (i, packet_item) in packet_buffer.iter().enumerate().take(INCOMING_PACKET_BUFFER_SIZE) {
                        if let Some(packet_item) = packet_item {
                            if packet_item.packet.same_message(&packet_header) {
                                // Mark this packet as received
                                packet_check_buffer[packet_item.packet.packet_index() as usize] = i as u8;
                            }
                        }
                    }

                    let mut all_packets_received = true;

                    // Check if all packets for this message have been received
                    for packet_index in packet_check_buffer.iter().take(total_packet_count) {
                        if *packet_index == PACKET_CHECK_BUFFER_EMPTY_VALUE {
                            // Not all packets received, break out of the loop
                            all_packets_received = false;
                            break;
                        }
                    }

                    if all_packets_received {
                        log!(Level::Trace, "[{}] All packets received for message", own_node_id);
                        // All packets received, create a RadioMessage
                        let mut radio_message = RadioMessage::new();
                        let mut assembly_failed = false;

                        // Check if all packets for this message have been received
                        for packet_index in packet_check_buffer.iter().take(total_packet_count) {
                            if let Some(packet_item) = &packet_buffer[*packet_index as usize] {
                                // Add the packet to the RadioMessage
                                if let Err(_) = radio_message.add_packet(&packet_item.packet) {
                                    log!(Level::Error, "[{}] Failed to add packet index {} to message", own_node_id, packet_index);
                                    assembly_failed = true;
                                    break;
                                }
                                packet_buffer[*packet_index as usize] = None; // Clear the packet from the buffer
                            } else {
                                log!(Level::Error, "[{}] Packet index {} not found in buffer", own_node_id, packet_index);
                                assembly_failed = true;
                                break;
                            }
                        }

                        if assembly_failed {
                            // Clean up remaining packets from this message
                            for packet in packet_buffer.iter_mut().take(INCOMING_PACKET_BUFFER_SIZE) {
                                if let Some(p) = packet {
                                    if p.packet.same_message(&packet_header) {
                                        *packet = None;
                                    }
                                }
                            }
                            continue; // Skip processing this malformed message
                        }

                        radio_message.set_sender_node_id(last_sender);

                        process_message(
                            radio_message,
                            received_packet.link_quality,
                            outgoing_message_queue_sender,
                            incoming_message_queue_sender,
                            &mut relay_manager,
                            own_node_id,
                        );
                    }
                }
            }
            //Handle processing results from embedding application
            Either4::Second(process_result) => {
                //check if process result is a block added and added it to last processing messages
                if let MessageProcessingResult::NewBlockAdded(ref msg) = process_result {
                    last_received_messages.add_message(msg);
                    log!(
                        Level::Trace,
                        "[{}] New block added to last received messages: {}",
                        own_node_id,
                        msg.sequence().unwrap_or(0)
                    );
                }
                if let MessageProcessingResult::NewTransactionAdded(ref msg) = process_result {
                    last_received_messages.add_message(msg);
                    log!(
                        Level::Trace,
                        "[{}] New transaction added to last received messages: {}",
                        own_node_id,
                        msg.sequence().unwrap_or(0)
                    );
                }

                if let MessageProcessingResult::AlreadyHaveMessage(message_type, sequence, payload_checksum) = process_result {
                    log!(
                        Level::Trace,
                        "[{}] Already have message: type={}, sequence={}, checksum={}. Clearing from packet_buffer",
                        own_node_id,
                        message_type,
                        sequence,
                        payload_checksum
                    );
                    //remove all packets of the message from packet buffer
                    for packet in packet_buffer.iter_mut().take(INCOMING_PACKET_BUFFER_SIZE) {
                        if let Some(p) = packet {
                            if p.packet.message_type() == message_type
                                && p.packet.sequence() == Some(sequence)
                                && p.packet.payload_checksum() == Some(payload_checksum)
                            {
                                *packet = None;
                            }
                        }
                    }
                    //add it to the last received message list, to handle next packets in shorter loop
                    last_received_messages.add_message_with_parameters(message_type, sequence, payload_checksum);
                } else {
                    relay_manager.process_processing_result(process_result);
                }
            }
            Either4::Third(_) => {
                if let RelayResult::SendMessage(response_message) = relay_manager.process_timed_tasks() {
                    let result = outgoing_message_queue_sender.try_send(response_message);
                    if let Err(result_error) = result {
                        let TrySendError::Full(failed_message) = result_error;
                        log!(
                            Level::Warn,
                            "[{}] Failed to send message to outgoing_message_queue. The queue is full. Dropping message: messagetype: {}, sender_node_id: {}",
                            own_node_id,
                            failed_message.message_type(),
                            failed_message.sender_node_id(),
                        );
                    };
                }
            }
            Either4::Fourth(_) => {
                // Time to check for missing packets
                log!(Level::Trace, "[{}] Checking for missing packets", own_node_id);
                next_missing_packet_check = Instant::now() + embassy_time::Duration::from_secs(retry_interval_for_missing_packets as u64);

                let mut missing_packet_item: Option<&PacketBufferItem> = None;

                for packet_item in packet_buffer.iter().take(INCOMING_PACKET_BUFFER_SIZE).flatten() {
                    let packet_age = Instant::now() - packet_item.arrival_time;
                    if packet_age.as_secs() >= retry_interval_for_missing_packets as u64 {
                        if let Some(missing_packets) = &missing_packet_item {
                            if packet_item.arrival_time > missing_packets.arrival_time {
                                missing_packet_item = Some(packet_item);
                            }
                        } else {
                            missing_packet_item = Some(packet_item);
                        }
                    }
                }

                if let Some(missing_packet) = missing_packet_item {
                    if missing_packet.packet.message_type() == MessageType::AddBlock as u8 {
                        if let (Some(sequence), Some(payload_checksum)) = (missing_packet.packet.sequence(), missing_packet.packet.payload_checksum()) {
                            let mut missing_message = RadioMessage::request_block_part_with(own_node_id, sequence, payload_checksum);
                            //collect non missing packet indexes (use the check buffer for that)
                            packet_check_buffer.fill(PACKET_CHECK_BUFFER_EMPTY_VALUE);

                            for packet_item in packet_buffer.iter().take(INCOMING_PACKET_BUFFER_SIZE).flatten() {
                                if packet_item
                                    .packet
                                    .same_message(&missing_packet.packet.data[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE])
                                {
                                    packet_check_buffer[packet_item.packet.packet_index() as usize] = 1_u8;
                                }
                            }

                            //add missing packet indexes to the message
                            for (i, packet_check) in packet_check_buffer.iter().enumerate().take(missing_packet.packet.total_packet_count() as usize) {
                                if *packet_check == PACKET_CHECK_BUFFER_EMPTY_VALUE {
                                    log!(
                                        Level::Debug,
                                        "[{}] Requesting missing packet index {} for message type {}, sequence {}, payload_checksum {}",
                                        own_node_id,
                                        i,
                                        missing_packet.packet.message_type(),
                                        sequence,
                                        payload_checksum
                                    );
                                    if missing_message.add_packet_index_to_request_block_part(i as u8).is_err() {
                                        log!(
                                            Level::Warn,
                                            "[{}] Failed to add packet index {} to RequestBlockPart message. Maximum indexes reached.",
                                            own_node_id,
                                            i
                                        );
                                    }
                                }
                            }

                            //send the message
                            log!(
                                Level::Debug,
                                "[{}] Sending missing packet request: {}",
                                own_node_id,
                                missing_message.sequence().unwrap_or(0)
                            );
                            let result = outgoing_message_queue_sender.try_send(missing_message);
                            if let Err(result_error) = result {
                                let TrySendError::Full(failed_message) = result_error;
                                log!(
                                    Level::Warn,
                                    "[{}] Failed to send message to outgoing_message_queue. The queue is full. Dropping message: messagetype: {}, sender_node_id: {}",
                                    own_node_id,
                                    failed_message.message_type(),
                                    failed_message.sender_node_id(),
                                );
                            };
                        }
                    }
                }
            }
        }
    }
}

/// Processes a fully assembled message for relay and application delivery
///
/// This function performs validation, relay coordination, and routing for received messages.
/// It handles CRC checking for critical message types, coordinates with the relay manager
/// for network forwarding decisions, and routes messages to the appropriate queue based on type.
///
/// # Processing Steps
///
/// 1. **CRC Validation**: For AddBlock and AddTransaction messages, verifies payload integrity
/// 2. **Relay Coordination**: Consults RelayManager for forwarding/duplicate detection
/// 3. **Queue Routing**: Routes eligible messages to outgoing (relay) or incoming (app) queues
///
/// # Arguments
///
/// * `message` - The fully assembled message to process
/// * `last_link_quality` - Link quality from the last received packet (0-255)
/// * `outgoing_message_queue_sender` - Queue sender for messages to relay/forward
/// * `incoming_message_queue_sender` - Queue sender for messages to deliver to application
/// * `relay_manager` - RelayManager instance for network topology and forwarding decisions
/// * `own_node_id` - This node's unique network identifier for logging
///
/// # Message Type Handling
///
/// Messages are categorized into:
/// - **Echo Protocol**: RequestEcho, EchoResult - handled by RelayManager only
/// - **Application Messages**: AddBlock, AddTransaction, Support, etc. - routed to application
/// - **Relay-only Messages**: Already-processed duplicates are not forwarded to application
///
/// # Error Handling
///
/// - Invalid CRC: Message is logged and dropped
/// - Full queues: Message is logged and dropped with warning
/// - Duplicates: Detected by RelayManager, not routed to application
fn process_message(
    message: RadioMessage,
    last_link_quality: u8,
    outgoing_message_queue_sender: OutgoingMessageQueueSender,
    incoming_message_queue_sender: IncomingMessageQueueSender,
    relay_manager: &mut RelayManager<CONNECTION_MATRIX_SIZE, WAIT_POOL_SIZE>,
    own_node_id: u32,
) {
    log::trace!(
        "[{}] Processing incomming message: type: {}, sender: {}, length: {}",
        own_node_id,
        message.message_type(),
        message.sender_node_id(),
        message.length()
    );
    //crc check, dropping message if it fails
    if (message.message_type() == MessageType::AddBlock as u8 || message.message_type() == MessageType::AddTransaction as u8)
        && !message.check_payload_checksum()
    {
        if let Some(sequence) = message.sequence() {
            log!(
                Level::Warn,
                "[{}] CRC check failed for message: type: {}, sequence: {}, sender: {}. Dropping message.",
                own_node_id,
                message.message_type(),
                sequence,
                message.sender_node_id()
            );
        } else {
            log!(
                Level::Warn,
                "[{}] CRC check failed for message without sequence: type: {}, sender: {}. Dropping message.",
                own_node_id,
                message.message_type(),
                message.sender_node_id()
            );
        }
        return;
    }

    let mut should_process = true;
    let relay_result = relay_manager.process_received_message(&message, last_link_quality);

    match relay_result {
        RelayResult::None => {
            // No action needed
        }
        RelayResult::SendMessage(message) => {
            let result = outgoing_message_queue_sender.try_send(message);
            if let Err(result_error) = result {
                let TrySendError::Full(failed_message) = result_error;
                log!(
                    Level::Warn,
                    "[{}] Failed to send message to outgoing_message_queue. The queue is full. Dropping message: messagetype: {}, sender_node_id: {}",
                    own_node_id,
                    failed_message.message_type(),
                    failed_message.sender_node_id(),
                );
            };
        }
        RelayResult::AlreadyHaveMessage => {
            should_process = false;
        }
    }

    if (message.message_type() == MessageType::AddBlock as u8
        || message.message_type() == MessageType::RequestFullBlock as u8
        || message.message_type() == MessageType::RequestBlockPart as u8
        || message.message_type() == MessageType::AddTransaction as u8
        || message.message_type() == MessageType::RequestNewMempoolItem as u8
        || message.message_type() == MessageType::Support as u8)
        && should_process
    {
        let result = incoming_message_queue_sender.try_send(IncomingMessageItem::NewMessage(message));

        if let Err(result_error) = result {
            let failed_message = match result_error {
                TrySendError::Full(IncomingMessageItem::NewMessage(msg)) => msg,
                _ => return,
            };
            log!(
                Level::Warn,
                "[{}] Failed to send message to incoming_message_queue. The queue is full. Dropping message: messagetype: {}, sender_node_id: {}",
                own_node_id,
                failed_message.message_type(),
                failed_message.sender_node_id(),
            );
        };
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use crate::relay_manager::RelayManager;
    use crate::MessageProcessingResult;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::Channel;

    fn scoring() -> ScoringMatrix {
        ScoringMatrix::new([[1, 1, 1, 1]; 4], 16, 48, 0)
    }

    fn new_rm() -> RelayManager<{ crate::CONNECTION_MATRIX_SIZE }, { crate::WAIT_POOL_SIZE }> {
        RelayManager::with(1, 1, 1, 1, scoring(), 1, 123)
    }

    #[test]
    fn request_echo_routes_only_to_outgoing() {
        type OutCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::OUTGOING_MESSAGE_QUEUE_SIZE }>;
        type InCh = Channel<CriticalSectionRawMutex, crate::IncomingMessageItem, { crate::INCOMING_MESSAGE_QUEUE_SIZE }>;

        let outgoing: &'static OutCh = Box::leak(Box::new(Channel::new()));
        let incoming: &'static InCh = Box::leak(Box::new(Channel::new()));

        let out_tx = outgoing.sender();
        let out_rx = outgoing.receiver();
        let in_tx = incoming.sender();
        let in_rx = incoming.receiver();

        let mut rm = new_rm();
        let msg = RadioMessage::request_echo_with(2);
        process_message(msg, 10, out_tx, in_tx, &mut rm, 1);

        // With echo pooling, there is no immediate outgoing echo
        assert!(out_rx.try_receive().is_err(), "no immediate echo expected");
        // Incoming should be empty as well
        assert!(in_rx.try_receive().is_err());
    }

    #[test]
    fn add_block_routes_to_incoming_when_not_duplicate() {
        type OutCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::OUTGOING_MESSAGE_QUEUE_SIZE }>;
        type InCh = Channel<CriticalSectionRawMutex, crate::IncomingMessageItem, { crate::INCOMING_MESSAGE_QUEUE_SIZE }>;

        let outgoing: &'static OutCh = Box::leak(Box::new(Channel::new()));
        let incoming: &'static InCh = Box::leak(Box::new(Channel::new()));

        let out_tx = outgoing.sender();
        let out_rx = outgoing.receiver();
        let in_tx = incoming.sender();
        let in_rx = incoming.receiver();

        let mut rm = new_rm();
        let msg = RadioMessage::add_block_with(3, 100, &[1, 2, 3]);
        process_message(msg, 5, out_tx, in_tx, &mut rm, 1);

        // Incoming should have the AddBlock
        let item = in_rx.try_receive().expect("expected an incoming message");
        let in_msg = match item {
            crate::IncomingMessageItem::NewMessage(m) => m,
            other => panic!("expected NewMessage, got {:?}", core::mem::discriminant(&other)),
        };
        assert_eq!(in_msg.message_type(), MessageType::AddBlock as u8);

        // Outgoing should be empty
        assert!(out_rx.try_receive().is_err());
    }

    #[test]
    fn duplicate_message_is_not_routed_to_incoming() {
        type OutCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::OUTGOING_MESSAGE_QUEUE_SIZE }>;
        type InCh = Channel<CriticalSectionRawMutex, crate::IncomingMessageItem, { crate::INCOMING_MESSAGE_QUEUE_SIZE }>;

        let outgoing: &'static OutCh = Box::leak(Box::new(Channel::new()));
        let incoming: &'static InCh = Box::leak(Box::new(Channel::new()));

        let out_tx = outgoing.sender();
        let out_rx = outgoing.receiver();
        let in_tx = incoming.sender();
        let in_rx = incoming.receiver();

        let mut rm = new_rm();

        // Pre-populate wait pool with a message
        let orig = RadioMessage::add_block_with(3, 5, &[9]);
        rm.process_processing_result(MessageProcessingResult::NewBlockAdded(orig));

        // Process an equal message: should be treated as duplicate
        let dup = RadioMessage::add_block_with(4, 5, &[9]); // different sender ignored in equality
        process_message(dup, 0, out_tx, in_tx, &mut rm, 1);

        assert!(in_rx.try_receive().is_err());
        assert!(out_rx.try_receive().is_err());
    }
}

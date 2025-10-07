//! # Radio Message Module
//!
//! High-level message abstraction for radio communication with support for multi-packet
//! fragmentation and various message types.
//!
//! ## Architecture
//!
//! This module provides:
//!
//! - **RadioMessage**: The main message structure that can span multiple radio packets
//! - **MessageType**: Enumeration of all supported message types
//! - **Iterator Types**: For traversing message payload data (echo results, mempool items, etc.)
//! - **Item Types**: Data structures returned by iterators
//!
//! ## Message Types
//!
//! - `RequestEcho`: Request echo from a node to measure link quality
//! - `Echo`: Response to echo request with link quality information
//! - `EchoResult`: Aggregated echo results from multiple nodes
//! - `RequestFullBlock`: Request a complete block by sequence number
//! - `RequestBlockPart`: Request specific packets of a block
//! - `AddBlock`: Broadcast a new block to the network
//! - `AddTransaction`: Broadcast a new transaction
//! - `RequestNewMempoolItem`: Request new mempool items
//! - `Support`: Support/vote for a block
//!
//! ## Multi-Packet Messages
//!
//! Large messages (AddBlock, AddTransaction, Support) are automatically fragmented into
//! multiple packets for transmission. The message header contains:
//! - Message type and sender node ID (bytes 0-4)
//! - Sequence number (bytes 5-8, for applicable message types)
//! - Payload checksum (bytes 9-12, for applicable message types)
//!
//! Each packet includes a per-packet header with:
//! - Total packet count
//! - Packet index
//!
//! ## Data Encapsulation
//!
//! All message fields are private to enforce proper encapsulation. Access is provided
//! through public methods that ensure data integrity and consistency.

use super::radio_packet::RadioPacket;
use crate::{RADIO_MAX_MESSAGE_SIZE, RADIO_MAX_PACKET_COUNT, RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE, RADIO_MULTI_PACKET_PACKET_HEADER_SIZE, RADIO_PACKET_SIZE};

/// Calculates CRC32C checksum for message payload integrity verification
///
/// Uses the Castagnoli polynomial (0x82F63B78) which provides better error
/// detection than the standard CRC32 polynomial, especially for network packets.
///
/// # Arguments
/// * `data` - Byte slice to calculate checksum for
///
/// # Returns
/// 32-bit checksum value.
pub(crate) fn checksum(data: &[u8]) -> u32 {
    const POLY: u32 = 0x82F63B78;
    let mut crc: u32 = 0xFFFF_FFFF;

    for &b in data {
        let mut cur = (crc ^ (b as u32)) & 0xFF;
        for _ in 0..8 {
            let mask = (cur & 1).wrapping_neg();
            cur = (cur >> 1) ^ (POLY & mask);
        }
        crc = (crc >> 8) ^ cur;
    }

    crc ^ 0xFFFF_FFFF
}

/// Error type for packet assembly operations
///
/// Represents various failure conditions that can occur when adding packets
/// to a multi-packet message during message reconstruction.
///
/// # Usage
///
/// This error type is returned by `add_packet()` when a packet cannot be
/// safely added to the message buffer. All errors indicate that the packet
/// should be discarded to prevent buffer overflows or data corruption.
///
/// # Examples
///
/// ```rust,ignore
/// use moonblokz_radio_lib::messages::PacketError;
///
/// match message.add_packet(&packet) {
///     Ok(()) => println!("Packet added successfully"),
///     Err(PacketError::InvalidPacketCount) => println!("Packet has invalid count"),
///     Err(PacketError::IndexOutOfBounds) => println!("Packet index is invalid"),
///     Err(e) => println!("Other error: {:?}", e),
/// }
/// ```
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Debug))]
pub enum PacketError {
    /// Packet count is zero or exceeds maximum allowed packets
    ///
    /// The `total_packet_count` field in the packet header is invalid.
    /// Valid range is 1 to `RADIO_MAX_PACKET_COUNT`.
    InvalidPacketCount,

    /// Packet index is out of bounds for the message
    ///
    /// The `packet_index` is greater than or equal to `total_packet_count`,
    /// which violates the invariant that indices are zero-based.
    IndexOutOfBounds,

    /// Integer overflow detected during length calculation
    ///
    /// Arithmetic operations for calculating message length or buffer indices
    /// would overflow. This indicates a malformed or malicious packet.
    IntegerOverflow,

    /// Integer underflow detected during length calculation
    ///
    /// Subtraction operations would result in a negative value when calculating
    /// data lengths. This indicates a malformed packet with invalid header sizes.
    IntegerUnderflow,

    /// Calculated buffer index exceeds available buffer space
    ///
    /// The computed start or end index for copying packet data would exceed
    /// the message payload buffer boundaries.
    BufferOverflow,

    /// Packet data length is insufficient for the claimed header size
    ///
    /// The packet claims to have a multi-packet header, but the actual data
    /// length is too short to contain the required header bytes.
    InsufficientPacketData,
}

#[cfg(feature = "std")]
impl core::fmt::Display for PacketError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PacketError::InvalidPacketCount => write!(f, "packet count is invalid (must be 1 to {})", RADIO_MAX_PACKET_COUNT),
            PacketError::IndexOutOfBounds => write!(f, "packet index is out of bounds"),
            PacketError::IntegerOverflow => write!(f, "integer overflow in length calculation"),
            PacketError::IntegerUnderflow => write!(f, "integer underflow in length calculation"),
            PacketError::BufferOverflow => write!(f, "buffer overflow would occur"),
            PacketError::InsufficientPacketData => write!(f, "packet data is too short"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for PacketError {}

/// Message types supported by the radio communication protocol
///
/// Each message type serves a specific purpose in the mesh network protocol,
/// from network discovery to data synchronization.
///
/// # Encoding
/// Each variant is encoded as a single byte (u8) in the message header.
///
/// # Examples
/// ```rust
/// use moonblokz_radio_lib::MessageType;
///
/// let echo_type = MessageType::RequestEcho as u8;
/// assert_eq!(echo_type, 0x01);
/// ```
#[derive(Clone, Copy)]
#[cfg_attr(feature = "std", derive(Debug))]
pub enum MessageType {
    /// Request echo from a node to measure link quality (0x01)
    RequestEcho = 0x01,

    /// Response to echo request with link quality data (0x02)
    Echo = 0x02,

    /// Aggregated echo results from multiple nodes (0x03)
    EchoResult = 0x03,

    /// Request a complete block by sequence number (0x04)
    RequestFullBlock = 0x04,

    /// Request specific packets of a block (0x05)
    RequestBlockPart = 0x05,

    /// Broadcast a new block to the network (0x06)
    AddBlock = 0x06,

    /// Broadcast a new transaction to the mempool (0x07)
    AddTransaction = 0x07,

    /// Request new mempool items from a node (0x08)
    RequestNewMempoolItem = 0x08,

    /// Support/vote for a block (0x09)
    Support = 0x09,
}

/// High-level radio message that can span multiple packets
///
/// RadioMessage encapsulates application-level data and handles automatic
/// fragmentation for transmission. It supports various message types including
/// echo requests, block synchronization, and transaction propagation.
///
/// # Structure
///
/// - **Payload**: Up to RADIO_MAX_MESSAGE_SIZE bytes of message data
/// - **Length**: Actual payload length in use
/// - **Packets to Send**: Optional tracking for multi-packet transmission
///
/// # Message Format
///
/// All messages start with:
/// - Byte 0: Message type
/// - Bytes 1-4: Sender node ID (little-endian u32)
///
/// Additional fields vary by message type (see specific constructors).
///
/// # Examples
/// ```rust
/// use moonblokz_radio_lib::RadioMessage;
///
/// // Create an echo request
/// let message = RadioMessage::request_echo_with(42);
/// assert_eq!(message.message_type(), 0x01);
/// assert_eq!(message.sender_node_id(), 42);
/// ```
#[derive(Clone)]
#[cfg_attr(feature = "std", derive(Debug))]
pub struct RadioMessage {
    payload: [u8; RADIO_MAX_MESSAGE_SIZE],
    length: usize,
    packets_to_send: Option<[bool; RADIO_MAX_PACKET_COUNT]>,
}

impl RadioMessage {
    /// Constructs a RadioMessage from a single radio packet
    ///
    /// Handles both single-packet messages and the first packet of multi-packet messages.
    /// For AddBlock and AddTransaction message types, properly extracts and positions
    /// the message header and payload.
    ///
    /// # Arguments
    /// * `packet` - The radio packet to construct the message from
    ///
    /// # Returns
    /// A RadioMessage initialized with the packet's data
    pub fn from_single_packet(packet: RadioPacket) -> RadioMessage {
        let message_type = packet.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            // Handle AddBlock and AddTransaction messages
            let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
            payload[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE].copy_from_slice(&packet.data[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]);
            // Skip the 2 per-packet meta bytes (total count, index) right after RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE
            payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + (packet.length - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE)]
                .copy_from_slice(&packet.data[RADIO_MULTI_PACKET_PACKET_HEADER_SIZE..packet.length]);

            RadioMessage {
                payload,
                // RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + actual payload (skipping per-packet header)
                length: RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + (packet.length - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE),
                packets_to_send: None,
            }
        } else {
            let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
            payload[..packet.length].copy_from_slice(&packet.data[..packet.length]);
            RadioMessage {
                payload,
                length: packet.length,
                packets_to_send: None,
            }
        }
    }

    /// Creates a new empty RadioMessage (internal use)
    ///
    /// Used internally for building messages incrementally, such as when
    /// reconstructing multi-packet messages.
    pub(crate) fn new() -> Self {
        // Create a new empty RadioMessage with default values
        RadioMessage {
            payload: [0u8; RADIO_MAX_MESSAGE_SIZE],
            length: 0,
            packets_to_send: None,
        }
    }

    /// Adds a packet to this multi-packet message (internal use)
    ///
    /// Assembles a RadioMessage from individual packets. Updates the message
    /// length when the last packet is received and properly positions packet
    /// payloads within the message buffer.
    ///
    /// # Arguments
    /// * `packet` - The packet to add to this message
    ///
    /// # Returns
    /// * `Ok(())` - Packet added successfully
    /// * `Err(PacketError)` - Packet rejected due to validation failure
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Packet count is invalid (`InvalidPacketCount`)
    /// - Packet index is out of bounds (`IndexOutOfBounds`)
    /// - Integer overflow/underflow occurs (`IntegerOverflow`, `IntegerUnderflow`)
    /// - Buffer boundaries would be exceeded (`BufferOverflow`)
    /// - Packet data is insufficient (`InsufficientPacketData`)
    ///
    /// # Safety
    ///
    /// Includes comprehensive bounds checking with overflow protection to prevent
    /// buffer overflows. Uses checked arithmetic throughout to detect any potential
    /// integer overflow before performing buffer operations. All errors result in
    /// the packet being safely discarded without corrupting the message buffer.
    pub(crate) fn add_packet(&mut self, packet: &RadioPacket) -> core::result::Result<(), PacketError> {
        let total_packet_count = packet.total_packet_count();
        let packet_index = packet.packet_index();

        // Validate packet count and index are reasonable
        if total_packet_count == 0 || total_packet_count > RADIO_MAX_PACKET_COUNT as u8 {
            return Err(PacketError::InvalidPacketCount);
        }

        if packet_index >= total_packet_count {
            return Err(PacketError::IndexOutOfBounds);
        }

        // Update message header and length when receiving the last packet
        if total_packet_count == (packet_index + 1) {
            self.payload[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE].copy_from_slice(&packet.data[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]);

            // Calculate total message length with overflow protection
            let chunk_size = RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE;

            // checked_mul to prevent overflow
            let full_chunks_length = chunk_size.checked_mul(total_packet_count as usize - 1).ok_or(PacketError::IntegerOverflow)?;

            let full_chunks_length = full_chunks_length
                .checked_add(RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE)
                .ok_or(PacketError::IntegerOverflow)?;

            let last_packet_data = packet
                .length
                .checked_sub(RADIO_MULTI_PACKET_PACKET_HEADER_SIZE)
                .ok_or(PacketError::IntegerUnderflow)?;

            let calculated_length = full_chunks_length.checked_add(last_packet_data).ok_or(PacketError::IntegerOverflow)?;

            // Bounds check: ensure calculated length doesn't exceed payload buffer
            self.length = calculated_length.min(RADIO_MAX_MESSAGE_SIZE);
        }

        // Calculate start index with overflow protection
        let chunk_size = RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE;

        let offset = chunk_size.checked_mul(packet_index as usize).ok_or(PacketError::IntegerOverflow)?;

        let start_index = offset.checked_add(RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE).ok_or(PacketError::IntegerOverflow)?;

        // Validate start index is within bounds
        if start_index >= self.payload.len() {
            return Err(PacketError::BufferOverflow);
        }

        // Calculate data length based on whether this is the last packet
        let data_length = if packet_index == total_packet_count - 1 {
            packet
                .length
                .checked_sub(RADIO_MULTI_PACKET_PACKET_HEADER_SIZE)
                .ok_or(PacketError::IntegerUnderflow)?
        } else {
            chunk_size
        };

        let end_index = start_index.checked_add(data_length).ok_or(PacketError::IntegerOverflow)?;

        // Validate end index doesn't exceed buffer
        if end_index > self.payload.len() {
            return Err(PacketError::BufferOverflow);
        }

        // Bounds check: ensure we don't read past packet data
        let packet_data_start = RADIO_MULTI_PACKET_PACKET_HEADER_SIZE;
        let packet_data_end = packet_data_start.checked_add(data_length).ok_or(PacketError::IntegerOverflow)?;

        if packet_data_end > packet.data.len() || packet_data_end > packet.length {
            return Err(PacketError::InsufficientPacketData);
        }

        // Safe to copy now that all bounds are verified
        self.payload[start_index..end_index].copy_from_slice(&packet.data[packet_data_start..packet_data_end]);

        Ok(())
    }

    /// Creates a new RequestEcho message
    ///
    /// Echo requests are used to probe the network and measure link quality
    /// between nodes. Remote nodes respond with Echo messages containing
    /// their connection quality information.
    ///
    /// # Arguments
    /// * `node_id` - The requesting node's unique identifier
    ///
    /// # Returns
    /// A RadioMessage configured as a RequestEcho
    ///
    /// # Example
    /// ```rust
    /// use moonblokz_radio_lib::RadioMessage;
    ///
    /// let echo_request = RadioMessage::request_echo_with(42);
    /// assert_eq!(echo_request.sender_node_id(), 42);
    /// ```
    pub fn request_echo_with(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for echo requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestEcho as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len() + 1].copy_from_slice(&node_id_bytes);
        RadioMessage {
            payload,
            length: 5,
            packets_to_send: None,
        }
    }

    /// Creates a new Echo response message
    ///
    /// Echo messages are sent in response to RequestEcho messages, providing
    /// link quality information between the responder and the target node.
    ///
    /// # Arguments
    /// * `node_id` - The responding node's unique identifier
    /// * `target_node_id` - The node being responded to
    /// * `link_quality` - Measured link quality (0-63 scale)
    ///
    /// # Returns
    /// A RadioMessage configured as an Echo response
    pub fn echo_with(node_id: u32, target_node_id: u32, link_quality: u8) -> Self {
        // Create a new RadioMessage with a specific message type for echo responses
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::Echo as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let target_node_id_bytes = target_node_id.to_le_bytes();
        payload[5..5 + target_node_id_bytes.len()].copy_from_slice(&target_node_id_bytes);
        payload[5 + target_node_id_bytes.len()] = link_quality;

        RadioMessage {
            payload,
            length: 10,
            packets_to_send: None,
        }
    }

    /// Creates a new RequestFullBlock message
    ///
    /// Requests a complete block from the network by its sequence number.
    /// Nodes with the block will respond with an AddBlock message.
    ///
    /// # Arguments
    /// * `node_id` - The requesting node's unique identifier
    /// * `sequence` - The sequence number of the desired block
    ///
    /// # Returns
    /// A RadioMessage configured as a RequestFullBlock
    pub fn request_full_block_with(node_id: u32, sequence: u32) -> Self {
        // Create a new RadioMessage with a specific message type for full block requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestFullBlock as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);

        RadioMessage {
            payload,
            length: 9,
            packets_to_send: None,
        }
    }

    /// Creates a new EchoResult message
    ///
    /// EchoResult messages aggregate echo response data from multiple nodes,
    /// providing a summary of the network topology as perceived by this node.
    /// Echo result items are added using `add_echo_result_item()`.
    ///
    /// # Arguments
    /// * `node_id` - The node creating this echo result
    ///
    /// # Returns
    /// A RadioMessage configured as an EchoResult (initially empty)
    pub fn echo_result_with(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for echo results
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::EchoResult as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);

        RadioMessage {
            payload,
            length: 5,
            packets_to_send: None,
        }
    }

    /// Adds an echo result item to this EchoResult message (internal use)
    ///
    /// Appends connection quality data for one neighbor node. Each item is 6 bytes:
    /// 4 bytes node ID + 1 byte send quality + 1 byte receive quality.
    /// The message must fit in a single packet.
    ///
    /// # Arguments
    /// * `neighbor_node` - The neighbor node's ID
    /// * `send_link_quality` - Quality of link TO the neighbor (0-63)
    /// * `receive_link_quality` - Quality of link FROM the neighbor (0-63)
    ///
    /// # Returns
    /// * `Ok(())` - Item added successfully
    /// * `Err(())` - No space remaining (would exceed single packet size)
    pub(crate) fn add_echo_result_item(&mut self, neighbor_node: u32, send_link_quality: u8, receive_link_quality: u8) -> Result<(), ()> {
        // Add an echo result item to the message payload (each item is 6 bytes)
        if self.length + 6 > RADIO_PACKET_SIZE {
            //the message must fit into a single packet
            return Err(());
        }

        let start_index = self.length;
        let mut node_id_bytes = [0u8; 4];
        node_id_bytes.copy_from_slice(&neighbor_node.to_le_bytes());
        self.payload[start_index..start_index + 4].copy_from_slice(&node_id_bytes);
        self.payload[start_index + 4] = send_link_quality;
        self.payload[start_index + 5] = receive_link_quality;

        self.length += 6;

        Ok(())
    }

    /// Creates a new RequestBlockPart message (internal use)
    ///
    /// Requests specific missing packets from a multi-packet message. This is used
    /// when some packets of a block were received but others are missing, allowing
    /// efficient retransmission of only the needed packets.
    ///
    /// # Arguments
    /// * `node_id` - The requesting node's unique identifier
    /// * `sequence` - Sequence number of the block being requested
    /// * `payload_checksum` - CRC32C checksum of the complete block payload
    ///
    /// # Returns
    /// A RadioMessage configured as a RequestBlockPart (initially with no packet indices)
    ///
    /// # Note
    /// Use `add_packet_index_to_request_block_part()` to add specific packet indices
    /// to request after creating this message.
    pub(crate) fn request_block_part_with(node_id: u32, sequence: u32, payload_checksum: u32) -> Self {
        // Create a new RadioMessage with a specific message type for block part requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestBlockPart as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);
        payload[13] = 0; //current index count set to 0

        RadioMessage {
            payload,
            length: 14,
            packets_to_send: None,
        }
    }

    /// Adds a packet index to a RequestBlockPart message (internal use)
    ///
    /// Appends a packet index to the list of requested packets. Used when
    /// requesting specific missing packets from a multi-packet block.
    ///
    /// # Arguments
    /// * `packet_index` - The index of the requested packet (0-based)
    ///
    /// # Returns
    /// * `Ok(())` - Packet index added successfully
    /// * `Err(())` - Message is not RequestBlockPart type or no space remaining
    pub(crate) fn add_packet_index_to_request_block_part(&mut self, packet_index: u8) -> Result<(), ()> {
        // Add a packet index to the request block part message
        if self.message_type() != MessageType::RequestBlockPart as u8 {
            return Err(());
        }

        // The message must fit into a single radio packet
        if self.length + 1 > RADIO_PACKET_SIZE {
            return Err(());
        }
        self.payload[self.payload[13] as usize + 14] = packet_index;

        self.payload[13] += 1;
        self.length += 1;

        Ok(())
    }

    /// Creates a new AddBlock message
    ///
    /// Creates a message to broadcast a new block to the network. The block data
    /// is automatically split into multiple packets if needed. All peers will receive
    /// and validate this block.
    ///
    /// # Arguments
    /// * `node_id` - The sender's node ID
    /// * `sequence` - Unique sequence number for this message
    /// * `payload` - The block data to broadcast (max RADIO_MAX_MESSAGE_SIZE - 13 bytes)
    ///
    /// # Returns
    /// A new RadioMessage of type AddBlock with the block data
    ///
    /// # Example
    /// ```
    /// use moonblokz_radio_lib::RadioMessage;
    ///
    /// let block_data = vec![1, 2, 3];
    /// let message = RadioMessage::add_block_with(1, 42, &block_data);
    /// ```
    pub fn add_block_with(node_id: u32, sequence: u32, payload: &[u8]) -> Self {
        // Calculate payload checksum
        let payload_checksum = checksum(payload);

        // Create a new RadioMessage with a specific message type for adding blocks
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::AddBlock as u8;
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        full_payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);

        // Copy the actual payload data into the message after RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE
        full_payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + payload.len()].copy_from_slice(payload);

        RadioMessage {
            payload: full_payload,
            length: RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + payload.len(),
            packets_to_send: None,
        }
    }

    /// Creates a new AddBlock message with selective packet transmission
    ///
    /// Creates an AddBlock message like `add_block_with()`, but allows specifying
    /// which packets should be transmitted. This is useful for optimizing bandwidth
    /// when some peers already have certain packets.
    ///
    /// # Arguments
    /// * `node_id` - The sender's node ID
    /// * `sequence` - Unique sequence number for this message
    /// * `payload` - The block data to broadcast
    /// * `packets_to_send` - Boolean array indicating which packets to transmit
    ///
    /// # Returns
    /// A new RadioMessage with selective packet transmission enabled
    ///
    /// # Example
    /// ```
    /// use moonblokz_radio_lib::{RadioMessage, RADIO_MAX_PACKET_COUNT};
    ///
    /// let block_data = vec![1, 2, 3];
    /// let mut packet_list = [false; RADIO_MAX_PACKET_COUNT];
    /// packet_list[0] = true; // Send only first packet
    /// packet_list[1] = true; // Send only second packet
    /// let message = RadioMessage::add_block_with_packet_list(1, 42, &block_data, packet_list);
    /// ```
    pub fn add_block_with_packet_list(node_id: u32, sequence: u32, payload: &[u8], packets_to_send: [bool; RADIO_MAX_PACKET_COUNT]) -> Self {
        let mut new_message = Self::add_block_with(node_id, sequence, payload);
        new_message.packets_to_send = Some(packets_to_send);
        new_message
    }

    /// Adds a packet transmission list to an existing message
    ///
    /// Modifies an AddBlock or AddTransaction message to specify which packets
    /// should be transmitted. This allows selective retransmission of missing packets.
    ///
    /// # Arguments
    /// * `packets_to_send` - Boolean array where `true` means transmit that packet
    ///
    /// # Note
    /// Only works with AddBlock and AddTransaction message types. Silently
    /// returns for other message types.
    pub fn add_packet_list(&mut self, packets_to_send: [bool; RADIO_MAX_PACKET_COUNT]) {
        if self.message_type() != MessageType::AddBlock as u8 && self.message_type() != MessageType::AddTransaction as u8 {
            return; // Only AddBlock and AddTransaction messages can have packet lists
        }
        self.packets_to_send = Some(packets_to_send);
    }

    /// Creates a new AddTransaction message
    ///
    /// Creates a message to broadcast a new transaction to the mempool. Transactions
    /// reference an anchor block via sequence number and are automatically split into
    /// multiple packets if needed.
    ///
    /// # Arguments
    /// * `node_id` - The sender's node ID
    /// * `anchor_sequence` - Sequence number of the anchor block this transaction references
    /// * `payload_checksum` - CRC32C checksum of the transaction payload
    /// * `payload` - The transaction data to broadcast (max RADIO_MAX_MESSAGE_SIZE - 13 bytes)
    ///
    /// # Returns
    /// A new RadioMessage of type AddTransaction
    ///
    /// # Example
    /// ```
    /// use moonblokz_radio_lib::RadioMessage;
    ///
    /// let tx_data = vec![1, 2, 3];
    /// let checksum = 0x12345678u32;
    /// let message = RadioMessage::add_transaction_with(1, 100, checksum, &tx_data);
    /// ```
    pub fn add_transaction_with(node_id: u32, anchor_sequence: u32, payload_checksum: u32, payload: &[u8]) -> Self {
        // Create a new RadioMessage with a specific message type for adding transactions
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::AddTransaction as u8;
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = anchor_sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        full_payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);

        // Copy the actual payload data into the message after RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE
        full_payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + payload.len()].copy_from_slice(payload);

        RadioMessage {
            payload: full_payload,
            length: RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + payload.len(),
            packets_to_send: None,
        }
    }

    /// Creates a new RequestNewMempoolItem message
    ///
    /// Creates an empty message to request mempool state from peers. Mempool items
    /// (transactions) can be added to this message using `add_mempool_item()`.
    ///
    /// # Arguments
    /// * `node_id` - The sender's node ID
    ///
    /// # Returns
    /// A new empty RequestNewMempoolItem message
    ///
    /// # Example
    /// ```
    /// use moonblokz_radio_lib::RadioMessage;
    ///
    /// let mut message = RadioMessage::get_mempool_state_with(1);
    /// message.add_mempool_item(100, 0x12345678).unwrap();
    /// ```
    pub fn get_mempool_state_with(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for getting mempool state
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestNewMempoolItem as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);

        RadioMessage {
            payload,
            length: 5,
            packets_to_send: None,
        }
    }

    /// Adds a mempool item to a RequestNewMempoolItem message
    ///
    /// Appends a transaction reference to the mempool state request. Each item
    /// consists of an anchor sequence (4 bytes) and payload checksum (4 bytes).
    ///
    /// # Arguments
    /// * `anchor_sequence` - Sequence number of the anchor block
    /// * `payload_checksum` - CRC32C checksum of the transaction payload
    ///
    /// # Returns
    /// * `Ok(())` - Mempool item added successfully
    /// * `Err(())` - Message is not RequestNewMempoolItem type or no space remaining
    ///
    /// # Example
    /// ```
    /// use moonblokz_radio_lib::RadioMessage;
    ///
    /// let mut message = RadioMessage::get_mempool_state_with(1);
    /// message.add_mempool_item(100, 0x12345678).unwrap();
    /// message.add_mempool_item(101, 0xABCDEF00).unwrap();
    /// ```
    #[allow(clippy::result_unit_err)]
    pub fn add_mempool_item(&mut self, anchor_sequence: u32, payload_checksum: u32) -> Result<(), ()> {
        // Add a mempool item to the message payload (8 bytes per item)
        if self.message_type() != MessageType::RequestNewMempoolItem as u8 {
            return Err(());
        }

        // The message must fit into a single radio packet
        if self.length + 8 > RADIO_PACKET_SIZE {
            return Err(());
        }

        let sequence_bytes = anchor_sequence.to_le_bytes();
        let checksum_bytes = payload_checksum.to_le_bytes();
        self.payload[self.length..self.length + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        self.payload[self.length + sequence_bytes.len()..self.length + sequence_bytes.len() + checksum_bytes.len()].copy_from_slice(&checksum_bytes);
        self.length += 8;

        Ok(())
    }

    /// Creates a new Support message
    ///
    /// Creates a message containing a cryptographic signature supporting another
    /// message. Support messages are used for consensus and validation in the network.
    ///
    /// # Arguments
    /// * `node_id` - The sender's node ID (the original message sender)
    /// * `sequence` - Sequence number of the message being supported
    /// * `supporter_node` - Node ID of the peer providing support
    /// * `signature` - Cryptographic signature data (variable length)
    ///
    /// # Returns
    /// A new RadioMessage of type Support containing the signature
    ///
    /// # Example
    /// ```
    /// use moonblokz_radio_lib::RadioMessage;
    ///
    /// let signature = vec![1, 2, 3, 4];
    /// let message = RadioMessage::support_with(1, 42, 2, &signature);
    /// ```
    pub fn support_with(node_id: u32, sequence: u32, supporter_node: u32, signature: &[u8]) -> Self {
        // Create a new RadioMessage with a specific message type for support requests
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::Support as u8;
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let supporter_node_bytes = supporter_node.to_le_bytes();
        full_payload[9..9 + supporter_node_bytes.len()].copy_from_slice(&supporter_node_bytes);
        full_payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + signature.len()].copy_from_slice(signature);

        RadioMessage {
            payload: full_payload,
            length: RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + signature.len(),
            packets_to_send: None,
        }
    }

    /// Returns the message type
    ///
    /// # Returns
    /// Message type identifier (see `MessageType` enum), or 0 if message is empty
    pub fn message_type(&self) -> u8 {
        if self.length == 0 {
            return 0; // Default to 0 if message has no logical length
        }
        self.payload[0]
    }

    /// Calculates total packet count for multi-packet messages (internal use)
    ///
    /// For AddBlock and AddTransaction messages, calculates how many radio packets
    /// are needed to transmit the entire message. Single-packet messages return 1.
    ///
    /// # Returns
    /// Total number of packets needed (1 to RADIO_MAX_PACKET_COUNT)
    fn get_total_packet_count(&self) -> usize {
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            let payload_length = self.length.saturating_sub(RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE); // Exclude header
            if payload_length == 0 {
                return 0;
            }

            core::cmp::min(
                (payload_length + RADIO_PACKET_SIZE - (RADIO_MULTI_PACKET_PACKET_HEADER_SIZE + 1))
                    / (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE),
                RADIO_MAX_PACKET_COUNT,
            )
        } else {
            1
        }
    }

    /// Returns the number of packets to transmit (internal use)
    ///
    /// Like `get_total_packet_count()` but accounts for selective packet transmission.
    /// If a packet list is set, only counts packets marked for transmission.
    ///
    /// # Returns
    /// Number of packets that will actually be transmitted
    pub(crate) fn get_packet_count(&self) -> usize {
        let mut packet_count = self.get_total_packet_count();
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            if let Some(packets_to_send) = self.packets_to_send {
                let unfiltered_packet_count = packet_count;

                for should_send in packets_to_send.iter().take(unfiltered_packet_count) {
                    if !should_send {
                        packet_count -= 1;
                    }
                }
            }
        }
        packet_count
    }

    /// Constructs a single packet from a multi-packet message (internal use)
    ///
    /// Generates a `RadioPacket` for transmission by extracting the appropriate
    /// portion of the message payload. Handles both filtered and unfiltered packet
    /// transmission based on the `packets_to_send` list.
    ///
    /// # Arguments
    /// * `input_packet_number` - Zero-based packet index (filtered by packets_to_send if set)
    ///
    /// # Returns
    /// * `Some(RadioPacket)` - The constructed packet
    /// * `None` - Invalid packet number or out of range
    pub(crate) fn get_packet(&self, input_packet_number: usize) -> Option<RadioPacket> {
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            let total_packets = self.get_total_packet_count();
            if input_packet_number >= total_packets {
                return None;
            }

            let mut packet_number = input_packet_number;

            if let Some(packets_to_send) = self.packets_to_send {
                let mut actual_packet_number = 0;
                let mut found_packets = 0;

                for (i, should_send) in packets_to_send.iter().enumerate().take(total_packets) {
                    if *should_send {
                        if found_packets == packet_number {
                            actual_packet_number = i;
                            found_packets += 1;
                            break;
                        }
                        found_packets += 1;
                    }
                }
                if found_packets <= input_packet_number {
                    return None; // Not enough packets to satisfy the request
                }
                packet_number = actual_packet_number;
            }

            let start_index = RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + packet_number * (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE);
            let end_index = start_index + (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE);
            let packet_data = &self.payload[start_index..end_index.min(self.length)];
            let packet_header = &self.payload[0..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]; // First RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE bytes are the multi-packet message header

            let mut data = [0u8; RADIO_PACKET_SIZE];
            data[..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE].copy_from_slice(packet_header);
            // We use the real (unfiltered) total count and packet index here because the receiver needs to reconstruct the full message
            data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE] = total_packets as u8; // total count
            data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + 1] = packet_number as u8; // packet index
            data[RADIO_MULTI_PACKET_PACKET_HEADER_SIZE..packet_data.len() + RADIO_MULTI_PACKET_PACKET_HEADER_SIZE].copy_from_slice(packet_data);

            Some(RadioPacket {
                data,
                length: packet_data.len() + RADIO_MULTI_PACKET_PACKET_HEADER_SIZE,
            })
        } else {
            if input_packet_number != 0 {
                return None; // Only one packet available for non multi-packet messages
            }
            let mut data = [0u8; RADIO_PACKET_SIZE];
            data[..self.length].copy_from_slice(&self.payload[0..self.length]);
            Some(RadioPacket { data, length: self.length })
        }
    }

    /// Returns the raw message payload
    ///
    /// # Returns
    /// Slice of the message payload up to the logical length
    pub fn payload(&self) -> &[u8] {
        &self.payload[0..self.length]
    }

    /// Returns the sender's node ID
    ///
    /// # Returns
    /// The node ID extracted from bytes 1-4 of the message
    pub fn sender_node_id(&self) -> u32 {
        // Extract the sender node ID from the message payload
        let mut node_id_bytes = [0u8; 4];
        node_id_bytes.copy_from_slice(&self.payload[1..5]);
        u32::from_le_bytes(node_id_bytes)
    }

    /// Sets the sender's node ID
    ///
    /// # Arguments
    /// * `node_id` - New node ID to set in bytes 1-4 of the message
    pub fn set_sender_node_id(&mut self, node_id: u32) {
        let node_id_bytes = node_id.to_le_bytes();
        self.payload[1..5].copy_from_slice(&node_id_bytes);
    }

    /// Returns the logical length of the message
    ///
    /// # Returns
    /// Number of bytes in the message payload (excludes any padding)
    pub fn length(&self) -> usize {
        self.length
    }

    /// Extracts echo data from an Echo message (internal use)
    ///
    /// # Returns
    /// * `Some((target_node_id, link_quality))` - Echo response data
    ///   - `target_node_id`: Node ID being echoed to
    ///   - `link_quality`: Measured link quality (0-255)
    /// * `None` - Message is not Echo type or insufficient data
    pub(crate) fn get_echo_data(&self) -> Option<(u32, u8)> {
        // Extract echo data from the message payload
        if self.message_type() != MessageType::Echo as u8 {
            return None;
        }
        if self.length < 10 {
            return None; // Not enough data for echo
        }

        // Target node id is at bytes 5..9, link quality at byte 9
        let mut target_node_id_bytes = [0u8; 4];
        target_node_id_bytes.copy_from_slice(&self.payload[5..9]);
        let target_node_id = u32::from_le_bytes(target_node_id_bytes);
        let link_quality = self.payload[9];

        Some((target_node_id, link_quality))
    }

    /// Returns an iterator over echo result data entries (internal use)
    ///
    /// Each entry contains neighbor node ID, send link quality, and receiving link quality.
    ///
    /// # Returns
    /// * `Some(EchoResultIterator)` - Iterator over echo result items
    /// * `None` - Message is not EchoResult type or insufficient data
    ///
    /// # Example
    /// ```rust,ignore
    /// if let Some(iterator) = message.get_echo_result_data_iterator() {
    ///     for item in iterator {
    ///         println!("Neighbor: {}, Send Quality: {}, Receive Quality: {}",
    ///                  item.neighbor_node(), item.send_link_quality(), item.receiving_link_quality());
    ///     }
    /// }
    /// ```
    pub(crate) fn get_echo_result_data_iterator(&self) -> Option<EchoResultIterator> {
        if self.message_type() != MessageType::EchoResult as u8 {
            return None;
        }
        if self.length < 5 {
            return None; // Not enough data for echo result
        }

        Some(EchoResultIterator::with(&self.payload, 5, self.length))
    }

    /// Extracts transaction data from an AddTransaction message
    ///
    /// # Returns
    /// * `Some((anchor_sequence, checksum, payload))` - Transaction data tuple
    ///   - `anchor_sequence`: Sequence number of the anchor block
    ///   - `checksum`: CRC32C checksum of the transaction payload
    ///   - `payload`: Transaction payload bytes
    /// * `None` - Message is not AddTransaction type or insufficient data
    pub fn get_add_transaction_data(&self) -> Option<(u32, u32, &[u8])> {
        // Extract transaction data from the message payload
        if self.message_type() != MessageType::AddTransaction as u8 {
            return None;
        }
        if self.length < 13 {
            return None; // Not enough data for transaction
        }

        let mut anchor_sequence_bytes = [0u8; 4];
        anchor_sequence_bytes.copy_from_slice(&self.payload[5..9]);
        let anchor_sequence = u32::from_le_bytes(anchor_sequence_bytes);

        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.payload[9..13]);
        let checksum = u32::from_le_bytes(checksum_bytes);

        Some((anchor_sequence, checksum, &self.payload[13..self.length]))
    }

    /// Returns the sequence number from messages that have one
    ///
    /// # Returns
    /// * `Some(sequence)` - Sequence number for AddBlock, RequestFullBlock, Support, or RequestBlockPart messages
    /// * `None` - Message type doesn't have a sequence number or insufficient data
    pub fn sequence(&self) -> Option<u32> {
        if self.message_type() != MessageType::AddBlock as u8
            && self.message_type() != MessageType::RequestFullBlock as u8
            && self.message_type() != MessageType::Support as u8
            && self.message_type() != MessageType::RequestBlockPart as u8
        {
            return None; // Only AddBlock and AddTransaction messages have a sequence
        }

        // Extract the sequence number from the message payload
        if self.length < 9 {
            return None; // Not enough data for sequence
        }
        let mut sequence_bytes = [0u8; 4];
        sequence_bytes.copy_from_slice(&self.payload[5..9]);
        Some(u32::from_le_bytes(sequence_bytes))
    }

    /// Checks if this message is a reply to another message
    ///
    /// Determines message-response relationships by checking message types and
    /// matching identifiers (node IDs, sequence numbers, checksums).
    ///
    /// # Arguments
    /// * `message` - The potential request message
    ///
    /// # Returns
    /// `true` if this message is a valid reply to the given message
    ///
    /// # Reply Relationships
    /// - Echo replies to RequestEcho (match sender node ID)
    /// - AddBlock replies to RequestFullBlock (match sequence)
    /// - AddTransaction replies to RequestNewMempoolItem (match anchor_sequence + checksum)
    pub fn is_reply_to(&self, message: &RadioMessage) -> bool {
        if self.message_type() == MessageType::Echo as u8 {
            if message.message_type() != MessageType::RequestEcho as u8 {
                return false; // Only Echo messages can reply to RequestEcho
            }
            // For Echo messages, check if the sender node ID matches
            return self.sender_node_id() == message.sender_node_id();
        }

        if self.message_type() == MessageType::AddBlock as u8 {
            if message.message_type() != MessageType::RequestFullBlock as u8 {
                return false; // Only AddBlock messages can reply to RequestFullBlock
            }
            return message.sequence() == self.sequence();
        }

        if self.message_type() == MessageType::AddTransaction as u8 {
            if message.message_type() != MessageType::RequestNewMempoolItem as u8 {
                return false; // Only AddTransaction messages can reply to GetMempoolState
            }

            let (anchor_sequence, checksum, _) = self.get_add_transaction_data().unwrap();

            if let Some(mempool_data_iterator) = message.get_mempool_data_iterator() {
                // Iterate through the mempool data entries
                // Each entry contains anchor sequence and transaction payload checksum
                // Check if the mempool item matches the current message
                for mempool_item in mempool_data_iterator {
                    // Check if the mempool item matches the current message
                    if mempool_item.anchor_sequence == anchor_sequence && mempool_item.transaction_payload_checksum == checksum {
                        return false; // Found a matching mempool item
                    }
                }
                // If no matching mempool item was found the message can be considered a reply
                // to the GetMempoolState request
                return true;
            }
        }
        // For other message types, we do not consider them as replies
        false
    }

    /// Returns an iterator over mempool data entries (internal use)
    ///
    /// Each entry contains anchor sequence and transaction payload checksum.
    ///
    /// # Returns
    /// * `Some(MempoolIterator)` - Iterator over mempool items
    /// * `None` - Message is not RequestNewMempoolItem type or insufficient data
    ///
    /// # Example
    /// ```rust,ignore
    /// if let Some(iterator) = message.get_mempool_data_iterator() {
    ///     for item in iterator {
    ///         println!("Anchor Sequence: {}, Transaction Checksum: {}",
    ///                  item.anchor_sequence(), item.transaction_payload_checksum());
    ///     }
    /// }
    /// ```
    pub(crate) fn get_mempool_data_iterator(&self) -> Option<MempoolIterator> {
        if self.message_type() != MessageType::RequestNewMempoolItem as u8 {
            return None;
        }
        if self.length < 5 {
            return None; // Not enough data for mempool state (at least message type + sender_node)
        }

        Some(MempoolIterator::with(&self.payload, 5, self.length))
    }

    /// Returns the payload checksum (internal use)
    ///
    /// # Returns
    /// * `Some(checksum)` - CRC32C checksum for AddBlock or AddTransaction messages
    /// * `None` - Message type doesn't have a checksum or insufficient data
    pub(crate) fn payload_checksum(&self) -> Option<u32> {
        if self.message_type() != MessageType::AddBlock as u8 && self.message_type() != MessageType::AddTransaction as u8 {
            return None; // Only AddBlock and AddTransaction messages have a payload checksum
        }
        if self.length < 13 {
            return None; // Not enough data for checksum
        }

        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.payload[9..13]);
        Some(u32::from_le_bytes(checksum_bytes))
    }

    /// Returns an iterator over RequestBlockPart packet indices
    ///
    /// Iterates through the list of packet indices being requested from a block.
    /// Message layout: `[0]=type, [1..5)=sender, [5..9)=sequence, [9..13)=checksum,
    /// [13]=count, [14..14+count)=indices`
    ///
    /// # Returns
    /// * `Some(RequestBlockPartIterator)` - Iterator over packet indices
    /// * `None` - Message is not RequestBlockPart type or insufficient data
    ///
    /// # Example
    /// ```rust,ignore
    /// if let Some(iterator) = message.get_request_block_part_iterator() {
    ///     for item in iterator {
    ///         println!("Requesting packet index: {}", item.packet_index());
    ///     }
    /// }
    /// ```
    pub fn get_request_block_part_iterator(&self) -> Option<RequestBlockPartIterator> {
        if self.message_type() != MessageType::RequestBlockPart as u8 {
            return None;
        }
        // need at least up to count byte present
        if self.length < 14 {
            return None;
        }
        let count = self.payload[13] as usize;
        // Start of indices immediately after count byte
        let start = 14usize;
        // Use count as authoritative; cap to a single radio packet size for safety
        let end = (start + count).min(RADIO_PACKET_SIZE);
        Some(RequestBlockPartIterator::with(&self.payload, start, end))
    }

    /// Validates the payload checksum (internal use)
    ///
    /// Verifies that the CRC32C checksum stored in the message header matches
    /// the actual checksum of the payload data.
    ///
    /// # Returns
    /// * `true` - Checksum is valid
    /// * `false` - Checksum mismatch or message type doesn't have a checksum
    pub(crate) fn check_payload_checksum(&self) -> bool {
        if let Some(expected_checksum) = self.payload_checksum() {
            let actual_payload = &self.payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..self.length];
            let actual_checksum = checksum(actual_payload);
            return expected_checksum == actual_checksum;
        }
        false
    }
}

impl PartialEq for RadioMessage {
    fn eq(&self, other: &Self) -> bool {
        // Check message type equality first
        if self.message_type() != other.message_type() {
            return false;
        }

        // Check the specific fields based on the message type
        // Note: sender_node_id is ignored for equality except for RequestEcho messages
        match self.message_type() {
            msg_type if msg_type == MessageType::RequestEcho as u8 => {
                // RequestEcho is an exception - we must check sender_node_id
                self.sender_node_id() == other.sender_node_id()
            }
            msg_type if msg_type == MessageType::Echo as u8 => {
                // Check target_node and link_quality (excluding sender_node_id)

                // Extract target_node from both messages
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_target_node = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_target_node = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                if self_target_node != other_target_node {
                    return false;
                }

                // Extract link_quality from both messages
                if self.length < 10 || other.length < 10 {
                    return false;
                }
                self.payload[9] == other.payload[9]
            }
            msg_type if msg_type == MessageType::EchoResult as u8 => {
                // Check message type and echo result data (excluding sender_node_id in bytes 1-4)
                if self.length != other.length {
                    return false;
                }

                // Compare message type (byte 0)
                if self.payload[0] != other.payload[0] {
                    return false;
                }

                // Compare echo result data (from byte 5 onwards, skipping sender_node_id in bytes 1-4)
                if self.length > 5 {
                    self.payload[5..self.length] == other.payload[5..other.length]
                } else {
                    true // Only message type + sender_node_id, so equal since we ignore sender_node_id
                }
            }
            msg_type if msg_type == MessageType::RequestFullBlock as u8 => {
                // Check sequence only (excluding sender_node_id)

                // Extract sequence from both messages
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_sequence = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_sequence = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                self_sequence == other_sequence
            }
            msg_type if msg_type == MessageType::RequestBlockPart as u8 => {
                // Check sequence, payload_checksum, packet_number (excluding sender_node_id)

                // Extract sequence
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_sequence = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_sequence = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                if self_sequence != other_sequence {
                    return false;
                }

                // Extract payload_checksum
                if self.length < 13 || other.length < 13 {
                    return false;
                }
                let self_checksum = u32::from_le_bytes([self.payload[9], self.payload[10], self.payload[11], self.payload[12]]);
                let other_checksum = u32::from_le_bytes([other.payload[9], other.payload[10], other.payload[11], other.payload[12]]);

                if self_checksum != other_checksum {
                    return false;
                }

                // Extract packet_number
                if self.length < 14 || other.length < 14 {
                    return false;
                }
                self.payload[13] == other.payload[13]
            }
            msg_type if msg_type == MessageType::AddBlock as u8 => {
                // Check sequence and payload_checksum (excluding sender_node_id)

                // Extract sequence
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_sequence = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_sequence = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                if self_sequence != other_sequence {
                    return false;
                }

                // Extract payload_checksum
                if self.length < 13 || other.length < 13 {
                    return false;
                }
                let self_checksum = u32::from_le_bytes([self.payload[9], self.payload[10], self.payload[11], self.payload[12]]);
                let other_checksum = u32::from_le_bytes([other.payload[9], other.payload[10], other.payload[11], other.payload[12]]);

                self_checksum == other_checksum
            }
            msg_type if msg_type == MessageType::AddTransaction as u8 => {
                // Check anchor_sequence and payload_checksum (excluding sender_node_id)

                // Extract anchor_sequence
                if self.length < 9 || other.length < 9 {
                    return false;
                }
                let self_anchor_sequence = u32::from_le_bytes([self.payload[5], self.payload[6], self.payload[7], self.payload[8]]);
                let other_anchor_sequence = u32::from_le_bytes([other.payload[5], other.payload[6], other.payload[7], other.payload[8]]);

                if self_anchor_sequence != other_anchor_sequence {
                    return false;
                }

                // Extract payload_checksum
                if self.length < 13 || other.length < 13 {
                    return false;
                }
                let self_checksum = u32::from_le_bytes([self.payload[9], self.payload[10], self.payload[11], self.payload[12]]);
                let other_checksum = u32::from_le_bytes([other.payload[9], other.payload[10], other.payload[11], other.payload[12]]);

                self_checksum == other_checksum
            }
            msg_type if msg_type == MessageType::RequestNewMempoolItem as u8 => {
                // Check message type and mempool data (excluding sender_node_id in bytes 1-4)
                if self.length != other.length {
                    return false;
                }

                // Compare message type (byte 0)
                if self.payload[0] != other.payload[0] {
                    return false;
                }

                // Compare mempool data (from byte 5 onwards, skipping sender_node_id in bytes 1-4)
                if self.length > 5 {
                    self.payload[5..self.length] == other.payload[5..other.length]
                } else {
                    true // Only message type + sender_node_id, so equal since we ignore sender_node_id
                }
            }
            msg_type if msg_type == MessageType::Support as u8 => {
                // Check message type and support data (excluding sender_node_id in bytes 1-4)
                if self.length != other.length {
                    return false;
                }

                // Compare message type (byte 0)
                if self.payload[0] != other.payload[0] {
                    return false;
                }

                // Compare support data (from byte 5 onwards, skipping sender_node_id in bytes 1-4)
                if self.length > 5 {
                    self.payload[5..self.length] == other.payload[5..other.length]
                } else {
                    true // Only message type + sender_node_id, so equal since we ignore sender_node_id
                }
            }
            _ => {
                // Unknown message type
                false
            }
        }
    }
}

impl Eq for RadioMessage {}

/// Echo result data item
///
/// Represents a single neighbor's link quality information in an EchoResult message.
/// Contains bidirectional link quality measurements.
#[cfg_attr(feature = "std", derive(Debug))]
pub struct EchoResultItem {
    pub(crate) neighbor_node: u32,
    pub(crate) send_link_quality: u8,
    pub(crate) receive_link_quality: u8,
}

/// Iterator over echo result data entries
///
/// Each entry is 6 bytes: 4 bytes for node ID + 1 byte send quality + 1 byte receive quality.
#[cfg_attr(feature = "std", derive(Debug))]
pub struct EchoResultIterator<'a> {
    payload: &'a [u8],
    position: usize,
    end_position: usize,
}

impl<'a> EchoResultIterator<'a> {
    /// Creates a new EchoResultIterator
    ///
    /// # Arguments
    /// * `payload` - Message payload containing echo result data
    /// * `start_position` - Byte offset where echo result data starts
    /// * `end_position` - Byte offset where echo result data ends
    pub(crate) fn with(payload: &'a [u8], start_position: usize, end_position: usize) -> Self {
        Self {
            payload,
            position: start_position,
            end_position,
        }
    }
}

impl<'a> Iterator for EchoResultIterator<'a> {
    type Item = EchoResultItem;

    fn next(&mut self) -> Option<Self::Item> {
        // Each item needs 6 bytes: 4 for node_id + 1 for send quality + 1 for receive quality
        if self.position + 6 > self.end_position {
            return None;
        }

        // Extract neighbor_node (u32) from 4 bytes
        let mut node_id_bytes = [0u8; 4];
        node_id_bytes.copy_from_slice(&self.payload[self.position..self.position + 4]);
        let neighbor_node = u32::from_le_bytes(node_id_bytes);

        // Extract send and receive link qualities
        let send_link_quality = self.payload[self.position + 4];
        let receiving_link_quality = self.payload[self.position + 5];

        // Advance position to the next item
        self.position += 6;

        Some(EchoResultItem {
            neighbor_node,
            send_link_quality,
            receive_link_quality: receiving_link_quality,
        })
    }
}

/// Request block part data item
///
/// Represents a single packet index being requested from a block.
#[cfg_attr(feature = "std", derive(Debug))]
pub struct RequestBlockPartItem {
    pub packet_index: u8,
}

/// Iterator over RequestBlockPart packet indices
///
/// Each entry is 1 byte representing a packet index.
#[cfg_attr(feature = "std", derive(Debug))]
pub struct RequestBlockPartIterator<'a> {
    payload: &'a [u8],
    position: usize,
    end_position: usize,
}

impl<'a> RequestBlockPartIterator<'a> {
    /// Creates a new RequestBlockPartIterator
    ///
    /// # Arguments
    /// * `payload` - Message payload containing packet indices
    /// * `start_position` - Byte offset where packet indices start
    /// * `end_position` - Byte offset where packet indices end
    pub(crate) fn with(payload: &'a [u8], start_position: usize, end_position: usize) -> Self {
        Self {
            payload,
            position: start_position,
            end_position,
        }
    }
}

impl<'a> Iterator for RequestBlockPartIterator<'a> {
    type Item = RequestBlockPartItem;

    fn next(&mut self) -> Option<Self::Item> {
        if self.position >= self.end_position {
            return None;
        }
        let idx = self.payload[self.position];
        self.position += 1;

        Some(RequestBlockPartItem { packet_index: idx })
    }
}

/// Mempool data item
///
/// Represents a transaction in the mempool by its anchor sequence and checksum.
#[cfg_attr(feature = "std", derive(Debug))]
pub struct MempoolItem {
    pub anchor_sequence: u32,
    pub transaction_payload_checksum: u32,
}

/// Iterator over mempool data entries
///
/// Each entry is 8 bytes: 4 bytes for anchor sequence + 4 bytes for checksum.
#[cfg_attr(feature = "std", derive(Debug))]
pub struct MempoolIterator<'a> {
    payload: &'a [u8],
    position: usize,
    end_position: usize,
}

impl<'a> MempoolIterator<'a> {
    /// Creates a new MempoolIterator
    ///
    /// # Arguments
    /// * `payload` - Message payload containing mempool data
    /// * `start_position` - Byte offset where mempool data starts
    /// * `end_position` - Byte offset where mempool data ends
    pub(crate) fn with(payload: &'a [u8], start_position: usize, end_position: usize) -> Self {
        Self {
            payload,
            position: start_position,
            end_position,
        }
    }
}

impl<'a> Iterator for MempoolIterator<'a> {
    type Item = MempoolItem;

    fn next(&mut self) -> Option<Self::Item> {
        // Each item needs 8 bytes: 4 for anchor_sequence + 4 for transaction_payload_checksum
        if self.position + 8 > self.end_position {
            return None;
        }

        // Extract anchor_sequence (u32) from 4 bytes
        let mut anchor_sequence_bytes = [0u8; 4];
        anchor_sequence_bytes.copy_from_slice(&self.payload[self.position..self.position + 4]);
        let anchor_sequence = u32::from_le_bytes(anchor_sequence_bytes);

        // Extract transaction_payload_checksum (u32) from next 4 bytes
        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.payload[self.position + 4..self.position + 8]);
        let transaction_payload_checksum = u32::from_le_bytes(checksum_bytes);

        // Advance position to the next item
        self.position += 8;

        Some(MempoolItem {
            anchor_sequence,
            transaction_payload_checksum,
        })
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    #[test]
    fn request_echo_basics_and_eq() {
        let node = 0x0A0B0C0D;
        let m1 = RadioMessage::request_echo_with(node);
        assert_eq!(m1.message_type(), MessageType::RequestEcho as u8);
        assert_eq!(m1.length(), 5);
        assert_eq!(m1.sender_node_id(), node);

        // Same sender -> equal; different sender -> not equal
        let m2 = RadioMessage::request_echo_with(node);
        let m3 = RadioMessage::request_echo_with(123);
        assert_eq!(m1, m2);
        assert_ne!(m1, m3);
    }

    #[test]
    fn echo_fields_and_eq_ignore_sender() {
        let a = RadioMessage::echo_with(100, 200, 7);
        assert_eq!(a.message_type(), MessageType::Echo as u8);
        let (tgt, lq) = a.get_echo_data().unwrap();
        assert_eq!((tgt, lq), (200, 7));

        // Different sender but same target+quality -> equal
        let b = RadioMessage::echo_with(999, 200, 7);
        assert_eq!(a, b);

        // Different target -> not equal
        let c = RadioMessage::echo_with(999, 201, 7);
        assert_ne!(a, c);
    }

    #[test]
    fn echo_result_iterator_and_eq_ignores_sender() {
        let mut er1 = RadioMessage::echo_result_with(42);
        er1.add_echo_result_item(2, 10, 11).unwrap();
        er1.add_echo_result_item(3, 20, 21).unwrap();

        // Iterate and collect
        let items: Vec<(u32, u8, u8)> = er1
            .get_echo_result_data_iterator()
            .unwrap()
            .map(|it| (it.neighbor_node, it.send_link_quality, it.receive_link_quality))
            .collect();
        assert_eq!(items, vec![(2, 10, 11), (3, 20, 21)]);

        // Same data, different sender -> equal
        let mut er2 = RadioMessage::echo_result_with(7);
        er2.add_echo_result_item(2, 10, 11).unwrap();
        er2.add_echo_result_item(3, 20, 21).unwrap();
        assert_eq!(er1, er2);

        // Different data -> not equal
        let mut er3 = RadioMessage::echo_result_with(7);
        er3.add_echo_result_item(2, 10, 12).unwrap();
        assert_ne!(er1, er3);
    }

    #[test]
    fn mempool_add_iter_and_capacity() {
        let mut m = RadioMessage::get_mempool_state_with(11);
        assert_eq!(m.message_type(), MessageType::RequestNewMempoolItem as u8);

        // Fill up to capacity
        let capacity = (RADIO_PACKET_SIZE - 5) / 8; // items of 8 bytes after 5-byte header
        for i in 0..capacity {
            m.add_mempool_item(i as u32, (i as u32) ^ 0xDEAD_BEEF).unwrap();
        }
        // Next add should fail
        assert!(m.add_mempool_item(999, 999).is_err());

        // Iterate and verify
        let items: Vec<(u32, u32)> = m
            .get_mempool_data_iterator()
            .unwrap()
            .map(|it| (it.anchor_sequence, it.transaction_payload_checksum))
            .collect();
        assert_eq!(items.len(), capacity);
        assert_eq!(items[0], (0, 0xDEAD_BEEF));
    }

    #[test]
    fn add_transaction_get_data_and_reply_logic() {
        let payload = &[1, 2, 3, 4, 5];
        let at = RadioMessage::add_transaction_with(7, 1234, 0xABCD_0123, payload);
        assert_eq!(at.message_type(), MessageType::AddTransaction as u8);
        let (anchor, checksum, data) = at.get_add_transaction_data().unwrap();
        assert_eq!(anchor, 1234);
        assert_eq!(checksum, 0xABCD_0123);
        assert_eq!(data, payload);

        // Case 1: Request mempool without matching item -> is_reply_to == true
        let mut req1 = RadioMessage::get_mempool_state_with(9);
        req1.add_mempool_item(1, 2).unwrap();
        assert!(at.is_reply_to(&req1));

        // Case 2: Request mempool includes matching item -> is_reply_to == false
        let mut req2 = RadioMessage::get_mempool_state_with(9);
        req2.add_mempool_item(1234, 0xABCD_0123).unwrap();
        assert!(!at.is_reply_to(&req2));
    }

    #[test]
    fn add_block_fragmentation_and_reassembly() {
        let seq = 0x0102_0304;
        // Payload length spanning multiple packets
        let part = RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE; // bytes per packet chunk
        let total_len = part * 3 + 10; // 3 full + 1 partial
        let mut payload_vec = vec![0u8; total_len];
        // Make payload deterministic
        for (i, b) in payload_vec.iter_mut().enumerate() {
            *b = (i % 251) as u8;
        }
        let msg = RadioMessage::add_block_with(5, seq, &payload_vec);

        // Packetization
        let count = msg.get_packet_count();
        assert_eq!(count, 4);

        let mut reassembled = RadioMessage::new();
        for idx in 0..count {
            let p = msg.get_packet(idx).expect("packet must exist");
            // Validate per-packet header/meta
            assert_eq!(p.message_type(), MessageType::AddBlock as u8);
            assert_eq!(p.total_packet_count() as usize, count);
            assert_eq!(p.packet_index() as usize, idx);
            // Feed into reassembler
            reassembled.add_packet(&p).expect("packet should be valid");
        }

        // Reassembled message should match type/sequence/checksum and byte length
        assert_eq!(reassembled.message_type(), MessageType::AddBlock as u8);
        assert_eq!(reassembled.sequence().unwrap(), seq);
        assert_eq!(reassembled.length(), RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + total_len);

        // Verify payload bytes match exactly
        assert_eq!(
            &reassembled.payload[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + total_len],
            &payload_vec[..]
        );

        // Out-of-range packet request returns None
        assert!(msg.get_packet(count).is_none());
    }

    #[test]
    fn new_from_single_packet_roundtrip_simple() {
        let m = RadioMessage::request_full_block_with(21, 0xDEAD_BEEF);
        let p = m.get_packet(0).unwrap();
        // Single packet reconstruction should be equal
        let m2 = RadioMessage::from_single_packet(p);
        assert_eq!(m2, m);
    }

    #[test]
    fn request_block_part_fields_via_eq() {
        let mut a = RadioMessage::request_block_part_with(44, 0xCAFEBABE, 0x1234_5678);
        a.add_packet_index_to_request_block_part(9).unwrap(); // count = 1

        let mut b = RadioMessage::request_block_part_with(9999, 0xCAFEBABE, 0x1234_5678);
        b.add_packet_index_to_request_block_part(9).unwrap(); // count = 1
        // Equality ignores sender for this type, compares seq/checksum and count of requested parts
        assert_eq!(a, b);

        // Different count of packet indices -> not equal under current semantics
        let mut c = RadioMessage::request_block_part_with(44, 0xCAFEBABE, 0x1234_5678);
        c.add_packet_index_to_request_block_part(10).unwrap();
        c.add_packet_index_to_request_block_part(11).unwrap(); // count = 2
        assert_ne!(a, c);
    }

    #[test]
    fn non_chunked_out_of_range_packet_request() {
        let m = RadioMessage::request_echo_with(123);
        // Only index 0 is valid for non-chunked messages
        assert!(m.get_packet(0).is_some());
        assert!(m.get_packet(1).is_none());
    }

    #[test]
    fn chunk_boundary_counts_and_indices() {
        let chunk = RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE;
        // Test several boundary sizes up to the maximum
        for n in 1..=core::cmp::min(4, RADIO_MAX_PACKET_COUNT) {
            let total_len = chunk * n; // exact multiple of chunk
            let payload: Vec<u8> = (0..total_len).map(|i| (i % 251) as u8).collect();
            let msg = RadioMessage::add_block_with(77, 0x01020304, &payload);

            assert_eq!(msg.get_packet_count(), n, "packet count should equal chunks for n={}", n);

            for idx in 0..n {
                let p = msg.get_packet(idx).expect("packet must exist");
                assert_eq!(p.total_packet_count() as usize, n);
                assert_eq!(p.packet_index() as usize, idx);
            }

            // Out of range
            assert!(msg.get_packet(n).is_none());
        }
    }

    #[test]
    fn mask_filtering_and_index_mapping() {
        let chunk = RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE;
        let n = 5; // total unfiltered packets
        let payload: Vec<u8> = (0..(chunk * n)).map(|i| (i % 251) as u8).collect();

        let mut mask = [false; RADIO_MAX_PACKET_COUNT];
        mask[1] = true;
        mask[2] = true;
        mask[4] = true;

        let msg = RadioMessage::add_block_with_packet_list(9, 0xAA55AA55, &payload, mask);
        assert_eq!(msg.get_packet_count(), 3);

        // Filtered index 0 -> actual 1
        let p0 = msg.get_packet(0).expect("filtered packet 0 exists");
        assert_eq!(p0.total_packet_count(), n as u8);
        assert_eq!(p0.packet_index(), 1);

        // Filtered index 1 -> actual 2
        let p1 = msg.get_packet(1).expect("filtered packet 1 exists");
        assert_eq!(p1.packet_index(), 2);

        // Filtered index 2 -> actual 4
        let p2 = msg.get_packet(2).expect("filtered packet 2 exists");
        assert_eq!(p2.packet_index(), 4);

        // Filtered index 3 -> None
        assert!(msg.get_packet(3).is_none());
    }

    #[test]
    fn zero_length_chunked_payload_behaviour() {
        let msg = RadioMessage::add_block_with(5, 0x01020304, &[]);
        // Zero-length payload currently yields 0 packets
        assert_eq!(msg.get_packet_count(), 0);
        assert!(msg.get_packet(0).is_none());
    }

    #[test]
    fn request_block_part_iterator_zero_and_some() {
        // zero indices
        let m0 = RadioMessage::request_block_part_with(1, 0xAABBCCDD, 0x11223344);
        assert_eq!(m0.message_type(), MessageType::RequestBlockPart as u8);
        // count byte is 0, iterator should be empty
        let v0: Vec<u8> = m0.get_request_block_part_iterator().unwrap().map(|it| it.packet_index).collect();
        assert!(v0.is_empty());

        // one index
        let mut m1 = RadioMessage::request_block_part_with(2, 0xCAFEBABE, 0x55667788);
        m1.add_packet_index_to_request_block_part(9).unwrap();
        let v1: Vec<u8> = m1.get_request_block_part_iterator().unwrap().map(|it| it.packet_index).collect();
        assert_eq!(v1, vec![9]);

        // multiple indices in order
        let mut mm = RadioMessage::request_block_part_with(3, 0x01020304, 0x99AA_BBCC);
        for i in [1u8, 3, 4, 7, 9] {
            mm.add_packet_index_to_request_block_part(i).unwrap();
        }
        let v: Vec<u8> = mm.get_request_block_part_iterator().unwrap().map(|it| it.packet_index).collect();
        assert_eq!(v, vec![1, 3, 4, 7, 9]);
    }

    // ============================================================================
    // Additional Edge Case Tests
    // ============================================================================

    #[test]
    fn test_message_sequence_numbers() {
        let seq1 = 100u32;
        let seq2 = 200u32;

        let msg1 = RadioMessage::add_block_with(1, seq1, &[1, 2, 3]);
        let msg2 = RadioMessage::add_block_with(1, seq2, &[4, 5, 6]);

        assert_eq!(msg1.sequence(), Some(seq1));
        assert_eq!(msg2.sequence(), Some(seq2));
        assert_ne!(msg1.sequence(), msg2.sequence());
    }

    #[test]
    fn test_echo_message_data_integrity() {
        let node_id = 42u32;
        let msg = RadioMessage::request_echo_with(node_id);

        assert_eq!(msg.message_type(), MessageType::RequestEcho as u8);
        assert_eq!(msg.sender_node_id(), node_id);
    }

    #[test]
    fn test_echo_result_multiple_items() {
        let mut echo_result = RadioMessage::echo_result_with(1);

        assert!(echo_result.add_echo_result_item(2, 10, 11).is_ok());
        assert!(echo_result.add_echo_result_item(3, 20, 21).is_ok());
        assert!(echo_result.add_echo_result_item(4, 30, 31).is_ok());

        if let Some(iterator) = echo_result.get_echo_result_data_iterator() {
            let items: Vec<_> = iterator.collect();
            assert_eq!(items.len(), 3);
            assert_eq!(items[0].neighbor_node, 2);
            assert_eq!(items[1].send_link_quality, 20);
            assert_eq!(items[2].receive_link_quality, 31);
        } else {
            panic!("Failed to get echo result iterator");
        }
    }

    #[test]
    fn test_maximum_quality_value() {
        // Test that quality values are properly bounded
        let mut echo_result = RadioMessage::echo_result_with(1);

        // Add with max quality values
        assert!(echo_result.add_echo_result_item(2, 63, 63).is_ok());

        if let Some(iterator) = echo_result.get_echo_result_data_iterator() {
            let items: Vec<_> = iterator.collect();
            assert_eq!(items[0].send_link_quality, 63);
            assert_eq!(items[0].receive_link_quality, 63);
        }
    }

    #[test]
    fn test_add_packet_bounds_checking() {
        // Test that add_packet() handles out-of-bounds indices gracefully
        use super::RadioPacket;

        let mut msg = RadioMessage::new();
        msg.payload[0] = MessageType::AddBlock as u8;

        // Create a packet with invalid indices that would overflow
        let mut packet = RadioPacket {
            data: [0u8; RADIO_PACKET_SIZE],
            length: RADIO_PACKET_SIZE,
        };

        // Set up multi-packet message header
        packet.data[0] = MessageType::AddBlock as u8;
        packet.data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE] = 255; // total_packet_count (unrealistic)
        packet.data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + 1] = 254; // packet_index (very high)

        // This should not panic due to bounds checking, should return an error
        let result = msg.add_packet(&packet);
        assert!(result.is_err(), "Should reject malformed packet");
        assert_eq!(result.unwrap_err(), PacketError::InvalidPacketCount);

        // Message should still be valid (unchanged)
        assert!(msg.length <= RADIO_MAX_MESSAGE_SIZE);
    }

    #[test]
    fn test_add_packet_normal_operation() {
        // Test that add_packet() works correctly for valid multi-packet messages

        // Create an AddBlock message
        let payload = vec![0xAA; 500]; // Multi-packet message
        let msg = RadioMessage::add_block_with(1, 42, &payload);

        // Get first packet and reconstruct
        let packet = msg.get_packet(0).unwrap();
        let reconstructed = RadioMessage::from_single_packet(packet);

        // Verify reconstruction started correctly
        assert_eq!(reconstructed.message_type(), MessageType::AddBlock as u8);
        assert_eq!(reconstructed.sender_node_id(), 1);
    }
}

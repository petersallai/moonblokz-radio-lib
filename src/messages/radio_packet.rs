//! # Radio Packet Module
//!
//! Low-level packet structure for radio transmission and reception.
//!
//! ## Architecture
//!
//! RadioPacket represents the wire format for radio communication. Each packet contains:
//! - Raw data buffer of fixed size (RADIO_PACKET_SIZE)
//! - Actual data length
//!
//! ## Packet Structure
//!
//! All packets start with a common header:
//! - Byte 0: Message type
//! - Bytes 1-4: Sender node ID
//!
//! For multi-packet messages (AddBlock, AddTransaction), additional fields follow:
//! - Bytes 5-8: Sequence number
//! - Bytes 9-12: Payload checksum
//! - Byte 13: Total packet count (in multi-packet header section)
//! - Byte 14: Packet index (in multi-packet header section, 0..N-1)
//!
//! ## Design Considerations
//!
//! - **Public Fields**: `data` and `length` are public for performance-critical operations
//!   where direct buffer access is necessary (e.g., radio hardware interfaces)
//! - **No Copy on Send**: Packets are designed to be used in-place without additional copying
//! - **Fixed Size**: All packets are the same size for predictable memory usage

use super::radio_message::MessageType;
use crate::{RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE, RADIO_MULTI_PACKET_PACKET_HEADER_SIZE, RADIO_PACKET_SIZE};

/// Low-level packet structure for radio transmission
///
/// Represents the wire-format data transmitted and received over the radio.
/// Contains a fixed-size buffer and actual data length.
///
/// # Public Fields
/// The `data` and `length` fields are intentionally public for performance-critical
/// zero-copy operations with radio hardware interfaces.
///
/// # Example
/// ```rust
/// use moonblokz_radio_lib::RadioPacket;
///
/// // Packets are typically created from RadioMessage or received from radio
/// // let packet = message.iter_packets().next().unwrap();
/// // let msg_type = packet.message_type();
/// ```
#[derive(Clone)]
#[cfg_attr(feature = "std", derive(Debug))]
pub struct RadioPacket {
    /// Raw packet data buffer of fixed size
    pub data: [u8; RADIO_PACKET_SIZE],

    /// Actual number of valid bytes in the data buffer
    pub length: usize,
}

impl RadioPacket {
    /// Extracts the message type byte from the packet header
    ///
    /// Message type is always at byte 0 of the packet.
    ///
    /// # Returns
    /// The message type as a u8 (compare with MessageType enum values)
    pub fn message_type(&self) -> u8 {
        self.data[0]
    }

    /// Extracts the sender node ID from the packet header
    ///
    /// Sender node ID is stored in bytes 1-4 as a little-endian u32.
    ///
    /// # Returns
    /// The sender's node ID
    pub fn sender_node_id(&self) -> u32 {
        let mut node_id_bytes = [0u8; 4];
        node_id_bytes.copy_from_slice(&self.data[1..5]);
        u32::from_le_bytes(node_id_bytes)
    }

    /// Extracts the sequence number from applicable message types
    ///
    /// Sequence numbers are present in AddBlock, RequestFullBlock, Support,
    /// and RequestBlockPart message types. They're stored in bytes 5-8 as
    /// a little-endian u32.
    ///
    /// # Returns
    /// * `Some(sequence)` if the message type supports sequences
    /// * `None` for message types without sequence numbers
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
        sequence_bytes.copy_from_slice(&self.data[5..9]);
        Some(u32::from_le_bytes(sequence_bytes))
    }

    /// Extracts the payload checksum from applicable message types (internal use)
    ///
    /// Payload checksums are present in AddBlock, RequestFullBlock, Support,
    /// and RequestBlockPart message types. Used for duplicate detection and
    /// payload integrity verification.
    ///
    /// # Returns
    /// * `Some(checksum)` if the message type includes a payload checksum
    /// * `None` for message types without checksums
    pub(crate) fn payload_checksum(&self) -> Option<u32> {
        if self.message_type() != MessageType::AddBlock as u8
            && self.message_type() != MessageType::RequestFullBlock as u8
            && self.message_type() != MessageType::Support as u8
            && self.message_type() != MessageType::RequestBlockPart as u8
        {
            return None; // Only AddBlock and AddTransaction messages have a sequence
        }

        // Extract the payload checksum from the message payload
        if self.length < 13 {
            return None; // Not enough data for payload checksum
        }
        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.data[9..13]);
        Some(u32::from_le_bytes(checksum_bytes))
    }

    /// Returns the total number of packets in this message
    ///
    /// For multi-packet messages (AddBlock, AddTransaction), extracts the total
    /// packet count from the multi-packet header. For single-packet messages,
    /// always returns 1.
    ///
    /// # Returns
    /// Total packet count (1 for single-packet messages, >1 for multi-packet)
    pub fn total_packet_count(&self) -> u8 {
        let message_type = self.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            if self.length < RADIO_MULTI_PACKET_PACKET_HEADER_SIZE {
                return 0; // Not enough data for packet count
            }
            self.data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]
        } else {
            1 // For other message types, always 1 packet
        }
    }

    /// Returns this packet's index within a multi-packet message
    ///
    /// For multi-packet messages, returns the 0-based index of this packet.
    /// For single-packet messages, always returns 0.
    ///
    /// # Returns
    /// Packet index (0-based)
    pub fn packet_index(&self) -> u8 {
        let message_type = self.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            if self.length < RADIO_MULTI_PACKET_PACKET_HEADER_SIZE {
                return 0; // Not enough data for packet index
            }
            self.data[RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE + 1]
        } else {
            0 // For other message types, always 0
        }
    }

    /// Checks if this packet belongs to the same message as the provided header (internal use)
    ///
    /// Compares message type and header fields to determine if two packets are
    /// part of the same multi-packet message. Used for packet assembly.
    ///
    /// # Arguments
    /// * `other_header` - Header bytes from another packet to compare with
    ///
    /// # Returns
    /// true if the packets belong to the same message, false otherwise
    pub(crate) fn same_message(&self, other_header: &[u8]) -> bool {
        if self.message_type() != other_header[0] {
            return false;
        }

        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            if self.length < RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE || other_header.len() < RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE {
                return false;
            }

            self.data[5..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE] == other_header[5..RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE]
        } else {
            false
        }
    }
}

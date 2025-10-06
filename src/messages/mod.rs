//! # Messages Module
//!
//! This module provides the core data structures for radio communication messages and packets.
//!
//! ## Architecture
//!
//! The messages module is organized into two main components:
//!
//! - **RadioMessage**: High-level message abstraction that can span multiple packets
//! - **RadioPacket**: Low-level packet structure for radio transmission
//!
//! ## Key Types
//!
//! - `RadioMessage`: Application-level messages with support for fragmentation
//! - `RadioPacket`: Wire-format packets for radio transmission
//! - `MessageType`: Enumeration of supported message types (Echo, Block, Transaction, etc.)
//! - Iterators: For traversing echo results, mempool items, and block part requests
//!
//! ## Message Fragmentation
//!
//! Large messages are automatically fragmented into multiple packets for transmission.
//! The receiving side reconstructs the original message from received packets.

// Module declarations
pub mod radio_message;
pub mod radio_packet;

// Re-export public types for convenient access
pub use radio_message::{EchoResultItem, EchoResultIterator, MessageType, RadioMessage};
pub use radio_packet::RadioPacket;

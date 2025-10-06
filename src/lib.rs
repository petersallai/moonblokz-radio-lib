//! # MoonBlokz Radio Library
//!
//! A no_std compatible radio communication library for building decentralized mesh networks.
//! This library provides the core infrastructure for packet-based radio communication with
//! support for multi-packet messages, relay management, and various radio hardware backends.
//!
//! ## Architecture
//!
//! The library is organized around several key components that work together to provide
//! reliable radio communication:
//!
//! - **Radio Device Layer**: Abstraction over different radio hardware (LoRa SX1262, simulator, echo)
//! - **Message/Packet Layer**: Data structures for messages and packets with fragmentation support
//! - **TX Scheduler**: Manages transmission timing and queuing
//! - **RX Handler**: Processes received packets and reconstructs multi-packet messages
//! - **Relay Manager**: Handles network topology and message relaying
//! - **Wait Pool**: Manages pending messages awaiting responses or relay
//!
//! ## Features
//!
//! - `radio-device-lora-sx1262`: LoRa SX1262 hardware support
//! - `radio-device-simulator`: Network simulator for testing
//! - `radio-device-echo`: Simple echo device for testing
//! - `std`: Standard library support (disabled for embedded targets)
//! - `embedded`: Embedded system support with static allocation
//!
//! ## Message Flow
//!
//! 1. Application creates a RadioMessage
//! 2. Message is queued via RadioCommunicationManager
//! 3. TX Scheduler breaks message into packets and schedules transmission
//! 4. Radio device transmits packets
//! 5. Remote node receives packets via RX Handler
//! 6. RX Handler reconstructs message and delivers to application
//! 7. Relay Manager may relay messages to extend network reach
//!
//! ## Security
//!
//! The MoonBlokz Radio Library is not implement any security features such as encryption or authentication,
//! because it is designed as a communication layer for a blockchain network. All authorization and integrity
//! checks are performed by the higher-level blockchain protocols.
//!
//! ## Reliability
//!
//! The library not handles retransmission of lost packets - this is managed by the higher-level blockchain protocol.
//! Also the protocol not depends on acknowledgments or handshakes to confirm receipt of messages. The network layer mostly
//! delivers messages to all nodes and the higher-level protocols handles the rare exceptions. Using this approach
//! minimizes radio traffic and maximizes network efficiency, which is critical on the LoRa networks.

#![cfg_attr(not(feature = "std"), no_std)]
#![allow(async_fn_in_trait)]

#[cfg(any(
    all(feature = "radio-device-echo", any(feature = "radio-device-lora-sx1262", feature = "radio-device-simulator")),
    all(feature = "radio-device-lora-sx1262", any(feature = "radio-device-echo", feature = "radio-device-simulator")),
    all(feature = "radio-device-simulator", any(feature = "radio-device-echo", feature = "radio-device-lora-sx1262")),
))]
compile_error!("Only one radio implementation feature can be enabled at a time");

#[cfg(any(
    all(feature = "memory-config-small", any(feature = "memory-config-medium", feature = "memory-config-large")),
    all(feature = "memory-config-medium", any(feature = "memory-config-small", feature = "memory-config-large")),
    all(feature = "memory-config-large", any(feature = "memory-config-small", feature = "memory-config-medium")),
))]
compile_error!("Only one memory configuration feature can be enabled at a time");

#[cfg(all(
    not(test),
    not(any(feature = "radio-device-echo", feature = "radio-device-lora-sx1262", feature = "radio-device-simulator"))
))]
compile_error!("At least one radio implementation feature must be enabled");

#[cfg(all(
    not(test),
    not(any(feature = "memory-config-small", feature = "memory-config-medium", feature = "memory-config-large"))
))]
compile_error!("At least one memory configuration feature must be enabled");

#[cfg(feature = "radio-device-lora-sx1262")]
pub mod radio_device_lora_sx1262;

#[cfg(feature = "radio-device-echo")]
pub mod radio_device_echo;

#[cfg(feature = "radio-device-simulator")]
pub mod radio_device_simulator;

#[cfg(feature = "radio-device-lora-sx1262")]
use crate::radio_device_lora_sx1262::RadioDevice;
#[cfg(feature = "radio-device-lora-sx1262")]
use crate::radio_device_lora_sx1262::radio_device_task;

#[cfg(feature = "radio-device-echo")]
use crate::radio_device_echo::RadioDevice;
#[cfg(feature = "radio-device-echo")]
use crate::radio_device_echo::radio_device_task;

#[cfg(feature = "radio-device-simulator")]
use crate::radio_device_simulator::RadioDevice;
#[cfg(feature = "radio-device-simulator")]
use crate::radio_device_simulator::radio_device_task;

use crate::tx_scheduler::tx_scheduler_task;
use embassy_executor::Spawner;
mod messages;
mod relay_manager;
mod rx_handler;
mod tx_scheduler;
pub mod wait_pool;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use log::log;
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

// Re-export types from messages module
pub use messages::{EchoResultItem, EchoResultIterator, MessageType, RadioMessage, RadioPacket};

/// Size of the header for multi-packet messages (13 bytes)
///
/// This header contains metadata for reconstructing the full message from packets,
/// including sequence numbers, total packet count, and payload information.
const RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE: usize = 13;

/// Size of the header for individual packets within multi-packet messages (15 bytes)
///
/// Each packet carries routing and fragmentation metadata including sender ID,
/// message type, packet index, and CRC information.
const RADIO_MULTI_PACKET_PACKET_HEADER_SIZE: usize = 15;

// Hardware dependent constants that affect compatibility between nodes
// Changing these values will make nodes incompatible with each other

/// Maximum size of a single radio packet in bytes
///
/// This is the wire-format size including all headers and payload data.
/// **Compatibility critical**: All nodes in the network must use the same value.
pub const RADIO_PACKET_SIZE: usize = 215;

/// Maximum size of a complete message payload in bytes
///
/// Messages larger than a single packet are automatically fragmented.
/// Must be calculated as: multiple of (RADIO_PACKET_SIZE - RADIO_MULTI_PACKET_PACKET_HEADER_SIZE) + RADIO_MULTI_PACKET_MESSAGE_HEADER_SIZE
/// **Compatibility critical**: All nodes in the network must use the same value.
pub const RADIO_MAX_MESSAGE_SIZE: usize = 2013;

/// Maximum number of packets a single message can be fragmented into
///
/// Calculated via ceiling division to ensure we can accommodate RADIO_MAX_MESSAGE_SIZE.
/// **Compatibility critical**: Derived from RADIO_PACKET_SIZE and RADIO_MAX_MESSAGE_SIZE.
pub const RADIO_MAX_PACKET_COUNT: usize = (RADIO_MAX_MESSAGE_SIZE + RADIO_PACKET_SIZE - 1) / RADIO_PACKET_SIZE;

/// Compile-time assertion that packet count fits in a u8
const _: () = assert!(RADIO_MAX_PACKET_COUNT <= 255, "RADIO_MAX_PACKET_COUNT must fit in a u8");

// Hardware dependent constants that only affect efficiency, not compatibility
// These can be tuned per node without breaking interoperability

/// Size of the connection quality matrix (NxN grid for node connections)
///
#[cfg(feature = "memory-config-small")]
const CONNECTION_MATRIX_SIZE: usize = 10;

#[cfg(feature = "memory-config-medium")]
const CONNECTION_MATRIX_SIZE: usize = 30;

#[cfg(feature = "memory-config-large")]
const CONNECTION_MATRIX_SIZE: usize = 100;

/// Size of the buffer for incoming packets awaiting assembly
///
/// Larger values allow more concurrent multi-packet message reconstructions
/// but consume more memory.

#[cfg(feature = "memory-config-small")]
const INCOMING_PACKET_BUFFER_SIZE: usize = 20;

#[cfg(feature = "memory-config-medium")]
const INCOMING_PACKET_BUFFER_SIZE: usize = 30;

#[cfg(feature = "memory-config-large")]
const INCOMING_PACKET_BUFFER_SIZE: usize = 50;

/// Number of messages that can wait for relay decisions simultaneously
///
/// Messages awaiting relay scoring and transmission scheduling are queued here.

#[cfg(feature = "memory-config-small")]
const WAIT_POOL_SIZE: usize = 5;

#[cfg(feature = "memory-config-medium")]
const WAIT_POOL_SIZE: usize = 10;

#[cfg(feature = "memory-config-large")]
const WAIT_POOL_SIZE: usize = 20;

/// Number of concurrent echo response collections that can be tracked
///
/// Echo protocol responses are buffered here during the gathering phase.

#[cfg(feature = "memory-config-small")]
const ECHO_RESPONSES_WAIT_POOL_SIZE: usize = 3;

#[cfg(feature = "memory-config-medium")]
const ECHO_RESPONSES_WAIT_POOL_SIZE: usize = 5;

#[cfg(feature = "memory-config-large")]
const ECHO_RESPONSES_WAIT_POOL_SIZE: usize = 10;

/// Size of the duplicate detection cache for received messages
///
/// Prevents reprocessing of messages already seen. Larger values improve
/// duplicate detection at the cost of memory.

#[cfg(feature = "memory-config-small")]
const LAST_RECEIVED_MESSAGE_BUFFER_SIZE: usize = 10;

#[cfg(feature = "memory-config-medium")]
const LAST_RECEIVED_MESSAGE_BUFFER_SIZE: usize = 20;

#[cfg(feature = "memory-config-large")]
const LAST_RECEIVED_MESSAGE_BUFFER_SIZE: usize = 30;

/// Maximum number of concurrent nodes supported in the network
///
/// In simulator mode, supports up to 1000 nodes for testing large networks.
/// For real hardware, typically set to 1 (single node deployment).
#[cfg(feature = "radio-device-simulator")]
pub const MAX_NODE_COUNT: usize = 1000;

/// Maximum number of concurrent nodes supported in the network
///
/// For embedded/hardware deployments, limited to 1 node per device.
#[cfg(not(feature = "radio-device-simulator"))]
pub const MAX_NODE_COUNT: usize = 1;

/// Configuration for radio transmission timing and network behavior
///
/// Controls the delays between packet and message transmissions to manage
/// radio spectrum usage, avoid interference, and implement network protocols.
///
/// # Example
/// ```rust
/// use moonblokz_radio_lib::{RadioConfiguration, ScoringMatrix};
///
/// let config = RadioConfiguration {
///     delay_between_tx_packets: 1,      // 1 second between packets
///     delay_between_tx_messages: 20,    // 20 seconds between messages
///     echo_request_minimal_interval: 86400, // Echo every 24 hours
///     echo_messages_target_interval: 100,   // Target 100 messages between echoes
///     echo_gathering_timeout: 10,           // Wait 10 seconds for echo responses
///     relay_position_delay: 10,             // 10 second delay for relay scoring
///     scoring_matrix: ScoringMatrix::new_from_encoded(&[255, 243, 65, 82, 143]),
///     retry_interval_for_missing_packets: 60, // Retry missing packets after 60 seconds
///    tx_maximum_random_delay: 200,        // Up to 200ms random delay
/// };
/// ```
pub struct RadioConfiguration {
    /// Delay in seconds between individual packets within a multi-packet message
    ///
    /// Prevents overwhelming the receiver and allows other nodes to transmit.
    pub delay_between_tx_packets: u16,

    /// Delay in seconds between complete message transmissions
    ///
    /// Provides fair channel access and prevents a single node from monopolizing the spectrum.
    pub delay_between_tx_messages: u8,

    /// Minimum interval in seconds between echo request broadcasts
    ///
    /// Echo requests probe the network to discover neighbors and measure connection quality.
    /// Lower values provide more frequent topology updates but increase network traffic.
    pub echo_request_minimal_interval: u32,

    /// Target number of messages to receive before sending an echo request
    ///
    /// Adaptive echo timing: if the network is active, echo less frequently.
    pub echo_messages_target_interval: u8,

    /// Timeout in seconds to wait for echo responses during the gathering phase
    ///
    /// After broadcasting an echo request, the node waits this long to collect responses
    /// from neighbors before processing the topology update.
    pub echo_gathering_timeout: u8,

    /// Delay in seconds used when calculating relay position in the scoring algorithm
    ///
    /// Nodes with better connections wait less time before relaying, implementing
    /// a distributed priority system.
    pub relay_position_delay: u8,

    /// Matrix defining relay decision scoring based on connection quality
    ///
    /// Used to evaluate whether this node should relay a message based on its
    /// connection quality compared to other nodes in the network.
    pub scoring_matrix: ScoringMatrix,

    /// Interval in seconds to retry requesting missing packets in a multi-packet message
    ///
    /// When packets are lost during transmission, this controls how often to request
    /// retransmission from the sender.
    pub retry_interval_for_missing_packets: u8,

    /// Maximum random delay in milliseconds added to transmission timing
    pub tx_maximum_random_delay: u16,
}

/// Error returned when attempting to send a message
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendMessageError {
    /// The outgoing message queue is full, cannot accept more messages
    ChannelFull,

    /// The RadioCommunicationManager has not been initialized yet
    NotInited,
}

#[cfg(feature = "std")]
impl std::fmt::Display for SendMessageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SendMessageError::ChannelFull => write!(f, "outgoing message queue is full"),
            SendMessageError::NotInited => write!(f, "RadioCommunicationManager not initialized"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for SendMessageError {}

/// Error returned when attempting to receive a message
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReceiveMessageError {
    /// The RadioCommunicationManager has not been initialized yet
    NotInited,
}

#[cfg(feature = "std")]
impl std::fmt::Display for ReceiveMessageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "RadioCommunicationManager not initialized")
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ReceiveMessageError {}

/// Size of the outgoing message queue
///
/// Messages queued for transmission are buffered here before being
/// fragmented and scheduled by the TX scheduler.
const OUTGOING_MESSAGE_QUEUE_SIZE: usize = 10;

/// Type alias for the outgoing message channel
type OutgoingMessageQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioMessage, OUTGOING_MESSAGE_QUEUE_SIZE>;

/// Type alias for the receiver end of the outgoing message channel
type OutgoingMessageQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioMessage, OUTGOING_MESSAGE_QUEUE_SIZE>;

/// Type alias for the sender end of the outgoing message channel
type OutgoingMessageQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioMessage, OUTGOING_MESSAGE_QUEUE_SIZE>;

/// Static allocation of outgoing message queue for embedded targets
#[cfg(feature = "embedded")]
static OUTGOING_MESSAGE_QUEUE: OutgoingMessageQueue = Channel::new();

/// Size of the incoming message queue
///
/// Received and validated messages are buffered here for application processing.
const INCOMING_MESSAGE_QUEUE_SIZE: usize = 10;

/// Type alias for the incoming message channel
type IncomingMessageQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, IncomingMessageItem, INCOMING_MESSAGE_QUEUE_SIZE>;

/// Type alias for the sender end of the incoming message channel
type IncomingMessageQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, IncomingMessageItem, INCOMING_MESSAGE_QUEUE_SIZE>;

/// Type alias for the receiver end of the incoming message channel
type IncomingMessageQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, IncomingMessageItem, INCOMING_MESSAGE_QUEUE_SIZE>;

/// Static allocation of incoming message queue for embedded targets
#[cfg(feature = "embedded")]
static INCOMING_MESSAGE_QUEUE: IncomingMessageQueue = Channel::new();

/// Size of the TX packet queue
///
/// Packets ready for transmission are buffered here before being sent
/// to the radio device. Larger than message queues since messages can
/// be fragmented into multiple packets.
const TX_PACKET_QUEUE_SIZE: usize = 16;

/// Type alias for the TX packet channel
type TXPacketQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioPacket, TX_PACKET_QUEUE_SIZE>;

/// Type alias for the receiver end of the TX packet channel
type TxPacketQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioPacket, TX_PACKET_QUEUE_SIZE>;

/// Type alias for the sender end of the TX packet channel
type TxPacketQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioPacket, TX_PACKET_QUEUE_SIZE>;

/// Static allocation of TX packet queue for embedded targets
#[cfg(feature = "embedded")]
static TX_PACKET_QUEUE: TXPacketQueue = Channel::new();

/// Size of the RX packet queue
///
/// Packets received from the radio device are buffered here before
/// being processed and assembled into messages by the RX handler.
const RX_PACKET_QUEUE_SIZE: usize = 16;

/// Type alias for the RX packet channel
type RxPacketQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, ReceivedPacket, RX_PACKET_QUEUE_SIZE>;

/// Type alias for the receiver end of the RX packet channel
type RxPacketQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, ReceivedPacket, RX_PACKET_QUEUE_SIZE>;

/// Type alias for the sender end of the RX packet channel
type RxPacketQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, ReceivedPacket, RX_PACKET_QUEUE_SIZE>;

/// Static allocation of RX packet queue for embedded targets
#[cfg(feature = "embedded")]
static RX_PACKET_QUEUE: RxPacketQueue = Channel::new();

/// Size of the RX state signaling queue
///
/// Small queue for signaling receive state changes to the TX scheduler,
/// preventing transmission during active reception.
const RX_STATE_QUEUE_SIZE: usize = 2;

/// Type alias for the RX state channel
type RxStateQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RxState, RX_STATE_QUEUE_SIZE>;

/// Type alias for the receiver end of the RX state channel
type RxStateQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RxState, RX_STATE_QUEUE_SIZE>;

/// Type alias for the sender end of the RX state channel
type RxStateQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RxState, RX_STATE_QUEUE_SIZE>;

/// Static allocation of RX state queue for embedded targets
#[cfg(feature = "embedded")]
static RX_STATE_QUEUE: RxStateQueue = Channel::new();

/// Size of the processing result queue
///
/// The application reports message processing results through this queue,
/// enabling the RX handler to coordinate responses and relay decisions.
const PROCESS_RESULT_QUEUE_SIZE: usize = 10;

/// Type alias for the processing result channel
type ProcessResultQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, MessageProcessingResult, PROCESS_RESULT_QUEUE_SIZE>;

/// Type alias for the sender end of the processing result channel
type ProcessResultQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, MessageProcessingResult, PROCESS_RESULT_QUEUE_SIZE>;

/// Type alias for the receiver end of the processing result channel
type ProcessResultQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, MessageProcessingResult, PROCESS_RESULT_QUEUE_SIZE>;

/// Static allocation of processing result queue for embedded targets
#[cfg(feature = "embedded")]
static PROCESS_RESULT_QUEUE: ProcessResultQueue = Channel::new();

/// Scoring matrix for relay decisions based on connection quality
///
/// This matrix encodes rules for deciding whether a node should relay a message
/// based on comparing its connection quality to the requestor versus the original
/// sender's connection quality.
///
/// The matrix is 4x4, with rows representing the network's connection quality
/// (from the original sender) and columns representing this node's connection quality.
/// Each cell contains a score (0-15) indicating relay priority.
///
/// Connection quality categories:
/// - 0: No connection (quality = 0)
/// - 1: Poor (quality < poor_limit)
/// - 2: Fair (poor_limit ≤ quality < excellent_limit)
/// - 3: Excellent (quality ≥ excellent_limit)
///
/// # Example
/// ```rust
/// use moonblokz_radio_lib::ScoringMatrix;
///
/// // Create matrix with encoded values
/// let matrix = ScoringMatrix::new_from_encoded(&[255, 243, 65, 82, 143]);
///
/// // Or create directly
/// let matrix = ScoringMatrix::new(
///     [[0, 15, 15, 15],   // Row 0: Network has no connection
///      [0,  0,  3,  4],   // Row 1: Network has poor connection
///      [0,  0,  0,  1],   // Row 2: Network has fair connection
///      [0,  0,  0,  0]],  // Row 3: Network has excellent connection
///     20,  // poor_limit
///     40,  // excellent_limit
///     15   // relay_score_limit
/// );
/// ```
pub struct ScoringMatrix {
    /// 4x4 matrix of relay scores
    ///
    /// Only the upper triangle is used (`matrix[row][col]` where `col > row`).
    /// Values are 0-15 (4-bit scores).
    pub matrix: [[u8; 4]; 4],

    /// Threshold for "poor" connection quality (0-63 scale)
    ///
    /// Quality values below this are considered poor.
    pub poor_limit: u8,

    /// Threshold for "excellent" connection quality (0-63 scale)
    ///
    /// Quality values at or above this are considered excellent.
    pub excellent_limit: u8,

    /// Minimum score required to relay a message (0-15 scale)
    ///
    /// If the calculated relay score is below this threshold, the node
    /// will not relay the message.
    pub relay_score_limit: u8,
}

impl ScoringMatrix {
    /// Creates a new ScoringMatrix with explicit values
    ///
    /// # Arguments
    /// * `matrix` - 4x4 array of relay scores (use upper triangle, col > row)
    /// * `poor_limit` - Threshold for poor connection quality (0-63)
    /// * `excellent_limit` - Threshold for excellent connection quality (0-63)
    /// * `relay_score_limit` - Minimum score to relay (0-15)
    pub const fn new(matrix: [[u8; 4]; 4], poor_limit: u8, excellent_limit: u8, relay_score_limit: u8) -> Self {
        Self {
            matrix,
            poor_limit,
            excellent_limit,
            relay_score_limit,
        }
    }

    /// Creates a ScoringMatrix from a compact 5-byte encoded representation
    ///
    /// The encoding uses only the upper triangle of the matrix (6 cells) since
    /// the lower triangle is unused in relay scoring. Limits are bit-packed
    /// to minimize space.
    ///
    /// # Encoding Format
    /// ```text
    /// Byte 0: [B1: 4 bits][C1: 4 bits]
    /// Byte 1: [D1: 4 bits][C2: 4 bits]
    /// Byte 2: [D2: 4 bits][D3: 4 bits]
    /// Byte 3: [poor_limit: 6 bits][excellent_limit bits 5-4: 2 bits]
    /// Byte 4: [excellent_limit bits 3-0: 4 bits][relay_score_limit: 4 bits]
    /// ```
    ///
    /// Where `B1 = matrix[0][1]`, `C1 = matrix[0][2]`, `D1 = matrix[0][3]`,
    /// `C2 = matrix[1][2]`, `D2 = matrix[1][3]`, `D3 = matrix[2][3]`
    ///
    /// # Arguments
    /// * `encoded` - 5-byte array containing the packed matrix and limits
    ///
    /// # Example
    /// ```rust
    /// use moonblokz_radio_lib::ScoringMatrix;
    ///
    /// // Default recommended values
    /// let matrix = ScoringMatrix::new_from_encoded(&[255, 243, 65, 82, 143]);
    /// assert_eq!(matrix.poor_limit, 20);
    /// assert_eq!(matrix.excellent_limit, 40);
    /// assert_eq!(matrix.relay_score_limit, 15);
    /// ```
    pub const fn new_from_encoded(encoded: &[u8; 5]) -> Self {
        let mut matrix = [[0; 4]; 4];

        matrix[0][1] = (encoded[0] >> 4) & 0x0F;
        matrix[0][2] = encoded[0] & 0x0F;
        matrix[0][3] = (encoded[1] >> 4) & 0x0F;
        matrix[1][2] = encoded[1] & 0x0F;
        matrix[1][3] = (encoded[2] >> 4) & 0x0F;
        matrix[2][3] = encoded[2] & 0x0F;
        // Limits are packed as:
        // poor_limit:      bits [7:2] of encoded[3] (6 bits)
        // excellent_limit: bits [1:0] of encoded[3] (upper 2 bits) concatenated with bits [7:4] of encoded[4] (lower 4 bits) => 6 bits total
        // relay_score:     bits [3:0] of encoded[4]
        let poor_limit = (encoded[3] >> 2) & 0x3F;
        let excellent_limit = ((encoded[3] & 0x03) << 4) | (encoded[4] >> 4);
        let relay_score_limit = encoded[4] & 0x0F;

        Self::new(matrix, poor_limit, excellent_limit, relay_score_limit)
    }
}

/// Internal state for RX operations communicated to TX scheduler
///
/// Signals when multi-packet reception is in progress so TX scheduler
/// can avoid transmitting during receive operations (half-duplex constraint).
enum RxState {
    /// Multi-packet message reception in progress
    ///
    /// Arguments: (current_packet_index, total_packet_count)
    PacketedRxInProgress(u8, u8),

    /// Multi-packet message reception completed
    PacketedRxEnded,
}

/// Result of processing a received message at the application layer
///
/// The application reports these results back to enable appropriate
/// responses, relay decisions, and network coordination.
pub enum MessageProcessingResult {
    /// Requested block was not found in this node's storage
    ///
    /// Argument: block sequence number requested
    RequestedBlockNotFound(u32),

    /// Requested block was found and is being returned
    ///
    /// Argument: RadioMessage containing the block data
    RequestedBlockFound(RadioMessage),

    /// Requested block parts were found and are being returned
    ///
    /// Arguments: (RadioMessage with block parts, requestor node ID)
    RequestedBlockPartsFound(RadioMessage, u32),

    /// A new block has been added to the local blockchain
    ///
    /// Argument: RadioMessage containing the new block
    NewBlockAdded(RadioMessage),

    /// A new transaction has been added to the mempool
    ///
    /// Argument: RadioMessage containing the transaction
    NewTransactionAdded(RadioMessage),

    /// A reply transaction is being sent in response to a request
    ///
    /// Argument: RadioMessage containing the reply transaction
    SendReplyTransaction(RadioMessage),

    /// A new support vote has been added
    ///
    /// Argument: RadioMessage containing the support vote
    NewSupportAdded(RadioMessage),

    /// Message was already received and processed (duplicate detection)
    ///
    /// Arguments: (message_type, sequence, payload_checksum)
    AlreadyHaveMessage(u8, u32, u32),
}

/// Item delivered to the application via the incoming message queue
///
/// Either a check for duplicate detection or a new message to process.
pub enum IncomingMessageItem {
    /// Query if message was already processed (for duplicate detection)
    ///
    /// Application should respond via report_message_processing_status with
    /// AlreadyHaveMessage if this message was seen before.
    /// Arguments: (message_type, sequence, payload_checksum)
    CheckIfAlreadyHaveMessage(u8, u32, u32),

    /// New message received from the network for application processing
    NewMessage(RadioMessage),
}

/// A packet received from the radio with associated signal quality metrics
pub struct ReceivedPacket {
    /// The received radio packet containing data
    pub packet: RadioPacket,

    /// Link quality measurement (0-63 scale, derived from RSSI and SNR)
    ///
    /// Higher values indicate better signal quality. Used for connection
    /// quality tracking in the relay manager.
    pub link_quality: u8,
}

/// Internal state of the RadioCommunicationManager
enum RadioCommunicationManagerState {
    /// Manager has been created but not yet initialized
    Uninitialized,

    /// Manager is initialized and operational
    Initialized {
        /// Sender for queuing outgoing messages
        outgoing_message_queue_sender: OutgoingMessageQueueSender,

        /// Receiver for retrieving incoming messages
        incoming_message_queue_receiver: IncomingMessageQueueReceiver,

        /// Sender for reporting message processing results
        process_result_queue_sender: ProcessResultQueueSender,
    },
}

/// Main interface for radio communication operations
///
/// Manages the radio communication stack, coordinating between the application,
/// message queues, radio device, and internal tasks (TX scheduler, RX handler,
/// relay manager).
///
/// # Lifecycle
/// 1. Create with `new()`
/// 2. Initialize with `initialize()` (spawns async tasks)
/// 3. Send messages with `send_message()`
/// 4. Receive messages with `receive_message()`
/// 5. Report processing results with `report_message_processing_status()`
///
/// # Example
/// ```rust,ignore
/// use moonblokz_radio_lib::{RadioCommunicationManager, RadioConfiguration, RadioMessage};
/// use embassy_executor::Spawner;
///
/// let mut manager = RadioCommunicationManager::new();
///
/// // Initialize (in async context with spawner and radio device)
/// manager.initialize(config, spawner, radio_device, own_node_id, rng_seed)?;
///
/// // Send a message
/// let message = RadioMessage::new_request_echo(own_node_id);
/// manager.send_message(message)?;
///
/// // Receive messages
/// loop {
///     match manager.receive_message().await? {
///         IncomingMessageItem::NewMessage(msg) => {
///             // Process message
///         }
///         IncomingMessageItem::CheckIfAlreadyHaveMessage(mtype, seq, csum) => {
///             // Check for duplicates
///         }
///     }
/// }
/// ```
pub struct RadioCommunicationManager {
    state: RadioCommunicationManagerState,
}

//TODO: generic Constant Params and consolidate new and initialize methods

impl RadioCommunicationManager {
    /// Creates a new uninitialized RadioCommunicationManager
    ///
    /// The manager must be initialized with `initialize()` before use.
    ///
    /// # Example
    /// ```rust
    /// use moonblokz_radio_lib::RadioCommunicationManager;
    ///
    /// let manager = RadioCommunicationManager::new();
    /// // Must call initialize() before sending/receiving messages
    /// ```
    pub const fn new() -> Self {
        // Initialize the RelayManager and TxScheduler
        RadioCommunicationManager {
            state: RadioCommunicationManagerState::Uninitialized,
        }
    }

    /// Initializes the radio communication system for embedded targets
    ///
    /// Spawns async tasks for radio device, TX scheduler, and RX handler, then
    /// transitions the manager to the initialized state. Uses static allocation
    /// for all queues (no dynamic memory).
    ///
    /// # Arguments
    /// * `radio_config` - Configuration for timing and network behavior
    /// * `spawner` - Embassy executor spawner for async tasks
    /// * `radio_device` - Initialized radio hardware device
    /// * `own_node_id` - This node's unique identifier in the network
    /// * `rng_seed` - Seed for random number generation (timing jitter, etc.)
    ///
    /// # Returns
    /// * `Ok(())` - Successfully initialized
    /// * `Err(())` - Failed to spawn one or more tasks
    ///
    /// # Example
    /// ```rust,ignore
    /// let mut manager = RadioCommunicationManager::new();
    /// manager.initialize(config, spawner, radio_device, 42, 12345)?;
    /// ```
    #[cfg(feature = "embedded")]
    pub fn initialize(
        &mut self,
        radio_config: RadioConfiguration,
        spawner: Spawner,
        radio_device: RadioDevice,
        own_node_id: u32,
        rng_seed: u64,
    ) -> Result<(), ()> {
        return self.initialize_common(
            radio_config,
            spawner,
            radio_device,
            &OUTGOING_MESSAGE_QUEUE,
            &INCOMING_MESSAGE_QUEUE,
            &PROCESS_RESULT_QUEUE,
            &TX_PACKET_QUEUE,
            &RX_PACKET_QUEUE,
            &RX_STATE_QUEUE,
            own_node_id,
            rng_seed,
        );
    }

    /// Initializes the radio communication system for standard library targets
    ///
    /// Spawns async tasks for radio device, TX scheduler, and RX handler, then
    /// transitions the manager to the initialized state. Uses heap allocation
    /// via Box::leak for queue storage.
    ///
    /// # Arguments
    /// * `radio_config` - Configuration for timing and network behavior
    /// * `spawner` - Embassy executor spawner for async tasks
    /// * `radio_device` - Initialized radio hardware device
    /// * `own_node_id` - This node's unique identifier in the network
    /// * `rng_seed` - Seed for random number generation (timing jitter, etc.)
    ///
    /// # Returns
    /// * `Ok(())` - Successfully initialized
    /// * `Err(())` - Failed to spawn one or more tasks
    ///
    /// # Note
    /// Queue memory is leaked (Box::leak) to obtain 'static lifetime required
    /// by Embassy channels. This is intentional and safe for long-lived managers.
    #[cfg(feature = "std")]
    pub fn initialize(
        &mut self,
        radio_config: RadioConfiguration,
        spawner: Spawner,
        radio_device: RadioDevice,
        own_node_id: u32,
        rng_seed: u64,
    ) -> Result<(), ()> {
        let outgoing_message_queue_temp: OutgoingMessageQueue = Channel::new();
        let outgoing_message_queue_static: &'static OutgoingMessageQueue = Box::leak(Box::new(outgoing_message_queue_temp));

        let incoming_message_queue_temp: IncomingMessageQueue = Channel::new();
        let incoming_message_queue_static: &'static IncomingMessageQueue = Box::leak(Box::new(incoming_message_queue_temp));

        let tx_packet_queue_temp: TXPacketQueue = Channel::new();
        let tx_packet_queue_static: &'static TXPacketQueue = Box::leak(Box::new(tx_packet_queue_temp));

        let rx_packet_queue_temp: RxPacketQueue = Channel::new();
        let rx_packet_queue_static: &'static RxPacketQueue = Box::leak(Box::new(rx_packet_queue_temp));

        let rx_state_queue_temp: RxStateQueue = Channel::new();
        let rx_state_queue_static: &'static RxStateQueue = Box::leak(Box::new(rx_state_queue_temp));

        let process_result_queue_temp: ProcessResultQueue = Channel::new();
        let process_result_queue_static: &'static ProcessResultQueue = Box::leak(Box::new(process_result_queue_temp));
        return self.initialize_common(
            radio_config,
            spawner,
            radio_device,
            outgoing_message_queue_static,
            incoming_message_queue_static,
            process_result_queue_static,
            tx_packet_queue_static,
            rx_packet_queue_static,
            rx_state_queue_static,
            own_node_id,
            rng_seed,
        );
    }

    /// Common initialization logic shared between embedded and std features
    ///
    /// Internal method that performs the actual initialization work:
    /// - Spawns radio device task
    /// - Spawns TX scheduler task  
    /// - Spawns RX handler task
    /// - Transitions manager state to Initialized
    ///
    /// # Arguments
    /// * `radio_config` - Configuration decomposed into individual parameters
    /// * `spawner` - Embassy executor for spawning tasks
    /// * `radio_device` - Radio hardware abstraction
    /// * `outgoing_message_queue` - Static reference to outgoing message channel
    /// * `incoming_message_queue` - Static reference to incoming message channel
    /// * `process_result_queue` - Static reference to processing result channel
    /// * `tx_packet_queue` - Static reference to TX packet channel
    /// * `rx_packet_queue` - Static reference to RX packet channel
    /// * `rx_state_queue` - Static reference to RX state signaling channel
    /// * `own_node_id` - This node's network identifier
    /// * `rng_seed` - RNG seed for generating unique seeds for each task
    ///
    /// # Returns
    /// * `Ok(())` - All tasks spawned successfully
    /// * `Err(())` - One or more tasks failed to spawn
    fn initialize_common(
        &mut self,
        radio_config: RadioConfiguration,
        spawner: Spawner,
        radio_device: RadioDevice,
        outgoing_message_queue: &'static OutgoingMessageQueue,
        incoming_message_queue: &'static IncomingMessageQueue,
        process_result_queue: &'static ProcessResultQueue,
        tx_packet_queue: &'static TXPacketQueue,
        rx_packet_queue: &'static RxPacketQueue,
        rx_state_queue: &'static RxStateQueue,
        own_node_id: u32,
        rng_seed: u64,
    ) -> Result<(), ()> {
        let mut rng = WyRand::seed_from_u64(rng_seed);

        // Destructure config to avoid partial moves later
        let RadioConfiguration {
            delay_between_tx_packets,
            delay_between_tx_messages,
            echo_request_minimal_interval,
            echo_messages_target_interval,
            echo_gathering_timeout,
            relay_position_delay,
            scoring_matrix,
            retry_interval_for_missing_packets,
            tx_maximum_random_delay,
        } = radio_config;

        // Spawn the radio task
        let radion_device_task_result = spawner.spawn(radio_device_task(
            radio_device,
            tx_packet_queue.receiver(),
            rx_packet_queue.sender(),
            rng.next_u64(),
        ));
        if radion_device_task_result.is_err() {
            return Err(());
        }
        log!(log::Level::Debug, "Radio device task spawned");

        let tx_scheduler_task_result = spawner.spawn(tx_scheduler_task(
            outgoing_message_queue.receiver(),
            rx_state_queue.receiver(),
            tx_packet_queue.sender(),
            delay_between_tx_packets,
            delay_between_tx_messages,
            tx_maximum_random_delay,
            rng.next_u64(),
        ));
        if tx_scheduler_task_result.is_err() {
            return Err(());
        }
        log!(log::Level::Debug, "TX Scheduler task spawned");

        let rx_handler_task_result = spawner.spawn(rx_handler::rx_handler_task(
            incoming_message_queue.sender(),
            outgoing_message_queue.sender(),
            rx_packet_queue.receiver(),
            rx_state_queue.sender(),
            process_result_queue.receiver(),
            echo_request_minimal_interval,
            echo_messages_target_interval,
            echo_gathering_timeout,
            relay_position_delay,
            scoring_matrix,
            retry_interval_for_missing_packets,
            own_node_id,
            rng.next_u64(),
        ));
        if rx_handler_task_result.is_err() {
            return Err(());
        }
        log!(log::Level::Debug, "RX Handler task spawned");
        log!(log::Level::Info, "Radio communication initialized");

        self.state = RadioCommunicationManagerState::Initialized {
            outgoing_message_queue_sender: outgoing_message_queue.sender(),
            incoming_message_queue_receiver: incoming_message_queue.receiver(),
            process_result_queue_sender: process_result_queue.sender(),
        };
        Ok(())
    }

    /// Sends a message to the radio network
    ///
    /// Queues a message for transmission. The message will be fragmented into
    /// packets if necessary, scheduled with appropriate timing delays, and
    /// transmitted by the radio device.
    ///
    /// # Arguments
    /// * `message` - The RadioMessage to send
    ///
    /// # Returns
    /// * `Ok(())` - Message queued successfully
    /// * `Err(SendMessageError::NotInited)` - Manager not initialized
    /// * `Err(SendMessageError::ChannelFull)` - Queue full, try again later
    ///
    /// # Example
    /// ```rust,ignore
    /// let message = RadioMessage::new_request_echo(own_node_id);
    /// manager.send_message(message)?;
    /// ```
    pub fn send_message(&self, message: RadioMessage) -> Result<(), SendMessageError> {
        let outgoing_message_queue_sender = match &self.state {
            RadioCommunicationManagerState::Uninitialized => {
                return Err(SendMessageError::NotInited);
            }
            RadioCommunicationManagerState::Initialized {
                outgoing_message_queue_sender, ..
            } => outgoing_message_queue_sender,
        };
        outgoing_message_queue_sender.try_send(message).map_err(|_| SendMessageError::ChannelFull)?;
        Ok(())
    }

    /// Receives a message from the radio network (async)
    ///
    /// Blocks until a message is available from the network. Messages are delivered
    /// as `IncomingMessageItem` which may be either a duplicate check query or a
    /// new message to process.
    ///
    /// # Returns
    /// * `Ok(IncomingMessageItem)` - Message or query received
    /// * `Err(ReceiveMessageError::NotInited)` - Manager not initialized
    ///
    /// # Example
    /// ```rust,ignore
    /// loop {
    ///     match manager.receive_message().await? {
    ///         IncomingMessageItem::NewMessage(msg) => {
    ///             // Process message and report result
    ///             let result = process_message(msg);
    ///             manager.report_message_processing_status(result);
    ///         }
    ///         IncomingMessageItem::CheckIfAlreadyHaveMessage(mtype, seq, csum) => {
    ///             if database.has_message(mtype, seq, csum) {
    ///                 manager.report_message_processing_status(
    ///                     MessageProcessingResult::AlreadyHaveMessage(mtype, seq, csum)
    ///                 );
    ///             }
    ///         }
    ///     }
    /// }
    /// ```
    pub async fn receive_message(&self) -> Result<IncomingMessageItem, ReceiveMessageError> {
        let incoming_message_queue_receiver = match &self.state {
            RadioCommunicationManagerState::Uninitialized => {
                return Err(ReceiveMessageError::NotInited);
            }
            RadioCommunicationManagerState::Initialized {
                incoming_message_queue_receiver,
                ..
            } => incoming_message_queue_receiver,
        };
        return Ok(incoming_message_queue_receiver.receive().await);
    }

    /// Reports the result of processing a received message
    ///
    /// The application calls this after processing a message to inform the radio
    /// stack about the outcome. This enables the RX handler and relay manager to:
    /// - Coordinate responses to requests
    /// - Make relay decisions based on message content
    /// - Track duplicate messages
    /// - Manage the wait pool for pending responses
    ///
    /// If the manager is not initialized, this call silently does nothing.
    ///
    /// # Arguments
    /// * `message_processing_result` - The processing outcome
    ///
    /// # Example
    /// ```rust,ignore
    /// match manager.receive_message().await? {
    ///     IncomingMessageItem::NewMessage(msg) => {
    ///         if let Some(block) = database.get_block(msg.sequence()) {
    ///             manager.report_message_processing_status(
    ///                 MessageProcessingResult::RequestedBlockFound(block)
    ///             );
    ///         } else {
    ///             manager.report_message_processing_status(
    ///                 MessageProcessingResult::RequestedBlockNotFound(msg.sequence())
    ///             );
    ///         }
    ///     }
    ///     // ...
    /// }
    /// ```
    pub fn report_message_processing_status(&self, message_processing_result: MessageProcessingResult) {
        let process_result_queue_sender = match &self.state {
            RadioCommunicationManagerState::Uninitialized => {
                return;
            }
            RadioCommunicationManagerState::Initialized {
                process_result_queue_sender, ..
            } => process_result_queue_sender,
        };
        process_result_queue_sender.try_send(message_processing_result).ok();
    }
}

// --- Link Quality Calculation Constants ---
// These values define the expected operational range for radio signal metrics.
// Values are typical for LoRaWAN and similar long-range radio systems.

/// Minimum RSSI value expected for decodable signals (in dBm)
///
/// Signals weaker than this are typically below the noise floor and cannot
/// be reliably decoded. Used as the lower bound for link quality normalization.
const RSSI_MIN: i16 = -120;

/// Maximum RSSI value for very strong signals (in dBm)
///
/// Represents a signal very close to the receiver with maximum strength.
/// Used as the upper bound for link quality normalization.
const RSSI_MAX: i16 = -30;

/// Minimum SNR value for decodable signals (in dB)
///
/// Can be negative for LoRa due to spread spectrum processing gain.
/// Used as the lower bound for SNR normalization.
const SNR_MIN: i16 = -20;

/// Maximum SNR value for very clean signals (in dB)
///
/// Represents a signal with minimal noise and interference.
/// Used as the upper bound for SNR normalization.
const SNR_MAX: i16 = 10;

/// Normalizes a value to a 0-63 scale based on defined min/max bounds
///
/// Clamps the input value within the specified range and linearly scales it
/// to the 0-63 output range. Values below `min` result in 0, values above
/// `max` result in 63.
///
/// # Arguments
/// * `value` - The input value to normalize (e.g., -90 for RSSI)
/// * `min` - The bottom of the input range (e.g., -120 for RSSI_MIN)
/// * `max` - The top of the input range (e.g., -30 for RSSI_MAX)
///
/// # Returns
/// A u8 value in the range 0-63 representing the normalized quality
///
/// # Example
/// ```rust
/// use moonblokz_radio_lib::normalize;
///
/// // RSSI of -90 dBm (between -120 and -30 range)
/// let quality = normalize(-90, -120, -30);
/// assert!(quality > 0 && quality < 63);
///
/// // Value below minimum
/// let quality = normalize(-150, -120, -30);
/// assert_eq!(quality, 0);
///
/// // Value above maximum
/// let quality = normalize(-20, -120, -30);
/// assert_eq!(quality, 63);
/// ```
pub fn normalize(value: i16, min: i16, max: i16) -> u8 {
    // 1. Clamp the value to ensure it's within the defined range.
    let clamped_value = value.max(min).min(max);

    // 2. Shift the range to start at 0.
    let shifted_value = clamped_value - min;

    // 3. Scale the value to the 0-63 range using integer arithmetic.
    // We multiply by 63 first to maintain precision before the division.
    let scaled_value = (shifted_value as u32 * 63) / (max - min) as u32;

    scaled_value as u8
}

/// Calculates combined link quality from RSSI and SNR measurements
///
/// Combines Received Signal Strength Indicator (RSSI) and Signal-to-Noise Ratio (SNR)
/// into a single quality metric on a 0-63 scale. The calculation uses a weighted average
/// favoring SNR (70%) over RSSI (30%) since SNR is a better indicator of link reliability.
///
/// The quality score is used by the relay manager to track connection quality between
/// nodes and make intelligent routing decisions.
///
/// # Arguments
/// * `rssi` - Raw RSSI value in dBm (typically -120 to -30)
/// * `snr` - Raw SNR value in dB (typically -20 to 10)
///
/// # Returns
/// A u8 link quality score in the range 0-63
/// - 0: Unusable link (very poor signal)
/// - 63: Excellent link (strong signal and low noise)
///
/// # Algorithm
/// 1. Normalize RSSI to 0-63 scale using RSSI_MIN/RSSI_MAX bounds
/// 2. Normalize SNR to 0-63 scale using SNR_MIN/SNR_MAX bounds
/// 3. Calculate weighted average: (30% × RSSI + 70% × SNR)
///
/// # Example
/// ```rust
/// use moonblokz_radio_lib::calculate_link_quality;
///
/// // Good signal with decent SNR
/// let quality = calculate_link_quality(-70, 5);
/// assert!(quality > 40); // Should be in "good" range
///
/// // Weak signal with poor SNR
/// let quality = calculate_link_quality(-110, -15);
/// assert!(quality < 20); // Should be in "poor" range
/// ```
pub fn calculate_link_quality(rssi: i16, snr: i16) -> u8 {
    // 1. Normalize both RSSI and SNR to a common 0-63 scale.
    let norm_rssi = normalize(rssi, RSSI_MIN, RSSI_MAX);
    let norm_snr = normalize(snr, SNR_MIN, SNR_MAX);

    // 2. Calculate the weighted average using integer math.
    // Weights: 7 for SNR, 3 for RSSI. Total weight is 10.
    // We use u32 for the intermediate calculation to prevent overflow.
    let quality = (3 * norm_rssi as u32 + 7 * norm_snr as u32) / 10;

    // The result is guaranteed to be in the 0-63 range.
    quality as u8
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use futures::executor::block_on;

    #[test]
    fn radio_config_constructs() {
        let _cfg = RadioConfiguration {
            delay_between_tx_packets: 1,
            delay_between_tx_messages: 20,
            echo_request_minimal_interval: 86400,
            echo_messages_target_interval: 100,
            echo_gathering_timeout: 10,
            relay_position_delay: 10,
            scoring_matrix: ScoringMatrix::new_from_encoded(&[255u8, 243u8, 65u8, 82u8, 143u8]),
            retry_interval_for_missing_packets: 60,
            tx_maximum_random_delay: 200,
        };
    }

    #[test]
    fn manager_send_message_not_inited() {
        let mgr = RadioCommunicationManager::new();
        let msg = RadioMessage::new_request_echo(123);
        match mgr.send_message(msg) {
            Err(SendMessageError::NotInited) => {}
            other => panic!("Expected NotInited, got: {:?}", core::mem::discriminant(&other)),
        }
    }

    #[test]
    fn manager_receive_message_not_inited() {
        let mgr = RadioCommunicationManager::new();
        let res = block_on(async { mgr.receive_message().await });
        match res {
            Err(ReceiveMessageError::NotInited) => {}
            other => panic!("Expected NotInited, got: {:?}", core::mem::discriminant(&other)),
        }
    }

    #[test]
    fn reexports_are_usable() {
        // Basic sanity that re-exported constructors work from the crate root
        let msg = RadioMessage::new_get_mempool_state(42);
        assert_eq!(msg.message_type(), MessageType::RequestNewMempoolItem as u8);
    }

    #[test]
    fn scoring_matrix_new_from_encoded_decodes_limits() {
        // Choose distinct limits to catch bit packing errors
        let poor: u8 = 0b10_1010; // 42
        let excellent: u8 = 0b11_0011; // 51
        let relay: u8 = 0b1010; // 10

        // Pack per the documented layout
        // encoded[3]: poor in bits [7:2], top 2 bits of excellent in [1:0]
        let e3 = (poor << 2) | (excellent >> 4);
        // encoded[4]: lower 4 bits of excellent in [7:4], relay in [3:0]
        let e4 = ((excellent & 0x0F) << 4) | (relay & 0x0F);
        let encoded = [0u8, 0u8, 0u8, e3, e4];

        let sm = ScoringMatrix::new_from_encoded(&encoded);
        assert_eq!(sm.poor_limit, poor & 0x3F);
        assert_eq!(sm.excellent_limit, excellent & 0x3F);
        assert_eq!(sm.relay_score_limit, relay & 0x0F);
    }

    #[test]
    fn test_scoring_matrix_default_values_encoding() {
        let encoded_value = [255u8, 243u8, 65u8, 82u8, 143u8];
        let sm = ScoringMatrix::new_from_encoded(&encoded_value);
        assert_eq!(sm.poor_limit, 20);
        assert_eq!(sm.excellent_limit, 40);
        assert_eq!(sm.relay_score_limit, 15);
        assert_eq!(sm.matrix[0][1], 15);
        assert_eq!(sm.matrix[0][2], 15);
        assert_eq!(sm.matrix[0][3], 15);
        assert_eq!(sm.matrix[1][2], 3);
        assert_eq!(sm.matrix[1][3], 4);
        assert_eq!(sm.matrix[2][3], 1);
    }

    // ============================================================================
    // Link Quality Calculation Tests
    // ============================================================================

    #[test]
    fn test_normalize_within_bounds() {
        let rssi = -75i16;
        let quality = normalize(rssi, -120, -30);
        assert!(quality > 0 && quality < 63);
        assert!(quality > 20 && quality < 50);
    }

    #[test]
    fn test_normalize_below_minimum() {
        let quality = normalize(-150, -120, -30);
        assert_eq!(quality, 0);
    }

    #[test]
    fn test_normalize_above_maximum() {
        let quality = normalize(-20, -120, -30);
        assert_eq!(quality, 63);
    }

    #[test]
    fn test_calculate_link_quality_strong_signal() {
        let quality = calculate_link_quality(-40, 8);
        assert!(quality > 50);
    }

    #[test]
    fn test_calculate_link_quality_weak_signal() {
        let quality = calculate_link_quality(-115, -18);
        assert!(quality < 15);
    }

    #[test]
    fn test_calculate_link_quality_deterministic() {
        let q1 = calculate_link_quality(-80, 0);
        let q2 = calculate_link_quality(-80, 0);
        assert_eq!(q1, q2);
    }

    #[test]
    fn test_rssi_snr_extreme_values() {
        let q1 = calculate_link_quality(i16::MIN, i16::MIN);
        assert_eq!(q1, 0);

        let q2 = calculate_link_quality(i16::MAX, i16::MAX);
        assert_eq!(q2, 63);
    }

    // ============================================================================
    // Scoring Matrix Tests
    // ============================================================================

    #[test]
    fn test_scoring_matrix_creation() {
        let matrix = ScoringMatrix::new([[0, 15, 15, 15], [0, 0, 3, 4], [0, 0, 0, 1], [0, 0, 0, 0]], 20, 40, 15);

        assert_eq!(matrix.poor_limit, 20);
        assert_eq!(matrix.excellent_limit, 40);
        assert_eq!(matrix.relay_score_limit, 15);
        assert_eq!(matrix.matrix[0][1], 15);
        assert_eq!(matrix.matrix[1][2], 3);
    }

    #[test]
    fn test_scoring_matrix_symmetry() {
        let matrix = ScoringMatrix::new_from_encoded(&[255u8, 243u8, 65u8, 82u8, 143u8]);

        // Lower triangle should be 0 by convention
        assert_eq!(matrix.matrix[1][0], 0);
        assert_eq!(matrix.matrix[2][0], 0);
        assert_eq!(matrix.matrix[2][1], 0);
        assert_eq!(matrix.matrix[3][0], 0);
        assert_eq!(matrix.matrix[3][1], 0);
        assert_eq!(matrix.matrix[3][2], 0);
    }

    // ============================================================================
    // Message Tests
    // ============================================================================

    #[test]
    fn test_message_types_are_distinct() {
        let types = [
            MessageType::RequestEcho as u8,
            MessageType::Echo as u8,
            MessageType::EchoResult as u8,
            MessageType::RequestFullBlock as u8,
            MessageType::RequestBlockPart as u8,
            MessageType::AddBlock as u8,
            MessageType::AddTransaction as u8,
            MessageType::RequestNewMempoolItem as u8,
            MessageType::Support as u8,
        ];

        for (i, &type1) in types.iter().enumerate() {
            for &type2 in types.iter().skip(i + 1) {
                assert_ne!(type1, type2);
            }
        }
    }

    #[test]
    fn test_message_clone() {
        let msg1 = RadioMessage::new_request_echo(123);
        let msg2 = msg1.clone();

        assert_eq!(msg1.sender_node_id(), msg2.sender_node_id());
        assert_eq!(msg1.message_type(), msg2.message_type());
    }

    #[test]
    fn test_empty_message_payload() {
        let msg = RadioMessage::new_add_block(1, 0, &[]);
        // Message length includes header (13 bytes: type + sender_id + sequence)
        // so empty payload still has header
        assert!(msg.length() >= 13);
    }

    #[test]
    fn test_node_id_zero() {
        let msg = RadioMessage::new_request_echo(0);
        assert_eq!(msg.sender_node_id(), 0);
    }

    #[test]
    fn test_sequence_number_wraparound() {
        let max_seq = u32::MAX;
        let msg = RadioMessage::new_add_block(1, max_seq, &[1, 2, 3]);
        assert_eq!(msg.sequence(), Some(max_seq));
    }

    #[test]
    fn test_large_payload_handling() {
        // Create message with maximum payload size minus headers
        let payload_size = RADIO_MAX_MESSAGE_SIZE - 13; // Account for header
        let large_payload = vec![0xAAu8; payload_size];
        let msg = RadioMessage::new_add_block(1, 100, &large_payload);
        assert!(msg.length() <= RADIO_MAX_MESSAGE_SIZE);
        assert!(msg.length() > payload_size); // Should include headers
    }

    // ============================================================================
    // Constants Validation Tests
    // ============================================================================

    #[test]
    fn test_radio_packet_size_constant() {
        assert_eq!(RADIO_PACKET_SIZE, 215);
        assert!(RADIO_PACKET_SIZE > 50);
        assert!(RADIO_PACKET_SIZE < 1024);
    }

    #[test]
    fn test_radio_max_message_size_constant() {
        assert_eq!(RADIO_MAX_MESSAGE_SIZE, 2013);
        assert!(RADIO_MAX_MESSAGE_SIZE > RADIO_PACKET_SIZE);
    }

    #[test]
    fn test_radio_max_packet_count_constant() {
        let expected = (RADIO_MAX_MESSAGE_SIZE + RADIO_PACKET_SIZE - 1) / RADIO_PACKET_SIZE;
        assert_eq!(RADIO_MAX_PACKET_COUNT, expected);
    }

    #[test]
    fn test_max_packet_count_fits_in_u8() {
        assert!(RADIO_MAX_PACKET_COUNT <= 255);
    }

    #[test]
    fn test_connection_matrix_size_constant() {
        assert_eq!(CONNECTION_MATRIX_SIZE, 100);
    }

    // ============================================================================
    // CRC and Integrity Tests
    // ============================================================================

    #[test]
    fn test_payload_checksum_consistency() {
        let payload = [1u8, 2, 3, 4, 5];
        let msg1 = RadioMessage::new_add_block(1, 100, &payload);
        let msg2 = RadioMessage::new_add_block(1, 100, &payload);

        assert_eq!(msg1.payload_checksum(), msg2.payload_checksum());
    }

    #[test]
    fn test_payload_checksum_sensitivity() {
        let payload1 = [1u8, 2, 3, 4, 5];
        let payload2 = [1u8, 2, 3, 4, 6];

        let msg1 = RadioMessage::new_add_block(1, 100, &payload1);
        let msg2 = RadioMessage::new_add_block(1, 100, &payload2);

        assert_ne!(msg1.payload_checksum(), msg2.payload_checksum());
    }

    // ============================================================================
    // Error Type Tests
    // ============================================================================

    #[test]
    #[cfg(feature = "std")]
    fn test_error_types_implement_std_error() {
        // Test SendMessageError
        let send_err1 = SendMessageError::ChannelFull;
        let send_err2 = SendMessageError::NotInited;

        // Verify Display implementation
        assert_eq!(format!("{}", send_err1), "outgoing message queue is full");
        assert_eq!(format!("{}", send_err2), "RadioCommunicationManager not initialized");

        // Verify std::error::Error trait is implemented
        let _: &dyn std::error::Error = &send_err1;
        let _: &dyn std::error::Error = &send_err2;

        // Test ReceiveMessageError
        let recv_err = ReceiveMessageError::NotInited;
        assert_eq!(format!("{}", recv_err), "RadioCommunicationManager not initialized");
        let _: &dyn std::error::Error = &recv_err;
    }

    #[test]
    fn test_error_types_are_debug_and_copy() {
        // Test SendMessageError
        let send_err = SendMessageError::ChannelFull;
        let _send_err_copy = send_err; // Test Copy
        let _send_err_debug = format!("{:?}", send_err); // Test Debug

        // Test ReceiveMessageError
        let recv_err = ReceiveMessageError::NotInited;
        let _recv_err_copy = recv_err; // Test Copy
        let _recv_err_debug = format!("{:?}", recv_err); // Test Debug
    }
}

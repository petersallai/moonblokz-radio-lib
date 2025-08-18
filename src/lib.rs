#![cfg_attr(not(feature = "std"), no_std)]
#![allow(async_fn_in_trait)] // We control the usage of this trait

#[cfg(any(
    all(feature = "radio-device-echo", any(feature = "radio-device-lora-sx1262", feature = "radio-device-simulator")),
    all(feature = "radio-device-lora-sx1262", any(feature = "radio-device-echo", feature = "radio-device-simulator")),
    all(feature = "radio-device-simulator", any(feature = "radio-device-echo", feature = "radio-device-lora-sx1262")),
))]
compile_error!("Only one radio implementation feature can be enabled at a time");

#[cfg(all(
    not(test),
    not(any(feature = "radio-device-echo", feature = "radio-device-lora-sx1262", feature = "radio-device-simulator"))
))]
compile_error!("At least one radio implementation feature must be enabled");

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
mod radio_message;
mod relay_manager;
mod rx_handler;
mod tx_scheduler;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use log::log;
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

// Re-export types from radio_message module
pub(crate) use radio_message::MessageType;
pub use radio_message::{EchoResultItem, EchoResultIterator, RadioMessage, RadioPacket};

//Hardware dependent constants, that affect compatibility of a node
const RADIO_PACKET_SIZE: usize = 200;
const RADIO_MAX_MESSAGE_SIZE: usize = 2000;

//Hardware dependent constants, that only affect efficiency of a node, but does not result incompatibility
const CONNECTION_MATRIX_SIZE: usize = 100;
const INCOMING_PACKET_BUFFER_SIZE: usize = 50;
const WAIT_POOL_SIZE: usize = 10;

#[cfg(feature = "radio-device-simulator")]
const MAX_NODE_COUNT: usize = 1000;

#[cfg(not(feature = "radio-device-simulator"))]
const MAX_NODE_COUNT: usize = 1;

/// Configuration for radio transmission timing
///
/// Controls the delays between packet and message transmissions to manage
/// radio spectrum usage and avoid interference.
pub struct RadioConfiguration {
    /// Delay in seconds between individual packets within a message
    pub delay_between_tx_packets: u8,
    /// Delay in seconds between complete message transmissions
    pub delay_between_tx_messages: u8,
    pub echo_request_minimal_interval: u32,
    pub echo_messages_target_interval: u8,
    pub echo_gathering_timeout: u8,
    pub relay_position_delay: u8,
    pub scoring_matrix: ScoringMatrix,
}
pub enum SendMessageError {
    ChannelFull,
    NotInited,
}

pub enum ReceiveMessageError {
    NotInited,
}

const OUTGOING_MESSAGE_QUEUE_SIZE: usize = 10;
type OutgoingMessageQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioMessage, OUTGOING_MESSAGE_QUEUE_SIZE>;
type OutgoingMessageQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioMessage, OUTGOING_MESSAGE_QUEUE_SIZE>;
type OutgoingMessageQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioMessage, OUTGOING_MESSAGE_QUEUE_SIZE>;

#[cfg(feature = "embedded")]
static OUTGOING_MESSAGE_QUEUE: OutgoingMessageQueue = Channel::new();

const INCOMING_MESSAGE_QUEUE_SIZE: usize = 10;
type IncomingMessageQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioMessage, INCOMING_MESSAGE_QUEUE_SIZE>;
type IncomingMessageQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioMessage, INCOMING_MESSAGE_QUEUE_SIZE>;
type IncomingMessageQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioMessage, INCOMING_MESSAGE_QUEUE_SIZE>;

#[cfg(feature = "embedded")]
static INCOMING_MESSAGE_QUEUE: IncomingMessageQueue = Channel::new();

const TX_PACKET_QUEUE_SIZE: usize = 16;
type TXPacketQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioPacket, TX_PACKET_QUEUE_SIZE>;
type TxPacketQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioPacket, TX_PACKET_QUEUE_SIZE>;
type TxPacketQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioPacket, TX_PACKET_QUEUE_SIZE>;

#[cfg(feature = "embedded")]
static TX_PACKET_QUEUE: TXPacketQueue = Channel::new();

const RX_PACKET_QUEUE_SIZE: usize = 16;
type RxPacketQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, ReceivedPacket, RX_PACKET_QUEUE_SIZE>;
type RxPacketQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, ReceivedPacket, RX_PACKET_QUEUE_SIZE>;
type RxPacketQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, ReceivedPacket, RX_PACKET_QUEUE_SIZE>;

#[cfg(feature = "embedded")]
static RX_PACKET_QUEUE: RxPacketQueue = Channel::new();

const RX_STATE_QUEUE_SIZE: usize = 2;
type RxStateQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RxState, RX_STATE_QUEUE_SIZE>;
type RxStateQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RxState, RX_STATE_QUEUE_SIZE>;
type RxStateQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RxState, RX_STATE_QUEUE_SIZE>;

#[cfg(feature = "embedded")]
static RX_STATE_QUEUE: RxStateQueue = Channel::new();

const PROCESS_RESULT_QUEUE_SIZE: usize = 10;
type ProcessResultQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, MessageProcessingResult, PROCESS_RESULT_QUEUE_SIZE>;
type ProcessResultQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, MessageProcessingResult, PROCESS_RESULT_QUEUE_SIZE>;

#[cfg(feature = "embedded")]
static PROCESS_RESULT_QUEUE: ProcessResultQueue = Channel::new();
pub struct ScoringMatrix {
    pub(crate) matrix: [[u8; 4]; 4],
    pub(crate) poor_limit: u8,
    pub(crate) excellent_limit: u8,
    pub(crate) relay_score_limit: u8,
}

impl ScoringMatrix {
    pub const fn new(matrix: [[u8; 4]; 4], poor_limit: u8, excellent_limit: u8, relay_score_limit: u8) -> Self {
        Self {
            matrix,
            poor_limit,
            excellent_limit,
            relay_score_limit,
        }
    }

    //Encoded form has a compressed representation of the matrix and limits
    //If we call the rows of the matrix with numbers (from 1 to 4) and columns with letters (A to D), the encoded form is as follows:
    // B1: 4 bits
    // C1: 4 bits
    // D1: 4 bits
    // C2: 4 bits
    // D2: 4 bits
    // D3: 4 bits
    // All the other cells of the scoring matrix are 0
    // Next the limits are also represented in the encoded form:
    // Poor limit: 6 bits
    // Excellent limit: 6 bits
    // Relay score limit: 4 bits
    // The total size of the encoded form is 5 bytes
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

// dbg_limits helper removed; tests access fields directly.

enum RxState {
    PacketedRxInProgress(u8, u8), // (packet_index, total_packet_count)
    PacketedRxEnded,
}

/// Trait defining the interface for radio device implementations
///
/// This trait provides a standard interface for different radio device types,
/// enabling abstraction over various radio hardware implementations.
///
/// Note: Initialization is not part of this trait as it is highly implementation-dependent
/// with different hardware requirements, pin configurations, and setup parameters.
///
/// # Examples
/// ```rust,ignore
/// use moonblokz_radio_lib::{RadioDeviceTrait, RadioPacket, RadioDeviceError};
///
/// async fn send_packet<T: RadioDeviceTrait>(radio: &mut T, packet: &RadioPacket) -> Result<(), RadioDeviceError> {
///     if !radio.is_initialized() {
///         return Err(RadioDeviceError::InitializationFailed);
///     }
///     
///     // Check channel before transmitting
///     if !radio.do_cad().await? {
///         radio.send_message(packet).await
///     } else {
///         Err(RadioDeviceError::TransmissionFailed) // Channel busy
///     }
/// }
/// ```

pub enum MessageProcessingResult {
    RequestedBlockNotFound(u32),                    //this node not has the block with the give sequence
    RequestedBlockFound(RadioMessage),              // this node has the requested block
    RequestedBlockPartFound(RadioMessage, u32, u8), // this node has the requested block part (RadioMessage, payload checksum, part_index)
    NewBlockAdded(RadioMessage),                    // a new block has been added to the node
    NewTransactionAdded(RadioMessage),              // a new transaction has been added to the node
    SendReplyTransaction(RadioMessage),             // a reply transaction has been sent
    NewSupportAdded(RadioMessage),                  // a new support has been added to the node
}

struct ReceivedPacket {
    packet: RadioPacket,
    link_quality: u8,
}

enum RadioCommunicationManagerState {
    Uninitialized,
    Initialized {
        outgoing_message_queue_sender: OutgoingMessageQueueSender,
        incoming_message_queue_receiver: IncomingMessageQueueReceiver,
    },
}

pub struct RadioCommunicationManager {
    state: RadioCommunicationManagerState,
}

//TODO: generic Constant Params and consolidate new and initialize methods

impl RadioCommunicationManager {
    pub const fn new() -> Self {
        // Initialize the RelayManager and TxScheduler
        RadioCommunicationManager {
            state: RadioCommunicationManagerState::Uninitialized,
        }
    }

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
        };
        Ok(())
    }

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

    pub async fn receive_message(&self) -> Result<RadioMessage, ReceiveMessageError> {
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

    pub fn report_message_processing_status(&self, _message: &RadioMessage, _success: bool) {
        // TODO: Implementation for reporting the status of message processing
    }
}
#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use futures::executor::block_on;

    #[test]
    fn radio_config_constructs() {
        let _cfg = RadioConfiguration {
            delay_between_tx_packets: 0,
            delay_between_tx_messages: 0,
            echo_request_minimal_interval: 1,
            echo_messages_target_interval: 1,
            echo_gathering_timeout: 1,
            relay_position_delay: 1,
            scoring_matrix: ScoringMatrix::new([[1, 1, 1, 1]; 4], 16, 48, 0),
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
}

#![cfg_attr(not(feature = "std"), no_std)]
#![allow(async_fn_in_trait)] // We control the usage of this trait

#[cfg(any(
    all(feature = "radio-device-echo", any(feature = "radio-device-lora-sx1262")),
    all(feature = "radio-device-lora-sx1262", any(feature = "radio-device-echo")),
))]
compile_error!("Only one radio implementation feature can be enabled at a time");

#[cfg(not(any(feature = "radio-device-echo", feature = "radio-device-lora-sx1262")))]
compile_error!("At least one radio implementation feature must be enabled");

#[cfg(feature = "radio-device-lora-sx1262")]
pub mod radio_device_lora_sx1262;

#[cfg(feature = "radio-device-echo")]
pub mod radio_device_echo;

#[cfg(feature = "radio-device-lora-sx1262")]
use crate::radio_device_lora_sx1262::RadioDevice;
#[cfg(feature = "radio-device-lora-sx1262")]
use crate::radio_device_lora_sx1262::radio_device_task;

#[cfg(feature = "radio-device-echo")]
use crate::radio_device_echo::RadioDevice;
#[cfg(feature = "radio-device-echo")]
use crate::radio_device_echo::radio_device_task;

use crate::tx_scheduler::tx_scheduler_task;
use embassy_executor::Spawner;
mod relay_manager;
mod rx_handler;
mod tx_scheduler;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use log::log;
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

//Hardware dependent constants, that affect compatibility of a node
const RADIO_PACKET_SIZE: usize = 200;
const RADIO_MAX_MESSAGE_SIZE: usize = 2000;

//Hardware dependent constants, that only affect efficiency of a node, but does not result incompatibility
const CONNECTION_MATRIX_SIZE: usize = 100;
const INCOMMING_PACKET_BUFFER_SIZE: usize = 50;
const WAIT_POOL_SIZE: usize = 10;

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
    pub echo_request_additional_interval_by_neighbor: u32,
}
pub enum SendMessageError {
    ChannelFull,
    NotInited,
}

pub enum ReceiveMessageError {
    NotInited,
}

const OUTGOING_MESSAGE_QUEUE_SIZE: usize = 10;
type OutgoingMessageQeueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioMessage, OUTGOING_MESSAGE_QUEUE_SIZE>;
type OutgoingMessageQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioMessage, OUTGOING_MESSAGE_QUEUE_SIZE>;
type OutgoingMessageQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioMessage, OUTGOING_MESSAGE_QUEUE_SIZE>;

#[cfg(feature = "embedded")]
static OUTGOING_MESSAGE_QUEUE: OutgoingMessageQeueue = Channel::new();

const INCOMMING_MESSAGE_QUEUE_SIZE: usize = 10;
type IncommingMessageQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioMessage, INCOMMING_MESSAGE_QUEUE_SIZE>;
type IncommingMessageQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioMessage, INCOMMING_MESSAGE_QUEUE_SIZE>;
type IncommingMessageQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioMessage, INCOMMING_MESSAGE_QUEUE_SIZE>;

#[cfg(feature = "embedded")]
static INCOMMING_MESSAGE_QUEUE: IncommingMessageQueue = Channel::new();

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

const process_result_queue_size: usize = 10;
type ProcessResultQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioMessage, process_result_queue_size>;
type ProcessResultQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioMessage, process_result_queue_size>;
type ProcessResultQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioMessage, process_result_queue_size>;

#[cfg(feature = "embedded")]
static PROCESS_RESULT_QUEUE: ProcessResultQueue = Channel::new();

enum RxState {
    PACKETED_RX_IN_PROGRESS(u8, u8), // (packet_index, total_packet_count)
    PACKETED_RX_ENDED,
}

/// Trait defining the interface for radio device implementations
///
/// This trait provides a standard interface for different radio device types,
/// enabling abstraction over variousradio harware implementations.
///
/// Note: Initialization is not part of this trait as it is highly implementation-dependent
/// with different hardware requirements, pin configurations, and setup parameters.
///
/// # Examples
/// ```rust,no_run
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

pub struct RadioMessage {
    pub(crate) payload: [u8; RADIO_MAX_MESSAGE_SIZE],
    pub(crate) length: usize,
}

enum MessageType {
    RequestEcho = 0x01,
    Echo = 0x02,
    RequestFullBlock = 0x03,
    RequestBlockPart = 0x04,
    AddBlock = 0x05,
    AddTransaction = 0x06,
    GetMempoolState = 0x07,
    Support = 0x08,
}

impl RadioMessage {
    pub fn new_from_single_packet(packet: RadioPacket) -> RadioMessage {
        let message_type = packet.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            // Handle AddBlock and AddTransaction messages
            let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
            payload[0] = message_type;
            payload[0..13].copy_from_slice(&packet.data[0..13]);
            payload[13..packet.length - 2].copy_from_slice(&packet.data[15..packet.length]);

            RadioMessage {
                payload,
                length: packet.length + 1,
            }
        } else {
            let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
            payload[..packet.length].copy_from_slice(&packet.data[..packet.length]);
            RadioMessage {
                payload,
                length: packet.length,
            }
        }
    }

    pub(crate) fn new_empty_message() -> Self {
        // Create a new empty RadioMessage with default values
        RadioMessage {
            payload: [0u8; RADIO_MAX_MESSAGE_SIZE],
            length: 0,
        }
    }

    pub(crate) fn add_packet(&mut self, packet: &RadioPacket) {
        let total_packet_count = packet.total_packet_count();
        if total_packet_count == packet.packet_index() + 1 {
            self.payload[0..13].copy_from_slice(&packet.data[0..13]);
            self.length = 13 + (RADIO_PACKET_SIZE - 15) * (total_packet_count as usize - 1) + packet.length - 15;
        }

        let start_index = 13 + (RADIO_PACKET_SIZE - 15) * packet.packet_index() as usize;
        let end_index = match packet.packet_index() == packet.total_packet_count() - 1 {
            true => start_index + packet.length - 15,
            false => start_index + (RADIO_PACKET_SIZE - 15),
        };

        self.payload[start_index..end_index].copy_from_slice(&packet.data[15..packet.length]);
    }

    pub fn new_request_echo(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for echo requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestEcho as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len() + 1].copy_from_slice(&node_id_bytes);
        RadioMessage { payload, length: 5 }
    }

    pub fn new_echo(node_id: u32, target_node_id: u32, link_quality: u8) -> Self {
        // Create a new RadioMessage with a specific message type for echo responses
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::Echo as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let target_node_id_bytes = target_node_id.to_le_bytes();
        payload[5..5 + target_node_id_bytes.len()].copy_from_slice(&target_node_id_bytes);
        payload[5 + target_node_id_bytes.len()] = link_quality;

        RadioMessage { payload, length: 10 }
    }

    pub fn new_request_full_block(node_id: u32, sequence: u32) -> Self {
        // Create a new RadioMessage with a specific message type for full block requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestFullBlock as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);

        RadioMessage { payload, length: 9 }
    }

    pub fn new_request_block_part(node_id: u32, sequence: u32, payload_checksum: u32, packet_number: u8) -> Self {
        // Create a new RadioMessage with a specific message type for block part requests
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::RequestBlockPart as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);
        payload[13] = packet_number;

        RadioMessage { payload, length: 14 }
    }

    pub fn new_add_block(node_id: u32, sequence: u32, payload_checksum: u32, payload: &[u8]) -> Self {
        // Create a new RadioMessage with a specific message type for adding blocks
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::RequestBlockPart as u8; // Reusing the same type for simplicity
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        full_payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);

        // Copy the actual payload data into the message
        full_payload[13..13 + payload.len()].copy_from_slice(payload);

        RadioMessage {
            payload: full_payload,
            length: 13 + payload.len(),
        }
    }

    pub fn new_add_transaction(node_id: u32, anchor_sequence: u32, payload_checksum: u32, payload: &[u8]) -> Self {
        // Create a new RadioMessage with a specific message type for adding transactions
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::AddTransaction as u8;
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..1 + node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = anchor_sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let checksum_bytes = payload_checksum.to_le_bytes();
        full_payload[9..9 + checksum_bytes.len()].copy_from_slice(&checksum_bytes);

        // Copy the actual payload data into the message
        full_payload[13..13 + payload.len()].copy_from_slice(payload);

        RadioMessage {
            payload: full_payload,
            length: 13 + payload.len(),
        }
    }

    pub fn new_get_mempool_state(node_id: u32) -> Self {
        // Create a new RadioMessage with a specific message type for getting mempool state
        let mut payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        payload[0] = MessageType::GetMempoolState as u8;
        let node_id_bytes = node_id.to_le_bytes();
        payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);

        RadioMessage { payload, length: 5 }
    }

    pub fn add_mempool_item(&mut self, anchor_sequence: u32, payload_checksum: u32) -> Result<(), ()> {
        // Add a mempool item to the message payload
        if self.length + 8 > RADIO_MAX_MESSAGE_SIZE {
            return Err(());
        }

        let sequence_bytes = anchor_sequence.to_le_bytes();
        let checksum_bytes = payload_checksum.to_le_bytes();
        self.payload[self.length..self.length + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        self.payload[self.length + sequence_bytes.len()..self.length + sequence_bytes.len() + checksum_bytes.len()].copy_from_slice(&checksum_bytes);
        self.length += 8;

        Ok(())
    }

    pub fn new_support(node_id: u32, sequence: u32, supporter_node: u32, signature: &[u8]) -> Self {
        // Create a new RadioMessage with a specific message type for support requests
        let mut full_payload = [0u8; RADIO_MAX_MESSAGE_SIZE];
        full_payload[0] = MessageType::Support as u8;
        let node_id_bytes = node_id.to_le_bytes();
        full_payload[1..node_id_bytes.len()].copy_from_slice(&node_id_bytes);
        let sequence_bytes = sequence.to_le_bytes();
        full_payload[5..5 + sequence_bytes.len()].copy_from_slice(&sequence_bytes);
        let supporter_node_bytes = supporter_node.to_le_bytes();
        full_payload[9..9 + supporter_node_bytes.len()].copy_from_slice(&supporter_node_bytes);
        full_payload[13..13 + signature.len()].copy_from_slice(signature);

        RadioMessage {
            payload: full_payload,
            length: 13 + signature.len(),
        }
    }

    pub fn message_type(&self) -> u8 {
        if self.payload.is_empty() {
            return 0; // Default to 0 if payload is empty
        }
        self.payload[0]
    }

    fn get_packet_count(&self) -> usize {
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            let payload_length = self.length - 13; // Exclude header
            return (payload_length + RADIO_PACKET_SIZE - 16) / (RADIO_PACKET_SIZE - 15);
        } else {
            return 1;
        }
    }

    fn get_packet(&self, packet_number: usize) -> Option<RadioPacket> {
        if self.message_type() == MessageType::AddBlock as u8 || self.message_type() == MessageType::AddTransaction as u8 {
            let total_packets = self.get_packet_count();
            if packet_number >= total_packets {
                return None;
            }

            let start_index = 13 + packet_number * (RADIO_PACKET_SIZE - 15);
            let end_index = start_index + (RADIO_PACKET_SIZE - 15);
            let packet_data = &self.payload[start_index..end_index.min(self.length)];
            let packet_header = &self.payload[0..13]; // First 13 bytes are the header

            let mut data = [0u8; RADIO_PACKET_SIZE];
            data[..13].copy_from_slice(packet_header);
            data[13] = total_packets as u8; // Total packet count in the header
            data[14] = packet_number as u8; // Packet number in the header
            data[15..packet_data.len() + 15].copy_from_slice(packet_data);

            Some(RadioPacket {
                data,
                length: packet_data.len() + 15,
            })
        } else {
            let mut data = [0u8; RADIO_PACKET_SIZE];
            data[..self.length].copy_from_slice(&self.payload[0..self.length]);
            Some(RadioPacket { data, length: self.length })
        }
    }

    pub fn sender_node_id(&self) -> u32 {
        // Extract the sender node ID from the message payload
        let mut node_id_bytes = [0u8; 4];
        node_id_bytes.copy_from_slice(&self.payload[1..5]);
        u32::from_le_bytes(node_id_bytes)
    }

    pub fn length(&self) -> usize {
        self.length
    }

    pub(crate) fn get_echo_data(&self) -> Option<(u8)> {
        // Extract echo data from the message payload
        if self.message_type() != MessageType::Echo as u8 {
            return None;
        }
        if self.length < 6 {
            return None; // Not enough data for echo
        }

        let link_quality = self.payload[5];

        Some(link_quality)
    }

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
}

pub struct RadioPacket {
    data: [u8; RADIO_PACKET_SIZE],
    length: usize,
}

impl RadioPacket {
    fn message_type(&self) -> u8 {
        self.data[0]
    }

    pub(crate) fn total_packet_count(&self) -> u8 {
        let message_type = self.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            if self.length < 15 {
                return 0; // Not enough data for packet count
            }
            return self.data[13];
        } else {
            return 1; // For other message types, always 1 packet
        }
    }

    pub(crate) fn packet_index(&self) -> u8 {
        let message_type = self.message_type();
        if message_type == MessageType::AddBlock as u8 || message_type == MessageType::AddTransaction as u8 {
            if self.length < 15 {
                return 0; // Not enough data for packet index
            }
            return self.data[14];
        } else {
            return 0; // For other message types, always 0
        }
    }
}

struct ReceivedPacket {
    packet: RadioPacket,
    link_quality: u8,
}

enum RadioCommunicationManagerState {
    Uninitialized,
    Initialized {
        radio_config: RadioConfiguration,
        outgoing_message_queue_sender: OutgoingMessageQueueSender,
        incomming_message_queue_receiver: IncommingMessageQueueReceiver,
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
            &INCOMMING_MESSAGE_QUEUE,
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
        let outgoing_message_queue_temp: OutgoingMessageQeueue = Channel::new();
        let outgoing_message_queue_static: &'static OutgoingMessageQeueue = Box::leak(Box::new(outgoing_message_queue_temp));

        let incomming_message_queue_temp: IncommingMessageQueue = Channel::new();
        let incomming_message_queue_static: &'static IncommingMessageQueue = Box::leak(Box::new(incomming_message_queue_temp));

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
            incomming_message_queue_static,
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
        outgoing_message_queue: &'static OutgoingMessageQeueue,
        incomming_message_queue: &'static IncommingMessageQueue,
        process_result_queue: &'static ProcessResultQueue,
        tx_packet_queue: &'static TXPacketQueue,
        rx_packet_queue: &'static RxPacketQueue,
        rx_state_queue: &'static RxStateQueue,
        own_node_id: u32,
        rng_seed: u64,
    ) -> Result<(), ()> {
        let mut rng = WyRand::seed_from_u64(rng_seed);

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

        let tx_scheduler_task_result = spawner.spawn(crate::tx_scheduler_task(
            outgoing_message_queue.receiver(),
            rx_state_queue.receiver(),
            tx_packet_queue.sender(),
            radio_config.delay_between_tx_packets,
            radio_config.delay_between_tx_messages,
            rng.next_u64(),
        ));
        if tx_scheduler_task_result.is_err() {
            return Err(());
        }
        log!(log::Level::Debug, "TX Scheduler task spawned");

        let rx_handler_task_result = spawner.spawn(rx_handler::rx_handler_task(
            incomming_message_queue.sender(),
            outgoing_message_queue.sender(),
            rx_packet_queue.receiver(),
            rx_state_queue.sender(),
            process_result_queue.receiver(),
            radio_config.echo_request_minimal_interval,
            radio_config.echo_request_additional_interval_by_neighbor,
            own_node_id,
            rng.next_u64(),
        ));
        if rx_handler_task_result.is_err() {
            return Err(());
        }
        log!(log::Level::Debug, "RX Handler task spawned");
        log!(log::Level::Info, "Radio communication initialized");

        self.state = RadioCommunicationManagerState::Initialized {
            radio_config,
            outgoing_message_queue_sender: outgoing_message_queue.sender(),
            incomming_message_queue_receiver: incomming_message_queue.receiver(),
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
        let incomming_message_queue_receiver = match &self.state {
            RadioCommunicationManagerState::Uninitialized => {
                return Err(ReceiveMessageError::NotInited);
            }
            RadioCommunicationManagerState::Initialized {
                incomming_message_queue_receiver,
                ..
            } => incomming_message_queue_receiver,
        };
        return Ok(incomming_message_queue_receiver.receive().await);
    }

    pub fn report_message_processing_status(&self, _message: &RadioMessage, _success: bool) {
        // TODO: Implementation for reporting the status of message processing
    }
}
#[cfg(test)]
mod tests {}

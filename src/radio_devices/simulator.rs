//! # Radio Device Simulator - Testing and Development Mock
//!
//! This module provides a simulated radio device implementation for testing and development
//! without requiring physical hardware. It mimics the behavior of a real radio transceiver
//! with configurable timing characteristics and channel activity detection.
//!
//! ## Architecture
//!
//! The simulator uses channel-based communication to emulate radio behavior:
//! - **Output Queue**: Sends packets from this node to the network simulator
//! - **Input Queue**: Receives packets from the network simulator to this node
//! - **TX/RX Integration**: Connects to the standard packet queues used by the library
//! - **CAD Simulation**: Emulates channel activity detection with random delays
//!
//! ## Key Components
//!
//! - **RadioDevice**: Simulated radio state with input/output queues
//!   - Manages packet transmission and reception
//!   - Simulates CAD (Channel Activity Detection) behavior
//!   - Thread-safe queue operations using Embassy channels
//!
//! - **RadioOutputQueue**: Messages sent to network simulator
//!   - SendPacket: transmit a packet to the simulated network
//!   - RequestCAD: check if channel is busy before transmitting
//!
//! - **RadioInputQueue**: Messages received from network simulator
//!   - ReceivePacket: incoming packet from another simulated node
//!   - CADResponse: result of channel activity check (busy/clear)
//!
//! - **radio_device_task**: Async task implementing the simulator logic
//!   - Processes outgoing packets from TX scheduler
//!   - Forwards incoming packets to RX handler
//!   - Simulates timing delays for CAD operations
//!
//! ## Timing Simulation
//!
//! The simulator adds realistic timing characteristics:
//! - CAD minimum wait: 300ms baseline delay
//! - CAD random jitter: 0-200ms additional delay
//! - Emulates real hardware timing constraints
//!
//! ## Design Considerations
//!
//! - Pool size supports MAX_NODE_COUNT concurrent simulated nodes
//! - Queue sizes configurable (default: 10 messages per queue)
//! - Random number generator for timing jitter
//! - Embassy async task model matches hardware implementation
//! - Compatible with the same API as physical radio devices

use crate::MAX_NODE_COUNT;
use crate::RadioPacket;
use crate::ReceivedPacket;
use crate::RxPacketQueueSender;
use crate::TxPacketQueueReceiver;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use log::{Level, log};
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

/// Minimum wait time in milliseconds after CAD detects a busy channel
///
/// When the simulated CAD operation reports the channel is busy, the radio
/// waits at least this duration before requesting CAD again.
const CAD_MINIMAL_WAIT_TIME: u64 = 300;

/// Maximum additional random wait time in milliseconds after CAD
///
/// Added to CAD_MINIMAL_WAIT_TIME to create randomized backoff, mimicking
/// real-world behavior and helping to avoid synchronized collisions when
/// multiple simulated nodes detect the same busy channel.
const CAD_MAX_ADDITIONAL_WAIT_TIME: u64 = 200;

/// Size of the radio output message queue
///
/// Determines how many outgoing messages (SendPacket or RequestCAD) can be
/// buffered before blocking. Default is 10 messages.
const RADIO_OUTPUT_QUEUE_SIZE: usize = 10;

/// Radio output queue type
///
/// Channel for sending messages from the simulated radio to the network simulator.
/// Uses critical section mutex for thread-safe access.
pub type RadioOutputQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioOutputMessage, RADIO_OUTPUT_QUEUE_SIZE>;

/// Radio output queue receiver type
///
/// Used by the network simulator to receive outgoing radio messages.
pub type RadioOutputQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioOutputMessage, RADIO_OUTPUT_QUEUE_SIZE>;

/// Radio output queue sender type
///
/// Used by the radio device to send messages to the network simulator.
pub type RadioOutputQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioOutputMessage, RADIO_OUTPUT_QUEUE_SIZE>;

/// Size of the radio input message queue
///
/// Determines how many incoming messages (ReceivePacket or CADResponse) can be
/// buffered before blocking. Default is 10 messages.
const RADIO_INPUT_QUEUE_SIZE: usize = 10;

/// Radio input queue type
///
/// Channel for receiving messages from the network simulator to the simulated radio.
/// Uses critical section mutex for thread-safe access.
pub type RadioInputQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioInputMessage, RADIO_INPUT_QUEUE_SIZE>;

/// Radio input queue receiver type
///
/// Used by the radio device to receive messages from the network simulator.
pub type RadioInputQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioInputMessage, RADIO_INPUT_QUEUE_SIZE>;

/// Radio input queue sender type
///
/// Used by the network simulator to send messages to the radio device.
pub type RadioInputQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioInputMessage, RADIO_INPUT_QUEUE_SIZE>;

/// Messages sent from simulated radio to network simulator
///
/// These messages represent operations the radio wants to perform on the
/// simulated network.
#[cfg_attr(feature = "std", derive(Debug))]
pub enum RadioOutputMessage {
    /// Request to transmit a packet to the network
    SendPacket(RadioPacket),
    /// Request to check channel activity (CAD) before transmission
    RequestCAD,
}

/// Messages received by simulated radio from network simulator
///
/// These messages represent events from the simulated network affecting
/// this radio device.
#[cfg_attr(feature = "std", derive(Debug))]
pub enum RadioInputMessage {
    /// An incoming packet received from another node in the network
    ReceivePacket(ReceivedPacket),
    /// Response to a CAD request indicating if the channel is busy
    /// - `true`: channel is busy, transmission should be delayed
    /// - `false`: channel is clear, safe to transmit
    CADResponse(bool),
}

/// Simulated radio device task - handles TX/RX with network simulator
///
/// This async task implements a simulated radio device that communicates with
/// a network simulator through input/output queues. It provides realistic CAD
/// behavior with random backoff timing.
///
/// # Task Pool
/// Pool size is MAX_NODE_COUNT to support multiple simulated radio instances
/// running concurrently in testing scenarios.
///
/// # Arguments
/// * `radio_device` - RadioDevice instance with input/output queues
/// * `tx_receiver` - Channel receiver for outgoing packets from TX scheduler
/// * `rx_sender` - Channel sender for incoming packets to RX handler
/// * `own_node_id` - This node's unique identifier for logging
/// * `rng_seed` - Seed for random number generator (used for CAD backoff)
///
/// # Behavior
/// - Races between receiving from network simulator and transmitting packets
/// - On input: Forwards ReceivePacket to RX handler, ignores unexpected CADResponse
/// - On TX: Performs CAD loop (request CAD, wait if busy, transmit when clear)
/// - Handles interleaved packets during CAD wait period with node identification in logs
///
/// # CAD Loop
/// 1. Send RequestCAD to network simulator
/// 2. Wait for CADResponse or interleaved ReceivePacket
/// 3. If busy: wait random duration (CAD_MINIMAL + random jitter) and retry
/// 4. If clear: send packet and exit loop
/// 5. If packet received during wait: forward to RX and retry CAD
///
/// # Network Simulator Integration
/// - Output queue: sends RequestCAD and SendPacket to simulator
/// - Input queue: receives CADResponse and ReceivePacket from simulator
/// - Simulator is responsible for implementing network topology and packet routing
#[embassy_executor::task(pool_size = MAX_NODE_COUNT)]
pub async fn radio_device_task(radio_device: RadioDevice, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender, own_node_id: u32, rng_seed: u64) {
    log!(Level::Info, "[{}] Simulated radio device task started", own_node_id);
    let mut rng = WyRand::seed_from_u64(rng_seed);
    loop {
        let mut next_cad = true;
        match select(radio_device.input_queue_receiver.receive(), tx_receiver.receive()).await {
            Either::First(message) => match message {
                RadioInputMessage::ReceivePacket(pkt) => {
                    log!(Level::Trace, "[{}] Received packet: {:?}", own_node_id, pkt);
                    rx_sender.send(pkt).await;
                }
                RadioInputMessage::CADResponse(_busy) => {
                    log!(Level::Warn, "[{}] Not waiting for CAD response. Dropping.", own_node_id);
                }
            },
            Either::Second(packet) => loop {
                log!(Level::Trace, "[{}] Requesting CAD", own_node_id);
                if next_cad {
                    radio_device.output_queue_sender.send(RadioOutputMessage::RequestCAD).await;
                }

                match radio_device.input_queue_receiver.receive().await {
                    RadioInputMessage::ReceivePacket(pkt) => {
                        log!(Level::Trace, "[{}] Received packet: {:?}", own_node_id, pkt);
                        rx_sender.send(pkt).await;
                        next_cad = false;
                    }
                    RadioInputMessage::CADResponse(busy) => {
                        log!(Level::Trace, "[{}] Received CAD response: busy={}", own_node_id, busy);
                        next_cad = true;
                        if busy {
                            Timer::after(embassy_time::Duration::from_millis(
                                CAD_MINIMAL_WAIT_TIME + rng.next_u64() % CAD_MAX_ADDITIONAL_WAIT_TIME,
                            ))
                            .await;
                            continue;
                        } else {
                            log!(Level::Trace, "[{}] Channel clear, sending packet: {:?}", own_node_id, packet);
                            radio_device.output_queue_sender.send(RadioOutputMessage::SendPacket(packet)).await;
                            break;
                        }
                    }
                }
            },
        }
    }
}

/// Simulated radio device - mock radio for testing
///
/// Represents a simulated radio device that communicates with a network simulator
/// through message queues. This allows testing radio communication logic without
/// physical hardware or even hardware simulation complexity.
///
/// # Architecture
/// - Sends outgoing commands (SendPacket, RequestCAD) to network simulator
/// - Receives incoming events (ReceivePacket, CADResponse) from network simulator
/// - Network simulator handles all topology, routing, and channel simulation
///
/// # Queue Management
/// - Output queue: buffered channel to simulator (RADIO_OUTPUT_QUEUE_SIZE)
/// - Input queue: buffered channel from simulator (RADIO_INPUT_QUEUE_SIZE)
/// - Non-blocking send/receive operations
///
/// # Use Cases
/// - Multi-node network testing without hardware
/// - Testing relay logic and message routing
/// - Simulating packet loss and collisions
/// - Development without embedded hardware
#[cfg_attr(feature = "std", derive(Debug))]
pub struct RadioDevice {
    /// Sender for outgoing messages to the network simulator
    output_queue_sender: RadioOutputQueueSender,
    /// Receiver for incoming messages from the network simulator
    input_queue_receiver: RadioInputQueueReceiver,
}

impl RadioDevice {
    /// Creates a new simulated radio device
    ///
    /// Constructs a radio device that communicates with a network simulator
    /// through the provided input and output queue endpoints.
    ///
    /// # Parameters
    /// * `output_queue_sender` - Sender for messages to the network simulator
    /// * `input_queue_receiver` - Receiver for messages from the network simulator
    ///
    /// # Returns
    /// A new RadioDevice instance ready for use with the network simulator
    ///
    /// # Example
    /// ```rust,ignore
    /// use embassy_sync::channel::Channel;
    /// use moonblokz_radio_lib::radio_device_simulator::*;
    ///
    /// static OUTPUT_QUEUE: RadioOutputQueue = Channel::new();
    /// static INPUT_QUEUE: RadioInputQueue = Channel::new();
    ///
    /// let radio = RadioDevice::with(
    ///     OUTPUT_QUEUE.sender(),
    ///     INPUT_QUEUE.receiver()
    /// );
    /// ```
    pub const fn with(output_queue_sender: RadioOutputQueueSender, input_queue_receiver: RadioInputQueueReceiver) -> Self {
        RadioDevice {
            output_queue_sender,
            input_queue_receiver,
        }
    }
}

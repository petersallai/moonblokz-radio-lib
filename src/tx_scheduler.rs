//! # TX Scheduler Module
//!
//! Manages transmission timing and packet scheduling to avoid collisions and ensure
//! fair channel access in the radio mesh network.
//!
//! ## Architecture
//!
//! The TX Scheduler is implemented as an async task that:
//! - Receives messages from the outgoing message queue
//! - Fragments messages into packets
//! - Schedules packet transmission with appropriate delays
//! - Monitors RX state to avoid transmitting during multi-packet reception
//!
//! ## Transmission Strategy
//!
//! 1. **Initial Random Delay**: Each message starts with a random delay to
//!    reduce the probability of collisions when multiple nodes want to transmit
//!
//! 2. **Inter-Packet Delay**: Configurable delay between packets of the same message
//!
//! 3. **Inter-Message Delay**: Configurable delay between different messages to ensure
//!    fair channel access and handle duty cycle limitations
//!
//! 4. **RX State Awareness**: Pauses transmission during multi-packet reception to
//!    avoid interrupting incoming messages
//!
//! ## Configuration
//!
//! - `delay_between_tx_packets`: Delay in seconds between individual packets
//! - `delay_between_tx_messages`: Delay in seconds between complete messages
//! - Random jitter is added to prevent systematic collisions

use core::cmp::max;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Timer};
use log::log;
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

use crate::MAX_NODE_COUNT;
use crate::{OutgoingMessageQueueReceiver, RxState, TxPacketQueueSender};

/// Handle RX state updates and update the delay timeout accordingly
fn calculate_rx_state_delay_timeout(rxstate: RxState, rx_state_delay_timeout: Instant, delay_between_packets: u16, own_node_id: u32) -> Instant {
    match rxstate {
        RxState::PacketedRxInProgress(packet_index, total_packet_count) => {
            let remaining_packets = (total_packet_count as u64).saturating_sub(packet_index as u64).max(1);
            let new_delay_timeout = Instant::now() + Duration::from_millis(remaining_packets * delay_between_packets as u64);
            let updated_timeout = max(rx_state_delay_timeout, new_delay_timeout);
            log!(
                log::Level::Trace,
                "[{}] Multi-packet RX in progress: packet {}/{}, TX delayed for: {} ms",
                own_node_id,
                packet_index,
                total_packet_count,
                (updated_timeout - Instant::now()).as_millis()
            );
            updated_timeout
        }
        RxState::PacketedRxEnded => {
            log!(log::Level::Trace, "[{}] Multi-packet RX ended, clearing TX delay", own_node_id);
            Instant::now()
        }
    }
}

/// Send all packets of a message to the radio device
async fn send_message_to_radio(
    message: &crate::RadioMessage,
    radio_device_sender: &TxPacketQueueSender,
    delay_between_packets_duration: Duration,
    own_node_id: u32,
) {
    let packet_count = message.get_packet_count();
    log!(
        log::Level::Trace,
        "[{}] Sending message to radio device: type: {}, packet count: {}",
        own_node_id,
        message.message_type(),
        packet_count
    );

    for i in 0..packet_count {
        // Get the packet to send
        if let Some(packet) = message.get_packet(i) {
            // Attempt to send the packet to the radio device
            match radio_device_sender.try_send(packet) {
                Ok(_) => {
                    // Successfully sent the packet, update last transmission time
                    log!(
                        log::Level::Trace,
                        "[{}] Sent packet to radio device: packet: {}/{}",
                        own_node_id,
                        i + 1,
                        packet_count
                    );
                }
                Err(embassy_sync::channel::TrySendError::Full(packet)) => {
                    log!(
                        log::Level::Warn,
                        "[{}] TX packet queue full, dropping packet. type: {}",
                        own_node_id,
                        packet.message_type()
                    );
                }
            }
            Timer::after(delay_between_packets_duration).await;
        } else {
            log!(
                log::Level::Error,
                "[{}] Failed to get packet {}/{} from message",
                own_node_id,
                i,
                message.get_packet_count()
            );
        }
    }
}

/// TX Scheduler Task
///
/// This task manages the transmission scheduling for radio packets. It ensures that:
/// 1. Messages are transmitted in the order they are received
/// 2. A configurable minimum delay is maintained between transmissions
/// 3. Failed transmissions don't affect the timing of subsequent transmissions
/// 4. Individual packets within a message respect packet-level delays (handled by the radio device)
///
/// The scheduler waits for incoming messages from the outgoing message queue and
/// forwards them to the radio device while respecting timing constraints.
///
/// # Parameters
/// * `outgoing_message_queue_receiver` - Receiver for incoming messages to transmit
/// * `radio_device_sender` - Sender for forwarding packets to the radio device
/// * `delay_between_packets` - Minimum delay in milliseconds between individual packets
/// * `delay_between_messages` - Minimum delay in seconds between message transmissions
/// * `tx_maximum_random_delay` - Maximum jitter in milliseconds added to transmission timing
/// * `own_node_id` - This node's unique identifier for logging
/// * `rng_seed` - Seed for the random number generator to ensure varied timing
///
#[embassy_executor::task(pool_size = MAX_NODE_COUNT)]
pub(crate) async fn tx_scheduler_task(
    outgoing_message_queue_receiver: OutgoingMessageQueueReceiver,
    rx_state_queue_receiver: crate::RxStateQueueReceiver,
    radio_device_sender: TxPacketQueueSender,
    delay_between_packets: u16,
    delay_between_messages: u8,
    tx_maximum_random_delay: u16,
    own_node_id: u32,
    rng_seed: u64,
) -> ! {
    let mut rng = WyRand::seed_from_u64(rng_seed);
    let mut last_tx_time = Instant::now();
    let mut rx_state_delay_timeout = Instant::now();
    let delay_between_messages_duration = Duration::from_secs(delay_between_messages as u64);
    let delay_between_packets_duration = Duration::from_millis(delay_between_packets as u64);
    log!(log::Level::Info, "[{}] TX scheduler task started", own_node_id);
    loop {
        match select(outgoing_message_queue_receiver.receive(), rx_state_queue_receiver.receive()).await {
            Either::First(message) => {
                loop {
                    // Calculate the time since last transmission
                    let elapsed = last_tx_time.elapsed();

                    // If not enough time has passed, wait for the remaining time (saturating)
                    let remaining_message_delay = if elapsed >= delay_between_messages_duration {
                        Duration::from_secs(0)
                    } else {
                        delay_between_messages_duration - elapsed
                    };

                    let remaining_rx_state_timeout = rx_state_delay_timeout.saturating_duration_since(Instant::now());

                    //get the maximum of the remaining delay and the rx state timeout
                    let mut remaining_delay = max(remaining_message_delay, remaining_rx_state_timeout);

                    if remaining_delay > Duration::from_secs(0) {
                        remaining_delay += Duration::from_millis(rng.next_u64() % tx_maximum_random_delay as u64); // Add a random jitter to the delay
                        match select(Timer::after(remaining_delay), rx_state_queue_receiver.receive()).await {
                            Either::First(_) => {
                                // Timer completed, proceed to send message
                                break;
                            }
                            Either::Second(rxstate) => {
                                // RX state update received, recalculate delay
                                rx_state_delay_timeout = calculate_rx_state_delay_timeout(rxstate, rx_state_delay_timeout, delay_between_packets, own_node_id);
                                // Continue loop to recalculate delays
                            }
                        }
                    } else {
                        break;
                    }
                }
                send_message_to_radio(&message, &radio_device_sender, delay_between_packets_duration, own_node_id).await;
                last_tx_time = Instant::now();
            }
            Either::Second(rxstate) => {
                rx_state_delay_timeout = calculate_rx_state_delay_timeout(rxstate, rx_state_delay_timeout, delay_between_packets, own_node_id);
            }
        }
    }
}

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
//! 1. **Initial Random Delay**: Each message starts with a random delay (0-2000ms) to
//!    reduce the probability of collisions when multiple nodes want to transmit
//!
//! 2. **Inter-Packet Delay**: Configurable delay between packets of the same message
//!    to allow other nodes to access the channel
//!
//! 3. **Inter-Message Delay**: Configurable delay between different messages to ensure
//!    fair channel access
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
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use log::log;
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

use crate::MAX_NODE_COUNT;
use crate::{OutgoingMessageQueueReceiver, RxState, TxPacketQueueSender};

const RANDOM_DELAY_MAX: u64 = 500; // Maximum random delay in milliseconds

/// TX Scheduler Task
///
/// This task manages the transmission scheduling for radio packets. It ensures that:
/// 1. Messages are transmitted in the order they are received
/// 2. A configurable minimum delay is maintained between transmissions
/// 3. Failed transmissions don't affect the timing of subsequent transmissions
/// 4. Individual packets within a message respect packet-level delays
///
/// The scheduler waits for incoming messages from the outgoing message queue and
/// forwards them to the radio device while respecting timing constraints.
///
/// # Parameters
/// * `outgoing_message_queue_receiver` - Receiver for incoming messages to transmit
/// * `radio_device_sender` - Sender for forwarding packets to the radio device
/// * `delay_between_packets` - Minimum delay in seconds between individual packets
/// * `delay_between_messages` - Minimum delay in seconds between message transmissions
#[embassy_executor::task(pool_size = MAX_NODE_COUNT)]
pub(crate) async fn tx_scheduler_task(
    outgoing_message_queue_receiver: OutgoingMessageQueueReceiver,
    rx_state_queue_receiver: crate::RxStateQueueReceiver,
    radio_device_sender: TxPacketQueueSender,
    delay_between_packets: u16,
    delay_between_messages: u8,
    rng_seed: u64,
) -> ! {
    let mut rng = WyRand::seed_from_u64(rng_seed);
    let mut last_tx_time = Instant::now();
    let mut rx_state_delay_timeout = Instant::now();
    let delay_between_messages_duration = Duration::from_secs(delay_between_messages as u64);
    let delay_between_packets_duration = Duration::from_millis(delay_between_packets as u64);
    log!(log::Level::Info, "TX scheduler task started");
    loop {
        match select(outgoing_message_queue_receiver.receive(), rx_state_queue_receiver.receive()).await {
            Either::First(message) => {
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
                    remaining_delay += Duration::from_millis(rng.next_u64() % RANDOM_DELAY_MAX); // Add a random jitter to the delay
                    Timer::after(remaining_delay).await;
                }

                let packet_count = message.get_packet_count();
                log!(
                    log::Level::Trace,
                    "Sending message to radio device: type: {}, packet count: {}",
                    message.message_type(),
                    packet_count
                );
                for i in 0..packet_count {
                    // Get the packet to send
                    //                    log!(log::Level::Debug, "Sending packet {}/{}", i + 1, packet_count);
                    if let Some(packet) = message.get_packet(i) {
                        // Attempt to send the packet to the radio device
                        match radio_device_sender.try_send(packet) {
                            Ok(_) => {
                                // Successfully sent the packet, update last transmission time
                                last_tx_time = Instant::now();
                                log!(log::Level::Trace, "Sent packet to radio device: packet: {}/{}", i + 1, packet_count);
                            }
                            Err(embassy_sync::channel::TrySendError::Full(packet)) => {
                                log!(log::Level::Warn, "TX packet queue full, dropping packet. type: {}", packet.message_type());
                            }
                        }
                        Timer::after(delay_between_packets_duration).await;
                    } else {
                        log!(log::Level::Error, "Failed to get packet {}/{} from message", i, message.get_packet_count());
                    }
                }
            }
            Either::Second(rxstate) => match rxstate {
                RxState::PacketedRxInProgress(packet_index, total_packet_count) => {
                    let remaining_packets = (total_packet_count as u64).saturating_sub(packet_index as u64).max(1);
                    let new_delay_timeout = Instant::now() + Duration::from_millis(remaining_packets * delay_between_packets as u64);
                    rx_state_delay_timeout = max(rx_state_delay_timeout, new_delay_timeout);
                    log!(
                        log::Level::Debug,
                        "Multi-packet RX in progress: packet {}/{}, TX delayed for: {} ms",
                        packet_index,
                        total_packet_count,
                        (rx_state_delay_timeout - Instant::now()).as_millis()
                    );
                }
                RxState::PacketedRxEnded => {
                    rx_state_delay_timeout = Instant::now();
                    log!(log::Level::Debug, "Multi-packet RX ended, clearing TX delay");
                }
            },
        }
    }
}

#[cfg(all(test, feature = "std"))]
#[allow(unused_assignments, unused_mut)]
pub(crate) async fn tx_scheduler_step(
    outgoing_message_queue_receiver: OutgoingMessageQueueReceiver,
    rx_state_queue_receiver: crate::RxStateQueueReceiver,
    radio_device_sender: TxPacketQueueSender,
    delay_between_packets: u16,
    delay_between_messages: u8,
    rng_seed: u64,
) {
    // A single-iteration version of the scheduler loop
    let mut last_tx_time = Instant::now();
    let mut rx_state_delay_timeout = Instant::now();
    let delay_between_messages_duration = Duration::from_secs(delay_between_messages as u64);
    let delay_between_packets_duration = Duration::from_millis(delay_between_packets as u64);
    let mut rng = WyRand::seed_from_u64(rng_seed);

    match select(outgoing_message_queue_receiver.receive(), rx_state_queue_receiver.receive()).await {
        Either::First(message) => {
            let elapsed = last_tx_time.elapsed();
            let remaining_message_delay = if elapsed >= delay_between_messages_duration {
                Duration::from_secs(0)
            } else {
                delay_between_messages_duration - elapsed
            };
            let remaining_rx_state_timeout = rx_state_delay_timeout.saturating_duration_since(Instant::now());
            let mut remaining_delay = max(remaining_message_delay, remaining_rx_state_timeout);
            if remaining_delay > Duration::from_secs(0) {
                remaining_delay += Duration::from_millis(rng.next_u64() % RANDOM_DELAY_MAX);
                if remaining_delay > Duration::from_secs(0) {
                    Timer::after(remaining_delay).await;
                }
            }
            let packet_count = message.get_packet_count();
            for i in 0..packet_count {
                log!(log::Level::Debug, "Sending packet {}/{}", i + 1, packet_count);
                if let Some(packet) = message.get_packet(i) {
                    match radio_device_sender.try_send(packet) {
                        Ok(_) => {}
                        Err(embassy_sync::channel::TrySendError::Full(_packet)) => {
                            log!(log::Level::Warn, "TX packet queue full, dropping packet.");
                        }
                    }
                    if delay_between_packets_duration > Duration::from_secs(0) {
                        Timer::after(delay_between_packets_duration).await;
                    }
                } else {
                    log!(log::Level::Error, "Failed to get packet {}/{} from message", i, message.get_packet_count());
                }
            }
        }
        Either::Second(rxstate) => match rxstate {
            RxState::PacketedRxInProgress(packet_index, total_packet_count) => {
                log!(log::Level::Debug, "Received RX state update in progress");
                let remaining_packets = (total_packet_count as u64).saturating_sub(packet_index as u64).max(1);
                let new_delay_timeout = Instant::now() + Duration::from_millis(remaining_packets * delay_between_packets as u64);
                rx_state_delay_timeout = max(rx_state_delay_timeout, new_delay_timeout);
            }
            RxState::PacketedRxEnded => {
                log!(log::Level::Debug, "Received RX state update ended");
                rx_state_delay_timeout = Instant::now();
            }
        },
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use crate::RadioMessage;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::Channel;

    type OutCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::OUTGOING_MESSAGE_QUEUE_SIZE }>;
    type RxStateCh = Channel<CriticalSectionRawMutex, crate::RxState, { crate::RX_STATE_QUEUE_SIZE }>;
    type TxCh = Channel<CriticalSectionRawMutex, crate::RadioPacket, { crate::TX_PACKET_QUEUE_SIZE }>;

    #[test]
    fn sends_all_packets_for_multi_packet_message_with_zero_delays() {
        let outgoing: &'static OutCh = Box::leak(Box::new(Channel::new()));
        let rxstate: &'static RxStateCh = Box::leak(Box::new(Channel::new()));
        let tx: &'static TxCh = Box::leak(Box::new(Channel::new()));

        let out_rx = outgoing.receiver();
        let rxstate_rx = rxstate.receiver();
        let tx_tx = tx.sender();
        let tx_rx = tx.receiver();

        // Build a message that spans multiple packets
        // Per-packet payload size excludes RADIO_MULTI_PACKET_PACKET_HEADER_SIZE bytes
        let part = crate::RADIO_PACKET_SIZE - crate::RADIO_MULTI_PACKET_PACKET_HEADER_SIZE;
        let total_len = part * 2 + 7; // 3 packets total (2 full + 1 partial)
        let payload: Vec<u8> = (0..total_len).map(|i| (i % 251) as u8).collect();
        let msg = RadioMessage::new_add_block(1, 0x1010_2020, &payload);
        let expected_packets = msg.get_packet_count();

        // Enqueue the message for transmission
        outgoing.sender().try_send(msg).unwrap();

        // Run a single scheduler step with zero delays
        futures::executor::block_on(tx_scheduler_step(out_rx, rxstate_rx, tx_tx, 0, 0, 42));

        // Collect packets sent to the radio device
        let mut collected = 0;
        while let Ok(pkt) = tx_rx.try_receive() {
            assert_eq!(pkt.message_type(), crate::MessageType::AddBlock as u8);
            assert_eq!(pkt.packet_index(), collected as u8);
            collected += 1;
        }
        assert_eq!(collected, expected_packets);
    }
}

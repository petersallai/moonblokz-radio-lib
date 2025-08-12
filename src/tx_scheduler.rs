use core::cmp::max;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use log::log;
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

use crate::{OutgoingMessageQueueReceiver, RxState, TxPacketQueueSender};

const RANDOM_DELAY_MAX: u64 = 2000; // Maximum random delay in milliseconds

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
#[cfg_attr(feature = "std", embassy_executor::task(pool_size = 10))]
#[cfg_attr(feature = "embedded", embassy_executor::task(pool_size = 1))]
pub(crate) async fn tx_scheduler_task(
    outgoing_message_queue_receiver: OutgoingMessageQueueReceiver,
    rx_state_queue_receiver: crate::RxStateQueueReceiver,
    radio_device_sender: TxPacketQueueSender,
    delay_between_packets: u8,
    delay_between_messages: u8,
    rng_seed: u64,
) -> ! {
    let mut last_tx_time = Instant::now();
    let mut rx_state_delay_timeout = Instant::now();
    let delay_between_messages_duration = Duration::from_secs(delay_between_messages as u64);
    let delay_between_packets_duration = Duration::from_secs(delay_between_packets as u64);
    let mut rng = WyRand::seed_from_u64(rng_seed);
    log!(log::Level::Info, "TX Scheduler Task started");
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
                for i in 0..packet_count {
                    // Get the packet to send
                    log!(log::Level::Debug, "Sending packet {}/{}", i + 1, packet_count);
                    if let Some(packet) = message.get_packet(i) {
                        // Attempt to send the packet to the radio device
                        match radio_device_sender.try_send(packet) {
                            Ok(_) => {
                                // Successfully sent the packet, update last transmission time
                                last_tx_time = Instant::now();
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
                    let new_delay_timeout = Instant::now() + Duration::from_secs(remaining_packets * delay_between_packets as u64);
                    rx_state_delay_timeout = max(rx_state_delay_timeout, new_delay_timeout);
                }
                RxState::PacketedRxEnded => {
                    rx_state_delay_timeout = Instant::now();
                }
            },
        }
    }
}

use crate::{CONNECTION_MATRIX_SIZE, INCOMMING_PACKET_BUFFER_SIZE, MessageType, RxState};
use embassy_futures::select::{Either3, select3};
use embassy_sync::channel::TrySendError;
use embassy_time::{Instant, Timer};
use log::{Level, log};

use crate::{
    IncommingMessageQueueSender, OutgoingMessageQueueSender, ProcessResultQueueReceiver, RadioMessage, RadioPacket, RxPacketQueueReceiver, RxStateQueueSender,
    relay_manager::RelayManager, wait_pool::WaitPool,
};

struct PacketBufferItem {
    packet: RadioPacket,
    arrival_time: Instant,
}

const PACKET_CHECK_BUFFER_EMPTY_VALUE: u8 = 255;

#[cfg_attr(feature = "std", embassy_executor::task(pool_size = 10))]
#[cfg_attr(feature = "embedded", embassy_executor::task(pool_size = 1))]
pub(crate) async fn rx_handler_task(
    incomming_message_queue_sender: IncommingMessageQueueSender,
    outgoing_message_queue_sender: OutgoingMessageQueueSender,
    rx_packet_queue_receiver: RxPacketQueueReceiver,
    rx_state_queue_sender: RxStateQueueSender,
    process_result_queue_receiver: ProcessResultQueueReceiver,
    rng_seed: u64,
) -> ! {
    let mut packet_buffer: [Option<PacketBufferItem>; INCOMMING_PACKET_BUFFER_SIZE] = [const { None }; INCOMMING_PACKET_BUFFER_SIZE];
    let mut packet_check_buffer: [u8; INCOMMING_PACKET_BUFFER_SIZE] = [PACKET_CHECK_BUFFER_EMPTY_VALUE; INCOMMING_PACKET_BUFFER_SIZE];

    loop {
        let relay_manager = RelayManager::<CONNECTION_MATRIX_SIZE>::new();
        let waitpool = WaitPool::new();

        match select3(
            rx_packet_queue_receiver.receive(),
            process_result_queue_receiver.receive(),
            Timer::after(embassy_time::Duration::from_millis(1000)),
        )
        .await
        {
            Either3::First(received_packet) => {
                let rx_packet = received_packet.packet;
                if rx_packet.total_packet_count() == 1 {
                    let radio_message = RadioMessage::new_from_single_packet(rx_packet);
                    let _ = incomming_message_queue_sender.try_send(radio_message);
                } else if rx_packet.total_packet_count() == 0 {
                    log!(Level::Warn, "Received empty packet, skipping");
                    continue;
                } else {
                    // Handle multi-packet message
                    let packet_index = rx_packet.packet_index() as usize;
                    let total_packet_count = rx_packet.total_packet_count() as usize;

                    if packet_index == total_packet_count - 1 {
                        rx_state_queue_sender.try_send(RxState::PACKETED_RX_ENDED).unwrap_or_else(|_| {
                            log!(Level::Error, "Failed to send PACKETED_RX_ENDED state. The queue is full.");
                        });
                    } else {
                        rx_state_queue_sender
                            .try_send(RxState::PACKETED_RX_IN_PROGRESS(packet_index as u8, total_packet_count as u8))
                            .unwrap_or_else(|_| {
                                log!(Level::Error, "Failed to send PACKETED_RX_IN_PROGRESS state. The queue is full.");
                            });
                    }

                    if packet_index as usize >= INCOMMING_PACKET_BUFFER_SIZE {
                        log!(Level::Error, "Packet index out of bounds: {}", packet_index);
                        continue;
                    }

                    let mut already_received = false;
                    let mut empty_index = PACKET_CHECK_BUFFER_EMPTY_VALUE;

                    for i in 0..INCOMMING_PACKET_BUFFER_SIZE {
                        if let Some(packet) = &packet_buffer[i] {
                            if packet.packet.data[0..15] == rx_packet.data[0..15] {
                                // This packet is already in the buffer, skip it
                                already_received = true;
                                break;
                            }
                        } else {
                            // Found an empty slot in the buffer
                            if empty_index == PACKET_CHECK_BUFFER_EMPTY_VALUE {
                                empty_index = i as u8;
                            }
                        }
                    }

                    if already_received {
                        continue;
                    }

                    //if we can't find an empty slot, we delete the message with the oldest last packet
                    if empty_index == PACKET_CHECK_BUFFER_EMPTY_VALUE {
                        let mut oldest_index: u8 = 0;
                        let mut oldest_time = Instant::now() + embassy_time::Duration::from_secs(60);
                        let mut oldest_packet_header: [u8; 13] = [0u8; 13];

                        for i in 0..INCOMMING_PACKET_BUFFER_SIZE {
                            if let Some(packet) = &packet_buffer[i] {
                                if packet.arrival_time < oldest_time {
                                    oldest_time = packet.arrival_time;
                                    oldest_index = i as u8;
                                    oldest_packet_header.copy_from_slice(&packet.packet.data[0..13]);
                                }
                            }
                        }

                        // Remove all packets for the oldest message
                        for i in 0..INCOMMING_PACKET_BUFFER_SIZE {
                            if let Some(packet) = &packet_buffer[i] {
                                if packet.packet.data[0..13] == oldest_packet_header {
                                    packet_buffer[i] = None;
                                }
                            }
                        }

                        empty_index = oldest_index;
                    }

                    let mut packet_header: [u8; 13] = [0u8; 13];
                    packet_header.copy_from_slice(&rx_packet.data[0..13]);

                    // Store the packet in the buffer
                    packet_buffer[empty_index as usize] = Some(PacketBufferItem {
                        packet: rx_packet,
                        arrival_time: Instant::now(),
                    });

                    // Check if we have all packets for this message
                    packet_check_buffer.fill(PACKET_CHECK_BUFFER_EMPTY_VALUE);
                    for i in 0..INCOMMING_PACKET_BUFFER_SIZE {
                        if let Some(packet_item) = &packet_buffer[i] {
                            if packet_item.packet.data[0..13] == packet_header {
                                // Mark this packet as received
                                packet_check_buffer[packet_item.packet.packet_index() as usize] = i as u8;
                            }
                        }
                    }

                    let mut all_packets_received = true;

                    // Check if all packets for this message have been received
                    for i in 0..total_packet_count {
                        if packet_check_buffer[i as usize] == PACKET_CHECK_BUFFER_EMPTY_VALUE {
                            // Not all packets received, break out of the loop
                            all_packets_received = false;
                            break;
                        }
                    }

                    if all_packets_received {
                        // All packets received, create a RadioMessage
                        let mut radio_message = RadioMessage::new_empty_message();

                        // Check if all packets for this message have been received
                        for i in 0..total_packet_count {
                            let packet_index = packet_check_buffer[i as usize];
                            if let Some(packet_item) = &packet_buffer[packet_index as usize] {
                                // Add the packet to the RadioMessage
                                radio_message.add_packet(&packet_item.packet);
                                packet_buffer[packet_index as usize] = None; // Clear the packet from the buffer
                            } else {
                                log!(Level::Error, "Packet index {} not found in buffer", packet_index);
                            }
                        }
                        process_message(
                            radio_message,
                            received_packet.rssi,
                            received_packet.snr,
                            outgoing_message_queue_sender,
                            incomming_message_queue_sender,
                        );

                        // Clear the buffer for the next message
                        for item in &mut packet_buffer {
                            *item = None;
                        }
                        for check in &mut packet_check_buffer {
                            *check = PACKET_CHECK_BUFFER_EMPTY_VALUE;
                        }
                    }
                }
            }
            Either3::Second(process_result) => {}
            Either3::Third(_) => {
                // Timeout occurred, continue to next iteration
                continue;
            }
        }
    }
}

fn process_message(
    message: RadioMessage,
    last_rssi: u8,
    last_snr: u8,
    outgoing_message_queue_sender: OutgoingMessageQueueSender,
    incomming_message_queue_sender: IncommingMessageQueueSender,
) {
    if message.message_type() == MessageType::RequestEcho as u8 {
        let echo_response = RadioMessage::new_echo(message.sender_node_id(), last_rssi, last_snr);
        let result = outgoing_message_queue_sender.try_send(echo_response);
        if let Err(result_error) = result {
            let failed_message = match result_error {
                TrySendError::Full(msg) => msg,
            };
            log!(
                Level::Warn,
                "Failed to send message to outgoing_message_queue. The queue is full. Dropping message: messagetype: {}, sender_node_id: {}",
                failed_message.message_type(),
                failed_message.sender_node_id(),
            );
        };
    }

    if message.message_type() == MessageType::AddBlock as u8
        || message.message_type() == MessageType::RequestFullBlock as u8
        || message.message_type() == MessageType::RequestBlockPart as u8
        || message.message_type() == MessageType::AddTransaction as u8
        || message.message_type() == MessageType::AddBlock as u8
        || message.message_type() == MessageType::GetMempoolState as u8
        || message.message_type() == MessageType::Support as u8
    {
        let result = incomming_message_queue_sender.try_send(message);

        if let Err(result_error) = result {
            let failed_message = match result_error {
                TrySendError::Full(msg) => msg,
            };
            log!(
                Level::Warn,
                "Failed to send message to incomming_message_queue. The queue is full. Dropping message: messagetype: {}, sender_node_id: {}",
                failed_message.message_type(),
                failed_message.sender_node_id(),
            );
        };
    }
}

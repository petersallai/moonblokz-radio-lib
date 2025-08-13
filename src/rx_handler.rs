use crate::relay_manager::RelayResult;
use crate::{CONNECTION_MATRIX_SIZE, INCOMING_PACKET_BUFFER_SIZE, MessageType, RxState, ScoringMatrix, WAIT_POOL_SIZE};
use embassy_futures::select::{Either3, select3};
use embassy_sync::channel::TrySendError;
use embassy_time::{Instant, Timer};
use log::{Level, log};
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

use crate::{
    IncomingMessageQueueSender, OutgoingMessageQueueSender, ProcessResultQueueReceiver, RadioMessage, RadioPacket, RxPacketQueueReceiver, RxStateQueueSender,
    relay_manager::RelayManager,
};

struct PacketBufferItem {
    packet: RadioPacket,
    arrival_time: Instant,
}

const PACKET_CHECK_BUFFER_EMPTY_VALUE: u8 = 255;

#[cfg_attr(feature = "std", embassy_executor::task(pool_size = 10))]
#[cfg_attr(feature = "embedded", embassy_executor::task(pool_size = 1))]
pub(crate) async fn rx_handler_task(
    incoming_message_queue_sender: IncomingMessageQueueSender,
    outgoing_message_queue_sender: OutgoingMessageQueueSender,
    rx_packet_queue_receiver: RxPacketQueueReceiver,
    rx_state_queue_sender: RxStateQueueSender,
    process_result_queue_receiver: ProcessResultQueueReceiver,
    echo_request_minimal_interval: u32,
    echo_messages_target_interval: u8,
    echo_gathering_timeout: u8,
    relay_position_delay: u8,
    scoring_matrix: ScoringMatrix,
    own_node_id: u32,
    rng_seed: u64,
) -> ! {
    let mut packet_buffer: [Option<PacketBufferItem>; INCOMING_PACKET_BUFFER_SIZE] = [const { None }; INCOMING_PACKET_BUFFER_SIZE];
    let mut packet_check_buffer: [u8; INCOMING_PACKET_BUFFER_SIZE] = [PACKET_CHECK_BUFFER_EMPTY_VALUE; INCOMING_PACKET_BUFFER_SIZE];
    let mut rng = WyRand::seed_from_u64(rng_seed);
    let mut relay_manager = RelayManager::<CONNECTION_MATRIX_SIZE, WAIT_POOL_SIZE>::new(
        echo_request_minimal_interval,
        echo_messages_target_interval,
        echo_gathering_timeout,
        relay_position_delay,
        scoring_matrix,
        own_node_id,
        rng.next_u64(),
    );
    loop {
        match select3(
            rx_packet_queue_receiver.receive(),
            process_result_queue_receiver.receive(),
            Timer::at(relay_manager.calculate_next_timeout()),
        )
        .await
        {
            Either3::First(received_packet) => {
                let rx_packet = received_packet.packet;
                if rx_packet.total_packet_count() == 1 {
                    let radio_message = RadioMessage::new_from_single_packet(rx_packet);
                    if let Err(result_error) = incoming_message_queue_sender.try_send(radio_message) {
                        let failed_message = match result_error {
                            TrySendError::Full(msg) => msg,
                        };
                        log!(
                            Level::Warn,
                            "Failed to send single-packet message to incoming_message_queue. The queue is full. Dropping message: messagetype: {}, sender_node_id: {}",
                            failed_message.message_type(),
                            failed_message.sender_node_id(),
                        );
                    }
                } else if rx_packet.total_packet_count() == 0 {
                    log!(Level::Warn, "Received empty packet, skipping");
                    continue;
                } else {
                    // Handle multi-packet message
                    let packet_index = rx_packet.packet_index() as usize;
                    let total_packet_count = rx_packet.total_packet_count() as usize;

                    if total_packet_count > INCOMING_PACKET_BUFFER_SIZE {
                        log!(
                            Level::Error,
                            "Total packet count {} exceeds buffer capacity {}. Dropping message.",
                            total_packet_count,
                            INCOMING_PACKET_BUFFER_SIZE
                        );
                        continue;
                    }

                    if packet_index == total_packet_count - 1 {
                        //we can drop messages it there is an unhandled message in the queue
                        _ = rx_state_queue_sender.try_send(RxState::PacketedRxEnded);
                    } else {
                        _ = rx_state_queue_sender.try_send(RxState::PacketedRxInProgress(packet_index as u8, total_packet_count as u8));
                    }

                    if packet_index as usize >= INCOMING_PACKET_BUFFER_SIZE {
                        //we can drop messages it there is an unhandled message in the queue
                        log!(Level::Error, "Packet index out of bounds: {}", packet_index);
                        continue;
                    }

                    let mut already_received = false;
                    let mut empty_index = PACKET_CHECK_BUFFER_EMPTY_VALUE;

                    for i in 0..INCOMING_PACKET_BUFFER_SIZE {
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

                        for i in 0..INCOMING_PACKET_BUFFER_SIZE {
                            if let Some(packet) = &packet_buffer[i] {
                                if packet.arrival_time < oldest_time {
                                    oldest_time = packet.arrival_time;
                                    oldest_index = i as u8;
                                    oldest_packet_header.copy_from_slice(&packet.packet.data[0..13]);
                                }
                            }
                        }

                        // Remove all packets for the oldest message
                        for i in 0..INCOMING_PACKET_BUFFER_SIZE {
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
                    for i in 0..INCOMING_PACKET_BUFFER_SIZE {
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
                            received_packet.link_quality,
                            outgoing_message_queue_sender,
                            incoming_message_queue_sender,
                            &mut relay_manager,
                        );
                    }
                }
            }
            Either3::Second(process_result) => {
                relay_manager.process_processing_result(process_result);
            }
            Either3::Third(_) => {
                if let RelayResult::SendMessage(response_message) = relay_manager.process_timed_tasks() {
                    let result = outgoing_message_queue_sender.try_send(response_message);
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
            }
        }
    }
}

fn process_message(
    message: RadioMessage,
    last_link_quality: u8,
    outgoing_message_queue_sender: OutgoingMessageQueueSender,
    incoming_message_queue_sender: IncomingMessageQueueSender,
    relay_manager: &mut RelayManager<CONNECTION_MATRIX_SIZE, WAIT_POOL_SIZE>,
) {
    let mut should_process = true;
    let relay_result = relay_manager.process_received_message(&message, last_link_quality);

    match relay_result {
        RelayResult::None => {
            // No action needed
        }
        RelayResult::SendMessage(message) => {
            let result = outgoing_message_queue_sender.try_send(message);
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
        RelayResult::AlreadyHaveMessage => {
            should_process = false;
        }
    }

    if (message.message_type() == MessageType::AddBlock as u8
        || message.message_type() == MessageType::RequestFullBlock as u8
        || message.message_type() == MessageType::RequestBlockPart as u8
        || message.message_type() == MessageType::AddTransaction as u8
        || message.message_type() == MessageType::RequestNewMempoolItem as u8
        || message.message_type() == MessageType::Support as u8)
        && should_process
    {
        let result = incoming_message_queue_sender.try_send(message);

        if let Err(result_error) = result {
            let failed_message = match result_error {
                TrySendError::Full(msg) => msg,
            };
            log!(
                Level::Warn,
                "Failed to send message to incoming_message_queue. The queue is full. Dropping message: messagetype: {}, sender_node_id: {}",
                failed_message.message_type(),
                failed_message.sender_node_id(),
            );
        };
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use crate::MessageProcessingResult;
    use crate::relay_manager::RelayManager;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::Channel;

    fn scoring() -> ScoringMatrix {
        ScoringMatrix::new([[1, 1, 1, 1]; 4], 16, 48, 0)
    }

    fn new_rm() -> RelayManager<{ crate::CONNECTION_MATRIX_SIZE }, { crate::WAIT_POOL_SIZE }> {
        RelayManager::new(1, 1, 1, 1, scoring(), 1, 123)
    }

    #[test]
    fn request_echo_routes_only_to_outgoing() {
        type OutCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::OUTGOING_MESSAGE_QUEUE_SIZE }>;
        type InCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::INCOMING_MESSAGE_QUEUE_SIZE }>;

        let outgoing: &'static OutCh = Box::leak(Box::new(Channel::new()));
        let incoming: &'static InCh = Box::leak(Box::new(Channel::new()));

        let out_tx = outgoing.sender();
        let out_rx = outgoing.receiver();
        let in_tx = incoming.sender();
        let in_rx = incoming.receiver();

        let mut rm = new_rm();
        let msg = RadioMessage::new_request_echo(2);
        process_message(msg, 10, out_tx, in_tx, &mut rm);

        // Outgoing should have one Echo
        let sent = out_rx.try_receive().expect("expected an outgoing message");
        assert_eq!(sent.message_type(), MessageType::Echo as u8);

        // Incoming should be empty
        assert!(in_rx.try_receive().is_err());
    }

    #[test]
    fn add_block_routes_to_incoming_when_not_duplicate() {
        type OutCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::OUTGOING_MESSAGE_QUEUE_SIZE }>;
        type InCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::INCOMING_MESSAGE_QUEUE_SIZE }>;

        let outgoing: &'static OutCh = Box::leak(Box::new(Channel::new()));
        let incoming: &'static InCh = Box::leak(Box::new(Channel::new()));

        let out_tx = outgoing.sender();
        let out_rx = outgoing.receiver();
        let in_tx = incoming.sender();
        let in_rx = incoming.receiver();

        let mut rm = new_rm();
        let msg = RadioMessage::new_add_block(3, 100, 0xDEAD_BEEF, &[1, 2, 3]);
        process_message(msg, 5, out_tx, in_tx, &mut rm);

        // Incoming should have the AddBlock
        let in_msg = in_rx.try_receive().expect("expected an incoming message");
        assert_eq!(in_msg.message_type(), MessageType::AddBlock as u8);

        // Outgoing should be empty
        assert!(out_rx.try_receive().is_err());
    }

    #[test]
    fn duplicate_message_is_not_routed_to_incoming() {
        type OutCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::OUTGOING_MESSAGE_QUEUE_SIZE }>;
        type InCh = Channel<CriticalSectionRawMutex, RadioMessage, { crate::INCOMING_MESSAGE_QUEUE_SIZE }>;

        let outgoing: &'static OutCh = Box::leak(Box::new(Channel::new()));
        let incoming: &'static InCh = Box::leak(Box::new(Channel::new()));

        let out_tx = outgoing.sender();
        let out_rx = outgoing.receiver();
        let in_tx = incoming.sender();
        let in_rx = incoming.receiver();

        let mut rm = new_rm();

        // Pre-populate wait pool with a message
        let orig = RadioMessage::new_add_block(3, 5, 0x1111_2222, &[9]);
        rm.process_processing_result(MessageProcessingResult::NewBlockAdded(orig));

        // Process an equal message: should be treated as duplicate
        let dup = RadioMessage::new_add_block(4, 5, 0x1111_2222, &[9]); // different sender ignored in equality
        process_message(dup, 0, out_tx, in_tx, &mut rm);

        assert!(in_rx.try_receive().is_err());
        assert!(out_rx.try_receive().is_err());
    }
}

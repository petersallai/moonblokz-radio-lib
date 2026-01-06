// This example runs on the Raspberry Pi Pico with a Waveshare board containing a Semtech Sx1262 radio.
// It demonstrates LORA P2P send functionality.

#![no_std]
#![no_main]

use embassy_futures::select::{select3, Either3};
use lora_phy::sx126x::TcxoCtrlVoltage;
use rp_usb_console;

use embassy_executor::Spawner;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use heapless::Vec;
use lora_phy::mod_params::{Bandwidth, CodingRate, SpreadingFactor};
use moonblokz_radio_lib::radio_devices::rp_lora_sx1262::RadioDevice;
use moonblokz_radio_lib::{IncomingMessageItem, RadioConfiguration};
use moonblokz_radio_lib::{RadioCommunicationManager, RadioMessage};

const TEST_BLOCK_SIZE: usize = 2000; // Size of the test block to send
const SEND_MESSAGE_INTERVAL_SECS: u64 = 30; // Interval between sending messages

type CommandChannel = Channel<CriticalSectionRawMutex, [u8; rp_usb_console::USB_READ_BUFFER_SIZE], 4>;
static COMMAND_CHANNEL: CommandChannel = Channel::new();

use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Break into debugger if attached, then spin.
    loop {
        cortex_m::asm::bkpt();
    }
}

/// Parse a /M_{sequence}_ command and extract the sequence number
fn parse_measurement_command(command: &str) -> Option<u32> {
    let trimmed = command.trim();
    if trimmed.starts_with("/M_") && trimmed.ends_with("_") {
        let inner = &trimmed[3..trimmed.len() - 1];
        inner.parse::<u32>().ok()
    } else {
        None
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    //let _driver = Driver::new(p.USB, Irqs);
    //  spawner.spawn(logger_task(driver)).unwrap();
    rp_usb_console::start(spawner, log::LevelFilter::Trace, p.USB, Some(COMMAND_CHANNEL.sender()));

    let command_receiver = COMMAND_CHANNEL.receiver();

    //waiting a bit to let the terminal connect to the device
    let mut debug_indicator = Output::new(p.PIN_25, Level::Low);
    debug_indicator.set_high();
    Timer::after_secs(10).await;
    debug_indicator.set_low();

    log::info!("Starting up");
    // Initialize the flash peripheral
    let mut flash: Flash<'_, FLASH, Blocking, { 2 * 1024 * 1024 }> = Flash::new_blocking(p.FLASH);

    // Create a buffer to store the unique ID
    let mut uid = [0u8; 8];
    let mut own_node_id = match flash.blocking_unique_id(&mut uid) {
        Ok(_) => {
            let uidu32 = u32::from_le_bytes(uid[2..6].try_into().unwrap()) % 9999 + 1;
            uidu32
        }
        Err(e) => {
            log::error!("Error reading unique ID: {:?}", e);
            0
        }
    };

    if own_node_id < 1000 {
        own_node_id += 1000; // Ensure node ID is at least 1000 to be the same length
    }

    log::info!("Own node id: {}", own_node_id);

    let mut radio_device: RadioDevice = RadioDevice::new();
    //TODO: use some general configuration for the radio, not hardcoded node_id
    if own_node_id == 2754 {
        //RP2040+Lora module
        if radio_device
            .initialize(
                p.PIN_3.into(),
                p.PIN_15.into(),
                p.PIN_20.into(),
                p.PIN_2.into(),
                None,
                p.SPI1,
                p.PIN_10,
                p.PIN_11,
                p.PIN_12,
                p.DMA_CH0.into(),
                p.DMA_CH1.into(),
                Some(TcxoCtrlVoltage::Ctrl1V7),
                900_000_000, // 900 MHz
                SpreadingFactor::_7,
                Bandwidth::_250KHz,
                CodingRate::_4_5,
                own_node_id,
            )
            .await
            .is_err()
        {
            log::error!("Failed to initialize radio device");
        };
    } else {
        //RP2040-Lora integrated
        if radio_device
            .initialize(
                p.PIN_13.into(),
                p.PIN_23.into(),
                p.PIN_16.into(),
                p.PIN_18.into(),
                Some(p.PIN_17.into()),
                p.SPI1,
                p.PIN_14,
                p.PIN_15,
                p.PIN_24,
                p.DMA_CH0.into(),
                p.DMA_CH1.into(),
                None,
                900_000_000, // 900 MHz
                SpreadingFactor::_7,
                Bandwidth::_125KHz,
                CodingRate::_4_5,
                own_node_id,
            )
            .await
            .is_err()
        {
            log::error!("Failed to initialize radio device");
        };
    }

    log::info!("Radio device initialized");
    let mut radio_communication_manager = RadioCommunicationManager::new();
    let radio_configuration = RadioConfiguration {
        delay_between_tx_packets: 200,
        delay_between_tx_messages: 5,
        echo_request_minimal_interval: 1,
        echo_messages_target_interval: 100,
        echo_gathering_timeout: 1,
        relay_position_delay: 5,
        scoring_matrix: moonblokz_radio_lib::ScoringMatrix::new_from_encoded(&[255u8, 243u8, 65u8, 123u8, 47u8]),
        retry_interval_for_missing_packets: 60,
        tx_maximum_random_delay: 200,
    };

    if radio_communication_manager
        .initialize(radio_configuration, spawner, radio_device, own_node_id, own_node_id as u64)
        .is_err()
    {
        log::error!("Failed to initialize radio communication manager");
    }
    log::info!("Radio communication manager initialized");
    let mut sequence_number: u32 = own_node_id * 10000;
    let mut arrived_sequences: Vec<u32, 1000> = Vec::new();
    let mut next_send_time = embassy_time::Instant::now() + Duration::from_secs(5);

    loop {
        match select3(
            radio_communication_manager.receive_message(),
            Timer::at(next_send_time),
            command_receiver.receive(),
        )
        .await
        {
            Either3::First(message) => {
                if let Ok(IncomingMessageItem::NewMessage(msg)) = message {
                    if msg.message_type() == moonblokz_radio_lib::MessageType::AddBlock as u8 {
                        if let Some(sequence) = msg.sequence() {
                            if arrived_sequences.contains(&sequence) {
                                // TODO: Already have message, should we report back?
                            } else {
                                log::info!(
                                    "[{}] *TM6* Received AddBlock message: sender: {}, sequence: {} length: {}",
                                    own_node_id,
                                    msg.sender_node_id(),
                                    sequence,
                                    msg.length()
                                );
                                radio_communication_manager.report_message_processing_status(moonblokz_radio_lib::MessageProcessingResult::NewBlockAdded(msg));
                                if arrived_sequences.len() == arrived_sequences.capacity() {
                                    // Remove the oldest sequence to make space
                                    arrived_sequences.remove(0);
                                }
                                arrived_sequences.push(sequence).unwrap_or_else(|_| {
                                    log::error!("Arrived sequences buffer full, cannot track more sequences");
                                });
                            }
                        }
                    } else if msg.message_type() == moonblokz_radio_lib::MessageType::RequestBlockPart as u8 {
                        let Some(sequence) = msg.sequence() else {
                            continue;
                        };
                        if arrived_sequences.contains(&sequence) {
                            log::debug!(
                                "Received RequestBlockPart: sender: {}, sequence: {} length: {}",
                                msg.sender_node_id(),
                                sequence,
                                msg.length()
                            );
                            let payload: [u8; TEST_BLOCK_SIZE] = [22; TEST_BLOCK_SIZE];

                            let mut response_message = RadioMessage::add_block_with(own_node_id, sequence, &payload);

                            if let Some(request_blockpart_iterator) = msg.get_request_block_part_iterator() {
                                let mut block_parts: [bool; moonblokz_radio_lib::RADIO_MAX_PACKET_COUNT] = [false; moonblokz_radio_lib::RADIO_MAX_PACKET_COUNT];
                                for part in request_blockpart_iterator {
                                    block_parts[part.packet_index as usize] = true;
                                }
                                response_message.add_packet_list(block_parts);
                                let _ = radio_communication_manager.report_message_processing_status(
                                    moonblokz_radio_lib::MessageProcessingResult::RequestedBlockPartsFound(response_message, msg.sender_node_id()),
                                );
                            }
                        }
                    }
                } else if let Ok(IncomingMessageItem::CheckIfAlreadyHaveMessage(message_type, sequence, payload_checksum)) = message {
                    if arrived_sequences.contains(&sequence) {
                        let _ = radio_communication_manager.report_message_processing_status(moonblokz_radio_lib::MessageProcessingResult::AlreadyHaveMessage(
                            message_type,
                            sequence,
                            payload_checksum,
                        ));
                    }
                }
            }
            Either3::Second(_) => {
                if Instant::now() >= next_send_time {
                    let payload: [u8; TEST_BLOCK_SIZE] = [22; TEST_BLOCK_SIZE];

                    let message = RadioMessage::add_block_with(own_node_id, sequence_number, &payload);
                    log::info!(
                        "[{}] *TM7* Sending AddBlock: sender: {}, sequence: {}, length: {}",
                        own_node_id,
                        message.sender_node_id(),
                        sequence_number,
                        message.length()
                    );
                    if radio_communication_manager.send_message(message).is_err() {
                        log::error!("Failed to send message with sequence number {}", sequence_number);
                    }
                    if arrived_sequences.len() == arrived_sequences.capacity() {
                        // Remove the oldest sequence to make space
                        arrived_sequences.remove(0);
                    }
                    arrived_sequences.push(sequence_number).unwrap_or_else(|_| {
                        log::error!("Arrived sequences buffer full, cannot track more sequences.");
                    });
                    sequence_number += 1;

                    next_send_time += Duration::from_secs(SEND_MESSAGE_INTERVAL_SECS);
                }
            }
            Either3::Third(command_msg) => {
                if let Ok(command_str) = core::str::from_utf8(&command_msg) {
                    if let Some(seq) = parse_measurement_command(command_str) {
                        log::debug!("[{}] *TM3* Start measurement: sequence: {}", own_node_id, seq);

                        // Create payload with repeating bytes of the sequence number (use low byte)
                        let seq_byte = (seq & 0xFF) as u8;
                        let payload: [u8; TEST_BLOCK_SIZE] = [seq_byte; TEST_BLOCK_SIZE];

                        let message = RadioMessage::add_block_with(own_node_id, seq, &payload);
                        if radio_communication_manager.send_message(message).is_err() {
                            log::error!("Failed to send measurement message with sequence number {}", seq);
                        }

                        // Track the sequence we just sent
                        if arrived_sequences.len() == arrived_sequences.capacity() {
                            arrived_sequences.remove(0);
                        }
                        arrived_sequences.push(seq).unwrap_or_else(|_| {
                            log::error!("Arrived sequences buffer full, cannot track more sequences.");
                        });
                    } else {
                        log::info!("Command: arrived {}", command_str);
                    }
                } else {
                    log::error!("Received invalid UTF-8 command");
                }
            }
        }
    }
}

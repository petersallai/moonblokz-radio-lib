// This example runs on the Raspberry Pi Pico with a Waveshare board containing a Semtech Sx1262 radio.
// It demonstrates LORA P2P send functionality.

#![no_std]
#![no_main]

mod panic;
use rp_usb_console;

use embassy_executor::Spawner;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use lora_phy::mod_params::{Bandwidth, CodingRate, SpreadingFactor};
use moonblokz_radio_lib::radio_device_lora_sx1262::RadioDevice;
use moonblokz_radio_lib::RadioConfiguration;
use moonblokz_radio_lib::{RadioCommunicationManager, RadioMessage};
use rp2040_hal::rom_data::reset_to_usb_boot;
// Note: frequency is set when initializing RadioDevice.

type CommandChannel = Channel<CriticalSectionRawMutex, [u8; 64], 4>;
static COMMAND_CHANNEL: CommandChannel = Channel::new();

#[embassy_executor::task]
async fn command_task(command_receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, [u8; 64], 4>) {
    loop {
        let msg = command_receiver.receive().await;
        if let Ok(command_str) = core::str::from_utf8(&msg) {
            log::info!("Processing command: {}", command_str);
            if command_str.starts_with("/BOOTSEL") {
                log::info!("Rebooting to BOOTSEL mode...");
                reset_to_usb_boot(0, 0);
            }
        } else {
            log::error!("Received invalid UTF-8 command");
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    //let _driver = Driver::new(p.USB, Irqs);
    //  spawner.spawn(logger_task(driver)).unwrap();
    rp_usb_console::start(spawner, log::LevelFilter::Trace, p.USB, COMMAND_CHANNEL.sender());
    spawner.spawn(command_task(COMMAND_CHANNEL.receiver())).unwrap();

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
    let own_node_id = match flash.blocking_unique_id(&mut uid) {
        Ok(_) => {
            let uidu32 = u32::from_le_bytes(uid[2..6].try_into().unwrap()) % 9999 + 1;
            log::info!("Unique ID: {}", uidu32);
            uidu32
        }
        Err(e) => {
            log::error!("Error reading unique ID: {:?}", e);
            0
        }
    };

    let mut radio_device: RadioDevice = RadioDevice::new();
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
            900_000_000, // 900 MHz
            SpreadingFactor::_7,
            Bandwidth::_250KHz,
            CodingRate::_4_5,
        )
        .await
        .is_err()
    {
        log::error!("Failed to initialize radio device");
    };
    log::info!("Radio device initialized");
    let mut radio_communication_manager = RadioCommunicationManager::new();
    let radio_configuration = RadioConfiguration {
        delay_between_tx_packets: 200,
        delay_between_tx_messages: 5,
        echo_request_minimal_interval: 10,
        echo_messages_target_interval: 100,
        echo_gathering_timeout: 1,
        relay_position_delay: 5,
        scoring_matrix: moonblokz_radio_lib::ScoringMatrix::new_from_encoded(&[255u8, 243u8, 65u8, 123u8, 47u8]),
        retry_interval_for_missing_packets: 60,
    };

    if radio_communication_manager
        .initialize(radio_configuration, spawner, radio_device, 1, 1)
        .is_err()
    {
        log::error!("Failed to initialize radio communication manager");
    }
    log::info!("Radio communication manager initialized");
    let mut index = 1;
    loop {
        debug_indicator.set_high();
        Timer::after_secs(5).await;
        debug_indicator.set_low();
        Timer::after_secs(1).await;
        let payload: [u8; 20] = [22; 20];
        //let payload: [u8; 10] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        let message = RadioMessage::new_add_block(own_node_id, index, &payload);
        if radio_communication_manager.send_message(message).is_ok() {
            log::info!("Sent message with index {}", index);
        } else {
            log::error!("Failed to send message with index {}", index);
        }
        log::info!("Radio communication manager running tasks: [{}]", index);
        index += 1;
    }
}

// This example runs on the Raspberry Pi Pico with a Waveshare board containing a Semtech Sx1262 radio.
// It demonstrates LORA P2P send functionality.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
// GPIO/SPI types are not referenced directly here, but we need Pin/Channel traits for `degrade()`.
use embassy_rp::dma::Channel;
use embassy_rp::gpio::Pin;
use embassy_rp::usb::Driver;
use embassy_rp::{bind_interrupts, peripherals, usb};
use moonblokz_radio_lib::radio_device_lora_sx1262::RadioDevice;
use moonblokz_radio_lib::RadioCommunicationManager;
use moonblokz_radio_lib::RadioConfiguration;
use {defmt_rtt as _, panic_probe as _};
// Note: frequency is set when initializing RadioDevice.

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, peripherals::USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut radio_device: RadioDevice = RadioDevice::new();
    if radio_device
        .initialize(
            p.PIN_13.degrade(),
            p.PIN_23.degrade(),
            p.PIN_16.degrade(),
            p.PIN_18.degrade(),
            Some(p.PIN_17.degrade()),
            p.SPI1,
            p.PIN_14,
            p.PIN_15,
            p.PIN_24,
            p.DMA_CH0.degrade(),
            p.DMA_CH1.degrade(),
            900_000_000, // 900 MHz
        )
        .await
        .is_err()
    {
        log::error!("Failed to initialize radio device");
    };

    let mut radio_communication_manager = RadioCommunicationManager::new();
    let radio_configuration = RadioConfiguration {
        delay_between_tx_packets: 1,
        delay_between_tx_messages: 10,
        echo_request_minimal_interval: 86400,
        echo_messages_target_interval: 100,
        echo_gathering_timeout: 10,
        relay_position_delay: 1,
        scoring_matrix: moonblokz_radio_lib::ScoringMatrix::new([[1, 1, 1, 1]; 4], 16, 48, 0),
    };
    _ = radio_communication_manager.initialize(radio_configuration, spawner, radio_device, 1, 1);

    /*radio_device.send_message(message::Message {
        payload: b"Hello, LoRa!",
        ..Default::default()
    });*/
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    //let mut debug_indicator = Output::new(p.PIN_25, Level::Low);
}

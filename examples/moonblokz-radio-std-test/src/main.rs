use embassy_executor::Spawner;
use embassy_futures::select::select_array;
use embassy_time::{Duration, Instant, Timer};
use env_logger;
use env_logger::Builder;
use log::LevelFilter;
use log::{debug, error, info, log, trace, warn}; // Import the macros you want to use
use moonblokz_radio_lib::RadioCommunicationManager;
use moonblokz_radio_lib::RadioConfiguration;
use moonblokz_radio_lib::RadioMessage;
use moonblokz_radio_lib::radio_device_echo::RadioDevice;

#[embassy_executor::task(pool_size = 10)]
async fn run(node_id: u32, radio_communication_manager: &'static RadioCommunicationManager) -> ! {
    let mut i: u8 = 0;
    loop {
        log!(log::Level::Debug, "sending message from node {}: {}", node_id, i);
        let payload: [u8; 1111] = [i; 1111];
        let _ = radio_communication_manager.send_message(RadioMessage::new_add_transaction(node_id, i as u32, 1, &payload));
        Timer::after(Duration::from_secs(5)).await;
        i += 1;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    Builder::new()
        .filter_level(LevelFilter::Debug) // Sets the default level to INFO
        // You can also add more filters if needed, e.g., for specific modules:
        // .filter_module("my_app::networking", LevelFilter::Debug)
        .init();

    log!(log::Level::Debug, "Starting up");
    let mut radio_device_1: RadioDevice = RadioDevice::new();

    let mut radio_communication_manager_temp_1 = RadioCommunicationManager::new();
    let radio_configuration = RadioConfiguration {
        delay_between_tx_packets: 1,
        delay_between_tx_messages: 10,
    };

    radio_communication_manager_temp_1.initialize(radio_configuration, spawner, radio_device_1);
    log!(log::Level::Debug, "radio communication manager 1 started");
    let radio_communication_manager_1: &'static RadioCommunicationManager = Box::leak(Box::new(radio_communication_manager_temp_1));
    log!(log::Level::Debug, "spawning task to send messages to radio communication manager 1");

    spawner.spawn(run(1, &radio_communication_manager_1)).unwrap();

    let mut radio_device_2: RadioDevice = RadioDevice::new();

    let mut radio_communication_manager_temp_2 = RadioCommunicationManager::new();
    let radio_configuration = RadioConfiguration {
        delay_between_tx_packets: 1,
        delay_between_tx_messages: 10,
    };

    let res = radio_communication_manager_temp_2.initialize(radio_configuration, spawner, radio_device_2);
    if res.is_err() {
        log!(log::Level::Error, "Error initializing radio communication manager 2: {:?}", res.err());
    }
    log!(log::Level::Debug, "radio communication manager 2 started");
    let radio_communication_manager_2: &'static RadioCommunicationManager = Box::leak(Box::new(radio_communication_manager_temp_2));
    log!(log::Level::Debug, "spawning task to send messages to radio communication manager 2");

    spawner.spawn(run(2, &radio_communication_manager_2)).unwrap();

    let mut radio_communication_managers = vec![];
    radio_communication_managers.push(radio_communication_manager_1);
    radio_communication_managers.push(radio_communication_manager_2);

    log!(log::Level::Debug, "task spawned, now receiving messages");
    loop {
        let (message, _) = select_array([radio_communication_manager_1.receive_message(), radio_communication_manager_2.receive_message()]).await;
        if let Ok(msg) = message {
            let (anchor_sequence, checksum, data) = msg.get_add_transaction_data().unwrap();
            log!(
                log::Level::Debug,
                "Received message: from node: {}, type: {}, sequence: {}, length: {}",
                msg.sender_node_id(),
                msg.message_type(),
                anchor_sequence,
                msg.length()
            );
        } else {
            log!(log::Level::Error, "Error receiving message");
            continue;
        }
    }
}

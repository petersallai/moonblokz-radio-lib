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

const CAD_MINIMAL_WAIT_TIME: u64 = 300; // Minimum wait time in milliseconds after CAD command
const CAD_MAX_ADDITIONAL_WAIT_TIME: u64 = 200; // Maximum additional wait time in milliseconds after CAD command

const RADIO_OUTPUT_QUEUE_SIZE: usize = 10;
pub type RadioOutputQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioOutputMessage, RADIO_OUTPUT_QUEUE_SIZE>;
pub type RadioOutputQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioOutputMessage, RADIO_OUTPUT_QUEUE_SIZE>;
pub type RadioOutputQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioOutputMessage, RADIO_OUTPUT_QUEUE_SIZE>;

const RADIO_INPUT_QUEUE_SIZE: usize = 10;
pub type RadioInputQueue = embassy_sync::channel::Channel<CriticalSectionRawMutex, RadioInputMessage, RADIO_INPUT_QUEUE_SIZE>;
pub type RadioInputQueueReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RadioInputMessage, RADIO_INPUT_QUEUE_SIZE>;
pub type RadioInputQueueSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, RadioInputMessage, RADIO_INPUT_QUEUE_SIZE>;

pub enum RadioOutputMessage {
    SendPacket(RadioPacket),
    RequestCAD,
}

pub enum RadioInputMessage {
    ReceivePacket(ReceivedPacket),
    CADResponse(bool),
}

#[embassy_executor::task(pool_size = MAX_NODE_COUNT)]
pub(crate) async fn radio_device_task(radio_device: RadioDevice, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender, rng_seed: u64) {
    log!(Level::Info, "Simulated radio device task started");
    let mut rng = WyRand::seed_from_u64(rng_seed);
    loop {
        let mut next_cad = true;
        match select(radio_device.input_queue_receiver.receive(), tx_receiver.receive()).await {
            Either::First(message) => match message {
                RadioInputMessage::ReceivePacket(pkt) => {
                    rx_sender.send(pkt).await;
                }
                RadioInputMessage::CADResponse(_busy) => {
                    log!(Level::Warn, "Not waiting for CAD response. Dropping.");
                }
            },
            Either::Second(packet) => loop {
                if next_cad {
                    radio_device.output_queue_sender.send(RadioOutputMessage::RequestCAD).await;
                }

                match radio_device.input_queue_receiver.receive().await {
                    RadioInputMessage::ReceivePacket(pkt) => {
                        rx_sender.send(pkt).await;
                        next_cad = false;
                    }
                    RadioInputMessage::CADResponse(busy) => {
                        next_cad = true;
                        if busy {
                            Timer::after(embassy_time::Duration::from_millis(
                                CAD_MINIMAL_WAIT_TIME + rng.next_u64() % CAD_MAX_ADDITIONAL_WAIT_TIME,
                            ))
                            .await;
                            continue;
                        } else {
                            radio_device.output_queue_sender.send(RadioOutputMessage::SendPacket(packet)).await;
                            break;
                        }
                    }
                }
            },
        }
    }
}

pub struct RadioDevice {
    output_queue_sender: RadioOutputQueueSender,
    input_queue_receiver: RadioInputQueueReceiver,
}
impl RadioDevice {
    pub const fn new(output_queue_sender: RadioOutputQueueSender, input_queue_receiver: RadioInputQueueReceiver) -> Self {
        RadioDevice {
            output_queue_sender,
            input_queue_receiver,
        }
    }
}

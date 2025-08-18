use core::char::MAX;

use crate::MAX_NODE_COUNT;
use crate::RxPacketQueueSender;
use crate::TxPacketQueueReceiver;
use log::{Level, log};

#[embassy_executor::task(pool_size = MAX_NODE_COUNT)]
pub(crate) async fn radio_device_task(mut radio_device: RadioDevice, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender, _rng_seed: u64) -> ! {
    log!(Level::Info, "Echo radio device task started");
    radio_device.run(tx_receiver, rx_sender).await
}

pub struct RadioDevice {}
impl RadioDevice {
    pub const fn new() -> Self {
        RadioDevice {}
    }

    async fn run(&mut self, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender) -> ! {
        loop {
            let packet = tx_receiver.receive().await;
            match rx_sender.try_send(crate::ReceivedPacket { packet, link_quality: 63 }) {
                Ok(_) => {}
                Err(embassy_sync::channel::TrySendError::Full(received_packet)) => {
                    // Backpressure: drop echoed packet and log
                    log!(
                        Level::Warn,
                        "RX queue full, dropping echoed packet. type: {}",
                        received_packet.packet.message_type()
                    );
                }
            }
        }
    }
}

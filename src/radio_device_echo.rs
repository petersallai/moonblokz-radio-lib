use crate::RxPacketQueueSender;
use crate::TxPacketQueueReceiver;
use log::log;

#[cfg_attr(feature = "std", embassy_executor::task(pool_size = 10))]
#[cfg_attr(feature = "embedded", embassy_executor::task(pool_size = 1))]
pub(crate) async fn radio_device_task(mut radio_device: RadioDevice, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender, _rng_seed: u64) -> ! {
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
            let _ = rx_sender.try_send(crate::ReceivedPacket {
                packet: packet,
                rssi: 0,
                snr: 0,
            });
        }
    }
}

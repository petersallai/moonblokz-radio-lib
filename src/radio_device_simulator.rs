use core::char::MAX;

use crate::MAX_NODE_COUNT;
use crate::RxPacketQueueSender;
use crate::TxPacketQueueReceiver;
use log::{Level, log};

#[embassy_executor::task(pool_size = MAX_NODE_COUNT)]
pub(crate) async fn radio_device_task(mut radio_device: RadioDevice, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender, _rng_seed: u64) {
    log!(Level::Info, "Simulated radio device task started");
    radio_device.tx_receiver = Some(tx_receiver);
    radio_device.rx_sender = Some(rx_sender);
}

pub struct RadioDevice {
    tx_receiver: Option<TxPacketQueueReceiver>,
    rx_sender: Option<RxPacketQueueSender>,
}
impl RadioDevice {
    pub const fn new() -> Self {
        RadioDevice {
            tx_receiver: None,
            rx_sender: None,
        }
    }
}

//! USB CDC logging & command channel for RP2040 (embassy).
//!
//! This module provides:
//! - A zero-heap logger (`UsbLogger`) implementing `log::Log` that formats
//!   lines into fixed-size buffers (`LogMessage`) and ships them over a USB
//!   CDC ACM interface.
//! - A transmit task that fragments each log message into 64-byte USB packets
//!   to reduce the chance of partial writes or host latency stalling progress.
//! - A receive task that collects inbound USB data into `LogMessage` objects
//!   and forwards them over a user-supplied `Channel` (for simple command /
//!   control use-cases).
//! - A `start()` convenience function wiring descriptors, class, and tasks.
//!
//! Design notes:
//! - No dynamic allocation: `LogMessage` is a fixed 255‑byte buffer. Longer
//!   messages are truncated and marked with an ellipsis.
//! - Non-blocking log enqueue: if the channel is full the log line is dropped
//!   intentionally to keep real-time behavior predictable.
//! - Fragmentation: Each `LogMessage` is split into `PACKET_SIZE` (64) byte
//!   chunks. This allows large lines to transmit even if the host drains
//!   intermittently.
//! - Safety: USB descriptor/state statics use a thin `StaticCell` wrapper
//!   (UnsafeCell + unsafe Sync) relying on single-core RP2040 + executor
//!   serialization. Access stays within initialization before tasks run.
//!
//! Usage:
//! ```ignore
//! // In main:
//! usb_serial::start(spawner, log::LevelFilter::Info, p.USB, COMMAND_CHANNEL.sender());
//! log::info!("Hello over USB");
//! // To read commands, receive from your COMMAND_CHANNEL in another task.
//! ```
//!
//! Limitations / TODO:
//! - Provide backpressure metrics (dropped log counter).
//! - Optional CRLF vs LF selection.
//! - Feature-gated extended formatting / timestamps.
//! - Graceful truncation marker for oversized messages.

use core::cell::UnsafeCell;
use core::fmt::{Result as FmtResult, Write};
use embassy_executor::Spawner;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{bind_interrupts, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender as UsbSender, State};
use embassy_usb::{Builder, UsbDevice};
use log::{LevelFilter, Log, Metadata, Record};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// ### Logger Setup ###

/// Size of each USB packet fragment.
const PACKET_SIZE: usize = 64;

/// Fixed-size (255 byte) log / command message buffer with packet helpers.
///
/// Implements `fmt::Write` so `write!` can be used directly. Messages longer
/// than the internal buffer are truncated without error.
pub struct LogMessage {
    len: usize,
    buf: [u8; 255],
}

impl LogMessage {
    /// Create an empty message buffer.
    fn new() -> Self {
        Self { len: 0, buf: [0; 255] }
    }

    /// Append a string (UTF-8 bytes) truncating if capacity exceeded.
    fn push_str(&mut self, s: &str) {
        for &b in s.as_bytes() {
            if self.len >= self.buf.len() {
                self.buf[self.len - 1] = b'.'; // Indicate truncation with ellipsis
                self.buf[self.len - 2] = b'.';
                self.buf[self.len - 3] = b'.';
                break;
            }
            self.buf[self.len] = b;
            self.len += 1;
        }
    }

    /// Number of 64-byte USB packets required to send this message.
    fn packet_count(&self) -> usize {
        self.len / PACKET_SIZE + if self.len % PACKET_SIZE == 0 { 0 } else { 1 }
    }

    /// Slice for a specific packet index (0-based) containing that chunk.
    fn as_packet_bytes(&self, packet_index: usize) -> &[u8] {
        let start = packet_index * PACKET_SIZE;
        let end = core::cmp::min(start + PACKET_SIZE, self.len);
        &self.buf[start..end]
    }
}

impl Write for LogMessage {
    fn write_str(&mut self, s: &str) -> FmtResult {
        self.push_str(s);
        Ok(())
    }
}

// Channel for sending log messages from the application to the USB sender task.
type LogChannel = Channel<CriticalSectionRawMutex, LogMessage, 4>;
static LOG_CHANNEL: LogChannel = Channel::new();

/// Logger implementation forwarding formatted lines into the channel.
struct USBLogger;

impl Log for USBLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut message = LogMessage::new();
            let path = if let Some(p) = record.module_path() { p } else { "" };
            if write!(&mut message, "[{}] {}: {}\r\n", record.level(), path, record.args()).is_ok() {
                // Non-blocking send. If the channel is full, the message is dropped.
                let _ = LOG_CHANNEL.try_send(message);
            }
        }
    }
    fn flush(&self) {}
}

static LOGGER: USBLogger = USBLogger;

// ### USB Tasks ###

/// USB RX task: waits for connection, then converts received UTF-8 packets
/// into `LogMessage` objects pushed onto the provided command channel.
#[embassy_executor::task]
async fn usb_rx_task(mut receiver: Receiver<'static, Driver<'static, USB>>, command_sender: Sender<'static, CriticalSectionRawMutex, [u8; 64], 4>) {
    loop {
        receiver.wait_connection().await;
        let mut buf = [0u8; 64];
        loop {
            match receiver.read_packet(&mut buf).await {
                Ok(_) => {
                    command_sender.send(buf).await;
                }
                Err(_) => break,
            }
        }
    }
}

/// USB TX task: drains log channel and writes each message fragment (64 bytes).
#[embassy_executor::task]
async fn usb_tx_task(mut sender: UsbSender<'static, Driver<'static, USB>>) {
    loop {
        sender.wait_connection().await;
        loop {
            // Wait for a log message from the channel.
            let message = LOG_CHANNEL.receive().await;
            let mut problem = false;
            for i in 0..message.packet_count() {
                let packet = message.as_packet_bytes(i);

                // Send the log message over USB. If sending fails, break to wait for reconnection.
                if sender.write_packet(packet).await.is_err() {
                    problem = true;
                }
            }
            if problem {
                break;
            }
        }
    }
}

/// USB device task: runs the USB device state machine.
#[embassy_executor::task]
async fn usb_device_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

// ### Public API ###

/// Initialize USB CDC logging & command channel and spawn supporting tasks.
///
/// Must be invoked once before using `log` macros. Performs descriptor / state
/// initialization and starts the RX, TX, and device tasks. The `command_sender`
/// receives any raw UTF‑8 text sent by the host over CDC.
pub fn start(spawner: Spawner, level: LevelFilter, usb_peripheral: Peri<'static, USB>, command_sender: Sender<'static, CriticalSectionRawMutex, [u8; 64], 4>) {
    // Initialize the logger (use racy variants on targets without atomic ptr support, e.g. thumbv6m/RP2040)
    unsafe {
        log::set_logger_racy(&LOGGER).unwrap();
        log::set_max_level_racy(level);
    }

    // Simple wrapper to mark our single-threaded statics as Sync. RP2040 + embassy executor: we ensure single-core access.
    struct StaticCell<T>(UnsafeCell<T>);
    unsafe impl<T> Sync for StaticCell<T> {}

    static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell(UnsafeCell::new([0; 256]));
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell(UnsafeCell::new([0; 256]));
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell(UnsafeCell::new([0; 256]));
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell(UnsafeCell::new([0; 128]));
    static STATE: StaticCell<State> = StaticCell(UnsafeCell::new(State::new()));

    let driver = Driver::new(usb_peripheral, Irqs);

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("Modular USB-Serial");
    config.serial_number = Some("12345678");
    config.max_power = 100;

    let mut builder = Builder::new(
        driver,
        config,
        unsafe { &mut *DEVICE_DESC.0.get() },
        unsafe { &mut *CONFIG_DESC.0.get() },
        unsafe { &mut *BOS_DESC.0.get() },
        unsafe { &mut *CONTROL_BUF.0.get() },
    );

    let class = CdcAcmClass::new(&mut builder, unsafe { &mut *STATE.0.get() }, 64);
    let (sender, receiver) = class.split();
    let usb = builder.build();

    // Spawn all the necessary tasks.
    spawner.spawn(usb_device_task(usb)).unwrap();
    spawner.spawn(usb_tx_task(sender)).unwrap();
    spawner.spawn(usb_rx_task(receiver, command_sender)).unwrap();
}

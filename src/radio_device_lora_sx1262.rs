//! # LoRa SX1262 Radio Device - Hardware Abstraction for SX1262 Transceiver
//!
//! This module provides a complete hardware abstraction layer for the Semtech SX1262
//! LoRa transceiver chip. It implements async radio operations with Embassy framework
//! integration and provides a type-safe interface for radio communication.
//!
//! ## Architecture
//!
//! The radio device implementation uses a state machine approach:
//! - **Uninitialized**: Initial state before hardware setup
//! - **Idle**: Radio is ready but not actively transmitting or receiving
//! - **Transmitting**: Currently sending a packet
//! - **Receiving**: Currently listening for packets
//! - **CAD (Channel Activity Detection)**: Scanning for channel activity before TX
//!
//! ## Key Components
//!
//! - **RadioDevice<S>**: Main state machine generic over state type
//!   - State transitions enforce valid operation sequences
//!   - Compile-time guarantees prevent invalid operations (e.g., TX while RX)
//!
//! - **SPI Communication**: Low-level interface to SX1262 chip
//!   - Command abstraction with register access
//!   - Busy pin monitoring for chip state
//!   - DMA support for efficient data transfer
//!
//! - **Packet Parameters**: Separate TX and RX configuration
//!   - Configurable preamble length, header type, CRC
//!   - Power amplifier configuration (PA_CONFIG)
//!   - Modulation parameters (spreading factor, bandwidth, coding rate)
//!
//! - **Queue Integration**: Async channel-based communication
//!   - TX packet queue: receives packets from scheduler
//!   - RX packet queue: forwards received packets to handler
//!   - Non-blocking operation with Embassy async primitives
//!
//! ## Channel Activity Detection (CAD)
//!
//! Before transmitting, CAD scans the channel for existing activity:
//! - Reduces packet collisions in dense networks
//! - Configurable detection parameters
//! - Falls back to immediate transmission if channel is clear
//!
//! ## Design Considerations
//!
//! - Generic state machine provides compile-time safety
//! - SPI operations use DMA for performance on embedded systems
//! - Packet size is fixed at compile time (RADIO_PACKET_SIZE)
//! - Link quality calculation uses RSSI values from radio
//! - Embassy async integration enables non-blocking operation
//! - Type-safe pin configuration with generic constraints
//!
//! ## Hardware Requirements
//!
//! - SX1262 LoRa transceiver chip
//! - SPI interface (MOSI, MISO, CLK, NSS pins)
//! - Control pins: RESET, DIO1, BUSY
//! - Optional: ANT_SW (antenna switch control)
//! - DMA channels for TX and RX operations

use crate::MessageType;
/// LoRa SX1262 Radio Device Implementation
///
/// This module provides a complete radio device implementation for the SX1262 LoRa transceiver,
/// with proper SPI initialization and state management.
///
/// # Features
/// - Async operation with Embassy framework
/// - State-based design preventing invalid operations
/// - Channel Activity Detection (CAD) support
/// - Configurable modulation parameters
/// - Separate TX/RX packet parameter management
/// - Queue-based message handling with separate TX/RX channels
/// - Type-safe SPI pin configuration with generic constraints
///
/// # Example
/// ```rust,no_run
/// use moonblokz_radio_lib::radio_device_lora_sx1262::{RadioDevice, RadioConfig};
/// use embassy_rp::peripherals::{PIN_10, PIN_11, PIN_12};
///
/// let mut radio = RadioDevice::new();
///
/// // Initialize with individual parameters
/// radio.initialize(
///     nss_pin, reset_pin, dio1_pin, busy_pin, None,
///     spi, clk_pin, mosi_pin, miso_pin,
///     tx_dma, rx_dma, 915_000_000
/// ).await?;
///
/// // Or use RadioConfig struct for grouped parameters
/// let config = RadioConfig {
///     spi_nss_pin: nss_pin,
///     reset_pin: reset_pin,
///     // ... other fields
/// };
/// ```
use crate::RADIO_PACKET_SIZE;
use crate::RadioPacket;
use crate::ReceivedPacket;
use crate::RxPacketQueueSender;
use crate::TxPacketQueueReceiver;
use crate::calculate_link_quality;

use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::gpio::AnyPin;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Config, Spi};
use embassy_time::Delay;
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::LoRa;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::TcxoCtrlVoltage;
use lora_phy::sx126x::{Sx126x, Sx1262};
use lora_phy::{mod_params::*, sx126x};
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

/// Radio hardware dependent constants:
///
/// Minimum wait time in milliseconds after CAD detects a busy channel
///
/// When Channel Activity Detection indicates the channel is busy, the radio
/// waits at least this duration before attempting CAD again.
const CAD_MINIMAL_WAIT_TIME: u64 = 300;

/// Maximum additional random wait time in milliseconds after CAD
///
/// Added to CAD_MINIMAL_WAIT_TIME to create randomized backoff, helping to
/// avoid synchronized collisions when multiple nodes detect the same busy channel.
const CAD_MAX_ADDITIONAL_WAIT_TIME: u64 = 200;

/// CAD operation timeout in milliseconds
///
/// Maximum time to wait for a CAD operation to complete before considering it failed.
const CAD_TIMEOUT_MS: u64 = 1000;

/// Radio device initialization errors
///
/// Represents specific errors that can occur during radio device initialization.
/// These provide more granular error information than the generic `InitializationFailed`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioDeviceInitError {
    /// Failed to create the SX126x interface variant
    InterfaceError,
    /// Failed to initialize the LoRa PHY layer
    LoraError,
    /// Failed to create modulation parameters
    ModulationParamsError,
    /// Failed to create TX packet parameters
    TXPacketParamsError,
    /// Failed to create RX packet parameters
    RXPacketParamsError,
}

#[cfg(feature = "std")]
impl std::fmt::Display for RadioDeviceInitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RadioDeviceInitError::InterfaceError => write!(f, "failed to create SX126x interface variant"),
            RadioDeviceInitError::LoraError => write!(f, "failed to initialize LoRa PHY layer"),
            RadioDeviceInitError::ModulationParamsError => write!(f, "failed to create modulation parameters"),
            RadioDeviceInitError::TXPacketParamsError => write!(f, "failed to create TX packet parameters"),
            RadioDeviceInitError::RXPacketParamsError => write!(f, "failed to create RX packet parameters"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for RadioDeviceInitError {}

/// Internal radio device errors
///
/// These errors represent failures during radio operations. Unlike
/// `RadioDeviceInitError`, these occur during runtime operation rather
/// than initialization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RadioDeviceError {
    /// Device was not initialized before attempting an operation
    InitializationFailed,
    /// Failed to transmit a packet
    TransmissionFailed,
    /// Failed to receive a packet
    ReceiveFailed,
    /// Channel Activity Detection operation failed
    CADFailed,
    /// Received packet failed CRC verification (soft-packet-crc feature)
    CRCMismatch,
}

#[cfg(feature = "std")]
impl std::fmt::Display for RadioDeviceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RadioDeviceError::InitializationFailed => write!(f, "device not initialized"),
            RadioDeviceError::TransmissionFailed => write!(f, "packet transmission failed"),
            RadioDeviceError::ReceiveFailed => write!(f, "packet reception failed"),
            RadioDeviceError::CADFailed => write!(f, "channel activity detection failed"),
            RadioDeviceError::CRCMismatch => write!(f, "packet CRC verification failed"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for RadioDeviceError {}

/// Calculate CRC-16-CCITT for packet integrity checking
///
/// Implements the CRC-16-CCITT algorithm with polynomial 0x1021 and
/// initial value 0xFFFF. Used when the `soft-packet-crc` feature is enabled
/// to provide packet integrity verification in software.
///
/// # Parameters
/// * `data` - Byte slice to calculate CRC for
///
/// # Returns
/// 16-bit CRC value
///
/// # Algorithm
/// - Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
/// - Initial value: 0xFFFF
/// - No final XOR
///
/// # Examples
/// ```rust,ignore
/// let data = b"123456789";
/// let crc = checksum16(data);
/// assert_eq!(crc, 0x29B1);
/// ```
fn checksum16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// State of the radio device
///
/// This enum ensures type safety by preventing operations on uninitialized hardware.
/// The `Inited` variant contains all the hardware resources and LoRa parameters
/// needed for radio operations.
enum RadioDeviceState {
    /// Device is not initialized - no hardware resources allocated
    NotInited,
    /// Device is initialized and ready for radio operations
    Inited {
        /// Optional transmit enable pin for PA control
        transmit_enable: Option<Output<'static>>,
        /// LoRa PHY instance with SX1262 configuration
        lora: LoRa<
            Sx126x<
                ExclusiveDevice<Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>, Output<'static>, Delay>,
                GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>,
                Sx1262,
            >,
            Delay,
        >,
        /// Modulation parameters (spreading factor, bandwidth, coding rate)
        mdltn_params: ModulationParams,
        /// Transmit packet parameters
        tx_pkt_params: PacketParams,
        /// Receive packet parameters
        rx_pkt_params: PacketParams,
    },
}

/// LoRa SX1262 radio device task - manages TX/RX operations
///
/// This async task runs the main radio loop, alternating between receiving
/// packets and transmitting queued packets. It handles Channel Activity Detection
/// (CAD) before transmission to avoid collisions.
///
/// # Task Pool
/// - Embedded builds: pool_size = 1 (single radio instance)
/// - Std builds: pool_size = 100 (testing/simulation with many nodes)
///
/// # Arguments
/// * `radio_device` - Initialized RadioDevice instance
/// * `tx_receiver` - Channel receiver for outgoing packets
/// * `rx_sender` - Channel sender for incoming packets
/// * `rng_seed` - Seed for random number generator (used for CAD backoff)
///
/// # Behavior
/// - Continuously races between RX and TX operations
/// - On RX: Receives packet and forwards to RX queue (drops if full)
/// - On TX: Performs CAD, waits if channel busy, transmits when clear
/// - Logs packet details at trace level for debugging
/// - Never returns (runs until executor stops)
///
/// # CAD Backoff
/// When CAD detects a busy channel, waits CAD_MINIMAL_WAIT_TIME plus a
/// random duration up to CAD_MAX_ADDITIONAL_WAIT_TIME before retrying.
#[cfg_attr(feature = "std", embassy_executor::task(pool_size = 100))]
#[cfg_attr(feature = "embedded", embassy_executor::task(pool_size = 1))]
pub(crate) async fn radio_device_task(mut radio_device: RadioDevice, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender, rng_seed: u64) -> ! {
    radio_device.run(tx_receiver, rx_sender, rng_seed).await
}

/// LoRa SX1262 Radio Device
///
/// Manages the complete lifecycle of a LoRa radio device including initialization,
/// message transmission, reception, and channel activity detection.
///
/// The device uses a state-based design to ensure hardware resources are properly
/// initialized before use and prevents invalid operations.
#[cfg_attr(feature = "std", derive(Debug))]
pub struct RadioDevice {
    /// Current state of the device (initialized or not)
    state: RadioDeviceState,
    /// Buffer for receiving radio packets
    receive_buffer: [u8; RADIO_PACKET_SIZE + 2], // Use consistent constant
}
impl RadioDevice {
    /// Create a new uninitialized radio device
    ///
    /// The device starts in the `NotInited` state and must be initialized
    /// with `initialize()` before use.
    ///
    /// # Examples
    /// ```rust
    /// use moonblokz_radio_lib::radio_device_lora_sx1262::RadioDevice;
    ///
    /// let radio = RadioDevice::new();
    /// assert!(!radio.is_initialized());
    /// ```
    pub const fn new() -> Self {
        RadioDevice {
            state: RadioDeviceState::NotInited,
            receive_buffer: [0u8; RADIO_PACKET_SIZE + 2],
        }
    }

    /// Initialize the radio device with hardware resources
    ///
    /// This method configures the SPI interface, GPIO pins, and LoRa PHY parameters.
    /// Must be called before any radio operations can be performed.
    ///
    /// # Parameters
    /// * `spi_nss_pin` - SPI chip select pin (active low)
    /// * `reset_pin` - Radio reset pin (active low)
    /// * `dio1_pin` - Digital I/O pin 1 for interrupts
    /// * `busy_pin` - Radio busy status pin
    /// * `transmit_pin_option` - Optional transmit enable pin for PA control
    /// * `spi` - SPI peripheral instance (SPI1)
    /// * `clk_pin` - SPI clock pin (must implement ClkPin<SPI1>)
    /// * `mosi_pin` - SPI master out, slave in pin (must implement MosiPin<SPI1>)
    /// * `miso_pin` - SPI master in, slave out pin (must implement MisoPin<SPI1>)
    /// * `tx_dma` - DMA channel for SPI transmit
    /// * `rx_dma` - DMA channel for SPI receive
    /// * `lora_frequency_in_hz` - LoRa carrier frequency in Hz (e.g., 915_000_000)
    ///
    /// # Returns
    /// * `Ok(())` if initialization succeeds
    /// * `Err(RadioDeviceInitError)` with specific error type if any step fails
    ///
    /// # Examples
    /// ```rust,no_run
    /// # async fn example() -> Result<(), moonblokz_radio_lib::radio_device_lora_sx1262::RadioDeviceInitError> {
    /// use moonblokz_radio_lib::radio_device_lora_sx1262::RadioDevice;
    /// use embassy_rp::peripherals::{PIN_10, PIN_11, PIN_12};
    ///
    /// let mut radio = RadioDevice::new();
    /// radio.initialize(
    ///     nss_pin, reset_pin, dio1_pin, busy_pin, None,
    ///     spi1, PIN_10, PIN_11, PIN_12,
    ///     tx_dma, rx_dma, 915_000_000
    /// ).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn initialize(
        &mut self,
        spi_nss_pin: Peri<'static, AnyPin>,
        reset_pin: Peri<'static, AnyPin>,
        dio1_pin: Peri<'static, AnyPin>,
        busy_pin: Peri<'static, AnyPin>,
        transmit_pin_option: Option<Peri<'static, AnyPin>>,
        spi: Peri<'static, embassy_rp::peripherals::SPI1>,
        clk_pin: Peri<'static, impl embassy_rp::spi::ClkPin<embassy_rp::peripherals::SPI1>>,
        mosi_pin: Peri<'static, impl embassy_rp::spi::MosiPin<embassy_rp::peripherals::SPI1>>,
        miso_pin: Peri<'static, impl embassy_rp::spi::MisoPin<embassy_rp::peripherals::SPI1>>,
        tx_dma: Peri<'static, embassy_rp::dma::AnyChannel>,
        rx_dma: Peri<'static, embassy_rp::dma::AnyChannel>,
        tcxo_ctrl: Option<TcxoCtrlVoltage>,
        lora_frequency_in_hz: u32,
        spreading_factor: SpreadingFactor,
        bandwidth: Bandwidth,
        coding_rate: CodingRate,
    ) -> Result<(), RadioDeviceInitError> {
        let spi_nss = Output::new(spi_nss_pin, Level::High);
        let reset = Output::new(reset_pin, Level::High);
        let dio1 = Input::new(dio1_pin, Pull::None);
        let busy = Input::new(busy_pin, Pull::None);
        let transmit_enable = if let Some(transmit_pin) = transmit_pin_option {
            Some(Output::new(transmit_pin, Level::High))
        } else {
            None
        };

        let spi = Spi::new(spi, clk_pin, mosi_pin, miso_pin, tx_dma, rx_dma, Config::default());
        let spi_device = match ExclusiveDevice::new(spi, spi_nss, Delay) {
            Ok(device) => device,
            Err(_err) => {
                return Err(RadioDeviceInitError::InterfaceError);
            }
        };

        let config = sx126x::Config {
            chip: Sx1262,
            tcxo_ctrl: tcxo_ctrl,
            use_dcdc: true,
            rx_boost: false,
        };
        let iv = match GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None) {
            Ok(interface) => interface,
            Err(_err) => {
                return Err(RadioDeviceInitError::InterfaceError);
            }
        };

        let mut lora = match LoRa::new(Sx126x::new(spi_device, iv, config), false, Delay).await {
            Ok(lora_instance) => lora_instance,
            Err(_err) => {
                return Err(RadioDeviceInitError::LoraError);
            }
        };

        let mdltn_params = {
            match lora.create_modulation_params(spreading_factor, bandwidth, coding_rate, lora_frequency_in_hz) {
                Ok(mp) => mp,
                Err(_err) => {
                    return Err(RadioDeviceInitError::ModulationParamsError);
                }
            }
        };
        let tx_pkt_params = {
            match lora.create_tx_packet_params(8, false, true, false, &mdltn_params) {
                Ok(pp) => pp,
                Err(_err) => {
                    return Err(RadioDeviceInitError::TXPacketParamsError);
                }
            }
        };

        let rx_pkt_params = {
            #[cfg(feature = "soft-packet-crc")]
            let rx_buffer_size = (RADIO_PACKET_SIZE + 2) as u8;
            #[cfg(not(feature = "soft-packet-crc"))]
            let rx_buffer_size = RADIO_PACKET_SIZE as u8;

            match lora.create_rx_packet_params(8, false, rx_buffer_size, true, false, &mdltn_params) {
                Ok(pp) => pp,
                Err(_err) => {
                    return Err(RadioDeviceInitError::RXPacketParamsError);
                }
            }
        };

        self.state = RadioDeviceState::Inited {
            transmit_enable,
            lora,
            mdltn_params,
            tx_pkt_params,
            rx_pkt_params,
        };

        Ok(())
    }

    /// Main radio loop - handles TX/RX operations with CAD
    ///
    /// This is the core operation loop that continuously races between receiving
    /// packets and transmitting queued packets. It implements intelligent channel
    /// access using CAD to avoid collisions.
    ///
    /// # Parameters
    /// * `tx_receiver` - Channel receiver for outgoing packets from TX scheduler
    /// * `rx_sender` - Channel sender for incoming packets to RX handler
    /// * `rng_seed` - Seed for random number generator (used for backoff timing)
    ///
    /// # Returns
    /// Never returns (infinite loop)
    ///
    /// # Operation Flow
    /// 1. Race between `receive_packet()` and `tx_receiver.receive()`
    /// 2. If RX wins: Process received packet, forward to RX queue
    /// 3. If TX wins: Perform CAD loop until channel clear, then transmit
    ///
    /// # CAD Loop
    /// - Check channel activity with timeout
    /// - If clear: transmit packet and exit loop
    /// - If busy/error/timeout: wait random duration and retry
    /// - Random backoff prevents synchronized collision
    ///
    /// # Error Handling
    /// - RX CRC mismatch: Log and continue (packet dropped)
    /// - RX queue full: Drop packet and log warning
    /// - TX failure: Log error and continue
    /// - CAD failure: Wait and retry
    async fn run(&mut self, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender, rng_seed: u64) -> ! {
        let mut rng = WyRand::seed_from_u64(rng_seed);
        loop {
            // Race between receiving a packet and getting a TX request
            match select(self.receive_packet(), tx_receiver.receive()).await {
                Either::First(rx_result) => match rx_result {
                    Ok(rx_packet) => {
                        if rx_packet.packet.message_type() == MessageType::AddBlock as u8
                            || rx_packet.packet.message_type() == MessageType::AddTransaction as u8
                        {
                            log::trace!(
                                "Received RX packet: type: {}, sender: {}, seq: {}, length: {}, packet: {}/{}",
                                rx_packet.packet.message_type(),
                                rx_packet.packet.sender_node_id(),
                                rx_packet.packet.sequence().unwrap_or(0),
                                rx_packet.packet.length,
                                rx_packet.packet.packet_index() + 1,
                                rx_packet.packet.total_packet_count(),
                            );
                        } else {
                            log::trace!(
                                "Received RX packet: type: {}, sender: {}, length: {}, link_quality: {}",
                                rx_packet.packet.message_type(),
                                rx_packet.packet.sender_node_id(),
                                rx_packet.packet.length,
                                rx_packet.link_quality
                            );
                        }

                        if rx_sender.try_send(rx_packet).is_err() {
                            log::warn!("RX queue full, dropping received packet.");
                        }
                    }
                    Err(RadioDeviceError::CRCMismatch) => {
                        log::debug!("Dropping packet due to CRC mismatch");
                    }
                    Err(_e) => {
                        log::warn!("Receive error from radio device");
                    }
                },
                Either::Second(tx_packet) => {
                    if tx_packet.message_type() == MessageType::AddBlock as u8 || tx_packet.message_type() == MessageType::AddTransaction as u8 {
                        log::trace!(
                            "Transmitting TX packet: type: {}, sender: {}, seq: {}, length: {}, packet: {}/{}",
                            tx_packet.message_type(),
                            tx_packet.sender_node_id(),
                            tx_packet.sequence().unwrap_or(0),
                            tx_packet.length,
                            tx_packet.packet_index() + 1,
                            tx_packet.total_packet_count(),
                        );
                    } else {
                        log::trace!(
                            "Transmitting TX packet: type: {}, sender: {}, length: {}",
                            tx_packet.message_type(),
                            tx_packet.sender_node_id(),
                            tx_packet.length
                        );
                    }
                    // Transmit the packet once: wait for a clear channel, then send and break
                    loop {
                        log::trace!("Starting CAD before transmit");
                        match select(self.do_cad(), Timer::after(embassy_time::Duration::from_millis(CAD_TIMEOUT_MS))).await {
                            Either::First(Ok(false)) => {
                                // Channel is clear; attempt a single send then exit loop
                                log::trace!("Channel clear, transmitting packet");
                                if self.send_packet(&tx_packet).await.is_err() {
                                    log::error!("Failed to transmit packet");
                                } else {
                                    log::trace!("Packet transmitted successfully");
                                }
                                break;
                            }
                            Either::First(Ok(true)) => {
                                log::trace!("Channel busy, waiting before retry");
                                let wait_ms = CAD_MINIMAL_WAIT_TIME + rng.next_u64() % CAD_MAX_ADDITIONAL_WAIT_TIME;
                                Timer::after(embassy_time::Duration::from_millis(wait_ms)).await;
                            }
                            Either::First(Err(_)) => {
                                log::trace!("CAD error, waiting before retry");
                                let wait_ms = CAD_MINIMAL_WAIT_TIME + rng.next_u64() % CAD_MAX_ADDITIONAL_WAIT_TIME;
                                Timer::after(embassy_time::Duration::from_millis(wait_ms)).await;
                            }
                            Either::Second(_) => {
                                log::trace!("CAD timeout, waiting before retry");
                                let wait_ms = CAD_MINIMAL_WAIT_TIME + rng.next_u64() % CAD_MAX_ADDITIONAL_WAIT_TIME;
                                Timer::after(embassy_time::Duration::from_millis(wait_ms)).await;
                            }
                        }
                    }
                }
            }
        }
    }

    /// Send a message using the radio device
    ///
    /// Transmits a radio packet using the configured modulation and packet parameters.
    /// The transmit enable pin (if configured) is automatically managed during transmission.
    ///
    /// # Parameters
    /// * `message` - The radio packet to transmit
    ///
    /// # Returns
    /// * `Ok(())` if transmission succeeds
    /// * `Err(RadioDeviceError::InitializationFailed)` if device is not initialized
    /// * `Err(RadioDeviceError::TransmissionFailed)` if transmission fails
    ///
    /// # Examples
    /// ```rust,no_run
    /// # async fn example(radio: &mut moonblokz_radio_lib::radio_device_lora_sx1262::RadioDevice, packet: &moonblokz_radio_lib::RadioPacket) -> Result<(), moonblokz_radio_lib::RadioDeviceError> {
    /// radio.send_message(packet).await?;
    /// # Ok(())
    /// # }
    /// ```
    async fn send_packet(&mut self, packet: &RadioPacket) -> Result<(), RadioDeviceError> {
        match &mut self.state {
            RadioDeviceState::NotInited => {
                // Return error if not initialized
                Err(RadioDeviceError::InitializationFailed)
            }
            RadioDeviceState::Inited {
                transmit_enable,
                lora,
                mdltn_params,
                tx_pkt_params,
                rx_pkt_params: _,
                ..
            } => {
                if packet.length == 0 || packet.length > RADIO_PACKET_SIZE {
                    return Err(RadioDeviceError::TransmissionFailed);
                }

                transmit_enable.as_mut().map(|te| te.set_low());
                #[cfg(feature = "soft-packet-crc")]
                {
                    let mut output_buffer = [0u8; RADIO_PACKET_SIZE + 2];
                    output_buffer[..packet.length].copy_from_slice(&packet.data[..packet.length]);
                    // Append CRC-16 to the packet data
                    let crc = checksum16(&packet.data[..packet.length]);
                    output_buffer[packet.length..packet.length + 2].copy_from_slice(&crc.to_le_bytes());

                    let slice = &output_buffer[..packet.length + 2];
                    log::trace!("Calculated CRC-16: {:04X}", crc);

                    match lora.prepare_for_tx(mdltn_params, tx_pkt_params, 22, slice).await {
                        Ok(()) => {}
                        Err(_err) => {
                            return Err(RadioDeviceError::TransmissionFailed);
                        }
                    };
                }
                #[cfg(not(feature = "soft-packet-crc"))]
                {
                    let slice = &packet.data[..packet.length.min(RADIO_PACKET_SIZE)];
                    match lora.prepare_for_tx(mdltn_params, tx_pkt_params, 22, slice).await {
                        Ok(()) => {}
                        Err(_err) => {
                            return Err(RadioDeviceError::TransmissionFailed);
                        }
                    };
                }

                match lora.tx().await {
                    Ok(()) => {
                        transmit_enable.as_mut().map(|te| te.set_high());
                        return Ok(());
                    }
                    Err(_err) => {
                        transmit_enable.as_mut().map(|te| te.set_high());
                        return Err(RadioDeviceError::TransmissionFailed);
                    }
                };
            }
        }
    }

    /// Receive a message from the radio
    ///
    /// Configures the radio for continuous receive mode and waits for an incoming packet.
    /// The received data is copied to a RadioPacket structure with metadata extracted
    /// from the first few bytes. DOS protection is not required in this use-case,
    /// because the bottleneck here is the LoRa network's speed.
    ///
    /// # Returns
    /// * `Ok(RadioPacket)` containing the received data and metadata
    /// * `Err(RadioDeviceError::InitializationFailed)` if device is not initialized
    /// * `Err(RadioDeviceError::ReceiveFailed)` if receive preparation fails
    /// * `Err(RadioDeviceError::TransmissionFailed)` if receive operation fails
    ///
    /// # Examples
    /// ```rust,no_run
    /// # async fn example(radio: &mut moonblokz_radio_lib::radio_device_lora_sx1262::RadioDevice) -> Result<(), moonblokz_radio_lib::RadioDeviceError> {
    /// let packet = radio.receive_message().await?;
    /// println!("Received {} bytes", packet.length);
    /// # Ok(())
    /// # }
    /// ```
    async fn receive_packet(&mut self) -> Result<ReceivedPacket, RadioDeviceError> {
        match &mut self.state {
            RadioDeviceState::NotInited => {
                // Return error if not initialized
                Err(RadioDeviceError::InitializationFailed)
            }
            RadioDeviceState::Inited {
                lora,
                rx_pkt_params,
                mdltn_params,
                ..
            } => {
                match lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params).await {
                    Ok(()) => {}
                    Err(_) => {
                        return Err(RadioDeviceError::ReceiveFailed);
                    }
                };
                match lora.rx(rx_pkt_params, &mut self.receive_buffer).await {
                    Ok((rx_len, packet_status)) => {
                        // Create RadioPacket from received data
                        let mut data = [0u8; RADIO_PACKET_SIZE];
                        if rx_len > (RADIO_PACKET_SIZE + 2) as u8 {
                            return Err(RadioDeviceError::ReceiveFailed);
                        }
                        let copy_len = rx_len as usize;

                        #[cfg(feature = "soft-packet-crc")]
                        {
                            // Verify CRC-16 on received packet
                            if copy_len < 2 {
                                return Err(RadioDeviceError::ReceiveFailed);
                            }
                            let data_len = copy_len - 2;
                            let received_crc = u16::from_le_bytes([self.receive_buffer[data_len], self.receive_buffer[data_len + 1]]);
                            let calculated_crc = checksum16(&self.receive_buffer[..data_len]);

                            if received_crc != calculated_crc {
                                log::trace!(
                                    "CRC mismatch: received={:04X}, calculated={:04X}. Dropping packet.",
                                    received_crc,
                                    calculated_crc
                                );
                                return Err(RadioDeviceError::CRCMismatch);
                            }

                            // Copy only the data portion (without CRC)
                            data[..data_len].copy_from_slice(&self.receive_buffer[..data_len]);

                            Ok(ReceivedPacket {
                                packet: RadioPacket { data, length: data_len },
                                link_quality: calculate_link_quality(packet_status.rssi, packet_status.snr),
                            })
                        }
                        #[cfg(not(feature = "soft-packet-crc"))]
                        {
                            data[..copy_len].copy_from_slice(&self.receive_buffer[..copy_len]);

                            Ok(ReceivedPacket {
                                packet: RadioPacket { data, length: copy_len },
                                link_quality: calculate_link_quality(packet_status.rssi, packet_status.snr),
                            })
                        }
                    }
                    Err(_err) => Err(RadioDeviceError::ReceiveFailed),
                }
            }
        }
    }

    /// Perform Channel Activity Detection (CAD)
    ///
    /// CAD is used to detect if there is any LoRa activity on the configured channel
    /// before transmitting, helping to avoid collisions in shared spectrum.
    ///
    /// # Returns
    /// * `Ok(true)` if channel activity is detected
    /// * `Ok(false)` if no channel activity is detected
    /// * `Err(RadioDeviceError::InitializationFailed)` if device is not initialized
    /// * `Err(RadioDeviceError::CADFailed)` if CAD operation fails
    ///
    /// # Examples
    /// ```rust,no_run
    /// # async fn example(radio: &mut moonblokz_radio_lib::radio_device_lora_sx1262::RadioDevice) -> Result<(), moonblokz_radio_lib::RadioDeviceError> {
    /// let channel_busy = radio.do_cad().await?;
    /// if !channel_busy {
    ///     // Safe to transmit
    /// }
    /// # Ok(())
    /// # }
    /// ```
    async fn do_cad(&mut self) -> Result<bool, RadioDeviceError> {
        match &mut self.state {
            RadioDeviceState::NotInited => {
                // Return error if not initialized
                Err(RadioDeviceError::InitializationFailed)
            }
            RadioDeviceState::Inited { lora, mdltn_params, .. } => {
                lora.prepare_for_cad(mdltn_params).await.map_err(|_| RadioDeviceError::CADFailed)?;
                match lora.cad(&mdltn_params).await {
                    Ok(cad_result) => Ok(cad_result),
                    Err(_err) => Err(RadioDeviceError::CADFailed), // Use correct error type
                }
            }
        }
    }

    /// Check if the radio device has been initialized
    ///
    /// # Returns
    /// * `true` if the device is in the `Inited` state
    /// * `false` if the device is in the `NotInited` state
    ///
    /// # Examples
    /// ```rust
    /// use moonblokz_radio_lib::radio_device_lora_sx1262::RadioDevice;
    ///
    /// let radio = RadioDevice::new();
    /// assert!(!radio.is_initialized());
    /// ```
    pub fn is_initialized(&self) -> bool {
        matches!(self.state, RadioDeviceState::Inited { .. })
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    #[test]
    fn test_crc16c_empty() {
        let data = [];
        let crc = checksum16(&data);
        assert_eq!(crc, 0xFFFF);
    }

    #[test]
    fn test_crc16c_single_byte() {
        let data = [0x00];
        let crc = checksum16(&data);
        assert_ne!(crc, 0xFFFF);
    }

    #[test]
    fn test_crc16c_known_values() {
        // Test with a known pattern
        let data = b"123456789";
        let crc = checksum16(data);
        // CRC-16-CCITT with init 0xFFFF for "123456789" should be 0x29B1
        assert_eq!(crc, 0x29B1);
    }

    #[test]
    fn test_crc16c_deterministic() {
        let data = [0x01, 0x02, 0x03, 0x04, 0x05];
        let crc1 = checksum16(&data);
        let crc2 = checksum16(&data);
        assert_eq!(crc1, crc2);
    }

    #[test]
    fn test_crc16c_sensitivity() {
        let data1 = [0x01, 0x02, 0x03];
        let data2 = [0x01, 0x02, 0x04];
        let crc1 = checksum16(&data1);
        let crc2 = checksum16(&data2);
        assert_ne!(crc1, crc2);
    }

    #[test]
    #[cfg(feature = "std")]
    fn test_radio_device_init_error_implements_std_error() {
        // Test all RadioDeviceInitError variants
        let errors = [
            RadioDeviceInitError::InterfaceError,
            RadioDeviceInitError::LoraError,
            RadioDeviceInitError::ModulationParamsError,
            RadioDeviceInitError::TXPacketParamsError,
            RadioDeviceInitError::RXPacketParamsError,
        ];

        for err in &errors {
            // Verify Display implementation
            let display_str = format!("{}", err);
            assert!(!display_str.is_empty());

            // Verify std::error::Error trait is implemented
            let _: &dyn std::error::Error = err;
        }

        // Test specific messages
        assert_eq!(format!("{}", RadioDeviceInitError::InterfaceError), "failed to create SX126x interface variant");
        assert_eq!(format!("{}", RadioDeviceInitError::LoraError), "failed to initialize LoRa PHY layer");
    }

    #[test]
    fn test_error_types_are_debug_and_copy() {
        // Test RadioDeviceInitError
        let init_err = RadioDeviceInitError::InterfaceError;
        let _init_err_copy = init_err; // Test Copy
        let _init_err_debug = format!("{:?}", init_err); // Test Debug
    }
}

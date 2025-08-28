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
use embassy_rp::gpio::AnyPin;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Config, Spi};
use embassy_time::Delay;
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::LoRa;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx126x, Sx1262};
use lora_phy::{mod_params::*, sx126x};
use rand_core::RngCore;
use rand_core::SeedableRng;
use rand_wyrand::WyRand;

const CAD_MINIMAL_WAIT_TIME: u64 = 1000; // Minimum wait time in milliseconds after CAD command
const CAD_MAX_ADDITIONAL_WAIT_TIME: u64 = 1000; // Maximum additional wait time in milliseconds after CAD command

/// Radio device initialization errors
///
/// Represents specific errors that can occur during radio device initialization.
/// These provide more granular error information than the generic `InitializationFailed`.
#[derive(Debug)]
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

#[derive(Debug)]
enum RadioDeviceError {
    InitializationFailed,
    TransmissionFailed,
    ReceiveFailed,
    CADFailed,
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
pub struct RadioDevice {
    /// Current state of the device (initialized or not)
    state: RadioDeviceState,
    /// Buffer for receiving radio packets
    receive_buffer: [u8; RADIO_PACKET_SIZE], // Use consistent constant
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
            receive_buffer: [0u8; RADIO_PACKET_SIZE],
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
        spi_nss_pin: AnyPin,
        reset_pin: AnyPin,
        dio1_pin: AnyPin,
        busy_pin: AnyPin,
        transmit_pin_option: Option<AnyPin>,
        spi: embassy_rp::peripherals::SPI1,
        clk_pin: impl embassy_rp::spi::ClkPin<embassy_rp::peripherals::SPI1>,
        mosi_pin: impl embassy_rp::spi::MosiPin<embassy_rp::peripherals::SPI1>,
        miso_pin: impl embassy_rp::spi::MisoPin<embassy_rp::peripherals::SPI1>,
        tx_dma: embassy_rp::dma::AnyChannel,
        rx_dma: embassy_rp::dma::AnyChannel,
        lora_frequency_in_hz: u32,
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
        let spi_device = ExclusiveDevice::new(spi, spi_nss, Delay);

        let config = sx126x::Config {
            chip: Sx1262,
            tcxo_ctrl: None,
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
            match lora.create_modulation_params(SpreadingFactor::_10, Bandwidth::_250KHz, CodingRate::_4_8, lora_frequency_in_hz) {
                Ok(mp) => mp,
                Err(_err) => {
                    return Err(RadioDeviceInitError::ModulationParamsError);
                }
            }
        };
        let tx_pkt_params = {
            match lora.create_tx_packet_params(4, false, true, false, &mdltn_params) {
                Ok(pp) => pp,
                Err(_err) => {
                    return Err(RadioDeviceInitError::TXPacketParamsError);
                }
            }
        };

        let rx_pkt_params = {
            match lora.create_rx_packet_params(4, false, RADIO_PACKET_SIZE as u8, true, false, &mdltn_params) {
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

    async fn run(&mut self, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender, rng_seed: u64) -> ! {
        let mut rng = WyRand::seed_from_u64(rng_seed);
        loop {
            // Race between receiving a packet and getting a TX request
            match select(self.receive_message(), tx_receiver.receive()).await {
                Either::First(rx_result) => match rx_result {
                    Ok(packet) => {
                        let _ = rx_sender.try_send(packet);
                    }
                    Err(_e) => {
                        //receive error happened, but we don't log it to converse binary size.
                    }
                },
                Either::Second(tx_packet) => {
                    // Transmit the packet once: wait for a clear channel, then send and break
                    loop {
                        match self.do_cad().await {
                            Ok(false) => {
                                // Channel is clear; attempt a single send then exit loop
                                let _ = self.send_message(&tx_packet).await;
                                break;
                            }
                            Ok(true) | Err(_) => {
                                // Channel busy or CAD error: wait then retry CAD
                                Timer::after(embassy_time::Duration::from_millis(
                                    CAD_MINIMAL_WAIT_TIME + rng.next_u64() % CAD_MAX_ADDITIONAL_WAIT_TIME,
                                ))
                                .await;
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
    async fn send_message(&mut self, message: &RadioPacket) -> Result<(), RadioDeviceError> {
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
                transmit_enable.as_mut().map(|te| te.set_low());
                let slice = &message.data[..message.length.min(RADIO_PACKET_SIZE)];
                match lora.prepare_for_tx(mdltn_params, tx_pkt_params, 20, slice).await {
                    Ok(()) => {}
                    Err(_err) => {
                        return Err(RadioDeviceError::TransmissionFailed);
                    }
                };

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
    /// from the first few bytes.
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
    async fn receive_message(&mut self) -> Result<ReceivedPacket, RadioDeviceError> {
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
                        if rx_len > RADIO_PACKET_SIZE as u8 {
                            return Err(RadioDeviceError::ReceiveFailed);
                        }
                        let copy_len = rx_len as usize;
                        data[..copy_len].copy_from_slice(&self.receive_buffer[..copy_len]);

                        Ok(ReceivedPacket {
                            packet: RadioPacket { data, length: copy_len },
                            link_quality: calculate_link_quality(packet_status.rssi, packet_status.snr),
                        })
                    }
                    Err(_err) => Err(RadioDeviceError::TransmissionFailed),
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

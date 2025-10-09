//! Radio device implementations
//!
//! This module contains different radio device implementations that can be used
//! with the MoonBlokz Radio Library:
//!
//! - `echo`: Simple echo device for testing that echoes back received packets
//! - `simulator`: Network simulator for testing multi-node scenarios
//! - `rp_lora_sx1262`: LoRa SX1262 radio implementation for Raspberry Pi Pico
//! - `lib`: Link quality utilities for RSSI and SNR calculations

pub mod link_quality_calculations;

#[cfg(feature = "radio-device-rp-lora-sx1262")]
pub mod rp_lora_sx1262;

#[cfg(feature = "radio-device-echo")]
pub mod echo;

#[cfg(feature = "radio-device-simulator")]
pub mod simulator;

// Re-export the active radio device implementation
#[cfg(feature = "radio-device-rp-lora-sx1262")]
pub use rp_lora_sx1262::{radio_device_task, RadioDevice};

#[cfg(feature = "radio-device-echo")]
pub use echo::{radio_device_task, RadioDevice};

#[cfg(feature = "radio-device-simulator")]
pub use simulator::{radio_device_task, RadioDevice};

// Re-export link quality utilities
pub use link_quality_calculations::{calculate_link_quality, normalize};

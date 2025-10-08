//! # Radio Device Echo - Loopback Device for Testing
//!
//! This module provides a minimal radio device implementation that echoes transmitted
//! packets back to the receiver. It's designed for basic testing and validation of the
//! radio communication stack without requiring network simulation or hardware.
//!
//! ## Architecture
//!
//! The echo device is the simplest possible radio implementation:
//! - Receives packets from the TX queue
//! - Immediately sends them to the RX queue with maximum link quality
//! - No actual radio transmission occurs
//! - No timing delays or CAD simulation
//! - Single-node testing only (no multi-node network)
//!
//! ## Key Components
//!
//! - **RadioDevice**: Minimal state struct (zero-sized)
//!   - No configuration required
//!   - Stateless operation
//!
//! - **radio_device_task**: Async task implementing echo logic
//!   - Infinite loop receiving from TX queue
//!   - Immediate forwarding to RX queue
//!   - Handles backpressure with packet dropping
//!
//! ## Link Quality
//!
//! All echoed packets are marked with maximum link quality (63):
//! - Simulates perfect signal reception
//! - Useful for testing connection quality logic
//! - Deterministic behavior for unit tests
//!
//! ## Use Cases
//!
//! - Unit testing message serialization/deserialization
//! - Validating message processing logic without network
//! - Debugging packet format issues
//! - Smoke testing basic communication flow
//! - Development without hardware dependencies
//!
//! ## Limitations
//!
//! - Cannot test multi-node behavior or relay logic
//! - No realistic timing characteristics
//! - No channel contention or collision simulation
//! - Single node only - no network topology
//!
//! ## Design Considerations
//!
//! - Pool size supports MAX_NODE_COUNT (though typically used with 1 node)
//! - Backpressure handling drops packets if RX queue is full
//! - Zero-cost abstraction with no runtime overhead
//! - Compatible with the same API as other radio device implementations

use crate::MAX_NODE_COUNT;
use crate::RxPacketQueueSender;
use crate::TxPacketQueueReceiver;
use log::{Level, log};

/// Echo radio device task - loops packets back for testing
///
/// This async task implements a simple loopback radio device that immediately
/// echoes transmitted packets back to the receiver with maximum link quality.
/// It runs in an infinite loop until the executor is stopped.
///
/// # Arguments
/// * `radio_device` - RadioDevice instance (zero-sized, stateless)
/// * `tx_receiver` - Channel receiver for outgoing packets
/// * `rx_sender` - Channel sender for incoming (echoed) packets
/// * `own_node_id` - This node's unique identifier for logging
/// * `_rng_seed` - Unused random seed (kept for API compatibility)
///
/// # Task Pool
/// Uses a pool of MAX_NODE_COUNT to support multiple concurrent instances,
/// though typically only one instance is used for single-node testing.
///
/// # Behavior
/// - Receives packets from TX queue (blocking)
/// - Forwards them to RX queue with link quality = 63 (maximum)
/// - Drops packets if RX queue is full (backpressure handling)
/// - Logs warnings when packets are dropped with node identification
#[embassy_executor::task(pool_size = MAX_NODE_COUNT)]
pub async fn radio_device_task(
    mut radio_device: RadioDevice,
    tx_receiver: TxPacketQueueReceiver,
    rx_sender: RxPacketQueueSender,
    own_node_id: u32,
    _rng_seed: u64,
) -> ! {
    log!(Level::Info, "[{}] Echo radio device task started", own_node_id);
    radio_device.run(tx_receiver, rx_sender, own_node_id).await
}

/// Echo radio device - loopback implementation for testing
///
/// A zero-sized, stateless radio device that echoes transmitted packets back
/// to the receiver. This is the simplest possible radio device implementation,
/// useful for testing without network simulation or hardware.
///
/// # Characteristics
/// - Zero runtime overhead (no state storage)
/// - No configuration required
/// - Deterministic behavior (no timing variation)
/// - Maximum link quality on all packets (63)
///
/// # Example
/// ```rust,ignore
/// let radio_device = RadioDevice::new();
/// // Device is ready to use immediately, no initialization needed
/// ```
#[cfg_attr(feature = "std", derive(Debug))]
pub struct RadioDevice {}

impl Default for RadioDevice {
    fn default() -> Self {
        Self::new()
    }
}

impl RadioDevice {
    /// Creates a new echo radio device
    ///
    /// This is a const constructor that can be used at compile time.
    /// The device requires no configuration and has no internal state.
    ///
    /// # Returns
    /// A new RadioDevice instance ready for use
    ///
    /// # Example
    /// ```rust,ignore
    /// const RADIO: RadioDevice = RadioDevice::new();
    /// ```
    pub const fn new() -> Self {
        RadioDevice {}
    }

    /// Main echo loop - receives and echoes packets indefinitely
    ///
    /// Implements the core loopback logic by receiving packets from the TX queue
    /// and immediately forwarding them to the RX queue with maximum link quality.
    ///
    /// # Arguments
    /// * `tx_receiver` - Channel receiver for outgoing packets to echo
    /// * `rx_sender` - Channel sender for incoming (echoed) packets
    ///
    /// # Returns
    /// Never returns (runs forever until executor stops)
    ///
    /// # Behavior
    /// - Blocks waiting for packets from TX queue
    /// - Echoes each packet to RX queue with link quality = 63
    /// - Drops packets if RX queue is full (logs warning)
    /// - Continues indefinitely
    ///
    /// # Backpressure Handling
    /// If the RX queue is full, the echoed packet is dropped and a warning
    /// is logged showing the message type. This prevents deadlock when the
    /// receiver cannot keep up with transmission rate.
    async fn run(&mut self, tx_receiver: TxPacketQueueReceiver, rx_sender: RxPacketQueueSender, own_node_id: u32) -> ! {
        loop {
            let packet = tx_receiver.receive().await;
            log::trace!("[{}] Echoing packet: type {}", own_node_id, packet.message_type());
            match rx_sender.try_send(crate::ReceivedPacket { packet, link_quality: 63 }) {
                Ok(_) => {}
                Err(embassy_sync::channel::TrySendError::Full(received_packet)) => {
                    // Backpressure: drop echoed packet and log
                    log!(
                        Level::Warn,
                        "[{}] RX queue full, dropping echoed packet. type: {}",
                        own_node_id,
                        received_packet.packet.message_type()
                    );
                }
            }
        }
    }
}

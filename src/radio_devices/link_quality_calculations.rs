//! Radio device link quality utilities
//!
//! This module provides utilities for calculating link quality metrics from
//! radio signal measurements including RSSI (Received Signal Strength Indicator)
//! and SNR (Signal-to-Noise Ratio).
//!
//! The link quality calculation combines both RSSI and SNR measurements into
//! a single quality score on a 0-63 scale, with SNR weighted more heavily
//! than RSSI since it's a better indicator of link reliability.

/// Minimum RSSI value for decodable signals (in dBm)
///
/// Represents the threshold below which the radio signal is too weak for reliable
/// communication. Signals weaker than this are typically below the noise floor and cannot
/// be reliably decoded. Used as the lower bound for link quality normalization.
const RSSI_MIN: i16 = -120;

/// Maximum RSSI value for very strong signals (in dBm)
///
/// Represents a signal very close to the receiver with maximum strength.
/// Used as the upper bound for link quality normalization.
const RSSI_MAX: i16 = -30;

/// Minimum SNR value for decodable signals (in dB)
///
/// Can be negative for LoRa due to spread spectrum processing gain.
/// Used as the lower bound for SNR normalization.
const SNR_MIN: i16 = -20;

/// Maximum SNR value for very clean signals (in dB)
///
/// Represents a signal with minimal noise and interference.
/// Used as the upper bound for SNR normalization.
const SNR_MAX: i16 = 10;

/// Normalizes a value to a 0-63 scale based on defined min/max bounds
///
/// Clamps the input value within the specified range and linearly scales it
/// to the 0-63 output range. Values below `min` result in 0, values above
/// `max` result in 63.
///
/// # Arguments
/// * `value` - The input value to normalize (e.g., -90 for RSSI)
/// * `min` - The bottom of the input range (e.g., -120 for RSSI_MIN)
/// * `max` - The top of the input range (e.g., -30 for RSSI_MAX)
///
/// # Returns
/// A u8 value in the range 0-63 representing the normalized quality
///
/// # Example
/// ```rust
/// use moonblokz_radio_lib::radio_devices::normalize;
///
/// // RSSI of -90 dBm (between -120 and -30 range)
/// let quality = normalize(-90, -120, -30);
/// assert!(quality > 0 && quality < 63);
///
/// // Value below minimum
/// let quality = normalize(-150, -120, -30);
/// assert_eq!(quality, 0);
///
/// // Value above maximum
/// let quality = normalize(-20, -120, -30);
/// assert_eq!(quality, 63);
/// ```
pub fn normalize(value: i16, min: i16, max: i16) -> u8 {
    // 1. Clamp the value to ensure it's within the defined range.
    let clamped_value = value.max(min).min(max);

    // 2. Shift the range to start at 0.
    let shifted_value = clamped_value - min;

    // 3. Scale the value to the 0-63 range using integer arithmetic.
    // We multiply by 63 first to maintain precision before the division.
    let scaled_value = (shifted_value as u32 * 63) / (max - min) as u32;

    scaled_value as u8
}

/// Calculates combined link quality from RSSI and SNR measurements
///
/// Combines Received Signal Strength Indicator (RSSI) and Signal-to-Noise Ratio (SNR)
/// into a single quality metric on a 0-63 scale. The calculation uses a weighted average
/// favoring SNR (70%) over RSSI (30%) since SNR is a better indicator of link reliability.
///
/// The quality score is used by the relay manager to track connection quality between
/// nodes and make intelligent routing decisions.
///
/// # Arguments
/// * `rssi` - Raw RSSI value in dBm (typically -120 to -30)
/// * `snr` - Raw SNR value in dB (typically -20 to 10)
///
/// # Returns
/// A u8 link quality score in the range 0-63
/// - 0: Unusable link (very poor signal)
/// - 63: Excellent link (strong signal and low noise)
///
/// # Algorithm
/// 1. Normalize RSSI to 0-63 scale using RSSI_MIN/RSSI_MAX bounds
/// 2. Normalize SNR to 0-63 scale using SNR_MIN/SNR_MAX bounds
/// 3. Calculate weighted average: (30% × RSSI + 70% × SNR)
///
/// # Example
/// ```rust
/// use moonblokz_radio_lib::radio_devices::calculate_link_quality;
///
/// // Good signal with decent SNR
/// let quality = calculate_link_quality(-70, 5);
/// assert!(quality > 40); // Should be in "good" range
///
/// // Weak signal with poor SNR
/// let quality = calculate_link_quality(-110, -15);
/// assert!(quality < 20); // Should be in "poor" range
/// ```
pub fn calculate_link_quality(rssi: i16, snr: i16) -> u8 {
    // 1. Normalize both RSSI and SNR to a common 0-63 scale.
    let norm_rssi = normalize(rssi, RSSI_MIN, RSSI_MAX);
    let norm_snr = normalize(snr, SNR_MIN, SNR_MAX);

    // 2. Calculate the weighted average using integer math.
    // Weights: 7 for SNR, 3 for RSSI. Total weight is 10.
    // We use u32 for the intermediate calculation to prevent overflow.
    let quality = (3 * norm_rssi as u32 + 7 * norm_snr as u32) / 10;

    // The result is guaranteed to be in the 0-63 range.
    quality as u8
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_rssi() {
        // Test RSSI normalization
        assert_eq!(normalize(RSSI_MIN, RSSI_MIN, RSSI_MAX), 0);
        assert_eq!(normalize(RSSI_MAX, RSSI_MIN, RSSI_MAX), 63);

        // Test mid-range value
        let mid_rssi = (RSSI_MIN + RSSI_MAX) / 2;
        let normalized = normalize(mid_rssi, RSSI_MIN, RSSI_MAX);
        assert!(normalized > 25 && normalized < 40);

        // Test out-of-bounds values
        assert_eq!(normalize(-150, RSSI_MIN, RSSI_MAX), 0);
        assert_eq!(normalize(-20, RSSI_MIN, RSSI_MAX), 63);
    }

    #[test]
    fn test_normalize_snr() {
        // Test SNR normalization
        assert_eq!(normalize(SNR_MIN, SNR_MIN, SNR_MAX), 0);
        assert_eq!(normalize(SNR_MAX, SNR_MIN, SNR_MAX), 63);

        // Test mid-range value
        let mid_snr = (SNR_MIN + SNR_MAX) / 2;
        let normalized = normalize(mid_snr, SNR_MIN, SNR_MAX);
        assert!(normalized > 25 && normalized < 40);

        // Test out-of-bounds values
        assert_eq!(normalize(-30, SNR_MIN, SNR_MAX), 0);
        assert_eq!(normalize(20, SNR_MIN, SNR_MAX), 63);
    }

    #[test]
    fn test_calculate_link_quality() {
        // Test with good signal
        let quality = calculate_link_quality(-70, 5);
        assert!(quality > 40, "Good signal should have quality > 40, got {}", quality);

        // Test with poor signal
        let quality = calculate_link_quality(-110, -15);
        assert!(quality < 20, "Poor signal should have quality < 20, got {}", quality);

        // Test with perfect signal
        let quality = calculate_link_quality(RSSI_MAX, SNR_MAX);
        assert_eq!(quality, 63, "Perfect signal should have quality 63, got {}", quality);

        // Test with worst signal
        let quality = calculate_link_quality(RSSI_MIN, SNR_MIN);
        assert_eq!(quality, 0, "Worst signal should have quality 0, got {}", quality);

        // Test SNR weighting (70% SNR vs 30% RSSI)
        let high_snr_low_rssi = calculate_link_quality(RSSI_MIN, SNR_MAX);
        let low_snr_high_rssi = calculate_link_quality(RSSI_MAX, SNR_MIN);
        assert!(
            high_snr_low_rssi > low_snr_high_rssi,
            "High SNR should outweigh high RSSI: {} vs {}",
            high_snr_low_rssi,
            low_snr_high_rssi
        );
    }
}

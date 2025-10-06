# Testing Guide for moonblokz-radio-lib

This document explains how to run tests for the moonblokz-radio-lib project.

## Test Configuration

The project is configured to run tests **without default features** to avoid dependency conflicts. Tests **require both a radio device feature and a memory configuration feature** because the `RadioCommunicationManager` uses `RadioDevice` types and depends on memory configuration constants.

## Running Tests

### Option 1: Using Cargo Aliases (Recommended)

The project provides convenient cargo aliases defined in `.cargo/config.toml`:

```bash
# Run tests with std, radio-device-echo, and memory-config-large features (RECOMMENDED)
cargo test-echo

# Alternative alias (same as test-echo)
cargo test-all
```

**Note:** The `radio-device-echo` feature is the simplest radio device implementation and doesn't have dependency conflicts, making it ideal for testing. The `memory-config-large` feature provides the most generous memory configuration (100-node connection matrix).

### Option 2: Using Full Commands

If you prefer explicit commands:

```bash
# Run all tests with echo radio device and large memory config (RECOMMENDED)
cargo test --lib --no-default-features --features std,radio-device-echo,memory-config-large

# Run specific test
cargo test --lib --no-default-features --features std,radio-device-echo,memory-config-large test_normalize_within_bounds

# Run tests with output
cargo test --lib --no-default-features --features std,radio-device-echo,memory-config-large -- --nocapture
```

## Test Organization

Tests are organized within the source files:

- **`src/lib.rs`** - Core functionality tests (link quality, scoring matrix, messages, constants, CRC)
- **`src/relay_manager.rs`** - RelayManager and ConnectionMatrix tests
- **`src/messages/radio_message.rs`** - RadioMessage and MessageType tests
- **`src/messages/radio_packet.rs`** - RadioPacket tests
- **`src/tx_scheduler.rs`** - TX scheduler tests
- **`src/rx_handler.rs`** - RX handler tests
- **`src/wait_pool.rs`** - WaitPool tests

## Test Categories

### Link Quality Tests

- Normalization (RSSI/SNR to 0-63 scale)
- Strong/weak signal detection
- Extreme value handling
- Deterministic behavior

### Scoring Matrix Tests

- Matrix creation and encoding/decoding
- Bit packing/unpacking
- Symmetry verification

### Message Tests

- Message type uniqueness
- Cloning and equality
- Empty/large payloads
- Sequence number handling

### Connection Matrix Tests

- Type alias behavior
- Initialization and access
- Quality and dirty bit manipulation
- Full-size matrix operations

### Constants Validation Tests

- Packet size constraints
- Message size relationships
- Type size validations

### CRC/Integrity Tests

- Checksum consistency
- Checksum sensitivity

## Known Issues

### Critical-Section Dependency Conflict

When running tests with `radio-device-lora-sx1262` (default feature), you may encounter a `critical-section` dependency conflict:

```rust
error[E0428]: the name `RawRestoreStateInner` is defined multiple times
```

**Workaround:** Use `--no-default-features --features std,radio-device-echo` as shown in the commands above.

### Tests Require a Radio Device Feature

Tests cannot run with only the `std` feature because `RadioCommunicationManager` requires `RadioDevice` types. You **must** include a radio device feature:

- ✅ `radio-device-echo` - Recommended for testing (simple, no conflicts)
- ⚠️ `radio-device-lora-sx1262` - May have dependency conflicts
- ✅ `radio-device-simulator` - Alternative for testing

**Always use:** `cargo test-echo` or include `radio-device-echo` in your feature flags.

## CI/CD Configuration

For continuous integration, use:

```yaml
# .github/workflows/test.yml example
- name: Run tests
  run: cargo test --lib --no-default-features --features std,radio-device-echo
```

## Test Coverage

Current test coverage includes:

- ~35+ new comprehensive tests
- ~30+ existing tests
- **Total: 65+ tests**

Coverage areas:

- ✅ Type aliases and type system
- ✅ Link quality calculations
- ✅ Scoring matrix operations
- ✅ Message creation and parsing
- ✅ Edge cases and boundaries
- ✅ Constants validation
- ✅ CRC/checksum integrity

## Adding New Tests

When adding new tests:

1. Add tests to the appropriate source file (not separate test files)
2. Use the `#[cfg(all(test, feature = "std"))]` guard for test modules
3. Follow existing test naming conventions: `test_<functionality>_<scenario>`
4. Add documentation comments explaining what the test validates
5. Run tests with `cargo test-std` or `cargo test-echo` to verify

## Example Test

```rust
#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    #[test]
    fn test_my_new_feature() {
        let result = my_function(42);
        assert_eq!(result, expected_value);
    }
}
```

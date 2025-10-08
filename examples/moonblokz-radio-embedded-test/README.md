# MoonBlokz Radio Embedded Test

This example demonstrates LORA P2P send functionality running on the Raspberry Pi Pico with a Waveshare board containing a Semtech Sx1262 radio.

## Compile-Time Log Level Filtering

This project uses the `log` crate's built-in compile-time log level filtering features that allow you to exclude lower-level log statements from the compiled binary, significantly reducing binary size and improving performance for embedded applications.

### Available Log Level Features

The following Cargo features control which log levels are compiled into the binary using the log crate's `max_level_*` features:

- `log-trace` - Includes all log levels (trace, debug, info, warn, error) via `log/max_level_trace`
- `log-debug` - Includes debug, info, warn, and error logs via `log/max_level_debug`
- `log-info` - Includes info, warn, and error logs via `log/max_level_info`
- `log-warn` - Includes warn and error logs only via `log/max_level_warn`
- `log-error` - Includes error logs only via `log/max_level_error`
- `log-off` - Disables all logging via `log/max_level_off`

**Important**: Only use one log level feature at a time. The log crate will produce a compile error if multiple `max_level_*` features are enabled simultaneously.

### Usage Examples

Build with different log levels using Cargo features:

```bash
# Build with default settings (no compile-time log filtering - all logs included)
cargo build --target thumbv6m-none-eabi

# Build with info level logging (excludes debug/trace logs at compile time)
cargo build --target thumbv6m-none-eabi --features log-info

# Build with debug level logging (excludes trace logs at compile time)
cargo build --target thumbv6m-none-eabi --features log-debug

# Build with error logging only (smallest binary, excludes debug/info/warn logs)
cargo build --target thumbv6m-none-eabi --features log-error

# Build with all logging disabled (smallest possible binary)
cargo build --target thumbv6m-none-eabi --features log-off

# Build with trace level logging (largest binary, most verbose)
cargo build --target thumbv6m-none-eabi --features log-trace
```

### In Your Code

Simply use the standard `log` crate macros. The compile-time filtering is handled automatically by the log crate based on the selected features:

```rust
use log::{trace, debug, info, warn, error};

// These will be compiled out if the corresponding max_level feature is not enabled
trace!("Very detailed tracing information");  // Only with log-trace
debug!("Debug information for development");   // Only with log-debug or log-trace  
info!("General information about program execution");  // Only with log-info, log-debug, or log-trace
warn!("Warning about potential issues");       // Only with log-warn, log-info, log-debug, or log-trace
error!("Error conditions that need attention"); // Only with log-error, log-warn, log-info, log-debug, or log-trace
```

### Binary Size Benefits

By using the log crate's compile-time log filtering, you can significantly reduce binary size:

- `log-off`: Smallest binary, no logging overhead at all
- `log-error`: Very small binary, critical errors only
- `log-warn`: Small binary, warnings and errors
- `log-info`: Medium binary, general information and above
- `log-debug`: Larger binary, includes development information  
- `log-trace`: Largest binary, very verbose logging

The log statements that are filtered out at compile time are completely removed from the binary, not just disabled at runtime.

### Runtime Log Level

The runtime log level is automatically set based on the compile-time feature selection using the `logging::get_runtime_log_level()` function, ensuring consistent behavior between compile-time filtering and runtime filtering.

## Building and Flashing

```bash
# Check the project builds correctly
cargo check --target thumbv6m-none-eabi

# Build for release with optimized logging
cargo build --target thumbv6m-none-eabi --release --no-default-features --features log-error

# Flash to device (if you have the appropriate tools set up)
./deploy.sh
```

## Hardware Requirements

- Raspberry Pi Pico
- Waveshare Pico-LoRa-SX1262 module or similar LoRa radio module
- USB connection for console output

## Project Structure

- `src/main.rs` - Main application logic with standard log macro usage
- `src/panic.rs` - Panic handler for embedded environment
- `Cargo.toml` - Project configuration with log crate max_level features
- `memory.x` - Memory layout for the target device

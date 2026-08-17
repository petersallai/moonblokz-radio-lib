# Repository Guidelines

## Project Structure & Module Organization
Main code is in `src/`. Keep responsibilities separated:
- `lib.rs`: public API, compile-time feature validation, config types.
- `radio_devices/`: hardware state machines (`echo`, `simulator`, `rp_lora_sx1262`).
- `tx_scheduler.rs`, `rx_handler.rs`, `relay_manager.rs`, `wait_pool.rs`: the three-task pipeline and relay decision flow.
- `messages/`: `RadioMessage`/`RadioPacket` serialization and fragmentation logic.

Examples:
- `examples/moonblokz-radio-std-test/` for host simulation.
- `examples/moonblokz-radio-embedded-test/` for RP2040 builds.

## Build, Test, and Development Commands
- `cargo build --features std,radio-device-simulator,memory-config-large`  
  Build for local simulation.
- `cargo run --example moonblokz-radio-std-test --features std,radio-device-echo,memory-config-large`  
  Run the standard example.
- `cargo test --features std,radio-device-echo,memory-config-large`  
  Run unit tests with the echo backend.
- `cargo test --features std,radio-device-simulator,memory-config-large`  
  Run unit tests with simulator backend.
- `cd examples/moonblokz-radio-embedded-test && cargo build --target thumbv6m-none-eabi --features radio-device-rp-lora-sx1262,memory-config-medium`  
  Build embedded example for RP2040.

## Architecture Invariants (From VII/2 and VII/3)
- Preserve deterministic memory behavior: no heap allocation, bounded queues, fixed-size buffers.
- Keep the three async-task model intact: TX scheduler, radio device, RX handler.
- Radio device must remain single-action at any moment (`tx` XOR `rx` XOR `CAD`).
- Relay logic is stateful and fire-and-forget: no ACK protocol, no TTL-based flooding control.
- Connection matrix updates come from echo flow plus normal packet traffic; stale links age out.

## RP2040/LoRa Bring-Up Notes (From VII/1)
- For WaveShare RP2040-LoRa wiring, verify SPI and control pin mapping before debugging higher layers.
- On boards without TCXO, `sx126x::Config.tcxo_ctrl` must be `None`; incorrect TCXO config can block `tx()`/`rx()`.
- Typical deploy flow for boards without debug probe: build ELF, convert with `elf2uf2-rs`, copy UF2 to `RPI-RP2`.

## Coding Style & Naming Conventions
Use Rust 2021 idioms with `snake_case` for functions/modules, `CamelCase` for types, and `SCREAMING_SNAKE_CASE` for constants. Use 4-space indentation and explicit `#[cfg(...)]` gates for `std` vs `no_std` paths. Preserve node-id-prefixed logs for multi-node tracing.

Run formatting/lints before opening a PR:
- `cargo fmt`
- `cargo clippy --features std,radio-device-echo,memory-config-large -- -D warnings`

## Testing Guidelines
Tests are colocated with code in `#[cfg(all(test, feature = "std"))]` modules (for example in `src/relay_manager.rs` and `src/messages/radio_message.rs`). Name tests by behavior, e.g. `echo_request_receives_and_queues_reply`. Cover both normal flow and relay edge cases when changing routing logic.

## Commit & Pull Request Guidelines
Use short, imperative commit subjects (example: `Refactor spawner parameter...`). Keep commits focused and describe *what changed*.

PRs should include:
- purpose and scope,
- enabled feature set(s) used for testing,
- command output summary for `cargo test`,
- linked issue(s) when applicable.

## Further Information
- Mesh Radio Algorithm: https://medium.com/moonblokz/moonblokz-series-part-vii-2-mesh-radio-algorithm-3650af3711f3
- Inside the Radio Module: https://medium.com/moonblokz/moonblokz-series-part-vii-3-inside-the-radio-module-d92545624d2b

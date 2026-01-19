# MoonBlokz Radio Library - AI Coding Agent Instructions

## Project Overview

This is a `no_std` Rust radio mesh networking library for the MoonBlokz blockchain's physical layer. It implements autonomous broadcast-based communication over LoRa, designed for microcontrollers with severe memory constraints.

**Core Philosophy**: No acknowledgments, no handshakes, no centralized coordination. The network self-organizes through periodic echo probing and intelligent relay decisions based on local topology knowledge.

**Design Principles**:
- **Deterministic Memory**: All memory is statically allocated at compile time. No heap usage, no runtime allocation errors.
- **Asynchronous Architecture**: Built on Embassy framework for efficient async/await on embedded systems without OS overhead
- **Fire-and-forget Transmission**: Messages are broadcast without acknowledgment. Reliability comes from redundant relay paths.
- **Reactive Fault Tolerance**: Network detects and recovers from missing data reactively through blockchain structure feedback
- **Stateful Relaying**: Nodes relay only if they don't already have the data, preventing loops and broadcast storms

**Key Constraints**:
- LoRa duty cycle limits (typically 1% airtime or less)
- Radio can only perform ONE action at a time (transmit XOR receive XOR CAD)
- 3D spatial operation (not limited to 2D topologies)
- Must work on RP2040 (Cortex-M0+, 264KB RAM, 2MB flash)

## Architecture

### Three-Task Async System (Embassy Framework)

The library runs three independent async tasks connected via bounded queues:

1. **TX Scheduler** (`src/tx_scheduler.rs`)
   - Entry point for all outgoing data (application + relay logic)
   - Enforces radio timing parameters and packetizes large messages
   - Queue acts as first protective backstop - drops new messages when overloaded
   - **Timing coordination**: Maintains predefined delay between messages, pauses during other node's multi-packet transmissions
   - **Duty Cycle Management**: Controlled via `delay_between_tx_messages` parameter to comply with LoRa airtime regulations
   - Once transmission starts, continues sending packets in sequence without interruption

2. **Radio Device** (`src/radio_devices/`)
   - Low-level manager for physical radio (constrained to ONE action at a time: TX or RX or CAD)
   - Default mode: always listening for packets
   - When packet ready to send: stops listening → CAD check → transmit if clear → resume listening
   - **State Machine**: Coordinates between listening, CAD, and transmission states
   - **Hardware Implementations**:
     - `rp_lora_sx1262.rs` - RP2040 + Semtech SX1262 LoRa chip (production, configurable pins/TCXO)
     - `simulator.rs` - Virtual radio for multi-node network simulation
     - `echo.rs` - Simple echo device for unit testing

3. **RX Handler** (`src/rx_handler.rs`)
   - Handles all inbound traffic and reassembles multi-packet messages
   - Communicates with application layer via `incoming_message_queue`
   - Triggers relaying logic via `RelayManager`
   - Queue acts as final buffer - drops newly received messages if application is slow

**Failure Isolation**: Architecture ensures failures (app bottleneck, congested channel) cause predictable message drops, NOT memory overflow or crashes.

### Five Core Components

1. **RadioDevice** - Hardware abstraction (see above)

2. **TX Scheduler** - Outgoing message coordination (see above)

3. **RX Handler** - Incoming message assembly (see above)

4. **RelayManager** (`src/relay_manager.rs`) - **Most Complex Module**
   - Maintains NxN connection quality matrix tracking all node pairs
   - Implements echo protocol (request_echo → echo → echo_result)
   - Makes relay decisions using scoring algorithm and connection matrix
   - Processes responses from application layer to determine relaying eligibility
   - No independent run loop - functions invoked from `rx_handler` task
   - **Connection Matrix**: Two-dimensional u8 array, upper 2 bits = dirty counter (aging), lower 6 bits = quality (0-63)
   - **Dirty Counter Aging**: Increments on each `request_echo`, resets to 0 on echo response/result, connection zeroed after 3 missed cycles

5. **WaitPool** (`src/wait_pool.rs`) 
   - Temporarily stores messages before relaying with calculated delay
   - Fixed-size array (capacity determined by memory config)
   - Each item stores: message, activation_time, aggregated message_connections, optional requester_index
   - Drops lowest-scoring items when full
   - Manages relay timing: delay = position × relay_position_delay + jitter

### Message Flow

```
App → OutgoingQueue → TXScheduler → RadioDevice → [network] → RadioDevice → RXHandler → RelayManager/WaitPool → App
```

### Echo Protocol (Critical for Network Discovery)

**Purpose**: Active probing process that builds and maintains the connection matrix showing link quality between all known nodes.

**Three-Step Process**:

1. **Probe Scheduling** - Node broadcasts `request_echo` message
   - Frequency determined by: minimum interval (e.g., 1 day) + target interval
   - Target interval adapts based on neighbor count (more neighbors = longer interval to reduce airtime)
   - **Before broadcast**: Increments dirty counter for ALL connections in matrix (aging mechanism)
   - Network expected to be largely static - infrequent probes acceptable

2. **Neighbor Responses** - Nodes send `echo` message with link quality
   - Response includes signal quality of the received `request_echo`
   - Random back-off delay added to prevent collision
   - Queued via dedicated `echo_responses_wait_pool` (separate from general relay pool)
   - If pool full: immediately transmit oldest pending echo
   - **On response**: Dirty counter reset to 0 for that connection

3. **Aggregation & Broadcast** - Original requester sends `echo_result`
   - Waits up to `echo_gathering_timeout` for responses
   - Compiles bidirectional link qualities into single message
   - Orders responders by link quality, sends as many as fit in packet
   - **On receiving result**: Dirty counter reset to 0 for all included connections

**Link Quality Calculation** (0-63 scale):
```
link_quality = 0.7 × SNR(normalized) + 0.3 × RSSI(normalized)
```
- SNR weighted higher (70%) - better predictor of packet delivery
- RSSI and SNR normalized from hardware range to 0-63
- Values below hardware floor clamped before normalization

**Dirty Counter Logic**: 
- Starts at 0 for fresh connections
- Increments to 1 after first missed `request_echo`
- Increments to 2 after second miss
- At 3 (MAX_DIRTY_COUNT): connection zeroed and considered lost
- **Survival**: Connections survive one missed echo cycle but are zeroed after two consecutive misses

**Continuous Updates**: Matrix also updated from ANY received radio packet (not just echo messages), keeping quality values current.

## Memory Constraints

**Three compile-time configurations** (mutually exclusive features):
- `memory-config-small` - CONNECTION_MATRIX_SIZE=10, ~25KB RAM
- `memory-config-medium` - CONNECTION_MATRIX_SIZE=30, ~60KB RAM  
- `memory-config-large` - CONNECTION_MATRIX_SIZE=100, ~120KB RAM (default)

These affect: connection matrix size, wait pool size, packet buffers. Do NOT change without understanding RAM implications.

**Debug Features**:
- `connection-matrix-logging` - Enables `MessageProcessingResult::RequestConnectionMatrixIntoLog` and `log_connection_matrix()` method
  - Logs connection matrix at error level in compact encoded format (A-Z, a-z, 0-9, -, _ = values 0-63)
  - Each log line prefixed with `[node_id]` for consistency
  - Long rows split into chunks of 100 characters per line

**Memory Management Philosophy**:
- **Fixed-Size Buffers**: All radio messages/packets allocated to maximum supported size
- **Bounded Data Structures**: All internal structures have fixed, compile-time size
- **Embassy Integration**: Framework uses static memory for queues and task state
- **No Heap Usage**: Eliminates runtime errors related to memory exhaustion
- **Deterministic Allocation**: All memory usage known at compile time

## Blockchain Messages

MoonBlokz nodes exchange seven core message types for blockchain synchronization:

1. **add_block** - Broadcasts newly created block to network (multi-packet capable)
2. **request_full_block** - Requests specific block by sequence number (and optionally parent hash)
3. **request_block_part** - Requests missing packets from incomplete `add_block` message
4. **add_transaction** - Broadcasts new transaction to network (multi-packet capable)
5. **request_transaction_part** - Requests missing packets from incomplete `add_transaction`
6. **request_new_mempool_item** - Propagates hashes of top mempool transactions (periodic, low-frequency)
7. **support** - Provides majority approval for fallback block creator

**Message Sizing**:
- **Multi-packet messages**: Only `add_block` and `add_transaction` support fragmentation
- **Single-packet messages**: All others fit in ~100 bytes, compatible with any radio hardware
- Fragmentation/reassembly handled transparently by TX Scheduler and RX Handler

## Network Resilience & Fault Tolerance

**Stateful Relaying (Loop Prevention)**:
- Nodes relay data propagation messages (`add_block`, `add_transaction`, `support`) ONLY if they don't already have the data
- Natural termination: propagation stops automatically when region is saturated
- No TTL needed: inherent loop prevention through "relay only once" rule
- Avoids broadcast storms that plague traditional flooding algorithms

**Reactive Fault Tolerance**:
- No confirmations required for normal operation
- Blockchain structure provides feedback loop (missing parent detected when child arrives)
- Node detects gap when receiving Block N+1 without Block N
- Automatically broadcasts `request_block` to recover missing block
- Re-sends request if no response within timeout
- Self-healing without explicit delivery confirmation

**Delay & Disorder Tolerance**:
- Fully asynchronous, delay-tolerant design
- Messages processed as they arrive, even out of order
- Child blocks cached temporarily until parent retrieved
- No complex priority queues needed
- Simplicity reduces overhead and improves stability

**Network Partitions & Convergence**:
1. **Deterministic Leader Selection**: Block creators chosen deterministically
2. **Majority Approval**: Subgroup needs >50% active nodes to approve fallback blocks (smaller partitions stall)
3. **Weight-Based Convergence**: 
   - Regular block weight = 1
   - Evidence block weight = number of support messages
   - All nodes converge on heaviest valid chain after partition heals
4. **Gap Recovery**: Nodes exchange missing blocks via `request_block` after reconnection

**Missing Packet Recovery**:
- Periodic scan for incomplete messages in RX buffer
- Identifies messages with most-recently-received packet older than timeout (default 60 seconds)
- Selects youngest incomplete message beyond threshold (prevents endless loops)
- Creates `request_block_part` message listing missing packet indices
- Other nodes respond with compact `add_block` reply containing only requested packets
- Improves efficiency: ~90% receive on first broadcast, remaining 10% catch up via targeted requests
- Allows aggressive relay tuning (fewer retransmissions) since gaps filled reactively

## Critical Constants (Compatibility-Breaking)

**Never change without coordinating network-wide upgrade:**
- `RADIO_PACKET_SIZE = 215` - Wire format size
- `RADIO_MAX_MESSAGE_SIZE = 2013` - Max payload before fragmentation

## Public API (RadioCommunicatorManager)

The library's public interface (`lib.rs`) provides four primary entry points:

1. **initialize()** - Initializes and starts radio communication node
   - Platform-specific implementations:
     - Embedded: uses static variables for queues
     - `std`: uses `Box::leak` for managed heap allocation
   - Parameters: `radio_config`, `spawner`, `radio_device`, `own_node_id`, `rng_seed`
   - Returns `Result<(), RadioInitError>`

2. **send_message()** - Broadcasts new radio message across network (non-blocking)
   - Places message into bounded `outgoing_message_queue`
   - **Drops message if queue full** (intentional: preserves predictable timing and deterministic behavior)
   - TX scheduler handles timing and packetization
   - Example: `manager.send_message(RadioMessage::request_echo_with(own_node_id))?`

3. **receive_message()** - Listens asynchronously for incoming radio events
   - Returns one of two outcomes:
     - `IncomingMessageItem::NewMessage(msg)` - New message from network
     - `IncomingMessageItem::CheckIfAlreadyHaveMessage(type, seq, checksum)` - Query to prevent redundant collection
   - Event-driven approach keeps CPU free while reacting immediately to network activity
   - Forms heart of node's message processing loop

4. **report_message_processing_status()** - Closes feedback loop between application and radio
   - Called after application processes message
   - Radio layer relays messages ONLY if application successfully processes them
   - Possible results:
     - `MessageProcessingResult::RequestedBlockFound(block)` - Node has requested data
     - `MessageProcessingResult::RequestedBlockNotFound(seq)` - Node missing data
     - `MessageProcessingResult::AlreadyHaveMessage(type, seq, checksum)` - Duplicate message
     - `MessageProcessingResult::NewBlockAdded` - Valid new data, prepare for relay
     - `MessageProcessingResult::RequestConnectionMatrixIntoLog` - Debug: dumps connection matrix to logs (requires `connection-matrix-logging` feature)

**Multiple Instance Support**: `RadioCommunicatorManager` struct allows multiple instances to coexist (essential for simulation use case).

## Radio Device Implementation Details

### Channel Activity Detection (CAD)

LoRa radios use CAD to sense whether channel is in use before transmission:
- Performs short, targeted scan of spectrum
- Samples small time window looking for LoRa preamble pattern
- If signal detected, node defers transmission until channel clear
- More efficient than continuous listening
- **Limitation**: Only detects preamble (header), not full payload
- Alternative approach: monitor RSSI and transmit only when signal below threshold

**MoonBlokz Approach**: Uses built-in LoRa CAD function for simplicity and reliability

### Hardware Behavior Workarounds

Two persistent issues encountered with RP2040 + SX1262:

1. **CAD Occasionally Sticks**: 
   - `LoRa.cad()` sometimes never returns, locking device in waiting state
   - Unclear if hardware quirk or driver issue
   - **Solution**: Added timeout mechanism (`CAD_TIMEOUT_MS = 1000`) - abort CAD if doesn't complete within fixed duration
   - See `rp_lora_sx1262.rs` run() method for timeout implementation using `select()`

2. **Missing CRC Validation**:
   - Hardware CRC support theoretically available but unreliable in practice
   - Kept receiving corrupted packets with hardware CRC enabled
   - **Solution**: Implemented software-based CRC-16-CCITT (2 extra bytes per packet)
   - Controlled via Cargo feature flag `soft-packet-crc` (auto-enabled by `radio-device-rp-lora-sx1262`)
   - Trade-off: slightly reduced max payload size, but far more dependable data link
   - CRC appended on TX, verified on RX with `checksum16()` function

### State Machine Flow

After initialization, radio device enters async loop waiting for two event types:
1. Packet arriving from network → immediately placed in `rx_packet_queue` for RX handler
2. Packet queued for transmission → check channel free via CAD → transmit if clear → resume listening

**Collision Avoidance**: If channel busy during CAD, system waits short interval before checking again

## Entity Types

Two core entity types:

1. **RadioMessage** - Complete logical message sent to/from network
   - Application-level abstraction
   - Variable size (can be fragmented)
   - What user sends/receives

2. **RadioPacket** - Physical packet traveling over radio link
   - Wire-level format
   - Fixed size (MTU limited to 215 bytes)
   - What radio transmits/receives

**Design Choice - Byte-Oriented API**:
- Store payloads as byte arrays, provide methods to interpret per message type
- Alternative (type-rich enum-based) would require nested enums (sequenced vs unsequenced → single vs multi-packet → concrete types)
- Byte-oriented approach simpler and avoids complicating every function
- Safety maintained through:
  - Typed constructors for every message kind (ensure valid encoding)
  - Field accessors return `Option<T>` when field not applicable to all types
  - Mutating methods return `Result<(), E>` to signal incompatibilities/validation errors

## Code Patterns

### Bitmask Operations on Connection Matrix

```rust
const QUALITY_MASK: u8 = 0b0011_1111;  // Lower 6 bits
const DIRTY_MASK: u8 = 0b1100_0000;    // Upper 2 bits
const DIRTY_SHIFT: u8 = 6;

// Extract quality: value & QUALITY_MASK
// Extract dirty: (value & DIRTY_MASK) >> DIRTY_SHIFT
// Set both: (dirty << DIRTY_SHIFT) | quality
```

### Relay Scoring Algorithm (`wait_pool.rs`)

**Core Data Structures**:
- `message_connections`: Array tracking message's link quality to every known node
- Initialized with sender's outgoing link qualities on first receipt
- Updated with MAX of all observed sender qualities as relays arrive
- Creates composite picture of which nodes likely received the message

**Link Quality Classes** (chain-configured thresholds):
- **Zero**: no connection (link_quality = 0)
- **Poor**: weak connection (0 < link_quality < poor_limit)
- **Fair**: normal connection (poor_limit ≤ link_quality < excellent_limit)
- **Excellent**: very strong connection (link_quality ≥ excellent_limit)

**Scoring Process**:
1. **Categorize Own Connections**: Classify node's links to all peers into 4 classes
2. **Categorize Message Connections**: Classify message's coverage using same thresholds
3. **Lookup Weights**: Use 4×4 `ScoringMatrix` to get weight for each (own_class, message_class) pair
4. **Calculate Aggregate Score**: Sum weights across all nodes
   - Only positive weights where own connection is in BETTER category than message connection
   - Minor fluctuations within same class don't influence decision
5. **Compare Against Threshold**: 
   - If score < `scoring_matrix.relay_score_limit` (0-15 scale): DROP from wait_pool
   - Message already has good coverage, relaying adds no value
6. **Determine Relay Position**:
   - Calculate scores for ALL nodes in network
   - Rank nodes in descending score order
   - Node's position in ranking determines relay order (position 0 = highest score)
7. **Calculate Delay**:
   ```
   delay = position × relay_position_delay + random_jitter
   ```
   - Higher scores relay SOONER (position 0 has shortest delay)
   - `relay_position_delay` is chain-configured constant
   - Random jitter prevents simultaneous transmissions

**Scoring Matrix Encoding** (5-byte compact format):
- 6 non-zero scoring values: 4 bits each (0-15 range)
- `poor_limit`: 6 bits (0-63)
- `excellent_limit`: 6 bits (0-63)  
- `relay_score_limit`: 4 bits (0-15)
- Total: 40 bits = 5 bytes exactly
- Distributed via genesis block (chain configuration)

**Dynamic Re-scoring**: As new relays arrive, nodes recalculate:
- Update `message_connections` with newly observed sender qualities
- Recalculate own relay score against updated coverage map
- May drop out of wait_pool if score falls below threshold
- Creates self-organizing propagation: strongest nodes relay first, redundant nodes withdraw

**Special Case - Query Replies**:
For `request_full_block` and `request_block_part` responses:
- Score based ONLY on link quality to requesting node
- Ensures replies sent by best-positioned node to reach requester

### Feature-Gated Code

```rust
#[cfg(feature = "std")]
// Standard library path (tests, simulator)

#[cfg(not(feature = "std"))]
// Embedded path (no_std, static allocation)
```

Radio device selected by mutually-exclusive features - see compile-time assertions in `lib.rs`.

### Logging Pattern

All log statements follow a consistent format with node ID prefix:
```rust
log::debug!("[{}] Message text here", self.own_node_id);
log::error!("[{}] Error message", self.own_node_id);
```
This enables filtering logs by node in multi-node simulations.

## Testing

```bash
# Run tests (requires std feature and a radio device)
cargo test --features std,radio-device-echo,memory-config-large
# Or with simulator:
cargo test --features std,radio-device-simulator,memory-config-large

# Tests live in #[cfg(all(test, feature = "std"))] blocks
# See relay_manager.rs tests module (after line 910) for examples
```

## Build Commands

```bash
# Development (simulator)
cargo build --features std,radio-device-simulator,memory-config-large

# Embedded (RP2040 + LoRa) - from examples/moonblokz-radio-embedded-test directory
cd examples/moonblokz-radio-embedded-test
cargo build --target thumbv6m-none-eabi --features radio-device-rp-lora-sx1262,memory-config-medium

# Run std example from workspace root
cargo run --example moonblokz-radio-std-test --features std,radio-device-echo,memory-config-large
```

## Common Pitfalls

1. **Matrix Semantics**: `connection_matrix[sender][receiver]` = link quality from sender→receiver
2. **Linear Search**: O(n) searches acceptable because CONNECTION_MATRIX_SIZE is small (hardware-limited)
3. **Dirty Counter**: Reset on echo response/result, NOT on echo request
4. **Relay Position**: Higher scores relay SOONER (position 0 = best coverage)
5. **Embassy Time**: All async timing uses `embassy_time::{Duration, Instant, Timer}`
6. **Feature Exclusivity**: Only ONE radio device and ONE memory config can be enabled (enforced by compile_error! in lib.rs)
7. **Task Pool Sizes**: `#[embassy_executor::task(pool_size = N)]` differs: embedded=1, std=100 (see radio_device_task)

## Documentation Style

- Module-level docs explain "why" and architecture
- Function docs focus on "what" parameters mean and edge cases
- See `relay_manager.rs` lines 1-40 for exemplary module documentation
- Include # Examples blocks showing actual usage patterns

## Key Files to Reference

- `src/lib.rs` - Public API, type definitions, and compile-time feature assertions
- `src/relay_manager.rs` - Core network logic and matrix management
- `src/wait_pool.rs` - Scoring algorithm implementation
- `src/tx_scheduler.rs` - Transmission timing and duty cycle management
- `src/rx_handler.rs` - Packet reassembly and message reconstruction
- `src/messages/radio_message.rs` - Message serialization format
- `src/radio_devices/rp_lora_sx1262.rs` - LoRa hardware abstraction with SX1262
- `src/radio_devices/simulator.rs` - Multi-node network simulation
- `README.md` - Algorithm descriptions and network behavior

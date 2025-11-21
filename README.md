# MoonBlokz Radio Library

MoonBlokz is a Decentralized Physical Infrastructure Network (DePIN) system optimized for microcontrollers and radio communication, operating independently of any central infrastructure. [https://www.moonblokz.com](https://www.moonblokz.com).

This repository contains the `no_std` Rust radio mesh networking library for MoonBlokz blockchain's physical layer, implementing autonomous broadcast-based communication over LoRa.

[![Watch the video](https://img.youtube.com/vi/_-OoIMKLwGY/hqdefault.jpg)](
  https://www.youtube.com/watch?v=_-OoIMKLwGY "Watch on YouTube"
)

## Documentation

For detailed information about the algorithms and implementation:

- [MoonBlokz Series part VII/2 — Mesh Radio Algorithm](https://medium.com/moonblokz/moonblokz-series-part-vii-2-mesh-radio-algorithm-3650af3711f3) - Comprehensive explanation of the mesh networking algorithms, connection matrix, echo protocol, and relay scoring
- [MoonBlokz series part VII/3 — Inside the Radio Module](https://medium.com/@peter.sallai/moonblokz-series-part-vii-3-inside-the-radio-module-d92545624d2b) - Deep dive into the implementation details, async architecture, memory management, and hardware integration

## Core Philosophy

- **No acknowledgments, no handshakes, no centralized coordination** - The network self-organizes through periodic echo probing and intelligent relay decisions
- **Deterministic memory management** - All memory statically allocated at compile time, no heap usage
- **Asynchronous architecture** - Built on Embassy framework for efficient async/await on embedded systems
- **Fire-and-forget transmission** - Messages broadcast without acknowledgment; reliability comes from redundant relay paths
- **Reactive fault tolerance** - Network detects and recovers from missing data through blockchain structure feedback
- **Stateful relaying** - Nodes relay only if they don't already have the data, preventing loops and broadcast storms

## Key Design Constraints

- LoRa duty cycle limits (typically 1% airtime or less)
- Radio can only perform ONE action at a time (transmit XOR receive XOR CAD)
- 3D spatial operation (not limited to 2D topologies)
- Must work on RP2040 (Cortex-M0+, 264KB RAM, 2MB flash)

## Architecture

### Three-Task Async System

The library runs three independent async tasks connected via bounded queues:

1. **TX Scheduler** - Entry point for all outgoing data (application + relay logic)
   - Enforces radio timing parameters and packetizes large messages
   - Queue acts as first protective backstop - drops new messages when overloaded
   - Maintains predefined delay between messages, pauses during other node's multi-packet transmissions
   - Controls duty cycle via `delay_between_tx_messages` parameter

2. **Radio Device** - Low-level manager for physical radio
   - Constrained to ONE action at a time: transmit XOR receive XOR CAD
   - Default mode: always listening for packets
   - When packet ready: stops listening → CAD check → transmit if clear → resume listening
   - Implementations: `rp_lora_sx1262.rs` (production), `simulator.rs` (testing), `echo.rs` (unit tests)

3. **RX Handler** - Handles all inbound traffic and reassembles multi-packet messages
   - Communicates with application layer via `incoming_message_queue`
   - Triggers relaying logic via RelayManager
   - Queue acts as final buffer - drops newly received messages if application is slow

**Failure Isolation**: Architecture ensures failures (app bottleneck, congested channel) cause predictable message drops, NOT memory overflow or crashes.

### Five Core Components

1. **RadioDevice** (`src/radio_devices/`) - Hardware abstraction with state machine
2. **TX Scheduler** (`src/tx_scheduler.rs`) - Outgoing message coordination and timing
3. **RX Handler** (`src/rx_handler.rs`) - Incoming message assembly and deduplication
4. **RelayManager** (`src/relay_manager.rs`) - **Most Complex Module**
   - Maintains NxN connection quality matrix tracking all node pairs
   - Implements echo protocol (request_echo → echo → echo_result)
   - Makes relay decisions using scoring algorithm
   - Processes application responses to determine relaying eligibility
5. **WaitPool** (`src/wait_pool.rs`) - Temporarily stores messages before relaying with calculated delay

## Algorithm Details

## Algorithm Details

### Network Topology

Nodes communicate using only broadcast radio signals. The majority of nodes have fixed positions and operate continuously. Some nodes may be mobile, but they are not the majority. The network is ad-hoc with no central authority to create structured partitions.

### Blockchain Messages

The nodes build a blockchain and communicate using seven message types:

1. **add_block** - Propagates a newly created block to the network (multi-packet capable)
2. **request_full_block** - Requests a specific block by sequence number (and optionally parent hash). Used when receiving a block whose parent is missing, or when no network activity is detected for a long time.
3. **request_block_part** - Requests missing packets from an incomplete `add_block` message
4. **add_transaction** - Propagates a new transaction to the network (multi-packet capable)
5. **request_transaction_part** - Requests missing packets from an incomplete `add_transaction` message
6. **request_new_mempool_item** - Propagates hashes of top mempool transactions. Nodes use this periodically (low frequency) if they don't have enough transactions to create a full block. If a node knows other top transactions, it replies with `add_transaction`. If multiple transactions are missing, the node randomly chooses one.
7. **support** - The blockchain uses deterministic algorithm to choose the creator of the next block. If that node cannot create the block, another node can take its place, but the majority of the network must approve it with support messages.

**Message Sizing**:

- Only `add_block` and `add_transaction` support multi-packet fragmentation
- All other messages fit in ~100 bytes, compatible with any radio hardware
- All messages have a standard header including sender node identifier and payload checksum

### Echo Protocol (Network Discovery)

Nodes use periodic `request_echo` and `echo` messages to map the network topology:

1. **Probe Scheduling** - Node broadcasts `request_echo` message
   - Frequency adapts based on neighbor count (more neighbors = longer interval to conserve bandwidth)
   - Before broadcast: increments dirty counter for ALL connections (aging mechanism)
   - Network expected to be largely static

2. **Neighbor Responses** - Nodes send `echo` message with link quality
   - Includes signal quality (RSSI/SNR) of received `request_echo`
   - Random back-off delay prevents collision
   - Queued via dedicated `echo_responses_wait_pool`

3. **Aggregation & Broadcast** - Original requester sends `echo_result`
   - Waits up to `echo_gathering_timeout` for responses
   - Compiles bidirectional link qualities into single message
   - Orders responders by link quality

**Link Quality Calculation** (0-63 scale):

```text
link_quality = 0.7 × SNR(normalized) + 0.3 × RSSI(normalized)
```

SNR weighted higher (70%) as better predictor of packet delivery.

### Connection Matrix

Based on echo responses, all nodes build a connection matrix storing signal strength between all neighbors:

- **Structure**: NxN matrix where rows=senders, columns=receivers
- **Cell Format**: Each u8 cell contains:
  - Lower 6 bits: link quality (0-63)
  - Upper 2 bits: dirty counter (0-3) for aging
- **Capacity**: Fixed maximum size (10/30/100 nodes depending on memory config)
- **Eviction**: If full, weakest connection is evicted
- **Asymmetry**: Links don't need to be symmetric (Q[a,b] ≠ Q[b,a])

**Dirty Counter Aging**:

- Starts at 0 for fresh connections
- Increments on each `request_echo` from that node
- Resets to 0 on echo response/result
- At 3 (MAX_DIRTY_COUNT): connection zeroed and considered lost
- Connections survive one missed echo cycle but are zeroed after two consecutive misses

**Continuous Updates**: Matrix also updated from ANY received radio packet, keeping quality values current.

### Adaptive Relaying Algorithm

When a node receives a message, it uses an adaptive algorithm to decide whether to answer/relay it:

**Echo Handling**:

- `request_echo` → Always answered (after slight random delay), never relayed
- `echo` / `echo_result` → Never relayed; used only for local neighbor discovery

**Block Requests**:

When `request_full_block` arrives:

- **If node has the block**: Ranks all neighbors by connection strength to requester, waits based on position in list. Nodes with stronger links wait longer, giving weaker (closer/better-placed) nodes chance to respond first. If no reply heard within waiting window, sends the block.
- **If node doesn't have block**: Waits for similar delay. If no one else responds or relays, relays the `request_full_block` message.

**Block Propagation**:

When receiving `add_block` message the node doesn't have:

- Identifies which neighbors likely missed the block (low-quality links)
- Checks which of its own links can best reach those nodes
- After connection-weighted delay, if relaying would still improve coverage, rebroadcasts message
- **Critical**: Nodes do NOT relay if they already have the block (stateful relaying prevents loops)

**Relay Scoring Algorithm**:

1. Track `message_connections`: array of message's link quality to every known node
   - Initialized with sender's outgoing link qualities on first receipt
   - Updated with MAX of all observed sender qualities as relays arrive
2. Categorize connections into 4 classes: Zero/Poor/Fair/Excellent (using chain-configured thresholds)
3. Use 4×4 `ScoringMatrix` to get weight for each (own_class, message_class) pair
4. Calculate aggregate score summing weights across all nodes
5. If score < `scoring_matrix.relay_score_limit` (0-15 scale): DROP from wait_pool
6. Determine relay position by ranking all nodes by score (descending)
7. Calculate delay: `position × relay_position_delay + random_jitter`
   - Higher scores relay SOONER (position 0 has shortest delay)

**Dynamic Re-scoring**: As new relays arrive, nodes recalculate scores and may drop out if score falls below threshold. Creates self-organizing propagation: strongest nodes relay first, redundant nodes withdraw.

**Other Message Types**:

- `add_transaction` → Same logic as `add_block`
- `request_new_mempool_item` → If node has missing transactions, randomly chooses one, calculates waiting time. If no other nodes send selected transaction during wait, sends `add_transaction` when time expires.
- `request_transaction_part` → Same logic as `request_block_part`
- `support` → Same logic as `add_block` and `add_transaction`

**Collision Avoidance**:

- All waiting times include random jitter
- Nodes use Channel Activity Detection (CAD) to delay transmission if channel busy
- Nodes delay transmissions if another node started sending multi-packet message

**Mobile vs Fixed Nodes**:

- Only fixed nodes added to connection_matrix
- Fixed node receiving from mobile node: calculates waiting times based on coverage to other neighbor nodes
- Mobile node receiving message: uses random function with constant average for waiting time (typically longer than fixed nodes)

### Network Resilience & Fault Tolerance

**No TTL Required**:

There is no TTL logic required for any messages. A message (`support`, `add_block`, `add_transaction`) is only relayed if the node does not already have it. If a message is already distributed in the network, relaying stops quickly naturally (stateful relaying prevents broadcast storms).

**Reactive Fault Tolerance**:

The network does not use any response messages for successful transmissions. If a node does not receive an `add_block` message, it will notice it one block later because the parent of the next block will be missing, and the node will send a `request_full_block` message to get it. If no reply arrives for a given time, the node will resend the request.

**Delay & Disorder Tolerance**:

The blockchain network is delay-tolerant by design, and nodes can handle out-of-order messages. Child blocks can be cached temporarily until parent retrieved. There is no need to prioritize between message types.

**Missing Packet Recovery**:

- Periodic scan for incomplete messages in RX buffer
- Identifies messages with most-recently-received packet older than timeout (default 60 seconds)
- Selects youngest incomplete message beyond threshold (prevents endless loops)
- Creates `request_block_part` message listing missing packet indices
- Other nodes respond with compact `add_block` reply containing only requested packets
- Improves efficiency: ~90% receive on first broadcast, remaining 10% catch up via targeted requests

**Network Partitions & Convergence**:

If nodes are separated into groups, divergent chains can be formed. The consensus algorithm limits this:

1. **Deterministic Leader Selection**: Sooner or later, all sub-groups reach a state where the deterministic chain creator selection logic selects a node not in that subgroup
2. **Majority Approval**: In that case, approval process begins and will likely only succeed in sub-group with more than half of active nodes
3. **Weight-Based Convergence**: If two sub-groups connect, all nodes query all block variants (through `request_full_block` logic), and nodes always select the heaviest chain variants:
   - Regular block weight = 1
   - Evidence block weight = number of support messages
4. **Gap Recovery**: Nodes exchange missing blocks after reconnection

### Configuration Parameters

Nodes utilize multiple parameters for fine-tuning the algorithm:

- **Compile-time parameters**: Size of connection matrix, memory configuration (small/medium/large)
- **Chain-configured parameters**: Most parameters come from the chain's configuration and are distributed by the genesis block, including:
  - Echo intervals and timeouts
  - Relay scoring matrix (5-byte compact format)
  - Link quality thresholds (poor_limit, excellent_limit)
  - Relay position delay
  - Duty cycle timing

### Memory Management

**Three compile-time configurations** (mutually exclusive features):

- `memory-config-small` - CONNECTION_MATRIX_SIZE=10, ~25KB RAM
- `memory-config-medium` - CONNECTION_MATRIX_SIZE=30, ~60KB RAM  
- `memory-config-large` - CONNECTION_MATRIX_SIZE=100, ~120KB RAM (default)

**Deterministic Allocation**:

- All memory statically allocated at compile time
- Fixed-size buffers for all radio messages/packets
- Bounded data structures (queues, connection matrix, wait pool)
- Embassy framework uses static memory for task state
- No heap usage eliminates runtime allocation errors

## Public API

The library's public interface provides four primary entry points through `RadioCommunicatorManager`:

1. **initialize()** - Initializes and starts radio communication node
2. **send_message()** - Broadcasts new radio message across network (non-blocking, drops if queue full)
3. **receive_message()** - Listens asynchronously for incoming radio events
4. **report_message_processing_status()** - Closes feedback loop between application and radio layer

Multiple instances supported for simulation use cases.

## Features

- `std` - Standard library support (required for tests)
- `embedded` - Embedded system support with static allocation
- `radio-device-echo` - Simple echo device for testing
- `radio-device-rp-lora-sx1262` - LoRa SX1262 hardware support (default)
- `radio-device-simulator` - Network simulator for testing
- `soft-packet-crc` - Software CRC for packet validation

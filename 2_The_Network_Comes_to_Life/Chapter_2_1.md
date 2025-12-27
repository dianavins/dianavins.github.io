---
title: "2.1 Visualizing the Network's Signal Processing in Hardware"
parent: "Chapter 2: The Network Comes to Life"
nav_order: 1
---

## 2.1 Visualizing the Network's Signal Processing in Hardware

### Overview: Three Phases of Execution

Every timestep of network execution goes through three distinct phases. Think of them as an assembly line for processing spikes:

```
PHASE 0: INPUT RECEPTION (Host → FPGA)
┌──────────────────────────────────────────────────────────────┐
│ Host sends input spikes                                      │
│ Written to: BRAM (axon spike masks)                          │
│ Time: ~1-10 microseconds (depends on PCIe transfer)          │
│ Status: Network is IDLE, waiting for execute command         │
└──────────────────────────────────────────────────────────────┘
                           │
                           │ execute() command sent
                           ▼
PHASE 1: EXTERNAL EVENT PROCESSING (Axon → Synapse Lookup)
┌──────────────────────────────────────────────────────────────┐
│ external_events_processor.v reads BRAM                       │
│ For each active axon:                                        │
│   1. Read axon pointer from HBM (where are synapses?)        │
│   2. Read synapse data from HBM (targets + weights)          │
│ Output: Synapses distributed to neuron groups               │
│ Time: ~1-5 microseconds (depends on # of active axons)       │
└──────────────────────────────────────────────────────────────┘
                           │
                           ▼
PHASE 2: POINTER DISTRIBUTION (Synapse Routing)
┌──────────────────────────────────────────────────────────────┐
│ pointer_fifo_controller.v receives synapse data from HBM     │
│ For each synapse:                                            │
│   1. Decode target neuron address                           │
│   2. Determine which neuron group (0-15)                     │
│   3. Push to that group's pointer FIFO                       │
│ Output: Each neuron group has queue of pending synapses     │
│ Time: ~100 nanoseconds (fast routing logic)                  │
└──────────────────────────────────────────────────────────────┘
                           │
                           ▼
PHASE 3: INTERNAL EVENT PROCESSING (Neuron State Updates)
┌──────────────────────────────────────────────────────────────┐
│ internal_events_processor.v updates neurons @ 450 MHz        │
│ For each synapse in FIFO (16 groups parallel):              │
│   1. Read neuron state from URAM (current V)                 │
│   2. Add synaptic weight: V_new = V_old + weight             │
│   3. Apply neuron model: leak, threshold check               │
│   4. If V >= threshold: SPIKE, reset V=0                     │
│   5. Write V_new back to URAM                                │
│   6. If spike: send to spike_fifo_controller                 │
│ Output: Updated neuron states, spike events                  │
│ Time: ~1-10 microseconds (depends on # of synapses)          │
└──────────────────────────────────────────────────────────────┘
                           │
                           ▼
SPIKE OUTPUT (FPGA → Host)
┌──────────────────────────────────────────────────────────────┐
│ spike_fifo_controller.v collects spikes                      │
│ Packages into 512-bit packets                                │
│ Sends via PCIe to host                                       │
│ Host reads via flush_spikes()                                │
│ Time: ~1-5 microseconds                                      │
└──────────────────────────────────────────────────────────────┘
```

**Total time per timestep:** ~5-30 microseconds depending on network activity
- Small networks with few spikes: ~5 µs
- Large networks with many spikes: ~30 µs
- Target: 1 millisecond per timestep → can run 30-200 timesteps in real-time

---

### The Data Journey: What Moves Where

Let's visualize where data lives at each phase, using our example where **axons a0, a1, a2 fire**.

#### Before Phase 0: Network is Idle

```
┌─────────────────────────────────────────────────────────────┐
│                        HOST                                 │
│  Python code:                                               │
│    network.step(['a0', 'a1', 'a2'])                         │
│                                                             │
│  In memory: List of axon names to send                     │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                        FPGA                                 │
│                                                             │
│  BRAM: Empty (no input pattern)                            │
│  ┌───────────────────────────────────────────────────────┐ │
│  │ Row 0: [0x0000...0000] All zeros                      │ │
│  │ Row 1: [0x0000...0000]                                │ │
│  │ ...                                                    │ │
│  └───────────────────────────────────────────────────────┘ │
│                                                             │
│  HBM: Contains network structure (from initialization)     │
│  ┌───────────────────────────────────────────────────────┐ │
│  │ Axon pointers, neuron pointers, synapses             │ │
│  │ (Unchanged, frozen since initialization)              │ │
│  └───────────────────────────────────────────────────────┘ │
│                                                             │
│  URAM: All neurons at V=0 (just cleared)                   │
│  ┌───────────────────────────────────────────────────────┐ │
│  │ h0: V=0, h1: V=0, h2: V=0, h3: V=0, h4: V=0          │ │
│  │ o0: V=0, o1: V=0, o2: V=0, o3: V=0, o4: V=0          │ │
│  └───────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

#### After Phase 0: Inputs Written to BRAM

```
┌─────────────────────────────────────────────────────────────┐
│                        FPGA                                 │
│                                                             │
│  BRAM: Input pattern written                               │
│  ┌───────────────────────────────────────────────────────┐ │
│  │ Row 0: [0x0000...0007]                                │ │
│  │         └─ Bits [2:0] = 0b111 (a0, a1, a2 active)    │ │
│  │                                                        │ │
│  │ (If network had more axons, more rows would be used)  │ │
│  └───────────────────────────────────────────────────────┘ │
│                                                             │
│  HBM: Unchanged (still contains network structure)         │
│  URAM: Unchanged (still all zeros)                         │
│                                                             │
│  Status: Waiting for execute() command                     │
└─────────────────────────────────────────────────────────────┘
```

**What's in BRAM Row 0:**
- BRAM row format: 256 bits = 16 neuron groups × 16 bits per group
- Each 16-bit mask indicates which neurons in that group should receive this axon
- Our network has all neurons in group 0, so:
  ```
  Bits [255:16]: All zeros (groups 1-15 unused)
  Bits [15:0]: 0x0007 = 0b0000000000000111
    Bit 0 = 1: Axon a0 active
    Bit 1 = 1: Axon a1 active
    Bit 2 = 1: Axon a2 active
    Bits 3-15 = 0: Axons a3, a4, ... inactive
  ```

Actually, wait. Let me reconsider the BRAM format. Looking at the gopa README and hardware_map.md:

BRAM stores axon spike masks. Each row corresponds to one timestep's input pattern. But actually, the format is:
- Each BRAM row (256 bits) = 16 masks × 16 bits
- Each 16-bit mask is for one neuron group
- The mask indicates which neurons in that group should receive the spike

But that's for routing synapses to neuron groups, not for marking which axons are active.

Let me re-read... Actually, looking at input_data_handler and external_events_processor descriptions:
- BRAM stores axon spike masks
- Each row contains data for axons
- The external_events_processor reads BRAM to find active axons

I think the format is simpler:
- Each BRAM address corresponds to an axon (or small group of axons)
- The data at that address indicates if the axon is active

Actually, from fpga_controller.py input_user(), I see:
```python
one_hot_bin = ["0"] * 256  # 256-bit field
for axon in inputSegment:
    one_hot_bin[axon%256] = "1"
```

So it's a 256-bit one-hot encoding of which axons are active in a 256-axon chunk. This gets written to BRAM.

Let me revise:

#### After Phase 0: Inputs Written to BRAM

```
┌─────────────────────────────────────────────────────────────┐
│                        FPGA BRAM                            │
│                                                             │
│  Row 0 (bytes 0-31): 256-bit one-hot encoding of active axons │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  Bit 0 = 1: Axon a0 is firing                          │ │
│  │  Bit 1 = 1: Axon a1 is firing                          │ │
│  │  Bit 2 = 1: Axon a2 is firing                          │ │
│  │  Bit 3 = 0: Axon a3 is silent                          │ │
│  │  Bit 4 = 0: Axon a4 is silent                          │ │
│  │  Bits 5-255 = 0: No other axons (network only has 5)   │ │
│  │                                                          │ │
│  │  As hex bytes (little-endian):                         │ │
│  │  [0] = 0x07 = 0b00000111 (bits 0,1,2 set)              │ │
│  │  [1]-[31] = 0x00 (all zeros)                           │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                             │
│  Rows 1-32767: Unused (our network fits in row 0)          │
└─────────────────────────────────────────────────────────────┘
```

#### During Phase 1: Reading HBM for Synapse Data

```
┌─────────────────────────────────────────────────────────────┐
│                   PHASE 1 DATA FLOW                         │
│                                                             │
│  external_events_processor reads BRAM:                      │
│    "Bits 0, 1, 2 are set → axons a0, a1, a2 are active"   │
│                                                             │
│  For each active axon, fetch from HBM:                     │
│                                                             │
│  ┌─ Axon a0 ────────────────────────────────────────────┐  │
│  │ 1. Read axon pointer from HBM[0x0000]:              │  │
│  │    Pointer = {length: 1 row, start: 0x8000}         │  │
│  │ 2. Read synapse row from HBM[0x8000]:               │  │
│  │    [a0→h0, wt=1000]                                 │  │
│  │    [a0→h1, wt=1000]                                 │  │
│  │    [a0→h2, wt=1000]                                 │  │
│  │    [a0→h3, wt=1000]                                 │  │
│  │    [a0→h4, wt=1000]                                 │  │
│  │    [unused] [unused] [unused]                       │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌─ Axon a1 ────────────────────────────────────────────┐  │
│  │ 1. Read axon pointer from HBM[0x0000]:              │  │
│  │    (Actually same row, offset 4 bytes)              │  │
│  │    Pointer = {length: 1 row, start: 0x8001}         │  │
│  │ 2. Read synapse row from HBM[0x8001]:               │  │
│  │    [a1→h0, wt=1000], [a1→h1, wt=1000], ...         │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌─ Axon a2 ────────────────────────────────────────────┐  │
│  │ Similar: reads HBM[0x8002]                          │  │
│  │    [a2→h0, wt=1000], [a2→h1, wt=1000], ...         │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                             │
│  Total synapses fetched: 3 axons × 5 synapses = 15 synapses│
└─────────────────────────────────────────────────────────────┘
```

#### During Phase 2: Distributing Synapses to Neuron Groups

```
┌─────────────────────────────────────────────────────────────┐
│              PHASE 2: POINTER DISTRIBUTION                  │
│                                                             │
│  pointer_fifo_controller receives 15 synapses from HBM      │
│  For each synapse, decode target and route to FIFO:        │
│                                                             │
│  Synapse: [target=h0 (neuron 0), weight=1000]              │
│    → Neuron 0 is in group 0 (0 ÷ 8192 = 0)                 │
│    → Push to pointer_fifo[0]                               │
│                                                             │
│  Synapse: [target=h1 (neuron 1), weight=1000]              │
│    → Neuron 1 is in group 0                                │
│    → Push to pointer_fifo[0]                               │
│                                                             │
│  ... (all 15 synapses go to group 0 in our small network)  │
│                                                             │
│  Pointer FIFO 0 now contains:                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Entry 0: {target: h0 (local addr 0), weight: 1000} │   │
│  │ Entry 1: {target: h1 (local addr 1), weight: 1000} │   │
│  │ Entry 2: {target: h2 (local addr 2), weight: 1000} │   │
│  │ Entry 3: {target: h3 (local addr 3), weight: 1000} │   │
│  │ Entry 4: {target: h4 (local addr 4), weight: 1000} │   │
│  │ Entry 5: {target: h0, weight: 1000}                │   │
│  │ Entry 6: {target: h1, weight: 1000}                │   │
│  │ Entry 7: {target: h2, weight: 1000}                │   │
│  │ Entry 8: {target: h3, weight: 1000}                │   │
│  │ Entry 9: {target: h4, weight: 1000}                │   │
│  │ Entry 10: {target: h0, weight: 1000}               │   │
│  │ Entry 11: {target: h1, weight: 1000}               │   │
│  │ Entry 12: {target: h2, weight: 1000}               │   │
│  │ Entry 13: {target: h3, weight: 1000}               │   │
│  │ Entry 14: {target: h4, weight: 1000}               │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  Pointer FIFOs 1-15: Empty (no neurons in those groups)    │
└─────────────────────────────────────────────────────────────┘
```

Notice: **Each hidden neuron (h0-h4) appears 3 times** in the FIFO because all 3 axons connect to all 5 hidden neurons!

#### During Phase 3: Neuron State Updates

Now the `internal_events_processor` processes these synapses and updates URAM:

```
┌─────────────────────────────────────────────────────────────┐
│          PHASE 3: NEURON STATE UPDATES                      │
│          (Processing neuron group 0 @ 450 MHz)              │
│                                                             │
│  Cycle 0: Read FIFO Entry 0: {h0, wt=1000}                 │
│    Read URAM[h0]: V_old = 0                                │
│    Compute: V_new = 0 + 1000 = 1000                        │
│    Check threshold: 1000 < 2000 → no spike                 │
│    Write URAM[h0]: V = 1000                                │
│                                                             │
│  Cycle 5: Read FIFO Entry 1: {h1, wt=1000}                 │
│    Read URAM[h1]: V_old = 0                                │
│    Compute: V_new = 0 + 1000 = 1000                        │
│    Write URAM[h1]: V = 1000                                │
│                                                             │
│  ... (similar for h2, h3, h4)                              │
│                                                             │
│  Cycle 25: Read FIFO Entry 5: {h0, wt=1000}  ← 2nd input! │
│    Read URAM[h0]: V_old = 1000                             │
│    Compute: V_new = 1000 + 1000 = 2000                     │
│    Check threshold: 2000 >= 2000 → SPIKE!                  │
│    Write URAM[h0]: V = 0 (reset)                           │
│    Send spike: neuron_id = h0 → spike_fifo                 │
│                                                             │
│  Cycle 30: Read FIFO Entry 6: {h1, wt=1000}  ← 2nd input! │
│    Read URAM[h1]: V_old = 1000                             │
│    Compute: V_new = 1000 + 1000 = 2000                     │
│    Check threshold: 2000 >= 2000 → SPIKE!                  │
│    Write URAM[h1]: V = 0 (reset)                           │
│    Send spike: neuron_id = h1 → spike_fifo                 │
│                                                             │
│  ... (h2, h3, h4 also spike on their 2nd input)            │
│                                                             │
│  Cycle 50: Read FIFO Entry 10: {h0, wt=1000}  ← 3rd input!│
│    Read URAM[h0]: V_old = 0 (was just reset)              │
│    Compute: V_new = 0 + 1000 = 1000                        │
│    Check threshold: 1000 < 2000 → no spike                 │
│    Write URAM[h0]: V = 1000                                │
│                                                             │
│  ... (all neurons accumulate 3rd input but don't spike)   │
│                                                             │
│  Final URAM state after Phase 3:                           │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ h0: V = 1000  (spiked at V=2000, reset, got 1 more) │   │
│  │ h1: V = 1000                                        │   │
│  │ h2: V = 1000                                        │   │
│  │ h3: V = 1000                                        │   │
│  │ h4: V = 1000                                        │   │
│  │ o0: V = 0     (haven't received inputs yet)         │   │
│  │ ... (outputs will spike in next phase)             │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

**Key insight:** Each neuron receives inputs sequentially from the FIFO. When h0 gets its first input (V=1000), it doesn't spike. When it gets the second input (V=2000), it spikes and resets. The third input arrives after the reset, so V=1000 again.

#### Hidden Neuron Spikes Trigger Phase 1 Again

Here's something crucial: **When hidden neurons spike, they become inputs for the next phase!**

```
┌─────────────────────────────────────────────────────────────┐
│         RECURRENT PROCESSING (Neuron Spikes)                │
│                                                             │
│  Hidden neurons spiked: h0, h1, h2, h3, h4                 │
│  These spikes are routed back to external_events_processor │
│                                                             │
│  Phase 1 (again): For each spiking neuron                  │
│    Read neuron pointer from HBM                            │
│    Read synapse data                                       │
│                                                             │
│  Neuron h0 spikes → fetch h0's output synapses from HBM:   │
│    [h0→o0, wt=1000]                                        │
│    [h0→o1, wt=1000]                                        │
│    [h0→o2, wt=1000]                                        │
│    [h0→o3, wt=1000]                                        │
│    [h0→o4, wt=1000]                                        │
│                                                             │
│  Similarly for h1, h2, h3, h4                              │
│  Total: 5 neurons × 5 outputs = 25 new synapses            │
│                                                             │
│  Phase 2 (again): Distribute to neuron groups              │
│    All go to group 0 (output neurons o0-o4)               │
│                                                             │
│  Phase 3 (again): Update output neurons                    │
│    Each output neuron receives 5 inputs × 1000 = 5000     │
│    o0: V = 0 + 5000 = 5000 >= 2000 → SPIKE!               │
│    o1: V = 0 + 5000 = 5000 >= 2000 → SPIKE!               │
│    ... (all output neurons spike)                          │
│                                                             │
│  Output neuron spikes are marked as "send to host"         │
│  (OpCode = 100 in their synapse entries)                   │
└─────────────────────────────────────────────────────────────┘
```

#### Final State After One Complete Timestep

```
┌─────────────────────────────────────────────────────────────┐
│                  FINAL STATE (Timestep 0)                   │
│                                                             │
│  URAM (Neuron States):                                     │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ h0: V = 1000  (accumulated 3 inputs, spiked once)   │   │
│  │ h1: V = 1000                                        │   │
│  │ h2: V = 1000                                        │   │
│  │ h3: V = 1000                                        │   │
│  │ h4: V = 1000                                        │   │
│  │ o0: V = 0     (accumulated 5000, spiked, reset)     │   │
│  │ o1: V = 0                                           │   │
│  │ o2: V = 0                                           │   │
│  │ o3: V = 0                                           │   │
│  │ o4: V = 0                                           │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  Spike Output (sent to host):                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Output spikes: o0, o1, o2, o3, o4                   │   │
│  │ (All 5 output neurons spiked)                       │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  Hidden neuron spikes (h0-h4) were internal—not reported   │
│  to host unless explicitly requested                       │
└─────────────────────────────────────────────────────────────┘
```

---

### Timing: How Long Does This Take?

Let's estimate the time for one complete timestep:

```
Phase 0: Input Writing
├─ PCIe transfer: 256 bits ÷ 14 GB/s ≈ 2 nanoseconds
├─ BRAM write: 1 clock cycle @ 225 MHz = 4.4 ns
└─ Total: ~10 nanoseconds (negligible)

Phase 1: External Events (First Pass - Axon Spikes)
├─ BRAM read: 3 cycles × 4.4ns = 13.2 ns
├─ HBM reads: 3 axons × (1 pointer + 1 synapse row)
│   ├─ 6 HBM reads × 150ns latency = 900 ns
│   └─ (Can overlap with pipelining: ~300-500 ns actual)
└─ Total: ~500 nanoseconds

Phase 2: Pointer Distribution
├─ Parse 15 synapses: 15 × 4.4ns = 66 ns
├─ FIFO writes: 15 × 4.4ns = 66 ns
└─ Total: ~130 nanoseconds

Phase 3: Neuron Updates (First Pass - Hidden Neurons)
├─ Process 15 synapses @ 450 MHz
│   Each synapse: 5 cycles (read FIFO, read URAM, compute,
│                           write URAM, check spike)
│   ├─ 15 synapses × 5 cycles = 75 cycles
│   └─ 75 cycles ÷ 450 MHz = 167 ns
└─ Total: ~170 nanoseconds

Phase 1-3: Second Pass (Hidden Neuron Spikes → Outputs)
├─ Phase 1: 5 neurons × 2 HBM reads = 10 reads × 150ns
│            (Pipelined: ~500ns)
├─ Phase 2: 25 synapses × 4.4ns = 110 ns
├─ Phase 3: 25 synapses × 5 cycles ÷ 450 MHz = 278 ns
└─ Total: ~890 nanoseconds

Spike Output
├─ Packet assembly: ~50 ns
├─ PCIe transfer: 512 bits ÷ 14 GB/s ≈ 3.7 ns
└─ Total: ~60 nanoseconds

TOTAL TIME PER TIMESTEP: ~2 microseconds
```

**2 microseconds per timestep!** This means the hardware can theoretically run **500,000 timesteps per second**. In practice, you're limited by PCIe overhead and host processing, but you can easily achieve 1000 timesteps/second (1 millisecond/timestep) for real-time operation.

---

### Key Concepts: Understanding the Hardware Mechanisms

Before we dive into code, let's clarify some hardware concepts that are crucial for understanding execution:

#### 1. Pipelines and State Machines

**State Machine:** A circuit that steps through defined states based on conditions.

```
Example: external_events_processor state machine

IDLE state:
  ├─ If execute command received: → SCAN_BRAM
  └─ Else: stay in IDLE

SCAN_BRAM state:
  ├─ Read next BRAM row
  ├─ Wait 3 cycles for BRAM data
  └─ → PARSE_MASK

PARSE_MASK state:
  ├─ Check each bit in mask
  ├─ For each '1' bit: request HBM read
  └─ → WAIT_HBM

WAIT_HBM state:
  ├─ Wait for HBM data ready
  └─ → PROCESS_SYNAPSES

... and so on
```

Each state executes for one or more clock cycles. The state machine "remembers" where it is using flip-flops that store the current state.

**Pipeline:** Overlapping execution stages to increase throughput.

```
Example: HBM Read Pipeline (3 stages)

Cycle 0: Stage 1: Send address for axon 0
Cycle 1: Stage 1: Send address for axon 1
         Stage 2: Wait for axon 0 data
Cycle 2: Stage 1: Send address for axon 2
         Stage 2: Wait for axon 1 data
         Stage 3: Receive axon 0 data, process
Cycle 3: Stage 2: Wait for axon 2 data
         Stage 3: Receive axon 1 data, process
Cycle 4: Stage 3: Receive axon 2 data, process

Without pipelining: 3 reads × 100ns = 300ns
With pipelining: 100ns (first read) + 2×10ns (overlap) = 120ns
```

#### 2. FIFOs: First-In-First-Out Queues

A FIFO is a buffer that stores data temporarily. Think of it as a line at a store:
- **Write side:** Customers entering the line (producers add data)
- **Read side:** Cashier serving customers (consumers take data)
- **FIFO full:** Line is full, new customers must wait
- **FIFO empty:** No customers, cashier waits

**Key signals:**
```verilog
wr_en:     Write enable (add data to FIFO)
din:       Data input
full:      FIFO is full (can't write)

rd_en:     Read enable (take data from FIFO)
dout:      Data output
empty:     FIFO is empty (can't read)
```

**Our usage:**
- **Input FIFO:** Stores commands from PCIe
- **Pointer FIFOs (16 of them):** Store synapses waiting to be processed by each neuron group
- **Spike FIFOs (8 of them):** Store spike events from neurons

#### 3. Read-Modify-Write Hazards

**Problem:** What if the same neuron gets two inputs very close together?

```
Cycle 0: Read h0 from URAM: V=0
Cycle 1: Compute: V_new = 0 + 1000 = 1000
Cycle 2: Write h0 to URAM: V=1000

But wait! What if another synapse for h0 is processed starting at Cycle 1?

Cycle 1: Read h0 from URAM: V=0  ← WRONG! Should be 1000
Cycle 2: Compute: V_new = 0 + 500 = 500
Cycle 3: Write h0 to URAM: V=500  ← WRONG! Lost the first update
```

**Solution: Hazard Detection**

The hardware tracks which neurons are "in flight" (being processed) and stalls if a conflict is detected:

```verilog
// Check if this neuron is already being processed
hazard = (neuron_addr == pipeline_stage1_addr) ||
         (neuron_addr == pipeline_stage2_addr) ||
         (neuron_addr == pipeline_stage3_addr);

if (hazard)
    stall = 1;  // Wait until pipeline clears
else
    proceed with read;
```

#### 4. Round-Robin Arbitration

When multiple sources compete for a resource, **round-robin** gives each source a fair turn.

```
Example: spike_fifo_controller collects from 8 spike FIFOs

Cycle 0: Check FIFO 0, if !empty, read
Cycle 1: Check FIFO 1, if !empty, read
Cycle 2: Check FIFO 2, if !empty, read
...
Cycle 7: Check FIFO 7, if !empty, read
Cycle 8: Back to FIFO 0
```

This prevents one busy FIFO from monopolizing the reader.

#### 5. Clock Domains and Synchronization

Our FPGA has **two clock domains:**
- **aclk = 225 MHz** (4.4 ns per cycle): Used by most modules
- **aclk450 = 450 MHz** (2.2 ns per cycle): Used by URAM for higher throughput

**Problem:** Data crossing between clock domains can cause **metastability** (undefined logic state).

**Solution:** Use async FIFOs with gray code synchronization (covered in Chapter 1's hardware_map.md).

When pointer_fifo writes data at 225 MHz and internal_events_processor reads at 450 MHz, the FIFO handles the synchronization safely.

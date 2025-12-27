---
title: Chapter 2
nav_order: 30
---

# Chapter 2: The Network Comes to Life

## Introduction

In Chapter 1, we initialized our network—programming the connectivity structure into HBM, clearing neuron states in URAM, and configuring network parameters. The FPGA now holds a frozen snapshot of our network architecture, but nothing is happening yet. The neurons are silent, their membrane potentials at zero.

In this chapter, we bring the network to life. We'll trace what happens when you call `network.step(['a0', 'a1', 'a2'])` to send input spikes to the first three axons. We'll follow these spikes as they:
1. **Phase 0:** Get written to BRAM as input patterns
2. **Phase 1:** Trigger reads from HBM to fetch synaptic connections
3. **Phase 2:** Flow into neurons, causing membrane potentials to rise and neurons to spike
4. **Phase 3:** Get collected and sent back to the host

We'll use our example network from the Introduction:
- **5 axons** (a0-a4) → **5 hidden neurons** (h0-h4) → **5 output neurons** (o0-o4)
- All weights = 1000
- Threshold = 2000
- **Input:** Axons a0, a1, a2 fire every timestep

By the end of this chapter, you'll understand exactly what happens in hardware during a single timestep, from input arrival to spike output, at the clock-cycle level.

---

## 2.1 The Execution Cycle: Hardware Visualization

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

---

## 2.2 The Code Behind Execution

Now that we understand the hardware flow, let's trace through the actual software and Verilog code that makes it happen.

### Phase 0: Sending Inputs to BRAM

#### Python Code: network.step()

File: `hs_api/api.py` (lines 470-552)

```python
def step(self, inputs, target="simpleSim", membranePotential=False):
    """Runs a step of the simulation."""

    # Convert symbolic input names to numerical indices
    # inputs = ['a0', 'a1', 'a2']
    formated_inputs = [
        self.connectome.get_neuron_by_key(symbol).get_coreTypeIdx()
        for symbol in inputs
    ]
    # formated_inputs = [0, 1, 2] (axon indices)

    if self.target == "CRI":
        if self.simDump:
            return self.CRI.run_step(formated_inputs)
        else:
            spikeResult = self.CRI.run_step(formated_inputs, membranePotential)
            # ... (parse and return spikes)
```

This calls `hs_bridge.network.run_step()`, which calls `fpga_controller.input_user()`.

#### Python Code: fpga_controller.input_user()

File: `hs_bridge/FPGA_Execution/fpga_controller.py` (lines 899-1031)

```python
def input_user(inputs, numAxons, simDump=False, coreID=0, reserve=True, cont_flag=False):
    """
    Generates the input command for a given timestep

    Parameters:
    - inputs: list of axon indices (e.g., [0, 1, 2])
    - numAxons: total number of axons (e.g., 5)
    """

    currInput = inputs  # [0, 1, 2]
    currInput.sort()    # Ensure sorted order

    coreBits = np.binary_repr(coreID, 5) + 3*'0'  # 5 bits for coreID
    coreByte = int(coreBits, 2)

    commandList = [0]*62 + [coreByte] + [1]  # Init packet: opcode 0x01

    # For networks with <= 256 axons, we use a single 256-bit chunk
    for count in range(math.ceil(numAxons/256)):
        # Create 256-bit one-hot encoding
        one_hot_bin = ["0"] * 256
        inputSegment = [i for i in currInput
                        if (256*count) <= i and i < (256*count+256)]

        for axon in inputSegment:
            one_hot_bin[axon%256] = "1"

        # Convert to bytes (8 bits per byte, little-endian)
        while one_hot_bin:
            curr_byte = one_hot_bin[:8][::-1]  # Reverse for endianness
            curr_byte = "".join(curr_byte)
            commandList = commandList + [int(curr_byte, 2)]
            one_hot_bin = one_hot_bin[8:]

        # Add tail: padding + coreID + opcode
        tail = 30*[0]
        commandList = commandList + tail + [coreByte, 0]  # Opcode 0x00

    # Example for inputs [0, 1, 2]:
    # one_hot_bin initially: ["0"]*256
    # After setting bits: one_hot_bin[0]="1", one_hot_bin[1]="1", one_hot_bin[2]="1"
    # First byte: "00000111" reversed = "11100000" = 0xE0... wait that's wrong

    # Let me trace more carefully:
    # one_hot_bin[0] = "1", one_hot_bin[1] = "1", one_hot_bin[2] = "1"
    # one_hot_bin = ["1", "1", "1", "0", "0", "0", ..., "0"]
    # curr_byte = one_hot_bin[:8] = ["1", "1", "1", "0", "0", "0", "0", "0"]
    # curr_byte reversed = ["0", "0", "0", "0", "0", "1", "1", "1"]
    # curr_byte string = "00000111"
    # int("00000111", 2) = 7 = 0x07 ✓

    # So commandList will contain:
    # [0]*62 + [coreID] + [1]  ← Init packet (opcode 0x01)
    # [0x07, 0x00, 0x00, ..., 0x00]  ← Data packet (256 bits = 32 bytes)
    # [0]*30 + [coreID] + [0]  ← Tail (opcode 0x00)

    command = np.array(commandList, dtype=np.uint64)

    # Send via DMA
    exitCode = dmadump.dma_dump_write(command, len(command),
                                       1, 0, 0, 0, dmadump.DmaMethodNormal)
```

**What gets sent:**
1. **Init packet (64 bytes):** Opcode 0x01, tells FPGA "input data incoming"
2. **Data packet (64 bytes):** Opcode 0x00, contains 256-bit one-hot mask
   - Byte 0 = 0x07 (bits 0,1,2 set for axons a0,a1,a2)
   - Bytes 1-31 = 0x00
   - Tail: padding + coreID + opcode

#### Verilog: command_interpreter Receives Input

File: `hardware_code/gopa/CRI_proj/command_interpreter.v` (conceptual)

```verilog
// State machine receives packets from Input FIFO
always @(posedge aclk) begin
    case (state)
        IDLE: begin
            if (!input_fifo_empty) begin
                input_fifo_rd_en <= 1'b1;
                state <= READ_PACKET;
            end
        end

        READ_PACKET: begin
            packet <= input_fifo_dout[511:0];
            opcode <= input_fifo_dout[511:504];
            coreID <= input_fifo_dout[503:496];
            payload <= input_fifo_dout[495:0];
            state <= PROCESS_OPCODE;
        end

        PROCESS_OPCODE: begin
            case (opcode)
                8'h01: begin  // Init input packet
                    // Prepare to receive data packet
                    state <= IDLE;  // Wait for next packet
                end

                8'h00: begin  // Input data packet
                    // Extract 256-bit one-hot mask
                    axon_mask[255:0] <= payload[255:0];

                    // Write to BRAM via input_data_handler
                    bram_wr_en <= 1'b1;
                    bram_wr_addr <= bram_row_addr;  // Address for this input
                    bram_wr_data <= axon_mask;

                    state <= IDLE;
                end
            endcase
        end
    endcase
end
```

#### Verilog: input_data_handler Writes to BRAM

File: `hardware_code/gopa/CRI_proj/input_data_handler.v`

```verilog
// Arbitrates BRAM access
// Priority: command_interpreter > external_events_processor

always @(posedge aclk) begin
    if (cmd_interp_wr_en) begin
        // Grant access to command interpreter
        bram_addr <= cmd_interp_addr;
        bram_we <= 1'b1;
        bram_din <= cmd_interp_data;  // 256-bit axon mask
    end
    else if (ext_events_rd_en) begin
        // Grant access to external events processor (reads only)
        bram_addr <= ext_events_addr;
        bram_we <= 1'b0;  // Read, not write
    end
end

// BRAM primitive (Xilinx RAMB36)
// Address = 15 bits, Data = 256 bits
RAMB36E2 #(
    .ADDR_WIDTH(15),
    .DATA_WIDTH(256)
) bram_inst (
    .CLKA(aclk),
    .ADDRA(bram_addr),
    .DINA(bram_din),
    .DOUTA(bram_dout),
    .WEA(bram_we),
    .ENA(1'b1)
);
```

**Physical operation:**
- `bram_addr = 0` (row 0)
- `bram_din = 256'h0000...0007` (bits 0,1,2 set)
- `bram_we = 1` (write enable)
- On next clock edge: BRAM cell capacitors charge/discharge to store the pattern
- Data is now persistent in BRAM until overwritten

---

### Triggering Execution: execute() Command

After writing inputs, we need to trigger execution:

```python
# In hs_bridge/network.py
def run_step(self, inputs):
    # Write inputs (just did above)
    fpga_controller.input_user(inputs, numAxons, coreID)

    # Trigger execution
    fpga_controller.execute(coreID)

    # Collect results
    spikes = fpga_controller.flush_spikes(coreID)
    return spikes
```

#### Python Code: fpga_controller.execute()

File: `hs_bridge/FPGA_Execution/fpga_controller.py` (lines 872-892)

```python
def execute(simDump=False, coreID=0):
    """Runs a single step of the network"""

    coreBits = np.binary_repr(coreID, 5) + 3*'0'
    command = np.array([0]*62 + [int(coreBits, 2), 6], dtype=np.uint64)
    # Opcode 0x06 = execute timestep

    exitCode = dmadump.dma_dump_write(command, len(command),
                                       1, 0, 0, 0, dmadump.DmaMethodNormal)
```

#### Verilog: command_interpreter Triggers external_events_processor

```verilog
always @(posedge aclk) begin
    case (opcode)
        8'h06: begin  // Execute command
            execute_pulse <= 1'b1;  // One-cycle pulse to ext_events_processor
            state <= IDLE;
        end
    endcase
end

// Wire to external_events_processor
assign ext_events_start = execute_pulse;
```

---

### Phase 1: External Event Processing

Now the real action begins!

#### Verilog: external_events_processor State Machine

File: `hardware_code/gopa/CRI_proj/external_events_processor.v` (conceptual - actual code has optimizations)

```verilog
// Simplified state machine for external event processing

reg [3:0] state;
localparam IDLE = 0;
localparam SCAN_BRAM = 1;
localparam WAIT_BRAM = 2;
localparam PARSE_MASK = 3;
localparam REQUEST_HBM = 4;
localparam WAIT_HBM = 5;
localparam PROCESS_SYNAPSES = 6;

reg [14:0] bram_row_counter;  // Which BRAM row are we scanning
reg [7:0] bit_index;           // Which bit in mask are we checking
reg [255:0] current_mask;      // Current BRAM row data

always @(posedge aclk) begin
    case (state)
        IDLE: begin
            if (ext_events_start) begin
                bram_row_counter <= 15'b0;
                state <= SCAN_BRAM;
            end
        end

        SCAN_BRAM: begin
            // Request BRAM read for current row
            bram_rd_en <= 1'b1;
            bram_rd_addr <= bram_row_counter;
            state <= WAIT_BRAM;
        end

        WAIT_BRAM: begin
            // Wait 3 cycles for BRAM read latency
            // (Could use a counter, simplified here)
            if (bram_rd_valid) begin
                current_mask <= bram_rd_data[255:0];
                bit_index <= 8'b0;
                state <= PARSE_MASK;
            end
        end

        PARSE_MASK: begin
            // Check if current bit is set
            if (current_mask[bit_index]) begin
                // This axon is active!
                // Calculate axon index
                axon_index <= (bram_row_counter * 256) + bit_index;
                state <= REQUEST_HBM;
            end
            else begin
                // Move to next bit
                bit_index <= bit_index + 1;
                if (bit_index == 255) begin
                    // Done with this row, move to next
                    bram_row_counter <= bram_row_counter + 1;
                    if (bram_row_counter == MAX_ROWS) begin
                        state <= IDLE;  // Done scanning all rows
                    end
                    else begin
                        state <= SCAN_BRAM;
                    end
                end
            end
        end

        REQUEST_HBM: begin
            // Calculate HBM address for axon pointer
            // Axon pointers start at 0x0000, 8 pointers per row
            pointer_row = axon_index / 8;
            pointer_offset = axon_index % 8;

            // Request read from HBM
            hbm_rd_en <= 1'b1;
            hbm_rd_addr <= {AXN_BASE_ADDR, pointer_row, 5'b00000};
            // Address format: base + row * 32 bytes

            state <= WAIT_HBM;
        end

        WAIT_HBM: begin
            // Wait for HBM controller to return data
            if (hbm_rd_valid) begin
                // Extract pointer for this axon
                axon_pointer <= hbm_rd_data[(pointer_offset*32)+:32];
                // Format: [31:23]=length, [22:0]=start_addr

                syn_start_addr <= axon_pointer[22:0] + SYN_BASE_ADDR;
                syn_length <= axon_pointer[31:23];

                state <= REQUEST_SYNAPSES;
            end
        end

        REQUEST_SYNAPSES: begin
            // Read synapse data rows
            // (Loop over syn_length rows, simplified here)
            hbm_rd_en <= 1'b1;
            hbm_rd_addr <= {syn_start_addr, 5'b00000};
            state <= WAIT_SYNAPSE_DATA;
        end

        WAIT_SYNAPSE_DATA: begin
            if (hbm_rd_valid) begin
                // Got 256 bits = 8 synapses
                synapse_data <= hbm_rd_data[255:0];

                // Send to pointer_fifo_controller for distribution
                syn_data_valid <= 1'b1;

                // Continue parsing mask
                bit_index <= bit_index + 1;
                state <= PARSE_MASK;
            end
        end
    endcase
end
```

**Example trace for our network (axons 0, 1, 2 active):**

```
Cycle 0: IDLE
  execute_pulse arrives

Cycle 1: SCAN_BRAM
  bram_rd_addr <= 0 (row 0)

Cycle 4: WAIT_BRAM (after 3-cycle latency)
  current_mask <= 256'h00...07 (bits 0,1,2 set)
  bit_index <= 0

Cycle 5: PARSE_MASK
  current_mask[0] == 1 → axon 0 is active
  axon_index <= 0

Cycle 6: REQUEST_HBM
  pointer_row = 0 / 8 = 0
  pointer_offset = 0 % 8 = 0
  hbm_rd_addr <= 0x0000_0000 (axon pointer row 0)

Cycle 50: WAIT_HBM (HBM latency ~100-200ns = ~22-45 cycles)
  hbm_rd_data[31:0] <= 0x0080_0000 (axon 0 pointer)
  syn_start_addr <= 0x8000
  syn_length <= 1

Cycle 51: REQUEST_SYNAPSES
  hbm_rd_addr <= 0x8000 * 32 = 0x0001_0000

Cycle 95: WAIT_SYNAPSE_DATA
  hbm_rd_data <= 256'h...03E8_0004_03E8_0003_03E8_0002_03E8_0001_03E8_0000
    Synapse 0: target=0 (h0), weight=1000 (0x03E8)
    Synapse 1: target=1 (h1), weight=1000
    Synapse 2: target=2 (h2), weight=1000
    Synapse 3: target=3 (h3), weight=1000
    Synapse 4: target=4 (h4), weight=1000
    Synapses 5-7: unused

  Send to pointer_fifo_controller

Cycle 96: PARSE_MASK
  bit_index <= 1
  current_mask[1] == 1 → axon 1 is active

... (repeat for axons 1 and 2)
```

---

### Phase 2: Pointer Distribution

#### Verilog: pointer_fifo_controller

File: `hardware_code/gopa/CRI_proj/pointer_fifo_controller.v`

```verilog
// Receives synapse data from HBM, routes to 16 neuron groups

reg [255:0] synapse_row_data;

always @(posedge aclk) begin
    if (syn_data_valid) begin
        synapse_row_data <= syn_data;

        // Parse 8 synapses in parallel
        for (i = 0; i < 8; i = i + 1) begin
            synapse[i] = synapse_row_data[(i*32)+:32];
            opcode[i] = synapse[i][31:29];
            target_addr[i] = synapse[i][28:16];  // 13-bit address
            weight[i] = synapse[i][15:0];

            // Calculate neuron group (top 4 bits of address)
            neuron_group[i] = target_addr[i][12:9];
            local_addr[i] = target_addr[i][8:0];  // Address within group

            // Write to appropriate FIFO
            if (synapse[i] != 32'h0000_0000) begin  // Not unused
                pointer_fifo_wr_en[neuron_group[i]] <= 1'b1;
                pointer_fifo_din[neuron_group[i]] <= {weight[i], local_addr[i]};
            end
        end
    end
end

// 16 pointer FIFOs (one per neuron group)
genvar g;
generate
    for (g = 0; g < 16; g = g + 1) begin : gen_fifos
        FIFO18E2 #(
            .DATA_WIDTH(32),  // 16-bit weight + 16-bit local address
            .DEPTH(512)
        ) pointer_fifo (
            .WR_CLK(aclk),
            .WR_EN(pointer_fifo_wr_en[g]),
            .DIN(pointer_fifo_din[g]),
            .FULL(pointer_fifo_full[g]),

            .RD_CLK(aclk450),  // Read side at 450 MHz
            .RD_EN(pointer_fifo_rd_en[g]),
            .DOUT(pointer_fifo_dout[g]),
            .EMPTY(pointer_fifo_empty[g])
        );
    end
endgenerate
```

**Example for first synapse (a0 → h0, weight=1000):**

```
synapse = 0x0000_03E8
opcode = 000 (regular synapse)
target_addr = 13'b0000000000000 = 0 (neuron h0)
weight = 16'h03E8 = 1000

neuron_group = 0[12:9] = 0 (top 4 bits)
local_addr = 0[8:0] = 0

pointer_fifo_wr_en[0] <= 1
pointer_fifo_din[0] <= {1000, 0} = 32'h03E8_0000
```

After all 15 synapses (3 axons × 5 targets each) are processed, `pointer_fifo[0]` contains 15 entries, all for group 0 since our network is small.

---

### Phase 3: Neuron State Updates

This is where the magic happens! Neurons integrate inputs and spike.

#### Verilog: internal_events_processor (Per-Bank State Machine)

File: `hardware_code/gopa/CRI_proj/internal_events_processor.v`

```verilog
// Simplified version for Bank 0 (one of 16 parallel copies)
// Runs @ 450 MHz (aclk450)

reg [2:0] state;
localparam IEP_IDLE = 0;
localparam CHECK_FIFO = 1;
localparam READ_URAM = 2;
localparam ACCUMULATE = 3;
localparam APPLY_MODEL = 4;
localparam WRITE_URAM = 5;
localparam CHECK_SPIKE = 6;

reg [12:0] local_neuron_addr;  // Address within this bank (0-8191)
reg [15:0] weight;
reg [35:0] V_old, V_new, V_final;
reg spike;

// Neuron parameters (programmed during initialization)
reg [35:0] threshold = 36'd2000;  // From write_neuron_type()
reg [5:0] leak = 6'd63;            // Max leak = no leak (IF neuron)
reg [1:0] neuron_model = 2'b00;   // 00 = IF

// Pipeline hazard tracking
reg [12:0] pipeline_addr [0:4];  // Track addresses in pipeline

always @(posedge aclk450) begin
    case (state)
        IEP_IDLE: begin
            if (!pointer_fifo_empty[0]) begin
                state <= CHECK_FIFO;
            end
        end

        CHECK_FIFO: begin
            // Read from pointer FIFO
            pointer_fifo_rd_en[0] <= 1'b1;
            state <= READ_URAM;
        end

        READ_URAM: begin
            // FIFO output valid (FWFT mode)
            {weight, local_neuron_addr} <= pointer_fifo_dout[0];

            // Check for hazard
            hazard = (local_neuron_addr == pipeline_addr[1]) ||
                     (local_neuron_addr == pipeline_addr[2]) ||
                     (local_neuron_addr == pipeline_addr[3]);

            if (hazard) begin
                // Stall until pipeline clears
                state <= READ_URAM;
            end
            else begin
                // Request neuron state from URAM
                uram_addr <= local_neuron_addr[12:1];  // Divide by 2
                uram_rd_en <= 1'b1;

                pipeline_addr[0] <= local_neuron_addr;
                state <= ACCUMULATE;
            end
        end

        ACCUMULATE: begin
            // URAM has 1-cycle latency
            uram_data_word <= uram_dout[71:0];

            // Select which neuron (2 neurons per 72-bit word)
            if (local_neuron_addr[0] == 1'b0)
                V_old = uram_data_word[35:0];   // Lower neuron
            else
                V_old = uram_data_word[71:36];  // Upper neuron

            // Integrate synaptic input
            V_new = V_old + weight;

            pipeline_addr[1] <= pipeline_addr[0];
            state <= APPLY_MODEL;
        end

        APPLY_MODEL: begin
            // Apply leak (if enabled)
            if (neuron_model != 2'b00) begin  // Not IF
                V_new = V_new - (V_new >> leak);
            end

            // Check threshold
            spike = (V_new >= threshold);

            // Reset if spike
            if (spike)
                V_final = 36'b0;
            else
                V_final = V_new;

            pipeline_addr[2] <= pipeline_addr[1];
            state <= WRITE_URAM;
        end

        WRITE_URAM: begin
            // Reconstruct 72-bit word
            if (local_neuron_addr[0] == 1'b0)
                uram_din = {uram_data_word[71:36], V_final};
            else
                uram_din = {V_final, uram_data_word[35:0]};

            // Write back to URAM
            uram_we <= 1'b1;
            uram_addr <= local_neuron_addr[12:1];

            pipeline_addr[3] <= pipeline_addr[2];
            state <= CHECK_SPIKE;
        end

        CHECK_SPIKE: begin
            if (spike) begin
                // Send spike to spike_fifo_controller
                spike_fifo_wr_en <= 1'b1;
                spike_fifo_din <= {4'b0000, local_neuron_addr};  // 17-bit global address
            end

            pipeline_addr[4] <= pipeline_addr[3];
            state <= IEP_IDLE;  // Back to check FIFO
        end
    endcase
end
```

**Example trace for neuron h0 receiving 3 inputs:**

```
Input 1: From axon a0, weight=1000

Cycle 0: CHECK_FIFO
  pointer_fifo_dout[0] = {1000, 0} (h0, weight 1000)

Cycle 1: READ_URAM
  local_neuron_addr = 0 (h0)
  weight = 1000
  uram_addr = 0 >> 1 = 0
  No hazard (pipeline empty)

Cycle 2: ACCUMULATE
  uram_dout[71:0] = {neuron 1 data, neuron 0 data}
  V_old = uram_dout[35:0] = 36'h0_0000_0000 (zero)
  V_new = 0 + 1000 = 1000

Cycle 3: APPLY_MODEL
  neuron_model = IF, no leak applied
  V_new = 1000
  spike = (1000 >= 2000) = 0 (no spike)
  V_final = 1000

Cycle 4: WRITE_URAM
  uram_din = {upper_neuron_data, 36'd1000}
  uram_we = 1

Cycle 5: CHECK_SPIKE
  spike = 0, no spike output
  Back to IDLE

Input 2: From axon a1, weight=1000

Cycle 10: CHECK_FIFO
  pointer_fifo_dout[0] = {1000, 0} (h0 again)

Cycle 11: READ_URAM
  uram_addr = 0
  Check hazard: local_addr (0) == pipeline_addr[1,2,3]?
    pipeline_addr cleared from last operation
  No hazard

Cycle 12: ACCUMULATE
  V_old = 1000 (from previous update!)
  V_new = 1000 + 1000 = 2000

Cycle 13: APPLY_MODEL
  spike = (2000 >= 2000) = 1 (SPIKE!)
  V_final = 0 (reset)

Cycle 14: WRITE_URAM
  uram_din = {upper_neuron_data, 36'b0}
  uram_we = 1

Cycle 15: CHECK_SPIKE
  spike = 1
  spike_fifo_wr_en = 1
  spike_fifo_din = 17'b0_0000_0000_0000_0000 (neuron 0 = h0)

Input 3: From axon a2, weight=1000

Cycle 20: CHECK_FIFO
  pointer_fifo_dout[0] = {1000, 0}

Cycle 21: READ_URAM
  uram_addr = 0

Cycle 22: ACCUMULATE
  V_old = 0 (was reset after spike!)
  V_new = 0 + 1000 = 1000

Cycle 23: APPLY_MODEL
  spike = (1000 >= 2000) = 0 (no spike)
  V_final = 1000

Cycle 24: WRITE_URAM
  uram_we = 1, V = 1000

Cycle 25: CHECK_SPIKE
  No spike
```

**Final state of h0:** V = 1000, spiked once during this timestep.

---

### Hidden Neuron Spikes → Output Neurons

When h0-h4 spike, they trigger a second round of Phase 1-3:

1. **Spike routing:** spike_fifo_controller sends spike events to external_events_processor
2. **Phase 1:** external_events_processor reads neuron pointers from HBM (similar to axon pointers)
3. **Phase 2:** pointer_fifo_controller distributes h0-h4's output synapses
4. **Phase 3:** internal_events_processor updates o0-o4

For output neurons:
```
Each output neuron receives: 5 hidden neurons × 1000 = 5000 input
o0: V = 0 + 5000 = 5000 >= 2000 → SPIKE
... (all outputs spike)
```

Output neuron spikes have OpCode=100 in their synapse entries, which tells spike_fifo_controller to send them to the host instead of back to external_events_processor.

---

### Reading Results: flush_spikes()

#### Python Code: fpga_controller.flush_spikes()

File: `hs_bridge/FPGA_Execution/fpga_controller.py` (lines 273-343)

```python
def flush_spikes(coreID=0):
    """Reads spike packets from FPGA via PCIe"""

    packetNum = 1
    spikeOutput = []
    n = 0

    time.sleep(800/1000000.0)  # Wait 800 µs for spike processing

    while True:
        exitCode, batchRead = dmadump.dma_dump_read(
            1, 0, 0, 0, dmadump.DmaMethodNormal, 64*packetNum
        )

        splitRead = np.array_split(batchRead, packetNum)
        splitRead.reverse()
        flushed = False

        for currentRead in splitRead:
            # Check packet type by tag
            if (currentRead[62] == 255 and currentRead[63] == 255):
                # FIFO Empty packet (0xFFFF tag)
                n += 1
                if n == 50:
                    flushed = True
                    break
            elif (currentRead[62] == 238 and currentRead[63] == 238):
                # Spike packet (0xEEEE tag)
                executionRun_counter, spikeList = read_spikes(currentRead)
                spikeOutput = spikeOutput + spikeList
                n = 0
            elif (currentRead[62] == 205 and currentRead[63] == 171):
                # Latency packet (0xCDAB tag) - end of execution
                executionRun_counter, spikeList = read_spikes(currentRead)
                spikeOutput = spikeOutput + spikeList
                flushed = True
                break
            else:
                logging.error("Non-spike packet encountered")

        if flushed:
            break

    # Read latency and HBM access count
    exitCode, batchRead = dmadump.dma_dump_read(...)
    latency = parse_latency(batchRead)

    exitCode, batchRead = dmadump.dma_dump_read(...)
    hbmAcc = parse_hbm_access_count(batchRead)

    return (spikeOutput, latency, hbmAcc)
```

#### Helper: read_spikes()

File: `hs_bridge/FPGA_Execution/fpga_controller.py` (lines 96-126)

```python
def read_spikes(data):
    """Decodes a spike packet"""

    data = np.flip(data)  # MSB first
    binData = [np.binary_repr(i, width=8) for i in data]
    binData = ''.join(binData)

    # Extract tag
    tag = int(binData[:-480], 2)

    # Extract execution counter (timestep)
    executionRun_counter = binData[-32:]

    # Extract spike data region
    spikeData = binData[-480:-32]

    # Parse individual spike packets (32 bits each)
    spikePacketLength = 32
    spikeList = []
    for spikePacket in [spikeData[i:i+32] for i in range(0, len(spikeData), 32)]:
        subexecutionRun_counter, address = processSpikePacket(spikePacket)
        if address is not None:
            spikeList.append((subexecutionRun_counter, address))

    return executionRun_counter, spikeList

def processSpikePacket(spikePacket):
    """Processes a single spike entry (32 bits)"""

    valid = bool(int(spikePacket[8]))  # Bit 23 in original packet
    if valid:
        subexecutionRun_counter = int(spikePacket[0:8], 2)
        address = int(spikePacket[-17:], 2)  # 17-bit neuron address
        return subexecutionRun_counter, address
    else:
        return None, None
```

**Example spike packet for our network:**

```
512-bit packet received from FPGA:

Bits [511:496] = 0xEEEE (spike packet tag)
Bits [495:32] = Spike data (14 spikes × 32 bits)
  Spike 0: valid=1, address=5 (o0)
  Spike 1: valid=1, address=6 (o1)
  Spike 2: valid=1, address=7 (o2)
  Spike 3: valid=1, address=8 (o3)
  Spike 4: valid=1, address=9 (o4)
  Spikes 5-13: valid=0 (unused)
Bits [31:0] = 0 (timestep counter)

Result: spikeList = [(0, 5), (0, 6), (0, 7), (0, 8), (0, 9)]
        Which converts to: ['o0', 'o1', 'o2', 'o3', 'o4']
```

---

## Conclusion: The Living Network

We've now traced the complete journey of a spike through the hardware:

1. **Phase 0:** User calls `network.step(['a0', 'a1', 'a2'])`
   - Inputs converted to one-hot mask: `0b00000111`
   - Written to BRAM via PCIe and command_interpreter

2. **Phase 1:** Execute command triggers external_events_processor
   - Reads BRAM: finds bits 0,1,2 set
   - For each active axon: reads pointer from HBM, then synapse data
   - 3 axons × 5 synapses = 15 synapses fetched

3. **Phase 2:** pointer_fifo_controller distributes synapses
   - All 15 go to neuron group 0 (our small network)
   - Each hidden neuron h0-h4 appears 3 times in FIFO

4. **Phase 3:** internal_events_processor updates neurons @ 450 MHz
   - h0: V=0 → 1000 → 2000 (SPIKE!) → 0 → 1000
   - h1-h4: same pattern
   - All hidden neurons spike after 2nd input

5. **Recurrent Phase 1-3:** Hidden spikes trigger output updates
   - 5 hidden × 5 outputs = 25 synapses
   - Each output gets 5000 input → all spike

6. **Spike Output:** o0-o4 sent to host via PCIe
   - Packaged into 512-bit spike packet
   - Read by flush_spikes()
   - Returned to user: `['o0', 'o1', 'o2', 'o3', 'o4']`

**Total time: ~2-5 microseconds** from input to output.

In the next timestep, neurons start with their updated membrane potentials (h0-h4 at V=1000, o0-o4 at V=0) and the process repeats. Over many timesteps, the network dynamics emerge from these rapid hardware updates, enabling real-time spike-based computation at millisecond timescales.

---
title: 0.4 Communication Mechanisms Basics
nav_order: 4
parent: "0 Introduction"
---

# Communication Mechanisms Basics

### **Packet-Based Communication (PCIe)**
**What it is:** Data encapsulated in discrete packets with headers and checksums

**Characteristics:**
- **Self-contained:** Each packet has address, length, payload
- **Routing:** Intermediate switches use header to route packet
- **Error detection:** CRC checksum, retry on corruption
- **No dedicated path:** Packets from multiple sources share physical lanes
  - Time-division multiplexing on the serial lanes
  - Switch buffers packets, forwards when lane available
- **Latency:** Variable (depends on congestion, packet size)
  - Small packet: ~500ns end-to-end (serialization + propagation + deserialization)
  - Large burst: ~10µs for 4KB (amortized per-byte cost is lower)

**PCIe TLP Flow (Memory Write example):**
1. **Host CPU:** Wants to write 512 bits to FPGA address 0xD000_0000
2. **PCIe Root Complex:** Formats TLP:
   ```
   Header: [Fmt=010 (64-bit write), Type=00000 (memory), Length=16 DW]
           [Requester=00:00.0, Tag=5, FirstBE=0xF, LastBE=0xF]
           [Address=0x0000_00D0_0000_0000]
   Payload: [64 bytes of data = 512 bits]
   LCRC: [32-bit CRC of header + payload]
   ```
3. **PCIe Link Layer:** Adds sequence number, splits into 256-byte packets (max TLP size)
4. **PCIe Physical Layer:** 128b/130b encodes, serializes onto 16 lanes
   - Each lane transmits ~1 bit every 125 picoseconds (8 Gb/s)
5. **FPGA PCIe Endpoint:** Deserializes, checks CRC, reassembles
6. **PCIe Block → AXI4:** Converts TLP to AXI4 write transaction
   - AWADDR=0xD000_0000, WDATA=payload, WVALID=1
7. **pcie2fifos:** Receives AXI4 write, pushes to Input FIFO

### **Bus-Based Communication (AXI4)**
**What it is:** Shared parallel wires with arbitration for multiple masters

**Characteristics:**
- **Dedicated wires:** Each signal is a separate wire (e.g., WDATA[511:0] = 512 wires)
- **Parallel:** All bits transfer simultaneously in one clock cycle
- **Handshake:** VALID/READY protocol ensures synchronization
- **Arbitration:** If multiple masters, arbiter grants access to one at a time
- **Predictable latency:** Fixed cycles for each transfer (if no contention)

**AXI4 Write Transaction (detailed timing):**
```
Clock Cycle   AWVALID  AWREADY  AWADDR    WVALID  WREADY  WDATA      BVALID  BREADY
─────────────────────────────────────────────────────────────────────────────────────
0             0        0        X         0       0       X          0       0
1 (Master)    1        0        0x8000    1       0       0xABCD...  0       0
2 (Slave)     1        1        0x8000    1       1       0xABCD...  0       0    ← Transfer!
3             0        1        X         0       1       X          0       0
4 (Slave)     0        0        X         0       0       X          1       0
5 (Master)    0        0        X         0       0       X          1       1    ← Response!
6             0        0        X         0       0       X          0       0
```
- Cycle 1: Master asserts AWVALID and WVALID (address and data ready)
- Cycle 2: Slave asserts AWREADY and WREADY (can accept), transfer occurs
- Cycle 5: Slave asserts BVALID (write complete), master asserts BREADY, response accepted

**Routing in FPGA:** AXI4 buses connect modules via routing fabric
- Not physical wires (like PCB traces)
- Programmable switches in FPGA fabric route signals
- Example: hbm_processor AWADDR[32:0] → routed to HBM interface pins
  - Router finds path through switch matrices, programs SRAM bits
  - Timing: Signal may pass through 10+ switches, takes ~2ns

### **Point-to-Point Communication (FIFO Handshake)**
**What it is:** Dedicated connection between two modules with ready/valid signals

**Characteristics:**
- **Direct:** Only 2 endpoints (1 writer, 1 reader)
- **Simple protocol:** WR_EN/FULL for write, RD_EN/EMPTY for read
- **No arbitration:** Dedicated resource, no sharing
- **Low latency:** Immediate if FIFO not full/empty
  - Write: If !FULL, assert WR_EN, data stored next cycle
  - Read (FWFT): If !EMPTY, data already on DO port

**FIFO Write Timing:**
```
Clock   WR_EN  DI[511:0]      WP   FULL
────────────────────────────────────────
0       0      X              0    0
1       1      0xAAAA...      0    0
2       0      X              1    0     ← Data stored, pointer incremented
3       1      0xBBBB...      1    0
4       0      X              2    0
...
510     1      0xFFFF...      509  0
511     0      X              510  0
512     1      0x1111...      510  1     ← FULL asserted, write rejected
513     0      X              510  1
```

**FIFO Read Timing (FWFT mode):**
```
Clock   RD_EN  DO[511:0]      RP   EMPTY
────────────────────────────────────────
0       0      X              0    1
(External write occurs)
1       0      0xAAAA...      0    0     ← Data appears same cycle as !EMPTY
2       1      0xAAAA...      0    0
3       0      0xBBBB...      1    0     ← Pointer incremented, next data appears
4       1      0xBBBB...      1    0
5       0      X              2    1     ← Last word read, EMPTY asserted
```

### **Comparison:**

| Mechanism | Bandwidth | Latency | Complexity | Use Case |
|-----------|-----------|---------|------------|----------|
| **PCIe (Packet)** | ~14 GB/s | ~500ns-10µs | High (TLP format, CRC, retry) | Host ↔ FPGA (long distance) |
| **AXI4 (Bus)** | ~28.8 GB/s @ 225MHz × 512-bit | ~4-20 cycles | Medium (5 channels, handshakes) | FPGA modules, HBM access |
| **FIFO (Point-to-Point)** | ~14.4 GB/s @ 225MHz × 512-bit | 1 cycle | Low (just counters + flags) | Producer-consumer pipelines |

**Why use different mechanisms?**
- **PCIe:** Standardized, works across physical boards, plug-and-play
- **AXI4:** Flexible, supports bursts, out-of-order, multiple masters
- **FIFO:** Simplest, lowest latency, decouples clock domains

---

## How Communication Actually Works: Step-by-Step Example

**Scenario:** User calls `network.step(['a0', 'a1'])` → Send axon spikes to FPGA

### Step 1: Software Preparation (hs_bridge)
```python
# In fpga_controller.input_user()
inputs = ['a0', 'a1']  # User-provided spikes
numAxons = 5           # Total axons in network

# Convert to one-hot bitmask
one_hot = [0] * 256     # 256-bit field
one_hot[0] = 1          # Axon a0
one_hot[1] = 1          # Axon a1
# one_hot is now [1, 1, 0, 0, 0, ..., 0]

# Pack into bytes (little-endian)
byte_array = []
for i in range(0, 256, 8):
    byte_val = 0
    for bit in range(8):
        if one_hot[i + bit]:
            byte_val |= (1 << bit)
    byte_array.append(byte_val)
# byte_array[0] = 0b00000011 = 0x03 (bits 0 and 1 set)

# Build 512-bit command packet
packet = [0] * 64  # 64 bytes = 512 bits
packet[63] = 0x00  # Opcode = 0x00 (input data)
packet[62] = 0x00  # CoreID = 0
# ... (fill remaining bytes with axon bitmask data)
packet[0:32] = byte_array[0:32]  # 256 bits of axon data

# Call DMA library
dmadump.dma_dump_write(np.array(packet), len(packet), ...)
```

### Step 2: Host → FPGA Transfer (PCIe DMA)
```
1. dmadump library:
   - Writes packet[] to DMA buffer in host memory (DDR4)
   - Buffer physical address: 0x8000_1000 (example)

2. dmadump library:
   - Writes MMIO registers on FPGA (PCIe Memory Write TLP):
     Register 0x100: Source address = 0x8000_1000
     Register 0x104: Destination = 0 (Input FIFO)
     Register 0x108: Length = 64 bytes
     Register 0x10C: Control = 0x1 (start DMA)

3. FPGA DMA engine (part of pcie2fifos):
   - Reads descriptor from MMIO registers
   - Issues PCIe Memory Read TLP:
     Address: 0x8000_1000
     Length: 64 bytes
   - Host root complex receives read request
   - Memory controller fetches data from DDR4
   - Returns data in Completion TLP

4. FPGA receives completion:
   - Deserializes 512-bit payload from TLP
   - Writes to Input FIFO:
     WR_EN <= 1
     DI[511:0] <= payload_data

5. Input FIFO stores data:
   - Writes to internal BRAM at write pointer address
   - Increments write pointer
   - Deasserts EMPTY flag (now contains data)
```

### Step 3: Command Interpretation (FPGA Internal)
```
Clock cycle N:
  command_interpreter checks Input FIFO:
    if (!input_fifo_empty) begin
      input_fifo_rd_en <= 1;
    end

Clock cycle N+1 (FIFO is FWFT):
  cmd_data = input_fifo_dout[511:0];
  opcode = cmd_data[511:504];  // Extract opcode = 0x00
  coreID = cmd_data[503:496];  // Extract coreID = 0x00
  payload = cmd_data[495:0];   // Axon bitmask data

  // Decode opcode
  if (opcode == 8'h00) begin
    state <= ROUTE_INPUT_DATA;
    bram_write_enable <= 1;
    bram_address <= <calculate BRAM row from coreID>;
    bram_data <= payload[255:0];  // 256-bit axon bitmask
  end

Clock cycle N+2:
  // Write to BRAM via input_data_handler
  Arbiter sees command_interpreter request
  Grants access (command has priority)
  BRAM receives:
    ADDR = row address
    DIN = 256-bit axon bitmask [255:0] = 0x0000...0003
      (bit 0 and 1 set for axons a0, a1)
    WE = 1 (write enable)
  BRAM performs write (data stored after 1 cycle)

Clock cycle N+3:
  command_interpreter:
    state <= IDLE;
    // Done processing this command
```

### Step 4: Execution Trigger (Separate Command)
```
User calls execute():
  fpga_controller.execute(coreID=0)
  Sends 512-bit packet with opcode 0x06
  (Same DMA process as above)

command_interpreter receives opcode 0x06:
  Asserts execute_pulse signal to external_events_processor
  external_events_processor state machine:
    IDLE → SCAN_BRAM
```

### Step 5: External Events Processing (Phase 1)
```
external_events_processor state machine:

State SCAN_BRAM:
  For each BRAM row (axon):
    // Request read from BRAM
    bram_rd_addr <= current_row;
    bram_rd_en <= 1;
    state <= WAIT_BRAM;

State WAIT_BRAM (3 cycles later):
  bram_data_valid <= 1;
  axon_masks[255:0] <= bram_dout[255:0];
  // axon_masks = 0x0000...0003 (bits 0,1 set)
  state <= PARSE_MASKS;

State PARSE_MASKS:
  For each bit in axon_masks:
    if (axon_masks[bit_idx] == 1) begin
      // Calculate HBM address for axon pointer
      axon_ptr_addr = AXN_BASE_ADDR + (current_row * 8 + bit_idx);
        // Example: row=0, bit=0 → addr = 0x0000_0000
        //          row=0, bit=1 → addr = 0x0000_0001

      // Request read from HBM via hbm_processor
      hbm_rd_en <= 1;
      hbm_rd_addr <= {axon_ptr_addr, 5'b00000};
        // Shift left 5 bits = multiply by 32 (byte address)
      state <= WAIT_HBM;
    end
```

### Step 6: HBM Access (AXI4 Transaction)
```
hbm_processor receives request:
  Queue read: address=0x0000_0000 (axon 0 pointer location)

AXI4 Read Transaction:
  Clock T0:
    ARVALID <= 1
    ARADDR <= 33'h0_0000_0000
    ARLEN <= 0 (single beat)
    ARSIZE <= 5 (32 bytes = 256 bits)

  Clock T1-T10: Wait for ARREADY
    HBM controller accepts request

  Clock T11-T50: HBM internal access
    Stack 0, Channel 0, Bank 0
    Row activation: Read row 0 into row buffer
    Column select: Bytes 0-31 from row buffer
    Sense amps detect charge on bitlines
    Data serialization: 256 bits → 1024-bit HBM bus

  Clock T51:
    RVALID <= 1 (from HBM controller)
    RDATA[255:0] <= HBM row data
      Contains 8 axon pointers (32 bits each)
      Axon 0 pointer: RDATA[31:0] = 0x0020_1000
        Length = 0x001 (1 synapse row)
        Start = 0x1000 (relative to SYN_BASE_ADDR)

  Clock T52:
    hbm_processor asserts RREADY
    Captures RDATA
    Parses pointer:
      syn_start_addr = 0x8000 + 0x1000 = 0x9000
      syn_length = 1

  Clock T53-T103: Read synapse row
    Issues new AXI4 read:
      ARADDR = 0x9000 * 32 = 0x0000_0001_2000
    HBM returns synapse data:
      RDATA[255:0] = 8 synapses
        [31:0]   = 0x0010_03E8 (opcode=0, addr=16, weight=1000)
        [63:32]  = 0x0011_03E8 (opcode=0, addr=17, weight=1000)
        ... (6 more synapses)
```

### Step 7: Pointer Distribution (Phase 2)
```
pointer_fifo_controller receives synapse data:
  syn_data[255:0] = HBM read result

  For each of 8 synapses in data:
    synapse[i] = syn_data[(i*32)+:32];
    opcode = synapse[31:29];
    target_addr = synapse[28:16];  // 13-bit neuron address
    weight = synapse[15:0];

    // Determine which neuron group
    neuron_group = target_addr[12:9];  // Top 4 bits
      // Example: target_addr=16 = 0b0000000010000
      //          neuron_group = 0b0000 = group 0

    // Write to corresponding FIFO
    pointer_fifo_wr_en[neuron_group] <= 1;
    pointer_fifo_din[neuron_group] <= {weight, target_addr[8:0]};
      // 32-bit format: [31:16]=weight, [15:0]=local address
      // local_addr = target_addr[8:0] = address within group
```

### Step 8: Neuron State Update (Phase 3)
```
internal_events_processor (Bank 0, @ 450 MHz):

Clock cycle M (FIFO has data):
  if (!pointer_fifo_empty[0]) begin
    fifo_rd_en[0] <= 1;
    state <= READ_FIFO;
  end

Clock cycle M+1:
  fifo_data = pointer_fifo_dout[0];
  weight = fifo_data[31:16];        // 1000
  local_addr = fifo_data[15:0];     // 16 (neuron index in this bank)

  // Request neuron state from URAM
  uram_addr <= local_addr[12:1];    // Divide by 2 (2 neurons per word)
  uram_rd_en <= 1;
  state <= READ_URAM;

Clock cycle M+2:
  uram_dout[71:0] = URAM read result
    // [71:36] = upper neuron state
    // [35:0]  = lower neuron state

  // Select neuron based on LSB of address
  if (local_addr[0] == 0)
    neuron_state = uram_dout[35:0];  // Lower neuron
  else
    neuron_state = uram_dout[71:36]; // Upper neuron

  // Current membrane potential
  V_old = neuron_state[35:0];  // Signed 36-bit value

  // Apply synaptic input
  V_new = V_old + weight;
    // Example: V_old=500, weight=1000 → V_new=1500

  state <= APPLY_MODEL;

Clock cycle M+3:
  // Apply leak (if enabled)
  if (leak_enable)
    V_new = V_new - (V_new >> leak_shift);

  // Check threshold
  spike = (V_new >= threshold);
    // Example: V_new=1500, threshold=2000 → spike=0 (no spike)

  // Reset if spike
  if (spike)
    V_final = 0;
  else
    V_final = V_new;
    // V_final = 1500 (no spike, keep accumulated value)

  state <= WRITE_URAM;

Clock cycle M+4:
  // Reconstruct 72-bit word
  if (local_addr[0] == 0)
    uram_din = {uram_dout[71:36], V_final};  // Update lower
  else
    uram_din = {V_final, uram_dout[35:0]};   // Update upper

  // Write back to URAM
  uram_we <= 1;
  uram_addr <= local_addr[12:1];
  uram_din <= uram_din;

  state <= CHECK_SPIKE;

Clock cycle M+5:
  if (spike) begin
    // Send spike to spike_fifo_controller
    spike_fifo_wr_en <= 1;
    spike_fifo_din <= {bank_id[3:0], local_addr[12:0]};
      // 17-bit global address: [16:13]=bank, [12:0]=local
  end

  state <= IDLE;  // Done processing this synapse
```

### Step 9: Spike Output (Back to Host)
```
spike_fifo_controller (if spike occurred):
  Collects spikes from 8 FIFOs
  Assembles 512-bit packet:
    [511:496] = 0xEEEE (spike packet tag)
    [495:32]  = spike data (up to 14 spikes)
    [31:0]    = timestep counter

  Writes packet to Output FIFO

Output FIFO → pcie2fifos:
  When packet available:
    Generates PCIe Memory Write TLP
    Address: Host DMA buffer (pre-configured)
    Payload: 512-bit spike packet

  Host memory receives write
  FPGA sends MSI-X interrupt

dmadump.dma_dump_read():
  CPU interrupt handler triggered
  Reads data from DMA buffer
  Returns to Python as numpy array

fpga_controller.flush_spikes():
  Parses packet:
    tag = packet[511:496]
    if (tag == 0xEEEE):
      for each spike in packet:
        neuron_id = spike[22:6]
        spike_list.append(neuron_id)

  Returns spike_list to hs_api
```

This entire flow (Steps 1-9) completes in ~1-5 microseconds for simple networks, demonstrating the hardware's ability to process neural network timesteps at millisecond rates.
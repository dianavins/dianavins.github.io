---
title: "1.2 Host-FPGA Communication"
parent: "Chapter 1: Initializing the Network"
nav_order: 2
---

## 1.2 How the Network is Written to the FPGA

Now that we know *where* everything ends up, let's understand *how* it gets there. This requires understanding how the host computer communicates with the FPGA hardware.

### What is Host-FPGA Communication?

Think of the host (your computer running Python) and the FPGA (the specialized chip) as two separate computers that need to talk to each other. They communicate through a physical link called **PCIe** (Peripheral Component Interconnect Express).

#### Analogy: Sending Mail Between Buildings

Imagine two buildings (Host and FPGA) connected by a mail chute:

```
┌─────────────────────┐                           ┌─────────────────────┐
│   HOST BUILDING     │                           │   FPGA BUILDING     │
│                     │                           │                     │
│  Person (Python)    │      PCIe "Mail Chute"    │  Mailroom Worker    │
│  writes a letter:   │    ═══════════════════►   │  (pcie2fifos.v)     │
│  "Put 0x03E8 at     │                           │  reads letter       │
│   address 0x8000"   │                           │  and delivers to    │
│                     │                           │  Storage Room (HBM) │
└─────────────────────┘                           └─────────────────────┘
```

**Key differences from actual mail:**
1. **Speed:** PCIe sends "letters" (data packets) at ~14 GB/second
2. **Automation:** Software libraries handle packing/unpacking automatically
3. **Direct Memory Access:** FPGA can reach into Host's memory and grab data without Host CPU doing the work

#### The Physical Link: PCIe

**PCIe (Peripheral Component Interconnect Express)** is a high-speed serial communication standard.

**Physical layer:** 16 wires going from Host motherboard to FPGA card
- Each wire (lane) carries 8 Gigabits/second
- 16 lanes × 8 Gb/s = 128 Gb/s raw = ~14 GB/s usable bandwidth

**What travels on PCIe:** Packets called **TLPs (Transaction Layer Packets)**
- Each packet has: Header (address, command type) + Payload (data)
- Example packet: "Write 32 bytes of data to FPGA address 0xD000_0000"

**Two communication modes:**

1. **Host-Initiated (MMIO - Memory-Mapped I/O):**
   - Host sends packet: "Write this data to FPGA address X"
   - FPGA receives packet, stores data
   - Used for: Sending commands, small data transfers

2. **FPGA-Initiated (DMA - Direct Memory Access):**
   - FPGA sends packet: "Read data from Host address Y and send it to me"
   - Host memory responds with data
   - Used for: Large bulk transfers (like our HBM programming)

During network initialization, we use **DMA** heavily because we're transferring potentially megabytes of synapse data.

---

### The Software-Hardware Stack for Initialization

When `CRI_network(target="CRI")` is called, a multi-layer software and hardware stack springs into action:

```
Layer 7: User Code
┌────────────────────────────────────────────────────────────────┐
│ from hs_api import CRI_network                                 │
│ network = CRI_network(axons, connections, config, outputs,     │
│                       target="CRI")                             │
└────────────────┬───────────────────────────────────────────────┘
                 │ Calls ▼
Layer 6: hs_api Internals
┌────────────────▼───────────────────────────────────────────────┐
│ hs_api/api.py: CRI_network.__init__()                          │
│ - Validates network structure                                  │
│ - Creates connectome object                                    │
│ - Calls: from hs_bridge import network                         │
│ - Instantiates: self.CRI = network(...)                        │
└────────────────┬───────────────────────────────────────────────┘
                 │ Calls ▼
Layer 5: hs_bridge Network Class
┌────────────────▼───────────────────────────────────────────────┐
│ hs_bridge/network.py: network.__init__()                       │
│ - Calls compiler to generate HBM data                          │
│ - Calls controller to program FPGA                             │
└────────────────┬───────────────────────────────────────────────┘
                 │ Calls ▼
Layer 4: fpga_compiler (HBM Data Generation)
┌────────────────▼───────────────────────────────────────────────┐
│ hs_bridge/FPGA_Execution/fpga_compiler.py                      │
│ - create_axon_ptrs(): Builds axon pointer array                │
│ - create_neuron_ptrs(): Builds neuron pointer array            │
│ - create_synapses(): Builds synapse data array                 │
│ - Output: NumPy arrays ready for DMA transfer                  │
└────────────────┬───────────────────────────────────────────────┘
                 │ Calls ▼
Layer 3: fpga_controller (FPGA Programming)
┌────────────────▼───────────────────────────────────────────────┐
│ hs_bridge/FPGA_Execution/fpga_controller.py                    │
│ - write_parameters_simple(): Programs neuron counts            │
│ - write_neuron_type(): Programs neuron model parameters        │
│ - clear(): Zeros URAM                                          │
│ - [Calls dmadump to transfer HBM data]                         │
└────────────────┬───────────────────────────────────────────────┘
                 │ Calls ▼
Layer 2: DMA Library (PCIe Transfer)
┌────────────────▼───────────────────────────────────────────────┐
│ hs_bridge/wrapped_dmadump/dmadump.py                           │
│ - dma_dump_write(data, length, ...): Sends data Host→FPGA      │
│ - Underlying C library interfaces with Linux kernel driver     │
└────────────────┬───────────────────────────────────────────────┘
                 │ PCIe TLPs ▼
Layer 1: FPGA Hardware Modules
┌────────────────▼───────────────────────────────────────────────┐
│ Verilog Modules (synthesized into FPGA fabric):                │
│ - pcie2fifos.v: Receives PCIe packets → Input FIFO             │
│ - command_interpreter.v: Parses commands, routes data          │
│ - hbm_processor.v: Writes data to HBM                          │
│ - internal_events_processor.v: Writes data to URAM             │
└─────────────────────────────────────────────────────────────────┘
```

---

### Step-by-Step: Initialization Sequence

Let's trace the exact sequence of events when you run:
```python
network = CRI_network(axons, connections, config, outputs, target="CRI")
```

#### **Phase 1: Network Compilation (Software - hs_bridge)**

**Step 1.1: CRI_network.__init__() validates and calls compiler**

File: `hs_api/api.py` (lines 141-156)
```python
if self.target == "CRI":
    logging.info("Initilizing to run on hardware")
    self.connectome.pad_models()
    formatedOutputs = self.connectome.get_outputs_idx()
    print("formatedOutputs:", formatedOutputs)
    self.CRI = network(  # ← Calls hs_bridge.network class
        self.connectome,
        formatedOutputs,
        self.config,
        simDump=simDump,
        coreOveride=coreID,
    )
    self.CRI.initalize_network()  # ← Triggers actual initialization
```

**Step 1.2: hs_bridge network class creates compiler**

File: `hs_bridge/network.py` (conceptual - not shown in our files, but referenced)
```python
def initalize_network(self):
    # Create compiler
    compiler = fpga_compiler(
        data=[self.axon_ptrs, self.neuron_ptrs, self.synapses],
        N_neurons=self.N_neurons,
        outputs=self.outputs,
        coreID=self.coreID
    )

    # Generate HBM programming data
    compiler.create_axon_ptrs()    # ← Generate axon pointer data
    compiler.create_neuron_ptrs()  # ← Generate neuron pointer data
    compiler.create_synapses()     # ← Generate synapse data

    # Program FPGA
    self.program_fpga()
```

**Step 1.3: fpga_compiler generates axon pointers**

File: `hs_bridge/FPGA_Execution/fpga_compiler.py` (lines 157-200)
```python
def create_axon_ptrs(self, simDump=False):
    '''Creates the necessary adxdma_dump commands to program axon pointers into HBM'''

    axn_ptrs = np.fliplr(self.axon_ptrs)  # Reverse for little-endian
    batchCmd = []

    for r, d in enumerate(axn_ptrs):  # For each row
        cmd = []
        for p in d:  # For each pointer in row (8 pointers per row)
            # p = (start_row, end_row) tuple
            # Build 32-bit pointer: [31:23]=length, [22:0]=start_address
            binAddr = np.binary_repr(p[1] - p[0], PTR_LEN_BITS) + \
                      np.binary_repr(p[0] + SYN_BASE_ADDR, PTR_ADDR_BITS)
            # binAddr is now 32-bit string like "000000001" + "00000000000000000000000"

            # Convert to bytes (4 bytes per pointer)
            cmd = cmd + [int(binAddr[:8], 2),    # Byte 0
                         int(binAddr[8:16], 2),   # Byte 1
                         int(binAddr[16:24], 2),  # Byte 2
                         int(binAddr[24:], 2)]    # Byte 3

        # Prepend HBM write command header
        # [511:504]=0x02 (HBM write opcode)
        # [503:496]=coreID
        # [495:0]=address + data
        rowAddress = '1' + np.binary_repr(r + AXN_BASE_ADDR, 23)  # 24-bit HBM address
        cmd = self.HBM_OP_RW_LIST + \
              [int(rowAddress[:8], 2),
               int(rowAddress[8:16], 2),
               int(rowAddress[16:], 2)] + cmd

        cmd.reverse()  # Reverse for endianness
        batchCmd = batchCmd + cmd

    # Send to FPGA via DMA
    exitCode = dmadump.dma_dump_write(np.array(batchCmd), len(batchCmd),
                                       1, 0, 0, 0, dmadump.DmaMethodNormal)
```

**What's happening here:**
- `self.axon_ptrs` is a NumPy array: `[[start0, end0], [start1, end1], ...]`
- For our network, axon 0: `[0, 0]` → length=1, start=0
- Converts to binary format: 9 bits for length + 23 bits for address
- Adds HBM write command opcode (0x02)
- Calls `dmadump.dma_dump_write()` to send via PCIe

**Example for Axon 0 pointer:**
```python
p = (0, 0)  # Start row 0, end row 0 (1 row total)
length = 0 - 0 = 0... wait, that's wrong!
# Actually the code does p[1] - p[0] but these are (start, end) inclusive
# So if start=0, end=0, that means 1 row (from 0 to 0 inclusive)
# But the binary repr treats it as end - start = 0
# Actually looking closer, length = p[1] - p[0] = end - start
# If there's 1 row, and we use inclusive indexing, end would equal start
# So length = 0... but we want to represent "1 row"
#
# Let me re-read: PTR_LEN_BITS = 9, stores number of rows
# The pointer stores: how many rows of synapses
# For axon 0 with 5 synapses, that fits in 1 row (8 synapses per row)
# So length should be 1
#
# Looking at line 176: binAddr = np.binary_repr(p[1] - p[0], PTR_LEN_BITS)
# If p = (start_row, end_row) and there's 1 row:
#   If 0-indexed and end is exclusive: p = (0, 1) → 1 - 0 = 1 ✓
#   If 0-indexed and end is inclusive: p = (0, 0) → 0 - 0 = 0 ✗
# The code must use exclusive end indexing
# So for axon 0: p = (0, 1) meaning rows [0, 1) = row 0
#
# Correcting:
p = (0, 1)  # Start row 0, end row 1 (exclusive) = 1 row
length = 1 - 0 = 1  # Binary: 0b000000001 (9 bits)
start = 0 + SYN_BASE_ADDR = 0 + 0x8000 = 0x8000  # Binary: 23 bits
binAddr = "000000001" + "00000000000001000000000"  # 32 bits total
        = 0b00000000100000000000001000000000
        = 0x0080_0000

Bytes: [0x00, 0x80, 0x00, 0x00]  (little-endian order in array)
```

**Step 1.4: fpga_compiler generates neuron pointers**

File: `hs_bridge/FPGA_Execution/fpga_compiler.py` (lines 225-268)

Same process as axon pointers, but for `self.neuron_ptrs` array. Writes to HBM starting at `NRN_BASE_ADDR = 0x4000`.

**Step 1.5: fpga_compiler generates synapses**

File: `hs_bridge/FPGA_Execution/fpga_compiler.py` (lines 271-360)
```python
def create_synapses(self, simDump=False):
    weights = self.synapses  # 2D array: rows × 8 synapses per row
    bigCmdList = []

    for r, d in enumerate(weights):  # For each synapse row
        cmd = []
        for w in d:  # For each synapse in row (up to 8)
            if w[0] == 0:  # Regular synapse
                # w = (opcode, target_address, weight)
                # Build 32-bit synapse: [31:29]=op, [28:16]=addr, [15:0]=weight
                binCmd = np.binary_repr(0, SYN_OP_BITS) + \
                         np.binary_repr(int(w[1]), SYN_ADDR_BITS) + \
                         np.binary_repr(int(w[2]), SYN_WEIGHT_BITS)
                # Example: op=0 (3 bits), addr=0 (13 bits), weight=1000 (16 bits)
                # binCmd = "000" + "0000000000000" + "0000001111101000"
                #        = 0b000_0000000000000_0000001111101000
                #        = 0x0000_03E8

                cmd = cmd + [int(binCmd[:8], 2),
                             int(binCmd[8:16], 2),
                             int(binCmd[16:24], 2),
                             int(binCmd[24:], 2)]

            elif w[0] == 1:  # Spike output entry
                # w = (1, neuron_index)
                binSpike = np.binary_repr(4, SYN_OP_BITS) + \
                           12*'0' + \
                           np.binary_repr(w[1], 17)
                # OpCode=100 (4 in decimal), address=neuron index, weight=0
                cmd = cmd + [int(binSpike[:8], 2),
                             int(binSpike[8:16], 2),
                             int(binSpike[16:24], 2),
                             int(binSpike[24:], 2)]

        # Prepend HBM write command
        rowAddress = '1' + np.binary_repr(r + SYN_BASE_ADDR, 23)
        cmd = self.HBM_OP_RW_LIST + \
              [int(rowAddress[:8], 2),
               int(rowAddress[8:16], 2),
               int(rowAddress[16:], 2)] + cmd

        cmd = np.flip(np.array(cmd, dtype=np.uint64))
        bigCmdList.append(cmd)

    # Send to FPGA in batches
    split = np.concatenate(bigCmdList)
    n = 10  # Batch size
    while True:
        element = split[:n*64]
        split = split[n*64:]
        if element.size == 0:
            break
        exitCode = dmadump.dma_dump_write(element, len(element),
                                           1, 0, 0, 0, dmadump.DmaMethodNormal)
```

**Example for first synapse (a0 → h0, weight=1000):**
```python
w = (0, 0, 1000)  # (opcode=0, target=h0=0, weight=1000)
binCmd = "000" + "0000000000000" + "0000001111101000"
       = 0x0000_03E8
Bytes: [0x00, 0x00, 0x03, 0xE8]
```

At this point, all HBM data is prepared as NumPy arrays. Now we need to send it!

---

#### **Phase 2: DMA Transfer (PCIe Communication)**

**Step 2.1: dmadump.dma_dump_write() prepares DMA**

File: `hs_bridge/wrapped_dmadump/dmadump.py` (Python wrapper for C library)
```python
def dma_dump_write(data, length, flag1, flag2, flag3, flag4, method):
    '''
    Sends data from host memory to FPGA via DMA

    Parameters:
    - data: NumPy array containing bytes to send
    - length: Number of bytes
    - method: DmaMethodNormal (0) for normal transfer

    Returns:
    - 0 on success, non-zero on error
    '''
    # This Python function calls a C extension
    # The C library handles:
    #   1. Allocating DMA-capable host memory buffer
    #   2. Copying 'data' into that buffer
    #   3. Getting physical address of buffer (for FPGA to read)
    #   4. Programming FPGA DMA registers via MMIO
    #   5. Waiting for DMA completion
```

**Step 2.2: Physical DMA operation**

What actually happens on the hardware:

```
1. Host allocates DMA buffer in RAM:
   Virtual address: 0x7FFF_1234_5000 (example)
   Physical address: 0x1_2345_6000 (translated by OS)
   Size: length bytes (e.g., 256 bytes for one HBM row)

2. Host copies data into DMA buffer:
   memcpy(dma_buffer, data, length)

3. Host writes to FPGA MMIO registers (via PCIe Memory Write TLP):
   PCIe Write to FPGA address 0xD000_0000 (example MMIO register):
     Value: 0x1_2345_6000 (physical address of buffer)

   PCIe Write to FPGA address 0xD000_0004:
     Value: 256 (length of transfer)

   PCIe Write to FPGA address 0xD000_0008:
     Value: 0x1 (start DMA, direction: read from host)

4. FPGA DMA engine (part of pcie2fifos.v) executes:
   - Reads descriptor from MMIO registers
   - Issues PCIe Memory Read TLP:
       Header: Read Request
       Address: 0x1_2345_6000 (host physical address)
       Length: 256 bytes

5. Host PCIe Root Complex receives read request:
   - Decodes address 0x1_2345_6000
   - Routes to memory controller
   - Memory controller reads from DDR4 SDRAM
   - Returns data in PCIe Completion TLP

6. FPGA receives Completion TLP:
   - Extracts 256 bytes of payload
   - Writes to Input FIFO (512-bit interface)
   - Asserts completion interrupt to host (MSI-X)
```

**PCIe Packet Example (Memory Read for DMA):**
```
TLP Header (16 bytes):
┌────────────────────────────────────────────────────────────┐
│ [127:125] Fmt = 001 (Memory Read, 64-bit address)         │
│ [124:120] Type = 00000 (Memory Read)                       │
│ [119:110] Length = 64 DW (256 bytes = 64 dwords)           │
│ [109:96]  Requester ID = 01:00.0 (Bus:Dev.Func of FPGA)   │
│ [95:88]   Tag = 5 (identifies this transaction)            │
│ [87:64]   Address[63:32] = 0x0000_0001 (upper 32 bits)     │
│ [63:2]    Address[31:2] = 0x2345_6000 >> 2 (lower 30 bits) │
│ [1:0]     Reserved                                          │
└────────────────────────────────────────────────────────────┘

No payload (this is a read request)

CRC (4 bytes): 0x12345678 (example)
```

**PCIe Completion Packet (Host's response):**
```
TLP Header:
- Fmt/Type = Completion with Data
- Completer ID = 00:00.0 (Host memory controller)
- Tag = 5 (matches request)
- Byte Count = 256

Payload (256 bytes):
  [First 64 bytes = HBM write command header + first pointers]
  0x02 0x00 ... (command data from dmadump array)

CRC: 0xABCDEF01
```

---

#### **Phase 3: FPGA Reception and HBM Programming**

**Step 3.1: pcie2fifos.v receives packet**

File: `hardware_code/gopa/CRI_proj/pcie2fifos.v` (Verilog, conceptual)

```verilog
// PCIe Endpoint IP presents AXI4 Write transaction
always @(posedge aclk) begin
    if (s_axi_wvalid && s_axi_wready) begin
        // Received 512-bit word from PCIe
        input_fifo_wr_en <= 1'b1;
        input_fifo_din <= s_axi_wdata[511:0];
    end
end

// Input FIFO stores data
// (Xilinx FIFO primitive handles this automatically)
```

**What's happening physically:**
- AXI4 bus has 512 wires for `s_axi_wdata`
- On clock rising edge where both `wvalid=1` and `wready=1`, data transfers
- `input_fifo_wr_en` signal triggers FIFO write
- FIFO is a BRAM primitive that stores the 512-bit word
- FIFO write pointer increments, `empty` flag deasserts

**Step 3.2: command_interpreter.v parses command**

File: `hardware_code/gopa/CRI_proj/command_interpreter.v`

```verilog
// State machine (simplified)
reg [2:0] state;
localparam IDLE = 0, READ_CMD = 1, ROUTE_DATA = 2;

always @(posedge aclk) begin
    case (state)
        IDLE: begin
            if (!input_fifo_empty) begin
                input_fifo_rd_en <= 1'b1;
                state <= READ_CMD;
            end
        end

        READ_CMD: begin
            // FIFO output valid (FWFT mode)
            cmd_word <= input_fifo_dout[511:0];
            opcode <= input_fifo_dout[511:504];  // Top 8 bits
            coreID <= input_fifo_dout[503:496];  // Next 8 bits
            payload <= input_fifo_dout[495:0];   // Remaining 496 bits
            state <= ROUTE_DATA;
        end

        ROUTE_DATA: begin
            case (opcode)
                8'h02: begin  // HBM write command
                    // Extract HBM address from payload
                    hbm_addr <= payload[495:472];  // 24-bit address
                    hbm_data <= payload[255:0];    // 256-bit data
                    hbm_wr_en <= 1'b1;
                    // Signal hbm_processor to write
                end

                8'h03: begin  // Clear URAM command
                    // Extract neuron address
                    // Signal internal_events_processor
                end

                8'h04: begin  // Network parameters
                    // Extract n_inputs, n_outputs
                    // Store in registers
                end

                // ... other opcodes
            endcase
            state <= IDLE;
        end
    endcase
end
```

**For our HBM write (opcode 0x02):**
```
Input: 512-bit word from Input FIFO

Bits [511:504] = 0x02 → opcode = HBM write
Bits [503:496] = 0x00 → coreID = 0
Bits [495:472] = 24-bit HBM row address
  Example: 0x800000 = row 0 in axon pointer region
Bits [471:0] = HBM data (256 bits of actual pointers/synapses + padding)

Command interpreter extracts:
  hbm_addr = 0x000000 (row address, relative to base)
  hbm_data[255:0] = pointer data

Asserts hbm_wr_en signal to hbm_processor
```

**Step 3.3: hbm_processor.v writes to HBM**

File: `hardware_code/gopa/CRI_proj/hbm_processor.v`

```verilog
// HBM write state machine (simplified)
reg [2:0] hbm_state;
localparam HBM_IDLE = 0, HBM_WRITE_ADDR = 1, HBM_WRITE_DATA = 2;

always @(posedge aclk) begin
    case (hbm_state)
        HBM_IDLE: begin
            if (hbm_wr_en) begin
                // Received write request from command_interpreter
                hbm_wr_addr_reg <= hbm_addr;
                hbm_wr_data_reg <= hbm_data;
                hbm_state <= HBM_WRITE_ADDR;
            end
        end

        HBM_WRITE_ADDR: begin
            // AXI4 Write Address Channel
            m_axi_awvalid <= 1'b1;
            m_axi_awaddr <= {hbm_wr_addr_reg, 5'b00000};  // Convert row to byte addr
            m_axi_awlen <= 8'd0;   // 1 beat
            m_axi_awsize <= 3'd5;  // 32 bytes = 2^5

            if (m_axi_awready) begin
                m_axi_awvalid <= 1'b0;
                hbm_state <= HBM_WRITE_DATA;
            end
        end

        HBM_WRITE_DATA: begin
            // AXI4 Write Data Channel
            m_axi_wvalid <= 1'b1;
            m_axi_wdata <= {256'b0, hbm_wr_data_reg};  // Pad to 512 bits (HBM bus width)
            m_axi_wstrb <= 64'hFFFFFFFF;  // All bytes valid
            m_axi_wlast <= 1'b1;           // Last beat

            if (m_axi_wready) begin
                m_axi_wvalid <= 1'b0;
                hbm_state <= HBM_IDLE;
                // Write complete
            end
        end
    endcase
end
```

**What's happening physically:**
- `m_axi_awaddr` is a 33-bit wire bus to HBM controller
- When `awvalid=1` and HBM controller asserts `awready=1`, address transfers
- Next cycle: `wdata[511:0]` bus carries 512 bits (256 bits of data + 256 bits padding)
- HBM controller decodes address: stack, channel, bank, row, column
- HBM performs DRAM write:
  1. Activate row (if different row than last access)
  2. Write data to sense amplifiers
  3. Precharge (close row)
- Takes ~100-200ns total
- `wready` asserts when HBM controller accepts data

**Step 3.4: HBM physically stores the data**

Inside the HBM chip (physical DRAM operation):

```
Address decoding:
  33-bit address 0x0_0100_0000 (example for row 0x8000 × 32 bytes)

  [32:30] Stack select = 0b000 → Stack 0
  [29:27] Channel select = 0b000 → Channel 0 within stack
  [26:13] Row address = 0b00000000100000 → Row 0x0020
  [12:5]  Column address = 0b00000000 → Column 0
  [4:0]   Byte offset = 0b00000 → Byte 0

HBM controller sequence:
  1. Activate command: Open row 0x0020 in Bank 0
     - Wordline voltage applied
     - Entire row (512 bytes) read into sense amps (row buffer)

  2. Write command: Write 32 bytes at column 0
     - Drive bitlines with new data
     - Sense amps latch data
     - Capacitors in DRAM cells charge/discharge

  3. Precharge command: Close row
     - Write data from sense amps back to cells
     - Wordline deasserted

  4. Data now stored in DRAM cells (1 transistor + 1 capacitor per bit)
     - Will persist for ~64ms before refresh needed
```

---

#### **Phase 4: Additional Initialization Steps**

**Step 4.1: Program network parameters**

File: `hs_bridge/FPGA_Execution/fpga_controller.py:683-721`
```python
def write_parameters_simple(n_outputs, n_inputs, coreID=0, simDump=False):
    """Writes the network parameters to the FPGA"""
    command = np.zeros(512)
    command[:8] = list(np.binary_repr(4, 8))      # Opcode 0x04
    command[8:16] = list(np.binary_repr(coreID, 8))
    command[-17:] = list(np.binary_repr(n_inputs, 17))   # 17-bit input count
    command[-34:-17] = list(np.binary_repr(n_outputs, 17)) # 17-bit output count
    command = to_dump_format(command)  # Convert to byte array

    exitCode = dmadump.dma_dump_write(command, len(command), ...)
```

This sends a command to `internal_events_processor.v` telling it:
- How many input axons exist (5 in our network)
- How many output neurons exist (5 in our network)

**Step 4.2: Program neuron types**

File: `hs_bridge/FPGA_Execution/fpga_controller.py:724-775`
```python
def write_neuron_type(stopAddr, Threshold, neuronModel, shift, leak, coreID=0):
    """Configures neuron model parameters"""
    command = np.zeros(512)
    command[:8] = list(np.binary_repr(8, 8))      # Opcode 0x08
    command[8:16] = list(np.binary_repr(coreID, 8))
    command[-34:-17] = list(np.binary_repr(stopAddr, 17))     # Last neuron index
    command[-70:-34] = list(np.binary_repr(Threshold, 36))    # Spike threshold
    command[-72:-70] = list(np.binary_repr(neuronModel, 2))   # 0=IF, 1=LIF, etc.
    command[-78:-72] = list(np.binary_repr(shift, 6))         # Leak shift amount
    command[-84:-78] = list(np.binary_repr(leak, 6))          # Leak value
    command = to_dump_format(command)

    exitCode = dmadump.dma_dump_write(command, len(command), ...)
```

This configures:
- **Threshold = 2000**: Neurons spike when V ≥ 2000
- **Neuron model = LIF**: Leaky integrate-and-fire
- **Leak parameters**: How much voltage leaks each timestep

The FPGA stores these in internal registers, which `internal_events_processor.v` uses during neuron updates.

**Step 4.3: Clear URAM (zero all membrane potentials)**

File: `hs_bridge/FPGA_Execution/fpga_controller.py:191-236`
```python
def clear(n_internal, simDump=False, coreID=0):
    """This function clears the membrane potentials on the fpga."""
    coreBits = np.binary_repr(coreID, 5) + 3*'0'

    for i in range(int(np.ceil(n_internal / ng_num))):  # ng_num = 16 neurons/group
        commandTail = np.array([0]*55 + [int(coreBits, 2), 3], dtype=np.uint64)
        numCol = 16  # 16 columns (neuron groups)
        clearCommandList = []

        for column in range(numCol):
            # Build clear command for this neuron group
            clearCommandList.append(
                np.concatenate([clear_address_packet(row=i, col=column), commandTail])
            )

        clearCommand = np.concatenate(clearCommandList)
        exitCode = dmadump.dma_dump_write(clearCommand, len(clearCommand), ...)
```

This sends opcode 0x03 commands to `internal_events_processor.v`, which writes zeros to all URAM addresses.

**What happens in hardware:**
```verilog
// internal_events_processor.v receives clear command
always @(posedge aclk450) begin
    if (clear_cmd) begin
        // For each neuron in this group
        uram_addr <= neuron_row;
        uram_we <= 1'b1;
        uram_din <= 72'b0;  // Write all zeros
    end
end
```

This zeroes the membrane potential for all neurons. After this, every neuron starts with V=0.

---

### Summary: Complete Initialization Flow

```
User Python Code:
  network = CRI_network(target="CRI")
       ↓
hs_api validates network
       ↓
hs_bridge.network.__init__()
       ↓
fpga_compiler generates HBM data:
  - Axon pointers array
  - Neuron pointers array
  - Synapses array
       ↓
dmadump.dma_dump_write() sends data via PCIe:
  - Host allocates DMA buffer
  - FPGA reads from host memory
  - Data flows: Host RAM → PCIe → FPGA Input FIFO
       ↓
command_interpreter.v parses commands:
  - Opcode 0x02 → HBM write
  - Routes data to hbm_processor
       ↓
hbm_processor.v writes to HBM:
  - AXI4 transaction to HBM controller
  - Physical DRAM write (activate → write → precharge)
       ↓
fpga_controller.write_parameters_simple():
  - Sends opcode 0x04
  - Programs n_inputs, n_outputs
       ↓
fpga_controller.write_neuron_type():
  - Sends opcode 0x08
  - Programs threshold, neuron model, leak
       ↓
fpga_controller.clear():
  - Sends opcode 0x03
  - Zeros all URAM (membrane potentials)
       ↓
FPGA is now initialized:
  ✓ HBM contains network structure (pointers, synapses, weights)
  ✓ URAM cleared (all neurons at V=0)
  ✓ Network parameters programmed (threshold, neuron model)
  ✓ Ready to receive inputs and execute
```

**Time elapsed:** Typically 10-100 milliseconds depending on network size
- Small network (our example): ~10 ms
- Large network (millions of synapses): ~100 ms
- Dominated by PCIe transfer time for large synapse arrays

---

## Conclusion

Network initialization is a **one-time compilation and transfer process** that transforms your high-level Python network definition into a physical configuration in the FPGA's memory hierarchy. Once initialized:

- **HBM stores the network structure** (connections and weights) - this doesn't change during execution
- **URAM stores neuron states** (membrane potentials) - this updates every timestep
- **BRAM stores input patterns** (which axons are firing) - this changes every timestep

In the next chapter, we'll see how this initialized network comes to life when we send inputs and execute timesteps.

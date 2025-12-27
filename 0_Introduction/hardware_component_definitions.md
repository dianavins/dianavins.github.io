---
title: 0.3 Hardware Component Definitions
nav_order: 3
parent: Introduction
---

# Hardware Component Definitions (Low-Level)

### **Host DDR4 SDRAM (System Memory)**
**What it is:** Dynamic Random Access Memory using capacitor charge storage
- **Physical cell:** 1 transistor + 1 capacitor per bit (1T1C DRAM cell)
- **Capacitor charge:** ~30 femtofarads, holds ~10,000 electrons
- **Refresh requirement:** Every 64ms (charge leaks), handled by memory controller
- **Organization:**
  - DIMMs (Dual Inline Memory Modules) - physical sticks
  - Each DIMM: 8-16 chips (ranks), each chip has 8 banks
  - Bank: Array of rows × columns (e.g., 65K rows × 1K columns)
  - Row activation: Reads entire row (8KB) into row buffer
  - Column access: Selects bytes from row buffer
- **Access latency:** ~60ns for row activation + column read
- **Bandwidth:** 64-bit bus × 3200 MT/s (DDR4-3200) = 25.6 GB/s per channel
- **hs_bridge usage:** Stores DMA buffers for PCIe transfers
  - `fpga_compiler` writes command arrays here
  - `dmadump.dma_dump_write()` provides address to FPGA via MMIO
  - FPGA reads directly from this memory (DMA bus master)

### **PCIe (Peripheral Component Interconnect Express)**
**What it is:** High-speed serial point-to-point interconnect using differential signaling

**Physical Layer:**
- **Lane composition:** 4 wires per lane (TX+, TX-, RX+, RX-)
  - Differential pair: Voltage difference between + and - carries signal
  - Noise immunity: Common-mode noise cancels out in differential
- **Signaling:** 8 Gb/s per lane (Gen3), uses 128b/130b encoding
  - 128 data bits + 2 sync bits = 130 transmitted bits
  - Efficiency: 128/130 = 98.5%
- **x16 configuration:** 16 independent lanes = 128 Gb/s raw = ~15.75 GB/s effective
- **Clocking:** Each lane has embedded clock (recovered from data transitions)
  - REFCLK (100 MHz) used for initial synchronization only
  - Data transitions provide timing (no separate clock wires)

**Link Layer:**
- **Framing:** Packets have Start (STP), End (END), and CRC-32 checksum
- **Flow control:** Credits system prevents buffer overflow
  - Receiver advertises available buffer space (credits)
  - Transmitter only sends when has credits
- **Reliability:** ACK/NAK for every packet, automatic retry on CRC error
- **Sequence numbers:** Detect lost packets

**Transaction Layer (what software sees):**
- **TLP (Transaction Layer Packet) structure:**
  ```
  [127:96]  Header Word 0: Format[7:5], Type[4:0], Length[9:0], ...
  [95:64]   Header Word 1: Requester ID, Tag, ...
  [63:0]    Header Word 2-3: Address (64-bit)
  [...]     Payload: 0 to 4096 bytes (typically 64-byte cache lines)
  ```
- **Transaction types:**
  - **Memory Write (Posted):** Host→FPGA, no response expected
    - Used by: `fpga_controller` sending commands
    - Address: Mapped to FPGA MMIO space (e.g., 0xD000_0000)
    - FPGA PCIe block decodes address → routes to pcie2fifos
  - **Memory Read:** FPGA→Host memory, response required
    - Used by: FPGA during DMA (reading descriptor, buffers)
    - FPGA presents host physical address (translated by IOMMU)
    - Host memory controller responds with Completion TLP
  - **MSI-X Interrupt:** FPGA→Host, special write to interrupt vector
    - FPGA writes to pre-configured address
    - CPU interrupt controller receives, triggers interrupt handler

**DMA (Direct Memory Access):**
- **What it is:** Allows FPGA to read/write host memory without CPU involvement
- **Setup (by hs_bridge):**
  1. CPU allocates buffer in system memory (malloc/mmap)
  2. CPU gets physical address (via IOMMU page tables)
  3. CPU writes descriptor to FPGA MMIO:
     - Buffer physical address
     - Transfer length
     - Direction (read/write)
     - Flags (interrupt on completion, etc.)
  4. CPU writes "go" bit to FPGA DMA control register
- **Execution (by FPGA):**
  1. FPGA reads descriptor (Memory Read TLP to host)
  2. FPGA reads source data (if host→FPGA direction)
     - Issues burst of Memory Read TLPs
     - Receives Completions with data
  3. FPGA writes data to destination (Memory Write TLPs)
  4. FPGA writes completion status (Memory Write TLP to status address)
  5. FPGA sends interrupt (MSI-X write)
- **Performance:** Bypasses CPU cache, uses host memory bandwidth directly
  - PCIe Gen3 x16: ~14 GB/s practical (accounting for overhead)
  - `dmadump` library: Batches transfers to amortize overhead

### **FPGA (Xilinx XCVU37p - Field-Programmable Gate Array)**
**What it is:** Silicon chip with reconfigurable logic fabric

**Physical Structure:**
- **Technology:** 20nm FinFET (Fin Field-Effect Transistor) CMOS process
- **Die size:** ~800 mm² (large chip, expensive manufacturing)
- **Power:** ~50-100W typical (varies with clock frequency, utilization)

**Configurable Logic Blocks (CLBs):**
- **Hierarchy:** FPGA fabric = array of CLBs + routing resources
- **CLB composition:** Each CLB has 8 LUTs + 16 Flip-Flops + carry logic
- **LUT (Look-Up Table):**
  - **Physical:** 64-bit SRAM (6 address bits = 2^6 = 64 entries)
  - **Function:** Implements any 6-input Boolean function
  - **Example:** AND gate: LUT[0b000000]=0, LUT[0b111111]=1, others=0
  - **Programming:** SRAM bits loaded from bitstream at configuration
- **Flip-Flop (FF):**
  - **Physical:** D-type register (master-slave latch pair)
  - **Function:** Stores 1 bit, updates on clock edge
  - **Inputs:** D (data), CLK (clock), CE (clock enable), RST (reset)
  - **Operation:** On rising edge of CLK: Q <= D (if CE=1)
- **Synthesis:** Verilog code → logic gates → mapped to LUTs+FFs
  - Combinational logic → LUTs
  - Registers (always @(posedge clk)) → FFs
  - Example: `assign out = a & b;` → 2-input LUT programmed as AND

**Routing Resources:**
- **Programmable interconnect:** Switches connecting CLB outputs to inputs
- **Switch matrix:** Crossbar at each routing junction
  - **Physical:** Transistor switches (pass gates) controlled by SRAM bits
  - **Configuration:** SRAM bits set which connections are active
- **Routing delay:** ~0.5-2ns depending on distance (wire length)
- **Timing closure:** Router tries to meet 225MHz (4.4ns period) constraints
  - Critical path: Longest combinational delay + routing delay < 4.4ns
  - If fails: Insert pipeline registers (adds latency but meets timing)

**Clock Distribution:**
- **Global clock tree:** H-tree topology (balanced routing to all CLBs)
  - Ensures all FFs see clock edge within ~100ps skew
- **Clock buffers:** BUFG primitives (dedicated high-fanout buffers)
  - Drive thousands of FFs with minimal skew
- **PLLs/MMCMs:** Generate 225 MHz and 450 MHz from 100 MHz reference
  - **PLL:** Phase-Locked Loop, tracks input frequency, generates multiples
  - **VCO:** Voltage-Controlled Oscillator inside PLL, runs at high freq (900-2000 MHz)
  - **Dividers:** Divide VCO down to desired output frequencies

**Block RAM (BRAM):**
- **Physical:** Dedicated SRAM blocks (not part of CLB fabric)
- **Technology:** 6-transistor SRAM cell (2 inverters + 2 access transistors)
  - Unlike DRAM, no refresh needed (static storage)
- **RAMB36E2 primitive:** 36 Kb per block
  - Configurable width: 1-72 bits
  - Depth adjusts inversely: 36K×1, 18K×2, ..., 512×72
- **Dual-port:** Supports simultaneous read/write on independent ports
- **Access:** Synchronous, 2-3 cycle latency depending on mode
  - Cycle 0: Present address
  - Cycle 1: Internal row decode
  - Cycle 2: Data valid on output
- **Our usage:** 32,768 × 256-bit (uses 256 RAMB36 primitives)
  - Actually configured as 256 blocks of 1K×256, then address mapped

**UltraRAM (URAM):**
- **Physical:** Dedicated memory blocks (like BRAM but higher density)
- **Technology:** 1T1C DRAM-like cell (but on-chip, no external bus)
  - Similar to HBM cells but integrated into FPGA die
  - Advantage: 4× density vs BRAM (36Kb → 288Kb per primitive)
  - Tradeoff: Requires refresh (built into primitive logic)
- **URAM288 primitive:** 288 Kb per block
  - Configured as 4096 × 72 bits for our design
- **Access:** Synchronous, 1 cycle read latency @ 450 MHz
  - Cycle N: Address presented
  - Cycle N+1: Data valid (faster than BRAM due to design)
- **Refresh:** Automatic, handled by primitive (transparent to user logic)
- **Our usage:** 16 banks × 288 Kb = 4.5 Mb total for neuron states

**Hard IP Blocks:**
- **PCIe block:** Dedicated silicon (not programmable fabric)
  - Location: Fixed position on die corner
  - Contains: SerDes (serializer/deserializer), PHY, MAC layers
  - Advantage: Higher performance, lower resource usage than soft IP
- **HBM interface:** Dedicated controllers for HBM2 protocol
  - 32 independent AXI ports (one per HBM channel)
  - Hardened logic for timing-critical signaling

### **HBM2 (High Bandwidth Memory)**
**What it is:** Stacked DRAM with wide internal buses for extreme bandwidth

**Physical Structure:**
- **3D stacking:** 4 DRAM dies stacked vertically
  - Each die: 512 Mb (64 MB) × 8 layers = 2 GB per stack
  - 4 stacks total = 8 GB capacity
- **Through-Silicon Vias (TSVs):**
  - Vertical conductors drilled through die thickness (~50 µm diameter)
  - Connect dies in stack: Data buses, power, ground
  - Advantage: Very short distance = low latency, high bandwidth
- **Silicon Interposer:**
  - Large (~1000 mm²) silicon substrate under HBM + FPGA
  - Microbumps (~50 µm pitch) connect HBM→interposer, FPGA→interposer
  - Advantage: Much higher density than PCB routing

**Why "High Bandwidth"?**
- **Wide buses:** Each HBM stack has 1024-bit interface
  - 8 channels × 128-bit per channel = 1024 bits total per stack
  - Compare DDR4: 64-bit bus (16× narrower)
- **High frequency:** 900 MHz DDR (1800 MT/s)
  - DDR: Data on both clock edges (double data rate)
- **Bandwidth calculation:**
  - Per stack: 1024 bits × 1800 MT/s = 1.8 Tb/s = 230 GB/s
  - 4 stacks: 230 × 4 = 920 GB/s theoretical peak
  - Practical (our system): ~400 GB/s sustained (accounting for overhead)

**DRAM Cell (basic storage element):**
- **Structure:** 1 transistor + 1 capacitor (1T1C)
  - Capacitor: Stores charge (~30 fF, ~10K electrons)
  - Transistor: Access gate (connects capacitor to bitline)
- **Write operation:**
  1. Activate wordline (turns on access transistor)
  2. Drive bitline to VDD (logic 1) or GND (logic 0)
  3. Capacitor charges/discharges through transistor
  4. Deactivate wordline (isolates capacitor)
- **Read operation (destructive):**
  1. Precharge bitline to VDD/2
  2. Activate wordline (connect capacitor to bitline)
  3. Capacitor shares charge with bitline
  4. Sense amplifier detects small voltage change on bitline
  5. Restore: Write value back to capacitor (read is destructive)
- **Refresh:** Every 64ms, all rows read and written back
  - Needed because capacitor leaks (tunneling current through dielectric)

**Memory Organization:**
- **Hierarchy:** Stack → Channel → Pseudo-Channel → Bank → Row → Column
- **Stack:** One of 4 physical DRAM stacks
- **Channel:** One of 8 independent 128-bit interfaces per stack
- **Bank:** One of 16 banks per channel (allows interleaved access)
- **Row:** 16,384 rows per bank
- **Column:** 1024 columns per row
- **Row buffer:** When row activated, entire row (512 bytes) read into buffer
  - Subsequent column accesses hit row buffer (fast, ~10ns)
  - Different row access: Must close old row, open new row (slow, ~50ns)
  - "Page hit" vs "page miss" performance difference

**AXI4 Interface:**
- **Protocol:** ARM Advanced eXtensible Interface
- **Channels:** 5 independent channels (all parallel):
  1. **Write Address (AW):** Master sends address + metadata
  2. **Write Data (W):** Master sends data payload
  3. **Write Response (B):** Slave acknowledges completion
  4. **Read Address (AR):** Master requests data
  5. **Read Data (R):** Slave returns data
- **Decoupling:** Address and data can be sent independently
  - Example: Send 4 read addresses, then receive 4 data responses
  - Allows pipelining and out-of-order completion
- **Handshake:** Every channel uses VALID/READY protocol
  - Source asserts VALID (data is stable)
  - Destination asserts READY (can accept data)
  - Transfer occurs when both are high (combinational AND gate)
- **Bursts:** Single address can request multiple beats (up to 256)
  - Address + Length → HBM returns sequence of data
  - Amortizes address overhead

**Access Latency Breakdown:**
- **Best case (row hit):** ~50ns
  - Address decode: 5ns
  - Column select: 10ns
  - Sense amp: 10ns
  - Data serialization: 10ns
  - AXI handshake: 15ns
- **Worst case (row miss):** ~200ns
  - Precharge old row: 30ns
  - Activate new row: 50ns
  - Column access: 50ns
  - Rest as above
- **Optimization in hbm_processor.v:** Prefetch next row during processing
  - Hides some latency with pipelining

### **FIFO (First-In-First-Out Buffer)**
**What it is:** Queue implemented in hardware for asynchronous data transfer

**Physical Implementation (Xilinx FIFO36E2):**
- **Storage:** Uses BRAM36 primitive (36 Kb SRAM)
- **Pointers:** Write pointer (WP) and Read pointer (RP)
  - Both are counters (e.g., 9-bit for 512-entry FIFO)
  - Increment on write/read operations
- **Empty/Full logic:**
  - Empty: WP == RP (no data to read)
  - Full: (WP + 1) mod DEPTH == RP (no space to write)
- **FWFT mode (First-Word Fall-Through):**
  - Data available on DO port same cycle as EMPTY deasserts
  - No need to assert RD_EN and wait (zero-latency read)
  - Implemented with extra output register + bypass mux

**Clock Domain Crossing (Async FIFO):**
- **Problem:** Write side at 225 MHz, read side at 450 MHz
  - Can't directly compare pointers (in different clock domains)
- **Solution:** Gray code + 2-FF synchronizer
  - **Gray code:** Only 1 bit changes per increment
    - Binary 3→4: 011→100 (2 bits change - metastability hazard)
    - Gray 3→4: 010→110 (1 bit changes - safe to synchronize)
  - **Synchronizer:** 2 flip-flops in receiving clock domain
    ```verilog
    always @(posedge rd_clk) begin
      wptr_gray_sync1 <= wptr_gray; // May be metastable
      wptr_gray_sync2 <= wptr_gray_sync1; // Stable
    end
    ```
    - First FF can enter metastable state (voltage between 0 and 1)
    - Second FF settles to valid 0 or 1 (metastability resolves)
    - Timing: Allow 1 full clock period for metastability resolution
- **Empty calculation:** Done in read clock domain using synchronized write pointer
- **Full calculation:** Done in write clock domain using synchronized read pointer

**Our FIFOs:**
- **Input/Output FIFOs:** 512-bit width × 512 depth (PCIe ↔ fabric)
- **Pointer FIFOs:** 32-bit width × 512 depth (HBM data → neuron groups)
- **Spike FIFOs:** 17-bit width × 512 depth (neurons → spike controller)

---


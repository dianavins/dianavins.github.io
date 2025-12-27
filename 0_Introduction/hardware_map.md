---
title: Hardware Map
nav_order: 10
parent: Introduction
---


# Hardware Architecture Map - Low-Level Physical Organization

## Complete Physical Hardware Stack

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                         HOST COMPUTER (Physical Layer)                         │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │  CPU (AMD EPYC 7502 - 2 sockets)                                         │  │
│  │  - Clock: ~2.5 GHz base, 128 PCIe Gen4 lanes total                       │  │
│  │  - Generates: Memory requests, PCIe Transaction Layer Packets (TLPs)     │  │
│  └──────────────────────────┬───────────────────────────────────────────────┘  │
│                             │ DDR4 channels (8-channel per socket)             │
│  ┌──────────────────────────▼───────────────────────────────────────────────┐  │
│  │  System Memory (DDR4 SDRAM - 1 TB total)                                 │  │
│  │  - Physical: DRAM cells (capacitor + transistor per bit)                 │  │
│  │  - Organization: Ranks → DIMMs → Banks → Rows → Columns                  │  │
│  │  - Width: 64-bit data bus per channel                                    │  │
│  │  - Contains: DMA descriptor rings, buffers for PCIe transfers            │  │
│  │  - DMA Buffer Layout (written by hs_bridge):                             │  │
│  │    Address: 0x... [Host virtual, mapped to physical by IOMMU]            │  │
│  │    Content: 512-bit aligned command packets                              │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
│                                                                                 │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │  PCIe Root Complex (integrated in CPU chipset)                           │  │
│  │  - Function: Translates CPU memory transactions → PCIe packets           │  │
│  │  - DMA Operation: FPGA acts as Bus Master                                │  │
│  │    1. CPU writes descriptors to memory (buffer addr, length, flags)      │  │
│  │    2. CPU writes to FPGA MMIO register to trigger DMA                    │  │
│  │    3. FPGA reads descriptor from host memory (PCIe Memory Read)          │  │
│  │    4. FPGA reads/writes data buffers directly (bypassing CPU)            │  │
│  │    5. FPGA writes completion status to memory                            │  │
│  │    6. FPGA signals interrupt to CPU (MSI-X)                              │  │
│  └──────────────────────────┬───────────────────────────────────────────────┘  │
└─────────────────────────────┼────────────────────────────────────────────────────┘
                              │
                ══════════════╧══════════════
                  PCIe Gen3 x16 Physical Link
                  (How it actually works)
                ══════════════╤══════════════
                              │
  ┌─────────────────────────────────────────────────────────────────────┐
  │ PCIe Physical Layer (what's on the actual copper/fiber traces)      │
  │ - 16 differential pairs (lanes): TX+ TX- RX+ RX- per lane           │
  │ - Each lane: Independent 8 Gb/s serial stream (8b/10b encoded)      │
  │ - Aggregate: 16 lanes × 8 Gb/s = 128 Gb/s raw = ~16 GB/s usable     │
  │ - Signals: REFCLK (100 MHz reference), PERST# (reset)               │
  │                                                                      │
  │ PCIe Link Layer (packet framing):                                   │
  │ - Adds: Sequence numbers, ACKs/NAKs, CRC-32 for error detection     │
  │ - Retransmits corrupted packets automatically                       │
  │                                                                      │
  │ PCIe Transaction Layer (what software sees):                        │
  │ - Transaction Layer Packets (TLPs):                                 │
  │   ┌──────────────────────────────────────────────────────┐         │
  │   │ TLP Header (12-16 bytes)          │ Payload (0-4KB)  │         │
  │   ├───────────────────────────────────┼──────────────────┤         │
  │   │ Fmt/Type | Length | Requester ID  │ Data (512-bit    │         │
  │   │ Tag | Address (64-bit)             │  chunks for our  │         │
  │   │                                    │  design)         │         │
  │   └──────────────────────────────────────────────────────┘         │
  │ - Types used:                                                       │
  │   * Memory Write (Posted): Host → FPGA, no response needed          │
  │   * Memory Read: FPGA → Host memory during DMA                      │
  │   * Completion: FPGA → Host with requested data                     │
  │   * MSI-X Interrupt: FPGA → Host notification                       │
  └─────────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────▼────────────────────────────────────────────────┐
│                    FPGA BOARD (Alpha-Data ADM-PCIE-9H7)                      │
│                                                                               │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │  Xilinx XCVU37p FPGA Die                                              │  │
│  │  - Physical: 9M ASIC gates, 2586 CLBs, 2160 DSP slices               │  │
│  │  - Technology: 20nm FinFET process                                     │  │
│  │  - Area: ~800 mm² silicon                                             │  │
│  │  ┌──────────────────────────────────────────────────────────────────┐ │  │
│  │  │  Clock Generation (physical oscillators & PLLs)                  │ │  │
│  │  │  ┌────────────────────────────────────────────────────────────┐  │ │  │
│  │  │  │ Crystal Oscillator (external component on PCB)            │  │ │  │
│  │  │  │ - Frequency: 100 MHz                                       │  │ │  │
│  │  │  │ - Technology: Quartz crystal piezoelectric resonator       │  │ │  │
│  │  │  │ - Drives: PCIe REFCLK, base clock for PLLs                │  │ │  │
│  │  │  └──────────────┬─────────────────────────────────────────────┘  │ │  │
│  │  │                 │                                                 │ │  │
│  │  │  ┌──────────────▼─────────────────────────────────────────────┐  │ │  │
│  │  │  │ MMCM (Mixed-Mode Clock Manager) - PLL #1                   │  │ │  │
│  │  │  │ - Input: 100 MHz                                           │  │ │  │
│  │  │  │ - VCO (Voltage-Controlled Oscillator): 900-2000 MHz        │  │ │  │
│  │  │  │ - Output 1: 225 MHz (÷ N from VCO) → aclk                 │  │ │  │
│  │  │  │ - Output 2: 450 MHz (÷ M from VCO) → aclk450              │  │ │  │
│  │  │  │ - Phase alignment: Both outputs edge-aligned               │  │ │  │
│  │  │  │ - Jitter: <100 ps peak-to-peak                            │  │ │  │
│  │  │  │ - Distribution: Clock tree (H-tree topology in silicon)   │  │ │  │
│  │  │  └────────────────────────────────────────────────────────────┘  │ │  │
│  │  └──────────────────────────────────────────────────────────────────┘ │  │
│  │                                                                         │  │
│  │  ┌──────────────────────────────────────────────────────────────────┐ │  │
│  │  │  Configurable Logic Blocks (CLBs) - The Fabric                  │ │  │
│  │  │  - Unit: Each CLB contains 8 LUTs + 16 Flip-Flops               │ │  │
│  │  │  - LUT (Look-Up Table): 6-input, 64-bit SRAM implements logic  │ │  │
│  │  │  - Flip-Flop: D-type register, clocked by aclk or aclk450      │ │  │
│  │  │  - Routing: Programmable interconnect switches between CLBs    │ │  │
│  │  │  - Our modules exist as:                                        │ │  │
│  │  │    * Synthesized logic (LUTs implementing combinational logic) │ │  │
│  │  │    * Registers (FFs storing state machine states, counters)    │ │  │
│  │  │    * Routed wires (programmable connections)                   │ │  │
│  │  └──────────────────────────────────────────────────────────────────┘ │  │
│  │                                                                         │  │
│  │  ┌──────────────────────────────────────────────────────────────────┐ │  │
│  │  │  PCIe Hard Block (dedicated silicon, not FPGA fabric)           │ │  │
│  │  │  - Physical: Hardened PCIe Gen3 x16 endpoint                    │ │  │
│  │  │  - Location: Corner of FPGA die (fixed placement)               │ │  │
│  │  │  - Connects to: PCIe connector on board → motherboard slot      │ │  │
│  │  │  - Output to fabric: AXI4 Memory-Mapped interface               │ │  │
│  │  │    ┌────────────────────────────────────────────────────┐       │ │  │
│  │  │    │ AXI4 Interface (what leaves PCIe block)            │       │ │  │
│  │  │    │ Clock: aclk (225 MHz, derived from REFCLK)         │       │ │  │
│  │  │    │ Signals (each is a wire in FPGA routing):          │       │ │  │
│  │  │    │  Write Address Channel:                            │       │ │  │
│  │  │    │    - AWADDR[63:0]: 64-bit address                  │       │ │  │
│  │  │    │    - AWVALID: Source ready (1-bit)                 │       │ │  │
│  │  │    │    - AWREADY: Destination ready (1-bit)            │       │ │  │
│  │  │    │  Write Data Channel:                               │       │ │  │
│  │  │    │    - WDATA[511:0]: 512-bit payload                 │       │ │  │
│  │  │    │    - WSTRB[63:0]: Byte enable (which bytes valid)  │       │ │  │
│  │  │    │    - WVALID, WREADY: Handshake                     │       │ │  │
│  │  │    │  Write Response: BRESP, BVALID, BREADY             │       │ │  │
│  │  │    │  Read Address: ARADDR, ARVALID, ARREADY            │       │ │  │
│  │  │    │  Read Data: RDATA[511:0], RVALID, RREADY           │       │ │  │
│  │  │    │                                                     │       │ │  │
│  │  │    │ Handshake Protocol (every cycle @ 225 MHz):        │       │ │  │
│  │  │    │  - Transfer occurs when VALID=1 AND READY=1         │       │ │  │
│  │  │    │  - Source asserts VALID, holds data stable          │       │ │  │
│  │  │    │  - Destination asserts READY when can accept        │       │ │  │
│  │  │    │  - Example: PCIe write to FPGA:                    │       │ │  │
│  │  │    │    T0: AWADDR=0x1000, AWVALID=1, AWREADY=0 (wait)  │       │ │  │
│  │  │    │    T1: AWADDR=0x1000, AWVALID=1, AWREADY=1 (xfer!) │       │ │  │
│  │  │    │    T2: WDATA=0x..., WVALID=1, WREADY=1 (data xfer) │       │ │  │
│  │  │    └────────────────────────────────────────────────────┘       │ │  │
│  │  └────────────────────────┬─────────────────────────────────────────┘ │  │
│  │                           │ AXI4 bus (512-bit + control signals)      │  │
│  │  ┌────────────────────────▼─────────────────────────────────────────┐ │  │
│  │  │  pcie2fifos.v (Verilog module synthesized into CLBs)            │ │  │
│  │  │  - Physical implementation:                                      │ │  │
│  │  │    * State machine: ~50 FFs (current state, counters)           │ │  │
│  │  │    * Address decode: ~100 LUTs (comparators for address ranges) │ │  │
│  │  │    * AXI4 handshake logic: ~200 LUTs                            │ │  │
│  │  │  - Converts: AXI4 bus transactions → FIFO writes/reads          │ │  │
│  │  │  - Location in fabric: Near PCIe block (constrained placement)  │ │  │
│  │  └────────────────────────┬─────────────────────────────────────────┘ │  │
│  │                           │                                            │  │
│  │        ┌──────────────────┴──────────────────┐                         │  │
│  │        │ Input FIFO          Output FIFO     │                         │  │
│  │        │ (FPGA FIFO primitive)                │                         │  │
│  │  ┌─────▼─────────────┐  ┌─────────────────────▼────┐                  │  │
│  │  │ FIFO36E2 Primitive│  │ FIFO36E2 Primitive       │                  │  │
│  │  │ (Xilinx hard IP)  │  │                          │                  │  │
│  │  │ - Physical: Dedicated│  │ - Same structure        │                  │  │
│  │  │   36Kb BRAM + logic│  │ - Direction: FPGA→Host  │                  │  │
│  │  │ - Width: 512 bits  │  │ - Depth: 512 entries    │                  │  │
│  │  │ - Depth: 512 entries│  │                          │                  │  │
│  │  │ - Write side:      │  │ - Write side:            │                  │  │
│  │  │   WR_CLK = aclk    │  │   WR_CLK = aclk          │                  │  │
│  │  │   WR_EN (enable)   │  │   WR_EN                  │                  │  │
│  │  │   DI[511:0] (data) │  │   DI[511:0]              │                  │  │
│  │  │ - Read side:       │  │ - Read side:             │                  │  │
│  │  │   RD_CLK = aclk    │  │   RD_CLK = aclk          │                  │  │
│  │  │   RD_EN            │  │   RD_EN                  │                  │  │
│  │  │   DO[511:0]        │  │   DO[511:0]              │                  │  │
│  │  │ - Status signals:  │  │ - Status: FULL, EMPTY    │                  │  │
│  │  │   FULL, EMPTY      │  │                          │                  │  │
│  │  │ - Mode: FWFT       │  │ - Latency: 1 cycle (FWFT)│                  │  │
│  │  │   (First Word Fall │  │                          │                  │  │
│  │  │    Through - data  │  │                          │                  │  │
│  │  │    available same  │  │                          │                  │  │
│  │  │    cycle as !EMPTY)│  │                          │                  │  │
│  │  └─────┬─────────────┘  └─────────────────────┬────┘                  │  │
│  │        │                                       │                        │  │
│  │  ┌─────▼───────────────────────────────────────▼────────────────────┐  │  │
│  │  │  command_interpreter.v                                           │  │  │
│  │  │  - Reads from Input FIFO when !EMPTY                             │  │  │
│  │  │  - Parses 512-bit word:                                          │  │  │
│  │  │    [511:504] = 8-bit opcode                                      │  │  │
│  │  │    [503:496] = 8-bit CoreID                                      │  │  │
│  │  │    [495:0] = 496 bits of payload                                 │  │  │
│  │  │  - Decoding (combinational logic, happens in <5ns):              │  │  │
│  │  │    opcode_is_input = (opcode == 8'h00) ? 1'b1 : 1'b0            │  │  │
│  │  │    opcode_is_hbm_write = (opcode == 8'h02) ? 1'b1 : 1'b0        │  │  │
│  │  │    ... (similar for other opcodes)                               │  │  │
│  │  │  - State machine (sequential logic, FF-based):                   │  │  │
│  │  │    IDLE → READ_OPCODE → ROUTE_DATA → IDLE                       │  │  │
│  │  │  - Outputs: Control signals to each target module               │  │  │
│  │  │    write_enable_to_bram, data_to_hbm, execute_pulse, etc.       │  │  │
│  │  └────┬──────────────────┬───────────────────┬──────────────────┬───┘  │  │
│  │       │                  │                   │                  │       │  │
│  │  ┌────▼──────────────────▼───────────────────▼──────────────────▼───┐  │  │
│  │  │  input_data_handler.v (BRAM Arbiter)                            │  │  │
│  │  │  - Physical: State machine in ~30 FFs + mux logic in ~50 LUTs  │  │  │
│  │  │  - Function: Arbitrates 2 masters accessing 1 BRAM             │  │  │
│  │  │    Master A: command_interpreter (writes from host)             │  │  │
│  │  │    Master B: external_events_processor (reads during execution)│  │  │
│  │  │  - Arbitration (priority encoder):                              │  │  │
│  │  │    if (cmd_interp_request) grant <= 2'b01; // Priority         │  │  │
│  │  │    else if (ext_events_request) grant <= 2'b10;                │  │  │
│  │  │  - Muxes signals to BRAM:                                       │  │  │
│  │  │    bram_addr <= (grant==2'b01) ? cmd_addr : ext_addr;          │  │  │
│  │  │    bram_we <= (grant==2'b01) ? cmd_we : 1'b0; // Only cmd writes│  │  │
│  │  │    bram_din <= cmd_data; // Data input                          │  │  │
│  │  │  ┌──────────────────────────────────────────────────────────┐  │  │  │
│  │  │  │ BRAM (Block RAM - Xilinx RAMB36E2 primitives)            │  │  │
│  │  │  │ - Physical: Dedicated 36Kb SRAM blocks in FPGA           │  │  │
│  │  │  │ - Organization: 32,768 rows × 256 bits                   │  │  │
│  │  │  │   (uses 256 RAMB36 primitives: 256×256b=65Kb each row)   │  │  │
│  │  │  │ - Address: 15-bit [14:0]                                 │  │  │
│  │  │  │ - Memory cell: 6-transistor SRAM cell per bit            │  │  │
│  │  │  │ - Access time: 3 clock cycles @ 225 MHz                  │  │  │
│  │  │  │   Cycle 0: Present address on ADDR bus                   │  │  │
│  │  │  │   Cycle 1: Internal row decode                           │  │  │
│  │  │  │   Cycle 2: Column mux, sense amp                         │  │  │
│  │  │  │   Cycle 3: Data valid on DOUT                            │  │  │
│  │  │  │ - Port A (Read/Write): 256-bit width                     │  │  │
│  │  │  │   ADDRA[14:0], DINA[255:0], DOUTA[255:0], WEA, ENA       │  │  │
│  │  │  │ - True Dual Port: Could use Port B, but we use only A    │  │  │
│  │  │  │                                                           │  │  │
│  │  │  │ - Content (programmed by fpga_compiler.py):              │  │  │
│  │  │  │   Row Address | Content                                  │  │  │
│  │  │  │   ────────────┼──────────────────────────────────        │  │  │
│  │  │  │   0x0000      │ [255:240] Neuron group 15 mask           │  │  │
│  │  │  │               │ [239:224] Neuron group 14 mask           │  │  │
│  │  │  │               │ ...                                       │  │  │
│  │  │  │               │ [15:0] Neuron group 0 mask               │  │  │
│  │  │  │   0x0001      │ Same format for next axon/event          │  │  │
│  │  │  │   ...         │                                           │  │  │
│  │  │  │                                                           │  │  │
│  │  │  │ Each 16-bit mask: Bitmask of which neurons in that group │  │  │
│  │  │  │                    should receive this axon spike         │  │  │
│  │  │  └──────────────────────────────────────────────────────────┘  │  │  │
│  │  └──────┬───────────────────────────────────────────────────────────┘  │  │
│  │         │ Read data (256-bit)                                          │  │
│  │  ┌──────▼──────────────────────────────────────────────────────────┐  │  │
│  │  │  external_events_processor.v (Phase 1: Axon Processing)         │  │  │
│  │  │  - Triggered by: execute pulse from command_interpreter         │  │  │
│  │  │  - Physical: ~500 LUTs, ~200 FFs (pipeline stages + counters)  │  │  │
│  │  │  - Pipeline Structure (3 stages for throughput):                │  │  │
│  │  │    Stage 1: Read BRAM row address from scan counter             │  │  │
│  │  │    Stage 2: Wait for BRAM read latency (3 cycles)               │  │  │
│  │  │    Stage 3: Parse returned 256-bit word, find active bits       │  │  │
│  │  │  - For each active axon spike mask:                             │  │  │
│  │  │    1. Calculate HBM address (combinational):                    │  │  │
│  │  │       hbm_addr = AXN_BASE_ADDR + (row_index << 5);              │  │  │
│  │  │         // AXN_BASE_ADDR=0, shift left 5 = multiply by 32       │  │  │
│  │  │         // Each row is 32 bytes (256 bits) in HBM               │  │  │
│  │  │    2. Assert read request to hbm_processor:                     │  │  │
│  │  │       hbm_rd_en <= 1'b1;                                        │  │  │
│  │  │       hbm_rd_addr <= hbm_addr;                                  │  │  │
│  │  │  - Timing: Processes ~100 axons/microsecond                     │  │  │
│  │  └──────┬──────────────────────────────────────────────────────────┘  │  │
│  │         │ HBM Read Requests                                           │  │
│  │  ┌──────▼──────────────────────────────────────────────────────────┐  │  │
│  │  │  hbm_processor.v (HBM Memory Controller)                        │  │  │
│  │  │  - Physical: ~800 LUTs, ~300 FFs + AXI4 state machines         │  │  │
│  │  │  - Interfaces:                                                   │  │  │
│  │  │    Input: Read/write requests from other modules                │  │  │
│  │  │    Output: AXI4 Master to HBM interface                         │  │  │
│  │  │  - State Machine (services requests):                           │  │  │
│  │  │    IDLE → CHECK_QUEUE → SEND_ADDR → WAIT_DATA → IDLE           │  │  │
│  │  │  - AXI4 Read Transaction (takes ~100-200ns = 22-45 cycles):     │  │  │
│  │  │    T0: Assert ARVALID, present ARADDR (33-bit HBM address)      │  │  │
│  │  │    T1-T10: Wait for ARREADY (HBM controller accepts)            │  │  │
│  │  │    T11-T44: HBM access latency (internal row/col decode)        │  │  │
│  │  │    T45: HBM asserts RVALID, presents RDATA[255:0]               │  │  │
│  │  │    T45: We assert RREADY, capture data                          │  │  │
│  │  │  - Pointer Chain Following:                                     │  │  │
│  │  │    Reads axon pointer row: [31:23]=length, [22:0]=start_addr   │  │  │
│  │  │    Then reads synapse rows from start_addr to start_addr+length│  │  │
│  │  │    Prefetches next row while processing current (pipelining)    │  │  │
│  │  └──────┬──────────────────────────────────────────────────────────┘  │  │
│  │         │ AXI4 Master (256-bit data, 33-bit address)                  │  │
│  │  ┌──────▼──────────────────────────────────────────────────────────┐  │  │
│  │  │  hbm_register_slice (Pipeline Stage)                            │  │  │
│  │  │  - Physical: ~300 FFs (one per AXI signal)                      │  │  │
│  │  │  - Purpose: Timing closure for long routing paths               │  │  │
│  │  │    HBM is physically far from main logic on FPGA die            │  │  │
│  │  │    Wire delay ~2ns, exceeds 225MHz period (4.4ns)               │  │  │
│  │  │    Solution: Insert register stage (adds 1 cycle latency)       │  │  │
│  │  │  - Simply FFs on all AXI signals:                               │  │  │
│  │  │    always @(posedge aclk) begin                                 │  │  │
│  │  │      araddr_q <= araddr_d;                                      │  │  │
│  │  │      arvalid_q <= arvalid_d;                                    │  │  │
│  │  │      ... (for all ~30 AXI signals)                              │  │  │
│  │  │    end                                                           │  │  │
│  │  └──────┬──────────────────────────────────────────────────────────┘  │  │
│  │         │                                                              │  │
└──┼─────────┼──────────────────────────────────────────────────────────────┼──┘
   │         ▼                                                              │
   │  ═══════════════════════════════════════════════════════════════      │
   │  HIGH BANDWIDTH MEMORY (HBM2 - Samsung/Hynix DRAM stacks)             │
   │  ═══════════════════════════════════════════════════════════════      │
   │                                                                         │
   │  Physical Structure:                                                   │
   │  - 4 DRAM stacks (dies) stacked vertically on silicon interposer      │
   │  - Each stack: 8 layers × 128 MB = 1 GB per stack (but only 2GB used) │
   │  - Total: 8 GB capacity (we use 2 GB per stack = 8 GB total)          │
   │  - Connection: Through-Silicon Vias (TSVs) between die layers         │
   │  - Interposer: Silicon substrate with microbumps connecting HBM→FPGA  │
   │                                                                         │
   │  Why "High Bandwidth"?                                                 │
   │  - Width: Each stack has 1024-bit interface (128 bytes)                │
   │  - Channels: 8 independent 128-bit channels per stack = 1024-bit total│
   │  - Frequency: 900 MHz DDR (1800 MT/s)                                  │
   │  - Bandwidth per stack: 1024 bits × 1800 MT/s = 230 GB/s               │
   │  - System total: 4 stacks × 230 GB/s = 920 GB/s peak                   │
   │  - We use: 32 AXI ports (1 per core), ~400 GB/s utilized               │
   │                                                                         │
   │  DRAM Cell Structure (what stores each bit):                           │
   │  - 1 capacitor (stores charge = data bit)                              │
   │  - 1 transistor (access gate)                                          │
   │  - Capacitor charge decays → requires refresh every 64ms               │
   │  - Read is destructive → must write back after read                    │
   │                                                                         │
   │  Memory Organization (hierarchical):                                   │
   │  ┌────────────────────────────────────────────────────────┐            │
   │  │ HBM Address [32:0] = 33 bits → 8 GB address space      │            │
   │  │ (But only use 2 GB = 31 bits actually used)            │            │
   │  │                                                         │            │
   │  │ [32:30] = Stack select (which of 4 DRAM stacks)        │            │
   │  │ [29:27] = Channel select (which of 8 channels in stack)│            │
   │  │ [26:13] = Row address (which row in DRAM bank)         │            │
   │  │ [12:5]  = Column address (which 256-bit word in row)   │            │
   │  │ [4:0]   = Byte offset (which byte in 32-byte word)     │            │
   │  │                                                         │            │
   │  │ Example: Read synapse data at address 0x0001_2A40      │            │
   │  │   Stack = 0b000 (stack 0)                              │            │
   │  │   Channel = 0b000 (channel 0 within stack)             │            │
   │  │   Row = 0b00000000001001 (row 9)                       │            │
   │  │   Column = 0b01001010 (column 74)                      │            │
   │  │   Byte = 0b00000 (byte 0)                              │            │
   │  │   → Reads 32 bytes from stack0/chan0/row9/col74        │            │
   │  └────────────────────────────────────────────────────────┘            │
   │                                                                         │
   │  Content Organization (written by fpga_compiler.py):                   │
   │  ┌────────────────────────────────────────────────────────┐            │
   │  │ REGION 1: Axon Pointers                                │            │
   │  │ Base: 0x0000_0000 (AXN_BASE_ADDR = 0)                  │            │
   │  │ Size: 16,384 rows × 32 bytes = 512 KB                  │            │
   │  │ Format per row (256 bits = 8 pointers × 32 bits):      │            │
   │  │                                                         │            │
   │  │ Row 0: [Axon 0 ptr] [Axon 1 ptr] ... [Axon 7 ptr]      │            │
   │  │ Row 1: [Axon 8 ptr] [Axon 9 ptr] ... [Axon 15 ptr]     │            │
   │  │ ...                                                     │            │
   │  │                                                         │            │
   │  │ Each pointer (32 bits):                                │            │
   │  │   Bits [31:23]: Length (9-bit) = number of syn rows    │            │
   │  │   Bits [22:0]:  Start address (23-bit) = HBM row index │            │
   │  │                 (adds to SYN_BASE_ADDR=0x8000)          │            │
   │  │                                                         │            │
   │  │ Example: Axon 5 pointer = 0x0020_1234                   │            │
   │  │   Length = 0b000000001 = 1 row of synapses             │            │
   │  │   Start = 0x1234 → actual addr = 0x8000 + 0x1234       │            │
   │  └────────────────────────────────────────────────────────┘            │
   │  ┌────────────────────────────────────────────────────────┐            │
   │  │ REGION 2: Neuron Pointers                              │            │
   │  │ Base: 0x0000_4000 (NRN_BASE_ADDR = 2^14 = 16384)       │            │
   │  │ Size: 16,384 rows × 32 bytes = 512 KB                  │            │
   │  │ Format: Identical to axon pointers                      │            │
   │  │         But points to synapses from neurons→neurons     │            │
   │  │                                                         │            │
   │  │ Row 16384: [Neuron 0 ptr] ... [Neuron 7 ptr]           │            │
   │  │ Row 16385: [Neuron 8 ptr] ... [Neuron 15 ptr]          │            │
   │  │ ...                                                     │            │
   │  └────────────────────────────────────────────────────────┘            │
   │  ┌────────────────────────────────────────────────────────┐            │
   │  │ REGION 3: Synapses (Variable Length)                   │            │
   │  │ Base: 0x0000_8000 (SYN_BASE_ADDR = 2^15 = 32768)       │            │
   │  │ Size: Depends on network (up to remaining ~7.5 GB)     │            │
   │  │ Format per row (256 bits = 8 synapses × 32 bits):      │            │
   │  │                                                         │            │
   │  │ Each synapse (32 bits):                                │            │
   │  │   Bits [31:29]: OpCode (3-bit)                         │            │
   │  │     000 = Regular synapse                              │            │
   │  │     100 = Output spike entry                           │            │
   │  │   Bits [28:16]: Target address (13-bit)                │            │
   │  │     For synapse: Neuron index or row address           │            │
   │  │     For output: Neuron index to send to host           │            │
   │  │   Bits [15:0]: Weight (16-bit signed fixed-point)      │            │
   │  │     Format: 1 sign bit + 15 fractional bits            │            │
   │  │     Value = weight / 2^15 * scale_factor               │            │
   │  │     Example: 0x7FFF = +32767/32768 ≈ 1.0               │            │
   │  │              0x8000 = -32768/32768 = -1.0              │            │
   │  │                                                         │            │
   │  │ Example row at address 0x8000:                         │            │
   │  │ [31:0]   = 0x0010_03E8 = OpCode 000, Addr 16, Wt 1000  │            │
   │  │ [63:32]  = 0x0011_03E8 = OpCode 000, Addr 17, Wt 1000  │            │
   │  │ [95:64]  = 0x0012_03E8 = OpCode 000, Addr 18, Wt 1000  │            │
   │  │ ... (5 more synapses)                                  │            │
   │  │                                                         │            │
   │  │ So this row represents 8 synapses:                     │            │
   │  │   (target=16, weight=1000)                             │            │
   │  │   (target=17, weight=1000)                             │            │
   │  │   (target=18, weight=1000)                             │            │
   │  │   etc.                                                  │            │
   │  └────────────────────────────────────────────────────────┘            │
   │                                                                         │
   │  HBM Read Transaction (what happens physically):                       │
   │  1. FPGA asserts ARVALID, presents ARADDR on AXI bus                   │
   │  2. HBM controller decodes address:                                    │
   │     - Selects stack, channel, bank from address bits                   │
   │     - Issues ACTIVATE command to open DRAM row                         │
   │  3. Row sense amplifiers read entire row into row buffer (512 bytes)   │
   │  4. Column select reads 32-byte chunk from row buffer                  │
   │  5. Data serialized onto 1024-bit HBM bus                              │
   │  6. HBM controller asserts RVALID, presents RDATA[255:0]               │
   │  7. FPGA captures data on clock edge where RREADY=1                    │
   │  Latency: ~100-200ns (row activation + column access + serialization)  │
   │                                                                         │
   │  Why Pointer Chains?                                                   │
   │  - Sparse connectivity: Neuron might have 100 synapses out of 131K     │
   │  - Dense array would waste: 131K × 32 bits = 4.2 MB per neuron         │
   │  - Pointer chain stores only actual connections                        │
   │  - Tradeoff: Requires pointer lookup + chase, but saves massive space  │
   │  ═══════════════════════════════════════════════════════════════      │
                              ▲                                              │
                              │ Read synapse data (256-bit rows)             │
┌──────────────┬──────────────┴──────────────────────────────────────────────┘
│  FPGA (cont) │
│  ┌───────────▼──────────────────────────────────────────────────────────┐
│  │  pointer_fifo_controller.v (Phase 2: Distribution)                   │
│  │  - Physical: ~400 LUTs, ~150 FFs                                     │
│  │  - Input: 256-bit HBM data (8 synapses)                              │
│  │  - Processing (combinational logic for each synapse):                │
│  │    synapse[i] = hbm_data[(i*32)+:32]; // Extract 32-bit chunk        │
│  │    opcode = synapse[31:29];                                          │
│  │    target_addr = synapse[28:16]; // 13-bit address                   │
│  │    weight = synapse[15:0];                                           │
│  │    neuron_group = target_addr[12:9]; // Top 4 bits = group 0-15     │
│  │                                                                       │
│  │  - Routing (uses decoder to select destination FIFO):                │
│  │    case (neuron_group)                                               │
│  │      4'd0: fifo_wr_en[0] <= 1; fifo_din[0] <= {weight, addr};       │
│  │      4'd1: fifo_wr_en[1] <= 1; fifo_din[1] <= {weight, addr};       │
│  │      ...                                                              │
│  │      4'd15: fifo_wr_en[15] <= 1; fifo_din[15] <= {weight, addr};    │
│  │    endcase                                                            │
│  │                                                                       │
│  │  - Output: 16 independent pointer FIFOs (one per neuron group)       │
│  │    ┌──────────────────────────────────────────────────────┐         │
│  │    │ Pointer FIFO (each of 16)                            │         │
│  │    │ - Width: 32 bits (16-bit weight + 16-bit local addr) │         │
│  │    │ - Depth: 512 entries (uses FIFO18E2 primitive)       │         │
│  │    │ - Mode: Async (write aclk, read aclk450)             │         │
│  │    │ - Physical: Uses 18Kb BRAM primitive                 │         │
│  │    └──────────────────────────────────────────────────────┘         │
│  └───────────┬──────────────────────────────────────────────────────────┘
│              │ 16 FIFOs → one per neuron group
│  ┌───────────▼──────────────────────────────────────────────────────────┐
│  │  Clock Domain Crossing (BRAM → URAM)                                │
│  │  - Write side: aclk = 225 MHz (BRAM, pointer FIFOs)                 │
│  │  - Read side: aclk450 = 450 MHz (URAM, neuron processing)           │
│  │  - Mechanism: Async FIFO with gray-coded pointers                   │
│  │    Write pointer (aclk domain): Increments when FIFO written        │
│  │    Read pointer (aclk450 domain): Increments when FIFO read         │
│  │    Synchronization: 2-FF synchronizer for pointer crossing          │
│  │      always @(posedge aclk450)                                       │
│  │        wptr_sync1 <= wptr; // First FF (may be metastable)          │
│  │        wptr_sync2 <= wptr_sync1; // Second FF (stable)              │
│  │    Empty calculation: rptr == wptr_sync2 (in aclk450 domain)        │
│  │    This ensures no data corruption during clock domain crossing     │
│  └───────────┬──────────────────────────────────────────────────────────┘
│              │ Read @ 450 MHz
│  ┌───────────▼──────────────────────────────────────────────────────────┐
│  │  internal_events_processor.v (Phase 3: Neuron Updates)              │
│  │  - Clock: aclk450 = 450 MHz (2× throughput)                         │
│  │  - Physical: ~2000 LUTs, ~800 FFs (16 parallel state machines)      │
│  │  - Parallelism: Processes 16 URAM banks simultaneously              │
│  │    Each bank handles 8,192 neurons (131,072 neurons total/core)     │
│  │                                                                       │
│  │  Per-Bank Processing (happens every 450 MHz clock cycle):           │
│  │  ┌────────────────────────────────────────────────────────────────┐ │
│  │  │ Bank N State Machine (one of 16 copies)                        │ │
│  │  │                                                                 │ │
│  │  │ State: IDLE → CHECK_FIFO → READ_URAM → ACCUMULATE →           │ │
│  │  │        APPLY_MODEL → WRITE_URAM → CHECK_SPIKE → IDLE          │ │
│  │  │                                                                 │ │
│  │  │ Cycle-by-cycle operation @ 450 MHz (2.22ns per cycle):         │ │
│  │  │                                                                 │ │
│  │  │ T0: CHECK_FIFO state                                           │ │
│  │  │     - Read pointer FIFO: !EMPTY?                               │ │
│  │  │     - If empty → IDLE                                          │ │
│  │  │     - If data: capture {weight, local_addr}                    │ │
│  │  │     - local_addr is index within this bank (0-8191)            │ │
│  │  │                                                                 │ │
│  │  │ T1: READ_URAM state                                            │ │
│  │  │     - Present address to URAM:                                 │ │
│  │  │       uram_addr <= local_addr[12:1]; // Divide by 2            │ │
│  │  │         (Each URAM word holds 2 neurons)                       │ │
│  │  │       uram_rd_en <= 1;                                         │ │
│  │  │                                                                 │ │
│  │  │ T2: ACCUMULATE state                                           │ │
│  │  │     - URAM returns data (1 cycle latency):                     │ │
│  │  │       uram_dout[71:0] contains 2 neurons × 36 bits             │ │
│  │  │     - Select which neuron:                                     │ │
│  │  │       if (local_addr[0] == 0)                                  │ │
│  │  │         neuron_data = uram_dout[35:0];  // Lower neuron        │ │
│  │  │       else                                                      │ │
│  │  │         neuron_data = uram_dout[71:36]; // Upper neuron        │ │
│  │  │     - Extract membrane potential:                              │ │
│  │  │       V_old = neuron_data[35:0]; // 36-bit signed value        │ │
│  │  │     - Add synaptic input:                                      │ │
│  │  │       V_new = V_old + weight; // 36-bit adder                  │ │
│  │  │                                                                 │ │
│  │  │ T3: APPLY_MODEL state                                          │ │
│  │  │     - Apply leak (if configured):                              │ │
│  │  │       if (leak_enable)                                         │ │
│  │  │         V_new = V_new - (V_new >> leak_shift);                 │ │
│  │  │           // Right shift = divide by 2^leak_shift              │ │
│  │  │           // Subtracts fraction of V (leak current)            │ │
│  │  │     - Check threshold:                                         │ │
│  │  │       spike = (V_new >= threshold);                            │ │
│  │  │     - Apply reset if spike:                                    │ │
│  │  │       if (spike)                                               │ │
│  │  │         V_final = 0; // Reset to 0                             │ │
│  │  │       else                                                      │ │
│  │  │         V_final = V_new;                                       │ │
│  │  │                                                                 │ │
│  │  │ T4: WRITE_URAM state                                           │ │
│  │  │     - Reconstruct 72-bit word:                                 │ │
│  │  │       if (local_addr[0] == 0)                                  │ │
│  │  │         uram_din = {uram_dout[71:36], V_final}; // Update lower│ │
│  │  │       else                                                      │ │
│  │  │         uram_din = {V_final, uram_dout[35:0]}; // Update upper │ │
│  │  │     - Write back to URAM:                                      │ │
│  │  │       uram_we <= 1;                                            │ │
│  │  │       uram_addr <= local_addr[12:1];                           │ │
│  │  │       uram_din <= uram_din;                                    │ │
│  │  │                                                                 │ │
│  │  │ T5: CHECK_SPIKE state                                          │ │
│  │  │     - If spike occurred:                                       │ │
│  │  │       spike_fifo_wr_en <= 1;                                   │ │
│  │  │       spike_fifo_din <= {bank_id[3:0], local_addr[12:0]};     │ │
│  │  │         // 17-bit global neuron address                        │ │
│  │  │     - Return to IDLE                                           │ │
│  │  │                                                                 │ │
│  │  │ Hazard Detection:                                              │ │
│  │  │   - Problem: Same neuron could be in pipeline twice            │ │
│  │  │   - Detection: Compare local_addr with in-flight addresses     │ │
│  │  │   - Resolution: Stall until first update completes             │ │
│  │  │     if (local_addr == pipeline_addr_T1 ||                      │ │
│  │  │         local_addr == pipeline_addr_T2 ||                      │ │
│  │  │         local_addr == pipeline_addr_T3)                        │ │
│  │  │       stall <= 1; // Wait in CHECK_FIFO state                  │ │
│  │  └────────────────────────────────────────────────────────────────┘ │
│  │                                                                       │
│  │  URAM Interface (each of 16 banks):                                  │
│  │  ┌────────────────────────────────────────────────────────────────┐ │
│  │  │ URAM288 Primitive (Xilinx URAM288 macro)                       │ │
│  │  │ - Physical: Dedicated UltraRAM block (hard silicon IP)         │ │
│  │  │ - Capacity: 288 Kb (36K × 8 bits), but we use 72-bit mode      │ │
│  │  │ - Configuration: 4096 rows × 72 bits                           │ │
│  │  │ - Technology: 1-transistor + 1-capacitor DRAM-like cell        │ │
│  │  │   (But on FPGA die, not external DRAM)                         │ │
│  │  │ - Access time: 1 clock cycle @ 450 MHz                         │ │
│  │  │   Cycle N: Present ADDR                                        │ │
│  │  │   Cycle N+1: Data valid on DOUT                                │ │
│  │  │ - Refresh: Not needed (technology holds charge longer)         │ │
│  │  │                                                                 │ │
│  │  │ Address [11:0] = 12 bits → 4096 rows                           │ │
│  │  │ Data [71:0] = 72 bits per row                                  │ │
│  │  │   [71:36] = Neuron 1 (upper): 36-bit membrane potential        │ │
│  │  │   [35:0]  = Neuron 0 (lower): 36-bit membrane potential        │ │
│  │  │                                                                 │ │
│  │  │ Total per bank: 4096 rows × 2 neurons/row = 8192 neurons       │ │
│  │  │ Total system: 16 banks × 8192 = 131,072 neurons                │ │
│  │  │                                                                 │ │
│  │  │ Memory Cell (conceptual, actual is proprietary):               │ │
│  │  │   Charge storage + access transistor                           │ │
│  │  │   Similar to DRAM but integrated on FPGA die                   │ │
│  │  │   Lower density than BRAM's 6T-SRAM but much denser overall    │ │
│  │  └────────────────────────────────────────────────────────────────┘ │
│  └───────────┬──────────────────────────────────────────────────────────┘
│              │ Spike outputs (17-bit neuron addresses)
│  ┌───────────▼──────────────────────────────────────────────────────────┐
│  │  spike_fifo_controller.v (Spike Collection & Routing)               │
│  │  - Physical: ~300 LUTs, ~100 FFs + 8 spike FIFOs                    │
│  │  - Input: 8 spike FIFOs (not 16 because 2 banks share 1 FIFO)       │
│  │    Each FIFO: 17-bit width, 512-entry depth                         │
│  │  - Arbitration: Round-robin across 8 FIFOs                          │
│  │    rr_counter <= rr_counter + 1; // Mod 8                           │
│  │    case (rr_counter)                                                 │
│  │      3'd0: if (!fifo0_empty) read_fifo0;                            │
│  │      3'd1: if (!fifo1_empty) read_fifo1;                            │
│  │      ...                                                             │
│  │    endcase                                                           │
│  │                                                                       │
│  │  - Packet Assembly (builds 512-bit output packet):                  │
│  │    ┌──────────────────────────────────────────────────────┐         │
│  │    │ Spike Packet Format (512 bits total)                 │         │
│  │    │ [511:496] Tag = 0xEEEE (identifies as spike packet)  │         │
│  │    │ [495:32]  Spike data: 14 spikes × 32 bits each       │         │
│  │    │           Each spike: [31:24]=0x00 (reserved)        │         │
│  │    │                       [23]=valid bit                 │         │
│  │    │                       [22:6]=neuron address (17-bit) │         │
│  │    │                       [5:0]=timestep_sub (6-bit)     │         │
│  │    │ [31:0]    Execution counter (timestep)               │         │
│  │    └──────────────────────────────────────────────────────┘         │
│  │                                                                       │
│  │  - Routing Decision:                                                 │
│  │    For each spike, check opcode (looked up from synapse entry):     │
│  │    - If OpCode=100 (output spike):                                  │
│  │      → Route to Output FIFO → PCIe → Host                           │
│  │      → fpga_controller.flush_spikes() retrieves these               │
│  │    - If OpCode=000 (recurrent synapse):                             │
│  │      → Route back to external_events_processor                      │
│  │      → Triggers Phase 1 again for neuron-to-neuron connections      │
│  └──────────┬────────────────────────────────────────────────────────────┘
│             │ Output spikes (512-bit packets)
│  ┌──────────▼────────────────────────────────────────────────────────────┐
│  │  Output FIFO → pcie2fifos → PCIe → Host                             │
│  │  - Retrieved by: fpga_controller.flush_spikes()                      │
│  │  - Host parsing (dmadump reads these bytes):                         │
│  │    1. Check tag: if (packet[511:496] == 0xEEEE) → spike packet       │
│  │    2. Extract timestep: timestep = packet[31:0]                      │
│  │    3. Parse spikes: for (i=0; i<14; i++)                             │
│  │         spike_word = packet[(i*32+32)+:32]                           │
│  │         if (spike_word[23]) // Valid bit                             │
│  │           neuron_id = spike_word[22:6]                               │
│  │           spike_list.append(neuron_id)                               │
│  └──────────────────────────────────────────────────────────────────────┘
└──────────────────────────────────────────────────────────────────────────┘
```

---

## Hardware Component Definitions (Low-Level)

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

## Communication Mechanisms Explained

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

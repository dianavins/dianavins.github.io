---
title: 0.2 Hardware Map
nav_order: 2
parent: Introduction
---


# 0.4 Hardware Architecture Map - Low-Level Physical Organization

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


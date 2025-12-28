---
title: 0.3 Hardware Component Definitions
nav_order: 3
parent: "0 Introduction"
---

# Hardware Component Definitions (Low-Level)

This guide introduces hardware components from the ground up, starting with basic concepts and building toward more complex systems. Each section assumes only knowledge from previous sections.

---

## Part 1: Foundational Concepts

### **Essential Terminology**

Before diving into hardware components, let's define some fundamental terms you'll see throughout:

**CPU (Central Processing Unit):**
- The "brain" of the computer - the chip that executes instructions
- Example: AMD EPYC 7502 (our host CPU)
- What it does: Runs your programs, performs calculations, controls other hardware
- **Important:** The CPU is a separate chip from memory - they talk to each other via connections

**System Memory (RAM):**
- The large storage area where the CPU keeps data it's working on
- Example: DDR4 memory sticks you plug into the motherboard
- **Not the same as the CPU** - it's external chips connected to the CPU
- Think of it as: CPU = chef, System Memory = kitchen counter where ingredients are laid out

**In our hs_bridge system:**
- Stores **command packets** that tell the FPGA what to do (e.g., "inject spike from axon 5", "run simulation for 10 timesteps")
- Stores **network configuration data** during initialization (before being transferred to FPGA's HBM)
- Serves as a **staging area** for data transfer: CPU writes data here, then FPGA reads it via DMA
- The FPGA communicates with system memory because:
  - It's where the CPU prepares data for the FPGA
  - Large capacity (gigabytes) - can hold entire network configurations
  - Shared between CPU and FPGA - both can access it (CPU via normal writes, FPGA via DMA)

**Buffer:**
- A temporary holding area for data
- Like a waiting room for data in transit
- **Types we'll see:**
  - **Row buffer:** Built into memory chips, holds one row's worth of data for fast repeated access
  - **DMA buffer:** Region of system memory set aside for transfers between devices
  - **FIFO buffer:** Hardware queue (First-In-First-Out) for data moving between components

**Bus:**
- A set of electrical wires that carry signals between components
- Like a highway connecting two locations
- **Examples:**
  - **DDR4 bus:** Wires connecting CPU to memory chips (64 bits wide)
  - **PCIe bus:** Wires connecting CPU to FPGA (512 bits wide in our case)
  - **AXI bus:** Wires inside the FPGA connecting modules
- **Width:** How many bits can travel in parallel (like highway lanes)

**Bus Master:**
- A device that can initiate (start) transfers on a bus
- Normally: CPU is the master, everything else responds
- With DMA: FPGA can also be a bus master (can request data without CPU help)
- Like: Normally only the manager can request files, but with DMA the assistant can too

**Packet:**
- A chunk of data wrapped with control information (headers)
- Like an envelope: has destination address, sender info, and the actual message inside
- **Examples:**
  - **PCIe TLP (Transaction Layer Packet):** Data moving over PCIe includes address, length, type
  - **Network packet:** Data over Ethernet/WiFi
- **Not all communication uses packets:** CPU talking to memory uses raw electrical signals, not packets

---

### **What is Memory?**

At its core, memory is a place to store digital information (bits: 0s and 1s). Think of it like a massive array of mailboxes, where each mailbox has:
- An **address** (which mailbox)
- **Contents** (what's stored in it)

When we say "read from memory," we're asking: "What's in mailbox #12345?"
When we say "write to memory," we're saying: "Put this value in mailbox #12345."

The two key questions for any memory technology are:
1. **How much can it store?** (capacity)
2. **How fast can we access it?** (bandwidth and latency)

Different types of memory make different trade-offs between these properties.

---

### **Host DDR4 SDRAM (System Memory) - The Basics**

**Full name:** DDR4 SDRAM = **D**ouble **D**ata **R**ate **4**th generation **S**ynchronous **D**ynamic **R**andom **A**ccess **M**emory

**What it is:** This is the main memory in your computer - the RAM that stores your programs and data while running.

**Why it exists:** CPUs (and other processors) need a place to store data they're working on. This data is too large to fit inside the processor itself, so we use external memory chips.

**The physical storage cell:**
- Each bit is stored using 1 transistor + 1 capacitor (called a "1T1C DRAM cell")
- The capacitor holds an electrical charge (~30 femtofarads, about 10,000 electrons)
- Charged capacitor = 1, discharged = 0

**Library building analogy:**

1. **DIMM (memory stick)** - The entire building. This is what you physically plug into the motherboard.
2. **Rank** - One floor of the building. Most DIMMs have chips on both sides; each side is a "rank."
3. **Chip** - One bookshelf on that floor. Each DIMM has 8-16 memory chips.
4. **Bank** - One section of a bookshelf. Each chip has 8 banks (like having 8 separate card catalogs).
5. **Row** - One shelf in that section. Each bank has ~65,000 rows.
6. **Column** - One book on that shelf. Each row has ~1,000 columns.
7. **DRAM cell** - A single page in a book. This is the smallest storage unit (1 transistor + 1 capacitor storing 1 bit).

So the full hierarchy: **Building → Floor → Bookshelf → Section → Shelf → Book → Page**
Or in hardware terms: **DIMM → Rank → Chip → Bank → Row → Column → DRAM cell**

When you read memory at [address](../supplementary_information/addresses_explained.md) 0x12345678, the memory controller breaks it down:
- "Go to DIMM #2, Rank #1, Chip #5, Bank #3, Row #42, Column #100"
- Like saying: "Building 2, Floor 1, Bookshelf 5, Section 3, Shelf 42, Book 100"

**Physical Organization:**
Think of memory like a filing system:
- **DIMMs (memory sticks):** The physical modules you plug into your motherboard
- **Chips:** Each DIMM has 8-16 chips
- **Banks:** Each chip has 8 banks (like having 8 separate filing cabinets)
- **Rows and Columns:** Each bank is organized as a grid (e.g., 65,000 rows × 1,000 columns)

**Row Buffer (the speed secret of DRAM):**

**What it is:**
- A small, fast cache built into each bank of the memory chip
- Holds one row's worth of data (~8KB)
- Made of fast SRAM-like circuits (see Appendix for SRAM definition)
- Located inside the memory chip, between the DRAM cells and the output pins

**How it works (simplified):**
1. **Open a row:** Memory controller sends "read row 42"
   - Entire row (~8,000 bytes) is copied into the row buffer
   - Time: ~50 nanoseconds

2. **Access data:** Request specific bytes from the open row
   - Much faster because data is already in the buffer
   - Time: ~10 nanoseconds

3. **Close the row:** Before opening a different row, must close current row
   - Time: ~30 nanoseconds

**Why this matters for performance:**
- **Row hit** (accessing same row again): Very fast (~10ns) - data already in buffer
- **Row miss** (accessing different row): Slow (~80ns) - must close old row, open new row
- **Sequential memory access** (like reading an array): Lots of row hits → fast
- **Random memory access** (jumping around): Lots of row misses → slow

**Performance characteristics:**
- **Access latency:** ~60 nanoseconds to read data
- **Bandwidth:** How much data per second we can transfer
  - 64-bit bus × 3200 MT/s (mega-transfers/second) = 25.6 GB/s
  - This is like a highway: 64 lanes wide, with traffic moving 3200 times per second

**Key takeaway:** DDR4 provides large capacity (typically 8-64 GB) at moderate speed. We'll see later how this serves as a staging area for data moving between the CPU and specialized hardware.

---

### **PCIe (Peripheral Component Interconnect Express) - The Basics**

**What it is:** A high-speed highway that connects your CPU/memory to peripheral devices like graphics cards, network cards, and FPGAs.

**Think of it like a highway system:**
- **Lanes:** Data travels in lanes (typically 1, 4, 8, or 16 lanes)
- **Bidirectional:** Each lane has traffic going both directions (transmit and receive)
- **Point-to-point:** Each device has its own dedicated connection (not a shared bus)

**Basic physical structure:**
- Each lane uses 4 wires: TX+, TX-, RX+, RX- (transmit and receive, each using a differential pair)
- **Differential signaling:** Instead of sending voltage on a single wire, we send the signal as the *difference* between two wires
  - Advantage: Noise affects both wires equally, so the difference remains clean
  - This is why PCIe can run at very high speeds reliably

**Speed:**
- PCIe Gen3: 8 gigabits/second per lane
- An x16 configuration (16 lanes): 16 × 8 = 128 Gb/s raw bandwidth
- After encoding overhead (128b/130b encoding): ~15.75 GB/s effective

**Key takeaway:** PCIe is the data highway connecting your main computer (CPU + memory) to specialized devices like our FPGA. It's fast, reliable, and provides dedicated connections.

---

## Part 2: Moving Data Between Components

Now that we understand basic memory (DDR4) and connections (PCIe), we need to understand *how* data moves between them.

### **Memory-Mapped I/O (MMIO)**

**MMIO = Memory-Mapped Input/Output**
- "Input/Output" (I/O) means communication with devices (keyboard, disk, FPGA, etc.)
- "Memory-Mapped" means we use memory addresses to talk to these devices

**The concept:** Make hardware devices look like memory.

**How it works:**
- The system has a **global address space** (like our mailbox analogy) - typically billions of addresses
  - From the **CPU's perspective:** It can access any address via load/store instructions
  - From the **FPGA's perspective:** When acting as a bus master (during DMA), it can also generate addresses to access system memory
  - The **memory controller** routes each address to the correct destination (RAM vs device registers)
- **Some addresses** refer to RAM (actual memory where data is stored)
- **Other addresses** refer to device registers (special control locations in hardware devices)
- **Who defines addresses:** The system designer/OS assigns address ranges (e.g., "addresses 0xD0000000-0xD0000FFF go to FPGA registers"). Individual FPGA modules don't generate their own address ranges - they respond to the ranges assigned to them.

**Example address map:**
```
Address 0x00000000 - 0x7FFFFFFF: System RAM (actual DDR4 memory)
Address 0xD0000000 - 0xD0000FFF: FPGA control registers (inside FPGA chip)
```

**What happens when you write to an MMIO address:**
```
CPU executes: memory[0xD0000000] = 0x00000001
```
1. CPU puts address 0xD0000000 on the address bus
2. CPU puts value 0x00000001 on the data bus
3. Memory controller sees this address is NOT in RAM range
4. Memory controller routes this over PCIe to the FPGA
5. PCIe wraps it in a TLP packet (Memory Write)
6. FPGA receives the packet, extracts the value
7. FPGA's hardware register at offset 0x0 gets the value 0x00000001
8. This might trigger: start simulation, stop simulation, reset, etc.

**In our system (hs_bridge):**
- Host software uses MMIO to send control commands to FPGA
- Example: `fpga.write_register(0xD0000000, start_flag=1)`
- This gets translated to a PCIe write that lands in FPGA control logic

**Key takeaway:** MMIO lets us control hardware devices using normal memory read/write operations. The CPU doesn't know (or care) if an address goes to RAM or a device - it just reads/writes, and the hardware routes it correctly.

---

### **DMA (Direct Memory Access)**

**What DMA is:**
- **Not a separate chip or component** - it's a capability/feature of how the system is designed
- **The concept:** Allow hardware devices (like our FPGA) to become "bus masters"
  - Bus master = can initiate read/write requests on the bus (normally only CPU does this)
  - With DMA, FPGA can read from system memory without CPU help
- **Implemented by:** PCIe hard block in FPGA + host memory controller + operating system support

**The problem DMA solves:**

Imagine the FPGA needs 1 GB of data from system memory:

**Without DMA (CPU-mediated transfer):**
```python
# This is what would happen without DMA - very slow!
for i in range(1024*1024*1024):  # 1 billion iterations!
    byte = cpu.read_memory(buffer_address + i)  # CPU reads from RAM
    cpu.write_to_fpga(byte)                      # CPU writes to FPGA via PCIe
    # CPU is fully occupied doing this grunt work
```
- CPU reads each byte from DDR4, then writes it to FPGA
- Slow: ~1 GB at maybe 100 MB/s = 10 seconds
- Wasteful: CPU can't do anything else during this time

**With DMA (FPGA direct access):**
```python
# CPU just sets up the transfer
cpu.tell_fpga("Read 1 GB from address 0x123456000")  # One MMIO write
cpu.continue_doing_other_work()  # CPU is free!
# Meanwhile, FPGA directly reads memory via PCIe
```
- FPGA reads directly from DDR4 over PCIe
- Fast: ~1 GB at 10 GB/s = 0.1 seconds
- Efficient: CPU just sets it up, then does other work

---

**How DMA works - Detailed walkthrough:**

### **Setup Phase (CPU/Host Software):**

**Step 1: Allocate a buffer in system memory**

In Python (using ctypes or similar):
```python
import ctypes

# malloc = "memory allocate" - a C function that reserves a chunk of memory
# We're requesting: 1024 × 1024 bytes = 1,048,576 bytes = 1 MB
# Why 1024? Because 1 KB (kilobyte) = 1024 bytes (power of 2: 2^10)
# So: 1024 bytes × 1024 = 1 MB (megabyte)

buffer_size = 1024 * 1024  # 1,048,576 bytes = 1 MB

# Allocate the memory (this asks the operating system for space)
# Returns a pointer - the memory address where our buffer starts
dma_buffer = (ctypes.c_uint8 * buffer_size)()  # Array of 1 MB bytes
# This memory is in DDR4 system RAM
```

**Why we use 1024 instead of 1000:**
- Computers use binary (base 2), not decimal (base 10)
- 2^10 = 1024 (convenient power of 2)
- So: 1 KB = 1024 bytes, 1 MB = 1024 KB = 1,048,576 bytes

**Step 2: Get the physical address**
```python
# Programs see "virtual addresses" (fake addresses assigned by OS)
# Hardware needs "physical addresses" (real DDR4 chip locations)
#
# Example:
#   Virtual address:  0x00007fff12340000 (what your program sees)
#   Physical address: 0x0000000123456000 (actual location in DDR4 chip)
#
# The FPGA needs the physical address to do DMA

# This is typically done by a kernel driver
physical_address = get_physical_address(dma_buffer)
# Returns something like: 0x123456000 (a real DDR4 address)
```

**Step 3: Fill the buffer with data**
```python
# Put the data we want to send into the DMA buffer
for i in range(buffer_size):
    dma_buffer[i] = my_data[i]  # Write data into system memory
# Now the data sits in DDR4, waiting to be read by FPGA
```

**Step 4: Tell the FPGA about the buffer (via MMIO)**
```python
# Write to FPGA control registers
# These MMIO writes go over PCIe to the FPGA

# Register 0: Buffer address (where to read from)
fpga_write_register(0xD0000000, physical_address)  # "Buffer is at 0x123456000"

# Register 1: Transfer length (how many bytes)
fpga_write_register(0xD0000004, buffer_size)  # "Transfer 1,048,576 bytes"

# Register 2: Direction (read or write?)
fpga_write_register(0xD0000008, DMA_READ)  # "FPGA will READ from this buffer"

# Register 3: Go command (start the transfer!)
fpga_write_register(0xD000000C, 1)  # "Start DMA now!"
```

After this, **the CPU is done!** It can go do other work.

---

### **Execution Phase (FPGA Hardware):**

All of this happens in hardware (no software running):

**1. FPGA reads the descriptor**
- FPGA's DMA controller sees the "go" bit was set (from register write above)
- Reads the configuration: address=0x123456000, length=1MB, direction=read

**2. FPGA becomes a bus master**
- Normally: CPU sends requests over PCIe, FPGA responds
- Now: FPGA sends requests over PCIe, host responds
- This is what "bus master" means

**3. FPGA issues Memory Read requests over PCIe**
```
FPGA creates PCIe TLP (Transaction Layer Packet):
  - Type: Memory Read Request
  - Address: 0x123456000
  - Length: 64 bytes (reads in chunks, not all at once)

Sends over PCIe → arrives at host memory controller
```

**Important:** DMA communication over PCIe **always uses packets** (TLPs). The FPGA doesn't directly touch the DDR4 memory - it sends packet requests to the host, and the host's memory controller handles the actual DDR4 access.

**4. Host memory controller responds**
```
Memory controller:
  1. Receives PCIe Memory Read TLP packet
  2. Extracts address: 0x123456000
  3. Translates to DDR4 coordinates: DIMM 2, Bank 3, Row 1000, Column 50
  4. Reads 64 bytes from DDR4 using DDR4 protocol (NOT packets - raw electrical signals)
  5. Packages the 64 bytes of data into a PCIe Completion TLP packet
  6. Sends Completion TLP back to FPGA over PCIe
```

**System Architecture Diagram:**

```
┌────────────────────────────────────────────────────────────────────┐
│                          HOST SYSTEM                               │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │                     CPU Die/Package                          │ │
│  │                                                              │ │
│  │  ┌──────────────┐          ┌──────────────────────┐        │ │
│  │  │              │          │ Memory Controller    │        │ │
│  │  │   CPU Cores  │<────────>│ (integrated in CPU)  │        │ │
│  │  │              │          │                      │        │ │
│  │  └──────────────┘          └────────┬─────────────┘        │ │
│  │                                     │▲ ▲                   │ │
│  │                                     ││ │ DDR4 bus          │ │
│  │  ┌──────────────┐                   ││ │ (wires, 64-bit,   │ │
│  │  │  PCIe Root   │                   ││ │  no packets)      │ │
│  │  │  Complex     │<──────────────────┘│ │                   │ │
│  │  │ (routes PCIe │────────────────────┘ │                   │ │
│  │  │  to memory   │                      │                   │ │
│  │  │ controller)  │                      │                   │ │
│  │  └──────┬───────┘                      │                   │ │
│  │         │ ▲                             │                   │ │
│  └─────────┼─┼─────────────────────────────┼───────────────────┘ │
│            │ │ PCIe TLP packets            │                     │
│            │ │ (bidirectional)             │                     │
│            │ │                             ▼                     │
│            │ │                  ┌─────────────────────┐          │
│            │ │                  │   DDR4 System RAM   │          │
│            │ │                  │   (DIMM sticks)     │          │
│            │ │                  │  - DMA buffers      │          │
│            │ │                  │  - Command packets  │          │
│            │ │                  └─────────────────────┘          │
│            │ │                                                   │
└────────────┼─┼───────────────────────────────────────────────────┘
             │ │
             │ │ PCIe lanes (wires)
             │ │ 16x Gen3, differential pairs
             ▼ │ TLP packets (bidirectional)
    ┌──────────────────┐
    │      FPGA        │
    │                  │
    │  ┌────────────┐  │
    │  │ PCIe Hard  │  │
    │  │   Block    │  │
    │  └────────────┘  │
    │                  │
    └──────────────────┘

Legend:
  ──>  : Data flow direction (arrows show communication paths)
  <──> : Bidirectional data flow

PCIe Root Complex: Hardware inside the CPU that manages PCIe connections.
Routes PCIe requests to/from the memory controller and CPU cores.
```

**Data Paths:**

**FPGA → Host (DMA Read):**
1. FPGA → PCIe TLP (Memory Read Request) → PCIe Root Complex
2. PCIe Root Complex → Memory Controller (decode address)
3. Memory Controller → DDR4 bus → System RAM (fetch data)
4. System RAM → DDR4 bus → Memory Controller (data returns)
5. Memory Controller → PCIe Root Complex (package as TLP)
6. PCIe Root Complex → PCIe TLP (Completion) → FPGA

**Host → FPGA (MMIO Write):**
1. CPU Cores → Memory Controller (write to MMIO address)
2. Memory Controller → PCIe Root Complex (route to PCIe)
3. PCIe Root Complex → PCIe TLP (Memory Write) → FPGA

**5. FPGA receives data**
- FPGA captures 64 bytes from Completion TLP (see "PCIe Transaction Details" section below for TLP types)
- **Completion TLP:** Response packet that carries the requested data back from a Memory Read request
- Stores in internal buffer or processes immediately
- Issues next read request for address 0x123456040 (next 64 bytes)
- Repeats until all 1 MB transferred

**6. FPGA signals completion**
```
When done reading all 1 MB:
  - FPGA writes status to memory (via DMA write)
  - FPGA sends MSI-X interrupt (special PCIe write)
  - Host interrupt controller notifies CPU
  - CPU's interrupt handler wakes up: "FPGA finished DMA!"
```

---

**In our hs_bridge system:**

```python
# fpga_compiler.py creates command arrays in DDR4
commands = create_command_array()  # Build array of 512-bit commands in system memory

# dmadump.dma_dump_write() sets up DMA
physical_addr = get_physical_address(commands)
fpga.write_mmio(DMA_ADDRESS_REG, physical_addr)  # Tell FPGA where commands are
fpga.write_mmio(DMA_CONTROL_REG, DMA_START)      # Start DMA transfer

# Now FPGA reads commands directly from DDR4 via DMA
# CPU is free to do other things
```

**Key concepts:**
- **DMA buffer:** A region of system memory (DDR4) designated for device transfers
  - NOT the row buffer (that's inside memory chips)
  - This is a large area (KB to GB) that we allocate with malloc/new
- **Bus master:** A device that can initiate transfers (FPGA acts as bus master during DMA)
- **Descriptor:** The metadata about the transfer (address, length, direction)

**Why CPU↔Memory communication still matters even with DMA:**
- CPU must allocate the DMA buffer (reserve space in DDR4)
- CPU must write data into the buffer before FPGA reads it
- Example in hs_bridge:
  1. CPU runs `fpga_compiler` to create **command arrays** in DDR4
     - Commands are 512-bit instructions for the FPGA
     - Examples: "inject spike from axon 5", "execute simulation for 10 timesteps", "write network weights to HBM"
  2. CPU writes these command packets to DDR4 (normal memory writes)
     - Commands control what the FPGA does: configure modules, trigger execution, load data
  3. CPU tells FPGA via MMIO: "Commands are at address 0x123456000"
  4. FPGA uses DMA to read those command packets from DDR4
  5. FPGA processes commands (e.g., runs neural network simulation), CPU does other work
     - FPGA interprets each command and performs the requested action
     - See `packet_encoding.md` in Supplementary Information for detailed command formats

**Key takeaway:** DMA is a capability that allows the FPGA to directly read/write system memory without CPU involvement. The CPU sets up the transfer, then the FPGA hardware and host memory controller handle it. This is much faster and frees the CPU to do other work.

---

### **PCIe Transaction Details**

Now that we understand what PCIe does (connects devices) and why DMA matters (efficient data transfer), let's see how PCIe actually accomplishes this.

**Transaction Layer Packets (TLPs):**
Think of TLPs as envelopes containing messages between devices.

**TLP structure:**
```
[Header: Who, what, where]
  - Format and type: What kind of transaction?
  - Length: How much data?
  - Address: Where in memory?
  - Requester ID: Who sent this?
[Payload: The actual data]
  - 0 to 4096 bytes
```

**Transaction types:**

**1. Memory Write (Posted):**
- **What:** Send data from one device to another
- **"Posted" means:** No response expected (fire and forget)
- **Example:** Host CPU writing a command to FPGA MMIO register
  - CPU sends TLP with address=0xD0000000, data=0x00000001 (start command)
  - FPGA receives and acts on it
  - No response sent back

**2. Memory Read:**
- **What:** Request data from memory
- **Requires response:** A "Completion" TLP with the data
- **Example:** FPGA reading DMA buffer from host memory
  - FPGA sends TLP: "Send me 64 bytes from address 0x123456000"
  - Host memory controller sends Completion TLP with the 64 bytes
  - FPGA receives the data

**3. MSI-X Interrupt:**
- **What:** A special write that triggers an interrupt
- **Example:** FPGA finished processing, needs to notify CPU
  - FPGA writes to a pre-configured "interrupt address"
  - Host interrupt controller receives this write
  - CPU's interrupt handler runs

**Link Layer (reliability):**
- **CRC checksums:** Detect data corruption
- **ACK/NAK protocol:** Acknowledge good packets, request retransmission for bad ones
- **Sequence numbers:** Detect lost packets
- **Flow control:** "Credit" system prevents buffer overflows
  - Receiver says: "I have room for 10 more packets"
  - Transmitter only sends 10, waits for more credits

**Physical Layer (the actual wires):**
- **Differential pairs:** TX+/TX-, RX+/RX- for each lane
- **Embedded clock:** No separate clock wire; timing recovered from data transitions
- **8b/10b encoding:** Extra bits ensure enough transitions for clock recovery

**Key takeaway:** PCIe implements a complete protocol stack - from physical signaling to reliable data transfer to high-level memory operations. This allows DMA and MMIO to work reliably at high speeds.

---

## Part 3: Specialized Hardware

Now we understand how the host system works (DDR4 memory, PCIe connections, DMA). Let's look at specialized hardware that can process data much faster than a CPU.

### **FPGA (Field-Programmable Gate Array) - The Basics**

**What it is:** An FPGA is a chip full of reconfigurable logic. Think of it as a blank canvas of digital circuits that you can reprogram to do whatever you want.

**Why FPGAs exist:**
- **CPUs are general-purpose:** They execute instructions one at a time (or a few in parallel)
  - Flexible: Can run any program
  - But: Limited parallelism
- **ASICs are special-purpose:** Custom silicon designed for one task
  - Fast: Optimized for specific task
  - But: Expensive to design and manufacture, can't be changed
- **FPGAs are in-between:** Reconfigurable hardware
  - Parallel: Can perform many operations simultaneously
  - Flexible: Can be reprogrammed for different tasks
  - Fast: Much faster than CPU for suitable tasks (e.g., our spiking neural network simulator)

**Our FPGA:** Xilinx XCVU37p (VU37P)
- **Technology:** 20nm FinFET manufacturing process
- **Die size:** ~800 mm² (large, expensive chip)
- **Power:** 50-100W typical (varies with design and clock speed)
- **Building Blocks:**
  - Configurable Logic Blocks (CLBs)
  - Programmable connection matrix for the CLBs
  - Global clock driving technology (to allow modules to run on the same clock and allow us define a clean state machine)
  - Block RAM
  - UltraRAM

---

### **FPGA Internal Structure**

**Configurable Logic Blocks (CLBs):**

The FPGA fabric is an array of CLBs connected by programmable routing.

**What's in a CLB?**
- **8 LUTs** (Look-Up Tables) - implement logic functions
- **16 Flip-Flops** - store state/register values
- **Carry logic** - for efficient arithmetic

**LUT (Look-Up Table):**

**Physical implementation:** 64-bit SRAM (Static Random Access Memory)
- **SRAM** = memory that holds data as long as power is on (see Appendix for full definition)
- **"Static"** means the module's behavior is fixed once programmed (doesn't change during execution)
  - The LUT configuration is loaded when you program the FPGA (upload the bitstream)
  - During execution, the LUT's function stays constant - it just evaluates its programmed logic
  - To change behavior, you must reprogram the entire FPGA
- **64-bit** = 64 memory cells storing 0s and 1s

**Function:** Can implement any 6-input Boolean function

**How it works - detailed explanation:**

Think of a LUT as a tiny lookup table with 64 entries:
```
┌──────────────┬────────┐
│   Address    │ Output │
│  (6 bits =   │ (1 bit)│
│   inputs)    │        │
├──────────────┼────────┤
│ 0b000000 (0) │   ?    │
│ 0b000001 (1) │   ?    │
│ 0b000010 (2) │   ?    │
│     ...      │  ...   │
│ 0b111110 (62)│   ?    │
│ 0b111111 (63)│   ?    │
└──────────────┴────────┘
```

The 6 input wires form a binary address that selects which of the 64 entries to read.

**Example: Programming a 2-input AND gate (simplified to 2 inputs for clarity)**

An AND gate outputs 1 only when BOTH inputs are 1:
```
Truth table for AND:
  A B │ Output
  ────┼────────
  0 0 │   0
  0 1 │   0
  1 0 │   0
  1 1 │   1    ← Only this outputs 1
```

To implement this in a LUT:
1. Use inputs A and B as the address (ignoring the other 4 input bits)
2. Program the SRAM contents to match the truth table:

```
Address (A,B) │ SRAM Contents │ Meaning
──────────────┼───────────────┼─────────────────
0b000000 (00) │      0        │ 0 AND 0 = 0
0b000001 (01) │      0        │ 0 AND 1 = 0
0b000010 (10) │      0        │ 1 AND 0 = 0
0b000011 (11) │      1        │ 1 AND 1 = 1 ✓
0b000100-111  │      0        │ (unused inputs)
```

**In Verilog:**
```verilog
// This Verilog code:
wire a, b, out;
assign out = a & b;  // AND gate

// Gets synthesized into a LUT where:
// - Inputs a, b connect to LUT input pins
// - LUT is programmed with the AND truth table above
// - LUT output connects to wire 'out'
// - When a=1, b=1: address=0b11, LUT outputs 1
// - When a=0, b=1: address=0b01, LUT outputs 0
```

**For a full 6-input example (like OR gate):**
```verilog
assign out = a | b | c | d | e | f;  // 6-input OR gate

// LUT programmed so:
// - Address 0b000000: outputs 0  (all inputs zero)
// - Address 0b000001: outputs 1  (at least one input is 1)
// - Address 0b000010: outputs 1
// - ...
// - Address 0b111111: outputs 1  (all inputs one)
// Total: 1 zero entry, 63 one entries
```

**Key insight:** The SRAM stores a complete lookup table mapping every possible input combination to the desired output. This is why LUTs can implement ANY 6-input Boolean function - just program the SRAM with the right truth table!

**Example:** `assign out = a & b;`
- Synthesis tool maps this to a LUT
- LUT is programmed (via SRAM bits) to implement the AND function
- Inputs a and b connect to LUT inputs
- LUT output connects to signal out

**Flip-Flop (Register):**
- **Physical implementation:** D-type register (master-slave latch pair)
- **Function:** Stores 1 bit, updates on clock edge
- **Inputs:**
  - D: Data input (value to store)
  - CLK: Clock (when to update)
  - CE: Clock Enable (enable updating)
  - RST: Reset (force to 0)
- **Operation:** On rising edge of CLK: if CE=1, then Q <= D

**Example:** `always @(posedge clk) q <= d;`
- This Verilog creates a flip-flop
- On each clock edge, q gets the value of d

**Synthesis flow:**
```
Verilog code → Logic gates → Map to LUTs + FFs

Example:
  Combinational logic: `assign out = a & b;` → Maps to LUTs
  Registers: `always @(posedge clk) q <= d;` → Maps to Flip-Flops

  (Text in `backticks` is actual Verilog code)
```

---

### **FPGA Routing and Timing**

**Programmable Interconnect:**
- **Problem:** We have thousands of LUTs and FFs that need to connect together
- **Solution:** Programmable switches (like a telephone switchboard)
  - **Switch matrix:** Crossbar at each routing junction
  - **Implementation:** Transistor pass gates controlled by SRAM bits
  - **Configuration:** SRAM bits determine which wires connect

**Routing delay:**
- Signals take time to travel through wires and switches
- Typical: 0.5-2 nanoseconds depending on distance
- This adds to the logic delay (time for LUTs to compute)

**Timing closure:**
Our design runs at 225 MHz (4.4 ns period). This means:
- **Critical path:** The longest path from one flip-flop to another must complete in < 4.4 ns
- Critical path time = LUT delay + routing delay + flip-flop setup time
- If too long: Must add pipeline registers (breaks path into shorter segments)
  - Tradeoff: Adds latency (more clock cycles) but meets timing

---

### **FPGA Clock Distribution**

**Challenge:** We have thousands of flip-flops that all need to see the clock edge at the same time.

**Solution: Global clock tree**
- **H-tree topology:** Balanced routing that fans out to all regions
  - Ensures all flip-flops see the clock edge within ~100 picoseconds (skew)
- **Clock buffers (BUFG):** Special high-fanout buffers
  - Can drive thousands of flip-flops without degradation

**PLLs and MMCMs (Clock generation):**
- **Input:** 100 MHz reference clock
- **Output:** 225 MHz and 450 MHz for our design
- **How they work:**
  - PLL: Phase-Locked Loop tracks input and generates multiples
  - VCO: Voltage-Controlled Oscillator runs at high frequency (900-2000 MHz)
  - Dividers: Divide VCO output to get desired frequencies

---

### **FPGA Memory: Block RAM (BRAM)**

**What it is:** Dedicated memory blocks built into the FPGA (separate from logic fabric)

**Why BRAMs exist:**
- Could build memory using LUTs (they're SRAMs after all)
- But: Inefficient - wastes logic resources
- Better: Dedicated memory blocks optimized for storage

**BRAM technology:**
- **6-transistor SRAM cell:** 2 cross-coupled inverters + 2 access transistors
- **Static storage:** Unlike DRAM, no refresh needed (data persists as long as powered)
- **Trade-off:** More transistors per bit than DRAM, but faster

**RAMB36E2 primitive (basic BRAM block):**
- **Capacity:** 36 Kilobits (36 Kb = 4.5 KB)
- **Configurable width:** 1 to 72 bits wide
  - Width × Depth = 36K bits
  - Examples: 36K×1, 18K×2, 9K×4, ..., 512×72
- **Dual-port:** Can read and write simultaneously on two independent ports

**Access timing:**
- **Synchronous:** Operates on clock edges (not asynchronous like CPU cache)
- **Latency:** 2-3 clock cycles
  - Cycle 0: Present address
  - Cycle 1: Internal row decode
  - Cycle 2: Data valid on output

**Our usage:**
- Configuration: 32,768 addresses × 256 bits wide
- Uses: 256 RAMB36 primitives (each configured and then address-mapped together)

---

### **FPGA Memory: UltraRAM (URAM)**

**What it is:** Higher-density memory blocks (like BRAM but bigger)

**Technology:**
- **1T1C DRAM-like cell:** 1 transistor + 1 capacitor (similar to DDR4 cells)
- **On-chip:** Integrated into FPGA die (not external)
- **Advantage:** 4× density vs BRAM (288 Kb vs 36 Kb per primitive)
- **Trade-off:** Requires refresh (but automatic, handled by primitive logic)

**URAM288 primitive:**
- **Capacity:** 288 Kilobits (36 KB)
- **Configuration:** 4096 words × 72 bits (typical for our design)

**Access timing:**
- **Synchronous:** Operates on clock edges
- **Latency:** 1 clock cycle @ 450 MHz (faster than BRAM!)
  - Cycle N: Address presented
  - Cycle N+1: Data valid

**Refresh:**
- Automatic and transparent to user logic
- Built into the primitive controller

**Our usage:**
- 16 banks × 288 Kb = 4.5 Megabits total
- Stores neuron state information

---

### **FPGA Hard IP Blocks**

**What are "Hard IP" blocks?**
- Most of the FPGA is reconfigurable fabric (LUTs, FFs, routing)
- Some functions are implemented as fixed silicon (not programmable)
- These are "Hard IP" blocks

**Why hard IP?**
- **Performance:** Dedicated circuits run faster than fabric implementation
- **Efficiency:** Use less power and less die area
- **Interfaces:** Some protocols require precise timing (hard to achieve in fabric)

**Our FPGA's Hard IP:**

**1. PCIe block:**
- **Location:** Fixed position on die corner (near pins)
- **Contains:** SerDes (serializer/deserializer), PHY, MAC layers
- **Advantage:** Meets PCIe Gen3 timing requirements reliably

**2. HBM interface controllers:**
- **Purpose:** Interface to High Bandwidth Memory (see next section)
- **Provides:** 32 independent AXI ports (one per HBM channel)
- **Why hard:** Timing-critical signaling for high-speed memory

---

## Part 4: Ultra-High-Performance Memory

We've covered DDR4 (main system memory, moderate bandwidth). Now let's look at specialized memory for extreme bandwidth.

**Where HBM fits in the system:**
- **HBM is physically attached to the FPGA** - they are packaged together on the same silicon interposer
- **It's a customization/option:** Not all FPGAs have HBM; our XCVU37p model includes 8 GB of HBM2
- **Think of it as:** The FPGA's "private" high-speed memory, while DDR4 is the host's "shared" memory
  - Host DDR4: Shared between CPU and FPGA, accessed via PCIe DMA
  - FPGA HBM2: Exclusive to FPGA, direct connection (no PCIe), much faster

### **HBM2 (High Bandwidth Memory) - Why It Exists**

**The bandwidth problem:**
- Our neural network simulation needs to read/write neuron states very quickly
- DDR4 provides ~25 GB/s per channel (good for general-purpose computing)
- But our application needs ~400-900 GB/s (much higher!)

**The solution: HBM2**
- Specialized memory technology optimized for bandwidth
- Trade-offs: More expensive, less capacity than DDR4
- Achieved bandwidth: ~920 GB/s theoretical, ~400 GB/s practical

**How HBM achieves high bandwidth:**
- **Wide buses:** 1024 bits per stack (vs 64 bits for DDR4) = 16× wider
- **High frequency:** 1800 MT/s (similar to DDR4)
- **Calculation:** 1024 bits × 1800 MT/s = 230 GB/s per stack
- **4 stacks:** 230 GB/s × 4 = 920 GB/s total

---

### **HBM2 Physical Structure**

**3D Stacking:**
- **Vertical stack:** 4 DRAM dies stacked on top of each other
- Each die: 512 Megabits (64 MB)
- 4 dies per stack × 4 stacks = 8 GB total capacity

**Through-Silicon Vias (TSVs):**
- **What they are:** Vertical conductors drilled through silicon die
- **Diameter:** ~50 micrometers
- **Purpose:** Connect dies in the stack (data buses, power, ground)
- **Advantage:** Very short distance = low latency, enables wide buses

**Silicon Interposer:**
- **What it is:** Large (~1000 mm²) silicon substrate
- **Sits under:** Both HBM stacks and FPGA die
- **Connection:** Microbumps (~50 µm pitch) connect components to interposer
- **Why silicon?**
  - Much finer pitch than PCB (printed circuit board) routing
  - Enables the 1024-bit wide buses (couldn't route this on PCB)

**The complete package:**
```
[    FPGA die    ] [HBM] [HBM] [HBM] [HBM]
  |  |  |  |  |     ||||   ||||   ||||   ||||  ← microbumps
[========== Silicon Interposer =============]
  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
[============ Package substrate =============]
```

---

### **HBM2 DRAM Cell (Storage Element)**

Like DDR4, HBM uses DRAM cells:

**1T1C structure:**
- **Capacitor:** Stores charge (~30 fF, ~10,000 electrons)
  - Charged = 1, discharged = 0
- **Transistor:** Access gate (connects capacitor to bitline)

**Write operation:**
1. Activate wordline (turns on transistor)
2. Drive bitline to VDD (logic 1) or GND (logic 0)
3. Capacitor charges/discharges through transistor
4. Deactivate wordline (isolates capacitor)

**Read operation (destructive):**
1. Precharge bitline to VDD/2 (midpoint voltage)
2. Activate wordline (connects capacitor to bitline)
3. Capacitor shares charge with bitline
   - Stored 1: Bitline rises slightly above VDD/2
   - Stored 0: Bitline falls slightly below VDD/2
4. Sense amplifier detects this tiny voltage change
5. **Restore:** Write value back (read destroys the capacitor charge)

**Refresh requirement:**
- Every 64ms, all rows must be read and rewritten
- Needed because capacitor leaks charge (quantum tunneling through dielectric)
- Automatic, handled by memory controller

---

### **HBM2 Memory Organization**

**Hierarchy:**
```
Stack (4 total)
  └─ Channel (8 per stack)
      └─ Bank (16 per channel)
          └─ Row (16,384 per bank)
              └─ Column (1,024 per row)
```

**Row buffer concept:**
- When you activate a row, the entire row (512 bytes) is read into a buffer
- Subsequent accesses to the same row are fast (~10 ns) - "page hit"
- Accessing a different row requires:
  1. Close current row (precharge)
  2. Open new row (activate)
  - This is slower (~50 ns) - "page miss"

**Performance implications:**
- **Best case (sequential access within row):** Very fast, high bandwidth
- **Worst case (random access across rows):** Slower, reduced bandwidth
- **Our design:** Tries to access memory sequentially to maximize page hits

---

### **HBM2 AXI4 Interface**

**What is AXI4?**
- **AXI:** Advanced eXtensible Interface (ARM standard)
- **Purpose:** Standard protocol for connecting memory and devices in hardware
- **Why standard?** Different IP blocks can interoperate (like USB for internal hardware)

**Five independent channels:**

**1. Write Address (AW):** Master sends where to write
- Signals: AWADDR (address), AWLEN (burst length), AWVALID/AWREADY

**2. Write Data (W):** Master sends what to write
- Signals: WDATA (data), WSTRB (byte enables), WVALID/WREADY

**3. Write Response (B):** Slave acknowledges completion
- Signals: BRESP (response code), BVALID/BREADY

**4. Read Address (AR):** Master requests data
- Signals: ARADDR (address), ARLEN (burst length), ARVALID/ARREADY

**5. Read Data (R):** Slave returns data
- Signals: RDATA (data), RRESP (response), RVALID/RREADY

**Key features:**

**Decoupling:**
- Address and data channels are independent
- Can send multiple read addresses, then receive data later
- Enables pipelining and out-of-order completion

**Handshake protocol (VALID/READY):**
- **Source asserts VALID:** "My data is ready"
- **Destination asserts READY:** "I can accept data"
- **Transfer occurs when:** VALID AND READY (both high)
- This allows flow control (receiver can apply backpressure)

**Bursts:**
- Single address can request multiple data beats (up to 256)
- Example: Address=0x1000, Length=16 → returns 16 consecutive words
- Amortizes address overhead (one address, many data)

---

### **HBM2 Access Latency**

**Best case (row hit): ~50 ns**
- Address decode: 5 ns
- Column select: 10 ns
- Sense amplifier: 10 ns
- Data serialization: 10 ns
- AXI handshake: 15 ns

**Worst case (row miss): ~200 ns**
- Precharge old row: 30 ns
- Activate new row: 50 ns
- Column access: 50 ns
- (rest as above)

**Optimization in our design:**
- Prefetch next row during processing current data
- Pipelines operations to hide latency
- Access patterns designed for row locality

---

## Part 5: Data Movement Primitives

Finally, we need ways to move data between all these components (host memory, PCIe, FPGA logic, HBM). FIFOs are the basic building block.

### **FIFO (First-In-First-Out Buffer)**

**What it is:** A hardware queue - data comes out in the same order it went in.

**Why FIFOs exist:**
- **Problem 1:** Different components run at different speeds
  - Example: PCIe sends data in bursts, FPGA processing is continuous
  - FIFO smooths out the rate mismatch
- **Problem 2:** Different components run on different clocks
  - Example: PCIe side at 225 MHz, HBM side at 450 MHz
  - FIFO safely transfers data between clock domains

**Think of it like:**
- A line at a coffee shop (first person in line is first served)
- A pipe (data flows through, can't jump ahead or reorder)

---

### **FIFO Implementation (Xilinx FIFO36E2)**

**Storage:** Uses BRAM36 primitive (36 Kb SRAM block)

**Pointers:**
- **Write pointer (WP):** Points to next location to write
- **Read pointer (RP):** Points to next location to read
- Both are counters that increment with each operation

**Status signals:**
- **Empty:** WP == RP (no data to read)
- **Full:** (WP + 1) mod DEPTH == RP (no space to write)
- Software/hardware checks these before reading/writing

**FWFT mode (First-Word Fall-Through):**
- Normal FIFO: Must assert RD_EN, wait 1 cycle, then data appears
- FWFT FIFO: Data appears on output port as soon as EMPTY goes low
- Implementation: Extra output register + bypass mux
- Advantage: Zero-latency read (useful for streaming pipelines)

---

### **Asynchronous FIFO (Clock Domain Crossing)**

**The problem:**
- Write side: 225 MHz clock
- Read side: 450 MHz clock
- Cannot directly compare pointers (in different clock domains!)

**Why this is hard:**
- If a signal changes in one clock domain and is read in another, **metastability** can occur
- Metastability: Flip-flop input violates setup/hold time → output voltage stuck between 0 and 1
- Can take nanoseconds (or longer!) to resolve to a valid logic level
- During metastability, output can oscillate or produce glitches

**The solution: Gray code + 2-FF synchronizer**

**Gray code:**
- Special binary encoding where only 1 bit changes per increment
- Examples:
  - Binary: 3→4 is 011→100 (3 bits change)
  - Gray: 3→4 is 010→110 (only 1 bit changes)
- **Why this helps:** If we catch the pointer mid-transition, we're only off by ±1 (not random garbage)

**2-FF synchronizer:**
```verilog
always @(posedge rd_clk) begin
  wptr_gray_sync1 <= wptr_gray;      // First FF (may go metastable)
  wptr_gray_sync2 <= wptr_gray_sync1; // Second FF (stable output)
end
```

**How it works:**
1. First FF captures signal from other clock domain
   - May go metastable (voltage between 0 and 1)
2. One full clock period passes (2.2 ns @ 450 MHz)
   - Metastability has time to resolve
3. Second FF captures now-stable value
   - Guaranteed valid 0 or 1

**Empty/Full calculation:**
- **Empty:** Calculated in read domain using synchronized write pointer
  - "Is the read pointer caught up to where the writer was?"
- **Full:** Calculated in write domain using synchronized read pointer
  - "Is the write pointer about to lap the reader?"

**Timing conservative:**
- Due to synchronization, pointers are slightly "old" (2-3 cycles)
- This makes FIFO appear fuller/emptier than reality (safe direction)
- Means: Slightly less efficient, but never corrupts data

---

### **FIFOs in Our System**

**Input/Output FIFOs (PCIe ↔ FPGA fabric):**
- **Width:** 512 bits (64 bytes) - matches PCIe TLP data width
- **Depth:** 512 entries
- **Purpose:** Buffer data transfers between PCIe and processing logic
- **Async:** Crosses clock domain (PCIe clock → fabric clock)

**Pointer FIFOs (HBM data → neuron groups):**
- **Width:** 32 bits
- **Depth:** 512 entries
- **Purpose:** Distribute memory addresses/pointers to different processing units
- **Sync:** Same clock domain (can use simpler FIFO)

**Spike FIFOs (neurons → spike controller):**
- **Width:** 17 bits (neuron ID + metadata)
- **Depth:** 512 entries
- **Purpose:** Collect spike events from processing units
- **Async:** Different processing clocks converge to controller clock

**Key insight:**
- FIFOs are everywhere in the design
- They buffer, smooth rate mismatches, and cross clock domains
- Simple concept, but essential for making everything work together

---

## Summary: How It All Fits Together

**Data flow example: Host sends commands to FPGA**

1. **Host preparation:**
   - Software creates command array in DDR4 memory (system RAM)
   - Gets physical address of buffer (e.g., 0x123456000)

2. **MMIO handoff:**
   - Software writes address to FPGA register via PCIe MMIO
   - "Here's the command buffer: 0x123456000"

3. **DMA transfer:**
   - FPGA reads descriptor from its register
   - FPGA initiates PCIe memory read transactions
   - Requests data from host memory at 0x123456000
   - Host responds with command data over PCIe

4. **Buffering:**
   - PCIe data arrives in bursts → stored in input FIFO
   - FIFO crosses clock domain (225 MHz → 450 MHz)

5. **Processing:**
   - FPGA logic reads commands from FIFO
   - Uses commands to coordinate processing
   - Reads/writes neuron data from/to HBM2 via AXI4

6. **HBM access:**
   - FPGA sends AXI read request to HBM
   - HBM controller activates row, reads data
   - Data returns via AXI → FPGA processes

7. **Results:**
   - Spike events → spike FIFOs
   - Output data → output FIFO → PCIe → host DDR4

**Every component has a role:**
- **DDR4:** Large staging area for host data
- **PCIe:** High-speed highway connecting host and FPGA
- **DMA:** Lets FPGA access host memory without CPU
- **FPGA:** Reconfigurable logic for parallel processing
- **BRAM/URAM:** Fast on-chip memory for state
- **HBM2:** Ultra-high-bandwidth memory for large datasets
- **FIFOs:** Buffers and clock domain crossings throughout

This is how modern heterogeneous computing systems work: specialized hardware (FPGA) accelerates specific tasks, while remaining integrated with general-purpose host system via high-speed interconnects.

---

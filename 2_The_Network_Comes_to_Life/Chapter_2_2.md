---
title: "2.2 The Code Behind Execution"
parent: "Chapter 2: The Network Comes to Life"
nav_order: 2
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

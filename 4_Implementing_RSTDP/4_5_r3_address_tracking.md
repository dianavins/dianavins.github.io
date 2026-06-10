---
title: "4.5 Tracking R3 Addresses Through HBM Bursts"
parent: "4 Implementing RSTDP"
nav_order: 5
---

# 4.5 Tracking Region 3 Addresses Through HBM Bursts

The Active Synapse Buffer in the next page wants to record, for every 512-bit synapse packet that arrives during Phase 2, **which R3 address it came from, so that we can determine the presynaptic neuron / specific synapse the pre/post spiking event is targeting.** Since the R4 address is computed from the R3 address, the synapse is easy to find from there. That information is not part of the packet — AXI4 read responses carry data and a small response ID, but not the address you asked for. The address has to be reconstructed.

This page is about how `hbm_processor.v` does that reconstruction. It's a self-contained piece of plumbing; once you understand it, the ASB in [4.6](4_6_coincidence_detection), which holds memory of the active synapses for the current and previous timesteps becomes trivial.

---

## The problem

Phase 2 of a timestep sends a sequence of read commands to HBM for synapse data. Each command is an AXI4 burst — one address-channel transfer (`hbm_arvalid`/`hbm_arready`) followed by up to 16 data-channel beats (`hbm_rvalid`/`hbm_rready`). The data beats stream back without re-stating the address.

```
TX (commands)        AR    AR        AR
                     ┌─┐   ┌─┐       ┌─┐
                  ───┘ └───┘ └───────┘ └────
                     0x8000│         │0x8003
                           0x8001

RX (beats)             R R   R R R R   R R R R
                       ┌─┐   ┌─────┐   ┌─────┐
                  ─────┘ └───┘     └───┘     └───
                       ↑   ↑       ↑   ↑       ↑
                    which addr does each beat belong to?
```

AXI4 guarantees in-order responses per `arid`, and `hbm_processor` uses `arid=0` for everything, so the beats arrive in the same order the addresses were sent. That's the only handle we have.

The original processor didn't need to know — Phase 2 just streams beats into IEP for membrane accumulation, no per-beat address bookkeeping. R-STDP needs the address because the ASB records it.

---

## The solution: a burst-descriptor FIFO

`hbm_processor.v` declares a small FIFO that the TX side pushes burst descriptors into and the RX side pops in lockstep. Source at [`hbm_processor.v:222`](../../hardware_code_rstdp/src/hbm_processor.v#L222):

```verilog
// Region 3 address tracking: burst descriptor FIFO (TX→RX inside hbm_processor)
localparam BDFIFO_DEPTH     = 128;
localparam BDFIFO_ADDR_BITS = 7;

reg [26:0] bdfifo_mem [0:BDFIFO_DEPTH-1];  // {ptr_addr[22:0], ptr_burst[3:0]}
reg [BDFIFO_ADDR_BITS:0] bdfifo_wr_ptr;
reg [BDFIFO_ADDR_BITS:0] bdfifo_rd_ptr;
```

Each entry is 27 bits: a 23-bit base address and a 4-bit burst length. 128 entries is more than enough for any realistic Phase 2 — the FIFO clears between timesteps.

### TX side: push one descriptor per burst command

In `TX_STATE_SEND_POINTER_READ_COMMANDS` at [line 684](../../hardware_code_rstdp/src/hbm_processor.v#L684):

```verilog
TX_STATE_SEND_POINTER_READ_COMMANDS: begin
    hbm_arvalid = 1'b1;
    if (hbm_arready) begin
        ptr_addr_inc = 1'b1;
        bdfifo_push  = 1'b1;     // ← push (ptr_addr, ptr_burst) into BDFIFO
        if (ptr_ctr[8:4]==ptr_len[8:4])
            tx_next_state = TX_STATE_POP_POINTER_FIFO;
    end
end
```

`bdfifo_push` strobes for one cycle each time HBM accepts an AR. The push uses the **current** `ptr_addr` and `ptr_burst` — the values that just got driven onto the AR channel:

```verilog
if (bdfifo_push && !bdfifo_full) begin
    bdfifo_mem[bdfifo_wr_ptr[BDFIFO_ADDR_BITS-1:0]] <= {ptr_addr, ptr_burst};
    bdfifo_wr_ptr <= bdfifo_wr_ptr + 1;
end
```

So after sending three bursts at addresses `A`, `B`, `C` with lengths `lA`, `lB`, `lC`, the BDFIFO contains `{A, lA}`, `{B, lB}`, `{C, lC}` in order.

### RX side: pop one descriptor per burst, count beats inside it

The RX-side reconstruction lives in its own sequential block at [line 896](../../hardware_code_rstdp/src/hbm_processor.v#L896). Five state registers do the work:

```verilog
reg [22:0] rx_burst_base;       // base addr of the burst we're currently receiving
reg  [3:0] rx_burst_len;        // length of that burst
reg  [3:0] rx_burst_beat_ctr;   // how many beats into it we are
reg [22:0] rx_beat_addr;        // address of the beat currently arriving
reg        rx_burst_active;     // 1 while we're in the middle of a burst
reg [22:0] rx_packet_addr;      // address of the first beat in a 512-bit pair
```

When the RX FSM is in `RX_STATE_READ_SYNAPSE_DATA` (Phase 2's read state) and no burst is active, it pops the next BDFIFO descriptor:

```verilog
if (!rx_burst_active && !bdfifo_empty) begin
    rx_burst_base     <= bdfifo_dout[26:4];
    rx_burst_len      <= bdfifo_dout[3:0];
    rx_beat_addr      <= bdfifo_dout[26:4];
    rx_burst_beat_ctr <= 4'd0;
    rx_burst_active   <= 1'b1;
    bdfifo_pop        <= 1'b1;
end
```

For each valid beat, `rx_beat_addr` is the current address; it increments after the beat is consumed:

```verilog
if (exec_hbm_rvalidready_2x && rx_burst_active) begin
    // (packet assembly — see next section)
    rx_beat_addr      <= rx_beat_addr + 1'b1;
    rx_burst_beat_ctr <= rx_burst_beat_ctr + 1'b1;

    if (rx_burst_beat_ctr == rx_burst_len)
        rx_burst_active <= 1'b0;   // last beat → pull next descriptor
end
```

When the last beat of a burst arrives, `rx_burst_active` drops and the next cycle pulls the next descriptor.

---

## Putting it onto the wire: `exec_hbm_region3addr`

What downstream consumers actually see is one output port: a 23-bit `exec_hbm_region3addr` register, declared on the module's port list at [line 51](../../hardware_code_rstdp/src/hbm_processor.v#L51):

```verilog
output reg [22:0] exec_hbm_region3addr,  // Region 3 address of current 512-bit synapse packet
```

This is updated on the same clock edge that the 512-bit packet becomes valid on `exec_hbm_rdata`. IEP latches both signals together.

### Why "current 512-bit packet" — and how `hbm_count` makes the pairing

HBM is 256 bits wide; IEP expects 512-bit packets (16 neuron-group slots × 32 bits). So `hbm_processor` combines two consecutive 256-bit beats into one 512-bit packet using a 1-bit toggle `hbm_count`:

```
hbm_rvalid
hbm_count       0           1           0           1
                ↓           ↓           ↓           ↓
              ┌─────┐     ┌─────┐     ┌─────┐     ┌─────┐
              │beat │     │beat │     │beat │     │beat │
              │  0  │     │  1  │     │  2  │     │  3  │
              └─────┘     └─────┘     └─────┘     └─────┘
                      ↑                       ↑
                  512b packet #0          512b packet #1
                  presented on            presented on
                  exec_hbm_rdata          exec_hbm_rdata
                  along with              along with
                  packet_addr = addr0     packet_addr = addr2
```

The RX address-reconstruction block aligns to this toggle: it latches `rx_packet_addr` on the **first** beat of the pair (`hbm_count==0`) and exposes it on `exec_hbm_region3addr` on the **second** beat (`hbm_count==1`), when the 512-bit packet is complete. Source at [line 930](../../hardware_code_rstdp/src/hbm_processor.v#L930):

```verilog
if (exec_hbm_rvalidready_2x && rx_burst_active) begin
    // First beat of pair: latch as packet address
    if (~hbm_count)
        rx_packet_addr <= rx_beat_addr;

    // Second beat of pair: output the latched address
    if (hbm_count)
        exec_hbm_region3addr <= rx_packet_addr;
    ...
end
```

So `exec_hbm_region3addr` is *the address of beat 0 of the current 512-bit packet*. That's the R3 address the host originally typed into the host compiler — exactly what the ASB needs to record.

---

## The `hbm_count` reset bug (read this if you touch the RX FSM)

If you're going to do anything that involves new RX states or extra HBM reads, you need to know about a bug that was fixed in step 8.

`hbm_count` toggles every time `hbm_rvalid && hbm_rready` fires. The WUE introduced in [4.7](4_7_weight_update_engine) issues **three** beats per coincFIFO entry (1 × R4 + 2 × R3). Three is odd. So after WUE finishes, `hbm_count` is left at `1`. On the next `exec_run`, Phase 2 would start receiving the *first* beat of a synapse pair while thinking it was the *second* — corrupting every 512-bit packet for the rest of the run.

The fix is at [`hbm_processor.v:972`](../../hardware_code_rstdp/src/hbm_processor.v#L972):

```verilog
always @(posedge clk) begin
    if (~resetn | tx_done_rst) begin   // ← tx_done_rst fires on exec_run
        hbm_rdata_lower <= 256'b0;
        hbm_rdata_upper <= 256'b0;
        hbm_count <= 1'b0;
    end
    else if (rx_curr_state == RX_STATE_WUE_WAIT_R4 && hbm_rvalid && hbm_rready) begin
        // FIX #3: Reset hbm_count when R4 beat is consumed, not conditionally in WAIT_R3_BEAT0.
        // If the first R3 beat arrives immediately (no idle bubble), the old reset was skipped.
        // Resetting here ensures beat 0 of R3 always sees hbm_count=0 → correct 512b assembly.
        hbm_count <= 1'b0;
    end
    else if (hbm_rvalid && hbm_rready) begin
        hbm_count <= ~hbm_count;
        ...
    end
end
```

Two distinct resets are in play here:

1. **`tx_done_rst` on `exec_run`** — clears `hbm_count` at the start of every new timestep so Phase 2 always begins aligned.
2. **WAIT_R4 consume** — clears `hbm_count` after the WUE's single-beat R4 read consumes a beat, so the first R3 beat that follows sees `hbm_count=0`.

A simulation-only assertion at [line 991](../../hardware_code_rstdp/src/hbm_processor.v#L991) catches future regressions:

```verilog
`ifdef SIM
always @(posedge clk) begin
    if (rx_curr_state == RX_STATE_PHASE1_DONE) begin
        if (hbm_count !== 1'b0)
            $error("[hbm_processor ASSERT] hbm_count=%0b != 0 at Phase 2 start ...");
    end
end
`endif
```

The full incident report is in [`CHANGES.md`](../../hardware_code_rstdp/CHANGES.md). The headline lesson: **any new state that consumes an odd number of HBM beats has to either reset `hbm_count` on entry or add a beat to make it even.**

---

## What downstream consumers see

From the IEP's perspective, the burst-descriptor FIFO and beat-pair assembly are invisible. IEP sees three signals fire together on the same clock edge:

| Signal | Width | Meaning |
|---|---|---|
| `exec_hbm_rvalidready` | 1 | "A new 512-bit synapse packet is on the bus this cycle." |
| `exec_hbm_rdata` | 512 | The packet itself. 16 × 32-bit slots, one per neuron group. |
| `exec_hbm_region3addr` | 23 | The R3 address of the packet's first beat. |

That's the interface the ASB reads from in [4.6](4_6_coincidence_detection).

---

## What to take from this page

- AXI burst responses come back in order but without addresses; you have to re-derive them.
- `hbm_processor.v` does that by pushing `(base, len)` into the **BDFIFO** when each burst is issued (TX) and counting beats inside each descriptor on receive (RX).
- The output is `exec_hbm_region3addr`, valid on the same edge as the 512-bit `exec_hbm_rdata` packet.
- `hbm_count` toggles to pair 256-bit beats into 512-bit packets. It must reset to `0` on `exec_run` (and after any odd-beat WUE read) or the next timestep's packets are misaligned.

Next: [4.6](4_6_coincidence_detection) — the Active Synapse Buffer uses `exec_hbm_region3addr` and the 16 target URAM addresses inside each packet to record a per-timestep "who-targeted-whom" table, then compares it against the next timestep's spikes.

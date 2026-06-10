---
title: "4.6 Coincidence Detection: The Active Synapse Buffer"
parent: "4 Implementing RSTDP"
nav_order: 6
---

# 4.6 Coincidence Detection: The Active Synapse Buffer

R-STDP's `Δw = R(t) · c(t)` update rule needs *coincidence* events as input. A coincidence is a single moment in time satisfying:

> a post-synaptic neuron crossed threshold this cycle **AND** a pre-synaptic source fired into it within the STDP window.

The processor's "STDP window" is one timestep: pre-synaptic activity counts if it happened in Phase 2 of the previous timestep. Spike-timing finer than one timestep is collapsed.

This page covers the hardware that detects coincidences and queues them up for the Weight Update Engine. The detection logic lives entirely inside `internal_events_processor.v` (no new modules) and adds three pieces:

1. The **Active Synapse Buffer (ASB)**, a double-buffered record of which neurons each synapse packet targeted in the previous timestep.
2. A **parallel comparator** that, for every URAM scan address in Phase 1, checks whether the ASB(prev) contains a matching entry.
3. A small FSM that pushes one **coincFIFO** entry per coincident `(group, R3 address)` pair, plus a parallel path for *membrane-only* pushes (used to update neuron membrane potentials when no coincidence fires).

There's a companion text file in the repo, [`IEP_coincidence_logic_explained.txt`](../../hardware_code_rstdp/IEP_coincidence_logic_explained.txt), that walks every line. This page covers the same material at a slightly higher altitude but with the same vocabulary.

---

## What the ASB stores and why it's double-buffered

The ASB has to answer this question, posed during Phase 1 of timestep T+1:

> "For neuron group G scanning URAM address A, did any synapse packet processed during Phase 2 of timestep T target group G at address A? If so, what was the R3 address of that packet?"

The straightforward implementation: a table keyed by `(group, address)`, populated during Phase 2, queried during the next Phase 1. The ASB is that table, with three twists.

### Twist 1: double-buffered, not cleared

Phase 1 of T+1 happens before Phase 2 of T+1 finishes. If we cleared the table at the start of T+1 we'd lose T's data before reading it. If we didn't clear it at all we'd accumulate stale entries forever. So the ASB has **two** buffers and a 1-bit selector `asb_sel`:

- During Phase 2 of T, IEP writes into buffer `asb_sel` (call it the *current* buffer).
- On `exec_run` of T+1, `asb_sel` flips. The buffer that was current during T becomes *prev*.
- Phase 1 of T+1 reads from `~asb_sel` (the prev buffer).
- Phase 2 of T+1 writes into the new current, overwriting last timestep's prev data.

```
T   exec_run                            T+1  exec_run
        │                                       │
        ▼                                       ▼
  ┌────────────────────────┐             ┌────────────────────────┐
  │  asb_sel = 0           │             │  asb_sel = 1           │
  │  Phase 2 writes  → [0] │             │  Phase 2 writes  → [1] │
  │  Phase 1 reads   ← [1] │             │  Phase 1 reads   ← [0] │
  │  (stale, ok)           │             │  (T's writes)          │
  └────────────────────────┘             └────────────────────────┘
```

### Twist 2: by group, not by address

Phase 1 scans URAM linearly: address 0, 1, 2, ... For each scan address, all 16 neuron groups read in parallel. The natural query is "did any ASB entry have group G targeting *this* scan address?", so the ASB is indexed by *entry number* (a write counter), and for each entry it stores the 16 target addresses, one per group.

```
asb_target[buffer][entry][group]  →  13-bit URAM full address
```

There are 64 entries per buffer (`ASB_DEPTH = 64`). 64 was chosen to cover a worst-case Phase 2 (every group spiking, every group's pointer chain emitting several packets) without going so large that the comparator generate block blows out routing.

### Twist 3: the R3 address is per-entry, not per-group

The R3 address identifies *which pre-synaptic axon* sent the packet. That's the same for all 16 groups within one packet, since a single 512-bit packet is exactly one R3 row. So the R3 address is stored once per entry:

```
asb_region3[buffer][entry]   →  23-bit R3 row address
```

### Declarations

[`internal_events_processor.v:316`](../../hardware_code_rstdp/src/internal_events_processor.v#L316):

```verilog
localparam ASB_DEPTH = 64;

reg [22:0]  asb_region3 [0:1][0:ASB_DEPTH-1];          // R3 addr per entry, per buffer
reg [12:0]  asb_target  [0:1][0:ASB_DEPTH-1][0:15];    // target URAM addr per group, per entry, per buffer
reg         asb_valid   [0:1][0:ASB_DEPTH-1];          // valid bit per entry, per buffer
reg [5:0]   asb_wr_count;                              // write pointer for current buffer
reg [5:0]   asb_prev_depth;                            // how many entries were in prev buffer
reg         asb_sel;                                   // which buffer is current
```

Total state: 2 buffers × 64 entries × (23 + 16×13 + 1) bits ≈ 30 Kb. Modest.

---

## Writing the ASB during Phase 2

The ASB write block at [`internal_events_processor.v:619`](../../hardware_code_rstdp/src/internal_events_processor.v#L619) has three mutually exclusive branches.

### Branch A: reset

Clears `asb_wr_count`, `asb_sel`, and all `asb_valid` bits.

### Branch B: `exec_run` (buffer swap)

```verilog
end else if (exec_run) begin
    asb_prev_depth <= asb_wr_count;
    asb_wr_count <= 0;
    asb_sel <= ~asb_sel;
    for (e = 0; e < ASB_DEPTH; e = e + 1)
        asb_valid[~asb_sel][e] <= 1'b0;
end
```

Three things happen on every `exec_run`:

1. Remember how many entries this timestep wrote (`asb_prev_depth`) so reads in the *next* timestep can stop early. (Not strictly required for correctness, since `asb_valid` would do it, but useful for debug.)
2. Reset the write counter.
3. Flip the buffer selector and invalidate the new write buffer. Note the `~asb_sel`: that's the *new* prev buffer being cleared, which was the *old* current buffer two timesteps ago.

### Branch C: Phase 2 write

```verilog
end else if (exec_hbm_rvalidready_d1 &&
             exec_hbm_rx_phase1_done &&
             exec_uram_phase1_done && !exec_uram_phase2_done &&
             (curr_state == STATE_POP_PTR_FIFO) &&
             asb_wr_count < ASB_DEPTH) begin
    asb_region3[asb_sel][asb_wr_count] <= exec_hbm_region3addr;
    asb_target[asb_sel][asb_wr_count][0]  <= uram_raddr_0_full_reg;
    asb_target[asb_sel][asb_wr_count][1]  <= uram_raddr_1_full_reg;
    // ... groups 2-15 ...
    asb_target[asb_sel][asb_wr_count][15] <= uram_raddr_15_full_reg;
    asb_valid[asb_sel][asb_wr_count] <= 1'b1;
    asb_wr_count <= asb_wr_count + 1;
end
```

There's a lot of gating, all of it earning its keep:

| Condition | Why it's there |
|---|---|
| `exec_hbm_rvalidready_d1` | One-cycle delay of `exec_hbm_rvalidready`. On the cycle the packet arrives, IEP is still *decoding* the 16 target addresses out of `exec_hbm_rdata`. One cycle later, the registered `uram_raddr_X_full_reg` values are stable. |
| `exec_hbm_rx_phase1_done && exec_uram_phase1_done && !exec_uram_phase2_done` | We're in Phase 2: Phase 1 of the same timestep is done but Phase 2 hasn't finished. |
| `curr_state == STATE_POP_PTR_FIFO` | IEP's main FSM state for Phase 2 synapse processing. `exec_hbm_rvalidready_d1` can also fire during Phase 1 (pointer reads); without this guard, pointer data would be misrecorded as synapse data. |
| `asb_wr_count < ASB_DEPTH` | Buffer not full. Synapses past entry 63 silently fall on the floor. |

The R3 address comes straight from the `hbm_processor` output covered in [4.5](4_5_r3_address_tracking). The 16 target URAM addresses come from `uram_raddr_X_full_reg`, which are registered versions of the decoded `[28:16]` bit slices of each 32-bit synapse slot. Decoding happens in the combinational `uram_raddr_X_full` block at [`internal_events_processor.v:909`](../../hardware_code_rstdp/src/internal_events_processor.v#L909) (the `if (~exec_uram_phase2_done)` branch).

---

## The Phase 1 comparator: `coinc_detected[15:0]`

In Phase 1 of the next timestep, IEP scans URAM addresses sequentially. For every scan address, the comparator answers: *for each of the 16 groups, did any valid ASB(prev) entry target this address?*

Implementation is a single generate block at [`internal_events_processor.v:665`](../../hardware_code_rstdp/src/internal_events_processor.v#L665):

```verilog
wire [ASB_DEPTH-1:0] asb_match [0:15];
wire [15:0] coinc_detected;

generate
    for (gi = 0; gi < 16; gi = gi + 1) begin : gen_group_match
        for (ei = 0; ei < ASB_DEPTH; ei = ei + 1) begin : gen_entry_match
            assign asb_match[gi][ei] = asb_valid[~asb_sel][ei] &&
                                       (asb_target[~asb_sel][ei][gi] == uram_waddr[gi]);
        end
        assign coinc_detected[gi] = |asb_match[gi];
    end
endgenerate
```

That's 16 × 64 = 1024 13-bit comparators in parallel, plus 16 OR reductions. All combinational; results are valid every cycle Phase 1's URAM scan presents a new address.

### Why `uram_waddr[gi]` instead of `uram_raddr[gi]`

The URAM has 1-cycle read latency. So `uram_raddr` (the live read port) is always one address ahead of the data currently sitting in `uram_rmwdata`. The register `uram_waddr[gi]` lags `uram_raddr` by one cycle, holding the address whose *data* is on `uram_rmwdata` right now. That's the address we want to compare, because `coinc_spiked` (next section) is also computed from `uram_rmwdata`. Pipeline-aligned.

### What `coinc_detected[G]` actually means

`coinc_detected[G] = 1` ⇒ "for group G, the URAM address currently being read had at least one synapse packet target it in the previous timestep's Phase 2." That's the **pre-synaptic was active** half of coincidence.

### Recovering which R3 caused the match

`asb_matched_r3[G]` is a combinational reduction at [`internal_events_processor.v:686`](../../hardware_code_rstdp/src/internal_events_processor.v#L686):

```verilog
reg [22:0] asb_matched_r3 [0:15];
always @(*) begin : asb_r3_extract
    integer g, m;
    for (g = 0; g < 16; g = g + 1) begin
        asb_matched_r3[g] = 23'd0;
        for (m = ASB_DEPTH-1; m >= 0; m = m - 1) begin
            if (asb_match[g][m])
                asb_matched_r3[g] = asb_region3[~asb_sel][m];
        end
    end
end
```

The reverse loop is deliberate: earlier-inserted entries win, deterministically. With a forward loop, later-write-wins would be non-deterministic with respect to packet insertion order. (Multiple ASB entries can match the same `(group, target)` when more than one R3 packet projects to the same post-synaptic neuron via that group.) This is the "secondary issue" fix from [`RTL_fixes_explained.txt`](../../hardware_code_rstdp/RTL_fixes_explained.txt).

---

## The spike check: `coinc_spiked[15:0]`

The other half of coincidence: did the post-synaptic neuron in group G just cross threshold? Source at [`internal_events_processor.v:708`](../../hardware_code_rstdp/src/internal_events_processor.v#L708):

```verilog
wire [15:0] coinc_spiked;
assign coinc_spiked[0]  = (uram_waddr[0][0])  ? (uram_rmwdata_upper_0  > threshold)
                                              : (uram_rmwdata_lower_0  > threshold);
// ... groups 1-15 ...
```

`uram_waddr[G][0]` selects upper vs. lower 36-bit half of the 72-bit URAM word, the same select that the membrane update logic uses. `uram_rmwdata_X` is the read-modify-write-hazard-resolved version of the URAM read data (covered in [Chapter 3.7](../3_Verilog_Files_Review/internal_events_processor)). Threshold is the same `threshold` register the original processor used.

### Why not just use `exec_uram_spiked`?

The original processor already has a 16-bit `exec_uram_spiked` signal computed at [`internal_events_processor.v:1865`](../../hardware_code_rstdp/src/internal_events_processor.v#L1865). Two reasons we don't reuse it:

1. **`exec_uram_spiked` is gated by `exec_uram_phase1_ready`**, a registered signal that asserts in `STATE_PUSH_PTR_FIFO`. Because it's registered, it stays low for one cycle after the FSM enters that state. During that one cycle, the URAM scan is *already* presenting address 0, and `exec_uram_spiked` is forced to zero. Result: the spike at scan address 0 is permanently lost.
2. The selector bit (`uram_waddr[G][0]` vs `~uram_raddr_G_full[0]`) is equivalent algebraically but easier to reason about combinationally.

`coinc_spiked` is an ungated combinational function of pipeline-aligned signals. It's correct the instant URAM data appears.

This is the first of three RTL fixes documented in [`RTL_fixes_explained.txt`](../../hardware_code_rstdp/RTL_fixes_explained.txt). The lesson generalizes: **don't gate a Phase 1 result with a signal that takes a cycle to come up after the FSM enters the Phase 1 state.**

---

## Combining the two: `phase1_coincidence`

One line, [`internal_events_processor.v:727`](../../hardware_code_rstdp/src/internal_events_processor.v#L727):

```verilog
wire [15:0] phase1_coincidence = coinc_spiked & coinc_detected;
```

Bit G is 1 iff both halves of coincidence fired for group G at this scan address. Both inputs are combinational from `uram_waddr[gi]` and `uram_rmwdata_X`, same pipeline stage, so no delay alignment is needed.

---

## Pushing to the coincFIFO

The coincFIFO is the work queue feeding the WUE. Format:

```
coincfifo_din [39:0]
┌────┬────────────────────────┬───────────────────────┐
│ 39 │ 38:23                  │ 22:0                  │
│ do │  group_mask (one-hot   │  R3 address           │
│ wu │  for true coincidence; │  (which pre-synapse)  │
│    │  multi-hot for         │                       │
│    │  membrane-only)        │                       │
└────┴────────────────────────┴───────────────────────┘
```

- `do_wu` = 1 ⇒ a Phase 1 coincidence push (this is an R-STDP event; WUE will compute a weight update).
- `do_wu` = 0 ⇒ a Phase 2 membrane-only push (this is *not* an R-STDP event; see "the second push path" below).

### Why sequential, not bulk

`phase1_coincidence` can have up to 16 bits set simultaneously, each potentially corresponding to a different R3 address. The FIFO can only accept one 40-bit entry per cycle, so the push logic iterates: one group's entry per cycle until the pending mask is drained.

### State

[`internal_events_processor.v:731`](../../hardware_code_rstdp/src/internal_events_processor.v#L731):

```verilog
reg [15:0] coinc_pending_mask;
reg        coinc_pushing;
reg [22:0] coinc_r3_latched [0:15];
```

- `coinc_pending_mask`: which groups still need an entry pushed. Bits clear as entries go out.
- `coinc_pushing`: high while the priority chain is draining `coinc_pending_mask`.
- `coinc_r3_latched[G]`: snapshot of `asb_matched_r3[G]` taken at detection time. Necessary because `asb_matched_r3` is combinational and changes every cycle as `uram_waddr` advances.

### Latch

```verilog
if ((curr_state == STATE_PUSH_PTR_FIFO) && !exec_uram_phase1_done &&
    phase1_coincidence != 16'd0) begin
    coinc_pending_mask <= phase1_coincidence;
    coinc_pushing <= 1'b1;
    for (cl = 0; cl < 16; cl = cl + 1)
        coinc_r3_latched[cl] <= asb_matched_r3[cl];
end
```

Same `curr_state == STATE_PUSH_PTR_FIFO` guard as `coinc_spiked` to avoid the `exec_uram_phase1_ready` 1-cycle startup gap.

### Drain (priority chain)

When `coinc_pushing` is high, a long priority `if/else if` chain fires for the lowest set bit in `coinc_pending_mask`:

```verilog
if      (coinc_pending_mask[0])  begin coincfifo_wren <= 1'b1; coincfifo_din <= {1'b1, 16'h0001, coinc_r3_latched[0]};  coinc_pending_mask[0]  <= 1'b0; end
else if (coinc_pending_mask[1])  begin coincfifo_wren <= 1'b1; coincfifo_din <= {1'b1, 16'h0002, coinc_r3_latched[1]};  coinc_pending_mask[1]  <= 1'b0; end
// ... groups 2-14 ...
else if (coinc_pending_mask[15]) begin coincfifo_wren <= 1'b1; coincfifo_din <= {1'b1, 16'h8000, coinc_r3_latched[15]}; coinc_pending_mask[15] <= 1'b0; end
```

One group per cycle. The group mask in each entry is *one-hot*: exactly one of `0x0001`, `0x0002`, ..., `0x8000`. The WUE decodes that to a group index via `onehot_to_idx()`.

### Exit

`coinc_pushing` doesn't drop until both `coinc_pending_mask == 0` *and* `phase1_coincidence == 0`. Why both? The Phase 1 URAM pipeline keeps `phase1_coincidence` high for ~3 cycles per spiking row (read latency × scan rate). If `coinc_pushing` dropped as soon as the mask emptied, the latch above would fire again on the next cycle while the same combinational coincidence is still asserted, producing a duplicate push. Holding `coinc_pushing` high until the coincidence drops avoids that.

---

## The second push path: Phase 2 membrane-only pushes

This part trips people up because it's a *separate* push from the Phase 1 coincidence push and uses the *same* FIFO entry format with `do_wu = 0`.

### Why it exists

Phase 2 of the original processor (Chapter 2.2) accumulates synaptic weights into membrane potentials. In the R-STDP design, that accumulation runs through a different code path when `exec_bram_spiked != 0` (i.e., some axon group fired this timestep). The membrane update happens *via the WUE's R3 read*, not via the normal Phase 2 ptrFIFO synapse delivery. The membrane-only push is how IEP tells WUE: "I need you to read R3 at this address even though no coincidence happened, just to deliver the synapse to the membrane update logic."

The mem-only push entry carries `do_wu = 0`, so WUE knows to read R3 but *not* to compute or write a new weight. The reward gate ([4.4](4_4_reward_and_address_mapping)) doesn't apply, since neither path writes weight when `do_wu` is 0.

### Where it fires

[`internal_events_processor.v:811`](../../hardware_code_rstdp/src/internal_events_processor.v#L811):

```verilog
end else if (!phase2_packet_push_seen &&
              (curr_state == STATE_POP_PTR_FIFO) &&
              exec_hbm_rvalidready_d1 &&
              exec_hbm_rx_phase1_done &&
              !exec_hbm_rx_phase2_done_d1 &&
              exec_bram_spiked != 16'd0 &&
              !coinc_r3_match &&
              !coincfifo_full) begin
    coincfifo_wren          <= 1'b1;
    coincfifo_din           <= {1'b0, exec_bram_spiked, exec_hbm_region3addr};
    phase2_packet_push_seen <= 1'b1;
end
```

The conditions split into three categories:

| Condition | Why |
|---|---|
| `curr_state == STATE_POP_PTR_FIFO`, `exec_hbm_rvalidready_d1`, `exec_hbm_rx_phase1_done`, `!exec_hbm_rx_phase2_done_d1` | "A Phase 2 synapse packet just arrived." |
| `exec_bram_spiked != 16'd0` | "An axon group fired this timestep, so there's a real membrane update to do via R3." |
| `!phase2_packet_push_seen`, `!coinc_r3_match`, `!coincfifo_full` | Suppression: don't push twice for the same packet, and don't push a membrane-only entry if a Phase 1 coincidence push already covered this R3 address. |

The third row is FIX #2 from `RTL_fixes_explained.txt`. The original code had a per-timestep one-shot (`mem_only_push_done`) that incorrectly suppressed pushes for *all subsequent packets* in the timestep after the first push. The replacement is `phase2_packet_push_seen`, which resets at every new packet boundary:

```verilog
if ((curr_state == STATE_POP_PTR_FIFO) && exec_hbm_rvalidready)
    phase2_packet_push_seen <= 1'b0;
```

And it carries the *correct* R3 address: `exec_hbm_region3addr`, not a slice of payload bits. This is FIX #1 from the same file.

A simulation assertion at the end of the file catches future regressions on both fixes:

```verilog
`ifdef SIM
always @(posedge clk) begin
    if (coincfifo_wren && !coincfifo_din[39] && (curr_state == STATE_POP_PTR_FIFO)) begin
        if (coincfifo_din[22:0] !== exec_hbm_region3addr)
            $error("[IEP ASSERT FIX1] mem-only push addr ... at time %0t", ...);
    end
end
`endif
```

### What about the group mask in the mem-only entry?

Bits `[38:23]` carry `exec_bram_spiked` directly. This is *multi-hot*, one bit per group with a firing axon. WUE handles this by reading the synapse packet and applying it across all set groups in the membrane update path; no weight update happens since `do_wu = 0`.

---

## Putting it all on a timeline

Single concrete example: 5-axon / 5-hidden / 5-output network from [Chapter 1.1](../1_Initializing_the_Network/Chapter_1_1). Say at timestep T axon `a0` fires; at timestep T+1, neuron `h0` crosses threshold; and a0→h0 weight is 1000 (encoded in R3 row `0x8000`, slot 0, group `G=0`).

```
                  T                                              T+1
─────────────────────────────────────────────  exec_run  ─────────────────────────────

Phase 0  host writes "a0 fires" into BRAM      (irrelevant)
Phase 1  pointer collection (unchanged)        scan URAM addrs 0,1,2,...
                                                  at addr(h0): uram_rmwdata > thr?
                                                    YES → coinc_spiked[0]=1
                                                  at addr(h0): asb_target[~sel][?][0]
                                                              == uram_waddr[0]?
                                                    YES → coinc_detected[0]=1
                                                  phase1_coincidence[0] = 1
                                                  → latch mask, latch asb_matched_r3[0]=0x8000
                                                  → push {1, 0x0001, 0x8000} to coincFIFO

Phase 2  pointer FIFO delivers R3 at 0x8000    ptr_fifo empty (h0 doesn't fire onward yet)
         IEP latches:                          WUE pops coincFIFO entry, reads R4 + R3 ...
            asb_region3[sel][0]=0x8000           (covered in 4.7 and 4.8)
            asb_target[sel][0][0]=addr(h0)
            asb_valid[sel][0]=1

         asb_wr_count++

ET decay (unchanged from prior step, see 4.8)
```

The two pieces this page covered (*recording in T* and *detecting and pushing in T+1*) are the entire ASB role. Everything downstream of the coincFIFO push is the WUE's problem.

---

## What to take from this page

- The ASB is a per-timestep record of "which neurons did Phase 2's synapse packets target?", double-buffered so this timestep's writes don't clobber last timestep's reads.
- Phase 1 of the next timestep runs `coinc_detected & coinc_spiked` in parallel across 16 groups, all combinational, all bypassing the `exec_uram_phase1_ready` startup-delay trap.
- True coincidences push `{do_wu=1, one-hot group, R3 addr}` to the coincFIFO; the priority chain emits one entry per cycle.
- A separate mem-only path pushes `{do_wu=0, multi-hot spiked mask, R3 addr}` when axons fired and no coincidence already covered that packet.
- Both paths use `exec_hbm_region3addr` (not payload slices) and a per-packet guard (`phase2_packet_push_seen`) to avoid duplicates.

Next, [4.7](4_7_weight_update_engine) covers what happens after the WUE pops a coincFIFO entry: R4 read, R3 read, ET handshake with IEP, weight compute, reward-gated write-back.

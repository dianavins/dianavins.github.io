---
title: "4.9 Putting It Together: One R-STDP Timestep"
parent: "4 Implementing RSTDP"
nav_order: 9
---

# 4.9 Putting It Together: One R-STDP Timestep

[4.4](4_4_reward_and_address_mapping)–[4.8](4_8_eligibility_trace) each covered one piece. This page stitches them into a single concrete timestep, in execution order, on the smallest network that exercises every piece of the R-STDP machinery.

Read [Chapter 2.1](../2_The_Network_Comes_to_Life/Chapter_2_1) first if you don't already have the Phase 0 / Phase 1 / Phase 2 sequence in your head. This page assumes that scaffolding and only annotates the *new* behavior on top.

---

## The toy network

```
   a (axon)  ──── w = 1500 ────►  n (neuron)
   threshold = 1000
   ET = 0 (initial)
```

- One axon `a`, one neuron `n`. `n` is in group 0.
- One synapse `a → n`, weight 1500, encoded in R3 row `0x8000` slot 0.
- One eligibility trace cell for the synapse, at URAM full address `13'h008` (bank 0, row 4, lower half).
- `n`'s membrane potential lives at URAM bank 0, row 0, lower half (`uram_full = 13'h000`).
- `threshold = 1000`.
- `region4_offset = 23'h010000`. R4 cell at `0x18000` holds the URAM pointer `13'h008`.
- `et_increment = 100`. `et_leak_shift = 4`. ET sweep range `[2, 8]`.
- The host sets `exec_reward = 1` before the timestep starts (`CMD_SET_REWARD`).

This network is small enough to fit one full timestep into one page, but it touches every new RTL piece.

---

## Initial state (end of T−1)

Coming into timestep T:

| State | Value | Why |
|---|---|---|
| `n`'s membrane potential | 0 | Just reset from a previous spike, or never updated. |
| Synapse weight (R3 row `0x8000` slot 0) | 1500 | Programmed by host. |
| ET (URAM `13'h008`) | 0 | Initialized by host. |
| ASB buffer 0 (`asb_sel=0`) | empty | This will be written during T's Phase 2. |
| ASB buffer 1 (`asb_sel=1`) | empty | Will be read during T+1's Phase 1. Currently has nothing. |
| `exec_reward` | 1 | Set once by host before the run. |

The host's input for T is: `a` fires this timestep. So `network.step(['a'])` (or the equivalent command stream — see [4.11](4_11_host_style_tb_level2)).

---

## Timestep T — laying the groundwork

In T, *no coincidence happens yet* — ASB(prev) is empty. T's job is to build the ASB(current) entry that T+1 will detect against.

### Phase 0 — input arrival (unchanged from Chapter 2)

Host writes the `a` spike into the BRAM Future buffer. On `exec_run`, the buffer roles flip. Nothing R-STDP-specific.

### Phase 1 — pointer collection (unchanged)

External events processor reads BRAM, generates `exec_bram_spiked` mask. Internal events processor scans URAM to find spiking neurons (none yet). HBM processor reads axon pointers and neuron pointers.

The only R-STDP-relevant thing during Phase 1 of T: **the coincidence comparator runs continuously**, but `coinc_detected = 0` everywhere because ASB(prev) is empty. No coincFIFO pushes. Membrane-only pushes also don't fire because `exec_hbm_rx_phase1_done` is still 0 during Phase 1 reads.

### Phase 2 — synapse delivery (where the ASB gets written)

The synapse packet for `a → n` arrives from HBM. `exec_hbm_region3addr = 0x8000`. `exec_hbm_rdata` carries the 512-bit packet whose group-0 slot has weight=1500 and target=`13'h000` (addr of `n`).

Two things happen *simultaneously* on the cycle that `exec_hbm_rvalidready_d1` fires inside `STATE_POP_PTR_FIFO`:

**(a) ASB write** — [4.6](4_6_coincidence_detection):

```
asb_region3[0][0]    <= 0x8000
asb_target[0][0][0]  <= 13'h000   (addr of n)
asb_target[0][0][1..15] <= ...    (decoded for unused groups)
asb_valid[0][0]      <= 1
asb_wr_count         <= 1
```

`asb_sel = 0` so the write goes to buffer 0. After this cycle, `asb_prev_depth` is irrelevant (this is T, not T+1), `asb_wr_count` is 1.

**(b) Membrane accumulation** — the original Phase 2 behavior. `n`'s membrane: `0 + 1500 = 1500`. URAM word at row 0 lower-half is updated.

The membrane-only push path *could* fire here (since `exec_bram_spiked != 0`), but on the cycle the packet arrives there's no Phase 1 coincidence push to be redundant with — and a mem-only push would correctly be made. WUE then runs and reads R3 for `a → n` via the mem-only entry, but with `do_wu = 0` it doesn't write the weight. The synapse's ET is touched (the RMW handshake fires for every entry), but with `et_increment = 100` added to ET=0 → ET=100. The reward gate doesn't apply to the ET.

Wait — the mem-only RMW does touch the ET. That's by current design but is one of the things the "fix the ET rule" next-step item might revisit, because in a proper STDP-windowed implementation the ET should only update on *coincidence*, not on every Phase 2 packet.

For this walkthrough, assume the membrane-only push path is suppressed by the `coinc_r3_match` check in [4.6](4_6_coincidence_detection) since `a → n`'s R3 address would later be matched in T+1's coincidence push — but in T it isn't, because `coinc_r3_latched[]` is still zero from the reset on T's `exec_run`. So in *this* particular toy run, the mem-only push *will* fire in T. ET goes from 0 to 100 in T. Document that here so you don't trip over it in the waveform.

### ET decay sweep at the end of T

After `exec_wue_done` and `et_rmw_state == ET_RMW_IDLE`, the main FSM enters the decay states. It sweeps `[2..8]`. Our ET is at addr 8 (=`{row 4, half 0}`), so it gets visited in the last iteration.

ET = 100, `et_leak_shift = 4`:

```
c_new = 100 - (100 >>> 4)
      = 100 - 6
      = 94
```

End of T: ET at addr 8 is 94. `n`'s membrane is 1500. `asb_buffer[0]` has one valid entry: `(R3 = 0x8000, target_for_group_0 = 13'h000)`.

T+1 begins with `exec_run`.

---

## Timestep T+1 — coincidence fires

Host sends another `network.step(['a'])` (or `network.step([])`, doesn't matter — `n` will spike regardless because its membrane is already over threshold). For drama let's say `a` fires again.

### Phase 0

BRAM update, buffer swap. `asb_sel` flips from 0 to 1 (so reads in this timestep come from buffer 0, writes go to buffer 1). `asb_valid[1][*]` cleared.

### Phase 1 — coincidence detected

This is where everything new happens.

URAM scan starts at address 0. On the cycle `uram_waddr[0] = 13'h000` (i.e., `n`'s address) — which is the very first cycle of the scan — the combinational comparators light up:

```
asb_match[0][0] = asb_valid[1 (prev)][0] && asb_target[1 (prev)][0][0] == uram_waddr[0]
                = wait, asb_sel just flipped to 1, so ~asb_sel = 0
                = asb_valid[0][0] && asb_target[0][0][0] == uram_waddr[0]
                = 1 && (13'h000 == 13'h000)
                = 1

coinc_detected[0] = |asb_match[0] = 1
```

Simultaneously, the spike check:

```
uram_rmwdata_lower_0 = 1500          (n's membrane, from T's Phase 2)
threshold            = 1000
uram_waddr[0][0]     = 0             → use lower half
coinc_spiked[0]      = (1500 > 1000) = 1
```

So `phase1_coincidence[0] = coinc_spiked[0] & coinc_detected[0] = 1`. All other groups are 0.

The latch fires:

```
coinc_pending_mask  <= 16'h0001
coinc_pushing       <= 1
coinc_r3_latched[0] <= asb_matched_r3[0] = 0x8000   (from asb_region3[0][0])
coinc_r3_latched[1..15] <= 0
```

Next cycle, the priority chain pushes one entry:

```
coincfifo_wren = 1
coincfifo_din  = {1'b1, 16'h0001, 23'h008000}
                  do_wu  group   r3_addr
```

The mem-only push path is *suppressed* this time because `coinc_r3_match` is now 1 — `coinc_r3_latched[0] = 0x8000` matches `exec_hbm_region3addr = 0x8000` later when the Phase 2 packet arrives. So no duplicate.

### Phase 2 — `n`'s spike triggers downstream (or not, in this toy network)

`n` has no downstream synapses (`n` is the output neuron). So the Phase 2 pointer FIFO for group 0 is empty. No new synapse packets. ASB(current = buffer 1) stays empty for the rest of T+1.

`n`'s membrane reset to 0 after the spike (per the neuron model in [Chapter 3.7](../3_Verilog_Files_Review/internal_events_processor)). The spike itself is enqueued onto the spike output FIFO for the host to read back.

### WUE — process the coincFIFO entry

Now the interesting part. After Phase 2 ends, WUE runs.

```
Cycle  WUE TX state           WUE RX state           ET RMW state      Signals
─────  ───────────────────    ───────────────────    ──────────────    ─────────────────────────
 1     POP_COINCFIFO          (idle)                 IDLE              coincfifo_rden=1
                                                                       latch r3_addr=0x8000
                                                                       group_idx=0
                                                                       r4_addr=0x018000

 2     SEND_R4_READ           WUE_WAIT_R4            IDLE              arvalid (r4 addr)
                                                                       arlen=0

 3     (AR accepted)                                                   arready=1

 4     SEND_R3_READ                                                    arvalid (r3 addr)
                                                                       arlen=1 (2 beats)

 5     (AR accepted)                                                   arready=1
       WAIT_RX

 6                            WUE_WAIT_R4            IDLE              hbm_rvalid (R4 beat)
                              rdata[12:0]=13'h008                      
                              wue_et_uram_addr<=13'h008
                              wue_et_addr_valid pulse
                              hbm_count<=0
                                                     pending=1
                                                     (latches addr+group)

 7                            WUE_WAIT_R3_BEAT0      IDLE→READ         uram_rden=0 (main FSM
                              hbm_rvalid (R3 beat0)                    in POP_PTR_FIFO after WUE)
                              wue_r3_beat0<=...                        uram_rden_0=1 (et_rmw)
                                                                       uram_raddr_0_full=13'h008

 8                            WUE_WAIT_R3_BEAT1      WAIT              uram_rdata_0[35:0]=94
                              hbm_rvalid (R3 beat1)                    old_tmp=94
                              wue_r3_beat1<=...                        sum=94+100=194
                              wue_r3_done<=1                           et_rmw_new_value<=194

 9                            WUE_COMPUTE            WRITE             uram_wren_0=1
                              (stall: et_done=0)                       uram_wdata_0={upper, 194}
                                                                       iep_et_valid pulse
                                                                       iep_et_value=194

10                            WUE catches            DONE              wue_et_received<=194
                              iep_et_valid                             wue_et_done<=1
                              
                              (now r3_done && et_done)
                              wue_new_w = clamp(1500+194)
                                        = 1694
                              wue_r3_wb_data <= ...
                              wue_r3_wb_pending<=1  (do_wu && reward)
                              wue_compute_done<=1

11     WAIT_RX sees           WUE_DONE → RX_IDLE     IDLE              
       compute_done
       wue_wb_active<=1
       wue_wb_data, wue_wb_addr loaded
       → WRITE_HBM_ADDR

12     WRITE_HBM_ADDR                                                  awvalid (r3_addr+1)
                                                                       (slot 0 is in beat 1)

13     (AW accepted)                                                   awready=1
       → WRITE_HBM_DATA

14     WRITE_HBM_DATA                                                  wvalid (new beat data)
                                                                       wdata = beat with
                                                                              slot0.weight=1694

15     (W accepted)                                                    wready=1
       → WRITE_HBM_RESP

16     WRITE_HBM_RESP                                                  bready=1

17     (bvalid)                                                        bvalid=1
       wue_wb_active<=0
       wue_r3_wb_pending<=0
       dbg_wue_wb_valid pulse
       → POP_COINCFIFO

18     POP_COINCFIFO
       (FIFO empty)
       → IDLE
       
       exec_wue_done asserts
       (RX_STATE_WUE_DONE)
```

After WUE finishes:
- R3 row `0x8000` slot 0 weight = **1694** (was 1500, plus ET of 194).
- ET at URAM `13'h008` = **194** (was 94, plus et_increment of 100).

### ET decay sweep at the end of T+1

Same as before. Sweep `[2..8]`:

```
ET at addr 8 = 194 - (194 >>> 4) = 194 - 12 = 182
```

End of T+1: ET = 182.

---

## What just happened, in one sentence each per piece

| Piece | What it did in this timestep |
|---|---|
| Reward register | Held `exec_reward = 1` throughout. Latched the eventual wb_pending = 1. |
| R3 address tracking | Made `exec_hbm_region3addr = 0x8000` available to IEP on the cycle the synapse packet arrived. |
| ASB | In T: recorded `(R3 = 0x8000, target = 13'h000 for group 0)`. In T+1: provided that record to the comparator. |
| Coincidence comparator | At `uram_waddr[0] = 13'h000`, both `coinc_spiked[0]` and `coinc_detected[0]` were 1 → push. |
| coincFIFO push FSM | Latched the mask + R3 addr, pushed one entry `{1, 0x0001, 0x8000}`. |
| WUE | Popped entry, issued R4+R3 reads, asked IEP for ET RMW, computed new weight = 1694, wrote it to R3. |
| ET RMW FSM | Read ET=94, added 100, saturated → wrote 194 back, signaled WUE. |
| Decay sweep | Decayed ET from 194 → 182. |

---

## What would change if reward were 0

If `exec_reward = 0`:

- ASB write in T: same.
- Phase 1 coincidence detection in T+1: same. `coinc_pending_mask`, push to coincFIFO: same.
- WUE pop, R4 read, ET RMW: same. The ET still goes 94 → 194.
- WUE compute: same arithmetic. `wue_new_w_comb = 1694`.
- **WUE wb_pending decision:** `wue_do_weight_update && exec_reward = 1 && 0 = 0` → `wue_r3_wb_pending = 0`.
- No write-back. R3 weight stays at 1500.

So eligibility *accumulates without reward*, but the weight only gets updated on the cycles where reward is high. That's the R-STDP rule from [4.1](4_1_what_is_rstdp), mapped directly onto the RTL.

---

## What would change with multiple synapses

If `n` were in group 5 instead of group 0, every signal subscript shifts: `coinc_spiked[5]`, `coinc_detected[5]`, the one-hot mask is `16'h0020`, etc. Everything still parallel — the comparator runs 16 of these simultaneously.

If there were a second axon `b` also synapsing onto `n`, T's Phase 2 would write a *second* ASB entry. T+1's coincidence comparator would match both — `asb_match[0]` would have two bits set. The reverse-order loop in `asb_r3_extract` ensures the earliest-inserted entry (i.e., `a`'s synapse, written first) wins. Only one coincFIFO entry is pushed; only `a → n`'s weight is updated. `b → n`'s synapse is silently *not* updated this timestep — that's a known limitation of the current design, documented in [`RTL_fixes_explained.txt`](../../hardware_code_rstdp/RTL_fixes_explained.txt) under "secondary issue."

If there were two coincident neurons in *different* groups (say `n` in group 0 and `m` in group 5), `phase1_coincidence` would be `16'h0021`. The priority chain in [4.6](4_6_coincidence_detection) would push two coincFIFO entries on consecutive cycles. WUE processes them serially, doing two complete pop-read-compute-write cycles.

---

## Where to look on a waveform

If you're staring at a VCD and trying to confirm "did everything happen right":

| Question | Signals to plot |
|---|---|
| Did the ASB get written? | `asb_wr_count`, `asb_sel`, `exec_hbm_region3addr`, `uram_raddr_0_full_reg` |
| Did coincidence detect? | `uram_waddr[0]`, `coinc_spiked`, `coinc_detected`, `phase1_coincidence` |
| Did the coincFIFO push correctly? | `coincfifo_wren`, `coincfifo_din[39]` (do_wu), `coincfifo_din[38:23]` (mask), `coincfifo_din[22:0]` (r3_addr) |
| Did WUE service it? | `hbm_curr_state`, `hbm_rx_curr_state`, `wue_active`, `wue_r3_done`, `wue_et_done`, `wue_compute_done` |
| Did the ET RMW fire? | `et_rmw_state`, `wue_et_addr_valid`, `iep_et_valid`, `dbg_et_old_value`, `dbg_et_new_value` |
| Did the weight get written? | `hbm_awvalid` + `hbm_awaddr`, `hbm_wvalid` + `hbm_wdata`, `dbg_wue_wb_valid` |
| Did decay run? | `curr_state` (3'd12 / 3'd13 / 3'd14), `et_raddr` |

Probe depth=1 in xsim is enough to surface all of these — they're all on top-level wires or on registers in the two modules. `iep_curr_state` and `hbm_rx_curr_state` are exposed as output ports so you don't need hierarchical reads.

---

## What to take from this page

- A coincidence requires *two* timesteps: one to log the synapse arrival in the ASB, one to detect post-synaptic spike at the matching address.
- Everything in T+1 from "Phase 1 detects coincidence" through "WUE writes the new weight" happens within one `exec_run` window, in dependency order, with the ET RMW running in parallel with the R3 read.
- The reward signal influences exactly one bit (`wue_r3_wb_pending`). ET still updates without reward. ASB and coincFIFO are reward-agnostic.
- The decay sweep is the last thing each timestep, after WUE drains.

Next: [4.10](4_10_integration_tb_level1) — how `step8_learning_integration_tb.sv` exercises everything you just read, and the conventions you need to follow to add a new test.

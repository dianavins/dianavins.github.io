---
title: "4.7 The Weight Update Engine"
parent: "4 Implementing RSTDP"
nav_order: 7
---

# 4.7 The Weight Update Engine

The Weight Update Engine (WUE) consumes the coincFIFO entries that [4.6](4_6_coincidence_detection) produced. For each entry it:

1. Reads R4 → gets the URAM pointer for that synapse's eligibility trace.
2. Asks IEP to read-modify-write the ET; in parallel reads R3 → gets the synapse's old weight.
3. Waits for both R3 and ET to come back.
4. Computes `new_w = clamp(old_w + et)`.
5. If `do_wu` and `exec_reward` are both set, writes the new weight back to R3.
6. Loops to the next coincFIFO entry.

WUE lives entirely inside [`hbm_processor.v`](../../hardware_code_rstdp/src/hbm_processor.v). It adds four TX states, five RX states, ~30 registers, and a handshake with IEP. This page covers everything except the IEP-side ET RMW — that's [4.8](4_8_eligibility_trace).

---

## Where WUE sits in the timestep

WUE runs *after* Phase 2 finishes, inside the same `exec_run` window. The transition lives at [`hbm_processor.v:693`](../../hardware_code_rstdp/src/hbm_processor.v#L693):

```verilog
TX_STATE_PHASE2_DONE: begin
    tx_phase_inc  = 1'b1;
    tx_next_state = TX_STATE_WUE_POP_COINCFIFO;
end
```

The RX side mirrors it at [line 846](../../hardware_code_rstdp/src/hbm_processor.v#L846):

```verilog
RX_STATE_PHASE2_DONE: begin
    rx_next_state = RX_STATE_WUE_WAIT_R4;
end
```

So the moment Phase 2's last synapse beat is processed, WUE starts. WUE runs as many iterations as there are coincFIFO entries, then returns to IDLE so the ET decay sweep (in IEP) can run.

```
exec_run
  │
  ▼
Phase 1 ──► Phase 2 ──► WUE ──► (ET decay sweep, IEP-side) ──► IDLE
                          ▲    ▲
                          │    │
                       coincFIFO drained
```

---

## The four WUE TX states

Defined at [`hbm_processor.v:358`](../../hardware_code_rstdp/src/hbm_processor.v#L358):

```verilog
localparam [3:0] TX_STATE_WUE_POP_COINCFIFO = 4'd12;
localparam [3:0] TX_STATE_WUE_SEND_R4_READ  = 4'd13;
localparam [3:0] TX_STATE_WUE_SEND_R3_READ  = 4'd14;
localparam [3:0] TX_STATE_WUE_WAIT_RX       = 4'd15;
```

Transitions, simplified:

```
                                              coincFIFO empty
                                              and no wb pending
                                                      │
                                                      ▼
   ┌─────────────────────────┐                ┌──────────────┐
   │ TX_STATE_WUE_POP_       │ ── pop entry ──┤ (eventually  │
   │ COINCFIFO               │   ────────►    │  fall back   │
   └────────────┬────────────┘                │  to IDLE)    │
                │                              └──────────────┘
                │ coincfifo_rden=1
                ▼
   ┌─────────────────────────┐
   │ TX_STATE_WUE_SEND_R4_   │ ── arvalid+arready ──┐
   │ READ  (arlen=0)         │                       │
   └────────────┬────────────┘                       │
                │                                    │
                ▼                                    │
   ┌─────────────────────────┐                       │
   │ TX_STATE_WUE_SEND_R3_   │ ── arvalid+arready ──┤
   │ READ  (arlen=1, 2 beats)│                       │
   └────────────┬────────────┘                       │
                │                                    │
                ▼                                    │
   ┌─────────────────────────┐                       │
   │ TX_STATE_WUE_WAIT_RX    │ ◄─── RX runs in       │
   │                         │       parallel        │
   │  if wb pending → write  │                       │
   │  else loop back to POP  │                       │
   └────────────┬────────────┘                       │
                │                                    │
                └────────────► loop to POP ──────────┘
```

The RX side is the more interesting half — it's where the data actually arrives and where the ET handshake completes. We'll get to that.

### TX_STATE_WUE_POP_COINCFIFO

[Line 701](../../hardware_code_rstdp/src/hbm_processor.v#L701):

```verilog
TX_STATE_WUE_POP_COINCFIFO: begin
    if (wue_wb_active) begin
        // Pending write (R3 after R4) — dispatch it before checking FIFO
        tx_next_state = TX_STATE_WRITE_HBM_ADDR;
    end else if (coincfifo_empty) begin
        tx_next_state = TX_STATE_IDLE;
    end else begin
        // Always pop and process: membrane update regardless of reward/coincidence
        coincfifo_rden = 1'b1;
        tx_next_state  = TX_STATE_WUE_SEND_R4_READ;
    end
end
```

Three branches:

- A write-back is queued from a prior compute → service it first.
- The FIFO is empty and nothing pending → done; return to IDLE.
- Otherwise → pop and start the R4 read.

When a new entry is popped, the WUE sequential block at [line 1063](../../hardware_code_rstdp/src/hbm_processor.v#L1063) latches the per-entry state and computes `r4_addr`:

```verilog
end else if (!coincfifo_empty) begin
    wue_tx_done          <= 1'b0;
    wue_do_weight_update <= coincfifo_dout[39];
    wue_group_mask       <= coincfifo_dout[38:23];
    wue_r3_addr          <= coincfifo_dout[22:0];
    wue_group_idx        <= onehot_to_idx(coincfifo_dout[38:23]);
    wue_hbm_addr         <= coincfifo_dout[22:0] + region4_offset;
    wue_active           <= 1'b1;
    wue_r4_reading       <= 1'b1;
    // Clear per-entry convergence flags for the new entry
    wue_r3_done          <= 1'b0;
    wue_et_done          <= 1'b0;
    ...
end
```

A few things worth noting:

- **`onehot_to_idx`** is a 16-way `casez` at [line 263](../../hardware_code_rstdp/src/hbm_processor.v#L263) that converts a one-hot mask to its 4-bit index. For mem-only pushes (`coincfifo_dout[38:23]` is multi-hot), this picks the lowest set bit. WUE will only do R3 write-back for that single bit; membrane updates for the other set bits happen via the same R3 read that comes back to IEP through `exec_wue_r3_valid`.
- **`wue_r3_done` and `wue_et_done`** are per-entry convergence flags. WUE waits for both before computing. Resetting them here makes the per-entry loop self-contained.
- **`wue_hbm_addr`** is the AR address mux source for the *next* read. Initial value is `r3_addr + region4_offset` (R4 address). After R4's AR is accepted it gets switched to `r3_addr` for the R3 read.

### TX_STATE_WUE_SEND_R4_READ and TX_STATE_WUE_SEND_R3_READ

Both send one AR transaction. They differ only in `hbm_arlen`:

```verilog
always @(*) begin
    if (wue_active) begin
        hbm_arlen = wue_r4_reading ? 4'd0 : 4'd1;   // R4: 1 beat; R3: 2 beats
    end
    ...
end
```

`wue_r4_reading` flips on the cycle R4's AR is accepted, at [line 1080](../../hardware_code_rstdp/src/hbm_processor.v#L1080):

```verilog
TX_STATE_WUE_SEND_R4_READ: begin
    if (hbm_arready) begin
        // R4 read accepted; switch to R3 address for next read
        wue_hbm_addr   <= wue_r3_addr;
        wue_r4_reading <= 1'b0;
    end
end
```

So the TX side issues both reads in immediate succession — no waiting. The RX side will receive R4 first, then R3.

### TX_STATE_WUE_WAIT_RX

[Line 725](../../hardware_code_rstdp/src/hbm_processor.v#L725):

```verilog
TX_STATE_WUE_WAIT_RX: begin
    if (wue_wb_active) begin
        tx_next_state = TX_STATE_WRITE_HBM_ADDR;
    end else if (wue_compute_done && !wue_r3_wb_pending) begin
        tx_next_state = TX_STATE_WUE_POP_COINCFIFO;
    end
end
```

Three things can happen:

- Write-back already active → dispatch immediately.
- Compute finished, no R3 write-back queued (e.g., `do_wu=0` or `exec_reward=0`) → loop back for next entry.
- Otherwise stay here until one of the above is true.

The compute itself, including the wb_pending decision, happens on the RX side.

---

## The five WUE RX states

Defined at [`hbm_processor.v:383`](../../hardware_code_rstdp/src/hbm_processor.v#L383):

```verilog
localparam [3:0] RX_STATE_WUE_WAIT_R4         = 4'd10;
localparam [3:0] RX_STATE_WUE_WAIT_R3_BEAT0   = 4'd11;
localparam [3:0] RX_STATE_WUE_WAIT_R3_BEAT1   = 4'd12;
localparam [3:0] RX_STATE_WUE_COMPUTE         = 4'd13;
localparam [3:0] RX_STATE_WUE_DONE            = 4'd14;
```

### WAIT_R4 — receive R4 beat, kick the ET RMW

[Line 1122](../../hardware_code_rstdp/src/hbm_processor.v#L1122):

```verilog
RX_STATE_WUE_WAIT_R4: begin
    if (hbm_rvalid && hbm_rready) begin
        // R4 beat now carries a URAM pointer, not an ET value
        wue_r4_ptr        <= hbm_rdata[12:0];
        wue_et_uram_addr  <= hbm_rdata[12:0];
        wue_et_group_idx  <= wue_group_idx;
        wue_et_addr_valid <= 1'b1;
        dbg_wue_et_addr_valid <= 1'b1;
    end
end
```

The R4 beat's low 13 bits *are* the URAM full address of this synapse's ET cell (see [4.4](4_4_reward_and_address_mapping)). WUE:

- Latches the pointer for its own debug.
- Drives `wue_et_uram_addr`, `wue_et_group_idx`, and a one-cycle pulse on `wue_et_addr_valid` over to IEP. This is what kicks the IEP ET RMW FSM ([4.8](4_8_eligibility_trace)).

`hbm_count` also resets here — see [4.5](4_5_r3_address_tracking) for why.

### WAIT_R3_BEAT0 and WAIT_R3_BEAT1 — receive the 512-bit synapse packet

R3 reads are 2-beat bursts. Two RX states, one per beat, at [line 1132](../../hardware_code_rstdp/src/hbm_processor.v#L1132):

```verilog
RX_STATE_WUE_WAIT_R3_BEAT0: begin
    if (hbm_rvalid && hbm_rready) begin
        wue_r3_beat0 <= hbm_rdata;
    end
end
RX_STATE_WUE_WAIT_R3_BEAT1: begin
    if (hbm_rvalid && hbm_rready) begin
        wue_r3_beat1 <= hbm_rdata;
        wue_r3_done  <= 1'b1;
        if (!wue_et_done)
            dbg_r3_arrived_first <= 1'b1;
    end
end
```

After the second beat, `wue_r3_done` is set. The two 256-bit beats together form a 512-bit packet:

```
wue_synapse [511:0] = {wue_r3_beat1, wue_r3_beat0}
```

There are 16 × 32-bit synapse slots in that 512 bits, one per group. The 16-bit weight for our group is at:

```verilog
wue_old_w_comb = wue_synapse[(15 - wue_group_idx)*32 +: 16];
```

The `(15 - wue_group_idx)*32 +: 16` slice picks the weight bits of slot G. The other 16 bits of the slot are the OpCode + target address (see [Chapter 1.1](../1_Initializing_the_Network/Chapter_1_1)) — irrelevant for the weight update; we'll preserve them when writing back.

There's also a parallel path that latches `iep_et_value` whenever IEP pulses `iep_et_valid`. This is fired *outside* the RX `case` statement so it works regardless of which RX state is active when the ET comes back:

```verilog
// Latch iep_et_value whenever iep_et_valid is asserted — fires in any RX state
if (iep_et_valid) begin
    wue_et_received      <= iep_et_value;
    wue_et_done          <= 1'b1;
    dbg_iep_et_valid     <= 1'b1;
    dbg_wue_et_received  <= iep_et_value;
    if (!wue_r3_done)
        dbg_et_arrived_first <= 1'b1;
end
```

The `dbg_r3_arrived_first` / `dbg_et_arrived_first` flags help debug timing races — usually R3 arrives first (HBM latency > URAM read latency), but neither is guaranteed.

### WUE_COMPUTE — stall until both R3 and ET are done

[Line 1145](../../hardware_code_rstdp/src/hbm_processor.v#L1145):

```verilog
RX_STATE_WUE_COMPUTE: begin
    if (wue_r3_done && wue_et_done && !wue_compute_done) begin
        dbg_wue_valid      <= wue_do_weight_update && exec_reward;
        dbg_wue_r3_addr    <= wue_r3_addr;
        dbg_wue_group_idx  <= wue_group_idx;
        dbg_wue_old_weight <= wue_old_w_comb;
        dbg_wue_new_weight <= wue_new_w_comb;

        // Prepare R3 write-back
        wue_r3_wb_data    <= wue_r3_wb_beat_comb;
        wue_r3_wb_addr    <= wue_r3_wb_addr_comb;

        // Weight updated only when reward is active
        wue_r3_wb_pending <= (wue_do_weight_update && exec_reward) ? 1'b1 : 1'b0;

        wue_compute_done  <= 1'b1;
    end
end
```

The `!wue_compute_done` guard means this fires exactly once per entry. The compute itself is combinational (see next section); this block just latches the result and decides whether to queue the write-back.

### WUE_DONE

The FSM exit point. Returns to `RX_STATE_IDLE`.

---

## The compute: sum, clamp, and beat reconstruction

All combinational, at [`hbm_processor.v:288`](../../hardware_code_rstdp/src/hbm_processor.v#L288).

### Sum + clamp

```verilog
wire signed [15:0] wue_old_w_comb = wue_synapse_comb[(15-wue_group_idx)*32 +: 16];
wire signed [36:0] wue_sum_comb = {wue_et_received[35], wue_et_received}
                                + {{21{wue_old_w_comb[15]}}, wue_old_w_comb};
wire signed [15:0] wue_new_w_comb =
    (wue_sum_comb > 37'sd32767)  ? 16'sd32767 :
    (wue_sum_comb < -37'sd32768) ? -16'sd32768 :
    wue_sum_comb[15:0];
```

Three steps:

1. **Sign-extend** old weight from 16 to 37 bits, ET from 36 to 37 bits.
2. **Add** in 37-bit signed.
3. **Clamp** to int16 range and take the low 16 bits.

37 bits is `max(16, 36) + 1`, enough to hold any signed sum without overflow before the clamp.

The clamp matters because R3 stores weights as 16-bit signed. If the eligibility trace is large (e.g., many recent coincidences with `et_increment = 500`) and the host's reward steps are sparse, individual updates can easily overflow without a clamp.

### Building the write-back beat

R3 stores 16 synapse slots per 512-bit packet (2 × 256-bit beats). Group 0 occupies the top of the upper beat; group 15 occupies the bottom of the lower beat. The write-back has to:

1. Pick *which* 256-bit beat to write (beat 0 for groups 0–7, beat 1 for groups 8–15 — wait, it's the reverse, see below).
2. Splice the new 16-bit weight into the right slot of that beat.
3. Leave every other bit untouched (other groups' weights, the OpCode and target address fields).

The code at [line 304](../../hardware_code_rstdp/src/hbm_processor.v#L304):

```verilog
always @(*) begin
    if (wue_group_idx <= 4'd7) begin
        wue_r3_wb_beat_comb = wue_r3_beat1;          // groups 0-7 in beat 1
        wue_r3_wb_addr_comb = wue_r3_addr + 23'd1;   // beat 1 at R3+1
        case (wue_group_idx)
            4'd0: wue_r3_wb_beat_comb[239:224] = wue_new_w_comb;
            4'd1: wue_r3_wb_beat_comb[207:192] = wue_new_w_comb;
            4'd2: wue_r3_wb_beat_comb[175:160] = wue_new_w_comb;
            4'd3: wue_r3_wb_beat_comb[143:128] = wue_new_w_comb;
            4'd4: wue_r3_wb_beat_comb[111:96]  = wue_new_w_comb;
            4'd5: wue_r3_wb_beat_comb[79:64]   = wue_new_w_comb;
            4'd6: wue_r3_wb_beat_comb[47:32]   = wue_new_w_comb;
            4'd7: wue_r3_wb_beat_comb[15:0]    = wue_new_w_comb;
            default: ;
        endcase
    end else begin
        wue_r3_wb_beat_comb = wue_r3_beat0;           // groups 8-15 in beat 0
        wue_r3_wb_addr_comb = wue_r3_addr;            // beat 0 at R3+0
        case (wue_group_idx)
            // ... mirror for groups 8-15 ...
        endcase
    end
end
```

Note the "groups 0–7 are in beat 1, groups 8–15 are in beat 0" — that's because the 512-bit packet is assembled with beat 1 on top:

```
wue_synapse [511:0] = {wue_r3_beat1, wue_r3_beat0}
                       └─── 256 ───┘└─── 256 ───┘
                       groups 0-7   groups 8-15
                       (in top half)(in bottom half)
```

so the high-numbered slots (which the convention puts at the *high* bit positions per the existing R3 packing) end up in beat 1. The exact bit slices come straight from the existing R3 packing convention covered in [Chapter 1.1](../1_Initializing_the_Network/Chapter_1_1).

Only one beat is written per entry. The other beat is left untouched in HBM. That halves the bandwidth cost of write-back vs. writing both beats.

---

## The reward gate

`wue_r3_wb_pending` is the queue bit that tells the TX side "go do a write-back." Its value is decided exactly once per entry, in WUE_COMPUTE:

```verilog
wue_r3_wb_pending <= (wue_do_weight_update && exec_reward) ? 1'b1 : 1'b0;
```

Three cases:

| `do_wu` | `exec_reward` | Result |
|---|---|---|
| 0 | × | No write. Mem-only pushes never write weights, period. |
| 1 | 0 | No write. Coincidence happened, but no reward → eligibility trace was updated (in IEP), weight stays put. |
| 1 | 1 | Write. R-STDP active update. |

This is the *only* place in the RTL where `exec_reward` is consumed. Everything else — coincidence detection, ET RMW — runs independently of reward.

---

## The write-back path

The actual AXI write reuses the same machinery the original processor uses for `CMD_HBM_RW` writes. Just two muxes at [line 422](../../hardware_code_rstdp/src/hbm_processor.v#L422):

```verilog
assign hbm_awaddr = wue_wb_active ? {5'd0, wue_wb_addr, 5'd0}
                                  : {5'd0, ci2hbm_dout[278:256], 5'd0};
assign hbm_wdata  = wue_wb_active ? wue_wb_data : ci2hbm_dout[255:0];
```

When the TX FSM enters `TX_STATE_WRITE_HBM_ADDR` from the WUE path, it sets `wue_wb_active=1` (loaded from `wue_r3_wb_pending` in WAIT_RX, see [line 1090](../../hardware_code_rstdp/src/hbm_processor.v#L1090)):

```verilog
TX_STATE_WUE_WAIT_RX: begin
    if (wue_compute_done) begin
        wue_compute_done <= 1'b0;
        if (wue_r3_wb_pending) begin
            wue_wb_data   <= wue_r3_wb_data;
            wue_wb_addr   <= wue_r3_wb_addr;
            wue_wb_active <= 1'b1;
        end
    end
end
```

The same WRITE_HBM_ADDR → WRITE_HBM_DATA → WRITE_HBM_RESP sequence used by the CI write path runs. Single beat (`awlen=0`). On `bvalid`, the TX FSM checks whether the write was for WUE and loops accordingly:

```verilog
TX_STATE_WRITE_HBM_RESP: begin
    hbm_bready = 1'b1;
    if (hbm_bvalid) begin
        if (wue_wb_active) begin
            // WUE write complete — loop back to pop next entry
            tx_next_state = TX_STATE_WUE_POP_COINCFIFO;
        end else begin
            ci2hbm_rden = 1'b1;
            tx_next_state = TX_STATE_IDLE;
        end
    end
end
```

WUE's write-back completion clears `wue_wb_active` and `wue_r3_wb_pending` in the sequential block at [line 1102](../../hardware_code_rstdp/src/hbm_processor.v#L1102), and pulses `dbg_wue_wb_valid` for the testbench to count.

### Why arbitration is "WUE waits for CI"

CI writes go through the same path. If a `CMD_HBM_RW` write arrives while WUE is mid-coincFIFO, who wins? Inspect the IDLE state at [line 598](../../hardware_code_rstdp/src/hbm_processor.v#L598):

```verilog
TX_STATE_IDLE: begin
    tx_addr_rst = 1'b1;
    if (exec_run) begin
        ...
    end else if (~ci2hbm_empty) begin
        // CI write or read
        ...
    end
end
```

Once an `exec_run` window starts, WUE has exclusive use of the HBM bus until coincFIFO drains. CI writes wait. That's enforced by the FSM structure: `TX_STATE_WUE_POP_COINCFIFO` only returns to IDLE when the FIFO is empty. The host driver code in `hs_bridge` already serializes CI commands behind `exec_step`, so this matches the host expectation. The compile-time decision is documented in [`plan.txt`](../../hardware_code_rstdp/plan.txt) step 6.

---

## Debug outputs you'll see in the TB

WUE exposes several debug registers for the integration TB. All listed on the module port list at [line 156](../../hardware_code_rstdp/src/hbm_processor.v#L156):

| Signal | Pulse? | Use |
|---|---|---|
| `dbg_wue_valid` | one cycle | At compute time. High iff `do_wu && exec_reward`. |
| `dbg_wue_r3_addr` | reg | R3 address of the entry currently being computed. |
| `dbg_wue_group_idx` | reg | Group index decoded from group_mask. |
| `dbg_wue_old_weight` | reg | Old weight extracted from R3 packet. |
| `dbg_wue_new_weight` | reg | Computed new weight after clamp. |
| `dbg_wue_wb_valid` | one cycle | On `hbm_bvalid` after WUE write — "the write committed." |
| `dbg_wue_et_received` | reg | The `iep_et_value` we used in the sum. |
| `dbg_wue_et_addr_valid` | one cycle | When WUE sent the URAM addr to IEP. |
| `dbg_iep_et_valid` | one cycle | When IEP returned the new ET. |
| `dbg_r3_arrived_first` | reg | 1 if R3 done before ET done. |
| `dbg_et_arrived_first` | reg | 1 if ET done before R3 done. |

The TB conventions for using these are in [4.10](4_10_integration_tb_level1).

---

## End-to-end through one coincFIFO entry

A coincFIFO entry from [4.6](4_6_coincidence_detection): `{do_wu=1, group_mask=0x0001, r3_addr=0x8000}`. Reward is high. Old weight is 1000. ET after IEP RMW returns 250.

```
TX FSM                              RX FSM                          Boundary signals
─────────────────────────────────────────────────────────────────────────────────────
POP_COINCFIFO                       WUE_WAIT_R4                     coincfifo_rden=1
  latch r3_addr=0x8000                                              ← 40-bit entry
  group_idx=0                                                       latched
  r4_addr = 0x8000 + REGION4_OFFSET

SEND_R4_READ                                                        hbm_arvalid=1
  arlen=0                                                           arlen=0

(AR accepted)                                                       hbm_arready=1
  wue_r4_reading <= 0                                               
  wue_hbm_addr   <= 0x8000

SEND_R3_READ                                                        hbm_arvalid=1
  arlen=1                                                           arlen=1

(AR accepted)                       
  ──► TX done, WAIT_RX               

WAIT_RX                             WUE_WAIT_R4
                                       (rvalid)
                                       hbm_rdata[12:0] = 13'h008
                                       wue_et_uram_addr <= 13'h008
                                       wue_et_addr_valid pulse       → IEP ET RMW fires
                                       hbm_count <= 0

                                    WUE_WAIT_R3_BEAT0
                                       (rvalid)
                                       wue_r3_beat0 <= hbm_rdata

                                    WUE_WAIT_R3_BEAT1
                                       (rvalid)
                                       wue_r3_beat1 <= hbm_rdata
                                       wue_r3_done  <= 1

                                    WUE_COMPUTE (stalled)
                                       wue_et_done still 0
                                       ...                          ← iep_et_valid pulse
                                       wue_et_received <= 250
                                       wue_et_done <= 1
                                       
                                       (now both done)
                                       wue_new_w = clamp(1000+250) = 1250
                                       wue_r3_wb_data = beat with
                                          slot[0].weight = 1250
                                       wue_r3_wb_pending <= 1
                                       (do_wu=1 && exec_reward=1)
                                       wue_compute_done <= 1

                                    WUE_DONE → RX_IDLE

WAIT_RX sees wue_compute_done
  load wue_wb_data/addr, set
  wue_wb_active <= 1
  → WRITE_HBM_ADDR

WRITE_HBM_ADDR                                                      hbm_awvalid=1
                                                                    hbm_awaddr = 0x8001<<5
                                                                    (beat 1 at R3+1)

(AW accepted) → WRITE_HBM_DATA                                      hbm_awready=1
                                                                    hbm_wvalid=1
                                                                    hbm_wdata = updated beat

(W accepted) → WRITE_HBM_RESP                                       hbm_wready=1

WRITE_HBM_RESP                                                      
(bvalid)                                                            hbm_bvalid=1
  wue_wb_active <= 0
  wue_r3_wb_pending <= 0
  dbg_wue_wb_valid pulse
  → POP_COINCFIFO

POP_COINCFIFO
  (FIFO empty)
  → IDLE → triggers IEP ET decay sweep
```

For an entry with `do_wu=1, exec_reward=0`, the same flow runs through compute but `wue_r3_wb_pending` stays 0; TX skips straight from WAIT_RX back to POP. For `do_wu=0`, same — and the membrane update path in IEP still picks up the synapse via `exec_wue_r3_valid` regardless.

---

## What to take from this page

- WUE runs after Phase 2 ends, processes coincFIFO entries one at a time, returns to IDLE when the FIFO is empty.
- Per entry: R4 read (1 beat → URAM ET pointer + kick IEP) → R3 read (2 beats → 512-bit synapse packet) → wait for both + ET RMW → compute → maybe write back.
- The compute is a 16-bit signed clamp of `old_w + et_received`, spliced into the correct slot of the correct R3 beat.
- The write-back happens only if `do_wu && exec_reward`. Mem-only entries (`do_wu=0`) never write.
- Write-back uses the same AXI machinery as CI writes; one beat per entry; CI writes wait until WUE drains.
- The `hbm_count` reset on `RX_STATE_WUE_WAIT_R4` is what keeps Phase 2 packet assembly correct in the *next* timestep — don't remove it.

Next: [4.8](4_8_eligibility_trace) — what happens inside IEP when `wue_et_addr_valid` pulses: the ET RMW FSM, the saturating add, the decay sweep, and the open caveat about the rule itself.

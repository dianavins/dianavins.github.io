---
title: "4.8 Eligibility Trace Read-Modify-Write and Decay"
parent: "4 Implementing RSTDP"
nav_order: 8
---

# 4.8 Eligibility Trace Read-Modify-Write and Decay

The eligibility trace `c(t)` is the bridge between coincidence and reward. It accumulates on each coincidence and decays slowly between events, so that when reward eventually arrives (potentially many timesteps later), the synapses that *recently* had coincident pre/post activity still have a nonzero `c` and get their weights updated.

In the FPGA, `c(t)` is a 36-bit signed integer stored in URAM, one per learning synapse. The host compiler assigns each synapse a URAM cell ([4.4](4_4_reward_and_address_mapping)), and `internal_events_processor.v` runs two pieces of machinery on those cells:

1. The **ET RMW FSM** — services WUE's "increment this ET" requests asynchronously, one at a time.
2. The **ET decay sweep** — runs once per timestep after WUE finishes; applies a shift-decay across the configured URAM range.

This page covers both, the CI commands that configure them, and an honest caveat about the rule itself.

---

## Caveat: the rule is a placeholder

`plan.txt`'s "next steps" list begins with **"fix the eligibility trace update rule."** The current implementation does:

```
c_new = saturate(c_old + et_increment)
```

That's an unsigned-style integer increment plus a saturating clamp. It is *not* a fully correct STDP-windowed eligibility trace. A correct implementation would gate the increment on the sign of `Δt = t_post − t_pre`, scale it by a kernel, or replace the constant `et_increment` with the output of a per-synapse STDP function ([4.1](4_1_what_is_rstdp) goes through the math).

The current RTL is enough to make the *plumbing* — handshake, RMW timing, decay sweep, saturation — work end-to-end and pass the integration TB. When you replace the rule, the only block you'll need to touch is the arithmetic in `ET_RMW_WAIT` (covered below). Everything around it (the FSM, URAM steering, decay sweep) is rule-independent and should stay.

If you're tasked with fixing the rule, start here, then move on to [4.12](4_12_next_steps).

---

## Where ETs live: a refresher

From [4.4](4_4_reward_and_address_mapping):

- 16 URAM banks, indexed by `group_idx [3:0]`.
- Each bank has 4096 × 72-bit words.
- Each 72-bit word holds **two** 36-bit signed values (upper + lower half).
- The 13-bit URAM full address splits as `{row[12:1], half_select[0]}`.

ETs share URAM with neuron membrane potentials. The host compiler is responsible for assigning ET addresses outside the address range used by membrane potentials. The configured ET range is `[et_uram_start_raddr .. et_uram_end_raddr]` — exposed to IEP as 13-bit ports, set by `CMD_SET_ET_RANGE`. The decay sweep uses this range; the RMW FSM has no opinion on which addresses are "valid ET addresses" — it accesses whichever URAM cell WUE points it at.

A typical sim setup uses `et_uram_start_raddr = 13'd2` (row 1, lower half), keeping ETs above neuron row 0 (see MEMORY.md).

---

## The ET RMW FSM

Lives entirely inside `internal_events_processor.v`. Five states declared at [line 426](../../hardware_code_rstdp/src/internal_events_processor.v#L426):

```verilog
reg  [2:0] et_rmw_state;
localparam [2:0] ET_RMW_IDLE  = 3'd0;
localparam [2:0] ET_RMW_READ  = 3'd1;
localparam [2:0] ET_RMW_WAIT  = 3'd2;
localparam [2:0] ET_RMW_WRITE = 3'd3;
localparam [2:0] ET_RMW_DONE  = 3'd4;
```

### Inputs and outputs

The boundary with WUE is four signals:

| Direction | Signal | Use |
|---|---|---|
| WUE → IEP | `wue_et_uram_addr [12:0]` | URAM full address of the ET cell to RMW |
| WUE → IEP | `wue_et_addr_valid` | One-cycle pulse: "go." Latched on any cycle, in any IEP state. |
| WUE → IEP | `wue_et_group_idx [3:0]` | Which of the 16 URAM banks |
| IEP → WUE | `iep_et_value [35:0]` | The post-RMW ET value |
| IEP → WUE | `iep_et_valid` | One-cycle pulse: "result on `iep_et_value` this cycle." |

Plus a CI-programmable parameter:

| Signal | Width | Set by |
|---|---|---|
| `et_increment` | 36 signed | `CMD_SET_ET_PARAMS` (covered below) |

### Latching the request

At any time, on any cycle, IEP can be told to do an ET RMW. The latch fires unconditionally on `wue_et_addr_valid`:

```verilog
// Latch incoming WUE request whenever wue_et_addr_valid pulses (any state)
if (wue_et_addr_valid) begin
    et_rmw_addr_latched      <= wue_et_uram_addr;
    et_rmw_group_idx_latched <= wue_et_group_idx;
    et_rmw_pending           <= 1'b1;
    dbg_et_uram_addr         <= wue_et_uram_addr;
end
```

`et_rmw_pending` is the "I have a request waiting" flag. The FSM picks it up in IDLE.

### The five states

#### ET_RMW_IDLE — wait, then start when URAM is free

```verilog
ET_RMW_IDLE: begin
    if (et_rmw_pending && !uram_rden) begin
        et_rmw_pending <= 1'b0;
        et_rmw_state   <= ET_RMW_READ;
    end
end
```

The `!uram_rden` guard is critical: the main FSM (Phase 1 scan, Phase 2 accumulation, ET decay) drives the *scalar* `uram_rden` signal to all 16 banks in lockstep. If the ET RMW FSM tries to assert a different per-group `uram_rden_G` at the same time, both fire and you get a collision. By waiting for `uram_rden == 0`, the ET RMW FSM ensures it has exclusive access to the URAM read bus.

This wait is normally short. WUE's `wue_et_addr_valid` fires in `RX_STATE_WUE_WAIT_R4` — after Phase 2 has ended and before the ET decay sweep starts — so the main FSM is in `STATE_POP_PTR_FIFO` with `uram_rden=0` most of the time.

#### ET_RMW_READ — steer the read

When this state is active, two combinational paths route the read to the target group only:

`uram_rden_G` per group ([line 992](../../hardware_code_rstdp/src/internal_events_processor.v#L992)):

```verilog
uram_rden_0 = uram_rden | (et_rmw_state == ET_RMW_READ && et_rmw_group_idx_latched == 4'd0);
// ... groups 1-15 ...
```

`uram_raddr_G_full` per group ([line 854](../../hardware_code_rstdp/src/internal_events_processor.v#L854)):

```verilog
if (et_rmw_state == ET_RMW_READ) begin
    // ET RMW: route the target ET address to all groups (only target group's rden fires)
    uram_raddr_0_full  = et_rmw_addr_latched;
    uram_raddr_1_full  = et_rmw_addr_latched;
    // ... all 16 groups get the same address ...
end
```

All 16 raddrs are set to the latched address, but only the target group's `rden` fires. That's the cheapest way to keep the URAM word read pipeline clean for the other groups without expanding the existing scalar-rden architecture.

The state transitions immediately:

```verilog
ET_RMW_READ: begin
    et_rmw_state <= ET_RMW_WAIT;
end
```

#### ET_RMW_WAIT — capture the old value, compute the new

URAM has 1-cycle read latency, so the data arrives one cycle after `rden`. WAIT handles that:

```verilog
ET_RMW_WAIT: begin
    begin : et_rmw_compute
        reg signed [36:0] sum_tmp;
        reg signed [35:0] old_tmp;
        old_tmp = et_rmw_addr_latched[0] ?
                  et_rmw_rdata_mux[71:36] : et_rmw_rdata_mux[35:0];
        sum_tmp = ($signed({et_rmw_addr_latched[0] ? et_rmw_rdata_mux[71]
                                                   : et_rmw_rdata_mux[35],
                           old_tmp})
                 + $signed({et_increment[35], et_increment}));

        et_rmw_old_value <= old_tmp;
        et_rmw_sum       <= sum_tmp;
        dbg_et_old_value <= old_tmp;

        // Saturate and register new value for use in WRITE cycle
        if (sum_tmp[36] != sum_tmp[35])
            et_rmw_new_value <= sum_tmp[36] ? 36'sh800000000 : 36'sh7FFFFFFFF;
        else
            et_rmw_new_value <= sum_tmp[35:0];
    end

    et_rmw_state <= ET_RMW_WRITE;
end
```

Step by step:

1. **Select half.** Bit `[0]` of the latched address picks upper (`[71:36]`) or lower (`[35:0]`) half of the 72-bit URAM word. `et_rmw_rdata_mux` is a 16-way mux that picks the target group's URAM data, driven by another combinational block at [line 442](../../hardware_code_rstdp/src/internal_events_processor.v#L442).
2. **Sign-extend** old value (36 → 37 bits) and `et_increment` (36 → 37 bits).
3. **Add** in 37-bit signed.
4. **Saturate.** If sign-extension bits disagree (overflow happened), clamp to ±2³⁵. Otherwise take the low 36 bits.

`et_rmw_new_value` is now a registered, saturated, signed 36-bit ready for the write cycle.

**This is the line you change when you implement a proper STDP rule.** Replace `et_increment` with whatever function of `Δt`, `et_old`, and any other signals you want. The 37-bit math and saturation stay.

#### ET_RMW_WRITE — write back, signal completion

```verilog
ET_RMW_WRITE: begin
    iep_et_value     <= et_rmw_new_value;
    iep_et_valid     <= 1'b1;
    dbg_et_new_value <= et_rmw_new_value;
    dbg_et_rmw_valid <= 1'b1;

    et_rmw_state <= ET_RMW_DONE;
end
```

The actual URAM write is driven by combinational logic outside the FSM — per-group `uram_wren_G` and `uram_wdata_G`, both of which have an `et_rmw_state == ET_RMW_WRITE` case at the top of their priority chains:

`uram_wren_G` at [line 1555](../../hardware_code_rstdp/src/internal_events_processor.v#L1555):

```verilog
assign uram_wren_0 = (et_rmw_state==ET_RMW_WRITE && et_rmw_group_idx_latched==4'd0) ? 1'b1
                   : (curr_state==STATE_ET_DECAY_WRITE) ? 1'b1
                   : ... (other cases) ...
                   : uram_wren[0];
```

`uram_wdata_G` at [line 1201](../../hardware_code_rstdp/src/internal_events_processor.v#L1201):

```verilog
uram_wdata_0 = (et_rmw_group_idx_latched==4'd0)
             ? (et_rmw_addr_latched[0] ? {et_rmw_new_value, uram_rmwdata_lower_0}
                                       : {uram_rmwdata_upper_0,  et_rmw_new_value})
             : uram_rmwdata_0;
```

Only the target group's `wren` fires. The target group's wdata is built from `uram_rmwdata` (the unchanged half) and `et_rmw_new_value` (the updated half), selected by `et_rmw_addr_latched[0]`. So the other 36-bit half of that URAM word is preserved bit-for-bit.

On the same cycle, `iep_et_valid` pulses to WUE. WUE's outside-the-case latch in [4.7](4_7_weight_update_engine) catches it and sets `wue_et_done`.

#### ET_RMW_DONE → IDLE

A single-cycle bubble so `iep_et_valid` is exactly one cycle wide. Then back to IDLE waiting for the next `wue_et_addr_valid`.

### Worst-case latency

```
Cycle  0: wue_et_addr_valid pulses; et_rmw_pending <= 1
Cycle  1: IDLE: wait for uram_rden=0
   ...    (could stall here while Phase 2 is still using the URAM bus —
            in WUE flow this is usually already true)
Cycle  N: → READ; per-group rden fires
Cycle  N+1: WAIT; URAM data arrives; compute sum, saturate
Cycle  N+2: WRITE; URAM wren fires; iep_et_valid pulses to WUE
Cycle  N+3: DONE → IDLE
```

So an RMW takes 3 cycles once it can start, plus the IDLE stall. WUE's `RX_STATE_WUE_COMPUTE` stall is dominated by whichever of {R3 burst, ET RMW} takes longer. R3 is typically slower (HBM read), so the ET handshake is usually free.

---

## The ET decay sweep

After WUE drains the coincFIFO, IEP sweeps the configured URAM range and applies one decay step to every ET cell. Three states in IEP's main FSM:

```verilog
localparam [3:0] STATE_ET_DECAY_READ  = 4'd12;
localparam [3:0] STATE_ET_DECAY_WAIT  = 4'd14;
localparam [3:0] STATE_ET_DECAY_WRITE = 4'd13;
```

### Entry condition

[Line 1662](../../hardware_code_rstdp/src/internal_events_processor.v#L1662):

```verilog
STATE_POP_PTR_FIFO: begin
    ...
    if (exec_wue_done && et_rmw_state == ET_RMW_IDLE) begin
        if (et_uram_start_raddr != 13'd0 && et_leak_shift != 4'd0)
            next_state = STATE_ET_DECAY_READ;
        else
            next_state = STATE_PHASE2_DONE;
    end
end
```

Three conditions to enter decay:

1. `exec_wue_done` — coincFIFO drained, WUE returned to RX_IDLE.
2. `et_rmw_state == ET_RMW_IDLE` — any in-flight ET RMW finished. Without this guard, the last entry's RMW could collide with the decay sweep.
3. ET configuration is sane (`start_raddr != 0`, `et_leak_shift != 0`). Setting either to 0 disables decay entirely — useful for tests that want to verify ET accumulation without confounding it with decay.

### Sweep address

A small counter at [line 1723](../../hardware_code_rstdp/src/internal_events_processor.v#L1723):

```verilog
always @(posedge clk) begin
    if (~resetn) begin
        et_raddr <= 13'd0;
        et_decay_active <= 1'b0;
    end else if (curr_state == STATE_POP_PTR_FIFO && exec_wue_done &&
                 et_uram_start_raddr != 13'd0) begin
        // Initialize sweep address when transitioning to ET decay
        et_raddr <= et_uram_start_raddr;
        et_decay_active <= 1'b1;
    end else if (curr_state == STATE_ET_DECAY_WRITE) begin
        if (et_raddr == et_uram_end_raddr)
            et_decay_active <= 1'b0;
        else
            et_raddr <= et_raddr + 1;
    end
end
```

`et_raddr` walks from `et_uram_start_raddr` to `et_uram_end_raddr` inclusive. One URAM word per iteration.

### Read–wait–write sequence

```
STATE_ET_DECAY_READ      uram_rden=1; address = et_raddr
                         all 16 banks read in parallel
                         ────────────────────────────
STATE_ET_DECAY_WAIT      (1 cycle for URAM latency)
                         ────────────────────────────
STATE_ET_DECAY_WRITE     uram_wren=1; wdata = decay function applied
                                       to both halves of all 16 banks
                         if et_raddr == et_uram_end_raddr → STATE_PHASE2_DONE
                         else                                 → STATE_ET_DECAY_READ
```

The address steering for the sweep is at [line 872](../../hardware_code_rstdp/src/internal_events_processor.v#L872):

```verilog
end else if (curr_state == STATE_ET_DECAY_READ ||
             curr_state == STATE_ET_DECAY_WAIT ||
             curr_state == STATE_ET_DECAY_WRITE) begin
    uram_raddr_0_full  = {et_raddr[12:1], 1'b0};
    // ... all groups get the same address ...
end
```

Note: `{et_raddr[12:1], 1'b0}` — the sweep always uses the *lower* half-select, because both halves of each URAM word are processed in one pass (next).

### The decay function

[Line 1219](../../hardware_code_rstdp/src/internal_events_processor.v#L1219):

```verilog
end else if (curr_state == STATE_ET_DECAY_WRITE) begin
    uram_wdata_0  = {uram_rmwdata_upper_0  - (uram_rmwdata_upper_0  >>> et_leak_shift),
                     uram_rmwdata_lower_0  - (uram_rmwdata_lower_0  >>> et_leak_shift)};
    // ... all 16 banks ...
end
```

Both halves are decayed in parallel:

```
c_new = c_old - (c_old >>> et_leak_shift)
       = c_old * (1 - 2^-et_leak_shift)
```

`>>>` is arithmetic right shift (sign-preserving). So `et_leak_shift=4` gives ~94% retention per timestep; `et_leak_shift=8` gives ~99.6%. The host picks the value based on the desired decay time constant.

`et_leak_shift = 0` disables decay — the entry guard above bypasses the sweep states entirely in that case.

### Cost

Each iteration is 3 cycles (READ → WAIT → WRITE) × 16 banks-in-parallel × 1 URAM word. So decaying `N` rows takes `3N` cycles. For a typical sim with ~200 learning synapses (~100 URAM rows), the sweep is well under 1 µs at 225 MHz. Scales linearly in `N`.

---

## CI commands for ET configuration

Two new commands in `command_interpreter.v`. Both follow the same single-cycle latch pattern as `CMD_SET_REWARD` ([4.4](4_4_reward_and_address_mapping)).

### `CMD_SET_ET_PARAMS` (0x0B)

```
512-bit packet
┌─────────┬───────────────────┬───────────┬──────────┐
│[511:504]│                   │ [39:36]   │ [35:0]   │
│  0x0B   │                   │ et_leak_  │ et_      │
│         │                   │  shift    │ increment│
└─────────┴───────────────────┴───────────┴──────────┘
```

Latch at [`command_interpreter.v:138`](../../hardware_code_rstdp/src/command_interpreter.v#L138):

```verilog
if (et_params_wren) begin
    et_increment  <= $signed(rxFIFO_dout[35:0]);
    et_leak_shift <= rxFIFO_dout[39:36];
end
```

`et_increment` is the constant added to every ET on coincidence (placeholder rule, see caveat above). `et_leak_shift` is the decay shift.

### `CMD_SET_ET_RANGE` (0x0C)

```
512-bit packet
┌─────────┬───────────────────┬───────────┬───────────┐
│[511:504]│                   │ [25:13]   │ [12:0]    │
│  0x0C   │                   │ end_raddr │ start_    │
│         │                   │           │  raddr    │
└─────────┴───────────────────┴───────────┴───────────┘
```

Latch at [`command_interpreter.v:143`](../../hardware_code_rstdp/src/command_interpreter.v#L143):

```verilog
if (et_range_wren) begin
    et_uram_start_raddr <= rxFIFO_dout[12:0];
    et_uram_end_raddr   <= rxFIFO_dout[25:13];
end
```

These are 13-bit URAM full addresses. The decay sweep visits every URAM word in `[start_raddr .. end_raddr]`. Set both to zero to disable the sweep.

---

## Debug outputs

| Signal | Pulse? | Use |
|---|---|---|
| `dbg_et_uram_addr` | reg | URAM address of the most recent RMW |
| `dbg_et_old_value` | reg | ET value read in WAIT |
| `dbg_et_new_value` | reg | ET value written in WRITE (post-saturation) |
| `dbg_et_rmw_valid` | one cycle | Pulse when WRITE fires |

The integration TB's `check_et_rmw()` task ([4.10](4_10_integration_tb_level1)) uses `dbg_et_old_value` and `dbg_et_new_value` to verify the math.

---

## One worked RMW

WUE pops a coincFIFO entry with `r3_addr=0x8000`, `group_idx=0`. R4 returns URAM pointer `13'h008`. `et_increment = 36'sd500`. Current ET at `(bank 0, row 4, lower half)` is `36'sd1234`.

```
Cycle 0   WUE: wue_et_addr_valid ↑
              wue_et_uram_addr   = 13'h008
              wue_et_group_idx   = 4'd0

          IEP: et_rmw_addr_latched <= 13'h008
               et_rmw_group_idx_latched <= 4'd0
               et_rmw_pending <= 1
               (et_rmw_state still IDLE)

Cycle 1   IEP: uram_rden=0 (main FSM in POP_PTR_FIFO after Phase 2)
               et_rmw_state <= READ
               et_rmw_pending <= 0

Cycle 2   IEP: ET_RMW_READ
               uram_rden_0 = 1 (target group)
               uram_raddr_0_full = 13'h008 → URAM word 0x004 read
               et_rmw_state <= WAIT

Cycle 3   IEP: ET_RMW_WAIT
               uram_rdata_0[35:0] = 36'sd1234  (lower half, addr_latched[0]=0)
               old_tmp = 36'sd1234
               sum_tmp = 37'sd1234 + 37'sd500 = 37'sd1734
               no overflow → et_rmw_new_value <= 36'sd1734
               et_rmw_state <= WRITE

Cycle 4   IEP: ET_RMW_WRITE
               uram_wren_0 = 1
               uram_wdata_0 = {upper_half_untouched, 36'sd1734}
               iep_et_valid ↑
               iep_et_value = 36'sd1734
               et_rmw_state <= DONE

          WUE: wue_et_received <= 36'sd1734
               wue_et_done <= 1

Cycle 5   IEP: et_rmw_state <= IDLE
```

Three cycles from READ to WRITE, plus latch and IDLE-exit overhead.

---

## What to take from this page

- ETs live in URAM, two ETs per 72-bit word, addressed by a 13-bit full address (`{row[11:0], half_select}`).
- The ET RMW FSM is asynchronous to the main IEP FSM: it can latch a request from WUE in any state, but only *starts* the RMW when `uram_rden=0`.
- The RMW arithmetic is `c_new = saturate(c_old + et_increment)` in 37-bit signed. This is a placeholder — the real STDP-windowed rule is open work.
- The decay sweep runs once per timestep after WUE, walks `[et_uram_start_raddr .. et_uram_end_raddr]`, applies `c -= c >>> et_leak_shift` to both halves of each word.
- All four parameters (`et_increment`, `et_leak_shift`, `et_uram_start_raddr`, `et_uram_end_raddr`) come from CI commands `CMD_SET_ET_PARAMS` and `CMD_SET_ET_RANGE`.
- The reward gate is *not* in this path. ETs always update; only the weight write (in WUE) is gated by `exec_reward`. That's deliberate — eligibility persists across reward boundaries.

Next: [4.9](4_9_full_timestep_walkthrough) — all five new pieces stitched together for one full R-STDP timestep, on the 5-axon network we've been using.

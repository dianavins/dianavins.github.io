---
title: "4.4 Reward Register & Address Mapping"
parent: "4 Implementing RSTDP"
nav_order: 4
---

# 4.4 Reward Register & Address Mapping

Two small things to get out of the way before the interesting machinery starts. Both are static infrastructure. Once you understand them, you'll see them referenced everywhere in 4.5-4.8 and never need to revisit.

1. **The reward register**: how `exec_reward` enters the FPGA and gates weight writes.
2. **Region 3 / Region 4 / URAM ET layout**: how a synapse's HBM address maps to where its eligibility trace lives.

---

## The reward register

R-STDP applies a weight change only when *both* the eligibility trace is non-zero *and* a reward signal is asserted. The reward signal comes from the host via a new command, latches into a single-bit register inside `command_interpreter.v`, and propagates as `exec_reward` to `hbm_processor.v`.

### `CMD_SET_REWARD`: the host-facing command

Defined in [`command_interpreter.v:166`](../../hardware_code_rstdp/src/command_interpreter.v):

```verilog
localparam [7:0] CMD_SET_REWARD = 8'h0A;   // write exec_reward (bit 0)
```

Like every CI command, the host sends a 512-bit `rxFIFO_dout` beat. CI decodes the opcode from `[511:504]` and the reward value from `[0]`:

```
512-bit packet for CMD_SET_REWARD
┌────────────┬───────────────────────────────────────┬──────────┐
│ [511:504]  │ [503:1]   unused                       │ [0]      │
│  opcode    │                                        │  reward  │
│  = 0x0A    │                                        │   bit    │
└────────────┴───────────────────────────────────────┴──────────┘
```

The latch happens in the registered always block at [`command_interpreter.v:134`](../../hardware_code_rstdp/src/command_interpreter.v):

```verilog
if (exec_reward_wren) begin
    exec_reward <= rxFIFO_dout[0];
end
```

`exec_reward_wren` is a single-cycle strobe asserted by the FSM combinational block at [`command_interpreter.v:340`](../../hardware_code_rstdp/src/command_interpreter.v) when the decoded opcode equals `CMD_SET_REWARD`. The register holds its value until the next `CMD_SET_REWARD` (or reset).

### Where `exec_reward` is consumed

`exec_reward` is a top-level wire connected straight into `hbm_processor` ([port at line 153](../../hardware_code_rstdp/src/hbm_processor.v#L153)). It does exactly one thing in RTL: it qualifies whether the weight write-back is queued. In the WUE `RX_STATE_WUE_COMPUTE` block at [line 1160](../../hardware_code_rstdp/src/hbm_processor.v#L1160):

```verilog
// Weight updated only when reward is active
wue_r3_wb_pending <= (wue_do_weight_update && exec_reward) ? 1'b1 : 1'b0;
```

So even if a coincidence was detected (`wue_do_weight_update=1`), no HBM write happens unless `exec_reward=1` at compute time. The eligibility trace itself is still updated either way (see [4.8](4_8_eligibility_trace)).

### Why a register and not a wire

A more obvious design would be to send the reward signal directly on the command packet that runs each timestep. The register form has two advantages:

- The host can set reward once and run many timesteps without re-sending it each step.
- The reward value is stable across whatever WUE entries are in flight when the host updates it, so there is no glitching mid-update.

The downside: the host has to remember the current state. A 100-timestep RL episode with reward changes only on steps 17 and 88 sends just three `CMD_SET_REWARD` packets.

---

## Region 3, Region 4, and the URAM ET layout

Chapter 1.1 introduced HBM Regions 1, 2, and 3:

- **Region 1** (base `0x0000`): axon pointers
- **Region 2** (base `0x4000`): neuron pointers
- **Region 3** (base `0x8000`): synapse data, 8 synapses per 256-bit row

R-STDP adds one more region:

- **Region 4**: eligibility trace **pointers** (not values), one 256-bit row per Region 3 row.

R4 sits at a fixed offset above R3, configurable per-deployment.

### `REGION4_OFFSET` and how R4 addresses are computed

`region4_offset` is a 23-bit input port to `hbm_processor.v` ([line 150](../../hardware_code_rstdp/src/hbm_processor.v#L150)) driven from a top-level register that the host can program. WUE uses it to derive the R4 address for any coincFIFO entry at [line 1069](../../hardware_code_rstdp/src/hbm_processor.v#L1069):

```verilog
wue_hbm_addr <= coincfifo_dout[22:0] + region4_offset;
```

So for any R3 row at address `R`:

```
r4_addr = r3_addr + region4_offset
```

If a deployment uses 24 KB of R3 (3000 rows of 256-bit) then `region4_offset = 23'd3000` puts R4 immediately above R3. The MEMORY.md notes a typical sim value of `23'h010000`.

R4 reverse-maps just as easily: `r3_addr = r4_addr − region4_offset`. The WUE never actually computes the reverse (it issues both reads from the same `r3_addr` field), but a TB writing R4 cells needs the forward form.

### What's in a Region 4 row

This is the part that surprises people. **R4 does not store ET values.** It stores a 13-bit pointer into URAM, one pointer per slot. The ET value itself lives in URAM.

Layout of one Region 4 beat (256 bits, one row):

```
[255:13]   unused
[12:0]     URAM full address of this synapse's ET cell
```

The 13-bit pointer is the *same* address format that IEP uses for membrane potentials in [Chapter 3.7 (`internal_events_processor.v`)](../3_Verilog_Files_Review/internal_events_processor). Recall: each URAM bank has 4096 × 72-bit words; each 72-bit word stores **two** 36-bit signed values; the 13-bit address splits as:

```
URAM full address [12:0]
┌──────────────────┬──────┐
│   row  [12:1]    │ [0]  │
│   (4096 rows)    │ half │
│                  │ sel  │
└──────────────────┴──────┘

[12:1]  selects the 72-bit word.
[0] = 0 selects the lower  36-bit half.
[0] = 1 selects the upper 36-bit half.
```

When WUE reads an R4 row, it forwards the 13 bits to IEP as `wue_et_uram_addr`. IEP's ET RMW FSM (covered in [4.8](4_8_eligibility_trace)) uses bits `[12:1]` to address the URAM word and bit `[0]` to mux between the upper and lower 36-bit halves.

### Why an indirection through URAM at all

The original step-2 design parked ET values directly in R4, with `r4[255:220] = signed 36-bit ET`. That had three problems:

1. **Race against decay.** The ET decay sweep needs random-access read-modify-write over a contiguous range; HBM is poorly suited to that pattern.
2. **Write amplification.** Every WUE entry would issue a 256-bit R4 write back, even though only 36 bits changed.
3. **No double-write protection.** Two coincFIFO entries targeting the same ET in one timestep could conflict if both tried to update the same R4 row.

Moving ETs into URAM (one per 36-bit half-word) gives single-cycle RMW, easy linear sweep for decay, and per-bank parallelism. The R4 row stays as a *pointer table* so the host's R-STDP-aware compiler still controls which URAM cell each synapse maps to.

The full reasoning is in [`hardware_code_rstdp/et_location_bug_fix_plan.txt`](../../hardware_code_rstdp/et_location_bug_fix_plan.txt) if you want it.

---

## A worked address example

Use the 5-axon / 5-hidden / 5-output network from [Chapter 1.1](../1_Initializing_the_Network/Chapter_1_1) for concreteness. Suppose:

- Axon `a0` connects to `h0` via synapse stored at R3 row `0x8000`, slot 0.
- `region4_offset = 23'h010000`.
- The host compiler assigned this synapse's ET to URAM bank G=3, row `0x4`, lower half.

Then the URAM full address of the ET is:

```
{ row[11:0] = 0x004, half = 0 } = 13'h008
```

And the R4 row at `0x18000` holds:

```
[255:13]  zeros
[12:0]    13'h008
```

When WUE pops a coincFIFO entry with `r3_addr = 23'h8000` and `group_mask = 16'h0008` (group 3):

1. WUE computes `r4_addr = 23'h8000 + 23'h010000 = 23'h018000`.
2. WUE issues a 1-beat HBM read at `r4_addr`. Beat returns `13'h008`.
3. WUE forwards `wue_et_uram_addr = 13'h008` and `wue_et_group_idx = 4'd3` to IEP.
4. IEP's ET RMW FSM reads URAM bank 3, word `0x4`, picks the lower half. Increments. Writes back.
5. IEP returns the new ET on `iep_et_value`. WUE adds it to the old weight (read from R3) and writes the new weight to R3.

Every page from here on assumes you can do this mapping in your head.

---

## Two sibling commands you'll see referenced

While we're in `command_interpreter.v`, two more new opcodes are worth knowing about. Both write ET-related configuration that [4.8](4_8_eligibility_trace) uses in depth. They are listed here just so you recognize them when grepping.

| Opcode | Name | Payload | What it sets |
|---|---|---|---|
| `8'h0A` | `CMD_SET_REWARD` | `[0]` | `exec_reward` (this page) |
| `8'h0B` | `CMD_SET_ET_PARAMS` | `[35:0]` → `et_increment`, `[39:36]` → `et_leak_shift` | Per-spike ET increment and decay shift |
| `8'h0C` | `CMD_SET_ET_RANGE` | `[12:0]` → `et_uram_start_raddr`, `[25:13]` → `et_uram_end_raddr` | Which URAM rows the decay sweep covers |

All three follow the same pattern: opcode in the top byte, payload in the low bits, single-cycle latch into a register at the top of `command_interpreter.v`.

---

## What to take from this page

- `CMD_SET_REWARD` (0x0A) latches `rxFIFO_dout[0]` into `exec_reward`. That register gates `wue_r3_wb_pending` in WUE.
- `r4_addr = r3_addr + region4_offset`. `region4_offset` is a CI-programmed input to `hbm_processor`.
- An R4 beat stores a 13-bit **URAM pointer**, not an ET value. The ET lives in URAM, two ETs per 72-bit word, addressed `{row[11:0], half}`.
- The two sibling commands (`CMD_SET_ET_PARAMS`, `CMD_SET_ET_RANGE`) configure decay parameters, covered in [4.8](4_8_eligibility_trace).

Next, [4.5](4_5_r3_address_tracking) explains how `hbm_processor` keeps track of which R3 address each 512-bit synapse packet came from, so the ASB has something to record.

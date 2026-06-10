---
title: "4.12 Where the Work Picks Up"
parent: "4 Implementing RSTDP"
nav_order: 12
---

# 4.12 Where the Work Picks Up

The hardware in 4.3-4.11 simulates a complete R-STDP loop. It is not yet running on silicon, integrated with `hs_api`, or exercising a real RL task. This page is the punch list of what's open, in roughly the order you should attack it, with pointers to the design notes that already exist.

If you're a new student trying to pick a starting project, read this page top to bottom. The items are in dependency order: earlier items unblock later ones, and the final item (the ET rule rewrite) is best tackled once you have the rest of the infrastructure to validate against. The general steps are human advised, details are generated and need double-checked.

---

## 1. Get the host TBs fully functioning; recreate the software plots

**Where it lives:**

- [`hardware_code_rstdp/tb/step8_rstdp_100step_tb.sv`](../../hardware_code_rstdp/tb/step8_rstdp_100step_tb.sv): the 100-step regression.
- [`prelimenary_rstdp/`](../../prelimenary_rstdp/): software R-STDP runs Diana built earlier (reference traces and plot scripts).
- [`hardware_code_rstdp/tb/generate_network_commands_rstdp.py`](../../hardware_code_rstdp/tb/generate_network_commands_rstdp.py): Python compiler scaffold for the test network.

**What's there now:** The 100-step TB runs and produces `prelimenary_rstdp/step8_rstdp_100step.csv` with spike rasters, ET values, reward timing, and weights per timestep. The PNG at `step8_rstdp_100step.png` is generated from that CSV.

**What's open:**

- Compare the hardware-generated trace against the software-generated trace from the same network under identical inputs. They should match bit-exactly (modulo decay rule choice).
- The current host-style TB ([4.11](4_11_host_style_tb_level2)) runs a small network; verify it reproduces the same shape of ET/weight trajectories as the software model.
- Extend the 100-step TB to larger networks. Where does the simulation become impractically slow? That sets a practical limit on what can be debugged in sim vs. requiring bitstream.

This is mostly about *validation* rather than RTL work. It's a good entry point for someone who wants to learn the system end-to-end without writing Verilog.

---

## 2. Bitstream + on-FPGA test

**Status:** RTL has not been synthesized or run on hardware. Sim only.

**What needs to happen:**

1. **Synthesis sanity check.** Run the RTL through Vivado synthesis. The ASB generate block (1024 comparators) and the WUE compute paths may exceed timing on the existing 225 MHz target, so measure before optimizing.
2. **Resource budget.** ASB is ~30 Kb of registers. Make sure the target part has it; the original ISN spec ([`hardware_code_original/ISN - Documentation.pdf`](../../hardware_code_original/ISN%20-%20Documentation.pdf)) lists URAM/BRAM/register budgets you can compare against.
3. **Bitstream + smoke.** Generate a bitstream, load it onto the FPGA via the existing JTAG/PCIe flow. Run the smallest possible network (one axon, one neuron, one synapse) and verify it does the right thing.
4. **Telemetry.** The current TB peeks RTL internals (`dbg_wue_*`, `dbg_et_*`, `et_rmw_state`). None of that is visible from the host once the bitstream is loaded. Decide which debug signals to expose as VIO/ILA or as readable CI registers *before* synthesizing. The starter telemetry set is in 3j below.

---

## 3. `hs_api` / `hs_bridge` integration

This is the single biggest piece of work, and the design notes for it already exist: [`hs_bridge_and_api_plan.txt`](../../hardware_code_rstdp/hs_bridge_and_api_plan.txt). Skim it before doing anything else; the items below are summaries.

### 3a. CI opcode gap

The RTL command interpreter knows about original opcodes `0x01`-`0x04` and `0x06`-`0x08`, plus the three new R-STDP opcodes added for this work:

- `CMD_SET_REWARD = 0x0A`
- `CMD_SET_ET_PARAMS = 0x0B`
- `CMD_SET_ET_RANGE = 0x0C`

The host-side opcode file `formats.org` in [`hs_bridge/`](../../hs_bridge/) only documents `0x01` (exec_step), `0x02` (HBM R/W), `0x03` (URAM read), and `0x04` (network params + flush). So there are already gaps: the host file doesn't know about `0x06`-`0x08` (which the RTL uses) *and* it doesn't know about the three new R-STDP opcodes.

Add all of them to `formats.org` with bit positions matching the RTL. CI opcode-skew between RTL and host is the single most common silent-failure source.

### 3b. Three-layer migration order

Migrate, validating at each layer:

1. **RTL.** Confirm the new opcodes parse correctly through the same packet path the TB uses (`ci2hbm`, `ci2iep`, `axonEvent`). [4.11](4_11_host_style_tb_level2)'s Level 2 TBs already do this. If the TB pushes packets directly into FIFOs (bypassing `pcie2fifos`), also exercise the upstream host-bridge decoder so you know it parses 512-bit beats correctly.
2. **`hs_bridge`.** Add Python functions in [`hs_bridge/base_functions.py`](../../hs_bridge/) that emit the right 512-bit hex string and hand it to `adxdma_dmadump wb`. Mirror the existing `execute()` / `write` / `read` pattern.
3. **`hs_api`.** Expose those as methods on `Network` (e.g. `network.set_reward(1)`, `network.set_et_params(inc, leak)`). Match the existing API style.

Don't try to jump from RTL straight to `hs_api`. Debugging across all three layers at once is hard.

### 3c. Network compilation: the OUTPUT pointer page

[`hs_bridge/compile_network.py`](../../hs_bridge/) already builds the INPUT pointer page (axon pointers) at HBM `0x000000`. The on-chip neuron-to-neuron Phase 1b path (validated by Level 2 test T5) needs the OUTPUT pointer page at HBM `0x004000`, with one entry per neuron pointing at its downstream synapses.

Confirm whether the existing compiler emits this page. If it doesn't, that's the biggest single piece of new compiler work; do it before tackling later items.

### 3d. ET URAM cell allocation

Each R-STDP synapse maps to a URAM ET cell. The current `generate_network_commands_rstdp.py` hand-picks addresses. The real compiler needs a deterministic allocator:

```python
def assign_et_address(synapse):
    # Given a synapse, return its URAM ET full address.
    # Must be deterministic: host and TB must produce the same mapping.
    ...
```

Decide whether to expose this as a synapse attribute or hide it inside `compile_network.py`.

### 3e. Readback for plotting

To recreate the 100-step plot on real hardware, per timestep you need:

- URAM read of each neuron's membrane potential cell (`CMD_IEP_RW`)
- URAM read of each tracked ET cell (`CMD_IEP_RW`)
- HBM read of the synapse row containing each tracked weight (`CMD_HBM_RW`)

Round-trip per step is slow on real PCIe (PCIe + DMA + subprocess overhead is ~ms each), versus nanoseconds in sim. If you need fast streaming, either batch reads at the end of a run, or add a new CI command that auto-emits a "telemetry beat" at every `exec_step` rather than serving pull-on-demand reads.

### 3f. exec_done synchronization

The TB waits for an internal signal (`exec_uram_phase2_done` and friends). Real hardware needs to expose this via either:

- A status register polled over CI.
- The existing outgoing-spike return path also signaling step completion (the `EEEE` / `execRun_ctr` path already in `formats.org`).

The latter is cleaner: every `exec_step` already returns spike data plus a counter, so the host can simply block on receiving that beat before sending the next packet.

### 3g. Soft reset path

The TB calls `apply_reset()` between tests. From host, you need either a CI-level soft-reset command or a register write to a control reg. Without it, ET / weight / ASB state from a previous run persists into the next. Add it as a new CI opcode (e.g. `CMD_SOFT_RESET`).

### 3h. Bitstream / RTL version pinning

Add a read-only version register exposed via a CI read command. On `Network.connect()`, `hs_bridge` reads it and refuses to run if the bitstream rev doesn't match what the host code expects. CI opcode-skew between RTL and host is the second-most-common silent-failure source.

### 3i. Validation strategy: golden trace

The 100-step sim from item 1 produces a deterministic CSV (ending at weight = 3645 for the current rule). Use that CSV as the acceptance test for host migration: same Poisson seed, same axon pattern, same reward schedule, run it through `hs_api`/`hs_bridge` on hardware, dump a CSV from the host side, diff against the sim CSV. Bit-exact agreement (modulo readback timing) means the migration is correct.

### 3j. Debug visibility on hardware

In sim you have VCD plus the AXI scoreboards. On hardware you only have what CI exposes. Before synthesizing, decide on a minimum HW-side telemetry set:

- coincFIFO push counter (readable counter register)
- WUE writeback count
- IEP FSM state register
- Last R3 address WUE wrote to

Add these as readable CI registers / commands *now*, while you still have the integration TB to verify them. Adding them later, while debugging on silicon, is much harder.

### 3k. Co-sim intermediate

Between sim-only and real bitstream, there's a useful middle step: keep the RTL in simulation but drive it from Python over a TCP socket or DPI bridge. The host code is *real*, but you still have VCD. This isolates "is my Python wrong?" from "is my hardware path wrong?" If you're new to FPGA debug, the day to set this up is worth it.

---

## 4. `simpleSim` integration

**Where it lives:** [`hs_api/simpleSim.py`](../../hs_api/), the Python software simulator that mirrors hardware behavior for development without an FPGA.

**Status:** No R-STDP support. Stops at the original two-phase model.

**What needs to happen:**

- Add a Python implementation of the ASB (Python dict keyed by `(group, target_addr)` works).
- Add a Python coincidence detection step at the right point in the timestep loop.
- Add the ET RMW path, decay sweep, and reward gate to match the RTL.
- Most importantly: **match the RTL bit-for-bit, including the saturation behavior and the ET URAM addressing format.**

The 100-step CSV from [4.11](4_11_host_style_tb_level2) is the golden trace. If `simpleSim` reproduces that CSV exactly, the integration is correct.

This is a contained software project. Good entry point for someone who likes Python.

---

## 5. RL task

The end goal of R-STDP on this hardware. Two candidate problems in `plan.txt`:

- **Frozen Lake V1** (Gymnasium). Discrete states (4×4 grid), discrete actions (up/down/left/right), sparse reward. Small enough to fit in URAM.
- **Lane keeping.** Continuous-ish, requires a perception stack feeding into the SNN.

Frozen Lake is the cleaner first target. Roughly:

- Encode the agent's grid position as one-hot axon firing (16 axons).
- Build a 16 → ~32 → 4 network with R-STDP-trained synapses on every layer.
- Output neurons spike → host reads spike via existing `txFIFO` path → host decodes action.
- After action, environment returns reward; host calls `CMD_SET_REWARD` for the next several timesteps (matching the credit-assignment window via the ET time constant).
- Many episodes → ETs accumulate eligible synapses; reward shapes the weight landscape.

This depends on items 2 (bitstream) and 3 (`hs_api`). It's the carrot at the end of the stick, not a starting project.

---

## 6. Optional extensions to the R-STDP rules

The current rules are a valid R-STDP implementation at one-timestep resolution. Coincidence detection collapses the STDP window to a binary "did pre fire last step AND did post spike this step?" check. The eligibility trace gets a fixed `et_increment` bump on each coincidence push and decays geometrically each timestep. Many practical R-STDP implementations look exactly like this.

This item exists only if you want *richer* rules. None of the extensions below are required for correctness.

**Current implementation:**

```
coincidence  = post spikes in T+1 Phase 1  AND  pre fired in T Phase 2 at this synapse  (ASB-driven)
c_new        = saturate(c_old + et_increment)   on each coincFIFO entry
c            = c - (c >>> et_leak_shift)        on every timestep (decay sweep)
```

### 6a. Verify the mem-only push ET behavior

The one item in this section that may actually be a bug: WUE issues the R4 read (and therefore the ET RMW handshake) on every coincFIFO entry, including mem-only entries (`do_wu = 0`). So an ET cell can get incremented when there's no real Phase 1 coincidence, just because pre-activity happened to land at that synapse during Phase 2.

Decide whether that's intended:

- If the eligibility trace is supposed to be pure coincidence-driven (the canonical reading), suppress the ET RMW for `do_wu = 0` entries. The fix lives in `hbm_processor.v` `RX_STATE_WUE_WAIT_R4`: gate `wue_et_addr_valid` on `wue_do_weight_update`.
- If pre-activity alone should contribute (a single-sided rule), the current behavior is fine. Document the choice.

### 6b. Add an LTD direction (two-sided rule)

The current rule has only one direction. Coincidence pushes `c` up. Decay pulls it down, but symmetric decay isn't depression in the STDP sense.

### 6c. Real STDP window instead of one-timestep coincidence

The current coincidence detection ([4.6](4_6_coincidence_detection)) collapses the STDP window to one timestep. The ASB only holds the *previous* timestep's synapse arrivals (double-buffered, swap on `exec_run`), so the only "spike timing" question the comparator can answer is binary: pre fired in T's Phase 2 or it didn't.

A proper STDP window over `N` timesteps (or sub-timestep cycle counts) would let the hardware approximate the kernel `W(Δt)` from [4.1](4_1_what_is_rstdp), with graded magnitudes for pre-before-post pairs and the opportunity to detect post-before-pre pairs for LTD.

### Tunables, not bugs

`et_increment`, `et_leak_shift`, `et_uram_start_raddr`, and `et_uram_end_raddr` are all live knobs settable via `CMD_SET_ET_PARAMS` and `CMD_SET_ET_RANGE`. Sweeping them to reproduce the [`prelimenary_rstdp/`](../../prelimenary_rstdp/) software traces is part of item 1, not a separate piece of work.

---

## Where the design notes live

A new student will benefit from reading these in order. They're all in [`hardware_code_rstdp/`](../../hardware_code_rstdp/):

| File | What's in it | When to read |
|---|---|---|
| [`plan.txt`](../../hardware_code_rstdp/plan.txt) | The 8-step bring-up plan + the "next steps" list this page is based on. | First. After the prereqs in [4.3](4_3_rstdp_hardware_roadmap). |
| [`CHANGES.md`](../../hardware_code_rstdp/CHANGES.md) | The step-8 workaround removal + the `hbm_count` RTL fix with reproducible test cases. | Read once. Lessons generalize. |
| [`RTL_fixes_explained.txt`](../../hardware_code_rstdp/RTL_fixes_explained.txt) | Three bug fixes (FIX #1 = mem-only push addr; FIX #2 = duplicate-push guard; FIX #3 = hbm_count R4→R3 transition) and the secondary `asb_r3_extract` ordering fix. | Read before touching the coincidence push logic or the WUE RX FSM. |
| [`IEP_coincidence_logic_explained.txt`](../../hardware_code_rstdp/IEP_coincidence_logic_explained.txt) | Line-by-line walkthrough of the ASB + coincidence logic. The source material for [4.6](4_6_coincidence_detection). | Read alongside [4.6](4_6_coincidence_detection) when you need finer-grained citations than this chapter provides. |
| [`et_location_bug_fix_plan.txt`](../../hardware_code_rstdp/et_location_bug_fix_plan.txt) | The polished plan for moving ETs from HBM R4 into URAM. Flags every interaction that could break. The source material for [4.7](4_7_weight_update_engine) and [4.8](4_8_eligibility_trace). | Read when working on the WUE ↔ IEP boundary or the ET RMW FSM. |
| [`hs_bridge_and_api_plan.txt`](../../hardware_code_rstdp/hs_bridge_and_api_plan.txt) | 11-point host migration plan. The source material for item 3 on this page. | Read before any `hs_bridge` or `hs_api` work. |

---

## Where the data lives

The simulation-only outputs are checked in:

- [`prelimenary_rstdp/step8_rstdp_100step.csv`](../../prelimenary_rstdp/): the 100-step golden CSV.
- [`prelimenary_rstdp/step8_rstdp_100step.png`](../../prelimenary_rstdp/): the plot of that CSV.
- [`hardware_code_rstdp/tb_logs/`](../../hardware_code_rstdp/tb_logs/): recent simulation log outputs. Useful for diffing against a fresh run after RTL changes.

If you change the RTL, regenerate both files and check the diff carefully before committing.

---

## End of Chapter 4

That's everything currently known and currently open in the R-STDP hardware track. Future chapters will accrete as items in this page complete. If you finish something on the list, update this page (and the relevant page above), because the docs are the handoff, not the code.

Welcome to the project.

---
title: "4.12 Where the Work Picks Up"
parent: "4 Implementing RSTDP"
nav_order: 12
---

# 4.12 Where the Work Picks Up

The hardware in 4.3–4.11 simulates a complete R-STDP loop. It is not yet running on silicon, integrated with `hs_api`, exercising a real RL task, or using a final eligibility-trace rule. This page is the punch list of what's open, in roughly the order you should attack it, with pointers to the design notes that already exist.

If you're a new student trying to pick a starting project: read this page top to bottom, then skim section 4 at the end ("How to choose a starting task"). Don't pick the biggest item first.

---

## 1. Fix the eligibility trace update rule

**Where it lives:** [`internal_events_processor.v`](../../hardware_code_rstdp/src/internal_events_processor.v), `ET_RMW_WAIT` block at [line 1795](../../hardware_code_rstdp/src/internal_events_processor.v#L1795).

**What's there now:**

```
c_new = saturate(c_old + et_increment)
```

`et_increment` is a constant — there's no dependence on spike timing, no STDP kernel, no Δt logic. Every coincidence push (and every mem-only push) results in the same increment.

**What it should be:**

A proper STDP-windowed update. The form in [4.1](4_1_what_is_rstdp):

```
ċ(t) = -c/τ_c + W(Δt) · δ(t − s_pre/post) · C_1
```

translated to discrete timesteps gives `c_new = decay(c_old) + W(Δt)` where `W(Δt)` is the STDP kernel — typically exponential with a positive lobe for `Δt > 0` (potentiation) and a negative lobe for `Δt < 0` (depression).

**The hardware to add:**

- A representation of `Δt` per ASB entry. The ASB ([4.6](4_6_coincidence_detection)) records *which timestep* an entry was inserted in implicitly (via buffer roles), but not finer than that. To capture sub-timestep timing you'd need to store the cycle counter at insertion time, or restructure the ASB to hold multi-timestep history.
- A small LUT or piecewise-linear approximation of `W(Δt)`. Likely a few dozen entries indexed by `Δt`.
- New arithmetic in `ET_RMW_WAIT`: read the kernel value, multiply (or shift-and-add) by `et_increment`, add to `c_old`, clamp.

**Caveats this opens up:**

- The current design has one ET per synapse. STDP could fire at any pre-synaptic spike or any post-synaptic spike, but the current RMW path is only triggered by the coincFIFO (which is post-synaptic-spike-driven). Pre-only spikes don't update ETs. Decide whether that's acceptable for the regime you're modeling, or restructure.
- Mem-only push entries (where there's no real coincidence, see [4.6](4_6_coincidence_detection)) currently *do* invoke ET RMW. With a proper STDP-windowed rule, this would be wrong — the rule would update an ET in response to pure Phase 2 traffic. Suppress the RMW for `do_wu = 0` entries.

**Suggested first cut:** a simple two-piece kernel — fixed positive `et_increment_post` on coincidence, fixed negative `et_increment_pre` on a (new) pre-only event path. Sweep `et_leak_shift` and the two increments until simple Pavlovian conditioning examples (from the [`prelimenary_rstdp/`](../../prelimenary_rstdp/) software runs) reproduce.

---

## 2. Get the host TBs fully functioning; recreate the software plots

**Where it lives:**

- [`hardware_code_rstdp/tb/step8_rstdp_100step_tb.sv`](../../hardware_code_rstdp/tb/step8_rstdp_100step_tb.sv) — the 100-step regression.
- [`prelimenary_rstdp/`](../../prelimenary_rstdp/) — software R-STDP runs Diana built earlier (reference traces and plot scripts).
- [`hardware_code_rstdp/tb/generate_network_commands_rstdp.py`](../../hardware_code_rstdp/tb/generate_network_commands_rstdp.py) — Python compiler scaffold for the test network.

**What's there now:** The 100-step TB runs and produces `prelimenary_rstdp/step8_rstdp_100step.csv` with spike rasters, ET values, reward timing, and weights per timestep. The PNG at `step8_rstdp_100step.png` is generated from that CSV.

**What's open:**

- Compare the hardware-generated trace against the software-generated trace from the same network under identical inputs. They should match bit-exactly (modulo decay rule choice).
- The current host-style TB ([4.11](4_11_host_style_tb_level2)) runs a small network; verify it reproduces the same shape of ET/weight trajectories as the software model.
- Extend the 100-step TB to larger networks. Where does the simulation become impractically slow? That sets a practical limit on what can be debugged in sim vs. requiring bitstream.

This is mostly about *validation* rather than RTL work. It's a good entry point for someone who wants to learn the system end-to-end without writing Verilog.

---

## 3. Bitstream + on-FPGA test

**Status:** RTL has not been synthesized or run on hardware. Sim only.

**What needs to happen:**

1. **Synthesis sanity check.** Run the RTL through Vivado synthesis. The ASB generate block (1024 comparators) and the WUE compute paths may exceed timing on the existing 225 MHz target — measure before optimizing.
2. **Resource budget.** ASB is ~30 Kb of registers. Make sure the target part has it; the original ISN spec ([`hardware_code_original/ISN - Documentation.pdf`](../../hardware_code_original/ISN%20-%20Documentation.pdf)) lists URAM/BRAM/register budgets you can compare against.
3. **Bitstream + smoke.** Generate a bitstream, load it onto the FPGA via the existing JTAG/PCIe flow. Run the smallest possible network (one axon, one neuron, one synapse) and verify it does the right thing.
4. **Telemetry.** The current TB peeks RTL internals (`dbg_wue_*`, `dbg_et_*`, `et_rmw_state`). None of that is visible from the host once the bitstream is loaded. Decide which debug signals to expose as VIO/ILA or as readable CI registers *before* synthesizing.

The [`hs_bridge_and_api_plan.txt`](../../hardware_code_rstdp/hs_bridge_and_api_plan.txt) item 10 ("Debug visibility loss") lists a starter telemetry set: coincFIFO push counter, WUE writeback count, IEP FSM state register, last R3 address WUE wrote to. Add these as readable CI registers *while you still have the TB to verify them*.

---

## 4. `hs_api` / `hs_bridge` integration

This is the single biggest piece of work, and the design notes for it already exist: [`hs_bridge_and_api_plan.txt`](../../hardware_code_rstdp/hs_bridge_and_api_plan.txt). Skim it before doing anything else; the items below are summaries.

### 4a. CI opcode gap

The three new CI commands (`CMD_SET_REWARD = 0x0A`, `CMD_SET_ET_PARAMS = 0x0B`, `CMD_SET_ET_RANGE = 0x0C`) have no host-side counterpart. The host-side opcode file [`formats.org`](../../hs_bridge/) only knows about the original opcodes (0x01–0x04, 0x06–0x08). Add the three new ones with their bit positions; the host driver and the RTL CI must agree.

This is the single most common silent-failure source. Do it before anything else in this section.

### 4b. Three-layer migration order

Migrate, validating at each layer:

1. **RTL** — confirm the new opcodes parse correctly through the same packet path the TB uses (`ci2hbm`, `ci2iep`, `axonEvent`). [4.11](4_11_host_style_tb_level2)'s Level 2 TBs already do this.
2. **`hs_bridge`** — add Python functions in [`hs_bridge/base_functions.py`](../../hs_bridge/) that emit the right 512-bit hex string and hand it to `adxdma_dmadump wb`. Mirror the existing `execute()` / `write` / `read` pattern.
3. **`hs_api`** — expose those as methods on `Network` (e.g. `network.set_reward(1)`, `network.set_et_params(inc, leak)`). Match the existing API style.

Don't try to jump from RTL straight to `hs_api` — debugging across all three layers at once is hard.

### 4c. Network compilation: the OUTPUT pointer page

[`hs_bridge/compile_network.py`](../../hs_bridge/) already builds the INPUT pointer page (axon pointers) at HBM `0x000000`. The on-chip neuron-to-neuron Phase 1b path (validated by Level 2 test T5) needs the OUTPUT pointer page at HBM `0x004000` — one entry per neuron pointing at its downstream synapses.

If the existing compiler doesn't emit this page, that's the biggest single piece of new compiler work. Confirm before tackling later items.

### 4d. ET URAM cell allocation

Each R-STDP synapse maps to a URAM ET cell. The current `generate_network_commands_rstdp.py` hand-picks addresses. The real compiler needs a deterministic allocator:

```python
def assign_et_address(synapse):
    # Given a synapse, return its URAM ET full address.
    # Must be deterministic — host and TB must produce the same mapping.
    ...
```

Decide whether to expose this as a synapse attribute or hide it inside `compile_network.py`.

### 4e. Readback for plotting

To recreate the 100-step plot on real hardware, per timestep you need:

- URAM read of each neuron's membrane potential cell
- URAM read of each tracked ET cell
- HBM read of the synapse row containing each tracked weight

Round-trip per step is slow on real PCIe (~ms each), vs. nanoseconds in sim. If you need fast streaming, either batch reads at the end of a run, or add a CI "telemetry beat" command that auto-emits per-step state. Item 5 in `hs_bridge_and_api_plan.txt` discusses this.

### 4f. exec_done synchronization

The TB waits for an internal signal (`exec_uram_phase2_done` and friends). Real hardware needs to expose this via either:

- A status register polled over CI.
- The existing outgoing-spike return path also signaling step completion (the `EEEE`/`execRun_ctr` already in `formats.org`).

The latter is cleaner: every `exec_step` already returns spike data + a counter; the host blocks on receiving that beat before sending the next packet.

### 4g. Soft reset path

The TB calls `apply_reset()` between tests. From host, you need either a CI-level soft reset command or a register write to a control reg. Without it, ET / weight / ASB state from a previous run persists into the next. Add it as a new CI opcode (e.g. `CMD_SOFT_RESET`).

### 4h. Bitstream / RTL version pinning

Add a read-only version register exposed via a CI read command. On `Network.connect()`, `hs_bridge` reads it and refuses to run if the bitstream rev doesn't match what the host code expects. CI opcode-skew between RTL and host is the second-most-common silent-failure source.

### 4i. Co-sim intermediate

Between sim-only and real bitstream, there's a useful middle step: keep the RTL in simulation but drive it from Python over a TCP socket or DPI bridge. The host code is *real*, but you still have VCD. This isolates "is my Python wrong?" from "is my hardware path wrong?" If you're new to FPGA debug, the day to set this up is worth it.

---

## 5. `simpleSim` integration

**Where it lives:** [`hs_api/simpleSim.py`](../../hs_api/) — the Python software simulator that mirrors hardware behavior for development without an FPGA.

**Status:** No R-STDP support. Stops at the original two-phase model.

**What needs to happen:**

- Add a Python implementation of the ASB (Python dict keyed by `(group, target_addr)` works).
- Add a Python coincidence detection step at the right point in the timestep loop.
- Add the ET RMW path, decay sweep, and reward gate to match the RTL.
- Most importantly: **match the RTL bit-for-bit, including the saturation behavior and the ET URAM addressing format.**

The 100-step CSV from [4.11](4_11_host_style_tb_level2) is the golden trace. If `simpleSim` reproduces that CSV exactly, the integration is correct.

This is a contained software project. Good entry point for someone who likes Python.

---

## 6. RL task

The end goal of R-STDP on this hardware. Two candidate problems in `plan.txt`:

- **Frozen Lake V1** (Gymnasium). Discrete states (4×4 grid), discrete actions (up/down/left/right), sparse reward. Small enough to fit in URAM.
- **Lane keeping.** Continuous-ish, requires a perception stack feeding into the SNN.

Frozen Lake is the cleaner first target. Roughly:

- Encode the agent's grid position as one-hot axon firing (16 axons).
- Build a 16 → ~32 → 4 network with R-STDP-trained synapses on every layer.
- Output neurons spike → host reads spike via existing `txFIFO` path → host decodes action.
- After action, environment returns reward; host calls `CMD_SET_REWARD` for the next several timesteps (matching the credit-assignment window via the ET time constant).
- Many episodes → ETs accumulate eligible synapses; reward shapes the weight landscape.

This depends on items 1 (rule fix), 3 (bitstream), and 4 (`hs_api`). It's the carrot at the end of the stick, not a starting project.

---

## How to choose a starting task

| If you have … | Start with … |
|---|---|
| Strong Python, light Verilog | Item 5 (`simpleSim`) or item 2 (trace comparison). |
| Strong Verilog, light SNN theory | Item 1 (ET rule) once you read up on STDP — or [`RTL_fixes_explained.txt`](../../hardware_code_rstdp/RTL_fixes_explained.txt) and look for any remaining edge cases. |
| Strong systems / driver background | Item 4 (`hs_api`/`hs_bridge` migration). The plan in `hs_bridge_and_api_plan.txt` is thorough; the work is mechanical once you understand the existing driver. |
| Strong digital design + want a synthesis project | Item 3 (bitstream + on-FPGA smoke). Doesn't require deep SNN knowledge; lots of practical FPGA debug. |
| Strong SNN + RL background | Item 6 (Frozen Lake) — but only after items 1, 3, 4 land. |

**Avoid:** Trying to do item 6 first. Trying to "rewrite the ET rule" without first running the existing 100-step TB and understanding where the current rule produces wrong behavior. Trying to do hs_api integration without first reading `hs_bridge_and_api_plan.txt` end-to-end.

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
| [`hs_bridge_and_api_plan.txt`](../../hardware_code_rstdp/hs_bridge_and_api_plan.txt) | 11-point host migration plan. The source material for item 4 on this page. | Read before any `hs_bridge` or `hs_api` work. |

---

## Where the data lives

The simulation-only outputs are checked in:

- [`prelimenary_rstdp/step8_rstdp_100step.csv`](../../prelimenary_rstdp/) — the 100-step golden CSV.
- [`prelimenary_rstdp/step8_rstdp_100step.png`](../../prelimenary_rstdp/) — the plot of that CSV.
- [`hardware_code_rstdp/tb_logs/`](../../hardware_code_rstdp/tb_logs/) — recent simulation log outputs. Useful for diffing against a fresh run after RTL changes.

If you change the RTL, regenerate both files and check the diff carefully before committing.

---

## End of Chapter 4

That's everything currently known and currently open in the R-STDP hardware track. Future chapters will accrete as items in this page complete. If you finish something on the list, update this page (and the relevant page above) — the docs are the handoff, not the code.

Welcome to the project.

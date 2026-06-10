---
title: "4.10 Simulation: The Integration Testbench (Level 1)"
parent: "4 Implementing RSTDP"
nav_order: 10
---

# 4.10 Simulation: The Integration Testbench (Level 1)

This page is about [`hardware_code_rstdp/tb/step8_learning_integration_tb.sv`](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv), the "Level 1" integration testbench from `plan.txt` step 8. The TB instantiates `hbm_processor` + `internal_events_processor` + a coincFIFO model + behavioral URAM/HBM models, then drives 16 system-level R-STDP scenarios.

You'll work with this file when you need to:

- Reproduce a bug a downstream test surfaced.
- Add a regression test for a new feature.
- Sanity-check that your RTL changes didn't break learning.

The whole TB is ~2600 lines, but ~80% is boilerplate (memory models, AXI4 plumbing). What you actually need to know is the conventions: how to preload memory, how to drive a timestep, how to check what happened. That's what this page covers.

A separate "Level 2" host-style TB (driving everything through `command_interpreter`) is documented in [4.11](4_11_host_style_tb_level2). Use this one for fast iteration; use Level 2 to validate the command path before bitstream.

---

## How to run it

The build commands are also in MEMORY.md and at the bottom of [`CHANGES.md`](../../hardware_code_rstdp/CHANGES.md). For Vivado:

```bash
cd hardware_code_rstdp

xvlog -sv -d SIM \
    src/hbm_processor.v \
    src/internal_events_processor.v \
    tb/step8_learning_integration_tb.sv

xelab -timescale 1ns/1ps step8_learning_integration_tb -s step8_sim

xsim step8_sim --runall
```

For Icarus:

```bash
iverilog -g2012 -D SIM \
    src/hbm_processor.v \
    src/internal_events_processor.v \
    tb/step8_learning_integration_tb.sv \
    -o step8_sim

vvp step8_sim
```

**Two compile defines worth knowing about:**

| Define | What it enables |
|---|---|
| `+define+SIM` | RTL-side assertions (`hbm_count != 0` at Phase 2 start, FIX#1 address check, FIX#2 duplicate-push check). Leave on. |
| `+define+STEP8_DEBUG` | Extra `$display` probes in IEP: ASB write traces, coincidence-detect traces, coincFIFO push traces. Useful when a single test fails and you need to see what happened in time order. Off by default to keep logs short. |

Expected outcome with both defines on and current RTL:

```
[PASS] Test1: no coincFIFO pushes (post didn't spike)
...
[PASS] Test16: ...

  TESTBENCH COMPLETE
  Passed: N / N
  Failed: 0 / N
```

If anything fails, the failing `check()` line tells you the test number and what was wrong.

---

## What the TB is and isn't

The 16 tests cover (per MEMORY.md, with current numbering):

| Tests | Coverage |
|---|---|
| 1-8 | Basic flows: no spike, coincidence without reward, coincidence with reward, full learning sequence over multiple timesteps, backpressure, hbm_count alignment regression, membrane-only updates, reward-independent membrane updates |
| 9-11 | ET RMW basic / accumulation / saturation |
| 12 | Weight clamping (sum > 32767 → 32767) |
| 13 | reward=0 gate (ET still updates, weight stays) |
| 14 | End-to-end coincidence with 8 per-stage assertions |
| 15 | ET decay integration |
| 16 | Multiple coincFIFO entries in one timestep |

This is **not** a network-level TB. It uses behavioral URAM/HBM and a coincFIFO model, drives the IEP and `hbm_processor` directly, and checks observable signals. There's no Python compiler, no axon-event processor, no command interpreter. For that, see [4.11](4_11_host_style_tb_level2).

---

## Conventions you have to follow

The TB has a small set of conventions that, if you violate, will make your new test silently pass or silently hang. Each one earns its keep.

### 1. **`preload_synapse` + `preload_r4_pointer` + `preload_et_uram` at test start only**

Three preload tasks set up the memories before each test:

```verilog
// Synapse weight (R3 row)
preload_synapse(23'h002000, 16'h0001, 13'd0, 16'sd50);

// R4 pointer → URAM ET addr (must be called for every R3 row that has an ET)
preload_r4_pointer(23'h002000, 13'd2);

// Initial ET value at URAM full address 13'd2, group 0
preload_et_uram(0, 13'd2, 36'sd0);
```

Source at [`step8_learning_integration_tb.sv:881`](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv#L881).

The non-obvious rule: **never call these between timesteps in the same test.** Doing so was the "circular check" workaround the TB rewrite removed (`CHANGES.md` table row 2). Mid-test refresh masks DUT bugs because you're comparing your own writes against your own writes. Always preload once at start, then run multiple `run_execution()` calls to drive the DUT.

### 2. **Set `et_increment` before any run that triggers WUE**

`et_increment` is a TB-driven signal (the TB plays the role of CI). Like the rest of the per-test config, set it in your test's setup block:

```verilog
et_increment = 36'sd500;
exec_reward  = 1'b1;
et_uram_start_raddr = 13'd2;
et_uram_end_raddr   = 13'd2;
et_leak_shift       = 4'd0;   // disable decay if you want pure accumulation
```

If `et_increment = 0`, every ET RMW is a no-op and weights won't change. This is a common cause of "my test runs but checks zero == zero."

### 3. **Use the AXI scoreboard, not `hbm_mem_aa[]`, for write checks**

The TB monitors `hbm_awvalid && hbm_awready` and `hbm_wvalid && hbm_wready` and captures every DUT-issued write into:

```verilog
reg [22:0]  scb_last_wr_addr;     // word address (awaddr[27:5])
reg [255:0] scb_last_wr_data;
integer     scb_test_wr_count;    // resets per test
```

Then `check_wue_write()` ([line 954](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv#L954)) does a full check against the scoreboard:

```verilog
check_wue_write(
    "Test3",
    23'h002000,    // exp_r3_addr
    4'd0,          // exp_group_idx
    16'sd200,      // exp_old_weight
    36'sd500,      // exp_et_value
    16'sd700       // exp_new_weight = clamp(200 + 500)
);
```

It verifies the WUE AR addresses, the captured `dbg_wue_*` registers, the AXI write address (beat 0 for groups 8-15, beat 1 for groups 0-7), and the weight bits extracted from the captured beat. **Read this task before writing a new test that involves a write.** It's the canonical "did everything happen right" check.

### 4. **Use `check_et_rmw()` for ET RMW**

Sibling task at [line 1013](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv#L1013). Verifies:

- ET RMW fired at least once this test (`et_rmw_valid_count >= 1`).
- The captured URAM address matches what you expected.
- `dbg_et_old_value` and `dbg_et_new_value` match your expected values.
- `iep_et_valid` pulsed at least once.

Doesn't directly read URAM because decay may have modified it after the RMW. The captured debug values are taken at RMW commit time, so they're what you want for verification.

### 5. **Drive `exec_run` with NBA, not blocking assign**

The standard task at [line 1115](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv#L1115):

```verilog
task automatic trigger_exec_run();
    begin
        @(posedge clk);
        exec_run <= 1'b1;   // NBA: RTL sees exec_run=1 at next posedge
        @(posedge clk);
        exec_run <= 1'b0;   // NBA: RTL already read exec_run=1 this posedge
    end
endtask
```

Why NBA: blocking `exec_run = 1` then `exec_run = 0` causes the deassertion to happen in the active region *before* the RTL's `always @(posedge clk)` block reads it. In Xcelium, on the second exec_run of a multi-run test, this race coincides with IEP's PHASE2_DONE → IDLE transition and the RTL sees `exec_run=0` for both consecutive cycles → IEP stays IDLE forever. NBA fires *after* RTL active-region reads, so the RTL correctly sees `exec_run=1` for exactly one cycle. The MEMORY.md note "TB Bug Fixed (Step 8, run 2 timeout)" is this fix.

**Always use `<=` for control signals into the DUT. Always.**

### 6. **Wait with signal + timeout, not fixed delays**

`wait_phase1_done()` at [line 1138](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv#L1138) and `wait_full_completion()` at [line 1189](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv#L1189):

```verilog
fork
    begin : timeout
        repeat (5000) @(posedge clk);
        $fatal(1, "[TIMEOUT] ...");
    end
    begin
        wait (hbm_curr_state    == TX_IDLE &&
              hbm_rx_curr_state == RX_IDLE &&
              iep_curr_state    == IEP_IDLE);
        @(posedge clk);   // extra margin
        @(posedge clk);
        disable timeout;
    end
join
```

Two patterns to copy:

- **Fork + disable** so a hung wait fails loudly with `$fatal(1, ...)` instead of stalling the simulator.
- **Wait for `IEP_IDLE` too,** not just HBM IDLE. `exec_run` is *ignored* by IEP while it's still in PHASE2_DONE; firing the next `exec_run` too early leaves IEP missing the pulse and hanging the next run.

`repeat (N)` fixed-cycle delays are reserved for reset (where timing isn't load-bearing) and the per-cycle `@(posedge clk)` margin sprinkled after a wait. Anything else → signal-driven.

### 7. **Use exact counts, not `>= 1`, on coincFIFO checks**

```verilog
check("Test1: no HBM writes", scb_test_wr_count == 0);     // not >= 0
check("Test3: exactly 1 HBM write", scb_test_wr_count == 1); // not >= 1
```

The "removed workarounds" table in `CHANGES.md` lists `coincfifo_push_count >= 1` as the loosest of the original checks. With exact counts, any duplicate or spurious push from a future RTL regression fails immediately rather than silently passing.

### 8. **Reset scoreboard counters per test, not per run**

`reset_test_counters()` at [line 1085](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv#L1085) zeroes `scb_test_wr_count`, `wue_valid_count`, `wb_valid_count`, etc. Call this **at the start of each test task**, not before each `run_execution()`. That way a multi-timestep test sees cumulative counts and can assert things like "Test4-T3: exactly 1 HBM write" after three timesteps with no writes followed by one timestep with a write.

### 9. **Backpressure via `coincfifo_bp_enable` + `coincfifo_bp_delay`**

The coincFIFO model has a built-in backpressure injector at [line 507](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv#L507):

```verilog
reg     coincfifo_bp_enable;
integer coincfifo_bp_delay;     // cycles to stall before releasing data
```

Setting `coincfifo_bp_enable = 1` and `coincfifo_bp_delay = 3` before a run delays visibility of pushed data by 3 cycles. Test 5 uses this to verify that WUE handles FIFO stalls without firing the write multiple times. Pattern:

```verilog
coincfifo_bp_enable = 1'b1;
coincfifo_bp_delay  = 3;
run_execution();
coincfifo_bp_enable = 1'b0;
```

---

## Anatomy of a test task

Tests are written as `automatic` tasks. The pattern is the same across all 16:

```verilog
task automatic testN_<name>();
    begin
        $display("\n========== TEST N: <description> ==========");
        reset_test_counters();
        clear_uram();
        clear_phase1_pointers();
        apply_reset();
        init_inputs();

        // ----- Setup -----
        preload_synapse(23'h002000, 16'h0001, 13'd0, /*weight*/ 16'sd200);
        preload_r4_pointer(23'h002000, /*uram_full=*/ 13'd2);
        preload_et_uram(0, 13'd2, /*initial ET=*/ 36'sd0);

        et_increment = 36'sd500;
        exec_reward  = 1'b1;

        // ----- Drive -----
        run_execution();   // one timestep, returns after IDLE
        // ... more run_execution() calls if multi-timestep ...

        // ----- Check -----
        check("TestN: exactly 1 HBM write", scb_test_wr_count == 1);
        check_wue_write("TestN", 23'h002000, 4'd0, 16'sd200, 36'sd500, 16'sd700);
        check_et_rmw("TestN", 13'd2, 36'sd0, 36'sd500);
    end
endtask
```

Test tasks are called in order from the main `initial` block at the bottom of the file. `$finish` at the end prints the pass/fail summary.

---

## How to add a new test

A worked example: add a test for **"ET decay should reduce a positive ET by ~6% per timestep with `et_leak_shift=4`."**

### Step 1: pick a test number

The current file goes up to Test 16. Use Test 17. Numbering is local to this file.

### Step 2: write the task

Drop into the file before the final `initial` block:

```verilog
task automatic test17_et_decay_rate();
    begin
        $display("\n========== TEST 17: ET decay rate ==========");
        reset_test_counters();
        clear_uram();
        clear_phase1_pointers();
        apply_reset();
        init_inputs();

        // Preload a single ET cell with a known value. No synapse, no coincidence.
        // Only the decay sweep should touch it.
        preload_et_uram(0, 13'd2, 36'sd1024);

        et_increment        = 36'sd0;    // no RMW will fire
        et_uram_start_raddr = 13'd2;
        et_uram_end_raddr   = 13'd2;
        et_leak_shift       = 4'd4;       // expect 1024 → 1024 - 64 = 960
        exec_reward         = 1'b0;

        // No axons fire, no synapses preloaded → Phase 2 idle → straight to decay.
        run_execution();

        // After one timestep, ET should be 1024 - (1024 >> 4) = 1024 - 64 = 960.
        check($sformatf("Test17: ET=%0d after 1 decay step, exp=960",
                        $signed(uram_mem[0][13'd2/2][35:0])),
              uram_mem[0][13'd2/2][35:0] === 36'sd960);
    end
endtask
```

Three things to note:

- `uram_mem[0][1][35:0]` directly reads the behavioral URAM model. That's fine for verifying decay since decay is the *only* writer to that cell in this test. (Don't do this for weight checks where the AXI scoreboard exists.)
- `13'd2 / 2 = 1` selects URAM row 1 (the half-select bit goes into `[0]`, which equals 0 → lower half).
- This test doesn't need `check_wue_write` or `check_et_rmw`, since only decay should fire.

### Step 3: call it from the main block

Find the `initial begin ... end` block near the bottom of the file (~line 2540). Add a call:

```verilog
initial begin
    ...
    test16_multiple_coincfifo_entries();
    test17_et_decay_rate();     // ← new
    ...
    $display("  TESTBENCH COMPLETE");
    ...
end
```

### Step 4: recompile and run

```bash
xvlog -sv -d SIM \
    src/hbm_processor.v \
    src/internal_events_processor.v \
    tb/step8_learning_integration_tb.sv
xelab -timescale 1ns/1ps step8_learning_integration_tb -s step8_sim
xsim step8_sim --runall | tail -50
```

If the test passes, you'll see `[PASS] Test17: ET=960 after 1 decay step, exp=960`. If it fails, the failing check tells you whether the value was wrong or the count was off. From there, `+define+STEP8_DEBUG` and a VCD trace are your next steps.

---

## When to write a Level 1 test vs. a smaller TB

Smaller `_tb.sv` files exist in [`tb/`](../../hardware_code_rstdp/tb/) for individual steps:

- [`region4_addr_map_tb.sv`](../../hardware_code_rstdp/tb/region4_addr_map_tb.sv): step 2 mapping arithmetic.
- [`region3_addr_tracking_tb.sv`](../../hardware_code_rstdp/tb/region3_addr_tracking_tb.sv): step 3 BDFIFO + RX address reconstruction.
- [`coincidence_coincfifo_tb.sv`](../../hardware_code_rstdp/tb/coincidence_coincfifo_tb.sv): step 4 ASB + coincidence push.
- [`step5_weight_update_tb.sv`](../../hardware_code_rstdp/tb/step5_weight_update_tb.sv): step 5 WUE read+compute (no write).
- [`command_interpreter_reward_tb.sv`](../../hardware_code_rstdp/tb/command_interpreter_reward_tb.sv): step 1 reward register.
- [`hbm_wue_et_increment_tb.sv`](../../hardware_code_rstdp/tb/hbm_wue_et_increment_tb.sv): focused on ET increment plumbing.
- [`bugfix_regression_tb.sv`](../../hardware_code_rstdp/tb/bugfix_regression_tb.sv): the three FIX#1/2/3 regressions from `RTL_fixes_explained.txt`.

Use these when you're debugging a single piece (e.g., "did I break the BDFIFO?"). They compile in seconds and reproduce in tens of cycles. Use the Level 1 integration TB when you want to confirm the *whole learning loop* still works after your changes.

---

## What to take from this page

- Run with `+define+SIM`. Add `+define+STEP8_DEBUG` when debugging.
- Conventions to follow: preload once at start, use scoreboards not memory peeks, use `<=` for `exec_run`, use signal-driven waits with timeouts, use exact counts, reset counters per test.
- The canonical checks are `check_wue_write()` (for weights) and `check_et_rmw()` (for ETs). Copy their argument shape.
- Adding a test = `task automatic testN_*()` → call from the main `initial` block → recompile.

Next, [4.11](4_11_host_style_tb_level2) covers the Level 2 host-style TB that drives the same scenarios through the command interpreter, the way `hs_bridge` will.

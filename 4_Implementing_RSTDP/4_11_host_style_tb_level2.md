---
title: "4.11 Host-Style Testbench (Level 2)"
parent: "4 Implementing RSTDP"
nav_order: 11
---

# 4.11 Host-Style Testbench (Level 2): Driving Through Commands

The Level 1 TB from [4.10](4_10_integration_tb_level1) drives IEP and `hbm_processor` directly. That makes it fast to write and fast to debug, but it bypasses the same command interpreter that the host computer (running `hs_bridge`) will use on real hardware. Bugs in the command path (opcode skew, bit-position errors, packet timing) won't show up in Level 1.

The Level 2 TBs are the same R-STDP scenarios driven entirely through `command_interpreter.v`'s `rxFIFO`. Stimulus goes in as 512-bit packets carrying CI commands; results come out as URAM reads via `txFIFO`. The TB is structurally the same shape as the eventual host code, which makes it the last validation step before bitstream.

This is `plan.txt` step 8 Level 2. Three files do the work:

| File | Role |
|---|---|
| [`step8_level2_common.svh`](../../hardware_code_rstdp/tb/step8_level2_common.svh) | Shared command-builder tasks (`send_set_reward`, `send_hbm_write`, etc.) and packet helpers. Included by all Level 2 TBs. |
| [`step8_level2_host_tb.sv`](../../hardware_code_rstdp/tb/step8_level2_host_tb.sv) | Smoke TB. Layers in modules one at a time (CI alone → +EEP → +HBM proc → +PFC) to localize bugs to the newly-added module. |
| [`step8_level2_rstdp_tb.sv`](../../hardware_code_rstdp/tb/step8_level2_rstdp_tb.sv) | Full R-STDP TB. Replicates Level 1's scenarios through the CI path. |

And one Python file that produces the actual command stream the TB consumes:

| File | Role |
|---|---|
| [`generate_network_commands_rstdp.py`](../../hardware_code_rstdp/tb/generate_network_commands_rstdp.py) | Builds the init command list for a 2-axon / 2-layer-1-neuron / 1-layer-2-neuron R-STDP network. Output is `network_init_commands_rstdp.txt`, a list of 128-hex-char strings (one per 512-bit packet). |

Plus the 100-timestep regression:

| File | Role |
|---|---|
| [`step8_rstdp_100step_tb.sv`](../../hardware_code_rstdp/tb/step8_rstdp_100step_tb.sv) | Drives the same network for 100 timesteps and dumps a CSV ([prelimenary_rstdp/step8_rstdp_100step.csv](../../prelimenary_rstdp/)) of spike rasters, ET values, reward timing, and weights. The golden trace for host migration. |

---

## What changes between Level 1 and Level 2

Level 1 talks to `hbm_processor` and IEP through their port lists. Level 2 talks only to:

- `rxFIFO_dout` + `rxFIFO_empty` (write side: TB drives, CI reads)
- `txFIFO_din` + `txFIFO_wren` + `txFIFO_full` (read side: CI drives, TB reads)
- AXI HBM channels (a real `command_interpreter` is in the loop, but the TB still provides the HBM behavioral model)

Everything else is RTL-to-RTL. The TB instantiates:

```
                    rxFIFO_dout
                         │ (TB drives)
                         ▼
                ┌─────────────────────┐
                │ command_interpreter │
                └───┬──────┬──────┬───┘
                    │      │      │  ci2hbm / ci2iep / axon events
                    ▼      ▼      ▼
       ┌─────────┐  ┌──────────┐  ┌─────┐
       │ hbm_    │  │ internal_│  │ EEP │
       │processor│  │ events_  │  └──┬──┘
       └────┬────┘  │ processor│     │
            │       └──────────┘     │
            ▼            ▲           ▼
       AXI HBM    coincFIFO       BRAM × 2
       (TB model)  PFC          (TB models)
                  (real)
                              ▼
                          txFIFO_din
                              │ (TB reads)
                              ▼
                          spike output
```

The TB still provides behavioral memories (HBM, BRAM, URAM) and an HBM AXI4 slave model, but it does not poke into RTL internals.

---

## The packet pattern: `send_pcie_packet`

Every Level 2 stimulus goes through one task. From [`step8_level2_common.svh:26`](../../hardware_code_rstdp/tb/step8_level2_common.svh#L26):

```verilog
task automatic send_pcie_packet(input [511:0] packet);
    begin
        rxFIFO_empty = 1'b1;
        rxFIFO_dout  = 512'd0;
        @(posedge aclk);
        @(posedge aclk);

        @(posedge aclk);
        rxFIFO_dout  = packet;
        @(posedge aclk);
        rxFIFO_empty = 1'b0;     // ← CI sees data
        @(posedge aclk);
        @(posedge aclk);
        @(posedge aclk);
        rxFIFO_empty = 1'b1;     // ← drained
        @(posedge aclk);
        @(posedge aclk);
        @(posedge aclk);
    end
endtask
```

The 6-cycle "settle, deassert empty, hold, reassert" pattern mimics how the host's PCIe DMA delivers packets, since CI samples `rxFIFO_empty` and `rxFIFO_dout` like it would in hardware. The fixed delays here are deliberate: they reflect realistic packet timing rather than chasing a particular FSM state.

All command builders wrap `send_pcie_packet`. To set reward:

```verilog
task automatic send_set_reward(input bit reward);
    reg [511:0] pkt;
    begin
        pkt = {CMD_SET_REWARD, 503'd0, reward};
        send_pcie_packet(pkt);
    end
endtask
```

To write a synapse:

```verilog
send_hbm_write(/*addr*/ 23'h002000,
               /*data*/ build_synapse_packet(16'h0001, 13'd0, 16'sd200));
```

To run one timestep:

```verilog
send_axon_events_16(/*mask*/ 16'h0001);
send_exec_step();
```

The full list of builders in `step8_level2_common.svh`:

| Builder | Drives |
|---|---|
| `send_set_network_params(n_in, n_out, threshold, model)` | `CMD_NTWK_PARAM_W` (0x04) |
| `send_neuron_type(threshold, model, ...)` | `CMD_NEURON_TYPE` (0x08) |
| `send_hbm_write(addr, data)` | `CMD_HBM_RW` (0x02) write |
| `send_hbm_read(addr)` | `CMD_HBM_RW` (0x02) read |
| `send_axon_events_16(mask)` | `CMD_EEP_W` (0x01), two-packet sequence: opcode then mask |
| `send_exec_step()` | `CMD_EXEC_STEP` (0x06) |
| `send_set_reward(bit)` | `CMD_SET_REWARD` (0x0A) |
| `send_set_et_params(et_increment, et_leak_shift)` | `CMD_SET_ET_PARAMS` (0x0B) |
| `send_set_et_range(start_raddr, end_raddr)` | `CMD_SET_ET_RANGE` (0x0C) |
| `send_bogus_command(opcode)` | Anything (for negative tests) |

Helper functions `make_pointer`, `make_synapse_slot`, `build_synapse_packet`, and `extract_weight_from_beat` build/parse data payloads with the same shapes as in Level 1.

---

## The smoke TB: layered isolation

`step8_level2_host_tb.sv` runs 6 tests (L2-T1 through L2-T6), each adding one more module to the loop:

| Test | Modules in loop | What it proves |
|---|---|---|
| L2-T1 | CI only | CI decodes config commands (CMD_NTWK_PARAM_W, CMD_SET_REWARD, CMD_SET_ET_PARAMS, CMD_SET_ET_RANGE). All output ports change. |
| L2-T2 | CI + EEP + 2 BRAMs | CMD_EEP_W writes the axon mask into BRAM correctly. |
| L2-T3 | + TB-driven `exec_iep_phase2_done` | CMD_EXEC_STEP transitions CI through WAIT_RUN and EXEC_DONE. |
| L2-T4 | + hbm_processor + PFC | Phase 1a, axon-driven HBM input-pointer reads land in PFC. |
| L2-T5 | (still no IEP) | Phase 1b, neuron-driven HBM output-pointer reads land in PFC. |
| L2-T6 | (both Phase 1 paths simultaneously) | Concurrent Phase 1a/1b dispatch through shared per-axon ptr-FIFOs. |

The smoke TB **does not include IEP**. That means Phase 2 won't drain. The TB stubs `exec_iep_phase2_done` directly so CI's WAIT_RUN state advances. The tests wait for `exec_hbm_rx_phase1_done` instead of a full timestep.

This is a useful pattern when you're adding a new CI command or a new path through `hbm_processor`: write a smoke test first, validate without the IEP complexity, then add to the full R-STDP TB once the smoke test passes.

---

## The full R-STDP TB

`step8_level2_rstdp_tb.sv` instantiates the full pipeline: CI + EEP + 2 BRAMs + `hbm_processor` + IEP + PFC + coincFIFO + 16 URAM models + behavioral HBM. Tests L2-T5 through L2-T8 cover:

- **L2-T5**: Phase 2 with no spike: membrane update + ET decay only.
- **L2-T6**: Coincidence push with `reward=0`: coincFIFO entry, no AXI write.
- **L2-T7**: Full R-STDP with `reward=1`: weight changes; verified via `CMD_HBM_RW` read-back.
- **L2-T8**: Multi-timestep accumulation including `spike_fifo_controller` so the TB can read spike output via `txFIFO`.

The read-back pattern in L2-T7 is the closest the TB gets to mimicking `hs_api`:

```verilog
// Run learning sequence
...
run_timestep_with_axons(16'h0001);
...

// Verify weight changed
send_hbm_read(/*addr*/ 23'h002000);
wait_for_txfifo();
check("L2-T7: post-learn weight read", txFIFO_dout == expected_weight_packet);
```

A real `hs_api.read_synapse()` call (from [4.2](4_2_synapse_read_write)) reduces to the same shape: send `CMD_HBM_RW` read packet, wait for the response on `txFIFO`, decode.

---

## Generating the command stream from Python

For more than a handful of synapses, hand-coding the init packets gets tedious. [`generate_network_commands_rstdp.py`](../../hardware_code_rstdp/tb/generate_network_commands_rstdp.py) builds the full init sequence from a high-level connectome description.

### What the script does

The reference network it builds:

```
   axon_0 ──[w=0.75]──► neuron_0 ──[w=0.75]──┐
                                              ├──► neuron_2
   axon_1 ──[w=0.75]──► neuron_1 ──[w=0.75]──┘

   All neurons: LIF, threshold = 2^19, leak = 63
```

The script uses the existing `hs_bridge` compiler (with the `simDump=True` flag, no real PCIe DMA) to produce the standard `CMD_HBM_RW` writes for axon pointers, neuron pointers, and synapse rows. Then it **extends** that command list with:

1. **Region 4 writes**: one `CMD_HBM_RW` per synapse row, payload = list of 13-bit URAM ET addresses (one per slot). Addresses are assigned sequentially starting at `N_padded` (immediately above the URAM neuron range).
2. **URAM ET clear writes**: one `CMD_IEP_RW` per `(group, row)` that holds an ET, zeroing the cell.

Output is `network_init_commands_rstdp.txt`, with one 128-hex-char string per line, each a 512-bit `rxFIFO` packet.

### Why this matters for the host migration

The script is run **offline**. Its output is the dataset the TB's `initial` block reads and replays via `send_pcie_packet`. In production, the same logic (Region 4 emission + ET URAM initialization) needs to happen inside `hs_bridge`'s real network compiler, [`compile_network.py`](../../hs_bridge/), so the host can program the FPGA the same way. That work is open and called out in [4.12](4_12_next_steps).

### Reading the output

The script also pretty-prints the decoded command stream, useful for sanity-checking what the TB is about to do:

```
--- Command 12: CMD_HBM_RW ---
  Hex: 02...
  WRITE HBM addr=2098176 (0x200400)
  -> Region 4 ET pointer LUT (Region3 addr=0x0F8400)
    slot[0]: URAM addr=16 (group=0, row=1)
    slot[1]: URAM addr=17 (group=1, row=1)

--- Command 18: CMD_IEP_RW ---
  WRITE URAM group=0 row=1 data=0  [ET clear, URAM addr=16]
```

The Region 4 row at `0x200400` says "Region 3 row `0x0F8400` has ET pointers in slots 0 and 1, addressed to URAM `{group 0, row 1}` and `{group 1, row 1}`." That matches what we'd expect from the address arithmetic in [4.4](4_4_reward_and_address_mapping).

---

## The 100-step regression

[`step8_rstdp_100step_tb.sv`](../../hardware_code_rstdp/tb/step8_rstdp_100step_tb.sv) takes the same 2-axon network and drives it for 100 timesteps with a Poisson axon input and a programmed reward pattern (specific steps set `reward=1`, others 0). The TB emits a CSV with one row per timestep tracking:

- Per-axon spike state
- Per-neuron spike state
- The learning synapse's ET
- The learning synapse's current weight in HBM
- `exec_reward`

Output goes to [`prelimenary_rstdp/step8_rstdp_100step.csv`](../../prelimenary_rstdp/). The plot at `step8_rstdp_100step.png` is generated from that CSV.

This is the **golden trace** for host migration. When `hs_api`/`hs_bridge` is taught to drive R-STDP on real hardware, run the same Poisson seed, same axon pattern, same reward schedule, dump a CSV from the host code, and diff against the sim CSV. If they match bit-exactly (modulo PCIe readback timing), the migration is correct.

The script handling this is `step8_rstdp_100step_axons.vh`, a generated header with the per-timestep axon mask sequence that is included by the 100-step TB.

---

## When to use Level 2 vs Level 1

| If you're … | Use … |
|---|---|
| Adding RTL inside `hbm_processor` or IEP | Level 1 ([4.10](4_10_integration_tb_level1)). Faster iteration. |
| Adding a new CI command or changing packet layout | Level 2 smoke TB ([`step8_level2_host_tb.sv`](../../hardware_code_rstdp/tb/step8_level2_host_tb.sv)). |
| Adding a new module to the pipeline | Level 2 smoke TB; add one layer at a time. |
| Changing the network compiler or `hs_bridge` driver | Level 2 R-STDP TB + regenerate command stream from Python. |
| Validating before bitstream | Both. Level 1 catches RTL regressions; Level 2 catches command-path regressions. |
| Producing reference traces for hardware comparison | The 100-step TB. |

---

## How to add a Level 2 test

Same pattern as Level 1, but inside `step8_level2_rstdp_tb.sv` and using the builder tasks:

```verilog
task automatic test_my_scenario();
    begin
        $display("\n========== L2-T9: my scenario ==========");
        apply_reset();

        // ----- Config via commands -----
        send_set_network_params(17'd2, 17'd3, 36'sd524288, 2'd2);
        send_neuron_type(36'sd524288, 2'd2, 6'd0, 6'd63);
        send_set_reward(1'b1);
        send_set_et_params(36'sd500, 4'd0);
        send_set_et_range(13'd2, 13'd4);

        // ----- Init memories via commands -----
        send_hbm_write(23'h002000, build_synapse_packet(16'h0001, 13'd0, 16'sd200));
        send_hbm_write(23'h012000, /*R4 with URAM ptr 13'd2*/ 256'h0...02);
        // (or use the Python script to generate these)

        // ----- Drive a timestep -----
        send_axon_events_16(16'h0001);
        send_exec_step();
        wait_for_exec_done();

        // ----- Read back and check -----
        send_hbm_read(23'h002000);
        wait_for_txfifo();
        check("L2-T9: weight updated",
              extract_weight_from_beat(txFIFO_dout[255:0], 4'd0) == 16'sd700);
    end
endtask
```

`wait_for_exec_done()` watches the CI exec-done bit (see the existing tests for the exact signal). `wait_for_txfifo()` waits for `txFIFO_wren`. Both are utility tasks already in the TB.

---

## What to take from this page

- Level 2 drives everything through `command_interpreter.v`'s `rxFIFO`. Same R-STDP scenarios as Level 1, but verified end-to-end through the command path.
- All stimulus is built from a small set of `send_*` tasks in [`step8_level2_common.svh`](../../hardware_code_rstdp/tb/step8_level2_common.svh).
- The smoke TB ([`step8_level2_host_tb.sv`](../../hardware_code_rstdp/tb/step8_level2_host_tb.sv)) layers in modules one at a time; use it to localize CI-command-path bugs.
- The full R-STDP TB ([`step8_level2_rstdp_tb.sv`](../../hardware_code_rstdp/tb/step8_level2_rstdp_tb.sv)) covers the full learning loop with read-back via `CMD_HBM_RW`.
- For non-trivial networks, generate the command stream with [`generate_network_commands_rstdp.py`](../../hardware_code_rstdp/tb/generate_network_commands_rstdp.py) and replay it.
- The 100-step TB produces the golden CSV that host migration code must match.

Next, [4.12](4_12_next_steps) covers the open work: what needs to happen to fix the ET rule, move from sim to bitstream, integrate with `hs_api`, integrate with `simpleSim`, and run an RL task.

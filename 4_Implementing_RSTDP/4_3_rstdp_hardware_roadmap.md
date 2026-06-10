---
title: "4.3 R-STDP Hardware Roadmap"
parent: "4 Implementing RSTDP"
nav_order: 3
---

# 4.3 R-STDP Hardware Roadmap

[4.1](4_1_what_is_rstdp) gave the biology and math. [4.2](4_2_synapse_read_write) showed how the host already reads and writes weights through `hs_api`. The rest of Chapter 4 documents the **hardware** that makes R-STDP run on the FPGA without host involvement on the hot path.

This page is the map. Read it once, then dip into 4.4-4.12 in order. Every page after this one assumes you've finished Chapters 0-3, [4.1](4_1_what_is_rstdp), and [4.2](4_2_synapse_read_write).

The source under discussion lives in [`hardware_code_rstdp/`](https://github.com/dianavins/dianavins.github.io). The original (pre-R-STDP) modules covered in [Chapter 3](../chapter_3) are still the foundation; the R-STDP work *extends* `hbm_processor.v` and `internal_events_processor.v` rather than replacing them.

---

## What R-STDP needs from the hardware

To run R-STDP on the FPGA, the hardware has to do three things the original processor never did:

1. **Detect coincidences in hardware.** For every neuron that spikes this timestep, find every pre-synaptic axon/neuron that fired into it last timestep. Software can't help, because the host doesn't see per-timestep state.
2. **Maintain an eligibility trace per synapse.** A small signed value that accumulates on coincidence and decays each timestep, independent of the weight.
3. **Update weights from the eligibility trace, gated by a reward signal.** Only when reward is high do eligible synapses actually change weight.

All three happen inside one timestep, after the existing Phase 1 / Phase 2 work that Chapters 2.1-2.3 describe.

---

## The new dataflow

```
                  ┌────────────────────────────────────────────────────────┐
                  │              Existing Phase 1 / Phase 2                 │
                  │   (Chapter 2, unchanged)                                 │
                  └────────────────────────────────────────────────────────┘
                                            │
                                            ▼
   ┌──────────────────────────────────┐    During Phase 2, every synapse
   │  Active Synapse Buffer (ASB)     │◄── packet's R3 address and 16
   │  in IEP, 64×2 entries × 16 grp   │    target URAM addrs get logged.
   └─────────────┬────────────────────┘
                 │ swap on exec_run
                 ▼
   ┌──────────────────────────────────┐    Phase 1 of next timestep:
   │  Phase 1 coincidence comparator  │    for each address scanned,
   │  phase1_coincidence[15:0]        │    check if any prior entry
   │  = coinc_spiked & coinc_detected │    targeted it AND if the
   └─────────────┬────────────────────┘    neuron just crossed threshold.
                 │
                 ▼
   ┌──────────────────────────────────┐    {do_wu, group_mask[16], r3_addr[23]}
   │  coincFIFO                       │    One entry per coincident group.
   │  40 bits wide                    │
   └─────────────┬────────────────────┘
                 │
                 ▼
   ┌──────────────────────────────────┐
   │  Weight Update Engine (WUE)      │    In hbm_processor. New TX/RX
   │  in hbm_processor                │    sub-FSMs after Phase 2 ends.
   └──┬───────────────────────────────┘
      │   ┌────────────────────────────────────────────────┐
      │   │  R4 read: HBM[r3_addr + REGION4_OFFSET]        │
      │   │  → 13-bit URAM pointer (NOT an ET value)       │
      │   └────────────────────────────────────────────────┘
      │           │
      │           ▼
      │   ┌────────────────────────────────────────────────┐
      │   │  ET RMW handshake to IEP                       │
      │   │  wue_et_uram_addr, wue_et_addr_valid →         │
      │   │  ← iep_et_value, iep_et_valid                  │
      │   └────────────────────────────────────────────────┘
      │           │
      │           ▼
      │   ┌────────────────────────────────────────────────┐
      │   │  R3 read: HBM[r3_addr], 2 beats (512 bits)     │
      │   │  → extract 16-bit old weight for group G       │
      │   └────────────────────────────────────────────────┘
      │           │
      │           ▼
      │   ┌────────────────────────────────────────────────┐
      │   │  new_w = clamp(old_w + iep_et_value)           │
      │   │  R3 write-back, gated by exec_reward           │
      │   └────────────────────────────────────────────────┘
      ▼
   ┌──────────────────────────────────┐    All ET URAM addresses in
   │  ET decay sweep in IEP           │    [start_raddr .. end_raddr]:
   │  value -= value >>> et_leak_shift│    one pass per timestep,
   └──────────────────────────────────┘    after WUE finishes.
```

The five new blocks above (ASB, coincidence comparator, coincFIFO, WUE, ET RMW + decay) are the entire R-STDP addition. Everything else in Chapter 2 still happens exactly as documented.

---

## New modules and FSMs in one sentence each

| Name | Lives in | What it does |
|---|---|---|
| **Reward register `exec_reward`** | `command_interpreter.v` → `hbm_processor.v` | One-bit latch set by `CMD_SET_REWARD`; gates whether WUE actually writes the new weight back. |
| **Burst-descriptor FIFO (BDFIFO)** | `hbm_processor.v` | 128-deep queue that lets the RX side reconstruct the R3 address of each 512-bit synapse packet as it arrives. |
| **Region 4 offset** | `hbm_processor.v` | `r4_addr = r3_addr + REGION4_OFFSET`. R4 now stores a 13-bit URAM pointer to the synapse's ET word. |
| **Active Synapse Buffer (ASB)** | `internal_events_processor.v` | Double-buffered 64×2 record of "which synapse packets arrived in Phase 2, and which neurons they targeted." |
| **Phase 1 coincidence comparator** | `internal_events_processor.v` | Pure combinational AND of "post-synaptic spiked now" and "pre-synaptic was in ASB(prev)." |
| **coincFIFO** | top-level wiring; produced by IEP, consumed by hbm_processor | 40-bit-wide queue carrying one work item per coincident `(group, R3 address)` pair. |
| **Weight Update Engine (WUE)** | `hbm_processor.v` | New TX states 12-15 and RX states 10-14. Pops coincFIFO, reads R4 + R3, computes new weight, writes R3 back. |
| **ET RMW FSM** | `internal_events_processor.v` | Separate 5-state FSM (IDLE → READ → WAIT → WRITE → DONE) that increments one ET URAM cell on request from WUE. |
| **ET decay sweep** | `internal_events_processor.v` | Two main-FSM states (ET_DECAY_READ/WRITE) that sweep `[et_uram_start_raddr .. et_uram_end_raddr]` once per timestep. |

If a signal name in 4.4-4.12 looks foreign, it's probably one of the boundary wires between these blocks: `coincfifo_din`, `wue_et_uram_addr`, `iep_et_value`, `et_increment`, `exec_reward`. Each gets defined where it first matters.

---

## Reading order

| Page | Concept | What you'll be able to do after reading |
|---|---|---|
| [4.4](4_4_reward_and_address_mapping) | Reward register + R3/R4 mapping + URAM ET layout | Trace any synapse address through to its ET cell. |
| [4.5](4_5_r3_address_tracking) | Burst-descriptor FIFO inside `hbm_processor` | Explain why every Phase 2 packet has a known R3 address by the time IEP sees it. |
| [4.6](4_6_coincidence_detection) | ASB + phase1_coincidence + coincFIFO push | Predict, for any toy network, exactly which coincFIFO entries get pushed in a given timestep. |
| [4.7](4_7_weight_update_engine) | WUE TX/RX states + reward gating | Walk through one WUE entry start-to-finish on a waveform. |
| [4.8](4_8_eligibility_trace) | ET RMW FSM + decay sweep + the "ET rule is not finalized" caveat | Write a TB that drives the ET RMW path and checks accumulation/saturation/decay. |
| [4.9](4_9_full_timestep_walkthrough) | The complete R-STDP timestep, all pieces stitched | Read a full-timestep waveform and identify which sub-FSM owns each interval. |
| [4.10](4_10_integration_tb_level1) | TB conventions for `step8_learning_integration_tb.sv` | Add a new test to the integration TB without rereading 2600 lines. |
| [4.11](4_11_host_style_tb_level2) | Host-style TB + Python command generator | Drive the FPGA from a command stream the way `hs_bridge` will, end-to-end. |
| [4.12](4_12_next_steps) | What's open: ET rule fix, host TBs, bitstream, hs_api, simpleSim, RL | Pick a starting project and find every doc that mentions it. |

Each page targets 15-25 minutes of reading. 4.6, 4.7, and 4.9 are the longest; budget more time for those.

---

## Where this maps onto `plan.txt`

`hardware_code_rstdp/plan.txt` is the 8-step bring-up plan Diana used to build the R-STDP additions. All eight steps are **complete in the RTL** but unevenly documented. This chapter covers them as follows:

| `plan.txt` step | Covered in |
|---|---|
| 0. Lock baseline | (not a doc topic; it's the baseline behind Chapters 0-3) |
| 1. Reward register | [4.4](4_4_reward_and_address_mapping) |
| 2. Region 4 mapping | [4.4](4_4_reward_and_address_mapping) |
| 3. R3 address tracking inside `hbm_processor` | [4.5](4_5_r3_address_tracking) |
| 4. Coincidence detection + coincFIFO | [4.6](4_6_coincidence_detection) |
| 5. WUE read-compute (no write) | [4.7](4_7_weight_update_engine) |
| 6. WUE write-back enabled | [4.7](4_7_weight_update_engine) |
| 7. Reward gate on weight write | [4.7](4_7_weight_update_engine) |
| 8 (Level 1). Integration TB | [4.10](4_10_integration_tb_level1) |
| 8 (Level 2). Host-style TB | [4.11](4_11_host_style_tb_level2) |
| "Next steps" (ET rule, host TBs, bitstream, hs_api, simpleSim, RL) | [4.12](4_12_next_steps) |

The eligibility trace machinery (URAM-resident ETs, RMW FSM, decay) doesn't have its own step number. It's woven through steps 4-7, so it gets its own page: [4.8](4_8_eligibility_trace).

---

## Where the design notes live in the repo

Several short text files in `hardware_code_rstdp/` aren't compiled into anything but capture decisions you'll want to know about:

- **`plan.txt`**: the 8-step bring-up plan.
- **`CHANGES.md`**: step 8 workaround removal and the `hbm_count` RTL fix.
- **`RTL_fixes_explained.txt`**: three bug fixes plus the secondary `asb_r3_extract` ordering fix, with reproducible test setups.
- **`IEP_coincidence_logic_explained.txt`**: line-by-line walkthrough of the ASB + coincidence logic (the source material for [4.6](4_6_coincidence_detection)).
- **`et_location_bug_fix_plan.txt`**: the polished plan for moving ETs from HBM R4 into URAM. Source material for [4.7](4_7_weight_update_engine) and [4.8](4_8_eligibility_trace).
- **`hs_bridge_and_api_plan.txt`**: what host migration still needs (opcodes, compiler changes, telemetry, version pinning). Source material for [4.12](4_12_next_steps).

Each page links to whichever of these is relevant.

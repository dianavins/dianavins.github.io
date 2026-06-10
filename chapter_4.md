---
title: 4 Implementing RSTDP
has_children: true
nav_order: 6
---

# 4 Implementing RSTDP

Reward-modulated Spike-Timing-Dependent Plasticity (R-STDP) is the learning rule this hardware was extended to support. It adds biologically inspired synaptic plasticity gated by a delayed reward signal, providing the substrate for reinforcement learning on a spiking neural network.

Chapter 4 is split into three arcs:

- **4.1-4.2** cover the *concept* and the existing host-side machinery (`hs_api` synapse R/W).
- **4.3-4.9** describe the *new hardware* that runs R-STDP without host involvement on the hot path. This is where most of Diana's recent work lives.
- **4.10-4.12** cover *verification and next steps*, including how to test changes in simulation, how to drive the system through commands, and where new students should pick up.

If you're new to the project, read [4.3](4_Implementing_RSTDP/4_3_rstdp_hardware_roadmap) first. It is the navigation hub for the hardware arc and tells you what assumptions each page makes.

---

## Table of Contents

| # | Page | Topic |
|---|---|---|
| 4.1 | [What is R-STDP?](4_Implementing_RSTDP/4_1_what_is_rstdp) | The biology and math: STDP, eligibility traces, reward modulation, and why R-STDP solves the delayed reward problem. |
| 4.2 | [Reading and Writing Synapses via hs_api](4_Implementing_RSTDP/4_2_synapse_read_write) | Host-side `read_synapse()` and `write_synapse()` from Python down to the FPGA. |
| 4.3 | [R-STDP Hardware Roadmap](4_Implementing_RSTDP/4_3_rstdp_hardware_roadmap) | Landing page for the hardware arc. New dataflow diagram, list of new FSMs and signals, reading order, mapping onto `plan.txt`. |
| 4.4 | [Reward Register & Address Mapping](4_Implementing_RSTDP/4_4_reward_and_address_mapping) | `CMD_SET_REWARD`, `exec_reward`, R3/R4 address relationship, URAM ET layout. |
| 4.5 | [Tracking R3 Addresses Through HBM Bursts](4_Implementing_RSTDP/4_5_r3_address_tracking) | The burst-descriptor FIFO inside `hbm_processor.v`, which tags each 512-bit synapse packet with its R3 address. |
| 4.6 | [Coincidence Detection: The Active Synapse Buffer](4_Implementing_RSTDP/4_6_coincidence_detection) | The ASB, parallel comparator, `coinc_spiked` ungated path, coincFIFO push logic, mem-only push path. |
| 4.7 | [The Weight Update Engine](4_Implementing_RSTDP/4_7_weight_update_engine) | WUE TX/RX states, R4 + R3 reads, ET handshake with IEP, sum + clamp, R3 write-back, reward gate. |
| 4.8 | [Eligibility Trace Read-Modify-Write and Decay](4_Implementing_RSTDP/4_8_eligibility_trace) | IEP-side ET RMW FSM, saturating add, per-group steering, decay sweep, CI configuration commands, the "ET rule is a placeholder" caveat. |
| 4.9 | [Putting It Together: One R-STDP Timestep](4_Implementing_RSTDP/4_9_full_timestep_walkthrough) | All five new pieces stitched together on a toy network, cycle by cycle, T and T+1. |
| 4.10 | [Simulation: The Integration Testbench (Level 1)](4_Implementing_RSTDP/4_10_integration_tb_level1) | Conventions for `step8_learning_integration_tb.sv`, scoreboard pattern, signal-driven waits, how to add a new test. |
| 4.11 | [Host-Style Testbench (Level 2)](4_Implementing_RSTDP/4_11_host_style_tb_level2) | Driving everything through `command_interpreter`'s `rxFIFO`, the Python command generator, the 100-step golden trace. |
| 4.12 | [Where the Work Picks Up](4_Implementing_RSTDP/4_12_next_steps) | Open work: ET rule fix, bitstream, `hs_api`/`hs_bridge` migration, `simpleSim`, RL task. How to choose a starting project. Index of in-repo design notes. |

---

## What this chapter assumes you know

- The two-phase execution model from [Chapter 2](chapter_2): Phase 0 (input), Phase 1 (pointer collection), Phase 2 (synapse accumulation).
- The HBM memory layout from [Chapter 1](chapter_1): Region 1 axon pointers, Region 2 neuron pointers, Region 3 synapse data.
- The roles of `hbm_processor.v` and `internal_events_processor.v` from [Chapter 3](chapter_3), at the level of "what reads from where, what writes to where."
- AXI4 read/write handshake basics from the [appendix](supplementary_information/appendix).
- The host-side `read_synapse()` / `write_synapse()` API from [4.2](4_Implementing_RSTDP/4_2_synapse_read_write).

[Chapters 0-3](index) cover all of that. The R-STDP hardware pages don't redefine these. They extend them.

---
title: "How to Use Claude Code for RTL Development"
parent: Supplementary Information
nav_order: 5
---

# How to Use Claude Code for RTL Development

This page describes the workflow that has worked best on this project for developing RTL with [Claude Code](https://claude.com/claude-code). The project is a Verilog/SystemVerilog implementation of an R-STDP spiking neural network processor targeting a Xilinx UltraScale+ FPGA, with a host-side C++ API and a large simulation testbench surface. RTL is unusually unforgiving — a single off-by-one in a state machine can stall a pipeline forever, and a missed reset can corrupt data on the *next* run instead of the current one. The patterns below are oriented around that reality.

Two roles for Claude Code dominate the work here:

1. **Building new ideas** — adding a new RTL feature, a new testbench, or a new pipeline stage from a rough specification.
2. **Debugging** — chasing down a failing test, a timing race, or a regression introduced by an earlier change.

The workflows for these two roles are different. Mixing them up — debugging like you are designing, or designing like you are debugging — wastes time and produces worse results.

---

## Section 1: Building New Ideas

### 1.1 Brainstorm before you implement

If you do not yet fully understand how to approach a problem — say, you know you need a read-modify-write FSM around URAM but you are not sure how to handle the 3-cycle latency hazard — **brainstorm with Claude Code before asking it to write code**. A productive brainstorming prompt names the constraint, the desired behavior, and asks for *options* rather than an answer:

> "I need to do a read-modify-write on URAM (1-cycle latency at 450 MHz) where the same address might be re-read 2 cycles later. What are 2–3 standard hardware patterns for handling this, and what are the tradeoffs of each?"

Asking for *multiple* options is important. A single answer feels authoritative; three answers force the model to compare them, which surfaces the real tradeoffs. Once you have understood the options, you can make the architectural choice yourself and then prompt for an implementation of *that specific choice* — which is a much smaller and more verifiable request than "design the RMW logic."

A good rule of thumb: **the prompt that produces good RTL is one you could hand to a junior engineer.** If the prompt is too vague for a person to act on without guessing, it is too vague for Claude Code as well.

### 1.2 Start with a written spec, not a prompt

The single most valuable thing you can do before invoking Claude Code on new RTL is **write down the spec yourself**, in a `.txt` or `.md` file inside the relevant directory (for example, [hardware_code_rstdp/plan.txt](../../hardware_code_rstdp/plan.txt) or [hs_bridge_and_api_plan.txt](../../hardware_code_rstdp/hs_bridge_and_api_plan.txt) in this repo). The spec does not need to be polished. It needs to contain:

- **What signal lives where.** Port lists, widths, valid/ready conventions. Hardware is wire-by-wire; ambiguity here becomes a bug later.
- **What state machine transitions exist.** Even a rough `IDLE → READ → WAIT → WRITE → DONE` sketch is enough to anchor the implementation.
- **What invariants must hold.** "`hbm_count` must reset to 0 at the start of every `exec_run`" is the kind of constraint that is invisible from code alone but determines correctness.
- **What you are explicitly *not* doing in this change.** Scope creep in RTL produces hard-to-localize regressions.

Once that file exists, point Claude Code at it. A prompt like *"implement the ET RMW FSM described in plan.txt, lines 40–120, in [internal_events_processor.v](../../hardware_code_rstdp/src/internal_events_processor.v)"* is dramatically more reliable than a freeform request, because both you and the model are now working from the same artifact.

### 1.3 Use Plan mode for architectural choices — and read every line of the plan

For changes that touch more than one module — for example, adding a new HBM region, or threading a new signal from the command interpreter through to the IEP — use Plan mode (Shift+Tab twice in the CLI). Plan mode forces Claude to lay out the full set of file edits before making any.

**Do not approve a plan without reading it line by line.** The plan is the cheapest place to catch a mistake. Once code is written, a wrong design decision is buried inside hundreds of lines of edits, and undoing it means re-prompting, re-compiling, and often re-debugging. Specifically, hand-check the plan for:

- Signals that should be added to the testbench top-level so they appear in the VCD.
- Reset conditions that need to be widened to cover the new state.
- New ports that need to be plumbed through every instantiation, not just the leaf module.
- Steps that look like cleanup or refactoring but were not in your spec — those are scope creep and should be removed.
- Steps that *assume* something you did not say (e.g., "I'll also add backwards compatibility for the old format"). If you did not ask for it, push back.

If anything in the plan is unclear, **ask Claude to justify it before you approve**:

> "Why are you adding a `valid` register on the `wue_et_uram_addr` output? Walk me through the cycle-by-cycle handshake you are assuming."

If the model cannot give a logical, mechanism-level reason — not "to be safe," not "best practice," but a concrete cycle-by-cycle account — the step is probably wrong, or at least premature. Vague justification is the most reliable warning sign for a flawed design step. Trust earned through specific, mechanistic reasoning; do not extend it on faith.

### 1.4 Write the testbench first, or at least alongside

Spiking neural network hardware has very little visible output unless you instrument for it. A common failure mode is to write 200 lines of new RTL, run a simulation, see "no spikes detected," and have no idea which of the 12 new wires is wrong.

When asking Claude Code to add a feature, ask it to add a **directed testbench task** in the same response — even if the task is just a wrapper around the existing `preload_synapse()` / `trigger_exec_run()` / `check_wue_write()` pattern. The testbench is what converts a vague "the simulation hangs" into a localized "Phase 1 completed, Phase 2 never started, IEP is stuck in `PHASE2_DONE`."

The [step8_learning_integration_tb.sv](../../hardware_code_rstdp/tb/step8_learning_integration_tb.sv) file in this repo is the reference shape for these tests: each numbered test exercises one invariant, uses signal-driven waits with fork/join timeouts (never `repeat(N)` fixed delays), and checks debug outputs that the RTL exposes specifically for verification.

### 1.5 Expose debug outputs as you go

While building, ask Claude Code to add `dbg_*` outputs from the RTL whenever a new internal state could matter. Examples from this project: `hbm_rx_curr_state[3:0]`, `dbg_wue_et_received`, `dbg_et_old_cap`, `dbg_et_new_cap`. These cost nothing in synthesis (they are optimized away if unused at the top level) and save hours when something inevitably breaks. The rule of thumb: **if a state would be useful to print in a waveform during a failure, expose it now, not after the failure.**

### 1.6 Compile early, simulate often

Run `xvlog -sv -d SIM ...` after even small RTL edits. Verilog's lint and elaboration errors are excellent — they catch dangling ports, width mismatches, and missing nets immediately. Claude Code can iterate on a compile-error loop very efficiently, but only if you actually run the compile. The slowest workflow is to batch up 400 lines of changes and then debug a wall of `xelab` errors all at once.

---

## Section 2: Debugging

Debugging RTL with Claude Code is a different discipline from designing it. The model is excellent at hypothesis generation, but it cannot see your waveform — *you* are the sensor, and the model is the analyst. The job is to feed it the right observations.

### 2.1 Capture observations before forming hypotheses

When a test fails, resist the urge to immediately ask *"why did this fail?"* Instead, gather:

1. **The exact test name and assertion that failed.** Not "test 8 broke" — paste the `$display` or `$error` line verbatim.
2. **The relevant VCD signals at the moment of failure.** State machine state codes, key counters, valid/ready handshakes for the channel involved.
3. **The last known-good commit.** `git log --oneline` on the file is a one-second lookup that often points directly at the culprit (see the `exec_bram_spiked` regression documented in the project memory, which was traced to commit `d07e446` simply by running `git log` on `internal_events_processor.v`).

Now give Claude Code all three. A debugging prompt that includes the failure message, the FSM state at failure, and the suspect commit is solving a much smaller search problem than a prompt that just says "test 8 is failing."

### 2.2 Make the model justify its hypothesis against the waveform

The single biggest debugging trap with any LLM on RTL is **plausible-but-wrong root cause analysis**. Claude will confidently propose "the reset is missing on `hbm_count`" when the real issue is a non-blocking-vs-blocking assignment race in the testbench. Both are the kind of thing that *could* cause the symptom, but only one is actually true on this run.

The defense is to demand evidence:

> "Before changing any RTL, tell me which specific cycle in the VCD I should look at to confirm your hypothesis, and what signal value at that cycle would prove or disprove it."

If the model cannot point at a cycle, it does not yet have a real hypothesis — it has a guess. Make it think harder before letting it edit. This is the debugging analogue of the design-phase rule from Section 1.3: **always require mechanistic justification before action.** "This should work" is not justification; "in cycle 42 the IEP latches `exec_run=0` because the blocking assignment fires before the active region read" is.

### 2.3 Bisect with `git`, not with intuition

When a regression appears between two known states, ask Claude Code to help bisect the commits between them. RTL regressions are often subtle (a removed reset condition, a tightened FSM gate), and the bisect output points at a single suspect commit in O(log N) simulation runs. This is how the `exec_bram_spiked` gate regression was localized in this project — the commit message *claimed* a membrane-update fix, but the actual change had killed the WUE mem-only push path for tests that didn't set `exec_bram_spiked`.

Lesson: **commit messages describe intent; diffs describe behavior.** Have Claude read the diff, not the message.

### 2.4 Trust the simulator's race semantics, not vibes

Several of the hardest bugs in this project — the `hbm_count` carryover, the `trigger_exec_run` NBA-vs-blocking race, the ASB spurious Phase 1 write — were all timing-region bugs that look fine in source but only fail in specific cycle alignments. When Claude Code proposes a fix for an intermittent failure, ask it to explain the fix in terms of **active vs. NBA regions**, not just "this should work." If it cannot describe why the old code raced and the new code does not, the fix is probably masking the bug instead of resolving it.

### 2.5 Prompt one step at a time — never bundle

The most common failure mode in an LLM debugging session is asking too many things at once. A prompt like *"look at the VCD, figure out what's wrong, and fix it"* almost always produces a confident-sounding edit that touches the wrong signal, because the model conflates *observation*, *diagnosis*, and *repair* into one pass and skips the parts that would have caught its own mistake.

**Issue one command at a time.** Wait for the model's output, hand-check it, push back if it skipped ahead, and only then send the next prompt. The phases below are the order that has worked on this project. Do not let the model jump phases.

**Phase 1 — Read the debug outputs and the VCD.** No analysis, no hypotheses, no fixes. Just a complete factual readout.

> "Read the simulation log at [tb_logs/step8_rstdp_100step_tb_log.txt](../../hardware_code_rstdp/tb_logs/step8_rstdp_100step_tb_log.txt) and the VCD waveform. Report only what you observe: which tests failed, what assertions triggered, what `dbg_*` signals showed at the moment of each failure, and what the FSM states were. Do not propose causes. Do not propose fixes. I want a flat list of observations."

This phase exists to keep the model honest. If you skip it, you will never know whether a later hypothesis was grounded in what actually happened or in what the model *assumed* happened.

**Phase 2 — Group the failures.** Many "different" failures in RTL are the same bug expressed in different tests.

> "From the observations in your previous response, group the failures by likely shared symptom. Tests that show the same FSM stall, the same missing handshake, or the same wrong-value pattern belong together. Do not yet speculate about causes — just produce the groups and explain what symptom each group shares."

This is the step that prevents you from chasing five bugs when there is really one. It also makes the search space dramatically smaller for Phase 3.

**Phase 3 — Enumerate possible causes, *without* proposing fixes.** This is the rule the model will most want to violate. Hold the line.

> "For each failure group, list 2–4 mechanistic explanations that could produce that exact symptom in this RTL. Each explanation must cite a specific signal, FSM transition, or reset condition in the actual source files. **Do not propose any fixes.** Do not say 'we could just' or 'one approach would be.' I only want causes. If you cannot tie a candidate cause to a specific line in the source, drop it from the list."

Banning fix-talk in this phase forces the model to stay in diagnostic mode. The moment fixes enter the prompt, the model anchors on whichever fix sounds cleanest and starts reasoning backward to justify it — which is exactly how plausible-but-wrong hypotheses get locked in.

**Phase 4 — Brainstorm fixes, with explicit reasoning for and against each one.** Only now do fixes enter the conversation.

> "For the most likely cause from each group, propose 2–3 candidate fixes. For each fix, give me:
> 1. The specific lines that would change.
> 2. Why this fix resolves the cited cause — at the cycle level, not 'because it should.'
> 3. What this fix could break elsewhere in the design (other FSMs that read the same signal, other tests that depend on the current behavior, timing implications at 225 MHz vs 450 MHz).
> 4. Whether this fix is consistent with the larger project goals — e.g., does it preserve the R-STDP semantics, the HBM region layout, the existing testbench conventions?
> Rank the fixes by which best satisfies (2), (3), and (4) together. Do not edit any files yet."

The "what could it break" and "is it consistent with the larger project" questions are what distinguish a local fix from a real one. RTL is full of fixes that pass the failing test and silently break two others; demanding that the model reason about collateral effects before editing catches most of these on paper, before they cost a simulation cycle.

**Phase 5 — Apply the chosen fix, then re-test.** After you have personally picked a fix from Phase 4's ranking, instruct the model to apply *only that fix* and nothing else. Then re-run the targeted test, then the regression suite, then move on.

The discipline of this five-phase loop feels slow on the first failure of the session. By the third failure it is dramatically faster than the bundled approach, because each phase has caught and discarded wrong hypotheses on its own terms, and the fix that finally lands is the one that survived four rounds of scrutiny.

### 2.6 Update [CHANGES.md](../../hardware_code_rstdp/CHANGES.md) and the project memory after every non-trivial fix

This project keeps a `CHANGES.md` and a Claude `MEMORY.md` precisely so that *the next debugging session* — which may be weeks later, or run by a future contributor — does not have to rediscover the same race condition. After a real bug is found, ask Claude Code to draft the memory entry. The format that has worked here is: **what the bug was**, **why it was hard to see**, **the minimal fix**, and **the invariant the fix preserves**. The third item is what prevents the bug from being reintroduced.

---

## Section 3: Anti-patterns to Avoid

A few habits that look productive but consistently produce worse RTL:

- **Approving a plan without reading it.** The plan is your last cheap checkpoint. Skipping it means paying for every mistake at edit time, compile time, and simulation time.
- **Accepting "this is best practice" as justification.** Best practice for what? Best practice *here*, given the existing reset scheme and clock domains? If the model cannot localize the reasoning to your design, the suggestion is generic, and generic suggestions are how subtle bugs enter.
- **"Just regenerate the whole file."** Verilog is order-sensitive in subtle ways (port ordering on instantiations, parameter overrides, generate-block scoping). Always prefer targeted edits.
- **Letting Claude pick the testbench timings.** Fixed `#100` delays and `repeat(N)` waits are a recipe for flaky tests. Use signal-driven waits (`@(posedge done)`, fork/join with timeouts) and ask the model to do the same.
- **Asking Claude to "make the test pass."** This is an instruction to remove assertions, not to fix bugs. Always frame debugging prompts as "find why the RTL produces output X when the spec says Y," never "make the assertion green."
- **Skipping the VCD.** If a test fails and you have not opened the waveform, you do not yet have enough information to debug. The model cannot open it for you — you have to be the eyes.
- **Trusting memory over the repo.** Memory entries (including the ones in `MEMORY.md`) describe state at the time they were written. Before acting on a remembered file path, signal name, or line number, verify it still exists. The fix that was correct three weeks ago may have been refactored.

---

## Section 4: A Suggested Session Shape

A productive RTL session with Claude Code generally looks like this:

1. **Brainstorm first** if the approach is not obvious. Ask for 2–3 options with tradeoffs before committing to one.
2. **Open or write a spec file** for what you are about to do, even if it is 10 lines.
3. **Use Plan mode** if more than one module is involved. Read the plan line by line. Push back on any step you cannot justify.
4. **Implement the RTL change and the corresponding testbench task together**, in one Claude turn.
5. **Compile immediately** (`xvlog -sv -d SIM ...`). Resolve elaboration errors before simulating.
6. **Run the targeted test** (not the full suite — that comes later). Capture the VCD.
7. **If it fails**: gather the failure message, the FSM state at the moment of failure, and the suspect commit. Feed all three to Claude before asking for a hypothesis. Make it justify the hypothesis against the waveform before editing.
8. **If it passes**: run the full regression suite to check for collateral damage.
9. **Document the change** in `CHANGES.md` and, for any non-obvious bug or invariant, update `MEMORY.md`.

This shape — brainstorm, spec, reviewed plan, paired test, compile, targeted sim, evidence-based debug, regression, document — is slower per iteration than freeform prompting. But it produces RTL that survives the next change, which is the only metric that matters in hardware design.

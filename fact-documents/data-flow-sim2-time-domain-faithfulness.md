# Data-Flow / Design: SIM_INPROC 2-Instance Time-Domain Faithfulness

Status: **IMPLEMENTED (Phase 1) — stages built, default-off byte-identical, GATE-2
+ Phase-0b pass. CRITICAL FINDING: the acceptance-test cascade is NOT reproducible
single-batch through ANY faithful impairment (it is intrinsically multi-batch /
long-session) — see §9. Surfaced for decision, NOT tuned around.**
Worktree: `C:/Users/kamer/mercury_wt/sim-faithful` — branch `win/sim-faithful`
Base: `9336c23` (SIM_INPROC 2-instance OFDM data decode) merged with
`6f3c83a` (deterministic-floor) + `3a8f3f8` (LEVER P preamble amortization).
Merge commit: `98397b6`. Companion doc:
`data-flow-sim2-ofdm-delivery-cadence.md` (the GATE-2 sample-accounting record).

This document is the cross-layer owner (CLAUDE.md §5) for the time-domain
impairment stages added to `cl_sim_awgn` and for the SOF-jitter term at the
wire→capture site. Every claim below is cited `file:line`. Open questions
are marked **[?]**. Wrong facts get struck through, not deleted.

---

## §0. Why this exists (the user's ask)

"Make the sim better so we can iterate faster, hardware-only tests are slow."
The 2-instance in-process sim (`MERCURY_SIM_2INST=1 -m SIM_INPROC -n`) is the
off-bench validator for OFDM-tier control-loop work. It already decodes OFDM
byte-correct (CFG15/16, GATE-2; commit `9336c23`). But it does NOT reproduce the
**LEVER P 1-symbol MINI cascade** seen on hardware: the MINI delivers
~409 bps vs the FULL/P-OFF ~3230 bps (−87%) on the IONOS testbed
(`preamble_hw/p_on_cfg16_clean.json` vs `p_off_cfg16_clean.json`), yet IN-SIM the
1-sym MINI "decodes byte-correct" and shows no cascade. The sim is too clean to
discriminate FULL from MINI.

## §1. ROOT CAUSE (code-verified) — why the sim is too clean

The in-process wire is a **perfect shared clock**: three impairments that exist
on real hardware are all ZERO in the sim.

1. **SFO = 0.** `sim2_drain_to_wire` moves whole integer `sp`-sample symbols and
   ticks ONE shared clock (`arq_commander.cc:9700-9732`); there is no
   `rational_resampler` in the wire. The two instances share a sample clock.
2. **CFO = 0.** `cl_sim_awgn::process` applies only the deterministic floor +
   per-symbol common-phase phase-noise + AWGN — NO carrier ramp
   (`sim_channel.h:543-563`). There is no cross-frame phase ramp.
3. **SOF-jitter = 0.** Single-symbol pacing froze the preamble frame-aligned
   (`arq_commander.cc:9790-9805`). The start-of-frame lands on an exact sample.

The Schmidl-Cox timing metric M = |P|² / (A²·R) (`ofdm.cc:2670-2677`) is
magnitude-squared, so its **mean** (µ = 1/(1+1/ρ)², S&C 1997 eq.19) is CFO- and
gain-INVARIANT — undamaged at the sim's clean SNR. That is exactly why the 1-sym
MINI "passes" in-sim. The failure lives in the metric **variance** (eq.20,
σ² ~ 1/L) and in the half-symbol-repeat identity r[d+m] = r[d+m+L].
`nsym_override=1` (`ofdm.cc:2629-2636`) cuts the integration length
L = nsym·Nfft/nIS by 4× (4 sym → 1) ⇒ ~4× variance ⇒ the peak-pick jitters
±1 OFDM symbol. With SFO present, the repeat halves sub-sample-misalign (deflate
|P|², un-averaged at small L) and the FFT window creeps across the batch tail;
stale-reused CFO (`telecom_system.cc:2462-2474`, the MINI skips Moose) and SOF
wander move `predicted_pos` (`telecom_system.cc:1342`) out from under the
±2·gi verify window.

### HW signature (the calibration target — `preamble_hw/p_on_rsp.log`)
metric = 1.000 PASS on the flat MINI plateau **while** meanH collapses to ~0.33
and LDPC iter = 101 on a wrong-offset buffer → mis-extract → batch-tail loss →
320-retx SACK cascade → hard-stall → 408.9 vs 3229.7 bps (−87%). The FULL/P-OFF
baseline survives because it re-acquires a 4-symbol preamble + a fresh Moose CFO
every frame.

## §2. The four impairments (Phase 1 — IMPLEMENTED in `sim_channel.h`)

IMPLEMENTATION NOTES (what shipped vs the plan below):
- SFO `cl_sim_sfo`: built as planned (4-tap cubic Lagrange Farrow, per-output-sample
  clamped walk + static bias, streaming/phase-continuous, n-in/n-out exact).
- CFO `cl_sim_cfo`: the static-residual part is as planned; the "slow random-walk
  drift" was IMPLEMENTED AS A CLAMPED AR(1) (Ornstein-Uhlenbeck), NOT a free Wiener
  walk. **Correction to the plan:** a Wiener walk drifts WITHIN a frame (∝√n) and
  breaks the FULL/ARM-A arm (the anti-tuning gate) — verified empirically (CFO=2-8 Hz
  Wiener broke ARM A, var→0.46, iter=101). The AR(1) is intra-frame-flat so Moose
  (FULL) tracks it while the MINI reuses the stale estimate. Knob `_CFO_WALK_HZ` is
  now the AR(1) STATIONARY std; added `_CFO_DRIFT_F3DB` (correlation BW, default 0.05).
- AGC `cl_sim_agc_transient`: built; silence→signal edge detected in process().
- SOF jitter: NOT shipped — §9/F1 shows the timing axis is pinned by the per-frame
  decode-drive, so a wire→cap sample-jitter is a no-op against the pinned window.
  The §4 audit skeleton is retained for if/when the decode-drive is changed.

The original plan (for reference):

Calibrated to MEASURED hardware, never magic numbers. Default-off = byte-identical.
Per-direction state lives INSIDE `cl_sim_awgn` (one instance per direction at
`arq_commander.cc:9661-9662`). Each new stochastic stage gets its OWN
`cl_sim_xoshiro` seeded by a fixed bijection of the ctor seed (the `pn_seed()`
pattern at `sim_channel.h:745-748`), NEVER drawing from the AWGN/PN `rng_`.

1. **SFO [DOMINANT]** — port GNU Radio `gr-channels sro_model_impl.cc`:
   per-sample Gaussian random-walk offset `d_sro` (Hz) clamped to ±`sro_max_dev`,
   `mu_inc = 1 + d_sro/fs`, applied with a short polyphase/Farrow fractional
   interpolator (~4-6 taps). New `cl_sim_sfo` stage invoked FIRST in
   `cl_sim_awgn::process` (`sim_channel.h:547`, before `det_.apply`).
   Magnitude: `sro_max_dev` ~50 ppm = 50e-6·48000 ≈ 2.4 Hz; walk std ±1-2 ppm/hr;
   range 25-90 ppm. Knobs `MERCURY_SIM2_SFO_PPM`(0=off) / `_SFO_WALK_PPM` /
   `_SFO_SEED`. The wire still delivers whole-integer `sp`-sample symbols and ticks
   the clock by exactly `sp` — only the CONTENT is the resampled drifting copy
   (GATE-2 sample accounting unchanged). The SFO flows through the REAL production
   RX `rational_resampler` (`telecom_system.cc:916`).
2. **CFO [secondary]** — new `cl_sim_cfo` stage between SFO and `det_.apply`:
   per-acquisition static residual ~N(0,σ_cfo) + slow clamped random-walk drift,
   applied as signal·exp(j2π·cfo·n/fs). DISTINCT from `cl_sim_phase_noise` (that is
   a per-symbol-reset common-phase EVM ceiling; CFO is a cross-frame phase ramp the
   MINI never re-estimates). σ_cfo ~5-20 Hz (post-Moose residual band,
   `ofdm.cc:2477`). Knobs `MERCURY_SIM2_CFO_HZ`(0=off) / `_CFO_WALK_HZ` / `_CFO_SEED`.
3. **SOF JITTER** — emerges from the SFO integer-skip; ADD an explicit i.i.d.
   ±1-3 sample per-frame term at the wire→cap site (`arq_commander.cc:9800-9805`)
   for ablation. Applied identically to FULL+MINI (discrimination is intrinsic:
   4-sym localizes start <1 sample, 1-sym cannot). Keep `prep_pull_inline`
   advancing exactly one symbol; jitter is in sample ALIGNMENT not symbol count.
   Knobs `MERCURY_SIM2_SOF_JITTER_SAMP`(0=off) / `_SOF_SEED`.
   **CROSS-LAYER (CLAUDE.md §5):** this term touches `ring_write_index` /
   `ofdm_drift_per_frame` / `coarse_metric` — the producer/consumer audit is §4
   below and MUST be completed before the jitter term ships.
4. **AGC TRANSIENT [add LAST, only if a residual gap]** — one-pole
   g[n] = g_ss + (g_0−g_ss)·exp(−n/τ) over the first ~1 symbol of a fresh burst.
   τ ~0.5-1 symbol. Knobs `MERCURY_SIM2_AGC_TAU_SYM`(0=off) / `_AGC_STEP_DB`.
   Lowest priority (metric is energy-normalized ⇒ a smooth ramp largely cancels).

### Plus: the 2-sym schedule knob (Phase 0 — DONE)
`preamble_sched_nsymb` (`telecom_system.cc:546-572`) returned a hardcoded 1 for
all tail frames. Phase 0 added `MERCURY_SIM2_MINI_NSYM` (default 1) so the MINI
tail is 1 or 2 (clamped to [1, full_nsymb]); read ONCE via function-local static
so the function stays PURE (TX and RX derive bit-identical schedules). Default 1
⇒ byte-identical to the pre-knob schedule. This is ARM C.

## §3. Determinism (GATE-2) preservation contract

- Each new stochastic stage uses its OWN xoshiro, seeded by a fixed bijection of
  the ctor seed (`pn_seed()` at `sim_channel.h:745-748`); per-direction seed
  separation propagates. The AWGN/PN `rng_` stream is NEVER perturbed.
- Default-off short-circuits to a NO-OP: no rng draw, no state change ⇒
  `MERCURY_SIM_2INST` with no extra env stays byte-identical. Re-run the GATE-2
  same-seed-twice check + the legacy 19-byte ROBUST_0 smoke.
- Integer-symbol clock tick (`arq_commander.cc:9700-9732`) UNCHANGED.
- PRODUCTION (`-m ARQ`) byte-identical: `cl_sim_awgn` has NO production callers;
  `g_sim_inproc_pump` is null-default (`arq_common.cc:130`) so the pump is inert
  in `-m ARQ` / `-x sim`. Confirm with a `-m ARQ` before/after byte-compare.

## §4. Cross-layer data-flow audit — SOF jitter (CLAUDE.md §5) **[TODO Phase 1]**

The SOF-jitter term (impairment 3) perturbs the sample ALIGNMENT of the bytes
written to the wire. It crosses PHY (timing acquisition / `coarse_metric`) and the
sim-pump layer (`ring_write_index`). Before shipping the jitter term, complete:

**RESOLUTION: the SOF-jitter term is NOT shipped (see §9/F1).** The per-frame
decode-drive pins the FFT window at a deterministic ring offset, so a wire→cap
sample-alignment jitter cannot move the decode result — it would be a no-op against
the pinned window (and adding a no-op that LOOKS like an impairment would be
misleading). The audit below is recorded for the future case where the decode-drive
is changed to run a real per-frame coarse search.

1. **Producers**: `ring_write_index` written by `prep_pull_inline`
   (`arq_commander.cc:9892`, `+= symbol_period % sp`). `coarse_metric` written by the
   OFDM timing search (`ofdm.cc:2820+`, `best_coarse_metric`). `ofdm_drift_per_frame`
   — per-frame timing-drift accumulator (PHY).
2. **Consumers**: `ring_write_index` read by the OFDM decode snapshot
   (`arq_common.cc:6214`, snapshots ONLY at `frames_to_read==0`) and the coarse
   search window. `coarse_metric` read by the sub-peak-recovery gate
   (`telecom_system.cc:3007`, `>= 0.97`) + the OFDM-OK/FAIL log
   (`telecom_system.cc:2991/3092`).
3. **Valid states**: pre-write `ring_write_index=0` (reset at TX-END,
   `arq_common.cc:4066`); `frames_to_read` armed to `preamble_nSymb+Nsymb` on
   RECEIVING entry.
4. **Invariants**: the decode snapshot assumes a frame-aligned window at the falling
   edge of `frames_to_read` to 0 (the per-frame decode-drive guarantees this); a SOF
   jitter would VIOLATE the alignment — which is precisely why it cannot be added
   without first un-pinning the decode-drive.
5. **What the jitter term changes**: it would shift the wire→cap byte alignment, but
   the decode-drive re-aligns to symbol boundaries, so no consumer observes it ⇒ no-op.

GATE-2 sample-accounting invariant (owned by the companion cadence doc): the wire
delivers whole-integer `sp`-sample symbols and ticks the shared clock by exactly
`sp`. The jitter perturbs CONTENT alignment, not symbol COUNT — verify this holds
at `arq_commander.cc:9700-9732` and `:9790-9805`.

## §5. Acceptance test (falsifiable, two-sided, anti-tuning)

Common env: `MERCURY_SIM_2INST=1 MERCURY_SIM2_CFG=16 MERCURY_SIM2_PIN=1
MERCURY_SIM2_SNR3K=900 MERCURY_SIM2_SEED=12345 MERCURY_SIM2_PAYLOAD_BYTES=8000`
(a single pinned batch with many tail frames; multi-batch NOT required for the
1-sym gate) + the impairment knobs.

- **ARM A** = P-OFF (`MERCURY_SIM2_PREAMBLE_AMORT=0`).
- **ARM B** = P-ON 1-sym (`MERCURY_SIM2_PREAMBLE_AMORT=1`, `MERCURY_SIM2_MINI_NSYM`
  unset/1).
- **ARM C** = 2-sym (`MERCURY_SIM2_PREAMBLE_AMORT=1 MERCURY_SIM2_MINI_NSYM=2`).

PASS-AFTER (after the impairments land):
- ARM A stays CLEAN (~3230 bps within 5%, 0 OFDM-FAIL, coarse mean ≥0.99,
  low_metric_frac <0.01 — proves discrimination, NOT blanket degradation).
- ARM B CASCADES (delivered ~409 bps, ≥70% drop; ~21% OFDM-FAIL at iter=101 with
  healthy meanH ~0.989; coarse mean ~0.53 bimodal 0.24-0.29 band low_metric_frac
  ~0.46; SUBPEAK-REJECT meanH ~0.33 at metric=1.000; ack_frac ~0.65; hard-stall).
  Primary bar = the −87% + the metric=1.000-while-meanH-collapses tell + iter=101;
  the histogram shape is the calibration target.
- ARM C RECOVERS (close to ARM A) — must match `wakxzqmmo`'s 2-sym HW JSON when it
  lands (`preamble_hw_p2/`, p2on JSON pending).
- ANTI-TUNING GATE: the SAME magnitude set must cascade B AND recover C.

## §6. Phase 0b — EVM-ceiling re-validation (GATING)

The time-domain levers stack ON the freq-domain deterministic-floor ceiling, not
replace it. CFG15/16 must still decode 1.00 @ EVM ~14.7 after the merge. If the
ceiling REGRESSES, STOP. **Result recorded below in §7.**

## §7. Phase 0 build/validate log

- Merge: `98397b6` = merge-base `9336c23` + det-floor `6f3c83a`
  (only `include/common/sim_channel.h`) + LEVER P `3a8f3f8` (9 disjoint files).
  Clean (no source conflicts; the only `<<<<<<<`-shaped lines are vendored zstd /
  fntree comment banners). Both features present and intact.
- `MERCURY_SIM2_MINI_NSYM` knob: `telecom_system.cc:546-572` (default 1,
  byte-identical; clamped [1, full_nsymb]; PURE).
- EVM-ceiling re-validation (Phase 0b): **PASS.** CFG16 single-batch (2000 B,
  impairments off) decodes clean: var≈0.0316-0.0332 ⇒ EVM ≈ 14.8-15.0 dB (within
  14.6±0.3), meanH 0.982-0.983, iter 3-4, 13 OK / 0 FAIL, bytes_ok=1. The det-floor
  survived the merge.
- Default-off byte-identity: legacy 19-byte ROBUST_0 smoke unchanged
  (rx="MERCURY-2INST-HELLO", iters=15); ARM A default-off == pre-impairment baseline
  (iters=4, sim_ms=14854, 13 OK/0 FAIL).
- GATE-2 determinism (impairments ON, completing/clean config, same seed twice):
  byte-identical OFDM stream + iters=4 + bytes_ok=1. (A non-completing cascade config
  differs ONLY at the tail by wall-clock-timeout truncation, not nondeterminism.)
- Production -m ARQ byte-identical: `cl_sim_awgn` has NO production callers (grep:
  only `arq_commander.cc` `test_sim_inproc_2`); never instantiated in -m ARQ.
- `--test-sim-clock`: 0 failed.

## §9. CRITICAL FINDINGS — the acceptance-test cascade is multi-batch-intrinsic

**F1 — SFO does not discriminate in-sim (sim-architecture limitation, not a bug).**
The per-frame decode-drive (single-symbol pacing + decode at `frames_to_read==0`,
`arq_commander.cc:9790-9842`, commit 9336c23) hands the OFDM decoder a FRAME-ALIGNED
window at a deterministic ring offset, BYPASSING the Schmidl-Cox timing acquisition
that SFO perturbs on HW. Verified: SFO at 50 ppm AND 500 ppm leaves ARM A and ARM B
byte-identical to off. The very fix that made OFDM decode in-sim also removed the
timing-acquisition fragility. SFO is faithfully modeled in the sample content (flows
through the real RX `rational_resampler`) but cannot move a pinned window. NOT tuned
around (that would mask, CLAUDE.md §2) — the stage is left correct + default-off.

**F2 — CFO is the in-sim lever but only over a LONG session.** MINI-vs-FULL is
explicit in the freq path (`telecom_system.cc:2484`: MINI reuses prev-frame CFO; FULL
re-runs Moose). An AR(1) drift exercises it. BUT a single short batch (13 frames ≈ 1 s)
is too short: at any drift where the FULL stays clean (≤0.5 Hz / f3db 0.1), the MINI's
stale-by-one-frame CFO is absorbed by the per-symbol CPE corrector + ZF estimator
(var ~0.01-0.03, 13 OK / 0-2 FAIL, bytes_ok=1) — no cascade (the code comment at
`telecom_system.cc:2489` predicted exactly this: "stale-by-one-frame CFO on a
clean/short tail frame is safe"). A drift fast enough to cascade the MINI (≥1 Hz) ALSO
breaks the FULL — the anti-tuning gate fails. Verified by sweep.

**F3 — the faithful cascade is intrinsically multi-batch / long-session.** The HW
cascade (`p_on_cfg16_clean.json`: 408.9 vs 3229.7 bps −87%; low_metric_frac 0.46;
ofdm_fail 56/264 ≈ 21%; nReSent 320; ack_frac 0.65) COMPOUNDS the stale-CFO per-frame
error across MANY batches over 300 s AND amplifies it through the SACK-retx cascade
into a hard stall. A single-batch in-sim test cannot reproduce a cross-batch-
compounding phenomenon through any faithful impairment.

**Correction to §5:** the design assumed `MERCURY_SIM2_PAYLOAD_BYTES=8000` is "a
single pinned batch with many tail frames; multi-batch NOT required." MEASURED: 8000 B
at CFG16 (frame ≈ 155 B data, batch = 25 frames ≈ 3.4 KB) spans ~3 batches; even a
single batch is only ~13-25 frames ≈ 1 s — far too short for the cross-batch cascade.
The faithful acceptance test REQUIRES the multi-batch path.

**DEPENDENCY:** multi-batch in-sim is BLOCKED. The sibling worktree
`win/sim-multibatch-deferTX` implements Option (a) defer-the-TX (audited, §6 of that
worktree's copy of this doc); it correctly stops the depth-1 reentrant-TX spin
(DEFER-TX fires 1154× at depth 1), but multi-batch STILL does not complete because of
a SECOND deadlock facet: the responder→commander SACK/ACK turnaround delivery does not
complete under the single-thread pump's depth-0-gated delivery, so the commander
LINK-TIMEOUTs and re-HAILs instead of advancing to batch 2. That facet is in the pump
delivery cadence — the territory of Option (b) (the role-agnostic pump rewrite) which
the task explicitly scoped OUT. Surfaced for a decision.

**NET:** impairment stages correct + deterministic + calibrated + default-off
byte-identical; Phase 0b holds. The two-sided ARM A/B/C discrimination is NOT
demonstrable single-batch and is gated on the multi-batch unblock (Option b).

## §10. F1 RESOLUTION — the PHY-level long-block-decode-under-SFO harness (BUILT)

F1/§9 says the in-process pump CANNOT exhibit timing-acquisition failure because
the per-frame decode-drive pins the FFT window. The DURABLE fix is NOT to un-pin the
pump (that reintroduces the 9336c23 delivery-cadence deadlock — §9 DEPENDENCY); it is
a SEPARATE PHY-level harness that makes the RX ACQUIRE timing from a long, SFO-drifted
continuous block. Built this session (`win/sim-faithful`):

**Entry:** `-m PLOT_PASSBAND -s <cfg>` with env `MERCURY_SFO_BLOCK_TEST=1`. Implemented
as `cl_telecom_system::sfo_block_test()` (`telecom_system.cc`, dispatched at the top of
`BER_PLOT_passband_process_main`). Single-file change + 1 header decl + a
`#include "common/sim_channel.h"`.

**Why this is faithful where the BER sweep / pump are not.** Both pinned paths set
`ofdm_forced_delay` to a KNOWN position and the RX then BYPASSES Schmidl-Cox + Moose
(`:504` sets it; `:1010/:2137/:2462` bypass). The harness instead:
1. TXes N back-to-back OFDM frames (the SAME `transmit_byte`) into ONE contiguous
   passband buffer — a real batch is back-to-back frames on ONE continuous TX clock.
   Per-frame preamble length is driven by LEVER P (`tx_preamble_nsymb_override` +
   `preamble_sched_nsymb`, exactly as `arq_common.cc:3918`), so FULL vs MINI arms emit
   the genuine 4-sym / 1-sym geometry.
2. Drifts the WHOLE buffer through ONE `cl_sim_sfo` (sim_channel.h). One stateful
   instance ⇒ the fractional accumulator + integer SOF-creep carry across the entire
   block; frame boundaries are NOT realigned (the HW continuous-clock drift). The
   integer part skips/repeats whole samples (SOF creep); the fractional part
   sub-sample-misaligns the Schmidl-Cox half-symbol repeat.
3. Per frame, presents a `buffer_Nsymb` window to the REAL `receive_byte` with
   `ofdm_forced_delay = -1` ⇒ real Schmidl-Cox time sync + Moose carrier sync + channel
   est + equalizer + LDPC; compares decoded bits to the known TX bits, and records the
   detected (drifting) preamble position + coarse metric to expose the timing creep.

**Placement gotcha (fixed).** The RX coarse-bounds gate (`:1701,:1730`) requires
`pream_symb_loc > preamble_nSymb`, so the frame must be placed with a lead of
`(preamble_nSymb+2)` symbols (mirrors the BER path's `(preamble_nSymb+2)*Nofdm+delay`
forced-delay convention). A 2-symbol lead landed the preamble at symbol 2 < bound 4 ⇒
every frame SKIPPED pre-LDPC (BER 0.5). With the correct lead the clean control decodes
fully.

**Build/baseline status (this session, Phase 1 — build only, NOT yet validated):**
- o3 build clean (no errors, no c++17 extension warning after rewriting the init-if).
- Installed to `C:\Program Files\Mercury\mercury.exe`.
- CLEAN CONTROL (SFO OFF, ESN0=900, CFG16, FULL arm, 4 frames): **4/4 decoded,
  block_BER=0**, det delay 7439-7440 vs expected 7440, metric 0.9987 — i.e. real
  acquisition works and the acceptance-test-1 baseline holds.
- Standard BER sweep CFG16 sanity unchanged (`--ber-esn0=30` → `30;0`).

**Knobs:** `MERCURY_SFO_BLOCK_NFRAMES` (60), `MERCURY_SFO_BLOCK_ESN0` (900=clean),
`MERCURY_SFO_BLOCK_SEED` (12345); SFO via `MERCURY_SIM2_SFO_PPM/_WALK_PPM/_MAX_PPM`;
MINI arm via `preamble_amortization_enabled` + `MERCURY_SIM2_MINI_NSYM`.

**STILL TO VALIDATE (Phase 2, NOT done here):** (1) SFO-MATTERS — a 60-frame CFG16
block at 50 ppm must DEGRADE vs the clean control (timing creeps, tail decode fails);
(2) P-REPRODUCTION — MINI (1-sym) under 50 ppm must FAIL on the tail while FULL (4-sym)
HOLDS, matching the HW −90% signature. If SFO cannot be made to affect decode in this
harness, the investment FAILED — report honestly, do not declare success on a still-
pinned window.

## §8. Calibration artifacts (already captured — CONSUME only)

- `preamble_hw/p_on_cfg16_clean.json` (1-sym), `p_off_cfg16_clean.json`,
  `p_on_rsp.log` (CLK drift + desync forensics, first fail :2728),
  `PHASE3_VERDICT.md`.
- `preamble_hw_p2/` (2-sym; p2on JSON pending from wf `wakxzqmmo`).
- DO NOT touch the concurrent HW run (wf `wme7070ep`/`wakxzqmmo`), the bench, or the
  butler — purely consume the JSON.

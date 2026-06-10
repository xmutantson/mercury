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

**VALIDATED (Phase 2, this session) — see §11 for the verdict and the falsified claim.**
Headline: GATE 1 (SFO-MATTERS) PASSES for the FULL arm — SFO now reaches Schmidl-Cox;
GATE 2 (P-REPRODUCTION) FAILS — the MINI failure is SFO-INDEPENDENT (a harness artifact),
so the harness does NOT yet reproduce the HW −90% SFO signature.

## §8. Calibration artifacts (already captured — CONSUME only)

- `preamble_hw/p_on_cfg16_clean.json` (1-sym), `p_off_cfg16_clean.json`,
  `p_on_rsp.log` (CLK drift + desync forensics, first fail :2728),
  `PHASE3_VERDICT.md`.
- `preamble_hw_p2/` (2-sym; p2on JSON pending from wf `wakxzqmmo`).
- DO NOT touch the concurrent HW run (wf `wme7070ep`/`wakxzqmmo`), the bench, or the
  butler — purely consume the JSON.

## §11. PHASE 2 VALIDATION RESULTS + VERDICT (this session, 2026-06-05)

All runs: `-m PLOT_PASSBAND -s 16`, CFG16 32-QAM, 60 frames, seed 12345.
MINI-arm selection required a small env hook (`MERCURY_SFO_BLOCK_ARM=MINI|FULL`) —
the PLOT_PASSBAND path has no ARQ pump to set `preamble_amortization_enabled`, so the
Phase-1 build could not actually run the MINI arm. Hook added this session (1-file,
`telecom_system.cc` top of `sfo_block_test()`; default = member flag = FULL).

### GATE 1 — SFO-MATTERS: **PASS (FULL arm).**
| run | det frame0 → frame59 | frames_decoded | block_BER | mean_metric |
|-----|----------------------|----------------|-----------|-------------|
| FULL clean (SFO OFF), 60f | 7439 → 7439 (no creep) | 60/60 | 0 | 0.99687 |
| FULL **50 ppm** SFO, 60f  | **7440 → 7393** (−47 samp creep) | 60/60 | 0 | 0.99367 |
| FULL **500 ppm** SFO, 60f | 7440 → 7355 (−85 samp creep) | 60/60 | 0 | 0.99276 |

The detected preamble position **CREEPS MONOTONICALLY** with SFO (−47 samples at
50 ppm; 50 ppm × 967200 block-samples ≈ 48 — physics matches). Under the OLD pinned
in-process pump AND the standard BER sweep, this would be byte-identical to SFO-off
(`det` fixed) — F1/§9. **So SFO now genuinely reaches the timing-acquisition logic.**
The FULL 4-sym preamble HOLDS (60/60) because each `receive_byte` RE-ACQUIRES the
drifted preamble via the full Schmidl-Cox search every frame.

CAVEAT on what GATE 1 proves: the harness exercises **per-frame timing
RE-ACQUISITION under accumulated drift** (window moves, Schmidl-Cox re-finds it). It
does NOT exercise a single 60-symbol decode window where timing must HOLD across the
whole block WITHOUT re-pinning — which is the literal big-block TEST 1 ("per-symbol
CPE/PEG tracker holds lock across a 60-symbol block, byte-correct TAIL decode"). The
harness re-pins per frame; big-block forbids the re-pin. So GATE 1 validates that the
SFO stage + acquisition wiring are live, but it is NOT yet the big-block tracker test.

### GATE 2 — P-REPRODUCTION: **FAIL (does NOT isolate SFO).**
| run | frames_decoded | block_BER | mean_metric_MINI |
|-----|----------------|-----------|------------------|
| MINI **clean (SFO OFF)**, 60f | 1/60 | 0.49227 | 0.130 |
| MINI **50 ppm** SFO, 60f      | 1/60 | 0.49227 | 0.131 |

MINI 50 ppm is **byte-identical to MINI clean** (both 37806/76800 biterr). SFO adds
**ZERO** to the MINI failure. The MINI arm fails because the STANDALONE harness runs a
cold full-buffer Schmidl-Cox search per `receive_byte` (`ofdm_forced_delay=-1`) that
cannot acquire a 1-sym preamble — the detected positions are garbage (det=26020,
13620, 10900: random data correlation peaks, not the preamble). The metric ~0.13 is
the search noise floor, not an SFO-degraded peak.

ROOT CAUSE (verified at `telecom_system.cc:1349-1371`): production MINI decode does
NOT use the cold full search. It uses the **batch-predict narrow search**, gated by
`preamble_amortization_enabled && ofdm_batch_active && ofdm_search_raw > 0`
(`:1349-1354`), which computes `predicted_pos = ofdm_skip*sym_samples +
ofdm_drift_per_frame` (`:1365`) and verifies in a tiny `±2·GI` window (`:1369-1371`).
The standalone harness never sets up `ofdm_batch_active`/`ofdm_search_raw`/`ofdm_skip`,
so the batch-predict path is dead and MINI fails UNCONDITIONALLY.

The Phase-1 comment (`telecom_system.cc:5549-5555`) asserting "a 1-sym preamble
presents a 4× shorter half-symbol repeat to the SAME [full] search, so its metric
under-integrates under SFO regardless of ARQ state" is **FALSIFIED**: production never
decodes a MINI frame via the full search, and the harness's full search fails on MINI
even at 0 ppm. The REAL SFO-vulnerability of LEVER P lives in the
`ofdm_drift_per_frame` PREDICTION + narrow `±2·GI` verify window (`:1365-1371`): under
SFO the predicted position drifts out of the narrow window → re-pin metric collapses →
HW −90%. **That batch-predict path is exactly what the standalone harness bypasses.**

### VERDICT: **PARTIAL_NEEDS_MORE.**
- SFO MATTERS in the new harness (GATE 1 PASS, FULL arm) — durable progress, the F1
  pin is broken for the acquisition-runs-per-frame case. Every future
  timing/preamble lever can now be sim-checked that SFO at least reaches Schmidl-Cox.
- But P-reproduction (GATE 2) is NOT achieved: the MINI failure is a harness artifact
  (no batch-predict state), SFO-independent, so it does NOT match the HW −90%
  signature. Declaring success here would be a band-aid (CLAUDE.md §2).
- NOT ready for big-block TEST 1: (a) GATE 2 not reproduced; (b) even GATE 1 is
  per-frame re-acquisition, not the single-window 60-symbol HOLD that TEST 1 needs.

### NEXT STEP (the right fix, do NOT shotgun):
Drive the harness's per-frame `receive_byte` through the production **batch-predict**
state, not the cold full search — set `receive_stats.ofdm_batch_active=1`,
`ofdm_search_raw>0`, and seed `ofdm_skip`/`ofdm_drift_per_frame` from the previous
frame (mirror the ARQ batch cadence at `arq_commander` that normally maintains them),
so the MINI frame is located by the narrow predict-verify window. THEN:
  (1) MINI clean must DECODE (predict window finds the 1-sym preamble at 0 ppm);
  (2) MINI 50 ppm must FAIL on the tail (drift walks the prediction out of the ±2·GI
      window, metric ~0.25), while FULL 50 ppm HOLDS — the true HW −90% P-repro.
SEPARATELY, for big-block TEST 1 proper, add a single-window long-block decode (one
`receive_byte` over an N-symbol block, timing acquired ONCE at the head) so the
per-symbol CPE/PEG tracker is what holds lock across the tail — the harness currently
re-acquires per frame and so cannot test a tracker-hold.

## §12. BIG-BLOCK TEST 1 — ONE-ACQUISITION DECODE-THE-TAIL (BUILT + RESULT, 2026-06-05)

The §11 NEXT STEP is now DONE. Two big-block harnesses were added (entry via
`-m PLOT_PASSBAND -s 16`), both single-thread, no HW, no butler.

### §12.1 ARCHITECTURAL CONSTRAINT (code-verified) — Nsymb is NOT free
A SINGLE LDPC codeword is hardwired to `N = framesize = MERCURY_NORMAL = 1600` bits
(`ldpc.cc:66`, `physical_defines.h:31`). CFG16's `Nsymb=9` is DERIVED so that
`nData * log2(M) == 1600` (`telecom_system.cc:4598`, with `nData` = data carriers of
the 9×50 grid after the Dy=3 pilot lattice). `data_container.set_size` then ties
`nBits = nData*log2M` to the codeword (`:4713`); `transmit_bit` pads with
`nVirtual_data = ldpc.N - nBits` (`:643`). **Therefore a literal "one preamble + 60
DATA symbols carrying ONE codeword" is IMPOSSIBLE** without a new ~10 k-bit LDPC code
(`nVirtual_data` would go negative → corruption). The big-block must be EITHER (A) K
back-to-back 1600-bit codeword-frames under one acquisition, OR (B) a single
60-symbol grid scored UNCODED (the timing/channel-estimation question is orthogonal
to FEC). Both were built.

### §12.2 Harness A — K-tiled LDPC-coded big-block (`MERCURY_SFO_BIGBLOCK=1`)
In `sfo_block_test()`. TX K FULL frames into one contiguous SFO-drifted buffer (one
`cl_sim_sfo`). RX acquires ONCE on frame 0 (real Schmidl-Cox), then decodes frames
1..K-1 at `head_delay + k*full_frame` via `ofdm_forced_delay` — NO re-acquisition.
Negative control `MERCURY_SFO_BIGBLOCK_NOTRACK=1` injects cumulative per-frame skew.
**Result (CFG16, K=7 = 63 data symbols, frames_decoded):**
| ppm | 0 | 25 | 50 | 80 | NOTRACK@50 |
|-----|---|----|----|----|-----------|
| dec | 7/7 | 7/7 | 7/7 | 7/7 | **2/7 (BER 0.076, monotonic tail fail)** |
Holds 9/9 up to the buffer limit (~80 data sym) at 80 ppm. The per-frame pilot
estimate re-derives per 9-sym codeword, so the only uncorrected error is the
cumulative frame-START offset, which over ≤80 sym at ≤80 ppm is ≤~7 samples — within
pilot-absorbable range. The NOTRACK control proves the harness BITES (tail fails
monotonically: biterr 71→134→138→153→186 across frames 2..6).

### §12.3 Harness B — GENUINE single 60-symbol grid (`MERCURY_SFO_GRID=1`)
In `sfo_grid_test()`. Temporarily rebuilds the `ofdm` object at `Nsymb=60` (32QAM,
33% pilots Dx=1/Dy=3 → nData=2000, nPilots=1000), TXs ONE grid of known 32QAM,
applies the SFO as the EXACT per-symbol subcarrier phase ramp
`exp(-j 2π k τ_n / Nfft)`, `τ_n = ppm·1e-6·n·(Nfft+Ngi)` (the omega+k·delta growth),
then decodes with ONE channel estimate interpolated across all 60 symbols
(`LS_channel_estimator` + `interpolate_bilinear_matrix`). Scored UNCODED, head vs
tail. Negative control `MERCURY_SFO_GRID_NOINTERP=1` freezes a flat-unity estimate
(no ramp removal). STEP-2 tracker `MERCURY_SFO_GRID_TRACK=1` = per-symbol CPE/PEG LS
line-fit of pilot phase-error vs k (intercept=CPE, slope=PEG), 9-sym sliding-window
average, de-rotate `exp(-j[ω+k·δ])` BEFORE the equalizer.
**Result (CFG16, Nsymb=60, uncoded_BER_all):**
| ppm | NOINTERP (neg-ctrl) | 2D-PILOT-EST (STEP 1) | CPE/PEG-TRACK (STEP 2) |
|-----|---------------------|------------------------|------------------------|
| 0   | 0                   | 0                      | 0                      |
| 25  | 0.0348 (tail 0.084) | 0.0012 (tail 0.0048)   | **0**                  |
| 50  | 0.1083 (tail 0.217) | 0.0160 (tail 0.0496)   | **0**                  |
| 80  | 0.1699 (tail 0.288) | 0.0460 (tail 0.113)    | **0**                  |

### §12.4 VERDICT — GO, with the tracker
- **Big-block timing HOLDS across 60 symbols under 50 ppm SFO with ONE acquisition.**
- On the K-tiled (per-9-sym-frame estimate) it holds with the EXISTING 2D-pilot est
  alone (no tracker). On the GENUINE single 60-sym grid (one estimate), the existing
  2D-pilot est does NOT cleanly hold — tail uncoded BER 5 % @50 ppm, 11 % @80 ppm,
  which would FAIL the rate-0.875 LDPC. The **CPE/PEG LS tracker (STEP 2) drives the
  single-grid tail to 0 BER across 0–80 ppm** — the anti-P core works exactly as the
  Speth/Fechtel/Meyr design predicts.
- Faithfulness PROVEN both ways: the NOTRACK (tiled) and NOINTERP (grid) negative
  controls FAIL the tail monotonically, so a "holds" result is real discrimination.
- The integer `rational_resampler` (no fractional trim) caps faithful blocks at ≤~80
  data symbols (creep <1 samp/frame); 60 is well inside. Did NOT chase 100-sym.
- NEXT: pilot-thin 33 %→6 % (Dy=3→Dy≈16) with the tracker carrying the timing (the
  net-PHY-recovery step = the design's TEST 2), then ARQ/SACK re-granularization so
  the big block is one ARQ unit. The tracker is the enabler for both.

## §13. TEST 2 — PILOT THINNING 33% → 6% (NET-PHY RECOVERY, BUILT + RESULT, 2026-06-05)

The §12.4 NEXT STEP is DONE. The §12.3 `sfo_grid_test` was extended with a
CONTINUAL+SCATTERED sparse-pilot lattice and a real LDPC coded decode. Entry is the
same (`-m PLOT_PASSBAND -s 16`), single-thread, no HW, no butler. **VERDICT: GO at 6%.**

### §13.1 The 6% layout (continual + scattered) — the configure() bypass
`MERCURY_SFO_GRID_THIN=1` OVERWRITES the lattice in-harness AFTER `ofdm.init()`
(telecom_system.cc ~5793-5870) — it does NOT touch the production
`cl_pilot_configurator::configure()` (ofdm.cc:1133), whose Dy=3 full-column lattice
is baked into all 17 configs and whose `nData` feeds the 1600-bit codeword sizing
(§12.1). The thin layout:
- **CONTINUAL columns** (`MERCURY_SFO_GRID_CONT_COLS`, default for 6% = **2**, carriers
  {0, Nc-1}): a pilot on EVERY symbol. These anchor the CPE/PEG per-symbol LS fit
  (omega+k·delta needs ≥2 pilots/symbol) AND feed the noise-variance estimator on
  every symbol.
- **SCATTERED diagonal** (`SCAT_DX`=12, `SCAT_DY`=4): interior pilots every 12 carriers,
  every 4th symbol, offset by `(n/SCAT_DY)` so the diagonal walks the band — the 2D
  samples the channel estimate needs without spending a full column.
- For Nc=50, Nsymb=60: 2 continual (120) + scatter (60) = **180 pilots = exactly 6.0%**,
  nData=2820 (vs 1000 pilots / nData=2000 at the stock 33%). pilots/sym min=2.

### §13.2 ROOT-CAUSE: the stock LS+DFT estimator SMEARS a sparse lattice
First attempt (sparse lattice fed to `LS_channel_estimator` + `smooth_channel_estimate_dft`)
gave **uncoded BER 0.34 even at 0 ppm** on a flat UNITY channel. DIAG: `|H|` ranged
0.05..1.37 (mean 0.38) instead of ~1.0; some data cells decoded to (0,0) (MMSE-erased
because |H|²<σ²). Mechanism: the per-symbol DFT smoother (ofdm.cc:2127) keeps ~`gi·Nc`≈13
time-domain taps assuming a DENSE REGULAR lattice makes H[k] flat across carriers. On a
sparse IRREGULAR lattice the per-cell LS scalar + pilot cells make H[k] rippled; its IFFT
is not a clean delta, so windowing SMEARS it. This is the dense-lattice assumption baked
into the estimator, not a tuning issue. **Fix (telecom_system.cc, thin path only):** after
the CPE/PEG tracker removes the SFO ramp the channel is flat unity, so the ML estimate is
the pilot-averaged complex gain `H̄ = mean(Y_pilot/X_pilot)` assigned to every cell, with
nv = the pilot residual EVM against H̄. With H̄ the estimate reads |H|=1.0 everywhere →
BER 0. (A production sparse-pilot CFG would need a sparse-capable 2D interpolator, e.g.
DFT-interp over the scattered lattice — out of TEST-2 scope; the flat-channel estimator is
exact for THIS deterministic-floor bench.)

### §13.3 RESULTS at 6% pilots (cont=2, scat dx=12/dy=4, 180 pilots)
**(a) UNCODED tracker hold** — clean channel, CPE/PEG track, SFO sweep:

| ppm | 0 | 25 | 50 | 80 |
|-----|---|----|----|----|
| uncoded_BER (track) | 0 | 0 | 0 | **0** |
| uncoded_BER (NO track, 2D only) | 0 | — | 0.060 (tail 0.117) | 0.141 |

The tracker is LOAD-BEARING: without it the sparse-pilot 2D estimate cannot hold the
ramp (tail 0.117 @50 ppm — would fail LDPC); with it, 0 BER across 0–80 ppm. The negative
control bites → the hold is real discrimination, not a lenient harness.

**(b) CODED K-codeword decode + nv** — `MERCURY_SFO_GRID_CODED=1`, K=8 real rate-0.875
(N=1600, K=1400, P=200) systematic LDPC codewords, demapped with the harness nv, AWGN via
`MERCURY_SFO_GRID_ESN0`. At EsN0=16 dB (near the 32-QAM r=0.875 operating point), 50 ppm:

| EsN0 dB | 18 | 16 | 14 | 12 | 30 |
|---------|----|----|----|----|----|
| codewords_decoded | 8/8 | 8/8 | 0/8 | 0/8 | 8/8 |
| nv | 0.0145 | 0.0229 | 0.0362 | 0.0573 | 0.00100 |
| iter_mean | 1.75 | 5.5 | 101(cap) | 101(cap) | 0 |

- nv TRACKS the true noise monotonically (0.001 @30 dB = 10⁻³ exactly; 0.057 @12 dB) and
  is **OK (>1e-5, NOT collapsed) at EVERY SNR incl. EsN0=30** — the E1/cfg16-nvfix collapse
  (nv→1e-6 → over-confident LLR → BP caps at 101 even at high SNR) does NOT occur.
- The iter-cap at 14/12 dB is the GENUINE LDPC waterfall edge (insufficient SNR), not an
  nv pathology — at 16/18/30 dB iter_mean is low and decode is clean.
- At EsN0=16, decode is **8/8 across the full 0/25/50/80 ppm SFO sweep**, nv≈0.023 stable.

**(c) NET-PHY** — block airtime = 60 sym × Tsym(0.02583 s) = 1.55 s; net = nData·log2M·rate/airtime:

| layout | pilots | nData | net-PHY |
|--------|--------|-------|---------|
| 33% stock | 1000 (33%) | 2000 | 5645 bps (= documented CFG16 5665, validates the model) |
| **6% thin** | **180 (6%)** | **2820** | **7960 bps** |
| 8% thin | 240 | 2760 | 7790 bps |
| 11.6% thin | 350 | 2650 | 7480 bps |

**6% pilots → net-PHY 7960 bps = 1.41× stock CFG16, > VARA Standard 7050** (in the predicted
7370–8125 range). GO on all three requirements.

### §13.4 Floor note — below 6%
On a CLEAN channel the tracker holds the SFO timing even at 4% (cont=2, no scatter):
uncoded BER 0 across 0–80 ppm. The practical floor is set by channel-estimate NOISE margin,
not SFO: at EsN0=16 a 4% layout shows ~1.8% uncoded BER (residual the LDPC still corrects:
coded 9/9), but 6% gives uncoded 0 even under noise. So **6% is the robust recommended
layout** (strict uncoded-0 across the SFO band + comfortable coded margin), not a fallback —
no lower floor was needed.

### §13.5 What is NOT done (honest scope)
- The 6% lattice + flat-channel estimator live in the TEST harness (`sfo_grid_test`), not in
  a production CFG. Production thinning needs (i) a sparse-capable 2D channel interpolator
  replacing the dense-lattice LS+DFT smoother (§13.2), (ii) re-derived `nData`→codeword
  sizing per config, (iii) the CPE/PEG tracker wired into the live RX before the equalizer.
- Bench is the deterministic FLAT-channel floor (SFO is the only impairment). A
  frequency-selective Watterson channel would stress the channel estimate harder; the
  flat-channel estimator is exact only for this floor. The tracker + nv findings transfer;
  the estimator choice is bench-specific.

## §14. TEST 3 — SPARSE-CAPABLE 2D CHANNEL INTERPOLATOR on a SELECTIVE channel (BUILT + RESULT, 2026-06-05)

The §13.5 production gap is now CLOSED in the harness. The §13.2 flat-ML shortcut
(H̄=mean(Y/X), one global scalar) was REPLACED with a real sparse-capable 2D channel
interpolator and validated on a frequency-SELECTIVE channel under SFO. The flat-ML
shortcut is kept as the negative CONTROL. Entry unchanged (`-m PLOT_PASSBAND -s 16`),
single-thread, no HW, no butler. **VERDICT: GO at ~7.2% pilots on realistic selective
channels; the SHIPPED-floor all-pass is frequency-UNDERSAMPLED (no estimator can hold
it — a bench artifact, see §14.5).**

### §14.1 What was built (telecom_system.cc, psk.{h,cc}, harness-scoped)
1. **Frequency-selective channel injection** (`MERCURY_SFO_GRID_CHAN`): applied as a
   per-carrier complex `T(j)` multiplying every cell (channel is time-invariant /
   freq-selective only; the SFO ramp sits ON TOP). `w_j` = the TRUE centered FFT-bin
   angular freq of logical carrier j (zero_padder mapping, ofdm.cc:331/697-723).
   - `=1` DET-FLOOR (phase-dispersive, |T|=1): the SHIPPED Schroeder all-pass closed
     form `A(e^{jw})=(-g+e^{-jwD})/(1-g·e^{-jwD})` cascaded `ap_n`× (sim_channel.h:750).
   - `=2` TWO-RAY (magnitude-selective): the `fsel_test` model `T=1+a·e^{-jwΔ}`
     (telecom_system.h:324), unit-power normalized → |T| FADES (deep nulls).
2. **Sparse-2D interpolator** `grid_sparse2d_estimator()` (`MERCURY_SFO_GRID_SPARSE2D=1`)
   — the DVB-T-style separable scattered-pilot estimate (Hoeher/Kaiser/Robertson, ICASSP
   1997): (a) raw LS H=Y/X at every pilot; (b) TIME interp per carrier across the scatter
   lattice's dy spacing + short Wiener/MMSE moving-average smoother; (c) FREQUENCY interp
   across carriers within each symbol — in POLAR form (|H| + UNWRAPPED phase, NOT complex-
   linear, which cuts a chord through the origin when adjacent pilots differ by >π and
   collapses |H|→0); (d) optional DDCE (`MERCURY_SFO_GRID_DDCE=1`). nv = pilot residual
   EVM against the FINAL interpolated H (never collapses; continual cols keep ≥2·Ngrid
   pairs).
3. **CSI-weighted LLR** in the coded path (`MERCURY_SFO_GRID_CSI=1`, default on) — the
   PRODUCTION formula (telecom_system.cc:2901-2927): per-data-carrier LLRs scaled by
   normalized |H_k|², clipped ±40, so the LDPC discounts the deep-null carriers (on a
   flat channel weights≈1 → no change, TEST-2 unaffected). `psk.slice_nearest()` added
   for the DDCE hard decision.
4. **GENIE estimate** (`MERCURY_SFO_GRID_GENIE=1`) — hands the equalizer the EXACT
   `Tchan[j]×SFO-ramp` to separate the LDPC/SNR decodability limit from estimator quality.

### §14.2 The flat-ML control FAILS on selective (the gap is real)
At 7.2% pilots, 50 ppm, the flat-ML shortcut (one global H̄) on a selective channel:
| channel | flat-ML uncoded BER | flat-ML decoded |
|---------|---------------------|-----------------|
| 2-ray a=0.3 (1.9:1 mag null) @EsN0=22 | 0.133 | **0/8** (BP capped 101) |
| all-pass g=0.3/D=6/n=1 (|T|=1, phase) @EsN0=20 | 0.501 (random) | **0/8** |
The global scalar averages the dispersive phase to ~noise (nv blows to 0.10/1.38). This
proves the sparse interpolator is NEEDED.

### §14.3 The sparse-2D interpolator HOLDS the selective channel + 50 ppm SFO
Same channels, same SNR, 7.2% pilots, sparse-2D (tracker OFF — the per-symbol estimate
from the continual columns tracks the SFO ramp inherently; head==tail across 0/25/50/80
ppm):
| channel | sparse-2D uncoded BER | decoded | nv | iter_mean |
|---------|-----------------------|---------|-----|-----------|
| 2-ray a=0.3 (1.9:1) @EsN0=22 50ppm | 0.0080 | **8/8** | 0.0036 (OK) | 2.0 |
| 2-ray a=0.5 (3:1) @EsN0=24 50ppm | 0.0080 | **8/8** | — | — |
| all-pass g=0.3/D=6/n=1 @EsN0=20 50ppm | 0.0030 | **8/8** | 0.0054 (OK) | 0.9 |
| all-pass g=0.4/D=8/n=1 (1.31 turns) @EsN0=20 50ppm | 0.0041 | **8/8** | — | — |
nv tracks the true noise monotonically (0.0036@22dB → 0.0008@30dB) and is **OK (not
collapsed) at every SNR** — the TEST-2 nv that holds is preserved (no E1/cfg16-nvfix).
SFO held to 0 walk-off across 0/25/50/80 ppm (head==tail).

### §14.4 PILOT DENSITY + LAYOUT — 6% needs a FREQUENCY-FOCUSED layout
The §13.1 6% layout (cont2 + dx12/dy4) spends pilots on the TIME axis (dy=4). But a
frequency-selective STATIC channel is time-invariant → time pilots are mostly wasted;
the budget belongs in FREQUENCY density. The §13.1 layout fails the selective channel
(uncoded 0.14 @6%), but a frequency-focused layout (cont2, **dx=4** scatter, **dy=8**
sparse in time) decodes 8/8 at **7.2%** (cont2/dx4/dy12 gives 7/8 at 6.0%). Density sweep
(2-ray a=0.3, EsN0=22, 50ppm, sparse-2D):
| layout | pilots | decoded |
|--------|--------|---------|
| cont2 dx12 dy4 (§13.1) | 6.0% | 0/8 |
| cont2 dx4 dy12 | 6.0% | 7/8 |
| **cont2 dx4 dy8** | **7.2%** | **8/8** |
| cont2 dx4 dy6 | 8.0% | 8/8 |
**Holdable density on a realistic selective channel ≈ 7.2% < 15%.** net-PHY at 7.2% =
nData(2784)·log2(32)·0.875 / (60·0.02583 s) = **7859 bps > VARA Standard 7050** (vs the
flat 6% 7960). The net-PHY recovery is PRESERVED on the selective channel.

### §14.5 HONEST bounds — what does NOT hold + the SNR penalty
- **SHIPPED det-floor all-pass (g=0.5/D=16/n=3) is frequency-UNDERSAMPLED**: its phase
  swings **4.13 full turns (−25.9 rad) across the 50-carrier band** (adjacent carriers
  differ by ~π, pilots-12-apart by 3.6π). This is past the Nyquist limit for ANY
  scattered-pilot interpolation — sparse-2D gives 0/8 (uncoded 0.079) at 7.2%. This is
  NOT an estimator failure: the SHIPPED floor was tuned as an EVM-CEILING model (fixed
  pilot residual), not a realistic delay-spread channel. A real HF channel's delay spread
  is bounded by the CP (Ngi); the interpolable regime is per-carrier phase < ~1.5 turns
  across the band (D·n small), which the mild all-pass (§14.3) and 2-ray cases satisfy.
- **The selective channel costs ~4-6 dB of SNR vs flat** even with GENIE (perfect) CSI:
  the 2-ray a=0.5 (3:1 null) needs EsN0=20 (GENIE 8/8) vs flat EsN0=16 — the deep
  magnitude null erases ~3 carriers of capacity and rate-0.875 has almost no margin. This
  is a CHANNEL-CAPACITY limit, not estimator quality (genie confirms it).
- **The sparse-2D pays a further ~4-6 dB on a FLAT channel vs flat-ML** (needs EsN0=22 for
  8/8 vs flat-ML EsN0=16): the per-cell interpolated estimate is noisier than a global
  average. So the production rule is: flat-ML on a flat channel, sparse-2D when the
  channel is selective (the estimator should be channel-adaptive, OR the SNR margin must
  absorb the sparse-estimate noise). The long-Wiener time-smoother does NOT help under SFO
  with the tracker off (averaging across symbols destroys the SFO ramp); WIENER_LEN=5
  (default, short) is best.

### §14.6 VERDICT — GO (with the density + SNR caveats)
- The sparse-2D interpolator DECODES the 6%-ish big block (7.2% pilots) on a realistic
  frequency-selective channel (magnitude-selective 2-ray AND phase-dispersive all-pass)
  under 50 ppm SFO, coded rate-0.875 LDPC 8/8, nv preserved, **net-PHY 7859 > VARA 7050**.
- The flat-ML control FAILS 0/8 on the same channels → the sparse interpolator is needed
  and works (real discrimination, not a lenient harness).
- Holdable density ≈ 7.2% (< 15%); the §13.1 6% layout needs re-targeting to FREQUENCY
  density for a static selective channel.
- Bounds reported honestly: the SHIPPED-floor all-pass is Nyquist-undersampled (bench
  artifact), and selective channels cost real SNR margin (channel capacity, genie-
  confirmed) plus a sparse-estimate noise penalty on flat.

### §14.7 What is NOT done (next increments, still out of scope)
- The interpolator lives in `grid_sparse2d_estimator` (harness), not the live RX. The
  §13.5 (ii) codeword re-sizing and (iii) live-RX wiring (move into `receive_byte`,
  ARQ re-granularization) remain the next increments — NOT done here (estimator +
  validation only, per scope).
- The Watterson time-VARYING (fading) channel is not tested — this bench is selective-
  but-STATIC. A fading channel would need the time-interp/Wiener to actually track time
  variation (here it only suppresses noise); that is a further increment.

## §15. SIM-FAITHFULNESS GAP CLOSED — in-sim big-block CASE ran GREEN while the LIVE path heap-overran (2026-06-05)

**The gap.** The CFG16 big-block had a heap-corruption bug (full audit:
`data-flow-bigblock-arq-unit.md §13`): the K=8 concatenated block was WRITTEN INTO a stock
single-frame TX slot (`transmit_byte` config-only divert → `transmit_bigblock` wrote ~79360
doubles into a ~16120-double frame slot) and the K*ldpc.K=11200-int decode was COPIED OUT into
`data_container.data_byte[N_MAX=1600]` (a 9600-int forward overrun). On HW both instances
aborted with glibc heap corruption on the FIRST CFG16 block (`free(): invalid next size` /
`invalid pointer` in the PHY-SWITCH deinit). **Yet `--test-sim-inproc-bigblock` CASE A/B ran
GREEN.** Why: the in-sim harness `run_block_loopback` passed a CORRECTLY block-sized
`info_bits_out` and a block-sized `tx_pb` to the production `receive_byte`/`transmit_byte`, so
neither the copy-out nor the divert ever touched an UNDERSIZED PRODUCTION allocation. Canaries
were disabled passthrough (`include/debug/canary_guard.h`), so the Windows allocator's slack hid
the smash. **The sim exercised the block PHY but NOT the stock production buffers the live ARQ
path hands it** — the same class of faithfulness gap this document tracks (the sim modeled the
DSP but not the live data-flow plumbing).

**How it was closed.** Added `--test-sim-inproc-bigblock` **CASE C** ("production-buffer
big-block TX/RX heap-overrun guard"): it drives the EXACT live buffers with explicit tail
canaries — the RX leg calls the real `receive_byte(rx_pb, out)` with `out` a `data_byte`-shaped
buffer (canary AT the N_MAX boundary, exactly as `arq_common.cc:6765` passes
`data_container.data_byte`); the TX leg calls the real `transmit_byte` at CFG16 into a
frame-sized slot (canary at the slot boundary) WITHOUT arming the block-emit scope — the exact
declined-batch/control fallthrough that overran. `MERCURY_BIGBLOCK_OLDGATE=1` restores the
pre-fix behavior on the SAME binary, so the fail-before is reproducible without a revert build:
pre-fix BOTH canaries are SMASHED (CASE C FAIL); post-fix BOTH intact (CASE C PASS). This is a
deterministic, platform-independent bounds check that no longer depends on the OS allocator's
tolerance — the faithfulness gap (in-sim green while live-overrunning) is closed by a canary the
production data-flow now trips on any mis-sizing. (Fix branch `fix/bigblock-cfg16-heap-overrun`
off `integ/bigblock-merge-2026-06-05`@`fe9d3e3`; net-PHY 7859/7960 > VARA 7050 preserved — a
buffer-sizing/free correction, not a wire-format change.)

## §16. SIM-FAITHFULNESS GAP CLOSED (again) — CASE A-D ran GREEN while the LIVE big-block delivered 0 app bytes (2026-06-05)

Same shape as §15 (in-sim green / live broken), a different live-only defect class. The CFG16
big-block CASE A-D unit tests (`--test-sim-inproc-bigblock`) and the standalone `BIGBLOCK_LIVE`
validator all passed byte-correct, yet the LIVE 2-instance ARQ session delivered **0 of the app
bytes** (HW + SIM_INPROC). Root causes (full detail in `data-flow-bigblock-arq-unit.md` §15):
1. **Use-after-free** in `receive_bigblock`: `bigblock_restore_stock_config()` reallocs
   `ready_to_process_passband_delayed_data` while the caller's `data` pointer still aims at the
   freed buffer -> normalize reads freed heap (SIGSEGV in SIM_INPROC, garbage decode on HW).
2. **Partial-block decode**: the live RX arms `frames_to_read` for ONE stock frame (~13 sym) but
   the block is ~64 sym, so the decode snapshots before the late codewords arrive (cw0 clean,
   cw1..K-1 garbage).
3. **Missing FIFO delivery**: the CLEAN-block carve left slots `RECEIVED` and never called
   `copy_data_to_buffer()`, so bytes never reached `fifo_buffer_rx`.

**Why CASE A-D / BIGBLOCK_LIVE stayed green** (the faithfulness gap):
- They hand a **caller-owned `std::vector`** as the RX passband `data` (not the data_container
  buffer), so the `receive_bigblock` realloc could never dangle it -> the UAF was invisible.
- They drive **one `receive_bigblock` directly** on a window pre-sized to span the whole block, so
  the live per-block `frames_to_read` wait (and the partial-block decode) was never exercised.
- They assert on **`messages_rx[]` directly** (`bigblock_test_delivered_varlen` / `count_received`),
  so the missing `copy_data_to_buffer` FIFO push was invisible.

**Closed by a new SIM_INPROC full-path case** — `--test-bigblock-fullpath`
(`cl_arq_controller::test_sim_inproc_bigblock_fullpath`). It runs the 2-instance lockstep stepper
(`test_sim_inproc_2`) pinned at CFG16 with big-block framing on, driving the REAL
`transmit_byte`/`transmit_bigblock` -> wire/capture ring -> `receive_byte`/`receive_bigblock` ->
`bigblock_receive_carve` -> `bigblock_block_to_arq` -> `copy_data_to_buffer` -> `fifo_buffer_rx`,
and asserts the FULL message is delivered byte-faithful (`rx_have==payload_len`, `bytes_ok=1`,
first block clean=K/K). FAIL-BEFORE/PASS-AFTER on the SAME binary via `MERCURY_BIGBLOCK_DEFEAT_FIX=1`
(+ `MERCURY_SIM2_STOP_AFTER_FIRST_BLOCK=1` to break right after the first partial block, avoiding the
slow/unstable post-partial retry loop). Result: fail-before first_clean=1/8 rx=0; pass-after
first_clean=8/8 rx=1200/1200; ALL PASS. The SIM_INPROC big-block transfer now routes through the
SAME `receive_bigblock` + carve + whiten the production HW path uses, so a sim pass predicts HW.

(Fix branch `fix/bigblock-whiten-align` off `integ/bigblock-final-2026-06-05`@`c746064`. Net-PHY
UNCHANGED — buffer-lifetime + RX-wait + FIFO-delivery, not geometry; > VARA 7050 preserved.)

## §17. D1 — TIMING-FAITHFUL big-block SFO repro: SFO is NOT the HW cw1..cw7 corruption (2026-06-06)

On `fix/bigblock-multicw-dewhiten`@`6e08f2e` (block-span RX window clamp shipped) the big-block
FULLY ENGAGES on HW (climb→CFG16, BIGBLOCK-ELECT, 12-13x carve K=8), but cw0 delivers byte-correct
(wire_bsi=5, used_hdr=1) while **cw1..cw7 are CORRUPT → per-cw CRC demotes → every block PARTIAL
clean=0 → 0/1200 byte-faithful (recv=113 = cw0 head only), DETERMINISTIC**. The in-process
`test_bigblock_multicw` ARM-A reports first_block clean=8/8, rx=1200/1200, bytes_ok=1 (§9/F1: the
in-proc sim bypasses Schmidl-Cox timing acquisition, so SFO/timing drift is invisible to it).

**HYPOTHESIS tested**: cw0 sits at the block start (fresh acquisition + preamble channel estimate)
and decodes clean; cw1..cw7 are progressively deeper into the ~74-symbol single-acquisition block
where SFO accumulates + the channel estimate goes stale, corrupting the later codewords on a real
channel. The design DEMOTED a separate CPE/PEG (common-phase + SFO-slope) tracker, betting the
per-symbol continual-pilot estimate ABSORBS SFO across the block.

**Harness (built, `scratch/d1-sfo-repro`@`0fcab7d`, off `6e08f2e`)**: `bigblock_sfo_test()`, entry
`-m PLOT_PASSBAND -s 16` env `MERCURY_BIGBLOCK_SFO_TEST=1`. TX a FULL K=8 CFG16 big-block via the
SHARED `bigblock_tx_passband` worker, drift the WHOLE contiguous capture buffer (lead silence +
block) through ONE stateful `cl_sim_sfo` (Farrow fractional resampler, sim_channel.h), optional
AWGN, decode via the REAL `bigblock_rx_passband` (ONE acquisition + MF-snap + pilot-EVM fine-timing
+ CPE/PEG track + sparse-2D/flat-ML continual-pilot estimate + CSI-LLR + per-cw LDPC + per-cw
known-payload gate). This is the timing-faithful complement the in-proc sim cannot be: it runs the
SAME production PHY worker the live/HW path runs, on a window that DOES span the whole block (so
the live `frames_to_read` arming bug is bypassed) — the ONLY new impairment vs the in-proc sim is
genuine SFO timing drift. Block geom (printed): cfg16 K=8, ldpc.N=1600/K=1400, block_samples=79360,
interp=4, Nofdm=310, Nsymb=9 → ~256-symbol contiguous block on one clock.

**RESULT (clean channel; per-cw clean/8, gradual onset pinned):**
| ppm | 0 | 25 | 50 | 90 | 100..140 | 150 | 200 | 300 | 400+ |
|-----|---|----|----|----|----------|-----|-----|-----|------|
| clean/8 | 8 | 8 | 8 | 8 | 8 | 7 (cw7) | 5 (cw5-7) | 2 (cw2-7) | 0 |

- The degradation is GRADUAL, tail-FIRST (cw7 breaks first, onset walks toward the head as ppm
  rises), monotone in ppm, deterministic, seed-invariant (static-bias SFO draws no rng).
- **First corruption at ~150 ppm; 8/8 clean through 140 ppm.** The HW-measured differential
  Fe-Pi crystal band is ~50-90 ppm — sitting in the all-clean region with ~60 ppm of margin.
- AWGN at a clean-cell 25 dB Es/N0: IDENTICAL to noiseless (8/8 to 90, cw7 at 150). At a marginal
  18 dB the decode goes RANDOM-scattered across codewords (no tail-first, no cw0-clean structure,
  acq_metric 0.91-0.93) — a 32-QAM/0.875 AWGN-floor effect, not the HW signature.
- `walk_ppm` with a large clamp drives the bias far past the static value within the buffer and
  collapses even the head acquisition (acq_metric→0.23, 0/8) — an acquisition failure, not a tail
  effect; the static-bias model (holds acquisition) is the right model for this question.

**VERDICT — the SFO/staleness mechanism is REAL but does NOT explain the HW failure at the HW
operating point.** (1) At 50-90 ppm (the HW band) the PHY worker decodes the FULL block 8/8
byte-correct — the design bet that the continual-pilot estimate absorbs SFO across the block HOLDS
to ~140 ppm. (2) SFO drift NEVER produces the HW cw0-clean/cw1..cw7-ALL-corrupt signature: it is
tail-first and partial (7/8, 5/8, 2/8), and only reaches "everything dead" at 300-400+ ppm (where
cw0 ALSO dies — never cw0-only). (3) ppm=0 clean = 8/8 here too (matches the in-proc sim) →
**the corruption is NOT in `bigblock_rx_passband` at HW conditions.** The HW recv=113=cw0-only
deterministic defect therefore lives DOWNSTREAM of the PHY worker on the LIVE path (capture-window
arming / block-span / carve / whiten / per-cw CRC plumbing — i.e. the `6e08f2e` block-span clamp
did not fully take on HW, or a sibling live-path bug in that chain remains), NOT in PHY timing
acquisition or channel-tracking. Next: instrument the LIVE receive_bigblock capture window + carve
on HW (recv=113 vs the 1200-byte block; does `frames_to_read`/`bigblock_rx_block_nsymb` actually
span all ~256 symbols at decode time on the live ring), per `data-flow-bigblock-arq-unit.md`.

(Scratch/diagnostic harness only — default-off, no production path change. `scratch/d1-sfo-repro`
off `fix/bigblock-multicw-dewhiten`@`6e08f2e`. cl_sim_sfo + sfo harness lineage already ancestral
at `6e08f2e` via `2a7a55c` — no cherry-pick needed.)

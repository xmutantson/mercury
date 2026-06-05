# Data-Flow / Design: SIM_INPROC 2-Instance Time-Domain Faithfulness

Status: **SKELETON (Phase 0 complete)** — impairment stages NOT yet implemented.
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

## §2. The four impairments to add (Phase 1+, NOT yet implemented)

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

1. **Producers** of `ring_write_index` / `ofdm_drift_per_frame` / `coarse_metric`:
   **[TODO — enumerate file:line each]**
2. **Consumers** of each: **[TODO — file:line each]**
3. **Valid states** incl. default-init (before any producer writes):
   **[TODO]**
4. **Invariants the consumers assume** (esp. uncommon paths: retx, FAIL re-anchor,
   config switch, batch boundary): **[TODO]**
5. **What the jitter term changes** — which assumption(s) it alters, walked per
   consumer: **[TODO]**

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
- EVM-ceiling re-validation: **see structured result for this Phase-0 run.**

## §8. Calibration artifacts (already captured — CONSUME only)

- `preamble_hw/p_on_cfg16_clean.json` (1-sym), `p_off_cfg16_clean.json`,
  `p_on_rsp.log` (CLK drift + desync forensics, first fail :2728),
  `PHASE3_VERDICT.md`.
- `preamble_hw_p2/` (2-sym; p2on JSON pending from wf `wakxzqmmo`).
- DO NOT touch the concurrent HW run (wf `wme7070ep`/`wakxzqmmo`), the bench, or the
  butler — purely consume the JSON.

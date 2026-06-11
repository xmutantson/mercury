# data-flow: Turbo-EQ — iterative decision-directed channel estimation

Cross-layer audit (CLAUDE.md §"Cross-Layer Data-Flow Audits") for **Lever #1 —
Turbo Equalization**. The turbo loop mutates the shared channel-estimation state
(`ofdm.estimated_channel`, `ofdm.noise_variance_estimate`) and the frame-buffer
state (`data_container.equalized_data` / `…demodulated_data` /
`…deinterleaved_data`) **multiple times per `receive_byte`**. This document is the
producer/consumer registry that audit requires BEFORE the change ships.

Design dossier: `bigblock_p3_hw/_deepdsp/RESEARCH_turbo-eq.md`.
Companion audit (the state this one extends): `data-flow-noise_variance_estimate.md`.

All file:line references are against the `feat/turbo-eq` worktree off `monitor`
(`cb5d519`). Facts are from executing/reading the code, not comments. Built
2026-06-10.

---

## §1 The state the turbo loop touches

| field | type | owner | meaning |
|---|---|---|---|
| `ofdm.estimated_channel[n*Nc+j]` | `st_channel_complex{value,status}` | `cl_ofdm` (`ofdm.h:266`) | per-cell channel estimate H + status. Turbo re-writes it each iteration from soft data (`data_aided_channel_estimator`). |
| `ofdm.noise_variance_estimate` | `double` | `cl_ofdm` (`ofdm.h:291`) | per-frame σ² → demapper LLR scale + MMSE erasure. Turbo re-estimates it each iteration (floored 1e-6, see I1). |
| `data_container.equalized_data` / `…ofdm_deframed_data` / `…ofdm_time_freq_deinterleaved_data` / `…demodulated_data` / `…deinterleaved_data` | buffers | `cl_data_container` | RX pipeline scratch. Turbo OVER-writes them on each pass (re-equalize → re-demap → re-decode). |
| `app_llr` (NEW, local) | `double*` | turbo loop local | the LDPC a-posteriori LLR exposed at `ldpc_decoder_SPA.cc` (the keystone). |

`estimated_channel` + `noise_variance_estimate` are the SAME coupled unit the
companion doc treats as one (the MMSE erasure `ofdm.cc:2365` reads both).

---

## §2 PRODUCERS — every path that WRITES the state

### §2.1 Pre-existing producers (unchanged, listed for completeness)
- `cl_ofdm::ZF_channel_estimator` `ofdm.cc:1521`
- `cl_ofdm::LS_channel_estimator` `ofdm.cc:1650`
- `cl_ofdm::LS_channel_estimator_tinterp` `ofdm.cc:1914`
- `cl_ofdm::channel_equalizer` `ofdm.cc:2338` — note it sets every cell's
  `estimated_channel[..].status=UNKNOWN` at `ofdm.cc:2371` AFTER equalizing (so H
  status is consumed-then-wiped within one pass).
Selection at `telecom_system.cc:2726-2744` on `ofdm.channel_estimator`.

### §2.2 NEW producer — `cl_ofdm::data_aided_channel_estimator(rx, xbar, v)`
`ofdm.cc` (added after `LS_channel_estimator_tinterp`). Modeled on
`LS_channel_estimator`. Writes `estimated_channel[n*Nc+j].value/.status` for:
- PILOT cells: `rx/X`, treated as perfect virtual pilots (`v=0`).
- DATA cells: `rx·conj(x̄)/(|x̄|²+v)` (MMSE-style; `+v` prevents blow-up at `x̄≈0`).
Then runs the SAME `interpolate_linear_col` / `interpolate_bilinear_matrix` /
`smooth_channel_estimate_dft` smoothing the LS estimator uses, over a now
TIME-DENSE lattice (pilots + reliable data cells). Writes `noise_variance_estimate`
from the pilot+reliable-data residual, **floored at
`estimate_noise_from_pilot_pairs(rx)` (cross-pilot AWGN floor) and `1e-6`** — the
identical I1 guard TINTERP uses (`ofdm.cc:2035-2037`).

### §2.3 NEW caller — the turbo loop
- **Harness**: `sfo_grid_test` turbo block (`telecom_system.cc`, after the coded
  per-codeword decode `~:6943`), gated `MERCURY_SFO_GRID_TURBO_ITERS` (default 1).
  Owns the whole grid → re-estimate is a straight dense-lattice call (no
  acquisition concern, R5).
- **Production**: `receive_byte` turbo block (`telecom_system.cc`, after the CRC
  gate `~:3038`), gated `MERCURY_TURBO_ITERS` (default 1). Runs ONLY when
  `MERCURY_TURBO_ITERS>1` AND the it=0 pass FAILED CRC (a frame that already
  passed CRC exits — zero extra cost, byte-identical).

### §2.4 NEW soft-feedback producers (do NOT touch shared OFDM state)
- `cl_ldpc::decode(.., double* app_llr=nullptr)` `ldpc.cc:281` → `decode_SPA(.., app_llr)`
  `ldpc_decoder_SPA.cc:25`: copies `LLRtmp[0..N-1]` (already computed every BP
  iteration at `:167-173`) into `app_llr` before return when non-null. **Zero
  behavior change when null** (every existing caller).
- `cl_psk::soft_remod(llr, nItems, xbar_out, v_out)` `psk.cc`: soft symbol mean
  `x̄=Σ a·P(a|llr)` + variance `v=Σ|a|²P − |x̄|²` from the SAME private
  `constellation[]`/`nBits` table `mod`/`demod` use (mapping auto-consistent).

---

## §3 CONSUMERS — every path that READS the state (from companion doc §3, with
turbo deltas)

| # | consumer | file:line | reads | turbo impact |
|---|---|---|---|---|
| C1 | `channel_equalizer` MMSE/ZF erasure | `ofdm.cc:2365` | H **and** nv | each turbo pass re-equalizes with the refined H+nv. On EXIT the FINAL-iteration H+nv are what ship — same contract as today's single pass. |
| C2 | `psk.demod` LLR scale | `telecom_system.cc:2954/2987` (`variance`=nv) | nv | each pass re-demaps with refined nv (floored I1). |
| C3 | CSI-weighted LLR | `telecom_system.cc:2933-2943` | H (|H|²) | each pass recomputes CSI from refined H. |
| C4 | `mean_H` SUBPEAK-REJECT / SKIP-H gates | `telecom_system.cc` SUBPEAK `~:3064`, SKIP-H gate | H | **computed at it=0 ONLY** (before the turbo block). Turbo runs AFTER these gates pass, so it cannot change a gate decision. mean_H reported in logs is the it=0 value (unchanged). |
| C5 | SKIP-VAR sync gate | `telecom_system.cc:2883` | nv | **evaluated at it=0 ONLY** (`~:2883`, BEFORE equalize/decode). Turbo runs strictly AFTER the SKIP-VAR gate has already passed for this frame, so the gate never sees a turbo-mutated nv. |
| C6 | `last_channel_selectivity` / gearshift 2D | companion §3.5 | H (std/mean) | computed from the published estimate; turbo's final H is more fade-faithful (advisory axis only; same direction as the TINTERP improvement). |
| C7 | subpeak recovery `goto ofdm_subpeak_retry_point` | `telecom_system.cc:2455` / `:3088` | delay (not this state) | turbo is mutually exclusive with subpeak retry: subpeak fires inside the it=0 fail branch BEFORE turbo; once subpeak exhausts (`subpeak_recover_phase>=2`) the turbo block runs on the final acquired position. No interleaving. |
| C8 | big-block re-entry | `receive_bigblock` `telecom_system.cc:9002` calls `receive_byte` `~:523` | the whole RX state per sub-frame | turbo is per-`receive_byte`-call and self-contained; each big-block sub-frame independently runs (or skips) its own turbo passes. No state crosses sub-frame boundaries (the loop leaves final-iteration state = single-pass contract). |
| C9 | SNR measurement (OFDM-OK path) | `telecom_system.cc:3102-3143` | nv (`variance`), H | reads the FINAL-iteration nv/H — the values that produced the successful decode, the honest post-turbo SNR. |
| C10 | ARQ diagnostic (advisory) | companion §3.9 | nv | reads published final nv. |

---

## §4 VALID STATES (esp. BEFORE any turbo producer writes)

1. **Before the turbo block runs at all** (it=0 path): identical to today. The
   it=0 pass IS the current straight-line code (estimate → equalize → demap →
   decode → CRC). Turbo only adds passes AFTER a CRC FAIL with `MERCURY_TURBO_ITERS>1`.
2. **app_llr before SPA writes it**: caller-allocated `N`-element buffer; SPA fills
   `[0..N-1]`. If the SPA early-exits because the initial parity check already
   passes (`nOnes==0` at `ldpc_decoder_SPA.cc:77`, loop at `:128` never entered),
   `LLRtmp[i]` was set to the input `LLRi[i]` at `:60` → `app_llr` = the channel
   LLR (a-posteriori == a-priori when already valid). Correct: a converged frame
   exits on CRC before turbo, so this path's app_llr is only ever used on a
   CRC-fail frame, where it carries the decoder's best estimate.
3. **xbar/v before soft_remod writes them**: caller-allocated; soft_remod fills
   every data symbol. An all-zero `llr` → `x̄=0, v≈1` (maximally uncertain → that
   cell is auto-ignored by the `+v` MMSE weight). No NaN/Inf (the `tanh`/sum form
   is bounded; `|x̄|≤1`, `0≤v≤1` for a unit-energy constellation).
4. **Degenerate frame** (Nsymb≤0 || Nc≤0): `data_aided_channel_estimator` falls
   back to `LS_channel_estimator` (same guard pattern as
   `LS_channel_estimator_tinterp` `ofdm.cc:1918-1924`). No consumer sees junk.

---

## §5 INVARIANTS the consumers assume + how the turbo producer maintains them

Carried forward from companion doc §5 (I1–I5) **plus the turbo-specific I6/I7**:

| # | invariant | consumer | how turbo maintains it |
|---|---|---|---|
| **I1** | **nv > 0 (≥1e-6), NEVER collapsed below true noise** — carried EACH iteration | C1 MMSE, C2 psk LLR, C5 SKIP-VAR | `data_aided_channel_estimator` re-estimates nv as `max(data+pilot residual, estimate_noise_from_pilot_pairs)` then floors at `1e-6` — the EXACT TINTERP I1 guard. The data-aided residual can only ADD to (never drop below) the pre-EQ cross-pilot AWGN floor. **R3 (HW-only nv-collapse) is NOT removed.** |
| I2 | nv tracks the demapper's EFFECTIVE post-EQ noise | C2 | residual is measured vs the FINAL smoothed/interpolated H of that iteration, vs the same equalized constellation the demapper sees. |
| I3 | every cell MEASURED with finite H | C1, C3, C4 | data-aided estimator writes every cell (pilots + data), then interpolates/freq-fills; degenerate frame → LS fallback. |
| I4 | mean(|H|) in the real-frame band | C4 mean_H | mean_H is computed at it=0 only (C4); turbo never re-touches the gate. |
| I5 | std(|H|)/mean(|H|) reflects true selectivity | C6 gearshift | advisory axis; turbo's denser estimate reads truer selectivity (same direction as TINTERP). |
| **I6** | **the loop leaves FINAL-iteration H+nv = the state every EXIT consumer (C1/C2/C3/C6/C9/C10) expects** — identical contract to today's single pass | all | the published state after the loop is the iteration that produced the delivered frame (CRC pass) OR the best-of by the monotone gate (§6.5). Never a half-written intermediate. |
| **I7** | **monotone-safe: turbo NEVER delivers a frame worse than the it=0 single-pass result** | the ARQ layer above | §6 guards: soft feedback + improve-only weighting + monotone-or-revert on the SPA unsatisfied-check count (`nOnes`, `ldpc_decoder_SPA.cc:178-190`). If a pass does not reduce unsatisfied checks, REVERT to the prior decode and stop. |

---

## §6 ERROR-PROPAGATION GUARDS (mandatory, all from cited prior art)

Implemented exactly per dossier §6:
1. **Soft, not hard, feedback** — feed `x̄,v` (soft), uncertain symbols `v→1`
   auto-down-weighted. The single most important guard.
2. **Extrinsic-only + clip** — `L_ext = app_llr − L_ch`, clipped to ±40 (reuse the
   existing clamp `telecom_system.cc:2977`).
3. **Damping** — `L_ext ← λ·L_ext_new + (1−λ)·L_ext_old`, λ default 0.7
   (`MERCURY_TURBO_DAMP`).
4. **Improve-only weighting** — blend the data-aided estimate via the per-cell `v`;
   keep the pilots-only estimate as the floor (worst case ≈ pilots-only, no harm).
5. **Monotone-or-revert** — track the SPA unsatisfied-check count across iterations;
   if iteration `it` does not reduce it vs `it−1`, revert to the `it−1` decode and
   stop. Guarantees I7.

With (1)+(4)+(5) the loop is monotone-safe by construction (POOR-profile
divergence → "may not help" not "may regress").

---

## §7 WHAT THIS CHANGE ALTERS (the fix's assumption deltas, per audit Q5)

- **Default-OFF**: `MERCURY_TURBO_ITERS` unset/=1 and `MERCURY_SFO_GRID_TURBO_ITERS`
  unset/=1 ⇒ NO turbo pass runs ⇒ byte-identical to monitor. **Proven** by the
  md5 + `--test-climb-engine` + `--test-ofdm-fine-timing` byte-identical gate.
- **app_llr arg null on every existing call** ⇒ `decode_SPA`/`cl_ldpc::decode`
  behavior unchanged for all current callers (the harness per-codeword decode,
  the production `ldpc.decode` at `telecom_system.cc:3004`, the bigblock decoders).
- **New methods** (`soft_remod`, `data_aided_channel_estimator`) are only reachable
  from the gated turbo blocks; ungated, they are never called.
- **Cross-layer**: the loop restores/leaves `estimated_channel` + nv in the
  final-iteration state every existing consumer already expects on EXIT (I6) — the
  same state today's single pass leaves. No consumer above (C1–C10) observes a
  mid-loop intermediate.

## §8 PAIRED REGRESSION TEST (CLAUDE.md "Cross-Layer regression tests")

`tools/test_turbo_eq.py` drives the SFO-GRID harness:
- **Failing-first**: C-poor (Watterson `CHAN=3`, `fd=1.0`) — `LS` it=1 FAILS to
  decode (high `post_FEC_info_BER`), `GENIE` PASSES (the documented TINTERP/Dy=3
  Nyquist wall). This is the test that fails before the turbo loop exists.
- **After fix**: `TURBO it=2` moves `post_FEC_info_BER` materially toward GENIE.
- **Default-OFF byte-identical**: `MERCURY_SFO_GRID_TURBO_ITERS` unset reproduces
  the base `[SFO-GRID-CODED]` line exactly.
- **Monotone-safe (C-errprop)**: C-poor at low Es/N0 — TURBO-with-guards
  `codewords_decoded ≥ LS` (never worse), proving I7.

---

## §9 TINTERP-SEED extension (`feat/turbo-tinterp-seed`, TURBO_EQ_VERDICT.md §5)

The faded-front follow-up: the TURBO_EQ_VERDICT.md §1 cold-start finding showed
plain-LS-seeded turbo floors at 0/K on POOR/1 Hz (the data-aided refiner needs an
it=0 decode under ~0.05 BER, which plain LS does not deliver). The §5 recommended
stack seeds it=0 with TINTERP (the warm faded estimator) and keeps the TINTERP H as
the it≥1 floor. **Measured: it=0 seed-swap breaks the Dy=3 wall (POOR 0/36 → 22/36,
mean BER 0.204 → 0.013); the it≥1 refinement does NOT add on top (honest negative).**
Verdict: `bigblock_p3_hw/_turboseed/TURBO_TINTERP_SEED_VERDICT.md`.

### §9.1 NEW gating field (producer-side delta)
`bool cl_ofdm::dd_seed_floor` (`ofdm.h`, default **false**). Read ONLY inside
`data_aided_channel_estimator`. **false ⇒ the §2.2 pilots-only-floor behavior,
byte-identical** (md5-proven BASE-vs-NEW). When true, two deltas in
`data_aided_channel_estimator` (`ofdm.cc:2072`):
1. **H floor (dossier §6.4 done right):** a low-confidence DATA cell (`v≥conf_thresh`)
   falls back to the SNAPSHOT of the INCOMING `estimated_channel` (the it=0 TINTERP
   seed), MEASURED, instead of UNKNOWN/pilots-only-interp (`ofdm.cc:2122-2131`). The
   snapshot is taken at function entry, before Pass 1 clobbers `estimated_channel`.
2. **nv warm-anchor (item 4, the decisive fix):** the nv floor uses
   `min(estimate_noise_from_pilot_pairs, incoming-TINTERP-nv)` instead of the
   cross-pilot differential alone (`ofdm.cc:2262-2277`). On a fast (fd=1) fade the
   cross-pilot differential overcounts the Doppler as noise (~0.40 vs the
   TINTERP-honest ~0.027), which ALONE collapses the it=1 LLRs (a 13× nv blow-up that
   reverts the warm seed). The data residual can still RAISE nv (improve-only); the
   1e-6 floor and the R3 / I1 invariants are preserved.

### §9.2 Harness wiring
`MERCURY_SFO_GRID_TURBO_SEED=tinterp` (`telecom_system.cc`, sfo_grid_test) sets
`turbo_seed_tinterp`, which (a) routes the it=0 estimator to
`LS_channel_estimator_tinterp` (the existing `MERCURY_SFO_GRID_TINTERP_PROD` branch,
`telecom_system.cc:6878`) and (b) sets `ofdm.dd_seed_floor=true` in the turbo-knob
block. Default unset = plain LS it=0 + pilots-only floor = byte-identical.

### §9.3 Invariant deltas (audit Q5)
- **I1 (nv ≥ 1e-6, never collapsed)**: PRESERVED. The warm-anchor only LOWERS the
  *floor companion* to the honest TINTERP nv (≥ its own 1e-6 floor); nv = max(data
  residual, that floor) ≥ 1e-6. No new collapse path (R3 untouched — TINTERP's nv is
  itself a measured residual, not the HW-only post-EQ-EVM collapse).
- **I6 (loop leaves final-iteration state)**: PRESERVED. The snapshot is a local
  `std::vector`; the function still writes the full `estimated_channel`+nv on exit.
- **I7 (monotone-safe)**: PRESERVED and STRENGTHENED — the best-of publish now reverts
  to the it=0 *TINTERP* seed (a much stronger floor than plain LS) on POOR.

### §9.4 Paired regression test
`tools/test_turbo_tinterp_seed.py` (sibling of `test_turbo_eq.py`; the latter owns the
DET-FLOOR cell, this owns the POOR cell). FAILS A3 on BASE (`feat/turbo-eq` ignores
the env → TINTERP-seed == LS-seed, 0/36, wall unbroken); PASSES on
`feat/turbo-tinterp-seed` (22/36, mean BER 0.013 < waterfall 0.05). Asserts A1 LS
floors / A2 GENIE compass / A3 material crossing / A4 monotone-safe / A5 default-off
deterministic.

### §9.5 Clean-channel note (NOT a regression of this change)
On flat AWGN, `LS_channel_estimator_tinterp` underperforms plain LS (1/6 vs 6/6) —
PRE-EXISTING TINTERP behavior, md5-identical on BASE and NEW via the existing
`MERCURY_SFO_GRID_TINTERP_PROD` hook (verdict §5: "TINTERP does not help clean CFG16").
`TURBO_SEED=tinterp` is a FADE-tier-only experiment knob (default-off, never set in
production); the production estimator selection (`channel_estimator`) is untouched.

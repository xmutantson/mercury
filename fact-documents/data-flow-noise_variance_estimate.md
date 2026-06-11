# data-flow: `noise_variance_estimate` (+ `estimated_channel`) — cross-layer audit

Shared channel-estimation state owned by `cl_ofdm`. This document is the
producer/consumer registry the CLAUDE.md §Cross-Layer audit requires before any
change to the OFDM channel-estimate / noise-variance state. Built 2026-06-10 for
`feat/fade-tinterp` (promoting the sim-proven TINTERP estimator to production); it
also retroactively documents the state the `fix/cfg16-nv-restore` LS residual
estimator and the A.1.4 cross-pilot estimator already share.

All file:line references are against the `monitor`/`feat/fade-tinterp` tree
(branch off `monitor` @de428f6). Facts are from executing/reading the code, not
comments.

---

## §1 The state

| field | type | owner | meaning |
|---|---|---|---|
| `ofdm.noise_variance_estimate` | `double` | `cl_ofdm` (`ofdm.h:280`) | per-frame post-estimate noise variance σ² fed to the demapper LLR scale + MMSE erasure. Default-init `0.01` (`ofdm.cc:88`). |
| `ofdm.estimated_channel[i*Nc+j]` | `st_channel_complex{value,status}` | `cl_ofdm` (`ofdm.h:255`) | per-cell complex channel estimate H + status (`UNKNOWN`/`MEASURED`). |

They are produced together by whichever estimator runs and consumed together by
the equalizer + demapper. The audit treats them as one coupled unit because the
MMSE erasure (`§3.1`) reads BOTH in the same expression.

---

## §2 PRODUCERS (every path that WRITES the state)

### §2.1 Production estimators (real RX decode path)
The production receive loop selects the estimator on `ofdm.channel_estimator`:
- `telecom_system.cc:2726-2745` (the live ARQ/decode loop, `RX_TX_callback`-driven).
- `telecom_system.cc:332-339` (a second estimate call site, plot/visualization path).

| `channel_estimator` | producer | writes nv at | writes H at |
|---|---|---|---|
| `ZERO_FORCE` (0) | `cl_ofdm::ZF_channel_estimator` `ofdm.cc:1520` | `ofdm.cc:1598` via `estimate_noise_from_pilot_pairs` (cross-pilot differential) | `ofdm.cc:1529-1585` + DFT smooth `ofdm.cc:1643` |
| `LEAST_SQUARE` (1) | `cl_ofdm::LS_channel_estimator` `ofdm.cc:1649` | `ofdm.cc:1816-1845` (pilot-residual vs final smoothed H, floored 1e-6); A/B toggle `ls_use_crosspilot_nv` `ofdm.cc:1854` | `ofdm.cc:1744-1772` + DFT smooth `ofdm.cc:1779` |
| `TIME_INTERP` (2) **[feat/fade-tinterp, NEW]** | `cl_ofdm::LS_channel_estimator_tinterp` `ofdm.cc:1909` | nv = pilot-residual vs interpolated H, **FLOORED at `estimate_noise_from_pilot_pairs` (cross-pilot AWGN floor)**, floored 1e-6 | per-carrier linear time-interp + freq-interp fill, NO DFT smooth |

`channel_estimator` is set from config at `telecom_system.cc:9573`
(`ofdm.channel_estimator = ofdm_channel_estimator`), after the NB→ZF override at
`9569-9571`. The **FADE-tier gate** (`feat/fade-tinterp`) sits immediately after,
`telecom_system.cc:9574+`: when `MERCURY_FADE_TINTERP` is set non-zero AND the
config is a WB LS config (still `LEAST_SQUARE` at that point), it promotes
`channel_estimator → TIME_INTERP`. Default-OFF (env unset) ⇒ byte-identical.

### §2.2 cross-pilot differential helper (a producer of nv only)
`cl_ofdm::estimate_noise_from_pilot_pairs(in)` `ofdm.cc:1475`. Returns σ²/|X|² from
adjacent same-column pilot deltas (|H_a−H_b|²/2). It is the production-available
AWGN floor estimator: it measures only the PRE-equalization thermal floor, invariant
to the channel-estimate smoother. Used directly by ZF (`§2.1`), as the A/B toggle for
LS, and as the **nv-FLOOR** by TIME_INTERP. Returns `0.01` if <1 valid pair; floors
at `1e-6`.

### §2.3 Harness / synthetic producers (NOT the production decode path)
These write the SAME state but only in `-m PLOT_PASSBAND` test harnesses; they never
run in a deployed link. Listed so a future estimator change knows every writer:
- `sfo_grid_test()` estimator ladder `telecom_system.cc:6700-6790`: GENIE (true H +
  true-AWGN nv `6724-6726`), SPARSE2D `6730`, FLAT-ML control `6752-6765`,
  **TINTERP_PROD regression hook `6767-6781` (calls the production
  `LS_channel_estimator_tinterp`)**, prod-else LS `6783`.
- `sfo_block_test()` / bigblock harnesses: `telecom_system.cc:6201, 7908, 8557` (each
  writes nv via a local pilot-residual walk, floored 1e-6).
- The `MERCURY_SFO_GRID_*` prototype frag (sim-only, NOT promoted) wrote nv with a
  known-EsN0 floor `pow(10,-EsN0/10)` — the harness-only floor the production method
  replaces with the cross-pilot measurement.

---

## §3 CONSUMERS (every path that READS the state)

### §3.1 `cl_ofdm::channel_equalizer` — MMSE/ZF erasure (reads BOTH H and nv)
`ofdm.cc:2173-2209`. For QAM (amplitude-restoration OFF), per cell:
`alpha = |H|² / (|H|² + noise_variance_estimate)`; if `alpha ≤ 0.1` (or |H|²<1e-12)
the subcarrier is ERASED (set to 0), else ZF-equalized `in/H`. **This is the most
nv-sensitive consumer**: if nv collapses toward 0, alpha→1 everywhere → NO erasure of
deep-faded carriers → ZF noise-amplifies them → garbage LLRs. The nv-floor is what
keeps the erasure honest. PSK path (amplitude-restoration ON) uses pure ZF and does
NOT read nv.

### §3.2 `psk.demod` — LLR scale (reads nv as `variance`)
`psk.cc:280-333`. `eff_var = max(variance, var_floor=0.001)`;
`LLR[k] = (Dmin1 − Dmin0) / eff_var`. `variance` is passed `noise_variance_estimate`
from `telecom_system.cc:2910` (`variance = ofdm.noise_variance_estimate`). A collapsed
nv → ~1000× over-confident LLRs → BP iter-caps at 101 → CRC fail (the E1/cfg16-nvfix
class). `var_floor=0.001` is the last-resort guard; the cross-pilot floor keeps nv
well above it (≈0.0065 on the GOOD fade cell).

### §3.3 CSI-weighted LLR (reads H only)
`telecom_system.cc:2917-2970`. Per data carrier weight `|H_k|²` (from
`estimated_channel`), mean-normalized, multiplies the bit-LLRs (clamped ±40). Tells
LDPC which carriers to trust. Reads H, not nv. TINTERP supplies a per-cell H so this
weight is per-carrier meaningful (vs the LS global-window value).

### §3.4 `mean_H` — SKIP-H / SUBPEAK-REJECT timing gates (reads H only)
`telecom_system.cc:2735-2752` computes `mean_H = mean(|H|)` over MEASURED cells;
`receive_stats.mean_H = mean_H`. Consumed by:
- SUBPEAK-REJECT `2803`: `coarse_metric ≥ 0.97 && mean_H < 0.5` → reject (Schmidl-Cox
  sub-peak fingerprint).
- SKIP-H `2821`: `mean_H < mean_h_gate_threshold` → skip LDPC (bad timing).
These gate on |H| MAGNITUDE. **Invariant TINTERP must preserve**: on a real
(non-collapsed-timing) frame, `mean(|H|)` from the interpolated estimate must be in
the same band as LS (≈0.7–1.0), not artificially low. Verified: TINTERP publishes
H ≈ the pilot Y/X magnitude (no DFT attenuation), so mean_H is preserved.

### §3.5 `last_channel_selectivity` — 2D channel-state lookup / gearshift (reads H only)
`telecom_system.cc:2761-2789`: `std(|H|)/mean(|H|)` over DATA cells →
`last_channel_selectivity`, fed to the gearshift 2D channel-state lookup
(`channel-state-2d-lookup.md`). Reads H. TINTERP changes the per-cell |H| pattern
(it now tracks the fade rather than averaging it) → selectivity will read HIGHER on a
faded channel (correct: the channel IS selective in time). This is an intended signal
improvement, not a regression; the gearshift consumes it as an advisory axis, not a
hard gate.

### §3.6 SKIP-VAR sync gate (reads nv)
`telecom_system.cc:2871`: `if(skip_var_gate_enabled && noise_variance_estimate >
skip_var_ceiling(cfg))` → skip LDPC (frame is noise / below this config's floor);
3 consecutive → abort trial loop. **Invariant TINTERP must preserve**: nv must be a
TRUE per-config noise estimate, not collapsed (else a real-noise frame passes the gate
and wastes an LDPC burn) and not inflated (else a decodable frame is wrongly dropped).
The cross-pilot floor + pilot-residual ceiling keeps nv in the honest band
(0.0065→0.0443 across GOOD→POOR fade, all < the cfg15 ceiling, none collapsed).

### §3.7 `measure_variance` (an INDEPENDENT post-EQ quantity, NOT this state)
`telecom_system.cc:2902` `measure_var = ofdm.measure_variance(equalized_data)` is a
SEPARATE diagnostic (post-EQ |Y/H−X|²), logged at `[FRAME-NV]` `2911` but NOT fed to
the demapper (the demapper uses `noise_variance_estimate`, `2910`). Documented to
avoid confusion: changing the estimator does not change which quantity drives the LLR.

### §3.8 `gearshift_coarse_metric` / `coarse_metric` (NOT a consumer of this state)
`receive_stats.coarse_metric` is the Schmidl-Cox/preamble correlation
(`ofdm.cc:2878-2965`), computed UPSTREAM of channel estimation from the preamble
samples — it does NOT read `estimated_channel` or `noise_variance_estimate`. It is
combined with `mean_H` only at the SUBPEAK-REJECT gate (`§3.4`). Listed here to record
that the estimator change has NO path to the coarse metric.

### §3.9 ARQ diagnostic (reads nv, advisory)
`arq_common.cc:7689` logs `ofdm.noise_variance_estimate` in a frame-stats line.
Read-only diagnostic, no control-flow effect.

---

## §4 VALID STATES (esp. before any producer writes)

1. **Default-init** (`ofdm.cc:88`): `noise_variance_estimate = 0.01`,
   `estimated_channel[*].status = UNKNOWN`, `value = 0`. If the equalizer/demapper
   ran here (they don't — an estimator always runs first in the decode loop), alpha
   would use 0.01 and H=0 → all-erase. No path reaches a consumer before a producer.
2. **Post-ZF / Post-LS / Post-TINTERP**: every cell `MEASURED`, nv ≥ 1e-6 (all three
   floor at 1e-6). nv band on a good frame ≈ 0.01–0.10; on a faded frame TINTERP nv
   rises with severity (measured 0.0065/0.0217/0.0443 for GOOD/MOD/POOR).
3. **Degenerate frame** (Nsymb≤0 or Nc≤0, or <1 pilot pair): each producer returns a
   safe default (LS/TINTERP nv→0.01 or cross-pilot 0.01; TINTERP falls back to
   `LS_channel_estimator` when N≤0||C≤0). No NaN/Inf reaches a consumer.
4. **Post-equalizer**: `channel_equalizer` sets every cell `status = UNKNOWN`
   (`ofdm.cc:2206`) after consuming it — the estimate is single-use per frame; the
   next frame re-produces it. (A consumer that reads H AFTER the equalizer would see
   UNKNOWN — none do; CSI-weight `§3.3` and mean_H `§3.4`/selectivity `§3.5` all read
   H BEFORE the equalizer call at `2901`.)

---

## §5 INVARIANTS the consumers assume + how each producer maintains them

| # | invariant | consumer | LS | ZF | TINTERP |
|---|---|---|---|---|---|
| I1 | nv > 0 (≥1e-6), never collapsed below true noise | psk LLR §3.2, MMSE erasure §3.1, SKIP-VAR §3.6 | residual floored 1e-6 | cross-pilot floored 1e-6 | **residual `max`'d with cross-pilot AWGN floor**, floored 1e-6 |
| I2 | nv tracks the demapper's EFFECTIVE post-EQ noise (not just pre-EQ thermal) | psk LLR §3.2 | residual vs FINAL smoothed H (the `fix/cfg16-nv-restore` reason) | cross-pilot (pre-EQ only; OK for PSK, |H|=1) | residual vs interpolated H = post-est EVM, floored at pre-EQ thermal |
| I3 | every cell MEASURED with a finite H | equalizer §3.1, mean_H §3.4, CSI §3.3 | yes | yes | yes (interp + freq-fill; degenerate→LS fallback) |
| I4 | mean(|H|) in the real-frame band (≈0.7–1.0), not artificially low | SKIP-H/SUBPEAK §3.4 | yes (DFT smooth preserves magnitude) | yes | yes (publishes pilot Y/X magnitude, no DFT attenuation) |
| I5 | std(|H|)/mean(|H|) reflects true selectivity | gearshift §3.5 | averages → UNDER-reads time selectivity | n/a | tracks fade → reads true time selectivity (intended improvement, advisory axis) |

**I2 nuance for TINTERP**: the harness prototype floored at the KNOWN Es/N0
`10^(-EsN0/10)`; production has no known Es/N0, so TINTERP floors at
`estimate_noise_from_pilot_pairs` (the cross-pilot differential), which measures the
SAME σ²/|X|² quantity. Empirically the production floor gives nv=0.00655 on the GOOD
cell where the harness known-EsN0 floor gave 0.00398 — same order, both well above the
over-confident region, both decode 5/5. The floor is the load-bearing invariant
(fade-estimator-prototypes.md §4.1).

---

## §6 WHAT `feat/fade-tinterp` CHANGES (the fix's assumption deltas)

1. **Adds a third producer** `TIME_INTERP` (`§2.1`). It does NOT modify `LS_channel_
   estimator`, `ZF_channel_estimator`, or `estimate_noise_from_pilot_pairs` — those
   are byte-identical. So when the FADE-tier gate is OFF (default), `channel_estimator`
   stays `LEAST_SQUARE`/`ZERO_FORCE` and NO consumer sees any change. **Proven**:
   gate-off A/B (`--test-ofdm-fine-timing`, sfo_grid harness PROD path) byte-identical
   to baseline `monitor` modulo wall-clock timing lines.
2. **When the gate is ON**, the only state delta is HOW H and nv are computed for WB LS
   configs. Walked every consumer above:
   - §3.1 MMSE erasure: H now per-cell (tracks fade), nv floored at cross-pilot AWGN →
     erasure correctly discounts deep nulls. ✓ (the win).
   - §3.2 psk LLR: nv honest (not collapsed) → no over-confidence. ✓ (I1/I2).
   - §3.3 CSI weight: per-cell |H|² now meaningful per carrier. ✓ improvement.
   - §3.4 mean_H gates: magnitude preserved (I4). ✓.
   - §3.5 selectivity: reads true time selectivity (I5); advisory, no hard gate. ✓.
   - §3.6 SKIP-VAR: nv in honest band, < ceiling, not collapsed. ✓ (I1).
3. **No consumer assumption is violated.** The single hazard (I1 nv-collapse) is the
   one the cross-pilot floor closes; without it the prototype smoke showed nv=1e-6
   (fade-estimator-prototypes.md §4.1) — carried here as the production floor.

---

## §7 Paired regression test (CLAUDE.md §Cross-layer regression tests)

`tools/test_fade_tinterp.py` drives the `MERCURY_SFO_GRID` harness on the three
Watterson profiles (md5-anchored binary) and asserts:
1. PROD LS decodes 0/K on MPG/GOOD (the documented production failure).
2. PRODUCTION `LS_channel_estimator_tinterp` (via `MERCURY_SFO_GRID_TINTERP_PROD=1`)
   decodes K/K on MPG/GOOD and ≥0.8·K on MPM/MOD — the promoted win (failing-first:
   the assert FAILS on the prod LS path, PASSES on TINTERP).
3. nv never collapses (>1e-5) on any profile (I1).
4. Gate-OFF byte-identity: the prod-else harness output md5 == baseline (default-off).
This test runs in-process (`-m PLOT_PASSBAND`), no IONOS, no RF.

---

## §8 Open questions / not-yet-validated [?]

- [?] MPP/POOR (1.0 Hz) remains a non-cross (Dy=3 under-samples the 1 Hz Doppler
  Nyquist — fade-estimator-prototypes.md §4 mechanism). Raising it needs Dy=2 ×
  TINTERP (costs wire) or the coherent tier — out of scope for this promotion.
- [?] HW confirmation: the lever is established in sim (no open HW question to
  ESTABLISH it; HW only CONFIRMS). A faithful fade loopback + bench run is the next
  gate before enabling the FADE tier by default.
- [?] The TINTERP cost vs LS on a CLEAN channel is shown byte-equal at decode (FLAT
  5/5 = 5/5) but the per-frame CPU cost (O(N·C) interp + a cross-pilot walk) has not
  been profiled against LS on a deployed RPi; the gate being default-off makes this
  non-blocking.

---

# §9 LIVE-PATH nv-COLLAPSE AUDIT (bench-pinned CFG16 decode-reliability lever)

Added 2026-06-11 for `sim/nvfix-live` (worktree `C:/Users/kamer/mercury_wt/nvfix-live`
off `monitor` @627c370). This section RE-AUDITS the producer/consumer registry of §2/§3
against the `627c370` line numbers (the §1-§8 cites were against `de428f6` and have
drifted) and adds the THREE live decode-path consumers the test-harness nvfix (a0e22c8)
never wired. The bench finding this serves: `bigblock_p3_hw/HW_BENCH10/PHASE0_VERDICT.json`.

## §9.0 The bench fact (do NOT re-litigate)
At the ~14.4 dB effective per-frame SNR analog ceiling on CLEAN WGN:40, the estimator
`nv` (`noise_variance_estimate`, median ~0.036) is ~5.83-6.3× BELOW the post-EQ
`measure_variance` `mvar` (median ~0.21), MAX ratio 6.9, NEVER > 8 (PHASE0_VERDICT.json
`FRAME_FAILURE_SIGNATURE` + `THREE_CANDIDATE_CAUSES.a_nv_collapse`). LLRs scale by `nv`
(`psk.cc:345` `LLR=(Dmin1−Dmin0)/eff_var`, `eff_var=max(nv,0.001)`) → over-confident →
CFG16 (32-QAM r0.875, ~1.6 dB under threshold) BP non-converges (iter=101, fails
CLUSTERED). CFG15 (16-QAM, ~1.4 dB margin) tolerates the SAME ~6× collapse and decodes
100%. On a FLAT channel `mvar` IS real post-EQ distortion (clip + Fe-Pi codec ENOB
quantization + analog-round-trip EVM = genuine per-symbol uncertainty) → the LLRs SHOULD
respect it. On a DISPERSIVE channel `mvar` is inflated by channel-estimate
REPRESENTATION error, NOT real per-symbol noise → substituting it OVER-softens and BREAKS
decode (the §9.4 misfire). The discriminator MUST be channel state (flat vs dispersive),
because no single ratio-K separates clean (5.83) from dispersive (>8).

## §9.1 PRODUCERS on `627c370` (re-cited; §2 semantics unchanged)
| state | producer | file:line @627c370 | notes |
|---|---|---|---|
| `nv` standard live path | `LS_channel_estimator` (WB CFG15/16 use `LEAST_SQUARE`, `telecom_system.cc:9717,9724-9766`) | nv written in `ofdm.cc` LS body (pilot-residual vs smoothed H, floored 1e-6) | the bench-10 path; per-pilot Y/X + DFT-smooth, so it DOES track per-carrier phase (unlike bigblock flat-ML §9.1b) |
| `nv` cross-pilot floor helper | `cl_ofdm::estimate_noise_from_pilot_pairs` | `ofdm.cc:1478-1521` | same-column adjacent-time pilot delta `|Hₐ−H_b|²/2`; measures PRE-EQ thermal floor; INVARIANT to frequency dispersion (delta is in TIME) → stays LOW on det-floor too (the §9.4 trap) |
| `mvar` (post-EQ) | `cl_ofdm::measure_variance` | `ofdm.cc:2396-2417` | `mean(|Y_eq[pilot] − X_pilot|²)` over PILOTs AFTER equalize; on flat = real post-EQ noise ≈ nv-band; on det-floor (flat-ML single-H̄ can't represent per-carrier phase) = large representation residual |
| `nv` bigblock flat-ML branch | `bigblock_rx_passband` flat-ML else-branch | `telecom_system.cc:8347-8361` | single H̄ = mean pilot Y/X; nv = pilot residual vs H̄; used when `last_channel_selectivity < 0.15` (flat) |
| `last_channel_selectivity` (the EXISTING flat/dispersive classifier) | standard live path | `telecom_system.cc:2766-2802` (`std(|H|)/mean(|H|)` over DATA cells), cached, getter `:10757` | **MAGNITUDE-only** → reads LOW on det-floor (det-floor `|A|≡1`, `:6696`) → CANNOT alone catch phase dispersion (§9.5) |

## §9.2 The THREE LIVE nv→LLR CONSUMERS that need the fix (raw nv, NO ratio guard)
All three feed `nv` straight to `psk.demod` as the LLR scale with ONLY a `<1e-9` floor:

| # | path / function | nv read @627c370 | demod call | guard today |
|---|---|---|---|---|
| C1 | STANDARD live ARQ decode `cl_telecom_system` per-frame RX (the bench-10 held-CFG16 path) | `variance = ofdm.noise_variance_estimate;` **`telecom_system.cc:2923`** | `psk.demod(...,variance)` `:2955` (CSI path) and `:2988` (non-CSI path) | NONE (raw nv; `eff_var` floor 0.001 is in psk.cc) |
| C2 | `bigblock_rx_passband` (live bigblock framing, default-off `bigblock_framing_enabled`, `:8145`; ref'd from `arq_commander.cc:13816`) | `cvar = ofdm.noise_variance_estimate; if(cvar<1e-9)cvar=1e-9;` **`telecom_system.cc:8526`** | `psk.demod(...,cvar)` `:8527` | `<1e-9` floor only |
| C3 | `bigblock_decode_from_wav` (WAV-decode mirror of C2) | same idiom **`telecom_system.cc:9074`** | `psk.demod(...,cvar)` `:9075` | `<1e-9` floor only |

The test-harness nvfix (a0e22c8) lives ONLY at `sfo_grid_test` **`telecom_system.cc:7195-7197`**
(`cvar=nv; if(nvfix && cvar < variance/K) cvar=variance;` K=8, env `MERCURY_SFO_GRID_NVFIX`,
collapse injected by `MERCURY_SFO_GRID_NV_FORCE` `:7182-7183`). It is NOT on C1/C2/C3 → it
CANNOT be A/B-tested on the bench (PHASE0_VERDICT `nvfix_wiring_assessment`). `mvar` is
ALREADY computed adjacent to C1 (`measure_var = ofdm.measure_variance(...)` `:2915`, logged
`[FRAME-NV]` `:2924`) — so C1 needs NO new producer call, only the gate.

## §9.3 CONSUMERS of the LLR the nv scales (downstream of the fix)
- `psk.demod` `psk.cc:299-350`: `eff_var=max(variance,var_floor=0.001)`; `LLR=(Dmin1−Dmin0)/eff_var`
  `:345`. Sole numeric consumer of the scalar this fix changes. `psk.demod_pas` `psk.cc:364`
  identical + a-priori prior (PAS path, CFG17 only).
- `ldpc.decode` (SPA): consumes the LLRs; over-confident → iter=101 non-converge (the symptom).
- CSI-weight multiply (`:2930-2983` C1, `:8528-8534` C2): scales the post-demod LLRs by
  normalized `|H_k|²`; reads H, NOT nv → ORTHOGONAL to this fix (the fix changes the scalar
  BEFORE this multiply; the multiply is mean-normalized so it does not re-introduce nv).

## §9.4 The DISPERSIVE MISFIRE (the constraint the gate must respect) — MEASURED
`data-flow-cfg17-shaped-64qam.md §7.1`: on the SFO-GRID det-floor (`MERCURY_SFO_GRID_CHAN=1`,
Schroeder all-pass `|A|≡1` phase-dispersive, `telecom_system.cc:6693-6699`) at 18 dB
seed12345 TINTERP-turbo: **NVFIX=0 → 7/7, NVFIX=1 → 0/7**; `stacknvfix` floors 0.0 across
ALL 14-22 dB / 6 seeds. MECHANISM: nv is LEGITIMATELY low (0.0093 = the cross-pilot thermal
floor, correct) while `mvar` is LARGE because the flat-ML/LS single-window estimate cannot
represent the per-carrier phase → `mvar` carries channel-est REPRESENTATION error, not real
per-symbol noise. The K=8 ratio-gate `nv < mvar/8` TRIPS → substitutes the large `mvar` →
over-softens → decode dies. **This is why a ratio-K alone is UNSAFE**: the clean case I must
FIX has ratio ~5.83 (< 8) and the dispersive case I must NOT trip has ratio > 8 — to fire on
clean, K would have to drop below 5.83, which then ALSO fires on dispersive → the misfire.
The two regimes are NOT separable in ratio space; they ARE separable by channel state.

## §9.5 The discriminator problem (why magnitude-selectivity alone is insufficient)
`last_channel_selectivity = std(|H|)/mean(|H|)` (`:2766-2802`) is the EXISTING flat/dispersive
classifier (threshold 0.15 at `:8338`: `sparse2d = sel>=0.15` ⇒ "selective→sparse-2D,
flat→flat-ML"). BUT the det-floor has `|A|≡1` (flat MAGNITUDE, phase-only dispersion,
`:6696`) → its magnitude CV reads LOW → `last_channel_selectivity` would (wrongly) call it
FLAT and the fix would fire → misfire. Magnitude selectivity catches the TWO-RAY/fade case
(`CHAN=2`, deep `|H|` nulls) but NOT the phase-only det-floor. A robust gate needs a signal
sensitive to PHASE dispersion / channel-est representation error, OR a conservative composite
(see §10.2). On the live LS path (C1) the per-pilot+DFT-smooth estimate tracks phase better
than the bigblock flat-ML, so the live-path det-floor `mvar` inflation is SMALLER than the
flat-ML harness case — but this must be MEASURED on the live path, not assumed (§10.5 [?]).

## §9.6 VALID STATES (delta vs §4) + INVARIANTS at the fix point
The fix reads `nv` and `mvar` AFTER an estimator has run and AFTER `channel_equalizer`
(both produced by `:2914-2915` before the C1 read at `:2923`). Pre-producer default-init
(`nv=0.01`, §4.1) is never seen at C1/C2/C3 (an estimator always runs first; SKIP-H/SKIP-VAR/
SUBPEAK gates at `:2816/2834/2884` `continue` BEFORE reaching the demod). INVARIANTS this fix
must preserve (the §5 table, restated for the demap-scalar):
- I1 nv>0 (≥1e-9): preserved — substituting `mvar` (also >0, post-EQ residual) keeps it >0.
- I-NEW (the fix's load-bearing invariant): substitute toward `mvar` ONLY when `mvar`
  represents REAL per-symbol noise (flat/clean), NEVER when `mvar` is channel-est
  representation error (dispersive). This is the §9.4 constraint as an invariant.
- ORTHOGONALITY: the fix changes ONLY the demap scalar at C1/C2/C3. It does NOT touch
  `ofdm.noise_variance_estimate` itself → the MMSE-erasure (`§3.1` `ofdm.cc` equalizer,
  reads nv), the SKIP-VAR gate (`:2884`, reads nv), the SNR report, and the gearshift
  selectivity axis are ALL UNCHANGED (matches a0e22c8's scope note: "demap_variance feeds
  ONLY the two psk.demod LLR calls"). The equalizer already ran (`:2914`) before C1 reads nv,
  so even a same-frame nv mutation could not retro-affect it — but the fix uses a LOCAL
  `cvar`, mutating nothing shared, which is the safe form.

# §10 DESIGN — channel-gated nv-collapse fix (the lever)

## §10.1 The fix (formula + gate)
At each of C1/C2/C3, replace the raw `cvar = nv` demap scalar with a CHANNEL-GATED
substitution computed from the ALREADY-AVAILABLE `nv` and `mvar`:

```
double demap_var = nv;                               // default = today's behavior
if (channel_is_flat && nv < mvar / K_LIVE)           // gate: flat AND collapse present
    demap_var = mvar;                                // de-soften the over-confident LLRs
if (demap_var < 1e-9) demap_var = 1e-9;              // existing floor, unchanged
// ... psk.demod(..., demap_var)
```

Env-gated `MERCURY_NV_COLLAPSE_FIX` (default 0 = OFF = byte-identical raw nv) so it is
mergeable default-off and A/B-able live on the bench WITHOUT the sfo_grid harness. The
`channel_is_flat` gate is the new element vs a0e22c8.

## §10.2 The gate signal — `channel_is_flat` (the design decision)
The det-floor `|A|≡1` proves magnitude-selectivity (`last_channel_selectivity`) is NECESSARY
but NOT SUFFICIENT (§9.5). Design the gate as a CONSERVATIVE composite that is flat ONLY when
BOTH magnitude AND phase-dispersion proxies say flat:

1. **Magnitude-flat**: `last_channel_selectivity >= 0` AND `< SEL_FLAT` (reuse the existing
   0.15 boundary from `:8338`, so the classifier is consistent with the bigblock estimator
   switch). `< 0` sentinel (no prior estimate) ⇒ treat as NOT-flat (robust, matches `:8335`).
2. **Phase-flat (the det-floor guard)**: bound the mvar/nv ratio FROM ABOVE. The clean-WGN
   collapse caps at ratio ≤ 6.9 (NEVER > 8, PHASE0_VERDICT `a_nv_collapse`); the det-floor
   misfire is at ratio > 8 (§9.4). So require `mvar/nv <= R_MAX` with `R_MAX ≈ 7.5` (above the
   clean max 6.9, below the dispersive >8). This UPPER bound is what makes the gate REJECT the
   dispersive representation-error regime: when mvar is inflated by phase dispersion the ratio
   blows past R_MAX → gate stays OFF → no substitution → no misfire. Combined with the
   magnitude-flat check it is belt-and-suspenders: det-floor fails the ratio bound; two-ray/
   fade fails the magnitude bound.

So `channel_is_flat := (0 <= last_channel_selectivity < SEL_FLAT) && (mvar <= R_MAX * nv)`.

## §10.3 The substitution trigger (lower bound) — does K retuning matter once gated?
With the channel-flat gate carrying the dispersive rejection, the LOWER trigger `nv < mvar/K_LIVE`
no longer has to STRADDLE the clean/dispersive boundary — its ONLY job is "is there a collapse
worth fixing on a channel already known flat." Pick `K_LIVE` so the gate FIRES on the bench
clean-WGN regime: the clean ratio is 5.83-6.3 (median), so `K_LIVE = 4` fires whenever
`mvar/nv > 4` (true for the whole 5.83-6.9 clean band) and is a NO-OP on a genuinely healthy
flat frame (ratio ~1, mvar≈nv). **So YES, K must be retuned from 8 to ~4 for the clean-WGN ~6×
regime** (PHASE0_VERDICT "needs re-tuning K=4 or K=2"). The window is then `4 < mvar/nv <= 7.5`
on a magnitude-flat channel = exactly the clean-WGN collapse band, and ONLY that band.
Restated: K_LIVE (=4, lower trigger) and R_MAX (=7.5, upper dispersive-reject bound) together
carve the clean-collapse window; the magnitude-flat check is the third, independent guard. The
flat-gate alone does NOT suffice without K retuning, because K=8 never fires at 6× (the bench
"SECOND_PROBLEM"); and K=4 alone (no R_MAX, no magnitude gate) WOULD misfire on det-floor
(ratio>8>4). All three are needed; none is redundant against the two failure modes.

## §10.4 Why this fixes clean CFG16 (5.83) WITHOUT the dispersive misfire (>8)
- CLEAN WGN CFG16 (the FIX target): magnitude-flat ✓ (selectivity low, true flat); ratio
  5.83 ∈ (4, 7.5] ✓ → gate FIRES → demap_var = mvar (~0.21 vs nv ~0.036) → LLRs de-softened
  ~6× → BP no longer over-confident → the ~1.6 dB-marginal frames that clustered at iter=101
  get a fighting chance to converge. This recovers part of the margin gap WITHOUT changing the
  14.4 dB EVM (it cannot — only the LLR softness changes; if the EVM floor is the hard wall the
  benefit is bounded, PHASE0_VERDICT `c_genuine_analog_enob_floor` — bench arbitrates §10.5).
- DET-FLOOR / dispersive (the MISFIRE to AVOID): ratio > 8 > R_MAX 7.5 → upper bound REJECTS →
  gate OFF → demap_var = nv (unchanged) → decode stays 7/7 (reproduces NVFIX-OFF). Even if the
  live-LS det-floor ratio were < 8, the magnitude/phase composite is the second guard. The
  documented 7/7→0/7 misfire CANNOT recur because the substitution never fires there.
- FREQUENCY-SELECTIVE fade / two-ray: magnitude-flat ✗ (selectivity ≥ 0.15, deep `|H|` nulls)
  → gate OFF → unchanged. Matches a0e22c8's no-regression proof on `fsel amp0.6 dly64`.
- HEALTHY flat (high SNR, no collapse): ratio ~1 < K_LIVE 4 → lower trigger never met → NO-OP
  → byte-identical. So default-ON would still be a NO-OP except in the collapse band; default-
  OFF env-gate makes the merge strictly safe.

## §10.5 Failing-first test + validation plan (Phase 2, NOT this phase)
- FAILING-FIRST: extend the `--test-cfg17` CELL-C pattern (NV_FORCE-injected collapse,
  `main.cc:502-515`) into a CELL that drives the LIVE C1 path (not just sfo_grid) and asserts
  the gated fix RECOVERS a collapsed-nv flat frame (fail-before raw-nv 2/7 → pass-after 7/7)
  AND is a NO-OP on the det-floor (CHAN=1 must stay 7/7 with the fix ON = the misfire guard).
- SIM SWEEP (the sim-only hot-wash, no IONOS): SFO-GRID coded-BER `MERCURY_SFO_GRID + _CODED +
  _CHAN`, K_LIVE/R_MAX sweep across CHAN=0 (flat, must IMPROVE or hold) / CHAN=1 (det-floor,
  must NOT regress) / CHAN=3 (Watterson, must NOT regress), 6 seeds. The clean ~6× collapse is
  NOT reproduced in-sim natively (it is an HW analog-EVM artifact) so NV_FORCE injects it for
  the FIX-fires direction; the NO-MISFIRE direction is native (real det-floor/fade ratios).
- BYTE-IDENTITY: `--test-climb-engine` must stay ALL-PASS with the env unset (default-off),
  and the SFO-GRID render md5 must equal monitor 627c370 with `MERCURY_NV_COLLAPSE_FIX` unset.
- [?] OPEN: the live-LS det-floor mvar/nv ratio has NOT been measured on C1 (only the bigblock
  flat-ML harness ratio is documented, §9.5). If it sits < 8 on the live path, R_MAX may need
  tightening or the magnitude/phase composite leaned on harder — MEASURE before committing K.
- [?] OPEN: whether the benefit survives the 14.4 dB analog ENOB floor is HW-only-confirmable
  (PHASE0_VERDICT `lever_for_(1)`); sim proves correctness + no-misfire, the bench proves the
  decode-rate lift. This lever is necessary-not-sufficient (the turnaround fix is the
  independent second requirement for the end-to-end CFG16 beat).

# §11 IMPLEMENTATION + HOT-WASH (Phase 2, BUILT + MEASURED on 627c370 worktree nvfix-live)

## §11.1 What was wired
A file-local helper `live_nvfix_demap_var(nv, mvar, selectivity)` (telecom_system.cc, just
after the externs at the top) encodes the §10 composite gate verbatim:
`channel_is_flat := (0<=sel<SEL_FLAT) && (mvar<=R_MAX*nv)`, fires `demap_var=mvar` only when
ALSO `nv<mvar/K_LIVE`. Env-gated `MERCURY_LIVE_NVFIX` (default 0 = raw nv = byte-identical).
Knobs `MERCURY_LIVE_NVFIX_{K,RMAX,SEL}` default to {4, 7.5, 0.15}. A second helper
`live_nvfix_selectivity(ofdm, nsym)` computes std|H|/mean|H| over DATA cells (the SAME
definition as :2820-2846) for the bigblock paths that do NOT update last_channel_selectivity.
Wired at all THREE live nv→LLR sites:
- C1 (standard live ARQ decode, the bench-10 held-CFG16 path): replaces
  `variance = ofdm.noise_variance_estimate` (~:3002); mvar(=measure_var) + member
  last_channel_selectivity already in scope. [FRAME-NV] log line PRESERVED VERBATIM (the
  fix's effect emitted on a SEPARATE gated [LIVE-NVFIX] line → stdout byte-identical when off).
- C2 (bigblock_rx_passband, ~:8530): mvar=`measure_variance(eq.data())`, sel local.
- C3 (bigblock_decode_from_wav, ~:9080): same idiom.
Plus the sfo_grid_test coded harness (~:7286): when MERCURY_LIVE_NVFIX is set it routes the
SAME helper, so the in-process failing-first test drives the REAL gate (NOT a parallel copy).

## §11.2 OPEN-question §10.5(a) CLOSED — the live det-floor ratio is FAR above R_MAX
MEASURED on CHAN=1 (Schroeder all-pass, the documented misfire): with the LS estimator the
gate reads `sel=0.181` (≥0.15) → magnitude gate REJECTS (`fired=0`); with the TINTERP-seed
turbo estimator active (the EXACT memory misfire where it decodes 6/6) the live ratio is
`mvar/nv = 474` (mvar 4.24 inflated by phase-dispersion representation error, nv 0.0089) ≫
R_MAX 7.5 → upper-bound REJECTS (`fired=0`) → decode stays 6/6. The naive K=8 lever WOULD
fire (`0.0089 < 4.24/8`) and over-soften to break it; the channel-gated fix does NOT. So
R_MAX=7.5 is safe on the live path; the §10.5(a) "if it dips < 8" risk does not materialize
(474 ≫ 8). K_LIVE=4 confirmed: on clean WGN at the waterfall edge the ratio is 5.5–6.9 → fires.

## §11.3 The failing-first test (`--test-live-nvfix`, main.cc) — ALL PASS
- CELL-A (CLEAN FLAT + injected ~6× collapse, CFG16 32-QAM, ESN0 swept {15.0,15.25,15.5} at
  the in-sim waterfall edge, NV_FORCE=0.0050 → ratio ~5.84 ∈ (4,7.5]): bare FAILS, fix
  RECOVERS. AUTHORITATIVE clean-process A/B (fresh process per arm) = bare 35/90 → fix 64/90 =
  **+32% net, fix≥bare EVERY cell** (3 ESN0 × 5 seeds). The in-process multi-cell test shares
  the global srand/rand cl_awgn uses (awgn.cc:37,87) so its net is a DETERMINISTIC LOWER BOUND
  (+12/72 = +17%); assertion is on the aggregate margin (≥+9), robust to that residue.
- CELL-B (MISFIRE GUARD, det-floor + TINTERP-turbo, the seed12345 @18dB trap): bare 18/18 →
  fix 18/18, gate `fired=0` (ratio 474 > R_MAX) → the documented 7/7→0/7 misfire CANNOT recur.
- CELL-C (healthy flat, no collapse): fix == bare 6/6 (ratio~1 < K_LIVE → strict no-op).

## §11.4 Regression + byte-identity (all GREEN)
- DEFAULT-OFF BYTE-IDENTICAL: 18 SFO-GRID coded renders (CHAN 0/1/2 × ESN0 14/16/18 × 2 seeds)
  are md5-IDENTICAL between the fix binary (MERCURY_LIVE_NVFIX unset) and a freshly-built
  pristine 627c370. The fix is a strict no-op when off.
- DISPERSIVE NO-REGRESSION (native, no forced collapse): CHAN=1 det-floor, CHAN=2 two-ray,
  CHAN=3 Watterson — fix OFF == fix ON for every seed (decode counts identical).
- `--test-climb-engine` ALL PASS (no gearshift/D2/D3 perturbation).
- `--test-cfg17` ALL PASS (the existing harness K=8 lever is untouched).

## §11.5 READY-FOR-BENCH
The live nvfix is the bench-pinned beat lever (PHASE0_VERDICT `lever_for_(1)`). Sim proves
(1) correctness — recovers clean collapsed-nv CFG16 by de-softening the over-confident LLRs;
(2) no-misfire — the det-floor/fade misfire CANNOT recur (R_MAX rejects ratio 474). What sim
CANNOT prove (§10.5(b), HW-only): whether the de-softening lift survives the real 14.4 dB
analog ENOB floor → bench-10 validates the HW held-CFG16 decode rises toward ~100% byte-
faithful with MERCURY_LIVE_NVFIX=1 vs the 82–96% bare baseline. Necessary-not-sufficient: the
reverse-ACK turnaround fix is the independent second requirement for the end-to-end beat.

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

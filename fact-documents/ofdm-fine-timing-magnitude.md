# OFDM fine-timing: phase-fragile real-dot-product → phase-invariant magnitude metric

**Status:** FIX SHIPPED on branch `fix/ofdm-fine-timing-magnitude` (base `f6e9631`).
SIM-validated (fail-before / pass-after, §4). NOT pushed, NOT merged, NO hardware run
yet (the next steps — adversarial review → DIRECTION-validate in sim → HW magnitude at
wgn0 — are separate and not done here).

**Date:** 2026-06-04. **Worktree:** `x:/Storage/Documents/hermes and mercury/mercury-ftr-wt`.
Build `bash build.sh o3`. Targeted test: `mercury.exe --test-ofdm-fine-timing` (fast,
deterministic — does NOT run the long stochastic `--test` Monte-Carlo BER suite).
**Conventions:** numbered sections (§N), `file.cc:line` citations, `[?]` for unknowns.

---

## §1 The bug — phase-fragile fine timing at the ROBUST→CONFIG_0 PHY switch

`cl_ofdm::time_sync_preamble_with_metric` (`source/physical_layer/ofdm.cc:2406`) is the
FINE per-trial timing refiner. It scans candidate sample positions and returns the
sample-precise `delay` the production receive path uses to place the FFT window
(`telecom_system.cc:2078-2082`).

The original loop scored each candidate on the **PHASE-SENSITIVE real projection** of the
preamble self-correlation:

```
corss_corr += a.real()*b.real() + a.imag()*b.imag();   // Re(conj(a)*b) = |a||b|cos(theta)
... corss_corr = corss_corr / sqrt(norm_a*norm_b);     // [-1,1] cosine
```

`Re(conj(a)·b) = |a||b|cos(theta)`. Under the residual CFO the production path leaves after
Moose sync (~±20 Hz, can be larger at the ROBUST→CONFIG_0 PHY switch where the channel
estimate restarts), the self-correlation phase `theta` drifts toward ±90°, the cosine
collapses toward 0 (and can go negative), and the peak position is mis-selected by ≥1 OFDM
symbol. The mistimed FFT window misaligns the pilots → `mean_H` collapses to ~0.29 → the
SKIP-H gate (`telecom_system.cc`, threshold 0.30) rejects the frame before LDPC → 0 decode.
This is the ~34× deep-cell lever at the ROBUST→CONFIG_0 transition.

## §2 The fix — per-lag magnitude coefficients, combined incoherently

The COARSE detector `time_sync_preamble_halfsym` (`ofdm.cc:2568-2664`) is already
phase-invariant: it scores `M = |P|²/(A²·R)`, `P = Σ conj(r[m])·r[m+L]` — but it uses a
**single, consistent lag** L (the repetition period Nfft/nIS). All contributions to `P`
share the same CFO phase ramp `e^{j2πf·L/fs}`, so they add coherently and `|P|²` is large
regardless of the absolute phase → CFO-invariant.

The fine timer is structurally different: it correlates **TWO different lags** —
1. the GI / cyclic-prefix lag = `Nfft` samples (CP vs FFT tail), and
2. the repetition-period lag = `L = Nfft/nIS` samples.

**The naive magnitude fix (sum both lags into ONE complex P, then |P|²) is WRONG** and was
the form left uncommitted by the prior agent. Under CFO the two lag families accrue
*different* phase ramps (Δt ∝ lag), so summing them before `|·|²` lets them partially
**cancel** — reintroducing exactly the ±symbol mistiming the fix was meant to remove
(measured: drift 6418 interp samples at CFO=40 Hz, §4).

**Correct fix (`ofdm.cc:~2484-2565`):** accumulate the two lag families in SEPARATE complex
accumulators (`Pg`=GI lag, `Pr`=repetition lag), each internally coherent / phase-invariant;
form each one's normalized magnitude coefficient `|P|²/(A·R)` (each bounded [0,1] by
Cauchy-Schwarz); and combine them **incoherently** as the average of the two coefficients
(still [0,1]). A lag family with ~zero energy (e.g. nIS=1 → no repetition pairs) is dropped
from the average rather than dividing by zero. This is noncoherent combining across the two
lag families — it keeps both timing sources (GI = sample precision, repetition = strong
period lock) without the cross-lag CFO cancellation. References: Schmidl & Cox 1997;
Wilson & Shang arXiv:2010.00762; FreeDV/STANAG/liquid-dsp all use magnitude fine-timing.

The `norm_a < 0.001 || norm_b < 0.001` silence gate is preserved (applied to the COMBINED
per-half energies Ag+Ar / Rg+Rr) — the VB-Cable digital-silence false-peak suppressor.

## §3 The `coarse_metric` [0,1] ARQ contract — PRESERVED (cross-layer audit)

The fine timer's `result.correlation` changed scale (old: real cosine [-1,1]; new: magnitude
average [0,1]). Cross-layer audit confirms **no contract breaks**:

1. **Producers of the fine `result`:** only `time_sync_preamble_with_metric` itself.
2. **Consumers of the fine `result`:** the ONLY caller is `telecom_system.cc:2078`, and it
   reads ONLY `fine_result.delay` (line 2082: `receive_stats.delay = s8_win_start +
   fine_result.delay;`). `fine_result.correlation` is **never read** anywhere in production
   (verified by grep of `time_sync_preamble_with_metric`). So the scale change on the fine
   `.correlation` is inert.
3. **The load-bearing `receive_stats.coarse_metric` (the ARQ contract, consumed at
   `arq_common.cc:6575, 6696, 6737, 6778, 6806-6807` with thresholds 0.10/0.15/0.30/0.5)
   is fed EXCLUSIVELY by the COARSE detectors** — `telecom_system.cc:1253` (verify),
   `:1399` (matched), `:1595`/`:1765`/`:2921` (retry). It is NEVER assigned from the fine
   timer. The fine fix touches neither the coarse detectors nor `coarse_metric`. Contract
   intact, scale unchanged.

## §3.5 New test-observability field `receive_stats.mean_H`

`st_receive_stats.mean_H` added (`telecom_system.h:101`), written write-once-per-trial at
`telecom_system.cc:2420` to the same `mean_H` the SKIP-H gate keys on; default -1.0
(`telecom_system.cc:853`). **No control-flow effect** — read only by the §4 unit tests so
they can assert on the gate-deciding quantity directly. Producer: one site. Consumers: tests
only.

## §4 Fail-before / pass-after (SIM, `--test-ofdm-fine-timing`)

Four tests in `mfsk_ctrl_codec_tests.cc` §22, run via the focused `run_ofdm_fine_timing_tests()`
entry point (main.cc `--test-ofdm-fine-timing`, returns before any audio/GUI init):

- `..._direct_clean` / `..._direct_cfo` — drive the fine timer in isolation
  (`passband_to_baseband` → `FIR_rx_time_sync` round-trip), CFO injected via the production
  `test_tx_carrier_offset` hook. Assert the CFO-detected peak stays within ±Ngi of the clean
  peak.
- `..._clean_no_regression` / `..._cfo_cliff` — drive the REAL `receive_byte` acquisition
  path (`ofdm_forced_delay=-1`), AWGN at SNR3k, read back production `receive_stats.delay` +
  `mean_H`. Assert fine timing within ±Ngi AND mean_H ≥ 0.30 (clears SKIP-H).

| Build | direct_clean | direct_cfo (40 Hz) | clean_norereg | cfo_cliff (30 Hz, SNR3k 8) |
|-------|--------------|--------------------|---------------|-----------------------------|
| **Originally-committed broken (single-P |P|²)** | OK | **FAIL** drift=6418 | OK | **FAIL** 5/10 seeds, mean_H=0.292 |
| **Pre-fix phase-sensitive (real-dot)** | OK | OK (drift 2, corr collapses 0.31) | OK | **FAIL** 2/10 seeds, mean_H=0.284 |
| **Corrected per-lag fix (this branch)** | OK corr=0.99 | **OK drift=0** corr=0.99 | OK derr=0 mean_H=0.998 | **OK 10/10 seeds**, worst derr=2, min mean_H=0.307 |

Both phase-fragile forms FAIL the production-path `cfo_cliff` test (mean_H < 0.30 → SKIP-H);
the corrected fix recovers all 10 seeds and lifts min mean_H above the gate. The direct
keystone cleanly separates the originally-committed single-P magnitude form (drift 6418).

`--test-climb-engine` also passes (0 failures) — no ARQ/gearshift regression.

## §5 Open / not-done here
- [ ] Adversarial review of the per-lag combining math.
- [ ] DIRECTION-validate in the (now-fast) sim.
- [ ] Hardware magnitude check at wgn0 on the IONOS testbed.

## §6 FIX A — matched-template fine timing at site-8 (2026-06-27, branch staging/wb-acq-matched)

The self-autocorrelation fine timer (`time_sync_preamble_with_metric`) is MEAN-limited (Schmidl-Cox:
metric mean is SNR-only, length buys variance not floor). For the lead/tail frames of a forward
CONFIG_0 batch it ties ~equally on the WRONG adjacent-symbol boundary → mistimed FFT window →
inflated pilot-residual nv → SKIP-VAR pre-LDPC reject (the steady `0x0c` drop; SNR-invariant
WGN:40==WGN:35). Root cause + sim validation: `ofdm-data-acquisition-fix-plan.md` §13 (matched
resolves 66-91% of the self-autocorr failures at the wgn0 operating point, sub-sample median |err|).

### §6.1 The change
- **Template build re-enabled**: `telecom_system.cc` OFDM `ofdm_corr_template` build was `#if 0`
  (comment "using Schmidl-Cox autocorrelation"); now LIVE in the non-MFSK config branch. Builds the
  FIR-round-tripped, pre-equalized preamble template at each config load. The `[TMPL-PREEQ]` /
  `[TMPL-INIT]` diagnostics are `g_verbose`-gated; the one-line `[PHY] OFDM corr template` summary
  stays (mirrors the MFSK template log).
- **Wired at site-8 fine stage**: after the self-autocorr `time_sync_preamble_with_metric` call,
  for OFDM-tier configs with the template built, run `time_sync_preamble_matched` over the SAME
  already-narrow fine slice (`baseband_data_fine_slice[s8_interior]`, length `s8_search_len`). The
  slice is seeded near the coarse position (`(pream_symb_loc-1)*Nofdm*M`), so matched is implicitly
  seeded — the codec2 narrow-re-est pattern, avoiding the deep-cell wrong-frame outliers a
  full-buffer matched search showed (§13.2). PREFER the matched delay when its metric clears the
  CONFIDENCE FLOOR `0.5·matched_nsymb`; else KEEP the self-autocorr (the fallback / N-th-peak retry
  source on `receive_stats.sync_trials`).

### §6.2 §5 / §3 cross-layer audit (`receive_stats.delay` shared site)
- **coarse_metric [0,1] ARQ contract** (consumers `arq_common.cc:6453/6574/6615`): UNTOUCHED. The
  matched delay feeds `receive_stats.delay` ONLY. `coarse_metric` is fed exclusively by the coarse
  detectors (the fine metric is documented as consumed by NO caller, `ofdm.cc` fine-fn header). NO
  threshold / mean_H / SKIP-VAR retune — matched is amplitude-independent (Cauchy-Schwarz).
- **Trial loop / subpeak recovery / SKIP-H retry**: all read `receive_stats.delay`. Matched is an
  IN-PLACE refinement of the same delay the with_metric call produced; when matched is low-confidence
  the delay is byte-identical to the self-autocorr (fallthrough), so the N-th-peak retry semantics
  (`location_to_return=sync_trials`) and the subpeak recovery are preserved unchanged.
- **ofdm_batch_active predict/verify drift IIR** (`ofdm_drift_per_frame`): consumes the chosen delay;
  matched only refines it within the already-bounded slice, so the IIR converges on the same scale.
- **CONFIDENCE GATE protects the deep cells**: §12 proved the wall at the deep cells is DECODE, not
  timing. There matched's metric is unmet → the gate keeps the self-autocorr → no regression of a
  decode-walled cell. The win is at the decode-CAPABLE wgn0 cell (decode proven by seq 2,3).

### §6.3 Fail-before / pass-after (`--test`, `test_ofdm_matched_template_fine_timing`)
`test_ofdm_matched_template_fine_timing` (in the `--test-ofdm-fine-timing` group): (1) FAIL-BEFORE /
PASS-AFTER on the template BUILD — asserts `ofdm_corr_template != NULL` + energy>0 after
`load_configuration(CONFIG_0)`; FAILS with the `#if 0` build disabled (template NULL → matched dead).
(2) FIDELITY PROBE (diagnostic): correlates the template against the REAL pre-equalized TX preamble
(`transmit_byte` path) at the known frame start and reports per-symbol Cauchy-Schwarz.

### §6.5 STATUS — template fidelity PROVEN; production override PARKED on a slice-coordinate blocker
The fidelity probe reports **per_sym_cs = 0.998 (total_cs 3.992 / 4 symbols)** against the real
pre-equalized TX preamble → the re-enabled `#if 0` template build is CORRECT and byte-faithful to the
TX (it ships, default-on; it also powers the existing `arq_common.cc` TX-SELFTEST). HOWEVER the
PRODUCTION matched-fine OVERRIDE at site-8 is **DEFAULT-OFF** behind `MERCURY_WB_MATCHED_FINE`:
- With the override ON, `test_ofdm_fine_timing_magnitude_clean_no_regression` (the REAL TX roundtrip,
  clean SNR3k=20) REGRESSES — `delay_err=1240` (exactly 1 symbol), `mean_H 0.998→0.302`, CRC fail.
- Trace: the self-autocorr returns the CORRECT slice offset (`self=1240`); but the matched detector,
  scanning the production `baseband_data_fine_slice`, finds its coarse global-max at slice-offset **0**
  with metric ~2.965, NOT at 1240 (which scores LESS). With per-symbol fidelity 0.998 proven, this is
  NOT a template-fidelity failure — it is a SLICE-COORDINATE / search-geometry mislocation: the
  production slice (mixed at `carrier_frequency + coarse_freq_offset`, guard-margin-padded, preamble
  starting ~1 symbol into the searched region) presents a 1-symbol-early plateau the matched coarse
  global-max picks, distinct from the clean isolated-buffer geometry the §13.2 SHADOW validator used.
  Seeding matched ±1 symbol around `self` did not help because `self` sits exactly 1 symbol into the
  slice → `seed = self - 1sym = 0` → the window still spans the early plateau.
- DISPOSITION (CLAUDE.md §2/§3: 3-fail STOP, no untested fix default-on): the override is a temporary,
  actively-driven A/B gate. FOLLOW-UP = align the matched search anchor to the slice's true preamble
  region (use `s8_win_start`/`pream_symb_loc` as the absolute anchor, not the relative `self`, and
  bound the coarse search to ±Ngi around the coarse-derived expected symbol so the early plateau is
  outside the search), then re-run the clean no-regression + the cfo cliff + the bench post-cross.
  The DURABLE win (template build) is landed; only the override wiring remains.

### §6.4 NOT resolved here / bench confirmation owed
Per the MEMORY bench-claim guard: the structural fix is sim/static-validated; HW confirmation (md5
the deployed binary FIRST) must measure the post-cross full-decode rate (2/6→6/6 or ≥5/6) and the
SKIP-VAR aborts/cell collapsing, with the steady `0x0c` bitmap filling toward `0x3f`. NOT run here.

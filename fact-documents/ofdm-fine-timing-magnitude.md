# OFDM fine-timing: phase-fragile real-dot-product → phase-invariant magnitude metric

**Status:** ~~FIX SHIPPED on branch `fix/ofdm-fine-timing-magnitude` (base `f6e9631`).
SIM-validated (fail-before / pass-after, §4). NOT pushed, NOT merged, NO hardware run
yet~~ **MERGED TO MAINLINE (re-grounded against HEAD 2026-07-01).** The per-lag `Pg`/`Pr`
noncoherent-combining fine timer is in the checked-out source:
`cl_ofdm::time_sync_preamble_with_metric()` now at **`ofdm.cc:2897`** (old anchor `:2406`),
with the two-per-lag magnitude-coefficient combining documented/implemented at
**`ofdm.cc:2903-3067`** (the "average of two per-lag magnitude coefficients |P|²/(A²·R)"
header at `:2903`, the noncoherent-combining form at `:2994-3003`). **[?]** The §5 next
steps (adversarial review, DIRECTION-validate in sim, HW magnitude at wgn0) were not
re-verified in this pass — status of those remains as recorded in §5.

> **ANCHOR DRIFT (verified HEAD):** function `:2406` → **`:2897`**; per-lag fix body
> `:~2484-2565` → **`:~3004-3067`**; coarse detector `time_sync_preamble_halfsym`
> `:2568-2664` and the `telecom_system.cc` consumer/`mean_H` anchors (`:2078-2082`,
> `:2420`) below are pre-drift — verify before trusting.

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

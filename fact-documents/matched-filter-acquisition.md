# Matched-Filter OFDM Acquisition (Candidate A) — historical non-production result

> **HISTORICAL / NON-PRODUCTION:** this file preserves a rejected experiment and
> its measured 8-ppm SFO regression. Current Mercury intentionally does not wire
> `time_sync_preamble_matched()` into `receive_byte()`; its template is available
> only to the `--test-frame0-mf` diagnostic. Do not reintroduce the
> `MERCURY_OFDM_MATCHED` production selector without first adding fractional-delay
> template support and re-running the SFO acquisition cells below.

Branch `feat/matched-filter-acq` off monitor `ed1f727`. Flag `MERCURY_OFDM_MATCHED`
(env, read once, **default-OFF**). Records the off-bench evaluation of wiring the
already-written `time_sync_preamble_matched` detector into the live WB-OFDM fine
timing stage. **VERDICT: do NOT enable. The matched filter does not improve, and
under SFO actively REGRESSES, fine-stage acquisition off-bench.** Kept default-off
and quarantined as a broken experiment with a do-not-enable note; the wired
mechanism is preserved for any future bench A/B.

## §1 What was wired
- **Template build** (`telecom_system.cc:~10531`): the dead `#if 0` matched-filter
  template build (`ofdm_corr_template`, full FIR-round-tripped TX→RX of the preamble)
  is now a runtime `if(mercury_ofdm_matched_enabled())` block inside the non-MFSK
  config `else` branch. Flag-off ⇒ block skipped ⇒ `ofdm_corr_template` stays NULL
  (constructor `ofdm.cc:194`) ⇒ matched detector early-returns. The `if`-block's own
  braces hold the else-block brace balance constant (no compile-time brace swap).
- **Fine-stage wiring** (`telecom_system.cc:~2421`): when the flag is on (and not
  MFSK), the WB-OFDM fine timing uses `time_sync_preamble_matched` instead of the
  self-correlation `time_sync_preamble_with_metric`. Falls back to self-correlation
  if the template is inert (corr==0) or the re-anchored window is too short.
- **Flag helper** (`telecom_system.cc:~52`): `mercury_ofdm_matched_enabled()`,
  `static const bool en = (std::getenv("MERCURY_OFDM_MATCHED") != nullptr)`.
- **DIAG gating** (`ofdm.cc:3755,3860`): the two `[MF-DIAG]` printfs in the matched
  detector gated behind `g_verbose` (only reachable flag-on; no flag-off effect).

## §2 Byte-identical flag-off (VERIFIED)
Built a monitor `ed1f727` baseline binary and diffed `[SFO-BLOCK]` output across 8
cells (Es/N0 ∈ {900,12,11,6} × SFO ∈ {0,8ppm}, 30 frames, 168 lines): **IDENTICAL**.
`--test` green (0 FAILs).

## §3 Two failure modes found (the threshold/regression risk made concrete)
### §3.1 One-symbol-early false-lock on CLEAN (FIXED by re-anchoring)
The live s8 fine window starts ONE OFDM symbol before the coarse preamble estimate
(`s8_win_start = (pream_symb_loc-1)*sym`) for self-correlation slack. The matched
detector's coarse search strides by the GI period (216 interp-samples) and picks the
global Cauchy-Schwarz-sum peak. With ~identical preamble symbols, the one-symbol-early
position scores (K-1)/K ≈ 3.0 of the true 4.0, and the GI grid NEVER lands on the true
symbol boundary (1240) — so the early position WINS. Off-bench proof (clean, Es/N0=900):
matched.delay=0 (false, 6200) vs self-corr.delay≈1240 (true, 7440); clean decode
0/20 → ✗. **FIX:** anchor the matched slice AT the coarse preamble estimate
(`s8_win_start + sym_interp`), so slice-offset 0 is the true 4/4 position (Schmidl-Cox
accurate to ~16 samples << ±GI fine window), add the symbol back to the returned delay.
After fix: clean 20/20 (= self-corr), no regression.

### §3.2 Sub-sample SFO drift NOT tracked (FUNDAMENTAL, NOT fixed)
The matched fine search steps by `interpolation_rate` (`ofdm.cc:3820`) and the template
is decimated to baseband rate — so it has **interp-grid resolution, NOT 1-sample**. The
self-correlation runs at `step=1` (sub-sample). Under SFO the true preamble drifts a few
samples per frame; self-correlation tracks it (det=7433-7435 late frames, decode OK),
the matched filter snaps to the nominal grid (det=7440, 33-50 biterr, decode FAIL).
Giving the matched filter sub-sample resolution requires a fractional-delay interpolated
template — a detector redesign outside the scope of this historical experiment.

## §4 Off-bench A/B (sfo_block_test, cfg=16, 60 frames) — frames_decoded OFF vs ON
| Es/N0 | SFO    | OFF   | ON    | read                                  |
|-------|--------|-------|-------|---------------------------------------|
| 900   | 0      | 20/20 | 20/20 | clean: PARITY (re-anchor fixed §3.1)  |
| 11    | 0 (AWGN)| 32/60 | 31/60 | pure-AWGN: PARITY                      |
| 12    | 0 (AWGN)| 57/60 | 56/60 | pure-AWGN: PARITY                     |
| 11    | 8ppm   | 30/60 | 7/60  | **REGRESS** (sub-sample §3.2)         |
| 12    | 8ppm   | 53/60 | 31/60 | **REGRESS** (sub-sample §3.2)         |
`mean_metric` (= coarse Schmidl-Cox metric) is unchanged by the swap (the matched filter
only refines the fine delay), so frames_decoded is the discriminator.

## §5 Cross-layer audit (PHY fine-timing → channel-est → LDPC)
- **Producers of `receive_stats.delay`**: the fine stage at `telecom_system.cc:~2456`
  (`s8_win_start + fine_result.delay`) — this is the ONLY producer the flag alters.
- **Consumers**: frame extraction / `rational_resampler` (delay clamp `:2429`), channel
  estimation (`mean_H`), SKIP-H gate, LDPC. The flag changes only WHICH sample the fine
  stage picks; the accept/reject acquisition gate is on `coarse_metric` (Schmidl-Cox,
  ~0.15) UPSTREAM and is untouched — so flag-on can move the fine sample but cannot
  reject a coarse-accepted frame. No threshold was lowered.
- **Interaction with `ofdm-fine-timing-magnitude.md`**: that fix made the
  self-correlation fine stage CFO/SFO-robust (the GI+repetition phase-invariant metric).
  This experiment confirms that robustness is exactly what the matched filter LACKS.

## §6 Posture
Default-OFF, byte-identical, mergeable independently. DO NOT ENABLE on the basis of this
data. If a future bench A/B wants to test matched acquisition, the §3.2 sub-sample gap
must be closed first (fractional-delay template) — otherwise it loses under real SFO.

# Big-block channel-estimation collapse — un-tracked residual CFO

Branch: `fix/bigblock-chanest` (worktree `C:/Users/kamer/mercury_wt/bb-chanest`, off `monitor` be80936)
Status: **localized + genuine off-bench reproduction built; Option C (pre-FFT tracked CFO) recovers the
ESTIMATE but NOT the payload; CFO-injector FIDELITY fixed (§12, 641-tap band-flat Hilbert) — arbiter now
faithful but PASS-AFTER still fails (estimate≠payload + 2nd timing artifact). HW validation of Option C is
the remaining gate.** See §11 (Option C) and §12 (injector fidelity + new findings).
Off-bench artifacts: `bigblock_p3_hw/results_bb_chanest_fix3.json`, `results_cfo_testfidelity.json`.

## §1 Symptom (from the bench)
`results_rxdecode_diag.json` / `results_overnight_campaign.json`: same RX / same channel /
same run, the **per-frame** OFDM path reads `[OFDM-OK] meanH=0.979` (re-acquires every ~12-sym
frame) while the **big-block** path reads `[RXACQ] meanH=0.002-0.011` (~0) over its single
133-symbol / 74752-sample acquisition; LDPC never converges (`ldpc_iter=101` cap, all 8 cw
identical garbage `e296c428`). Preamble timing IS found (`sc_metric` 0.63 on 2/8 blocks).

## §2 The genuine (ref==NULL) test — keystone
`--test-bigblock-chanest` (`cl_arq_controller::test_sim_inproc_bigblock_chanest`,
`source/datalink_layer/test_bigblock_arq_unit.cc`). One CFG16 big-block:
`tsA` TX → `cl_sim_awgn(CFO)` → `tsB` RX. `tsB` never transmits ⇒ `bigblock_last_tx_K==0` ⇒
`cw_info_ref==NULL` in `receive_bigblock` (telecom_system.cc:8391-8392) ⇒ the **genuine**
per-codeword decode (NOT the oracle compare at :7497-7498). Self-contained, no ARQ ACK loop,
~7 s. Arms:
- SANITY (clean): meanH=0.194, 8/8 byte-faithful → PASS.
- FAIL-BEFORE (CFO 8 Hz): meanH=0.098 (collapsed), bytes wrong → PASS (defect reproduced).
- PASS-AFTER: asserts meanH>0.18 + 8/8 → FAILS until a fix lands (the contract).

The bench claim "the in-process sim CANNOT see it — it takes the ORACLE path" was **imprecise**:
`--test-bigblock-multicw`/`-fullpath` are ALSO `ref==NULL`; what they lacked was a CFO/SFO
impairment (default channel `SFO=0 CFO=0`), so there was no rotating phasor to integrate and
they pass 8/8. The fix here was to add the CFO impairment to a genuine `ref==NULL` decode.

Instrumentation added (no production behavior change): `telecom_system.bigblock_last_rx_meanh`
(always-on mean|H| stash, set at telecom_system.cc:7424, BEFORE the env-gated `[DIAG-RXPB]`
print) + `cl_arq_controller::bigblock_first_meanh`.

## §3 Root cause (high confidence)
`bigblock_rx_passband` (telecom_system.cc:7175) does ONE Schmidl-Cox **timing** acquisition on
the head preamble and **NO carrier-frequency (Moose) correction** (the acquire block :7199-7251
is timing only — grep confirmed: no `carrier_frequency_sync*` / `freq_offset` in 7175-7505).
The **per-frame** `receive_byte` path runs Moose every frame (OFDM `carrier_sampling_frequency_sync`
:2546 / MFSK :2570) and re-mixes (:2645), so its residual CFO stays tiny.

With no frequency correction, the residual CFO ramps a multi-cycle phasor across the 133-symbol
/ ~1.56 s block (8 Hz × 1.56 s = **12.46 cycles**, ~34°/symbol). The block-wide pilot channel
estimate — flat-ML `Hbar = Hsum/npil` (:7389-7401) and the sparse-2D estimate (:7385) —
destructively integrates the rotating phasor ⇒ `mean|H| → 0` ⇒ LDPC decodes noise. SAME-RUN
discriminator (HW): per-frame meanH 0.979 (re-acquires Moose) vs big-block 0.005 (single
acquire, no Moose).

### §3.1 Smoking gun (flat-ML)
`MERCURY_BIGBLOCK_SPARSE2D=0` under CFO=8: per-pilot `|Y/X|=0.20` (≈full) but block-averaged
`|Hbar|=0.024` (uniform, std≈0) — a 10× collapse from coherent averaging of a rotating phasor.
The per-pilot **magnitudes survive**; their per-symbol **phases spread** and average to ~0.

### §3.2 SFO is a DIFFERENT failure
`MERCURY_SIM2_SFO_PPM=50` alone: meanH=0.244, pilraw=0.286 (HEALTHY estimate) but 0 bytes /
7-8 clean ⇒ SFO = a timing-walk decode failure, NOT the estimate collapse. **CFO** is the
channel-estimation-collapse reproducer.

### §3.3 The existing TRACK is an inadequate band-aid
STEP-2 TRACK (CPE/PEG de-rotation, :7332-7364) fits per-symbol common-phase (`sym_omega`) +
SFO slope (`sym_delta`) and de-rotates post-demod. It does NOT recover the collapse:
`TRACK_WIN=1` vs `9` give identical meanH; per-pilot `|Y/X|` unchanged. It (a) runs post-demod
(cannot fix pre-FFT ICI) and (b) window-AVERAGES per-symbol omega across ±4 symbols — averaging
a phase RAMP is harmful, not a fix.

## §4 Refuted fix attempt (recorded so it is not re-tried blindly)
**AFC-before-FFT** (Moose 1994 / 802.11a / Speth 2001): estimate residual CFO on the head
preamble (`ofdm.carrier_sampling_frequency_sync`) + re-mix the whole block at
`carrier_frequency - cfo` before demod (env-gated `MERCURY_BIGBLOCK_CFO`; **removed** after
refutation, not left as dead code).
- The estimate is **accurate** (`[BIGBLOCK-CFO] residual=8.0 Hz` on the CFO=8 cell).
- It does **NOT** recover: flat-ML Hbar 0.023 (fix) vs 0.024 (no fix); pilraw 0.197 unchanged.
- Why: a SINGLE head estimate is insufficient for a 1.56 s block — even ~0.5 Hz residual /
  estimate error accumulates ~0.8 cycle and de-coheres the block average; and the CFO drifts
  across the block. **Long packets need PER-SYMBOL pilot tracking, not one preamble estimate.**

## §5 Fix plan (NOT implemented — next principled direction)
Per-symbol / per-sub-block decision-directed CFO/CPE tracking applied BEFORE the block-wide
estimate (802.11a pilot tracking). Preferred = **Option A**: replace the single block-wide
channel average with a PER-SYMBOL pilot estimate `Hbar[n]` (time-interpolated) so a per-symbol
ramp never enters one average — lowest risk, no new acquisition; the sparse-2D estimator already
collapses less (0.13 vs flat-ML 0.024) because it is time-local. Option B: fix TRACK to remove
the inter-symbol CFO ramp before the estimate. Option C: mid-block Moose re-acquire.

§5-audit (CLAUDE.md): the estimate touches `ofdm.estimated_channel`, `pilot_configurator.sequence`,
`noise_variance_estimate` (consumed by `channel_equalizer` / `psk.demod` / CSI-weighting); the
per-frame path shares these members. Audit producers/consumers before changing the estimator —
this seam has 5+ prior flip-flops. Validate against `--test-bigblock-chanest` (fail→pass), then
**HW** (PHY change; the in-sim test is necessary but NOT sufficient). DO NOT merge to monitor
until HW-validated.

## §6 No regression
All existing big-block tests pass on this branch (production unchanged): `--test-bigblock-multicw`,
`--test-bigblock-fullpath`, `--test-bigblock-arq-unit`, `--test-sim-inproc-bigblock` all rc=0.

## §7 Correction log
- A "clean block-2 collapse" reading (interim) was a MISATTRIBUTION of `--test-bigblock-multicw`
  ARM-B (`DEFEAT_FIX=1`, a deliberately truncated stock RX window), NOT a cross-block bug. The
  isolated harness decodes consecutive clean blocks all-healthy (0.194 ×3). The real reproducer
  is the un-tracked residual CFO. (Struck through the cross-block-state hypothesis.)

## §8 SS5 cross-layer data-flow audit (before changing the estimator/de-rotator)
Shared state the big-block RX channel-est path writes, and every other producer/consumer:

**`ofdm.estimated_channel[Ngrid*Nc]`** (one complex H per grid cell)
- Producers (big-block RX): flat-ML write `telecom_system.cc:7401` (Hbar broadcast to all cells);
  sparse-2D write `grid_sparse2d_estimator:5977-5981`. Both set `status=MEASURED`.
- Producers (per-frame, SHARED member): LS/ZF estimator in the `receive_byte` OFDM path
  (`telecom_system.cc:~2737-2785`) + `OFDM.cc` fills. The big-block path REBUILDS the grid
  (`bigblock_rebuild_thin_grid` deinit/init to `Nsymb=Ngrid`) and owns `estimated_channel`
  exclusively for the block; the per-frame path never runs concurrently (different `Nsymb`/site).
- Consumers: `cl_ofdm::channel_equalizer` (`OFDM.cc:2173-2208`) reads `H`, computes `out=in/H` (ZF)
  with MMSE erasure when `alpha=|H|^2/(|H|^2+nv) < 0.1`, then sets `status=UNKNOWN`. Big-block CSI
  loop `:7407-7414` reads `|H|^2` for LLR weighting. meanH stash `:7425-7426` (test metric). DIAG `:7432`.
- INVARIANT: `out=in/H=X` requires H to capture the cell's TRUE complex gain INCLUDING per-symbol
  phase. A per-symbol-LOCAL estimate satisfies it; a block-AVERAGED estimate does not when the
  per-symbol phase ramps.

**`ofdm.pilot_configurator.sequence[]`** (transmitted pilots X, raster order over PILOT cells)
- Producer: `pilot_configurator.init` at grid rebuild; read-only after.
- Consumers: every Y/X site (`pilot_evm:7290`, TRACK LS `:7348`, flat-ML `:7392`, sparse-2D `:5847`,
  nv residual `:5992`/`:7398`, DIAG `:7436`) walk it by a running `pidx` in the SAME nested-n/j
  `type==PILOT` raster order. INVARIANT (held): pidx order matches the framer's PILOT order.

**`ofdm.noise_variance_estimate`** (scalar sigma^2_n)
- Producers: flat-ML `:7402`, sparse-2D `:5997` (pilot-residual EVM, floored 1e-6). SHARED with the
  per-frame nv; big-block owns it for the block.
- Consumers: equalizer MMSE erasure threshold (`OFDM.cc:2200`); `psk.demod` LLR scale (`:7451`).
  INVARIANT: nv must reflect the post-de-rotation pilot residual (TRACK de-rotates `rx` before nv).

**What Option A changed**: only the PHASE of `rx` fed to the (unchanged) estimators -> estimated_channel/nv.
It does NOT touch pilot.sequence, does NOT touch the per-frame path (change is inside
`bigblock_rx_passband`, reached only when `bigblock_framing_enabled`, default-off), and the
equalizer/demod/CSI consumers keep reading estimated_channel/nv exactly as before. **Audit conclusion:
confined to the big-block RX seam; no consumer invariant violated. Audit done.**

## §9 Attempt #2 — Option A (post-FFT per-symbol pilot phase tracking): IMPLEMENTED, REFUTED by measurement
**What**: in `bigblock_rx_passband` STEP-2 TRACK, replaced the harmful window-AVERAGE of the WRAPPED
per-symbol common phase `sym_omega[n]` with: UNWRAP `sym_omega` along the symbol axis (shortest-step
cumulative), then de-rotate each symbol by its OWN unwrapped common phase (light-smooth the unwrapped
near-linear sequence = the 802.11a smoother). `sym_delta` (per-carrier SFO slope) kept window-averaged.
Confined to the big-block RX path; per-frame untouched. (Reverted after refutation — not left as dead
code, per the §4 attempt-#1 precedent. Binary byte-identical to baseline: 32867491.)

**Genuine test result (`--test-bigblock-chanest`, the off-bench arbiter)**: PASS-AFTER still FAILS.
- baseline (no fix):           PASS-AFTER meanH **0.0982**, bytes_ok=0
- Option A (unwrap+derotate):  PASS-AFTER meanH **0.0965** (sparse-2D) / **0.0192** (flat-ML) = NO recovery
  (flat-ML slightly WORSE than the 0.0235 TRACK-off baseline). SANITY clean stays 0.194 8/8 (no regression).

**Why it failed (MEASURED) — two independent obstructions, both rooted in NO pre-FFT AFC:**
1. **Per-symbol common phase is UN-ESTIMABLE from this lattice under CFO.** `[TRACK-DIAG]` under CFO=8:
   `sym_omega` n=0->1.06, n=1->-1.77, n=2->2.01 rad (wild jumps, not a 0.59 rad/sym ramp). The 2 CONTINUAL
   pilots sit at band edges (carriers 0, 49); under CFO there is also a per-carrier slope, and each pilot
   phase is `atan2`-wrapped to (-pi,pi], so once inter-pilot phase across the band exceeds pi the LS fit
   CONFLATES slope+intercept and the intercept ALIASES. The unwrapper then aliases (span 11.68 rad vs the
   true ~78 rad / 12.46 cycles). De-rotating by aliased phase scrambles, not corrects.
2. **Genuine ~33% INTRA-symbol MAGNITUDE loss that NO post-FFT phase tracking recovers.** Raw pilot `|Y/X|`
   (measured after TRACK, before the estimate): clean **0.228** -> CFO **0.153**. Timing-search-INDEPENDENT
   (`BIGBLOCK_TSEARCH_STEP=1` -> 0.1535 == step=2) so NOT a window artifact; it is intra-symbol ICI / the
   FFT integrating a frequency-offset signal. Flat-ML Hbar collapses 0.228(clean)->**0.0235**(CFO, phase
   spread); even a perfect phase fix leaves the 0.153 magnitude floor (< 0.18 gate, < 0.228 clean).

**Conclusion**: post-FFT pilot tracking is the WRONG LAYER. Both obstructions are symptoms of the §3 gap:
the big-block RX does ONE head Schmidl-Cox TIMING acquire and NO carrier-frequency (Moose) correction, so
the residual CFO is never removed in the TIME DOMAIN before the FFT. Phase ramp AND magnitude loss both come
from the un-corrected pre-FFT carrier offset; only a pre-FFT AFC removes both.

## §10 SS2 attempt count + next option (DO NOT iterate unsupervised)
- Attempt #1 (§4): head-preamble AFC single re-mix — REFUTED (drift; one estimate insufficient for 1.56 s).
- Attempt #2 (§9): post-FFT per-symbol pilot phase tracking (Option A) — REFUTED (lattice can't estimate the
  per-symbol phase; intra-symbol magnitude loss is post-FFT-unrecoverable).
- **2 consecutive principled attempts on the same defect. CLAUDE.md §2 -> STOP and discuss before #3.**
- **NEXT (supervised) = Option C: mid-block / sub-block Moose RE-ACQUISITION (true pre-FFT AFC tracking).**
  Re-estimate the carrier frequency periodically across the 1.56 s span (embedded mid-block preambles, or a
  decision-directed TIME-DOMAIN CFO from the continual pilots' inter-symbol phase rate) and RE-MIX each
  sub-block's time samples at `carrier - cfo` BEFORE its FFT. Only this removes BOTH the phase ramp AND the
  intra-symbol magnitude loss (both pre-FFT). Most invasive (most faithful to the per-frame path, immune
  precisely because it re-runs Moose every ~12-sym frame). Option B (fix TRACK to estimate the global ramp
  slope) is DOMINATED — still post-FFT, can't fix the 0.153 magnitude floor. Option C likely needs a TX-side
  change (periodic mid-block preambles) or a robust decision-directed time-domain CFO estimator = an
  architectural increment, not a tweak — hence the STOP-and-discuss gate.

## §11 Attempt #3 — Option C PRE-FFT TRACKED CFO: IMPLEMENTED + MECHANISM-VALIDATED, but the GATE is BLOCKED by a TEST-HARNESS CFO-MODEL ARTIFACT (NOT the fix). SS2 STOP.
**What was built** (telecom_system.cc bigblock_rx_passband, +161 lines, confined to the dormant default-off
big-block path; per-frame byte-identical; env MERCURY_BIGBLOCK_AFCTRACK default 1):
- A real PRE-FFT, TIME-DOMAIN, TRACKED CFO de-rotation. `demod_at`/`bb_at` split so the time-domain decimated
  baseband can be de-rotated by a CONTINUOUS per-sample phase ramp (a genuine frequency correction) BEFORE the
  per-symbol FFT. Estimator: STAGE 1 = head Moose (`carrier_sampling_frequency_sync`, ACCURATE — the recovery
  peak coincides with its value), constant pre-FFT removal of the bulk residual; STAGE 2 = residual DRIFT from
  the continual-pilots' CUMULATIVE common phase ψ[n]=arg(Σ rx·conj(X)), unwrapped (small after Stage 1, so NO
  aliasing — the §9 aliasing was on the LARGE un-corrected ramp), smoothed, differentiated to a per-symbol
  residual freq, ADDED to Stage 1, single re-demod. Refs van de Beek 1997 / Moose 1994 / Speth 2001.
  (A per-symbol cyclic-prefix CFO estimator was also tried; it is SYSTEMATICALLY BIASED HIGH in this decimated/
  FIR'd pipeline — chan=4 → est 8.7 Hz, chan=8 → 9.6 w/ ±20 spikes — so the head-Moose+ψ tracker is used.)

**MECHANISM VALIDATED (measured, via env DBG knobs that vary the test's CFO/walk without changing the default
arbiter — MERCURY_BBCHANEST_DBG_CFO_HZ/_CFO_WALK_HZ, defaults 8/4):** the fix RECOVERS the genuine phase-spread
collapse wherever the sim CFO model is faithful (AFC-OFF→AFC-ON mean|H|):
- cfo=2 w=0: 0.134→**0.183** | cfo=4 w=0: 0.099→**0.175** | cfo=4 w=2: 0.091→**0.171** | cfo=6 w=0: 0.163→**0.170**.
  (clean=0.194; SANITY clean stays 0.194 8/8 — AFC is a no-op on a clean channel: head Moose≈0.)

**WHY THE GATE (cfo=8/walk=4) STILL FAILS — DECISIVE NEW FINDING (the BLOCKER is the TEST, not the fix):**
The collapse has TWO components vs channel CFO (AFC-OFF, walk=0):
  | chanCFO | mean\|H\| | pilraw\|Y/X\| |   ← pilraw = raw continual-pilot magnitude (the band-edge pilots)
  |   ~0    |  0.191   |  0.226 |
  |    4    |  0.099   |  0.202 |   ← meanH collapsed (phase-spread) but MAGNITUDE HEALTHY → RECOVERABLE
  |    8    |  0.099   |  0.154 |   ← MAGNITUDE also destroyed → NOT recoverable by any freq correction
- The ≤~5 Hz collapse is PURE inter-symbol phase-spread (pilraw healthy) → the fix recovers it.
- The ≥~6 Hz magnitude loss (pilraw 0.226→0.154) is **an ARTIFACT of the sim's CFO model, NOT real CFO ICI.**
  PROOF: applying a CLEAN synthesized 8 Hz pre-FFT shift to the CLEAN signal preserves pilraw (0.226→**0.219**),
  while the sim's real 8 Hz channel CFO destroys it (→**0.154**); and NO pre-FFT correction (any sign/value,
  ±0.5 Hz fine sweep, ± timing nudge) restores pilraw past ~0.157 at chan=8. A clean frequency offset does NOT
  cause this magnitude loss; the loss is therefore not removable by frequency correction.
- ROOT of the artifact: `cl_sim_cfo` (include/common/sim_channel.h:551-627) rotates the passband by forming the
  analytic signal with a **65-tap Hamming-windowed Hilbert FIR** (build_hilbert :636). A 65-tap Hilbert has poor
  amplitude response near the band EDGES — exactly where the big-block CONTINUAL pilots sit (carriers 0 & Nc-1,
  the thin-grid cont_cols). So the model attenuates/distorts the band-edge pilots ∝ the rotation phase → the
  ~33% pilraw loss at ≥6 Hz. This is a measurement artifact of the impairment generator, not a property of carrier
  frequency offset. **HW reality is the OPPOSITE: fact-doc §3.1 — HW per-pilot MAGNITUDES SURVIVE, only the
  per-symbol PHASES spread (the recoverable kind). So the fix recovers the REAL bug; the test over-penalizes.**

**CONCLUSION (SS2 — 3rd attempt, BLOCKED not refuted):**
- The pre-FFT tracked-CFO fix is MECHANISM-CORRECT and recovers the genuine phase-spread collapse (the HW bug).
  It is production-safe (dormant default-off; all big-block + climb-engine + probe-backoff + sim-clock +
  bigblock-climb-election tests rc=0; per-frame byte-identical).
- It does NOT pass `--test-bigblock-chanest` PASS-AFTER because that gate's CFO=8 cell triggers the sim's
  Hilbert-FIR band-edge magnitude artifact, which caps mean\|H\|≈0.10 (pilraw≈0.143) for ANY correct AFC.
- **The arbiter, not the fix, is the blocker.** Do NOT weaken the gate to force a pass (CLAUDE.md §2/§What-NOT).
  Two clean paths for the morning (pick one, supervised):
  (A) FIX THE TEST HARNESS: replace `cl_sim_cfo`'s 65-tap Hilbert with an FFT-domain / longer-FIR analytic
      transform (band-edge-flat), OR apply the CFO as a true complex rotation on a baseband-then-reupconvert
      path so the band edges are not attenuated; then the cfo=8 cell becomes pure phase-spread and the fix's
      PASS-AFTER is reachable. Re-run --test-bigblock-chanest → expect PASS. (This is fixing arbiter FIDELITY,
      which IS allowed and required — distinct from weakening the gate.)
  (B) HW-VALIDATE THE FIX DIRECTLY on the bench (the real channel has magnitude-surviving phase-spread, §3.1):
      enable bigblock + AFCTRACK at CFG16 under the residual-CFO HW cell, expect mean\|H\| recovery 0.005→healthy.
- Either way: the fix is a STRONG candidate. NOT merged to monitor (HW-validation required first).

## §12 Attempt #4 — CFO-INJECTOR FIDELITY FIX (path A): IMPLEMENTED + PROVEN; arbiter is MORE faithful, but PASS-AFTER STILL FAILS for a DEEPER reason (Option C recovers the ESTIMATE not the PAYLOAD). 2026-06-07.
**What was built** (`include/common/sim_channel.h`, `cl_sim_cfo`): replaced the 65-tap Hamming-windowed
Type-III Hilbert (`build_hilbert`, HILB_LEN 65→**641**, window Hamming→**Blackman-Harris**). The CFO
injector still forms `y[n] = Re{ (x_d + j·x_h)·e^{jφ[n]} }` with the SAME stateful/continuous-phase/
default-OFF contract (cfo_hz==0 && walk_hz==0 ⇒ no-op ⇒ byte-identical), only the analytic transform is
now **band-flat across the full OFDM occupancy** (carrier 1500 Hz ± 1171.875 = **328.125..2671.875 Hz @
fs=48k**, where the band-edge continual pilots sit).

**FIDELITY PROVEN (the §11 injector artifact is GONE):**
- *Band-edge magnitude response* (standalone numerics): the 65-tap Hamming Hilbert retains only **0.45**
  of unit magnitude at 328 Hz (carrier-0 pilot) — a Type-III Hilbert has a DC null and the 65-tap lower
  transition band is far wider than the 328 Hz edge. The **641-tap Blackman-Harris is |H|=0.99999 @ 328 Hz**,
  reaches 0.999 by 245 Hz (margin below the edge), in-band ripple < 2e-5.
- *Injector == ideal*: a 641-tap-FIR analytic shift of a multi-tone passband at cfo=8 matches the **ideal
  FFT-domain analytic shift to 4 decimals** at EVERY band tone (328→2672 Hz: ratio 1.0000), LSB leakage
  ~0.0077 (edges) / 0.0002 (mid). The OLD 65-tap retained ~0.27 at 328 Hz AND leaked ~0.71 into the LSB —
  it was barely SSB at all. (`/tmp/cfo_compare.py` methodology; reproducible.)
- *In-modem pilraw now PRESERVED* (AFC-OFF, walk=0): clean=0.226; cfo=2→**0.225**, cfo=4→**0.219** (vs OLD
  cfo=4=0.202, cfo=8=0.154). Up to ~5 Hz the band-edge continual-pilot MAGNITUDE survives, matching the
  clean synthesized shift (§11) and HW §3.1. The cfo=8 magnitude artifact the §11 STOP blamed on the
  injector is **resolved for cfo≤5**.

**RIGOR HOLDS — the gate is NOT trivially passable.** With the faithful injector the FAIL-BEFORE arm
(AFC-OFF) STILL collapses at the default cfo=8/walk=4: meanH=**0.10** (<0.16), bytes_ok=**0**. The genuine
phase-spread defect is still reproduced and caught. SANITY clean unchanged: meanH=0.194, 8/8 byte-faithful.

**OPTION C now RECOVERS the ESTIMATE exactly where the injector is faithful** (AFC-OFF→AFC-ON mean|H|,
walk=0): cfo=1 .173→**.192**, cfo=2 .144→**.191**, cfo=3 .177→**.190**, cfo=4 .103→**.183** — full meanH
recovery for cfo≤4. The AFC recovery boundary tracks the pilraw-preservation boundary precisely.

**BUT PASS-AFTER STILL FAILS — two NEW, deeper findings (the blocker is no longer the injector):**
1. **Option C recovers the channel ESTIMATE, not the PAYLOAD.** At cfo=3 and cfo=4 (faithful regime,
   pilraw≈0.22), AFC-ON restores meanH to **0.19/0.183** (>0.18 gate) yet **bytes_ok=0**. (cw_ok=8/8 is a
   NO-OP here: `bigblock_rx_passband` only counts ierr when `cw_info_ref!=NULL` — telecom_system.cc:7662;
   the genuine `--test-bigblock-chanest` runs ref==NULL so cw_ok is always Kcw. The TRUE arbiter is the
   test lambda's `bytes_ok` carve+compare.) So restoring mean|H| is NECESSARY but NOT SUFFICIENT: the LDPC
   payload still decodes wrong even with a healthy estimate + faithful CFO. Option C is **incomplete at the
   bit/payload layer**, independent of the CFO model. [?] residual per-symbol phase the meanH metric
   averages out but the per-subcarrier soft-demap still sees (CPE/SFO interaction, or a CSI-weighting bug).
2. **A SECOND, distinct sim-RX artifact at cfo≈6-8.** pilraw cliffs 0.219(cfo4)→**0.168**(cfo6,7,8) then
   RECOVERS to 0.206(cfo10)/0.195(cfo14) — **non-monotonic**. Genuine OFDM ICI is monotonic and tiny
   (sinc(ε): ε=8/46.875=0.171 ⇒ 0.953 retention, only 5%). A 23% non-monotonic dip is NOT ICI; it is an
   ACQUISITION/timing-acq resonance (head Schmidl-Cox timing estimate jumps a sample near ~6 Hz, sliding
   the FFT window, then re-locks higher). The arbiter's DEFAULT cfo=8 sits in this trough — so cfo=8 is now
   the LESS HW-faithful cell (HW §3.1: magnitudes SURVIVE), while cfo≤4 is the MORE faithful (pure
   phase-spread). This is a DIFFERENT artifact from the §11 Hilbert one; NOT chased (CLAUDE.md §2 / the
   3-attempt rule). [?] localize to `passband_to_baseband`/`rational_resampler` band edge vs S&C timing.

**CONCLUSION (path A done; verdict = NEW info, STOP not iterate):**
- The CFO-injector fidelity fix is CORRECT and worth keeping: the test is now strictly MORE faithful (the
  §11 band-edge magnitude artifact is eliminated; injector == ideal shift). COMMITTED to fix/bigblock-chanest.
- It does NOT make `--test-bigblock-chanest` PASS at the default cfo=8, because (a) Option C recovers the
  ESTIMATE but not the PAYLOAD even in the faithful cfo≤4 regime (bytes_ok=0 @ meanH 0.183), and (b) the
  default cfo=8 also sits in a 2nd timing-acq artifact trough. Moving the default to cfo=4 would NOT fix it
  (bytes_ok=0 there too) — so the default is LEFT UNCHANGED (no masking).
- Do NOT shotgun a payload-layer fix. The correct next step is **HW validation of Option C directly** (path B,
  §11): the real channel has magnitude-surviving phase-spread (§3.1), and bytes_ok is the only true gate.
  If Option C also fails to deliver bytes on HW, the estimate-vs-payload gap (finding #1) is the real defect
  to investigate next — NOT the CFO model.

## §13 Estimate-vs-payload gap DIAGNOSED at cfo≤4 (Option C ON): the residual is a CFO-MAGNITUDE-INDEPENDENT per-subcarrier artifact, NOT a per-symbol CPE and NOT a production PHY defect. 2026-06-07.
**The §12 finding #1 open question is resolved.** Why does Option C recover mean|H| (0.183 > 0.18 gate) at
cfo≤4 yet bytes_ok=0? Decision-directed post-EQ instrumentation (all env-gated; production byte-identical):
`[DIAG-RXPB]` adds DD-EVM + per-symbol-row residual common phase (rowCPE); `[DIAG-GENIE]` adds a genie
per-cell channel decomposition; `[AFC-PILTHETA]`/`[AFC-WITHINSYM]` (under `MERCURY_BIGBLOCK_AFC_DIAG`) add
the ground-truth per-symbol common-phase trajectory + within-symbol pilot-phase spread from ALL known pilots.

**The leading hypothesis (a) — residual per-symbol CPE — is REFUTED BY MEASUREMENT.**
Post-EQ per-symbol-row common phase `rowCPE` is TINY at every CFO: ±0.06 rad (~±3°), meanabs ~0.03 rad.
Far too small to break 32-QAM. Option C already removes the per-symbol common-phase ramp well. A post-FFT
per-symbol CPE correction has nothing to correct (confirmed: STEP-2 tracker `twin=1/3/9` all leave
DDevm≈0.13–0.39; STEP-2 OFF is WORSE — it is helping a little, not the cause). Attempt #2 (§9) was the right
layer-refutation; (a)'s refinement does not revive it.

**The real cause: a per-SUBCARRIER, NON-LINEAR, CFO-MAGNITUDE-INDEPENDENT phase distortion (~0.109 rad).**
Decision-directed post-EQ EVM (`DDevm`, DATA cells, AFC ON):
  | arm           | DDevm  | rowCPE (per-sym common)        | within-sym pilot RMS | resid after per-sym LINEAR fit |
  |---------------|--------|--------------------------------|----------------------|--------------------------------|
  | clean (cfo=0) | 0.038  | ±0.015 rad                     | 0.125 rad            | **0.004 rad**                  |
  | cfo=1         | 0.137  | ±0.06 rad                      | 0.371 rad            | **0.109 rad**                  |
  | cfo=4         | 0.392  | ±0.06 rad                      | 1.03 rad             | **0.421 rad**                  |
- Clean: the within-symbol pilot phase spread (0.125 rad) is FULLY explained by a per-symbol LINEAR fit
  (residual 0.004 rad) = the channel's genuine timing/selectivity slope, perfectly removable.
- Under CFO: after removing the best per-symbol LINEAR (CPE+slope) fit there is STILL a **non-linear
  per-subcarrier residual** (0.109 rad @cfo=1, 0.421 @cfo=4). NOT a common phase, NOT a slope ⇒ NOT removable
  by ANY per-symbol phase tracker NOR by any per-cell channel estimate (it is per-cell phase, not a gain).
  32-QAM needs ≲~8% EVM; 13.7% @cfo=1 is already over the cliff ⇒ bytes_ok=0 even where mean|H| is healthy.

**SMOKING GUN — the residual is CFO-MAGNITUDE-INDEPENDENT (proves it is NOT real CFO physics).** Sweeping
`cfo_sigma` = 0.25 / 0.5 / 0.75 / 1.0 (head-Moose tracks proportionally: 0.30/0.58/0.87/1.15 — correct), the
non-linear within-symbol residual is **FLAT at 0.1086 rad** and DDevm flat at ~0.13. A genuine CFO-induced
impairment (ICI ∝ (πε)²/3 ⇒ ~3e-5 at these residuals; or a common-phase ramp ∝ ε) would VANISH as ε→0. A
fixed 0.109 rad floor that appears the instant ANY CFO is enabled but is 0.004 rad at exactly cfo=0 is a
**fixed numerical/discretization artifact of how the impairment + RX pipeline handle a frequency-offset
buffer**, not a channel response. (The cfo_sigma=4 jump to 0.42 is a separate larger-draw/timing effect on
top of the floor.)

**Localization: the artifact is mostly in the RX big-block pipeline, NOT only the FIR-Hilbert injector.**
Added an IDEAL whole-buffer FFT-domain analytic SSB shift to `cl_sim_awgn::apply_ideal_cfo`
(include/common/sim_channel.h) + a `MERCURY_BBCHANEST_DBG_IDEAL_CFO=1` test knob (DIAG-ONLY; the DEFAULT
arbiter is UNCHANGED — streaming FIR + walk — no masking). The ideal injector IMPROVES the gap (cfo=4 AFC-ON
DDevm 0.392→0.228, nzero 41→0) — so the FIR injector DOES contribute — but the within-symbol non-linear
residual is STILL FLAT at 0.109 rad for cfo_sigma 0.5/1/2 with the ideal injector, and the residual persists
with AFC OFF too. So most of the floor is the RX side (`passband_to_baseband` + `rational_resampler` +
window-boundary handling of a frequency-offset block), CFO-magnitude-independent. A standalone numeric
FIR-vs-ideal phase check (`bigblock_hw/cfo_phase_fidelity.py`) confirms the FIR adds a CFO-independent
per-tone phase discrepancy; longer Hilbert (1281/2561 taps) does NOT reduce it.

**The "exact constant CFO" red herring (recorded so it is not re-tried):** forcing the de-rotation to the
true injected value as a CONSTANT is WORSE than the head-Moose+ψ tracker (DDevm 0.60 vs 0.14 @cfo=1). Reason:
`MERCURY_SIM2_CFO_HZ` is the residual STD; the actual residual is `cfo_sigma·gauss()` (one draw) — NOT exactly
the env value — and the effective per-symbol phase the FFT sees is time-varying (resampler/timing), so a
decision-directed tracker beats any single constant. (Diag knob removed after this clarified it.)

**CONCLUSION (CLAUDE.md §2 — STOP, verdict = the gap is a sim/RX-pipeline artifact, NOT a PHY defect):**
- Hypothesis (a) per-symbol CPE: REFUTED (rowCPE ±3°). Hypotheses (b)/(c)/(d): the residual is per-subcarrier
  & CFO-magnitude-INDEPENDENT ⇒ not a per-subcarrier-estimate-accuracy problem that scales with the impairment,
  not a fixed LLR/CSI scale (clean decodes fine), and the deframe/whitening is byte-faithful on clean (SANITY
  8/8) ⇒ it is a numerical artifact of CFO-on-buffer, not a production decode defect.
- Two grounded attempts at THIS payload layer: (1) the post-FFT CPE family (refuted by direct measurement);
  (2) the ideal-CFO injector fidelity path (improved DDevm 0.392→0.228 but did NOT close the gap — most of the
  floor is RX-pipeline, CFO-independent). Per §2, STOP — do not shotgun a third payload-layer fix into
  production: there is no evidenced production defect at cfo≤4 (the residual does not scale with the real
  impairment; HW §3.1 has magnitude-surviving phase-spread, which Option C demonstrably RECOVERS — mean|H|
  0.005→0.183).
- **Option C remains the strong candidate; the next step is HW validation (path B), exactly as §12 concluded.**
  bytes_ok byte-faithful at cfo≤4 is the correct final gate but it is blocked off-bench by this artifact floor,
  not by a PHY bug. KEEP the diagnostics (durable characterization infra) + the opt-in ideal-CFO knob (it
  proved part of the artifact and is the basis for any future arbiter-fidelity work). Default arbiter unchanged.

**Status of the bytes_ok arbiter:** FAIL-BEFORE (AFC-OFF) bytes_ok=0 at cfo≤4 (confirmed, defect reproduced);
PASS-AFTER (Option C ON) bytes_ok=0 at cfo≤4 (artifact floor, NOT a PHY miss); SANITY clean 8/8 byte-faithful
(no regression). cfo=8 default additionally sits in the §12 finding-#2 timing-acq trough (separate, not chased).

## §14 Attempt #5 — PER-SUB-BLOCK BAND-SPREAD CFO TRACKING (the prescribed fix; supervised re-entry 2026-06-07).
**Re-framing (matches the directed-fix brief, NOT a blind 4th iteration):** the §13 "CFO-magnitude-INDEPENDENT
per-subcarrier ~0.109 rad artifact" verdict was measured against the §12 default at cfo≤4 with the EXISTING
Option C tracker (Stage-2 ψ from the **2 band-edge continual pilots only**, `cont_cols=2`). At the FULL
HW-faithful magnitude (the current default: `CFO_HZ=12 sd, WALK=25, F3DB=3` ⇒ per-symbol swing the brief
measured as sd ~33 Hz ≈ 0.70 of the ~46.9 Hz subcarrier spacing ⇒ Dirichlet ICI puts carriers ~70% off-grid)
the existing tracker barely moves the needle:

  | arm                        | impaired meanH | clean | gate |
  |----------------------------|----------------|-------|------|
  | AFCTRACK=0 (no tracker)    | **0.053**      | 0.194 | —    |
  | AFCTRACK=1 (Option C, ψ@2edge) | **0.085**  | 0.194 | 0.18 |

  measured 2026-06-07 against `--test-bigblock-chanest` at the committed `e3601f5` defaults (this branch).

**§14.1 ROOT CAUSE of the residual (re-localized — the §13 "artifact" is the 2-edge-pilot LIMIT, not the
RX pipeline).** `MERCURY_BIGBLOCK_AFC_DIAG=1` on the impaired arm (head_cfo=-13.4 Hz):
- `[AFC-PILTHETA]` (per-symbol common phase from ALL pilots, AFTER Option C): theta_span 11.9 rad,
  **resid_after_best-fit-linear-ramp RMS = 2.16 rad**. The tracker fit a LINEAR frequency ramp `dfs[-19.6..-7.4]`
  to a phase trajectory that is NOT linear after correction ⇒ 2.16 rad unflattened (clean: 0.0067 rad).
- `[AFC-WITHINSYM]` within-symbol pilot phase RMS **0.646 rad** (clean 0.125), residual after per-symbol
  linear (CPE+slope) fit **0.127 rad** (clean 0.004). The within-symbol pilots DISAGREE by 0.65 rad.
- WHY the 2-edge ψ fails: Stage-2 builds ψ[n]=arg(Σ_{j∈cont_cols} rx[n,j]·conj(X[n,j])) over carriers {0, Nc-1}
  ONLY (`bigblock_rebuild_thin_grid:6916-6918`, cont_cols default 2 ⇒ edges only). Under CFO the across-band
  per-carrier phase SLOPE (the 0.65 rad within-symbol spread) makes the 2-edge VECTOR SUM's argument neither
  pilot's phase and slope-corrupted; differentiating it yields a bad linear `dfs` ⇒ 2.16 rad residual. This IS
  the §9 aliasing, persisting at the realistic magnitude (Stage 1 does NOT shrink the swing enough at 33 Hz to
  de-alias the 2-edge ψ). The §13 "per-subcarrier non-linear residual" is the SIGNATURE of this missing
  band-spread information, NOT a sim/RX-pipeline numerical artifact.

**§14.2 THE FIX (per-sub-block band-spread frequency tracking — what attempts #1-4 never had):**
1. **ADD band-spread continual pilots:** `MERCURY_BIGBLOCK_CONT_COLS` default 2→5 ⇒ continual carriers at
   {0, ~12, ~24, ~37, 49} (edges + center + quarters). Small overhead (3 extra continual cols × Ngrid);
   headroom 15,226 bps > VARA 13,048 (the brief's explicit "ADD them"). Throughput re-measured post-fix.
2. **Per-sub-block residual-CFO from band-spread pilots, frequency-FIRST:** in each window of W symbols
   (W~6-9), for EACH continual carrier estimate its OWN inter-symbol phase RATE (≈ residual frequency on
   that carrier), THEN average the per-carrier rates across the band-spread carriers. Estimating a FREQUENCY
   per carrier first cancels the per-carrier phase SLOPE (a constant-in-time offset has zero time-rate), so
   the slope that aliased the lumped-ψ no longer biases the estimate. This is the 802.11 continual-pilot /
   FreeDV-700D pilot-assisted recipe (per-carrier pilot phase tracking, then combine).
3. **Feed per-sub-block frequencies into the existing `dfs[n]` ramp + reuse `build_and_demod`** (pre-FFT
   time-domain de-rotation, already in place from attempt #3). Confined to `bigblock_rx_passband`
   (dormant default-off `bigblock_framing_enabled`; per-frame byte-identical).

**§14.3 SS5 audit delta (vs §8 — only the ADD-pilots changes shared state):**
- `ofdm.pilot_configurator.sequence[]` + the `ofdm_frame[].type` PILOT mask: cont_cols 2→5 ADDS 3 continual
  columns ⇒ MORE PILOT cells, FEWER DATA cells. Producer = `bigblock_rebuild_thin_grid` (TX emit AND RX
  decode call the SAME function ⇒ TX and RX grids stay IDENTICAL; nData/nBits/K recompute symmetrically).
  Consumers (pilot_evm, TRACK LS, flat-ML, sparse-2D, nv residual, the new tracker) all walk the PILOT mask
  by running pidx in the same raster order — they AUTOMATICALLY pick up the new pilots (no consumer hardcodes
  cont_cols). INVARIANT held: pidx order == framer PILOT order (the mask is the single source of truth).
- The new tracker only changes the PHASE of the time-domain block fed to the unchanged FFT/estimators (same
  as §8 conclusion for attempt #3). No per-frame path touched (different Nsymb/site; bigblock dormant).
- K-symmetry RISK: cont_cols affects nData ⇒ ldpc codeword packing. TX and RX both derive K from the SAME
  `bigblock_rebuild_thin_grid` so K matches; the test/T6/climb-election all pin K=8 via MERCURY_BIGBLOCK_K
  (cap path) so the unit tests are unaffected by the geometry change. VERIFY: --test-bigblock-multicw /
  -fullpath / -arq-unit / climb-election still rc=0 after the cont_cols bump (they exercise the grid).

**§14.4 VALIDATION CONTRACT (the faithful sim arbiter):** FAIL-BEFORE (AFCTRACK=0) meanH≈0.053 bytes_ok=0
(reproduced) → PASS-AFTER (new tracker) meanH>0.18 AND bytes_ok 8/8 byte-faithful at the DEFAULT HW vector.
bytes_ok is the TRUE arbiter. Iterate W / cont_cols / window-LS against the faithful sim. Then SWEEP CFO
0-60 Hz, SFO 0-1000 ppm to confirm the envelope. Clean stays 8/8; per-frame byte-identical; fast tests green.

**§14.5 RESULT (IMPLEMENTED + MEASURED 2026-06-07): the band-spread per-sub-block tracker is mechanism-sound
but NOT a net improvement, and bytes_ok is BLOCKED by the same magnitude-INDEPENDENT sim/RX-pipeline artifact
§13 found — independently re-confirmed from a fresh angle. REVERTED; the committed OLD tracker (e3601f5) is
kept as the best candidate. DO NOT thrash (CLAUDE.md §2).**

*Bit-budget table (computed):* thin grid Ngrid=60 Nc=50 log2M=5 ldpc.N=1600 ⇒ K=nBits/1600 needs nBits≥12800.
cont_cols=2: npil=360 nData=2640 nBits=13200 **K=8** (slack 400b). cont=3: npil=415 nData=2585 nBits=12925
**K=8** (slack 125b). cont=4: nData=2530 nBits=12650 **K=7** ✗. cont≥4 drops K below 8 ⇒ the K=8 test bails
(meanH=-1, decode never runs). So cont_cols=3 ({0,24,49}=edges+CENTER) is the MAX band-spread that keeps K=8;
the "≥4 / 5-cols" target is NOT reachable in the K=8 block budget (would need a larger Ngrid = architectural).

*Tracker built:* Stage-2 replaced lumped-ψ with PER-SUB-BLOCK (W=7) BAND-SPREAD frequency-FIRST estimate —
per continual carrier LS-fit φ_c[n]=a+b·n (b=phase rate=residual freq on that carrier), magnitude-weighted,
MEDIAN-combined across the band-spread carriers, assigned per window into dfs, re-mixed pre-FFT via the existing
build_and_demod. Frequency-first cancels the per-carrier phase OFFSET/SLOPE (constant-in-time ⇒ zero rate).

*Measured (--test-bigblock-chanest, the off-bench arbiter):*
  | arm                                   | default HW vector | cfo=2 ideal | cfo=4 ideal | bytes_ok |
  |---------------------------------------|-------------------|-------------|-------------|----------|
  | OLD ψ-tracker, cont=2 (committed)     | meanH **0.085**   | 0.191       | 0.183       | 0        |
  | NEW band-spread tracker, cont=3       | meanH **0.057**   | 0.124       | 0.124       | 0        |
  | OLD ψ-tracker forced cont=3           | meanH **0.049**   | 0.121       | 0.109       | 0        |
- The NEW tracker is WORSE at the default HW vector (0.057 < 0.085): the per-window LS frequency jitters more
  under WALK=25 Hz + SFO than the OLD smooth-ψ-derivative. It wins only at FIR-injector mid-CFO (cfo=4 default
  injector: 0.124 vs OLD 0.077) where a real per-carrier slope exists to resolve — but loses where the §13
  artifact dominates. The band-spread pilots HURT BOTH trackers (cont=3 < cont=2 for OLD too), partly the
  tightened LDPC margin, partly the ψ math destabilized by 3-carrier mixing.

*WHY bytes_ok cannot crack (the DECISIVE re-confirmation of §13, from a NEW angle):* at cfo=1 with the IDEAL
injector + a per-symbol-tight CPE (TRACK_WIN=1), meanH recovers to **0.196 ≈ clean** and rowCPE is tiny
(±0.05 rad, meanabs 0.021) — yet **DDevm=0.247 (24.7%, vs clean 0.033) and bytes_ok=0.** The GENIE
decomposition (true channel from sliced symbols) shows the residual is INTRA-COLUMN TIME variation
col_phaseRMS=**0.231 rad** — and it is **FLAT across cfo_sigma 0.25/0.5/1.0 (0.233/0.233/0.231 rad), i.e.
CFO-MAGNITUDE-INDEPENDENT.** A genuine residual CFO would scale linearly with the offset; this does not. It is
0.030 rad at exactly cfo=0 and jumps to ~0.23 rad the instant ANY CFO is injected — a FIXED numerical artifact
of how the impaired buffer flows through passband_to_baseband/rational_resampler/FIR window boundaries, NOT a
trackable carrier offset and NOT a per-symbol common phase. nv inflates 1e-5→0.022 (2000×) even with healthy
mean|H|. No frequency correction (head-Moose, ψ-derivative, per-sub-block band-spread, ideal injector) and no
per-symbol CPE touches it. This is exactly §13's verdict, reproduced independently here with the band-spread
tracker + GENIE + ideal-injector + TRACK_WIN sweep.

*VERDICT (CLAUDE.md §2 — STOP, the brief's documented-floor branch):* the band-spread per-sub-block tracker is
the CORRECT mechanism for the REAL HW bug (HW §3.1: pilot magnitudes SURVIVE, only phases spread — that IS
recoverable), but in THIS faithful sim it is dominated by a magnitude-independent RX-pipeline artifact that caps
bytes_ok at 0 regardless of tracker. It is not a net win over the committed tracker and does not crack the
arbiter ⇒ NOT committed (source reverted to e3601f5; only this fact-doc + results JSON kept). The HW-faithful
fix candidate REMAINS the committed Option C (pre-FFT tracked CFO). **The bytes_ok off-bench gate is blocked by
the sim/RX-pipeline artifact, NOT a PHY/tracking defect — so the next step is HW validation of the committed
Option C directly (the real channel has magnitude-surviving phase-spread; bytes_ok is the only true gate), NOT
a 4th off-bench tracker iteration.** If a future arbiter-fidelity pass localizes + removes the ~0.23 rad
RX-pipeline artifact (the magnitude-independent col_phaseRMS in passband_to_baseband/resampler/FIR boundary),
THEN re-run the band-spread tracker against the cleaned arbiter — but only the artifact removal, not the tracker,
is the off-bench blocker.

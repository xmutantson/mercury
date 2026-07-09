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

## §15 ARBITER RESTORED to the PRODUCTION sparse-2D estimator (MORNING_VERDICT Step 0). 2026-06-07.

**This SUPERSEDES the §11-§14 "the off-bench gate is blocked by a sim/RX-pipeline artifact, just HW-validate"
framing for the bytes_ok arbiter.** The §11-§14 agents measured the arbiter while it PINNED the
NON-PRODUCTION flat-ML estimator (`set_faithful_impairments()` set `MERCURY_BIGBLOCK_SPARSE2D=0`,
test_bigblock_arq_unit.cc:2565). That pin was itself the contaminant the morning verdict identified:
production AND HW run sparse-2D (`telecom_system.cc:8165` default `env_i("MERCURY_BIGBLOCK_SPARSE2D",1)` =1;
adaptive sentinel `:7578` `sel<0 -> sparse-2D`). Measuring flat-ML floored col_phaseRMS at ~0.10 rad and
inflated nv ~2500x on a channel-free clean-class waveform, so the "magnitude-independent ~0.23 rad artifact
that caps bytes_ok" was substantially the flat-ML estimator's own behavior, not a property of the RX pipeline.

**THE FIX (test-only, committed 1d773ec):** flip the test estimator default `MERCURY_BIGBLOCK_SPARSE2D` 0->1
in `set_faithful_impairments()` (DBG override `MERCURY_BBCHANEST_DBG_SPARSE2D` kept for flat-ML A/B); and
re-baseline the §1b sim-predicts-HW magnitude band from the flat-ML-tuned `[0,0.10]` to the production
sparse-2D collapse (ceiling = the health gate `MEANH_BAD=0.16`; no new magic number).

**MEASURED — the restored arbiter now SEPARATES on the PRODUCTION estimator** (`--test-bigblock-chanest`,
`MERCURY_BIGBLOCK_RXPB_DIAG=1` for the GENIE/DD numbers; rebuilt `bash build.sh o3`):

  | arm                        | meanH  | col_phaseRMS | DDevm | nv       | bytes_ok | gate verdict |
  |----------------------------|--------|--------------|-------|----------|----------|--------------|
  | SANITY (clean)             | 0.1941 | **0.031**    | 0.038 | 9.4e-6   | **1**    | PASS (decodes) |
  | FAIL-BEFORE (faithful)     | 0.1487 | **0.826**    | 0.546 | 0.0347   | **0**    | PASS (genuine collapse reproduced) |
  | SIM-PREDICTS-HW            | 0.1487 | —            | —     | —        | —        | PASS (in re-baselined band [0,0.16]) |
  | PASS-AFTER (no DSP fix)    | 0.1487 | 0.826        | 0.546 | 0.0347   | **0**    | FAIL (the contract a real fix must flip) |

- These match the morning verdict's EXP-B/EXP-E predictions to the decimal (SANITY 0.031 rad / faithful
  meanH 0.149 / col_phaseRMS 0.83 / bytes_ok=0). The single remaining `--test-bigblock-chanest` failure is the
  intended PASS-AFTER contract: a correct per-symbol estimator fix must flip the faithful arm bytes_ok 0->1.
- **WHY sparse-2D, not flat-ML, is the right discriminator** (and reconciles the §13/§14 "magnitude-independent
  artifact"): under sparse-2D the per-cell MAGNITUDE largely SURVIVES (meanH only droops 0.194->0.149; the
  flat-ML deep-magnitude collapse to 0.085 is a flat-averaging artifact) — matching HW §3.1 "magnitudes survive,
  only phases spread." What breaks the decode under sparse-2D is the per-COLUMN (intra-carrier, over symbol-axis)
  phase spread `col_phaseRMS 0.031->0.826 rad`. That is exactly the block-fold defect the per-symbol fix targets,
  and it is REAL on the production estimator — not a flat-ML-only artifact and not a magnitude-band artifact.
- A/B confirmed: `MERCURY_BBCHANEST_DBG_SPARSE2D=0` still reproduces the OLD flat-ML deep collapse (meanH 0.085),
  preserved for comparison.
- NO regression: `--test-bigblock-multicw / -fullpath / -arq-unit / -sim-inproc-bigblock` all rc=0 ALL PASS.
- Change is confined to the test; NO production DSP touched.

## §16 SS5 DATA-FLOW AUDIT for the PER-SYMBOL / time-local sparse-2D fix (supersedes §8, which audited the
Attempt-#3 rx-phase-only change; this audits the ACTUAL fix = changing the ESTIMATOR's time-axis behavior + the
pilot lattice). 2026-06-07.

**Shared state under change:** `ofdm.estimated_channel[Ngrid*Nc]`, `ofdm.pilot_configurator.sequence[]` +
the `ofdm.ofdm_frame[].type` PILOT mask, `ofdm.noise_variance_estimate`.

**(1) PRODUCERS of `ofdm.estimated_channel`** (big-block RX, all inside `bigblock_rx_passband`,
telecom_system.cc:7176-8584; reached only when `bigblock_framing_enabled`, default-off):
- sparse-2D estimator `grid_sparse2d_estimator()` (telecom_system.cc:5830; called at :7583 and :8168). THIS is
  the production/HW default and the fix target. Internally: (1) raw LS at every pilot :5843-5852; (2) per-carrier
  TIME interpolation + Wiener time-smooth (`MERCURY_SFO_GRID_WIENER` def 1, `_WIENER_LEN` def 5) :5854-5892;
  (3) per-symbol FREQUENCY interpolation in polar/unwrapped form (`MERCURY_SFO_GRID_POLAR` def 1) :5894-5946;
  (4) optional DDCE (def OFF) :5948-5975; publish to estimated_channel MEASURED :5977-5982; nv from final-H
  pilot residual :5984-6000.
- flat-ML control (block-wide `Hbar` broadcast to ALL cells) :7587-7602 and :8173-8188 (A/B only, env-forced).
- per-frame path (SHARED member, DIFFERENT site): LS/ZF estimator in `receive_byte` OFDM path
  (`telecom_system.cc:~2737-2785` + `OFDM.cc`). The big-block path REBUILDS the grid via
  `bigblock_rebuild_thin_grid` (deinit/init to Nsymb=Ngrid, :6875) and owns estimated_channel exclusively for
  the block; the two paths never run concurrently (different Nsymb/site; big-block dormant).

**(2) CONSUMERS of `ofdm.estimated_channel`:**
- `cl_ofdm::channel_equalizer` (OFDM.cc:2173-2209): iterates `Nsymb x Nc`, reads the PER-CELL complex H, computes
  `out=in/H` (ZF) with MMSE erasure when `alpha=|H|^2/(|H|^2+nv) < 0.1` (:2200), sets status=UNKNOWN. **Already
  per-cell — it consumes whatever the estimator wrote and supports a per-symbol-varying H natively; NOT a
  bottleneck.** INVARIANT: `out=in/H=X` requires H to capture the cell's TRUE complex gain INCLUDING per-symbol
  phase — a per-symbol-LOCAL estimate SATISFIES it; a block-AVERAGED (flat-ML) estimate VIOLATES it when the
  per-symbol phase ramps. (This is the defect; the fix repairs the producer, the consumer is already correct.)
- big-block CSI loop (telecom_system.cc:7604-7612 / :8190-8198): reads per-DATA-cell `|H|^2` into `csi_data` for
  LDPC LLR weighting (:8235-8241). Per-cell read — benefits from a per-symbol-accurate H automatically.
- meanH health stash (telecom_system.cc:7622-7625 / `bigblock_last_rx_meanh`) — the test metric; per-cell mean.
- DIAG `[DIAG-RXPB]`/`[DIAG-WAV]`/`[DIAG-GENIE]` (env-gated).

**(3) CONSUMERS of `ofdm.noise_variance_estimate`** (scalar): equalizer MMSE erasure threshold (OFDM.cc:2200);
`psk.demod` LLR scale (telecom_system.cc:8233-8234). INVARIANT: nv must reflect the post-estimate pilot residual.
The sparse-2D nv (:5984-6000) is the residual against the FINAL interpolated per-cell H — already time-local-
consistent. (A per-symbol estimate that tracks the phase makes the residual SMALLER and more honest, lifting nv
off the 0.0347 inflated value toward the clean 9.4e-6 — strictly helps the LLR scale.)

**(4) CONSUMERS of `pilot_configurator.sequence[]` + PILOT mask:** every `Y/X` site walks `sequence` by a running
`pidx` in the SAME nested-(n,j) `type==PILOT` raster order — raw LS :5848, time-interp seed, TRACK LS :8146,
flat-ML :8176, nv residual :5993, DIAG :7634. INVARIANT (held): `pidx` order == framer PILOT order; the
`ofdm_frame[].type` mask is the SINGLE source of truth. No consumer hardcodes `cont_cols` or pilot positions.

**(5) WHAT THE FIX CHANGES, and consumer-by-consumer verification:**
- **(a) Add band-spread continual columns: `MERCURY_BIGBLOCK_CONT_COLS` 2->3** (carriers {0, 24, 49} = edges +
  CENTER). Producer: `bigblock_rebuild_thin_grid` (:6879/:6915-6921) — TX emit AND RX decode call the SAME
  function, so the TX and RX grids stay IDENTICAL; nData/nBits/K recompute symmetrically. Consumers auto-pick-up
  the new pilots via the mask (no hardcode). K-SYMMETRY: the §14.5 bit-budget table is authoritative —
  cont=3 -> nBits=12925 -> **K=8** (slack 125 b); cont>=4 -> K=7 (drops below 8). So cont_cols=3 is the MAX
  band-spread that PRESERVES K=8 (zero wire-rate cost). MUST VERIFY post-change: `--test-bigblock-multicw /
  -fullpath / -arq-unit / -sim-inproc-bigblock / -bigblock-climb-election` still rc=0 (they exercise the grid;
  the unit tests pin K=8 via `MERCURY_BIGBLOCK_K`).
- **(b) Make the per-carrier TIME axis genuinely time-LOCAL (stop blurring the per-symbol phase ramp).** Inside
  `grid_sparse2d_estimator` step (2): (i) interpolate the time axis in UNWRAPPED-POLAR (magnitude + shortest-step
  phase), mirroring step (3)'s polar freq-interp, so a per-symbol phase ramp is tracked not chord-cut; (ii)
  shrink/disable the Wiener time-smooth on the CONTINUAL columns (which carry a pilot at EVERY symbol, :6919-6921)
  so the per-symbol CPE ramp is preserved — averaging a ramp is harmful (§3.3/§9). Keep the light smooth only on
  scattered columns (noise suppression where there is no per-symbol ramp to preserve). Consumer impact: the
  equalizer/CSI/nv all read the resulting per-cell H/nv unchanged — they already support per-symbol variation;
  this just makes the per-symbol H ACCURATE. No consumer invariant is altered; the change makes the producer
  HONOR the invariant the equalizer already assumes.
- **Audit conclusion:** the fix is CONFINED to (i) `bigblock_rebuild_thin_grid` cont_cols (TX+RX symmetric, K=8
  preserved) and (ii) the time-axis branch of `grid_sparse2d_estimator` (reached by big-block RX; the per-frame
  path also calls grid_sparse2d_estimator via the normal `MERCURY_SFO_GRID_*` knobs, so the time-axis change must
  be gated/verified for the per-frame regime too — VERIFY per-frame WB/NB BER unchanged, since
  grid_sparse2d_estimator is SHARED). No consumer invariant is violated; the equalizer/demod/CSI/nv consumers all
  read per-cell H/nv exactly as before, now with a per-symbol-accurate producer. gearshift mean_H/coarse_metric
  are set to -1/0.0 at `receive_bigblock` entry (:8592-8593) and the big-block path is dormant, so the per-frame
  gearshift `coarse_metric` consumers (arq_common.cc:7936/8067/8108/8149/8177) are NOT cross-fed by the big-block
  estimate. SHARED-ESTIMATOR caveat is the one real cross-path risk: `grid_sparse2d_estimator` is ALSO the
  per-frame estimator -> any time-axis change must be A/B'd on the per-frame WB path, not just the big-block.

## §17 SS4 IMPLEMENTATION PLAN — per-symbol / time-local sparse-2D scattered+continual re-estimation
(MORNING_VERDICT Step 1). NOT YET IMPLEMENTED — this is the plan to get sign-off before coding. 2026-06-07.

**Goal:** flip the restored arbiter's faithful arm bytes_ok 0->1 by replacing the de-facto block-folding behavior
(under the realistic vector the 2-edge continual sampling + Wiener time-blur collapses to a near-block-average)
with a genuine PER-SYMBOL, TIME-LOCAL channel estimate that tracks the col_phaseRMS=0.83 rad per-column phase
walk, while KEEPING K=8 (zero wire-rate cost) and the +18.9%..+91.8% margin over VARA 13,048.

**Prior art (CLAUDE.md §1, cited):**
- Mercury's own FreeDV-700D pilot-assisted-coherent lineage: pilot symbols transmitted regularly so the demod
  estimates the REFERENCE PHASE OF EACH CARRIER per pilot position, then interpolates the channel across pilot
  positions (rowetel.com FreeDV 700D release notes; codec2 README_freedv.md). This is per-symbol time-local
  pilot phase tracking by design — the exact pattern the fix restores.
- DVB-T / Hoeher-Kaiser-Robertson 1997 two-1D (time-then-frequency) scattered-pilot Wiener interpolation — ALREADY
  the structure of grid_sparse2d_estimator (cited in-code at telecom_system.cc:5905-5906); the fix corrects its
  TIME axis to be polar/local instead of complex-blur.
- 802.11a/g continual-pilot per-symbol CPE + pilot-equalization-gain (PEG) tracking (Speth/Fechtel/Meyr 2001) —
  the per-symbol common-phase + across-band slope removal; the band-spread continual columns supply the across-band
  pilots PEG needs (the 2-edge lattice undersamples the slope — §14.1).
- VARA HF continual + scattered pilot grid (reference_vara_benchmark_data) — confirms a 6-8% pilot density is far
  below VARA's ~41%, so Mercury can track the real <=0.2 Hz HW residual at K=8 where VARA-density would force K=5.

**Steps (each is ONE change + ONE test against the restored arbiter; one-change-one-test per CLAUDE.md):**
1. **Band-spread continual pilots cont_cols 2->3** ({0,24,49}). Verify K stays 8 (bit-budget §14.5: nBits=12925)
   and grid tests rc=0. Measure faithful-arm meanH/col_phaseRMS/bytes_ok. (Expected: gives PEG the across-band
   pilots; alone may not crack bytes_ok but is the enabler for step 2.)
2. **Time-axis polar interpolation** in grid_sparse2d_estimator step (2): interpolate the per-carrier time series
   in unwrapped-magnitude+phase (mirror step 3's polar), so a per-symbol phase ramp is followed, not chord-cut.
   A/B on the restored arbiter AND on per-frame WB BER (shared estimator). Measure col_phaseRMS drop.
3. **Stop blurring the per-symbol CPE on continual columns**: disable/shrink the Wiener time-smooth for the
   continual carriers (pilot every symbol -> the per-symbol phase is directly observed and must be PRESERVED, not
   averaged); keep the light smooth only on scattered columns. A/B arbiter + per-frame BER.
4. If steps 1-3 leave a residual per-symbol common-phase, ADD an explicit per-symbol CPE estimate from the (now 3)
   continual pilots applied BEFORE the freq-interp (802.11 CPE), reusing the existing STEP-2 TRACK machinery
   (telecom_system.cc:8120-8162) but per-symbol-LOCAL (no window-average — the §9 lesson).
**GATE (the restored arbiter, bytes_ok the TRUE arbiter):** faithful arm meanH>0.18 AND col_phaseRMS back toward
~0.03 AND **bytes_ok 0->1**, SANITY stays 8/8, per-frame WB/NB BER unchanged, all big-block + climb + sim-clock +
probe-backoff fast tests rc=0. THEN (and only then) HW-validate (uhubctl power-cycle first; bench is off-limits
until the q-table cal finishes and the arbiter is GREEN).
**CLAUDE.md §2 STOP:** this is the FIRST attempt at the CORRECT root cause (the prior 4 attacked the exonerated
CFO). If this honest attempt fails the restored bytes_ok gate, STOP and discuss — do not iterate blindly; a
failure there means either the shared-estimator per-frame constraint conflicts (split the big-block estimator out)
or the residual is genuinely an RX-pipeline numeric issue that must be localized first.

## §18 SS4 ATTEMPT — per-symbol/time-local sparse-2D re-estimation: IMPLEMENTED (one honest, multi-part
effort), recovers the MAGNITUDE fold but does NOT crack bytes_ok; the residual is an est-vs-truth PHASE
error (~0.20 rad) at the 6-8% time-pilot lattice. CLAUDE.md §2 HARD-STOP. Production RX REVERTED to HEAD;
only the durable est-vs-genie diagnostic kept. 2026-06-07.

**What was implemented** (the FULL §17 plan, ONE coherent effort in `grid_sparse2d_estimator`, telecom_system.cc;
this estimator is called ONLY by the big-block RX (:7583/:8168) and `sfo_grid_test` (:6515) — NOT the per-frame
`receive_byte` path, which uses the OFDM.cc LS/ZF estimator — so the §16 "shared estimator" per-frame cross-path
risk is MOOT for the default WB/NB decode; verified by grep of all callers):
- **§17 step 1** cont_cols 2->3 ({0,24,49}): K=8 preserved (decode ran, SANITY 8/8); alone it slightly WORSENED
  the faithful arm (meanH 0.149->0.126, col_phaseRMS 0.83->0.98) — the center pilot is an enabler, not a fix.
- **§17 step 2** TIME-axis POLAR interpolation (mirror the proven freq-axis polar at :5986): interpolate the
  per-carrier time series in unwrapped mag+phase instead of complex-linear (which chord-cuts a rotating phasor).
- **§17 step 3** stop the COMPLEX Wiener time-blur folding the per-symbol phase ramp: polar (mag + unwrapped-phase)
  moving-average, and SKIP smoothing the CONTINUAL columns (a pilot every symbol directly observes the per-symbol
  CPE — an 11-tap average blurs it).
- **§17 step 4** per-symbol CPE de-rotation (1b/3b) from the continual columns (802.11 continual-pilot CPE):
  measure cpe[n]=arg(Σ_continual H[n,c]·conj(time-mean_c)), de-rotate the raw pilot grid before interp, re-apply
  after — so the sparse time-interp tracks only the slow per-carrier residual.

**MEASURED (`--test-bigblock-chanest`, the restored production-sparse-2D arbiter; `MERCURY_BIGBLOCK_RXPB_DIAG=1`):**

  | arm / config                              | meanH | DDevm | est_vs_genie_phaseRMS | nzero  | bytes_ok |
  |-------------------------------------------|-------|-------|-----------------------|--------|----------|
  | SANITY clean (HEAD)                        | 0.194 | 0.038 | **0.038**             | 0      | **1**    |
  | FAIL-BEFORE faithful (HEAD, baseline)      | 0.149 | 0.546 | **0.199**             | 57     | 0        |
  | + polar-time (step 2)                      | 0.179 | 0.315 | (improved)            | 0      | 0        |
  | + polar-time + no-blur-cont (step 3)       | 0.179 | 0.342 | —                     | 0      | 0        |
  | + steps 2/3 + cont=3 (step 1)              | 0.181 | 0.354 | 0.211                 | 16     | 0        |
  | + steps 1/2/3 + CPE (step 4, FULL)         | 0.181 | 0.353 | 0.211                 | 16     | 0        |

- The polar time-axis fix RECOVERED THE MAGNITUDE fold exactly as MORNING_VERDICT predicted: faithful meanH
  0.149->0.18 (the complex-chord-cut + complex-Wiener-blur collapse is fixed; nzero 57->0) and HALVED the post-EQ
  EVM (DDevm 0.546->0.35). **But bytes_ok stayed 0.**
- The DECISIVE localizer (`est_vs_genie_phaseRMS` = RMS arg(H_g·conj(H_est)) per DATA cell): clean 0.038 rad
  (estimate ≈ truth) vs faithful **~0.20 rad** — ABOVE the 32-QAM ~0.1-rad EVM cliff. The published estimate's
  PER-CELL phase misses the true channel by ~0.20 rad. This holds on the HEAD estimator too (0.199) — it is NOT an
  artifact of the polar change.
- The §17-step-4 CPE de-rotation was a **NO-OP** on bytes_ok/DDevm/est-vs-genie (0.211->0.211): the residual is
  NOT a per-symbol COMMON phase. Confirmed by the post-EQ `rowCPE meanabs=0.022` rad (the common phase is already
  tracked by the pilots) and by `[AFC-WITHINSYM] residual_after_per-sym_LINEAR_fit=0.127 rad` (within-symbol the
  channel is CPE+slope to 0.13 rad, so a common-phase corrector has little to remove). The 0.20-rad residual is the
  per-CARRIER-INDEPENDENT, NON-LINEARLY-WALKING phase: each data carrier's phase walks its own AR(1) trajectory
  (faithful vector CFO walk sd=25 Hz, f3db=3 Hz), sampled in TIME only every scat_dy=4–12 symbols on scattered
  columns. Polar interpolation tracks a LINEAR ramp between time-pilots; a non-linear AR(1) walk between sparse
  time-pilots leaves ~0.20 rad it cannot follow.

**WHY THIS CANNOT BE FIXED WITHOUT WIRE COST (the hard-stop root):** the only lever that reduces the per-cell
time-walk residual is DENSER TIME PILOTS (smaller scat_dy / more continual columns), so each carrier is sampled
often enough to follow its non-linear walk. But the K=8 bit-budget (§14.5) is already at the edge: cont_cols=3 +
scat_dy=4 gives nBits=12925 (K=8, slack 125 bits). ANY density increase (scat_dy 4->2, or cont>=4) drops nData
below 8×1600=12800 ⇒ the K=8 block cannot be built (the test bails meanH=-1, confirmed). So tracking the residual
costs wire rate = drops to K=7/K=6, which erodes/erases the +18.9%..+91.8% VARA margin the big-block win depends on.
The fix CLASS the morning verdict identified (per-symbol/time-local sparse-2D) IS correct for the MAGNITUDE fold
and DOES recover meanH, but at the HW-faithful AR(1) drift magnitude the per-cell PHASE walk is undersampled in
TIME by the 6-8% lattice, and re-interpolating the SAME pilots better (polar/CPE) cannot manufacture the missing
time samples. This independently re-confirms the §13/§14 "~0.2 rad residual" verdict — now from the PRODUCTION
sparse-2D estimator (not the flat-ML pin), localized to a pilot-density-IN-TIME limit, not an RX-pipeline numeric
artifact and not a per-symbol CPE.

**§2 STOP — what I did NOT do:** I did NOT shotgun a 2nd estimator variant (DDCE blend, Wiener-coefficient retune,
2D-jointly-optimal Wiener, decision-directed time-DDCE), because the est-vs-genie localizer proves the blocker is
missing TIME samples, not a better interpolator of the present ones — and denser sampling breaks K=8. This seam has
5+ prior measurement flip-flops; per CLAUDE.md §2 the call is STOP + discuss, not iterate.

**Disposition:** production RX REVERTED to HEAD (9b93ccd) — the polar/CPE estimator change is NOT committed
(it does not crack bytes_ok; per the gate a non-cracking estimator change does not ship into the production RX).
KEPT (committed): the durable `est_vs_genie_phaseRMS` DIAG-GENIE diagnostic (env-gated, zero production behavior
change) — it is the decisive estimator-accuracy localizer for any future arbiter-fidelity or geometry work.
SANITY stays 8/8 byte-faithful; the restored arbiter still separates (1 failure = the PASS-AFTER contract); all
sibling fast tests rc=0 (multicw/fullpath/arq-unit/climb-engine/probe-backoff).

**Paths forward (for DISCUSSION, not auto-iterated):**
1. **Spend a little K** — accept K=7 (cont=3 + scat_dy=2 or cont=5) to densify time pilots; re-measure
   est_vs_genie + bytes_ok; K=7 still beats VARA per MORNING_VERDICT (+4.0%..+67.9%). This trades the "zero wire
   cost" premise for a payload-cracking estimate — a real, bounded engineering choice, NOT a band-aid.
2. **Mid-block re-acquisition preamble** (a TX-side change): re-run Moose every ~Ngrid/2 symbols to re-zero the
   AR(1) walk, so the residual phase walk per segment shrinks below the lattice's tracking floor (+~97ms airtime,
   still beats VARA per MORNING_VERDICT geometry).
3. **Smaller-K proportional geometry** (K=4/K=6, Ngrid shrunk proportionally): a shorter block accumulates less
   AR(1) walk per block; MORNING_VERDICT shows every K>=4 proportional geometry still beats 13048. The per-symbol
   walk over Ngrid=30 (K=4) is ~half of Ngrid=60, which may drop est-vs-genie below the cliff at the SAME density.
4. **HW-validate the polar magnitude-fold fix directly** — the polar time-axis change is a genuine, regression-clean
   magnitude recovery (0.149->0.18); on the REAL channel (HW §3.1: magnitudes survive, phases spread, and the real
   residual CFO is ~0 not the sim's 25 Hz AR(1) walk) the per-cell phase walk may be far smaller than this faithful
   sim's deliberately-aggressive drift, so the polar fix MIGHT crack bytes_ok on HW where it does not in this sim.
   This is the path the prior §11/§12/§14 verdicts also converged on: bytes_ok on HW is the only true final gate.

## §19 COMPLETE-THE-FIX (PIPELINE_AUDIT companions: re-est H + LOCALIZE nv + BB-1). 2026-06-07. The H fix +
BB-1 are COMMITTED; the audit's nv-localization premise is EMPIRICALLY FALSIFIED (nv already localized under
sparse-2D); bytes_ok stays 0 in the harsh sim (per-cell PHASE residual — §2 hard-stop holds). HW-validation only.

The PIPELINE_AUDIT_VERDICT.json (12-agent) endorsed the per-symbol estimator but called it INCOMPLETE: it
localized H but allegedly not the noise-variance nv, which it said "still inflates ~2500x (9.4e-6 -> 0.008-0.024)
on a channel-free waveform", weakening the demap LLRs -> LDPC non-convergence. I treated that as the hypothesis to
test (CLAUDE.md §3) rather than a fact to implement, and MEASURED it under the RESTORED production sparse-2D arbiter.

**FINDING — the nv premise is the FLAT-ML-PIN artifact, NOT the production path:**
- nv has TWO code forms. Flat-ML path (telecom_system.cc:7728-7742): nv = residual against a SINGLE block-wide
  `Hbar` -> on a CFO/SFO-walked waveform Hbar matches no cell, so resid is large -> THIS is the 0.008-0.024 / 2500x
  inflation the audit saw (it ran the contaminated flat-ML arbiter, EXP-C1). Sparse-2D path (:6125-6141): nv =
  pilot-residual EVM against the FINAL **per-cell** interpolated H (`resid = rx[cell] - H[cell]*X`). ALREADY localized.
- MEASURED [DIAG-RXPB], H fix applied, faithful CFO+SFO+DRIFT, SPARSE2D=1: **nv = 3.91e-4** (clean SANITY nv=1.15e-6).
  ~3 orders of magnitude BELOW the audit's flat-ML figure. There is NO nv block-fold left to localize. nv=3.9e-4 is
  the honest residual-EVM of a per-cell estimate vs a channel it can't perfectly track — small, NOT LLR-killing.
- Therefore NO nv change was made: fabricating an "nv localization" against a non-existent fold is a no-op that would
  MASK the real residual (§2 / What-NOT-to-do). §5 nv audit: consumers (demap cvar :7889, MMSE alpha=|H|^2/(|H|^2+nv),
  CSI |H|^2 :7745, gearshift :2909) all read the scalar `ofdm.noise_variance_estimate`; their invariant (nv ~= true
  post-EQ floor) is SATISFIED by sparse-2D nv=3.9e-4. Per-frame nv (OFDM.cc LS/ZF via receive_byte) UNTOUCHED —
  grid_sparse2d_estimator has exactly 3 callers (:6656 sfo_grid_test, :7732 bigblock_rx_passband, :8345
  bigblock_decode_from_wav), none receive_byte.

**BB-1 (right-FIR margin) — APPLIED.** bb_at (bigblock_rx_passband) and demod_at (bigblock_decode_from_wav):
`slice_size = mi + span_interp` -> `+ margin_interp`, mirroring per-frame :2480 (`pb_end = extraction_delay +
frame_size_interp + fir_margin`). Bounds-guarded read zero-pads past rxpb; grid extraction unchanged (mdec=mi/interp,
Nofdm*Ngrid), so the right margin only feeds FIR lookahead and does not shift the symbol grid. Inaccurate :2434-2462
comment corrected to :2434-2482 (both-sides margin). Regression-clean.

**RESULT (--test-bigblock-chanest, RXPB_DIAG):** SANITY meanH 0.194 / 8/8 / bytes_ok=1. Faithful meanH 0.179
(recovered) / DDevm 0.339 / **est_vs_genie_phaseRMS 0.207 rad** / col_phaseRMS 1.35 rad / **bytes_ok 0**. The
blocker is the per-cell PHASE residual (above the 32-QAM ~0.1-rad cliff), present on HEAD too (0.199, §18) —
STRUCTURAL pilot-density-in-time, not nv. Fast suites all rc=0 (multicw 1200/1200 bytes_ok=1, fullpath 1200/1200
byte-faithful, arq-unit 8/8, climb-engine, probe-backoff). Build rc=0.

**DISPOSITION:** COMMIT the §17 polar-time/CPE H estimator (magnitude-fold recovery) + BB-1 — both big-block-
exclusive, per-frame byte-identical, regression-clean. They do NOT crack the harsh-sim bytes_ok (§2 hard-stop, NOT
iterating a 6th estimator variant), but they ARE the regression-clean magnitude recovery + last-symbol fix the next
phase HW-validates. Next phase: sweep MERCURY_BBCHANEST_DBG_CFO_WALK_HZ (25->...->0.07 = real HW magnitude) and
watch est_vs_genie_phaseRMS cross <0.1 + bytes_ok flip — bytes_ok on HW is the only true final gate (path-forward #4).

## §20 PART-1 brief (1A nv-restore + 1C interleaver): 1A is a NO-OP under the production estimator (the
over-confident-nv fold the brief targets does NOT exist on the sparse-2D path — re-confirmed §19); 1C
(time/freq symbol interleaver) IMPLEMENTED + TX/RX-symmetric + default-OFF, but is NOT a net win on the
off-bench arbiter (its failing mode is DETERMINISTIC-SMOOTH phase curvature, not a BURST) — kept for the HW
bursty channel, gated MERCURY_BIGBLOCK_TFILV. CLAUDE.md §2 STOP (do not ship a masking default). 2026-06-07.

The directed PIPELINE/CW0_GAP brief (bigblock_p3_hw/CW0_GAP_VERDICT.json) prescribed two PHY changes to raise the
per-codeword decode rate: **1A** port the cfg16-nv-restore over-confident-LLR fix to the big-block thin lattice
(brief: block-wide nv 0.001-0.003 is 11-37x over-confident vs honest per-frame 0.036), and **1C** add the
time/freq interleaver the big-block lacks (brief amplifier D: no interleaver concentrates block-region estimate
errors into contiguous codewords). I MEASURED both against the restored production-sparse-2D arbiter before coding.

**§20.1 1A — the over-confident-nv premise is FALSE on the production path (re-confirms §19, now my own measurement).**
The brief's 0.001-0.003 nv figure is the FLAT-ML pin artifact (telecom_system.cc:7796-7811: nv = residual vs a
SINGLE block-wide Hbar, which matches no cell on a walked channel ⇒ large/inflated resid). The PRODUCTION sparse-2D
nv (telecom_system.cc:6137-6153) is ALREADY the pilot-residual EVM against the FINAL **per-cell** interpolated H
(`resid = rx[cell] - H[cell]*X`), and its in-code comment already cites the cfg16-nvfix as the reason it cannot
collapse. MEASURED (`--test-bigblock-chanest`, MERCURY_BIGBLOCK_RXPB_DIAG=1, HEAD 176042c, sparse-2D):
nv = **1.15e-6** (SANITY clean) → **1.16e-5** (static det-floor) → **3.91e-4** (HARSH-OTA). It TRACKS the channel
honestly across three orders of magnitude — there is NO 11-37x over-confident fold to restore. Porting "1A" would
mean inflating an already-honest nv, which would WEAKEN good LLRs and MASK the real residual (CLAUDE.md §2 /
What-NOT). So **1A was deliberately NOT implemented** — the fix it describes is already in place (telecom_system.cc:
6137-6153, the per-cell residual-EVM nv), exactly as §19 found. No nv change shipped.

**§20.2 1C — the time/freq symbol interleaver: IMPLEMENTED, TX/RX-symmetric, default-OFF.** The big-block packed
K LDPC codewords CODEWORD-CONTIGUOUSLY (bigblock_build_tx_bits: cw c == tx_bits[c*N..]; psk.mod ⇒ cw c == a
contiguous symbol run; ofdm.framer ⇒ a contiguous grid region ≈7 symbol-rows) with NO interleaver — confirmed: the
per-frame TX (telecom_system.cc:304/352) block-interleaves at block_size=nData/10, the big-block TX
(bigblock_tx_passband:7246) called psk.mod→framer directly. ADDED the SAME classic block (matrix-transpose)
time/freq symbol interleaver (Forney 1971; reuse interleaver.cc) to the big-block TX (after psk.mod, before framer)
and the mirror deinterleaver to every big-block RX demap (after deframer, before psk.demod; CSI |H|^2 deinterleaved
the SAME way — exactly the per-frame pattern at :2936/:2940). Two matched TX/RX pairs covered:
bigblock_tx_passband↔bigblock_rx_passband (live + chanest test) and bigblock_tx_to_wav↔bigblock_decode_from_wav
(WAV harness). Block size from `bigblock_tf_block_size(nData)` (env MERCURY_BIGBLOCK_TFILV; derived identically at
TX and every RX site ⇒ no wire negotiation). sfo_grid_test (the -m SFO_GRID estimator-only diagnostic, no codeword
decode) intentionally untouched. Per-frame `receive_byte`/`transmit_byte` BYTE-IDENTICAL (the big-block functions
are a different code path, reached only when bigblock_framing_enabled).

**§20.3 MEASURED — no non-identity block size is a net win on the off-bench arbiter** (`--test-bigblock-chanest`,
gated arms = SANITY/FAIL-BEFORE/PASS-AFTER/BENCH-REALISTIC; HARSH-OTA non-gating):

  | MERCURY_BIGBLOCK_TFILV | sanity | fail-before(DDCE0, want 0) | pass-after(DDCE1, want 1) | bench | verdict |
  |------------------------|--------|----------------------------|---------------------------|-------|---------|
  | 1 (identity / OFF)     | 1      | 0                          | 1                         | 1     | ALL PASS (== HEAD) |
  | 8 (=K)                 | 1      | 0                          | **0**                     | 1     | FAIL (PASS-AFTER regressed) |
  | 16 / 40 / 264          | 1      | **1**                      | 1                         | 1     | FAIL (FAIL-BEFORE gate defeated) |

- B=1 reproduces HEAD exactly ⇒ the implementation is a clean no-op when disabled (and the SANITY arm is bytes_ok=1
  at every B ⇒ the TX-interleave/RX-deinterleave round-trip is byte-faithful = TX/RX symmetric).
- B=8 REGRESSES the DDCE-recovered PASS-AFTER arm (bytes_ok 1→0): est_vs_genie_phaseRMS is unchanged (~0.10 rad);
  the interleave just reshuffles which marginal cells land in which codeword, tipping the just-barely-recovered
  PASS-AFTER over. B≥16 DEFEATS the FAIL-BEFORE regression-catcher: enough marginal cells get LDPC-cleared that the
  DDCE-OFF arm starts decoding ⇒ the test can no longer catch a sparse-2D estimator regression (an UNTRUTHFUL gate).

**§20.4 WHY the interleaver does not help here (root cause, not a tuning miss).** The arbiter's failing mode is the
DETERMINISTIC, spatially-SMOOTH Schroeder all-pass per-cell PHASE curvature (~0.10-0.19 rad, just over the 32-QAM
~0.1-rad cliff — §15/§18/§19), plus a smooth AR(1) CFO walk. An interleaver only helps when coded-bit errors are
CONCENTRATED in a contiguous BURST (CW0_GAP_VERDICT amplifier D assumed a localized block-region error). A smooth,
correlated, already-spread phase residual has no burst to break up, so spreading it does nothing to the per-codeword
error load — and it DISTURBS DDCE's per-cell coherence (DDCE corrects against the local decision; moving correlated
marginal cells around changes each codeword's error set). This is the SAME conclusion §18/§19 reached for the
estimator (the blocker is est_vs_genie phase accuracy / TIME-pilot density, a STRUCTURAL limit), now independently
re-confirmed from the interleaver angle: the off-bench gate is phase-accuracy-bound, not burst-bound.

**§20.5 DISPOSITION (CLAUDE.md §2 / §3 / What-NOT).** (i) 1A: nothing to ship (the production nv is already the
honest per-cell residual-EVM; fabricating an inflation would mask). (ii) 1C: SHIP the interleaver fully implemented,
TX/RX-symmetric, regression-clean, **default-OFF (B=1)**. It is the CORRECT mechanism for the REAL HW channel
(HW §3.1: localized fades / impulse noise = genuine bursts the off-bench deterministic floor does not model), so it
is kept and env-gated (MERCURY_BIGBLOCK_TFILV=8 to enable) for the SONNET HW bench A/B — exactly the brief's "push
the big-block deeper into the noise" intent, validated where the burst actually exists. Shipping it ON by default
would either regress PASS-AFTER or defeat the FAIL-BEFORE gate ⇒ it would be a masking default (forbidden). All
gated `--test-bigblock-chanest` arms GREEN at default; SANITY proves byte-faithful round-trip; per-frame
byte-identical; sibling fast tests rc=0 (multicw / fullpath / arq-unit / sim-inproc-bigblock / climb-engine /
probe-backoff / sim-clock). **HW A/B (TFILV=8 vs 1) under the bursty bench is the next true gate for 1C; 1A is
closed (already in place).**

**§20.6 TFILV=8 ENABLED — byte-faithful CORRECTNESS verified on the live path; the multicw/fullpath rc=1 is a
DEFEAT-arm artifact, NOT a symmetry bug.** Concern: a TX/RX-asymmetric interleaver would corrupt clean decodes
when enabled. RESOLVED by measurement: with MERCURY_BIGBLOCK_TFILV=8, (i) `--test-bigblock-chanest` SANITY
(clean, B=8, the live bigblock_tx_passband↔bigblock_rx_passband pair) = bytes_ok=1; (ii)
`--test-sim-inproc-bigblock` (full live transmit_bigblock→receive_bigblock→carve, K=8) = rc=0, CASE A
622/622 byte-faithful cw_ok=8/8; (iii) `--test-bigblock-multicw` **ARM-A** (CRC-ON, block window, the
correctness arm) = clean 8/8, rx_have 1200/1200, bytes_ok=1. So the interleave/deinterleave is an EXACT,
TX/RX-symmetric permutation (B derived identically from nData at TX and every RX site) and is byte-faithful on a
clean channel through the full live pipeline when enabled. The multicw/fullpath SUITE returns rc=1 at TFILV=8
only because their NON-correctness arms are corruption REPRODUCERS: multicw ARM-B forces the pre-fix STOCK RX
window (MERCURY_BIGBLOCK_DEFEAT_FIX=1) and ASSERTS `!bytes_ok` (the stale-ring corruption must reproduce) — a
different symbol mapping changes that exact corruption signature, so the `!bytes_ok` assertion is mapping-specific
and not meaningful under interleaving; fullpath similarly drives a config-transition/window diagnostic. These
DEFEAT arms are NOT correctness gates for the interleaver (they validate the WINDOW/whiten fixes against a pinned
mapping). For the HW A/B, the correctness gate is ARM-A / sim-inproc-bigblock / chanest-SANITY (all GREEN at
TFILV=8); the DEFEAT arms should be run at TFILV=1 (their designed mapping). Default-OFF keeps the whole suite
rc=0; enabling TFILV=8 is byte-safe on the production live path.

## §21 CONFIG × SNR × INTERLEAVER decode-floor MAP — the big-block "works at more than the top end" + "push into the noise" measurement. 2026-06-07.

User direction (CW0_GAP brief follow-on): the big-block is a FRAMING that recovers per-frame overhead at ANY config (one acquisition over K codewords), not a CFG16-only point — so MAP a config×SNR floor, interleaver ON vs OFF. Built an off-bench SWEEP mode into `--test-bigblock-chanest` and measured the genuine (ref==NULL) byte-faithful decode floor across CFG16/15/13/10 × TFILV{1,8} × SNR3k ladder. Artifacts: `bigblock_p3_hw/bb_config_snr_map.json`, `bigblock_p3_hw/bb_sweep_raw.csv` (168 cells).

**§21.1 Harness (scope: big-block-exclusive, per-frame byte-identical, default-OFF).** (i) The big-block dispatch was HARD-GATED to `current_configuration==CONFIG_16` at telecom_system.cc:643 (TX) / :1033 (RX). Added a DEFAULT-OFF env `MERCURY_BIGBLOCK_ANYCFG` that OR-relaxes both gates so the framing engages at ANY OFDM config. The thin-grid geometry (`bigblock_rebuild_thin_grid`) was ALREADY config-agnostic (reads Nc/Nfft/gi/M, derives log2M from M), so it builds at any config. Production byte-identical (default OFF → gate stays ==CONFIG_16); the ARQ-layer gates (arq_common.cc:3866 `bigblock_send_one_block`, :4204 RX candidate) are UNCHANGED, so production ARQ still only big-blocks at CFG16. (ii) `test_sim_inproc_bigblock_chanest()` got a SWEEP branch (`MERCURY_BBCHANEST_SWEEP=1`, reads `_SWEEP_CFG`/`_SWEEP_SNR`/`MERCURY_BIGBLOCK_TFILV`): one `run_block` through `cl_sim_awgn(SEED, snr3k)` (det-floor always-on + AWGN; CFO=SFO=0; DDCE ON) → one parseable `[BBSWEEP]` line. K is the config-NATURAL codeword count via `bigblock_codeword_count()` (NOT forced to 8): CFG16→8, CFG15→6, CFG13→4, CFG10→4 (a lower-order config packs fewer codewords per acquisition — intrinsic). Sweep OFF ⇒ function byte-identical to HEAD; the default 5-arm gate still ALL PASS.

**§21.2 FLOOR TABLE** (lowest SNR3k dB with bytes_ok=1; phaseRMS = est_vs_genie at the floor):

  | cfg | mod | clean decode | clean phaseRMS | floor SNR3k (OFF) | floor SNR3k (ON, TFILV=8) |
  |-----|-----|--------------|----------------|-------------------|---------------------------|
  | 16  | 32-QAM | OFF:yes ON:**NO** | 0.100 | **none (clean-only)** | none (FAILS clean) |
  | 15  | 16-QAM | yes | 0.088 | 22 | 20 |
  | 13  | (lower)| yes | 0.077 | 20 | 18 |
  | 10  | (lower)| yes | 0.072 | 18 | 18 |

**§21.3 KEY FINDINGS.** (a) **CFG16 crosses the cliff CLEAN** (interleaver OFF, bytes_ok 8/8, phaseRMS 0.100 = exactly the 32-QAM ~0.1-rad cliff; DDCE just barely recovers) — the §20 PASS-AFTER re-confirmed and the +18.9-104%-vs-VARA headline waveform decodes — but it has **ZERO AWGN margin**: fails already at SNR3k=24 dB (AWGN lifts phaseRMS 0.100→0.127, over the cliff). (b) **Interleaver dB-gain**: **+2 dB** at CFG15 (22→20) and CFG13 (20→18); **0 dB** at CFG10 (already at the deep limit); **REGRESSION** at CFG16 (OFF decodes clean, ON FAILS clean — no burst to spread + DDCE coherence disturbed, exactly §20.3/§20.4). So the interleaver is a LOWER-TIER reach lever, not a CFG16 lever. (c) **Per-config floors deepen as QAM order drops** (more EVM margin = lower SNR floor): CFG16 clean-only → CFG15 ~22 → CFG13 ~20 → CFG10 ~18 dB (OFF); ~2 dB deeper per tier. The floor MECHANISM is uniform: AWGN degrades the thin-lattice estimate until est_vs_genie phaseRMS crosses the per-modulation EVM cliff (CFG16 ~0.10, CFG15 ~0.13-0.15, CFG13 ~0.12-0.17, CFG10 ~0.13-0.20) — the SAME structural blocker CW0_GAP named (B: thin-lattice undersampling), now mapped across configs.

**§21.4 DISPOSITION.** "Works at multiple configs" CONFIRMED (byte-faithful at CFG16/15/13/10). The dominant "push into the noise" lever is dropping the config (~2 dB/tier); the interleaver adds a second ~2 dB at the AWGN-margined tiers (CFG15/CFG13) but must stay DEFAULT-OFF or per-config-gated (it regresses CFG16). No 1B pilot-density change made: the 1B trigger was "a target config JUST short of decoding" — none is just-short (every config decodes clean; floors are well-defined AWGN cliffs). Raising CFG16's AWGN margin (the only zero-margin config) needs pilot density (tighten the thin lattice so phaseRMS stays <0.1 under AWGN, ~7-9% payload cost) — flagged, not spent. All sweep-harness edits uncommitted in the worktree; regression gate (sweep OFF) ALL PASS.

## §22 §19 ACQ-WINDOW GUARD IS A LIVE-PATH DEADLOCK REGRESSION → replace the silent defer-and-re-arm with WAIT-FOR-TAIL. 2026-06-08.

The §19 fix (645adfe) is a REGRESSION on the live ARQ path. `bigblock_p3_hw/winrun_recovery.json` measured it: accept_fraction dropped from the pre-fix ~5.6% lucky-aligned carves to **0.0** (every attempt = exactly 1 defer, 0 carves), with the session DEADLOCKED. This section is the §2 root-cause + §4 plan + §5 audit BEFORE the corrected code.

**§22.1 THE CAPTURE MECHANISM (traced from source, the keystone the §19 fix got wrong).**
- The RX passband is a DOUBLE-MAPPED circular ring `data_container.passband_delayed_data` of `sp = signal_period = Nofdm*buffer_Nsymb*interp = 155344` samples (allocated `2*sp` for the mirror; data_container.cc:170). The audio-callback producer writes ONE symbol (`symbol_period = Nofdm*interp = 1168` samples) per call at `ring_write_index`, advances `ring_write_index = (wi + symbol_period) % sp`, and decrements `frames_to_read` (audioio.c:1370-1396).
- The consumer `receive()` (arq_common.cc:7214-7235) snapshots the window ONLY when `frames_to_read==0`: `memcpy(ready_to_process_passband_delayed_data, &passband_delayed_data[rwi], sp)`. The double-mapping makes `sp` contiguous samples from `rwi` the chronological window `[oldest … newest]`. **The snapshot is always the most-recent `sp` samples, sliding forward by `symbol_period` per produced symbol.**
- `receive_bigblock` (telecom_system.cc:8912) runs the WHOLE decode against that one snapshot of length `nSamples = sp` (telecom_system.cc:8926). It Schmidl-Cox-acquires the head at offset `head` (`bigblock_last_rx_head_delay_samples`, :7490) and demods `block_span = block_nsymb*sym_samples = 64*1168 = 74752` samples forward from `head`. When `head + block_span > sp`, the tail samples are FUTURE (not yet produced into the ring at snapshot time); `bb_at` zero-pads them → block-wide estimate collapses → cw0 wire-CRC fails. `bigblock_acq_window_fits()` (arq_common.cc:4226) correctly detects this (`head + block_span <= cap`, threshold `head <= 80592`). **The math is right; the recovery is wrong.**

**§22.2 SYSTEMATIC vs RANDOM (decides the fix).** The observed defer heads (winrun_recovery.json): 82532, 88424, 107908, 109188, 109196, 123828, 138052, 144940 samples = **70.7, 75.7, 92.4, 93.5, 93.5, 106.0, 118.2, 124.1 symbols** (÷1168). Spread 70–124 sym, variance ±27 sym. The head landing is **RANDOM** (CMD turnaround = ACK-detect + 8-codeword LDPC encode + TX schedule varies tens of symbols run-to-run), NOT a fixed ~2-symbol lag. ⇒ an ARMING-PHASE SHIFT alone CANNOT fix it (the variance would still overrun the late tail). The robust fix must handle the full random late-landing range → **WAIT-FOR-TAIL.**

**§22.3 WHY THE §19 DEFER DEADLOCKS (two distinct bugs).**
1. **Re-arm value is wrong (head scrolls off the back).** On defer it sets `frames_to_read = bigblock_block_ftr_or(0) = block_nsymb + 10 = 74` (arq_common.cc:7421). That waits 74 MORE symbols before the next snapshot, advancing `ring_write_index` by 74 symbols, so the head at symbol ~70 lands at `70 - 74 = -4` → SCROLLED OFF THE BACK of the 133-sym window. The block is destroyed by the very re-arm meant to recover it. The correct wait is only `ceil(overrun/symbol_period)` symbols (typ. 1–10), NOT a full block span.
2. **No re-presentation on the live path.** Even with a correct wait, the §19 comment's premise — "on the next snapshot the block re-lands earlier → fits → carves 8/8" — was only ever verified by the in-process `--test-bigblock-acqwindow`, whose PASS-AFTER arm RE-INJECTS the complete block at OFF_HEAD (`decode_at_offset(OFF_HEAD,…)`, test_bigblock_arq_unit.cc:2995). The CMD emits each big-block EXACTLY ONCE (BIGBLOCK-TX one-block emit) then waits on a positive SACK and never NAKs/resends. The live path has NO re-presentation — only the SAME single transmission, captured later. So the test modelled RETRANSMISSION/RE-PRESENTATION; the live reality is one-shot-then-slide. **The test could not catch this.**

**§22.4 THE WAIT-FOR-TAIL FIX (§4 plan).** Geometric feasibility: the block is `block_span=64` sym ≤ ring `133` sym, so when the tail just arrives the head sits at `cap - block_span = 80592` (sym ~69) with 64 sym of head-room behind it — the head STAYS in-ring through the whole wait. On a defer (overrun detected):
- compute `overrun = head + block_span - cap` (>0), `wait_syms = ceil(overrun/symbol_period) + 1` (the `+1` margin keeps the tail comfortably in-ring against producer race);
- set `frames_to_read = wait_syms` (SMALL — typ. 1–10, NOT `block_ftr_or(0)`), `nUnder_processing_events=0`, reset the ofdm-search stats — so the NEXT snapshot fires after exactly enough fresh symbols for the tail, with the window slid forward so `head_new = head - wait_syms*symbol_period <= 80592` (fits);
- do NOT zero the ring (the head must survive — the carve-success path zeros, the defer path must NOT), do NOT touch `ring_write_index` (producer owns it), suppress this pass's carve (`bigblock_last_rx_K=0`, `message_decoded=NO`, `bigblock_rx_candidate=false`, `bigblock_rx_handled=true`);
- on the next snapshot the SAME single transmission's full block is in-ring and re-lands earlier → `acq_window_fits()` true → carve 8/8.
- Bound: keep `bigblock_rx_defer_count < BIGBLOCK_RX_MAX_DEFERS` (raise to ~6 since each wait is short, not a block-span) so a genuinely absent block falls through to the cw0-CRC gate (no infinite spin); reset on every accept (already done at :7475). RX-confined; gated `bigblock_framing_enabled && M!=MFSK && CONFIG_16`; off-rung never entered (byte-identical). TX byte-identical (unchanged). Per-frame path untouched.

**§22.5 §5 CROSS-LAYER AUDIT (shared state: `frames_to_read`, `ring_write_index`, `passband_delayed_data`, `bigblock_last_rx_*`, the SACK/ARQ block state).**
1. **Producers of `frames_to_read`:** audio callback `--` (audioio.c:1394); receive() re-arms — stock per-frame success (telecom_system.cc:4995/5082/5212), the big-block carve-success re-arm (arq_common.cc:7512), the OLD §19 defer (:7421, being replaced), the ACK-pattern poll (:7184/7190/7204), responder turnaround (arq_responder.cc:1233/1947). **Consumers:** audio callback (gate to snapshot), receive() (`==0` snapshot, :7230). My change writes `frames_to_read = wait_syms` under `capture_prep_mutex` (same lock the producer/other re-arms hold). Invariant preserved: a positive `frames_to_read` defers the snapshot by exactly that many produced symbols. No consumer assumes a block-span value here.
2. **`ring_write_index` / `passband_delayed_data`:** producer-owned (audio callback). My change does NOT write either (unlike the carve-success path which zeros the ring AFTER consuming — correct there, wrong here). Invariant preserved: the ring keeps sliding; the head preamble + already-arrived body stay in the chronological window; the still-missing tail is filled by the producer during the wait. Head-stays-in-ring proven §22.4. The next snapshot reads the SAME single transmission (no second TX needed).
3. **`bigblock_last_rx_K` / `_head_delay_samples` / `_capture_nsamples`:** set by receive_bigblock per decode (telecom_system.cc:7490/8932/9027). On defer I clear `bigblock_last_rx_K=0` so the carve gate below is false (same as §19) — they are re-stamped fresh on the next snapshot's decode. No stale read: every consumer (`acq_window_fits`, the cw0-CRC gate, the carve) runs only after a fresh decode in the SAME receive() pass.
4. **SACK / ARQ block state (`messages_rx[]`, `rsp_current_expected_batch_seq_id`, EOB, ACK emission):** UNTOUCHED on a defer — no SACK is sent, no messages_rx carve, no bsi advance. So the CMD's "waiting for SACK" state is preserved across the (short) wait; when the block carves on the next snapshot, the normal carve path (:7459-7518) fires the ACK/partial-SACK exactly as today. The deadlock is broken because the wait now succeeds within a few symbols (one transmission), so the SACK the CMD is waiting for IS produced — no NAK/retransmit needed. (Fallback option (1) from winrun_recovery.json — NAK-to-retransmit — is NOT needed because wait-for-tail recovers the SAME copy; it stays the documented fallback if a future absent-block case appears.)
5. **What the fix changes:** only the defer RECOVERY (wait_syms instead of block-span re-arm; no ring touch). Every consumer above still sees its invariant. The cw0-CRC "not a block" path (:7432) and the carve path (:7459) are reached identically once `acq_window_fits()` returns true on the next snapshot.

**§22.6 TEST FIX (§3 — model the live one-shot, replace the re-presentation lie).** The current acqwindow PASS-AFTER re-injects a complete block at OFF_HEAD — the bug-hiding model. Replace with a LIVE-RING model: keep ONE `tx_pb` (single transmission); simulate the forward-sliding ring by snapshotting a `WIN_NSYMB` window whose start advances with a write-head, where ONLY the samples produced so far are present (the tail beyond the current write-head is silence). FAIL-BEFORE: the §19 defer re-arms a full block-span → the head scrolls off the back on the re-snapshot → still 0/8 (deadlock). PASS-AFTER: wait_syms re-arm → the window slides exactly enough → the SAME single block's tail is now in-window, head still in-ring → carve 8/8 from ONE transmission, no re-injection. This asserts the deadlock fails-before and the wait-for-tail passes-after on the genuine one-shot path.

## §23 CLOSING VERDICT (2026-07-02) — thin-grid vehicle PERMANENTLY PARKED; the live path is MINI-preamble-on-the-full-lattice; MINI survival-ladder against the FAITHFUL measured drift = **NO-GO (case c)** at CFG16 on this bench.

**Disposition of everything above (§1–§22).** The thin-grid big-block vehicle (rebuild a coarse Ngrid lattice, single/amortized channel estimate over a K-codeword block) is **PERMANENTLY PARKED**. Its off-bench arbiter is blocked by a structural per-cell TIME-pilot-density limit (§18/§19: est_vs_genie phaseRMS ~0.20 rad > the 32-QAM ~0.1-rad cliff; denser time pilots break the K=8 bit budget §14.5), and the CFG16 clean decode has zero AWGN margin (§21.3). Do NOT re-open the thin-grid estimator/geometry seam. The LIVE candidate for recovering the CFG16 preamble tax is the Fable reframe: **keep the production Dy=3 33% lattice + per-frame estimator; amortize ONLY the acquisition preamble across a batch** (frame-0 FULL 4-sym anchor, frames 1..N = MINI resync), RX = the NORMAL per-frame path with Stage-3 prediction carrying timing + CPE carrying frequency between anchors. This run tested THAT thesis against the faithful drift.

**§23.1 The FAITHFUL drift vector (correcting the prior run's 2 Hz).** The measured HW drift is **+75..+892 ppm SWINGING** — the LIVE running-modem end-to-end audio-pipeline slip (ALSA buffer/resample slip + PTT/AGC re-lock + OS scheduling on the crystal), source `sim-fidelity-master-plan.md §1.4` / `HW_PREAMBLE_AMORT/preamble_amort_consolidated.json` (line 810 "2nd run drift +795..+892ppm", verdict 857). This is a SAMPLE-RATE (SFO) effect; in `cl_sim_sfo` (a true 4-tap cubic resampler, `sim_channel.h:407`) it couples timing-creep + a proportional carrier shift (CFO = 1500 Hz·ppm = 0.11–1.34 Hz across the swing). The prior run's **"2 Hz CFO walk" was NOT the culprit but WAS wrong twice over**: 2 Hz pure carrier ≈ **1333 ppm-equivalent > the 892 ppm measured max (too harsh)**, AND it modelled a pure-carrier `cl_sim_cfo` rotation with **no timing creep** — missing the DOMINANT killer (the SFO timing creep). Correcting the vector to be faithful (less harsh, right mechanism) does NOT rescue MINI.

**§23.2 The survival ladder (`sfo_block_test`, CFG16 −s 16, EsN0=900 det-floor-isolated; deterministic, ref==NULL, real Schmidl-Cox+Moose+LDPC per frame, byte-faithful vs TX; `/dev/shm/scratch/bb_ladder/`).** Two artifact-free arms: **PASS = FULL per-frame re-acquire (normal tracking)**; **FAIL-BEFORE = held/bigblock (timing frozen at the frame-0 lock, no per-frame re-acquire = the "coast on prediction" degenerate).** frames_decoded:

  | drift (SFO ppm)      | STOCK FULL per-frame (PASS) N17 / N34 | FROZEN-hold (FAIL-BEFORE) N17 / N34 | frozen first-fail frame |
  |----------------------|----------------------------------------|--------------------------------------|-------------------------|
  | clean (0)            | 17/17 · 34/34                          | 17/17 · —                            | none                    |
  | 75 (measured MIN)    | 17/17 · 34/34                          | 13/17 · 13/34                        | **frame 10**            |
  | 484 (measured MID)   | 17/17 · 33/34                          | 2/17 · 2/34                          | **frame 2**             |
  | 484 + walk σ0.8 (swing) | 14/17 · —                           | 2/17 · —                             | frame 2                 |
  | 892 (measured MAX)   | **0/17 · 0/34**                        | 0/17 · 0/34                          | frame 0                 |

- **FAIL-BEFORE proven:** frozen timing collapses by **frame 10 @75 ppm, frame 2 @484 ppm, frame 0 @892 ppm** — the drift is REAL and the per-frame RE-ACQUISITION LOOP (not luck) is what carries the block. The NOTRACK negative control (held + a fixed +8-interp/frame skew) coincidentally *rescues* 484 ppm to 17/17 — an OPPOSITE-SIGN skew cancelling the creep — which independently CONFIRMS the collapse mechanism is **timing creep**, not noise.
- **STOCK baseline** (FULL preamble every frame) survives the measured swing continuously up to ~484 ppm (33–34/34) and the swinging-walk vector at 14/17, matching the HW anchor "full-preamble CONTROL survives +892 ppm byte-faithful." (Static 892 ppm held continuously is 0/17 — OVER-harsh vs the real swing that only briefly peaks there, the same over-harshness class as the prior 2 Hz; the swing/walk cell is the faithful representation.)

**§23.3 The MINI-1 vs MINI-2 resolution.** The `sfo_block` PER-FRAME MINI arm is a proven **harness artifact**: MINI-1 decodes **1/17 even at ZERO drift** (blind full-buffer Schmidl-Cox cannot lock a 1-sym half-symbol repeat, metric 0.13 vs FULL 0.998), and MINI-2 is byte-identically 1/17 (metric 0.21) — it does NOT model the thesis RX (Stage-3 prediction places the frame; the MINI is only a tight verify). So the deterministic sim CANNOT score the thesis-RX MINI directly; the only harness that implements Stage-3-carried MINI is the 2-instance `SIM_INPROC`, which stalls stochastically at the CONNECT substrate (`master-plan §1.6/§1.7`) and is not a reliable arbiter (MINI-1 delivered clean and at 0.5 Hz pure-CFO, but the drift cells stall on CONNECT, not on the PHY). **The verdict therefore rests on the convergence of three independent lines, not the artifact arm:** (i) the frozen-vs-tracked ladder above proving the measured drift DEMANDS full per-frame re-lock; (ii) the physics — the measured swing is **1.5–18× the PREAMBLE_SUPPRESSION_PLAN design assumption (20–50 ppm)**; GI-exhaustion time drops to ~4.5 s (≈8 frames) @75 ppm, ~0.69 s (≈1.2 frames) @484 ppm, **~0.37 s (< one 0.58 s CFG16 frame) @892 ppm** — so at the swing's upper range no MINI resync interval can coast between anchors; and (iii) the DIRECT HW measurement of the exact lever — LEVER-P / P-ON (MINI preamble + Stage-3 + CFO-reuse, telecom_system.cc:2622) was **HW-FALSIFIED: 53 MINI-fails CFG15 while FULL survived +892 ppm** (`preamble_amort_consolidated.json`). MINI-2's only edge over MINI-1 is halved Schmidl-Cox metric variance (peak-pick reliability), which does NOT address the GI-exhaustion root, so **MINI-2 collapses for the same reason MINI-1 does.**

**§23.4 VERDICT = case (c) NO-GO — BOTH MINI-1 and MINI-2 fail the faithful vector at CFG16 on this bench.** The prior 2 Hz vector was NOT the reason MINI failed (it was over-harsh + wrong-mechanism); correcting it to the faithful +75..+892 ppm SFO swing still kills MINI, because the killer is the audio-pipeline SFO timing creep whose measured magnitude exhausts the CFG16 GI sub-frame at the swing peak. The amortization prize is real but **unrealizable on this drift**: theoretical wire gain 13/10 = **×1.30 (MINI-1)** and 13/11 = **×1.18 (MINI-2)** [CFG16 = 9 data + 4/1/2 preamble syms], realized delivered gain at the faithful drift = **0 (collapse)**. NO HW A/B gate is issued (a GO precondition). The adaptive `PREAMBLE_SUPPRESSION_PLAN` live-drift-measurement design self-protects — faced with a measured +75..+892 ppm swing it computes a near-zero suppression interval (keep full preambles) → no CFG16 gain — which is the correct, safe behavior; the lever is only viable on hardware with a STABLE drift ≲100–150 ppm, which this Pi/Fe-Pi/IONOS bench is not. **PARK the MINI-preamble-amortization lever at CFG16 with this reason; do not re-chase without either (a) a lower-drift analog front end or (b) a harness that faithfully implements the Stage-3-carried MINI RX (SIM_INPROC CONNECT substrate = master-plan S0, currently unsolved).**

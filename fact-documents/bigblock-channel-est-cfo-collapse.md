# Big-block channel-estimation collapse — un-tracked residual CFO

Branch: `fix/bigblock-chanest` (worktree `C:/Users/kamer/mercury_wt/bb-chanest`, off `monitor` be80936)
Status: **localized + genuine off-bench reproduction built; fix NOT landed (one principled attempt refuted).**
Off-bench artifact: `bigblock_p3_hw/results_bb_chanest_fix.json`.

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

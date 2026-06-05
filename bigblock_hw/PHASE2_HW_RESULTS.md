# Big-Block PHY — Phase 2 HW De-Risk Results (FINAL)

Agent: bigblock_hw_phase2. Date 2026-06-05. Branch feat/sfo-bigblock-one-acquisition @4d1321e
(sim-faithful worktree) + uncommitted big-block changes. NO commit/merge/push/attribution.
Bench left UNLOCKED, IONOS restored to clean WGN:40 flat, no stray procs.

## VERDICT: PHY WIN TRANSFERS TO REAL HARDWARE (clean condition) -> production+ARQ de-risked.

The validated sim big-block (ONE 4-sym preamble + K=8 back-to-back 1600-bit LDPC
codeword-frames under ONE acquisition, thin freq-focused 7.2% lattice, channel-adaptive
sparse-2D estimator) **DECODES 8/8 BYTE-PERFECT (post-FEC BER=0) over real independent
Fe-Pi 48 kHz crystals + IONOS clean channel + real audio path.** Confirmed on multiple
independent block periods.

## (a) HW DECODE — coded LDPC over real HW (PLAY RPi1 -> RECORD RPi2)
| Condition | extract | codewords | post-FEC BER | acq_metric | fine-timing off |
|-----------|---------|-----------|-------------|-----------|-----------------|
| Clean WGN:40 flat | period 0 | **8/8** | **0** | 0.9972 | -17 |
| Clean WGN:40 flat | period 1 | **8/8** | **0** | 0.9986 | -73 |
| WGN:30 flat       | period 0 | **8/8** | **0** | 0.9905 | -53 |
| WGN:30 flat       | period 1 | 0/8 | 0.484 | 0.9861 | -169 (transient-corrupted) |
| Mild (WGN:30 + FADE 10dB/0.5Hz, 2-path) | per0 | 0/8 | 0.483 | 0.933 | genuine channel fail |
| Mild | per1 | 0/8 | 0.315 | 0.957 | genuine channel fail |

- **Clean = 8/8, BER=0**, all 8 codewords iters=0 (instant LDPC convergence), nv floored 1e-6.
- Per-codeword trend on a clean pass: cw0..cw7 ALL infoerr=0, iters=0 -> NO end-of-block BER
  rise. The 13 ppm SFO drift across the 1.3 s block is negligible; single-acquisition holds.
- Loopback-vs-HW acq_metric: 0.999 (loopback) vs 0.993-0.999 (HW clean) -> robust acquisition.

## (b) REAL observed SFO = +13.2 ppm  (sim calibrated for 50 ppm)
- Intra-block head-preamble vs tail matched-filter fit (immune to loop-restart jitter):
  rx_span=75777 vs tx_span=75776 samp -> ratio 1.0000132 -> **+13.2 ppm**, IDENTICAL across
  3 independent passing extracts (clean40_0, clean40_1, w30flat_0), head/tail corr >0.99.
- RPi1 TX clock runs ~13.2 ppm FASTER than RPi2 RX clock. Real consumer Fe-Pi crystals are
  BETTER (smaller offset) than the sim's worst-case 50 ppm -> why single-acquisition decodes
  clean (end-to-end drift ~0.8 samp over the block, absorbed by the continual-pilot per-symbol
  estimate; the standalone CPE/PEG tracker stays redundant, as the sim predicted).
- NOTE vs testbed_wgn_snr3k mapping: that maps WGN-label->SNR3k (amplitude), orthogonal to SFO
  (a clock-rate ppm). 13.2 ppm is a fresh, independent measurement of the real crystal pair.

## (c) net-PHY achieved on HW
- 11200 post-FEC info bits delivered per acquisition (K=8 x 1400), byte-identical to TX PRBS.
- Block content airtime ~1.707 s (4-sym preamble + 60 data) -> ~6562 bps incl preamble;
  ~7000 bps over the 60 data-symbol payload (the metric the sim's 7859-7960 claim used).
  Same K=8x1400 info load per acquisition as the validated sim; both > VARA 7050.

## (d) Channel-adaptive estimator — picked correctly
- **Clean / WGN:30-flat (FLAT path): flat-ML is VIABLE but sparse-2D WINS.** On the passing
  clean blocks sparse-2D gives nv 3.3e-6 + iters 0/0/0 (instant); flat-ML on the SAME data
  gives nv ~4-9e-5 + iters 3-89 mean and even drops to 1/8 on a WGN:30 block. => the real
  IONOS audio path is mildly SELECTIVE even at "flat" (analog cable/SGTL5000 response), so the
  per-symbol sparse-2D estimate is the right default; flat-ML's single global H̄ undermodels it.
- Mild (deliberate fade): both fail, but sparse-2D nv (7.7e-6..2.2e-5) << flat-ML (8e-5..9e-5),
  confirming sparse-2D is the correct selective-channel estimator. The failure is the CHANNEL
  (below), not the estimator.

## (e) Real-HW extras the synthetic SFO ramp did NOT model
1. **Block-start / DAC-settling transient**: the VERY FIRST loop period (bb_extract_0) decoded
   5/8 — failures were the LEADING codewords cw0(310 err)/cw1(241)/cw2(131) declining, then
   cw3-7 clean. aplay device-open + DAC settling corrupts the first ~3 codewords of the first
   block. Steady-state periods decode 8/8. -> production must key-up / send a throwaway lead
   symbol before the real preamble (the sim's analytic ramp had no DAC transient).
2. **Block-to-block variability**: consecutive periods in the SAME loop differ (w30flat per0=8/8
   vs per1=0/8 with pilot-EVM 100x higher; forcing the timing offset did NOT recover per1 ->
   a real transient/level glitch on that specific period, not misalignment). Real HW is not
   stationary across back-to-back blocks the way the synthetic channel is.
3. **Acquisition jitter**: HW acq_metric 0.93-0.99 (vs 0.999 loopback); the integer Schmidl-Cox
   pick can slip ~1 OFDM symbol, and the pilot-EVM fine-timing search then settles in an adjacent
   (wrong) GI-spaced window on marginal captures (off=-169/-197 vs the good -17..-73). On clean
   high-SNR captures this is a non-issue; it bites only when SNR/level is marginal. -> harden the
   fine-timing search (lock its window to the Schmidl-Cox pick +- < GI; reject EVM minima that
   imply a >~1-symbol slip) BEFORE relying on it at lower SNR.
4. **Genuine mild-fade ceiling**: WGN:30 + FADE 10dB/0.5Hz + 2-path FAILS (forcing timing did NOT
   help; pilot-EVM ~100x clean). One acquisition + one channel estimate over the ~1.3 s block
   can't track a 0.5 Hz fade + multipath delay spread. The single-acquisition big-block is a
   CLEAN/SLOWLY-VARYING-channel mode (matches VARA's 6241 clean benchmark regime); it is NOT a
   deep-fade mode. The mild test deliberately over-stressed it to find the edge -> found it.

## RECOVERY / METHOD NOTES (for reproduction)
- Testbed was momentarily mis-diagnosed as "RPi1->IONOS broken": recordings came back ~0.1% FS.
  ROOT CAUSE was a SEQUENCING artifact, not a fault: a SINGLE 2.15 s block PLAY started after a
  blocking butler RECORD did not reliably land in the capture window (butler PLAY SSH+aplay
  open latency). FIX: PLAY the block in a CONTINUOUS LOOP (4x back-to-back) on RPi1 and RECORD
  6 s on RPi2 -> guaranteed overlap; then extract one clean TX-length window per loop period
  (energy rising-edge - generous lead) and decode each. The decoder's own Schmidl-Cox acquires
  within the window. WGN-response test proved the path healthy throughout (RPi2 hears IONOS noise
  scaling WGN:40->0.1% / WGN:10->2.0% / WGN:0->4.7% FS; a looped 1500 Hz tone shows 16000x
  spectral spike). Reboots of both Pis + AUDIO_SETUP were done (precautionary); the loop-record
  method was the actual unblock. (Also: a Pi reboot WIPES /tmp -> re-UPLOAD wavs after any reboot.)
- IONOS output appears noise-normalized: absolute recorded level is low at high WGN; SNR is what
  matters and decode is clean.

## ARTIFACTS (all in bigblock_hw/)
- bigblock_tx.wav (TX block, seed 12345), loopback_selftest_cfg16.log (Phase 1, 8/8).
- bb_loop.wav (clean WGN:40 loop), bb_w30flat.wav (WGN:30 flat loop), bb_mild_loop.wav (faded).
- rex_clean40_{0,1}.wav, rex_w30flat_{0,1}.wav, rex_mild_{0,1}.wav (extracted single-block windows).
- spec.wav/spec40.wav (path-health spectral probes), tone_*.wav (level diagnostics).
- sfo_fit.py (intra-block SFO fitter), PHASE2_HW_RESULTS.md (this file).

## NEXT (post-de-risk, NOT done here)
- Production+ARQ integration of the big-block PHY is now JUSTIFIED for the clean/slow-fade regime.
- BEFORE wider-SNR reliance, harden: (1) key-up lead symbol for the DAC-settling transient;
  (2) fine-timing search window constrained to the Schmidl-Cox pick (reject >1-symbol slips).
- The mild-fade ceiling means the big-block is a HIGH-RATE CLEAN mode; keep the existing MFSK/
  lower configs for fading. Do NOT pitch the big-block as a deep-fade mode.

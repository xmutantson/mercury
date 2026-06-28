# Data-flow: OFDM acquisition bounds gate (acq → extract)

Owner: TRACK-B acq-bounds fix (2026-06-27). Branch `staging/acq-bounds`.

## §1 Symptom (batch-2 audit)

On a back-to-back OFDM batch, the Schmidl-Cox coarse cursor occasionally lands
**1–3 symbols past `upper_bound`**, and the bounds-recovery block force-FAILs
(`pream_symb_loc = 0`, telecom_system.cc:1867-1871) **even though the frame body
that produced the detection is still entirely in the ring buffer and would
extract cleanly.** The frame is DISCARDED (FAIL → buffer-shift), not deferred or
recovered. This is distinct from the `frame_overflow_symbols` deferral path
(:1811-1816, which signals "capture more audio") and from the fine-timing /
SKIP-VAR graveyards.

## §2 Coordinates and their units (all SYMBOL units unless noted)

- `pream_symb_loc = floor(receive_stats.delay / sym_samples)` — detected preamble
  symbol, floored. (telecom_system.cc:1787)
- `ofdm_eff = ofdm_search_raw - nUnder_processing_events` — anti-re-decode cursor
  (the symbol from which the search began). SYMBOL units: confirmed by
  `predicted_pos = ofdm_skip * sym_samples` at telecom_system.cc:1484, where
  `ofdm_skip` is the same expression (:1456).
- `lower_bound = preamble_nSymb` (:1820)
- `upper_bound = buffer_Nsymb - (Nsymb + rx_eff_preamble)` (:1832) — the highest
  preamble SYMBOL from which a complete frame still fits, **symbol-floored**.
- EXTRACTION fit predicate (telecom_system.cc:2556-2562), in INTERPOLATED SAMPLES:
  `frame_size_interp = Nofdm*(Nsymb + rx_eff_preamble)*interp`;
  the frame is extractable iff
  `receive_stats.delay <= buf_size_interp - frame_size_interp`.

## §3 Root cause — symbol-floored gate vs sample-exact fit + plateau lead

Two independent contributors:

1. **Symbol-floored vs sample-exact.** `upper_bound` is a symbol-floored bound;
   the extraction's true fit test is sample-exact (`delay <= buf - frame_size`,
   in samples). A detected position whose FLOORED symbol = `upper_bound+1` can
   still satisfy the sample-exact predicate (the buffer holds `buffer_Nsymb`
   whole symbols; the frame needs `Nsymb+rx_eff_preamble`; symbol-floor
   granularity leaves a 1-symbol band that the floored gate rejects but the
   sample test accepts).

2. **Schmidl-Cox Phase-1 plateau lead (~3 symbols).** The halfsym metric is ~1.0
   across the WHOLE preamble plateau — MEASURED ~870 samples / ~2.8 symbols in
   the CFG16 K=8 acquisition (ofdm.cc:3238-3240). The GI-stride coarse search can
   therefore report a position up to ~3 symbols AHEAD of the true preamble onset.
   The true onset is `<= upper_bound` (in bounds) — only the plateau-lead-shifted
   DETECTED position exceeds it. Force-FAILing on the shifted detection discards a
   frame whose true onset is in-bounds and whose body is in-buffer.

The force-FAIL gate `ofdm_eff > upper_bound` (:1867) conflates the legitimate
anti-re-decode floor (don't rescan already-decoded frames) with "no extractable
frame exists." When the cursor is in the plateau-lead band just past
`upper_bound`, a real un-decoded frame still exists and the recovery scan, given
the chance, re-anchors Schmidl-Cox to the in-bounds true onset.

## §4 The audit (CLAUDE.md §5)

State changed: the bounds-recovery GATE (`ofdm_eff > upper_bound` → force-FAIL),
which governs whether the recovery scan runs. Does NOT change `ofdm_search_raw`,
`nUnder_processing_events`, `pream_symb_loc` semantics, or the extraction path.

1. **Producers** of the gate inputs:
   - `ofdm_search_raw`: arq_common.cc (advanced by frame stride after decode;
     reset to 0 on re-anchor/turboshift — many sites, e.g. :2381,:4846,:6073).
   - `nUnder_processing_events`: audioio.c:1425 (++ per buffer underrun); reset 0
     in arq_common.cc at the same re-anchor sites.
   - `upper_bound`: derived from `buffer_Nsymb`, `Nsymb`, `rx_eff_preamble` (:1832).
2. **Consumers** of the gate outcome:
   - force-FAIL path sets `pream_symb_loc = 0` → decode block (:1978) skipped →
     `message_decoded` stays NO → caller (arq_common) shifts buffer.
   - recovery `else` branch: signal-start scan + Schmidl-Cox retry, accepts only
     `retry_symb > lower_bound && retry_symb <= upper_bound` (:1965-1966).
3. **Valid states before any producer writes:** `ofdm_search_raw=0`,
   `nUnder_processing_events=0` (fresh anchor) → `ofdm_eff=0` → never force-FAIL.
   The force-FAIL only fires deep into a batch when the cursor has advanced.
4. **Invariant the consumers assume:** anti-re-decode — the recovery scan must NOT
   re-find already-decoded frames. The scan already enforces this with
   `scan_start = max(lower_bound+1, ofdm_eff)` (:1890-1891) and the accept-gate
   `retry_symb <= upper_bound`. So entering recovery is SAFE: a re-found old frame
   would land at `retry_symb < ofdm_eff` (excluded by scan_start) or fail the
   accept-gate; only a genuinely un-decoded in-bounds onset is accepted.
5. **What the fix changes:** the force-FAIL predicate. Instead of force-FAILing
   whenever the floored cursor `ofdm_eff > upper_bound`, force-FAIL only when even
   the plateau-lead-widened cutoff is exceeded, i.e.
   `ofdm_eff > upper_bound + plateau_lead_symb`. Within the band
   `(upper_bound, upper_bound + plateau_lead_symb]` the recovery scan runs; its
   own anti-re-decode `scan_start` and `retry_symb <= upper_bound` accept-gate
   keep the re-decode invariant intact (verified: the scan starts at `ofdm_eff`,
   never below the already-decoded region, and only accepts an in-bounds onset).
   `plateau_lead_symb` = ceil(measured plateau / sym) = 3 (the ~2.8-sym CFG16
   plateau, ofdm.cc:3239). On all non-amortization / re-anchor paths `ofdm_eff=0`
   so the predicate is unchanged → byte-identical when the feature is dormant.

## §5 Fix-before/after test

`test_acq_bounds_recovery` (datalink_layer/test_acq_bounds.cc): synth-fire a
buffer with a real WB preamble whose floored symbol = `upper_bound + 1` (frame
body fully in-buffer), with `ofdm_search_raw`/`nUnder` set so `ofdm_eff` lands in
the plateau-lead band. Pre-fix: force-FAIL → `pream_symb_loc==0`,
`message_decoded==NO`, frame discarded. Post-fix: recovery scan re-anchors to the
in-bounds onset → frame recovered (`pream_symb_loc` in (lower,upper], delay set).
Plus a no-regress assertion: a cursor genuinely past the band
(`ofdm_eff > upper_bound + 3`) still force-FAILs (anti-re-decode preserved).

## §6 CFO/SFO no-regress

`--test` full suite (BER/loopback/acquisition) must stay all-pass; the fix only
widens the recovery ENTRY cutoff by a fixed 3-symbol band and never changes the
accepted position's bounds, the Moose CFO estimator, or SFO handling.

## §7 MEASURED results (2026-06-28, worktree staging/acq-bounds, o3)

Directed `--test-acq-bounds`, SAME binary, env-toggled gate:
- FAIL-BEFORE (`ACQ_BOUNDS_FAILBEFORE=1`, original `ofdm_eff > upper_bound`):
  CONFIG_0 upper_bound=160. CELL1 onset=160 ofdm_eff=161 -> detected_symb=161
  (SKIP, out of bounds) -> decoded=0 DISCARDED -> suite FAILED (1 cell failure).
- PASS-AFTER (fix active): CELL1 -> recovery re-anchored to detected_symb=159
  (in bounds) -> decoded=1 RECOVERED -> ALL PASS. CELL2 (cursor genuinely past
  band: empty-buf ofdm_eff=168 AND below-floor onset=2) -> no decode FORCE-FAIL_OK
  (anti-re-decode preserved) in BOTH arms.

Isolation note: with `ofdm_defer_overflow_enabled` ON, a tail-band detection
returns early via the overflow-DEFER path (frame_overflow_symbols>0, "capture more
audio") before the bounds gate; the discard the fix targets is the defer-OFF /
cursor-past-upper_bound path (telecom_system.cc:1801). The test drives that path
(ts.ofdm_defer_overflow_enabled=false), which is a supported production mode.

CFO/SFO no-regress: `MERCURY_SFO_BLOCK_TEST=1 -m PLOT_PASSBAND -s 16` (acquisition
under SFO drift) -> frames_decoded=60/60, frames_zero_ber=60/60, block_BER=0,
mean_metric_FULLpre=0.9969. The normal in-bounds path (ofdm_eff=0) is byte-
identical; the SFO harness coarse detection lands in-bounds (pream_symb=6,
bounds=[4,115]) exactly as before.

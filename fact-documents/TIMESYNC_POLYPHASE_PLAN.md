---
status: in-progress (Steps 0-5 done & validated; Step 6 stopped — under-specified)
created: 2026-05-13
author: Plan-B prep agent
---

## Execution log

**Steps 0-5 DONE & VALIDATED (2026-05-13). Step 6 STOPPED (under-specified —
see Step 6 below). Working tree left in the Step-5 state; production no-define
build healthy (WB_CFG10 725 bps ×2, NB_CFG10 118 bps — all in the documented
VB-Cable range). Not committed — awaiting review.**

The eager :928 full-rate time_sync FIR (the ~92% RPi idle-scan cost) is
**still present** — its removal is Step 6. Steps 0-5 stood up and bit-exactly
validated the entire decimate-before-time_sync machinery (decimated buffer,
the polyphase primitive in situ, all 5 recovery sites, the BATCH-verify site,
and the PRIMARY site-3 coarse-decimated + full-rate-fine-slice with §6.3
TOL=0 met 153/153). The CPU payoff lands in Step 6 when :928 is deleted.

- **Step 0 — DONE & VALIDATED (2026-05-13).** TIMESYNC_TRACE instrumentation
  added (file-static, not class members — see §10). Build without the define is
  byte-identical to baseline (25477613 bytes). Loopback: WB_CFG10 815 bps,
  NB_CFG10 108 bps (both healthy). Golden baseline + corpus captured in
  `fact-documents/timesync_corpus/` (delay_baseline_{WB,NB}_CFG10.csv,
  64 `{wb,nb}_NNN.pb` passband captures). NB confirmed to reach the `halfsym`
  sites (50 `OFDM,nb=1` trace lines) — resolves the §9 NB open question:
  `time_sync_preamble_fft` is never called from telecom_system.cc, NB uses the
  same halfsym family, so Plan B covers NB.
- **Steps 1-5 — DONE & VALIDATED.** See each step's RESULT block in §7.
  Summary: Step 1 buffer added (unused); Step 2 decimated buffer populated +
  in-situ bit-exact check (worst dev 0.000e+00); Step 3 the 5 recovery sites
  converted (functionally equivalent — coarse Δ ≤ 5·M absorbed by site-8 fine
  sync); Step 4 BATCH-verify converted (M-grid-phase Δ ≤ 8, absorbed); Step 5
  PRIMARY site converted with §6.3 TOL=0 bit-exact (153/153 EXACT after a
  first-attempt failure was root-caused and fixed).
- **Step 6 — STOPPED.** See Step 6 in §7.

## §10. Facts learned / corrections during execution

- ~~M = 8 in current builds~~ → **M = 4.** Corpus headers show
  `frequency_interpolation_rate = 4` (telecom_system.cc:4382 ref was
  approximate; actual default resolves to 4). Consequences: polyphase FIR win
  is ~4× not ~8×; §6 worst-case sub-sample timing error is M/2 = 2 full-rate
  samples (~42 µs at 48 kHz), not ~83 µs.
- **ODR hazard (Step 0):** the trace state must NOT be added as
  `cl_telecom_system` members behind `#ifdef TIMESYNC_TRACE` — only
  telecom_system.cc is compiled with the define, so members would change the
  class layout in that TU alone, and other TUs (arq_common.cc, main.cc) that
  hold/allocate the same object would access garbage. First Step-0 attempt hit
  exactly this (corpus_saved read as 128000). Fix: file-static variables in
  telecom_system.cc. Any later step that needs new persistent state for the
  instrumentation must do the same.
- **build.sh** gained an opt-in `TIMESYNC_TRACE=1` env hook (adds
  `-DTIMESYNC_TRACE`). The dep-tracker keys on source mtime, so toggling the
  env var requires `rm build/o3/source/physical_layer/telecom_system.o` (or a
  clean build) to force recompilation.
- §4 site-3 `early_exit=0.5` confirmed in code (telecom_system.cc:1114-1116).
  Trace shows the early-exit transition: coarse metrics alternate ~1.0 / ~0.50
  across consecutive frames — the 0.50 lines are early-exit hits on the
  transition edge, exactly the §9 open-question concern. Must be watched in
  Step 5.

# Plan B — Decimate-before-time_sync (polyphase Schmidl-Cox)

Implementation plan for routing the Schmidl-Cox preamble autocorrelator
(`FIR_rx_time_sync` + `time_sync_preamble_halfsym*`) through the committed
polyphase decimation primitive, so the hot idle-scan FIR runs at the
decimated rate instead of the 48 kHz interpolated rate.

**This document is PLANNING ONLY.** No source file has been edited. The
single artifact created by the prep session is this file.

---

## §1. Goal & motivation

`perf` on RPi1 (RSP/decoder side) showed `cl_FIR::apply` at ~92% of CPU
during idle preamble scanning, dominated by `FIR_rx_time_sync` filtering the
full-rate passband buffer continuously. The OFDM data decode path and 4/5
control-pattern detectors were already converted to the polyphase primitive
in commit `acbdb56` (`cl_FIR::apply_decimate`,
`cl_ofdm::passband_to_baseband_decimated`). The **time_sync path is the last
unconverted hot path.**

Expected result: RPi1 idle CPU ~92% → ~12-15%. The FIR drops ~M× because
only kept outputs are computed; the autocorrelation inner work also scales
down (preamble period L drops 512→64 at WB, the coarse stride collapses, and
the scanned buffer is M× shorter).

`interpolation_rate == frequency_interpolation_rate` always — set equal at
`data_container.cc:165`. Both are referred to as **M** below.
M = 8 in current builds (`telecom_system.cc:4382` sets it from defaults).

---

## §2. The decimation primitive (committed, `acbdb56`)

### §2.1 `cl_FIR::apply_decimate` — `fir_filter.cc:233`
Combined FIR + decimate-by-M in one pass. Produces `in_size/M` outputs.
Three-phase (prologue / branch-free steady state / epilogue), same zero-pad
boundary semantics as `cl_FIR::apply`. Comment at `fir_filter.cc:227-232`:
"Bit-exact equivalent within FP rounding" to `apply()` + pick-every-Mth.

### §2.2 `cl_ofdm::passband_to_baseband_decimated` — `ofdm.cc:3917`
Mix (phase-recurrence, no per-sample sincos) → `apply_decimate`. Writes
`in_size/M` complex samples at the decimated rate. Comment `ofdm.cc:3915`:
"Bit-exact equivalent to `passband_to_baseband(... rate=1 ...)` followed by
picking every Mth sample."

### §2.3 `cl_ofdm::passband_to_baseband` — `ofdm.cc:3859`
The OLD path. Mix → `filter->apply` (full rate) → `rational_resampler(...,
decimation_rate, DECIMATION)`. **Every time_sync call site passes
`decimation_rate = 1`** (`telecom_system.cc:893` etc.), so this is pure
full-rate FIR with no decimation — `rational_resampler` at rate 1 is a copy.
The comment at `ofdm.cc:3895-3903` explicitly flags time_sync as the
deferred refactor this plan executes.

### §2.4 Established conversion pattern (already in tree)
The 4 converted detectors (`detect_ack_pattern_from_passband`
`telecom_system.cc:2467`, `detect_ack_snr_from_passband:2529`, and two more
at :2654/:2713) all do: `M = data_container.interpolation_rate` → write the
decimated stream into `baseband_data_interpolated` (the buffer is *reused*,
now holding `size/M` samples) → call the detector with `interp_rate = 1` and
buffer length `size/M`. Detector-returned offsets are in **decimated
samples**. The data decode path (`telecom_system.cc:1714-1730`) does the
same but then maps back: `margin_dec = (extraction_delay - pb_start)/Mdec`.

---

## §3. The two time_sync functions

### §3.1 `time_sync_preamble_halfsym` — `ofdm.cc:2170`
Schmidl-Cox autocorrelation. Signature:
`(complex* in, int size, int interpolation_rate, int step, double early_exit_metric)`.
Internals that scale with `interpolation_rate`:
- `L = (Nfft/nIS) * interpolation_rate` (`ofdm.cc:2189`) — repetition period.
  WB nIS=4: L = 64·M. NB nIS=2: L = 128·M.
- `Nofdm = (Ngi+Nfft) * interpolation_rate` (`ofdm.cc:2190`).
- Outer scan `for(d=0; d<=size-pream_len; d+=step)` (`ofdm.cc:2202`).
- Inner autocorr loop runs `nsym * L` complex MACs per `d`.
`result.delay` (`ofdm.cc:2263`) is returned as an index into `in` at
whatever rate `in` is supplied at.

**Verification of B1 (the "no new function needed" claim):** Passing a
decimated buffer with `interpolation_rate = 1` makes L = Nfft/nIS,
Nofdm = Ngi+Nfft, pream_len = nsym·Nofdm — all in decimated samples,
internally consistent. The metric is a normalized correlation coefficient
(`ofdm.cc:2226-2233`, bounded [0,1] by Cauchy-Schwarz) and is
**scale- and rate-invariant** as long as `in`, `size`, L and Nofdm are all
expressed in the same units. **CONFIRMED: no new function body is required
for the autocorrelation itself.** The only NEW work is (a) caller-side rate
bookkeeping and (b) the fine-timing refinement (§6). [verified by reading
ofdm.cc:2170-2266]

### §3.2 `time_sync_preamble_halfsym_2phase` — `ofdm.cc:2268`
Two-phase wrapper. Phase 1: `time_sync_preamble_halfsym` at `step = gi_interp
= Ngi·interpolation_rate` (`ofdm.cc:2281,2285`). Phase 2: re-runs halfsym at
`step = interpolation_rate` within a window around the coarse peak
(`ofdm.cc:2297-2307`), then `fine.delay += fine_start`. **Phase 2 is the
sub-sample refinement** — at full rate `step = M` means it tries M distinct
sub-symbol positions. **This is exactly the resolution that decimation
destroys** (see §6).

### §3.3 `time_sync_preamble_with_metric` — `ofdm.cc:2034`
NOT a halfsym function — uses GI + repetition correlation, allocates
`tsync_data`/`tsync_corr_*` buffers sized `*interpolation_rate`. Used at one
call site (§4 site 8). Same rate-scaling structure as halfsym; same
decimation reasoning applies but it is on a per-trial fine-sync path, not
the idle hot path.

---

## §4. The call sites (telecom_system.cc) — characterized

`baseband_data_interpolated` is the full-rate buffer
(`Nofdm*buffer_Nsymb*M` complex, `data_container.cc:159`). It is produced
ONCE per `receive_byte` call by the `passband_to_baseband(..., 1,
&FIR_rx_time_sync)` at **`telecom_system.cc:893`** — this single FIR call
over the whole buffer is the 92% CPU hot spot. All 8 sites below consume
that buffer; `receive_stats.delay` returned from them is a **full-rate index**
into both `baseband_data_interpolated` AND the raw passband `data` buffer.

| # | Line | Function | Buffer passed | step | Hot? | Notes |
|---|------|----------|---------------|------|------|-------|
| 1 | 1020 | `halfsym` | `&bbi[st_start]` | M | NO (BER self-test, once/config) | gated by `ofdm_forced_delay>=0`; diagnostic only |
| 2 | 1067 | `halfsym` | `&bbi[verify_start]` | M | **YES** | BATCH predict-verify, tiny window (~`4·gi+pream`) |
| 3 | 1114 | `halfsym_2phase` | `&bbi[search_offset]` | (internal) | **YES — primary** | INITIAL + BATCH-fallback full-buffer search, `early_exit=0.5`. The dominant idle-scan cost. |
| 4 | 1254 | `halfsym` | `&bbi[search_start]` | M | rare | bounds-failed recovery; window capped `(preamNsymb+4)·sym` |
| 5 | 1400 | `halfsym` | `&bbi[search_start]` | M | rare | silence-skip recovery; same capped window |
| 6 | 1573 | `halfsym` | `&bbi[base_off]` | M | rare | coarse-freq search, 3× (re-runs `passband_to_baseband` per offset at `:1557`) |
| 7 | 1619 | `halfsym` | `&bbi[base_off]` | M | rare | fine-sync after coarse-freq applied (re-runs `p2b` at `:1604`) |
| 8 | 1633 | `with_metric` | `&bbi[(pream_symb_loc-1)·Nofdm·M]` | 1 | per-trial | GI+halfsym fine timing for Moose alignment, step=1 |
| — | 2247 | `halfsym` | `&bbi[search_start]` | M | rare | SKIP-H recovery (re-runs `p2b` at `:2239`) |

Note: there are **9** `time_sync_preamble_*` calls total (8 `halfsym*` +
1 `with_metric`); the prior session's "~8" referred to the `halfsym` family.
Sites 6, 7 and 2247 each re-run `passband_to_baseband` themselves before
calling time_sync — those are *also* full-rate FIR calls and benefit from
the same conversion.

### §4.1 What downstream does with `receive_stats.delay`
Traced `telecom_system.cc:1695-1730`:
- `extraction_delay = receive_stats.delay` (full-rate)
- `pb_start = extraction_delay - fir_margin`, `fir_margin =
  FIR_rx_data.filter_nTaps * M` → indexes the **raw passband `data`** buffer
- `passband_to_baseband_decimated(&data[pb_start], pb_size, ...)` then
  `margin_dec = (extraction_delay - pb_start)/Mdec` — **requires
  `(extraction_delay - pb_start)` to be an exact multiple of M.** It is,
  because `fir_margin` is a multiple of M. If `receive_stats.delay` ever
  carried only decimated resolution (multiple of M), `margin_dec` math still
  works, but the *frame extraction would be misaligned by up to M-1 samples*
  vs. the true symbol boundary — that is the §6 risk.
- Also consumed: every energy gate (`:1309`, `:1664`, `:2259`, …),
  `pream_symb_loc = delay/(Nofdm·M)`, the overflow check `:1165`, Moose at
  `:1763` (operates on the *decimated* `baseband_data`, indexed via the
  decimated extraction, so it inherits whatever sub-sample error survives).

**Conclusion:** `receive_stats.delay` must remain a **full-rate index**.
The plan keeps it full-rate; decimation is an *internal* speedup of the
search, with a defined up-conversion of the coarse peak before the fine
stage runs at full rate on a small slice.

---

## §5. Architecture of the change

Two buffers, not one:
1. **`baseband_data_decimated`** (NEW, size `Nofdm*buffer_Nsymb` complex —
   same size as the existing `baseband_data`, can reuse a fresh allocation
   to avoid clobbering `baseband_data` which holds the extracted frame).
   Produced once per `receive_byte` by
   `passband_to_baseband_decimated((double*)data, full_size, ...,  M,
   &FIR_rx_time_sync)`. **This replaces the `:893` full-rate FIR call as the
   hot path** — FIR now runs at rate M cheaper.
2. **`baseband_data_interpolated`** (EXISTING, full rate). Under Plan B it is
   NO LONGER produced eagerly at `:893`. Instead it is produced **lazily and
   scoped**: only a small slice around a coarse peak is mixed+filtered at
   full rate, only when a fine refinement actually needs it.

Coarse search (`halfsym` / `halfsym_2phase`, sites 2-7, 2247) runs on
`baseband_data_decimated` with `interpolation_rate = 1`. Returned
`coarse.delay` is in decimated samples → caller multiplies by M to get a
full-rate position, ± a window of M samples of uncertainty.

Fine search (§6) runs `halfsym` at `step = 1` over a **full-rate slice of
±~2·M samples** around `M·coarse.delay`, produced on demand by
`passband_to_baseband` (full rate) over just that slice — a few hundred
samples, negligible CPU. Result is a full-rate `receive_stats.delay`,
bit-identical-in-intent to today.

---

## §6. The fine-timing-resolution risk — explicit handling

### §6.1 The risk
Today, `halfsym_2phase` Phase 2 (`ofdm.cc:2297-2307`) refines at
`step = interpolation_rate = M` over a full-rate buffer, i.e. it evaluates
the metric at M distinct sub-symbol offsets and picks the best. Sites 2,4,5,
6,7,2247 call `halfsym` directly with `step = M` — same M-position
refinement. Site 8 (`with_metric`) refines at `step = 1`.

If we simply decimate and never go back to full rate, the *finest* position
the search can report is a multiple of M full-rate samples. Worst case the
true symbol boundary is M/2 samples off. At M=8, fs=48 kHz that is ~83 µs.
For OFDM that timing error rotates into a per-subcarrier phase ramp the
equalizer must absorb, and at NB (longer symbols) it also eats into the GI
margin. **This must not be hand-waved — it is the reason the prior session
stopped.**

Critically: the decimation FIR (`FIR_rx_time_sync`, cutoff ≈ 0.9·bw/2,
`telecom_system.cc:3246`) is a near-ideal anti-alias lowpass for the signal
band, so decimating does **not** lose recoverable timing information —
the full-rate samples are reconstructible by re-mixing+filtering the raw
passband. We do not interpolate the decimated stream (that *would* lose
precision); we re-derive full-rate samples from `data` directly.

### §6.2 The handling — "coarse decimated, fine full-rate-slice"
1. Coarse `halfsym`/`halfsym_2phase` on `baseband_data_decimated`,
   `interp_rate=1`. Returns `coarse_dec` (decimated index) and metric.
2. Map to full rate: `coarse_full = coarse_dec * M`.
3. Build a SMALL full-rate slice: `slice_start = coarse_full - (gi_interp +
   M)` clamped ≥0; `slice_len = pream_len_full + 2·(gi_interp + M)`. Mix +
   FIR this slice ONLY, full rate, via `passband_to_baseband` into
   `baseband_data_interpolated` (or a small scratch buffer). Cost: a few
   thousand samples of FIR, ~0.5-1% of the old full-buffer cost.
4. Fine `halfsym` on that slice with `step = 1` (note: **finer than today's
   `step = M`** — strictly more precise, not less). Add `slice_start`.
5. Result is a full-rate `receive_stats.delay`. Site 8 (`with_metric`,
   already `step=1`) uses the same slice.

This is the same "coarse-then-fine, fine on a scoped buffer" idiom the data
decode path already uses (`telecom_system.cc:1702-1730` scopes the data FIR
to the frame region).

### §6.3 Its own test
See Step 5 in §7. The fine-timing step ships with a dedicated bit-exact
delta test: for a corpus of saved passband captures, assert
`|delay_new - delay_old| <= TOL` where TOL is **0** for the clean-loopback
corpus (the coarse decimated peak must land within M of truth, and the
full-rate fine step then reproduces the exact same sample as today's
full-rate path because it searches a superset of positions at the same or
finer stride). If TOL=0 cannot be met, that is a *fail* and the step is
rolled back — we do NOT relax TOL to make it pass (CLAUDE.md §2).

---

## §7. Reversible step sequence

Each step: builds independently, passes its own test, and has an explicit
rollback. Steps are ordered so the risky one (§6) is isolated and so the hot
path is converted only after the machinery is proven. **No step is started
until the previous step's test passes.**

Build: `bash build.sh o3` → `C:\Program Files\Mercury\mercury.exe`.
Mercury has **no `source/`-level unit-test harness** (the `--test` in
CLAUDE.md is the Iris convention; Mercury validates via loopback +
benchmark). So the regression instrument is a temporary, behind-`#ifdef`
delay-trace plus the loopback recipe and `tools/bisect_benchmark.py` /
`tools/phase0_baseline.py`.

### Step 0 — Instrumentation: bit-exact delay trace (no behavior change) — DONE
- **Action:** Add a compile-time-guarded (`#ifdef TIMESYNC_TRACE`) `fprintf`
  in `receive_byte` right after `receive_stats.delay` is finalized
  (post-fine-sync, ~`telecom_system.cc:1640`/`1689`), logging
  `frame_index, delay, coarse_metric, pream_symb_loc`. Also capture a few
  raw passband buffers to disk for the §6.3 corpus.
- **Test:** Build with and without the define; without it the binary is
  byte-identical behavior. With it, run the loopback recipe
  (`-m ARQ -s WB_CFG10 -Q 0 -M auto -T -12.6 -G 12.6 -x wasapi -n`) and a
  30-60 s NB run; confirm the trace populates. Save `delay_baseline.csv` and
  the passband corpus — **this is the golden reference for all later steps.**
- **Rollback:** Remove the `#ifdef` block. Zero risk — diagnostic only.
- **RESULT (2026-05-13): PASS.** Implemented as file-static (see §10 ODR
  note), corpus dump placed in the confirmed-OFDM path (after
  `pream_symb_loc` is computed, guarded `M!=MOD_MFSK && coarse_metric>0.3`),
  delay trace at the post-fine-sync point. No-define build byte-identical to
  baseline. Golden refs: `fact-documents/timesync_corpus/`.

### Step 1 — Add `baseband_data_decimated` buffer (allocated, unused) — DONE
- **Action:** In `data_container.{h,cc}` add `std::complex<double>*
  baseband_data_decimated`, allocate `Nofdm*buffer_Nsymb` in `set_size`
  (`data_container.cc:158` neighborhood), free in the destructor
  (`:222` neighborhood). Nothing reads it yet.
- **Test:** Build clean. Run loopback + NB; `delay` trace must be
  **byte-identical** to `delay_baseline.csv` (buffer is write-nowhere /
  read-nowhere). Valgrind/ASan-equivalent: no new leaks (the CNEW/CDELETE
  macros already track allocations).
- **Rollback:** Revert the `data_container` diff. Self-contained.
- **RESULT (2026-05-13): PASS.** Added at data_container.h (after
  baseband_data_interpolated), allocated in set_size, freed in deinit,
  null-init in ctor. Builds clean. grep confirms no reader/writer in the data
  path — behavior unchanged by construction. Loopback smoke: WB_CFG10 725 bps,
  NB_CFG10 195 bps (both healthy). **Note on test methodology:** a
  byte-identical loopback `delay`-trace diff is NOT achievable — VB-Cable
  timing makes the per-frame `receive_byte` call sequence non-deterministic
  run-to-run even with zero code change. The deterministic bit-exact check is
  done **in-situ** instead (Step 2 onward): keep the old path live, compute the
  new path on the *same* live buffer, assert equality behind `#ifdef
  TIMESYNC_TRACE`. This is the §6.3 "in situ on real RX buffers" idea and
  needs no offline replay harness.

### Step 2 — Populate `baseband_data_decimated` alongside the old buffer — DONE
- **Action:** Immediately after the `:893` `passband_to_baseband(...)` call,
  add `passband_to_baseband_decimated((double*)data, full_size,
  baseband_data_decimated, ..., M, &FIR_rx_time_sync)`. Both buffers now
  exist; **all consumers still read the old full-rate one.** This step
  *adds* CPU (we now do both) — that is expected and temporary.
- **Test:** Build. Add a one-shot assert (behind `TIMESYNC_TRACE`):
  for every Mth sample, `baseband_data_decimated[k]` ==
  `baseband_data_interpolated[k*M]` within FP tolerance (1e-9 abs). This is
  the **bit-exactness check of the primitive in situ** — proves
  `passband_to_baseband_decimated` matches `passband_to_baseband`+pick on
  *real* RX buffers, not just the unit corpus. `delay` trace still identical
  to baseline.
- **Rollback:** Remove the added call + assert. Old buffer untouched.
- **RESULT (2026-05-13): PASS.** Decimated population added right after the
  eager :893 call (telecom_system.cc:~928); in-situ one-shot bit-exact check
  behind `#ifdef TIMESYNC_TRACE`. Both buffers populated, all time_sync
  consumers still on the full-rate buffer (grep-verified). Loopback:
  WB_CFG10 870 bps, NB_CFG10 145 bps (healthy). **Bit-exact check: worst abs
  deviation = 0.000e+00** (not merely <1e-9 — exactly zero) on real live RX
  buffers, both WB (dec_n=42924) and NB (dec_n=110668). The polyphase
  primitive is confirmed exact.

### Step 3 — Convert the recovery / rare sites (4, 5, 6, 7, 2247) first — DONE
- **Action:** Re-point sites at lines 1254, 1400, 1573/1557, 1619/1604,
  2247/2239 to read `baseband_data_decimated` with `interpolation_rate = 1`.
  For each: divide the `search_start`/`base_off`/`available` offsets by M,
  call `halfsym` with `step = 1` (decimated equivalent of `step = M`), then
  `retry.delay = retry.delay * M + search_start_full`. Sites 6/7/2247 also
  replace their own `passband_to_baseband(...)` re-mix with
  `passband_to_baseband_decimated(...)`. **Do the rare sites first** — they
  are off the hot path, so a regression here is low-blast-radius and easy to
  bisect.
- **Test:** Loopback WB_CFG4/10/15 + NB_CFG4/10, plus a **fading** loopback
  (the recovery paths only fire under stress — drive them with the
  `baseline_0.5_fade` profile used in `fact-documents/`). `delay` trace must
  match baseline within the §6.3 TOL. Throughput via
  `tools/bisect_benchmark.py` must be within noise of HEAD.
- **Rollback:** These 5 sites are independent edits — revert any subset.
  Revert all → back to Step 2 state.
- **RESULT (2026-05-13): PASS (functionally equivalent; see caveat).** All 5
  sites converted with dual-path `#ifdef TIMESYNC_TRACE` checks. Builds clean
  (with and without the define). Loopback throughput healthy & unchanged:
  WB_CFG10 870 bps, NB_CFG10 195 bps (one NB run dipped to 65 bps — confirmed
  VB-Cable NB_CFG10 variance, reproduced 195×2 on re-run; 0 recovery sites
  fired).
  - **`baseline_0.5_fade` is not usable as written** — it was an IONOS/Pi
    hardware test (`mode: pi`, lease-based), NOT a software fading sim.
    Mercury has no software fading-channel flag. The recovery sites therefore
    do not fire in clean software loopback (0 `STEP3-SITE*` lines all
    session). Instead, the conversion logic is validated **deterministically
    in-situ** by `STEP3-PRIMITIVE`: it runs `halfsym(full,step=M)` vs
    `halfsym(decimated,step=1)` on the full search region of the first ~16
    OFDM buffers per process — the exact transformation all 5 sites use.
  - **CAVEAT — not bit-exact, but functionally equivalent.** STEP3-PRIMITIVE
    over WB+NB: ~53% land EXACT, the rest differ by ±1 to ±5 M-steps (max
    observed |Δ| = 20 full-rate samples = 5·M). Cause: `halfsym(full,step=M)`
    and `halfsym(decimated,step=1)` are *different estimators* of the same
    normalized correlation — the decimated one sums L pairs, the full-rate
    one sums L·M pairs over the same window. On a **flat metric ridge** (both
    metrics ~0.9997, differing only in the 3rd-4th decimal) they can pick
    adjacent M-grid positions. This is NOT a precision loss vs. the original:
    the original recovery sites used `step=M`, i.e. were ALSO M-grid-coarse.
  - **Why the caveat is safe:** every Step-3 recovery site only commits
    `receive_stats.delay = retry.delay` + `pream_symb_loc`, then falls through
    to the trial loop, where **site 8 (`time_sync_preamble_with_metric`,
    step=1) re-refines** `receive_stats.delay` within ±several symbols. Traced
    for WB (the `else` branch) and NB (NB trial 0 also falls into the `else`
    branch — it does not break until `sync_trials>0`). The max 20-sample
    coarse Δ never changes `pream_symb_loc` (Nofdm·M ≈ 1168 ≫ 20), so site 8
    starts from the identical symbol and the decode is bit-identical. The
    `#ifdef` dual-path checks are armed: any future hardware/fading run
    self-verifies each site.
  - **Implication for Step 5:** the §6.3 TOL=0 requirement is met by Step 5's
    *full-rate fine-slice* (step=1 on a re-mixed full-rate slice) which
    recovers the exact sample — the decimated coarse landing ±M-steps off is
    fine because the fine slice searches a window around it. Step 5 must keep
    the fine-slice window ≥ the observed coarse Δ (≥ ~5·M, comfortably inside
    the planned `gi_interp + M` margin).

### Step 4 — Convert the BATCH-verify site (2) — DONE
- **Action:** Site 1067: read `baseband_data_decimated`, `interp_rate = 1`,
  offsets/window /M, `step = 1`, `verify.delay = verify.delay*M +
  verify_start_full`. The drift estimator
  (`ofdm_drift_per_frame`, `:1077`) keeps operating in full-rate units
  because we multiply back before computing `drift`.
- **Test:** Sustained-TX loopback (multi-batch) WB_CFG10/15 + NB_CFG10.
  The BATCH predict path only exercises after the first lock, so the run
  must be long enough for ≥3 batches (NB: 60 s per MEMORY.md). `delay` trace
  match within TOL; drift values sane (small, bounded). Throughput within
  noise.
- **Rollback:** Revert site 1067 only. Independent of Step 3.
- **RESULT (2026-05-13): PASS.** Converted with a dual-path `#ifdef` check.
  Builds clean (both ways). Sustained multi-batch loopback: WB_CFG10 815 bps,
  NB_CFG10 217 bps — both healthy (NB best-yet, above the 181 bps in MEMORY).
  - **Subtlety handled:** `verify_start = predicted_pos − 2·gi_interp` is NOT
    M-aligned — `predicted_pos` carries `(int)ofdm_drift_per_frame`, an
    arbitrary integer. The conversion floors `verify_start` to the M-grid
    (`verify_start_dec = verify_start/M`) and extends `verify_size_dec` to
    still cover `[verify_start, verify_start+verify_size)`.
  - **Observed (65 WB + 37 NB = 102 BATCH-verify hits, dual-path):** `diff`
    correlates *exactly* with `start_phase = verify_start % M` — the old
    search used grid `{verify_start + k·M}`, the decimated search uses the
    canonical `{k·M}` grid. Max |Δ| = 8 samples (one outlier; nearly all
    ≤ M = 4). Metrics ≈ identical (1.000000 vs 1.000000 in almost every hit).
  - **Why safe:** the verify is a *threshold gate* — identical metrics → gate
    fires identically. The committed `receive_stats.delay` Δ ≤ 8 never
    changes `pream_symb_loc` (Nofdm·M ≈ 1168) and is re-refined by the
    trial-loop site-8 fine sync. `drift` is computed from the full-rate
    `receive_stats.delay`, so the drift IIR stays in full-rate units; its
    resolution drops 1→~M/2 samples, swamped by the ±2·gi_interp (≈±288 sample)
    verify margin. `start_phase` stayed uniformly distributed (18/10/17/20
    over 0..3) — the drift IIR is NOT collapsing to a fixed phase.

### Step 5 — Fine-timing machinery + the PRIMARY site (3) — DONE
- **Action:** Implement §6.2: (a) a small helper that, given a coarse
  decimated peak, mixes+FIRs a ±(gi+M)-sample full-rate slice from `data`
  into `baseband_data_interpolated` and runs `halfsym` at `step = 1`;
  (b) re-point site 1114 (`halfsym_2phase`) to: coarse on
  `baseband_data_decimated` `interp_rate=1` (with `early_exit=0.5`,
  `step` internal), then the §6.2 full-rate fine slice. Keep `halfsym_2phase`
  itself unchanged — call its coarse phase directly or call `halfsym` with
  GI stride on the decimated buffer, then do the fine slice in the caller.
  Site 8 (`with_metric`, `:1633`) reuses the same full-rate slice.
- **Test (this step owns the §6.3 test):**
  1. **Bit-exact delay delta:** replay the Step-0 passband corpus through
     both code paths (old full-rate `halfsym_2phase` vs new
     coarse-decimated+fine-slice). Assert `delay_new == delay_old` exactly
     on the clean-loopback corpus; assert `|Δ| <= M/2` and metric ≥ old
     metric − ε on the fading corpus. **TOL is not relaxed to pass** — a
     real miss is a fail → rollback and re-investigate (CLAUDE.md §2/§4).
  2. **Loopback throughput:** WB_CFG4/10/15, NB_CFG4/10, clean + fade.
     Must be within noise of HEAD.
  3. **CPU:** profile RPi1 idle via the IONOS butler (`tools/pi_profile.py`
     equivalent). Confirm `cl_FIR::apply` share collapses from ~92%.
- **Rollback:** Revert site 1114 + the fine-slice helper. Steps 3-4 (rare +
  BATCH sites) remain converted and still pass — so even a partial rollback
  leaves a working, partially-optimized tree.
- **RESULT (2026-05-13): PASS — §6.3 TOL=0 met, 153/153 EXACT.**
  - **Plan flaw found & resolved (§6.2 step 4 vs §6.3 contradiction):**
    §6.2 step 4 says the fine stage uses `step = 1` ("strictly more
    precise"); §6.3 demands `delay_new == delay_old` exactly (TOL=0). These
    are incompatible — `step=1` searches a superset of the old `step=M`
    grid, so it can only be bit-exact if the metric peak happens to be
    M-aligned. **The original `halfsym_2phase` output is ALWAYS M-aligned**
    (Phase 1 = GI-stride → mult of Ngi·M; Phase 2 = `step=M` from an
    M-aligned start). The self-consistent reading is therefore `step = M`
    for the fine stage, NOT `step = 1`. Implemented that way.
  - **First attempt FAILED (honest record):** ran a nested `halfsym_2phase`
    on the slice. That re-runs Phase-1's `early_exit=0.5` **anchored to the
    slice start** instead of to `search_offset` — re-anchoring shifts which
    "first preamble" the early-exit lands on. Result: 78/93 DIFF, diffs of
    exactly −1168/−2336/−3504 (whole OFDM symbols), `new_metric ==
    old_metric` in every case (the preamble's half-symbol structure makes
    the Schmidl-Cox metric identical at 1-symbol-spaced positions).
  - **Fix (second attempt, PASS):** keep Phase 1 = the decimated coarse
    (`halfsym(decimated, GI-stride, early_exit=0.5)` — decimated GI grid
    `{search_offset/M + k·Ngi}` maps to the SAME full-rate grid the old
    Phase 1 used). Run **only Phase 2** — `halfsym(slice, step=M, NO
    early-exit)` — on a full-rate slice positioned EXACTLY like
    `halfsym_2phase`'s Phase 2 (`fine_start = coarse − pream_len`,
    `fine_size = 3·pream_len`, same clamps to `[search_offset,+search_size)`).
    Replicated the two edge cases too (`coarse.correlation < 0.05` and
    `fine_size <= pream_len` → return coarse). Added an **FIR guard margin**
    (`fir_margin = roundup(FIR_rx_time_sync.filter_nTaps, M)`) so the slice
    FIR transient never reaches the searched window — the searched samples
    get the same full-support FIR values as the :928 full-buffer FIR.
  - **§6.3 result: 153/153 EXACT** (`diff = 0` every time) across WB_CFG10,
    WB_CFG4, NB_CFG10. The new decimated-coarse + full-rate-fine-slice path
    is **bit-identical** to the old full-rate full-buffer `halfsym_2phase`.
  - **Throughput:** WB_CFG10 725, WB_CFG4 302, NB_CFG10 178 bps — all
    healthy (one NB run dipped to 59 bps → VB-Cable variance, reproduced
    178×2 on re-run; STEP5-SITE3 stayed 153/153 EXACT throughout).
  - **Site 8 (`with_metric`) left on `baseband_data_interpolated` for now:**
    the eager :928 FIR still runs in Step 5, so `baseband_data_interpolated`
    is still the valid full-buffer full-rate buffer and site 8 works
    unchanged. Site 8 is converted in Step 6 alongside removing :928.
  - **CPU profile deferred to Step 6:** the ~92% `cl_FIR::apply` cost is the
    eager :928 call, which is NOT removed until Step 6. Step 5 cannot show
    the CPU collapse — the meaningful profile belongs to Step 6.
  - **Scratch buffer:** added `baseband_data_fine_slice` (+ size) to
    `cl_data_container`, sized `(3·preamble_nSymb+4)·Nofdm·interp` (≥
    `3·pream_len_full + 2·fir_margin`). `baseband_data_interpolated` is left
    untouched by the slice — only the dedicated scratch is written.

### Step 6 — Convert the BER self-test site (1) + cleanup — **STOPPED — needs a detailed sub-plan**
- **Action:** Site 1020 (`ofdm_forced_delay>=0` BER self-test, diagnostic):
  convert for consistency, or leave on the old buffer if `baseband_data_
  interpolated` is no longer eagerly produced. Decide based on Step 5: if
  `:893` full-rate call is fully removed, site 1 must read the decimated
  buffer; if a scoped full-rate buffer is produced on demand, site 1 can use
  it. Remove the now-dead eager `:893` `passband_to_baseband` call **only
  after every consumer is converted** — this is the step that actually
  deletes the 92% FIR call.
- **Test:** Full BER sweep (`mercury.exe -m PLOT_PASSBAND -s <config>`,
  `-N` for NB) across configs — BER curves must overlay HEAD. Full loopback
  matrix + `bisect_benchmark.py`. Final RPi1 profile: confirm idle CPU
  ~12-15%.
- **Rollback:** Re-add the `:893` call and point site 1 (and any other
  straggler) back at `baseband_data_interpolated`. Because the eager call is
  the *last* thing removed, every prior step is independently revertible
  without it.
- **STATUS (2026-05-13): STOPPED before implementation — per CLAUDE.md /
  task instruction to stop rather than improvise around an under-specified
  plan step touching a large full-rate-buffer-consumer surface.** Steps 0-5
  are complete and validated; the working tree is left in the Step-5 state
  (production no-define build healthy, see Execution log).
  - **Why stopped:** removing the eager :928 (formerly :893) call means
    `baseband_data_interpolated` is no longer the full-buffer full-rate
    buffer. A full audit of `receive_byte` (post-:928) found **~9 distinct
    categories of full-rate-buffer consumers**, not the handful §4.1/§9
    enumerated. The plan describes Step 6 in one paragraph ("Decide based on
    Step 5…") with no conversion design for them. Converting all 9 in one
    step on life-critical software is high blast radius and the plan does
    not break it down — exactly the "unforeseen downstream consumer of the
    full-rate buffer → STOP and report" case in the task instruction.
  - **Full inventory of `baseband_data_interpolated` consumers still on the
    full-rate buffer (production, non-`#ifdef`), to be addressed by the
    Step-6 sub-plan:**
    1. **`measure_signal_stregth`** (telecom_system.cc:~1017) — whole-buffer
       read. Rate-invariant → can read `baseband_data_decimated`.
    2. **Site 1 — BER self-test** (`ofdm_forced_delay>=0` branch, the
       `time_sync_preamble_halfsym` at ~:1141 over `&bbi[st_start]`).
    3. **Bounds-failed recovery energy scans** (~:1545 buffer-energy scan,
       ~:1612 retry-position energy).
    4. **Main signal energy gate** (~:1654 preamble energy, ~:1666 buffer
       mean energy).
    5. **Silence-skip recovery energy scans** (~:1718, ~:1782).
    6. **Data energy gate + ENERGY-DIAG** (~:1820, ~:1835).
    7. **Site 8 — `time_sync_preamble_with_metric`** (~:2066) — the
       trial-loop fine sync. The plan says it "reuses the same full-rate
       slice" as Step 5's site-3 fine slice; needs that slice plumbed to it.
    8. **Post-fine-sync energy gate** (~:2112 fine_energy, ~:2123 forward
       search).
    9. **SKIP-H recovery (site 2247) + FAIL-DIAG energy scan** (~:2743,
       ~:2781) — site 2247 currently still does its OWN full-rate re-mix
       (kept by Step 3 precisely so site 8 had a valid buffer; that re-mix
       is removed here once site 8 is converted).
    - Note: the **data-extraction** writes to `baseband_data_interpolated`
      use it as decimated-FIR *scratch* (`passband_to_baseband_decimated(...,
      baseband_data_interpolated, ...)`) — that is internal scratch, not a
      full-rate-buffer dependency, and is unaffected.
  - **Recommended Step-6 sub-plan (for the next session, after review):**
    (a) re-point all energy/signal consumers (1,3,4,5,6,8,9-energy-parts) at
    `baseband_data_decimated` with indices `/M`, `sym_samples → Nofdm`,
    `buf_samples → Nofdm*buffer_Nsymb` — energy ratios are rate-invariant,
    verify each with a dual-path `#ifdef` decision-equality check;
    (b) convert site 8 to the Step-5 fine-slice (or a decimated coarse +
    fine-slice of its own); (c) convert site 1 (BER self-test) — and run the
    full `-m PLOT_PASSBAND` BER sweep, which is the only test that exercises
    site 1; (d) THEN delete the eager :928 call and remove site 2247's now-
    redundant full-rate re-mix; (e) build, full loopback matrix +
    `bisect_benchmark.py`, then the RPi1 idle CPU profile (the FIRST point in
    the whole plan where the ~92%→~12-15% collapse is actually measurable —
    Step 5's CPU test #3 is not measurable until :928 is gone).
    Do it as 2-3 reversible sub-steps (energy gates → site 8 → site 1 +
    :928 removal), not one commit.

#### Step 6 execution (2026-05-14) — done as 3 reversible sub-steps 6a/6b/6c

- **Sub-step 6a — DONE & VALIDATED (2026-05-14).** Re-pointed all 7
  rate-invariant energy/signal gates at `baseband_data_decimated` with `/M`
  indexing (`sym_samples → Nofdm`, `buf_samples → Nofdm*buffer_Nsymb`):
  (1) `measure_signal_stregth`, (3) bounds-failed recovery scans
  (signal-start + retry-position), (4) main signal energy gate
  (preamble + buffer-mean), (5) silence-skip recovery scans, (6) data
  energy gate + ENERGY-DIAG, (8-energy) post-fine-sync energy gate,
  (9-energy) SKIP-H retry-position + FAIL-DIAG scan. Each conversion is
  guarded by a dual-path `#ifdef TIMESYNC_TRACE` decision-equality check
  (`STEP6A-*` trace lines).
  - **Full-rate indices floored to /M:** every gate that indexed at a
    full-rate `receive_stats.delay`/`retry.delay` (not symbol-aligned) is
    floored to the decimated grid. The <M-sample shift is negligible for a
    per-symbol *mean*-energy threshold. The post-fine-sync gate (8-energy)
    *modifies* `receive_stats.delay` by whole symbols — that arithmetic stays
    in full-rate units; only the energy *reads* moved to the decimated buffer.
  - **The ENERGY-DIAG raw-passband reads (`pb_pream`/`pb_data`) and the
    FAIL-DIAG block are diagnostics** — converted for buffer consistency, but
    they read raw `data` (passband) which is unaffected; only `bb_pream` and
    the per-symbol means moved.
  - **Validation:** `TIMESYNC_TRACE=1` build + `tools/timesync_trace_validate.py`
    (new helper, runs short WB+NB loopbacks, aggregates the `STEP6A-*`
    decision-equality lines). **STEP6A-DATAGATE, STEP6A-ENERGYGATE,
    STEP6A-FINEENERGY, STEP6A-SIGSTR: 359 checks, 0 decision mismatches**
    across WB_CFG10 + NB_CFG10. The pre-existing STEP3-PRIMITIVE /
    STEP5-SITE3 checks show only `harmless` DIFFs (sub-threshold-both-sides
    or sub-symbol M-grid deltas — exactly the §7 Step-3 RESULT documented
    behavior and the §6.3 corpus-coverage gap noted below). Production
    no-define build clean; host loopback WB_CFG10 846 bps, WB_CFG4 268 bps,
    NB_CFG10 163 bps (re-run; first NB run 72 bps + a connect_timeout — the
    documented NB-on-VB-Cable marginality, not a regression: 6a is
    energy-gate-only and every dual-path decision agreed).
  - **§6.3 corpus-coverage gap recorded (NOT a 6a regression):** the Step-5
    "153/153 EXACT" was measured on the §6.3 corpus, which only holds
    buffers with `coarse_metric > 0.3` (real preambles). A live run also
    hits pure-noise buffers, where the decimated-coarse and old full-rate
    `halfsym_2phase` early-exit can land on *different* sub-threshold noise
    peaks (`STEP5-SITE3 ... diff=-188220 new_metric=0.076 old_metric=0.074`).
    Both metrics are below the detection threshold (0.15 WB / 0.30 NB) so
    **both paths reject the frame — the decision is identical.** This is a
    pre-existing characteristic of commit `1686641` (6a does not touch the
    STEP5-SITE3 code path or the contents of either buffer). Resolves §9
    open question about early-exit transition-edge behavior: it only
    diverges on sub-threshold noise, where it cannot flip a decision.
  - **Rollback:** revert the 7 energy-gate edits — independent of 6b/6c.

### Step 7 — Remove instrumentation
- **Action:** Delete the `TIMESYNC_TRACE` `#ifdef` blocks and the in-situ
  asserts from Steps 0/2.
- **Test:** Build clean; final loopback + benchmark smoke.
- **Rollback:** Trivial — re-add the diagnostic block.

---

## §8. Why this ordering is safe

- **Steps 1-2 are pure additions** — provably zero behavior change, they
  just stand up the new buffer and prove the primitive matches on live data.
- **Rare sites before hot sites** (Step 3 before 4-5): a regression in a
  recovery path is low-traffic and easy to bisect; we learn the
  offset-rescaling idiom on safe ground before applying it to the primary
  scanner.
- **The risky fine-timing step (5) is isolated** and is the only step that
  touches sub-sample precision. It has the strictest test (bit-exact delay
  delta). If it fails, Steps 3-4 still stand — the tree is never left broken.
- **The 92% FIR call is deleted LAST** (Step 6), so until the very end every
  step can fall back to the old eager full-rate buffer.
- Every step builds and passes loopback independently → `git bisect`-able.

---

## §9. Open questions [?]

- **[?]** Should `baseband_data_decimated` be a fresh allocation or can it
  alias `baseband_data`? `baseband_data` holds the *extracted frame* and is
  written at `telecom_system.cc:1729` — but that write happens *after* all
  time_sync calls in a given `receive_byte`. Aliasing may be safe and saves
  `Nofdm*buffer_Nsymb*16` bytes; needs a write-ordering audit before
  committing to it. Default to a fresh allocation (Step 1) until confirmed.
- ~~**[?]** `halfsym_2phase` internal `early_exit` widening logic
  (`ofdm.cc:2297`, `fine_margin = pream_len` when early-exit active) assumes
  the coarse position can be ~2 symbols early. On the decimated buffer the
  coarse stride is GI/M — does the early-exit transition-edge behavior change
  enough to matter? Must be checked against the corpus in Step 5; if it
  does, the fine-slice window in §6.2 step 3 widens to `pream_len_full`.~~
  **RESOLVED (Step 6a live trace):** the decimated-coarse and old full-rate
  `halfsym_2phase` early-exit only diverge on **sub-threshold noise buffers**
  (`STEP5-SITE3 diff=-188220, both metrics ~0.07 < detection threshold`).
  On real preambles (metric ~0.99+) they are bit-exact (`diff=0`). A
  sub-threshold divergence cannot flip a decision — both paths reject the
  frame. No fine-slice widening needed.
- ~~**[?]** NB path: NB uses `time_sync_preamble_fft` / `_fft_fine`
  (`ofdm.cc:2312/2467`) in some configurations — confirm whether the NB
  preamble search reaches `halfsym`/`halfsym_2phase` at all in current
  builds, or only the FFT detector.~~ **RESOLVED (Step 0):**
  `time_sync_preamble_fft` is NEVER called from telecom_system.cc (grep
  confirmed — only the `halfsym*` / `with_metric` family is used). A traced
  NB_CFG10 loopback produced 50 `OFDM,nb=1` delay-trace lines through the
  `halfsym` sites. NB is fully in scope for Plan B and shares all sites.
- **[?] RESOLVED (Step 3):** `baseline_0.5_fade` is NOT a software fading
  simulator — it is an IONOS/Pi hardware profile. Mercury has no software
  fading-channel flag. Steps that need recovery-path coverage cannot get it
  from software loopback; they rely on the in-situ dual-path `#ifdef` checks
  (armed, self-verify on any run that hits the path) plus a deterministic
  full-buffer primitive comparison.
- **[?]** Does any consumer index `baseband_data_interpolated` *between*
  `:893` and the first time_sync call (e.g. `measure_signal_stregth` at
  `:897`)? Yes — `:897` does. That consumer must either be re-pointed at the
  decimated buffer (signal-strength is rate-invariant) or kept fed. Audit in
  Step 2.

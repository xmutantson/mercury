---
title: Plan C — MFSK time_sync polyphase conversion
status: PLANNING + RESEARCH ONLY — no source edited; this file is the only artifact
created: 2026-05-14
author: Plan-C research agent
depends-on: TIMESYNC_POLYPHASE_PLAN.md (Plan B, Steps 0-7 complete & committed
            1686641..2022256 on branch monitor)
---

# Plan C — MFSK time_sync polyphase conversion (the "10th consumer")

Plan B converted all 9 OFDM `FIR_rx_time_sync` consumers + the idle-scan
`measure_signal_only` to the polyphase decimated primitives. During Step 6c a
**10th consumer** was found and deliberately left untouched: the MFSK preamble
detectors `time_sync_mfsk_corr` / `time_sync_mfsk`, which still consume the
full-rate `baseband_data_interpolated` buffer produced by the eager
`passband_to_baseband(... rate=1 ...)` call kept alive *only* for
`M == MOD_MFSK` (`telecom_system.cc:918-921`). Plan C is the research +
reversible-action plan to convert that last consumer.

**This document is PLANNING ONLY. No source file has been edited.**

---

## §1. Goal & scope

Convert the MFSK preamble detectors so the full-rate
`passband_to_baseband(... rate=1 ...)` FIR at `telecom_system.cc:920` is no
longer needed — i.e. remove the *last* `cl_FIR::apply` (full-rate FIR) call
from the production RX path, making the conversion uniform across every
Mercury mode.

The MFSK detectors are used by the ROBUST family:
- `ROBUST_0/1/2` are `_modulation = MOD_MFSK` (`telecom_system.cc:4513-4530`).
- NB ROBUST: `M = 8` (ROBUST_0) or `M = 4` (ROBUST_1/2) (`telecom_system.cc:4597-4598`).
- The detector *selection* is by `ofdm.mfsk_corr_template != NULL`
  (`telecom_system.cc:993`): when a correlation template exists →
  `time_sync_mfsk_corr` (waveform cross-correlation); else → `time_sync_mfsk`
  (FFT energy ratio). Per the code comment at `telecom_system.cc:995` and
  `ofdm.cc:3069-3079`, `time_sync_mfsk_corr` is the NB+WB cross-correlation
  path; `time_sync_mfsk` is the WB FFT-energy fallback.

**Out of scope:** the ARQ idle-loop cadence (`usleep(2000)` at
`arq_common.cc:2250`), already noted out-of-scope by Plan B §7's Step-6
Pi-profile correction. Plan C does NOT change idle cadence.

---

## §2. Characterization of the MFSK detectors

### §2.1 `time_sync_mfsk_corr` — `ofdm.cc:3080`
Waveform cross-correlation of a pre-generated baseband preamble template
(`mfsk_corr_template`, `mfsk_corr_template_nsymb` symbols) against the received
baseband buffer. Signature (`ofdm.cc:3080-3082`):
`(complex* baseband_interp, int buffer_size_interp, int interpolation_rate,
  int search_start_symb, double* out_metric)`.

Internal structure (two phases):

**Phase 1 — coarse, 4× sub-symbol OVERSAMPLED scan** (`ofdm.cc:3095-3194`):
- `sym_period_interp = Nofdm * interpolation_rate` (`ofdm.cc:3088`).
- `P1_OVERSAMPLE = 4`; `p1_step = sym_period_interp / P1_OVERSAMPLE`
  (`ofdm.cc:3100-3101`) — the scan stride is **1/4 of a symbol**, not a full
  symbol.
- Outer scan `for(base_interp = p1_start_interp; ...; base_interp += p1_step)`
  (`ofdm.cc:3111`).
- Per candidate: per-symbol normalized correlation, summed/averaged over
  `template_nsymb` symbols (`ofdm.cc:3122-3161`). A `per_sym_floor = 0.05`
  rejects positions where any symbol fails (`ofdm.cc:3149-3153`, Bug #34).
- Keeps the **top-K = 8** candidates in a sorted array (`ofdm.cc:3104-3181`).
- Early-exit on the first candidate with `metric > 0.5` (`ofdm.cc:3192-3193`)
  — prefers the *earliest* strong preamble.

**Phase 2 — fine refinement** (`ofdm.cc:3205-3270`):
- For each of the (up to 8) Phase-1 candidates, search
  `for(d = coarse - search_half; d <= coarse + search_half; d += interpolation_rate)`
  with `search_half = sym_period_interp` (`ofdm.cc:3209,3218`).
- **The step is `interpolation_rate`** (= M), i.e. it walks the M decimation
  phases within ±1 symbol of the coarse candidate.
- Early-exit when a candidate's refined metric exceeds 0.5 (`ofdm.cc:3268`).
- Threshold `0.5` on the **Phase-2 refined** metric (`ofdm.cc:3276-3285`,
  Bug #34/#40/#44). Returns `best_fine` — an index into `baseband_interp` at
  the rate `baseband_interp` was supplied at — or `-1`.

**Critical fact — the correlator already decimates-on-read.** Both Phase 1 and
Phase 2 read the buffer as `baseband_interp[rx_offset + n * interpolation_rate]`
(`ofdm.cc:3132`, `ofdm.cc:3240`). The inner correlation sums only `Nofdm`
samples — it reads **every M-th sample** of the full-rate buffer. The full-rate
buffer is used purely as a *source of M distinct decimation phases*: the
`p1_step = sym_period_interp/4` stride and the Phase-2 `+= interpolation_rate`
stepping are both just **selecting which decimation phase** the correlation
runs on. [verified by reading ofdm.cc:3080-3286]

### §2.2 `time_sync_mfsk` — `ofdm.cc:2925`
FFT-energy-ratio detector (WB fallback when no correlation template). For each
symbol position `s` (stepped by **one full symbol**, `ofdm.cc:2964`), it
decimates a symbol out of the buffer (`decimated_sym[i] =
baseband_interp[offset + i*interpolation_rate]`, `ofdm.cc:2978`), FFTs it, and
scores `e_target/e_total` summed over `preamble_nSymb` symbols. `offset =
sym_idx * sym_period_interp + Ngi * interpolation_rate` (`ofdm.cc:2971`).
Threshold `preamble_nSymb * {0.3 NB | 0.5 WB}` (`ofdm.cc:3016`). Returns
`best_sym_idx * sym_period_interp` (`ofdm.cc:3064`) or `-1`.

**`time_sync_mfsk` has NO sub-symbol oversampling** — it scans on a 1-symbol
grid and reads a *single* decimation phase (phase 0, the `Ngi*interpolation_rate`
offset is symbol-internal, fixed). It is therefore far simpler to convert than
`time_sync_mfsk_corr`. [verified by reading ofdm.cc:2925-3067]

### §2.3 What downstream does with the returned `delay`
`receive_stats.delay` from the MFSK detector is consumed (`telecom_system.cc`):
- The trial loop for MFSK is **single-trial, no refinement**
  (`telecom_system.cc:1776-1782`): "MFSK: time_sync_mfsk already found optimal
  position, no refinement needed ... Trial 0: use delay from initial sync
  as-is." There is **no later fine-sync site** that re-refines an MFSK delay
  (unlike OFDM, where Plan B's Step-3 coarse Δ was absorbed by site-8
  `with_metric`). **The MFSK detector's output is final.**
- Frame extraction (`telecom_system.cc:2036,2065`): `extraction_delay =
  receive_stats.delay`; `pb_start = extraction_delay - fir_margin` with
  `fir_margin = FIR_rx_data.filter_nTaps * frequency_interpolation_rate`;
  `passband_to_baseband_decimated(&data[pb_start], ...)` then
  `margin_dec = (extraction_delay - pb_start)/Mdec`. `(extraction_delay -
  pb_start)` equals `fir_margin`, a multiple of M, so the index math is exact
  **regardless** of `receive_stats.delay`'s own resolution — but the *frame
  window* is positioned at `receive_stats.delay`, so a timing error there
  directly misaligns the FFT window.
- Energy gates (`telecom_system.cc:1709,1738`, etc.) index at
  `receive_stats.delay / de_M` — rate-invariant, unaffected by Plan C.

**Conclusion:** `receive_stats.delay` for MFSK must retain timing precision
*at least as good as today* — specifically the Bug #44 guarantee in §3. It
must remain expressible as a full-rate index (the frame-extraction and
energy-gate arithmetic above all assume that), but its *resolution* need only
match what Phase 2 produces today (an `interpolation_rate`-grid value relative
to a `p1_step`-grid coarse candidate — see §3).

---

## §3. The Bug #44 constraint — stated precisely

**Provenance.** The `(Bug #44)` annotations at `ofdm.cc:3099` and `ofdm.cc:3275`
were introduced by commit `79ebbbb` (2026-02-23, "WB preamble detection, NB
OFDM fixes…"). `git blame` confirms the `P1_OVERSAMPLE` block is wholly from
`79ebbbb`. Note: §19 of `fact-documents/NARROWBAND_FEATURE.md` describes a
*different* "Bug #44" ("NB MFSK Turnaround Estimate Too Small" — an ARQ
buffer-sizing bug). The bug number was reused; the **code comment's Bug #44 is
the sub-symbol-oversampling fix**, and that is the constraint Plan C must
preserve. The code comment is precise and self-contained — Plan C cites it
directly. [verified: git blame -L 3095,3105 ofdm.cc → 79ebbbb;
git log -S P1_OVERSAMPLE → 79ebbbb only]

**What `79ebbbb` changed.** Before `79ebbbb`, `time_sync_mfsk_corr` Phase 1
scanned on a **full-symbol grid** (`base_interp = s * sym_period_interp`) and
returned a single `best_sym_idx`; Phase 2 then refined ±1 symbol around that
*one* coarse point. `79ebbbb`:
1. changed Phase 1 to a **1/4-symbol grid** (`p1_step = sym_period_interp/4`),
2. made Phase 1 collect the **top-K = 8** candidates instead of one,
3. ran Phase 2 over **all** top-K candidates.
[verified: git show 79ebbbb -- ofdm.cc]

**Why the 4× oversampling is necessary (the constraint).** From the in-code
rationale (`ofdm.cc:3095-3099`): with `M = 4` NB, a Phase-1 candidate that is
half a symbol misaligned scores `metric ~= 0.25` — *comparable to data-content
false peaks* (`~0.22`). On a pure full-symbol grid the worst-case misalignment
is exactly ½ symbol, so the **true preamble can score at or below the
data-content floor**. Consequences that break detection:
- Phase 1's `metric > 0.5` early-exit never fires on the true preamble (its
  coarse score is ~0.25), so Phase 1 keeps scanning and may early-exit on, or
  rank above the true preamble, a *data-content* peak.
- If the true preamble is not in the top-K, Phase 2 never gets a seed within
  ±1 symbol of it → Phase 2 cannot recover it.
At **4× oversampling the worst-case misalignment is 12.5 % of a symbol**,
keeping the true preamble's Phase-1 metric `> 0.76` — comfortably above the
data-content floor, so it always early-exits / always ranks into the top-K,
and Phase 2 always has a good seed. (This is also why Phase 2's threshold-0.5
gate at `ofdm.cc:3276` works — it relies on Phase 1 having delivered a seed
within ±1 symbol of truth.)

**The constraint, stated for Plan C:** any converted `time_sync_mfsk_corr`
MUST evaluate the preamble metric on a grid no coarser than **¼ symbol**
(`P1_OVERSAMPLE = 4`) during the coarse stage, AND must refine on the
**`interpolation_rate` (= M)** grid during the fine stage. Equivalently: the
converted path must be able to evaluate the correlation at **all M decimation
phases** at sub-symbol positions — the exact thing a single-phase
decimate-by-M throws away. A naive `passband_to_baseband_decimated` (which
produces decimation phase 0 only) does **not** satisfy this; that is precisely
why Plan B Step 6c stopped here. `time_sync_mfsk` (§2.2) has no such
constraint — it has no oversampling and reads one phase.

---

## §4. Is this worth doing? — honest priority assessment

**Verdict: LOW priority. Defer unless an MFSK/ROBUST CPU profile shows a real
problem. Plan C is a "make it uniform / close the inventory gap" task, not a
measured-win task.**

Reasoning, with citations:

1. **The MFSK full-rate FIR is NOT on the true-idle hot path.** Plan B's
   Step-6 Pi profile (`TIMESYNC_POLYPHASE_PLAN.md` §7, "Step 6 RPi1 idle-CPU
   profile") found the dominant idle FIR cost was `measure_signal_only`
   (`telecom_system.cc:2714`), called from `arq_common.cc:2240` only when
   `link_status == IDLE || DROPPED`. `measure_signal_only` does **not** call
   any MFSK detector — it is `passband_to_baseband_decimated` +
   `measure_signal_stregth` only, and it is **mode-agnostic and already
   decimated** (converted by Plan B Step 6c-ext, commit `23c4ab7`). So during
   *true idle* there is no MFSK full-rate FIR at all, for any config.
   [verified by reading telecom_system.cc:2714-2752, arq_common.cc:2219-2247]

2. **The MFSK full-rate FIR at `:920` fires only inside `receive_byte`**,
   which `arq_common.cc:2222-2223` documents runs only while
   `link_status == LISTENING` — i.e. only when an MFSK/ROBUST link is *up and
   actively waiting for a frame*. It is a per-RX-buffer cost during an active
   ROBUST connection, not a free-running idle scan.

3. **ROBUST is the low-SNR fallback, used at low throughput.** ROBUST_0/1/2
   waterfalls are −13/−11/−8 dB (`telecom_system.cc:5123-5125`); these are the
   slow weak-signal modes. The symbol period is long (NB ROBUST frames are
   ~5× WB per MEMORY.md), so `receive_byte` is called *infrequently* relative
   to the OFDM data path. The absolute number of full-rate FIR passes per
   second in a ROBUST link is far below the OFDM idle-scan rate Plan B
   targeted.

4. **No profile data exists showing MFSK RX CPU is a problem.** Plan B's
   motivation (§1) was a *measured* RPi1 perf profile (`cl_FIR::apply` at
   ~92 %). There is no equivalent profile for an active ROBUST link.
   CLAUDE.md §2 ("fix root causes, not symptoms") and the "don't gold-plate a
   non-win" instruction both argue against converting on spec.

5. **Counter-point (why it is not zero priority):** (a) it closes the Plan B
   §7 inventory — leaving one un-converted full-rate FIR caller is a latent
   trap for the next person who greps for `cl_FIR::apply`; (b) on a Pi, a
   ROBUST link *is* CPU-constrained and the conversion is a clean ~M× win on
   that path with no SNR risk if done correctly; (c) `time_sync_mfsk` (the WB
   FFT detector, §2.2) is *trivial* to convert (no Bug #44 constraint) and
   could be done cheaply on its own.

**Recommended posture:** Do **Step 1** (convert `time_sync_mfsk`, the easy
half) opportunistically — it is low-risk and removes a real full-rate FIR
caller. **Gate Steps 2-4** (the `time_sync_mfsk_corr` polyphase conversion)
behind an actual MFSK/ROBUST-link Pi CPU profile (§7 test plan includes
capturing it). If that profile shows MFSK RX FIR is negligible, **stop after
Step 1** and record the decision — do not convert `time_sync_mfsk_corr`
purely for inventory tidiness.

---

## §5. Researched approach options (with citations)

The problem: run a preamble correlator that needs sub-sample (M-phase,
¼-symbol) timing resolution, sourcing from a stream we want to FIR at the
*decimated* rate for the ~M× CPU win.

### Option A — coarse-on-decimated + fine on a full-rate slice (Plan B Step 5 idiom)
Mirror Plan B's proven PRIMARY-site pattern (`TIMESYNC_POLYPHASE_PLAN.md` §6.2,
Step-5 RESULT "153/153 EXACT"): run the coarse stage on a single-phase
decimated buffer, then mix+FIR a *small full-rate slice* around the coarse peak
from raw `data` and run the fine stage on it.

- **Pro:** already proven and committed in this codebase for the OFDM
  Schmidl-Cox path; bit-exact there.
- **Con — does NOT fit `time_sync_mfsk_corr`.** Plan B's OFDM coarse stage is
  rate-*invariant* (a normalized autocorrelation, §3.1 of Plan B): a
  single-phase decimated buffer is a valid coarse search. The MFSK Bug #44
  constraint (§3) says the **coarse** stage itself needs ¼-symbol / all-M-phase
  resolution — a single-phase decimated coarse search is exactly the failure
  mode `79ebbbb` fixed. So Option A would have to do the *coarse* stage at full
  rate (the expensive part), defeating the purpose. **Rejected for
  `time_sync_mfsk_corr`.** (Option A *is* essentially Step 1 for the simpler
  `time_sync_mfsk`, which has no coarse-resolution constraint.)

### Option B — polyphase decimator producing all M phases (RECOMMENDED)
Decompose the anti-alias FIR into its **M polyphase sub-filters** and produce
**M decimated output streams** — one per decimation phase — in a single pass
over the passband buffer. The MFSK correlator then runs entirely at the
decimated rate, indexing phase `p` stream for sub-symbol offset `p`.

This is the **textbook polyphase decomposition** for the case where the
decimated *phase* carries information you must keep. Standard references:
- **Crochiere & Rabiner, _Multirate Digital Signal Processing_ (Prentice-Hall,
  1983)**, Ch. 3 — the canonical derivation: an FIR + decimate-by-M is exactly
  M polyphase sub-filters `h_p[n] = h[nM + p]`, and producing all M phases
  costs the *same* total MACs as one full-rate FIR pass but is M parallel
  decimated convolutions.
- **liquid-dsp** `firpfb` (polyphase filterbank) / `firdecim` — Joseph Gaeddert's
  liquid-dsp implements exactly this: `firpfb_crcf` evaluates an FIR at a
  selectable sub-sample phase; `symsync` (symbol timing recovery) is built on
  `firpfb` precisely to get sub-sample timing on a decimated stream.
  (github.com/jgaeddert/liquid-dsp, `src/filter/src/firpfb.proto.c`,
  `src/filter/src/symsync.proto.c`.)
- **GNU Radio** `gr::filter::pfb_arb_resampler` and
  `digital::pfb_clock_sync_ccf` — GNU Radio's clock-recovery block is a
  polyphase filterbank: it holds `n_filters` phase branches and selects the
  branch giving the best timing, the direct analog of Mercury's "M decimation
  phases." (gnuradio/gr-digital, `lib/pfb_clock_sync_ccf_impl.cc`.)

**Why it fits Mercury cleanly:** `cl_FIR::apply_decimate` (`fir_filter.cc:233`)
*already* computes one phase — output `m` uses input window centered at
`m*M` (`fir_filter.cc:250,268,282`). Adding a `phase` parameter so the window
centers at `m*M + phase` turns it into a polyphase sub-filter selector with a
**one-line change** to each of the three loops. Calling it M times (phases
`0..M-1`), or once into M output buffers, yields the M decimation phases. Total
work ≈ one full-rate FIR pass — but it replaces the full-rate `cl_FIR::apply`
(which itself was the cost) and, more importantly, the correlator's own
`Nofdm`-sample inner loop is **unchanged** (it already reads decimated; we just
give it the right phase stream). The Bug #44 ¼-symbol grid maps exactly: with
M = 4, the 4 decimation phases ARE the 4 `P1_OVERSAMPLE` sub-symbol offsets
(`p1_step = sym_period_interp/4` and a phase-`p` stream are the same set of
samples). For M = 8 (NB ROBUST_0) the M = 8 phases are *finer* than
`P1_OVERSAMPLE = 4` — strictly ≥ the Bug #44 guarantee.

- **Pro:** preserves Bug #44 exactly (in fact ≥ it for M = 8); the inner
  correlation loop is untouched → low SNR risk; same total FIR MACs as today
  but via the cheaper branch-free `apply_decimate` kernels; reuses an existing,
  bit-exact-tested primitive with a minimal extension.
- **Con:** needs M decimated buffers (M × `Nofdm*buffer_Nsymb` complex) instead
  of one — but each is M× smaller than the full-rate buffer, so total memory is
  the *same* as today's single `baseband_data_interpolated`. Needs the
  `apply_decimate` phase-offset extension (small, but it is a shared primitive
  → must be bit-exact-verified, see §7 Step 2).

### Option C — upsample around the coarse peak
Run a single-phase decimated coarse search, then *upsample* (interpolate) the
decimated stream around the coarse peak to recover sub-sample resolution.

- **Pro:** small code surface.
- **Con — violates Plan B §6.1's own warning** and CLAUDE.md §1. Plan B §6.1
  explicitly says: "We do not interpolate the decimated stream (that *would*
  lose precision); we re-derive full-rate samples from `data` directly."
  Interpolating a decimated stream cannot recover what the decimation removed;
  it only invents a band-limited guess. And it still leaves the *coarse* stage
  single-phase — the Bug #44 failure mode. **Rejected.**

### Recommendation
**Option B (polyphase decimator producing all M phases) for
`time_sync_mfsk_corr`.** It is the only option that preserves the Bug #44
coarse-stage constraint, it reuses a primitive already in the tree, and the
correlator's numerically sensitive inner loop is left bit-identical.
**Option A (Plan B Step-1/§6.2 idiom, here just "convert to a single-phase
decimated buffer") for `time_sync_mfsk`**, which has no coarse-resolution
constraint.

**Single strongest citation:** Crochiere & Rabiner, _Multirate Digital Signal
Processing_ (1983), Ch. 3 — the polyphase decomposition `h_p[n] = h[nM+p]` that
makes "FIR + decimate, but keep all M phases" cost the same as one full-rate
FIR. liquid-dsp's `symsync`/`firpfb` and GNU Radio's `pfb_clock_sync_ccf` are
the working-code embodiments of that result for symbol-timing recovery.

---

## §6. Architecture of the change

### §6.1 New primitive — `cl_FIR::apply_decimate` phase-offset extension
Add an optional `int phase = 0` parameter to `cl_FIR::apply_decimate`
(`fir_filter.cc:233`). The only change: in all three loops, the input window
center becomes `m*M + phase` instead of `m*M` (and the steady-state
`m_start_steady` / `m_end_steady` bounds shift by `phase`). With `phase = 0`
the function is **byte-identical** to today (the existing callers are
unaffected). `phase ∈ [0, M)` selects polyphase sub-filter `p`.

Optionally also add `passband_to_baseband_decimated_allphases(...)` in
`ofdm.cc` that mixes once and calls the phased `apply_decimate` M times into M
output buffers — but calling the phased primitive M times from the caller is
equally fine and keeps the change smaller. Recommend: just the phase parameter,
call M times.

### §6.2 New buffer(s)
`baseband_data_mfsk_phases` — M contiguous decimated streams, each
`Nofdm*buffer_Nsymb` complex (total = `M*Nofdm*buffer_Nsymb` =
`Nofdm*buffer_Nsymb*frequency_interpolation_rate` = exactly the size of the
existing `baseband_data_interpolated`). Allocate in `data_container.cc`
`set_size` (`data_container.cc:~158` neighborhood, beside
`baseband_data_decimated` which Plan B added), free in deinit. Layout:
`baseband_data_mfsk_phases[p * (Nofdm*buffer_Nsymb) + k]` = decimation phase
`p`, decimated sample `k`.

### §6.3 Converted `time_sync_mfsk_corr`
The function body's correlation math is **unchanged**. What changes is purely
*which buffer/stride it reads*:
- Today: one full-rate buffer, `interpolation_rate = M`, reads
  `buf[off + n*M]` → effectively phase `(off mod M)`, decimated sample
  `(off div M) + n`.
- Plan C: M decimated buffers, `interpolation_rate = 1`. A search position
  `base_interp` (full-rate units) maps to phase `p = base_interp mod M` and
  decimated base `b = base_interp / M`; the inner loop reads
  `phases[p][b + k*Nofdm + n]`. Phase 1's `p1_step = sym_period_interp/4`
  walk and Phase 2's `+= interpolation_rate` walk both become walks over
  `(phase, decimated-index)` pairs — the *same set of sample positions*,
  re-indexed.
- The returned `delay` is reconstructed to a full-rate index exactly as today:
  `delay_full = b*M + p` — bit-identical to the value the current code
  returns, because it is the same sample position expressed the same way.

Net effect: the correlation reads the *identical samples* it reads today
(within FIR FP rounding — `apply_decimate` vs `apply`+pick is documented
bit-exact, `fir_filter.cc:227-232`), so the metric, the top-K ranking, the
early-exits, and the Bug #44 ¼-symbol behavior are all preserved by
construction.

### §6.4 Converted `time_sync_mfsk` (the WB FFT detector)
No phase concern — it reads phase 0 only and scans on a 1-symbol grid. It can
read `baseband_data_decimated` (Plan B's existing single-phase decimated
buffer, populated by `passband_to_baseband_decimated`) with
`interpolation_rate = 1` directly. This is Option A applied to a detector that
has no coarse-resolution constraint — a straight re-point, the same transform
as Plan B Step 6a's energy gates.

### §6.5 Removing the eager `:920` full-rate FIR
Once both detectors are converted, the `if(M == MOD_MFSK)
passband_to_baseband(... rate=1 ...)` block at `telecom_system.cc:918-921` is
dead and is removed. The `M != MOD_MFSK` guard on the decimated-buffer
population at `telecom_system.cc:930` is widened to "always" (the decimated
buffer — and, for `time_sync_mfsk_corr`, the M-phase buffer — is now populated
for MFSK too). After this step, `cl_FIR::apply` (full-rate FIR) has **zero**
production callers anywhere in the RX path.

---

## §7. Reversible step sequence

Each step builds independently, has its own test, and an explicit rollback.
**No step starts until the previous step's test passes.** No step is started
on `time_sync_mfsk_corr` (Steps 2-4) until §4's gating profile (captured in
Step 0) justifies it.

Build: `bash build.sh o3` → `C:\Program Files\Mercury\mercury.exe`.
Mercury has **no source-level unit-test harness** — validation is via loopback
and BER, exactly as Plan B documented (`TIMESYNC_POLYPHASE_PLAN.md` §7 preamble).
Relevant recipes:
- NB ROBUST loopback: `-m ARQ -s ROBUST_0 -Q 0 -M nb -T -12.6 -G 12.6 -x wasapi -n`
  (also `ROBUST_1`, `ROBUST_2`). Per MEMORY.md, NB needs ≥60 s measure windows.
- WB ROBUST loopback: same with `-M auto` / WB selection.
- BER: `mercury.exe -m PLOT_PASSBAND -s ROBUST_0 -N` (and `ROBUST_1/2`); the
  BER path uses `mfsk_fixed_delay` (`telecom_system.cc:379-391`) so it bypasses
  the detector — useful as a *control* (must be unchanged) but it does **not**
  exercise the detector. Detector coverage comes from the ARQ loopback.

Because Plan B's `TIMESYNC_TRACE` instrumentation was removed in Step 7
(commit `2022256`), Plan C re-introduces a **minimal, `#ifdef`-guarded
dual-path check** for its risky step (Step 3), following Plan B's proven
in-situ methodology (`TIMESYNC_POLYPHASE_PLAN.md` Step-1 RESULT "Note on test
methodology" — a byte-identical loopback delay trace is not achievable because
VB-Cable timing is non-deterministic; the deterministic check is dual-path
in-situ). Use a fresh define name, e.g. `MFSK_TIMESYNC_TRACE`, and follow the
§10 ODR rule from Plan B (**file-static variables in the .cc, never
`#ifdef`-guarded class members**). Add the matching opt-in env hook to
`build.sh` as Plan B did.

### Step 0 — Capture the gating profile (no code change) — DECISION GATE
- **Action:** Deploy current `monitor` HEAD (`2022256`) to RPi1/RPi2 via
  `tools/mercury_deploy_rpi.py`. Run an active **NB ROBUST_0 link** (commander
  + responder, sustained data) and capture an RPi1 call-graph perf profile of
  the *responder* `receive_byte` path, the same methodology as Plan B's Step-6
  Pi profile. Record the `cl_FIR::apply` (full-rate FIR) share *inside an
  active ROBUST link*.
- **Test:** none — this is a measurement.
- **Decision:** if `cl_FIR::apply` is a meaningful share (say > ~10 %) of the
  active-ROBUST RX profile → proceed to Steps 2-4. If negligible → do **Step 1
  only**, then stop and record "Steps 2-4 deferred: no measured win" in this
  document's execution log.
- **Rollback:** n/a.

### Step 1 — Convert `time_sync_mfsk` (WB FFT detector) to the decimated buffer
- **Action:** Re-point the `time_sync_mfsk` call at `telecom_system.cc:1005`
  to pass `data_container.baseband_data_decimated` with `interpolation_rate =
  1` and buffer length `Nofdm*buffer_Nsymb`. Inside `time_sync_mfsk`
  (`ofdm.cc:2925`), with `interpolation_rate = 1` every `*interpolation_rate`
  collapses — the FFT reads contiguous decimated samples, the symbol-grid scan
  is unchanged, the returned `best_sym_idx * sym_period_interp` is now a
  decimated index → caller multiplies by M to restore a full-rate
  `receive_stats.delay`. Ensure `baseband_data_decimated` is populated for
  MFSK (today `telecom_system.cc:930` skips MFSK — widen that guard, or
  populate it specifically when `mfsk_corr_template == NULL`).
- **Test:** WB ROBUST loopback (`ROBUST_0/1/2`) — must connect, gearshift, and
  pass data within noise of HEAD. (`time_sync_mfsk` only runs when
  `mfsk_corr_template == NULL`, i.e. the WB FFT-energy path — confirm the WB
  ROBUST configs actually take this branch on this build; if WB ROBUST also
  builds a corr template, `time_sync_mfsk` is dead code and this step is a
  no-op cleanup, which the test will show as "branch never taken".)
- **Rollback:** revert the `telecom_system.cc:1005` call site + the
  `time_sync_mfsk` `interpolation_rate` handling. Self-contained; independent
  of Steps 2-4.

> **RESULT — Step 1 DONE (commit `5334b3b`, branch `monitor`, 2026-05-14)**
>
> **Change made** (one file, `source/physical_layer/telecom_system.cc`):
> - Call site (was `:1005`, now `~:1010`): `time_sync_mfsk` now reads
>   `data_container.baseband_data_decimated` with `interpolation_rate = 1` and
>   buffer length `Nofdm*buffer_Nsymb`. Returned value is a decimated index;
>   the call site multiplies by `data_container.interpolation_rate` to restore
>   a full-rate `receive_stats.delay` (preserving `-1` for "no preamble").
>   `time_sync_mfsk`'s function body and signature were NOT changed — with
>   `interpolation_rate = 1` every `*interpolation_rate` collapses naturally.
> - Buffer-population guard (`:930`): widened from `if(M != MOD_MFSK)` to
>   `if(M != MOD_MFSK || ofdm.mfsk_corr_template == NULL)` so the decimated
>   buffer is populated on the MFSK FFT-detector path. `time_sync_mfsk_corr`
>   (the Bug #44 detector) is untouched and still reads the full-rate buffer.
> - `search_start` (in symbols, rate-invariant — confirmed: `mfsk_search_raw`
>   is set as `frame_end_symb - frames_to_read`, `arq_common.cc:4640`) passes
>   straight through unchanged.
>
> **Open question §9 [?] RESOLVED — `time_sync_mfsk` is dead code at runtime.**
> `mfsk_corr_template` is built *unconditionally* for every MFSK config
> (`telecom_system.cc:4859` — a plain `if(M == MOD_MFSK)`, no NB/WB split),
> so at runtime `ofdm.mfsk_corr_template != NULL` is always true and
> `telecom_system.cc:993` always takes the `time_sync_mfsk_corr` branch.
> Step 1 is therefore a **no-op cleanup at runtime** (the converted branch is
> never executed), exactly as this plan anticipated ("branch never taken").
> It still correctly removes `time_sync_mfsk` as a *reader* of the full-rate
> `baseband_data_interpolated` buffer. Consequence for §4: the cost/benefit
> of removing the eager `:920` full-rate FIR shifts **entirely** onto Step 3
> (`time_sync_mfsk_corr`) — Step 1 alone cannot let the eager FIR be removed.
>
> **Test result (PASS):** `bash build.sh o3` clean. WB ROBUST_0 ARQ loopback
> (`tools/robust_loopback_test.py 100 100 --gearshift`): connected via HAIL +
> MFSK preamble detection (T+40.9s), turboshifted 100→0→7→15 FORWARD+REVERSE,
> delivered **54 RX-DATA frames**; 14 `cfg=100` MFSK decodes (via
> `time_sync_mfsk_corr`). WB ROBUST_1 loopback: connected, HAIL + 3 `cfg=101`
> MFSK ACKs, no crash (turboshift still in progress at window end — known slow,
> not a regression). **`MFSK-SYNC` verbose lines = 0** in both runs →
> confirms the converted `time_sync_mfsk` branch is never taken at runtime,
> as predicted. No regression vs HEAD.
>
> **Steps 2-4 status: NOT STARTED** — gated behind §4 / Step 0 (active-ROBUST
> Pi CPU profile). Per the task scope, only Step 1 was executed.

### Step 2 — Add the polyphase phase-offset to `cl_FIR::apply_decimate`
- **Action:** Add `int phase = 0` to `cl_FIR::apply_decimate`
  (`fir_filter.cc:233` + the declaration in `fir_filter.h`). Change the window
  center in all three loops to `m*M + phase`; shift `m_start_steady` /
  `m_end_steady` so the steady-state range still has a fully-in-bounds window.
  Add `baseband_data_mfsk_phases` (M-stream buffer, §6.2) to `data_container`
  — allocated, **not yet read by anyone**.
- **Test:** This is a pure-addition step (existing callers pass no `phase` →
  `phase = 0` → byte-identical). Build clean. Run the **full Plan B regression**
  — WB_CFG10 / NB_CFG10 loopback + a BER point — and confirm throughput/BER
  unchanged (proves the `phase = 0` default did not perturb the shared
  primitive). Add a one-shot `#ifdef MFSK_TIMESYNC_TRACE` in-situ assert: for a
  live passband buffer, `apply_decimate(in, out, n, M, phase=p)[k]` ==
  `apply(in, tmp, n)[k*M + p]` for all `p ∈ [0,M)` and a sample of `k` — the
  bit-exactness check of the polyphase primitive on real data, mirroring Plan B
  Step 2's "worst abs deviation = 0.000e+00" check.
- **Rollback:** revert the `fir_filter.{h,cc}` diff + the `data_container`
  buffer. Self-contained.

### Step 3 — Convert `time_sync_mfsk_corr` to the M-phase decimated buffers (RISKY — owns the Bug #44 test)
- **Action:** (a) After the existing decimated-buffer population in
  `receive_byte`, when `mfsk_corr_template != NULL`, also populate
  `baseband_data_mfsk_phases`: mix once + call the phased `apply_decimate` for
  `p = 0..M-1` into the M sub-streams (or add and use
  `passband_to_baseband_decimated_allphases`). (b) Re-point the
  `time_sync_mfsk_corr` call at `telecom_system.cc:996` to the M-phase buffer
  with `interpolation_rate = 1`. (c) Inside `time_sync_mfsk_corr`, replace the
  single-buffer `baseband_interp[off + n*interpolation_rate]` reads with the
  `phases[p][b + k*Nofdm + n]` indexing of §6.3; the Phase-1 `p1_step` walk and
  Phase-2 `+= interpolation_rate` walk become `(phase, decimated-index)` walks
  over the identical sample set; reconstruct the returned `delay` as
  `b*M + p`. **Keep the correlation math, the per-symbol floor, the top-K
  logic, the early-exits, and both thresholds byte-for-byte unchanged.**
- **Test (this step owns the Bug #44 test):**
  1. **Dual-path bit-exact delay check** (`#ifdef MFSK_TIMESYNC_TRACE`): keep
     the old full-rate `passband_to_baseband` + old-path `time_sync_mfsk_corr`
     live alongside the new path; assert `delay_new == delay_old` **exactly**
     and `metric_new == metric_old` exactly on every buffer with a real
     preamble candidate. The new path reads the same samples re-indexed, so
     **TOL = 0 is required** — a real miss is a fail → rollback and
     re-investigate (CLAUDE.md §2; mirrors Plan B §6.3 / Step 5's TOL=0
     contract). On pure-noise buffers a sub-threshold divergence is acceptable
     iff both paths return `-1` (decision-equal) — same carve-out Plan B
     Step 6a documented.
  2. **NB ROBUST loopback, all three modes** (`ROBUST_0` M=8, `ROBUST_1`/
     `ROBUST_2` M=4 — M=4 is the Bug #44 worst case). Each must connect,
     complete turboshift, and pass data within noise of HEAD over a ≥60 s
     window. ROBUST_2 (LDPC rate 1/4, the mode Bug #33b showed is least
     timing-tolerant) is the canary — if timing regresses, ROBUST_2 fails
     first.
  3. **NB BER control:** `-m PLOT_PASSBAND -s ROBUST_0/1/2 -N` — must overlay
     HEAD (this path uses `mfsk_fixed_delay`, so it is a *control*: it proves
     Plan C did not perturb anything *outside* the detector).
- **Rollback:** revert the `time_sync_mfsk_corr` body + the
  `telecom_system.cc:996` call site + the M-phase population. The
  phased-`apply_decimate` primitive (Step 2) stays — it is inert without a
  caller. Step 1 stays. Tree returns to "Step 1 + Step 2" state, still
  building and passing.

### Step 4 — Remove the eager `:920` full-rate FIR + cleanup
- **Action:** Delete the `if(M == MOD_MFSK) passband_to_baseband(...)` block
  at `telecom_system.cc:918-921`. Widen the decimated-buffer population guard
  at `telecom_system.cc:930` so MFSK populates `baseband_data_decimated`
  (needed by Step 1's `time_sync_mfsk` and the MFSK energy gates) and, when
  `mfsk_corr_template != NULL`, `baseband_data_mfsk_phases` (Step 3). Confirm
  by grep that no production (non-`#ifdef`) reader of
  `baseband_data_interpolated` remains on the MFSK path. Then remove the
  `MFSK_TIMESYNC_TRACE` instrumentation (a final Step-7-style cleanup) — or
  leave it one commit for a follow-up, matching Plan B's structure.
- **Test:** Full ROBUST loopback matrix (NB `ROBUST_0/1/2` + WB `ROBUST_0/1/2`)
  + full BER sweep for the ROBUST configs — all within noise / overlaying
  HEAD. Re-run the Step-0 active-ROBUST RPi1 profile and confirm
  `cl_FIR::apply` (full-rate FIR) is **gone** from the active-ROBUST RX
  profile, replaced by `apply_decimate`. Record before/after CPU like Plan B's
  Step-6 profile table.
- **Rollback:** re-add the `:920` block and re-narrow the `:930` guard; because
  the eager call is removed **last**, every prior step remains independently
  valid after a Step-4 rollback.

---

## §8. Why this ordering is safe

- **Step 1 is independent and low-risk** — `time_sync_mfsk` has no Bug #44
  constraint; converting it first banks a real (if small) win with no
  sub-sample-precision risk and exercises the "MFSK populates the decimated
  buffer" plumbing on safe ground.
- **Step 2 is a pure addition** — the `phase` parameter defaults to the
  existing behavior; the M-phase buffer has no reader. Provably zero behavior
  change, and it bit-exactly proves the polyphase primitive on live data
  before anything depends on it.
- **Step 3 is the only risky step** (it owns sub-sample precision and the
  Bug #44 guarantee) and is fully isolated: it has the strictest test (TOL=0
  dual-path) and a clean rollback to a still-working "Step 1 + 2" tree.
- **The eager full-rate FIR is deleted LAST** (Step 4), so until the very end
  every step can fall back to the old full-rate buffer — exactly Plan B's
  ordering discipline.
- Every step builds and passes loopback independently → `git bisect`-able.

---

## §9. Open questions [?]

- **[?]** Does the WB ROBUST path actually build an `mfsk_corr_template`? If
  yes, `time_sync_mfsk` (`ofdm.cc:2925`) is dead code in current builds and
  Step 1 is a no-op cleanup (still worth doing for clarity, but the §4
  cost/benefit shifts entirely to Step 3). The Step-1 test ("branch never
  taken") resolves this. Resolve before finalizing Step 1's expected outcome.
- **[?]** For NB ROBUST_0, `M = 8` → the 8 decimation phases are *finer* than
  `P1_OVERSAMPLE = 4`. Step 3 must decide: keep the Phase-1 walk on the
  ¼-symbol grid (sample only 4 of the 8 phases at the coarse stage, matching
  today exactly — recommended, bit-exact) **or** opportunistically use all 8
  (finer than today — strictly more precise, but then it is *not* bit-exact vs
  HEAD and the TOL=0 test must be reasoned about like Plan B Step 5's
  `step=M`-vs-`step=1` analysis). Recommend: **match today exactly** (sample
  the same ¼-symbol grid) so the TOL=0 dual-path test is meaningful; treat
  "use all M phases" as a separate, later precision-improvement task with its
  own SNR validation.
- **[?]** Memory: `baseband_data_mfsk_phases` is the same total size as
  `baseband_data_interpolated`. Once Step 4 removes the eager `:920` call, is
  `baseband_data_interpolated` still needed for the MFSK path at all? It is
  still used as decimated-FIR *scratch* by the MFSK frame-extraction
  (`telecom_system.cc:2065,2146` write into it) — so it cannot be freed, but
  confirm no *MFSK preamble-detection* reader remains (grep audit in Step 4).
- **[?]** `time_sync_mfsk_corr`'s `search_start_symb` parameter
  (`telecom_system.cc:1000`, anti-re-decode skip) — confirm the
  symbol→decimated-index mapping of `search_start` is exact under the M-phase
  re-indexing (it should be: `search_start` is in symbols, and a symbol is
  `Nofdm` decimated samples in every phase stream). Verify in Step 3.

---

## §10. Summary for the main thread

- **Bug #44 constraint:** `time_sync_mfsk_corr` must evaluate the preamble
  correlation on a grid no coarser than **¼ symbol** at the coarse stage and on
  the **M-decimation-phase** grid at the fine stage. With M=4 NB ROBUST_1/2, a
  full-symbol-grid coarse search lets the true preamble score ~0.25 —
  indistinguishable from data-content false peaks (~0.22) — so detection
  fails. A single-phase `passband_to_baseband_decimated` throws away exactly
  the M phases the detector needs; that is why Plan B Step 6c stopped here.
- **Recommended approach:** **Option B — extend `cl_FIR::apply_decimate` with a
  polyphase `phase` offset and produce all M decimated phase-streams**, then
  run the (unchanged) correlator at the decimated rate indexing the right
  phase. Strongest citation: **Crochiere & Rabiner, _Multirate DSP_ (1983),
  Ch. 3** (polyphase decomposition `h_p[n]=h[nM+p]`); embodied in liquid-dsp
  `symsync`/`firpfb` and GNU Radio `pfb_clock_sync_ccf`.
- **Worth-it / priority verdict:** **LOW priority.** The MFSK full-rate FIR is
  NOT on the true-idle hot path (`measure_signal_only` is mode-agnostic and
  already decimated); it fires only inside `receive_byte` during an *active*
  ROBUST link, and ROBUST is the slow low-SNR fallback. No profile shows it is
  a problem. Recommendation: do **Step 1** (convert the trivial `time_sync_mfsk`
  WB FFT detector) opportunistically; **gate Steps 2-4** behind an actual
  active-ROBUST-link Pi CPU profile (Step 0). Do not convert
  `time_sync_mfsk_corr` purely for inventory tidiness.
- **Ordered reversible steps:** **Step 0** capture active-ROBUST Pi profile
  (decision gate) → **Step 1** convert `time_sync_mfsk` to the single-phase
  decimated buffer → **Step 2** add polyphase `phase` offset to
  `apply_decimate` + allocate the M-phase buffer (pure addition) → **Step 3**
  convert `time_sync_mfsk_corr` to the M-phase buffers, owns the TOL=0
  dual-path Bug #44 test → **Step 4** remove the eager `:920` full-rate FIR +
  cleanup. Each step: independent build, own loopback/BER test, explicit
  rollback; eager FIR removed last.

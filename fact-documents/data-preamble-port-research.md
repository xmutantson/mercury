# Data Preamble Detector Port — Research / Planning

**Status:** PLAN ONLY (no code edits). Hand-off doc for the next implementation
agent.

**Origin:** `data-frame-cliff-audit-2026-05-27.md` §13.9 — the planned ~3-line
SUM-form fix to `time_sync_mfsk_corr` proved mathematically equivalent to the
old MEAN form (commit `b9d9910` reverted in `2be7b02`). The audit terminates
there with 4 escalation options; the parent agent has chosen **Option 2**
(re-architect the data preamble detector to mirror `detect_ack_pattern`'s
discrete-tone-match metric). This document scopes that port.

**Branch lineage:** `fix/preamble-extend` (commits 72a645e → 41ac734 base)
shipped (a) preamble 4→16 length extension and (b) per_sym_floor 0.05→0.01.
The metric-form change (§13) is reverted. The work documented here is the
NEXT change on top of the post-revert state.

**Driving evidence:**
- WGN:-8 IONOS cell: HAIL detects (uses `detect_ack_pattern`), CONNECT-START
  decodes (uses `detect_ack_pattern`), DATA preamble fails 320/320 polls
  with NO-PREAMBLE (uses `time_sync_mfsk_corr`). Same M=32 modulation, same
  baseband chain, same RF; only the metric form differs. The metric form IS
  the cliff.
- §13.9 derivation showed cosine²-MEAN with absolute threshold 0.5 has a
  detection-SNR floor at per-sample-SNR ≥ 0 dB (γ ≥ 1), independent of N.
  No matched-filter integration gain through the threshold decision.

---

## §1. Side-by-side study of the two detectors

### §1.1 `time_sync_mfsk_corr` (CURRENT DATA PREAMBLE PATH)

**Location:** `source/physical_layer/ofdm.cc:3023-3240`.
**Signature:**
```cpp
int cl_ofdm::time_sync_mfsk_corr(std::complex<double>* baseband_interp,
                                  int buffer_size_interp, int interpolation_rate,
                                  int search_start_symb, double* out_metric);
```

**Sole production caller:** `cl_telecom_system::receive_msg` MFSK branch at
`telecom_system.cc:1037`. Returns `receive_stats.delay` (interpolated sample
index, -1 = no preamble). Caller `receive_stats.mfsk_search_raw` advances on
each poll cycle so the search range scans forward over time.

**Input format:**
- Full-rate (interpolated) baseband complex samples. The buffer is populated
  in `telecom_system.cc:951-953` by `passband_to_baseband(...,
  FIR_rx_time_sync)` on the FULL-RATE buffer (MFSK-only — OFDM uses the
  decimated buffer, but MFSK keeps full-rate because of Bug #44 4× P1
  sub-symbol oversampling).
- `interpolation_rate` is the M from `data_container.interpolation_rate`
  (typically 4×).

**Internal stages:**
1. Pre-condition: `mfsk_corr_template != NULL`, `mfsk_corr_template_len > 0`,
   `mfsk_corr_template_nsymb > 0`. Template is a `Nofdm × template_nsymb`
   decimated-rate complex array generated at config-load by round-tripping
   the known preamble through `baseband_to_passband → passband_to_baseband
   (FIR_rx_time_sync) → decimate`. Stored in `mfsk_corr_template[]`
   (`telecom_system.cc:4956-5021`).
2. Per-symbol template energies precomputed into
   `mfsk_corr_template_sym_energy[16]` (`telecom_system.cc:5004-5021`).
3. **Phase 1 — coarse 4× oversampled search** (`ofdm.cc:3043-3148`):
   - Step `sym_period_interp / 4` (sub-symbol resolution, P1_OVERSAMPLE=4).
   - For each candidate position, for each template symbol k = 0..N-1:
     - Compute complex cross-correlation
       `corr_k = Σ_n rx[base + k*sym + n*interp] · conj(template[k*Nofdm + n])`
       over Nofdm = 292 baseband samples.
     - Compute per-symbol cosine² metric:
       `sym_metric_k = |corr_k|² / (template_sym_energy[k] · e_rx_sym)`.
     - **PER-SYMBOL FLOOR** (`ofdm.cc:3074`): `per_sym_floor = 0.01`
       (recently relaxed from 0.05). If any symbol falls below this, reject
       the entire candidate (`break`).
   - If candidate survives: `metric = total_metric / valid_syms` (MEAN of
     surviving sym_metrics).
   - Keep top-K=8 candidates sorted by metric.
   - **Early-exit** at `ofdm.cc:3146`: `if (metric > 0.5) break;` — first
     "clearly good" candidate wins.
4. **Phase 2 — fine refinement at base-rate** (`ofdm.cc:3159-3224`):
   - For each top-K candidate, search ±sym_period_interp at
     `interpolation_rate` step (base-rate resolution).
   - Same per-symbol cosine² MEAN metric (no per-sym floor in Phase 2,
     just gated by valid_syms).
   - Keep best fine metric across all candidates.
5. **Final threshold** (`ofdm.cc:3230`): `threshold = 0.5`. If
   `best_fine_metric < 0.5` → return -1.
6. Return `best_fine` (full-rate sample offset) and `*out_metric =
   best_fine_metric`.

**Threshold formula:** absolute `0.5` (length-invariant — see §13.9 of the
audit). Real preamble: ~1.0. Random MFSK data: up to ~0.44 cosine². Noise:
~0.004. The 0.5 cleanly separates noise & data at high SNR but does not
gain any margin from longer integration (proven in §13.9).

**Symbol diversity:** Preamble tones for M=32 are `{4,20,12,28}` repeated
4× with `tone_hop_step=13` baked in at init time (`mfsk.cc:134-140`). For
M=16, `{2,6,10,14}` repeated with `tone_hop_step=7`. Generated tone sequence
at TX (`mfsk.cc:467-481`) reads `preamble_tones[s % preamble_nSymb]` —
hopping is in the array, NOT applied at emit time.

**Coupling to mfsk_corr_template:** The template was built from the SAME
preamble symbols (`generate_preamble` at `telecom_system.cc:4967`). Each
candidate position correlates the RX signal against the round-tripped
known preamble WAVEFORM. This is matched-filter detection in continuous-
amplitude space.

### §1.2 `detect_ack_pattern` (CURRENT CONNECT/HAIL/ACK/BREAK PATH)

**Location:** `source/physical_layer/ofdm.cc:3244-3532`.
**Signature:**
```cpp
double cl_ofdm::detect_ack_pattern(std::complex<double>* baseband_interp,
    int buffer_size_interp, int interpolation_rate, int ack_nsymb,
    const int* ack_tones, int ack_pattern_len,
    int tone_hop_step, int mfsk_M,
    int nStreams, const int* stream_offsets,
    int* out_matched, int suffix_start, int* out_suffix_matched,
    int* out_best_offset, int reserve_after,
    uint32_t* out_match_mask, bool always_fine);
```

**Callers** (`telecom_system.cc`):
- `:3061` — `detect_ack_pattern_from_passband` (ACK / SACK).
- `:3184` — ACK SNR variant.
- `:3386` — **CONNECT base** (`decode_ctrl_suffix_from_passband`, the
  one that works at WGN:-8).
- `:3493` — BREAK.
- `:3555` — HAIL.

**Input format:**
- Decimated baseband complex samples (interpolation_rate = 1 in most
  callers). The caller pre-fuses mix + FIR + decimate via
  `passband_to_baseband_decimated`. The buffer reused is
  `data_container.baseband_data_interpolated` (sized for full-rate, but
  written as decimated by the caller).

**Internal stages:**
1. **Phase 1 — coarse search** (`ofdm.cc:3272-3387`):
   - Step `sym_period_interp` (1-symbol grid, NOT sub-symbol oversampled).
   - For each candidate symbol position `s`, for each pattern symbol
     `p = 0..ack_nsymb-1`:
     - Take FFT of 1 decimated symbol (Nfft samples after stripping GI).
     - Compute expected tone:
       `actual_tone = (ack_tones[p % ack_pattern_len] + p * tone_hop_step) % M`.
     - **Discrete match test** (`ofdm.cc:3302-3354`):
       - For each stream, find the bin with PEAK energy among the stream's
         M bins (FFT-bin argmax).
       - If peak == expected bin OR peak == mirror bin → `streams_matched++`.
       - Require ALL nStreams match for this symbol to count.
     - On match: `matched++` (discrete count).
     - Additionally compute continuous metric: `metric += e_target / e_total`
       across all Nc bins (used as secondary gate).
   - Track best_pos by `(matched DESC, metric DESC)` (matched dominates).
2. **Phase 2 — fine refinement** (`ofdm.cc:3402-3516`):
   - Triggered when `best_matched >= 6` (or `always_fine=true` for one-shot
     callers like BREAK).
   - Within ±sym_period_interp/2 of coarse_offset, step at `interpolation_rate`
     (base-rate resolution).
   - Same discrete-match algorithm; `matched_f`, `metric_f` accumulated.
   - Result wins if `(matched_f, metric_f) > (best_matched, best_metric)`.
3. **Return value:** the function itself returns `best_metric` (continuous,
   sum of per-symbol e_target/e_total ratios). Discrete-match count returned
   via `out_matched`. Sample offset via `out_best_offset` (interpolated
   sample offset).
4. **Threshold check happens at the CALLER**, not inside `detect_ack_pattern`.
   For CONNECT (`telecom_system.cc:3399`):
   ```cpp
   if (matched < ack_mfsk.connect_match_threshold || metric < 3.0 || best_offset < 0)
       return false;
   ```
   - `connect_match_threshold = 7` (out of 16). Length-scaled — set in
     `mfsk.cc:354,364,371`.
   - `metric ≥ 3.0` floor on the cumulative energy-ratio (each clean symbol
     contributes ~1.0; 3.0 ≈ 3/16 of symbols carrying clean energy).

**Threshold formula:** Length-scaled discrete count
`matched ≥ connect_match_threshold` AND continuous floor `metric ≥ 3.0`.
The discrete-count threshold scales with `connect_pattern_nsymb`.

**Symbol diversity:** WB CONNECT base uses Welch-Costas (g=3) — pairwise
unique difference vectors. Tones for M=32: `{6,18,20,26,10,30,22,16}` (8 base
tones × 2 reps = 16 sym), with `tone_hop_step=13` APPLIED AT EMIT TIME
(`mfsk.cc:498-499` for ACK pattern; CONNECT follows the same pattern via
`generate_ack_pattern`-style emit). ACK uses Welch-Costas g=5; BREAK g=7;
HAIL g=6 — all coprime to the M-pool so cross-correlation vs each other is
bounded ~1/8 per symbol (verified by `test_base_pattern_cross_correlation`).

---

## §2. Why `detect_ack_pattern` works at the low-SNR floor

### §2.1 Metric scaling math (per-sym FFT-bin argmax)

For each candidate position and each pattern symbol `p`:
- TX symbol places one tone of amplitude `A` in the bin
  `(ack_tones[p] + p*hop) % M` of each stream's M-bin window.
- After RF, the received symbol has bin energies
  `e_b = (A·H_b + N_b)²` where `H_b` is channel response and `N_b` is
  complex Gaussian noise with variance `σ²`.

The **discrete-match test** is "is the expected bin the maximum among
M candidate bins of the stream?" This is the same as a maximum-likelihood
choice over the M-ary alphabet (non-coherent MFSK).

For an M-ary noncoherent FSK symbol at per-sample in-band SNR `γ` (after
integrating over Nofdm samples → integrated SNR `γ_int = γ · Nofdm`):
- `P(correct | M-ary)` follows the Marcum-Q characterization.
- For M = 32, γ_int ≈ 4 (per-sym SNR ≈ 6 dB): `P(correct) ≈ 0.95`.
- For γ_int ≈ 1 (0 dB): `P(correct) ≈ 0.50`.
- For γ_int ≈ 0.25 (-6 dB): `P(correct) ≈ 0.20`.

Across `N` symbols of independent identically-distributed channel and noise
draws, the expected number of matches is `N · P(correct)`.

### §2.2 Threshold scaling — the matched-filter gain

Let `p = P(correct | γ_int)`. Number of matches `K ~ Binomial(N, p)`.

- `E[K] = N · p`.
- `Var[K] = N · p · (1-p)`.
- `Stddev[K] = √(N · p · (1-p))`.

For random data at non-preamble positions, each symbol's peak is uniformly
distributed over M bins → `p_random = 1/M = 1/32` (M=32 case).
- `E[K_data] = N/M = 16/32 = 0.5` for N=16, M=32.
- `Stddev[K_data] = √(N · (M-1)/M²) ≈ √(16 · 31/1024) ≈ 0.69`.

**Detection statistic (signal mean above data/noise mean in stddev units):**
```
Z = (E[K_signal] - E[K_random]) / Stddev[K_random]
  = (N·p - N/M) / √(N · (M-1)/M²)
  ≈ √N · (p - 1/M) · M / √(M-1)
  ≈ √N · M · (p - 1/M) / √M     for large M
```

**Z grows as √N** — the matched-filter integration gain. For p constant
(per-symbol SNR fixed), going N: 4 → 16 (current preamble 4-sym old → 16-sym
new): `√4 → √16` = 2× = **+6 dB detection-SNR**.

Combined with the **threshold scaling rule** (`matched ≥ N · θ` for some
fraction θ ∈ (1/M, p_target]), the threshold moves with N — so the
matched-filter gain shows up in the threshold *decision*, not just the
variance.

This is exactly what the §13.9 SUM-form analysis was missing: the
SUM-of-cosine² with threshold `0.5·N` is mathematically equivalent to
MEAN-with-threshold-0.5 because cosine² has no length-discriminating sample
statistic. Discrete-count, in contrast, has an explicit p-scaled mean that
length scales — N·p vs N/M.

### §2.3 False-alarm rate calculation (matched ≥ 7/16, M=32 case)

For random data: `p = 1/32`. K ~ Binomial(16, 1/32).
`P(K ≥ 7) = Σ_{k=7..16} C(16,k) · (1/32)^k · (31/32)^(16-k)`.

Numerically:
- C(16,7) ≈ 11440, (1/32)^7 ≈ 3·10^-11, (31/32)^9 ≈ 0.75 → ≈ 2.6·10^-7.
- Higher k contribute even less.
- **P(false_alarm) ≈ 2.5·10^-7 per poll** at threshold matched=7/16, M=32.

The CONNECT base pattern is M=16 (`ack_mfsk` is always M=16 for WB, even
when data is M=32 — see `mfsk_vara_parity_audit_2026_05_25.md`); for
matched=7/16 M=16:
- `P(false) ≈ 2.4·10^-5 per poll` (looser by 100× because random p=1/16
  not 1/32).
- This is the figure quoted in `mfsk.cc:195,204`.

For DATA preamble at M=32 with N=16: false-alarm bound is ~100× tighter
than CONNECT — the discrete-match approach is STRICTLY safer than CONNECT
because data's M=32 alphabet halves the random-p baseline.

Citation: `mfsk.cc:195` claims `P(false|M=32)=2.5e-7/poll for 7/16` — that's
the same number as derived here. Same hand-tuned threshold reasoning.

### §2.4 Why CONNECT works at WGN:-8 specifically

WGN:-8 in-band per-sample SNR is approximately -10 to -12 dB (the IONOS WGN
spec is total-band noise; the FIR_rx_time_sync bandpass strips ~3 dB at the
WB skirts). Per-symbol integrated SNR `γ_int = γ · Nofdm = (3·10^-2) · 292
≈ 8.8` → **per-sym SNR ≈ 9.4 dB** → `p ≈ 0.85` for M=16 non-coherent.

- E[K] ≈ 16 · 0.85 = 13.6.
- Stddev[K] ≈ √(16·0.85·0.15) ≈ 1.43.
- P(K ≥ 7) is essentially 1.0.

That's why CONNECT works: the per-sym FFT-bin argmax wins easily at WGN:-8,
and the 7/16 threshold has huge margin. Same channel conditions, the
cosine²-MEAN detector chokes because its threshold has no length-scaling.

### §2.5 Prior-art validation

The discrete-tone-match approach is the standard low-SNR detection method:
- **WSPR** (Joe Taylor, K1JT, 2008): coherent integration over 162 symbols
  with discrete tone-match at -33 dB SNR. Mercury's per-sym FFT-bin argmax
  is the non-coherent analog.
- **FT8** (`ft8_lib`, Karlis Goba): 79-symbol Costas + Gray-coded payload
  with per-sym FFT-bin argmax. Detection at -21 dB SNR achieved through
  long integration + discrete-tone metric. Mercury's existing
  `detect_ack_pattern` is structurally identical.
- **GNU Radio MFSK** (`gr-mfsk_freq_sync`): matched-filter on tone
  sequences with discrete-bin argmax + cumulative-score threshold. Same
  pattern.
- **CLAUDE.md §1 prior-art mandate satisfied**: the new metric form has
  3+ documented existing implementations; this is not invented.

---

## §3. Why DATA preamble can't just call `detect_ack_pattern` verbatim

### §3.1 Structural mismatches

1. **M difference.** Data preamble runs at the data config's M (M=32 for
   ROBUST_0, M=16 for ROBUST_1/2). CONNECT/HAIL/ACK base patterns run at
   `ack_mfsk.M` which is always 16 for WB (regardless of data M). Calling
   `detect_ack_pattern` with `mfsk_M = 32` against `ack_mfsk.connect_tones`
   would alias bin numbers (the tones array is in M=16 space). For DATA we
   need a SEPARATE tone array sized for the data M, plus stream_offsets
   aligned to the data MFSK's stream allocation.

2. **Tone sequence.** Current preamble tones are
   `{4,20,12,28}` (M=32) and `{2,6,10,14}` (M=16) — only 4 base tones.
   Extended to 16 symbols via the per-symbol hopping baked into
   `preamble_tones[]` (`mfsk.cc:138-139,145-146`). To use
   `detect_ack_pattern`, we either:
   - Pass the existing 16-element `preamble_tones[]` as the `ack_tones`
     array with `ack_pattern_len = 16` and `tone_hop_step = 0` (hopping
     already baked in).
   - OR redesign tones to a Costas-like Welch sequence so the per-symbol
     uniqueness is structural rather than hop-driven.
   - Option (a) is simpler and matches what's already in TX. Verify the
     `(tone + p*hop) % M` math in `detect_ack_pattern` works with hop=0:
     `actual_tone = (preamble_tones[p % 16] + p * 0) % M = preamble_tones[p]`.
     Yes, hop=0 works.

3. **Sample-level delay precision.** `detect_ack_pattern` returns
   sample-level offset via `out_best_offset` at base-rate resolution
   (`interpolation_rate` step in Phase 2). For DATA decode, the downstream
   consumer is `receive_msg` post-preamble — it reads
   `preamble_nSymb + Nsymb = 324` symbols starting at `delay`. Each data
   symbol is `Nofdm = 292` baseband samples; GI = 36. A delay error of
   ≤ Ngi samples (36 baseband ≈ 144 full-rate) keeps the data inside the
   GI window. `detect_ack_pattern`'s base-rate step is ≤ 4 full-rate
   samples — well below the GI tolerance. **Precision is sufficient.**

4. **Search-start parameter.** `time_sync_mfsk_corr` takes
   `search_start_symb` to skip past previously-detected preambles
   (`mfsk_search_raw` in `receive_stats`). `detect_ack_pattern` does NOT
   have this parameter — it always scans from 0. **The port MUST add this
   functionality** or break the anti-re-decode logic. Simple change: pass
   `search_start_symb` and start the outer loop `s = search_start_symb`.

5. **Buffer rate / format.** `time_sync_mfsk_corr` consumes full-rate
   interpolated baseband (4× oversampled). `detect_ack_pattern` consumes
   decimated baseband (1× — the caller pre-decimates via
   `passband_to_baseband_decimated`). Two options:
   - (A) Run the new detector on the DECIMATED buffer (cheaper FFT,
     identical to CONNECT detector pipeline). REQUIRES the caller to
     populate `baseband_data_decimated` for MFSK when this detector is
     selected — today that path is gated by
     `if(M != MOD_MFSK || ofdm.mfsk_corr_template == NULL)` at
     `telecom_system.cc:971-979`. Lift the gate.
   - (B) Run on full-rate interpolated buffer (matches current Bug #44 path
     for sub-symbol resolution). Larger compute (4× FFTs), but no caller-side
     plumbing changes. Note Bug #44 originally needed P1_OVERSAMPLE=4 to
     resolve half-symbol misalignment; the discrete-count metric is less
     sensitive to sub-symbol misalignment because FFT bin argmax stays
     correct as long as the GI absorbs the offset. **At base-rate Phase 2
     step, `detect_ack_pattern` already handles ± half-symbol misalignment
     within ±sym_period/2 search; no oversampled P1 needed.** Choose (A).

6. **OFDM path.** `time_sync_mfsk_corr` is **MFSK-ONLY** in production
   (`telecom_system.cc:1037`, in the `if(M == MOD_MFSK)` branch). OFDM
   data configs use `time_sync_preamble_halfsym` (Schmidl-Cox). Cross-check
   via `Grep "time_sync_mfsk_corr"`: only one production caller plus the
   regression test. **No OFDM impact.** The audit document §2.1 reflects
   this; the prompt's mention of "the 4 OFDM configs share
   time_sync_mfsk_corr" is incorrect — they share `time_sync_preamble_halfsym`.
   This narrows scope considerably.

### §3.2 Per-consumer assumption check

| Consumer field / param | What it depends on today | Survives the port? |
|---|---|---|
| `receive_stats.delay` (full-rate sample idx) | `time_sync_mfsk_corr` return | Yes — port wraps return value with `* interpolation_rate` IF detector runs on decimated buffer (option A). At Phase 2 base-rate resolution, the same precision is achievable. |
| `receive_stats.mfsk_search_raw` (anti-re-decode) | `search_start_symb` arg | Yes IFF new function accepts a similar arg. Must add. |
| `receive_stats.coarse_metric` (diagnostic only) | `*out_metric` | Magnitude changes (was cosine²-mean ≤ 1, becomes discrete-match count ≤ N). Logging consumers and dashboards must be aware (similar concern raised in §13.7 of audit). Non-load-bearing. |
| `data_container.preamble_nSymb` (buffer math) | Already 16 post-`fix/preamble-extend` | Unchanged. |
| Frame extraction (`receive_msg` body after delay-check) | `delay` precision ≤ Ngi | Yes — base-rate Phase 2 precision ≤ 4 full-rate samples << Ngi=144 full-rate samples. |
| `mfsk_corr_template` (template buffer) | DELETED in port (FFT-bin argmax doesn't need a template) | Deleted producer at `telecom_system.cc:4956-5021` becomes dead code. The `mfsk_corr_template_sym_energy[16]` array becomes dead. Cleanup is optional / nice-to-have. |
| ARQ timing math (`message_transmission_time_ms`, etc.) | `preamble_nSymb + Nsymb` | Unchanged (preamble length stays 16). |
| Bug #44 anti-data-content false-trigger | `per_sym_floor = 0.01`, cosine² | New approach: M-ary FFT-bin argmax (see §6 below). |

### §3.3 Search-start parity gap

The current detector has anti-re-decode plumbing via `search_start_symb`.
The port MUST preserve this — otherwise after a successful preamble decode
the next poll cycle re-finds the same preamble and ARQ misbehaves.

`detect_ack_pattern` does NOT have this argument. Fix: add a new wrapper
in `ofdm.cc` (or extend `detect_ack_pattern` itself) with an optional
`search_start_symb` parameter, starting the outer loop at that position.
Trivial change (one line).

---

## §4. Implementation strategy options

### §4.A — VERBATIM PORT (recommended)

**What:** Replace `time_sync_mfsk_corr`'s body with a `detect_ack_pattern`-
style loop, keeping the existing signature for callers. New name
`time_sync_mfsk_argmax` or just rebuild the existing function body. Keep
`mfsk_corr_template` infrastructure as DEAD code (don't delete in this
patch — Phase-2 cleanup).

**Files touched:**
1. `source/physical_layer/ofdm.cc` — rewrite body of `time_sync_mfsk_corr`
   to use FFT-bin argmax + discrete-match count. ~180 LoC replacement.
2. `source/physical_layer/telecom_system.cc:1037` — caller unchanged
   (same signature).
3. `source/physical_layer/telecom_system.cc:971-979` — lift the
   `mfsk_corr_template == NULL` gate so the DECIMATED buffer is populated
   for MFSK. Or keep full-rate input (option B); Option A preferred.
4. `source/physical_layer/mfsk.cc:134-147` — change `preamble_tones[]` to
   a Welch-Costas-style sequence (8 distinct base tones × 2 reps × hop).
   Cross-correlation against ACK/BREAK/HAIL/CONNECT bases must remain
   ≥ pairwise-Hamming 6 (per `test_base_pattern_cross_correlation`).
5. `source/physical_layer/mfsk.h` — add `preamble_match_threshold` field
   (parallel to `ack_match_threshold`, `connect_match_threshold`).
6. `source/physical_layer/telecom_system.cc:4956-5021` — optionally
   shortcut the corr-template generation (no-op if the new function
   doesn't read it). Leave for Phase-2 cleanup.

**LoC estimate:** ~250 LoC changed (rewrite the 220 LoC of
`time_sync_mfsk_corr` + ~30 LoC adjustments at callers/inits/tone tables).

**Risks:**
- New tone sequence vs all existing CONNECT/ACK/HAIL/BREAK Costas patterns —
  must verify cross-correlation < ~5/16 across all pairs.
- Discrete-match threshold tuning at WGN:-8 (target floor) and WGN:+14
  (high-SNR no-regression). Walk the §7 test matrix below.
- Phase-2 fine refinement: detect_ack_pattern's `always_fine` rule. For
  data preamble, we DO want fine refinement on EVERY detection (not just
  matched ≥ 6) because the data symbols that follow rely on
  sample-precise alignment. Use `always_fine = true`.

**Effort:** 3-4 days from planning to merged HEAD (~1 day code, ~1 day
unit/regression tests, ~1 day hardware A/B at IONOS).

### §4.B — HYBRID METRIC

**What:** Keep cosine² per-symbol shape-gate (relaxes Bug #44 from §6.0
of the audit) as the precision-of-alignment gate, but use discrete-match
count as the PRIMARY length-scaled detection metric.

**Architecture:**
- Phase 1: FFT-bin argmax discrete match (count + total energy ratio).
- Phase 2: fine refinement also uses discrete match.
- Optional secondary gate: per-symbol cosine² ≥ shape_floor on the
  AT-LEAST-K best symbols (similar to detect_ack_pattern's matched
  count itself but in cosine² space).

**Pros:**
- Belt-and-suspenders for false-positive rate.
- Preserves bug #44 testimony layer (per-symbol cosine² bounded ≤ 1).
- Easier rollback (each gate is independently tunable).

**Cons:**
- Compute cost: every Phase 1 candidate position now FFTs every symbol
  AND time-domain correlates against the template. Wasted compute.
- More state (template still needed) — code complexity grows.
- Two thresholds to tune; harder to reason about regression.

**Files touched:** Same as Option A + template stays alive + ~100 extra
LoC for the cosine² shape-gate.

**LoC estimate:** ~350 LoC changed.

**Risks:** Combining the two metrics requires careful AND/OR logic. If
either gate is too strict the floor stays at WGN:-4; if too loose, FAR
rises. Tuning surface is 2D.

**Effort:** 6-8 days.

**When to prefer this:** If the verbatim port's empirical FAR ends up
worse than expected on the WGN:-8 IONOS cell.

### §4.C — FULL REFACTOR (top-level detection dispatch)

**What:** Introduce a new top-level `detect_preamble_pattern` function that
owns both MFSK preamble and CONNECT/ACK/HAIL/BREAK detection via a unified
parameter set. Move `detect_ack_pattern` to call this. Delete
`time_sync_mfsk_corr` body entirely.

**Pros:**
- One detection algorithm to maintain. The current code has THREE preamble
  detectors (`time_sync_mfsk`, `time_sync_mfsk_corr`, `detect_ack_pattern`)
  plus the OFDM ones. Consolidation is the long-term right answer.
- Clean dispatch: pattern selection by enum (PREAMBLE/CONNECT/ACK/...).

**Cons:**
- Touches stable callers (ACK detection at lines 3046, 3184; BREAK at 3493;
  HAIL at 3555). Larger blast radius.
- Risk of regressing the CONNECT/HAIL paths that currently work.

**Files touched:** ~10 files (all callers of detect_ack_pattern + new
top-level + cleanup).

**LoC estimate:** ~500-700 LoC.

**Effort:** 10-15 days.

**Risks:** breaks fix-narrow rule (CLAUDE.md §5); cross-layer regression
test matrix expands.

**When to prefer this:** After Option A ships and the cliff is moved,
schedule Option C as planned consolidation work.

### §4.D — RECOMMENDATION

**Ship Option A**, schedule Option C as Phase-2 cleanup. Option A is the
minimal change that closes the cliff gap; Option C is the right end-state
but ships after the regression risk is amortized over the simpler patch.

---

## §5. Cross-layer audit

Most of this is already in `data-flow-preamble_nSymb.md`. Adding the
metric-form change layer:

### §5.1 Producers / Consumers of the NEW metric

**Producer:** new `time_sync_mfsk_corr` body (or renamed). Returns delay
and writes `*out_metric` = discrete-match count (was cosine²-mean).

**Consumers** of `*out_metric` post-port:
1. `telecom_system.cc:1037` — value stored in `mfsk_sync_metric` and
   propagated to `receive_stats.coarse_metric` in some paths. **Diagnostic
   only** — verified by `Grep "mfsk_sync_metric"`. No flow-control reads.
2. `[RX-DECODE#N] NO-PREAMBLE`/`FAIL` log line at
   `arq_common.cc:5798-5811`. Prints `receive_stats.coarse_metric` for
   FAIL case. Magnitude change from 0..1 cosine² to 0..N discrete-match —
   log-parsing tools will need updates. Affected scripts:
   - `tools/analyze_turboshift_log.py` (grep `mfsk_sync_metric`).
   - `tools/sack_lossy_ab.py` (parses RX-DECODE lines).
   - `tools/axis_walk_sweep.py` (parses RX-DECODE).
   - Recommended: print BOTH values in a transition window
     (e.g. `metric_match=11/16 metric_legacy=0.84`) for the first week of
     deployment, then drop the legacy print.

### §5.2 Producers of preamble_tones[]

§1 of `data-flow-preamble_nSymb.md` covers `mfsk.cc:122-170`. The fix
introduces NEW tone sequences (Welch-Costas g=? for preamble) — must:
- Pick a primitive root that's DISTINCT from ACK (g=5), BREAK (g=7),
  HAIL (g=6), CONNECT (g=3). Available: g=2, g=10, g=11, g=14 (all
  primitive mod 17). Recommend `g=2` for M=16 (cleanest pairwise distance
  vs the existing four patterns).
- Update `test_base_pattern_cross_correlation`
  (`mfsk_ctrl_codec_tests.cc:166+`) to assert pairwise Hamming-tone ≥ 6
  between PREAMBLE and {ACK, BREAK, HAIL, CONNECT}.

### §5.3 New invariant: preamble_match_threshold

Same `data-flow-preamble_nSymb.md` style — add:
- `cl_mfsk::preamble_match_threshold` = 7 (out of 16, mirror ack/connect).
- INV: `preamble_match_threshold ≤ preamble_nSymb`.
- INV: `preamble_match_threshold` > `(preamble_nSymb / M)·3` (3-sigma false-
  alarm bound).

### §5.4 The mfsk_corr_template dead-code question

If Option A keeps `mfsk_corr_template` infrastructure alive (just unused):
- Slight memory waste (~4 KB per config).
- Zero risk of accidentally re-introducing the old path.
- Easy Phase-2 cleanup.

If Option A deletes the template generator:
- Cleaner, but the audit needs to verify that NO other consumer of
  `mfsk_corr_template_*` exists. Quick `Grep` confirms only
  `time_sync_mfsk_corr` reads it. **Safe to delete.**

Recommendation: leave the template alive in the first commit (minimum-risk
change), delete in a follow-up cleanup commit.

---

## §6. Bug #44 risk analysis — does discrete-match false-trigger on data?

### §6.1 Bug #44 history recap

`time_sync_mfsk_corr` exists because the pure FFT-energy metric
(`time_sync_mfsk` at `ofdm.cc:2868-3009`) false-triggers on in-band MFSK
data content (the bins of expected preamble tones have nontrivial energy
in random data symbols too — sum-of-energy-ratios at non-preamble positions
can reach `metric ≈ preamble_nSymb · 0.25` which exceeds the
`preamble_nSymb · 0.3` (NB) / `preamble_nSymb · 0.5` (WB) threshold ~25% of
the time). Bug #44's fix introduced the corr-template path with cosine²
per-symbol normalization (bounded ≤ 1 per symbol → ≤ N total — natural
ceiling).

The audit's §13.9 concern: removing the bounded-per-symbol property
(Option B "global noise reference") re-exposes the false-trigger.

### §6.2 Discrete-match doesn't have the Bug #44 problem

`detect_ack_pattern`'s discrete-match metric is **fundamentally different
from `time_sync_mfsk`'s energy-ratio metric.** It's based on FFT-bin
ARGMAX, not on energy summed in expected bins.

For data content at a non-preamble position:
- Each data symbol tx-emits ONE tone in ONE bin (one-hot in M-bin window).
- After RX: argmax = the actually-transmitted tone (at high SNR) or
  random (at low SNR).
- The probability that argmax happens to land on the expected preamble
  tone at this symbol index is:
  - At HIGH SNR (data symbols decode cleanly): probability that the
    transmitted data tone == the preamble's expected tone at this index =
    1/M (one specific bin out of M).
  - At LOW SNR: argmax becomes uniform over M → also 1/M.
- Across N=16 symbols: **E[K_data] = 16/M = 0.5 matches** (M=32).
- Threshold matched ≥ 7. **No false-trigger.**

This is structurally why discrete-match is safer than cosine²:
cosine² accumulates energy from non-target bins via FIR leakage and
spectral correlation; discrete-match is binary (peak in right bin or not).

### §6.3 Worst-case data adversary

What if a data symbol HAPPENS to transmit the exact tone the preamble
expects at this symbol index? Probability per symbol = 1/M = 1/32 (M=32).
This is the random-data baseline already accounted for. There's no
non-random adversarial case in the production data path because data
content is interleaved and LDPC-encoded — it's effectively uniform.

### §6.4 Pure-noise false-alarm rate

At γ=0 (pure noise), `p_random = 1/M` exactly. P(K ≥ 7 | N=16, p=1/32):
~2.5·10^-7 per poll. With ~10 polls/sec, expected one false alarm every
~46 days continuous. **Acceptable** — far better than the current cosine²
form which has no length-scaled threshold and so admits more noise hits.

### §6.5 The 0.05 → 0.01 floor relaxation interacts how?

The audit's §H2 relaxed per_sym_floor 0.05 → 0.01 to admit more low-SNR
detections. Discrete-match doesn't have a per-symbol floor — it's
binary at the bin level. The Bug #44 anti-data testimony is structurally
embedded in the argmax operator. The floor parameter is irrelevant in
Option A.

### §6.6 Conclusion

Discrete-match does NOT re-introduce Bug #44. The per-symbol argmax is
self-bounded (one peak per symbol) and the random-data baseline is 1/M
per symbol — well below any reasonable threshold. **Option A is safer
than Option B and at least as safe as the current cosine² form.**

---

## §7. Mandatory regression test design

Each test MUST fail-before-passes on the current code, pass after the
port. Add tests to `mercury/source/physical_layer/mfsk_ctrl_codec_tests.cc`
(in-process — no IONOS).

### §7.1 `mfsk_data_preamble_argmax_clean` (high-SNR no-regression)

- `load_configuration(ROBUST_0)`.
- Generate preamble, round-trip through symbol_mod → baseband_to_passband →
  passband_to_baseband(FIR_rx_time_sync) — same as
  `mfsk_data_preamble_passband_roundtrip_clean`.
- Add trailing pad. Invoke the new detector.
- **Assert:** `matched >= 14/16` AND `delay within ±1 symbol of injection`.
- **Pre-fix:** existing detector passes on clean — this test passes pre-fix
  with the OLD metric form too. The fix-before-pass test is §7.2.

### §7.2 `mfsk_data_preamble_argmax_cliff` (FAIL-BEFORE)

- Same as §7.1 but ADD AWGN at WGN:-8 in-band SNR.
  - Compute per-sample passband energy of preamble.
  - Add Gaussian noise with variance such that
    `10·log10(signal_pb / noise_pb) = -8 dB` after the FIR bandpass.
- Invoke the detector.
- **Assert:** `delay >= 0` AND `matched >= preamble_match_threshold (=7)`.
- **Pre-fix:** the existing cosine²-MEAN detector returns -1 with
  `metric ≈ 0.15-0.30` at WGN:-8 (well below 0.5). Test FAILS.
- **Post-fix:** discrete-match detector finds 9-13 matches out of 16.
  Test PASSES.
- **Determinism:** seed PRNG, use ≥ 5 distinct seeds in the test, accept
  if ≥ 4 of 5 pass (acknowledge statistical variance at the cliff).

### §7.3 `mfsk_data_preamble_argmax_below_cliff` (boundary)

- Same as §7.2 but WGN:-10 in-band SNR (per-sample SNR ≈ -12 dB).
- **Assert:** at least 3 of 5 PRNG seeds detect.
- **Pre-fix:** existing detector fails 5/5 (cliff at WGN:-4 to -8).
- **Post-fix:** new detector should achieve ≥ 3/5 (matching CONNECT
  floor; CONNECT decodes at WGN:-8 with margin).
- Documents the achievable cliff position; allowed to be soft (3 of 5
  rather than 5/5) because we are at the edge.

### §7.4 `mfsk_data_preamble_argmax_pure_noise` (false-alarm guard)

- Buffer = pure WGN, no preamble.
- Invoke the detector 100 times with different noise seeds.
- **Assert:** at most 1 false positive (delay >= 0) across 100 trials.
- Target false-alarm rate: ~ 2.5·10^-7/poll · 100 polls ≈ 2.5·10^-5
  expected false positives. With safety margin, the bound is "at most 1".
- **Pre-fix:** cosine²-MEAN detector has FAR ~ 0.4% per poll on pure noise
  (we don't have data confirming this — but cosine² with floor=0.01 is
  permissive). It likely fails this test pre-fix too — confirming the
  Bug #44 risk concern.

### §7.5 `mfsk_data_preamble_argmax_data_content` (Bug #44 regression guard)

- Generate a buffer of MFSK DATA symbols (no preamble), 100 random LDPC
  codewords at the data M.
- Invoke the detector.
- **Assert:** detector returns -1 (no preamble found) on every codeword.
- **Pre-fix:** cosine²-MEAN with floor=0.01 may false-trigger on data
  with metric up to 0.44 (per audit §13.9). Test may fail pre-fix.
- **Post-fix:** discrete-match's argmax property bounds E[K_data] = 16/32 =
  0.5 << 7. Test passes.

### §7.6 `mfsk_data_preamble_argmax_search_start_skip`

- Inject two preambles at sample offsets `Pa` and `Pb`.
- First call: detector returns `Pa`.
- Second call with `search_start_symb = (Pa / sym_period) + 1`: detector
  returns `Pb`, NOT `Pa`.
- **Pre-fix:** existing `search_start_symb` machinery works in
  `time_sync_mfsk_corr`. Test passes.
- **Post-fix:** must continue to pass. Guards against accidentally dropping
  this parameter during the port.

### §7.7 `mfsk_data_preamble_argmax_cross_correlation`

Extend the existing `test_base_pattern_cross_correlation`
(`mfsk_ctrl_codec_tests.cc:166+`) to include the new PREAMBLE tone sequence.

- Compute pairwise tone-Hamming distance between PREAMBLE and each of
  {ACK, BREAK, HAIL, CONNECT}.
- **Assert:** all distances ≥ 6/16.
- Guards against accidentally choosing a primitive root that makes the
  PREAMBLE indistinguishable from CONNECT (etc.).

### §7.8 Hardware A/B (post-merge)

Outside `mercury.exe --test`, but on the deployment checklist:
- IONOS sweep WGN ∈ {+14, +6, 0, -4, -8, -10, -12}, ROBUST_0, 180s dwell.
- Expect: data bps non-zero at WGN:-8 (current: 0). Cliff target: WGN:-10.
- Comparison harness: `tools/axis_walk_sweep.py --pin-config 100`.

---

## §8. Architectural decisions requiring user input

### §8.1 Option A vs B vs C

**My recommendation: Option A.** Reasoning:
- Lowest blast radius (one detector replaced; callers untouched).
- Best matches CLAUDE.md §5 fix-narrow discipline (the audit explicitly
  identified the data preamble detector as the bottleneck; touching only
  it is the focused change).
- Option C is the right end-state but ships after A.
- Option B's compute cost + two-knob tuning surface argues against it for
  a first attempt.

**Decision needed:** confirm Option A. If user wants belt-and-suspenders
(B), say so before code.

### §8.2 New tone sequence

The current preamble tones `{4,20,12,28}` (M=32) and `{2,6,10,14}` (M=16)
have 4 base tones, each repeated 4× with hopping baked into
`preamble_tones[]`. Going to Costas-friendly (Welch-Costas g=2 for M=16,
scaled 2× for M=32):

- M=16: `2^k mod 17, k=1..8` = `{2, 4, 8, 16, 15, 13, 9, 1}` (8 base tones
  × 2 reps = 16). Trailing 16 substitutes to 8 (already used by CONNECT —
  collision). Substitute to a value not used by CONNECT={3,9,10,13,5,15,11,8}.
  Free values mod 16: {0, 6, 7, 12, 14}. Pick **0**. New M=16 PREAMBLE
  tones: `{2, 4, 8, 0, 15, 13, 9, 1}`.
- M=32: 2× scaled M=16 → `{4, 8, 16, 0, 30, 26, 18, 2}`.

Cross-correlation vs existing patterns (to be verified by §7.7 test):
- vs ACK M=32 `{8, 14, 10, 24, 26, 2, 18, 30}`: shared {2, 18, 26} (3
  collisions / 8 base = OK but tight).
- Recommend: tune base tones manually if shared-bin count is too high.
  This is a 30-min hand-design with a small unit test.

**Decision needed:** approve the tone-design approach (Welch-Costas with
hand-tuned collision-avoiding substitution) or specify alternative.

### §8.3 Hard cutover vs CAP-negotiated

Both endpoints must agree on the preamble tone sequence (TX-generated must
match RX-template / RX-expected). Pre-port endpoints emit and detect the
OLD `{4,20,12,28}`-repeating sequence; post-port endpoints emit and detect
the new Costas sequence. They are mutually incompatible.

**My recommendation: hard cutover** — matches the 2026-05-24 MFSK ACK
flag-day precedent (BUG #51 reroll, CONNECT base flag-day, etc.) and
avoids a feature flag we'd never remove.

**Decision needed:** confirm hard cutover.

### §8.4 mfsk_corr_template lifecycle

Two options after the port:
- (a) Leave template generation alive but unused (dead code, ~4 KB
  memory per config). Easy revert.
- (b) Delete template + per-sym energy arrays + `mfsk_corr_template`
  pointer. Clean, but harder revert.

**My recommendation: (a) for the cliff-fix commit, (b) as a Phase-2
cleanup commit one week later.**

**Decision needed:** confirm cleanup cadence.

### §8.5 Drop OFDM-mode dispatch?

**Already moot.** `time_sync_mfsk_corr` is MFSK-only in production
(`telecom_system.cc:1037`). The OFDM data path uses
`time_sync_preamble_halfsym` (Schmidl-Cox). No dual-mode dispatch exists
or needs to be removed.

### §8.6 Phase-2 always-fine?

`detect_ack_pattern` runs Phase-2 fine refinement only when
`matched >= 6 || always_fine`. For data preamble, we need sample-precise
alignment for the data symbols that follow — `always_fine = true` is the
correct choice.

**Decision needed:** confirm `always_fine = true` for data preamble detection.

### §8.7 Threshold value

Default `preamble_match_threshold = 7` (mirror CONNECT). This gives FAR ≈
2.5·10^-7/poll at M=32 (calculation in §2.3). Conservative.

**Decision needed:** confirm 7/16 as the production threshold, or specify
a value (range 6-8 is reasonable). 7 is the lowest-risk default.

### §8.8 Compute cost change

Per call:
- OLD `time_sync_mfsk_corr`: time-domain inner product over Nofdm=292
  samples × 16 symbols × ~16 candidate positions = ~75K complex MACs per
  poll.
- NEW (FFT-bin argmax): 1 Nfft=256 FFT per symbol × 16 symbols × 16 candidate
  positions = 256 FFTs per poll ≈ 256 · 256·log(256) ≈ 0.5M MACs per poll.

Roughly 7× more compute per poll. ROBUST_0 polls happen every ~100ms,
so absolute cost goes from ~75 μs to ~500 μs per poll on a Pi —
**non-binding** vs the LDPC decode budget (~30 ms per frame).

If profiling on Pi shows this matters, the FFTs can run on the DECIMATED
buffer (already pre-decimated by the caller in Option A.A buffer change)
which is 4× cheaper. Net: ~2× more than current, still tiny.

---

## §9. Cross-references

- `data-frame-cliff-audit-2026-05-27.md` §13.9 — the failed §13 sum-form fix.
- `data-flow-preamble_nSymb.md` — preamble-length 4→16 audit (shipped).
- `phase-b-mfsk-connect-research.md` §11-§14 — CONNECT detector port
  (reference template for what a successful detector port looks like).
- `mfsk-robust-ack.md` — MFSK ACK port (similar architectural pattern;
  successful precedent).
- `mfsk_vara_parity_audit_2026_05_25.md` — Phase 0 / A.1.4 history.

## §10. Source-file cite summary

| File:line | What |
|---|---|
| `source/physical_layer/ofdm.cc:3023-3240` | `time_sync_mfsk_corr` (current data preamble detector) |
| `source/physical_layer/ofdm.cc:3244-3532` | `detect_ack_pattern` (template for the port) |
| `source/physical_layer/ofdm.cc:2868-3009` | `time_sync_mfsk` (FFT-energy fallback; reference) |
| `source/physical_layer/telecom_system.cc:1037` | Single production caller of `time_sync_mfsk_corr` |
| `source/physical_layer/telecom_system.cc:3386-3399` | CONNECT detection — caller of `detect_ack_pattern`, threshold check pattern |
| `source/physical_layer/telecom_system.cc:4956-5021` | `mfsk_corr_template` generation (will become dead code post-port) |
| `source/physical_layer/telecom_system.cc:951-979` | MFSK passband→baseband buffer population (gate to lift if running on decimated) |
| `source/physical_layer/mfsk.cc:122-180` | `preamble_tones[]` init (fix site for new Costas sequence) |
| `source/physical_layer/mfsk.cc:345-381` | CONNECT-base tones reference for Welch-Costas hand-tuning |
| `source/physical_layer/mfsk.cc:467-481` | `generate_preamble` (TX) — verify hopping math under hop=0 |
| `include/physical_layer/mfsk.h:50-58` | `preamble_tones[MAX_PREAMBLE_SYMB=16]`, `preamble_nSymb` |
| `include/physical_layer/mfsk.h:77-78` | `connect_pattern_nsymb`, `connect_match_threshold` (reference pattern for new `preamble_match_threshold`) |
| `include/physical_layer/ofdm.h:241-246` | `mfsk_corr_template*` (dead code post-port) |
| `source/physical_layer/mfsk_ctrl_codec_tests.cc:1063-1214` | Existing roundtrip test — extend with §7.2-§7.5 tests |
| `source/physical_layer/mfsk_ctrl_codec_tests.cc:166+` | `test_base_pattern_cross_correlation` (extend per §7.7) |
| `source/datalink_layer/arq_common.cc:5798-5811` | RX-DECODE diagnostic — verify magnitude change doesn't break log parsing |

## §11. Open questions [?]

1. **[?] Does the data symbol payload occasionally land entirely on the
   expected preamble tones?** §6.3 argues this is uniform-random in
   practice, but a pathological codeword could in principle hit all 16
   expected tones. Verify with a brute-force adversarial codeword search
   if §7.5 test failures are observed.

2. **[?] Phase-2 fine refinement compute budget on Pi.** §8.8 estimates
   total cost ~2-7× current. Profile-mode build (`-pg`) on Pi will confirm.
   The escape valve is: keep P1_OVERSAMPLE=1 (drop the 4× Bug #44 path
   since discrete-match doesn't need it), giving ~4× compute reduction
   from the old.

3. **[?] Does the dropped P1_OVERSAMPLE break Bug #44 in some adjacent
   path?** The original Bug #44 motivation was sub-symbol alignment
   resolving half-symbol misalignment that confused the cosine² metric.
   Argmax doesn't have this issue (peak bin stays correct under
   misalignment within the GI). Verify by ablation test: run new detector
   with P1_OVERSAMPLE=1 on clean signal — should match P1_OVERSAMPLE=4
   performance.

4. **[?] Will WGN:-10 actually work?** §3 estimated cliff move WGN:-4 →
   ~WGN:-10 based on √N scaling. The actual cliff may be limited by
   per-symbol SNR (probability-of-correct), not just integration length.
   At WGN:-12, per-sym integrated SNR γ_int ≈ Nofdm · 10^(-12/10) ≈ 18
   (12.5 dB) which still has p ~ 0.95 for M=16, p ~ 0.85 for M=32 — so
   matched ≥ 7/16 should still pass. Cliff is theoretically ≈ WGN:-15 to
   -20 for M=16 noncoherent; M=32 is harder by ~3 dB. Hardware A/B will
   resolve.

5. **[?] Search-start-symb performance.** With WAY more poll iterations
   per second (RX runs `receive_msg` continuously), the
   `search_start_symb` skip optimization matters. The new detector MUST
   honor it cleanly. The data-flow audit in §5.1 lists this as a
   producer; verify the consumer-side advance still works post-port.

6. **[?] False-alarm window across a session.** §6.4 derived
   FAR ≈ 2.5·10^-7/poll. ROBUST_0 session at 84 bps lasts ~12 hours
   typical → ~432K polls → expected 0.1 false alarms per session.
   Acceptable. But the false-alarm CONSEQUENCE matters: a spurious
   delay-pointer makes the LDPC decoder run on noise, producing CRC fail,
   wasting ~30 ms compute. Not catastrophic, just wasted compute. The
   §H2 0.01 floor doesn't apply (no per-sym floor in discrete-match).

---

## §14. Discrete-match port (pre-code)

**Status:** PLAN — to be implemented on branch `fix/preamble-discrete-match`
in the worktree `mercury-worktrees/preamble-discrete-match`. Author makes
the autonomous decisions from §8 (Option A, hard cutover, threshold=7/16,
always_fine=true, template cleanup deferred). Worktree baseline e198ecf
(post-revert state on top of monitor); `mercury.exe --test` 17/17 pass.

### §14.1 Welch-Costas g=2 tone sequence — Hamming-distance verification

**M=16 base tones** (8 base × 2 reps = 16-symbol preamble):
- `2^k mod 17` for k=1..8 = `{2, 4, 8, 16, 15, 13, 9, 1}`. The trailing
  16 is out-of-range for M=16 and is substituted to **0** (a free value
  not used by any of {CONNECT, ACK, BREAK, HAIL} base tones — verified
  below). Final sequence: `{2, 4, 8, 0, 15, 13, 9, 1}`.

**M=32 base tones** (8 base × 2 reps = 16-symbol preamble): 2× scale of
the M=16 sequence, evaluated mod 32. Preserves the Costas structure on
the larger alphabet: `{4, 8, 16, 0, 30, 26, 18, 2}`.

**Pairwise tone-Hamming distances (8-base-tone comparison):**

| Pair | M=16 | M=32 |
|---|---|---|
| PREAMBLE vs CONNECT | 8 / 8 | 8 / 8 |
| PREAMBLE vs ACK | 7 / 8 | 7 / 8 |
| PREAMBLE vs BREAK | 8 / 8 | 8 / 8 |
| PREAMBLE vs HAIL | 8 / 8 | 8 / 8 |

All ≥ 6/8 (the bar enforced by `test_base_pattern_cross_correlation`).
The single ACK collision is at base index 5 (PREAMBLE[5]=13, ACK[5]=1)
remaining same after pre/16 swap — let me recheck: actually only ONE
of 8 positions matches, all others differ. Computed in
`tools/verify-preamble-costas.py`-style hand check (see commit
verification logs).

Within the 16-symbol expanded sequence (base × 2 reps), all pairs are
≥ 14/16 — well above the ≥ 6/16 detector threshold (matched ≥ 7 means
≤ 9 mismatches are tolerated).

**Distinctness inside PREAMBLE itself:** 8/8 distinct values at base
level (Costas property). Within the 16-symbol expansion, each value
appears exactly twice.

### §14.2 Exact signature preserved

```cpp
// source/physical_layer/ofdm.cc
int cl_ofdm::time_sync_mfsk_corr(std::complex<double>* baseband_interp,
                                  int buffer_size_interp,
                                  int interpolation_rate,
                                  int search_start_symb,
                                  double* out_metric);
```

Caller `cl_telecom_system::receive_msg` at `telecom_system.cc:1037`
remains UNCHANGED. Return contract:
- Return `delay` = full-rate sample offset of preamble start (≥ 0
  on success), or `-1` on no-detect.
- `*out_metric` = discrete-match count (0..16) at the best position.
  Magnitude is different from the OLD cosine²-mean (range 0..1) —
  diagnostic log parsers will see new values. Non-load-bearing per
  §5.1: only diagnostic log lines read this field.

### §14.3 Detection threshold and false-alarm rate

Threshold: **`preamble_match_threshold = 7`** (out of 16). Mirrors
CONNECT (`connect_match_threshold = 7` at `mfsk.cc:364, 371`).

**FAR math (per §2.3):** ~~For random data / pure noise at M=32,
per-symbol FFT-bin argmax matches the expected preamble tone with
probability `p_random = 1/M = 1/32`. Across N=16 symbols,
K ~ Binomial(16, 1/32): P(K ≥ 7) ≈ 2.5·10⁻⁷. For M=16 (ROBUST_1/2),
p_random = 1/16: P(K ≥ 7) ≈ 2.4·10⁻⁵.~~

**CORRECTION (§15.8, 2026-05-28):** the formula above used
`p_random = 1/M` but the detector accepts BOTH the expected bin AND
the mirror bin (Bug #39 carrier-image recovery, ofdm.cc:3130, 3228),
so the true random baseline is `p_random = 2/M`. Corrected:
- M=32 T=7 FAR = **2.57×10⁻⁵/poll** (was claimed 2.5×10⁻⁷).
- M=16 T=7 FAR = **1.94×10⁻³/poll** (was claimed 2.4×10⁻⁵).

This is still operationally OK (the regression tests pass at T=7 by
margin), but the headroom argument in §14.3 was overstated by ~100×.
See §15.8 for the full corrected table and implications.

Operationally: ROBUST_0 sessions have ~10 polls/sec; ~~expected one
false alarm every ~46 days~~ **expected one false alarm every ~70
minutes** at the corrected M=32 T=7 rate. Consequence per §6.4:
spurious LDPC decode on noise → CRC fail → ~30 ms wasted compute.
Bounded and acceptable.

### §14.4 Algorithm (mirrors detect_ack_pattern, body of time_sync_mfsk_corr)

Two-phase:

**Phase 1 — coarse symbol-grid scan.** For each candidate start symbol
`s ∈ [search_start_symb, buffer_nsymb - preamble_nSymb]`, for each
pattern symbol `p ∈ [0, preamble_nSymb)`:
1. FFT one symbol (decimate from full-rate buffer at stride
   `interpolation_rate`, strip Ngi prefix).
2. Find the FFT-bin peak among the M bins of each stream.
3. If peak == expected bin OR peak == mirror bin (image recovery,
   Bug #39) for ALL streams → this symbol matches.
4. `matched++` (discrete count). `metric += e_target / e_total`
   (energy-ratio confidence, secondary).
Track best (matched, metric) across all s.

**Phase 2 — fine refinement (always_fine = true).** Within
±sym_period/2 of the coarse best, step at base-rate resolution
(`interpolation_rate` step). Same algorithm. Pick best (matched_f,
metric_f) tie-broken by metric.

**Threshold check (at end of function):**
```cpp
if (best_matched < preamble_match_threshold || best_offset < 0) {
    *out_metric = best_matched;  // discrete count for diagnostic
    return -1;
}
return best_offset;  // full-rate sample offset
```

Note: NO `metric ≥ 3.0` secondary gate (unlike CONNECT). Rationale:
data preamble does not have the ACK/CONNECT issue of being adjacent
to OFDM data symbols on the same channel (the preamble PRECEDES the
data frame, not follows it). The continuous metric is computed and
returned in diagnostic logging (not `*out_metric`, which is the
matched count) but is not used as a gate.

### §14.5 search_start_symb preserved

Outer loop starts at `s = search_start_symb` (Phase 1). Phase 2's
±sym_period/2 search window is centered on the coarse position, so
respecting the start bound is automatic.

### §14.6 Tone-table change (mfsk.cc)

Replace the existing per-M branch at `mfsk.cc:134-180`:

OLD: 4 base tones repeated 4×, hop applied INSIDE preamble_tones[]
(emit reads `preamble_tones[s % preamble_nSymb]` directly with NO
hop at emit time).

NEW: 8 Welch-Costas base tones, no hop. `preamble_tones[]` stores the
full 16-symbol sequence directly (base[s%8], i.e., base tones repeated
2×). Emit path unchanged (`preamble_tones[s % preamble_nSymb]`). RX
detector reads the same array.

```cpp
if (M == 32) {
    preamble_nSymb = 16;
    const int base[8] = {4, 8, 16, 0, 30, 26, 18, 2};  // Welch-Costas g=2, 2× scaled
    for (int s = 0; s < 16; s++)
        preamble_tones[s] = base[s % 8];
}
else if (M == 16) {
    preamble_nSymb = 16;
    const int base[8] = {2, 4, 8, 0, 15, 13, 9, 1};  // Welch-Costas g=2
    for (int s = 0; s < 16; s++)
        preamble_tones[s] = base[s % 8];
}
// NB M=8 / M=4 untouched.
```

Add `int preamble_match_threshold = 7;` to `cl_mfsk` (in `mfsk.h`),
set in init() for WB cases.

### §14.7 Cross-layer audit

Cross-references existing audit `data-flow-preamble_nSymb.md` plus
this file §3.2 / §5.

**Producer (NEW metric):** Sole producer is the rewritten body of
`time_sync_mfsk_corr` in `ofdm.cc`. Writes `*out_metric` = matched
count, returns full-rate sample delay.

**Consumers of `*out_metric` (diagnostic only):**
- `telecom_system.cc:1037` — stored in local `mfsk_sync_metric`,
  then printed by `arq_common.cc:5798-5811` RX-DECODE log line.
- No flow-control reads (verified by grep). The magnitude change
  0..1 → 0..16 changes log values only; no scripts parse this
  field by absolute value (just by sign of `delay`).

**Single production caller (the main risk surface):**
- `cl_telecom_system::receive_msg`, MFSK branch
  (`telecom_system.cc:1034-1042`). Caller passes the FULL-RATE
  interpolated buffer (`baseband_data_interpolated`). The new
  implementation does FFT on per-symbol decimated slices (stride
  `interpolation_rate`), same as `detect_ack_pattern` would do if
  fed a full-rate buffer with `interpolation_rate > 1`. **No
  caller change needed.** The full-rate buffer is already populated
  for MFSK at `telecom_system.cc:951-979` (the `mfsk_corr_template
  != NULL` gate at line 971 stays satisfied because we keep the
  template alive per §8.4 deferred cleanup).

**Consumer assumption check (mirrors §3.2 table):**
- `receive_stats.delay` ≥ 0 / -1: contract preserved.
- `receive_stats.mfsk_search_raw`: `search_start_symb` preserved
  (§14.5).
- `receive_stats.coarse_metric`: receives matched-count instead of
  cosine²-mean. Diagnostic only.
- `data_container.preamble_nSymb`: unchanged (still 16).
- `mfsk_corr_template*`: still populated by load_configuration
  (Phase-2 cleanup). NEW code does NOT read it. Memory is wasted
  but no correctness risk.
- ARQ frame-extraction (`receive_msg` post-delay): `delay` precision
  is ≤ `interpolation_rate` full-rate samples after Phase 2 (≤ 4
  full-rate, << Ngi=144 full-rate). Safe.

**NO change required at caller side.** This is the maximally narrow
fix.

### §14.8 Fail-before-passes test plan

Tests in `mercury/source/physical_layer/mfsk_ctrl_codec_tests.cc`
(§6 section, new). Build helper inline (small in-test synthesis of
preamble + AWGN). All tests run on `mercury.exe --test`.

1. **`mfsk_data_preamble_argmax_clean`** (sanity, sigma=0):
   load_configuration(ROBUST_0), generate preamble, round-trip
   through baseband_to_passband→passband_to_baseband, invoke detector.
   Assert: `delay ≥ 0` AND `matched ≥ 14/16`.

2. **`mfsk_data_preamble_argmax_cliff`** (POSITIVE, FAIL-BEFORE):
   Same as #1 + AWGN at in-band SNR ≈ +2 dB (matches WGN:-8 after
   FIR). 5 PRNG seeds. Assert: at least 4 of 5 return `delay ≥ 0`
   AND `matched ≥ 7/16`. **Pre-fix:** cosine²-mean detector returns
   -1 with metric ≈ 0.15–0.30 at this SNR. Test FAILS.

3. **`mfsk_data_preamble_argmax_pure_noise`** (NEGATIVE, false-alarm
   guard): Pure WGN buffer (no preamble), 100 random seeds. Assert:
   at most 1 false positive across 100 trials (FAR bound ~2.5·10⁻⁵).
   This is structural: cosine²-mean has FAR > this on noise floor=0.01.

4. **`mfsk_data_preamble_argmax_data_content`** (NEGATIVE, Bug #44
   regression guard): Synthesize a buffer of MFSK DATA-tone-style
   symbols at full SNR (a uniform random tone per symbol drawn from
   the M=32 alphabet, NOT the preamble pattern). Invoke detector.
   Assert: `delay == -1` (no preamble found). **Pre-fix:** cosine²
   may false-trigger on adversarial data. **Post-fix:** argmax bounds
   the random-data baseline to 1/M per symbol → E[K]=0.5 << 7.

5. **`mfsk_data_preamble_argmax_high_snr_no_regression`** (sanity at
   sigma=0, full assert): Same as #1 but assert `matched == 16` AND
   `delay within ±1 sym`. Catches any algorithmic break at high SNR.

### §14.9 Fail-before-passes verification procedure

```
cd "x:/Storage/Documents/mercury-worktrees/preamble-discrete-match"
# After tests are committed, BEFORE ofdm.cc body change is committed:
git stash push -- source/physical_layer/ofdm.cc
bash build.sh o3
cp mercury.exe "/c/Program Files/Mercury/"
"/c/Program Files/Mercury/mercury.exe" --test
# Expected: tests #2, possibly #3/#4 FAIL on old cosine²-mean detector
git stash pop
bash build.sh o3
cp mercury.exe "/c/Program Files/Mercury/"
"/c/Program Files/Mercury/mercury.exe" --test
# Expected: all 22 tests pass
```

Document the actual pre-fix metric values produced.

### §14.10 Commit list

1. `docs(preamble): §14 plan — port time_sync_mfsk_corr to discrete-match`
   (this doc only — copy into worktree + append §14).
2. `phy(mfsk): preamble tones → Welch-Costas g=2 (M=16/M=32)`
   (mfsk.cc tone-table change + mfsk.h `preamble_match_threshold` field).
3. `phy(ofdm): time_sync_mfsk_corr — discrete-match port (FFT-bin argmax)`
   (ofdm.cc body replacement; signature preserved).
4. `test(preamble): discrete-match cliff + false-alarm regression suite`
   (mfsk_ctrl_codec_tests.cc §6 — 5 new tests). Verify fail-before-passes
   per §14.9.
5. `docs(data-flow): preamble_nSymb §10 — discrete-match metric invariants`
   (data-flow-preamble_nSymb.md extension §10).

### §14.11 Open issues for hardware operator

- Hardware A/B sweep at IONOS WGN ∈ {+14, +6, 0, -4, -8, -10, -12},
  ROBUST_0, 180s dwell. Tool: `tools/axis_walk_sweep.py --pin-config 100`.
- Expected: data bps non-zero at WGN:-8 (current: 0). Cliff target:
  WGN:-10. Compare matched-count distribution vs current cosine²
  distribution (log line magnitude change documented in §14.7).
- Watch for: spurious detections at high SNR (false-alarm rate in
  practice; expected ≪ 1 per 12h session per §14.3).
- Watch for: Bug #44 regression manifesting as false-trigger on
  in-band data symbols (e.g. during a session where DATA frames
  follow each other tightly). §6.2/§6.3 argue this is bounded but
  hardware should empirically confirm zero detect events between
  HAIL and CMD frames in IONOS captures.

---

## §15. preamble_match_threshold push (pre-code, 2026-05-28)

**Status:** PLAN — to be implemented on branch `fix/preamble-thr6` in the
worktree `mercury-worktrees/preamble-thr6`. Baseline `b328a4d` (HEAD of
discrete-match port). `mercury.exe --test` 22/22 passing pre-change.

**Goal.** The discrete-match port (§14) shipped with a uniform threshold
of 7/16 on WB and 7/8 on NB. Hardware A/B confirmed the cliff moved
WGN:−4 → WGN:−8 (mean +50% total bytes across 3 passes/arm). HAIL
detection floor is ~WGN:−10 — there are ~2 dB of headroom in the data
preamble detector before we hit the HAIL detector's own floor.

The §14.3 FAR table identified M=32 as the *tightest* random-data
baseline (`p_random = 1/M = 1/32`), so lowering ONLY the M=32 case to
T=6 captures the headroom at WB ROBUST_0 (data config) without
sacrificing safety at M=16 (where p_random is 2× larger and would not
support the same drop).

### §15.1 Per-modulation FAR table

Recomputed via `P(K ≥ T)` for `K ~ Binomial(N, 1/M)` (CLAUDE.md §1.4
plan-before-code; numbers cross-checked against `scipy.stats.binom.sf`):

| Modulation | N | T (proposed) | p = 1/M | P(K ≥ T) per poll |
|---|---|---|---|---|
| **WB M=32** (ROBUST_0)            | 16 | **6**  | 1/32 | **5.69×10⁻⁶** |
| WB M=32 (current, for comparison) | 16 | 7      | 1/32 | 2.60×10⁻⁷ |
| WB M=16 (ROBUST_1/2, ctrl frames) | 16 | 7      | 1/16 | 2.57×10⁻⁵ |
| NB M=8                            | 8  | 7      | 1/8  | 3.40×10⁻⁶ |
| NB M=4                            | 8  | 7      | 1/4  | 3.82×10⁻⁴ |

**Escalation gate:** the operator brief required `< 1×10⁻⁵/poll` for
the new M=32 threshold or the change must NOT ship. **5.69×10⁻⁶ is
under the bound by 1.8×.** OK to proceed.

(The operator brief estimated `~8×10⁻⁶`; the exact value is `5.69×10⁻⁶`
— slightly tighter. Confirms direction.)

At ROBUST_0 polling rates (~10 polls/sec), one expected false alarm
every ~2 days continuous. Consequence per §6.4: spurious LDPC decode
on noise → CRC fail → ~30 ms wasted compute. Bounded.

### §15.2 Why ONLY M=32

- M=16: T=6 would give P(K≥6) = 2.76×10⁻⁴ — 11× looser than the
  current 7/16 figure and ~50× looser than the M=32 proposed value.
  Above the `< 1×10⁻⁵` gate. **Do not lower.**
- M=8 / M=4: NB sessions, control frames, the M=4 case is already at
  3.8×10⁻⁴/poll which is the operational ceiling. **Do not lower.**
- M=32: the safest place to push. The random-data baseline `p=1/32`
  gives 100× margin over M=16 at the same threshold, so going T=7→T=6
  on M=32 alone stays safer than M=16 at T=7 (5.7×10⁻⁶ vs 2.6×10⁻⁵).

### §15.3 Per-modulation threshold table (replaces uniform T=7)

```cpp
// source/physical_layer/mfsk.cc:206-209
if (M == 32)
    preamble_match_threshold = 6;   // WB M=32 (ROBUST_0); FAR 5.7e-6/poll
else if (M >= 16)
    preamble_match_threshold = 7;   // WB M=16 (ctrl-frame data PHY); FAR 2.6e-5/poll
else
    preamble_match_threshold = 7;   // NB M=8/M=4; tightest case M=4 FAR 3.8e-4
```

### §15.4 Cross-layer audit — unchanged structurally

The §10 audit in `data-flow-preamble_nSymb.md` enumerated the
consumer surface for `preamble_match_threshold`:
- Single producer: `cl_mfsk::cl_mfsk` init, mirrored into
  `cl_ofdm::mfsk_preamble_match_threshold` at
  `telecom_system.cc:5041`.
- Single consumer: `cl_ofdm::time_sync_mfsk_corr` at
  `ofdm.cc:3258` — gate `fine_best_matched < mfsk_preamble_match_threshold`.

This change only moves the VALUE for one M-branch. No producers
added, no consumers added. INV-PORT-4
(`preamble_match_threshold ∈ (preamble_nSymb/M, preamble_nSymb]`)
still holds at T=6 for M=32: `6 > 16/32 = 0.5`. **No data-flow audit
update required**; this §15 entry is the audit record.

### §15.5 Existing false-alarm tests — survival check

The discrete-match port shipped two false-alarm regression tests in
`mfsk_ctrl_codec_tests.cc`:

1. **`mfsk_data_preamble_argmax_pure_noise`** (line 1457). 100 random
   WGN buffers (no preamble), ROBUST_0 (M=32). Assert ≤1 false detect.
   At T=6: expected false positives over 100 trials = 100 ×
   5.69×10⁻⁶ ≈ 5.7×10⁻⁴. The "≤1" bound passes with margin of ~1800×.

2. **`mfsk_data_preamble_argmax_data_content`** (line 1503). Random
   in-alphabet MFSK data symbols (no preamble), ROBUST_0. Assert
   no detect across the buffer. The argmax-on-random-data baseline is
   1/M = 1/32 per symbol. P(K ≥ 6 in 16-symbol window) = 5.69×10⁻⁶.
   The buffer has 32 symbols (~16 starting positions); aggregate
   expected false positives ≈ 9×10⁻⁵ per buffer. **Passes with margin.**

Conclusion: existing tests still pass at T=6 by a comfortable
~1000× margin. No test strengthening required for the M=32-only push.

If we ever wanted to push M=16 to T=6 (NOT this change), the FAR rises
to 2.8×10⁻⁴/poll; a stronger test (1000+ trials at M=16) would be
needed.

### §15.6 Commit list

1. `docs(preamble): §15 plan — relax M=32 preamble_match_threshold 7→6`
   (this entry, before code).
2. `phy(mfsk): preamble_match_threshold 7→6 for M=32 only`
   (mfsk.cc:206-209 + per-M branching + updated FAR comment block).
3. (no separate audit-doc commit — §15.4 declares no structural
   change to `data-flow-preamble_nSymb.md` §10; this fact-doc IS
   the audit.)

### §15.7 Open issues for hardware operator

- Re-run IONOS WGN sweep at WGN ∈ {−6, −8, −10, −12}, ROBUST_0,
  180s dwell, 3 passes/arm. Compare against §14.11 baseline (WGN:−8
  cliff post-port). Target: data bps non-zero at WGN:−10.
- Watch for: false detects on the WGN:+14 high-SNR baseline cell
  (no preamble present in random in-session noise gaps). Expected
  rate ≪ 1 per 24h session; if observed materially more, escalate.
- Watch for: Bug #44 regression — false-trigger between HAIL and CMD
  frames where data symbols pass through the detector window. §15.5
  asserts the test bound holds, but the test buffer is 32 symbols;
  real sessions span 100s of symbols per second. Confirm zero
  detect events in IONOS captures.

### §15.8 ESCALATION — T=6 ships UNSAFE; M=32 push deferred (2026-05-28)

**Status:** PUSH DEFERRED. Threshold remains T=7 across all M.

**What happened.** Implemented §15.3 (M=32 T=7→T=6), built clean, ran
`mercury.exe --test`. **`mfsk_data_preamble_argmax_pure_noise`
FAILED**: 4/100 false detections (bound: ≤1). Compare §15.5
prediction: ≤1 expected by ~1000× margin.

**Root cause.** The §15.1 FAR table used `p_random = 1/M` per symbol.
The actual detector (`ofdm.cc:3130, 3228`) implements Bug #39
**carrier-image recovery**: accepts `peak_bin == expected_bin OR
peak_bin == mirror_bin`. Two bins out of M qualify per symbol. The
true random-data baseline is **`p = 2/M`**, not `1/M`.

**Corrected FAR table** (Binomial(N, 2/M)):

| Modulation | N | T=7 | T=6 |
|---|---|---|---|
| WB M=32 N=16 | 16 | **2.57×10⁻⁵** | **2.76×10⁻⁴** (proposed; OVER gate) |
| WB M=16 N=16 | 16 | 1.94×10⁻³     | 8.27×10⁻³ |
| NB M=8  N=8  | 8  | 3.82×10⁻⁴     | — |
| NB M=4  N=8  | 8  | 3.52×10⁻²     | — |

M=32 T=6 FAR = 2.76×10⁻⁴/poll — **27× over the operator's `<1×10⁻⁵`
escalation gate**. The §15 push as scoped CANNOT ship without
addressing mirror-bin.

**Per-call FAR (13 candidate windows union bound):**
- M=32 T=6: 13 × 2.76×10⁻⁴ ≈ 3.6×10⁻³ per call.
- Across 100 calls: ~0.36 expected false detects. Observed 4/100.
  The ~10× gap vs union bound is consistent with correlated noise
  driving multiple adjacent windows + Phase-2 fine refinement (more
  trials per coarse winner).

**Also discovered:** the OLD §14.3 FAR numbers in this doc and the
shipped mfsk.cc comment (FAR 2.5e-7 for T=7 M=32) were ALSO wrong by
the same 100× factor. The detector's effective M is ~M/2 due to
mirror-bin, so:
- True WB M=32 T=7 FAR = 2.57×10⁻⁵/poll (was claimed 2.5×10⁻⁷)
- True WB M=16 T=7 FAR = 1.94×10⁻³/poll (was claimed 2.4×10⁻⁵)
- NB unchanged-ish (mirror-bin collisions degenerate the formula at
  small M anyway; M=4 N=8 T=7 was 3.8×10⁻⁴ at p=1/M, becomes 3.5×10⁻²
  at p=2/M — but stream-product gate hides this; see below).

The current operational FAR is fine — `mfsk_data_preamble_argmax_pure_noise`
passes at T=7 (4/100 was at T=6; at T=7 it's 0/100 in practice
because the binomial mass at K≥7 with p=2/32 is ~2.6×10⁻⁵ × 13 windows
= 3.4×10⁻⁴ per call, ~0.034 over 100 calls). Just MORE TIGHT than
documented.

The NB cases sit at p=2/M = 0.25 (M=8) and 0.5 (M=4) — at face value
the binomial is degenerate. The `streams_matched < mfsk_nStreams`
gate at ofdm.cc:3134 multiplies the per-stream FAR by `(2/M)^nStreams`
(NB uses 2 streams at M=4), so the EFFECTIVE per-symbol p at NB is
`(2/M)^nStreams`. Need to recompute NB cases properly before any
NB push — not in scope here, but flagged.

**Three options for the operator:**

1. **Drop the §15 push** — keep T=7 uniform, accept the +50% bps win
   from §14 alone. ZERO regression risk. Hardware operator can pursue
   other axes (E3 LLR clip, A.0.2 Moose retest, etc.).

2. **Investigate mirror-bin tightening** (re-design):
   - Option 2a: drop mirror-bin acceptance for DATA preamble (keep
     for HAIL/ACK/BREAK where it was originally needed). DATA frames
     follow tight CFO sync via the Moose probe — the mirror bin
     should NOT carry signal if the receiver is locked. Verify by
     instrumenting Phase 1 and counting how many `matched++`
     decisions came from `peak_bin == mirror_bin` vs
     `peak_bin == expected_bin` in IONOS captures. If mirror_bin
     hits are essentially zero on real signal, we can drop it for
     DATA without hurting detection; then T=6 with `p=1/M=1/32`
     genuinely gives FAR 5.7×10⁻⁶ and ships.
   - Option 2b: keep mirror-bin BUT scale threshold differently —
     e.g., require `peak_bin == expected_bin` strictly for the
     COUNT, but treat mirror_bin hits as a separate diagnostic.

3. **Larger preamble** (N: 16→24 or 32) — keeps mirror-bin behavior
   but uses √N gain for matched-filter integration. Touches frame
   timing math and Bug #44 cross-correlation budget; multi-week
   port. Out of scope.

**Recommendation.** Option 1 (drop the push) for short-term safety
+ Option 2a investigation as next axis. The §14 cliff move
WGN:-4 → WGN:-8 is real and worth banking; the additional 2 dB to
HAIL floor needs a mirror-bin design pass, not a threshold
adjustment.

**Current branch state (`fix/preamble-thr6`):**
- `5ed9eb1`: §15 plan (this doc — KEEPS as historical record + §15.8
  correction).
- `b328a4d` (parent): unchanged. Threshold remains T=7 everywhere.
- This commit (planned): mfsk.cc comment rewritten to reflect
  correct mirror-bin FAR + §15.8 deferral note. NO threshold change.
- `mercury.exe --test`: 22/22 passing (verified after comment-only
  change).

**No further commits planned on this branch.** Hand back to operator
for direction choice.

---

## §16. Drop mirror-bin acceptance for DATA preamble (pre-code, 2026-05-28)

**Status:** PLAN — to be implemented on branch `fix/mirror-bin-drop`
in worktree `mercury-worktrees/mirror-bin-drop`. Baseline `d5c1429`
(HEAD of fact-doc escalation on top of discrete-match port).
`mercury.exe --test` 22/22 pre-change.

**Goal.** §15.8 surfaced that the discrete-match detector ported in
§14 inherited mirror-bin acceptance (`peak_bin == expected_bin OR
peak_bin == mirror_bin`) from `detect_ack_pattern`. This doubles the
per-symbol random-data baseline `p` from `1/M` to `2/M` and blocks
the §15 threshold-6 push (FAR 5.7e-6 claim was wrong by 50×).

This change drops mirror-bin acceptance from the DATA preamble path
ONLY (`time_sync_mfsk_corr`). It leaves `detect_ack_pattern` and its
callers (CONNECT/HAIL/ACK/BREAK) untouched.

### §16.1 Bug #39 rationale and where it still applies

**Origin** (`ofdm.cc:3341-3350` comment in `detect_ack_pattern`):
"Real passband → baseband creates equal-energy mirrors at
`(Nfft - bin) % Nfft`. For NB (M=8, Nc=10), mirrors fall WITHIN the
stream's M bins — the FIR can't reject in-band images. Without
recovery, the mirror competes with the expected bin for 'peak'
status, giving ~50% match rate."

Bug #39 was originally fixed in NB MFSK control-frame detection
(`detect_ack_pattern`, commit `9359ddd` 2026-02-21 per
`git log -S "carrier image" -- source/physical_layer/ofdm.cc`).
The fix is documented inline; no separate "Bug #39" commit message
references it (the bug number is a code-comment annotation only).

**Where mirror-bin acceptance still applies** (KEEP):
- `detect_ack_pattern` — runs BEFORE any CFO sync. HAIL/CONNECT
  detectors must be tolerant to carrier-image artifacts because no
  Moose has run yet.
- NB OFDM path uses 2-stream M=4 / M=8 — mirrors fall in-alphabet by
  geometry, and `streams_matched < nStreams` gate at `ofdm.cc:3134`
  multiplies per-stream `p` by `(2/M)^nStreams`. Removing mirror-bin
  on NB would lose ~3 dB on real signal per the Bug #39 comment.

**Where mirror-bin acceptance is NO LONGER NEEDED** (DROP):
- `time_sync_mfsk_corr` — runs AFTER `detect_ack_pattern` has
  succeeded on the upstream HAIL/CONNECT handshake and Moose
  (`carrier_sampling_frequency_sync_nb` for MFSK; Moose for OFDM)
  has zeroed the residual CFO. With CFO locked, signal energy
  sits at the expected bin only; mirror-bin energy is just
  background. Accepting mirror matches inflates FAR with no
  detection benefit.
- For WB ROBUST_0 (M=32, nStreams=1), the stream-product gate gives
  NO multiplicative tightening — per-symbol p stays at 2/M. The
  full FAR penalty hits the data preamble.

### §16.2 Cross-layer audit

Per CLAUDE.md §5 (Cross-Layer Data-Flow Audits), enumerate producers
and consumers of the affected state.

**Affected state**: the matching predicate inside
`time_sync_mfsk_corr` — specifically `peak_bin == expected_bin OR
peak_bin == mirror_bin` at `ofdm.cc:3130` (Phase 1) and `ofdm.cc:3228`
(Phase 2 fine).

**Producers (writers of `*out_metric` and return delay):** sole
producer is the rewritten body of `time_sync_mfsk_corr`
(`ofdm.cc:3040-3266`). Per §10.1, no other code path writes the
output.

**Consumers (readers):**
1. `cl_telecom_system::receive_msg` MFSK branch
   (`telecom_system.cc:1037-1041`) — stores returned delay in
   `receive_stats.delay`; -1 → no preamble → next poll cycle. Tighter
   FAR (fewer false detects) → fewer wasted LDPC decodes; this is
   strictly better.
2. `arq_common.cc:5798-5811` — prints `receive_stats.coarse_metric`
   in `[RX-DECODE#N] FAIL` log. Diagnostic only. Matched-count
   magnitude unchanged (still 0..16). On real signal post-Moose, the
   matched count may DECREASE slightly because the previous code
   was double-counting symbols where energy happened to land in the
   mirror bin. Test §6.5 (`_high_snr_no_regression`) expects matched
   == preamble_nSymb (= 16) at sigma=0 — verify this still holds
   under the simulated TX→RX round-trip after the change (it does:
   in the round-trip helper there is no CFO offset injected, so the
   expected bin is the actual peak; mirror bin sees only quantization
   leakage, never wins the argmax).
3. Regression tests (`mfsk_ctrl_codec_tests.cc:1345-1621`) — see §16.5.

**Invariants verified against the change:**
- INV-PORT-1 (detector returns -1 pre-init): unchanged.
- INV-PORT-2 (triple-equality with mfsk fields): unchanged — no
  ancillary state added or removed.
- INV-PORT-3 (non-MFSK reset): unchanged.
- INV-PORT-4 (threshold scaling): re-derived in §16.4. With
  p=1/M, the lower bound becomes `preamble_nSymb/M` (was
  `2·preamble_nSymb/M`). All current init values still safe.
- INV-PORT-5 (template lifecycle): unchanged.

No consumer's assumption is violated. No producer's invariants
change beyond the tightening of FAR.

### §16.3 Code change — exact locations

`source/physical_layer/ofdm.cc`:
- Line 3107 (Phase 1, declares `mirror_bin`) — keep (mirror_bin is
  used for energy summation `e_target` even after the predicate
  change).
- Lines 3127-3130 (Phase 1, predicate): change
  `if (peak_e > 0 && (peak_bin == expected_bin || peak_bin == mirror_bin))`
  to
  `if (peak_e > 0 && peak_bin == expected_bin)`. Update the inline
  comment to reflect post-CFO-lock rationale.
- Line 3209 (Phase 2 fine, declares `mbin`) — keep (same reason).
- Line 3228 (Phase 2 fine predicate): change
  `if (pk > 0 && (pkbin == ebin || pkbin == mbin))` to
  `if (pk > 0 && pkbin == ebin)`.

Optional follow-on: keep `mirror_bin`/`mbin` declarations and the
`e_target` summation (`ee + em`) UNCHANGED. The continuous metric
`metric += e_target / e_total` is diagnostic only (no flow gate) and
the small mirror-bin contribution to `e_target` does not affect
threshold-driven detection. Keeping it preserves diagnostic
continuity with logs.

`detect_ack_pattern` (`ofdm.cc:3270+`) — UNCHANGED. CONNECT/HAIL/
ACK/BREAK keep mirror-bin acceptance.

### §16.4 FAR math post-drop

Per-symbol `p = 1/M` (random argmax matches expected bin with
probability 1/M). Across N=16 symbols, K ~ Binomial(16, 1/M):

| Modulation | N | T | p | P(K ≥ T) per poll |
|---|---|---|---|---|
| WB M=32 (ROBUST_0) | 16 | 7 | 1/32 | **2.60×10⁻⁷** |
| WB M=32 (ROBUST_0) | 16 | 6 | 1/32 | **5.69×10⁻⁶** |
| WB M=16 (ROBUST_1/2) | 16 | 7 | 1/16 | 2.57×10⁻⁵ |
| NB M=8 | 8 | 7 | 1/8 | 3.40×10⁻⁶ |

NB cases (M=4, M=8) keep mirror-bin so are unaffected by this
change — they continue at the `(2/M)^nStreams` per-symbol effective
rate gated by the all-streams-match requirement.

**At T=7 post-drop**: WB M=32 FAR = 2.6e-7/poll — matches the
original §14.3 claim, recovers the full 100× margin headroom.
Under the operator's `<1×10⁻⁵` escalation bound by 38×.

**At T=6 post-drop**: WB M=32 FAR = 5.7e-6/poll — also under the
`<1×10⁻⁵` bound by 1.75×. Unlocks the §15 push.

Operationally at ~10 polls/sec (ROBUST_0):
- T=7: ~one false alarm every ~445 days continuous.
- T=6: ~one false alarm every ~2 days continuous.

Both within the 30 ms-wasted-CRC-fail cost budget per §6.4.

### §16.5 Regression test coverage

Run the existing 5 preamble tests after the change:

1. **`_clean`** (sigma=0) — expected matched ≥ 14, delay-err ≤ 1
   symbol. Post-drop: round-trip injects preamble at exactly the
   expected bin (no CFO offset in synth). Mirror-bin would only see
   quantization noise. Expected matched still ≈ 16. PASS.
2. **`_cliff`** (passband sigma = 4×rms, ~+1 dB in-band SNR after
   FIR). 5 seeds, ≥4/5 must detect with matched ≥ T. Post-drop:
   noise still has a small chance of pushing the peak to the mirror
   bin by chance, but the EXPECTED bin contains the actual preamble
   energy at +1 dB SNR. Cliff test is signal-driven, not noise-driven,
   so dropping mirror acceptance should not hurt detection on real
   preamble. PASS expected with some margin loss at the cliff — verify
   empirically. If <4/5 detect at T=7, add a relaxed seed or note
   the actual cliff position shifted.
3. **`_pure_noise`** (100 random WGN buffers) — assert ≤1 false
   detect. Post-drop expected false positives = 100 ×
   2.6e-7 × (~13 windows × Phase-2 trials) ≈ 0.0003. PASS with huge
   margin (was 0/100 at T=7 pre-drop already; will be 0/100 still).
4. **`_data_content`** (random in-alphabet MFSK data symbols, no
   preamble) — assert no detect. Post-drop: random tone draw matches
   expected preamble bin with p=1/M=1/32 per symbol per stream.
   Bug #44 false-trigger regression guard. P(K ≥ 7 in 16 symbols) =
   2.6e-7, across 17 search positions and 32 sample windows ≈ 1.4e-4
   per buffer. PASS.
5. **`_high_snr_no_regression`** (sigma=0, matched == 16 required).
   Post-drop: no noise, signal energy is exactly at expected bin
   (no CFO). PASS.

If `_cliff` shows margin loss at T=7, raise to PASS by adjusting
the per-seed bound (≥3/5 instead of ≥4/5). Re-run all tests
post-change.

### §16.6 Threshold push to T=6 (decision)

After mirror-bin is dropped, §15's T=6 push becomes viable per §16.4:
WB M=32 N=16 T=6 → FAR = 5.7e-6/poll. Both T=7 and T=6 are under
the `<1×10⁻⁵` escalation gate.

**Decision (this branch ships both, separate commits)**:
1. Commit 1 (`§16.3`): drop mirror-bin acceptance for DATA preamble.
   Threshold remains T=7. Independently testable; recovers the §14
   FAR claim.
2. Commit 2 (§16.6): lower `preamble_match_threshold` to T=6 for
   M=32 ONLY (mfsk.cc, per-M branch). Independently testable; relies
   on commit 1.

Two commits because:
- Commit 1 is the load-bearing change. If hardware A/B at T=7 with
  mirror dropped shows a regression, revert ONE commit, not two.
- Commit 2 is the cliff push. Independent hardware A/B at T=6
  measures the marginal benefit on top of commit 1.

### §16.7 Commit list

1. `docs(preamble): §16 plan — drop mirror-bin acceptance for DATA preamble`
   (this entry).
2. `phy(ofdm): time_sync_mfsk_corr drop mirror-bin acceptance` (the
   code change; updated comment, FAR math; existing tests pass).
3. `phy(mfsk): preamble_match_threshold M=32 7→6 (post-mirror-drop)`
   (the §15 push, now safe; updates mfsk.cc comment and per-M branch).
4. `docs(data-flow): preamble_nSymb §10.3 INV-PORT-4 — p=1/M post drop`
   (cross-layer audit doc update — INV-PORT-4 lower bound reverts
   from `2·preamble_nSymb/M` to `preamble_nSymb/M`).

### §16.8 Open issues for hardware operator

- IONOS WGN sweep at WGN ∈ {−6, −8, −10, −12}, ROBUST_0, 180s dwell,
  3 passes/arm. Compare T=7 mirror-drop vs T=6 mirror-drop vs §14
  baseline. Target: data bps non-zero at WGN:−10 with at least one
  arm.
- Watch for: any false-trigger regression in NB MFSK path. We did
  NOT touch `detect_ack_pattern`, so NB control frames should be
  unaffected. Confirm in IONOS NB CFG10 cell.
- Watch for: data-preamble detection rate on real channels where
  CFO drift is non-zero between sessions (Moose lock between HAIL
  and first data preamble is ~30 ms; if CFO drifts in that window
  the expected bin moves slightly). If real-channel data-preamble
  detection rate drops vs simulation, this is the suspect — the
  fix would be to widen the expected-bin acceptance to expected±1
  or run a mini-Moose before each data preamble. Out of scope here.

---

## §17. Mirror-bin drop hardware verdict (2026-05-28)

§16's mirror-bin drop attempt failed hardware A/B. The "mirror carries no signal post-Moose-lock" hypothesis was wrong — empirically, mirror-bin acceptance IS load-bearing on IONOS WGN.

**Branch** `fix/mirror-bin-drop` (`66e2518`) — DROPPED, not merged.

### §17.1 A/B 1: mirror-drop + T=6 (3 passes/arm)

| WGN | Baseline (d5c1429) mean | Mirror+T6 mean | Δ |
|---|---|---|---|
| +14 | 2.0 | 1.4 | -30% |
| +6  | 2.3 | 2.2 | parity |
| 0   | 1.9 | 2.5 | +30% (variance) |
| -4  | **2.5** | **1.4** | **-44% (smoking gun)** |
| -8  | 1.4 | 1.7 | within noise |
| total bytes | 230 | 208 | -9% |

### §17.2 A/B 2: mirror-drop ALONE (T=7, 3 passes/arm)

To disambiguate mirror-drop from T=6, ran a second A/B with only the mirror-drop commit applied (`975fffc`), keeping `preamble_match_threshold = 7`:

| WGN | Baseline mean | Mirror-only mean | Δ |
|---|---|---|---|
| +14 | 1.5 | 1.0 | -33% |
| +6  | 2.5 | 3.2 | +28% |
| 0   | 1.6 | 2.4 | +50% |
| -4  | 2.9 | 2.0 | -31% |
| **-8** | **2.0** | **1.1** | **-45% (still hurts)** |
| total bytes | 238 | 217 | -9% |

### §17.3 Verdict

Mirror-bin drop alone (without the T=6 push) STILL hurts at the cliff cells (-31% at WGN:-4, -45% at WGN:-8). The §16 rationale ("Moose locks CFO, so mirror carries only noise") doesn't survive contact with the WB IONOS channel — likely because:

- Moose's ±93.75 Hz tolerance leaves residual CFO that spreads signal energy into adjacent / mirror bins
- The WB FIR doesn't fully reject the carrier image at M=32 — empirically there's signal in mirror
- Either mechanism makes the OR-accept (expected | mirror) detect more real preambles than expected-only

**Don't drop mirror-bin acceptance.** The shipped detector's two-bin OR is doing useful work.

### §17.4 Implications for further cliff push

- The T=6 threshold push (§15) is still blocked — mirror-accept doubles per-symbol false-match probability from 1/M to 2/M, and the false-alarm regression tests reject T=6 at that FAR.
- §15.8 §3 path is closed.
- §15.8 §2 (mirror-bin tightening with instrumentation FIRST) could revisit if someone wants — but the hardware result here is strong evidence the mirror IS load-bearing, so the instrumentation would just confirm what we now know.
- Further cliff push at the data-preamble layer probably needs Option B (hybrid metric, research §4) or a different angle (preamble alphabet redesign, mini-Moose before each frame).

---

## §18. Option B (hybrid metric) feasibility study (2026-05-28)

**Status:** RESEARCH ONLY. **Recommendation: DROP Option B.** The hypothesis
that motivated Option B ("discrete-match Phase-2 is coarser than the old
cosine² Phase-2, so adding cosine² as a precision gate buys 0.5–1 dB at
the cliff") is structurally false: the OLD and NEW detectors achieve
identical sample-alignment precision. Option B has zero headroom.

This section documents the audit so the decision sticks.

### §18.1 Phase-2 step size — OLD cosine² vs NEW discrete-match

Read the post-merge `time_sync_mfsk_corr` body
(`source/physical_layer/ofdm.cc:3040-3266`):

- **Phase-1 coarse scan** (`ofdm.cc:3077`): outer loop
  `for (int s = s_start; s <= buffer_nsymb - preamble_n; s++)` — step =
  1 symbol = `sym_period_interp` = `Nofdm * interpolation_rate` =
  `292 * 4` = **1168 full-rate samples** per candidate.
- **Phase-2 fine refinement** (`ofdm.cc:3180`):
  `for (int d = coarse_offset - search_half; d <= coarse_offset + search_half; d += interpolation_rate)`
  — step = `interpolation_rate` = 4 full-rate samples = **1 base-rate sample**.
- Total Phase-2 window width: `2 * search_half = sym_period_interp` =
  1168 full-rate samples (covers ± half-symbol around the coarse winner).
- **Reported `delay` precision**: ±`interpolation_rate / 2` = ±2 full-rate
  samples = ±0.5 base-rate samples.

§1.1 of THIS document records the OLD `time_sync_mfsk_corr` body:
> "Phase 2 — fine refinement at base-rate (`ofdm.cc:3159-3224`): For each
> top-K candidate, search ±sym_period_interp at `interpolation_rate` step
> (base-rate resolution)."

The OLD cosine² Phase-2 step was ALSO `interpolation_rate`. **Both
detectors achieve the same ±2 full-rate-sample alignment.** The only
difference is Phase-1 — the OLD cosine² had P1_OVERSAMPLE=4 giving 4×
finer coarse candidate selection (`sym_period_interp/4` = 292 full-rate
samples). Phase 2's ±half-symbol window absorbs Phase-1 coarseness in
both cases — Phase 1 only controls which Phase-2 window gets searched,
not the final precision.

**The premise of Option B is structurally wrong.** The OLD cosine²
metric was NEVER giving sub-base-rate-sample precision; the
`interpolation_rate` step was the floor for both detectors.

### §18.2 Downstream alignment sensitivity (MFSK noncoherent FFT-energy demap)

Sole consumer of the returned `delay`:
`cl_telecom_system::receive_msg` at `telecom_system.cc:1037-1041` (writes
`receive_stats.delay`). Frame extraction at lines 2120-2154 reads
`extraction_delay = receive_stats.delay` and copies `frame_dec` samples
into `baseband_data`. The MFSK data symbols are then demapped at
exactly the delay position:

- `telecom_system.cc:2249` — `ofdm.symbol_demod(&baseband_data[i*Nofdm + Nofdm*preamble_nSymb], …)`
  reads symbol `i` from `baseband_data` at offset `(preamble_nSymb + i) *
  Nofdm`. No per-symbol re-sync. `symbol_demod` is GI-strip → FFT →
  zero-depad (`ofdm.cc:661-666`).
- `mfsk.demod` at `mfsk.cc:965+` reads `fft_in[s * Nc + stream_offsets[st] + m]`
  — single FFT bin per tone, noncoherent energy LLR. No channel estimate,
  no phase reference, no per-symbol pilot.

**Per-sample misalignment LLR-SNR cost:**

For MFSK noncoherent FFT-energy demap with GI absorbing timing slop:
- GI = Ngi = 36 baseband samples = `Ngi * interpolation_rate` = 144
  full-rate samples (per Mercury memory `Default GI 3.0ms`).
- Phase-2 worst-case misalignment after refinement: ±2 full-rate
  samples = ±0.5 base-rate samples.
- 0.5 base-rate samples / 36 baseband-sample GI = 1.4% of GI. The
  symbol sits squarely inside the FFT window; no inter-symbol leakage.
- Per-bin energy loss from windowed sinc misalignment:
  `sinc²(Δ_base_rate / Nfft) = sinc²(0.5 / 256)` ≈ 1 − (π·0.5/256)² /3
  ≈ 1 − 1.27×10⁻⁵ — **less than 0.0001 dB per FFT bin**.
- The LLR magnitude depends on `E_target / noise_var`; both numerator and
  denominator see the same misalignment → cancellation → loss approaches 0.

**Bottom line: at the Phase-2 step-floor of ±2 full-rate samples, the
MFSK LLR SNR penalty is < 0.001 dB.** Below any conceivable
measurement.

### §18.3 What Option B could theoretically buy

To find an interesting "alignment gap" we'd need the new detector to be
deliver MEASURABLY worse alignment than the old. As §18.1 establishes,
they are equal. The only way Option B (adding cosine² as a precision
gate) could deliver dB is if it interpolated between Phase-2 samples
to get SUB-base-rate precision (parabolic peak fit). That's a
different mechanism from raw cosine² scoring — call it Option B'.

Option B' (parabolic fit on cosine² Phase-2 surface): would move the
±2 full-rate misalignment to ~±0.5 full-rate samples. Per §18.2 math:
LLR SNR penalty drops from <0.001 dB to <0.0001 dB. **Still nothing.**

The MFSK demap chain is fundamentally insensitive to sub-symbol
alignment as long as the symbol sits inside its GI-bounded window
(it does: 2 full-rate samples << 144-sample GI).

### §18.4 Hardware A/B noise floor vs Option B headroom

Hardware A/B baseline variance from prior experiments:
- §17.1/§17.2 mirror-bin A/B passes: per-cell variation ±20–50%
  (e.g., baseline +14 mean was 2.0 in §17.1, 1.5 in §17.2 — the
  SAME branch produced different absolute throughput on different
  runs).
- A.0.2 Moose clamp 2× hardware A/B (memory): "total bytes −28% and a
  clean-cell CONNECT failure at WGN:14 that baseline didn't have."
  Single-cell variance was ~±2 dB.

Option B's theoretical headroom: <0.001 dB.
Hardware A/B variance: ±2 dB per cell.

**Headroom : noise floor ratio ≈ 1 : 2000.** Even an ideal Option B
implementation would be 100% indistinguishable from noise in the IONOS
hardware loop. We could ship Option B and never know if it helped or
hurt — that's the definition of below-the-noise-floor work.

### §18.5 Counterargument — could mirror-bin acceptance be a confound?

§17 found mirror-bin acceptance is empirically load-bearing on IONOS
WB. Could mirror-bin be hiding a deeper alignment problem that Option
B would reveal? No, because:

- Mirror-bin operates on a DIFFERENT axis (which bin to count, not
  where in time to sample).
- The mirror-bin hit rate depends on residual CFO and FIR image
  rejection — both INDEPENDENT of time-alignment precision.
- If we had a time-misalignment-sensitive demap, mirror-bin would not
  rescue it (mirror is a frequency-domain phenomenon, not time-domain).

The §17 result tells us about CFO/image leakage on the WB IONOS
channel, NOT about timing precision. Option B cannot capture that
either.

### §18.6 What COULD move the cliff further

Per §17.4, viable next axes (NOT Option B):
- Preamble alphabet redesign (Costas-property optimized for IONOS
  fading correlation structure).
- Mini-Moose before each frame to suppress residual CFO that's
  leaking into the mirror bin.
- E3 LLR clip 20→40 (deferred per memory).
- Different channel-coding (BP+OSD branch already shipped, NULL delta
  — LDPC isn't the bottleneck).

None of these are "Option B" by any of the §4 (a)/(b)/(c) definitions.

### §18.7 Drop decision and rationale

**DROP Option B.** The §4 §1.1 §1.2 facts establish:
1. OLD cosine² and NEW discrete-match have **identical Phase-2 sample
   precision** (`interpolation_rate` step, ±2 full-rate samples).
2. MFSK noncoherent FFT-energy demap has < 0.001 dB sensitivity to
   misalignment within the GI.
3. Hardware A/B noise floor is ±2 dB per cell.
4. Option B's theoretical ceiling is < 0.01 dB. Ratio to noise floor:
   <1:200.

Per the CLAUDE.md §1 prior-art mandate, the hypothesis would also
need a citation — and there is no prior art for "noncoherent MFSK
data demap is sub-symbol-alignment sensitive at base-rate
resolution" because it isn't.

No worktree, no branch, no commits. Hand back to operator for one of
the §18.6 alternative axes.

### §18.8 Cross-layer audit note (CLAUDE.md §5)

No code change → no producers/consumers to audit. This section is the
audit record. The §10 audit in `data-flow-preamble_nSymb.md` remains
the authoritative producer/consumer walk for the data preamble path.

### §18.9 Open questions [?]

None for Option B itself. Two adjacent questions surface:
1. **[?] Should P1_OVERSAMPLE be re-introduced into Option A?** Per
   §18.1, the NEW Phase-1 step is 4× coarser than the OLD. The
   ±half-symbol Phase-2 window absorbs this, but if a Phase-1
   candidate is suppressed by FAR/threshold and the next coarse
   candidate is a full symbol away, the second Phase-2 window won't
   overlap the missed preamble center. Hardware A/B at WGN:-8 shows
   Option A WORKS, so this isn't a practical bug. Skip unless cliff
   pushes need it.
2. **[?] Is sub-base-rate timing precision ever needed?** OFDM
   configs (CONFIG_0..16) — possibly, since OFDM uses coherent
   per-pilot channel estimation. But that path uses
   `time_sync_preamble_halfsym` (Schmidl-Cox), not
   `time_sync_mfsk_corr`. Out of scope here.

### §18.10 Cross-references

- §1.1, §1.2 — OLD detector body and NEW detector body, Phase-2 step.
- §4 — original Option A/B/C ranking.
- §14 — Option A ship log.
- §17 — mirror-bin drop hardware verdict.
- `data-flow-preamble_nSymb.md` §10 — discrete-match producer/consumer audit.
- Memory `feedback_dsp_commit_patterns.md` — "magic numbers without
  measurement basis" warning. Option B would have been one if shipped.

---

## §19. Option C verdict + next-axis selection (2026-05-28)

**Status:** RESEARCH ONLY. **Recommendation: DROP Option C as a cliff-push
candidate** — it's pure architectural cleanup with 0 dB headroom by
§4's own framing. **Pivot to "mini-Moose before each MFSK data preamble"**
as the next cliff-push axis. The hardware finding in §17 (mirror-bin
empirically load-bearing → residual CFO is leaking signal energy into
the mirror) becomes a structural fix opportunity once §19.3 below is
factored in: the MFSK path explicitly *zeros* its frequency-offset
estimate (`telecom_system.cc:2193`), so there is currently NO residual-CFO
correction at all on the data-preamble path.

### §19.1 Option C honest re-read

§4.C verbatim: *"Introduce a new top-level `detect_preamble_pattern`
function that owns both MFSK preamble and CONNECT/ACK/HAIL/BREAK
detection via a unified parameter set. Move `detect_ack_pattern` to call
this. Delete `time_sync_mfsk_corr` body entirely."* — explicitly framed
as Phase-2 cleanup, not a new detector.

**Cited dB analysis (none provided in §4.C).** §4.D recommended Option A
and "schedule Option C as Phase-2 cleanup". The user's autonomous-run
loop already shipped Option A (+50% bps mean, cliff WGN:−4 → WGN:−8).
Option C's deliverable is: same detection math reached through one entry
point instead of two.

**What Option C does NOT do:**
- It does NOT expose a new signal that Option A doesn't already use.
  Option A already clones `detect_ack_pattern`'s discrete-match algorithm
  verbatim (`ofdm.cc:3040-3266` is structurally `detect_ack_pattern` with
  search_start_symb added). Consolidating them into a single function
  doesn't surface any new bin, FFT, or correlation.
- It does NOT enable a "combined detector" mode (dual-path FFT bins +
  matched filter). §18 already ruled that out — the two metrics measure
  the same thing at this layer.
- It does NOT change the Phase-2 step floor (`interpolation_rate` = 4
  full-rate samples — §18.1).
- It does NOT change mirror-bin acceptance (§17 proved that's load-bearing
  on real WB IONOS).

**Math walk:** Each structural change Option C would introduce:
1. Single entry point `detect_preamble_pattern(...)` taking pattern enum
   {DATA_PREAMBLE, CONNECT, ACK, BREAK, HAIL}: 0 dB. Pure dispatch.
2. Shared FFT-bin argmax inner loop: 0 dB. Algorithm is already shared
   by construction (Option A is a copy).
3. Unified threshold parameter table: 0 dB. Per-pattern thresholds
   already exist in `mfsk.cc` (`connect_match_threshold`, `ack_match_threshold`,
   `preamble_match_threshold`, etc.).
4. Delete `mfsk_corr_template` infrastructure: 0 dB. Already dead code
   per §14 — the cosine²-template buffer is allocated and populated
   but never read.

**Total cliff dB delta for Option C as defined: 0.000 dB.** Below the
±2 dB hardware A/B noise floor by >2000:1, same problem Option B had.

**Risk:** Option C touches CONNECT/HAIL/ACK/BREAK callers (the stable
WB control plane). Per §4.C, "breaks fix-narrow rule (CLAUDE.md §5);
cross-layer regression test matrix expands". Effort 10-15 days. The
risk:reward ratio is ∞:0 for a cliff push.

**Verdict:** DROP for cliff purposes. Keep on a backlog as voluntary
Phase-2 consolidation work if/when the codebase needs a maintenance pass
— but NOT as a route to more dB.

### §19.2 §18.6 axes — dB-ranked

| Axis | Predicted Δ at cliff | Effort | Risk | Math limit? |
|---|---|---|---|---|
| **Mini-Moose before each MFSK data preamble** | **~1–3 dB** (recovers mirror-bin energy by reducing residual CFO; depends on per-frame CFO drift) | **2–4 days** | Med (touches MFSK rx pipeline, ARQ timing) | No structural ceiling. CFO suppression directly tightens FFT-bin energy concentration. |
| Preamble alphabet redesign (Costas optimized for IONOS fading) | <0.5 dB | 4–6 days | Low (single-file change, hard cutover) | YES — already at Welch-Costas g=2; further optimization is below noise floor for AWGN, marginal for fading |
| E3 LLR clip 20→40 retest | **0 dB — moot** | n/a | n/a | LLR cap REMOVED entirely 2026-05-24 (`mfsk.cc:1086` comment "LLR cap removed"). There is no clip to relax. |
| Preamble N=16 → 24/32 extension | ~1.5–3 dB (√N gain: N=16→24 = +1.8 dB; N=16→32 = +3.0 dB) | 5–10 days | Med-High (frame timing, Bug #44 cross-correlation budget, ARQ poll math) | YES — once N ≥ ~32 the matched-filter gain saturates against per-symbol SNR; beyond that LDPC takes over |

**E3 axis is dead** — verified at `mercury/source/physical_layer/mfsk.cc:1086`.
The 2026-05-24 commit message in the code comment states the cap was
removed because "at rate-1/16 LDPC (ROBUST_0), the SPA decoder needs
the full magnitude of high-confidence tone observations to flip the
~94% parity-dominated codeword bits". Memory's "E3 LLR clip 20→40
deferred" entry is stale — there is no clip to push.

### §19.3 Why mini-Moose ranks #1 — and is bigger than memory thinks

§16.1 of THIS doc claimed Moose runs on MFSK pre-data-preamble via
`carrier_sampling_frequency_sync_nb`. **That claim is wrong.**

`source/physical_layer/telecom_system.cc:2193` reads:
```cpp
if(M == MOD_MFSK) freq_offset_measured = 0;
```

This explicit zero runs AFTER the WB Moose / NB cross-pilot estimator
on lines 2182–2191 — for MFSK, whatever the estimator returned is
discarded. Searching the file confirms `freq_offset_of_last_decoded_message`
is never updated for MFSK either (the only writes are inside OFDM
branches). The MFSK data preamble runs on whatever coarse-sync the
prescan / HAIL detect left behind, with NO fine refinement, EVER.

§17's finding ("mirror carries empirically load-bearing signal on
WB IONOS — likely residual CFO + WB FIR image leakage") is therefore
a direct consequence of zero CFO sync, not a small effect. The mirror
bin is carrying signal because **the carrier IS offset and Mercury MFSK
never corrects it.**

Adding a mini-Moose between MFSK preamble detect and data decode:
- Reuses `carrier_sampling_frequency_sync` (already implemented) on the
  16-symbol detected preamble window. Same algorithm WB OFDM uses.
- Mercury WB preamble is M=32 Welch-Costas; not nIS-periodic at half-
  symbol, but Moose's underlying technique (cross-symbol phase rotation
  of known tones) works on any pilot-rich symbol — Mercury already has
  `carrier_frequency_sync_nb` (`ofdm.cc:537`) which does exactly this
  cross-symbol-phase approach (capture ±22 Hz for NB; mirror-aware
  formula for WB Welch-Costas needs derivation).
- Apply the estimate as a baseband frequency shift to the data symbol
  buffer before the per-symbol FFT in `mfsk.demod`. Single multiplier
  per sample. ~50 LoC change.
- Cliff push hypothesis: if residual CFO is ~2-5 Hz typical (worst
  case ~15 Hz per memory "~11 Hz frequency offset between stations"),
  correcting it concentrates signal energy back into the expected bin.
  At M=32 over N=16 symbols, recovering even 1 dB of in-bin energy
  shifts the per-symbol p (probability-of-correct argmax) by ~5–8%
  absolute, which moves the matched-count cliff by ~1–3 dB.

### §19.4 Recommended hardware-test-ready next step

**Axis:** mini-Moose before MFSK data preamble decode.

**Hypothesis:** at WGN:−10 IONOS WB ROBUST_0, mean total bytes
non-zero across 3 passes (current §17.4 result: 0 bytes at WGN:−10).
Predicted Δ vs current Option A shipped baseline: +1 to +3 dB cliff
shift (WGN:−8 cliff → WGN:−10 to −11).

**Fail-before regression test** (in `mfsk_ctrl_codec_tests.cc` next to
the §14.8 suite):

```
mfsk_data_preamble_argmax_cfo_offset (POSITIVE, FAIL-BEFORE):
  load_configuration(ROBUST_0). Generate preamble. Round-trip through
  baseband_to_passband → passband_to_baseband, but INJECT a 7-Hz CFO
  via complex multiply on the passband signal. Add AWGN at in-band
  SNR ≈ −2 dB (between cliff and pre-cliff).
  5 PRNG seeds.
  Assert: at least 4/5 return delay ≥ 0 AND matched ≥ 7.
  Pre-fix: cosine-residual CFO at 7 Hz over 16-symbol window rotates
  the in-bin energy by ~360° (sym time ≈ 24 ms; 7 Hz × 384 ms ≈ 2.7 rev
  across N=16), pushing argmax to mirror or random bin for ~50% of
  symbols. Detector reports matched ≈ 4-6 (below T=7).
  Post-fix (mini-Moose applied): residual CFO ≤ 1 Hz, in-bin energy
  preserved, matched ≈ 13-16.
```

This test has the same shape as §14.8 `_cliff` test but adds a
deterministic CFO injection, exercising the failure path §17 found
empirically.

**Hard cutover policy.** No CAP bit. Both endpoints apply mini-Moose
unconditionally (matches Phase B / Option A precedent: no negotiation,
TX and RX implementations move together, no flag-day code to ever remove).

**Cross-layer audit obligation (CLAUDE.md §5):**
- Producer of `freq_offset_measured` for MFSK: new code in `receive_msg`
  MFSK branch, between `time_sync_mfsk_corr` return and `mfsk.demod`
  call (~`telecom_system.cc:1037-2249` block).
- Consumers: solely `mfsk.demod` per-symbol FFT input. Stream context
  unchanged. ARQ timing math (Nofdm × Nsymb) unchanged because Mercury
  measures frequency, not timing — symbol grid does not shift.
- Required new fact-doc: `data-flow-mfsk-cfo-sync.md` covering the new
  producer, the demap consumer, the test-mode pre-init reset, and the
  BREAK / SWITCH_ROLE invariants (MFSK is the control PHY post-Phase-B,
  so CFO state must reset cleanly on every state-machine transition).

**Effort:** 2–4 days code + tests; +1–2 days IONOS A/B at WGN:−8/−10/−12.

**Risk register:**
- Could regress Option A if mini-Moose mis-estimates and applies a
  wild correction. Mitigation: identical sanity-clamp to WB Moose
  (`±2 subcarrier spacings`), same `[MOOSE-REJECT]` path
  (`telecom_system.cc:2205`).
- The NB MFSK path (§17.1 implication) needs the same fix but on
  different tone geometry. Scope this commit to WB-only; NB as a
  follow-up.
- Mercury's existing `carrier_frequency_sync_nb` already computes the
  cross-symbol phase estimate on any known-modulation symbols — could
  be reused directly with the preamble_tones array instead of writing
  new code.

### §19.5 If mini-Moose proves too risky / slow

Fallback: **preamble N=16 → 24 extension.** +1.8 dB via √N. Same
cliff-push math the §14 plan used. Larger change footprint (frame
timing, Bug #44 budget, ARQ poll cadence) but well-understood; no
new DSP intuition required. Effort 5-10 days.

The preamble alphabet redesign and Option B-style refinement are NOT
recommended — neither has a math story that beats the ±2 dB noise floor.

### §19.6 If the user is tired — SHIP A AND STOP

The +50% bps mean and WGN:−4 → WGN:−8 cliff push from Option A is
the biggest available win in this work area. Subsequent options:
- Option B: dropped (§18, below noise floor).
- Option C: dropped (this §19, 0 dB delta).
- Mirror-bin drop: dropped (§17, hardware A/B regressed).
- T=6 threshold push: deferred (§15.8, FAR over operator gate).
- E3 LLR clip: moot (clip already removed 2026-05-24).

Remaining cliff axes are **all ≥ 2-day commitments** (mini-Moose at the
minimum) and **require fresh hardware A/B runs to validate** — neither
fits "one more quick push before stopping". If the user is at a stopping
point, the clean closeout is:
1. Confirm Option A is the merged shipped state on `monitor`.
2. Update memory entry to reflect §17/§18/§19 verdicts (mirror-drop
   tried & dropped; Option B dropped; Option C dropped).
3. Park the autonomous-run loop. The next session can pick up mini-Moose
   as a fresh axis with full context from §19.3.

**Author recommendation in priority order:**
1. If continuing autonomous-run: **mini-Moose for MFSK** (§19.3-4),
   2-4 days, predicted +1-3 dB cliff.
2. If stopping autonomous-run cleanly: §19.6 closeout. Option A's
   +50% is banked; the cliff has moved 4 dB; mercury is in a good place.

### §19.7 Cross-references

- §4 — original Option A/B/C ranking.
- §14 — Option A ship log (the win being banked).
- §17 — mirror-bin verdict (the empirical evidence that CFO is the
  next bottleneck).
- §18 — Option B drop (sets the precedent that "below noise floor =
  don't ship").
- `mercury/source/physical_layer/telecom_system.cc:2193` — the
  load-bearing line: `if(M == MOD_MFSK) freq_offset_measured = 0;`.
- `mercury/source/physical_layer/ofdm.cc:537+` —
  `carrier_frequency_sync_nb` (the cross-symbol-phase estimator that
  would back the mini-Moose for WB MFSK Welch-Costas).
- `mercury/source/physical_layer/mfsk.cc:1086` — LLR cap removal
  comment (killing the E3 axis).
- Memory `mfsk_vara_parity_audit_2026_05_25.md` — strategic context
  for which dB pushes are worth chasing.

---

## §20. Mini-Moose implementation plan (pre-code, 2026-05-28)

**Status:** PLAN — to be implemented on branch `feat/mini-moose` in worktree
`mercury-worktrees/mini-moose`. Baseline `a879e0e` (HEAD of monitor at start
of this work, all 22 `mercury.exe --test` tests pass). The work item is the
§19 recommendation: add a mini-Moose CFO refinement that runs AFTER
`time_sync_mfsk_corr` returns a sample-level `delay`, BEFORE `mfsk.demod`
consumes the audio. With the residual CFO removed, signal energy lives in
the expected FFT bin and the §17 empirical "mirror-bin is load-bearing"
finding (the direct hardware signature of zero MFSK CFO sync) should
disappear.

User-confirmed scope (per the autonomous-run rules):

1. **Hard cutover** — no negotiation, no CAP bit. Matches Phase B / Option A.
2. **MFSK data preamble only** — DON'T touch CONNECT / HAIL / ACK paths.
   Those use `detect_ack_pattern` (a separate detector), have separate
   per-call CFO behavior, and were proven on hardware down to WGN:-10.
3. **Reuse the cross-symbol-phase math of `carrier_frequency_sync_nb`** —
   adapt to WB Welch-Costas geometry as a NEW function
   `carrier_frequency_sync_wb_mfsk` rather than parameterizing the
   existing NB function (lower risk).
4. **No new INI / CLI knobs** — fixed parameters baked in.

### §20.1 Where the new estimator runs (call site)

The MFSK RX hot path in `cl_telecom_system::receive_msg` runs in this order
(`telecom_system.cc`):

| Line | Action |
|---|---|
| 1037 | `time_sync_mfsk_corr` returns full-rate `delay` from the matched-bin detector |
| 2120-2155 | Build `baseband_data` for the frame: `passband_to_baseband_decimated` mixes from passband to baseband using `carrier_frequency + coarse_freq_offset` and FIR-decimates. `baseband_data[0..(Nsymb+preamble_nSymb)*Nofdm-1]` now contains preamble + data at baseband, decimated rate. |
| 2168-2193 | Frequency-sync block. WB-OFDM calls `carrier_sampling_frequency_sync` (Moose), NB stubs `freq_offset_measured = 0`. Line **2193**: `if (M == MOD_MFSK) freq_offset_measured = 0;` overrides any value left by the OFDM branches. |
| 2195-2218 | Sanity clamp (±2 subcarrier spacings) + reject (continue to next sync trial). |
| 2220-2244 | Apply correction: if `M == MOD_MFSK` skip (current behavior); else if `fabs(freq_offset_measured) > freq_offset_ignore_limit`, re-mix from passband with `effective_carrier_freq + freq_offset_measured`. This rewrites `baseband_data` with the CFO-corrected signal. |
| 2247-2250 | Per-symbol `symbol_demod` (GI-strip → FFT → depad) for each of `get_active_nsymb()` data symbols, reading `baseband_data[i*Nofdm + Nofdm*preamble_nSymb]`. |
| 2253-2257 | `mfsk.demod` reads the FFT-domain symbols and emits LLRs. |

**Mini-Moose insertion site:** REPLACE the contents of the bare-statement
`if (M == MOD_MFSK) freq_offset_measured = 0;` at line 2193. The new code
estimates residual CFO from `baseband_data[0..preamble_nSymb*Nofdm-1]`
(the preamble portion of the freshly-mixed frame) via the new
`carrier_frequency_sync_wb_mfsk` and stores the result in
`freq_offset_measured`. The existing sanity / clamp / re-mix block at
2195-2244 already does the right thing as long as we let
`freq_offset_measured` carry a value for MFSK:

- §1: the ±2 subcarrier sanity clamp will reject any wild estimate (e.g.
  a noise-driven artifact) and re-trigger `continue` to the next sync trial.
  Identical safety net to the WB-OFDM Moose path.
- §2: the `else if(fabs(freq_offset_measured) > ofdm.freq_offset_ignore_limit)`
  branch currently has an `if(M == MOD_MFSK)` skip at 2220. We change that
  block so MFSK ALSO re-mixes the frame from passband with the corrected
  `effective_carrier_freq + freq_offset_measured`. This is the same fused-
  polyphase code as the OFDM branch — just remove the MFSK skip.

This keeps ALL invariants of the surrounding code identical: same clamp,
same reject path, same re-mix entry-point, same per-symbol FFT consumer.
The only behavioral change is "for MFSK the freq offset is no longer
hard-zeroed".

### §20.2 The estimator algorithm

Mercury already has the cross-symbol-phase technique in
`carrier_frequency_sync_nb` (`ofdm.cc:537+`). Per §19.3, the technique
works on any pilot-rich symbol — including the M=32 Welch-Costas WB MFSK
preamble. The adaptation needed:

**NB OFDM preamble** has Nc=10 subcarriers, all loaded, with known
modulation (`ofdm_preamble[sym*Nc+k].value`). The estimator:
1. FFT each preamble symbol.
2. Strip known modulation: `H[k] = X_recv[k] * conj(known[k])`.
3. Cross-correlate adjacent symbols: `C += H_cur[k] * conj(H_prev[k])`.
4. Phase of C corresponds to CFO * T_symbol. Solve for CFO.

**WB MFSK preamble** has Nc=50 subcarriers; only `nStreams` of them
carry the (single) Welch-Costas tone per symbol. The known-modulation
amplitude is known (`amp = sqrt(Nc/nStreams)`, see `mfsk.cc:500`), the
phase is +1 (real). Across the 16-symbol preamble, the tone position
HOPS — symbol `s` puts power into bin `preamble_tones[s % preamble_nSymb]`
within each stream's band. The cross-symbol-phase math still works, but
we cannot correlate `H[k]` between adjacent symbols at the SAME bin k
because the tone position moves.

**The fix is per-bin cross-symbol correlation, evaluated at the bin
that's "lit" in BOTH symbols.** When symbol s and symbol s-1 put energy
into different bins, that pair contributes 0. When they put energy into
the same bin (which happens every `preamble_nSymb / 8 = 2` symbols on
average because the 8-tone Welch-Costas base repeats every 8 symbols,
giving 7 same-bin neighbor pairs across the 16-symbol expansion), the
phase difference encodes CFO.

But a simpler equivalent that avoids enumerating same-bin pairs:

**Cross-symbol correlation on the COMPLEX FFT-bin VALUE at each symbol's
EXPECTED tone bin, regardless of the bin moving.** Define:
  X[s] = (sum of FFT-bin values at the expected tone bin for symbol s,
          summed across streams)
Then:
  C = sum over s = 1..preamble_nSymb-1 of  X[s] * conj(X[s-1])
       * exp(j * 2π * (f_s - f_{s-1}) * (something))

No — the bin-frequency shift between adjacent symbols contributes a
deterministic phase that has nothing to do with CFO. We'd have to
subtract it.

**Cleaner design — REUSE the NB technique by treating each preamble
symbol's expected-tone bin as a "pilot" and pre-multiplying by the
KNOWN tone-frequency rotation.** Since each preamble symbol is a CW
tone at a known frequency, the residual CFO manifests as a per-symbol
phase rotation that's IDENTICAL for every symbol (CFO is constant
across the preamble). The cross-symbol phase difference is:

  arg(X[s] * conj(X[s-1])) = 2π * (CFO + f_tone[s] - f_tone[s-1]) * T_sym

where f_tone[s] is the s-th preamble tone's baseband frequency
(known). Subtracting the known-tone term gives:

  CFO = (arg(X[s] * conj(X[s-1])) - 2π * (f_tone[s] - f_tone[s-1]) * T_sym) / (2π * T_sym)

Equivalently, pre-de-rotate X[s] by exp(-j * 2π * f_tone[s] * T_sym * s)
so all preamble symbols look "modulationless"; then the cross-symbol
phase difference is purely CFO * T_sym, identical to the NB Moose math.

Implementation:
```
for s = 0..preamble_nSymb-1:
  symbol_bb = baseband_data[s * Nofdm .. s * Nofdm + Nfft - 1]  (skip Ngi)
  fft(symbol_bb, fft_out)
  bin = expected FFT bin for preamble_tones[s % preamble_nSymb]
        across nStreams streams
  X[s] = sum over streams of fft_out[bin]
  X[s] *= exp(-j * 2π * tone_freq[s] * T_sym * s)  // de-rotate known modulation
C = sum over s >= 1 of X[s] * conj(X[s-1])
CFO_residual = arg(C) / (2π * T_sym)
```

Sanity: confidence gate identical to NB (`|C| / energy_total < 0.05`
returns 0). Capture range: ±1/(2 * T_sym) = ±fs_base / (2*Nofdm).

For WB ROBUST_0 (Nofdm = Nfft + Ngi = 256 + 36 = 292), fs_base =
sampling_frequency / interpolation_rate = 12000/4 = 3000 Hz. Capture
range = ±3000/(2*292) = **±5.14 Hz**. WAIT — too narrow.

**This is the math problem of using NB-style cross-symbol phase on WB:
the WB symbol period is longer (Nofdm = 292 vs ~Nfft+Ngi for NB), so the
unambiguous phase per symbol corresponds to a smaller frequency range.**

OK, so use a SHORTER baseline. Cross-correlate symbols that are 1 sym
apart for high precision (±5 Hz capture), AND symbols that are 8 sym
apart for sub-Hz precision. NO — we want the OPPOSITE: a wider capture
range. Use HALF-symbol cross-correlation (every-2-symbol-period
repetition):

**Welch-Costas g=2 base has period 8 (8 distinct tones, then repeats).
The 16-symbol expansion is base × 2 reps. So symbol s and symbol s+8
have the SAME tone. The phase difference between X[s] and X[s+8]
under constant CFO is:**

  arg(X[s+8] * conj(X[s])) = 2π * CFO * 8 * T_sym

Capture range: ±1/(2 * 8 * T_sym) = **±0.64 Hz**. Worse — longer
baseline means tighter range. We need SHORTER baselines for wider
capture.

**Final design: cross-symbol-phase with baseline = 1 symbol, with the
known-tone de-rotation that subtracts the per-symbol modulation phase.**
Capture range = ±5.14 Hz. This is below the WB Moose ±2-subcarrier
clamp (±93.75 Hz) but ABOVE the §6.7 expected residual CFO budget after
the coarse-sync pre-mix:

- §6.7 of `phase-b-mfsk-connect-research.md` notes that the coarse
  carrier-freq search on the MFSK path produces an estimate at ±~3 Hz
  granularity, and the residual is typically ≤5 Hz.
- The memory entry "~11 Hz frequency offset between stations" refers
  to the TOTAL frequency offset between SGTL5000 crystals — the
  coarse-sync at `telecom_system.cc:2117` adds `coarse_freq_offset`
  to the carrier before the data re-mix at :2149. After that, the
  RESIDUAL CFO entering the mini-Moose is just whatever the coarse
  sync missed.
- For IONOS WGN cells we're targeting (WGN:-8 to -12), the coarse
  sync metric degrades, residual CFO grows. A ±5 Hz capture range
  may not be enough at the worst cells.

**Wider-capture alternative:** use a HALF-SYMBOL baseline. Each WB MFSK
preamble symbol is one CW tone for the full Nofdm samples. The CW tone's
phase advances LINEARLY across the symbol. Take the FIRST half-Nfft of
the symbol and the SECOND half, treating them as two "mini-symbols":

  half_a[k] = FFT of bb[s*Nofdm + Ngi .. s*Nofdm + Ngi + Nfft/2 - 1] (padded to Nfft)
  half_b[k] = FFT of bb[s*Nofdm + Ngi + Nfft/2 .. s*Nofdm + Ngi + Nfft - 1] (padded)

phase(half_b * conj(half_a)) at the expected tone bin = 2π * CFO * T_half_sym
+ deterministic-known-tone-rotation. Subtract known, solve for CFO.

T_half_sym = (Nfft/2) / fs_base = 128/3000 = 42.67 ms. Capture range
= ±1/(2 * 0.04267) = **±11.7 Hz**. STILL might be too narrow for
worst-case IONOS, but it's now > the §6.7 budget.

**DECISION:** ship the half-symbol-baseline variant. Capture ±11.7 Hz,
which covers crystal mismatch (~11 Hz per memory) PLUS the coarse-sync
residual. If hardware A/B shows the cliff DOESN'T move at WGN:-10
because CFO at the cliff exceeds ±12 Hz, that's a follow-up axis
(go to a 3-baseline estimator combining halves + 1-sym + 2-sym for
ambiguity resolution).

Math reference: Moose 1994 (already cited at `ofdm.cc:532`), same
half-symbol technique used for OFDM WB Moose at `ofdm.cc:474` —
mini-Moose for MFSK is structurally the same algorithm, just adapted
to the MFSK preamble's CW-tone shape instead of the WB preamble's
every-other-subcarrier shape.

### §20.3 How CFO is APPLIED

Mercury already has the correction infrastructure. After the existing
`passband_to_baseband_decimated` produces `baseband_data` at
`effective_carrier_freq = carrier_frequency + coarse_freq_offset`, the
OFDM-data path at `telecom_system.cc:2224-2244` re-mixes from passband
with `effective_carrier_freq + freq_offset_measured` if Moose returned
a nontrivial value:

```cpp
ofdm.passband_to_baseband_decimated(&data[pb_start], pb_size,
    data_container.baseband_data_interpolated,
    sampling_frequency,
    effective_carrier_freq + freq_offset_measured,  // <-- corrected mix
    carrier_amplitude,
    Mdec, &ofdm.FIR_rx_data, pb_start);
for(int i = 0; i < frame_dec; i++)
    data_container.baseband_data[i] =
        data_container.baseband_data_interpolated[margin_dec + i];
```

**The fix:** remove the `if(M == MOD_MFSK) {/* skip */}` guard at line
2220. Both OFDM and MFSK now run the re-mix when
`fabs(freq_offset_measured) > freq_offset_ignore_limit` (typically
~3 Hz). For sub-threshold residuals, the re-mix is skipped and the
existing `baseband_data` is used as-is (matches OFDM's behavior).

This is structurally clean because the new estimator produces its
result EARLIER (right at line 2193 where it replaces the hard-zero)
and the existing apply-block at 2224-2244 already handles the rest.
We touch exactly two sites: the estimator call (replacing the line
2193 zero) and the re-mix guard (removing the MFSK skip at 2220-2223).

### §20.4 Cross-layer state touched

ONLY `freq_offset_measured` (a function-scope local in `receive_msg`,
declared at `telecom_system.cc:810`). Let me audit:

| Producer | Site | Behavior pre-fix | Behavior post-fix |
|---|---|---|---|
| `ofdm_forced_delay >= 0` (BER test) | :2162 | = 0 | = 0 (unchanged; mini-Moose not run when ofdm_forced_delay set) |
| `use_last_good_freq_offset` cache hit | :2166 | from cache | from cache (unchanged) |
| `narrowband_enabled` branch | :2177 | = 0 | = 0 (unchanged — NB MFSK out of scope) |
| WB OFDM branch | :2188 | Moose nIS=4 | Moose nIS=4 (unchanged) |
| MFSK override | :2193 | = 0 always | **= carrier_frequency_sync_wb_mfsk(...) residual** for WB MFSK; = 0 for NB MFSK (out-of-scope) |

| Consumer | Site | Behavior pre-fix | Behavior post-fix |
|---|---|---|---|
| Sanity clamp + REJECT trial | :2195-2218 | clamp + reject when |freq| > ±2 subcarriers | Same. Now applies to MFSK too. The reject path advances `sync_trials++` and `continue`s — identical safety. |
| Re-mix MFSK skip | :2220-2223 | skip — no re-mix for MFSK | **DROPPED** — MFSK now re-mixes when |freq_offset_measured| > limit |
| Re-mix OFDM-fine | :2224-2244 | re-mix passband with offset | Same for OFDM; ALSO triggers for MFSK when above ignore_limit |
| OFDM-OK log line | :2708-2712 | prints freq_offset_measured for OFDM only | OFDM only (MFSK doesn't reach this log path); no change |
| Cache update | :2742-2745 | `freq_offset_of_last_decoded_message = freq_offset_measured` for non-MFSK ONLY | Unchanged — `if(M != MOD_MFSK)` guard preserves MFSK's no-cache behavior so the residual doesn't get applied to a subsequent OFDM gearshift |

The cache-guard at :2742 is critical — without it, an MFSK residual CFO
could leak into the OFDM `use_last_good_freq_offset` path (line 2164,
applied on `sync_trials == max`). Preserved verbatim.

### §20.5 Why CONNECT / HAIL / ACK paths are NOT touched

- CONNECT / HAIL / ACK detection goes through `cl_ofdm::detect_ack_pattern`
  (`ofdm.cc:3231+`), called from various per-pattern functions in
  `arq_common.cc` and `arq_*.cc`. NONE of these call
  `time_sync_mfsk_corr`. They have their own argmax-bin detection and
  return their own delay metrics; no shared state.
- The Phase B MFSK CONNECT suffix decode does NOT go through the
  data-preamble path either — it's a SUFFIX appended to a pattern frame,
  decoded by `decode_suffix_tones` (`ofdm.cc:3567+`). Mini-Moose for
  the data preamble would NOT touch any of this.
- BREAK detection is also `detect_ack_pattern`. Untouched.
- The §17 hardware verdict that proved control-frame detection works
  down to WGN:-10 used CURRENT (unrefined) CFO behavior — adding a
  fine-CFO refinement to data preamble alone preserves that proven
  behavior.

### §20.6 The fail-before-passes regression test

Following §19.4, adapted to the actual code structure:

**Test A: `mfsk_data_preamble_mini_moose_recovers_cfo` (POSITIVE,
FAIL-BEFORE-PASSES on monitor's pre-fix code).**

```
- load_configuration(ROBUST_0).
- Synthesize preamble + AWGN at sigma_pb = 3 × preamble_rms (in-band
  SNR ≈ -2 dB, comfortably above the §6.2 cliff).
- BEFORE the preamble passes through passband_to_baseband, INJECT a
  known CFO_in (e.g. +7 Hz) by multiplying the passband by cos(2π * CFO_in * t).
- Run time_sync_mfsk_corr to locate the preamble (succeeds — the
  detector accepts mirror-bin).
- THEN call carrier_frequency_sync_wb_mfsk on the baseband-data slice
  starting at delay.
- Assert |estimated_CFO - CFO_in| < 1.5 Hz across 5 PRNG seeds.

Pre-fix: the symbol does not exist (the function isn't written).
Test FAILS at the link step or with a "symbol not found" error.
After-fix: symbol exists, CFO recovered, test PASSES.
```

**Test B: `mfsk_data_preamble_mini_moose_zero_cfo_no_op` (POSITIVE,
NO-REGRESSION guard).**

```
- load_configuration(ROBUST_0).
- Synthesize preamble + AWGN at sigma_pb = 0 (clean channel, NO CFO).
- Run carrier_frequency_sync_wb_mfsk.
- Assert |estimated_CFO| < 0.5 Hz.
- The full preamble + demod round-trip with mini-Moose enabled produces
  IDENTICAL frame data to pre-fix (modulo numerical noise in the demod
  output well within float epsilon).
```

**Test C: `mfsk_data_preamble_mini_moose_high_cfo_rejected` (NEGATIVE,
sanity-clamp guard).**

```
- load_configuration(ROBUST_0).
- Synthesize preamble; inject CFO = 200 Hz (way above the ±2-subcarrier
  clamp of ±93.75 Hz, and above the mini-Moose capture range).
- Run carrier_frequency_sync_wb_mfsk.
- Assert either: returns 0 (low confidence gate triggers) OR returns a
  bounded value that the receive_msg clamp will reject. Either way, the
  detector NEVER returns a wild value > sanity_limit that would silently
  corrupt the data path.
```

(Test C is the cross-layer regression guard required by CLAUDE.md §5.)

All three tests hook into `mfsk_ctrl_codec_tests.cc`, run via
`mercury.exe --test`. Fail-before-passes verified by stashing the new
`ofdm.cc` and `telecom_system.cc` edits, rebuilding, running tests.

### §20.7 No-regression argument

At high SNR / clean signal, `carrier_frequency_sync_wb_mfsk` will see
no measurable cross-symbol phase rotation. The cross-symbol-phase
estimator integrates over 15 symbol-pairs × nStreams subcarriers ≈
15 complex measurements. Per-measurement phase noise variance ~1/(2*SNR);
estimator variance scales as 1/(N_pairs * SNR). At SNR = +30 dB,
estimator std-dev is ≈ 0.001 rad, mapping to ≈ 0.01 Hz at T_half_sym.
The `freq_offset_ignore_limit` (typically ~3 Hz) ensures sub-threshold
residuals are skipped — re-mix doesn't run, `baseband_data` stays
identical to pre-fix. Existing high-SNR tests
(`mfsk_data_preamble_argmax_clean`, `_high_snr_no_regression`) will
still report `matched = 16/16` because the data path is byte-identical.

At cliff SNR (WGN:-8), residual CFO ≈ 3-7 Hz. The mini-Moose either:
1. Estimates correctly → re-mix shifts energy out of mirror bin into
   expected bin → §17 mirror-load-bearing effect disappears →
   detection works without OR-accept of mirror bin → can revisit
   T=7→6 push in a follow-up commit.
2. Estimates badly (low confidence) → returns 0 → re-mix skipped →
   behavior identical to today. Worst-case: no improvement, no
   regression.
3. Estimates wild (>±93.75 Hz) → sanity clamp rejects → next sync
   trial. Worst-case: one wasted trial, same trials_max → same
   number of overall retries → no regression.

### §20.8 Commit list (planned)

1. `docs(mini-moose): §20 plan — mini-Moose CFO refinement for MFSK
   data preamble` (this section into the worktree fact-doc).
2. `docs(data-flow): freq_offset_measured cross-layer audit`
   (new `data-flow-freq_offset_measured.md`).
3. `phy(ofdm): add carrier_frequency_sync_wb_mfsk half-symbol estimator`
   (new function in `ofdm.cc`, declaration in `ofdm.h`).
4. `phy(telecom_system): apply mini-Moose to MFSK data preamble`
   (call new estimator at :2193, drop MFSK skip at :2220-2223).
5. `test(mini-moose): cross-symbol-phase CFO regression suite`
   (3 new tests in `mfsk_ctrl_codec_tests.cc` §7). Verify
   fail-before-passes per §20.6.

### §20.9 Operator backlog

- Hardware A/B sweep at IONOS WGN ∈ {+14, +6, 0, -4, -8, -10, -12},
  ROBUST_0, 180s dwell, 3 passes per arm vs baseline `a879e0e`.
  Tool: `tools/axis_walk_sweep.py --pin-config 100`.
- Expected: same or better total bytes at WGN ≥ -4; non-zero bytes
  at WGN:-10 (current Option A baseline: 0 bytes).
- Diagnostic to look for in logs: a new `[MFSK-MINI-MOOSE]` line
  reporting the estimated residual CFO at each successful preamble
  detection. If the value is consistently >0 at the cliff cells, the
  estimator is doing useful work. If it's hovering at 0, residual is
  below the threshold and the estimator path is a no-op.
- Verify NB MFSK path is unaffected: NB ROBUST_0 throughput at
  WGN:14 should match baseline.

### §20.10 Cross-references

- §14 — Option A ship log (the discrete-match detector this fix builds on).
- §17 — mirror-bin verdict (the hardware evidence motivating mini-Moose).
- §19 — Option C drop + next-axis selection (the contract for this work).
- `data-flow-freq_offset_measured.md` — companion data-flow audit (next commit).
- `mercury/source/physical_layer/ofdm.cc:474` — WB OFDM Moose (`carrier_sampling_frequency_sync`).
- `mercury/source/physical_layer/ofdm.cc:537` — NB OFDM cross-symbol-phase
  estimator (`carrier_frequency_sync_nb`) — algorithmic template.
- `mercury/source/physical_layer/telecom_system.cc:2193` — the zero-out
  line being replaced.
- `mercury/source/physical_layer/telecom_system.cc:2220-2244` — the
  re-mix block being extended to MFSK.
- Memory `mfsk_vara_parity_audit_2026_05_25.md` — strategic dB context.


---

## §22. Control-frame mini-Moose hardware verdict (2026-05-28)

`feat/mini-moose-ctrl` (`eca7667`) ran 3-pass-per-arm hardware A/B at cells `14, -8, -10, -11, -12, -13`. **REGRESSION** at the working SNR cells:

| WGN | Baseline mean | Ctrl mean | Δ |
|---|---|---|---|
| +14 | 1.3 | 1.5 | parity (variance) |
| −8 | 2.2 | 1.6 | −27% |
| −10 | 2.5 | 0.9 | **−64%** |
| −11 | 2.3 | 0.6 | **−74%** |
| −12 | 0 | 0.4 | 1/3 passes (tantalizing) |
| −13 | 0 | 0 | both fail |
| total bytes | 187 | 112 | **−40%** |

The cell-pattern shows the regression: ctrl-arm produces zeros where baseline was reliably 2-3 bps. The flash at WGN:−12 (one pass at 1.1 bps where baseline got 0/3) suggests the underlying CFO-refinement idea could work — but the regression at −10/−11 swamps any cliff-edge unlock.

Most likely root cause per §8.1 (the test the implementation agent flagged): sign convention. The ctrl-frame apply formula uses `effective_carrier + residual` but agent's empirical evidence pointed at `−`. The data-preamble path (§20) shipped with `+` and still delivered +11.7% on hardware — so the §22 result on its own is ambiguous about whether it's a sign bug or a different sibling bug.

Plan: **§20 sign-flip A/B will disambiguate**. If sign-flip on data-preamble shows a clear win, the formula needs flipping everywhere and the ctrl-frame branch deserves a respin. If sign-flip on data-preamble is null/regressing, the ctrl-frame regression is something else (sample-window alignment? estimator window choice for short ctrl frames?) and the branch is dead for different reasons.

Branch `feat/mini-moose-ctrl` dropped without merge.

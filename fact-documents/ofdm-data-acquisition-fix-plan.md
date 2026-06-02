# OFDM-Data Acquisition Fix — Research / Plan (rank-2 "beat VARA mid-SNR")

**Status:** PLAN ONLY. No code edits this pass. Hand-off doc for the
implementing agent.

**Date:** 2026-06-01. Inner repo `mercury/` @ monitor `fef293f`.

**Problem (characterized by agent a6b4f78a):** OFDM BPSK configs CONFIG_0-3
(62/137/211/286 bps) DECODE cleanly down to SNR3k −7.7/−6.2/−4.7/−3.7 dB, but
the OFDM-data frame cannot be ACQUIRED below SNR3k ≈ **+0.9 dB**. The 4-symbol
WB preamble Schmidl-Cox self-correlation metric (`ofdm.cc:2602`,
M=|P|²/(A²·R)) floors at the noise-coincidence level (~0.11) and only crosses
the **0.15** detect threshold (`telecom_system.cc:1131,1466`) at +0.9 dB.
A fixed SKIP-VAR gate (`telecom_system.cc:2527`, nv>0.5) bails the decoder at
the same spot. **5-9 dB of decode reach is wasted.** This is the #53
"OFDM data fails on real HF" root cause.

**Headline conclusion:** the cliff is a *metric-form* problem, not a
threshold-tuning problem. The Schmidl-Cox **autocorrelation** metric has a
**length-invariant ceiling** set purely by SNR — lengthening the preamble does
NOT lower the cliff (only tightens detection variance). To move the floor you
must switch to **coherent cross-correlation against the KNOWN preamble**, whose
detection mean grows with integration length. **Mercury already contains an
unused, fully-written coherent detector** (`time_sync_preamble_fft`,
`ofdm.cc:2684`, ZERO production callers) — the fix is to wire it in, not invent
one. This is the exact analog of the shipped MFSK b328a4d fix
(cosine²-MEAN → discrete-FFT-bin argmax).

---

## §1. The proven analog — b328a4d (MFSK data-preamble fix)

**Commit b328a4d** (2026-05-27) and its predecessor `da5f5b2` (the PHY change)
moved the MFSK data cliff WGN:−4 → WGN:−8 (~4 dB, +50% bytes). Full mechanism
in `data-preamble-port-research.md` (§1, §2) and `data-frame-cliff-audit-
2026-05-27.md` (§13.9, the failed sum-form attempt that proved the
length-invariance theorem).

### §1.1 The mechanism, cited

- **OLD MFSK detector** `time_sync_mfsk_corr` (`ofdm.cc:3023-3240`): per-symbol
  cosine²-similarity vs a round-tripped template, then **MEAN** across symbols,
  absolute threshold 0.5. `data-frame-cliff-audit §13.9`
  (`ofdm.cc` lines cited there) proved: `mean(sym_metric) ≥ 0.5` ⟺
  `sum(sym_metric) ≥ 0.5·N` — **identical decision regardless of N.** The
  detection-SNR floor sits at per-sample SNR ≥ 0 dB (γ≥1), independent of
  preamble length. Longer preamble buys variance reduction (√N) but NOT a
  lower floor. (This is the central theorem; it transfers directly to
  Schmidl-Cox — see §3.2.)

- **NEW MFSK detector** = port of `detect_ack_pattern` (`ofdm.cc:3244-3532`):
  per-symbol **discrete FFT-bin argmax** matched against a KNOWN tone sequence;
  metric = **COUNT** of symbols where the expected bin is the FFT-peak bin;
  threshold `matched ≥ 7/16` (length-scaled). Why it works at low SNR
  (`data-preamble-port-research.md §2.2`): the detection statistic
  `Z = (E[K_signal] − E[K_random]) / Stddev[K_random] ≈ √N·M·(p − 1/M)/√M`
  **grows as √N** — the matched-filter integration gain shows up in the
  threshold *decision* because the threshold scales with N (`N·θ` for fraction
  θ), unlike the autocorrelation mean.

- **Tone design** (`data-preamble-port-research.md §14.1`,
  `mfsk.cc:122-170`): Welch-Costas g=2 for M=16 `{2,4,8,0,15,13,9,1}`, 2× scaled
  for M=32, hop baked into `preamble_tones[]`. Cross-correlation vs
  CONNECT(g=3)/ACK(g=5)/BREAK(g=7)/HAIL(g=6) kept pairwise-Hamming ≥ 6/16
  (`test_base_pattern_cross_correlation`). FAR 2.5×10⁻⁷/poll at M=32, 7/16.

### §1.2 Why the analog is *structural*, not just thematic

Both the MFSK and OFDM data preambles failed at the same conceptual point: a
**self-referential / shape metric** (cosine²-MEAN; |P|²/A²R autocorrelation)
whose detection mean is **independent of integration length**, gated by an
absolute threshold. The MFSK fix replaced it with a metric matched against the
**known transmitted sequence** (FFT-bin argmax). The OFDM fix is the same move:
replace the autocorrelation with **coherent correlation against the known
preamble subcarrier values** (`ofdm_preamble[].value`). The key difference is
that for OFDM, **the replacement detector is already written and tested-shaped
in the tree** (`time_sync_preamble_fft`), whereas for MFSK it had to be ported
from `detect_ack_pattern`.

---

## §2. The OFDM-data acquisition path (current)

### §2.1 The detector in use: Schmidl-Cox self-correlation

**`cl_ofdm::time_sync_preamble_halfsym`** (`ofdm.cc:2542-2638`) and its 2-phase
wrapper **`time_sync_preamble_halfsym_2phase`** (`ofdm.cc:2640-2682`).

- Exploits the preamble's **time-domain L-sample periodicity**: WB nIS=4 →
  every-4th subcarrier on → period L=Nfft/4=64; NB nIS=2 → L=Nfft/2=128
  (`ofdm.cc:2560-2561`, configurator `ofdm.cc:1341-1370`).
- Metric (`ofdm.cc:2598-2605`):
  `P = Σ conj(r[d+m])·r[d+m+L]`, `A² = Σ|r[d+m]|²`, `R = Σ|r[d+m+L]|²`,
  `M = |P|²/(A²·R)` — bounded [0,1] by Cauchy-Schwarz, phase-invariant.
- Summed over `nsym = preamble_configurator.Nsymb` symbols (`ofdm.cc:2563,
  2579`).
- Energy-weighted earliest-peak selection (`ofdm.cc:2607-2632`); early-exit at
  `early_exit_metric` (caller passes the 0.15 WB threshold via the
  halfsym_2phase coarse stage, then fine at base-rate).

### §2.2 Preamble structure (TX) — confirmed 4 symbols WB

- **OFDM-data preamble length** `ofdm_preamble_configurator_Nsymb = 4` for
  **every** config (`physical_config.cc:50`; `telecom_system.cc:5026-5089`,
  all branches set 4). Stored into `ofdm.preamble_configurator.Nsymb`
  (`telecom_system.cc:5297`) and into `data_container.preamble_nSymb`.
  - NOTE: the `narrowband_enabled ? 8 : 16` at `telecom_system.cc:5311` is the
    **MFSK** preamble (`data_container.preamble_nSymb` for ROBUST modes), a
    DIFFERENT field/path. The OFDM-data preamble is 4. The prompt is correct.
- Preamble subcarriers built in `cl_preamble_configurator::configure`
  (`ofdm.cc:1341+`): every-`subcarrier_step`-th bin populated; WB step=4 →
  nIdentical_sections=4, ~12 preamble bins/symbol (`ofdm.cc:1355-1357`).
- TX emits preamble + `Nsymb` data symbols
  (`telecom_system.cc:4524/4543/4579`, length `Nofdm·interp·(Nsymb+preamble
  Nsymb)`).

### §2.3 The RX state machine — 13 call sites

`time_sync_preamble_halfsym` is invoked from **13 sites** in
`telecom_system.cc` (grep-confirmed): 1171 (BER self-test), 1243 (BATCH
predict-verify, HOT), 1323 / 1390 (coarse / fine main acquisition), 1561 / 1734
/ 2881 (bounds-failed recovery retries), 1950 / 2005 (coarse-freq ±30 Hz
search, site 6/7). The 2-phase wrapper is the coarse entry; sites pass step=1
on the decimated buffer (Plan-B Step-3 pattern, all mapped back to full-rate).

Primary acquisition flow (`receive_msg`, OFDM branch `telecom_system.cc:1110+`):
1. passband→baseband decimate (`:1188-1190`).
2. coarse Schmidl-Cox over the buffer → `receive_stats.delay`,
   `receive_stats.coarse_metric` (`:1252, 1322-1422`). Gate:
   `correlation ≥ preamble_detect_threshold` (0.15 WB / 0.30 NB, `:1129-1131,
   1464-1466`).
3. bounds check `pream_symb_loc ∈ (lower,upper]` (`:1456-1486`); else recovery
   (energy scan + retry Schmidl-Cox).
4. coarse-freq ±30 Hz search if trial 0 fails (`:1909-2010`).
5. fine GI+halfsym for Moose alignment, then per-trial Moose freq sync
   (`:2011+`).
6. channel estimate (ZF for NB, LS+DFT-smooth) → `noise_variance_estimate`
   (`ofdm.cc:1598`, `estimate_noise_from_pilot_pairs` `ofdm.cc:1475-1518`).
7. **mean_H gate** (`:2486`) then **SKIP-VAR gate** (`:2527`) → LDPC.

### §2.4 The unused coherent detector ALREADY in the tree

**`cl_ofdm::time_sync_preamble_fft`** (`ofdm.cc:2684-2837`) and
**`time_sync_preamble_fft_fine`** (`ofdm.cc:2839+`).

- **ZERO production callers.** Grep `time_sync_preamble_fft\b` across the whole
  repo returns only `ofdm.cc` (definition) + `ofdm.h` (declaration). Orphaned.
- Mechanism (`ofdm.cc:2762-2792`): for each candidate position, FFT each
  preamble symbol, accumulate **coherently** across symbols against the known
  preamble values:
  `bin_accum[b] += fft_out[bin] · conj(ofdm_preamble[sym·Nc+sc].value)`,
  then `metric = Σ_b |bin_accum[b]|²`. Coherent sum → signal grows ∝Nsymb,
  noise ∝√Nsymb → **metric ∝ Nsymb² (signal) vs Nsymb (noise)**. The
  comment (`ofdm.cc:2696-2701`) states exactly this: "preamble ≈ Nsym, data ≈ 1,
  threshold ~2".
- Final normalization (`ofdm.cc:2811-2835`): `correlation = metric / energy`
  (energy at preamble bins), giving a ratio ~Nsymb at preamble, ~1 at
  noise/data. **This is a DIFFERENT scale from the Schmidl-Cox [0,1] metric**
  — central integration constraint (§5).
- This is the OFDM-domain equivalent of `detect_ack_pattern`'s discrete-bin
  approach, but **coherent** (uses the complex preamble value, not just bin
  argmax) — strictly stronger when CFO is small, weaker if CFO rotates the
  phase across the FFT window (see §6 risk).

---

## §3. The design — port the coherent detector onto OFDM acquisition

### §3.1 Does OFDM structure allow a b328a4d-class detector? YES.

The OFDM preamble is a KNOWN sequence of subcarrier values
(`ofdm_preamble[].value`), so a matched/coherent correlation is directly
applicable and **already implemented** (`time_sync_preamble_fft`). No PHY/tone
redesign is needed (unlike MFSK, which needed a new Costas sequence). The
preamble already has nIdentical_sections=4 redundancy that the autocorrelation
exploits; the FFT detector instead exploits the full known-value content.

### §3.2 The dB-budget math (cited, derived)

**Schmidl-Cox ceiling (the cliff cause).** Schmidl & Cox 1997 derive the metric
mean at correct timing (reproduced at dspillustrations.com, eq. 19):
> **μ = σ_s⁴/(σ_s²+σ_n²)² = 1/(1+1/ρ)²**, ρ = in-band SNR = σ_s²/σ_n².
> σ²(M) ∝ 1/L (eq. 20) — variance shrinks with integration, mean does NOT.

- ρ=−3 dB → μ = 1/(1+2)² = **0.111** ← exactly the prompt's "floors at ~0.11".
- μ = 0.15 (the WB threshold) → ρ = **−2.0 dB** (mean-crossing). The reliable
  detection point sits above this by ~3σ; with L=64 the spread pushes the
  *effective* cliff to the observed SNR3k ≈ +0.9 dB.
- **μ depends only on ρ.** Lengthening the preamble (more symbols, or larger L)
  shrinks σ² (better detection *probability* at fixed SNR) but cannot move the
  μ=threshold SNR floor. This is the SAME length-invariance theorem proved for
  MFSK cosine²-MEAN in `data-frame-cliff-audit §13.9`. **Threshold-lowering and
  preamble-lengthening alone cannot reach the −7.7 dB decode depth.** Confirmed
  by literature: hybrid auto+cross-correlation is the standard low-SNR fix
  (UTD Access-2020; arXiv 2010.00762; IEEE 802.11a uses cross-correlation on
  the long training field for exactly this reason).

**Coherent cross-correlation gain (the fix).** `time_sync_preamble_fft`'s
metric is a coherent sum over Nsymb·(bins/sym) complex terms. Detection SNR
after coherent integration over `Nint = Nsymb · n_bins · Nfft-window` samples
scales as the integration length. For the *known-sequence* matched filter, the
output SNR = input-SNR × (time-bandwidth product of the integration). Going from
the autocorrelation (which self-references half the symbol, effective
integration ~L=64 with the 1/(1+1/ρ)² penalty) to a coherent matched filter over
the full 4 symbols × ~12 bins recovers the matched-filter gain. **Conservative
estimate: one detector swap ≈ +4 to +6 dB** at the detect decision (matches the
b328a4d MFSK precedent: a structurally identical swap moved that cliff ~4 dB).
This wins the SNR3k:0 cell and likely −3 to −4.

**Preamble lengthening (step 2, processing gain).** With the COHERENT detector
(not the autocorrelation), doubling Nsymb 4→8 adds genuine matched-filter
gain. Coherent integration: doubling the integration length → **+3 dB**
detection SNR (signal power ×4, noise variance ×2 in the |Σ|² statistic →
SNR ×2). 4→16 → **+6 dB**. (Caveat: literature warns more repeated parts raise
**sensitivity to frequency selectivity / CFO** — see §6. The coherent sum across
symbols accumulates CFO phase rotation, so the practical gain saturates unless
CFO is corrected first; the existing ±30 Hz coarse-freq search + Moose handles
this, but the coherent window must be validated against residual CFO.)

### §3.3 Staged plan

**STEP 1 — Wire in the coherent FFT detector (the ~4 dB win, wins SNR3k:0).**

- Replace the *WB OFDM* coarse-detect metric: at the primary acquisition site
  (`telecom_system.cc:1322-1422`) and BATCH verify (`:1243`), call
  `time_sync_preamble_fft` (+ `time_sync_preamble_fft_fine` for sample-precise
  alignment) **instead of / in addition to** `time_sync_preamble_halfsym`.
  Keep `time_sync_preamble_halfsym` for NB initially (NB nIS=2 has only 4
  preamble bins/sym — coherent gain is thinner; validate separately).
- **Metric-scale reconciliation (CRITICAL, see §5):** the FFT metric is
  ~Nsymb-scaled (correlation ~4 at preamble, ~1 noise), NOT [0,1]. EITHER
  (a) re-normalize the FFT `correlation` into a [0,1]-comparable quantity so
  the existing 0.15 WB threshold and ALL ARQ consumers of `coarse_metric`
  keep working, OR (b) introduce a separate threshold for the FFT path and
  rescale `coarse_metric` before it reaches ARQ. **(a) is strongly preferred**
  — it confines the change to the detector and leaves the cross-layer contract
  intact. Concretely: normalize `metric` by `(Nsymb² · per-bin-energy)` so a
  clean preamble → ~1.0 and noise → ~1/Nsymb, then the relationship to a
  [0,1] threshold mirrors Schmidl-Cox. Tune the WB threshold to preserve the
  measured high-SNR margin (NOT to mask the cliff — the cliff moves because
  the *metric mean now scales with SNR differently*, not because the threshold
  dropped).
- **dB budget:** +4 to +6 dB at the detect decision. Cliff SNR3k +0.9 → ~−3
  to −4. Wins the 0 dB cell (rank-2 goal) and CONFIG_3 decode depth (−3.7).
- **Fail-before/pass-after test:** §4.

**STEP 2 — Lengthen the OFDM preamble (to reach decode depth −5 to −8).**

- Increase `ofdm_preamble_configurator_Nsymb` 4 → 8 (then evaluate 16) at the
  per-config init (`telecom_system.cc:5026-5089`) — but ONLY after Step 1,
  because lengthening only helps with the COHERENT detector (§3.2).
- **dB budget:** 4→8 ≈ +3 dB; 4→16 ≈ +6 dB (coherent). Combined with Step 1,
  reaches the −7.7 dB CONFIG_0 decode depth.
- **Cost:** per-frame overhead. ROBUST/CONFIG_0 data frame is long (~5 s), so
  4→8 (+4 symbols × ~26 ms ≈ +100 ms) is <2% overhead; for high-rate configs
  (CONFIG_10+) the preamble is a larger fraction — consider **per-config
  preamble length** (long for CONFIG_0-3 where the acquisition cliff bites,
  short for CONFIG_10+ which never operate at the floor). This is a clean
  policy: the configs that need acquisition reach are exactly the low-rate ones.
- **Cross-layer:** `preamble_nSymb` feeds buffer geometry, ARQ timing
  (`message_transmission_time_ms`), drift math. Requires the §5
  `data-flow-preamble_nSymb`-style audit (the MFSK 4→16 change already walked
  this — reuse that audit's consumer list). **Flag-day** (both endpoints
  rebuild; matches MFSK-evolution discipline).

### §3.4 Why not just lower the 0.15 threshold? (rejected)

CLAUDE.md §1.2 forbids threshold-tuning to mask failures, AND §3.2 proves it
cannot work: at −3 dB the Schmidl-Cox mean is 0.11; to detect at −7.7 dB you'd
need threshold ≈ 0.03, which is **below the data/noise false-peak level**
(data symbols hit 0.15-0.45 per the NB comment at `:1124`). The autocorrelation
metric has no headroom there. Only the metric-form change opens reach.

---

## §4. Fail-before / pass-after tests

### §4.1 In-process unit test (no IONOS) — primary gate

Add to `mercury/source/physical_layer/` test suite (mirror the MFSK
`mfsk_data_preamble_argmax_cliff` pattern, `mfsk_ctrl_codec_tests.cc`):

- **`ofdm_preamble_coherent_cliff` (FAIL-BEFORE):**
  1. `load_configuration(CONFIG_0)` (WB BPSK, 4-sym preamble, rate 1/16).
  2. Generate preamble + a known data frame; round-trip
     `baseband_to_passband → passband_to_baseband(FIR_rx_time_sync)`.
  3. Add AWGN at in-band SNR = **−3 dB** (compute per-sample passband noise
     variance from preamble passband energy / target ρ).
  4. Invoke the NEW coherent detector. **Assert** `delay` within ±Ngi of
     injection AND `correlation ≥ threshold`.
  5. **Pre-fix:** the Schmidl-Cox `time_sync_preamble_halfsym` returns
     metric ≈ 0.11 < 0.15 → delay rejected. FAILS. **Post-fix:** coherent
     detector crosses. PASSES. Seed PRNG; ≥4/5 seeds.
- **`ofdm_preamble_coherent_clean` (no high-SNR regression):** sigma=0,
  assert detect + delay precision (passes pre AND post).
- **`ofdm_preamble_coherent_pure_noise` (FAR guard):** pure WGN, 100 seeds,
  assert ≤1 false detect. The FFT metric's data/noise floor (~1 vs ~Nsymb)
  is the discriminator; tune threshold so FAR ≤ ~1e-5/poll.
- **`ofdm_preamble_coherent_data_content` (false-trigger guard):** buffer of
  OFDM DATA symbols (no preamble), assert no detect (the "data ≈ 1" property,
  `ofdm.cc:2696`).
- **`ofdm_preamble_cfo` (CFO robustness, the §6 risk):** inject preamble +
  residual CFO at ±20 Hz (post-coarse-freq residual that Moose handles),
  assert detect still fires. Guards the coherent-sum phase-rotation risk.

### §4.2 Sim cliff sweep — the acquisition self-test the char agent used

Reuse the `[BER-DET]` acquisition self-test path
(`telecom_system.cc:1144-1184`, prints `metric/delay/expected/OK|WARN-lowSNR`
once per config under `ofdm_forced_delay`) driven by
`mercury.exe -m PLOT_PASSBAND -s <config>` at descending injected SNR.

- Define `tools/ofdm_acq_cliff_sweep.py`: for CONFIG_0..3, sweep injected
  in-band SNR from +6 down to −10 dB in 1 dB steps; at each, run PLOT_PASSBAND
  with a forced-delay frame + AWGN and parse `[BER-DET]` to record the lowest
  SNR with `OK` (metric ≥ threshold AND delay within Ngi).
- **Pass criterion:** the acquisition cliff (lowest-OK SNR) moves from
  ~+0.9 dB (baseline) to ≤ −3 dB after Step 1, ≤ −6 dB after Step 2, and stays
  ABOVE the decode depth (−7.7 for CONFIG_0) so acquisition is no longer the
  bottleneck. This is the headline fail-before/pass-after artifact.

### §4.3 Hardware A/B (post-merge, on the deploy checklist — NOT this pass)

IONOS WGN sweep, CONFIG_0-3 pinned, 180 s dwell, baseline-vs-fix arms via
`tools/sack_lossy_ab.py` / `axis_walk_sweep.py`. Expect non-zero data bps at
SNR3k:0 and below where baseline gets 0. Verify arms by branch HEAD + feature
marker grep before trusting JSON labels (MEMORY:
feedback_verify_ab_arms_before_trusting_labels).

---

## §5. Cross-layer (§5) audit — producers / consumers

### §5.1 `receive_stats.coarse_metric` — LOAD-BEARING in ARQ (the key hazard)

Unlike the MFSK `out_metric` (diagnostic-only), the OFDM `coarse_metric` feeds
ARQ flow control. **Producers:** the Schmidl-Cox detect sites write it
(`telecom_system.cc:1252, 1322-1422`, BATCH-verify `:1252`).
**Consumers (flow-control reads — grep `coarse_metric` in `arq_common.cc`):**
- `arq_common.cc:6453` — BREAK-probe gate `coarse_metric < 0.30` (only probe
  for BREAK when no real preamble present). If the new metric's "no preamble"
  value isn't < 0.30, BREAK detection changes.
- `arq_common.cc:6574, 6615` — FTR / batch-active decisions gated
  `coarse_metric >= 0.5`. If a clean preamble no longer reads ≥ 0.5, batch
  logic breaks.
- `arq_common.cc:6074, 6604, 6609` — diagnostic prints (non-load-bearing).
**Invariant the consumers assume:** `coarse_metric ∈ [0,1]`, ~0.9 clean
preamble, < 0.3 no-preamble, ≥ 0.5 confident. **THE FIX MUST PRESERVE THIS
SCALE** (§3.3 option (a): normalize the FFT metric to [0,1] with the same
clean≈1 / noise≈small semantics). If the scale is preserved, every ARQ consumer
keeps working unchanged. If not, all three flow-control sites must be re-derived
— do NOT ship a rescale that depends on consumers being "reasonable".

### §5.2 `noise_variance_estimate` — the SKIP-VAR input

**Producer:** `estimate_noise_from_pilot_pairs` (`ofdm.cc:1475-1518`), called
at `ofdm.cc:1598` after channel estimate. Cross-pilot differential:
`nv = mean over column-pilot pairs of 0.5·|H_raw[i]−H_raw[i−Dy]|²`, floored
1e-6. Units: channel-gain² (since `H_raw=Y/X`); for a clean SGTL5000 frame
nv≈0.01-0.10, garbage 1.7-3.3 (`telecom_system.cc:2519`).
**Consumers of the SKIP-VAR DECISION:**
- `telecom_system.cc:2527` — the gate itself (`nv > 0.5` → skip LDPC).
- `telecom_system.cc:2542` — `receive_stats.frame_skip_var_aborted = true`
  after 3 consecutive SKIP-VAR → signals caller to advance the buffer.
- caller of `frame_skip_var_aborted` (arq layer buffer-shift logic).
**The problem:** for a rate-1/16 config that DECODES at −7.7 dB, the
post-EQ pilot-noise nv at that SNR is ≈ 2-3 (well above 0.5) → the gate bails
before LDPC runs. The 0.5 was calibrated for higher-rate configs where
nv>0.5 genuinely means undecodable. It is **blind to the code rate**: rate
1/16 tolerates far higher nv than rate 14/16.

### §5.3 `preamble_nSymb` / `preamble_configurator.Nsymb` (Step 2 only)

Feeds buffer geometry (`data_container.set_size`, `telecom_system.cc:4407,
4411`), TX length (`:4524`), microphone/speaker buffer sizing (`:5746, 5763`),
ARQ frame-timing math, drift IIR. The MFSK 4→16 change already audited the
parallel field — reuse `data-flow-preamble_nSymb.md` consumer list and extend
for the OFDM field. Changing it is a flag-day (both endpoints).

---

## §6. SKIP-VAR rate-adaptation plan (§4 of the prompt)

**Goal:** relax the fixed `nv > 0.5` gate to a **per-config threshold scaled by
code rate**, WITHOUT re-opening false-acquisition risk.

### §6.1 The principle

A rate-`r` LDPC code decodes down to an Eb/N0 set by its threshold; the
tolerable per-symbol noise variance scales inversely with rate. Rate 1/16
(CONFIG_0) decodes at nv ~2-3; rate 14/16 (CONFIG_16) needs nv < ~0.3. The
single 0.5 constant is therefore correct for mid-rate and wrong (too strict) for
low-rate. **Fix:** `skip_var_threshold(config) = base · (rate_ref / code_rate)`
or a per-config table indexed by `current_configuration`, calibrated so the
threshold sits just above each config's measured decode-fail nv.

### §6.2 Sourcing the per-config value (NO magic numbers — CLAUDE.md §1)

Do NOT hand-pick constants. Derive them: run the existing `[FRAME-NV]` print
(`telecom_system.cc:2566`, logs `nv` + `mvar` + config per trial) in the §4.2
PLOT_PASSBAND cliff sweep, record the nv at the lowest SNR each config still
decodes (CRC pass), set the per-config threshold = that nv × safety margin
(e.g. 1.3×). This makes every threshold value traceable to a measured decode
boundary, satisfying §1 "magic numbers without measurement basis".

### §6.3 §5 audit for SKIP-VAR change

- **Producers** of the threshold: a new per-config lookup (one site) replacing
  the literal 0.5 at `telecom_system.cc:2527`.
- **Consumers:** §5.2 list — only the gate at `:2527` and the
  `frame_skip_var_aborted` 3-strike abort at `:2542`. Relaxing the threshold
  for low-rate configs means **more trials reach LDPC** at low SNR. Verify:
  (a) the 3-consecutive-SKIP-VAR buffer-advance still fires when the signal IS
  pure noise (else RX wastes trials on noise) — keep a hard upper bound (e.g.
  nv > 5.0 always skips regardless of rate, since no config decodes there);
  (b) LDPC iteration budget can absorb the extra low-SNR decode attempts
  (Q3 already raised the cap; profile on Pi).
- **Interaction with mean_H gate** (`:2486`): the mean_H gate fires BEFORE
  SKIP-VAR and is a separate (timing-quality) guard. Relaxing SKIP-VAR does not
  touch mean_H. But confirm a relaxed SKIP-VAR doesn't let bad-timing frames
  (low mean_H) through — mean_H gate still catches those.
- Keep `--skip-var-gate=off` CLI escape intact for validation
  (`telecom_system.cc:45`).

### §6.4 FAR consideration for SKIP-VAR

SKIP-VAR is a *decode* gate, not an *acquisition* gate — a false relaxation
costs wasted LDPC compute (CRC fails), NOT a false ARQ event. So the risk of
relaxing it is bounded (compute, not correctness). The acquisition FAR is owned
by §5.1's detector threshold, which is the load-bearing one.

---

## §7. Risks + FAR budget

1. **Metric-scale collision with ARQ (HIGHEST).** §5.1. Mitigation: normalize
   the FFT metric to [0,1] clean≈1/noise≈small (§3.3a). If impossible, re-derive
   all three `coarse_metric` ARQ consumers. This is the make-or-break
   integration risk — historically (#53) OFDM acquisition is fragile.
2. **CFO sensitivity of coherent integration (HIGH).** Literature (search §1
   results) warns more repeated/coherently-summed parts raise sensitivity to
   frequency offset. The coherent sum across symbols accumulates CFO phase
   rotation across the preamble; at large CFO the |Σ|² collapses. Mitigation:
   the existing ±30 Hz coarse-freq search (`telecom_system.cc:1909-2010`) runs
   the detector at trial carriers — keep it; add the §4.1 `ofdm_preamble_cfo`
   test; if needed, accumulate coherently **within** symbol and non-coherently
   **across** symbols (the FFT detector is already per-symbol FFT — a
   non-coherent-across-symbols variant `Σ_sym |bin_accum_sym|²` is the CFO-robust
   fallback, trading ~1.5 dB for CFO immunity).
3. **FAR budget.** Acquisition false-alarm consequence: spurious delay →
   LDPC on noise → CRC fail → wasted ~30 ms + possible buffer mis-advance.
   The FFT detector's data/noise floor (~1 vs ~Nsymb²) is a strong
   discriminator; target FAR ≤ 1e-5/poll (looser than the autocorrelation is
   fine because the consequence is compute, not a wire event). Validate with
   §4.1 pure-noise + data-content tests. Do NOT loosen below the data-false-peak
   level.
4. **Don't regress the MFSK detector or non-OFDM-data paths.** The change is
   confined to the OFDM (`M != MOD_MFSK`) branch of `receive_msg`. MFSK uses
   `time_sync_mfsk_corr` (untouched). ACK/HAIL/BREAK/CONNECT use
   `detect_ack_pattern` (untouched). Verify by grep that no shared init is
   perturbed.
5. **NB OFDM.** NB nIS=2 → 4 preamble bins/sym; coherent gain is thinner and
   NB already raises threshold to 0.30. Scope Step 1 to **WB first**; evaluate
   NB separately (it may need its own normalization / threshold). Do not assume
   the WB normalization transfers.
6. **Step-2 flag-day + buffer geometry.** Lengthening the preamble touches
   buffer sizing and ARQ timing (§5.3). Do the full data-flow audit before
   changing the constant. Per-config preamble length adds a dimension to the
   geometry math — verify `set_size` and drift handling for the variable.
7. **`time_sync_preamble_fft` may have latent bugs (orphaned code).** It has
   ZERO production exercise, so it has never been hardware-validated. Treat
   STEP 1 as bringing up NEW code: unit-test it hard (§4.1) before trusting it.
   It may need the same `early_exit` / earliest-peak / energy-floor robustness
   the Schmidl-Cox path accreted (`ofdm.cc:2607-2632`) against VB-Cable silence
   and back-to-back frames.

---

## §8. Decision-grade summary (the staged plan)

| Step | Change | dB budget | Wins | Test |
|---|---|---|---|---|
| **1** | Wire in `time_sync_preamble_fft` (+`_fine`) as the WB OFDM coarse detector, normalized to preserve `coarse_metric ∈ [0,1]` ARQ contract | **+4 to +6 dB** at detect decision | SNR3k:0 cell (rank-2 goal); CONFIG_3 reach (−3.7) | §4.1 `ofdm_preamble_coherent_cliff` (FAIL-before at −3 dB) + §4.2 PLOT_PASSBAND cliff sweep |
| **2** | Lengthen OFDM preamble 4→8 (→16), per-config (long for CONFIG_0-3), flag-day | 4→8 **+3 dB**, 4→16 **+6 dB** (coherent only) | Reaches −7.7 decode depth (CONFIG_0) | §4.2 sweep cliff ≤ −6 dB + `data-flow-preamble_nSymb` audit |
| **SKIP-VAR** | Per-config nv threshold scaled by code rate, values measured from `[FRAME-NV]` decode-boundary sweep; hard cap nv>5 always skips | unlocks the decode reach Steps 1-2 expose | low-rate configs decode at nv 2-3 | §6.2 measured table + §6.3 audit (3-strike abort still fires on pure noise) |

**Order:** Step 1 first (biggest win, smallest blast radius, confined to the
detector). Validate it moves the sim cliff to ≤ −3 dB. THEN SKIP-VAR
rate-adaptation (so the freshly-acquired low-SNR frames actually reach LDPC).
THEN Step 2 preamble lengthening (flag-day, needs the geometry audit) to reach
full decode depth. Each step has its own fail-before/pass-after test; each is
independently revertible.

**Prior art (CLAUDE.md §1):** Schmidl & Cox 1997 (metric mean derivation);
IEEE 802.11a (cross-correlation on the long training field — the canonical
auto-then-cross two-stage); hybrid auto+cross-correlation low-SNR papers
(UTD Access 2020 `FrameDetectionWeile`; arXiv 2010.00762 modified
Schmidl-Cox); and the in-tree b328a4d MFSK precedent (the structurally
identical metric-form swap that shipped a measured ~4 dB cliff move).

---

## §9. Source-cite summary

| File:line | What |
|---|---|
| `ofdm.cc:2542-2638` | `time_sync_preamble_halfsym` — current Schmidl-Cox detector (the cliff) |
| `ofdm.cc:2598-2605` | the `M=|P|²/(A²·R)` metric (autocorrelation, length-invariant mean) |
| `ofdm.cc:2640-2682` | `time_sync_preamble_halfsym_2phase` — coarse/fine wrapper |
| `ofdm.cc:2684-2837` | **`time_sync_preamble_fft`** — UNUSED coherent detector (the fix) |
| `ofdm.cc:2839+` | `time_sync_preamble_fft_fine` — unused fine refinement |
| `ofdm.cc:2762-2792, 2811-2835` | coherent bin_accum + Nsymb²-scale normalization |
| `ofdm.cc:1341-1370` | preamble subcarrier configurator (WB nIS=4, ~12 bins/sym) |
| `ofdm.cc:1475-1518` | `estimate_noise_from_pilot_pairs` — nv producer (SKIP-VAR input) |
| `ofdm.cc:1598` | nv assignment site |
| `telecom_system.cc:1110-1486` | OFDM-data RX acquisition flow |
| `telecom_system.cc:1129-1131, 1464-1466` | `preamble_detect_threshold` 0.15 WB / 0.30 NB |
| `telecom_system.cc:1144-1184` | `[BER-DET]` acquisition self-test (the sweep harness) |
| `telecom_system.cc:1243-1256` | BATCH predict-verify (HOT detect call, writes coarse_metric) |
| `telecom_system.cc:1909-2010` | ±30 Hz coarse-freq search (CFO handling, 2 detect calls) |
| `telecom_system.cc:2486` | mean_H gate (separate guard, fires before SKIP-VAR) |
| `telecom_system.cc:2527-2547` | **SKIP-VAR gate** (`nv>0.5`) + 3-strike abort |
| `telecom_system.cc:2566` | `[FRAME-NV]` print (source of measured nv for §6.2) |
| `telecom_system.cc:5026-5089` | `ofdm_preamble_configurator_Nsymb = 4` (all configs) |
| `physical_config.cc:50` | OFDM preamble Nsymb default = 4 |
| `arq_common.cc:6453` | ARQ consumer: BREAK-probe gate `coarse_metric < 0.30` |
| `arq_common.cc:6574, 6615` | ARQ consumers: FTR/batch `coarse_metric >= 0.5` |
| `data-frame-cliff-audit-2026-05-27.md §13.9` | the length-invariance theorem (proven for MFSK) |
| `data-preamble-port-research.md §2.2, §14.1` | b328a4d mechanism + √N gain |

## §10. Open questions [?]

1. [?] Exact normalization constant for the FFT metric to map clean→1.0. Needs
   one PLOT_PASSBAND clean run to measure the raw `metric/energy` at a clean
   preamble for each WB config, then divide. (Do NOT guess — measure.)
2. [?] Does `time_sync_preamble_fft` need the earliest-peak/energy-floor
   robustness the Schmidl-Cox path accreted (VB-Cable silence, back-to-back
   frames, `ofdm.cc:2607-2632`)? Likely yes — port those guards.
3. [?] CFO window: at what residual CFO does the coherent-across-symbols sum
   collapse? Bench with `ofdm_preamble_cfo` test; if < ±20 Hz, switch to
   non-coherent-across-symbols (`Σ_sym|bin_accum_sym|²`) and re-budget (~−1.5 dB).
4. [?] Per-config vs global preamble length (Step 2): does variable
   `preamble_nSymb` break any geometry invariant in `set_size` / drift IIR?
   Audit before implementing.
5. [?] NB OFDM coherent gain with only 4 preamble bins/sym — is Step 1 worth it
   for NB, or keep Schmidl-Cox for NB? Measure separately.

---

## §11. STEP 1 IMPLEMENTATION LOG (2026-06-01)

**Worktree:** `C:/Users/kamer/mercury_wt/ofdm-coherent-acq`, branch
`sim/ofdm-coherent-acq` off monitor `fef293f`. SIM-ONLY (no bench).
Build: `bash build.sh o3` (foreground) → worktree-local `mercury.exe`.

### §11.1 MEASURED raw FFT-detector scale (resolves §10 #1, §10 #2-ortho)

`[MEASURE]` harness `ofdm_coherent_detector_measure_scale`
(`mfsk_ctrl_codec_tests.cc` §22.0) hard-exercises the orphan
`time_sync_preamble_fft` (plan risk #7) on **WB CONFIG_0** (BPSK, rate 1/16,
preamble_nSymb=4) and prints the RAW `correlation` (= `metric/energy`) at
clean / AWGN sweep / pure-noise, averaged over 5 seeds. Result:

| Condition (passband sigma) | FFT-raw mean [min,max] | Schmidl-Cox mean |
|---|---|---|
| clean (sigma=0)             | **3.9999** [4.00,4.00] | 0.9989 |
| 2×RMS                       | 3.771 [3.75,3.82]      | 0.5444 |
| 4×RMS                       | 3.116 [2.92,3.28]      | 0.2352 |
| 8×RMS                       | 2.641 [2.24,3.01]      | 0.1282 |
| 11×RMS (≈ in-band −3 dB)    | **1.774** [1.35,2.27]  | **0.1046** (< 0.15 → **FAILS**) |
| 16×RMS                      | 1.431 [0.99,1.63]      | 0.1101 |
| pure-noise 8×RMS            | **1.058** [0.90,1.20]  | 0.1216 |
| pure-noise 16×RMS           | **1.162** [0.94,1.26]  | 0.1304 |

**Findings:** (1) the orphan detector RUNS without NaN/crash on every case
(risk #7 cleared — it is now exercised by `--test`). (2) Raw scale is exactly
the comment's claim: **clean ≈ Nsymb (4.0), pure-noise ≈ 1.0–1.16 (≈1)**,
preamble-in-noise interpolates monotonically. (3) **Cliff confirmed**: at
in-band ≈ −3 dB (11×RMS) Schmidl-Cox = 0.105 < the 0.15 WB threshold → the
incumbent FAILS; the coherent FFT-raw = 1.77, well above the pure-noise floor
≈ 1.16 → it still discriminates. This IS the +4-6 dB headroom.

### §11.2 The [0,1] re-normalization (resolves §10 #1) — the §5.1 make-or-break

Naive `raw/Nsymb` gives clean=1.0 BUT noise = 1.0/4 = 0.25 — that lands ABOVE
the 0.15 detect threshold and NEAR the 0.30 BREAK gate → would BREAK the
contract. The correct transform subtracts the random-walk floor (≈1) and
normalizes by the clean-minus-floor span:

> **norm = clamp( (raw − 1) / (preamble_nSymb − 1), 0, 1 )**

Validated against the measured table (Nsymb=4 → divide by 3):

| Condition | norm = (raw−1)/3 | vs contract |
|---|---|---|
| clean (raw 4.0)        | **1.000** | ✓ ≥0.97 sub-peak/iter gates (telecom 2469/2706), ≥0.5 confident |
| −3 dB (raw 1.77)       | **0.257** | ✓ ABOVE 0.15 detect → DETECTS (cliff moves); in [0.15,0.30) band = correct "weak but present" |
| pure-noise 8× (1.06)   | **0.020** | ✓ < 0.15 (rejected), < 0.30 (BREAK-probe fires), < 0.5 |
| pure-noise 16× (1.16)  | **0.054** | ✓ < 0.15 |
| 16× preamble (1.43)    | **0.143** | ≈ the true detection limit (just under 0.15) |

This PRESERVES every `coarse_metric` consumer's assumption. Implemented as
`cl_ofdm::normalize_fft_metric(raw, preamble_nSymb)` + thin wrappers
`time_sync_preamble_fft_norm` / `time_sync_preamble_fft_fine_norm` that call
the orphan raw functions UNCHANGED then normalize `.correlation` (so the
raw functions stay byte-identical and the §22.0 measurement stays valid).

### §11.3 §5 cross-layer audit — `receive_stats.coarse_metric` (FULL list)

The plan §5.1 listed 3 consumers; the actual grep (`coarse_metric` across
`source/`) found MORE load-bearing thresholds. COMPLETE list, all assuming
`∈[0,1]`, clean≈0.9-1.0, false/noise<0.15:

**PRODUCERS (write coarse_metric):** `telecom_system.cc:852` (init 0.0),
`:1139` (forced-delay BER, =10.0 sentinel — UNTOUCHED), `:1252` (BATCH verify),
`:1398` (PRIMARY detect), `:1594` (bounds-recovery), `:1764` (silence-recovery),
`:2916` (coarse-freq retry). **Change A rewrites the metric at `:1252` and
`:1398` only** (primary + batch); the recovery/coarse-freq producers stay
Schmidl-Cox and still emit [0,1] — mixing is SAFE because both detectors now
share the clean≈1/noise<0.15 semantics.

**CONSUMERS (read coarse_metric) — verified each still triggers correctly:**
- `telecom_system.cc:1248` BATCH `>= preamble_detect_threshold (0.15)` — clean→1.0 ✓
- `telecom_system.cc:1400` PRIMARY `< threshold (0.15)` skip-demod — noise 0.02-0.05 < 0.15 ✓, −3dB 0.257 ≥ 0.15 passes ✓
- `telecom_system.cc:1590` recovery `>= threshold` — Schmidl-Cox producer, unchanged ✓
- `telecom_system.cc:1669` energy gate `< 0.10` weak-peak skip — noise 0.02-0.05 < 0.10 (skips) ✓, real preamble 1.0 ≥ 0.10 ✓
- `telecom_system.cc:2469` sub-peak reject `>= 0.97 && mean_H<0.5` — clean=1.0 ≥ 0.97 ✓ (gate still able to fire on saturated metric)
- `telecom_system.cc:2706` iteration-cap subpeak `>= 0.97` — clean=1.0 ✓
- `arq_common.cc:6453` BREAK-probe `< 0.30` — OFDM clean 1.0 ≥ 0.30 (correctly suppresses BREAK), noise 0.02-0.05 < 0.30 (allows BREAK probe) ✓
- `arq_common.cc:6574` v2-SACK dispatch `>= 0.5` ("real SACK_RSP ≥0.9") — clean OFDM 1.0 ≥ 0.5 ✓
- `arq_common.cc:6615` beyond-bounds fast-forward `>= 0.5` — clean 1.0 ✓
- `arq_common.cc:6656` batch-FAIL `< 0.5` — clean 1.0 ≥ 0.5 (keeps batch), false <0.5 ✓
- `arq_common.cc:6684-6685` medium band `>= 0.15 && < 0.5` — preamble-in-noise at the cliff (0.257) lands here = the intended "weak preamble, zero & retry" behavior ✓
- diagnostic prints (`:6074,6604,6609,6648,6790`, telecom `:1421,:1472,:1658,:1850,:2956`) — non-load-bearing ✓

**INVARIANT PRESERVED.** No consumer needs re-derivation. The normalization
maps clean→1.0 (satisfies the ≥0.97 saturation gates), noise→~0.02-0.05
(under every low threshold), cliff-preamble→0.257 (in the [0.15,0.30) weak
band — semantically correct: "present but not confident").

### §11.4 Wiring scope + WHY minimal (resolves §10 #2 partially)

The acquisition state machine (`receive_msg`, OFDM branch) is layered: the
PRIMARY coarse detect only needs **symbol-level** accuracy to set
`pream_symb_loc`; **site 8 (`telecom_system.cc:2077`,
`time_sync_preamble_with_metric`) re-refines `receive_stats.delay` to full-rate
sample precision** from raw passband around `pream_symb_loc`. So the primary
detector's precision requirement is only "find the right symbol + GI step",
which the decimated FFT detector meets. Therefore Change A wires the FFT
detector at exactly TWO sites, both on the DECIMATED buffer (interp=1, same
buffer the incumbent decimated Schmidl-Cox coarse already uses):

- **PRIMARY** (`telecom_system.cc` ~1320-1398): replace the decimated
  Schmidl-Cox 2-phase (coarse `:1323` + FIR-slice fine `:1390`) with a single
  `time_sync_preamble_fft_norm` (coarse) + `time_sync_preamble_fft_fine_norm`
  (sample-precise on decimated grid) call. Delay mapped ×interp to full-rate.
- **BATCH verify** (`:1243`): replace the small-window decimated Schmidl-Cox
  with `time_sync_preamble_fft_norm` on the verify window.

Recovery (`:1561`, `:1734`), coarse-freq ±30 Hz (`:1950`, `:2005`), the BER
self-test (`:1171`), and site-8 fine-sync stay Schmidl-Cox / with_metric
(UNCHANGED). WB-only: gated `!narrowband_enabled` (NB nIS=2, 4 bins/sym — plan
§5 risk #5, deferred). MFSK path (`M==MOD_MFSK`) untouched.

### §11.5 The mode-0 → mode-1 pivot (CFO + FAR drove it)

The orphan detector as written (mode 0: per-bin COHERENT ACROSS SYMBOLS,
`Σ_b|Σ_sym fft·conj(pre)|²`) FAILED two of the §4.1 guards — MEASURED:

- **CFO collapse** (`ofdm_coherent_cfo`, mode 0): 0/6 detect at ±20 Hz. The CFO
  sweep showed raw metric 3.23 (0 Hz) → 1.36 (5 Hz) → 0.74 (8 Hz, BELOW the
  noise floor) → it tolerates only ≈±3-4 Hz residual. The across-symbol
  coherent sum accumulates ~170° of CFO phase over the 4-symbol span.
- **Fat pure-noise tail**: 7-10/100 noise buffers crossed the 0.15 detect
  threshold; noise MAX norm ≈ 0.33 (raw 1.99).

**Fix (plan §6 risk #2): combine_mode 1** = coherent across BINS within a symbol,
NON-coherent across symbols (`Σ_sym|Σ_b fft·conj(pre)|²`). The bins of one symbol
share that symbol's timing phase (coherent gain survives energy-normalization:
clean ≈ n_bins ≈ 12, noise ≈ 1), but symbols are summed in POWER so
symbol-to-symbol CFO drift no longer collapses the sum. Implemented as a
`combine_mode` param (default 0 preserves the orphan's exact original metric
byte-for-byte; the `_norm` production wrappers pass 1). Result:
**`ofdm_coherent_cfo` ±20 Hz → 6/6** ✓.

A naive fully-non-coherent variant (both axes) was tried and REJECTED: dividing
per-bin `|fft·conj(pre)|²` by per-bin `|fft|²` cancels the signal power → no
discrimination after energy-normalization. Coherence must be retained on ONE
axis; bins (timing-robust at the GI-stride coarse grid) is the right one.

### §11.6 Measured-clean-ref calibration (resolves §10 #1, the ≥0.97 gates)

`normalize_fft_metric(raw, clean_ref)` maps `(raw−1)/(clean_ref−1)` clamped to
[0,1]. clean_ref is the MEASURED clean raw, calibrated once per config switch in
`load_configuration` (WB OFDM only): synthesize the preamble through the SAME
decimated round-trip the production detector sees and cache the raw mode-1
metric in `ofdm.fft_clean_ref` (≈ **11.14** for WB CONFIG_0). Falls back to the
theoretical bins/symbol if uncalibrated. **Two bugs found+fixed during bring-up
(both = decimation mismatch):** (1) the calibration first fed a FULL-rate buffer
with interp=1 → grabbed Nfft consecutive full-rate samples → clean_ref≈1.25 (≈
noise); fixed by decimating (decimation_rate=interp) exactly as production's
`baseband_data_decimated`. (2) the §22 test harness `synth_ofdm_preamble_buffer`
likewise had to decimate + call the detector with interp=1 to MATCH production
(it had used full-rate+interp). With the correct calibration, **clean → 1.0**
(`ofdm_coherent_clean` ✓), preserving the telecom_system.cc:2469/2706 ≥0.97
sub-peak/iteration gates.

### §11.7 RESIDUAL FAR (the honest Step-1 caveat) — fundamental, downstream-gated

At the −3 dB cliff with a 4-symbol preamble, MEASURED: cliff-signal mean norm
≈ 0.17-0.21 OVERLAPS the pure-noise tail (worst-of-100 norm ≈ 0.22). So **no
threshold can both detect at −3 dB AND give near-zero FAR with only 4 preamble
symbols** — a near-zero-FAR gate would re-wall the cliff. This is a DELIBERATE
trade (plan §7.3): Schmidl-Cox has ≈0 FAR but cliffs at +0.9 dB; the coherent
detector reaches −3 dB at the cost of a **~5.5% pure-noise acquisition FAR**
(`ofdm_coherent_pure_noise`: mean norm 0.067 — typical poll safe; FAR 11/200 at
≥0.15). A false acquisition is NOT a wire event: it yields a random channel →
the DOWNSTREAM mean_H gate (`telecom_system.cc:2487`, <0.30 → SKIP-H) and
SKIP-VAR reject it before LDPC. Cost = one cheap channel-estimate + gate.
**Step 2 (lengthen the preamble) is what buys the FAR margin** (more integration
separates signal from the noise tail). `ofdm_coherent_pure_noise` asserts the
defensible properties (mean ≪ 0.15; FAR bounded ≤ 10%), NOT a near-zero gate.

### §11.7b Periodicity early-lock (characterized; bounded; site-8-corrected)

The OFDM preamble repeats the SAME Zadoff-Chu sequence every symbol
(`ofdm.cc:1304`), so the coherent detector cannot distinguish the true start
from a position k<Nsymb symbols earlier — they alias to nearly-equal metric. In
a buffer with LEADING content (prior frames/silence — the real case), the
earliest-above-50% selection locks UP TO Nsymb symbols EARLY (MEASURED via a
leading-context probe: 2-sym lead → coarse+fine lock 2 syms early; metric stays
0.85-1.0). This is the SAME ambiguity class Schmidl-Cox has; production handles
it via **site 8** (`time_sync_preamble_with_metric`, telecom_system.cc ~:2090)
which re-refines the FULL-RATE demod delay from a `(preamble_nSymb+4)`-symbol
window around `pream_symb_loc` — wide enough to recover a ≤Nsymb-early lock. The
aliased metric (≥0.85) still satisfies every load-bearing ARQ threshold (≥0.5
for FTR/batch/SACK, ≥0.30-suppress for BREAK); only the ≥0.97 sub-peak gates may
not fire on an aliased lock — benign (they guard a Schmidl-Cox-specific
pathology). `ofdm_coherent_leading_context` (§22.7) PINS this bounded behavior.
**[?] The full acquisition-state-machine interaction (bounds check + site-8
recovery) under leading context is the one piece SIM cannot fully exercise
in-process — flagged for the §4.3 hardware A/B (deploy checklist).**

### §11.8 Change B SKIP-VAR — MEASURED per-config thresholds

`[FRAME-NV]`/BER decode-boundary sweep (`§23.0`, skip-var OFF, FER<1 onset),
WB, MEASURED:

| Config | rate | decode-boundary Es/N0 | boundary nv | threshold (×1.3) | old 0.5 gate |
|---|---|---|---|---|---|
| CONFIG_0 | 1/16 | −3 dB | 1.430 | **1.86** | WOULD SKIP |
| CONFIG_1 | 2/16 | −1 dB | 1.234 | **1.60** | WOULD SKIP |
| CONFIG_2 | 3/16 | 0 dB | 1.116 | **1.45** | WOULD SKIP |
| CONFIG_3 | 4/16 | +1 dB | 1.052 | **1.37** | WOULD SKIP |
| CONFIG_10 | 6/16 | +8 dB | 0.348 | 0.5 (unchanged) | ok |

`skip_var_threshold()` returns these for WB CONFIG_0-3, **0.5 for everything
else AND all of NB** (non-WB-OFDM byte-identical — NB acquisition is unchanged
Schmidl-Cox so it can't reach the deep SNR the relaxation targets). Hard cap at
the gate: `nv > thr || nv > 5.0` (no config decodes above 5; bounds the
relaxation, keeps the 3-strike pure-noise abort firing). Validated:
`skip_var_low_rate_decodes_at_boundary` — **CONFIG_0 @ −3 dB DECODES with the
production gate ON (FER=0.17, nv=1.467 > old 0.5 → old gate skipped it, <
new 1.86 → new gate admits it).** Fail-before/pass-after.

### §11.9 Integration sweep + headline result

`tools/ofdm_acq_cliff_sweep.py` (PLOT_PASSBAND `--ber-esn0` per point,
parses `[BER-DET]` vs new `[BER-DET-FFT]`). CONFIG_0/3, Es/N0 +8→−10:
**coherent FFT metric clears the 0.15 threshold down to −10 dB Es/N0 where
Schmidl-Cox metric+delay cliffs at −4 dB** (≥6 dB reach gain; exceeds the
+0.9→≤−3 Step-1 target). NOTE: the self-test window has leading context, so the
`[BER-DET-FFT]` delay-OK column is contaminated by the §11.7b periodicity alias
(metric strong, delay flagged) — the tool reports a delay-agnostic
"FFT-reach" column for the true acquisition reach. The CLEANEST
fail-before/pass-after artifact is the in-process `ofdm_coherent_cliff`
(production-matching offset-0 synth): **Schmidl-Cox 4/5 FAIL (<0.15) vs coherent
5/5 PASS (≥0.15, delay within ±1 symbol) at in-band −3 dB.**

### §11.10 dB gain vs the +4-6 expected; surprises

- **Detect-reach gain ≈ +6 dB** (sweep: −4 → −10 dB Es/N0 metric-reach), at or
  above the plan's +4-6 dB. The cliff move EXCEEDS the +0.9 → ≤−3 Step-1 target.
- **Surprises (all MEASURED, none fatal):** (1) the orphan mode-0 detector is
  CFO-fragile (collapses by 5 Hz) and has a fat noise tail — required the mode-1
  pivot (plan-sanctioned §6 risk #2). (2) Two decimation-mismatch bugs in the
  clean-ref calibration / test harness (clean_ref 1.25 → 11.14). (3) Residual
  ~5.5% pure-noise FAR at −3 dB is FUNDAMENTAL to a 4-symbol preamble (Step 2
  needed for low FAR) — downstream-gated, not a wire event. (4) Periodicity
  early-lock under leading context (identical ZC symbols) — bounded,
  site-8-corrected, ≥0.97 gates go inactive (benign).
- **`mercury.exe --test`: 61/61 pass** (50 baseline + 8 §22 Change A + 3 §23
  Change B). MFSK + NB OFDM paths byte-identical (verified by diff: all
  acquisition changes `!narrowband_enabled`-gated; SKIP-VAR NB-gated to 0.5;
  mode-0 default preserves the orphan metric exactly).
- **SIM ONLY.** Hardware A/B (plan §4.3) remains on the deploy checklist —
  especially to confirm the §11.7b acquisition-state-machine interaction.

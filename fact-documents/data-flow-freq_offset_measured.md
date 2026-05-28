# Data-Flow Audit: `freq_offset_measured` (MFSK mini-Moose)

**Status**: Authoritative as of 2026-05-28, written BEFORE the
mini-Moose CFO refinement lands on `feat/mini-moose`. Every future
change to the MFSK / OFDM frequency-sync flow that touches this state
MUST update this document.

**Driving work item**: `data-preamble-port-research.md` §19/§20. The
MFSK data-preamble path currently has NO fine CFO sync — the line
`telecom_system.cc:2193` (`if(M == MOD_MFSK) freq_offset_measured = 0;`)
explicitly zeros the offset measured by the WB/NB Moose estimators in
the OFDM-only fork. §17 hardware A/B showed mirror-bin acceptance is
load-bearing on IONOS WB → residual CFO is leaking signal energy. This
fix produces a residual-CFO estimate on the MFSK data preamble and
applies it via the existing OFDM re-mix infrastructure.

**Cross-layer mandate**: CLAUDE.md §5 "Cross-Layer Data-Flow Audits".
Mercury's bug history (retx-queue, Phase B Wave 2 v1) shows that
silently changing one branch's behavior of a shared local variable
shipped sibling bugs within hours. This audit walks every producer and
consumer of `freq_offset_measured` BEFORE the fix lands.

---

## §1 Producers — code paths that write `freq_offset_measured`

`freq_offset_measured` is a function-scope local declared at
`source/physical_layer/telecom_system.cc:810` inside
`cl_telecom_system::receive_msg`. Lifetime is one call to `receive_msg`;
state does not persist across calls (with the exception of the cache
write at §2.5 below). Per-call producers, in source order:

### §1.1 Initial value at function entry
- **File:line**: `telecom_system.cc:810`
- **Write**: `double freq_offset_measured = 0;`
- **Reach**: every call to `receive_msg`.
- **Note**: This is the safe-default state. If any producer below is
  skipped, the function continues with 0 (treated as "no frequency
  correction needed" by all consumers).

### §1.2 BER test forced-delay branch
- **File:line**: `telecom_system.cc:2161-2163`
- **Write**: `freq_offset_measured = 0;`
- **Reach**: when `ofdm_forced_delay >= 0` (BER test path).
  Reach is rare (test mode only).
- **Rationale**: BER test injects known signal at zero offset; skip
  estimator to avoid garbage estimate corrupting the BER vs SNR curve.
- **Touched by fix?** NO — the mini-Moose code does NOT run when
  `ofdm_forced_delay >= 0`. BER tests stay byte-identical.

### §1.3 Cached "last good" reuse on sync exhaustion
- **File:line**: `telecom_system.cc:2164-2167`
- **Write**: `freq_offset_measured = receive_stats.freq_offset_of_last_decoded_message;`
- **Reach**: when `receive_stats.sync_trials == effective_trials_max
  && use_last_good_freq_offset == YES &&
  receive_stats.freq_offset_of_last_decoded_message != 0`.
  Reach is "final retry" — after all primary trials fail, fall back to
  last known good value.
- **Source**: cache is only written for OFDM (see §2.5). For MFSK the
  cache field stays at its previous value (0 if no OFDM frame has been
  decoded since startup; or the last OFDM Moose result if there was one).
- **Touched by fix?** NO — this branch only fires on the final retry
  AND only when the cache is non-zero AND only when `use_last_good_freq_offset`
  is enabled. The cache is OFDM-only by §2.5 invariant. For MFSK, the
  primary-trial path runs the mini-Moose (§1.6); only if all primary
  trials fail AND a prior OFDM run wrote a cache value does this branch
  fire.

### §1.4 NB-OFDM branch
- **File:line**: `telecom_system.cc:2168-2180`
- **Write**: `freq_offset_measured = 0;`
- **Reach**: when `narrowband_enabled == true`. Reach includes both
  NB-OFDM (CONFIG_0..16 + `-N`) and NB-MFSK (ROBUST_0..2 + `-N`).
- **Rationale**: existing NB design choice — both Moose and
  `carrier_frequency_sync_nb` are unreliable at NB (Moose loses
  Schmidl-Cox repetition assumption, NB-NB gave -17 Hz on zero-offset
  channel after Bug #41 preamble change). NB-OFDM uses ZF estimator's
  residual phase-rotation tolerance.
- **Touched by fix?** NO — NB-MFSK explicitly out of scope per §19.4
  risk register. The §1.6 producer is gated `!narrowband_enabled &&
  M == MOD_MFSK` to preserve this branch's behavior for NB MFSK.

### §1.5 WB-OFDM Moose
- **File:line**: `telecom_system.cc:2181-2191`
- **Write**: `freq_offset_measured =
  ofdm.carrier_sampling_frequency_sync(...)`. Nfft/4 (nIS=4) half-symbol
  Moose using the every-other-subcarrier preamble structure.
- **Reach**: when `!narrowband_enabled && M != MOD_MFSK`. WB OFDM only.
- **Capture range**: ±2 subcarrier spacings = ±93.75 Hz.
- **Touched by fix?** NO — WB-OFDM behavior preserved. The mini-Moose
  estimator does NOT replace this branch; it adds a NEW branch for
  WB MFSK.

### §1.6 MFSK override — PRE-FIX
- **File:line**: `telecom_system.cc:2193`
- **Write**: `if(M == MOD_MFSK) freq_offset_measured = 0;`
- **Reach**: every MFSK call (NB + WB).
- **Behavior**: discards anything the §1.4/§1.5 branches wrote. (The
  §1.4 NB branch already writes 0, so the override is a no-op there.
  For MFSK WB the override is a hard reset to 0.)

### §1.6 MFSK override — POST-FIX
- **File:line**: `telecom_system.cc:2193` (the line is replaced, the
  semantics differ by mode).
- **Write**: for WB MFSK: `freq_offset_measured =
  ofdm.carrier_frequency_sync_wb_mfsk(&data_container.baseband_data[0],
  bandwidth/(double)data_container.Nc, data_container.preamble_nSymb,
  mfsk.preamble_tones, mfsk.M, mfsk.nStreams, mfsk.stream_offsets,
  sampling_frequency / data_container.interpolation_rate);`
- **For NB MFSK**: `freq_offset_measured = 0;` (NB MFSK unchanged).
- **Reach**: every MFSK call. WB-MFSK branch produces a measured
  residual; NB-MFSK branch produces 0.
- **Confidence gate**: `carrier_frequency_sync_wb_mfsk` returns 0 if
  internal `|C|/energy_total < 0.05` (mirrors `carrier_frequency_sync_nb`
  at `ofdm.cc:600`).
- **Magnitude bound**: returned value is mathematically bounded by the
  half-symbol baseline capture range, ±11.7 Hz at WB ROBUST_0.
  (Implementation note: the `arg(C)` wraps at ±π, so estimates
  artificially "fold back" inside ±capture-range. Out-of-range true
  CFO maps to an in-range alias. The §2.1 sanity clamp accepts up to
  ±93.75 Hz — well outside the alias band; alias artifacts are inside
  the clamp window, treated as legitimate estimates by downstream
  consumers.)

### §1.7 Sanity clamp + cap
- **File:line**: `telecom_system.cc:2214-2217`
- **Write**: `freq_offset_measured = ±max_correction` (clipped to
  ±1 subcarrier spacing).
- **Reach**: after any of §1.2-§1.6, when no `continue` (REJECT) fires.
- **Touched by fix?** NO — same clamp covers MFSK now. Post-fix
  MFSK values are bounded ±11.7 Hz, well under the ±1-subcarrier
  cap (±46.875 Hz). Clamp is unreachable in MFSK post-fix unless the
  half-symbol alias wraps into out-of-range territory — and even then,
  clamping to ±46.875 Hz is safer than the pre-fix hard-zero.

---

## §2 Consumers — code paths that read `freq_offset_measured`

In source order:

### §2.1 Sanity reject / clamp gate
- **File:line**: `telecom_system.cc:2204-2218`
- **Read**: `fabs(freq_offset_measured) > moose_sanity_limit` (2 ×
  subcarrier_spacing ≈ 93.75 Hz). If exceeded AND
  `sync_trials < effective_trials_max`: print `[MOOSE-REJECT]`,
  `sync_trials++`, `continue` to next sync trial.
- **Pre-fix MFSK behavior**: `freq_offset_measured = 0`, sanity check
  always passes. MFSK never enters the reject path.
- **Post-fix MFSK behavior**: estimator returns a value in ±11.7 Hz
  (alias-folded); sanity check still passes. MFSK still never enters
  the reject path in normal operation. If the estimator returns a
  pathological value due to NaN propagation (e.g. all-zero input
  with `energy_total == 0`), the gate at `ofdm.cc:600` returns 0 —
  again, sanity check passes.
- **Invariant**: post-fix, the sanity reject path is exercisable on
  MFSK only if a bug in the new estimator returns a non-finite or
  wild value. The internal confidence gate guarantees finite output.

### §2.2 Apply correction — re-mix path — PRE-FIX
- **File:line**: `telecom_system.cc:2220-2244`
- **Read**: `fabs(freq_offset_measured) > ofdm.freq_offset_ignore_limit`
  (≈ 3 Hz for WB).
- **Pre-fix logic**:
  - `if(M == MOD_MFSK) { /* skip — no channel estimation to
    compensate */ }`
  - `else if (above ignore limit)` → re-mix from passband at
    `effective_carrier_freq + freq_offset_measured`, overwrite
    `baseband_data` with the corrected frame.
- **Pre-fix MFSK behavior**: NEVER re-mixes. `baseband_data` is the
  coarse-sync version (`carrier_frequency + coarse_freq_offset`).

### §2.2 Apply correction — re-mix path — POST-FIX
- **File:line**: `telecom_system.cc:2220-2244`
- **Post-fix logic**:
  - DROP the MFSK skip at 2220-2223.
  - `if (fabs(freq_offset_measured) > ofdm.freq_offset_ignore_limit)`
    → re-mix from passband. Applies to BOTH MFSK and OFDM.
- **Post-fix MFSK behavior**: re-mixes when the mini-Moose estimate
  exceeds `freq_offset_ignore_limit` (≈ 3 Hz). For estimates below
  that, no-op (baseband_data unchanged).
- **Consumer assumption verification**: the re-mix block reads:
  - `data` (passband input buffer)
  - `pb_start`, `pb_size` (extraction window — computed at lines
    2121-2136 from `extraction_delay`, identical for MFSK and OFDM).
  - `effective_carrier_freq` (= `carrier_frequency + coarse_freq_offset`,
    computed at 2117).
  - `sampling_frequency`, `carrier_amplitude` (unchanged).
  - `Mdec` = `data_container.interpolation_rate` (unchanged).
  - `&ofdm.FIR_rx_data` (the same data-decode FIR used at 2149).
  - `pb_start` for the polyphase offset (unchanged).

  All inputs are valid for MFSK. The MFSK skip pre-fix was deliberate
  "no channel estimation, no benefit" — but the mini-Moose's purpose
  IS to provide a correction that benefits the FFT-energy demap (by
  shifting bin alignment back to expected). Skip semantics no longer
  apply.

### §2.3 OFDM-OK log line
- **File:line**: `telecom_system.cc:2706-2713`
- **Read**: print `freq_offset_measured` in the `[OFDM-OK]` line.
- **Reach**: gated `if (M != MOD_MFSK)` — MFSK doesn't print this line.
- **Touched by fix?** NO — gate unchanged. MFSK doesn't reach this log.

### §2.4 OFDM-FAIL log line
- **File:line**: `telecom_system.cc:2608-2611` (approx, in the OFDM
  fail branch within the trial loop).
- **Read**: prints `freq_offset_measured` in the `[OFDM-FAIL]` line.
- **Reach**: same OFDM-only gate as §2.3.
- **Touched by fix?** NO.

### §2.5 Cache write — `freq_offset_of_last_decoded_message`
- **File:line**: `telecom_system.cc:2742-2745`
- **Read**: `receive_stats.freq_offset_of_last_decoded_message =
  freq_offset_measured;` AND `receive_stats.freq_offset = freq_offset_measured;`
- **Reach**: gated `if (M != MOD_MFSK)`. MFSK never writes to either cache.
- **Importance**: this gate is THE protection against MFSK residuals
  leaking into the OFDM `use_last_good_freq_offset` fallback (§1.3).
- **Touched by fix?** NO — gate preserved verbatim. The mini-Moose's
  output never enters the cache. (If a future fix wants to cache MFSK
  residuals, it must coordinate with the §1.3 consumer — a single
  cache field shared across modes would re-introduce the bug this
  gate prevents.)

### §2.6 Stream / SACK / ARQ consumers
Grep audit confirms: NO ARQ, SACK, streaming, or Q-table state reads
`freq_offset_measured` or `freq_offset_of_last_decoded_message`. The
field is PHY-only.

### §2.7 GUI / live constellation
- **File:line**: `telecom_system.cc:2735` GUI push.
- **Read**: indirect — the constellation comes from `ofdm_deframed_data`,
  which only exists for OFDM (`M != MOD_MFSK`). MFSK pushes a "clear"
  signal (`gui_push_constellation(nullptr, 0, M, true)`).
- **Touched by fix?** NO. The MFSK path doesn't expose constellation
  to the GUI.

---

## §3 Valid states

| State | freq_offset_measured (post-§1.6) | Sanity gate (§2.1) | Re-mix (§2.2) | Cache (§2.5) |
|---|---|---|---|---|
| Function entry | 0 | passes | skipped (below ignore_limit) | n/a yet |
| BER test (ofdm_forced_delay≥0) | 0 (§1.2) | passes | skipped | n/a |
| Last-good reuse (sync_trials==max, cache set) | cached value | depends | depends | not re-written |
| NB-OFDM | 0 (§1.4) | passes | skipped | written (=0) |
| WB-OFDM | Moose result (±93.75 Hz capped) | conditional reject | conditional re-mix | written |
| NB-MFSK (pre-fix) | 0 (§1.6) | passes | skipped (MFSK branch) | NOT written (§2.5 gate) |
| NB-MFSK (post-fix) | 0 (§1.6 NB branch returns 0) | passes | skipped (below ignore_limit) | NOT written (§2.5 gate) |
| WB-MFSK (pre-fix) | 0 (§1.6) | passes | skipped (MFSK branch) | NOT written |
| WB-MFSK (post-fix) — typical (residual ≤ 11.7 Hz) | mini-Moose estimate | passes | conditional re-mix | NOT written |
| WB-MFSK (post-fix) — low-confidence input | 0 (estimator confidence gate) | passes | skipped | NOT written |
| WB-MFSK (post-fix) — pathological (NaN) | 0 (NaN propagation guarded) | passes | skipped | NOT written |

---

## §4 Invariants (post-fix)

### INV-1: OFDM behavior unchanged

For every `M != MOD_MFSK` config, `freq_offset_measured` flows
through §1.2/§1.3/§1.4/§1.5 (BER / cache / NB-zero / WB-Moose)
identically to pre-fix. Sanity gate (§2.1) and re-mix (§2.2) consumer
behavior is identical for OFDM. Cache write (§2.5) is identical.
Verified by inspection: the fix touches `telecom_system.cc:2193`
(MFSK-only line) and `telecom_system.cc:2220-2223` (the
`if(M == MOD_MFSK) skip` block); no other site is modified.

### INV-2: NB-MFSK behavior unchanged

For `narrowband_enabled && M == MOD_MFSK`, the post-fix §1.6 producer
takes the `narrowband_enabled` branch and writes 0. Identical to
pre-fix. Verified by gating in the mini-Moose dispatch:
`if(!narrowband_enabled && M == MOD_MFSK) freq_offset_measured =
ofdm.carrier_frequency_sync_wb_mfsk(...); else if(M == MOD_MFSK)
freq_offset_measured = 0;`

### INV-3: Cache write gate preserved

`telecom_system.cc:2742` (`if(M != MOD_MFSK)`) is NOT modified. MFSK
residuals never enter `receive_stats.freq_offset_of_last_decoded_message`.
The §1.3 last-good-reuse path is therefore safe — only previously-cached
OFDM Moose values can be reused, never MFSK mini-Moose values.

### INV-4: Mini-Moose output is finite

`carrier_frequency_sync_wb_mfsk` returns a finite `double` for all
inputs. NaN guard: when `energy_total < 1e-10`, return 0 (mirror of
`carrier_frequency_sync_nb` at `ofdm.cc:600`). When `|C|` underflows
to 0, `arg(C)` returns 0, scale factor is finite, return is 0. Result
is bounded: |return| ≤ π * carrier_freq_width * Nfft / (2π * Nofdm/2)
= carrier_freq_width * Nfft / Nofdm (half-symbol baseline). For
WB ROBUST_0: 46.875 × 256 / 292 ≈ 41.1 Hz absolute bound; typical
range ±11.7 Hz with confidence-gate suppression of low-quality estimates.

### INV-5: Sanity clamp still bites pathological cases

If a future bug in `carrier_frequency_sync_wb_mfsk` returns a value
outside ±93.75 Hz, the sanity gate at `:2205` triggers REJECT, advances
`sync_trials`, and retries. Identical safety to today's WB OFDM Moose.
The reject path does NOT mark the frame as decoded; data integrity
preserved.

### INV-6: Re-mix consumer sees same buffer shape

The MFSK re-mix call in the post-fix §2.2 path uses the same arguments
as the OFDM re-mix at `:2233-2241`: `data + pb_start`, `pb_size`,
`baseband_data_interpolated`, `sampling_frequency`,
`effective_carrier_freq + freq_offset_measured`, `carrier_amplitude`,
`Mdec`, `&ofdm.FIR_rx_data`, `pb_start`. Buffer shape (pb_size = Nofdm
× (Nsymb + preamble_nSymb) × interp + 2×fir_margin) is identical for
MFSK and OFDM at the same Nsymb / preamble_nSymb / interp settings.
The subsequent copy loop (`for i=0..frame_dec; baseband_data[i] =
baseband_data_interpolated[margin_dec + i]`) is identical too.

---

## §5 What the fix changes — per-consumer walk

Fix scope:

1. `source/physical_layer/ofdm.cc`: add `cl_ofdm::carrier_frequency_sync_wb_mfsk(...)`.
   New function, no existing call sites.
2. `include/physical_layer/ofdm.h`: add declaration.
3. `source/physical_layer/telecom_system.cc:2193`: replace
   `if(M == MOD_MFSK) freq_offset_measured = 0;` with the mini-Moose
   dispatch (WB-MFSK estimator; NB-MFSK still zero).
4. `source/physical_layer/telecom_system.cc:2220-2223`: DELETE the
   `if(M == MOD_MFSK) { /* skip */ }` block, falling through to the
   `else if(fabs(freq_offset_measured) > ofdm.freq_offset_ignore_limit)`
   branch.

Per-consumer impact verification (the §2 audit row by row):

- §2.1 sanity gate: still bounds output to ±93.75 Hz. MFSK now
  exercises this path under normal operation but post-INV-4 the
  estimator output stays within ±11.7 Hz typical / ±41.1 Hz absolute
  → never triggers REJECT. Safe.
- §2.2 re-mix: post-fix MFSK runs the OFDM-shape re-mix block. Per
  INV-6, buffer math identical. Safe.
- §2.3-§2.4 logs: OFDM-only gates preserved. Unchanged.
- §2.5 cache: gate preserved (INV-3). MFSK residuals don't leak into
  OFDM gearshift. Safe.
- §2.6 ARQ etc.: no consumers exist. Safe by inspection.
- §2.7 GUI: MFSK gate preserved. Unchanged.

---

## §6 Test-mode pre-init invariants (CLAUDE.md §5 "uncommon paths")

The new estimator `carrier_frequency_sync_wb_mfsk` reads:
- `Nfft`, `Ngi` (set by `load_configuration` → 256, 36 for WB).
- `mfsk_preamble_tones[]`, `mfsk_M`, `mfsk_nStreams`,
  `mfsk_stream_offsets[]` (passed as parameters from the caller).
- The input baseband buffer (caller's responsibility — must contain
  `preamble_nSymb` symbol slots at indices `0..preamble_nSymb*Nofdm`).

Pre-init guard: the new function checks `if (preamble_nSymb <= 0 ||
mfsk_M <= 0 || mfsk_nStreams <= 0) return 0.0;` — identical to the
guard at `ofdm.cc:3046` for `time_sync_mfsk_corr`. Pre-init test
paths that call the estimator before `load_configuration` will see
the default-init `0`s and the early return. Safe.

Test paths in `mfsk_ctrl_codec_tests.cc` that construct
`cl_telecom_system` always call `load_configuration(ROBUST_0)` before
testing — the §6 (mfsk_data_preamble_argmax_*) tests already establish
this convention. The new §7 (mini-moose) tests follow it.

---

## §7 Cross-layer regression test plan

Three tests added to `mfsk_ctrl_codec_tests.cc` §7, hooked into
`run_mfsk_ctrl_codec_tests()`:

### Test A — `mfsk_data_preamble_mini_moose_recovers_cfo`
- POSITIVE, FAIL-BEFORE-PASSES.
- Synthesize preamble + AWGN at sigma_pb = 3 × preamble_rms.
- Inject CFO_in = +7 Hz on the passband BEFORE `passband_to_baseband`.
- Run the new estimator on the post-FIR baseband; assert
  |estimated_CFO - 7 Hz| < 1.5 Hz over 5 PRNG seeds.

### Test B — `mfsk_data_preamble_mini_moose_zero_cfo_no_op`
- POSITIVE, NO-REGRESSION guard.
- Clean preamble (sigma_pb = 0, CFO_in = 0).
- Run the estimator; assert |estimated| < 0.5 Hz.
- The full demod round-trip with mini-Moose enabled produces matched =
  preamble_nSymb (just like the existing
  `mfsk_data_preamble_argmax_high_snr_no_regression`). Verifies the
  estimator doesn't disturb the clean-signal path.

### Test C — `mfsk_data_preamble_mini_moose_pure_noise_safe`
- NEGATIVE, CONFIDENCE-GATE / sanity guard.
- Pure WGN buffer (no preamble), sigma_pb non-zero.
- Run the estimator; assert returns a value bounded by the sanity
  limit (±93.75 Hz). Specifically: the function must NOT return NaN,
  Inf, or out-of-bounds values that would corrupt downstream.

Fail-before-passes verification: revert the `ofdm.cc` and
`telecom_system.cc` changes via `git stash`, rebuild, run tests. Test A
fails at link time (`carrier_frequency_sync_wb_mfsk` not declared);
B & C fail similarly. Restore via `git stash pop`; rebuild;
all 25 tests pass.

---

## §8 Open questions [?]

1. **[?] Does the mini-Moose move the cliff?** Plan §19.3 predicts
   +1-3 dB. Hardware A/B will resolve this. The PHY estimator math is
   sound; the open question is whether residual CFO is in fact the
   dominant cliff-bottleneck on IONOS WB.

2. **[?] Should NB MFSK also get mini-Moose?** §19.4 risk register
   says NB has different tone geometry and a separate fix. This audit
   leaves NB MFSK behavior unchanged. Future work: design an NB-MFSK
   variant if NB shows the same mirror-bin acceptance pattern.

3. **[?] Is half-symbol baseline the right capture-range tradeoff?**
   §20.2 chose ±11.7 Hz to cover crystal mismatch + coarse-sync
   residual. If hardware A/B shows the residual exceeds ±12 Hz at the
   cliff, follow-up axes are: (a) 2-baseline ambiguity resolution
   (1-sym + half-sym), (b) Lange–Mengali style data-aided estimator,
   (c) tighter coarse-sync first.

---

## §9 References

- `data-preamble-port-research.md` §19 — next-axis selection (the
  contract for this work).
- `data-preamble-port-research.md` §20 — mini-Moose plan (the design
  this audit covers).
- `data-flow-preamble_nSymb.md` — adjacent audit (preamble length).
- `mercury/source/physical_layer/telecom_system.cc:810` — function-scope
  declaration of `freq_offset_measured`.
- `mercury/source/physical_layer/telecom_system.cc:2193` — the fix site.
- `mercury/source/physical_layer/telecom_system.cc:2220-2244` — the
  re-mix consumer the fix extends to MFSK.
- `mercury/source/physical_layer/telecom_system.cc:2742-2745` — the
  cache write gate the fix preserves.
- `mercury/source/physical_layer/ofdm.cc:474` — WB OFDM Moose
  (`carrier_sampling_frequency_sync`).
- `mercury/source/physical_layer/ofdm.cc:537` — NB OFDM cross-symbol-
  phase estimator (`carrier_frequency_sync_nb`) — algorithmic template.

---

## §10 Audit ownership

- **Created**: 2026-05-28, alongside the mini-Moose feature branch
  `feat/mini-moose`.
- **Update obligation**: this document is updated by every commit on
  `feat/mini-moose` (and after merge to `monitor`) that touches
  `freq_offset_measured`, the mini-Moose estimator, or the re-mix
  block. A future change that re-introduces an MFSK-specific skip in
  the re-mix path MUST update §2.2 with the rationale.
- **Drift check**: every change to `telecom_system.cc` near line 2193
  or 2220 should grep this file and update the line numbers.

---

## §11 Sign-flip experiment audit (2026-05-28, branch `fix/sign-flip`)

See `data-preamble-port-research.md` §23 for the experimental rationale.
This audit captures the cross-layer impact of flipping the sign on
`freq_offset_measured` in the apply formula at `telecom_system.cc:2274`
from `effective_carrier_freq + freq_offset_measured` to
`effective_carrier_freq - freq_offset_measured`.

### §11.1 Producers — unchanged

§1.1 through §1.7 producers are all UNCHANGED. The estimator
(`carrier_frequency_sync_wb_mfsk`) still returns the same value as
pre-flip. The sanity clamp at `:2247-2250` still bounds the value to
±subcarrier_spacing. NB-MFSK still writes 0. WB-OFDM Moose still writes
its own Schmidl-Cox estimate.

### §11.2 Consumer change — §2.2 re-mix only

Only consumer §2.2 is structurally changed. The re-mix block at
`:2262-2279` now passes `effective_carrier_freq - freq_offset_measured`
to `passband_to_baseband_decimated`.

| Mode | Pre-§20 (monitor before 233c8a1) | Post-§20 (monitor 41e889f) | Post-§23 (this branch) |
|---|---|---|---|
| NB-OFDM | freq=0 → no re-mix | freq=0 → no re-mix | freq=0 → no re-mix |
| WB-OFDM | re-mix at `effective + δ_Moose` | re-mix at `effective + δ_Moose` | re-mix at `effective - δ_Moose` |
| NB-MFSK | freq=0 → no re-mix (skipped) | freq=0 → no re-mix (gate) | freq=0 → no re-mix (gate) |
| WB-MFSK | freq=0 → skip (mode-gated) | re-mix at `effective + δ_miniMoose` | re-mix at `effective - δ_miniMoose` |

The MFSK skip at the old `:2220-2223` was already dropped in §20; the
§23 flip applies to BOTH OFDM and MFSK because the gate is mode-agnostic.
This means the experiment couples WB OFDM behavior — see §11.4.

### §11.3 Consumer §2.5 cache write — unchanged shape, opposite values

The cache write at `:2782` still gates `if(M != MOD_MFSK)`. MFSK
residuals still don't enter the cache. But for WB-OFDM, the cache will
now contain the SAME magnitude estimate as before (the estimator output
is unchanged), even though the apply path uses the opposite sign. On the
next-frame `use_last_good_freq_offset` fallback (§1.3), the cached value
is read into `freq_offset_measured` then passed through the same flipped
apply formula. So the cache-fallback re-mix flips too. Consistent
behavior across single-frame and cache-fallback paths.

### §11.4 INV-1 broken by this experiment

Pre-§23 INV-1 stated: "OFDM behavior unchanged for every M != MOD_MFSK
config." Post-§23 this is FALSE — WB-OFDM re-mix now uses the opposite
sign too. The experiment intentionally couples OFDM along for the ride
because the §22 ctrl-frame regression and the §20 win are both about
the same `effective_carrier ± δ` arithmetic. A clean hardware A/B at
both robust-MFSK cells AND high-SNR OFDM cells will disambiguate:

- Clean OFDM + Clean MFSK gain at the flipped sign → original `+` was
  the bug across both modes.
- Clean OFDM unchanged + MFSK gain → only the WB MFSK estimator's
  return sign was wrong; the correct production fix is to flip the
  estimator at `ofdm.cc:799` instead of the apply formula. (This
  branch then deserves a follow-up that narrows the apply change.)
- Clean OFDM regression + MFSK gain → opposite-sign for the two
  estimators is the correct production state. The clean production
  fix is to flip the WB MFSK ESTIMATOR (not the apply), restoring
  OFDM behavior and keeping MFSK improved.
- Clean OFDM unchanged + MFSK unchanged → null result. The §20 win
  came from something other than CFO correction (see §23 H3).
- Clean OFDM regression + MFSK regression → original `+` was correct
  for both. The §22 ctrl-frame regression was something other than
  sign. Revert this branch.

### §11.5 Invariants preserved

INV-2 (NB-MFSK unchanged): YES, NB-MFSK still has freq=0 (no re-mix).
INV-3 (cache gate preserved): YES, the `if(M != MOD_MFSK)` gate at
`:2780` is not touched.
INV-4 (mini-Moose output finite): YES, estimator unchanged.
INV-5 (sanity clamp still bites): YES, clamp unchanged.
INV-6 (re-mix buffer shape): YES, only the `fc` argument's sign on
δ changes — buffer math, FIR, decimation rate, polyphase offset all
identical.

### §11.6 New regression test commitment

`mfsk_data_preamble_mini_moose_apply_sign_invariance` (data-preamble-
port-research.md §23.7) drives the full RX path with and without an
injected CFO and asserts the LLR magnitudes match within 10%. This test
catches the case where the apply formula doubles the CFO instead of
cancelling it.

The test is hooked into `mercury.exe --test` and must pass on this
branch. On `monitor` HEAD with the OLD sign, the test's outcome is the
fail-before-passes verification:

- If OLD-sign passes the new test too → §23 H3 (sign is irrelevant,
  re-mix is a no-op at the +7 Hz injection level used).
- If OLD-sign fails the new test (LLR magnitudes off by >10%) → §23 H1
  (the OLD sign was wrong; this branch's new sign is correct).

§23.10 [?]2 captures the H3 outcome as an open issue requiring a
distinct explanation for the §20 hardware win.

### §11.7 Drift check post-experiment

After hardware A/B resolution:
- If H1 confirmed (sign-flip wins) → merge to monitor; this audit's §11
  becomes the new authoritative description. Update §2.2 to make the
  `-` formula the documented baseline.
- If H2 confirmed (sign-flip regresses) → revert this branch; this §11
  stays as a record of the experiment, with §11.4 amended noting the
  empirical resolution.
- If H3 confirmed (null) → revert this branch; this §11 stays as a
  record; investigate the §20 win's actual mechanism.

### §11.8 The §21.9 sign-convention concern — resolution status

`data-preamble-port-research.md` §21.9 (the §22 ctrl-frame branch's
sign-convention note, referenced from the §22 hardware verdict) raised
a [?] about whether `+freq_offset_measured` was the correct apply
direction for the half-symbol-baseline cross-correlation estimator.
This experiment EXPERIMENTALLY DISAMBIGUATES that [?] via hardware A/B.

Pre-experiment state of [?]: open. The §20 hardware A/B (+11.7% bytes,
+55% bps) could be explained by either H1 (sign was wrong but small
δ at high SNR masked it) or H2 (sign was correct). The §22 hardware
regression of the same formula on a different code path is consistent
with both: H1 says ctrl-frame had large enough δ for the sign error to
bite; H2 says ctrl-frame had a different sibling bug.

Post-experiment state of [?]: will be set by hardware A/B per §11.4
decision matrix.

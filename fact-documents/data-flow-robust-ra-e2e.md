# Data-flow audit — ROBUST_RA end-to-end (real freq-sync → combiner → Bessel-I0 → GF16-RA)

**Status:** WIN CAMPAIGN (b) build — wiring the GF(16)-RA R¼ "−10 data mode" end-to-end
through the PRODUCTION RX chain (detector + mini-Moose + synced energy extraction) and
proving the integrated sim cliff vs the §7 genie/fading threshold.
**Branch:** `sim/robust-ra-e2e` off the combiner base `sim/data-detector-deepen-p3 @9cecc8f`
(= monitor @8fc1211 + build-1 M16×2 stream-energy combiner). Worktree
`C:/Users/kamer/mercury_wt/robust-ra-e2e`. **SIM ONLY** (dev host; v13 owns the IONOS).
**Date:** 2026-06-02. **Conventions:** §N sections, `file:line` citations, `[?]` unknowns.
Citations are against this worktree's tree unless noted.

This is the CLAUDE.md §5 cross-layer shared-state audit REQUIRED before the structural
change. It enumerates producers / consumers / valid-states / invariants for every piece of
shared state the ROBUST_RA wiring touches, and states exactly what the change alters.

---

## §0 The load-bearing numbers (verified, reconciled)

The P0 fact-doc (`robust-ra-data-code-p0.md`) records:
- **§3 (genie AWGN, deepest-cell):** GF16-RA **R¼** K=200 cliff = **−13.34** dB SNR3k
  (R⅓ −10.84, R½ −10.12); the interp-P=0.5 reference (§7.4) **R¼ = −13.54**, R⅓ −11.55.
- **§7 (Watterson fading + CFO, the MAKE-OR-BREAK gate):** **R¼ holds ≤ −10 on ALL three
  realistic profiles (MPG/MPM/MPP) with 2.6–2.9 dB margin: cliffs −12.78 / −12.63 / −12.92**,
  fading penalty only +0.6 to +0.9 dB vs the genie interp −13.54; **+5 Hz CFO costs ≤0.26 dB**
  (R¼ still clears by 2.5–2.8 dB). R⅓ sits ON the −10 line (−9.49 to −10.08), does NOT hold.
- **§7.6 VERDICT: PASS via R¼ / M16×2** (77 bps net wire > VARA's 71). R⅓ NOT recommended
  once fading is in the picture.

**Reconciliation of the task's "−12.9 genie threshold":** the task framed −12.9 as "FEC+fading
with GENIE sync". Precisely: −12.9 is the **fading** R¼ cliff (MPP), and the **genie** R¼ is
−13.34 (deepest) / −13.54 (interp). The operative end-to-end PASS threshold this build proves
against is therefore: the FULL real-sync chain (real `time_sync_mfsk_corr` + `carrier_frequency_
sync_wb_mfsk` mini-Moose + synced energy extraction + Bessel-I0 GF16-RA R¼ decode) must hold a
cliff in the **−12 to −13 dB** band to MATCH the genie/fading FEC-reach — i.e. the open question
is whether REAL freq-sync eats the ~2.7 dB fading margin that §7 proved on genie sync. The
detector half of that question was already answered by build-1: the M16×2 *detector* cliff is
−13.89 (combiner, deeper than the FEC). The remaining unknown is mini-Moose + the integrated chain.

---

## §1 Architectural decision (additive; gearshift/Q-table BYTE-IDENTICAL)

The task: "a NEW config ID; keep ROBUST_0/1/2 intact — full RA-replaces-ROBUST is a separate
follow-on … the new path is additive." Decision:

**ROBUST_RA = 103**, a new MOD_MFSK config with the **M16×2 geometry (identical to ROBUST_1/2)**
and the GF(16)-RA R¼ Q-ary codec on the RX decode (replacing `mfsk.demod`+LDPC for THIS config
only). **ROBUST_RA is NOT inserted into `FULL_CONFIG_LADDER`** — it is reached only by explicit
`-s 103` pin (exactly how the HW delivered-rate test drives it). This keeps:
- `FULL_CONFIG_LADDER` / `FULL_CONFIG_LADDER_SIZE` (=20) unchanged → every gearshift
  navigation function (`config_ladder_up/down[_n]`, `config_is_at_top/bottom`,
  `config_ladder_index`, `session_floor_anchor`) returns byte-identical results;
- the entire `arq_commander.cc` ladder/turboshift test battery (≈ §A–§K, hundreds of
  `check(... == ROBUST_2 ...)` assertions hardcoding ladder positions) untouched;
- the effective-rate Q-table (indexed by ladder configs) untouched.

`is_robust_config()` IS widened to include 103 so the robust-tier code paths (MFSK geometry,
CONNECT preamble reps, robust-floor logic) treat ROBUST_RA as a robust config. `NUMBER_OF_
ROBUST_CONFIGS` stays semantically "the ladder robust count" → see §5 for the exact rule
applied so no ladder consumer is destabilized.

---

## §2 The PRODUCTION MOD_MFSK data-frame RX chain (what the e2e test/impl drives)

Traced in `source/physical_layer/telecom_system.cc::receive_msg`:

1. **Detector** — `time_sync_mfsk_corr` (`telecom_system.cc:1069`, def `ofdm.cc:3462`).
   Returns `receive_stats.delay` (full-rate passband index of the preamble). Build-1's
   stream-energy combiner is INSIDE this fn, gated `mfsk_nStreams>=2` (`ofdm.cc:3560/3696`),
   so M16×2 ROBUST_RA gets the −13.89 combiner detector for free.
2. **Mini-Moose** — `carrier_frequency_sync_wb_mfsk` (`telecom_system.cc:2244`, def `ofdm.cc:619`).
   Residual CFO from the freshly-mixed baseband preamble → `freq_offset_measured`. WB only
   (`!narrowband_enabled`); NB-MFSK forced 0 (`:2256`).
3. **Sanity clamp** (`:2265-2283`): reject if |CFO| > 2·subcarrier_spacing (advance trial);
   else clamp to ±1 subcarrier_spacing.
4. **Re-mix** (`:2307-2324`): if |CFO| > `freq_offset_ignore_limit`, re-baseband the frame
   region at `effective_carrier_freq − freq_offset_measured` (§23 sign convention).
5. **Per-symbol FFT** — `symbol_demod` (`:2330-2333`) → `ofdm_symbol_demodulated_data[i*Nc..]`.
6. **Demap** — `mfsk.demod` (`:2340`) → soft LLRs → LDPC. **THIS is the step ROBUST_RA
   replaces** with the GF16-RA Q-ary decode (Bessel-I0 intrinsic, `gf16ra::soft_decode_k`).

The e2e TEST mirrors steps 1–4 with the real production fns, then extracts the N×16 energy
matrix at the SYNCED offset via `decode_suffix_energies` (which already does M16×2 per-stream
energy SUM, `ofdm.cc:4336-4345`) and runs `gf16ra::soft_decode_k`. The genie harness used a
FIXED `pattern_offset=4096/interp` (`mfsk_ctrl_codec_tests.cc` AWGN sweep); the e2e test uses
the offset RECOVERED by `time_sync_mfsk_corr` + the CFO-corrected baseband. That substitution
IS the make-or-break measurement.

---

## §3 Shared state #1 — gearshift config ladder (`include/common/common_defines.h`)

- **Producers (of the ladder constants):** `FULL_CONFIG_LADDER[]` :126, `FULL_CONFIG_LADDER_SIZE`
  :132, `NUMBER_OF_ROBUST_CONFIGS` :91, `ROBUST_0/1/2` :92-94, `is_robust_config` :96.
- **Consumers:** `config_ladder_index` :134; `config_ladder_up/up_n/down/down_n` :152-200;
  `config_is_at_top/bottom` :202-216; `session_floor_anchor` :238; `arq_commander.cc`
  turboshift/anchor logic + its entire self-test battery; `arq_common.cc` reset/floor;
  the effective-rate Q-table (config-indexed); GUI config menus (`main.cc`).
- **Valid states:** a config is either an OFDM config [0..16] (`is_ofdm_config`) or a robust
  config [100..102] (`is_robust_config`), or CONFIG_NONE (-1). The ladder array is the ONLY
  ordering authority; `config_ladder_index` returns -1 for configs not in the array.
- **Invariants consumers assume:**
  - INV-L1: `config_ladder_index(c) ∈ [0, SIZE)` for every config the gearshift can LAND on.
    A config not in the array returns -1 → `config_ladder_up/down` return the input UNCHANGED
    (`:158 if(idx<0) return config`). **This is the safety property that lets ROBUST_RA live
    OUTSIDE the ladder:** the gearshift can never *navigate to* 103 (it's not in the array and
    no `*_up/down` step produces it), and if something *pinned* 103 then asked the ladder to
    move, the fns return 103 unchanged (no crash, no wrong rung). 103 is only ever set by an
    explicit `-s 103`.
  - INV-L2: every `arq_commander` test asserts a SPECIFIC ladder neighbor (e.g.
    `config_ladder_up(ROBUST_1)==ROBUST_2`). Adding 103 to the array WOULD shift indices and
    break these. **Not adding 103 to the array keeps all indices identical → all tests pass
    byte-identical.**
  - INV-L3: `is_robust_config(c)` gates robust-tier behavior (MFSK geometry select
    `telecom_system.cc:5232/5449/5497`, CONNECT preamble reps `common_defines.h:104`, robust
    floor). ROBUST_RA MUST satisfy `is_robust_config(103)==true` to get M16×2 + robust handling.
- **What the fix changes:**
  - `#define ROBUST_RA 103` (new); `is_robust_config` range widened `… <= 102` → `… <= 103`.
  - `NUMBER_OF_ROBUST_CONFIGS`: **left at 3.** Audit of its consumers (below) shows it means
    "count of LADDER robust configs". ROBUST_RA is not a ladder config, so 3 is still correct
    for every consumer. (If a consumer iterated `ROBUST_0 + i` for `i<NUMBER_OF_ROBUST_CONFIGS`
    it would produce 100,101,102 — never 103 — which is the desired "RA is off-ladder" behavior.)
  - `FULL_CONFIG_LADDER` / SIZE: **UNCHANGED.** ⇒ INV-L1/L2 hold; gearshift + Q-table
    byte-identical.
  - Verified: `is_robust_config` widening does NOT change any ladder index (the array is the
    ordering authority, not the predicate). The predicate only flips robust-tier *behavior*
    selection for config 103, which previously was "not a config" (no behavior). No existing
    config's predicate result changes.
  - **`NUMBER_OF_ROBUST_CONFIGS` consumer check [VERIFY in impl]:** grep every use; confirm
    none assumes "all robust configs are contiguous-and-laddered up to ROBUST_0+N". (See §8.)

---

## §4 Shared state #2 — per-config PHY params (`load_configuration`, telecom_system.cc:5029+)

- **Producer:** the `if(configuration==…)` cascade (:5029 CONFIG_0 … :5148 ROBUST_0, :5155
  ROBUST_1, :5162 ROBUST_2) sets locals `_modulation`, `_ldpc_rate`, `ofdm_preamble_
  configurator_Nsymb`, `ofdm_channel_estimator`. Then reinit decisions (:5207-5247),
  `M=_modulation` (:5295), `ldpc.rate=_ldpc_rate` (:5296), MFSK geometry select (:5449/5497),
  preamble template + ofdm mirror (:5510-5590).
- **Consumers:** `M` gates the entire MOD_MFSK vs OFDM RX/TX split (hundreds of
  `if(M==MOD_MFSK)`); `ldpc.rate` sizes the LDPC matrices (the LDPC consumer — see INV-C1);
  MFSK geometry (`mfsk.M/nStreams/stream_offsets/preamble_tones`) feeds detector + mini-Moose
  + demod; `ofdm.mfsk_*` mirror feeds the combiner detector.
- **Valid states / default-init:** a config that hits NONE of the `if` arms leaves `_modulation`
  / `_ldpc_rate` at their PRE-CASCADE values (read just above :5029 from
  `default_configurations_telecom_system` / current). **This is the default-init bite:** an
  unhandled ROBUST_RA would silently inherit CONFIG_0's modulation/rate → wrong PHY, no error.
  ⇒ ROBUST_RA MUST have an explicit arm.
- **Invariants:**
  - INV-C1 (LDPC reinit): `if(_ldpc_rate != ldpc.rate) reinit ldpc` (:5223). ROBUST_RA does
    NOT use the binary LDPC for data (it uses GF16-RA), but `ldpc.rate` must still be a VALID
    rate the LDPC subsystem can initialize (the codepath at :5223/5243 runs unconditionally),
    AND the control-frame / handshake LDPC path still uses it. **Decision:** set ROBUST_RA
    `_ldpc_rate = 1/16.0` (same as ROBUST_0/1) so the LDPC subsystem initializes to a known-good
    matrix; the GF16-RA decode bypasses it for DATA frames only. Control frames at ROBUST_RA
    are MFSK-suffix (Phase B) + the same ctrl path as other robust configs — unaffected.
  - INV-C2 (MFSK geometry select): the three `current_configuration==ROBUST_0 ? M32×1 : M16×2`
    sites (:5232, :5449, :5497) — ROBUST_RA (103 ≠ ROBUST_0) takes the `else` = **M16×2**.
    This is correct and requires NO edit to those sites. The preamble template + ofdm mirror
    (:5510-5590) are geometry-driven (read `mfsk.M/nStreams/...`), so they produce the M16×2
    template + Welch-Costas preamble automatically.
  - INV-C3 (preamble Nsymb): `if(M==MOD_MFSK) ofdm.preamble_configurator.Nsymb = nb?8:16`
    (:5310). ROBUST_RA is MOD_MFSK ⇒ WB 16-symbol preamble automatically. The per-config
    `ofdm_preamble_configurator_Nsymb=4` set in the config arm is overridden to 16 by :5311 for
    all MFSK — matches ROBUST_0/1/2 exactly.
- **What the fix changes:** add an explicit `else if(configuration==ROBUST_RA)` arm mirroring
  ROBUST_1's (`_modulation=MOD_MFSK; _ldpc_rate=1/16.0; Nsymb=4; estimator=LEAST_SQUARE`). No
  other site in load_configuration needs an edit (the ROBUST_0 guards correctly route 103→M16×2).

---

## §5 Shared state #3 — MFSK geometry + preamble (`cl_mfsk`, mfsk.cc) and ofdm mirror

- **Producers:** `mfsk.init(M,Nc,nStreams)` (mfsk.cc:78) sets `nBits`, `tone_hop_step`
  (M=16→7), `stream_offsets` (centered, `:120-124`), `preamble_tones` (Welch-Costas g=2,
  16 symb for WB, `:126+`). The ofdm mirror (telecom_system.cc:5582-5589) copies M / nStreams /
  stream_offsets / preamble_nsymb / preamble_tones / match_threshold into `ofdm.mfsk_*`.
- **Consumers:**
  - `time_sync_mfsk_corr` (ofdm.cc:3462) reads `ofdm.mfsk_M/nStreams/stream_offsets/
    preamble_tones/preamble_nsymb` — detector + combiner.
  - `carrier_frequency_sync_wb_mfsk` (ofdm.cc:619) reads `mfsk.preamble_tones/M/nStreams/
    stream_offsets` (passed by receive_msg :2248-2249) — mini-Moose.
  - `mfsk.mod` / `mfsk.demod` (mfsk.cc:971/1025) read M/nBits/nStreams/stream_offsets/
    tone_hop_step — TX modulate / RX demap.
  - `decode_suffix_energies` (ofdm.cc:4308, used by the e2e test + the GF16-RA RX path) reads
    tone_hop_step / mfsk_M / nStreams / stream_offsets to build the N×16 energy matrix.
  - `build_gf16ra_data_audio` (test TX, mfsk_ctrl_codec_tests.cc) reads `ack_mfsk.M/nStreams/
    tone_hop_step/stream_offsets` — see INV-G1.
- **Invariants:**
  - INV-G1 (TX/RX geometry MUST match): the energy de-hop in `decode_suffix_energies`
    (`data_tone=(t−hop)%M`, `:4347`) and in `mfsk.demod` (`E[m]=E_raw[(m+hop)%M]`, mfsk.cc:1082)
    assume the SAME `tone_hop_step` and `stream_offsets` the TX used. For the e2e test, TX must
    lay tones with the SAME `cl_mfsk` instance's geometry the RX detector/extractor reads.
    **CRITICAL for the e2e test:** the genie AWGN harness builds on `ack_mfsk` (M=16,
    **nStreams=1** — CONFIG_0's ACK MFSK) and reports 2-stream bps by DOUBLING. The e2e test
    MUST instead use a **real M16×2 geometry** (the ROBUST_RA / ROBUST_2 `mfsk` instance,
    nStreams=2) so the combiner (gated nStreams≥2) actually fires and the per-stream energy SUM
    is exercised. ⇒ the e2e test loads ROBUST_2 (or ROBUST_RA) to get `mfsk` at M16×2, and
    builds TX on THAT instance.
  - INV-G2 **[CORRECTED after measurement — §G2-CORRECTION]**: my first draft assumed
    nStreams=2 carries **2 INDEPENDENT** GF(16) symbols/period (⌈N/2⌉ periods, "doubles bps").
    **That is WRONG and was falsified by the clean self-check** (14/200 = chance at every
    offset). The production extractor `decode_suffix_energies` (ofdm.cc:4336-4348) and the
    build-1 combiner detector (ofdm.cc:3560-3580) BOTH implement **frequency DIVERSITY**: they
    SUM each candidate tone's energy ACROSS streams, then one argmax → **ONE symbol per period**,
    the SAME symbol replicated in both stream bands. So the honest M16×2 ROBUST_RA is a codeword
    of N symbols over **N periods** (not N/2), 2-branch noncoherent diversity (array gain), NOT
    2 independent symbols. **The P0 fact-doc's "M16×2 DOUBLES bps → 77 bps" projected the
    independent-stream model that the code does not implement; the real diversity bps = the
    1-stream number (39 bps at R¼ K=200).** TX builder (`build_robust_ra_e2e_audio`) + the
    production decode (`decode_robust_ra_data`) both place/sum the same symbol in both streams.
    FIXED: clean self-check → **200/200 (chain VALID)** with this model.
  - INV-G3 (preamble identity): detector + mini-Moose both key off the SAME `preamble_tones`
    the TX preamble used (Welch-Costas g=2). Generated once by `mfsk.init`; the e2e test reuses
    the loaded config's `mfsk.preamble_tones` for both TX-preamble synthesis and RX.
- **What the fix changes:** NOTHING in mfsk.cc / the ofdm mirror — ROBUST_RA reuses the M16×2
  geometry verbatim. The e2e test adds NEW code that respects INV-G1/G2/G3.

---

## §6 Shared state #4 — `freq_offset_measured` (mini-Moose) — see data-flow-freq_offset_measured.md

- **Producer (MFSK):** `carrier_frequency_sync_wb_mfsk` (ofdm.cc:619) via receive_msg :2244-2250.
- **Consumers:** the sanity clamp (:2265-2283), the re-mix (:2307-2324, uses
  `effective_carrier_freq − freq_offset_measured`), and `receive_stats.freq_offset[_of_last_
  decoded_message]` (:2827-2828).
- **Invariants:** INV-F1 — WB-MFSK only (NB forced 0). INV-F2 — confidence gate returns 0 if
  |C|/energy<0.05 (ofdm.cc:791) ⇒ on a noise-dominated frame mini-Moose contributes 0 (no
  re-mix), which is the SAFE default (the genie offset is already near-correct from the
  detector). INV-F3 — the §23 apply sign (`− freq_offset_measured`) is the convention HW A/B
  will resolve; the e2e test inherits whatever the combiner branch ships (sign-invariance
  regression test `mfsk_data_preamble_mini_moose_apply_sign_invariance` already guards it).
- **What the fix changes:** nothing in this path. The e2e test EXERCISES it (the whole point) by
  injecting a residual CFO and a fading channel and letting mini-Moose + clamp + re-mix run
  before the GF16-RA decode. ROBUST_RA data frames flow through this path identically to
  ROBUST_1/2 (it is `M==MOD_MFSK && !narrowband` gated, config-agnostic).

---

## §7 Shared state #5 — the GF16-RA codec graph storage (`gf16ra::g_k_*`, mfsk_ctrl_codec.cc)

- **Producers:** `gf16ra::configure_k(K,repfact)` (:723) sets `g_k_K/repfact/NC/N`, flags
  `g_k_inited=false`; `init_k()` (:738) builds `g_k_acc_idx/wlog/factor_edges`; `encode_k`
  (:799), `soft_decode_k` (:813) call `init_k()` (idempotent rebuild guard).
- **Consumers:** `encode_k` / `soft_decode_k` read the `g_k_*` graph.
- **Invariants:** INV-K1 — `g_k_*` is INDEPENDENT of the K=13 ctrl-suffix graph (`g_*` via
  `configure()/encode()/soft_decode()`); the P0 fact-doc §6 verified the ctrl path is
  byte-identical and all 50 prior ctrl tests pass. INV-K2 — `configure_k` MUST be called before
  `encode_k`/`soft_decode_k` with matching K/repfact; the rebuild is forced on every
  `configure_k`. INV-K3 (single-threaded) — `g_k_*` are file-static globals; the data RX path
  is single-threaded per frame (receive_msg is not reentrant for a given telecom_system). The
  production RA data path calls `configure_k(K_data, 3)` once at config-load / first-use, then
  `soft_decode_k` per frame — no interleaving with the K=13 ctrl decode (different storage).
- **What the fix changes:** the production RA RX path becomes a NEW consumer of `g_k_*`
  (currently only the test consumes it). Must ensure `configure_k` is invoked with the RA data
  K/repfact before the first `soft_decode_k`. ⇒ the impl configures the RA codec at
  load_configuration(ROBUST_RA) time (or guards each decode with a configured-check).

---

## §8 Open items to VERIFY during implementation (don't ship assuming)

1. **`NUMBER_OF_ROBUST_CONFIGS` consumers** — grep all uses; confirm none breaks with RA
   off-ladder while the define stays 3. [§3]
2. **`is_robust_config(103)` ripple** — grep every `is_robust_config` call; confirm widening to
   include 103 only enables desired robust-tier behavior for an explicitly-pinned 103, and
   changes NO behavior for configs 0..102 (the predicate result for those is unchanged). [§3]
3. **Frame geometry / buffer sizing** — `data_container` buffers (`baseband_data_interpolated`
   sized `buffer_Nsymb`) must hold an RA data frame of N symbol-periods. The genie test used a
   PRIVATE decimation buffer to dodge the §5.2(1) overflow; the PRODUCTION RA frame size must
   be set so `buffer_Nsymb` ≥ the RA codeword's symbol-period count. [VERIFY frame-fill /
   nBits sizing for ROBUST_RA — this is the production-wiring risk the P0 doc flagged.]
4. **Byte-identical proof** — `git diff 9cecc8f` must show: common_defines.h (+ROBUST_RA define,
   widened predicate), load_configuration (+1 arm), the RX decode gate (+RA branch), the codec
   (cherry-picked, additive), the test (+e2e test + Watterson harness). NO edits to
   `FULL_CONFIG_LADDER`, arq_commander, the Q-table, or the OFDM/PSK paths. `mercury --test`
   green with the SAME prior-count + the new e2e test.

---

## §9 What the e2e test proves (the make-or-break)

`test_robust_ra_e2e_cliff` (new, mfsk_ctrl_codec_tests.cc): for each channel (AWGN, then
MPG/MPM/MPP via the §7 Watterson harness, optional +CFO), sweep σ; per trial:
encode_k(K,3) → lay the N coded symbols on the **real M16×2 `mfsk` geometry** (2/period) +
prepend the real Welch-Costas preamble → passband → [Watterson fade] → AWGN → **real
`time_sync_mfsk_corr`** (combiner detector) to FIND the preamble offset → **real
`carrier_frequency_sync_wb_mfsk`** mini-Moose + clamp + re-mix → `decode_suffix_energies` at
the SYNCED offset → `gf16ra::soft_decode_k` (Bessel-I0). Frame OK iff all K info symbols match.
Report the interp-P=0.5 cliff on the physical `snr3k_db` axis and ASSERT it lands in the
−12 to −13 band (the §7 genie/fading threshold) for R¼. **Fail-before:** without the RA mode
(no `configure_k` / no synced extraction) there is no decode → asserts fail. **Pass-after:**
the wired chain decodes at the threshold. **Fail-before** is concrete: on the pure combiner
base (before the P0 codec cherry-pick) the test does not COMPILE (no `gf16ra::soft_decode_k`);
with the codec but the WRONG geometry (the independent-stream draft) the clean self-check is
14/200 and the assert FAILS. **Pass-after:** the diversity geometry + the wired chain decode
200/200 clean and hold the cliff. The test drives the PRODUCTION decode method
`cl_telecom_system::decode_robust_ra_data` (RX-5), so the production Q-ary RA data path is
covered by the gate (no parallel test-only decoder).

---

## §10 RESULTS — the make-or-break ANSWER (measured 2026-06-02, dev host, SIM)

**Repro:** build `sim/robust-ra-e2e`; `MERCURY_RA_E2E_ONLY=1 [MERCURY_RA_E2E_FADE=ALL]
[MERCURY_RA_E2E_CFO=5] mercury.exe --test`.

### §10.1 The chain is VALID end-to-end
Clean (σ=0) self-check through the REAL chain (`time_sync_mfsk_corr` → `carrier_frequency_
sync_wb_mfsk` → `symbol_demod` → `decode_robust_ra_data`): **200/200 info symbols OK** ("chain
VALID") at K=200 R¼ on the real M16×2 geometry. The integration is faithful (guards a harness
bug from masquerading as a code result).

### §10.2 END-TO-END AWGN R¼ cliff (THE GATE) — PASS
| metric | value |
|---|---|
| END-TO-END AWGN R¼ cliff (interp P=0.5) | **−12.28 dB SNR3k** |
| genie-sync R¼ reference (P0 §7.4 interp) | −13.54 dB |
| **real-sync penalty (the open question)** | **+1.26 dB** |
| vs −10 target | **−2.28 dB (clears comfortably)** |
| net wire bps (diversity M16×2) | 39 bps |
| test verdict | **PASS** (`[OK] robust_ra_e2e_cliff`) |

Per-cell detect% (the combiner detector's own reach) vs P_frame:
`−9.78:100%/0.95 · −11.12:94%/0.86 · −12.28:71%/0.29 · −13.30:50%/0.00`. **At the cliff the
limiter is DETECTION, not the FEC** — P_frame collapses as detect% falls below ~70%. The build-1
combiner's −13.89 was the *coarse-only P3 ref scorer*; the FULL production detector (coarse +
fine pass + length-scaled threshold + the production FAR posture) reaches ~−13.3 at 50% detect,
so the integrated cliff sits at −12.28 (decode needs detect well above 50%).

### §10.3 ANSWER to the make-or-break
**Does real freq-sync eat the −10 margin? NO.** The full real chain (real combiner-detect +
real mini-Moose CFO sync + synced extraction + Bessel-I0 GF16-RA R¼ decode) holds an AWGN cliff
of **−12.28 dB**, i.e. **2.28 dB below the −10 target** and only **+1.26 dB shallower than the
genie −13.54**. Sync costs ~1.26 dB (dominated by the detector reach, not the Moose), and the
mode still clears −10 with > 2 dB margin. The genie −10 reach is REAL through production sync.
**Fading arms (MPG/MPM/MPP) [+ optional CFO]:** run via `MERCURY_RA_E2E_FADE=ALL`; see the run
log — the §7 genie→fading penalty was +0.6 to +0.9 dB at R¼, so the end-to-end fading cliffs are
expected ≈ −11 to −12 (AWGN −12.28 + ~+0.8 fading), still clearing −10. [run-log values to be
pasted on completion]

### §10.4 Geometry correction is load-bearing (§G2-CORRECTION)
The measured result is on the **diversity** M16×2 model (one symbol/period, both streams) — the
ONLY model the production extractor + combiner implement. The P0 "77 bps" independent-stream
projection is not realizable with the current extractor; the honest M16×2 ROBUST_RA rate is the
1-stream 39 bps at R¼ (the diversity buys ~2-branch array gain / reach, not 2× rate). This is
consistent with MEMORY's "Mercury wins on REACH not rate."

---

## §11 Production wiring — what shipped vs the explicit P2 follow-on (scope decision)

**Shipped (additive, byte-identical elsewhere):**
- `ROBUST_RA = 103` config define + `is_robust_config` widened to 103 (common_defines.h).
  Off-ladder (NOT in `FULL_CONFIG_LADDER`) → gearshift + Q-table byte-identical.
- `load_configuration(ROBUST_RA)` arm (MOD_MFSK, M16×2 via the ROBUST_0!= guards, ldpc rate
  1/16 for the ctrl path; the RA *data* decode bypasses binary LDPC).
- `cl_telecom_system::decode_robust_ra_data(...)` — the PRODUCTION Q-ary RA data decode
  primitive (synced per-symbol FFT → diversity energy combine → `gf16ra::soft_decode_k` →
  info bits). Exercised end-to-end by the gate test (the test's RX-5 calls THIS method).
- The build-1 M16×2 stream-energy combiner detector — already in the base, exercised by the
  M16×2 gate test.

**Explicit P2 follow-on (NOT done here — deliberately, per CLAUDE.md §4/§5):**
- **Frame-geometry reconciliation for an arbitrary-payload ARQ session at ROBUST_RA:** making
  `data_container.nBits` / the TX encode (`transmit_bit`) / the bit-interleaver bypass / ARQ
  batch sizing line up with the RA codeword length N = (1+repfact)·K, so a `-s 103` ARQ session
  round-trips real bytes through `receive_msg`→CRC→ARQ. This is a multi-subsystem change
  (data_container + TX + RX + ARQ batch) — the canonical cross-layer surface §5 warns against
  shipping half-wired. The PHY decode is proven; the byte/CRC/ARQ convergence is the next
  increment. **Reason NOT spliced into `receive_msg` now:** a gated branch into the shared
  binary-LDPC decode flow without the geometry reconciliation would be a band-aid (CLAUDE.md
  "no band-aids on broken architecture") and risk the OFDM/other-MFSK paths.

**§8 audit items resolved:**
1. `NUMBER_OF_ROBUST_CONFIGS` — **0 consumers in source/** (define-only). Left at 3 (= ladder
   robust count). Safe. ✓
2. `is_robust_config(103)` ripple — audited all ~35 call sites: config-validity gates now accept
   103 (correct — pin-able); robust-tier behavior gates (batch=1, robust timing/SNR, turbo
   settle, nIteration_max=200) are correct for an RA frame; **NO predicate result changes for
   configs 0..102.** Widening is purely additive. ✓
3. Frame geometry — characterized (see P2 follow-on above); the gate test uses a private symfft
   buffer sized to N, exactly as the genie used a private decimation buffer. ✓
4. Byte-identical — `git diff 9cecc8f` confirmed scope: common_defines.h, telecom_system.cc
   (+RA arm, +decode method), telecom_system.h (+decl), mfsk_ctrl_codec.{cc,h} (cherry-picked
   codec), mfsk_ctrl_codec_tests.cc (+harness +e2e test +env gate), + this fact-doc. NO edits to
   FULL_CONFIG_LADDER, arq_commander, the Q-table, OFDM/PSK. [--test full-suite green to confirm]

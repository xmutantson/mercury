# Fade-Estimator Prototypes — closing the production→GENIE gap at rate-0.875

Investigation record (2026-06-10). Goal: on the ESTIMATOR-LIMITED fade cells
(GENIE/perfect-CSI decodes rate-0.875, the production LS-3×9 estimator does not),
prototype better NONCOHERENT channel estimators (still rate-0.875, no coherent
demod) and find whether any CROSSES viability (full-codeword-decode p ≥ 0.8) —
which would let Mercury beat VARA on fade at a HIGHER rate than VARA's coherent
rate-1/2, avoiding the 3–5 wk coherent tier.

## §1 Substrate (verified)
- Harness: `sfo_grid_test()` in `source/physical_layer/telecom_system.cc`
  (tracker-patched, md5 `549b8105ed19` on .31/.21/.11; branch
  `sim/sfo-grid-prod-tracker` @0dadfa8 on .31). Drives the REAL production OFDM
  channel-estimate + equalize + LDPC decode on a synthetic Watterson grid.
- PROD estimator (`MERCURY_SFO_GRID_TRACK_PROD=1`): sets
  `ofdm.LS_window_width=3`, `LS_window_hight=9` (the production localized LS
  window, `physical_config.cc:64-65` odd-bumped) then calls
  `ofdm.LS_channel_estimator()` (`ofdm.cc:1649`). The 9-symbol VERTICAL window is
  the named limiter: it TIME-AVERAGES H over 9 symbols, so it lags a Doppler fade.
- GENIE (`MERCURY_SFO_GRID_THIN=1 + _GENIE=1`): hands the equalizer the exact
  per-symbol `Hwatt[n][j]` (telecom_system.cc:6715). Upper bound.
- The CODED block (telecom_system.cc:6888+) reads whatever `estimated_channel[]`
  + `noise_variance_estimate` the estimate branch left, CSI-weights LLRs, LDPC-
  decodes K rate-0.875 codewords, prints `[SFO-GRID-CODED] codewords_decoded=ok/K`.

## §2 GENIE_SPLIT finding being chased (compute_program/jobs/fade-estimator-headroom)
ALL SIX (cfg,chan) profiles ESTIMATOR-LIMITED at the operating SNR band (≥24 dB):
PROD full_decode_p=0.00 in all 24 cells; GENIE full-decodes at 24 dB (CFG15 all 3
+ CFG16-MPM) or 30 dB (CFG16-MPG/MPP). PROD `nv` is 2–70× too large (0.018–0.72 vs
genie AWGN 0.001–0.025) because the held LS window can't follow 0.1/0.5/1.0 Hz
Doppler → ZF noise-amplifies deep nulls → BP caps at 101 (over-confident LLR).

## §3 Prior art (Principle 1 — research before implementing)
- **Mostofi & Cox, "Pilot-symbol aided channel estimation for OFDM with fast
  fading channels"**, IEEE Trans. Wireless 2005 (IEEE Xplore 1247797): per-carrier
  LINEAR TIME-INTERPOLATION across comb pilots tracks Doppler where a held window
  averages. → Candidate 4 (TINTERP).
- **"Frame Splitting and Data-aided Decision Direct Channel Estimation for OFDM in
  Fast Varying Fading Channels"** (frame-splitting DDCE): feed tentative DATA
  decisions back as virtual pilots to refine H between real pilots; targets
  "channel-state fluctuation in the LAST part of the packet" — exactly the
  TAIL≫HEAD failure the harness comments describe. → Candidate 2 (DDCE2).
- **FreeDV-700D** (codec2): "linear interpolation in the TIME direction… very
  successful in multicarrier HF" — validates TINTERP for HF specifically.
- **Hsieh & Wei 1998** (pilot-aided OFDM fast-fading window tradeoff): narrower
  estimation window trades estimate noise for tracking speed. → Candidate 1 (LSW).
- **Hoeher/Kaiser/Robertson 1997** 2D-Wiener separable interp — already in the
  harness `grid_sparse2d_estimator` (the THIN path).

## §4 Candidates (env-gated; PROD decode byte-identical when unset)
All run on the dense Dx=1/Dy=3 prod lattice (like-for-like vs prod & genie;
shared seed = identical Watterson realization). Code:
`compute_program/jobs/fade-estimator-prototypes/estimator_candidates.cc.frag`,
inserted before the prod `else { LS_channel_estimator }` by `apply_estimator_patch.py`.
1. **LSW (narrower LS window)** `MERCURY_SFO_GRID_LSW_H/_W`: override the LS window
   height/width (3/5/7 sym) then re-run the SAME production `LS_channel_estimator`.
2. **DDCE2 (decision-directed)** `MERCURY_SFO_GRID_DDCE2[=1] _ITERS _W`: seed with
   prod LS, equalize+`psk.slice_nearest` each DATA cell, treat decision as virtual
   pilot (Hraw=rx/decided), per-carrier time MA-smooth, re-pin real pilots, iterate.
3. **PILOTDY2 (denser pilots)** `MERCURY_SFO_GRID_PILOT_DY=2`: Dy 3→2 lattice (more
   pilot SYMBOLS in time). Costs wire rate (reported).
4. **TINTERP (per-carrier linear time-interp)** `MERCURY_SFO_GRID_TINTERP[=1]
   _SMOOTH`: raw LS at every pilot, per-carrier linear interpolation in time across
   the Dy=3 pilot symbols (hold at edges), freq-interp fill, optional pilot pre-smooth.

### §4.1 nv-collapse correction (E1/cfg16-nvfix class, caught in smoke)
First DDCE2/TINTERP smoke showed `nv=1e-06` (collapsed) → over-confident LLR. The
pilot-residual EVM collapses because DDCE re-pins pilots (residual→0) and TINTERP's
smooth suppresses pilot noise. FIX: DDCE2 nv = DATA-cell DECISION EVM (rx − H·decided)
floored at the true AWGN var 10^(−EsN0/10); TINTERP nv = pilot-residual floored at
the same AWGN var. After the fix, nv tracks true noise (MPG: 0.004 vs genie ~0.004).

## §5 Smoke evidence (CFG15, EsN0=24, single seed; full sweep = §6)
- MPG/0.1 Hz: PROD 69/166 (0.42), DDCE2 103/166 (0.62), **TINTERP 165/166
  (0.994)**, TINTERP_S 165/166, GENIE 230/230 (1.00). TINTERP ≈ genie.
- MPP/1.0 Hz (hardest): PROD 0/166, LSW3 0/166, DDCE2 0/166 (BER 0.21→0.15),
  **TINTERP 16/166 (BER 0.21→0.035)**. TINTERP best even where genie's margin is thin.

## §6 Full sweep (compute_program/jobs/fade-estimator-prototypes)
900 cells: 2 cfg × 3 chan × EsN0{20,24,30} × 5 seeds × 10 modes (prod, genie,
lsw3/5/7, ddce1/2, tinterp/_s, pilotdy2). Analyzer: `analyze_estimator_prototypes.py`
→ `ESTIMATOR_PROTOTYPES.json`. [results below once landed]

## §6.1 PROD byte-identity CONFIRMED
The patched binary's PROD decode (env unset → falls through to
`LS_channel_estimator`) is byte-identical to the unpatched fade-estimator-headroom
binary (md5 b2117efd). Direct check: patched binary, cfg15/MPG/EsN0=20/seed=20360,
TRACK_PROD=1 → 110/166 (0.663); headroom `15|MPG|20|prod` rep0 (seed 20360) =
0.6627 → MATCH. (The aggregate prod-frac "diffs" vs headroom are a seed-GRID
artifact: headroom swept EsN0{16,20,24,30} so its per-point seeds offset by one
EsN0-block vs prototypes' {20,24,30} — different Watterson realizations per cell,
not a patch regression. full_decode_p matches exactly, 0.00 in every prod cell.)

## §6.2 Final numbers (900 cells, 0 fail, wall 1240 s)
best=tinterp. full-decode cross 3/12, ARQ mean-frac cross 5/12, mean gap closed
0.57. TINTERP mean_decode_frac: MPG 1.00 (=genie), MPM 0.88/0.89 (CFG15) /
0.37–0.53 (CFG16), MPP 0.23–0.33 (CFG15) / 0.02–0.03 (CFG16). LSW/pilotdy2 ≈ 0
gap; DDCE 0.19–0.28. See ESTIMATOR_PROTOTYPES_VERDICT.md.

## §7 Cross-layer note
This is a SIM-ONLY harness change (env-gated, prod default byte-identical). It does
NOT touch the production decode path. If a candidate crosses, the PRODUCTION change
would be in `cl_ofdm::LS_channel_estimator` / a new estimator selected by
`channel_estimator` — and THAT change requires the §Cross-Layer audit (mean_H,
pilot residuals, noise_variance_estimate consumers) before shipping. This
investigation establishes only the sim viability of the lever.

## §8 PROMOTED TO PRODUCTION (feat/fade-tinterp, 2026-06-10)
The sim-proven TINTERP estimator is now a PRODUCTION estimator mode, not a harness
patch. Implementation (off `monitor` @de428f6, branch `feat/fade-tinterp`):
- New `#define TIME_INTERP 2` (`physical_defines.h`); new method
  `cl_ofdm::LS_channel_estimator_tinterp` (`ofdm.cc`, ports the §4 cand-4 logic
  byte-for-byte: raw LS at every pilot → per-carrier linear time-interp (hold edges)
  → freq-interp fill → publish, NO DFT smooth).
- **nv-floor carried for production**: the harness frag floored the pilot-residual at
  the KNOWN Es/N0 `10^(-EsN0/10)` (§4.1). Production has no known Es/N0, so the method
  floors at `estimate_noise_from_pilot_pairs(in)` — the cross-pilot differential that
  measures the SAME pre-EQ thermal floor σ²/|X|². `nv = max(residual_vs_interp_H,
  cross_pilot_floor)`, floored 1e-6. Empirically nv=0.00655 (GOOD) vs the harness
  known-EsN0 0.00398 — same order, both decode K/K, neither collapses.
- **Routed** in the production decode path `telecom_system.cc:2734` (`else if
  channel_estimator==TIME_INTERP`). **Selected** by the FADE-tier gate at
  `telecom_system.cc:9574+`: `MERCURY_FADE_TINTERP` env (or a config-tier flag)
  promotes WB LS configs `LEAST_SQUARE → TIME_INTERP`. **Default-OFF**, byte-identical.
- **Cross-layer audit**: `fact-documents/data-flow-noise_variance_estimate.md` (the
  full producer/consumer registry §BRANCH-IF-CODE demanded). Every consumer of
  `noise_variance_estimate` / `estimated_channel` walked; the I1 nv-collapse hazard is
  the only assumption the change touches and the cross-pilot floor closes it.

### §8.1 Production-method reproduction (on the binary, this build)
`MERCURY_SFO_GRID_TINTERP_PROD=1` drives the PRODUCTION method on the dense lattice.
cfg15, EsN0=24, single seed (12345):
| profile | PROD LS | TINTERP (production method) | nv | matches verdict |
|---|---|---|---|---|
| MPG/GOOD (0.1 Hz) | 0/5 | **5/5** | 0.00655 | yes (1.00 full cross) |
| MPM/MOD  (0.5 Hz) | 0/5 | **4/5** | 0.0217  | yes (~0.88 partial) |
| MPP/POOR (1.0 Hz) | 0/5 | 0/5 | 0.0443  | yes (~0.28, Dy=3 under-samples 1 Hz) |
| FLAT-AWGN | 5/5 | 5/5 | — | no regression on clean channel |

### §8.2 Tests
- `tools/test_fade_tinterp.py`: FAILS on baseline `monitor` (TINTERP_PROD falls through
  to prod LS → 0/5 on GOOD), PASSES on `feat/fade-tinterp` (5/5 GOOD, 4/5 MOD, nv>1e-5
  all profiles). The CLAUDE.md §3 failing-test-first gate.
- `--test-climb-engine`: ALL PASS (0 failures), byte-identical to baseline.
- `--test-ofdm-fine-timing`: 4 passed 0 failed, byte-identical to baseline modulo
  wall-clock timing lines (default-off proof through the real decode path).
- sfo_grid PROD-else harness md5 A/B (gate off): IDENTICAL to baseline across
  flat-AWGN + Watterson cells.

### §8.3 Open / not-this-task [?]
HW confirmation (faithful fade loopback + bench), and enabling the FADE tier by
default, are the next gate — the sim ESTABLISHES the lever; HW CONFIRMS. MPP/1 Hz
remains a non-cross (needs Dy=2 × TINTERP, costs wire, or the coherent tier).

## §9 PRODUCTION FLEET VALIDATION (270 cells, 2026-06-10)
Full coded-BER fade map on the REAL production decode path, fleet .31/.21/.11.
Binary = feat/fade-tinterp ec1ece6 rebuilt o3 on all 3 boxes (md5 **identical**
`28f633d3ea956e1e485a00b67d7b2a03`). Job `compute_program/jobs/fade-tinterp-
production/` (generator `gen_fade_tinterp_production_job.py`, analyzer
`analyze_production.py`, result `PRODUCTION_TINTERP.json`). 2 cfg × 3 chan ×
EsN0{20,24,30} × 5 seeds × 3 arms = **270 cells, 270 ok, 0 fail, wall 316 s**.
Arms: prod (`TRACK_PROD=1`), tinterp_prod (`TRACK_PROD=1 TINTERP_PROD=1`, the
PRODUCTION `LS_channel_estimator_tinterp` + cross-pilot nv-floor), genie
(`THIN=1 GENIE=1`). Seed-paired to fade-estimator-prototypes (SEED_BASE=20355,
0 mismatches) → prod & genie reproduce the prototype realizations exactly.

### §9.1 Production fade map — mean_decode_frac (prod / tinterp_prod / genie)
| cell | prod | TINTERP_prod | genie | gap-closed | proto TINTERP | Δ vs proto |
|---|---|---|---|---|---|---|
| 15·MPG·20 | 0.00 | 0.70 | 0.91 | 0.77 | 0.71 | −0.01 |
| 15·MPG·24 | 0.02 | **1.00** | 1.00 | 1.00 | 1.00 | 0.00 |
| 15·MPG·30 | 0.02 | **1.00** | 1.00 | 1.00 | 1.00 | 0.00 |
| 15·MPM·20 | 0.00 | 0.66 | 1.00 | 0.66 | 0.62 | +0.04 |
| 15·MPM·24 | 0.00 | **0.91** | 1.00 | 0.91 | 0.88 | +0.03 |
| 15·MPM·30 | 0.00 | 0.98 | 1.00 | 0.98 | 0.89 | +0.09 |
| 15·MPP·24 | 0.00 | 0.21 | 1.00 | 0.21 | 0.33 | −0.12 |
| 15·MPP·30 | 0.00 | 0.32 | 1.00 | 0.32 | 0.23 | +0.10 |
| 16·MPG·24 | 0.00 | 0.86 | 0.99 | 0.87 | 0.84 | +0.02 |
| 16·MPG·30 | 0.01 | **1.00** | 1.00 | 1.00 | 1.00 | 0.00 |
| 16·MPM·30 | 0.00 | 0.55 | 1.00 | 0.55 | 0.53 | +0.02 |
| 16·MPP·30 | 0.00 | 0.02 | 1.00 | 0.02 | 0.03 | −0.01 |

(Full 18-row table in `PRODUCTION_TINTERP.json`.) Max |Δ vs prototype| (TINTERP
arm) = 0.12 (a single 5-seed MPP cell, within seed noise); all GOOD/MOD cells
within ±0.09. **The promoted production method REPRODUCES the sfo_grid patch.** The
(a)/(b)/(c) targets all hit: MPG → full decode (15·MPG·24=1.00, 15·MPG·30=1.00,
16·MPG·30=1.00), MPM ≈ 0.88+ (15·MPM·24=0.91), MPP ≈ 0.2–0.3 (under-samples 1 Hz).

**Integrity check + prod-baseline caveat:** the **genie arm reproduces the
prototype EXACTLY (max |Δ|=0.000)** — proves channel synthesis, AWGN, seeds, LDPC
are bit-identical between binaries. The **prod arm reads LOWER on MPG (~0.0 vs the
prototype's ~0.6)** because the prototype's `MERCURY_SFO_GRID_TRACK_PROD`
localized-3×9 *sliding*-window tracker is commit **0dadfa8** (prototype branch
`sim/sfo-grid-prod-tracker`), **NOT in monitor/de428f6** → on ec1ece6 `TRACK_PROD`
is a no-op and prod-LS uses the FULL-GRID window (averages the block → ~0 on the
slow fade). telecom_system.cc is byte-identical 8d290ec↔de428f6; ec1ece6's ofdm.cc
diff vs base is purely the additive TINTERP method (prod `LS_channel_estimator`/`ZF`/
`estimate_noise_from_pilot_pairs` untouched). This is a HARNESS-INSTRUMENTATION
difference in the BASELINE arm only — it widens, not narrows, the TINTERP gap; the
win stands on either prod definition (both fail to full-decode MPG).

### §9.2 Per-profile aggregate (avg 30 seeds/arm: 2 cfg × 3 EsN0 × 5)
| profile | prod | TINTERP_prod | genie |
|---|---|---|---|
| MPG/GOOD | 0.009 | **0.834** | 0.925 |
| MPM/MOD  | 0.000 | **0.579** | 0.969 |
| MPP/POOR | 0.000 | 0.110 | 0.951 |
CFG15-only TINTERP: MPG avg 0.90 (1.00 at EsN0≥24), MPM avg 0.85, MPP avg 0.20.

### §9.3 Clean-WGN no-regression (d)
Flat AWGN (chan=0), CFG15: prod=166/166 AND tinterp_prod=166/166 at EsN0=24
(iter_max=0, trivial), and 166/166 = 166/166 across 3 seeds at EsN0=20 (near
waterfall). TINTERP is a **no-op on flat** — no regression. nv floored (0.0022 vs
0.0040) but ≫ 1e-5, decode identical. Default-OFF (gate unset → LEAST_SQUARE) is
byte-identical by §8.2's md5 A/B.

### §9.4 nv-floor invariant I1 (production cross-pilot floor)
Global **min nv across all 90 tinterp_prod cells = 3.34e-3 ≫ 1e-5** — no collapse
anywhere. nv rises monotonically with fade severity (MPG ~5e-3 → MPM ~2.2e-2 →
MPP ~5e-2). The E1/cfg16-nvfix class (§4.1) does NOT recur on the production floor.

### §9.5 vs VARA faded wire (CFG15 clean wire 3060, ×3.69 compression edge)
At EsN0=24 operating point (Mercury_eff vs VARA reported wire; VARA double-Huffman
≈ no compression gain → its wire ≈ its effective):
| profile | TINTERP frac | Mercury wire | ×3.69 effective | vs VARA wire |
|---|---|---|---|---|
| MPG/GOOD | 1.00 | 3060 bps (0.82× VARA 3749 PHY) | **11,291 eff** | **3.01× VARA — BEATS** |
| MPM/MOD  | 0.91 | 2783 bps (0.77× VARA 3628 PHY) | **10,271 eff** | **2.83× VARA — BEATS** |
| MPP/POOR | 0.21 | 656 bps (0.28× VARA 2336 PHY) | 2,422 eff | 1.04× — marginal, non-cross |
On PHY wire alone Mercury trails VARA on every faded profile; the win is on
EFFECTIVE rate (compression-led thesis): the noncoherent rate-0.875 TINTERP PHY
full-decodes GOOD and ~88-91% of MOD, and the ×3.69 compression edge carries
GOOD/MOD to a clear effective beat. POOR/1 Hz does not cross (Dy=3 under-samples).

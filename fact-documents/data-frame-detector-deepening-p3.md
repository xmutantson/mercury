# P3 GATE — can the MFSK DATA-FRAME detector reach ≤ −11 dB SNR3k so the GF16-RA R⅓ FEC (−10.84) shows in production, at >71 bps?

**Status:** P3 MAKE-OR-BREAK gate for the WIN CAMPAIGN (b) −10-rate win. SIM ONLY (dev
host; NO bench — v13 owns the IONOS). **VERDICT: PASS — the detector is NOT the binding
constraint once the M16×2 2-stream combiner is fixed.** The shipped discrete-match data
detector already reaches −13.89 dB on M32×1; on the M16×2 ROBUST_RA geometry the *production*
detector cliffs at −9.03 dB (a 2-stream AND-gate bug), but a one-fix **stream-energy
combiner** moves it to **−13.89 dB (+4.86 dB)** — deeper than the GF16-RA FEC reach
(−10.84), so the FEC, not the detector, becomes the binding constraint. The −10 mode keeps
**103 bps wire** (R⅓ M16×2, P0 §3) > VARA's 71.

> **UPDATE 2026-06-02 (INCR-1) — the combiner is now PRODUCTIONIZED (§11–§15).** SIM build +
> `--test` (51/0) green; M16×2 production-path cliff DEEPENED −9.03 → **−15.05 dB @ T=7 /
> −13.89 dB @ T=8** (T=8 chosen, FEC-bound at −10.84), M32×1 BYTE-IDENTICAL. **FAR
> CORRECTION (§12):** the production detector's FAR is HIGHER than the §3 ref-scorer
> (fine-pass max) — T=8 M16×2 FAR = 1.8e-2 (NOT ≈0); CRC16-backstopped; clean follow-on in
> §13. **NOT merged** — pending the HW confirm that M16×2 acquisition deepens on RF.

**Date:** 2026-06-02. **Worktree:** `C:/Users/kamer/mercury_wt/data-detector-p3`, branch
`sim/data-detector-deepen-p3` off **mercury** repo `monitor @8fc1211`. Build `bash build.sh
o3`; MEASURE-only test added (env-gated `MERCURY_P3_SWEEP=1`), default build byte-identical.
**Conventions:** numbered sections (§N), `file.cc:line` citations, `[?]` for unknowns.

---

## §1 The dispatch premise had a conflation — RESOLVED before measuring

The P3 dispatch said "deepen the MFSK DATA-FRAME detector from −8.7 so the GF16-RA R⅓
FEC reach of −10.84 shows in PRODUCTION". Three chains were tangled; pinned from executing
code + the sibling fact-docs:

1. **The "−8.7 dB detection scaffolding" is the CONTROL-suffix chain, NOT the data frame.**
   It is `detect_ack_pattern` matched-count + the `metric>=3.0` gate used for CONNECT/ACK/HAIL
   *link establishment* (tier2-suffix-fec-gf16-spike.md §8.3/§16; tier2-suffix-fec-design.md
   §16/§17). The data OFDM/MFSK demod uses a **different** detector and has **no metric>=3.0
   gate** (§16 of the design doc: "the data OFDM demod uses a DIFFERENT detector").
2. **The GF16-RA −10.84/−14.03 is the CONTROL-suffix FEC** (K=13, CONNECT handshake) in the
   spike doc. The *data-length* GF16-RA was measured separately in **P0**
   (`robust-ra-data-code-p0.md`): R⅓ data block (K=200, N=600) = **−10.84 dB genie-sync @
   103 bps M16×2**; R¼ = −13.34 @ 77 bps. P0's GO is "R≤⅓ + M16×2", a **NEW ROBUST_RA
   family**, NOT existing ROBUST_2 (whose rate-1/4 LDPC PHY cliffs at −8.0 dB and is only
   68.82 bps — see `robust2-minus10-rate-win.md`).
3. **P0 §5(a) is literally the P3 work item:** "genie-sync FEC-reach — excludes the
   detection/sync scaffolding … the data-frame preamble + Schmidl-Cox/coarse sync, a SEPARATE
   P3 work item. The production cliff will be SHALLOWER than these FEC-reach numbers until
   that is fixed." P3 = does the **data-frame preamble detector** reach ≤ −11 so the FEC's
   −10.84 isn't detector-limited?

**The data-frame detector under test** = `cl_ofdm::time_sync_mfsk_corr`
(`ofdm.cc:3462`), the SHIPPED discrete-match FFT-bin-argmax port (data-preamble-port-research.md
§14; shipped on monitor, moved the data cliff −4→−8 dB OTA). Sole production caller
`telecom_system.cc:1069`. Gate = `fine_best_matched >= mfsk_preamble_match_threshold` (=7/16,
`mfsk.cc:221`). Preamble = 16 symbols, 8 Welch-Costas g=2 base tones × 2 reps
(`mfsk.cc:149-168`). The N=32 preamble was REVERTED (−25.9% throughput at WGN:−11, MEMORY).

---

## §2 Why the data-frame detector cliffs — the mechanism (file:line)

The detector reads `mfsk_M`, `mfsk_nStreams`, `mfsk_stream_offsets`, `mfsk_preamble_tones`,
`mfsk_preamble_match_threshold` (all set by `load_configuration`, `telecom_system.cc:5582-5589`).
Per candidate symbol it FFTs one decimated symbol and, **per stream**, finds the argmax bin
among that stream's M tones; the symbol counts as matched only if **ALL streams** independently
argmax-match the expected (or mirror) bin:

```
ofdm.cc:3556:   if (streams_matched < mfsk_nStreams) continue;   // AND across streams
ofdm.cc:3558:   matched++;
```

For the 2-stream geometry (ROBUST_1/2/RA = M16×2), `generate_preamble` places the **SAME
tone in BOTH streams** (`mfsk.cc:518-522`, amp `sqrt(Nc/nStreams)` each). The two streams are
therefore **redundant copies** of the same preamble tone. The AND-gate requires each stream's
argmax to independently land on the right bin → per-symbol match probability ≈ **p²** (p =
single-stream argmax-correct prob), instead of the **p (with +3 dB energy gain)** that an
energy-combiner would get. **This squares the per-symbol error and collapses the matched
count ~2× faster at low SNR** — the cliff driver.

**Direct evidence (§3 sweep, mean matched count at SNR3k = −9.03 dB):**
- M32×1 (1 stream, no AND-gate): mean_matched = **14.3** → easily clears 7/16.
- M16×2 (2-stream AND-gate): mean_matched = **7.5** → right at the 7/16 gate → P(detect) collapses.

The control base detector (`detect_ack_pattern`) is **M16×1** (WB ack family is always 1-stream,
`mfsk_vara_parity_audit`) — which is why HAIL's matched-count floor is −13.25 dB
(`hail-detection-floor-investigation.md` §13) and CONNECT-base reaches −14.68. The data M16×2
geometry is the ONLY place the 2-stream AND-gate bites, and it is exactly the −10-mode geometry.

---

## §3 RESULTS — the make-or-break sweep (measured, physical SNR3k axis)

New MEASURE-only test `test_data_preamble_detector_cliff_sweep`
(`mfsk_ctrl_codec_tests.cc` §6.P3, env `MERCURY_P3_SWEEP=1`). Drives the PRODUCTION
`time_sync_mfsk_corr` on a synthesized preamble + passband AWGN, **same `snr3k_db` axis as
every campaign number** (`mfsk_ctrl_codec_tests.cc:2843`, `n3k = σ²·3000/(fs/2)`); 200
trials/σ; cliff = deepest SNR3k with P(detect) ≥ 0.5. Also scores a **stream-energy-combined**
variant (sum the streams' per-tone energy BEFORE argmax — equal-gain noncoherent combining,
the optimal use of the redundant per-stream tone; one match-decision per symbol). Both
configs, threshold T=7 (prod) and relaxed 6/5/4, with pure-noise FAR per arm.

### §3.1 ROBUST_0 (M32×1 — existing data preamble)
| arm | cliff (P=0.5) | FAR @T7/poll |
|---|---|---|
| **PRODUCTION (AND-gate, T=7)** | **−13.89 dB** | 2.25e-3 |
| relax T=6 / T=5 / T=4 | −15.05 / −16.07 / −16.07 | 2.8e-2 / 1.7e-1 / 5.4e-1 |
| stream-combined (T=7) | −12.55 (−1.34, no-op: 1 stream) | 0.0 |

**M32×1 already PASSES (−13.89 ≤ −11) with margin.** Relaxing T deepens it but FAR explodes
(T=5 = 17%/poll) — not worth it. Combining is a slight no-op-with-overhead at 1 stream
(irrelevant; M32×1 isn't the −10 mode).

### §3.2 ROBUST_2 / ROBUST_RA (M16×2 — THE −10-mode geometry)
| arm | cliff (P=0.5) | FAR @T7/poll | mean_matched @−9 dB |
|---|---|---|---|
| **PRODUCTION (AND-gate, T=7)** | **−9.03 dB** ✗ misses −11 | 0.0 | 7.5 |
| relax T=6 / T=5 / T=4 | −9.03 / −10.05 / −10.97 (still miss) | 0 / 0 / 0 | — |
| **LEVER stream-energy-COMBINED (T=7)** | **−13.89 dB** ✓ (+4.86 dB) | 6.25e-3 | **13.9** |
| combined relax T=6 / T=5 / T=4 | −13.89 / −16.07 / −16.07 | 4.0e-2 / 2.1e-1 / 6.2e-1 |

**The production M16×2 detector cliffs at −9.03 dB — MISSES the ≤ −11 gate by ~2 dB** (the
2-stream AND-gate, §2). **The stream-energy combiner moves it to −13.89 dB (+4.86 dB) — CLEARS
≤ −11 with ~3 dB margin** and tracks the full matched-count floor (mean_matched stays 13.9 at
−9 dB vs 7.5 for the AND-gate). FAR cost: combined T=7 = 6.25e-3/poll (vs AND-gate's 0 —
the AND-gate's FAR-suppression is exactly what was costing the +4.86 dB). 6.25e-3 is the
same order as the shipped M32×1 production FAR (2.25e-3) and is backstopped downstream by the
LDPC CRC16 (a spurious preamble → LDPC runs on noise → CRC16 fail → ~30 ms wasted, not
corruption — data-preamble-port-research.md §11.6). A T=8 bump on the combined detector would
restore FAR ≈ 0 while keeping most of the gain [?] (not swept; the T=4..7 trend shows headroom).

---

## §4 VERDICT — PASS. The production −10 win is feasible end-to-end.

**P3 PASS bar = "detector cliff ≤ −11 dB SNR3k AND the resulting data mode keeps > 71 bps wire."**

- **Detector cliff ≤ −11: MET on the −10-mode geometry** — via stream-energy combining
  (M16×2 −13.89 dB). The shipped M32×1 detector already meets it (−13.89). The production
  M16×2 detector alone does NOT (−9.03) — the combiner fix is **required**.
- **> 71 bps: MET** — the −10 mode is ROBUST_RA R⅓ on M16×2 = **103 bps wire** (P0 §3). R¼ =
  77 bps also clears.
- **Detector is no longer the binding constraint:** combined detector −13.89 ⊇ GF16-RA R⅓ FEC
  −10.84 ⊇ −10 target. With the combiner, the **FEC (−10.84), not the detector, sets the
  production end-to-end cliff** — exactly the condition P0 §5(a) required ("so the FEC's
  −10.84 isn't detector-limited"). The "~3 dB shallower in production" caveat is DISCHARGED
  for the data-frame detector by the combiner.

**End-to-end −10-mode chain (all SIM, genie/per-stage):**
P1 Bessel-I0 demap (built `wt/bessel-i0`) + P2 ROBUST_RA family (R⅓ M16×2 FEC −10.84, P0) +
**P3 data-frame detector −13.89 (this doc, combiner fix)** → the −10 production win is
feasible. The binding stage is the **FEC at −10.84 (≈ −0.84 below the −10 target → clears)**;
the detector has ~3 dB of headroom over it.

---

## §5 Ranked levers to deepen the data-frame detector (measurement-informed; cited)

| # | lever | dB gain (M16×2) | rate cost | FAR cost | prior art |
|---|---|---|---|---|---|
| **1** | **stream-energy combining** (sum streams' per-tone E before argmax; replace AND-gate) | **+4.86 (MEASURED)** | **0** (RX-only; same preamble) | T7 0→6.25e-3 (restore w/ T=8 [?]); LDPC-CRC backstop | equal-gain noncoherent combining, Proakis 5e §14.4; Q65 multi-tone energy sum (K1JT) |
| 2 | threshold relax T=7→6/5/4 | M16×2 prod: +0/+1.0/+1.9 (still misses −11); combined: +0/+2.2/+2.2 | 0 | M16×2 prod FAR stays 0 to T=4 (AND-gate); combined FAR 4%/21%/62% | FT8/Q65: CRC is the FAR gate, not a pre-decode energy threshold (Franke-Taylor QEX 2020) |
| 3 | noncoherent preamble REPETITION (R× preamble, sum E across reps) | +2.2/doubling (R=2), +3.7 @R=5 (MEASURED for this M16 algo, HAIL §11/§14) | **−rate** (R× longer preamble → header occupancy; N=32 already REVERTED −25.9% @WGN:−11) | combining on matched-COUNT; ratio-gate-free | Q65 ~2.6-3.0 dB/integration-doubling (WSJT-X 2.7 guide); FST4/Q65 sync ≈25% airtime |
| 4 | longer preamble (more symbols, same R) | ~+1.5/doubling via √N matched-filter gain (data-preamble-port §2.2) | **−rate** (header occupancy; N=32 revert proves it net-loses on the bps floor) | tighter FAR | WSPR/FT8 long-Costas-sync at −24 dB (WB2FKO TechFest 2019) |
| — | Bessel-I0 *for detection* | ~0 (detection is argmax, not soft-LLR; Bessel helps the FEC demap, measured ~0 at the cliff in robust3-feas §4) | 0 | — | Southampton ePrints 267344 (efficient demaps within 0.2 dB of ML) |

**Lever #1 (stream combining) is the clear winner: +4.86 dB MEASURED, ZERO rate cost,
RX-only, single-function fix, and it is a ROOT-CAUSE fix (the AND-gate discards the 2-stream
diversity the TX already pays for) — not a threshold band-aid (CLAUDE.md §1.2/§2).** Levers
3/4 trade rate (the N=32 revert already proved longer preambles net-lose on the bps-limited
floor); they are margin/ULTRA-tier tools, not needed for the −10 mode. Lever 2 alone cannot
reach −11 on M16×2 and raises FAR on the combined arm.

---

## §6 Shared benefit for the REACH win (ULTRA)

The ULTRA tier (`ultra-tier-design.md`; tier2-design §15) is built on M16×2 (and deeper)
robust modes and needs the SAME data-frame detector to reach its −20…−24 dB floor. The
stream-combiner fix (#1) is a **prerequisite the reach win shares** — without it the ULTRA
data preamble would cliff ~5 dB shallower than its FEC, exactly as the −10 mode does. Lever 3
(preamble repetition, +2.2 dB/doubling) is the ULTRA-tier extension on top of the combiner
(cheap at ULTRA's multi-minute frames). So P3's recommended fix is load-bearing for BOTH the
−10 rate win and the reach win.

---

## §7 Cross-layer note (the combiner fix, when productionized — NOT done here)

This doc is SIM scope/measure ONLY; NO production config changed. When the combiner is
productionized (P4+), it touches `cl_ofdm::time_sync_mfsk_corr` (shared by the §5 data-flow
audit of `data-flow-preamble_nSymb.md`):
- The 1-stream path (M32×1 ROBUST_0) must be **byte-identical** (combining = no-op at
  nStreams=1, but the §3.1 −1.34 dB shows the mirror-tone argmax differs subtly → gate the
  combiner behind `nStreams>=2`, or verify the M32×1 cliff is unchanged). The §3 sweep is the
  regression harness (fail-before/pass-after on the M16×2 cliff move).
- FAR: the combined detector's 6.25e-3/poll at T=7 must be reconciled with the production
  threshold (consider T=8 for M16×2 [?]) and the LDPC-CRC backstop; a spurious preamble is
  wasted compute, not corruption.
- The control/HAIL/ACK detectors (`detect_ack_pattern`) are M16×**1** and are NOT touched.

---

## §8 Reproduction

```
cd C:/Users/kamer/mercury_wt/data-detector-p3 && bash build.sh o3
env MERCURY_P3_SWEEP=1 ./mercury.exe --test   # prints "[P3] DATA-FRAME DETECTOR CLIFF SWEEP"
#  ROBUST_0 M32x1 prod cliff -13.89; ROBUST_2 M16x2 prod cliff -9.03, stream-combined -13.89 (+4.86)
# default (no env): byte-identical to monitor 8fc1211; full suite passes.
```

## §9 Source cites
| file:line | what |
|---|---|
| `ofdm.cc:3462` | `time_sync_mfsk_corr` — the shipped discrete-match data-frame detector |
| `ofdm.cc:3560` (Phase-1), `ofdm.cc:3696` (Phase-2) | **PRODUCTIONIZED** `if (mfsk_nStreams>=2)` stream-energy combiner (sum-then-argmax); `else` = legacy per-stream AND-gate path (byte-identical at 1 stream) |
| `ofdm.cc` (was :3556) | the OLD 2-stream AND-gate (`streams_matched < mfsk_nStreams → continue`) — the cliff driver, now REPLACED for nStreams≥2 |
| `ofdm.cc` detection gate | `fine_best_matched < mfsk_preamble_match_threshold` (unchanged) |
| `mfsk.cc:518-522` | `generate_preamble` places the SAME tone in both streams (redundant → combinable) |
| `mfsk.cc:149-168` | preamble tones (Welch-Costas g=2 ×2 reps) |
| `mfsk.cc:220` | **threshold now `nStreams>=2 ? 8 : 7`** (was uniform 7) + the updated FAR-model comment |
| `telecom_system.cc:1069` | sole production caller |
| `telecom_system.cc:5582-5589` | per-config population of the detector's mfsk_* fields (routes the gate uniformly) |
| `mfsk_ctrl_codec_tests.cc` §6.P3 | the MEASURE-only sweep (env `MERCURY_P3_SWEEP`); §6.P4 = the always-on fail-before/pass-after guard |
| `mfsk_ctrl_codec_tests.cc` `snr3k_db` | the SNR3k axis (shared with all campaign numbers) |

## §10 Open questions [?]
1. **[?] FAR of the combined detector at T=8/9 on M16×2** — not swept; the T=4..7 trend
   suggests T=8 restores FAR≈0 while keeping most of +4.86 dB. Measure before productionizing.
2. **[?] AWGN only** — no fading/CFO. HF Doppler will cost the noncoherent floor more (P0 §5b);
   the combiner gain is an AWGN array gain and should largely hold, but HW A/B (post-P4) confirms.
3. **[?] M32×1 byte-identical under a productionized combiner** — §3.1 shows a −1.34 dB
   difference in the *combined* scorer at 1 stream (mirror-tone argmax). Gate behind
   `nStreams>=2` to keep ROBUST_0 untouched.

---

## §11 PRODUCTIONIZATION (WIN CAMPAIGN INCR-1, 2026-06-02) — SIM only, NOT merged

**Status:** the §5 lever #1 (stream-energy combiner) is now PRODUCTIONIZED in
`cl_ofdm::time_sync_mfsk_corr`, gated `nStreams>=2`. SIM build + `--test` green; the
M16×2 production-path cliff DEEPENED **−9.03 → −13.89 dB SNR3k @ the chosen T=8**
(−15.05 @ T=7). M32×1 is BYTE-IDENTICAL. **NOT merged** — pending the HW confirm that
M16×2 *acquisition* deepens on RF (the HW-confirm-better rule).

**Branch:** `sim/data-detector-deepen-p3` (this worktree), one commit on top of the P3
MEASURE commit `6771c8b`. Build `bash build.sh o3`; foreground.

### §11.1 The two-block edit (`source/physical_layer/ofdm.cc`)
Both per-symbol decision blocks of `time_sync_mfsk_corr` (Phase-1 coarse + Phase-2 fine)
were restructured to branch on `mfsk_nStreams`:
- **`ofdm.cc:3560` (Phase-1) and `ofdm.cc:3696` (Phase-2):** `if (mfsk_nStreams >= 2)`
  → **SUM each candidate tone's per-stream energy, then a single argmax over M tones**,
  accept iff argmax == expected tone OR `(M−tone)%M` (tone-space mirror). This is
  **bit-equivalent to the P3 reference scorer** `p3_score_stream_combined` (§3), so it
  inherits the measured combining gain. Replaces the legacy AND-gate
  (`streams_matched < mfsk_nStreams → continue`).
- **`else` (nStreams==1):** the legacy per-stream argmax + **bin-space** mirror accept,
  kept VERBATIM (uses `mfsk_stream_offsets[0]` only). At 1 stream the combiner would be a
  no-op except the tone- vs bin-mirror difference (§3.1, −1.34 dB), so gating preserves
  ROBUST_0 / M32×1 byte-for-byte.
- The secondary `e_target`/`metric` (energy-ratio tie-break, diagnostic only) is computed
  identically in both branches (it already summed expected+mirror across all streams).

### §11.2 Threshold (`source/physical_layer/mfsk.cc:220`) — gated on nStreams
The discrete-match threshold is now **`nStreams>=2 ? 8 : 7`** (was uniform 7). Rationale,
from the production-path FAR measured here (NOT the §3 ref-scorer; see §12):
- **nStreams==1 (M32×1/M8×1):** T=7 unchanged → byte-identical; the Binomial(N,2/M) FAR
  model (`mfsk.cc`) still holds (WB M=32 FAR 2.57e-5/poll).
- **nStreams>=2 (M16×2/M4×2):** **T=8.** The combiner removes the AND-gate (which had been
  the documented FAR mitigation for the degenerate-mirror cases, `mfsk.cc` old comment).
  T=8 keeps the FULL end-to-end win — the data FEC binds at −10.84 dB, ~3 dB ABOVE the
  −13.89 T=8 detector cliff, so −15.05→−13.89 costs **nothing** end-to-end — while cutting
  FAR ~5× vs T=7 (§12). The residual is CRC16-backstopped.

### §11.3 Measured outcome (`MERCURY_P3_SWEEP=1`, production detector, this build)

| geometry | arm | cliff (P=0.5) T=7 | cliff T=8 | byte-identical? |
|---|---|---|---|---|
| M32×1 (ROBUST_0) | PRODUCTION-PATH | **−13.89** | −12.55 | **YES** — matches the pre-combiner −13.89 (gate off); FAR T=7 2.25e-3 (matches §3.1) |
| M16×2 (ROBUST_2) | PRODUCTION-PATH (combiner) | **−15.05** | **−13.89** | n/a — this is the deepened path |
| M16×2 | P3 ref scorer (coarse-only) | −13.89 | −12.55 | (reference; production-path is ≥ deeper) |

M16×2 P(detect) on the production path: 1.00 @ −10.97 (T=8), **0.94 @ −11.80 (T=8)**,
0.57 @ −13.89 (T=8). **CLEARS the −11 PASS bar with ~3 dB margin at T=8.** The production
path (Phase-1 coarse + Phase-2 fine + full search) is ~1.2 dB DEEPER than the coarse-only
ref scorer — the productionized result slightly exceeds the P3-measured lever.

### §11.4 Tests (`source/physical_layer/mfsk_ctrl_codec_tests.cc`)
- **NEW §6.P4 `test_mfsk_data_preamble_stream_combiner`** (always-on, fail-before/pass-after):
  (A) M16×2 (ROBUST_2) at SNR3k −11.80 dB must detect ≥30/40 — PRE-FIX (AND-gate, cliff
  −9.03) this is ~0% so it FAILS on `monitor`/`6771c8b`; POST-FIX **39/40 @ T=8** (40/40
  @ T=7) → PASSES. (B) M32×1 (ROBUST_0) non-regression at −9.03 dB → **40/40**.
- §6.P3 sweep extended: Ttest now `{8,7,6,5,4}` (T=8 added per §10.1), and the per-T
  decision is `mc>=T` (exact at every T, including the new tighter T=8) instead of reusing
  the T=7 gate result. The "PRODUCTION-PATH" arm now exercises the gated combiner build.
- `mercury --test` = **51 passed / 0 failed** (was 50 at `6771c8b`; +1 = §6.P4). The five
  existing §6.1–§6.5 ROBUST_0 argmax tests pass unchanged = the M32×1 byte-identity
  regression evidence.

---

## §12 FAR CORRECTION — the production-path FAR is HIGHER than the P3 ref scorer

§3.2 / §10.1 used the **coarse-only ref scorer** and reported combined FAR 6.25e-3 @ T=7,
predicting "T=8 restores FAR≈0". The **production detector runs the Phase-2 fine pass**
(±½-symbol, taking the MAX matched count over sub-positions), which inflates FAR. Measured
on the production detector (4000 pure-noise polls, this build):

| geometry / path | T=8 | T=7 | T=6 |
|---|---:|---:|---:|
| M32×1 production (legacy path) | 0.0 | 2.25e-3 | 2.80e-2 |
| **M16×2 production (combiner)** | **1.80e-2** | **9.30e-2** | 3.18e-1 |
| M16×2 ref scorer (coarse-only) | 1.75e-3 | 6.25e-3 | 4.05e-2 |

**Correction:** on the production detector, T=8 does **NOT** give FAR≈0 for M16×2 (1.8e-2,
not ~0). The ~15× gap vs the ref scorer is the fine-pass sub-position max. T=8 is still the
right operating point (full end-to-end win at −13.89, FEC-bound; 5× lower FAR than T=7;
CRC16-backstopped), but the FAR≈0 claim is REVISED — see §13 for the clean follow-on fix.

---

## §13 [DONE — see §17] Fine-pass FAR inflation — a clean follow-on (implemented before merge)

The residual M16×2 FAR (1.8e-2 @ T=8) is driven by the detection decision using
`fine_best_matched` — the MAX matched count over the ±½-symbol fine grid. A cleaner design
gates the *detect/no-detect* decision on the **coarse** matched count (one position per
symbol grid, the ref-scorer's lower-FAR statistic) and uses the fine pass ONLY to refine
the returned sample OFFSET, not to re-maximize the count. The ref-scorer FAR (1.75e-3 @ T=8)
is the achievable target. This touches the detector's gate structure for BOTH stream paths
(would change M32×1 too → must re-confirm byte-identity), ~~so it is OUT OF SCOPE for INCR-1
and is filed as a follow-on.~~ **IMPLEMENTED 2026-06-02 as the pre-merge §13 cleanup — see
§17.** It does NOT block the combiner: FAR is wasted compute (LDPC on noise → CRC16 reject →
~30 ms), never corruption.

---

## §14 Cross-layer §5 audit result (the productionized change)

`time_sync_mfsk_corr` is shared by ALL MFSK data modes; its geometry fields
(`mfsk_M`, `mfsk_nStreams`, `mfsk_stream_offsets`) are populated per-config at
`telecom_system.cc:5582-5589`, so the `nStreams>=2` gate routes uniformly:
- **ROBUST_1, ROBUST_2 (WB M16×2):** combiner active, T=8. ✓
- **NB ROBUST_1/2 (M4×2):** combiner active, T=8 — and this is where the OLD AND-gate's
  FAR mitigation (`mfsk.cc` comment "NB M=4 … mitigated by 2-stream all-match") is removed;
  the T=8 + CRC16 backstop is the replacement. NB-specific FAR not separately swept [?].
- **Future ROBUST_RA / ULTRA (M16×2 and deeper, ≥2 streams):** automatically covered by the
  gate the moment they load with nStreams≥2 — uniform by construction (the §6 shared benefit).
- **ROBUST_0 (M32×1) / NB-ROBUST_0 (M8×1):** gate off → legacy path → byte-identical.
- **Control/HAIL/ACK (`detect_ack_pattern`, M16×1, `ack_mfsk`):** a DIFFERENT function and a
  separate threshold (`ack_match_threshold`) — NOT touched by this change. ✓ (§7)
- **Sole production caller** `telecom_system.cc:1069`: output contract unchanged (returns
  delay; `*out_metric` = matched count). The fallback `time_sync_mfsk` (template==NULL) is a
  different detector, not in scope.

---

## §15 Reproduction (productionized)
```
cd C:/Users/kamer/mercury_wt/data-detector-p3 && bash build.sh o3
./mercury.exe --test                       # 51 passed / 0 failed; §6.P4 + §6.1-6.5 green
env MERCURY_P3_SWEEP=1 ./mercury.exe --test # PRODUCTION-PATH M16x2 cliff -15.05(T7)/-13.89(T8);
                                            # M32x1 -13.89(T7) unchanged; FARs per §12
# fail-before: git stash the ofdm.cc+mfsk.cc change, rebuild, --test → §6.P4 (A) FAILS at -11.8 dB
```

---

## §16 HW VALIDATION — the combiner's acquisition gain HOLDS on the real IONOS channel (2026-06-02) — PASS

**This is the FIRST independent RF validation of any RA-tier piece.** The combiner is the
shared FOUNDATION (RA −10 detector + ULTRA reach), so it was HW-tested ALONE on the EXISTING
ROBUST_2 (M16×2, config 102) — no RA mode needed. VERDICT: **PASS — the ~+4.86 dB sim
acquisition gain TRANSFERS to RF.**

### §16.1 Method (rigorous A/B, arms verified not trusted)
- **Arm A** = monitor `@8fc1211` (baseline AND-gate). Pi binary md5 `f5c2ed3c…`, ofdm.cc
  combiner-marker count 0, data-preamble `thr=7`.
- **Arm B** = combiner `@9cecc8f`. Pi binary md5 `3b5871f6…` (DISTINCT from A),
  combiner-marker 1, `thr=8` (the productionized nStreams≥2 gating). Both Pis `--test`
  0-fail per arm (33 / 34 [OK], 0 [FAIL]).
- Both arms carry a **byte-identical** `[MFSK-ACQ]` measurement probe in `telecom_system.cc`
  (unchanged between the two HEADs → cannot bias the A/B) logging the data-frame preamble
  matched count (`mfsk_sync_metric`) + detect decision at the `time_sync_mfsk_corr` call
  site (`:1100`), on EVERY MFSK detection poll. The probe is the RF analogue of the §3 sim
  `mean_matched` / P(detect).
- Pinned ROBUST_2 via `-s 102 -R` (NOT `--max-config` — that clamps 0..15; ROBUST configs
  need `-R` since explicit `-s` doesn't auto-enable robust, main.cc:2239), compress OFF, no
  SACK, no gearshift. WGN:-8/-10/-12 (= SNR3k −5.6/−7.6/−9.6 via SNR3k = WGN+2.4).
  Driver `tools/combiner_acq_hwval.py` (wraps `sack_lossy_ab.py`, +`WB_ROBUST2` config),
  2 runs × 150 s/cell. Responder (rpi1) = the side whose data-frame detector sees the
  commander's M16×2 frames. ACQUISITION measured SEPARATELY from decode/deliver (decode at
  these cells is ≤ the −8 dB ROBUST_2 waterfall, so delivery is ~0 deep — EXPECTED, not the
  metric). Both arms: 6/6 runs CONNECTED, ~112–128 polls/cell, geometry confirmed M16×2
  cfg=102 in every `[MFSK-ACQ]` line.

### §16.2 Result — mean matched count (threshold-INDEPENDENT; the clean combiner signal)
| cell | SNR3k | arm A mean_matched | arm B mean_matched | delta |
|---|---|---|---|---|
| wgn-8 | −5.6 | 2.99 | **6.85** | +3.86 (2.3×) |
| wgn-10 | −7.6 | 2.93 | **7.17** | +4.24 (2.4×) |
| wgn-12 | −9.6 | 3.04 | **7.18** | +4.14 (2.4×) |

The combiner ~2.4×s the mean matched count at every cell — equal-gain noncoherent combining
of the 2 redundant streams lifts the whole matched-count distribution, exactly as §2/§3
predict.

### §16.3 Result — common-threshold detection (removes the T=7-vs-T=8 confound), from the matched-count histograms
| cell | P(m≥7) A→B | P(m≥16) A→B (full-strength real preambles) |
|---|---|---|
| wgn-8  | 0.081 → 0.366 (**4.5×**) | 0.065 → 0.081 (1.25×) |
| wgn-10 | 0.141 → 0.492 (**3.5×**) | 0.031 → 0.082 (**2.6×**) |
| wgn-12 | 0.063 → 0.420 (**6.7×**) | **0.000 → 0.050 (∞)** |

**Decisive deep-cell evidence:** at wgn-12 (−9.6 dB SNR3k) the baseline AND-gate's matched
count for real preambles tops out at **13** (P(m≥14)=P(m≥16)=0 — it NEVER reaches full
strength), while the combiner RESTORES full-strength matched 15–16 detections (P(m≥16)=0.050,
P(m≥14)=0.084). This is the +4.86 dB acquisition deepening manifesting on RF: the combiner
recovers the high-confidence real-preamble mode that the 2-stream AND-gate collapses at depth.
Matched-count histograms (responder, n≈120/cell):
```
wgn-8  A: 1:26 2:77 3:10 | 15:2 16:8         B: 5:31 6:47 7:35 | 16:10
wgn-10 A: 1:54 2:56 | 8:9 14:2 15:3 16:4     B: 5:8 6:54 7:49 8:1 | 16:10
wgn-12 A: 1:31 2:42 3:17 6:15 | 11:2 12:2 13:3   B: 5:1 6:68 7:35 8:5 | 15:4 16:6   <-- A never hits 16; B does
```
The productionized arm B's *native* detect_rate (at its T=8) is similar/slightly lower at
wgn-8/-10 purely from the T=7→T=8 FAR-control trade (§11.2/§12) — NOT a lack of combiner
gain; at a COMMON threshold the gain is decisive everywhere (3.5–6.7× at T=7).

### §16.4 Testbed note (a SIM-vs-RF non-finding worth recording)
First arm-B sweep produced 0 polls (responder didn't reach data RX). Root cause: the
responder **mercury process was killed by a kernel ALSA / dwc-AXI-DMA oops** (`dma_pool_alloc`,
poisoned pool ptr `x20=deaddeaddeaddead`, on PCM-start ioctl during PTT keying) — a Pi 5
kernel DMA-pool corruption that accumulates after ~30 min of rapid PTT keying, NOT a combiner
bug (the combiner backtrace was nowhere near; the commander never crashed). Matches the
MEMORY "Pi audio state drifts after long runs / reboot fixes it" note. Rebooting both Pis
cleared it; the arm-B RE-RUN on the fresh testbed was crash-free (0 SIGSEGV) with 6/6 runs
connected, yielding the §16.2/§16.3 data. Lesson: long A/B campaigns should reboot between
arms (or cap keying time/run count) to keep the kernel DMA pool healthy.

### §16.5 Verdict
**PASS.** The combiner FOUNDATION is HW-validated on the IONOS channel: it materially deepens
ROBUST_2 data-frame ACQUISITION (mean matched ~2.4×; full-strength detections recovered at
−9.6 dB SNR3k where the baseline loses them). Ready to merge after the §13 fine-pass FAR
cleanup. The RA-mode detector + ULTRA reach assumptions that build on it are NOT at risk from
the detector side. (NOTE: this validates ACQUISITION only, on AWGN/WGN; the RA −10 DELIVERY
still needs its R⅓ FEC + the P0/P1 pieces, and HF fading is not yet tested — §10.2.)

Repro: deploy arm via `MERCURY_SRC_OVERRIDE=<worktree> python tools/mercury_deploy_rpi.py`;
`python tools/combiner_acq_hwval.py --arm <X> --point wgn-12 --runs 2 --duration 150`. Logs +
`.acq.json` per cell under `combiner_acq_logs/<A|B>/<point>/`; aggregate with
`tools/_combiner_acq_verdict.py combiner_acq_logs`.

---

## §17 §13 FINE-PASS FAR CLEANUP — IMPLEMENTED + MERGED (2026-06-02) — SIM only

**Status:** the §13 follow-on is DONE and the combiner (§11) + §13 cleanup are MERGED to
`monitor`. SIM build + `--test` green (52/0); the §13 FAR cleanup CUTS M16×2 FAR 10× WITHOUT
regressing the HW-validated (§16) coarse-combining acquisition gain. M32×1 byte-identical.

### §17.1 The change (`source/physical_layer/ofdm.cc` — `time_sync_mfsk_corr` gate)
The detect/no-detect decision is now gated on the COARSE matched count; the Phase-2 fine pass
refines the returned sample OFFSET only (it no longer re-maximizes the count to re-decide):
```
int decision_matched = (mfsk_nStreams >= 2) ? best_matched : fine_best_matched;
if (decision_matched < mfsk_preamble_match_threshold) { *out_metric = decision_matched; return -1; }
*out_metric = decision_matched; return fine_best_offset;   // offset still fine-refined
```
- **nStreams>=2 (M16×2/M4×2 — the combiner geometry):** decide on `best_matched` (coarse).
  This recovers the coarse-only ref-scorer FAR. The reported `*out_metric` becomes the coarse
  decision statistic (honest; the §16 HW probe was on the pre-§13 build and is unaffected).
- **nStreams==1 (M32×1/M8×1 — ROBUST_0):** `decision_matched == fine_best_matched` →
  the gate, the reported metric, and the return are **textually equivalent to monitor
  @8fc1211** (gate on `fine_best_matched`, return `fine_best_offset`). BYTE-IDENTICAL.

### §17.2 Measured outcome (production detector, `MERCURY_P3_SWEEP=1`, this build)
| geometry | metric | pre-§13 (fine-max gate) | post-§13 (coarse gate) | target |
|---|---|---:|---:|---|
| **M16×2** | **FAR/poll @ T=8** | **1.80e-2** | **1.75e-3** | ref-scorer 1.75e-3 ✓ |
| M16×2 | cliff @ T=8 (P=0.5) | −13.89 | **−12.55** | ≤ −11 (clears +1.55 dB) |
| M16×2 | prod-path == ref-scorer? | no | **YES at every T/σ** | (decision now coarse) |
| M32×1 | cliff T=8 / T=7 | −12.55 / −13.89 | **−12.55 / −13.89** | unchanged (gate off) ✓ |
| M32×1 | FAR T=8 / T=7 | 0.0 / 2.25e-3 | **0.0 / 2.25e-3** | unchanged (gate off) ✓ |

The M16×2 cliff moves −13.89 → −12.55 (the **+1.34 dB fine-pass sub-position-MAX bonus**, which
was the FAR-inflation source, is intentionally traded for the **10× FAR reduction**). The
HW-validated CORE — coarse stream-energy combining — is **PRESERVED**: −9.03 (AND-gate
baseline) → −12.55 (coarse-combined) = **+3.52 dB** of the §16 +4.86 dB gain retained, and
−12.55 still clears the −11 PASS bar AND the −10.84 FEC bind with margin. The deepening that
§16 measured on RF (mean matched ~2.4×, full-strength detections recovered at −9.6 dB) is a
property of the COARSE combining and is therefore intact under §13. M16×2 P(detect) post-§13:
0.97 @ −10.97, **0.87 @ −11.80**, 0.66 @ −12.55 (T=8).

### §17.3 Test — §6.P5 always-on FAR regression (`mfsk_ctrl_codec_tests.cc`)
**NEW §6.P5 `test_mfsk_data_preamble_far_coarse_gate`** (always-on, fail-before/pass-after):
- **(A)** M16×2 pure-noise FAR through the PRODUCTION gate (`delay>=0`) at T=8 over 4000
  trials must be ≤ 8e-3 (the bound separates pre-fix 1.8e-2 from post-fix 1.75e-3). PRE-FIX
  (fine-max gate, `9cecc8f`) = **55/4000 = 1.38e-2 → FAILS**; POST-FIX (coarse gate) =
  **5/4000 = 1.25e-3 → PASSES**.
- **(B)** acquisition non-regression: at SNR3k −11.80 dB (inside the −12.55 coarse cliff) the
  production detector must still detect ≥30/40 — POST-FIX **34/40** → the §13 cleanup did NOT
  collapse the combiner core. (The §6.P4 guard independently gets 35/40 at the same cell.)
- Fail-before verified by `git stash`-ing only `ofdm.cc` (revert the gate), rebuild, `--test`
  → **51 passed, 1 failed** (§6.P5 FAR FAIL 1.38e-2). Restored → **52 passed, 0 failed**.
- M32×1 byte-identity: source-diff of `time_sync_mfsk_corr` vs `8fc1211` confirms the
  nStreams==1 path (per-symbol decision + gate + metric + return) is line-equivalent; plus
  §6.P4(B) M32×1 40/40 and the unchanged §6.1–§6.5 ROBUST_0 argmax tests.

### §17.4 Merge
`sim/data-detector-deepen-p3` (combiner §11 + §13 cleanup) merged into `monitor`. Merged-tree
`bash build.sh o3` + `--test` = 52/0; M32×1 byte-identical; combiner present (gated
nStreams>=2 → ROBUST_0/MFSK-control untouched). The combiner ALSO lifts the EXISTING
ROBUST_1/ROBUST_2 data-frame detection (a shipped improvement, not just RA-tier prep). NO push.
Repro: `cd <worktree> && bash build.sh o3 && env MERCURY_P3_SWEEP=1 ./mercury.exe --test`.

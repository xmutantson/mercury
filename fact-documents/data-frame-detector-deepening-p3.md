# P3 GATE — can the MFSK DATA-FRAME detector reach ≤ −11 dB SNR3k so the GF16-RA R⅓ FEC (−10.84) shows in production, at >71 bps?

**Status:** P3 MAKE-OR-BREAK gate for the WIN CAMPAIGN (b) −10-rate win. SIM ONLY (dev
host; NO bench — v13 owns the IONOS). **VERDICT: PASS — the detector is NOT the binding
constraint once the M16×2 2-stream combiner is fixed.** The shipped discrete-match data
detector already reaches −13.89 dB on M32×1; on the M16×2 ROBUST_RA geometry the *production*
detector cliffs at −9.03 dB (a 2-stream AND-gate bug), but a one-fix **stream-energy
combiner** moves it to **−13.89 dB (+4.86 dB)** — deeper than the GF16-RA FEC reach
(−10.84), so the FEC, not the detector, becomes the binding constraint. The −10 mode keeps
**103 bps wire** (R⅓ M16×2, P0 §3) > VARA's 71.

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
| `ofdm.cc:3556` | the 2-stream AND-gate (`streams_matched < mfsk_nStreams → continue`) — the cliff driver |
| `ofdm.cc:3680` | detection gate `fine_best_matched < mfsk_preamble_match_threshold` |
| `mfsk.cc:518-522` | `generate_preamble` places the SAME tone in both streams (redundant → combinable) |
| `mfsk.cc:149-168,221` | preamble tones (Welch-Costas g=2 ×2 reps) + threshold=7/16 |
| `telecom_system.cc:1069` | sole production caller |
| `telecom_system.cc:5582-5589` | per-config population of the detector's mfsk_* fields |
| `mfsk_ctrl_codec_tests.cc` §6.P3 | this measurement (env `MERCURY_P3_SWEEP`) |
| `mfsk_ctrl_codec_tests.cc:2843` | `snr3k_db` axis (shared with all campaign numbers) |

## §10 Open questions [?]
1. **[?] FAR of the combined detector at T=8/9 on M16×2** — not swept; the T=4..7 trend
   suggests T=8 restores FAR≈0 while keeping most of +4.86 dB. Measure before productionizing.
2. **[?] AWGN only** — no fading/CFO. HF Doppler will cost the noncoherent floor more (P0 §5b);
   the combiner gain is an AWGN array gain and should largely hold, but HW A/B (post-P4) confirms.
3. **[?] M32×1 byte-identical under a productionized combiner** — §3.1 shows a −1.34 dB
   difference in the *combined* scorer at 1 stream (mirror-tone argmax). Gate behind
   `nStreams>=2` to keep ROBUST_0 untouched.

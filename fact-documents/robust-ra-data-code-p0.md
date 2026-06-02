# P0 GATE — does GF(16)-RA transfer from the K=13 suffix to a DATA-length block and hold a cliff ≤ −10 dB SNR3k at ≥71 bps?

**Status:** P0 MAKE-OR-BREAK gate for the WIN CAMPAIGN (b) −10-rate win. SIM ONLY (dev
host; NO bench — v13 owns the IONOS). **VERDICT: MARGINAL/NEGATIVE at R½ — the codec
TRANSFERS, but the R½ data-length cliff lands AT the −10 line, not below it.**
**Date:** 2026-06-02. **Worktree:** `C:/Users/kamer/mercury_wt/robust-ra-data-p0`,
branch `sim/robust-ra-data-p0` off monitor @8fc1211.
**Conventions:** numbered sections (§N), `file.cc:line` citations, `[?]` for unknowns.

This doc reconciles the WIN-campaign scoping claim (GF16-RA R½ ≈ −11.75 transfers to a
data mode ≈ 135 bps @ −11.75, clears −10) against the prior NEGATIVE
(`sim/robust3-gf16ra @a0ee58e`, "binary LDPC wins by 2 dB"). Both prior numbers were on
DIFFERENT axes/block-lengths; THIS measures the data-length cliff on the SAME PHYSICAL
passband axis as the −11.75 suffix number so the −10 comparison is apples-to-apples.

---

## §1 The two prior measurements were NOT in contradiction (the reconciliation)

| source | block | code | cliff | SNR3k axis | baseline |
|---|---|---|---|---|---|
| suffix spike §8.4 (`tier2-suffix-fec-gf16-spike.md`) | K=13 | GF16-RA R½ | **−11.75** | physical passband `snr3k_db` | uncoded/Tier-1 |
| failed branch §6.2 (`robust3-gf16ra-data-code-feasibility.md`) | K=200 | GF16-RA R½ | **−7.0** | analytical Eb/N0 + (−16.0) anchor | binary LDPC same model |

The −11.75 vs −7.0 gap is TWO compounding factors, neither an error:
1. **Block length** (K=13 vs K=200): the non-binary short-block advantage shrinks as the
   block lengthens (the failed-branch thesis §6.3, cited Chang & Divsalar NB-LDPC
   short-block near-Shannon; advantage evaporates past ~N=600 bits).
2. **SNR3k axis convention** (~2–3 dB): the failed branch uses an ANALYTICAL
   `ebn0db_to_snr3k = ebn0 + 10log10(4R) − 16.0` (the −16.0 anchored to the binary
   ROBUST_3 +5.89 Eb/N0 → −7.1 SNR3k) on idealized `signal+CN(0,1)` energies
   (`mfsk_ctrl_codec_tests.cc:3655-3658` on a0ee58e). The suffix spike + this work use the
   PHYSICAL passband convention: `snr3k_db = 10log10(p_sig/(σ²·3000/(fs/2)))` where `p_sig`
   is the measured passband power of the synthesized signal (`mfsk_ctrl_codec_tests.cc:2833,2843`).
   The failed branch itself noted its model is ~2 dB off the §8.4 passband (its K=13 R¼
   = −12.0 vs §8.4's −14.03). **This work confirms the offset: my K=13 R½ on the physical
   axis = −11.91, matching the suffix −11.75 within 0.16 dB; the failed branch's −7.0 + ~3 dB
   axis-offset ≈ my −10.12 physical at K=200. The two AGREE once the axis is unified.**

**The scoping agent's error:** assuming the K=13 −11.75 transfers UNCHANGED to data length.
It does not — the cliff degrades ~1.8 dB going K=13→K=200 at fixed R½ (measured below).

---

## §2 Method (genie-sync FEC-reach on the physical axis; CLAUDE.md §1/§3)

New test `test_gf16_ra_data_length_cliff_sweep` (`mfsk_ctrl_codec_tests.cc` §20). Reuses
the K-generalized codec ported from a0ee58e (`gf16ra::configure_k/encode_k/soft_decode_k`,
verified-correct per failed-branch §6.1: corrects 100/800 symbols at R¼ P=1.00) — the
decoder uses the **Bessel-I0 intrinsic** (`log_i0_approx`, `mfsk_ctrl_codec.cc` soft_decode_k),
NOT the square-law bit-LLR demap (`mfsk.cc:1006-1143`, the BICM path the task says to avoid).

- **TX (genie):** K-symbol GF(16)-RA codeword laid one tone/symbol onto N = K + repfact·K
  OFDM-MFSK symbols on `ack_mfsk` (M=16, nStreams=1, the real WB M=16 noncoherent-FSK PHY,
  `telecom_system.cc:5835-5836`). amp `sqrt(Nc/nStreams)`, tone-hop, `symbol_mod`,
  power-normalize, `baseband_to_passband`, `peak_clip` — byte-for-byte the suffix synthesis
  (`build_gf16ra_suffix_audio`, mfsk_ctrl_codec_tests.cc:3175) minus the CONNECT base pattern
  (a data frame has none). Builder = `build_gf16ra_data_audio`.
- **CH:** per-sample passband AWGN at σ.
- **RX (genie):** `passband_to_baseband_decimated` (center-tap aligned, zero net group delay,
  `fir_filter.cc:250/268`) → `decode_suffix_energies(pattern_offset=4096/interp,
  pattern_nsymb=0, suffix_len=N)` extracts the N×16 per-tone energy matrix at genie symbol
  offsets (NO detection scaffolding — the §16-exonerated FEC-reach criterion, same as the
  suffix spike's "(B) FEC reach") → `gf16ra::soft_decode_k` (Bessel-I0). Frame OK iff ALL K
  info symbols match. **PRIVATE decimation buffer sized to dec_size** — `data_container`'s
  `baseband_data_interpolated` is sized `buffer_Nsymb~32` at CONFIG_0 and would overflow at
  N up to 800 (the §5.2(1) "frame-geometry silent corruptor"; the v1 harness hit exactly
  this → P=0 everywhere; fixed).
- **Self-validation guard:** each cell first decodes the CLEAN (σ=0) reference; if the genie
  offset is wrong this is 0% even at ∞ SNR. ALL cells report 13/13, 200/200, 400/400 OK →
  genie offset VALID (guards against mistaking a harness bug for a code FAIL).
- **Axis:** `snr3k_db(p_sig, σ, fs=48000)` — bit-identical to the −11.75 suffix axis.
- **bps:** K info symbols × 4 bits over N/nStreams symbol periods of 25.83 ms each (one
  OFDM block = Nofdm=310, interp=4, the CONFIG_0/ack_mfsk GI). nStreams=1 = 1-stream;
  the M16×2 ROBUST_3 geometry (nStreams=2) DOUBLES bps at the same per-tone SNR.

---

## §3 RESULTS (measured; fine grid 0.1-σ steps, NTR=200; physical snr3k_db axis)

Reproduce: build `sim/robust-ra-data-p0`; `MERCURY_P0_FINE=1 mercury.exe --test` →
test `gf16_ra_data_length_cliff_sweep`. Both coarse (NTR=60) and fine runs:
**51 passed, 0 failed; build clean.**

Cliff = linear-interpolated P(frame-decode)=0.5 on the physical SNR3k axis:

| block | rate | N | **cliff (interp P=0.5)** | net bps 1-str / 2-str | vs −10 | vs suffix −11.75 |
|---|---|---|---|---|---|---|
| **K=13** (suffix cross-check) | ½ | 26 | **−11.91** | 77 / 155 | −1.91 (clears) | −0.16 ✓ matches |
| **K=200** (= 800 info bits, data) | ½ | 400 | **−10.12** | 77 / 155 | **−0.12 (barely clears)** | +1.63 |
| **K=400** (= 1600 info bits, data) | ½ | 800 | **−9.77** | 77 / 155 | **+0.23 (MISSES)** | +1.98 |
| K=200 | ⅓ | 600 | −10.84 (coarse) | 52 / 103 | −0.84 (clears) | +0.91 |
| K=200 | ¼ | 800 | −13.34 (coarse) | 39 / 77 | −3.34 (clears) | −1.59 |

**Transfer cross-check VALIDATES:** K=13 R½ on this genie/physical axis = −11.91, matching
the suffix spike's −11.75 (within 0.16 dB; the residual is grid/trial noise). So the harness
is faithful and the GF(16)-RA codec **DOES transfer** structurally from the suffix to data
length (decoder healthy: clean-decode 100% at K=13/200/400; the failed-branch §6.1 capability
result reproduced).

**But the R½ cliff degrades with block length** (the short-block-advantage loss, qualitatively
confirming the failed-branch thesis but ~1.8 dB, NOT the ~4-5 dB its analytical model implied):
−11.91 (K=13) → −10.12 (K=200) → −9.77 (K=400). At the mandated data lengths (200-400), **R½
straddles −10**: it does not HOLD a cliff ≤ −10 with margin — it sits on the line and slides
below the threshold as the block grows.

---

## §4 VERDICT — MARGINAL/NEGATIVE on the gate as written; the −10-rate WIN at R½ is NOT clean

**P0 PASS bar = "hold a cliff ≤ −10.0 dB SNR3k at ≥71 bps net wire" at R½.**

- **≥71 bps: MET** (R½ = 77 bps 1-stream, 155 bps 2-stream).
- **cliff ≤ −10 at R½: FAILS the "hold a cliff" bar.** K=200 = −10.12 (clears by 0.12 dB, i.e.
  inside harness noise of the boundary); K=400 = −9.77 (misses by 0.23 dB). R½ at data length
  is **AT −10, not below it**. It does not "hold a cliff ≤ −10" — it sits on the line.

**To clear −10 with real margin you must DROP RATE**, which trades against bps:
- R⅓ (N=600): −10.84, but 52 bps 1-stream (only ≥71 at 2-stream = 103 bps).
- R¼ (N=800): −13.34 (clears by 3.34), but 39 bps 1-stream (only ≥71 at 2-stream = 77 bps).

**⇒ The −10-rate WIN is achievable ONLY by combining (a) a sub-½ rate (≤⅓) AND (b) the M16×2
two-stream geometry to recover bps.** E.g. R⅓ + M16×2 = −10.84 @ 103 bps clears both bars;
R¼ + M16×2 = −13.34 @ 77 bps clears with comfortable margin. The clean R½ "≈135 bps @ −11.75"
the scoping agent projected is NOT real at data length — that was the K=13 suffix number.

**Reconciliation with the failed branch (`a0ee58e`):** NOT contradicted. (1) The failed branch
measured the *relative* binary-LDPC-beats-GF16-by-2-dB at K=200 on its analytical axis — a
fair, convention-independent delta for the *code-vs-code* question (it argued against swapping
ROBUST_3's binary LDPC). THIS work answers a DIFFERENT question (the GF16 code's *absolute*
cliff vs the −10 target on the physical axis) and finds R½ at −10.12/−9.77. (2) The failed
branch's −7.0 absolute + the measured ~3 dB analytical→physical axis offset ≈ this work's
−10.12 — **the two AGREE on the physics**; they differ only in axis convention and in the
question asked. The failed branch's conclusion ("GF16 doesn't beat binary at data length, the
residual is the noncoherent M-FSK floor") STANDS; this work adds that the GF16 code's physical
cliff at R½ data length is ~−10, i.e. ON the target line, clearing it only at lower rate.

---

## §5 Decision input for P1-P6

- **GO, but at R≤⅓ + M16×2, NOT R½.** The −10-rate win is feasible *if* the production
  ROBUST_RA family is designed at rate ⅓ (−10.84, 103 bps) or ¼ (−13.34, 77 bps) on the M16×2
  two-stream geometry — both clear ≤ −10 at ≥71 bps. A rate-½ ROBUST_RA does NOT hold ≤ −10.
- **The scoping "≈135 bps @ −11.75" target is unreachable** — it conflated the K=13 suffix
  cliff with a data-mode cliff. Realistic: ~77-103 bps @ −10.8 to −13.3 (R¼-R⅓, M16×2).
- **vs VARA:** the failed-branch frame put VARA L4 at −10 SNR3k / ~175 raw bps. A ROBUST_RA at
  R¼/M16×2 (−13.34, 77 bps) reaches DEEPER than VARA's −10 but at <½ the bps; R⅓/M16×2
  (−10.84, 103 bps) is ~parity on SNR with ~60% the bps. The win is REACH, not rate (consistent
  with MEMORY's "Mercury wins on REACH not rate").
- **Caveats [?]:** (a) genie-sync FEC-reach — excludes the detection/sync scaffolding that
  §8.3/§11/§16/§17 showed pins the END-TO-END cliff ~3 dB shallower in production (the
  data-frame preamble + Schmidl-Cox/coarse sync, a SEPARATE P3 work item — "detection
  scaffolding"). The production cliff will be SHALLOWER than these FEC-reach numbers until
  that is fixed. (b) AWGN only — no fading/CFO (HF Doppler will cost more; the design floor is
  noncoherent square-law M-FSK). (c) the K-generalized RA graph is generic machine-generated,
  not hand-optimized like Q65's K=13 tables — a tuned NB-LDPC/QRA-at-length might buy a
  fraction of a dB (failed-branch §7), not enough to move R½ comfortably below −10.

---

## §6 Build / test health, scope (CLAUDE.md §5)

- **`mercury --test` = 51 passed / 0 failed** (coarse + fine). Build clean (only pre-existing
  WASAPI `%d`/DWORD format warnings).
- **Byte-identical when off:** the ported codec adds `gf16ra::configure_k/encode_k/soft_decode_k`
  with INDEPENDENT graph storage (`g_k_*`) — the K=13 ctrl `configure()/encode()/soft_decode()`
  path is untouched, all 50 prior ctrl tests pass. New test is MEASURE-only (no production
  caller, no wire change). Default build identical to monitor 8fc1211.
- **NO production wiring** (P0 is the gate; P1 Bessel-I0 data demap / P2 the ROBUST_RA family /
  P3 detection scaffolding follow on a GO). NO bench used.

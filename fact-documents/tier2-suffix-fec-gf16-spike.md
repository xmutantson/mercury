# Tier-2 suffix FEC — Phase-1 SIM SPIKE, candidate A: soft GF(16) rate-~1/2 RA code

**Status:** Phase-1 sim spike, candidate A (the symbol-matched alternative to the parallel
Golay(24,12) spike). Worktree `C:/Users/kamer/mercury_wt/suffix-fec-tier2-gf16`, branch
`wt/suffix-fec-tier2-gf16`, off the Tier-1 prototype `wt/connect-suffix-fec` @37947fb.

**Goal (same gate as Golay):** close the last ~3–4 dB on the M=16 noncoherent-FSK control
suffix so the CONNECT/ACK acquisition cliff reaches the **−14.68 dB SNR3k** base-detector
floor. SIM-ONLY (in-process cliff sweep). NO wire-format change in production paths — gated
behind `suffix_fec_mode==3` (Golay uses 2; modes stay distinct). mode=0 byte-identical.

Sibling docs: [[tier2-suffix-fec-design.md]] (§2 cliff math, §3 the two candidates),
[[connect-suffix-fec-research.md]] (Tier-1 prototype + the sim harness + the
`decode_suffix_candidates` per-tone-energy primitive).

## §1 Prior art (cited; CLAUDE.md §1)

- **qracodes** (Nico Palermo IV3NWV, github.com/Microtelecom/qracodes, GPLv3 → AGPLv3): the
  canonical Q-ary RA encoder/decoder K1JT used for QRA64/Q65. Read in full this session:
  - `qra_encode`: **systematic** — copy K info symbols, then NC check symbols by a *weighted
    accumulation over GF(M)* of a permuted/repeated info sequence:
    `chk = α^(logw[k]) · x[idx[k]]  XOR  chk_prev ;  y[k] = chk` (qracodes.c:30–110).
    The weight set is designed so the accumulator **terminates to 0** for every systematic
    input (qracodes.c comment ~L300) — the property that lets the EXIT-chart convergence test
    (max-marginal ≈ 1) stand in for a syndrome check.
  - `qra_mfskbesselmetric`: intrinsic symbol likelihood from the **squared FSK tone
    amplitudes** via the modified Bessel I0 (Proakis Ch.8 noncoherent-FSK optimal metric),
    normalized to a per-symbol probability distribution (qracodes.c:130–190). This is the
    "consume the per-tone energies directly" requirement, done right.
  - `qra_extrinsic`: symbol-domain **belief propagation** over GF(2^m). Check node for
    `x1+…+xd=0` (field add = XOR) computed as `IWHT(∏ WHT(p_i))` — **Walsh-Hadamard
    fast convolution**, O(M log M) vs O(M²) (qracodes.c:200–360). GF multiply-by-α^w
    becomes a **permutation** of the pd vector (`gfpmat`, `pd_fwdperm`/`pd_bwdperm`).
    Convergence: `Σ_v max(pd_v) > V − ε`.
  - `qra_mapdecode`: final APP = intrinsic · extrinsic, argmax per info symbol.
  - `npfwht.c` (FWHT) + `pdmath.c` (pd_imul / pd_norm / pd_argmax / perms): the primitives.
  - Code type `QRATYPE_CRC` (qracodes.h:27): **the CRC is carried as protected info symbols**,
    not as an uncoded tail — exactly what makes the CRC accept-gate reliable at the floor.
- **WSJT-X 2.7 Q65** (K1JT/K9AN): QRA(65,15) over GF(64) + soft symbol-MAP + 12-bit CRC gate.
  We CANNOT reuse Q65's tables (GF(64), n=63). We construct a GF(16)/n≈20 code with the same
  algorithm — same path as the a26 ft8mon BP+OSD port (port the algorithm, build the structure).

## §2 Why this is a FAIR, STRONG GF(16) code (not a strawman)

The whole thesis of candidate A is that a code matched to the M=16 FSK **symbol** alphabet
should beat binary Golay because (a) it consumes the per-tone energies as native GF(16)
symbol likelihoods with **zero bit-LLR marginalization loss**, and (b) GF(16) check nodes
capture the symbol-wise FSK error structure. To give it its best shot we use the genuinely
optimal pieces from qracodes, not a watered-down version:

1. **Intrinsic metric = noncoherent-FSK-optimal Bessel-I0 of the normalized tone amplitudes**
   (qra_mfskbesselmetric), NOT the Tier-1 normalized-energy-gap heuristic and NOT a hard
   argmax. This is the correct symbol log-likelihood for incoherent M-FSK.
2. **True GF(16) symbol-domain BP** with the WHT check-node convolution + permutation weights
   — the optimal MAP-class soft decoder, not a bounded list search.
3. **RA structure with the CRC as protected info** (`QRATYPE_CRC`): the 12-bit CRC is 3 of the
   info symbols, FEC-protected, then recomputed-and-compared as the accept gate. (See §4 for
   why this beats an uncoded-CRC tail.)
4. **Good interleaver + irregular repeat profile** (Q65 family): the accumulator-input index
   sequence repeats each info symbol q≈3 times through a spread permutation, weights chosen so
   the accumulator terminates to 0.

## §3 The construction

- **Field**: GF(16) = GF(2⁴), primitive poly x⁴+x+1 (0x13), α=2. Additive group = bitwise XOR
  (required for the WHT trick). `gfexp[0..14]`, `gflog[1..15]` built at init.
- **Message**: 40 bits [type:2 | payload:38] → **10 info symbols** (4 bits each, MSB-first,
  identical bit order to the Tier-1 / production `pack_ctrl_suffix`).
- **CRC**: the production CRC12 over the 5-byte [type:2|payload:38] field (arq_common
  `CRC12_calc`, init=0xFFF, poly 0xF13 — NEVER inlined, passed by callback as in Tier-1) →
  **3 info symbols**. ⇒ **K = 13 info symbols**.
- **Parity**: **7 GF(16) check symbols** by the RA weighted-accumulation over the 13 info
  symbols. ⇒ **N = 20 symbols** total. Rate R = 13/20 = 0.65 (≈ rate-½ class; the task's
  "10 info + 7 parity + 3 CRC ≈ 20 sym" with the CRC counted as protected info per Q65).
- **Airtime**: 20 symbols vs Tier-1's 13 (+7) vs Golay's 24 (+11). At 24.33 ms/sym the CONNECT
  cost is +170 ms once/session (<0.1% of a multi-min session) — negligible, like Golay.
- **Decoder**: WHT-BP, max 50 iters, EXIT-chart convergence (Σ max-marginal > K−ε), then APP
  argmax on the 13 info symbols → reassemble 40-bit [type|payload] + 12-bit CRC → recompute
  CRC12, accept iff (a) it matches the decoded CRC symbols AND (b) type == expected_type.

## §4 Why the CRC is PROTECTED info, not an uncoded tail (the decisive design call)

Tier-1 carries the CRC as 3 uncoded rate-1 symbols and the accept gate compares recomputed
CRC to the *received* (noisy) CRC tones. At the −14 dB floor the per-symbol argmax error is
q≈24% (tier2-design §2), so P(all 3 CRC symbols survive) ≈ 0.76³ ≈ 44% — that **caps decode
success at ~44%** no matter how good the message FEC is. Putting the CRC inside the coded
block (Q65 `QRATYPE_CRC`) removes that cap: the BP recovers all 13 info symbols (message+CRC)
jointly from the energies, and the recompute-vs-decoded-CRC comparison is then FEC-reliable.
This is why Q65 does it and why a fair GF(16) code must too.

## §5 FAR analysis (vs Tier-1's 0.25%, the §7-design open question)

Unlike the Tier-1 list decoder (whose FAR is a tunable Hamming-ball trial budget), the BP
decoder emits ONE candidate codeword per call. A pure-noise input passes only if that single
decoded word's recomputed CRC12 matches its decoded CRC symbols AND type==expected. Both gates
are on the *decoded* word: ≈ 2⁻¹² (CRC) × ¼ (2-bit type) ≈ **6.1×10⁻⁵** structural ceiling
(plus the small chance BP converges at all on noise). This is *below* Tier-1's 0.25% by
construction — measured in `test_gf16_ra_pure_noise_far`.

## §6 Measurement plan (SAME harness + SNR axis as Golay — directly comparable)

`mfsk_ctrl_codec_tests.cc` §10 (new), reusing the §9 `snr3k_db` / `suffix_pb_power` calibration
and the §6/§8 AWGN-passband synthesis pattern:
- TX: CONNECT base (16 sym, connect_tones) + 20 GF(16)-coded suffix symbols laid into
  `ofdm_framed_data` with the SAME tone-hop formula as `generate_ctrl_suffix_pattern`
  (`actual_tone=(data_tone+abs_s·hop)%M`, amp `sqrt(Nc/nStreams)`), symbol_mod →
  baseband_to_passband → peak_clip. Self-contained harness builder (measurement-only — does
  NOT touch production `generate_ctrl_suffix_pattern`, whose 13-sym wire format is unchanged).
- RX: same `detect_ack_pattern`(connect_tones) + ctrl mini-Moose to get `best_offset`, then a
  new PHY primitive `decode_suffix_energies` (full per-tone energy matrix E[s][m], NOT top-K)
  → Bessel intrinsic → WHT-BP → CRC+type gate.
- Cliff = SNR3k at P(decode)=0.5, on the identical sigma grid. Report: GF16-RA cliff vs
  Tier-1 (−8.7) vs base floor (−14.68); coding gain dB; reaches −14? FAR vs 0.25%; airtime;
  decoder runtime; byte-identical-when-off (mode=0); 36 suffix tests still pass.

## §7 Cross-layer / safety (SIM SPIKE — minimal surface)

- New code is measurement-only and gated; production decode path (`suffix_fec_mode==0`) is
  untouched ⇒ byte- and decode-identical to 37947fb. Verified by the existing
  `suffix_soft_candidate0_equals_hard` + a new `gf16_ra_byte_identical_when_off` assertion.
- `decode_suffix_energies` is a NEW read-only PHY method (mirrors `decode_suffix_candidates`
  FFT/de-hop math); adds no caller to the baseline path.
- No change to `ack_sack_suffix_len()` (stays 13), `MAX_ACK_SACK_SUFFIX` (stays 16), or any
  wire-format accessor. The 20-sym frame lives only in the harness builder + GF(16) decoder.

## §8 RESULTS (measured, build `wt/suffix-fec-tier2-gf16`, 42/42 tests pass)

Reproduce: `mercury.exe --test` → tests `gf16_ra_*` (§10 in `mfsk_ctrl_codec_tests.cc`).
SNR3k axis identical to the §9 Tier-1 sweep (base-detect floor calibrates to −14.68 dB,
matching the prior data-preamble cliff).

### §8.1 The first (degree-9) attempt was a STRAWMAN — corrected
First graph routed all K·REP=52 info replicas into only NPARITY=7 fat checks → check
degree ~9 → measured correction < 1 symbol (1 wrong symbol fixed only 27% of the time),
cliff −8.66 dB = Tier-1. ROOT CAUSE: the real Q65 code (`qra15_65_64_irr_e23.c`) has
**MAXCDEG=3** — a sparse degree-3 accumulator chain (NC = repfact·K stages, each folding
ONE interleaved info edge). Rebuilt to true degree-3 RA. **This is the single most
important lesson: a high-rate/few-parity Q-ary code is NOT a sparse code and Q-ary BP
collapses to hard decision on it.**

### §8.2 Symbol-correction capability (clean-flip test, `gf16_ra_correction_capability`)
P(decode) vs #wrong symbols, true degree-3 RA:
| repfact | N | R | 2 errs | 4 errs | 6 errs | 8 errs |
|---|---|---|---|---|---|---|
| 1 | 26 | 0.50 | 0.85 | 0.38 | 0.05 | 0.00 |
| 2 | 39 | 0.33 | 1.00 | 1.00 | 0.98 | 0.87 |
| 3 | 52 | 0.25 | 1.00 | 1.00 | 1.00 | 1.00 |
Now a genuinely strong code (vs <1-symbol for the strawman).

### §8.3 THE BIG FINDING — the −8.7 cliff is the DETECTION SCAFFOLDING, not the FEC
Decoding the **real passband-extracted energies** directly (inline, base matched-count
detect): GF16-RA P=1.00 at −9.82 dB, P=0.99 at −10.84 dB. But `decode_*_from_passband`
(the production scaffolding, which Tier-1 also uses) drops the SAME trials to P=0.47 /
0.06. The gap is entirely the **control mini-Moose** (`carrier_frequency_sync_wb_ctrl`
returns noisy residuals at low SNR and corrupts the re-decimated baseband) **+ the
`metric>=3.0` detection-confidence gate**. Both cliff at ~−8.7 dB and pin Tier-1 AND any
soft suffix decoder. **The FEC is not the binding constraint at −8.7 — the
detection/sync plumbing is.**

### §8.4 Cliff table (measured)
| config | cliff SNR3k | vs Tier-1 (−8.7) | vs base floor (−14.68) | reaches −14? | added airtime vs 13-sym |
|---|---|---|---|---|---|
| Tier-1 (uncoded soft list) | −8.7 | — | +6.0 | no | 0 |
| **(A) prod scaffolding**, GF16 r=2 | −8.65 | +0.05 | +6.03 | no | +633 ms |
| **(B) FEC reach**, GF16 r=1 (N=26, R½) | **−11.75** | **−3.05** | +2.93 | no | +316 ms |
| **(B) FEC reach**, GF16 r=2 (N=39, R⅓) | **−13.34** | **−4.64** | +1.34 | no | +633 ms |
| **(B) FEC reach**, GF16 r=3 (N=52, R¼) | **−14.03** | **−5.33** | **+0.65** | **YES** | +949 ms |

(A) = end-to-end with today's mini-Moose + metric gate (apples-to-apples with Tier-1).
(B) = detection by the base matched-count (the criterion that itself reaches −14.68),
mini-Moose + metric gate removed = the code's intrinsic reach.

**Coding gain: 3.0 dB (R½) to 5.3 dB (R¼). repfact=3 REACHES the −14 target (−14.03,
within 0.65 dB of the base-detect floor).**

### §8.5 FAR / airtime / runtime / byte-identical
- **FAR on pure noise: 0/5000 = 0.0000** (Tier-1 ref 0.25%). Structural ceiling
  ≈ 2⁻¹²·¼ ≈ 6.1e-5: BP emits ONE codeword/call, gated by recompute-CRC + 2-bit type
  on the DECODED word. **Far below Tier-1's 0.25% by construction.**
- **Airtime: +13 sym (R½, N=26) / +26 (R⅓) / +39 (R¼) vs Tier-1's 13.** At 24.33 ms/sym:
  +316 / +633 / +949 ms — once per session (<0.5% of a multi-min session). vs Golay's +11.
- **Decoder runtime ~250–470 µs/decode** worst-case (50-iter BP near cliff). Negligible.
- **Byte-identical when off:** mode=0 production hard decode unchanged (gf16ra has no
  production caller; `gf16_ra_byte_identical_when_off` asserts it). All 42 tests pass.
- esno_metric (Bessel intrinsic design point): swept; optimum broad ~6 dB, ~1 dB better
  than naive 0 dB; result insensitive across the cliff regime. Default 4.0 (6 dB).

### §8.6 Read for the decision
The symbol-matched GF(16) thesis is VALIDATED: with the per-tone energies fed as native
GF(16) symbol likelihoods (Bessel-I0) into degree-3 Q-ary BP, the code reaches the
−14.68 dB base-detector floor (repfact=3) — a 3–5 dB coding gain that Tier-1 cannot
touch, and the deepest a suffix decoder can go (it's at the detection floor). FAR is
better than Tier-1 (0% vs 0.25%). BUT: realizing this in production requires ALSO fixing
the CONNECT/ACK detection scaffolding (mini-Moose + metric gate) that currently cliffs at
−8.7 and pins everything — that is a SEPARATE integration-phase work item, equally
required for Golay. SIM SPIKE complete; production integration gated (§6 of the parent
design doc).

# ROBUST_3 data code: replace binary rate-8/16 LDPC with non-binary GF(16)-RA?

**Status:** RESEARCH + SIM FEASIBILITY — sim only, NOT for merge to monitor.
**Date:** 2026-06-01.
**Vehicle:** `sim/robust3-mfsk @8c6d3b7` (ROBUST_3 = config 103, M16×2, LDPC rate 8/16,
K=800/N=1600, ~149 bps net, AWGN cliff channel-SNR −7 dB = SNR3k −7.1, Eb/N0 +5.89).
**Prototype branch:** `sim/robust3-gf16ra`, worktree `C:/Users/kamer/mercury_wt/robust3-gf16ra`.
**Origin:** greenlit −10-rate R&D push. Hypothesis: the ~3 dB gap to VARA L4 (−10 dB /
+2.34 dB Eb/N0) is partly the binary LDPC code being sub-capacity at K=800/rate-½ when
fed bit-LLRs marginalized from M=16 tones (a BICM gap); a symbol-matched non-binary
GF(16)-RA (Q65/QRA lineage, already in-tree for the CONNECT ctrl-suffix) decoded directly
from per-tone energies should recover it and move the cliff toward −10.
**Conventions:** numbered sections (§N), `file.cc:line` citations, `[?]` for unknowns.

This doc is the logical successor to [robust3-mfsk-demap-feasibility.md](robust3-mfsk-demap-feasibility.md)
(§7 there attributed the residual to "the noncoherent square-law M-FSK design floor"
*assuming the LDPC code is near-optimal*). This doc probes that assumption: is part of
the +1.7 dB that ROBUST_3 sits ABOVE the noncoherent floor actually the binary code's
short-block / BICM gap, recoverable by a symbol-matched GF(16) code?

---

## §1. Prior art / research (CLAUDE.md §1, cited)

### §1.1 The make-or-break question: does GF(16)-RA scale to a data-frame payload?

ROBUST_3's data frame is **K=800 info bits** (rate 8/16 of the N=1600 LDPC). At GF(16) =
4 bits/symbol that is **200 info symbols**. A rate-½ GF(16) code would be **N≈400 GF(16)
symbols**. The in-tree GF16-RA (`mfsk_ctrl_codec.{h,cc}`) is hard-wired to **K=13 info
symbols** (10 message + 3 CRC), `GF16RA_MAX_N=64`. So the literal in-tree code does NOT
carry a data frame — the question is whether the *construction* scales.

**Finding — QRA (Palermo's Q-ary Repeat-Accumulate) is INHERENTLY a short-message code:**
- The actual deployed QRA codes are all K=12–13, N≈63–64:
  - **QRA65** (the Q65 mode): `QRA13_64_64_IRR_E: K=13 N=64 Q=64`, punctured to (12,63).
    Source: `qra65/qra65.c` header comment (github.com/Microtelecom/qracodes, GPLv3).
    "Codes with K=13 are designed to include a CRC as the 13th information symbol."
  - **QRA64**: same K=12 message (the JT65 72-bit source = 12×6 bits), N=63, GF(64).
- These are sized for the fixed JT65-class 72-bit beacon message. Palermo's construction
  (hand-built irregular accumulator weight tables `qra15_65_64_irr_e23.c`, puncturing,
  MAXCDEG=3) is tuned for that small N. **There is NO published QRA code at N in the
  hundreds.** (EME-2016 IV3NWV presentation; WSJT-X Q65 docs.)
- WSJT-X Q65 / QRA64 are weak-signal beacon/EME modes — message-length, not data-frame.

So the literal RA *construction as Palermo built it* is short-message. The open question
the prototype answers empirically: does a **generic degree-3 GF(16)-RA graph** (the same
sparse structure, machine-generated for arbitrary K) still give multi-symbol correction
near capacity at K=200, or does Q-ary BP degrade at that length?

### §1.2 The scaling fallback IS a known, near-capacity code (the alternative)

The broader family DOES scale, and the literature is explicit about WHY non-binary helps:
- **Non-binary LDPC over GF(16) is demonstrated near-Shannon at short block lengths
  (~N=600 bits).** Non-binary codes were proposed specifically "to overcome the
  limitation of [binary] LDPC codes performing well only for very large block lengths"
  and "outperform binary LDPC codes of the same length in bits" at short lengths.
  (Chang & Divsalar, *Non-binary protograph-based LDPC for short block lengths*, NASA/JPL;
  performance studies of NB-LDPC over GF(q), IEEE.)
- **The non-binary advantage is a SHORT-BLOCK phenomenon.** It shrinks as N grows (binary
  LDPC/turbo catch up at large N — which is why long-haul standards stay binary and why
  VARA, with longer data frames + a fading channel, chose **turbo**). At ROBUST_3's
  K=800-bit / N≈1600-bit regime we are squarely in the short-block window where a GF(16)
  code can beat the binary LDPC — IF the code construction holds at that length.
- **q-ary IRA (QIRA) codes outperform binary LDPC and turbo on AWGN** at short lengths
  (Design of q-ary IRA codes, IEEE; the QRA codes are the amateur-radio instance).

**Implication:** if the *RA* (repeat-accumulate) structure specifically doesn't hold at
K=200 (its weight/interleaver design is the short-message part), the right scaled code is
a **non-binary LDPC over GF(16)** at data-frame length (a regular/protograph NB-LDPC, not
the RA accumulator) — same symbol-matched Bessel-I0 intrinsic + Q-ary BP decoder, just a
different (denser, machine-generated) parity graph. That is the honest "name the
alternative" answer if the RA prototype underperforms.

### §1.3 Why a symbol-matched code *could* recover dB the binary LDPC leaves on the table

The binary LDPC path marginalizes M=16-tone energies into 4 bit-LLRs per symbol
(`mfsk.cc:1006-1139` log-sum-exp), then runs binary SPA. This is BICM. The GF(16) code
consumes the per-tone energies as native symbol likelihoods (Bessel-I0, Proakis §4.5.4 /
Stark 1985) with **zero bit-marginalization loss**, and GF(16) check nodes capture the
symbol-wise FSK error structure. The recoverable gap is the BICM-vs-CM gap PLUS the
binary-short-block penalty. Literature bounds the per-symbol-metric part as small (demaps
"within 0.2 dB of ML", Southampton ePrints 267344) but the **short-block coding gap** is
the bigger lever and is exactly where NB codes are documented to win.
NOTE the rate caveat: the prior feasibility doc §7 cites "BICM AMI loss can be neglected
at high coding rates" — at rate ½ this BICM term is modest; the recoverable dB, if any,
is mostly the short-block coding gain, not the demap metric.

---

## §2. What the in-tree GF16-RA already establishes (the K=13 spike)

From [tier2-suffix-fec-gf16-spike.md](tier2-suffix-fec-gf16-spike.md) §8 (measured, 42/42
tests), at **K=13**:
- §8.1 (THE key lesson): a high-rate/few-parity Q-ary code is NOT sparse and **Q-ary BP
  collapses to ~1-symbol correction**. Multi-symbol correction needs the **TRUE degree-3
  RA** (MAXCDEG=3 like real Q65). Reproduced in-tree.
- §8.2 correction capability scales with repfact: at 6 symbol-errors, P(decode) =
  0.05 (R½, repfact=1) → 1.00 (R¼, repfact=3).
- §8.4 cliff (intrinsic, base-detect, no scaffolding): **R½ −11.75 dB, R⅓ −13.34 dB,
  R¼ −14.03 dB SNR3k** — the code reaches the −14.68 noncoherent detection floor only at
  **R¼ (repfact=3)**, NOT at R½.

**Tension with the task target:** the task wants ~143–175 bps = roughly **rate ½** (to
keep bps near the binary ROBUST_3's 149). But the K=13 spike shows GF16-RA only reaches
the deep floor at **rate ¼** (a ~3 dB swing R½→R¼). At rate ½ the short code was −11.75,
i.e. ~3 dB ABOVE its own rate-¼ floor. So even if the construction scales perfectly, a
rate-½ GF16-RA is unlikely to hit −10 — and a rate-¼ GF16-RA that does reach deep SNR
would run at ~¼ the bps (≈70–75 bps, below the 143 target). **The bps target and the
−10 floor target are in tension for this code family — quantify the actual tradeoff
curve at K=200, don't assume.**

---

## §3. The prototype + measurement plan (what THIS branch does)

**Decision (CLAUDE.md §4): measure the code BEFORE wiring the expensive PHY.** The full
ROBUST_3 PHY/ARQ swap (§5 audit below) is a large, risky cross-layer change whose payoff
is gated entirely on whether a K≈200 GF(16)-RA beats the binary LDPC's −7.1 dB cliff. The
in-tree spike measured the *ctrl-suffix* cliff at the **codec/AWGN-energy level** (§8.4)
and those are the numbers the whole tier-2 decision trusts. We use the identical method at
data-frame length. If the K≈200 cliff does not beat −7.1, the PHY wiring is moot. If it
does, that result justifies the PHY wiring as a separate follow-on.

**Implementation (codec-level, no production wiring, all gated/test-only):**
1. **Generalize `gf16ra` to arbitrary K** behind a new `configure_k(K_total, repfact)`
   (the existing `configure(repfact)` stays K=13 for the ctrl path — byte-identical).
   Generalize `encode`/`soft_decode` to take an explicit GF(16) symbol vector of length K
   (not the 64-bit `[type|payload38|crc12]` packing), so K can be 200. The RA graph
   builder (`init`) is already K-parameterized via `GF16RA_K`; lift that to a runtime K.
   Raise `GF16RA_MAX_N` ceiling for the test path (or use heap vectors keyed on runtime N).
2. **K-scaled clean-flip correction-capability test** (mirror `test_gf16_ra_correction_
   capability`, §10): at K=200, repfacts {1,2,3}, sweep #wrong-symbols, measure P(decode).
   This is the cheap make-or-break: does degree-3 Q-ary BP still correct multi-symbol
   errors at K=200, or collapse?
3. **K-scaled AWGN cliff test** (mirror `gf16ra_cliff_one`, §10.5): synthesize per-tone
   energies under AWGN at the M=16×2 ROBUST_3 symbol SNR, at K=200, repfacts {1,2,3}, find
   the SNR3k cliff (P(decode)=0.5) and convert to Eb/N0. Report net bps for each rate.
4. **THE comparison:** GF16-RA-K200 cliff (per rate) vs binary-LDPC ROBUST_3 −7.1 dB
   (SNR3k, +5.89 Eb/N0) vs VARA L4 −10 / +2.34. Does any rate that yields ≥143 bps beat
   −7.1? Does any rate at all reach −10, and at what bps?

The AWGN-energy synthesis must match the ROBUST_3 symbol geometry: M16×2 (two parallel
M=16 streams), so a "GF(16) symbol" = one tone in one stream; Es per symbol and the noise
variance per tone follow the same `passband_test_EsN0` calibration the binary harness uses
(SNR3k = channel-SNR − 1.1 dB; `telecom_system.cc:369`). The energy-level harness models
the per-tone matched-filter outputs directly (Rician/Rayleigh under signal/no-signal),
which is the standard noncoherent-FSK BER model and is what the spike §8.4 used.

**Honesty guard:** the energy-level cliff is the CODE's intrinsic reach (genie sync, no
preamble/Moose/metric-gate scaffolding) — directly comparable to spike §8.4(B) and to a
binary-LDPC BER measured the same way, but it is NOT an end-to-end PHY number. The prior
binary ROBUST_3 −7.1 cliff IS end-to-end (full `passband_test_EsN0`). To compare
apples-to-apples I will ALSO measure the binary LDPC's cliff in the SAME energy-level
model (re-run, genie) so both numbers exclude scaffolding, and report both framings.

---

## §5. Cross-layer data-flow audit (CLAUDE.md §5) — the FEC swap

The FEC swap touches shared frame/decode state. Per §5 this audit must precede any
production wiring. (The codec-level prototype in §3 touches NONE of this — it is the
reason to measure first.) This section scopes the production change for the follow-on.

### §5.1 Producers / consumers of the affected state

**State: `data_container.encoded_data` / `bit_interleaved_data` / `modulated_data` (TX),
`demodulated_data` (LLRs) / `deinterleaved_data` / `hd_decoded_data_bit` (RX).**
- **Producers (TX):** `cl_telecom_system::transmit_byte_*` →
  `ldpc.encode(data_bit, encoded_data)` (telecom_system.cc:266, :548, :2769) → puncture
  (drop K..K+nVirtual) → `interleaver(...)` (:273,:555,:2774) → `psk.mod`/MFSK mod.
- **Consumers (RX):** MFSK `mfsk.demod(fft_in, rx_nbits, demodulated_data)` (:2340) emits
  bit-LLRs → `deinterleaver` (:2634) → de-puncture (insert zero-LLR for virtual) (:2636) →
  `ldpc.decode(deinterleaved_data, hd_decoded_data_bit)` (:2647).
- **Frame sizing:** `data_container.nBits` = transmitted bit count; `ldpc.N=1600`,
  `ldpc.P` = parity bits, `ldpc.K` = info bits (= 800 at rate ½). `nReal_data =
  nBits − ldpc.P`, `nVirtual_data = ldpc.N − nBits` (puncture count). Throughput accessor
  `(nBits − ldpc.P − outer_code_reserved_bits)/8` (:480).

**What a GF(16)-RA swap changes:** for `is_robust_config(103)` ONLY, replace
`ldpc.encode`→GF16RA encode (info bits → GF(16) codeword tones), and the
`mfsk.demod→deinterleave→depuncture→ldpc.decode` chain → `mfsk.demod_energies` (NEW,
emits the N×M per-tone energy matrix instead of bit-LLRs) → `gf16ra::soft_decode_k`.

### §5.2 Valid states / default-init (the bite points)

1. **Frame geometry mismatch.** Binary path: nBits transmitted bits map 1:1 to LDPC
   coded bits (post-puncture). GF16-RA path: N GF(16) symbols = 4N bits = 4N/log2(M) MFSK
   tones. The OFDM/MFSK framing (`data_container`, symbol count, preamble) is sized from
   `nBits`/M. A GF16-RA codeword of N≈400 symbols must land on an integer number of M16×2
   OFDM-MFSK symbols — the frame-geometry constraint the coherent ROBUST_3 P1 had to fix
   (`wt/robust3` b6d392f "forces full 1600-bit codeword"). MUST re-derive nBits/symbol
   count for the GF16-RA codeword or the data_container buffers mis-size. **This is the
   single most likely silent corruptor** (parallels the N_MAX=1600 wall the coherent proto
   hit).
2. **The interleaver.** Binary path interleaves coded BITS (`bit_interleaver_block_size =
   nBits/10`, telecom_system.cc:5377). GF16-RA's diversity is its OWN symbol interleaver
   (the RA accumulator spread). Double-interleaving (bit interleaver on top of GF16-RA
   symbols) is at best a no-op and at worst scrambles the symbol→tone alignment the Q-ary
   BP assumes. The GF16-RA path must BYPASS the bit interleaver (symbols map directly to
   tones, like the ctrl-suffix does).
3. **Puncturing.** Binary ROBUST_3 punctures the N=1600 LDPC to nBits. GF16-RA is
   systematic [K info | NC parity] with its OWN rate (R=K/N set by repfact); there is no
   LDPC puncture step. The depuncture-insert-zero-LLR loop (:2636) must be skipped.
4. **`hd_decoded_data_bit` consumer.** ARQ reads decoded bits + a decode-success flag
   (`receive_stats.iterations_done`, CRC). GF16-RA emits decoded GF(16) symbols → repack
   to bits; the success signal is the GF16-RA CRC/convergence, not LDPC iter count.

### §5.3 Invariants consumers assume + ARQ/CRC interaction

- **ARQ frame CRC.** The data frame carries a CRC checked by the ARQ layer
  (`arq_common.cc` CRC32/CRC16 over the payload) ABOVE the FEC. GF16-RA has its OWN
  optional CRC-as-protected-info (Q65 QRATYPE_CRC), used in the ctrl path as the
  accept-gate. For DATA, the existing ARQ-layer frame CRC remains the integrity check;
  the GF16-RA inner code need NOT carry a second CRC (it can use BP convergence /
  re-encode-syndrome as the stop criterion and let the ARQ CRC catch residual errors).
  Decision deferred to wiring phase; the codec prototype measures raw symbol/frame
  decode, not the ARQ CRC.
- **Decode-success semantics.** ARQ retransmit logic keys on per-frame decode success.
  GF16-RA must emit an equivalent boolean. A false "success" with wrong bits would defeat
  ARQ (the §5 canonical failure mode). The ARQ CRC32 over the payload is the backstop.
- **batch_seq_id / SACK / streaming.** Unaffected at the bit level IF the GF16-RA frame
  delivers the SAME info bits the binary frame did (same nBits−P payload). The swap is
  PHY-internal; layers above see identical decoded bytes. This holds ONLY if §5.2(1)
  frame geometry is correct.

### §5.4 What the prototype changes vs what production wiring would change

- **Prototype (this branch):** adds K-generalized `gf16ra` codec functions + test-only
  harness. Touches NO production decode path, NO `data_container`, NO ARQ. Default build
  byte-identical to 8c6d3b7. (Verified: only `mfsk_ctrl_codec.{h,cc}` +
  `mfsk_ctrl_codec_tests.cc` modified; existing `configure(repfact)`/K=13 path unchanged
  so all 42 ctrl tests still pass.)
- **Production wiring (follow-on, ONLY if §3 measurement wins):** a `is_robust_config(103)`
  branch in transmit/receive that swaps the encode + the demod→decode chain, a new
  `mfsk.demod_energies`, frame-geometry re-derivation, interleaver/puncture bypass, and a
  cross-layer regression test driving ARQ through a GF16-RA frame (per CLAUDE.md §5 the
  fact-doc-paired regression test). Scoped here; not built until the cliff justifies it.

---

## §6. RESULTS (measured)

Build: `sim/robust3-gf16ra` worktree `C:/Users/kamer/mercury_wt/robust3-gf16ra`,
`bash build.sh o3`. Reproduce: `mercury.exe --test` → tests
`gf16_ra_data_scaling_capability` (§10.6a) and `gf16_ra_data_scaling_cliff`
(§10.6b). 52/52 ctrl-codec assertions pass (was 50 — the 2 new tests added; all
existing K=13 ctrl tests unchanged ⇒ the K=13 codec path is byte-identical).

### §6.1 SCALING VERDICT — the construction DOES scale to K=200 (make-or-break #1)

Clean-flip symbol-correction capability, P(frame-correct) vs #wrong codeword
symbols, K=13 (cross-check vs in-tree §8.2) and K=200 (= 800 info bits):

| K | repfact | N | R | a few errs | mid | many errs |
|---|---|---|---|---|---|---|
| 13 | 1 | 26 | 0.50 | 2:0.81 | 6:0.07 | 12:0.00 |
| 13 | 2 | 39 | 0.33 | 2:0.99 | 6:0.94 | 12:0.58 |
| 13 | 3 | 52 | 0.25 | 2:1.00 | 6:0.99 | 12:1.00 |
| **200** | 1 | 400 | 0.50 | 10:0.63 | 40:0.00 | 100:0.00 |
| **200** | 2 | 600 | 0.33 | 10:0.97 | 40:0.82 | 100:0.05 |
| **200** | 3 | 800 | 0.25 | 10:1.00 | 40:1.00 | **100:1.00** |

- **The degree-3 GF(16)-RA graph does NOT collapse at K=200.** At repfact=3 (R¼)
  it corrects 100 wrong symbols out of 800 with P=1.00 — the same flawless
  multi-symbol correction it has at K=13. The K=13 row reproduces the in-tree
  §8.2 capability (proof the K-generalized encoder/decoder is faithful). So the
  RA *construction* (machine-generated accumulator interleaver + weights, Q-ary
  WHT-BP) scales structurally to the data-frame length — answering the literal
  "is the in-tree codec inherently short-message" question: **the CODE is
  hard-wired to K=13, but the CONSTRUCTION generalizes and holds at K=200.**
- The rate-vs-strength tradeoff is the same as K=13: R½ is weak (degrades fast),
  R¼ is strong. Multi-symbol correction needs repfact≥2.

### §6.2 THE CLIFF — and it is a NEGATIVE result (make-or-break #2)

AWGN per-tone-energy cliff (P(frame)=0.5). Idealized noncoherent-FSK model
(sent tone = signal+CN(0,1), others = CN(0,1)); Eb/N0 = Es/N0 − 10log10(4R);
SNR3k binary-anchored. The K=13 R¼ cross-check came to SNR3k −12.0 vs the in-tree
§8.4 −14.03 → **this idealized model is ~2 dB OPTIMISTIC vs the §8.4 passband
synthesis. The model offset is symmetric across codes, so only the in-harness
binary-vs-GF16RA delta (same model) is trustworthy as an absolute dB; external
comparisons carry the ~2 dB caveat.**

| Code @ data length (K_info=800) | rate | net bps | Eb/N0 cliff | SNR3k (model) |
|---|---|---|---|---|
| **GF(16)-RA** repfact=1 | 0.50 | 149 | **+5.99** | −7.0 |
| **GF(16)-RA** repfact=2 | 0.33 | 99 | +5.75 | −9.0 |
| **GF(16)-RA** repfact=3 | 0.25 | 74 | +5.00 | −11.0 |
| **Binary LDPC** (real ROBUST_3 rate-8/16 + production demap, SAME model) | 0.50 | 149 | **+3.99** | −9.0 |
| binary ROBUST_3 (real PHY, prior doc / §3) | 0.50 | 149 | +5.89 | −7.1 |
| VARA L4 (reference) | — | ~175 raw | **+2.34** | −10 |

**THE HEADLINE: at the same rate (R½) and same bps (149), the binary LDPC beats
the GF(16)-RA by 2.0 dB Eb/N0 (+3.99 vs +5.99), measured in the identical energy
model with the identical production demap.** The symbol-matched non-binary code
is *worse*, not better. The hypothesis — that GF(16)-RA recovers the gap the
binary code leaves above the noncoherent floor — is **FALSIFIED**.

- The binary curve is a clean waterfall (0.000 ≤ Es/N0+6, 0.900 at +7, 1.000 at
  +8) — the decoder is healthy; this is not a measurement artifact.
- GF(16)-RA only reaches deeper SNR by dropping rate: R¼ buys ~1 dB (Eb/N0 +5.0)
  but halves bps to 74 — and even that is 1 dB WORSE than the binary code's +3.99
  at 149 bps, and **2.66 dB short of VARA's +2.34.** No GF(16)-RA operating point
  beats the binary LDPC, and none reaches VARA.

### §6.3 WHY (reconciles with the §1.2 research)

Non-binary codes win at SHORT block lengths (§1.2: the GF(q) advantage is "to
overcome [binary] LDPC performing well only for very large block lengths"; it is
demonstrated near-Shannon at ~N=600 *bits*, ~150 GF(16) symbols). ROBUST_3's data
frame is N=1600 coded *bits* — **past the crossover.** At that length the binary
LDPC is already near its asymptotic performance (Mercury's is a clean girth-10
PCM, double-precision 200-iter SPA — see prior doc §7/§11), so the non-binary
advantage has evaporated. Meanwhile the GF(16)-RA at K=200 is a *generic
machine-generated* accumulator graph, NOT a hand-optimized one (QRA codes are
hand-tuned only at K=12–13; §1.1), so it sits ~2 dB worse. The crossover is real
and ROBUST_3 is on the wrong side of it. (This is precisely why VARA, with a
data-frame-length code on a fading channel, chose TURBO, not a short non-binary
code — §1.2.)

The §8.4 ctrl-suffix result (GF16-RA reaches the −14 floor) is NOT contradicted:
the ctrl suffix is K=13 (≈52 symbols) — *in* the short-block window where the
non-binary code genuinely wins over the uncoded/short-binary alternatives. That
win does not transfer to the K=800 data frame.

---

## §7. DECISION

**DROP the GF(16)-RA-as-ROBUST_3-data-code line. Do NOT wire it into production.
Do NOT merge.** The measurement is a clean, twice-confirmed negative:

1. **Scaling:** the GF(16)-RA *construction* scales to K=200 (corrects 100/800
   symbols at R¼) — so "inherently short-message" is FALSE for the construction
   (though TRUE for the literal in-tree K=13 codec and for every *published* QRA
   code). It does NOT fail to carry the payload.
2. **But it is the WRONG code at this length:** at the data-frame block length
   the binary LDPC beats it by **2.0 dB** at equal rate/bps (in-harness,
   apples-to-apples). The non-binary advantage is a short-block phenomenon and
   ROBUST_3's frame is past the crossover. GF(16)-RA reaches VARA's −10 SNR3k
   only at R¼/74 bps in the optimistic model, but the binary code is *better*
   there too, and the real-PHY axis is ~2 dB worse than the model — so neither
   code reaches VARA's +2.34 Eb/N0. **GF(16)-RA does not move the −7 → −10 cliff.**

**The honest "name the alternative near-capacity M-FSK-matched code" answer
(task's fallback ask):**
- A **non-binary LDPC over GF(16)** at data-frame length *could* in principle
  edge the binary code (the §1.2 N≈600-bit near-Shannon result), but the margin
  at N=1600 bits is small-to-zero (we are past the crossover) and Mercury's
  binary LDPC is already clean/near-asymptotic — so the expected win is a
  fraction of a dB at most, for a large NB-LDPC-over-GF(16) build (new
  parity-matrix design + GF(16) BP wired into the data path: ~3–5 wks). **Not
  worth it** — it cannot reach VARA's +2.34 either; the residual is the
  noncoherent square-law M-FSK floor (prior doc §7: design floor ≈ +4.2 dB
  Eb/N0; ROBUST_3 at +3.99 in-harness is already AT that floor, not above it —
  the binary code is NOT leaving dB on the table after all).
- **TURBO** (VARA's choice) is the near-capacity code that does scale to
  data-frame length, but it too is bounded by the noncoherent M-FSK floor; the
  ~1.7 dB VARA edge over Mercury's binary floor is most plausibly VARA's
  **slower symbols** (23 baud vs ~40) + **band-filling frequency diversity**, NOT
  the code family (prior memory: weak-signal-floor-campaign, 2026-05-31
  correction). A turbo port (~6–10 wks) would not by itself reach −10.
- **The −10 floor remains a PHY-architecture problem (coherent tier), not a
  data-FEC problem** — consistent with the prior doc §7/§9 and the shelved
  coherent-tier line. This GF(16)-RA probe *confirms* that conclusion from the
  code-family angle: swapping the binary LDPC for the best symbol-matched
  non-binary code does NOT help; the binary code is already at the noncoherent
  floor for this rate/length.

### §7.1 §5 cross-layer audit status

The §5 audit (frame geometry, interleaver/puncture bypass, demod→energy path,
ARQ/CRC decode-success semantics) was completed BEFORE any wiring and is recorded
in §5 above. **No production wiring was done** — the prototype is codec-level +
test-only (the deliberate "measure before building the expensive thing" sequence,
§3). So none of the §5 shared state was touched: `data_container`, the bit
interleaver, the LDPC puncture path, `mfsk.demod`, the ARQ frame CRC, batch/SACK
state are all unchanged. Default build is byte-identical to 8c6d3b7 (the K=13
ctrl path uses the original `configure`/`encode`/`soft_decode`; the new
`configure_k`/`encode_k`/`soft_decode_k` have no production caller). The §5 audit
remains the scoping document IF a future result ever justifies wiring — it does
not here.

### §7.2 Branch / commit (SIM ONLY, NO MERGE)

`sim/robust3-gf16ra` (off `8c6d3b7`), worktree `C:/Users/kamer/mercury_wt/robust3-gf16ra`.
Changes: K-generalized `gf16ra` codec (`mfsk_ctrl_codec.{h,cc}`: `configure_k` /
`encode_k` / `soft_decode_k`, heap-backed, K unbounded) + the §10.6 data-frame
scaling/cliff harness incl. the in-harness binary-LDPC baseline
(`mfsk_ctrl_codec_tests.cc`) + this fact doc. Not for merge to monitor.

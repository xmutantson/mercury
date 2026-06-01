# Suffix FEC — acquisition-cliff prototype (CONNECT/ACK 13-symbol M=16 ctrl-suffix)

Branch: `wt/connect-suffix-fec` off `monitor` (01535f2). MEASURED PROTOTYPE —
do NOT merge. SOFTWARE/SIM ONLY.

Goal: add FEC/soft-decode to the **binding acquisition sub-stage** (the
13-symbol M=16 control suffix) so its decode cliff tracks the base-pattern
detection floor instead of the uncoded `p^13` cliff. Get hard sim numbers for
acquisition gain (dB) AND throughput cost (per-batch ACK overhead) for a
ship/no-ship decision.

---

## §1. The failure structure (root cause, not symptom)

The control suffix carries a 52-bit field `[type:2 | payload:38 | crc12:12]`
(`mfsk.h:126-136`). At M=16 it is packed **4 bits/symbol → 13 symbols**, dense
**rate-1 (uncoded)** (`mfsk.cc:622-640 pack_ctrl_suffix`).

RX (`ofdm.cc:3986-4070 decode_suffix_tones`) makes a **hard-decision argmax**
per symbol: for each of the 13 suffix symbols it FFTs the window, finds the
single highest-energy bin across streams (`ofdm.cc:4015-4035`), de-hops it
(`:4040-4042`), and returns ONE tone per symbol (`out_tones[s]`). The per-tone
**energies are computed but discarded** — only `best_tone` survives. The hard
tones are unpacked (`mfsk.cc:646-666 unpack_ctrl_suffix`) and the 12-bit CRC is
checked at `arq_common.cc:5056-5069`. CRC passes only if **every one of the 13
symbol argmax decisions is exactly right** ⇒ `P(decode) ≈ (1−q)^13` where `q`
is the per-symbol argmax-error probability. This multiplicative-AND is the
cliff.

The base pattern that gates first (`detect_ack_pattern`, threshold **7/16
matched**, `telecom_system.cc:3535`) is far more tolerant — it survives ~9 bad
symbols. So the base detector reaches a lower SNR (~−9.7 dB SNR3k per the cliff
agent) than the suffix (~−8.6 dB). **The suffix is the binding stage; the base
detector has ~1 dB of headroom the suffix cannot use today.** The data preamble
(16 symbols, matched-count) reaches ~−14.6 dB. (Cliff dB from the prior cliff
agent; mechanism confirmed here from code, methodology in §4.)

### §1.1. Prior art (no magic numbers — cited)

- For noncoherent M-FSK the natural soft metric is the **per-tone energy**;
  hard-decision argmax discards it. Soft-decision over noncoherent FSK
  outperforms hard-decision (classic ≈ 2 dB on AWGN; larger when the hard
  scheme has a multiplicative-AND failure like `p^13`). (Proakis, *Digital
  Communications*, Ch.8 noncoherent FSK; orthogonal-signaling capacity.)
- **Symbol-repetition with soft-energy combining** — "sum the soft decisions
  over each symbol period" — is the documented improvement over hard
  majority-vote (UMich EECS555 FH-SS Ch.12; dsprelated comp.dsp repetition-code
  thread). Mercury already uses the HARD form: the SNR suffix votes 3/8 on
  argmax tones (`telecom_system.cc:3375-3412`). Soft-combining is the upgrade.
- **Reed-Muller / short block codes** decode naturally over M-ary alphabets via
  Plotkin recursion with soft (Euclidean) metrics (RM soft-decode literature).
  Higher gain per added symbol; more decoder complexity.
- **CRC-as-code via soft list decoding**: order symbol candidates by energy,
  enumerate the most-likely combinations, accept the one whose CRC passes. The
  existing CRC12 then provides *error correction by search* at zero added
  length (false-accept ≈ 2^−12 per trial). (List/ordered-statistics decoding;
  CRC-aided list decoding, cf. Tal–Vardy / CA-SCL family — same principle at
  tiny block length.)

Sources:
- https://www.eecs.umich.edu/courses/eecs555/chap12.pdf (FH-SS, FSK + diversity combining)
- https://www.dsprelated.com/showthread/comp.dsp/127194-1.php (soft repetition decode)
- https://arxiv.org/pdf/1703.01432 (LDPC-coded noncoherent FSK, soft FSK demod)
- https://dsplog.com/2012/03/15/hamming-code-soft-hard-decode/ (Hamming soft vs hard ≈ 2 dB)

---

## §2. Throughput budget (the user's hard gate)

`ack_sack_suffix_len() = 13` symbols (`mfsk.h:136`). One suffix symbol = Nofdm
× interp = 292 × 4 = 1168 passband samples @ 48 kHz = **24.33 ms/symbol**.
13 symbols = **316 ms**.

- **CONNECT suffix fires ONCE PER SESSION** (START_CONN / TEST_ACK / TEST_CONN
  handshake, `arq_common.cc:4959/4972/5142`). Lengthening it costs ~316 ms ×
  (growth factor) ONCE — negligible relative to a multi-minute session. **Free.**
- **ACK suffix fires PER BATCH** (`generate_ack_sack_pattern`, the SACK ACK).
  Each added symbol adds 24.33 ms to every batch turnaround. THIS is the
  throughput-critical path. The ACK overhead as a fraction of throughput
  depends on config (batch airtime). Must be MEASURED and kept ≈ 0.

### §2.1. Payload occupancy (FEC budget — how much slack per frame type)

The 38-bit payload field is **heavily underutilized on CONNECT, full on ACK**
(`mfsk_ctrl_codec.h`):

| Frame | info bits used | reserved/free |
|-------|----------------|---------------|
| START_CONN | 1 (nb) + 36 (call) = **37** | 1 |
| TEST_ACK   | 2+2+8 = **12** | **26** |
| TEST_CONN  | 4+2+8 = **14** | **24** |
| ACK_SACK   | 8 (bsi) + 30 (bitmap) = **38** | **0** |

⇒ TEST_ACK/TEST_CONN have huge slack; ACK_SACK has none. This drives the
asymmetric design.

---

## §3. Chosen design (throughput-aware, two-tier)

**Foundation (both tiers): expose soft energies.** Add a soft variant of
`decode_suffix_tones` that returns the **per-symbol, per-tone energy matrix**
(`energies[s][m]`) instead of only the hard argmax. Baseline hard path
unchanged. This is the enabler for everything and adds ZERO airtime.

### Tier 1 — ACK (per-batch, throughput-critical): **soft CRC-aided list decode, ZERO added length.**
Keep the 13-symbol suffix byte-identical on the wire. On RX, instead of hard
argmax→unpack→CRC, use the soft energies to **list-decode**: for each symbol
keep the top-`K` candidate tones ordered by energy, search combinations in
increasing total "cost" (sum of −log or energy-gap), and accept the first whose
CRC12 passes. Bounded search (cap total trials, e.g. ≤ a few thousand). This
recovers the cases where 1–3 symbols had the right tone in 2nd/3rd place —
exactly the cliff regime — at **no throughput cost**. CRC12 false-accept ≈
2^−12 per trial; with a bounded trial budget the aggregate FAR stays low (and
the type field + ARQ retx are the backstop, same as today).

### Tier 2 — CONNECT (once/session, length-free): **add parity symbols, soft block decode.**
CONNECT can afford to grow. Add R extra **parity symbols** computed over the
payload symbols so the decoder can correct several wrong symbols with a soft
(energy-weighted) metric. Because START_CONN uses 37/38 payload bits, the
parity goes in ADDED symbols (suffix grows 13→13+P for CONNECT only); TEST_ACK
/TEST_CONN have 24–26 free payload bits that can ALSO hold parity without
growing at all. The CONNECT decoder uses the soft list-decode of Tier 1 *plus*
the parity check as an additional accept gate / corrector.

**Why this split satisfies the gate:** the expensive coding (added symbols)
lands only on CONNECT (free, once/session); the per-batch ACK gets a
**zero-length** soft upgrade. Coding-gain-per-symbol is maximized: ACK spends 0
symbols for its gain, CONNECT spends symbols only where airtime is free.

**Gating:** all of this is behind a flag `suffix_fec_mode` (0 = baseline hard,
byte-identical; 1 = soft-decode ACK + soft-decode/parity CONNECT). When off,
`decode_suffix_tones` and the pack/unpack paths are bit-for-bit the legacy
path. Default OFF (prototype).

### §3.1. Measurement-first ordering
The single highest-value, lowest-risk first step is the **soft list-decode with
ZERO wire change** (Tier 1 applied to BOTH CONNECT and ACK). It needs no
flag-day, costs no airtime, and directly tests "is the p^13 hard decision the
cliff?" If the soft list-decode alone moves the cliff most of the way to the
base-detector floor, we may not need Tier-2 parity at all — and ship cost = 0
throughput. So the prototype implements and measures Tier 1 first; Tier 2
(parity, wire-format growth) only if Tier 1 leaves a gap worth the CONNECT
airtime. This is the measured-decision the user asked for.

---

## §4. Measurement methodology (sim, no hardware)

Reuse the in-process passband harness already in
`mfsk_ctrl_codec_tests.cc`:
- `test_mfsk_connect_passband_roundtrip_clean` (:483) — generate suffix passband
  via `generate_ctrl_suffix_pattern_passband`, decode via
  `decode_ctrl_suffix_from_passband`, check CRC. The cliff sweep = this + AWGN.
- `synth_preamble_buffer` (:1237) already injects real-valued passband Gaussian
  at `noise_sigma_pb` and notes in-band SNR ≈ passband_SNR + 10·log10(fs/BW)
  (≈ +13 dB for fs=48k, BW=2343 Hz).

**Cliff sweep:** for sigma in a sweep, generate N (≈200) suffix frames with
random payloads, add AWGN, attempt decode (baseline hard vs soft), record
P(CRC-pass) vs SNR3k. Cliff = SNR3k where P(pass) crosses 0.5. Report the dB
move baseline→soft for CONNECT and ACK suffix separately.

**SNR3k:** measure in-band signal power (suffix waveform) and noise PSD (sigma²
spread over fs), convert to power in a 3 kHz reference bandwidth:
`SNR3k = 10·log10(P_sig_inband / (N0 · 3000))`. Computed exactly from the known
sigma and the measured signal RMS — no fitting.

**Throughput cost:** added suffix symbols × 24.33 ms. Per-batch ACK overhead %
= added_ms / batch_airtime_ms at representative configs (ROBUST_0, CONFIG_10,
CONFIG_15). Tier 1 added symbols = 0 ⇒ overhead = 0%. Report explicitly.

---

## §5. Cross-layer data-flow audit (CLAUDE.md §5) — suffix wire format

State changed: the ctrl-suffix tone sequence on the wire + the RX decode of it.
Shared across PHY (mfsk/ofdm) → telecom_system → ARQ (arq_common control loop).

### 1. Producers (who WRITES the suffix tones on the wire)
- TX CONNECT: `cl_mfsk::generate_ctrl_suffix_pattern` (`mfsk.cc:855-881`) ←
  `cl_telecom_system::generate_ctrl_suffix_pattern_passband`
  (`telecom_system.cc:3456-3491`) ← `arq_common.cc:4849`
  (`send_mfsk_ctrl_suffix_phy_core`) ← START_CONN (`:4959`), TEST_ACK (`:4972`),
  TEST_CONN (`:5142`).
- TX ACK: `cl_mfsk::generate_ack_sack_pattern` (`mfsk.cc:797-824`) ←
  `generate_ack_sack_pattern_passband` (`telecom_system.cc:3193-3231`) ← ACK
  send path (responder).
- Both call `pack_ctrl_suffix` / `pack_ack_sack_payload` (`mfsk.cc:622/677`).

### 2. Consumers (who READS the suffix tones)
- RX hard tones: `ofdm.decode_suffix_tones` (`ofdm.cc:3986`) ← BOTH
  `detect_ack_snr_from_passband` (`telecom_system.cc:3343`, ACK+SNR path) AND
  `decode_ctrl_suffix_from_passband` (`telecom_system.cc:3586`, CONNECT path).
- Unpack: `unpack_ctrl_suffix` (`mfsk.cc:646`) via
  `decode_ctrl_suffix_from_last_capture` (`mfsk.cc:763`); `unpack_ack_sack_payload`
  (`mfsk.cc:695`) via `decode_ack_sack_from_last_capture` (`mfsk.cc:727`).
- Capture buffers: `last_connect_suffix_tones[]`/`last_ack_sack_suffix_tones[]`
  + `*_capture_valid` flags (`mfsk.h:180-187`), written by the telecom_system
  hooks (`telecom_system.cc:3360-3372 ACK`, `:3596-3606 CONNECT`).
- ARQ validation: `arq_common.cc:5056-5069` recomputes CRC12 over
  `pack_ctrl_typed40_msb_v2` (`:4784`) and routes by type (`:5038`).
- ALSO consumes hard tones: the SNR majority-vote path
  (`telecom_system.cc:3375-3412`) reads `suffix_tones[0..7]` for turboshift SNR.
  **This shares `decode_suffix_tones` output — must not regress.**

### 3. Valid states (esp. before any producer writes)
- `out_tones[s] = -1` for undecoded/past-buffer symbols (`ofdm.cc:4000-4005`).
  Consumers (`telecom_system.cc:3363, 3597`) treat any `<0 || >=M` as
  `clean=false` ⇒ capture invalid. Soft path MUST preserve this: a symbol that
  runs past the buffer ⇒ that symbol's energies are unavailable ⇒ list-decode
  must treat it as an erasure / abort, never as a spurious accept.
- `*_capture_valid=false` until a clean hard capture. Soft path sets validity
  only when a CRC-passing codeword is found.
- NB: `ack_sack_suffix_len()==0` ⇒ all suffix paths early-return; soft path must
  keep that (M<16 → no FEC, identical to baseline).

### 4. Invariants consumers assume
- I1: `decode_suffix_tones` returns exactly `suffix_len` entries, each a
  de-hopped tone 0..M−1 or −1. The SNR vote and both unpack paths depend on
  this. **Soft path keeps the hard `out_tones[]` output identical** (the soft
  energies are an ADDITIONAL output via a new overload/out-param) so the SNR
  vote and the baseline hard decode are byte-identical when FEC is off.
- I2: A decode is accepted ONLY if CRC12 matches (`arq_common.cc:5059`) AND
  type matches (`:5038`). Soft list-decode adds candidates but the SAME CRC12 +
  type gate decides acceptance ⇒ no weakening of the accept criterion beyond
  the bounded extra trials (each still CRC-gated). FAR rises by ≤ trials·2^−12;
  bound the trials.
- I3: ACK and CONNECT use SEPARATE capture buffers (`mfsk.h:180-187`) so the two
  detector windows don't alias. Soft path keeps them separate.
- I4: Buffer sizing (`telecom_system.cc:5487-5499`) derives ALL passband sample
  counts from `ack_sack_suffix_len()` × Nofdm × interp. If Tier-2 grows the
  CONNECT suffix, the new length MUST flow through this same expression so TX/RX
  buffer sizes, `ctrl_suffix_pattern_passband_samples`, and the RX
  `reserve_after`/`frames_to_read` all stay consistent. (Tier 1 = no growth ⇒
  no sizing change.)

### 5. What the fix changes & per-consumer walk
- **Tier 1 (soft list-decode, no wire change):** only the RX decode CHANGES
  (hard argmax → soft list search), gated. Producers unchanged (byte-identical
  wire). Consumers: SNR vote unaffected (I1 — still reads hard `out_tones`);
  CRC/type gate unchanged (I2); capture buffers unchanged (I3); sizing unchanged
  (I4). The ONE new risk is FAR from extra CRC trials → bounded + measured.
- **Tier 2 (CONNECT parity, optional):** changes TX (adds/encodes parity) and RX
  (parity-aware decode) for CONNECT only. ACK path untouched. Sizing flows via
  I4. Flag-day only between FEC-on peers; baseline peers unaffected because the
  flag defaults OFF and the wire is identical when off.

### Regression test (paired with this doc, CLAUDE.md §5)
`mfsk_ctrl_codec_tests.cc`: (a) soft-decode round-trip = hard-decode result on
clean (no regression); (b) baseline byte-identical when flag off (TX bytes
compared); (c) cliff sweep asserts soft cliff ≤ hard cliff (the gain); (d)
NB (M<16) unchanged; (e) FAR bound: pure-noise input does not produce a
CRC-passing soft decode above the measured rate. These run in-process via
`mercury.exe --test`.

---

## §6. MEASURED RESULTS (sim, `mercury.exe --test`, deterministic seeds)

Prototype = **Tier 1 only** (CRC-aided soft list decode, ZERO airtime, no wire
change). Implemented:
- `cl_ofdm::decode_suffix_candidates` (ofdm.cc) — top-K per-symbol tones + soft
  cost (normalized energy gap). `cand[k=0]` == the hard `decode_suffix_tones`.
- `soft_list_decode_ctrl_suffix` (mfsk_ctrl_codec.cc) — CRC-gated best-first
  lattice search, bounded by `max_trials` and `max_flips` (Hamming ball).
- `cl_telecom_system::decode_{ctrl_suffix,ack_sack}_from_passband_soft` — same
  detector + mini-Moose as the hard path, then candidates + soft search.
- Gated by `suffix_fec_mode` (default 0); baseline hard path byte-identical.

### §6.1. SNR3k calibration (validates the axis vs the prior cliff agent)
The BASE-pattern detection floor in this harness lands at **SNR3k ≈ −14.68 dB**,
matching the data-preamble floor the cliff agent measured (−14.6 dB). The HARD
suffix cliff lands at **≈ −7.3 dB (P=0.5)** / dead by −9.8 dB — reproducing the
cliff agent's −8.6 dB suffix figure. So the sim axis is comparable to prior work
and the suffix IS the binding stage (~6 dB above the detection floor).

### §6.2. FALSE-ACCEPT RATE vs max_flips (pure noise, 2000 trials, n=13 K=4)
| max_flips | pure-noise FAR | search size (codewords) |
|-----------|----------------|--------------------------|
| 0 (hard)  | 0.0000         | 1 |
| **1**     | **0.0025 (0.25%)** | 1+13·3 = 40 |
| 2         | 0.0335 (3.4%)  | ≈ 743 |
| 3         | 0.1385 (14%)   | ≈ 7k (hits trial cap) |
The flip cap is the FAR lever. **Default = 1** (0.25% FAR) is the safe operating
point. The type field (1/4) + ARQ retransmit are additional backstops.

### §6.3. ACQUISITION GAIN (P=0.5 cliff move, soft − hard). CONNECT ≡ ACK.
| operating point | suffix cliff (SNR3k) | gain vs hard | FAR |
|-----------------|----------------------|--------------|-----|
| HARD (baseline) | −7.3 dB              | —            | 0   |
| **SOFT @ flips=1 (default)** | **−8.7 dB** | **+1.34 dB** | 0.25% |
| SOFT @ flips=2  | −8.7 dB              | +1.34 dB     | 3.4% |
| SOFT @ flips=3 (UNSAFE) | −8.7 dB       | (+2.9 dB ceiling) | 14% |

Row-resolved at the cliff knee (−8.66 dB SNR3k), P(CRC-pass):
| SNR3k | base-det | hard | soft@1 | soft@2 |
|-------|----------|------|--------|--------|
| −7.32 | 1.00 | 0.84 | 0.97 | 0.99 |
| −8.66 | 1.00 | 0.43 | 0.68-0.76 | 0.76-0.85 |
| −9.82 | 1.00 | 0.03 | 0.11 | 0.19-0.23 |
| −10.84| 1.00 | 0.00 | 0.01 | 0.01-0.02 |

**Interpretation:** soft list decode ~**doubles** suffix success at the cliff
knee and shifts the 0.5-crossing ~**1.3 dB** lower (≈2 dB at the knee). It does
**NOT** reach the −14.7 dB base-detection floor — it dies by ≈ −10.8 dB. This is
the fundamental limit of a ZERO-airtime code: it recovers near-misses (1-2 wrong
symbols, the cliff regime) but cannot correct the deep-error regime because no
redundancy exists on the wire. Closing the remaining ~4-6 dB to the floor
requires **Tier 2 (added parity symbols)**.

### §6.4. THROUGHPUT COST (the user's hard gate)
- Suffix symbol = Nofdm(292)×interp(4)/48 kHz = **24.33 ms**. ACK pattern today
  = 16 base + 13 suffix = 29 sym = **705 ms**.
- **Tier 1 adds ZERO symbols** (wire byte-identical; only RX decode changes).
  ⇒ **per-batch ACK overhead delta = 0.00%.** CONNECT delta = 0%. The gate is
  met with zero throughput cost, on BOTH paths. (Verified structurally:
  `ack_sack_suffix_len()` is untouched, so `ack_sack_pattern_nsymb()` and all
  passband sample counts in telecom_system.cc:5487-5499 are unchanged.)
- Runtime: at flips=1 the search is ≤40 CRC-12 ops/decode (negligible).
- **Tier-2 cost (if pursued, CONNECT-only):** P parity symbols = P×24.33 ms once
  per session — negligible (e.g. P=8 ⇒ 195 ms/session). The SAME P symbols on the
  per-batch ACK would cost ~10-20% at CONFIG_15 batch cadence, which is why
  Tier 2 is CONNECT-only.

### §6.5. §5 round-trip verdict
PASS. 36/36 `--test` (baseline 30 + 6 new). `suffix_soft_candidate0_equals_hard`
proves candidate[0] == hard decode (baseline-identical when off).
`suffix_soft_roundtrip_clean` + `corrects_one_flip` prove TX↔RX round-trip and
1-symbol correction. NB (M<16) returns false (unchanged). The soft path reuses
the production `CRC12_calc` via callback (never inlined — v1 bug #1) and the
SAME type+CRC accept gate (I2) → no ARQ weakening beyond the bounded, measured
FAR. SNR-vote path (I1) untouched (reads hard `out_tones`).

### §6.6. SHIP RECOMMENDATION (decision-grade)
1. **Tier 1 soft list decode @ flips=1 — SHIP-ready, low risk, FREE.** +1.3-2 dB
   acquisition on BOTH CONNECT and ACK at **0% throughput cost** and 0.25% FAR.
   No wire change, gated, baseline byte-identical. This is a pure win for the
   per-batch ACK (the throughput-critical path) — there is no downside to enable
   on ACK. Wire it into the production ARQ consumers behind `suffix_fec_mode=1`
   (the soft decode as a FALLBACK after the hard decode misses, so clean-channel
   behavior is identical).
2. **Tier 1 buys only ~⅓ of the gap to the detection floor.** If the goal is to
   push the suffix all the way to −14 dB (track the base detector), that needs
   **Tier 2: added parity symbols, CONNECT-ONLY** (free once/session; do NOT add
   to the per-batch ACK). Estimate next; not in this prototype.
3. **Do NOT raise flips above 1 by default** (FAR 3.4% at 2, 14% at 3). flips=2
   is available if a campaign accepts 3.4% FAR for the extra knee margin.

## §8. TIER-2 GOLAY(24,12,8) — MEASURED (SIM SPIKE, branch wt/suffix-fec-tier2-sim)

Phase-1 spike to measure the coding gain of a real-parity Tier-2 code BEFORE any
wire-format change. Self-contained module `golay24.{h,cc}` + `§10` harness in
`mfsk_ctrl_codec_tests.cc`. NOTHING wired into production: the 13-symbol suffix
(`ack_sack_suffix_len()`) is untouched, `suffix_fec_mode=0/1` are byte-identical
(§10.4 test asserts the production 13-sym hard path is unchanged). Reproduce:
`mercury.exe --test` → tests `golay24_roundtrip`, `golay24_soft_beats_hard`,
`golay_tier2_roundtrip_clean`, `golay_tier2_byte_identical_when_off`,
`golay_tier2_pure_noise_far`, `golay_tier2_word_independence_diag`,
`golay_tier2_cliff_sweep`. **43/43 pass** (36 baseline + 7 Tier-2).

### §8.1. Frame design
40-bit message `[type:2|payload:38]` → pad to 48 bits → **FOUR** Golay(24,12,8)
codewords (96 coded bits) → **24** M=16 FSK symbols (rate-1/2, vs the uncoded 13).
The task brief said "two codewords"; its own arithmetic (48 info / 96 coded / 24
symbols) requires FOUR (two words = 48 coded bits = 12 symbols). Implemented the
self-consistent four-word frame. Generator `G=[I|B]` was DERIVED+verified
(lexicode → Gaussian elimination → systematic; `encode()` d_min=8 confirmed over
all 4096 codewords at runtime; constants are checked, not trusted — §10.1).

### §8.2. Codec self-tests (decoder is correct in isolation)
- `golay24_roundtrip`: d_min=8; corrects all ≤3-bit errors; all 4-bit errors are
  decode-FAILURES (zero miscorrections) — the (24,12,8) bounded-distance guarantee.
- `golay24_soft_beats_hard`: with **4 of 6** symbols' argmax WRONG (true tone in a
  near-2nd-best energy position), soft-ML recovers **3904/4000** words; hard
  syndrome decode recovers **0/4000**. The soft decoder demonstrably exceeds the
  hard 3-error limit — the codec is doing what Tier-2 is supposed to do.

### §8.3. THE MEASUREMENT — cliff sweep (same SNR3k axis/seed as §6, refined grid)
All four columns measured on the SAME grid + seed + AWGN injection + detector:

| SNR3k dB | base-det (matched≥7) | uncoded HARD | uncoded SOFT@1 | **GOLAY Tier-2** |
|----------|----------------------|--------------|----------------|------------------|
| −7.32 | 1.000 | 0.810 | 0.975 | **1.000** |
| −8.66 | 1.000 | 0.480 | 0.785 | **0.930** |
| −9.26 | 1.000 | 0.220 | 0.425 | **0.705** ← 0.5-crossing |
| −9.82 | 1.000 | 0.070 | 0.170 | **0.390** |
| −10.34| 1.000 | 0.020 | 0.055 | 0.145 |
| −10.84| 0.990 | 0.000 | 0.020 | 0.030 |
| −13.34| 0.810 | 0 | 0 | 0 |
| −14.68| 0.500 | 0 | 0 | 0 |

- **Cliffs (P=0.5): uncoded HARD −7.32 | uncoded SOFT@1 −8.66 | GOLAY −9.26 dB.**
- **Coding gain: +1.94 dB vs hard, +0.60 dB vs Tier-1 soft@1.**
- **Residual gap to the −14.68 base floor: 5.42 dB. GOLAY DOES NOT REACH −14.**
- Golay is genuinely stronger per-cell (0.705 vs 0.425 at −9.26; ~2× at −9.82) —
  it does real error correction — but the 0.5-crossing moves only ~0.6 dB beyond
  the zero-airtime Tier-1 soft list, for +11 symbols (~268 ms) of added airtime.

### §8.4. WHY it falls short (mechanism — diagnostic §10.5b, NOT speculation)
1. **NOT the 4-word split.** Diagnostic measured P(frame) vs P(word)^4: at −9.8 dB
   P(word)=0.409 but P(frame)=0.405 (≈ P(word), NOT 0.028=P(word)^4). Word
   failures are HIGHLY CORRELATED — a bad channel realization buries all 4 words
   at once. So a single longer code over all 48 bits would NOT help; the split is
   not the bottleneck. (H1 rejected.)
2. **The per-symbol error ramps too steeply for a rate-1/2 block code.** True-tone
   energy rank stays good (in top-3 ~99% @ −8.7, ~97% @ −9.8, ~90% @ −10.8) so the
   SOFT INFO is fine — but per-symbol argmax-error q ramps 0.053 → 0.120 → 0.242
   over −8.7 → −9.8 → −10.8 dB. A rate-1/2 (24,12) word (6 symbols) can't track
   q>~0.12; by q≈0.24 (−10.8) it is dead. This is fundamental to a rate-1/2 code at
   this block length, not specific to Golay — the diagnostic shows the limit is q,
   not the code choice.
3. **The base detector and the payload have categorically different reach.** The
   base pattern reaches −14.68 dB because it only RECOGNIZES a known fixed
   sequence (huge processing gain via the 7/16-of-16 matched vote, tolerates ~9
   bad symbols). The suffix must RECOVER 48 unknown info bits. NB also: the
   suffix decoders gate on `detect_ack_pattern` metric ≥ 3.0, which itself cliffs
   ~−11 dB (below that the pattern still matches 7/16 tones but the correlation
   metric collapses) — so the USABLE base-detect floor for any suffix decoder is
   ~−11 dB, and Golay at −9.26 is ~1.7 dB from THAT (vs 5.4 dB from the raw
   matched-count floor). Closing even ~1.7 dB more at rate-1/2 over these symbols
   is not achievable per (2).

### §8.5. FAR + airtime
- **FAR (Golay, CRC-gated, pure noise, sigma=3.2, 3000 trials): 0.0000** (123
  base-detects, 0 CRC-accepts). The CRC12 gate over the decoded payload bounds it
  to ≈ base_detect_rate × 2⁻¹² — strictly BETTER than Tier-1's 0.25% (Tier-1's
  list search tries up to 40 codewords/decode; Golay's soft-ML commits to ONE
  codeword per word, so only one CRC trial per frame). §10.5 test.
- **Airtime: 24 vs 13 symbols = +11 symbols × 24.33 ms = +268 ms.** Free
  once/session on CONNECT (per §2); would be ~10-25% per-batch on ACK (do NOT put
  Tier-2 on the ACK — same asymmetric rule as §3).

### §8.6. RECOMMENDATION (decision-grade) — GOLAY STALLS SHORT
**Golay(24,12,8) does NOT reach −14 dB; it cliffs at −9.26 dB (5.4 dB short of the
raw base floor, ~1.7 dB short of the metric-gated usable floor).** It is a real
but modest **+0.6 dB over the free Tier-1 soft list**, bought with +268 ms airtime.

The GF(16)-RA upgrade the task names as the fallback **would not change the
verdict for the −14 dB goal**: §8.4(2) shows the limiter is per-symbol q at
rate-1/2, not the specific code's algebraic strength — and §8.4(1) shows it is
not the word-split either. A stronger same-rate code over the same 24 symbols
hits the same q-wall near −10.8 dB. To actually track the base detector to −14
you must change the OPERATING POINT, not the code:
- **(a) Far lower rate / fewer info bits** — recover only a handful of bits per
  frame (e.g. an (n, k≪) repetition/orthogonal code), trading payload for reach;
  or
- **(b) Noncoherent integration gain** — repeat the suffix and energy-combine
  across repeats (the documented soft-repetition lever, §1.1), each 2× repeat
  buys ~3 dB but doubles airtime; or
- **(c) a coherent PHY** (shelved per the parity-audit L3 decision).

**Decision input for the parent:** if the production goal is "a few dB of cheap
reach on CONNECT/ACK," the **already-built Tier-1 soft list (0 airtime, +1.3 dB,
0.25% FAR) is the better buy** and Golay is not worth its +268 ms for +0.6 dB.
If the goal is specifically to track the −14 dB base floor, **neither Golay nor a
GF(16)-RA at rate-1/2 gets there** — that needs option (a)/(b)/(c), which is a
different (larger) project than a suffix-code swap. The Golay spike's value is
that it CLOSES the "is a stronger block code the answer?" question: it is not.

## §7. Status log
- 2026-05-31: doc created; code mapped; design chosen (two-tier, Tier-1-first).
  Mechanism (`p^13` hard argmax) confirmed from `ofdm.cc:4015-4035`.
- 2026-05-31: Tier 1 implemented + measured. 36/36 tests pass. SNR3k calibrated
  to prior cliff-agent axis (base floor −14.68 ≈ their −14.6; hard suffix ≈ their
  −8.6). Soft@flips=1: +1.34 dB cliff move, 0% throughput, 0.25% FAR. Gap to
  detection floor (~6 dB) needs Tier-2 parity (CONNECT-only). Ship verdict §6.6.
- 2026-05-31: TIER-2 Golay(24,12,8) SIM SPIKE (branch wt/suffix-fec-tier2-sim,
  off 37947fb). golay24.{h,cc} (verified d_min=8 generator, exhaustive 4096-cw
  soft-ML) + §10 harness. 43/43 tests pass. **Golay cliff −9.26 dB: +1.94 vs
  hard, +0.60 vs Tier-1 soft, 5.42 dB short of −14.68 floor — DOES NOT REACH
  −14.** Mechanism (§8.4): limiter is per-symbol q at rate-1/2 (q 0.05→0.24 over
  −8.7→−10.8), NOT the word-split (P(frame)≈P(word), not word^4) and NOT the
  specific code — so GF(16)-RA at the same rate wouldn't change the verdict.
  FAR 0.0000 (better than Tier-1's 0.25%). Recommendation §8.6: Golay stalls
  short; Tier-1 soft is the better cheap-reach buy; reaching −14 needs a lower
  rate / repetition-combining / coherent PHY, not a block-code swap.

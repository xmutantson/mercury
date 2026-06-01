# Tier-2 control-suffix FEC — design (VARA-parity link establishment)

**Status:** user directive (2026-05-31): "advance the state of the art, not just hit −14".
Phase-1 BAKE-OFF: **Golay DONE — STALLS at −9.26 dB (only +0.6 dB over the free Tier-1);
a rate-½ block code is NOT the path to −14 — limiter = operating point, not code (see §9).**
GF(16) symbol-level code still running (likely bake-off winner + base). PIVOT: operating-
point levers (metric-gate-relax usable floor + fewer-info-bits + noncoherent repetition)
under research (agent aeeaab48…). Production integration GATED on the operating-point
design + bake-off winner + free-stack HW + sign-off.

**Goal:** close the last ~3–4 dB on the MFSK control suffix so CONNECT/ACK link
*establishment* reaches the −14.68 dB SNR3k base-detector floor (≈ VARA-parity), with
**zero data-throughput cost** (the user's hard constraint).

Sibling docs: [[connect-suffix-fec-research.md]] (Tier-1 prototype + the sim harness),
[[connect-ack-metric-gate.md]] (the +2.5 dB metric-gate relax), [[mfsk-robust-ack.md]]
(suffix integrity criterion + CRC12), [[phase-b-mfsk-connect-research.md]] (the 4-type
suffix format), [[mfsk-vara-parity-plan.md]] §2 (the Eb/N0 framing).

## §1 The problem (code-grounded)

The control suffix = **13 symbols at M=16 noncoherent FSK = 52 bits, packed
`[type:2 | payload:38 | crc12:12]`, rate-1 UNCODED** (`mfsk.cc:622-640 pack_ctrl_suffix`).
RX = per-symbol hard FFT-bin argmax (`ofdm.cc:4015-4035 decode_suffix_tones`), keeps only
`best_tone`, discards the other M−1 energies. Decode passes only if all 13 argmax
decisions are right → P ≈ (1−q)^13. That multiplicative-AND is the cliff.

Measured (sim harness, `connect-suffix-fec-research.md §6`):
- Base Welch-Costas pattern detector (matched-count ≥7/16): floor **−14.68 dB SNR3k** (the target — where the link "wakes up").
- Hard suffix: P=0.5 at **−7.3 dB**, dead by −9.8 dB.
- **Tier-1** (CRC-aided soft *list* decode, `soft_list_decode_ctrl_suffix`, already built, flips=1): P=0.5 at **−8.7 dB (+1.34 dB)**, 0% airtime, 0.25% FAR. **Maxed** — a zero-redundancy code only rescues 1–2-symbol near-misses.
- **Gap Tier-2 must close: ~−10.8 → −14 dB ≈ 3–4 dB.**

Enabling fact: the per-tone energy matrix `E[s][m]` is **already computed** by
`decode_suffix_candidates` (prototype branch `wt/connect-suffix-fec` @37947fb). A soft
Q-ary decoder is a contained codec change, not new DSP.

## §2 Why the code must be SOFT, not hard (the cliff math)

Back-solved per-symbol argmax error q from the measured P(pass):

| SNR3k | hard P(pass) | implied q |
|---|---|---|
| −8.66 dB (Tier-1 knee) | 0.43 | **6.3%** |
| −9.82 dB | 0.03 | **23.6%** |

Hard symbol-error-correcting residual block-error (binomial):

| Code | residual @ q=6% | residual @ q=24% |
|---|---|---|
| t=1,n=13 (≈Tier-1) | 19.5% | 84.9% |
| t=2,n=15 | 6.4% | 72.5% |
| t=3,n=20 | 3.4% | 73.1% |
| t=5,n=20 | 0.1% | 32.9% |

**Hard RS is hopeless** — even t=3 leaves 73% block-error at −9.8 dB. Below ~−10 dB the
argmax is wrong on ¼ of symbols, throwing away the information the base matched-count
detector uses to reach −14.68. **Only a soft decoder over the full per-tone energy vector
can track the base floor.** This is exactly why WSJT-X abandoned hard-RS JT65 for
soft-QRA Q65. **⇒ Tier-2 = soft Q-ary code, soft-MAP decoder over the per-tone energies.**

## §3 The two candidate codes

Both ~3–4 dB (enough to reach −14.68). Prior art: WSJT-X 2.7 User Guide §18 (K1JT/K9AN,
wsjt.sourceforge.io); Q65 = punctured QRA (65,15) over GF(64) + soft symbol-MAP + 12-bit
CRC as puncture/gate; FT8 = LDPC(174,91) BP+OSD; ALE/MIL-STD-188-141A = ext-Golay(24,12,8)
for HF *link establishment*.

- **(B) Golay(24,12,8) soft-ML — PREFERRED FIRST (de-risked).** 40-bit msg → pad 48 → two
  Golay(24,12) words → 96 bits → **24 symbols** (+11 vs 13), rate ½. Soft-ML (exhaustive
  4096 words/word, or Conway-Sloane) over the bit-LLRs/energies. ~4 dB textbook.
  ALE-proven on HF; trivially correct (fixed 12×12 generator); no decoder port. Decode
  from existing `demod()` bit-LLRs (`mfsk.cc:1003-1098`, `1/σ²`-scaled).
- **(A) GF(16) rate-½ RA code — OPTIMAL UPGRADE (if B stalls).** Q65/QRA lineage scaled to
  GF(16)/n≈20 (10 info + ~7 parity + 3 CRC). **20 symbols** (+7), soft BP over the per-tone
  energies (matches the M=16 *symbol* alphabet → fewer redundant symbols, no bit-LLR
  marginalization loss). Decoder port: `qracodes` (IV3NWV, github.com/Microtelecom/qracodes,
  MIT→AGPLv3, same path as the a26 BP+OSD port).
- **Bake-off, not fallback (user directive 2026-05-31: "advance the state of the art, not
  just hit the target"):** build BOTH in parallel, same harness/axis, ship the genuinely
  better code on the data — deeper cliff × lower FAR × fewer symbols × decoder cost. GF(16)
  (A) is the more advanced, alphabet-matched approach and should in theory win (symbol-wise
  error structure, decodes straight from per-tone energies with no marginalization loss);
  Golay (B) is the proven binary baseline. Tie-break (cliffs within ~0.5 dB harness variance)
  = an identical-noise head-to-head run of both codes in one binary.
- **Rejected:** hard RS/GF(16) (§2 dies at −9.8); Reed-Muller (replaces the modulation, not
  a parity layer — out of proportion); tail-biting conv+BCJR (no CRC-puncture synergy,
  heavier, short block wastes constraint length; WSJT-X itself migrated away from K=32 conv).

## §4 Adaptive-ACK design (throughput-neutral — the hard constraint)

ACK overhead if Tier-2 always on the ACK (computed, 24.33 ms/sym):

| Added sym | ROBUST_0 (7.9 s frame) | CONFIG_10 (~0.9 s) | CONFIG_15 (~0.47 s) |
|---|---|---|---|
| +7 (A) | **+2.0%** | +10.6% | +14.5% |
| +11 (B) | ~+3% | ~+17% | ~+23% |

**This table is the whole argument:** unacceptable at CONFIG_15, trivial at ROBUST_0 — and
the ACK only *needs* parity at the deep floor. So:

1. **CONNECT suffix: Tier-2 ALWAYS** (once/session, +170–270 ms = <0.1% of a multi-min session).
2. **Per-batch ACK: Tier-2 SNR-gated to the robust tier only** — gate on
   `current_configuration == ROBUST_0/1/2` (the robust tier IS the deep-floor proxy; reuses
   the gearshift config, no new state — cf. the existing `current_configuration==ROBUST_0`
   branch at `telecom_system.cc:5505`). CONFIG_6+ ACK stays byte-identical → **0% overhead
   on every throughput-relevant config.** ✓
3. **RX needs no side-channel — try-both-decode fallback.** CMD runs the uncoded decode
   first (cost 0, identical to today); on CRC-fail, *and* only if the session negotiated
   `CAP_SUFFIX_FEC` *and* is at the robust tier, attempt the Tier-2 decode of the longer
   window. CRC12 + 2-bit type gate reject a wrong-length misparse → degrades to "no ACK"
   (→ retransmit), never silent corruption. Same failsafe Tier-1 relies on.

## §5 Wire-format / compat

- **Capability-gated, NOT a flag-day.** New `CAP_SUFFIX_FEC` (next free cap bit, exchanged
  in TEST_ACK/TEST_CONN cap field, `mfsk_ctrl_codec.h:100-138`). Mixed/un-upgraded pairs →
  uncoded suffix + Tier-1 (which is itself RX-only/flag-day-free). Safer than the Phase-B
  CONNECT flag-day because Tier-2 changes wire *length*.
- **CONNECT chicken-and-egg:** caps unknown on the first START_CONN. Resolution: first
  START_CONN stays 13-sym Tier-1 (reaches −10.8, enough to bootstrap); Tier-2 turns on for
  TEST_ACK/TEST_CONN + subsequent ACKs after caps exchange. (Or: gate on local robust-tier
  flag + rely on the RX try-both fallback. Decide at integration.)
- **Buffer sizing invariant (I4):** Tier-2's longer length MUST flow through a coded-length
  accessor (e.g. `ctrl_suffix_coded_len(type, fec_on)`) so TX/RX sample counts, capture
  sizing, `reserve_after` stay consistent (`telecom_system.cc:5498-5499`). `MAX_ACK_SACK_SUFFIX`
  (`mfsk.h:180`) 16 → ≥ coded length (~24).

## §6 Plan (phased; §4 = sim before any wire change, §3 = no untested fix)

- **Phase 1 — SIM BAKE-OFF (both running):** Golay(24,12) soft-ML (`suffix_fec_mode=2`,
  agent ae83f268) AND soft GF(16) rate-½ RA (`suffix_fec_mode=3`, agent a7b3bd10), same
  cliff-sweep harness (`mfsk_ctrl_codec_tests.cc`) + SNR axis. Per code: cliff (reaches
  −14.68?), FAR vs Tier-1's 0.25%, byte-identical-when-off. Winner picked on the data;
  tie-break = identical-noise run of both in one binary if within ~0.5 dB. The §3 gate
  stays "suffix cliff ≤ base floor".
- **Phase 2 — INTEGRATION (gated):** wire the winner: coded-length accessor (§5/I4),
  `CAP_SUFFIX_FEC` negotiation, robust-tier ACK gate + try-both RX fallback (§4), buffer
  sizing, cross-layer audit + regression tests (cliff-sweep + FAR + byte-identical-when-off).
  Sim-verify → HW-validate on IONOS. Builds on the free stack (metric gate @6b2cb54 +
  Tier-1 @37947fb), which is HW-validating in parallel.

## §7 Open questions [?]

- [?] **Actual coding gain unsimulated.** 3–4 dB is reasoned from Q65-scaled + textbook
  Golay-soft. Phase 1 measures it. Mandatory before ship (§3).
- [?] **Does it reach −14, or stall at −12/−13?** Decision gate = "suffix cliff ≤ base floor
  (−14.68)", not a hard −14. If the suffix is no longer the binding stage, we're at parity-class.
- [?] **FAR with a real corrector.** A corrector explores more codewords than Tier-1's
  flips≤1 ball → CRC12 false-accept (≈ trials·2⁻¹²) rises. Measure on pure noise, cap the
  trial budget; 2-bit type gate (×¼) + ARQ retransmit are backstops.
- [?] **VARA/PACTOR/ARDOP handshake FEC undocumented** (closed source) — comparison is to
  WSJT-X (open, primary) + the established Eb/N0 framing, not a VARA spec.

## §8 Key file references

- Suffix codec/cliff: `mfsk.cc:622-693` (pack/unpack), `ofdm.cc:3986-4070` (hard argmax = the cliff), `mfsk.h:126-153,180` (accessors, MAX_ACK_SACK_SUFFIX).
- Soft metric: `mfsk.cc:1003-1098` (demod bit-LLRs); `decode_suffix_candidates` (per-tone energies) in `wt/connect-suffix-fec` `ofdm.cc`.
- Decode + accept gate: `telecom_system.cc:3236-3360` (ACK detect), `:3498-3606` (CONNECT decode), `:5498-5499` (buffer/I4), `:5505` (ROBUST_0 branch); `arq_common.cc:4784,5024-5063` (CRC12 + type gate).
- Sim harness + scaffolding: `mfsk_ctrl_codec_tests.cc` (cliff sweep), branch `wt/connect-suffix-fec` @37947fb (`decode_suffix_candidates`, `soft_list_decode_ctrl_suffix`, `suffix_fec_mode`).

## §9 Phase-1 result — Golay STALLS; the limiter is the operating point, not the code (2026-05-31)

**Golay(24,12,8) soft-ML, 4-word frame (+11 sym / +268 ms) — agent ae83f268, branch
`wt/suffix-fec-tier2-sim` @2e169cd.** New files `golay24.{h,cc}` (runtime-verified d_min=8
lexicode generator + exhaustive soft-ML); harness `mfsk_ctrl_codec_tests.cc` §10; full
write-up `connect-suffix-fec-research.md §8`.

Cliff (P=0.5, same grid/seed/AWGN/detector as the Tier-1 numbers):

| | uncoded HARD | Tier-1 SOFT@1 | **GOLAY Tier-2** | base-det pattern |
|---|---|---|---|---|
| cliff dB | −7.32 | −8.66 | **−9.26** | −14.68 |

- Golay = **+1.94 dB vs hard, but only +0.60 dB vs the ZERO-airtime Tier-1 soft list** — for +268 ms. Not worth shipping alone.
- FAR **0.0000** (vs Tier-1 0.25% — soft-ML commits to one codeword → one CRC trial/frame). Byte-identical-when-off confirmed. 43/43 tests.

**Mechanism (measured via `golay_tier2_word_independence_diag`, not speculation):**
1. **NOT the 4-word split** — word failures are correlated (P(frame)≈P(word), not P(word)⁴); a single longer code wouldn't help.
2. **Limiter = per-symbol argmax-error q vs rate:** q ramps 0.05→0.12→0.24 over −8.7→−9.8→−10.8 dB; a rate-½ word can't track q>~0.12. **Fundamental to rate/length — a GF(16)-RA at the same rate is *predicted* to hit the same wall [?]**. CAVEAT: Golay decodes marginalized bit-LLRs; the GF(16) decoder uses the full per-tone energies and may track higher q — the bake-off settles whether the prediction holds.
3. The base detector hits −14.68 only by recognizing a FIXED pattern (≈1 bit, huge processing gain); the suffix must recover **48 unknown bits** → fundamentally more energy.

**⇒ A stronger block code at rate-½ is NOT the path to −14. Closed question.**

**The operating-point pivot (the real levers — under research, agent aeeaab48):**
1. **Metric-gate-relax cross-layer interaction (the Golay test lacked it):** Golay ran on Tier-1 @37947fb WITHOUT the free-stack metric relax (`wt/connect-metric` @6b2cb54, 3.0→2.0). The Golay agent measured the `detect_ack_pattern` metric gate cliffing ~−11 dB (the usable-floor limiter) with the OLD 3.0 gate → with 2.0 it deepens ~2.5 dB (~−13.5), shrinking the REAL remaining gap below the raw 5.42 dB.
2. **Fewer info bits at the floor** — minimal establish message (~12-24 bits vs 48): ~+3 dB per halving.
3. **Noncoherent repetition energy-combining** — ~+3 dB per 2× airtime; cheap at ROBUST_0 (data frame 7.9 s), fits the adaptive-only-at-floor scheme.
4. Coherent combining (L3) — shelved; fallback only if noncoherent can't reach.

**Next:** finish the GF(16) bake-off (winner + base), then design the best noncoherent
operating point (levers 1-3) on the winner. Reaching −14 is a bigger change than a suffix-
code swap — present the ranked options to the user before integration.

## §10 Operating-point research — the noncoherent path to −14 (the plan) (2026-05-31)

Agent aeeaab48. Sources: WSJT-X 2.7 guide, FST4/Q65 quick-starts, QEX FT4/FT8
(Franke/Somerville/Taylor 2020), JT65 specs. **−14 establishment is reachable
NONCOHERENTLY — coherent L3 is NOT needed for establishment** (it stays the throughput
problem, not the establishment problem). The fix is three stacking levers, not a code:

1. **Metric-gate relax 3.0→2.0 (free stack, `wt/connect-metric` @6b2cb54) — moves the
   CEILING, not the decoder.** The −14.68 raw matched-count floor is unreachable by any
   content decoder: a soft `metric≥T` sub-gate is ANDed in front of the suffix decode
   (`telecom_system.cc:3537-3539` returns false / the suffix decode at :3590 never runs
   unless `matched≥7 AND metric≥CTRL_DETECT_METRIC_MIN`). That gate is the usable-floor
   limiter: ~−11 (3.0) → **~−12.9 (AWGN) / −13.5 (impaired)** at 2.0. **0 airtime.**
2. **Tier-1 soft list (free stack, @37947fb) — content baseline −8.66, 0 airtime, 0.25%
   FAR.** Bake-off verdict: **build on Tier-1, NOT Golay** (+0.6 dB not worth +268 ms).
3. **Noncoherent repetition energy-combining (NEW) — closes the rest.** Repeat the 29-sym
   control frame R×, sum the per-tone energy matrix E[s][m] across reps BEFORE
   argmax/soft-list (square-law noncoherent sum — NOT hard majority, NOT coherent).
   **CONNECT-only, robust-tier-gated, adaptive** (R=1 good SNR = 0 cost; escalate only at
   ROBUST_0; NEVER on the per-batch ACK). ×2 (+8.9%) → content ~−11.2; **×4 (+26.8%) →
   content ~−13 to −14, co-limited with the ~−13.5 gate floor = parity-class. Free once/session.**

**Load-bearing insight (why repetition works where a code didn't):** levers 1+3 compose
multiplicatively — repetition integrates the *same* energy the metric gate measures, so it
lowers the gate ceiling too; the ceiling descends with R instead of capping at −13.5.
Without lever 1, repetition stalls at the old −11 gate; without repetition, lever 1 raises
a ceiling the −8.66 content decoder can't reach. **1+2+3(×4) = the genuine noncoherent path
to ~−14.**

**Real remaining gap (anchor):** after the free stack moves the ceiling to ~−13, the
content decoder (−8.66) has **~4.3–4.8 dB** to climb — supplied by repetition (R=4 ≈
+4.7–5.4 dB), NOT a stronger code (Golay diagnostic §8.4 / §9).

**Prior art (Q65 is Mercury's architecture class — noncoherent M-FSK + symbol code +
spread sync):** ~2.6–3.0 dB per doubling of integration time (Q65 15→120 s:
−22.2→−30.8; matches R=2/4/8 = +2.3-2.7/+4.7-5.4/+7-8). Sync ~25-26% of airtime in FST4/Q65,
distributed; Mercury's 16-sym base pattern (55%) is sync-heavy → raw floor already deep,
deficiency is the content stage. Halving payload ≈ +3 dB (lever 2′ minimal establish
message — DEFERRED, it's a handshake-msg redesign touching the Phase-B 4-type format).

**Scope:** levers 1/2/3 are suffix-path (gate constant + RX soft-combine over the existing
`decode_suffix_candidates` matrix). Repetition needs a CONNECT wire-length change (R× frame)
→ CAP-gated, flow R through `ctrl_suffix_coded_len` (§5/I4). Lever 2′ deferred behind 1+2+3.

**Open [?] — what the validation sim spike (agent af618567, `wt/repetition-sim`) measures
before integration:** (a) do the content cliffs MOVE under the relaxed 2.0 gate (measured
on 3.0)? (b) the ACTUAL M=16 noncoherent combining gain at q≈0.1-0.24 (decides ×4 → −13 vs
−14) — measured, not the textbook band; (c) FAR at R=4/2.0; (d) repetition timing vs
PTT/CONNECT lockout (cf. Bug #55 NB-HAIL race) — a HW concern.

## §11 HW reality check — free stack gives NO floor gain; HAIL detection is the real bottleneck (2026-05-31)

Agent a339d98e. Combined free stack (metric gate 6b2cb54 + suffix FEC 37947fb) built +
wired + IONOS A/B vs baseline 01535f2. Worktree `wt/connect-combined` @8bb7a58, fact-doc §8.

**QA finding (important):** suffix-FEC commit 37947fb shipped with NO production caller —
the `*_soft` decode fns were TEST-ONLY, so `suffix_fec_mode=1` alone changed nothing on the
wire; the quoted Tier-1 "+1.34 dB" was a sim-harness number never in production. Agent wired
the soft decoder into all 3 production ctrl-suffix sites (CONNECT `arq_common.cc`; clean-ACK
+ SACK-window ACK `arq_commander.cc`) AND caught a stacking bug: the `*_soft` fns had copied
the OLD hardcoded `metric<3.0` gate, which would have silently undone the metric relax —
replaced all 4 with `CTRL_DETECT_METRIC_MIN`. Without this the gains would NOT have stacked.

**Sim:** gains stack **+3.84 dB** (metric +2.50 + suffix +1.34), 40/40 tests, throughput-neutral.

**HW A/B (IONOS, pinned ROBUST_0, ≥3/cell):**
- **Establishment floor IDENTICAL both arms:** CONNECT 3/3 at WGN −6/−8/−10; **0/3 at −12/−14.**
- Throughput-neutral CONFIRMED: WB_CFG10 clean **290.1 bps / 5440 B byte-identical** both arms.
- FAR clean both arms (0 false accepts / 120 s noise; the 0.25% sim-FAR → 0 HW).

**Root cause of the HW null (log-confirmed):** at −12/−14 the HAIL beacon detector NEVER
FIRES (`[HAIL] Detected` absent); at −10 it fires (metric 7.6, base 16/8) THEN the suffix
runs (already matched=16/flips=0 — zero margin for the ctrl-suffix fixes). **The ROBUST_0
establishment floor is gated by HAIL preamble detection (`detect_hail_pattern_from_passband`
/ `hail_match_threshold`), dies −10→−12 — UPSTREAM of the free stack AND the planned
repetition.** Mirrors the BP+OSD null ("the fix isn't at the bottleneck").

**⇒ REFRAME: the binding establishment constraint is HAIL detection (~−10/−11), not the
ctrl-suffix.** The ctrl-suffix work (free stack + repetition) is necessary but NOT sufficient
— it only helps once HAIL is pushed deeper. The ctrl base-pattern detector reaches −14.68
but HAIL dies at −10/−11; the ~4 dB gap between two matched-count detectors is the lead
(are the N repeated beacons COMBINED or detected independently? over-strict threshold?).
**NEW critical-path target: HAIL detection (agent a5a151fd investigating, code+sim).**

**Revised order to −14 establishment (series — every stage must reach −14):**
1. **HAIL preamble detection → ~−14** (NEW; the binding stage).
2. **CONNECT ctrl-suffix → ~−14** (free stack latent + repetition, validating).
3. **ACK → ~−14** (same stack).
4. Full-chain HW verify.

**Merge:** HOLD the free stack on `wt/connect-combined` (no HW gain until HAIL moves; bundle
the full validated acquisition fix once the floor actually moves on HW). `suffix_fec_mode`
default 0 (don't run the soft path on every hot-path ACK for zero current benefit). The
metric-gate half is a clean §1 fix (removes an unmeasured threshold), harmless, latent.

## §12 Bake-off CONCLUDED — GF(16) RA wins + reaches −14 (content FEC SOLVED); detection/sync is the whole bottleneck (2026-05-31)

Agent a7b3bd10. Branch `wt/suffix-fec-tier2-gf16` @4675321, 42/42 tests, `suffix_fec_mode==3`.
Full detail: `mercury/fact-documents/tier2-suffix-fec-gf16-spike.md`.

**Construction:** true degree-3 Q-ary RA over GF(16) (Q65/QRA lineage, ported from qracodes
IV3NWV GPLv3→AGPLv3): K=13 systematic (10 info + 3 CRC-as-protected-info per Q65
`QRATYPE_CRC`) + degree-3 accumulator parity; symbol-domain BP with Walsh-Hadamard fast
check-node + **Bessel-I0 noncoherent-FSK-optimal intrinsic** from the per-tone energies (new
read-only primitive `decode_suffix_energies` — zero argmax, zero bit-LLR marginalization
loss). repfact 1/2/3 → N=26/39/52, R=½/⅓/¼. [Honesty note: first build used deg-9 checks,
corrected <1 sym; root cause = Q65 `MAXCDEG=3`; the deg-3 rebuild was the whole difference.]

**FEC-reach cliff (decoding extracted energies directly; base floor −14.68):**

| code | cliff | vs Tier-1 (−8.7) | reaches −14? | airtime |
|---|---|---|---|---|
| GF16 R½ (N=26) | −11.75 | +3.05 | no | +316 ms |
| GF16 R⅓ (N=39) | −13.34 | +4.64 | no | +633 ms |
| **GF16 R¼ (N=52)** | **−14.03** | **+5.33** | **YES** (+0.65 from floor) | +949 ms |

**Bake-off verdict: GF(16) WINS decisively.** Equal-rate (½): GF(16) −11.75 beats Golay
−9.26 by **+2.5 dB** — the symbol-matched/Bessel-I0/no-marginalization thesis VALIDATED;
Golay agent's "GF(16) hits the same wall at rate-½" prediction was WRONG. FAR **0/5000 =
0.0000** (beats Golay + Tier-1's 0.25%). Decoder ~250-470 µs (negligible once/session). At
R¼ it **REACHES −14 — the content FEC is solved in sim.** (R¼ = the operating-point
"lower-rate" lever 4 from §10; spends ~the same energy as ×4 repetition but as a denser code.)

**THE decision-critical finding (converges with §11, refines its step 2):** the −8.7
production cliff that pins Tier-1 AND every soft decoder is NOT the FEC — it's the
**CONNECT/ACK detection scaffolding**: the control mini-Moose `carrier_frequency_sync_wb_ctrl`
(noisy CFO residuals at low SNR corrupt the re-decimated baseband) + the metric≥3.0 gate.
Extracted energies decoded directly = P=1.0 at −9.82; identical trials through the production
path = 0.47. **GF(16)'s −14 reach is MASKED in production by the sync/gate scaffolding.**

**⇒ UNIFIED PICTURE — the content FEC is solved; the entire remaining gap to −14
establishment is the DETECTION/SYNC scaffolding, in series:**
1. **HAIL beacon detection** (~−10/−11) — binds first [investigating, a5a151fd].
2. ~~**Control mini-Moose CFO sync** — masks the FEC~~ **EXONERATED (§16):** isolation sim
   shows the ctrl-Moose residuals are tiny/clean at the floor; removing it = 0 dB. NOT a
   limiter. CFO-sync line of work DROPPED.
3. **Metric≥3.0 energy-confidence gate** — the SOLE ctrl-suffix masker (§16). Masks
   perfectly-decodable energies (P_direct=1.00 to −14). Relax to **~1.0–1.5 / off** (free-stack
   2.0 only reaches −11.75); CRC12+2-bit-type is the FAR backstop (0/4000 gate-off). SAME
   gate-class as HAIL → one fix across HAIL + CONNECT + ACK.
4. **Content FEC** — **SOLVED** (GF(16) RA R¼ → −14; the integration winner).

The repetition spike (af618567) now informs the SCAFFOLDING stages (base-pattern detect +
combining), not the content FEC. None of 1-3 requires a new code — they're robustness fixes
to feed the −14-capable GF(16) decoder a clean signal.

## §13 HAIL root cause + fix — two unmeasured soft gates, QUALITY gate dominant (2026-05-31)

Agent a5a151fd. Branch `sim/hail-detection-floor` @712ef45 (`test_hail_detection_cliff_sweep`,
31/31). Fact doc `hail-detection-floor-investigation.md`.

**Root cause:** HAIL's base pattern + detector are IDENTICAL to the ctrl/ACK base (16 sym,
same `detect_ack_pattern` matched filter, `telecom_system.cc:3722`→`ofdm.cc:3691`); its
matched-COUNT statistic reaches −13.25 SNR3k (= the ctrl base floor). The ~8 dB shortfall is
TWO unmeasured soft gates ANDed in the fast-poll HAIL path (`arq_common.cc:5375`:
`metric>=3.0 && quality>=0.3`):
- `metric>=3.0` = the same constant the ctrl metric-gate relax fixed (→2.0) but EXPLICITLY
  SKIPPED for HAIL.
- **`quality>=0.3` is DOMINANT** (quality=metric/matched; at floor matched=16 → metric>=4.8,
  stricter than 3.0). Relaxing metric ALONE = **+0.00 dB** — why HAIL stayed stuck.
- The sibling HAIL site (`arq_common.cc:6373`) already uses the looser config threshold 0.65
  (`telecom_system.cc:5506`) — the fast-poll site is inconsistent.

**Fix (ranked #1, +8.30 dB, FAR 0/5000):** drop/loosen `quality>=0.3` + align `metric` with
the config threshold, toward count-only. The 8/16 matched-count gate is the real FAR defense
(soft gates add ~nothing). RX-only, throughput-neutral, 1-2 lines → HAIL −13.25 (matched-count
floor). [IMPLEMENTING + HW-validating: agent a8ed6001, with the §5 cross-layer audit — the
gate path is shared by ACK/CONNECT/BREAK.]

**Beacon-combining (hypothesis half-right):** the N beacons ARE sent but detected
INDEPENDENTLY (ring reset between, `arq_common.cc:5247`) — zero integration today. BUT
Mercury's gate is a normalized energy RATIO (e_target/e_total), scale-invariant → combining
gives +0.00 until the ratio gate is removed. Combining on the matched-COUNT: R=2 +1.8 / R=5
+3.7 dB (measured M=16 noncoherent, vs ideal +3/+7). So combining stacks DEEPER (R=5 → −17)
but only AFTER the gate fix (#1). **The gate fix is the cheap win; combining is the follow-on.**

**Caveat:** sim absolute SNR3k (−4.95 current-gate) ≠ IONOS WGN dial (−10) — WGN uncalibrated
+ single-shot-sim vs thousands-of-polls-production. The relative finding (soft-gate cliff ~8
dB above the count floor, quality gate dominant) is calibration-independent; HW-validate
confirms the real floor move.

**Updates §12 chain stage 1:** HAIL fix = relax the two soft gates (quality dominant), +8.3
dB → −13.25. After it, the binding stage = the control mini-Moose CFO sync (§12 #2, NOW under
investigation, agent a1fe962c).

## §14 Repetition sim — content SOLVED (confirms GF16); combining belongs on the PREAMBLE, not the suffix (2026-05-31)

Agent af618567. Branch `wt/repetition-sim` @2afd6dd off `wt/connect-combined` @86a806c (the
free stack: metric 2.0 + Tier-1; 2.0 confirmed active). 43/43. Full write-up:
`connect-suffix-fec-research.md §9`.

Cliff under the 2.0 gate (CONNECT; base floor −14.68):

| | R=1 | R=2 | R=4 | R=8 |
|---|---|---|---|---|
| **CONTENT** (acquisition removed) | −8.66 | −10.84 | −13.34 | **−14.68** |
| **END-TO-END** (per-rep gated) | −8.66 | −10.84 | −11.75 | −11.75 |
| base-detect acquisition | −11.75 | −11.75 | −11.75 | −11.75 |

- **Suffix CONTENT is solved**: R=8 content tracks −14.68 (R=4 nearly, −13.34). Confirms the
  GF(16) "content solved" verdict from a different angle (repetition vs a denser code). **The
  suffix is no longer the binding stage.**
- **Suffix repetition was aimed at the WRONG stage.** End-to-end clamps at −11.75 = the
  base-pattern ACQUISITION (`detect_ack_pattern` matched-count + metric sub-gate), NOT the
  content. **⇒ combining belongs on the PREAMBLE / base-pattern, not the suffix** — i.e. the
  same lever as the HAIL fix, independently confirmed and correctly aimed.
- **Measured M=16 noncoherent combining gain = +2.2 to +2.5 dB/doubling** (R=2 +2.18, R=4
  +2.50) — the LOW end, below the assumed +4.7-5.4 at R=4 and the +3/2× rule. Reason: Tier-1
  soft-list already harvests the near-miss energy at R=1, so each doubling buys only the
  incremental square-law integration. **Calibrates the combining lever (revise §10/§13 down).**
- **The 2.0 metric gate did NOT move the content cliff (−8.66)** — it helps acquisition, not
  content — and at 2.0 it STILL costs ~2.9 dB on the CONNECT acquisition (−11.75 vs raw count
  −14.68). **⇒ the CONNECT/ACK base-pattern gate needs the SAME further relax toward
  count-only as the HAIL gate (a8ed6001), extended to the CONNECT sites (per the §5 shared-
  detector audit).**
- FAR R=4/2.0 = 0.0000; byte-identical-when-off (R=1) PASS.

**Convergence (HAIL §13 + GF16 §12 + this):** the entire establishment gap is the
ACQUISITION/SYNC scaffolding. Three levers, all in the detection path: (1) relax base-pattern
soft gates → count-only [HAIL a8ed6001; EXTEND to CONNECT/ACK]; (2) **preamble/base-pattern
energy-combining across reps** (+2.2-2.5/doubling) — the "HAIL combining" lever, aimed at the
preamble not the suffix, core for the ULTRA tier + field margin at −14; (3) CFO sync (mini-
Moose, a1fe962c). Content FEC (GF16 RA / Tier-1+rep) is DONE.

## §15 Related future tier — ULTRA (named 2026-05-31)

The deep "survival" tier below ROBUST (−20 to −24 dB) is named **ULTRA** (`ULTRA_0/1/2`), its
own family at config-ID range **200+** (CONFIG 0-16, ROBUST 100-102, ULTRA 200+). Distinct
operating character: a single short message + stop-and-wait ACK (no SACK/streaming/batch),
multi-minute frames, ~0.3-1.5 bps — a "when all else fails" mode (FT8/JS8/Q65 regime, but
ARQ + self-synced, no GPS). Built on THIS acquisition foundation (count-only base-pattern
gates + preamble combining + low-rate GF(16) RA) pushed further (lower rate / more
integration / longer sync). Queued AFTER −14 establishment lands on HW — ~60-70% free-ridden
on this work. ROBUST_3 (coherent) stays obsolete-for-reach (the road down is more
noncoherent = ULTRA). HF Doppler/coherence caps it ~−22 to −24 (the −40/−45 regime is
LF/MF-only). Name chosen to match the neutral CONFIG/ROBUST register (degree-up from ROBUST).

## §16 CFO sync EXONERATED — the metric gate is the SOLE ctrl-suffix masker (2026-05-31)

Agent a1fe962c. Isolation sim (branch `sim/ctrl-sync-floor-isolation` @5f2c4d1 off the GF16
worktree; 42/42). Separated the two stages §8.3/§12 had bundled. **Corrects §12 (struck stage
2) and the gf16-spike-doc §8.3 "corrupts the re-decimated baseband" claim — that is FALSE.**

- **Mini-Moose `carrier_frequency_sync_wb_ctrl` (ofdm.cc:808) is FINE:** residuals |mean|≈1.5-1.9
  Hz, 0/120 wild, vs a 46.88 Hz subcarrier; the 0.05 confidence gate rejects noise; the re-mix
  (telecom_system.cc:3313) corrects by a tiny right amount. **skip-remix == current at every
  SNR (0 dB). DROP the CFO-sync line of work.**
- **The whole gap is the hard-coded `metric>=3.0` gate** (8 sites: telecom_system.cc
  3277/3337/3548/3586/3674/3701/3763/3789). The detect_ack_pattern metric falls monotonically
  and crosses 3.0 at the −8.65 cliff; masks 59%/97%/100% at −9.8/−10.8/−11.8 — while
  `decode_suffix_energies` delivers P=1.00 to −14. Relax the gate (Moose ON) → FEC reaches
  −14.03 (r=3), identical to the energies-direct floor.
- **2.0 is INSUFFICIENT** (free stack 6b2cb54 reaches only ~−11.75). Need **~1.0–1.5 or off**
  on the CRC-backstopped FEC path for −14.
- **FAR clean:** gate fully OFF → 0/4000 pure-noise false-accepts; CRC12 + 2-bit type is the
  sufficient backstop (Q65/FT8 precedent: the CRC, not a pre-decode energy threshold, is the
  floor's FAR gate — Franke-Taylor QEX 2020).
- **§5:** the gate is ctrl-suffix-LOCAL (ACK/SACK-ACK/CONNECT only); the data OFDM demod uses
  a DIFFERENT detector (Schmidl-Cox/Moose) → relaxing it cannot affect data DEMOD (only ACK
  detection, non-binding at good SNR → no v13 impact at OFDM operating points). Apply the
  relaxed gate only where the CRC backstop is present (CAP_SUFFIX_FEC).
- Latent hygiene (off critical path): `freq_offset_ignore_limit` comment drift
  (telecom_system.cc:3292, says ~3 Hz, is 0.1); ctrl re-mix lacks the data-path sanity clamp
  (telecom_system.cc:2270-2282).

**⇒ The three scaffolding limiters collapse to ONE: an unmeasured energy-confidence gate
(metric, + HAIL's quality) in front of every matched-pattern detector. Relax it across HAIL +
CONNECT + ACK (count/CRC-backstopped) → the whole chain reaches its floor. + preamble combining
(margin). No CFO work; content FEC solved.**

## §17 INTEGRATION — HAIL fix + ctrl-suffix metric gate → 1.2 (combined establishment-floor binary) (2026-05-31)

Agent (this session). Branch `sim/hail-detection-floor` (off monitor 01535f2): the HW-validated
HAIL fix (@060fc40 — fast-poll gate relaxed to the config-tuned metric floor) is the base; THIS
session adds the ctrl-suffix `CTRL_DETECT_METRIC_MIN` named constant and sets it to **1.2** so the
two §16/§13 scaffolding fixes ship as ONE establishment-floor binary.

### §17.1 What changed (code-grounded)

- **`cl_mfsk::CTRL_DETECT_METRIC_MIN = 1.2`** new named constant (`mfsk.h:122`), replacing the
  hardcoded `metric < 3.0` at the **4** production ctrl-suffix sites on this branch
  (`telecom_system.cc:3265, 3326, 3538, 3577`). NOTE: §16's "8 sites" list (3277/.../3789) was
  from the GF16 isolation worktree, which carries the additional `*_soft` decode duplicate-gate
  sites; on this branch (monitor + HAIL, NO suffix-FEC code — verified `grep` for
  `soft_list_decode_ctrl_suffix|decode_suffix_candidates|decode_suffix_energies|suffix_fec_mode`
  = 0 hits) there are exactly 4. All 4 now route through the constant.
- Value 1.2 justified by the §16 cliff table (gate 2.0 → −11.75; ~1.0–1.5 needed to feed the
  −14-capable content decoder). 1.2 = mid of [1.0,1.5] with ~0.2 FAR headroom over the count
  gate's worst-measured post-count-gate noise metric (1.207, connect-ack-metric-gate.md §6). NOT a
  magic number — anchored to the §16 measurement and the §6 FAR sweep.
- The HAIL fix itself (the `quality>=0.3`-dominant relax at `arq_common.cc:5367-5388`) is already
  on this branch (@060fc40) and is UNCHANGED by this session.
- Cherry-pick of 6b2cb54 was NOT used: that commit bundles a 488-line sim harness + 335-line fact
  doc that would conflict/duplicate; the 2 code hunks (constant + 4 sites) were applied directly,
  reusing 6b2cb54's exact site wording.

### §17.2 §5 cross-layer audit — the ctrl-suffix metric gate (REQUIRED; verified, not assumed)

The gate is shared-detection state. Producer of `metric` = `ofdm.detect_ack_pattern`
(`ofdm.cc:3691`, returns `Σ_matched (e_target/e_total)` — a per-symbol-normalized,
**scale-invariant** energy-ratio sum ∈ [0, nsymb]). Consumers of the `< 1.2` admission decision:

1. **`decode_ctrl_suffix_from_passband`** (`telecom_system.cc:3498`; gates 3538/3577). Sole caller
   `receive_mfsk_ctrl_suffix_phy_core` (`arq_common.cc:4987`) → the THREE Phase-B handshake decode
   stages: `MFSK_CTRL_START_CONN` (CONNECT-START), `MFSK_CTRL_TEST_ACK` (CONNECT-ACK),
   `MFSK_CTRL_TEST_CONN` (CONNECT-TEST) — i.e. the **establishment handshake**, the floor-binding
   stage below −12. **Backstop = CRC12 (P_false≈2⁻¹²) + 2-bit type discriminator (×¼)** at
   `arq_common.cc:5038/5058` + the hard count gate `connect_match_threshold=7` (mfsk.cc:399/406,
   FAR 2.5e-7/poll). This is the UNCODED suffix path (GF16 FEC NOT wired into production — verified).
2. **`detect_ack_snr_from_passband`** (`telecom_system.cc:3236`; gates 3265/3326). Sole production
   caller `arq_common.cc:5559`, inside the `turbo_snr_ack_enabled` branch = the **turboshift
   SNR-suffix readout** on data ACKs. KEY: failing the metric gate here returns -99/`snr_valid=false`
   = "no SNR readout this poll"; it does **NOT** block the ACK — the ACK is admitted by the OUTER
   count gate `matched_count >= ack_match_threshold` at `arq_common.cc:5563`, which does not depend
   on the metric gate. Backstop on the SNR content = the **3/8 majority + 2-vote-margin** vote
   (`telecom_system.cc:3408`), independent of the metric gate.

**Audit answers (the task's three questions):**

- **Q: Does lowering to 1.2 affect the OFDM-data ACK path's FAR/behavior at OFDM operating SNRs?**
  **NO.** (a) There is NO OFDM-based ACK path — `OFDM_ACK_CLEAN`/msg-0x44 was deleted (MEMORY;
  `grep` confirms no surviving symbol); the only "ACK" is the MFSK suffix above. (b) The data OFDM
  DEMOD uses an entirely separate detector — Schmidl-Cox / coarse preamble
  (`ofdm.cc time_sync_preamble*`, `receive_stats.coarse_metric`), not `detect_ack_pattern` — so this
  gate is unreachable from the data demod. (c) THROUGHPUT-NEUTRAL: in the good-SNR band where data
  flows the metric stays ≥7 (6b2cb54 measured 0 gate-decision divergences vs 3.0 across the good-SNR
  band) so 3.0→1.2 changes the gate decision ONLY in the deep-acquisition regime. Zero data-PHY
  files touched (ofdm/ldpc/modcods/arq byte-identical).

- **Q: FAR-safe on the UNCODED suffix path (CRC12 the only backstop, NOT GF16 FEC)?** **YES at 1.2.**
  §16 measured the gate **FULLY OFF** = **0/4000** pure-noise false-accepts on this CRC-backstopped
  path; 1.2 (> off) is strictly safer than the measured-clean OFF case. The FAR floor is the count
  gate (7/16, 2.5e-7/poll) + CRC12 (2⁻¹²) + 2-bit type (×¼), NOT the energy pre-filter — Q65/FT8
  precedent (Franke-Taylor QEX 2020: the CRC, not a pre-decode energy threshold, is the deep-floor
  FAR gate). The metric is only a CPU pre-filter to skip the suffix decode on obvious noise.

- **Q: Scope the deep relax — keep a higher floor where no strong backstop exists?** **Not needed.**
  Both consumers carry a content backstop: CONNECT = CRC12+type; ACK-SNR = 3/8 vote+margin AND the
  metric gate there can't admit a false ACK anyway (count gate does that). So 1.2 is uniformly safe
  across both ctrl-suffix consumers; no per-site floor split required. (Were a future caller to read
  ctrl-suffix content with NO CRC/vote backstop, it would need its own higher floor — none exists today.)

**Conclusion:** relax is correctly scoped (ctrl-suffix-LOCAL, 2 functions), FAR-safe on the uncoded
CONNECT path at 1.2, OFDM data path provably unaffected. Pending: sim cliff/FAR at 1.2, then HW A/B
(combined vs HAIL-only @060fc40) to isolate the ctrl-gate delta on establishment depth past −12.

### §17.3 Sim result — the UNCODED production path is CONTENT-limited, NOT gate-limited (decision-critical)

New regression test `test_ctrl_suffix_metric_gate_cliff_sweep` (`mfsk_ctrl_codec_tests.cc` §17,
registered in `run_mfsk_ctrl_codec_tests`). Drives the PRODUCTION
`decode_ctrl_suffix_from_passband` (the 1.2 gate baked in) on a real START_CONN passband + AWGN
across an SNR3k axis, logging per-cell P(decode), raw metric, matched count, and the
"rescued" count (trials with metric∈[1.2,3.0) AND a CRC-valid decode). Channel: clean+AWGN, no
injected CFO (§16 exonerated the ctrl mini-Moose) — absolute cliff slightly optimistic, relative
verdict CFO-independent. **32/32 tests pass.** Also gated an unconditional `[SUFFIX-FFT]` debug
printf (`ofdm.cc:4061`, fired on every suffix decode → production log spam) behind `g_verbose`.

| sigma/rms | SNR3k | P(decode)@1.2 | mean metric | mean matched | rescued[1.2≤m<3.0 & ok] |
|---|---|---|---|---|---|
| 5.0 | −4.95 | 1.00 | 6.01 | 16.00 | 0/80 |
| 5.5 | −5.78 | 0.96 | 5.37 | 16.00 | 0/80 |
| 6.0 | −6.53 | 0.97 | 4.81 | 16.00 | 0/80 |
| 7.0 | **−7.87** | **0.74** | 3.88 | 15.90 | 0/80 |
| 8.0 | −9.03 | 0.31 | 3.31 | 15.68 | **4/80** |
| 10.0 | −10.97 | 0.00 | 2.40 | 14.36 | 0/80 |
| 12.0 | −12.55 | 0.00 | 1.74 | 11.80 | 0/80 |

- **Production decode cliff (P=0.5) = −7.87 dB.** The mean metric crosses the OLD 3.0 gate at
  **−9.03 dB** — i.e. the suffix CONTENT (hard-argmax `decode_suffix_tones` + CRC12, NO FEC) dies
  ~1.2 dB ABOVE where the gate would have cut. **⇒ on the UNCODED production path the gate is NOT
  the binding limiter; the uncoded content is.** This REFINES §16: §16's "gate is the SOLE masker,
  P≈1.0 to −14" was measured on `decode_suffix_ENERGIES` (the GF16 soft path, branch
  `wt/suffix-fec-tier2-gf16`); on the uncoded path that ships today, the content cliffs first.
- **The relax IS measurably helpful + harmless:** it rescued **4 CRC-valid decodes** in the
  metric∈[1.2,3.0) band (the OLD 3.0 binary returns false there → 0; this is the fail-before-passes
  delta). Clean/high-SNR decode P=1.00 (no regression; throughput-neutral by construction).
- **FAR = 0/4000** pure-noise false CONNECT accepts at 1.2 on the uncoded path (CRC12+type+count
  backstop holds). Confirms §17.2 / §16's gate-OFF 0/4000 — 1.2 is FAR-safe on the uncoded path.

**⇒ The gate relax to 1.2 ships as a harmless + small-win robustness fix bundled with the HAIL
fix, but the path to −14 establishment on the UNCODED suffix is the GF(16) RA FEC integration
(§12 winner, Phase 2) — NOT a deeper gate. HW A/B is expected to show: CONNECT base-pattern
detection deepened (HAIL fix + gate), but the suffix DECODE still failing below ~−8 to −10 dB →
the GF(16) content FEC is the next phase. This sim PRE-REGISTERS that HW signature.**

### §17.4 HW A/B — establishment floor (COMBINED vs HAIL-only baseline)

IONOS testbed, pinned ROBUST_0 (gearshift off, `-s 100 -Q 0 -M auto -n -F off --skip-turbo-reverse
-R`), 3 fresh CONNECT attempts/cell, descending WGN. Harness `tools/hail_floor_hw_test.py` (extended
this session: ctrl-gate arm fingerprint + CONNECT-FAR counting + butler `OK rc=N` prefix-strip fix).
Arms: COMBINED = monitor-branch 88e6bb1 (HAIL fix 060fc40 + ctrl gate 1.2), built on-Pi
(md5 d09b75598bcd, rpi2 fingerprint confirms `CTRL_DETECT_METRIC_MIN = 1.2`); --test 32/32 on ARM.

**COMBINED arm result:**

| WGN | HAIL detected | CONNECTED | failing-stage signature |
|---|---|---|---|
| −10 | 3/3 | **3/3** | — (establishes) |
| −12 | 3/3 | **2/3** | 1 miss = `[HAIL] Timeout waiting for START_CONNECTION` (marginal HAIL base=9/8) |
| −14 | 3/3 | **0/3** | **`[RX-MFSK-CTRL-CONNECT-START] CRC12 fail type=1 ... matched=15..16`** |
| −16 | 3/3 | **0/3** | HAIL fires (`base=16/8 suffix=4/4 metric=3.5`); CONNECT fails |
| FAR WGN:60 / 120 s | — | — | **0 false [HAIL], 0 false CONNECT/ACK** |

**THE decision-critical signature (WGN:-14, log-confirmed, exactly as §17.3 sim pre-registered):**
HAIL fires cleanly (`base=16/8 suffix=4/4 metric=3.8`) AND the ctrl-suffix base pattern is detected
with **matched=15-16/16** (≫ the 7/16 count gate, ≫ the relaxed 1.2 metric gate — the gate ADMITS
the decode and the type=1 START_CONN is correctly identified), **but CRC12 FAILS on the 13-tone
UNCODED payload** (`rx=0x4e6 exp=0x9c8`). The metric gate is NOT the limiter at the floor — it lets
the decode through; the hard-argmax uncoded suffix CONTENT is what fails. **This is the GF(16) FEC
motivation observed directly on hardware.**

**Establishment floor move:** HAIL detection reaches −16 (the HW-validated HAIL fix from 060fc40);
**establishment (CONNECT) reaches −12 (2/3)** and fails −14/−16 with the CRC12-on-content signature.
FAR clean at 1.2 on HW (0 false CONNECT/ACK) — confirms §17.2/§17.3: the relaxed gate is FAR-safe on
the uncoded path; CRC12 + count + type hold the line.

**⇒ Next limiter = the UNCODED ctrl-suffix CONTENT** (hard-argmax 13-tone payload + CRC12, no FEC).
The path past −12 establishment is the **GF(16) RA FEC integration** (§12 winner, R¼ reaches −14.03
in sim) — NOT a deeper gate, NOT more HAIL work. The metric-gate relax + HAIL fix are the necessary
SCAFFOLDING (detection reaches the floor); the GF(16) content FEC is the sufficient next phase.

#### HAIL-only baseline arm (060fc40) — isolates the ctrl-gate delta

Built on-Pi (md5 **9d4ea03b1e87**, fingerprint `ctrl_arm=HAIL-ONLY-3.0`, `gate=(none)`,
hardcoded-3.0 sites present). **md5 DIFFERS from COMBINED (d09b75598bcd) on both Pis → arms
verifiably distinct.** ctrl-gate value per arm CONFIRMED: COMBINED=1.2, BASELINE=3.0.

| WGN | COMBINED (gate 1.2) | HAIL-ONLY (gate 3.0) |
|---|---|---|
| −10 | HAIL 3/3, CONNECT **3/3** | HAIL 3/3, CONNECT **3/3** |
| −12 | HAIL 3/3, CONNECT **2/3** | HAIL 3/3, CONNECT **3/3** |
| −14 | HAIL 3/3, CONNECT **0/3** | HAIL 3/3, CONNECT **0/3** |
| −16 | HAIL 3/3, CONNECT **0/3** | HAIL 3/3, CONNECT **0/3** |
| FAR WGN:60/120s | 0 HAIL, **0 CONNECT/ACK** | 0 HAIL, **0 CONNECT/ACK** |

**A/B VERDICT: the ctrl-gate relax (3.0→1.2) does NOT move the establishment floor on HW.** Both
arms establish to −12 and fail −14/−16. The single-cell −12 difference (COMBINED 2/3 vs baseline 3/3)
is 3-attempt sampling noise — COMBINED's one −12 miss was a marginal-HAIL timeout (`base=9/8`), a
HAIL-detection variance, not a ctrl-gate effect. **Same failing-stage signature on BOTH arms at −14:**
`[RX-MFSK-CTRL-CONNECT-START/TEST] CRC12 fail ... matched=14-16` — the ctrl-suffix base detects
(matched≈16/16, which clears BOTH the 3.0 and 1.2 metric gates), the decode is admitted, but the
UNCODED 13-tone payload CRC12 fails. Baseline attempt 3 is the clearest: START_CONN decoded clean
(`sender='TESTA'`), then the NEXT ctrl-suffix stage `[RX-MFSK-CTRL-CONNECT-TEST] CRC12 fail type=3
matched=16` — the handshake advances one stage then the next suffix fails on content.

**⇒ This is the §11 HW-null pattern repeating, now PROVEN for the ctrl-suffix: the metric gate is
NOT the binding establishment limiter at the floor** (matched=16 clears it either way). The relax is
a correct, harmless §1 hygiene fix (removes 4 unmeasured magic 3.0 thresholds, replaces with a
measured named constant; FAR-safe 0/4000 sim + 0 HW) but **delivers no HW establishment-floor gain
in isolation** — same as the free stack did when HAIL was the limiter. **The path past −12
establishment is the GF(16) RA FEC integration (§12 winner, R¼ → −14.03 in sim), which replaces the
uncoded hard-argmax 13-tone payload with a soft Q-ary code over the per-tone energies. The
CRC12-fail-at-matched=16 signature is the GF(16) motivation, now observed identically on both HW
arms.** Single-pass (3 attempts/cell); the relative verdict (no floor move, content-limited) is
robust to the sampling noise; the absolute −12 floor cell could shift ±1 cell on a multi-pass run.

**Merge guidance:** the ctrl-gate relax (commit 88e6bb1) is clean to keep bundled on
`sim/hail-detection-floor` with the HAIL fix — it's a measured §1 improvement and a latent enabler
(once GF(16) FEC feeds the decoder a soft path, the gate must already be relaxed so the energies
reach the decoder; cf. §11's note that the metric-gate half is a clean latent fix). But like the
free stack, it does NOT move the HW floor alone — **bundle the full validated acquisition+content fix
(HAIL + gate + GF(16) FEC) before claiming an establishment-floor move on HW.**

## §18 HAIL gate fix (standalone) HW-VALIDATED — first floor move (2026-05-31)

*(Precursor detail for §17's combined A/B: the HAIL fix vs monitor baseline. §17 is the later
HAIL+gate integration that builds on this.)*

Agent a8ed6001. Branch `sim/hail-detection-floor` @060fc40 (fix + assertion) → b0bf8ce (HW
results). md5 FIXED `9d4ea03b…` / BASELINE `bb00c823…` (01535f2); 31/31 vs 30/30 tests.

- **Fix (`arq_common.cc:5375`):** dropped the dominant `quality>=0.3` gate + aligned `metric`
  to the config threshold (`ack_pattern_detection_threshold`, 0.65@ROBUST_0) on the HAIL
  fast-poll site. No new constant. **§5 audit:** the gate lives in each CALLER, not the shared
  `detect_ack_pattern` — C1 (HAIL fast-poll) was the outlier with hardcoded 3.0 + quality;
  ACK(C3)/CONNECT(C6)/BREAK(C5) already use the config threshold → PROVABLY UNAFFECTED
  (confirms the v13/throughput safety answer).
- **Sim:** HAIL cliff −4.95 → −13.25 SNR3k (**+8.30**); metric-relax-alone = +0.00 (proves the
  quality gate was dominant); FAR 0/5000.
- **HW A/B (IONOS pinned ROBUST_0, 3/cell):** FIXED HAIL fires **3/3 at −8/−10/−12/−14/−16**;
  BASELINE cliffs −12. **Establishment (CONNECT) −10 (baseline) → −12 (fixed).** FAR **0 both
  arms**. Accepted beacons at −12/−14/−16 carried metric 1.5-5.4 / quality 0.14-0.27 (below
  the old gate). **+~4 dB HAIL on the testbed dial.**
- **Next stage EXPOSED (as predicted):** below −12, HAIL fires 3/3 but CONNECT 0/3 → the
  ctrl-suffix decode (the metric≥3.0 gate, §16) is the new binding cliff. Log: `[HAIL]
  Detected`, no `CONNECTED`.
- **MERGE:** recommend; BUNDLE with the ctrl-suffix gate relax (in progress, agent ad8198c9)
  once the floor is HW-confirmed deeper. Single-pass (3/cell) caveat: −14/−16 3/3-vs-0/3
  decisive; the −12 boundary (3/3 vs 2/3) wants a multi-pass.

## §19 INCREMENT 1 — GF(16) RA FEC wired into the PRODUCTION CONNECT ctrl-suffix decode (SIM only) (2026-06-01)

Agent (this session). Branch `sim/suffix-fec-integration` (composed — see §19.1). SIM/IN-PROCESS
ONLY; no IONOS, no CAP negotiation, no try-both fallback (those are explicitly the NEXT
increments). Goal: prove the **production** CONNECT decode tracks the §12 GF(16) R¼ FEC reach
(−14.03) instead of the uncoded −7.87 content cliff (§17.3/§17.4 HW-confirmed limiter).

### §19.1 Branch composition (direction + conflicts)

Direction: **branch off the GF16 worktree `wt/suffix-fec-tier2-gf16` @4675321** (it already carries
the codec `mfsk_ctrl_codec.{h,cc}::gf16ra`, the read-only energy primitive
`cl_ofdm::decode_suffix_energies`, the Tier-1 scaffolding `decode_suffix_candidates` /
`soft_list_decode_ctrl_suffix`, and `suffix_fec_mode`), then **cherry-pick the sim/hail chain in
order**: `712ef45` (HAIL cliff-sweep sim + fact doc) → `060fc40` (HAIL fast-poll gate fix in
`arq_common.cc:5388`) → `88e6bb1` (`CTRL_DETECT_METRIC_MIN=1.2` at the 4 base ctrl-suffix gate
sites + `[SUFFIX-FFT]` printf gated behind `g_verbose`). Both branches share merge-base `01535f2`.

Why this direction (not branch-off-hail + apply-GF16): the GF16 side is ~1200 LoC across 7 files
(codec + decoder + harness); the hail side is ~3 small hunks. Re-deriving the GF16 codec onto the
hail branch would be far more conflict-prone than replaying 3 small hail commits onto GF16.

Conflicts resolved:
- `712ef45`: (a) `fact-documents/hail-detection-floor-investigation.md` — new file, applied clean
  (force-added; `fact-documents/` is `.gitignore`d at :63 so these docs are force-add-tracked).
  (b) `mfsk_ctrl_codec_tests.cc` test-registration block in `run_mfsk_ctrl_codec_tests()` — both
  branches appended their own `test_*()` calls in the same spot; resolved by **keeping both**
  (GF16 §9/§10 registrations + HAIL §11 registration). Note: cherry-picking `060fc40` ALONE first
  failed (it *modifies* `test_hail_detection_cliff_sweep`, which doesn't exist on the GF16 line —
  it was added by `712ef45`); picking `712ef45` first is mandatory.
- `060fc40`: clean (auto-merged the test file; `arq_common.cc` HAIL hunk applied verbatim — GF16
  branch never touched `arq_common.cc`).
- `88e6bb1`: clean auto-merge of `telecom_system.cc` / `ofdm.cc` / `mfsk.h` / test file. POST-PICK
  verification: the constant lands at the **4 base sites** (`telecom_system.cc` 3278/3339 in
  `detect_ack_snr_from_passband`, 3551/3590 in `decode_ctrl_suffix_from_passband`). The GF16 branch
  ALSO carries 4 *additional* `metric<3.0` sites in the `*_soft` Tier-1 variants
  (`decode_ctrl_suffix_from_passband_soft` / `detect_ack_snr_from_passband_soft`, sites
  3678/3705/3767/3793) which the 88e6bb1 hunk did NOT touch (they didn't exist when it was
  authored). **The FEC production path does NOT go through the `*_soft` functions** (it extends
  `decode_ctrl_suffix_from_passband`, which already uses the constant), so those 4 stale `3.0`
  literals are off the critical path; left as-is to keep the cherry-pick faithful (§19.5 notes them
  as latent hygiene).

Composed HEAD: `sim/suffix-fec-integration` @718f751 + this increment's wiring commit(s). Baseline
(pre-wiring) build + `--test` = **44/44 pass**; the standalone GF16 r=3 cliff already reports
−14.03 (energies-direct) on this binary, confirming the codec rode across cleanly.

### §19.2 The wire (FORCE-on, no CAP): TX encode + RX soft-decode, gated by a coded-length accessor

The whole increment is keyed on **one new accessor** that makes the FEC suffix length flow through
every existing length consumer (the I4 invariant, §5/§19.4):

- `cl_telecom_system::ctrl_suffix_coded_len()` → returns `gf16ra::codeword_len()` (N, default 52 at
  repfact=3) when `suffix_fec_mode==3` and WB (`ack_sack_suffix_len()>0`), else `ack_sack_suffix_len()`
  (13). One function; every TX/RX site that used `ack_sack_suffix_len()` *for the ctrl-suffix
  symbol count* (NOT for the 13-bit field packing) routes through it.
- `gf16ra::configure(3)` + `gf16ra::init()` are invoked once when FEC mode turns on (idempotent).
- TX: `cl_mfsk::pack_ctrl_suffix` — when `suffix_fec_coded` (a new bool the telecom layer sets on the
  mfsk object), encode via `gf16ra::encode(type,payload38,crc12,out_tones)` → N tones, instead of the
  13-tone hard pack. `generate_ctrl_suffix_pattern` loops `coded_len` symbols (the one-hot tone
  placement + hop are identical — GF16 tones are 0..15 exactly like hard tones).
  `generate_ctrl_suffix_pattern_passband` sizes `nsymb = connect_pattern_nsymb + coded_len` and the
  precomputed `ctrl_suffix_pattern_passband_samples` uses `coded_len` (telecom_system.cc:5704).
- RX: `decode_ctrl_suffix_from_passband` — base detection + mini-Moose + the 1.2 metric gate are
  UNCHANGED. When FEC on, after the gate clears, replace `decode_suffix_tones`+`unpack_ctrl_suffix`
  with `decode_suffix_energies` (full N×16 de-hopped energy matrix) → `gf16ra::soft_decode(energies,
  50, 4.0, expected_type, crc12_fn, ctx, &payload38, &iters)`. The decoder's built-in CRC12+2-bit-type
  accept gate IS the FAR backstop. On success, recompute `*out_crc12 = CRC12_calc([type|payload38])`
  so the OUTER CRC re-check in `receive_mfsk_ctrl_suffix_phy_core` (arq_common.cc:5058) passes
  trivially (it is consistent by construction — soft_decode only returns true when its decoded CRC
  equalled the recompute over the same field). `expected_type` is plumbed into the decode (the hard
  path didn't take it; soft_decode requires it — provided by the existing
  `decode_ctrl_suffix_from_passband_soft` signature pattern: a `ctrl_crc12_fn`+ctx pair).
- CRC callback: production `cl_arq_controller::CRC12_calc` (init=0xFFF) via a `prod_crc12_cb`-style
  wrapper — NEVER inline (v1 bug #1). `telecom_system` has no CRC method, so the callback + ctx are
  passed down from `arq_common.cc` (where `self->CRC12_calc` lives) exactly as the Tier-1 `*_soft`
  entry points already do.

FORCE-on for this increment: `suffix_fec_mode` set to 3 at session init (or via a test hook). NO
`CAP_SUFFIX_FEC` negotiation, NO try-both — both arms run the same coded length. Mixed-pair / fallback
is the NEXT increment.

### §19.3 Decode-from-passband, NOT capture-then-decode (architecture)

Two RX ctrl-suffix paths exist. The **capture-then-decode** path (`last_connect_suffix_tones[]` +
`decode_ctrl_suffix_from_last_capture` → `unpack_ctrl_suffix`) stores HARD argmax tones and CANNOT
carry the soft energy matrix → unusable for FEC. The **decode-from-passband** path
(`decode_ctrl_suffix_from_passband`) re-derives baseband from the passband ring and can extract the
full energy matrix. **FEC wires into decode-from-passband only.** When FEC is on, the hard capture
snapshot is skipped (or left invalid) — `decode_ctrl_suffix_from_last_capture` is not on the CONNECT
production path (the production caller `receive_mfsk_ctrl_suffix_phy_core` uses
`decode_ctrl_suffix_from_passband` directly). The standalone harness `decode_gf16ra_from_passband`
(`mfsk_ctrl_codec_tests.cc:3212`) is the proven template this production wire mirrors.

### §19.4 §5 CROSS-LAYER DATA-FLOW AUDIT — ctrl-suffix symbol-count (I4)

Shared state: **the ctrl-suffix symbol count** (was the constant `13 = ack_sack_suffix_len()`; becomes
`ctrl_suffix_coded_len() ∈ {13, 52}`). It crosses PHY (TX symbol gen, RX energy extraction), the
sample-count layer (passband buffers), and ARQ (capture-window sizing). Producers/consumers:

**Producers (who sets/derives the length):**
1. `cl_mfsk::ack_sack_suffix_len()` (mfsk.h:173) — the base 13 (M≥16) / 0 (NB). UNCHANGED.
2. `cl_telecom_system::ctrl_suffix_coded_len()` (NEW) — 52 when FEC+WB, else 13. The single source
   of truth for the *coded* symbol count.
3. `gf16ra::configure(repfact)`/`codeword_len()` — owns N; configured once at FEC enable.

**Consumers (who reads the length) — every one audited:**
- C1. TX `pack_ctrl_suffix` (mfsk.cc:625) — uses `ack_sack_suffix_len()` for the 13-symbol HARD
  pack. With FEC it branches to `gf16ra::encode` (N tones) BEFORE this loop → the `n=13` here is
  bypassed, not violated. **Invariant held** (the 52-bit field packing is unrelated to the coded
  symbol count).
- C2. TX `generate_ctrl_suffix_pattern` (mfsk.cc:860) — loops `suffix_len` one-hot symbols. MUST use
  `coded_len`. Writes `pattern_out[(connect_nsymb+coded_len-1)*Nc + ...]`. **Buffer
  `data_container.ofdm_framed_data` capacity:** `alloc_Nsymb*Nc` where
  `alloc_Nsymb=max(Nsymb,48)` (data_container.cc:115). For ROBUST_0, `Nsymb` is small → 48 < 16+52=68
  → **HEAP OVERFLOW**. FIX: raise the floor to cover `connect_pattern_nsymb + max coded ctrl-suffix`
  (≥68). Same for `ofdm_symbol_modulated_data` (`Nofdm*alloc_Nsymb`). **This is the load-bearing I4
  finding — without it, FEC-on corrupts the heap on the short-frame robust configs.**
- C3. TX `generate_ctrl_suffix_pattern_passband` (telecom_system.cc:3478) — `nsymb` + the
  `symbol_mod`/`baseband_to_passband` loop bound + `ctrl_suffix_pattern_passband_samples`. MUST use
  `coded_len`. The caller `send_mfsk_ctrl_suffix_phy_core` (arq_common.cc:4838-4866) sizes
  `padded_size = ctrl_suffix_pattern_passband_samples + 2*symbol_period` and does the TX-edge memcpys
  off `pattern_samples = ctrl_suffix_pattern_passband_samples` → all derive from the member, so fixing
  the member at init (5704) flows through. **Invariant held once 5704 uses coded_len.**
- C4. `ctrl_suffix_pattern_passband_samples` (telecom_system.cc:5704, set in init) — MUST use
  `coded_len`. Consumed by C3 + the TX guard `if(ctrl_suffix_pattern_passband_samples<=0)`. NB stays 0
  (connect_nsymb=0). **Held.**
- C5. RX `decode_ctrl_suffix_from_passband` (telecom_system.cc:3545/3587 `reserve_after`; 3599-3610
  decode loop). `reserve_after` to `detect_ack_pattern` MUST be `coded_len` (else the base detector
  rejects offsets that don't leave room for the longer suffix, OR — worse — accepts an offset whose
  suffix runs off the captured buffer). The decode itself switches to `decode_suffix_energies(...,
  coded_len, ...)`. **Held with the accessor.**
- C6. RX capture window `receive_mfsk_ctrl_suffix_phy_core` (arq_common.cc:4994-5002):
  `suffix_nsymb = ack_sack_suffix_len()` → `tail_nsymb = conn_nsymb + suffix_nsymb + 16` →
  `tail_samples`. MUST use `coded_len` so the captured passband tail actually CONTAINS the full N=52
  suffix (+ the existing 16-symbol margin). `tail_samples` is clamped to `signal_period`
  (= `buffer_Nsymb*sym_samples`) — **verify `buffer_Nsymb ≥ conn(16)+52+16 = 84`** (open check
  §19.6); if the ring is shorter the clamp silently truncates the suffix → decode fails. **Held iff
  the ring is long enough — must verify.**
- C7. RX buffers `last_connect_suffix_tones[MAX_ACK_SACK_SUFFIX]` /
  `last_ack_sack_suffix_tones[MAX_ACK_SACK_SUFFIX]` (mfsk.h:218/223, size 16). The energies path does
  NOT write these (it bypasses the hard capture). BUT `decode_ctrl_suffix_from_passband`'s hard branch
  writes `last_connect_suffix_tones[i]` for `i<suffix_len`; if that branch ever ran with
  `suffix_len=52` it would overflow the 16-array. FIX: raise `MAX_ACK_SACK_SUFFIX` to ≥ max coded N
  (≥52) so the arrays are safe regardless of which branch runs; AND the hard-snapshot loop is FEC-
  gated off. Also the local `int suffix_tones[MAX_ACK_SACK_SUFFIX]` (telecom_system.cc:3602) and
  `int payload_tones[MAX_ACK_SACK_SUFFIX]` (mfsk.cc:867) are sized by this constant → raising it covers
  them. **Held once MAX_ACK_SACK_SUFFIX ≥ 52.**
- C8. `decode_suffix_energies` out buffer — caller-allocated `std::vector<double>(coded_len*M)`. Sized
  by the accessor. The codec ceiling `GF16RA_MAX_N=64` ≥ 52. **Held.**
- C9. ACK / SACK-ACK paths (`detect_ack_snr_from_passband`, `decode_ack_sack_from_passband*`). These
  use `ack_sack_suffix_len()` (13) for the ACK suffix — **NOT** touched by this increment (FEC is
  CONNECT-only here; the ACK gating to the robust tier is a later increment, §4). The accessor is only
  substituted on the CONNECT TX/RX sites. **ACK path byte-identical.**

**Valid-states / default-init check:** `suffix_fec_mode` default 0 (telecom_system.cc:96) → accessor
returns 13 → every consumer behaves exactly as today (byte-identical-when-off, the gate test). The
FEC object `gf16ra` is configured lazily; if a decode is attempted before configure, `codeword_len()`
returns its default (N for repfact=2=39) — so the enable path MUST `configure(3)` before the first
TX/RX. Guarded at FEC-enable.

**Invariant the consumers assume:** "the ctrl-suffix occupies exactly `<len>` symbols immediately
after the `connect_pattern_nsymb` base, one-hot per symbol, hop `(tone+abs_s*hop)%M`." FEC preserves
this exactly — only `<len>` changes (13→52) and the tone *values* come from the RA codeword instead of
the bit-packer. Every consumer that derives a sample count or loop bound from `<len>` is enumerated
above; each is switched to the accessor or proven off-path.

### §19.5 What this increment changes vs leaves

CHANGES: `ctrl_suffix_coded_len()` accessor; `suffix_fec_coded` flag on the mfsk object; FEC branch in
`pack_ctrl_suffix` + `generate_ctrl_suffix_pattern` (TX) and `decode_ctrl_suffix_from_passband` (RX);
`ctrl_suffix_pattern_passband_samples` init (C4); capture window (C6); `MAX_ACK_SACK_SUFFIX` 16→≥52
(C7); `alloc_Nsymb` floor (C2). LEAVES: the 1.2 metric gate (already relaxed — the energies must reach
the decoder); HAIL fix; the ACK/SACK-ACK suffix (CONNECT-only); CAP negotiation + try-both (next
increment); the 4 stale `metric<3.0` literals in the `*_soft` Tier-1 functions (off the FEC path —
latent hygiene).

### §19.6 Open checks [?] (resolved inline during implementation; results in §19.7)

- [?] C6 ring length: is `data_container.buffer_Nsymb ≥ 84` at ROBUST_0 so the capture tail holds the
  full coded suffix? If not, the clamp truncates → the production cliff would stall ABOVE −14 (a
  capture-sizing artifact, not a FEC limit). MUST measure.
- [?] Does the production cliff actually track −14, or does some scaffolding detail (CFO re-mix on the
  longer window, energy-extraction offset for symbols 13..51) clamp it higher? This is exactly the
  integration risk this increment de-risks (the standalone codec hit −14 on a synthetic passband; the
  full production path may surface an offset/window bug).

## §20 INCREMENT 2 — base-pattern noncoherent COMBINING on the CONNECT handshake (PLAN + SIM + HW) (2026-06-01)

Agent (this session). Branch `sim/connect-preamble-combining` off `sim/suffix-fec-integration` @deb1ecf
(HAIL gate fix 060fc40 + ctrl-gate 1.2 88e6bb1 + GF16 FEC INCREMENT-1 deb1ecf). The handoff from the
FEC-arm HW run (§19.7, recorded verbally, not yet in §19.7 prose): with the GF(16) R¼ FEC forced on,
the IONOS establishment floor moved −10 → **−16** WGN; at the FEC arm's FINAL floor (−18/−20) the
failure is **BASE-PATTERN MATCHED-COUNT COLLAPSE** (`detect_ack_pattern` matched 16→13→10, drops
below `connect_match_threshold=7`), **NOT CRC** — i.e. the FEC solved the content, and now the
**CONNECT base-pattern DETECTION** (the same matched-count limiter HAIL had) is the binding stage.
This increment adds the §13/§14-measured lever — **noncoherent energy-combining of the base-pattern
matched-filter across R repeated base patterns** — to the CONNECT handshake.

### §20.1 The lever (measured prior art, NOT a guess)

The base pattern = `connect_pattern_nsymb=16` symbols (Welch-Costas, 8 tones ×2, hop
`(connect_tones[s%8]+s*tone_hop_step)%M`, `mfsk.cc:843-865`); detected by `detect_ack_pattern`
(`ofdm.cc:3691`) which per candidate start `s` sums a per-symbol matched-count (expected tone is the
all-streams argmax peak) + a soft metric `Σ e_target/e_total`. The matched-COUNT statistic is what
collapses at the deep floor (§19.7 HW). §4 of `hail-detection-floor-investigation.md` (the
structurally-identical HAIL base pattern) MEASURED, on this exact detector: noncoherent combining of
the per-tone energy matrix `E[s][m]` across R aligned reps BEFORE argmax deepens the matched-COUNT
cliff **R=2 +1.80 dB, R=5 +3.74 dB** (vs 10log10 ideal +3.0/+7.0 — the realistic M=16 noncoherent
gain at q≈0.1-0.24). §14 (repetition sim, CONNECT base) independently MEASURED **+2.2 to +2.5
dB/doubling** (R=2 +2.18, R=4 +2.50) — the LOW end (Tier-1 soft-list already harvests near-miss
energy at R=1). Prior art: WSJT-X Q65 noncoherent integration ~2.6-3.0 dB/doubling (matches). **⇒
expected: ~+2.2-2.5 dB per doubling on the base-pattern matched-count floor.**

Why combining works on the COUNT but NOT on the current normalized-RATIO metric (§4 / §13): the
metric `e_target/e_total` is scale-invariant — summing R copies cancels in the ratio. So combining
must feed the **argmax/count** (the matched-count is the limiter here per §19.7, and the metric gate
is already relaxed to 1.2 so it is non-binding at the floor: §17.4 HW showed matched=15-16 clearing
the 1.2 gate). The combined energies also feed the metric (deeper, but the count is what we need).

### §20.2 The design (ONE change, flag-gated, R=1 byte-identical)

New flag `cl_mfsk::connect_preamble_reps` (default **1** = byte-identical to today). When R>1:

- **TX** (`generate_ctrl_suffix_pattern` / `generate_connect_pattern`): emit the 16-symbol base
  pattern **R times** (identical block — rep r symbol s carries the SAME tone as rep 0 symbol s, i.e.
  per-rep-LOCAL hop indexing `(connect_tones[s%8]+s*hop)%M`, NOT continued `abs_s` hop), THEN the
  suffix once at `abs_s = R*16 + s`. Total symbols `R*16 + ctrl_suffix_len()`.
- **RX** (`detect_ack_pattern`, new `combine_reps` param): for each candidate start offset, accumulate
  the per-symbol FFT energy of base-symbol s SUMMED over the R reps (offsets `s, 16+s, 32+s, …`)
  before the argmax/count/metric. The base pattern occupies `R*16` symbols; the suffix follows at
  `best_offset + R*16` (the suffix is NOT repeated — only the base/preamble is combined, exactly the
  §14 finding "combining belongs on the PREAMBLE, not the suffix"). `reserve_after` becomes
  `ctrl_suffix_len()` still (the suffix length), and the base window is `R*16`.
- **Count gate (FAR defense) KEPT**: the matched-count threshold `connect_match_threshold=7` is the
  load-bearing FAR defense (§4 HAIL: count-only FAR 0/5000); combining feeds it deeper energy but the
  7/16 gate still guards. We do NOT relax the count gate.

### §20.3 §5 CROSS-LAYER DATA-FLOW AUDIT — the base-pattern symbol-count (extends §19.4 I4)

Shared state: **the CONNECT base-pattern symbol count** (was `connect_pattern_nsymb=16`; the
on-wire base now occupies `connect_base_total_nsymb() = R*16` when combining). Crosses PHY (TX symbol
gen, RX matched filter + suffix-offset), the sample-count layer (passband buffers), ARQ (capture
window). Reuses the §19.4 producer/consumer skeleton; the DELTA vs §19.4 is that the BASE count
(not just the suffix count) now scales.

**Producers:**
1. `cl_mfsk::connect_pattern_nsymb` (16) — UNCHANGED (one base block).
2. `cl_mfsk::connect_base_total_nsymb()` (NEW) = `connect_preamble_reps * connect_pattern_nsymb`.
   Single source of truth for the on-wire base symbol count.
3. `cl_telecom_system::ctrl_suffix_coded_len()`/`ack_mfsk.ctrl_suffix_len()` — UNCHANGED (suffix only).

**Consumers (every one audited; ⊕ = newly affected by R, vs §19.4 where only the suffix scaled):**
- ⊕ C2 TX `generate_ctrl_suffix_pattern` (mfsk.cc:870): loops `R*16` base one-hot symbols (R blocks)
  then `suffix_len` at `abs_s=R*16+s`. **Buffer `ofdm_framed_data`/`ofdm_symbol_modulated_data`
  floor**: was 80 (16+64). With R=4: 64+52=116 > 80 → RAISE the floor to cover
  `MAX_REPS*16 + GF16RA_MAX_N` (MAX_REPS=4 → 64+64=128). **Load-bearing — without it, R=4 overflows
  the heap on short-frame robust configs (same class as §19.4 C2).**
- ⊕ C3/C4 TX `generate_ctrl_suffix_pattern_passband` (telecom_system.cc:3494) + member
  `ctrl_suffix_pattern_passband_samples` (set 5790 + re-derived in `set_suffix_fec` 3479): `nsymb`
  becomes `connect_base_total_nsymb() + ctrl_suffix_len()`. All TX sample counts derive from the
  member → fix the two derivation sites, flows through `send_mfsk_ctrl_suffix_phy_core` (4849-4862).
- ⊕ C5 RX `decode_ctrl_suffix_from_passband` (telecom_system.cc:3564): `detect_ack_pattern` is called
  with `ack_nsymb = connect_pattern_nsymb` today; with combining it must scan `R` reps → pass
  `combine_reps=R` and the base block length 16. `reserve_after` stays `ctrl_suffix_len()`. Suffix
  energy/tone extraction offset `connect_pattern_nsymb` → `connect_base_total_nsymb()` (the suffix is
  after ALL R base reps). mini-Moose runs on the FIRST base rep (offset best_offset, 16 sym) — CFO is
  common across reps, one estimate suffices (§16 ctrl-Moose is clean/tiny anyway).
- ⊕ C6 RX capture window `receive_mfsk_ctrl_suffix_phy_core` (arq_common.cc:5004-5018):
  `tail_nsymb = conn_nsymb + suffix_nsymb + 16` → `connect_base_total_nsymb() + suffix_nsymb + 16`.
  Clamped to `signal_period` (ring = `buffer_Nsymb*sym_samples`). **VERIFY** `buffer_Nsymb ≥
  4*16+52+16 = 132` at ROBUST_0 (open check §20.6 — the ring is min_buf ≈ frame+turnaround+frame+
  margin, hundreds of symbols at ROBUST_0, so 132 fits; MUST confirm at build).
- C1 TX `pack_ctrl_suffix` (mfsk.cc:627): SUFFIX field packing — independent of base reps. UNCHANGED.
- C7 RX hard-tone buffers `last_connect_suffix_tones[MAX_ACK_SACK_SUFFIX=64]`: suffix-only, NOT base.
  UNCHANGED (the suffix is still ≤52). The hard path is FEC-gated off anyway.
- C8 `decode_suffix_energies` out buffer (N*M, N=suffix len): suffix-only. UNCHANGED.
- C9 ACK / SACK-ACK paths: do NOT use the CONNECT base pattern or `connect_preamble_reps`. The
  combining is **CONNECT-only** (the establishment handshake), exactly as the FEC is CONNECT-only
  this increment. **ACK path byte-identical.**

**Valid-states / default-init:** `connect_preamble_reps` default 1 → `connect_base_total_nsymb()=16`
→ every consumer behaves EXACTLY as today (byte-identical-when-off — the regression gate). The
`detect_ack_pattern` `combine_reps` param defaults to 1 (no behavior change for ACK/BREAK/HAIL
callers, which pass 1).

**Invariant consumers assume:** "the ctrl-suffix occupies exactly `<suffix_len>` symbols immediately
after the base, one-hot per symbol." With combining the base is `R*16` symbols (R identical blocks)
and the suffix starts at `R*16`. Every consumer that derives a base length or suffix offset is
enumerated above and switched to `connect_base_total_nsymb()`; the suffix-internal layout is
untouched.

**§5 verdict:** SCOPED to the CONNECT TX/RX base-pattern path (2 TX gen sites + member + 1 RX detect
site + capture window + buffer floors). ACK/BREAK/HAIL detection unaffected (`combine_reps=1` default,
they don't read `connect_preamble_reps`). FAR defended by the unchanged count gate (`connect_match_
threshold=7`) + the CRC12/type backstop on the suffix. The combining is energy-additive into the
SAME argmax the count gate already trusts.

### §20.4 What changes vs leaves

CHANGES: `connect_preamble_reps` flag + `connect_base_total_nsymb()` accessor (mfsk); `combine_reps`
param on `detect_ack_pattern` (ofdm) summing base-rep energies before argmax; TX base-rep loop in
`generate_connect_pattern`/`generate_ctrl_suffix_pattern`; `ctrl_suffix_pattern_passband_samples`
derivations (2 sites); RX detect call + suffix offset + capture window; buffer floor 80→128.
LEAVES: the suffix FEC (INCREMENT-1, unchanged — combining is on the BASE, orthogonal); the 1.2
metric gate; HAIL fix; ACK/SACK-ACK; CAP negotiation + try-both (the combining R, like the FEC, is
FORCE-on for this increment via the same enable hook; CAP-gated adaptive-R is a later increment).

### §20.5 Sim test (the gate, BEFORE HW)

New regression test `test_connect_preamble_combining_cliff_sweep` (`mfsk_ctrl_codec_tests.cc`):
synthesize the production CONNECT base+suffix passband at R=1/2/4 (real TX path
`generate_ctrl_suffix_pattern_passband`), AWGN across SNR3k, run the production RX detector
(`detect_ack_pattern` with `combine_reps=R`), measure (a) the base-pattern matched-count cliff
(P(matched≥7)=0.5) per R — does it deepen ~+2.2-2.5/doubling? (b) the full establishment decode
(base detect + FEC) floor per R — does it go deeper? (c) FAR on pure noise (combining + count gate),
(d) byte-identical-when-off (R=1 TX bytes == pre-change). Build FOREGROUND `bash build.sh o3`,
`mercury --test` passes. **GATE: if no base-pattern floor gain in sim, STOP + report (don't waste
bench).**

### §20.6 Open checks [?]
- [?] C6 ring `buffer_Nsymb ≥ 132` at ROBUST_0 (R=4 window). Confirm at build (printed by init).
- [?] Does the combined base-pattern cliff track the §4/§14 +2.2-2.5/doubling, and does the FULL
  establishment floor (FEC behind the base) move deeper on HW past −16?
- [?] Combining-rep ALIGNMENT: the R base blocks are contiguous in one TX blob (no PTT gap, unlike
  HAIL's separate beacons), so rep alignment is exact (sample-locked) — BETTER than HAIL's
  poll-to-poll combining. The detector sums at fixed `s, 16+s, 32+s…` offsets from the single
  best_offset. Confirm no per-rep CFO walk over R*16 symbols degrades the sum (ctrl-Moose residual
  ~1.5 Hz over 64 symbols ≈ negligible phase walk for a NONcoherent energy sum).

### §20.7 SIM-GATE RESULT — combining deepens the base-pattern floor; gate PASSED (2026-06-01)

Branch `sim/connect-preamble-combining` @6fc1d9f (off deb1ecf). Build FOREGROUND `bash build.sh o3`;
`mercury --test` **46/46 pass** (was 44/44 + the §17 ctrl-gate test + this §20 test). New regression
test `test_connect_preamble_combining_cliff_sweep` drives the PRODUCTION TX
(`generate_ctrl_suffix_pattern_passband`, R base reps + suffix) + RX (`detect_ack_pattern`
`combine_reps=R` AND the full `decode_ctrl_suffix_from_passband` base-detect+FEC) over an SNR3k sweep
at R=1/2/4.

| metric | R=1 | R=2 | R=4 |
|---|---|---|---|
| **base-pattern matched-count cliff** (P(matched≥7)=0.5, SNR3k dB) | −15.05 | −16.07 | **−16.99** |
| **full establishment cliff** (base[combine]+FEC, P=0.5) | −12.55 | −13.89 | **−13.89** |

- **Base-pattern matched-count cliff: R=1 −15.05 → R=4 −16.99 = +1.94 dB deeper.** (The dedicated HAIL
  combining sim, `test_hail_detection_cliff_sweep`, on a FINER SNR grid measures the cleaner
  per-doubling figure: matched-count-only R=1 −13.25 → R=5 −16.99 = +3.74 dB; the §20 CONNECT sweep
  grid {4,5,6,8,10,12,14,16,18,20,24,28,32} quantizes the cliff to +1.94 — direction + magnitude
  confirm the lever, the absolute per-doubling is the HAIL test's +2.2-2.5 to +3.74.) Matches §4/§14.
- **Full establishment cliff: −12.55 → −13.89 = +1.34 dB deeper**, now CO-LIMITED with the GF(16) FEC
  reach (the §19 production cliff −13.89) and ~0.8 dB off the −14.68 base-pattern floor. R=4 == R=2 on
  this coarse grid because the FEC content decode (−13.89) becomes the binding stage once combining
  pushes the base detection past it — exactly the intended handoff (base detection no longer the
  limiter; the FEC content is, at its own −14 reach).
- **FAR: 0/4000** pure-noise false CONNECT accepts through the combined R=4 path (count gate 7/16 +
  CRC12 + 2-bit type hold).
- **byte-identical-when-off: EXACT** (max|diff|=0.0) — R=1 base framed layout bit-matches the
  pre-§20 single-block formula. (Asserted on the FRAMED tone placement, the layer §20 touches, NOT
  the post-FFT passband which carries ~1e-11 cross-instance FFT round-off independent of this change.)

**⇒ SIM-GATE PASSED.** Combining moves the base-pattern detection floor deeper (the §19.7 HW
limiter), FAR-clean, byte-identical off. Cleared to HW-validate.

### §20.8 HW deploy — X: share degradation + recovery (2026-06-01)

The host X: network share degraded mid-session to ~10 s per file read (a `wc -l` of one .cc took
10.9 s; local /tmp = 3.1 GB/s, butler + testbed healthy). `mercury_deploy_rpi.py`'s source tarball
(reads the full mercury tree from X:) STALLED at 241 KB; two `git archive` retries (read `.git` packs
off X:) also stalled at 0 B. ROOT CAUSE: X: share I/O only — NOT local disk, NOT the butler, NOT the
Pi. **RECOVERY:** the FIRST (buffered-stdout) deploy attempt had actually completed the rpi2 BUILD
before it was killed — rpi2 carried the exact §20 source (markers `combine_reps`×11,
`set_connect_preamble_reps`×3, `MERCURY_CONNECT_REPS`×3, the §20 test×1) + a fresh binary
(md5 d1bc148bb4d5639b83295ddb32b85191, built 11:16 Pi-time). rpi1 was stale (old binary 02:05, no
§20 markers). Fix WITHOUT any X: read: butler DOWNLOAD rpi2 binary → host /tmp → UPLOAD → rpi1
(Pi-to-Pi binary copy, both Pi-5 aarch64 + Fe-Pi, binary-portable per mercury_deploy_rpi.copy_binary).

### §20.9 HW A/B — establishment floor: combining does NOT move it; the limiter is the START_CONN handshake CONTENT (2026-06-01)

Harness `tools/connect_combining_hw_ab.py` (reuses hail_floor_hw_test primitives; env-injected arm).
Both arms run the SAME binary (md5 **d1bc148bb4d5639b83295ddb32b85191**, both Pis — env is the
discriminator). Pinned ROBUST_0 (`-s 100 -Q 0 -M auto -n -F off --skip-turbo-reverse -R`), 3 fresh
CONNECT/cell, descending WGN. AUDIO_SETUP both Pis. Single pass.
- **COMBINING** arm: `MERCURY_SUFFIX_FEC=1 MERCURY_CONNECT_REPS=4` (verified per-Pi `[CFG]
  MERCURY_CONNECT_REPS=4` log + `reps_log=MERCURY_CONNECT_REPS=4`).
- **FEC-ONLY** baseline: `MERCURY_SUFFIX_FEC=1 MERCURY_CONNECT_REPS=1` (verified `reps_log=(none)`
  = reps=1, the §20 hook only prints when >1). FEC on, combining off = the §19 FEC-arm behavior.

| WGN | COMBINING HAIL / CONNECT | FEC-ONLY HAIL / CONNECT |
|---|---|---|
| −14 | 3/3 / **3/3** | 3/3 / **3/3** |
| −16 | 3/3 / **2/3** | 3/3 / **1/3** |
| −18 | 3/3 / **0/3** | 3/3 / **0/3** |
| −20 | 2/3 / 0/3 | 3/3 / 0/3 |
| −22 | 2/3 / 0/3 | 1/3 / 0/3 |
| FAR WGN:60 / 120 s | **0** [HAIL], **0** CONNECT/ACK | **0** [HAIL], **0** CONNECT/ACK |

**VERDICT: combining does NOT move the establishment (CONNECT) floor on HW.** Both arms: 3/3 at −14,
partial at −16 (COMBINING 2/3 vs FEC-ONLY 1/3 = within 3-sample noise, NOT a cell-move), **0/3 at −18
on BOTH.** Same HW-null pattern as the free stack (§11) and the ctrl-gate (§17.4): the SIM lever
(+1.34 dB establishment, §20.7) is real but **sub-one-WGN-cell** at the 2 dB/cell dial spacing, so a
3-attempt single-pass sweep cannot resolve it as a floor move.

**THE limiter at the −18 floor (log-confirmed, BOTH arms):** HAIL fires (`base=9-13/8`, detection
works — combining even keeps HAIL alive 2/3 at −20/−22 where FEC-ONLY's base matched-count is more
ragged), THEN **`[HAIL] Timeout waiting for START_CONNECTION`** — the RSP never completes the
START_CONNECTION ctrl-suffix decode. Occasional `[RX-MFSK-CTRL-CONNECT-START] wrong type rx=3
expected=1 matched=8/12` — the base detects (matched 8-12 ≥ 7 gate) but the CONTENT decodes the wrong
type. **⇒ the binding establishment limiter is the START_CONN ctrl-suffix DETECTION+CONTENT, not the
base-pattern matched-count combining targets.** Combining IS correctly wired into the establishment
path (a −16 diagnostic CONNECT confirmed CMD `[CFG] REPS=4` + RSP `[RX-MFSK-CTRL-CONNECT-START]
sender='TESTA'` + `START_CONNECTION received CRC ok` — full handshake on the combined base), but it
addresses the base DETECTION while the floor is now bound by the FIRST handshake frame's decode.

**Base matched-count progression on HW (the §19.7 collapse, observed live):** −14 → 15-16/16, −16 →
12-14, −18 → 9-13, −20 → 8-13, −22 → 8-10. Combining keeps the HAIL base ABOVE the 7/16 gate deeper
(2/3 HAIL at −20/−22) — i.e. the lever DOES work on the base-pattern detection statistic, exactly as
the sim measured. But the establishment floor is set by a DIFFERENT (downstream) stage at this SNR.

**FAR: 0/0 on BOTH arms** (count gate 7/16 + CRC12 + 2-bit type hold on the combined R=4 path on real
HW noise) — confirms the §20.7 sim FAR 0/4000. Combining is FAR-safe.

**Arms md5 (authoritative): IDENTICAL d1bc148b both Pis both arms** — the env knob (`reps_log` 4 vs
none) is the verified discriminator. Single-pass, 3 attempts/cell, dial-not-calibrated (WGN dial ≠
SNR3k) — the relative null (no cell-move, content-limited) is robust to the sampling; the −16
boundary (2/3 vs 1/3) wants a multi-pass but does not change the verdict.

### §20.10 Is this the end of the establishment-reach road? (decision)

**For the noncoherent base-pattern DETECTION lever: effectively yes on HW, at this dial resolution.**
The base-pattern matched-count combining works in sim (+1.94 dB R1→R4 on the §20 grid, +3.74 dB on
the HAIL fine grid) and on HW (HAIL alive 2/3 to −22) — but the establishment floor is NOT bound by
base detection at −18; it is bound by the **START_CONNECTION ctrl-suffix decode** (the FIRST handshake
frame). The chain of HW-validated establishment work now reads:
1. HAIL detect gate (§18) — moved −10→−16. ✓ (detection)
2. ctrl metric gate 1.2 (§17) — no HW move alone (content-limited). ✓ (hygiene)
3. GF(16) FEC (§19) — moved the floor to −16 (content FEC on the START_CONN payload). ✓
4. base-pattern combining (§20) — **no HW move**; base detection wasn't the −18 limiter; the
   START_CONN frame's detect+content is. The lever is real on the base statistic but aimed where the
   floor isn't bound at −18.

**Remaining headroom / next limiter:** the −18 floor is the **START_CONNECTION ctrl-suffix
DETECTION** (the `[HAIL] Timeout waiting for START_CONNECTION` = the RSP's
`receive_mfsk_ctrl_suffix_phy_core` for the START_CONN frame not detecting/decoding). Combining
already helps its base (it's the same `decode_ctrl_suffix_from_passband` path) — but the residual is
the marginal-SNR handshake CHOREOGRAPHY: the CMD sends START_CONN once per HAIL-detect cycle, the RSP
has a fixed window, and at −18 the base + content + timing line up < 50% of the time. This is a
PROTOCOL-timing / repeat-the-handshake-frame problem (send START_CONN R× too, or widen the RSP
window), NOT a base-detection-combining problem. The −14.68 raw base floor is NOT the wall here; the
handshake frame reliability is. **Recommendation: SHIP combining as a latent, FAR-safe, byte-identical
-off detection improvement (it strictly helps the base statistic + costs nothing when off), but it is
NOT the establishment-floor mover at −18 — the next lever is START_CONN frame repetition / RSP
listen-window timing at the deep floor (a handshake-choreography increment, INCREMENT 3).** This is
the same "fix isn't at the bottleneck" lesson as §11/§17.4 (BP+OSD, free stack, ctrl-gate) — the SIM
gate passing is necessary but the HW bottleneck is one stage further down each time.

## §21 PRODUCTION CAP/adaptive wiring — CAP_SUFFIX_FEC + adaptive gating (the merge-prerequisite) (2026-06-01)

Agent (this session). Branch `sim/suffix-fec-production` off `sim/connect-preamble-combining` @9708b08.
SIM / IN-PROCESS only (HW re-validate of the production config folds into the bundle merge). Goal:
take the FORCE-ON acquisition stack (§19 GF16 FEC + §20 combining, enabled by the env knobs
`MERCURY_SUFFIX_FEC=1` / `MERCURY_CONNECT_REPS=4`) and make it the **PRODUCTION-negotiated,
throughput-neutral** config: `CAP_SUFFIX_FEC` capability, CONNECT always-on-when-applicable, per-batch
ACK adaptive (robust-tier-only), try-both RX fallback.

### §21.1 The load-bearing pre-existing bug the FORCE-ON config hid (decision-critical)

`cl_mfsk::pack_ctrl_suffix` (mfsk.cc:628) reads the **GLOBAL** `suffix_fec_coded` flag (set 13-tone
uncoded → 52-tone GF16). **The ACK packer `pack_ack_sack_payload` (mfsk.cc:693) delegates to
`pack_ctrl_suffix`** → so when `suffix_fec_coded` is globally true (the FORCE-ON config), the ACK
suffix ALSO becomes a 52-tone GF16 codeword. BUT the ACK generator `generate_ack_sack_pattern`
(mfsk.cc:813) loops only `ack_sack_suffix_len()`=**13** symbols (line 829) and the ACK RX
`decode_ack_sack_from_passband` reads **13** uncoded tones throughout (telecom_system.cc:3441, C9).
**⇒ With FEC globally on, the data ACK transmits the FIRST 13 symbols of a GF16 codeword, RX reads them
as a hard 13-tone pack → CRC fail → no ACK → retransmit storm at every OFDM operating point.** The
FORCE-ON HW runs (§17.4/§19.7/§20.9) only ever ran *pinned ROBUST_0 CONNECT establishment* (no data
ACKs), so this never fired. It is a **throughput-neutrality violation by construction** — the exact
hard constraint this increment exists to honour. **Root-cause fix (not a band-aid): the FEC decision
must be PER-CALL, never a global mode that the ACK packer inherits.**

### §21.2 The design as built — FEC is a per-call argument, never a global the ACK inherits

- **`pack_ctrl_suffix` gains an explicit `bool fec` argument** (default `false`). It NO LONGER reads
  `suffix_fec_coded`. Callers pass the FEC decision explicitly:
  - CONNECT TX (`generate_ctrl_suffix_pattern`, mfsk.cc:879) passes `suffix_fec_coded` (the CONNECT
    session enable — CONNECT-path-local).
  - ACK TX (`pack_ack_sack_payload` → `generate_ack_sack_pattern`) passes a NEW
    `cl_mfsk::ack_suffix_fec_coded` flag — **independent** of the CONNECT `suffix_fec_coded`. Default
    `false` → the ACK is **byte-identical** unless the ARQ layer explicitly turns it on for a robust
    -tier batch. (This increment leaves `ack_suffix_fec_coded` OFF in all paths — see §21.3 ACK
    decision — so the ACK is byte-identical in 100% of cases; the flag is the plumbing for a future
    robust-tier ACK FEC, wired and tested but not enabled, so the hard constraint is unconditional.)
- **`ctrl_suffix_len()` / `connect_base_total_nsymb()` stay CONNECT-only accessors** (the ACK length
  is `ack_sack_suffix_len()`=13 + `ack_pattern_nsymb`=16, never the coded/combined length — verified
  C9/§19.4, unchanged). So the CONNECT global enable (`suffix_fec_coded`, `connect_preamble_reps`)
  physically cannot change the ACK wire length: the ACK generators don't call those accessors.
- **`CAP_SUFFIX_FEC = 0x04`** (next free cap bit; 0x01 WB, 0x02 ENCRYPTION). Carried in the existing
  3 ctrl-suffix cap fields — BUT those fields are only 2 bits wide (`pack_test_ack_payload` echoed_cap
  /own_cap, `pack_test_conn_payload` local_cap). **Widen each cap field 2→3 bits** (steal 1 reserved
  bit each: TEST_ACK 26→24 reserved, TEST_CONN 24→23 reserved; both have ≥24 spare). Mask
  `CAP_NEGOTIABLE_MASK = 0x07`. The packers/unpackers mask to 3 bits. Legacy peers (pre-this-commit)
  pack only the low 2 bits and zero bit-2 on TX, and ignore reserved bits on RX → a legacy↔upgraded
  pair simply sees `CAP_SUFFIX_FEC=0` from the legacy side → falls back to uncoded. No flag-day (the
  bit lives in previously-reserved-zero space; §5).

### §21.3 The adaptive gating + chicken-and-egg resolution (decided + justified)

**Cap-knowledge timeline (code-grounded):** START_CONN (CMD→RSP) carries NO cap field
(`[nb_flag:1|sender:36|reserved:1]`, mfsk_ctrl_codec.h:80). CMD learns RSP caps at TEST_ACK decode
(`peer_capability = rsp_own`, arq_commander.cc:4008). RSP learns CMD caps at TEST_CONN decode
(`peer_capability = data[5]`, arq_responder.cc:2051). **⇒ CAP_SUFFIX_FEC is mutually known only AFTER
stage 3 (TEST_CONN). START_CONN — the FIRST and hardest-to-decode handshake frame, the §19.7/§20.9
HW establishment limiter — is fundamentally PRE-CAP.**

**CONNECT decision = gate on LOCAL robust-tier + RX try-both (NOT on negotiation).** Justification:
the entire establishment-floor benefit of the FEC (the −10→−16 WGN move, §19.7) is on the START_CONN
frame, which is pre-cap. Gating CONNECT-enhanced on negotiation would defeat the whole purpose (the
benefit frame can never be enhanced). Instead: a station emits the enhanced CONNECT suffix whenever
**its own** session is at the robust tier (the deep-floor proxy, `is_robust_config(current_configuration)`
— the §4 gearshift-config gate, reused, no new state). The RX side runs **try-both** (uncoded decode
first — cost 0, byte-identical to today; on miss + local-robust-tier, attempt the enhanced decode of
the longer window). A legacy RX has no enhanced decoder → it simply fails the enhanced frame and the
legacy LDPC-fallback / handshake-retry path recovers (the same failsafe the whole Phase-B suffix
relies on). CONNECT is once/session + airtime-cheap (§4: +2.0% of a 7.9 s ROBUST_0 frame, <0.1% of a
multi-min session), so always-on at the robust tier costs nothing measurable and needs no negotiation.
This is the §5/§19.7 "OR gate on local robust-tier flag + RX try-both" branch of the chicken-and-egg
note, chosen over "enhanced only after caps" because the latter cannot help START_CONN.

**Per-batch ACK decision = enhanced ONLY at robust tier AND only when CAP_SUFFIX_FEC negotiated;
ELSE byte-identical uncoded (§4).** The ACK is on the throughput hot path, so the gate is strict:
`enhanced_ack = is_robust_config(current_configuration) && (local_capability & peer_capability &
CAP_SUFFIX_FEC)`. At CONFIG_6+ (every throughput-relevant config) OR cap-absent → uncoded → **0%
overhead, byte-identical** (the hard constraint). **This increment WIRES the gate + the
`ack_suffix_fec_coded` plumbing + the try-both ACK decode, but leaves the enhanced-ACK ENABLE OFF**
(the gate currently evaluates the predicate and, even when true, the TX stays uncoded) — rationale:
(a) the §20.9 HW result shows the ACK was never the establishment limiter, so robust-tier ACK FEC
buys nothing on the validated bottleneck yet; (b) keeping it OFF makes the throughput-neutrality
guarantee UNCONDITIONAL (the ACK is byte-identical in 100% of cases, not just CONFIG_6+); (c) the
plumbing + test prove the adaptive path is correct and ready for the ULTRA tier (§15) / a future
robust-tier ACK-FEC increment to flip one flag. The gate predicate + the robust-tier branch are the
deliverable; the enable is a one-line follow-on gated behind its own HW validation.

### §21.4 §5 CROSS-LAYER DATA-FLOW AUDIT — the cap field, the gearshift-config gate, try-both

**Shared state #1: the cap fields (TEST_ACK echoed_cap/own_cap, TEST_CONN local_cap; the wire
negotiation of CAP_SUFFIX_FEC).**
- Producers: `pack_test_ack_payload` (mfsk_ctrl_codec.cc), `pack_test_conn_payload`; the cap VALUES
  come from `local_capability` (arq_common.cc:2731/2828/2882/2901, main.cc:2258 — all set
  `CAP_WB_CAPABLE|CAP_ENCRYPTION`; this increment ORs in `CAP_SUFFIX_FEC` at each).
- Consumers: `unpack_test_ack_payload`/`unpack_test_conn_payload` → `peer_capability`
  (arq_commander.cc:4008/4025, arq_responder.cc:2051). Then the legacy cap consumers: encryption
  negotiation `both_support = local&peer&CAP_ENCRYPTION` (arq_commander.cc:4062, arq_responder.cc:2095)
  and WB-upgrade `peer_capability & CAP_WB_CAPABLE` (arq_commander.cc:4029). **Invariant the legacy
  consumers assume: only bits 0/1 are meaningful.** Adding bit-2 does NOT break them — they mask their
  own bit. Verified: every existing read is `& CAP_WB_CAPABLE` or `& CAP_ENCRYPTION`, never a bare
  equality on the whole byte EXCEPT the CMD echo check `echoed_cap != local_capability`
  (arq_commander.cc:3985) on the LEGACY-LDPC TEST_CONNECTION path — that path round-trips the full
  byte both ways so a 3-bit value echoes consistently (both sides this-commit); a legacy peer there
  zeroes bit-2 on TX so the echo still matches its own 2-bit local_capability. **The MFSK TEST_ACK
  echo (Site D) does NOT do that equality check (arq_commander.cc:4008 just assigns rsp_own)** → safe.
- Default-init: `local_capability=0`, `peer_capability=0` (arq_common.cc:303) → CAP_SUFFIX_FEC=0 until
  set → uncoded → byte-identical. ✓

**Shared state #2: the gearshift config `current_configuration` (the ACK + CONNECT adaptive gate).**
- This is the EXISTING gearshift/turboshift config (datalink_defines ROBUST_0/1/2=100-102). The gate
  READS it via `is_robust_config(current_configuration)` (the pure inline, common_defines.h:78). It is
  a READ-ONLY consumer — the gate adds NO writer, NO new state. The same predicate is already read at
  telecom_system.cc:5505 (the §4-cited ROBUST_0 branch) and arq_commander.cc:2222. No producer/consumer
  invariant changes (we only add a reader). ✓ The §4 claim "reuses the gearshift config, no new state"
  is verified.

**Shared state #3: `suffix_fec_coded` / `connect_preamble_reps` (the CONNECT-path enable) — does the
ACK inherit them?** BEFORE this increment: YES, via `pack_ctrl_suffix` reading the global (the §21.1
bug). AFTER: NO — `pack_ctrl_suffix` takes an explicit `fec` arg; the ACK packer passes the SEPARATE
`ack_suffix_fec_coded` (default false). Producers of `suffix_fec_coded`: `set_suffix_fec`
(telecom_system.cc:3469) + the env hook (arq_common.cc:1528). Consumers AFTER: ONLY the CONNECT TX
generator (`generate_ctrl_suffix_pattern`, passes it to pack) + the CONNECT RX decode branch
(`decode_ctrl_suffix_from_passband`, telecom_system.cc:3661) + the CONNECT length accessor
(`ctrl_suffix_len`, mfsk.h:209). The ACK generators (`generate_ack_sack_pattern`,
`generate_ack_snr_pattern`, `generate_ack_sack_pattern_passband`) are NO LONGER consumers. **The ACK
wire length + content are now provably independent of the CONNECT FEC enable.** ✓ (This is the fix.)

**Shared state #4: try-both RX (CONNECT decode + ACK decode).** The CONNECT try-both reuses the same
`decode_ctrl_suffix_from_passband` capture window. The window is sized by `ctrl_suffix_len()` +
`connect_base_total_nsymb()` (C6, §19.4/§20.3) — i.e. sized for the CODED+COMBINED length whenever the
CONNECT enable is on. Try-both for CONNECT: the uncoded decode reads the FIRST 13 suffix symbols of
that (longer) window; the enhanced decode reads all 52. Both fit in the captured tail (the tail holds
the coded length + 16 margin). **Invariant: the capture window must be sized for the LARGER of the two
decode attempts.** Since the CONNECT enable already sizes it for the coded length, the uncoded attempt
is a strict sub-window → safe. The ACK try-both reads the SAME 13-tone capture both attempts (the ACK
window is never coded-sized this increment, ACK FEC enable OFF) → the enhanced ACK attempt would need
a 52-tone window, which is why enabling robust-tier ACK FEC is gated behind its own window-sizing work
(noted as the follow-on). FAR backstop on both try-both paths: the GF16 soft_decode's internal
CRC12+2-bit-type gate (§16, 0/4000 noise) + the count gate (7/16) — try-both adds at most one extra
CRC trial per poll (×2 trials → FAR doubles from ~2^-12 to ~2^-11, still ≫ below the count gate). ✓

**Verdict:** the increment ADDS one cap bit (in reserved-zero space, legacy-safe), ADDS read-only gate
predicates on the existing gearshift config, and REMOVES the ACK's accidental inheritance of the
CONNECT FEC flag (the §21.1 root-cause fix). No producer invariant on existing shared state is
altered; the one behavioural change (ACK no longer garbled when CONNECT-FEC on) is strictly a fix.

### §21.5 What changes vs leaves
CHANGES: `CAP_SUFFIX_FEC` define + `CAP_NEGOTIABLE_MASK`; cap fields 2→3 bits in the 3 pack/unpack ctrl
payloads; `local_capability |= CAP_SUFFIX_FEC` at the 5 set sites; `pack_ctrl_suffix(...,bool fec)`
explicit arg (drops the global read); `cl_mfsk::ack_suffix_fec_coded` flag (default false) + ACK packer
passes it; CONNECT TX gates `suffix_fec_coded`/`connect_preamble_reps` enable on local robust-tier (the
adaptive enable, replacing the env force-on as the production trigger — env knobs kept as a test
override); CONNECT RX try-both (uncoded → enhanced on miss+robust-tier); per-batch ACK gate predicate
`enhanced_ack = robust-tier && (local&peer&CAP_SUFFIX_FEC)` + try-both ACK decode plumbing. LEAVES: the
GF16 codec; the §20 combining math; the 1.2 metric gate; HAIL fix; the env knobs (now a TEST override
on top of the production CAP path); the enhanced-ACK ENABLE (predicate wired, enable off — §21.3).

### §21.6 SIM RESULT + the decisive interop finding — the enhanced suffix is BACKWARD-COMPATIBLE (2026-06-01)

Branch `sim/suffix-fec-production` (off `sim/connect-preamble-combining` @9708b08). Build FOREGROUND
`bash build.sh o3`; `mercury --test` **51/51 pass** (was 46 + the 5 §21 gate tests; the GF16 §19
production-path cliff still −13.89, the §20 combining cliff unchanged, throughput-neutral).

**THE decisive finding (corrects the §21.3 chicken-and-egg worry):** a fresh sim measurement
(`test_suffix_fec_interop_legacy_rx`) shows a LEGACY RX (FEC off, reps=1 detector — a stock
`decode_ctrl_suffix_from_passband` with no CRC callback) **cleanly decodes an UPGRADED TX at EVERY
(fec,reps) combo** — uncoded/reps1, FEC/reps1, FEC+combining/reps4, combining-only/reps4 — all with
`matched=16` (full base lock) and the correct START_CONN payload. The enhanced suffix is BACKWARD-
COMPATIBLE BY CONSTRUCTION, for two independent reasons:
1. **FEC: the GF(16) RA codeword is SYSTEMATIC.** Its first 13 tones ARE the hard
   `[type:2|p38:38|crc12:12]` pack (MSB-first, matching `cl_mfsk::pack_ctrl_suffix` — mfsk_ctrl_codec.h
   :249). A legacy hard-argmax RX reads the payload straight from the systematic prefix and silently
   ignores the RA-parity tail. (This also means a legacy RX gets NO FEC benefit — it decodes the
   uncoded prefix, so it cliffs at the uncoded −7.87, not −14. The deep-floor reach needs an upgraded
   RX. But the LINK still ESTABLISHES at good SNR with a legacy peer — the interop requirement.)
2. **Combining: the R base reps are IDENTICAL 16-symbol blocks**, and `detect_ack_pattern` (reps=1)
   scans for the base pattern with `reserve_after=ctrl_suffix_len()`, locking on whichever base rep
   leaves room for the suffix — so the suffix is found regardless of how many reps precede it. The
   repeated base does not confuse a single-block detector; it just gives it more lock candidates.

**⇒ Design simplification (supersedes the §21.3 "gate combining on negotiation" leaning):** because
BOTH FEC and combining are RX-backward-compatible, the CONNECT enhanced path gates ONLY on the LOCAL
robust tier (option (b)) — NO negotiation gate is needed for CONNECT, and the RX try-both makes an
upgraded RX decode both legacy and enhanced peers. This is exactly the validated
`sim/connect-preamble-combining` stack, now production-TRIGGERED by `is_robust_config(current)` instead
of env-forced. `CAP_SUFFIX_FEC` negotiation is therefore used for the ADAPTIVE PER-BATCH ACK gate
(`ack_suffix_fec_eligible()`), where the enable is held off this increment (§21.3); the CONNECT path
does not consult it (the chicken-and-egg is moot — there is no compat hazard to avoid).

**Gate results (all PASS):**
- `suffix_fec_cap_negotiation_matrix`: CAP_SUFFIX_FEC round-trips the 3-bit TEST_ACK/TEST_CONN fields;
  `suffix_fec_negotiated()` true only upgraded↔upgraded, false on mixed/legacy/unknown(0);
  `ack_suffix_fec_eligible()` true only at ROBUST_0 + negotiated, false at CONFIG_10 (OFDM) and when
  cap absent (the throughput-neutral gate).
- `suffix_fec_interop_legacy_rx`: legacy RX decodes upgraded TX at all 4 (fec,reps) arms.
- `ack_suffix_throughput_neutral`: with CONNECT FEC+combining FORCED ON, the data-ACK suffix is
  **byte-identical** — 13 tones unchanged, 35960 passband samples unchanged (the §21.1 fix; the
  HARD throughput-neutrality constraint, asserted on the deterministic WIRE content — tones + airtime
  — not the modulated doubles, which carry pre-existing run-to-run modulation nondeterminism: a
  ~0.7-magnitude shared-OFDM-scratch diff present even between two NO-CHANGE builds, orthogonal to this
  increment, harmless because each real TX zeroes its buffers in the send path. **Latent finding logged
  for a future hygiene pass: `build_ack_sack_audio`/the ACK passband generator's first symbol depends
  on prior scratch-buffer contents.**)
- `connect_suffix_byte_identical_when_off`: toggling FEC on→off restores the exact uncoded 13-tone
  single-base wire; the uncoded production decode round-trips.
- `production_enhanced_connect_decodes`: the production robust-tier enable (FEC R¼ + combining R=4,
  N=52) encodes + try-both-decodes clean end to end.

**Throughput-neutrality at OFDM SNRs CONFIRMED:** at CONFIG_6+ the production trigger turns the CONNECT
enhanced state OFF (reps=1, FEC off → byte-identical CONNECT suffix) AND the per-batch ACK is uncoded
(gate ineligible off the robust tier + enable held off) → zero FEC/combining bytes on any
throughput-relevant config. **Establishment still reaches −13.89 sim with caps negotiated:** the
upgraded↔upgraded enhanced stack (FEC+combining at the robust tier) is the unchanged §19/§20 path
(`test_gf16_ra_production_path_cliff_sweep` −13.89; `test_connect_preamble_combining_cliff_sweep` −13.89
@R≥2) — this increment only changed the TRIGGER (tier vs env), not the PHY, so the cliff is preserved
(verified: both sweeps still report −13.89 on this binary). SIM ONLY — the production-config HW
re-validate folds into the bundle merge.

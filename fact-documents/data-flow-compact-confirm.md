# Data-flow: compact coded reverse-confirm suffix (Option B) — C9 ACK-tail audit

Owner: overnight autonomous ARQ/FEC engineer. Created 2026-06-27 on
`staging/short-coded-confirm` (base 346d305f = crossing 027cc927 + cfg8 fix
acb4edf9). This is the CLAUDE.md §5 cross-layer audit that MUST complete before
the Option B wire change. It enumerates every producer/consumer of the ACK-tail
suffix symbol count and resolves the same-window disambiguation question the
SPEC flagged as the top open item.

Companion: `_research/coded_confirm/{SPEC.md,feasibility.json,feasibility.md}`
(feasibility BER: K=5 N=10 GF(16)-RA compact confirm = -11.50 dB content cliff,
+4.18 dB deeper than the uncoded-13 suffix, 3 symbols shorter; FEASIBLE).

## §1 The shared state: the ACK tail after the 16-sym base

After the 16-symbol ACK base pattern, the same audio tail is the home of THREE
suffix types today, all M=16 noncoherent FSK, all tone-hopped continuing the base
hop (`abs_s = ack_pattern_nsymb + s`):

| suffix type | length (sym) | accessor | content gate |
|---|---|---|---|
| turboshift SNR suffix | 8 (`SNR_SUFFIX_LEN`) | `ack_snr_pattern_nsymb()` | majority vote, 3/8 |
| ACK+SACK / clean-data-ACK (uncoded) | 13 (`ack_sack_suffix_len()`) | `ack_sack_pattern_nsymb()` | CRC12 over [bsi\|\|bitmap] |
| ACK+SACK FEC (held off) | `gf16ra::codeword_len()` (52 @ R1/4) | `ctrl_suffix_len()` w/ `ack_suffix_fec_coded` | CRC12 (in soft_decode) |

Option B ADDS a FOURTH: the compact confirm at N=10 (K=5 GF(16)-RA, R1/2).

## §2 Producers (TX) — file:line

- `cl_mfsk::generate_ack_sack_pattern` (mfsk.cc:872) — emits 16 base +
  `ack_sack_suffix_len()` suffix tones. Calls `pack_ctrl_suffix(..., fec)` where
  `fec = ack_suffix_fec_coded` (the §21 per-call flag, NOT the CONNECT global).
- `cl_mfsk::generate_ack_snr_pattern` (mfsk.cc:1095) — emits 16 base +
  `SNR_SUFFIX_LEN`=8 suffix tones (all same tone = quantized SNR).
- `cl_mfsk::pack_ctrl_suffix` (mfsk.cc:648) — `fec=true` → `gf16ra::encode` ⇒
  `gf16ra::codeword_len()` tones; `fec=false` → 13-symbol hard bit-pack.
- `cl_telecom_system::generate_ack_sack_pattern_passband` (arq_common.cc:7016
  sets `ack_mfsk.ack_suffix_fec_coded = (ARQ_ACK_SUFFIX_FEC_ENABLE!=0) &&
  ack_suffix_fec_eligible()` then clears it after TX). **ARQ_ACK_SUFFIX_FEC_ENABLE
  = 0** (common_defines.h:66) ⇒ ACK FEC is OFF in 100% of cases today; the ACK
  suffix is always the 13-uncoded pack.
- CONNECT FEC producer: `set_suffix_fec(on,3)` (telecom_system.cc:4092) →
  `gf16ra::configure(3); gf16ra::init()` (K=13, N=52). Enabled at arq_common.cc:2392
  for the CONNECT path. This is the ONLY production consumer of the gf16ra singleton.

## §3 Consumers (RX) — file:line

- `cl_telecom_system::detect_ack_snr_from_passband` (telecom_system.cc:~3900) —
  THE shared detector. On any ACK base match it captures
  `capture_len = max(SNR_SUFFIX_LEN=8, ack_sack_suffix_len()=13)` = 13 de-hopped
  tones into `ack_mfsk.last_ack_sack_suffix_tones[]` (sized MAX_ACK_SACK_SUFFIX=64,
  mfsk.h:273). decode_suffix_tones writes -1 past buffer end (safe).
- SNR consumer: the majority-vote block (telecom_system.cc:~4018) reads the first
  8 captured tones.
- SACK / clean-data-ACK consumer: `decode_ack_sack_from_passband`
  (telecom_system.cc:4069) → `detect_ack_snr_from_passband` (re-detect) →
  `decode_ack_sack_from_last_capture` → `unpack_ack_sack_payload` (13 tones).
- Soft ACK consumer (Tier-1 list): `decode_ack_sack_from_passband_soft`
  (telecom_system.cc:4513) → `decode_suffix_candidates` → `soft_list_decode_ctrl_suffix`.
- The commander accept gate: `cl_arq_controller::cmd_clean_data_ack_crc_valid`
  (arq_commander.cc:100) — captures the tail (window sized
  `ack_nsymb + max(snr_pattern, sack_suffix) + 16`), calls
  `decode_ack_sack_from_passband`, then CRC12 over the 5-byte [bsi\|\|bitmap] +
  bsi-in-window + all-ones (CLEAN) gate. Consumed at arq_commander.cc:4285.
- CONNECT-suffix FEC consumer: `decode_ctrl_suffix_from_passband` /
  `..._soft` (telecom_system.cc:4187/4424) → `decode_suffix_energies` →
  `gf16ra::soft_decode` (K=13). CONNECT base pattern (connect_tones), NOT ack_tones —
  so the BASE pattern already discriminates CONNECT from ACK before any suffix decode.

## §4 The disambiguation answer (the C9 question)

**How does the RX distinguish compact-confirm(10) vs SNR-suffix(8) vs SACK-suffix(13)
in the same ACK tail?** Resolved:

1. **Base pattern first.** CONNECT vs ACK is resolved by the base tones
   (connect_tones vs ack_tones) BEFORE any suffix interpretation. So the CONNECT-FEC
   suffix (K=13) NEVER shares the window with the ACK suffixes — it rides the CONNECT
   base. This removes CONNECT-vs-ACK from the tail-aliasing problem entirely.
2. **Within the ACK base**, the three ACK-tail types are disambiguated by CONTENT
   GATE, not by length negotiation. The capture is ONE fixed-length grab (13 tones);
   each consumer interprets its own prefix length and applies its own gate:
   - SNR: majority vote (no CRC — but it only sets a throughput HINT, never confirms data).
   - SACK/clean-data-ACK: CRC12 over [bsi\|\|bitmap] + bsi-in-window + all-ones.
   - Compact confirm (NEW): gf16ra K=5 soft_decode over the first N=10 energies +
     CRC12 recompute + bsi-in-window. N=10 < 13 ⇒ fits the existing capture; NO
     buffer/ring resize needed (the SHORT-suffix direction is strictly safe — unlike
     the CONNECT 52-symbol case that forced MAX_ACK_SACK_SUFFIX 16→64).
3. **No cross-acceptance.** A wrong interpretation fails its CRC (the SACK CRC12 is
   over a different field layout than the compact-confirm CRC12; the gf16ra K=5
   soft_decode recomputes the CRC over the decoded bsi and rejects on mismatch). The
   commander tries the compact path first (cheap, short) then the existing clean-data
   path; whichever CRC validates wins. The two CRC12s are over DIFFERENT inputs so a
   13-uncoded ACK cannot validate as a compact confirm and vice versa.

## §5 The architectural blocker (the §21.1 sibling) — and the fix

`gf16ra` is a namespace of all-STATIC module globals (mfsk_ctrl_codec.cc:359-381:
`g_N`, `g_NC`, `g_repfact`, `g_acc_idx[]`, `g_acc_wlog[]`, `g_factor_edges[]`) and
`GF16RA_K` is a compile-time const = 13. It is a SINGLETON config. The codebase
already recognized this (`current_repfact()` save/restore, mfsk_ctrl_codec.h:266
"the graph is process-global; a caller that temporarily configure()s a different
repfact must restore it so a concurrent consumer at another repfact is not
corrupted").

Option B at K=5 needs a SECOND, DIFFERENT graph (K=5 ⇒ different g_N/g_NC/acc_idx),
coexisting with the K=13 CONNECT FEC. If it shared the singleton via configure(),
a CONNECT-FEC decode and a compact-confirm decode interleaved in one poll would
corrupt each other — exactly the §21.1 bug (an FEC-on CONNECT silently coded the
data ACK to 52 tones). DECISION: implement the compact codec as PHYSICALLY SEPARATE
K=5 state (`g5_*` graph + `encode_compact` / `soft_decode_compact` entries), reusing
ONLY the stateless/shared-field math (fwht16, pd_*, log_i0_approx, gf_mul, the
once-built field tables g_gfexp/g_gflog which are K-independent). Zero mutation of
the K=13 globals ⇒ the proven CONNECT FEC path stays byte-identical (regression
safety), and the two configs can never clobber each other. This follows the existing
`encode_config_tag`/`soft_decode_config_tag` parallel-variant precedent
(mfsk_ctrl_codec.cc:751), extended to a different K (its own graph).

## §6 Invariants the consumers assume (verify the fix maintains each)

- I1 (capture length): the detector grabs `max(8,13)=13` tones; N=10 ≤ 13 ⇒
  satisfied, no resize.
- I2 (byte-identical-when-off): with the compact confirm NOT selected, every ACK is
  the 13-uncoded wire. The compact path is gated by a per-call flag + a clean-batch
  responder decision (NEVER a global). The K=13 gf16ra globals are untouched ⇒ CONNECT
  FEC byte-identical.
- I3 (CRC content separation): compact CRC12 is over [bsi] (1 byte, all-ones bitmap
  implicit); clean-data-ACK CRC12 is over [bsi\|\|bitmap] (5 bytes). Different inputs ⇒
  no cross-validation.
- I4 (length accessor): a `compact_confirm_len()` = N accessor routes every
  sample-count consumer (the SPEC §5.1 I4 pattern). N=10<13<52 ⇒ existing buffers/ring
  strictly safe.
- I5 (false-confirm): system FAR = CRC12(2^-12) × bsi-in-window(~2/256) ×
  base-count-gate(~2.5e-5/poll) ≈ 5e-11/poll. GT2 (bare-pattern confirm) stays retired
  — the compact confirm carries CRC12 over content, cannot regress to pattern presence.

## §7 What the fix changes (per-consumer walk)

- detector: UNCHANGED (still captures 13). The compact decode reads the energy matrix
  for the first N=10 of the same window (a separate `decode_compact_confirm_from_passband`
  via decode_suffix_energies, mirroring decode_ack_sack_from_passband_soft).
- SNR consumer: UNCHANGED (still reads 8, majority vote).
- clean-data-ACK consumer: UNCHANGED (still 13-uncoded CRC12). The commander tries the
  compact path FIRST; if it validates, accept; else fall through to clean-data-ACK.
- responder producer: selects compact (clean batch==all-ones) vs SACK (partial). Default
  OFF (per-call flag) until the build arm flips it for the clean ROBUST-tier reverse confirm.

## §8 Status

C9 audit COMPLETE. Disambiguation resolved (content-gate, base-pattern-first, separate
K=5 state). Build proceeds with the physically-separate-K=5-graph design (§5). Paired
regression tests: test_compact_confirm_cliff_sweep (BER), byte-identical-when-off,
FAR-on-noise, and a state-machine clean→compact→accept / partial→SACK→no-false-clean.

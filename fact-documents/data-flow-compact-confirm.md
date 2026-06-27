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

## §9 LIVE RX-PATH defect: the bare-pattern gate's 8-symbol pre-gate (2026-06-27)

The codec is sim-proven, but the LIVE commander REJECTED the compact confirm. The
in-process roundtrip test (test_compact_confirm_passband_roundtrip_clean) PASSED only
because it called `decode_compact_confirm_from_passband()` DIRECTLY — it bypassed the
live commander accept gate.

### §9.1 The wiring bug
The commander accept gate (arq_commander.cc, the "Data ACK pattern detected" else-if)
gated the compact decode BEHIND the bare-pattern presence gate:
```
!sack_window_open && receive_ack_pattern()
                  && ( ... || (ARQ_COMPACT_CONFIRM_ENABLE && cmd_compact_confirm_crc_valid()) || ...)
```
`receive_ack_pattern()` is a CRC-LESS 7/16 base-pattern presence gate whose only job is
to protect the legacy bare ACK. When it short-circuits FALSE, the self-validating
`cmd_compact_confirm_crc_valid()` (its OWN 16-sym base detect + GF(16) soft-decode +
CRC12 over [bsi] + bsi-in-window) is NEVER reached → every confirm missed.

### §9.2 The TRUE mechanism (corrected — bench-claim guard)
The original hypothesis ("the bare correlator under-PEAKS the shorter 26-sym frame,
4-5/16") was NOT reproduced. The clean-room sweep (test_compact_confirm_live_rx_path,
the DIAG lines) shows the in-window correlator peak is **16/16 for BOTH** the 26-sym
compact AND the 29-sym ACK+SACK frame — the base 16-sym pattern is byte-identical
(both call `generate_ack_pattern`). The real defect is `receive_ack_pattern()`'s
**8-symbol ENERGY PRE-GATE** (arq_common.cc, `probe_n = 8*sym_samples` in production):
it probes ONLY the last 8 symbols of the tail for energy before running the FFT
correlator. When the reverse confirm lands EARLIER in the tail window — which the live
capture-prep timing does (inter-Pi clock drift + PTT/turnaround drain scroll
post-frame idle silence in behind the frame before the frames_to_read==0 snapshot) —
that 8-symbol probe reads SILENCE and SKIPS the correlator → bare gate returns FALSE
though the frame is fully present. Measured phase tolerance over a 31-phase sweep
(post-frame silence 0..30 sym):

| detector | accepts |
|---|---|
| bare presence gate (receive_ack_pattern, 8-sym pre-gate) | 8 / 31 phases |
| self-validating compact decode (cmd_compact_confirm_crc_valid, full-tail correlator) | 17 / 31 phases |

So the bare gate is PHASE-FRAGILE (the 8-sym pre-gate is a production CPU optimization
that assumes the prep thread leaves a fresh ACK in the newest symbols); the compact
decode has no pre-gate and finds the frame across >2× the phases. The legacy ACK+SACK
ACK survives because its acceptance ALSO goes through receive_ack_pattern() — it is the
SAME pre-gate fragility, just masked because a missed clean ACK falls back to a SACK
window / timeout-retx; the compact confirm had no such fallback (it was a one-shot).

### §9.3 The fix (root cause, not band-aid)
Extract the bare/compact arm into ONE shared predicate
`cl_arq_controller::cmd_compact_confirm_live_accept(sack_window_open, compact_enabled,
use_legacy_chain=false)` (arq_commander.cc). It tries the SELF-VALIDATING compact
confirm FIRST, DECOUPLED from receive_ack_pattern(); on accept it advances the ring via
`commit_ack_pattern_consumed()` (the frames_to_read=4 the bare gate's accept branch
would have done). The legacy bare ACK arm (receive_ack_pattern() + WB content gate
cmd_clean_data_ack_crc_valid()) is UNCHANGED. The bare 7/16 presence gate is NOT
weakened (no threshold band-aid) — the CRC12 over [bsi] is the false-confirm protection
for the decoupled compact path. compact_enabled = ARQ_COMPACT_CONFIRM_ENABLE (held off)
⇒ byte-identical to the legacy bare arm when 0.

### §9.4 Invariants re-verified by the fix
- I3 / I5 (false-confirm): RE-VERIFIED on the live path — noise tail REJECTED;
  out-of-window bsi REJECTED (bsi-in-window gate); a 13-uncoded ACK+SACK does NOT
  cross-validate as a compact confirm (different CRC fields). The decoupling moves the
  protection from the bare 7/16 count to the CRC12 — STRICTER, not weaker.
- Ring advance: the compact accept now calls commit_ack_pattern_consumed() (it used to
  ride the bare gate's frames_to_read=4). No un-consumed-ring regression.
- Legacy ACK+SACK / bare ACK / control-ACK / BREAK paths: receive_ack_pattern()
  itself is UNTOUCHED → byte-identical.

### §9.5 Test
`test_compact_confirm_live_rx_path` (source/datalink_layer/test_compact_confirm_rx.cc,
CLI --test-compact-confirm-rx; runs in `--test`). Drives the FULL live RX path
(passband → commander capture ring → cmd_compact_confirm_live_accept). FAIL-BEFORE
(MERCURY_COMPACT_RX_FAILBEFORE=1, same binary): the compact confirm is REJECTED at the
defect phase (rc=1). PASS-AFTER (default): accepted via the decoupled CRC-gated decode;
the in-window phase still accepts in both arms; ACK+SACK still accepts; noise +
out-of-window + cross-frame all reject (rc=0, 9/9). The DIAG lines print the 16/16
in-window peak and the 8/31-vs-17/31 phase-tolerance evidence.

### §9.6 Follow-up [?]
The 8-symbol pre-gate phase fragility is GENERAL (it also clips the legacy clean ACK's
phase tolerance, masked only by the SACK/timeout fallback). A separate, larger fix —
widening or removing the production 8-symbol pre-gate (it is a CPU optimization, and the
full-tail correlator already runs) — would harden ALL reverse-ACK detection, not just
the compact confirm. Tracked here as an open lever, NOT folded into this fix (scope:
this fix decouples the self-validating confirm; the pre-gate widening touches every
receive_ack_pattern() caller and needs its own CPU-budget + FAR audit).

## §10 Producer↔consumer MATRIX + the SECOND live-land defect (SACK-window shadow, 2026-06-27)

f1da9cbe (§9) fixed the SACK-CLOSED bare-gate decoupling. A full §5 producer/consumer
trace (`_research/coded_confirm/{audit_rsp_send.md, audit_cmd_accept.md, audit_SYNTHESIS.md}`)
found a SECOND, independent live-land defect for batch>1. This section owns the
producer↔consumer disagreement for the compact suffix.

### §10.1 Producer (RSP) — WHERE compact is emitted
`arq_responder.cc:2571-2583`, inside the CLEAN funnel. FOUR conditions (all required):
(a) `ARQ_COMPACT_CONFIRM_ENABLE != 0` (common_defines.h:82-83, DEFAULT 0);
(b) `!cumulative_ack_enabled` (mutual-exclusion w/ FORGIVING-ACK Tier 2, :2572);
(c) `compact_confirm_suffix_len() > 0` == `M>=16` == WB only (mfsk.h:278-280);
(d) `wire_bsi == ack_bsi` (no cumulative n_r reshape, :2574).
The CLEAN funnel does NOT branch on batch size (`audit_rsp_send.md §1`): b1-clean and
b>1-clean take the IDENTICAL path. ROBUST/NB (M=8) -> suffix_len==0 -> compact never
emitted; ROBUST pins data_batch_size=1 (arq_common.cc:1285,1318-1335). The partial path
(2221-2438) NEVER emits compact (complete-batch / all-ones only).

### §10.2 Consumer (CMD) — WHERE compact is accepted
`arq_commander.cc:4448-4451` (the data-ACK else-if) -> two routes:
(i) `v2_ack_pat_pre_detected` (set by the Branch-2 SACK-window probe, :3951); or
(ii) `cmd_compact_confirm_live_accept(sack_window_open, ENABLE!=0)` (:278).
The compact decoder `decode_compact_confirm_from_passband` is reachable ONLY via
`cmd_compact_confirm_crc_valid()` (:222) called from `cmd_compact_confirm_live_accept` (:309),
which HARD-RETURNS false at `:287 if(sack_window_open) return false;`.
The Branch-2 SACK-window probe (:3732-4028) decodes ONLY the 13-uncoded ACK+SACK
(`decode_ack_sack_from_passband`, CRC12 over 5-byte `[bsi||bitmap]`, :3788-3851) — it
NEVER calls the compact decoder.

### §10.3 The MATRIX (RSP-sends vs CMD-accepts; the one structural MISMATCH)
ENABLE assumed flipped to 1. SACK default-ON: `disable_sack=false` (arq_common.cc:865),
`sack_enabled=!disable_sack` (arq_commander.cc:6625), `axis3_sack_mode=SACK_MODE_ON`
(arq_common.cc:779,7088), `sack_v2_enabled` negotiated true on WB.

| batch | SACK | clean/part | config | RSP sends            | CMD accepts via                         | MATCH |
|-------|------|------------|--------|----------------------|-----------------------------------------|-------|
| 1     | n/a  | clean      | ROBUST | bare (M=8, no compact)| Branch3 bare arm (:321)                 | OK (no compact) |
| 1     | n/a  | clean      | WB     | COMPACT (2576)       | Branch3 :309->:314 (SACK-closed)        | OK — compact LANDS (timing-fragile, §10.4) |
| >1    | on   | clean(all) | WB     | COMPACT (2576)       | Branch2 13-uncoded CRC FAIL :3841; Branch3 bails :287; Branch5 never | **MISMATCH — DROPPED** |
| >1    | on   | clean      | NB     | bare (M=8)           | OFDM/bare (suffix gate false)           | OK (no compact) |
| >1    | on   | partial    | WB     | MFSK-suffix / SACK_RSP| Branch2 partial -> sack_detected :3978  | OK (compact never partial) |
| >1    | on   | clean      | WB,cum | MFSK-suffix (2572)   | Branch2 clean -> v2_pre_detected :3951   | OK (compact suppressed) |

### §10.4 The structural MISMATCH (batch>1, SACK-on, clean, WB)
`sack_window_open==true` for any b>1 SACK session once the poll clock passes
`ack_pattern_time_ms` (:3713). Branch-2 is the ONLY decoder in the window; it reads the
compact tail AS a 13-uncoded `[bsi||bitmap]` frame -> compact's CRC12 (over `[bsi]` only)
fails the 5-byte CRC12 at :3841 -> dropped. Branch-3 bails at :287 before its compact
decode (:309). Branch-5 fires on neither route. CMD waits out `receiving_timeout` and
retransmits. The non-cross-validation is BY DESIGN (the I3 false-confirm invariant, §6) —
the same property that blocks false-confirm also means the SACK probe cannot recover a
compact frame. batch==1 WB (Row 2) inherits this whenever the window opens before the
compact poll (timing-dependent, not structural).

### §10.5 The fix landscape (do not ship blindly — §5 entanglement)
- fix-a (LOW): scope RSP compact-send to `data_batch_size==1` WB (add a 5th condition at
  2571-2574). Removes the Row-3 mismatch; b>1-clean reverts to the proven 13-uncoded
  suffix. RSP-only, no SACK touch. Ships the codec ON where it lands.
- fix-b (MED, entangles P2a): give the compact an accept entry point INSIDE the SACK
  window — either insert `cmd_compact_confirm_crc_valid()` in Branch-2 (~:3754) or relax
  the :287 bail for the compact arm (the CRC12-over-[bsi] is the false-confirm guard, NOT
  the SACK window; :287 exists for the CRC-LESS bare arm, :283-288). This rides the
  SACK-Design-A acceptance surface, which is wrong-sign on noisy (BREAK-spiral,
  [[faithful_revalidation_overturn_map]] P2a) — co-rework with P2a, and with lever-C
  (tone-burst) which needs the SAME surface.
- INVARIANT to re-verify on fix-b (CLAUDE.md §5): Branch-2 clean/partial routing
  (:3941-4010), `v2_ack_pat_pre_detected` consumer (Branch-5, :4448), the dedupe
  clean-vs-partial split (:3937-3939), and the ring-advance contract
  (`commit_ack_pattern_consumed` vs `frames_to_read=0` at :4071) — so a compact accept in
  the window does not double-advance or collide with the OFDM dispatch bookkeeping.

### §10.6 ROBUST reach [?]
The compact STRUCTURALLY cannot fire on ROBUST (M=8, mfsk.h:279) — the dominant
batch==1 / climb-stalled traffic ([[faithful_beat_vara_climb_binding]]). Shortening the
reverse turnaround there needs a robust-tier (M=8) compact codeword, a SEPARATE lever
from the WB M=16 compact audited here. The fix-a/fix-b scope does NOT touch the
climb-binding traffic.

## §11 fix-b SHIPPED — compact accept INSIDE the SACK window (2026-06-27)

fix-b (§10.5) implemented and tested. The Row-C structural MISMATCH (§10.4) is removed:
the commander now accepts the compact confirm INSIDE the SACK window for batch>1-WB-clean.

### §11.1 The change (root cause, not a band-aid)
NEW shared predicate `cl_arq_controller::cmd_compact_confirm_sack_window_accept(bool
compact_enabled, bool* out_pre_detected)` (arq_commander.cc, defined right after
cmd_compact_confirm_crc_valid). The Branch-2 SACK-window probe
(`process_messages_rx_acks_data`, inside `if(ack_sack_suffix_len() > 0)`) calls it FIRST,
BEFORE the 13-uncoded `decode_ack_sack_from_passband`:
```
if(cmd_compact_confirm_sack_window_accept(ARQ_COMPACT_CONFIRM_ENABLE != 0,
                                          &v2_ack_pat_pre_detected))
    mfsk_handled_this_poll = true;
if(!mfsk_handled_this_poll) { ...the UNCHANGED 13-uncoded decode + OFDM dispatch... }
```
The predicate runs the SELF-VALIDATING compact decode `cmd_compact_confirm_crc_valid()`
(its own tail snapshot + 16-sym base detect + GF(16) K=5 soft-decode + CRC12-over-[bsi]
+ bsi-in-window). On a valid compact CRC it routes through the EXACT 13-uncoded CLEAN
branch state (split-dedupe via `sack_clean_confirmation_accepted`, `cmd_last_applied_clean_bsi`,
`inband_retag_confirm_from_sack`, and sets the caller's `v2_ack_pat_pre_detected` LOCAL
through the out-param). `v2_ack_pat_pre_detected` routes to the proven CLEAN funnel (the
"Data ACK pattern detected" else-if) — NO `commit_ack_pattern_consumed()`/frames_to_read
mutation here (the v2 clean path does not advance the ring that way; the skipped OFDM
dispatch owns `frames_to_read=0`), so NO double-advance vs the Branch-3 arm.

`cmd_compact_confirm_crc_valid()` gained an optional `uint8_t* out_bsi` (NULL default) so
the Branch-2 caller dedupes/re-tags on the decoded bsi; the SACK-closed Branch-3 caller is
byte-identical (passes NULL).

### §11.2 The :287 bail is UNCHANGED (Branch-3 / SACK-closed arm preserved)
The §10.2 hard-return `cmd_compact_confirm_live_accept: if(sack_window_open) return false`
is LEFT IN PLACE — it correctly bails the SACK-CLOSED-arm code path (where the lenient bare
match false-fires on OFDM SACK_RSP body audio). The in-window compact accept now lives in
its OWN Branch-2 entry point that does NOT use the bare-match arm. The CRC12-over-[bsi] (not
the SACK window) is the false-confirm guard for both arms — this realizes the §6 I3/I5
invariant (compact-for-clean, SACK-for-partial) WITHOUT weakening it.

### §11.3 §5 cross-layer audit (the two consumers + the routing/dedupe contract)
- **Consumer 1: the CLEAN funnel** (`v2_ack_pat_pre_detected` -> the "Data ACK pattern
  detected" else-if). It expects a CLEAN, all-ones, in-window, deduped batch. The compact
  confirm IS all-ones by type (RSP emits it only for a fully-received batch), the bsi-in-
  window gate is inside cmd_compact_confirm_crc_valid, and the dedupe mirrors the 13-uncoded
  clean branch -> the funnel's preconditions are met identically. VERIFIED: no double-count
  of `nBatches_fully_acked` on a repeated compact (test §(a)).
- **Consumer 2: the PARTIAL SACK path** (`sack_detected` -> the retransmit-queue builder).
  UNTOUCHED. The compact decode REJECTS a SACK partial frame (different field layout/length)
  so a partial NEVER routes through the compact entry point; on a compact MISS the code falls
  straight through to the UNCHANGED 13-uncoded decode that feeds both clean and partial. The
  Branch-2 clean/partial split (:is_clean_confirmation), `v2_ack_pat_pre_detected` consumer
  (the CLEAN else-if), and the ring-advance contract (`frames_to_read=0` at the OFDM dispatch)
  are unchanged on every non-compact path.
- **No-cross-validate (I3):** the compact CRC12-over-[bsi] cannot validate a 13-uncoded
  [bsi||bitmap] suffix and vice-versa -> the two transports never steal each other's frames
  (test "no cross-validate"). FAR unchanged.

### §11.4 Tests (FAIL-BEFORE / PASS-AFTER, CLAUDE.md §3)
`test_compact_confirm_sack_window_rx_path` (test_compact_confirm_rx.cc, in `--test` and
`--test-compact-confirm-rx`). Drives the SHARED Branch-2 predicate for batch>1 (data_batch_size=8),
SACK-on, clean, WB.
- FAIL-BEFORE (`MERCURY_COMPACT_SACK_WINDOW_FAILBEFORE=1`, same binary): replays the pre-fix
  Branch-2 (only `decode_ack_sack_from_passband` runs on the compact tail) -> 5-byte CRC12
  fail -> the compact confirm is REJECTED -> rc=1 (the §10.4 dropped-confirm bug).
- PASS-AFTER (default): the decoupled compact decode ACCEPTS it (returns true, sets
  v2_ack_pat_pre_detected) -> rc=0. Plus: a duplicate is consumed but NOT re-credited; a
  corrupted-suffix compact is REJECTED (GF16+CRC12 guard); a 13-uncoded ACK+SACK is NOT
  cross-accepted as compact (falls to the legacy path, which still decodes it); out-of-window
  bsi REJECTED; gate-off (compact_enabled=false) returns false (byte-identical pre-fix path).
Full `--test` suite: 67 passed, 0 failed (sim_clock 0 failed, Winlink dict 12/0).

### §11.5 Status
fix-b SHIPPED on `staging/short-coded-confirm`. Still gated OFF by ARQ_COMPACT_CONFIRM_ENABLE
(held off pending the faithful real-audio batch>1-WB re-verify) — byte-identical to the
pre-fix path when 0. The §10.4 4.7x batch>1-WB regression at ENABLE=1 is the bug this removes;
faithful-sim ENABLE=1 batch>1-WB A/B is the next gate before flipping the master enable.

## §12 fix-b FAITHFUL RE-VERIFY — the 4.7x regression is GONE (2026-06-27)

The §11.5 gate is CLEARED. Faithful real-audio A/B at HEAD 6bcc3964 on the EXACT
batch>1-WB-SACK scenario that regressed 4.7x at f1da9cbe (§10.4).

### §12.1 Builds (both from 6bcc3964)
- e0 (ENABLE=0, base): md5 `97fc94d4750b2fe6025868d92496bdf7`
- e1 (ENABLE=1, fix):  md5 `a62ec44ed81facb6692e4b29c07a1756` (via `EXTRA_CFLAGS_ENV=-DARQ_COMPACT_CONFIRM_ENABLE=1`)
- md5s DIFFER; `.note.gnu.build-id`=6bcc3964 in both; distinct from the prior f1da9cbe pair (9835c9a7/992ff1ce).
- `--test`: 49 [OK] / 0 FAIL both arms. `--test-compact-confirm-rx`: PASS-AFTER rc=0; FAIL-BEFORE
  (`MERCURY_COMPACT_SACK_WINDOW_FAILBEFORE=1`) rc=1; `compact_confirm_sack_window_rx_path` PASS (0 fail).

### §12.2 Result (cfg8-PINNED WB, payload 24576 -> batch>1, SACK Design A on; N=3 pairs x 2 boxes + N=4 focused-partial)
| scenario | e1 rx | e0 rx | e1 breaks | e1 [CMD-COMPACT-CONFIRM] in-SACK-window accepts | false-confirm |
|---|---|---|---|---|---|
| clean .31/.11    | 12730/12730 | 12640/12506 | 0/0 | 7.0/7.0 per cell | 0 |
| revworse .31/.11 | 12640/11970 | 12596/12104 | 0/0 | 6.0/4.0 per cell | 0 |
| partial snr27 .31| 11708       | 9832        | 0   | 3.0 (+SACK part 4.5) | 0 |

- **compact ACCEPTED inside the SACK window**: `[CMD-COMPACT-CONFIRM] CLEAN (in-SACK-window)` fires
  4-7/clean cell on BOTH boxes — was **0** at f1da9cbe. `compact_accepted_any=True` (e1), False (e0).
- **4.7x regression GONE**: e1≈12700 vs e0≈12600 = parity (e1>=e0 on clean), breaks 0 (was 3.0/cell).
- **partial-SACK intact**: SACK PARTIAL markers both arms; e1 MIXES compact (clean sub-batch) + SACK
  retx (partial sub-batch) in one session; compact REJECTS partial frames (no cross-validate); e1 rx>=e0.
- **false-confirm = 0** everywhere. **reverse airtime SHORTER**: compact wire ≈977-992ms vs
  MFSK-suffix ≈1047-1066ms (~70ms), and now accepted live. **revworse**: parity, breaks 0, still accepted.

### §12.3 Status
PASS. ARQ_COMPACT_CONFIRM_ENABLE=1 is clear to flip default-ON for the WB M=16 reverse confirm
(per CLAUDE.md "don't leave a proven fix default-off"). Artifacts: `_research/coded_confirm/
{reverify_fixb.json, REVERIFY_FIXB.md, reverify_raw_{31,11}.json, partial_only_snr27.json}` +
fleet `/home/kameron/optBfixb/`. ROBUST/NB M=8 stays out of scope (compact is WB-only, §10.6).

## §13 FLIPPED default-ON + MERGED + PUSHED (2026-06-27)
All §12 JUDGE conditions YES, so the gate shipped default-ON.
- `include/common/common_defines.h:91` `#define ARQ_COMPACT_CONFIRM_ENABLE 1` (was 0).
- Flip commit `ebc3ea84` ("arq: flip compact coded reverse-confirm (Option B) default-ON for WB
  M=16") on `staging/short-coded-confirm`.
- Default-ON build verified locally (fleet unreachable this session): `bash build.sh o3` ->
  installed; `--test` 49/0; `--test-compact-confirm-rx` 9/9 PASS rc=0 on the DEFAULT-ON binary
  (in-SACK-window accept, fresh-CLEAN->clean-funnel, dup-not-re-credited, no-cross-validate,
  legacy 13-uncoded still decodes, corrupted-suffix REJECTED, out-of-window REJECTED, gate-off
  byte-identical).
- `monitor` fast-forwarded to `ebc3ea84` (origin/monitor `d57c9afb` was a strict ANCESTOR of
  staging — 75 ahead / 0 behind -> clean FF, no merge commit). The FF was done in the
  `merge-followgate` worktree (monitor was checked out there) to avoid disrupting any worktree.
- Pushed: `origin/monitor d57c9afb..ebc3ea84` (verified). Zero attribution.
- Result file: `_research/coded_confirm/FLIP_RESULT_v4.md`.
- Out of scope (unchanged): ROBUST/NB M=8 return-path (compact is WB-only); FORGIVING-ACK
  cumulative path (responder gates compact OFF when cumulative_ack_enabled).

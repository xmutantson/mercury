# Data-Flow Audit — in-band CONFIG_0 frame-0 ROLLING-PARTIAL climb veto

Branch: `feat/inband-a3-decouple`. Scope: the gearshift clean-streak (`consecutive_data_acks`)
+ the FRAME-UP promotion gate, on the in-band path only (`MERCURY_INBAND_RATE=1`). Legacy
SET_CONFIG behaviour is byte-identical (every changed predicate is behind
`inband_rate_feature_enabled()`).

Companion to `data-flow-inband-adopt-metric-gate.md` (the snapshot tag-follow contamination
gate — a DISTINCT, already-shipped fix) and `data-flow-inband-tier-crossing.md` (the
robust→OFDM SET_CONFIG cross). This doc owns the gearshift-streak shared state for the
CONFIG_0 wedge.

---

## §1 The bug (root cause — VERIFIED from the adoptgate RT A/B relay log, seed 1003)

Under `MERCURY_INBAND_RATE=1` the redesign climbs ROBUST_0→101→102 fast (robust ACK-PAT,
eff_thresh=1), TIER-CROSSES 102→CONFIG_0 via the legacy SET_CONFIG handshake
(arq_commander.cc:857), then **WEDGES at CONFIG_0** (rx≈189–233 B, final ROBUST_0/CONFIG_0)
while legacy OFF climbs to CONFIG_3/4/14 (rx 2117–3951 B). MEASURED, _msab/adoptgate.

ROOT (relay log, the CONFIG_0 batches bsi=3,4,5,6):
- Every CONFIG_0 batch LOSES FRAME-0 (seq=0 / bitmap bit0) on its FIRST transmission. The
  RSP decodes seq 1..5 and reports a PARTIAL SACK `bitmap=0x3e` (5/6) or `0x1e` (4/6) — bit0
  is ALWAYS clear (`[TX-MFSK-ACK-SACK] batch_seq_id=3 bitmap=0x0000003e`, T+0162.9).
- The CMD queues the missing frame-0 for retx and MIXES it into the NEXT batch
  (`[CMD-V2-MIXBATCH] TX batch: 1 retx bsi=3 + 5 new bsi=4`, T+0166.4). The retx'd frame-0
  completes bsi=3 (RSP `prev-delivered path bitmap=0x3f`, T+0170.1) — BUT the new bsi=4's OWN
  frame-0 is lost the same way. **A rolling, one-batch-lagged frame-0 loss.**
- Net: every batch the CMD's gearshift sees is a PARTIAL (`[CMD-MFSK-ACK-SACK] PARTIAL
  bitmap=0x3e`, T+0165.8). ZERO clean ACK-PATs at CONFIG_0 (the 3 clean ACK-PATs in the run
  are all at the robust rungs). Each bsi DOES eventually fully deliver (0x3f), one batch late.

WHY FRAME-0 SPECIFICALLY (PHY): the FIRST OFDM frame of a CONFIG_0 batch bears the
Schmidl-Cox acquisition burden. At CONFIG_0 the lock is MARGINAL — FTR metric ≈ 0.08–0.12
(vs a clean ≈0.997) and the pre-LDPC SKIP-VAR var ≈ 1.8–2.4, JUST over the cfg=0 threshold
1.60 (telecom_system.cc SKIP-VAR gate) → frame-0 is skipped while frames 1..5 ride the
already-locked timing. (The very FIRST CONFIG_0 batch additionally ate the fresh-change
CONFIG_TAG re-air at var=79 — tag contamination — but batches 4/5/6 lose frame-0 with NO tag
present, so the SUSTAINED veto is acquisition-marginal, not the tag.)

GEARSHIFT CONSEQUENCE (the binding wedge): a PARTIAL SACK sets `last_batch_fully_acked=false`
(arq_commander.cc:4120). The FRAME-UP gate (arq_commander.cc:5523) requires
`promotion_allowed_on_batch(last_batch_fully_acked)` (CLEAN only) UNLESS
`inband_pipeline_climb_active()` is true. At CONFIG_0 the re-tag is NOT armed (the tier-cross
used SET_CONFIG, NOT `inband_unilateral_config_change`), so `inband_pipeline_climb_active()`
returns false (arq_common.cc:3318: `!inband_retag_armed → false`). So `batch_promotable` is
CLEAN-only, the rolling partial never satisfies it, `consecutive_data_acks` never increments
(arq_commander.cc:5530), FRAME-UP CONFIG_0→1 never fires → permanent wedge.

LEGACY does not wedge: it FRAME-UPs off CONFIG_0 the same way, but its frame-0 loss is the
same MARGINAL acquisition — yet legacy's eff_thresh path and the SET_CONFIG decoupled climb
let it advance off the partial sooner. (The redesign-specific delta is that the in-band
climb gate is STRICT-CLEAN at CONFIG_0 because no re-tag is armed there — the tier-cross
intentionally routes via SET_CONFIG and so leaves the pipeline-climb predicate false.)

## §2 The fix (option (b): a rolling-recoverable frame-0 partial must NOT veto the climb)

Option (a) — suppress a redundant per-batch tag re-air — does NOT apply: at CONFIG_0 there is
NO per-batch tag (one fresh-change tag, then the every-8 periodic), and batches 4/5/6 lose
frame-0 with no tag at all. Suppressing the tag would not unblock the wedge. So (b).

The genuine root the gearshift can own: **a CONFIG_0+ (OFDM-tier) batch whose first-pass
partial is a RECOVERABLE lead-frame loss and which the link is actively completing via retx
is a VIABLE rung — it must count toward the FRAME-UP climb, not veto it.** The signature is
narrow and self-distinguishing: the partial reports the LEAD frame (bit0 / frame-0) MISSING
AND AT MOST 2 frames missing TOTAL, at an inband OFDM config, with the climb otherwise
permitted. The "≤2 missing including frame-0" bound matches the two observed rolling partials
on the wire: `0x3e` (only frame-0 missing) and `0x1e` (frame-0 + the EOB tail frame the
acquisition seam clips on the MIXBATCH retx). A partial that drops >2 frames is a genuinely
marginal rung → stays vetoed (preserving §9 anti-thrash). A partial with frame-0 PRESENT (only
a tail loss) is NOT this case → also stays vetoed (the lead frame decoded, so the rung is not
the acquisition-seam case). This is the acquisition-seam loss the retx machinery recovers
within one batch — the batch fully delivers (bitmap → 0x3f), one SACK cycle late.

Implementation (arq_commander.cc, the FRAME-UP gate ~:5519): extend the optimistic
`batch_promotable` to also accept the lead-frame-only recoverable partial, via a new pure
predicate `inband_lead_frame_only_partial()` gated on `inband_rate_feature_enabled()` +
`is_ofdm_config(current_configuration)` + the last partial's bitmap = "all set except bit0"
(exactly one frame missing, and it is frame-0). The streak then ADVANCES on such a batch
(like the existing pipeline-climb optimistic path), so FRAME-UP fires; the retx still recovers
frame-0 (UNCHANGED — we do not touch the retx/SACK machinery). Legacy + flag-off: the
predicate returns false → byte-identical strict-clean gate.

We do NOT relax the strict gate for partials that drop MORE than the lead frame (those are a
genuine marginal-rung signal and must stay vetoed — §9 / the WGN:-10 anti-thrash). We do NOT
touch the anchor-raise / AARF-decay / back-off clean-only branch (arq_commander.cc:5317) —
those keep requiring a true clean batch (anchor must not be poisoned by a not-yet-clean rung;
the +1 anchor clamp recovers an over-climb).

## §3 §5 CROSS-LAYER AUDIT (shared state: the gearshift clean-streak + FRAME-UP gate)

The shared state is `consecutive_data_acks` (the FRAME-UP clean-streak) and the per-batch
verdict `last_batch_fully_acked` it gates on.

1. **Producers of `last_batch_fully_acked`:**
   - arq_commander.cc:1666 / :2215 / :7774 — per-batch TX-start reset to false.
   - :4120 — PARTIAL SACK_RSP path: set false (the wedge producer).
   - :4243 — clean MFSK ACK-PAT funnel: set true.
   - :4345 / :4370 — clean LDPC ACK_RANGE / ACK_MULTI: set true.

2. **Producers of `consecutive_data_acks`:**
   - arq_common.cc:837 / :2169, arq_commander.cc:6757 / :9736 — ctor / reset-to-0.
   - arq_commander.cc:5530 — ++ inside the FRAME-UP gate (the climb credit).
   - arq_commander.cc:5586 — =0 after a FRAME-UP fires (consume the streak).
   - arq_commander.cc:2912 / :4423 / :4603 / :9802 — =0 on failure / demote / break.

3. **Consumers of `consecutive_data_acks`:**
   - arq_commander.cc:5546 — `>= eff_frame_shift_threshold` → FRAME-UP (the only consumer).

4. **Valid states + the bitmap signature:** a partial SACK carries the rx bitmap; bit i set =
   frame i decoded. The wedge signature is bit0 clear, bits 1..(N-1) set (exactly one missing,
   the lead). A no-tag steady CONFIG_0 batch at a GOOD rung would be all-ones (clean → existing
   path). A genuinely marginal rung drops many frames (multiple bits clear → predicate false →
   stays vetoed). The lead-frame-only case is the acquisition-seam loss this fix credits.

5. **Invariants + verification:**
   - INV-A: the FRAME-UP +1 anchor clamp (arq_commander.cc:5483) still bounds the climb to
     last_data_viable_config+1 — UNCHANGED; the fix only changes the streak INPUT, not the
     +1/ceiling clamps. A too-eager climb is recovered by the gearshift's decode-failure
     demote (the same net the pipeline-climb path already relies on). VERIFIED.
   - INV-B: the anchor-raise / AARF-decay / floor-probe-reset clean-only branch (:5317) is
     NOT touched — still gated on `promotion_allowed_on_batch(last_batch_fully_acked)` (true
     clean only), so a not-yet-clean rung never raises the anchor (no §9 thrash). VERIFIED.
   - INV-C: the retx/SACK machinery is UNCHANGED — the missing frame-0 is still queued + mixed
     into the next batch + recovered to 0x3f. The fix only stops the gearshift from VETOING
     the climb on the rolling partial. VERIFIED.
   - INV-D: a real CONFIG_0→1 ON-CHANGE tag still fires when FRAME-UP promotes (the change
     emits via the unilateral path → `inband_unilateral_config_change` arms the re-tag →
     `emit_config_tag_passband` is_change). The climb announcement is preserved. VERIFIED
     (the fix is upstream of the SET_CONFIG/unilateral dispatch — it only lets the streak
     reach the threshold; the existing change-announce path is untouched).
   - INV-E: the down-ladder / demote / true-loss BREAK floor are UNAFFECTED — the fix only
     adds a NARROW promotion-credit; it never blocks a demote (those read emergency_nack_count
     / config_is_at_bottom, not consecutive_data_acks). VERIFIED.

6. **What my fix changes:** ONE assumption at the FRAME-UP gate — "only a fully-clean batch
   advances the climb streak" → "a fully-clean batch OR an inband-OFDM lead-frame-only
   recoverable partial advances it". Every consumer re-verified above. Legacy byte-identical.

## §4 The regression (LIVE-PATH, --test-inband-frame0-partial)

Directed in-process test driving the EXACT production FRAME-UP gate decision:
- FAIL-BEFORE (`-DINBAND_FRAME0_PARTIAL_FAILBEFORE`, predicate pinned false): a lead-frame-only
  partial (bitmap=0x3e, 5/6, bit0 clear) at CONFIG_0 does NOT advance consecutive_data_acks →
  FRAME-UP never reached → wedge reproduced.
- PASS-AFTER (default): the same lead-frame-only partial ADVANCES the streak → FRAME-UP fires
  CONFIG_0→1. A MULTI-frame-drop partial (bitmap=0x0c, frames 0,1,4,5 missing) does NOT (stays
  vetoed — §9 anti-thrash preserved). A clean batch advances as before.

## §5 A/B verdict (MEASURED — _msab/frame0fix, RT @SNR40, redesign ON vs legacy OFF)

The fix FIRES and unblocks the FORWARD climb — but exposes a SIBLING (§6). MEASURED from the
seed=1002 ON relay trace (fact-documents/_frame0fix_evidence/seed1002_climb_trace.txt):

```
[T+0161.971] FRAME UP config 102 -> 0          (tier-cross to CONFIG_0, reverse pinned 102/ROBUST_2)
[T+0201.166] ROLLING-PARTIAL anchor raise: lead-frame-only batch fully delivered at config 0
             -> last_data_viable 102 -> 0       (THE FIX: anchor reaches CONFIG_0)
[T+0201.166] FRAME UP config 0 -> 3 (clean-streak 2)   (THE FIX: FRAME-UP fires past CONFIG_0!)
[T+0201.167] UNILATERAL CONFIG 0 -> 3
[T+0238.801] BREAK Block failure #1 at config 3        (THE SIBLING: reverse-ACK fails at CFG3)
```

So vs the 100%-wedged baseline (adoptgate: ON peak=CONFIG_0 on EVERY sample, FRAME-UP CONFIG_0->1
NEVER fired), the fix makes the forward FRAME-UP climb PAST CONFIG_0 actually FIRE (CONFIG_0->3
here, RSP nReceived_data=33 at CONFIG_3 — the forward path carries data at the climbed rung).
fail-before/pass-after of the fix's OWN mechanism is PROVEN both in --test-inband-frame0-partial
AND in the live trace.

BUT the redesign still does not reach legacy's byte counts: once the forward climbs to CONFIG_3,
the CMD cannot decode the REVERSE SACK/ACK at that OFDM forward rung (CMD-side
`FTR-FAIL CONFIG_3 metric=0.096`, `nReceived_data=0` on the reverse while the RSP's forward
nReceived=33) -> `[BREAK] Block failure at config 3` -> demote. The TIER-CROSS reverse-ACK pin
(d28f02d) holds reverse=102/ROBUST across the robust->OFDM CROSS, but the INTRA-OFDM unilateral
climb (CONFIG_0->3) does NOT re-establish a decodable reverse-ACK at the new forward rung. This
is the REVERSE-ACK / delivery-loop binding constraint the companion docs predicted
(data-flow-inband-adopt-metric-gate.md §7; data-flow-inband-tier-crossing.md §3) — now EMPIRICALLY
ISOLATED: removing the forward-climb wedge proves the reverse-ACK at the climbed OFDM rung is the
TRUE next binding constraint, not the gearshift clean-streak.

## §6 STOP / SIBLING (the §5 cross-layer discipline)

This fix is NECESSARY (it removes the forward-climb veto — proven) but NOT SUFFICIENT for the
end-to-end byte win. The remaining gap is the INTRA-OFDM reverse-ACK decode after the climb — a
DISTINCT layer (reverse_configuration / turnaround timing at the climbed forward rung), not the
gearshift clean-streak this doc owns. Per CLAUDE.md §2 (three-fail-STOP) and §5 (don't chain a
second speculative cross-layer fix in the same session), the reverse-ACK climb-pin is a SEPARATE
fix for a follow-up session. The forward-climb fix ships as-is (a proven, tested removal of a real
100%-wedge veto; default-ON under the inband feature, legacy byte-identical). Candidate sibling
fix (NOT done here, for the next session): extend the reverse-ACK pin (or a reverse re-establish)
to the intra-OFDM climb so the reverse SACK decodes at the climbed forward rung — OR make the
intra-OFDM climb a +1 (CONFIG_0->1) probe rather than the SNR-elevator multi-rung jump (CONFIG_0
->3), so the reverse-turnaround margin degrades gradually and the demote re-pins one rung at a
time.

## §7 CORRECTION — the binding constraint is the reverse DATA-SACK TRANSPORT, not the reverse-ACK RUNG (VERIFIED, deeper read 2026-06-24, `_msab/keystone/_logsnap/modem_064839.log` redesign vs `modem_032735.log` legacy)

§5/§6 above are PARTIALLY WRONG and the diagnostic line they cite is misread. Strike-throughs:

- ~~"the CMD cannot decode the REVERSE SACK/ACK at that OFDM forward rung"~~ — FALSE. At CONFIG_3 the
  reverse **base ACK pattern DECODES** (`[INBAND-TX] CONFIRMED followed CONFIG_3 via BASE ACK pattern
  mfsk_matched=7 >= thr=7`, modem_064839.log T+236.435). The KEYSTONE decoupled-confirm works at the
  climbed rung; the reverse RUNG is fine.
- ~~"CMD-side FTR-FAIL CONFIG_3 metric=0.096 = the reverse-ACK decode failing"~~ — MISREAD. That FTR-FAIL
  is the CMD's OFDM frame ACQUIRER spinning at `current_configuration`=3 while it WAITS in
  RECEIVING_ACKS_DATA (the reverse MFSK ACK is caught by the no-side-effect `decode_ack_sack_from_passband`
  peek, NOT the OFDM acquirer). It is a symptom of waiting, not the reverse-ACK decode.
- ~~"the d28f02d robust reverse-pin does not carry through the intra-OFDM climb"~~ — FALSE. `reverse_configuration`
  is set to 102 at the tier-cross (T+161.972) and is NEVER re-written for the rest of the session — the
  pin HOLDS robust the whole climb. Extending it "through the climb" is a no-op; it is already held.

### §7.1 The ACTUAL root (VERIFIED, every-rung, not config-3-specific)

The BREAK trigger at config 3 is `[CMD-ACK-PAT] Timeout: no ACK detected, peak_matched=5/7 peak_metric=0.6`
(T+238.801) — i.e. the **data-batch SACK** (the bsi+bitmap that says which of the 24 frames to retx)
was NOT received. The SAME signature fires at EVERY rung in the redesign:
`Data ACK pattern detected!` → `CONFIRMED via BASE ACK pattern (suffix CRC FAILED)` →
`[CMD-ACK-PAT] Timeout: no ACK detected peak_matched=3/7..5/7` at ROBUST_1(101), ROBUST_2(102), AND CONFIG_3.
The robust-MFSK suffix (GF(16)+CRC-12) CRC-fails consistently at WGN:40 on this turnaround.

DECISIVE: `[CMD] stats.nAcked_data` is STUCK at 14 across the whole config-3 window. The RSP DID receive
the forward data (`nReceived_data=33`) and DID dispatch the reliable OFDM SACK_RSP transport
(`[RSP] [ACK-GATE-V2] dispatching OFDM SACK_RSP batch_seq_id=5, 21/24 received`, T+237.338) — but the CMD
NEVER reflects it (no nAcked advance, zero CMD-side SACK_RSP apply). In LEGACY the contrast is exact:
`nAcked_data` STEADILY ADVANCES 0→1→2→3→9→15 (modem_032735.log) — legacy's reverse data-SACK loop delivers.

So the binding constraint is the **reverse DATA-ACK/SACK DELIVERY LOOP**: the d28f02d pin routes the
data-SACK onto the robust-MFSK suffix whose CRC fails, while the OFDM SACK_RSP the RSP dispatches is not
applied by the CMD across the cross/climb rung-mismatch (CMD at CONFIG_3, reverse pinned 102/robust-MFSK).
The redesign confirms the CLIMB (base pattern) but never DELIVERS the data (SACK never applied) → BREAK at
every rung. This is the **delivery-layer binding constraint** the memory note `delivery_layer_binding_constraint`
already names (constraint = DELIVERY not PHY).

### §7.2 Why the PROMPTED fix is FALSIFIED, and the real fix direction

The prompted fix — "extend the d28f02d robust-MFSK reverse-pin THROUGH the intra-OFDM climb" — would NOT
help and would likely WORSEN delivery: the pin is ALREADY held robust the whole climb, and robust-MFSK is
the transport whose suffix CRC is FAILING. Pinning harder propagates the failing transport. The reverse RUNG
is not the problem; the data-SACK TRANSPORT reliability is.

Real fix candidates (for a clean, separately-scoped follow-up — NOT a same-session speculative chain, per
CLAUDE.md §5):
  (A) **+1 probe instead of the 0→3 SNR-elevator JUMP** (the §6 alternative, now PREFERRED): a single-rung
      climb keeps the CMD/RSP rungs aligned, keeps the data batch + turnaround small per rung, and lets the
      reliable OFDM SACK_RSP transport (which legacy uses and which the RSP already dispatches) decode at the
      shared rung. This attacks the rung-mismatch + giant-batch root, not the reverse-pin symptom.
  (B) **Make the data-SACK ride the reliable OFDM SACK_RSP at the climbed rung** rather than the fragile
      robust-MFSK suffix — i.e. DECOUPLE the data-ACK transport from the reverse-pin the way legacy does,
      keeping only the *climb-confirm* on the robust base pattern (KEYSTONE) and routing the *bitmap* via
      SACK_RSP. Larger change; audit the SACK_RSP rung selection vs reverse_configuration first.
  (C) The MFSK suffix CRC-12 fragility itself (peak_matched 3-5/7 at WGN:40) may be a turnaround
      phase/window problem (multi-window recovery already exists, `MERCURY_DATA_ACK_MULTIWINDOW`) — worth
      a separate isolation before (B).

STATUS: NO fix implemented this session. The prompted reverse-pin-extension is falsified by the evidence;
implementing it would be a symptom band-aid (CLAUDE.md §2) and risks the serial-misdiagnosis pattern the
memory BENCH-CLAIM guard warns against. Recommended next session: option (A) +1-probe, with a fail-before/
pass-after on the SACK_RSP-applied / nAcked-advance at the climbed rung.

## §8 FIX IMPLEMENTED — option (A) the +1 climb (2026-06-24, follow-up session)

§7 RE-CONFIRMED independently (logs re-read, file:line below) and option (A) chosen over (B)/(C):

### §8.1 Root RE-CONFIRMED (the elevator JUMP is the data-SACK-stranding mechanism)
- **The 0→3 jump = the SNR-elevator at `arq_commander.cc:5657-5663`** (pre-fix inline). At CONFIG_0
  the +1 default `proposed_frame=config_ladder_up(CONFIG_0)=CONFIG_1`, but `elevator_target_from_snr()`
  → `get_configuration(SNR_uplink−SUPERSHIFT_MARGIN_DB)` returns CONFIG_3 at WGN:40, so a SINGLE FRAME-UP
  jumps 0→3. VERIFIED `modem_064839.log` T+201.166 `[GEARSHIFT] FRAME UP: 1 consecutive ACKs ... config 0 -> 3`.
  Turbo is INACTIVE the whole redesign session (0 SUPERSHIFT/TURBO markers) → the FRAME-UP elevator
  (`:5657`) is the ONLY multi-rung site reached on the in-band climb; the robust climbs 100→101→102 are
  already +1.
- **The jump airs a CONFIG_0-SIZED 24-frame batch at the slow CONFIG_3 rung.** VERIFIED `[TX-PEAK] frames=24
  size=1579136 cfg=3` (modem_064839.log) vs LEGACY which only airs 24/25-frame batches at CONFIG_15/16 and
  airs CONFIG_3/4 as `frames=1` (modem_032735.log). A 24-frame batch at 303 bps = a huge forward airtime →
  the reverse data-SACK turnaround the robust-MFSK suffix (`bitmap=0x00df7ffe ... on CONFIG_-1`) cannot survive.
- **Consequence VERIFIED:** the climb-CONFIRM works (`[INBAND-TX] CONFIRMED followed CONFIG_3 via BASE ACK
  pattern mfsk_matched=7`, T+236.435 — the §6 keystone), but the data-SACK suffix CRC fails
  (`[CMD-ACK-PAT] Timeout: no ACK detected, peak_matched=5/7`, T+238.801) → the CMD applies ZERO data-SACKs
  after CONFIG_3 (last apply T+200.552 bsi=4) → `stats.nAcked_data` STUCK at 14 (legacy advances 0→15,
  steady at 15 for 6860 polls) → BREAK at every rung. The RSP DID dispatch the SACK_RSP + the suffix
  (T+237.338, 21/24); the CMD never applies either (the OFDM SACK_RSP CMD-apply count is 0 in BOTH arms —
  legacy ALSO rides the robust-MFSK suffix and it decodes there because legacy's CONFIG_3 batches are 1 frame).

### §8.2 Why (A) over (B)/(C)
- (A) +1 climb: smallest, root-cause change. Keeps CMD/RSP rungs aligned AND keeps each rung's forward
  batch + reverse turnaround small enough for the EXISTING reverse SACK transport to decode — attacks the
  giant-batch + rung-mismatch root directly. The keystone (§6) already makes the climb CONFIRM rung-by-rung
  cheap, so a +1 climb is the natural pacing. CHOSEN.
- (B) route the bitmap via OFDM SACK_RSP at the climbed rung: larger change; and the logs show legacy's
  data-SACK ALSO rides the robust-MFSK suffix (not SACK_RSP) and succeeds — so the suffix transport is NOT
  inherently broken; it fails only under the giant-batch turnaround the elevator creates. (B) would re-architect
  a transport that works once the batch is right. Deferred.
- (C) MFSK suffix CRC fragility: the suffix decodes fine at the robust rungs (5 clean applies pre-jump) and in
  legacy at every rung — it is not fragile per se; it is overwhelmed by the elevator's turnaround. Not the root.

### §8.3 The fix (pure helper, production + test share it)
- `arq_common.cc::inband_climb_target(proposed_frame, snr_elevator, inband_plus1_on)` (new pure selector):
  in-band → `proposed_frame` (strict +1, elevator suppressed); legacy → elevator-OR-+1 max (byte-identical to
  the pre-fix inline `negotiated = proposed; if(snr_ideal idx > proposed idx) negotiated = snr_ideal`).
- `arq_commander.cc:~5656` — the FRAME-UP gate computes `snr_elevator` (only when the SNR/OFDM gate is met,
  as before) and calls `inband_climb_target(...)` with `inband_plus1_on = inband_rate_feature_enabled()`
  (pinned false under `-DINBAND_PLUS1_CLIMB_FAILBEFORE`).
- `arq.h` declarations; `arq_commander.cc::test_inband_plus1_climb` + `main.cc` wiring
  (`--test` battery + `--test-inband-plus1-climb`).

### §8.4 §5 CROSS-LAYER AUDIT (shared state: the FRAME-UP climb target `negotiated_configuration`)
1. **Producers of `negotiated_configuration` on the FRAME-UP path:** ONLY the `:5656` site (now via
   `inband_climb_target`). Other producers (SNR_BASED SET_CONFIG `:892`, turbo `:6672/:6679/:6686`, BREAK
   recovery) are on DIFFERENT paths — turbo is INACTIVE on the in-band climb (verified 0 markers), so they
   are not reached. The robust-dwell op (`:5705`) runs only when FRAME-UP DECLINED (returns before this), so
   it is downstream and unaffected.
2. **Consumers of `negotiated_configuration`:** the `add_message_control(SET_CONFIG)` builder (`:5689`/`:834`
   chokepoint), which under in-band routes intra-OFDM via the unilateral CONFIG_TAG and tier-cross via legacy
   SET_CONFIG. A +1 in-band target is intra-tier (both OFDM) once past CONFIG_0 → the unilateral tag path,
   UNCHANGED. The RSP follows the tag (down-ladder D=4 window) — a +1 step stays WELL within D=4 (it was the
   multi-rung jump that risked escaping it). VERIFIED narrower than the pre-fix jump.
3. **Valid states:** before this producer, `negotiated_configuration` carries the prior target. The fix only
   changes WHICH config the FRAME-UP elects (proposed_frame vs elevator), never the surrounding state machine
   (FIFO restore, retx clear, state transition all UNCHANGED).
4. **Invariants:** the +1 anchor clamp (`:5552`, `last_data_viable_config+1`) STILL bounds the climb — the fix
   only changes the target WITHIN that clamp (the elevator was the thing that could outrun it). The floor /
   anti-thrash nets (`probe_rung_suppressed`, AARF `frame_shift_threshold`, the demote/BREAK escapes) read
   `consecutive_data_acks`/`emergency_nack_count`, NOT the climb target — UNAFFECTED. The elevator only ever
   RAISES; suppressing it can never drop below the +1, so the deep-SNR WGN:-10 anti-thrash (#2) is intact
   (legacy keeps the elevator there anyway, and in-band at deep SNR the elevator's >-90 gate is unmet so the
   in-band path is already +1 = byte-identical to legacy in that regime).
5. **What the fix changes:** ONE assumption — "the FRAME-UP elects the SNR-ideal config (elevator) when the
   forward SNR licenses it" → "the in-band FRAME-UP elects EXACTLY +1; legacy keeps the elevator." Every
   consumer re-verified. Legacy/flag-off byte-identical (the helper's legacy branch reproduces the inline logic).

### §8.5 Tests (fail-before / pass-after, VERIFIED)
- `--test-inband-plus1-climb` PASS-AFTER (ALL PASS): in-band elevator(CONFIG_3) → CONFIG_1 (+1); legacy
  elevator(CONFIG_3) → CONFIG_3 (honored, byte-identical); intra-OFDM in-band elevator(CONFIG_8) suppressed → +1.
- FAIL-BEFORE (`-DINBAND_PLUS1_CLIMB_FAILBEFORE`, clean build): in-band A JUMPS 0→3 (got=3, the wedge
  reproduced) and E JUMPS to CONFIG_8 — the exact elevator jumps that strand the reverse SACK. MEASURED.
- Full `--test` suite: [result recorded in the commit]; production byte-identical off the flag.

## §9 CORRECTION — fix B ("make the CMD apply the OFDM SACK_RSP") is FALSIFIED; the §7 SACK_RSP framing is a MISREAD (VERIFIED 2026-06-24, `_msab/keystone/_logsnap/modem_064839.log` redesign vs `modem_032735.log` legacy + `_msab/sackdeliv/aggregate.json` trustworthy A/B)

A fix-B session was tasked to "make the CMD APPLY the reliable OFDM SACK_RSP the RSP already sends" so
`nAcked_data` advances. STEP-1 verification of the two cited logs FALSIFIES the premise. The §7.1 line
"the RSP DID dispatch the reliable OFDM SACK_RSP transport ... but the CMD NEVER reflects it" is a
MISREAD of the `[ACK-GATE-V2] dispatching OFDM SACK_RSP` log. Strike it:

- ~~"the RSP dispatches a reliable OFDM SACK_RSP that the CMD ignores"~~ — FALSE. The
  `[ACK-GATE-V2] dispatching OFDM SACK_RSP (...)` line (`arq_responder.cc:2212`) is the per-batch
  ACK-GATE **announce**, printed BEFORE the transport-choice branch. The RSP then PREFERS the MFSK
  suffix (`arq_responder.cc:2239/2290`: `used_mfsk_path`; `send_sack_v2_frame` is the FALLBACK, taken
  only when the suffix path returns 0). On the wire BOTH arms used the MFSK suffix EXCLUSIVELY:
  `[TX-ACK-SACK] ... via MFSK suffix` redesign 7×, legacy 7×; the actual OFDM SACK_RSP wire-TX
  (`TX-SACK-V2` / `synthetic OFDM SACK_RSP TX`, `arq_responder.cc:3950`) fired **0×** in BOTH. No
  SACK_RSP frame is ever put on the wire in either arm.
- The CMD's OFDM SACK_RSP apply path EXISTS and is reachable (`arq_commander.cc:3923-3998`, entered
  on `!mfsk_handled_this_poll` at `:3847`, prints `[CMD-SACK-V2] decoded SACK_RSP ... applying` at
  `:3989`). It fired **0×** in BOTH arms — because the RSP sent no SACK_RSP frame to apply, not
  because the CMD ignores it. **Legacy DELIVERS (2693 B median, climbs to CONFIG_3-15) WITHOUT EVER
  APPLYING AN OFDM SACK_RSP.** So routing the redesign's bitmap onto SACK_RSP cannot make it match
  legacy — legacy doesn't use that transport.

### §9.1 The ACTUAL divergence (the reverse data-ACK that advances `nAcked_data` is the MFSK suffix in BOTH arms; the redesign's batch is too BIG for it)
The reverse data-SACK that advances `nAcked_data` is the **robust-MFSK ACK-SACK suffix**, applied via
`[CMD-MFSK-ACK-SACK] CLEAN/PARTIAL` (`arq_commander.cc:3714+`/`:3782`/`:3825`), in BOTH arms.
MEASURED:
- LEGACY: **5 CLEAN, 0 PARTIAL** MFSK-ACK-SACK → `nAcked_data` advances steadily 0→15; suffix
  `peak_matched` reaches **7/7**. Legacy airs **`frames=1` at EVERY OFDM rung** (`TX-PEAK ... frames=1
  cfg=0/15/16`), so the reverse turnaround is short and the GF(16)+CRC-12 suffix decodes cleanly.
- REDESIGN: **3 CLEAN + 2 PARTIAL** → `nAcked_data` sticks at **14**, then suffix CRC starts failing
  (`[CMD-ACK-PAT] Timeout: no ACK detected, peak_matched=3/7..5/7`, never above 5/7) → `[BREAK] Block
  failure at config 3` (T+238.801). The redesign airs **`frames=24 cfg=3`** and **`frames=6 cfg=0`**
  — the elevator JUMP (§8 root, the SNR-elevator `arq_commander.cc:5657`) packs a CONFIG_0-sized
  24-frame batch onto the slow CONFIG_3 rung, whose long forward airtime/turnaround overwhelms the
  (otherwise-working) suffix transport.

CONCLUSION: the binding constraint is the **forward batch size / turnaround at the climbed rung**
(the §8 elevator-jump root), NOT the reverse-SACK transport selection. Both arms share the identical
MFSK-suffix reverse transport; legacy survives it by airing 1-frame batches, the redesign breaks it by
airing giant batches. Fix B (route the bitmap via OFDM SACK_RSP) attacks a transport that (a) is never
on the wire and (b) legacy never needs — a symptom band-aid (CLAUDE.md §2) and the exact
serial-misdiagnosis the memory BENCH-CLAIM guard warns against.

### §9.2 Status — NO fix B implemented; the open lever is the §8 +1-climb's batch sizing
The §8 +1-climb (HEAD `65bb60b`) suppresses the elevator JUMP (CONFIG_0→3 → CONFIG_0→1), but the
`_msab/sackdeliv` trustworthy A/B (this HEAD) still shows REDESIGN rx median **77 B** / CONFIG_0×5 +
null×1, all stalled, vs LEGACY **2693 B** / CONFIG_3-15 (`rx_median_delta_pct=-97.1`). So even with
the +1 climb the redesign does not reach the OFDM rungs in budget — the climb is too slow and/or the
CONFIG_0 batch itself is multi-frame (`frames=6 cfg=0`) where legacy airs `frames=1`. The real next
lever is **forward batch SIZING on the in-band climb** (match legacy's 1-frame batches at a fresh OFDM
rung so the existing suffix transport survives the turnaround) — NOT a reverse-transport swap. This is
the same "giant-batch turnaround" root §8.1 already named; it is a SEPARATE, owner-ratifiable change,
not a same-session speculative chain (CLAUDE.md §5).

This is an **architecture-shaped** boundary: the in-band unilateral climb tends to carry the prior
rung's batch geometry across a rung change, where legacy's SET_CONFIG handshake re-seats a 1-frame
batch at the new rung. Whether to re-seat the in-band batch geometry per rung is an OWNER decision.

## §10 FIX-1 IMPLEMENTED — DEFER the climb while a lead-frame-only partial's hole is OUTSTANDING (2026-06-24, this session)

### §10.1 Root RE-CONFIRMED (4-agent adversarial convergence; source+log cited)
The redesign wedge at ~77B is an **ARQ/DELIVERY CONTIGUITY** orphan, the SIBLING that commit `2801d7c`
EXPOSED — NOT the §8/§9 batch-sizing framing (which is a downstream symptom of the same orphan). Chain
(each step source-verified on this HEAD):
1. A CONFIG_0 batch loses frame-0 (Schmidl-Cox acquisition seam) → the SACK comes back PARTIAL
   (`[CMD-MFSK-ACK-SACK] PARTIAL ... bitmap=0x0000001e`, seq0 missing, n_miss≤2).
2. The partial SACK ENQUEUES the missing frame-0 for retx: `retransmit_count++` at
   `arq_commander.cc:4106` (SACK_RSP path) / the shared `if(sack_detected)` big block (MFSK suffix).
   So immediately after a lead-frame-only partial, **`retransmit_count > 0` — the hole is outstanding.**
3. `last_partial_lead_frame_only` is set TRUE (`arq_commander.cc:4145` SACK_RSP / `:3806` MFSK), so
   `inband_lead_frame_only_partial()` (the 2801d7c predicate) returns TRUE.
4. The ROLLING-PARTIAL anchor-raise (`arq_commander.cc:5483`, log `[GEARSHIFT] ROLLING-PARTIAL anchor
   raise ... FRAME UP config 0 -> 3`) raises `last_data_viable_config` so the FRAME-UP +1 clamp permits
   the next rung, then the FRAME-UP gate (`:5606`) advances `consecutive_data_acks` to threshold and
   FIRES the config-change — **all while `retransmit_count > 0`.**
5. The config-change fire calls `clear_retx_queue()` (`arq_commander.cc:5708`, log `[RETX-CLEAR]
   dropping N stale retransmit frame(s)`), which ABANDONS frame-0 under the new epoch (`arq_common.cc:1165`).
6. The orphaned hole → the RSP never receives frame-0 → decoded `bsi` runs ahead of `last_delivered`
   → `RSP-V2-GAP-ABORT ... refusing silent concatenation` → wedge. (This integrity guard is CORRECT;
   we fix the CAUSE — the abandoned hole — not the guard.)

The mixbatch DRAINS the retx within one batch (`arq_commander.cc:1919`, `retransmit_count = leftover`);
the bug is purely that the climb FIRES (and `clear_retx_queue`s) *before* that drain, in the same poll.

### §10.2 The fix (gate the two ORPHANING side-effects on `retransmit_count`, KEEP the streak credit)
New pure predicate `inband_climb_hole_outstanding()` (`arq_common.cc`): `inband_rate_feature_enabled()
&& retransmit_count > 0`. Two AND-terms added:
- **Anchor-raise** (`arq_commander.cc:5495`): `else if(inband_lead_frame_only_partial() &&
  !inband_climb_hole_outstanding())` — defer the anchor-raise while the hole is outstanding.
- **FRAME-UP fire** (`arq_commander.cc:5653-5654`): `if(consecutive_data_acks >= eff_frame_shift_threshold
  && !inband_climb_hole_outstanding())` — defer the config-change fire (the `clear_retx_queue()` site)
  while the hole is outstanding.

**2801d7c RECONCILIATION (the needle — NOT re-broken):** the streak CREDIT (`consecutive_data_acks++`
at `arq_commander.cc:5625`) lives OUTSIDE both gated blocks and is UNGATED — the lead-frame-only partial
still BUILDS the climb streak (2801d7c's whole purpose). Only the two side-effects that orphan the hole
are deferred. The mixbatch drains the retx within one batch; the NEXT whole batch has
`retransmit_count == 0`, the gate opens, and the ALREADY-BUILT streak fires the climb PROMPTLY — no
re-serialization, no re-block of the forward climb. A genuinely CLEAN/WHOLE batch never has an
outstanding hole (`retransmit_count == 0`), so the gate is a no-op there → the 2801d7c
forward-climb-on-clean fires immediately (proven in the test, "hole drained (retx=0): FRAME-UP fires
promptly (2801d7c NOT re-broken)").

### §10.3 §5 CROSS-LAYER AUDIT (shared state: the climb/anchor/batch-completion gate vs `retransmit_count`)
Producers of `retransmit_count` (the hole signal): set >0 at the partial-SACK capture
(`arq_commander.cc:4057-4106`); drained to `leftover` when the mixbatch is built+sent (`:1919`); zeroed
by `clear_retx_queue()` (`arq_common.cc:1174`) and the ctor/reset sites. Consumer added: the two gate
AND-terms above (read-only). The fix changes ONE assumption — "the climb may fire/raise the moment the
lead-frame-only partial is credited" → "...only once the partial's retx hole has drained
(`retransmit_count==0`)". The 6 consumers the task flagged MUST NOT regress, verified:
1. **GAP-ABORT integrity guard** (`RSP-V2-GAP-ABORT ... refusing silent concatenation`): UNCHANGED and
   now NO LONGER TRIGGERED — the fix removes its CAUSE (the abandoned hole). We did not touch or weaken
   the guard. VERIFIED (the guard is the symptom the orphan produced; the fix is upstream).
2. **102→0 robust→OFDM HINGE re-init**: the tier-cross uses the legacy SET_CONFIG handshake
   (`arq_commander.cc:857`), a DIFFERENT producer path than the FRAME-UP gate. `retransmit_count` is 0
   at a fresh tier-cross (no partial enqueued yet), so the gate is a no-op there. UNTOUCHED. VERIFIED.
3. **Steady-state OFDM data decode / retx machinery**: the fix touches only the gearshift FRAME-UP
   gate + the anchor-raise; the retx capture (`:4057-4106`), mixbatch build (`:1919`), and SACK apply
   are byte-unchanged — frame-0 is still queued, mixed, and recovered exactly as before. The fix only
   stops the climb from RACING the drain. VERIFIED (test: streak CREDIT retained; `--test` 58/0).
4. **Legacy SET_CONFIG path (flag off)**: `inband_climb_hole_outstanding()` returns false off-flag
   (`!inband_rate_feature_enabled()`), so both AND-terms are `&& true` → byte-identical. The
   anchor-raise branch is `else if(inband_lead_frame_only_partial() && ...)` and
   `inband_lead_frame_only_partial()` is itself feature-gated off → the whole branch never enters
   off-flag (as before 2801d7c). VERIFIED (test E: "flag-off: defer gate is a no-op").
5. **2801d7c forward-climb-streak crediting** (must STILL climb on a clean/whole batch): the
   `consecutive_data_acks++` credit is ungated; a clean/whole batch has `retransmit_count==0` → fire
   is immediate. VERIFIED (test: "hole drained (retx=0): FRAME-UP fires promptly (2801d7c NOT
   re-broken)"; the sub-threshold case proves the defer never fires EARLY — only the threshold holds it).
6. **RETX-CLEAR generic config-change recovery** (`clear_retx_queue` at the gearshift-down `:5802/:5896`,
   the watchdog `:5714`, and the FRAME-UP fire `:5708`): the fix only DELAYS the FRAME-UP-fire site
   (`:5708`) until the hole drains; it does NOT alter the down/watchdog RETX-CLEAR sites (those are
   recovery paths that legitimately drop stale retx on a DEMOTE/watchdog, a different intent). The
   generic config-change recovery semantics are preserved — the fix only stops a forward CLIMB from
   firing it on a not-yet-drained hole. VERIFIED.

Generic coverage: the gate keys on `retransmit_count`, not on CONFIG_0 — so it covers EVERY climb
(0→1, 1→3, 3→6, …→16) whose trailing batch carries an outstanding lead-frame-only hole, not just 0→3.

### §10.4 Tests (fail-before / pass-after, VERIFIED; new `--test-inband-climb-defer`)
Drives the EXACT production decisions — the FRAME-UP fire AND-term (`:5653`) and the anchor-raise
AND-term (`:5495`) — across `retransmit_count = {1 (hole), 0 (drained)}` at CONFIG_0:
- **PASS-AFTER** (default): hole outstanding (`retx=1`) → fire DEFERRED, anchor-raise DEFERRED, but the
  streak CREDIT is RETAINED; hole drained (`retx=0`) → fire + raise fire PROMPTLY (2801d7c not re-broken);
  a sub-threshold streak at `retx=0` is held by the THRESHOLD (not the defer) and still accrues; flag-off
  → the gate is a no-op. ALL PASS.
- **FAIL-BEFORE** (`-DINBAND_CLIMB_DEFER_FAILBEFORE`, clean build): the gate is pinned false → the climb
  FIRES and the anchor RAISES *while the hole is outstanding* (`got=1`) — the orphan reproduced (the
  `clear_retx_queue()`-abandons-frame-0 racing the drain). MEASURED: same input, opposite fire/raise
  verdict vs the fixed build → the gate is load-bearing.
- Full `mercury.exe --test`: **58 passed, 0 failed** (+ Winlink dict 12/0), `[TEST-FRAME0-PARTIAL] ALL
  PASS`, `[TEST-CLIMB-DEFER] ALL PASS`. Production byte-identical off the flag. EXIT 0.

STATUS: FIX-1 implemented + tested. The next step (a long realtime A/B, redesign ON vs legacy OFF) is
OWNER-driven, NOT run here. If the A/B shows the climb is now too SLOW (deferred too long) or the
batch-sizing root (§9.2) still binds after the orphan is removed, that is a SEPARATE, owner-ratifiable
lever — not a same-session speculative chain (CLAUDE.md §5).

## §11 FIX-2 ("re-seat data_batch_size=1 on the unilateral climb") — MECHANISM FALSIFIED, NOT IMPLEMENTED (2026-06-24, STEP-1 verification this session)

A FIX-2 session was tasked with: "the redesign's `inband_unilateral_config_change()` (`arq_common.cc:3111`)
refills the TX FIFO at the new config but NEVER calls `set_data_batch_size(1)`, so it carries the PRIOR
rung's MULTI-FRAME batch geometry across the climb; legacy's SET_CONFIG ACK-apply re-seats a 1-frame batch
per OFDM rung; re-seat to 1 to match legacy." STEP-1 firsthand verification FALSIFIES the mechanism on
THREE counts (source+log cited, this HEAD 7bba3ed):

- ~~"the unilateral path NEVER re-seats data_batch_size, so it carries the prior rung's geometry"~~ — FALSE.
  `inband_unilateral_config_change()` calls `load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES)`
  (`arq_common.cc:3153`). `load_configuration` does NOT early-return for PHYSICAL_LAYER_ONLY (the `if(level!=FULL)`
  at `:2077` is only a canary check; the body continues). Its OFDM batch-scaling block (`:2221-2237`,
  NOT gated on `level==FULL`) calls `set_data_batch_size(fixed_batch)` with `fixed_batch = radio_batch_size`
  (=25) clamped to `max_batch = round(30000/message_transmission_time_ms)`. So the unilateral path RE-SEATS
  `data_batch_size` from the NEW config's frame time, exactly like a FULL load. NO prior geometry is carried.

- ~~"legacy's SET_CONFIG ACK-apply re-seats a 1-frame batch per OFDM rung"~~ — FALSE. The legacy SET_CONFIG
  ACK-apply (`arq_commander.cc:6516-6591`) calls the SAME `load_configuration(data_configuration,
  PHYSICAL_LAYER_ONLY, YES)` (`:6526`) and refills the FIFO with the VERBATIM-identical loop
  (`:6573-6591` == the unilateral `arq_common.cc:3155-3174`). It NEVER calls `set_data_batch_size(1)`.
  Legacy and the redesign's unilateral path are BYTE-IDENTICAL w.r.t. batch sizing. There is no per-rung
  `set_data_batch_size(1)` re-seat in legacy for the redesign to "miss".

- VERIFIED on the wire (`_msab/keystone/_logsnap`, TX-PEAK `frames=N cfg=` histograms): BOTH arms air
  `frames=6 cfg=0` (2× each). Legacy ADDITIONALLY reaches `frames=1` at cfg=0(1×)/4/13/14/15/16 and
  `frames=25` at cfg=15/16. The redesign NEVER reaches `frames=1` at any OFDM rung; it airs `frames=6 cfg=0`
  then `frames=24 cfg=3`. So the `frames=` divergence is REAL but its cause is NOT a `data_batch_size`
  re-seat — `frames = message_batch_counter_tx = min(data_batch_size, #ADDED_TO_LIST messages available)`
  (the TX batch-fill `arq_commander.cc:1941-2026`, capped at `:1945`/`:2003`/`:2022`). Legacy airs `frames=1`
  at the transient OFDM rungs because it CLIMBS THROUGH them fast (the SET_CONFIG fast dedicated control-ACK)
  with its FIFO mostly drained at each step (only ~1 frame queued); the redesign WEDGES at cfg=0/cfg=3 with a
  full 6/24-frame backlog and thus packs the full `data_batch_size` cap.

CONSEQUENCE — FIX-2 as specified would NOT help and is a SYMPTOM band-aid (CLAUDE.md §2):
re-seating `data_batch_size=1` on the unilateral climb would force the redesign to deliver its real 6/24-frame
backlog as 6/24 SEPARATE 1-frame batches, each with its OWN reverse-SACK turnaround — MORE turnarounds at the
slow rung, not fewer — and the frame-0 Schmidl-Cox seam would then hit EVERY 1-frame batch (each batch's sole
frame IS the lead frame), changing the rolling-partial character the existing FIX-1 / 2801d7c fixes rely on.
The `frames=` count is a CONSEQUENCE of the wedge (slow climb + backlog accumulation), not its cause. The
binding root remains the one §8.1/§9.2 already named: the reverse data-ACK/SACK DELIVERY LOOP at the climbed
rung (the §8 +1-climb already suppresses the elevator JUMP; the residual is delivery throughput / climb speed),
NOT a forward batch-geometry re-seat.

STATUS: NO FIX-2 implemented. The prompted mechanism is falsified by firsthand source+log verification;
implementing the re-seat would be the serial-misdiagnosis the memory BENCH-CLAIM guard warns against
(the root has shifted repeatedly this session). Re-seating forward batch geometry per rung remains an
owner-ratifiable lever ONLY if re-framed against the DELIVERY-loop root — but it must NOT be sold as
"matching legacy's per-rung 1-frame re-seat", because legacy has no such re-seat.

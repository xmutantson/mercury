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

[filled after the A/B]

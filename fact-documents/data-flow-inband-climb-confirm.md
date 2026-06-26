# Data-Flow Audit / Design — in-band CLIMB-CONFIRM pivot (fast reverse control-ACK)

Branch: `feat/inband-a3-decouple`, HEAD `a3e045f` (FIX-1 `7bba3ed` + §11 fact-doc).
Scope: the prompt-approved PIVOT — keep the in-band CONFIG_TAG ANNOUNCE unilateral, but
gate the CLIMB-ADVANCE on a FAST reverse control-ACK confirming the RSP adopted the new
rung, INSTEAD of the slow clean-data-SACK streak. OFF-BENCH, design-step only.

Companion: `data-flow-inband-frame0-rolling-partial.md` (the §1–§11 climb-wedge chain),
`data-flow-inband-tier-crossing.md` (§6 = the KEYSTONE), `data-flow-inband-adopt-metric-gate.md`.

---

## §1 STEP-1 FINDING — the pivot is ALREADY IMPLEMENTED (decision: STOP-for-review, do NOT re-build)

The prompt frames the binding root as: "the redesign's CLIMB is gated on a SLOW clean
DATA-SACK streak (`consecutive_data_acks`); at the marginal rung clean batches never
accumulate so it never climbs … LEGACY climbs via a FAST DECOUPLED SET_CONFIG control-ACK …
THE PIVOT: gate the CLIMB-ADVANCE on a FAST reverse control-ACK confirming rung adoption,
decoupled from the clean-data-SACK streak."

**That mechanism already exists on this HEAD.** It was built across the §1.8 pipeline-climb,
the §6 KEYSTONE, the §2 rolling-partial unblock, the §8 +1-climb, and the §10 hole-defer
(commits up to `a3e045f`). VERIFIED by reading the live code, this session:

### §1.1 The FAST decoupled climb-confirm = the KEYSTONE (option (b)/(c), already wired live)
`inband_retag_confirm_from_base_pattern(mfsk_matched, ack_match_threshold)`
(`arq_common.cc:3487`), called LIVE from the ACK-decode path at `arq_commander.cc:3686` with
the live decode's `mfsk_matched` + `telecom_system->ack_mfsk.ack_match_threshold`.

- The RX already sends a reverse MFSK ACK per data batch with a robust BASE tone-pattern
  (correlation `mfsk_matched`, threshold 7/16, P(false)=2.4e-5/poll) + a fragile bsi+bitmap
  SACK suffix (GF(16)+CRC-12). The KEYSTONE confirms the climb from the **BASE pattern ALONE**
  — DECOUPLED from the marginal CRC-12 suffix (which fails at WGN:40 on the turnaround,
  peak_matched 3–5/7). It fires on the suffix-FAILED path (`if(!decoded)`, `:3685`).
- This IS the prompt's option (b)/(c): reuse the existing reverse MFSK ACK / SACK header to
  carry a fast rung-adopt-confirm. NO new wire frame, NO new RX TX, NO new control op — the
  disarm post-state is byte-identical to `inband_retag_confirm_from_sack`. It is also exactly
  the prior-art REFINEMENT (3) the research memory `hf_rate_adapt_prior_art` prescribed
  (PACTOR CS4/CS5: "carry intra-tier up/down as a couple of bits in the reverse ACK so even
  fine steps are cheaply receiver-acked").

### §1.2 The climb-ADVANCE is ALREADY decoupled from the clean-data-SACK streak = the PIPELINE-CLIMB
`inband_pipeline_climb_active()` (`arq_common.cc:3335`), consumed at the FRAME-UP gate
`arq_commander.cc:5601/5615`:
```
bool inband_climb_pipeline = inband_pipeline_climb_active();           // :5601
bool batch_promotable = (inband_climb_pipeline || inband_lead_frame_partial)
    ? (data_ack_received==YES)                                         // OPTIMISTIC: partial OR clean
    : promotion_allowed_on_batch(last_batch_fully_acked);             // legacy strict-clean
```
While a CLIMB re-tag is armed, FRAME-UP advances `consecutive_data_acks` on ANY
forward-healthy data ACK (`data_ack_received==YES`, **partial or clean**) WITHOUT waiting for
the slow clean fully-acked confirm. The prompt's premise — "needs N CLEAN data batches at the
current rung before FRAME-UP fires" — is NO LONGER the gate under inband: the pipeline path
relaxed it to "any forward-healthy ACK" (`arq_common.cc:3304-3328` header). The slow
clean-streak gate survives ONLY for legacy (flag-off → predicate false → byte-identical).

### §1.3 The climb DECISION is the gearshift SNR signal, paced +1/rung
The climb target is `inband_climb_target(proposed_frame, snr_elevator, inband_plus1_on)`
(`arq_commander.cc:5711`, §8): under inband it is STRICT +1 (elevator suppressed) so the
CMD/RSP rungs stay aligned and each rung's batch+turnaround stays small. The over-climb net
is `inband_retag_escalate_if_climb_exhausted` (`arq_common.cc:3536`): an un-followed climb
auto-demotes to `inband_last_confirmed_config` (NEVER a BREAK, never below the floor).

**CONCLUSION**: the prompt's pivot — unilateral announce + fast decoupled reverse-ACK
climb-confirm + climb-advance gated on that confirm rather than clean-data — is the EXISTING
design on `a3e045f`. There is no new climb-confirm mechanism to build; building one would
duplicate the keystone/pipeline and re-introduce the very serialization they removed.

---

## §2 WHY THE REDESIGN STILL STALLS (the REAL residual binding constraint — VERIFIED)

Despite the full fast-confirm machinery, the trustworthy A/B `_msab/parity_fix1/aggregate.json`
(this HEAD, RT @SNR3k=40dB, n=6/arm, interleaved, FTRT-fair) measures:

| arm | deliver | stall | rx_median | rx_max | final config modes |
|-----|---------|-------|-----------|--------|--------------------|
| **ON** (inband) | 0.0 | 1.0 | **47.5 B** | 1631 | null×2, ROBUST_0×2, CONFIG_0×1, ROBUST_2×1 |
| **OFF** (legacy)| 0.0 | 0.5 | **2129.5 B** | 5867 | null×3, CONFIG_5, CONFIG_4, CONFIG_16 |

The ON arm is pinned at the robust/CONFIG_0 floor — the climb-confirm machinery FIRES (the
keystone log `[INBAND-TX] CONFIRMED followed CONFIG_3 via BASE ACK pattern` is present in the
prior keystone trace, §7 of the companion doc) but the **forward DATA never fully delivers**.

ROOT (companion doc §7.1 / §9.1, VERIFIED from `_msab/keystone/_logsnap`): the binding
constraint is the **reverse DATA-ACK/SACK DELIVERY LOOP**, NOT the climb-confirm:
- `stats.nAcked_data` STICKS (14 in the keystone trace) — the data bitmap that says which
  frames to retx never reliably returns, so partial batches never complete → BREAK at every
  rung. In legacy `nAcked_data` advances steadily 0→15.
- Both arms ride the IDENTICAL robust-MFSK ACK-SACK suffix for the reverse data-SACK; legacy
  survives it by airing **1-frame batches at fresh OFDM rungs** (short turnaround), the
  redesign breaks it by carrying a **multi-frame batch** (`frames=6 cfg=0` / `frames=24
  cfg=3`) whose long forward airtime+turnaround overwhelms the (otherwise-working) suffix CRC.

This is the DELIVERY-LAYER binding constraint the memory `delivery_layer_binding_constraint`
already names (constraint = DELIVERY not PHY). It is a DISTINCT layer from the climb-confirm
the pivot targets — the climb-confirm pivot is ALREADY DONE and is NOT the residual root.

---

## §3 §5 CROSS-LAYER AUDIT — climb/confirm/config shared state (producers/consumers, must-not-regress)

The pivot's would-be edit touches the climb/confirm/config state. Since the pivot is already
implemented, this audit DOCUMENTS the current producer/consumer graph and confirms what any
FUTURE delivery-loop fix MUST NOT regress.

### §3.1 Producers
- **CONFIG_TAG announce (unilateral)**: `inband_unilateral_config_change` (`arq_common.cc:3111`)
  ARMS `inband_retag_armed` + sets `inband_retag_config` / `inband_pre_announce_config` /
  `inband_announce_bsi` (filled on first emit, `:2769`).
- **Climb-confirm**: KEYSTONE `inband_retag_confirm_from_base_pattern` (`arq_common.cc:3487`,
  called `arq_commander.cc:3686`) + the CRC-valid `inband_retag_confirm_from_sack`
  (`:3445`, called `:3775/:3812/:3982` + RSP `arq_responder.cc:6389`). Both DISARM the re-tag
  and set `inband_last_confirmed_config`.
- **Climb-advance streak**: `consecutive_data_acks++` (`arq_commander.cc:5625`, UNGATED — the
  2801d7c credit) → FRAME-UP fire (`:5653`, gated `>= eff_thresh && !inband_climb_hole_outstanding()`).
- **FIX-1 hole-gate**: `inband_climb_hole_outstanding()` (`arq_common.cc:3392`) = feature-on &&
  `retransmit_count>0`; producer of `retransmit_count` is the partial-SACK capture (`:4057-4106`),
  drained by the mixbatch (`:1919`).
- **Demote**: `inband_route_failure_demote` (the Class-A degrade chokepoint) +
  `inband_retag_escalate_if_climb_exhausted` (`arq_common.cc:3536`).
- **bsi-rebase HINGE-2 / tier-cross**: legacy SET_CONFIG (`arq_commander.cc:857`) for robust↔OFDM.

### §3.2 Consumers
- FRAME-UP gate `batch_promotable` (`:5615`) reads `inband_pipeline_climb_active()` +
  `inband_lead_frame_only_partial()`; FRAME-UP fire reads `inband_climb_hole_outstanding()`.
- D4 escalation reads `inband_retag_armed` / `inband_retag_count` / `inband_last_confirmed_config`.
- The +1 anchor clamp (`:5565`) reads `last_data_viable_config`.
- The reverse data-SACK apply (`[CMD-MFSK-ACK-SACK]`, `:3714+`) advances `nAcked_data` (the
  residual binding root — §2).

### §3.3 MUST-NOT-REGRESS (and current status)
1. **Legacy byte-identical** — every inband predicate is gated on
   `inband_rate_feature_enabled()` → false off-flag. INTACT.
2. **FIX-1 hole-defer** (`inband_climb_hole_outstanding`) — the climb must not fire while a
   lead-frame-only retx hole is outstanding (else `clear_retx_queue` orphans frame-0 →
   GAP-ABORT). INTACT; any delivery-loop fix must keep the `:5653` AND-term.
3. **GAP-ABORT integrity guard** (`RSP-V2-GAP-ABORT … refusing silent concatenation`) —
   correct, untouched; FIX-1 removed its CAUSE.
4. **Steady-state** — the keystone is strictly-additive (fires only on `!decoded`); the
   pipeline gate self-disarms on confirm → zero steady-state overhead.
5. **Demote / down-ladder** — read `emergency_nack_count`/`config_is_at_bottom`, not the
   climb-confirm; the escalation net auto-demotes an over-climb (NEVER a BREAK). INTACT.
6. **bsi-rebase HINGE-2 / tier-cross** — uses legacy SET_CONFIG; `retransmit_count==0` at a
   fresh cross so the hole-gate is a no-op there. INTACT.

A future delivery-loop fix (the §2 root) MUST preserve all six and is a SEPARATE, scoped change.

---

## §4 DECISION (the prompt's decision gate)

**STOP-for-review.** The pivot is a CLEAN reuse of existing machinery — but it is ALREADY
BUILT (the keystone + pipeline-climb + +1-climb + hole-defer). Implementing "a fast reverse
control-ACK climb-confirm decoupled from the clean-data-SACK streak" would DUPLICATE the
keystone/pipeline and re-introduce the serialization they removed — a no-op at best, a
regression at worst. Per CLAUDE.md §2 (root-cause, no symptom band-aids) and §5 (don't chain
a speculative cross-layer fix), the honest STEP-1 verdict is: the climb-confirm pivot is DONE;
the residual binding constraint is the reverse DATA-SACK DELIVERY LOOP at the climbed rung
(§2), a DISTINCT layer. That is an owner-ratifiable delivery-layer change (candidate
directions: match legacy's 1-frame fresh-rung batch geometry so the existing suffix transport
survives the turnaround — companion §9.2; or a multi-window / more-robust reverse data-SACK
transport — companion §7.2 (B)/(C)), NOT the climb-confirm mechanism the prompt scoped.

No code changed this session (design/verification only).

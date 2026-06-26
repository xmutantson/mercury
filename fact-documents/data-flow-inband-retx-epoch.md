# Data-Flow Audit — In-Band Retx-Queue / bsi-Epoch / BREAK-Reshrink (0-deliver)

Branch: `fix/inband-downladder-delivery` (off `feat/inband-rate-adapt`; carries the
redesign 1b1ee01 + the §3 deliver-before-break 98edd4d + liveness guard 9b916b7 +
guard connect-gate 1994a85).
Scope: the `MERCURY_INBAND_RATE=1` ON arm's residual **decode-but-0-deliver** under a
forward-healthy reverse-ACK turnaround miss → CMD EXHAUSTED-BREAK.
Status: **IMPLEMENTED (owner-ratified). The §5 consolidated rework is in: a forward-
healthy reverse-ACK turnaround MISS on a still-decoding inband link is routed to the
NO-BREAK `inband_route_failure_demote()` re-present instead of the connect-liveness
guard's `send_break_pattern()`. Closes HOLE A/B/C at once; the genuine BREAK (true-loss)
and config-change paths are intact. Paired regression `--test-inband-deliver` (3 cases,
fail-before reproduces the 0-deliver). Build o3 green; `--test` exit 0; legacy
byte-identical (purely additive, feature-gated). See §7.**

This is the 4th+ defect in the same gearshift-acq → delivery layer boundary
(after the D1→D5 prev-bump chain). Per CLAUDE.md §2 a config-unchanged no-op BREAK
that detonates the in-flight retx queue is an architectural fault, not a one-line
miss; the canonical retx-queue sibling chain (184fdcc→76f6185→ff829d5) is exactly
what proactive auditing here prevents.

---

## §0. The symptom (HW cycle1 ON logs + source-confirmed)

`MERCURY_INBAND_RATE=1`: the RX decodes **785 DATA frames** yet delivers **0 bytes**.
Observed chain (HW logs):

1. A forward-healthy reverse-ACK turnaround MISS (the first SACK lands after the CMD
   retry budget) →
2. CMD `[BREAK] All retries exhausted` (arq_commander.cc:337) →
3. `[BREAK] Anchor floor (exhausted): … clamping up to 15` (arq_commander.cc:353) →
4. `[BREAK] Dropping N step(s): config 15 -> 15` (arq_commander.cc:357) — a config
   **NO-OP** (`target == emergency_previous_config`) that STILL runs the full
   teardown →
5. CMD `[RETX-CLEAR] dropping N stale retransmit frame(s)` (arq_common.cc:1141,
   called arq_commander.cc:391) + an implicit **bsi-epoch ADVANCE** (the re-streamed
   batch carries a NEW `cmd_batch_seq_id`, incremented at arq_commander.cc:~2029) →
6. RX `break_detected` handler drops to ROBUST_0 (arq_responder.cc:440-504),
   reshrinks `data_batch_size→1`, and WIPES the in-flight prev/cur bsi state →
7. the in-flight batch (e.g. 24/25 — one tail frame missing) is stranded:
   `[RSP-V2-PREV-STALE]` discard (arq_common.cc:8441) / the prev-deliver gate
   `received >= expected` (arq_responder.cc:1067-1068) never passes →
8. every re-adopt is non-contiguous → `sack_v2_readopt_has_gap` →
   `rsp_gap_abort_teardown` (arq_responder.cc:867 → arq_common.cc:8576) →
   `reset_session_state` (arq_common.cc:8611). The CMD never resets; it keeps
   streaming; the RX re-adopts mid-stream and gaps again. = 0 bytes.

---

## §1. Producers / Consumers (CLAUDE.md §5)

### 1.1 The EXHAUSTED-BREAK path (CMD) — incl. the config-clamp no-op
PRODUCERS (who reaches/runs it):
- `arq_commander.cc:335-404` — the `emergency_break_retries==0` (exhausted) arm of
  the `emergency_break_active` state machine. **NOT inband-gated** — shared legacy
  path; both arms converge here once `emergency_break_active==1`.
- The inband arm reaches `send_break_pattern()`/`emergency_break_active=1` ONLY via
  the true-loss floor `inband_cmd_dead_batch_floor_reached()` (arq_commander.cc
  ~4076-4091) OR the new connect-liveness guard (data-flow-inband-connect-liveness.md
  §2, fires the SAME retained BREAK). Class-A degradation on a *still-delivering* link
  is supposed to route to `inband_route_failure_demote()` (NO-BREAK) — but the
  observed run reached the BREAK anyway (the floor/liveness backstop fired).
- `break_target_with_anchor()` (arq_commander.cc:152-159): `target =
  config_ladder_down_n(15, step)` (idx ≤17) is clamped UP to
  `last_data_viable_config` (=CONFIG_15, idx 18) → `target == 15 ==
  emergency_previous_config` whenever `breaks_since_last_data_success < 2` and the
  anchor sits at the live config. **This is the no-op case.**
CONSUMERS / EFFECTS (what the body does, arq_commander.cc:363-403, with **NO
`if(target==emergency_previous_config)` early-out** — VERIFIED, no such guard):
- `load_configuration(ROBUST_0, PHYSICAL_LAYER_ONLY, YES)` (:366) → robust pin →
  `set_data_batch_size(1)` (arq_common.cc:2175) — the reshrink chokepoint.
- `messages_tx[]` FREE + `fifo_buffer_tx.push_front` re-stage (:384-389).
- `clear_retx_queue()` (:391).
- `add_message_control(SET_CONFIG)` + `connection_status=TRANSMITTING_CONTROL` (:397-401).
- **MISSING:** any `cmd_batch_seq_id` rollback (contrast §1.3).

### 1.2 `clear_retx_queue()` + the in-flight retransmit queue (CMD)
PRODUCER (the single owner, R029): `clear_retx_queue()` arq_common.cc:1137-1147 —
zeroes `retransmit_count` (discarding `retransmit_frames[]` + parallel arrays
`_lengths/_positions/_types/_batch_seq_ids/_seq_with_eob`, read only over
`[0,retransmit_count)`).
CALLED FROM (every messages_tx-freeing recovery): both BREAK arms
(arq_commander.cc:303, :391), the no-break demote helpers (:2953, :4612), the SACK
re-stage (:4118, :4456, :5131), the FRAME-UP BREAK (:2612-2613).
CONSUMERS of `retransmit_frames[]`: `send_batch()` builds the v2 retx prefix from
`[0,retransmit_count)`; the SACK handler (arq_commander.cc:3649-3715) repopulates it
under the ORIGINAL `batch_seq_id` (:3702) so SACK retransmits are inherently
contiguous.

### 1.3 The bsi/parity epoch (`cmd_batch_seq_id`)
PRODUCERS: ctor / `reset_session_state` (init); the new-data batch builder
`+1 mod 256` (arq_commander.cc:~2029); **rollback to the in-flight bsi** at TWO
no-break demote helpers — `inband_route_failure_demote()` (arq_commander.cc:2966,
`cmd_batch_seq_id = min_inflight_bsi`) and the D3 reverse-ACK-starve demote
(arq_commander.cc:4625). **The BREAK paths (:240-320 ACK-received, :337-404
EXHAUSTED) do NOT roll back** — they free `messages_tx[]` without capturing
`min_inflight_bsi`, so the re-streamed batch carries an ADVANCED bsi.
CONSUMERS: the RX route-decision (current vs prev), `sack_v2_readopt_has_gap`,
`delivery_step_is_gap`, `bump_bsi_and_transfer_prev`.

### 1.4 RX in-flight prev batch (`messages_rx_prev[]`, `rsp_prev_batch_*`,
`rsp_current_expected_batch_seq_id`)
PRODUCERS: `bump_bsi_and_transfer_prev()` (transfer cur→prev); the prev-route in
`receive_v2_data_frame()` (RECEIVED + `rsp_prev_batch_received_count++`); the
frame-driven prev-deliver; `rescan_prev_on_batch_shrink()` (re-count within new
bound, **orphan accounting**, arq_common.cc:1086); `deliver_complete_inflight_before_break()`
(arq_common.cc:3999); **the RX BREAK handlers WIPE it** —
`rsp_current_expected_batch_seq_id = -1; rsp_prev_batch_seq_id = -1`
(arq_responder.cc:482-483 break_detected, :573-574 TERMINAL).
CONSUMERS: the prev-deliver gate `rsp_prev_batch_active && received >= expected`
(arq_responder.cc:1067-1068); `rescan_prev_on_batch_shrink`; the PREV-STALE
discard (arq_common.cc:8438-8451).

### 1.5 `deliver_complete_inflight_before_break()` (the §3 98edd4d primitive)
PRODUCER: arq_common.cc:3999-4060. Inband-gated; delivers a **COMPLETE** prev
(`received >= expected`, hard-returns 0 for a PARTIAL prev at :4012). Called BEFORE
the bsi-wipe at both RX BREAK sites (arq_responder.cc:481, :571).
**LIMIT: it cannot rescue a PARTIAL in-flight batch** — exactly the observed 24/25.

### 1.6 gap-abort / stale-discard / reset_session_state
`rsp_gap_abort_teardown()` arq_common.cc:8576-8613: sets `link_status=DROPPED`,
clears the bsi family + `messages_rx_prev[]` + carve-arm, `reset_session_state()`.
Fired from the delivery-time PREV gate (arq_responder.cc:1140) and the re-adopt
gate (arq_responder.cc:867/875) when `sack_v2_readopt_has_gap`/`delivery_step_is_gap`
see a forward skip (`fwd∈[2,128]`; both return false for `fwd==0` duplicate and
`fwd==1` contiguous successor — VERIFIED in arq.h, the gates are CORRECT).

### 1.7 SACK implicit-confirm termination
The SACK handler (arq_commander.cc:3649-3715) ACKs received frames (:3658), saves
missing frames to `retransmit_frames[]` under the original bsi (:3702), marks them
ACKED for cleanup (:3713). **No independent termination hole** — it does NOT
confirm-and-retire a batch the RX still shows partial; the missing frame stays
queued. Self-consistent. The hole is the BREAK path bypassing SACK entirely.

---

## §2. Invariant table — does a config-UNCHANGED no-op BREAK violate it?

| # | Invariant the delivery path assumes | Violated by the no-op BREAK? | Where |
|---|---|---|---|
| INV-1 | An in-flight RECEIVED-**partial** batch survives until delivered or genuinely lost | **YES** | RX BREAK handler wipes `rsp_prev_batch_seq_id`/`rsp_current_expected_batch_seq_id` (arq_responder.cc:482-483,573-574) + reshrinks batch→1 (rescan orphans partial); `deliver_complete_inflight_before_break` only rescues a COMPLETE prev (arq_common.cc:4012) |
| INV-2 | `cmd_batch_seq_id` epoch advances ONLY for a genuinely-new batch; a re-presented in-flight batch keeps its bsi (contiguous) | **YES** | BREAK paths re-stream the in-flight batch under an ADVANCED bsi (no `min_inflight_bsi` rollback at arq_commander.cc:337-404, unlike :2966/:4625) |
| INV-3 | The in-flight retransmit queue survives until the batch is delivered or genuinely lost | **YES** | `clear_retx_queue()` runs unconditionally on the no-op BREAK (arq_commander.cc:391) — drops the outstanding tail frame's retx |
| INV-4 | `last_delivered` advances monotonically; the high-water is never regressed | **YES (indirectly)** | the gap-abort `reset_session_state` sets `rsp_last_delivered = -1` (arq_common.cc:8611-8612), destroying the high-water mid-stream |
| INV-5 | A BREAK is a real recovery event (config change OR true loss), never a config NO-OP | **YES** | the anchor clamp resolves `target == emergency_previous_config` yet the full teardown still fires (arq_commander.cc:357 + :363-403, no early-out) |
| INV-6 | The gap-abort fires ONLY on a genuine forward skip | NO (gate is correct) | `sack_v2_readopt_has_gap`/`delivery_step_is_gap` return false for contiguous/duplicate — it fires only BECAUSE INV-2 was already violated |

INV-5 is the architectural smell; INV-1/INV-2/INV-3 are the three coupled
mechanical holes; INV-4/INV-6 are downstream consequences.

---

## §3. The §2 GATE decision — ONE root or MULTIPLE? → **MULTIPLE (3 coupled holes)**

The task's STEP-3 hypothesis was "the 0-deliver is the single config-unchanged-BREAK
detonation; make the no-op BREAK a no-op." **The audit FALSIFIES single-root.** A
point fix (whether "no-op when config unchanged" OR "roll `cmd_batch_seq_id` back on
the BREAK path") does NOT restore delivery of a PARTIAL in-flight batch. Three
INDEPENDENT holes, each verified:

**HOLE A — CMD epoch advance (INV-2/INV-3).** The BREAK paths re-stream the in-flight
batch under an ADVANCED `cmd_batch_seq_id` and `clear_retx_queue()` the outstanding
tail. *Symptoms downstream of A* (all closed by a `min_inflight_bsi` rollback like
arq_commander.cc:2966): the PREV-STALE discard (arq_common.cc:8438-8451, reached only
on a NEW-bsi `bump_bsi_and_transfer_prev`), and the `sack_v2_readopt_has_gap` /
`delivery_step_is_gap` aborts (they fire only because the epoch advanced).
→ **closed by a CMD-side bsi-rollback on the BREAK recovery (necessary).**

**HOLE B — RX partial-prev destruction (INV-1).** INDEPENDENT of A. The RX
`break_detected` handler (arq_responder.cc:440-504) and the inband TERMINAL-BREAK
handler (arq_responder.cc:561-583) BOTH (i) call
`deliver_complete_inflight_before_break()` which rescues ONLY a COMPLETE prev
(arq_common.cc:4012 hard-returns 0 for `received < expected`), then (ii) wipe the prev
bsi state (arq_responder.cc:482-483, 573-574) and (iii) `load_configuration(ROBUST_0)`
→ `set_data_batch_size(1)` → `rescan_prev_on_batch_shrink(1)` orphans the partial prev
(arq_common.cc:1086-1129). **So even a perfectly-contiguous re-send has NO live prev to
complete** — the RX re-adopts a fresh current batch from frame 0. A CMD bsi-rollback
does NOT touch this. → **needs a partial-prev-preserving re-adopt across the BREAK, OR
HOLE C.**

**HOLE C — the config-NO-OP BREAK itself (INV-5).** A still-delivering inband link is
dropped to ROBUST_0 for a config it never left. Even fixing A+B leaves an
architecturally wrong "recover by tearing down a healthy link." The *right*
architecture for a forward-healthy reverse-ACK miss is the turnaround design's
"decouple the miss from the demote" (memory: turnaround_solution_design) — route the
degradation through `inband_route_failure_demote()` (NO-BREAK, already rolls the bsi)
or a new "reverse-ACK miss ≠ BREAK" path, so HOLE B's prev-destruction is never
reached and HOLE A's rollback is inherited for free. **This is the consolidated fix
the redesign's own premise ("decouple BREAK from rate/loss") actually requires.**

### Why a point fix is unsafe (the §2 STOP rationale)
- "No-op when config unchanged" alone: leaves HOLE A (a config-CHANGED demote 15→10
  ALSO advances the epoch and gaps) and does nothing for HOLE B (the partial prev is
  destroyed by the very next genuine BREAK). It also changes the SHARED legacy
  EXHAUSTED-BREAK path (arq_commander.cc:337 is not inband-gated) → legacy regression
  risk unless carefully gated, and a legacy true-loss config-unchanged BREAK (RX
  genuinely already at ROBUST_0) legitimately needs the ROBUST_0 re-coordination.
- "bsi-rollback on the BREAK path" alone: closes HOLE A's symptoms but HOLE B still
  zero-delivers any PARTIAL in-flight batch (the dominant case — a turnaround miss
  strands the batch one tail frame short). Verified: arq_common.cc:4012 +
  arq_responder.cc:482-483.
- The three holes sit on the SAME BREAK/epoch/reshrink seam the D1→D5 prev-bump chain
  did. Shipping a narrow fix here is the exact pattern CLAUDE.md §2/§5 forbids — it
  would expose the sibling within hours of the next A/B.

---

## §4. Sibling list (for the owner decision)

1. **CMD BREAK epoch advance** — arq_commander.cc:337-404 (and the ACK-received twin
   :240-320, and every other CMD BREAK site that frees `messages_tx[]` without a
   `min_inflight_bsi` capture, e.g. :2612-2613) lacks the `cmd_batch_seq_id` rollback
   the no-break demote helpers have (arq_commander.cc:2966, :4625). [VERIFIED]
2. **RX partial-prev destruction on BREAK** — `deliver_complete_inflight_before_break`
   rescues only COMPLETE prev (arq_common.cc:4012); the RX BREAK handlers then wipe the
   prev bsi state (arq_responder.cc:482-483, 573-574) and reshrink batch→1 (orphan via
   `rescan_prev_on_batch_shrink`, arq_common.cc:1086). A PARTIAL in-flight batch is
   irrecoverable across a BREAK. [VERIFIED]
3. **Config-NO-OP BREAK (architectural)** — `break_target_with_anchor` clamps
   `target==emergency_previous_config` yet the full teardown runs with no early-out
   (arq_commander.cc:357, :363-403). A forward-healthy reverse-ACK miss should never
   reach a BREAK at all (turnaround-design decoupling). [VERIFIED]

---

## §5. Recommended consolidated rework (NOT implemented — owner ratification per §2)

The single coherent fix that closes all three, consistent with the redesign's
"decouple BREAK from rate/loss" premise and the turnaround design:

**Route a forward-healthy reverse-ACK turnaround miss on a still-decoding inband link
to a NO-BREAK re-present (the `inband_route_failure_demote` discipline) instead of the
EXHAUSTED-BREAK** — so (a) the in-flight retx queue + partial prev are PRESERVED (HOLE
B never reached), (b) `cmd_batch_seq_id` rolls back to `min_inflight_bsi` and the
re-sent batch is CONTIGUOUS (HOLE A inherited), (c) no config-NO-OP teardown of a
healthy link (HOLE C). Keep the GENUINE config-change BREAK and the true-loss BREAK
fully intact (they must still clear/epoch/teardown). Gate on
`inband_rate_feature_enabled()`; legacy byte-identical.

This is a multi-function, cross-layer change (CMD recovery routing + RX partial-prev
survival) — by CLAUDE.md §4 it requires the plan-first + owner approval before code,
and a paired regression (drive a partial in-flight batch through a forward-healthy
reverse-ACK miss; assert PRESERVED + delivered in-order; keep a config-CHANGE BREAK
case asserting clear/epoch STILL happen).

## §6. Open questions [?]
- [?] Which CMD BREAK trigger actually fired in the 785-frame run — the dead-batch
  floor or the connect-liveness guard? Both converge on arq_commander.cc:337 and the
  same RX prev-wipe, so the §3 verdict holds either way, but the routing fix in §5
  must intercept the trigger BEFORE `send_break_pattern()` (the floor/guard sites),
  not only the EXHAUSTED arm. Confirm on the bench A/B.
- [?] Does a NO-BREAK re-present need a ROBUST_0 coordination round for a genuinely
  faded reverse channel, or can the contiguous re-send ride the current OFDM config?
  (`inband_route_failure_demote` re-coordinates via SET_CONFIG; a same-config
  re-present may not need it.) Resolve in the §5 design. **[RESOLVED in §7: the
  re-present demotes ONE rung (config_ladder_down) via the unilateral CONFIG_TAG, so it
  re-coordinates onto a lower OFDM rung — it does NOT need a ROBUST_0 round; the genuine
  fade is still caught by the down-ladder + the bottom-rung dead-batch floor.]**

---

## §7. IMPLEMENTED ROUTING + DISCRIMINATOR (the ratified §5 rework)

Commit on `fix/inband-downladder-delivery`. Purely ADDITIVE (4 files, +506/-0): nothing
in any legacy path was edited; the only new live-path code sits inside the already-
`inband_rate_feature_enabled()`-gated `inband_connect_liveness_guard()`.

### §7.1 Where the BREAK was actually coming from (the §6 [?] resolved by source)
Every inband DATA-path Class-A trigger (retx-runaway :1628, turbo :2574/:2680/:2776,
emergency-NACK :4693, frame-gearshift :4073/:4295) ALREADY routes through
`inband_route_failure_demote()` first and only reaches `send_break_pattern()` via
`inband_cmd_dead_batch_floor_reached()` (the SESSION_DEAD floor, at the ladder bottom).
The **one** inband path that fires a BREAK on a forward-healthy CONNECTED link is the
**connect-liveness guard** (`inband_connect_liveness_guard()`, arq_commander.cc:3066):
it fires when `link_status==CONNECTED` AND `stats.nAcked_data` is FLAT for N polls AND
not in a data phase. **`stats.nAcked_data` advances ONLY on a RECEIVED reverse-ACK/SACK**
(the sole producers are arq_commander.cc:61, :3659, :3688, :3714 — VERIFIED). A
forward-HEALTHY reverse-ACK turnaround MISS — the RX decodes our forward DATA fine but
its SACK lands outside the CMD listen window — therefore leaves `nAcked_data` flat and
trips this guard, which fired the `send_break_pattern()` that detonated HOLE A/B/C. So
the §6 [?] is resolved from source: the trigger is the **connect-liveness guard**, not a
data-path BREAK.

### §7.2 The rework (file:line old→new)
- **arq_commander.cc:3061 NEW** `cmd_has_inflight_data_batch() const` — true iff any
  `messages_tx[i].status != FREE && length > 0` (the same in-flight-frame scan every
  BREAK/demote re-stage uses, e.g. :298/:386/:2923). The CMD-side proxy for
  "forward-healthy / not genuinely dead".
- **arq_commander.cc:3178-3198 NEW** (inside the guard's production `#else`, BEFORE the
  `INBAND_LIVENESS_MAX_BREAKS` check, gated `#ifndef INBAND_DELIVER_FAILBEFORE`): if
  `cmd_has_inflight_data_batch() && !config_is_at_bottom(current_configuration,
  robust_enabled)` → `inband_route_failure_demote(config_ladder_down(current,robust),
  "forward_healthy_revack_miss")`; on success re-arm the stall window + the liveness-break
  budget and `return true` (NO `send_break_pattern`, NO `emergency_break_active`). On a
  no-op demote (raced to the bottom) it falls through to the genuine BREAK below
  (unchanged). The pre-existing BREAK firing at :3200+ is byte-untouched.
- **arq.h:3402 NEW** declaration of `cmd_has_inflight_data_batch()`.

### §7.3 The discriminator (forward-healthy vs true-loss) — correct at BOTH edges
- forward-HEALTHY ⇔ an in-flight forward DATA batch is queued (`messages_tx[]` non-FREE)
  AND a lower rung exists. The no-break demote PRESERVES the retx queue + partial prev
  (no `send_break_pattern` ⇒ the RX `break_detected` handler is never entered ⇒ no
  ROBUST_0 reshrink-orphan — HOLE B), rolls `cmd_batch_seq_id` back to `min_inflight_bsi`
  (contiguous re-present, no D3.1 GAP-ABORT — HOLE A), and never tears down a healthy link
  (HOLE C). Does NOT tick the true-loss floor; does NOT consume the liveness-break budget.
- genuine DEAD/livelock ⇔ NO in-flight DATA batch (a connect/negotiate handshake stuck
  with zero forward DATA ever — the guard's original purpose) OR already at the ladder
  bottom (no rung to demote to). Falls through to the bounded §7 BREAK→ROBUST_0, intact.
- **Too-narrow edge** (re-opens the 0-deliver): closed — the demote preserves the partial
  prev + contiguous bsi (verified: `--test-inband-deliver` PART A/B flip vs fail-before).
- **Too-broad edge** (strands a dead session in a no-break loop): closed — each demote
  LOWERS the rung; at the bottom the demote returns false ⇒ the genuine BREAK fires, and
  the `INBAND_LIVENESS_MAX_BREAKS` hard-reset still backstops a link the demotes can't save.

### §7.4 Invariants now HELD (cf. the §2 violation table)
| # | Invariant | Status after §7 |
|---|-----------|-----------------|
| INV-1 | a RECEIVED-partial in-flight batch survives until delivered/lost | **HELD** — no `send_break_pattern` on a forward-healthy miss ⇒ the RX prev-wipe (arq_responder.cc:482-483) is never reached; the partial survives + completes on the contiguous re-present (`--test-inband-deliver` B1-B3). |
| INV-2 | `cmd_batch_seq_id` advances only for a genuinely-new batch | **HELD** — the no-break route rolls the bsi back to `min_inflight_bsi` (PART A4). |
| INV-3 | the in-flight retx queue survives until delivered/lost | **HELD on the forward-healthy path** — the no-break re-present re-queues plaintext at the lower rung (the demote's own `clear_retx_queue` is a re-stage, not a loss); on a GENUINE config-change/true-loss the clear is correct (PART D3). |
| INV-5 | a BREAK is a real recovery event, never a config NO-OP | **HELD** — a forward-healthy reverse-ACK miss never reaches a BREAK at all; the config-NO-OP EXHAUSTED-BREAK is no longer reached by this trigger. |

### §7.5 Paired regression — `--test-inband-deliver` (arq_responder.cc `test_inband_deliver`)
Wired into `mercury.exe --test` (main.cc) + a standalone `--test-inband-deliver` flag.
fail-before = rebuild with `-DINBAND_DELIVER_FAILBEFORE` (removes BOTH the guard
discriminator AND switches the test to assert the bug state). VERIFIED both arms:
- **PART A** (CMD routing): pass-after NO BREAK (`emergency_break_active==0`), demoted
  CONFIG_10→9, bsi rolled 10→7, floor+liveness-budget untouched. fail-before: BREAK fired
  (`==1`), NOT demoted (stays 10), bsi NOT rolled.
- **PART B** (RX consequence): pass-after a 24/25 PARTIAL prev is PRESERVED → completes →
  all 25 frames (400 B) delivered in order. fail-before: the BREAK reshrink orphans the
  partial (received 24→1) → **0 app bytes** (the HW 0-deliver, faithfully reproduced).
- **PART C** (genuine true-loss STILL BREAKs): (i) the guard with NO in-flight DATA STILL
  fires the BREAK; (ii) at the ladder bottom the SESSION_DEAD floor STILL fires at exactly
  N∈{1,2,3}. Identical in both arms (the discriminator does not weaken the death path).
- **PART D** (genuine config-change STILL clears/epochs): retx cleared, unilateral config
  applied, bsi rolled, the unilateral one-shot armed. Identical in both arms.

Build: `bash build.sh o3` green. `mercury.exe --test` exit 0, all 6 groups `failed=0`
(incl. `test_inband_liveness` + `test_inband_downladder`, which drive the SAME guard /
deliver primitives → no regression). Legacy byte-identical by construction (+506/-0,
feature-gated; OFF early-returns from the guard before the discriminator).

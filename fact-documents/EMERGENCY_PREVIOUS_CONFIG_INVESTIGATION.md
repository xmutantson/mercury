# Emergency Previous Config Investigation

Date: 2026-05-16
Branch: monitor @ 04a8438 (post Bug D)
Trigger: Open follow-up from `SACK_DESIGN_A_PLAN.md` §7.13.14.7 (in turn
quoting `BREAK_FAILSAFE_INVESTIGATION.md` §11.4):

> `emergency_previous_config` is intentionally never refreshed during
> Phase 1 retries (`arq_commander.cc:1318`). With `-R` it's benign,
> without `-R` it pinned the loop's target to a still-failing config.
> Worth a separate review.

Bug D (commit `1acdb3c`, `arq_responder.cc` BREAK handler now resets
`messages_control.status = FREE`) closed the post-BREAK stuck-state
that made the never-refresh policy *fatal*. The remaining question:
**is never-refresh-on-Phase-1-retry correct, or is it now a
recovery-time pessimisation that should be fixed?**

Cross-ref: `POST_BREAK_STUCK_INVESTIGATION.md` (the Bug D investigation).

## §1 Verdict

**A — Never-refresh on Phase-1 retry/exhaust is correct by design.**

The asymmetry between Phase-1 (no refresh) and Phase-2 (refresh) is
deliberate and semantically required:

- Phase-1 retry/exhaust ≡ "RSP didn't hear us on the coordination
  layer." Same plan, same target, retry the transmission. Refreshing
  the anchor would change the target — but the target hasn't been
  disproved, only un-delivered.
- Phase-2 retry/exhaust ≡ "RSP heard us, loaded the target, but the
  target config doesn't actually work end-to-end." Refresh anchor to
  current (= the just-failed target) so the next descent advances.

The Phase-2 refresh at `arq_commander.cc:1278` is the descent
mechanism. The Phase-1 not-refresh at `arq_commander.cc:1318` is the
*absence* of descent — by design.

Proposed action: **add a one-line WHY comment** at the existing WHAT
comment on line 1318. Comment-only diff in §6 below. **Owner approval
required before any source edit.**

## §2 What `emergency_previous_config` is

Definition: snapshot of `current_configuration` taken at the moment a
BREAK is triggered. Used by the BREAK-ACK handler as the anchor for
`config_ladder_down_n(emergency_previous_config, break_drop_step,
robust_enabled)` to compute the descent target.

Initial value: `arq_common.cc:340` sets `=CONFIG_0` at struct init.
Reset at `arq_common.cc:2834` `reset_session_state()` to
`init_configuration`.

### §2.1 Write sites in arq_commander.cc

| Line | Site | Refreshes to | Trigger |
|------|------|--------------|---------|
| 1278 | Phase-2 probe exhaust | `current_configuration` (= target) | Target config doesn't work end-to-end |
| 1318 | Phase-1 exhaust | **NOT REFRESHED** (comment "Keep emergency_previous_config unchanged") | Coordination SET_CONFIG retries exhausted |
| 1496 | Turbo FORWARD probe BREAK (SNR target known) | `failed_config` | Turboshift probe failed mid-climb |
| 1501 | Turbo FORWARD probe BREAK (no SNR target) | `failed_config` | Turboshift probe failed mid-climb |
| 1572 | Turbo SWITCH_ROLE BREAK | `current_configuration` | SWITCH_ROLE NAck'd 3× during turbo |
| 1631 | Frame-gearshift-up failed BREAK | `working_config` (= config below failed target) | Up-shift control frame NAck |
| 1681 | Control-failure-threshold BREAK | `current_configuration` | `emergency_nack_count >= threshold` on control failures |
| 2189 | Frame-gearshift-up data failed BREAK | `working_config` | Data NAck immediately after up-shift |
| 2301 | Frame-gearshift data fail (pattern-ACK path) | `working_config` | Same, alternate ACK path |
| 2345 | **Block-failure-threshold BREAK** | `current_configuration` | `emergency_nack_count >= threshold` on data block failures (the canonical steady-state trigger) |

Read sites: lines 60, 62, 140, 147, 149 (all inside the BREAK-ACK
handler, lines 55-194). Read nowhere else.

## §3 What "Phase 1" is

Walking `arq_commander.cc:40-197` (`process_messages_commander()` — the
`emergency_break_active` branch):

1. **BREAK trigger** (e.g. line 2328-2357): sets
   `emergency_previous_config = current_configuration`,
   `emergency_break_active = 1`, `emergency_break_retries = 3`,
   `send_break_pattern()`.

2. **BREAK pattern → poll for ACK** (line 53-122): If ACK received,
   compute `target = config_ladder_down_n(emergency_previous_config,
   break_drop_step, robust_enabled)` (line 60). Then **double**
   `break_drop_step` (1→2→4, capped, line 64). Load
   coordination floor (ROBUST_0 or CONFIG_0, line 71-74). Queue
   SET_CONFIG carrying `target`. Set `break_recovery_phase = 1`,
   `break_recovery_retries = 2`. State → `TRANSMITTING_CONTROL`.

3. **Phase 1 — coordination SET_CONFIG at floor** (`break_recovery_phase
   == 1`): Send SET_CONFIG. If ACKed, line 3171-3183 transitions to
   `break_recovery_phase = 2`. If NOT ACKed:
   - Phase-1 retry left? Re-send (line 1300-1308). Do **not** touch
     `emergency_previous_config`.
   - Retries exhausted? Re-arm BREAK (line 1311-1331). Do **not**
     touch `emergency_previous_config` (line 1318 comment).

4. **Phase 2 — probe SET_CONFIG at target** (`break_recovery_phase ==
   2`): Send SET_CONFIG (line 3181) at the *target* config (already
   loaded by Phase-1 ACK processing in `process_control_commander()`).
   If ACKed, line 3184-3206 transitions to `break_recovery_phase = 0`,
   resets `break_drop_step = 1`, resumes data. If NOT ACKed:
   - Probe-retry left? Re-send (line 1250-1259).
   - Retries exhausted? **Refresh** `emergency_previous_config =
     current_configuration` (line 1278 — this is the target, not the
     floor, because we already moved here). Lower
     `supershift_proven_ceiling`. Re-arm BREAK.

The asymmetry: Phase-2 exhaust refreshes the anchor (line 1278) so the
next descent advances; Phase-1 exhaust does not (line 1318).

## §4 The descent walk worked example

Walk the math. Assume `robust_enabled = false` (no `-R`), BREAK fires
at CFG_15. `break_drop_step` starts at 1 (`arq_common.cc:341`).

| Iter | Event | emergency_prev | step (before/after BREAK-ACK) | target = ladder_down_n(prev, step) |
|------|-------|----------------|------------------------------|------------------------------------|
| 0 | BREAK at CFG_15 (line 2345 sets prev=15) | 15 | 1 / 2 | 14 |
| 1a | Phase-1 OK, Phase-2 OK at CFG_14 | — | reset to 1 (line 3187) | data resumes at CFG_14 |

OR if CFG_14 doesn't work:

| 1b | Phase-1 OK, Phase-2 EXHAUST at CFG_14 (line 1278: prev=14) | 14 | 2 / 4 | 12 |
| 2b | Phase-1 OK, Phase-2 EXHAUST at CFG_12 (line 1278: prev=12) | 12 | 4 / 4 (cap) | 8 |
| 3b | Phase-1 OK, Phase-2 EXHAUST at CFG_8 (line 1278: prev=8) | 8 | 4 / 4 | 4 |
| 4b | Phase-1 OK, Phase-2 EXHAUST at CFG_4 (line 1278: prev=4) | 4 | 4 / 4 | 0 (floor clamp, `common_defines.h:140`) |
| 5b | Phase-1 OK, Phase-2 OK at CFG_0 | — | reset to 1 | data resumes at floor |

The descent is bounded and progressive: 15 → 14 → 12 → 8 → 4 → 0 in
five BREAK rounds. Phase 1 not-refresh is irrelevant to the success
path here because Phase 1 succeeded each time (coordination floor is
reliable post-Bug-D).

If Phase 1 ALSO failed at iteration N (e.g. transient noise hides the
SET_CONFIG on the coordination layer):

| N.a | Phase-1 EXHAUST (line 1318: prev unchanged) | prev_N | 4 (unchanged) | prev_N - 4 (same as before) |
| N.b | New BREAK-ACK heard → target = prev_N - 4 (no progress) | prev_N | 4 (cap) | prev_N - 4 (re-tries same target) |
| N.c | This time Phase-1 succeeds, Phase-2 attempts (prev_N - 4) | — | — | — |

So a transient Phase-1 hiccup costs ONE redundant BREAK round on the
same target — but progress is preserved because the target was the
correct next step regardless. Refreshing prev during Phase-1 retry
would have ADVANCED the target on a Phase-1 hiccup, which is exactly
the WRONG action — the previous target was never disproved, only
un-delivered.

## §5 Why -R vs no-R changed the impact (history, pre-Bug-D)

Per `BREAK_FAILSAFE_INVESTIGATION.md` §11.4 the prior subagent flagged
the never-refresh as load-bearing only without `-R`. Reason:

- With `-R`: coordination floor = ROBUST_0 (32-MFSK 1/16, decodes at
  ~-12 dB Es/N0). Phase 1 always succeeded → Phase 2 path was always
  exercised → line 1278 refresh fired → descent advanced.
- Without `-R`: coordination floor = CONFIG_0 OFDM (BPSK 1/16). Phase 1
  could fail at low SNR. Combined with the pre-Bug-D RSP control-slot
  wedge (slot stuck non-FREE → SET_CONFIG silently dropped → Phase 1
  always fails), the loop **could never reach Phase 2**, line 1278 was
  never hit, and never-refresh-on-Phase-1 meant the descent target was
  permanently pinned at `original_X - 4` (`break_drop_step` cap).

Bug D (commit `1acdb3c`) removed the RSP wedge: post-BREAK
`messages_control.status = FREE` is now forced. Phase 1 is reliable
again on CFG_0 OFDM. The pathology is no longer reachable in normal
operation.

## §6 Proposed comment-only diff (ready for owner review, DO NOT APPLY)

Current `arq_commander.cc:1311-1331`:

```cc
                else
                {
                    // Phase 1 exhausted — BREAK again to resync
                    printf("[BREAK-RECOVERY] Phase 1 failed, re-sending BREAK\n");
                    fflush(stdout);
                    receiving_timer.stop();
                    receiving_timer.reset();
                    messages_control.status = FREE;
                    // Keep emergency_previous_config unchanged (still targeting original settle config)
                    emergency_break_active = 1;
                    emergency_break_retries = 1;
                    break_recovery_phase = 0;
```

Proposed:

```cc
                else
                {
                    // Phase 1 exhausted — BREAK again to resync
                    printf("[BREAK-RECOVERY] Phase 1 failed, re-sending BREAK\n");
                    fflush(stdout);
                    receiving_timer.stop();
                    receiving_timer.reset();
                    messages_control.status = FREE;
                    // Keep emergency_previous_config unchanged. Phase-1 failure
                    // means RSP did not HEAR the SET_CONFIG on the coordination
                    // layer — the target was never disproved, only un-delivered.
                    // Retry the SAME plan. The descent mechanism lives in the
                    // Phase-2 exhaust path at line 1278 (refresh on disproved
                    // target) and the break_drop_step doubling at line 64.
                    // See EMERGENCY_PREVIOUS_CONFIG_INVESTIGATION.md §1, §3.
                    emergency_break_active = 1;
                    emergency_break_retries = 1;
                    break_recovery_phase = 0;
```

Diff is 5 added comment lines, 1 deleted (the old WHAT comment). No
behavioral change. Owner sign-off required before commit.

## §7 §4.3.4 invariants — no impact

The seven safe-state invariants of `SACK_DESIGN_A_PLAN.md` §4.3.4 are
untouched by this analysis (this is a documentation change at most).
For the record:

| # | Invariant | Touched? |
|---|-----------|----------|
| 1 | One outstanding batch | No |
| 2 | batch_seq_id monotonicity | No |
| 3 | No silent corruption | No |
| 4 | Bounded recovery on single-axis failure | No (BREAK recovery is bounded by `break_drop_step` cap × ladder depth, independent of refresh policy) |
| 5 | Reversibility of any single policy move | No |
| 6 | Axis 1 supremacy | No (BREAK path already calls `policy_axis1_supremacy_on_move()` at line 1324 — was added in Step 12) |
| 7 | `supershift_proven_ceiling` analogue for Axis 2 | No |

## §8 Verdict-bearing facts (one-paragraph summary)

`arq_commander.cc:1278` refreshes `emergency_previous_config = current_configuration`
when Phase-2 (target probe) exhausts because the target has been
disproved. `arq_commander.cc:1318` does NOT refresh when Phase-1
(coordination SET_CONFIG) exhausts because the target has not been
disproved — only the delivery of the request to RSP failed. The
asymmetry is correct: refreshing on Phase-1 exhaust would conflict
with `break_drop_step`'s descent-doubling semantics (line 64) and
would falsely advance the descent on a transient
coordination-layer hiccup. The descent is driven by the
break_drop_step doubling on each BREAK-ACK and by the Phase-2 refresh
on each disproved target. Combined, the descent path is bounded and
progressive (§4 worked example: CFG_15 → 0 in 5 BREAK rounds). With
Bug D fixed (`1acdb3c`), Phase-1 is reliable on both ROBUST_0 and
CFG_0 floors, so the historical pathology where Phase 1 wedged
forever (pre-Bug-D, no-`-R`) is no longer reachable.

## §9 Open / [?]

- [?] **Is there a deterministic test that exercises Phase-1 exhaust
  in the post-Bug-D world?** None currently. The §11 walk in
  `BREAK_FAILSAFE_INVESTIGATION.md` shows BREAK descent works
  end-to-end but does not specifically force Phase-1 exhaust. A
  hand-crafted unit test that simulates a Phase-1 coordination ACK
  miss could verify the documented intent.
- [?] **Is the `break_drop_step` cap of 4 itself optimal?** Out of
  scope of this investigation. A larger cap (e.g. 8) would let the
  loop reach the floor in fewer BREAK rounds when the original failing
  config is high (e.g. CFG_15 → floor takes 5 rounds today;
  cap=8 would take 4). Not a correctness issue.

## §10 Status

- Verdict: **A (never-refresh is correct)**.
- Source edit: **none applied.** Proposed comment-only diff in §6 is
  ready for owner review.
- Cross-ref: `POST_BREAK_STUCK_INVESTIGATION.md` (Bug D), 
  `BREAK_FAILSAFE_INVESTIGATION.md` §8.2, §11.4 (the original flag).
- Plan-doc update: `SACK_DESIGN_A_PLAN.md` §7.13.16 added with verdict
  summary and pointer to this fact doc.

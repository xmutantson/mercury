# Data-flow: in-band connect-liveness guard (control-plane livelock backstop)

Status: ACTIVE. Built 2026-06-20 from the HW A/B livelock diagnosis (cycle3 ON arm).
Owner structure: the commander forward-progress watchdog under MERCURY_INBAND_RATE.
Pairs with the regression test `cl_arq_controller::test_inband_liveness()`
(`--test-inband-liveness`, wired into `--test`).

## §1 The defect (root cause, HW-verified)

On the in-band rate-adapt ON arm (`MERCURY_INBAND_RATE=1`) a deep-demote cycle delivered
**0 bytes** because of a **control-plane livelock**, NOT a down-ladder fault (the down-ladder
never ran on those cycles). HW saved-log signature (cycle3 ON): both ends `link_status==CONNECTED`,
but ~92% of polls report `connection_status: Transmitting control` with `nReceived_data=0` AND
`nAcked_data=0` for the whole run; ONE unilateral CONFIG 15->14 fired, still no data.

Why legacy (OFF) recovers and ON does not:

- Legacy escalates a missed control-ACK to a BREAK->ROBUST_0 resync (arq_commander.cc:2823-2867,
  the `else if(link_status==CONNECTED && ...)` branch) and recovers, delivering 76-96 KB.
- The redesign SUPPRESSES that escalation (arq_commander.cc:2806-2820, "BREAK escalation
  SUPPRESSED ... inband owns rate/loss"): a missed control-ACK is silently freed, no BREAK, no
  accounting. The design RETAINS a true-loss BREAK backstop, but it is wired ONLY to data-loss
  (`inband_cmd_dead_batch_floor_reached()` ticks only on a Class-A *data*-batch total-loss at the
  ladder bottom — arq_commander.cc:1623/3925/4146/4665). A connect/negotiate handshake that stalls
  with ZERO forward DATA progress never ticks that floor -> the retained BREAK never fires.
- The legacy session watchdog cannot catch it either: `link_timer` (link_timeout=10000ms,
  arq_common.cc:4745) is RESTARTED on every control-ACK (arq_commander.cc:2128) and every queued
  control message, so a link making *control-plane* progress but ZERO *data-plane* progress keeps
  kicking `link_timer` and never trips the 10s drop. `connection_attempt_timer`
  (connection_timeout=30000ms, arq_common.cc:4626) is gated on link_status in
  {CONNECTING,NEGOTIATING,CONNECTION_ACCEPTED} — the livelock is at link_status==CONNECTED, so it
  is out of scope. NET: under inband there is NO backstop for a control-plane livelock.

## §2 The guard (the fix)

A commander-side forward-progress watchdog, gated on `inband_rate_feature_enabled()` (legacy
byte-identical when OFF). Once per `process_messages_commander()` poll, BEFORE the dispatch:

- **Stall signal**: no advance in `stats.nAcked_data` (the commander's monotonic forward-DATA-
  delivered counter; incremented at arq_commander.cc:61/3508/3537/3563 on data ACKs; reset only at
  session init arq_common.cc:530) **while NOT in a data-bearing phase** (connection_status is
  neither TRANSMITTING_DATA nor RECEIVING_ACKS_DATA). On any advance OR any data-bearing poll the
  no-progress streak resets to 0. This is the exact HW signature: control-TX/Idle + nAcked_data flat.
- **Threshold N**: `INBAND_LIVENESS_STALL_POLLS = 200` consecutive no-progress control-plane polls
  (env override `MERCURY_INBAND_LIVENESS_POLLS`). Rationale: a normal connect+negotiate completes in
  a few control round-trips (low tens of polls at worst); 200 control-plane polls with ZERO data
  delivered is unambiguously a livelock, not a slow handshake. A normal data batch keeps
  connection_status in the DATA phases (streak reset every batch) so a slow batch cannot trip it.
- **Recovery action**: fire the SAME retained true-loss BREAK the §7 floor uses
  (`send_break_pattern()` + emergency-break state machine: emergency_break_active=1,
  emergency_break_retries=3, frames_to_read=4, receiving_timer.start()) — identical to
  arq_commander.cc:4665-4678. This is the design's retained true-loss backstop, re-wired to also
  fire on a liveness stall. BREAK->ROBUST_0 is exactly how legacy recovers the same channel.
- **Bound**: `cmd_inband_liveness_breaks` caps liveness-BREAKs per session at
  `INBAND_LIVENESS_MAX_BREAKS = 3`. On the 4th stall, escalate to a real session reset
  (link_status=DROPPED via the same path the link_timer watchdog uses) so the guard cannot thrash.
  Both counters reset on any data delivery and in reset_session_state().

## §3 Cross-layer audit (CLAUDE.md §5)

State the guard's recovery touches: it fires the EXISTING `send_break_pattern()` recovery, so it
inherits the audited BREAK semantics — it adds NO new mutation of batch/connect state beyond what a
§7 true-loss BREAK already does.

1. **Producers of the streak** (`cmd_inband_liveness_no_progress_polls`): the guard itself, once per
   poll (arq_commander.cc, top of process_messages_commander). Reset on data delivery (the §9.7
   `cmd_inband_session_dead_batches=0` success site, arq_commander.cc:4693) and reset_session_state.
2. **Consumers**: only the guard's own threshold test. No other layer reads it.
3. **Valid states before any producer writes**: 0 (ctor in-class init + reset_session_state). A
   fresh/idle session has streak 0; the guard never fires before N data-less control polls accrue.
4. **Invariants the recovery must preserve**:
   - rsp_current_expected_batch_seq_id / cmd_batch_seq_id: the BREAK recovery re-queues in-flight TX
     to the FIFO and rebuilds the batch at ROBUST_0 (arq_commander.cc:279-301), the SAME proven path
     legacy uses; no in-flight batch is orphaned that legacy would not also re-queue. The guard fires
     ONLY in control-TX/Idle with nAcked_data flat — i.e. when NO data batch is making progress — so
     it cannot interrupt a healthy mid-batch delivery.
   - link_status: the BREAK keeps link_status==CONNECTED and drives the emergency-break state machine
     to resync; the bounded escalation (4th stall) sets link_status=DROPPED via the audited
     link_timer watchdog path (reset_session_state + reset_all_timers).
   - 98edd4d deliver-before-break primitive (`deliver_complete_inflight_before_break`): the liveness
     BREAK fires only when nAcked_data is flat in control-TX/Idle — there is no COMPLETE in-flight
     data batch awaiting delivery at that point (a completing batch would advance nAcked_data and
     reset the streak), so the primitive is a no-op here, but the recovery path still routes through
     the same send_break_pattern() that 98edd4d guards, so no regression.
5. **What the fix changes**: it adds ONE new trigger for the already-audited true-loss BREAK. It does
   NOT alter the down-ladder, the tag-demote, the set_config=0/no-cascade win (those run in DATA
   phases where the streak is held at 0), or any byte-path. OFF -> the guard is never entered.

## §4 Regression (test_inband_liveness, --test-inband-liveness, in --test)

- **FAIL-BEFORE** (`-DINBAND_LIVENESS_FAILBEFORE`): the guard is compiled out -> a synthetic
  control-TX/no-data-progress poll sequence runs unbounded, send_break_pattern_count stays 0
  (livelock reproduced). The directed assert (a BREAK fires within the bound) FAILS.
- **PASS-AFTER**: the guard fires send_break_pattern() within INBAND_LIVENESS_STALL_POLLS and the
  bound caps it; a data delivery resets the streak (no false-fire during data flow); OFF arm never
  fires (byte-identical).

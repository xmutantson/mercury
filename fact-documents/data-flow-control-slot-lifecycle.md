# Data-Flow Audit — `messages_control` slot lifecycle (RSP RECEIVED-drop bug)

Status: ACTIVE. Created 2026-06-27 for the RX-CTRL-DROP fix (branch
`staging/rx-ctrl-drop`). This document OWNS the producer/consumer list for the
single `messages_control` mailbox slot. Every change to the slot's lifecycle
updates this doc (CLAUDE.md §5).

All `file:line` references are against the tree at branch `staging/rx-ctrl-drop`
(HEAD aa8606ec). Line numbers drift; re-grep `messages_control` if they no
longer match.

---

## §0 Summary of the bug

`messages_control` is a SINGLE slot (a one-deep mailbox, `arq.h:3509`). On the
RSP, an inbound CONTROL frame is copied into it ONLY when it is FREE
(`arq_responder.cc:825`); if it is already non-FREE the incoming frame is
**silently dropped** (the `[RX-CTRL-DROP]` log at `:848`, no recovery). The slot
has **no timeout / cleanup escape out of the RECEIVED state**: once a path leaves
it stuck in RECEIVED, every subsequent control frame (SET_CONFIG, SWITCH_ROLE,
REPEAT_LAST_ACK, CLOSE_CONNECTION, ...) is dropped for the life of the session →
the reverse control plane is dead → demote-amplifier / connect-fail / BREAK
spiral.

Four stuck-slot vectors leave the slot in RECEIVED with no consumer:

- **V1 — unhandled `(code, link_status)` fall-through.** `process_control_responder()`
  (`arq_responder.cc:2735`) is a chain of `if/else if`. The terminal `else`
  (`:3621`) handles ONLY `CLOSE_CONNECTION` (`:3623`); there is NO `else` after
  it and NO terminal FREE at the end of the function (`:3660`). Any control code
  that does not match a guard — OR any valid code arriving in a `link_status` the
  guards exclude (e.g. SET_CONFIG while not CONNECTED, START_CONNECTION while
  CONNECTED, a corrupted/unknown code byte) — returns with the slot LEFT in
  RECEIVED. The slot is never re-FREEd.

- **V2 — `link_timeout` / connection-timeout RESPONDER recovery.** `update_status()`
  link death (`arq_common.cc:5844`), connection-attempt timeout (`:5727`), and
  max-attempts (`:5797`) all call `reset_session_state()` then take a
  RESPONDER→LISTENING branch. The COMMANDER branches explicitly set
  `messages_control.status = FREE` (`:5749`, `:5785`, `:5833`, `:5889`); the
  RESPONDER branches (`:5896`, the bare `link_timeout` RESPONDER arm) do NOT.
  `reset_session_state()` never touches the slot (see V4). So a RESPONDER that
  times out with a RECEIVED slot keeps it stuck across the LISTENING transition.

- **V3 — watchdog RESPONDER recovery.** `update_status()` watchdog
  (`arq_common.cc:5904`): the COMMANDER branch FREEs `messages_tx[i]` (`:5913`),
  the RESPONDER branch FREEs `messages_rx[i]` (`:5945`). NEITHER frees
  `messages_control`. A watchdog recovery that fires with a RECEIVED control slot
  leaves it stuck.

- **V4 — crypto / STRICT early-returns + CLOSE_CONNECTION teardown rely on
  `reset_session_state()`, which does NOT free the slot.** `reset_session_state()`
  (`arq_common.cc:6856-7098`) resets ~every session field but NEVER assigns
  `messages_control.status` (verified: no `messages_control` write in the whole
  function body). The KEY_ACTIVATE-mismatch path (`arq_responder.cc:3186`) and
  the CLOSE_CONNECTION non-monitor teardown (`:3629`, `:3640`) call
  `reset_session_state()` and change `link_status`, leaving the slot RECEIVED.

---

## §1 Producers (every write to `messages_control.status`)

RSP receive path:
- `arq_responder.cc:825-841` — FREE→RECEIVED on inbound CONTROL copy (the ONE
  producer of RECEIVED on the RSP data path). Gated on `status==FREE`; else the
  frame is DROPPED (`:848` log).

RSP consume/teardown path (status writes inside `process_control_responder`,
`process_messages_acknowledging_control`, `process_buffer_data_responder`):
- `:502`, `:674` — FREE on BREAK / config-reset paths.
- `:1621` — RECEIVED→ACKED (the normal consume: control ACK queued).
- `:1834`, `:2031`, `:2433`, `:2590` — →FREE after ACK / branch completion.
- `:2760`, `:2828` — →FREE in the monitor / START_CONNECTION accept.
- `:3018`, `:3066` — →FREE in TEST_CONNECTION_ACK build.
- `:3209`, `:3223`, `:3289`, `:3398` — →FREE in SWITCH_BANDWIDTH / SET_CONFIG /
  SWITCH_ROLE branches.
- `:3505`, `:3554`, `:3593`, `:3618`, `:3635` — →FREE in SET_LINK_PARAMS /
  ROBUST_DWELL / CLOSE-monitor branches.

CMD path (the slot is reused as a PENDING_ACK TX mailbox on the CMD side):
- `arq_common.cc:9220` — →PENDING_ACK after a control frame is TX'd.
- `arq_common.cc:5718-5722` — PENDING_ACK→ACK_TIMED_OUT on the CMD ack-timeout
  (the EXISTING escape this fix MIRRORS for the RSP RECEIVED state).
- CMD timeout-recovery FREEs: `:5749`, `:5785`, `:5833`, `:5889`.
- `arq_commander.cc:1000` sets `ack_timeout`; `arq_commander.cc:5919` sets
  PENDING_ACK in a synthetic-fire test.

Construction / config:
- `arq_common.cc:1066`, `:1070` — `ack_timeout` init (NOT status).
- `reset_session_state()` (`:6856`) — **does NOT write status** (V4).

## §2 Consumers (every read of `messages_control.status` / `.data` that branches)

- `arq_responder.cc:279`, `:382` — defer-receive gate: only copy a new frame when
  `status==FREE` (so a stuck non-FREE slot BLOCKS all further control RX — this is
  the amplification mechanism).
- `arq_responder.cc:825` — the produce gate (FREE → copy; else drop `:848`).
- `arq_responder.cc:864`, `:1562`, `:1618` — RECEIVED → run
  `process_control_responder()` / queue the control ACK.
- `arq_common.cc:1064`, `:1085` — `PENDING_ACK` reads in resend bookkeeping.
- `arq_common.cc:5718` — `PENDING_ACK` ack-timeout (CMD escape).
- `arq_commander.cc` test asserts (`:4691`, `:4701`, `:5681`, `:6257`, ...).

## §3 Valid states + the BEFORE-any-producer default

`status ∈ {FREE, RECEIVED, ACKED, PENDING_ACK, ACK_TIMED_OUT}`.
Slot is allocated/`data` set up in `init()`; default after init is FREE.
The mailbox is one-deep: at most one control frame in flight per direction.

INVARIANT the consumers assume: **a RECEIVED slot is consumed (→ACKED→FREE, or
→FREE directly) within ONE control round-trip.** The producer gate
(`:279/:382/:825`) and the whole reverse control plane depend on the slot
RETURNING to FREE. Nothing enforced that invariant on the RSP — there was no
timeout escape (the CMD has one at `:5718`; the RSP RECEIVED state had none).

## §4 Invariants & how each producer must maintain them

1. Every path that ends a control frame's processing MUST return the slot to
   FREE (or ACKED, which the ACK path then drives to FREE). V1–V4 are the paths
   that violated this.
2. Every session-teardown (timeout / watchdog / reset / disconnect / role-switch)
   MUST FREE the slot. Centralizing the teardown FREE into
   `reset_session_state()` makes this hold for ALL teardown callers at once
   (fix change C).

## §5 What the fix changes (and the per-consumer walk)

Three changes, each restoring INVARIANT-1/2 at a different vector:

- **(A) RECEIVED-watchdog force-FREE** in `update_status()` — mirror the CMD
  PENDING_ACK escape (`:5718`). When the slot is RECEIVED and its per-slot
  `ack_timer` has run longer than a generous multiple of `ack_timeout_control`,
  force `status=FREE` + log. The `ack_timer` is idle in the RSP RECEIVED state
  (it is only armed for the CMD PENDING_ACK use), so reusing it is safe; it is
  STARTED at the RECEIVED producer (`:825` block). This is the catch-all escape
  for V1/V2/V3 regardless of which path stranded the slot.
  - Consumer walk: `:279/:382/:825` produce gate — after force-FREE the slot is
    FREE → next control frame copies in normally (the bug fix). `:864/:1562/1618`
    consume — unaffected (a RECEIVED slot still consumed normally on the fast
    path; the watchdog only fires after the round-trip window with no consume).
    CMD `:5718` PENDING_ACK escape — DISJOINT state (PENDING_ACK ≠ RECEIVED), no
    interaction. Multiplier chosen so the watchdog NEVER pre-empts a legitimate
    same-tick consume.
- **(B) catch-all FREE on the V1 fall-through** — add a terminal `else`/cleanup
  at the end of `process_control_responder()` so any unhandled `(code,
  link_status)` FREEs the slot immediately (no need to wait for the watchdog).
  - Consumer walk: only reached when no handler matched, i.e. the frame would
    otherwise have been a silent no-op leaving the slot stuck. FREEing it is
    strictly safe (the frame was not actionable). The CMD will retransmit if it
    needed an ACK — identical to a lost-on-air control frame.
- **(C) centralize teardown FREE into `reset_session_state()`** — add
  `messages_control.status = FREE;` (and `messages_control_bu.status = FREE;`) in
  `reset_session_state()`. This closes V2 (RESPONDER timeout arms), V3 (watchdog
  — note watchdog does NOT call reset, so it ALSO gets the explicit free in (A)),
  and V4 (crypto/CLOSE teardown).
  - Consumer walk: every existing `reset_session_state()` caller already treats
    the session as torn down; the explicit FREEs that several callers do AFTER
    `reset_session_state()` (`:5749`, `:5785`, `:5833`, `:5889`) become
    redundant-but-harmless (idempotent). No caller depends on the slot SURVIVING
    a reset (verified: no read of `messages_control.status` between a
    `reset_session_state()` call and the next produce gate).

## §6 Regression test

`cl_arq_controller::test_rx_ctrl_drop()` (CLI `--test-rx-ctrl-drop`), in-process
synthetic-fire (no PHY/IONOS/RF):
- FAIL-BEFORE arm (`-DRX_CTRL_DROP_FAILBEFORE`): the watchdog escape + the
  fall-through FREE + the reset FREE are compiled OUT. A slot stranded in RECEIVED
  stays stuck and a subsequent control frame is dropped → the pass-after assert
  fails.
- PASS-AFTER: strand the slot (RECEIVED, ack_timer past the watchdog bound), run
  `update_status()`, assert the slot is FREEd; then deliver a fresh control frame
  through the produce gate and assert it lands (status RECEIVED, the new code).
  Also asserts `reset_session_state()` FREEs a RECEIVED slot, and the V1
  fall-through FREEs an unhandled code.

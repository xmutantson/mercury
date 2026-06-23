# Data-Flow: Climb-UP `cmd_batch_seq_id` Rollback (CLIMB-CHURN)

Branch: `fix/climb-churn-bsi-rollback` (off `monitor` @ `3036767`).
Status: fix implemented + in-process fails-before/passes-after gate green; build +
master `--test` green. Throughput sim A/B (the ~83 bps clean-channel throttle
lift) QUEUED behind another agent's sim use — NOT yet run. VERIFIED items are
marked (V); INFERRED items (I).

## §1 Symptom

On a rapid mid-transfer clean-channel climb (CONFIG_0 → 16), bulk delivery
throttles to ~83 bps even though frames are ARQ-ACKed. Bytes are ACKed but never
pushed to the app FIFO; it self-resolves only after the config holds ~30 s and the
bsi epochs re-converge. Affects BOTH legacy and the in-band redesign. (I — symptom
from the upstream diagnosis; this branch fixes the mechanism + proves it in-process.)

## §2 Root cause (V — code-confirmed on monitor @ 3036767)

`cmd_batch_seq_id` is the CMD-side epoch counter for the NEXT new-data batch
(arq.h:2450, incremented at arq_commander.cc:2010 after each new-data `send_batch()`).

The DEMOTE / BREAK recovery paths roll it BACK to the stranded in-flight batch's
bsi BEFORE freeing `messages_tx[]`, so the re-presented batch stays CONTIGUOUS with
the RSP's preserved delivery high-water:
- FIX-9 D3 demote: arq_commander.cc:4057-4108 (captures `min_inflight_bsi`, rolls).
- M6 BREAK requeue: arq_commander.cc:4271-4300 (same capture+roll; default-ON).

The CLIMB-UP promote SET_CONFIG emits did **NOT** roll back — they re-present the
in-flight (already-RSP-delivered, not-yet-CMD-ACKed) batch under whatever ADVANCED
epoch the rapid climb reached:
- FRAME-UP gearshift climb: arq_commander.cc:4597-4615 (restores in-flight data to
  the TX FIFO + frees `messages_tx[]`, emits SET_CONFIG — NO bsi roll). PRIMARY site.
- Turbo OVER-CLIMB emitter `finish_turbo_direction()`: arq_commander.cc:~4787
  (`cleanup()` leaves un-ACKed in-flight frames, emits SET_CONFIG — NO bsi roll).
- Optimizer climb (FOURTH SET_CONFIG producer): arq_commander.cc:~696 (`cleanup()`
  + SET_CONFIG — NO bsi roll). Gated on `block_under_tx==NO`; a partial-SACK
  completion can still leave PENDING_ACK in-flight frames.

On the RSP, a mid-transfer config change re-baselines `cur=prev=-1` but PRESERVES
`rsp_last_delivered_batch_seq_id` (the high-water) — arq_responder.cc:3080-3099. The
next data frame re-adopts through `sack_v2_readopt_has_gap(adopted_bsi,
last_delivered)` (arq.h:1395). If the climb-up re-present carries an epoch ≥
last_delivered+2, that gate sees a ≥2 forward jump and HOLDS delivery
(`[RSP-V2-GAP-ABORT]`). The delivery-time backstop `delivery_step_is_gap`
(arq.h:1427) is the same ≥2 inviolable guard. The bytes are correct; only the
bsi-epoch LABEL is wrong. (V)

## §3 The fix

New shared helper `roll_back_cmd_bsi_to_inflight(tag)`
(arq_commander.cc:~4655, decl arq.h:~490): scans `messages_tx[]` for the EARLIEST
(mod-256) in-flight (status!=FREE && length>0) `batch_seq_id` and rolls
`cmd_batch_seq_id` back to it. Ported VERBATIM from the FIX-9 D3 / M6 capture loop.
Gated IDENTICALLY: `sack_v2_enabled && !compression_enabled` (no-op otherwise;
the compression path's `restore_tx_from_compressed()` owns its own re-stage). MUST
run BEFORE the caller frees `messages_tx[]`.

Wired into the three climb-up emit sites, each BEFORE any messages_tx free / cleanup:
- FRAME-UP gearshift: arq_commander.cc:4597 (before the FIFO-restore+free loop).
- Turbo over-climb: arq_commander.cc:~4787 (before `cleanup()`).
- Optimizer: arq_commander.cc:~696 (before `cleanup()`).

Does NOT loosen the RSP gap-gate (the no-silent-wrong-bytes backstop is untouched).

## §4 §5 Audit — producers / consumers of `cmd_batch_seq_id`

### Producers (writes) (V — full sweep, source/datalink_layer/*.cc)
1. arq_common.cc:622 — init to 0 (session start / reset).
2. arq_commander.cc:2010 — `+1 & 0xFF` after each new-data `send_batch()` (the
   normal advance; only when `batch_includes_new_data`).
3. arq_commander.cc:4113 — FIX-9 D3 demote rollback (DOWNWARD).
4. arq_commander.cc:4304 — M6 BREAK rollback (DOWNWARD; default-ON).
5. **arq_commander.cc:~4696 — NEW climb-up rollback helper (UPWARD).**
(Plus the in-process test harness scaffold seeds, which never run in production.)

### Consumers (reads) (V)
- arq_commander.cc:1762 — new-data fill stamps `messages_tx[i].batch_seq_id =
  (cmd_batch_seq_id & 0xFF)`. After the roll the re-encoded batch is stamped
  contiguously with the RSP high-water — the intended effect.
- arq_commander.cc:3114 — `sack_v2_bsi_in_window(rx_bsi, cmd_batch_seq_id)` SACK_RSP
  window gate. After the roll the window re-centers on the in-flight batch (the
  batch now being re-transmitted) — correct.
- The RSP does NOT read `cmd_batch_seq_id` (it runs its own `rsp_*` window). The
  arq_responder.cc:5525/5578 refs are the SACK in-window UNIT test only.

### Valid states / default-init
`cmd_batch_seq_id` starts 0. Before any new-data batch is staged it is the bsi the
NEXT batch will carry. The roll only fires when ≥1 in-flight non-FREE frame exists;
otherwise it is a NO-OP (helper returns -1, counter untouched) — so the
session-start / no-traffic / between-batch states are byte-identical to pre-fix.

### Invariants the consumers assume — preserved? (V)
- INV-1: all non-FREE frames of an in-flight block share ONE `batch_seq_id` (one
  batch in flight on the per-frame path) → "earliest mod-256" == the in-flight bsi.
  SAME assumption the FIX-9/M6 rollbacks rely on; unchanged.
- INV-2: the re-presented batch must be `==last_delivered` (dedup) or
  `==last_delivered+1` (contiguous successor) for the RSP to accept. The in-flight
  batch is by construction the un-ACKed-but-delivered (==hw) or
  un-ACKed-undelivered (==hw+1) batch → the roll lands EXACTLY in the accept set.
- INV-3: mutual exclusivity — a single poll climbs OR demotes OR breaks, each with
  an immediate `return`. No double-roll across the up + down rollbacks.
- INV-4: compression path untouched (gate excludes it; `restore_tx_from_compressed`
  owns its re-stage). v1 sessions never read the v2 gap-gate (gate excludes them).
- INV-5: gap-gate NOT loosened — `sack_v2_readopt_has_gap` / `delivery_step_is_gap`
  unchanged; the producer is corrected so the gate never needs to fire on a
  legitimate climb, while still catching a genuine dropped-batch hole.

### What the fix changes
Only the bsi LABEL of the re-presented climb-up batch (epoch → in-flight bsi).
Behaviorally identical to the proven default-on demote/BREAK rollbacks against the
SAME two consumers (:1762, :3114). No PHY/OFDM-config flow change; lossless-requeue
invariant preserved (the data bytes path — FIFO push-back / `restore_tx_from_*` —
is untouched; only bookkeeping is corrected).

## §5 Test (CLAUDE.md §3) — in-process, no PHY/audio

`test_climb_bsi_rollback()` (arq_commander.cc, decl arq.h, CLI
`--test-climb-bsi-rollback`, also in the master `--test` suite). Drives the REAL
producer (`roll_back_cmd_bsi_to_inflight`) + the REAL RSP predicate
(`sack_v2_readopt_has_gap`). Arms: ARM1 reproduction (hw=4, in-flight=5, epoch
outran to 8); ARM2 multi-frame same-batch + FREE-hole + near-wrap (hw=200);
ARM3 mod-256 WRAP (hw=255, in-flight=0, epoch=3); ARM4 gated NO-OPs
(compression-on, v2-off, no-in-flight).

Evidence (V):
- FAILS-BEFORE (helper neutered → return -1; rebuilt): non-defeat arm FAILS rc=1,
  6 failures — re-present stays at the advanced epoch (8) → gap vs hw=4 → RSP HOLD.
- PASSES-AFTER (fix restored): rolls 8→5 / 206→201 / 3→0, all contiguous, PASS rc=0.
- `MERCURY_CLIMB_BSI_ROLLBACK_DEFEAT=1` reproduces the bug in the shipped binary
  (no rebuild) for an A/B.
- Master `--test`: GREEN (rc=0), `[TEST-CLIMB-BSI] PASS` included.

## §6 Open

- [?] Throughput confirmation (realtime-sim clean-channel bulk A/B showing the
  ~83 bps throttle lifts) is QUEUED behind another agent's sim use; run + merge
  after it confirms the e2e win. Do NOT merge to monitor before then.

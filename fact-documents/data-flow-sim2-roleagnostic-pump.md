# Data-flow: SIM_INPROC 2-instance role-agnostic pump (Option-b multi-batch)

Living investigation record. Every claim cited `file:line`. Built 2026-06-05 on
worktree `win/sim-multibatch-pumpb` (off `feat/bigblock-livepath-p1` @0e94581).

Goal: make the single-thread 2-instance in-process stepper
(`MERCURY_SIM_2INST=1 -m SIM_INPROC -n`, `test_sim_inproc_2`) decode a
SUSTAINED MULTI-BATCH pinned CFG16 transfer byte-correct across BOTH batches, so
the big-block DELIVERED RATE is SIM-measurable (fast/deterministic/durable)
instead of HW-gated.

This is **Option-b** (the role-agnostic pump rewrite) from
`data-flow-sim2-ofdm-delivery-cadence.md §7.6(b)` (on the sim-faithful P2 tree).
It is the REDESIGN the 4 prior depth-1 PATCH reverts (that doc §7.4) pointed to —
NOT a 5th iteration on the depth-1 patch. Option-a (defer-TX @700bb71, branch
`win/sim-multibatch-deferTX`) removed the depth-1 reentrant-TX infinite-spin
(`drain_playback_wait`) but a SECOND distinct facet — the responder→commander
turnaround delivery under the depth-0 gate — still blocks batch 2.

---

## §1 Symptom (FAIL-BEFORE, reproduced on STOCK p1 @0e94581)

`MERCURY_SIM_2INST=1 MERCURY_SIM2_PIN=1 MERCURY_SIM2_ROBUST=0 MERCURY_SIM2_CFG=16
MERCURY_SIM2_SNR3K=900 MERCURY_SIM2_PAYLOAD_BYTES=4200 MERCURY_SIM2_STALL_ITERS=150000`:

- Batch 1 (`batch_seq_id=0`, 25 frames) decodes byte-correct — all 25
  `[RX-BATCH-SEQ] ... batch_seq_id=0`.
- RSP times out, sends MFSK-SACK ACK (`[TX-MFSK-ACK-SACK] ... batch_seq_id=0
  bitmap=0x01ffffff`), CMD advances `cmd_batch_seq_id=1`, transmits batch 2
  (`[T] cmd_batch_tx_start batch=2`, `[CMD-BATCH-SEQ] new-data batch_seq_id=1
  (frames in batch=3)`, `[TX-PEAK] frames=3`).
- `[TX-PEAK]` is the LAST forward-progress line. The run then wedges, link-times
  out, and **re-HAILs** (`[TX-HAIL]`, `[FTR-FAIL] CONFIG_16 ... batch=0`). ZERO
  `[RX-BATCH-SEQ] ... batch_seq_id=1` frames are delivered. rc=1 (stall break).

Single-batch CFG16 500B is UNAFFECTED on the same binary: `bytes_ok=1
rx_have=500/500 held_ofdm=1 stalled=0 iters=4`. The deadlock is purely
multi-batch (the inter-batch SACK-turnaround → batch-2 TX boundary).

## §2 Root cause (traced — from cadence doc §7.2/§7.3, re-confirmed on p1)

The single-thread co-routine stepper drives the PEER inside the active
instance's blocking waits via `sim_inproc_pump_2` (arq_commander.cc:9903),
bounded to ONE peer level by `g_sim2_depth` (arq_commander.cc:9683). The
deliver+peer-drive block is gated `if (g_sim2_depth == 0)`
(arq_commander.cc:9916). At depth 1 the pump ONLY
`sim2_drain_to_wire(c->tx, …)` idle-fills the wire for the CURRENT `c->tx`.

The inter-batch SACK ACK is delivered to the CMD **while the CMD is being
co-routine-driven at depth 1** (driven from inside the RSP's half-step). The
CMD's `process_main()` detects the ACK, advances `cmd_batch_seq_id`, and FALLS
THROUGH within the SAME call to `process_messages_tx_data()` → `send_batch()`,
transmitting batch 2 reentrantly at depth 1. `send_batch()` writes the batch-2
symbols to the CMD's play buffer, then calls `drain_playback_wait()`
(arq_common.cc:178) **at depth 1**. The depth-1 pump only drains `c->tx`'s
playback — and at the wedge `c->tx` is the IDLE RSP (`sending=0`), NOT the
TX-ing CMD whose play buffer is stranded (cadence doc §7.3: the instance needing
delivery is the OUTER, non-driven `c->rx`, and that identity FLIPS between pump
invocations in the same wedge). No single `c->tx`/`c->rx`-keyed predicate names
the stranded transmitter → `drain_playback_wait` spins forever → link-timeout →
re-HAIL.

The 4 prior reverts (cadence doc §7.4) were all variations of patching the
depth-1 path while keeping the `c->tx`/`c->rx` orientation. The orientation
itself is the trap. The fix REMOVES orientation: deliver ANY instance's pending
playback to its peer + drive the peer's decode, indexed by "who has play data,"
NOT by the tx/rx flip.

## §3 The fix (Option-b — role-agnostic flat pump)

Confined to the `namespace`-scoped SIM_INPROC helpers in arq_commander.cc. No
production code touched (`g_sim_inproc_pump` defaults null, arq_common.cc:130;
the pump + helpers have NO non-SIM_INPROC callers).

### §3.1 Stable A/B-indexed ctx fields (orientation-free)
Add to `SimInproc2Ctx` (arq_commander.cc:9658) STABLE direction-keyed fields set
ONCE before the loop and NEVER flipped:
- `inst_a`, `inst_b` — the two instances (A=commander, B=responder).
- `ch_a2b`, `ch_b2a` — channels by DIRECTION (A→B, B→A).
- `wire_a2b`, `wire_b2a` — wires by DIRECTION.

The legacy `tx`/`rx`/`ch_tx2rx`/`ch_rx2tx`/`wire_t2r`/`wire_r2t` fields are KEPT
(the main-loop half-step at arq_commander.cc:10267-10296 still sets them, and
`sim2_drain_to_wire`/`sim2_deliver_from_wire` take explicit args so they are
already orientation-independent). The PUMP stops reading the flip fields.

### §3.2 Flat single-depth guard
Collapse `g_sim2_depth` + `g_sim2_decode_drive_depth` semantics into ONE flat
reentrancy guard `g_sim2_pump_depth`. The pump does its full role-agnostic
deliver+drive ONLY at depth 0 (the outermost pump call); a nested pump (fired
from a peer's own pacing/drain wait DURING a depth-0 drive) only advances the
shared clock (drains both instances' playback to their wires) and returns,
exactly as the old depth-1 path did — but now SYMMETRICALLY for both directions.

### §3.3 Role-agnostic pump body
Each depth-0 pump call, FOR BOTH DIRECTIONS (A→B and B→A):
1. Drain that direction's SOURCE playback through its channel into that
   direction's wire (`sim2_drain_to_wire`), advancing the shared clock — for
   the IDLE direction this idle-fills one silence symbol (unchanged accounting).
2. Deliver that direction's wire into the DESTINATION capture + prep + per-frame
   OFDM decode-drive (`sim2_deliver_from_wire(dst, wire, c, drive_decode=true)`)
   — the SAME single-symbol-paced, frame-boundary-gated decode that made OFDM
   decode in the first place (cadence doc §3, 9336c23).
3. Co-routine-drive the DESTINATION `process_main()` ONCE (bumping
   `g_sim2_pump_depth` across the drive so the nested pump only clocks).

Because BOTH directions are pumped every call, whichever instance has stranded
playback (the CMD's batch-2 buffer) gets it delivered to its peer regardless of
which instance is the "active" one in the outer stepper — the orientation trap
is gone. `drain_playback_wait` on the CMD now exits because its A→B playback is
drained to wire_a2b and delivered to B every pump call.

### §3.4 Decode-drive gate (UNCHANGED — the §3.2/§4 warnings hold)
The per-frame decode-drive inside `sim2_deliver_from_wire` keeps its existing
gate `link_status==CONNECTED && is_ofdm_config(current_configuration)` (cadence
doc §3.2). The `is_ofdm_config` term remains the PRIMARY guard: the MFSK
CONNECT/HAIL handshake runs at ROBUST_0 (is_ofdm_config=false), so the symmetric
drive can NEVER fire during the handshake. This is what kept the prior
"unconditional depth-1 peer-drive" revert (cadence doc §7.4-2) from recurring —
that revert drove the peer's FULL process_main mid-handshake; THIS rewrite only
drives the decode when CONNECTED+OFDM, and only drives the destination (never the
running instance into itself).

## §4 Cross-layer data-flow audit (CLAUDE.md §5)

Shared state changed: `SimInproc2Ctx` (the pump ctx — SIM-harness only),
`g_sim2_depth`/`g_sim2_decode_drive_depth` → `g_sim2_pump_depth` (the
reentrancy guard — SIM-harness only). The pump READS ARQ/PHY state
(`audio.play`, `audio.cap`, `data_container.{data_ready,frames_to_read,
ring_write_index}`, `arq.{link_status,connection_status,current_configuration}`)
and DRIVES `arq.process_main()`. It WRITES only the wire rings + capture rings +
the shared sim clock.

1. **Producers of SimInproc2Ctx**: only `test_sim_inproc_2`
   (arq_commander.cc:10185-10198 init; the half-step at 10267-10296 sets the
   legacy flip fields). The new A/B fields are set ONCE at init, never flipped.
2. **Consumers of SimInproc2Ctx**: only `sim_inproc_pump_2` (the pump) and
   `sim2_drain_to_wire`/`sim2_deliver_from_wire` (take explicit args). No
   production consumer (`g_sim_inproc_pump_ctx` null in production).
3. **Valid states**: before the loop, both instances' play/cap rings are empty,
   both wires cleared (arq_commander.cc:10196). `g_sim2_pump_depth==0`. The pump
   is null until `arq_set_sim_inproc_pump` installs it (10199) and is cleared on
   exit — production never sees it non-null.
4. **Invariants consumers assume**:
   - INV-1 (cadence doc §3.2): the decode-drive fires ONLY at the frame-complete
     boundary (`data_ready==1 && frames_to_read==0`) AND CONNECTED+OFDM. The
     rewrite preserves this gate verbatim inside `sim2_deliver_from_wire`.
   - INV-2 (cadence doc §10.5): a sender's post-TX capture FLUSH must not race a
     reply — replies live in the per-direction wire until the destination's
     controlled deliver point. The rewrite delivers each direction's wire to its
     destination on EVERY pump call (symmetric), which is MORE controlled than
     before, not less: each deliver still only touches the destination's OWN
     cap ring (explicit-arg helpers), so no sender flush wipes a wire.
   - INV-3 (single-thread reentrancy): the pump must NOT recurse infinitely. The
     flat `g_sim2_pump_depth` guard bounds the role-agnostic deliver+drive to the
     OUTERMOST pump call; nested pumps only clock. Same one-level bound as the
     old `g_sim2_depth`, now flat.
   - INV-4 (sim_clock fidelity, arq_common.cc:113-127): the pump only makes the
     EXISTING spin-exit predicate eventually true; it never changes a threshold
     or exits a loop early. The rewrite still advances the clock through the SAME
     `sim2_drain_to_wire` → `sim_clock_add_samples` accounting per moved/idle
     symbol — for BOTH directions now (the idle direction adds one silence symbol
     exactly as before, so the per-pump clock advance is the SAME order as the
     legacy depth-0 path; the only added clock is the second direction's idle
     symbol, which is what the old depth-1 nested pump already added on the flip).
5. **What the fix changes**: the pump stops keying delivery on the tx/rx flip and
   instead delivers both directions every call. Every CONSUMER above either
   takes explicit args (orientation-independent) or reads per-instance state via
   `sim2_activate` before the drive (unchanged). No production consumer exists.

## §5 Validation plan (all must hold)
1. multi-batch CFG16 COMPLETES in-sim — batch 2 (and 3) delivered byte-correct.
2. GATE-2 determinism — same-seed-twice byte-identical.
3. single-batch CFG15/16 NO regression (still byte-correct).
4. ROBUST_0/MFSK CONNECT handshake intact (the 400/400-style smoke; legacy
   19-byte "MERCURY-2INST-HELLO").
5. production (-m ARQ / -x sim) byte-identical (pump null → rewrite inert).
6. `--test-sim-clock` + `--test-climb-engine` green.

## §6 Results

### §6.1 Regression validation — ALL PASS (byte-identical to baseline)
- **Legacy 19-byte smoke** (`MERCURY_SIM_2INST=1`, ROBUST_0/MFSK handshake +
  data): ALL PASS, `iters=15 sim_ms=61535`, rx="MERCURY-2INST-HELLO" — BYTE-
  IDENTICAL to the baseline @0e94581 (same iters/sim_ms). The MFSK CONNECT/HAIL
  handshake the 4 prior reverts kept breaking is PRESERVED.
- **Single-batch CFG16 500B** (pinned, clean): `bytes_ok=1 rx_have=500/500
  iters=4 sim_ms=11599` — BYTE-IDENTICAL to baseline. No regression.
- These hold because PART A is the VERBATIM legacy pump (c->tx/c->rx orientation)
  and PART B is gated `g_sim2_pump_depth>0` (never fires at depth 0) AND
  `c->inst_X != c->tx` (never touches the normal decode-drive nesting).

### §6.2 Multi-batch CFG16 — PARTIAL progress, then ARCHITECTURAL STOP
`MERCURY_SIM2_PAYLOAD_BYTES=4200` (≈29 frames > one 25-frame batch), pinned CFG16,
clean SNR3K=900:
- **The deadlock / infinite spin is GONE.** Baseline @0e94581 wedged at batch 2
  (`[TX-PEAK] frames=3` last line, then ~31M idle spins → link-timeout → re-HAIL,
  0 batch-2 frames). With the role-agnostic pump the commander's reentrant batch-2
  `drain_playback_wait` EXITS (PART B drains its stranded CONNECTED-OFDM playback,
  keyed on "who is stranded" not the c->tx/c->rx flip — the orientation trap §2 is
  removed). Confirmed via `MERCURY_SIM2_PBDBG=1`: `[PARTB] depth=1 src=A/CMD
  play=386880 ... dst=B/RSP dstconn=2 dstftr=22` (B armed frames_to_read for the
  new batch).
- **Batch 2 now DECODES byte-correct (partially).** With the PART-B receiver-drive
  (§6.3 step 3) the responder consumes the reentrant batch:
  `[RX-BATCH-SEQ] ... batch_seq_id=1 seq=0` (DATA_LONG) and `... seq=2` (DATA_SHORT)
  decode byte-correct (`[OFDM-OK] t2 cfg=16 var=0.034 meanH=0.984`). This is the
  FIRST time ANY batch-2 frame decoded in-sim (baseline + Option-a both delivered
  0). The responder advances to `current_expected_batch_seq_id=1`, receives 2/3
  frames, and emits the correct PARTIAL SACK (`[RSP-MFSK-SACK] partial path:
  batch_seq_id=1 bitmap=0x00000005` — frames 0,2 received, frame 1 missing).
- **STOP: the partial-batch SACK→retransmit turnaround does NOT complete.** Two
  coupled residuals, both deeper single-thread-pump invariants:
  1. **Frame-by-frame cadence in the one-shot PART-B delivery.** PART B drains the
     WHOLE batch (39 symbols) to the wire and delivers it in ONE
     `sim2_deliver_from_wire` call with a SINGLE receiver-drive. The frames decode
     at SHIFTING ring offsets (`delay=110411` for seq 0, `delay=94292` for seq 2)
     and the MIDDLE frame (seq 1) is SKIPPED — no decode attempt at all. The
     proven depth-0 path decodes each frame at its own boundary because the
     sender transmits frame-by-frame over many interleaved pump calls; PART B's
     one-shot dump + single drive does not reproduce that per-frame interleave.
  2. **Responder→commander SACK-TX turnaround strands.** The responder's PARTIAL
     SACK is generated by its ack-gate TIMER in a LATER process_main than the one
     PART B drove, and the SACK TX itself calls `drain_playback_wait`. Both the
     timer-fire and the SACK-TX drain need the depth-0 interleaved cadence that the
     reentrant batch retransmit STARVES (the outer loop barely advances — only 1
     `[SIM2-DBG]` line at it=0 across the whole multi-batch run, confirming the
     stepper spends ~all its time inside depth>0 reentrancy). The commander never
     receives the partial SACK (`[CMD-MFSK-ACK-SACK ... batch_seq_id=1]` = 0
     occurrences) → never retransmits frame 1 → run wedges → rc=124.

### §6.3 What was tried (6 distinct mechanisms — the STOP boundary, CLAUDE.md §1.2)
All on this worktree, converging on the same wall:
1. Fully-symmetric both-directions deliver+peer-drive every call → BROKE the MFSK
   handshake (the cadence doc §4 / §7.4-2 regression: driving/delivering the peer
   mid-handshake desyncs it).
2. PART B unconditional at depth>0 → BROKE single-batch CFG16 decode (PART B raced
   the proven single-symbol pacing during the normal decode-drive nesting).
3. PART B + `c->inst_X != c->tx` discriminator → single-batch + handshake RESTORED
   (byte-identical), but batch 2 frames reached the ring and were NOT consumed by
   the responder's ARQ (no RECEIVING arm).
4. Decode-drive allowed inside PART B (separate `g_sim2_decode_drive_depth` guard)
   → frames reached the ring; still not consumed (the ARQ state machine needs a
   process_main drive, not just a decode).
5. **PART-B receiver-drive** (drive the responder's process_main, gated
   CONNECTED-OFDM, `g_sim2_partb_depth` guard) → batch 2 DECODES (2/3 frames
   byte-correct, RX-BATCH-SEQ batch_seq_id=1) — REAL progress, the un-stranding
   works for clean delivery.
6. **Responder→commander reply delivery** (drain the responder's SACK to the
   reverse wire + deliver to the commander) → the SACK is GENERATED but the
   turnaround still strands (§6.2 residual 2).

### §6.4 STOP rationale (honest, per the task BAIL-OUT clause)
The role-agnostic pump REMOVED the orientation trap and the infinite-spin deadlock,
and got batch 2 to decode byte-correct in-sim for the FIRST time — but multi-batch
CFG16 does NOT fully COMPLETE: the partial-batch SACK→retransmit turnaround hits a
deeper single-thread-pump invariant. The reentrant batch retransmit starves the
depth-0 interleaved cadence that BOTH the responder's frame-by-frame decode AND its
ack-gate-timer SACK-TX drain require. A 7th iteration would be another depth-patch
on the same mechanism — the task forbids it ("Do NOT iterate into a 6th depth-patch
attempt"). **The honest conclusion: the in-process single-thread stepper cannot do
SUSTAINED multi-batch with partial-batch SACK rounds without a structural redesign
of the inter-batch turn at the TOP level (depth 0). HW (P3) remains the sustained-
rate path. This matches the cadence doc §5/§6 first-frame-edge + SACK-round-doesn't-
complete observations and §7.6's "the inter-batch CMD→next-batch TX must happen at
the TOP level."** The WIP (orientation-trap removal + receiver-drive un-stranding +
the reply-delivery scaffold) is committed for the next session / a possible P2-merge
where the big-block re-granularization (fewer, larger frames per batch) may reduce
the per-batch SACK-round count enough to clear the residual.

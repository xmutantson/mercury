# data-flow: SIM_INPROC 2-instance time-domain faithfulness + multi-batch defer-TX

Living investigation record. Every claim cited `file:line`. Built 2026-06-05.

This document owns the cross-layer audit for two related SIM_INPROC changes:
- **§1-§5** (impairment-worktree `win/sim-faithful`): the time-domain channel
  impairments (SFO/CFO/AGC) added to `cl_sim_awgn` so the in-process 2-instance
  sim reproduces the LEVER-P MINI-preamble cascade off-bench.
- **§6** (this worktree `win/sim-multibatch-deferTX`): the Option-(a) depth-gated
  defer-the-TX fix for the multi-batch reentrancy deadlock, with the MANDATORY
  TRANSMITTING_DATA re-entry cross-layer audit (CLAUDE.md §5).

---

## §6. Multi-batch deadlock — Option (a) defer-the-TX + re-entry audit

### §6.1 Symptom (fail-before, reproduced)
`MERCURY_SIM_2INST=1 MERCURY_SIM2_CFG=16 MERCURY_SIM2_PIN=1 MERCURY_SIM2_SNR3K=900
MERCURY_SIM2_PAYLOAD_BYTES=8000` hangs (rc=124 timeout). Batch 1 delivers all 25
frames (`[RX-BATCH-SEQ] ... batch_seq_id=0`, 25/25), batch 2 STARTS
(`[T] cmd_batch_tx_start ... batch=2`) but delivers 0 frames
(`batch_seq_id=1 frames: 0`) → infinite spin. Verified on the pre-fix sibling
binary (`/tmp/mb_prefix.log`).

### §6.2 Root cause (code-verified)
The single-thread 2-instance stepper co-routine-drives the PEER inside the active
instance's blocking waits via `sim_inproc_pump_2` (arq_commander.cc:~9920), bounded
to one peer level by `g_sim2_depth`. The depth-0-only deliver/drive block is gated
at `if (g_sim2_depth == 0)` (arq_commander.cc:~9916); at depth>0 the pump ONLY
drains+clocks the OTHER direction.

When the COMMANDER is itself the peer being driven at depth 1 and it starts a NEW
data batch, `process_messages_tx_data()` → `transmit_batch()` → `tx_transfer()`
(fills the play buffer) → `drain_playback_wait()` (arq_common.cc:4046) spins until
the play buffer drains. But at depth>0 the pump never moves the commander's play
buffer to the wire (depth-0 deliver gated off) ⇒ the play buffer never drains ⇒
infinite spin.

### §6.3 The fix (Option a)
Depth-gated early-return at the TOP of `process_messages_tx_data()`
(arq_commander.cc:1288):
```
if(arq_sim_inproc_active() && arq_sim2_current_depth() > 0)
    return;
```
- `arq_sim_inproc_active()` (arq.h:92, def arq_common.cc:193) is non-null ONLY
  under the SIM_INPROC stepper ⇒ production (-m ARQ) is byte-identical (the
  guard short-circuits, no behaviour change).
- `arq_sim2_current_depth()` is a new external-linkage accessor for the file-scope
  `g_sim2_depth` (arq_commander.cc, declared just above the anonymous namespace so
  both the namespace pump and the global accessor see it; the member fn precedes
  the pump and reaches it via a forward declaration). depth>0 ⇒ reentrant.
- The return leaves `connection_status == TRANSMITTING_DATA` untouched, so
  `process_commander()` (arq_commander.cc:532 `else if(connection_status==
  TRANSMITTING_DATA)`) re-enters `process_messages_tx_data()` on the NEXT
  top-level (depth-0) tick (arq_commander.cc:591), where the TX runs normally and
  the pump CAN drain the play buffer.

Option (b) (the role-agnostic pump rewrite) was explicitly NOT taken (2-3 day
durable consolidation, out of scope).

### §6.4 TRANSMITTING_DATA re-entry cross-layer audit (CLAUDE.md §5)
Shared state crossed: `messages_batch_tx[]` (ARQ batch array), `cmd_batch_seq_id`
(ARQ SACK-v2 batch counter), `block_under_tx` (ARQ TX gate).

**1. Producers (writers) of these structures inside process_messages_tx_data():**
- `messages_batch_tx[i].status = ADDED_TO_BATCH_BUFFER` — arq_commander.cc:1369
  (v1 retx), :1593 (mixed/new-data builder). Both AFTER the early-return point.
- `cmd_batch_seq_id = (cmd_batch_seq_id + 1) & 0xFF` — arq_commander.cc:1895.
  AFTER the early-return point.
- `block_under_tx = YES` — arq_commander.cc:1402 (retx), and per-batch in the
  builder. AFTER the early-return point.

**2. Consumers (readers) on the next top-level tick:**
- `process_commander()` re-dispatch (arq_commander.cc:532/591) — reads only
  `connection_status` to decide to re-call. Unchanged on the deferred path.
- `process_buffer_data_commander()` (arq_commander.cc:10746) — reads
  `block_under_tx`, `retransmit_count`, `fifo_buffer_tx`. Unchanged on deferred.
- SACK/ACK handlers read `cmd_batch_seq_id`, `messages_batch_tx[]` — unchanged.

**3. Valid states BEFORE any producer writes (the deferred precondition):**
The early-return fires BEFORE `mtl::log_event("cmd_tx_data_entry")` and BEFORE any
of the §6.4-1 writes. Therefore on the deferred path NONE of the three structures
is mutated: they sit in EXACTLY the state the previous depth-0 tick left them
(post-ACK cleanup: covered slots ACKED→FREE; cmd_batch_seq_id at its last value;
block_under_tx at its last value). This is the identical precondition the batch
builder expects on a normal (depth-0) entry.

**4. Invariants the consumers assume — verified maintained:**
- INV-A "cmd_batch_seq_id increments exactly once per dispatched batch"
  (data-flow-robust-tier-arq-batch.md). The deferred path does NOT reach line 1895
  ⇒ no spurious increment. The batch is dispatched once, on the depth-0 re-entry.
- INV-B "messages_batch_tx[] is rebuilt from scratch each batch (counter reset)".
  The deferred path does not touch the array; the depth-0 re-entry rebuilds it
  normally (message_batch_counter_tx reset at the top of the builder).
- INV-C "block_under_tx==YES gates new-data staging until finalize".
  The deferred path does not flip it; the gate state is preserved across the defer.

**5. What the fix changes:** it adds ONE additional code path (deferred no-op
return) that runs ONLY under SIM_INPROC at depth>0. Because it mutates nothing,
every consumer sees the pre-fix state on re-entry. No consumer assumption is
violated. The only externally observable difference is TIMING: the batch is
dispatched one+ top-level ticks later. In the sim the shared clock advances only
when the pump moves samples (arq_commander.cc:9716 sim_clock_add_samples); a
deferred return adds zero samples, so the dispatch slips by ~0 sim-clock time ⇒
NO ack/link/watchdog timeout is perturbed (all timers run on sim-clock samples).

### §6.5 Validation
- Fail-before: rc=124 deadlock at batch 2 (§6.1).
- Pass-after: see §6.6 (multi-batch CFG16 delivers batch-2+ in-sim).
- GATE-2 determinism + production -m ARQ byte-identity preserved (the guard is
  inert when the pump is null).

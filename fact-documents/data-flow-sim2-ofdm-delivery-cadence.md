# Data-flow: 2-instance in-process sim — OFDM delivery cadence (wf-sim-controlloop)

Status: FIX SHIPPED (worktree `sim-controlloop`, branch `wf-sim-controlloop`,
on top of `fb05338`). Makes the 37×-FTRT 2-instance in-process stepper
(`test_sim_inproc_2`, `MERCURY_SIM_2INST=1`) OFDM-DATA-capable so it can validate
the throughput regime (CFG15/CFG16), not only MFSK/ROBUST_0.

This document covers the shared state changed by the fix: the WIRE→capture→ring
delivery cadence in the SIM_INPROC-only helpers
`arq_commander.cc:sim2_drain_to_wire` / `sim2_deliver_from_wire` /
`prep_pull_inline`. It is a sim-harness fact doc (no production code touched).

## §1 Symptom (FAIL-BEFORE, fb05338)

Pinned CFG15 (`MERCURY_SIM2_PIN=1 MERCURY_SIM2_ROBUST=0 MERCURY_SIM2_CFG=15
MERCURY_SIM2_SNR3K=900`, clean AWGN) delivered ZERO OFDM bytes:
`rx_have=0/1000 bytes_ok=0`, stalled. The OFDM acquisition jittered:
- `[OFDM-SYNC] coarse: pream_symb=` jumped wildly between attempts
  (28,27,19,11,5,33,47,62,…) — the preamble's ABSOLUTE ring offset moved by
  whole symbols between consecutive decode attempts.
- `[OFDM-OK] t0` (preamble) often passed, then `[OFDM-FAIL] t2 … var=nan
  meanH=nan crc=0x805A` — the DATA-symbol decode landed on garbage because the
  snapshot window was not frame-aligned.
MFSK/ROBUST_0 (the legacy 19-byte cell) was unaffected (400/400) — it survives
identical jitter via its wide-margin 1-D symbol-grid re-lock
(telecom_system.cc:1114-1154).

## §2 Root cause (cadence, not DSP)

The DSP is correct (decodes on RF + in the BER self-test). The bug is the
in-process FEED cadence:

1. `sim2_drain_to_wire` (arq_commander.cc:~9693) drains the WHOLE play buffer
   into the wire when the sender is transmitting (one frame in one shot), but
   idle-fills only ONE silence symbol per pump when the sender is idle.
2. `sim2_deliver_from_wire` (original) drained the ENTIRE wire into the RX
   capture, then ran `prep_pull_inline` ONCE. `prep_pull_inline` then walked
   ALL of capture into the ring, advancing `ring_write_index` by a VARIABLE
   count (a variable idle-silence run + the whole data frame) BEFORE any decode.
3. The RX runs its OFDM decode only inside `process_main()`, which executes once
   per top-level half-step (and once per pump co-routine drive). The decode
   consumer (arq_common.cc:6214) snapshots the 134-symbol window ONLY when
   `frames_to_read==0`. Because prep over-advanced the ring past the frame
   boundary (clamping `frames_to_read` at 0 and continuing to feed), the single
   decode saw a jittered, over-advanced window → wrong FFT window → var=nan.

In PRODUCTION the capture-prep thread feeds ONE symbol per `sim_paced_wait`
tick (audioio.c:1295-1376) while the decode thread polls `data_ready`
CONCURRENTLY, so it consumes each frame the moment `frames_to_read` hits 0 at a
frame-aligned `ring_write_index`. The sim collapsed that concurrency into a
single serialized "dump-all-then-decode-once" step.

REFUTED hypotheses (confirmed not the cause): ring_write_index jumps in
prep_pull_inline (it advances by exactly symbol_period/iter, byte-identical to
audioio.c:1366-1367); sp_max staleness (OFDM failed even pinned same-cfg).

## §3 Fix (confined to the SIM_INPROC helpers, 9693-9743 region)

Two parts, one logical change to the delivery cadence:

### §3.1 Single-symbol pacing (necessary)
`sim2_deliver_from_wire`: move EXACTLY ONE symbol wire→cap, then
`prep_pull_inline` (cap now holds one symbol → prep advances the ring by ONE
symbol), per loop iteration. Mirrors the production 1-symbol-per-tick feed.

### §3.2 Per-frame decode drive (sufficient)
Single-symbol pacing ALONE was insufficient — the once-per-half-step decode
still over-advanced the ring across the whole delivered run before snapshotting
(verified: reaches the data phase but t2 var=nan, 0 t2-OK). So, on the
frame-complete boundary (`data_ready==1 && frames_to_read==0`), drive
`dst->process_main()` ONCE so it snapshots+decodes at the frame-aligned ring
offset before more symbols shift the window — reproducing the production
concurrent decode-thread timing.

GATE (critical — see §4): `link_status==CONNECTED &&
is_ofdm_config(current_configuration)`. The `is_ofdm_config` term is the PRIMARY
guard: the MFSK CONNECT/HAIL handshake runs at ROBUST_0 (is_ofdm_config=false),
so the drive can NEVER fire during the handshake (which works 400/400 and
desynced when driven). ROBUST/MFSK DATA also stays on the legacy
once-per-half-step cadence (byte-identical). Re-entrancy is guarded by
`g_sim2_decode_drive_depth` (a nested deliver fired by the pump from dst's own
pacing wait does not recurse the drive) and `g_sim2_depth` is bumped across the
drive so any nested pump only drains+clocks (skips its depth-0 deliver/drive
block, exactly like the existing co-routine peer-drive).

drive_decode=true at: the depth-0 pump rx site (PRIMARY data path — data frame
delivered to rx during tx's send wait) and the two top-level rx (B) catch-up
sites. NOT at the §10.5 reply-into-tx site (dst==the running tx instance —
driving it would recurse into itself) nor the A-target reply sites (A receives
ACK/control there; left on the legacy cadence to avoid perturbing A's
commander-side gearshift — A was observed to climb 15→16 when driven there).

## §4 Negative results (do NOT redo these)

- drive_decode at ALL sites incl §10.5 reply-into-tx → handshake hang
  (recursive process_main into the running tx instance).
- Gate on `connection_status==RECEIVING` (instead of/added to CONNECTED) →
  REGRESSED the decode (RX-BATCH=0). Twice. Keep the gate strictly
  `link_status==CONNECTED`.
- `break` out of the deliver loop after one decode → partial windows (the
  anti-spin re-arms frames_to_read to 8, a non-frame-aligned value). Worse.
- Per-SYMBOL drive (data_ready==1 without frames_to_read==0) → slow + 0 t2-OK.
  The frame-boundary (frames_to_read==0) trigger is required.

## §5 VALIDATION (FAIL-BEFORE → PASS-AFTER)

Pinned, clean AWGN (`MERCURY_SIM2_PIN=1 MERCURY_SIM2_ROBUST=0
MERCURY_SIM2_SNR3K=900 MERCURY_SIM2_PAYLOAD_BYTES=1000`):

- **CFG16 (default seed)**: `bytes_ok=1`, `rx_have=1000/1000`, `held_ofdm=1`,
  `stalled=0`, iters=4. `[OFDM-OK] t2 cfg=16 var=0.0019 meanH=1.000`, delay
  ~138879 STABLE, coarse ≥0.998. Full batch decoded byte-correct in-stepper.
- **CFG15 (seed=1, seed=7)**: `bytes_ok=1`, `rx_have=1000/1000`, iters=4.
  `[OFDM-OK] t2 cfg=15 var=0.0026 meanH=1.000`, delay ~142599 STABLE
  (`pream_symb=113` consistent, no jitter), coarse 0.997-0.999. `ACK-GATE PASS:
  received 6/6`.
- **CFG15 default seed 12345 / seed 99**: frames DECODE byte-correct (RX-BATCH-SEQ
  DATA_LONG, t2 var=0.0022 meanH=1.000, the full SACK→retransmit ARQ loop
  functions), but the FIRST frame (seq 0) is missed at the CONNECTING→CONNECTED
  edge → a SACK retransmit round that does not complete within the 10-min wall
  cap (the symbol-paced sim clock crawls through the long ~10-15 s ACK timeouts).
  This is a seed-dependent first-frame TIMING edge + harness throughput limit,
  NOT an OFDM-decode failure (other seeds at the SAME config deliver bytes_ok=1).
  Broadening the gate to catch seq 0 (the RECEIVING term) regressed the decode
  (§4), so it was reverted; the first-frame edge is left as a known harness
  limitation, tracked here.

GATE-2 determinism: same seed twice → BYTE-IDENTICAL on BOTH the legacy cell
(iters=15 sim_ms=61587 pump.calls=509) AND the OFDM cell (CFG15 seed=1: iters=4
sim_ms=12788 pump.calls=225 looped/idle/clock identical).

Legacy MERCURY_SIM_2INST=1 (no extra env, ROBUST_0/MFSK): BYTE-IDENTICAL to
baseline (iters=15, rx="MERCURY-2INST-HELLO", ALL PASS).

`--test-climb-engine` ALL PASS (0 failures); `--test-sim-clock` 0 failed.

Production byte-identical: `sim2_deliver_from_wire` / `sim2_drain_to_wire` /
`prep_pull_inline` have NO non-SIM_INPROC callers; `g_sim_inproc_pump` defaults
null (arq_common.cc:130) and is installed only by the SIM_INPROC steppers, so
`-m ARQ` / paced `-x sim` never run any of this code.

## §6 Open item

CFG15 default-seed first-frame (seq 0) miss at the CONNECTING→CONNECTED edge.
Root: the decode-drive gate (`link_status==CONNECTED`) is not yet satisfied when
seq 0 is fed; the RECEIVING broadening that would catch it regresses the steady
decode. A correct fix likely needs a one-shot "first OFDM data frame at the
link-up edge" drive that does not use the RECEIVING term. Out of scope for the
delivery-cadence fix; the OFDM-capable goal (byte-correct in-stepper decode of a
pinned CFG15/CFG16 batch) is met for both configs.

## §7 MULTI-BATCH RE-LOCK — ARCHITECTURAL STOP (2026-06-05, on top of 6f3c83a)

**Goal:** make a 2+-batch pinned CFG16 transfer (clean SNR3K=900, deterministic
floor ON) decode every frame byte-correct across BOTH batches. **Result: STOPPED
— deep architectural property of the single-thread co-routine stepper. 4 fix
attempts, all reverted; tree restored to 6f3c83a (validator intact, single-batch
+ GATE-2 + production byte-identical all confirmed PASS).**

### §7.1 Symptom (reproduced, deterministic)
Pinned CFG16, `MERCURY_SIM2_PAYLOAD_BYTES=4200` (≈29 frames > one 25-frame SACK
batch). **Batch 0 (25 frames) decodes byte-correct** (`[OFDM-OK] delay=137691`,
all 25 `RX-BATCH-SEQ batch_seq_id=0`). The RSP times out, sends the MFSK-SACK ACK
(`[TX-MFSK-ACK-SACK] Done, flushed capture buffer, ftr=23`), CMD advances to
`cmd_batch_seq_id=1`, transmits batch 1 (`[TX-PEAK] frames=3 size=80600`) — and
**the run wedges**: `[TX-PEAK]` is the last forward-progress line, then ~31M idle
pump spins, eventually a link-timeout → re-HAIL. Batch 1 NEVER decodes (0
`batch_seq_id=1` frames delivered).

### §7.2 Root cause (traced, not inferred) — reentrant batch-N TX at depth 1
The deadlock is a single-thread co-routine REENTRANCY/ORIENTATION boundary:

1. The inter-batch SACK ACK is delivered to the CMD **while the CMD is being
   co-routine-driven at depth 1** (driven from inside the RSP's half-step). The
   CMD's `process_main()` detects the ACK, advances `cmd_batch_seq_id`, and
   FALLS THROUGH within the SAME call to `process_messages_tx_data()` →
   `send_batch()`, transmitting batch 1 — i.e. it **initiates a fresh multi-frame
   DATA batch TX reentrantly at depth 1**.
2. `send_batch()` writes the 39 batch-1 symbols to its play buffer
   (`[TX-PEAK]`), then calls `drain_playback_wait()` — **at depth 1**. (Confirmed:
   `[WAIT-DRAIN] play=386880 active=A/CMD` spins forever; 386880 B = 48360 doubles
   = 39 symbols = the WHOLE batch, none drained.)
3. The pump's deliver+peer-drive block is gated `if (g_sim2_depth == 0)`
   (arq_commander.cc:9916). At depth 1 it is SKIPPED — the depth-1 path only
   `sim2_drain_to_wire(c->tx, …)` idle-fills the wire. So the batch-1 TX audio is
   never delivered to the peer; the CMD's `drain_playback_wait` spins forever.

Batch 0 works because it is transmitted at the **top level / depth 0** (the
main-loop CMD half-step `A->process_main()` directly), where the depth-0 block
DOES deliver A.play→B + decode-drive. The stepper supports a peer doing RECEIVE +
short reply at depth 1, but NOT a peer INITIATING a multi-frame DATA TX there.

### §7.3 Why it is architectural (the orientation trap — the STOP reason)
The fix direction (deliver the reentrant depth-1 TX to the peer) is correct, but
the stepper's `c->tx`/`c->rx` orientation at the wedge is **not consistently the
TX-ing instance**. Instrumented at the live deadlock (pump heartbeat + per-call
gate trace):
- One sampled depth-1 pump invocation: `c->tx=B/RSP sending=0 txcfg=16
  c->rx=A/CMD rxplay=386880` — the TX-ing instance (CMD, with the stranded play)
  is the OUTER, non-driven `c->rx`, while the driven `c->tx` is the idle RSP.
- The drain at (1) (`sim2_drain_to_wire(c->tx, …)`) only ever touches `c->tx`, so
  it never drains the stranded CMD playback regardless of depth gating.

The instance that needs its playback delivered is whichever side is NOT the
current depth's `c->tx`, AND that identity differs between pump invocations
within the SAME wedge (the flip swaps it each level). There is no single
`c->tx`/`c->rx`-keyed predicate that reliably names the stranded transmitter —
fixing it robustly requires restructuring the co-routine drive/flip model so any
instance with pending playback is delivered independent of its tx/rx role. That
is a redesign of the stepper's core reentrancy model, not a localized helper fix.

### §7.4 Fix attempts (all tried, all reverted)
1. **Depth-1 decode-drive-only** (deliver `c->wire_t2r`→`c->rx` + bounded
   decode-drive, no peer process_main): batch-1 preamble now SEEN
   (`metric=0.631 delay=137689`, was 0.000) but never locks; re-wedges. Delivery
   reaches the peer but the bounded drive doesn't run the RSP's full receive loop.
2. **Unconditional depth-1 peer-drive** (`g_sim2_depth <= 1`, full deliver+drive):
   **REGRESSED the MFSK CONNECT/HAIL handshake** — wedged at the HAIL stage, 0
   CONNECT. Confirms the §3.2 warning that driving the peer mid-handshake desyncs
   it (the handshake is non-OFDM at ROBUST_0).
3. **Narrow-gated depth-1 peer-drive** keyed on `c->tx` (sending && CONNECTED &&
   `is_ofdm_config(c->tx…)`): handshake restored, but the gate NEVER fires at the
   wedge — at depth 1 `c->tx` is the idle RSP (`sending=0`), not the TX-ing CMD.
   Same `metric=0.631` partial + re-wedge.
4. **Orientation-corrected — stranded-`c->rx` drain** (drain `c->rx`'s play via
   `ch_rx2tx`/`wire_r2t`, deliver to `c->tx`): the gate FIRED 0 times — at the
   live wedge `depth1_rx_ofdm_tx` is never simultaneously true (the orientation at
   the deadlock differs from the heartbeat-sampled orientation; they are distinct
   pump invocations in the same wedge). Same wedge → link-timeout → re-HAIL.

### §7.5 Validation of the STOP (no regression on revert)
After reverting all 4 attempts (tree == 6f3c83a):
- **Single-batch CFG16 (500 B)**: `rx_have=500/500 bytes_ok=1`, ALL PASS.
- **GATE-2**: legacy 19 B smoke decodes "MERCURY-2INST-HELLO", ALL PASS, and is
  byte-identical across two same-seed runs (deterministic).
- **Production byte-identical**: only the SIM_INPROC stepper helpers were ever
  touched; `git status` shows NO tracked-file changes vs 6f3c83a.
- `--test-sim-clock` (0 failed) + `--test-climb-engine` (ALL PASS) green.

### §7.6 What a real fix needs (for the next session)
Restructure the stepper so the inter-batch CMD→next-batch TX happens at the TOP
level (depth 0). Two candidate directions (NOT yet attempted — would be a fresh
plan, not a 5th iteration on the same mechanism):
- **(a) Defer the reentrant TX:** make the depth-1 co-routine drive let the CMD
  only PROCESS the ACK (update batch_seq_id / mark ACKed) and RETURN before
  `send_batch()`, so the next main-loop CMD half-step (depth 0) transmits batch 1
  on the working depth-0 path. Requires a SIM_INPROC-only "do not start a new
  batch TX while `g_sim2_depth>0`" guard at the `process_messages_tx_data` entry
  (test-harness gate, production untouched) — and verifying the ARQ state machine
  re-enters the TX cleanly next top-level tick.
- **(b) Role-agnostic pump:** rebuild the pump to deliver ANY instance's pending
  playback to its peer + drive the peer's receive loop, indexed by "who has play
  data" rather than the `c->tx`/`c->rx` flip — removing the orientation trap. This
  is the larger redesign §7.3 implies.

## §8 BIG-BLOCK live-path: geometry-helper ring-zeroing under single-symbol pacing (2026-06-06, diag/livepath-sim)

**Context.** `--test-bigblock-livepath` drives the real gearshift robust→CFG16
SET_CONFIG + `send_batch` → `bigblock_send_one_block` (TX emits, `bbtx>0`
confirmed) + the cw0-CRC carve gate. The gate RUNS (B reaches CFG16) but the
cw0-CRC REJECTS and full byte-faithful delivery is blocked. The prior diagnostic
(arq_commander.cc:11305-11331) attributed this to the big-block geometry helpers
calling `bigblock_restore_stock_config()` → a full `load_configuration` reinit
that ZEROES the RX ring mid-accumulation under the sim's single-symbol pacing.
This section is the Phase-1 VERDICT + the mandated cross-layer audit.

### §8.1 VERDICT: (A) SIM-ONLY — NOT a production RX bug

`bigblock_restore_stock_config()` (telecom_system.cc:8056-8063) sets
`current_configuration = CONFIG_NONE` then calls `load_configuration(stock)`.
Because `current_configuration==CONFIG_NONE`, load_configuration forces a FULL
reinit (`reinit_subsystems = st_reinit_subsystems()`, telecom_system.cc:8664-8674)
→ `reinit_subsystems.data_container==YES` (telecom_system.cc:4743-4756) →
`data_container.set_size()` which `memset`s `passband_delayed_data` +
`ready_to_process_passband_delayed_data` to 0 and reallocates them
(data_container.cc:171-173 memset, :248-249 free in deinit). So EVERY restore
ZEROES + REALLOCS the RX ring. This is real shared-RX-state destruction.

**The difference is WHEN it fires relative to block accumulation:**

PRODUCTION continuous-stream RX (the real path):
- The audio capture-prep thread fills the ring CONCURRENTLY, one symbol per
  `sim_paced_wait` tick (audioio.c:1295-1376), decrementing `frames_to_read`.
- `receive()` (arq_common.cc:6897) only enters the decode block when
  `frames_to_read==0`; while `>0`, the WHOLE block accumulates and NO geometry
  helper / receive_byte / restore runs.
- At `frames_to_read==0` the full window is memcpy'd to
  `ready_to_process_passband_delayed_data` (arq_common.cc:6902) then
  receive_byte→receive_bigblock runs. receive_bigblock SNAPSHOTS `data` into a
  LOCAL std::vector BEFORE any restore (telecom_system.cc:8255-8260), so its own
  restore (telecom_system.cc:8329) zeroing the ring cannot corrupt the block being
  decoded.
- After the carve, `frames_to_read` is re-armed to `block_nsymb+10`
  (arq_common.cc:7127); the NEXT block accumulates from a clean ring. The restore
  fired at an INTER-BLOCK quiescent point.
- The other restore sites (`bigblock_block_ftr_or` via `bigblock_rx_block_nsymb`,
  called at TX-turnaround re-arms arq_common.cc:4991/5103/5575) fire right after
  the modem finished a TX and flushed its capture buffer to begin a fresh listen —
  again, no in-flight block accumulation to corrupt.
- CONCLUSION: NO production restore fires while a block's already-captured samples
  sit mid-accumulation in the ring.

SIM single-symbol-paced RX (the artifact):
- `sim2_deliver_from_wire` (arq_commander.cc:9994) runs a tight `while` loop:
  deliver one symbol → `prep_pull_inline` → and on the falling edge of
  `frames_to_read==0`, drive `process_main()` (→ decode → carve → restore →
  re-arm `frames_to_read=block_nsymb+10`). The loop then KEEPS DELIVERING the
  remaining symbols already queued in `dst->audio.cap`/wire in the SAME iteration
  burst, writing them into the JUST-REALLOCATED/ZEROED ring at a desynced offset.
  When `frames_to_read` next hits 0, the snapshot reads silence/garbage → cw0-CRC
  fails. Production has no "already-buffered future-block symbols" at the instant
  of reallocation because its feed is real-time concurrent, not a pre-filled wire.

### §8.2 Production-impact one-liner
NOT a production bug. The ring-zeroing fires only at inter-block/turnaround
quiescent points on the real continuous-stream RX. It does NOT explain the HW
`bbtx=3 / 0-bytes-delivered` symptom — that is the testbed channel-emulator analog
low-pass collapse (MEMORY.md `testbed_emulator_lowpass_collapse`), a physical
hardware fault, not this code path.

### §8.3 Cross-layer data-flow audit (CLAUDE.md mandate)

State under change: the SIM-harness delivery cadence around the RX ring
(`passband_delayed_data` / `ready_to_process_passband_delayed_data`),
`current_configuration`, and the block-accumulation gate (`frames_to_read`).

1. **Producers** (who WRITES the ring / accum state):
   - PROD capture-prep: audioio.c:1295-1376 (1 symbol/tick, decrements
     frames_to_read).
   - SIM capture-prep equivalent: `prep_pull_inline` arq_commander.cc:10049-10095
     (1 symbol/iter; decrements frames_to_read, sets data_ready).
   - Ring REALLOC/ZERO: `data_container.set_size` data_container.cc:95-173 (called
     from load_configuration telecom_system.cc:4750/4754, reached via
     `bigblock_restore_stock_config`).
   - Post-carve manual ring wipe: arq_common.cc:7119-7123 (zeros both ring halves
     after a block decode).
   - frames_to_read re-arm: arq_common.cc:7127 (post-carve), :4991/:5103/:5575
     (TX-turnaround via bigblock_block_ftr_or), :6446/:6857/:6871 (HAIL/ACK poll).
2. **Consumers** (who READS the ring / accum state):
   - PROD decode: `receive()` arq_common.cc:6897-7012 (snapshots ring→
     ready_to_process when frames_to_read==0, then receive_byte/receive_bigblock).
   - receive_bigblock telecom_system.cc:8214-8337 (reads `data`, immediately
     snapshots to local buffer 8255-8260).
   - SIM decode-drive: sim2_deliver_from_wire arq_commander.cc:10019-10038
     (drives process_main on the frames_to_read==0 falling edge).
3. **Valid states / pre-write defaults**: BEFORE any producer, the ring is
   memset-0 (data_container.cc:171). frames_to_read default after a fresh arm =
   block_nsymb+10 (the count of symbols to accumulate before the next snapshot).
   `bigblock_last_rx_K` default 0 (carve gate false until a block decodes).
4. **Invariants the consumers assume**:
   - INV-1: when `receive()`/decode-drive fires (frames_to_read==0), the ring's
     `signal_period` window holds EXACTLY one fully-accumulated block at a
     frame-aligned offset.
   - INV-2: no producer reallocates/zeros the ring while a partially-accumulated
     block's samples are live in it.
   PROD maintains both (restore only at quiescent boundaries; current block
   snapshotted before restore). SIM VIOLATES INV-2: the deliver loop pours the
   next block's queued symbols into the reallocated ring before the next snapshot.
5. **What the fix changes** (see §8.4): the SIM deliver loop must not pour
   post-restore symbols into the ring within the same burst — it must let the
   reallocated ring re-accumulate the next block from clean state, exactly as
   production's concurrent feed does. Production code is UNTOUCHED, so every
   production consumer is unaffected by construction.

### §8.4 Fix (Phase 2) — SIM-harness only
See the live-path test + sim2_deliver_from_wire change on diag/livepath-sim
(commit recorded below). One change: after a decode-drive fires the carve+restore
(falling edge of frames_to_read), STOP pouring further symbols into the ring in
the same deliver burst — return so the reallocated ring re-accumulates the next
block cleanly on subsequent deliver calls, mirroring production's concurrent
feed. Production-path code is byte-identical (the change is inside the
SIM_INPROC-only sim2_deliver_from_wire helper).

### §8.5 Block fits ONE batch (why this is the ONLY blocker)
At CFG16 K=8: 8 codewords × ldpc.K(1400) = 11200 info bits = 1400 wire bytes/block;
the APP capacity is 1400 − hdr_total(2+2K=18) − K·CRC(8) = **1374 bytes/block**
(arq_common.cc:3753-3762). The test PAYLOAD=1374 → the WHOLE payload is ONE big
block = ONE batch. So this transfer NEVER hits the §7 multi-batch reentrancy
wedge; the geometry-helper ring-zeroing was the sole remaining blocker to full
1374/1374 byte-faithful delivery on the live path.

### §8.6 RESULT after the ring-preservation fix (commit on diag/livepath-sim)
- **Ring-zeroing FIXED**: `--test-bigblock-livepath` snapshot rms 0.000000 →
  0.047630 (block now PRESENT; `[OFDM-SYNC]` preamble acquires metric=0.992).
  No regression: pinned fullpath 1200/1200 (8/8 clean), multicw ALL PASS,
  arq-unit 8/8, climb-engine ALL PASS, legacy 2INST smoke byte-identical.
- **Full byte-faithful delivery PROVEN on the PINNED full path**
  (`--test-bigblock-fullpath`: 1200/1200 bytes, first_block clean=8/8) through the
  REAL receive_bigblock + de-whiten + carve + FIFO path; `--test-bigblock-multicw`
  K=8 full-block byte-faithful (ALL PASS).
- **UNPINNED live-handshake full delivery NOT yet reached**: the FIRST big-block
  lands at the robust→CFG16 SET_CONFIG transition edge where the sim decode-drive
  window arming is not yet settled, so it misses cw0-CRC and the symbol-paced clock
  crawls through the SACK-retransmit ACK timeout past the wall cap. SAME class as §6
  (first-frame at link-up edge), NOT a delivery-path defect (identical decode path
  delivers 1200/1200 when settled). Follow-on: a one-shot "first OFDM block at the
  PHY-switch edge" decode-drive that does not depend on the steady gate.
- **THROUGHPUT vs VARA (clean channel, deterministic on-air airtimes)**: Mercury
  production PPMd8+zstd streaming compresses Project Gutenberg #84 (pg84,
  448,885 B) → 121,496 B (**3.695×**). Block carries 1374 compressed app bytes in
  1.6533 s airtime (6649 bps compressed wire). effective = orig·8 / (88.4 blocks ·
  cycle): airtime-only ceiling **24,564 bps**; nominal 913 ms turnaround
  **15,825 bps**; conservative 1014 ms **15,226 bps**. **WIN across the whole band
  vs VARA HF Standard 13,048 bps.** Clean-channel number (no SFO/CFO, matching
  VARA's bar); HW confirmation pending (testbed emulator low-pass collapse).
  Numbers + harness: `bigblock_p3_hw/results_simproof.json`,
  `bigblock_p3_hw/measure_compress.cc`.

## §9 MULTI-BATCH WEDGE — 2nd ASSAULT (2026-06-08, bb-d3 @9ee13f6) — STILL ARCHITECTURAL

Re-attacked the §7 wedge on `fix/bigblock-d3-carve`. **Result: re-confirmed
ARCHITECTURAL STOP — 4 MORE fix attempts (8 total with §7.4), all reverted; tree
restored to 9ee13f6.** New facts beyond §7:

### §9.1 The bb-d3 BASELINE is broader-broken than §7 framed
At clean 9ee13f6 the SIM_INPROC stepper delivers **ZERO OFDM data bytes on ANY
free-flow arm**, not just multi-batch:
- Legacy 19-B short-text arm (ROBUST_0/MFSK): **WORKS** — `rx="MERCURY-2INST-HELLO"`,
  CONNECT clean, 0 deadlock-breaks. (The MFSK/ROBUST handshake + small-payload path
  is the ONLY healthy delivery path.)
- `--test-bigblock-multicw` (PINNED CFG16, K=8 single block, STOP_AFTER_FIRST_BLOCK):
  **PASSES its unit asserts** (arm A clean=8/8 byteok) — but that is the narrow
  pinned single-block path, NOT free-flow.
- **Free-flow `MERCURY_SIM2_PAYLOAD_BYTES` (the discriminator's CONTROL): 0 bytes.**
  • Clean climb 20000 B: `switch_seq 100->101->102->0->100`, reaches CONFIG_0, decodes
    9 `RX-BATCH-SEQ` frames, then collapses to ROBUST_0; `rx_bytes=83/20000 stalled=1`,
    2 deadlock-breaks (matches `DISCRIMINATOR_RESULT.json` byte-for-byte).
  • **Pinned CFG16 500 B / 1500 B: ALSO 0/N**, with deadlock-breaks DURING the ROBUST_0
    CONNECT handshake (128960/327360-B tails abandoned) — i.e. even the pre-OFDM
    handshake TX wedges when the run is destined for CFG16. So this is NOT a
    "batch-0-works, only multi-batch-sustain-fails" gap (the §7.1 framing); the
    free-flow OFDM data delivery is broken from the first SACK batch.
  • The memory note "pinned CFG16 bytes_ok=1 in iters=4" was on `wf-sim-controlloop`
    @6f3c83a (the §7 base), NOT on bb-d3 — bb-d3 has diverged.

### §9.2 The 4 new attempts (what + why each failed)
1. **§7.6 (a) — `process_messages_tx_data` depth>0 deferral guard** (SIM-only, +accessor
   `arq_sim_inproc_depth()`): batch-0 delivers, but batch-1 STARVES. Live trace
   (`MERCURY_SIM2_TXDBG`): after batch-0 ACK the CMD is in TRANSMITTING_DATA (conn=1)
   but is ONLY ever entered at depth=1 (1234 consecutive deferrals) — the depth-0
   re-entry the guard relies on NEVER happens because the stepper is wedged inside the
   peer's nested wait (the iteration counter stays `it=0` through the whole window).
   Link-timeout → re-HAIL → mutual `RECEIVING (conn=2)` deadlock. (a) is FALSIFIED: the
   deferred-to depth-0 tick is unreachable while the peer holds the stepper.
2. **§7.6 (b) — blanket symmetric pump at every depth>0** (absolute-identity a/b
   drain+deliver, drive_decode=false): REGRESSED CONNECT — the symmetric capture feed
   desyncs the MFSK handshake (re-confirms §7.4 #2).
3. **Symmetric pump GATED to a nested-TX-drain window** (`g_sim2_in_nested_drain`
   bracket in `drain_playback_wait` at depth>0) + CONNECTED+OFDM gate: still breaks the
   pinned-CFG16 turnaround — the gate fires during batch-0's own SACK-ACK turnaround
   (B's ACK at CFG16 is `is_ofdm_config==true`) and the out-of-cadence delivery corrupts
   the decode pacing.
4. **Saturation-keyed stranded-transmitter drain** (`sim2_pump_drain_stranded`: at
   depth>0, key on the global `playback_buffer`==a/b to name the ACTIVE transmitter
   ORIENTATION-FREE, act ONLY when its outgoing wire has SATURATED — the precise wedge
   signal, no-op on a normal turnaround that never fills the wire): does NOT regress the
   legacy arm (verified identical), but still does not DELIVER the free-flow batch
   (frames decode — 9 `RX-BATCH-SEQ` — but `rx_have=0`; delivery-to-FIFO is blocked
   separately, and the ROBUST handshake wedge is upstream of the OFDM gate).

### §9.3 Why localized fixes keep failing — the irreducible coupling
The single-thread stepper couples THREE things that production runs concurrently on
separate threads: (i) the depth-0 per-frame DECODE-DRIVE (needs exact single-symbol
pacing, no interference), (ii) the SACK-ACK turnaround (small nested TX that must NOT
trigger symmetric delivery), and (iii) a nested multi-frame batch TX that MUST be
delivered. No single predicate (depth / decode-drive-depth / CONNECTED+OFDM /
wire-saturation / active-instance) cleanly separates (ii) from (iii) AND leaves (i)
untouched — every gate that catches (iii) also perturbs (i) or (ii). This is the §7.3
orientation/reentrancy trap restated with the full coupling.

### §9.4 The ONLY faithful fix is the stepper-core redesign (§7.6 implied, now mandatory)
Decouple `send_batch()` from the BLOCKING `drain_playback_wait()` under SIM_INPROC:
the OUTER stepper loop must be the SOLE DAC-drain + RX-consume driver (one symbol per
instance per outer iteration, both directions, like the two real audio threads), and
`send_batch` must QUEUE-and-return rather than spin-drain. That removes the nested-drain
reentrancy entirely (no depth>0 TX-drain ever exists), so (i)/(ii)/(iii) stop sharing a
call stack. This is a multi-day stepper rewrite with real regression risk to the
VALIDATED MFSK/legacy/bigblock-unit paths and a full GATE-2 + `--test-sim-clock` +
`--test-climb-engine` re-validation — NOT a localized helper patch. Recommend a fresh
design doc + plan-first (CLAUDE.md §4) before any code. Per CLAUDE.md §1.2 (≥3 failed
attempts ⇒ STOP, architectural) no further localized iteration on the current stepper.

## §10 STEPPER-CORE REWRITE — Phase b §1.5 audit + the no-op-drain change (2026-06-08, bb-d3)

Phase a (commit `535f573`) added the outer-loop symbol-pump skeleton behind
`MERCURY_SIM2_STEPPER=outer` (drain-before-tick, fixed A-before-B, one-symbol helpers
`sim_step_drain_one_symbol`/`sim_step_feed_one_symbol`), with `drain_playback_wait`
NOT yet changed. Phase b makes the SIM_INPROC drain QUEUE-and-return.

### §10.1 The state the fix changes (5-question §1.5 audit)
**State:** `drain_playback_wait()` exit semantics (`arq_common.cc:178`); `playback_buffer`
occupancy; the per-direction WIRE rings; the post-TX RX-ring bookkeeping at each of the 9
drain sites.

1. **Producers of `playback_buffer`:** every TX site via `tx_transfer`→`write_buffer`
   (`audioio.c:1691`); the call sites surrounding the 9 drains (`arq_common.cc:4998/5450/
   5461/5695/5701/5832/5836/6294/6301/6505/6509/6687/6691/7032/7036`). DRAIN (occupancy
   shrink): TODAY the pump `sim2_drain_to_wire`; AFTER this fix (under `outer`) the outer
   loop's `sim_step_drain_one_symbol`.
2. **Consumers of `playback_buffer`-empty:** ONLY `drain_playback_wait` itself
   (`arq_common.cc:215` exit predicate + `:213` NO-PROGRESS floor) and the harness
   `wire_quiescent` SET_CONFIG gate (`arq_commander.cc:10845`, reads `A/B->audio.play`).
   The 9 post-TX-bookkeeping tails DO NOT read it (§10.2).
3. **Valid states / pre-write defaults:** `playback_buffer` empty before any `tx_transfer`;
   wire rings `clear_buffer`'d at install; RX ring memset-0 before any producer.
4. **Invariant consumers assume (INV-D):** each TX site runs SYNCHRONOUS post-drain
   bookkeeping; INV-D = "no site reads `size_buffer(playback_buffer)==0` as a precondition."
5. **What the fix changes:** under the outer stepper, `drain_playback_wait` returns
   immediately (queue-and-return); the outer loop is the sole drainer. So the bookkeeping
   fires while TX symbols are STILL QUEUED. Verified safe per §10.2.

### §10.2 The 9-site post-TX-bookkeeping audit (INV-D) — RESULT: ALL CLEAN
Every `drain_playback_wait()` site walked + its synchronous post-drain code:

| Site | Function | Post-drain bookkeeping | Reads play-empty? |
|------|----------|------------------------|-------------------|
| `:5003` | `send()` single ctrl/data | `last_message_sent_type/code`, `last_received_message_sequence=-1` (STATE-only) | NO |
| `:5076` | `send_batch()` big-block | capture-ring reset (RX ring only) + `rx_mute=0` + `messages_tx[].PENDING_ACK` + `frames_to_read` arm | NO |
| `:5466` | `send_batch()` per-frame | capture-ring reset (RX ring only) + `rx_mute=0` + `messages_tx[].PENDING_ACK` + frees batch slots | NO |
| `:5704` | `send_ack_pattern()` | `rx_mute=1`→reset(RX only)→`rx_mute=0`→`frames_to_read` arm | NO |
| `:5838` | `send_ack_pattern_with_snr()` | same RX-only flush | NO |
| `:6304` | `send_mfsk_ack_sack()` | same RX-only flush | NO |
| `:6511` | break pattern | same RX-only flush | NO |
| `:6693` | HAIL/beacon pattern | same RX-only flush + `ftr=2` | NO |
| `:7038` | HAIL pattern | same RX-only flush + `ftr=2` | NO |

**CONCLUSION:** Every site's post-drain code touches ONLY the RX ring
(`passband_delayed_data` / `capture_buffer` / `ring_write_index` / `frames_to_read`) and
STATE fields (`rx_mute`, `messages_tx[].status`, `last_message_sent_*`). NONE reads
`playback_buffer` occupancy. The queued-not-yet-drained TX symbols live in `audio.play`/wire,
which no post-drain tail touches; `rx_mute` already gates self-echo. The
`messages_tx[].PENDING_ACK` / `RECEIVING_ACKS_DATA` / `receiving_timer` transition is
STATE-only and production-faithful to set immediately (production sets it the instant the
last sample is handed to the DAC, not when it physically egresses). **INV-D HOLDS for all 9
sites; no site needs an ordering fix.**

### §10.3 The fix (Phase b)
`drain_playback_wait()` (`arq_common.cc`): add `if (g_sim2_outer_stepper_active) return;`
at the TOP (after the `sim_clock_on` read), BEFORE the spin loop. Gated on a NEW SIM-internal
flag `g_sim2_outer_stepper_active` (set true by `test_sim_inproc_2` ONLY while the outer
stepper loop runs, cleared on exit) — NOT on `arq_sim_inproc_active()` alone, because under
`legacy` (still runnable until Phase d) the pump-driven blocking drain must stay. Production
(`pump==null`, flag false) keeps the verbatim unbounded body — G9 byte-identical.

Because the drain now returns immediately under `outer`, `process_main()` no longer blocks;
the Phase-a outer loop already DRAINS the queued TX one symbol/iter (the top-level catch-up
drain/feed) and DELIVERS it to the peer with per-frame decode pacing. The nested-drain
reentrancy (the §9.3 (i)/(ii)/(iii) coupling) never occurs — `process_main` returns after
queueing, and there is no depth>0 TX-drain. The NO-PROGRESS `[SIM2-DEADLOCK-BREAK]` floor can
therefore never fire under `outer` (its non-firing is a positive test signal — §6.1 G1).

### §10.4 AS-BUILT (the naive §10.3 plan needed THREE refinements — discovered in implementation)
The pure "no-op drain for ALL TX sites under the outer flag" of §10.3 BROKE the MFSK handshake
and stranded the OFDM block. Three discriminations were required (all gated on the outer flag;
production untouched):

1. **MODULATION + DATA-BATCH gate, not a blanket no-op.** The MFSK CONNECT/HAIL + control-ACK
   handshake REQUIRES intra-`process_main` co-routine pump delivery: the peer's reply must land
   in the RX ring BEFORE the same call's `receive_*` poll (the outer loop cannot inject delivery
   mid-`process_main`, and `send_*_pattern`'s post-TX RX-ring flush would wipe an out-of-band
   reply). The outer loop alone broke CONNECT (G2: `connected=0`, hit iter cap). Fix: a per-modu-
   lation + per-frame-type seam. `sim2_activate(m)` tags `g_sim2_active_is_ofdm =
   is_ofdm_config(m->current_configuration)`; `send_batch()` brackets its body with an RAII
   `g_sim2_in_data_batch_tx` flag. The drain seam `sim_outer_stepper_paces_active() =
   outer && active_OFDM && in_data_batch_tx` fires ONLY for the OFDM big-block DATA batch — the
   MFSK handshake / ACK patterns / control keep the legacy co-routine pump (burst-safe for MFSK,
   G2 proves it). The OFDM conjunct keeps the 19-B legacy arm BYTE-IDENTICAL (dropping it routes
   the small MFSK data batch through egress and shifts G2 iters 15→16 — a determinism regression).
2. **EGRESS-to-WIRE, not no-op-leave-in-play.** A block left in `audio.play` is DISCARDED by the
   SAME `process_main`'s post-TX ACK path (`clear_buffer(playback_buffer)`, `arq_commander.cc`
   2915/3140/3240) the instant the CMD enters the ACK wait — the RX never gets it. So the OFDM
   data drain spins a NEW **drain-ONLY pump** (`sim_inproc_drain_to_wire_only`, installed via
   `arq_set_sim_inproc_drain_only_pump`) that moves `tx.play → wire` (NO RX deliver, NO decode
   burst, NO peer-recurse) until play empties. The wire is `AUDIO_PAYLOAD_BUFFER_SIZE`=12.288 MB
   (~19 blocks) so a single in-flight block never saturates → the spin always terminates (no
   deadlock floor needed). The OUTER loop then FEEDS `wire → RX` ONE SYMBOL PER ITER (the §8/INV-A
   decode cadence). Outer-loop drain/feed GRANULARITY is OFDM-gated: OFDM → one symbol/iter (so
   the CMD's ACK poll interleaves + decode never over-advances a frame boundary); MFSK → whole-
   buffer catch-up (the pump already moved it → no-op). The end-of-`process_main` §5.7 pacing-floor
   pump is suppressed in the OFDM PHASE (`sim_outer_stepper_ofdm_phase() = outer && active_OFDM`)
   so it never bursts the queued block after `send_batch` returns.
3. **`ptt_busy_wait`/`pumped_settle_wait` disposition.** `ptt_busy_wait` is CLOCK-ONLY on the OFDM
   data path (`sim_outer_stepper_paces_active()` — advances the virtual clock by `delay_ms`, no
   pump) so the PTT turnaround does not burst the queued block; on the handshake/ACK path it keeps
   the verbatim pump spin. `pumped_settle_wait` KEEPS the pump under the outer stepper (its callers
   are the handshake/control-ACK settle guards that need intra-call delivery; the OFDM data batch
   is never in play during a `pumped_settle_wait` call). The control-ACK arrival rescan
   (`arq_common.cc` ~7437) keeps running (pump-driven) for the SET_CONFIG turnaround.

RESULT (verified, `--test-sim-sustain`, ROBUST_0→CFG16 live SET_CONFIG, 1374-B K=8 block):
- BEFORE (legacy pump): `deadlock_breaks=2`, the OFDM data path WEDGES (multi-MB TX tail abandoned).
- AFTER (outer): `deadlock_breaks=0`, the CFG16 K=8 block carves **CLEAN 8/8**, bytes reach the RX
  FIFO (`rx_have=1374/1374`). The (iii) DATA-path nested-drain wedge (§9.3) is GONE.
- G2 BYTE-IDENTICAL (iters=15 sim_ms=61535 pump calls=507…), G3/G4/G5/G6/G7 ALL PASS, G8 same-seed
  byte-identical, G9 production diff confined to the gated SIM seams + the SIM_INPROC TU + the new
  `--test-sim-sustain`.

### §10.5 OPEN — Phase c: the multi-batch ACK-turnaround TIMING (the remaining sustain blocker)
Full byte-correct SUSTAIN across MANY batches is NOT yet achieved and is the documented Phase-c
item. ROOT CAUSE (traced, not inferred): under the per-symbol feed, the OFDM big-block ACK
round-trip overruns the CMD's `receiving_timeout`. The block egresses `play → wire` inside
`send_batch` (clock +~1.65 s), then the RSP CONSUMES it over the following feed-iters; the virtual
clock advances on the TX-egress AND (via idle-fills) during the RX-consume, so the data airtime is
effectively counted ~2× and the ACK arrives at `rx_t≈3700 ms` vs `receiving_timeout=2686 ms`
(`polls=0`/late → `nAcked_data=0` → the CMD re-sends `bsi=0` → duplicate delivery → `bytes_ok=0`;
on free-flow the resulting "batch failure" makes the climb COLLAPSE off OFDM and the BREAK/control
turnaround — still legacy-pump — re-trips the floor). The clean fix is the drain/feed/clock-
accounting RECONCILIATION the design assigned to Phase c (couple TX-drain-time and RX-consume-time
to ONE symbol of channel time per iter; route the post-CONNECT control/BREAK turnaround through the
outer loop). Per CLAUDE.md §1.2, after >3 turnaround-timing attempts this is STOPPED here as a
distinct Phase-c workstream — Phase b ships the proven DATA-path wedge removal + clean OFDM carve.

### §10.6 C0 — the ACK-turnaround timing fix (clock-attribution instrumented, 2026-06-09)
PRE-CODE measurement (one-change-one-test, CLAUDE.md §4): a clock-attribution counter
(`sim2_drain_to_wire` bulk vs `sim_step_drain_one_symbol` per-symbol) on the `--test-sim-sustain`
outer arm (`MERCURY_SIM2_DBG=1`) pinpointed the double-count EXACTLY. Per-outer-iter clock delta
(`[C0-ITER]`), batch-1 OFDM block (K=8, 79360 samples = 1653 ms airtime, `receiving_timeout=2686`):
- `it=11` (`receiving_timer.start` @ clk=1395760, the post-TX timer is AFTER this point): the EGRESS
  fires `sim2_drain_to_wire` ONCE for the whole block — `bulk=79360(n=1)`. This is the legit TX
  airtime and it is PRE-TIMER, so it does NOT count against `receiving_timeout`. (Not the bug.)
- `it=12`: `bulk=174840(n=141)` — **141 `sim2_drain_to_wire` calls add 174840 samples (3643 ms) in
  ONE outer iter**, inside B/RSP's decode-drive `process_main` ACK turnaround (the nested legacy
  pump `sim_inproc_pump_2` idle-fills/drains the reverse wire while B waits). The CMD's
  `receiving_timer` (started @ it=11) accrues this whole 3643 ms → `elapsed=3694ms > 2686ms`,
  `polls=0` → ACK-timeout → BREAK → re-send `bsi=0` (`fallback=1`) → duplicate → `bytes_ok=0`.
- The outer loop's OWN per-symbol pacing is correctly bounded: `step=2480(n=2)` per iter (2 symbols).

ROOT CAUSE (single sentence): in the OFDM data phase, the shared virtual clock is advanced INDE-
PENDENTLY by every per-instance/per-direction `sim2_drain_to_wire` (egress whole-block + the nested
ACK-turnaround pump's 141 idle-fills), so one outer iteration can jump the clock by HUNDREDS of
symbol-times — but production's two concurrent audio threads advance ONE shared wall-clock by ONE
symbol per quantum. The clock is double/over-counted because TX-drain and RX-consume each bump it.

FIX (couple to ONE symbol of channel time per outer iter — the design's §6/§9 keystone): in the
OFDM data phase (`sim_outer_stepper_ofdm_phase()`), the per-instance/per-direction drains
(`sim2_drain_to_wire`, `sim_step_drain_one_symbol`) MOVE BYTES but do NOT each bump the shared clock;
the OUTER LOOP advances the shared clock by EXACTLY ONE symbol ONCE per iteration (both directions
advance one symbol of wire-time together, sharing one symbol of wall-clock). Consequence: the
64-symbol block consume costs 64 symbol-times (its airtime, ONCE) paced one-symbol-per-iter by the
outer loop; the existing RECEIVING_ACKS_DATA restart guard (arq_commander.cc, OFDM phase) pins the
CMD's `receiving_timer` at ~0 while `wire_a2b>0`; after the block fully consumes, the receiving
window measures only the genuine ACK round-trip (~ack_pattern 413 ms + ptt/decode margins) which
fits inside `receiving_timeout=2686ms` → ACK detected → `nAcked_data>0` → no re-send → no duplicate
→ `bytes_ok=1`. This is NOT widening `receiving_timeout` (forbidden threshold-masking); it removes
the SPURIOUS clock time the sim injected that production never had.

### §10.7 C0 — CORRECTION: the timing fix is NECESSARY but NOT SUFFICIENT — a DEEPER blocker (the RSP never TRANSMITS the clean big-block data-ACK). STOP per CLAUDE.md §1.2 (2026-06-09)
The §10.6 timing model was VERIFIED correct but implementing it surfaced a SECOND, dominant blocker
that the §10.6 timing fix CANNOT resolve. Two fix attempts on the timing (one-change-one-test, both
reverted, full revalidation owed):
- ATTEMPT-1 (whole-iter `g_sim2_outer_owns_clock` clock suppression + one tick/iter): HUNG. The
  clock-driven waits (`ptt_busy_wait`/`pumped_settle_wait`) inside B's nested ACK turnaround exit on
  `get_elapsed_time_ms() >= delay_ms`; suppressing the clock for the whole OFDM-phase iter froze them
  → infinite spin inside one `process_main`. (Clock authority cannot be blanket-removed during the
  nested waits that depend on it.) Reverted.
- ATTEMPT-2 (gate OFF the `receive_ack_pattern` intra-call re-scan spin under outer+OFDM, so the
  re-scan's ~94×`pumped_settle_wait(52)`=~4.9 s window no longer inflates the CMD timer): the timing
  HALF FIXED — `polls=0 → polls=34`, the receiving window no longer overruns before the first poll.
  But `bytes_ok` STAYED 0: the CMD polled 34× and detected NOTHING (`peak_matched=0/7
  peak_metric=0.0`). Reverted.

The DEEPER ROOT CAUSE (measured, pristine 5e7cc15 binary, `MERCURY_SIM2_DBG=1`):
**the RSP carves the clean K=8 block but NEVER TRANSMITS a data-batch ACK on CFG16.** Evidence,
all from the SAME run:
- `[BIGBLOCK-ARQ] CLEAN block bsi=0 K=8 -> 1 ACK (all-ones) ... delivered_fifo=1` — the carve
  (`bigblock_block_to_arq` / `bigblock_arq_decide_and_ack`, test_bigblock_arq_unit.cc:199-264) DELIVERS
  to the FIFO + bumps bsi, but is DELIVERY-ONLY: it does NOT call `send_ack_pattern` /
  `send_mfsk_ack_sack`. "1 ACK (all-ones)" is a LOG of the ARQ DECISION, not a wire transmission.
- The generic clean-batch ACK transmitter is the ACK-GATE (`[RSP-RX-TIMEOUT] Entering ACK-GATE ...`
  → `send_batch`/`send_ack_pattern`, arq_responder.cc:1490/1983). Across the whole run that print
  fires **0 times** for the big-block; `stats.nAcks_sent_data = 0` throughout; the ONLY ACK on the
  wire is the CONNECT handshake (`[TX-ACK-PAT] Sending ACK pattern on CONFIG_100`). There is NO
  `[TX-ACK-PAT] ... CONFIG_16` and NO `[RSP-MFSK-SACK] clean path`.
- So no data ACK is EVER on `wire_b2a`. The CMD's `[CMD-ACK-PAT] Timeout peak_metric=0.0` is therefore
  CORRECT — there is nothing to detect. The CMD BREAKs and re-sends `bsi=0`; the RSP carves it AGAIN
  (`fallback=1`); the duplicate fills the FIFO → `rx_have=1374/1374 bytes_ok=0`. The timing overrun
  (§10.6) and the missing transmission are BOTH present, but the missing transmission is the
  bytes_ok blocker: even with perfect timing there is no ACK to receive.

WHY this was invisible until now: the single-arming `--test-bigblock-fullpath` (G5, bytes_ok=1) and
`--test-bigblock-multicw` (G6) drive ONE instance's TX→PHY→RX carve and assert on the delivered FIFO /
`messages_rx[]` DIRECTLY — they never run the 2-instance RSP→CMD data-ACK ROUND TRIP. The 2-instance
sustain (G1) is the FIRST test that requires the RSP to actually transmit the clean big-block data-ACK
back to the CMD — and that transmission path is not wired for the big-block carve under the 2-inst
stepper (the carve, fired from B's decode-drive at depth 1, delivers but does not route to the
ACK-GATE; B's top-level RECEIVING→ACK-GATE transition would see `rx_received=0` anyway because the
carve's `copy_data_to_buffer()` already marked the slots ACKED and freed them).

STATUS: STOPPED per CLAUDE.md §1.2 (two timing attempts failed to flip bytes_ok; the third diagnosis
shows the scoped item — a timing coupling — is NOT the root cause) and §2 (a timing-only fix would
mask the missing-ACK; forbidden). The C0 scope as written ("couple to one symbol of channel time so
the batch-ACK completes inside receiving_timeout") presupposes the ACK is transmitted; it is not.
The TRUE Phase-c fix is TWO coupled changes, both in the RSP/ARQ layer + the stepper clock:
  (C0-a) WIRE THE CLEAN BIG-BLOCK DATA-ACK TRANSMISSION: after `bigblock_block_to_arq` carves a CLEAN
         block (n_clean==K) on the live 2-inst path, the RSP must TRANSMIT the clean-batch ACK
         (`send_mfsk_ack_sack` all-ones / `send_ack_pattern`) on CFG16 — the carve currently only
         delivers+bumps. Audit `copy_data_to_buffer` freeing the slots before the ACK-GATE counts
         them (the ACK-GATE's `rx_received` would be 0). This is the ACTUAL bytes_ok blocker.
  (C0-b) THEN the §10.6 clock-coupling so the now-transmitted ACK round-trip fits receiving_timeout
         WITHOUT the re-scan's full-window spin inflating the CMD timer. (ATTEMPT-2's re-scan gate is
         the right shape for the timing half once an ACK actually exists to deliver.)
This is a cross-layer ARQ change (CLAUDE.md §5), larger than the one-line timing coupling C0 was
scoped as. NOT shipped; the worktree source is left at pristine 5e7cc15 (only this fact-doc updated).
The two reverted attempts + this corrected diagnosis are recorded so the next session does not
re-chase the timing-only framing.

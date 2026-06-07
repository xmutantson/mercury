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
- ~~**THROUGHPUT vs VARA (clean channel, deterministic on-air airtimes)**: Mercury
  production PPMd8+zstd streaming compresses Project Gutenberg #84 (pg84,
  448,885 B) → 121,496 B (**3.695×**). Block carries 1374 compressed app bytes in
  1.6533 s airtime (6649 bps compressed wire). effective = orig·8 / (88.4 blocks ·
  cycle): airtime-only ceiling **24,564 bps**; nominal 913 ms turnaround
  **15,825 bps**; conservative 1014 ms **15,226 bps**. **WIN across the whole band
  vs VARA HF Standard 13,048 bps.** Clean-channel number (no SFO/CFO, matching
  VARA's bar); HW confirmation pending (testbed emulator low-pass collapse).
  Numbers + harness: `bigblock_p3_hw/results_simproof.json`,
  `bigblock_p3_hw/measure_compress.cc`.~~

- **CORRECTION (monitor-hygiene, 2026-06-07 — the VARA "WIN" above is FALSE; the
  big-block does NOT genuinely decode end-to-end):** the 15,226-24,564 bps "WIN
  across the whole band vs VARA" figures were derived from the in-process SIM
  gates (`--test-bigblock-fullpath`/`-multicw`/`-livepath`), which all take an
  **ORACLE decode path** (`bigblock_last_tx_K>0` ⇒ the reference block is handed
  to the decoder, `ref!=NULL` @ `telecom_system.cc:8336`), BYPASSING genuine
  LDPC acquisition+decode. The throughput projection therefore assumed a
  byte-faithful block that the live 2-instance receiver (`ref==NULL`) does NOT
  produce.
  - **HW bench, FIRST UNCONFOUNDED healthy-bench run** (Approach-A flat-gain,
    `A-flatgain`@`84f7add`; `bigblock_p3_hw/results_bb_hw_A.json`): the link was
    healthy (CFG15 5000 B on-Pi byte-faithful @10891.6 bps, `ofdm_ok=33`,
    `meanH=0.979`), the climb held CFG16 and EMITTED big-blocks (`bbtx=3`,
    `bsi=5`, `K=8`), yet the RSP rejected **EVERY** block at the cw0 wire-CRC
    gate → `carve_k8=0`, **0/1374 app bytes delivered**, deterministic 2/2.
    `pg84_win_loss: NA (decode not 8/8 → pg84 skipped)`. This is NOT the prior
    "emulator low-pass collapse" attribution — the WB path was alive
    (`meanH=0.979`, `ofdm_ok=17` during the big-block emit); the genuine
    (`ref==NULL`) RX produces wrong cw0 bits and the gate correctly rejects.
  - **The defect is a localized big-block channel-estimation / decode defect**,
    masked in sim by the oracle (`ref!=NULL`) bypass. (The earlier in-sim
    `meanH≈0`-over-the-133-sym-block framing was a sim-harness symptom of the
    same not-genuinely-decoding class.) The pinned `1200/1200` / `1374/1374`
    "byte-faithful" sim results in this section are ORACLE-confounded and do NOT
    establish live decode.
  - **What actually holds:** Approach-A's flat-gain **TX-LEVEL** discipline
    works on HW (`g≈0.733`, −2.69/−2.71 dB, `raw_rms 0.183→fir_rms 0.134` at the
    modulator — the level math is confirmed). **The per-frame (stock CFG16) path
    is at ~PARITY** with VARA, not a win: ~13,350 vs VARA ~13,048 bps.
  - **The VARA win is NOT achieved.** Big-block end-to-end delivery is BLOCKED
    pending a fix to the big-block channel-estimation/decode defect on the live
    (`ref==NULL`) path. The big-block framing rung remains **default-OFF**
    (`bigblock_framing_enabled=false`, `telecom_system.h:532`; only set by env
    `MERCURY_BIGBLOCK_FRAMING` or the `--test-bigblock-*` harnesses), so CFG16
    in production runs the byte-identical stock per-frame path and does NOT elect
    the non-decoding block rung (verified `arq_common.cc:1122-1124`).

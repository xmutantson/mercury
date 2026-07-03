# data-flow-stream-offset.md — the absolute-byte-stream cursors (Option W)

**Owner doc for the NEW shared state introduced by Option W:**
`tx_stream_committed` (sender cursor), `rx_stream_delivered` (receiver cursor),
`tx_stream_stamp[256]` (per-bsi latched {start,length}).

**Why this exists (root):** `silent-corruption-residual.md` §11–§14 proves FOUR
distinct silent byte-corruption mechanisms at marginal SNR (WGN:25), each a case
where Mercury's BATCH-RELATIVE `(batch_seq_id, frame_id)` sequencing stays
LOCALLY consistent while the ABSOLUTE app-byte-stream position DRIFTS (a dropped
tail frame at a shrunk expected-count, a re-packed/orphaned batch, an
out-of-order prev delivery, a forward Δ=+6847 hole). Every existing guard checks
a PROXY invariant (re-queue completeness / batch-SIZE equality / bsi-contiguity /
EOB frame-count) — NONE checks the invariant that actually matters:

> **THE INVARIANT (§14.4):** the delivered transported-byte stream at the
> RECEIVER == the committed transported-byte stream at the SENDER, by ABSOLUTE
> byte offset, in order.

Option W (Fable-5 steer `wuj3xqvod`, worklog 2026-07-03) enforces this TCP-style
end-to-end invariant with per-batch byte-sequence stamps + a delivery-boundary
check. This doc is the CLAUDE.md-mandated cross-layer producer/consumer audit of
the new cursor state, written BEFORE the code.

Branch: `fix/stream-offset-w` off `monitor@bfacbcf6`. MERGE = owner fork (wire
change). Provenance of all `file:line` below: the worktree at
`.../scratchpad/stream-offset-wt` (== bfacbcf6).

---

## §0 Offset DOMAIN — TRANSPORTED (post-compression) bytes, NOT raw app bytes

Fable's CORRECTED design counts **TRANSPORTED bytes** = the exact payload bytes
that get split into DATA frames and re-assembled on the wire (post-compression,
post-encrypt, incl. compression header + crypto padding + auth tag). This makes
compression / padding / EOB **non-issues**: both peers count the SAME on-wire
bytes per batch, independent of the codec.

- With **compression OFF** (all 4 captured mechanisms, §11–§14), transported ==
  raw app bytes, so the cursor offset == the app-source offset the harness
  measures. This is the exact domain the 4 mechanisms corrupt.
- With **compression ON**, the cursor verifies the receiver reassembled EXACTLY
  the transported bytes the sender framed, contiguous + in order. Given a synced
  streaming decompressor (an EXISTING invariant, not W's job), a faithful
  transported stream ⇒ a faithful app stream. W verifies POSITION/CONTIGUITY;
  the ADD-ON running CRC-32 (STEP 3) verifies transmission integrity/content.
  W CANNOT by itself catch a streaming-decompressor desync that keeps the
  transported stream contiguous — that is a distinct (uncaptured) class; the
  CRC-32 co-stamp closes it.

**Sender transported length per batch** (`process_buffer_data_commander`,
arq_commander.cc:20093):
- compressed leg: `comp_size` AFTER compress → (optional) crypto-pad to
  `target_comp` (:20421) → encrypt (+tag) (:20495 `comp_size = enc_size`). The
  frame-split loop (:20534) frames exactly `comp_size` bytes.
- raw leg (:20561): Σ `data_read_size` per popped frame (headerless), == the
  member `batch_uncompressed_size` in this leg (raw ⇒ transported==app).

**Receiver transported length per batch** (`copy_data_to_buffer`,
arq_common.cc:14020): Σ `messages_rx[i].length` for `i < data_batch_size` where
`status==ACKED` — compressed leg computes this as `assembled_size` (:14040), raw
leg pushes each `messages_rx[i].length` (:14228). Identical quantity.

---

## §1 The new state (declared in include/datalink_layer/arq.h)

| member | side | meaning |
|--------|------|---------|
| `uint64_t tx_stream_committed` | CMD (sender) | cumulative transported bytes committed to built batches, in build order. The offset at which the NEXT new batch starts. |
| `StreamStamp tx_stream_stamp[256]` | CMD | per-bsi latched `{uint64 start; uint32 length; bool valid}`, indexed by `batch_seq_id & 0xFF`. Latched at BUILD, re-emitted verbatim on retx/mixbatch, start restored on re-stage. |
| `uint64_t rx_stream_delivered` | RSP (receiver) | cumulative transported bytes pushed through `copy_data_to_buffer` (delivered), in delivery order. |

`StreamStamp` is a POD added to arq.h. Both cursors are `uint64_t` (offset domain
> 4 GB safe; the wire field is a wrapping 32-bit low word, §CORE).

---

## §2 PRODUCERS (every site that WRITES the new state) — file:line

### 2.1 `tx_stream_committed` + `tx_stream_stamp[]`
- **THE ONE LATCH SITE (write/advance):** `process_buffer_data_commander`
  (arq_commander.cc:20093), at the END of the data-staging `if`-block (after both
  the compressed leg :20157 and the raw leg :20561, before the `if` closes
  :20633). When `batch_committed_len > 0` (new transported bytes were framed this
  call): `stream_tx_latch(cmd_batch_seq_id & 0xFF, batch_committed_len)`.
  - `cmd_batch_seq_id` here == the bsi the new frames get in
    `process_messages_tx_data` (:2343 `messages_tx[i].batch_seq_id =
    cmd_batch_seq_id & 0xFF`); it is not mutated between staging and send (the
    +1 advance is post-send, :2591). So the latched bsi == the wire bsi.
  - `stream_tx_latch(bsi,len)`: `stamp[bsi]={tx_stream_committed,len,valid=true}`;
    `tx_stream_committed += len`. Idempotent per build: the re-entry guards
    (`live_newdata_staged` :20251 compressed; `fill_limit -= already_staged`
    :20605 raw; `stage_ok` :20139) guarantee new bytes are popped ONCE per bsi,
    so `batch_committed_len>0` fires once per (bsi, build-generation). On a REBUILD
    after a re-stage rollback the cursor was reset to `stamp[bsi].start`, so the
    re-latch recomputes the SAME start (overwrite is idempotent on start; the
    length may differ if rebuilt at a new config — that is the correct new stamp).

### 2.2 `tx_stream_committed` ROLLBACK (the re-stage un-commit) — TWO FUNNELS
Every one of the 10 re-stage caller sites (below) frees the in-flight
`messages_tx[]` batch and re-queues its bytes to `fifo_buffer_tx` through EXACTLY
one of two byte-restore funnels. The cursor rollback lives at the TOP of each
funnel (before it frees `messages_tx`), computing `min_inflight_bsi` from the
non-FREE `messages_tx[]` frames (the SAME scan the callers use, e.g.
arq_commander.cc:3490-3503) and calling `stream_tx_rollback_inflight()`:
`tx_stream_committed = tx_stream_stamp[min_inflight_bsi].start` (guarded on
`min_inflight_bsi>=0 && stamp.valid`; idempotent — see §4-INV3).

- **Funnel 1 — `restage_requeue_tx_messages()` (arq_common.cc:13976):** the
  non-compressed re-stage (§13). Rollback added at its top.
- **Funnel 2 — `restore_tx_from_compressed()` (arq_common.cc:14252):** the
  compressed/streaming/encrypted twin (calls `restore_backup_buffer_data()`
  :14360, or the reassemble-decompress branch :14308). Rollback added at its top,
  BEFORE any `messages_tx[i].status=FREE` (:14277/14302/…).

**The 10 caller re-stage sites** (all route through Funnel 1 or 2):
`arq_commander.cc` 707, 795 (BREAK ACK-recovery), 2199, 2225 (D3 demote),
3510/3514, 5038/5042, 5245, 5366/5370, 5517/5521, 6229. Each is a
compression-gated `if(compression_enabled) restore_tx_from_compressed(); else
restage_requeue_tx_messages();` pair (e.g. :3508-3517). Because the rollback is
in the funnels, it fires for ALL 10 regardless of whether the caller also rolls
`cmd_batch_seq_id` (only 4 callers do: `cmd_batch_seq_id = min_inflight_bsi` at
3529, 5536, 5741, 6311) — decoupling the cursor from the caller-side bsi bookkeeping
removes the site-enumeration fragility that Fable flagged as MOST-LIKELY-TO-BITE.

- **The 3 BREAK `push_front` sites (arq_commander.cc:581, 669 …)** that §13 left
  as already-correct do NOT call the funnels; they re-queue directly. Audit: these
  are inside the BREAK handler whose recovery ALSO routes the compressed leg
  through `restore_tx_from_compressed` (707/795). VERIFY at implementation whether
  the raw-leg direct push_front at 581/669 needs its own `stream_tx_rollback_inflight()`
  call (it re-queues in-flight bytes → the cursor must roll back there too, else a
  BREAK on a NON-compressed session leaves the cursor high). See §7 open item O1.

### 2.3 `rx_stream_delivered`
- **THE ONE ADVANCE SITE:** `copy_data_to_buffer` (arq_common.cc:14020), at
  `copy_data_done` (:14244): `rx_stream_delivered += delivered_transported`, where
  `delivered_transported` = Σ delivered `messages_rx[i].length` accumulated in
  both the compressed (:14050) and raw (:14228) legs. This is the SINGLE receiver
  funnel — all delivery paths call it: in-order (arq_common.cc:13930/13938),
  cross-storage PREV (arq_responder.cc:1318), the §5098 orphan path
  (arq_common.cc:5153), and bigblock (arq_commander.cc:17558). No delivery
  bypasses it → the cursor + STEP-2 backstop cover every path.

### 2.4 Init / reset
- `reset_session_state()` (arq_common.cc:7050): `tx_stream_committed=0;
  rx_stream_delivered=0; for all i: tx_stream_stamp[i].valid=false`. A fresh
  session re-anchors both cursors at 0.
- Constructor / `init()` in-class default: same (mirrors the `cmd_batch_seq_id=0`
  ctor init, arq_common.cc:700).

---

## §3 CONSUMERS (every site that READS the new state) — file:line

### FOUNDATION (this commit) — cursors only, NO wire, NO gate yet
- The regression test (`test_stream_offset.cc`, `--test-stream-offset`) reads the
  cursors + stamps to assert the invariant at every transition. No production
  consumer branches on the cursors yet.

### CORE W (STEP 2) — adds the wire stamp + the two RSP checks
- **Sender emit:** the D5 header build (arq_common.cc `send_batch`/frame serialize,
  DATA_LONG 7-byte + DATA_SHORT 6-byte headers around :8709/8739) reads
  `tx_stream_stamp[bsi]` and appends `{start_lo32, length16}` to the batch's
  EOB-bearing frame. Retx/mixbatch re-emit the LATCHED stamp (mirror D5's
  `sack_retransmit_active` emit-0 discipline) — NEVER recompute.
- **Receiver parse + store:** frame parse stores the received stamp per-bsi
  (`rx_stream_stamp[256]`).
- **RSP PRIMARY check** (byte-GATE the clean ACK, arq_responder.cc near ACK-GATE
  PASS :2487 / BATCH-DONE :2596): before emitting a clean bitmap / BATCH-DONE,
  assert `delivered_transported_for_this_bsi == rx_stream_stamp[bsi].length`; on
  shortfall WITHHOLD the clean ACK (let existing SACK partial-retx re-request the
  missing frames) — do NOT credit, do NOT flush backup, do NOT build a new
  re-request path. Makes mechanism-4 "declare complete at shrunk count" impossible.
- **RSP BACKSTOP check** (`copy_data_to_buffer` :14020, before appending): assert
  `rx_stream_stamp[bsi].start == rx_stream_delivered`; on mismatch raise LOUD
  `[RSP-V2-STREAM-SHIFT]` + `rsp_gap_abort_teardown` (arq_common.cc:10051).

---

## §4 VALID STATES + INVARIANTS

**Valid states of `tx_stream_committed`:** monotone-nondecreasing EXCEPT a
re-stage rollback to a prior `stamp.start`. Always == Σ transported lengths of
all currently-live (built, not-rolled-back) batches in build order.

**Valid states of a stamp[bsi]:** `valid=false` (never built / post-session-reset)
→ `valid=true {start,length}` (latched at build) → same or overwritten (rebuild
after rollback, same start; or wrap-around reuse 256 batches later, fresh start).

**Valid states of `rx_stream_delivered`:** monotone-nondecreasing, advances by a
batch's delivered transported length exactly ONCE per successful delivery. Never
rolls back (a failed batch is not delivered → no advance; re-delivery advances
once). A double-delivery would advance twice → the STEP-2 backstop catches it.

**INV1 (the enforced invariant):** at the start of batch B's delivery,
`rx_stream_delivered == tx_stream_stamp[B].start` (the sender's committed offset
at B's build). Enforced by the STEP-2 backstop; the cursor makes it checkable.

**INV2 (latch idempotency):** new bytes are popped ONCE per (bsi, build-gen), so
`stream_tx_latch` commits each batch's transported length exactly once. Guarded by
`stage_ok` / `live_newdata_staged` / `already_staged`.

**INV3 (rollback is LIFO + idempotent):** the only batch ever re-staged is the
single in-flight batch (Mercury builds one batch at a time,
`messages_tx[]` holds one batch; "one batch is in flight on the per-frame path",
arq_commander.cc:5488). So `min_inflight_bsi` is the most-recently-latched live
bsi and `tx_stream_committed = stamp[min_inflight_bsi].start` is exactly the
LIFO un-commit. Calling the rollback from BOTH a funnel and (redundantly) a caller
is safe: it resets to the same `stamp.start` each time (idempotent), and nothing
re-latches between. If >1 batch were ever in flight and all failed, rolling to the
EARLIEST (`min_inflight_bsi`) start + rebuilding re-accumulates correctly (each
rebuilt batch overwrites its stamp with the fresh cumulative start).

**INV4 (rebuild start-stability — the correctness crux):** a re-staged batch,
rebuilt (possibly at a NEW config → NEW transported length), MUST carry the SAME
`stamp.start` as its original build, because the RECEIVER anchored delivery of
that bsi at that start (`rx_stream_delivered` did NOT advance for the failed
batch — it was never delivered). Guaranteed by: rollback resets
`tx_stream_committed` to `stamp.start`, and the rebuild's latch recomputes
`start = tx_stream_committed = stamp.start`.

**INV5 (transported == transported):** the sender's `batch_committed_len` (bytes
framed) == the receiver's delivered `Σ messages_rx[i].length` (bytes reassembled)
for a fully-delivered batch. Holds because the frame-split (:20534) and the
reassembly (:14042) move the identical byte span, incl. crypto pad + tag (§0).

---

## §5 WHAT THE FIX CHANGES (per-consumer walk)

FOUNDATION changes NO consumer behavior (cursors are write-only bookkeeping +
a test reader). The only risk it introduces is a MIS-MAINTAINED cursor (a latch
that double-commits, or a re-stage that fails to roll back) — which the
deterministic test (§6) exists to catch as Fable's pre-cohort ground-truth gate.

CORE W (STEP 2) changes two RSP consumers:
- ACK-GATE / BATCH-DONE: previously credited a batch on
  `received==expected(count)`; now ALSO requires `delivered_bytes==stamp.length`.
  FALSE-FIRE guard: the stamp carries LATCHED committed bytes ONLY, never
  `data_batch_size` / `rx_batch_total_frames` / expected-counts — so a legitimate
  batch-size resize or count-shrink does NOT trip it (those change counts, not the
  committed byte length of the frames actually sent).
- copy_data_to_buffer: previously appended unconditionally; now asserts
  `stamp.start==rx_stream_delivered` first. FALSE-FIRE guard: PREV/mixbatch
  deliveries are contiguous by construction (each carries its own latched start);
  the check only fires on a genuine hole/shift/dup.

---

## §6 REGRESSION TEST (`test_stream_offset.cc`, `--test-stream-offset`)

Deterministic, in-process, no RF. Drives the PRODUCTION cursor helpers +
the PRODUCTION re-stage funnel (`restage_requeue_tx_messages`, the §13 helper the
task named — with synthetic `messages_tx[]`, mirroring `test_restage_requeue.cc`)
through EVERY transition, asserting `stamp[bsi].start == the true cumulative
transported origin` at each:
1. **Sequential builds** (compression on/off lengths): latch bsi 0..N; assert
   `stamp[k].start == Σ len[0..k-1]` and `tx_stream_committed == Σ len[0..k]`.
2. **Retx / mixbatch re-emit:** read `stamp[k]` again; assert unchanged, cursor
   unchanged; latch bsi k+1; assert `start[k+1]==start[k]+len[k]`.
3. **Re-stage rollback (Funnel 1):** stage in-flight bsi K in `messages_tx[]`,
   call `restage_requeue_tx_messages()`; assert `tx_stream_committed==stamp[K].start`;
   rebuild K (new length); assert `start==old start` (INV4).
4. **BREAK→ROBUST→re-climb:** demote (rollback K) then rebuild K at a robust
   (tiny) length then re-climb (bsi K+1 at a large length); assert contiguity.
5. **rx advance + INV1:** advance `rx_stream_delivered` by each delivered length;
   assert it equals the corresponding `stamp.start` at each boundary.
Fail-before/pass-after for the 4 mechanisms is added in STEP 2 (env-defeat arms).

---

## §7 OPEN ITEMS
- **O1** [?] BREAK direct `push_front` at arq_commander.cc:581/669 (non-compressed
  BREAK leg) — confirm it re-queues in-flight bytes and add
  `stream_tx_rollback_inflight()` there too if the funnels don't already cover the
  BREAK path. (If BREAK always routes non-compressed through
  `restage_requeue_tx_messages`, it is covered.)
- **O2** [?] Role-switch (SWITCH_ROLE) mid-stream: cursors are role/session-scoped
  and reset on session reset; a mid-stream role flip on a bidirectional transfer
  is out of scope for the WGN:25 one-directional cohort. Documented, not handled.
- **O3** STEP 2 wire: extend the D5 DATA_LONG(7B)/DATA_SHORT(6B) headers with the
  {start_lo32,length16} stamp on the EOB frame; both ends rebuild (no version bits,
  `iris_no_version_bits_preship`).
- **O4** STEP 3 add-ons: running CRC-32 co-stamp (+4B/batch, content/CRC-escape);
  EOT exchange (total_committed_bytes + final CRC in disconnect, ~8B once) — the
  ONLY true end-to-end tail-drop check (Fix-A shape).

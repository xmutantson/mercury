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

**The caller re-stage sites.** The COMPRESSED leg of every recovery/demote site routes through
Funnel 2 (`restore_tx_from_compressed`), and the DEMOTE / CFG16-HOLD raw legs route through
Funnel 1 (`restage_requeue_tx_messages`, sites 2204/2230/3519/5047/5250/5375/5526) — both funnels
roll the cursor back at their top.

> **CORRECTION (2026-07-03, Opus §16 re-audit — visible per CLAUDE.md):** ~~all 10 caller
> re-stage sites route through Funnel 1 or 2; each is a compression-gated
> `if(compression_enabled) restore_tx_from_compressed(); else restage_requeue_tx_messages();`
> pair.~~ **FALSE for the three raw BREAK/climb legs.** The `else` (non-compressed) leg of the
> BREAK phase-1 (arq_commander.cc:707→:715 open-coded loop :719), BREAK-EXHAUSTED (:800→:807),
> and GEARSHIFT FRAME-UP (:6234→:6241) sites is an OPEN-CODED reverse-iter `push_front` loop,
> NOT a `restage_requeue_tx_messages()` call — so its cursor un-commit is NOT in a funnel and
> must be called inline. Only :715 originally did (`stream_tx_rollback_inflight()` at :715);
> **:807 and :6241 OMITTED it → the F1/F4.1 blocker (silent-corruption-residual.md §16/§17.4).**
> FIXED: `stream_tx_rollback_inflight()` added inline at :807 and :6241, byte-mirroring :715.
> The 4 raw DEMOTE/CFG16-HOLD legs DO route through Funnel 1 (correct). Only 4 callers roll
> `cmd_batch_seq_id` back (3529/5536/5741/6311) + the FRAME-UP `roll_back_cmd_bsi_to_inflight`
> (:6229); the restore-from-BACKUP legs (arq_common.cc:6106/6194/6288/7501/3305) do NOT and are a
> distinct FRESH-bsi class (gap-gated, not BACKSTOP-checked) — documented residual, §16.1.

- **The 3 raw BREAK/climb `push_front` legs (arq_commander.cc:715/:807/:6241)** re-queue in-flight
  bytes directly (not via a funnel) and EACH now calls `stream_tx_rollback_inflight()` inline
  BEFORE freeing `messages_tx[]` — RESOLVED (was O1). The compressed leg of each routes through
  `restore_tx_from_compressed` (707/800/6234), which rolls back at its top.

### 2.3 `rx_stream_delivered`
- **THE ONE ADVANCE SITE:** `copy_data_to_buffer` (arq_common.cc:14020), at
  `copy_data_done` (:14244): `rx_stream_delivered += delivered_transported`, where
  `delivered_transported` = Σ delivered `messages_rx[i].length` accumulated in
  both the compressed (:14050) and raw (:14228) legs. This is the SINGLE receiver
  funnel — all delivery paths call it: in-order (arq_common.cc:13930/13938),
  cross-storage PREV (arq_responder.cc:1318), the §5098 orphan path
  (arq_common.cc:5153), and bigblock (arq_commander.cc:17558). No delivery
  bypasses it → the cursor + STEP-2 backstop cover every path.

### 2.4 Init / reset / config-change invalidation
- `reset_session_state()` (arq_common.cc:7050): `tx_stream_committed=0;
  rx_stream_delivered=0; for all i: tx_stream_stamp[i].valid=false;
  rx_stream_stamp[i].valid=false`. A fresh session re-anchors both cursors at 0.
- Constructor / `init()` in-class default: same (mirrors the `cmd_batch_seq_id=0`
  ctor init, arq_common.cc:700).
- **`rx_stream_invalidate_stamps()` (F4.2, arq_common.cc) — called from
  `load_configuration()` right after the no-change early-return:** invalidates every
  `rx_stream_stamp[]` on ANY real config change. A config change ALWAYS re-stages the sender's
  in-flight batch, so a parsed-undelivered RX stamp is stale by construction; a demote-to-ROBUST
  rebuild (robust frames carry NO stamp) would otherwise leave a stale OFDM-sized
  `stamp[bsi].length` that permanently WITHHOLDs the byte-gate (BREAK spiral). Both W predicates
  no-op on an invalid stamp, so the rebuilt batch re-parses a fresh stamp on its EOB frame.
  Also closes the 256-wrap stale-start false teardown. Fail-before: `MERCURY_W_CFG_STAMP_KEEP`.
  See silent-corruption-residual.md §16.2/§17.5.

---

## §3 CONSUMERS (every site that READS the new state) — file:line

### FOUNDATION (this commit) — cursors only, NO wire, NO gate yet
- The regression test (`test_stream_offset.cc`, `--test-stream-offset`) reads the
  cursors + stamps to assert the invariant at every transition. No production
  consumer branches on the cursors yet.

### CORE W (STEP 2) — adds the wire stamp + the two RSP checks — **SHIPPED**
(commits: STEP 2a wire = `fc94ad43`, STEP 2b checks = this commit. file:line below
are on the STEP-2 branch.)
- **Gate helper** `w_stamp_rides()` (arq.h, inline): `sack_v2_enabled &&
  header_carries_d5 && max_frame >= W_STAMP_MIN_MAXFRAME`. DETERMINISTIC on both
  peers (both run load_configuration) → no wire negotiation. Both the TX reserve and
  the RX parse gate on it, so they always agree.
- **Sender emit:** `send_batch` (arq_common.cc, after the type dispatch, before the
  payload copy ~:9124): on any frame with the EOB bit at `w_stamp_rides()` configs,
  writes `{start_lo32:u32 LE, length16:u16 LE}` = `W_EOB_STAMP_BYTES`(6) from the
  LATCHED `tx_stream_stamp[bsi]` (bsi = the frame's `batch_seq_id`, the ORIGINAL bsi
  on retx/mixbatch) AFTER the D5 byte, growing header_length by 6. NEVER recompute,
  NEVER keyed off counts. The ERR-HDR-OVERFLOW guard (:9167) is relaxed by the stamp
  width. Build RESERVES `W_EOB_RESERVE`(7) payload bytes on the EOB frame — raw leg
  (last-frame pop cap, arq_commander.cc :20623) + compressed leg (`batch_capacity -=
  7` :20233 + last-chunk cap :20544) — so the EOB frame is always a DATA_SHORT whose
  header+payload+stamp == the codeword C exactly (no overflow).
- **Receiver parse + store:** `w_parse_eob_stamp(stamp_off)` (arq_common.cc, def
  before copy_data_to_buffer) called from the DATA_LONG (:13185) and DATA_SHORT
  (:13251) parses; stores `rx_stream_stamp[bsi] = {start_lo32, length16, valid}` and
  shifts the payload past the stamp. Absent stamp = safe no-op (returns 0).
- **RSP PRIMARY check** (byte-GATE the clean ACK, arq_responder.cc right after
  `[ACK-GATE] PASS` :2487, before the backpressure hold): sum DELIVERED bytes = Σ
  RECEIVED/ACKED `messages_rx[i].length` for `i<data_batch_size`; if `<
  rx_stream_stamp[bsi].length` WITHHOLD the clean ACK (re-arm exactly like the
  partial-batch SACK-suppress hold — do NOT ACK/bump/deliver) so SACK partial-retx
  re-requests. Compares ONLY to the latched stamp.length (never expected/batch_size).
  `MERCURY_W_BYTEGATE_DEFEAT=1` = fail-before. Makes mechanism 1/4 impossible.
- **RSP BACKSTOP check** (`copy_data_to_buffer` top, before delivery): if
  `rx_stream_stamp[decrypt_delivered_bsi].valid` assert `stamp.start ==
  (uint32)rx_stream_delivered`; on mismatch raise LOUD `[RSP-V2-STREAM-SHIFT]` +
  `rsp_gap_abort_teardown()` + return (no shifted delivery). The stamp is CONSUMED
  (invalidated) at copy_data_done after the cursor advance, so a 256-batch wraparound
  reuse with a LOST EOB frame reads invalid (skip), never a stale start.
  `MERCURY_W_STREAM_SHIFT_DEFEAT=1` = fail-before.

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
- **O1** **RESOLVED (2026-07-03, Opus §16).** The non-compressed BREAK/climb legs are the
  OPEN-CODED reverse `push_front` loops at arq_commander.cc:715 (BREAK phase-1), :807
  (BREAK-EXHAUSTED), :6241 (GEARSHIFT FRAME-UP) — NOT funnel calls (see §2.2 CORRECTION). A
  from-scratch re-audit confirmed :715 already called `stream_tx_rollback_inflight()` but :807
  and :6241 did NOT → they left the cursor high → the RSP BACKSTOP false-tore-down a byte-correct
  session (the F1/F4.1 blocker). FIXED: the call added inline at :807 and :6241. Regression
  `--test-stream-offset` Part R drives the PRODUCTION re-stage (fail-before via
  `MERCURY_W_RESTAGE_ROLLBACK_DEFEAT`). The restore-from-BACKUP legs
  (arq_common.cc:6106/6194/6288/7501/3305) are a DISTINCT fresh-bsi class (no
  roll_back_cmd_bsi_to_inflight → bsi gap handled by the pre-existing v2 gap-gate BEFORE the
  BACKSTOP) — documented residual, not a cohort blocker (silent-corruption-residual.md §16.1).
- **O2** [?] Role-switch (SWITCH_ROLE) mid-stream: cursors are role/session-scoped
  and reset on session reset; a mid-stream role flip on a bidirectional transfer
  is out of scope for the WGN:25 one-directional cohort. Documented, not handled.
- **O3** STEP 2 wire — **DONE** (2a `fc94ad43`, 2b `0d72b63f`, 2c `996a2f8f`). The
  stamp rides the EOB DATA_SHORT frame (always DATA_SHORT by the W_EOB_RESERVE build
  reservation); both RSP checks enforce it; the 4-mechanism gate proves gated/loud.
  Note vs the original sketch: the stamp is NOT folded into the generic header-length
  fn (per §8.2 site 1) — it is an EOB-frame-only +6 with the payload reserved down, so
  non-EOB frames are byte-unchanged and the RX parses the stamp only on the EOB frame.
- **O4** STEP 3 add-ons — **DEFERRED** (turn-key spec in §8.6): running CRC-32 co-stamp
  (same-position value corruption / streaming-decompressor desync) + EOT
  total_committed_bytes check (last-batch tail-drop blind spot). Both touch the riskiest
  subsystems (build path / session teardown); out of scope for the correctness-first core.
- **O5** [?] `w_stamp_rides()` threshold: the stamp is skipped when
  `max_frame < W_STAMP_MIN_MAXFRAME`(15). Verify cfg0's max_frame (the smallest OFDM
  config) at cohort time; if cfg0's max_frame ≥ 15 the stamp rides there too (harmless —
  reservation leaves ≥8 payload). The 4 captured mechanisms are all cfg13-16, so the
  gate is live where it matters regardless.

---

## §8 STEP-2 (CORE W) TURN-KEY IMPLEMENTATION SPEC — the wire stamp + 2 RSP checks

FOUNDATION (commit `dd382db8`) landed the cursors + ground-truth test. STEP 2 puts the
latched stamp on the wire and adds the two RSP checks. It is ATOMIC (both ends rebuild —
no version bits, `iris_no_version_bits_preship`) and MUST land green+committed or not at
all (task rule: never a half-applied wire change). This section is the exact plan; it
MIRRORS the existing D5 `batch_total_frames` per-batch-field plumbing (a proven precedent
in this exact tree) so a follow-up agent can execute directly.

### §8.1 Wire layout — stamp on the EOB-bearing frame ONLY (once per batch, ~0.2%)
D5 already appends `batch_total_frames` as the LAST header byte on the EOB frame only.
Append the W stamp immediately after it, EOB-frame-only, at OFDM configs
(`header_carries_d5==true`; robust configs carry no D5 byte and no stamp — §8.5):
- stamp = `{ start_lo32 : uint32 LE (4B), length16 : uint16 LE (2B) }` = 6 B.
  `start_lo32 = tx_stream_stamp[bsi].start & 0xFFFFFFFF` (wrapping 32-bit; 4 GB window,
  ample per HF session). `length16 = tx_stream_stamp[bsi].length` (a batch's transported
  bytes never exceed ~16 KB, fits u16).
- New header lengths on the EOB frame: DATA_LONG 7→13 B, DATA_SHORT 6→12 B. The
  NON-EOB frames are UNCHANGED (stamp is EOB-only), so per-frame payload budget is
  unchanged for them; only the EOB frame loses 6 payload bytes.

### §8.2 The ~8 sites (D5-mirror), file:line on `bfacbcf6`+FOUNDATION
1. **Header-length fns** `effective_data_long_header_length` / `effective_data_short_header_length`
   (used for the EOB frame's payload budget): add +6 for the EOB frame when the stamp
   rides. CAUTION: these fns feed payload budgeting on BOTH ends and MANY call sites — the
   cleanest surgical approach is a SEPARATE `eob_stamp_len(header_carries_d5)` added only
   where the EOB frame is built/parsed, NOT folded into the generic header-length fn
   (avoids rippling the non-EOB budget). Verify the EOB frame's payload is sized down by 6.
2. **TX write (batched path):** arq_common.cc ~9063 (DATA_SHORT byte[5]) / ~9087
   (DATA_LONG byte[6]) — right after writing `batch_total_frames_wire`, when this is the
   EOB frame, append the 6 stamp bytes from `tx_stream_stamp[bsi]` (bsi = the frame's
   `batch_seq_id`). RETX/MIXBATCH: the retx frames carry a PRIOR bsi; look up
   `tx_stream_stamp[that_bsi]` and emit its LATCHED stamp (the stamp persists until the
   batch is credited/rolled-back). Mirror D5's `sack_retransmit_active` handling — but
   emit the LATCHED stamp, not 0 (W's stamp is KNOWN for a retx'd batch).
3. **TX single-send path:** arq_common.cc ~8738/8763 — single send() has no batch
   context; emit a sentinel (all-0 / a `stamp_present=0` flag) so RX does not mis-parse.
   (Single-send DATA is not a batch-delivery path; RX skips the W check for it.)
4. **RX parse:** arq_common.cc ~13124 (SHORT) / ~13187 (LONG) — where D5
   `rx_buffer_batch_total_frames` is staged from the EOB frame, ALSO parse the 6 stamp
   bytes into `rx_stream_stamp[bsi] = {start_lo32, length16, valid}`.
5. **RSP PRIMARY check (byte-GATE the clean ACK):** arq_responder.cc near ACK-GATE PASS
   (:2487) / BATCH-DONE (:2596) — BEFORE emitting the clean bitmap / BATCH-DONE for bsi,
   assert `delivered_transported_for_this_bsi == rx_stream_stamp[bsi].length16`. On
   shortfall: WITHHOLD the clean ACK (do NOT bump last_delivered, do NOT credit, do NOT
   flush backup) → the existing SACK partial-retx re-requests the missing frames. Makes
   mechanism-4 ("declare complete at a shrunk expected-count") IMPOSSIBLE — the byte
   count, not the frame count, gates completion. FALSE-FIRE: compare bytes to the LATCHED
   `length16` only, NEVER to `data_batch_size`/`rx_batch_total_frames`/expected-counts.
   `delivered_transported_for_this_bsi` = Σ ACKED `messages_rx[i].length` for the bsi
   (the same quantity `copy_data_to_buffer` accumulates as `delivered_transported`).
6. **RSP BACKSTOP check:** copy_data_to_buffer (arq_common.cc:14096) — BEFORE the append
   (before advancing rx_stream_delivered), if `rx_stream_stamp[bsi].valid` assert
   `rx_stream_stamp[bsi].start_lo32 == (uint32_t)rx_stream_delivered`. Mismatch → LOUD
   `[RSP-V2-STREAM-SHIFT] stamp.start=%u rx_delivered=%u bsi=%d` + `rsp_gap_abort_teardown`
   (arq_common.cc:10051) — catches any 5th positional mechanism that bypasses the PRIMARY
   gate, at the FIRST divergent byte (not 1.6 batches late like GAP-ABORT). The
   delivered-bsi is `decrypt_delivered_bsi` (already set before each copy_data_to_buffer).
7. **Header decl:** add `StreamStamp rx_stream_stamp[256];` to arq.h (RSP side) + init in
   reset_session_state (invalidate all).

### §8.3 Test extension (`test_stream_offset.cc`) — reproduce all 4 mechanisms
Add env-defeat arms + assert each is GATED (primary) or LOUD (backstop), never silently
delivered — fail-before(env-defeat)/pass-after (mirror the §13 restage test contract):
- **Mech-1 (EOB-undercount tail-drop):** deliver a batch with fewer ACKED frames than the
  stamp.length → PRIMARY withholds the clean ACK (assert no BATCH-DONE, SACK re-requests).
- **Mech-2 (CMD>RSP batch-size desync):** already loud via (B); assert W PRIMARY ALSO
  catches it by byte shortfall (defense-in-depth).
- **Mech-3 (re-stage orphan/reorder):** §13 fixes the CMD side; assert that if a shift is
  INJECTED (env), the BACKSTOP `start != rx_delivered` fires LOUD.
- **Mech-4 (c31w104 shrunk-count forward Δ):** deliver bsi with a stamp.start ahead of
  rx_stream_delivered → BACKSTOP LOUD at the first byte; and a complete-at-shrunk-count →
  PRIMARY withholds. Reproduce the Δ=+6847 shape from silent-corruption-residual.md §14.1.

### §8.4 Retx/mixbatch re-emit + rollback interplay (the false-fire guard)
The stamp is re-emitted verbatim on retx (from `tx_stream_stamp[bsi]`, still valid). On a
re-stage the FOUNDATION rollback resets the cursor to stamp.start and the rebuild
re-latches the SAME start (INV4) → the re-emitted/rebuilt stamp is consistent with the
receiver's anchor. No new false-fire is introduced by retx because the stamp VALUE is
position, not count.

### §8.5 Robust-config handling
`header_carries_d5==false` at ROBUST_0..2 (arq_common.cc:2252, is_robust_config). Those
batches carry batch=1 and no D5 byte; the stamp rides ONLY when `header_carries_d5==true`.
On robust, the RSP skips the W checks (no stamp present) — robust delivery is batch=1 and
already covered by the batch-relative path; the W gate re-engages at OFDM configs where the
4 mechanisms actually occur (all 4 captures corrupt at cfg13-16, §11-§14).

### §8.6 STEP 3 (add-ons) — **DEFERRED to a follow-up clip (turn-key spec below)**
Rationale (2026-07-03): CORE W (2a/2b/2c) is complete, tested, committed — the clean
checkpoint and the primary deliverable for a strong-oracle WGN:25 cohort. Both add-ons
touch the TWO riskiest subsystems (the build path; session teardown), so a rushed/untested
version would violate the no-untested-fix + don't-ship-half-wired discipline. They are
enhancements, not part of the 4-mechanism gate (which the two shipped checks already close).
Scoped here so a follow-up executes directly.

**(3a) Running CRC-32 co-stamp (+4 B on the EOB frame) — closes SAME-POSITION VALUE
corruption / per-frame-CRC escape / streaming-decompressor desync (§0).** Mirror the cursor's
snapshot mechanism EXACTLY so re-stage rollback is handled:
- Add `uint32_t crc` to `StreamStamp` (both tx and rx) + a running `uint32_t tx_stream_crc`
  (CMD) / `rx_stream_crc` (RSP), reset to the CRC-32 init in reset_session_state().
- CMD: in BOTH frame-split legs (raw pop loop / compressed comp_buf), fold the batch's
  committed transported bytes into `tx_stream_crc` incrementally. Snapshot
  `tx_stream_stamp[bsi].crc = tx_stream_crc_before_this_batch` at the latch (the value BEFORE
  folding — the anchor, exactly like `.start`). On re-stage rollback restore
  `tx_stream_crc = tx_stream_stamp[min_bsi].crc`; a rebuild re-folds from that anchor (INV4
  for the CRC). Emit the POST-batch running CRC on the wire (or emit the anchor + let RX fold
  — pick one and match ends).
- RSP: fold delivered transported bytes (the `assembled` ciphertext / raw messages_rx[i].data
  — the TRANSPORTED domain, NOT decompressed plaintext) into `rx_stream_crc` at
  copy_data_to_buffer; after each batch compare to the stamped running CRC; mismatch → LOUD
  `[RSP-V2-STREAM-CRC]` + teardown. Byte-exactness holds by INV5 (TX-committed == RX-delivered
  transported bytes). Test: a same-position byte-flip (position/length correct → W's positional
  checks INERT) → CRC mismatch caught; fail-before env defeats the compare.
- RISK the follow-up must clear: the CRC rollback-anchor must be maintained on EVERY re-stage
  path the cursor covers (the two funnels + BREAK legs, §2.2) or a healthy re-stage false-fires.

**(3b) EOT exchange (~8-12 B once) — the ONLY true end-to-end LAST-BATCH tail-drop check.** A
lost FINAL batch leaves no next-batch stamp to catch it (the W per-batch checks are inherently
one-batch-behind). At clean disconnect, CMD sends `total_committed_bytes` (= tx_stream_committed,
u64) [+ final tx_stream_crc if 3a shipped] in the DISCONNECT control frame; RSP asserts
`rx_stream_delivered == total_committed_bytes` [+ CRC] → LOUD `[RSP-V2-EOT-SHORT]` on mismatch.
Absent EOT (lost frame / abrupt drop) = safe no-op. RISK the follow-up must clear: the disconnect
path is session-teardown state with a documented SWITCH_ROLE / idle-switchrole-race history
(MEMORY.md) — wire the field into the EXISTING DISCONNECT control code, do NOT add a new
handshake, and audit the teardown race before touching it.

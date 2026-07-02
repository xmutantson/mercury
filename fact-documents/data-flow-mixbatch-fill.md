# data-flow-mixbatch-fill.md — final-partial-batch byte-integrity (TX fill over-pop)

Shared-state fact document for the CMD-side new-data staging path
(`messages_tx[]` fill) and its interaction with the v2 mixbatch retx prefix.
Built 2026-07-02 from the cfg8 WGN40 capstone byte-integrity failure exposed by
the SACK-de-dup stall fix (`af13507` → `c215754c`).

## §1 Symptom (the reported bug)

A COMPLETED cfg8 transfer HARD-FAILS delivered-byte integrity on the capstone
real-audio smoke (fleet .31, `Loopback_B`, WGN SNR3k=40, cfg100 start, pg84 4096 B,
`MERCURY_INBAND_RATE=1`): `byte_integrity_ok=false`, `good_prefix_bytes=3288`,
`integrity_mismatch_bytes≈813` from offset ≈3288, `rx_bytes=4101 > tx_bytes=4096`
(`delivered_exceeds_fed=true`). Batches 0/1 deliver byte-PERFECT; the corruption
begins on the FINAL partial batch.

The prior fact doc (`inband-dataplane-stall-post-leap.md` §8) attributed this to the
RSP forward/EOB path ("~12 stale frames appended, nframes=25"). **That attribution
was WRONG** (the `nframes=25` in `[RSP-MFSK-SACK]` is `data_batch_size`, the ACK
bitmap width — batches 0/1 also print `nframes=25` while delivering 25/24 frames).
Ground-truth instrumentation (below) shows the RSP delivers the CORRECT frame COUNT;
the frames are **out of order**, and the true root is CMD-side.

## §2 Ground truth (instrumented, two independent roots)

Instrumented the RSP delivery loop (`copy_data_to_buffer`, dump every ACKED slot's
`bsi/len/b0`) and the CMD TX fill (`[TXPOP]` each `fifo_buffer_tx.pop`, `[TXNEW]`
each new-data slot's `pos/len/b0`) and every `fifo_buffer_*` push. Re-ran the smoke.
The final batch's delivered bytes decode (positional pattern `byte[k]=k&0xFF` +
block-index) to a **rotation**: content chunk `c` sits in slot `(c-1) mod N`, so the
batch's FIRST chunk (offset 3288) is delivered LAST, plus a 5-byte tail overage.

**Root A (dominant — the ~813-byte scramble).** CMD `[TXNEW]` for the final batch:
`txslot=0..11 pos=0..11` (fresh, offsets 3355-4100) then `txslot=23 pos=12`
(offset 3221) and `txslot=24 pos=13` (offset 3288) — the batch's two EARLIEST
chunks assigned the HIGHEST ids. Traced to the PRIOR batch: it was a mixbatch
(`2 retx bsi=0 + 23 new bsi=1`); the fill popped 25 new frames but only 23 fit
after the retx prefix, leaving 2 surplus (offsets 3221, 3288) in `messages_tx[23]/[24]`.

**Root B (secondary — the +5 phantom, `rx=4101`).** The single 5-byte over-read is
the already-delivered ROBUST batch (offset 0-4) re-staged from `fifo_buffer_backup`
to the TX-FIFO **tail** at the robust→cfg8 climb and re-sent at the end. See §7.
Root B is PRE-EXISTING (`rx=4101` in every run, masked by Root A's larger scramble)
and is NOT fixed here (§7.3).

## §3 The state: `messages_tx[]` new-data fill

- **Producer P1 (fill)** — `process_buffer_data_commander()` no-compression leg,
  `arq_commander.cc:~21141-21165`. Pops up to `fill_limit` frames of `max_frame`
  bytes from `fifo_buffer_tx` and calls `add_message_tx_data()` per frame.
- **Producer P2 (slot alloc)** — `add_message_tx_data()`, `arq_commander.cc:2051-2074`.
  Writes each frame to the FIRST `FREE` `messages_tx[]` slot (ascending), sets
  `status=ADDED_TO_LIST`, `id=i`.
- **Producer P3 (mirror)** — `fifo_buffer_backup.push()` at `arq_commander.cc:21153`
  (every popped frame is also backed up for config-change re-frame / restore).
- **Consumer C1 (assemble)** — `process_messages_tx_data()`, `arq_commander.cc:2414-2586`.
  Builds a v2 mixbatch: R = `min(retransmit_count, data_batch_size)` retx frames in
  slots `[0,R)`, then iterates `messages_tx[]` ASCENDING and assigns each
  `ADDED_TO_LIST` frame `pos = message_batch_counter_tx - R`, `id = pos`
  (`set_batch_tx_slot`, :2318-2319), until `message_batch_counter_tx == data_batch_size`.
  The remainder stay `ADDED_TO_LIST` (unsent surplus).
- **Consumer C2 (RSP delivery)** — the RSP places each frame at `messages_rx[id]`
  (`add_message_rx_data`, `arq_responder.cc:55`) and `copy_data_to_buffer()` drains
  slots `[0,data_batch_size)` ASCENDING into the app FIFO.

## §4 The invariant C1/C2 rely on (violated)

**"`messages_tx[]` slot index == content (FIFO) order for new-data frames."**
C1 numbers `pos/id` by ascending slot; C2 delivers by ascending id. So the content
must be laid out in ascending slots. P2 (first-free-slot) preserves this ONLY while
slots free in order — which holds iff every staged frame is sent each batch.

## §5 Root A: how the invariant breaks (over-pop)

`fill_limit` was `data_batch_size` UNCONDITIONALLY, but C1 can only send
`data_batch_size - R` new frames on a mixbatch. When R>0 the fill over-stages by R;
C1 sends the first `data_batch_size - R` (ascending slot) and leaves R surplus in the
HIGH slots. Those slots stay occupied; the NEXT batch's fill (P2 first-free) puts
FRESH (later) content in the freed LOW slots, so C1 numbers the fresh content with
LOWER pos/id than the OLDER surplus → out-of-order delivery. rx>tx because a short
final frame is also mis-sized by the shifted layout.

Deterministically reproduced in-process: `--test-mixbatch-fill-overpop`
(`arq_commander.cc test_mixbatch_fill_overpop`) drives the REAL
`process_buffer_data_commander()` across two mixbatch cycles and reconstructs the
delivered stream. FAIL-BEFORE (`MERCURY_MIXBATCH_OVERPOP_DEFEAT=1`): cycle-1 stages
25 (over-pops), delivered stream REORDERS at offset 1541 (=23×67). PASS-AFTER: cycle-1
stages 23 (respects R), delivered stream byte-exact contiguous.

## §6 The fix (Root A) — one change, the fill respects the retx prefix

`arq_commander.cc:~21142` (no-compression fill). Cap `fill_limit` so the fill never
stages more new frames than the upcoming mixbatch can carry:

```
fill_limit = data_batch_size - min(retransmit_count, data_batch_size) - already_staged
```
(clamped ≥0; `already_staged` = current `ADDED_TO_LIST` count so repeated fills of one
batch don't over-stage). No surplus is ever created → slot order == content order →
C1/C2 invariant restored. Gated on `sack_v2_enabled`; `MERCURY_MIXBATCH_OVERPOP_DEFEAT=1`
reverts it (fail-before). No-retx / v1: `retransmit_count==0` ⇒ `fill_limit==data_batch_size`
(byte-identical to pre-fix).

**Walk every consumer (§3):** C1 now finds exactly the frames it will send, all sent,
all freed each batch → P2 re-fills from slot 0 in order → the invariant holds for the
next batch. C2 (RSP) unchanged. Throughput: was popping R wasted frames per mixbatch
(surplus); the cap removes that waste (no regression). `fifo_buffer_backup` (P3):
fewer frames pushed per over-staged batch, but backup is a per-batch re-frame net that
is flushed at every ACK/finalize and re-populated by the next fill — no consumer reads
a stale over-staged backup.

**Verified (real-audio smoke, cap-on arm):** `good_prefix_bytes` 3288 → **4096**
(the entire payload now delivers byte-perfect, in order); the transfer still SUSTAINS
(the de-dup fix intact, 3 batch deliveries, 0 breaks).

## §7 Root B (the +5 phantom) — diagnosed, NOT fixed here

### §7.1 Mechanism
On the robust→cfg8 climb the SUPER-ACK leap (`arq_common.cc:4687-4700`,
`roll_back_cmd_bsi_to_inflight("SUPERACK")`) queues a config change routed through the
in-band chokepoint to `inband_unilateral_config_change()` (`arq_common.cc:3511`). Its
"re-fill for the new config" loop (`:3562-3574`) frees `messages_tx[]` and restores any
leftover `fifo_buffer_backup` content into `fifo_buffer_tx` via a **tail** `push`
(:3570). The already-DELIVERED 1-frame robust batch (offset 0-4) is still in backup, so
it is re-appended to the TX-FIFO tail and re-sent at the END of the transfer → a 5-byte
duplicate (`rx=4101`, first bad offset at the payload end). The RSP cannot dedup it:
the robust frame delivered as `bsi=-1` but the re-send rides a later bsi.

### §7.2 Design contract already documents this
`inband_unilateral_config_change` comment (`arq_common.cc:3555-3561`): "For producers
that already restored to fifo_buffer_tx + flushed backup (the FRAME-UP path,
arq_commander.cc:6762-6776), backup is empty so this restore is a no-op." The FRAME-UP
climb producer flushes backup (`arq_commander.cc:6774`) + push_FRONTs `messages_tx`; the
SUPER-ACK leap producer does NEITHER → it VIOLATES the contract (non-empty backup here).

### §7.3 Why NOT fixed in this change (honest report)
The naive fix (flush backup in the SUPER-ACK leap) was BUILT + smoke-tested and
**REGRESSED** the data plane: `rx_bytes=2421` (partial), `good_prefix=5`, `break_any=14`,
transfer did not complete — flushing/reordering the backup disrupts the climb state
machine (LOSSLESS-PROMOTE bsi rollback, `cleanup()`, and the re-frame all read shared
state here). `push_front` instead of `push` only MOVES the duplicate to the stream start
(the robust `bsi=-1` still can't be deduped). Root B is a distinct, pre-existing
cross-layer bug (climb re-stage vs in-flight-ACK vs backup lifecycle) that needs its own
data-flow audit + regression; it is scoped out of this fix to avoid a bug-driven sibling
regression (CLAUDE.md §Cross-Layer). Root A resolves the reported ~813-byte scramble
(good_prefix 3288→4096); Root B is the residual +5.

### §7.4 Repro / evidence for the follow-up
`arq_eobfixA_capon` smoke (Root A only): `first_bad=4096`, `integrity_mismatch_bytes=1`,
`good_prefix=4096`, `rx=4101`. `[FIFO-PUSH sz=128000 len=5 b0=0]` at the climb (`b0=0` =
offset 0-4) is the re-staged delivered robust frame.

## §8 Regression

`--test-mixbatch-fill-overpop` (env `MERCURY_MIXBATCH_OVERPOP_DEFEAT` = fail-before).
In-process, no PHY/audio/TCP. Drives the REAL fill; reconstructs the RSP delivery order;
asserts byte-exact contiguity. Paired with this doc per CLAUDE.md §Cross-layer regression.

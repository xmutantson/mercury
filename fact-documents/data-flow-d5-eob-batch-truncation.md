# Data-Flow: D5 — EOB-inference batch truncation FIX (IMPLEMENTED)

**Status**: IMPLEMENTED + unit-proven on branch `sim/trackc-d5-coupled-chain`
(worktree `C:/Users/kamer/mercury_wt/coupled-chain`), branched from **monitor
`627c370`**. This is the Track-C STEP-2 deliverable: the INTEGRITY fix that stops
the lost-EOB-tail SILENT-WRONG-BYTES leak (valuable regardless of the CFG16 beat).
The full §5 cross-layer audit + the Direction-1 design live in
`bigblock_p3_hw/_trackc/TRACK_C_D2D3D5_DESIGN.md` §5.3 and
`fact-documents/data-flow-prev-bump.md` §8 (both on the design branch
`sim/trackc-d2d3d5-design`). THIS doc records the IMPLEMENTATION as landed on
monitor's line base, with every cite re-located against `627c370`.

---

## §1 The defect (one paragraph)

The prev cross-storage `expected_count`, the SACK partial-gate `expected`, and the
in-place ACK-gate `effective_batch` were all INFERRED from
`last_received_end_of_batch_seq` — a single-frame-of-evidence length channel (only
the EOB-bit-7-marked LAST frame carries the batch length). When that EOB frame is
lost, the inference latches a SHORT length, the genuinely-missing tail frame in
`[expected_count, data_batch_size)` is FREE, the COUNT gate fires, and
`copy_data_to_buffer()` concatenates the present slots — silently dropping the tail
(~155 B at CFG16), NO `[RSP-V2-GAP-ABORT]`, md5_match FALSE. The SACK span
(`expected = last_received_end_of_batch_seq + 1`) ALSO collapsed to exclude the lost
tail, so the RX could never even SACK it (the dead-end) — structurally unhealable by
the existing retx machinery (TRACK_C design §4.3).

## §2 The fix — Direction 1: carry the TX-authoritative count on the wire

Carry a per-batch `batch_total_frames` (the TX's `message_batch_counter_tx`, the
authoritative frame count) on EVERY v2 DATA frame of a batch, so it survives the
loss of ANY single frame including the EOB frame. Consumers prefer the wired count
over the EOB inference.

### §2.1 Wire format (monitor line base)
- New v2 header byte, APPENDED LAST so every PRIOR field's parse is byte-identical:
  - DATA_LONG v2: `[type, conn_id, seq(EOB bit7), batch_seq_id, id, batch_total_frames]`
    — `DATA_LONG_HEADER_LENGTH_V2` 5 → **6** (datalink_defines.h).
  - DATA_SHORT v2: `[type, conn_id, seq(EOB bit7), batch_seq_id, id, length, batch_total_frames]`
    — `DATA_SHORT_HEADER_LENGTH_V2` 6 → **7**.
- **ROBUST gate (the load-bearing cross-layer interaction):** robust configs
  (ROBUST_0..2, `is_robust_config`) are pinned to `data_batch_size=1` — D5 is
  MEANINGLESS at batch=1 (a 1-frame batch has no lost-tail-of-batch hazard) AND the
  extra header byte would steal the scarce ROBUST_0 payload (12-byte frame), dropping
  `max_frame` below `COMPRESS_HEADER_SIZE` (7) and re-opening the streaming-compression
  deadlock the `test_robust0_compress_deadlock` C0 invariant guards. So a per-config
  flag `header_carries_d5 = !is_robust_config(configuration)` (recomputed in
  `load_configuration`, arq_common.cc) gates the byte OUT at robust. The helpers
  `effective_data_{long,short}_header_length(sack_v2, with_d5=true)` (arq.h) take the
  flag; the no-D5 lengths are `DATA_*_HEADER_LENGTH_V2_NO_D5` (= the pre-D5 5/6). TX
  and RX both re-run `load_configuration` on the SET_CONFIG handshake, so they agree
  on the per-config wire header length.
- Gated under the EXISTING `sack_v2_enabled` capability — **byte-identical for
  v1/legacy/NB** (no new byte emitted; EOB inference unchanged). For a v2 NO-LOSS
  batch `rx_batch_total_frames == last_received_end_of_batch_seq + 1`, so the derived
  `expected_count`/`expected`/`effective_batch` are IDENTICAL — byte-identical on the
  faithful path. The change BITES only on the EOB-loss path (today silent-wrong-bytes).
- `0` on the wire = unknown/legacy → RX keeps any prior latched count (or falls back
  to the EOB inference). Retransmit frames (`sack_retransmit_active`) emit 0 (the retx
  radio batch ≠ the original crypto batch, so its frame count would mislead).

### §2.2 PRODUCERS (monitor 627c370)
- EOB-mark TX (unchanged): arq_common.cc:5611-5615 (bit-7 on last data frame).
- **batch_total_frames TX**: `send_batch()` arq_common.cc (the
  `batch_total_frames_wire = message_batch_counter_tx` block + the per-type writes at
  offsets [5]/[6], gated `header_carries_d5`); `send()` single-frame path writes 0.
- RX stage: `rx_buffer_batch_total_frames` set in the DATA_LONG/SHORT parse
  (arq_common.cc, offsets [5]/[6], gated `header_carries_d5`; 0 → -1).
- RX promote: match-current block arq_responder.cc (`rx_batch_total_frames =
  rx_buffer_batch_total_frames` when >0), match-prev block arq_responder.cc (latch +
  re-derive `rsp_prev_batch_expected_count` when a surviving prev frame reveals a
  longer count — the `[RSP-V2-D5-PREVEXP]` log).
- Reset: at the bsi bump (`bump_bsi_and_transfer_prev`, after consuming for
  prev_expected), at BATCH-DONE (arq_responder.cc), and at the loud gap-abort teardown
  (`rsp_gap_abort_teardown`) — so a stale count never poisons the next batch.

### §2.3 CONSUMERS switched from EOB inference → wired count (each `else if`-fallback)
- prev `prev_expected` — `bump_bsi_and_transfer_prev` arq_common.cc:6396-6418.
- in-place ACK-gate `effective_batch` — arq_responder.cc (the
  `if(rx_batch_total_frames>0 && !d5_infer_defeat_cur)` branch).
- SACK partial-gate `expected` — arq_responder.cc (same pattern). NOW the lost tail is
  INSIDE `expected` → `rx_received < expected` → the RX SACKs it → CMD retransmits →
  faithful. This is the heal path the inference foreclosed.

### §2.4 Composition (the integrity guarantee)
Wired count → a lost EOB leaves `received_count < expected_count` → the COUNT gate
(arq_responder.cc:786-787) HOLDS the prev (no L1 needed on monitor: the lost frame is
the HIGHEST seq, so no tail slot can over-count it). Then EITHER the SACK retransmit
fills the tail → FAITHFUL, OR the prev is stale-discarded undelivered → high-water
un-advanced → the next current commit trips `delivery_step_is_gap` (D3.1) → LOUD
`[RSP-V2-GAP-ABORT]`. **Faithful-after-retx OR loud-abort — never a silent skip.**

## §3 Failing-test-first (CLAUDE.md §3)
`--test-eob-loss-batch-truncation` (arq_responder.cc::`test_eob_loss_batch_truncation`,
CLI in main.cc). Drives the REAL `bump_bsi_and_transfer_prev()` producer, the REAL
prev-completion count gate, the REAL `copy_data_to_buffer()` via the production
messages_rx↔messages_rx_prev pointer swap, and the REAL `fifo_buffer_rx` as a
byte-exact oracle (compression OFF, distinct per-slot bytes `slot*16+j`).
- CASE-A (lost EOB tail, slot 29 of a 30-frame batch):
  - **fail-before** (`MERCURY_D5_INFER_DEFEAT=1`): `prev_expected=29` → count gate
    fires with slot 29 FREE → 464 B (29 frames) delivered, slot-29 bytes ABSENT →
    `truncated_29=1 faithful_30=0` (the silent skip is reproduced).
  - **pass-after** (default): `prev_expected=30`, `received=29<30` → gate HOLDS
    (0 B delivered); inject the slot-29 retransmit → received=30 → all 30 frames
    deliver in order, 480 B, `faithful_30=1`.
- CASE-B (no-loss 30-frame batch): faithful in BOTH modes (wired == inference == 30).
Executed: default `[TEST-D5-EOBLOSS] PASS: fails=0 (defeat=0)`; defeat
`PASS: fails=0 (defeat=1)`. Both rc=0.

## §4 No-regression battery (executed, FIX binary `mercury_trackC.exe`)
ALL rc=0: `--test-eob-loss-batch-truncation` (default + defeat),
`--test-eob-poison-prev-retx`, `--test-batch-shrink-strands-prev`, `--test-gap-abort`,
`--test-inorder-demote`, `--test-sack-oow-reject`, `--test-retx-clear-on-recovery`,
`--test-clean-batch-viability`, `--test-v2-pendingack-flip-alias`,
`--test-partial-bsi-advance={mfsk,ofdm}`, `--test-climb-engine`,
`--test-data-anchored-promote`, **`--test-robust0-compress-deadlock`** (the
deadlock-floor C0 — the cross-layer interaction that surfaced the robust gate; back to
ALL PASS, max_frame=7), `--test-phantom-ack-gate`, `--test-rx-drain-backpressure`,
`--test-bigblock-arq-unit` (8/8, incl. T8 synth-EOB → expected_count=8 NOT 25),
`--test-bigblock-multicw` (ALL PASS, byte-OK), `--test-bigblock-carve-suspend-unit`,
`--test-bigblock-acqwindow`, `--test-ofdm-fine-timing`, `--test-probe-backoff`,
`--test-policy-axis{1,2,3}-{fire,miss}`.

## §5 Scope / what this fix does NOT do
- It does NOT change v1/legacy/NB wire format (byte-identical) nor the robust frame
  (byte-identical via the `header_carries_d5` gate).
- It is the INTEGRITY fix only (D5). D2/D3 (the CFG16-acquisition climb fix) are
  SEPARATE and live in `TRACK_C_D2D3D5_DESIGN.md` §5.1/§5.2 — NOT in this commit.
- It is INERT until the climb reaches the prev-bump-under-CFG16 regime, but harmless
  before then; it is a STRICT integrity improvement (silent-skip → faithful/loud-abort)
  on the EOB-loss path.

## §6 Cross-layer audit checklist (future changes)
Before changing ANY of: the v2 header layout (datalink_defines.h
`DATA_*_HEADER_LENGTH_V2[_NO_D5]`); the helpers `effective_data_*_header_length`
(arq.h); `header_carries_d5` (set in `load_configuration`); the batch_total_frames TX
(`send`/`send_batch` arq_common.cc) or RX parse/promote (arq_common.cc/arq_responder.cc);
the prev `prev_expected` (arq_common.cc:6396), the SACK `expected`, or the in-place
`effective_batch` — walk §2/§3/§4 here AND update
`fact-documents/data-flow-prev-bump.md` + `TRACK_C_D2D3D5_DESIGN.md`.
**Special vigilance**: (i) the ROBUST deadlock floor (`max_frame >= COMPRESS_HEADER_SIZE`)
must hold — never let the D5 byte ride at a robust/batch=1 config. (ii) any consumer
that re-derives a batch length from `last_received_end_of_batch_seq` re-opens D5 unless
the wired count is the PRIMARY source.

# Data-Flow Audit: streaming-compression × `batch_capacity` × frame-fill

**Status**: Authoritative as of 2026-06-01 on `fix/robust0-compress-deadlock`
(off `monitor` @ `fef293f`). Paired regression test:
`mercury --test-robust0-compress-deadlock` (in-process, no PHY/audio/TCP).
Every change to the commander data-fill path
(`arq_commander.cc::process_buffer_data_commander`), to `compress_block()`'s
fit logic, or to the per-config frame budget that feeds `batch_capacity` MUST
update this document.

**Context**: This document owns the producer/consumer/invariant facts for the
interaction between the **streaming compression context**, the per-config
**batch capacity** (`data_batch_size × max_frame`), the **TX frame-fill** in
`process_buffer_data_commander()`, and **forward progress** (application bytes
actually delivered, FIFO drain, clean-batch climb credit). It exists because of
the ROBUST_0 compression deadlock found on hardware 2026-06-01 (agent
aa92936f): with compression ON, Mercury delivered **0 application bytes at any
SNR** since every ARQ session now starts at ROBUST_0 (batch=1) after
MFSK-CONNECT shipped. CLAUDE.md §5 mandates auditing the shared state instead
of patching the proximate symptom.

Key declarations:
- `cl_compressor compressor;` member of `cl_arq_controller` (`arq.h:1786`).
- `bool compression_enabled;` (`arq.h:1787`).
- `int batch_uncompressed_size;` (`arq.h:1792`) — TX throughput stat only.
- `int data_batch_size;` (see `data-flow-batch-size.md`) — pinned to 1 at
  robust (`arq_common.cc:1334`).
- `COMPRESS_HEADER_SIZE = 7` (streaming, with CRC16),
  `COMPRESS_HEADER_SIZE_LEGACY = 5` (`mercury_compress.h:38-39`).
- `get_header_size() = streaming_active ? 7 : 5` (`mercury_compress.h:83`).

---

## §1 The invariant

> **At ANY config — including ROBUST_0 (batch=1, smallest frame) — one
> `process_buffer_data_commander()` data-fill that pops `N>0` raw bytes from
> `fifo_buffer_tx` MUST stage a batch that carries `>0` application bytes to
> the peer, and the FIFO MUST drain by the carried byte count.**

If a fill pops bytes but stages a batch carrying 0 application bytes (and pushes
all popped bytes back), the link sends a DATA frame every cycle yet delivers
nothing: the FIFO never drains, `batch_uncompressed_size` stays 0, no clean
data batch ever completes, and the gearshift gets no climb signal. That is a
deadlock — 0 bps forever, even at high SNR.

### §1.1 Why the invariant breaks at ROBUST_0 (the measured numbers)

`load_configuration(100 / ROBUST_0)` computes (verified empirically via
`mercury -m PLOT_PASSBAND -s 100`: `nBits=1600`, `LDPC_rate=0.062` = 1/16):

```
get_frame_size_bytes() = (nBits - ldpc.P - outer_code_reserved_bits)/8
                       = (1600 - 1500 - 0)/8 = 12 bytes              (telecom_system.cc:480)
nBytes_header = max(ACK_MULTI=3, CONTROL_ACK=3,
                    eff_long_v2=5, eff_short_v2=6) = 6                (arq_common.cc:1296-1311)
max_data_length   = 12 - 6 = 6                                       (arq_common.cc:1313,1317)
max_header_length = 6
max_frame = max_data_length + max_header_length
          - effective_data_long_header_length(sack_v2=true)
        = 6 + 6 - 5 = 7                                              (arq_commander.cc:8708)
data_batch_size = 1   (robust pin)                                  (arq_common.cc:1334)
batch_capacity = data_batch_size * max_frame = 1 * 7 = 7            (arq_commander.cc:8719)
```

So at ROBUST_0 **`batch_capacity == 7 == COMPRESS_HEADER_SIZE`**. The streaming
compression header alone exactly fills the per-batch budget, leaving **0 bytes**
for any compressed (or raw-fallback) payload.

`compress_block(in, in_len, out, out_capacity=7)` (`mercury_compress.cc:436`):
- computes `total = hdr_size(7) + best_comp_size(≥1 for any non-empty input)`,
- `total > out_capacity(7)` is always true → returns **-1**
  (`mercury_compress.cc:526-536`), AND on the streaming path calls
  `streaming_reset()` first (`:530-534`).

### §1.2 Why the symptom is "0 bytes forever", not "no frame"

The data-fill's `comp_size == -1` handler and RAW fallback
(`arq_commander.cc:8786-8879`) DO stage a frame — an empty one:

1. Streaming `-1` branch (`:8789-8799`): push back 40% of `raw_size`,
   `streaming_reset()`, `break`. `compress_ok` stays **false**.
2. RAW fallback (`:8837`): `max_raw = batch_capacity - hdr_sz = 7 - 7 = 0`.
   `raw_size (>0) > max_raw (0)` → push back ALL of `raw_size`, set
   `raw_size = 0`.
3. Build a 7-byte ALGO_RAW header, `memcpy(comp_buf+7, staging, 0)`,
   `comp_size = 7 + 0 = 7`. Set `batch_uncompressed_size = raw_size = 0`
   (`:8871`).
4. Frame split (`:8944`): `chunk = 7 == max_frame` →
   `add_message_tx_data(DATA_LONG, 7, …)` accepts (7 ≤ 7) → **one 7-byte
   header-only DATA frame is staged**, carrying 0 application bytes.

**Measured** (`mercury --test-robust0-compress-deadlock` on `fef293f`):
`[COMPRESS-TX] 0 raw -> 7 comp (1 frames), ratio_est=2.00, fill=100%` and
`batch_uncompressed_size=0 staged_frames=1 fifo_before=4096 fifo_after=4096`.
The FIFO does not drain; the cycle repeats indefinitely.

### §1.3 Why compression-OFF delivers (the asymmetry)

The no-compression branch (`arq_commander.cc:8971-8998`) pops `max_frame` bytes
straight into a frame with **no compression header** (the DATA frame header is
the ARQ header already accounted for in `max_frame`). A ROBUST_0 frame therefore
carries up to `max_frame = 7` application bytes. `batch_uncompressed_size +=
data_read_size > 0`, the FIFO drains, batches complete, the link climbs. The
7-byte compression header is the entire difference.

### §1.4 Why it was never caught before

- Q-table calibrations and SACK A/Bs run `compress='off'` (sack_lossy_ab.py
  passes `-F off`; memory: "cals/A-Bs run compress=OFF").
- Older compress-ON runs were OFDM-pinned (turboshift started high), never the
  ROBUST_0→climb production path. Since MFSK-CONNECT shipped (monitor 41ac734),
  EVERY session starts at ROBUST_0, exposing the smallest-frame worst case to
  the compression path for the first time.

---

## §2 Producers — every path that writes the shared state

### §2.1 The compressed/raw batch buffer + `batch_uncompressed_size`
`process_buffer_data_commander()` (`arq_commander.cc:8655-9024`),
`compression_enabled` branch (`:8710-8970`):
- `:8738` `raw_size = fifo_buffer_tx.pop(staging, initial_pop)` — pops raw.
- `:8758` `compress_block(staging, raw_size, comp_buf, batch_capacity)`.
- `:8826` (compress_ok path) `batch_uncompressed_size = raw_size`;
  `:8825` `fifo_buffer_backup.push(staging, raw_size)`;
  `:8829` `compressor.set_pending_raw(staging, raw_size)` (streaming).
- `:8871` (RAW-fallback path) `batch_uncompressed_size = raw_size` (== 0 in the
  bug); same backup/pending writes.
- `:8944-8962` frame split → `add_message_tx_data(DATA_LONG/SHORT, chunk, …)`
  writes `messages_tx[]`.
- `:8794/:8803/:8845` `fifo_buffer_tx.push_front(...)` — pushes unsent raw back.

### §2.2 The streaming compression context (PPMd carry + zstd prefix)
`cl_compressor` (`mercury_compress.cc`). Producers that mutate the model/prefix:
- `streaming_enable()` (`:201`) — arm; `ppmd_model_warm=false`,
  `stream_batch_count=0`.
- `compress_block()` (`:436`) — `ppmd_compress`/`zstd_compress_buf` advance the
  model as a side effect; `set_pending_raw` stashes the raw for the post-ACK
  commit.
- `streaming_reset()` (`:244`) — re-init PPMd, drop prefix/pending; called on
  every `-1` (`:530-534`) and from the data-fill `-1` branch (`:8798`).
- `commit_pending()` — promotes the pending raw into the committed model after a
  successful data ACK; called from `finalize_block_commander()`
  (`arq_commander.cc:4833`).

### §2.3 `data_batch_size` / `max_frame` (the capacity inputs)
Owned by `data-flow-batch-size.md`. `set_data_batch_size(1)` at robust
(`arq_common.cc:1334`); `set_max_buffer_length()` sets `max_data_length`/
`max_header_length` (`arq_common.cc:547`, called from `load_configuration`
`:1317`). `max_frame` is derived read-only at `:8708`.

---

## §3 Consumers — every path that reads the shared state

### §3.1 Delivered-payload / FIFO-drain (the invariant's subject)
- The peer RX: `decompress_block()` (`mercury_compress.cc:577`) reads the staged
  frame's compression header and yields the application bytes. A 0-payload
  ALGO_RAW frame yields 0 bytes — nothing is delivered to the RX application.
- `fifo_buffer_tx` free space — drains only if popped bytes are NOT pushed back.

### §3.2 Throughput stat
`finalize_block_commander()` (`arq_commander.cc:4836-4838`):
`if(batch_uncompressed_size>0) gui_add_throughput_bytes_tx(...)`; then resets to
0. (GUI-only; not a control gate.)

### §3.3 Streaming commit / desync
`finalize_block_commander()` (`:4832-4833`) `commit_pending()` after a data ACK.
With the bug, an empty batch is ACK'd and an empty advance is committed each
cycle.

### §3.4 Gearshift clean-batch climb gate (downstream)
The FRAME-UP climb requires a CLEAN fully-delivered batch
(`promotion_allowed_on_batch(last_batch_fully_acked)`,
`arq_commander.cc:3709`, and `consecutive_data_acks`). A perpetual stream of
0-payload batches yields no genuine data delivery → the climb has no real
signal. (Secondary effect; the primary defect is 0 delivered bytes.)

---

## §4 Valid states (enumerated) + the broken one

For a single `process_buffer_data_commander()` fill with `compression_enabled`:

| # | config | batch_capacity | compress_block | staged payload | FIFO drains | OK? |
|---|--------|----------------|----------------|----------------|-------------|-----|
| S1 | OFDM (e.g. CONFIG_10) | ≫ 7 (e.g. 25×~30) | >0 (fits) | >0 | yes | ✓ |
| S2 | OFDM, incompressible block | large | -1 then push-back+retry, or RAW-fallback with `max_raw>0` | >0 | yes | ✓ |
| S3 | ROBUST_0, compress OFF | 7 | n/a (raw frame) | up to 7 | yes | ✓ |
| **S4** | **ROBUST_0, compress ON** | **7 == hdr** | **-1 every time** | **0** | **no** | **✗ DEADLOCK** |

S4 is the broken state: `batch_capacity ≤ get_header_size()` makes a payload
impossible. The RAW fallback's `max_raw = batch_capacity - hdr_sz ≤ 0` is the
exact arithmetic that zeroes the payload.

### §4.1 Pre-write (default-init) states
`batch_uncompressed_size` ctor = 0 (`arq_common.cc:313`). Streaming model:
`ppmd_model_warm=false`, `stream_batch_count=0` after `streaming_enable()`. On
the first ROBUST_0 batch the model is cold, so `compress_block` takes the
non-warm branch (`mercury_compress.cc:474`, zstd+PPMd), still returns -1 because
`hdr+payload>7`.

---

## §5 What the fix changes + per-consumer walk

**Root cause**: the commander data-fill assumes `batch_capacity` always leaves
room for the compression header + ≥1 payload byte. At ROBUST_0
`batch_capacity == get_header_size()`, so that assumption is false and the RAW
fallback degenerates to a 0-byte payload.

**Why the naïve fix is WRONG (the §6 trap)**: a first instinct is "route
ROBUST_0 batches through the existing uncompressed `else` branch at `:8971`".
That branch emits a **headerless** raw frame. But the RX decompress path is
gated **session-wide** on `if(compression_enabled)` (`arq_common.cc:6889`): when
compression is negotiated, the RX *always* reassembles the batch and calls
`decompress_block()`, which parses a 7-byte streaming header
(`mercury_compress.cc:579,587-590`). A headerless TX frame would be mis-parsed
as a compression header → corruption. The TX gate (`:8710`) and the RX gate
(`:6889`) are a **matched pair on the same `compression_enabled` boolean**; the
fix must flip them together, never just one.

**Minimal fix (chosen) — symmetric capacity gate on both TX and RX**:
introduce one pure helper

```
bool cl_arq_controller::compression_viable_for_batch() const {
    // Compression can carry payload only if the per-batch budget exceeds the
    // streaming header. At ROBUST_0, batch_capacity == get_header_size() == 7,
    // so this is false and the session uses the (symmetric) uncompressed path.
    if(!compression_enabled) return false;
    int max_frame = max_data_length + max_header_length
                  - effective_data_long_header_length(sack_v2_enabled);
    int batch_capacity = data_batch_size * max_frame;
    if(cipher_suite.is_active()) batch_capacity -= AUTH_TAG_SIZE;
    return batch_capacity > compressor.get_header_size();
}
```

and use it at BOTH gates:
- TX `process_buffer_data_commander()` `:8710` `if(compression_enabled)` →
  `if(compression_viable_for_batch())`.
- RX `copy_data_to_buffer()` `:6889` `if(compression_enabled)` →
  `if(compression_viable_for_batch())`.

Both peers compute the SAME answer from the SAME inputs — **no wire negotiation
required** (consistent with the project's "no negotiation pre-ship" rule): at
robust both sides hold `data_batch_size == 1` (the `data-flow-batch-size.md` §1
invariant, enforced) and the same `current_configuration`, so `max_frame` and
`batch_capacity` are identical on both sides. When the link climbs to an OFDM
rung, `batch_capacity ≫ header` → the helper returns true again on both sides
→ compression resumes. The streaming model is left **frozen** (untouched, not
reset) on both sides while at robust, so it stays in lock-step.

This is the minimal root fix: it removes the impossible-capacity case
(`max_raw = batch_capacity - hdr ≤ 0`) at its source by not entering the
compression path when the budget cannot hold a payload, instead of papering
over the -1 in the fallback. It MUST work at ROBUST_0 (batch=1, smallest frame =
worst case) — and it does, falling through to the byte-for-byte raw path that
already delivers at ROBUST_0 with compress OFF (state S3), with BOTH sides in
agreement.

**Per-consumer verification of the changed assumption**:
- §3.1 delivered payload: TX `else` branch (`:8971`) stages up to `max_frame`(7)
  raw app bytes/frame; RX `else` branch (`:7046`) pushes each frame's raw bytes
  straight to `fifo_buffer_rx`. Symmetric, headerless, delivers >0. FIFO drains.
  Invariant §1 restored. ✓ (asserted by C2/C3/C4).
- RX `decompress_block()`: NOT called for a robust batch (the `:6889` gate is
  now false on the RX too). No header mis-parse. ✓
- §3.2 throughput stat: TX `else` sets `batch_uncompressed_size += data_read_size
  > 0` → GUI counts real bytes. ✓
- §3.3 / §6.1 streaming hygiene: TX `else` branch never calls `set_pending_raw`,
  so `pending_raw_len` stays 0; `commit_pending()` on the robust batch ACK is a
  no-op (`mercury_compress.cc:269` early-return). RX `else` never calls
  `streaming_commit`, so the RX model does not advance either. Model frozen,
  symmetric. **[VERIFIED §6.1]**: see §6.1 for the stale-pending edge case.
- §3.4 climb gate: real data delivers → clean batches complete → climb signal
  is genuine. ✓

**Alternatives considered & rejected**:
- *Route robust through the existing TX `else` only (not the RX)*: breaks the
  matched TX/RX gate pair (§6 trap) → RX mis-parses headerless frame. REJECTED.
- *Emit ALGO_RAW WITH a 7-byte streaming header at robust*: the header alone is
  7 bytes == the whole 7-byte frame at batch=1 → still 0 payload. Does not solve
  S4.
- *Split a compressed block across multiple ROBUST_0 frames*: batch=1 = one
  frame; nothing to split into. N/A.
- *Switch to the legacy 5-byte header at robust*: leaves only 2 payload
  bytes/frame AND requires the RX `hdr_size` (its own `get_header_size()`) to
  match — fragile, and worse throughput than the 7-byte raw path. REJECTED.

---

## §6 RX-side symmetry audit  [RESOLVED 2026-06-01]

The fix flips the COMMANDER TX **and** the RESPONDER RX off the compression path
together, via the shared `compression_viable_for_batch()` helper, so a robust
batch is a plain headerless DATA frame on both sides.

- **[VERIFIED]** RX decompress gate is session-wide `if(compression_enabled)`
  at `arq_common.cc:6889`; its `else` (`:7046`) pushes each `messages_rx[i]`
  frame's raw bytes directly to `fifo_buffer_rx` with no header parse — the
  exact mirror of the TX `else` (`:8971`). Changing BOTH gates to
  `compression_viable_for_batch()` keeps them matched.
- **[VERIFIED]** `decompress_block()` (`mercury_compress.cc:587-590`) reads
  `algo` and the streaming flag from the per-frame header — but it is simply not
  reached for a robust batch under the fix, so per-frame-vs-session
  interpretation is moot at robust.
- **[VERIFIED]** Both sides hold identical `data_batch_size`(=1, per
  `data-flow-batch-size.md` §1) and `current_configuration` at robust ⇒
  identical `batch_capacity` ⇒ identical helper result ⇒ no wire mismatch
  without negotiation.

### §6.1 Streaming `pending_raw` / model hygiene  [RESOLVED 2026-06-01]
- `commit_pending()` (`mercury_compress.cc:267-272`) early-returns when
  `pending_raw_len <= 0`. `set_pending_raw()` is only called on the TX
  compression path (`arq_commander.cc:8829`, `:8874`). Under the fix the robust
  batch takes the TX `else` path, never calls `set_pending_raw`, so
  `pending_raw_len` is 0 and the post-ACK `commit_pending()` is a no-op for the
  model. ✓
- **Stale-pending edge case**: if an OFDM batch had set `pending_raw` and then a
  BREAK dropped the link to robust BEFORE that batch's ACK, the pending could
  linger. But the BREAK→robust recovery path calls
  `restore_tx_from_compressed()` (`arq_commander.cc:254/340/3257`) which
  reassembles + re-pushes the in-flight data, and the per-batch
  `commit_pending()` only fires on a genuine data ACK (`:4833`) for a batch that
  set pending. The robust batches under the fix never set pending, so they
  cannot commit a *stale* value: `commit_pending` commits whatever
  `pending_raw_len` currently is, and that value is only ever written by the
  immediately-preceding compression-path fill. To be defensive and make the
  freeze explicit, the fix ALSO calls `compressor.clear_pending()` once when a
  batch is routed through the non-viable (robust) path, documented at the call
  site. This guarantees no OFDM-set pending survives into a robust ACK. ✓

---

## §7 BREAK-recovery / `restore_tx_from_compressed()` interaction  [VERIFIED 2026-06-01]

A separate consumer of `messages_tx[]` content is the BREAK / FRAME-UP-DATA-FAIL
recovery, which (when `compression_enabled`) calls `restore_tx_from_compressed()`
to recover in-flight data for re-send at the new config
(`arq_commander.cc:254`, `:340`, `:3257`). Concern raised by the fix: under the
fix a robust batch's `messages_tx[]` frames are **headerless raw** (not a
compression block), so a path that fed them to `decompress_block()` would
mis-parse them.

**VERIFIED SAFE** — `restore_tx_from_compressed()` (`arq_common.cc:7094`) has a
FIRST guard `if(compressor.is_streaming())` (`:7099-7109`): it
`streaming_reset()` + `clear_pending()` + `restore_backup_buffer_data()` and
**returns before any `decompress_block()` call**. In production streaming is
ALWAYS on when `compression_enabled` (streaming is unconditional — MEMORY:
"Streaming compression is unconditional (CAP_STREAMING removed)"), so the
streaming branch is always taken and the `decompress_block` path at `:7148` is
never reached for a streaming session. The recovery restores from
`fifo_buffer_backup`, which the fix's uncompressed `else` branch STILL populates
(`arq_commander.cc` `else` path: `fifo_buffer_backup.push(message_TxRx_byte_buffer,
data_read_size)`), so the raw bytes are present and correctly re-queued. The
non-streaming `decompress_block` fallback (`:7145-7164`) further has a `dec_size
<= 0 → restore_backup_buffer_data()` safety net even if it were ever reached.
No mis-decompression is possible. ✓

---

## §8 Test + verification results (2026-06-01, fix/robust0-compress-deadlock)

**Repro test** `mercury --test-robust0-compress-deadlock`
(`arq_commander.cc::test_robust0_compress_deadlock`):
- FAIL-BEFORE (on `fef293f`, pre-fix logic): C2 FAIL
  `batch_uncompressed_size=0`, C4 FAIL `drained=0`; log
  `[COMPRESS-TX] 0 raw -> 7 comp (1 frames) fill=100%`, FIFO `4096 -> 4096`.
- PASS-AFTER (with the fix): ALL PASS — `batch_uncompressed_size=7`,
  `staged_frames=1`, FIFO `4096 -> 4089` (drained by exactly 7). The link makes
  forward progress at ROBUST_0 with compression enabled.

**Regressions** (see commit message / session log for the run output):
- `mercury --test` (MFSK ctrl-codec + LDPC/passband suite): GREEN.
- Behavior is byte-identical to `fef293f` whenever
  `compression_viable_for_batch()` is true (all OFDM rungs:
  `batch_capacity = batch(≥5..25) * max_frame(≫7) ≫ 7`). The ONLY behavior
  change is at robust configs (100/101/102, batch pinned to 1), where both TX
  and RX now use the headerless uncompressed path — the same path compress-OFF
  already uses successfully at ROBUST_0.

**Hardware confirmation still required**: this is a SIM/loopback validation. The
fixed compression path must be confirmed to deliver on the IONOS testbed at
ROBUST_0 (the original HW symptom from agent aa92936f) and to resume compression
correctly when the link climbs to OFDM. NOT merged to monitor — committed on
`fix/robust0-compress-deadlock` for review (production-critical).

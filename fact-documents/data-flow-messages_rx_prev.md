# Data-Flow Audit: messages_rx_prev[]

**Status**: Authoritative as of 2026-05-26 (after R7 fix on `fix/r7-loc-bound`,
paired with this document). Every change to `messages_rx_prev[]` or the
`rsp_prev_batch_*` counters MUST update this document.

**Context**: `messages_rx_prev[]` is the SACK Design A Step 8a parallel
storage for prev-batch retransmits. It exists because the v2 SACK path
bumps `rsp_current_expected_batch_seq_id` at SACK_RSP send time (not at
ACK-GATE-PASS time), so late retransmits for the now-prev batch must land
somewhere that doesn't clobber the in-flight current batch.

Per CLAUDE.md §"Cross-Layer Data-Flow Audits", this document is the
canonical owner of producer/consumer/invariant facts for this state.

---

## §1 Producers — code paths that write to `messages_rx_prev[]`

Declaration: `mercury/include/datalink_layer/arq.h` — pointer member of
`cl_arq_controller`, allocated to `nMessages` slots (default 120, max 255)
at `arq_common.cc:1545`.

### 1.1 Initial state (init_messages_buffers / reset_messages)

`arq_common.cc:1545-1573` — `new st_message[nMessages]`, each slot:
- `length = 0`
- `status = FREE`
- `type = NONE`
- `data = NULL` then allocated to `N_MAX/8 + CANARY_SIZE` bytes
- `batch_seq_id = -1`

Counters initialized at `arq_common.cc:206-209` and re-zeroed in
`reset_session_state` at `arq_common.cc:1724-1725`:
- `rsp_prev_batch_received_count = 0`
- `rsp_prev_batch_expected_count = 0`

### 1.2 Step 8a transfer at bsi-bump (`bump_bsi_and_transfer_prev`)

`arq_common.cc:4085-4103` — when current batch N is sealed, its
`messages_rx[i]` slots are TRANSFERRED (not copied) into
`messages_rx_prev[i]` and the source slots are freed. Loop bound is
`i<data_batch_size && i<nMessages` (line 4085) — safe by construction
because the slots being transferred have already been validated by the
`add_message_rx_data` bound check (`arq_responder.cc:54`).

Sets:
- `rsp_prev_batch_seq_id = rsp_current_expected_batch_seq_id`
- `rsp_prev_batch_active = true`
- `rsp_prev_batch_received_count = xferred_received` (post-transfer
  RECEIVED-count snapshot)
- `rsp_prev_batch_expected_count = prev_expected` (computed as EOB+1 or
  `data_batch_size`, then clamped to `[1, nMessages]` at line 4072)

**Note on the clamp at 4072**: `prev_expected` can be up to `nMessages`
(255) in theory, but in practice it's always `≤ data_batch_size` because
`last_received_end_of_batch_seq` is set from a frame ID that was
validated to be `< data_batch_size` by `add_message_rx_data:54`. So
`expected_count` is **effectively** capped at `data_batch_size`.

### 1.3 Live retransmit storage path (Step 8a — match-prev branch)

`arq_responder.cc:484-507` — when an incoming OFDM DATA frame's
`batch_seq_id` matches `rsp_prev_batch_seq_id` and `rsp_prev_batch_active`
is true, the frame is stored into `messages_rx_prev[loc]` where
`loc = (unsigned char)messages_rx_buffer.id`.

**This is the R7 producer.** Pre-R7 the bound was `loc < nMessages`
(255). R7 tightens it to `loc < data_batch_size` (typically 25-30).

On a fresh write (prev_status was neither RECEIVED nor ACKED), bumps
`rsp_prev_batch_received_count++` at line 507.

### 1.4 Slot-free paths

- `arq_common.cc:4058-4059` — `rsp_prev_batch_active` true at bsi-bump
  time: clear stale slots back to FREE before re-use.
- `arq_common.cc:4108-4110` — defensive scan over `[data_batch_size,
  nMessages)` post-transfer to ensure those slots are FREE.
- `arq_responder.cc:562-563` — post-delivery cleanup: all `nMessages`
  slots back to FREE after `copy_data_to_buffer()` returns.

### 1.5 Test-mode synthetic-fire producer

`arq_responder.cc:2757-2771` (inside `test_partial_bsi_advance`) —
synthetic injection path used by `--test-partial-bsi-advance=ofdm`. Bound
is `loc >= 0 && loc < this->nMessages`. This path is benign because the
test never sets `loc >= data_batch_size` (it's driven by `arrivals[k].seq`
which the test constructs to be in the valid range).

---

## §2 Consumers — code paths that read `messages_rx_prev[]` or counters

### 2.1 Prev-batch completion gate (the primary consumer)

`arq_responder.cc:526-573` — when
`rsp_prev_batch_active && rsp_prev_batch_received_count >=
rsp_prev_batch_expected_count`, swaps `messages_rx = messages_rx_prev`,
marks all RECEIVED slots ACKED (loop bound `i<data_batch_size &&
i<nMessages` at line 550), calls `copy_data_to_buffer()` (which iterates
ACKED slots up to `data_batch_size`), then restores the pointer and
clears slot status to FREE.

This is the gate the R7 bug used to fire prematurely.

### 2.2 Canary check

`arq_common.cc:1448-1453` — periodic memory-corruption canary scan over
all `nMessages` slots. Read-only.

### 2.3 Diagnostic prints (read counters, not slots)

Multiple sites print `rsp_prev_batch_received_count` and
`rsp_prev_batch_expected_count` for telemetry. Read-only, no effect on
state machine.

### 2.4 Test-mode synthetic-fire consumer

`arq_responder.cc:2778-2788` — same completion-gate pattern as 2.1, used
only by `test_partial_bsi_advance`. Logs `[RSP-V2-PREV-DELIVER-BEGIN]`
and deactivates prev — does NOT call `copy_data_to_buffer` because the
test injects synthetic IDs only.

### 2.5 Destructor

`arq_common.cc:1708-1719` — frees the per-slot `data` arrays and the
outer pointer.

---

## §3 Valid states of `messages_rx_prev[]` + counters

| state | rsp_prev_batch_active | received_count | expected_count | slots with status≠FREE | valid? |
|---|---|---|---|---|---|
| empty (default / post-reset / post-delivery) | false | 0 | 0 | 0 | ✓ |
| transferred-not-yet-complete | true | 0..expected | data_batch_size or EOB+1 | received_count slots RECEIVED, in `[0, data_batch_size)` | ✓ |
| live-receiving (Step 8a writes) | true | grows toward expected | unchanged from transfer | grows; all in `[0, data_batch_size)` after R7 | ✓ |
| **prematurely-complete (PRE-R7 bug)** | true | ≥ expected via writes from bit-errored `loc ≥ data_batch_size` | unchanged | slots both in valid range AND in `[data_batch_size, nMessages)` (the bug's ghost slots) | **INVALID — R7 bug class** |
| stale-on-second-bump | true | (any) | (any) | (any) | Bug-handled at `arq_common.cc:4047-4060` (`[RSP-V2-PREV-STALE]` discard) |

**The R7 invalid state in detail**: a bit-errored DATA frame ID byte that
decodes to a value `v ∈ [data_batch_size, nMessages)` (e.g. 30..119
typical) used to (a) write `messages_rx_prev[v]` to a slot the delivery
loop never reads (line 550 iterates `i<data_batch_size`) AND (b) bump
`rsp_prev_batch_received_count` at line 507. With `expected_count ≤
data_batch_size`, even one bit-errored ID could close the gate before
the real frames arrived. The actual data in the bit-errored frame is
likely garbage (LDPC may have flagged it elsewhere, but the ID byte
itself is unchecked at this layer). Result: partial-batch delivery to
the upper layer, downstream PPMd desync, streaming reset cascade.

---

## §4 Invariants the consumers assume

### 4.1 Completion-gate invariants (consumer §2.1)

**INV-CONS-1**: Every slot in `messages_rx_prev[0..data_batch_size)`
that has `status == RECEIVED` represents a real frame the producer
intended to land there. The delivery loop at line 550 transmutes
RECEIVED → ACKED for exactly these slots; `copy_data_to_buffer` then
reads only ACKED slots. A spurious RECEIVED inside `[0, data_batch_size)`
delivers garbage to the application.

**INV-CONS-2**: `rsp_prev_batch_received_count` is the count of
RECEIVED slots in `[0, data_batch_size)`. The gate fires when this
matches `rsp_prev_batch_expected_count` (also bounded by
`data_batch_size`). If the count is bumped by writes to slots
**outside** `[0, data_batch_size)`, the gate fires before the real
frames have arrived. **This is the R7 invariant the pre-fix producer
broke.**

**INV-CONS-3**: After delivery, all `nMessages` slots are back to FREE
(line 562). The next bsi-bump transfer at §1.2 / §1.4 relies on this so
that the defensive "FREE outside [0, data_batch_size)" scan at
`arq_common.cc:4108` is a no-op fast path.

### 4.2 Transfer-path invariants (producer §1.2)

**INV-PROD-1**: All slots written by the transfer loop are at indices
`< data_batch_size` (loop bound at `arq_common.cc:4085`). Slots in
`[data_batch_size, nMessages)` are not touched on transfer; they must
have been FREE already (defensive re-FREE at 4108).

**INV-PROD-2**: `rsp_prev_batch_expected_count` is set from
`prev_expected` which is `min(data_batch_size, last_received_eob+1,
nMessages)`. **Practically** this equals `data_batch_size` or smaller —
the `nMessages` clamp at line 4072 is dead code given that EOB is set
from a validated frame ID.

### 4.3 Live-write invariants (producer §1.3, post-R7)

**INV-PROD-3 (R7)**: An incoming DATA frame is stored into
`messages_rx_prev[loc]` ONLY if `loc ∈ [0, data_batch_size)`. Bit-errored
IDs in `[data_batch_size, nMessages)` are rejected (logged as
`[RSP-V2-PREV-DROP] reason=length_or_loc_out_of_range`) and do NOT bump
the received_count.

Pre-R7 this invariant was missing: the bound was `loc < nMessages`. The
new-data path at `arq_responder.cc:54` always had the right bound;
prev-batch path was the asymmetric one.

---

## §5 The R7 fix — what changed and why

### Bug

`arq_responder.cc:484` (pre-fix) wrote to `messages_rx_prev[loc]` when
`loc < nMessages` (up to 255), but the completion gate at
`arq_responder.cc:527` fires when
`rsp_prev_batch_received_count >= rsp_prev_batch_expected_count`, and
`expected_count` is effectively bounded at `data_batch_size` (§4.2
INV-PROD-2). A bit-errored ID byte decoding to `v ∈
[data_batch_size, nMessages)`:

1. Wrote payload to `messages_rx_prev[v]` — a slot the delivery loop at
   line 550 never reads.
2. Bumped `rsp_prev_batch_received_count++` at line 507.
3. Tripped the gate before real frames arrived → premature batch
   completion → partial delivery to application → downstream PPMd
   desync → streaming reset.

Severity: HIGH. Estimated 5-15% SACK throughput loss at WGN<20 per
mfsk-vara-parity-plan.md §3 R7.

### Fix (single-layer, producer-side)

Per CLAUDE.md §"Cross-Layer Data-Flow Audits" rule "constrain the
producer," added bound at `arq_responder.cc:484`:

```cpp
// R7 fix (data-flow-messages_rx_prev.md §5):
// Constrain loc to [0, data_batch_size) — NOT [0, nMessages).
if(loc < 0 || loc >= this->data_batch_size)   len_ok = false;
if(loc >= this->nMessages)                    len_ok = false;
```

The first check is the actual fix; the second is preserved as
defense-in-depth in case `data_batch_size > nMessages` ever becomes
possible (it shouldn't, but cheap to keep).

Matches the symmetric bound at `arq_responder.cc:54`
(`add_message_rx_data`, the new-data path). Bit-errored IDs are now
silently rejected; consumer invariants INV-CONS-1, INV-CONS-2 are
preserved.

### Why not fix the consumer

The consumer's iteration at line 550 (`i<data_batch_size &&
i<nMessages`) is already correct — it would just ignore the ghost
slots. The bug is purely the counter-bump at line 507. Fixing only the
counter (e.g. by gating the bump on `loc < data_batch_size`) would
still let bit-errored writes corrupt slots outside the delivery range
— wasted work + potential canary tripwire. Producer-side bound is the
clean fix.

### What about §1.5 (test-mode producer)?

The test path at `arq_responder.cc:2757-2771` is bounded by `loc <
nMessages` only. It's benign because the test driver controls `arrivals[k].seq`
and never injects out-of-range IDs. Not changed by R7. If a future test
adds bit-error injection, this bound should be tightened too.

---

## §6 Cross-layer audit checklist (for future changes to this state)

Before changing any of:

- `messages_rx_prev[]` lifecycle (alloc / free / reset paths)
- The Step 8a routing decision at `arq_responder.cc:383-453`
- The match-prev storage path at `arq_responder.cc:459-583`
- The bsi-bump transfer at `arq_common.cc:4038-4124`
  (`bump_bsi_and_transfer_prev`)
- `rsp_prev_batch_received_count` / `rsp_prev_batch_expected_count`
  semantics
- `data_batch_size` (Axis 2 dynamic resize, gearshift)
- `nMessages` allocation

Walk through producers (§1), consumers (§2), valid states (§3), and
invariants (§4). Update this document with any new producer/consumer/
invariant introduced.

**Special vigilance** when `data_batch_size` changes at runtime (Axis 2):
the gate's invariant relies on `expected_count` being bounded by
`data_batch_size`, but if `data_batch_size` decreases mid-batch (it
shouldn't, but verify), a prior-stored slot in the now-out-of-range
zone could become stranded.

---

## §7 Regression test coverage

### Existing tests

- `--test-partial-bsi-advance=ofdm` at
  `arq_responder.cc:2700+` (function `test_partial_bsi_advance`) —
  exercises §1.5 / §2.4 synthetic-fire path. Does NOT inject bit-errored
  IDs.

### Recommended regression test for R7 (NOT yet implemented)

**Shape**: in-process synthetic-fire test
`test_messages_rx_prev_bit_errored_id` — runs purely in cl_arq_controller
with no DSP. Setup:

1. Initialize controller, set `sack_v2_enabled=true`, `data_batch_size=25`,
   `nMessages=255`.
2. Call `bump_bsi_and_transfer_prev()` after seeding `messages_rx[0..23]`
   with 24 RECEIVED slots → `rsp_prev_batch_expected_count = 25`,
   `received_count = 24`.
3. Inject ONE simulated DATA frame at `process_messages_rx_data_control`
   entry with `match_prev=true` and `messages_rx_buffer.id = 100`
   (bit-errored, ∈ [25, 255)).
4. **Assert (PRE-R7)**: `rsp_prev_batch_received_count` would have been
   bumped to 25, completion gate would have fired, `copy_data_to_buffer`
   called with `messages_rx_prev[0..23]` filled + `messages_rx_prev[100]`
   stranded. Premature delivery.
5. **Assert (POST-R7)**: `rsp_prev_batch_received_count` stays 24,
   `rsp_prev_batch_active` still true, `[RSP-V2-PREV-DROP] reason=
   length_or_loc_out_of_range` logged. Gate does NOT fire.

Test lives in `source/datalink_layer/arq_responder.cc` (synthetic-fire
entry point following the `test_partial_bsi_advance` pattern) or as a
wrapper in `tools/test_*.py`.

### Loopback test (optional, broader coverage)

`tools/sack_lossy_ab.py` with `--enable-sack-v2` already exercises the
prev-batch path at lossy SNRs. Pre-R7 vs post-R7 throughput at WGN:18 /
WGN:20 should show the 5-15% recovery cited in
mfsk-vara-parity-plan.md §3 R7.

---

## §8 Open questions

- **[?]** Is the §1.5 test-mode producer ever reachable from
  non-synthetic-fire paths? If a future production code path exposes
  `nMessages`-bounded loc, R7's coverage is incomplete.
- **[?]** What's the empirical bit-error rate on the DATA frame's ID
  byte at WGN:18? The R7 throughput-recovery estimate of 5-15% assumes
  ~one bit-errored ID per batch; needs measurement.
- **[?]** Axis 2 dynamic `data_batch_size` resize: does any code path
  shrink `data_batch_size` mid-batch with stored slots in the
  now-truncated range? Quick check:
  `grep -n 'data_batch_size *=' source/datalink_layer/*.cc`. If yes,
  add a reset of `rsp_prev_batch_received_count` after rescan.
- **[?]** Does `prev_expected` at `arq_common.cc:4072` ever actually
  reach `nMessages` (the dead-code clamp)? If never, simplify to
  `min(data_batch_size, eob+1)` and drop the clamp.

---

## §9 Related fact documents

- `data-flow-retx-queue.md` — TX-side retx queue (sibling state; same
  cross-layer-audit principle). The R7 bug is the RX-side analogue of
  the "Bug 2: Padded-slot retx capture" pattern: producer wrote ghost
  entries that bypassed consumer assumptions.
- `data-flow-optimizer.md` — Axis 2 controller (consumer of
  `data_batch_size`; its dynamic-resize behavior is referenced in §6
  vigilance note and §8 open question).

---

## §10 CLIMB C1 + C2 — prev-path delivery confirmation + robust batch=1

**Status**: shipped on `fix/climb-combined` (worktree off `monitor` 3b1726a),
2026-05-30. Paired regression: `mercury.exe --test-climb-combined` (C1.1–C1.11 +
C2a–C2f, all-inline replay of the production dedup + batch-recompute decisions;
fail-before verified by reverting both predicates to their pre-fix form →
C1.5/C1.7/C1.8 + C2a/C2b/C2c FAIL).

### The disease (the "climb stall")

At a marginal rung a batch loses ≥1 frame → RSP sends a PARTIAL SACK → CMD sets
`last_batch_fully_acked=false` (correct: §9 CLEAN-BATCH VIABILITY — a partial
must NOT promote) and retransmits the gaps. The retransmitted frames carry the
ORIGINAL bsi (`arq_commander.cc:1207/1466`; `cmd_batch_seq_id` only advances on a
new-data batch, `:1722`), so on the RSP they match `rsp_prev_batch_seq_id` and
route to `messages_rx_prev[]` (§1.3). When they complete the prev batch, the
completion gate (§2.1) delivers it LOCALLY and logs `[RSP-V2-PREV-DELIVERED]`.

**Pre-C1 the prev path emitted NOTHING back.** So the batch that just FULLY
delivered never produced a clean confirmation on the CMD — `last_batch_fully_acked`
stayed false forever, `nBatches_fully_acked` never incremented, and the rung
never satisfied the promotion gate (`promotion_allowed_on_batch`,
`arq_commander.cc:3427/3498`; success-rate_data_clean, `:4555-4557`). The link
stayed alive (keepalive) but could not climb. This is distinct from — and
downstream of — the §9 partial-must-not-promote rule: §9 correctly blocks the
partial; C1 supplies the MISSING clean signal once the retransmit finishes.

### C1 fix — two halves

**Producer (new, RSP side)** — `arq_responder.cc`, inside the
`[RSP-V2-PREV-DELIVERED]` block (after the prev-batch completion gate §2.1):
emit an MFSK ACK+SACK with `bsi = rsp_prev_batch_seq_id` and an all-ones bitmap
sized from the CURRENT `data_batch_size`. This mirrors the clean-batch ACK
transport at the ACK-GATE (`arq_responder.cc:1644-1690`). WB-only
(`ack_sack_suffix_len() > 0`); the dedicated `ack_mfsk` codec is config/mode-
independent (`telecom_system.cc:3063`) so no `set_mfsk_ctrl_mode()` toggle is
needed. NB (suffix_len==0) keeps batch=1 (C2) and never routes a multi-frame
retransmit through the prev path, so the bare-pattern fallback is intentionally
NOT used (it would be ambiguous with the current-batch ACK detector).

**Consumer (CMD side)** — the existing MFSK-ACK-SACK handler
(`arq_commander.cc:2496-2533`) decodes the all-ones bitmap → classifies CLEAN →
sets the per-poll `v2_ack_pat_pre_detected` local → funnels through the clean-ACK
handler (`:2929-2956`) which sets `last_batch_fully_acked=true` and bumps
`nBatches_fully_acked` (the SAME producer set as a first-pass clean batch).
`promotion_allowed_on_batch` and the FRAME-UP +1 anchor clamp (`:3480-3487`)
are UNCHANGED — C1 only supplies the input they were already waiting for.

**The dedup hazard C1 had to fix** — the handler keyed duplicate suppression on
a single `cmd_last_applied_sack_bsi` (`:2509`). The original partial SACK set it
to the prev bsi (`:2545`), so the all-ones confirmation for the SAME bsi was
dropped as a "duplicate" and the fix would have been inert. C1 splits the dedup
by event class: a CLEAN (all-ones) confirmation is exempt from the PARTIAL
tracker and deduped only against a NEW member `cmd_last_applied_clean_bsi`
(`arq.h`, init -1 in the ctor, `arq_common.cc`). Rationale: a partial SACK is a
RETRANSMIT decision (re-applying re-queues nothing → suppress repeats); a clean
confirmation is a TERMINAL batch-complete STATE TRANSITION that supersedes the
partial. The clean tracker still dedups a REPEATED clean (RSP re-emit / ring
echo) so `nBatches_fully_acked` cannot double-count (test C1.9/C1.10).

### C2 fix — robust keeps batch=1

`set_configuration` installs `data_batch_size=1` for ROBUST_0/1/2
(`arq_common.cc:1229-1234`), and the OFDM batch-scaling at `:1269` is itself
gated `!is_robust_config` for the same reason. But the SACK-NEGOTIATION batch
recompute (CMD `arq_commander.cc` TEST_CONNECTION handler; RSP `arq_responder.cc`
`process_control_responder`) re-ran the 30 s formula UNCONDITIONALLY with a hard
`if(max_batch<5) max_batch=5` floor — clobbering robust's batch=1 up to 5+ the
moment SACK negotiated. At the MFSK cliff P(batch)=p^N, so a 5-frame batch turns
one bad frame into a whole-batch loss and the link cannot climb off ROBUST_0.
C2 gates the WHOLE recompute on `!is_robust_config(current_configuration)` on
BOTH sides (the symmetry is mandatory — an asymmetric override was the original
Bug #9 "RSP batch size not updated after SACK negotiation"). OFDM behavior is
byte-for-byte unchanged (test C2d/C2e/C2f).

### §2.6 New consumer of the prev-batch counters/seq (C1 producer)

`arq_responder.cc` `[RSP-V2-PREV-DELIVERED]` block — reads `rsp_prev_batch_seq_id`
(still valid; only `_active`/`_received_count`/`_expected_count` are zeroed at the
top of the block, NOT `_seq_id`) to address the all-ones confirmation. Read-only
w.r.t. `messages_rx_prev[]` slots (those are already FREE by this point, §2.1).

### §4.4 New invariant

**INV-C1-1**: the all-ones confirmation bitmap is sized from the CURRENT
`data_batch_size`, which MUST equal the CMD's `data_batch_size` at decode time
(it does — CMD/RSP carry the same negotiated value, and C2 keeps robust=1 on
both sides). If they ever diverged, the CMD's `all_ones` (`:2513`) would not
match `rx_bitmap`, the confirmation would be classified PARTIAL or rejected, and
the batch would simply not promote (graceful degradation to the pre-C1 behavior —
no corruption, no double-delivery).

**INV-C1-2**: the prev-path confirmation TX is safe mid-DATA-processing because
the prev batch only completes via a retransmit-only batch (the "one outstanding
batch" rule, `arq_responder.cc` tail comment near the prev block), so no
concurrent new-data frames are clipped by the confirmation's `rx_mute` flush
(the same flush every ACK TX performs).

### §6 addendum — new vigilance

Add to the §6 checklist: the `[RSP-V2-PREV-DELIVERED]` confirmation TX (C1
producer) and the CMD clean/partial dedup split (`cmd_last_applied_clean_bsi` vs
`cmd_last_applied_sack_bsi`). Any change to batch-size negotiation must preserve
CMD/RSP symmetry (C2) or the all-ones sizing breaks (INV-C1-1).
- `mfsk-vara-parity-plan.md` §3 R7 — the bug report that drove this fix.

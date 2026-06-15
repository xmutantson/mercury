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

**Note on the clamp** (line numbers now `arq_common.cc:6447-6454` on
monitor `8bf97d9`; the `4085-4103` / `4072` refs above predate the §6g
hoist): `prev_expected` can be up to `nMessages` (255) in theory, but in
practice it's always `≤ data_batch_size` because `last_received_end_of_batch_seq`
is set from a frame ID that was validated to be `< data_batch_size`. So
`expected_count` is **effectively** capped at `data_batch_size`.

> ~~`prev_expected` is always the TRUE batch length.~~ **CORRECTED
> 2026-06-15 (D5):** `prev_expected` can be inferred STRICTLY SHORTER
> than the TX's true frame count when the EOB (tail) frame of the bumped
> batch is LOST. Then `last_received_end_of_batch_seq` reflects a LOWER
> index (a stale/aliased earlier EOB), so `prev_expected < data_batch_size`
> while a genuinely-sent-but-lost tail frame occupies a FREE slot in
> `[prev_expected, data_batch_size)`. The SET-gate (§2.1) checks only
> `[0, expected_count)`, passes, and the tail frame is silently dropped.
> This is **D5** — see §4.5 (INV-D5) and `PREV_BUMP_VERDICT.md` §2.

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

### 4.4 FIX-A — the prev-path is now LIVE at the ROBUST tier (2026-06-03)

**Update (FIX-A, branch `fix/robust-dwell-batch`)**: before FIX-A the entire
SACK partial / prev-batch machinery was **DEAD at the ROBUST tier** because the
robust data batch was force-pinned to 1 (`set_data_batch_size()` chokepoint), and
the partial branch (`arq_responder.cc:1462` `if(data_batch_size > 1 …)`) plus the
`<=1` SACK-suppression short-circuit (`:1504`) meant a single-frame robust batch
never reached `bump_bsi_and_transfer_prev()` / the prev-delivery loop. FIX-A lifts
the robust batch to `[1..ROBUST_DWELL_BATCH_MAX(8)]` on a PROVEN+PARKED dwell
(`robust_dwell_batch_eligible()`, `arq.h`), so at a robust dwell with batch 4-8 the
prev-path becomes EXERCISED at ROBUST_0 WB.

**No new invariant is broken** — every invariant in §4 is `data_batch_size`-
parametric and `messages_rx_prev[]` is sized to `nMessages` (default 120, §1.1), so
4-8 robust slots fit trivially (the path was already validated for OFDM batch ≥ 25).
The one thing the FIX-A audit confirms: the M=16 `ack_mfsk` SACK suffix is
config-independent (`telecom_system.cc:3095`) so `ack_sack_suffix_len()==13` at
ROBUST_0 WB — the bitmap that drives the prev/partial path exists. On **NB** robust
(`ack_sack_suffix_len()==0`) there is no bitmap, so the FIX-A gate keeps NB robust
at batch=1 (conjunct (b)) and the prev-path stays dead there — unchanged.
See `data-flow-robust-tier-arq-batch.md` §3.7 / L1.

### 4.5 INV-D5 — the EOB-short-inference truncation (the lost-EOB tail-drop)

**Defect (D5, `PREV_BUMP_VERDICT.md` §2; byte-attributed):** the producer
`bump_bsi_and_transfer_prev()` (§1.2) infers
`prev_expected = min(data_batch_size, last_received_end_of_batch_seq+1)`.
When the EOB (tail) frame of the bumped batch is **LOST**,
`last_received_end_of_batch_seq` reflects a LOWER index, so `prev_expected`
is set SHORTER than the TX's true `data_batch_size`. The §2.1 SET-gate
(`rsp_prev_batch_received_count >= rsp_prev_batch_expected_count`,
`arq_responder.cc:916`/`:3653`) checks only `[0, expected_count)` — which
IS complete — so it PASSES, and the genuinely-missing tail frame in
`[expected_count, data_batch_size)` (a FREE slot) is **silently dropped**.
Trace: `cum_before=31113 … expected_prev=29 statuses{ACK=29 FREE=1}` — the
FREE slot is exactly index 29 (the tail); TX sent 30. A one-frame ~155-byte
silent skip per truncated batch, accumulating across the climb (the >26KB
md5 divergence). The gate cannot catch it: `expected_count` ITSELF is the
corrupted value (consumer §2.1 trusts a producer invariant the lost-EOB
path violates — the canonical CLAUDE.md §5 failure mode).

**Why no false-positive on a legitimate adaptive-short batch:** the bump is
only reached on the PARTIAL-SACK path (`rx_received < expected`,
`arq_responder.cc:1801`). A legitimately-short batch whose **received** EOB
sets `expected = EOB+1` has `rx_received == expected` (all frames up to the
received EOB present) → it ACK-GATE-PASSes COMPLETE and **never reaches the
bump**. So at the bump, a FREE slot at/after the inferred EOB is a frame the
inference declared "not in the batch" that the TX may genuinely have sent
and lost. (This corrects `data-flow-prev-bump.md` §5.6, which ruled "no D5"
by considering only a RECEIVED tail — the LOST-EOB tail IS the defect.)

**INTERIM PRODUCER-SIDE GUARD (Phase 0.2, branch `feat/delivery-layer-phase01`,
`arq_common.cc` `bump_bsi_and_transfer_prev()`):** when
`prev_expected < data_batch_size` AND a FREE slot exists in
`[prev_expected, data_batch_size)`, raise the existing `rsp_gap_abort_teardown()`
([RSP-V2-GAP-ABORT], DROPPED, bsi family + prev buffer cleared) INSTEAD of
arming the short prev. Converts the silent-wrong-bytes skip into a loud
correct-prefix + teardown (integrity-sound). **DEFAULT-OFF** (env
`MERCURY_D5_LOUD_ABORT`, `d5_loud_abort_enabled_common()`); unset → BYTE-IDENTICAL
to monitor `8bf97d9`. This is `PREV_BUMP_VERDICT.md` §4 **direction 2** (the
conservative safety net): more aborts, never a silent drop. The byte-faithful
**root cure** is the TX-authoritative wire-carried per-batch frame count
(direction 1 = A4, architectural, Phase 3) — set `expected_count` from the
wire, not the inference; a lost EOB then leaves `received < expected` so the
SET-gate HOLDS and the tail is recovered via retransmit.

**Producer change vs consumers:** the fix constrains the PRODUCER (refuse the
ambiguous short-inference) and touches NO consumer; the §2.1 SET-gate is
unchanged. Regression: `--test-d5-lost-eob` (fail-before = silent short-seal;
pass-after = loud GAP-ABORT; ARM3 no-false-positive on a complete batch).

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
- `--test-d5-lost-eob` (`test_d5_lost_eob_abort`, `arq_responder.cc`) —
  the §4.5 INV-D5 regression. Drives the REAL `bump_bsi_and_transfer_prev()`
  with a LOST-EOB batch (`prev_expected`=29 inferred, FREE tail slot 29 in
  a 30-frame batch). ARM1 fail-before (`MERCURY_D5_LOUD_ABORT` unset) =
  silent short-seal; ARM2 pass-after (=1) = loud `[RSP-V2-GAP-ABORT]`,
  DROPPED, no short prev armed; ARM3 (both modes) = no false-positive on a
  complete batch.

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
- `mfsk-vara-parity-plan.md` §3 R7 — the bug report that drove this fix.

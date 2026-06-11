# Data-Flow Audit: PREV-BUMP cross-storage IN-ORDER delivery invariant

**Status**: Authoritative as of 2026-06-11. **FIX LANDED** on the worktree
`C:/Users/kamer/mercury_wt/cfg16-acq-d2d3` @ `feat/prev-bump-fix` (branched from
`feat/cfg16-acq-d2d3 @ 05fb9b7` = monitor + D1 acq-fix + H1 reverse-ACK + D2/D3, the
binary that REACHES CFG16 and so EXPOSES this defect). This is the mandatory
CLAUDE.md §"Cross-Layer Data-Flow Audits" gate for the **D4** silent-wrong-bytes leak
the D2/D3 e2e runs hit (`bigblock_p3_hw/_cfg16acqd2d3/CFG16_ACQ_D2D3_VERDICT.md` §3,
executed `result.json` md5_match:FALSE on 3/4 FIX runs). Every change to the structures
in §1 MUST update this document.

**FIX SUMMARY (this session):** two coupled changes (§5.2), failing-test-first via the
`MERCURY_PREVBUMP_DEFEAT` defeat env on the SAME binary (the codebase's established
`MERCURY_GAP_ABORT_DEFEAT`/`MERCURY_LOSSY_DEMOTE` pattern):
1. **L1** — the prev completion gate (`arq_responder.cc:786`) is now a SET check:
   `... && (prevhole_defeat || prev_batch_is_frame_complete())`. The new pure helper
   (`arq_common.cc cl_arq_controller::prev_batch_is_frame_complete()`, declared
   `arq.h`) returns true IFF every slot in `[0, expected_count)` of
   `messages_rx_prev[]` is RECEIVED-or-ACKED. A tail slot in
   `[expected_count, data_batch_size)` can no longer cover a missing low slot; the
   holed batch is HELD (faithful when the SACK retransmit completes the set) instead
   of delivered with a silent byte jump. BYTE-IDENTICAL on the faithful path.
2. **L2** — `rescan_prev_on_batch_shrink()` (`arq_common.cc`) now routes an
   `orphaned_received>0` shrink through the SAME loud `rsp_gap_abort_teardown()`
   (`[RSP-V2-GAP-ABORT]`, DROPPED) the batch-level D3.1 gate uses, instead of the
   silent re-derive + truncated delivery. This extends D3.1's byte-faithful-OR-loud-
   abort promise symmetrically to the within-batch frame-drop case.
   Regression: `--test-prevbump-frame-hole` (new, L1/L2/L1-CLEAN, fail-before via
   `MERCURY_PREVBUMP_DEFEAT=1`); `--test-batch-shrink-strands-prev` (R035) updated to
   the new L2 invariant (default loud-abort; defeat preserves pre-fix re-derive).
   Full integrity battery green (§5.5). **No D5 sibling** (the tail-slot delivery is
   bsi-gated same-batch in-order data, §5.3).

**Scope split vs the existing docs (no duplication):**
- `data-flow-messages_rx_prev.md` owns the **per-slot storage lifecycle** of
  `messages_rx_prev[]` (alloc/free/reset, the R7 bit-errored-`loc` bound). It already
  flags the `data_batch_size`-shrink hazard as UNVERIFIED (§6/§8). It does NOT own the
  delivery-time in-order invariant.
- `data-flow-arq-recovery-cluster.md` owns the R035 batch-shrink rescan and the EOB
  poison (R038) as race-class fixes. It addressed the **streaming-desync** symptom of a
  shrink, NOT the **silent-byte-hole** symptom.
- **THIS doc owns the cross-storage PREV-BUMP → delivery in-order invariant**: the gate
  at `arq_responder.cc:786`, the swap-deliver loop at `:888`, `copy_data_to_buffer()`
  (`arq_common.cc:9690`), and how the D3.1 batch-level contiguity gate
  (`delivery_step_is_gap`, `arq.h:1470`) does and does NOT cover them. The residual
  hole (§4 / §5) is a WITHIN-batch frame hole that the batch-level gate cannot see.

**All file:line below were RE-LOCATED against the live `feat/prev-bump-fix` (==05fb9b7)
tree this session and verified by executing the e2e logs.** Where they differ from the
companion docs (which drifted), this doc wins for the delivery-path lines.

---

## §0 BLUF — the residual hole in one paragraph

The cross-storage prev-batch is delivered when its RECEIVED-slot **COUNT** reaches
`expected_count` (`arq_responder.cc:786-787`:
`rsp_prev_batch_received_count >= rsp_prev_batch_expected_count`). The delivery loop
(`arq_responder.cc:888`) and `copy_data_to_buffer()` (`arq_common.cc:9712-9729`) then
iterate `[0, data_batch_size)`, **concatenating ACKED slots in index order and SILENTLY
skipping any non-ACKED hole** — with no per-slot completeness check. The D3.1 integrity
capstone (`delivery_step_is_gap`, `arq.h:1470`) only checks **batch-level** bsi
contiguity (`last_delivered → bsi`), so it accepts this delivery (bsi IS contiguous) and
NEVER aborts. Two producer realizations let the COUNT reach `expected_count` while a slot
**inside** the delivered range is still FREE → a within-batch frame HOLE → silent-wrong-
bytes, NO `[RSP-V2-GAP-ABORT]`: **(L1)** EOB-derived `expected_count < data_batch_size`,
so RECEIVED slots in `[expected_count, data_batch_size)` count toward the gate while a
low-index slot stays FREE; **(L2)** the R035 reshrink lowers `expected_count`/
`received_count` to the new bound and drops an orphaned RECEIVED frame, delivering the
shorter batch as "complete." Both were executed in the e2e logs (§4.4).

---

## §1 The cross-storage state — declarations

All members of `cl_arq_controller` (`include/datalink_layer/arq.h`), RX/responder side:

```
arq.h  int  rsp_current_expected_batch_seq_id;   // current bsi window (-1 = unset/re-baseline)
arq.h  int  rsp_prev_batch_seq_id;               // the sealed prev bsi (-1 = none)
arq.h  bool rsp_prev_batch_active;               // prev cross-storage live
arq.h  int  rsp_prev_batch_received_count;       // # RECEIVED slots counted toward the gate
arq.h  int  rsp_prev_batch_expected_count;       // FROZEN at PREV-BUMP from min(data_batch_size, eob+1)
arq.h  int  rsp_last_delivered_batch_seq_id;     // FIX-8 reset-surviving delivery high-water (-1 = none)
```

`messages_rx_prev` = pointer member, `nMessages` slots (default 120). Full slot
lifecycle owned by `data-flow-messages_rx_prev.md` §1; not duplicated here.

`data_batch_size` (`arq.h`, default 1, sole OFDM setter `set_data_batch_size()` at
`arq_common.cc`) is the DELIVERY-LOOP bound; `last_received_end_of_batch_seq`
(`arq.h`, -1=none) feeds `expected_count`. These two diverging is the L1 enabler.

The pure batch-level contiguity predicate (the D3.1 keystone):
`static bool delivery_step_is_gap(int bsi, int last_delivered_bsi)` —
`arq.h:1470-1477`. It compares ONLY bsi mod-256 forward distance; **it has no slot/
frame argument and cannot see a within-batch hole** (§4.1).

---

## §2 PRODUCERS — every path that writes the prev cross-storage / high-water

### 2.1 Arm the prev cross-storage at bsi-bump (`bump_bsi_and_transfer_prev`)
`arq_common.cc:6355-6441`. When current batch N is sealed:
- Transfer loop `arq_common.cc:6402-6421`: copies `messages_rx[i]` → `messages_rx_prev[i]`
  for `i<data_batch_size`, frees source. `xferred_received` counts EVERY RECEIVED slot in
  `[0, data_batch_size)` (`:6414`) — **including indices ≥ the EOB-derived `prev_expected`**
  (this is the L1 seed: the transfer-count can carry tail slots beyond `expected`).
- `prev_expected` derivation `arq_common.cc:6382-6389`:
  `prev_expected = data_batch_size; if(eob>=0 && eob+1<prev_expected) prev_expected=eob+1;`
  → **`expected_count` can be STRICTLY LESS than `data_batch_size`** (L1 root).
- Sets `rsp_prev_batch_seq_id = old current` (`:6430`), bumps current (`:6431`),
  `rsp_prev_batch_active=true` (`:6433`), `received_count = xferred_received` (`:6434`),
  `expected_count = prev_expected` (`:6435`). Logs `[RSP-V2-PREV-BUMP] … transferred=D
  received_on_transfer=R/E` — the e2e logs show `transferred=30 … expected=25..29`
  (D>E) directly (§4.4).
- Stale-prev guard `arq_common.cc:6364-6377`: if a prev was still active, discards it
  (FREE all slots) but does NOT deliver — handled, see `data-flow-messages_rx_prev.md`.

### 2.2 Live retransmit write into the cross-storage (the count producer)
`arq_responder.cc:719-777` (`v2_route_to_prev` branch). For an OFDM DATA frame whose bsi
matches `rsp_prev_batch_seq_id` and `rsp_prev_batch_active`:
- Bound `arq_responder.cc:744`: `if(loc<0 || loc>=data_batch_size) len_ok=false;` (R7).
  **Note the bound is `data_batch_size`, NOT `expected_count`** — so a retransmit for a
  slot in `[expected_count, data_batch_size)` is ACCEPTED and stored.
- Stores payload (`:754-765`), `status=RECEIVED`.
- `arq_responder.cc:766-767`: `if(prev_status not RECEIVED/ACKED) received_count++;` —
  **bumps the gate counter for ANY index in `[0, data_batch_size)`, including indices
  `>= expected_count`** (L1 mechanism: an out-of-`expected`-range tail slot can satisfy
  the count while a low hole remains).

### 2.3 Reshrink the prev counters on a data_batch_size shrink (R035)
`rescan_prev_on_batch_shrink()` — `arq_common.cc:1029-1072`. Called from the
`set_data_batch_size()` chokepoint on SHRINK while prev active. WRITES:
- `received_count := #RECEIVED in [0,new_batch)` (`:1070`)
- `expected_count := min(old_expected, new_batch)` (`:1071`)
- Fires `streaming_reset()` IF `orphaned_received>0 && is_streaming() && batch_data_delivered`
  (`:1054-1061`). **It does NOT prevent the subsequent prev-delivery and does NOT recover
  the orphaned frame's bytes** (L2 leak — §4.3). Logs `[RSP-V2-PREV-RESHRINK] … received
  R->r expected E->e orphaned_received=K`. e2e: `received 6->5 expected 11->10
  orphaned_received=1` (§4.4).

### 2.4 Deliver + advance high-water on prev completion
`arq_responder.cc:866-917`. After the gate (§3.1) fires and the (optional) big-block
CRC-32 / D3.1 contiguity checks pass:
- Pointer swap `messages_rx = messages_rx_prev` (`:878`).
- Slot promote loop `arq_responder.cc:888-892`: `for(i<data_batch_size) if(RECEIVED)
  status=ACKED;` — **promotes ONLY RECEIVED slots; a FREE hole stays FREE.**
- `copy_data_to_buffer()` (`:893`) — see §3.3.
- `advance_last_delivered(rsp_prev_batch_seq_id)` (`arq_responder.cc:911` →
  `arq_common.cc:6454-6467`): the FIX-8 monotonic-with-wrap high-water producer. **Argument
  is the BSI only** — it records that batch `bsi` was delivered, asserting NOTHING about
  whether that batch's frames were all present.
- `[RSP-V2-PREV-DELIVERED] … last_delivered=bsi` log (`:912-917`).

### 2.5 Advance high-water on the in-place current-batch commit (`BATCH-DONE`)
`arq_responder.cc:1930-1943`. The OTHER `advance_last_delivered` producer (current batch,
not cross-storage), gated behind the SAME batch-level `delivery_step_is_gap` check at
`:1917`. Same blind spot: advances on bsi, asserts nothing about slot completeness.

### 2.6 Re-baseline / teardown producers (clear the cross-storage)
- DEMOTE-REBASE `arq_responder.cc:2797-2816`: on a REAL config change mid-transfer,
  clears `rsp_current_expected_batch_seq_id=-1`, `rsp_prev_batch_*=0/false`, FREEs all
  prev slots, **preserves `rsp_last_delivered_batch_seq_id`** so the next adopt re-checks
  the gap-gate. This correctly DROPS an in-flight prev partial (so the demote path is NOT
  a within-batch-hole producer — verified §4.4: the orphan/L1 leaks fire on the steady
  CFG16 multi-batch path, not at the demote).
- GAP-ABORT teardown `rsp_gap_abort_teardown()` — `arq_common.cc:6474-6507`: the LOUD
  path. Clears the bsi family + prev buffer, `link_status=DROPPED`, `reset_session_state()`.
  This is the ONLY path that refuses a delivery; it fires ONLY on a batch-level bsi skip.

### 2.7 High-water reset producers
`rsp_last_delivered_batch_seq_id` is reset to -1 by `reset_session_state()` and the
re-adopt teardown; set forward only by §2.4/§2.5. (Owned here because the in-order
invariant is the high-water's reason to exist.)

---

## §3 CONSUMERS — every path that reads the prev cross-storage / delivers from it

### 3.1 The prev completion GATE (the primary consumer — and the count-only flaw)
`arq_responder.cc:786-787`:
```
if(rsp_prev_batch_active
   && rsp_prev_batch_received_count >= rsp_prev_batch_expected_count)
```
**This is a CARDINALITY test, not a SET test.** It fires when the NUMBER of RECEIVED
slots reaches `expected_count`; it does NOT verify that the RECEIVED set ==
`{0,1,…,expected_count-1}`. Consumes `received_count`, `expected_count`,
`rsp_prev_batch_active`. This is the gate the L1/L2 leaks trip prematurely.

### 3.2 The big-block CRC-32 gate (partial consumer — narrow coverage only)
`arq_responder.cc:801-829`. If `bigblock_partial_armed && bsi==bigblock_partial_block_bsi`,
re-verifies a whole-block CRC-32 and REJECTS on mismatch (FIX-2). **Only covers an armed
big-block carve** — a normal per-frame batch (`bigblock_partial_armed==false`) passes
through with NO whole-batch integrity check (`:802` condition false → `prev_deliver_ok`
stays true). So per-frame CFG16 multi-batch (the D4 regime) is NOT covered.

### 3.3 The delivery reassembler (`copy_data_to_buffer` — where the hole is dropped)
`arq_common.cc:9690-9897`. Two legs, BOTH silently skip non-ACKED holes:
- Compression leg `arq_common.cc:9712-9729`: `for(i<data_batch_size) if(ACKED)
  memcpy(assembled+=, …) else status=FREE;` — **a FREE/non-ACKED slot at index j in the
  middle is simply skipped**; slots j+1.. are concatenated directly after j-1. No
  completeness check, no gap marker. `assembled` is then decrypted/decompressed and
  pushed to the app FIFO (`fifo_push_rx`, `:9809`) as the complete batch.
- No-compression leg `arq_common.cc:9863-9883`: identical concatenate-ACKED-skip-holes.
- The decompress path commits the streaming PPMd carry on a full store
  (`streaming_commit`, `:9814`) — so a holed delivery ALSO advances the streaming model
  past the missing bytes (the desync that R035's reset tries, partially, to defend).

### 3.4 The D3.1 delivery-time contiguity gate (the inviolable backstop — and its blind spot)
`delivery_step_is_gap()` (`arq.h:1470-1477`), invoked at:
- PREV-DELIVER site `arq_responder.cc:851-865` (BEFORE the §3.3 copy).
- BATCH-DONE site `arq_responder.cc:1917-1929`.
- Re-adopt site `arq_responder.cc:3942` / teardown emits at `:3708,:3945,:3968`.

It returns TRUE only when `bsi` skips `last_delivered` by `>=2` mod-256 — a BATCH-level
hole. **It receives only `(bsi, last_delivered)` — there is no slot vector, so a fully-
contiguous bsi step that delivers a frame-incomplete batch is INVISIBLE to it.** This is
the precise residual hole in the D3.1 "byte-faithful-OR-loud-abort" promise (§4.2).

### 3.5 FIX-8 re-adopt gate (post-reset consumer of the high-water)
`arq_responder.cc` re-adopt block (`sack_v2_readopt_has_gap` → `:3708`/teardown). Reads
`rsp_last_delivered_batch_seq_id` to detect a hole across a session reset. Same batch-
level granularity → same blind spot to within-batch holes already committed before the
reset.

### 3.6 Other consumers (read-only, no delivery effect)
- Canary scan `data-flow-messages_rx_prev.md` §2.2.
- Diagnostic prints of the counters (PREV-RX/BUMP/DELIVERED logs).

---

## §4 VALID STATES + the broken invariant (the residual hole)

### 4.1 What `delivery_step_is_gap` / `advance_last_delivered` actually guarantee
The FIX-8/D3.1 pair guarantees a **BATCH-level in-order delivery**: the sequence of
delivered bsi is monotonic-with-wrap and contiguous, OR the link loud-aborts. It does
NOT, and structurally cannot (no slot argument), guarantee that each delivered batch
contained all its frames. **D3.1's promise is "no dropped BATCH," not "no dropped FRAME."**

### 4.2 The invariant the consumers ASSUME vs the one that actually holds
- **INV-ASSUMED (the D3.1 promise, what §3.3 `copy_data_to_buffer` and the app rely on):**
  *Every batch reaching `copy_data_to_buffer` via the prev gate (§3.1) is byte-complete —
  every slot in `[0, expected_count)` is RECEIVED — so concatenating ACKED slots yields
  the exact in-order prefix; otherwise the link loud-aborts.*
- **INV-ACTUAL (what the code maintains):** *The COUNT of RECEIVED slots in
  `[0, data_batch_size)` is `>= expected_count`, and the delivered bsi is batch-
  contiguous.* The count can be reached by slots OUTSIDE `[0, expected_count)` (L1) or
  after a shrink that orphaned a frame (L2), so INV-ASSUMED is **false** while
  INV-ACTUAL is true → the gate passes, the gap-abort stays silent, and a frame-holed
  batch is delivered as complete.

### 4.3 The residual hole — two producer realizations

**L1 (the dominant, orphaned_received=0 leak): EOB-short `expected_count` + count-only gate.**
- Producer §2.1 sets `expected_count = min(data_batch_size, eob+1)`; under drift/loss the
  RSP can latch an EOB at a seq `< data_batch_size-1` → `expected_count < data_batch_size`.
- Producer §2.2 accepts + counts RECEIVED slots over the FULL `[0, data_batch_size)`
  range (bound is `data_batch_size`, `:744`; count-bump is index-agnostic, `:766-767`).
- Consumer §3.1 fires when `received_count >= expected_count`. Because slots in
  `[expected_count, data_batch_size)` count, the gate can fire with a FREE hole inside
  `[0, expected_count)`.
- Consumer §3.3 concatenates the ACKED slots in index order, silently dropping the hole.
  The decompressor sees a byte-stream missing one frame's plaintext mid-batch → wrong
  bytes for the rest of that batch (and the streaming carry desyncs).
- Consumer §3.4 sees a contiguous bsi → no abort. **Silent-wrong-bytes.**

**L2 (the orphaned_received>0 leak): R035 reshrink drops an in-range RECEIVED frame.**
- Producer §2.3 lowers `expected_count`/`received_count` to the new (smaller) bound and
  counts `orphaned_received` RECEIVED frames now in `[new,old)`. Those frames' bytes are
  unreachable by the delivery loop (`i<data_batch_size=new`).
- It fires `streaming_reset()` but **delivers the shorter batch anyway** as "complete."
  The orphaned frame's bytes are LOST from the byte stream; `streaming_reset` only
  resyncs the PPMd model on the NEXT batch — it does not retroactively make the truncated
  batch faithful. Consumer §3.4 sees a contiguous bsi → no abort. **Silent-wrong-bytes.**

### 4.4 Executed evidence (`bigblock_p3_hw/_cfg16acqd2d3/e2e/*/`, this session)
| FIX run | rx B | md5_match | GAP-ABORT | leak realization (from the log) |
|---|---|---|---|---|
| `fix_s3_300` | 23098 | **TRUE** | 1× (bsi=12 vs last=10) | matched ONLY because a batch-level GAP-ABORT TRUNCATED to a clean prefix + DROPPED the link — D3.1 caught a *batch* hole, not faithful completion |
| `fix_s7_250` | 42008 | **FALSE** | 0× | **L2**: `[RSP-V2-PREV-RESHRINK] 15->10 … received 6->5 expected 11->10 orphaned_received=1` then `PREV-DELIVERED last_delivered=18` — orphaned frame dropped, batch delivered complete |
| `fix_s11_300` | 54098 | **FALSE** | 0× | **L1**: `[RSP-V2-PREV-BUMP] … transferred=30 … expected=29/25/26` (E<D) — count gate fired with tail slots beyond `expected` while low holes filled by id=0..4 retransmits; NO orphan reshrink |
| `diag_s7_long` | 77503 | **FALSE** | 1× (late, bsi=28) | **L1** before the late batch-abort: `transferred=30 … expected=26` deliveries slipped through PREV path BEFORE the bsi-level abort fired |

The two `orphaned_received=0` mismatches (`fix_s11_300`, `diag_s7_long`) prove L1 is a
DISTINCT leak from the R035 orphan (L2) — the count-only gate corrupts even when the
reshrink reports zero orphans. `recv_md5 != md5(tx_prefix[:delivered])` confirms the
delivered bytes are NOT a clean in-order prefix (`sim_arq_channel.py:836`
`_md5_of_looped_prefix`).

### 4.5 State table
| state | active | received_count | expected_count | slot set RECEIVED | delivered? | faithful? |
|---|---|---|---|---|---|---|
| complete-contiguous | true | == E | E (== or < D) | exactly `[0,E)` | yes (gate) | ✓ |
| **L1 holed** | true | == E (via slots ≥E) | E < D | superset of size E w/ a hole `<E` | yes (gate, no abort) | **✗ silent** |
| **L2 orphaned** | true | == e (post-reshrink) | e = new<old | `[0,e)` complete but a frame lost in `[e,old)` | yes (gate, no abort) | **✗ silent (frame dropped)** |
| batch-skip | (n/a) | (any) | (any) | (any) | **NO** — `delivery_step_is_gap` LOUD-aborts | n/a (link dropped) |

---

## §5 The residual leak — exact file:line + minimal fix

### 5.1 The leak site (the missing assert)
The hole is delivered at the **count-only gate** + the **completeness-blind reassembler**.
The single decision point that lets a frame-holed batch through is:

- **`arq_responder.cc:786-787`** — the prev completion gate is `received_count >=
  expected_count` (a count). It must ALSO require that every slot in `[0, expected_count)`
  is RECEIVED before delivering.

The supporting blind spots that make the count-gate fatal:
- **`arq_responder.cc:744`** (prev-write bound is `data_batch_size`, not `expected_count`)
  + **`:766-767`** (count-bump is index-agnostic) → L1 over-count from tail slots.
- **`arq_common.cc:9712-9729` / `:9863-9883`** (`copy_data_to_buffer` skips holes silently).
- **`arq_responder.cc:851` / `:1917`** (`delivery_step_is_gap` is batch-level only).
- **`arq_common.cc:1054-1061`** (R035 reshrink resets streaming but delivers the
  truncated batch anyway) → L2.

### 5.2 The minimal fix (extend byte-faithful-OR-loud-abort to within-batch holes)
**Constrain the producer/gate (preferred — the consumer `copy_data_to_buffer` cannot
distinguish "intended short batch" from "holed batch" without this), NOT the silent
reassembler.** Two coupled, minimal changes:

1. **Make the prev gate a SET check, not a COUNT check.** At the gate
   `arq_responder.cc:786-787`, before delivering, require contiguous completeness of
   `[0, expected_count)`:
   add a pure helper `prev_batch_is_frame_complete()` that returns true iff
   `messages_rx_prev[i].status == RECEIVED` for **every** `i in [0, expected_count)`
   (mirroring the count but per-slot). Gate becomes
   `active && received_count >= expected_count && prev_batch_is_frame_complete()`.
   This closes L1: a tail slot in `[expected,data_batch_size)` no longer satisfies a
   missing low slot, so the gate waits for the real retransmit (the CMD re-sends it; SACK
   already drives this) OR the eventual stale-discard / demote-rebase path handles it.
   The fix is byte-identical on the faithful path (when the set IS complete, the helper is
   true). It is the RX-side analogue of the R7 "constrain the producer" decision
   (`data-flow-messages_rx_prev.md` §5).

2. **Make L2 (reshrink orphan) a LOUD abort, not a silent truncation.** In
   `rescan_prev_on_batch_shrink()` (`arq_common.cc:1054`), when `orphaned_received>0` the
   prev batch is PROVABLY no longer deliverable intact. Instead of only
   `streaming_reset()` + deliver-truncated, route through the SAME
   `rsp_gap_abort_teardown()` (`arq_common.cc:6474`) the batch-level gate uses
   (`[RSP-V2-GAP-ABORT]`, DROPPED, refuse silent concatenation). This extends the D3.1
   "loud-abort" promise to the within-batch frame-drop case symmetrically.

**Why not patch `copy_data_to_buffer` to detect the hole there?** Because by the time the
reassembler runs, an EOB-short batch (`expected < data_batch_size`) is legitimately
allowed to have FREE slots in `[expected, data_batch_size)` — the reassembler cannot tell
a legitimate short-batch tail from an illegitimate low hole without `expected_count`. The
gate (which HAS `expected_count`) is the correct owner of the completeness decision. This
is the CLAUDE.md §5 "constrain the producer/gate, don't depend on the consumer being
reasonable" rule.

### 5.3 Consumer walk for the fix (anti-sibling check — COMPLETED, fix landed)
- **§3.1 gate**: tightened — now requires the set, not the count
  (`arq_responder.cc:786`, the new condition is
  `... && (prevhole_defeat || prev_batch_is_frame_complete())`). Faithful path
  byte-identical (helper true ⇒ the count gate already fired ⇒ no faithful delivery
  delayed). Holed path: gate stays closed → the prev sits in the SAME state the
  system already handles for ANY incomplete prev (`received_count<expected_count`):
  either the CMD retransmit fills the held low slot (the frames ARE arriving on the
  prev path — that is why the prev exists — so the CMD is already retransmitting
  the earlier-batch frames, including the held low slot), OR the next bsi-bump hits
  the stale-discard (`bump_bsi_and_transfer_prev` :6364-6377, FREEs without
  delivery) / demote-rebase. **NO new deadlock surface**: "count-complete-but-set-
  incomplete" transitions to either complete or discarded, both already-handled
  terminal states. ✓
  - **Composes with D3.1 (the important property)**: if a held prev (bsi=N) is
    eventually stale-discarded undelivered, the high-water stays at N-1; the next
    current commit at N+1 is `delivery_step_is_gap(N+1, N-1)` = forward-step 2 →
    LOUD ABORT. So a held-then-discarded prev becomes a BATCH-level hole that D3.1
    catches. The pre-fix premature delivery MASKED this loss (advanced high-water to
    N silently). The fix is strictly MORE correct: byte-faithful-OR-loud-abort holds
    end-to-end. ✓
- **§3.3 `copy_data_to_buffer`**: unchanged — only ever reached for a frame-complete
  `[0,expected_count)` prev now, so its skip-holes leg is never exercised with a real
  LOW hole. NOTE (investigated, NOT a D5): it still iterates `[0, data_batch_size)`,
  so RECEIVED tail slots in `[expected_count, data_batch_size)` are ALSO delivered.
  That is NOT silent-wrong-bytes: the route-to-prev branch (`arq_responder.cc:673-674`)
  + the prev-write (`:765`) gate every stored prev slot on `bsi ==
  rsp_prev_batch_seq_id`, so a tail slot carries REAL, in-order data for the SAME
  batch (a frame arrived at a seq beyond the EOB inference — the batch was longer than
  the EOB guess, not corrupt). Delivering it in index order is faithful. The fix's
  contract is "no LOW slot skipped," which the SET check over `[0,expected_count)`
  guarantees; the tail is in-order and same-batch. ✓ (no D5)
- **§3.4 `delivery_step_is_gap`**: unchanged — still the batch-level backstop; the new
  within-batch gate is ABOVE it (held earlier) AND the batch-level net catches a
  discarded-held prev (above). ✓
- **L2 teardown reroute**: `rsp_gap_abort_teardown` already clears the prev family +
  resets the session; existing loud path, no new state. **RE-ENTRANCY VERIFIED SAFE**:
  `reset_session_state()` (`arq_common.cc:3847-4026`) does NOT call
  `set_data_batch_size()` — it touches config/compressor/cipher/optimizer/SACK-axes
  + the high-water, never the batch setter. The reshrink runs at the chokepoint
  BEFORE `this->data_batch_size = clamped/target` (`:1187`/`:1212`); the trailing
  store after a teardown is harmless on a now-DROPPED link. The test exercises this
  EXACT path (`set_data_batch_size(10)` → reshrink → teardown → reset_session_state)
  and passes (no crash/hang). ✓
- **R035 streaming defense** (`data-flow-arq-recovery-cluster.md` §5.2): NO double
  reset. The orphan path now `return`s immediately after the teardown
  (`arq_common.cc`), whose `reset_session_state()` calls `compressor.deinit()`; the
  old `streaming_reset()` leg is only reached on the non-orphan path (or under
  `MERCURY_PREVBUMP_DEFEAT=1`, which preserves the original R035 behavior). ✓

### 5.4 Regression test (CLAUDE.md cross-layer-test rule — IMPLEMENTED, pairs with this doc)
In-process synthetic-fire `--test-prevbump-frame-hole`
(`arq_responder.cc::test_prevbump_frame_hole`, CLI wired in `main.cc`), THREE arms,
each driving the REAL prev gate decision (the count gate + the REAL
`prev_batch_is_frame_complete()` helper, respecting `MERCURY_PREVBUMP_DEFEAT`), the
REAL `copy_data_to_buffer()` through the production messages_rx<->prev pointer swap,
and the REAL `fifo_buffer_rx` as a byte-exact oracle (no-compression leg, distinct
per-slot bytes `slot*16+j`):
- **L1-CLEAN arm** (false-positive guard): a frame-COMPLETE prev (slots {0..5},
  expected 6) delivers byte-identical in BOTH modes (helper true).
- **L1 arm**: `data_batch_size=30`, EOB → `expected_count=26`; RECEIVED `{0..23,26,27}`
  (count 26 reaches expected via the tail slots) with a HOLE at `{24,25}` inside
  `[0,26)`. FAIL-BEFORE (`MERCURY_PREVBUMP_DEFEAT=1`): count gate fires,
  `copy_data_to_buffer` concatenates the 26 ACKED slots `{0..23,26,27}` (416 B) with
  slot-26 bytes at the slot-24 seam (a silent byte jump 23→26) — asserted ≠ the
  buggy concat is faithful. PASS-AFTER: helper false → gate HELD (nothing delivered);
  inject the {24,25} retransmits → the full received set `{0..27}` delivers in order
  (448 B, contiguous, no hole, no jump).
- **L2 arm**: arm prev `expected=11` with a RECEIVED orphan at index 12 (`[0..9]`
  complete); SHRINK `data_batch_size 15→10` via the REAL `set_data_batch_size(10)`
  chokepoint. FAIL-BEFORE: silent re-derive (orphan bytes lost), link NOT dropped,
  prev kept active. PASS-AFTER: `[RSP-V2-GAP-ABORT]` loud teardown,
  `link_status==DROPPED`, prev/bsi family cleared, no silent delivery.

**Executed results (FIX binary, both modes rc=0):**
- `MERCURY_PREVBUMP_DEFEAT=1 --test-prevbump-frame-hole` → L1 + L2 reproduce the
  silent-wrong-bytes (fail-before confirmed), rc=0.
- `--test-prevbump-frame-hole` (default) → L1 gate held then faithful `{0..27}`;
  L2 loud GAP-ABORT DROPPED, rc=0 (byte-faithful-OR-loud-abort).

The paired R035 test `--test-batch-shrink-strands-prev` was UPDATED to the new L2
invariant: DEFAULT asserts the loud abort (DROPPED + prev cleared);
`MERCURY_PREVBUMP_DEFEAT=1` asserts the pre-fix silent re-derive + streaming defense
(preserves the original R035 counter-rederivation coverage). Both arms rc=0.

**e2e arm (NOT run this session — deliberate stand-down):** a sibling agent's
climb-sim was actively running against this worktree (holding `mercury.exe`, ports
7010/7014, `-x sim`), so per CLAUDE.md ("any re-launch MUST first check for a live
sibling and STAND DOWN — a concurrent duplicate corrupts a live bench experiment")
the local-Windows `fix_s7_250`/`fix_s11_300` climb-sim re-run was NOT launched; the
FIX binary was linked to `mercury_prevbump.exe` to avoid disturbing the sibling. The
in-process arms drive the REAL gate + reassembler + teardown + chokepoint and are the
CLAUDE.md primary regression. The e2e md5_match re-confirm is queued for when the
worktree is free (it is the bench-9 (2) assert in `CFG16_ACQ_D2D3_VERDICT.md` §5).

### 5.5 No-regression battery (executed this session, FIX binary)
`bigblock_p3_hw/_prevbump/BATTERY_RESULTS.txt`. ALL rc=0:
- **default (11)**: `--test-prevbump-frame-hole`, `--test-climb-engine`,
  `--test-inorder-demote`, `--test-gap-abort`, `--test-partial-bsi-advance=mfsk`,
  `--test-partial-bsi-advance=ofdm`, `--test-batch-shrink-strands-prev`,
  `--test-eob-poison-prev-retx`, `--test-sack-oow-reject`,
  `--test-retx-clear-on-recovery`, `--test-v2-pendingack-flip-alias`.
- **fail-before / defeat (5)** (each still reproduces its bug):
  `MERCURY_PREVBUMP_DEFEAT=1` ×{prevbump-frame-hole, batch-shrink-strands-prev},
  `MERCURY_GAP_ABORT_DEFEAT=1` ×{inorder-demote, gap-abort},
  `MERCURY_LOSSY_DEMOTE=1` ×inorder-demote.
The D3.1 (`inorder-demote`), FIX-8 (`gap-abort`), and R035 (`batch-shrink`) integrity
nets all still pass — the SET-gate + reshrink-loud-abort are an EXTENSION of, not a
regression to, the byte-faithful-OR-loud-abort battery.

### 5.6 Is there a D5? — ~~NO (the chain terminates here)~~ **YES — CORRECTED 2026-06-11 (PREV_BUMP_VERDICT.md). There IS a D5: EOB-inference batch truncation.**
~~Per CLAUDE.md §2/§5, completing a coupled fix often surfaces the next sibling. The two
candidates investigated this session, both ruled out:~~
1. **Tail-slot delivery** (`copy_data_to_buffer` iterates `data_batch_size` >
   `expected_count`): NOT silent-wrong-bytes — every prev slot is bsi-gated
   (`arq_responder.cc:673-674` route, `:765` store) so a RECEIVED tail slot is REAL
   in-order data for the SAME batch; delivering it is faithful (§5.3). ✓ (still true)
2. **Held-prev deadlock / stale-discard loss**: NOT a new defect — a held-then-
   discarded prev leaves the high-water un-advanced, so the next current commit trips
   the batch-level `delivery_step_is_gap` (D3.1) → LOUD ABORT. ✓ (still true)

~~The within-batch frame-hole was the LAST silent path… No 6th patch needed.~~

**CORRECTION (executed e2e re-run + byte-attributed trace, PREV_BUMP_VERDICT.md §2):** the
"tail slot is faithful" analysis (#1 above) considered only a *RECEIVED* tail slot. It
MISSED the **LOST-EOB tail** case: when the EOB frame(s) of a bumped batch are the missing
ones, `prev_expected = min(data_batch_size, last_received_end_of_batch_seq+1)`
(`arq_common.cc:6414-6421`) sets `expected_count` SMALLER than the TX's true frame count.
The lost tail frame lands in `[expected_count, data_batch_size)` as **FREE**, the SET-gate
over `[0, expected_count)` PASSES (that range IS complete), and the missing tail frame is
silently dropped → a one-frame (~155 B at CFG16) SILENT SKIP. md5_match FALSE on every
CFG16-reaching e2e seed (drift AND clean; fix-ON ≡ fix-DEFEAT ≡ original D2/D3 → PRE-EXISTING).
The SET-gate cannot catch it because `expected_count` ITSELF is the corrupted value (the gate
trusts the producer's inferred length). **D5 = the producer (`prev_expected` EOB inference) is
wrong on the EOB-loss path; every consumer trusts it.** Fix directions in PREV_BUMP_VERDICT.md
§4 (root cause = carry TX-authoritative batch length, do not infer it). This is a
STOP-and-discuss architecture-level item, not a 6th narrow patch. **The full §5 audit +
the chosen Direction-1 design (wire `batch_total_frames` on every v2 DATA frame) are in §8
below, on the monitor `627c370` line base.**

---

## §6 Cross-layer audit checklist (future changes to this state)
Before changing ANY of: the prev completion gate (`arq_responder.cc:786`); the prev-write
bound/count (`:744`,`:766`); `bump_bsi_and_transfer_prev` (`arq_common.cc:6355`, esp. the
`prev_expected` derivation `:6382`); `rescan_prev_on_batch_shrink` (`arq_common.cc:1029`);
`copy_data_to_buffer` (`arq_common.cc:9690`); `advance_last_delivered` /
`delivery_step_is_gap` / `rsp_gap_abort_teardown` — walk §2/§3/§4/§5 and UPDATE this
document plus the companions (`data-flow-messages_rx_prev.md`,
`data-flow-arq-recovery-cluster.md`, `data-flow-batch-size.md`).

**Special vigilance**: any change that lets `expected_count` and `data_batch_size` diverge
(EOB inference, Axis-2 resize, robust-dwell revert) re-opens L1 unless the gate stays a
SET check. Any change to the shrink path re-opens L2 unless the orphan stays a loud abort.

---

## §7 Related fact documents
- `data-flow-messages_rx_prev.md` — per-slot storage lifecycle + the R7 `loc` bound. THIS
  doc's L1 is the delivery-time analogue: R7 stopped a bit-errored HIGH `loc` from bumping
  the count; L1 is a VALID high `loc` (in `[expected,data_batch_size)`) bumping the count
  past a low hole — the bound (`data_batch_size`) is correct for storage but too loose for
  the completeness gate.
- `data-flow-arq-recovery-cluster.md` — R035 (shrink rescan) / R038 (EOB poison). R035's
  streaming-desync defense is the SYMPTOM; L2 is the byte-loss the same shrink causes that
  R035 did not close. R038 (EOB-only-on-match-current) reduces but does not eliminate the
  EOB-short condition that seeds L1.
- `bigblock_p3_hw/_cfg16acqd2d3/CFG16_ACQ_D2D3_VERDICT.md` §3 — the D4 escalation that
  surfaced this (executed e2e md5_match:FALSE evidence).
- `bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md` — the D3.1 batch-level contiguity
  design this doc extends (it explicitly scoped "dropped-batch hole," not dropped-frame).
- `bigblock_p3_hw/_trackc/TRACK_C_D2D3D5_DESIGN.md` — the coupled D2/D3/D5 design + §5
  audit on the monitor `627c370` line base (this §8 is the D5 half; D2/D3 live there).

---

## §8 D5 — EOB-inference batch truncation: §5 audit + Direction-1 design (2026-06-11, Track-C)

**Re-located against monitor `627c370`** (worktree `C:/Users/kamer/mercury_wt/trackc-d2d3d5`).
The §1-§5 above cite the `05fb9b7` (cfg16-acq-d2d3) tree; the monitor lines for the D5
sites are below — a merge touches THESE. D5 is the producer-side defect §5.6 corrected the
"no D5" conclusion to admit. This §8 is the full §5-audit treatment + the chosen fix.

### §8.1 The defect (executed, PREV_BUMP_VERDICT.md §2)
`bump_bsi_and_transfer_prev()` derives the prev's `expected_count` by INFERENCE:
`prev_expected = data_batch_size; if(last_received_end_of_batch_seq>=0){ eob =
last_received_end_of_batch_seq+1; if(eob<prev_expected) prev_expected=eob; }`
(monitor **arq_common.cc:6382-6389**; written to `rsp_prev_batch_expected_count` at
**arq_common.cc:6435** region). The batch length is signalled on the wire by exactly ONE
frame — the EOB-bit-7-marked LAST frame (TX **arq_common.cc:5610-5614**
`sequence_number |= 0x80`; RX decode **arq_common.cc:8966-8974** `rx_buffer_eob_seq =
eob_seq`). **When that EOB frame is LOST**, `last_received_end_of_batch_seq` latches a
LOWER index (the highest OTHER seq seen), so `expected_count < true_count`; the missing
tail frame in `[expected_count, data_batch_size)` stays FREE; the SET-gate over
`[0, expected_count)` PASSES; the tail is silently dropped. Executed: TX
`[CMD-V2-MIXBATCH] … = 30 total`, RX delivers `expected_prev=29 FREE=1` → one CFG16 frame
(~155 B) silently absent; md5_match FALSE on EVERY CFG16-reaching seed (drift AND clean).

### §8.2 PRODUCERS of the inferred length (monitor lines)
- **EOB-mark TX** arq_common.cc:5610-5614 (single-frame signal).
- **EOB-decode RX** arq_common.cc:8966-8974 (`rx_buffer_eob_seq`, promoted to
  `last_received_end_of_batch_seq` on match-current).
- **prev `expected_count`** arq_common.cc:6382-6389 — the D5 ROOT producer.
- **in-place ACK-GATE `effective_batch`** arq_responder.cc:1087 region (same EOB inference
  for the CURRENT batch; the AT-:1087 comment already names the hazard).
- **SACK-bitmap `expected`** arq_responder.cc:1639 (`expected = last_received_end_of_batch_
  seq + 1`).

### §8.3 CONSUMERS that trust the inferred length (monitor lines)
- prev completion GATE arq_responder.cc:787 (count test over `expected_count`).
- `copy_data_to_buffer` reassembler (skips FREE tail silently).
- SACK partial gate arq_responder.cc:1672 (`rx_received < expected`) — **the dead end:**
  when the EOB is lost, `expected = highest-seq+1` → `rx_received >= expected` → FALSE → NO
  SACK for the lost tail → the lost frame is NEVER retransmitted (the RX does not know it
  exists). D5 is structurally UNHEALABLE by the existing retx machinery (§4.3 of the
  Track-C design); the length channel has no redundancy for its own terminator.

### §8.4 The broken invariant
**INV-ASSUMED** (gate + reassembler + SACK + D3.1): `expected_count` == the TX's true
per-batch frame count. **INV-VIOLATED**: on the EOB-loss path it is highest-seq-RECEIVED+1
< true count. The producer is wrong on the uncommon EOB-loss path; every consumer is
reasonable but blind — the canonical CLAUDE.md §5 failure mode.

### §8.5 The fix — Direction 1: carry the count on the wire (root cause)
Carry a per-batch `batch_total_frames` (1..MAX_SACK_BATCH_SIZE=32) on EVERY DATA frame of
a batch (survives loss of any single frame, including the EOB frame), under the EXISTING
`sack_v2_enabled` gate. Realization: grow the v2 DATA_LONG/DATA_SHORT header by ONE byte
(the same reversibility pattern as the `batch_seq_id` byte, arq.h:264-271); 0 reserved =
unknown/legacy. The seq field (7-bit slot) and `batch_seq_id` (full mod-256) have NO spare
bits (`static_assert(MAX_SACK_BATCH_SIZE<=128)` arq.h:289, bsi wrap) → a new byte is the
only clean carrier. Consumers latch `rx_batch_total_frames` on any received frame of the
bsi and set `prev_expected` (arq_common.cc:6382), the SACK `expected` (arq_responder.cc:1639),
and the in-place `effective_batch` (arq_responder.cc:1087) FROM IT, not from the EOB
inference (which becomes the v1/legacy fallback only).

**Composition with L1:** wired count → a lost EOB leaves `received_count < expected_count`
→ the L1 SET-gate HOLDS → SACK retransmit (now the tail IS inside `expected`, so the RX
asks for it) → faithful; OR stale-discard → high-water un-advanced → next current commit
trips `delivery_step_is_gap` (D3.1) → LOUD `[RSP-V2-GAP-ABORT]`. **Faithful-after-retx OR
loud-abort — never a silent skip.** D5 makes the L1 gate's UNIVERSE trustworthy; L1 without
D5 trusts a corrupted universe.

**Direction 2** (loud-abort every EOB-short bump) is the RETAINED SAFETY NET for a residual
unknown-count case, NOT the primary fix (too blunt — aborts every legitimately-short final
batch). **Direction 3** (hold the bump until EOB confirmed) is REJECTED (changes control
timing, strains the half-duplex turnaround budget).

**Byte-identical:** `sack_v2_enabled==false` → no new byte, EOB inference unchanged → v1/
legacy/NB byte-identical. v2 no-loss → `rx_batch_total_frames == eob+1` → derived
`expected_count` identical → faithful path byte-identical. The change bites only on the
EOB-loss path (today silent-wrong-bytes).

### §8.6 Failing-test-first
New `--test-eob-loss-batch-truncation`: 30-frame batch, drop frame 29 (the EOB), bump.
FAIL-BEFORE (`MERCURY_D5_INFER_DEFEAT=1`, EOB inference kept): `expected_count=29`, prev
delivers 29 frames (silent skip), md5 ≠ tx_prefix. PASS-AFTER (wired count):
`expected_count=30`, `received_count=29<30` → SET-gate HOLDS → inject frame-29 retransmit →
faithful 30-frame delivery; assert the RX SACKs frame 29 (now inside `expected`).
Byte-identical guard: no-loss v2 batch ≡ today; v1 batch byte-for-byte unchanged.

### §8.7 §5.6 correction status
§5.6's struck "no D5" is CONFIRMED corrected: D5 = the lost-EOB-tail the original §5.6
"tail slot is faithful" analysis missed (it considered only a RECEIVED tail). The chain is
D1→D2→D3→D4(L1/L2)→D5 — five coupled defects at the gearshift-acq→delivery boundary
(CLAUDE.md §2/§5 STOP-and-design, satisfied by TRACK_C_D2D3D5_DESIGN.md).

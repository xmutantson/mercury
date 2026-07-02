# Data-Flow Audit: ARQ Recovery / Retx-Queue / Prev-Batch / SACK-v2 Cluster

**Status**: Authoritative as of 2026-06-06 (built from the 170-agent adversarial
race audit, `race_audit/RACE_AUDIT_REPORT.md` + `race_audit_rollup.json`). This is
the mandatory CLAUDE.md §"Cross-Layer Data-Flow Audits" gate for the FIVE confirmed
cluster races **R029 / R030 / R035 / R038 / R039** plus the independent **R006**.
NONE of these fixes may be implemented before this document's per-fix consumer walk
(§5) is reviewed. Every change to the structures in §1 MUST update this document.

**Why one document for five races**: R029/R030/R035/R038/R039 all read or write ONE
shared-state cluster — the TX retransmit queue (`retransmit_*` parallel arrays +
`retransmit_count`), `messages_tx[]` identity/status, `data_batch_size`,
`messages_rx_prev[]` + `rsp_prev_batch_*` counters, `last_received_end_of_batch_seq`
/ `effective_batch`, and the SACK-v2 accept window. This is the exact sibling-bug
trap CLAUDE.md cites (retx-queue chain 184fdcc→76f6185→ff829d5, 2026-05-22). Fixing
any one in isolation risks surfacing a sibling. R006 (`shutdown_` non-atomic bool) is
INDEPENDENT of the cluster and audited briefly in §7.

**Companion docs** (do not duplicate — cross-reference):
`data-flow-retx-queue.md` (TX retx queue producers/consumers/invariants),
`data-flow-messages_rx_prev.md` (RX prev-batch — §6/§8 ALREADY flag R035 unverified),
`data-flow-batch-size.md` (the `set_data_batch_size()` chokepoint),
`data-flow-robust-tier-arq-batch.md` (robust-dwell batch op).

**Line-number note**: the rollup line numbers had drifted; all line numbers below were
RE-LOCATED against the live `monitor` tree on 2026-06-06 and are authoritative as of
this date. Where they differ from the rollup, this doc wins.

---

## §1 The shared-state cluster — declarations

All members of `cl_arq_controller` (`mercury/include/datalink_layer/arq.h`):

### 1.1 TX retransmit queue (R029, R030)
```
arq.h:1386  int retransmit_count;                                      // # frames queued
arq.h:1388  unsigned char retransmit_frames[MAX_RETRANSMIT_HEADROOM][MAX_SACK_FRAME_SIZE];
arq.h:1402  unsigned char retx_scratch[MAX_RETRANSMIT_HEADROOM][MAX_SACK_FRAME_SIZE]; // 184fdcc
arq.h:1403  int retransmit_frame_lengths[MAX_RETRANSMIT_HEADROOM];
arq.h:1404  int retransmit_frame_positions[MAX_RETRANSMIT_HEADROOM];    // = messages_tx array index i at capture
arq.h:1405  int retransmit_frame_types[MAX_RETRANSMIT_HEADROOM];
arq.h:1406  int retransmit_frame_batch_seq_ids[MAX_RETRANSMIT_HEADROOM]; // ORIGINAL bsi (epoch label)
arq.h:1418  unsigned char retransmit_frame_seq_with_eob[MAX_RETRANSMIT_HEADROOM]; // orig seq byte (low7=slot, bit7=EOB)
```
`MAX_RETRANSMIT_HEADROOM = 2*MAX_SACK_BATCH_SIZE = 64` (arq.h:216), `MAX_SACK_BATCH_SIZE=32` (arq.h:208).

### 1.2 messages_tx[] identity/status (R029, R030)
`st_message messages_tx[nMessages]` (nMessages default 120). Per-slot fields that
matter here: `.id`, `.sequence_number`, `.batch_seq_id`, `.status`, `.length`, `.data`.
Codebase-wide identity invariant: `add_message_tx_data` sets `messages_tx[i].id = i`
(`arq_commander.cc:1254`) — **array-index == .id == wire-position**. The v2 mixbatch
new-data fill loop OVERWRITES it: `messages_tx[i].id = pos_in_new_batch`
(`arq_commander.cc:1643`), and the retx prefix sets the wire id from the ORIGINAL
byte: `messages_batch_tx[r].id = original_seq_eob & 0x7F` (`arq_commander.cc:1457`).

### 1.3 data_batch_size (R035)
`arq.h: int data_batch_size`. Default 1 (`arq_common.cc:209`). Sole OFDM setter is the
chokepoint `set_data_batch_size()` (`arq_common.cc:644`); SACK-test direct assigns
deliberately bypass it (`arq_responder.cc:2949/3054`, documented at arq_common.cc:663-665).

### 1.4 messages_rx_prev[] + counters (R035, R038)
```
arq.h:1455  int  rsp_prev_batch_seq_id;            // -1 = none
arq.h:1587  bool rsp_prev_batch_active;
arq.h:1596  int  rsp_prev_batch_received_count;
arq.h:1600  int  rsp_prev_batch_expected_count;   // FROZEN at arm-time from data_batch_size / EOB+1
```
`messages_rx_prev` = pointer member, `nMessages` slots. Full lifecycle owned by
`data-flow-messages_rx_prev.md` — do not duplicate; this doc adds the
`data_batch_size`-shrink hazard that doc's §6/§8 flagged UNVERIFIED.

### 1.5 EOB / effective_batch (R038)
`arq.h:2445 int last_received_end_of_batch_seq;` (-1 = none). `effective_batch` is a
LOCAL recomputed in two RSP consumers from this global. Default/cleared = -1
(`arq_common.cc:497`, and cleared at `arq_responder.cc:1602/1631/1792`).

### 1.6 SACK-v2 accept window (R039)
`arq.h:2142 int cmd_last_applied_sack_bsi;` (the partial-dedupe tracker). The "window"
is the derived pair `{cmd_bsi, prev_bsi}` = `{cmd_batch_seq_id & 0xFF, (that-1)&0xFF}`,
recomputed at each accept arm.

---

## §2 Producers (writers) — file:line

### 2.1 retransmit queue / retransmit_count
- **Init**: `arq_common.cc:230` (`retransmit_count=0`, ctor/init_messages_buffers).
- **SACK capture (sole per-entry producer)**: `arq_commander.cc:2896-2962` — the
  `if(sack_detected)` loop. Per missing slot `i` it writes `retransmit_frames[]`,
  `_lengths[]`, `_positions[i]=i` (:2942), `_types[]`, `_batch_seq_ids[]=messages_tx[i].batch_seq_id`
  (:2949 — the EPOCH LABEL), `_seq_with_eob[]=messages_tx[i].sequence_number` (:2956),
  `retransmit_count++` (:2958). v2 = APPEND mode (does NOT reset count; :2889-2890).
- **Survivor shift**: `arq_commander.cc:1576-1585` — memmove down by R after the
  mixbatch drains R entries; `retransmit_count = leftover` (:1585).
- **Drain/reset sites**: `arq_commander.cc:1354` (v1 retx-only consume),
  `arq_commander.cc:1502` (runaway-BREAK — the ONLY recovery-class clear),
  `arq_commander.cc:2890` (v1 SACK reset, NOT v2), `arq_commander.cc:9086` (a reset path).
- **NOT cleared by ANY messages_tx-freeing recovery site** — see §2.2. This is R029.

### 2.2 messages_tx[]-freeing recovery sites that LEAVE retransmit_count dangling (R029)
Each frees `messages_tx[]` (status=FREE) and re-queues plaintext to `fifo_buffer_tx`,
but NONE touches `retransmit_count`/`retransmit_frames[]`:
- **Watchdog timeout** (COMMANDER): `arq_common.cc:2269-2272` (FREE loop), restore at :2274-2291.
- **Gearshift-down SNR_BASED**: `arq_common.cc:2356-2359` (FREE loop), restore :2361-2378.
- **Gearshift-down SUCCESS_BASED_LADDER**: `arq_common.cc:2449-2452` (FREE loop), restore :2454-2471.
- **BREAK recovery (ACK-received phase-1)**: `arq_commander.cc:258-264` (FREE loop, else-branch of `restore_tx_from_compressed`).
- **BREAK recovery (EXHAUSTED)**: `arq_commander.cc:344-350`.
- **`reset_session_state()`**: `arq_common.cc:3185-3339` — exhaustively re-inits
  session state but has NO `retransmit_count=0` line (verified read 2026-06-06). Called
  from FORCED_ROLE_SWITCH (`arq_common.cc:2329`), disconnect, role-switch, etc.
- **`restore_tx_from_compressed()`**: `arq_common.cc:7216` — rebuilds `messages_tx[]`
  from plaintext precisely because in-flight encrypted bytes are fragile, but does NOT
  touch the retx queue (whose bytes are ALSO encrypted under the dying epoch).

### 2.3 messages_tx[] identity (.id) producers (R030)
- `add_message_tx_data`: `arq_commander.cc:1254` `.id = i` (array index).
- v2 mixbatch new-data fill: `arq_commander.cc:1643` `.id = pos_in_new_batch`.
- v2 mixbatch retx prefix wire slot: `arq_commander.cc:1457` `messages_batch_tx[r].id = original_seq_eob & 0x7F`.

### 2.4 messages_tx[].status PENDING_ACK flip — post-TX (R030)
- `arq_common.cc:3978-3991` — after `send_batch`, iterate `messages_batch_tx[0..counter)`;
  for each DATA frame `id = (unsigned char)messages_batch_tx[i].id` (:3982), set
  `messages_tx[id].status = PENDING_ACK` (:3984). **Indexes by `.id`, NOT by array slot.**

### 2.5 data_batch_size producers (R035)
- Chokepoint `set_data_batch_size()`: `arq_common.cc:691` (robust range clamp),
  `:708/:712` (OFDM clamp). Sole non-test setter.
- RSP SET_LINK_PARAMS (Axis-2 down/up): `arq_responder.cc:2669` `set_data_batch_size(target)`.
- RSP ROBUST_DWELL_BATCH_OP (dwell raise/revert): `arq_responder.cc:2753`.
- CMD-side movers funnel through the same setter: `arq_commander.cc:5725/5849`,
  `arq_common.cc:854/1419/1482/1508`.
- Test direct-assigns (bypass): `arq_responder.cc:2949 (=25)`, `:3054 (=30)`,
  `arq_commander.cc:5915/5985/6004/6093`.

### 2.6 rsp_prev_batch_expected_count producers (R035)
- ONLY `bump_bsi_and_transfer_prev()`: `arq_common.cc:4436` sets it from
  `prev_expected` (derived at :4383-4390 from `data_batch_size` or
  `last_received_end_of_batch_seq+1`). **No `data_batch_size`-shrink path re-derives it.**
- Reset to 0: ctor/`reset_session_state` and post-delivery (`arq_responder.cc:768`).

### 2.7 last_received_end_of_batch_seq producers (R038)
- **Pre-routing capture (the poison)**: `arq_common.cc:6362` — inside `receive()`,
  for ANY connection-id-matching, CRC-valid (message_decoded) DATA frame with bit-7 set,
  BEFORE the responder classifies match-current / match-prev / drop.
- **Clears (-1)**: `arq_common.cc:497` (init), `arq_responder.cc:1602` (post-SACK),
  `:1631` (ACK-GATE suppress), `:1792`, `arq_commander.cc:4610`.

### 2.8 SACK-v2 accept (R039)
- `decode_sack_v2_frame()`: `arq_common.cc:4795-4863` — CRC8-only (`rx_crc!=computed_crc`
  reject at :4818); writes `*out_batch_seq_id = payload[0]` (:4837) with NO bsi validation.

---

## §3 Consumers (readers) — file:line

### 3.1 retransmit queue consumers
- **v2 mixed-batch builder**: `arq_commander.cc:1467-1594` — `if(sack_v2_enabled &&
  retransmit_count>0)`. Reads ALL parallel arrays, sets wire `bsi` from
  `retransmit_frame_batch_seq_ids[r]` (:1562), copies bytes to `retx_scratch[r]` (:1550).
  **NO EPOCH GUARD** — drains whatever bsi/crypto-epoch the captured frames carry.
- **v1 retx-only builder**: `arq_commander.cc:1291-1354` (`!sack_v2_enabled` path).
- **Runaway safety-net (pre-pop)**: `arq_commander.cc:1479` reads `retransmit_count`.
- **SACK capture self-ref**: `arq_commander.cc:2909` (`retransmit_count < HEADROOM`).

### 3.2 messages_tx[] status/identity consumers (R030 blast radius)
- **Post-TX PENDING_ACK flip**: `arq_common.cc:3982-3984` (writes by `.id` — see §2.4).
- **Clean full-batch ACK**: `arq_commander.cc:3119-3126` — iterates by ARRAY INDEX `i`,
  calls `register_ack(i)` (:3124). `register_ack` (`arq_commander.cc:51-58`) only acts if
  `messages_tx[message_id].status==PENDING_ACK`.
- **SACK bitmap apply**: `arq_commander.cc:2896-2962` — iterates by ARRAY INDEX `i`,
  reads `sack_bitmap[i]` (:2901). **Bitmap is indexed by array slot, but the wire frame
  carried `.id = pos_in_new_batch`** — the divergence R030 exploits.
- **LDPC fallback ACK_RANGE/ACK_MULTI**: `arq_commander.cc:3221-3253` (by id from wire).
- **ACK-timeout aging**: `update_status` PENDING_ACK→ACK_TIMED_OUT→FAILED_ path
  (`arq_commander.cc:3399`, :3121, :1659-1678).

### 3.3 data_batch_size consumers (R035 + cross-cutting)
- **Prev-write bound (LIVE)**: `arq_responder.cc:686` `loc >= data_batch_size` reject.
- **Prev transfer loop bound (LIVE)**: `arq_common.cc:4403` `i<data_batch_size`.
- **Prev delivery loop bound (LIVE)**: `arq_responder.cc:752`.
- **`prev_expected` derivation**: `arq_common.cc:4383` (reads `data_batch_size` at arm).
- **ACK-GATE `expected`/`effective_batch`**: `arq_responder.cc:866,1423`.
- (many more — clean-ACK target `(1<<data_batch_size)-1`, SACK bitmap width, etc.)

### 3.4 rsp_prev_batch_expected_count consumers (R035)
- **Live completion gate**: `arq_responder.cc:728-729`
  (`received_count >= expected_count` → deliver + streaming defense).
- **Stale-prev discard**: `arq_common.cc:4365-4378` — on re-bump while prev active,
  FREEs `messages_rx_prev[]` slots; **NO delivery, NO streaming_reset()**. (R035 hazard.)

### 3.5 last_received_end_of_batch_seq consumers (R038)
- **Per-frame timer**: `arq_responder.cc:872-876` → `effective_batch`; gate at :897
  (`batch_rx_frame_count >= effective_batch` → immediate ACK timeout).
- **Final ACK-GATE**: `arq_responder.cc:1427-1431` → `expected`; gate at :1462
  (`rx_received < expected` decides PASS vs SACK).
- **`prev_expected` derivation** (shared with R035): `arq_common.cc:4384`.

### 3.6 SACK-v2 accept arms (R039 — the asymmetry)
- **OFDM SACK_RSP arm**: `arq_commander.cc:2814-2851` — calls `decode_sack_v2_frame`
  (:2819), then ONLY `(int)rx_bsi == cmd_last_applied_sack_bsi` exact-dup reject (:2824).
  **NO `{cmd_bsi,prev_bsi}` window check.** Bitmap applied by slot index in the
  subsequent `if(sack_detected)` block (:2901), no per-slot bsi guard.
- **MFSK clean-ACK arm**: `arq_commander.cc:121-126` — HAS the window check.
- **MFSK partial arm**: `arq_commander.cc:2642-2645` (`bsi_in_window`) gates the bitmap
  application at :2700-2701 inside `if(bsi_in_window && bitmap_ok && !duplicate)` (:2669).

---

## §4 Valid states + the broken invariants

### 4.1 retransmit queue (R029)
| state | retransmit_count | _batch_seq_ids[r] | crypto-epoch of bytes | valid? |
|---|---|---|---|---|
| empty | 0 | (n/a) | (n/a) | ✓ |
| populated current-epoch | >0 | current/prev bsi | current crypto batch | ✓ |
| **stale post-recovery** | >0 (DANGLING) | OLD-config bsi | OLD/dead crypto epoch | **INVALID — R029** |

**INV-R029 (broken)**: every entry in `retransmit_frames[0..count)` must belong to the
LIVE crypto epoch + current bsi window. Producers §2.2 free `messages_tx[]` and restore
PLAINTEXT under a NEW epoch but leave `retransmit_count>0` pointing at frames encrypted
under the DEAD epoch. The §3.1 consumer (no epoch guard) prepends them to the first
post-recovery v2 mixbatch. Severity bounded by the RSP `[RSP-V2-DROP]` guard
(`arq_responder.cc:618-628`) which drops bsi∉{current,prev}; silent corruption needs an
8-bit bsi mod-256 collision. → medium.

### 4.2 messages_tx[] identity (R030)
**INV-R030 (broken)**: the post-TX flip (§2.4) assumes `messages_tx[.id]` is the slot
that holds the frame's payload/tracking. True ONLY while `array-index == .id`. The v2
mixbatch builder breaks this: retx prefix `.id = orig_wire_pos` (data in `retx_scratch`,
NOT `messages_tx`); new-data `.id = pos_in_new_batch` (≠ array index `i` once holes
exist). When cleanup-churn diverges array index from wire position, the flip writes
`messages_tx[old_wire_id]` — a FREE slot (→ spurious `PENDING_ACK len=0` → ages to
ACK_TIMED_OUT/`nNAcked_data++` → FAILED_/`nLost_data++`, possible garbage resend) or a
slot holding the NEXT batch's queued new data (corrupts its tracking). Delivered payload
is NOT corrupted (184fdcc `retx_scratch` decoupled the wire bytes) → medium.

### 4.3 prev-batch counters under data_batch_size shrink (R035)
**INV-R035 (broken)**: `rsp_prev_batch_expected_count` (frozen at OLD `data_batch_size`)
must remain reachable by the LIVE-`data_batch_size`-bounded prev-write path. An Axis-2
down (15→10) or robust-dwell revert (8→1) shrinks `data_batch_size` while prev is active;
the §2.5 setters never re-derive `expected_count`. A prev frame whose slot ∈ [new,old)
is rejected by the LIVE bound (`arq_responder.cc:686`), so `received_count` can never
reach the frozen `expected_count` → prev never delivers via §3.4 gate → eventual
re-bump hits the stale-discard (`arq_common.cc:4365-4378`) which FREEs **without
streaming_reset()** → PPMd model desync (ff829d5 class). → medium.

### 4.4 effective_batch / EOB poison (R038)
**INV-R038 (broken)**: `last_received_end_of_batch_seq` must reflect the EOB of the
CURRENT batch only. The producer §2.7 (`arq_common.cc:6362`) captures it pre-routing for
ANY CRC-valid EOB DATA frame. A CRC-valid prev-retransmit / late-duplicate of a SHORTER
prior batch sets `effective_batch = eob+1 < current real size`; if the current batch's
own EOB is lost/late while earlier current frames accumulate to `>= that shorter size`,
the §3.5 gate PASSes early → truncated batch delivered as complete → bsi bumped →
un-arrived current frames dropped out of window. CRC16 kills the bit-flipped-bsi vector,
but legitimate prev-retransmit / late-duplicate of a shorter batch are CRC-valid. → medium.

### 4.5 SACK-v2 accept window (R039)
**INV-R039 (broken)**: the OFDM SACK_RSP consumer must reject `rx_bsi ∉ {cmd_bsi,
prev_bsi}` before applying the bitmap, mirroring the MFSK arms (§3.6). It does not. A
double-checksum (LDPC + CRC8) false-decode out-of-window SACK_RSP with `bsi !=
cmd_last_applied_sack_bsi` is applied by slot index against the CURRENT `messages_tx[]`
describing a DIFFERENT batch → silent mis-ACK (lost frame spuriously bit=1) or needless
retransmit (bit=0). Routine-timing race is refuted (one-outstanding-batch invariant +
§6g bump hoist); the live exploit needs the rare double-checksum false-accept, but it is
an UNINTENDED asymmetry on a silent-data-loss path. → medium (returns to high if the §6g
bump discipline ever regresses).

---

## §5 Per-fix consumer walk (the anti-sibling-bug check)

For each fix: which invariant it changes, then EVERY consumer of the touched state
re-verified. Sibling interactions flagged explicitly.

### 5.1 R029 — `clear_retx_queue()` at every recovery site + reset_session_state
**Changes**: adds a producer that zeroes `retransmit_count` (and is the canonical
reset) at the §2.2 sites. Restores INV-R029.
**Consumer walk** (§3.1):
- v2 mixbatch builder (`arq_commander.cc:1467`): with count==0 post-recovery it simply
  builds a pure new-data batch. The plaintext that the recovery already re-queued to
  `fifo_buffer_tx` re-enters as fresh new-data under the new epoch — NO data loss. ✓
- v1 retx-only / safety-net / capture-self-ref: all read count==0 → no-op. ✓
**Sibling interactions**:
- **R030**: IF R030's preferred fix (separate retx-prefix structure) lands, the recovery
  clear must ALSO zero that new structure. Sequencing matters — see §6. With R030's
  MINIMAL fix (discriminate `i<v2_retx_prefix_count`), no new state to clear. The
  `clear_retx_queue()` helper MUST be the single owner of "zero ALL retx-prefix state"
  so R030's structure (if added) is cleared in one place.
- **R035/R038/R039**: no shared state — R029 is TX-side recovery, those are RX/accept. ✓
**Verdict: SAFE. Confirm fix sketch** (factor `clear_retx_queue()` mirroring
`arq_commander.cc:1502`, call from all §2.2 sites + add to `reset_session_state()`).
Re-located sites differ slightly from rollup; the AUTHORITATIVE site list is §2.2.

### 5.2 R035 — rescan + re-derive at the `set_data_batch_size()` chokepoint on shrink
**Changes**: on shrink-with-prev-active, the chokepoint rescans `messages_rx_prev[]`
RECEIVED slots `< new`, recomputes `received_count`, re-derives
`expected_count = min(old, new)`; if any RECEIVED prev slot is orphaned in [new,old),
fires the streaming desync defense (`streaming_reset()` guarded by `is_streaming() &&
batch_data_delivered`) before the inevitable discard. Restores INV-R035.
**Consumer walk** (§3.3, §3.4):
- Live completion gate (`arq_responder.cc:728`): now sees a re-derived
  `expected_count ≤ new` that the LIVE-bounded prev-write path CAN reach → prev completes
  + delivers normally. ✓
- Stale-discard (`arq_common.cc:4365`): only reached if the rescan still can't complete;
  now the streaming defense already fired at the chokepoint so the discard no longer
  silently desyncs PPMd. ✓ (Alternative: make the discard leg symmetric with the
  delivery leg's `streaming_reset` — see §6 note.)
- Transfer/prev-write/delivery loop bounds (all LIVE `data_batch_size`): unchanged; the
  rescan only adjusts COUNTERS to match the live bound, not the bounds. ✓
**Sibling interactions**:
- **R038 (POSITIVE)**: R035's rescan and `bump_bsi_and_transfer_prev`'s `prev_expected`
  both read `last_received_end_of_batch_seq` / `data_batch_size`. R038's fix (EOB only
  on match-current) makes that value reflect the genuine current batch, so R035's
  re-derived `expected_count` is MORE accurate, not less. R038 STRENGTHENS R035's
  precondition. Land R038 first or together (§6).
- **Test-mode caveat**: SACK-test direct-assigns bypass the chokepoint
  (`arq_responder.cc:2949/3054`). The rescan lives in the chokepoint, so those tests are
  unaffected — and the production shrink writers (§2.5) DO funnel through it. The R035
  regression test MUST drive the shrink via `set_data_batch_size()`, not a direct assign.
- **R029/R030/R039**: no shared state. ✓
**Verdict: SAFE, but co-land with R038. Confirm fix sketch** — chokepoint is
`arq_common.cc:644`; both shrink writers (`arq_responder.cc:2669`, `:2753`) funnel through
it, confirmed 2026-06-06.

### 5.3 R038 — capture EOB only on confirmed match-current
**Changes**: delete the pre-routing write at `arq_common.cc:6362`; re-emit inside the
match-current routing block (v2: only when `bsi == rsp_current_expected_batch_seq_id`;
v1 unchanged). Restores INV-R038.
**Consumer walk** (§3.5):
- Per-frame timer (`arq_responder.cc:872`): now `effective_batch` shrinks ONLY on a real
  current-batch EOB → no early ACK from a prev-retransmit. ✓
- Final ACK-GATE (`arq_responder.cc:1427`): same — `expected` no longer poisoned. ✓
- `prev_expected` derivation (`arq_common.cc:4384`) — **SIBLING with R035**: `bump_bsi_
  and_transfer_prev` runs at `arq_responder.cc:1546` INSIDE the ACK-GATE block, reading
  `last_received_end_of_batch_seq` BEFORE it is cleared at :1602/:1631. With R038, that
  value is the genuine current batch's EOB (or -1), so `prev_expected` is correct. The
  R038 fix sketch's "verify `prev_expected` unaffected" is answered: it IS affected, in
  the CORRECT direction (it becomes accurate). No new bug; R038+R035 are mutually
  reinforcing. ✓
**Sibling interactions**:
- **R035**: positive, as above. Co-land.
- **R039**: independent (accept-side, not RX-routing). ✓
- **v1 path**: must stay byte-for-byte unchanged — the re-emit MUST be gated so v1
  sessions still set EOB exactly where they do today (v1 has no bsi routing; emit in the
  v1 storage path unconditionally). Verify the v1 ACK-GATE (`arq_responder.cc:1462`,
  `data_batch_size>1` branch) sees identical `expected`.
**Verdict: SAFE. Confirm fix sketch** — producer at `arq_common.cc:6362`, re-emit in the
match-current block (the v2 routing decision is at `arq_responder.cc:615`, match-current
storage at :846-954). The rollup's ":846 match-current block" is the `else if(!v2_route_drop)`
storage block; emit there gated on `match_current` (recompute or thread the flag).

### 5.4 R039 — add `{cmd_bsi,prev_bsi}` window guard to the OFDM SACK_RSP arm
**Changes**: at `arq_commander.cc:2823`, after `decode_sack_v2_frame` returns `rx_bsi`,
if `!((unsigned)rx_bsi==cmd_bsi || (unsigned)rx_bsi==prev_bsi)` log `[CMD-SACK-V2-OOW]`
and treat as CRC fail (fall through to timeout-driven full-batch retransmit). Restores
INV-R039. Mirrors the MFSK arm at `arq_commander.cc:2642-2645`.
**Consumer walk** (§3.6):
- The bitmap-apply block (`arq_commander.cc:2896-2962`) now only runs for in-window
  `rx_bsi` → no apply against a foreign-batch `messages_tx[]`. ✓
- Existing exact-dup reject (`rx_bsi==cmd_last_applied_sack_bsi`, :2824) is preserved and
  additive. ✓
- `policy_evaluate_axis3` / arrival history: only updated on accepted SACK → unchanged
  for valid SACKs; an OOW reject feeds `policy_evaluate_axis3(false)` (the else at :2848),
  identical to a CRC fail — correct (a false-decode SHOULD count as a miss). ✓
**Sibling interactions**: NONE. R039 is a pure consumer-side hardening on the accept arm;
touches no producer of cluster state. Independent of R029/R030/R035/R038. ✓
**Verdict: SAFE, fully independent. Confirm fix sketch** — guard site
`arq_commander.cc:2823`, mirror source `arq_commander.cc:2642-2645`, both confirmed
2026-06-06. Optional deeper per-slot `messages_tx[i].batch_seq_id==rx_bsi` check in the
bitmap loop is additive and harmless.

### 5.5 R006 — `shutdown_` → `std::atomic<bool>`
**Changes**: behavior-neutral type change. No cluster state. See §7.

---

## §6 SAFE FIX ORDER

The five cluster fixes are mostly independent; two pairs interact. Recommended order:

1. **R039** (first — fully independent, lowest blast radius, pure accept-side guard,
   trivial fail-before/pass-after test). No prerequisite.
2. **R038** (RX EOB capture) — land BEFORE or WITH R035 because R038 makes
   `prev_expected` accurate, which R035 relies on. Independent of R029/R030/R039.
3. **R035** (prev-batch shrink rescan at the chokepoint) — co-land with or immediately
   after R038. Its regression test must drive the shrink via `set_data_batch_size()`.
4. **R029** (clear_retx_queue at recovery sites + reset_session_state) — land BEFORE R030
   so the `clear_retx_queue()` helper exists as the single owner of "zero all retx-prefix
   state"; R030's fix then extends that helper if it adds a structure.
5. **R030** (v2 PENDING_ACK flip aliasing) — LAST of the cluster. If the PREFERRED fix
   (separate `(bsi,low7)`-keyed retx-prefix structure) is chosen, it MUST register the
   new structure in `clear_retx_queue()` (R029) and in `data-flow-retx-queue.md`. If the
   MINIMAL fix (discriminate `i<v2_retx_prefix_count` + route by `(bsi,low7)`) is chosen,
   no new state and the ordering constraint relaxes. **Recommend the MINIMAL fix first**
   (smaller blast radius, no new clear-site coupling), escalate to the separate structure
   only if the minimal discriminator proves insufficient.

R006 may land at any point (independent; behavior-neutral).

**One-change-one-test discipline** (CLAUDE.md / MEMORY autonomous-iteration): implement
+ test each fix individually in the above order; do not bundle (DSP-commit-pattern
lesson: every multi-variable cluster commit in this codebase shipped a residual bug).

---

## §7 R006 (independent) — `shutdown_` non-atomic bool

**Files**: `main.cc:225` `bool shutdown_;`; `audioio/audioio.c:52` `extern bool
shutdown_;`. Written `true` at `main.cc:2577` and `audioio.c:886/1264/1381/1579/1624`;
read in ~14 spin loops across the main thread (`main.cc:2500/2693/2709/2754/2770/2790/
2812`) AND the audio capture/playback/sim threads (`audioio.c:669/1055/1275/1289/1292/
1554/1584/1622/1624/1626/1631/1645/1650`).

**Verdict**: the HANG symptom is refuted (no `-flto`; every spin body has an opaque
out-of-TU call forcing a reload; no SIGINT handler) but the formal C/C++ DATA RACE / UB
is REAL (concurrent unsynchronized read+write of a non-atomic object). One build-flag
(`-flto`, clang cross-module IPA) or a new same-TU fence-free spin from biting. → low.

**Fix**: `std::atomic<bool> shutdown_{false}` (matching `gui_state.h:185`
`std::atomic<bool> request_shutdown{false}`), `memory_order_relaxed` on the spins. For
the C TU (`audioio.c`), use `_Atomic bool` via `stdatomic.h` OR a small `extern "C"`
accessor in the C++ TU. Behavior-neutral. Validate with ThreadSanitizer over a clean
shutdown if available (before: data race; after: clean), else a clean-shutdown smoke.

**Consumer walk**: all 14 reads are `while(!shutdown_)` / `if(shutdown_)` predicates —
relaxed atomic load is drop-in. No ordering dependency on other state (it's a pure
termination flag). SAFE, independent.

---

## §8 Regression tests (one per fix, CLAUDE.md cross-layer-test rule)

All in-process synthetic-fire (no IONOS/RF), following the `--test-partial-bsi-advance`
/ `--test-climb-engine` / `--test-probe-backoff` pattern (CLI flag in `main.cc:~815`,
dispatch `main.cc:~1863` → `ARQ.test_X()`). Run in the FOREGROUND only (NEVER the full
`mercury.exe --test` — long Monte-Carlo, hangs background agents per MEMORY trap).

- **R029** `--test-retx-clear-on-recovery`: populate `retransmit_count>0` w/ a known
  OLD bsi; fire each §2.2 recovery trigger; assert `retransmit_count==0` AND the first
  post-recovery v2 mixbatch carries NO pre-recovery bsi.
- **R030** `--test-v2-pendingack-flip-alias`: build a v2 mixbatch with the array index
  space DIVERGED from wire positions (holes + retx prefix); run the post-TX flip; assert
  no FREE/foreign slot left PENDING_ACK and `nNAcked_data`/`nLost_data` not inflated.
- **R035** `--test-batch-shrink-strands-prev`: arm prev with `expected=15` (RECEIVED
  slots incl one in [10,15)); call `set_data_batch_size(10)` (NOT a direct assign);
  assert counts re-derived, gate reachable, and on orphan a single `streaming_reset`
  fires (pre-fix: prev never completes, stale-discard FREEs w/o reset).
- **R038** `--test-eob-poison-prev-retx`: deliver a short prev-batch EOB retransmit
  prefix while a lossy current batch's own EOB is lost; assert NO premature ACK-GATE
  PASS post-fix (pre-fix: early PASS + truncated delivery).
- **R039** `--test-sack-oow-reject`: inject a CRC8-valid SACK_RSP with `rx_bsi` outside
  `{cmd_bsi,prev_bsi}`; assert `[CMD-SACK-V2-OOW]` logged + bitmap NOT applied (pre-fix:
  applied → mis-ACK).
- **R006**: clean-shutdown smoke under TSan if available.

---

## §9 Cross-layer audit checklist (future changes to this cluster)

Before changing ANY of: the `retransmit_*` arrays / `retransmit_count`; the post-TX
PENDING_ACK flip (`arq_common.cc:3978`); the v2 mixbatch identity helper
(`arq_commander.cc:1446`); `set_data_batch_size()` (`arq_common.cc:644`);
`bump_bsi_and_transfer_prev()` (`arq_common.cc:4356`); the pre-routing EOB capture
(`arq_common.cc:6362`); the SACK-v2 accept arms (`arq_commander.cc:2814` /
`decode_sack_v2_frame`) — walk §2/§3/§4/§5 and UPDATE this document plus the relevant
companion (`data-flow-retx-queue.md`, `data-flow-messages_rx_prev.md`,
`data-flow-batch-size.md`, `data-flow-robust-tier-arq-batch.md`).

---

## §10 As-built (branch `fix/confirmed-races`, 2026-06-06)

All six fixes implemented + tested in worktree `C:/Users/kamer/mercury_wt/race-fixes`
(off `monitor` @2aff9e6), one commit each, in the §6 SAFE ORDER. ~~NOT merged.~~ Each
ships an in-process synthetic-fire `--test-*` with a fail-before/pass-after assertion;
full results in `race_audit/race_fix_results.json`.

> **CORRECTION 2026-07-01: ALL SIX MERGED to mainline.** Verified in the current
> `mercury/source/` tree: `clear_retx_queue()` (R029) is live at its single-owner sites
> (`arq_commander.cc:585/673/3394/4879/5217/5373/6090` + `reset_session_state`);
> `sack_v2_bsi_in_window()` (R039, PURE static in `arq.h`) is the live SACK-v2 accept
> guard; the R038 staging field / R035 `rescan_prev_on_batch_shrink()` / R030 flip
> resolver / R006 atomic-shutdown are all present. The §10.1 "no sibling bug" outcome
> held through the merge.

| race | commit | test flag | implementation note |
|---|---|---|---|
| R006 | 4d9fd88 | `--test-shutdown-atomic` | `_Atomic bool` (audit sketch) does NOT parse in `audioio.c` — that TU is compiled as **C++** (`build.sh:463`). Used `std::atomic<bool>` with **C language linkage** (`extern "C"`) in both TUs to match `main.cc`'s `extern "C"` symbol; links clean. |
| R039 | d774c4d | `--test-sack-oow-reject` | Added PURE static `sack_v2_bsi_in_window()` (arq.h) shared by the OFDM arm guard + the test; mirrors MFSK arm. MFSK arm left as-is. |
| R038 | 0daf597 | `--test-eob-poison-prev-retx` | Added per-frame staging field `rx_buffer_eob_seq` (arq.h §1.5). `receive()` stages for v2; the match-current storage block promotes to `last_received_end_of_batch_seq`. v1 keeps the direct `receive()` write (byte-identical). |
| R035 | 85f78c1 | `--test-batch-shrink-strands-prev` | Added `rescan_prev_on_batch_shrink()`, called from the `set_data_batch_size()` chokepoint in BOTH clamp branches before the store. Test drives the REAL chokepoint (not a direct-assign). |
| R029 | 940ad98 | `--test-retx-clear-on-recovery` | Added `clear_retx_queue()` (single owner). The audit listed `restore_tx_from_compressed` generically — it has **4 callers** (BREAK ACK-recovery, BREAK EXHAUSTED, gearshift FRAME-UP-DATA-FAILED, gearshift FRAME-UP); placing the clear at the top of that helper covers all 4 compression paths in one place, plus calls in the non-compressed else-branches, watchdog, both gearshift-down recoveries, and `reset_session_state()`. |
| R030 | ff973b4 | `--test-v2-pendingack-flip-alias` | MINIMAL fix (no new persistent structure → no R029 clear-site coupling). Promoted `v2_retx_prefix_count` to a member; added `v2_flip_resolve_slot()` (skip retx-prefix, route new-data by `(bsi,low7)`). KEY SAFETY FINDING: retx-prefix frames have NO live `messages_tx[]` slot at TX time — the slot is freed to ACKED at SACK capture (`arq_commander.cc:2983`) and the payload lives in `retx_scratch`; their delivery is SACK-bitmap-tracked, so SKIPPING them in the flip is correct. The `(bsi,low7)` match is provably UNIQUE via the pre-existing duplicate-tuple validation sweep (`arq_commander.cc:1797`). |

**§10.1 Sibling-bug outcome**: none. The audit's §5 per-fix consumer walk held for
all six — no fix exposed a sibling bug, no sketch proved wrong, nothing was blocked.
R038+R035 co-landed (R038 first) and are mutually reinforcing as predicted (the
`prev_expected` derivation at `bump_bsi_and_transfer_prev` now reads the genuine
current EOB). R029 landed before R030; R030's MINIMAL fix added no persistent state,
so `clear_retx_queue()` needed no extension.

**§10.2 Line-number note**: arq_common.cc line numbers above (and in §1–§9) drifted as
the fixes added code; the cited symbols/anchors are authoritative, not the numbers.

---

## §11 M6 — BREAK-path lossless requeue (`cmd_batch_seq_id` rollback)

**Branch** `feat/m6-lossless-requeue` (off `9afe802`). ~~**Default-OFF** behind
`MERCURY_BREAK_LOSSLESS_REQUEUE`; BYTE-IDENTICAL when unset. HELD (no merge/push/HW).~~

> **CORRECTION 2026-07-01: M6 LANDED in mainline and is now DEFAULT-ON.** The gate flipped
> from opt-in to an escape hatch: `break_lossless_requeue_enabled()` (`arq_commander.cc:62`)
> now returns TRUE unless `MERCURY_BREAK_LOSSLESS_REQUEUE_DISABLE` is set to any non-empty
> value (`arq_commander.cc:60-68`). The rollback fires at the Anchor-rung BREAK behind
> `break_lossless_requeue_enabled() && sack_v2_enabled && !compression_enabled`
> (`arq_commander.cc:~5564`). See `data-flow-retx-queue.md` §10 (same fix, its owner doc).

### §11.1 Root cause (the recovery cascade's terminal bug)
The Anchor-rung emergency BREAK (`arq_commander.cc`, the
`emergency_nack_count >= emergency_nack_threshold` block ending in `send_break_pattern()`
+ `return`) fires in every HW cascade cell. It sets `emergency_break_active=1`, sends the
BREAK pattern, and returns **WITHOUT rolling `cmd_batch_seq_id` back to the stranded
in-flight batch**. The stranded batch's bytes ARE preserved: the BREAK recovery handler
(`arq_commander.cc:288-296` ACK-phase / `:376-384` EXHAUSTED) FIFO-push-back + FREEs
`messages_tx[]` on recovery — but that handler does NOT touch `cmd_batch_seq_id`. So the
post-recovery re-send rebuilds the batch under the **already-advanced** epoch counter
(`cmd_batch_seq_id` had incremented past the in-flight bsi at `arq_commander.cc:1915` when
the batch was first dispatched). Meanwhile the RSP's BREAK self-heal
(`arq_responder.cc:474-475`) resets `rsp_current/prev_expected = -1` but the delivery
high-water `rsp_last_delivered_batch_seq_id` SURVIVES (INV-4). The next data frame
re-adopts via `sack_v2_readopt_has_gap(fresh_bsi, high-water)` (`arq.h:1395`): a fresh
epoch bsi is a `>=2` forward jump from the high-water → `[RSP-V2-GAP-ABORT]` → DROPPED →
the transfer terminally dies. **The GAP-ABORT is CORRECT (integrity guard — NOT weakened).
The bug is the BREAK leaving the hole.**

### §11.2 The fix (`arq_commander.cc`, Anchor-rung BREAK, immediately before
`send_break_pattern()`)
Gated `break_lossless_requeue_enabled() && sack_v2_enabled && !compression_enabled` (the
SAME preconditions FIX-9 D3 uses). PORTS the FIX-9 D3 capture+rollback verbatim: scan
`messages_tx[]` for the EARLIEST (mod-256) non-FREE `batch_seq_id` = the in-flight batch's
ORIGINAL bsi, then `cmd_batch_seq_id = min_inflight_bsi`. The recovery re-send then carries
the CONTIGUOUS bsi the RSP expects next → `sack_v2_readopt_has_gap` returns false → no
abort. The demote/BREAK mechanism itself is UNCHANGED; only the bsi bookkeeping is
corrected. Helper `break_lossless_requeue_enabled()` (`arq_commander.cc`, next to
`sack_rx_trace_enabled`): unset/`"0"` → 0 (disabled, byte-identical); any other non-empty
value → 1.

### §11.3 Producers of `cmd_batch_seq_id`
- `arq_commander.cc:1673,1690` — assign `cmd_batch_seq_id & 0xFF` to each new-data frame's
  `batch_seq_id` (the LABEL the wire carries).
- `arq_commander.cc:1915` — `cmd_batch_seq_id = (cmd_batch_seq_id + 1) & 0xFF` AFTER a batch
  with ≥1 new-data frame is dispatched (the advance that creates the epoch gap).
- `arq_commander.cc` FIX-9 D3 demote (`:4018`) — `cmd_batch_seq_id = min_inflight_bsi`
  (the EXISTING lossless rollback; the SET_CONFIG sibling of this fix).
- **NEW (M6)** — Anchor-rung BREAK path — `cmd_batch_seq_id = min_inflight_bsi`
  (default-off; the new producer this fix adds).
- Session reset paths set it to a base (e.g. `reset_session_state`); not relevant to the
  in-flight-strand window.

### §11.4 Consumers of `cmd_batch_seq_id` (every read, and why the rollback is safe)
1. **New-data frame bsi assignment** (`:1673,:1690`) — on the NEXT batch built post-BREAK.
   After rollback this reads `min_inflight_bsi`, so the re-sent batch carries the
   contiguous bsi. INTENDED — this IS the fix's effect.
2. **SACK-v2 accept window** `{cmd_bsi, prev_bsi}` (`:128`, `:2826`, `:3019`,
   `sack_v2_bsi_in_window`) — guards which incoming SACK_RSP frames are applied. After the
   BREAK the link re-baselines at ROBUST_0 and re-climbs; no SACK_RSP for the OLD epoch is
   in flight (the BREAK aborted the old PHY exchange), and any post-recovery SACK is for the
   rolled-back-and-re-sent batch, which matches the rolled-back window. The D3 path relies
   on the IDENTICAL property (it also rolls the counter back then re-emits). SAFE.
3. **Diagnostic prints** (`:128,:1400,:1825,:1841,:2684,…`) — display only. SAFE.
The recovery handler (`:219-396`) reads `messages_tx[]`, the FIFO, the config-ladder
fields, `break_drop_step`, `emergency_previous_config` — it does NOT read or write
`cmd_batch_seq_id`. So the rolled-back value set at the trigger SURVIVES untouched through
recovery into the re-send. VERIFIED by `grep cmd_batch_seq_id` over `:219-396` (no hits).

### §11.5 BREAK-vs-D3 equivalence (the §5 audit — is the rollback safe on the BREAK path?)
The ONE worry: the D3 lossless-demote premise (CASE 8) is that the in-flight CFG16 batch
was ALREADY DELIVERED by the RSP (high-water advanced past it) — a narrow reverse-ACK-
starvation scenario. Does the generic Anchor-rung BREAK share a safe premise? **Yes, and
the rollback is contiguous in BOTH sub-cases:**
- The Anchor BREAK fires on `emergency_nack_count >= threshold` = K consecutive block-
  failures with NO data-ACK (clean OR partial); ANY data-ACK resets the counter to 0
  (`arq_commander.cc:4184`). So the stranded in-flight batch is one for which the CMD got
  NO ACK. On the RSP it is in ONE of two states:
  - **(a) Delivered, ACK lost** (the reverse-ACK-starvation / D3-like case): high-water
    advanced TO the in-flight bsi. Re-presenting at `min_inflight_bsi == high-water` is a
    DUPLICATE → `sack_v2_readopt_has_gap(b, b)` = false (dedup, INV-4). SAFE.
  - **(b) Never delivered** (genuine forward-decode failure — the channel cratered): high-
    water did NOT advance; it sits at `min_inflight_bsi - 1`. Re-presenting at
    `min_inflight_bsi == high-water + 1` is the CONTIGUOUS successor →
    `sack_v2_readopt_has_gap` = false. SAFE.
- In BOTH sub-cases `min_inflight_bsi ∈ {high-water, high-water+1}` because the earliest
  un-ACKed batch in `messages_tx[]` IS, by the in-order ARQ invariant, the next batch the
  RSP needs after its contiguous-delivered high-water. This is the SAME invariant the D3
  rollback relies on; the BREAK path does not weaken it. The hole the GAP-ABORT catches is
  created ONLY by the fresh-epoch advance (skipping past `min_inflight_bsi`); rolling back
  removes exactly that skip.
- **Difference handled**: the D3 demote frees+pushes `messages_tx[]` AND rolls back at the
  SAME site (`:3968-4018`). The BREAK defers the free+push to the recovery handler
  (`:288-296/:376-384`). M6 captures `min_inflight_bsi` at the trigger (where
  `messages_tx[]` still hold the stranded batch, BEFORE the recovery frees them) and rolls
  back there; the deferred free+push is bsi-agnostic, so the split is benign. NO consumer
  invariant breaks.
- **Compression path**: gated OUT (`!compression_enabled`). `restore_tx_from_compressed()`
  owns its own re-stage; unchanged. (Same scoping as D3.)

### §11.6 Test (extends `--test-inorder-demote` CASE 1)
`MERCURY_BREAK_LOSSLESS_REQUEUE` selector added to `test_inorder_demote()`. CASE 1 (BREAK
cur=-1 re-adopt) now models both arms, driving the REAL `sack_v2_readopt_has_gap` +
delivery commit + `fifo_buffer_rx`:
- **fail-before** (knob unset): the recovery re-sends fresh-epoch bsi=8 →
  `sack_v2_readopt_has_gap(8, high-water=4)` = true → GAP-ABORT, DROPPED, delivered EXACTLY
  [0..4] (the pre-M6 behavior + FIX-8 regression guard — byte-identical).
- **pass-after** (`MERCURY_BREAK_LOSSLESS_REQUEUE=1`): the BREAK rolled `cmd_batch_seq_id`
  back to the earliest in-flight bsi (5 = high-water+1) → recovery re-sends contiguous bsi=5
  → `sack_v2_readopt_has_gap(5, 4)` = false → NO abort, batch 5 delivered, [0..5] in-order
  (192B), link CONNECTED.
`MERCURY_GAP_ABORT_DEFEAT` (the integrity-guard silent-concat fail-before) is independent
and preserved. The integrity battery (`--test-gap-abort`, `--test-partial-bsi-advance`,
`--test-inorder-demote` knob-unset) stays green.

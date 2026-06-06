# Data-Flow Audit: big-block as ONE ARQ unit (P2 re-granularization)

**Status**: P2.1/2.2/2.4/2.5/2.6 WIRED (P2.0 draft + this build), authoritative on
`feat/bigblock-livepath-p2` (built off `feat/bigblock-livepath-p1` @ `0e94581`).
Paired with the in-process regression `--test-bigblock-arq-unit`
(`source/datalink_layer/test_bigblock_arq_unit.cc`) — now **PASS 3/3** (was FAIL 0/3
at P2.0). Every change to the structures in §0 — or to any predicate that gates a
write — MUST update this document (CLAUDE.md §"Cross-Layer Data-Flow Audits").

**P2 BUILD STATUS (this session, see §9):**
- **P2.4/2.5/2.6 (ARQ batch/SACK granularization)** — DONE. `bigblock_block_to_arq()`
  carves cw_ok→messages_rx (RECEIVED iff cw_ok[c]==1), sets synthetic EOB=K-1 BEFORE
  any prev-sizing (INV-4/RISK-4), one-ACK + single-bsi on a clean block (INV-1), and
  selective-repeat of EXACTLY the clear-bit sub-codewords via the stock CFG16 per-frame
  retx queue (INV-3, retransmit_count==popcount). Regression `--test-bigblock-arq-unit`
  flips FAIL 0/3 → PASS 3/3.
- **P2.1 (feed real ARQ bytes)** — DONE (TX side). `transmit_bigblock` now packs the K
  codewords from REAL ARQ bytes (data/nBytes → payload_bits, LSB-first byte_to_bit) when
  a payload is handed; nBytes==0 keeps the seeded-PRBS known payload so the byte-correct
  loopback gate is unchanged (8/8). The RX carve of the decoded info bits into K
  byte-sub-units is the ARQ caller's step (consumed by `bigblock_block_to_arq`'s
  tx_payload arg, exercised by the unit test). The full live ARQ↔PHY send/receive
  integration (replacing the per-frame send loop with the block entry) is the production
  wiring validated at P3 (HW / Option-b pump).
- **P2.2 (normalization-bypass)** — DONE (the stock normalization is now run on the
  big-block RX). Factored the EXACT stock receive_byte RX passband normalization +
  impulse-blanking (was inline at telecom_system.cc:1081-1124) into
  `rx_passband_normalize_and_blank()`; called from BOTH receive_byte (byte-identical
  extraction, stock path unchanged) and receive_bigblock (BEFORE the estimator). INV-8.
  **RESIDUAL FINDING (NOT P2.2):** the live AWGN validator STILL fails at ≤~40 dB Es/N0
  (deterministic, identical BER 0.43 across runs; clean PASS, 60/45 dB PASS, 35/30 dB
  FAIL). This is a SEPARATE, PRE-EXISTING P1 PHY scaling/LLR bug in the big-block
  estimator path (the ~25 dB implementation loss is not a true noncoherent SNR cliff and
  is NOT masked by the normalization). It is OUT of the ARQ-granularization scope, does
  NOT affect the in-process ARQ unit test (INV-8: the test runs on the clean carve, no
  channel), and is GATED before any noisy/HW test (P3). Per CLAUDE.md §1/§2 it needs a
  proper DSP root-cause investigation (research before guessing; no threshold-tuning to
  mask), NOT a shotgun fix — flagged for P3 PHY work.
- **P2.3 (hot-path realloc move)** — DEFERRED to P3 (validation-gated). The plan's
  RISK-1 fear (the per-block `bigblock_restore_stock_config()` full reload reallocating
  the `data_container` buffers the ARQ aliases via `messages_tx.data`) is **UNFOUNDED**:
  `telecom_system.cc` has ZERO references to `messages_tx` — `load_configuration`
  reallocates the PHY `data_container` (sample buffers), NOT the ARQ message queues
  (those are owned by `cl_arq_controller::init_messages_buffers`, never touched by the
  PHY reload). So there is no per-block ARQ-alias corruption. The residual cost is PURELY
  performance (per-block OFDM deinit/init) and matters only for a SUSTAINED multi-block
  rate, which is single-batch-limited in the in-process sim (§8 [?]) and thus needs P3
  HW / the Option-b pump to validate the move — moving the reload to config-load now
  would be an UNTESTED structural change (CLAUDE.md §3). Deferred with the canary plan in
  §8 [?].

**Scope of this audit**: P2 changes the ARQ *data-unit granularity* at the
CFG16-bigblock rung from **"one frame = one preamble = one ACK"** to
**"one big-block = one acquisition = one ACK over K=8 sub-codewords, with
selective-repeat per sub-codeword."** P2 touches PHY-feed (P2.1/2.2/2.3) and ARQ
batch/SACK (P2.4/2.5/2.6) ONLY. It does NOT touch the optimizer
(`optimizer_is_in_control()` `arq.h:2041-2058`, `last_data_viable_config`
`:2086`, `anchor_consec_break_fails` `:2106`, `probe_backoff` `:2031`). The
gearshift's only role is electing the `bigblock_framing_enabled` flag at the top
rung.

**P1 state we build on** (`0e94581`): the validated big-block PHY is in the LIVE
`transmit_byte`/`receive_byte` (`telecom_system.cc:608` / `:985`), gated on
`bigblock_framing_enabled` (`telecom_system.h:532`, default OFF). The RX worker
`bigblock_rx_passband()` exposes a per-codeword clean vector `cw_ok_out`
(`telecom_system.cc:7287/7302`, stashed to `bigblock_last_rx_cw_ok`
`telecom_system.cc:7862`) = the **K-bit SACK granularity** P2 consumes. K =
`nBits/ldpc.N` from the thin grid (`telecom_system.cc:7285`), default 8 (capped
by `MERCURY_BIGBLOCK_K`). 8/8 byte-correct on the live clean path; ZERO stock
CFG15/16 regression (byte-identical A/B). NO ARQ-layer big-block entry point
exists yet (`grep bigblock source/datalink_layer/` = empty as of P1) — that gap
is exactly what P2 fills.

---

## §0 Structures this audit covers

| structure | decl | role in P2 |
|---|---|---|
| `messages_tx[]` | `arq.h:2495` | CMD TX frame queue. ARQ payload bytes the block carries (P2.1 feed). |
| `messages_batch_tx[]` | `arq.h:1882` | the per-frame batch the send loop iterates (`arq_common.cc:3911`). P2.4 replaces the loop with ONE block entry. |
| `messages_rx[]` | `arq.h:2496` | RSP current-batch RX storage. P2.4 carves the K decoded sub-units into K slots. |
| `messages_rx_prev[]` | `arq.h:1652` | RSP prev-batch storage for late retransmits (see `data-flow-messages_rx_prev.md`). |
| `batch_seq_id` (per-frame) | `arq.h:239` | wire bsi each frame advertises. |
| `cmd_batch_seq_id` | `arq.h:1502` | CMD next-new-data-batch counter (mod 256). ONE block = ONE bump. |
| `rsp_current_expected_batch_seq_id` | `arq.h:1524` | RSP expected current bsi. Adoption + partial bump fire ONCE per block. |
| `rsp_prev_batch_seq_id / _active / _received_count / _expected_count` | `arq.h:1530/1662/1671/1675` | prev-batch state machine. RISK-4. |
| the SACK bitmap (`bool[]` / `uint32`) | `arq_responder.cc:801`, `arq_commander.cc:131` | clean-batch all-ones target + partial gaps. P2: the K-bit `cw_ok` IS the bitmap. |
| end-of-batch flag `last_received_end_of_batch_seq` | `arq.h:2520` | EOB seq# (bug #58). P2: set SYNTHETICALLY to K-1 at block decode (RISK-4). |
| `block_under_tx` | `arq.h:1443` | TX block-in-flight marker. |
| `data_batch_size` | `arq.h:1435` | batch size; at the bigblock rung the RSP sizes batch = K (see `data-flow-batch-size.md`). |
| `bigblock_last_rx_cw_ok` (PHY) | `telecom_system.h:595` | the K-bit per-codeword clean vector the ARQ SACK reads. |

---

## §1 Producers — every code path that WRITES the structures

### 1.1 `messages_tx[]` — CMD TX payload
- `process_messages_tx_data()` (`arq_commander.cc:1138`) fills `messages_tx[i]`
  from the application FIFO, iterating `i<data_batch_size`.
- ctor/`init_messages_buffers()` (`arq_common.cc:~1545`): each slot
  `status=FREE, length=0, data=<N_MAX/8 buf>, batch_seq_id=-1`.
- **P2.1 NEW**: `transmit_bigblock(data,nBytes,out)` currently IGNORES
  `data/nBytes` and emits seeded PRBS (`telecom_system.cc:7795 (void)data`). P2.1
  wires `bigblock_tx_passband(...,payload_bits)` so the block carries K=8
  codewords of the ACTUAL `messages_tx` bytes. **PRODUCER CHANGE: the block's
  systematic info bits now come from `messages_tx`, not PRBS.** No change to
  `messages_tx` ITSELF — P2.1 only READS it (so it is a consumer of
  `messages_tx`, listed in §2.1; the producer entry here is the PHY input buffer).

### 1.2 `messages_rx[]` — RSP RX storage
- New-data path `add_message_rx_data()` (`arq_responder.cc:54`): bound
  `loc < data_batch_size`, writes one frame per `receive_byte`.
- **P2.4 NEW**: at the bigblock rung, ONE `receive_bigblock` decode produces K
  info-bit sub-units (`telecom_system.cc:7858-7859 out[]`). P2.4 carves them into
  `messages_rx[0..K-1]` in ONE call (replaces the K separate `receive_byte` +
  `add_message_rx_data` writes). **PRODUCER CHANGE: K slots written per block,
  not one per frame.** The per-slot invariants (§4) are unchanged — each carved
  sub-unit still sets `status, length, id, sequence_number, batch_seq_id` exactly
  as the per-frame path does.

### 1.3 `last_received_end_of_batch_seq` — EOB
- Per-frame decode sets it from wire bit-7 (`arq_common.cc:6639`:
  `last_received_end_of_batch_seq = message_TxRx_byte_buffer[2] & 0x7F`).
- Cleared to -1 at `arq_common.cc:604`, `arq_responder.cc:1624/1653/1814`,
  `arq_commander.cc:4629`.
- **P2.4 NEW**: the block has ONE acquisition and no per-frame bit-7 wire byte —
  so P2.4 sets `last_received_end_of_batch_seq = K-1` SYNTHETICALLY at block
  decode. **RISK-4: this MUST be set BEFORE the partial-SACK prev-transfer branch
  (`arq_responder.cc:912-925`) and before `bump_bsi_and_transfer_prev()` reads it
  (`arq_common.cc:4548`), or `rsp_prev_batch_expected_count` sizes from a stale -1
  (`arq.h:1675-1678`) → the prev-batch never completes** (regression case 3).

### 1.4 `cmd_batch_seq_id` — CMD new-data bsi counter
- Bumped once per NEW-DATA batch build (`arq_commander.cc`, the new-data-batch
  builder). Retransmits carry their ORIGINAL bsi
  (`retransmit_frame_batch_seq_ids[]` `arq.h:1481`), NOT the live counter.
- **P2.6**: ONE block = ONE new-data batch → ONE `cmd_batch_seq_id` bump per
  block. UNCHANGED mechanism (a block IS one batch); the only delta is that the
  batch now contains K sub-codewords instead of `data_batch_size` per-frame
  frames.

### 1.5 `rsp_current_expected_batch_seq_id` + prev-batch counters — `bump_bsi_and_transfer_prev()`
- `bump_bsi_and_transfer_prev()` (`arq_common.cc:4520`): on batch seal, transfers
  `messages_rx → messages_rx_prev`, sets `rsp_prev_batch_seq_id = current`,
  bumps `rsp_current_expected_batch_seq_id = (current+1)&0xFF`, sets
  `rsp_prev_batch_active=true`, `received_count=xferred_received`,
  `expected_count=prev_expected` where
  `prev_expected = min(data_batch_size, last_received_end_of_batch_seq+1, nMessages)`
  (`arq_common.cc:4547-4554`). Early-returns unless
  `sack_v2_enabled && rsp_current_expected_batch_seq_id>=0` (`:4522`).
- Live retransmit storage into prev (`arq_responder.cc:484-507`, R7-bounded
  `loc<data_batch_size`).
- **P2.6**: at the bigblock rung this helper fires ONCE per block (not per
  frame). Its inputs (`data_batch_size`=K, `last_received_end_of_batch_seq`=K-1)
  come from P2.4. **No change to the helper body** — P2 only changes WHEN it
  fires (once per block) and the K-sized inputs.

### 1.6 `block_under_tx`
- TX-in-flight marker, set when a block goes on the wire, cleared on ACK.
- **P2.4**: at the bigblock rung, set once per block TX (one acquisition), not
  per frame. UNCHANGED semantics (it already marks "a block is on the wire").

### 1.7 the SACK bitmap
- RSP clean emit: `bitmap_u32 = (1u<<data_batch_size)-1u`
  (`arq_responder.cc:801/1711`). CMD all-ones target:
  `all_ones = (data_batch_size>=32)?0xFFFFFFFF:(1u<<data_batch_size)-1u`
  (`arq_commander.cc:131/2518`).
- **P2.4 NEW**: at the bigblock rung the K-bit `bigblock_last_rx_cw_ok`
  (`telecom_system.h:595`) IS the bitmap. Clean block ⇒ all K bits set ⇒ matches
  the CMD all-ones target `(1<<K)-1`. One bad codeword ⇒ exactly that bit clear ⇒
  partial SACK. **PRODUCER CHANGE: the bitmap source is the PHY cw_ok vector, not
  the per-frame RECEIVED scan.** The CMD all-ones target must use K (= the block's
  codeword count, which the RSP sizes `data_batch_size = K`, so the existing
  `(1<<data_batch_size)-1` formula already yields `(1<<K)-1` — symmetry holds via
  `data_batch_size==K` on both sides; see `data-flow-batch-size.md` §1
  invariant).

### 1.8 `data_batch_size`
- Sole production setter `set_data_batch_size()` (`arq_common.cc:561`); all
  producers in `data-flow-batch-size.md` §2.
- **P2.4**: at the bigblock rung the RSP sets `data_batch_size = K` so the
  all-ones target `(1<<K)-1` matches the K-bit cw_ok bitmap. CMD likewise. The
  CMD==RSP symmetry invariant (`data-flow-batch-size.md` §1) is the load-bearing
  one and is PRESERVED (both elect K at the same rung).

---

## §2 Consumers — every code path that READS the structures

### 2.1 `messages_tx[]`
- The send loop `arq_common.cc:3911-3929` struct-copies `messages_tx[i]` into
  `messages_batch_tx[i]` (which ALIASES `messages_tx[i].data`, `arq.h:1465-1477`)
  then `transmit_byte`s each. **P2.1 reads `messages_tx[0..K-1].data` to feed the
  block payload.** RISK-3: `transmit_bigblock`/`bigblock_restore_stock_config`
  does a FULL `load_configuration` reload per call (`telecom_system.cc:7807`),
  which REALLOCATES the `data_container` buffers the ARQ aliases via
  `messages_tx.data` (`arq.h:1465-1477`) and clobbers `receive_stats`. P2.3 moves
  the rebuild/reload OUT of per-block transmit/receive into config-LOAD (once per
  framing election) and config-unload — so a multi-block session does NOT
  deinit/init per block.

### 2.2 `messages_rx[]`
- ACK-GATE `process_messages_acknowledging_data` counts RECEIVED slots
  `i<data_batch_size` (`arq_responder.cc:~907`), `copy_data_to_buffer()` delivers
  ACKED slots up to `data_batch_size`. **P2.4**: the K carved sub-units are
  RECEIVED in `messages_rx[0..K-1]`; the gate counts K → clean block ⇒ K/K ⇒
  one ACK. Consumer UNCHANGED (it already counts `data_batch_size`=K slots).

### 2.3 `last_received_end_of_batch_seq`
- ACK-GATE expected-count: `eob = last_received_end_of_batch_seq+1; if(eob<expected) expected=eob`
  (`arq_responder.cc:1021-1025`-style, also `arq_common.cc:4548-4552`). The
  EOB-fast-path SACK turnaround branch (`arq_responder.cc:912`) gates on
  `last_received_end_of_batch_seq >= 0`.
- `bump_bsi_and_transfer_prev()` reads it to size `prev_expected`
  (`arq_common.cc:4548`). **RISK-4 consumer**: if P2.4 sets it AFTER the prev
  branch, the consumer reads -1 → `prev_expected = data_batch_size` (not K-1+1=K
  — equal IF data_batch_size==K, but if data_batch_size>K the prev sizes too
  large and never completes). P2.4 sets it to K-1 BEFORE both consumers.

### 2.4 `rsp_current_expected_batch_seq_id` + prev counters
- bsi-routing of incoming frames: `match_current = (abs_bsi == rsp_current_expected_batch_seq_id)`,
  `match_prev = (abs_bsi == rsp_prev_batch_seq_id)` (`arq_responder.cc:332-441`).
  Non-match ⇒ `[RSP-V2-DROP] out_of_window`. **P2.5 consumer**: selective-repeat
  retx frames are STOCK CFG16 per-frame frames carrying their ORIGINAL bsi → they
  route via `match_current`/`match_prev` exactly as today (RISK-5: a session mixes
  big-block new-data + per-frame retx; the RX capture loop already handles
  per-frame preambles, so stock-framed retx is transparent).
- prev completion gate `received_count >= expected_count`
  (`arq_responder.cc:526-573`). **P2.6 consumer**: fires once per block.

### 2.5 the SACK bitmap
- CMD `sack_clean_confirmation_accepted()` (`arq_commander.cc:131`) compares
  `rx_bitmap == all_ones`. **P2.4/2.5 consumer**: CMD reads the K-bit bitmap; all
  K set ⇒ clean ⇒ block ACKed; else the clear bits select the failed
  sub-codewords for `retransmit_frames[]` (`arq.h:1463`).

### 2.6 `cmd_batch_seq_id`
- Read when building a new-data batch (stamps each frame's `batch_seq_id`) and
  snapshotted into `cmd_sack_v2_last_rx_batch_seq_id`. **P2.6 consumer**:
  read once per block.

### 2.7 `data_batch_size`
- Full consumer list in `data-flow-batch-size.md` §3. The
  divergence-sensitive one is §3.1 (the all-ones target). **P2.4**: K must equal
  `data_batch_size` on BOTH sides at the bigblock rung (the §1 invariant).

---

## §3 Valid states — especially BEFORE any producer writes

Default-init values that bite (CLAUDE.md §5.3):

| structure | default / pre-write value | bite |
|---|---|---|
| `messages_tx[i].status` | `FREE` | block feed must skip FREE slots / size to filled count. |
| `messages_rx[i].batch_seq_id` | `-1` (unset) | carved sub-units MUST stamp the block's bsi or they route as out_of_window. |
| `cmd_batch_seq_id` | ctor (`arq_common.cc:281`-region) | first block must bump from a valid base. |
| `rsp_current_expected_batch_seq_id` | ctor; adopted at first DATA | first block adoption fires ONCE. |
| `last_received_end_of_batch_seq` | **-1** | **RISK-4**: if P2.4 leaves it -1 at the prev branch, `prev_expected` mis-sizes. MUST be K-1 before the branch. |
| `rsp_prev_batch_expected_count` | `0` (`arq_common.cc:381/2127`) | sized from `last_received_end_of_batch_seq+1`; stale -1 ⇒ `data_batch_size` not K. |
| `rsp_prev_batch_active` | `false` | prev path dead until first bump. |
| `bigblock_last_rx_cw_ok` | empty vector | sized to K only after a `receive_bigblock`; ARQ must not read it before a decode. |
| `data_batch_size` | ctor 1 (`arq_common.cc:146`) | must be elected to K at the bigblock rung on BOTH sides. |
| `block_under_tx` | 0 | one block sets it once. |

**The block-decode valid-state enumeration** (the new state machine P2 adds):

| block state | cw_ok | messages_rx[0..K-1] | EOB | bsi | valid? |
|---|---|---|---|---|---|
| clean K=8 | all-ones (K bits set) | K RECEIVED | K-1 (synthetic) | bumped ONCE | ✓ case 1 |
| one bad codeword | one bit clear | K-1 RECEIVED, 1 FREE | K-1 | NOT bumped (partial) | ✓ case 2 |
| lost-EOB (no synthetic set) | (any) | (any) | **-1 (stale)** | prev mis-sizes | INVALID — RISK-4 (case 3 guards) |
| mixed new-data + stock retx | new-data via block, retx via match_current/prev | both populated | K-1 then per-frame | one bump for block | ✓ RISK-5 |

---

## §4 Invariants the consumers assume (verify each producer maintains them)

**INV-1 (one ACK per block)**: a clean K=8 block produces exactly ONE ACK with an
all-ones K-bit bitmap, and `cmd_batch_seq_id`/`rsp_current_expected_batch_seq_id`
bump exactly ONCE. *Producer obligation (P2.4/2.6)*: the block-decode path must
populate K RECEIVED slots and fire `bump_bsi_and_transfer_prev()` once — NOT K
times. **Test case 1 asserts this.**

**INV-2 (K-bit SACK = cw_ok)**: the per-codeword clean vector
`bigblock_last_rx_cw_ok` (length K) IS the SACK bitmap. A clear bit at index c ⇔
sub-codeword c failed ⇔ `messages_rx[c].status != RECEIVED`. *Producer obligation
(P2.4)*: carve cw_ok into messages_rx[] 1:1 (RECEIVED iff cw_ok[c]==1).
**Test case 2 asserts this.**

**INV-3 (selective-repeat is per-sub-codeword)**: one bad codeword retransmits
EXACTLY that codeword (via STOCK CFG16 per-frame framing into
`retransmit_frames[]` carrying the ORIGINAL bsi), NOT the whole block. *Producer
obligation (P2.5)*: the K-bit SACK selects only the clear bits;
`retransmit_count == popcount(~cw_ok & ((1<<K)-1))`. **Test case 2 asserts
exactly-one retx.**

**INV-4 (synthetic EOB ordering — RISK-4)**: `last_received_end_of_batch_seq` is
K-1 at block decode and is set BEFORE the partial-SACK prev-transfer branch
(`arq_responder.cc:912-925`) and before `bump_bsi_and_transfer_prev()`
(`arq_common.cc:4548`). *Consumer reliance*: `rsp_prev_batch_expected_count` =
`min(data_batch_size, EOB+1)` = K (since data_batch_size==K). If EOB is stale -1,
`prev_expected = data_batch_size` — equal to K only if data_batch_size==K;
the test sizes the RSP batch=K so the lost-EOB case is detectable. **Test case 3
asserts the prev sizes to K and the prev batch COMPLETES.**

**INV-5 (CMD==RSP batch symmetry at the rung)**: `data_batch_size == K` on BOTH
sides (the load-bearing invariant from `data-flow-batch-size.md` §1) so the
all-ones target `(1<<K)-1` matches the emitted bitmap. *Producer obligation*: both
sides elect K at the bigblock rung. **All three test cases run with
data_batch_size==K.**

**INV-6 (RX delivered == TX bytes at every transition)**: after every state
transition (clean ACK, partial SACK + retx fill, lost-EOB recovery) the bytes
delivered to the application equal the bytes the TX block carried. *Producer
obligation*: P2.1's carve (RX) is the exact inverse of P2.1's pack (TX). **All
three cases assert delivered == TX bytes.** This is the CLAUDE.md
"RX state matches TX state at every transition" requirement.

**INV-7 (no per-block deinit/init — RISK-3)**: a multi-block session must NOT call
`load_configuration` per block (it reallocates `data_container` buffers aliased by
`messages_tx.data` and clobbers `receive_stats`). *Producer obligation (P2.3)*:
the thin-lattice rebuild + stock restore live in config-load/unload (once per
framing election), not in `transmit_bigblock`/`receive_bigblock`. **Asserted by
P2.3's own check; the unit test documents the requirement but the multi-block
alias-survival is a HW/sustained-rate concern (P3), see §6.**

**INV-8 (normalization not bypassed — P2.2)**: the captured passband is routed
through the stock RX passband normalization + impulse blanking BEFORE the
big-block estimator. *Producer obligation (P2.2)*: PROVEN broken today — the live
AWGN validator FAILS at 30 dB (BER 0.43) because `bigblock_rx_passband` skips
`receive_byte`'s normalization. The clean 8/8 gate is unaffected; the path is
non-functional on any real channel until fixed. **This is a PHY-feed invariant,
not an ARQ-state one — the in-process ARQ unit test runs on the clean carve (no
channel), so it does not depend on INV-8; INV-8 is gated before any noisy/HW
test (P2.2, mandatory).**

---

## §5 What my fix changes (P2) — walk every consumer

Per CLAUDE.md §5.5, for each altered assumption, walk every consumer:

1. **Data unit: per-frame → per-block.** Alters the producer of `messages_rx[]`
   (§1.2: K slots per block) and `messages_batch_tx[]` (§2.1: one block entry
   replaces the K-frame loop). Consumers — ACK-GATE (§2.2), `copy_data_to_buffer`
   — already iterate `data_batch_size`=K slots, so they are UNCHANGED. ✓
2. **SACK source: RECEIVED-scan → cw_ok vector.** Alters §1.7. Consumer
   `sack_clean_confirmation_accepted` (§2.5) compares against `(1<<K)-1`; with
   data_batch_size==K (INV-5) the all-ones formula already yields `(1<<K)-1`. ✓
3. **EOB: wire bit-7 → synthetic K-1.** Alters §1.3. Consumers §2.3
   (`bump_bsi_and_transfer_prev` sizing, the ACK-GATE expected-count, the
   EOB-fast-path turnaround). RISK-4: ordering — set BEFORE the prev branch.
   Verified by test case 3. ✓
4. **bsi bump cadence: per-frame seal → per-block seal.** Alters §1.4/1.5.
   Consumers §2.4 (bsi-routing of subsequent arrivals) and §2.6 unchanged in
   mechanism; only fires once per block. ✓
5. **Retx framing: block → STOCK CFG16 per-frame (P2.5).** Does NOT alter the
   retx queue (`retransmit_frames[]` `arq.h:1463`), `messages_rx_prev` routing,
   or bsi routing — retx is byte-UNCHANGED stock framing. Only NEW-DATA uses
   big-block framing. The RX capture loop already handles per-frame preambles
   (RISK-5 transparent). ✓ — this is the explicit de-risk: one bad codeword does
   NOT force a whole-block resend.

**Structures explicitly NOT changed by P2**: `optimizer_is_in_control()`
(`arq.h:2041-2058`), `last_data_viable_config` (`:2086`),
`anchor_consec_break_fails` (`:2106`), `probe_backoff` (`:2031`). The gearshift
only elects the framing flag at the top rung (no optimizer-state change).

---

## §6 Paired regression test (CLAUDE.md §"Cross-layer regression tests")

`--test-bigblock-arq-unit` (`cl_arq_controller::test_bigblock_arq_unit`,
`source/datalink_layer/test_bigblock_arq_unit.cc`), following the
`test_partial_bsi_advance` synthetic-fire pattern (in-process, no DSP/IONOS/RF).
THREE cases, each asserting RX delivered bytes == TX bytes at every transition:

- **Case 1 (clean K=8 block → one ACK)**: an all-ones K-bit cw_ok must produce
  ONE ACK, an all-ones K-bit bitmap, and bump `cmd_batch_seq_id` /
  `rsp_current_expected_batch_seq_id` exactly ONCE. (INV-1, INV-2, INV-6.)
- **Case 2 (one-bad-codeword → partial K-bit SACK + selective-repeat)**: a cw_ok
  with exactly one clear bit must yield a partial SACK with exactly that bit
  clear, retransmit EXACTLY that one codeword via STOCK CFG16 per-frame retx +
  `messages_rx_prev`, and after the retx fill deliver K/K bytes. (INV-2, INV-3,
  INV-6.)
- **Case 3 (lost-EOB → synthetic EOB=K-1 holds)**: with the wire EOB absent, the
  block decode sets `last_received_end_of_batch_seq = K-1` synthetically BEFORE
  the prev branch; the RSP sizes batch = K and `rsp_prev_batch_expected_count`
  = K; the prev batch COMPLETES. (INV-4, INV-5, INV-6 + RISK-4.)

**Fail-before / pass-after discipline**: the test drives the production block→ARQ
entry `cl_arq_controller::bigblock_block_to_arq()`. At P2.0 that entry is a
one-line STUB returning `BIGBLOCK_ARQ_NOT_WIRED` (NO ARQ logic — it populates
nothing). With nothing populated, all three cases' post-conditions fail (no
RECEIVED slots, no bsi bump, no delivered bytes), so **the test FAILS before P2
wiring (rc=1)**. P2.4/2.5/2.6 replace the stub body with the real block→ARQ
carve + SACK + bsi-once logic → the post-conditions hold → **the test PASSES**.
This makes the wiring bisectable: the stub commit (P2.0) is the fail-before
anchor; the wiring commit (P2.x) flips it to pass.

**What the in-process unit test CANNOT cover** (deferred to P3 HW /
the Option-b sustained pump): a SUSTAINED multi-block delivered RATE (one block ≈
1.4 KB; a Winlink message is multi-block). The in-process 2-instance sim's pump
may be single-batch-limited. INV-7 (no per-block deinit/init) and INV-8
(normalization bypass) are multi-block / channel concerns — INV-8 is gated before
ANY noisy test (P2.2); the sustained rate + the compress-ON VARA-parity verdict
are P3.

### §6a SACK-GATE extension (T4/T6/T7/T8/T9) — the R-B close-out gate

The original 3 cases proved the carve mechanics but HARDCODED `data_batch_size = K`
in each case setup, so a CMD/RSP batch-size DIVERGENCE (bug #9 / R-B) would PASS the
unit test and still REPRODUCE the 4-wire-failures on the wire. The SACK-GATE
extension (this session, `feat/bigblock-sack-gate-p1`) closes that gap. Now 8/8:

- **T2/T3/lost-EOB** — the original CASE1/CASE2/CASE3 (unchanged).
- **T4 (multi non-contiguous bad {1,4,6})** — popcount fidelity the single-bad CASE2
  cannot reach: cw_ok clear at {1,4,6} ⇒ the RECEIVED-scan SACK bitmap reads `0xAD`
  (the LSB-first pack the production producer `arq_responder.cc:1582-1594` emits) and
  the retx queue holds EXACTLY 3 frames at positions {1,4,6} carrying the ORIGINAL
  bsi (INV-2/INV-3, `retransmit_count == popcount`). bsi does NOT bump (partial).
- **T6 (THE #9 GATE — election symmetry vs the PRODUCTION setter)** — the GO/NO-GO.
  Builds TWO independent `cl_telecom_system` + `cl_arq_controller` (CMD + RSP), loads
  a REAL CFG16 grid into each (which on its own elects `data_batch_size = 25` via the
  30s formula — the bug-#9 SEED), turns on `bigblock_framing_enabled`, and runs the
  PRODUCTION `sack_negotiated_recompute_batch()` on BOTH (`"CMD"` = TEST_CONNECTION_ACK
  path, `"RSP"` = TEST_CONNECTION path — the SAME shared body). ASSERTS both elect
  `data_batch_size == K == 8` (from the SAME geometry source `bigblock_codeword_count()`,
  NOT hardcoded — `MERCURY_BIGBLOCK_K` pins the cap to 8 deterministically), both derive
  the identical `all_ones = (1<<data_batch_size)-1 == 0xFF` (the exact
  `cmd_clean_data_ack_crc_valid:136-139` expression), and the RSP's `0xFF` is accepted
  (`rx_bitmap == all_ones` AND `sack_clean_confirmation_accepted()`). Observed:
  `batch=25 -> pinned 8` on BOTH peers. FAIL-BEFORE proven: reverting the R-B pin
  (`bigblock_rung=false`) elects 25 on both ⇒ `all_ones=0x1FFFFFF != 0xFF` ⇒ T6 FAILS
  7/8. This is the divergence-proof property the climb-fix family lacked. (INV-5.)
- **T7 (#12 silence / no all-ones bypass)** — a `0xFF` is NOT credited without a valid
  CRC12. Drives the production accept gate's CRC predicate
  (`cmd_clean_data_ack_crc_valid:117-124`): a forged/silence `0xFF` with the wrong
  CRC12 is rejected BEFORE the all-ones comparison; the same bytes with the correct
  CRC12 pass. There is no all-ones bypass.
- **T8 (synthetic-EOB sizes prev at batch > K — the DIVERGENT RISK-4)** — CASE3 sized
  `data_batch_size == K` so a stale-EOB bug would yield the SAME number and pass
  silently. T8 sets `data_batch_size = 25 (> K)` BEFORE the decode and asserts the
  synthetic `EOB=K-1` drives `rsp_prev_batch_expected_count == EOB+1 == 8`, NOT the
  batch default 25 (INV-4 with a divergent batch).
- **T9 (padded-slot vs real-loss)** — drives the padded-slot guard predicate
  (`arq_commander.cc:2945-2952`): a GENUINELY FILLED clear-bit slot (length>0, bsi>=0,
  type!=NONE) MUST enqueue a retx; a PADDED slot (length==0 / bsi<0 / type==NONE) is
  swallowed (ACKED, no retx). Guards a future guard-broadening that would swallow real
  big-block sub-codeword losses.
- **T5 (#11 majority-vote)** — NOT a new addition: the SNR-tone 3/8-majority +
  2-vote-margin decode lives in the MFSK ctrl codec (`telecom_system.cc:3408`,
  covered by `mfsk_ctrl_codec_tests.cc`); the bitmap producer fidelity (the LSB-first
  RECEIVED-scan pack, `arq_responder.cc:1582-1594`) is exercised by T4. T5 is the
  existing-coverage line, not a re-implemented tone encoder.
- **T10 (mixed-session RF demux)** — explicitly DEFERRED to P3 (full RF demux of
  per-frame retx vs the next block's single acquisition needs HW / the Option-b pump;
  see §8 [?] RISK-5). The in-process routing assertions are covered by T4's
  position/bsi checks.

**R-B CLOSED (grep-verified):** the `set_data_batch_size(K) + nominal_batch_size = K`
pin is INSIDE the single `cl_arq_controller::sack_negotiated_recompute_batch()` body
(`arq_common.cc:979-988`); the production callers are `arq_commander.cc:4263` (CMD,
TEST_CONNECTION_ACK) and `arq_responder.cc:2181` (RSP, TEST_CONNECTION) — the SAME
shared body ⇒ cannot diverge. It is NOT a CMD-only / RSP-only / carve-only path
(`bigblock_block_to_arq` does not call `set_data_batch_size`). The optimizer/gearshift
authority (`optimizer_is_in_control`, `last_data_viable_config`,
`anchor_consec_break_fails`, `probe_backoff`) is UNTOUCHED — `--test-climb-engine`
ALL PASS (0 failures), incl D5/D'6 (OFDM CONFIG_10 still scales to the >=5 SACK floor).

---

## §7 Related fact documents
- `data-flow-batch-size.md` — `data_batch_size` (INV-5 symmetry; the all-ones
  target divergence). The bigblock rung elects K on both sides.
- `data-flow-messages_rx_prev.md` — prev-batch storage + the
  `rsp_prev_batch_*` counters (RISK-4 sizing; INV-4).
- `bigblock-hw-wav-derisk.md` — the P1 PHY validation (cw_ok exposure,
  RISK-1/2/3 PHY-side).
- `sack_partial_bsi_advance.md` — the `bump_bsi_and_transfer_prev` hoist (§6g);
  the test pattern this audit's regression test follows.

---

## §8 Open questions
- **[?]** Does the in-process 2-instance sim pump deliver MULTIPLE blocks
  back-to-back, or is it single-batch-limited (Option-b deferred)? Determines
  whether the sustained delivered RATE is measurable in sim or needs P3 HW.
- **[RESOLVED, downgraded]** RISK-1/RISK-3: the per-block reload does NOT realloc
  the `messages_tx.data` the ARQ aliases — `telecom_system.cc` has ZERO references
  to `messages_tx` (grep-confirmed); `load_configuration` reallocates the PHY
  `data_container`, not the ARQ message queues. So there is no per-block ARQ-alias
  corruption. The P2.3 config-load move is now PURELY a sustained-rate performance
  optimization (avoid per-block OFDM deinit/init) — deferred to P3 where a
  multi-block canary on `messages_tx[i].data` + a sustained-rate measurement can
  validate it (moving it now = untested structural change, CLAUDE.md §3).
- **[?] (NEW, P3 PHY)** The big-block live AWGN path has a deterministic ~25 dB
  implementation loss (clean/60/45 dB PASS, 35/30 dB FAIL with identical BER 0.43).
  This is a PRE-EXISTING P1 PHY scaling/LLR bug (NOT the normalization, which P2.2
  fixed faithfully and which did not move the cliff). Needs a DSP root-cause trace
  (noise_variance_estimate / CSI-LLR clamp / constellation scaling in
  `bigblock_rx_passband`) before any noisy/HW big-block test. Gated before P3.
- **[?]** P2.5 mixed session: when a block's selective-repeat (stock CFG16
  per-frame) is in flight AND a new block follows, does the RX capture loop
  correctly demux per-frame retx vs the next block's single acquisition? RISK-5
  says transparent (per-frame preambles already handled); verify on HW (P3).

---

## §9 Implementation record (P2.1/2.2/2.4/2.5/2.6 build)

**Files changed** (3, none in the ARQ commander/responder/common = optimizer/gearshift
untouched):
- `source/datalink_layer/test_bigblock_arq_unit.cc` — `bigblock_block_to_arq()` stub
  replaced with the real carve + synthetic-EOB + bsi-once + selective-repeat body
  (P2.4/2.5/2.6). The 3-case regression flips FAIL 0/3 → PASS 3/3.
- `source/physical_layer/telecom_system.cc` — `transmit_bigblock` feeds real ARQ bytes
  as the block payload (P2.1, PRBS fallback when nBytes==0);
  `rx_passband_normalize_and_blank()` factored from receive_byte's inline normalization
  and called from receive_bigblock before the estimator (P2.2); receive_byte's inline
  block replaced by the identical helper call (byte-identical extraction).
- `include/physical_layer/telecom_system.h` — `rx_passband_normalize_and_blank()` decl.

**Verification (all green):**
- `--test-bigblock-arq-unit` → PASS 3/3 (CASE1 clean 8/8 + bsi 7→8; CASE2 one-bad-cw=3
  partial 7/8 + retx_count=1 + full 8/8; CASE3 lost-EOB synthetic eob=7 + expected=8 +
  prev complete + 128/128). Was FAIL 0/3 at P2.0 — the fail-before/pass-after contract.
- `--test-partial-bsi-advance=ofdm` PASS, `=mfsk` PASS — stock SACK path no regression.
- `--test-climb-engine` ALL PASS (0 failures) — gearshift/optimizer untouched.
- bigblock clean live validator (`MERCURY_BIGBLOCK_LIVE=1 -m PLOT_PASSBAND -s 16`) →
  VERDICT PASS 8/8 byte-correct — P1 PHY no regression; the P2.2 receive_byte refactor
  is byte-identical on the stock path.

**Commits** (bisectable, per P4): the P2.0 audit+failing-test anchor is `d7eaf1f`; this
build is a SEPARATE commit on `feat/bigblock-livepath-p2` (PHY-wiring + ARQ
granularization). NO monitor merge, NO push, NO Claude/Anthropic attribution.

---

## §10 Implementation record — STEP 2/3 LIVE SEND-PATH WIRING (P3 prereq)

**Built on** the integration tree `integ/bigblock-merge-2026-06-05` (merge of
`feat/bigblock-sack-gate-p1` + `feat/bigblock-livepath-p2-diag` off
`feat/bigblock-livepath-p2 @269942e`). STEP 1 (merge + gate re-verify) was the prior
turn; THIS section is the STEP 2 (wire `send_batch`↔block) + STEP 3 (in-sim single-block
end-to-end) deliverable. The `bigblock_framing_enabled` flag is FORCED true for
validation; the gearshift AUTO-election is DEFERRED to P4 (NOT in this commit).

**Files changed** (4 — telecom_system.cc UNCHANGED, so the PHY is byte-identical):
- `source/datalink_layer/arq_common.cc`:
  - `bigblock_send_one_block()` (NEW) — TX producer of the block. Packs the K=8
    new-data frames' REAL ARQ payload bytes (`messages_batch_tx[i].data[0..length-1]`,
    sub_len = `ldpc.K/8`) into ONE block payload and emits it via the PRODUCTION
    `transmit_byte` → `transmit_bigblock` (P2.1 real-bytes arg) + one `tx_transfer`.
    DECLINES (returns false → stock per-frame path) for MFSK / retx
    (`sack_retransmit_active`) / mixed-control / oversized batches → retx stays STOCK
    CFG16 per-frame (P2.5, INV-3).
  - `send_batch()` — gated branch `if(bigblock_send_one_block())` right after `ptt_on`:
    on a handled block, runs the SAME post-TX bookkeeping as the per-frame tail
    (drain + capture-flush + unmute + ptt-off, then ack-timer/`PENDING_ACK` per DATA
    frame's owning `messages_tx` slot, then clears `messages_batch_tx` + resets
    `frames_to_read`) and returns. Default-OFF → the stock loop is byte-identical.
  - `bigblock_receive_carve()` (NEW) — RX consumer wiring. Re-packs the K decoded
    info-bit sub-units (LSB-first, the TX pack's inverse) into K*sub_len bytes and
    calls the already-§5-audited `bigblock_block_to_arq()` (carve cw_ok→messages_rx[],
    synthetic EOB=K-1, one ACK / partial SACK / bsi-once).
  - `receive()` — gated branch after `receive_byte` returns: when
    `bigblock_framing_enabled && M!=MFSK && is_ofdm_config && bigblock_last_rx_K>0`,
    calls `bigblock_receive_carve` and SKIPs the per-frame `messages_rx_buffer`
    dispatch (one acquisition = one carve, not K per-frame parses). Default-OFF.
- `include/datalink_layer/arq.h` — decls for the three new methods + the TX block
  stash (`bigblock_tx_block_payload/K/sub_len/bsi/ndata`) + `test_sim_inproc_bigblock`;
  `#include <vector>` made explicit.
- `source/datalink_layer/test_bigblock_arq_unit.cc` — `test_sim_inproc_bigblock()`
  (STEP 3 harness): two real instances (CMD A + RSP B), CFG16 grid + R-B pin
  (`data_batch_size==K==8` on both via the shared `sack_negotiated_recompute_batch`),
  drives the PRODUCTION `transmit_bigblock` (real 1400 bytes) → clean PHY block
  loopback (the PROVEN bigblock_livepath path, channel-free for determinism) →
  `receive_bigblock` → `bigblock_receive_carve`/`bigblock_block_to_arq` → ACK-GATE /
  R-B clean-ACK match. CASE A clean = byte-faithful CMD→RSP→ACK→CMD; CASE B one-bad-cw
  = partial SACK + selective-repeat completes.
- `source/main.cc` — `--test-sim-inproc-bigblock` flag + dispatch.

**§5 audit — the NEW producers/consumers (walk):**
1. **TX producer `messages_batch_tx[]` → block payload** (`bigblock_send_one_block`).
   READS `messages_batch_tx[i].data/length` (the per-frame app payload the new-data
   builder filled). Does NOT change `messages_batch_tx` (read-only feed). The block's
   bsi = `messages_batch_tx[0].batch_seq_id` (one block = one batch, INV-1). The
   post-TX `messages_tx[id].status=PENDING_ACK` bookkeeping is byte-identical to the
   per-frame tail (same loop). ✓
2. **RX consumer `bigblock_last_rx_cw_ok` → `messages_rx[]`** (`bigblock_receive_carve`
   → `bigblock_block_to_arq`). The carve is the EXACT P2.4/2.5/2.6 body already audited
   (§1.2/§1.7/§2.2/§2.5). The live `receive()` skips the per-frame parse on a handled
   block (sets `received_message_stats.message_decoded=NO`) so the K-per-frame
   `add_message_rx_data` writes do NOT double-fire. ✓
3. **EOB / bsi** — set by `bigblock_block_to_arq` (synthetic EOB=K-1 BEFORE prev-sizing,
   INV-4; bsi bump ONCE on a clean block, INV-1). The live RX carve passes
   `rsp_current_expected_batch_seq_id` as the block bsi (the block has no per-frame wire
   bit-7 to carry it). **[?] P3:** the wire-carried bsi for a block (so CMD/RSP cannot
   drift the block bsi across a multi-block session) is a P3/HW detail — the single-block
   in-sim path uses the expected bsi, which is correct for ONE block. ✓ for single-block.
4. **Optimizer/gearshift authority** — `optimizer_is_in_control` (`arq.h:2041-2058`),
   `last_data_viable_config` (`:2086`), `anchor_consec_break_fails` (`:2106`),
   `probe_backoff` (`:2031`) — NONE referenced by the new code (grep-verified). No
   bundle=bug. `--test-climb-engine` ALL PASS (0 failures). ✓

**Verification (all green):**
- `--test-sim-inproc-bigblock` → ALL PASS (rc=0): **CASE A clean** decoded=1 K_rx=8
  cw_ok=8/8 recv=8/8 **delivered=1400/1400 byte-faithful** bsi 7→8 one_bump=1
  rsp_bitmap=0xFF==cmd_all_ones=0xFF (R-B) ack_match=1 cmd_credits=1; **CASE B**
  one-bad-cw=3 partial=7/8 bad_absent=1 retx_count=1 at pos 3 + bsi=9 no_bump=1 →
  selective-repeat fill → full=8/8 **delivered=1400/1400**.
- `--test-bigblock-arq-unit` → 8/8 (T1-T9, incl T6 election-symmetry). rc=0.
- `--test-climb-engine` ALL PASS (0 failures); `--test-sim-clock` 0 failed.
  `--test-partial-bsi-advance=ofdm` + `=mfsk` PASS — stock per-frame path no regression.
- bigblock LIVE validator (`MERCURY_BIGBLOCK_LIVE=1 -m PLOT_PASSBAND -s 16`) → 8/8
  byte-correct, post_FEC_BER=0 — PHY no regression (telecom_system.cc unchanged).

**GO/NO-GO:** the big-block IS ARQ-drivable end-to-end — a SINGLE block ARQ-drives
CMD→RSP→ACK→CMD byte-faithful in the in-process sim, the clean ACK matches via R-B
(0xFF==0xFF), and a one-bad-codeword partial completes via selective-repeat. All merge
gates remain green. READY for P3 HW (deploy + sustained transfer + compress-on vs VARA).
The sustained MULTI-block delivered RATE over the paced/real wire is the P3/HW
deliverable (Option-b STOP) — NOT attempted here (one block = one ARQ unit in-sim).

**Commit** (bisectable, per P4): a SEPARATE commit on `integ/bigblock-merge-2026-06-05`
ON TOP of the two merge commits (so the merge and the send-path wiring are distinct).
NO monitor merge, NO push, NO Claude/Anthropic attribution.

---

## §11 PHASE 1 (P3 prereq, no-HW) — block-BSI ON THE WIRE + compression transparency

**Why** (the §10 item-3 open question): the live RX carve `receive()` (`arq_common.cc:6677-6680`)
derived the block bsi from the RX's OWN `rsp_current_expected_batch_seq_id` — a LOCAL guess,
NOT the bsi the TX block advertised. For ONE block this is correct (RX-expected == TX-bsi).
But a SUSTAINED multi-block session has ONE acquisition per block and no per-frame wire bit-7
(`arq_common.cc:6639`) to carry the bsi, so if any block is dropped/missed the CMD and RSP bsi
counters can DRIFT with no wire mechanism to resynchronize → every subsequent block routes
`out_of_window` (`arq_responder.cc:332-441`) and the session stalls. Phase 1 closes this by
carrying the block bsi ON THE WIRE (FEC-protected) so RX adopts the TX's authoritative bsi.

### §11.1 Design — a self-describing block header inside codeword 0 (LDPC-protected)

The block payload is a FIXED `K*sub_len` bytes (`sub_len = ldpc.K/8`), packed LSB-first as the
K codewords' systematic info bits (`transmit_bigblock:8029 byte_to_bit`; the geometry is locked,
so we CANNOT append bytes beyond `K*sub_len`). The wire header occupies a small PREFIX of
**codeword 0** so it rides inside cw0's systematic info bits ⇒ LDPC-protected, decoded with the
block in the SAME single acquisition (no extra preamble, no separate field). It is
SELF-DESCRIBING so the RX delivers each sub-codeword its EXACT frame length (REQUIRED for
compression transparency — see §11.4 / INV-10):

```
  Block header (cw0 prefix), hdr_total = BB_HDR_FIXED(2) + 2*K bytes:
    payload[0]            = block_bsi (low 8 bits)        <- THE wire bsi (drift-proof)
    payload[1]            = n_data    (0..K)              <- # filled sub-codewords this block
    payload[2 + 2*c + 0]  = length[c] low  byte           } per-codeword app length table
    payload[2 + 2*c + 1]  = length[c] high byte           }  (c = 0..K-1)
  App payloads (frame c -> sub-codeword c, ALIGNED to the LDPC codeword for cw_ok/SACK):
    cw0 app:  payload[hdr_total .. sub_len-1]              (frame 0, capacity sub_len - hdr_total)
    cwc app:  payload[c*sub_len .. c*sub_len + length[c]-1]  c=1..K-1 (full sub_len capacity)
```

**Why frame c MUST stay aligned to LDPC codeword c (NOT a contiguous byte stream):** the PHY
exposes a PER-LDPC-CODEWORD clean vector `cw_ok` (K bits, `telecom_system.h:595`) = the SACK
granularity (INV-2). Selective-repeat re-sends EXACTLY the failed codeword (INV-3). If a frame's
payload spanned codeword boundaries, a single bad codeword would corrupt two frames and break
INV-2/3. So each frame stays in its OWN codeword's `sub_len` region; the header carries the real
lengths so the carve trims the per-codeword zero-pad on delivery.

**Why a header in cw0 (not a separate PHY field):** (a) the PHY treats the payload as OPAQUE
bytes ⇒ ZERO PHY change (`telecom_system.cc` byte-identical); (b) FEC-protected (rides in cw0's
info bits); (c) decoded in the normal carve — RX reads the header back AFTER the decode and uses
the wire bsi as authoritative, replacing the local `rsp_current_expected_batch_seq_id` guess, and
the length table so each delivered slot byte-matches its TX frame (compression transparency).

**Capacity guard (INV-9, NEW):** cw0's reduced capacity `sub_len - hdr_total` must still hold a
full compressed frame `max_frame` (`arq.h:2072`). The big-block lattice (Ngrid=60) gives
`sub_len` ≫ `max_frame` + `hdr_total`, so the guard holds with margin;
`bigblock_send_one_block` DECLINES the block (→ stock per-frame path) if `sub_len <= hdr_total`
or if frame-0's `length > sub_len - hdr_total` (defensive; never hit at the CFG16 rung).
`length[c]` is a uint16 (≤ `sub_len`); `max_frame ≈ 173` ≤ 255 today but uint16 is future-proof.

### §11.2 Producers / Consumers delta (the §1/§2 audit for the NEW wire header)

- **PRODUCER (TX) `bigblock_send_one_block`** (`arq_common.cc:3643`): writes the cw0 header
  prefix `[bsi, n_data, length[0..K-1] (uint16 LE)]`; packs frame 0's app bytes at payload offset
  `hdr_total`, frames 1..K-1 at `c*sub_len` (codeword-aligned). The TX stash `bigblock_tx_block_*`
  records the per-codeword lengths too (ground truth for the in-sim harness). Optimizer/gearshift
  authority UNTOUCHED (grep-verified, as §10).
- **CONSUMER (RX) `bigblock_receive_carve`** (`arq_common.cc:3754`): the decoded info bits are
  re-packed (LSB-first inverse, unchanged) into `payload[]`; the carve PARSES the cw0 header
  (`wire_bsi`, `n_data`, `length[]`), then hands `bigblock_block_to_arq()` the wire bsi + the
  per-codeword lengths + the app-byte base offsets (cw0 at `hdr_total`, cwc at `c*sub_len`).
- **CONSUMER `bigblock_block_to_arq`** (`test_bigblock_arq_unit.cc:102`): NEW signature carries a
  `const int* sub_lengths` (per-codeword app byte length) + a `header_offset` for cw0. It stamps
  `messages_rx[c].length = sub_lengths[c]` (NOT `sub_len`) and copies exactly that many bytes from
  the codeword's app base ⇒ the delivered slot byte-matches the TX frame (INV-6 with VARIABLE
  lengths; compression transparency). The bsi stamp (`messages_rx[c].batch_seq_id = block_bsi`,
  INV-2), the synthetic EOB (INV-4), the one-ACK/partial/bsi-once flow are UNCHANGED — block_bsi
  now comes from the WIRE. Selective-repeat (INV-3) re-sends the failed codeword's app bytes
  (its `sub_lengths[c]` bytes from `retransmit_frames[]`), unchanged in mechanism.
- **CONSUMER (live `receive()`)** (`arq_common.cc:6677-6685`): no longer passes
  `rsp_current_expected_batch_seq_id` as the block bsi. The wire-bsi is read INSIDE
  `bigblock_receive_carve` from the decoded payload, so the live path passes a SENTINEL and the
  carve uses the wire value. (RISK-6: a total acquisition miss leaves no decoded payload — the
  carve already guards `info_bits==NULL`; on the live path `bigblock_last_rx_K>0` gates entry,
  so a decoded block is present when the carve runs.)

### §11.3 Single-block byte-faithfulness with EXPLICIT wire-bsi (the validation)

`test_sim_inproc_bigblock` CASE A is upgraded to the DRIFT-PROOF assertion: the RSP's local
`rsp_current_expected_batch_seq_id` is set to a DIFFERENT value than the TX block's bsi (the
drift the multi-block session would suffer), and the test asserts the carve recovers the TX bsi
FROM THE WIRE (`messages_rx[c].batch_seq_id == TX block_bsi`, NOT the RSP's wrong local guess),
delivers 1400/1400 byte-faithful, and the ONE-bump lands on the wire bsi. This is the
fail-before/pass-after for the wire-bsi: before the change the carve stamps the RSP's wrong local
bsi (drift) ⇒ the assertion FAILS; after, it stamps the wire bsi ⇒ PASS.

### §11.4 Compression transparency to the big-block path (CONFIRMED, no interaction)

Compression is at the ARQ batch layer ABOVE the big-block framing — VERIFIED by data flow:
- TX: `compressor.compress_block(staging, raw_size, comp_buf, batch_capacity)`
  (`arq_commander.cc:10868`) produces the COMPRESSED bytes; they are split into frames via
  `add_message_tx_data(DATA_LONG/SHORT, chunk, comp_buf+pos)` (`:11061/11063`) → `messages_tx[]`.
  So `messages_tx`/`messages_batch_tx[i].data` already hold COMPRESSED bytes BEFORE any framing.
- `bigblock_send_one_block` (`arq_common.cc:3685-3697`) reads `messages_batch_tx[i].data` (the
  already-compressed app payload) and transports it verbatim. The big-block carries the
  COMPRESSED bytes; it has NO knowledge of / interaction with the compressor.
- RX: `bigblock_block_to_arq` carves the bytes into `messages_rx[]` byte-faithfully (INV-6,
  1400/1400). Downstream `copy_data_to_buffer` → decompress runs IDENTICALLY to the per-frame
  path (the carve sets the same `messages_rx[c]` fields `add_message_rx_data` would).
- INV-10 (NEW, compression↔framing): the splitter sizes each frame ≤ `max_frame` (`arq.h:2072`);
  the big-block packs each frame into a `sub_len`-capacity sub-codeword and the WIRE LENGTH TABLE
  (§11.1) carries each frame's exact length so the RX delivers EXACTLY `length[c]` bytes per slot
  (not `sub_len` with zero-pad — that would inject pad bytes into the reassembled compressed
  stream and break decompress). `sub_len >= max_frame` (cw0 = `sub_len - hdr_total >= max_frame`,
  INV-9) ⇒ no frame truncated ⇒ the variable-length compressed byte stream reassembles
  byte-faithfully (`copy_data_to_buffer` `arq_common.cc:7657-7661` copies `messages_rx[i].length`
  bytes per ACKED slot) ⇒ decompress is byte-faithful. CONCLUSION: compression is TRANSPARENT to
  big-block framing once the carve delivers the WIRE length (the P2.0 carve that delivered a
  uniform `sub_len` was a LATENT compression-breaking bug, fixed here). The compressor itself
  (`compress_block` `arq_commander.cc:10868` → frames `:11061/11063` → `messages_tx`) is byte-for
  -byte UNCHANGED; the big-block carries the compressed bytes opaquely.

### §11.5 Verification (Phase 1 gates) — ALL GREEN

- `--test-sim-inproc-bigblock` → CASE A **drift-proof wire-bsi** (TX bsi=7 recovered from the wire
  despite the RSP's local guess=12 AND the carve fallback=10 both WRONG; every carved slot stamps
  the wire bsi) + **VARIABLE-length byte-faithful** (delivered 619/619, frames 12/49/86/123/160/
  26/63/100 bytes); CASE B one-bad-cw=3 → partial SACK + selective-repeat (retx at pos 3, original
  bsi, REAL length) completes 619/619.
- `--test-bigblock-arq-unit` → 8/8 (T1-T9) — the new `bigblock_block_to_arq` args are defaulted
  (NULL `sub_lengths`, `cw0_offset=0`) so the legacy uniform-length unit-test path is byte-identical.
- bigblock LIVE validator (`MERCURY_BIGBLOCK_LIVE=1 -m PLOT_PASSBAND -s 16`) → 8/8 byte-correct,
  post_FEC_BER=0 — the PRBS `nBytes==0` path is UNTOUCHED by the whitening; stock `transmit_byte`/
  `receive_byte` per-frame path is byte-identical (whitening is INSIDE the `nBytes>0` big-block
  branch only, gated by `bigblock_framing_enabled`).
- `--test-climb-engine` ALL PASS (0 failures); `--test-sim-clock` 0 failed;
  `--test-partial-bsi-advance=ofdm` + `=mfsk` PASS — optimizer/gearshift + stock SACK no regression.

### §11.6 ROOT-CAUSE FINDING — big-block payload was missing ENERGY DISPERSAL (whitening)

While validating variable-length (compression-realistic) frames, the in-sim loopback decoded the
big-block payload to GARBAGE: **788/1400 byte errors with a ZERO-PADDED payload, 0/1400 errors
with a full-entropy payload** (DBG-confirmed both ways, deterministic). Root cause (CONFIRMED, not
guessed): the STOCK per-frame OFDM TX XORs its info bits with a PRBS before LDPC encode
(`bit_energy_dispersal`, `telecom_system.cc:665`; the descrambler exists since the modem's origin,
`interleaver.cc:111`) precisely so an arbitrary payload — including the long runs of zeros a SHORT
compressed frame produces when zero-padded to `sub_len` — modulates to a well-conditioned signal.
The big-block real-bytes path (`transmit_bigblock` `nBytes>0`) packed the payload bits STRAIGHT
into the LDPC codewords with NO dispersal, so a zero-heavy payload produced a degenerate
constellation the channel estimator / CSI-LLR could not decode. The P1 seeded-PRBS validator never
exposed this because PRBS is full-entropy by construction.

**Fix (root cause, using Mercury's OWN existing mechanism — NOT a new DSP guess):** apply the same
self-inverse XOR energy dispersal to the big-block payload. A fixed-seed LCG PRBS
(`bigblock_whiten_payload_bits`, seed `BIGBLOCK_WHITEN_SEED`, deterministic so TX/RX agree with no
wire negotiation) whitens the WHOLE `K*ldpc.K`-bit payload in `transmit_bigblock` (real-bytes
branch only) before encode; the ARQ RX de-whitens the decoded bits in `bigblock_receive_carve`
(via the public `cl_telecom_system::bigblock_whiten_bits`) before re-packing to bytes. Self-inverse
⇒ TX-whiten + RX-de-whiten recovers the exact payload. The PRBS validator path (`nBytes==0`) does
NOT use this scrambler (its payload is already random + gated by its own cw_info_ref), so it stays
8/8. **This was a PRE-EXISTING big-block PHY gap (P1 carried it latently) that ONLY a non-PRBS
payload exposes — exactly the realistic compressed-frame case Phase 1 set out to validate.** It is
ALSO almost certainly a contributor to the §2.2 "~25 dB implementation loss / BER 0.43 at 30 dB"
open question (that was measured with a non-PRBS / partly-zero live AWGN payload); the residual P3
PHY DSP trace should re-measure with whitening ON.

**Files changed (Phase 1):**
- `include/datalink_layer/datalink_defines.h` — `BIGBLOCK_HDR_FIXED_BYTES` (2) +
  `BIGBLOCK_HDR_TOTAL_BYTES(K)` (= 2 + 2K) wire-header layout constants.
- `source/datalink_layer/arq_common.cc` — `bigblock_send_one_block` packs the cw0 wire header
  `[bsi, n_data, length[0..K-1] uint16 LE]` + frame-0 app at `hdr_total`; `bigblock_receive_carve`
  parses the wire header (authoritative bsi + length table), DE-WHITENS the decoded bits, hands the
  per-codeword lengths + cw0 offset to the carve; the live `receive()` carve passes
  `use_wire_header=true` (wire bsi authoritative, `rsp_current_expected_batch_seq_id` is a fallback
  only).
- `source/datalink_layer/test_bigblock_arq_unit.cc` — `bigblock_block_to_arq` gains
  `sub_lengths`/`cw0_offset` (defaulted; legacy path byte-identical) + delivers the exact wire
  length per slot; `bigblock_test_delivered_varlen` helper; `test_sim_inproc_bigblock` CASE A/B
  upgraded to the drift-proof wire-bsi + variable-length byte-faithful assertions.
- `include/physical_layer/telecom_system.h` + `source/physical_layer/telecom_system.cc` —
  `bigblock_whiten_payload_bits` (static) + `bigblock_whiten_bits` (public) energy dispersal;
  applied in `transmit_bigblock` real-bytes branch. Stock per-frame path UNCHANGED.

**Commit** (Phase 1): a SEPARATE commit on `integ/bigblock-merge-2026-06-05` ON TOP of `676f055`.
NO monitor merge, NO push, NO Claude/Anthropic attribution. P3 HW is the NEXT phase.

## §12 PHASE 2 (P3 HW) — the VARA-parity verdict attempt (2026-06-05)

Goal: deploy the integration branch to BOTH Pis, FORCE the big-block framing flag, run a
SUSTAINED compressible Winlink message at a clean IONOS channel (WGN:40), measure the
EFFECTIVE delivered rate vs VARA EFFECTIVE = 6241 × LZHUF_ratio.

Commit `528234c` on `integ/bigblock-merge-2026-06-05` (on top of `0300ea0`). Tools added:
`tools/bigblock_p3_hw.py` (orchestrator) + `tools/bigblock_p3_pihelper.py` (Pi-local driver).
Results dir: `bigblock_p3_hw/` (STATUS.md + per-run logs/JSONs).

### §12.1 ROOT-CAUSE FIX — big-block engaged at EVERY OFDM config, stalling the climb [the headline code finding]
The forcing mechanism (`MERCURY_BIGBLOCK_FRAMING=1` → `bigblock_framing_enabled=true`,
telecom_system ctor) turned the flag on GLOBALLY. But ALL FIVE engagement gates checked only
`M != MFSK` / `is_ofdm_config()` (configs **0..16**) / `!is_robust_config()` — NONE required
CONFIG_16. The gearshift starts at ROBUST_0 and climbs through CONFIG_0..15 (every one
`is_ofdm_config()==true`, `!is_robust_config()`), so the block engaged at those rungs with a
DIFFERENT, unvalidated geometry. **HW-observed at CONFIG_0:** `[BIGBLOCK-ARQ] PARTIAL block
bsi=0 K=1 ... clean=0 -> SACK gaps`, `[BIGBLOCK-RX] carve: K=1 ... sub_len=12` (vs the validated
CFG16 K=8 sub_len=175). Every batch carved `clean=0` → no clean ACK → `[GEARSHIFT] FRAME UP DATA
FAILED -> BREAK to 102`. The climb could NEVER reach CFG16; delivered ~53 bytes in 7 min.

FIX (root-cause, CLAUDE.md §2 — gate on the validated rung, not threshold-tune): added
`current_configuration == CONFIG_16` to all five points:
- `telecom_system.cc` PHY `transmit_byte` branch (:626) and `receive_byte` branch (:1007)
- `arq_common.cc` `bigblock_send_one_block` TX gate (:~3653), `receive()` RX carve gate (:~6787,
  was `is_ofdm_config`), `sack_negotiated_recompute_batch` K-pin (:~982, was `!is_robust_config`)
- PLOT_PASSBAND / unit validators run at `-s 16` (current_configuration==16) → unaffected.

VALIDATION: in-sim `--test-sim-inproc-bigblock` ALL PASS (CASE A 619/619 byte-faithful, CASE B
selective-repeat), `--test-bigblock-arq-unit` 8/8, `--test-climb-engine` 0 failures. **HW
structural validation:** with the fix the gearshift climb PROGRESSES
ROBUST_0(100)→ROBUST_1(101)→ROBUST_2(102)→CONFIG_0→…→CONFIG_4 with clean MFSK ACKs
(`matched=16`) and **ZERO** `BIGBLOCK-ARQ` engagement at low configs (was constant PARTIAL spam).
The big-block no longer corrupts the low-config data path.

### §12.2 Infra bugs found+fixed in the P3 harness (not Mercury code)
- **mercury binds 127.0.0.1 ONLY** (`tcp_socket.cc:95` `INADDR_LOOPBACK`, all platforms): the
  direct-LAN-connect pattern (connect to Pi IP:8400) CANNOT reach mercury. The ARQ session must be
  driven Pi-LOCAL. FIX: `bigblock_p3_pihelper.py` runs ON each Pi (uploaded via butler), connects
  to 127.0.0.1 ports; the orchestrator launches mercury + helpers via SSH and downloads results.
- **butler UPLOAD/DOWNLOAD split args on whitespace** (`ionos_butler.py:309` `rest.split()`): the
  workspace path `…\hermes and mercury\…` has spaces → UPLOAD returns OK but transfers NOTHING
  (silent). FIX: stage every up/download through a SPACE-FREE temp dir + verify-after-upload guard.
- **CONNECT needs `-R`**: with `-s 16` (no `-R`) the connect control frames go out on CONFIG_16
  and the RSP control-ACK turnaround fails (HW: HAIL ok, RSP gets CONNECT_START, CMD "attempt N of
  15"). FIX: launch with `-R` so the handshake runs at the ROBUST/MFSK tier, then climb.

### §12.3 BLOCKER — testbed CONNECT instability + marginal SNR prevented the CFG16 measurement [?]
After the §12.1 fix the climb works but never reached CFG16 in the dwell window: reached only
CONFIG_4 in 140s, gearshift SNR reads ~1.0 dB at WGN:40 (should be ~42 dB per
`testbed_wgn_snr3k_mapping`: SNR3k = WGN + 2.4). The ARQ CONNECT was unreliable/slow (3–8 min,
sometimes never finalizing): the MFSK handshake reaches RSP `link_status:Connected to TESTA` +
CMD `Connection Accepted by TESTB` / `[RX-MFSK-CTRL-CONNECT-ACK] echoed_cap=0x01==0x01` but the
CMD does NOT reliably transition to the CONNECTED state that emits `CONNECTED <call>…` on the TCP
control socket. Run-to-run INCONSISTENT (one run sat `link_status:Idle`, never attempting).
Matches the MEMORY-documented Pi audio-state drift (`pi_audio_state_drift.md`, ~6h uptime, many
bench sessions today). A Pi REBOOT (the documented fix) did NOT recover the connect reliability;
a 200→420 s connect-timeout bump did not catch a finalize either. **Open [?]:** the connect
finalization at CONNECTION_ACCEPTED on the MFSK control tier is intermittent on this testbed
state — a pre-existing ARQ/testbed issue, OUTSIDE the big-block scope, but it gates the P3
sustained measurement. The big-block STRUCTURAL fix (§12.1) is validated; the EFFECTIVE-delivered
VARA-parity NUMBER is NOT yet obtained on HW (no sustained CFG16 block transfer completed).

**Commit** (P3 HW): `528234c` on `integ/bigblock-merge-2026-06-05` (on top of `0300ea0`). NO
monitor merge, NO push, NO attribution. Bench left UNLOCKED + tables as-found.

---

## §13. HEAP-OVERRUN ROOT CAUSE — the K-codeword block written into / read from STOCK single-frame buffers (2026-06-05, fix branch `fix/bigblock-cfg16-heap-overrun` off `integ/bigblock-merge-2026-06-05`@`fe9d3e3`)

**Symptom (HW-observed, A/B-localized to `MERCURY_BIGBLOCK_FRAMING=1`):** the FIRST CFG16
big-block TX aborts BOTH instances with glibc heap corruption — COMMANDER `free(): invalid
next size` right after an OVERSIZED `[TX-PEAK] ... size=45552 cfg=16` burst; RESPONDER
`free(): invalid pointer` in the CFG16 PHY-SWITCH deinit. Env UNSET ⇒ CFG16 completes clean.
So the fault is in the big-block CFG16 TX/RX path: a K=8 concatenated block written
into/freed from an allocation sized for ONE stock CFG16 frame.

### §13.1 The TWO production overruns (both confirmed in CASE C, see §13.4)
**(A) TX divert — COMMANDER.** The big-block branch in `transmit_byte`
(`telecom_system.cc:626`, pre-fix) gated ONLY on `bigblock_framing_enabled && M!=MFSK &&
current_configuration==CONFIG_16` — it ignored WHICH caller / which `out` buffer it was
handed. But at the CFG16 rung the gearshift + ARQ control loop issue MANY stock per-frame
(`send_batch` loop `arq_common.cc:4317`, into `&batch_frames_output_data[pack_cursor]`, one
`frame_output_size`≈16120-double slot) and single-frame (`send()` `arq_common.cc:4011`, into
`ready_to_transmit_passband_data_tx`, `total_frame_size` doubles) transmits — every
CONTROL/ACK/SACK_RSP frame, and the WHOLE batch whenever `bigblock_send_one_block()` DECLINES
(mixed-control batch, `sack_retransmit_active`, `n_data>K`, `len0>cw0_cap`;
`arq_common.cc:3666-3706`). Each of those handed a FRAME-sized `out`. The unconditional
divert routed them into `transmit_bigblock`→`bigblock_tx_passband`, which writes the WHOLE
K-block passband (`(Nofdm*pre_nSymb + Nofdm*Ngrid)*interp` doubles — **MEASURED 79360 at the
live thin grid**, telecom_system.cc:7058-7065) into the frame slot ⇒ ~63k-double forward heap
smash of `batch_frames_output_data_filtered1/2`. `transmit_bigblock` never set
`tx_last_emitted_frame_samples`, so the `frame_len` clamp (`arq_common.cc:4321`) under-advanced
`pack_cursor` and HID the overrun in software; the next `delete[] batch_frames_output_data`
(`arq_common.cc:4522`) aborted `free(): invalid next size`. (The 45552 in the HW log was the
in-bounds size of the VICTIM stock buffer, printed AFTER the smash.)

**(B) RX copy-out — RESPONDER.** `receive_bigblock` decodes `Kout*ldpc.K = 8*1400 = 11200`
info bits, then copied them with `for(i<copy_bits) out[i]=info_bits[i]`
(`telecom_system.cc:8166-8167`, pre-fix) into the caller's `out`. On the live ARQ path
`out == data_container.data_byte = int[N_MAX=1600]` (`arq_common.cc:6765`; alloc
`data_container.cc:108`; `N_MAX` `physical_defines.h:31`) ⇒ **9600-int (38400-byte) forward
overrun** smashing the adjacent `data_container` chunk headers (`encoded_data`,
`bit_interleaved_data`, …). Latent until the next `bigblock_restore_stock_config()` →
`load_configuration` → `data_container.deinit()` CDELETE chain (`data_container.cc:222-227`)
freed a clobbered neighbor ⇒ `free(): invalid pointer` in the PHY-SWITCH deinit. The consumer
`bigblock_receive_carve` (`arq_common.cc:3847`, `nbits=K*sub_len*8=11200`) likewise OVER-READ
the 1600-int `data_byte`.

### §13.2 Why the in-sim CASE A/B ran GREEN while the live path crashed (the sim-faithfulness gap)
`run_block_loopback` (`test_bigblock_arq_unit.cc`) passed a CORRECTLY block-sized
`info_bits_out` (`(K_tx+1)*ldpc.K + ldpc.K`) to `receive_byte` and a block-sized `tx_pb`
(`bigblock_tx_total_samples()`) to `transmit_byte` — so NEITHER the copy-out NOR the divert
ever touched an undersized PRODUCTION allocation. Canaries were disabled passthrough
(`include/debug/canary_guard.h`), so the Windows allocator's slack hid the smash. The sim
exercised the block PHY but NOT the stock production buffers the live ARQ path hands it.

### §13.3 The fix (root cause: SIZE from real geometry + CONSTRAIN the producer; never enlarge a magic constant)
- **Producer constraint (TX, fixes COMMANDER on every decline/control/per-frame path).** Added
  a per-call block-emit intent `bigblock_emit_as_block` (+ `bigblock_emit_out_capacity`) on
  `cl_telecom_system` and a RAII `bigblock_emit_scope` (`telecom_system.h`). The
  `telecom_system.cc:626` branch now ALSO requires `bigblock_emit_as_block`. Only the dedicated
  block driver `bigblock_send_one_block` (`arq_common.cc`) and the two loopback validators set
  it — and ONLY while they pass a BLOCK-sized buffer (`bigblock_tx_total_samples()`-derived).
  Every stock per-frame/single-frame/control `transmit_byte` at CFG16 keeps the per-frame OFDM
  geometry. A HARD GUARD inside `transmit_bigblock` computes `required =
  bigblock_tx_total_samples()` and REFUSES (no write, `assert`) when it exceeds
  `bigblock_emit_out_capacity`. Defense-in-depth: a block emit now stamps
  `tx_last_emitted_frame_samples` so the `frame_len` bookkeeping can never again mask an overrun.
- **RX output sizing (fixes RESPONDER on every path).** Added `std::vector<int>
  bigblock_rx_infobits` on `cl_telecom_system`; `receive_bigblock` sizes it to
  `(K_expected+1)*ldpc.K + ldpc.K` (≥ `Kout*ldpc.K`) and lands the decode THERE. The ARQ carve
  reads from that member (`arq_common.cc:6794` now passes
  `telecom_system->bigblock_rx_infobits.data()`, NOT `data_byte`). The legacy copy into `out`
  is BOUNDED by `N_MAX` so the stock `data_byte[N_MAX]` is never overrun and its N_MAX consumers
  need no re-audit. `bigblock_livepath_loopback`'s byte-correct re-check now reads the member too.

### §13.4 Producer/consumer audit update for the big-block TX/RX info-bit buffers
- **`bigblock_rx_infobits` (NEW).** Producer: `receive_bigblock` copy-in (`telecom_system.cc`,
  sized `(K_expected+1)*ldpc.K + ldpc.K`). Consumers: `bigblock_receive_carve`
  (`arq_common.cc:3847`, reads `K*sub_len*8 = K*ldpc.K` ints — `<=` size by construction since
  `sub_len==ldpc.K/8`); the live-path validator §7 re-check; CASE A/B/C. INVARIANT: capacity
  `>= Kout*ldpc.K`; held by construction.
- **`data_container.data_byte[N_MAX]`.** Producer on the big-block path is now ONLY the
  N_MAX-bounded legacy copy in `receive_bigblock` (never `> N_MAX`). The full block decode no
  longer flows through it — so its long list of stock N_MAX consumers is unchanged/safe.
- **`batch_frames_output_data` (per-frame TX slots) / `ready_to_transmit_passband_data_tx`
  (single-frame TX).** Producer `transmit_byte` at CFG16 now stays on the per-frame OFDM path
  unless the caller armed `bigblock_emit_as_block` with a block-sized buffer ⇒ writes
  `<= frame_output_size` / `total_frame_size`. INVARIANT restored: a block waveform is emitted
  ONLY into a `bigblock_tx_total_samples()`-sized buffer (the `block_pb` in
  `bigblock_send_one_block`, `tx_pb` in the validators).

### §13.5 Reproducer — `--test-sim-inproc-bigblock` CASE C (deterministic, local, fail-before/pass-after on ONE binary)
CASE C (`test_bigblock_arq_unit.cc`) drives the EXACT production buffers with explicit tail
canaries: **RX leg** calls the REAL `receive_byte(rx_pb, out)` with `out` a `data_byte`-shaped
buffer (canary AT the `N_MAX` boundary); **TX leg** calls the REAL `transmit_byte` at CFG16 into
a frame-sized slot (canary at the slot boundary) WITHOUT arming the emit scope — the exact
declined-batch/control fallthrough. `MERCURY_BIGBLOCK_OLDGATE=1` restores the pre-fix gate
(config-only divert + unbounded copy-out) on the SAME binary so the fail-before is reproducible
without a revert build; buffers are over-allocated so the pre-fix write trips the boundary
canary WITHOUT a process-killing smash.

| run | RX canary | TX canary | rx_member | result |
|-----|-----------|-----------|-----------|--------|
| `MERCURY_BIGBLOCK_OLDGATE=1` (pre-fix) | **0 (SMASHED)** | **0 (SMASHED)** | 14000≥11200 | **CASE C FAIL** |
| (fixed, default) | 1 (intact) | 1 (intact) | 14000≥11200 | **CASE C PASS** |

Measured geometry: `frame_slot=16120  block_n=79360  n_tx=79360` ⇒ the block is **4.9×** the
stock frame slot. **Regression:** `--test-sim-inproc-bigblock` ALL PASS (CASE A 619/619
byte-faithful, CASE B selective-repeat, CASE C); `--test-bigblock-arq-unit` 8/8;
`--test-climb-engine` 0 failures. Net-PHY is a function of the UNCHANGED block geometry (pilot
thinning / nData / log2M / rate) — this is a buffer-sizing + free correction, NOT a wire-format
change — so the §13-sibling sim2 net-PHY (7859/7960 bps > VARA 7050) is preserved.

**Commit** (heap-overrun fix): on `fix/bigblock-cfg16-heap-overrun` off
`integ/bigblock-merge-2026-06-05`@`fe9d3e3`. NO monitor merge, NO push, NO attribution.

## §14. FINAL CONSOLIDATION — heap-fix + CRC-fix + LAN-bind in one binary (2026-06-05)

Branch `integ/bigblock-final-2026-06-05` (worktree `C:/Users/kamer/mercury_wt/bigblock-final`)
off `integ/bigblock-merge-2026-06-05`@`fe9d3e3`:
- `git merge --no-ff fix/bigblock-cfg16-heap-overrun` — clean (descended from `fe9d3e3`).
- `git merge --no-ff feat/bigblock-cw-crc8` — conflicted ONLY in
  `test_bigblock_arq_unit.cc` (both branches add a sim "CASE C" to
  `test_sim_inproc_bigblock`). `arq_common.cc` + `telecom_system.cc` auto-merged (disjoint
  line ranges: CRC touches `bigblock_send_one_block` payload-assembly :3700-3785 + the carve
  CRC-demote :3908-3940 + `bigblock_rx_passband` LLR-corruptor :7414; heap touches the same
  function's TX-buffer/emit-scope :3809-3822 + the `receive()` member-carve call :6892 +
  `transmit_byte`/`transmit_bigblock`/`receive_bigblock` :642/8126/8226). Verified both
  change-sets coexist: cw caps (`cw0_cap`/`cwc_cap`) + CRC stamping over `block_payload_bytes`
  AND the `bigblock_emit_scope`/`block_pb_capacity` sizing; carve recomputes CRC + demotes
  cw_ok BEFORE the cw0 header-trust gate AND reads from the dedicated `bigblock_rx_infobits`
  member. The CRC byte is one of the `K*sub_len` info bytes → the passband sample count
  (`block_pb_capacity` from `bigblock_tx_total_samples()`) is unchanged; no capacity edit needed.
- `git cherry-pick 06cbbce` (LAN-bind, platform-conditional tcp_socket bind) — clean.

**CASE-C collision resolved by KEEPING BOTH, renaming the CRC one to CASE D.** The shared
`build_block_wire` lambda (auto-merged) now stamps the per-codeword wire CRC, so BOTH the
heap-canary case (CASE C) and the bit-flip case (CASE D) drive CRC-valid wire blocks.

**Cross-layer reconciliation required for CASE D** (the canonical CLAUDE.md §5 trap): CASE D
was authored against the pre-heap-fix RX, reading the decoded bits from `receive_byte`'s `out`
param. The heap-fix re-routed the full `K*ldpc.K` decode into `bigblock_rx_infobits` and bounded
`out` to `N_MAX` (~1.1 codewords). CASE D's direct-CRC-proof and its `bigblock_receive_carve`
call both had to read from `tsB->bigblock_rx_infobits` (the full member) — exactly as the live
`receive()` path and the heap-fix's CASE A/B do. Pre-reconciliation CASE D FAILED in the normal
run (`crc_match_others=0 partial=1/8`); post-reconciliation it PASSES (`partial=7/8 retx_count=1`).

**Two fail-befores, both on the SAME consolidated binary** (no revert build):
- `MERCURY_BIGBLOCK_OLDGATE=1` → CASE C heap canary FAILs (`rx_canary_ok=0 tx_canary_ok=0`).
- `MERCURY_BIGBLOCK_NOCRC=1` (NEW reproducer hook added to the carve CRC-demote, mirrors
  OLDGATE) → CASE D FAILs with the silent-corruption signature (`partial=8/8 retx_count=0`).

**Regression (all FOREGROUND, green):** `--test-sim-inproc-bigblock` ALL PASS (CASE A/B + heap
CASE C + bit-flip CASE D); `--test-bigblock-arq-unit` 8/8 (T1-T9); `--test-climb-engine` 0
failures; plus `--test-sim-clock`/`--test-clean-batch-viability`/`--test-data-anchored-promote`/
`--test-probe-backoff`/`--test-phantom-ack-gate` all 0 failures.

**Net-PHY WITH the 8-byte/block CRC overhead** (K=8 codewords × 1 CRC-8 byte = 8/1400 = 0.57%
of the info bytes): 7.2% selective layout = **7814 bps net-PHY (7713 bps app-delivered) > VARA
Standard 7050**; 6% flat = 7915 (7813 app). The CRC win is preserved.

NO monitor merge, NO push, NO attribution.

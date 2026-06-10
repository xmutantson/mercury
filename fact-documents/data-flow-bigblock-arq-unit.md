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

## §15. LIVE-PATH 0-BYTE DELIVERY — NOT whitening; use-after-free + partial-block decode + missing FIFO delivery (2026-06-05, fix branch `fix/bigblock-whiten-align` off `integ/bigblock-final-2026-06-05`@`c746064`)

The consolidated final binary (§14) TXed a CFG16 big-block to completion but delivered **0 of
the app bytes** end-to-end. The leading hypothesis (and the task framing) was a TX/RX **whitening
(energy-dispersal) misalignment** — a double de-whiten or a span/seed/offset mismatch. **All three
parallel source-audits REFUTED whitening, and the empirical reproduction CONFIRMED the refutation:
whitening was never the bug.** Three other root causes, all on the LIVE 2-instance path that the
synthetic CASE A-D bypass, were found and fixed. SIM_INPROC (`MERCURY_SIM_2INST=1 -m SIM_INPROC`
with `MERCURY_BIGBLOCK_FRAMING=1`, pinned CFG16) DOES route the big-block through the REAL
`transmit_byte`/`transmit_bigblock` -> wire/capture ring -> `receive_byte`/`receive_bigblock` ->
`bigblock_receive_carve` -> `bigblock_block_to_arq`, so it reproduces the live bug (the CASE A-D
unit tests do not — see §15.4).

### §15.1 Root cause 1 — USE-AFTER-FREE of the RX passband buffer across the config rebuild/restore
`receive_bigblock` (telecom_system.cc:8205) derives the block K when `bigblock_last_tx_K<=0` (the
RX side: it never TXed a block) by calling `bigblock_rebuild_thin_grid()` + `bigblock_restore_stock_config()`
(:8222-8224). **`bigblock_restore_stock_config()` calls `load_configuration()`, which DEINITS+REINITS
the `data_container` — FREEING and REALLOCATING `ready_to_process_passband_delayed_data`.** The `data`
pointer the live caller (`receive()`, arq_common.cc:6864) passed IS that buffer, so after the rebuild
`data` is **dangling** (freed). The next line, `rx_passband_normalize_and_blank(data, nSamples)` (:8245),
reads freed/re-used heap. gdb proof: at the normalize, `data=0x63fb040` but the LIVE
`ready_to_process_passband_delayed_data=0x63fe040` — **different buffers** (the realloc moved it). In
SIM_INPROC this SIGSEGVs (over-reading the freed region) / corrupts the heap; on HW the allocator
happened to leave readable-but-stale bytes -> garbage decode -> `wire_bsi=159`, every per-codeword CRC
fails -> `PARTIAL clean=0` -> 0 delivered (the exact HW symptom). **FIX:** snapshot `data[0..nSamples)`
into a stable LOCAL `std::vector<double>` at the TOP of `receive_bigblock` (before any rebuild) and run
the whole decode against the snapshot (telecom_system.cc, "USE-AFTER-FREE ROOT-CAUSE FIX").

### §15.2 Root cause 2 — the live RX decoded a PARTIAL block (one-stock-frame wait)
After the UAF fix the crash was gone but only **cw0 decoded byte-perfect; cw1..K-1 were garbage**
(channel estimate `nzero(<0.3mean)=1755/3000`, `mean|H|` collapsing). The codewords map sequentially
across the block OFDM symbols (`clr[c*ldpc.N+i]` from `deframed` in raster order), so cw0 = early
symbols, cw7 = late symbols. The big-block is **preamble + Ngrid = ~64 OFDM symbols**, but the live RX
arms `frames_to_read = preamble_nSymb + Nsymb` = **ONE stock frame (~13 symbols)** at every data-receive
entry (e.g. arq_responder.cc:1197). The decode snapshot fires at `frames_to_read==0` — after only the
block HEAD is captured — so the late codewords read silence/stale ring -> CRC-fail. The standalone
`BIGBLOCK_LIVE` validator and CASE A-D pass because they hand the WHOLE block in a window sized to span
it; only the live ring-fed path under-waits. **FIX:** new geometry helper
`cl_telecom_system::bigblock_rx_block_nsymb()` (= preamble_nSymb + Ngrid); arm `frames_to_read` to that
block span when big-block framing is active at CFG16 (arq_responder.cc data-receive arming + a re-arm +
full-ring wipe after each big-block carve in arq_common.cc `receive()`). After this, the first block
decodes **8/8 byte-faithful**.

### §15.3 Root cause 3 — CLEAN big-block never delivered to the app FIFO
With cw0..7 all-clean, `bigblock_block_to_arq` CLEAN branch (test_bigblock_arq_unit.cc) set the K
slots `RECEIVED` and bumped bsi, but **nothing downstream marked them `ACKED` and called
`copy_data_to_buffer()`** on the live path — so the decoded sub-units never reached `fifo_buffer_rx`
(the app RX FIFO). The unit tests never noticed: they read `messages_rx[]` DIRECTLY
(`bigblock_test_delivered_*`), never the FIFO. **FIX:** the CLEAN branch now marks RECEIVED->ACKED and
calls `copy_data_to_buffer()` (mirrors the prev-batch retx delivery at arq_responder.cc:760-767), and
seeds `rsp_current_expected_batch_seq_id` from the authoritative wire bsi when it is still -1 (first
block) so the one-bsi-transition advances. A new member `bigblock_skip_fifo_delivery` (default false =
live deliver) is set TRUE by the unit-test bringups so CASE1..N / CASE A-D keep their messages_rx[]
assertions. Result: SIM_INPROC pinned-CFG16 delivers the FULL 1200-byte (one full block) message
**byte-faithful, `bytes_ok=1`, ALL PASS**.

### §15.4 Producer/consumer correction — the whitening was symmetric all along
- **TX whiten** (the ONLY TX site): `transmit_bigblock` (telecom_system.cc:8161)
  `bigblock_whiten_payload_bits(payload, Kpack*ldpc.K, BIGBLOCK_WHITEN_SEED=0x5A3C96E1)` BEFORE LDPC encode.
- **RX de-whiten** (the ONLY RX site): the carve (arq_common.cc:3892) `bigblock_whiten_bits(dw, K*sub_len*8)`
  AFTER LDPC decode. `sub_len=ldpc.K/8=175`, so `K*sub_len*8 = K*1400 = K*ldpc.K = Kpack*ldpc.K` —
  **spans MATCH, same seed, offset 0, contiguous, self-inverse.**
- `receive_bigblock` de-whitens **ZERO** times; `bigblock_rx_passband` de-whitens **ZERO** times — **no
  double de-whiten.** The stock per-frame `bit_energy_dispersal` is NOT applied to the big-block on either
  side. **Empirical proof:** once the partial-block bug was fixed, the de-whitened `payload[0..15]` matched
  the TX block payload byte-for-byte (`00 08 9b 00 9b 00 ...`) and ALL 8 per-codeword wire CRCs matched
  (`calc==wire`). Do NOT re-plumb the whitening.

### §15.5 NEW full-path regression — `--test-bigblock-fullpath` (closes the cross-layer gap, CLAUDE.md §5)
`cl_arq_controller::test_sim_inproc_bigblock_fullpath()` drives the 2-instance SIM_INPROC CFG16
big-block transfer through the **REAL** TX-encode->whiten->PHY->`receive_bigblock` de-whiten->carve->
`copy_data_to_buffer` FIFO-deliver path and asserts the full message is delivered **byte-faithful**
(every codeword clean + `payload[0]==bsi` + per-codeword CRC pass + `rx_have==payload_len`,
`bytes_ok=1`). FAIL-BEFORE/PASS-AFTER on the SAME binary via `MERCURY_BIGBLOCK_DEFEAT_FIX=1` (restores
the one-stock-frame partial-block wait; the fail-before captures the FIRST block PARTIAL outcome and
breaks immediately via `MERCURY_SIM2_STOP_AFTER_FIRST_BLOCK=1` to avoid the slow/unstable post-partial
retry loop). The sibling UAF fail-before is reproduced separately by `MERCURY_BIGBLOCK_DEFEAT_FIX_UAF=1`
(SEGV — cannot share a process with pass-after, documented not folded in). **Result:**
`FAIL-BEFORE: first_block clean=1/8 rx_have=0/1200 full=0` ; `PASS-AFTER: first_block clean=8/8
rx_have=1200/1200 full=1` ; **ALL PASS**. This is the test the synthetic CASE A-D could not catch:
they hand a caller-owned `std::vector` as the RX passband (no UAF), drive one `receive_bigblock`
directly (no per-block wait), and assert on `messages_rx[]` directly (no FIFO delivery).

### §15.6 Regression (all FOREGROUND, green)
`--test-bigblock-fullpath` ALL PASS (fail-before/pass-after) · `--test-sim-inproc-bigblock` ALL PASS
(CASE A-D) · `--test-bigblock-arq-unit` 8/8 (T1-T9) · `--test-climb-engine` 0 failures ·
`MERCURY_BIGBLOCK_LIVE=1 -m PLOT_PASSBAND -s 16` -> 8/8 byte-correct, post_FEC_BER=0. **Net-PHY
UNCHANGED** (K=8, sub_len=175, block_samples=79360 at CFG16 — the fix is buffer-lifetime + RX-wait +
FIFO-delivery, NOT geometry); the §14 net-PHY > VARA 7050 holds.

### §15.7 Open — multi-block sequencing (NOT a whitening/carve issue, out of scope here)
A SINGLE full block (<=1400 bytes) delivers byte-faithful. A MULTI-block payload delivers block 1 CLEAN
then block 2 acquires the WRONG window (decodes garbage, every CRC fails) — a SACK/ACK-turnaround +
SIM single-symbol-pacing cadence interaction across blocks, NOT the whiten/carve path (block 1 proves
those correct). This was never reachable before (the UAF crashed the first block). Tracked as a
follow-on for the sim-cadence fact docs; the byte-faithful single-block delivery is the deliverable here.

NO monitor merge, NO push, NO attribution.

---

## §16. RUNG ELECTION ON THE GEARSHIFT CLIMB — close the last "not-wired-into-the-gearshift (P4)" gap (2026-06-06, branch `fix/bigblock-rung-election` off `integ/bigblock-climb-2026-06-06`@`b5fa65d`, worktree `C:/Users/kamer/mercury_wt/bigblock-election`)

**Symptom (HW-confirmed on `integ/bigblock-climb-2026-06-06`@`b5fa65d`):** the climb
reaches CFG16 via the `-R` turbo path (negotiated/data_cfg/current=16, SNR up=15.0)
and the af5f012 RX carve runs on HW (13× `[BIGBLOCK-RX] carve K=8 cw_ok=8`), but the
big-block delivers **0/1200** because **the big-block rung is never ELECTED on the
climb path**: CMD has 0 `[BIGBLOCK-TX]`, `bigblock_rung=0` on BOTH peers, every block
PARTIAL `clean=0`.

**Root cause:** `sack_negotiated_recompute_batch()` (`arq_common.cc:944`) — which pins
`data_batch_size=K` (the R-B/#9 pin, §16.4 INV-5), the precondition for the clean-ACK
`all_ones` mask to equal the K-bit big-block bitmap — is invoked ONLY at CONNECT
negotiation (`arq_commander.cc:4263` TEST_CONNECTION_ACK, `:7131/:7141/:7175/:8987`;
`arq_responder.cc:2204` TEST_CONNECTION) and from the synthetic-fire/unit-test paths.
It is **never re-invoked when the gearshift switches the live config to CONFIG_16 on
the climb.** On a robust/`-R` connect the negotiation ran at ROBUST_0 (`current_configuration`
was ROBUST_0 → the `bigblock_rung` branch's `current_configuration==CONFIG_16` guard was
false → no K-pin), and nothing re-elects the rung when the climb later lands on CFG16.
So at CFG16 the climb holds the stock OFDM 30s batch (`data_batch_size≈25`) while the
RX carve emits a K=8 bitmap → CMD `all_ones=(1<<25)-1=0x1FFFFFF ≠ 0xFF` → zero clean
credit → every block PARTIAL. This is the documented "Not wired into the gearshift
(that is P4)" gap (`telecom_system.cc:80`, `arq_common.cc:976-983` comment).

**This audit covers the NEW PRODUCER** of `data_batch_size` (the gearshift-transition
election trigger) and the NEW reader-relevant transition of the election condition
`bigblock_rung = bigblock_framing_enabled && current_configuration==CONFIG_16`. The §1/§2
producer/consumer lists and §4 invariants below are EXTENDED, not replaced.

### §16.1 Producers — `bigblock_rung` (the election condition) and `data_batch_size`

`bigblock_rung` is NOT a stored member; it is a derived predicate recomputed wherever
the big-block path gates. Its two inputs:

- `telecom_system->bigblock_framing_enabled` (bool member, `telecom_system.h`).
  Producers: ctor env-read `MERCURY_BIGBLOCK_FRAMING` (`telecom_system.cc:81-82`, set
  ONCE, persistent); `transmit_bigblock`/`bigblock_*` self-clear on a degenerate block
  (`telecom_system.cc:8346/8363`); the unit-test direct sets (`test_bigblock_arq_unit.cc`,
  test-only). It is a "framing-mode PERMITTED" flag — both peers carry the SAME value
  (both have the env var in a session); it does NOT toggle per-config.
- `current_configuration` (int member). SOLE production producer:
  `cl_arq_controller::load_configuration()` writes it at `arq_common.cc:1512`
  (`this->current_configuration=configuration`) on EVERY config switch — connect,
  every gearshift up/down step, BREAK→ROBUST_0, turbo-reverse, NB switch. This is the
  single chokepoint every gearshift transition flows through, on BOTH the CMD and RSP
  side (each peer runs its own `load_configuration` from its own gearshift logic).

`data_batch_size` producers (full list in `data-flow-batch-size.md` §2; setter
chokepoint `set_data_batch_size()` `arq_common.cc:751`):
- `load_configuration()` itself: FULL-load seed `default_configuration_ARQ.batch_size`
  (`:1567`); robust pin `set_data_batch_size(1)` (`:1630`); OFDM 30s-scaling
  `set_data_batch_size(fixed_batch)` where `fixed_batch=sack_enabled?radio_batch_size(25):10`
  (`:1656`).
- `sack_negotiated_recompute_batch()` (`arq_common.cc:944`): at the bigblock rung pins
  `set_data_batch_size(K)`, `K=telecom_system->bigblock_codeword_count()` (`:986-991`);
  else (non-robust, non-bigblock) the 30s formula (`:994-1004`).
- Axis-2 adaptive controller `policy_evaluate_axis2()` (`arq_commander.cc:5769` up/down
  ±5) + RSP adopt of `SET_LINK_PARAMS` (`arq_responder.cc:2714`). **THE SIBLING RISK,
  see §16.5.**
- Robust dwell op + connect-path recomputes (§2 of `data-flow-batch-size.md`).

**NEW PRODUCER (this fix):** at the END of `load_configuration()` (after all batch/timing/
buffer init is settled, `arq_common.cc:~1824`), when the just-loaded config makes
`bigblock_rung` TRUE, call the SHARED election body `sack_negotiated_recompute_batch()`,
which re-pins `data_batch_size=K`. Because `load_configuration()` runs on BOTH peers at
their respective CFG16 transitions and `bigblock_framing_enabled` + `bigblock_codeword_count()`
are identical on both, the election fires SYMMETRICALLY and elects the SAME K (the
divergence-proof property — same shared body, same geometry source). This is exactly the
T6 election-symmetry contract, now fired by the gearshift transition instead of only by the
connect handlers.

### §16.2 Consumers — unchanged set, re-verified at the new producer

The election sets `data_batch_size=K`; its consumers are the §2.7 / `data-flow-batch-size.md`
§3 set. The divergence-sensitive one (§2.5 / `data-flow-batch-size.md` §3.1): the clean-ACK
`all_ones` target `(1<<data_batch_size)-1` computed independently on each side
(`cmd_clean_data_ack_crc_valid` `arq_commander.cc:136-139`; the RSP big-block bitmap is the
K-bit `cw_ok`). With both peers at `data_batch_size==K`, both derive `0xFF` and match the
emitted bitmap. The TX switch `bigblock_send_one_block()` (`arq_common.cc:3648`) and the RX
carve gate (`arq_common.cc:6888`) self-gate on `bigblock_framing_enabled && M!=MFSK &&
current_configuration==CONFIG_16` — they need NO extra wiring; they engage automatically once
the config is CFG16. The ONLY missing piece was the batch-size election, which keeps their
K-bit SACK in sync with the ARQ clean-ACK gate.

### §16.3 Valid states — especially the climb seed BEFORE the election fires

| moment | `bigblock_framing_enabled` | `current_configuration` | `data_batch_size` | bigblock TX/RX engage? | clean-ACK works? |
|---|---|---|---|---|---|
| robust connect (`-R`) | true (env) | ROBUST_0 | 1 (robust pin) | no (MFSK + !CFG16) | n/a (per-frame MFSK) |
| climbing CFG6..15 | true | CFG6..15 | 25 (30s scale) | no (!CFG16) | stock OFDM per-frame |
| **lands on CFG16 — PRE-FIX** | true | CFG16 | **25 (stale 30s)** | **TX/RX yes (auto-gated)** | **NO — all_ones 0x1FFFFFF ≠ 0xFF (bug #9)** |
| **lands on CFG16 — POST-FIX** | true | CFG16 | **K=8 (election)** | TX/RX yes | **YES — all_ones 0xFF == bitmap 0xFF** |
| later Axis-2 up-move at CFG16 (PRE §16.5 guard) | true | CFG16 | 8→13 (un-pinned) | TX yes (block emits min(K,batch)), RX K=8 | **NO — all_ones 0x1FFF ≠ 0xFF (re-diverges!)** |
| later Axis-2 (POST §16.5 guard) | true | CFG16 | held K=8 | yes | YES |

The PRE-FIX "lands on CFG16" row is the exact HW-observed `bigblock_rung=0`/clean=0 failure.
The Axis-2 row is the latent SIBLING bug §16.5 closes in the same change.

### §16.4 Invariants — INV-5 extended to the gearshift transition

**INV-5 (CMD==RSP batch symmetry at the rung)** (§4): `data_batch_size==K` on BOTH peers
at the bigblock rung so the `all_ones` target matches the emitted bitmap. *Producer
obligation EXTENDED:* in addition to the connect handlers, the gearshift CFG16 transition
must elect K on BOTH peers. Maintained because the SHARED election body runs from the SINGLE
`load_configuration` chokepoint each peer hits at its CFG16 transition, with identical inputs
(`bigblock_framing_enabled`, `bigblock_codeword_count()`), so it CANNOT diverge.

**INV-5b (NEW — the bigblock-rung batch is geometry-locked, not link-adaptive):** once the
rung is elected, `data_batch_size` MUST remain == K for the lifetime of the CFG16 dwell.
Any mid-session producer that moves it away from K re-opens bug #9. *Producer obligation:*
suppress the Axis-2 adaptive batch controller at the bigblock rung (§16.5). The K-pin is a
PHY-geometry fact (K = `nBits/ldpc.N` codewords per acquisition), not a link-quality knob.

All other invariants (INV-1..INV-4, INV-6..INV-8) are PHY/carve invariants the election does
not touch — it only sets `data_batch_size`; the carve, EOB, bsi, and selective-repeat paths
are unchanged.

### §16.5 SIBLING BUG found by the audit — Axis-2 can un-pin K mid-session (CLAUDE.md §5)

`policy_evaluate_axis2()` (`arq_commander.cc:5589`) is the OFDM adaptive batch controller.
It is gated OFF only for robust configs (`if(is_robust_config(current_configuration)) return;`
`:5608`). At CFG16 it RUNS: after `AXIS2_UP_GOOD_RUN=4` clean batches it steps
`data_batch_size += AXIS2_STEP(5)` up to `AXIS2_BATCH_CEIL`, and `set_data_batch_size(13)`
accepts it (non-robust branch), then `SET_LINK_PARAMS` pushes the RSP to the same value
(`arq_responder.cc:2714`, clamped `[10,32]`). That moves BOTH peers off K=8 → `all_ones`
becomes `(1<<13)-1` while the carve still emits a K=8 bitmap → re-diverges (bug #9), mid-session.
This is latent (the pinned `-s16` path is blocked by a separate CONNECT control-ACK no-decode
bug and never ran long enough for Axis-2 to fire; T6 runs the election in isolation), but it
is on the exact path this fix is about to make live. **Fix (root cause, mirrors the robust
guard at `:5608`):** early-return `policy_evaluate_axis2()` at the bigblock rung
(`bigblock_framing_enabled && current_configuration==CONFIG_16`) so K stays geometry-locked.
This is NOT a threshold tune — the bigblock rung's batch is dictated by codeword geometry, so
the adaptive controller has no valid axis to act on there.

### §16.6 What the fix changes (walk every consumer)

1. **NEW gearshift-transition election (`load_configuration` tail).** Alters the §1.8 / §16.1
   producer set: `data_batch_size` is now re-pinned to K whenever a config switch lands on
   CFG16 with framing on. Consumers: the clean-ACK `all_ones` gate (§2.5) now sees `0xFF` at
   CFG16 (was `0x1FFFFFF`); `recalculate_ack_timeout_for_batch()` (called inside the body)
   re-sizes the data-ACK timeout for batch=K. The TX/RX big-block gates (§16.2) are unchanged
   (already config-gated). Stock per-frame path (non-CFG16, or framing off) is BYTE-IDENTICAL
   — the body's outer guard `bigblock_framing_enabled && current_configuration==CONFIG_16`
   is false there and the call early-returns to the stock 30s/robust branch already run by
   `load_configuration`. ✓
2. **NEW Axis-2 suppression at the bigblock rung.** Alters the §16.1 Axis-2 producer: it no
   longer moves `data_batch_size` at CFG16-with-framing. Consumer of Axis-2's output
   (`SET_LINK_PARAMS` → RSP adopt) simply never fires at that rung, so the two peers stay at
   K. Every OTHER config (CONFIG_0..15, framing off) is UNCHANGED — Axis-2 runs exactly as
   before. ✓
3. **Structures explicitly NOT changed:** the optimizer (`optimizer_is_in_control()`
   `arq.h:2041-2058`, `last_data_viable_config`, `anchor_consec_break_fails`,
   `probe_backoff`), the carve, EOB, bsi, retx queue. This is a batch-size election at the
   CFG16 rung, NOT an authority/gearshift change — the gearshift still owns config selection;
   we only react to its CFG16 transition. ✓

### §16.7 Symmetry argument (the R-B / #9 contract)

The election fires from `load_configuration()`, which BOTH peers execute independently when
their own gearshift drives them to CFG16. The body reads only `bigblock_framing_enabled`
(identical: both have `MERCURY_BIGBLOCK_FRAMING=1`) and `bigblock_codeword_count()` (identical
PHY geometry, the same `nBits/ldpc.N` the TX/RX workers use; deterministic, capped by
`MERCURY_BIGBLOCK_K`). There is no link-quality, timing, or peer-specific input. Therefore
both peers elect the SAME K at their CFG16 transition and the all-ones targets match — the
divergence-proof property the connect-path election already had, now extended to the climb.
(Empirically asserted by the new `--test-bigblock-climb-election` symmetry case and by T6.)

### §16.8 Paired regression — `--test-bigblock-climb-election`

New in-process case (`bigblock_test_climb_election()`, `test_bigblock_arq_unit.cc`, fired by
the `--test-bigblock-climb-election` CLI flag) following the T6 pattern but proving the
election fires from the GEARSHIFT TRANSITION, not from an explicit `sack_negotiated_recompute_batch()`
call:
- Build two instances (CMD + RSP). Seed the bug-#9 state: `load_configuration(CONFIG_15,FULL,YES)`
  (30s formula → `data_batch_size=25` on both), framing OFF at CFG15 (rung not elected).
- Set `bigblock_framing_enabled=true` (the persistent env-style flag, both peers).
- **Fire the gearshift transition:** `load_configuration(CONFIG_16, FULL, YES)` — the SAME
  entry the climb uses. Assert the transition ELECTED the rung: `data_batch_size==K==8`
  on BOTH peers, symmetric, `all_ones==0xFF`. **fail-before** (`MERCURY_BIGBLOCK_DEFEAT_ELECTION=1`
  skips the new tail call): the transition leaves `data_batch_size=25`, `all_ones=0x1FFFFFF` →
  assertion fails. **pass-after**: election fires → 8/0xFF.
- Then drive the REAL emit+deliver: `bigblock_send_one_block()` returns true at CFG16 (the TX
  switch engages), the block goes through the production carve, and the RX delivers byte-faithful
  (re-using the CASE-A loopback + carve). This proves the elected rung actually EMITS and DELIVERS,
  not merely that the batch number changed.

Also extended `--test-bigblock-fullpath` doc note: the gearshift-transition seam is now covered.

### §16.9 Build & regression result

Build `o3` clean (71 files, 0 errors; only pre-existing vendored-audio format warnings).
All FOREGROUND, GREEN:

- **`--test-bigblock-climb-election`** (NEW): `ALL PASS (0 failures)`.
  - FAIL-BEFORE (`MERCURY_BIGBLOCK_DEFEAT_ELECTION=1`): the CFG15→CFG16 transition does
    NOT elect → `cfg16 batch cmd=25 rsp=25`, `cmd_all_ones=0x1FFFFFF` (≠ 0xFF) — the bug-#9
    observable reproduced.
  - PASS-AFTER (default env): the CFG16 transition ELECTS `batch cmd=8 rsp=8`
    (symmetric, `all_ones cmd=0xFF rsp=0xFF`) with NO explicit election call; the elected
    rung's TX gate engages and a big-block EMITS (`K_tx=8 n_tx=79360`) + DELIVERS
    byte-faithful (`recv=8/8 delivered=622/622`, `carve_rc=0`).
- **`--test-bigblock-arq-unit` (T1–T9)**: `8/8 cases passed`. T6 election-symmetry
  (the R-B/#9 gate): `cmd_batch=8 rsp_batch=8 K=8 | cmd_all_ones=0xFF rsp_all_ones=0xFF
  rsp_bitmap=0xFF | gate_match=1 dedupe_ok=1`.
- **`--test-bigblock-fullpath`**: `ALL PASS` (FAIL-BEFORE first_clean=1/8 rx=0 ;
  PASS-AFTER first_clean=8/8 rx=1200/1200) — the live 2-instance pinned-CFG16 big-block
  delivery is unregressed (the election ALSO fires on its CFG16 setup, still green).
- **`--test-sim-inproc-bigblock` (CASE A–D)**: `ALL PASS` — carve/selective-repeat/
  heap-guard/wire-CRC unchanged.
- **`--test-climb-engine`**: `ALL PASS (0 failures)` — the Axis-2 bigblock-rung suppression
  (§16.5) and the election wiring do not regress the gearshift/climb/dwell logic.
- **Net-PHY UNCHANGED**: `MERCURY_BIGBLOCK_LIVE=1 -m PLOT_PASSBAND -s 16` →
  `codewords_decoded=8/8 post_FEC_BER=0 infoerr=0/11200 VERDICT=PASS`, K=8,
  block_samples=79360 — byte-identical to the §15.6 baseline. The fix is ARQ batch-size
  election + Axis-2 gating only; PHY geometry is untouched.

**Sim climb (Option-b limitation):** the single-thread in-process 2-instance sim is the
documented OFF-bench validator, but it cannot drive the multi-batch ROBUST/CFG15→CFG16
turbo climb to completion (the `-R` turbo path + multi-batch climb is not reproduced by the
lockstep stepper; the fullpath test PINS CFG16 for exactly this reason). The election seam
is therefore validated by the SYNTHETIC `--test-bigblock-climb-election` test, which fires
the EXACT `load_configuration(CONFIG_16)` gearshift entry the climb uses and asserts
fail-before/pass-after election + symmetric K + emit + byte-faithful delivery. This is the
CLAUDE.md §3-compliant "fails before / passes after" proof for the wiring seam.

## §17. MULTI-CODEWORD CORRUPTION ON THE LIVE CLIMB — the §15.2 partial-block window fix was INCOMPLETE; the per-block ACK-turnaround re-arms (and the data-path FAIL/anti-spin re-arm) are NOT big-block-aware (2026-06-06, branch `fix/bigblock-multicw-dewhiten` off `fix/bigblock-rung-election`@`984c43b`, worktree `C:/Users/kamer/mercury_wt/bigblock-multicw`)

**Symptom (HW-confirmed on `integ/bigblock-rung-election`@`984c43b`):** after the §16 rung
election lands and the gearshift reaches CFG16 with both peers electing the big-block rung,
the RX carves `13× [BIGBLOCK-RX] carve K=8` but delivers **0 byte-faithful app bytes**. The
NOCRC isolation is decisive: with the per-cw wire-CRC ON, every block is PARTIAL `clean=0`
(cw1..cw7 demoted — the CRC is CORRECTLY rejecting genuinely-corrupt codewords); with
`MERCURY_BIGBLOCK_NOCRC=1` the blocks go CLEAN and "deliver" but the bytes are WRONG (recv
head `fa66a8..` ≠ sent head `32ab98..`, the 1200B block not found at any offset, `wire_bsi`
reads garbage). **cw0 is byte-correct** (`used_hdr=1`, `wire_bsi` correct) — only cw1..cw7 are
corrupt → a PER-CODEWORD signature.

### §17.1 NOT a whiten / per-cw offset / stride misalignment — REFUTED (again) by reproduction

The task framing was a "TX-encode vs live-RX de-whiten / payload-mapping MISALIGNMENT for the
MULTI-codeword case." This is **REFUTED**, four ways:
1. The §15.4 producer/consumer trace: TX whiten span `Kpack*ldpc.K` @ seed `0x5A3C96E1` offset 0
   == RX de-whiten span `K*sub_len*8 = K*ldpc.K` @ same seed/offset; self-inverse; `Kpack==K==8`;
   `ldpc.K=1400` is an exact multiple of 8 so codeword `c`'s info bits land at the exact byte
   boundary `c*sub_len = c*175`. No per-cw offset/stride asymmetry exists in the source (cw0's
   +18 header offset is INSIDE cw0's own `[0..175)` region and does NOT shift cw1..7, which start
   at the fixed boundary `N*175`).
2. **`--test-bigblock-fullpath` (PAYLOAD=1200, a FULL K=8 single block spanning all 8 codewords)
   PASSES 8/8 byte-faithful** through the REAL `transmit_bigblock`→whiten→PHY (cl_sim_awgn channel,
   distorted not noiseless)→`receive_bigblock`→de-whiten→carve→FIFO path. If the whiten/offset/stride
   were wrong, this full-K=8 single block could not deliver byte-faithful. It does.
3. The candidate fix (swap `bigblock_whiten` for stock `bit_energy_dispersal` per codeword) would
   NOT change the corruption: the corrupt bytes come from a stale-ring DECODE WINDOW (§17.2), which
   NO whitener can recover — de-whitening stale bits yields different-but-still-wrong bits. We did
   NOT swap the whitener (`used_stock_dispersal=false`); the whiten is correct and TX/RX-matched.
4. EMPIRICAL: `MERCURY_BIGBLOCK_DEFEAT_FIX=1 MERCURY_BIGBLOCK_NOCRC=1` (PIN CFG16, PAYLOAD=1200,
   STOP_AFTER_FIRST_BLOCK) reproduces the EXACT HW signature in the faithful sim:
   `[BIGBLOCK-RX] carve: K=8 cw_ok_count=8 wire_bsi=0 (used_hdr=1) ... rx_have=1200/1200 bytes_ok=0`
   — cw0 correct, block "delivered", bytes WRONG. The SAME run with the full window (DEFEAT_FIX=0)
   delivers `clean=8/8 bytes_ok=1`. The ONLY variable is the RX capture-window size.

### §17.2 ROOT CAUSE — the big-block decode snapshots a STOCK-FRAME-sized capture window after every per-block ACK turnaround

`receive_byte` routes to `receive_bigblock` whenever `bigblock_framing_enabled && M!=MFSK &&
current_configuration==CONFIG_16` (telecom_system.cc:1024-1026), regardless of window size.
`receive_bigblock` decodes `nSamples = Nofdm * buffer_Nsymb * interp` (telecom_system.cc:8219) —
it reads `buffer_Nsymb` symbols from the ring. But the SNAPSHOT (capture-prep handoff) fires only
when `frames_to_read` counts down to 0 (capture-prep thread on HW; `sim2_deliver_from_wire`
arq_commander.cc:9931 in the sim). So `frames_to_read` controls HOW MANY FRESH SYMBOLS are
accumulated into the ring before the block is handed to the decoder.

The big-block spans `bigblock_rx_block_nsymb() = preamble_nSymb(4) + Ngrid(60) = 64` OFDM symbols
(telecom_system.cc:8073-8088). The stock CFG16 frame is `rx_frame = get_active_nsymb(9) +
preamble_nSymb(4) = 13`. Codewords map SEQUENTIALLY across the block's OFDM symbols (cw0 = earliest
symbols, cw7 = latest — `clr[c*ldpc.N+i]` in deframer raster order, telecom_system.cc:7442). So when
`frames_to_read` is armed to a STOCK FRAME (~13–23 symbols) instead of the block span (~74), the
snapshot fires after only the block HEAD is freshly captured: cw0's symbols are fresh → cw0 decodes
byte-correct; cw1..cw7's symbols are NOT yet in the ring (silence / stale / the post-carve full-ring
wipe at arq_common.cc:6986) → cw1..cw7 decode from non-signal → DETERMINISTIC garbage.

### §17.3 Why §15.2's fix was INCOMPLETE — the unguarded sibling re-arm sites (CLAUDE.md §5)

§15.2 added `bigblock_rx_block_nsymb()` and guarded TWO `frames_to_read` writers:
- `arq_responder.cc:1208-1219` — the messages_control TURNAROUND arming (the FIRST block).
- `arq_common.cc:6972-6994` — the POST-CARVE re-arm (gated on `bigblock_last_rx_K>0`, a successful
  prior carve).

It MISSED every per-block ACK-turnaround re-arm in the SUSTAINED data path. On the live HW climb a
multi-block session sends a data/SACK ACK after EACH block; the ACK-send tail flushes the ring and
re-arms `frames_to_read` to the STOCK FRAME with NO big-block awareness, CLOBBERING the post-carve
`block_nsymb+10` arming for the NEXT block. The unguarded sibling sites (the NEW producers this audit
adds to the §1/§16.1 producer list for `frames_to_read` at the bigblock rung):
- `send_ack_pattern()` `arq_common.cc:4899-4903` (`[TX-ACK-PAT]`): `frames_to_read = rx_frame + 10`.
  **THE primary per-block turnaround in the main RSP data path.**
- `send_ack_pattern_with_snr()` `arq_common.cc:5010-5014` (`[TX-ACK-SNR]`): same.
- the SACK-ACK send tail `arq_common.cc:5481-5485` (`[TX-MFSK-ACK-SACK]`): same.
- `arq_responder.cc:1915-1920` (WB data-ACK reload): `frames_to_read = preamble_nSymb + Nsymb + 10`.
- `arq_responder.cc:1677-1681` (ACK-GATE partial-batch retx reset): `frames_to_read = preamble_nSymb
  + get_active_nsymb()`.
- the data-path OFDM-FAIL/anti-spin re-arm `arq_common.cc:7557` (`ftr=8/2`) and `:7879-7897`
  (`frames_to_read = rx_frame`). These fire on a FAILED big-block decode (`message_decoded==NO`,
  which the carve forces) when `frames_to_read` has counted to 0 — re-arming a stock frame so the
  NEXT block is truncated again (the chicken-and-egg recurrence: only a CLEAN carve arms the full
  window via :6994, but a truncated window cannot produce a clean carve).

### §17.4 The fix — one shared clamp helper at every data-path re-arm (reuse the §15.2 mechanism)

Per CLAUDE.md §2 (reuse the existing aligned mechanism; no parallel band-aid), add ONE helper
`cl_arq_controller::bigblock_block_ftr_or(int stock_ftr)` that returns
`max(stock_ftr, telecom_system->bigblock_rx_block_nsymb()+10)` when the bigblock rung is active
(`telecom_system->bigblock_framing_enabled && telecom_system->M!=MOD_MFSK &&
current_configuration==CONFIG_16`) and `stock_ftr` UNCHANGED otherwise. Wrap EVERY data-path
`frames_to_read = <stock>` re-arm at the sites in §17.3 in this helper. The helper reads the SAME
`bigblock_rx_block_nsymb()` the §15.2 sites use — no new geometry source, divergence-proof. On every
non-CFG16 / framing-off / MFSK path the guard is false → the helper returns the stock value → the
binary is BYTE-IDENTICAL to baseline (verified: net-PHY + stock per-frame configs unchanged).

REPRODUCER HOOK: `MERCURY_BIGBLOCK_DEFEAT_FIX=1` is extended to ALSO bypass the new clamp (helper
returns `stock_ftr` when the env is set), so the SAME binary reproduces the pre-fix stock-frame
arming for the fail-before/pass-after A/B. Production never sets it.

### §17.5 S5 cross-layer audit — `frames_to_read` at the bigblock rung

1. **Producers** (writers of `frames_to_read` reachable at CFG16+framing during a data session):
   the §17.3 list + the §15.2-guarded `:1208`/`:6994` + the success re-arm `arq_common.cc:7126`
   (gated `message_decoded==YES`, NOT taken for big-block — the carve forces NO at :6960, verified).
   The capture-prep COUNTDOWN producer (`dc->frames_to_read--`, arq_commander.cc:10001 sim;
   capture-prep thread on HW) is unchanged — it only DECREMENTS; the clamp raises the ARMED value.
2. **Consumers**: the snapshot trigger `frames_to_read==0` (sim `sim2_deliver_from_wire`
   arq_commander.cc:9931; HW capture-prep). `receive_bigblock` reads `buffer_Nsymb` samples
   (telecom_system.cc:8219), so the consumer needs the FULL block accumulated before the snapshot —
   the invariant the clamp restores.
3. **Valid states** before any producer writes at the rung: `frames_to_read` defaults to a
   stock-frame seed from `load_configuration` (the §16.3 climb seed). The clamp raises it to the
   block span the first time a data-path re-arm runs at CFG16+framing — so even the FIRST block
   (which on HW never routed through the `:1208` control arming on the climb) gets the full window.
4. **Invariant the consumer assumes** (NEW — INV-9b): at the bigblock rung, the armed
   `frames_to_read` MUST be ≥ `bigblock_rx_block_nsymb()` so the snapshot waits for the whole block.
   Every producer in §17.3 now maintains it via the clamp. `buffer_Nsymb` (=128 at CFG16) ≥ the
   block span (74), so the ring can hold a full block — the window was the only deficient axis.
5. **What the fix changes**: only the ARMED window magnitude at the rung. It does NOT touch the
   carve, whiten, de-whiten, per-cw CRC, bsi, EOB, selective-repeat, batch-size election (§16), or
   the optimizer. Stock per-frame configs (0..15) and MFSK are byte-identical (guard false).

### §17.6 NOT block-spanned — the partial-batch retx capture (intentionally stock-frame)
`arq_responder.cc:1677-1681` (ACK-GATE partial-batch reset) is NOT wrapped in the clamp. When a
big-block is PARTIAL (some cw demoted by the wire-CRC), the SACK retransmit is sent via STOCK
per-frame framing — `bigblock_send_one_block()` declines while `sack_retransmit_active`
(arq_common.cc:3718) — so the RX legitimately expects stock per-frame frames during the retx
window. Block-spanning that arming would over-wait for a block that is not coming. The NEXT
NEW-DATA batch (a fresh big-block) is re-armed by the ACK-send paths (§17.3, now fixed), so the
next block still gets its full window. (Audited against `data-flow-batch-size.md` §3 / the retx
queue; the retx path is per-frame, the new-data path is the block.)

### §17.7 Reproduction + validation (faithful sim, fail-before / pass-after)
The deterministic faithful reproducer (no HW, no climb needed): `MERCURY_SIM_2INST=1 -m SIM_INPROC`
PIN CFG16 framing-on PAYLOAD=1200 (full K=8 block) STOP_AFTER_FIRST_BLOCK=1, NOCRC=1:
- FAIL-BEFORE (`MERCURY_BIGBLOCK_DEFEAT_FIX=1`, stock window): `[BIGBLOCK-RX] carve: K=8
  cw_ok_count=8 wire_bsi=0 (used_hdr=1) ... rx_have=1200/1200 bytes_ok=0` — cw0 correct, the
  block forced CLEAN and "delivered", but the BYTES ARE WRONG (the HW signature).
- PASS-AFTER (DEFEAT_FIX=0, block window): `... rx_have=1200/1200 bytes_ok=1` — byte-faithful.
The ONLY variable is the armed `frames_to_read` window — proving the root cause and refuting the
whiten/offset theory (§17.1.3).

The paired regression `--test-bigblock-multicw` (`cl_arq_controller::test_sim_inproc_bigblock_multicw()`,
arq_commander.cc) drives a FULL K=8 block (1200B, all 8 codewords) in THREE arms in one process:
(A) CRC-ON block-window -> `clean=8/8` (per-cw CRC PASSES all 8, NOT demoting) + byte-faithful;
(B) NOCRC stock-window (DEFEAT_FIX=1) -> forced-clean but `bytes_ok=0` (cw1..cw7 corrupt);
(C) NOCRC block-window -> `bytes_ok=1`. This is the K>1 byte-faithfulness test the 622-byte synthetic
cases and the single-arming fullpath could not catch. NO monitor merge, NO push, NO attribution.

### §17.8 The §17 audit was STILL incomplete — the SWITCH_ROLE bidirectional re-arm + the multi-block sim limitation (2026-06-06, branch `fix/bigblock-rearm-window` off `fix/bigblock-multicw-dewhiten`@`6e08f2e`, worktree `C:/Users/kamer/mercury_wt/bigblock-rearm`)

The §17.3 producer list enumerated the RSP-direction data-path re-arms but MISSED one sibling on the
COMMANDER side: the **SWITCH_ROLE turnaround re-arm** (`arq_commander.cc:4618-4637`, in
`process_control_commander()`). On a turboshift role reversal the commander becomes the
RESPONDER/receiver: `data_configuration` was (re)loaded for the return path at `arq_commander.cc:4565`
and MAY be CONFIG_16 + big-block framing, and this peer then awaits the new TX side's FIRST DATA —
which at the big-block rung is a full block (`bigblock_rx_block_nsymb()` ~64 OFDM symbols). The
pre-fix arming was a STOCK frame (`rx_frame + 10`), so the snapshot fired on the block HEAD only:
cw0 fresh → byte-correct, cw1..cw7 from the stale ring → the §17.2 truncated-window signature on the
REVERSE / bidirectional path. The single-direction RSP tests never exercised it.

**FIX**: route `arq_commander.cc:4635` through `bigblock_block_ftr_or()` — the SAME shared clamp the
§17.4 sites use (no new geometry source; byte-identical off-rung: framing-off / M==MFSK /
config != CONFIG_16 → returns `rx_frame+10` unchanged). This is the only code change beyond §17.4.

**§17.6 re-confirmed** (the comment at `arq_responder.cc:1677-1689` was elaborated, behaviour
UNCHANGED): the ACK-GATE partial-batch retx re-arm (`arq_responder.cc:1690`) stays STOCK-frame —
`bigblock_send_one_block()` declines while `sack_retransmit_active` (`arq_common.cc:3718`, CMD sets it
`arq_commander.cc:1803`), so selective-repeat retx is per-frame and the RX legitimately expects stock
frames during the retx window. NOT a gap. The CLOSE_CONNECTION / disconnect re-arms
(`arq_responder.cc:1418`, `arq_commander.cc:500`, `arq_commander.cc:4973`) reset the config to the
init seed before LISTENING, so the helper guard is false there too — correctly stock.

**MULTI-BLOCK SIM LIMITATION (resolves §8 [?])**: an attempt to add a SECOND-block re-arm regression
(driving PAYLOAD ≥ 2 blocks with a `MERCURY_SIM2_STOP_AFTER_NBLOCKS` hook + per-block clean tracking)
was BUILT then REMOVED as non-viable. Empirically (this worktree, NOCRC + CRC-ON, PAYLOAD=2400,
PIN CFG16 framing-on): the in-process 2-instance sim CARVES both blocks (block 2 `cw_ok_count=8`,
`fallback=1` bsi) but **never reaches `loop done` / `bytes_ok`** — it spins indefinitely on block-2
sequencing and never delivers the full message. The sim is effectively SINGLE-BLOCK-LIMITED for
end-to-end byte-faithful delivery. Worse, `cw_ok_count`/`n_clean` tracks the per-cw CRC pass (under
NOCRC always K, regardless of window), NOT byte-truth — so a `blocks_all_clean` assertion does NOT
differentiate fail-before from pass-after for block 2. A multi-block test on that signal would either
HANG (full-payload `memcmp` never completes) or FALSE-GREEN (cw_ok_count). The dangling infra was
therefore removed rather than shipped (CLAUDE.md: no false-green / hanging tests, no dead code).

**VALIDATION of this fix**: the window-arming root cause IS proven byte-truthfully by the EXISTING
`--test-bigblock-multicw` (ARM-B `bytes_ok=0` fail-before vs ARM-C `bytes_ok=1` pass-after, full K=8
~64-sym block via the REAL `test_sim_inproc_2` wire path, which exercises `send_ack_pattern`
[TX-ACK-PAT @4942]) and `--test-bigblock-fullpath` (first_clean 1/8 rx=0 → 8/8 rx=1200/1200). The
SWITCH_ROLE site itself is UNREACHABLE in-process — the sim PINs the config (`MERCURY_SIM2_PIN=1`,
turboshift OFF, SWITCH_ROLE count = 0 observed), so role reversal never fires. The SWITCH_ROLE wrap
is a FAITHFUL application of the §17.4-proven mechanism to the audit-found sibling; its live
role-reversal path is validated at P3/P4 HW. NO monitor merge, NO push, NO attribution.

## §18. TX-LEVEL PARITY — the big-block transmits HOT on real HW; route it through the IDENTICAL conditioning chain a stock CFG16 OFDM DATA frame uses (2026-06-06, branch `fix/cfg16-controlack-hold`, worktree `C:/Users/kamer/mercury_wt/cfg16-hold`, on `464e1fa` + the uncommitted pre-eq/level-cal change)

### §18.1 HW symptom + the loopback red herring

User watching the REAL bench scope (2026-06-06): the big-block frames run **~1400-1450 mVp-p**
(~+3.2 dB) over the 1000 mVp-p calibrated sweet spot, while ACKs and stock OFDM frames sit at
calibration. Regular OFDM decodes fine on HW; the big-block does not. A prior loopback measurement
(agent a3d777f5) reported the block ~6 dB QUIETER — that is NON-REPRESENTATIVE: `telecom_system.cc`
comment at the OFDM TX scaling notes "OFDM TX_SIG gain folded into level cal; loopback uses raw
level". Trust the HW scope: the big-block is HOT.

### §18.2 The COMPLETE stock CFG16 OFDM-DATA TX chain (every stage to the DAC), and what the block bypassed

A regular CFG16 DATA frame: `transmit_byte` (OFDM, `NO_FILTER_MESSAGE`) → `send_batch` packs the
per-frame raw passband and band-limits the WHOLE batch → `tx_transfer`. Stages, in order:

| # | stage | stock site | block (`bigblock_tx_passband`, telecom_system.cc:7030) |
|---|---|---|---|
| a | `pre_equalization_channel[j]` per-subcarrier (preamble + data) | transmit_byte:805,813 | YES :7072-7078 / :7093-7096 (uncommitted change) |
| - | `symbol_mod` | :831,838 | YES |
| b+d | level-cal norm `/power_normalization * sqrt(output_power_Watt)*[preamble_boost]*get_tx_gain(TX_SIG_OFDM)` | :859-866 (mfsk_boost=get_tx_gain(TX_SIG_OFDM) for OFDM, :850) | YES :7100-7110 (uncommitted change, `*ofdm_tx_gain`) |
| - | `baseband_to_passband` | :880-881 | YES :7113-7116 |
| - | `peak_clip` (preamble + data) | :883-884 | YES :7117-7118 |
| **c** | **`FIR_tx1` → `FIR_tx2` band-limit** | **send_batch arq_common.cc:4591-4592** (edge-pad :4577-4585, extract per-frame :4715) | **WAS MISSING** — §18.3 fix adds it |

`tx_transfer` (audioio.c:1658) does NO normalization — it writes doubles to the playback ring. There
is no separate output-power/RMS/DAC stage after FIR. So stage (d) the task referenced IS the
`sqrt(output_power_Watt)*get_tx_gain(TX_SIG_OFDM)` normalization, and it was ALREADY applied by the
uncommitted change. **The ONLY remaining stock-chain stage the block bypassed is the FIR band-limit (c).**

### §18.3 ROOT CAUSE of the overdrive — pre-eq and FIR are a MATCHED PAIR; the block applied pre-eq but not its canceling FIR

`get_pre_equalization_channel()` (telecom_system.cc:9475) computes pre-eq by passing a known symbol
through the EXACT cascade `baseband_to_passband → FIR_tx1 → FIR_tx2 → passband_to_baseband(FIR_rx) →
symbol_demod` and setting `pre_eq[j] = modulated[j]/demodulated[j]` — i.e. **pre-eq is precisely the
inverse of the per-subcarrier FIR response** (it BOOSTS the band edges, measured `[PRE-EQ] min_mag
1.60 max_mag 4.78` at CFG16). In the stock path FIR then ATTENUATES those same edges back → net flat
at the calibrated level. The block applied pre-eq (edge boost) but skipped the FIR (edge cut), so the
edges stayed boosted → HOT.

`--test-bigblock-txlevel` (in-process, device-free; same `transmit_byte` entry) quantifies it
(bb-vs-stock+FIR, the production HW reference; STOCK+FIR data peak 0.492 rms 0.139):

| block conditioning | RMS ratio (data) | peak ratio | byte gate |
|---|---|---|---|
| pre-eq + level-cal, **no FIR** (the +pre-eq baseline) | **1.31 (+2.33 dB)** HOT | 1.17 (+1.38 dB) | 8/8 PASS |
| level-cal only (no pre-eq, no FIR) | 0.52 (−5.74 dB) QUIET | 0.46 | 8/8 (=464e1fa) |
| **pre-eq + level-cal + FIR** (§18 fix, opt-in) | **0.96 (−0.35 dB)** / 1.04 whole | 1.07 (+0.62 dB) | **7/8 FAIL** |

The +2.33 dB matches the HW scope (~+3.2 dB). Level parity REQUIRES the full pre-eq+FIR pair — no
shortcut (no-pre-eq is −5.7 dB quiet; pre-eq-no-FIR is +2.3 dB hot). The residual +0.62 dB peak with
FIR is a PAPR effect (one long ~79k-sample waveform's extreme peak > a 16k-sample frame's at equal
power); RMS (the level metric) is ~1.0.

### §18.4 The FIX (arq_common.cc `bigblock_send_one_block`, before `tx_transfer`) — OPT-IN, BLOCKED on RX margin

Band-limit the block through the SAME `FIR_tx1→FIR_tx2` cascade, mirroring `send_batch` EXACTLY:
edge-replicate a `frame_output_size` lead+trail pad (absorbs the ~96-tap cascade group-delay
transient), filter, extract the `[pad, pad+n_tx)` real region, `tx_transfer` that. Architecturally
faithful: the stock FIR lives at the batch-assembler layer (`send_batch`), NOT in `transmit_byte`,
so the block's FIR belongs in `bigblock_send_one_block`, NOT in `bigblock_tx_passband`.

**BLOCKER (CLAUDE.md §2):** the FIR costs ~0.5-1 dB of decode margin the big-block's BESPOKE RX
(`bigblock_rx_passband`, sparse-pilot estimate + CSI-weighted LLR) cannot absorb on a 32-QAM CFG16
block: exactly ONE MIDDLE codeword (`cw2`, CARVE-CRC `calc=0f wire=e8` → miscorrection, NOT an
edge/window/timing artifact) fails on the PERFECT channel; 8/8→7/8; the partial-block path then SACKs
the gap and never first-block-delivers (gate hang). Decisive comparison: the pre-eq-no-FIR block
"passes" 8/8 only because its UN-cut pre-eq edge boost (~+4.8×) over-powers the edge subcarriers — it
passes BY running +2.3 dB hot, not by real margin (its channel is MORE selective, std|H| 0.046 vs the
FIR block's 0.031, yet it decodes — selectivity is NOT the discriminator; edge-subcarrier SNR is). At
the calibrated level the stock `receive_byte` RX survives the same FIR (broadband preamble-LS estimate
recovers edge SNR) but the big-block RX is ~1 cw short.

**Two RX recovery attempts FAILED** (so per §2 the RX margin work is deferred to its own audited fix +
regression, NOT shotgunned here): (1) force sparse-2D estimator (`MERCURY_BIGBLOCK_SPARSE2D=1`) → still
7/8; (2) broadband-preamble per-subcarrier pre-correction (divide data by `H_pre[j]` measured off the
block preamble, mirroring receive_byte) → made |H| MORE selective (the block preamble is not a flat
broadband pilot — it carries its own per-subcarrier structure + preamble_boost, so dividing injected
that structure), 7/8. Reverted.

**Shipped state:** FIR is OPT-IN `MERCURY_BIGBLOCK_FIR=1` (default OFF). Default path = pre-eq +
level-cal block (validated 8/8 multicw + fullpath) — still +2.3 dB hot, the residual this fix could
not safely close. `--test-bigblock-txlevel` applies the FIR unconditionally so the achievable AFTER
ratio (~1.0) is on the record (`MERCURY_BIGBLOCK_NOFIR=1` to measure the default wire).

### §18.5 The real fix path (cost) — big-block RX edge-SNR recovery

The block must transmit at the calibrated level (pre-eq+FIR) AND its RX must recover edge-subcarrier
SNR the way stock `receive_byte` does. Candidate: a TRUE broadband channel estimate from the block
preamble using the SAME known-preamble convention + LS window receive_byte uses (the naive Y/X
pre-correction in §18.4 attempt 2 was wrong because it ignored the preamble's own modulation/boost
profile). Needs a §5 cross-layer audit against the §7.4 grid freeze + §15-17 window invariants and a
dedicated fail-before/pass-after regression at the calibrated level. Until then, big-block on HW must
either run hot (decodes via the over-boost) or stay opt-in. NO monitor merge, NO push, NO attribution.

## §19. ACQUISITION-WINDOW POSITION — the §17 fix made the snapshot WAIT for a block-span of fresh symbols (the COUNT) but NOT WAIT for the block to land EARLY enough in the window (the POSITION); a block whose preamble lands late has its tail still IN THE FUTURE at snapshot time → zero-padded tail → block-wide estimate collapses → cw0 wire-CRC fails. (2026-06-07, branch `fix/bigblock-chanest`, worktree `C:/Users/kamer/mercury_wt/bb-chanest`)

### §19.1 The defect (HW, `bigblock_p3_hw/ACQ_GATE_ANALYSIS.json`)

On the percw HW bench the big-block BBTX-GATE passed only ~5.6% (1/18 attempts), yet EVERY pass
decoded 8/8 byte-faithful (`[BIGBLOCK-RX] carve K=8 cw_ok_count=8`, rsp.log:2932). LEVEL ruled out
(the −2.66 dB flat-gain cut lands the block at the per-frame FIR'd RMS; an OLD full-level binary ALSO
failed). GATE ruled out (8/8 on every accept; the cw0 wire-CRC only rejects genuinely truncated/silent
blocks). The DECISIVE TRIPLET (all near-perfect Schmidl-Cox locks, metric 0.998–0.999): the ONLY pass
had `preamble_symbol=0` (delay 0); the two fails had `preamble_symbol=116` and `=128` of the 133-symbol
window — i.e. ~48–56 of the block's 64 symbols extend PAST the window end. So a clean timing lock is
NECESSARY but NOT SUFFICIENT: the block POSITION within the captured window decides pass/fail, and the
block lands at a RANDOM phase each attempt (observed preamble symbols 0,6,7,15,27,30,51,54,71,75,76,
100,116,128).

### §19.2 ROOT CAUSE — the snapshot fires before the late-landing block's tail has been captured (the ring physically holds only `buffer_Nsymb` symbols of history)

The §17 fix arms `frames_to_read = bigblock_rx_block_nsymb()+10` at every data-path re-arm, so a
block-span of FRESH symbols accumulates before the snapshot. But the snapshot fires when `frames_to_read`
counts to 0 (`audioio.c:1394` HW; `arq_commander.cc:10120` sim), at WHATEVER `ring_write_index` the
write head sits — and the ARMING POINT (when `frames_to_read` was set) has NO phase relationship to when
the CMD's over-the-air block actually starts. So the block arrives somewhere inside the countdown window.
When the block starts LATE in the countdown, its tail is STILL ARRIVING (future samples) at the instant
`frames_to_read` hits 0 — those tail samples are NOT YET in the ring.

The ring `passband_delayed_data` is a DOUBLE-MAPPED buffer of length `2*sp` where the writer mirrors
`[wi]` into `[wi+sp]` every symbol (`audioio.c:1366-1392`, comment: "Reading sp samples from any
position in [0,sp) gives a contiguous chronological view via the mirror"). It holds EXACTLY `sp =
Nofdm*buffer_Nsymb*interp` samples of history. `bigblock_rx_passband` runs Schmidl-Cox over that `sp`
window to FIND the preamble (`head_delay`, telecom_system.cc:7431-7477) then `bb_at` reads the 64-symbol
block FORWARD from `data_start = head_delay + pre_nSymb*sym_samples` (telecom_system.cc:7506-7539),
ZERO-PADDING any read past `rxpb.size()` (`src<rxpb.size()? : 0.0`, :7522). When `head_delay` lands later
than `(buffer_Nsymb − block_nsymb)` symbols into the window, the forward read runs off the captured
samples → zero-padded tail → the block-wide sparse-2D pilot estimate integrates over a half-silent grid
and collapses → cw0's CRC fails even on a perfect lock.

WHY "ENLARGE THE READ" CANNOT WORK (refutes the ACQ_GATE_ANALYSIS "preferred" enlarge-buffer-read
option as literally stated): reading MORE than `sp` samples from the ring re-reads the `[rwi+sp]` MIRROR
of the OLDEST data, NOT future samples — the tail physically is not in the buffer yet. The only ways to
get the full block in-window are (a) grow the ring `buffer_Nsymb` ≥ block_span+slack AND wait for the
tail to arrive, or (b) DEFER the snapshot one arming cycle so the tail arrives and the block re-lands
earlier in the (unchanged) window. (b) is root-cause, RX-confined, TX-byte-identical, and reuses the
§17 `bigblock_block_ftr_or()` mechanism; (a) perturbs the shared `signal_period`/allocation. We take (b).

### §19.3 The fix — a window-position guard at the bigblock RX decode site (Option A, defer-and-re-arm)

In `receive()` (arq_common.cc) on the bigblock-RX branch, AFTER `receive_byte`→`receive_bigblock` has
located the preamble but BEFORE the carve, check whether the located block fit in the captured window.
`receive_bigblock` exposes the located head position as `bigblock_last_rx_head_delay_samples` (NEW; set
in `bigblock_rx_passband` from `head_delay`) and the captured length `bigblock_last_rx_capture_nsamples`
(NEW; = the `nSamples` it decoded). The guard computes
`block_end = head_delay + (pre_nSymb + Ngrid)*sym_samples` and, when
`block_end > capture_nsamples` (the tail was zero-padded), DEFERS: it does NOT carve, re-arms
`frames_to_read` to `bigblock_rx_block_nsymb()+10` (so a fresh full block-span accumulates and the tail
arrives), and does NOT wipe the ring (so the just-arrived head is not discarded). A bounded
defer-counter (`bigblock_rx_defer_count`, reset on every accept) caps consecutive defers so a genuinely
absent block cannot spin; on cap-exceed it falls through to the stock cw0-CRC gate (which rejects the
truncated block as today — no regression, just no infinite defer). On the NEXT arming cycle the block
has fully arrived and re-lands earlier in the window → `block_end ≤ capture_nsamples` → carve 8/8.

Because each defer accumulates ≥ block_span fresh symbols, the re-snapshot is GUARANTEED to contain the
whole block (the block cannot be longer than block_span), so a single defer suffices in the common case
and the first-DELIVERED fraction approaches the per-frame acquisition rate (≫ the ~5.6% / ~52%
one-shot). The guard is gated STRICTLY on `bigblock_framing_enabled && M!=MOD_MFSK &&
current_configuration==CONFIG_16 && bigblock_last_rx_K>0`; off-rung it is never entered → byte-identical.
REPRODUCER HOOK: `MERCURY_BIGBLOCK_DEFEAT_ACQGUARD=1` bypasses the defer (carve the truncated block as
pre-fix) for the fail-before/pass-after A/B. Production never sets it.

### §19.4 S5 cross-layer audit — the new RX members + the snapshot-position decision

1. **Producers**
   - `bigblock_last_rx_head_delay_samples` (NEW): written ONCE per decode in `bigblock_rx_passband`
     from the located `head_delay` (telecom_system.cc, after the SC/MF-snap acquire), and reset to −1
     at the top of `bigblock_rx_passband` (acq-fail leaves it −1). Mirror of the existing
     `bigblock_last_rx_meanh` stash convention (telecom_system.cc:190 stash, §11 health metric).
   - `bigblock_last_rx_capture_nsamples` (NEW): written in `receive_bigblock` to the `nSamples` it
     decoded (telecom_system.cc:8918), so the ARQ guard knows the exact captured length without
     re-deriving `buffer_Nsymb` (which `bigblock_restore_stock_config` may have changed by carve time).
   - `bigblock_rx_defer_count` (NEW, ARQ controller): incremented on each defer, reset to 0 on accept.
   - `frames_to_read` at the defer re-arm: a NEW data-path producer at the bigblock rung — wrapped in
     the EXISTING `bigblock_block_ftr_or()` so it stays on the §17.5 producer list and cannot diverge.
2. **Consumers**
   - `bigblock_last_rx_head_delay_samples` + `bigblock_last_rx_capture_nsamples`: read ONLY by the new
     guard in `receive()` (arq_common.cc) to compute `block_end` and decide defer-vs-carve. No other
     reader; both are diagnostic-stash members like `bigblock_last_rx_meanh`.
   - `bigblock_rx_defer_count`: read only by the guard (cap check). Reset by the guard on accept and by
     `init_messages_buffers`/connection reset (default 0).
   - The carve consumers (`bigblock_receive_carve`, `messages_rx[]`, bsi/EOB, SACK) run UNCHANGED on
     the accept path; on a defer they are SKIPPED entirely (no partial state written).
3. **Valid states** before any producer writes: `head_delay=-1` (acq fail) and `capture_nsamples=0`
   (no decode yet). The guard treats `head_delay<0 || capture_nsamples<=0` as "not a locatable block" →
   does NOT defer (lets the existing cw0-CRC gate handle it as today). `defer_count` defaults 0.
4. **Invariant the carve consumer assumes** (NEW — INV-10): the carve runs ONLY when the FULL block was
   captured (`head_delay + block_span_samples ≤ capture_nsamples`). §17's INV-9b guaranteed enough
   fresh symbols accumulated (the COUNT); INV-10 adds that the located block also FITS the window (the
   POSITION). The guard establishes INV-10 before every carve; when violated it defers instead of
   carving a truncated block. Every other carve consumer (whiten, per-cw CRC, bsi, EOB, batch-size
   election §16) is unchanged and only ever sees a full-block carve.
5. **What the fix changes**: it adds a DEFER decision BEFORE the carve, keyed on two new RX-stash
   members. It does NOT touch the carve, the cw0-CRC gate, the channel estimate, nv, pilots, LDPC, the
   TX, or the per-frame path. Stock per-frame configs (0..15) and MFSK never enter the guard (rung
   gate false). The defer re-arm reuses `bigblock_block_ftr_or()` (§17.4), so the armed window stays on
   the audited producer list. The post-carve full-ring wipe (arq_common.cc:7423) runs ONLY on accept;
   on a defer the ring is left intact so the just-arrived head survives to the next snapshot.

### §19.5 §19.2 vs §17 — these are DISTINCT axes (both required)
- §17 (COUNT): arm `frames_to_read ≥ block_span` so a block-span of FRESH symbols accumulates before
  the snapshot. Without it the snapshot fired after one STOCK frame → only the block HEAD was fresh.
- §19 (POSITION): even with a block-span COUNT armed, the snapshot fires at a random write-head phase,
  so the block can land late and its tail be unborn at snapshot time. §19 defers until the block
  fully arrived AND fits the window. §17 is necessary (enough symbols) but not sufficient (right phase);
  §19 closes the phase axis. Both gated identically on the CFG16 big-block rung.

### §19.6 Regression — `--test-bigblock-chanest` driven at several in-window preamble offsets (fail-before / pass-after)
The existing chanest harness (`run_block`, test_bigblock_arq_unit.cc) custom-sizes `buffer_Nsymb` to
fit the whole window, so it can NEVER reproduce the position bug. NEW `--test-bigblock-acqwindow`
(`test_sim_inproc_bigblock_acqwindow`) drives ONE genuine K=8 block into a FIXED production-sized
133-symbol window at THREE preamble offsets (symbol 0, mid ~60, near-end ~120) and asserts:
- FAIL-BEFORE (`MERCURY_BIGBLOCK_DEFEAT_ACQGUARD=1`): the near-end offset carves a TRUNCATED block →
  cw0-CRC fail / `bytes_ok=0` (the position bug reproduced in-process).
- PASS-AFTER (guard ON): every offset whose block fits decodes 8/8; the late offset DEFERS
  (`[BBTX-ACQ-DEFER]`, no carve) and the re-presented complete block decodes 8/8 `bytes_ok=1`.
Plus CLEAN (offset 0) stays 8/8, and `--test-bigblock-multicw/-fullpath/-arq-unit` + `--test-climb-engine`
stay green, PER-FRAME byte-identical. NO monitor merge, NO push, NO attribution.

## §20. CFG16 CARVE-COOLDOWN — the climb/election re-election MEMORY (WALL-B FIX-5). On a CLEAN pg84 long stream the loop THRASHES: climb→CFG16 carve-park→3 carve-fails→FIX-4 demote→CFG15→the gearshift RE-CLIMBS to CFG16 (clean channel, SNR says go)→re-elects the carve-dead big-block rung→re-parks on the carve that NEVER fires→repeat; 0/897770 delivered. FIX-4 pins `supershift_proven_ceiling=CFG15` but `finish_turbo_direction()` UNCONDITIONALLY resets it to the probe top (CFG16) on every ROBUST→CFG16 re-climb (arq_commander.cc:4215), so that pin has a lifetime of ~one cycle. (2026-06-09, branch `fix/wallb-cascade`, worktree `C:/Users/kamer/mercury_wt/wallb-fix`. Design+audit: `bigblock_p3_hw/_wallb/fix5/FIX5_DESIGN.md`, `FIX5_AUDIT.md`.)

### §20.1 State family B — `bigblock_carve_cooldown_batches` / `bigblock_carve_cooldown_span` (NEW, CMD-only)
A SEPARATE CMD-side field the turbo state machine does NOT touch (so it survives the :4215 reset that defeats FIX-4), consulted as an additional INDEX-CAP (→CFG15) at every climb hook. AARF form: 1st demote arms `BB_CARVE_COOLDOWN_BASE`=24 batches; each RE-demote WHILE active DOUBLES, capped at `BB_CARVE_COOLDOWN_MAX`=384 (16×BASE, mirrors `BREAK_DROP_STEP_MAX`'s 16× span idiom). Unit = completed BATCHES not wall-time (the thrash cadence is dominated by variable deaf-peer BREAK/watchdog storms — minutes/cycle; a batch count self-scales with link speed).

1. **Producers**:
   - ARM/extend: `arq_commander.cc:~3691` — the FIX-4 carve-viability deadline block, immediately after the existing `supershift_proven_ceiling = carve_fallback` pin. `span = bigblock_carve_cooldown_next_span((batches>0)?span:0, BASE, MAX); batches = span;` (re-arm while active = GROW; post-expiry = BASE).
   - DECREMENT (expiry): `arq_commander.cc:~5466` — the `gear_shift_algorithm==SUCCESS_BASED_LADDER` branch of `finalize_block_commander()`, which runs exactly ONCE PER COMPLETED BATCH (both the SACK-v2 policy path and the v1 inline path flow through here), the same per-batch cadence as `gear_shift_blocked_for_nBlocks`/`ceiling_success_count`. NOT per `receive()` tick (audit R6).
   - CLEAR-on-success: `arq_commander.cc:~3851` — the data-ACK reset site (where `emergency_nack_count=0`), GATED on `current_configuration==CONFIG_16 && telecom_system->bigblock_framing_enabled && M!=MOD_MFSK` (a real CFG16 carve landed). Clears `_batches`+`_span` to 0 (re-enables CFG16) and resets the AARF.
   - INIT: member-initializer `{0}` (arq.h) + zeroed in the ctor/init block (`arq_common.cc:~772`) and in `reset_session_state()` (`arq_common.cc:~3803`), mirroring `supershift_proven_ceiling=-1` (audit R3 — a fresh session never inherits a stale cooldown).
2. **Consumers** (all via the pure member `apply_bigblock_cooldown_cap(proposed)`, an index-monotone never-raise clamp to `bigblock_carve_cooldown_ceiling()`=CFG15 when armed, else identity):
   - Gearshift LADDER-UP `ceiling_blocked` — BOTH twins: v1 inline `arq_commander.cc:~5541`, v2 policy `~5704`.
   - Gearshift CEILING-RECOVERY raise — BOTH twins (suppress raising the proven ceiling above CFG15): v1 `~5563`, v2 `~5727`.
   - Turbo SNR-SUPERSHIFT / step-1 probe target — `arq_commander.cc:~5106` (the LOAD-BEARING hook: this survives the ROBUST collapse and re-climbs).
   - `elevator_target_from_snr()` return — `arq_commander.cc:191` (shared by the SNR re-trigger :5180 and the FRAME-UP elevator, so capping here covers both).
   - `finish_turbo_direction()` `start_config` — `arq_commander.cc:~4210` (NON-NEGOTIABLE: the exact site that resets `supershift_proven_ceiling` and defeats FIX-4).
   - Q-table optimizer `opt_pending_switch_cfg` dispatch — `arq_commander.cc:~599` (audit R7: a FOURTH CFG16 producer that bypasses the gearshift/turbo gates; capped here too).
3. **Valid states / default-init**: `_batches==0` (default) ⇒ no cap, every climb path no-ops the cooldown — the normal byte-identical state for non-bigblock and clean-CFG16 runs. `_batches>0` ⇒ CFG15 cap active. Boundary: `_batches` reaches 0 by decrement ⇒ re-probe re-enabled, `_span` retained for AARF until a carve-success (reset 0) or a fresh post-expiry demote (starts a new window at BASE).
4. **Invariants**: **INV-B1** `_batches>0` ⇒ no climb selects index>index(CFG15) — upheld by applying the cap at ALL the producers of `current_config=CONFIG_16` (the 6 enumerated hooks; audit §5.5 proved completeness). **INV-B2** CMD-only, never on the wire, RSP never reads it. **INV-B3** (the single most load-bearing gate, audit R2) a per-frame CFG15 data-ACK must NOT clear the cooldown — only a CFG16 carve does — upheld by the `current_config==CONFIG_16 && framing` clear gate.
5. **What FIX-5 changes**: it ANDs an INDEPENDENT never-raise cap into the SAME consumer sites as `supershift_proven_ceiling`; the effective ceiling becomes `min(supershift_proven_ceiling-cap, cooldown-cap, WB/NB ceiling, max_config_override)`, all index-monotone clamps ⇒ order-independent, can only LOWER ⇒ no new over-climb. On a non-bigblock channel or a clean CFG16 with a working carve the cooldown is never armed, so behavior is byte-identical (the `--test-bigblock-fullpath` live transfer holds CFG16 and delivers 1200/1200 — the cooldown never arms).

### §20.2 Regression — `--test-climb-engine` Part T (pure helpers) + `--test-bigblock-climb-election` FIX-5 section (real CMD/RSP pair)
- Part T (arq_commander.cc test_climb_engine, mirrors Parts R/S): T0 arms BASE; T1 AARF doubling capped at MAX (24→48→…→384, no overflow); T2 ceiling=CFG15 armed / −1 off; T3 `apply_bigblock_cooldown_cap` refuses CFG16→CFG15 armed, identity off; T4 FIX-4 target == FIX-5 ceiling; T5 INV-B3 clear-gate (CFG16-carve clears, CFG15-ACK / framing-off do NOT); T6 post-expiry fresh fade resets span to BASE; T7 the cooldown SURVIVES the `supershift_proven_ceiling=CFG16` reset and STILL caps at CFG15 (the limit-cycle break).
- `--test-bigblock-climb-election` adds an 8-check FIX-5 section on a REAL pair (load_configuration / framing real): re-election REFUSED while armed, per-frame CFG15 delivery rung selected, turbo-reset survival, INV-B3, clear-on-CFG16-carve, exponential growth capped.
- FAIL-BEFORE via `-DWALLB_FIX5_FAILBEFORE` (the helpers compile to 0/−1 = no cooldown): Part T fails 9 checks, the election section fails 5; PASS-AFTER all green. `--test-bigblock-carve-suspend-unit` (FIX-3), `--test-bigblock-arq-unit`, `--test-bigblock-fullpath` (carve-success unaffected), `--test-probe-backoff`, `--test-data-anchored-promote` all stay green. CMD-only, no RSP/wire/DSP change. NO monitor merge, NO push, NO attribution.

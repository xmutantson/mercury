# Big-Block Integrity — block-level CRC anchor over the K-codeword carve

**Owner structure:** the assembled big-block payload (`block_payload` / `block_payload_bytes`,
`K*sub_len` bytes) and its on-wire integrity fields (per-cw CRC-8 + whole-block CRC-32).
**Created:** 2026-06-08 from the P3 HW NO-GO (SILENT WRONG-BYTE) + the D2_BLOCKCRC review.
**Worktree:** `C:/Users/kamer/mercury_wt/bb-d3`, branch `fix/bigblock-d3-carve`.
V1 (§1-§5) line numbers verified against HEAD **`10f7ad5`** (the D2_BLOCKCRC commit on top of
`f9f35f1`=D2+D3). V2 (§9-§11) is the follow-on commit closing Findings 1+2; its line numbers
are verified against the post-V2 source.

---

## §1. The bug this anchors against (the HW NO-GO)

`bigblock_p3_hw/WINRUN_FINAL_VERDICT.json`: a D3 carve declared all K=8 per-cw CRC-8 clean
(`n_clean=8/all_K`) and delivered a full **1374-byte** block via `copy_data_to_buffer`, but
`recv_md5 != sent_md5` (771539…d2 vs 05f835…ea). The 8-bit per-cw gate **false-passed** a
self-consistent LDPC-miscorrected block. Joint residual is NOT 2^-64 (the 8 false-passes are
correlated, not independent), so the per-cw layer alone cannot bound the block. Verdict:
SILENT_WRONG_BYTE, NO-GO, shipping blocker for a life-critical modem.

## §2. The fix (D2_BLOCKCRC, commit 10f7ad5)

A whole-block **CRC-32** (reflected IEEE 802.3, poly `0xEDB88320`, init/final `0xFFFFFFFF`)
stacked ON TOP of the per-cw CRC-8s, in cw(K-1)'s trailer.

- **Geometry** (datalink_defines.h:223,228-229): `BIGBLOCK_BLOCK_CRC_BYTES=4`;
  `BIGBLOCK_BLOCK_CRC_OFFSET(K,sub_len) = K*sub_len - BIGBLOCK_CW_CRC_BYTES - 4`. For
  K=8/sub_len=175 → field=`[1395..1398]`; cw7 per-cw CRC-8 at `1399`.
- **TX stamp** (arq_common.cc): `CRC32_calc` over the full `K*sub_len` image, written
  BEFORE the per-cw CRC-8 loop (both zeroed sets still 0). Helper `CRC32_calc` at
  arq_common.cc:9494-9510, declared arq.h. Algorithm verified: `CRC32("123456789")=0xCBF43926`.
- **RX verify** (arq_common.cc): fires ONLY when `info_bits!=NULL && !defeat && n_clean_precheck==K`.
  Zeroes the SAME bytes (own 4 + all K per-cw CRC-8 tails), recomputes, on mismatch clears ALL
  `cw_ok` → `n_clean=0` → PARTIAL. Runs AFTER per-cw demote, BEFORE `bigblock_block_to_arq`.
- **Decoupling:** the CRC-32 treats BOTH its own 4 field bytes AND all K per-cw CRC-8 tails as
  zero, so the two CRC layers never feed each other — TX and RX compute an identical image.
- **Reproducer hooks:** `MERCURY_BIGBLOCK_DEFEAT_BLOCKCRC=1` disables the reject;
  `MERCURY_BIGBLOCK_FALSEPASS_CW=N` injects a re-stamped corrupt codeword. Whitening seed
  `0x5A3C96E1` (self-inverse XOR).

## §3. VERIFIED SOUND (review synthesis 2026-06-08)

1. **CRC-32 correct + covers the full payload.** Residual ≈ 2^-32 (2.3e-10)/block, HD≥4 to
   91607 bits ≫ the ~11200-bit block. Far below the per-cw CRC-8 2^-8 (3.9e-3) floor.
2. **No full-clean (n_clean==K) bypass.** The block-CRC verify is the SOLE gate before the only
   clean-delivery `copy_data_to_buffer`. A mismatch forces n_clean=0/PARTIAL/never-deliver.
3. **Per-cw CRC-8 UNWEAKENED** (CLAUDE.md §2): block-CRC is purely additive.
4. **De-whiten cancels** (whitening seed `0x5A3C96E1`, self-inverse XOR). Both CRCs on plaintext.
5. **Per-frame + D2 paths intact.** Block-CRC gated to CONFIG_16 + `bigblock_framing_enabled`.

## §4. FINDING 1 (V1) — SHIPPING BLOCKER: cap overlap → deterministic false-reject LIVELOCK

The 4-byte block-CRC field in cw(K-1)'s trailer is **NOT reserved** in the production app
capacity in V1 — only the test builder reserves it. Max full K=8 = `cw0_cap(156)+7*cwc_cap(174)
= 1374` — EXACTLY the HW NO-GO's `bytes_delivered=1374`. A genuinely-clean 1374B block has
frame[7]=174, which overlaps the block-CRC field at cw7-local `[170..173]` → TX writes app bytes
there, then overwrites with the CRC → RX zeroes & recomputes → MISMATCH → reject → re-emit → same
length → **PERMANENT LIVELOCK**. **CLOSED in V2 §9 (FIX-1).**

## §5. FINDING 2 (V1) — CONCERN: PARTIAL-path bypass (CRC-8-false-passed KEPT codeword)

Block-CRC fires only when `n_clean_precheck==K`. When per-cw CRC-8 demotes a GENUINE gap
(`n_clean<K`), the block-CRC is SKIPPED. A KEPT codeword that CRC-8 false-passes (wrong bytes) is
carved RECEIVED, transferred to `messages_rx_prev[]` (`bump_bsi_and_transfer_prev`), and on gap
recovery delivered via `copy_data_to_buffer` at **arq_responder.cc:767** with NO block-CRC
recompute. Residual ~2^-8 (3.9e-3) per false-passed codeword. **CLOSED in V2 §10 (FIX-2).**

---

# V2 — closing Findings 1 + 2 (commit on `fix/bigblock-d3-carve`)

## §9. FIX-1 — reserve the block-CRC field in cw(K-1) capacity (LIVELOCK)

The block-CRC field MUST be reserved out of cw(K-1)'s app capacity on BOTH TX and RX so a
genuinely-clean full block never writes app bytes into the field. The cap reservation now lives
in ONE place at TX — the new `bigblock_pack_block()` helper — exercised by BOTH production
(`bigblock_send_one_block`) AND the new MAX-PAYLOAD test arm, so the test and production cannot
drift (the V1 blocker existed precisely because the test builder reserved `cap-4` while production
did not).

- **TX (`bigblock_pack_block`, arq_common.cc):** the per-codeword app cap is
  `cap = (i==0) ? cw0_cap : (i==K-1 ? cwc_cap - BIGBLOCK_BLOCK_CRC_BYTES : cwc_cap)`. cw(K-1) app
  capacity drops 174→170; the block-CRC field at cw-local `[170..173]` is now OUTSIDE the app
  region, so the field write at the trailer never collides with an app byte and the TX CRC-32
  image (app bytes never in `[170..173]`) matches the RX recompute.
- **RX (`bigblock_receive_carve`, arq_common.cc):** the delivered-length cap for the last
  codeword mirrors TX: `cap = (c==K-1) ? (sub_len - BIGBLOCK_CW_CRC_BYTES - BIGBLOCK_BLOCK_CRC_BYTES)
  : (sub_len - BIGBLOCK_CW_CRC_BYTES)` (cw0 also `- hdr_total`). So a wire length-table value that
  (pre-fix) would have delivered into the field is clamped to 170.
- **New max full K=8 = cw0_cap(156)+6*cwc_cap(174)+cwK-1_cap(170) = 1370 bytes** (was 1374). The
  4 bytes are spent on the block-CRC.
- **TEST (MAX-PAYLOAD arm):** a clean full K=8 block with frame[K-1]=`cwc_cap` (the boundary the
  V1 builder's `cap-4` deliberately avoided) packed via `bigblock_pack_block` then carved.
  Fail-before (V1 cap, env `MERCURY_BIGBLOCK_DEFEAT_CAPFIX=1`): the field overlaps app →
  block-CRC mismatch → carve routes PARTIAL/livelock (NOT byte-faithful delivery). Pass-after
  (the reservation): byte-faithful delivery of all reserved bytes.

## §10. FIX-2 — block-CRC over the SACK-completed assembled block (silent-wrong-byte on PARTIAL)

Route EVERY assembled-block delivery through a block-CRC gate. The full-clean path is already
gated in `bigblock_receive_carve` (§2). FIX-2 adds the SAME guarantee to the PARTIAL/prev-batch
completion delivery (arq_responder.cc:738, the `copy_data_to_buffer` at :767).

**Why the gate must be at completion, not the carve:** `bump_bsi_and_transfer_prev`
(arq_common.cc:5760-5790) transfers only the per-frame **app bytes** of each kept codeword into
`messages_rx_prev[]`; the codeword-aligned wire image (per-cw CRC tails + the block-CRC field) is
discarded. The recovered gap codeword (a stock CFG16 per-frame retx) also lands as app bytes
(arq_responder.cc:706-709). So at completion the prev slots hold the full block's per-codeword app
bytes — exactly enough to **reassemble the `K*sub_len` codeword-aligned image** the TX computed the
block-CRC-32 over (app bytes at each codeword base, zero pad, per-cw CRC tails = 0, block-CRC field
= 0; cw0 header reconstructed from the stashed `[bsi, n_data, length-table]`).

**Mechanism (big-block-scoped):**
- **Stash at PARTIAL carve** (`bigblock_receive_carve`): when a big-block routes PARTIAL AND
  cw(K-1) decoded clean (`cw_ok[K-1]` true after the per-cw demote — so its trailer, which holds
  the block-CRC field, is CRC-8-validated), record into `bigblock_partial_*`: `K`, `sub_len`,
  `hdr_total`, `block_bsi`, the per-codeword wire app-length table, `cw0_offset`, and the
  expected wire block-CRC-32 value read from the decoded payload's cw(K-1) trailer. `armed=true`.
  When cw(K-1) is itself the gap, the field is untrustworthy and is NEVER recovered (the app-only
  retx does not carry `[170..173]`), so the gate is NOT armed for that rare sub-case — that one
  codeword stays at the unchanged per-cw CRC-8 floor; NO false-reject is ever introduced.
- **Gate at completion** (arq_responder.cc, just before the prev-batch `copy_data_to_buffer`):
  if `bigblock_partial_armed && rsp_prev_batch_seq_id == bigblock_partial_block_bsi`, reassemble
  the `K*sub_len` image from `messages_rx_prev[0..K-1]` and recompute CRC-32. On **mismatch**: do
  NOT deliver — clear the prev-batch (free slots, `rsp_prev_batch_active=false`, reset counts) so
  the CMD's ACK-timeout re-emits the whole block (fresh decode). On **match**: deliver as today.
  `armed` is one-shot (consumed at completion or on the next PARTIAL carve).
- **Scope:** the gate is keyed to a stashed big-block bsi, so a non-big-block prev-batch
  completion (`armed=false` or bsi mismatch) is byte-identical to before — regular per-frame
  delivery is UNCHANGED (CLAUDE.md big-block-scope honored).
- **TEST (FALSEPASS-on-PARTIAL arm, multicw ARM-G):** a PARTIAL block (one GENUINE gap forces
  n_clean<K) where ALSO one KEPT codeword is a CRC-8-passing corruption
  (`MERCURY_BIGBLOCK_FALSEPASS_CW`), driven through the live ACK-GATE + prev-batch completion.
  Fail-before (`MERCURY_BIGBLOCK_DEFEAT_PARTIALCRC=1`): the completed block delivers wrong bytes
  (the §5 residual). Pass-after: the completion block-CRC rejects → the block is NOT delivered.

## §11. §1.5 CROSS-LAYER DATA-FLOW AUDIT — assembled-block-delivery state (both gates)

New shared state (V2): `bigblock_partial_{armed,block_bsi,K,sub_len,hdr_total,cw0_offset,
expected_block_crc32, lengths[]}` — the block-integrity context stashed at a big-block PARTIAL
carve so the prev-batch completion can re-verify the whole-block CRC-32 over the reassembled image.

1. **PRODUCERS (writers):**
   - The TX wire block-CRC field + per-cw CRC tails: `bigblock_pack_block` (arq_common.cc), called
     by `bigblock_send_one_block` (production) and the MAX-PAYLOAD test arm. Reserves the field in
     cw(K-1) (§9).
   - The V2 stash: `bigblock_receive_carve` (arq_common.cc) writes `bigblock_partial_*` ONLY on a
     big-block PARTIAL carve with cw(K-1) clean (§10). Reset (`armed=false`) at the start of every
     carve and on prev-batch completion (consumed).
2. **CONSUMERS (readers):**
   - Full-clean gate: `bigblock_receive_carve` block-CRC verify (§2) — fires when n_clean==K,
     clears all cw_ok on mismatch → routes PARTIAL. UNCHANGED by V2.
   - PARTIAL/SACK-completed gate: the new check in arq_responder.cc at the prev-batch completion
     reassembles from `messages_rx_prev[]` and compares to `bigblock_partial_expected_block_crc32`.
   - `bigblock_block_to_arq` (test_bigblock_arq_unit.cc) — the carve consumer of cw_ok; unchanged.
     It still routes n_clean<K to the PARTIAL/ACK-GATE branch (FIX-2 only adds a gate AFTER
     completion, not a new carve path).
3. **VALID STATES:** before any producer writes, `bigblock_partial_armed=false` (default-init) →
   the completion gate is a no-op (regular per-frame + clean big-block delivery untouched). A total
   acq miss (info_bits==NULL) never stashes. A degenerate sub_len that cannot hold the field →
   bounds guard skips both the field reservation and the stash. cw(K-1)-gap PARTIAL → not armed.
4. **INVARIANTS the consumers assume:**
   (a) TX and RX zero the SAME bytes (block-CRC field + per-cw CRC tails) → identical image, no
       self-reference (held by §2 + §9's reservation keeping app bytes out of the field).
   (b) The reassembled completion image equals the TX CRC-32 input iff every kept slot's app bytes
       are byte-correct — so a false-passed kept codeword (wrong bytes) ⇒ image differs ⇒ CRC-32
       differs ⇒ reject. Verified: cw0 header reconstructed from stashed `[bsi,n_data,lengths]`;
       app bytes from prev slots at each codeword base; pad/CRC-tails/field zeroed.
   (c) The expected block-CRC value is trustworthy iff cw(K-1) was CRC-8-clean at carve (the field
       lives in cw(K-1) and CRC-8 span covers it) — so the gate is armed ONLY then → no false
       reject of a genuinely-clean completion.
   (d) On mismatch the consumer re-requests (clears prev, lets the CMD re-emit) — no new delivery
       path, no optimizer/gearshift state touched (the carve never touched it; the responder
       completion path likewise leaves config state alone).
5. **WHAT THE FIX CHANGES:** FIX-1 alters the cw(K-1) app cap on both sides (assumption (a) is now
   MAINTAINED instead of violated). FIX-2 adds a reject decision BEFORE the prev-batch
   `copy_data_to_buffer`; the only new consumer of `messages_rx_prev[]` is read-only (reassemble +
   CRC), and on mismatch it frees the prev slots — a state the existing stale-prev handling
   (`bump_bsi_and_transfer_prev`) already tolerates. No producer of `messages_rx_prev[]` changes.

## §12. Open questions [?]

- [?] cw(K-1)-gap PARTIAL completions stay at the per-cw CRC-8 floor for that one codeword (gate
  not armed). Closing it would require carrying the block-CRC field in cw(K-1)'s retx (app-only
  today). Deferred — the dominant false-pass surface (any interior/cw0 kept codeword with cw(K-1)
  clean) IS covered, and a cw(K-1)-gap completion would have to ALSO false-pass a different kept
  codeword (joint ~2^-8 × the cw(K-1)-gap fraction) to slip through.

## §14. V3 — close §10 (n_data<K clean-PARTIAL false-reject livelock) + class-complete matrix

Built ON TOP of V2 (`cd3d5ca`). The CRC-32 algorithm, cw(K-1) trailer placement, FIX-1 cap
reservation, and FIX-2 PARTIAL gate are UNCHANGED. ONE behavioral change (the §10 blocker the
BLOCKCRC_V2_VERDICT flagged NO-GO) + a class-complete test matrix.

### §14.1 The n_data fix (closes the BLOCKCRC_V2_VERDICT §10 blocker)

`bigblock_partial_block_crc_ok()` previously HARD-CODED the reassembled cw0 header byte
`img[1] = (K & 0xFF)`. The TX writes the REAL `n_data` (filled-codeword count) there
(`block_payload_bytes[1] = n_data`, arq_common.cc:3987), `< K` for an under-filled batch
(FIFO-drained / end-of-document tick — near-certain on a finite document's final CFG16 tick).
A genuinely-clean `n_data<K` PARTIAL block reassembled with `img[1]=K` ≠ TX n_data → CRC-32
MISMATCH → REJECT → re-emit byte-identical → re-REJECT FOREVER (deterministic false-reject
LIVELOCK, byte-safe / liveness-unsafe — the SAME prohibited shape §4/§9 killed, reintroduced by
FIX-2 on the under-filled block class).

**Fix:** stash the REAL decoded n_data at the FIX-2 arm site and use it at the gate.
- Carve parses `wire_n_data = payload[1]` (clamped [1,K]) when cw0 is clean — the FIX-2 arm
  precondition (arq_common.cc:4675).
- Arm site stashes `bigblock_partial_n_data = wire_n_data` (arq_common.cc:4765); reset to `-1` on
  every carve entry (arq_common.cc:4740).
- `bigblock_partial_block_crc_ok()` builds `img[1] = bigblock_partial_n_data` (arq_common.cc:4843).
- Reproducer hook `MERCURY_BIGBLOCK_DEFEAT_NDATA=1` restores the pre-V3 `img[1]=K` (the
  fail-before) on the SAME binary. Production never sets it.

### §14.2 §1.5 PRODUCER/CONSUMER AUDIT — `bigblock_partial_n_data` (new shared state, arq.h, init -1)

1. **PRODUCERS:** carve arm site (arq_common.cc:4765) `= (wire_n_data∈[1,K])?wire_n_data:K`;
   carve reset (arq_common.cc:4740) `= -1` every entry; member default `-1` (arq.h). Tests drive
   it only via the production carve.
2. **CONSUMERS:** `bigblock_partial_block_crc_ok()` (arq_common.cc:4831/:4843) — SOLE reader,
   `img_ndata = (n_data∈[1,K])?n_data:K; img[1]=img_ndata`.
3. **VALID STATES:** `-1` (unset — default-init OR a non-arming carve; the consumer is only reached
   when `bigblock_partial_armed`, set in the SAME block that writes a valid `[1,K]` n_data, so the
   consumer never sees `-1`; the `?:`→K fallback is defensive and matches pre-V3 for that
   impossible case). Armed: `[1,K]`.
4. **INVARIANTS:** (a) the stashed n_data equals the TX n_data the CRC-32 covered — both are the
   SAME wire cw0 header byte `payload[1]` (TX wrote, carve parsed from the clean cw0 decode). (b) the
   reassembled header (bsi,n_data,lengths) now matches the TX exactly, so the ONLY CRC-32 input
   difference is per-codeword app bytes → a kept-codeword false-pass still MISMATCHES (FIX-2
   preserved; n_data only removes a FALSE mismatch on a genuinely-clean block).
5. **WHAT CHANGES:** `img[1]` goes constant-`K` → decoded n_data. For n_data==K (the only case V2
   exercised) the value is IDENTICAL → byte-for-byte no change (verified: CASE F + matrix
   full-clean-K rows MATCH-on-clean / MISMATCH-on-corrupt with the SAME CRC values). For n_data<K it
   converts a guaranteed false-reject into a correct MATCH. Reset/arm lifecycle unchanged (one-shot).
   No optimizer/gearshift/per-frame state touched (big-block-scoped; partial-bsi-advance GREEN).

### §14.3 CLASS-COMPLETE MATRIX (`--test-sim-inproc-bigblock`, test_bigblock_arq_unit.cc)

One case per delivery class, each driving the PRODUCTION carve to arm the stash then exercising
`bigblock_partial_block_crc_ok()` over `messages_rx_prev[]` exactly as the responder completion
does. For each: NO livelock (clean always delivers) AND NO silent wrong-byte beyond the cw(K-1) floor.

| Class | n_data | gap | expected | result |
|-------|--------|-----|----------|--------|
| G-underfilled (V3 fix) | 5 | mid | armed; clean→deliver, corrupt→reject | PASS |
| full-clean-K | 8 | mid | armed; clean→deliver, corrupt→reject | PASS |
| cw0-gap | 8 | cw0 | NOT armed (header untrusted) → per-cw floor, no livelock | PASS |
| cwKm1-gap | 8 | cw(K-1) | NOT armed (CRC-bearer is the gap) → DOCUMENTED per-cw CRC-8 floor until cw(K-1) arrives; no new bypass, no livelock | PASS |
| mid-cw-gap | 8 | mid | armed; gates on completion | PASS |
| G-underfilled-gap | 4 | mid | armed; clean→deliver, corrupt→reject | PASS |

Plus an explicit **V3 n_data fail-before/pass-after** (n_data=3<K): `DEFEAT_NDATA=1` → the
genuinely-clean under-filled reassembly REJECTS (livelock first cycle); default → DELIVERS. PROVEN
on the same binary. `cwKm1-gap` is the accepted bounded §12 floor (NOT a new bypass): that one
codeword rides the per-cw CRC-8 ~2^-8 until cw(K-1) arrives clean, at which point the block is
re-carved full-clean and the §2 gate fires; the matrix asserts it does NOT livelock.

## §13. Cross-references

- `bigblock_p3_hw/WINRUN_FINAL_VERDICT.json` — the HW NO-GO this anchors against.
- `bigblock_p3_hw/_blockcrc/D2_BLOCKCRC_RESULT.json` — V1 fix design record (commit 10f7ad5).
- `bigblock_p3_hw/_blockcrc/BLOCKCRC_VERDICT.json` — the V1 review verdict (Findings 1+2 origin).
- `bigblock_p3_hw/_blockcrc/BLOCKCRC_V2_RESULT.json` — the V2 fix record (commit cd3d5ca).
- `bigblock_p3_hw/_blockcrc/BLOCKCRC_V2_VERDICT.json` — V2 synthesis (NO-GO; §10 n_data blocker).
- `bigblock_p3_hw/_blockcrc/BLOCKCRC_V3_RESULT.json` — V3 fix record (§10 n_data fix + class matrix).

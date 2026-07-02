# data-flow-correctness-consolidation.md — ONE cross-layer audit over the merged correctness fix set

Consolidated shared-state audit for branch `integ/correctness-consolidation`
(off `monitor 720d4bcc`, HEAD `aa1b57e5`). Built 2026-07-02 per CLAUDE.md
§Cross-Layer. **Purpose:** the fixes below were each fail-before/pass-after in
isolation; the DOMINANT residual risk (Fable consult Q2) is that they touch the
SAME shared state — mixbatch fill, TX/RX FIFOs, the ACK-confirm/backup path, the
reverse-SACK de-dup tracker — and therefore may collide with EACH OTHER, not with
`monitor`. This document answers the 5 audit questions ONCE over the COMBINED
change and walks every consumer of the shared state, flagging any invariant one
fix breaks that another assumes.

## §0 The fix set (8 commits / 7 logical fixes, 6 shared-state structures)

| # | commit | fix | layer | shared state written | env/compile defeat |
|---|--------|-----|-------|----------------------|--------------------|
| stall | `77a33fd3` (←af13507c) | partial-SACK de-dup re-keyed on `cmd_batch_seq_id` under the cap (post-leap stall) | ARQ reverse-ACK credit | `cmd_last_applied_sack_bsi` (de-dup tracker) | `-DCUMULATIVE_ACK_DEDUP_FAILBEFORE` |
| — | `22eeedd8` (←c215754c) | fact-doc §8 only (no code) | — | — | — |
| B1 | `c235ff4b` (←5b4ed575) | mixbatch fill over-pop, NO-COMPRESSION leg | CMD TX staging | `messages_tx[]`, `fifo_buffer_tx`, `fifo_buffer_backup` | `MERCURY_MIXBATCH_OVERPOP_DEFEAT` |
| B2 | `e2ba8831` (←787c840) | mixbatch fill over-pop + destructive force-FREE, COMPRESSION leg | CMD TX staging | `messages_tx[]`, `fifo_buffer_tx`, `batch_capacity` | `MERCURY_MIXBATCH_OVERPOP_DEFEAT` (shared) |
| D | `ce2335bf` (←ad26615) | stale-partial FALSE-ACK guard (`partial_sack_target_is_inflight`) | ARQ reverse-ACK validate | *reads* `cmd_batch_seq_id` + de-dup gate (no new state) | `-DCUMULATIVE_ACK_STALEPARTIAL_FAILBEFORE` |
| A | `57684aa1` | defer mid-flight `data_batch_size` SHRINK that would orphan RECEIVED prev frames | RSP delivery | `data_batch_size`, `rsp_deferred_batch_shrink` | `MERCURY_BATCHSHRINK_ORPHAN_DEFEAT` |
| C/H#1 | `c34259ba` | `fifo_buffer_backup` re-stage double-delivery: ACK-confirm-keyed flush | CMD backup lifecycle | `fifo_buffer_backup` | `MERCURY_BACKUP_CONFIRMFLUSH_DEFEAT` |
| H#3 | `aa1b57e5` | RX-FIFO back-pressure post-ACK loss: gate the clean ACK on app-FIFO room | RSP delivery ↔ app | *reads* `fifo_buffer_rx`; HOLDS `messages_rx[]` RECEIVED | `MERCURY_RXFIFO_BACKPRESSURE_DEFEAT` |

Endpoints matter: **stall, D, B1, B2, C are COMMANDER-side** (TX / reverse-ACK
consumer); **A, H#3 are RESPONDER-side** (RX delivery). This split is the first-order
reason most pairs cannot collide — they run in different roles on the same
`cl_arq_controller` and touch disjoint arrays. The intra-role pairs (stall∥D on
CMD; A∥H#3 on RSP; B1∥B2∥C on CMD-TX) are the ones this audit scrutinises.

---

## §1 PRODUCERS (every path that WRITES the shared state)

### S1 — `messages_tx[]` slot lifecycle (CMD TX), states FREE / ADDED_TO_LIST / ADDED_TO_BATCH_BUFFER / PENDING_ACK / ACK_TIMED_OUT / ACKED
- **B1 fill (no-comp)** `arq_commander.cc` `process_buffer_data_commander()` no-comp leg: pops ≤ `fill_limit` from `fifo_buffer_tx` → `add_message_tx_data()` sets `ADDED_TO_LIST`. B1 CAPS `fill_limit = data_batch_size − min(retransmit_count,data_batch_size) − already_staged` (was `data_batch_size` unconditional).
- **B2 fill (comp)** comp leg: sizes `crypto_frames = data_batch_size − min(retransmit_count,dbs)` (was `data_batch_size`); `batch_capacity = crypto_frames·max_frame`; splits `comp_buf` into `ceil(comp/max_frame)` `ADDED_TO_LIST` frames. B2 also adds a **force-FREE guard**: skips staging when any slot is live (`ADDED_TO_LIST`/`ADDED_TO_BATCH_BUFFER`) and makes the pre-split force-FREE loop `continue` past live slots.
- `add_message_tx_data()` (`arq_commander.cc:2051`) — first-FREE-slot writer (unchanged).
- `send_batch()` flips `ADDED_TO_LIST`→`ADDED_TO_BATCH_BUFFER`→`PENDING_ACK` (unchanged).
- **register_ack()** (`arq_commander.cc:80`) `PENDING_ACK`→`ACKED` (unchanged) — then calls **C** (see S4).
- Recovery/config-change frees (`clear_retx_queue`, `inband_unilateral_config_change`) (unchanged).

### S2 — `data_batch_size` (both roles; the ACK-bitmap width AND the delivery bound)
- `set_data_batch_size()` (`arq_common.cc:~1360-1436`) is the SOLE chokepoint. Robust branch clamps to `[1,ROBUST_DWELL_BATCH_MAX]`; OFDM branch clamps to the header bound.
- **A inserts a DEFER gate** at BOTH branches: `if(defer_shrink_if_would_orphan_prev(target)) return;` (`arq_common.cc:1400`, `:1430`) — writes `rsp_deferred_batch_shrink=target` and leaves `data_batch_size` UNCHANGED (held large) when the shrink would orphan RECEIVED prev frames.
- **A applies** the deferred target via `rsp_apply_deferred_batch_shrink()` (`arq_common.cc:1269`) which re-enters `set_data_batch_size(t)` once the prev clears.

### S3 — `rsp_deferred_batch_shrink` (RSP scalar, NEW state; −1 = none)
- Written by `defer_shrink_if_would_orphan_prev` (set target), `rsp_apply_deferred_batch_shrink` (clear to −1), init `arq_common.cc:738`, re-init `:5768`, test reset `arq_responder.cc:12454`.
- Apply sites (prev-clears): frame-driven prev-deliver `arq_responder.cc:1346`; prev-deliver completion `arq_common.cc:5167`.

### S4 — `fifo_buffer_backup` (CMD; raw source of the in-flight batch for config-change re-frame)
- **Producer (mirror)** B1/B2 fill push every popped frame (`arq_commander.cc:~21153`).
- **Flush (steady state, pre-existing)** `finalize_block_commander()` — SKIPPED when a control frame is queued or new-data is staged (the Root-B hole).
- **C ADDS a flush** `maybe_backup_confirm_flush()` (`arq_commander.cc:109`) called from `register_ack()` (`:90`): flushes IFF `role==COMMANDER` AND `retransmit_count==0` AND NO `messages_tx[]` slot is `PENDING_ACK`/`ACK_TIMED_OUT`/`ADDED_TO_LIST`/`ADDED_TO_BATCH_BUFFER`.
- **Config-change re-stage (consumer that also writes tx)** `inband_unilateral_config_change()` restores leftover backup → `fifo_buffer_tx` tail (`arq_common.cc:~3570`) (unchanged; the mixbatch-doc §7 Root-B re-stage lives here).

### S5 — `messages_rx[]` / `messages_rx_prev[]` (RSP delivery arrays) + `fifo_buffer_rx` (app FIFO)
- Forward decode writes `messages_rx[i]=RECEIVED` (unchanged).
- Batch-complete ACK-GATE marks RECEIVED→ACKED, bumps bsi, `copy_data_to_buffer()` drains `[0,data_batch_size)`→`fifo_push_rx()` (unchanged core).
- **H#3 inserts a HOLD gate** BEFORE that commit (`arq_responder.cc:2450-2477`): if `fifo_buffer_rx.get_free_size() < rx_fifo_batch_need()`, keep `messages_rx` RECEIVED (do NOT ACK/bump/deliver), re-arm receive timer, `return`.
- `rx_fifo_batch_need()` (`arq_common.cc:13802`) READS `messages_rx[0..data_batch_size)` lengths (no-comp) or `COMPRESS_WORKSPACE_SIZE` (comp).
- **A reads** `messages_rx_prev[target..old)` for the orphan test (`arq_common.cc:1251`).
- `bump_bsi_and_transfer_prev` (`arq_common.cc:9824`) transfers `messages_rx`→`messages_rx_prev`, freezes `prev_expected` from the D5 wired count clamped to `data_batch_size` (unchanged; the strand-safety A relies on).

### S6 — reverse-SACK de-dup tracker `cmd_last_applied_sack_bsi` (+ `cmd_last_applied_clean_bsi`) (CMD)
- Producer (wire, RSP): `cumulative_ack_bsi_field()` reshapes the on-wire bsi to `n_r` under the cap at `arq_responder.cc:1382/2323/2352/2534` (unchanged).
- **stall** re-keys the partial tracker write: `partial_sack_dedup_key(rx_bsi, cmd_batch_seq_id, cap_on)` = `cmd_batch_seq_id&0xFF` (cap on) / `rx_bsi&0xFF` (cap off), applied at BOTH consumers (MFSK `arq_commander.cc:~4412/4526`; OFDM SACK_RSP `:4721/4731`).
- **D** does NOT write S6 — it inserts a READ-ONLY reject (`partial_sack_target_is_inflight`) at the same two apply sites, gating whether the bitmap is applied at all.

---

## §2 CONSUMERS (every path that READS the shared state) + which fix perturbs it

### S1 `messages_tx[]`
- **C1 assemble** `process_messages_tx_data()` (`arq_commander.cc:2414`) builds the v2 mixbatch: R retx frames then ascending `ADDED_TO_LIST`→`pos=id=counter−R`. Consumes B1/B2's staging. B's cap guarantees `staged == data_batch_size−R` so C1 sends ALL staged, frees ALL each batch → slot order == content order (the mixbatch-fill.md §4 invariant).
- **C consumer** `maybe_backup_confirm_flush()` READS every slot's status to decide the flush. B (fewer surplus `ADDED_TO_LIST`) makes C's "zero un-confirmed" predicate reachable MORE often — see §4-I2.
- **C2 RSP delivery** places frames at `messages_rx[id]`, drains ascending (unchanged).

### S2 `data_batch_size`
- **copy_data_to_buffer / frame-driven prev-deliver** iterate `[0,data_batch_size)`; slots ≥ dbs cleared to FREE WITHOUT delivering — the ORPHAN mechanism A defends (`arq_common.cc:13746`, `arq_responder.cc:1309`).
- **rx_fifo_batch_need (H#3)** iterates `[0,data_batch_size)` summing RECEIVED/ACKED lengths.
- **ACK bitmap width** (`data_batch_size>=32 ? 0xFFFFFFFF : (1<<dbs)-1`) at every clean/partial encode.
- CMD fill cap (B) reads `data_batch_size`.

### S4 `fifo_buffer_backup`
- **Config-change re-stage** `inband_unilateral_config_change` restore (`arq_common.cc:~3570`) — the ONLY consumer that must NOT be starved. It needs ONLY UN-confirmed raw (in-flight data to re-frame at the new config). C flushes ONLY when zero un-confirmed exists ⇒ never removes raw this consumer needs (§4-I3).

### S5 `messages_rx[]` / `fifo_buffer_rx`
- **D3.1 delivery-time gap gate** + bsi bump (`arq_responder.cc:2496`) run AFTER H#3's hold check ⇒ on a hold they never run (nothing delivered) — correct.
- **copy_data_to_buffer** consumes RECEIVED slots on the NON-held path (unchanged).

### S6 de-dup tracker
- **Window resolver** `cumulative_ack_covers()` (`arq_commander.cc:4388/4708`) reads the RAW `n_r` for the backward self-heal `[n_r−W..n_r+1]` — UNTOUCHED by stall+D.
- **Clean de-dup** `sack_clean_confirmation_accepted` clean branch keys on `cmd_last_applied_clean_bsi` (clean n_r advances per delivery → never frozen) — UNTOUCHED.
- **compact-confirm** clean-only path — UNTOUCHED (and design-gated OFF under the cap anyway, stall-doc §5).

---

## §3 VALID STATES (enumerated, esp. the pre-write / uncommon states)

1. **Steady OFDM, cap OFF (legacy fleet).** Every fix is byte-identical here: B `retransmit_count==0`⇒`fill_limit==dbs`; A `sack_v2` false or no prev⇒no defer; stall/D key on `rx_bsi` (per-batch bsi) unchanged; C flush is new but idempotent w/ finalize; H#3 holds only under genuine app back-pressure. `rsp_deferred_batch_shrink=−1`.
2. **cap ON, n_r ADVANCING (healthy Tier-2 climb).** stall keys partials on the advancing `cmd_batch_seq_id`; D accepts (target==in-flight); C flushes each fully-confirmed batch; A defers only across an active partial prev.
3. **cap ON, n_r FROZEN, partials in flight (the post-leap stall state).** stall applies successive-batch partials (was the deadlock); D still accepts them (frozen-n_r current partial has target `n_r+1==cmd_bsi`, stall-doc §7.3 / D AP5a).
4. **cap ON, n_r FROZEN by app BACK-PRESSURE (H#3 hold).** NEW valid state introduced by H#3: a COMPLETE-but-undeliverable batch freezes `rsp_last_delivered_batch_seq_id` deliberately; the RSP sends NO reverse ACK; the CMD ACK-times-out and full-retransmits (legit flow-control stall). Distinct from state 3: there are NO partial SACKs to de-dup, so stall/D never engage on this batch. See §4-I5.
5. **Mid-partial-batch SHRINK (A defer active).** `data_batch_size` HELD large, `rsp_deferred_batch_shrink=target≥0`, an active prev holding RECEIVED slots in `[target,old)`. Cleared the instant the prev delivers.
6. **Mixbatch with retx pending, comp leg, `messages_tx` dirty (B2 force-FREE guard active).** Staging SKIPPED this tick; batch clears (ACK→FREE); next tick stages. No pop, no free ⇒ no data loss.
7. **Default-init / re-init.** `rsp_deferred_batch_shrink=−1` (init `:738`, re-init `:5768`); de-dup trackers `−1`; backup empty. No fix reads uninitialised state.

---

## §4 INVARIANTS each consumer assumes — verified per fix, cross-fix collisions checked

**I1 — `messages_tx[]` slot index == FIFO content order for new-data (C1 assemble ← B).**
B guarantees `staged == dbs−R` ⇒ every staged frame sent+freed each batch ⇒ P2 first-free re-fills from slot 0 in order. B2's force-FREE guard guarantees no live staged frame is destroyed. HOLDS. *Cross-fix:* C reads `messages_tx` status but never WRITES it (only flushes S4), so C cannot perturb I1. A/H#3 are RSP-side (no `messages_tx`). **No collision.**

**I2 — C's flush predicate "backup holds ONLY delivered raw" (C ← B, register_ack).**
C flushes IFF `retransmit_count==0` AND no `PENDING_ACK/ACK_TIMED_OUT/ADDED_TO_LIST/ADDED_TO_BATCH_BUFFER` slot. B's cap REDUCES leftover surplus `ADDED_TO_LIST` frames (old over-pop left R surplus that would BLOCK C's predicate). So **B strengthens I2's reachability — a positive interaction, not a collision.** Ordering within a poll: if `register_ack`(C) runs before the next fill(B), C sees a clean state and flushes, then B stages fresh from `fifo_buffer_tx` (not backup) — safe; if fill(B) runs first, `ADDED_TO_LIST` present ⇒ C correctly declines. Both orders safe.

**I3 — the config-change re-stage needs only UN-confirmed raw (S4 consumer ← C).**
`inband_unilateral_config_change` re-frames IN-FLIGHT (un-confirmed) data at the new config; a delivered batch's raw in the backup is pure liability (mixbatch-doc §7 Root-B: the +5 phantom re-send). C flushes ONLY when zero un-confirmed data exists ⇒ it can only remove ALREADY-DELIVERED raw ⇒ **C never starves the re-stage AND likely CLOSES the mixbatch-doc §7 Root-B residual** (the delivered ROBUST batch re-staged at the climb): if that batch is fully confirmed at register_ack, C empties it before the leap. This is a COHERENT positive interaction — the byte-integrity result (rx==tx, TASK 3) is the empirical confirmation. HOLDS. *Caveat (scoped, both docs agree):* a SUSTAINED unbroken loss run keeps `retransmit_count>0`, so C never fires mid-run — per-batch raw framing is the documented follow-on (fifo-backup §6.3); not a regression, the backup is capacity-bounded (128000 B).

**I4 — prev-batch delivery bound == the framed span the RSP received (A ← copy_data_to_buffer).**
copy_data_to_buffer / prev-deliver iterate `[0,data_batch_size)`. A holds `data_batch_size` at the OLD (larger) value while a prev holds RECEIVED slots in `[new,old)`, so the prev delivers its FULL span (no orphan hole). Strand-safety: `bump_bsi_and_transfer_prev` freezes `prev_expected` from the D5 per-batch count CLAMPED to `data_batch_size` (`arq_common.cc:9824`) ⇒ a LATER prev under the held-large size still expects the TRUE count, never over-expects. HOLDS. Stale-guard: `rsp_apply_deferred_batch_shrink` discards a deferred target that is no longer a genuine shrink (never grows dbs back up). **Cross-fix A∥H#3 (both RSP, both read `data_batch_size`):** H#3's `rx_fifo_batch_need` sums only RECEIVED/ACKED slot lengths over `[0,dbs)`; a held-large dbs only extends the scan over FREE slots (length 0 contribution) ⇒ `need` is unaffected by A's hold ⇒ no false over-hold. A operates on `messages_rx_prev` (prev batch); H#3 on `messages_rx` (current batch) — disjoint arrays. **No collision.**

**I5 — reverse-SACK crediting (S6 ← stall+D, the co-designed pair).**
stall re-keys the partial de-dup on `cmd_batch_seq_id` (the in-flight batch a bitmap maps to by slot index); D rejects any partial whose resolved target `n_r+1 != cmd_bsi`. These are complementary gates on the SAME two apply sites and were built together. D preserves stall by construction: the current-batch partial stall targets has `n_r = cmd_bsi−1` ⇒ `target = n_r+1 = cmd_bsi` ⇒ D accepts (regression AP5a). The window resolver `cumulative_ack_covers` (raw n_r) and the clean tracker are untouched by both ⇒ self-heal + clean confirms intact (stall-doc §7.3). **stall∥D COHERE — the ONE deliberate same-state pair, tests prove it (AP1–AP5c).**
*Cross-fix with H#3 (state 4):* H#3's app-back-pressure hold freezes n_r WITHOUT emitting partials, so it never presents the frozen-n_r-with-partials input stall/D govern. The two freeze mechanisms (de-dup deadlock vs deliberate flow-control) are disjoint. **No collision.**

**I6 — INV3 "never ACK/free un-confirmed data" (H#3, C, A all assert a form of it).**
- H#3: never ACKs a batch whose delivered tail would be dropped ⇒ no ACKed-but-undelivered byte.
- C: never flushes un-confirmed raw (predicate I2).
- A: never orphans RECEIVED prev bytes (defers the shrink).
All three are INDEPENDENT strengthenings of the same "no silent loss" property at three different chokepoints (RX app boundary / CMD backup / RSP delivery bound). They do not share mutable state across the assertions, so they compose additively. HOLDS.

---

## §5 WHAT THE COMBINED CHANGE ALTERS (and the re-walk of every consumer)

The merged set changes FIVE decisions, each on a distinct chokepoint, each guarded so `cap OFF / retransmit_count==0 / no-prev / FIFO-has-room` is byte-identical to `monitor`:
1. **How many new frames stage per mixbatch** (B: cap by `dbs−R`) — consumer C1 re-walked: sends all staged, invariant I1 restored.
2. **When `data_batch_size` may shrink** (A: defer across an orphaning prev) — consumers copy_data_to_buffer / prev-deliver / rx_fifo_batch_need / ACK-width re-walked (I4): prev delivers full span; H#3 `need` unaffected; ACK width transiently wider is harmless (RSP encodes for the live dbs).
3. **When delivered raw leaves the backup** (C: ACK-confirm flush) — consumer inband_unilateral_config_change re-walked (I3): only delivered raw removed; re-stage never starved; §7 Root-B residual likely closed.
4. **When a complete batch may ACK+deliver** (H#3: gate on app-FIFO room) — consumers D3.1/bsi-bump/copy_data_to_buffer re-walked: all sit downstream of the hold and simply don't run on a hold; CMD retransmit re-fires the handler.
5. **How a partial SACK is credited/validated under the cap** (stall re-key + D reject) — consumers window-resolver/clean-de-dup/compact-confirm re-walked (I5): untouched; only the partial de-dup gate changed, on both transports consistently.

**No fix writes a state another fix reads in a way that violates the reader's invariant.** The two intra-role same-state pairs — **B∥C on `messages_tx`+backup** and **stall∥D on the de-dup tracker** — are BOTH coherent (B strengthens C's predicate; D was co-designed to preserve stall). The two RSP-side fixes — **A∥H#3** — read a common scalar (`data_batch_size`) but operate on disjoint arrays and A's held-large value provably does not perturb H#3's need computation.

---

## §6 VERDICT — the fixes COHERE (they do not collide)

- **Cross-endpoint (5 of the pairs):** CMD-side {stall, D, B1, B2, C} vs RSP-side {A, H#3} touch disjoint arrays in different roles — structurally cannot collide.
- **CMD-TX cluster {B1, B2, C}:** B caps staging and never destroys live frames; C only READS slot status and flushes ONLY delivered backup raw. B *helps* C (fewer surplus frames blocking the flush predicate); C likely closes the residual mixbatch §7 Root-B that B explicitly scoped out. Coherent + additive.
- **CMD-ACK pair {stall, D}:** the one deliberate same-state pair, co-designed; D preserves stall by construction; `--test-climb-engine` AP1–AP5c proves both arms in one binary.
- **RSP pair {A, H#3}:** share `data_batch_size` read-only; A's held-large value contributes only zero-length FREE slots to H#3's `need`; disjoint message arrays. Coherent.
- **Global safety:** all seven fixes are independent strengthenings of ONE property — "no silent delivered-byte loss/reorder" — at seven distinct chokepoints; every one is gated so the whole non-Tier-2 / no-back-pressure / cap-OFF fleet stays byte-identical, and every one carries a same-binary fail-before toggle + an in-master-suite regression.

**Merge-readiness (this document's scope): the combined change is internally coherent; no invariant one fix relies on is broken by another.** The empirical gates that confirm it end-to-end are the full `--test` (all regressions pass together in one binary) and the N≥8 byte-integrity-clean baseline (rx==tx on every completed transfer) — reported alongside this audit.

### Open items (documented, not regressions)
- mixbatch §7 Root-B (+5 phantom) — expected CLOSED by C (I3); confirm via TASK-3 byte-integrity.
- fifo-backup §6.3 per-batch raw framing — the sustained-loss-run flush case C scopes out; capacity-bounded meanwhile.
- Tier-2 stall-doc §8.3 combined `(rx_bsi,bitmap)` content key — D covers the LATE-straggler case; the content-pair hardening remains a belt-and-suspenders follow-on.

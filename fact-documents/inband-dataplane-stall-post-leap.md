# Data-Flow Audit — in-band data-plane STALL after the SUPER-ACK leap holds cfg8

Build: manifest `integ/capstone-manifest @ 226b1ae9` (`Mercury 0.4.2 build 226b1ae9`).
Env (arm B, full-stack): `MERCURY_INBAND_RATE=1 ARQ_COMPACT_CONFIRM_ENABLE=1
MERCURY_HARQ_CHASE=1 MERCURY_TINTERP_SEED=1`, reverse-pin default-on.
Evidence: capstone `TOP_B` arm-B raw ARQ trace, primarily
`/dev/shm/scratch/capstone/results/logs_TOP_B/arq_TOP_B00.log` (fleet .31), durable
summaries in `_research/capstone_R1/` (`SCORE.txt`, `TOP_B.json`). No re-run needed — the
post-leap trace was intact.

Symptom (capstone wgu71bih5, honest negative): full-stack connects, SUPER-ACK leaps
ROBUST→cfg8 and the leap HOLDS, then the in-band data plane stalls: `active_fraction 0.003`
(vs legacy 0.042), ~12–16 s reverse-ACK listen windows, ONE batch delivered then near-silence,
`compact_confirm_ok=0` everywhere. 10–45× WORSE delivered than the legacy monitor baseline.

---

## §1 ROOT (one sentence)

**It stalls because, once the cumulative-ACK cap is negotiated (FORGIVING-ACK Tier 2,
`cap=0x05`), the RSP writes `n_r` (the contiguous delivery high-water,
`rsp_last_delivered_batch_seq_id`) into the reverse-SACK bsi field instead of the per-batch
bsi (`cumulative_ack_bsi_field`, `include/common/common_defines.h:709`), so after the first
delivered batch every subsequent partial SACK carries the SAME frozen wire value `bsi=0`; the
CMD's SACK de-dup guard `sack_clean_confirmation_accepted()`
(`include/datalink_layer/arq.h:1651`) keys on that raw `rx_bsi`, so it discards every
post-batch-0 partial SACK as a "duplicate" (`arq_commander.cc:4412` de-dup call,
`:4526` tracker set), the bitmap is never applied, `stats.nAcked_data` never advances, and the
CMD's cheap-miss discriminator escalates "NO forward-ACK progress" → re-air cap → BREAK/demote
(`arq_commander.cc:5588–5660`). The reverse SACK is DECODED but never CREDITED — a deterministic
cross-layer collision between the Tier-2 wire encoding and the SACK de-dup key.**

The frame-0 rolling-partial loss (`data-flow-inband-frame0-rolling-partial.md`) is the TRIGGER
(every post-tier-cross OFDM batch loses seq-0 on first TX, so every batch is a PARTIAL and the
SACK path is load-bearing). That loss is *tolerated* in legacy (cap OFF, SACK recovers it one
batch late). With the cap ON it becomes a hard deadlock.

Determinism: **deterministic bug, not an architectural limit.** Fixable in the de-dup key /
encoding contract. Matches the leap-stick precedent (a discrete state-machine defect, not a
channel wall). Reproduces on all 7 arm-B cells (SCORE.txt: `active_fraction` 0.003 ± 0,
`retx_rounds`=1, `set_config`=0, GOOD user B/min ≈ 327 flat).

---

## §2 Post-leap event trace (arq_TOP_B00.log, one batch = 25 frames, cfg8)

Negotiation (both sides): `T+43.342 [RSP]` / `T+57.828 [CMD]`
`[FORGIVING-ACK-T2] cumulative-n_r SACK NEGOTIATED (local_cap=0x05 peer_cap=0x05)` → **cap ON**.

Leap (HOLDS):
- `T+84.808 [RSP] [RSP-SUPERACK] clean ROBUST decode … -> recommend CONFIG_8` (inline suffix).
- `T+87.202 [CMD] [CMD-SUPERACK] … DIRECT LEAP`; `[SUPERACK] … rolling cmd_batch_seq_id 1 -> 0`;
  `[INBAND-TX] UNILATERAL CONFIG 100 -> 8`.
- `T+89.570 [RSP] [INBAND-RX] CONFIG_TAG follow … CONFIG_8 (was 100)` → adopt; `T+90.333`
  ROBUST-TIER tag-follow, BREAK avoided; `T+112.870` DOWN-LADDER RESYNC CONFIG_8 **DECODED**.
  RSP is genuinely on cfg8 and decoding cfg8 data.

Batch 0 (bsi=0) — the ONLY batch that ever delivers:
- `T+87.598 [CMD] [CMD-BATCH-SEQ] new-data batch_seq_id=0 (frames in batch=25)`.
- RSP receives seq 1..24, misses seq-0 (rolling frame-0 loss).
- `T+109.896 [RSP] [ACK-GATE] SACK: received 24/25`;
  `[RSP-MFSK-SACK] partial path: batch_seq_id=0 (wire_bsi=0 n_r=0 cum=1) bitmap=0x01fffffe`.
  **n_r just went −1→0** (batch 0 not yet delivered at first partial; fallback made wire_bsi=0).
- `T+111.158 [CMD] [CMD-MFSK-ACK-SACK] PARTIAL … bitmap=0x01fffffe matched=16` — CREDITED
  (first partial, de-dup tracker empty). `T+111.390 [CMD] stats.nAcked_data= 25`.
- `T+113.150 [RSP] [RSP-V2-PREV-DELIVERED] prev_batch_seq_id=0 deliveries_total=1
  last_delivered=0`. **First and ONLY delivery. n_r now frozen at 0.**

Batch 1 (bsi=1) onward — the deadlock:
- `T+111.762 [CMD] [CMD-BATCH-SEQ] new-data batch_seq_id=1 (frames in batch=25)`; RSP receives
  seq 3..23.
- `T+133.029 [RSP] [ACK-GATE] SACK: received 21/25`;
  `[RSP-MFSK-SACK] partial path: batch_seq_id=1 (wire_bsi=0 n_r=0 cum=1) bitmap=0x00fffff8`.
  **RSP internally knows batch=1, but the WIRE carries bsi=0 (= n_r).**
- `T+135.x [CMD] [CMD-MFSK-ACK-SACK-MW] late ACK+SACK found … bsi=0 bitmap=0x00fffff8 matched=16`
  — the CMD DECODES it (repeatedly across phases 1/3,2/3,3/3) but never applies it.
  `stats.nAcked_data` stays **25** for the rest of the run (only two distinct values all run:
  `T+5.061`→0, `T+111.390`→25; never advances again).
- `T+160.585 [CMD] [REVSACK-CHEAPMISS] data-SACK miss at config 8 — re-air cap 3 reached with
  NO forward-ACK progress; ESCALATING to the demote/BREAK path`; `[BREAK] Block failure #1`.
- Repeats: BREAK `#2 T+203.774`, `#3 T+247.197`. RSP received batch-1 frames **93 times**
  (`RX-BATCH-SEQ batch_seq_id=1` ×93) yet delivered it **0 times** — full-batch BREAK re-sends
  that keep re-dropping the marginal first frame, with the corrective SACK bitmaps de-dup'd away.
- `T+247.2+ [RSP] [OFDM-SYNC] 3 consecutive SKIP-VAR — abort trial loop` (spam);
  `T+264.716 [CMD] [LINK-TIMEOUT] Commander retrying connection at init config`. Link collapses,
  limps to end (`T+643`).

Distinct wire values proving the freeze (`arq_TOP_B00.log`, RSP-MFSK-SACK):
`batch_seq_id=0 → wire_bsi=0`, `batch_seq_id=1 → wire_bsi=0`, `batch_seq_id=2 → wire_bsi=0`.

---

## §3 Static trace — the collision (producer vs consumer of the on-wire bsi field)

Shared state: the 8-bit bsi field carried in the MFSK ACK+SACK / SACK_RSP frame.

**Producer (RSP), `cumulative_ack_bsi_field` — `include/common/common_defines.h:709`:**
```
inline unsigned char cumulative_ack_bsi_field(unsigned char per_batch_bsi,
        int high_water, bool cap_on) {
    if (cap_on && high_water >= 0)
        return (unsigned char)(high_water & 0xFF);   // = n_r (rsp_last_delivered_batch_seq_id)
    return per_batch_bsi;                             // legacy = per-batch bsi
}
```
Call sites (all pass `rsp_last_delivered_batch_seq_id` as high_water):
`arq_responder.cc:2323` (partial), `:2534` (clean), `:1382` (prev-delivered). With cap ON and
`last_delivered=0`, the field is **0 for every batch** until a NEW delivery advances n_r — which
never happens, because the SACK that would enable that delivery is itself de-dup'd (below).

**Consumer 1 — batch resolver `cumulative_ack_covers` (CORRECT).** `arq_commander.cc:4382–4391`
resolves the true in-flight target: with cap ON, `rx_bsi=n_r` addresses target in
`[n_r-W .. n_r+1]`, so batch 1 (`= n_r+1`) IS in-window (`bsi_in_window == true`). The self-heal
math is fine; it is NOT the failing gate.

**Consumer 2 — de-dup guard `sack_clean_confirmation_accepted` (THE FAILING GATE).**
`include/datalink_layer/arq.h:1651`:
```
static bool sack_clean_confirmation_accepted(int rx_bsi, bool is_all_ones,
                                             int last_applied_clean_bsi,
                                             int last_applied_sack_bsi) {
  if (is_all_ones) return rx_bsi != last_applied_clean_bsi;   // clean tracker
  return rx_bsi != last_applied_sack_bsi;                      // partial tracker
}
```
Applied at `arq_commander.cc:4412` as `duplicate = !sack_clean_confirmation_accepted(rx_bsi,
is_clean, cmd_last_applied_clean_bsi, cmd_last_applied_sack_bsi)`, and the trackers are set to
the raw `rx_bsi` on apply (`:4526` partial, `:4429`/`:4731` clean). This guard was written for
per-batch bsi semantics (each batch's bsi is distinct). Under cap ON the field is `n_r`, which
**repeats across distinct in-flight batches** whenever n_r is frozen:
- batch 0 partial applied → `cmd_last_applied_sack_bsi = 0`.
- batch 1 partial arrives with `rx_bsi = n_r = 0` → `0 != 0` = false → **duplicate → discarded
  before the `if(bsi_in_window && bitmap_ok && !duplicate)` apply block (`arq_commander.cc:4420`).**
  `cumulative_ack_covers` said "apply it"; the de-dup says "seen it." De-dup wins → nothing applied.

**Escalation — `arq_commander.cc:5588–5660`.** The cheap-miss discriminator resets its re-air
streak only when `stats.nAcked_data != cmd_revsack_reair_last_acked` (a decode advanced). With
every batch-1 SACK discarded, `nAcked_data` is pinned at 25, the streak reaches
`REVSACK_CHEAPMISS_MAX_REAIRS`, and the code prints "NO forward-ACK progress" and falls through
to `emergency_nack`/BREAK/demote. The 12–16 s listen windows are `calculate_receiving_timeout`
re-arming for a reverse-ACK that, even when it arrives and decodes, is thrown away.

Deadlock closure: batch 1 never completes → `last_delivered` stuck at 0 → `n_r` stuck at 0 →
`wire_bsi` stuck at 0 → all future SACKs de-dup'd. Self-reinforcing; irreversible without BREAK,
and BREAK only full-re-sends (re-dropping the same marginal frame) without fixing the key.

Verdict on the four candidate mechanisms in the brief:
- (a) "CMD never advances past first batch waiting on a reverse-ACK" — TRUE as the visible
  symptom, but the ACK is not missing; it arrives and decodes (§2, MW lines).
- (b) "RSP reverse-ACK malformed/mistimed for cfg8" — FALSE; the SACK is well-formed, CRC-valid,
  and matched=16 at the CMD.
- (c) "BSI/batch-state desync from the leap" — **TRUE and this is the root**, but the desync is
  not the leap itself; it is the cap-ON `wire_bsi=n_r` encoding colliding with the de-dup key.
- (d) other — no.

---

## §4 A/B contrast (why legacy arm A does NOT stall)

`logs_TOP_A/arq_TOP_A00.log`: `[FORGIVING-ACK-T2] cumulative-n_r SACK **off** (local_cap=0x01
peer_cap=0x01)`. Cap OFF → `cumulative_ack_bsi_field` returns the **per-batch** bsi, which is
distinct every batch (observed `wire_bsi = n_r = 10,11,12,…,17,…`, one per delivered batch). The
de-dup key therefore differs per batch → no false de-dup → SACK recovery works → `active_fraction
0.042`, GOOD B/min ~14,797. The bug is **switched on by the cumulative-ACK cap that only arm B
negotiates**, and armed by the frame-0 rolling partial that makes every batch a PARTIAL.

---

## §5 Compact-confirm `ok=0` — RESOLVED (design mutual-exclusion, NOT an independent bug, NOT a symptom)

- Compact-confirm IS compiled in on the manifest: `include/common/common_defines.h:100`
  `#define ARQ_COMPACT_CONFIRM_ENABLE 1` (the res-JSON `arm_lever_warnings` "held-off default 0 /
  env is a no-op" text is STALE harness knowledge for this commit).
- It is DESIGN-GATED OFF whenever cumulative-ACK is negotiated. Dispatch gate
  `arq_responder.cc:2554–2557`:
  ```
  if (ARQ_COMPACT_CONFIRM_ENABLE
      && !cumulative_ack_enabled            // <-- suppressed when cap ON
      && … compact_confirm_suffix_len() > 0
      && wire_bsi == ack_bsi) { send_mfsk_compact_confirm(…) }
  ```
  The comment states the intent: "NOT used when cumulative-ack is negotiated … the compact path
  requires `wire_bsi == ack_bsi`." Arm B has the cap ON, so `!cumulative_ack_enabled == false`
  → compact-confirm is UNCONDITIONALLY skipped. Hence `compact_confirm_ok=0` everywhere.
- Conclusion: compact-confirm and cumulative-ACK are **mutually exclusive by design; the cap
  wins.** `ok=0` is EXPECTED and says nothing about the stall. It is neither an independent bug
  nor a symptom of the stall — it shares the SAME upstream switch (cumulative-ACK negotiated).

So one switch — the negotiated cumulative-ACK cap — both (a) reshapes `wire_bsi` into the
de-dup collision that STALLS the plane, and (b) disables compact-confirm.

---

## §6 Fix SKETCH (do NOT implement under the freeze)

Root contract violation: under cap ON the wire bsi is `n_r`, which is NOT a per-batch identity,
but the SACK de-dup treats it as one. Fix the de-dup to key on batch IDENTITY + bitmap CONTENT,
not the raw `n_r`. Options, least-invasive first:

1. **De-dup on (resolved target, bitmap), not raw rx_bsi (preferred).** The apply site already
   resolves the true in-flight batch via `cumulative_ack_covers` (= `n_r+1` for a partial). Key
   `cmd_last_applied_sack_bsi` on that RESOLVED target (`cmd_batch_seq_id`), and/or additionally
   require the bitmap to differ from the last-applied bitmap to count as a duplicate. Rationale:
   the de-dup exists only to avoid re-counting an IDENTICAL repeated burst; a partial SACK whose
   bitmap changed (or whose resolved batch changed) is genuinely new and must be applied. This
   preserves both FORGIVING-ACK self-heal and compact-confirm's separate path.
2. **Track a `(rx_bsi, bitmap_u32)` pair** for the partial de-dup instead of `rx_bsi` alone. A
   new batch under a frozen `n_r` presents the same `rx_bsi` but a different bitmap → not a dup.
   Smallest diff; keys on content, which is correct.
3. (Reject) Un-freeze `wire_bsi` by always sending per-batch bsi — breaks the Tier-2 self-heal
   the cap was built for; do not.

Guardrails / cross-layer audit to pair with the fix (per CLAUDE.md §Cross-Layer):
- Producer set: `cumulative_ack_bsi_field` sites `arq_responder.cc:2323/2534/1382`.
- Consumer set: de-dup `arq_commander.cc:4412` + trackers `:4429/:4526/:4731`; window resolver
  `:4382–4391`; the bare-arm clean-ACK CRC path `arq_commander.cc:~304/378` uses the same helper
  and must get the same key fix. Verify the OFDM `SACK_RSP` transport (`send_sack_v2_frame`,
  `:2352`) does not carry the same collision.
- The frame-0 rolling-partial (`data-flow-inband-frame0-rolling-partial.md`) is the amplifier: it
  makes EVERY cfg8 batch a partial, so the de-dup fix is load-bearing for sustained delivery.
  A second, independent hardening (reduce the seq-0 acquisition drop at the tier-cross) would cut
  the retx load but is NOT required to end the stall.

Required regression (fails-before / passes-after, in-process synthetic-fire, no RF): drive
cap-ON, deliver batch 0, then present batch 1 as a PARTIAL (frame-0 missing) with `wire_bsi=n_r`
frozen; assert the CMD APPLIES the batch-1 SACK (`nAcked_data` advances) instead of de-dup'ing it
(a `-DCUMULATIVE_ACK_DEDUP_FAILBEFORE` pin should reproduce the stall). Extend the existing
`sack_clean_confirmation_accepted` unit block (`arq_commander.cc:12641`) with a frozen-n_r case.

---

## §7 FIX IMPLEMENTED — partial-SACK de-dup re-keyed off the frozen n_r

Implements §6 (options 1+2 combined). The CMD partial-SACK de-dup is re-keyed from the
raw frozen wire bsi (= n_r under the cap) onto the IN-FLIGHT BATCH IDENTITY the bitmap
is actually applied to (`cmd_batch_seq_id`). New PURE static predicate
`cl_arq_controller::partial_sack_dedup_key(rx_bsi, cmd_batch_seq_id, cap_on)`
(`include/datalink_layer/arq.h`, immediately after `sack_clean_confirmation_accepted`):
- **cap ON**  → `cmd_batch_seq_id & 0xFF` (advances per batch; a new in-flight batch
  under a frozen n_r is genuinely NEW, not a duplicate).
- **cap OFF** → `rx_bsi & 0xFF` (legacy per-batch bsi — byte-identical; the whole
  non-Tier-2 fleet and every existing test stay bit-for-bit).
`-DCUMULATIVE_ACK_DEDUP_FAILBEFORE` pins the frozen-n_r key (reproduces the stall).

### §7.1 SECOND CONSUMER — the §6 open item RESOLVED (cross-layer audit payoff)
§6's guardrail asked to "verify the OFDM SACK_RSP transport does not carry the same
collision." **It DOES.** `cumulative_ack_bsi_field` reshapes the wire bsi to n_r on
BOTH reverse-SACK transports, and BOTH CMD consumers share the SAME tracker
`cmd_last_applied_sack_bsi`:
- Producers (RSP): MFSK partial `arq_responder.cc:2323/2334`; OFDM SACK_RSP
  `arq_responder.cc:2352/2354` — both call `cumulative_ack_bsi_field(sacked_bsi,
  rsp_last_delivered_batch_seq_id, cumulative_ack_enabled)` → n_r when cap on.
- Consumers (CMD): MFSK ACK+SACK de-dup `arq_commander.cc:4412` (helper) + tracker
  write `:4526`; OFDM SACK_RSP de-dup `arq_commander.cc:4721` (direct compare) +
  tracker write `:4731`.
Fixing only the MFSK consumer would leave the OFDM path broken AND write an
INCONSISTENT key into the SHARED tracker (a landmine per CLAUDE.md §Cross-Layer). So
the SAME `partial_sack_dedup_key` is applied to BOTH consumers; the shared tracker now
always holds the in-flight batch identity (cap on) / per-batch bsi (cap off) regardless
of which transport last wrote it.

### §7.2 The 5-question audit (verified against 226b1ae9 source)
1. **Producers of the reverse-SACK bsi**: cap-ON → n_r via `cumulative_ack_bsi_field`
   at `arq_responder.cc:1382` (prev-delivered), `:2323/2334` (MFSK partial),
   `:2352/2354` (OFDM SACK_RSP), `:2534` (clean). cap-OFF → per-batch `sacked_bsi`.
2. **Consumers of rx_bsi + the de-dup tracker**: window resolver
   `cumulative_ack_covers` (`arq_commander.cc:4388-4392` MFSK, `:4708-4711` OFDM) —
   reads raw n_r, UNCHANGED. Partial de-dup: MFSK `:4412` + OFDM `:4721` (fixed).
   Clean de-dup: `sack_clean_confirmation_accepted` clean branch keyed on
   `cmd_last_applied_clean_bsi` (`:4429`, compact-confirm `:378/390`) — UNCHANGED
   (clean n_r advances per delivery, never frozen). Tracker reset: RSP-side
   `arq_responder.cc:13048`.
3. **Valid states**: cap-ON, n_r FROZEN (no new delivery) — the stall state, every
   post-first partial repeats n_r. cap-ON, n_r ADVANCING — clean confirms bump it per
   delivery. cap-OFF — per-batch bsi distinct per batch. Default-init tracker = -1.
4. **Invariants each consumer assumes**: `cumulative_ack_covers` assumes rx_bsi is the
   raw n_r (backward-window self-heal `[n_r-W..n_r+1]`) — the fix does NOT touch it.
   The partial de-dup assumed rx_bsi was a per-batch identity (FALSE under cap) — the
   defect; the fix supplies the true identity (`cmd_batch_seq_id`, the batch the bitmap
   maps to by SLOT INDEX). The clean de-dup assumes the clean wire bsi advances per
   delivery — TRUE, unchanged.
5. **What the fix changes**: ONLY the partial de-dup KEY (frozen n_r → in-flight batch
   identity), on BOTH consumers of the shared tracker. Re-verified every consumer:
   window resolver (untouched), clean de-dup (untouched), compact-confirm (clean-only,
   untouched), OFDM SACK_RSP (fixed consistently).

### §7.3 How the fix PRESERVES the Tier-2 self-heal (the invariant §6 flagged)
The self-heal lives in `cumulative_ack_covers` (`arq_commander.cc:4388-4392`,
`:4708-4711`), which RESOLVES which batch a report covers by reading the raw frozen
n_r over the bounded window `[n_r-W .. n_r+1]`. The fix does NOT touch the wire
encoding, does NOT touch `cumulative_ack_covers`, and does NOT change what value flows
into it — rx_bsi (= n_r) is still passed RAW. Only the downstream DE-DUP gate is
re-keyed. The CLEAN tracker still keys on rx_bsi (clean wire bsi advances on every
delivery → never frozen). Net: self-heal fully intact; de-dup no longer false-collides.

### §7.4 Regression (arq_commander.cc `test_climb_engine` Part A', CLI `--test-climb-engine`)
Fail-before/pass-after in one binary via `-DCUMULATIVE_ACK_DEDUP_FAILBEFORE`:
- **AP1**: cap-ON partial key DISTINCT for batch1 vs batch0 under a frozen n_r=0.
- **AP2 (THE stall assertion)**: batch1 PARTIAL under a frozen n_r is APPLIED, not
  deduped (→ nAcked_data advances).
- **AP3**: a repeated batch1 partial (same in-flight batch) is still deduped (no retx
  re-populate).
- **AP4**: cap-OFF legacy per-batch key unchanged.
FAIL-BEFORE build → AP1+AP2 FAIL (deduped → nAcked_data freezes). FIXED build → all
PASS. Full `mercury --test` unaffected (all-pass).

### §7.5 NOTE for owner — compact-confirm ⟂ cumulative-ACK design redundancy (do NOT fix here)
Per §5, compact-confirm is DESIGN-GATED off whenever the cumulative cap is negotiated
(`!cumulative_ack_enabled` gate, `arq_responder.cc:2554-2557`), so it is INERT in the
manifest arm-B config (the cap and compact-confirm are mutually exclusive — the cap
wins). Two reverse-ACK-cheap features that can never co-engage → flag as a
design-redundancy follow-up for the owner, NOT part of this stall fix.

---

## §8 DATA-PLANE-SUSTAINS SMOKE (fixed binary af13507) — stall RESOLVED, EOB sibling bug EXPOSED

Real-audio smoke on fleet .31 (snd-aloop Loopback_B, bridge WGN SNR3k=40, cfg100 start,
arm `full_stack` = MERCURY_INBAND_RATE=1 + reverse-pin + tinterp + ack_slot, pg84 4096 B,
-W WB, gearshift on). Log `logs/arq_sackdedup2_af13507.log`.

### §8.1 The stall is RESOLVED (all STEP-5 sustain criteria met)
| metric | PRE-FIX (§2 capstone TOP_B) | POST-FIX (smoke2, af13507) |
|---|---|---|
| cap negotiated | cap=0x05 ON | cap=0x05 ON (T+43s) — same trigger |
| leap | ROBUST->cfg8 held | ROBUST->cfg8 held (T+84-86s) |
| batches delivered | **1** (deliveries_total=1) | **3** (deliveries_total=3); num_batch_deliveries=4 |
| nAcked_data | **frozen at 25** all run | **0 -> 25 -> 49 -> 62** (advances) |
| REVSACK-CHEAPMISS escalate | 3 -> BREAK | **0** |
| BREAKs | **3** (#1/#2/#3) | **0**; demote_events 0 |
| active_fraction | **0.003** | **0.036** (12x; > legacy 0.042 ballpark) |
| forward delivery | ~1 batch/600s stall, link collapses | full 4096 B delivered, wall 156.6 s |
| climbed past ROBUST | (stalled at cfg8) | True (cfg8) |

The de-dup fix does exactly what §1/§3 predicted: the partial SACKs for batches 1,2 are
now APPLIED (not discarded as frozen-n_r duplicates), so nAcked_data advances, the missing
frame-0 of each batch is retransmitted, each batch completes, n_r advances, and the
REVSACK-CHEAPMISS BREAK loop never arms. The first 3288 delivered bytes (batches 0 and 1)
are byte-PERFECT, confirming the de-dup fix is correct for every batch it governs.

### §8.2 EXPOSED sibling bug — final PARTIAL batch delivered as FULL 25 frames (EOB length)
The completed transfer HARD-FAILS byte-integrity: `byte_integrity_ok=false`,
`good_prefix_bytes=3288`, `integrity_mismatch_bytes=813` (3 segments from offset 3288),
`rx_bytes=4101 > tx_bytes=4096` (double-delivery, `delivered_exceeds_fed=true`).
ROOT (RSP forward/EOB path, NOT the CMD reverse-SACK de-dup this fix touches): the FINAL
partial batch is batch 2, which the CMD sent as `[CMD-V2-MIXBATCH] 13 new bsi=2` (13 real
frames), yet the RSP delivered it as `[RSP-V2-PREV-DELIVERED] prev_batch_seq_id=2 ...
bitmap=0x01ffffff nframes=25` — a FULL 25-frame batch. The extra ~12 frames are stale
cross-storage buffer content ("cross-storage path drained; current-batch storage
untouched") that decompress to garbage -> the 813 corrupted tail bytes + 5-byte overage.
The delivered frame count is set by the RSP EOB inference / `rx_buffer_batch_total_frames`
(arq_responder.cc ~:1479) on the FORWARD data path; nothing the CMD reverse-SACK de-dup
controls can set it. It is PRE-EXISTING, masked by the stall (pre-fix never completed a
transfer to reach a final partial batch), and EXPOSED for the first time by this fix
letting the plane run to completion (the exact "fixing one layer exposes a sibling bug"
pattern, CLAUDE.md §Cross-Layer). It must be fixed BEFORE Tier-2 cumulative-ACK can ship
default-on. Candidate area: the final-partial-batch EOB length inference + the cross-storage
prev-delivered drain count (see --test-eob-loss-batch-truncation, [[eob_loss_batch_truncation]]).

### §8.3 Secondary observation — stale prev-batch SACK now APPLIES under the cmd_batch_seq_id key (benign here)
With the new key, a LATE `[CMD-MFSK-ACK-SACK] PARTIAL batch_seq_id=1 (cmd_batch_seq_id=3)`
(wire bsi=1 = a stale n_r from batch-2 in-flight) is APPLIED, whereas the old raw-rx_bsi key
de-duped it (batch 2's own partials also carried wire bsi=1, so the tracker already held 1).
Benign in this run — batches 0/1 delivered byte-perfect and the affected current batch
delivered CLEAN — because the bitmap maps by slot index onto an already-satisfied batch. But
it is a latent robustness gap: a future hardening should reject a partial whose resolved
target lies at-or-below the delivery high-water (already retired), e.g. combine the
cmd_batch_seq_id key with a "not below n_r" guard or the (rx_bsi,bitmap) content pair.
NOT the cause of §8.2.

# Root-Cause: the RESIDUAL silent byte-corruption at marginal SNR (WGN:25)

**Status (2026-07-03):** ROOT **CONFIRMED** from the captured failure `res_c3100`
(monitor binary `mercury_1e60` = `1e60ef80`). The THIRD distinct silent-false-accept
trigger, DISTINCT from the two already-landed fixes (Fix A = `57684aa1`; EOB-undercount
tail-drop = D5 wired count). It survives the whole `integ/correctness-consolidation`
set (Findings 1-3 of `delivery-integrity-audit-monitor.md`).

**★ TRUE ROOT: a CMD↔RSP `data_batch_size` DESYNC (CMD=30, RSP=25) at bsi 11.** This is
the SHARED-STATE class that `data-flow-batch-size.md` already owns (that doc exists
because this exact CMD/RSP batch-mismatch failed the wire FOUR times, a different
producer each time). `data-flow-batch-size.md §3.1` documented the desync's consequence
as a *stall* ("no clean confirmation"); **this capture proves a WORSE, undocumented
consequence: SILENT DATA CORRUPTION** — the RSP delivers the batch TRUNCATED at its own
smaller size, orphaning the sender's extra frames → a permanent one-batch stream shift.

> **CORRECTION (visible, per CLAUDE.md):** the first draft of this doc (same day)
> blamed the `fifo_buffer_backup` / BREAK re-stage (Root-B ACK-loss variant). ~~That was
> the ROOT.~~ It is a DOWNSTREAM CONSEQUENCE: the batch-size desync makes the RSP emit a
> 25-bit clean bitmap that mismatches the CMD's 30-bit all-ones target, so the CMD never
> credits bsi 11 → retransmit storm → BREAK → the re-stage. Root-cause is the desync;
> the re-stage/BREAK cascade is what the desync TRIGGERS. §5 keeps the re-stage analysis
> as the consequence chain.

**Provenance:** monitor `1e60ef80`, read in `C:/Users/kamer/mercury_wt/merge-followgate`.
Capture at `_research/_res_c3100_capture/` (res JSON + 2.2 MB `arq_c3100.log`, CMD+RSP).
Fleet source: `192.168.2.31:/dev/shm/wgn25/logs_c31/`.

---

## §1 The symptom (`res_c3100.json`)

- `traffic="random-binary"`, 262144 B fed; `rx_bytes=35781` (partial — aborted at
  bsi 15). `byte_integrity_ok=false`, `integrity_mismatch_bytes=8350`,
  `integrity_first_bad_offset=27431`. **`35781 − 27431 = 8350`** — the ENTIRE tail from
  27431 is corrupt (random-binary makes any misalignment mismatch ~255/256, so it reads
  as "7 segments").
- `uniqueness_ok=true`, `delivered_exceeds_fed=false` are **NOT health signals** — both
  only test `rx_total > tx_total`, which cannot fire on a partial transfer; they are
  BLIND to an internal shift (harness fix, §7).
- Compression OFF (`Force compression: off`, B2F never detected, no encryption) →
  codec RAW → **streaming PPMd/zstd desync RULED OUT**. Climb reached cfg16; corruption
  is MID-STREAM after a good climb; `demote_events=7`, `break_events=3`.

## §2 Byte arithmetic pins the fault to the bsi-11 25-frame boundary

Per-batch delivered size from the CMD `[CMD-BACKUP-CONFIRM-FLUSH] flushing N B` log
(RAW ⇒ flushed raw == delivered app bytes):

| bsi | CMD frames built | RSP delivered | cumulative delivered |
|-----|------------------|---------------|----------------------|
| 0..10 | 1,1,1,6,6,25,25,30,25,25,25 | (all clean) | **23581** |
| **11** | **30** (`nframes=30 cfg=16`) | **25 frames = 3850 B** (RSP `ACK-GATE PASS 25/25`) | **27431** |
| 12 | 30→25 (post-BREAK cfg15) | 4175 B (ALL CORRUPT) | 31606 |
| 13 | 30→25 | 4175 B (ALL CORRUPT) | 35781 |

`23581 + 25·154 = 27431` = **exactly `integrity_first_bad_offset`**; `4175+4175 = 8350`
= **exactly `integrity_mismatch_bytes`**. So bsi 11 delivered its FIRST 25 frames
correctly and the corruption begins at the point where the CMD's batch (30 frames)
exceeds the RSP's batch (25 frames).

## §3 The DESYNC, proven on both sides (`arq_c3100.log`)

- **CMD built bsi 11 at batch=30:** `T+233.860 [CMD] cmd_batch_tx_start batch=12
  nframes=30 cfg=16` + `[CMD-BATCH-SEQ] new-data batch_seq_id=11 (frames in batch=30)`.
  The CMD's Axis-2 controller had stepped 25→30 (`T+193.791 [POLICY-AXIS2] ... batch=30`)
  and re-issued `T+231.704 [CMD-LINK-PARAMS] SET_LINK_PARAMS TX: batch=30`, then
  `T+233.855 [CMD-LINK-PARAMS-ACKED] round-trip complete (local batch=30) — resuming
  data TX` **and immediately built the 30-frame bsi 11.**
- **RSP delivered bsi 11 at batch=25:** `T+242.428 [RSP] Entering ACK-GATE: rx_count=25
  ... batch=25` → `[ACK-GATE-DIAG] rx=25/25 exp=25 seqs: 0..24` → `[ACK-GATE] PASS:
  received 25/25 (expected 25)` → `[RSP-V2-BATCH-DONE] last_delivered=11` →
  `[RSP-MFSK-SACK] clean path bsi=11 bitmap=0x01ffffff nframes=25`. The RSP was at
  batch=25 the WHOLE cfg16 dwell (`load_configuration(16)` at T+221.453 pins
  radio_batch_size=25, `data-flow-batch-size.md §2.1`).
- **The RSP NEVER applied batch=30 for bsi 11:** the only `[RSP-LINK-PARAMS] APPLIED
  batch 25 -> 30` in the whole log is at `T+176.377` (before the cfg13→15→16 changes
  that reset it to 25). Control-frame tally: CMD `nSent_control=13`, RSP
  `nReceived_control=12` (1 lost). The CMD advanced its LOCAL batch to 30 and built the
  batch on a "round-trip complete" that did NOT guarantee the RSP durably APPLIED 30.

**Consequence chain (§5) — why it becomes silent corruption, not just a stall:**
1. RSP delivers bsi 11 truncated to its own 25 frames (source `[23581,27431)`), treats
   it COMPLETE, and emits a 25-bit clean bitmap `0x01FFFFFF`.
2. Frames 25-29 of bsi 11 (source `[27431,28204)`, ~773 B) exceed the RSP's batch → they
   are never delivered as part of bsi 11 (dropped, later `prev_inactive_late_retransmit`).
3. The CMD's clean-target is `all_ones=(1<<30)-1=0x3FFFFFFF` (`data-flow-batch-size.md
   §3.1`); `0x01FFFFFF ≠ 0x3FFFFFFF` → bsi 11 is NOT credited clean (`nAcked_data`
   stays 170 = batches 0-10 only) → the CMD retransmits all 30 frames (RSP drops them)
   → `[GEARSHIFT] FRAME UP DATA FAILED` → `[BREAK]` cfg16→ROBUST_0→cfg15.
4. The BREAK re-stage (`arq_commander.cc:711-716`, compression-off leg) FREEs bsi 11's
   `messages_tx[]` and re-queues them (push_front into a 99.89%-full `fifo_buffer_tx`, so
   the push mostly FAILS — `fifo_buffer.cc:107` no-ops when `length>get_free_size()`).
   Either way the source stream is now misaligned by the orphaned 773 B: the RSP's app
   position after bsi 11 is 27431, but the CMD has moved its source pointer past frame 29
   (source 28204). **The 773 B `[27431,28204)` are delivered by NEITHER batch** → the
   whole tail shifts → every byte from 27431 is wrong → transfer eventually
   `[RSP-V2-GAP-ABORT]`s at bsi 15.

**Decisive counterfactual:** had the sizes AGREED (both 30 OR both 25), bsi 11 would
deliver as one whole batch, the clean bitmap would match, no retransmit/BREAK, no
orphan, no corruption. The desync is NECESSARY and SUFFICIENT. Everything downstream
(ACK-reject, retransmit, BREAK, re-stage) is the desync's consequence.

## §4 Cross-layer audit (CLAUDE.md §"Cross-Layer Data-Flow Audits")

Shared state: `data_batch_size` (canonical owner `data-flow-batch-size.md`). This audit
EXTENDS that doc with the silent-corruption consequence.

1. **PRODUCERS of `data_batch_size` in play here** (`data-flow-batch-size.md §2`):
   `load_configuration` per-config pin (`arq_common.cc` OFDM branch → radio_batch_size 25)
   — runs on BOTH sides at every config change and RE-SEEDS 25; CMD Axis-2 step-up
   (`policy_evaluate_axis2`, +5 after 4 good batches, `arq_commander.cc`) — CMD-ONLY, then
   mirrored to RSP by SET_LINK_PARAMS (`arq_commander.cc CMD-LINK-PARAMS` →
   `arq_responder.cc §2.6 apply`).
2. **CONSUMERS that DIVERGE under a mismatch:** the clean-batch all-ones target
   (`data-flow-batch-size.md §3.1`, CMD `(1<<batch)-1` vs RSP-emitted bitmap) → ACK never
   credited; the RSP ACK-GATE expected-count (`arq_responder.cc`) → delivers truncated at
   the SMALLER size; CMD block-build loop (`i<data_batch_size`) → packs the LARGER size.
3. **The VIOLATED invariant (`data-flow-batch-size.md §1`):** *"CMD `data_batch_size` ==
   RSP `data_batch_size`."* Held at connect + at the T+176 apply; BROKEN at bsi 11 because
   a config-change reset (both → 25) + a CMD-only Axis-2 re-step (CMD → 30) + an
   unapplied SET_LINK_PARAMS left CMD=30, RSP=25.
4. **The NEW consequence this audit adds:** the doc's §3.1 only foresaw a STALL. The RSP
   ACK-GATE consumer, on a mismatch where CMD>RSP, does NOT stall — it DELIVERS the batch
   truncated to the RSP's size and treats it complete → the sender's surplus frames are
   orphaned → **silent stream shift**, a strictly worse outcome than the stall.
5. **Atomicity gap (the fix target):** the CMD applies its Axis-2 step LOCALLY and builds
   the next batch at the new size on a control-round-trip that does NOT prove the RSP
   durably APPLIED the size before that batch arrives.

## §5 Downstream consequence chain (formerly mis-blamed as root)

The re-stage / BREAK cascade in §3 steps 3-4 is REAL and DID execute (`arq_commander.cc:
711-716` non-compressed re-stage; `restore_backup_buffer_data` `arq_common.cc:14271` is
the compressed twin; Fix C confirm-flush `arq_commander.cc:109-128` cannot fire because
the ACK is never credited). But it is DRIVEN by the desync-induced ACK-reject, not an
independent bug. A fix at the re-stage layer would NOT prevent the corruption because the
orphaned 773 B are already lost the moment the RSP truncates bsi 11 at 25 frames — BEFORE
the BREAK. Fix at the ROOT (§7).

## §6 Ruled OUT
- Streaming PPMd/zstd desync (RAW codec; no `[DECOMPRESS]`/CRC16 errors).
- Mixbatch over-pop split (Finding 2, fixed `c235ff4b`/`e2ba8831`; no decompress fails).
- fifo-backup re-stage double-delivery as ROOT (Fix C `c34259ba`) — it is the CONSEQUENCE,
  not the root (§5).
- PHY / undetected-CRC escape — every DATA frame passed its own CRC; whole-batch shift.
- Connect / contention / ruler — survived those fixes.

## §7 FIX DESIGN (the ROOT fix; integrity D0, LOUD detect + resync)

> **★ SHIPPED STATUS (2026-07-03, branch `fix/batchsize-desync-backstop` off `monitor`
> 1e60ef80):**
> - **(B) LOUD BACKSTOP — SHIPPED + PROVEN.** New pure detector
>   `batchsize_desync_detected(sender_total_frames, local_batch, sack_v2)` (`arq.h`); the
>   RSP ACK-GATE (`arq_responder.cc`, after the `expected` clamp) raises
>   `[RSP-V2-BATCHSIZE-DESYNC]` + `rsp_gap_abort_teardown()` (link DROPPED, no delivery)
>   when `rx_batch_total_frames > data_batch_size`. `MERCURY_BATCHSIZE_DESYNC_DEFEAT=1`
>   reverts on the SAME binary. Deterministic test `test_batchsize_desync_delivery()`
>   (`--test-batchsize-desync`, wired into `--test` + `test_climb_engine` Part D as D6):
>   **fail-before (defeat=1)** reproduces the 25+25 silent shift (`truncated_shift=1
>   faithful=0`, 400 B, link CONNECTED); **pass-after (default)** aborts (0 B silently
>   delivered, link DROPPED); CASE-MATCHED faithful (backstop INERT — no regression);
>   CASE-STEPDOWN safe. `mercury.exe --test` green. This ALONE fully eliminates the silent
>   corruption — the ~3% now aborts LOUDLY (recoverable) instead of silently shifting.
> - **(A) confirmed-before-use step-up — DESIGNED, status below.** (B) is the D0 integrity
>   guarantee regardless of (A); (A) narrows the window so the abort rarely fires. See the
>   §7-A analysis + §10 for the exact ripple (the eager local apply at
>   `policy_evaluate_axis2` is asserted by `test_climb_engine` B3, so deferral requires a
>   paired test update) and why it is gated on climb-regression proof.
> - **(C) harness internal-shift gate — status in §10.** The realaudio harness
>   `byte_integrity_ok` (position-by-position vs a non-periodic canonical pattern,
>   `arq_realaudio.py rx_thread_fn`) already FAILS a shifted delivery (it caught `res_c3100`
>   at offset 27431); the fact-doc concern was that `uniqueness_ok`/`delivered_exceeds_fed`
>   are count-only and blind — the verdict already keys on `byte_integrity_ok` first.

Enforce the `data-flow-batch-size.md §1` invariant at BATCH-BUILD time. Ranked:

**(A) — RECOMMENDED / root / guaranteed: confirmed-before-use for a batch-size step-up.**
The CMD must NOT build a batch at a stepped-UP `data_batch_size` until the RSP has
POSITIVELY APPLIED it. **The current round-trip confirmation is UNRELIABLE** — precise
evidence: the RSP's SET_LINK_PARAMS handler ACKs the control frame ONLY on the apply
(CRC-pass) path (`arq_responder.cc:3678` `set_data_batch_size(target)` → `:3696`
`[RSP-LINK-PARAMS] APPLIED` → `:3704` ACK), yet the run has **NO second `APPLIED batch`
log** (only T+176.377) while the CMD logged `[CMD-LINK-PARAMS-ACKED] round-trip complete`
at T+233.855 and immediately built the 30-frame bsi 11. So the CMD's round-trip fired
WITHOUT the RSP applying 30 — either the CMD matched a NON-SET_LINK_PARAMS control ACK
(false positive: the transition at `arq_commander.cc:7333` does not verify the ACK is
*for* this op), OR the SET_LINK_PARAMS was the 1 lost control frame (CMD `nSent_control=13`
vs RSP `nReceived_control=12`) and the CMD proceeded anyway. The CMD's own comment
(`arq_commander.cc:7326-7328`: *"the ACK confirms RSP also applied"*) encodes the FALSE
assumption. Fix: keep building at the OLD (RSP-known) size until the RSP echoes an
explicit APPLIED-confirmation the CMD VERIFIES is for THIS size change (a dedicated
apply-ack, mirroring the `ROBUST_DWELL_BATCH_OP 0x44` dedicated-op precedent in
`data-flow-batch-size.md §5.1-UPDATE, data-flow-robust-tier-arq-batch.md §10`, NOT the
fire-and-forget SET_LINK_PARAMS with an unverified round-trip). A step-DOWN is always safe
(RSP delivering fewer than sent just SACKs the remainder). This guarantees CMD ≤
RSP-confirmed size at build time, so the RSP never truncates a batch below what the CMD
packed.

**(B) — defense-in-depth LOUD guard (ship WITH A): RSP refuses to silently truncate.**
The RSP's ACK-GATE / delivery must treat *evidence the sender used a larger batch* — a
received frame for the current/prev bsi with `id >= data_batch_size` — as a batch-size
desync → do NOT deliver the batch as complete at the smaller size; raise
`[RSP-V2-BATCHSIZE-DESYNC]` and hold/abort so the mismatch surfaces LOUD and the CMD
resyncs, never a silent orphan. (Timing caveat: the surplus frames may arrive just after
the ACK-GATE fires; combine with a short post-deliver window OR gate the ACK-GATE
completion on "no higher-id frame pending" — detail in the test, §8.)

**(C) — harness blind-spot fix (independent, safe):** the capstone/fleet integrity gate
must detect INTERNAL shift/dup on a PARTIAL transfer (compare `delivered[0:N]` to
`fed[0:N]` AND scan for a repeated/orphaned sub-run), not only `rx_total>tx_total`. Then
this class fails LOUDLY in CI instead of hiding behind `uniqueness_ok=true`.

**Recommended:** (A) as the root guarantee + (B) as the LOUD backstop + (C) for CI.

## §8 DETERMINISTIC in-process test (fail-before / pass-after)

Model on `test_climb_engine` Part D (`data-flow-batch-size.md §6`) which ALREADY drives
the CMD/RSP batch-symmetry state — extend it to the mid-session desync + delivery:
1. Bring a CMD/RSP pair to cfg16; set RSP `data_batch_size=25` (its config default) and
   force the CMD `data_batch_size=30` WITHOUT the RSP applying it (the observed state —
   simulate an unapplied/lost SET_LINK_PARAMS, e.g. env `MERCURY_BATCHSIZE_DESYNC=1`).
2. CMD builds a 30-frame batch of distinct per-slot bytes; feed the RSP.
3. **fail-before (HEAD):** RSP ACK-GATE PASSes at 25/25, delivers 25 frames, orphans
   frames 25-29 → the RX byte-oracle != TX stream (silent shift reproduced), no abort.
4. **pass-after (A):** with confirmed-before-use, the CMD builds at 25 (RSP-known size)
   → both deliver 25 → faithful; OR **(B):** the RSP raises `[RSP-V2-BATCHSIZE-DESYNC]`
   and does NOT silently deliver the truncated batch.

## §9 Cross-layer checklist / where this lives
Update `data-flow-batch-size.md` (the OWNER) with the silent-corruption consequence (§4.4
here) and the fix. Before touching `data_batch_size` producers, the Axis-2 step, the
SET_LINK_PARAMS apply, or the ACK-GATE expected-count, walk §3/§4 here. INVARIANT:
**a batch-size step-UP must be RSP-confirmed-applied before the CMD builds a batch at the
new size; the RSP must never SILENTLY deliver a batch truncated below the sender's size.**

## §10 Open items / next experiment
- **(B) DONE** (see §7 SHIPPED block): built (`bash build.sh o3`), `--test` green, fail-
  before/pass-after proven. A bounded WGN:25 fleet cohort should confirm the ~3% no longer
  SILENTLY fails (a loud `[RSP-V2-BATCHSIZE-DESYNC]` abort + resync, or a clean transfer,
  is a PASS; a silent byte-mismatch is not). If the fleet is busy with the capstone, the
  deterministic in-process proof (D6 / `--test-batchsize-desync`) is the merge gate.
- **(A) DESIGNED, NOT merged — the precise mechanism + why it is gated:** the ROOT eager
  apply is `policy_evaluate_axis2` `arq_commander.cc:8741` `set_data_batch_size(to)` — the
  CMD applies the UP-step LOCALLY the instant the up-move is DECIDED, before any RSP
  confirm (the comment at :8775-8780 admits it proceeds even when the SET_LINK_PARAMS is
  NOT sent, "the local batch size already changed", relying on EOB self-correct). MINIMAL
  correct fix = DEFER the up-step apply: stash `to`, keep building at `from` (the RSP-known
  size), and apply `set_data_batch_size(to)` ONLY in the SET_LINK_PARAMS ACK handler
  (`arq_commander.cc:7323`). Step-DOWN applies immediately (always safe — CMD<RSP only
  STALLS, never corrupts; and (B) does NOT fire on CMD<RSP). TWO reasons it is gated behind
  climb-regression proof rather than shipped in this pass: (1) the eager apply is ASSERTED
  by `test_climb_engine` Part B3 (batch grows on `policy_evaluate_axis2` alone, no ACK) —
  deferral requires a PAIRED honest B3 update (drive the up-move THEN the ACK-confirm); (2)
  the capture's ACTUAL trigger is a FALSE-POSITIVE control-ACK match (the SET_LINK_PARAMS
  was the 1 lost control frame — CMD nSent=13 vs RSP nReceived=12 — yet the CMD logged
  "round-trip complete"), which deferral alone does NOT fix (the CMD would still apply on
  the spurious ACK); fixing THAT needs either a dedicated apply-ack wire op (mirror
  `ROBUST_DWELL_BATCH_OP 0x44`, bigger interop/test surface) or hardened control-ACK
  sequence matching in `process_control_commander` (`arq_commander.cc:6751`). (B) backstops
  ALL residual desyncs LOUDLY, so shipping (B) first is the integrity-correct order.
- [?] Pin the EXACT reason the RSP didn't apply the T+231.7 SET_LINK_PARAMS batch=30
  (lost control frame — 1 of 13 was lost — vs config-reset race vs apply guard). With (B)
  landed this is a fix-verification detail for (A), not an integrity blocker.
- [?] Confirm the same desync corrupts with compression/encryption ON (expected yes; the
  truncation happens at the RSP ACK-GATE, upstream of the codec — and (B) fires there too).

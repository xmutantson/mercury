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
- Implement (A)+(B) with the §8 test; build (`bash build.sh o3`); `mercury.exe --test`
  green; then a bounded WGN:25 fleet cohort confirms the ~3% no longer SILENTLY fails
  (a loud DESYNC-abort + resync, or a clean transfer, is a PASS; a silent byte-mismatch
  is not).
- [?] Pin the EXACT reason the RSP didn't apply the T+231.7 SET_LINK_PARAMS batch=30
  (lost control frame — 1 of 13 was lost — vs config-reset race vs apply guard). (A)
  makes the CMD robust to ALL three, so this is a fix-verification detail, not a blocker.
- [?] Confirm the same desync corrupts with compression/encryption ON (expected yes; the
  truncation happens at the RSP ACK-GATE, upstream of the codec).

## §11 COHORT-2 FAIL — the residual is a NO-FIXED-POINT VALUE transform, NOT a reassembly shift (2026-07-03)

**Status:** the (B) LOUD backstop (`1b99534c`, merged) is REGRESSION-FREE but INSUFFICIENT.
Cohort-2 = 120 real WGN:25 runs of the (B) binary: **4 SILENT byte-corruptions, 0 loud
`[RSP-V2-BATCHSIZE-DESYNC]` aborts, 116 clean** → the dominant real-channel trigger is
**NOT** `rx_batch_total_frames > data_batch_size` (the CMD>RSP size desync of §1-§9). The
res_c3100 root (B) targeted was byte-arithmetic-correct but is a MINORITY mechanism.

Captures preserved (durable): `_research/_batchsize_c2_captures/{cell}/` — full arq_*.log
(2.6-3.9 MB CMD+RSP), bridge, spawn.out, res JSON for the 4 fails:

| cell | box/wave | rx_bytes | first_bad | mismatch | max_cfg | rebuilt bsi | class |
|------|----------|----------|-----------|----------|---------|-------------|-------|
| c11w112 | .11 w1 c12 | 165 | 113 | 52 | ROBUST(cfg0) | bsi18 ×2 | TINY |
| c21w202 | .21 w2 c02 | 15100 | 15047 | 53 | cfg13 | bsi9 ×3 | TINY |
| c21w113 | .21 w1 c13 | 63172 | 37767 | 25405 | cfg16 | bsi28 ×2 | HUGE |
| c31w010 | .31 w0 c10 | 90109 | 45911 | 44198 | cfg16 | bsi16 ×2 | HUGE |

### §11.1 The decisive signature: `mismatch == rx − first_bad` EXACTLY (all 4)
In ALL FOUR cells the corrupt tail runs from `first_bad` to the LAST delivered byte and
**every byte in it differs** (`integrity_mismatch_bytes == rx_bytes − first_bad_offset`
exactly: 52=165−113, 53=15100−15047, 25405=63172−37767, 44198=90109−45911).

I simulated the harness's EXACT chunked-compare logic (`arq_realaudio.py rx_thread_fn`)
against the real SHA-256 keyed-random source (`capstone_arms.random_slice`) for every
candidate corruption. Only a **per-byte NO-FIXED-POINT value map** reproduces the exact
identity; every POSITIONAL error leaves ~1/256 coincidental matches (99.6%, NOT exact):

| corruption model | mismatch over the 44198-B tail | exact identity? |
|---|---|---|
| hole / +154 byte shift | 44004 (99.56%) | NO |
| duplicate / −154 shift | 44004 | NO |
| foreign source region (dup of another batch) | 44045 | NO |
| reorder / positional permutation | ~99.6% | NO |
| keystream-XOR **with** zero bytes (PRBS/scrambler) | 43999 | NO |
| 1-bit shift / nibble-swap / pair-swap / self-diff | 41411–44034 | NO |
| **XOR-const / ADD-const / NOT (no fixed point)** | **44198 (100%)** | **YES** |

**Therefore: `delivered[P] = f(source[P])` where `f` is a per-byte bijection with NO
fixed point, evaluated at the SAME position P.** This DECISIVELY RULES OUT the entire
class Fix-A and (B) targeted (dropped/orphaned/duplicated/reordered/shifted batches — all
positional, all ~99.6%). The delivered frames are at the RIGHT position with the RIGHT
source content, but each byte's VALUE is mangled by a fixed no-fixed-point map.

### §11.2 What f is NOT / what it points to
- Compression OFF (`Force compression: off`, both peers) and encryption OFF
  (`enc_activated=false`, `nonce_enc_count=0`) → no codec/keystream transform in software.
- No PHY scrambler/whitener exists in the tree (`grep scrambl|whiten|prbs|lfsr` → only
  zstd-internal, inert). No `^=`/XOR in the RSP delivery path (`copy_data_to_buffer`,
  `arq_responder.cc`, `arq_common.cc` delivery) — only CRC polynomials.
- A no-fixed-point per-byte map of the CORRECT-position source = the signature of a
  **decode-level coset/value error that PASSES the per-frame CRC** (a CRC-ESCAPE), NOT an
  ARQ reassembly bug. Candidate: a constellation/equalizer value error uniform across the
  frame → `source XOR mask`; a LINEAR CRC (+ systematic LDPC) passes a coset-shifted
  codeword, so the frame is silently ACCEPTED and delivered. (Full 0xFF inversion likely
  fails CRC; the exact mask must be identified from captured bytes — §11.4.)

### §11.3 Common control-flow context (all 4)
Every failing cell has a data batch **BUILT ≥2×** (a BREAK/re-stage REBUILD) and delivers
through the cross-storage PREV path (`[RSP-V2-PREV-DELIVERED]`, `arq_responder.cc:1280`)
with heavy MIXBATCH (`retx bsi=k + new bsi=k+1`). In c31w010 the corruption boundary
`first_bad=45911` lands at the bsi15→bsi16 delivery boundary; bsi16 was rebuilt (2×
`CMD-BATCH-SEQ new-data batch_seq_id=16`, T+294 and T+320) around a `[BREAK] Block failure
#1 at config 16`. The rebuild/re-lock is the TRIGGER that admits the value-corrupted
frames; the value corruption itself is at DECODE, not reassembly.

### §11.4 OPEN (next, before any fix): identify `f` from CAPTURED BYTES
The existing captures are LOGS only (harness compares in-flight, discards bytes). To pin
`f` (constant-XOR mask ⇒ constellation rotation; else a different coset) I must REPRODUCE
with a byte-dumping harness (`rx_thread_fn` saves the delivered stream + expected source on
integrity-fail) and XOR delivered⊕source. Repro: deterministic `-x sim` (WGN + SNR3k via
`tools/sim_channel_relay.py`) preferred (repeatable ⇒ a HEAD-failing unit test); else the
proven real-audio fleet cohort (~3.3%). **Fix direction (task-mandated for a CRC-escape):
an END-TO-END per-batch/block integrity digest (strong CRC-32 over the delivered block,
sender-transmitted, RX-verified) that catches value-corruption → retx; NEVER silent-accept.
NOT a per-frame CRC tweak.**

## §11.CORRECTION (same day) — §11.1's "value transform" was computed against the WRONG source; it IS a reassembly SHIFT

~~§11.1 concluded the corrupt tail is a per-byte NO-FIXED-POINT VALUE transform (⇒ CRC-escape),
because a positional shift of the source left ~1/256 coincidences (99.6%, not the exact 100%).~~
**That test used the SHA-256 keyed-random stream (`capstone_arms.random_slice`). The COHORT-2
harness that actually produced these res JSONs is the OLDER fleet `bsz_tools` build, whose TX +
verifier use `CA.tx_chunk()`/`CA.expected_slice()` = a PURE PERIOD-256 COUNTER `byte[k]=k%256`
(no block index, no SHA — verified from `/dev/shm/bsz_tools.tgz`).**

For a period-256 counter, a positional shift by Δ gives `delivered[P]=(P+Δ)%256` vs
`expected[P]=P%256`, which differ at EVERY P iff `Δ%256 ≠ 0`. Re-running the harness's exact
compare over the period-256 source:

| shift Δ | Δ%256 | mismatch over the 44198-B tail | exact identity? |
|---|---|---|---|
| 154 | 154 | 44198 | **YES** |
| 773 | 5 | 44198 | **YES** |
| 1 | 1 | 44198 | **YES** |
| 256 | 0 | **0 (INVISIBLE)** | n/a |

**So the exact `mismatch==rx−first_bad` identity is FULLY CONSISTENT WITH A POSITIONAL
REASSEMBLY SHIFT of Δ bytes (Δ not a multiple of 256)** — NOT a value transform. §11.1's
transform conclusion is WITHDRAWN. This RE-CONFIRMS the fact-doc §1-§3 class (a dropped/
orphaned/duplicated span ⇒ whole-tail shift), but from a root the (B) `CMD>RSP batch_size`
guard does NOT cover (it never fired in cohort-2).

**Two consequences:**
1. **The period-256 oracle is WEAK** — it is BLIND to any shift/dup/reorder that is a multiple
   of 256 bytes (and to a re-delivered 256-aligned block). The TRUE silent rate is ≥ the
   observed 3.3%. The stronger newer oracle (`canonical_bytes` 24-bit block index +
   `random_slice`) MUST be used for the fix-verification cohort (this is fact-doc §7(C), now
   REQUIRED not optional).
2. The dominant real-channel root is a **reassembly SHIFT tied to the batch REBUILD (every
   failing cell built a data bsi ≥2×) + cross-storage PREV delivery + MIXBATCH** (§11.3),
   NOT the CMD>RSP size desync. Next: get Δ from a byte capture (patched harness re-run) and
   trace the exact dropped/duplicated span at the rebuild boundary (in progress).

## §12 ROOT CONFIRMED (2026-07-03) — re-stage re-queue ORPHANS in-flight bytes → reassembly SHIFT

**Byte capture** (patched harness, fleet re-run): a fresh WGN:25 fail `capA31w013`
(rx=93, first_bad=77, mismatch=16=93−77) captured delivered vs expected:
`DELIV=7d7e7f80…` `EXPEC=4d4e4f50…` → `delivered[77]=0x7d(125)`, `expected[77]=0x4d(77)`
⇒ **Δ=48: `delivered[P]=source[P+48]`** — a 48-byte HOLE, a positional SHIFT. Confirms §11.CORRECTION
(shift, not value transform). (Degenerate ROBUST cell: a ~48-B robust frame orphaned.)

**Root code** — the re-stage re-queue of in-flight (un-ACKed) frames back into `fifo_buffer_tx`:
- `fifo_buffer.cc:85-101 push()` and `:104-127 push_front()` BOTH silently NO-OP (return 0) when
  `length > get_free_size()` (fifo full).
- The DEMOTE / CFG16-HOLD re-stage sites (`arq_commander.cc:2065,2096,3390,4875,5083,5213,5369`)
  re-queue with **`fifo_buffer_tx.push()` (append to BACK)** and **IGNORE the return**, then
  `messages_tx[i].status = FREE` **unconditionally**. Two defects vs the BREAK path
  (`:581,:669`, which uses `push_front`):
  1. **push (BACK) reorders**: it puts the in-flight batch BEHIND any newer app data still in the
     fifo; the re-stage then rolls `cmd_batch_seq_id` back to the in-flight bsi and the next-built
     batch pops the FRONT (= newer source) → labeled with the old bsi → shift by the reordered span.
  2. **silent drop + unconditional FREE**: when the fifo is FULL the push discards the bytes and the
     frame is freed anyway → the in-flight bytes are gone → hole → shift.
- The fifo is **chronically full**: `fifo_buffer_tx_size=128000` (datalink_config.cc:28) < the 262144-B
  transfer; the app is throttled by TCP backpressure so the fifo sits near-full. So on ANY re-stage the
  in-flight batch cannot be re-queued → orphan/reorder. The `[CFG16-HOLD] FIX-9 LOSSLESS DEMOTE`
  comment ("so no bytes are dropped") is FALSE.
- **c31w010 evidence**: `[CFG16-HOLD] FIX-9 LOSSLESS DEMOTE: rolling cmd_batch_seq_id 17 -> 16`
  (T+319.099, arq_commander.cc:5382) fires immediately before the corrupt re-sent bsi16
  (first_bad=45911 ≈ the bsi15/bsi16 delivery boundary). Every failing cell built a data bsi ≥2×
  (a re-stage), matching this.

**Why (B) never fires**: (B) guards `rx_batch_total_frames > data_batch_size` (a CMD>RSP size desync).
This root is a CMD-side re-stage orphan with CMD==RSP batch sizes → (B) is INERT. Different root.

**FIX DIRECTION (task: loud detect + retx / never silent-orphan)**: the re-stage re-queue must be
(a) ORDER-PRESERVING (`push_front`, not `push`) and (b) LOSSLESS — it must not FREE a frame whose
bytes it could not re-queue. Root-preventive: reserve fifo headroom for the max in-flight batch so a
re-stage `push_front` always fits; and/or check the return and LOUD-detect+preserve on a would-be drop.

## §13 FIX (2026-07-03) — order-preserving, lossless re-stage re-queue + ingestion reserve

**Root (from §12):** 7 demote/CFG16-HOLD re-stage sites re-queued in-flight `messages_tx[]`
frames with a FORWARD-iter `fifo_buffer_tx.push()` (append to the BACK, return ignored, frame
freed unconditionally) → the in-flight block landed BEHIND newer app data (reorder) or was
silently dropped when the chronically-full fifo had no room (orphan) → a positional reassembly
SHIFT. The 3 BREAK sites already did it correctly (reverse-iter `push_front`).

**Fix (three parts, all in the CMD TX path):**
1. **`restage_requeue_tx_messages()`** (`arq_common.cc`, new helper) — reverse-iter `push_front`
   so the in-flight block is re-queued CONTIGUOUS + IN-ORDER at the FRONT (ahead of newer app
   data), and CHECK the `push_front` return: any shortfall is surfaced LOUD via `[RESTAGE-ORPHAN]`,
   never a silent orphan. Replaces all 7 open-coded forward-`push()` loops (arq_commander.cc
   2065/2096/3390/4875/5083/5213/5369). The 3 BREAK `push_front` loops are already correct and
   left as-is.
2. **Ingestion reserve** (`arq_common.cc` receive-gate, ~:6393) — gate the app→`fifo_buffer_tx`
   ingestion on `free_size >= MAX_BUFFER_SIZE + in_flight_bytes` (sum of non-FREE `messages_tx`
   lengths), so `free >= in_flight` is an INVARIANT (pop-to-build grows free and in_flight
   equally → the margin holds). Guarantees a re-stage `push_front` ALWAYS fits (no drop) — the
   fix for the fifo-full ORPHAN (e.g. the captured Δ=48 ROBUST cell + c31w010's partial drop),
   which the helper alone cannot prevent. Backpressure to the app is unchanged in kind.
3. **Loud detect** — the helper's `[RESTAGE-ORPHAN]` log makes any residual (reserve ever
   insufficient) VISIBLE, per the task mandate "never silent-accept".

`MERCURY_RESTAGE_ORPHAN_DEFEAT=1` restores the pre-fix behavior (forward `push()`-to-BACK +
unreserved gate) for the fail-before arm.

**Cross-layer audit (fifo_buffer_tx + messages_tx, CLAUDE.md §Cross-Layer):**
- PRODUCERS of fifo_buffer_tx: app ingestion (`receive_arq` :6393, now reserve-gated); demote/BREAK
  re-stage (now `restage_requeue_tx_messages`); demote refill from backup (`arq_common.cc:3232`).
- CONSUMERS: batch build pop (`process_buffer_data_commander`). INVARIANT restored: bytes popped to
  build a batch are ALWAYS re-queueable losslessly + in order on a re-stage (`free >= in_flight`),
  so the re-sent batch (after the `cmd_batch_seq_id` rollback) carries the ORIGINAL contiguous
  source, not shifted. No consumer assumes a fill level beyond the (tightened) ingestion gate.

**Test (`test_restage_requeue.cc`, `--test-restage-requeue-orphan`):** stages newer app data D in
the fifo + an in-flight block B in messages_tx, calls the production helper, asserts the re-queued
stream == B++D byte-exact.
- **FAIL-BEFORE** (`MERCURY_RESTAGE_ORPHAN_DEFEAT=1`): `requeued=4000 order_ok=0 first_bad=0` →
  FAIL (rc=1) — the in-flight block landed behind D (reorder = the observed shift).
- **PASS-AFTER** (default): `order_ok=1` → PASS (rc=0) — contiguous, in order, zero loss.
Build `build.sh o3` EXIT=0; `mercury.exe --test` green (see §13 verify). Deterministic, no RF.

**Fix-vs-(B):** (B) (`monitor 1b99534c`, MERGED) is a LOUD backstop for a DIFFERENT root (CMD>RSP
size desync) and stays. This §13 fix targets the DOMINANT real-channel root (the re-stage orphan/
reorder shift) that (B) never covered. Built/validated on `staging/enc-bsi-nonce@683651fe`;
the 7 buggy loops are almost certainly identical on `monitor` → cherry-pick there.

## §14 The 4th mechanism + the systemic pivot (recap — full analysis in the main-tree version + worklog)

On the fixed-`monitor@bfacbcf6` binary a 4th DISTINCT silent mechanism survived (`c31w104`,
WGN:25): a FORWARD positional shift Δ=+6847 where the RSP declared bsi10/bsi11 COMPLETE at
SHRUNK expected-counts (delivered 21-23 of 25 frames) → silent tail-frame holes → later
content re-anchored at an earlier stream slot; the loud GAP-ABORT fired ~1.6 batches LATE.
**The common thread across ALL 4 mechanisms:** each existing guard checks a PROXY invariant
(re-queue completeness / batch-SIZE equality / bsi-contiguity / EOB frame-count) — NONE checks
the invariant that actually matters: **delivered app/transported bytes == source bytes by
ABSOLUTE offset, in order.** Fable-5 (`wuj3xqvod`, worklog 2026-07-03) steered: go straight to
**Option W** (the TCP-style absolute-byte-stream stamp + delivery check), skip the narrow
Option N (a proxy-based duplicate of W's weaker half). MERGE = owner fork (wire change).

## §15 Option W — SHIPPED so far (branch `fix/stream-offset-w` off `monitor@bfacbcf6`)

**FOUNDATION landed + committed (`dd382db8`):** the absolute-byte-stream cursors that make the
Option-W invariant CHECKABLE — Fable's pre-cohort ground-truth gate. See the full
producer/consumer audit in `data-flow-stream-offset.md`.
- `tx_stream_committed` + `tx_stream_stamp[256]{start,length,valid}` latched at the ONE pop
  funnel `process_buffer_data_commander()` (transported-byte domain); rolled back to the
  in-flight batch's start at every re-stage byte-restore point (`stream_tx_rollback_inflight`
  in `restage_requeue_tx_messages`, `restore_tx_from_compressed`, + the two open-coded BREAK
  legs) so a rebuild re-anchors at the SAME start (INV4).
- `rx_stream_delivered` advanced by the reassembled transported length at the ONE receiver
  funnel `copy_data_to_buffer()`. Both cursors reset at `reset_session_state()` + ctor.
- Regression `--test-stream-offset` (`test_stream_offset.cc`): drives the PRODUCTION
  latch/rollback/receiver funnels through sequential builds, retx re-emit, re-stage rollback
  + rebuild, BREAK→ROBUST→re-climb, mixbatch, RX delivery — 34/34 checks PASS, asserting
  `stamp[bsi].start == the true cumulative transported origin` (INV1-INV4).
- `build.sh o3` EXIT=0; `mercury.exe --test` EXIT=0 (fully green, 403 PASS summaries);
  `--test-restage-requeue-orphan` still PASS (rollback-in-funnel is a guarded no-op on
  unlatched stamps → no regression).

**NOT yet shipped (STEP 2 CORE W + STEP 3 add-ons):** the WIRE stamp + the two RSP checks
(PRIMARY byte-gate the clean ACK; BACKSTOP start==rx_delivered LOUD) + the 4-mechanism test.

**CORE W SHIPPED (2026-07-03, branch `fix/stream-offset-w`):** STEP 2 landed + committed +
green — the 4 silent-corruption mechanisms are now GATED/LOUD, never silently delivered.
- **STEP 2a `fc94ad43`** — the wire stamp `{start_lo32:u32, length16:u16}` on the EOB frame,
  mirroring D5 plumbing. Build RESERVES `W_EOB_RESERVE`(7) payload bytes on the EOB frame
  (raw + compressed legs) so header+payload+stamp == the codeword C exactly (no overflow).
  Gated by `w_stamp_rides()` (deterministic on both peers). ~0.2% overhead (6 B on 1 of ~25
  frames/batch). See `data-flow-stream-offset.md §3` (SHIPPED consumers) + §8.
- **STEP 2b `0d72b63f`** — the two RSP checks: PRIMARY byte-gate (delivered bytes < committed
  stamp.length → WITHHOLD the clean ACK → SACK re-requests) makes mechanism 1/4 impossible;
  BACKSTOP (wire stamp.start != rx_stream_delivered → LOUD `[RSP-V2-STREAM-SHIFT]` + teardown)
  catches any positional shift (mech 3/4) at the FIRST divergent byte. Both no-op on an absent
  stamp. Fail-before env arms `MERCURY_W_BYTEGATE_DEFEAT` / `MERCURY_W_STREAM_SHIFT_DEFEAT`.
- **STEP 2c `996a2f8f`** — the 4-mechanism deterministic GATE in `--test-stream-offset`
  (Parts G/H, 47 checks): each mechanism reproduced in-process via the SHARED production
  predicates `w_stream_shift_detected()` / `w_bytegate_shortfall()`; pass-after gates all with
  0 silent bytes; each fail-before env arm SILENTLY delivers the shift (mech-1/4 short 3000/
  3150 B < 3750; mech-2 3750 < 4500; mech-3/4 positional 3750 B delivered past the hole).
- Build `build.sh o3` EXIT=0; `mercury.exe --test` EXIT=0 (68/0 + Winlink 12/0);
  `--test-stream-offset` 47/47; `--test-restage-requeue-orphan` PASS. No false-fire on healthy
  loopback traffic.
- **STEP 3 add-ons DEFERRED** (running CRC-32 co-stamp for same-position value corruption; EOT
  total-committed-bytes check for the last-batch tail-drop blind spot) — turn-key spec in
  `data-flow-stream-offset.md §8.6`; both touch the riskiest subsystems (build path / session
  teardown), out of scope for the correctness-first core.
- MERGE = owner fork (wire change). Ready for a strong-oracle WGN:25 cohort + owner decision.
STOPPED CLEAN at the FOUNDATION checkpoint (budget) rather than leave a half-applied wire
change — per the task rule. The exact turn-key spec (byte layout, the ~8 D5-mirror sites, the
two check sites, retx re-emit, robust-config handling, the 4-mechanism test plan) is
`data-flow-stream-offset.md §8`. **Until STEP 2 lands + a strong-oracle WGN:25 cohort shows 0
silent fails, the real-channel silent corruption at marginal SNR is STILL OPEN.** Option W is
NOT complete; the FOUNDATION alone detects nothing on the wire (it is the measurement
substrate, not the guard).

## §16 OPUS PRE-COHORT FIX PASS — the two blockers + two follow-ups closed (2026-07-03)

Acting on Fable's §17 merge review + a from-scratch re-audit (task-mandated: "do not trust
the prior audit; enumerate all legs"). All file:line on `fix/stream-offset-w` at the fix HEAD.

### §16.1 From-scratch re-audit — every site that re-queues in-flight `messages_tx[]` to `fifo_buffer_tx`
The Option-W BACKSTOP false-fires only on a SAME-bsi in-order rebuild whose stamp.start is
wrong. That class = the legs that free the in-flight `messages_tx[]` and re-queue its bytes for
a rebuild at the SAME bsi (roll_back_cmd_bsi_to_inflight / BREAK). Enumerated ALL re-queue sites:

| # | site | leg | rolls cmd_bsi to in-flight? | calls `stream_tx_rollback_inflight()`? | verdict |
|---|------|-----|------------------------------|-----------------------------------------|---------|
| 1 | arq_commander.cc:715 (loop :719) | BREAK phase-1 raw | via BREAK→ROBUST | **YES** (:715) | OK |
| 2 | arq_commander.cc:807 | BREAK-EXHAUSTED raw | via BREAK→ROBUST | **NO → FIXED** | **F1/F4.1** |
| 3 | arq_commander.cc:6241 | GEARSHIFT FRAME-UP raw | YES (:6229) | **NO → FIXED** | **F1/F4.1** |
| 4 | Funnel 1 `restage_requeue_tx_messages()` (arq_common.cc:14088) | 7 demote/CFG16-HOLD sites (2204,2230,3519,5047,5250,5375,5526) | 4 sites roll | **YES** (:14093) | OK |
| 5 | Funnel 2 `restore_tx_from_compressed()` (arq_common.cc:14502) | compressed twin (707,800,3515,5043,5371,5522,6234) | — | **YES** (:14508) | OK |
| 6 | `restore_backup_buffer_data()` (arq_common.cc:14616) | inside Funnel 2 + direct at load_configuration:2185 (FULL) | — | via Funnel 2 (covered); FULL = connect/reset | OK / out-of-scope |

**Distinct class — restore-from-BACKUP legs (FRESH-bsi, do NOT call roll_back_cmd_bsi_to_inflight):**

| site | leg | in cohort? | note |
|------|-----|-----------|------|
| arq_common.cc:6106 | watchdog recovery (COMMANDER) | rare (60s+ silence) | residual |
| arq_common.cc:6194 | SNR_BASED gearshift-down | NO (default=SUCCESS_BASED_LADDER) | residual |
| arq_common.cc:6288 | SUCCESS_BASED gearshift-down (timer) | reachable | residual |
| arq_common.cc:7501 | SET_CONFIG ACK-apply refill (forward push) | config-apply | residual |
| arq_common.cc:3305 | inband_unilateral_config_change refill | in-band path | residual |

These free `messages_tx[]` then restore the in-flight batch's plaintext from `fifo_buffer_backup`
WITHOUT rolling `cmd_batch_seq_id` back → the rebuild takes a FRESH (advanced) bsi → the RECEIVER
sees a bsi GAP the pre-existing v2 gap-gate (`sack_v2_readopt_has_gap`) holds on BEFORE
`copy_data_to_buffer` (where the byte-position BACKSTOP lives) is ever reached. So they are NOT
the same-bsi in-order false-fire class the BACKSTOP checks; they are a pre-existing (non-Option-W)
fresh-bsi/gap behavior. Left as a documented **residual hardening candidate** (adding the guarded
idempotent `stream_tx_rollback_inflight()` before their free-loop is safe and would only help, but
touches legacy watchdog/timer/inband paths and is out of the named cohort-blocker scope).

**Not re-stage legs (confirmed OUT):** arq_commander.cc:20331/20340/20384 = pre-latch staging
push-BACK of EXCESS popped bytes (before the latch — cursor not yet advanced); :18152/:18210 =
in-C `sim2` livepath harness (not production; unused by the real-audio cohort); :15394/15534/
15710/15825, :17809 = test harness. arq_common.cc:14591/14607/14637 = inside Funnel 2 /
restore_backup (covered by Funnel 2's rollback).

**Conclusion:** the ONLY same-bsi Option-W re-stage legs missing the rollback are **:807 and
:6241** — confirming Fable §17.4. `data-flow-stream-offset.md §2.2` misclassified these three raw
legs (:715/:807/:6241) as `restage_requeue_tx_messages()` funnel pairs; they are OPEN-CODED
push_front loops. Corrected there.

### §16.2 Fixes applied
- **F1/F4.1 (blocker):** added `stream_tx_rollback_inflight()` at the two open-coded raw legs
  (arq_commander.cc:807 BREAK-EXHAUSTED, :6241 GEARSHIFT FRAME-UP), byte-mirroring the correct
  :715. `stream_tx_rollback_inflight()` gained a `MERCURY_W_RESTAGE_ROLLBACK_DEFEAT` fail-before
  arm (no-op) so the production funnel the test drives faithfully reproduces the pre-fix leg.
- **F4.2 (blocker):** new `rx_stream_invalidate_stamps()` (arq_common.cc), called from
  `load_configuration()` right after the no-change early-return, invalidates every
  `rx_stream_stamp[]` on a real config change. Kills the demote-to-ROBUST permanent WITHHOLD (a
  stale OFDM stamp robust frames can never refresh), the config-change stale variant, and the
  256-wrap stale-start false teardown. Both W predicates already no-op on an invalid stamp
  (verified: `w_bytegate_shortfall` :14194, `w_stream_shift_detected` :14209), so the gate at
  arq_responder.cc:2506 does NOT fire on an absent/invalidated stamp. Fail-before:
  `MERCURY_W_CFG_STAMP_KEEP`.
- **F2 (follow-up — decided with evidence):** SOFTENED the byte-gate withhold comment
  (arq_responder.cc) to state the TRUE recovery. Sending a SACK on the shortfall was REJECTED: the
  batch is frame-count complete (every slot in `[0,data_batch_size)` present), so a SACK bitmap
  over that window is ALL-ONES → would FALSE-CREDIT the batch (the exact silent accept the gate
  prevents); the short/surplus bytes live OUTSIDE the RSP window (the CMD>RSP surplus frames have
  id ≥ data_batch_size; a tail-short frame is still "present"). Recovery is the CMD ACK-timeout
  full retx → rebuild (fresh EOB stamp; a demote also invalidates via F4.2) → completes, OR escalates
  to BREAK (LOUD, bounded). Outcome = COMPLETE-or-LOUD, never silent, never a permanent hang.
  Proven by Part S.
- **F3 (latent):** the single-frame `send()` DATA path (arq_common.cc:8713) — dead for DATA today
  (all DATA goes through `send_batch`) — now emits the W stamp on an EOB DATA frame (mirroring
  send_batch:9132) + relaxed its header guard by `w_stamp_added`, so a FUTURE DATA caller's EOB
  frame is wire-consistent and the RX's `w_parse_eob_stamp` cannot mis-read payload as a stamp.

### §16.3 Test hole closed (why the blockers slipped past 55/55)
The deterministic suite drove Funnel 1 directly but NEVER a missing-rollback open-coded leg nor a
config-change stamp-invalidation. Added to `test_stream_offset.cc`:
- **Part R** (F1/F4.1): an uncompressed FRAME-UP climb stages an in-flight batch at origin S,
  drives the PRODUCTION re-stage, rebuilds at a HIGHER config, asserts rebuilt `stamp.start == S`
  (INV4) AND `w_stream_shift_detected()` FALSE (no BACKSTOP teardown). FAIL-BEFORE via
  `MERCURY_W_RESTAGE_ROLLBACK_DEFEAT=1` (cursor stays S+L → rebuilt start S+L → shift detected → rc=1).
- **Part S** (F4.2/F2): a demote-to-ROBUST same-bsi rebuild — a stale OFDM stamp WOULD withhold;
  drives the PRODUCTION `rx_stream_invalidate_stamps()`; asserts the stamp is invalidated AND the
  withhold CLEARS (no permanent stall / BREAK spiral). FAIL-BEFORE via `MERCURY_W_CFG_STAMP_KEEP=1`.

Both drive PRODUCTION helpers (not hand-set result stamps). Deferred STEP-3 (EOT + running CRC-32)
unchanged — §17.2/§17.3 accept it for this merge.

## §17 FABLE-5 MERGE REVIEW of CORE W (2026-07-03, read-only; §16 = the Opus pre-cohort fix pass above)

Reviewed `fix/stream-offset-w` @ `9685f6ad` (diff vs `monitor@bfacbcf6`) against the corrected
steer (worklog 2026-07-03 07:0x; §14.7). All file:line below are on the branch.

### §17.1 F1 — FIDELITY: faithful, with ONE coverage gap (the gap is the blocker, §17.4)
- **(a) ACK byte-GATED at the batch-complete boundary — YES.** The PRIMARY check sits
  immediately after `[ACK-GATE] PASS` (arq_responder.cc:2491-2534), BEFORE the clean ACK is
  emitted and BEFORE delivery; a shortfall WITHHOLDS (re-arm identical to the partial-batch
  SACK-suppress hold, `return`) — not next-batch detection. The "one boundary too late"
  correction is realized. NOTE: the cross-storage PREV completion (arq_responder.cc:1307-1321,
  mechanism-4's site) is NOT byte-gated — a short PREV delivery passes the start-check
  (position correct), advances `rx_stream_delivered` short, and the NEXT batch's BACKSTOP
  refuses the shifted delivery (arq_common.cc:14235-14262) BEFORE any corrupt byte reaches the
  app. Outcome for mech-4 via PREV = LOUD teardown at the first divergent byte, not withhold-
  then-SACK recovery. Never silent; acceptable; a PREV-side byte-gate is a worthwhile follow-up.
- **(b) Stamp from committed pop-funnel bytes ONLY, re-sent latched — YES.** Latch input =
  `comp_size` (arq_commander.cc:20579) / sum of `data_read_size` (:20668), latched once at the
  ONE funnel (:20687-20688); `send_batch` emits the LATCHED `tx_stream_stamp[bsi]` verbatim
  keyed on the frame's ORIGINAL `batch_seq_id` (arq_common.cc:9139-9152 → `w_emit_eob_stamp`
  :14162), incl. on retx/mixbatch — the stamp on retx is even STRONGER than D5 (D5 emits 0 on
  `sack_retransmit_active`, arq_common.cc:9041; the stamp re-emits the known latched value).
  RSP compares only to `stamp.length` / `stamp.start` (arq_common.cc:14186-14212), never to
  `data_batch_size`/expected-counts. Wire arithmetic verified symmetric: W_EOB_RESERVE 7 =
  6 stamp + 1 DATA_SHORT-vs-LONG header delta; RX clamp `max_frame-7` (arq_common.cc:13244)
  == TX reserved payload (arq_commander.cc:20656-20661, 20556-20561).
- **(c) ONE write site — YES; ONE rollback helper — YES; rollback COVERAGE — NO (2 sites
  missed).** See §17.4. The audit doc's claim that all 10 re-stage callers route through the
  two funnels (data-flow-stream-offset.md §2.2) is FALSE for two raw legs, and the worklog
  claim "rolled back at ... + the 2 BREAK legs" is only true for leg 1.

### §17.2 F2 — the deferred EOT: NOT a merge-blocker; here is the precise residual
The steer's last-batch worry was against a BACKSTOP-only design (inherently one-batch-behind).
The corrected PRIMARY fires at the final batch's OWN completion, so CORE W DOES catch a
truncated final batch **whenever the stamp arrived**: the EOB frame received on any
(re)transmission carries it (retx re-emits the latched stamp, §17.1b), and with D5 riding
EVERY data frame (arq_common.cc:9030-9048) the count machinery cannot silently under-expect
once ANY frame of the batch arrived — an incomplete count never PASSes the gate, so SACK
re-requests until the EOB (and its stamp) lands. The remaining SILENT final-batch hole
therefore requires ALL of: the EOB frame lost on every attempt, AND a count-machinery bug
(a genuine 5th mechanism) that declares complete anyway, AND it being the final batch. That
is a narrow defense-in-depth residual, and the strong-oracle cohort (delivered-vs-source
byte compare) measures exactly it. EOT remains the ONLY true end-to-end check (it also
covers "final batch never seen at all + clean-disconnect-believed-complete", which per-batch
stamps can never see) — it must land as the scheduled STEP-3 follow-up before the capstone
integrity claim, but deferring it past THIS merge is sound.

### §17.3 F3 — the deferred CRC-32: accepted residual, not a blocker
Same-position value corruption must false-pass LDPC AND the per-frame CRC16_MODBUS_RTU
(telecom_system.cc:674-685, 3300-3304; ~2^-16 per corrupted codeword) at the right offset and
length; AEAD covers it entirely when encryption is on. All four CAPTURED mechanisms were
positional (the c31w104 byte-decode ruled value-transform OUT, §14.1); no value-corruption
mechanism has ever been observed. The CRC-32 co-stamp (and the streaming-decompressor-desync
class it alone covers, data-flow-stream-offset.md §0) is correct as STEP-3 follow-up.

### §17.4 F4 / MERGE-BLOCKER — TX-cursor rollback misses TWO open-coded raw-leg re-stage sites
`stream_tx_rollback_inflight()` is called in funnel 1 (arq_common.cc:14093), funnel 2
(:14508), and inline at the BREAK phase-1 raw leg (arq_commander.cc:715). Two raw
(non-compressed) re-stage legs re-queue in-flight bytes + free `messages_tx[]` WITHOUT it:
1. **GEARSHIFT FRAME-UP climb** — arq_commander.cc:6237-6247 (push_front :6241). The audit
   misclassified this site as a funnel pair ("6229"); its else-leg is open-coded.
2. **BREAK-EXHAUSTED recovery** — arq_commander.cc:803-812 (push_front :807). The audit's
   O1 named both BREAK legs; only the first (:715) got the call.
**Failure scenario (deterministic, healthy transfer):** uncompressed session at a
stamp-riding config; in-flight batch K {start=S, len=L} latched; FRAME-UP fires with K staged
(the exact state `roll_back_cmd_bsi_to_inflight("GEARSHIFT")` :6229 exists to handle) → bytes
re-queued, cursor NOT rolled back (stays S+L) → rebuild latches stamp[K]={S+L, L2} → RSP
(delivered through S) sees `stamp.start(S+L) != rx_stream_delivered(S)` →
`[RSP-V2-STREAM-SHIFT]` **FALSE TEARDOWN of a byte-correct session on an ordinary
mid-transfer climb**. Same shape for BREAK-EXHAUSTED. Loud, not silent — but it kills
completion-rate on the exact cohort configuration (compression OFF) and would poison the
cohort before it measures anything. The deterministic suite passes because
`--test-stream-offset` drives funnel 1 directly, never these two open-coded legs.
**Fix (small, in-design):** call `stream_tx_rollback_inflight()` at the top of both legs
(mirror :715) — or better, route both through `restage_requeue_tx_messages()`; correct
data-flow-stream-offset.md §2.2; extend the test to drive an open-coded-leg re-stage
(latch → push_front loop → assert cursor == stamp.start).

### §17.5 F4 second finding — RSP stale-stamp on config change (fix before cohort)
`rx_stream_stamp[bsi]` is consumed only at DELIVERY (arq_common.cc:14497-14498) and reset
only at session reset. A stamp parsed for a batch that is then RE-STAGED sender-side
(demote/BREAK) goes STALE while valid:
- Rebuild at a LOWER OFDM config, same bsi (the 4 demote sites roll `cmd_batch_seq_id`
  back): rebuilt length L2 < stale L1 → byte-gate withholds until the rebuilt EOB's fresh
  stamp overwrites → transient, self-healing (retx re-carries the stamp). Acceptable.
- **Rebuild at ROBUST, same bsi: robust frames carry NO stamp (`w_stamp_rides()` false), so
  the stale OFDM-sized `stamp[K].length` can NEVER refresh** — the byte-gate runs at robust
  too (the gate at arq_responder.cc:2506 checks only `sack_v2_enabled`, not
  `w_stamp_rides()`; process_messages_acknowledging_data serves the MFSK/robust ACK path) →
  permanent WITHHOLD → ACK-exhaust → BREAK spiral → link failure. A LOUD STALL of the exact
  demote-to-robust recovery the modem needs at marginal SNR. Precondition (EOB received but
  batch incomplete, then demote-to-robust) is common at WGN:25.
- 256-wrap variant: a never-delivered stale stamp re-read at bsi reuse with a lost EOB →
  stale-start false teardown.
**Fix (3 lines):** invalidate all `rx_stream_stamp[]` in `load_configuration()` (RSP side, on
any config change) — every config change re-stages sender-side, so any parsed-undelivered
stamp is stale by construction. Kills all three variants. Add a targeted check.

### §17.6 Minor residuals (note, don't block)
- `w_emit_eob_stamp` emits {0,0} if `tx_stream_stamp[bsi]` is somehow unlatched
  (arq_common.cc:14169-14175): length16=0 keeps the byte-gate inert but a parsed start=0
  could false-fire the backstop. Judged unreachable (every new-data batch latches at the
  funnel); documented residual.
- A `w_stamp_rides()` flip between build (reserve) and send (emit) would overflow C by <=6
  bytes → PHY-CRC fail → retx; same exposure class as the existing D5/`header_carries_d5`
  flip, managed by the same re-stage-on-config-change discipline. Pre-existing class.
- Withhold recover-vs-stall: the withhold re-arm is byte-identical to the proven
  partial-batch hold; the CMD sees "clean ACK lost", an existing recoverable state; worst
  case ACK-exhaust → BREAK (loud). Cohort tracks RECOVERED vs STALLED as designed.

### §17.7 VERDICT
**Option W CORE as implemented is MERGE-BLOCKED by §17.4** (two missing
`stream_tx_rollback_inflight()` calls ⇒ deterministic false stream-shift teardown on
mid-transfer climb / BREAK-EXHAUSTED with in-flight bytes, uncompressed sessions), **and
NEEDS §17.5** (RSP stamp invalidation on config change) **before the cohort** — else the
cohort burns compute measuring these two false-fire modes instead of the silent-corruption
rate. Both fixes are a handful of lines INSIDE the shipped design (no wire change, no new
mechanism). Everything else is faithful to the corrected steer — the invariant, the
boundary, the latched-verbatim stamp, the funnel discipline, the false-fire rules, and the
test contract are right. **Deferring STEP 3 is ACCEPTABLE for this merge** (§17.2, §17.3):
CORE W's PRIMARY covers the final batch whenever the stamp/EOB arrived; EOT + CRC-32 land as
the scheduled follow-up and EOT must be in before any capstone end-to-end integrity claim.
After the two fixes + green suite + the strong-oracle WGN:25 cohort (0 silent, no
false-fire, withholds recover): present the owner merge fork.

## §18 PREV-GATE SHIPPED — the F1(a) cross-storage byte-gate follow-up (2026-07-03)

Fable's §17.1(a) flagged the cross-storage PREV completion as NOT byte-gated and named a
"PREV-side byte-gate = worthwhile follow-up." The pre-cohort capture `res_c11w006` (WGN:25,
seed 7) is the exemplar: `byte_integrity_ok=false, integrity_mismatch_bytes=12,
integrity_first_bad_offset=289` — a silent 12-byte corruption (`DELIV c9785fc3…` vs
`EXPEC da6e7aa9…`). This clip ships that gate.

**The mechanism (from `_research/_c11w006_capture/arq_c11w006.log`):**
- The session STALLED at robust/cfg0 (`max_config_reached=102`, `wb_configs_seen=[0]`) — it
  never climbed past cfg0.
- The leaking delivery = bsi=16 at **cfg0** (`[FRAME-NV] cfg=0`, `[OFDM-OK] t0 cfg=0`): a
  6-frame batch (`[RSP-V2-PREV-RX] bsi=16 ... seq=1/6`) declared complete at a SHRUNK count of
  4 (`prev_received=4/4`, SACK `nframes=6`) — mechanism-4 (complete-at-shrunk-count). It
  delivered a 12-byte short/shifted tail at stream offset 289 via the cross-storage PREV path
  (`[RSP-V2-PREV-DELIVERED] prev_batch_seq_id=16 ... last_delivered=16`), UN-gated.
- The stamp-INDEPENDENT GAP-ABORT caught the LATER hole at bsi=18
  (`[RSP-V2-GAP-ABORT] ... bsi=18 non-contiguous with last_delivered=16`) — AFTER the 12-byte
  corruption. So Fable's §17.1(a) "the next BACKSTOP refuses before any corrupt byte reaches the
  app; never silent" was INCOMPLETE: the short PREV delivery ITSELF is the leak, and the
  next-batch positional guard only stops the FOLLOWING batch.

**The fix (arq_responder.cc:1287-1328):** BEFORE the PREV swap→deliver body, gate on
`w_bytegate_shortfall(rsp_prev_batch_seq_id, messages_rx_prev)` — a new array-parameterized
overload of the primary predicate (arq_common.cc:14255) that reconciles DELIVERED bytes over
the SAME window `copy_data_to_buffer` will deliver from `messages_rx_prev[]`. On a shortfall
WITHHOLD: no swap/deliver, no cursor advance, no clean ACK; leave `messages_rx_prev[]` RECEIVED
+ `rsp_prev_batch_active` so the CMD's ACK-timeout retransmits the tail (COMPLETE-or-LOUD,
identical discipline to the primary gate). FALSE-FIRE safe: absent/invalid stamp ⇒ no-op; a
legit full PREV (delivered==committed) does not trip it. `MERCURY_W_PREV_BYTEGATE_DEFEAT=1` =
fail-before. Full audit: data-flow-stream-offset.md §9.

**Test (`--test-stream-offset` Part T):** drives the PRODUCTION predicate + the PRODUCTION
`copy_data_to_buffer` reassembler (the test_batchsize_desync_delivery contract), grounded on
`res_c11w006` (offset 289 / 12 bytes). Verified on the o3 binary:
- pass-after (default): `T(fix): short PREV withheld → 0 leaked bytes ... got=0 want=0`.
- fail-before (`MERCURY_W_PREV_BYTEGATE_DEFEAT=1`): `T(defeat): un-gated short PREV leaked its
  12-byte tail ... got=12 want=12`.
- `T0` (complete PREV) + `T0b` (absent stamp) assert no false-fire; the full suite is green.

**HONEST SCOPE — the c11w006 caveat (data-flow-stream-offset.md O5/O6):** the Option-W stamp
does NOT ride at cfg0 (per-frame payload ~4 B < `W_STAMP_MIN_MAXFRAME`=15) or at robust
(`header_carries_d5 = !is_robust`, arq_common.cc:2258). So this stamp-based PREV gate — LIKE
the primary gate and the BACKSTOP — is STRUCTURALLY INERT at robust/cfg0. It closes the
PREV-path asymmetry for STAMP-RIDING configs (cfg13-16, where all 4 captured mechanisms occur);
it does **NOT** close the c11w006 cell itself (a robust/cfg0 short-PREV, bounded only by the
late stamp-independent GAP-ABORT). Closing the robust/cfg0 short-PREV is a DISTINCT residual
(needs a stamp-independent reference — mechanism-4 shrinks the frame-count reference, so not a
trivial reuse) and is out of Option W's scope. NET: the PREV-path gap is closed where stamps
ride; the c11w006 cell is not.

## §19 EOT SHIPPED — the end-to-end last-batch tail-drop + running-CRC check (2026-07-03)

Option W's per-batch checks (PRIMARY byte-gate / BACKSTOP / PREV-gate) are inherently ONE BATCH
BEHIND: a truncated FINAL batch has no next-batch stamp to backstop it. This clip lands the EOT
(end-of-transfer) check — the ONLY true end-to-end catch — on branch `feat/eot-turnkey` (off
`monitor@9f0d50e1`, where Option W CORE + PREV-gate are merged).

**The fix (data-flow-stream-offset.md §8.6 SHIPPED / §10):** at a graceful disconnect the sender's
CLOSE_CONNECTION frame carries `[ committed:u64 ][ crc32:u32 ][ crc8:u8 ]` (13 B after `data[0]`,
frame length 14) — wired into the EXISTING CLOSE control code (NO new handshake, honouring the
teardown-race constraint). The receiver, on RECEIVING CLOSE and BEFORE `reset_session_state()`
zeroes its cursors, asserts `rx_stream_delivered == committed AND rx_stream_crc == crc32`; mismatch
→ LOUD `[RSP-V2-EOT-SHORT]` (the transfer is NOT declared complete — never a silent short/wrong
final delivery). A running stream CRC-32 (`tx_stream_crc` / `rx_stream_crc`) folds every committed /
delivered transported byte at the config-independent funnels, with a per-bsi rollback anchor
(`StreamStamp.crc`) restored in lockstep with `.start` at every re-stage — so it survives the
demote/BREAK re-stage machinery (INV4 for the CRC). The counter catches a SHORT tail; the CRC ALSO
catches a same-LENGTH final-content corruption the counter is blind to.

**Coverage vs the per-batch gates — EOT covers robust/cfg0.** Both accumulators fold at the ONE build
funnel / `copy_data_to_buffer`, NOT gated by `w_stamp_rides()`, so — UNLIKE the stamp-based PRIMARY /
BACKSTOP / PREV gates (structurally inert at robust/cfg0, §18 O5/O6) — the EOT counter + CRC are
maintained END-TO-END at ALL configs. A robust/cfg0 tail-drop that reaches a graceful disconnect is
now caught (the c11w006 *shape* at end-of-transfer). NOTE: c11w006 itself GAP-ABORTed mid-stream so
it never reached a CLOSE — EOT is the graceful-close catch, not a mid-stream one; the mid-stream
robust/cfg0 short-PREV (§18 O6) remains a distinct residual.

**Deterministic test (`--test-stream-offset` Part U):** drives the PRODUCTION receiver fold
(`copy_data_to_buffer` advances `rx_stream_delivered` + folds `rx_stream_crc`) + the PRODUCTION
predicate `w_eot_mismatch()` on the mechanism-1/Fix-A shape (a final batch whose tail frame is
dropped on delivery). Verified on the o3 binary:
- **pass-after (default):** U0 complete-transfer no-false-fire; U0b no-data no-op; U tail-drop trips
  (delivered 300 < committed 400); U2 same-length content corruption caught by the CRC (counter
  blind); `U: real tail-drop declared INCOMPLETE` GREEN. `--test-stream-offset` EXIT=0.
- **fail-before (`MERCURY_W_EOT_DEFEAT=1`):** the control-flow CHECK turns RED (EXIT=1) — proving
  that WITHOUT the EOT guard the short final delivery is SILENTLY declared complete.

**FALSE-FIRE discipline (proven):** legit complete transfer (delivered==committed, crc==crc) →
no fire (U0); no-data / pure-receiver close (0==0, INIT==INIT) → no fire (U0b); robust-only clean
transfer matches (config-independent accumulators) → no fire; absent/legacy/garbled CLOSE (crc8
fails) → absent-EOT safe no-op. SCOPE: EOT fires on the CLOSE-RECEIVER side (covers the standard
CMD-drains-and-closes → RSP-checks one-directional flow); a receiver-initiated disconnect is not
checked (O2 role-switch, out of scope). The per-batch same-position-corruption EOB co-stamp (3a)
that would LOCALISE content corruption mid-stream remains DEFERRED (out of the compression-off cohort).

**Wire cost:** 14 B on the CLOSE_CONNECTION frame, once per graceful disconnect (was 1 B). Zero cost
on the data path. **Merge:** owner fork (wire change); lands only after its regression-cohort gate.

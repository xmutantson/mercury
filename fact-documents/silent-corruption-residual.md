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

### §13.verify — result + honest status (2026-07-03)
- `build.sh o3` EXIT=0 (clean). `mercury.exe --test` EXIT=0, FULL suite GREEN — incl. the demote/
  re-stage groups my change touches: `[TEST-INBAND-DELIVER] ALL PASS`, the `LOSSLESS DEMOTE`
  contiguous-rollback path, `[TEST-A3-DECOUPLE]` byte-faithful ALL PASS, `[TEST-INBAND-DROP/FOLLOW/
  PB/DOWNLADDER/…]` ALL PASS. (`[CRYPTO] ERROR … rejecting KX` lines are the expected negative-test
  rejections.)
- `--test-restage-requeue-orphan`: FAIL-BEFORE (`MERCURY_RESTAGE_ORPHAN_DEFEAT=1`) rc=1 order_ok=0
  first_bad=0; PASS-AFTER rc=0 order_ok=1 zero-loss.
- **Commit `d51cf40e`** on `fix/restage-requeue-orphan-shift` (off `staging/enc-bsi-nonce@683651fe`),
  author xmutantson, zero attribution. The 7 buggy `push(messages_tx[i].data …)` loops exist
  IDENTICALLY on `monitor` (grep = 7) → clean cherry-pick target.
- **STATUS — honest:** the DOMINANT root (CMD-side re-stage orphan/reorder → positional shift) is
  decisively identified (Δ=48 capture, exact-identity on all 4, code + c31w010 FIX-9 log) and the fix
  is DETERMINISTICALLY PROVEN (fail-before/pass-after) + full `--test` green. **NOT yet
  cohort-confirmed on the real-channel metric.** The conclusive check is a ~120-cell WGN:25 cohort
  with the FIXED binary (rate ~3.3% ⇒ a small run is inconclusive), using the STRONGER oracle
  (`canonical_bytes` 24-bit block-index + `random_slice`, since the period-256 ruler is blind to
  256-aligned shifts — §11.CORRECTION pt 1). That cohort would also surface any residual RSP-side
  shift source (cross-storage PREV delivery was NOT exhaustively audited; the CMD re-stage is the
  strongly-evidenced dominant source). If a fixed-binary cohort still shows silent fails → that is
  the 2nd failed cross-domain audit → Fable-5 consult gate.

## §14 THE 4th DISTINCT SILENT MECHANISM — c31w104 (fixed-binary cohort-3, 2026-07-03)

**Status:** the §13 restage fix (order-preserving/lossless re-queue + ingestion reserve) DID
eliminate the DOMINANT CMD-side mechanism (cohort-3 dropped the prior 4→1 silent fail), but a
**4th, DISTINCT silent corruption survives** — `c31w104`, a fixed-binary (`/dev/shm/build/mfix/
mercury`) WGN:25 run, box .31 cohort-3 wave-1 cell-04. This is the **campaign's own §13.verify
pre-registered trigger**: "if a fixed-binary cohort still shows silent fails → that is the 2nd
failed cross-domain audit → Fable-5 consult gate." The gate is now MET.

**Evidence (durable):** `_research/_c31w104_capture/` — `arq_c31w104.log` (3.1 MB CMD+RSP),
`capbytes_c31w104.txt` (the byte-dump: DELIV vs EXPEC at the fault), `res_c31w104.json`,
`bridge_c31w104_stats.json`, `spawn_c31w104.out`, `bsz3_driver.sh`. Fleet source:
`192.168.2.31:/dev/shm/bsz3_31/logs_w1/`. Oracle = **`random_slice`** (SHA-256 keyed-random,
key `mercury-capstone-incompressible-v1`, `tools/sim/realaudio/capstone_arms.py:542`) — the
STRONGEST oracle (NOT the period-256 ruler; §11.CORRECTION pt 1 requirement satisfied).

### §14.1 The DECISIVE byte decode — a FORWARD positional shift of Δ=+6847
`res_c31w104`: `rx_bytes=27246`, `first_bad=27079`, `mismatch=166`, `segments=2`. The two
mismatch segments are CONTIGUOUS (27079+25=27104, 27104+142=27246 = the whole 167-byte tail;
166/167 differ = exactly the strong-oracle 1/256 coincidence rate → whole tail wrong). I
reproduced `random_slice` and searched the source stream for the DELIVERED bytes:

- `EXPEC` == `random_slice(27079,·)` and `random_slice(27104,·)` — **confirms the prefix
  `[0,27079)` was delivered CORRECTLY** and pins the oracle.
- The DELIVERED bytes at `[27079,27246)` are **`random_slice(33926,167)`** — i.e. **source
  `[33926,34093)`** (both segments identical Δ; seg1@27079→src 33926, seg2@27104→src 33951).
- **Δ = +6847 (FORWARD).** NOT a value transform (DELIV⊕EXPEC is not constant: 0x66^0x24=0x42,
  0x40^0x50=0x10). NOT a stale EARLIER slot (Δ is forward). A **positional forward shift**.

### §14.2 bsi map — bsi13-range content delivered in the bsi11 delivery slot
Per-batch source ranges from the CMD flush/build sizes (RAW ⇒ flush==source bytes; cfg15 batch
= 25×167 = 4175 B): bsi11 = `[23906,28081)`, bsi12 = `[28081,32256)`, bsi13 = `[32256,36431)`.
- delivered-at position **27079 → bsi11** (3173 B in, ~frame 19).
- DELIV origin **33926 → bsi13** (1670 B in, ~frame 10).
- The RSP delivered `[0,27079)` correctly (through bsi11's first ~19 frames), then **SKIPPED
  source `[27079,33926)` = 6847 B ≈ 1.64 batches (rest of bsi11 + all bsi12 + head of bsi13)**
  and appended **bsi13 content** — then aborted. A hole + forward shift.

### §14.3 Control-flow context — the RSP cross-storage PREV path + SHRUNK expected-counts
The corruption zone is SATURATED with the RSP-side cross-storage PREV delivery
(`arq_responder.cc:1266-1321`, "swapping messages_rx pointer for delivery") — root-cause-2's
flagged-but-unaudited path. In `arq_c31w104.log`:
- bsi9/10/11 ALL delivered via `[RSP-V2-PREV-DELIVERED]` (T+221/235/247).
- **The PREV batches were declared COMPLETE at SHRUNK expected-counts**: `[RSP-V2-PREV-BUMP]`
  shows bsi10 `received_on_transfer=19/23` then PREV-DELIVER `received=23/23`; bsi11
  `17/21` then `received=21/21`. So bsi10/bsi11 were declared done at expected=23 and 21 — but
  the CMD BUILT them at 25 frames (`[CMD-BATCH-SEQ] new-data batch_seq_id=10/11 (frames in
  batch=25)`). The RSP delivered a batch it declared "complete" while ≥2-4 tail frames of that
  batch were NEVER delivered → a silent hole; the stream then re-anchors on later content.
- `[RSP-V2-GAP-ABORT]` DID fire — but at **bsi13** (T+270.522, `non-contiguous with
  last_delivered=11`) — i.e. the loud guard fired **~1.6 batches too late**, AFTER the silent
  bsi13-content shift had already entered the delivered stream.

### §14.4 Why every existing loud guard is INERT/LATE — the ARCHITECTURAL signal
- `[RESTAGE-ORPHAN]` (§13, CMD-side re-queue guard): NEVER FIRED — `DESYNC_c31.json` n_fired=0.
  This is NOT the CMD re-stage mechanism; it is a DIFFERENT site (RSP delivery / expected-count).
- `[RSP-V2-BATCHSIZE-DESYNC]` ((B), `1b99534c`): NEVER FIRED — CMD==RSP batch size (both 25); the
  desync here is expected-COUNT shrink, not batch-SIZE. Different root.
- `[RSP-V2-GAP-ABORT]`: fired LATE (bsi13), after the silent shift was already delivered.
- **The common thread across ALL 4 mechanisms:** each existing guard checks a *PROXY* invariant —
  re-queue completeness (RESTAGE-ORPHAN), batch-SIZE equality (BATCHSIZE-DESYNC), bsi contiguity
  (GAP-ABORT), EOB frame-count (Fix A / D5). **NONE checks the invariant that actually matters:
  the delivered app-byte stream == the source app-byte stream, by ABSOLUTE byte offset, in order.**
  Mercury sequences the app stream by `(batch_seq_id, frame_id)` — a BATCH-RELATIVE scheme. Every
  silent-corruption mechanism found (4 now) is a case where the `(bsi,frame)` sequencing stays
  LOCALLY CONSISTENT while the ABSOLUTE byte-stream position drifts (a dropped tail frame at a
  shrunk expected-count, a re-packed/orphaned batch, an out-of-order prev delivery). Any producer
  that keeps its proxy invariant internally consistent corrupts SILENTLY. This is the cross-layer
  "sibling bug" pattern CLAUDE.md warns about, at its 4th iteration.

### §14.5 THE SYSTEMIC BACKSTOP — a TCP-style absolute-byte-stream sequence check (DESIGN)
**Prior art:** TCP catches exactly this class (gap / reorder / dup / truncation) with **byte
sequence numbers + a checksum**, not batch/segment-relative IDs. Mercury's ARQ should enforce the
same end-to-end invariant at the delivery boundary.

**Invariant to enforce:** the RSP's cumulative delivered-byte offset at the start of each batch's
delivery == the CMD's cumulative committed-byte offset at the start of that batch. Any divergence
= a gap/shift/dup → LOUD + retx the missing span, NEVER silent-deliver.

**Hook points (both already exist as single funnels):**
- SENDER (CMD): the batch-build commit point, `process_buffer_data_commander` (the pop from
  `fifo_buffer_tx` into `messages_tx`). Maintain an authoritative cursor `tx_stream_committed`
  = total source bytes committed to batches so far. Stamp each batch with `batch_start_offset =
  tx_stream_committed` (before adding this batch's payload). The cursor is the SINGLE SOURCE OF
  TRUTH for stream position — it advances by EXACTLY the payload bytes committed, so any
  orphan/reorder/re-pack that changes which source bytes a batch carries is reflected as an
  offset the RSP can check.
- RECEIVER (RSP): the delivery boundary, `copy_data_to_buffer()` / `fifo_push_rx()`
  (`arq_common.cc:13944-13970`, the RAW path; :13781 compressed path). Maintain
  `rx_stream_delivered` = total app bytes pushed to the RX FIFO. Before delivering a batch, assert
  `batch_start_offset == rx_stream_delivered`. Mismatch → `[STREAM-OFFSET-DESYNC]` LOUD; do NOT
  deliver; the span `[rx_stream_delivered, batch_start_offset)` is the missing region → drive the
  existing ARQ retx by the bsi range that covers it, OR abort-and-resync (like GAP-ABORT but keyed
  on the RIGHT invariant and at the RIGHT time — the FIRST byte of divergence, not 1.6 batches late).

**Wire cost:** the `batch_start_offset` is a per-batch field. A 5-byte (40-bit) absolute offset
covers ≤1 TB/session; a 4-byte (32-bit, wrapping) offset covers 4 GB and is ample for any HF
session, at **~4 B / batch** = 4/4175 ≈ **0.10 %** overhead at cfg15 (less at higher configs,
more only for tiny robust batches where a 1-2 B varint suffices). Carry it in the EOB frame (which
already terminates each batch) or a dedicated batch-header field. **OPTIONAL add-on:** a per-batch
CRC-32 of the payload-as-sent (+4 B) catches a per-frame-CRC ESCAPE (a bit error that passes the
frame CRC) — not needed for the 4 positional mechanisms but closes the value-corruption class too.

**Why it SUPERSEDES per-mechanism whack-a-mole:** it checks the ACTUAL end-to-end invariant at the
consumer, so it catches EVERY positional shift/drop/reorder/dup — all 4 mechanisms found AND any
future 5th — LOUDLY at the first divergent byte, regardless of which producer/layer caused it.
It converts silent-data-loss into loud-recoverable structurally, ending the serial per-root hunt.

**Honest limit:** Mercury's RECEIVER has no independent oracle for the source CONTENT (only the
harness does), so this backstop verifies stream POSITION/CONTIGUITY (offset) + optionally
transmission integrity (payload CRC). It CANNOT catch a sender that sends wrong bytes while
stamping a CONSISTENT offset AND a matching CRC — but that requires the sender's cursor to be
authoritative-yet-wrong, which the design makes the single source of truth precisely to prevent.
For all 4 observed mechanisms (positional shifts/holes with the app-stream re-anchoring) the
offset check is NECESSARY AND SUFFICIENT to catch loudly.

### §14.6 DECISION — Fable-5 gate MET; wire-format change ⇒ owner fork
- c31w104 does **NOT** have a clean, cheap, ISOLATED per-mechanism fix: the root is entangled with
  the EOB-inference / D5 / PREV-shrink / SHRINK-DEFER machinery that Fix A + D5 + the shrink-defer
  already patch. A 4th narrow patch there is precisely the whack-a-mole the 4-mechanism pattern
  proves is losing (each prior narrow fix exposed a sibling). NOT shipped as a narrow patch.
- The systemic backstop is the correct structural fix but is a **WIRE-FORMAT change** (new
  per-batch offset field) — the task's explicit Fable/owner escalation trigger.
- Both Fable-5 gate conditions are satisfied: (i) the campaign's own §13.verify pre-registered
  "fixed-binary cohort still shows a silent fail = 2nd failed cross-domain audit"; (ii) the fix is
  an architectural decision beyond a clean autonomous implementation.
- **ACTION:** consult Fable-5 with a structured brief (4-mechanism pattern + the c31w104 decode +
  what each prior fix did + the systemic-vs-whack-a-mole + wire-format question); act on its steer
  with ONE attempt; then surface the fork to the owner. See §14.7 for the consult brief.

### §14.7 FABLE-5 STRUCTURED BRIEF + OWNER FORK (this agent could not spawn Fable — routed up)
This agent is a workflow-spawned SUBAGENT with no Agent/Workflow spawn tool available, so it could
not itself page Fable-5. The gate IS met (§14.6). The complete brief is recorded here for the
orchestrator/owner to route to Fable-5 (`model:"fable"`), then decide.

**BRIEF — the problem (metric):** at marginal SNR (WGN:25, random-binary, compression+encryption
OFF) Mercury SILENTLY delivers a positionally-corrupted app byte stream ~a few % of transfers. The
metric is silent byte-integrity failures per fixed-binary cohort. After THREE root fixes it is
still 1/14 in cohort-3 (c31w104), and each prior fix revealed a distinct sibling.

**BRIEF — what's been tried (4 distinct roots, each a real fix, each exposed a sibling):**
1. Fix A (EOB-inference undercount / tail-drop) — wired the true frame count so a lost-EOB batch
   isn't declared short. Landed. Sibling surfaced ↓.
2. (B) CMD>RSP `data_batch_size` desync (`1b99534c`) — LOUD `[RSP-V2-BATCHSIZE-DESYNC]` when a
   received frame's id ≥ RSP batch size. Landed, regression-free. Did NOT cover the next root ↓.
3. §13 re-stage orphan/reorder (Δ=+48 capture) — 7 CMD demote/CFG16-HOLD re-stage sites re-queued
   in-flight frames with forward `push()` (reorder) + silent drop on a full fifo; fixed with
   order-preserving `push_front` + ingestion reserve + LOUD `[RESTAGE-ORPHAN]`. Eliminated the
   DOMINANT mechanism (cohort silent 4→1). Did NOT cover c31w104 ↓.
4. c31w104 (THIS) — decode-proven FORWARD shift Δ=+6847: RSP declared bsi10/bsi11 COMPLETE at
   SHRUNK expected-counts (23,21) though transferred=25 → silent tail-frame holes → bsi13 content
   re-anchored at the bsi11 stream slot. RESTAGE-ORPHAN + BATCHSIZE-DESYNC INERT; GAP-ABORT fired
   1.6 batches LATE. Entangled with the EOB/D5/SHRINK-DEFER machinery → no clean isolated patch.

**BRIEF — the evidence (decisive):** `random_slice` decode proves DELIV[27079,27246)==source
[33926,34093), Δ=+6847, prefix correct; NOT a value transform (DELIV⊕EXPEC non-constant). bsi map
puts the origin in bsi13 and the slot in bsi11. Full capture `_research/_c31w104_capture/`.

**BRIEF — the specific question:** every existing guard checks a PROXY invariant (re-queue
completeness / batch-SIZE / bsi-contiguity / EOB-count); the corruption is always an ABSOLUTE
byte-stream position drift the proxy misses. Is a TCP-style **per-batch absolute-byte-offset stamp
+ delivery-boundary contiguity check** (§14.5; ~4 B/batch ≈ 0.1%; a WIRE-FORMAT change) the right
structural fix that SUPERSEDES per-mechanism whack-a-mole — or is there a lower-cost consumer-side
invariant (e.g. RSP refuse to declare a batch complete when delivered/expected < the already-wired
`transferred` count) that closes the class WITHOUT a wire change, accepting it won't catch a
future content-shift-with-consistent-counts 5th mechanism? What would you ship?

**OWNER FORK (surface if Fable's one steer doesn't dissolve it):**
- **Option W (systemic, wire-format):** add the absolute-byte-offset stamp + delivery check
  (§14.5). Closes the whole silent-shift class loudly + localizes retx. Cost: ~0.1% wire, a
  wire-format bump (both ends rebuild — acceptable pre-ship per `iris_no_version_bits_preship`
  discipline), and touching the batch/EOB framing + both delivery funnels. RECOMMENDED as the
  durable end-state; needs owner sign-off because it changes the wire.
- **Option N (narrow, no wire change):** RSP-side LOUD guard at the PREV/BATCH-DONE delivery
  boundary: if a batch is declared complete with delivered/expected < the observed `transferred`
  count AND the shortfall slots were never delivered → `[RSP-V2-TAILDROP-HOLE]` + refuse
  silent-complete (abort-resync). Cheaper, reversible, but (i) entangled with the legitimate
  data_batch_size-shrink / SHRINK-DEFER path (false-fire risk — must distinguish a real resize
  from a hole), and (ii) whack-a-mole — will not catch a 5th mechanism that keeps counts
  consistent. A stopgap, not the end-state.
- **Recommendation:** ship Option W (systemic) as the structural end-state; it is the only fix that
  makes silent-data-loss STRUCTURALLY impossible for this whole class. Until W lands, real-channel
  silent corruption at marginal SNR is **STILL OPEN** (~a few % of WGN:25 transfers).

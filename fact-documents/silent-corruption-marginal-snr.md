# Silent byte-corruption at marginal SNR (WGN:25) — rx_btf=-1 current-batch shrink tail-drop

**Status**: ROOT-CAUSED + FIXED + unit-proven (fail-before/pass-after) on branch
`fix/eob-undercount-silent-tail`, branched from **monitor `1e60ef80`** (the merged
correctness-consolidation). Pairs the shared-state fact docs
`data-flow-d5-eob-batch-truncation.md`, `baseline-double-delivery.md`, and
`data-flow-mixbatch-fill.md` (the seed-6 batch-boundary family — a SIBLING, not this).

---

## §1 The symptom (as reported)

At a marginal SNR patch (real-audio WGN:25), a COMPLETED transfer fails end-to-end
byte integrity: the delivered app stream is byte-CORRECT through some mid-stream
offset (e.g. correct thru offset 41257) and then CORRUPTED from there (e.g. 7665
bytes wrong). The bytes pass every modem frame check (per-frame CRC OK, decode OK) —
this is NOT a CRC/undetected-error escape. It is an ARQ **reassembly** bug: real,
in-order, already-RECEIVED user bytes are silently DROPPED at a batch boundary, so
everything after the drop is shifted → md5_match FALSE, with NO `[RSP-V2-GAP-ABORT]`
(the batch delivers CONTIGUOUSLY, just short).

## §2 Root cause — a mid-flight `data_batch_size` SHRINK orphans the CURRENT batch

### §2.1 The two facts that compose into the bug

1. **`header_carries_d5 = !is_robust_config(configuration)`** (arq_common.cc:2247).
   The D5 wired per-batch `batch_total_frames` count — the authoritative frame count
   that survives the loss of any single frame and CURES the EOB-inference undercount
   at WB configs (`data-flow-d5-eob-batch-truncation.md`) — is **gated OUT at every
   robust config** (ROBUST_0..2 = configs 100–102, `common_defines.h:155`). So at a
   robust config **`rx_batch_total_frames` (rx_btf) is permanently −1** — the robust
   tier is ALWAYS in the exact "D5-inference-defeated" state that
   `MERCURY_D5_INFER_DEFEAT=1` simulates at WB, with NO wired-count backstop. The D5
   fact-doc scoped this out on the assumption "robust is pinned to batch=1 → no
   lost-tail hazard" — that assumption is **STALE** (see fact 2).

2. **Robust is no longer batch-1-pinned.** `set_data_batch_size` (arq_common.cc:1382)
   FIX-A (`data-flow-robust-tier-arq-batch.md` §5.2) made the robust clamp a RANGE
   clamp `[1..ROBUST_DWELL_BATCH_MAX]` (`ROBUST_DWELL_BATCH_MAX = 8`,
   `common_defines.h:188`); the ROBUST_DWELL_BATCH_OP transport raises robust to a
   multi-frame batch. So a **multi-frame batch runs at rx_btf=−1**.

### §2.2 The unguarded producer

`defer_shrink_if_would_orphan_prev` (arq_common.cc:1241) — Fix A
(`baseline-double-delivery.md`) — defers a mid-flight `data_batch_size` shrink that
would orphan already-RECEIVED frames, BUT it scanned **only `messages_rx_prev[]`**
(the sealed prev batch, line 1252 pre-fix) and early-returned unless
`rsp_prev_batch_active`. The **CURRENT** batch (`messages_rx[]`) was never checked.

So when a mid-flight shrink fires (a WB→robust demote or an Axis-2 down-move at a
marginal-SNR patch — the DEMOTE-AMPLIFIER regime) while the CURRENT batch holds
RECEIVED frames in `[new, old)` and no prev is active, the shrink **APPLIES**. Then:

- `bump_bsi_and_transfer_prev` (arq_common.cc:9853) transfers only
  `[0, data_batch_size)` = `[0, new)` into `messages_rx_prev[]` and **FREEs the source
  slots** (transfer loop bounded by `data_batch_size`, :9954-9973 post-fix) — the
  `[new, old)` RECEIVED frames (real in-order user bytes) are **dropped at the seal**.
- `prev_expected` collapses to `new` (default `= data_batch_size`; rx_btf=−1 so the
  wired-count re-derivation at arq_responder.cc:1165 cannot fire; the EOB inference
  cannot GROW it past `data_batch_size`).
- The prev-completion count gate (`received_count >= expected_count`,
  arq_responder.cc:1208) fires with `received == expected == new`, and
  `copy_data_to_buffer()` delivers only `[0, new)` — a **silent short delivery**.

`delivery_step_is_gap` (D3.1) does NOT fire: the batch delivered CONTIGUOUSLY at the
correct bsi, just SHORT — the high-water advances normally. Hence "correct thru
offset X then corrupted," no gap-abort, md5 FALSE.

This is why the D5 wired-count fix (which cures the identical undercount at WB) does
NOT cover it: at robust the wired count is absent BY DESIGN, so the ONLY protection
is not letting the shrink orphan the current batch in the first place.

### §2.3 Fail-before reproduction (in-process, drives the REAL producers)

`--test-batch-shrink-orphan-current` (arq_responder.cc `test_batch_shrink_orphan_current`,
CLI + master `--test` in main.cc). rx_btf=−1 (`header_carries_d5=false`,
`rx_batch_total_frames=-1`), compression OFF (byte-exact), a COMPLETE 25-frame current
batch in `messages_rx[]`. Drives the REAL `set_data_batch_size(8)` chokepoint, the REAL
`bump_bsi_and_transfer_prev()` seal, the REAL prev count gate + `copy_data_to_buffer()`
reassembler, and `fifo_buffer_rx` as a byte oracle.
- **fail-before** (`MERCURY_BATCHSHRINK_ORPHAN_DEFEAT=1`): `dbs 25→8 (shrink APPLIED)`,
  `prev_expected=8`, delivered = **128 B (8 frames)**, `truncated_new=1` — the 17-frame
  (272-byte) tail SILENTLY dropped (the bug reproduced).
- **pass-after** (default): `deferring 25→8 … orphan 0 prev + 17 current`,
  `dbs 25→25 (DEFERRED)`, `prev_expected=25`, delivered = **400 B (25 frames)**,
  `faithful_full=1` — zero bytes lost.

## §3 The fix (RX-side, config-agnostic, no wire change)

1. **`defer_shrink_if_would_orphan_prev`** (arq_common.cc:1241) now ALSO scans
   `messages_rx[]` (the current batch) for RECEIVED slots in `[new, old)` — the
   `messages_rx` sibling of the existing `messages_rx_prev` guard — and defers the
   shrink if either store would be orphaned (dropped the `!rsp_prev_batch_active`
   early-return so a current-batch orphan defers even with no prev active). The batch
   is held at its framed span and delivers/seals COMPLETE; the deferred target applies
   the instant the batch clears.
2. **`rsp_apply_deferred_batch_shrink()`** added at the in-order BATCH-DONE delivery
   (arq_responder.cc:2686) — previously only the prev-deliver sites applied the
   deferred shrink, so a current-batch that delivered IN-ORDER left the shrink pending.
   Self-guards (no-op while a prev still holds).
3. **LOUD seal-orphan detector** (charter: "a silent-false-accept must become a loud
   detect") — `bump_bsi_and_transfer_prev` (arq_common.cc, before the transfer loop)
   logs `[RSP-V2-SEAL-ORPHAN]` if any RECEIVED current-batch slot would be stranded
   beyond `data_batch_size` at seal. Log-only (delivery unchanged) — a regression alarm
   that turns any future silent tail-drop into a diagnosable event. With fix (1) it
   cannot fire on the reproduced path.

Env `MERCURY_BATCHSHRINK_ORPHAN_DEFEAT=1` reverts (1) on the SAME binary (one
fail-before toggle shared with the Fix A prev guard).

## §4 Cross-layer interaction with the just-merged seed-6/mixbatch fixes (1e60ef80)

The merged correctness consolidation carries the **CMD-side** seed-6/mixbatch fixes
(`data-flow-mixbatch-fill.md`: Root A no-comp fill-cap, Root A comp `crypto_frames`
cap, Root A/ii force-FREE guard) — they cap `messages_tx[]` new-data staging so a v2
mixbatch never over-pops the retx prefix. **This fix is on the RX side** (RSP prev/
current reassembly gate + the `set_data_batch_size` shrink chokepoint) and shares NO
state with them: the mixbatch fixes touch `messages_tx[]`/`fifo_buffer_tx`/
`fifo_buffer_backup` (producers); this touches `messages_rx[]`/`messages_rx_prev[]`/
`data_batch_size`/`rsp_deferred_batch_shrink` (consumers). The `MERCURY_MIXBATCH_OVERPOP_DEFEAT`
(CMD) and `MERCURY_BATCHSHRINK_ORPHAN_DEFEAT` (RX) envs are independent. Both are
byte-boundary reassembly siblings but at DIFFERENT layers — clean distinct roots. The
existing prev-guard sibling (`test_batch_shrink_orphan_defer`) and the D5 test
(`test_eob_loss_batch_truncation`) both still PASS after this change.

## §5 Cross-layer audit — `data_batch_size` shrink vs the deferred-shrink state

- **Producers of `rsp_deferred_batch_shrink`**: `defer_shrink_if_would_orphan_prev`
  (set, arq_common.cc:1255, now on prev OR current orphan); reset at session start
  (arq_common.cc:738), and consumed/cleared in `rsp_apply_deferred_batch_shrink`.
- **Consumers (apply sites) of the deferred shrink**: prev-deliver completion
  (arq_responder.cc:1346), pre-BREAK flush (arq_common.cc:5167), **in-order BATCH-DONE
  (arq_responder.cc:2686 — added here)**, and the Fix A unit test (12369). All are
  reached when a held batch clears; `rsp_apply_deferred_batch_shrink` self-guards on
  `rsp_prev_batch_active` and only re-applies a genuine still-shrink (never grows).
- **Invariant preserved**: the CMD/RSP robust-batch agreement (set_data_batch_size
  chokepoint) is not broken — the defer is transient (the in-flight OLD-size batch
  delivers, THEN the shrink applies), identical to the Fix A prev precedent which
  already holds `data_batch_size` at the old value across a robust config while a prev
  delivers. No deadlock: every batch clears (deliver/seal/BREAK/reset), each clear
  applies the pending shrink; `rsp_deferred_batch_shrink` resets on session reset.
- **No regression for the common path**: a shrink with no RECEIVED slot in `[new,old)`
  (empty current batch, or only low slots occupied) does NOT defer — byte-identical to
  pre-fix. `!sack_v2_enabled` / v1 / NB: `defer_shrink_if_would_orphan_prev` early-
  returns — byte-identical.

## §6 Vigilance for future changes

Before changing `defer_shrink_if_would_orphan_prev`, the `bump_bsi_and_transfer_prev`
transfer bound, the prev count gate, `copy_data_to_buffer`'s `[0,data_batch_size)`
delivery bound, or `header_carries_d5` — re-walk §2/§5 here AND
`baseline-double-delivery.md` + `data-flow-d5-eob-batch-truncation.md`. The load-bearing
composition: **a batch must never be delivered at a span shorter than the frame count
it was framed at** — at rx_btf=−1 there is no wired count to re-derive it, so the ONLY
defense is refusing to shrink `data_batch_size` (or `expected_count`) below the
already-RECEIVED span.

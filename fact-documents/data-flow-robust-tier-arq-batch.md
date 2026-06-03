# Data-Flow Audit: robust-tier `data_batch_size` (FIX-A — decouple the ROBUST dwell batch from the *steady-state* batch=1 pin)

**Status**: IMPLEMENTED 2026-06-03 on branch `fix/robust-dwell-batch` (off
`feat/sim-clock`). The PURE-ANALYSIS sections below (written against inner-mercury
`monitor` `0c75cc2`) drove the implementation; §10 records the AS-BUILT delta
(exact file:line on `fix/robust-dwell-batch`, the design choices the analysis left
open, and the unit-test + sim results). Where a §1–§9 line number disagrees with
§10, §10 is authoritative (the branch evolved past `0c75cc2`). Validation: the
in-process `--test-climb-engine` units (Parts D'1–D'6, D'1/D'3 FAIL-BEFORE→PASS-AFTER
proven) + FTRT ARQ sim; HW muething is the final arbiter (not yet run).

**Companion / parent doc**: `fact-documents/data-flow-batch-size.md` is the
canonical owner of `data_batch_size` producer/consumer facts and the
**CMD/RSP-symmetry "4-wire-failure" invariant**. THIS doc is the FIX-A delta on
top of it: it does not restate every producer/consumer (see the parent for the
full list) but it RE-WALKS each one against the specific change FIX-A makes and
flags the landmines. Where this doc and the parent disagree, the parent's
file:line wins and the discrepancy is called out (§0.2). Every change to the
robust-tier batch pin MUST update BOTH docs.

---

## §0 Brief-vs-source reconciliation (READ FIRST — the task brief is partly stale)

The task brief describes the floor-batch pin via a function
`climb_owns_robust_batch()` (`arq.h:334`) returning `gear_shift_on==YES`, a
chokepoint at `arq_common.cc:617-619`, `sack_negotiated_recompute_batch()` at
`arq_common.cc:771`, and a regression test `tools/test_robust_batch_sack.py`.
**None of those symbols/paths/line-numbers exist on `monitor` `0c75cc2`.** They
appear to come from a synthesized description or a stale/local ref. Grounding the
audit in the brief verbatim would have shipped a fix against code that isn't
there. The REAL monitor mechanism is below. Discrepancies, for the record:

### §0.1 What actually pins ROBUST to batch=1 on `monitor`

There is NO `climb_owns_robust_batch()` and NO `gear_shift_on`-keyed batch pin.
`git grep climb_owns_robust_batch 0c75cc2` = 0 hits. The robust batch=1 pin is
**config-keyed, NOT gear-shift-keyed**, enforced at TWO production sites plus a
chokepoint, all gated on `is_robust_config(current_configuration)` and NONE on
`gear_shift_on`:

1. **`load_configuration()` robust branch** (`arq_common.cc:1332-1337`):
   ```cpp
   if(is_robust_config(configuration))   // :1332
   {
       set_data_batch_size(1);
       set_ack_batch_size(1);
       set_control_batch_size(1);
   }
   ```
   (There is a SECOND `if(is_robust_config(configuration))` block at
   `arq_common.cc:1441` — the post-`message_transmission_time_ms` ACK-pattern
   batch block; both must be re-walked.)
   Runs on EVERY config load that lands on a robust config (connect, BREAK→ROBUST_0,
   turbo-reverse). `current_configuration` is already `=configuration` at `:1269`
   before this. The OFDM batch-scaling at `:1372` is `!is_robust_config`-gated.

2. **`set_data_batch_size()` CHOKEPOINT** (`arq_common.cc:573-618`):
   ```cpp
   if(is_robust_config(current_configuration) && data_batch_size != 1)   // :595
   {
       if(this->data_batch_size != 1)
           printf("[BATCH-CHOKEPOINT] robust config %d: clamped requested batch %d -> 1 ...\n", ...);
       this->data_batch_size = 1;   // :604
       return;
   }
   ```
   THE single enforcement point. ANY producer that calls the setter while
   `current_configuration` is robust is force-clamped to 1. (The brief's
   `:617-619` is the *non*-robust clamp tail `data_batch_size < (max_data_length+…)`
   — a different clamp.)

3. **`sack_negotiated_recompute_batch()`** (defined `arq_common.cc:741`, NOT
   `:771`): its `if(!is_robust_config(current_configuration))` guard means the
   SACK floor (`>=5`) is NEVER applied at robust — it leaves the `load_configuration`
   pin of 1 in place.

So FIX-A's premise — "gate `climb_owns_robust_batch()` on active climb intent
instead of `gear_shift_on` alone" — **cannot be implemented as written**: there
is no such function, and the pin is not `gear_shift_on`-keyed. The CORRECT FIX-A
target is the three config-keyed sites above (§5). The brief's *intent* (run
batch 4-8 at a settled ROBUST_0 dwell) is preserved; the *mechanism* is restated
to match `monitor`.

### §0.2 The brief's `~36 B quantum` mechanism — partly confirmed, partly corrected

CONFIRMED: at robust, batch IS pinned to 1, so a large message is delivered one
MFSK frame at a time with full ACK turnaround between frames (>90% dead time on
the deep-SNR floor). CORRECTED: the cause is the config-keyed pin (§0.1), NOT a
`gear_shift_on` gate. The pin is INTENTIONAL and load-bearing for the *climb*
(§1, §4) — it is not an accidental `-g` side effect. Removing it naively
RE-INTRODUCES the exact 4-wire-failure class the parent doc documents. THIS is
the whole reason FIX-A is a §5 cross-layer audit and not a one-line gate flip.

### §0.3 The brief's `test_robust_batch_sack.py` does not exist

`find . -name test_robust_batch_sack*` = 0 hits in the entire workspace. There is
NO such script to "re-run with `-g`". The real, in-process, paired regression
test is `--test-climb-engine` → `test_climb_engine()` (`arq_commander.cc:6401`),
whose **Part D** (`:6635-6712`) is the connect-path CMD/RSP-batch-symmetry test.
The §6 test plan below extends Part D rather than inventing a missing script. (If
a wire-level `-g` Python harness is wanted it must be WRITTEN; `sack_v2_loopback_test.py`
and `quick_sack_test.py` in `mercury/tools/` are the closest existing scaffolds.)

---

## §1 The shared state and the invariant FIX-A must not break

**Shared state**: `int data_batch_size` (member of `cl_arq_controller`,
declared `arq.h:1290`, ctor default `1` at `arq_common.cc:146`). Crosses
PHY ↔ ARQ ↔ SACK. The CMD packs a batch of this many DATA frames; the RSP forms
its RX expectation from it; **BOTH sides independently compute the clean-batch
all-ones SACK target `(1<<data_batch_size)-1`** (§3.1).

**The 4-wire-failure invariant (parent doc §1, unchanged by FIX-A)**:

> At any moment, **CMD `data_batch_size` == RSP `data_batch_size`**. The two
> sides compute the clean-ACK target `(1<<batch)-1` independently; if they ever
> diverge, the all-ones targets never match, no clean confirmation is credited,
> the climb cannot start/advance, and the link LINK-TIMEOUTs.

This invariant is SYMMETRY, not the specific value 1. The parent doc additionally
asserts the value MUST be **1 at robust steady state** — for a *distinct* reason
(§4): at the MFSK cliff `P(batch clean) = p^N`, batch=1 is the STRICTEST clean
target (one frame == a clean batch) so a clean ACK is achievable at the floor SNR;
at batch=N the clean target `0x{2^N-1}` requires ALL N frames to survive
first-pass, which they don't at the floor → zero clean credit → frozen anchor.

**FIX-A's job**: lift batch from 1 to 4-8 at a ROBUST dwell ONLY when the link is
already PROVEN to deliver clean batches at this rung (so the `p^N` penalty is
acceptable), WITHOUT (a) breaking CMD/RSP symmetry and (b) starving the climb of
the strict batch=1 clean credit it needs WHILE still climbing. The whole design
hinges on distinguishing "still proving the rung / climbing" (needs batch=1) from
"rung proven, parked here, just moving bulk data" (can afford batch 4-8 + SACK).

---

## §2 PRODUCERS — every write to `data_batch_size` (file:line on `monitor` 0c75cc2)

All production writes funnel through `set_data_batch_size()` (`arq_common.cc:573`)
EXCEPT two test-only direct assigns (§2.8). The setter is the chokepoint.

### 2.1 `set_data_batch_size()` — THE chokepoint / sole setter (`arq_common.cc:573-618`)
- `:595` robust clamp: `if(is_robust_config(current_configuration) && data_batch_size != 1){ … this->data_batch_size = 1; (:604) return; }`
- `:608-617` non-robust path: store requested value, clamped to `< max_data_length+max_header_length-ACK_MULTI_ACK_RANGE_HEADER_LENGTH-1`.
- Reads member `current_configuration` (live PHY). **FIX-A MUST change this — the
  `!= 1` robust clamp is what would clamp a 4-8 batch right back to 1.**

### 2.2 `load_configuration()` robust branch (`arq_common.cc:1332-1337`; second twin at `:1441`)
- `:1334` `set_data_batch_size(1)` for `is_robust_config(configuration)` (`if` at `:1332`).
- Reads the `configuration` arg; member `current_configuration` already set to it
  at `:1269`. Runs at connect, BREAK→ROBUST, turbo-reverse, any robust load.
- **FIX-A MUST change this**: the *initial* robust batch must still be 1 (climb
  starts unproven), but FIX-A's promotion-to-4-8 happens LATER (§5), so this site
  stays `set_data_batch_size(1)` — it is the correct INITIAL value. (Verify the
  chokepoint change in §2.1 still permits a later explicit raise.)

### 2.3 `load_configuration()` FULL default (`arq_common.cc:1322`)
- `:1322` `if(level==FULL) set_data_batch_size(default_configuration_ARQ.batch_size);`
  Runs BEFORE §2.2 robust branch; if config is robust the §2.2 branch then pins 1.
  Unchanged by FIX-A (the robust branch overrides it).

### 2.4 `load_configuration()` OFDM batch-scaling (`arq_common.cc:1372-1388`)
- `:1386` `set_data_batch_size(fixed_batch)` where `fixed_batch = sack_enabled ?
  radio_batch_size(25) : 10`, clamped to a 30 000 ms / `message_transmission_time_ms`
  ceiling, `!is_robust_config(configuration)`-gated. OFDM only. Unchanged by FIX-A.

### 2.5 `sack_negotiated_recompute_batch()` — CMD+RSP shared SACK recompute (`arq_common.cc:741-765`)
- `:758` `set_data_batch_size(new_batch)` inside `if(!is_robust_config(current_configuration))`.
  Called by CMD at TEST_CONNECTION_ACK (`arq_commander.cc:4143`) and RSP at
  TEST_CONNECTION (`arq_responder.cc:2159`) — IDENTICAL body, cannot diverge.
- Robust path: guard FALSE → no write → leaves the §2.2 pin of 1. **FIX-A leaves
  this alone at connect** (connect is unproven → batch=1 correct); the 4-8 raise
  is a separate post-proof producer (§5.2), NOT this connect-time recompute.

### 2.6 `policy_evaluate_axis2()` — Axis-2 adaptive batch (OFDM only) (`arq_commander.cc`)
- Robust guard at top: `if(is_robust_config(current_configuration)) return;`
  (parent doc §2.4, the 86d39b4/§2.3-Bug3 guard). Steps `data_batch_size +=
  AXIS2_STEP(5)` clamped `[AXIS2_BATCH_FLOOR=10, AXIS2_BATCH_CEIL=32]` then sends
  SET_LINK_PARAMS. **FIX-A MUST NOT route the robust 4-8 raise through Axis-2** —
  Axis-2's `[10,32]` floor would clamp 4-8 UP to 10 on the RSP and re-create
  Bug 3's CMD=6/RSP=10 mismatch (climb-engine doc §2.1 step 5). KEEP this guard.

### 2.7 RSP SET_LINK_PARAMS apply (`arq_responder.cc:2609-2615` region)
- On CRC-pass, clamps CMD's requested batch to `[AXIS2_BATCH_FLOOR=10,
  AXIS2_BATCH_CEIL=32]` then `set_data_batch_size(target)`. Driven ONLY by
  Axis-2 (§2.6), which never fires at robust → never reaches here at robust.
  The §2.1 chokepoint backstops any stray robust frame. **FIX-A's robust raise
  must use a DIFFERENT transport than SET_LINK_PARAMS** (see §5.2 landmine L4) or
  it inherits this `[10,32]` clamp.

### 2.8 Test-only DIRECT assigns (BYPASS the setter) (`arq_responder.cc`)
- `this->data_batch_size = 25;` (SACK-v2 mixbatch test) and `= 30;` (synthetic
  SET_LINK_PARAMS test) — see parent doc §2.7 for exact lines on `fix/climb-engine`;
  on `monitor` confirm via `git grep "this->data_batch_size ="`. They bypass the
  chokepoint deliberately (OFDM-batch SACK tests, `current_configuration` not robust).
  Not affected by FIX-A.

### 2.9 (NEW under FIX-A) the post-proof robust dwell raise — §5.2
The ONLY new producer FIX-A introduces. Writes 4-8 at a robust config AFTER the
rung is proven. Must (a) pass the relaxed chokepoint, (b) propagate to RSP via a
symmetric transport, (c) revert to 1 on any climb resume / BREAK / config change.

---

## §3 CONSUMERS — every read of `data_batch_size`, re-walked for FIX-A

### 3.1 Clean-batch all-ones SACK target (THE symmetry-sensitive consumer)
- **CMD**: `arq_commander.cc:131-133` (`sack_clean_confirmation_accepted()`,
  `all_ones = (data_batch_size>=32)?0xFFFFFFFF:((1u<<data_batch_size)-1u)`) and
  the data-ACK arm at `arq_commander.cc:2514-2522`.
- **RSP emit**: `arq_responder.cc:801` (clean prev-path), `:1576` (partial path),
  `:1711` (clean path): `bitmap_u32 = (data_batch_size>=30)?0x3FFFFFFFu:((1u<<data_batch_size)-1u)`.
  NOTE: RSP caps at **30 bits** (Phase B Wave 1 flag-day, `:1707/:1554/:797`),
  CMD caps at 32. **FIX-A range 4-8 is far below 30** so the 30/32 cap divergence
  does not bite — but the audit records it.
- **Invariant**: CMD `all_ones` == RSP `bitmap_u32` REQUIRES CMD batch == RSP
  batch. FIX-A's 4-8 raise is SAFE here IFF both sides adopt the SAME value
  atomically (§5.2). **This is the consumer the 4 wire failures broke.**

### 3.2 CMD block-build / TX loop
- `arq_commander.cc:2641` (`for(i<data_batch_size && i<MAX_SACK_BATCH_SIZE)`),
  `:2842` (`if(i<data_batch_size && sack_bitmap[i])`), `:9145` (`for(i<data_batch_size)`),
  data-frame producer (`arq_common.cc` block-build). Iterating to 4-8 instead of
  1 is the WHOLE POINT — packs 4-8 frames per TX. No assumption broken; the loops
  are already `data_batch_size`-parametric. ✓

### 3.3 RSP RX expectation / ACK-GATE (`process_messages_acknowledging_data`, `arq_responder.cc:1387+`)
- Expected count: `:1421` `int expected = data_batch_size;` overridden by the EOB
  bit-7 marker `:1432-1435` (`expected = last_received_end_of_batch_seq + 1`,
  capped to `data_batch_size`). The EOB marker (CMD producer §3.6) makes `expected`
  correct even when the compressed message fills fewer than `data_batch_size` frames.
  At batch 4-8 robust this works exactly as it does at OFDM batch≥5. ✓ (Verify the
  CMD sets the EOB bit-7 on the last robust frame — §3.6.)
- **`data_batch_size > 1` branch gate** (`arq_responder.cc:1462`
  `if(data_batch_size > 1 && rx_received < expected && !passive_monitor)`):
  **LANDMINE L1.** At batch=1 this branch is NEVER taken → only the clean PASS
  funnel runs. At batch 4-8 robust it IS taken on a partial → enters the SACK
  partial path. This is INTENDED under FIX-A (selective retransmit), but it means
  the partial path (and `messages_rx_prev`, `bump_bsi_and_transfer_prev`) is now
  exercised at ROBUST_0 — previously DEAD at batch=1 (parent §3.4). Must verify
  that path's robust-tier correctness (§4.3).
- **`data_batch_size <= 1` SACK suppression** (`arq_responder.cc:1504`, log at `:1512`):
  explicit comment "SACK_RSP is meaningless on a single-frame batch … The EOB
  bit-7 on the lone DATA frame is the all-or-nothing receipt indicator. Suppress
  the dispatch entirely." At batch 4-8 this `<=1` short-circuit is NOT taken, so
  the partial SACK_RSP / MFSK-suffix dispatch fires. **This is the answer to the
  brief's question "does the MFSK SACK path assume batch==1 anywhere?"** — it does
  NOT assume it; it has an explicit `<=1` branch that is simply SKIPPED at batch>1.
  No code change needed; the path already supports batch>1. ✓

### 3.4 MFSK SACK suffix transport (config-AGNOSTIC, already works at ROBUST)
- `send_mfsk_ack_sack(bsi, bitmap_u32)` called from `arq_responder.cc:805`
  (prev), `:1576` (partial), `:1715` (clean). Gated only by `MFSK_ACK_SACK_ENABLED`
  (`common_defines.h:35` = 1) AND `ack_sack_suffix_len() > 0`.
- `ack_sack_suffix_len()` (`mfsk.h:194`): `return (M >= 16) ? 13 : 0;`. The ACK
  codec is the **dedicated `ack_mfsk` at M=16, config-INDEPENDENT**
  (`telecom_system.cc:3095` "Always use dedicated ack_mfsk (M=16, nStreams=1) —
  config-independent"). So `ack_sack_suffix_len()==13` for ANY WB session
  including ROBUST_0 (whose DATA PHY is M=32×1). The partial path comment at
  `arq_responder.cc:1556` confirms: "No optimizer-territory gate: the pattern
  correlator gives this path ROBUST_0-grade detection at any config, which is
  exactly where partial-batch recovery matters most."
- **CONCLUSION: the M=16 MFSK SACK selective-retransmit suffix is ALREADY
  config-agnostic and ALREADY supports a 30-bit bitmap at ROBUST_0 WB.** FIX-A
  needs NO new SACK transport — it only needs to (a) let robust batch be >1 and
  (b) ensure CMD/RSP symmetry on that value.
- **LANDMINE L2 (NB)**: `ack_sack_suffix_len()==0` when `M < 16` — i.e. a
  **narrowband** session (`narrowband_enabled` → ROBUST_0 is M=8, `telecom_system.cc:5232`).
  On NB there is NO SACK suffix; the receiver treats any pattern hit as a clean
  ACK (`arq_responder.cc:1690-1694` comment). At NB, a batch>1 partial CANNOT be
  selectively retransmitted (no bitmap), so FIX-A MUST keep NB robust at batch=1.
  The FIX-A gate (§5.1) MUST therefore additionally require
  `ack_sack_suffix_len() > 0` (equivalently `!narrowband_enabled`).

### 3.5 ACK-timeout math (`recalculate_ack_timeout_for_batch`, `set_ack_timeout_data`)
- Scales the data-ACK timeout by `data_batch_size` (`arq_common.cc:657/674/678`
  region + the `:1413+` block: `set_ack_timeout_data((data_batch_size+2)*msg_time…)`).
  At batch 4-8 the timeout grows ∝ batch — CORRECT, the batch takes longer to TX.
  FIX-A MUST call `recalculate_ack_timeout_for_batch()` after the robust raise
  (and after the revert-to-1), exactly as `sack_negotiated_recompute_batch()`
  does at `arq_common.cc:763`. **LANDMINE L3**: if the timeout is NOT recomputed,
  CMD times out mid-batch and full-retransmits → throughput LOSS, the opposite of
  FIX-A's goal. Recompute on BOTH sides.

### 3.6 EOB bit-7 producer (CMD) — makes `expected` correct at batch>1
- `arq_commander.cc:1654` `messages_batch_tx[last_idx].sequence_number |= 0x80;`,
  `:1659/:1702` `messages_tx[last_new_data…].sequence_number |= 0x80;`. Sets bit 7
  on the LAST DATA frame so the RSP derives `expected` (§3.3). Already runs for
  any batch size (it is the "last new-data frame" logic, not batch==1-specific).
  ✓ — but VERIFY in the FTRT sim that a robust 4-8 batch actually carries the EOB
  bit (the robust frame format must have a sequence_number byte; confirm the
  robust DATA header includes it — landmine L5).

### 3.7 `messages_rx_prev` completion / bsi transfer
- `bump_bsi_and_transfer_prev()` (`arq_common.cc:~4086/4106/4128`) bounds loops by
  `data_batch_size`. Parent §3.4: "dead at batch=1". **LANDMINE L1 (cont.)**: at
  batch 4-8 robust this becomes LIVE. The prev-path is used when a partial batch
  is SACK-rescued across a bsi bump (`arq_responder.cc:795-815` prev-delivered
  path). Must verify it bounds correctly at batch 4-8 and that
  `messages_rx_prev[]` is sized ≥8 (it is sized for OFDM batch≥25, so 4-8 fits).
  See `data-flow-messages_rx_prev.md` — UPDATE it: prev-path is no longer dead at
  robust under FIX-A.

### 3.8 Climb-promotion gates (the credit consumer FIX-A must not starve)
- FRAME-UP (`arq_commander.cc:3473+`) advances `consecutive_data_acks` ONLY on a
  CLEAN (all-ones) batch via `promotion_allowed_on_batch(last_batch_fully_acked)`
  (`arq.h:621`); a partial SACK does NOT advance it (`:3498`). Per-tier clean
  threshold: robust=1, OFDM=2 (`arq.h:1907`).
- **THE CENTRAL TENSION**: at batch 4-8 robust, a CLEAN batch now requires ALL
  4-8 frames to survive first-pass (`p^N`). At the FLOOR SNR that may never happen
  → `last_batch_fully_acked` never TRUE → anchor frozen → **climb stalls** — the
  exact Bug-3 failure mode (climb-engine doc §2.1). THIS is why FIX-A gates the
  raise on "rung ALREADY PROVEN" (§5.1): once `last_data_viable_config` ≥ this
  rung AND the clean-streak is established, the climb has already done its job at
  this rung; further climb is gated by the NEXT rung's SNR (which the dwell isn't
  trying to reach). The dwell raise is for a PARKED link, not a climbing one.

---

## §4 VALID STATES (incl. default-init / pre-write values)

| State | `data_batch_size` value | How reached |
|---|---|---|
| ctor / pre-connect | **1** (ctor default, `arq_common.cc:146`) | object construction; `reset_session_state` |
| robust connect (unpinned `-g -R`) | **1** | `load_configuration(ROBUST_0)` §2.2; SACK recompute §2.5 leaves it |
| OFDM connect (`-s 10`) | **≥5** | §2.2 OFDM branch / §2.5 SACK floor |
| robust dwell, rung UNPROVEN (climbing) | **1** (MUST stay) | the climb is still earning `last_data_viable_config` |
| robust dwell, rung PROVEN + parked (FIX-A) | **4-8** | §5.2 post-proof raise (NEW) |
| robust→OFDM promotion | **≥5** | `load_configuration(CONFIG_0)` §2.4 |
| OFDM→robust BREAK | **1** | `load_configuration(ROBUST_0)` §2.2 pins 1 (FIX-A reverts the 4-8) |
| NB robust (any) | **1** (MUST stay) | no SACK suffix (L2); pin 1 always |

**Critical default-init fact (parent §4)**: on the unpinned connect,
`current_configuration == ROBUST_0` (live PHY) but `negotiated_configuration ==
CONFIG_0` (ctor default — connect path NEVER writes it). FIX-A's gate MUST key on
`current_configuration` (like every other robust guard), NEVER on
`negotiated_configuration` — using the latter is the literal 4th-wire-failure bug.

---

## §5 WHAT THE FIX CHANGES

FIX-A does NOT exist as `climb_owns_robust_batch()` (§0.1). The correct,
source-grounded FIX-A is: **introduce a post-proof robust dwell-batch raise,
gated on PROVEN-rung + PARKED intent + WB, propagated symmetrically to the RSP,
and reverted on any climb resume / BREAK / config change.** Four coordinated edits:

### §5.1 The gate predicate — `robust_dwell_batch_eligible()` (NEW pure helper, `arq.h`)

The brief's "active climb intent / this rung not already proven the ceiling" is
INVERTED for the dwell: the dwell raise wants the OPPOSITE of active-climb-intent
— it wants the rung PROVEN and the climb NOT currently trying to leave. Exact
predicate (PURE, read-time, no side effects):

```cpp
// True iff a ROBUST dwell may safely run batch > 1 (the FIX-A relaxation).
// ALL must hold:
//  (a) live PHY is a ROBUST config            — is_robust_config(current_configuration)
//  (b) WB session (SACK suffix exists)        — telecom_system->ack_mfsk.ack_sack_suffix_len() > 0
//                                                (NB: M=8 => suffix_len==0 => no bitmap => MUST stay 1; landmine L2)
//  (c) this rung is PROVEN delivered           — config_ladder_index(current_configuration)
//                                                  <= config_ladder_index(last_data_viable_config)
//      (the anchor has reached or passed this rung => clean batch(es) already
//       confirmed here at batch=1; the climb has earned this rung)
//  (d) the clean streak is ESTABLISHED + parked HERE — clean_batches_config == current_configuration
//      && clean_batches_at_current_config >= ROBUST_DWELL_PROOF_BATCHES (NEW const, e.g. 2)
//      (proves we are PARKED on a sustained-clean robust rung, not transiently)
//  (e) not mid-promotion this poll             — !(promotion_allowed_on_batch(last_batch_fully_acked)
//                                                   && config_ladder_index(current_configuration)+1
//                                                      <= config_ladder_index(supershift_proven_ceiling))
//      i.e. there is NO un-probed higher rung the climb is actively targeting
//      (if a higher rung is still reachable+unproven, KEEP batch=1 so the strict
//       clean credit keeps advancing the anchor — §3.8 central tension)
bool cl_arq_controller::robust_dwell_batch_eligible() const {
    if(!is_robust_config(current_configuration)) return false;
    if(telecom_system->ack_mfsk.ack_sack_suffix_len() <= 0) return false;   // NB guard (L2)
    if(config_ladder_index(current_configuration)
       > config_ladder_index(last_data_viable_config)) return false;        // rung not proven
    if(clean_batches_config != current_configuration) return false;          // streak not here
    if(clean_batches_at_current_config < ROBUST_DWELL_PROOF_BATCHES) return false;
    // (e) higher-rung-reachable check: if the ceiling allows a rung above us, the
    // climb still owns the batch (keep 1). Only park-and-bulk when we're at the
    // proven ceiling for the current channel.
    if(config_ladder_index(current_configuration)
       < config_ladder_index(supershift_proven_ceiling)) return false;
    return true;
}
```

RATIONALE per condition: (a) scope; (b) NB has no SACK bitmap so a partial is
unrecoverable (L2); (c)+(d) the rung is genuinely delivering (the `p^N` clean
target is achievable here, else the anchor wouldn't have reached it and the streak
wouldn't be sustained); (e) the dwell raise is for a link PARKED at its
channel-limited ceiling — if a higher rung is still reachable, keeping batch=1
preserves the strict clean credit that drives the climb upward (§3.8). Reads ONLY
existing members (`current_configuration`, `last_data_viable_config`,
`clean_batches_config`, `clean_batches_at_current_config`, `supershift_proven_ceiling`,
all CMD-side climb state) + one new const. NB: `supershift_proven_ceiling` and the
clean-streak members are CMD-only — so the EVALUATION runs on the CMD; the RSP
gets the resulting batch via the symmetric transport (§5.2), it does NOT
re-evaluate the predicate (it has no climb state). This is identical to how the
Axis-2 batch decision is CMD-only and shipped to the RSP via SET_LINK_PARAMS.

### §5.2 The producer — CMD raises, RSP mirrors (symmetric transport)

Where to fire: in the CMD clean-batch acceptance path, AFTER FRAME-UP has decided
NOT to promote (i.e. the climb is parked), co-located with the existing
`policy_evaluate_axis2()` feed at `arq_commander.cc:2990-2992` BUT gated by
`robust_dwell_batch_eligible()` instead of falling into the Axis-2 robust-guard
early-return. Pseudo-hunk (CMD side):

```cpp
// arq_commander.cc, in the clean-data-ACK handler, after the FRAME-UP decision:
if(robust_dwell_batch_eligible() && data_batch_size != ROBUST_DWELL_BATCH) {
    // PARKED on a proven robust ceiling rung => switch to multi-frame batch +
    // M=16 MFSK SACK selective retransmit. Symmetric transport: a dedicated
    // ROBUST_DWELL_BATCH control frame (NOT SET_LINK_PARAMS — see L4), echoed
    // by the RSP, applied via set_data_batch_size on BOTH sides only after the
    // RSP's ack confirms adoption. Until confirmed, both stay at the current value.
    request_robust_dwell_batch(ROBUST_DWELL_BATCH);   // NEW: queues the symmetric frame
}
```

**The chokepoint relaxation (§2.1)** — the SINGLE structural change that lets the
4-8 value survive `set_data_batch_size()`:

```cpp
// arq_common.cc:595 — was: if(is_robust_config(current_configuration) && data_batch_size != 1)
// FIX-A: clamp to [1 .. ROBUST_DWELL_BATCH_MAX] at robust instead of hard-1.
if(is_robust_config(current_configuration)) {
    int lo = 1, hi = ROBUST_DWELL_BATCH_MAX;   // e.g. 8
    if(data_batch_size < lo || data_batch_size > hi) {
        // clamp into the robust-legal range; log on a real clamp
        int clamped = data_batch_size < lo ? lo : hi;
        if(this->data_batch_size != clamped)
            printf("[BATCH-CHOKEPOINT] robust config %d: clamped batch %d -> %d "
                   "(robust dwell range [%d,%d]; CMD/RSP must agree)\n",
                   current_configuration, data_batch_size, clamped, lo, hi);
        this->data_batch_size = clamped;
        recalculate_ack_timeout_for_batch();   // L3: keep timeout in sync
        return;
    }
    this->data_batch_size = data_batch_size;
    recalculate_ack_timeout_for_batch();        // L3
    return;
}
```

**Why a NEW control frame, not SET_LINK_PARAMS (LANDMINE L4)**: the RSP
SET_LINK_PARAMS handler (§2.7) clamps any requested batch to `[AXIS2_BATCH_FLOOR=10,
AXIS2_BATCH_CEIL=32]`. Routing 4-8 through it yields RSP=10 while CMD=4-8 →
the exact Bug-3 CMD≠RSP mismatch (climb-engine §2.1 step 5). FIX-A MUST use a
transport whose RSP handler applies the value through the (relaxed) chokepoint
WITHOUT the `[10,32]` clamp. Two options: (i) a NEW `ROBUST_DWELL_BATCH` control
op; (ii) reuse SET_LINK_PARAMS but add a robust branch in its RSP handler that
skips the `[10,32]` clamp when `is_robust_config(current_configuration)`. Option
(i) is cleaner (no risk to the OFDM Axis-2 contract); option (ii) is less code.
Either way, the RSP applies via `set_data_batch_size()` so the relaxed chokepoint
re-validates the range, and CMD does NOT advance its own `data_batch_size` until
the RSP's confirmation arrives (atomic adoption — see L6).

### §5.3 The revert — back to 1 on climb-resume / BREAK / config change

The 4-8 dwell batch is ONLY valid while PARKED. It MUST revert to 1 the instant
the link tries to climb again, BREAKs, or changes config:

- **Config change**: `load_configuration()` §2.2 already calls
  `set_data_batch_size(1)` on every robust load → with the relaxed chokepoint
  (§5.2) 1 is in-range → batch returns to 1. ✓ (Verify: a robust→robust reload,
  e.g. ROBUST_0→ROBUST_1 via climb, hits §2.2 and resets to 1. It does:
  `load_configuration` is called on every rung change.)
- **BREAK**: the BREAK recovery path (`arq_commander.cc:202/294`) calls
  `load_configuration(robust_0, PHYSICAL_LAYER_ONLY)` → §2.2 → 1. ✓ (PHYS_ONLY
  still hits the robust branch — it is not level-gated, parent §5.1.)
- **Climb-resume mid-rung (no config change)**: if the channel improves and a
  higher rung becomes reachable while parked, `robust_dwell_batch_eligible()`
  condition (e) flips FALSE (a higher rung is now `< supershift_proven_ceiling`),
  and the CMD must issue a symmetric revert-to-1 BEFORE the next promotion attempt
  (so the strict clean credit can resume driving the anchor). **LANDMINE L7**:
  if the revert is not symmetric (CMD reverts to 1, RSP still at 4-8), the
  all-ones target diverges again. The revert uses the SAME symmetric transport as
  the raise (§5.2), and the CMD must NOT count clean credit on any batch
  transmitted while a raise/revert is in flight (treat in-flight batches as
  non-promoting — set `last_batch_fully_acked=false` for the transition batch).

### §5.4 Per-consumer re-walk verdict (does each consumer still hold?)

| Consumer | Holds at batch 4-8 robust? | Why / what to verify |
|---|---|---|
| §3.1 all-ones target | ✓ IFF symmetric | 4-8 < 30/32 caps; symmetry via §5.2 |
| §3.2 CMD TX loops | ✓ | already `data_batch_size`-parametric |
| §3.3 RSP expected/`>1` gate | ✓ | EOB derives `expected`; `>1` branch now active (intended); `<=1` suppression skipped (intended) |
| §3.4 MFSK SACK suffix | ✓ (WB) / ✗ (NB) | config-agnostic at M=16; NB guard L2 in §5.1(b) |
| §3.5 ACK timeout | ✓ IFF recomputed | L3: `recalculate_ack_timeout_for_batch()` in relaxed chokepoint |
| §3.6 EOB producer | ✓ verify L5 | confirm robust DATA header carries sequence_number for bit-7 |
| §3.7 `messages_rx_prev` | ✓ verify | now live at robust; sized for ≥25, 4-8 fits; update its fact doc |
| §3.8 climb credit | ✓ BY DESIGN | gate (e) keeps batch=1 while climb owns the rung; raise only when parked |

---

## §6 PAIRED REGRESSION TEST (CLAUDE.md §"Cross-layer regression tests")

The brief's `tools/test_robust_batch_sack.py` does not exist (§0.3). Use and
EXTEND the existing in-process `test_climb_engine()` (`arq_commander.cc:6401`,
run via `mercury.exe --test`). Parts A-D are KEPT unchanged (Part D = the
connect-path batch=1 symmetry, `:6635-6712`). ADD **Part D'** (robust dwell raise):

- **D'1 — gate stays CLOSED while climbing**: set `current_configuration=ROBUST_0`,
  `last_data_viable_config=ROBUST_0`, `clean_batches_config=ROBUST_0`,
  `clean_batches_at_current_config=0`, `supershift_proven_ceiling=CONFIG_4`
  (a higher rung reachable). Assert `robust_dwell_batch_eligible()==false` and
  that the CMD raise is NOT issued → batch stays 1. (Fail-before would not apply;
  this is new behavior. The assertion guards condition (c)/(d)/(e).)
- **D'2 — gate OPENS when parked + proven (WB)**: `clean_batches_at_current_config
  = ROBUST_DWELL_PROOF_BATCHES`, `supershift_proven_ceiling=ROBUST_0` (parked at
  ceiling), WB (`ack_sack_suffix_len()>0`). Assert `robust_dwell_batch_eligible()
  ==true`; drive the CMD raise + the RSP mirror through the §5.2 transport; assert
  **CMD batch == RSP batch == ROBUST_DWELL_BATCH (4-8)**, both in `[1,8]`.
- **D'3 — NB stays pinned (L2)**: same as D'2 but `narrowband_enabled=true`
  (so `ack_sack_suffix_len()==0`). Assert `robust_dwell_batch_eligible()==false`,
  batch stays 1 on BOTH sides.
- **D'4 — relaxed chokepoint still clamps out-of-range**: at ROBUST_0,
  `set_data_batch_size(25)` (a rogue robust over-request) → assert clamped to
  `ROBUST_DWELL_BATCH_MAX` (8), NOT 1, NOT 25; and `set_data_batch_size(1)` → 1.
  (This REPLACES old D4 which asserted clamp-to-1; D4's intent — "no path bypasses
  the invariant" — is preserved as "no path escapes `[1,8]`".)
- **D'5 — revert symmetry on config change**: after D'2 (batch=4-8), call
  `load_configuration(ROBUST_1, …)` (a climb step). Assert batch reset to 1 on the
  side that reloaded; assert a robust→robust reload does NOT leave a stale 4-8.
- **D'6 — OFDM unchanged**: re-run old D5 (CONFIG_10 scales to ≥5) — FIX-A must
  not touch the OFDM path.

**Wire test (FTRT sim then HW)**: the brief's "re-run with `-g`" maps to running
the unpinned `-g -R` cascade in the FTRT sim at a deep-SNR cell where ROBUST_0 is
the channel ceiling, with a multi-frame payload, and asserting BOTH:
(1) **delivered bytes/min RISES** vs the batch=1 baseline (the floor-throughput win); and
(2) **CMD `data_batch_size` == RSP `data_batch_size`** at every poll (the symmetry
invariant — assert via the `[BATCH-CHOKEPOINT]` / `[SACK]` / `[RSP-MFSK-SACK]
nframes=` log fields the code already emits on both sides). Then HW via
`tools/muething_throughput.py` (compare `muething_results.json` vs
`.BASELINE_*.json`); SNR3k = WGN_label + 2.4.

**Why local is NECESSARY-but-INSUFFICIENT** (parent §6): the 4 prior wire
failures all PASSED local and FAILED the wire because they hand-set state and
never modeled the connect-path default-init NOR the live multi-rung climb where
rung N+1's promotion depends on rung N's batch=1 clean credit. D'1 + the FTRT
symmetry assertion close exactly those gaps; HW is still required for the floor
SNR `p^N` behavior (the sim's channel model must reproduce the per-frame error
rate or the throughput-rise assertion is not trustworthy).

---

## §7 LANDMINES (consolidated — every one cited)

- **L1** (`arq_responder.cc:1462`, `:1504`): the `data_batch_size > 1` partial
  branch + the `<=1` SACK-suppression. At robust 4-8 the `>1` branch goes LIVE
  (intended) and the `<=1` suppression is SKIPPED (intended). No code change, but
  the `messages_rx_prev` / partial path (parent §3.4 "dead at batch=1") is now
  EXERCISED at ROBUST — verify its robust-tier correctness; update
  `data-flow-messages_rx_prev.md`.
- **L2** (`mfsk.h:194`, `telecom_system.cc:5232`, `arq_responder.cc:1690`): NB
  robust has `ack_sack_suffix_len()==0` (M=8) → NO SACK bitmap → a multi-frame
  partial is UNRECOVERABLE. The §5.1(b) guard MUST keep NB robust at batch=1.
- **L3** (`arq_common.cc:763`, `:1413+`): ACK-timeout scales by `data_batch_size`.
  The relaxed chokepoint MUST call `recalculate_ack_timeout_for_batch()` on every
  robust raise AND revert, on BOTH sides, or CMD times out mid-batch.
- **L4** (`arq_responder.cc:2609-2615`): SET_LINK_PARAMS clamps to `[10,32]`. Do
  NOT route the 4-8 robust raise through it (→ RSP=10 ≠ CMD=4-8 = Bug 3). Use a
  dedicated robust-dwell op OR a robust branch in the SET_LINK_PARAMS RSP handler
  that skips the `[10,32]` clamp.
- **L5** (`arq_commander.cc:1654/1702`): confirm the ROBUST DATA frame header
  carries a `sequence_number` byte so the EOB bit-7 marker (§3.6) actually rides
  the last robust frame; if robust frames don't carry it, the RSP `expected`
  count (§3.3) defaults to `data_batch_size` and a short final batch under-counts.
  VERIFY in the FTRT sim.
- **L6 (atomic adoption)**: CMD must NOT advance its own `data_batch_size` until
  the RSP confirms the new value (the all-ones target diverges during the
  in-flight window). The raise transport must be confirm-then-apply on the CMD
  side (the RSP applies on receipt; CMD applies on the RSP's ACK), mirroring how
  Axis-2 SET_LINK_PARAMS is CMD-decided / RSP-applied.
- **L7 (revert symmetry)**: a CMD-only revert to 1 (climb resume) without the
  symmetric RSP revert re-creates the divergence. Use the same transport for
  revert as for raise (§5.3); mark the transition batch non-promoting.
- **L8 (climb starvation — the central tension)**: lifting robust batch while the
  climb still owns the rung freezes the anchor (`p^N` clean target unachievable at
  the floor → no clean credit → no promotion) — the literal Bug-3 dormancy
  (climb-engine §2.1). The §5.1(c)(d)(e) gate is what prevents this; it is the
  load-bearing part of FIX-A, not the chokepoint relaxation.

---

## §8 Open questions [?]

- **[?]** Exact value of `ROBUST_DWELL_BATCH` (4 vs 6 vs 8) and
  `ROBUST_DWELL_PROOF_BATCHES`: choose EMPIRICALLY in the FTRT sim by sweeping
  delivered-B/min vs batch at a fixed deep-SNR cell where ROBUST_0 is the ceiling.
  Larger batch amortizes ACK turnaround MORE but raises the `p^N` full-batch-clean
  risk; the optimum depends on the floor per-frame error rate. Do NOT pick a magic
  number without the measured sweep (CLAUDE.md §1 / "magic numbers without
  measurement basis").
- **[?]** Does `supershift_proven_ceiling` reliably equal the channel's true
  ceiling at a deep-SNR cell, or does it lag? If it lags, condition (e) may keep
  the gate closed (batch=1) when the link is genuinely parked. Cross-check against
  the `last_data_viable_config` anchor in the FTRT sim; if (e) is too conservative,
  relax it to "no clean-credit advance in the last K batches" (a parked-detector)
  rather than a ceiling comparison.
- **[?]** Whether the MFSK SACK suffix's 30-bit bitmap + the CRC12 false-accept
  guard remain reliable at the FLOOR SNR with 4-8 outstanding frames (the suffix
  itself must decode for the selective retransmit to work). The partial path
  comment claims "ROBUST_0-grade detection at any config" — VERIFY at the floor
  in the FTRT sim, since a suffix-decode failure on a partial batch falls back to
  the legacy MFSK pattern (clean-only) and the partial is lost.

---

## §9 Related fact documents
- `fact-documents/data-flow-batch-size.md` — canonical `data_batch_size` owner;
  the 4-wire-failure invariant + the chokepoint (this doc is the FIX-A delta).
- `fact-documents/gearshift-climb-engine.md` — the climb promotion gates, the
  anchor (`last_data_viable_config`), Bug 3 DORMANCY (§2) = the exact failure FIX-A
  must avoid; `clean_batches_*`, `supershift_proven_ceiling` semantics.
- `fact-documents/data-flow-messages_rx_prev.md` — the prev-storage state; UPDATE:
  it is no longer dead at robust once FIX-A allows robust batch>1 (L1).
- `fact-documents/mfsk-robust-ack.md` — the M=16 ACK+SACK suffix transport
  (`bsi:8 | bitmap:32 | crc12:12`), the config-agnostic / WB-only facts (§3.4, L2).

---

## §10 AS-BUILT (branch `fix/robust-dwell-batch`, off `feat/sim-clock`, 2026-06-03)

The implementation follows §5 exactly, with the design choices §5/§8 left open
resolved as below. All line numbers are on `fix/robust-dwell-batch` (which is ahead
of `0c75cc2`; the chokepoint that §2.1 cited at `:595` is now `arq_common.cc:636`).

### §10.1 The edits (every file:line)

1. **`include/common/common_defines.h`** (after `CONNECT_PREAMBLE_REPS_PROD`):
   `ROBUST_DWELL_BATCH_MAX 8`, `ROBUST_DWELL_BATCH 4`, `ROBUST_DWELL_PROOF_BATCHES 2`
   (OR-5 — starting values; SWEEP recorded in §10.5).
2. **`include/datalink_layer/datalink_defines.h`** (the freed `0x44` slot):
   `#define ROBUST_DWELL_BATCH_OP 0x44` — the DEDICATED control op (OR-2/L4: NOT
   SET_LINK_PARAMS, whose RSP handler clamps to `[10,32]`). Wire payload
   `[op | batch:u8 | crc8:u8]`, CRC8 over batch only.
3. **`include/datalink_layer/arq.h`**:
   - PURE core `robust_dwell_batch_eligible_core(current,anchor,streak_cfg,
     clean_streak,proven_ceiling,suffix_capable)` (after `fast_probe_clean_streak`)
     — the full conjunct (a)-(e) of §5.1, fully unit-testable with no live
     telecom_system. Member wrapper `robust_dwell_batch_eligible() const` supplies the
     live climb state + `suffix_capable = telecom_system->ack_mfsk.ack_sack_suffix_len()>0`.
   - members `int pending_robust_dwell_batch; bool robust_dwell_batch_active;` +
     `bool evaluate_robust_dwell_batch();` (after `pending_link_params_*`).
4. **`source/datalink_layer/arq_common.cc`**:
   - ctor init (after `pending_link_params_*=-1`): `pending_robust_dwell_batch=-1;
     robust_dwell_batch_active=false;`.
   - `set_data_batch_size()` chokepoint (was force-to-1, now range-clamp
     `[1..ROBUST_DWELL_BATCH_MAX]`) + `recalculate_ack_timeout_for_batch()` on an
     actual change to/from a multi-frame batch (L3). The seed-to-1 (prev==1)
     deliberately does NOT recompute (message_transmission_time_ms not yet current
     inside load_configuration).
   - `load_configuration()` robust branch (after `set_data_batch_size(1)`):
     `robust_dwell_batch_active=false` — the config-change revert leg (§5.3).
   - `reset_session_state()`: reset both new members (per-session revert).
5. **`source/datalink_layer/arq_commander.cc`**:
   - `add_message_control()`: new `else if(code==ROBUST_DWELL_BATCH_OP)` encoder
     (modeled on SET_LINK_PARAMS, clamp `[1..ROBUST_DWELL_BATCH_MAX]`, null-guard for
     synthetic-fire).
   - `evaluate_robust_dwell_batch()` (new, after `policy_evaluate_axis2`): the CMD
     raise/revert decision; applies locally + sends the op; returns true iff it
     queued a control frame.
   - call site: `process_messages_rx_acks_data()`, right after the FRAME-UP ceiling
     scope closes and BEFORE `connection_status=TRANSMITTING_DATA` — i.e. the PARKED
     point (FRAME-UP `return`s on a promotion, so we reach here only when the climb
     declined to climb this poll). `if(evaluate_robust_dwell_batch()) return;` so the
     op TX supersedes the DATA transition.
   - `test_climb_engine()`: old D4 updated (clamp-to-MAX, not clamp-to-1) + new
     Parts D'1–D'6 (+ D'2b/D'2c/D'3b).
6. **`source/datalink_layer/arq_responder.cc`**:
   - new `else if(... && code==ROBUST_DWELL_BATCH_OP)` RSP handler (after the
     SET_LINK_PARAMS block): CRC8-check, apply via `set_data_batch_size()` with NO
     `[10,32]` clamp (the relaxed robust chokepoint re-validates `[1..MAX]`),
     `connection_status=ACKNOWLEDGING_CONTROL` to ACK.

### §10.2 Resolved design choices (what §5/§8 left open)

- **Transport (L6 atomicity)**: chose the PROVEN Axis-2 pattern over a
  confirm-then-apply protocol — CMD applies locally NOW and the
  `add_message_control()`→`TRANSMITTING_CONTROL`→`RECEIVING_ACKS_CONTROL` handshake
  gates the next DATA TX, so the RSP adopts + ACKs BEFORE any DATA batch is built at
  the new size. No DATA batch is ever built with CMD≠RSP. The transition batch is
  marked `last_batch_fully_acked=false` (L7) so no stray clean-credit fires mid-change.
- **Condition (e) (§8 [?] "does proven_ceiling lag")**: kept the §5.1 ceiling
  comparison but hardened the `proven_ceiling < 0` case to return FALSE (a fresh
  connect with no bounded ceiling is NOT a parked link → keep batch=1). If sim/HW
  shows (e) is too conservative (gate stays shut on a genuinely parked link), the
  §8 fallback is a "no clean-credit advance in the last K batches" parked-detector;
  not needed unless evidence demands it.
- **Extra gate at the call site**: `evaluate_robust_dwell_batch()` additionally
  requires `sack_v2_enabled` (a robust partial is only recoverable when the v2
  selective-retransmit path is on) before raising — belt-and-suspenders over (b).

### §10.3 Per-consumer re-walk — confirmed AS-BUILT
The §5.4 table holds as built. §3.3's `>1` partial branch (`arq_responder.cc`
`if(data_batch_size>1 …)`) goes LIVE and the `<=1` suppression is SKIPPED at a robust
dwell — both intended; no code change needed (the path was already batch-parametric).
`data-flow-messages_rx_prev.md` §4.4 updated (prev-path live at robust>1; sized to
nMessages=120, 4-8 fits).

### §10.4 Unit-test results (`mercury.exe --test-climb-engine`)
ALL PASS. Parts D'1–D'6 (+D'2b/c, D'3b) green. FAIL-BEFORE proven by reverting the
(b) suffix guard AND (e) ceiling guard in the PURE core → D'1 FAILS (got=1 want=0,
the OR-1/L8 regression — gate opens while a higher rung is reachable) AND D'3 FAILS
(got=1 want=0, the L2 regression — NB lifts batch with no SACK bitmap); restoring the
guards → both PASS. `--test-sim-clock` 7/7. Full `--test` unchanged.

### §10.5 OR-5 sweep [?] — STARTING values; sim CONFIRMS the mechanism, B/min RISE needs HW
`ROBUST_DWELL_BATCH=4`, `ROBUST_DWELL_BATCH_MAX=8`, `ROBUST_DWELL_PROOF_BATCHES=2`
are STARTING values. **The FTRT sim CONFIRMED the gate + transport + CMD/RSP-symmetry
mechanism fires correctly** but **could NOT produce a STABLE settled-ROBUST data dwell**
to sweep delivered-B/min vs batch — the §8 [?] is now answered NEGATIVELY for the sim:

- Across cells WGN:-6/-8/-9 (`--start-cfg 100 --robust`, wgn, 150-300s), the dwell
  gate OPENED exactly when proven+parked (e.g. WGN:-6: `[ROBUST-DWELL] RAISE robust
  dwell batch 1 -> 4 at config 102 (eligible=1 anchor=102 streak_cfg=102 streak=2
  ceiling=101)`) and the RSP MIRRORED it (`[RSP-ROBUST-DWELL] APPLIED batch 1 -> 4
  (rx=4 crc8=0x8c)`) — **CMD batch == RSP batch == 4, identical crc8**. No CMD≠RSP
  mismatch was observed in any run. The dedicated op + relaxed chokepoint + symmetric
  adoption WORK end-to-end in-sim.
- BUT no cell gave a STABLE parked window: the gate only opens at the TOP robust rung
  the climb reaches (cond (c)+(e)), whose up-probe is always CONFIG_0 (OFDM). The
  sim's documented OFDM data-ACK turnaround coupling defect (`sim-arq-channel.md` §10)
  collapses every CONFIG_0 probe → `FRAME UP DATA FAILED (pat): config 0 -> BREAK to
  102` → the link oscillates ROBUST_2↔CONFIG_0 and never sustains the dwell. Deeper
  cells (WGN:-9/-10) sit at the cliff edge and oscillate WITHIN robust
  (ROBUST_0↔ROBUST_2) without parking. Either way: the dwell fires but is disrupted
  within a batch or two — no multi-batch parked window to measure a B/min delta.
- **CONCLUSION**: the synthetic-fire unit tests (Parts D'1–D'6, FAIL-BEFORE→PASS-AFTER
  on D'1/D'3) are the CORRECTNESS proof; the FTRT sim is the MECHANISM proof
  (gate+op+symmetry fire correctly, no CMD/RSP divergence). The delivered-rate RISE
  (batch-4 B/min > batch-1 B/min at a settled ROBUST dwell) needs HW (a deep-but-clean
  cell where the robust ceiling holds and the OFDM up-probe genuinely fails on RF, so
  the ceiling latches and the link PARKS — exactly the condition the sim's OFDM
  defect prevents). The OR-5 sweep of ROBUST_DWELL_BATCH/MAX/PROOF_BATCHES is
  therefore DEFERRED to the HW muething run; the values above are conservative
  starting points (batch 4 amortizes ACK turnaround ~4× while keeping the p^N
  full-batch-clean risk modest; MAX 8 caps the SACK bitmap well under the 30/32 caps;
  PROOF_BATCHES 2 matches the OFDM sustained-anchor N).

# Data-Flow Audit: robust-tier ARQ batch sizing + SACK/ACK state

**Status**: Authoritative as of 2026-06-02 on `win/incr2-robust-arq-batch`
(branched off `monitor @0c75cc2`). Built for WIN-CAMPAIGN increment 2 (lift the
robust-tier ARQ chokepoint: batch ≥ 2 + SACK selective-retransmit in the robust
tier, to take delivered net @ −10 from ~53-75 toward the ~95 genie).

**Scope**: the shared state that gates whether the **robust tier** (ROBUST_0/1/2
= configs 100/101/102, `is_robust_config()`) runs stop-and-wait (batch=1,
pattern-ACK) or selective-retransmit (batch≥2, SACK). This document supplements
`data-flow-batch-size.md` (the owner of the `data_batch_size` invariant) and
focuses on the **robust-specific** producers/consumers and the SACK/ACK
transports.

**Companion docs**: `data-flow-batch-size.md` (the batch=1 invariant + its
4-wire-failure history), `mfsk-robust-ack.md` (the MFSK ACK+SACK suffix
transport), `gearshift-climb-engine.md` (the climb promotion gates that the
batch=1 pin protects), `robust2-minus10-rate-win.md` (§5/§6 list "batch ≥ 2 for
ROBUST_2" as INCR-3, the increment this doc supports).

---

## §1 The §1 root-cause finding (safeguard vs unfinished feature)

**Question (WIN-CAMPAIGN incr2, CLAUDE.md §1)**: is the robust tier
batch=1 + no-SACK a **deliberate safeguard** (deep-SNR reliability needs it) or
an **unfinished feature** (SACK built for OFDM, never extended to robust)?

**Answer: BOTH, in different layers — and the distinction is load-bearing.**

### §1.1 The SACK selective-retransmit transport is an UNFINISHED FEATURE at robust (case b)

The entire SACK selective-retransmit machinery — transport, CRC, retransmit
queue, CMD partial handler, RSP partial emit — is **config-agnostic and
WB-capable**, and **already fires at any WB config** the moment batch>1. It is
NOT excluded from robust by any reliability gate; it is dormant at robust ONLY
because batch is pinned to 1 (a 1-frame batch is trivially all-or-nothing, so
there is nothing to selectively retransmit). Evidence (executing-code cites):

- **The MFSK ACK+SACK suffix transport runs at a FIXED M=16, config-independent**:
  `telecom_system.cc:3095` ("Always use dedicated ack_mfsk (M=16, nStreams=1) —
  config-independent"). `ack_sack_suffix_len() = (M>=16) ? 13 : 0`
  (`mfsk.h:194`). So the 13-tone SACK suffix (carrying `[bsi:8 | bitmap:30 |
  crc12:12]`) is available at **every WB data config including ROBUST_0/1/2** —
  the ACK codec does not switch to the data PHY. (ROBUST_0 M=32, ROBUST_1/2
  M=16×2 are all WB; the ACK codec is its own M=16 regardless.)
- **The CMD-side SACK probe is NOT config-gated**: `arq_commander.cc:2473-2474`
  `sack_window_open = sack_enabled && data_ack_received==NO && elapsed >
  ack_pattern_time_ms`. Inside it (`:2492-2677`), the MFSK-suffix decode
  (`decode_ack_sack_from_passband`, CRC12-verified) populates `sack_bitmap[]`,
  sets `sack_detected`, and feeds the **same** retransmit-queue/Axis blocks the
  OFDM `SACK_RSP` path uses. Gate is `sack_v2_enabled && axis3_sack_mode!=OFF &&
  ack_sack_suffix_len()>0` — never `!is_robust_config`.
- **The RSP-side partial-SACK emit IS batch-gated, not config-gated**:
  `arq_responder.cc:1462` `if(data_batch_size > 1 && rx_received < expected
  ...)` and `:1504-1515` `else if(data_batch_size <= 1) { ... "SACK_RSP
  suppressed (multi-frame batches only)" }`. The comment (`:1506-1510`) says
  SACK is "meaningless on a single-frame batch (which is the default in MFSK /
  ROBUST modes)". So **SACK is suppressed BECAUSE batch=1, not the other way
  around** — confirming the transport itself is config-agnostic.
- **The MFSK partial SACK emit at robust is already written**: `arq_responder.cc:
  1556-1576` packs the per-frame bitmap into a u32 and calls
  `send_mfsk_ack_sack(sacked_bsi, bitmap_u32)` "No optimizer-territory gate: the
  pattern correlator gives this path ROBUST_0-grade detection at any config,
  which is exactly where partial-batch recovery matters most" (`:1549-1552`). The
  code anticipates robust; only batch=1 keeps it dormant.

**Conclusion (b)**: SACK at robust is a built-but-never-enabled feature, gated
solely behind batch=1. The campaign's own `robust2-minus10-rate-win.md` §5/§6
lists "batch ≥ 2 for ROBUST_2 (INCR-3)" as a planned, not-yet-done increment.

### §1.2 The batch=1 PIN is a DELIBERATE SAFEGUARD — but a CONDITIONAL one (case a, scoped)

`set_data_batch_size()` (`arq_common.cc:573-619`) force-clamps robust→batch=1 at
the **sole production setter** (the "BATCH-CHOKEPOINT"), and
`load_configuration()` pins robust→1 at `arq_common.cc:1332-1337`. Per
`data-flow-batch-size.md` §1/§4 this is a hard-won safeguard against a SPECIFIC
failure: **CMD/RSP all-ones-target divergence**. Both sides independently compute
the clean-batch promotion target as `(1<<data_batch_size)-1`
(`arq_commander.cc:131-134`, `:2602-2605`; `arq_responder.cc:801`, `:1711`). If
CMD batch ≠ RSP batch the targets never match → no clean credit → **the gearshift
climb cannot start or advance**. Four successive climb-fix wire failures were all
this class (`data-flow-batch-size.md` §4).

**The crucial nuance this audit adds**: the batch=1 pin protects the **gearshift
CLIMB**, not PHY reliability. At the MFSK floor SNR a clean *all-ones* multi-frame
batch never arrives first-pass, so a climbing rung needs batch=1 to ever see its
first clean batch and promote (`data-flow-batch-size.md` §1: "one delivered MFSK
frame == a clean all-ones batch ... STRICTER than batch≥5"). **This concern
applies ONLY while the climb is active.** When the link is **PINNED** at a robust
config (`gear_shift_on==NO`, e.g. the −10 rate-win runs `-s 102` with no `-g`),
there is **no promotion gate at all** — the all-ones-target symmetry that
motivated batch=1 is moot, and batch≥2 + SACK is the correct delivered-rate
choice (a single MFSK frame per batch caps delivered throughput far below the
PHY rate; `robust2-minus10-rate-win.md` §5).

So the safeguard is **real but conditional**: batch=1 is mandatory *while
climbing*, optional *while pinned*. The safe lift is to allow robust batch≥2 ONLY
when the climb is not the consumer — see §5.

---

## §2 Producers — every code path that writes `data_batch_size` (robust-relevant)

(Full producer list in `data-flow-batch-size.md` §2; here, robust-relevant ones
and how each interacts with the lift.)

| # | Site | Robust behavior today | Under the lift |
|---|---|---|---|
| 2.1 | `load_configuration()` robust branch `arq_common.cc:1332-1337` | `set_data_batch_size(1)` unconditionally | unchanged — still requests 1; the chokepoint decides the final value (§5) |
| 2.2 | `set_data_batch_size()` chokepoint `arq_common.cc:595-606` | clamps robust→1 always | clamps robust→1 ONLY when climb-relevant (§5); allows the pinned robust dwell batch otherwise |
| 2.3 | `sack_negotiated_recompute_batch()` `arq_common.cc:741-765` | `!is_robust_config` → skips robust (batch stays 1) | sets the pinned robust dwell batch when pinned (§5); still skips the ≥5 OFDM floor at robust |
| 2.4 | CMD/RSP TEST_CONNECTION(_ACK) call §2.3 | both run identical predicate on `current_configuration` | both run identical predicate → symmetric by construction |
| 2.5 | `policy_evaluate_axis2()` `arq_commander.cc` (Axis-2) | `if(is_robust_config) return;` early-out | UNCHANGED — Axis-2 never touches robust batch (keep the guard) |
| 2.6 | RSP `SET_LINK_PARAMS` apply `arq_responder.cc:2669` | clamps to [10,32], robust never receives one | UNCHANGED — robust dwell batch is NOT carried by SET_LINK_PARAMS (it rides OFDM PHY, undecodable at the robust floor) |

---

## §3 Consumers — every code path that reads `data_batch_size` at robust

| # | Site | batch=1 assumption | Holds at robust batch≥2? |
|---|---|---|---|
| 3.1 | Clean all-ones target `arq_commander.cc:131-134`,`:2602-2605`; `arq_responder.cc:801`,`:1711` | target = `(1<<batch)-1` must match CMD↔RSP | YES iff CMD batch == RSP batch (the §5 lift keeps them equal by construction) |
| 3.2 | CMD block-build / TX loop `i<data_batch_size` (`arq_common.cc` data producer) | none — works for any batch | YES |
| 3.3 | RSP ACK-GATE partial branch `arq_responder.cc:1462`,`:1504` | `data_batch_size>1` ENABLES the partial-SACK emit | YES — this is exactly the path we WANT to enable |
| 3.4 | CMD SACK probe / retransmit queue `arq_commander.cc:2492-2677` | none — config-agnostic | YES (transport is M=16, config-independent) |
| 3.5 | `messages_rx_prev` completion `arq_common.cc:4086+`, `bump_bsi_and_transfer_prev()` | dead at batch=1; active at batch≥2 | Active — must verify prev-storage works at robust (see §4 invariant 4) |
| 3.6 | ACK-timeout math `arq_common.cc:657/674/...` scales by `data_batch_size` | larger batch → longer timeout | YES — `recalculate_ack_timeout_for_batch()` runs in §2.3 helper |
| 3.7 | Gearshift climb promotion `gearshift-climb-engine.md` | NEEDS batch=1 to get a clean batch at the floor | **NO at batch≥2 while climbing** — this is the safeguard. The §5 lift keeps batch=1 whenever the climb is the consumer. |

---

## §4 Invariants the consumers assume (and how the lift preserves each)

1. **CMD batch == RSP batch (all-ones symmetry, §3.1).** The whole
   `data-flow-batch-size.md` doc exists for this. **The lift MUST keep CMD and
   RSP batch identical at robust.** A per-peer CLI flag (`gear_shift_on` is set
   locally from `-g` at `main.cc:2356`) is NOT a safe discriminator on its own —
   if one operator runs `-g` and the peer runs pinned, the two would hold
   different `gear_shift_on` and diverge → the exact 4-wire-failure mode. The
   lift therefore keys the robust dwell batch on a value **both sides compute
   identically at connect**, run through the **shared** `sack_negotiated_
   recompute_batch()` helper (§2.3/§2.4) so CMD and RSP execute textually
   identical code — symmetric by construction, not by operator discipline.
2. **The climb gets a clean batch at the floor (§3.7).** Preserved by keeping
   batch=1 whenever the climb can promote (`gear_shift_on==YES` AND the live
   config is a robust climb rung). The dwell batch≥2 applies only to the
   **pinned** robust case where no promotion gate exists.
3. **ACK-timeout scales with batch (§3.6).** `recalculate_ack_timeout_for_batch()`
   already runs unconditionally in the §2.3 helper; a larger robust batch widens
   the data-ACK timeout correctly. Verify the robust ACK turnaround (MFSK ACK
   ~750ms-1s) fits within `(batch+2)*msg_time + ...` — at ROBUST_2 msg_time is
   large (~4-7s/frame) so batch=2 timeout is generous.
4. **prev-storage correctness (§3.5).** At batch≥2 the retransmit/prev-storage
   path (`messages_rx_prev`, `bump_bsi_and_transfer_prev`) becomes live at
   robust. `data-flow-messages_rx_prev.md` covers this state; the path is
   transport-agnostic (it operates on `messages_rx_prev[]` indices, not PHY). The
   clean-prev-delivered ACK already has a robust-capable MFSK-suffix emit
   (`arq_responder.cc:805`). The comment at `:788` ("robust is batch=1 and never
   uses the prev path") becomes stale under the lift and is corrected there.
5. **Uncommon paths (BREAK, config switch, session reset).**
   - **BREAK → ROBUST_0** (`load_configuration(ROBUST_0)`): sets
     `current_configuration=ROBUST_0` then requests batch=1 (§2.1). If a BREAK
     fires during a pinned robust dwell, `gear_shift_on` is still NO, so the
     dwell batch reapplies symmetrically on both sides at the next recompute —
     but a BREAK is a recovery event; batch reverting to 1 transiently is safe
     (stop-and-wait is the conservative state). The chokepoint guarantees no
     stray batch≠agreed value.
   - **robust→OFDM promotion**: only happens with `gear_shift_on==YES` (climb),
     where the dwell batch never applied (batch was 1). OFDM scaling unchanged.
   - **session reset / reconnect**: `sack_negotiated_recompute_batch()` re-runs
     at the new TEST_CONNECTION on both sides → re-establishes the agreed batch
     symmetrically.

---

## §5 What the fix changes (the safe, minimal lift)

**Design**: introduce a single config-symmetric robust dwell batch, applied ONLY
when the climb is not the consumer, through the existing shared helper so CMD and
RSP cannot diverge.

1. **New member `robust_dwell_batch` (default 1)**, settable via CLI
   (`--robust-batch N`, default 1 = byte-identical to today). It is a config
   constant on each peer; for a real session both operators set the same value
   (or leave the default). Because it is applied through the **shared** recompute
   helper keyed on `current_configuration` (symmetric at connect), and because
   batch=1 is the default, the **default build is byte-identical to monitor**.
2. **`set_data_batch_size()` chokepoint** (`arq_common.cc:595`): change the
   robust clamp from "always 1" to "clamp to 1 ONLY when the climb is active"
   — i.e. `is_robust_config(current_configuration) && climb_owns_batch()` →
   force 1. `climb_owns_batch()` = `gear_shift_on==YES && turboshift_phase !=
   TURBO_DONE-equivalent for the rung` (conservatively: `gear_shift_on==YES`).
   When the climb is NOT active (pinned), allow the requested robust batch
   (which is `robust_dwell_batch`, ≥1).
3. **`sack_negotiated_recompute_batch()`** (`arq_common.cc:750`): in the robust
   branch (currently skipped), when not climbing, `set_data_batch_size(
   robust_dwell_batch)` so both sides adopt the same dwell batch at connect.
4. **`load_configuration()` robust pin** (`arq_common.cc:1334`): request
   `robust_dwell_batch` instead of hard 1 (the chokepoint still forces 1 if the
   climb owns the batch).
5. **KEEP** the Axis-2 robust early-out (§2.5) and SET_LINK_PARAMS clamp (§2.6)
   unchanged — robust dwell batch is NOT mid-session-adaptive and NOT carried by
   the OFDM SET_LINK_PARAMS frame.

**Why this is safe at every transition**: see §4. The default (`robust_dwell_batch
=1`) reproduces today's behavior bit-for-bit. With the climb active, robust is
still pinned to 1 (invariant 2 preserved). Only the pinned robust dwell — where no
promotion gate exists and CMD/RSP run identical recompute code — opens batch≥2 +
the already-built SACK selective-retransmit path.

### §5.1 The remaining cross-peer agreement caveat (HONEST open item)

`robust_dwell_batch` is a per-peer CLI constant, not wire-negotiated. If two
operators set DIFFERENT non-default values, they diverge (invariant 1 violated).
This is acceptable for the SIM increment and the pinned −10 rate-win (single
controlled testbed, both peers configured together). **A production deployment
that lets the two sides pick different robust batches REQUIRES wire negotiation**
— the right vehicle is the TEST_CONNECTION capability/SNR exchange
(`arq_responder.cc:2167-2174` has spare capability bits), NOT the OFDM-PHY
SET_LINK_PARAMS (undecodable at the robust floor). That negotiation is a separate,
larger increment (touches the connect handshake incr1/establishment also edits)
and is flagged for the integration phase. For incr2 the default stays 1
(byte-identical) and the lift is validated pinned, where symmetry is guaranteed
by identical configuration.

---

## §6 Paired regression test (CLAUDE.md §"Cross-layer regression tests")

`tools/test_robust_batch_sack.py` — a loopback/sim throughput A/B on **ROBUST_2**
(an EXISTING robust mode, independent of incr1's GF16-RA PHY) under induced frame
loss:

- **Fail-before** (`--robust-batch 1`, today's default): stop-and-wait — each
  lost frame costs a full ACK-timeout + retransmit round-trip. Measured delivered
  throughput = baseline.
- **Pass-after** (`--robust-batch N≥2`): selective-retransmit — the RSP SACKs the
  partial, CMD retransmits only the missing frame(s). Measured delivered
  throughput STRICTLY GREATER under the same loss.

Plus an in-process `--test` assertion (`test_robust_batch_chokepoint` in
`arq_commander.cc`) that drives the chokepoint + recompute helper through:
- pinned robust (`gear_shift_on==NO`, robust_dwell_batch=N): CMD batch == RSP
  batch == N;
- climbing robust (`gear_shift_on==YES`): CMD batch == RSP batch == 1 (climb
  safeguard preserved — the fail-before of the climb-protection half);
- OFDM (`current_configuration=CONFIG_10`): both scale to ≥5 (unchanged).

---

## §7 Related fact documents
- `data-flow-batch-size.md` — the `data_batch_size` invariant owner; §4 the
  4-wire-failure history. This doc's §5 lift updates that doc's §1 note that
  robust is "always 1" → "always 1 while climbing; `robust_dwell_batch` while
  pinned".
- `mfsk-robust-ack.md` — the MFSK ACK+SACK suffix transport (the config-agnostic
  M=16 codec that makes SACK-at-robust physically work).
- `gearshift-climb-engine.md` — the climb promotion gates the batch=1 pin
  protects (the conditional safeguard, §1.2).
- `robust2-minus10-rate-win.md` — §5/§6: "batch ≥ 2 for ROBUST_2 (INCR-3)"; the
  increment this doc supports. PHY cliff measured −8 (pinned), so the dwell
  batch≥2 operates at ≥ −8 SNR3k.

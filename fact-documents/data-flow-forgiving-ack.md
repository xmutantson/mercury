# Data-Flow Audit: Forgiving-ACK (Tier 1 — decouple forward-healthy reverse-ACK miss from the BREAK→ROBUST_0 demote; Tier 2 — cumulative-n_r self-healing SACK)

**Status**: Authoritative as of 2026-06-13 on `feat/forgiving-ack-tier1` (off
`integ/overnight-shippable` @ `f359f01`). Tier 1 is the standalone root fix; the
**TIER 2** section below (cumulative-n_r status report, built on top of the Tier-1
commit `55957ea`) is also authoritative. Every change to `emergency_nack_count`,
the BREAK trigger, `break_drop_step`, `breaks_since_last_data_success`, the
forward-health latch / `forgiving_ack_consec_forgiven`, OR to the SACK `bsi` field
SEMANTICS / `rsp_last_delivered_batch_seq_id` / the `CAP_CUMULATIVE_ACK`
negotiation MUST update this document.

**Context**: This audit covers the **Tier 1 root fix** of the turnaround-cascade
campaign (judge verdict 2026-06-13, `tasks/w1sx2th28.output` result.judge): a
*forward-healthy* reverse-ACK miss must NOT feed `emergency_nack_count` →
BREAK → ROBUST_0. The gap from the ~0.69 structural active-fraction ceiling to
the measured 0.08–0.26 is a SINGLE mechanism — ONE dropped forward-healthy
reverse-ACK detonates a 100–365 s BREAK→ROBUST_0 robust crawl
(`_research/ARQ_TIME_BUDGET_AUDIT.md` §5/§8.3). The fix makes such a miss CHEAP
(one same-gear re-air) so the reverse-ACK is no longer load-bearing.

This change touches **shared ARQ state that crosses ≥2 layers** (PHY reverse-ACK
correlator result → ARQ failure-taxonomy → climb-engine demote anchor), so per
CLAUDE.md §"Cross-Layer Data-Flow Audits" it required this audit BEFORE wiring.

**The LIFE-CRITICAL safety constraint**: the decouple must apply ONLY to a
forward-healthy reverse-ACK miss. A GENUINE link death (forward also dead / link
lost / repeated misses with no forward health) MUST STILL BREAK→ROBUST_0 — the
safety net must NOT be disabled. The bound `forgiving_ack_consec_forgiven` (§5)
guarantees the link cannot loop forever re-airing into a dead link.

**Default-off**: the whole tier is gated by env `MERCURY_FORGIVING_ACK`
(default-off ≡ byte-identical to the monitor base). The fail-before stub is
`-DFORGIVING_ACK_FAILBEFORE`.

---

## §0 Declarations & code anchors (this tree)

- `int emergency_nack_count;` — `include/datalink_layer/arq.h:3132`. "consecutive
  failed data blocks". The BREAK accumulator.
- `int emergency_nack_threshold;` — `arq.h:3133` (default 3, set
  `arq_common.cc:640`).
- `int break_drop_step;` — `arq.h:3137`. ladder steps to drop; the panic path
  force-sets it to 100 (clamp to ROBUST_0 floor).
- `int breaks_since_last_data_success;` — `arq.h:3142`. BREAK panic-mode counter;
  `>=2` ⇒ panic-jump to ROBUST_0.
- `int last_data_viable_config;` — the CMD-side **forward-health anchor**. Raised
  ONLY through `data_anchor_raise_target()` after the SUSTAINED-ANCHOR gate
  (`arq_commander.cc:4266`, OFDM N=2 consecutive clean batches). This is the
  authoritative CMD-visible "the forward link was sustainably healthy at an OFDM
  rung" signal — the SAME one FIX-9 D3 already trusts (`arq_commander.cc:3890`,
  `is_ofdm_config(last_data_viable_config)`).
- `int link_status;` — `arq.h:2678`. `CONNECTED == 2`
  (`datalink_defines.h:41`).
- `enum TurboshiftPhase { TURBO_FORWARD, TURBO_REVERSE, TURBO_DONE };` —
  `arq.h:2920`. `turboshift_phase == TURBO_DONE` ⇒ not mid-handshake.
- `int data_ack_received;` — set NO at TX start (`arq_commander.cc:1428/1934`),
  set YES on any clean/partial SACK decode (`:3140/:3264/...`). The failure
  branch we hook is `if(data_ack_received == NO)` at `arq_commander.cc:3596`.
- The `emergency_nack_count++` is at **`arq_commander.cc:3703`** in this tree
  (the design cited `:3585/:3586` from a different build; `:3585` here is mid-
  comment; the structural anchor is "the line that does `emergency_nack_count++`
  inside the `data_ack_received==NO` branch, before the BREAK gate at `:4027`").
- The BREAK gate: `arq_commander.cc:4027`
  `if(emergency_nack_count >= emergency_nack_threshold && !config_is_at_bottom
  && !emergency_break_active && turboshift_phase==TURBO_DONE && gear_shift_on==YES)`.
- The panic-jump: `arq_commander.cc:4046-4054`
  (`breaks_since_last_data_success++; if(>=2) break_drop_step=100`).
- The success reset: `arq_commander.cc:4140` (`emergency_nack_count = 0`).

NEW state introduced by this fix (declared in `arq.h`, CMD-only, never on the
wire):
- `int forgiving_ack_consec_forgiven{0};` — consecutive forward-healthy
  reverse-ACK misses that the fix FORGAVE (re-aired same-gear) with NO landed
  data-ACK in between. The safety bound. Producer/consumer in §1/§2.
- `static const int FORGIVING_ACK_MAX_CONSEC = 8;`
  (`common_defines.h`) — the bound: after this many consecutive forgiven misses
  with no success, the fix STOPS forgiving and falls through to the existing
  BREAK path (the inverse-cascade safety).
- `inline bool forgiving_ack_should_decouple(...)` (`common_defines.h`) — the
  PURE decision helper (with `-DFORGIVING_ACK_FAILBEFORE` stub), replayed
  directly by `--test-forgiving-ack`.

---

## §1 Producers (every code path that WRITES the audited state)

### `emergency_nack_count` (the state the fix protects)
1. `arq_commander.cc:3703` — `emergency_nack_count++` inside `data_ack_received==NO`.
   **THE FIX INSERTS UPSTREAM OF THIS**: if the forward-health latch is healthy
   and the bound is not exhausted, the fix re-enqueues and `return`s BEFORE this
   `++`, so the counter is NOT advanced.
2. `arq_commander.cc:3688` — `= 0` on the gearshift-FRAME-UP-data-failed demote.
3. `arq_commander.cc:3842` — `= 0` on the FIX-4 carve-viability deadline demote.
4. `arq_commander.cc:4007` — `= 0` on the FIX-9 D3 reverse-ACK-starvation demote.
5. `arq_commander.cc:4140` — `= 0` on any data-ACK (clean OR partial) success.
6. ctor + `reset_session_state` — `= 0` init.

### `breaks_since_last_data_success`
- `arq_commander.cc:4046` — `++` at the BREAK trigger (the panic accumulator).
- `arq_commander.cc:4184` — `= 0` on a CLEAN fully-delivered batch
  (`promotion_allowed_on_batch`).
- ctor / session reset — init 0.

### `break_drop_step`
- `arq_commander.cc:4053` — `= 100` on the panic-jump (`breaks>=2`).
- `arq_commander.cc:4183` — `= 2` reset on a clean batch.
- the BREAK-recovery ladder helper (`break_target_with_anchor`) consumes it.

### `last_data_viable_config` (the forward-health ANCHOR the latch reads)
- SOLE raise producer: `arq_commander.cc:4266` via `data_anchor_raise_target()`,
  gated by the SUSTAINED-ANCHOR gate (OFDM N=2 consecutive clean batches at the
  rung). A partial SACK / single retransmit-rescued batch never raises it
  (`arq_commander.cc:4181` `promotion_allowed_on_batch`).
- Lowered ONLY by the anchor-DEMOTE path (`arq_commander.cc:4087`) after K
  consecutive anchor-rung BREAKs. **The fix never writes it.**

### NEW: `forgiving_ack_consec_forgiven`
- Producer-INCREMENT: the fix path itself — incremented each time the fix
  forgives a miss (re-airs same-gear) at `arq_commander.cc` (the new branch
  before `:3703`).
- Producer-RESET to 0: at `arq_commander.cc:4140` (the existing success reset
  block, `data_ack_received==YES`) — ANY data-ACK (clean OR partial) proves the
  reverse channel landed, so the forgiven-streak resets UNGATED, symmetric with
  `emergency_nack_count` and `cfg16_revack_starve_fails`. Also reset to 0 on the
  existing demote paths (§1 #2–#4) and on session reset, so a config move starts
  fresh.
- INIT 0 in ctor + `reset_session_state` (parity with the other CMD-only
  streak counters).

---

## §2 Consumers (every code path that READS the audited state)

### `emergency_nack_count`
- `arq_commander.cc:3724` — the `[BREAK] Block failure #%d` print.
- `arq_commander.cc:3775` — input to `bigblock_carve_fallback_target` (FIX-4).
- `arq_commander.cc:4027` — **THE BREAK GATE** (`>= emergency_nack_threshold`).
- The fix's effect on every consumer: because the fix `return`s BEFORE the `++`
  on a forgiven miss, `emergency_nack_count` is UNCHANGED on that turnaround, so
  every consumer sees the same value it would have seen had the batch never been
  sent. No consumer observes a partially-advanced counter.

### `forgiving_ack_consec_forgiven` (new)
- SOLE consumer: `forgiving_ack_should_decouple()` (the bound check
  `consec_forgiven < FORGIVING_ACK_MAX_CONSEC`). When the bound is hit the helper
  returns false ⇒ the fix does NOT forgive ⇒ control falls through to the
  unchanged `:3703` `++` and the existing BREAK path. The link cannot loop
  forever re-airing into a dead forward channel.

### `last_data_viable_config` / `is_ofdm_config(last_data_viable_config)`
- Read by FIX-9 D3 (`arq_commander.cc:3890`), the BREAK floor anchor
  (`break_target_with_anchor`, `:156`), the anchor-demote path (`:4072`), and now
  by `forgiving_ack_should_decouple()` as the persistent forward-health signal.
  The fix only READS it; it never writes it ⇒ no producer-side perturbation of
  any existing consumer.

---

## §3 Valid states (especially BEFORE any producer writes — default-init bites)

| State | `last_data_viable_config` | `link_status` | `turboshift_phase` | `breaks_since_last_data_success` | Latch verdict |
|---|---|---|---|---|---|
| Cold session, never delivered | session-floor anchor (ROBUST_0 on -R; start/pinned cfg otherwise — seated by §17 `session_floor_anchor`, NOT CONFIG_0) | not CONNECTED | TURBO_FORWARD | 0 | **UNHEALTHY** (link not connected ⇒ no decouple; genuine cold-link failure still BREAKs) |
| Mid-handshake (turbo climbing) | climbing | CONNECTED | ≠ TURBO_DONE | 0 | **UNHEALTHY** (turbo not done ⇒ no decouple) |
| Held OFDM, prior turnaround landed | OFDM rung (≥CONFIG_0, ≤CONFIG_16) | CONNECTED | TURBO_DONE | 0 | **HEALTHY** ⇒ decouple a fresh miss |
| Held OFDM, this is the 2nd+ consecutive miss, NO success between | OFDM rung | CONNECTED | TURBO_DONE | 0 (no BREAK fired yet — fix kept it from breaking) | **HEALTHY until bound** — forgiven up to FORGIVING_ACK_MAX_CONSEC, then UNHEALTHY (falls through to BREAK) |
| Forward channel genuinely collapsed at a robust/MFSK rung | robust/MFSK (NOT ofdm) | CONNECTED | TURBO_DONE | rising | **UNHEALTHY** (`is_ofdm_config(anchor)`==false) ⇒ existing BREAK fires |
| Link lost mid-transfer | stale | not CONNECTED | any | any | **UNHEALTHY** (link not connected) ⇒ existing BREAK/recovery |

**Default-init guard**: `last_data_viable_config` is seated at the session FLOOR
(`session_floor_anchor`, `arq_commander.cc` §17 fix — ROBUST_0 on -R, the
start/pinned config otherwise), NOT at CONFIG_0. So a cold -R session does NOT
present a spurious OFDM anchor: `is_ofdm_config(anchor)` is FALSE at t=0 on
robust, ⇒ the latch is UNHEALTHY at cold start ⇒ a genuine cold-link failure
takes the existing BREAK path. This is the SAME init the §17 anchor-poison fix
established and the SAME signal FIX-9 D3 already relies on; the forgiving-ack
latch inherits its correctness, it does not re-derive it.

---

## §4 Invariants the consumers assume (and that the fix must maintain)

- **INV-FA-1 (counter monotonic-or-reset)**: `emergency_nack_count` is either
  `++`'d on a real block-failure or reset to 0 on success/demote; no consumer
  ever sees it skip. The fix maintains this: a forgiven miss leaves it UNCHANGED
  (the fix returns before `++`), which is a valid state (== "this turnaround did
  not count as a block failure"). The BREAK gate still fires the instant the
  counter legitimately reaches the threshold via NON-forgiven failures.
- **INV-FA-2 (safety net preserved)**: a genuine link death STILL reaches the
  BREAK→ROBUST_0 path. Maintained two ways: (a) the latch is UNHEALTHY whenever
  the forward link is not sustainably-OFDM-healthy, link is down, or turbo is
  mid-handshake (§3), so those failures bypass the fix and hit `:3703`++ → BREAK;
  (b) even when the latch is HEALTHY, the bound `forgiving_ack_consec_forgiven <
  FORGIVING_ACK_MAX_CONSEC` forces a fall-through to BREAK after N forgiven
  misses with no success — so a forward channel that DEGRADES from healthy to
  dead AFTER the anchor was set cannot wedge the link forever; it BREAKs within
  N batches.
- **INV-FA-3 (no shared-state corruption)**: the fix WRITES only
  `forgiving_ack_consec_forgiven` (new, CMD-only) and the re-enqueue of the
  failed batch's frames (the SAME FIFO push-back the gearshift/FIX-4/FIX-9 BREAK
  paths already perform). It does NOT write `emergency_nack_count`,
  `break_drop_step`, `breaks_since_last_data_success`, `last_data_viable_config`,
  or any anchor/ceiling — so every existing consumer of those is byte-for-byte
  unaffected when the fix is OFF, and only the *absence* of a spurious `++` is
  the difference when ON.
- **INV-FA-4 (default-off byte-identity)**: gated by
  `getenv("MERCURY_FORGIVING_ACK")`. With the env unset the new branch is never
  entered (the guard short-circuits before any state read/write), so the SFO-grid
  / `--test` render is byte-identical to the base. Proven by md5 of the
  `--test-climb-engine` deterministic render vs the base (see §6).
- **INV-FA-5 (re-air epoch correctness)**: the re-enqueued batch must be re-sent
  as the SAME batch (same gear) the RSP last expected, NOT a fresh epoch — else
  the RSP sees a non-contiguous bsi and GAP-ABORTs (the D3.1 / lossless-demote
  hazard). The fix re-uses the EXISTING same-gear retransmit machinery (the
  failed batch's frames stay PENDING/are pushed back exactly as the
  `data_ack_received==NO` path already re-presents them; we do NOT advance
  `cmd_batch_seq_id` or load a new config), so the re-air carries the
  contiguous bsi by construction. The fix never crosses a config boundary, so the
  lossless-demote bsi-rollback (FIX-9) is N/A here.

---

## §5 What my fix changes (which assumption(s) it alters; walk every consumer)

The fix alters EXACTLY ONE thing: on a `data_ack_received==NO` turnaround where
the forward-health latch is HEALTHY, the link is CONNECTED, turbo is DONE, and
the forgiven-bound is not exhausted, it re-enqueues the batch for a same-gear
re-air and `return`s WITHOUT `emergency_nack_count++` / without touching the
BREAK machinery. Walk of every consumer of the altered state:

1. **BREAK gate (`:4027`)** — sees `emergency_nack_count` UNCHANGED ⇒ does not
   fire on this forgiven turnaround. CORRECT: the goal is to not break on a
   forward-healthy miss. It STILL fires when the counter reaches threshold via
   non-forgiven failures (latch unhealthy) or when the bound is exhausted.
2. **Panic path (`:4046`)** — never reached on a forgiven turnaround (the fix
   returned before the gate). `breaks_since_last_data_success` is therefore NOT
   advanced by a forgiven miss. CORRECT: a forward-healthy miss is not evidence
   the channel cratered.
3. **FIX-4 carve / FIX-9 D3 demotes (`:3770/:3888`)** — these run UPSTREAM of the
   `:3703` `++` in source order, BEFORE the new fix branch? **No** — see §7
   ordering decision: the fix branch is placed AFTER the FIX-4/FIX-9 D3 demote
   blocks and immediately BEFORE the `emergency_nack_count++` at `:3703` is NOT
   possible because `++` precedes the demote blocks. The fix is therefore placed
   **before the `emergency_nack_count++` at `:3703`**, which is itself before
   FIX-4/FIX-9-D3 (those read `emergency_nack_count` AFTER the `++`). On a
   forgiven miss the fix returns before `++`, so FIX-4/FIX-9-D3 are also not
   reached this turnaround — which is correct: a forgiven forward-healthy miss
   is recovered by a cheap re-air, it does not need the CFG16→CFG15 demote
   (which is the heavier, throughput-reducing escape). If the SAME rung keeps
   missing past the bound, the fix stops forgiving and the unchanged
   `++`→FIX-9-D3 / BREAK sequence runs exactly as today.
4. **`last_data_viable_config` consumers** — unaffected (the fix never writes it).
5. **opt_record_batch / optimizer** — NOTE: the fix is placed AFTER the existing
   `opt_record_batch(failed=true)` + `rate_opt.notify_cooldown_tick()` calls
   (`arq_commander.cc:3612/3620`), so a forgiven miss is STILL recorded as a
   failed batch in the optimizer window (it WAS a zero-byte turnaround) and the
   cooldown still drains. This is intentional: the optimizer's effective-rate
   accounting must see the lost airtime; the fix only prevents the CLIMB-ENGINE
   demote, not the throughput bookkeeping. (Verified the opt block does not early-
   return.)

**The §5 conclusion**: no consumer's assumption is violated. The fix CONSTRAINS
the producer (`emergency_nack_count` is not advanced on a forward-healthy
forgiven miss) and adds one new CMD-only counter with a single consumer
(the bound). The genuine-death safety is preserved by the latch's UNHEALTHY
states (§3) AND the FORGIVING_ACK_MAX_CONSEC bound (INV-FA-2).

---

## §6 Regression test (paired with this document, per CLAUDE.md)

`--test-forgiving-ack` (new CLI flag, in-process synthetic-fire; no IONOS/RF)
drives the `-g-ON` state machine through every transition this document
enumerates and replays the PURE helper directly. Each assertion is
fail-before (`-DFORGIVING_ACK_FAILBEFORE`) / pass-after.

- **FB/PA #1 (the root proof)** — a forward-healthy reverse-ACK miss:
  - FAIL-BEFORE: `forgiving_ack_should_decouple()` returns false (stub) ⇒ the
    counter would `++` ⇒ repeated drives sail `emergency_nack_count` to the BREAK
    threshold (3) and beyond (the design's "sails to #7") ⇒ assert FAIL.
  - PASS-AFTER: the helper returns true ⇒ the counter stays at 0 across many
    forward-healthy misses, NO BREAK ⇒ assert PASS.
- **SAFETY #2 (genuine death still breaks)** — a forward-DEAD miss (anchor robust
  / link down / turbo not done): the helper returns false ⇒ the counter `++`s to
  threshold ⇒ BREAK still fires ⇒ assert PASS (with the fix ON). Proves the
  safety net is intact.
- **SAFETY #3 (consecutive-forgiven bound escalates)** — drive
  FORGIVING_ACK_MAX_CONSEC+1 consecutive forward-healthy misses with NO success:
  the first N are forgiven (counter 0), the (N+1)-th hits the bound ⇒ the helper
  returns false ⇒ the counter `++`s ⇒ escalation toward BREAK ⇒ assert PASS.
  Proves the link cannot loop forever re-airing into a dead forward channel.
- **RESET #4** — a data-ACK (clean OR partial) after some forgiven misses resets
  `forgiving_ack_consec_forgiven` to 0 ⇒ a later miss is forgiven fresh ⇒ assert
  PASS. Proves the bound is per-stall, not cumulative.

Full `--test` must remain GREEN; default-off (env unset) render md5 must equal
the base (`integ/overnight-shippable`).

---

## §7 Insertion-point decision (where in `:3596`…`:4027` the branch lands)

Source-order facts in the `data_ack_received==NO` block:
1. `:3598` `consecutive_data_acks=0`, `:3603` `clean_batches_at_current_config=0`
   (streak resets — these fire for ALL failed batches, forgiven or not; we keep
   them, a forgiven miss WAS a failed delivery).
2. `:3612` `opt_record_batch(failed=true)`, `:3620` cooldown tick (kept — §5 #5).
3. `:3638-3700` the gearshift-FRAME-UP-just-applied retry/BREAK (returns early on
   its own paths).
4. **`:3703` `emergency_nack_count++`** ← THE FIX BRANCH IS INSERTED IMMEDIATELY
   BEFORE THIS LINE.
5. `:3716` `cfg16_revack_starve_fails++` (after the `++`).
6. `:3770` FIX-4, `:3888` FIX-9 D3 (read `emergency_nack_count` after the `++`).
7. `:4027` the BREAK gate.

The fix lands at (4): after the streak/optimizer bookkeeping (so a forgiven miss
is still counted as lost airtime by the throughput layer), after the gearshift-
just-applied early returns (those are a distinct, narrower recovery for the
PHY-switch rx-mute race and must keep their precedence), and immediately before
the `emergency_nack_count++`. On a forgiven miss the fix re-enqueues + returns,
so steps (5)(6)(7) are skipped THIS turnaround — correct, because the cheap
re-air supersedes both the CFG16→CFG15 demote and the BREAK. The gearshift-just-
applied block at (3) is left ABOVE the fix so a first-batch-after-PHY-switch miss
still uses its dedicated single-retry-then-BREAK logic (the fix does not subsume
it; they are orthogonal recovery mechanisms and (3) returns before reaching the
fix).

---

# TIER 2 — the cumulative-n_r status report (self-healing spine)

**Status**: Authoritative as of 2026-06-13 on `feat/forgiving-ack-tier1`,
extending the Tier-1 commit `55957ea`. Every change to the SACK `bsi` field
SEMANTICS, the `rx_bsi` apply window, `rsp_last_delivered_batch_seq_id`, or the
`CAP_CUMULATIVE_ACK` negotiation MUST update this section.

**Context** (judge verdict, `tasks/w1sx2th28.output` result.judge TIER 2 +
staged_plan Stage 2): Tier 1 made a forward-healthy reverse-ACK miss CHEAP (one
same-gear re-air instead of a 100–365 s BREAK→ROBUST_0 crawl). Tier 2 makes a
*residual* miss **FREE**: reshape the SACK into a STANAG-5066-style CUMULATIVE
status report — a missed report is SUPERSEDED by the next one, so the CMD
recovers the ACK for free on the following turnaround. The STANAG-5066 lens
(result.investigate): "the header of every D_PDU includes an EOT field … if even
a single header is received error-free, the receiver knows when it will be safe
to send an ACK" — the cumulative-status analogue is selective-repeat's
*acknowledged-through* high-water: a single later report carries forward
everything every earlier report would have.

## §T2.0 The reshape (semantics-only, NO wire-width change)

The MFSK ACK+SACK payload is `[bsi:8 | bitmap:32 | crc12:12]`
(`arq_common.cc:6660`); the OFDM SACK_RSP frame carries `bsi:8` + the per-frame
bitmap. Tier 2 reinterprets the **existing 8-bit `bsi` field**, when the
capability is negotiated, as `n_r` = the cumulative **contiguous delivery
high-water** (`rsp_last_delivered_batch_seq_id`). The 30/32-bit selective bitmap
is UNCHANGED — it describes the per-frame state of the batch ABOVE `n_r` (the
in-flight batch). No PHY change: `mfsk.cc` `pack_ack_sack_payload` treats `bsi`
as 8 opaque bits and never interprets them (`source/physical_layer/mfsk.cc:715`);
the entire semantic shift lives in the ARQ producer (RSP send) and consumer (CMD
apply). The CRC12 (`arq_common.cc:6717-6726`) covers `[bsi||bitmap]` byte-for-
byte regardless of what the bsi byte MEANS, so its protection is intact.

## §T2.1 The capability negotiation (default-off, interop-safe)

- `#define CAP_CUMULATIVE_ACK 0x04` (`datalink_defines.h`), `CAP_NEGOTIABLE_MASK`
  widened `0x03 → 0x07`.
- Advertised in `local_capability` ONLY when the env opt-in is set
  (`MERCURY_CUMULATIVE_ACK`), gated at the per-mode `local_capability` assignments
  (`arq_common.cc:3550/3647/3701/3720`). Default-off ⇒ bit never set ⇒ the
  reshape never engages ⇒ byte-identical to `55957ea`.
- Agreed into a session bool `cumulative_ack_enabled = (local_capability &
  CAP_CUMULATIVE_ACK) && (peer_capability & CAP_CUMULATIVE_ACK)` — the SAME
  `both_support` pattern as `encryption_enabled` (`arq_responder.cc:2367`,
  `arq_commander.cc:4821`). Computed once at the TEST_CONNECTION /
  TEST_CONNECTION_ACK negotiation on BOTH ends.
- **Wire transports** (no width change on either):
  - LDPC TEST_CONNECTION `data[5]` and TEST_CONNECTION_ACK echo `data[2]/[1]` are
    FULL bytes (`arq_responder.cc:2444/2497-2498`, `arq_commander.cc:4744/4767/
    4784`). Bit 2 (0x04) fits trivially — bits 2..7 were previously always zero.
    A legacy peer sends 0 there ⇒ `both_support` false ⇒ per-batch fallback.
  - MFSK ctrl-suffix TEST_CONN/TEST_ACK cap fields were masked to 2 bits
    (`mfsk_ctrl_codec.cc:160-174/196-210`); widened to `CAP_NEGOTIABLE_MASK` reusing
    the existing **reserved** payload bits ("must be 0 on TX, ignored on RX" —
    the EXACT §21 precedent that already carried a 3rd cap bit, since removed).
    This is a SEMANTICS reuse of reserved bits, NOT a 38-bit-payload-width change.
    A legacy peer leaves them 0 ⇒ bit 2 reads 0 ⇒ per-batch fallback.
- **INTEROP SAFETY**: a Tier-2 CMD talking to a non-Tier-2 RSP (or vice-versa)
  computes `cumulative_ack_enabled = false` (the peer never advertised 0x04), so
  BOTH the RSP `bsi` field stays per-batch AND the CMD apply stays per-batch.
  The cumulative interpretation engages ONLY when BOTH ends advertised it — a
  one-sided reshape (which would mis-apply a per-batch bsi as an n_r, ACKing
  unsent batches) is structurally impossible.

## §T2.2 The contiguous-high-water invariant — n_r CANNOT represent a gap (LIFE-CRITICAL)

`rsp_last_delivered_batch_seq_id` is the **reset-surviving delivery high-water**,
advanced ONLY at the two REAL in-order delivery commits:
- BATCH-DONE (`arq_responder.cc:1935`) — guarded by `delivery_step_is_gap(...)`
  (`:1917`): if delivering this batch would step the high-water by ≥2 (a hole),
  it GAP-ABORTs instead of delivering — so the high-water never skips an
  undelivered batch.
- PREV-DELIVER (`arq_responder.cc:911`) — guarded by the SAME `delivery_step_is_gap`
  gate moved BEFORE the copy (`:851-864`), check-then-deliver.

The producer is `advance_last_delivered(bsi)` (`arq_common.cc:6457`): it advances
ONLY on a forward step `fwd∈[1,128]` (mod-256), is idempotent on a re-delivery
(fwd==0), and IGNORES a backward late-older-prev (fwd∈[129,255], audit R1). So:

> **INV-T2-CONTIG**: `rsp_last_delivered_batch_seq_id` is the highest batch B such
> that batches `…, B-2, B-1, B` were ALL delivered to the application FIFO in
> order. It can NEVER point past an undelivered batch (the gap gate aborts the
> transfer before that can happen). Therefore "everything ≤ n_r is delivered" is
> a TRUE statement about application bytes, not an optimistic estimate.

This is the load-bearing safety property: a cumulative ACK saying "≤ n_r is
acknowledged" can never falsely ACK a batch the app did not receive, because the
RSP only ever raises n_r through a gap-gated in-order commit. A mis-applied
cumulative ACK = silent data loss (the directive's life-critical worry); this
invariant is what makes it impossible.

## §T2.3 Producers / consumers of the reshaped `bsi` field

PRODUCERS (RSP writes the bsi field on the wire):
- `send_mfsk_ack_sack(batch_seq_id, bitmap)` — clean ACK funnel
  (`arq_responder.cc` BATCH-DONE clean path) and the partial path
  (`arq_responder.cc:1790`). The `batch_seq_id` argument is the field value.
- `send_sack_v2_frame(bitmap, nframes, sacked_bsi)` — OFDM SACK_RSP partial
  (`arq_responder.cc:1806`, `arq_common.cc:6512`).
- Tier-2 wrap: a pure helper `cumulative_ack_bsi_field(per_batch_bsi, high_water,
  cap_on)` = `cap_on ? (high_water & 0xFF) : per_batch_bsi`. Applied at EACH send
  call site so when `cumulative_ack_enabled` the field carries n_r; otherwise
  byte-identical. When `high_water < 0` (nothing delivered yet) the helper falls
  back to `per_batch_bsi` — never sends a negative/0xFF garbage n_r.

CONSUMERS (CMD reads the bsi field):
- MFSK clean-ACK apply (`arq_commander.cc:2782-2785`): the in-window check
  `rx_bsi == cmd_bsi || rx_bsi == prev_bsi`.
- MFSK partial-SACK apply (same block, `:2840`).
- OFDM SACK_RSP apply (`arq_commander.cc:2975`): `sack_v2_bsi_in_window(rx_bsi,
  cmd_batch_seq_id)`.
- The startup MFSK arm (`arq_commander.cc:130`): same in-window check.
- Tier-2 wrap: a pure helper `cumulative_ack_covers(rx_n_r, target_bsi, cap_on,
  per_batch_in_window)`. When `cap_on`, `target_bsi` is addressed iff it lies in
  `[n_r - W .. n_r + 1]` (mod 256): sub-range (a) `((n_r - target) & 0xFF) <=
  CUMULATIVE_ACK_WINDOW` = at-or-below the high-water = CONFIRMED DELIVERED (the
  cumulative ACK / self-heal); sub-range (b) `target == (n_r+1)&0xFF` = the
  in-flight PARTIAL batch the 30-bit selective bitmap describes (the contiguous
  successor; Mercury is batch-level stop-and-wait so the only batch above n_r is
  n_r+1). The forward overhang is BOUNDED to EXACTLY 1, so a corrupt n_r still
  cannot ACK a far-future / unsent batch (§T2.4 safety preserved). When `!cap_on`
  it returns `per_batch_in_window` (byte-identical fallback). This REPLACES the
  `rx_bsi==cmd_bsi||rx_bsi==prev_bsi` acceptance test ONLY when the cap is on — it
  GENERALIZES "prev (one batch back)" to "any batch ≤ n_r within W (plus the
  partial n_r+1)", which is exactly the self-heal (a later n_r covers an earlier
  missed report). All apply sites call it TWICE (target=cmd_bsi, target=prev_bsi)
  and OR the results — the outstanding batch is one of those, and the bitmap→
  messages_tx[] slot mapping is by INDEX (independent of the bsi value), unchanged.

## §T2.4 Why "≤ n_r acked" is safe against the CMD TX queue (stop-and-wait model)

`cmd_batch_seq_id` advances by 1 each time a new-data batch is SENT
(`arq_commander.cc:1915`), and `messages_tx[]` holds ONE batch's frames at a
time. The CMD is batch-level stop-and-wait. So at the reverse-ACK decision the
only outstanding batch is `prev_bsi = (cmd_batch_seq_id-1)&0xFF`. A cumulative
n_r:
- **n_r == prev_bsi** ⇒ the outstanding batch is confirmed delivered ⇒ identical
  to today's clean-ACK-for-prev acceptance.
- **n_r > prev_bsi** is impossible in steady stop-and-wait (RSP cannot have
  delivered a batch the CMD never sent); the bounded window only looks BACKWARD
  from n_r, never forward, so a corrupt n_r ahead of `cmd_batch_seq_id` is
  rejected by the per-batch fallback edge (it is NOT within `[n_r-W..n_r]` for
  any target the CMD holds) — no future/unsent batch is ever ACKed.
- **n_r < prev_bsi (a STALE/old report)** ⇒ `prev_bsi` is NOT within `[n_r-W..n_r]`
  forward of n_r ⇒ NOT accepted as a confirmation of the current batch ⇒ the CMD
  keeps waiting / re-airs (Tier 1). Correct: an old report must not retire a
  newer outstanding batch. The CMD only treats the CURRENT outstanding batch as
  acked when n_r REACHES it.

The apply NEVER fabricates an ACK for a batch the CMD did not send: it only
decides whether the batch it is CURRENTLY waiting on (`cmd_bsi`/`prev_bsi`) is
covered by n_r. The 30-bit bitmap (the selective part for the batch above n_r)
drives the per-frame retransmit queue EXACTLY as before — Tier 2 does not touch
the bitmap→messages_tx slot mapping.

## §T2.5 Composition with Tier-1 and the existing machinery

- **Tier 1 (re-air)**: when a report is missed, Tier 1 re-airs the batch cheaply.
  Tier 2 means the NEXT turnaround's n_r retires the batch for free even if the
  re-air's own ACK is also lost — Tier 2 makes Tier-1's re-air RARE, never
  conflicts with it. Both are independently env-gated; Tier 2 default-off.
- **D5 / prev-bump / partial-bsi (the gearshift-acq→delivery 5-defect chain)**:
  Tier 2 READS `rsp_last_delivered_batch_seq_id` (already maintained by that
  machinery) and reuses the EXISTING `delivery_step_is_gap` gate — it adds NO new
  producer of the high-water and NO new delivery path, so it cannot become the
  6th defect. The bitmap path is untouched, so partial-bsi advance / prev-bump
  are unaffected.
- **GAP-ABORT**: unchanged — if a genuine hole appears, the RSP GAP-ABORTs and
  never raises n_r past it, so the cumulative ACK can't paper over a torn
  transfer.

## §T2.6 The test (`--test-cumulative-ack`, fail-before / pass-after)

`-DCUMULATIVE_ACK_FAILBEFORE` forces `cumulative_ack_covers` to the per-batch
fallback even when cap_on (the pre-Tier-2 behavior), making each assertion a true
fail-before/pass-after toggle in the SAME binary.
- **A (SELF-HEAL)**: deliver batches N then N+1; DROP N's report (its ACK never
  reaches the CMD). FAIL-BEFORE: N+1's per-batch report does NOT retroactively
  cover N (N is only in `{N+1,N}` window by luck of being prev — so the harder
  proof drops N's report while the CMD has already advanced; per-batch CANNOT
  recover N once it is >1 behind). PASS-AFTER: N+1's `n_r=N+1` covers N
  cumulatively (`((N+1)-N)&0xFF=1 ≤ W`) — N is acknowledged for free, never
  re-aired.
- **B (GAP-INVARIANT)**: a deliberate gap (N delivered, N+1 LOST, N+2 arrives) ⇒
  `advance_last_delivered` + `delivery_step_is_gap` hold the high-water at N (the
  real producer); assert `cumulative_ack_covers(n_r=N, target=N+1, …)` is FALSE —
  the missing N+1 is NOT cumulatively ACKed.
- **C (CAP-GATE)**: cap NOT negotiated (Tier-2 CMD + non-Tier-2 RSP) ⇒
  `cumulative_ack_covers(..., cap_on=false, per_batch_in_window=X)` returns X —
  pure per-batch fallback, no misapply of a per-batch bsi as an n_r.
- **D (COMPOSE-TIER-1)**: a forgiven (Tier-1) re-air whose own ACK is also lost is
  retired by the next n_r (the self-heal subsumes the re-air); assert the batch
  is covered without any further re-air.

Full `--test` GREEN; default-off (`MERCURY_CUMULATIVE_ACK` unset) render md5 ==
the Tier-1 base `55957ea`.

## §T2.7 Validation results (2026-06-13)

- `--test-cumulative-ack` PASS-AFTER (clean o3): **ALL PASS (0 failures)**, rc=0
  (P1–P11 helper truth tables, A1–A3 self-heal, B1–B5 gap-invariant via the REAL
  `advance_last_delivered`+`delivery_step_is_gap` producers, C1–C3 cap-gate,
  D1 compose-Tier-1).
- `--test-cumulative-ack` FAIL-BEFORE (`-DCUMULATIVE_ACK_FAILBEFORE`, clean o3):
  rc=1 — the cumulative-math P-arm (P4/P5/P7/P9) FAILS (the apply helper is pinned
  to the per-batch fallback) and the staged self-heal/gate/compose arms flip to
  their per-batch (no-recovery) expectations. True fail-before → pass-after toggle.
- Full `mercury --test` rc=0 (incl. the updated MFSK ctrl-codec 3-bit-cap
  round-trips `pack_unpack_test_ack_payload` / `pack_unpack_test_conn_payload` and
  the CONNECT passband round-trips). `--test-climb-engine` rc=0. Integrity battery
  rc=0: `--test-gap-abort`, `--test-inorder-demote`, `--test-sack-oow-reject` (the
  OFDM SACK window consumer Tier 2 modifies), `--test-batch-shrink-strands-prev`,
  `--test-retx-clear-on-recovery`, `--test-phantom-ack-gate`. Tier-1
  `--test-forgiving-ack` rc=0 (no regression).
- **DEFAULT-OFF BYTE-IDENTITY vs `55957ea`** (env `MERCURY_CUMULATIVE_ACK` unset):
  md5 of the deterministic ARQ renders is IDENTICAL between the base-55957ea binary
  and the Tier-2 binary — `--test-climb-engine` (722e60a2…), `--test-sack-oow-reject`
  (eeab4af0…), `--test-gap-abort` (b6e833ad…). The cumulative reshape + capability
  negotiation are fully inert without the opt-in. build.sh o3 clean (only the
  pre-existing winsock/WASAPI warnings). A full `mercury --test` log diff
  base-vs-Tier-2 (env-off) shows ONE differing line — a `[TIMING] total=46.2ms vs
  45.1ms` wall-clock instrumentation line (the same jitter present between any two
  runs of the SAME binary), i.e. NO behavioral delta.

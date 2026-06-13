# Data-Flow Audit: Forgiving-ACK (Tier 1 — decouple forward-healthy reverse-ACK miss from the BREAK→ROBUST_0 demote)

**Status**: Authoritative as of 2026-06-13 on `feat/forgiving-ack-tier1` (off
`integ/overnight-shippable` @ `f359f01`). Every change to `emergency_nack_count`,
the BREAK trigger, `break_drop_step`, `breaks_since_last_data_success`, or to the
new forward-health latch / `forgiving_ack_consec_forgiven` MUST update this document.

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

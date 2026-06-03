# Fact Document: Gearshift TOP-CONFIG ceiling — reach-but-can't-HOLD CONFIG_16 on a clean link

**Status**: Authoritative as of 2026-06-03 on `fix/hold-top-config` (branched from
`fix/oneway-stall` @2d5ab21, which is monitor + the WIN-campaign Fix #1 idle
SWITCH_ROLE self-teardown fix). Investigation of the WIN-campaign Fix #2: the
gearshift settles at CONFIG_14 on a clean link (0.74×) while ONE diagnostic run
that HELD CONFIG_16 hit 1.82× (BEAT VARA). This doc owns the producer/consumer
facts for the TOP-of-ladder down-move state: `break_drop_step`,
`last_data_viable_config` at the top rungs, and their interaction with the §11
sustained-anchor RAISE gate.

Companion to `gearshift-climb-engine.md` (the off-ROBUST_0 climb + the §10/§11
deep-SNR down-hysteresis). That doc fixed the climb *up*; this doc fixes the
*hold* at the top.

---

## §1 The symptom (measured, IONOS clean cell — `muething_results.json` 2026-06-03)

The Muething campaign (`tools/muething_throughput.py`, production gearshift
`-g -M auto`, NO `--max-config`) on the **clean** cell (IONOS WGN:40):

| run | steady_config | peak_config | final | client B/min | VARA B/min | ratio |
|-----|---------------|-------------|-------|--------------|------------|-------|
| 0   | CONFIG_14     | CONFIG_16   | CONFIG_16 | 34788    | 46809      | 0.74× |
| 1   | CONFIG_14     | CONFIG_16   | CONFIG_14 | 32580    | 46809      | 0.70× |

The config **switch_seq** (from RSP `[PHY] Config N active` markers):
- run 0: `ROBUST_0 ROBUST_0 ROBUST_1 ROBUST_2 CONFIG_0 CONFIG_4 CONFIG_13 CONFIG_14 CONFIG_15 CONFIG_16 CONFIG_14 CONFIG_16`
- run 1: `… CONFIG_14 CONFIG_15 CONFIG_16 CONFIG_14`

**config_counts** (run 0): `… CONFIG_14:2, CONFIG_15:1, CONFIG_16:2`.

Two facts jump out:
1. The link DOES reach CONFIG_16 (peak=16). It is not a hard ceiling; the climb
   works (Fix-set from `gearshift-climb-engine.md` is intact).
2. The drop from the top is `CONFIG_16 → CONFIG_14` — **a 2-rung drop that SKIPS
   CONFIG_15** (CONFIG_15 appears exactly ONCE = the climb-UP pass only; it never
   appears on the way down). The link then oscillates `14 ↔ 16`, and CONFIG_14
   dominates the latter half ⇒ `steady=CONFIG_14`.

The ONE diagnostic run that HELD CONFIG_16 → 1.82× is the existence proof:
holding the top config is the high-cell lever to beat VARA.

`steady_config` semantics (`muething_throughput.py:262`): the most-frequent config
in the **latter half of the switch sequence** (by switch count). So `steady=CONFIG_14`
≠ "stuck at 14" — it means the modem oscillates around 14↔16 and 14 wins the count.

---

## §2 NOT Fix #1's domain (the stuck-at-CONFIG_0 bimodality is separate)

The WIN-campaign asked: is the CONFIG_14 ceiling resolved by Fix #1 (the idle
SWITCH_ROLE self-teardown, `fix/oneway-stall`)? **No — they are disjoint:**

- Fix #1 targets the *idle* TX-empty FIFO firing the role-swap timer mid-bulk
  transfer (the one-way-transfer stall). Its signature is a role swap while the
  app socket is momentarily idle.
- The CONFIG_14 ceiling occurs while the link is **actively transferring at full
  rate** (run 0 moved 278 kB in 480 s). There is no idle teardown; the link is
  busy and just cannot hold the top rung.

The clean-cell ceiling is built ON TOP of Fix #1 (this branch = `fix/oneway-stall`)
so the measurement reflects post-Fix-#1 behavior.

**The bimodality** (wgn20 run-0 = stuck low at 0.00×, run-1 = CONFIG_13 at 0.49×)
is ALSO separate — and is a *climb-RATE* variance, not the top ceiling and not an
idle teardown. wgn20 run-0's switch_seq is a monotonic `ROBUST_0→ROBUST_1→ROBUST_2
→CONFIG_0` that never got past CONFIG_0 in the window (slow climb + run-to-run
channel variance); run-1 reached CONFIG_13. Neither shows the idle-teardown
signature (a sudden drop to ROBUST_0 from a working config while idle). The
climb-rate quest is explicitly LOW priority (the ~92 s climb is banked and
amortizes for long transfers — `gearshift-climb-engine.md` §12-§14). Scope of THIS
fix: the residual CONFIG_14-not-16 top ceiling only.

---

## §3 Root cause (file:line) — the 2-rung BREAK overshoot at the top

### §3.1 The down-path that fires at the top is the established-data emergency BREAK

Three down-paths exist from an OFDM rung; only ONE drops 2 rungs:

| path | site | step | 16 → |
|------|------|------|------|
| LADDER-DOWN (per-block low success) | `arq_commander.cc:5067` | −1 (`config_ladder_down`) | 15 |
| FRAME-UP-data-failed (1st batch after PHY-switch) | `arq_commander.cc:3422`/`:3438`, `:2365` | −1 (`break_drop_step=0`) | 15 |
| **established-data emergency BREAK** (3 consecutive total-NAcks) | `arq_commander.cc:3471` trigger → `:209`/`:307` recovery | **−`break_drop_step` (=2)** | **14** |

The measured drop is `16 → 14` skipping 15 (§1) ⇒ the **established-data emergency
BREAK** is the dominant top-of-ladder down-path. The other two would land on 15
and CONFIG_15 would show repeated counts; it shows exactly one (climb-up only).

### §3.2 Why the drop is exactly 2 rungs — `break_drop_step` reset to 2 on clean

`break_drop_step` is the BREAK recovery aggression (rungs dropped per recovery).
Its INITIAL value for each independent BREAK event is set by the clean-batch credit
block:

`arq_commander.cc:3592` (inside `promotion_allowed_on_batch(last_batch_fully_acked)`):
```cpp
break_drop_step = 2;       // Reset to initial aggression (2 steps).
```
So after ANY clean batch at the top, the next emergency BREAK uses `break_drop_step=2`.

BREAK recovery (`arq_commander.cc:209`, ACK-received path; `:307`, exhausted path):
```cpp
int raw_target = config_ladder_down_n(emergency_previous_config, break_drop_step, robust_enabled);
int target     = break_target_with_anchor(raw_target);   // floor at the anchor
...
break_drop_step *= 2;      // :222 / :321 — doubles WITHIN this recovery's retries
```
At the top: `emergency_previous_config = current_configuration = CONFIG_16` (set at
the trigger, `:3559`), `break_drop_step=2` ⇒ `raw_target = config_ladder_down_n(
CONFIG_16, 2) = CONFIG_14`. The doubling at `:222`/`:321` escalates only across
RETRIES within one recovery; the NEXT clean batch resets it to 2 again (`:3592`).
So **2 is the per-event initial drop**, and it overshoots CONFIG_15.

`break_drop_step = 2` is an UNDOCUMENTED magic number — no fact-doc rationale, no
measurement basis (CLAUDE.md §1 "magic numbers without measurement basis"). The
comment ("initial aggression (2 steps)") asserts intent without justification.

### §3.3 Why the overshoot prevents HOLDING CONFIG_16 — the anchor floor can't catch it

`break_target_with_anchor` (`arq_commander.cc:147-154`) floors a raw BREAK target
UP to `last_data_viable_config` (the anchor), except under the breaks≥2 panic.
So a drop is only contained AT THE TOP if the **anchor has ratcheted to the top**.

The anchor RAISE gate (`data_anchor_raise_target`, `arq.h:711`; §11 of
`gearshift-climb-engine.md`) requires `clean_streak >= sustained_anchor_threshold`,
which for OFDM is **`SUSTAINED_ANCHOR_N_OFDM = 2` CONSECUTIVE fully-clean (all-ones,
zero-SACK) batches** at the rung (`arq.h:677-679`, `:715`). The clean streak
`clean_batches_at_current_config` is reset to 0 on ANY failed (no-ACK) block
(`arq_commander.cc:3371`).

At the marginal 32QAM top (CONFIG_15/16), per-frame loss is high even on a clean
IONOS channel (historical "<12% per-frame success at 20 dB" — `common_defines.h:114`;
SACK makes the config VIABLE, recovering losses as PARTIAL batches, but a
SACK-rescued batch is not a fully-clean all-ones batch). So 2 *consecutive*
fully-clean batches at 15/16 is hard to accumulate before a down-move knocks the
link off — **the anchor loses the race to the top**, stays at ≤CONFIG_14, and
`break_target_with_anchor` therefore floors the 2-rung drop at CONFIG_14 (not 15).

The 1.82× run is the case where the anchor DID reach CONFIG_16 (by channel luck it
got 2 consecutive cleans at 15 and at 16): then `break_target_with_anchor` floored
EVERY subsequent drop back to 16 → it HELD 16.

### §3.4 The two root causes are ONE dynamic — the overshoot drives the anchor race

The 2-rung drop is the PRIME MOVER: by overshooting to CONFIG_14 it (a) loses
CONFIG_15 every cycle and (b) forces the re-climb to re-traverse 14→15→16, so the
link spends little time at 15-16 and the anchor never gets its 2 consecutive cleans
there. A 1-rung drop (16→15) lands the link on the next-best config AND keeps it at
15-16, where 2-consecutive-cleans at 15 (less marginal than 16) is achievable → the
anchor ratchets to 15, `break_target_with_anchor` holds 15+, and FRAME-UP/elevator
re-probe 16 from proven ground. Fixing the overshoot relieves the anchor race as a
side effect — they are the same loop.

---

## §4 The fix — initial established-data BREAK aggression 1 rung, not 2

Change the clean-credit reset (`arq_commander.cc:3592`):
```cpp
break_drop_step = 1;       // initial aggression: ONE rung. The :222/:321 doubling
                           // escalates 1->2->4 across a recovery's retries; the
                           // breaks>=2 panic-jump (:3492 -> 100) still force-floors
                           // a genuinely crashing channel. A 2-rung INITIAL drop
                           // (the old value) overshoots the top of the ladder
                           // (CONFIG_16 -> CONFIG_14, skipping 15) and prevents
                           // holding the top. See gearshift-top-config-ceiling.md §3/§4.
```

### §4.1 Why 1 is correct (root cause, not a threshold-mask)

- A single transient burst of 3 total-NAcks at the top (the emergency_nack
  threshold) does NOT mean the config is 2 rungs too high — it means *one* rung
  down is the conservative, minimal recovery. Dropping 2 rungs on the FIRST BREAK
  is an unmeasured over-aggression.
- The existing doubling (`:222`/`:321`) preserves fast escape on a genuinely
  crashing channel: if the 1-rung recovery's coordination needs retries, the step
  becomes 2, 4, … within that recovery.
- The panic-jump (`breaks_since_last_data_success >= 2` ⇒ `break_drop_step = 100`,
  `arq_commander.cc:3486-3493`) is UNTOUCHED: two BREAKs with no clean between still
  force a jump to the ladder floor. Crash-escape latency is unchanged.
- It is the MINIMAL change: one integer literal at the one site that sets the
  per-event initial aggression for established-data BREAKs.

### §4.2 Effect on the top-of-ladder steady state

- BREAK at CONFIG_16 now drops to CONFIG_15 (1 rung), not CONFIG_14.
- The link holds at CONFIG_15-16 instead of CONFIG_14-16: an immediate ~1-rung
  steady-config lift, and the anchor can now ratchet to 15 (then 16) because the
  link dwells at the top — making "HOLD CONFIG_16" reachable over a sustained
  transfer (per §3.4). Lifts the clean-cell ratio toward the 1.82× existence proof.

---

## §5 Cross-layer audit (CLAUDE.md §5) — `break_drop_step`

Shared state changed: `break_drop_step` (BREAK recovery aggression). It is CMD-only
ARQ state (the RSP follows the CMD's SET_CONFIG target; it has no `break_drop_step`).

### 5.1 Producers (every writer of `break_drop_step`)

- **`arq_commander.cc:3592`** — reset to 2 on clean batch credit. **THIS FIX: → 1.**
  The per-event INITIAL aggression for established-data emergency BREAKs.
- `arq_commander.cc:222`, `:321` — `*= 2` doubling within a recovery's retries
  (ACK-received / exhausted). UNTOUCHED.
- `arq_commander.cc:2254` — turbo CEILING path computes the step from the SNR-target
  delta (`steps = idx(failed) − idx(snr_target)`), else `:2259` sets 1. UNTOUCHED.
  (Turbo CEILING is the SUPERSHIFT-probe failure path, not the steady-state climb.)
- `arq_commander.cc:2331` — turbo SWITCH_ROLE break sets 1. UNTOUCHED.
- `arq_commander.cc:2390`, `:3441` — frame-up-failed paths set 0. UNTOUCHED.
- `arq_commander.cc:3492` — panic-jump forces 100. UNTOUCHED.
- ctor / reset_session_state init (`arq_common.cc`). UNTOUCHED (init value is not
  the per-event initial; the `:3592` reset on the first clean sets it before any
  established-data BREAK).
- test sites `:6246`, `:6569` (test_climb_engine / other tests). UNTOUCHED.

### 5.2 Consumers (every reader of `break_drop_step`)

- **`arq_commander.cc:209`, `:307`** — `config_ladder_down_n(emergency_previous_config,
  break_drop_step, …)` to compute the BREAK recovery target. The SOLE behavioural
  consumers. After the fix the per-event initial target is 1 rung down (was 2).
- The `[BREAK]` log lines (`:219`, `:299`, `:318`) print it. Cosmetic.

### 5.3 Valid states

`break_drop_step ∈ {0, 1, 2, 4, …, 100}`. 0 = the frame-up-failed paths (drop via
`config_ladder_down` of the *target*, not via `_n`). 1 = NEW per-event initial (was
2) + turbo SWITCH_ROLE. 2/4/… = doubled within a recovery or the old initial. 100 =
panic-jump floor. `config_ladder_down_n` clamps at the ladder floor (ROBUST_0 /
CONFIG_0), so any value ≥ ladder height is equivalent to "floor".

### 5.4 Invariants the consumers assume — verified

1. **`break_target_with_anchor` still floors at the anchor.** UNCHANGED — the fix
   only changes `raw_target` (one rung higher than before), then the SAME anchor
   floor applies. At the top with anchor < 15 the floor is a no-op (raw_target=15 ≥
   anchor) → lands at 15. Correct.
2. **The breaks≥2 panic bypass still reaches the floor.** UNCHANGED — panic forces
   `break_drop_step=100` BEFORE the recovery reads it, overriding the initial 1.
   `break_target_with_anchor` is bypassed under panic (`:149`). Crash/cliff escape
   intact.
3. **The doubling escalation still applies.** UNCHANGED — `:222`/`:321` double the
   (now-1) value across a recovery's retries. A recovery that needs multiple
   coordination rounds still escalates 1→2→4.
4. **WGN:-10 anti-thrash (§10/§11 of gearshift-climb-engine.md) is preserved.** The
   WGN:-10 escape is the panic-jump + the §10 anchor DEMOTION, both INDEPENDENT of
   the `:3592` initial value. With anchor pinned at the cliff rung,
   `break_target_with_anchor` floors EVERY raw_target to the anchor regardless of
   whether the drop step is 1 or 2 — so the initial-step change is INERT at the
   cliff until the §10 demotion lowers the anchor (which is unaffected). No regression.

### 5.5 What the fix changes — single assumption altered

The per-event INITIAL BREAK aggression (rungs dropped on the FIRST recovery round
of an independent BREAK) drops from 2 to 1. Every consumer reads the recovery target
and either issues a SET_CONFIG to it or floors it at the anchor; both behave
correctly with a 1-rung initial target. No consumer relied on the 2-rung initial
drop (it was an unmeasured magic number, never a load-bearing invariant).

---

## §6 Regression test — `--test-climb-engine` Part Q (fail-before / pass-after)

Part Q drives the REAL BREAK-recovery target computation
(`config_ladder_down_n(emergency_previous_config, break_drop_step, robust_enabled)`
+ `break_target_with_anchor`) — the SAME primitives the production recovery sites
(`:209`/`:307`) call — at the top of the ladder, in a WB (non-robust) session.

- **Q0**: documents the TUNABLE — `break_drop_step` after the `:3592` clean reset is
  the post-fix initial aggression. Asserts the recovery from CONFIG_16 with the
  POST-FIX initial step lands on CONFIG_15 (1 rung), not CONFIG_14.
- **Q1 (THE ceiling assertion, fail-before)**: emergency_previous_config=CONFIG_16,
  anchor=CONFIG_13 (below the drop target so the floor is inert), POST-FIX initial
  `break_drop_step` ⇒ `break_target_with_anchor(config_ladder_down_n(CONFIG_16, step))
  == CONFIG_15`. Pre-fix (step=2): == CONFIG_14 → FAIL. Post-fix (step=1): ==
  CONFIG_15 → PASS. This is the 2-rung-overshoot → 1-rung fix, isolated.
- **Q2 (anchor floor still catches the top once anchored)**: anchor=CONFIG_16, the
  recovery target floors UP to CONFIG_16 (the link HOLDS the top once the anchor
  reaches it). Independent of the step value — passes before & after (proves the
  hold mechanism is the anchor, not the step).
- **Q3 (panic-jump still reaches the floor, SAFETY)**: `breaks_since_last_data_success
  = 2` ⇒ the breaks≥2 bypass returns the raw target unclamped; with
  `break_drop_step = 100` (panic value) the recovery floors at CONFIG_0 (WB) — the
  crash escape is intact and UNAFFECTED by the initial-step change. Passes before &
  after.
- **Q4 (doubling escalation preserved)**: from the POST-FIX initial step, one `*= 2`
  recovery round yields a 2-rung drop (CONFIG_16 → CONFIG_14) — proves the
  escalation still reaches the old aggression on a recovery that needs a retry.
  Passes before & after (the doubling is untouched).

**FAIL-BEFORE protocol**: temporarily restore `break_drop_step = 2` at the test's
post-reset priming (model the pre-fix initial) → Q1 FAILS (got CONFIG_14, want
CONFIG_15); Q0/Q2/Q3/Q4 stay PASS (they assert step-independent invariants or the
doubling). **PASS-AFTER**: with the shipped `break_drop_step = 1`, all Q PASS.

The over-climb / anti-thrash regressions (Parts E/F = §10/§11 WGN:-10
down-hysteresis; Part J = the §13 elevator over-climb; Parts K = §16 anchor-tier;
Part M = §17 anchor-init-poison) and `--test` (30/30) must stay green — verified
post-fix (the fix touches only the BREAK INITIAL step, not the anchor gate, the
elevator, or the +1 clamp).

### §6.1 HONEST scope (§8 of gearshift-climb-engine.md applies)

Part Q proves the BREAK recovery from the top drops 1 rung (was 2) and that the
anchor floor + panic-jump + doubling are structurally intact. The FULL wire
confirmation — a clean-cell session now HOLDS CONFIG_15-16 (lifting the ratio toward
the 1.82× proof) AND WGN:-10 still settles ROBUST_0 (the §10/§11 anti-thrash, which
runs with the panic/demotion escapes the fix does not touch) — is the parent's ONE
post-fix bench re-measure (the like-for-like Muething re-run). SIM-first: the
marginal-top regime (high per-frame loss + SACK-partial recovery) is NOT reproducible
in a near-lossless loopback, so the in-process Part Q (which drives the exact
production decision primitives deterministically) is the validation vehicle — the
same pattern Parts E/F use for the WGN:-10 thrash.

---

## §7 Related fact documents

- `gearshift-climb-engine.md` — the off-ROBUST_0 climb UP + the §10/§11 deep-SNR
  down-hysteresis. This doc's `break_drop_step` fix is the TOP-of-ladder counterpart:
  §10/§11 fixed the deep-SNR thrash (can't hold the FLOOR sensibly); §3/§4 here fix
  the high-SNR overshoot (can't hold the TOP). The anchor (`last_data_viable_config`)
  and `break_target_with_anchor` are shared; this fix does NOT touch the anchor RAISE
  gate (§11) or the DEMOTE producer (§10) — only the BREAK initial step that feeds
  `break_target_with_anchor`.
- `gearshift-start-and-recovery.md` (§6/§7) — the Option-B anchor + clean-batch
  viability design (referenced by the code; lives on other branches).

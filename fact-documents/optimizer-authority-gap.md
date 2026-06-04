# Optimizer Authority Gap — over-climb→collapse cross-layer audit

**Status:** IMPLEMENTED 2026-06-03 on branch `fix/over-climb-authority` (off `c5d67cc`). See §8 for the as-built record (every edit file:line, the 5-reader verdict, the existing-test reconciliation, R6/R7 FAIL-BEFORE→PASS-AFTER). Analysis below (§0-§7) is the pre-implementation audit and is unchanged; §8 is the build log.
**Base ref:** inner `mercury/` repo, branch `fix/sustainable-config-hold` HEAD = `c5d67cc`,
parent = `730ffca` (WIN-integ on top of monitor `0c75cc2`). All file:line citations are
against **730ffca** unless noted (the FIX-1/FIX-2 base; `c5d67cc` is built on it and the
task's line numbers — arq.h:2053, arq_common.cc:3330 — match 730ffca, NOT monitor 0c75cc2
where `optimizer_is_in_control()` sits at arq.h:1955).
**Monitor delta note:** monitor→730ffca changed arq.h (+98 lines incl the GF16-RA tier),
arq_commander.cc (+511), arq_common.cc (+123). Line numbers shift on monitor; the semantic
audit below is invariant. Build the fixed UNIT off `c5d67cc` (already on this base).

This document owns the producer/consumer map for two pieces of shared gearshift state:
1. `optimizer_is_in_control()` — the authority predicate (FIX-1).
2. The `breaks_since_last_data_success` panic + `last_data_viable_config` anchor state
   (FIX-2 + c5d67cc Component 2).

---

## §0. The bug, stated precisely (the keystone mechanism)

`supershift_retrigger_target()` (arq.h:830-877) is the SINGLE shared chokepoint all three
promotion paths funnel through (FRAME-UP elevator via `elevator_target_from_snr()`
arq_commander.cc:174-186; SUPERSHIFT turbo re-trigger arq_commander.cc:4678; the FRAME-UP
default-+1-then-elevator at the rx-ACK path). Its FIRST statement is:

```
arq.h:834   if(optimizer_owns)
arq.h:835     return snr_ideal;            // Q-table owns the band -- no anchor clamp
```

`optimizer_owns` is passed as `optimizer_is_in_control()` at every caller (arq_commander.cc:183,
4678; arq.h:1960 invariant comment). When it is true, the function returns `snr_ideal`
**uncapped** — the `is_ofdm_config(anchor)` +1 clamp (arq.h:854) AND the
`RETRIGGER_MAX_LEAP` leap_cap (arq.h:865-867, the cap c5d67cc Component 1 lowers 13→4) are
**both bypassed**.

`optimizer_is_in_control()` (arq.h:2053-2058) is INDEX-ONLY:

```
arq.h:2053   bool optimizer_is_in_control() const {
arq.h:2054     if (optimizer_disabled || !rate_opt.is_enabled()) return false;
arq.h:2055     int handoff = rate_opt.min_calibrated_cfg(narrowband_enabled == YES);
arq.h:2056     if (handoff <= 0) return false;
arq.h:2057     return config_ladder_index(current_configuration) >= config_ladder_index(handoff);
arq.h:2058   }
```

It returns true the instant `current_configuration` index ≥ the lowest-calibrated-cfg index,
**with NO channel/sack-rate predicate**. So on an UNCALIBRATED or NOISY channel where the
EVM-SNR over-reports and the climb has already reached (say) CONFIG_6+, this returns true,
the `optimizer_owns` short-circuit fires, and the MAX_LEAP clamp that should bound the
over-climb is DISABLED. Meanwhile the optimizer's own batch-end evaluator RECUSES on exactly
the channel predicate this function lacks:

```
arq_common.cc:3330   if (min_cfg >= 0 && current_configuration < min_cfg) return false;
arq_common.cc:3331   if (max_sack >= 0.0 && get_current_sack_rate() > max_sack) return false;
```

So `opt_evaluate_batch_end()` goes SILENT (no `opt_pending_switch_cfg` set → no optimizer
move) on the noisy channel, while `optimizer_is_in_control()` simultaneously says "true"
(disabling the gearshift clamps). **Result: NEITHER controller bounds the climb** — the
authority gap. The link over-climbs (CONFIG_0 idx3 → CONFIG_13 idx16 in one leap under the
old MAX_LEAP=13), the high config cannot sustain mid-SNR, it fails, and the un-anchored
LADDER-DOWN + breaks≥2 panic collapses it to ROBUST_0 (§3).

**FIX-1 closes the gap by giving `optimizer_is_in_control()` the SAME channel predicate
`opt_evaluate_batch_end()` uses** — so on a noisy/uncalibrated channel it returns false,
the `optimizer_owns` short-circuit does NOT fire, and c5d67cc's MAX_LEAP=4 clamp BITES
above CONFIG_6. **This is why c5d67cc + FIX-1 + FIX-2 must ship as ONE unit: c5d67cc's
MAX_LEAP is INERT above CONFIG_6 without FIX-1** (the `optimizer_owns` early-return jumps
over the leap_cap line entirely).

---

## §1. FIX-1 audit — `optimizer_is_in_control()` (arq.h:2053-2058)

### §1.1 Producers (writers of the state it reads)

`optimizer_is_in_control()` reads three inputs; their producers:

| Input | Producer file:line | Notes |
|-------|--------------------|-------|
| `optimizer_disabled` | `set_optimizer_disabled()` arq.h:2049; CLI `--no-optimizer` main.cc; forced `true` in 6 test/un-clamp sites (arq_commander.cc:6000, 6440, 6985, 7369, 7571, 7766, 8435) | A `true` here already forces the predicate false — FIX-1 adds NO new interaction. |
| `rate_opt` table (`is_enabled()`, `min_calibrated_cfg()`, `max_calibrated_sack_rate()`) | `rate_opt.load()` at startup (rate_optimizer.h:122-134); populated from the Q-table JSON. `-1`/`-1.0` when unloaded. | FIX-1 adds a read of `max_calibrated_sack_rate()` — same accessor `opt_evaluate_batch_end()` already calls (arq_common.cc:3329). |
| `current_configuration` | gearshift/turbo/BREAK/optimizer config moves (all `negotiated_configuration` → `load_configuration`) | unchanged by FIX-1. |
| **NEW** `get_current_sack_rate()` (arq.h:2246-2253) reads `opt_window_count`, `opt_batch_sack_count[]`, `opt_window_head` | producers: `opt_on_batch_end()` ring writes (arq.h ~2300+), reset by `opt_reset_window()` (arq_commander.cc:566) | Already a live, maintained consumer input — `opt_evaluate_batch_end()` reads it at arq_common.cc:3331. FIX-1 reuses the identical read. |

`get_current_sack_rate()` returns **0.0** when `opt_window_count == 0` (arq.h:2247) — i.e.
at session start / right after `opt_reset_window()`. Semantics of the new predicate at
window=0: `0.0 <= max_sack` is true → the sack-rate axis does NOT recuse. This is CORRECT
and INTENTIONAL: it MATCHES `opt_evaluate_batch_end()`, which at window=0 also does not
recuse on the sack axis (`get_current_sack_rate()` returns 0.0 ≤ max_sack). The min_cfg
index axis (already present) is what gates at session start; the sack axis only bites once
≥1 batch has been observed and the channel proves noisy (high sack rate). **No new
cold-start hazard** — the two predicates stay symmetric.

### §1.2 Consumers (the 5 functional readers) — each must want the SAME "is the optimizer genuinely steering THIS channel?" semantics

Grep `optimizer_is_in_control` across source (730ffca) yields the call sites; the
FUNCTIONAL readers (excluding comment refs at 808/887/1960/2507 and test-setup `=true`
assignments at 6000/6440/6985/7369/7571/7766/8435) are:

| # | Reader | file:line | What it gates | Wants tighter semantics? |
|---|--------|-----------|---------------|--------------------------|
| R1 | FRAME-UP elevator, `process_messages_rx_acks_data()` | arq_commander.cc:**3709** (Option-B data-anchored clamp) + **3715** (`optimizer_owns_upward_frame`) | When true, EXEMPTS the +1/anchor clamp AND lets the optimizer (not gearshift) own the upward frame move. | **YES.** On a noisy channel the optimizer is NOT steering (it recused at :3330). Tightening → gearshift's data-anchored +1 clamp re-applies. Strictly safer. |
| R2 | legacy LADDER-UP (v1, sack_v2 off), `finalize_block_commander()` | arq_commander.cc:**5002** (entry gate `&& !optimizer_is_in_control()`) + **5013** (Option-B re-check) | When true, gearshift LADDER-UP yields entirely. | **YES.** Tightening → gearshift LADDER-UP re-engages on the noisy channel (with its own 85%-clean + last_data_viable+1 guards). Safer. |
| R3 | v2 LADDER-UP, `policy_evaluate_axis1()` (def arq_commander.cc:5142) | arq_commander.cc:**5158** (entry gate) + **5170** (Option-B re-check) | identical to R2 on the SACK-v2 policy path. | **YES.** Same as R2. |
| R4 | turbo SNR-SUPERSHIFT chokepoint, turbo re-trigger | arq_commander.cc:**4678** (`optimizer_owns` arg into `supershift_retrigger_target`) — and the sibling `apply_optimizer_handoff_cap_to_target(&snr_target)` at arq_commander.cc:**4626** | When true, the turbo target is returned UNCLAMPED (no leap_cap, no anchor+1). | **YES.** This is the over-climb path. Tightening is the whole point — it makes the MAX_LEAP=4 leap_cap bite on the noisy turbo climb. |
| R5 | controlled elevator shared method, `elevator_target_from_snr()` | arq_commander.cc:**182-183** (`optimizer_is_in_control()` arg into `supershift_retrigger_target`) | identical short-circuit as R4, on the FRAME-UP elevator path. | **YES.** Same as R4. |

**Conclusion: all 5 readers want the IDENTICAL "optimizer genuinely steering THIS channel"
semantics.** Every one currently treats index-only-true as "the Q-table owns this band, so
stand down" — and on a noisy/uncalibrated channel that is FALSE (the Q-table recused). All 5
become STRICTLY SAFER under the tighter predicate: the gearshift's own anchor-based clamps
(+1 / last_data_viable+1 / leap_cap) re-engage, which is exactly the behavior the noisy
channel needs.

### §1.3 No reader DEADLOCKS on index-only-true — the one thing to rule out

The dangerous case would be a reader that RELIES on `optimizer_is_in_control()==true` to
escape a state it would otherwise be stuck in (so tightening it would trap the link).
Checked all 5:
- R1/R2/R3 use it to SUPPRESS upward moves. Tightening → MORE upward moves allowed (gearshift
  re-engages), never fewer. Cannot deadlock — the worst case is the pre-optimizer gearshift
  behavior, which by construction is not stuck (it has its own ladder + BREAK).
- R4/R5 use it to UNCLAMP the leap. Tightening → the leap is CLAMPED to anchor+MAX_LEAP, which
  is still a multi-rung UP move (≥ anchor+1, arq.h:854 guarantees ≥ anchor_cap). Cannot trap
  downward.
- **Downward authority is unaffected.** BREAK / LADDER-DOWN / panic are NEVER gated on
  `optimizer_is_in_control()` (verified: no `optimizer_is_in_control` in the BREAK path
  3460-3580, the LADDER-DOWN at 5077/5255, or `break_target_with_anchor` 147-153). So the
  safety-net descent is available regardless of FIX-1.
- **The optimizer's OWN config move is independent of this predicate.** The optimizer applies
  moves via `opt_pending_switch_cfg` (set by `opt_evaluate_batch_end()` at arq_common.cc:2960/
  3080/3210, consumed at arq_commander.cc:541-572). That consumer gate (`:541-553`) does NOT
  call `optimizer_is_in_control()`. So FIX-1 does NOT reduce the optimizer's ability to steer
  WHEN IT IS CALIBRATED — once the channel returns to the calibrated region, `opt_evaluate_
  batch_end()` re-engages, sets `opt_pending_switch_cfg`, and the move applies through this
  independent path. FIX-1 only stops the gearshift from STANDING DOWN while the optimizer is
  recused. **The two predicates become symmetric — exactly the intended handoff invariant
  (arq.h:807-813 doc comment: "Below [the calibrated band], the optimizer has no data and
  gearshift's SNR-based ladder runs").**

### §1.4 Valid states of `optimizer_is_in_control()`

| State | When | Current return | FIX-1 return |
|-------|------|----------------|--------------|
| disabled | `optimizer_disabled` or `!is_enabled()` | false | false (unchanged) |
| no table | `min_calibrated_cfg() <= 0` | false | false (unchanged) |
| below band | `idx(current) < idx(handoff)` | false | false (unchanged) |
| **in band, channel calibrated** | `idx(current) >= idx(handoff)` AND `sack_rate <= max_sack` | **true** | **true** (unchanged) |
| **in band, channel NOISY (uncalibrated region)** | `idx(current) >= idx(handoff)` AND `sack_rate > max_sack` | **true (BUG)** | **false (FIX)** |

Only the last row changes — exactly the over-climb scenario.

### §1.5 What FIX-1 changes (assumption walk)

FIX-1 alters ONE assumption: "in-band ⟹ optimizer owns." It adds "AND the channel is within
the calibrated sack-rate range." Walked every consumer in §1.2/§1.3: all 5 want the tighter
semantics; none deadlocks; downward authority and the optimizer's own-move path are
untouched. The predicate now MIRRORS `opt_evaluate_batch_end()`'s recusal (arq_common.cc:
3330-3331) so the two controllers hand off at the SAME boundary instead of leaving a gap.

---

## §2. FIX-2 audit — the breaks≥2 panic-collapse + the anchor state

### §2.1 The panic-collapse state machine (producers/consumers)

**State:** `breaks_since_last_data_success` (int), `last_data_viable_config` (the anchor),
`break_drop_step` (int), `anchor_consec_break_fails` (int).

**Producers of `breaks_since_last_data_success`:**
| Action | file:line | Effect |
|--------|-----------|--------|
| increment on a BREAK firing | arq_commander.cc:**3485** (`breaks_since_last_data_success++`) | inside the `emergency_nack_count >= threshold` BREAK trigger (3472). |
| panic latch | arq_commander.cc:**3486-3492** (`if(>=2) break_drop_step = 100`) | forces the recovery target to ROBUST_0. THE PANIC-COLLAPSE. |
| reset on CLEAN data | arq_commander.cc:**3606** (`= 0` inside `promotion_allowed_on_batch(last_batch_fully_acked)`) | a clean batch clears the panic counter. |

**Consumers of `breaks_since_last_data_success`:**
| Reader | file:line | Use |
|--------|-----------|-----|
| panic latch itself | arq_commander.cc:3486 | `>=2` → break_drop_step=100 |
| anchor-floor bypass | `break_target_with_anchor()` arq_commander.cc:**149** (`if(>=2) return raw_target`) | under panic, the BREAK recovery does NOT floor up to the anchor → it reaches ROBUST_0. |

**Producers of `last_data_viable_config` (the anchor):**
| Action | file:line | Effect |
|--------|-----------|--------|
| RAISE on clean batch | arq_commander.cc:**3661** `= data_anchor_raise_target(clean_batches_config, current_configuration, last_data_viable_config, clean_batches_at_current_config)` | SOLE on-delivery raise. §11 sustained-N gate + §16 tier-crossing gate (arq.h:730). Only ever raises. |
| DEMOTE on K anchor-rung BREAKs | arq_commander.cc:**3526** `= demoted` (via `anchor_demote_target`) | only fires when `current_configuration == last_data_viable_config` (3517) AND `anchor_consec_break_fails >= ANCHOR_DEMOTE_BREAK_FAILS` (3520). Only ever lowers. |

**Consumers of `last_data_viable_config` (the anchor floor):**
| Reader | file:line | Use |
|--------|-----------|-----|
| BREAK recovery floor | `break_target_with_anchor()` arq_commander.cc:**150-151** (`if(idx(raw)<idx(anchor)) return anchor`) | floors a BREAK drop UP to the anchor (unless panic). |
| FRAME-UP +1 clamp | arq_commander.cc:3709, 6606 (test) | `idx(proposed) > idx(anchor)+1 → blocked`. |
| LADDER-UP +1 clamp | arq_commander.cc:5013, 5170 | same as FRAME-UP. |
| leap_cap base | `supershift_retrigger_target()` arq.h:849/865 (`anchor` param = last_data_viable_config) | leap = anchor + MAX_LEAP. |
| **c5d67cc Component 2: LADDER-DOWN floor** | `break_target_with_anchor(config_ladder_down(...))` at arq_commander.cc:5080 (v1) + 5255 (v2, after c5d67cc) | NEW consumer added by c5d67cc — floors the LADDER-DOWN target at the anchor. |

### §2.2 The two collapse half-mechanisms and the two fixes

**Collapse half A (LADDER-DOWN marches past the anchor):** pre-c5d67cc, the success-rate
LADDER-DOWN (arq_commander.cc:5080 v1 / 5255 v2) used raw `config_ladder_down()` floored
ONLY by `config_is_at_bottom` (ROBUST_0). Sustained low success walked the config rung-by-
rung to the FLOOR, past a proven-viable rung. → **c5d67cc Component 2 fixes this** (route
both LADDER-DOWN targets through `break_target_with_anchor` + a never-go-UP guard).

**Collapse half B (the breaks≥2 panic jumps straight to ROBUST_0):** arq_commander.cc:3486-
3492 sets `break_drop_step=100` on the SECOND consecutive BREAK-without-clean-data, and
`break_target_with_anchor()` bypasses the anchor floor under panic (arq.h:149) → the next
BREAK recovery lands at ROBUST_0. This fires **regardless of whether the breaking config is
the proven anchor or a speculative over-shoot.** → **FIX-2 fixes this.**

The over-climb dynamic produces collapse half B: the link over-climbs to (e.g.) CONFIG_13
while the anchor is still CONFIG_4. CONFIG_13 fails twice (it can't carry mid-SNR data) →
panic latches at the OVER-SHOT rung → ROBUST_0 collapse, even though CONFIG_4 (the anchor)
was never given a chance. **FIX-2 gates the panic latch on `current_configuration <=
last_data_viable_config`** so only the ANCHOR rung failing twice (a genuine cliff) panic-
collapses; a speculative over-shoot instead routes through the normal one-rung BREAK descent
(`break_drop_step` stays at its doubling 1→2→4…) which c5d67cc Component 2 floors at the
anchor → the link DEMOTES to the proven rung and HOLDS.

### §2.3 Valid states & invariants the FIX-2 consumers assume

INVARIANT I1 (deep-cliff escape preserved): a genuine cliff — where even the ANCHOR rung
fails repeatedly — MUST still reach ROBUST_0. FIX-2 preserves this: when `current ==
last_data_viable_config` (the anchor itself is breaking), the gate passes and the panic
latches exactly as today. The `anchor_consec_break_fails` demotion (3517-3531) ALSO keeps
lowering the anchor on consecutive anchor-rung BREAKs, so a sustained deep cliff walks the
anchor down toward ROBUST_0 and the panic + Component-2 floor follow it down. Escape latency
at a TRUE cliff is unchanged.

INVARIANT I2 (over-shoot does not collapse): when `current > last_data_viable_config` (a
speculative rung above the proven anchor), FIX-2 SKIPS the panic latch. The BREAK still fires
(`break_drop_step` doubling), and c5d67cc Component 2 floors the recovery target at the
anchor (arq.h:150-151, panic bypass NOT taken because the counter never reached 2) → the
link steps down to the anchor and HOLDS. This is the DEMOTE-1-and-hold behavior.

INVARIANT I3 (`break_target_with_anchor` panic bypass stays keyed on the SAME counter): both
the panic latch (3486) and the anchor-floor bypass (arq.h:149) read
`breaks_since_last_data_success >= 2`. FIX-2 gates the INCREMENT-to-latch, NOT the bypass
threshold — so the two stay consistent: if FIX-2 prevents the counter from reaching 2 at an
over-shoot, the bypass also does not fire (the floor stays active). They cannot desync.

### §2.4 What FIX-2 changes (assumption walk over consumers)

FIX-2 alters one assumption at the panic latch: "two BREAKs without clean data ⟹ crash,
jump to ROBUST_0." It adds "AND the breaking config is at-or-below the proven anchor." Walk:
- `break_target_with_anchor` (the bypass consumer, arq.h:149): unaffected by the gate itself;
  it keys on the counter value, which FIX-2 simply keeps at <2 during an over-shoot. Under a
  real anchor-cliff the counter still reaches 2 and the bypass fires (I1). ✓
- `break_drop_step` consumers (BREAK recovery at arq_commander.cc:209-212, 307-311): receive
  the normal doubling (1→2→4) instead of 100 during an over-shoot → the recovery drops in
  bounded steps, floored at the anchor by Component 2. ✓
- Anchor DEMOTE (3517-3531): unaffected — it is keyed on `current == anchor` and
  `anchor_consec_break_fails`, an independent counter. A deep cliff still demotes the anchor. ✓

---

## §3. c5d67cc diff summary (the existing local branch)

`c5d67cc` (HEAD of `fix/sustainable-config-hold`, parent `730ffca`). Two files, two
components.

**Component 1 — RETRIGGER_MAX_LEAP 13 → 4** (`include/common/common_defines.h`, the
`#define RETRIGGER_MAX_LEAP` at the ~line-399 block on 730ffca, hunk `@@ -388,12 +388,25 @@`):
- Old `#define RETRIGGER_MAX_LEAP 13`; new `4`. Rationale block rewritten (cites the HW
  Muething campaign): 13 was the WHOLE OFDM tier (CONFIG_0 idx3 + 13 = idx16 = CONFIG_13),
  so it never bounded the over-climb. 4 makes the climb prove-as-it-goes (anchor+4 ≈ one
  constellation+code-rate step; CONFIG_0→4→8→12→16 over a bulk transfer). Single chokepoint
  (`supershift_retrigger_target` arq.h:865-867) → covers all three promotion paths; deep-cliff
  INERT (gated on `is_ofdm_config(anchor)` at arq.h:854, so a ROBUST anchor still clamps to
  anchor+1).
- **CONFIRMED INERT WITHOUT FIX-1:** the leap_cap line (arq.h:865-867) sits AFTER the
  `if(optimizer_owns) return snr_ideal;` early-return (arq.h:834). Above CONFIG_6 on a noisy
  channel, `optimizer_is_in_control()==true` (index-only) → the early-return fires → the
  leap_cap is never reached → MAX_LEAP=4 has NO effect. **FIX-1 is the precondition that makes
  Component 1 bite above CONFIG_6.**

**Component 2 — anchor-floor LADDER-DOWN** (`source/datalink_layer/arq_commander.cc`, two
twin sites):
- v1 (`finalize_block_commander`, hunk `@@ -5077,7 +5077,24 @@`): replaces
  `negotiated_configuration = config_ladder_down(current, robust_enabled)` with
  `raw_down = config_ladder_down(...); floored = break_target_with_anchor(raw_down);
  if(idx(floored) > idx(current)) floored = current; negotiated = floored;`.
- v2 (`policy_evaluate_axis1`, hunk `@@ -5238,7 +5255,33 @@`): identical replacement on the
  SACK-v2 policy path.
- Effect: a success-rate LADDER-DOWN never drops BELOW the proven anchor (`break_target_with_
  anchor` floors UP to `last_data_viable_config`), EXCEPT under the breaks≥2 panic (where
  `break_target_with_anchor` returns raw → descent to ROBUST_0 preserved). The never-go-UP
  guard handles the over-climbed case (anchor ≥ current → HOLD at current, don't re-climb
  into the failing config via the down path).
- Tests: Part R added to `test_climb_engine()` (hunk `@@ -8500,6 +8583,143 @@`), R0-R2
  (Component 1 over-climb bound, fail-before models MAX_LEAP=13, deep-cliff INERT), R3-R5
  (Component 2 LADDER-DOWN hold + panic bypass). Existing assertions J1/J4/J5/JJ2/JJ3/FP-J2/
  K'2/K''4 re-parameterised on the macro (numeric landing follows MAX_LEAP, intent preserved).

**c5d67cc does NOT touch `optimizer_is_in_control()` or the breaks≥2 panic latch** — those
are FIX-1 and FIX-2 respectively, the missing pieces of the unit.

---

## §4. The exact hunks (IMPLEMENTATION-READY)

> Apply on top of `c5d67cc` (which is on base `730ffca`). Line anchors are 730ffca/c5d67cc.

### §4.1 FIX-1 — `include/datalink_layer/arq.h` (the `optimizer_is_in_control()` body, ~2053-2058)

```cpp
  bool optimizer_is_in_control() const {
    if (optimizer_disabled || !rate_opt.is_enabled()) return false;
    int handoff = rate_opt.min_calibrated_cfg(narrowband_enabled == YES);
    if (handoff <= 0) return false;
    if (config_ladder_index(current_configuration) < config_ladder_index(handoff))
      return false;
    // AUTHORITY GAP FIX (optimizer-authority-gap.md §1): index-in-band is NOT
    // sufficient to claim the optimizer is STEERING THIS CHANNEL. On a noisy /
    // uncalibrated channel the EVM-SNR over-reports and the climb reaches an
    // in-band index, but the optimizer's own batch-end evaluator RECUSES on the
    // channel (opt_evaluate_batch_end, arq_common.cc:3330-3331) — so it sets no
    // opt_pending_switch_cfg and steers nothing, while this predicate (pre-fix)
    // returned true and DISABLED the gearshift anchor + RETRIGGER_MAX_LEAP clamps
    // in supershift_retrigger_target() (arq.h:834). Neither controller then bounds
    // the climb → over-climb → collapse. Mirror the SAME max_sack recusal the
    // optimizer uses, so the two hand off at the identical boundary: the optimizer
    // is "in control" ONLY when the live channel is within the calibrated SACK-rate
    // range. (get_current_sack_rate()==0.0 at window=0 → 0.0 <= max_sack → does not
    // recuse at cold start, symmetric with opt_evaluate_batch_end.)
    double max_sack = rate_opt.max_calibrated_sack_rate(narrowband_enabled == YES);
    if (max_sack >= 0.0 && get_current_sack_rate() > max_sack)
      return false;
    return true;
  }
```

Rationale for the EXACT mirror: `opt_evaluate_batch_end()` (arq_common.cc:3328-3331) reads
`min_cfg = rate_opt.min_calibrated_cfg(is_nb)` and `max_sack =
rate_opt.max_calibrated_sack_rate(is_nb)` and recuses on `current < min_cfg ||
get_current_sack_rate() > max_sack`. FIX-1 uses the identical accessors and the identical
sack-rate comparison so the two predicates are guaranteed symmetric (no drift). The min_cfg
axis is already represented (the `idx(current) >= idx(handoff)` line, where `handoff =
min_calibrated_cfg`); FIX-1 adds the max_sack axis that was missing.

NOTE on `apply_optimizer_handoff_cap_to_target()` (arq.h:2061-2071, the sibling turbo cap):
it does NOT call `optimizer_is_in_control()` — it independently re-derives `handoff`. Leave
it AS-IS. It only CAPS the turbo target at the handoff config; it never UNCLAMPS, so it is
not part of the authority gap. (If a future audit wants symmetry there too, that is a
separate, lower-risk change — out of scope for this unit.)

### §4.2 FIX-2 — `source/datalink_layer/arq_commander.cc` (the breaks≥2 panic latch, ~3485-3492)

Pre-fix (730ffca:3485-3492):
```cpp
				breaks_since_last_data_success++;
				if(breaks_since_last_data_success >= 2)
				{
					printf("[BREAK-PANIC] %d BREAKs without data success — "
						"forcing jump to ROBUST_0 (break_drop_step=100)\n",
						breaks_since_last_data_success);
					fflush(stdout);
					break_drop_step = 100;  // clamp at floor of ladder
				}
```

Post-fix:
```cpp
				breaks_since_last_data_success++;
				// DWELL-CONFIRM (optimizer-authority-gap.md §2.2/§2.3): only the ANCHOR
				// rung failing twice is a genuine cliff that warrants the panic jump to
				// ROBUST_0. A SPECULATIVE OVER-SHOOT (current ABOVE the proven anchor —
				// the over-climb the §15/MAX_LEAP bound + FIX-1 are meant to prevent, and
				// the residual the gearshift may still produce) must NOT collapse the link:
				// it routes through the normal one-rung BREAK descent (break_drop_step
				// doubling 1→2→4) which c5d67cc Component 2 floors at last_data_viable_config
				// → the link DEMOTES to the proven rung and HOLDS, instead of cratering to
				// ROBUST_0 and re-climbing from scratch. The deep-cliff escape is PRESERVED:
				// when current <= anchor the latch fires exactly as before, and the §10
				// anchor DEMOTE (arq_commander.cc:3517-3531) still walks the anchor down on
				// sustained anchor-rung BREAKs so a true cliff reaches ROBUST_0. (I1/I2/I3.)
				if(breaks_since_last_data_success >= 2 &&
				   config_ladder_index(current_configuration) <=
				   config_ladder_index(last_data_viable_config))
				{
					printf("[BREAK-PANIC] %d BREAKs without data success at/below anchor "
						"(cfg=%d, anchor=%d) — forcing jump to ROBUST_0 (break_drop_step=100)\n",
						breaks_since_last_data_success, current_configuration,
						last_data_viable_config);
					fflush(stdout);
					break_drop_step = 100;  // clamp at floor of ladder
				}
				else if(breaks_since_last_data_success >= 2)
				{
					printf("[BREAK] %d BREAKs without data success but cfg=%d is ABOVE anchor "
						"%d (over-shoot) — anchor-floored descent, NOT a panic collapse\n",
						breaks_since_last_data_success, current_configuration,
						last_data_viable_config);
					fflush(stdout);
					// break_drop_step keeps its doubling (set elsewhere); Component 2 floors
					// the recovery target at the anchor → DEMOTE-1-and-hold.
				}
```

INTERACTION CHECK with `break_target_with_anchor()` (arq.h:149): under the over-shoot branch,
`break_drop_step` is NOT 100 and `breaks_since_last_data_success` IS ≥2 — so
`break_target_with_anchor` would take its `>=2 → return raw_target` panic bypass (arq.h:149),
which would DEFEAT Component 2's floor. **This must be reconciled.** Two equivalent options;
recommend Option A (tighter, keeps the counter honest):

- **Option A (recommended): gate the `break_target_with_anchor` panic-bypass on the SAME
  anchor predicate.** Change arq.h:149 from
  `if(breaks_since_last_data_success >= 2) return raw_target;`
  to
  `if(breaks_since_last_data_success >= 2 && config_ladder_index(current_configuration) <=`
  `   config_ladder_index(last_data_viable_config)) return raw_target;`
  This makes the panic bypass and the panic latch use the IDENTICAL predicate (I3 holds by
  construction), so an over-shoot's recovery is floored at the anchor (Component 2 effective)
  while a true anchor-cliff still bypasses to ROBUST_0. NOTE: `break_target_with_anchor` is
  `const`; `current_configuration` and `last_data_viable_config` are members readable in a
  const method (verified — the existing body already reads `last_data_viable_config` at
  arq_commander.cc:150). No signature change needed.

- **Option B: do NOT increment `breaks_since_last_data_success` on an over-shoot BREAK at all**
  (move the `++` inside the `current <= anchor` guard). Then the counter never reaches 2 at an
  over-shoot, so BOTH the latch AND the arq.h:149 bypass stay inactive without touching
  arq.h. Cleaner single-site change, BUT it changes the SEMANTICS of the counter (it becomes
  "consecutive ANCHOR-rung BREAKs without clean data"), which subtly overlaps with
  `anchor_consec_break_fails` (3518) — risk of conceptual drift. Prefer A.

**Ship FIX-2 as the arq_commander.cc:3485 hunk ABOVE + the Option-A arq.h:149 one-line gate.**

### §4.3 arq.h:149 (`break_target_with_anchor`) — Option-A companion to FIX-2

Pre-fix (arq_commander.cc:147-153):
```cpp
int cl_arq_controller::break_target_with_anchor(int raw_target) const
{
	if(breaks_since_last_data_success >= 2)
		return raw_target;  // panic-jump safety net — let it reach ROBUST_0
	if(config_ladder_index(raw_target) < config_ladder_index(last_data_viable_config))
		return last_data_viable_config;
	return raw_target;
}
```
Post-fix:
```cpp
int cl_arq_controller::break_target_with_anchor(int raw_target) const
{
	// DWELL-CONFIRM (optimizer-authority-gap.md §4.3): the panic bypass fires ONLY
	// when the breaking config is at/below the proven anchor (a genuine cliff) —
	// IDENTICAL predicate to the breaks>=2 panic latch (arq_commander.cc:3486). An
	// over-shoot above the anchor does NOT bypass: it floors UP to the anchor so
	// c5d67cc Component 2's LADDER-DOWN hold + the BREAK recovery both DEMOTE-and-
	// hold at the proven rung instead of collapsing to ROBUST_0.
	if(breaks_since_last_data_success >= 2 &&
	   config_ladder_index(current_configuration) <=
	   config_ladder_index(last_data_viable_config))
		return raw_target;  // panic-jump safety net — true cliff, let it reach ROBUST_0
	if(config_ladder_index(raw_target) < config_ladder_index(last_data_viable_config))
		return last_data_viable_config;
	return raw_target;
}
```

---

## §5. How the three compose (no double-clamp, no deadlock)

The unit = c5d67cc (Component 1 leap_cap + Component 2 LADDER-DOWN floor) + FIX-1 (authority
predicate) + FIX-2 (panic dwell-confirm + arq.h:149 companion). They act on DISJOINT decision
points and chain cleanly:

```
 CLIMB  ── supershift_retrigger_target (arq.h:830) ─────────────────────────────┐
        FIX-1 decides optimizer_owns:                                           │
          calibrated channel → owns=true  → return snr_ideal (optimizer steers; │
                                            its own opt_pending_switch_cfg path  │
                                            applies the move; leap_cap N/A)      │
          noisy/uncalibrated  → owns=false → fall through to the leap_cap        │
                                            (c5d67cc Component 1: anchor+4)  ◄────┘ MAX_LEAP bites
        ─ single clamp applies (anchor+MAX_LEAP); NO double-clamp because the
          early-return and the leap_cap are mutually exclusive branches.

 DOWN   ── success-rate LADDER-DOWN (arq_commander.cc:5080/5255) ──
        c5d67cc Component 2 floors at the anchor (break_target_with_anchor),
        never-go-UP guard prevents re-climb. ONE floor, applied once per move.

 BREAK  ── breaks>=2 latch (arq_commander.cc:3486) ──
        FIX-2 + arq.h:149 companion: over-shoot → NOT panic, recovery floored at
        anchor (Component 2 path) → DEMOTE-and-hold;
        anchor-cliff → panic latch + bypass → ROBUST_0 (deep-cliff escape).
```

**No double-clamp:** Component 1 (leap_cap) bounds UPWARD moves; Component 2 + FIX-2 bound
DOWNWARD/recovery moves. They never both fire on the same move. FIX-1 gates which branch of
the UPWARD chokepoint runs (optimizer early-return vs leap_cap) — exactly one runs.

**No deadlock:** §1.3 proved no reader of `optimizer_is_in_control()` relies on index-only-
true to escape a state; downward authority (BREAK/LADDER-DOWN/panic) is never gated on it;
the optimizer's own-move path (opt_pending_switch_cfg, arq_commander.cc:541) is independent
of FIX-1 and re-engages the moment the channel returns to the calibrated region. FIX-2 keeps
the deep-cliff escape (I1) and the panic-bypass/latch symmetry (I3). The worst case anywhere
is the pre-optimizer gearshift behavior, which is self-bounded (ladder + BREAK + anchor).

**Why one unit:** c5d67cc's leap_cap is INERT above CONFIG_6 without FIX-1 (the `optimizer_
owns` early-return at arq.h:834 jumps over it on the noisy channel). FIX-2 without Component 2
would floor the recovery at the anchor but the success-rate LADDER-DOWN would still march
past it. Component 2 without FIX-2 would floor LADDER-DOWN but the breaks≥2 panic would still
crater an over-shoot to ROBUST_0. The three close the three independent leaks of the SAME
over-climb→collapse pathology; shipping any subset leaves a residual leak that masks the win
on HW (the documented "sim-validated batch fix moved ZERO on HW" failure mode).

---

## §6. The sim test (the over-climb→collapse regression)

**Repro target:** the FTRT ARQ sim (being built concurrently on `feat/sim-clock`; touches
arq.h:2263 `opt_now_ms`, arq_common.cc PTT busy-spins ~3814/3885/4040/4132, timer.cc,
audioio.c, main.cc). The sim must drive the modem through a climb on a channel whose EVM-SNR
over-reports relative to the OFDM-data-viable rate (mid-SNR, sack_rate above the calibrated
max), and record the `switch_seq` (the sequence of `current_configuration` values).

**Assertion (fail-before / pass-after):**
- `repro_overclimb_collapse`: TRUE on the UNFIXED binary (parent `730ffca`, MAX_LEAP=13,
  index-only `optimizer_is_in_control`, ungated panic): `switch_seq` shows CONFIG_0 → a
  multi-rung leap to ≥CONFIG_12 → (fail) → COLLAPSE to ROBUST_0. The collapse signature:
  `min(switch_seq[after the leap]) == ROBUST_0` AND the link does NOT re-establish a
  sustained mid config (no rung held for ≥N consecutive batches).
- `repro_overclimb_collapse`: FALSE on the FIXED unit (c5d67cc + FIX-1 + FIX-2): the climb
  rises in bounded steps (each leap ≤ anchor+4), and once a rung proves non-sustainable the
  link DEMOTES one rung and HOLDS — `switch_seq` STOPS collapsing to ROBUST_0 after the
  climb. Assert: `ROBUST_0 not in switch_seq[after first OFDM rung]` AND a single
  mid-config is held for ≥N consecutive batches (the sustainable rung).

**Caveat (from c5d67cc commit msg + this audit):** plain loopback is near-lossless → the
high config never FAILS → the over-climb→collapse dynamic cannot manifest. The FTRT sim MUST
inject a channel model where the high OFDM config's batch success_rate falls below
`gear_shift_down_success_rate_precentage` while the EVM-SNR (post-EQ, saturates ~14.5) still
maps several rungs high AND `get_current_sack_rate()` exceeds the Q-table's
`max_calibrated_sack_rate`. Without that injected mismatch the test cannot fail-before. If
the FTRT sim cannot inject per-config success rates, fall back to the in-process
`--test-climb-engine` Part R primitives (which c5d67cc already added: R0-R2 prove the bound,
R3-R5 prove the hold) PLUS two new Part-R cases that FIX-1/FIX-2 add:
  - **R6 (FIX-1):** set `current_configuration >= handoff` and `get_current_sack_rate() >
    max_calibrated_sack_rate` (stub a noisy window); assert `optimizer_is_in_control()==false`
    (PASS-AFTER) where the index-only predicate returned true (FAIL-BEFORE). Then drive
    `supershift_retrigger_target(CONFIG_16, snr20, anchor=CONFIG_8, optimizer_owns=
    optimizer_is_in_control(), ...)` and assert the result == `config_ladder_up_n(CONFIG_8,
    RETRIGGER_MAX_LEAP,...)` (CONFIG_12), NOT CONFIG_16 — i.e. the leap_cap BITES above
    CONFIG_6 only because FIX-1 made owns=false.
  - **R7 (FIX-2):** `current=CONFIG_13`, `last_data_viable_config=CONFIG_4`,
    `breaks_since_last_data_success` incremented to 2 at the over-shoot; assert the panic
    latch does NOT fire (`break_drop_step != 100`) and `break_target_with_anchor(
    config_ladder_down(CONFIG_13))` floors to CONFIG_4 (HOLD), NOT ROBUST_0 (PASS-AFTER);
    FAIL-BEFORE: `break_drop_step == 100` and the recovery reaches ROBUST_0. Then the
    anchor-cliff case `current=CONFIG_4==anchor`, breaks=2 → panic latches (`break_drop_step
    ==100`) and `break_target_with_anchor` bypasses → ROBUST_0 (I1 preserved, PASS both).

**HW arbiter (post-sim):** `tools/muething_throughput.py` (writes `muething_results.json`,
compare against `.BASELINE_*.json`). SNR3k = WGN_label + 2.4. The win condition is the
degraded cells STOP ending pinned at ROBUST_0 and instead HOLD a sustainable mid config that
beats VARA's B/min in that cell. The sim gates the HW run; do not skip it (CLAUDE.md §3).

---

## §7. Open questions [?]

- [?] FIX-1 sack-axis at exactly the handoff rung on a TRANSIENT noise spike: a single noisy
  window could flip `optimizer_is_in_control()` false mid-transfer, handing a calibrated cell
  briefly back to gearshift. The `opt_window` is a rolling mean (arq.h:2247-2253) so a single
  spike is damped; `opt_evaluate_batch_end()` already tolerates this (same input). Likely
  benign — confirm on HW that calibrated cells do not thrash optimizer↔gearshift. Add an OPT
  log line if observed.
- [?] Component 1 MAX_LEAP=4 clean-climb speed: c5d67cc notes "try 5/6 if clean climbs too
  slowly." The FTRT sim should also record clean-channel time-to-CONFIG_16 (CONFIG_0→4→8→12→16
  = ~4 bounded leaps amortized over a bulk transfer) to confirm the WGN:30 fast climb is not
  materially regressed. Not a correctness gate, a tuning datum.
- [?] `apply_optimizer_handoff_cap_to_target()` (arq.h:2061) symmetry: it caps the turbo
  target at the handoff config but re-derives handoff independently and ignores sack_rate.
  Out of scope for this unit (it only caps, never unclamps) but flagged for a future audit if
  the turbo path shows residual over-climb after this unit ships.

---

## §8. IMPLEMENTATION RECORD — built 2026-06-03, branch `fix/over-climb-authority` (off `c5d67cc`)

The unit was implemented exactly as §4 prescribed. As-built file:line (re-located by name on
`c5d67cc`; the analysis's 730ffca numbers held within a few lines):

### §8.1 FIX-1 — `include/datalink_layer/arq.h`, `optimizer_is_in_control()` body (`:2053-2077`)
Added the channel-recusal mirror AFTER the existing index check, mirroring
`arq_common.cc:3328-3331` EXACTLY (same `>= 0` / `>= 0.0` "table never loaded" sentinels):
```cpp
const bool is_nb = (narrowband_enabled == YES);
... (index check unchanged) ...
int    min_cfg  = rate_opt.min_calibrated_cfg(is_nb);   // arq.h:2072
double max_sack = rate_opt.max_calibrated_sack_rate(is_nb);
if (min_cfg >= 0 && current_configuration < min_cfg) return false;   // :2074
if (max_sack >= 0.0 && get_current_sack_rate() > max_sack) return false; // :2075
return true;
```
Const-correct: `get_current_sack_rate()` const (arq.h:2246), both rate_opt helpers const
(rate_optimizer.h:130/133), the method itself const. The `current_configuration < min_cfg`
half is REDUNDANT with the existing index check for OFDM configs (ladder index is monotonic
with config id, common_defines.h:144-157), so it is harmless; the **load-bearing new
discriminator is the `sack_rate > max_sack` half** — that is what recuses on a NOISY (but
high-config) channel, which is the authority gap the index-only predicate left open.

### §8.2 The 5-reader audit — VERDICT (all SAFER under tightening, NONE deadlocks)
Walked every reader of `optimizer_is_in_control()` on `c5d67cc`:
- **R1** `process_messages_rx_acks_data` FRAME-UP (arq_commander.cc:3709 +1 clamp, :3715
  `optimizer_owns_upward_frame`). Under FIX-1 on a noisy channel: `!optimizer_is_in_control()`
  now TRUE → the +1 clamp ACTIVATES (the desired bite) and `optimizer_owns_upward_frame`
  false → gearshift FRAME-UP no longer "yields to optimizer" but is bounded to anchor+1 by the
  clamp. Climbs one rung at a time. No deadlock.
- **R2** legacy LADDER-UP `finalize_block_commander` (:5000-5002 gate, :5013 +1 clamp). Gate
  opens (gearshift takes over) AND +1 clamp activates — bounded climb. No deadlock.
- **R3** v2 `policy_evaluate_axis1` (:5173-5175 gate, :5187 +1 clamp). Identical to R2.
- **R4** turbo SNR-SUPERSHIFT chokepoint (:4678-4681 passes `optimizer_is_in_control()` as the
  `optimizer_owns` arg to `supershift_retrigger_target`). THE KEYSTONE: the helper's
  `if(optimizer_owns) return snr_ideal;` early-return (arq.h:834-835) bypasses BOTH the +1
  anchor clamp AND the leap_cap (:865-867). Under FIX-1 on a noisy channel `optimizer_owns`
  becomes false → early-return NOT taken → the leap_cap BITES. This is the mechanism R6 proves.
  (Sibling `apply_optimizer_handoff_cap_to_target` at :4626 uses `min_calibrated_cfg` directly,
  NOT `optimizer_is_in_control()` — unaffected, separate narrower cap.)
- **R5** elevator `elevator_target_from_snr()` (:182-183 passes `optimizer_is_in_control()` to
  the same helper). Same as R4 — leap_cap/+1 clamp now bite on the elevator path. No deadlock.
- Optimizer's OWN-move path `opt_pending_switch_cfg` (:541-572) does NOT read
  `optimizer_is_in_control()`; it is driven by `opt_evaluate_batch_end()`, which recuses on the
  SAME predicate FIX-1 mirrors. So when the channel is noisy BOTH go silent together (was the
  gap: they disagreed); when the channel re-enters the calibrated region BOTH re-engage
  together. **Consistency restored — no deadlock, no double-clamp** (the early-return and the
  leap_cap are mutually exclusive: if optimizer_owns the cap is skipped, else it applies).

### §8.3 FIX-2 — `source/datalink_layer/arq_commander.cc`
- **Panic latch** (the breaks≥2 `break_drop_step=100`, `:3502-3540` region). Gated on
  `at_or_below_anchor = config_ladder_index(current_configuration) <=
  config_ladder_index(last_data_viable_config)`. Only an AT/BELOW-anchor BREAK latches the
  collapse; an OVER-SHOOT logs `[BREAK-PANIC] ... OVER-SHOOT ... NOT collapsing` and routes
  through the anchor-floored descent. Deep-cliff escape (I1) preserved: at the cliff
  current==anchor → idx<=idx TRUE → panic still fires.
- **Option-A companion** — `break_target_with_anchor()` (`:161-171`). The panic-bypass
  (`if(breaks>=2) return raw_target;`) is now gated with the IDENTICAL `at_or_below_anchor`
  predicate (`:148-159` comment, `:163-167`). Without this, the method would return the deep
  raw target on an over-shoot and defeat c5d67cc Component-2's floor. Const-correct (reads
  `current_configuration`, `last_data_viable_config`, `breaks_since_last_data_success`).
  Faithful to the live BREAK recovery site (arq_commander.cc:229): there
  `current_configuration == emergency_previous_config` (the breaking rung) until a new config
  loads, so the predicate references the same rung the panic latch keyed on — the two gates
  are coherent.

### §8.4 Cross-layer reconciliation of EXISTING panic-bypass unit tests (the §5 sibling-bug gate)
FIX-2 Option-A added a `current_configuration` dependency to `break_target_with_anchor`. Every
existing test that calls it under panic was audited:
- **A3** (`test_data_anchored_promote`, :6107): set `current_configuration=ROBUST_1`
  (==anchor) explicit → bypass fires, reaches ROBUST_0. PASS.
- **E4** (`test_climb_engine` Part E): set `current_configuration=CONFIG_0` (==anchor) explicit
  → bypass fires. PASS.
- **P2e** (`test_phantom_ack`): current=ROBUST_2 > anchor=ROBUST_0 (over-shoot) → bypass NO
  LONGER fires, BUT anchor==ROBUST_0==floor so the anchor floor itself returns ROBUST_0.
  Reach preserved; comment corrected to "anchor floor == ROBUST_0". PASS.
- **C2** (`test_clean_batch_viability` Part C): anchor==ROBUST_0==floor, same as P2e. PASS
  unchanged.
- **R5** (`test_climb_engine` Part R, `ladder_down_floored` helper): helper now sets
  `current_configuration=current` so it faithfully replays production; R5 (current==anchor,
  panic) still reaches CONFIG_3. PASS.
- **Q3** (`test_climb_engine` Part Q, :8615-8632): **BEHAVIOR INTENTIONALLY CHANGED BY FIX-2.**
  Was "panic-jump (breaks≥2, step=100) still floors to CONFIG_0 regardless of anchor" with
  expected CONFIG_0. The scenario is an OVER-SHOOT (breaking rung CONFIG_16, anchor CONFIG_13),
  which is EXACTLY what FIX-2 must not collapse. Updated: set `current_configuration=CONFIG_16`
  explicit, expect **CONFIG_13** (HOLD at anchor, no collapse). This is the unit-level
  manifestation of the fix. PASS.
- LADDER-DOWN twins (`finalize_block_commander:5134`, `policy_evaluate_axis1:5321`) — non-panic
  paths (breaks normally 0); when panic coincides, FIX-2 makes them MORE consistent with
  Component-2 (over-shoot floors at anchor); the `if(floored>current) floored=current`
  never-go-UP guard backstops. Strictly safer, no regression.

### §8.5 Tests R6 + R7 (synthetic-fire, channel-free) — added to `test_climb_engine()` Part R
- New test seam: `cl_rate_optimizer::set_test_calibration(min_cfg, max_sack, is_nb)`
  (rate_optimizer.h:114) marks the optimizer enabled + seeds the below-table-range gate inputs
  WITHOUT a JSON load. Not called by any production path.
- **R6** (FIX-1, :8789): seeds handoff=CONFIG_6, max_sack=0.30, current=CONFIG_8 (above
  handoff → index check passes), and a window with `get_current_sack_rate()==1.0` (noisy).
  R6a: `optimizer_is_in_control()==false` (recuses). R6c: feeding that into the REAL
  `supershift_retrigger_target(snr_ideal=CONFIG_16, anchor=CONFIG_8, ...)` bounds the target to
  anchor+RETRIGGER_MAX_LEAP = CONFIG_12 (the leap_cap bites). R6e: the pre-fix
  `optimizer_owns=true` path returns CONFIG_16 (the leak).
- **R7** (FIX-2, :8861): R7a — an over-shoot (cfg15 > anchor10, breaks≥2) does NOT latch
  `break_drop_step=100` (local mirror of the break-site gate). R7b — the REAL
  `break_target_with_anchor` floors the over-shoot's deep recovery target to the anchor
  CONFIG_10 (HOLD, no collapse). R7c/R7d — contrast: an AT-anchor BREAK (current==anchor)
  STILL latches 100 and reaches the floor (deep-cliff escape I1 preserved).

### §8.6 FAIL-BEFORE → PASS-AFTER (explicit, per CLAUDE.md §3)
Temporarily reverted FIX-1 (commented out the recusal in `optimizer_is_in_control()`) AND
FIX-2 Option-A (restored the unconditional `if(breaks>=2) return raw_target;`), rebuilt,
re-ran `--test-climb-engine`: **5 FAILURES** — R6a (got=1 want=0: predicate stayed true),
R6c (got=16 want=12: leap_cap bypassed → CONFIG_16 over-climb), R6d (target==CONFIG_16),
R7b (got=0 want=10: over-shoot cratered to the floor CONFIG_0). Restored both fixes from
backup (0 TEMP-REVERT markers remain), rebuilt → `--test-climb-engine` **ALL PASS (0
failures)**, full `mercury.exe --test` **54 passed, 0 failed**. The revert reproduces the
exact over-climb→collapse signature the unit eliminates.

### §8.7 Build & suites (synthetic-fire only — no audio device opened)
`bash build.sh o3` clean (pre-existing warnings only) → copied to `C:\Program Files\Mercury\
mercury.exe`. `--test` = 54/0; `--test-climb-engine` = ALL PASS incl R6/R7. HW muething A/B on
the MID cells (wgn20/wgn10 — does it HOLD a sustainable high config vs over-climb→collapse) is
the NEXT step (bench, no audio), NOT run here.

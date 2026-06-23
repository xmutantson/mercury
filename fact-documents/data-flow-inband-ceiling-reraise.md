# Data-flow: `supershift_proven_ceiling` under the in-band no-BREAK redesign

Owner structure: `supershift_proven_ceiling` (cl_arq_controller member, the
"never-climb-above-this-proven-rung" cap consumed by the FRAME-UP / SUPERSHIFT
climb gates). This doc covers the IN-BAND deep-stall bug and the re-raise fix.
Built 2026-06-22 from the inband A/B 97B-vs-168B (~0.58x) loss.

All `file:line` refs are `source/datalink_layer/arq_commander.cc` unless noted.
Lines are at the `feat/inband-a3-decouple` HEAD this doc was written against
(aae73c9); re-verify if the file shifts.

## §1 The bug (VERIFIED code + the prior A/B log)

Under the in-band redesign (`MERCURY_INBAND_RATE=1`) a Class-A degradation routes
through `inband_route_failure_demote()` (the no-BREAK tag-demote) instead of the
legacy BREAK→ROBUST cascade. That demote PINS the ceiling at the demoted rung:

  - `:3073`  `supershift_proven_ceiling = demote_target;`  (the pin — producer #P1)

The ONLY production sites that RE-RAISE the ceiling are turbo / SUPERSHIFT /
BREAK-recovery paths:

  - `:2415`  turbo-direction new_ceil lower-only update
  - `:2942`  turbo new_ceiling lower-only update
  - `:5043`  (climb path) lower-only update
  - `:5547`  ROBUST→CFG16 turbo re-climb reset to `start_config`
  - `:6556`  turbo finish reset
  - `:6934 / :6976`  SNR-retrigger raise-to-`proposed` / negotiated
  - `:7104 / :7146`  duplicate retrigger block raise

EVERY one of those lives behind a turbo / BREAK / SUPERSHIFT entry the no-BREAK
inband demote DELIBERATELY never fires. So after an inband degrade the ceiling
stays pinned at the demoted rung forever, and the FRAME-UP gate walls all climb:

  - `:5340-5342`  `frame_ceiling_blocked = (ceiling>=0 && index(proposed_frame) >
                   index(ceiling)) || ...`   (consumer #C1, the wall)

Net: 0 FRAME-UP attempts after a degrade → deep_stall → the inband arm delivers
97B where legacy (which re-climbs via BREAK→turbo) delivers 168B (~0.58x).
BORROWED-ASSUMPTION bug: legacy's demote was ALWAYS paired with a BREAK-recovery
re-raise; the no-BREAK inband path breaks that pairing and nothing re-raises.

## §2 Producers of `supershift_proven_ceiling`

PINS / lowers (demote): #P1 `:3073` inband demote; `:2635/:2801` BREAK ladder-down;
`:4734` CFG16 carve fallback; `:4901` D3 frame-up-unfollowable; `:4364` connect floor.
RAISES: the turbo/SUPERSHIFT/BREAK sites listed in §1.
RESETS to -1 (no cap): `:7938/:8004/:8117/:9167/:9940` (session reset / test).
NEW (this fix): the inband clean-batch re-raise at the FRAME-UP clean branch
(`:~5305`, gated `inband_rate_feature_enabled()`), via the pure helper
`inband_ceiling_raise_target()` (arq.h).

## §3 The fix — option (b), inband-scoped clean-batch re-raise

Pure helper `cl_arq_controller::inband_ceiling_raise_target(current_ceiling,
anchor, streak_config, clean_streak)` in `include/datalink_layer/arq.h`
(immediately after `data_anchor_raise_target`). It MIRRORS the anchor-raise:

  - no-op when `current_ceiling < 0` (no inband demote happened → nothing to lift);
  - requires `clean_streak >= sustained_anchor_threshold(streak_config)` (the §11
    sustained-anchor bar — robust N=1 / OFDM N=2): the demoted rung must
    RE-PROVE itself sustained-clean;
  - only ever RAISES, and CAPS the raise at `anchor` (== `last_data_viable_config`,
    the §1.1 confirmed-delivery anchor): the ceiling never exceeds PROVEN ground.

Production call (clean-batch branch, right after the `data_anchor_raise_target`
raise so it consumes the freshly-updated anchor):

```
if(inband_rate_feature_enabled())
    supershift_proven_ceiling = inband_ceiling_raise_target(
        supershift_proven_ceiling, last_data_viable_config,
        clean_batches_config, clean_batches_at_current_config);
```

Why option (b) over option (a) (skip the pin entirely when inband): (a) removes
the cap so the SOLE bound becomes the `+1` anchor clamp; but the inband demote
does NOT lower `last_data_viable_config` (§5), so a stale-high anchor under (a)
would license the FRAME-UP `+1` to leap toward the failed rung on the FIRST clean
batch (no sustained proof) → re-demote thrash. (b) gates the re-raise on N
sustained-clean batches and caps it at the anchor, so re-climb is evidence-based.

## §4 Consumers of `supershift_proven_ceiling` — does the fix break any?

  - #C1 FRAME-UP gate `:5340` — the fix raises the cap so a permitted +1 probe is
    no longer walled. Still bounded: the `+1` anchor clamp `:5348-5350`
    (`index(proposed) > index(anchor)+1 → blocked`) is UNTOUCHED, so FRAME-UP
    still advances ONE rung per promotion and never leaps above anchor+1.
  - SUPERSHIFT/SNR gate `:9328` (test) and `:6885/:7052` (retrigger) — same cap
    read; raising it only widens the permitted set, still anchor-bounded.
  - `snr_ideal` clamp `:218`, elevator caps `:218/:6398` — these CAP a target DOWN
    to the ceiling; raising the ceiling can only let a high-SNR target through,
    and that target is itself `last_data_viable_config`-clamped by the retrigger
    helper (`:220`, SAFETY #2/#3). No new over-climb.

NO consumer assumes the ceiling is monotone-non-increasing within a session
(the turbo/BREAK raise sites already raise it), so a raise here violates no
invariant.

## §5 Over-climb / legacy-byte-identical verification

OVER-CLIMB: the ceiling can rise only to `last_data_viable_config` (PROVEN
delivery), and the independent `+1` anchor clamp still caps each FRAME-UP at
anchor+1. A too-high rung that fails to deliver triggers the gearshift's own
decode-failure demote (`frame_gearshift_data_failed_nack` `:4337` /
`frame_gearshift_data_failed_pat` `:4559` → `inband_route_failure_demote` →
re-pin). So the loop self-corrects; no runaway.

LEGACY BYTE-IDENTICAL: the production call is gated on
`inband_rate_feature_enabled()` (false in legacy → the line is never reached).
Even if it were reached with no inband demote, `current_ceiling < 0` makes the
helper an identity no-op. The legacy turbo/BREAK ceiling discipline is untouched.

STALE ANCHOR (the one residual): the inband demote never lowers
`last_data_viable_config` (no BREAK-path anchor demote `:5004-5039` fires).
A stale-high anchor lets the ceiling rise above the demoted rung+1 — but the `+1`
clamp still forces ONE-rung-per-promotion climb with per-rung clean-delivery
proof, and a failing rung re-demotes. This is a PRE-EXISTING condition (the `+1`
clamp already reads the same stale anchor); the fix does not worsen it.

## §6 Regression test (CLAUDE.md §3, fail-before/pass-after)

`--test-climb-engine` Part X (arq_commander.cc test_climb_engine):
  - drives the REAL `inband_ceiling_raise_target` helper (the pure decision the
    production branch calls), under fail-before macro `-DINBAND_CEILING_FAILBEFORE`;
  - FAIL-BEFORE: helper returns the pinned ceiling (deep_stall) → the modelled
    FRAME-UP gate stays blocked → assertion fails;
  - PASS-AFTER: after N sustained-clean batches the helper raises the ceiling to
    the anchor → the FRAME-UP gate releases → config rises above the demoted rung.

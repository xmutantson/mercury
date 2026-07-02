# Data-flow audit — `data_configuration` / `forward_configuration` on the RSP in-band adopt path

Structure owned: the config-copy triad `current_configuration`, `data_configuration`,
`forward_configuration` (+ `reverse_configuration`, `negotiated_configuration`,
`last_data_configuration`) as they relate to the RSP adopting a new forward rung under
`MERCURY_INBAND_RATE`. Companion to `wb-dwell-post-leap.md` (the symptom trace).

Built 2026-07-01 for the SUPER-ACK ROBUST->WB leap-stick fix (branch
`integ/superack-cleanlock`). All line refs are `source/datalink_layer/*.cc` @ HEAD
`17dcd795`.

## 0. The bug (one paragraph)

After the SUPER-ACK ROBUST->WB leap the CMD advances all its forward config copies
coherently (`current_configuration = data_configuration = forward_configuration = CONFIG_8`
via `inband_unilateral_config_change`, arq_common.cc:3478-3479+3487) and tags CONFIG_8 on
the passband. The RSP hears the tag and follows via
`detect_and_follow_config_tag` -> `inband_adopt_resynced_config`
(arq_common.cc:3388 / :5631), but that helper only calls
`load_configuration(followed_config, PHYSICAL_LAYER_ONLY, NO)` (arq_common.cc:5642), which
writes ONLY `current_configuration` (arq_common.cc:2243). It does NOT touch
`data_configuration` (still ROBUST_0 = 100 from the robust session start,
arq_common.cc:1895) or `forward_configuration`. The next forward CONTROL frame the RSP
receives + ACKs runs the post-ACK restore-data-config step (arq_responder.cc:1800), sees
`data_configuration(100) != current_configuration(8)`, and does `load_configuration(100)`
-> snaps the RSP PHY back to ROBUST while the CMD stays at CONFIG_8. Link diverges,
forward delivery freezes, the CMD's reverse-SACK cheap-miss cap escalates to BREAK.
Deterministic.

## 1. PRODUCERS (writers)

### `data_configuration`
- arq_common.cc:859  ctor default `= CONFIG_0`.
- arq_common.cc:1895 / :1901 session init `= initial_mode` (**ROBUST_0=100 for a robust
  start** — the value present at leap time on the RSP).
- arq_common.cc:3479 **CMD unilateral change** `= target_cfg` (the coherent CMD advance).
- arq_common.cc:7332 recovery/ladder-down `= config_ladder_down(...)`.
- arq_common.cc:8111 reset `= init_configuration`.
- arq_common.cc:13603 / :14784 climb/settle winners (CMD side).
- arq_responder.cc:536 / :678 robust-seat / SET_CONFIG-adjacent RSP writes.
- arq_responder.cc:1978 SWITCH_ROLE asymmetric `= forward_configuration`.
- arq_responder.cc:3385 / :3440 **LEGACY SET_CONFIG RSP handler** `= forward_configuration`
  (the exact path CONFIG_TAG replaced — the reference the fix mirrors).
- **GAP (the bug):** `inband_adopt_resynced_config` (arq_common.cc:5631) — the shared
  RSP tag-follow + down-ladder adopt helper — does NOT write it. **Fix adds it here.**

### `forward_configuration`
- arq_common.cc:860 ctor default `= CONFIG_NONE`.
- arq_common.cc:3478 **CMD unilateral change** `= target_cfg`.
- arq_common.cc:8112 reset `= CONFIG_NONE`.
- arq_common.cc:14785 climb winner (CMD side).
- arq_commander.cc:657/751/1346/1352/7702 CMD negotiation writers.
- arq_responder.cc:1968 SWITCH_ROLE swap `= reverse_configuration`.
- arq_responder.cc:3367 **LEGACY SET_CONFIG RSP handler** `= data[1]` (forward wire field).
- **GAP (the bug):** not written by `inband_adopt_resynced_config`. **Fix adds it here.**

Both tag-follow (arq_common.cc:3388) and the Stage-4 down-ladder resync
(arq_common.cc:5601) funnel through the ONE helper `inband_adopt_resynced_config`, so a
single edit in the helper closes both entry points.

## 2. CONSUMERS (readers) — RSP-reachable, each re-verified against the fix

1. **arq_responder.cc:1800** post-ACK restore-data-config (`if(data_configuration !=
   current_configuration) load_configuration(data_configuration,...)`). **This is the bug's
   trigger.** After the fix `data_configuration == current_configuration == 8` -> the branch
   is a **no-op** (correct: no spurious revert to ROBUST). VERIFIED-GOOD.
2. **arq_responder.cc:1758/1764, :1820, :2766** the LDPC/robust-ACK restore family (send ACK
   on `ack_configuration` MFSK, then `load_configuration(data_configuration)` to return to
   the data rung). Pre-fix these restored to ROBUST_0 (wrong); post-fix they restore to
   CONFIG_8 (correct). VERIFIED-HELPED.
3. **arq_responder.cc:1769/1806/1823** `inband_finalize_ofdm_adopt_ring(data_configuration)`
   — guarded by `is_ofdm_config(data_configuration)`. Pre-fix `data_configuration=100`
   (robust) -> the ring-shrink is skipped; post-fix `=8` (OFDM) -> ring correctly finalized
   to natural geometry (which the adopt helper already did at :5648 anyway; idempotent).
   VERIFIED-GOOD.
4. **arq_common.cc:2100** `parallel_monitor_decode` try-order HINT: prioritizes
   `forward_configuration` then `reverse_configuration` as the first decode attempts, guarded
   `fwd >= 0 && fwd < NUMBER_OF_CONFIGS`. Post-fix `forward_configuration=8` makes the monitor
   try CONFIG_8 FIRST (exactly right). A robust rung (100+) fails the `< NUMBER_OF_CONFIGS`
   guard and is silently skipped as a hint (harmless). VERIFIED-HELPED.
5. **arq_responder.cc:1955** SWITCH_ROLE `has_asymmetric = (forward_configuration != NONE &&
   reverse_configuration != NONE)`. THE ONLY consumer whose *branch selection* changes:
   pre-fix `forward_configuration` was never set in-band (stuck at its init/last value, often
   CONFIG_NONE) so `has_asymmetric` could be FALSE; post-fix it is CONFIG_8 (non-NONE) and, on
   a tier-cross, `reverse_configuration` is already pinned by the existing reverse-pin
   (arq_common.cc:3414) -> `has_asymmetric` becomes TRUE. **This is legacy-equivalent, not a
   regression:** in a LEGACY gearshifted session both fields are always set by SET_CONFIG
   (arq_responder.cc:3367-3368), so `has_asymmetric=TRUE` is the *normal* legacy state and the
   asymmetric forward/reverse swap (arq_responder.cc:1967-1980) is the *designed* behavior the
   reverse-pin was built to feed. The in-band path merely omitted the forward half; the fix
   restores it. VERIFIED-GOOD (matches legacy).
6. **CMD-side reads** (arq_commander.cc gearshift/negotiation; arq_common.cc:3487 the
   unilateral load) run only on the COMMANDER role. `inband_adopt_resynced_config` /
   `detect_and_follow_config_tag` run only on the RSP receiving forward data (all production
   callers are `rx->...` in arq_responder.cc: :599/:1046/:4969/:6880/:7365/:7422/:5231/:5698/
   :9017/:7892). The CMD receives only reverse ACK/SACK and never runs the tag-follow, so the
   fix cannot clobber the CMD's forward-config ownership. VERIFIED-NOT-REACHED-ON-CMD.
7. **Test-only** `test_inband_adopt_ring_*` callers (arq_responder.cc:8654/8698/8723/8767/
   8833) assert ONLY ring state (`ring_write_index`, `ofdm_search_raw`, peak), never the
   config triad. VERIFIED-UNAFFECTED.

## 3. VALID STATES

- Default-init: `data_configuration=CONFIG_0`, `forward_configuration=CONFIG_NONE`.
- Robust session start (the leap scenario): `data_configuration=ROBUST_0(100)`,
  `forward_configuration` = its init/last value.
- Post-leap, PRE-fix (broken): `current_configuration=8`, `data_configuration=100`,
  `forward_configuration` unchanged -> DIVERGENCE at arq_responder.cc:1800.
- Post-leap, POST-fix (correct): `current_configuration = data_configuration =
  forward_configuration = 8` -> coherent with the CMD.

## 4. INVARIANTS consumers assume, and whether the fix preserves them

- **INV-1**: the restore-data-config consumers (`:1800/:1758/:1820`) assume
  `data_configuration` names the rung the RSP should be seated at to receive/continue
  forward data. Pre-fix the adopt left it stale (violated the invariant silently); the fix
  RE-ESTABLISHES the invariant. No consumer wanted `data_configuration` to stay at ROBUST_0
  as a "fallback" — the ONLY intentional fallback restore in this codebase is via `cleanup()`
  / BREAK / `reset_session_state()` which re-seat the triad explicitly (arq_common.cc:8111-
  8112), NOT via a stale `data_configuration`.
- **INV-2**: `has_asymmetric` assumes `forward_configuration`/`reverse_configuration` reflect
  the *current* asymmetric split. The fix makes the in-band path honor this exactly as the
  legacy SET_CONFIG path does. No consumer treats RSP-side `forward_configuration==CONFIG_NONE`
  as a required sentinel (the only NONE-check is the `has_asymmetric` predicate, which is
  legacy-driven-TRUE anyway).

## 5. WHAT THE FIX CHANGES (one assumption altered, re-verified per consumer)

Single change: in the shared helper `inband_adopt_resynced_config`
(arq_common.cc:5631), immediately after
`load_configuration(followed_config, PHYSICAL_LAYER_ONLY, NO)`, add
`data_configuration = followed_config;` and `forward_configuration = followed_config;`
(mirroring the CMD pair at arq_common.cc:3478-3479 and the legacy SET_CONFIG RSP handler
at arq_responder.cc:3367+3385/3440).

- Altered assumption: "the RSP adopt advances only `current_configuration`." Now it advances
  the full forward triad, matching the CMD and the legacy handler.
- Consumer #1 (:1800): becomes a correct no-op. GOOD.
- Consumers #2/#3: restore to the correct rung. GOOD/HELPED.
- Consumer #4 (:2100): better decode hint. HELPED.
- Consumer #5 (:1955 has_asymmetric): legacy-equivalent activation. GOOD.
- Consumers #6/#7: not reached on CMD / test-only ring asserts. UNAFFECTED.

**No consumer breaks. Ship.** `negotiated_configuration` and `last_data_configuration` are
deliberately NOT touched (the legacy RSP SET_CONFIG handler does not set them either; the
task scope is the CMD pair, and the simple thing is correct pre-ship).

## 6. Regression coverage

`--test-inband-leap-stick` (source/datalink_layer/arq_responder.cc): build an RSP at
ROBUST_0 (`data_configuration=100`), drive `inband_adopt_resynced_config(CONFIG_8)`
(the production tag-follow adopt), then assert `current_configuration==8 &&
data_configuration==8 && forward_configuration==8`. FAIL-BEFORE: pre-fix
`data_configuration`/`forward_configuration` stay at 100/init. PASS-AFTER: all three == 8.

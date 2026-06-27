# Gearshift start-config regression + promotion/recovery bugs

**Status:** diagnosed 2026-05-29 (3 parallel research agents). Fix direction
proposed, PENDING owner approval. Owner is the original author of the early-2026
gearshift/turboshift logic. Companion to [[turbo-cascade-timeout.md]] (whose
backstop fix is now understood to paper over Bug 1 below).

## §1 Symptom (v7 hardware run, WGN:-10, `-Q 0 -M auto -g -R`)

After CONNECT (HAIL handshake completes at ROBUST_0/MFSK), the modem immediately
blind-climbs the OFDM ladder CONFIG_1→2→…→7 — climbing AWAY from the known-good
ROBUST_0 it just connected on — delivers 0 bytes, the 45 s backstop aborts it to
ROBUST_0, data flows at ROBUST_0+ROBUST_1, then the steady-state gearshift
promotes it to ROBUST_2 where data STALLS and never recovers for ~290 s.

## §2 Three distinct root-cause bugs

### Bug 1 — Start config is CONFIG_1, not ROBUST_0 (CONFIRMED REGRESSION)
The deployed Pi binary is a GUI build (`build.sh:159` always sets
`-DMERCURY_GUI_ENABLED`; deploy builds with it). In GUI builds the
"gearshift-on + no explicit `-s` ⇒ start ROBUST_0" default is **compiled out**:
- `main.cc:2040-2042`: `#ifndef MERCURY_GUI_ENABLED  mod_config = ROBUST_0; #endif`
- `main.cc:2032` (GUI branch): `mod_config = g_settings.initial_config`, which
  defaults to **CONFIG_1** (`ini_parser.cc:215`). Deploy excludes `.ini`
  (`mercury_deploy_rpi.py:56`), so the Pi keeps CONFIG_1.
→ `init_configuration = CONFIG_1` → `current_configuration = CONFIG_1` through the
all-MFSK handshake (handshake never calls load_configuration) → turboshift starts
the climb from CONFIG_1, not ROBUST_0 (`arq_commander.cc:3835`
`turboshift_last_good = current_configuration`).

**Git archaeology + owner clarification (2026-05-29):**
- **Working design = `2d3a667` (2026-02-12)**: introduced turboshift; set
  `mod_config = ROBUST_0` for gearshift-without-`-s` for ALL builds; forward probe
  = climb +1 from ROBUST_0 on ACK, latch `turboshift_last_good`, declare ceiling
  on NAck, probe reverse, then data. This is the owner's intended design.
- **`25fe06b` (2026-02-15)** added a GUI config path (`mod_config =
  g_settings.initial_config`) and gated the forced-ROBUST_0 to `#ifndef
  MERCURY_GUI_ENABLED`. **The `#ifdef` fork is INTENTIONAL and correct** (owner
  confirmed): it lets GUI users *configure* their start config — gearshift-off /
  fixed-config / known-good-channel use. The actual regression is narrower: the GUI
  default it reads (`g_settings.initial_config`) was left at **CONFIG_1**
  (`ini_parser.cc:215`) instead of ROBUST_0. So a GUI build with no/default INI
  starts mid-OFDM-ladder. **Fix = change the default, keep the fork.**

### Bug 2 — FRAME-UP promotes on pure ACK-count, no data-viability check
Steady-state gearshift (`arq_commander.cc:3343-3390`): `consecutive_data_acks >=
frame_shift_threshold` (=3, `arq_common.cc:334`) → `config_ladder_up` one rung.
No SNR, no effective-rate, no test that the next rung carries data — only the
historical `supershift_proven_ceiling`/`max_config_override` caps. So 3 ACKs at
ROBUST_1 jumped to ROBUST_2 (rate 1/16 → 1/4, ~4× weaker FEC) blindly
(`cmd.log:10918`). At WGN:-10 ROBUST_2 can't carry data; nothing checked.

### Bug 3 — Phantom (noise-driven) ACK
> **CORRECTION (2026-05-29, SHIPPED fix — supersedes the entry-point analysis
> below).** The phantom's ENTRY POINT is the **no-CRC bare `receive_ack_pattern()`
> arm** of the clean-batch DATA-ACK acceptance at `arq_commander.cc:2891`
> (`!sack_window_open && receive_ack_pattern()`), NOT a CRC-passing noise match in
> `detect_ack_pattern`'s mirror logic. `receive_ack_pattern()` (arq_common.cc:5556)
> returns a BARE bool on `matched_count >= ack_match_threshold && metric >=
> ack_metric_threshold` — NO CRC, NO content validation. The WGN:-10 phantom
> (`cascade_diag_wgn-10/...cmd.log:11507`: matched=7, metric=0.66, on CONFIG_0, with
> NO corresponding RSP transmission) fired through this bare arm, set
> `data_ack_received=YES`, raised `last_data_viable_config` to CONFIG_0, and reset
> `breaks_since_last_data_success=0` — so BREAK could never escape to ROBUST_0
> (24-BREAK thrash). The three REAL data ACKs in that same run all arrived via the
> CRC12-validated MFSK-suffix path (matched=16, valid CRC12) — i.e. the
> `v2_ack_pat_pre_detected` branch, NOT the bare arm.
>
> ~~The earlier "v7 phantom" note (`cmd.log:12932`, "suffix energies all 0.0,
> matched=16") was conflated with this bug.~~ That v7 event was a **REAL,
> CRC-valid RSP ACK**, not a phantom — a different phenomenon. The
> `detect_ack_pattern` mirror-bin regression analysis below remains a real
> sensitivity of the scorer, but it is NOT the entry point of the 24-BREAK thrash
> and the ABSOLUTE-ENERGY floor that was built to address it (`mfsk_detect_min_peak_energy`,
> commit `901f092`) was **REVERTED as non-viable** (`aecb561`) — real and phantom
> energy overlap, so no energy floor cleanly separates them.
>
> **SHIPPED FIX (this session): a CONTENT/CRC gate on the bare arm.** The clean
> discriminator is CONTENT, not energy: a real WB data ACK carries a CRC12-valid
> MFSK suffix; the phantom does not. The bare arm now requires (WB only) a
> CRC12-valid, in-window, clean-batch suffix before accepting; NB / suffix-incapable
> sessions (RSP sends a bare ACK pattern — `arq_responder.cc:1679-1683`) keep
> bare-pattern acceptance. See **§8** for the full producer/consumer audit, the
> fix, and the regression test (`--test-phantom-ack-gate`). The original
> mirror-bin analysis (historical, for the scorer's separate sensitivity) follows.

**WHEN/WHY (git-traced 2026-05-29):** the false-alarm was introduced in `55354671`
("NB/WB negotiation, MFSK ARQ optimizations, NB OFDM freq sync, mutex fix", authored
2026-02-21). It rewrote the per-symbol match in `cl_ofdm::detect_ack_pattern`
(ofdm.cc:3636 — the SINGLE scorer all ACK/SACK/SNR/CONNECT/BREAK/HAIL detection
funnels through):
- **Added mirror-bin acceptance** (ofdm.cc:3741): match now passes if argmax ==
  expected_bin OR its mirror `(Nfft-expected_bin)%Nfft`. Old test (28d82045,
  2026-02-09) required expected to BE the band argmax (one bin, P≈1/M on noise).
  For WB ACK the mirror bins land IN-BAND → genuine 2nd candidate → ~doubles
  per-symbol noise-match probability.
- **Doubled the metric numerator** (ofdm.cc:3714-3717): `e_target = e_expected +
  e_mirror` (was `e_expected`). Metric is a scale-invariant fraction
  `Σ e_target/e_total` with NO absolute-energy floor; only guard is `peak_e > 0`
  (catches exactly-zero only, not the ~1e-8 near-silence v7 showed as `e=0.00`).
- Made for NB carrier-image recovery (Bug #39) — the mirror IS real for NB. So the
  fix CANNOT simply revert; it must add an absolute signal-presence floor while
  preserving mirror acceptance for NB. Validate across all 6 callers (§5 audit).
- Dormant Feb→now [inferred] because it only bites in deep-SNR silent ACK-polling
  windows — the regime the WGN:-10 cliff work newly exercises.
- NOT b328a4d / the 2026-05-28 preamble-discrete-match (that changed
  `time_sync_mfsk_corr`, the data PREAMBLE detector — a DIFFERENT function; classic
  preamble≠ACK trap). NOT the metric-threshold changes.
- Same `detect_ack_pattern` gates the turboshift climb AND starves BREAK recovery →
  one fix addresses both the probe over-reach and the no-recovery.

### Bug 3 (original symptom notes) — Phantom (noise-driven) ACK defeats BREAK recovery
At ROBUST_2 the link was mute (RSP `nReceived_data` frozen at 18). BREAK-down
needs `emergency_nack_count >= 3` (`arq_common.cc:378`), incremented per block
failure (`arq_commander.cc:3247`), but **reset to 0 on ANY accepted ACK**
(`arq_commander.cc:3307`, the `data_ack_received==YES` branch). A single MFSK
ACK-suffix decoded from **zero-energy noise** (`cmd.log:12932`, `bitmap=0x00000001`,
suffix energies all 0.0, `matched=16`) set `data_ack_received=YES` between the two
real block failures (`cmd.log:12638`, `14341`), resetting the counter so it never
reached 3 → BREAK never fired (`grep "Sending emergency BREAK pattern" = 0`). The
ACK-acceptance sites (`arq_commander.cc:2772` SACK_RSP, `:2857` MFSK) have no
`rx_count>0` / non-zero-energy gate. This both inflates CMD nAcked (24→25 vs RSP's
frozen 18) AND starves the safety counter. No independent throughput watchdog
exists; the Q-table optimizer is dormant below its calibrated OFDM band
(`arq.h:1286-1291`) so it can't downshift ROBUST_2 either.

## §3 Unifying principle

All three (and the original turbo-cascade bug) are one disease:
**control-plane success (SET_CONFIG ACK, MFSK ACK) does NOT imply data-plane
viability — and the gap is large at deep SNR.** The system promotes on
control-plane signals (cascade climb, FRAME-UP) and resets its safety counters on
control-plane signals (even noise-faked ones), so it climbs into and gets stuck at
configs where data cannot flow. At moderate SNR control≈data viability, which is
why the owner's original design worked then and the failure only surfaces now at
the WGN:-10 frontier.

## §4 Proposed fix direction (PENDING owner approval)

1. **Bug 1 (start ROBUST_0) — SHIPPED (local) 2026-05-29:** changed the GUI INI
   default `initial_config` CONFIG_1 → ROBUST_0 (`ini_parser.cc:215`, +
   `setup_dialog.cc:70`, + `#include common_defines.h`). KEPT the `#ifdef` fork
   (intentional GUI configurability — a GUI user can still raise the start config).
   main.cc reverted to original. Build clean, 30/30 tests. Pending hardware verify.
2. **Bug 3a (phantom ACK):** gate ACK/SACK acceptance on non-zero suffix energy /
   `rx_count>0` so noise can't fake an ACK (`arq_commander.cc:2772`, `:2857`).
   Restores the BREAK safety counter AND fixes nAcked/nReceived divergence.
3. **Bug 2 + 3b (data-viability promotion + recovery):** promotion (FRAME-UP and/or
   the forward probe ceiling) gated on actual DATA delivery (RSP-confirmed), not
   control-ACK count — so it finds the rung where data flows (ROBUST_1 at WGN:-10)
   and stops; and a stalled rung drops back (BREAK-down, now un-starved by 3a, or a
   throughput watchdog). This is the deeper re-engineering.

Relationship to [[turbo-cascade-timeout.md]]: the 45 s backstop + abort + latch
papered over Bug 1's blind-climb. With Bug 1 fixed it becomes a true last-resort
net (rarely fires) — keep as defense-in-depth or remove (owner decision).

## §5 Open design questions for the owner (gearshift author)
- In your original design, was forward-probe promotion meant to climb on
  control-ACK success or on confirmed DATA delivery? (Archaeology shows 2d3a667
  climbed on ACK; the deep-SNR failure suggests data-anchoring is needed now.)
  **ANSWERED (owner, 2026-05-29): data-anchored — see §6 Option B.**
- Expected WGN:-10 outcome (owner stated): probe should discover ROBUST_1 is the
  ceiling (data flows at ROBUST_0/1, not ROBUST_2) and STAY at ROBUST_1.

## §6 Option B — data-anchored gearshift promotion (IMPLEMENTED 2026-05-29)

**Owner decision:** promotion climbs on CONFIRMED DATA DELIVERY, not control-ACK
success. The start config is already ROBUST_0 (Bug 1 fix shipped, §4.1). This
section is the design of record for the promotion/recovery rework.

> **BASELINE NOTE (important for git archaeology):** the `monitor` HEAD is
> `aecb561`, but the WORKING TREE at the time Option B was written already
> contained substantial UNCOMMITTED gearshift work that is NOT in HEAD:
> the SUPERSHIFT `turbo_supershift_announce_pending` re-entry guard (arq.h ~1268,
> 0 occurrences at HEAD / 8 in the tree), the 2026-05-29 no-SNR `+1` SUPERSHIFT
> caps ("no-SNR fallback drove cfg1->cfg16" comments), and the Bug-1 start-config
> fix (§4.1 "SHIPPED (local)" = uncommitted). Option B was layered ON TOP of that
> tree. Therefore `git diff aecb561 -- arq_commander.cc` MIXES pre-existing work
> with Option B; the Option-B-only sites are identifiable by the `Option B`
> comment marker and the `last_data_viable_config` / `break_target_with_anchor` /
> `test_data_anchored_promote` identifiers. Build + 30/30 `--test` confirm Option B
> is compatible with the pre-existing tree.

### §6.1 Principle
`§3` established the disease: control-plane success ≫ data-plane viability at
deep SNR. Option B closes the gap by anchoring every promotion and recovery
decision to the highest rung that has actually carried a data batch this
session:

> **`last_data_viable_config`** = highest ladder rung (by `config_ladder_index`)
> at which a DATA batch was *confirmed delivered* (clean ACK / SACK_RSP /
> LDPC-ACK) since the session began.

Rules:
- **Start at ROBUST_0, go straight to data.** No control-only SUPERSHIFT probe.
- **Climb only on data.** FRAME-UP still promotes on `consecutive_data_acks`,
  but no up-shifter may pick a destination more than +1 rung above
  `last_data_viable_config` (the probe rung), unless the Q-table optimizer owns
  the band (`optimizer_is_in_control()`).
- **Recover to the data-viable rung, not the floor.** BREAK target is floored at
  `last_data_viable_config`. EXCEPTION: the `breaks_since_last_data_success >= 2`
  panic-jump to ROBUST_0 is preserved as the safety net (a hard-cratered channel
  must still be able to reach the absolute floor).
- **Q-table handoff UNCHANGED.** `optimizer_is_in_control()` / `min_calibrated_cfg`
  gate the optimizer's authority over CONFIG_6+; Option B never touches it and
  always yields to it.

### §6.2 The five changes (file:line in the working tree post-implementation)
Line numbers below are the ACTUAL post-edit working-tree lines (verified by grep
2026-05-29), not the pre-edit estimates from the task brief.
1. **Disable SUPERSHIFT control-probe** at the 3 entry sites
   (`arq_commander.cc` ~1948 post-SWITCH_BANDWIDTH-fail-NB, ~3763
   START_CONNECTION-ACK, ~3969 SWITCH_BANDWIDTH-ACK-WB). The true-branch (compute
   snr_target → `add_message_control(SET_CONFIG)` → `TRANSMITTING_CONTROL`) is
   replaced by the existing else-branch behavior: `turboshift_active=false;
   turbo_supershift_announce_pending=false; turboshift_phase=TURBO_DONE;
   connection_status=TRANSMITTING_DATA`. All handshake/role/NB-WB side effects
   (`switch_narrowband_mode`, `wb_upgrade_pending=false`, the `[BW-NEG]` prints)
   execute BEFORE these blocks and are untouched. Net: data starts at ROBUST_0;
   no control-only climb.
2. **`last_data_viable_config`** added to `cl_arq_controller` (`arq.h`, with the
   turbo/gearshift ceiling members ~1269). Init to `init_configuration` in the
   ctor (`arq_common.cc` ~345) and in `reset_session_state` (~2896). SET to
   `current_configuration` at the confirmed-data-success path
   (`arq_commander.cc:3284`, the `data_ack_received==YES` branch — proven by
   brace-trace, see §6.3). This is the SOLE producer on delivery.
3. **FRAME-UP lock-before-promote / hysteresis.** FRAME-UP promote
   (`arq_commander.cc:3338`) still fires on `consecutive_data_acks >=
   frame_shift_threshold`, but its destination is now subject to the §6.2-5 anchor
   gate. The 3 post-promote-fail BREAK paths already double `frame_shift_threshold`
   (`:3010` nack, `:3184` pat, `:2210` control-NAck) — confirmed present, kept.
   These BREAK paths drop `data_configuration`/`negotiated_configuration` and set
   `supershift_proven_ceiling` but DO NOT touch `last_data_viable_config` (only a
   delivered batch does), so a failed probe can't inflate the anchor.
4. **BREAK recovers to `last_data_viable_config`** (`arq_commander.cc:81`). After
   `target = config_ladder_down_n(emergency_previous_config, break_drop_step, ...)`,
   floor by ladder index: if `config_ladder_index(target) <
   config_ladder_index(last_data_viable_config)` → `target = last_data_viable_config`.
   GATED OFF when `breaks_since_last_data_success >= 2` so the panic-jump to
   ROBUST_0 (set via `break_drop_step=100` at `:3253`) survives.
5. **Gate the other two up-shifters + the retrigger.** A destination whose
   `config_ladder_index` exceeds `config_ladder_index(last_data_viable_config)+1`
   is blocked unless `optimizer_is_in_control()`:
   - legacy ladder LADDER UP (`arq_commander.cc:4583`): `data_anchor_blocked`
     folded into the existing `ceiling_blocked` test.
   - `policy_evaluate_axis1` LADDER UP (`:4731`): same.
   - SUPERSHIFT-retrigger (`:4344`): re-trigger suppressed when `snr_ideal` index
     > anchor+1 and not optimizer-controlled.

### §6.2.1 As-built line table (working tree, 2026-05-29)
| # | What | File:line |
|---|------|-----------|
| 2 | member decl | `include/datalink_layer/arq.h:1293` |
| 2 | ctor init | `arq_common.cc:349` |
| 2 | reset_session_state init | `arq_common.cc:2904` |
| 2 | producer (on delivery) | `arq_commander.cc:3284` (in `else` at :3282 = data_ack_received==YES) |
| 4 | helper `break_target_with_anchor` | `arq_commander.cc:67` (decl `arq.h:557`) |
| 4 | BREAK floor call (ACK-recv) | `arq_commander.cc:100` |
| 4 | BREAK floor call (exhausted) | `arq_commander.cc:199` |
| 1 | SUPERSHIFT entry — post-NB-SWITCH_BANDWIDTH-fail | `arq_commander.cc:~1987` |
| 1 | SUPERSHIFT entry — START_CONNECTION-ACK | `arq_commander.cc:~3775` |
| 1 | SUPERSHIFT entry — SWITCH_BANDWIDTH-ACK-WB | `arq_commander.cc:~3930` |
| 5 | FRAME-UP anchor gate | `arq_commander.cc:3330` |
| 5 | SUPERSHIFT-retrigger cap | `arq_commander.cc:4271` |
| 5 | legacy ladder LADDER-UP gate | `arq_commander.cc:4513` |
| 5 | policy_evaluate_axis1 LADDER-UP gate | `arq_commander.cc:4668` |
| T | test method `test_data_anchored_promote` | `arq_commander.cc:5458` (decl `arq.h:564`) |
| T | CLI flag parse + dispatch | `main.cc:728`,`main.cc:1680` |

### §6.3 Brace-trace proof that :3284 is the data-success anchor
The task brief and the source comments call `:3284` "the data_ack_received==YES
branch", but visual indentation suggested it was nested inside
`if(data_ack_received==NO)` at `:3130`. Resolved mechanically (awk brace-depth
counter over `:3060`-`:3372`, relative to the `:3064` outer-else body = depth 0):
- `:3072 if(data_ack_received==NO && pattern>0)` opens/closes at depth 0.
- `:3130 if(data_ack_received==NO)` opens at depth 0; body (`:3179`,`:3224`
  emergency_nack_count++, `:3233 if(threshold)`) at depth ≥1; the threshold-if has
  NO trailing else.
- **`:3282 else` is at depth 0** → it is the `else` of `:3130`'s
  `if(data_ack_received==NO)`, i.e. the **`data_ack_received==YES`** path. NOT the
  else of the inner `:3233` threshold-if (that would be depth 1).
So `:3284 emergency_nack_count=0 "Reset on success"` executes exactly when a batch
was confirmed delivered. Correct anchor confirmed.

### §6.4 fails-before / passes-after signal
- **In-process:** `mercury.exe --test-data-anchored-promote` (synthetic-fire,
  added with this change) primes `last_data_viable_config=ROBUST_1`,
  `current_configuration=ROBUST_2`, and asserts: (a) BREAK-target calc floors at
  ROBUST_1 (not ROBUST_0); (b) up-shifter from anchor=ROBUST_1 permits ROBUST_2
  (anchor+1) but blocks CONFIG_0 (anchor+2); (c) `optimizer_is_in_control()`
  bypasses the gate. Exits 0 on pass, 1 on fail. (Pre-change the BREAK target
  would land at ROBUST_0 and the up-shifter would permit CONFIG_0 → assertions
  fail.)
- **IONOS cascade bench (the system-level validating test, owner runs it):**
  TODAY (aecb561): unpinned `-Q 0 -M auto -g -R` at WGN:-10 rides control-ACKs to
  CONFIG_13, delivers 0 bytes. AFTER Option B: settles at ROBUST_1
  (= `last_data_viable_config` once a ROBUST_0/1 batch delivers), data flows,
  recovers to ROBUST_1 on stall instead of climbing to high configs.

## §7 Producer/consumer audit (CLAUDE.md §5) — promotion state

Covers `last_data_viable_config` (new) and the existing promotion-state members
it interacts with: `negotiated_configuration`, `supershift_proven_ceiling`,
`data_ack_received`. One-time audit for the Option B change.

### §7.1 `last_data_viable_config` (NEW)
**Type/home:** `int`, `cl_arq_controller` member (`arq.h` ~1269).

**Producers (writers):**
- `arq_common.cc` ctor (~345): `= init_configuration` (default-init before any
  session).
- `arq_common.cc reset_session_state` (~2896): `= init_configuration` (per-session
  reset; called on connect/disconnect/role-swap — same sites as
  `supershift_proven_ceiling=-1`).
- `arq_commander.cc:3284` (data_ack_received==YES success path): `=
  current_configuration`. SOLE on-delivery producer. Fires once per confirmed
  batch (clean MFSK ACK `:2834`, SACK_RSP `:2749`, LDPC ACK_RANGE/ACK_MULTI
  `:2931/:2951` all funnel here via `data_ack_received=YES` → `:3282 else`).

**Consumers (readers):**
- `arq_commander.cc:81` (BREAK target floor): clamps `target` up to it, gated off
  under panic.
- `arq_commander.cc:4583`/`:4731` (legacy + axis1 LADDER UP gate): blocks
  destinations > index+1 unless optimizer in control.
- `arq_commander.cc:4344` (SUPERSHIFT-retrigger gate): suppresses retrigger past
  index+1.

**Valid states & default-init:** before the first confirmed batch it equals
`init_configuration` (= ROBUST_0 for `-R` gearshift sessions, per Bug-1 fix). So
at session start the BREAK floor and up-shifter gate both anchor at ROBUST_0 —
correct: nothing has carried data yet, so recovery to ROBUST_0 and a +1 probe to
ROBUST_1 are exactly the desired behavior. The value is monotonic-up within a
config epoch (only the success path raises it) and is reset only at
session/role boundaries via `reset_session_state`.

**Invariants:**
- I1: index(`last_data_viable_config`) ≤ index(highest delivered rung). Held: only
  `:3284` raises it, and only after `data_ack_received==YES`.
- I2: never below ROBUST_0. Held: `init_configuration` floor + `current_configuration`
  is always a valid ladder member.
- I3: a FAILED probe never raises it. Held: the FRAME-UP-fail BREAK paths
  (`:3010/:3184/:2210`) and the turbo-forward BREAK (`:2097`) write
  `supershift_proven_ceiling`/`negotiated_configuration` but NOT
  `last_data_viable_config`.

### §7.2 `negotiated_configuration` (existing — interaction check)
**Producers:** all the SUPERSHIFT/FRAME-UP/LADDER/BREAK sites set it as the next
SET_CONFIG target (`:81 (=target via load)`, `:100`, `:1977/3792/3999` SUPERSHIFT,
`:3340` FRAME-UP, `:4271/4595/4742` LADDER, BREAK paths `:3042/3200/2230`).
**Consumers:** the SET_CONFIG TX path + the SET_CONFIG-ACK handler
(`load_configuration` at ~3155) which is the canonical place `current_configuration`
advances. **Option B impact:** Change 1 stops writing it in the 3 entry sites
(no SET_CONFIG queued → it keeps its `reset_session_state` value =
init_configuration = ROBUST_0). Change 4/5 only NARROW the values it can take
(floor up on BREAK, cap on UP). No consumer assumption violated: it still always
holds a valid ladder member and is still the SET_CONFIG argument.

### §7.3 `supershift_proven_ceiling` (existing — interaction check)
**Producers:** lowered on BREAK/LADDER-DOWN/turbo-fail (`:3014/2107/2176/2264/
3258/4663`), raised by CEILING RECOVERY (`:4621`), set at turbo-forward finish
(`:3433`), reset to -1 in ctor/reset/NB-switch (`:345/2896/1949`).
**Consumers:** every up-shifter already caps its target at it (`:3324/3782/4261/
4590/4737`). **Option B impact:** Change 5's anchor gate is ADDITIVE to (logically
ANDed with) the existing ceiling gate — `last_data_viable_config+1` and
`supershift_proven_ceiling` are independent caps; the tighter one wins. No
interaction hazard: both are "don't climb past X" predicates. The anchor is
generally tighter at deep SNR (it tracks DATA, the ceiling tracks BREAK history
which §3 showed can sit too high because control frames decoded).

### §7.4 `data_ack_received` (existing — the anchor trigger)
**Producers:** set `YES` at `:2749/2834/2931/2951` (the four delivery-confirm
paths), default `NO` per batch at TX start (~`:1129`). **Consumers:** the entire
`:3000`/`:3064`/`:3130`/`:3294`/`:3309`/`:3331` decision tree, plus FRAME-UP gate
`:3331`. **Option B impact:** Change 2 adds ONE reader at `:3284` (inside the
`==YES` else). It does not alter when `data_ack_received` is set or cleared, so
no consumer of `data_ack_received` is affected. ~~The §3 phantom-ACK concern (Bug
3a, noise faking `data_ack_received=YES`) is OUT OF SCOPE for Option B (separate
fix on the ACK-acceptance energy gate) and is noted as a residual risk: a
phantom ACK would falsely raise `last_data_viable_config`. Mitigation deferred to
Bug-3a.~~ **UPDATE (2026-05-29): Bug-3a SHIPPED as a CONTENT/CRC gate (§8), not an
energy gate (that was reverted). After §8, `data_ack_received=YES` is CRC-gated on
ALL accept paths, so the phantom can no longer reach this `==YES` else block — the
anchor trigger is now trustworthy.** Option B never regressed it (a phantom would
also have reset `emergency_nack_count` at the same site, so the disease predates
Option B); §8 removes the entry point for both.

## §8 Bug 3a — phantom-ACK CONTENT/CRC gate (SHIPPED 2026-05-29)

The root-cause fix for Bug 3 (§2). **Energy was the wrong axis** (the
`mfsk_detect_min_peak_energy` floor, commit `901f092`, was reverted `aecb561`:
real/phantom energy overlap). **CONTENT is the clean axis:** a real WB data ACK
carries a CRC12-valid MFSK suffix; the phantom (which passes
`receive_ack_pattern()`'s bare matched/metric gate, arq_common.cc:5556) does not.

### §8.1 Producer/consumer audit (CLAUDE.md §5)

**The state being constrained:** `data_ack_received` set to `YES` on the
clean-batch DATA-ACK bare-pattern arm at `arq_commander.cc:2891`.

1. **Producer of `v2_ack_pat_pre_detected`** (the CRC-validated accept flag): set
   at `arq_commander.cc:2441` ONLY inside the CLEAN branch, reachable only after:
   `decode_ack_sack_from_passband()` succeeded (`:2381`) → CRC12 verified
   (`:2397-2398`, `rx_crc12 == CRC12_calc(...)`, else `decoded=false`) →
   `bsi_in_window && bitmap_ok && !duplicate` (`:2428`) → `rx_bitmap == all_ones`
   (`:2433`). **CONFIRMED strictly CRC-gated.** The design's assumption holds.
2. **Does RSP ALWAYS send the CRC suffix for a data ACK? NO.** Three legitimate
   bare-`send_ack_pattern()` (no-CRC) fallbacks exist:
   - NB / `ack_sack_suffix_len()==0` (suffix exists only for M≥16; `mfsk.h:136`):
     `arq_responder.cc:1679-1683`.
   - WB but `send_mfsk_ack_sack()` returned 0 (suffix send failed): same
     fallback, `arq_responder.cc:1675-1683`.
   - Non-v2 sessions (legacy): `arq_responder.cc:1685-1689`.
   → A blanket "require CRC" (variant 2a) would reject real NB / fallback ACKs.
   **Variant 2b (conditional gate) is required.**
3. **Callers of `receive_ack_pattern()`** (3): `arq_commander.cc:92` (emergency
   BREAK poll — CONTROL), `:1702` (control-ACK detection — CONTROL), `:2891` (the
   data-ACK bare arm — THE bug). Control + BREAK MUST keep bare-pattern behavior
   (control frames carry no suffix) — the fix touches ONLY `:2891`.
4. **Anchor/panic block** (`arq_commander.cc:3383` else of `if(data_ack_received
   == NO)` at `:3119`; brace-trace in §6.3): runs iff `data_ack_received==YES`.
   Raises `last_data_viable_config` (`:3396`) and resets
   `breaks_since_last_data_success` (`:3387`). After the §8.2 gate,
   `data_ack_received==YES` is CRC-gated on all paths (bare arm now gated;
   SACK_RSP at `:2749` CRC8-protected; v2 pre-detect CRC-gated) → the phantom
   cannot reach this block. The anchor trigger is trustworthy without further
   change.

### §8.2 The fix (variant 2b — conditional CONTENT gate, bare arm only)

`arq_commander.cc:2891` bare arm changed from
`(!sack_window_open && receive_ack_pattern())`
to
`(!sack_window_open && receive_ack_pattern() && (ack_sack_suffix_len() <= 0 || cmd_clean_data_ack_crc_valid()))`.

- **WB (suffix-capable):** a bare match is accepted ONLY if
  `cmd_clean_data_ack_crc_valid()` finds a CRC12-valid, in-window, clean-batch
  (all-ones) suffix in the current passband tail. The phantom (no suffix) is
  rejected → falls through to the normal timeout-retransmit path (safe).
- **NB / suffix-incapable (`ack_sack_suffix_len()==0`):** the `<= 0` short-circuit
  keeps bare-pattern acceptance — byte-for-byte unchanged, and NB never pays the
  FFT cost of the peek.
- **`cmd_clean_data_ack_crc_valid()`** (`arq_commander.cc`, new, ~line 60): a
  READ-ONLY tail peek (no `frames_to_read` mutation; mirrors the §7.13.30
  no-side-effect peek at `:2370`). Window math + decode + CRC12 + bsi-in-window +
  all-ones mirror the v2 pre-detect block (`:2351-2433`).
- **Pure policy predicate** `data_ack_bare_pattern_acceptable(suffix_capable,
  crc_suffix_valid)` (`arq.h`, `return suffix_capable ? crc_suffix_valid : true`)
  encodes the policy and is what the unit test drives; the call site uses the
  equivalent short-circuiting inline form `!suffix_capable || crc_valid`.
- **Step-3 companion (anchor/panic gate on rx-confirmed delivery): SKIPPED.** The
  data-success block (`:3383`) has NO in-scope `rx_count` signal (it's a SACK-path
  local). A clean full-batch ACK legitimately delivers data without an `rx_count`
  here; gating on a synthesized `rx_count>0` would risk suppressing legitimate
  clean-batch promotions (Option B's own mechanism). With `data_ack_received` now
  CRC-gated, the defense-in-depth is unnecessary and not worth the regression risk.

### §8.3 Regression test (`--test-phantom-ack-gate`)

`cl_arq_controller::test_phantom_ack_gate()` (arq_commander.cc, after
`test_data_anchored_promote`). Wired in main.cc (flag decl + parser + invoke,
mirrors `--test-data-anchored-promote`). 9 assertions:
- **Part 1 (pure policy matrix):** P1a WB phantom (no CRC) REJECTED ← THE bug
  assertion; P1b WB real ACK (CRC-valid) accepted; P1c/P1d NB bare accepted
  (suffix-independent).
- **Part 2 (cross-layer invariant):** with the phantom rejected, P2a
  `data_ack_received` stays NO; P2b `last_data_viable_config` NOT raised; P2c BREAK
  panic counter NOT reset; P2d/P2e BREAK still reaches ROBUST_0 under panic.

**FAIL-before / PASS-after VERIFIED:** temporarily forcing
`data_ack_bare_pattern_acceptable` to the pre-fix always-accept behavior (`return
true`) and rebuilding made the test FAIL at P1a (phantom accepted, got=1 want=0),
P2a (`data_ack_received`→YES), and P2b (`last_data_viable_config`→102=ROBUST_2 —
the exact anchor corruption that defeats BREAK recovery). Restoring the gate →
ALL PASS. Full `mercury.exe --test` = 30 passed / 0 failed in both builds.

## §9 CLEAN-BATCH VIABILITY — partial SACK must not promote the rung (SHIPPED 2026-05-29)

**Baseline:** `monitor` HEAD `af14a9e` (Option B §6 + phantom-ACK gate §8 committed).
This is the second deep-SNR-cliff disease from §3, distinct from the phantom (§8):
even a GENUINE, CRC-valid PARTIAL-bitmap SACK at a marginal rung was laundered
into "this rung is data-viable" and promoted it. §8 closed the noise-faked entry;
§9 closes the partial-batch entry. Both are instances of the §3 principle.

### §9.1 Symptom + root cause
At IONOS WGN:-10 the gearshift oscillates CONFIG_0(id 0) ↔ ROBUST_0(id 100). A
single CRC12-valid PARTIAL SACK at marginal CONFIG_0 sets `data_ack_received=YES`
(`arq_commander.cc:2818`, inside the `if(sack_detected)` block at :2702). On the
next timeout-expiry tick the data-success `else` (`:3383`, = `data_ack_received==YES`
per §6.3 brace-trace) fires ALL FOUR promotion consumers — even though the batch
was only partially delivered. The rung gets pinned as the BREAK floor / re-probed.

**The disease:** a partial SACK is a LINK-LIVENESS signal (it drives retransmit of
the missing frames) but was being read as a DATA-VIABILITY signal (the rung carries
full data). At moderate SNR partial≈viable so it worked; at the cliff a rung can
pass 1/25 frames forever and never be viable — yet promote.

### §9.2 The four promotion consumers (all fed by partial-SACK acceptance)
| # | Consumer | file:line (af14a9e) | What it does on a partial batch (the bug) |
|---|----------|---------------------|--------------------------------------------|
| 1 | Anchor-raise | `arq_commander.cc:3394-3396` | `last_data_viable_config = current_configuration` (monotonic) — pins the rung |
| 2a | Panic reset | `arq_commander.cc:3387` | `breaks_since_last_data_success = 0` — clears panic so BREAK can't latch to ROBUST_0 |
| 2b | Aggression reset | `arq_commander.cc:3386` | `break_drop_step = 2` — resets BREAK descent aggression |
| 3 | FRAME-UP | `arq_commander.cc:3454` | `consecutive_data_acks++` — climbs at `>= frame_shift_threshold` |
| 4 | LADDER-UP success | `arq_commander.cc:4495-4497` | `success_rate_data = 100*nBatches_acked/nBatches_sent`; partial bumps `nBatches_acked` (:2819-2820) → reads 100% → clears the 85% up-gate (:4611 / :4765) |

### §9.3 Principle of the fix (CLEAN-BATCH VIABILITY, and ONLY that)
A partial SACK STILL sets `data_ack_received=YES` and STILL drives retransmit —
UNCHANGED. We introduce ONE new signal "the batch just confirmed was FULLY
delivered (all-ones bitmap)" and gate the FOUR promotion consumers on it. Out of
scope (explicit): re-probe backoff / hold-down / cooldown / AARF penalty.

### §9.4 The clean signal — mechanism chosen
`bool last_batch_fully_acked` — `cl_arq_controller` member. Same lifecycle as
`data_ack_received` (the trigger the consumers gate behind):
- reset FALSE at the two batch-TX-start sites (`:1244`, `:1739`) alongside
  `data_ack_received = NO`, and in `reset_session_state` (`arq_common.cc:2949`)
  and the ctor (`arq_common.cc`).
- set TRUE at each CLEAN (all-ones) acceptance site; left FALSE at the PARTIAL site.

Why a flag (not "reuse `v2_ack_pat_pre_detected`"): `v2_ack_pat_pre_detected` is a
per-POLL flag local to `process_messages_rx_acks_data()` (reset each call); it does
NOT cover the LDPC ACK_RANGE/ACK_MULTI clean paths and is not live at the :3383
gearshift tick. A member flag with the `data_ack_received` lifecycle is live at the
gearshift tick (a later tick than acceptance) and covers every clean path.

For consumer 4 (success-rate) the brief forbids repurposing `nBatches_acked`
(audited: its SOLE reader is the success-rate calc; `stats.nBatches_acked` is
write-only/diagnostic — never read). So we ADD a parallel
`last_transmission_block_stats.nBatches_fully_acked` (and `stats.nBatches_fully_acked`
for symmetry), bumped ONLY on clean paths, and a SEPARATE rate
`success_rate_data_clean` = `nBatches_fully_acked / nBatches_sent`. The two
LADDER-UP gates read the clean rate; `success_rate_data` (on `nBatches_acked`) is
UNCHANGED and continues to drive the DOWN-shift trigger / reset + logging.
`nBatches_acked` keeps its exact meaning for any other (future) consumer. (The
first draft swapped the global numerator — reverted because it leaked into the
down-shift path; see §9.6 consumer-4 row.)

### §9.5 Producer/consumer audit (CLAUDE.md §5) — `last_batch_fully_acked` + `nBatches_fully_acked`

**(1) Producers of the clean signal (every path that sets it TRUE/FALSE):**
- FALSE (per-batch reset): `arq_commander.cc:1244` (TX start, WB ackpat batch),
  `arq_commander.cc:1739` (TX start, LDPC/NB batch), `arq_common.cc:2949`
  (reset_session_state), ctor (`arq_common.cc`, after `last_data_viable_config`).
- TRUE — CLEAN/all-ones paths ONLY:
  - **Clean ACK_PAT block** `arq_commander.cc:2935` (`data_ack_received=YES`). This
    is the funnel for: (a) MFSK all-ones suffix → `v2_ack_pat_pre_detected=true` at
    `:2521` (the `if(rx_bitmap==all_ones)` branch of the MFSK-ACK-SACK decode); and
    (b) the bare-pattern data-ACK arm at `:2919-2923` (WB now CRC-gated by §8;
    NB/suffix-incapable bare). Both reach the SAME `data_ack_received=YES` block.
  - **LDPC clean ACKs** `arq_commander.cc:3032` (ACK_RANGE) and `:3052` (ACK_MULTI).
    These are full-batch range/multi acks — clean by construction (no SACK cycle).
- FALSE explicitly — PARTIAL path: `arq_commander.cc:2818` (`if(sack_detected)`
  block; `data_ack_received=YES` but `last_batch_fully_acked=false`). `sack_detected`
  is set TRUE by BOTH partial entry points: the MFSK PARTIAL branch `:2541` (the
  `else` of `if(rx_bitmap==all_ones)` at `:2513`) and the OFDM SACK_RSP decode
  `:2672`. Setting the flag FALSE here is belt-and-suspenders (the per-batch reset
  already left it FALSE), and guards against any future path that sets it TRUE
  earlier in the same epoch.
- `nBatches_fully_acked++`: at `:2936`/`:3033`/`:3053` (clean sites, paired with the
  existing `nBatches_acked++`). NOT bumped at `:2819` (partial).

**(2) Consumers (every reader):**
- Consumer 1 (anchor-raise) `:3394-3396`: now `if(last_batch_fully_acked && index>...)`.
- Consumer 2a (panic reset) `:3387` + 2b (`break_drop_step`) `:3386`: now `if(last_batch_fully_acked){...}`.
- Consumer 3 (FRAME-UP) `:3454`: `consecutive_data_acks++` now gated by adding
  `last_batch_fully_acked` to the `if(data_ack_received==YES && ...)` at `:3448`.
- Consumer 4 (success-rate): a SEPARATE clean-only rate `success_rate_data_clean`
  (from `nBatches_fully_acked`) is computed alongside `success_rate_data` in
  `finalize_block_commander()` and read by the TWO LADDER-UP gates ONLY. CRITICAL
  scoping decision (see below): `success_rate_data` itself is LEFT on `nBatches_acked`
  — it also drives the DOWN-shift trigger / reset (`:4667`/`:4736` legacy + axis1
  twin) and all `[GEARSHIFT]` logging, where a partial SACK IS a genuine delivery
  and must keep its meaning. Swapping the global numerator (the first draft) would
  have made a healthy-but-lossy link (each SACK-recovered batch consumes 2
  `nBatches_sent` increments but 1 `nBatches_fully_acked`) read ~50% and spuriously
  DOWN-shift below the 55% threshold. The two-rate split confines §9 to the
  UP-promotion path exactly as the brief scopes it ("make the UP-promotion
  success-rate reflect CLEAN batches").
- No OTHER reader of `last_batch_fully_acked` or `nBatches_fully_acked`.

**(3) Valid states / default-init:** before the first batch, flag=FALSE (ctor +
reset_session_state). So at session start NONE of the four consumers promote on a
spurious partial — correct (nothing delivered yet). The flag is per-batch (reset at
every TX start), so it never goes stale across batches. `nBatches_fully_acked`
starts 0 (ctor + reset_telecom_system_statistics-equivalent + the per-window resets
at `:1058`/`:4551` paired with `nBatches_acked=0`).

**(4) Invariants each consumer assumes & verification on every path:**
- I1 (flag TRUE ⇒ the batch that set `data_ack_received=YES` this epoch was
  all-ones): held — only the three clean funnels set TRUE; the partial path sets
  FALSE; within one `data_ack_received` epoch exactly ONE acceptance path fires
  (once `data_ack_received==YES`, the clean `else if(data_ack_received==NO…)` at
  :2919 is gated off, and a fresh `sack_detected` only fires on a NEW SACK_RSP for a
  later batch_seq_id). Verified for: full batch (clean funnel), partial batch
  (sack_detected, FALSE), BREAK (returns before :3383; never sets the flag),
  config switch / session reset (reset_session_state → FALSE), OFDM SACK_RSP path
  (:2672→:2818 FALSE), MFSK-ACK-SACK path (clean :2521→:2935 TRUE; partial
  :2541→:2818 FALSE), LDPC fallback (:3032/:3052 TRUE).
- I2 (consumer 4's rate ≤ clean delivery rate): held — `nBatches_fully_acked` is
  bumped only at the three clean sites, never at the partial site; `nBatches_sent`
  unchanged. A partial-only run reads 0% (not 100%) → up-gate never clears.
- I3 (a partial batch never raises `last_data_viable_config`): held — consumer 1 now
  ANDs `last_batch_fully_acked`; the partial path leaves it FALSE.

**(5) What the fix changes per consumer:** see §9.2 → §9.6. Each consumer gains a
`last_batch_fully_acked` predicate (1,2,3) or swaps to the clean counter (4). No
consumer's *downstream* assumption changes: anchor still monotonic-up & bounded by
delivered rungs; panic counter still resets on REAL data; FRAME-UP still climbs on
consecutive successes; success-rate still gates at 85%. Interaction with Option B
(§7): the anchor `last_data_viable_config` is now raised ONLY on clean batches —
strictly tighter than before, so every Option-B consumer (BREAK floor, up-shifter
gates) sees an anchor that is ≤ the old value → can only be MORE conservative,
never less. No Option-B invariant violated. Interaction with §8: orthogonal — §8
gates the noise-faked phantom out of `data_ack_received=YES`; §9 gates the genuine
partial out of PROMOTION while keeping it in liveness/retransmit.

### §9.6 As-built diff (file:line, af14a9e → working tree)
| # | Site | Change |
|---|------|--------|
| decl | `arq.h` (~1334, after `last_data_viable_config`) | `+ bool last_batch_fully_acked;` |
| decl | `arq.h` struct st_telecom_system_statistics (~268) | `+ int nBatches_fully_acked;` |
| decl | `arq.h` (test method, after `test_phantom_ack_gate`) | `+ int test_clean_batch_viability();` |
| init | `arq_common.cc` ctor (~349) | `+ last_batch_fully_acked=false;` |
| init | `arq_common.cc` reset_session_state (~2949) | `+ last_batch_fully_acked = false;` (by `data_ack_received=NO`) |
| init | `arq_common.cc` stats reset (~107/127) | `+ {stats,last_transmission_block_stats}.nBatches_fully_acked=0;` |
| reset | `arq_commander.cc:1244` | `+ last_batch_fully_acked = false;` (by `data_ack_received = NO`) |
| reset | `arq_commander.cc:1739` | `+ last_batch_fully_acked = false;` |
| reset | `arq_commander.cc:1058` / `:4551` | `+ ...nBatches_fully_acked = 0;` (paired with nBatches_acked=0) |
| P (clean) | `arq_commander.cc:2935` | `+ last_batch_fully_acked = true;` |
| P (clean) | `arq_commander.cc:2936` | `+ ...nBatches_fully_acked++;` (both stats + block) |
| P (clean) | `arq_commander.cc:3032`/`:3052` | `+ last_batch_fully_acked=true; +nBatches_fully_acked++;` |
| P (partial) | `arq_commander.cc:2818` | `+ last_batch_fully_acked = false;` (explicit, belt+suspenders) |
| C1 | `arq_commander.cc:3394` | `if(last_batch_fully_acked && index(cur)>index(anchor)) anchor=cur;` |
| C2 | `arq_commander.cc:3385-3387` | gate `break_drop_step=2` + `breaks_since_last_data_success=0` on `if(last_batch_fully_acked)` (keep `emergency_nack_count=0` UNGATED — see §9.7) |
| C3 | `arq_commander.cc:3448` | add `&& last_batch_fully_acked` to the FRAME-UP `if` |
| C4 | `arq_commander.cc:4553-4560` | ADD `success_rate_data_clean` (from `nBatches_fully_acked`) alongside `success_rate_data` (kept on `nBatches_acked`); UP gates `:4684`/`:4841` read the clean rate; DOWN-shift/logging keep reading `success_rate_data` |
| C4 | `arq.h` (~1337, after `gear_shift_down_consecutive_fails`) | `+ double success_rate_data_clean;` (ctor `arq_common.cc:333`, reset `:2953` both `=100.0`) |
| T | `arq_commander.cc` (after test_phantom_ack_gate) | `test_clean_batch_viability()` |
| T | `main.cc` (~334 decl, ~748 parse, ~1719 invoke) | `--test-clean-batch-viability` |

### §9.7 Design decision: `emergency_nack_count=0` (`:3385`) stays UNGATED
The data-success `else` resets THREE things at :3385-3387. `breaks_since_last_data_success`
(2a) and `break_drop_step` (2b) are PANIC/aggression state — gated clean-only so a
partial run can't clear panic (the whole point: let CONFIG_0 latch panic and reach
ROBUST_0). `emergency_nack_count` (the per-rung BREAK trigger counter, threshold 3
at :3334) is LEFT ungated: a partial SACK means the receiver DID get some frames, so
the rung is not in the "3 consecutive total block failures" state that
`emergency_nack_count` tracks. Gating it clean-only would let a string of partials
accumulate `emergency_nack_count` toward a BREAK that the channel doesn't warrant
(partials are progress). Panic (`breaks_since_last_data_success`) is the counter that
must latch through partials; `emergency_nack_count` is not. This keeps BREAK
behavior on the partial path identical to pre-fix (no behavior change for the
liveness role), satisfying the "keep BREAK-poll behavior unchanged" constraint.

### §9.8 Regression test (`--test-clean-batch-viability`)
`cl_arq_controller::test_clean_batch_viability()` (after `test_phantom_ack_gate`),
wired in main.cc. Drives the REAL gearshift consumers by priming state and
replaying the exact :3383-block + :4495 logic the way the existing
`test_data_anchored_promote` drives the real `policy_evaluate_axis1`. Assertions:
- **P0 (pure predicate):** `promotion_allowed_on_batch(false)==false`,
  `promotion_allowed_on_batch(true)==true`.
- **A (partial batch does NOT promote):** prime anchor=ROBUST_0, cur=CONFIG_0,
  panic counter=1, break_drop_step=8, consecutive_data_acks=1; run the gated
  consumer logic with `last_batch_fully_acked=false`: A1 `last_data_viable_config`
  NOT raised (stays ROBUST_0); A2 `breaks_since_last_data_success` NOT reset (stays 1);
  A3 `break_drop_step` NOT reset (stays 8); A4 `consecutive_data_acks` NOT advanced
  (stays 1); A5 `success_rate_data_clean` reads 0% (UP gate not cleared); A6
  `success_rate_data` (down-shift/log) UNCHANGED at 100% — proves §9 didn't perturb
  the down-shift path.
- **B (clean batch DOES promote):** same priming, `last_batch_fully_acked=true`:
  B1 anchor raised to CONFIG_0; B2 panic reset to 0; B3 `break_drop_step` reset to 2;
  B4 `consecutive_data_acks` advanced to 2; B5 `success_rate_data_clean` reads 100%.
- **C (panic can still latch after partial-only run):** with anchor at ROBUST_0 and
  partials only (panic never reset, so it COULD reach >=2), drive
  `breaks_since_last_data_success=2` and assert `break_target_with_anchor(raw=ROBUST_0)`
  reaches ROBUST_0 (panic bypasses the anchor floor — BREAK can reach the floor).

**FAIL-before / PASS-after:** the test routes every consumer (incl. the consumer-4
clean-rate numerator) through the PURE helper `promotion_allowed_on_batch(
last_batch_fully_acked)` (≡ `return last_batch_fully_acked;`); forcing it to
`return true` (pre-fix behavior, partial==clean) makes A1/A2/A3/A4/A5 FAIL (partial
promotes). With the gate, ALL PASS. A6 PASSES in both (success_rate_data unchanged).

## §10 CLIMB-LATENCY — guarded SNR-seed of the START + ROBUST-tier pipeline (2026-06-27)

**Status:** implemented on `staging/climb-latency`. Companion to
[[gearshift-climb-engine.md]] §13/§14 (elevator) and
[[data-flow-inband-tier-crossing.md]] §2/§3 (hybrid cross routing).

### §10.1 Symptom (groundtruth, trustworthy fixed-scorer commit 7adbb4e)
Trio-ON (`MERCURY_INBAND_RATE`/`CUMULATIVE_ACK`/`INBAND_A3_DECOUPLE`), clean
channel: the RSP DOES cross to CONFIG_0 (raw-log confirmed) but only at ~150 s ->
~5 B/min. Breakdown: connect 43 s + **ROBUST_0 68 s (DOMINANT)** + ROBUST_1 ~17 s
+ ROBUST_2+cross ~17 s. The ROBUST crawl (~102 s) is wasted on a clean channel
that could have opened near the SNR-appropriate WB config.

Two root causes:
1. **No SNR-seed at start.** The SUCCESS_BASED_LADDER no-BW-upgrade branch
   (`arq_commander.cc:6889-6895`, the `connection_status=TRANSMITTING_DATA`
   fall-through) deliberately starts data at `init_configuration` (ROBUST_0 for
   `-R`) and crawls rung-by-rung (the §6 Option-B anti-blind-climb fix). On a
   *clean* channel this throws away ~85 s before even reaching OFDM.
2. **First ROBUST rung un-pipelined.** `inband_pipeline_climb_active()`
   (`arq_common.cc:3398`) requires `inband_retag_armed`, which is only set AFTER a
   climb's SET_CONFIG/unilateral fires. So the ROBUST_0 dwell falls under the
   strict clean-batch gate (`effective_frame_shift_threshold`), un-pipelined; the
   pipeline only kicks in from the 2nd rung.

### §10.2 Fix (1) — guarded SNR-seed of the start target
New helper `connect_seed_target()` (`arq_commander.cc`), called ONCE in the
SUCCESS_BASED_LADDER no-BW branch right before the `TRANSMITTING_DATA`
fall-through. It returns a start config strictly ABOVE `init_configuration` ONLY
when the connect SNR clearly licenses it; otherwise CONFIG_NONE (no seed -> start
ROBUST_0 exactly as today). When it returns a real target the branch routes
through the EXISTING SET_CONFIG path (the proven fast tier-cross with the
dedicated ACK + the §3 reverse-robust pin) instead of starting raw.

**Over-seed guard (the load-bearing part).** The connect SNR
(`measurements.SNR_uplink`) at this point is the **control-plane MFSK suffix SNR**
(`snr_uplink_from_suffix`, arq_common.cc:11795 / responder 2925) — the SAME value
the §15 deep-SNR over-climb guard distrusts because the MFSK suffix decodes at
~1 dB even when OFDM data cannot. So the seed must be conservative:
- requires `inband_rate_feature_enabled() && gear_shift_on==YES` and a valid SNR
  (`SNR_uplink > -90`);
- maps through `get_configuration(SNR_uplink - CONNECT_SEED_MARGIN_DB)` with a
  LARGE margin (`CONNECT_SEED_MARGIN_DB`, common_defines.h) that subsumes the
  control-vs-data SNR gap — so a marginal channel maps back to ROBUST_0;
- caps the seed at a conservative ceiling `CONNECT_SEED_CONFIG_CAP` (mid-ladder,
  NOT the top) so even a clearly-clean channel opens at a SAFE WB rung the +1
  ladder/elevator then climbs the rest of the way;
- applies the SAME NB / `supershift_proven_ceiling` / bigblock-cooldown caps the
  elevator uses (shared `apply_bigblock_cooldown_cap` chokepoint);
- returns CONFIG_NONE if the capped result is NOT strictly above
  `init_configuration` (degenerate -> no seed).

It does NOT reuse `elevator_target_from_snr()` directly: that helper's
`supershift_retrigger_target` anchor-clamp would force the result to
`anchor_cap = ROBUST_1` at connect (the anchor is ROBUST_0 by design), neutering
any WB seed. The over-seed guard here is the conservative margin + ceiling, NOT
the anchor clamp.

This does NOT touch the §6 Option-B no-blind-climb invariant for WEAK channels:
on anything that is not clearly clean, `connect_seed_target()` returns CONFIG_NONE
and the start is ROBUST_0 byte-identical. It does NOT change the mid-climb +1
elevator pin (arq_commander.cc:6043 / `inband_climb_target`).

### §10.3 Fix (2) — pipeline the FIRST ROBUST climb rung
`inband_pipeline_climb_active()` is extended: when the feature is on, the live
config is a ROBUST config, and the gearshift is climbing (gear_shift_on, OFDM
ceiling not yet reached), the FIRST ROBUST rung is also eligible for the
optimistic FRAME-UP advance (advance the climb streak on a forward-healthy data
ACK without waiting for the strict clean fully-acked confirm). This collapses the
ROBUST_0->1->2 serialization. Safety: the SAME overshoot net
`inband_retag_escalate_if_climb_exhausted()` (arq_common.cc:3629) recovers a too-
eager climb to `inband_last_confirmed_config` (or, at session start where that is
CONFIG_NONE, `config_ladder_down(pre_announce)` — never craters) — NEVER a BREAK,
never below the floor. This is the SAME safety envelope §1.8's OFDM pipeline
relies on. It does NOT confirm a climb (the RETIRED §6 base-pattern false-confirm,
data-flow-inband-basepattern-confirm-falseconfirm.md, is untouched) — it only
relaxes the *advance* gate; the CONFIRM still rides the config-discriminating
suffix bsi (`inband_retag_confirm_from_sack`).

### §10.4 §5 cross-layer audit (connect + gearshift state)
Shared state touched: `current_configuration` / `forward_configuration` /
`negotiated_configuration` / `reverse_configuration` (the config owners) and the
`connection_status`/`link_status` start transition.

**Producers of the start config:**
- ctor / `init()` seat `current_configuration = init_configuration`
  (arq_common.cc init path); CONNECT skips `reset_session_state` (§17).
- The SET_CONFIG builder (arq_commander.cc:1145-1211) — the §3.1 chokepoint ALL
  gearshift/optimizer/demote producers funnel through. The seed REUSES this exact
  builder via `add_message_control(SET_CONFIG)` (no new transport).
- `inband_unilateral_config_change` (arq_common.cc:3174) for intra-tier tags.

**Consumers:** TX encode (`process_messages_tx_data` reads
`current_configuration`); the RSP follow (`detect_and_follow_config_tag` /
SET_CONFIG handler arq_responder.cc:680); the FRAME-UP gate (reads the live
config); the BREAK/anchor machinery (`last_data_viable_config`,
`break_target_with_anchor`).

**Invariant checks:**
1. The seed is a robust->OFDM TIER CROSS (ROBUST_0 -> an OFDM rung).
   `inband_config_change_is_tier_crossing` (arq_common.cc:3253) returns TRUE, so
   the SET_CONFIG builder routes it via the LEGACY handshake (the fast dedicated
   ACK) AND applies the §3 reverse-robust pin (`inband_tier_cross_reverse_config`)
   — reverse stays on the live ROBUST rung so the reverse SACK decodes on MFSK
   across the cross. This is the proven cross path; the seed adds no new routing.
2. The anchor `last_data_viable_config` is NOT raised by the seed (it is a
   speculative start, like the elevator). It follows CONFIRMED clean delivery only
   (§1.1). If the seeded WB rung cannot carry data, the existing BREAK path demotes
   via `break_target_with_anchor` to the ROBUST floor (panic bypass reaches
   ROBUST_0). So a mistaken seed is RECOVERABLE — same envelope as the elevator.
3. The seed runs ONLY on the no-BW-upgrade fall-through (after the
   `we_want_wb && currently_nb` SWITCH_BANDWIDTH branch). If a WB upgrade is
   pending the seed is skipped (the SWITCH_BANDWIDTH must complete first); after
   the upgrade the steady-state climb seeds via the normal ladder.
4. Fix (2)'s optimistic advance is bounded by the +1 elevator pin
   (`inband_climb_target` returns proposed_frame under inband) and the D4 net — it
   cannot over-promote past one rung per FRAME-UP, and an unfollowable rung
   auto-demotes.

### §10.5 FAIL-before / PASS-after tests
- `--test-connect-snr-seed` (`arq_commander.cc`): drives the pure
  `connect_seed_target_core()` selector across SNR cells.
  - CLEAN (high control SNR, get_configuration(SNR-margin) lands well above
    ROBUST_0): seed returns a WB OFDM config strictly above ROBUST_0 but
    <= `CONNECT_SEED_CONFIG_CAP`. FAIL-BEFORE (`CONNECT_SEED_FAILBEFORE`): the seed
    is forced to CONFIG_NONE -> start stays ROBUST_0 -> assert FAILS (proves the
    seed is load-bearing).
  - MARGINAL / deep-SNR cliff (SNR_uplink low, control suffix ~1 dB):
    `get_configuration(SNR - CONNECT_SEED_MARGIN_DB)` maps back to ROBUST_0 ->
    `connect_seed_target_core()` returns CONFIG_NONE (no over-seed). PASSES in both
    arms (the over-seed guard holds).
- `--test-robust-pipeline`: assert `inband_pipeline_climb_active()` returns TRUE on
  a ROBUST config mid-climb under the feature, FALSE off-feature / at the OFDM
  ceiling. FAIL-BEFORE (`INBAND_ROBUST_PIPELINE_FAILBEFORE`): pinned false ->
  assert FAILS.

**Over-seed guard present:** the conservative `CONNECT_SEED_MARGIN_DB` +
`CONNECT_SEED_CONFIG_CAP` + the strictly-above-ROBUST_0 gate; verified by the
MARGINAL test cell returning CONFIG_NONE.

# Data-Flow Audit — In-Band Hybrid Tier-Crossing Routing

Branch: `feat/inband-a3-decouple` (off `feat/inband-rate-adapt`).
Scope: the `MERCURY_INBAND_RATE=1` (opt-in) config-change transport. The default-off
legacy path is byte-identical — the entire change is entry-gated on
`inband_rate_feature_enabled()` (the chokepoint guard at
`arq_commander.cc:834`) and the new predicate is only reached from inside that gate.

This document owns the routing decision for an in-band config change: which transport
(legacy SET_CONFIG control handshake vs the in-band unilateral CONFIG_TAG) a given
target uses, and the shared state that must survive the choice.

---

## §0. The problem (VERIFIED off-bench, prior agent)

Under `MERCURY_INBAND_RATE=1`, the redesign announces a rate change with a passband
CONFIG_TAG on the next batch and follows it unilaterally — NO SET_CONFIG control
handshake. The in-band tag DOES cross robust→OFDM (the FRAME-UP fires, the RX follows,
`[INBAND-TX] CONFIRMED followed`), but the unilateral tag is STRUCTURALLY SLOWER for a
TIER CROSSING: each rung's confirm serializes behind the slow data-SACK turnaround
(`receiving_timeout=12412ms`, ~12.4 s/rung). The robust→OFDM cross therefore slips
~3 s PAST a fixed test budget — a coin flip. Measured: the redesign capped at
ROBUST_2/53 B while legacy reached CONFIG_4/101 B in the same budget. Legacy's
SET_CONFIG control handshake has a FAST DEDICATED ACK (decoupled from the data-SACK),
crossing ~3 s earlier.

Conclusion: the in-band tag is the wrong tool for the TIER crossing; it is the right
tool for intra-tier rate adapts (where the rung-by-rung confirm is the design).

---

## §1. The fix (HYBRID)

At the in-band config-change chokepoint `arq_commander.cc:834` (the §3.1 chokepoint
every gearshift/optimizer/demote/BREAK producer funnels through), route by tier:

- **Tier crossing** (`is_robust_config(current_configuration) != is_robust_config(target)`,
  EITHER direction): FALL THROUGH to the legacy SET_CONFIG control-handshake builder
  (the proven, fast-dedicated-ACK path). The unilateral tag is NOT taken; no tag armed.
- **Intra-tier** (both robust, or both OFDM 0–16): KEEP the in-band CONFIG_TAG
  (`inband_unilateral_config_change`) — it works there, and the D1/D4/down-ladder/A3
  machinery is all intra-tier.
- **CONFIG_NONE / no-op target**: not a crossing; falls through to the legacy builder
  (matches the flag-off degenerate path, byte-identical).

Predicate factored into `cl_arq_controller::inband_config_change_is_tier_crossing(int)`
(`arq_common.cc`, after `inband_unilateral_config_change`) so the chokepoint AND the
directed regression drive the EXACT same decision.

Files:
- `arq_commander.cc:834` — chokepoint: `bool tier_crossing =
  inband_config_change_is_tier_crossing(inband_target); if(tier_crossing) {...fall
  through...} else if(inband_unilateral_config_change(...)) {...tag...}` (the `else if`
  guarantees mutual exclusion = no double-emit).
- `arq_common.cc` — `inband_config_change_is_tier_crossing` (pure predicate; the
  `INBAND_TIER_CROSSING_FAILBEFORE` macro pins it to "never a crossing" for the
  fails-before arm).
- `include/datalink_layer/arq.h` — declarations.
- `arq_commander.cc::test_inband_tier_crossing_routing` + `main.cc` wiring
  (`--test` and `--test-inband-tier-crossing`).

---

## §2. Cross-layer audit (CLAUDE.md §5)

Shared state crossed by the routing choice: `cmd_batch_seq_id` epoch, the retx queue,
the down-ladder, `rsp_current_expected_batch_seq_id`, and the in-band re-tag state
(`inband_retag_armed`, `inband_last_announced_config`, …).

1. **Producers of the routing choice**: only the chokepoint at `arq_commander.cc:834`
   (inside `add_message_control(SET_CONFIG)`), reached only when
   `inband_rate_feature_enabled()`.

2. **Consumers / downstream of a CROSSING (now legacy SET_CONFIG)**:
   - The legacy builder (`arq_commander.cc:852+`) emits SET_CONFIG with
     `forward_configuration = negotiated_configuration` (SUCCESS_BASED_LADDER, the
     redesign's algorithm) — IDENTICAL to `inband_target`, so no member needs
     adjusting at the chokepoint.
   - The ACK-apply (`arq_commander.cc:6286-6298`) calls
     `load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES)` — the SAME
     backup-preserving PHY-only load the unilateral path uses
     (`arq_common.cc:3123`). Message buffers are NOT torn down; the
     **`cmd_batch_seq_id` epoch and the retx queue survive** (PHYSICAL_LAYER_ONLY).

3. **Valid states / no stale tag state at a crossing**:
   - The CONFIG_TAG only emits on OFDM DATA batches (`emit_config_tag_passband`
     no-ops when `ack_mfsk.ack_sack_suffix_len() <= 0`, i.e. the robust/MFSK tier).
     A robust→OFDM CLIMB therefore has NO armed re-tag before the cross (nothing was
     emittable at the robust tier) → routing it via SET_CONFIG arms nothing stale.
   - An OFDM→robust DEMOTE: pre-fix the unilateral path armed a re-tag for a robust
     target that could NEVER emit (M=MFSK after the load) — a latent dead-arm. The
     fix routes it via SET_CONFIG instead, so NO stale robust re-tag is armed = an
     improvement, not a regression.
   - After a SET_CONFIG cross INTO OFDM, `inband_last_announced_config` is still
     CONFIG_NONE / the old config, so the FIRST intra-OFDM DATA batch is a `is_change`
     (`arq_common.cc:2739`) → the tag fires and announces the new config to the RX.
     **Intra-tier tagging resumes cleanly.**

4. **Intra-tier INTACT**: an intra-OFDM or intra-robust change has
   `is_robust_config(current)==is_robust_config(target)` → `tier_crossing==false` →
   the `else if(inband_unilateral_config_change(...))` path is taken EXACTLY as before.
   The down-ladder (`inband_route_failure_demote`), D1 repeat
   (`inband_retag_confirm_from_sack`), D4 escalation
   (`inband_retag_escalate_if_climb_exhausted`), and A3 are all intra-tier and
   UNAFFECTED. Verified by the test's intra-tier assertions (both arms expect the tag).

5. **No double-emit**: `tier_crossing` short-circuits BEFORE
   `inband_unilateral_config_change` via the `else if`; exactly one of {SET_CONFIG
   builder, unilateral tag} runs, never both.

---

## §3. The cross-oscillation root + the reverse-ACK PIN (VERIFIED off-bench)

After §1 routed the cross via legacy SET_CONFIG, the redesign STILL oscillated
ROBUST_2 (102) ↔ CONFIG_0, never sustaining an OFDM rung, capped at CONFIG_0 with
35 emergency BREAKs (`_pipe_ab/redesign360.arqlog`, WGN:40, `MERCURY_INBAND_RATE=1`).
Legacy sustains the climb past CONFIG_0 on the SAME channel.

### §3.1 Root (two-part, both VERIFIED from paired logs)

**PART A — reverse-ACK pinned to the OFDM forward rung at the cross.** The redesign's
intra-ROBUST climb (100→101→102) rides the in-band CONFIG_TAG (unilateral, NO
SET_CONFIG), so `reverse_configuration` is NEVER seeded to a robust rung — it sits at
its ctor sentinel `CONFIG_NONE` (`arq_common.cc:759`/`:6245`). At the robust→OFDM cross
the §1 hybrid routes through the legacy SET_CONFIG builder, whose SUCCESS_BASED_LADDER
fallthrough `if(reverse_configuration == CONFIG_NONE) reverse_configuration =
forward_configuration` (`arq_commander.cc:~935`) sets reverse to the FORWARD OFDM rung
(0). The reverse SACK suffix must then decode on OFDM-0 in a tight turnaround and TIMES
OUT every cross:
`[GEARSHIFT] SET_CONFIG: forward=0 reverse=0` → `[CMD-ACK-SNR] ACK detected, suffix
timeout` → data-block-failure BREAK.
LEGACY NEVER hits this: its SET_CONFIG climb seeds reverse to a ROBUST rung at the first
step and HOLDS it for the whole climb — `forward=0 reverse=101` at the cross
(`_a3proof/legacy_ftrt.modem.log`), so its reverse SACK rides a rock-solid MFSK rung.

**PART B — the connect-LIVENESS GUARD false-fires a true-loss BREAK during the cross
handshake (the dominant root; CORRECTED from the initial anchor-clamp hypothesis).**
After PART A alone the reverse SACK rides ROBUST_2 (`reverse=102`) yet the oscillation
PERSISTED: 8/8 remaining BREAKs were the connect-liveness guard
(`inband_connect_liveness_guard`, `arq_commander.cc:3253`) firing
`[INBAND-LIVENESS] no forward-DATA progress for 200 control-plane polls ... connect/
negotiate LIVELOCK; firing the retained true-loss BREAK` (MEASURED, `_xfix/on_partA.arqlog`).
MECHANISM: data flowed during the intra-ROBUST climb
(`cmd_inband_liveness_last_acked=3`), so the connect/negotiate exemption
(`arq_commander.cc:3324-3332`, which only covers the PRE-DATA handshake) no longer
applies. The robust→OFDM cross then sits in `RECEIVING_ACKS_CONTROL` (conn=6) with
`nAcked_data` FLAT for the whole cross (the SET_CONFIG ACK + the ~12.4s data-SACK
turnaround at the robust rung), so the guard accrues 200 polls and false-fires. That
BREAK then anchor-clamps to the robust anchor (`break_target_with_anchor`,
`arq_commander.cc:185-191`, `config 102 -> 102`) → `UNILATERAL CONFIG 100 -> 102` →
ROBUST_2 → re-cross → oscillate. This is the SAME guard-masquerading-as-architectural
pattern as the WB-negotiate false-fire (§5 of data-flow-inband-connect-liveness.md) and
the idle-switchrole race — a LEGITIMATE slow control handshake whose state signature is
identical to a livelock. The anchor-clamp is the proximate snap-back, but the BREAK
should never have fired: the deliberate upward tier-cross is not a livelock.

### §3.2 The fix

**PART A (the root-cause fix):** on an in-band tier-cross, PIN `reverse_configuration`
to the ROBUST side of the boundary (mirroring legacy's reverse-robust hold) instead of
letting it fall to the OFDM forward rung. Robust side = the LIVE config on a
robust→OFDM up-cross (it just carried data reliably) or the TARGET on an OFDM→robust
down-cross. Selector factored into the pure
`cl_arq_controller::inband_tier_cross_reverse_config(from, to, inband_on)`
(`arq_common.cc`, after `inband_config_change_is_tier_crossing`) so the production site
AND the directed regression drive the EXACT same decision. Applied at the
SUCCESS_BASED_LADDER reverse-seed (`arq_commander.cc:~930`), gated on
`inband_rate_feature_enabled()` AND a real crossing → flag-off / intra-tier / SNR_BASED
are byte-identical. With reverse on a robust MFSK rung the reverse SACK decodes across
the cross → the cross is ACKed → no data-block-failure → no BREAK → no oscillation.

**PART B (REQUIRED — the dominant root):** EXEMPT the in-band tier-crossing control
handshake from the connect-liveness guard's stall accrual — the SAME exemption the
WB-negotiate gets (§5 of data-flow-inband-connect-liveness.md), extended to cover the
post-data tier-cross. While in a control phase (`TRANSMITTING_CONTROL` /
`RECEIVING_ACKS_CONTROL`) AND the in-flight config change
(`negotiated_configuration`) crosses the tier vs the live config, hold the streak at 0.
Factored into the pure
`cl_arq_controller::inband_tiercross_handshake_exempts_liveness(conn_status, target)`
(`arq_common.cc`) so the guard site AND the directed regression drive the EXACT same
decision. Applied at `arq_commander.cc:~3334` (right after the connect/negotiate
exemption), under `#ifndef INBAND_TIERCROSS_LIVENESS_FAILBEFORE`. Self-limiting: once the
cross ACKs (current==target, no longer a crossing) the exemption disengages and the
guard resumes. BOUNDED + genuine-death net intact: a truly stuck cross is owned by the
control-ACK-miss / FRAME-UP-FAILED tag-demote machinery (`arq_commander.cc:2897`/`:2984`),
NOT this guard; and a POST-cross DATA stall (current==target, not a crossing) arms the
guard normally. The anchor-clamp (PART B-proximate) is LEFT UNCHANGED — with the BREAK no
longer false-firing, the clamp is no longer reached on the cross, and it remains correct
for a GENUINE deep-SNR demote (the panic-jump bypass `:187` + anchor-DEMOTE escape
`:5076-5111` are untouched).

### §3.3 Files (this fix)
- PART A — `arq_common.cc` `inband_tier_cross_reverse_config` (pure selector;
  `INBAND_REVERSE_PIN_FAILBEFORE` reverts it to the no-pin fails-before);
  `arq_commander.cc:~930` — the reverse-seed pin at the SET_CONFIG builder.
- PART B — `arq_common.cc` `inband_tiercross_handshake_exempts_liveness` (pure predicate);
  `arq_commander.cc:~3334` — the exemption in `inband_connect_liveness_guard`, under
  `#ifndef INBAND_TIERCROSS_LIVENESS_FAILBEFORE` (the E2E fails-before macro).
- `include/datalink_layer/arq.h` — declarations.
- `arq_commander.cc::test_inband_tier_cross_reverse_pin` (covers BOTH parts) + `main.cc`
  wiring (`--test` and `--test-inband-reverse-pin`).

---

## §3-tests'. Tests for §3 (reverse-pin + liveness exemption)

- **Directed unit** `test_inband_tier_cross_reverse_pin` (`--test` +
  `--test-inband-reverse-pin`): drives BOTH pure selectors across the matrix.
  - PART A crossings (ROBUST_2→CONFIG_0, ROBUST_0→CONFIG_16, CONFIG_4→ROBUST_0) assert
    reverse pinned to the robust rung; intra-tier / CONFIG_NONE / feature-OFF assert no
    pin (CONFIG_NONE). FAILS-BEFORE (`-DINBAND_REVERSE_PIN_FAILBEFORE`): the 3 crossing
    asserts FLIP to FAIL (got=CONFIG_NONE, exit 1) — MEASURED. PASSES-AFTER: all PASS
    (exit 0) — MEASURED.
  - PART B liveness exemption: cross-in-flight @ control phase asserts EXEMPT; intra-OFDM
    control + DATA-phase assert NOT exempt. PASSES-AFTER all PASS — MEASURED.
- **E2E (realtime-sim)**: WGN:40, `MERCURY_INBAND_RATE=1`, start ROBUST_0, 360s,
  `--turnaround-drift`, `tools/sim/sim_arq_channel.py` (see `_xfix/`).
  - BEFORE (redesign360, pre-fix): 35 BREAKs, capped at CONFIG_0, oscillates 102↔0,
    rx≈311 B, `repro_deep_stall=true`.
  - PART A only: 8 BREAKs (ALL 8 = liveness false-fire), still oscillates, rx≈354 B —
    proves PART A necessary-but-insufficient and isolates PART B's root.
  - PART A+B (passes-after, MEASURED `_xfix/on_partAB`): the tier-cross oscillation is
    GONE — **1** cross 102→0 (vs 7 before), **0** cross-BREAKs, **0** liveness false-fires
    on the cross. It crosses to CONFIG_0 (peak_config=CONFIG_0) and HOLDS it; the later
    0→102 demote is a LEGITIMATE CONFIG_0 data-decode degradation routed via the NO-BREAK
    tag-demote (`[INBAND-NOBREAK] frame_gearshift_data_failed_pat`), not the oscillation.
    rx 311→397 B. Diagnosed bug (7-cross 35-BREAK 102↔0 limit cycle) RESOLVED.
  - Fails-before macro for PART B: `-DINBAND_TIERCROSS_LIVENESS_FAILBEFORE` compiles out
    the exemption → reproduces the oscillation.

## §3-tests. Tests (fails-before / passes-after) — §1 routing

- **Routing (unit)** `test_inband_tier_crossing_routing` (`--test` +
  `--test-inband-tier-crossing`): drives the production predicate across the matrix.
  CROSSINGS (ROBUST_0→CONFIG_0, CONFIG_4→ROBUST_0, ROBUST_0→CONFIG_16) assert
  `routes_to_set_config == true` unconditionally; intra-tier (CONFIG_0→CONFIG_4,
  CONFIG_16→CONFIG_4, ROBUST_0→ROBUST_1) and CONFIG_NONE assert `false` (both arms).
  - FAILS-BEFORE (`-DINBAND_TIER_CROSSING_FAILBEFORE`): predicate pinned to false →
    the 3 crossing assertions FLIP to FAIL (exit 1). MEASURED.
  - PASSES-AFTER (clean): all PASS (exit 0). MEASURED. Runs inside the full `--test`
    suite (exit 0). MEASURED.

- **E2E (realtime-sim)**: paired ON-hybrid (`mercury.exe`) vs ON-failbefore
  (`mercury_fb.exe`, `-DINBAND_TIER_CROSSING_FAILBEFORE`) at WGN:40 from ROBUST_0,
  `MERCURY_INBAND_RATE=1`, `tools/sim/sim_arq_channel.py`. The hybrid arm CROSSES
  robust→OFDM (reaches a CONFIG_n, delivers) where the fail-before arm is capped at
  the robust tier. See `_hybrid_e2e/`. [result recorded in the branch commit message]

---

## §5'. Cross-layer audit — `reverse_configuration` (the PART A pin)

`reverse_configuration` is shared state (the responder→commander config for the reverse
link / the post-SWITCH_ROLE return path). The PART A pin changes WHAT value it holds
across a tier-cross. Audit per CLAUDE.md §5:

1. **Producers** (writers of `reverse_configuration`):
   - `arq_commander.cc:892` (SNR_BASED SET_CONFIG) — `get_configuration(SNR_uplink)`.
     UNTOUCHED (the pin is in the SUCCESS_BASED_LADDER branch only).
   - `arq_commander.cc:~899` (SUCCESS_BASED_LADDER seed) — the pin site. PRE-pin:
     `if(==CONFIG_NONE) = forward_configuration`. POST-pin: on an in-band crossing,
     `= the robust side`; otherwise the legacy seed runs UNCHANGED.
   - BREAK recovery `:310-311` / `:404-405` — `if(==CONFIG_NONE) = target`. UNTOUCHED.
   - Turbo `:5628`, `:6632` — `= start_config` / `current_configuration`. UNTOUCHED
     (turbo is a separate climb engine; not reached on the in-band FRAME-UP cross).
   - SWITCH_ROLE swap `:6210-6214` (CMD) / `arq_responder.cc:1907-1909` (RSP) —
     swaps forward↔reverse. Reads the value; see consumers.
   - Ctor / session reset `arq_common.cc:759`/`:6245` — `CONFIG_NONE` (the init the
     redesign was stuck at, the root of PART A).
   - RSP adopt `arq_responder.cc:3220` — `= messages_control.data[2]` (the wire field
     the pin populates). This is the cross-link delivery of the pin to the peer.

2. **Consumers** (readers):
   - `messages_control.data[2]` at `arq_commander.cc:905` — the wire field. The pin
     makes it a ROBUST rung on a cross; the RSP adopts it at `:3220`.
   - SWITCH_ROLE swap (above): on a role swap the RSP's NEW forward = old reverse, i.e.
     the RSP would TX its return-path data on the pinned rung. On a cross that is a
     ROBUST rung (slow but reliable) — **this is exactly legacy's behavior** (legacy
     held reverse=101 through the whole climb, so legacy's RSP also returns on robust
     until the reverse direction climbs on its OWN SET_CONFIG). NOT a regression; the
     reverse direction promotes independently later via its own intra-tier adapt.
   - `cl_arq_controller::config_state_string` (`arq_common.cc:1923`) — diagnostic read.

3. **Valid states / invariant**: before any producer writes, `reverse_configuration ==
   CONFIG_NONE`. The pin only acts on a real in-band crossing; the `==CONFIG_NONE`
   legacy fallback still runs AFTER the pin (so a non-crossing CONFIG_NONE still seeds
   to forward, unchanged). The pin value is always a valid robust config (100–102), so
   `data[2]` and the SWITCH_ROLE swap always see a loadable PHY config.

4. **What the fix changes**: on an in-band tier-cross, `data[2]` carries a ROBUST rung
   instead of the OFDM forward rung. Every consumer above either (a) is unaffected
   (diagnostics, the SNR_BASED/turbo/BREAK producers), or (b) gets the legacy-equivalent
   robust value (the wire field + the SWITCH_ROLE return path). No consumer assumes
   reverse==forward on a cross — legacy proves the opposite is the correct, working
   state.

## §5''. Cross-layer audit — `inband_connect_liveness_guard` / emergency-BREAK (PART B)

PART B changes WHEN the liveness guard accrues a stall. Audit per CLAUDE.md §5:

1. **Producer of the guard decision**: only `inband_connect_liveness_guard()`
   (`arq_commander.cc:3253`), called once per poll from `process_messages_commander`
   (`:256`), and ONLY when `inband_rate_feature_enabled()` (the function early-returns
   false OFF -> byte-identical legacy).
2. **What the exemption gates**: the stall ACCRUAL (`cmd_inband_liveness_no_progress_polls++`)
   and thus the BREAK fire. The exemption holds the streak at 0 while a tier-cross control
   handshake is in flight. It does NOT touch `nAcked_data`, `cmd_batch_seq_id`, the retx
   queue, `messages_tx[]`, or `current_configuration` — it only suppresses a false stall
   count. So no batch/epoch/config shared state is perturbed.
3. **Genuine-death net intact** (the over-broad edge):
   - A truly stuck cross (RSP never ACKs the SET_CONFIG): the SET_CONFIG control message
     exhausts its resends and the control-ACK-miss / FRAME-UP-FAILED handlers
     (`arq_commander.cc:2897`/`:2984`) route it to the in-band tag-demote — NOT the
     liveness guard. So a dead cross still recovers; the guard exempting it just removes
     the DUPLICATE false BREAK that was racing the demote.
   - A POST-cross DATA stall (the cross landed, current==target, NO longer a crossing):
     `inband_config_change_is_tier_crossing` returns false → the exemption is OFF → the
     guard arms exactly as before. The data-phase forward-healthy-miss demote
     (`:3384`) and the `INBAND_LIVENESS_MAX_BREAKS` hard-reset backstop are unchanged.
4. **Anchor-clamp UNCHANGED** (PART B-proximate): `break_target_with_anchor`
   (`:185-191`) and the BREAK recovery SM are byte-identical. With the BREAK no longer
   false-firing on the cross, the clamp is simply not reached there; for a GENUINE
   deep-SNR demote it still clamps correctly (the panic-jump bypass `:187` + anchor-DEMOTE
   escape `:5076-5111` are untouched). The deliberate upward cross is fixed by NOT firing
   the BREAK, not by changing the clamp.
5. **Legacy / intra-tier unaffected**: OFF the guard early-returns; an intra-tier control
   op has `is_tier_crossing==false` → exemption OFF → the guard works as before. Verified
   by the directed test's intra-OFDM-control and DATA-phase assertions.
6. **SIBLING REGRESSION CAUGHT + CLOSED (CLAUDE.md §5 in action)**: the FIRST PART B cut
   (exempt on `control phase && is_tier_crossing(negotiated)` ALONE) silently BROKE the
   existing `test_inband_liveness` F3/F4 cases (`arq_responder.cc:6090-6111`) — a GENUINE
   post-data control-plane livelock staged at the bottom rung (ROBUST_0, in-flight
   PENDING_ACK batch) carries a stale CROSSING `negotiated_configuration`, so the bare
   exemption swallowed it (the backstop never fired: got fired_at=-1 want 5). The
   `--test` suite caught it (exit 1, all visible PASS but `[TEST-INBAND-LIVENESS] FAIL`
   F3/F4) — baseline efc2260 exits 0, my pre-fix binary exit 1. FIX: add the THIRD
   conjunct `!cmd_has_inflight_data_batch()` — the real FRAME-UP cross is a PURE control
   op (data was pushed to FIFO at `arq_commander.cc:5515-5520`, no inflight batch) whereas
   the genuine post-data livelock carries an inflight batch. The exemption now stands down
   on F3/F4 (genuine backstop fires) and still fires on the real cross. Directed test
   gains the `crossing target BUT inflight DATA → NOT exempt` assertion; `--test` exits 0,
   F3/F4 green.

## §4. Open questions

- [?] The full climb from ROBUST_0 to the OFDM boundary is slow (~100 s to the first
  robust promote at the hailing rate). That is a SEPARATE throughput question (does
  the redesign BEAT legacy end-to-end?) tracked under the A/B campaign; this fix only
  makes the crossing itself fast once the climb reaches the boundary.

- [?] **SURFACED SIBLING (out of scope for this fix) — the `ROBUST_DWELL_BATCH_OP`
  liveness false-fire.** After PART A+B fixed the tier-cross oscillation, the PART A+B
  e2e (`_xfix/on_partAB.arqlog`) shows **5** residual `[INBAND-LIVENESS] ... LIVELOCK`
  BREAKs, ALL on CONFIG_102 (ROBUST_2), EACH preceded by a `[CMD-ROBUST-DWELL]
  ROBUST_DWELL_BATCH_OP TX` (`arq_commander.cc:1065`/`:5607`). This is the SAME
  guard-masquerading-as-architectural pattern (a legitimate in-band control handshake —
  the robust batch-size dwell op — sits in `RECEIVING_ACKS_CONTROL` with `nAcked_data`
  flat, identical signature to a livelock) but on a DIFFERENT control op, NOT a
  tier-cross, so PART B's tier-cross exemption correctly does not cover it. Each is in
  budget (#1/3) and the link recovers + keeps delivering, so it is not fatal, but it adds
  churn and keeps the BREAK count above legacy's ~0. The PRINCIPLED fix is to exempt ANY
  legitimate in-band control op in flight from the liveness stall (the guard's purpose is
  a DATA-phase livelock, not a control handshake) — but that BROADENS the liveness guard's
  exemption to shared state and warrants its own §5 audit + owner ratification, so it is
  deliberately LEFT for a follow-up rather than silently widened here.

---

## §6. THE KEYSTONE — generalize the decoupled-confirm to INTRA-tier promotes

Status: PLAN (2026-06-23, `feat/inband-a3-decouple`, HEAD §19/§20). Owner-approved
**Design B** (keep the in-band CONFIG_TAG for the ANNOUNCE; decouple ONLY the CONFIRM
onto the robust base-pattern reverse ACK, NOT a new control op — lean reuse).

### §6.0 Problem (VERIFIED, static reads + the §19 deadbatch A/B baseline)

The §1 hybrid only decoupled the robust↔OFDM **tier-BOUNDARY** confirm (onto the legacy
SET_CONFIG fast control-ACK). Every **intra-tier rung promote** (robust 100→101→102, the
CONFIG_0 adoption, intra-OFDM 0→4→…) still confirms via `inband_retag_confirm_from_sack`
(`arq_common.cc:3325`), which is called from EXACTLY THREE sites, ALL inside data-SACK
decode handlers that require a CRC-valid bsi-bearing MFSK-ACK-SACK suffix:
`arq_commander.cc:3758` (MFSK-ACK-SACK clean), `:3783` (partial), `:3953` (OFDM SACK_RSP).
So the intra-tier confirm rides the **bsi-bearing reverse-MFSK-data-SACK suffix** — the
GF(16)+CRC-12 coded block that decodes only at higher SNR than the base ACK pattern
(keystone reliability map wf_97f50b94: peak_matched 3–5/7 sub-threshold on this channel →
suffix CRC fails → no confirm → climb stalls). The §1.8 pipeline
(`inband_pipeline_climb_active`, `arq_common.cc:3277`) only relaxes the FRAME-UP *advance*
gate; it does NOT decouple the *confirm*. So the climb caps at the robust tier / CONFIG_0.

§19 deadbatch A/B (`_msab/deadbatch/aggregate.json`, WGN SNR3k=40, N=3, this HEAD):
REDESIGN rx median **0** B / max 113 B, final_config ROBUST_2|null (capped, never climbs);
LEGACY rx median **5651** B / max **7475** B, final_config CONFIG_5|CONFIG_6 (climbs into
OFDM). The gap is the data-COUPLED intra-tier confirm.

### §6.1 The fix (Design B — lean reuse of the base ACK pattern)

The reverse MFSK ACK the RX already sends per data batch has TWO parts: a robust BASE
tone-pattern (correlation `mfsk_matched`, threshold `ack_match_threshold=7/16`,
P(false)=2.4e-5/poll — `mfsk.cc:257/266`) and an APPENDED bsi+bitmap SACK suffix
(GF(16)+CRC-12). The CMD decodes both in ONE call `decode_ack_sack_from_passband`
(`arq_commander.cc:3606-3608`), which returns `mfsk_matched` (the base correlation)
**independently of whether the suffix CRC passed**. Today the confirm fires only on the
CRC-valid suffix path (`decoded==true`). When the suffix CRC fails (`:3659-3669` →
`decoded=false`) the climb confirm is LOST even though `mfsk_matched>=threshold` PROVES
the RX transmitted a reverse ACK *after* demodulating the forward batch at the announced
config.

**THE FIX:** add a decoupled confirm consumer — when a CLIMB re-tag is armed
(`inband_retag_armed` && climb-up by ladder index) AND the base ACK pattern matched
(`mfsk_matched >= ack_match_threshold`), CONFIRM the climb (disarm the re-tag) **even
when the bsi-bearing suffix CRC failed**. The tag still ANNOUNCES (unchanged); only the
CONFIRM moves off the suffix-decode onto the base pattern. This is DSP-justified (the base
pattern is more robust than the suffix) and is a pure REUSE: no new RX TX (the RX already
sends the ACK), no new control op, no new wire frame, no new session-wide state — just one
new confirm helper + one hook at the CRC-fail branch.

Because the base pattern carries NO bsi, the confirm cannot use the at-or-after-anchor bsi
test (`inband_retag_confirm_from_sack`'s safety). Substitute the equivalent ANNOUNCE-EMITTED
guard: only confirm via the base pattern once the climb's tag has actually been EMITTED at
least once (`inband_announce_bsi >= 0` — the FIRST emit fills it, `arq_common.cc:2769`), so
a base ACK for a pre-announce in-flight batch cannot false-confirm. New helper
`inband_retag_confirm_from_base_pattern(int mfsk_matched, int ack_match_threshold)`
(`arq_common.cc`, beside `inband_retag_confirm_from_sack`), gated on
`inband_rate_feature_enabled()` && `inband_retag_armed` && climb-up &&
`inband_announce_bsi >= 0` && `mfsk_matched >= ack_match_threshold`. Macro
`INBAND_BASEPATTERN_CONFIRM_FAILBEFORE` pins it to `return false` (the data-coupled
fails-before) for the directed test.

Hook: `arq_commander.cc:~3669` (the CRC12-fail branch, after `decoded=false`), passing
`mfsk_matched` and `telecom_system->ack_mfsk.ack_match_threshold`. Strictly additive —
the existing CRC-valid suffix confirm at `:3758/:3783/:3953` is UNTOUCHED (it still fires
first whenever the suffix decodes; the base-pattern path only adds a confirm when the
suffix FAILED).

### §6.2 Files
- `arq_common.cc` — `inband_retag_confirm_from_base_pattern` (new helper; FAILBEFORE macro).
- `arq_commander.cc:~3669` — the hook at the CRC12-fail branch.
- `include/datalink_layer/arq.h` — declaration.
- `arq_commander.cc::test_inband_basepattern_confirm` (new directed test) + `main.cc`
  wiring (`--test` + `--test-inband-basepattern-confirm`).

### §6.3 §5 cross-layer audit — `inband_retag_armed` / climb-confirm transport

Shared session-wide state changed: the SET of signals that DISARM `inband_retag_armed`
(the promote-confirm). Adds the base-pattern path; removes nothing.

1. **Producers (writers) of the confirm/disarm:**
   - `inband_retag_confirm_from_sack` (`arq_common.cc:3344-3347`) — the existing CRC-valid
     suffix confirm. UNTOUCHED.
   - `inband_retag_escalate_if_climb_exhausted` (`:3393-3396`) — the D4 auto-demote disarm
     (R floor reached, no confirm). UNTOUCHED.
   - `inband_unilateral_config_change` (`:3105`) — ARMS a fresh re-tag (a NEW climb/drop).
   - Ctor/session reset (`:772`, `:6477`). UNTOUCHED.
   - **NEW:** `inband_retag_confirm_from_base_pattern` — disarms on a base-pattern match
     for an EMITTED climb whose suffix CRC failed.
2. **Consumers (readers) of `inband_retag_armed`:** the emit gate (re-emit the tag while
   armed), `inband_pipeline_climb_active` (`:3289`), `inband_retag_escalate_if_climb_exhausted`
   (`:3365`), the down-ladder. All read "armed?"; the new producer only flips armed→false
   on a PROVEN follow, identical post-state to the existing confirm (sets
   `inband_last_confirmed_config`, clears `inband_retag_config/announce_bsi/count`).
3. **Valid states / false-confirm guard:** before the FIRST emit `inband_announce_bsi==-1`
   → the new helper returns false (cannot confirm a not-yet-announced climb). After emit,
   `mfsk_matched>=7/16` proves a reverse ACK followed a forward batch at the announced
   config. A base ACK for a STALE/pre-announce batch: the climb re-anchors `announce_bsi`
   to the newest rung on each emit (`:3104-3108`/`:2769`), and a base match only proves
   "an ACK came back," so the residual risk is confirming the climb one batch early — but
   the climb is OPTIMISTIC by design (§1.8) and a too-early confirm only DISARMS the
   re-emit; if the RX did NOT actually follow, the next batch's FRAME-UP stalls at the
   unfollowable rung and D4 auto-demotes to `inband_last_confirmed_config` (the existing
   net, `:3361`). So an over-eager base confirm is RECOVERABLE, never a BREAK, never below
   the floor — same safety envelope §1.8 relies on.
4. **(a) Tier-BOUNDARY confirm intact:** the boundary still routes via SET_CONFIG
   (`arq_commander.cc:858`, §1) — it never reaches the intra-tier in-band confirm path, so
   the base-pattern consumer cannot perturb it. The boundary's fast control-ACK is a
   SEPARATE transport (`messages_control.status==ACKED`). UNCHANGED.
5. **(b) Reverse-pin holds during an intra-OFDM climb:** an intra-OFDM promote is NOT a
   tier crossing → `inband_tier_cross_reverse_config` returns CONFIG_NONE (`:3199-3200`) →
   `reverse_configuration` is untouched by the pin and holds whatever the last cross/legacy
   seed set (a robust rung after the robust→OFDM cross). The base-pattern confirm does NOT
   write `reverse_configuration`. So the reverse stays robust while the forward climbs —
   verified-by-construction (the pin is crossing-only; the new path touches neither).
6. **(c) A REAL failed promote still demotes:** if the RX genuinely cannot follow the
   climbed-to rung, NO reverse ACK comes back (`mfsk_matched < threshold`) → the new helper
   returns false → no false confirm → the re-tag stays armed → D4 escalation
   (`inband_retag_escalate_if_climb_exhausted`) reaches R and AUTO-DEMOTES to
   last-confirmed (§4.1). The §20 CMD-side genuine block-failure demote (reverse-ACK decode
   failure 13s apart) is on a DIFFERENT producer (the data-block-failure path) and is
   UNTOUCHED. The down-ladder is intra-tier and UNTOUCHED.
7. **(d) Legacy byte-identical:** the helper early-returns false when
   `!inband_rate_feature_enabled()`; the hook is reached only after the existing inband
   gates. Flag-off, the CRC12-fail branch runs verbatim (no confirm). The
   `INBAND_BASEPATTERN_CONFIRM_FAILBEFORE` macro reproduces the data-coupled pre-fix.

### §6.4 Tests
- **Directed unit** `test_inband_basepattern_confirm` (`--test` +
  `--test-inband-basepattern-confirm`): drives `inband_retag_confirm_from_base_pattern`
  across the matrix — (armed climb, emitted, mfsk_matched>=7) → CONFIRM; (mfsk_matched<7)
  → no confirm; (not emitted, announce_bsi<0) → no confirm; (armed DROP not climb) → no
  confirm; (feature OFF) → no confirm. FAILS-BEFORE
  (`-DINBAND_BASEPATTERN_CONFIRM_FAILBEFORE`): the CONFIRM assertion FLIPS to FAIL.
- **LIVE-PATH (realtime-sim) fail-before/pass-after:** WGN SNR3k=40 from start-cfg 100,
  `MERCURY_INBAND_RATE=1`, `tools/sim/sim_arq_channel.py`. Fail-before
  (`mercury_fb.exe`, `-DINBAND_BASEPATTERN_CONFIRM_FAILBEFORE`): the intra-tier confirm
  rides the suffix and STALLS (caps at robust/CONFIG_0). Pass-after (`mercury.exe`): the
  confirm rides the base pattern → the climb proceeds past CONFIG_0 toward the OFDM rungs.
- **A/B:** N=5 both arms, `_msab/keystone`, vs LEGACY's CONFIG_6/7475B (§19 baseline) /
  the 7647B target — does REDESIGN now climb past CONFIG_0?

### §6.5 Scope (NOT this change — the B/C/D follow-ons)
This keystone is (A) of a 3–4 fix path. EXPECTED outcome: the climb UNBLOCKS but throughput
may still trail legacy if forward frames drop (M1 SACK over-fire / SKIP-VAR poison; M2 ~73%
coarse-acq miss). Those are B (coalesce the redesign SACK cadence + suppress
prev-delivered-during-fresh-batch) and C (seat the CONFIG_0 forward capture ring at natural
`load_configuration` geometry on in-band adoption), sequenced AFTER measuring how far (A)
gets. One-change-one-test (§3) — B/C are NOT folded in here.

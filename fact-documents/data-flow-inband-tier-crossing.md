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

## §3. Tests (fails-before / passes-after)

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

## §4. Open questions

- [?] The full climb from ROBUST_0 to the OFDM boundary is slow (~100 s to the first
  robust promote at the hailing rate). That is a SEPARATE throughput question (does
  the redesign BEAT legacy end-to-end?) tracked under the A/B campaign; this fix only
  makes the crossing itself fast once the climb reaches the boundary.

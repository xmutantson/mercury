#!/usr/bin/env python3
"""Mechanical ownership audit for Gearshift-v2 ACTIVE.

ACTIVE may receive telemetry/failure requests from legacy ARQ machinery, but the
three Axis-1/recovery execution chokepoints must remain behind the v2 owner:
SET_CONFIG, CONFIG_TAG/unilateral changes, and BREAK.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
CMD = (ROOT / "source/datalink_layer/arq_commander.cc").read_text()
COMMON = (ROOT / "source/datalink_layer/arq_common.cc").read_text()
OPT_H = (ROOT / "include/datalink_layer/rate_optimizer.h").read_text()
OPT_CC = (ROOT / "source/datalink_layer/rate_optimizer.cc").read_text()

failed = []

def req(name, cond):
    print(f"[GS2-OWNER-AUDIT] {'PASS' if cond else 'FAIL'} {name}")
    if not cond:
        failed.append(name)

def between(text, start, end):
    a = text.find(start)
    b = text.find(end, a + len(start)) if a >= 0 else -1
    return text[a:b if b >= 0 else None] if a >= 0 else ""

demote = between(
    CMD,
    "bool cl_arq_controller::inband_route_failure_demote",
    "bool cl_arq_controller::inband_cmd_dead_batch_floor_reached",
)
setcfg = between(
    CMD,
    "else if(code==SET_CONFIG)",
    "else if(code==KEY_EXCHANGE_1)",
)
live = between(
    CMD,
    "bool cl_arq_controller::inband_connect_liveness_guard()",
    "// ============================================================================",
)
unilateral = between(
    COMMON,
    "bool cl_arq_controller::inband_unilateral_config_change",
    "bool cl_arq_controller::inband_config_change_is_tier_crossing",
)
brk = between(
    COMMON,
    "void cl_arq_controller::send_break_pattern()",
    "void cl_arq_controller::",
)

req("owner API declared",
    all(x in OPT_H for x in (
        "owns_link_experiment(unsigned long long now_ms = 0) const",
        "transition_matches(int from_cfg, int to_cfg) const",
        "authorize_external_transition(",
        "authorize_hard_recovery(",
    )))
req("owner API implemented",
    all(x in OPT_CC for x in (
        "cl_rate_optimizer::owns_link_experiment(unsigned long long now_ms) const",
        "cl_rate_optimizer::authorize_external_transition(",
        "cl_rate_optimizer::authorize_hard_recovery(",
    )))

a = demote.find("authorize_external_transition(")
m = demote.find("restage_requeue_tx_messages()")
req("failure demote asks owner before mutating payload/config state",
    a >= 0 and m >= 0 and a < m)
req("foreign demote target is telemetry only; full v2 evaluator selects destination",
    "opt_evaluate_batch_end(&owner_target)" in demote
    and "owner selected %d->%d" in demote)

a = setcfg.find("authorize_external_transition(")
t = setcfg.find("inband_unilateral_config_change(")
req("SET_CONFIG asks owner before selecting CONFIG_TAG transport",
    a >= 0 and t >= 0 and a < t)

req("generic liveness yields for full v2 experiment lifetime",
    "rate_opt.owns_link_experiment(opt_now_ms())" in live)
req("generic liveness is telemetry-only and full v2 evaluator selects destination",
    "opt_record_batch(" in live
    and "opt_evaluate_batch_end(&owner_target)" in live
    and "generic BREAK suppressed" in live
    and 'inband_route_failure_demote(lower, "liveness_stall")' not in live)

req("direct CONFIG_TAG execution requires matching owner",
    "rate_opt.transition_matches(current_configuration, target_cfg)" in unilateral)
req("foreign transition API refuses idle legacy-selected targets",
    "telemetry-only; Gearshift must select destination" in OPT_CC)
req("floor BREAK requires Gearshift no-admissible-candidate verdict",
    'last_v2_decision.reason == "no-admissible-candidate"' in OPT_CC)
req("owned CONFIG_TAG fallback is explicitly marked",
    'climb_unfollowable_autodemote", true, true' in COMMON
    and 'nack_accelerated_demote", true, true' in COMMON)

a = brk.find("rate_opt.owns_link_experiment(opt_now_ms())")
h = brk.find("rate_opt.authorize_hard_recovery(")
p = brk.find("ptt_on();")
req("BREAK cannot preempt owned switch/probe", a >= 0
    and "rate_opt.owns_link_experiment(opt_now_ms())" in brk)
req("BREAK hard recovery is owner-authorized before RF emission",
    h >= 0 and p >= 0 and h < p)
req("above-floor BREAK is converted to owner-mediated demote",
    'inband_route_failure_demote(lower, "legacy_break_request")' in brk)

req("liveness hard reset is owner-gated",
    '"liveness-floor-recovery"' in CMD)
req("pure-silence reconnect is owner-gated under ACTIVE",
    "pure-silence reconnect request" in CMD
    and 'inband_route_failure_demote(hint, "demote_silence_peer_unreachable")' in CMD)
req("ACTIVE BREAK recovery settles at floor and returns authority to v2",
    "gearshift_v2_finish_break_at_floor()" in CMD
    and "legacy BREAK ladder disabled, v2 will reacquire upward" in CMD)

req("legacy turboshift is retired on ACTIVE CONNECTED handoff",
    "retiring legacy TURBOSHIFT controller" in CMD
    and "rate_opt.controls_link() && link_status == CONNECTED && turboshift_active" in CMD)
req("legacy ladder is structurally subordinate in ACTIVE",
    "if (rate_opt.get_mode() == GEARSHIFT_V2_ACTIVE)" in (ROOT / "include/datalink_layer/arq.h").read_text()
    and "return true;  // v2 owns every ordinary data-rate transition" in (ROOT / "include/datalink_layer/arq.h").read_text())
req("legacy topgear selector cannot execute under ACTIVE",
    "!rate_opt.controls_link()" in CMD and "topgear_elect_feature_enabled()" in CMD)
req("legacy demote requests are telemetry-only outside owned fallback",
    "telemetry failure reason=" in demote
    and "opt_evaluate_batch_end(&owner_target)" in demote)

req("liveness hard reset consumes owner authorization before DROPPED",
    '"liveness-floor-recovery"' in live
    and "authorize_hard_recovery" in live
    and live.find("authorize_hard_recovery") < live.find("link_status = DROPPED"))
req("pure-silence reconnect is not an independent ACTIVE recovery authority",
    '"demote_silence_peer_unreachable"' in CMD
    and "pure-silence reconnect request" in CMD
    and "rate_opt.authorize_hard_recovery" in CMD)

if failed:
    print(f"[GS2-OWNER-AUDIT] FAILURES={len(failed)}")
    sys.exit(1)
print("[GS2-OWNER-AUDIT] ALL PASS")

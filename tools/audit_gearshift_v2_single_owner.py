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
        "owns_link_experiment() const",
        "transition_matches(int from_cfg, int to_cfg) const",
        "authorize_external_transition(",
        "authorize_hard_recovery(",
    )))
req("owner API implemented",
    all(x in OPT_CC for x in (
        "cl_rate_optimizer::owns_link_experiment() const",
        "cl_rate_optimizer::authorize_external_transition(",
        "cl_rate_optimizer::authorize_hard_recovery(",
    )))

a = demote.find("authorize_external_transition(")
m = demote.find("restage_requeue_tx_messages()")
req("failure demote asks owner before mutating payload/config state",
    a >= 0 and m >= 0 and a < m)

a = setcfg.find("authorize_external_transition(")
t = setcfg.find("inband_unilateral_config_change(")
req("SET_CONFIG asks owner before selecting CONFIG_TAG transport",
    a >= 0 and t >= 0 and a < t)

req("generic liveness yields for full v2 experiment lifetime",
    "rate_opt.owns_link_experiment()" in live)
req("generic liveness reports a downshift request through owner",
    'inband_route_failure_demote(lower, "liveness_stall")' in live)

req("direct CONFIG_TAG execution requires matching owner",
    "rate_opt.transition_matches(current_configuration, target_cfg)" in unilateral)

a = brk.find("rate_opt.owns_link_experiment()")
h = brk.find("rate_opt.authorize_hard_recovery(")
p = brk.find("ptt_on();")
req("BREAK cannot preempt owned switch/probe", a >= 0)
req("BREAK hard recovery is owner-authorized before RF emission",
    h >= 0 and p >= 0 and h < p)
req("above-floor BREAK is converted to owner-mediated demote",
    'inband_route_failure_demote(lower, "legacy_break_request")' in brk)
req("ACTIVE BREAK recovery settles at floor and returns authority to v2",
    "gearshift_v2_finish_break_at_floor()" in CMD
    and "legacy BREAK ladder disabled, v2 will reacquire upward" in CMD)

if failed:
    print(f"[GS2-OWNER-AUDIT] FAILURES={len(failed)}")
    sys.exit(1)
print("[GS2-OWNER-AUDIT] ALL PASS")

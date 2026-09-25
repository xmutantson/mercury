#!/usr/bin/env python3
"""Fail-closed responder EOT observation for production real-audio results."""

import copy
import re


EOT_OK_MARKER = b"[RSP-V2-EOT-OK]"
EOT_OK_LINE = re.compile(
    rb"^(?:(?:\[T\+[0-9]+(?:\.[0-9]+)?\] )?\[RSP\] )?"
    rb"\[RSP-V2-EOT-OK\] end-of-transfer verified: "
    rb"delivered==committed=[0-9]+(?=\s|$)"
)
TRANSFER_SCOPE = re.compile(
    rb"(?<![A-Za-z0-9_])(?:transfer|tag)\s*=\s*"
    rb"(?:\"([^\"]*)\"|'([^']*)'|([^\s,\]\)}]+))"
)
MISSING_EOT_VERIFICATION = "missing_eot_verification"
MALFORMED_WHOLE_SESSION_SCHEMA = "malformed_whole_session_schema"
CANONICAL_WHOLE_SESSION_FIELDS = (
    "whole_session_status",
    "whole_session_scorable",
    "whole_session_credited_bytes",
)


def _scope_value(value):
    if isinstance(value, bytes):
        return value
    if isinstance(value, str):
        return value.encode("utf-8")
    return None


def _line_scope_values(line):
    return [next(group for group in match.groups() if group is not None)
            for match in TRANSFER_SCOPE.finditer(line)]


def responder_eot_ok_count(log_path, scored_transfer=None):
    """Return in-scope responder terminal-proof emissions in an ARQ log."""
    count = 0
    scored_scope = _scope_value(scored_transfer)
    with open(log_path, "rb") as stream:
        for line in stream:
            if EOT_OK_LINE.match(line) is None:
                continue
            line_scopes = _line_scope_values(line)
            if line_scopes and (scored_scope is None
                                or any(value != scored_scope
                                       for value in line_scopes)):
                continue
            count += 1
    return count


def _primary_whole_session(result):
    windows = result.get("windows")
    return (windows.get("primary_whole_session")
            if isinstance(windows, dict) else None)


def _positive_credit(value):
    return (isinstance(value, (int, float))
            and not isinstance(value, bool)
            and value > 0)


def objective_completion_or_credit(result):
    """Derive completion from every representation accepted by this scorer.

    This union controls only failure typing.  EOT creditability itself does not
    depend on this detection: without exactly one raw marker, the normalizer
    unconditionally clears every score and scorable flag first.
    """
    primary = _primary_whole_session(result)
    return bool(
        result.get("whole_session_status") == "COMPLETE"
        or result.get("status") == "COMPLETE"
        or (isinstance(primary, dict)
            and primary.get("status") == "COMPLETE")
        or result.get("whole_session_claimed_complete")
        or (isinstance(primary, dict) and primary.get("completed"))
        or _positive_credit(result.get("whole_session_credited_bytes"))
        or _positive_credit(result.get("credited_bytes"))
        or _positive_credit(result.get("scored_good_prefix_bytes"))
        or (isinstance(primary, dict)
            and _positive_credit(primary.get("credited_bytes")))
    )


def missing_canonical_whole_session_fields(result):
    """Return absent production-schema fields; values are checked elsewhere."""
    return [name for name in CANONICAL_WHOLE_SESSION_FIELDS
            if name not in result]


def _append_instrument_reason(result, reason):
    reasons = result.get("instrument_invalid_reasons")
    if not isinstance(reasons, list):
        reasons = []
    if reason not in reasons:
        reasons.append(reason)
    result["instrument_invalid_reasons"] = reasons


def _invalidate_primary(primary, reason):
    if not isinstance(primary, dict):
        return
    primary["byte_completed"] = bool(primary.get("completed"))
    primary["completed"] = False
    primary["status"] = "VOID"
    primary["void_reason"] = reason
    primary["terminal_eot_verified"] = False
    primary["scorable"] = False
    primary["credited_bytes"] = 0
    primary["rate_Bps"] = None
    primary["content_Bmin"] = None


def normalize_terminal_eot_result(result):
    """Enforce the production schema and the marker/credit invariant.

    The caller must first attach ``terminal_eot_ok_count`` from the scoped raw
    log.  Cohort reduction calls this again, making its admission boundary
    independently fail closed for malformed or forged result JSON.
    """
    gated = copy.deepcopy(result)
    objective_completion = objective_completion_or_credit(gated)
    missing_fields = missing_canonical_whole_session_fields(gated)
    prior_missing = gated.get("whole_session_schema_missing_fields")
    if isinstance(prior_missing, list):
        missing_fields = list(dict.fromkeys(prior_missing + missing_fields))
    schema_valid = not missing_fields and gated.get(
        "whole_session_schema_valid") is not False
    marker_count = gated.get("terminal_eot_ok_count")
    terminal_eot_verified = bool(
        type(marker_count) is int
        and marker_count == 1
        and gated.get("terminal_eot_verified") is True)

    gated["terminal_eot_verified"] = terminal_eot_verified
    gated["whole_session_claimed_complete"] = bool(objective_completion)
    gated["whole_session_schema_valid"] = schema_valid
    gated["whole_session_schema_missing_fields"] = missing_fields

    if not schema_valid:
        gated["instrument_invalid"] = True
        _append_instrument_reason(gated, MALFORMED_WHOLE_SESSION_SCHEMA)

    # Fail closed by construction: marker proof is tested before and
    # independently of any completion/status representation.
    if not terminal_eot_verified:
        gated["whole_session_scorable"] = False
        gated["whole_session_credited_bytes"] = 0
        if "credited_bytes" in gated:
            gated["credited_bytes"] = 0
        gated["scored_good_prefix_bytes"] = 0
        primary = _primary_whole_session(gated)
        if isinstance(primary, dict):
            primary["terminal_eot_verified"] = False
            primary["scorable"] = False
            primary["credited_bytes"] = 0

    void_reason = None
    if objective_completion and not terminal_eot_verified:
        void_reason = MISSING_EOT_VERIFICATION
    elif not schema_valid:
        void_reason = MALFORMED_WHOLE_SESSION_SCHEMA

    if void_reason is None:
        return gated

    gated["whole_session_status"] = "VOID"
    gated["whole_session_void_reason"] = void_reason
    gated["void_reason"] = void_reason
    gated["whole_session_scorable"] = False
    gated["whole_session_credited_bytes"] = 0
    gated["whole_session_rate_Bps"] = None
    gated["whole_session_content_Bmin"] = None
    gated["scored_good_prefix_bytes"] = 0
    if "status" in gated:
        gated["status"] = "VOID"
    if "credited_bytes" in gated:
        gated["credited_bytes"] = 0
    gated["delivered_full_byte_only"] = bool(gated.get("delivered_full"))
    gated["delivered_full"] = False
    gated["delivered_Bmin"] = None
    gated["delivered_user_Bmin"] = None
    gated["user_content_Bmin"] = None
    gated["wall_user_rate_Bps"] = None
    gated["scored_wall_user_rate_Bps"] = None
    gated["vs_vara"] = None
    gated["rx_bps_wall"] = None
    gated["verdict"] = "VOID"

    _invalidate_primary(_primary_whole_session(gated), void_reason)

    anatomy = gated.get("anatomy")
    if isinstance(anatomy, dict):
        anatomy["delivered_user_Bmin"] = None
        anatomy["user_content_Bmin"] = None
        anatomy["wire_Bmin"] = None
    return gated


def apply_terminal_eot_gate(result, log_path):
    """Attach raw-log EOT evidence and enforce fail-closed creditability.

    Byte delivery and endpoint release remain useful diagnostics, but neither is
    responder-owned terminal proof.  Only ``[RSP-V2-EOT-OK]`` authorizes a
    COMPLETE/scorable/credited whole-session result.
    """
    gated = copy.deepcopy(result)
    marker_count = responder_eot_ok_count(log_path, gated.get("tag"))
    gated["terminal_eot_ok_count"] = marker_count
    gated["terminal_eot_verified"] = marker_count == 1
    return normalize_terminal_eot_result(gated)

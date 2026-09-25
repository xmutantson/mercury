#!/usr/bin/env python3
"""Fail-closed reader for the real-audio bridge channel coordinate."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path


SNR_ASSERTION_TOLERANCE_DB = 0.05
FADED_PROFILES = frozenset({"MPG", "MPM", "MPP"})


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _same_cell(actual: object, expected: object) -> bool:
    """Compare channel labels without rejecting harmless numeric spelling."""
    if not isinstance(actual, str) or not isinstance(expected, str):
        return False
    actual_kind, separator_a, actual_value = actual.strip().partition(":")
    expected_kind, separator_e, expected_value = expected.strip().partition(":")
    if actual_kind.upper() != expected_kind.upper() or separator_a != separator_e:
        return False
    if not separator_a:
        return True
    try:
        return math.isclose(float(actual_value), float(expected_value),
                            rel_tol=0.0, abs_tol=1.0e-9)
    except ValueError:
        return actual_value.strip().upper() == expected_value.strip().upper()


def attested_snr3k(
        stats_path: str | os.PathLike[str], *, expected_cell: str,
        expected_profile: str, expected_seed: int, expected_passthrough: bool,
        requested_snr3k: float | None = None,
        tolerance_db: float = SNR_ASSERTION_TOLERANCE_DB) -> tuple[float | None, dict]:
    """Return only a bridge-attested, VARA-scorable external SNR coordinate.

    ``requested_snr3k`` is an assertion, never a coordinate producer.  Missing,
    malformed, identity-mismatched, or assertion-mismatched bridge evidence
    returns ``None`` so callers cannot emit a VARA comparison accidentally.
    The audit object retains the realized bridge coordinate for diagnosis.
    """
    path = Path(stats_path)
    audit = {
        "source": "bridge.channel_attestation",
        "stats_path": str(path),
        "valid": False,
        "instrument_invalid": True,
        "vara_scorable": False,
        "vara_scoring_disabled_reason": None,
        "requires_explicit_calibrated_snr": False,
        "reasons": [],
        "realized_snr3k_db": None,
        "requested_snr3k_db": requested_snr3k,
        "requested_delta_db": None,
        "tolerance_db": float(tolerance_db),
        "channel_attestation": None,
    }

    if not path.is_file():
        audit["reasons"].append("stats_missing")
        return None, audit
    try:
        stats = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        audit["reasons"].append("stats_unreadable")
        return None, audit
    att = stats.get("channel_attestation") if isinstance(stats, dict) else None
    if not isinstance(att, dict):
        audit["reasons"].append("attestation_missing")
        return None, audit
    audit["channel_attestation"] = att

    commanded = _number(att.get("commanded_snr"))
    offset = _number(att.get("realized_snr_offset_db"))
    p_sig = _number(att.get("realized_p_sig"))
    seed = att.get("seed")
    passthrough = att.get("passthrough")
    profile = att.get("profile")
    cell = att.get("cell")

    if commanded is None:
        audit["reasons"].append("commanded_snr_invalid")
    if offset is None:
        audit["reasons"].append("snr_offset_invalid")
    if p_sig is None or p_sig <= 0.0:
        audit["reasons"].append("signal_power_invalid")
    if not isinstance(seed, int) or isinstance(seed, bool):
        audit["reasons"].append("seed_invalid")
    elif seed != int(expected_seed):
        audit["reasons"].append("seed_mismatch")
    if not isinstance(passthrough, bool):
        audit["reasons"].append("passthrough_invalid")
    elif passthrough != bool(expected_passthrough):
        audit["reasons"].append("passthrough_mismatch")
    if not isinstance(profile, str):
        audit["reasons"].append("profile_invalid")
    elif profile.strip().upper() != str(expected_profile).strip().upper():
        audit["reasons"].append("profile_mismatch")
    if not _same_cell(cell, expected_cell):
        audit["reasons"].append("cell_mismatch")

    if commanded is not None and offset is not None:
        realized = commanded + offset
        explicit_realized = _number(att.get("realized_snr3k"))
        if explicit_realized is not None:
            if not math.isclose(realized, explicit_realized, rel_tol=0.0,
                                abs_tol=1.0e-9):
                audit["reasons"].append("realized_coordinate_mismatch")
            realized = explicit_realized
        audit["realized_snr3k_db"] = realized
        if requested_snr3k is not None:
            requested = _number(requested_snr3k)
            if requested is None:
                audit["reasons"].append("requested_snr3k_invalid")
            else:
                delta = requested - realized
                audit["requested_delta_db"] = delta
                if abs(delta) > tolerance_db:
                    audit["reasons"].append("requested_snr3k_mismatch")

    audit["valid"] = not audit["reasons"]
    audit["instrument_invalid"] = not audit["valid"]
    faded_profile = (isinstance(profile, str) and
                     profile.strip().upper() in FADED_PROFILES)
    audit["requires_explicit_calibrated_snr"] = faded_profile
    audit["vara_scorable"] = bool(
        audit["valid"] and passthrough is False and
        (not faded_profile or requested_snr3k is not None))
    if audit["valid"] and passthrough is True:
        audit["vara_scoring_disabled_reason"] = "passthrough_channel"
    elif audit["valid"] and faded_profile and requested_snr3k is None:
        audit["vara_scoring_disabled_reason"] = "faded_profile_not_physically_calibrated"
    if not audit["vara_scorable"]:
        return None, audit
    return audit["realized_snr3k_db"], audit

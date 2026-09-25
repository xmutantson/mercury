#!/usr/bin/env python3
"""preflight_overrides.py -- registry check for behaviour-restoring / defeat switches.

The registry (_research/PREFLIGHT_OVERRIDES.json) lists every MERCURY_* env seam that
restores a former behaviour or defeats a fix (census by git grep; unverified entries are
marked "[?]"). Rule:
  * an efficacy / normal run REJECTS when any registered name is present in its
    environment with a value outside that entry's inert_values ([?] entries: presence
    rejects);
  * a negative-control run declares NEGATIVE_CONTROL=<name> (env or --negative-control);
    exactly the declared switch may be active, the run is labelled NEGATIVE-CONTROL and can
    never be booked as a green gate or an efficacy result; any other active override
    still rejects.

Usage:
  preflight_overrides.py check [--env-file F] [--env K=V ...] [--from-os-environ]
                               [--negative-control NAME] [--json OUT]
  preflight_overrides.py list [--verified-only]
Exit: 0 = CLEAN, 3 = NEGATIVE-CONTROL (declared, valid), 1 = REJECT, 2 = unusable input.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

VERSION = "preflight_overrides/1.2 (2026-09-25; c_atoi = glibc (int)strtol: saturate to 64-bit long, wrap to int32)"
ROOT = Path(__file__).resolve().parents[1]
# A registry beside this file (the harness directory) wins over the workspace layout.
REGISTRY = (Path(__file__).resolve().parent / "PREFLIGHT_OVERRIDES.json"
            if (Path(__file__).resolve().parent / "PREFLIGHT_OVERRIDES.json").is_file()
            else ROOT / "_research" / "PREFLIGHT_OVERRIDES.json")


def load_registry(path: Path = REGISTRY):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("schema") != "preflight-overrides/1":
        raise ValueError("registry schema must be preflight-overrides/1")
    return {e["name"]: e for e in data["overrides"]}


C_SPACE = " \t\n\v\f\r"   # C isspace() in the "C" locale: what atoi() skips


LONG_MIN, LONG_MAX = -(1 << 63), (1 << 63) - 1   # glibc x86_64/aarch64 long (the fleet's C library)


def c_atoi(s) -> int:
    """glibc atoi() = (int) strtol(s, NULL, 10): skip leading whitespace, one optional sign, then ASCII digits; no digits -> 0
    ("yes", "off", "0x1" -> 0); strtol SATURATES to LONG_MIN/LONG_MAX (64-bit long) on overflow, and the cast to a 32-bit int keeps
    the low 32 bits (two's complement), so "4294967296" -> 0 (switches a lever OFF), "-4294967295" -> 1, "2147483648" ->
    -2147483648, "99999999999999999999" -> LONG_MAX -> -1 (ph_refute3 G-atoi; verified against glibc: _scratch/ph_refute3/atoi_glibc.py)."""
    s = str(s)
    i = 0
    while i < len(s) and s[i] in C_SPACE:
        i += 1
    sign = 1
    if i < len(s) and s[i] in "+-":
        sign = -1 if s[i] == "-" else 1
        i += 1
    j = i
    while j < len(s) and s[j] in "0123456789":
        j += 1
    if j == i:
        return 0
    v = max(LONG_MIN, min(LONG_MAX, sign * int(s[i:j])))   # strtol saturation
    return ((v + (1 << 31)) % (1 << 32)) - (1 << 31)       # (int) cast: low 32 bits, two's complement


def lever_off(value) -> bool:
    """The modem's own read of a default-ON lever: `e && *e && atoi(e) == 0` (e.g. arq_common.cc MERCURY_SCALABLE_SACK) and the
    helper `(e && *e) ? atoi(e) : 1` (telecom_system.cc env_i): unset or empty keeps the default; ANY other value whose C atoi()
    is 0 switches the lever off ("0", "00", " 0", "-0", "false", "off", "yes", "0x1")."""
    return value is not None and str(value) != "" and c_atoi(value) == 0


def off_by_rule(entry, value) -> bool:
    """Does `value` switch this default-ON lever off, by the lever's OWN read in the modem (entry off_rule, recorded with the read
    it came from in off_rule_source)? atoi0_nonempty = `e && *e && atoi(e) == 0`; atoi0_set = `if (e) { if (atoi(e) == 0) off }`
    (an empty value is set and parses to 0); strcmp = exact off_values; first_char_0 = `e && e[0] == '0'`; not_in = any set value
    outside on_values (a mode selector whose other modes restore the legacy path)."""
    v = str(value)
    rule = entry.get("off_rule", "atoi0_nonempty")
    if rule == "atoi0_nonempty":
        return lever_off(v)
    if rule == "atoi0_set":
        return c_atoi(v) == 0
    if rule == "strcmp":
        return v in {str(x) for x in entry.get("off_values", [])}
    if rule == "first_char_0":
        return v[:1] == "0"
    if rule == "not_in":
        return v not in {str(x) for x in entry.get("on_values", [])}
    raise ValueError("%s: unknown off_rule %r" % (entry.get("name"), rule))


def active(entry, value) -> bool:
    """A defeat/legacy switch is active for any value outside its inert_values; a default-ON lever (entry carries active_values
    ["0"], kind default-on-lever) is overridden by every value its own read in the modem treats as off (off_by_rule: C atoi
    semantics or the exact comparison the code makes, never string equality with "0"). Inert values compare stripped."""
    if value is None:
        return False
    if "active_values" in entry:
        if [str(x).strip() for x in entry["active_values"]] == ["0"]:
            return off_by_rule(entry, value)
        return str(value).strip() in {str(x).strip() for x in entry["active_values"]}
    v = str(value).strip()
    return v not in {str(x).strip() for x in entry.get("inert_values", [])}


def check_env(env: dict, negative_control: str | None = None, registry_path: Path = REGISTRY):
    reg = load_registry(registry_path)
    declared = negative_control or env.get("NEGATIVE_CONTROL") or None
    found = {k: v for k, v in env.items() if k in reg and active(reg[k], v)}
    reasons = []
    if declared and declared not in reg:
        reasons.append("NEGATIVE_CONTROL=%s is not a registered override" % declared)
    if declared and declared in reg and declared not in found:
        reasons.append("NEGATIVE_CONTROL=%s declared but the switch is not active in the environment" % declared)
    for name, value in sorted(found.items()):
        if name == declared:
            continue
        reasons.append("registered override %s=%s active (%s, status %s) without a negative-control declaration"
                       % (name, value, reg[name]["kind"], reg[name]["status"]))
    if reasons:
        status = "REJECT"
    elif declared:
        status = "NEGATIVE-CONTROL"
    else:
        status = "CLEAN"
    return {"tool": VERSION, "status": status, "reasons": reasons, "declared_negative_control": declared,
            "active_overrides": found, "registry": str(registry_path), "registry_names": len(reg)}


def read_env_file(path):
    env = {}
    for raw in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        raw = raw.strip()
        if raw and not raw.startswith("#") and "=" in raw:
            k, v = raw.split("=", 1)
            env[k.strip()] = v.strip()
    return env


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check")
    c.add_argument("--env-file")
    c.add_argument("--env", action="append", default=[])
    c.add_argument("--from-os-environ", action="store_true")
    c.add_argument("--negative-control")
    c.add_argument("--registry", type=Path, default=REGISTRY)
    c.add_argument("--json", type=Path)
    ls = sub.add_parser("list")
    ls.add_argument("--verified-only", action="store_true")
    ls.add_argument("--registry", type=Path, default=REGISTRY)
    a = ap.parse_args(argv)
    try:
        if a.cmd == "list":
            for name, e in sorted(load_registry(a.registry).items()):
                if a.verified_only and e["status"] != "verified":
                    continue
                print("%-48s %-16s %-8s %s" % (name, e["kind"], e["status"], ",".join(e["reads"][:2])))
            return 0
        env = {}
        if a.from_os_environ:
            env.update(os.environ)
        if a.env_file:
            env.update(read_env_file(a.env_file))
        for kv in a.env:
            if "=" in kv:
                k, v = kv.split("=", 1)
                env[k] = v
        rep = check_env(env, a.negative_control, a.registry)
    except Exception as exc:
        print("PREFLIGHT-OVERRIDES REJECT: unusable input: %s" % exc)
        return 2
    if a.json:
        a.json.write_text(json.dumps(rep, indent=1) + "\n", encoding="utf-8")
    print("PREFLIGHT-OVERRIDES %s%s" % (rep["status"], (": " + "; ".join(rep["reasons"])) if rep["reasons"] else ""))
    return {"CLEAN": 0, "NEGATIVE-CONTROL": 3, "REJECT": 1}[rep["status"]]


if __name__ == "__main__":
    raise SystemExit(main())

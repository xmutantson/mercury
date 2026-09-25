"""override_guard.py -- launch-time check of registered behaviour-restoring env switches.

Thin adapter so a staged harness directory can run the registry check of
tools/preflight_overrides.py. Search order for the checker and its registry:
  1. $MERCURY_PREFLIGHT_REGISTRY (registry JSON path) + preflight_overrides.py next to
     this file or in ../../ (workspace tools/);
  2. PREFLIGHT_OVERRIDES.json next to this file (staged harness directory);
  3. ../../../_research/PREFLIGHT_OVERRIDES.json (workspace layout).
If no registry is reachable the result is status UNAVAILABLE (recorded in the cell
result; the evidence manifest re-checks the recorded env), never a silent CLEAN.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _registry_path():
    cands = [os.environ.get("MERCURY_PREFLIGHT_REGISTRY"),
             os.path.join(HERE, "PREFLIGHT_OVERRIDES.json"),
             os.path.normpath(os.path.join(HERE, "..", "..", "..", "_research", "PREFLIGHT_OVERRIDES.json"))]
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return None


def _checker():
    for d in (HERE, os.path.normpath(os.path.join(HERE, "..", ".."))):
        if os.path.isfile(os.path.join(d, "preflight_overrides.py")):
            if d not in sys.path:
                sys.path.insert(0, d)
            import preflight_overrides  # noqa: E402
            return preflight_overrides
    return None


def check(cell_env, negative_control=None):
    reg = _registry_path()
    po = _checker()
    if reg is None or po is None:
        return {"status": "UNAVAILABLE", "reasons": ["registry or checker not reachable from %s" % HERE],
                "declared_negative_control": negative_control}
    from pathlib import Path
    env = {k: v for k, v in cell_env.items() if k.startswith("MERCURY_") or k == "NEGATIVE_CONTROL"}
    rep = po.check_env(env, negative_control=negative_control, registry_path=Path(reg))
    return rep

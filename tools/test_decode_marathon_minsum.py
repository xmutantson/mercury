#!/usr/bin/env python3
"""
feat/decode-marathon LEVER E regression: MIN-SUM CHECK-NODE update for the SPA
decoder (env MERCURY_LDPC_MINSUM, default off). The ONLY lossy lever, but a
decode-QUALITY candidate — NMS/OMS/SCMS replace the libm tanh/atanh check node
with the magnitude-min + sign-product (two-smallest-magnitude trick), and the
normalization / self-correction is the missing ingredient that may recover the
naive-SPA BP-non-convergence floor (memory cfg16_decode_loss_is_ldpc_bp).

References: Fossorier/Mihaljevic/Imai TCOM 1999 (min-sum / two-min); Chen &
Fossorier TCOM 2002 + Comm.Lett. 2002 (NMS/OMS, alpha~0.8); Savin ISIT 2008
(SCMS, erase sign-flipping var->check messages).

Same 27-cell coded SFO-GRID fleet vehicle as the sibling levers (decode-marathon.md
§3 lever D, decode-marathon-C.md). Fail-before / pass-after per CLAUDE.md §3.

Checks
------
TEST-1  DEFAULT-OFF BYTE-IDENTICAL: with MERCURY_LDPC_MINSUM unset, the 27-cell
        [SFO-GRID-CODED] render md5 is IDENTICAL to the base @4ad4f70 render.
        Zero behaviour change off (SPA tanh/atanh bit-for-bit).

TEST-2  MIN-SUM DECODES, NO CLEAN REGRESSION: with min-sum enabled (each of NMS,
        OMS, SCMS), across the cells where SPA fully converges (flooding_ok ==
        tot) the min-sum decode is NOT regressed (ms_ok == tot) — the lossy
        kernel must not drop a clean codeword.

TEST-3  NON-CONVERGER NOT REGRESSED (fleet): with the recommended pairing
        (SCMS + layered), the fleet total codewords_decoded is >= the SPA fleet
        total (the decode-quality intent: recover BP non-convergers, never lose).

Exit 0 = all pass.
"""
import hashlib
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

CHANS = (0, 1, 2)
SEEDS = (12345, 999, 77)
ESN0S = (15, 16, 17)

# Base @4ad4f70 27-cell [SFO-GRID-CODED] render md5 (NSYMB=600, -s 16, CODED).
# Identical to the documented base @943a083 (the marathon levers are default-off
# byte-identical), decode-marathon-E.md §6 TEST-1. Overridable for a re-pin.
BASE_MD5 = os.environ.get("DECODE_MARATHON_BASE_MD5",
                          "4575011ba38e23575ef3f3bbce0703b2")

_CODED_RE = re.compile(
    r"codewords_decoded=(\d+)/(\d+)\s+fail=\d+\s+post_FEC_info_BER=([0-9.eE+-]+)")

# Min-sum env presets under test. None => SPA (off). Each tuple is a dict of env.
MS_VARIANTS = {
    "nms":  {"MERCURY_LDPC_MINSUM": "1", "MERCURY_LDPC_MINSUM_VARIANT": "nms"},
    "oms":  {"MERCURY_LDPC_MINSUM": "1", "MERCURY_LDPC_MINSUM_VARIANT": "oms",
             "MERCURY_LDPC_MS_ALPHA": "0.3"},
    "scms": {"MERCURY_LDPC_MINSUM": "1", "MERCURY_LDPC_MINSUM_VARIANT": "scms"},
}
# All marathon decode gates we must clear when toggling, so each run is clean.
_MS_KEYS = ("MERCURY_LDPC_MINSUM", "MERCURY_LDPC_MINSUM_VARIANT",
            "MERCURY_LDPC_MS_ALPHA", "MERCURY_LDPC_LAYERED")


def find_binary():
    for c in (os.environ.get("MERCURY_BIN"),
              os.path.join(ROOT, "mercury.exe"),
              os.path.join(ROOT, "mercury")):
        if c and os.path.exists(c):
            return c
    sys.exit("ERROR: mercury binary not found. Set MERCURY_BIN=<path>.")


def run_cell(binary, chan, seed, esn0, ms_env=None, layered=False, timeout=300):
    env = dict(os.environ)
    for k in _MS_KEYS:
        env.pop(k, None)
    env.update({
        "MERCURY_SFO_GRID": "1",
        "MERCURY_SFO_GRID_CODED": "1",
        "MERCURY_SFO_GRID_NSYMB": "600",
        "MERCURY_SFO_GRID_CHAN": str(chan),
        "MERCURY_SFO_GRID_SEED": str(seed),
        "MERCURY_SFO_GRID_ESN0": str(esn0),
    })
    if ms_env:
        env.update(ms_env)
    if layered:
        env["MERCURY_LDPC_LAYERED"] = "1"
    p = subprocess.run([binary, "-m", "PLOT_PASSBAND", "-s", "16"],
                       env=env, capture_output=True, text=True, timeout=timeout)
    return p.stdout + p.stderr


def coded_lines(out):
    return [ln for ln in out.splitlines() if "[SFO-GRID-CODED]" in ln]


def parse_cell(out):
    ok = tot = None
    for ln in out.splitlines():
        m = _CODED_RE.search(ln)
        if m:
            ok, tot = int(m.group(1)), int(m.group(2))
    return ok, tot


def grid(binary, ms_env=None, layered=False):
    render, cells = [], {}
    for chan in CHANS:
        for seed in SEEDS:
            for esn0 in ESN0S:
                out = run_cell(binary, chan, seed, esn0, ms_env, layered)
                render.extend(coded_lines(out))
                cells[(chan, seed, esn0)] = parse_cell(out)
    return "\n".join(render) + "\n", cells


def md5(s):
    return hashlib.md5(s.encode()).hexdigest()


def fleet_total(cells):
    return sum(ok for ok, _ in cells.values() if ok is not None)


def main():
    binary = find_binary()
    print(f"[test_decode_marathon_minsum] binary = {binary}")
    fails = []

    # --- TEST-1: default-off byte-identical ---
    off_render, spa = grid(binary, ms_env=None, layered=False)
    off_md5 = md5(off_render)
    print(f"[TEST-1] OFF render md5 = {off_md5}  (base = {BASE_MD5})")
    if off_md5 == BASE_MD5:
        print("[TEST-1] PASS: default-off SPA render byte-identical to base.")
    else:
        print("[TEST-1] FAIL: OFF render differs from base @4ad4f70!")
        fails.append("TEST-1")
    spa_total = fleet_total(spa)
    print(f"[TEST-1] SPA fleet codewords_decoded total = {spa_total}")

    # --- TEST-2: min-sum decodes, no clean-cell regression (each variant) ---
    for name, ms_env in MS_VARIANTS.items():
        layered = (name == "scms")  # SCMS pairs with layered (decode-marathon-E.md §2)
        _, msc = grid(binary, ms_env=ms_env, layered=layered)
        regressed = []
        for key, (ok, tot) in spa.items():
            if ok is None or tot is None:
                continue
            if ok == tot:  # SPA fully converged this cell
                m_ok, m_tot = msc[key]
                if m_ok != m_tot:
                    regressed.append((key, m_ok, m_tot))
        m_total = fleet_total(msc)
        if not regressed:
            print(f"[TEST-2:{name}] PASS: no clean-cell regression; "
                  f"fleet total = {m_total} (SPA {spa_total})")
        else:
            print(f"[TEST-2:{name}] FAIL: clean cells regressed: {regressed[:5]}")
            fails.append(f"TEST-2:{name}")

    # --- TEST-3: SCMS+layered fleet total not below SPA (quality intent) ---
    _, scms = grid(binary, ms_env=MS_VARIANTS["scms"], layered=True)
    scms_total = fleet_total(scms)
    if scms_total >= spa_total:
        print(f"[TEST-3] PASS: SCMS+layered fleet total {scms_total} "
              f">= SPA {spa_total} (non-convergers not regressed)")
    else:
        print(f"[TEST-3] FAIL: SCMS+layered fleet total {scms_total} "
              f"< SPA {spa_total}")
        fails.append("TEST-3")

    if fails:
        print(f"[RESULT] FAIL ({', '.join(fails)})")
        return 1
    print("[RESULT] ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

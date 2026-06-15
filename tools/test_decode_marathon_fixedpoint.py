#!/usr/bin/env python3
"""
feat/decode-marathon LEVER G regression: FIXED-POINT (int16) MIN-SUM check-node /
var-node / syndrome (env MERCURY_LDPC_FIXEDPOINT, default off; REQUIRES
MERCURY_LDPC_MINSUM). Quantizes the min-sum decode STATE (messages R/Q + the APP)
to int16 with a fixed LLR scale (Q-format, MERCURY_LDPC_FIXEDPOINT_SCALE, default
64 => Q9.6) and saturation (MERCURY_LDPC_FIXEDPOINT_SAT, default 4096 fixed units
= +-64.0 LLR). The min-sum kernel (lever E, ms_check_row) uses NO transcendentals
=> it quantizes cleanly to saturating integer arithmetic; on NEON int16 gives ~2x
the lanes of float (8x int16 vs 4x float / 128-bit reg) on the Pi5/A76. THIS test
proves the QUANTIZATION CORRECTNESS (BER parity) on Windows; the SIMD speedup is
Pi-only.

References (cited, not invented): Zhang/Wang/Parhi ISCAS 2001 (finite-precision
BP); Chen/Dholakia/Eleftheriou/Fossorier/Hu TCOM 2005 (quantized normalized/offset
min-sum, scale + uniform quantizer); xdsopl/LDPC (saturating int8 NMS/OMS/SCMS
reference, channel LLR = float LLR x FACTOR); AFF3CT (production quantized MS BP).
Standard result: a well-scaled int16 (even int8) min-sum is within ~0.1 dB of
float min-sum.

Same 27-cell coded SFO-GRID fleet vehicle as the sibling levers
(decode-marathon-E.md §6). Fail-before / pass-after per CLAUDE.md §3.

Checks
------
TEST-1  DEFAULT-OFF BYTE-IDENTICAL: with MERCURY_LDPC_FIXEDPOINT unset (and
        min-sum also unset), the 27-cell [SFO-GRID-CODED] render md5 is IDENTICAL
        to the base @d26de02 render. Zero behaviour change off (SPA tanh/atanh
        bit-for-bit). This is the SAME render md5 the lever-E driver pins.

TEST-2  FIXED-POINT DECODES, NO CLEAN REGRESSION vs FLOAT min-sum: across the
        cells where FLOAT NMS fully converges (ms_ok == tot), the int16 NMS decode
        is NOT regressed (fp_ok == tot) — the quantized kernel must not drop a
        clean codeword the float kernel got. Run for NMS-flooding and SCMS-layered.

TEST-3  FIXED-POINT FLEET TOTAL within tolerance of FLOAT min-sum (the ~0.1 dB
        parity claim): the int16 fleet codewords_decoded total is >= the float
        min-sum fleet total minus a small slack (quantization is allowed to cost a
        few non-marginal cells but must not collapse). NMS-flooding + SCMS-layered.

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

# Base @d26de02 27-cell [SFO-GRID-CODED] render md5 (NSYMB=600, -s 16, CODED).
# Identical to the documented base @4ad4f70 / @943a083 (the marathon levers are
# default-off byte-identical), decode-marathon-E.md §6 TEST-1. Overridable for a
# re-pin via DECODE_MARATHON_BASE_MD5.
BASE_MD5 = os.environ.get("DECODE_MARATHON_BASE_MD5",
                          "4575011ba38e23575ef3f3bbce0703b2")

_CODED_RE = re.compile(
    r"codewords_decoded=(\d+)/(\d+)\s+fail=\d+\s+post_FEC_info_BER=([0-9.eE+-]+)")

# Float vs fixed-point presets. Each "float" entry is the float min-sum reference;
# the matching "fp" entry adds MERCURY_LDPC_FIXEDPOINT=1 (int16) — everything else
# identical, so a divergence is purely the quantization.
PRESETS = {
    "nms_float":  {"env": {"MERCURY_LDPC_MINSUM": "1",
                           "MERCURY_LDPC_MINSUM_VARIANT": "nms"},
                   "layered": False},
    "nms_fp":     {"env": {"MERCURY_LDPC_MINSUM": "1",
                           "MERCURY_LDPC_MINSUM_VARIANT": "nms",
                           "MERCURY_LDPC_FIXEDPOINT": "1"},
                   "layered": False},
    "scms_float": {"env": {"MERCURY_LDPC_MINSUM": "1",
                           "MERCURY_LDPC_MINSUM_VARIANT": "scms"},
                   "layered": True},
    "scms_fp":    {"env": {"MERCURY_LDPC_MINSUM": "1",
                           "MERCURY_LDPC_MINSUM_VARIANT": "scms",
                           "MERCURY_LDPC_FIXEDPOINT": "1"},
                   "layered": True},
}
# All marathon decode gates we must clear when toggling, so each run is clean.
_KEYS = ("MERCURY_LDPC_MINSUM", "MERCURY_LDPC_MINSUM_VARIANT",
         "MERCURY_LDPC_MS_ALPHA", "MERCURY_LDPC_LAYERED",
         "MERCURY_LDPC_FIXEDPOINT", "MERCURY_LDPC_FIXEDPOINT_SCALE",
         "MERCURY_LDPC_FIXEDPOINT_SAT")

# TEST-3 fleet slack: quantization may cost a few cells but must not collapse.
# 27 cells * up to 25 cw each; allow up to 10 codewords of fleet loss.
FLEET_SLACK = int(os.environ.get("DECODE_MARATHON_FP_SLACK", "10"))


def find_binary():
    for c in (os.environ.get("MERCURY_BIN"),
              os.path.join(ROOT, "mercury.exe"),
              os.path.join(ROOT, "mercury")):
        if c and os.path.exists(c):
            return c
    sys.exit("ERROR: mercury binary not found. Set MERCURY_BIN=<path>.")


def run_cell(binary, chan, seed, esn0, extra_env=None, layered=False, timeout=300):
    env = dict(os.environ)
    for k in _KEYS:
        env.pop(k, None)
    env.update({
        "MERCURY_SFO_GRID": "1",
        "MERCURY_SFO_GRID_CODED": "1",
        "MERCURY_SFO_GRID_NSYMB": "600",
        "MERCURY_SFO_GRID_CHAN": str(chan),
        "MERCURY_SFO_GRID_SEED": str(seed),
        "MERCURY_SFO_GRID_ESN0": str(esn0),
    })
    if extra_env:
        env.update(extra_env)
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


def grid(binary, extra_env=None, layered=False):
    render, cells = [], {}
    for chan in CHANS:
        for seed in SEEDS:
            for esn0 in ESN0S:
                out = run_cell(binary, chan, seed, esn0, extra_env, layered)
                render.extend(coded_lines(out))
                cells[(chan, seed, esn0)] = parse_cell(out)
    return "\n".join(render) + "\n", cells


def md5(s):
    return hashlib.md5(s.encode()).hexdigest()


def fleet_total(cells):
    return sum(ok for ok, _ in cells.values() if ok is not None)


def main():
    binary = find_binary()
    print(f"[test_decode_marathon_fixedpoint] binary = {binary}")
    fails = []

    # --- TEST-1: default-off byte-identical (SPA, no min-sum, no fixed-point) ---
    off_render, _ = grid(binary, extra_env=None, layered=False)
    off_md5 = md5(off_render)
    print(f"[TEST-1] OFF render md5 = {off_md5}  (base = {BASE_MD5})")
    if off_md5 == BASE_MD5:
        print("[TEST-1] PASS: default-off SPA render byte-identical to base.")
    else:
        print("[TEST-1] FAIL: OFF render differs from base @d26de02!")
        fails.append("TEST-1")

    # --- TEST-2 + TEST-3 per pairing: float reference vs int16 fixed-point ---
    for pair, (float_name, fp_name) in (("NMS-flooding", ("nms_float", "nms_fp")),
                                        ("SCMS-layered", ("scms_float", "scms_fp"))):
        fl = PRESETS[float_name]
        fp = PRESETS[fp_name]
        _, fcells = grid(binary, extra_env=fl["env"], layered=fl["layered"])
        _, qcells = grid(binary, extra_env=fp["env"], layered=fp["layered"])
        f_total = fleet_total(fcells)
        q_total = fleet_total(qcells)

        # TEST-2: no clean-cell regression (float fully converged => int16 too).
        regressed = []
        for key, (ok, tot) in fcells.items():
            if ok is None or tot is None:
                continue
            if ok == tot:  # float min-sum fully converged this cell
                q_ok, q_tot = qcells[key]
                if q_ok != q_tot:
                    regressed.append((key, q_ok, q_tot))
        if not regressed:
            print(f"[TEST-2:{pair}] PASS: no clean-cell regression "
                  f"(int16 fleet {q_total}, float {f_total})")
        else:
            print(f"[TEST-2:{pair}] FAIL: clean cells regressed under int16: "
                  f"{regressed[:5]}")
            fails.append(f"TEST-2:{pair}")

        # TEST-3: fleet total within slack of float (the ~0.1 dB parity claim).
        if q_total >= f_total - FLEET_SLACK:
            print(f"[TEST-3:{pair}] PASS: int16 fleet {q_total} >= float "
                  f"{f_total} - {FLEET_SLACK} (quantization parity)")
        else:
            print(f"[TEST-3:{pair}] FAIL: int16 fleet {q_total} < float "
                  f"{f_total} - {FLEET_SLACK} (quantization collapsed)")
            fails.append(f"TEST-3:{pair}")

    if fails:
        print(f"[RESULT] FAIL ({', '.join(fails)})")
        return 1
    print("[RESULT] ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

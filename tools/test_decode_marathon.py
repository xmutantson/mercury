#!/usr/bin/env python3
"""
feat/decode-marathon LEVER D regression: LAYERED (row-by-row / horizontal-shuffled)
BP scheduling for the SPA decoder (env MERCURY_LDPC_LAYERED, default off).

Drives the coded SFO-GRID harness — the same 27-cell fleet vehicle the sibling
levers (ldpc-decode-accel.md §1, turnaround-eff.md §6) use. Fail-before /
pass-after contract.

Layered BP processes the P parity-check rows ONE AT A TIME, immediately folding
each updated check->var message into a running a-posteriori sum so a LATER row in
the SAME iteration reads fresher var->check extrinsics ("the most recent
information is disseminated"). Standard result (Hocevar SIPS 2004;
Sharon/Litsyn/Goldberger IT 2007): ~2x faster convergence (about half the
iterations) at bit-comparable BER, often a slightly LOWER non-converger floor.

Checks
------
TEST-1  DEFAULT-OFF BYTE-IDENTICAL: with MERCURY_LDPC_LAYERED unset, the 27-cell
        [SFO-GRID-CODED] render md5 is IDENTICAL to the base @943a083 render.
        Zero behaviour change off (flooding bit-for-bit).

TEST-2  BER-COMPARABLE: with MERCURY_LDPC_LAYERED=1, across the 27 cells
        codewords_decoded is NOT regressed (layered_ok >= flooding_ok per cell)
        and post_FEC_info_BER is comparable (layered_ber <= flooding_ber + eps).
        The fleet non-converger rate must NOT regress.

TEST-3  ITER DROP: on the cells that actually converge (>=1 codeword decoded,
        not cap-pinned), layered iter_mean is materially lower than flooding
        (the ~2x convergence-speed win). Aggregated over converging cells the
        layered/flooding iter_mean ratio is well below 1.

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

# Base @943a083 27-cell [SFO-GRID-CODED] render md5 (NSYMB=600, -s 16, CODED).
# Computed from the clean base @943a083 binary (decode-marathon.md §3 TEST-1).
# Overridable via env DECODE_MARATHON_BASE_MD5 for a re-pin on a later base.
BASE_MD5 = os.environ.get("DECODE_MARATHON_BASE_MD5",
                          "4575011ba38e23575ef3f3bbce0703b2")

_CODED_RE = re.compile(
    r"codewords_decoded=(\d+)/(\d+)\s+fail=\d+\s+post_FEC_info_BER=([0-9.eE+-]+)")
_ITER_RE = re.compile(
    r"iter_mean=([0-9.eE+-]+)\s+iter_min=(\d+)\s+iter_max=(\d+)")


def find_binary():
    for c in (os.environ.get("MERCURY_BIN"),
              os.path.join(ROOT, "mercury.exe"),
              os.path.join(ROOT, "mercury")):
        if c and os.path.exists(c):
            return c
    sys.exit("ERROR: mercury binary not found. Set MERCURY_BIN=<path>.")


def run_cell(binary, chan, seed, esn0, layered, timeout=300):
    env = dict(os.environ)
    env.update({
        "MERCURY_SFO_GRID": "1",
        "MERCURY_SFO_GRID_CODED": "1",
        "MERCURY_SFO_GRID_NSYMB": "600",
        "MERCURY_SFO_GRID_CHAN": str(chan),
        "MERCURY_SFO_GRID_SEED": str(seed),
        "MERCURY_SFO_GRID_ESN0": str(esn0),
    })
    if layered:
        env["MERCURY_LDPC_LAYERED"] = "1"
    else:
        env.pop("MERCURY_LDPC_LAYERED", None)
    p = subprocess.run([binary, "-m", "PLOT_PASSBAND", "-s", "16"],
                       env=env, capture_output=True, text=True, timeout=timeout)
    return p.stdout + p.stderr


def coded_lines(out):
    return [ln for ln in out.splitlines() if "[SFO-GRID-CODED]" in ln]


def parse_cell(out):
    ok = tot = None
    ber = None
    iter_mean = iter_max = None
    for ln in out.splitlines():
        m = _CODED_RE.search(ln)
        if m:
            ok, tot, ber = int(m.group(1)), int(m.group(2)), float(m.group(3))
        m2 = _ITER_RE.search(ln)
        if m2:
            iter_mean, iter_max = float(m2.group(1)), int(m2.group(3))
    return ok, tot, ber, iter_mean, iter_max


def grid(binary, layered):
    render = []
    cells = {}
    for chan in CHANS:
        for seed in SEEDS:
            for esn0 in ESN0S:
                out = run_cell(binary, chan, seed, esn0, layered)
                render.extend(coded_lines(out))
                cells[(chan, seed, esn0)] = parse_cell(out)
    return "\n".join(render) + "\n", cells


def md5(s):
    return hashlib.md5(s.encode()).hexdigest()


def main():
    binary = find_binary()
    print(f"[test_decode_marathon] binary = {binary}")
    fails = []

    off_render, flood = grid(binary, layered=False)
    off_md5 = md5(off_render)
    print(f"[TEST-1] OFF render md5 = {off_md5}  (base = {BASE_MD5 or '<unset>'})")
    if not BASE_MD5:
        print("[TEST-1] SKIP md5 compare (DECODE_MARATHON_BASE_MD5 unset); "
              "pin this value from a clean base @943a083 build.")
    elif off_md5 == BASE_MD5:
        print("[TEST-1] PASS: default-off byte-identical to base @943a083")
    else:
        print("[TEST-1] FAIL: OFF render differs from base!")
        fails.append("TEST-1")

    _, lay = grid(binary, layered=True)

    # ---- TEST-2: BER-comparable, non-converger rate not regressed ----------
    fleet_flood_ok = fleet_lay_ok = fleet_tot = 0
    regressions = []
    eps = 1e-6
    for cell in flood:
        fok, ftot, fber, _, _ = flood[cell]
        lok, ltot, lber, _, _ = lay[cell]
        fleet_flood_ok += fok
        fleet_lay_ok += lok
        fleet_tot += ftot
        if lok < fok or lber > fber + eps:
            regressions.append((cell, fok, lok, fber, lber))
    print(f"[TEST-2] fleet codewords flooding={fleet_flood_ok}/{fleet_tot} "
          f"layered={fleet_lay_ok}/{fleet_tot}  (non-converger rate "
          f"flooding={fleet_tot-fleet_flood_ok} layered={fleet_tot-fleet_lay_ok})")
    if not regressions:
        print("[TEST-2] PASS: no cell regressed (layered_ok>=flooding_ok and "
              "BER comparable); fleet non-converger rate not regressed")
    else:
        print(f"[TEST-2] FAIL: {len(regressions)} cell(s) regressed: {regressions}")
        fails.append("TEST-2")

    # ---- TEST-3: iter_mean drops on converging cells -----------------------
    conv_flood_sum = conv_lay_sum = 0.0
    nconv = 0
    examples = []
    for cell in flood:
        fok, ftot, _, fim, fmax = flood[cell]
        lok, ltot, _, lim, lmax = lay[cell]
        # A converging cell: all codewords decoded and not cap-pinned (so the mean
        # reflects real convergence iterations, not the 101 cap-mass).
        if fok == ftot and lok == ltot and ftot > 0 and fim is not None and fim < 90:
            conv_flood_sum += fim
            conv_lay_sum += lim
            nconv += 1
            examples.append((cell, fim, lim))
    if nconv > 0:
        ratio = conv_lay_sum / conv_flood_sum if conv_flood_sum > 0 else 1.0
        print(f"[TEST-3] over {nconv} fully-converging cells: "
              f"flooding iter_mean sum={conv_flood_sum:.2f} "
              f"layered iter_mean sum={conv_lay_sum:.2f}  ratio={ratio:.3f}")
        for c, fi, li in examples[:6]:
            print(f"          {c}: flooding={fi:.2f} -> layered={li:.2f}")
        if ratio < 0.9:
            print(f"[TEST-3] PASS: layered converges ~{1.0/ratio:.2f}x faster "
                  f"(iter_mean ratio {ratio:.3f} < 0.9)")
        else:
            print(f"[TEST-3] FAIL: layered iter_mean ratio {ratio:.3f} not < 0.9")
            fails.append("TEST-3")
    else:
        print("[TEST-3] FAIL: no fully-converging cell to measure the iter drop")
        fails.append("TEST-3")

    print()
    if fails:
        print(f"[test_decode_marathon] FAILED: {fails}")
        sys.exit(1)
    print("[test_decode_marathon] ALL PASS")


if __name__ == "__main__":
    main()

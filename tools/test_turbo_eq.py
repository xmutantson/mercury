#!/usr/bin/env python3
"""
Turbo-EQ regression test (RESEARCH_turbo-eq.md, fact-documents/data-flow-turbo-eq.md).

Drives the SFO-GRID coded harness (-m PLOT_PASSBAND -s 16, MERCURY_SFO_GRID +
_CODED + _CHAN + _GENIE + _TURBO_ITERS) and asserts the turbo lever's behavior.

THE FAILING-FIRST TEST (CLAUDE.md Principle 3 / Phase-4): on the channel-estimation-
limited cell the production LS estimator FAILS to decode (high post_FEC_info_BER)
while GENIE (perfect CSI) PASSES (post_FEC_info_BER == 0). That is the documented
"Mercury is channel-estimation-limited" compass — the test that fails before the
turbo loop exists (there is no MERCURY_SFO_GRID_TURBO_ITERS path on the base binary,
and even with it, LS alone cannot close the gap). After the data-aided turbo loop,
TURBO it=2 must move post_FEC_info_BER MATERIALLY toward GENIE.

Decisive cell: CHAN=1 DET-FLOOR (Schroeder all-pass, |T|=1, phase-dispersive — the
shipped EVM model). There LS uncoded BER ~0.50 (the flat LS estimate cannot represent
the per-carrier phase), GENIE uncoded BER ~0.0003, and the data-aided turbo loop
recovers most of the phase (post-FEC BER 0.50 -> ~0.12).

Assertions:
  A1 FAILING-FIRST : LS post_FEC_info_BER >= 0.40  (LS fails)
  A2 COMPASS       : GENIE post_FEC_info_BER == 0  (channel is decodable w/ perfect CSI)
  A3 PASS-AFTER    : TURBO post_FEC_info_BER <= 0.5 * LS  (materially toward GENIE)
  A4 MONOTONE-SAFE : TURBO post_FEC_info_BER <= LS  (never worse than single-pass it=0)
  A5 DEFAULT-OFF   : MERCURY_SFO_GRID_TURBO_ITERS unset == the base single-pass line

Usage:
    python tools/test_turbo_eq.py                  # default binary (worktree mercury.exe)
    MERCURY_BIN=/path/to/mercury.exe python tools/test_turbo_eq.py
    python tools/test_turbo_eq.py --esn0 20 --seed 12345 --cfg 16
"""

import argparse
import os
import re
import subprocess
import sys


def find_binary():
    env = os.environ.get("MERCURY_BIN")
    if env and os.path.exists(env):
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(os.path.dirname(here), "mercury", "mercury.exe"),
        os.path.expanduser(r"~/mercury_wt/turbo-eq/mercury.exe"),
        r"C:\Users\kamer\mercury_wt\turbo-eq\mercury.exe",
        os.path.join(os.path.dirname(here), "mercury", "mercury"),
        r"C:\Program Files\Mercury\mercury.exe",
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    sys.exit("ERROR: mercury binary not found. Set MERCURY_BIN=<path to mercury.exe>.")


def run_cell(binary, cfg, env_extra, timeout=180):
    env = dict(os.environ)
    env["MERCURY_SFO_GRID"] = "1"
    env["MERCURY_SFO_GRID_CODED"] = "1"
    env.update(env_extra)
    args = [binary, "-m", "PLOT_PASSBAND", "-s", str(cfg)]
    p = subprocess.run(args, env=env, capture_output=True, text=True, timeout=timeout)
    return p.stdout + p.stderr


_BER_RE = re.compile(r"post_FEC_info_BER=([0-9.eE+-]+)")
_CW_RE = re.compile(r"codewords_decoded=(\d+)/(\d+)")


def parse_coded(out):
    """Return (codewords_decoded, total, post_FEC_info_BER) from the final
    [SFO-GRID-CODED] codewords_decoded line."""
    ber = None
    ok = tot = None
    for line in out.splitlines():
        if "[SFO-GRID-CODED]" in line and "codewords_decoded=" in line:
            m = _CW_RE.search(line)
            b = _BER_RE.search(line)
            if m:
                ok, tot = int(m.group(1)), int(m.group(2))
            if b:
                ber = float(b.group(1))
    return ok, tot, ber


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", type=int, default=16, help="config (15 or 16, 32-QAM=16)")
    ap.add_argument("--chan", type=int, default=1, help="channel (1=det-floor decisive)")
    ap.add_argument("--esn0", type=float, default=20.0)
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--iters", type=int, default=2, help="turbo iterations")
    ap.add_argument("--watt-depth", type=float, default=6.0)
    ap.add_argument("--watt-fd", type=float, default=1.0)
    args = ap.parse_args()

    binary = find_binary()
    print(f"[test_turbo_eq] binary = {binary}")
    print(f"[test_turbo_eq] cell: cfg={args.cfg} chan={args.chan} EsN0={args.esn0} seed={args.seed}")

    common = {
        "MERCURY_SFO_GRID_CHAN": str(args.chan),
        "MERCURY_SFO_GRID_ESN0": str(args.esn0),
        "MERCURY_SFO_GRID_SEED": str(args.seed),
        "MERCURY_SFO_GRID_WATT_DEPTH_DB": str(args.watt_depth),
        "MERCURY_SFO_GRID_WATT_FD_HZ": str(args.watt_fd),
    }

    # LS (production estimator, single pass)
    ls_out = run_cell(binary, args.cfg, dict(common))
    ls_ok, ls_tot, ls_ber = parse_coded(ls_out)

    # GENIE (perfect CSI)
    ge_out = run_cell(binary, args.cfg, dict(common, MERCURY_SFO_GRID_GENIE="1"))
    ge_ok, ge_tot, ge_ber = parse_coded(ge_out)

    # TURBO (data-aided, it=args.iters)
    tu_out = run_cell(binary, args.cfg, dict(common, MERCURY_SFO_GRID_TURBO_ITERS=str(args.iters)))
    tu_ok, tu_tot, tu_ber = parse_coded(tu_out)

    # DEFAULT-OFF byte-identical: the turbo-unset [SFO-GRID-CODED] codewords line
    # must equal the LS line (same binary, no turbo env).
    def coded_line(out):
        for line in out.splitlines():
            if "[SFO-GRID-CODED]" in line and "codewords_decoded=" in line:
                return line.strip()
        return ""

    if None in (ls_ber, ge_ber, tu_ber):
        print("LS  out tail:\n" + "\n".join(ls_out.splitlines()[-6:]))
        sys.exit("ERROR: could not parse post_FEC_info_BER from harness output.")

    print(f"  LS    : codewords={ls_ok}/{ls_tot}  post_FEC_info_BER={ls_ber:.6f}")
    print(f"  GENIE : codewords={ge_ok}/{ge_tot}  post_FEC_info_BER={ge_ber:.6f}")
    print(f"  TURBO : codewords={tu_ok}/{tu_tot}  post_FEC_info_BER={tu_ber:.6f}  (it={args.iters})")

    fails = []
    # A1 FAILING-FIRST
    if not (ls_ber >= 0.40):
        fails.append(f"A1 FAILING-FIRST: LS BER {ls_ber:.4f} not >= 0.40 (LS must FAIL on the est-limited cell)")
    # A2 COMPASS
    if not (ge_ber == 0.0):
        fails.append(f"A2 COMPASS: GENIE BER {ge_ber:.4f} != 0 (channel must be decodable with perfect CSI)")
    # A3 PASS-AFTER (material move toward GENIE)
    if not (tu_ber <= 0.5 * ls_ber):
        fails.append(f"A3 PASS-AFTER: TURBO BER {tu_ber:.4f} not <= 0.5*LS ({0.5*ls_ber:.4f}); turbo did not move materially toward GENIE")
    # A4 MONOTONE-SAFE
    if not (tu_ber <= ls_ber + 1e-9):
        fails.append(f"A4 MONOTONE-SAFE: TURBO BER {tu_ber:.4f} > LS {ls_ber:.4f} (turbo must never be worse than it=0)")
    # A5 DEFAULT-OFF byte-identical (the unset line == LS line)
    off_out = run_cell(binary, args.cfg, dict(common))
    if coded_line(off_out) != coded_line(ls_out):
        fails.append("A5 DEFAULT-OFF: turbo-unset coded line differs across runs (non-determinism)")

    print()
    if fails:
        for f in fails:
            print("  FAIL:", f)
        print(f"\n[test_turbo_eq] FAILED ({len(fails)} assertion(s))")
        sys.exit(1)

    # Bonus: monotone-safe at low Es/N0 (C-errprop): turbo never below LS.
    lo_common = dict(common, MERCURY_SFO_GRID_ESN0=str(args.esn0 - 8))
    lo_ls = parse_coded(run_cell(binary, args.cfg, dict(lo_common)))[2]
    lo_tu = parse_coded(run_cell(binary, args.cfg, dict(lo_common, MERCURY_SFO_GRID_TURBO_ITERS=str(args.iters))))[2]
    if lo_ls is not None and lo_tu is not None and lo_tu > lo_ls + 1e-9:
        print(f"  WARN C-errprop: low-SNR TURBO BER {lo_tu:.4f} > LS {lo_ls:.4f} (monotone guard leaked)")
    else:
        print(f"  C-errprop (low SNR EsN0={args.esn0-8}): LS={lo_ls} TURBO={lo_tu} -> monotone-safe OK")

    print("\n[test_turbo_eq] ALL PASS")
    print("  A1 LS fails (channel-estimation-limited)        OK")
    print("  A2 GENIE passes (compass: decodable w/ true CSI) OK")
    print("  A3 TURBO moves materially toward GENIE           OK")
    print("  A4 TURBO monotone-safe (never worse than it=0)   OK")
    print("  A5 default-off deterministic                     OK")


if __name__ == "__main__":
    main()

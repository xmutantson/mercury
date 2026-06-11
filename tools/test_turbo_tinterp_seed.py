#!/usr/bin/env python3
"""
Turbo-EQ TINTERP-SEED regression test (TURBO_EQ_VERDICT.md §5 recommended stack;
sibling of tools/test_turbo_eq.py which covers the DET-FLOOR cell).

THE FADED-FRONT EXPERIMENT (TURBO_EQ_VERDICT.md §1/§5): on the POOR/1 Hz Watterson
fade (depth6, fd=1.0 — the Dy=3 pilot-Nyquist wall), the PLAIN-LS-seeded turbo loop
cold-starts past the LDPC waterfall (post_FEC_info_BER 0.12-0.26) and stays 0/K on
EVERY seed — the documented turbo COLD-START failure (the data-aided refiner needs an
it=0 decode under ~0.05 BER to mint reliable virtual pilots, and plain-LS does not
deliver it). GENIE (perfect CSI) decodes K/K → the channel IS decodable; Mercury is
channel-estimation-limited (the compass holds).

THE FIX (verdict §5, recommended stack): seed the it=0 INIT estimate with TINTERP
(the warm faded estimator, LS_channel_estimator_tinterp) instead of plain LS, AND keep
the TINTERP H as the low-confidence FLOOR inside data_aided_channel_estimator (items
1+2), with the nv anchored to the warm-seed nv so the fast-fade cross-pilot differential
does not blow nv up and revert the seed (item 4). Driven by ONE env:
MERCURY_SFO_GRID_TURBO_SEED=tinterp. Default-off (unset) is byte-identical to plain LS.

FAILING-FIRST (CLAUDE.md Principle 3 / Phase-4): on the BASE binary (feat/turbo-eq,
no TINTERP-seed wiring) MERCURY_SFO_GRID_TURBO_SEED is an UNKNOWN env, so TINTERP-seed
== LS-seed and the crossing assertion A3 FAILS. On the seed-swapped binary the warm
TINTERP it=0 seed breaks the Dy=3 wall and A3 PASSES.

HONEST NEGATIVE (verdict §1): the data-aided it>=1 refinement does NOT add on top of
the TINTERP it=0 seed on this POOR cell (the seed is already at its ceiling; only GENIE
reaches K/K). The win is the it=0 SEED-SWAP, captured by the published (monotone-best-of)
decode-fraction. The loop stays monotone-safe (A4); the test documents this honestly.

Decisive metric: AGGREGATE published decode-fraction over the seed set. LS-seeded turbo
floors at 0/K on POOR; TINTERP-seeded crosses materially (most seeds decode, all seeds'
BER drops under the LDPC waterfall ~0.05).

Assertions (over the SEEDS set, POOR Watterson fd=1.0):
  A1 FAILING-FIRST : LS-seeded published decode-fraction == 0  (LS floors on POOR)
  A2 COMPASS       : GENIE decodes K/K every seed (BER==0; channel decodable w/ true CSI)
  A3 CROSSING      : TINTERP-seeded aggregate decode-fraction MATERIALLY > LS-seeded
                     (>= half the codewords AND mean BER under the waterfall), proving
                     the warm seed breaks the Dy=3 wall
  A4 MONOTONE-SAFE : TINTERP-seeded BER <= LS-seeded BER on EVERY seed (never worse)
  A5 DEFAULT-OFF   : turbo-unset coded line is deterministic (byte-identical-default
                     gate; the cross-binary md5 proof lives in the verdict harness)

Usage:
    python tools/test_turbo_tinterp_seed.py
    MERCURY_BIN=/path/to/mercury.exe python tools/test_turbo_tinterp_seed.py
    python tools/test_turbo_tinterp_seed.py --esn0 20 --cfg 16 --iters 2
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
        os.path.join(os.path.dirname(here), "mercury.exe"),
        os.path.join(os.path.dirname(here), "mercury"),
        r"C:\Users\kamer\mercury_wt\turbo-tinterp-seed\mercury.exe",
        r"C:\Program Files\Mercury\mercury.exe",
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    sys.exit("ERROR: mercury binary not found. Set MERCURY_BIN=<path to mercury.exe>.")


_BER_RE = re.compile(r"post_FEC_info_BER=([0-9.eE+-]+)")
_CW_RE = re.compile(r"codewords_decoded=(\d+)/(\d+)")


def run_cell(binary, cfg, env_extra, timeout=180):
    env = dict(os.environ)
    env["MERCURY_SFO_GRID"] = "1"
    env["MERCURY_SFO_GRID_CODED"] = "1"
    env.update(env_extra)
    args = [binary, "-m", "PLOT_PASSBAND", "-s", str(cfg)]
    p = subprocess.run(args, env=env, capture_output=True, text=True, timeout=timeout)
    return p.stdout + p.stderr


def parse_coded(out):
    """Return (codewords_decoded, total, post_FEC_info_BER) from the FINAL
    [SFO-GRID-CODED] codewords_decoded line (the published monotone-best-of result)."""
    ber = ok = tot = None
    for line in out.splitlines():
        if "[SFO-GRID-CODED]" in line and "codewords_decoded=" in line:
            m = _CW_RE.search(line)
            b = _BER_RE.search(line)
            if m:
                ok, tot = int(m.group(1)), int(m.group(2))
            if b:
                ber = float(b.group(1))
    return ok, tot, ber


def coded_line(out):
    for line in out.splitlines():
        if "[SFO-GRID-CODED]" in line and "codewords_decoded=" in line:
            return line.strip()
    return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", type=int, default=16, help="config (15 or 16, 32-QAM=16)")
    ap.add_argument("--esn0", type=float, default=20.0)
    ap.add_argument("--iters", type=int, default=2, help="turbo iterations (saturates at 2)")
    ap.add_argument("--watt-depth", type=float, default=6.0)
    ap.add_argument("--watt-fd", type=float, default=1.0, help="Doppler Hz (1.0 = POOR, the Dy=3 wall)")
    ap.add_argument("--seeds", type=str, default="11111,22222,33333,44444,55555,12345")
    ap.add_argument("--waterfall", type=float, default=0.05, help="LDPC waterfall BER (under = decodable)")
    args = ap.parse_args()

    binary = find_binary()
    seeds = [s.strip() for s in args.seeds.split(",") if s.strip()]
    print(f"[test_turbo_tinterp_seed] binary = {binary}")
    print(f"[test_turbo_tinterp_seed] POOR cell: cfg={args.cfg} chan=3 (Watterson) "
          f"depth={args.watt_depth} fd={args.watt_fd} EsN0={args.esn0} iters={args.iters}")
    print(f"[test_turbo_tinterp_seed] seeds = {seeds}")
    print()

    def common(seed):
        return {
            "MERCURY_SFO_GRID_CHAN": "3",            # Watterson time-varying
            "MERCURY_SFO_GRID_ESN0": str(args.esn0),
            "MERCURY_SFO_GRID_SEED": str(seed),
            "MERCURY_SFO_GRID_WATT_DEPTH_DB": str(args.watt_depth),
            "MERCURY_SFO_GRID_WATT_FD_HZ": str(args.watt_fd),
        }

    ls_ok_sum = ls_tot_sum = 0
    ti_ok_sum = ti_tot_sum = 0
    ge_ok_sum = ge_tot_sum = 0
    ls_ber_sum = ti_ber_sum = 0.0
    per_seed_worse = []           # A4: TINTERP-seed worse than LS-seed on this seed?
    genie_fail = []               # A2: GENIE failed to decode K/K on this seed?
    parse_fail = []

    print(f"  {'seed':>8} | {'LS-seed turbo':>22} | {'TINTERP-seed turbo':>22} | {'GENIE':>12}")
    print(f"  {'-'*8} | {'-'*22} | {'-'*22} | {'-'*12}")
    for seed in seeds:
        c = common(seed)
        ls_ok, ls_tot, ls_ber = parse_coded(run_cell(binary, args.cfg, dict(c, MERCURY_SFO_GRID_TURBO_ITERS=str(args.iters))))
        ti_ok, ti_tot, ti_ber = parse_coded(run_cell(binary, args.cfg, dict(c, MERCURY_SFO_GRID_TURBO_SEED="tinterp", MERCURY_SFO_GRID_TURBO_ITERS=str(args.iters))))
        ge_ok, ge_tot, ge_ber = parse_coded(run_cell(binary, args.cfg, dict(c, MERCURY_SFO_GRID_GENIE="1")))

        if None in (ls_ok, ti_ok, ge_ok, ls_ber, ti_ber, ge_ber):
            parse_fail.append(seed)
            print(f"  {seed:>8} | PARSE FAIL")
            continue

        ls_ok_sum += ls_ok; ls_tot_sum += ls_tot; ls_ber_sum += ls_ber
        ti_ok_sum += ti_ok; ti_tot_sum += ti_tot; ti_ber_sum += ti_ber
        ge_ok_sum += ge_ok; ge_tot_sum += ge_tot

        if ti_ber > ls_ber + 1e-9:
            per_seed_worse.append((seed, ls_ber, ti_ber))
        if not (ge_ber == 0.0 and ge_ok == ge_tot):
            genie_fail.append((seed, ge_ok, ge_tot, ge_ber))

        print(f"  {seed:>8} | {f'{ls_ok}/{ls_tot} BER={ls_ber:.4f}':>22} | "
              f"{f'{ti_ok}/{ti_tot} BER={ti_ber:.4f}':>22} | {f'{ge_ok}/{ge_tot} BER={ge_ber:.4f}':>12}")

    if parse_fail:
        sys.exit(f"\nERROR: could not parse harness output for seeds {parse_fail}.")

    ls_mean_ber = ls_ber_sum / len(seeds)
    ti_mean_ber = ti_ber_sum / len(seeds)
    print()
    print(f"  AGGREGATE: LS-seed {ls_ok_sum}/{ls_tot_sum} decoded (mean BER {ls_mean_ber:.4f}) | "
          f"TINTERP-seed {ti_ok_sum}/{ti_tot_sum} decoded (mean BER {ti_mean_ber:.4f}) | "
          f"GENIE {ge_ok_sum}/{ge_tot_sum}")
    print()

    fails = []

    # A1 FAILING-FIRST: LS-seeded floors at 0 decoded on POOR.
    if not (ls_ok_sum == 0):
        fails.append(f"A1 FAILING-FIRST: LS-seed decoded {ls_ok_sum}/{ls_tot_sum} != 0 "
                     f"(LS must FLOOR on POOR/1 Hz — the documented cold-start wall)")

    # A2 COMPASS: GENIE decodes K/K every seed.
    if genie_fail:
        fails.append(f"A2 COMPASS: GENIE failed to decode K/K on {genie_fail} "
                     f"(channel must be decodable with perfect CSI — the est-limited compass)")

    # A3 CROSSING: TINTERP-seed materially > LS-seed: >= half the codewords decoded AND
    # mean BER under the LDPC waterfall (proves the warm seed breaks the Dy=3 wall).
    half = ti_tot_sum / 2.0
    if not (ti_ok_sum > ls_ok_sum and ti_ok_sum >= half and ti_mean_ber < args.waterfall):
        fails.append(
            f"A3 CROSSING: TINTERP-seed {ti_ok_sum}/{ti_tot_sum} (mean BER {ti_mean_ber:.4f}) did not "
            f"cross materially vs LS-seed {ls_ok_sum}/{ls_tot_sum} "
            f"(need decoded > LS AND >= {half:.0f} AND mean BER < waterfall {args.waterfall}); "
            f"the TINTERP it=0 seed did NOT break the Dy=3 Nyquist wall")

    # A4 MONOTONE-SAFE: TINTERP-seed never worse than LS-seed (per seed).
    if per_seed_worse:
        fails.append(f"A4 MONOTONE-SAFE: TINTERP-seed BER WORSE than LS-seed on {per_seed_worse} "
                     f"(the warm seed must never regress below plain LS)")

    # A5 DEFAULT-OFF deterministic (the cross-binary byte-identical proof is in the verdict).
    c0 = common(seeds[0])
    off1 = coded_line(run_cell(binary, args.cfg, dict(c0)))
    off2 = coded_line(run_cell(binary, args.cfg, dict(c0)))
    if off1 != off2 or not off1:
        fails.append("A5 DEFAULT-OFF: turbo-unset coded line non-deterministic across runs")

    if fails:
        print()
        for f in fails:
            print("  FAIL:", f)
        print(f"\n[test_turbo_tinterp_seed] FAILED ({len(fails)} assertion(s))")
        sys.exit(1)

    print("[test_turbo_tinterp_seed] ALL PASS")
    print(f"  A1 LS-seed floors 0/{ls_tot_sum} on POOR (cold-start wall)        OK")
    print(f"  A2 GENIE decodes {ge_ok_sum}/{ge_tot_sum} (compass: est-limited)             OK")
    print(f"  A3 TINTERP-seed crosses: {ti_ok_sum}/{ti_tot_sum}, mean BER {ti_mean_ber:.4f} < {args.waterfall}  OK")
    print( "  A4 TINTERP-seed monotone-safe (never worse than LS)        OK")
    print( "  A5 default-off deterministic                               OK")
    print()
    print("  HONEST NEGATIVE (verdict §1): the WIN is the it=0 SEED-SWAP; the data-aided")
    print("  it>=1 refinement does not add on top on POOR (only GENIE reaches K/K). The")
    print("  loop is monotone-safe; the published decode is the best-of (it=0 TINTERP).")


if __name__ == "__main__":
    main()

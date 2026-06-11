#!/usr/bin/env python3
"""
test_sim_airtime_wire.py — regression for GAP #1: the SIM harness must report an
AIRTIME-DERIVED wire rate that reconciles with the HW per-frame airtime budget.

THE GAP (bigblock_p3_hw/_cfg16hold2/CFG16HOLD2_VERDICT.md, MEMORY): the verdict
claimed the FTRT sim under-reports absolute wire "~7-10x" vs HW (HW CFG15 "10920
bps" vs sim "~1000-1600"). That comparison was apples-to-oranges on BOTH ends:
  * the sim's ~1000-1600 was a ROBUST->CFG15 climb-RAMP-DILUTED whole-run average
    (rx_bytes / WALL-clock over a window that spends ~100s climbing before the
    first CFG15 frame), and
  * the HW "10920" was an app-socket BURST window (5000 B drained in ~3.66s),
    NOT the per-frame channel wire rate.
The per-frame channel wire rate is the modem's own airtime model rbc (Tf =
Ts*(Nsymb+preamble); rbc = nData*log2M*LDPC_CR/Tf): CFG15 rbc = 3348.4 bps
(`mercury -l`). HW eff confirms it (CFG13 clean eff_bps 2292 ~= CFG13 rbc 2290).

THE FIX (this test gates it): the relay now accounts SIGNAL vs SILENCE chunks per
direction (airtime vs turnaround/idle) and the harness derives a `wire_bps_airtime`
= delivered_bytes*8 / (signal_chunks_in_delivering_direction * CHUNK_SAMPLES / FS).
That number is the per-frame CHANNEL wire rate and MUST reconcile with HW rbc.

This test PINS CFG15 (--no-gearshift) on a clean channel so there is NO climb ramp,
runs a short cell, and asserts:
  T1  the harness emits `wire_bps_airtime` AND `airtime_secs` (FAILS before the fix:
      the field does not exist).
  T2  wire_bps_airtime reconciles with HW CFG15 rbc=3348 within +-25% (the sim
      airtime model is HW-representative, NOT 7-10x under).
  T3  the relay emits a SIGNAL/SILENCE airtime breakdown (the GAP-#1 instrumentation).

Run:  python tools/sim/test_sim_airtime_wire.py [--bin PATH]
Exit 0 PASS, 1 FAIL. Light: one short pinned cell, auto-isolated port.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
# tools/sim -> tools -> mercury (matches sim_arq_channel.py's MERCURY_ROOT).
MERCURY_ROOT = os.path.dirname(os.path.dirname(HERE))
HARNESS = os.path.join(HERE, "sim_arq_channel.py")
# The harness requires a GUARD-1-marked -x sim binary. Default to the canonical
# build (CLAUDE.md: bash build.sh o3 -> C:\Program Files\Mercury\mercury.exe);
# fall back to a mercury.exe alongside the repo root. Override with --bin.
DEFAULT_BIN = os.environ.get(
    "MERCURY_BIN",
    r"C:\Program Files\Mercury\mercury.exe")

HW_CFG15_RBC = 3348.4     # mercury -l, CONFIG_15 airtime-derived wire (bps)
RECON_TOL = 0.25          # +-25% reconciliation band


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=DEFAULT_BIN)
    ap.add_argument("--port", type=int, default=53190)
    ap.add_argument("--secs", type=int, default=80)
    args = ap.parse_args()

    bin_path = args.bin
    if not os.path.isfile(bin_path):
        alt = os.path.join(MERCURY_ROOT, "mercury.exe")
        if os.path.isfile(alt):
            bin_path = alt
        else:
            print(f"SKIP: mercury binary not found: {args.bin} (and no {alt})")
            return 0

    jf = os.path.join(tempfile.gettempdir(), "test_sim_airtime_wire.json")
    if os.path.exists(jf):
        os.remove(jf)

    cmd = [sys.executable, HARNESS, "--bin", bin_path,
           "--snr", "40", "--profile", "wgn",
           "--secs", str(args.secs), "--start-cfg", "15", "--no-gearshift",
           "--port", str(args.port), "--json", jf]
    print("=== GAP#1 airtime-wire regression (pinned CFG15 clean) ===")
    print(" ".join(cmd) + "\n")
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=args.secs + 120)
    sys.stdout.write(r.stdout[-2000:])
    if r.returncode != 0:
        print(f"\nFAIL: harness exited {r.returncode}")
        print(r.stderr[-1500:])
        return 1

    if not os.path.exists(jf):
        print("\nFAIL: harness wrote no json")
        return 1
    d = json.load(open(jf))

    ok = True

    # T1: the airtime-derived wire fields must exist.
    has_fields = ("wire_bps_airtime" in d) and ("airtime_secs" in d)
    print(f"\n[T1] wire_bps_airtime + airtime_secs present : "
          f"{'OK' if has_fields else 'FAIL (field missing -> pre-fix)'}")
    if not has_fields:
        ok = False

    # T2: reconcile with HW rbc.
    if has_fields and d.get("wire_bps_airtime"):
        w = d["wire_bps_airtime"]
        ratio = w / HW_CFG15_RBC
        recon = (1.0 - RECON_TOL) <= ratio <= (1.0 + RECON_TOL)
        print(f"[T2] wire_bps_airtime={w:.1f}  HW rbc={HW_CFG15_RBC}  "
              f"ratio={ratio:.3f}  : {'OK' if recon else 'FAIL'}")
        if not recon:
            ok = False
        # also report the wall-clock number for contrast
        print(f"     (contrast: rx_bps wall-clock = {d.get('rx_bps')} bps, "
              f"airtime_secs={d.get('airtime_secs')}, wall_secs={d.get('wall_secs')})")
    else:
        print("[T2] SKIP (no wire_bps_airtime)")
        ok = False

    # T3: relay airtime breakdown present in the harness json.
    has_breakdown = ("airtime_signal_chunks" in d) and ("airtime_silence_chunks" in d)
    print(f"[T3] relay airtime breakdown (signal/silence chunks) : "
          f"{'OK' if has_breakdown else 'FAIL'}")
    if not has_breakdown:
        ok = False

    print("\n" + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

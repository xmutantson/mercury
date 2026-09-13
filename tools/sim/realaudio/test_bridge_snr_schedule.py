#!/usr/bin/env python3
"""Behavior + default-off compatibility gate for the --snr-schedule option of
realaudio_bridge_s32_c.

Both gates run through the deterministic file I/O modes, so no snd-aloop card is
needed:

  behavior : a step schedule (25 dB for the first half of the stream, 10 dB for
             the second) is applied to a 1500 Hz tone in --double mode.  The
             realized SNR measured on each half of the impaired output, and the
             emitted snr_schedule_applied log, must both track the schedule.
             On a bridge that does not know --snr-schedule the flag is rejected
             and this gate fails.
  compat   : with no --snr-schedule the --vector output is byte-identical to a
             reference bridge (when BRIDGE_BIN_BASELINE points at one), and a
             constant schedule at the static SNR is inert (byte-identical to no
             schedule at all) -- i.e. the schedule machinery adds no RNG draws
             and does not disturb the sample path.

Environment overrides:
  BRIDGE_BIN           bridge under test        (default: ./realaudio_bridge_s32_c)
  BRIDGE_BIN_BASELINE  optional reference bridge for the byte-identity check
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BIN = Path(os.environ.get("BRIDGE_BIN", str(HERE / "realaudio_bridge_s32_c")))
BASELINE = os.environ.get("BRIDGE_BIN_BASELINE")

FS = 48000.0
PERIOD = 1024
INT_MAX = 2147483647.0
SIG_REF = 0.15
SNR_TOL_DB = 0.5           # realized-vs-scheduled tolerance per region
REALIZED_TOL_DB = 0.1      # applied-vs-realized consistency inside the log


def run_double(binary, x, *opts, statsfile=None):
    with tempfile.TemporaryDirectory(prefix="snr_sched_") as td:
        pi, po = Path(td) / "in.f64", Path(td) / "out.f64"
        np.asarray(x, dtype="<f8").tofile(pi)
        argv = [str(binary), "--double-in", str(pi), "--double-out", str(po), *opts]
        cp = subprocess.run(argv, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        if cp.returncode != 0:
            raise RuntimeError(f"{binary} exited {cp.returncode}: "
                               f"{cp.stderr.decode(errors='replace').strip()}")
        out = np.fromfile(po, dtype="<f8")
        stats = json.loads(Path(statsfile).read_text()) if statsfile else None
        return out, stats


def run_vector_md5(binary, stereo, *opts):
    with tempfile.TemporaryDirectory(prefix="snr_sched_") as td:
        pi, po = Path(td) / "in.s32", Path(td) / "out.s32"
        np.asarray(stereo, dtype="<i4").tofile(pi)
        argv = [str(binary), "--vector-in", str(pi), "--vector-out", str(po), *opts]
        subprocess.run(argv, check=True, stdout=subprocess.DEVNULL)
        return hashlib.md5(Path(po).read_bytes()).hexdigest()


def snr3k(sig, out):
    # Same inversion of Channel._noise_std the faithfulness gate uses:
    # var = P_sig * 24000 / (SNR * 3000)  ->  SNR = P_sig * 8 / var.
    noise = out - sig
    return 10.0 * math.log10(float(np.mean(sig * sig)) * 8.0 /
                             float(np.mean(noise * noise)))


def tone(seconds):
    n = int(round(seconds * FS))
    t = np.arange(n, dtype=np.float64) / FS
    return SIG_REF * math.sqrt(2.0) * np.sin(2.0 * math.pi * 1500.0 * t)


def behavior_gate(report):
    hi, lo, step = 25.0, 10.0, 2.0
    x = tone(4.0)
    schedule = ("# t_offset_s,snr3k_db\n"
                f"0.0,{hi:g}\n"
                f"{step:g},{hi:g}\n"
                f"{step + 1e-4:g},{lo:g}\n"
                f"6.0,{lo:g}\n")
    with tempfile.TemporaryDirectory(prefix="snr_sched_cfg_") as td:
        sf = Path(td) / "sched.txt"
        sf.write_text(schedule)
        stats_path = Path(td) / "stats.json"
        try:
            out, stats = run_double(
                BIN, x, "--profile", "wgn", "--snr", f"{hi:g}", "--sig-ref", f"{SIG_REF:g}",
                "--seed", "11", "--snr-schedule", str(sf), "--statsfile", str(stats_path),
                statsfile=stats_path)
        except (RuntimeError, FileNotFoundError, json.JSONDecodeError) as exc:
            report["behavior"] = {"pass": False, "error": str(exc),
                                  "note": "bridge rejected --snr-schedule or emitted no log"}
            return False

    def region(a, b):
        i0, i1 = int(a * FS), int(b * FS)
        return snr3k(x[i0:i1], out[i0:i1])

    realized_hi = region(0.3, 1.7)     # before the step
    realized_lo = region(2.3, 3.9)     # after the step

    applied = stats["snr_schedule_applied"]
    intent = stats["snr_schedule_intent"]
    origin = stats["snr_schedule_origin"]
    hi_events = [e for e in applied if e["t_offset_s"] < 1.9]
    lo_events = [e for e in applied if e["t_offset_s"] > 2.1]

    ok_output = abs(realized_hi - hi) <= SNR_TOL_DB and abs(realized_lo - lo) <= SNR_TOL_DB
    ok_applied = (bool(hi_events) and bool(lo_events)
                  and all(abs(e["applied_snr3k_db"] - hi) < 1e-6 for e in hi_events)
                  and all(abs(e["applied_snr3k_db"] - lo) < 1e-6 for e in lo_events))
    ok_realized = all(abs(e["realized_snr3k"] - e["applied_snr3k_db"]) <= REALIZED_TOL_DB
                      for e in applied)
    ok_first_last = (applied and abs(applied[0]["applied_snr3k_db"] - hi) < 1e-6
                     and abs(applied[-1]["applied_snr3k_db"] - lo) < 1e-6)
    ok_intent = len(intent) == 4
    ok_base = origin.get("time_base") == "sample_clock"
    ok = bool(ok_output and ok_applied and ok_realized and ok_first_last and ok_intent and ok_base)

    report["behavior"] = {
        "scheduled_hi_db": hi, "scheduled_lo_db": lo, "step_s": step,
        "realized_hi_db": realized_hi, "realized_lo_db": realized_lo,
        "applied_events": len(applied), "intent_rows": len(intent),
        "time_base": origin.get("time_base"),
        "output_tracks_schedule": bool(ok_output),
        "applied_log_tracks_schedule": bool(ok_applied),
        "realized_matches_applied": bool(ok_realized),
        "first_last_present": bool(ok_first_last),
        "pass": ok,
    }
    return ok


def compat_gate(report):
    rng = np.random.default_rng(0xB0A710)
    ch0 = rng.integers(-2 ** 30, 2 ** 30, 200000, dtype=np.int32)
    ch1 = rng.integers(-2 ** 31, 2 ** 31, ch0.size, dtype=np.int32)
    inp = np.column_stack((ch0, ch1)).astype("<i4").ravel()
    common = ("--profile", "wgn", "--snr", "18", "--sig-ref", f"{SIG_REF:g}", "--seed", "7")

    md5_off = run_vector_md5(BIN, inp, *common)

    md5_const = None
    with tempfile.TemporaryDirectory(prefix="snr_sched_const_") as td:
        cf = Path(td) / "const.txt"
        cf.write_text("0.0,18\n100.0,18\n")
        try:
            md5_const = run_vector_md5(BIN, inp, *common, "--snr-schedule", str(cf))
        except subprocess.CalledProcessError:
            md5_const = None
    constant_inert = md5_const is not None and md5_off == md5_const

    if BASELINE:
        md5_baseline = run_vector_md5(BASELINE, inp, *common)
        baseline_equal = md5_off == md5_baseline
    else:
        md5_baseline = None
        baseline_equal = None

    ok = constant_inert and (baseline_equal in (None, True))
    report["compat"] = {
        "md5_default_off": md5_off,
        "md5_constant_schedule": md5_const,
        "constant_schedule_inert": constant_inert,
        "baseline_binary": BASELINE,
        "md5_baseline": md5_baseline,
        "baseline_byte_identical": baseline_equal,
        "pass": bool(ok),
    }
    return ok


def main():
    if not BIN.exists():
        subprocess.run(["make", "-C", str(HERE), "realaudio_bridge_s32_c"], check=True)
    report = {"binary": str(BIN), "baseline": BASELINE}
    gates = {"behavior": behavior_gate(report), "compat": compat_gate(report)}
    report["gates"] = gates
    report["verdict"] = "PASS" if all(gates.values()) else "FAIL"

    out = HERE / "snr_schedule_results.json"
    out.write_text(json.dumps(report, indent=2) + "\n")
    b = report["behavior"]
    print(f"SNR-SCHEDULE {report['verdict']}")
    if "realized_hi_db" in b:
        print(f"BEHAVIOR realized hi/lo = {b['realized_hi_db']:.4f}/{b['realized_lo_db']:.4f} dB "
              f"(scheduled {b['scheduled_hi_db']:.1f}/{b['scheduled_lo_db']:.1f}), "
              f"events={b['applied_events']} time_base={b['time_base']} "
              f"out_tracks={b['output_tracks_schedule']} log_tracks={b['applied_log_tracks_schedule']} "
              f"realized_ok={b['realized_matches_applied']}")
    else:
        print(f"BEHAVIOR gate failed before any schedule was applied: {b.get('error', '?')} "
              f"({b.get('note', '')})")
    c = report["compat"]
    print(f"COMPAT default_off_md5={c['md5_default_off']} const_md5={c['md5_constant_schedule']} "
          f"constant_inert={c['constant_schedule_inert']} "
          f"baseline_md5={c['md5_baseline']} baseline_equal={c['baseline_byte_identical']}")
    print(f"report={out}")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

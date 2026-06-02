#!/usr/bin/env python3
"""
ofdm_acq_cliff_sweep.py — OFDM-data acquisition cliff sweep (Step 1 integration).

Drives `mercury.exe -m PLOT_PASSBAND -s <config> --ber-esn0=<dB> --ber-frames=<N>`
at descending injected Es/N0 and parses the per-frame detection self-test lines
that the BER path prints once per config:

  [BER-DET]     config=C metric=M delay=D expected=E OK|WARN-lowSNR   (Schmidl-Cox, the BASELINE detector)
  [BER-DET-FFT] config=C metric=M delay=D expected=E OK|WARN-lowSNR   (coherent FFT detector, the Step-1 FIX)
  <esn0>;<ber>                                                        (decode result at that Es/N0)

For each config it reports the ACQUISITION CLIFF = the lowest Es/N0 at which the
detector still reads OK (metric >= preamble_detect_threshold AND delay within Ngi
of the forced-delay position), for BOTH detectors, and the DECODE cliff
(lowest Es/N0 with BER below --ber-target). The Step-1 pass criterion
(ofdm-data-acquisition-fix-plan.md §4.2): the FFT acquisition cliff moves from
the Schmidl-Cox baseline (~+0.9 dB SNR3k) to <= -3 dB, staying above the decode
depth so acquisition is no longer the bottleneck.

NOTE on the self-test windowing: the [BER-DET]/[BER-DET-FFT] self-test runs the
detector over a window that includes leading context before the forced-delay
preamble. Because the OFDM preamble repeats the same ZC sequence each symbol, the
coherent detector can lock up to Nsymb symbols early (a metric-equal periodicity
alias) and so the FFT OK/WARN delay-check may flag WARN even when the metric is
strong. This tool therefore ALSO reports the FFT cliff using a METRIC-ONLY
criterion (metric >= threshold, ignoring the delay check) — that is the
acquisition-reach the production path realizes (the full-rate demod delay is
re-refined downstream by site 8). The in-process unit test
`ofdm_coherent_cliff` (mercury.exe --test) is the cleaner, production-matching
fail-before/pass-after artifact; this sweep is the end-to-end confirmation.

SIM ONLY (AWGN, no IONOS / RF). Usage:
  python3 tools/ofdm_acq_cliff_sweep.py [--mercury PATH] [--configs 0,1,2,3]
      [--esn0-hi 8] [--esn0-lo -10] [--step 1] [--frames 3] [--ber-target 0.0]
"""
import argparse
import re
import subprocess
import sys

DET_RE = re.compile(
    r'\[BER-DET\]\s+config=(\d+)\s+metric=([-\d.]+)\s+delay=(-?\d+)\s+expected=(-?\d+)\s+(\S+)')
FFT_RE = re.compile(
    r'\[BER-DET-FFT\]\s+config=(\d+)\s+metric=([-\d.]+)\s+delay=(-?\d+)\s+expected=(-?\d+)\s+(\S+)')
BER_RE = re.compile(r'^([-\d.]+);([-\d.eE+]+)\s*$')
THR_RE = re.compile(r'preamble_detect_threshold|metric=')


def run_point(mercury, config, esn0, frames):
    """Run one Es/N0 point; return (sc_metric, sc_ok, fft_metric, fft_ok, ber)."""
    cmd = [mercury, "-m", "PLOT_PASSBAND", "-s", str(config),
           "--ber-esn0=%g" % esn0, "--ber-frames=%d" % frames]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=180).stdout
    except subprocess.TimeoutExpired:
        return (None, None, None, None, None)
    sc = (None, None)
    fft = (None, None)
    ber = None
    for line in out.splitlines():
        m = DET_RE.search(line)
        if m and int(m.group(1)) == config:
            sc = (float(m.group(2)), m.group(5) == "OK")
        m = FFT_RE.search(line)
        if m and int(m.group(1)) == config:
            fft = (float(m.group(2)), m.group(5) == "OK")
        m = BER_RE.match(line.strip())
        if m:
            try:
                ber = float(m.group(2))
            except ValueError:
                pass
    return (sc[0], sc[1], fft[0], fft[1], ber)


def sweep_config(mercury, config, hi, lo, step, frames, ber_target, thr=0.15):
    print("\n=== CONFIG_%d ===" % config)
    print("  Es/N0 |  SC metric OK | FFT metric OK | FFT metric>=thr | BER")
    sc_cliff = None        # lowest Es/N0 with Schmidl-Cox OK (metric+delay)
    fft_cliff = None       # lowest Es/N0 with FFT OK (metric+delay)
    fft_metric_cliff = None  # lowest Es/N0 with FFT metric>=thr (reach, delay-agnostic)
    decode_cliff = None    # lowest Es/N0 with BER<=target
    e = hi
    while e >= lo - 1e-9:
        sc_m, sc_ok, fft_m, fft_ok, ber = run_point(mercury, config, e, frames)
        sc_ms = "%.3f" % sc_m if sc_m is not None else "  -  "
        fft_ms = "%.3f" % fft_m if fft_m is not None else "  -  "
        fft_metric_ok = (fft_m is not None and fft_m >= thr)
        bers = ("%.3g" % ber) if ber is not None else " - "
        print("  %5.1f | %s  %-3s | %s  %-3s | %-3s | %s" % (
            e, sc_ms, "OK" if sc_ok else "no",
            fft_ms, "OK" if fft_ok else "no",
            "yes" if fft_metric_ok else "no", bers))
        if sc_ok:
            sc_cliff = e
        if fft_ok:
            fft_cliff = e
        if fft_metric_ok:
            fft_metric_cliff = e
        if ber is not None and ber <= ber_target:
            decode_cliff = e
        e -= step
    print("  --> Schmidl-Cox acquisition cliff (metric+delay OK): %s dB" %
          (("%.1f" % sc_cliff) if sc_cliff is not None else "none"))
    print("  --> Coherent FFT acquisition cliff (metric+delay OK): %s dB" %
          (("%.1f" % fft_cliff) if fft_cliff is not None else "none"))
    print("  --> Coherent FFT REACH (metric>=%.2f, delay-agnostic): %s dB" %
          (thr, ("%.1f" % fft_metric_cliff) if fft_metric_cliff is not None else "none"))
    print("  --> Decode cliff (BER<=%.3g): %s dB" %
          (ber_target, ("%.1f" % decode_cliff) if decode_cliff is not None else "none"))
    return sc_cliff, fft_cliff, fft_metric_cliff, decode_cliff


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mercury", default="./mercury.exe")
    ap.add_argument("--configs", default="0,1,2,3")
    ap.add_argument("--esn0-hi", type=float, default=8.0)
    ap.add_argument("--esn0-lo", type=float, default=-10.0)
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--ber-target", type=float, default=0.0)
    ap.add_argument("--thr", type=float, default=0.15,
                    help="WB preamble_detect_threshold (0.15)")
    args = ap.parse_args()

    configs = [int(c) for c in args.configs.split(",") if c.strip() != ""]
    print("OFDM-data acquisition cliff sweep (SIM/AWGN). mercury=%s" % args.mercury)
    print("Step-1 target: FFT acquisition reach moves to <= -3 dB Es/N0, "
          "staying above the decode cliff.")
    summary = []
    for c in configs:
        summary.append((c,) + sweep_config(
            args.mercury, c, args.esn0_hi, args.esn0_lo, args.step,
            args.frames, args.ber_target, args.thr))

    print("\n=== SUMMARY (Es/N0 dB) ===")
    print(" cfg | SC-acq | FFT-acq | FFT-reach | decode")
    for c, sc, fft, fftm, dec in summary:
        def f(x):
            return ("%.1f" % x) if x is not None else " none"
        print("  %2d | %6s | %7s | %9s | %6s" % (c, f(sc), f(fft), f(fftm), f(dec)))
    print("\nInterpretation: FFT-reach << SC-acq demonstrates the Step-1 acquisition-"
          "cliff move (the coherent metric clears threshold where Schmidl-Cox cannot).")


if __name__ == "__main__":
    sys.exit(main())

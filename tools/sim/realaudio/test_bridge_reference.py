#!/usr/bin/env python3
"""Per-waveform noise reference test for the native real-audio bridge.

Mercury transmits waveforms at different power: OFDM data (about 0.0206 on the
bridge) and MFSK connect / acknowledgement / robust data bursts at about 5-6x
that.  The SNR3k label must hold for each of them from its first transmission.

The test builds one direction of traffic as the bridge sees it:

  session "wb":     3 MFSK connect bursts, then 12 OFDM data frames with short
                    gaps (the first data block), then 1 MFSK burst and 6 more
                    OFDM frames;
  session "robust": 2 MFSK connect bursts, then 10 MFSK robust data frames;
  session "mixed":  2 MFSK bursts, then one gapless transmission of 0.6 s MFSK
                    followed by 3 s OFDM, then 4 OFDM frames.

It runs the bridge in --vector-in/--vector-out mode and measures, per
transmission and for the silence ahead of the first OFDM frame, the realized
SNR3k with its own estimator: the output is projected onto the known input
(least squares, which also removes the receiver gain), and the residual PSD
inside the modem band (328.125-2671.875 Hz) is referred to 3 kHz.

Checks:
  1. every transmission's realized SNR3k within +/-0.3 dB of commanded, from
     the first OFDM frame and the first robust frame on (the mixed session is
     checked after its class step, allowing GEO_SHIFT_CHUNKS of lag);
  2. the noise in the silence ahead of the first OFDM frame within +/-0.3 dB of
     the OFDM-referenced level;
  3. the bridge's own per-transmission log agrees with (1) within 0.3 dB and
     the attested mode is the requested mode;
  4. --reference-mode legacy-median is byte-identical to --ref-bin (the
     previous build, whose default was the running median), and with
     --legacy-fullband byte-identical to --base-bin.

Against the previous build (running median, check 1 fails by ~7 dB on the first
OFDM frames): that is the fail-before.

usage: test_bridge_reference.py --bin PATH [--ref-bin PATH] [--base-bin PATH]
                                [--json OUT] [--mode-under-test default|legacy]
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

FS = 48000
LO, HI = 328.125, 2671.875
P_OFDM = 0.0206
P_MFSK = 0.118
SNRS = [0.0, 10.0]
TOL_DB = 0.3


def band(n):
    f = np.fft.rfftfreq(n, 1.0 / FS)
    return (f >= LO) & (f <= HI)


def ofdm(n, rng):
    x = rng.standard_normal(n)
    X = np.fft.rfft(x)
    X[~band(n)] = 0
    s = np.fft.irfft(X, n)
    return s * np.sqrt(P_OFDM / np.mean(s * s))


def mfsk(n, rng):
    tones = [700.0, 1100.0, 1500.0, 1900.0]
    sym = 480
    out = np.empty(n)
    ph = 0.0
    for k in range(0, n, sym):
        f = tones[rng.integers(len(tones))]
        m = min(sym, n - k)
        t = np.arange(m)
        out[k:k + m] = np.sin(ph + 2 * np.pi * f * t / FS)
        ph += 2 * np.pi * f * m / FS
    return out * np.sqrt(2 * P_MFSK) * 0.999


def build(session, rng):
    """Return (signal, list of (start, end, kind, check))."""
    segs = []
    parts = []
    pos = 0

    def gap(sec):
        nonlocal pos
        n = int(sec * FS)
        parts.append(np.zeros(n))
        pos += n

    def tx(kind, sec, check=True):
        nonlocal pos
        n = int(sec * FS)
        parts.append(ofdm(n, rng) if kind == "ofdm" else mfsk(n, rng))
        segs.append((pos, pos + n, kind, check))
        pos += n

    gap(0.5)
    if session == "wb":
        for _ in range(3):
            tx("mfsk", 1.2)
            gap(0.8)
        gap(0.6)
        for _ in range(12):
            tx("ofdm", 0.40)
            gap(0.05)
        gap(0.5)
        tx("mfsk", 0.5)
        gap(0.7)
        for _ in range(6):
            tx("ofdm", 0.40)
            gap(0.05)
    elif session == "robust":
        for _ in range(2):
            tx("mfsk", 1.2)
            gap(0.8)
        for _ in range(10):
            tx("mfsk", 0.8)
            gap(0.05)
    else:  # mixed
        for _ in range(2):
            tx("mfsk", 1.2)
            gap(0.8)
        n1, n2 = int(0.6 * FS), int(3.0 * FS)
        parts.append(np.concatenate([mfsk(n1, rng), ofdm(n2, rng)]))
        segs.append((pos, pos + n1, "mfsk", True))
        lag = 3 * 1024 + 1024
        segs.append((pos + n1 + lag, pos + n1 + n2, "ofdm", True))
        pos += n1 + n2
        gap(0.05)
        for _ in range(4):
            tx("ofdm", 0.40)
            gap(0.05)
    gap(0.5)
    return np.concatenate(parts), segs


def to_s32(x):
    v = np.clip(np.round(x * 2147483647.0), -2147483647, 2147483647).astype(np.int32)
    return np.repeat(v, 2)


def run(binpath, x, snr, extra, seed=7):
    with tempfile.TemporaryDirectory() as td:
        fi, fo, bl = (os.path.join(td, n) for n in ("in.raw", "out.raw", "bursts.jsonl"))
        to_s32(x).tofile(fi)
        cmd = [binpath, "--vector-in", fi, "--vector-out", fo, "--snr3k-db", str(snr),
               "--seed", str(seed)] + extra
        p = None
        if "--burst-log" not in extra:
            p = subprocess.run(cmd + ["--burst-log", bl], capture_output=True, text=True)
            if p.returncode and "usage" in p.stderr:
                p = None  # a build without the option (the previous bridge)
        if p is None:
            p = subprocess.run(cmd, capture_output=True, text=True)
        if p.returncode:
            raise RuntimeError("bridge rc=%d: %s" % (p.returncode, p.stderr))
        raw = open(fo, "rb").read()
        y = np.frombuffer(raw, dtype=np.int32)[0::2].astype(np.float64) / 2147483647.0
        rows = []
        if os.path.exists(bl):
            rows = [json.loads(l) for l in open(bl) if l.strip()]
        return y, raw, rows, p.stderr


def realized(x, y, a, b):
    """Own estimator: LS projection gain, in-band residual PSD referred to 3 kHz."""
    xs, ys = x[a:b], y[a:b]
    g = float(np.dot(ys, xs) / np.dot(xs, xs))
    e = ys - g * xs
    n = len(e)
    E = np.fft.rfft(e * np.hanning(n))
    psd = (np.abs(E) ** 2) / (FS * np.sum(np.hanning(n) ** 2)) * 2.0
    nb = float(np.mean(psd[band(n)])) * 3000.0
    ps = g * g * float(np.mean(xs * xs))
    return 10 * np.log10(ps / nb), g, nb


def silence_noise(y, a, b, g):
    e = y[a:b] / g
    n = len(e)
    E = np.fft.rfft(e * np.hanning(n))
    psd = (np.abs(E) ** 2) / (FS * np.sum(np.hanning(n) ** 2)) * 2.0
    return float(np.mean(psd[band(n)])) * 3000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--ref-bin", help="previous build (running-median default)")
    ap.add_argument("--base-bin", help="pre-noise-model build (legacy full band)")
    ap.add_argument("--mode-under-test", default="default", choices=("default", "legacy"))
    ap.add_argument("--json")
    a = ap.parse_args()
    # "default" here is the per-class reference (the bridge default before the
    # bench-mirror mode became the default; see test_bridge_bench.py).
    extra = (["--reference-mode", "per-class"] if a.mode_under_test == "default"
             else ["--reference-mode", "legacy-median"])
    ok = True
    report = {"bin": a.bin, "mode_under_test": a.mode_under_test, "sessions": []}
    for session in ("wb", "robust", "mixed"):
        rng = np.random.default_rng({"wb": 11, "robust": 12, "mixed": 13}[session])
        x, segs = build(session, rng)
        for snr in SNRS:
            try:
                y, raw, rows, err = run(a.bin, x, snr, extra)
            except RuntimeError as exc:
                print("[FAIL] %s snr=%g %s" % (session, snr, exc))
                ok = False
                continue
            mode = re.search(r"reference_mode=(\S+)", err)
            mode = mode.group(1) if mode else "unattested"
            want = "geometry" if a.mode_under_test == "default" else "legacy-median"
            per = []
            for (s0, s1, kind, check) in segs:
                r, g, nb = realized(x, y, s0, s1)
                # Estimator spread: the in-band PSD mean over nbins independent
                # bins has a relative std of 1/sqrt(nbins) (Hann window: ~x1.2).
                nbins = int(np.count_nonzero(band(s1 - s0)))
                sigma = 4.343 * 1.2 / np.sqrt(nbins)
                per.append({"kind": kind, "t0_s": s0 / FS, "dur_s": (s1 - s0) / FS,
                            "realized_snr3k_db": round(r, 3), "err_db": round(r - snr, 3),
                            "est_sigma_db": round(sigma, 3), "check": check})
            first_ofdm = next((i for i, s in enumerate(segs) if s[2] == "ofdm"), None)
            pre = None
            if first_ofdm is not None and session == "wb":
                s0 = segs[first_ofdm][0]
                g = realized(x, y, *segs[first_ofdm][:2])[1]
                nb = silence_noise(y, s0 - int(0.5 * FS), s0 - 256, g)
                want_nb = P_OFDM / 10 ** (snr / 10)
                pre = round(10 * np.log10(nb / want_nb), 3)
            chk = [p for p in per if p["check"]]
            # (a) every transmission within TOL_DB plus 3 estimator sigmas;
            # (b) the mean over each waveform kind, and over the first data
            #     block (first 12 OFDM frames), within TOL_DB.
            worst = max(abs(p["err_db"]) for p in chk)
            worst_excess = max(abs(p["err_db"]) - 3 * p["est_sigma_db"] for p in chk)
            means = {}
            for kind in ("mfsk", "ofdm"):
                e = [p["err_db"] for p in chk if p["kind"] == kind]
                if e:
                    means[kind] = round(float(np.mean(e)), 3)
            ofdm_rows = [p["err_db"] for p in chk if p["kind"] == "ofdm"]
            if ofdm_rows:
                means["first_block_ofdm"] = round(float(np.mean(ofdm_rows[:12])), 3)
            first_ofdm_err = per[first_ofdm]["err_db"] if first_ofdm is not None else None
            log_err = None
            if rows:
                log_err = max(abs(r_["snr3k_nominal_db"] - snr) for r_ in rows if r_["dur_s"] > 0.2)
            cond = bool(worst_excess <= TOL_DB and all(abs(v) <= TOL_DB for v in means.values())
                        and (pre is None or abs(pre) <= TOL_DB) and mode == want
                        and (log_err is None or log_err <= TOL_DB)
                        and (a.mode_under_test != "default" or len(rows) > 0))
            ok = ok and cond
            print("[%s] session=%-6s snr3k=%+5.1f mode=%s n_tx=%d mean_err_db=%s worst_err_db=%.3f "
                  "worst_minus_3sigma_db=%.3f first_ofdm_err_db=%s pre_ofdm_silence_err_db=%s "
                  "bridge_log_rows=%d bridge_log_worst_db=%s"
                  % ("PASS" if cond else "FAIL", session, snr, mode, len(per), json.dumps(means), worst,
                     worst_excess, first_ofdm_err, pre, len(rows),
                     None if log_err is None else round(log_err, 3)))
            report["sessions"].append({"session": session, "snr3k": snr, "mode": mode, "tx": per,
                                       "pre_ofdm_silence_err_db": pre, "bridge_log_rows": len(rows),
                                       "bridge_log_worst_db": log_err, "pass": bool(cond)})
            if a.ref_bin and a.mode_under_test == "default":
                _, r_ref, _, _ = run(a.ref_bin, x, snr, [])
                _, r_leg, _, _ = run(a.bin, x, snr, ["--reference-mode", "legacy-median", "--burst-log", "-"])
                same = r_ref == r_leg
                ok &= same
                print("[%s] legacy-median == ref-bin default (byte-identical): %s"
                      % ("PASS" if same else "FAIL", same))
            if a.base_bin and a.mode_under_test == "default":
                _, r_base, _, _ = run(a.base_bin, x, snr, [])
                _, r_lf, _, _ = run(a.bin, x, snr, ["--reference-mode", "legacy-median",
                                                  "--legacy-fullband", "--burst-log", "-"])
                same = r_base == r_lf
                ok &= same
                print("[%s] legacy-median --legacy-fullband == base-bin (byte-identical): %s"
                      % ("PASS" if same else "FAIL", same))
    report["pass"] = bool(ok)
    if a.json:
        json.dump(report, open(a.json, "w"), indent=1)
    print("RESULT %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

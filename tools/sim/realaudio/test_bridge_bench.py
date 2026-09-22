#!/usr/bin/env python3
"""Bench-mirror noise model test for the native real-audio bridge.

The IONOS channel simulator adds one fixed noise level per S:N setting and never
measures its input, so a waveform that is louder than the reference level
really gets more SNR.  The bridge default (--reference-mode bench) mirrors
that: one noise level, set from the Mercury OFDM WB data power (--ofdm-ref),
fixed from the first sample and the same on both directions.  "--snr3k N" is
the SNR3k the OFDM WB data realizes; any other waveform realizes
N + 10*log10(P_waveform / ofdm_ref).

One direction of traffic is built as the bridge sees it: 0.5 s of silence,
three MFSK connect bursts (narrow control tone set), twelve OFDM WB frames (a
first data block), an MFSK acknowledgement-like burst, six NB OFDM frames, six
one-tone and four two-tone MFSK robust data frames (wide tone set).

Checks (own estimator: least-squares projection of the output onto the known
input, residual in-band PSD referred to 3 kHz):
  1. every transmission realizes commanded + 10*log10(P_tx / ofdm_ref) within
     +/-0.3 dB plus 3 estimator sigmas; per-kind means within +/-0.3 dB;
  2. the noise in the leading silence (from sample 0) is at the OFDM-referenced
     level within +/-0.3 dB;
  3. bench output is byte-identical to --reference-mode fix --psig-fix ofdm_ref
     (the noise never moves);
  4. the bridge's per-transmission record: realized minus commanded equals its
     level offset (within 0.01 dB), agrees with (1) within 0.3 dB, and labels
     every transmission of 0.2 s or more with its true waveform kind;
  5. a dry run attests bench mode and equal forward / reverse noise;
  6. with --ref-bin (the previous per-class default build): per-class,
     legacy-median and fix modes are byte-identical to that build.
Against --ref-bin run in its own default (per-class) mode, check 1 fails by
the MFSK level offset (about +7.6 dB): that build did not mirror the bench.

usage: test_bridge_bench.py --bin PATH [--ref-bin PATH] [--json OUT]
       test_bridge_bench.py --bin PATH --expect-fail   (fail-before run)
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

from test_bridge_reference import FS, band, realized, silence_noise, run, to_s32

P_OFDM = 0.0206          # synthetic OFDM WB data power (the bridge default reference)
P_NB = 0.0221            # NB OFDM: gain 2.317 over one fifth of the carriers
P_MFSK_CONNECT = 0.127
P_MFSK_DATA = 0.118      # one-tone robust data (config 100)
P_MFSK_2S = 0.0674       # two-tone robust data (configs 101-103)
SNRS = [0.0, 10.0]
TOL_DB = 0.3


def ofdm_band(n, rng, lo, hi, power):
    x = rng.standard_normal(n)
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1.0 / FS)
    X[~((f >= lo) & (f <= hi))] = 0
    s = np.fft.irfft(X, n)
    s *= np.sqrt(power / np.mean(s * s))
    # Mercury clips OFDM data at 10 dB PAPR.
    cap = np.sqrt(power * 10.0)
    return np.clip(s, -cap, cap)


CTL_TONES = (1200.0, 1400.0, 1600.0, 1800.0)                 # narrow control set
# Wider robust data set: 8 tones over 800-2200 Hz, an equivalent width of about
# 1.5 kHz like Mercury's robust data MFSK on the bridge (1519-1595 Hz).
DATA_TONES = tuple(800.0 + 200.0 * k for k in range(8))


def mfsk(n, rng, power, tones=CTL_TONES, ntones=1):
    """Constant-envelope MFSK: ntones simultaneous equal tones per symbol."""
    sym = 480
    out = np.zeros(n)
    ph = np.zeros(ntones)
    for k in range(0, n, sym):
        # Distinct tones in one symbol (two equal tones on one frequency would
        # be a single tone at twice the amplitude, which MFSK never keys).
        fs = rng.choice(tones, size=ntones, replace=False)
        m = min(sym, n - k)
        t = np.arange(m)
        for j, f in enumerate(fs):
            out[k:k + m] += np.sin(ph[j] + 2 * np.pi * f * t / FS)
            ph[j] += 2 * np.pi * f * m / FS
    return out * np.sqrt(power / np.mean(out * out))


def build(rng):
    parts, segs, pos = [], [], 0

    def gap(sec):
        nonlocal pos
        n = int(sec * FS)
        parts.append(np.zeros(n))
        pos += n

    def tx(kind, sec, power):
        nonlocal pos
        n = int(sec * FS)
        if kind == "ofdm-wb":
            s = ofdm_band(n, rng, 328.125, 2671.875, power)
        elif kind == "ofdm-nb":
            s = ofdm_band(n, rng, 1265.625, 1734.375, power)
        elif kind == "mfsk-ctl":
            s = mfsk(n, rng, power)
        else:
            s = mfsk(n, rng, power, DATA_TONES, 2 if power < 0.09 else 1)
        parts.append(s)
        segs.append((pos, pos + n, kind, float(np.mean(s * s))))
        pos += n

    gap(0.5)
    for _ in range(3):
        tx("mfsk-ctl", 1.2, P_MFSK_CONNECT)
        gap(0.8)
    for _ in range(12):
        tx("ofdm-wb", 0.40, P_OFDM)
        gap(0.05)
    gap(0.5)
    tx("mfsk-ctl", 0.5, P_MFSK_CONNECT)
    gap(0.7)
    for _ in range(6):
        tx("ofdm-nb", 0.40, P_NB)
        gap(0.05)
    gap(0.5)
    for _ in range(6):
        tx("mfsk-data", 0.8, P_MFSK_DATA)
        gap(0.05)
    gap(0.5)
    for _ in range(4):
        tx("mfsk-data", 0.8, P_MFSK_2S)
        gap(0.05)
    gap(0.5)
    return np.concatenate(parts), segs


def dry_run(binpath, extra):
    with tempfile.TemporaryDirectory() as td:
        sf = os.path.join(td, "s.json")
        p = subprocess.run([binpath, "--dry-run", "--snr3k-db", "10", "--statsfile", sf] + extra,
                           capture_output=True, text=True)
        if p.returncode:
            return None
        return json.load(open(sf))


def evaluate(binpath, x, segs, snr, extra, ofdm_ref):
    y, raw, rows, err = run(binpath, x, snr, extra)
    mode = re.search(r"reference_mode=(\S+)", err)
    mode = mode.group(1) if mode else "unattested"
    per = []
    for (s0, s1, kind, p_tx) in segs:
        r, g, nb = realized(x, y, s0, s1)
        want = snr + 10 * np.log10(p_tx / ofdm_ref)
        # Estimator spread: (a) the in-band residual PSD mean over nbins bins
        # (Hann window: ~x1.2); (b) the least-squares gain, whose relative
        # amplitude error is sqrt(Pn / (Ps * N_eff)) with N_eff = 2 * B * T
        # independent samples in the modem band, doubled in power dB.
        nbins = int(np.count_nonzero(band(s1 - s0)))
        n_eff = 2.0 * (2671.875 - 328.125) * (s1 - s0) / FS
        sig_psd = 4.343 * 1.2 / np.sqrt(nbins)
        sig_gain = 2 * 4.343 * np.sqrt(10 ** (-want / 10) / n_eff)
        sigma = float(np.hypot(sig_psd, sig_gain))
        per.append({"kind": kind, "t0_s": round(s0 / FS, 4), "dur_s": (s1 - s0) / FS,
                    "p_tx": p_tx, "want_db": round(want, 3), "realized_db": round(r, 3),
                    "err_db": round(r - want, 3), "est_sigma_db": round(sigma, 3)})
    g0 = realized(x, y, *segs[0][:2])[1]
    nb0 = silence_noise(y, 256, int(0.5 * FS) - 256, g0)
    lead = round(10 * np.log10(nb0 / (ofdm_ref / 10 ** (snr / 10))), 3)
    return y, raw, rows, mode, per, lead


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--ref-bin", help="previous build (per-class default)")
    ap.add_argument("--expect-fail", action="store_true",
                    help="run the checks on --bin in its own default mode (fail-before)")
    ap.add_argument("--json")
    a = ap.parse_args()
    ofdm_ref = P_OFDM
    extra = [] if a.expect_fail else ["--ofdm-ref", repr(ofdm_ref)]
    ok = True
    report = {"bin": a.bin, "sessions": []}
    rng = np.random.default_rng(21)
    x, segs = build(rng)
    for snr in SNRS:
        y, raw, rows, mode, per, lead = evaluate(a.bin, x, segs, snr, extra, ofdm_ref)
        worst_excess = max(abs(p["err_db"]) - 3 * p["est_sigma_db"] for p in per)
        means = {}
        for kind in ("ofdm-wb", "ofdm-nb", "mfsk-ctl", "mfsk-data"):
            e = [p["err_db"] for p in per if p["kind"] == kind]
            means[kind] = round(float(np.mean(e)), 3)
        levels = {k: round(float(np.mean([p["want_db"] - snr for p in per if p["kind"] == k])), 3)
                  for k in means}
        c1 = worst_excess <= TOL_DB and all(abs(v) <= TOL_DB for v in means.values())
        c2 = abs(lead) <= TOL_DB
        c3 = None
        if not a.expect_fail:
            _, raw_fix, _, _ = run(a.bin, x, snr, ["--reference-mode", "fix", "--psig-fix", repr(ofdm_ref),
                                                    "--burst-log", "-"])
            c3 = raw_fix == raw
        c4 = None
        log_note = "no bridge log"
        long_rows = [r for r in rows if r.get("dur_s", 0) >= 0.2]
        if rows and "kind" in rows[0]:
            # Match each true transmission to the bridge's record by start time.
            matched, lab_ok, own_ok, lvl_ok = 0, 0, 0, 0
            for p in per:
                cand = [r for r in long_rows if abs(r["t0_s"] - p["t0_s"]) < 0.03]
                if not cand:
                    continue
                r = cand[0]
                matched += 1
                lab_ok += r["kind"] == p["kind"]
                own_ok += abs((r["snr3k_nominal_db"]) - p["realized_db"]) <= TOL_DB + 3 * p["est_sigma_db"]
                lvl_ok += abs((r["snr3k_nominal_db"] - r["commanded_snr3k_db"]) - r["level_vs_ofdm_ref_db"]) <= 0.01
            c4 = matched == len(per) and lab_ok == matched and own_ok == matched and lvl_ok == matched
            log_note = "matched=%d/%d kind_ok=%d own_agree=%d level_identity=%d" % (
                matched, len(per), lab_ok, own_ok, lvl_ok)
        cond = bool(c1 and c2 and (c3 is None or c3) and (c4 is None or c4)
                    and (a.expect_fail or mode == "bench") and (a.expect_fail or c4 is not None))
        ok = ok and cond
        print("[%s] snr3k=%+5.1f mode=%s n_tx=%d level_db=%s mean_err_db=%s worst_minus_3sigma_db=%.3f "
              "lead_silence_err_db=%s bench==fix(ref)=%s bridge_log(%s)"
              % ("PASS" if cond else "FAIL", snr, mode, len(per), json.dumps(levels), json.dumps(means),
                 worst_excess, lead, c3, log_note))
        report["sessions"].append({"snr3k": snr, "mode": mode, "tx": per, "lead_silence_err_db": lead,
                                   "bench_equals_fix": c3, "bridge_log": log_note,
                                   "bridge_rows": rows, "pass": bool(cond)})
        if a.ref_bin and not a.expect_fail:
            for m, e in (("per-class", ["--reference-mode", "per-class"]),
                         ("legacy-median", ["--reference-mode", "legacy-median"]),
                         ("fix", ["--reference-mode", "fix", "--psig-fix", "0.0206"])):
                e_ref = list(e)
                if m == "per-class":
                    e_ref = ["--reference-mode", "geometry"]
                _, r_ref, _, _ = run(a.ref_bin, x, snr, e_ref + ["--burst-log", "-"])
                _, r_new, _, _ = run(a.bin, x, snr, e + ["--burst-log", "-"])
                same = r_ref == r_new
                ok &= same
                print("[%s] %s == ref-bin %s (byte-identical): %s"
                      % ("PASS" if same else "FAIL", m, " ".join(e_ref), same))
    if not a.expect_fail:
        st = dry_run(a.bin, ["--ofdm-ref", repr(ofdm_ref)])
        b = (st or {}).get("bench", {})
        c5 = bool(st and st.get("psig_mode") == "bench" and b.get("fwd_noise_variance") == b.get("rev_noise_variance")
                  and abs(float(b.get("ofdm_reference_power", 0)) - ofdm_ref) < 1e-15
                  and b.get("ofdm_reference_source") == "cli")
        ok &= c5
        print("[%s] dry-run attestation: psig_mode=%s ofdm_ref=%s source=%s fwd_noise=%s rev_noise=%s"
              % ("PASS" if c5 else "FAIL", (st or {}).get("psig_mode"), b.get("ofdm_reference_power"),
                 b.get("ofdm_reference_source"), b.get("fwd_noise_variance"), b.get("rev_noise_variance")))
    report["pass"] = bool(ok)
    if a.json:
        json.dump(report, open(a.json, "w"), indent=1)
    print("RESULT %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

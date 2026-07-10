#!/usr/bin/env python3
"""
feat/decode-marathon LEVER H/I regression: SINGLE-PRECISION OFDM FFT
(env MERCURY_FFT_FLOAT, default off).

ofdm.cc transforms the OFDM symbols with PocketFFT in DOUBLE (Nfft=256, plus the
Nc-point preamble/SFO transforms). Lever H/I adds a CONTAINED float path: the
complex<double> OFDM/channel-estimator buffers are untouched; only the transform
kernel inside fft()/ifft() converts to complex<float>, runs a pocketfft_c<float>
plan (its own size-keyed cache, parallel to the double cache), and converts back.
DEFAULT OFF => the double path runs bit-for-bit.

This is a completionist throughput lever, not a hot-path fix: the FFT is ~75-130
us/frame, far below the LDPC decode. Single-precision NEON on the Pi5 A76 packs
4 floats vs 2 doubles per 128-bit reg (~2x FFT throughput); float FFT is the
HF-modem standard (VARA, codec2). The numerical impact of a 256-pt float FFT is
~1e-6 relative, well below the post-EQ EVM / LDPC LLR scale, so decode is
unaffected — TEST-2 asserts exactly that.

Drives the coded SFO-GRID harness — the same 27-cell fleet vehicle the sibling
levers (decode-marathon.md TEST-1, ldpc-decode-accel.md §1) use. Fail-before /
pass-after contract.

Checks
------
TEST-1  DEFAULT-OFF BYTE-IDENTICAL: with MERCURY_FFT_FLOAT unset, the 27-cell
        [SFO-GRID-CODED] render md5 is IDENTICAL to the base @151569e render
        (double path unchanged). Zero behaviour change off.

TEST-2  FLOAT-PATH DECODE INTACT: with MERCURY_FFT_FLOAT=1, across the 27 cells
        codewords_decoded is NOT regressed (float_ok >= double_ok per cell) and
        post_FEC_info_BER is comparable (float_ber <= double_ber + eps). The
        single-precision transform must not cost the decoder any codewords.

TEST-3  FLOAT PATH IS LIVE: with MERCURY_FFT_FLOAT=1 the render actually DIFFERS
        from the OFF render (the float kernel ran — the rounding shows in the nv
        / iter fields), proving the env gate is wired and not a no-op. (A pure
        guard against accidentally compiling the branch out.)

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

# Base @151569e (marathon levers D+C+E+G) 27-cell [SFO-GRID-CODED] render md5
# (NSYMB=600, -s 16, CODED). Pinned below from a clean @151569e build; this
# lever (H/I) adds the float path strictly default-off so the OFF render must
# match. Overridable via env for a re-pin on a later base.
BASE_MD5 = os.environ.get("DECODE_MARATHON_FFT_BASE_MD5",
                          "4575011ba38e23575ef3f3bbce0703b2")

_CODED_RE = re.compile(
    r"codewords_decoded=(\d+)/(\d+)\s+fail=\d+\s+post_FEC_info_BER=([0-9.eE+-]+)")


def find_binary():
    for c in (os.environ.get("MERCURY_BIN"),
              os.path.join(ROOT, "mercury.exe"),
              os.path.join(ROOT, "mercury")):
        if c and os.path.exists(c):
            return c
    sys.exit("ERROR: mercury binary not found. Set MERCURY_BIN=<path>.")


def run_cell(binary, chan, seed, esn0, fftfloat, timeout=300):
    env = dict(os.environ)
    env.update({
        "MERCURY_SFO_GRID": "1",
        "MERCURY_SFO_GRID_CODED": "1",
        "MERCURY_SFO_GRID_NSYMB": "600",
        "MERCURY_SFO_GRID_CHAN": str(chan),
        "MERCURY_SFO_GRID_SEED": str(seed),
        "MERCURY_SFO_GRID_ESN0": str(esn0),
    })
    if fftfloat:
        env["MERCURY_FFT_FLOAT"] = "1"
    else:
        env.pop("MERCURY_FFT_FLOAT", None)
    p = subprocess.run([binary, "-m", "PLOT_PASSBAND", "-s", "16"],
                       env=env, capture_output=True, text=True, timeout=timeout)
    return p.stdout + p.stderr


def coded_lines(out):
    return [ln for ln in out.splitlines() if "[SFO-GRID-CODED]" in ln]


def parse_cell(out):
    ok = tot = None
    ber = None
    for ln in out.splitlines():
        m = _CODED_RE.search(ln)
        if m:
            ok, tot, ber = int(m.group(1)), int(m.group(2)), float(m.group(3))
    return ok, tot, ber


def grid(binary, fftfloat):
    render = []
    cells = {}
    for chan in CHANS:
        for seed in SEEDS:
            for esn0 in ESN0S:
                out = run_cell(binary, chan, seed, esn0, fftfloat)
                render.extend(coded_lines(out))
                cells[(chan, seed, esn0)] = parse_cell(out)
    return "\n".join(render) + "\n", cells


def md5(s):
    return hashlib.md5(s.encode()).hexdigest()


def main():
    binary = find_binary()
    print(f"[test_decode_marathon_fftfloat] binary = {binary}")
    fails = []

    off_render, dbl = grid(binary, fftfloat=False)
    off_md5 = md5(off_render)
    print(f"[TEST-1] OFF render md5 = {off_md5}  (base = {BASE_MD5})")
    if BASE_MD5 == "__PIN_FROM_BASE__":
        print("[TEST-1] SKIP md5 compare (BASE_MD5 unpinned); pin this value "
              "from a clean base @151569e build, or pass "
              "DECODE_MARATHON_FFT_BASE_MD5=<md5>.")
    elif off_md5 == BASE_MD5:
        print("[TEST-1] PASS: default-off byte-identical to base @151569e")
    else:
        print("[TEST-1] FAIL: OFF render differs from base!")
        fails.append("TEST-1")

    flt_render, flt = grid(binary, fftfloat=True)

    # ---- TEST-2: float-path decode not regressed ---------------------------
    # HARD invariant: the single-precision FFT must not cost a single codeword
    # (float_ok >= double_ok, per cell and fleet-wide). SOFT: the post-FEC BER
    # of cells that already have residual uncorrected bits may move by the float
    # rounding of a 256-pt FFT — ~1e-6 relative in the transform, which surfaces
    # as a tiny LLR perturbation on FAILING codewords only. We allow that drift
    # (whichever is larger of BER_REL_TOL relative or BER_ABS_TOL absolute) but
    # NOT a codeword loss. Observed max drift on this fleet is ~3.5e-5 absolute.
    BER_REL_TOL = 0.05   # 5% relative — generous; the real drift is << this
    BER_ABS_TOL = 1e-3   # absolute floor for near-zero BERs
    fleet_dbl_ok = fleet_flt_ok = fleet_tot = 0
    cw_regressions = []
    ber_regressions = []
    max_ber_drift = 0.0
    for cell in dbl:
        dok, dtot, dber = dbl[cell]
        fok, ftot, fber = flt[cell]
        fleet_dbl_ok += dok
        fleet_flt_ok += fok
        fleet_tot += dtot
        if fok < dok:
            cw_regressions.append((cell, dok, fok))
        if fber is not None and dber is not None:
            drift = fber - dber
            if drift > max_ber_drift:
                max_ber_drift = drift
            tol = max(BER_ABS_TOL, BER_REL_TOL * dber)
            if drift > tol:
                ber_regressions.append((cell, dber, fber, drift))
    print(f"[TEST-2] fleet codewords double={fleet_dbl_ok}/{fleet_tot} "
          f"float={fleet_flt_ok}/{fleet_tot}  (max BER drift {max_ber_drift:.2e})")
    if not cw_regressions and not ber_regressions:
        print("[TEST-2] PASS: no codeword lost (float_ok>=double_ok) and BER "
              "drift within float-FFT rounding tolerance — single-precision FFT "
              "costs the decoder nothing")
    else:
        if cw_regressions:
            print(f"[TEST-2] FAIL: {len(cw_regressions)} cell(s) lost codewords: "
                  f"{cw_regressions}")
            fails.append("TEST-2")
        if ber_regressions:
            print(f"[TEST-2] FAIL: {len(ber_regressions)} cell(s) BER drifted "
                  f"beyond float tolerance: {ber_regressions}")
            fails.append("TEST-2")

    # ---- TEST-3: float path is actually live (env gate wired) --------------
    flt_md5 = md5(flt_render)
    print(f"[TEST-3] ON render md5 = {flt_md5}")
    if flt_md5 != off_md5:
        print("[TEST-3] PASS: ON render differs from OFF — the float kernel ran "
              "(env gate wired, not compiled out)")
    else:
        print("[TEST-3] FAIL: ON render identical to OFF — float path never "
              "executed (env gate dead?)")
        fails.append("TEST-3")

    print()
    if fails:
        print(f"[test_decode_marathon_fftfloat] FAILED: {fails}")
        sys.exit(1)
    print("[test_decode_marathon_fftfloat] ALL PASS")


if __name__ == "__main__":
    main()

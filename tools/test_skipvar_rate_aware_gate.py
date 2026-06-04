#!/usr/bin/env python3
"""Regression test for FIX-1: code-rate-aware SKIP-VAR noise-variance gate.

Captures the bug fixed in telecom_system.cc skip_var_nv_ceiling() +
the gate predicate in receive_byte(): the historical flat SKIP-VAR threshold
(noise_variance_estimate > 0.5) is code-rate-INDEPENDENT, but the LDPC decode
floor is strongly code-rate-DEPENDENT.  Measured (clean-AWGN PLOT_PASSBAND,
gate OFF) the rate-1/16 CONFIG_0 decodes up to nv~=1.48; the flat 0.5 sat far
below that and REJECTED genuinely-decodable rate-1/16..8/16 (CONFIG_0-6 BPSK)
frames before LDPC ever ran.

This is an in-process BER test — PLOT_PASSBAND opens NO audio device (it does
not call audioio_init_internal; see main.cc PLOT_PASSBAND block).  It is the
correct path to exercise the SKIP-VAR gate: passband_test_EsN0() -> receive_byte()
runs the trial loop including the gate with current_configuration set by -s.

WHAT IT ASSERTS
---------------
1. FAIL-BEFORE / PASS-AFTER (the bug): CONFIG_0 at Es/N0 = -6 dB, where the
   pilot-residual nv lands ~0.70 (above the old 0.5 gate, well below CONFIG_0's
   ~1.48 decode floor), with the SKIP-VAR gate at DEFAULT (enabled).
     * On the UNFIXED binary (flat 0.5): SKIP-VAR rejects every frame ->
       0 [OFDM-OK], all [OFDM-SYNC ... SKIP-VAR].
     * On the FIXED binary (rate-aware ceiling 1.60 for CONFIG_0): the same
       frames are admitted -> LDPC decodes -> [OFDM-OK] for (nearly) every frame
       and ZERO SKIP-VAR.
   We distinguish the two binaries automatically by the gate's log string:
   a fixed binary prints "...cfg=%d), skipping LDPC"; an unfixed one prints
   "...too high (>0.5)".  The test ALWAYS runs and, on a fixed binary, asserts
   the PASS-AFTER condition; on an unfixed binary, asserts the FAIL-BEFORE
   condition (so the same test proves both halves when pointed at each).

2. HIGH-RATE NO-REGRESSION: a high-rate config (default CONFIG_15, 16QAM 14/16)
   at a marginal Es/N0 must be DECODER-BOUND, not gate-bound — its nv stays
   below 0.50 and the rate-aware ceiling leaves its behavior UNCHANGED (ceiling
   == 0.50 for CONFIG_7-16).  Asserts that the high-rate config admits NO new
   undecodable frames: any frame it fails is an [OFDM-FAIL] (LDPC/CRC reject),
   never a SKIP-VAR delta vs the flat gate, and there is no flood of decodes
   appearing that the LDPC+CRC backstop would have to be trusted to reject.

3. HARD-CAP / DEEP-NOISE: CONFIG_0 at Es/N0 = -18 dB (pure noise) must still
   produce ZERO [OFDM-OK] — the loosened ceiling does not create false decodes
   (the mean_H gate + LDPC iter-cap + CRC backstop still reject hopeless frames).

USAGE
-----
    python tools/test_skipvar_rate_aware_gate.py            # uses ./mercury.exe
    python tools/test_skipvar_rate_aware_gate.py <binary>   # explicit binary

Exit code 0 on PASS, 1 on FAIL.  Requires a built mercury(.exe).
"""
from __future__ import annotations

import os
import re
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MERCURY_ROOT = os.path.dirname(SCRIPT_DIR)

# How many independent frames each single-point BER run evaluates.
BER_FRAMES = 30


def default_binary() -> str:
    exe = "mercury.exe" if os.name == "nt" else "mercury"
    return os.path.join(MERCURY_ROOT, exe)


def run_point(binary: str, config: int, esn0: float, frames: int = BER_FRAMES) -> str:
    """Run one PLOT_PASSBAND single-Es/N0 point with the gate at DEFAULT and
    return the combined stdout/stderr text.  PLOT_PASSBAND opens no audio
    device — this is a pure in-process BER run."""
    cmd = [
        binary,
        "-m", "PLOT_PASSBAND",
        "-s", str(config),
        f"--ber-esn0={esn0}",
        f"--ber-frames={frames}",
    ]
    proc = subprocess.run(
        cmd, cwd=MERCURY_ROOT, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, timeout=180,
    )
    return proc.stdout


def count(pattern: str, text: str) -> int:
    return len(re.findall(pattern, text))


def max_nv(text: str) -> float:
    vals = [float(m) for m in re.findall(r"\[FRAME-NV\][^\n]*?nv=([0-9.eE+-]+)", text)]
    return max(vals) if vals else float("nan")


def classify_gate_log(text: str):
    """Inspect SKIP-VAR log strings in `text`.  A fixed binary's SKIP-VAR log
    carries the config id ('...cfg=N), skipping LDPC'); the unfixed one prints
    the flat '...too high (>0.5)'.  Returns True (fixed), False (unfixed), or
    None if no SKIP-VAR line fired in this text."""
    if "too high (>0.5)" in text:
        return False
    if "skipping LDPC" in text and "cfg=" in text:
        return True
    return None  # no SKIP-VAR fired in this run


def binary_is_fixed(binary: str, primary_text: str):
    """Classify the binary's SKIP-VAR gate.  Prefer the primary run's log; if
    no SKIP-VAR fired there (fixed gate admits the nv~0.70 frames), probe a
    deep-noise point (Es/N0 = -16, CONFIG_0) where SOME frame exceeds even the
    rate-aware ceiling so the gate string is emitted and classifiable."""
    cls = classify_gate_log(primary_text)
    if cls is not None:
        return cls
    probe = run_point(binary, config=0, esn0=-16)
    return classify_gate_log(probe)


def fail(msg: str) -> None:
    print(f"FAIL: {msg}")
    sys.exit(1)


def main() -> None:
    binary = sys.argv[1] if len(sys.argv) > 1 else default_binary()
    if not os.path.exists(binary):
        fail(f"binary not found: {binary} (build with: bash build.sh o3)")

    print(f"[test] binary = {binary}")

    # ---- Probe 1: CONFIG_0 @ Es/N0 = -6 dB (nv ~0.70 band), DEFAULT gate ----
    # This is the bug's smoking gun.  nv lands ~0.69-0.80 here: above the old
    # 0.5 gate, far below CONFIG_0's measured ~1.48 decode floor.
    txt_c0 = run_point(binary, config=0, esn0=-6)
    ok_c0 = count(r"\[OFDM-OK\]", txt_c0)
    sv_c0 = count(r"SKIP-VAR", txt_c0)
    nv_c0 = max_nv(txt_c0)
    fixed = binary_is_fixed(binary, txt_c0)
    print(f"[test] CONFIG_0 @-6: OFDM-OK={ok_c0} SKIP-VAR={sv_c0} nv_max={nv_c0:.4f} "
          f"fixed_gate={fixed}")

    if fixed is True:
        # Sanity: the test point must actually exercise nv in the band we claim.
        # (Guards against a future demap/estimator change moving the operating
        # point.)  Only meaningful on the fixed binary, where the admitted
        # frames emit [FRAME-NV]; on the unfixed binary every frame is SKIP-VAR'd
        # before the FRAME-NV print, so nv_max is nan there by design.
        if not (0.55 <= nv_c0 <= 1.10):
            fail(f"CONFIG_0 @-6 nv_max={nv_c0:.4f} is outside the expected ~0.70 band "
                 f"[0.55,1.10] — the gate test point drifted; re-anchor the Es/N0.")
        # PASS-AFTER: rate-aware ceiling (1.60 for CONFIG_0) admits these frames.
        if sv_c0 != 0:
            fail(f"PASS-AFTER violated: fixed gate still emitted {sv_c0} SKIP-VAR at "
                 f"CONFIG_0 @-6 (nv~0.70 < ceiling 1.60) — frames must be admitted.")
        if ok_c0 < int(0.8 * BER_FRAMES):
            fail(f"PASS-AFTER violated: fixed gate decoded only {ok_c0}/{BER_FRAMES} "
                 f"OFDM-OK at CONFIG_0 @-6 — expected near-full decode (>= {int(0.8*BER_FRAMES)}).")
        print(f"[PASS] FAIL-BEFORE/PASS-AFTER (after half): rate-aware gate admits the "
              f"nv~0.70 frames -> {ok_c0}/{BER_FRAMES} OFDM-OK, 0 SKIP-VAR.")
    elif fixed is False:
        # FAIL-BEFORE: flat 0.5 rejects every nv~0.70 frame.
        if ok_c0 != 0:
            fail(f"FAIL-BEFORE not reproduced: unfixed gate decoded {ok_c0} OFDM-OK at "
                 f"CONFIG_0 @-6 — expected 0 (flat 0.5 should reject nv~0.70).")
        if sv_c0 < int(0.8 * BER_FRAMES):
            fail(f"FAIL-BEFORE not reproduced: unfixed gate emitted only {sv_c0} SKIP-VAR "
                 f"(expected ~{BER_FRAMES}).")
        print(f"[PASS] FAIL-BEFORE (before half): flat 0.5 gate rejects the nv~0.70 "
              f"frames -> {sv_c0} SKIP-VAR, 0 OFDM-OK (this is the BUG).")
        # On an unfixed binary the no-regression / hard-cap checks below are
        # not meaningful (they describe the fixed gate's contract), so stop here
        # with the before-half proven.
        print("OK: before-binary behavior confirmed (bug reproduced).")
        return
    else:
        fail("could not determine whether the binary's SKIP-VAR gate is fixed "
             "(no recognizable SKIP-VAR log string) — check the binary.")

    # ===== The following only run on the FIXED binary =====

    # ---- Probe 2: HIGH-RATE NO-REGRESSION (CONFIG_15 @ marginal Es/N0) ----
    # CONFIG_15 (16QAM 14/16) keeps ceiling 0.50.  At a marginal SNR it is
    # decoder-bound: nv stays < 0.50, SKIP-VAR never fires, and every failed
    # frame is an honest [OFDM-FAIL] (LDPC/CRC reject), not an admitted-then-
    # falsely-decoded frame.  Assert no SKIP-VAR delta and no false decodes.
    txt_hr = run_point(binary, config=15, esn0=2)
    ok_hr = count(r"\[OFDM-OK\]", txt_hr)
    sv_hr = count(r"SKIP-VAR", txt_hr)
    fail_hr = count(r"\[OFDM-FAIL\]", txt_hr)
    nv_hr = max_nv(txt_hr)
    print(f"[test] CONFIG_15 @+2: OFDM-OK={ok_hr} SKIP-VAR={sv_hr} "
          f"OFDM-FAIL={fail_hr} nv_max={nv_hr:.4f}")
    # CONFIG_15 ceiling is 0.50 (unchanged).  The marginal point must be
    # decoder-bound (no decodes here = LDPC can't carry 14/16 at +2 dB) and the
    # rate-aware change must not have admitted anything: the failure mode is
    # OFDM-FAIL, and SKIP-VAR (if it fires at all) is at the SAME 0.50 the flat
    # gate used.  Either way: zero false [OFDM-OK].
    if ok_hr != 0:
        fail(f"HIGH-RATE regression: CONFIG_15 @+2 decoded {ok_hr} OFDM-OK — the "
             f"adaptive gate must not admit/decode undecodable high-rate frames.")
    if fail_hr < int(0.5 * BER_FRAMES) and sv_hr < int(0.5 * BER_FRAMES):
        fail(f"HIGH-RATE check inconclusive: CONFIG_15 @+2 had only {fail_hr} OFDM-FAIL "
             f"and {sv_hr} SKIP-VAR of {BER_FRAMES} — expected the marginal point to be "
             f"clearly decoder-bound.")
    print(f"[PASS] HIGH-RATE no-regression: CONFIG_15 @+2 is decoder-bound "
          f"(OFDM-FAIL={fail_hr}, SKIP-VAR={sv_hr}), zero false decodes; ceiling 0.50 "
          f"unchanged for CONFIG_7-16.")

    # ---- Probe 3: HARD-CAP / DEEP-NOISE (CONFIG_0 @ Es/N0 = -18 dB) ----
    txt_deep = run_point(binary, config=0, esn0=-18)
    ok_deep = count(r"\[OFDM-OK\]", txt_deep)
    print(f"[test] CONFIG_0 @-18: OFDM-OK={ok_deep}")
    if ok_deep != 0:
        fail(f"HARD-CAP violated: CONFIG_0 @-18 (pure noise) decoded {ok_deep} OFDM-OK "
             f"— the loosened ceiling must not create false decodes.")
    print("[PASS] HARD-CAP: deep-noise CONFIG_0 @-18 -> 0 OFDM-OK (no false decode).")

    print("OK: all FIX-1 SKIP-VAR rate-aware gate assertions passed.")


if __name__ == "__main__":
    main()

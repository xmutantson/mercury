#!/usr/bin/env python3
"""
feat/turnaround-eff regression: sub-peak/trial-multiplier kill (#1) +
syndrome-weight non-convergence early-term (#3), with the SHARED detector.

Drives the coded SFO-GRID harness (the same vehicle ldpc-decode-accel.md §1
used). Fail-before / pass-after, per CLAUDE.md §3. See
fact-documents/turnaround-eff.md §6.

Checks
------
TEST-1  DEFAULT-OFF BYTE-IDENTICAL: with BOTH MERCURY_SYND_EARLYTERM and
        MERCURY_SUBPEAK_KILL unset, the 27-cell [SFO-GRID-CODED] render md5 is
        IDENTICAL to the base @65eb2be render. Proves zero behaviour change off.

TEST-3a EARLY-TERM FIRES: on uncorrectable cells (det-floor / cliff) with
        MERCURY_SYND_EARLYTERM=1, the shared detector trips (>=1 codeword) and
        bounds the doomed-frame decode to ~iter 12 (et_iter_max <= 16), instead
        of running to the iteration cap (the fail-before: off => no trip line).

TEST-3b NO FALSE-EARLY-STOP (lossless): with MERCURY_SYND_EARLYTERM=1 the
        [SFO-GRID-CODED] lines are byte-IDENTICAL to the OFF render across the
        whole 27-cell grid — every correctable codeword still decodes, every
        BER unchanged, every fail still classified FAIL. This is the §3
        correctness gate (a slow-but-real converger is NOT killed).

TEST-1k SUBPEAK-KILL OFF-PATH: MERCURY_SUBPEAK_KILL=1 alone (no EARLYTERM) does
        NOT perturb the coded render (the kill only changes the RX trial-loop
        sub-peak goto, which the single-grid harness does not exercise) — proves
        the env is inert on the decode path. The multiplier drop + wall-clock are
        Pi-confirmed (turnaround-eff.md §7).

Exit 0 = all pass.
"""
import hashlib
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# Base @65eb2be 27-cell [SFO-GRID-CODED] render md5 (NSYMB=600, K=62), computed
# directly from the base binary (turnaround-eff.md §6 TEST-1).
BASE_MD5 = "4575011ba38e23575ef3f3bbce0703b2"

CHANS = (0, 1, 2)
SEEDS = (12345, 999, 77)
ESN0S = (15, 16, 17)


def find_binary():
    for c in (os.environ.get("MERCURY_BIN"),
              os.path.join(ROOT, "mercury.exe"),
              os.path.join(ROOT, "mercury"),
              r"C:/Program Files/Mercury/mercury.exe"):
        if c and os.path.exists(c):
            return c
    sys.exit("ERROR: mercury binary not found. Set MERCURY_BIN=<path>.")


def run_cell(binary, chan, seed, esn0, extra=None, timeout=300):
    env = dict(os.environ)
    env.update({
        "MERCURY_SFO_GRID": "1",
        "MERCURY_SFO_GRID_CODED": "1",
        "MERCURY_SFO_GRID_NSYMB": "600",
        "MERCURY_SFO_GRID_CHAN": str(chan),
        "MERCURY_SFO_GRID_SEED": str(seed),
        "MERCURY_SFO_GRID_ESN0": str(esn0),
    })
    if extra:
        env.update(extra)
    p = subprocess.run([binary, "-m", "PLOT_PASSBAND", "-s", "16"],
                       env=env, capture_output=True, text=True, timeout=timeout)
    return p.stdout + p.stderr


def coded_lines(out):
    return [ln for ln in out.splitlines() if "[SFO-GRID-CODED]" in ln]


def earlyterm_lines(out):
    return [ln for ln in out.splitlines() if "[SFO-GRID-EARLYTERM]" in ln]


def grid_render(binary, extra=None):
    """Return the concatenated [SFO-GRID-CODED] lines of the 27-cell grid + the
    list of (cell, earlyterm_lines)."""
    coded = []
    et = []
    for chan in CHANS:
        for seed in SEEDS:
            for esn0 in ESN0S:
                out = run_cell(binary, chan, seed, esn0, extra)
                coded.extend(coded_lines(out))
                et.append(((chan, seed, esn0), earlyterm_lines(out)))
    return "\n".join(coded) + "\n", et


def md5(s):
    return hashlib.md5(s.encode()).hexdigest()


def main():
    binary = find_binary()
    print(f"[test_turnaround_eff] binary = {binary}")
    fails = []

    # ---- TEST-1: default-off byte-identical -------------------------------
    off_render, _ = grid_render(binary, extra=None)
    off_md5 = md5(off_render)
    print(f"[TEST-1] OFF render md5 = {off_md5}  (base = {BASE_MD5})")
    if off_md5 == BASE_MD5:
        print("[TEST-1] PASS: default-off byte-identical to base @65eb2be")
    else:
        print("[TEST-1] FAIL: OFF render differs from base!")
        fails.append("TEST-1")

    # ---- TEST-3b: lossless (no false-early-stop) --------------------------
    et_render, et_lines = grid_render(binary, extra={"MERCURY_SYND_EARLYTERM": "1"})
    et_md5 = md5(et_render)
    print(f"[TEST-3b] EARLYTERM render md5 = {et_md5}")
    if et_md5 == off_md5:
        print("[TEST-3b] PASS: EARLYTERM lossless — [SFO-GRID-CODED] byte-identical "
              "to OFF (zero convergers lost, all BER unchanged, fails still FAIL)")
    else:
        print("[TEST-3b] FAIL: EARLYTERM changed the coded render (false-early-stop)!")
        fails.append("TEST-3b")

    # ---- TEST-3a: early-term fires + bounds the doomed decode -------------
    n_trip_cells = 0
    worst_et_max = -1
    _ET_RE = re.compile(r"tripped=(\d+)/(\d+).*et_iter_max=(\d+)")
    for cell, lines in et_lines:
        for ln in lines:
            m = _ET_RE.search(ln)
            if m:
                trip, tot, etmax = int(m.group(1)), int(m.group(2)), int(m.group(3))
                if trip > 0:
                    n_trip_cells += 1
                    worst_et_max = max(worst_et_max, etmax)
    print(f"[TEST-3a] cells with >=1 trip = {n_trip_cells}, worst et_iter_max = {worst_et_max}")
    # Fail-before: confirm the OFF run has NO earlyterm line at all.
    off_had_et = any(lines for _, lines in
                     grid_render(binary, extra=None)[1])
    if n_trip_cells > 0 and 0 < worst_et_max <= 16 and not off_had_et:
        print("[TEST-3a] PASS: detector fires on uncorrectable cells, bounds doomed "
              f"decode to iter<=16 (was iter-cap); OFF has no trip (fail-before)")
    else:
        print("[TEST-3a] FAIL: detector did not fire as expected")
        fails.append("TEST-3a")

    # ---- TEST-1k: SUBPEAK_KILL inert on the coded decode path -------------
    sk_render, _ = grid_render(binary, extra={"MERCURY_SUBPEAK_KILL": "1"})
    if md5(sk_render) == off_md5:
        print("[TEST-1k] PASS: MERCURY_SUBPEAK_KILL does not perturb the coded "
              "render (kill only touches the RX trial-loop sub-peak goto; "
              "multiplier + wall-clock are Pi-confirmed, turnaround-eff.md §7)")
    else:
        print("[TEST-1k] FAIL: SUBPEAK_KILL changed the coded render!")
        fails.append("TEST-1k")

    print()
    if fails:
        print(f"[test_turnaround_eff] FAILED: {fails}")
        sys.exit(1)
    print("[test_turnaround_eff] ALL PASS")


if __name__ == "__main__":
    main()

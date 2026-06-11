#!/usr/bin/env python3
"""
test_fade_tinterp.py -- regression for the FADE-tier TIME_INTERP estimator
(feat/fade-tinterp). Pairs the data-flow audit fact-documents/
data-flow-noise_variance_estimate.md (CLAUDE.md cross-layer regression rule).

Drives the in-process MERCURY_SFO_GRID harness (-m PLOT_PASSBAND), no IONOS / no RF.
Asserts, on the ITU-R Watterson profiles the 900-cell sim verdict used
(compute_program/jobs/fade-estimator-prototypes/ESTIMATOR_PROTOTYPES_VERDICT.md):

  FAILING-FIRST (the production LS path is the BEFORE state):
    A) PROD LS estimator decodes 0/K codewords on MPG/GOOD (0.1 Hz) -- the
       documented production failure (telecom_system.cc final `else { LS }`).
  AFTER-FIX (the promoted PRODUCTION method, via MERCURY_SFO_GRID_TINTERP_PROD):
    B) TIME_INTERP decodes K/K on MPG/GOOD  -- full cross (verdict 1.00).
    C) TIME_INTERP decodes >= 0.8*K on MPM/MOD (0.5 Hz) -- ARQ-viable (verdict ~0.88).
    D) nv never collapses (> 1e-5) on ANY profile -- the I1 cross-pilot nv-floor
       invariant (E1/cfg16-nvfix class; fade-estimator-prototypes.md §4.1).
  DEFAULT-OFF byte identity is asserted separately by the harness PROD-else md5 A/B
  in the workflow; this script asserts the win + the nv-floor.

Usage:  python tools/test_fade_tinterp.py [path/to/mercury.exe]
        (defaults to ./mercury.exe in the repo root)
Exit 0 = all pass, non-zero = a regression.
"""
import os
import re
import subprocess
import sys

# ITU-R F.1487 profiles, identical to the prototype generator
# (compute_program/fleet/runner/gen_fade_estimator_prototypes_job.py:48-50):
#   name, chan_sel, depth_db, fd_hz, dly_samp
PROFILES = {
    "MPG": (3, 10, 0.1, 24),   # GOOD  -> full cross expected
    "MPM": (3, 12, 0.5, 48),   # MOD   -> ARQ-viable (>=0.8) expected
    "MPP": (3, 16, 1.0, 96),   # POOR  -> non-cross (documented), nv must still be sane
}
CFG = 15
ESN0 = 24

CODED_RE = re.compile(r"codewords_decoded=(\d+)/(\d+)")
NV_RE = re.compile(r"\bnv=([0-9.eE+\-]+)")


def run(mercury, env_extra):
    env = dict(os.environ)
    env["MERCURY_SFO_GRID"] = "1"
    env["MERCURY_SFO_GRID_CODED"] = "1"
    env["MERCURY_SFO_GRID_ESN0"] = str(ESN0)
    env.update(env_extra)
    p = subprocess.run([mercury, "-m", "PLOT_PASSBAND", "-s", str(CFG)],
                       env=env, capture_output=True, text=True, timeout=300)
    return p.stdout + p.stderr


def cell_env(name):
    chan, depth, fd, dly = PROFILES[name]
    return {
        "MERCURY_SFO_GRID_CHAN": str(chan),
        "MERCURY_SFO_GRID_WATT_DEPTH_DB": str(depth),
        "MERCURY_SFO_GRID_WATT_FD_HZ": str(fd),
        "MERCURY_SFO_GRID_WATT_DLY": str(dly),
    }


def parse(out):
    cw = CODED_RE.search(out)
    decoded, total = (int(cw.group(1)), int(cw.group(2))) if cw else (None, None)
    # last nv on the CODED result line
    nvm = None
    for line in out.splitlines():
        if "SFO-GRID-CODED" in line and "nv=" in line:
            m = NV_RE.search(line)
            if m:
                nvm = float(m.group(1))
    return decoded, total, nvm


def main():
    mercury = sys.argv[1] if len(sys.argv) > 1 else "./mercury.exe"
    if not os.path.exists(mercury):
        print("FAIL: mercury binary not found: %s" % mercury)
        return 2

    fails = []

    # A) FAILING-FIRST: PROD LS on MPG/GOOD must decode 0/K.
    out = run(mercury, cell_env("MPG"))
    dec, tot, nv = parse(out)
    if dec is None:
        fails.append("A: no CODED result parsed (prod LS / MPG)")
    elif dec != 0:
        fails.append("A: PROD LS on MPG/GOOD decoded %d/%d (expected 0/%d -- the "
                     "before-state failure)" % (dec, tot, tot))
    print("[A] PROD LS  MPG/GOOD  decoded=%s/%s nv=%s  (want 0/K)" % (dec, tot, nv))

    # B) AFTER-FIX: PRODUCTION TIME_INTERP on MPG/GOOD must decode K/K.
    env = cell_env("MPG"); env["MERCURY_SFO_GRID_TINTERP_PROD"] = "1"
    out = run(mercury, env)
    dec, tot, nv = parse(out)
    if dec is None:
        fails.append("B: no CODED result parsed (TINTERP / MPG)")
    elif dec != tot:
        fails.append("B: TIME_INTERP on MPG/GOOD decoded %d/%d (expected full %d/%d "
                     "-- the promoted win)" % (dec, tot, tot, tot))
    if nv is None or nv <= 1e-5:
        fails.append("D-MPG: TIME_INTERP nv collapsed (%s <= 1e-5) -- nv-floor "
                     "invariant I1 violated" % nv)
    print("[B] TINTERP  MPG/GOOD  decoded=%s/%s nv=%s  (want K/K, nv>1e-5)" % (dec, tot, nv))

    # C) AFTER-FIX: PRODUCTION TIME_INTERP on MPM/MOD must decode >= 0.8*K.
    env = cell_env("MPM"); env["MERCURY_SFO_GRID_TINTERP_PROD"] = "1"
    out = run(mercury, env)
    dec, tot, nv = parse(out)
    if dec is None:
        fails.append("C: no CODED result parsed (TINTERP / MPM)")
    elif dec < 0.8 * tot:
        fails.append("C: TIME_INTERP on MPM/MOD decoded %d/%d (expected >= 0.8*%d "
                     "ARQ-viable)" % (dec, tot, tot))
    if nv is None or nv <= 1e-5:
        fails.append("D-MPM: TIME_INTERP nv collapsed (%s <= 1e-5)" % nv)
    print("[C] TINTERP  MPM/MOD   decoded=%s/%s nv=%s  (want >=0.8*K, nv>1e-5)" % (dec, tot, nv))

    # D) nv sane (not collapsed) on MPP/POOR too (decode is a documented non-cross).
    env = cell_env("MPP"); env["MERCURY_SFO_GRID_TINTERP_PROD"] = "1"
    out = run(mercury, env)
    dec, tot, nv = parse(out)
    if nv is None or nv <= 1e-5:
        fails.append("D-MPP: TIME_INTERP nv collapsed (%s <= 1e-5)" % nv)
    print("[D] TINTERP  MPP/POOR  decoded=%s/%s nv=%s  (decode non-cross OK; nv>1e-5)" % (dec, tot, nv))

    if fails:
        print("\n=== FAIL (%d) ===" % len(fails))
        for f in fails:
            print("  - " + f)
        return 1
    print("\n=== test_fade_tinterp: ALL PASS ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())

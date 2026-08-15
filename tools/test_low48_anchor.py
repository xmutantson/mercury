#!/usr/bin/env python3
"""Fail-closed geometry and high-SNR decode regression for config 105."""

import os
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BINARY = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / "mercury"
EXPECTED_DATA = "0,3,5,8,10,13,15,18,21,23,26,28,31,34,36,39,41,44,46,49"
EXPECTED_PILOTS = "1,6,11,17,22,27,32,38,43,48"


def run(args, env=None):
    merged = os.environ.copy()
    if env:
        merged.update(env)
    return subprocess.run(
        [str(BINARY), *args], cwd=ROOT, env=merged,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=60, check=False,
    )


def fail(message, result=None):
    print(f"FAIL: {message}", file=sys.stderr)
    if result is not None:
        print(result.stdout[-6000:], file=sys.stderr)
    raise SystemExit(1)


if not BINARY.is_file():
    fail(f"binary not found: {BINARY}")

# Fail-before/pass-after: before config 105 existed this invocation was rejected;
# after the change it must expose the exact fixed geometry and decode every frame.
smoke = run([
    "-m", "PLOT_PASSBAND", "-s", "105",
    "--ber-esn0=30", "--ber-frames=8",
])
if smoke.returncode != 0:
    fail("config 105 PLOT_PASSBAND invocation failed", smoke)

geom = re.search(
    r"\[LOW48-GEOMETRY\] dataCols=([^ ]+) pilotCols=([^ ]+) "
    r"nData=(\d+) nPilots=(\d+) nZero=(\d+)",
    smoke.stdout,
)
if not geom:
    fail("missing fixed-geometry attestation", smoke)
if geom.groups() != (EXPECTED_DATA, EXPECTED_PILOTS, "800", "400", "800"):
    fail(f"unexpected or overlapping carrier map: {geom.groups()}", smoke)

active = re.search(
    r"Config 105 active: M=4 LDPC_rate=0\.375 BW=2344Hz "
    r"Nc=50 Nsymb=40 nBits=1600",
    smoke.stdout,
)
if not active:
    fail("active geometry/rate does not match S20-R6", smoke)

rate = re.search(r"Modulation: 105\s+Bitrate: ([0-9.]+) bps", smoke.stdout)
if not rate or abs(float(rate.group(1)) - 545.45) > 0.02:
    fail("production-GI nominal client rate is not 545.45 bit/s", smoke)
if len(re.findall(r"\[OFDM-OK\].*cfg=105", smoke.stdout)) != 8:
    fail("high-SNR loopback did not CRC+LDPC-decode all 8 frames", smoke)
if not re.search(r"^30(?:\.0+)?;0(?:\.0+)?$", smoke.stdout, re.MULTILINE):
    fail("high-SNR loopback BER is nonzero", smoke)
print("PASS: config 105 fixed S20-R6 geometry and 8/8 high-SNR decode")

# Config 105 is deliberately BER-only and cannot leak through another mode.
wrong_mode = run(["-m", "PLOT_BASEBAND", "-s", "105"])
if wrong_mode.returncode == 0 or "Wrong modulation config 105" not in wrong_mode.stdout:
    fail("config 105 escaped its explicit PLOT_PASSBAND-only reachability gate", wrong_mode)
print("PASS: config 105 remains outside production/negotiated modes")

# The next-free-ID contract also proves that the reserved predecessor remains
# unknown rather than aliasing a production or experimental geometry.
unknown = run([
    "-m", "PLOT_PASSBAND", "-s", "104",
    "--ber-esn0=30", "--ber-frames=1",
])
if unknown.returncode == 0 or "Wrong modulation config 104" not in unknown.stdout:
    fail("unknown config 104 did not fail closed", unknown)
print("PASS: unknown config fails closed")

# G1 safety regression: the old sweep accepted 64*4 carriers into Nc=50 and
# could index beyond the allocated grid.  It must now stop before modulation.
invalid_mfsk = run(
    ["-m", "PLOT_PASSBAND", "-s", "100", "--ber-esn0=30", "--ber-frames=1"],
    {"MERCURY_MFSK_SWEEP_M": "64", "MERCURY_MFSK_SWEEP_NSTREAMS": "4"},
)
if invalid_mfsk.returncode == 0 or "[MFSK-GUARD] rejecting M=64 nStreams=4 Nc=50" not in invalid_mfsk.stdout:
    fail("invalid MFSK carrier product was not rejected before modulation", invalid_mfsk)
print("PASS: invalid MFSK M*nStreams product fails closed")

print("ALL PASS")

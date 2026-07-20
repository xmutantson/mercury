#!/usr/bin/env python3
"""Guard the rejected matched-filter acquisition experiment.

This is intentionally not part of the default fast unit suite: its exercise arm
runs the 60-frame CONFIG_16 SFO block.  The production receiver must retain the
sub-sample self-correlation fine-timing path, and the historical measurements
that explain why must remain in-tree.
"""

import os
import pathlib
import re
import subprocess
import sys
from typing import Tuple


ROOT = pathlib.Path(
    os.environ.get("MERCURY_SOURCE_ROOT", pathlib.Path(__file__).resolve().parents[1])
).resolve()
DOC = ROOT / "fact-documents" / "matched-filter-acquisition.md"
TELECOM = ROOT / "source" / "physical_layer" / "telecom_system.cc"


def find_binary() -> pathlib.Path:
    if len(sys.argv) > 1:
        return pathlib.Path(sys.argv[1]).resolve()
    for name in ("mercury", "mercury.exe"):
        candidate = ROOT / name
        if candidate.is_file():
            return candidate
    raise RuntimeError("mercury binary not found; pass its path as argv[1]")


def run_sfo_delivery(binary: pathlib.Path) -> Tuple[int, int]:
    env = os.environ.copy()
    env.pop("MERCURY_OFDM_MATCHED", None)
    env.update(
        {
            "MERCURY_SFO_BLOCK_TEST": "1",
            "MERCURY_SFO_BLOCK_NFRAMES": "60",
            "MERCURY_SFO_BLOCK_ESN0": "12",
            "MERCURY_SFO_BLOCK_SEED": "12345",
            "MERCURY_SIM2_SFO_PPM": "8",
            "MERCURY_SFO_BLOCK_ARM": "FULL",
        }
    )
    proc = subprocess.run(
        [str(binary), "-m", "PLOT_PASSBAND", "-s", "16"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        timeout=180,
    )
    output = proc.stdout + proc.stderr
    match = re.search(r"frames_decoded=(\d+)/(\d+)", output)
    if proc.returncode != 0 or not match:
        raise RuntimeError(
            "CONFIG_16 8-ppm SFO exercise did not complete\n" + output[-4000:]
        )
    return int(match.group(1)), int(match.group(2))


def main() -> int:
    try:
        binary = find_binary()
        decoded, total = run_sfo_delivery(binary)
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f"FAIL: {exc}")
        return 1

    # The rejected matched path delivered only 31/60 in this exact 12-dB,
    # 8-ppm cell; retain comfortable margin above it without pinning every
    # future decoder improvement to the historical self-correlation count 53.
    if total != 60 or decoded < 45:
        print(
            "FAIL: CONFIG_16 8-ppm SFO delivery regressed: "
            f"frames_decoded={decoded}/{total}, require >=45/60"
        )
        return 1
    print(f"PASS: CONFIG_16 8-ppm SFO delivery frames_decoded={decoded}/{total}")

    telecom = TELECOM.read_text(encoding="utf-8")
    if "MERCURY_OFDM_MATCHED" in telecom:
        print("FAIL: rejected MERCURY_OFDM_MATCHED production selector returned")
        return 1
    if re.search(r"\.time_sync_preamble_matched\s*\(", telecom):
        print("FAIL: matched detector is called from the production telecom path")
        return 1
    print("PASS: matched detector remains diagnostic-only")

    if not DOC.is_file():
        print("FAIL: historical matched-filter SFO verdict is missing")
        return 1
    doc = DOC.read_text(encoding="utf-8")
    required = (
        "HISTORICAL / NON-PRODUCTION",
        "30/60 | 7/60",
        "53/60 | 31/60",
        "fractional-delay",
    )
    missing = [token for token in required if token not in doc]
    if missing:
        print("FAIL: historical verdict lost required evidence: " + ", ".join(missing))
        return 1
    print("PASS: measured negative result and prerequisite remain documented")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

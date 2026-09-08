#!/usr/bin/env python3
"""Regression for SIM_INPROC propagation of the existing --gi option."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
BINARY = ROOT / "mercury"


def run(extra: list[str]) -> str:
    env = os.environ.copy()
    env.update(
        {
            "MERCURY_SIM_2INST": "1",
            "MERCURY_SIM2_CFG": "16",
            "MERCURY_SIM2_ROBUST": "0",
            "MERCURY_SIM2_PIN": "1",
            "MERCURY_SIM2_SNR3K": "900",
            "MERCURY_SIM2_PAYLOAD_BYTES": "1024",
            "MERCURY_SIM2_MAXITERS": "200000",
            "MERCURY_SIM2_STALL_ITERS": "50000",
        }
    )
    completed = subprocess.run(
        [str(BINARY), "-m", "SIM_INPROC", "-n", *extra],
        cwd=ROOT,
        env=env,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=60,
    )
    if completed.returncode != 0:
        raise AssertionError(
            f"mercury exited {completed.returncode}\n{completed.stdout[-4000:]}"
        )
    if "bytes_ok=1" not in completed.stdout:
        raise AssertionError(f"payload did not complete\n{completed.stdout[-4000:]}")
    return completed.stdout


def main() -> int:
    explicit = run(["--gi", "1.33"])
    assert "[SIM_INPROC] honoring --gi 1.33 ms" in explicit
    assert "[TEST-SIM-2INST] explicit GI: 1.33 ms (Ngi=16)" in explicit
    assert "Config 16 active" in explicit
    assert "Nofdm=272" in explicit

    default = run([])
    assert "[SIM_INPROC] honoring --gi" not in default
    assert "[TEST-SIM-2INST] explicit GI:" not in default

    print("PASS: explicit --gi reaches both SIM_INPROC peers; unset path unchanged")
    return 0


if __name__ == "__main__":
    sys.exit(main())

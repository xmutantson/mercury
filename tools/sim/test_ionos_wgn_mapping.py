#!/usr/bin/env python3
"""Gate the WGN-label compatibility map against the physical IONOS sweep."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_channel_relay as relay


# _research/equal_snr_ionos_20260721_v9_gated15s/results.json
PHYSICAL_SNR3K = {
    -10: -5.176, -6: -1.308, -2: 2.702, 0: 4.690, 2: 6.704,
    6: 10.724, 10: 14.713, 14: 18.711, 22: 26.699, 30: 34.628,
    40: 43.528,
}


def main():
    ok = True
    for label, physical in PHYSICAL_SNR3K.items():
        mapped = relay.parse_cell(f"WGN:{label}")
        error = mapped - physical
        passed = abs(error) <= 0.25
        ok = ok and passed
        print(f"WGN:{label:+3d} mapped={mapped:+7.3f} "
              f"physical={physical:+7.3f} error={error:+.3f} "
              f"[{'PASS' if passed else 'FAIL'}]")
    literal = relay.parse_cell("12.345")
    passed = math.isclose(literal, 12.345, abs_tol=1e-12)
    ok = ok and passed
    print(f"literal={literal:.3f} [{'PASS' if passed else 'FAIL'}]")
    print("IONOS WGN mapping " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

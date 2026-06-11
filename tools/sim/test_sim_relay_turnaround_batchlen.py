#!/usr/bin/env python3
"""
test_sim_relay_turnaround_batchlen.py — SUPERSEDED (v1 mechanism error).

WHY THIS TEST IS RETIRED (SIMTURNCAL_VERDICT.json):
This test encoded the v1 batch-length accrual mechanism, which was WRONG. It
drove a SINGLE TurnaroundDrift instance with n_frames of forward signal chunks
and read that SAME instance's owed offset at its NEXT onset — i.e. it asserted a
PER-DIRECTION SELF-accrual: a direction's own just-ended burst pushed its OWN
next onset late. In the relay, the forward (a2b OFDM) and reverse (b2a MFSK ACK)
directions are SEPARATE TurnaroundDrift instances, so a per-direction self-accrual
meant the long FORWARD OFDM batch accrued an offset that was spent inserting
SILENCE AHEAD OF the FORWARD signal burst (sim_channel_relay.py v1 lines 705-712)
-> de-aligned the forward OFDM decode (relay ins=90313 samp / 286 ms into the
forward signal; FTR-FAIL x414, SKIP-VAR x116, metric collapse, modem proc_died;
held-CFG16 ~1897 bps, 3.2x too optimistic vs the bench-9 597.6).

THE CORRECTED v2 MECHANISM is CROSS-DIRECTION (TurnaroundCoupler): the accrual
amount is keyed to the FORWARD (a2b) batch airtime but the LATE silence is
inserted into the REVERSE (b2a) turnaround GAP — the forward OFDM stays BIT-EXACT
and the reverse MFSK ACK arrives late at the CMD, landing outside its window. The
v2 contracts are validated by test_sim_relay_turnaround_xdir.py.

This shim DELEGATES to the v2 test so the file name stays discoverable and any
CI/loop that invokes it exercises the corrected mechanism. It exits with the v2
test's status.
"""
import os
import runpy
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_V2 = os.path.join(_HERE, "test_sim_relay_turnaround_xdir.py")


def main():
    print("=== test_sim_relay_turnaround_batchlen.py SUPERSEDED ===")
    print("    v1 per-direction self-accrual was the WRONG mechanism "
          "(SIMTURNCAL_VERDICT.json).")
    print("    Delegating to the corrected CROSS-DIRECTION test: "
          "test_sim_relay_turnaround_xdir.py\n")
    sys.argv = [_V2]
    try:
        runpy.run_path(_V2, run_name="__main__")
    except SystemExit as e:
        return int(e.code) if e.code is not None else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())

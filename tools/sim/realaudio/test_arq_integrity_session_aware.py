#!/usr/bin/env python3
"""Session-aware byte-integrity oracle regression.

Reproduces the contention-certification round-5 corrupt-cell shear: three ARQ
sessions delivering 24 + 0 + 18432 application bytes, each restarting the
payload at offset 0 (a disconnect/reconnect leaves the local KISS data socket
open while the responder re-delivers the transfer from its beginning).  A
cumulative positional oracle -- one that never realigns the expected-source
offset at a session boundary -- reads this clean multi-session delivery as
CORRUPT, shearing at the first boundary (byte 24, the round-5 first_bad_offset,
~1/256 spurious match rate).  The session-aware oracle realigns each session to
application offset 0 and reads it clean.

This is the fail-before proof, self-contained in one deterministic run: the
cumulative arm reproduces the shipped-broken behavior and reads False; the
session-aware arm reads True.  A genuine mid-session corruption is still caught
after the fix (the realignment moves the OFFSET, it does not erase real mismatch
evidence).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arq_realaudio import ByteIntegrityTracker, traffic_slice

LEDGER = (24, 0, 18432)          # per-session delivered bytes (round-5 ledger)
TRAFFIC = "random-binary"


def _feed_ledger(session_aware):
    """Drive the exact 3-session ledger; realign at each boundary iff
    session_aware.  Each session re-delivers the payload from offset 0.  With
    session_aware False the expected-source offset accumulates monotonically --
    a faithful reproduction of the pre-fix cumulative oracle."""
    tracker = ByteIntegrityTracker(TRAFFIC)
    for index, count in enumerate(LEDGER):
        if count:
            tracker.feed(traffic_slice(TRAFFIC, 0, count))
        if index < len(LEDGER) - 1 and session_aware:
            tracker.note_disconnect()
    return tracker


def main():
    aware = _feed_ledger(session_aware=True)
    cumulative = _feed_ledger(session_aware=False)

    # Session-aware oracle: the delivery is clean.
    snap = aware.snapshot()
    assert aware.ok is True, "session-aware oracle must read the ledger clean"
    assert snap["byte_integrity_ok"] is True
    assert snap["content_md5_ok"] is True, "delivered md5 must equal expected md5"
    assert snap["total_received_bytes"] == sum(LEDGER)
    assert snap["max_session_bytes"] == max(LEDGER)
    assert snap["integrity_first_bad_offset"] is None
    # Two data-bearing sessions (the empty middle session bumps disconnects but
    # not the session count).
    assert snap["integrity_disconnects"] == len(LEDGER) - 1
    assert snap["integrity_sessions"] == 2

    # Cumulative oracle (the shipped-broken behavior): the same clean delivery
    # reads as CORRUPT, and the shear lands exactly at the first session
    # boundary (byte 24), the round-5 first_bad_offset.
    cum = cumulative.snapshot()
    assert cumulative.ok is False, "cumulative oracle must SHEAR on the ledger"
    assert cum["byte_integrity_ok"] is False
    assert cum["content_md5_ok"] is False
    assert cum["integrity_first_bad_offset"] == LEDGER[0], (
        f"shear must land at byte {LEDGER[0]}, got "
        f"{cum['integrity_first_bad_offset']}")

    # A genuine mid-session corruption is still caught session-aware.
    corrupt = ByteIntegrityTracker(TRAFFIC)
    good = bytearray(traffic_slice(TRAFFIC, 0, 4096))
    good[2048] ^= 0xFF
    corrupt.feed(bytes(good))
    corrupt.note_disconnect()
    corrupt.feed(traffic_slice(TRAFFIC, 0, 4096))
    assert corrupt.ok is False, "a real mid-session corruption must survive reset"
    assert corrupt.snapshot()["integrity_first_bad_offset"] == 2048

    print("SESSION-AWARE INTEGRITY PASS: aware clean, cumulative shears at "
          f"byte {LEDGER[0]}, real corruption preserved")
    return 0


if __name__ == "__main__":
    sys.exit(main())

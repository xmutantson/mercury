#!/usr/bin/env python3
"""Unit coverage for the RX-OVERRUN-TOTAL merged-log parser."""
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arq_realaudio as ra  # noqa: E402


# Literal merged-log shape produced by log_output() around the production
# reset_session_state() emission. Multiple reconnect teardowns produce multiple
# rows, including independent CMD and RSP session segments.
MULTI_SEGMENT_REAL_LOG = """\
[T+0002.015] [RSP] [CFG] load_configuration(100) current=-1 level=FULL backup=NO
[T+0043.710] [CMD] link_status:Connected to TESTB
[T+0091.442] [RSP] RX-OVERRUN-TOTAL n=7
[T+0091.450] [CMD] RX-OVERRUN-TOTAL n=0
[T+0117.003] [RSP] link_status:Disconnected
[T+0117.081] [RSP] RX-OVERRUN-TOTAL n=13
[T+0117.094] [CMD] RX-OVERRUN-TOTAL n=2
"""

ZERO_SEGMENT_REAL_LOG = """\
[T+0002.015] [RSP] [CFG] load_configuration(100) current=-1 level=FULL backup=NO
[T+0043.710] [CMD] link_status:Connected to TESTB
[T+0044.102] [RSP] stats.nReceived_data= 1
"""

MALFORMED_REAL_LOG = """\
[T+0091.442] [RSP] RX-OVERRUN-TOTAL n=7
[T+0117.081] [RSP] RX-OVERRUN-TOTAL n=not-a-counter
"""


def main():
    total, segments = ra.parse_rx_overrun_metrics(io.StringIO(MULTI_SEGMENT_REAL_LOG))
    assert segments == [7, 0, 13, 2], segments
    assert total == 22, total

    total, segments = ra.parse_rx_overrun_metrics(io.StringIO(ZERO_SEGMENT_REAL_LOG))
    assert segments == [], segments
    assert total is None, total

    try:
        ra.parse_rx_overrun_segments(io.StringIO(MALFORMED_REAL_LOG))
    except ValueError as e:
        assert "malformed RX-OVERRUN-TOTAL line 2" in str(e), str(e)
    else:
        raise AssertionError("malformed RX-OVERRUN-TOTAL line was accepted")

    print("[OK] RX overrun parser: multi-segment sum=22, absent=null, malformed=rejected")
    return 0


if __name__ == "__main__":
    sys.exit(main())

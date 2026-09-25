#!/usr/bin/env python3
"""Directed tests for the last-delivered-byte time (ticket #218).

The result carries the arrival time of the last delivered byte (last_rx_at_s) and of the last
byte-exact prefix growth, derived from the delivery oracle's events, so the endpoint of an
INCOMPLETE cell is known. Driven through the harness's own rx_thread_fn over a socketpair with
a scripted partial delivery.

Run against another harness directory with HARNESS_DIR=<dir> (fail-before check).
"""
import hashlib
import os
import socket
import sys
import threading
import time
import unittest

HARNESS_DIR = os.environ.get("HARNESS_DIR") or os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HARNESS_DIR)
import capstone_arms as CA  # noqa: E402
import arq_realaudio as AR  # noqa: E402
from canonical_window_scorer import DeliveryOracle  # noqa: E402

TRAFFIC = "random-binary"


def _run_rx(segments, gap_s, payload_target, corrupt_index=None):
    """Send `segments` (lengths) of the reference stream with `gap_s` between them,
    then leave the socket open and idle (a stalled transfer). Returns the oracle
    snapshot, the completion dict, the harness-clock send times and the stop time."""
    a, b = socket.socketpair()
    oracle = DeliveryOracle(lambda off, n: CA.traffic_expected_slice(TRAFFIC, off, n))
    stop = threading.Event()
    res = {"tx": 0, "rx": 0}
    res_lock = threading.Lock()
    integ = {"mismatch_bytes": 0, "mismatch_segments": 0, "first_mismatch_off": None,
             "cap": [], "rx_md5": hashlib.md5()}
    completion = {"at_s": None, "count_at_s": None, "payload_target": payload_target}
    t0 = time.monotonic()
    th = threading.Thread(target=AR.rx_thread_fn,
                          args=(b, stop, res, integ, TRAFFIC, oracle, t0, res_lock, completion),
                          daemon=True)
    th.start()
    sent_at = []
    off = 0
    for i, n in enumerate(segments):
        data = bytearray(CA.traffic_slice(TRAFFIC, off, n))
        if i == corrupt_index:
            data[n // 2] ^= 0xFF
        sent_at.append(time.monotonic() - t0)   # taken before the send: rx cannot precede it
        a.sendall(bytes(data))
        off += n
        time.sleep(gap_s)
    time.sleep(0.5)          # stalled: no more bytes, socket still open
    stop_at = time.monotonic() - t0
    stop.set()
    th.join(timeout=5)
    a.close()
    b.close()
    return oracle.snapshot(), completion, sent_at, stop_at


class LastDeliveredByteTime(unittest.TestCase):
    def test_incomplete_cell_has_endpoint(self):
        snap, completion, sent_at, stop_at = _run_rx([4000, 3000, 2500], 0.25, 50000)
        self.assertIsNone(completion["at_s"])               # incomplete: no completion time
        t = AR.delivery_endpoint_times(snap["events"])
        self.assertGreaterEqual(t["delivery_event_count"], 1)
        self.assertEqual(snap["received_bytes"], 9500)
        # the last delivered byte arrived after the last send and well before the stop
        self.assertGreaterEqual(t["last_rx_at_s"], sent_at[-1])
        self.assertLess(t["last_rx_at_s"], sent_at[-1] + 0.2)
        self.assertLess(t["last_rx_at_s"], stop_at)
        self.assertEqual(t["last_good_prefix_advance_at_s"], t["last_rx_at_s"])

    def test_complete_cell_endpoint_matches_completion(self):
        snap, completion, sent_at, stop_at = _run_rx([6000, 4000], 0.25, 10000)
        t = AR.delivery_endpoint_times(snap["events"])
        self.assertIsNotNone(completion["at_s"])
        self.assertEqual(t["last_good_prefix_advance_at_s"], completion["at_s"])
        self.assertGreaterEqual(t["last_rx_at_s"], completion["at_s"])

    def test_corruption_separates_good_prefix_time(self):
        snap, completion, sent_at, stop_at = _run_rx([3000, 3000, 3000], 0.25, 50000,
                                                     corrupt_index=1)
        t = AR.delivery_endpoint_times(snap["events"])
        self.assertIsNotNone(snap["first_bad_offset"])
        self.assertLess(t["last_good_prefix_advance_at_s"], t["last_rx_at_s"])

    def test_no_delivery(self):
        t = AR.delivery_endpoint_times([])
        self.assertEqual(t, {"last_rx_at_s": None, "last_good_prefix_advance_at_s": None,
                             "delivery_event_count": 0})

    def test_result_json_emits_the_fields(self):
        src = open(AR.__file__, encoding="utf-8").read()
        for key in ("last_rx_at_s", "last_good_prefix_advance_at_s", "delivery_event_count"):
            self.assertIn('"%s": delivery_times["%s"]' % (key, key), src)


if __name__ == "__main__":
    unittest.main(verbosity=2)

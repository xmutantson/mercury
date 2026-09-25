#!/usr/bin/env python3
"""Offline tests for raw payload retention (raw_evidence.py) through the harness's
own rx/tx thread functions (arq_realaudio.rx_thread_fn / tx_thread_fn over a socketpair)."""

import json
import os
import socket
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import capstone_arms as CA  # noqa: E402
import raw_evidence  # noqa: E402

try:
    import arq_realaudio as AR  # noqa: E402
except Exception as exc:  # pragma: no cover - reported as a skip reason
    AR = None
    AR_ERR = repr(exc)


class RawEvidenceUnit(unittest.TestCase):
    def test_retains_and_compares_independently(self):
        with tempfile.TemporaryDirectory() as d:
            ev = raw_evidence.RawEvidence(d, "t", "random-binary", CA.traffic_slice)
            ref = CA.traffic_slice("random-binary", 0, 5000)
            ev.tx(ref[:3000])
            ev.tx(ref[3000:])
            bad = bytearray(ref[:4000])
            bad[1234] ^= 0x55
            ev.rx(bytes(bad[:2000]))
            ev.rx(bytes(bad[2000:]))
            rec = ev.finalize(4000, 5000)
            self.assertTrue(rec["tx_matches_reference"])
            self.assertEqual(rec["independent_compare"], "MISMATCH")
            self.assertEqual(rec["independent_first_mismatch_offset"], 1234)
            self.assertEqual(rec["rx_payload_bytes"], 4000)
            self.assertEqual(Path(rec["rx_payload_path"]).read_bytes(), bytes(bad))
            self.assertEqual(rec["errors"], [])

    def test_timed_out_attempt_keeps_bytes_before_finalize(self):
        with tempfile.TemporaryDirectory() as d:
            ev = raw_evidence.RawEvidence(d, "t", "random-binary", CA.traffic_slice)
            ev.rx(CA.traffic_slice("random-binary", 0, 777))
            # no finalize (harness killed): bytes are already on disk
            self.assertEqual(len(Path(ev.rx_path).read_bytes()), 777)
            ev.finalize()  # release the handles so the temp dir can be removed on Windows

    def test_count_disagreement_is_recorded(self):
        with tempfile.TemporaryDirectory() as d:
            ev = raw_evidence.RawEvidence(d, "t", "random-binary", CA.traffic_slice)
            ev.rx(CA.traffic_slice("random-binary", 0, 10))
            rec = ev.finalize(harness_rx_bytes=11)
            self.assertTrue(any("harness rx count" in e for e in rec["errors"]))


@unittest.skipIf(AR is None, "arq_realaudio import failed")
class HarnessThreadPath(unittest.TestCase):
    """Drive the production rx/tx thread functions of the harness over a socketpair."""

    def test_rx_and_tx_threads_retain_bytes(self):
        with tempfile.TemporaryDirectory() as d:
            AR._RAW_EVIDENCE = raw_evidence.RawEvidence(d, "cell", "random-binary", CA.traffic_slice)
            try:
                a, b = socket.socketpair()
                stop = threading.Event()
                res = {"tx": 0, "rx": 0}
                lock = threading.Lock()
                tx_control = {"target": 6000, "warm_max_ahead": 1 << 20}
                exhaustion = {"at_s": None}
                t0 = time.monotonic()
                txt = threading.Thread(target=AR.tx_thread_fn,
                                       args=(a, stop, res, tx_control, "random-binary", t0, lock, exhaustion))
                oracle = AR.DeliveryOracle(lambda o, n: CA.traffic_expected_slice("random-binary", o, n))
                integ = {"mismatch_bytes": 0, "mismatch_segments": 0, "first_mismatch_off": None, "cap": [],
                         "rx_md5": __import__("hashlib").md5()}
                completion = {"at_s": None, "count_at_s": None, "payload_target": 6000}
                rxt = threading.Thread(target=AR.rx_thread_fn,
                                       args=(b, stop, res, integ, "random-binary", oracle, t0, lock, completion))
                rxt.start()
                txt.start()
                txt.join(10)
                deadline = time.time() + 10
                while res["rx"] < 6000 and time.time() < deadline:
                    time.sleep(0.05)
                stop.set()
                a.close()
                rxt.join(5)
                b.close()
                rec = AR._RAW_EVIDENCE.finalize(res["rx"], res["tx"])
            finally:
                AR._RAW_EVIDENCE = None
            self.assertEqual(rec["rx_payload_bytes"], 6000)
            self.assertEqual(rec["tx_sent_bytes"], 6000)
            self.assertTrue(rec["tx_matches_reference"])
            self.assertEqual(rec["independent_compare"], "MATCH")
            self.assertEqual(rec["errors"], [])


if __name__ == "__main__":
    unittest.main()

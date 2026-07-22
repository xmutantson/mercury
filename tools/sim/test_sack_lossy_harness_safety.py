#!/usr/bin/env python3
"""Deterministic safety and integrity tests for the IONOS sweep harness."""

import importlib.util
import pathlib
import threading
import unittest
from unittest import mock


HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    "sack_lossy_ab", HERE / "sack_lossy_ab.py")
SLA = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SLA)


class _FakeSocket:
    def __init__(self, chunks):
        self._chunks = list(chunks)

    def settimeout(self, _timeout):
        pass

    def connect(self, _address):
        pass

    def recv(self, _size):
        return self._chunks.pop(0) if self._chunks else b""

    def close(self):
        pass


class HarnessSafetyTest(unittest.TestCase):
    def test_graceful_cleanup_has_twelve_second_term_only_window(self):
        seen = []

        def fake_send(_bs, command, t=0):
            seen.append((command, t))
            return "OK rc=0\nMERCURY_CLEAN"

        with mock.patch.object(SLA, "b_send", fake_send):
            ok, _reply = SLA.graceful_stop_mercury(object(), "lease", "rpi1")

        self.assertTrue(ok)
        command, timeout = seen[0]
        self.assertIn("killall -TERM mercury", command)
        self.assertIn('[ "$i" -lt 12 ]', command)
        self.assertNotIn("killall -9", command)
        self.assertNotIn("pkill -9", command)
        self.assertGreaterEqual(timeout, 25)

    def test_graceful_cleanup_fails_closed_on_survivor(self):
        with mock.patch.object(SLA, "b_send", return_value="OK rc=3\nMERCURY_SURVIVORS"):
            ok, reply = SLA.graceful_stop_mercury(
                object(), "lease", "rpi2", grace_s=1)
        self.assertFalse(ok)
        self.assertIn("SURVIVORS", reply)

    def test_stream_oracle_accepts_exact_payload_wrap(self):
        payload = b"Mercury-0123456789"
        expected = (payload * 4)[:57]
        chunks = [expected[:7], expected[7:31], expected[31:], b""]
        result = {}
        with mock.patch.object(SLA.socket, "socket", return_value=_FakeSocket(chunks)):
            SLA.receiver_thread("host", 1, result, threading.Event(), payload)
        self.assertEqual(result["bytes"], 57)
        self.assertEqual(result["mismatch_bytes"], 0)
        self.assertTrue(result["byte_exact"])
        self.assertEqual(result["received_sha256"], result["expected_sha256"])

    def test_stream_oracle_rejects_same_length_corruption(self):
        payload = b"Mercury-0123456789"
        received = bytearray((payload * 3)[:43])
        received[29] ^= 0x20
        result = {}
        chunks = [bytes(received[:13]), bytes(received[13:]), b""]
        with mock.patch.object(SLA.socket, "socket", return_value=_FakeSocket(chunks)):
            SLA.receiver_thread("host", 1, result, threading.Event(), payload)
        self.assertEqual(result["bytes"], 43)
        self.assertEqual(result["mismatch_bytes"], 1)
        self.assertEqual(result["first_mismatch_offset"], 29)
        self.assertFalse(result["byte_exact"])
        self.assertNotEqual(result["received_sha256"], result["expected_sha256"])

    def test_corrupt_throughput_is_not_a_valid_cell(self):
        self.assertEqual(SLA.valid_run_bps({"bps": 123.0, "byte_exact": True}),
                         123.0)
        self.assertIsNone(SLA.valid_run_bps(
            {"bps": 123.0, "byte_exact": False, "mismatch_bytes": 1}))
        self.assertIsNone(SLA.valid_run_bps({"bps": 0, "byte_exact": True}))


if __name__ == "__main__":
    unittest.main()

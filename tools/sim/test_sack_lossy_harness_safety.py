#!/usr/bin/env python3
"""Deterministic safety and integrity tests for the IONOS sweep harness."""

import importlib.util
import json
import pathlib
import tempfile
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

    def test_deploy_attestation_rejects_dirty_or_wrong_source(self):
        attestation = {
            "md5": "1" * 32,
            "sha256": "2" * 64,
            "src_git_rev": "a" * 40,
            "src_git_dirty": False,
        }
        with tempfile.TemporaryDirectory() as td:
            path = pathlib.Path(td) / "attestation.json"
            path.write_text(json.dumps(attestation), encoding="utf-8")
            loaded = SLA.load_deploy_attestation(
                path, {"git_rev": "a" * 40, "git_dirty": False})
            self.assertEqual(loaded["sha256"], "2" * 64)
            with self.assertRaises(RuntimeError):
                SLA.load_deploy_attestation(
                    path, {"git_rev": "a" * 40, "git_dirty": True})
            with self.assertRaises(RuntimeError):
                SLA.load_deploy_attestation(
                    path, {"git_rev": "b" * 40, "git_dirty": False})

    def test_both_remote_binary_hashes_must_match(self):
        expected = {"md5": "1" * 32, "sha256": "2" * 64}

        def good_send(_bs, command, t=0):
            value = expected["sha256"] if "sha256sum" in command else expected["md5"]
            return f"OK rc=0\n{value}  /home/pi/mercury-dev/mercury"

        with mock.patch.object(SLA, "b_send", good_send):
            observed = SLA.attest_remote_binaries(object(), "lease", expected)
        self.assertEqual(observed["rpi1"]["sha256"], expected["sha256"])
        self.assertEqual(observed["rpi2"]["md5"], expected["md5"])

        def one_bad_send(_bs, command, t=0):
            if "rpi2" in command and "sha256sum" in command:
                return f"OK rc=0\n{'3' * 64}  /home/pi/mercury-dev/mercury"
            return good_send(_bs, command, t)

        with mock.patch.object(SLA, "b_send", one_bad_send), \
                self.assertRaises(RuntimeError):
            SLA.attest_remote_binaries(object(), "lease", expected)

    def test_runtime_environment_is_sorted_symmetric_and_shell_safe(self):
        runtime_env = SLA.normalize_runtime_env([
            "MERCURY_SCALABLE_SACK=1",
            "MERCURY_PILOT_TARGET_CFG=15",
            "MERCURY_NOTE=value with space",
        ])
        self.assertEqual(list(runtime_env), sorted(runtime_env))
        rendered = SLA.runtime_env_shell(runtime_env)
        self.assertIn("MERCURY_PILOT_TARGET_CFG=15", rendered)
        self.assertIn("MERCURY_SCALABLE_SACK=1", rendered)
        self.assertIn("MERCURY_NOTE='value with space'", rendered)
        with self.assertRaises(ValueError):
            SLA.normalize_runtime_env(["bad-key=1"])
        with self.assertRaises(ValueError):
            SLA.normalize_runtime_env(["MERCURY_X=1", "MERCURY_X=2"])


if __name__ == "__main__":
    unittest.main()

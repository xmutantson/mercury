#!/usr/bin/env python3
"""Offline checks for card-scoped, graceful-only cleanup."""

import signal
import unittest
from unittest import mock

import ra_cleanup


class FakeProcess:
    def __init__(self, pid=123, survives=False):
        self.pid = pid
        self.survives = survives
        self.terminated = False

    def poll(self):
        return None

    def terminate(self):
        self.terminated = True

    def wait(self, timeout):
        if self.survives:
            raise TimeoutError("still alive")
        return 0


class GracefulCleanupTests(unittest.TestCase):
    def test_popen_cleanup_has_no_hard_kill_path(self):
        exited = FakeProcess()
        stuck = FakeProcess(pid=124, survives=True)
        survivors = ra_cleanup.terminate_processes_gracefully(
            [exited, stuck], timeout=10)
        self.assertTrue(exited.terminated)
        self.assertTrue(stuck.terminated)
        self.assertEqual(survivors, [stuck])
        self.assertFalse(hasattr(exited, "kill"))

    @mock.patch.object(ra_cleanup.time, "sleep")
    @mock.patch.object(ra_cleanup, "_wait_for_pids", return_value=[])
    @mock.patch.object(ra_cleanup, "_cmdline")
    @mock.patch.object(ra_cleanup, "_iter_pids", return_value=iter([10, 20]))
    @mock.patch.object(ra_cleanup, "_own_pids", return_value={10})
    @mock.patch.object(ra_cleanup.os, "kill")
    def test_scoped_cleanup_sends_sigterm_only(
            self, kill, own_pids, iter_pids, cmdline, wait_for_pids, sleep):
        cmdline.return_value = (
            "mercury -m ARQ -p 21000 -i hw:Loopback,1,0")
        signalled = ra_cleanup.scoped_cleanup(
            "Loopback", [0, 1, 2, 3], [21000, 21004], settle=0)
        self.assertEqual([pid for pid, _ in signalled], [20])
        kill.assert_called_once_with(20, signal.SIGTERM)
        wait_for_pids.assert_called_once_with([20], timeout=10.0)

    @mock.patch.object(ra_cleanup.time, "sleep")
    @mock.patch.object(ra_cleanup, "_wait_for_pids", return_value=[20])
    @mock.patch.object(ra_cleanup, "_cmdline", return_value=(
        "realaudio_bridge_s32.py --fwd-cap hw:Loopback,1,0"))
    @mock.patch.object(ra_cleanup, "_iter_pids", return_value=iter([20]))
    @mock.patch.object(ra_cleanup, "_own_pids", return_value=set())
    @mock.patch.object(ra_cleanup.os, "kill")
    def test_survivor_fails_closed_without_sigkill(
            self, kill, own_pids, iter_pids, cmdline, wait_for_pids, sleep):
        with self.assertRaisesRegex(RuntimeError, "survived graceful cleanup"):
            ra_cleanup.scoped_cleanup(
                "Loopback", [0, 1, 2, 3], [21000, 21004], settle=0)
        kill.assert_called_once_with(20, signal.SIGTERM)


if __name__ == "__main__":
    unittest.main()

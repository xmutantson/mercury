#!/usr/bin/env python3
"""Directed tests for the wb arm bandwidth flags (ticket #243).

In ARQ mode -W alone is overwritten at start-up (the session starts NB unless -Q 0 with
automatic bandwidth), so the wb arm must add -Q 0.

Run against another harness directory with HARNESS_DIR=<dir> (fail-before check).
"""
import os
import sys
import unittest

HARNESS_DIR = os.environ.get("HARNESS_DIR") or os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HARNESS_DIR)
import arq_realaudio as AR  # noqa: E402


class WbArmStartsWideband(unittest.TestCase):
    def test_wb_passes_probe_zero(self):
        c = AR.modem_bandwidth_args("wb", False)
        self.assertIn("-W", c)
        self.assertEqual(c[c.index("-Q") + 1], "0")

    def test_auto_keeps_the_probe(self):
        self.assertEqual(AR.modem_bandwidth_args("auto", False), ["-M", "auto"])

    def test_auto_skip_probe(self):
        self.assertEqual(AR.modem_bandwidth_args("auto", True), ["-M", "auto", "-Q", "0"])

    def test_nb(self):
        self.assertEqual(AR.modem_bandwidth_args("nb", False), ["-M", "nb"])


if __name__ == "__main__":
    unittest.main(verbosity=2)

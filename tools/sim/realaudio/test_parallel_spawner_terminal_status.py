#!/usr/bin/env python3
"""Terminal safety-contract tests for the real-audio cohort spawner."""

import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import textwrap
import unittest
from unittest import mock

import parallel_spawner as spawner


_FAKE_HARNESS = r"""
import argparse
import json
import os
from pathlib import Path

ap = argparse.ArgumentParser(add_help=False)
ap.add_argument("--json", required=True)
ap.add_argument("--tag", required=True)
ap.add_argument("--snr3k", type=float)
args, _ = ap.parse_known_args()

mode = os.environ["SPAWNER_TERMINAL_TEST_MODE"]
print("fake-cell-raw-artifact", flush=True)
if mode == "missing":
    raise SystemExit(0)

path = Path(args.json)
if mode == "malformed":
    path.write_text("{not-json", encoding="utf-8")
    raise SystemExit(0)

result = {
    "tag": args.tag,
    "seed": 1,
    "connected": True,
    "delivered_full": True,
    "delivered_full_count_only": True,
    "rx_bytes": 1,
    "tx_bytes": 1,
    "byte_integrity_ok": True,
    "uniqueness_ok": True,
    "instrument_invalid": False,
    "vara_scoring_enabled": True,
    "snr3k": args.snr3k,
    "anatomy": {},
    "verdict": "OK",
}
if mode == "byte_integrity_false":
    result["byte_integrity_ok"] = False
    result["integrity_mismatch_bytes"] = 1
elif mode == "uniqueness_false":
    result["uniqueness_ok"] = False
elif mode == "instrument_invalid":
    result["instrument_invalid"] = True

path.write_text(json.dumps(result), encoding="utf-8")
"""


class ParallelSpawnerTerminalStatusTest(unittest.TestCase):
    def run_case(self, mode):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            fake_harness = root / "fake_arq_realaudio.py"
            fake_harness.write_text(
                textwrap.dedent(_FAKE_HARNESS), encoding="utf-8")
            logdir = root / "logs"
            out = root / "cohort.json"
            argv = [
                str(Path(spawner.__file__)),
                "--n", "1",
                "--bin", sys.executable,
                "--bridge", str(root / "unused_bridge.py"),
                "--secs", "1",
                "--payload", "1",
                "--start-cfg", "100",
                "--snr", "24",
                "--snr3k", "24",
                "--profile", "wgn",
                "--tag-prefix", "terminal_",
                "--logdir", str(logdir),
                "--out", str(out),
            ]
            captured_out = io.StringIO()
            captured_err = io.StringIO()
            with mock.patch.object(spawner, "HARNESS", str(fake_harness)), \
                    mock.patch.object(spawner, "scoped_cleanup_cells"), \
                    mock.patch.object(spawner, "_arq_cell_census",
                                      return_value=0), \
                    mock.patch.object(spawner.CA, "probe_binary",
                                      return_value={}), \
                    mock.patch.object(sys, "argv", argv), \
                    mock.patch.dict(os.environ, {
                        "SPAWNER_TERMINAL_TEST_MODE": mode,
                    }), \
                    contextlib.redirect_stdout(captured_out), \
                    contextlib.redirect_stderr(captured_err):
                rc = spawner.main()

            # Failure must never erase evidence. The summary and completion
            # marker are terminal artifacts, and child stdout stays raw.
            self.assertTrue(out.exists(), mode)
            self.assertTrue(Path(str(out) + ".done").exists(), mode)
            raw = logdir / "spawn_terminal_00.out"
            self.assertTrue(raw.exists(), mode)
            self.assertIn("fake-cell-raw-artifact",
                          raw.read_text(encoding="utf-8"), mode)
            summary = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(1, len(summary["per_run"]), mode)

            result_path = logdir / "res_terminal_00.json"
            if mode == "malformed":
                self.assertEqual("{not-json",
                                 result_path.read_text(encoding="utf-8"))
                self.assertEqual("MISSING_RESULT",
                                 summary["per_run"][0]["verdict"])
            elif mode == "missing":
                self.assertFalse(result_path.exists())
                self.assertEqual("MISSING_RESULT",
                                 summary["per_run"][0]["verdict"])
            else:
                self.assertTrue(result_path.exists(), mode)
            return rc

    def test_valid_cell_exits_zero(self):
        self.assertEqual(0, self.run_case("valid"))

    def test_safety_failures_exit_nonzero_after_writing_artifacts(self):
        for mode in (
                "byte_integrity_false",
                "uniqueness_false",
                "missing",
                "malformed",
                "instrument_invalid"):
            with self.subTest(mode=mode):
                self.assertNotEqual(0, self.run_case(mode))


if __name__ == "__main__":
    unittest.main()

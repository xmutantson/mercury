#!/usr/bin/env python3
"""Spawn-smoke both supported real-audio bridge launch paths."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arq_realaudio as arq


class BridgeStartupSmokeTest(unittest.TestCase):
    def assert_starts(self, command, marker):
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True)
        output, _ = process.communicate(timeout=10)
        self.assertEqual(process.returncode, 0, output)
        self.assertIn(marker, output)

    def test_python_bridge_help_equivalent_starts(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge = Path(directory) / "probe_bridge.py"
            bridge.write_text(
                "import sys\n"
                "if sys.argv[1:] == ['--help']:\n"
                "    print('PYTHON_BRIDGE_STARTED')\n"
                "    raise SystemExit(0)\n"
                "raise SystemExit(2)\n",
                encoding="utf-8")

            prefix = arq.bridge_command_prefix(str(bridge))
            self.assertEqual(prefix[0], sys.executable)
            self.assert_starts(
                prefix + ["--help"], "PYTHON_BRIDGE_STARTED")

    def test_native_elf_bridge_help_starts(self):
        native = Path(sys.executable).resolve()
        with native.open("rb") as stream:
            self.assertEqual(stream.read(4), b"\x7fELF")

        prefix = arq.bridge_command_prefix(str(native))
        self.assertEqual(prefix, [str(native)])
        self.assert_starts(prefix + ["--help"], "usage:")


if __name__ == "__main__":
    unittest.main()

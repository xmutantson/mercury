#!/usr/bin/env python3
"""Unit tests for native-first real-audio bridge selection."""
import io
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import parallel_spawner as spawner
import arq_realaudio as arq


class BridgeDefaultSelectionTest(unittest.TestCase):
    def test_native_binary_present(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            native = root / "realaudio_bridge_s32_c"
            python = root / "realaudio_bridge_s32.py"
            native.touch()
            python.touch()

            selected = spawner.default_bridge_path(str(native), str(python))

            self.assertEqual(selected, str(native))

    def test_native_binary_absent_warns_and_falls_back(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            native = root / "realaudio_bridge_s32_c"
            python = root / "realaudio_bridge_s32.py"
            python.touch()
            warning = io.StringIO()

            selected = spawner.default_bridge_path(str(native), str(python))
            spawner.warn_for_python_fallback(
                selected, str(native), str(python), stream=warning)

            self.assertEqual(selected, str(python))
            self.assertIn("WARNING: native bridge is absent", warning.getvalue())

    def test_native_invocation_uses_winning_nice_level(self):
        with tempfile.TemporaryDirectory() as directory:
            native = Path(directory) / "realaudio_bridge_s32_c"
            native.touch(mode=0o755)

            self.assertEqual(
                arq.bridge_command_prefix(str(native)),
                ["sudo", "-n", "/usr/bin/nice", "-n", "-5", str(native)])


if __name__ == "__main__":
    unittest.main()

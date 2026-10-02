"""Execute build.sh's dependency check without invoking a compiler or modem.

Set MERCURY_TEST_BASH to an MSYS bash executable when running on Windows.
Otherwise the tests use bash from PATH. All dependencies and mtimes are local
temporary fixtures; only needs_rebuild() is extracted from the build script.
"""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = "source/common/rro_telemetry.cc"
OBJECT = "build/source/common/rro_telemetry.o"
DEPENDENCIES = "build/source/common/rro_telemetry.d"
BUILD_ID = "include/common/build_id.h"
OTHER_HEADER = "include/common/other.h"


class BuildDependencyTrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bash = os.environ.get("MERCURY_TEST_BASH") or shutil.which("bash")
        if not cls.bash:
            raise RuntimeError("bash is required; set MERCURY_TEST_BASH on Windows")
        source = (ROOT / "build.sh").read_text(encoding="utf-8")
        match = re.search(r"(?m)^needs_rebuild\(\)\s*\{\n[\s\S]*?^\}", source)
        if not match:
            raise AssertionError("build.sh needs_rebuild() function was not found")
        cls.function = match.group()

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="mercury-dependency-test-")
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        # Whole seconds and a wide gap avoid filesystem timestamp granularity.
        self.write_file(SOURCE, 1000)
        self.write_file(OTHER_HEADER, 1000)
        self.write_file(BUILD_ID, 1000)
        self.write_file(OBJECT, 2000)

    def write_file(self, name, mtime):
        path = self.directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"dependency fixture\n")
        os.utime(path, (mtime, mtime))

    def write_dependencies(self, lines, newline):
        (self.directory / DEPENDENCIES).write_bytes(
            (newline.join(lines) + newline).encode("utf-8"))

    def run_check(self, expected, function=None):
        env = os.environ.copy()
        env.pop("BASH_ENV", None)
        env.pop("ENV", None)
        result = subprocess.run(
            [self.bash, "--noprofile", "--norc", "-s", "--", SOURCE, OBJECT],
            input=(function or self.function) + '\nneeds_rebuild "$1" "$2"\n',
            cwd=self.directory, env=env, capture_output=True, text=True,
            timeout=10, check=False)
        self.assertEqual(result.returncode, expected,
                         f"stdout={result.stdout!r}; stderr={result.stderr!r}")

    def test_changed_final_build_id_header_lf_and_crlf(self):
        self.write_file(BUILD_ID, 3000)
        for newline in ("\n", "\r\n"):
            with self.subTest(newline=repr(newline)):
                self.write_dependencies(
                    [f"{OBJECT}: {SOURCE} {OTHER_HEADER} {BUILD_ID}"], newline)
                self.run_check(0)

    def test_changed_header_after_continuations_lf_and_crlf(self):
        self.write_file(BUILD_ID, 3000)
        for newline in ("\n", "\r\n"):
            with self.subTest(newline=repr(newline)):
                self.write_dependencies(
                    [f"{OBJECT}: {SOURCE} \\", f" {OTHER_HEADER} \\",
                     f" {BUILD_ID}"], newline)
                self.run_check(0)

    def test_unchanged_dependencies_lf_and_crlf(self):
        for newline in ("\n", "\r\n"):
            with self.subTest(newline=repr(newline)):
                self.write_dependencies(
                    [f"{OBJECT}: {SOURCE} \\", f" {OTHER_HEADER} {BUILD_ID}"],
                    newline)
                self.run_check(1)

    def test_missing_dependency_file_rebuilds(self):
        self.run_check(0)

    def test_missing_object_rebuilds(self):
        (self.directory / OBJECT).unlink()
        self.run_check(0)

    def test_newer_source_rebuilds(self):
        self.write_file(SOURCE, 3000)
        self.write_dependencies([f"{OBJECT}: {SOURCE} {BUILD_ID}"], "\r\n")
        self.run_check(0)

    def test_crlf_regression_without_normalization_misses_changed_header(self):
        self.write_file(BUILD_ID, 3000)
        self.write_dependencies([f"{OBJECT}: {SOURCE} {BUILD_ID}"], "\r\n")
        legacy = self.function.replace('            line="${line%$\'\\r\'}"\n', "", 1)
        self.assertNotEqual(legacy, self.function,
                            "the localized CRLF normalization must be present")
        self.run_check(1, function=legacy)
        self.run_check(0)


if __name__ == "__main__":
    unittest.main()

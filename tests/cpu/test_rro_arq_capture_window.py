"""Source-contract regression for the production ARQ capture-window handoff.

This checks the real receive() extraction and disabled guard without launching a
modem or manufacturing a telemetry event. Packet/hook unit tests remain separate
from production SIM coverage.
"""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[2]
ARQ_SOURCE = ROOT / "source/datalink_layer/arq_common.cc"


def cpp_code(source):
    """Remove comments and literals so braces/calls in text cannot pass checks."""
    return re.sub(
        r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        lambda match: "\n" * match.group().count("\n"), source)


def block_after(source, pattern):
    match = re.search(pattern, source)
    assert match is not None, f"missing source anchor: {pattern}"
    start = source.index("{", match.start())
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if depth == 0:
            return source[start + 1:end]
    raise AssertionError("unterminated source block")


def assert_production_handoff(source):
    code = cpp_code(source)
    body = block_after(code, r"void\s+cl_arq_controller::receive\s*\(\s*\)\s*\{")
    fresh = block_after(body, r"if\s*\(\s*telecom_system->data_container\."
                       r"frames_to_read\s*==\s*0\s*\)\s*\{")
    assert body.count("record_capture_window_handoff") == 1
    assert fresh.count("record_capture_window_handoff") == 1
    compact = re.sub(r"\s+", "", body)
    assert ("intsignal_period=telecom_system->data_container.Nofdm*"
            "telecom_system->data_container.buffer_Nsymb*"
            "telecom_system->data_container.interpolation_rate;") in compact
    copy = re.search(
        r"memcpy\s*\(\s*telecom_system->data_container\."
        r"ready_to_process_passband_delayed_data\s*,\s*"
        r"&telecom_system->data_container\.passband_delayed_data\s*"
        r"\[\s*rwi\s*\]\s*,\s*signal_period\s*\*\s*"
        r"sizeof\s*\(\s*double\s*\)\s*\)\s*;", fresh)
    assert copy is not None, "handoff must describe the actual window copy"
    guard = re.search(
        r"if\s*\(\s*signal_period\s*>\s*0\s*&&\s*"
        r"rro::Telemetry::instance\s*\(\s*\)\.enabled\s*\(\s*\)\s*\)\s*"
        r"rro::Telemetry::instance\s*\(\s*\)\.record_capture_window_handoff"
        r"\s*\(\s*signal_period\s*\)\s*;", fresh)
    assert guard is not None, "disabled path must skip only the observation"
    assert copy.end() < guard.start(), "cannot observe before the extraction"
    unlocks = list(re.finditer(
        r"MUTEX_UNLOCK\s*\(\s*&capture_prep_mutex\s*\)\s*;", fresh))
    preceding = [unlock for unlock in unlocks if unlock.end() <= guard.start()]
    assert preceding, "observation must follow capture mutex release"
    assert copy.end() < preceding[-1].start()
    assert fresh[preceding[-1].end():guard.start()].strip() == ""


class ProductionArqCaptureWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = ARQ_SOURCE.read_text(encoding="utf-8")

    def test_actual_production_extraction_once_after_unlock(self):
        assert_production_handoff(self.source)

    def test_disabled_guard_cannot_be_removed(self):
        changed = self.source.replace(
            "signal_period > 0 && rro::Telemetry::instance().enabled()",
            "signal_period > 0", 1)
        self.assertNotEqual(changed, self.source)
        with self.assertRaises(AssertionError):
            assert_production_handoff(changed)

    def test_bytes_cannot_be_reported_as_samples(self):
        changed = self.source.replace(
            "record_capture_window_handoff(signal_period);",
            "record_capture_window_handoff(signal_period * sizeof(double));", 1)
        self.assertNotEqual(changed, self.source)
        with self.assertRaises(AssertionError):
            assert_production_handoff(changed)

    def test_stale_window_cannot_count_as_a_fresh_extraction(self):
        changed = self.source.replace(
            "void cl_arq_controller::receive()\n{",
            "void cl_arq_controller::receive()\n{\n"
            "rro::Telemetry::instance().record_capture_window_handoff(1);", 1)
        self.assertNotEqual(changed, self.source)
        with self.assertRaises(AssertionError):
            assert_production_handoff(changed)

    def test_comment_only_hook_does_not_pass(self):
        changed = self.source.replace(
            "rro::Telemetry::instance().record_capture_window_handoff(signal_period);",
            "// rro::Telemetry::instance().record_capture_window_handoff(signal_period);",
            1)
        self.assertNotEqual(changed, self.source)
        with self.assertRaises(AssertionError):
            assert_production_handoff(changed)


if __name__ == "__main__":
    unittest.main()

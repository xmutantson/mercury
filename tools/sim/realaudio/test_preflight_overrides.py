#!/usr/bin/env python3
"""Offline tests for preflight_overrides.py and the harness adapter override_guard.py."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import preflight_overrides as po  # noqa: E402
import override_guard  # noqa: E402

SEAM = "MERCURY_CTL_GAIN_SELECTOR_DEFEAT"


class RegistryTest(unittest.TestCase):
    def test_registry_loads_and_names_the_ctl_gain_seam(self):
        reg = po.load_registry()
        self.assertIn(SEAM, reg)
        self.assertEqual(reg[SEAM]["status"], "verified")
        self.assertGreater(len(reg), 50, "census by git grep, not a hand list")
        self.assertTrue(all(e["status"] in ("verified", "[?]") for e in reg.values()))

    def test_clean_env(self):
        self.assertEqual(po.check_env({"MERCURY_GEARSHIFT_V2": "active"})["status"], "CLEAN")

    def test_inert_value_is_clean(self):
        self.assertEqual(po.check_env({SEAM: "0"})["status"], "CLEAN")
        self.assertEqual(po.check_env({SEAM: ""})["status"], "CLEAN")

    def test_active_override_rejects(self):
        r = po.check_env({SEAM: "1"})
        self.assertEqual(r["status"], "REJECT")

    def test_unverified_entry_rejects_on_presence(self):
        reg = po.load_registry()
        unknown = next(n for n, e in reg.items() if e["status"] == "[?]")
        self.assertEqual(po.check_env({unknown: "0"})["status"], "REJECT")

    def test_declared_negative_control(self):
        r = po.check_env({SEAM: "1"}, negative_control=SEAM)
        self.assertEqual(r["status"], "NEGATIVE-CONTROL")
        r2 = po.check_env({SEAM: "1", "NEGATIVE_CONTROL": SEAM})
        self.assertEqual(r2["status"], "NEGATIVE-CONTROL")

    def test_negative_control_does_not_cover_a_second_override(self):
        reg = po.load_registry()
        other = next(n for n in reg if n != SEAM)
        r = po.check_env({SEAM: "1", other: "1"}, negative_control=SEAM)
        self.assertEqual(r["status"], "REJECT")

    def test_declared_but_inactive_or_unregistered_rejects(self):
        self.assertEqual(po.check_env({}, negative_control=SEAM)["status"], "REJECT")
        self.assertEqual(po.check_env({"MERCURY_X": "1"}, negative_control="MERCURY_NOT_A_SWITCH")["status"], "REJECT")

    def test_cli_exit_codes(self):
        cli = [sys.executable, str(HERE / "preflight_overrides.py"), "check"]
        self.assertEqual(subprocess.run(cli + ["--env", "A=1"]).returncode, 0)
        self.assertEqual(subprocess.run(cli + ["--env", SEAM + "=1"]).returncode, 1)
        self.assertEqual(subprocess.run(cli + ["--env", SEAM + "=1", "--negative-control", SEAM]).returncode, 3)

    def test_harness_adapter_uses_the_registry(self):
        self.assertEqual(override_guard.check({SEAM: "1"})["status"], "REJECT")
        self.assertEqual(override_guard.check({SEAM: "1"}, SEAM)["status"], "NEGATIVE-CONTROL")
        self.assertEqual(override_guard.check({"PATH": "/bin"})["status"], "CLEAN")

    def test_harness_adapter_reports_unavailable_not_clean(self):
        old = os.environ.get("MERCURY_PREFLIGHT_REGISTRY")
        try:
            os.environ["MERCURY_PREFLIGHT_REGISTRY"] = "/nonexistent.json"
            orig = override_guard._registry_path
            override_guard._registry_path = lambda: None
            self.assertEqual(override_guard.check({SEAM: "1"})["status"], "UNAVAILABLE")
            override_guard._registry_path = orig
        finally:
            if old is None:
                os.environ.pop("MERCURY_PREFLIGHT_REGISTRY", None)
            else:
                os.environ["MERCURY_PREFLIGHT_REGISTRY"] = old


class DefaultOnLeverTest(unittest.TestCase):
    """ph_fix (refuter G10): a default-ON lever switched off with =0 is an override too; only =0 overrides it."""
    LEVER = "MERCURY_SCALABLE_SACK"   # arq_common.cc: return !(e && *e && atoi(e) == 0);  // ON unless explicitly disabled with =0

    def test_zero_levers_are_registered_from_the_census(self):
        reg = po.load_registry()
        levers = [e for e in reg.values() if e["kind"] == "default-on-lever"]
        self.assertGreaterEqual(len(levers), 30)
        self.assertTrue(all(e["active_values"] == ["0"] and e["status"] == "[?]" for e in levers))
        self.assertIn(self.LEVER, reg)

    def test_zero_rejects_other_values_clean(self):
        """expected values from the modem's read (the requirement), not from this tool: `e && *e && atoi(e) == 0` switches the
        lever off, so every non-empty value whose C atoi() is 0 is an override (ph_refute2 G10: "yes" was wrongly CLEAN)"""
        for v in ("0", " 0", "00", "-0", "+0", "0 ", "\t0", "false", "off", "no", "yes", "x", "0x1", "-"):
            self.assertEqual(po.check_env({self.LEVER: v})["status"], "REJECT", repr(v))
        for v in ("1", "", " 7", "-1", "1abc", "+2"):
            self.assertEqual(po.check_env({self.LEVER: v})["status"], "CLEAN", repr(v))

    def test_each_lever_uses_its_own_read(self):
        """the off-rule comes from each lever's read in the modem (off_rule_source), e.g. a mode selector or an exact strcmp"""
        reg = po.load_registry()
        cases = [("MERCURY_GEARSHIFT_V2", {"legacy": True, "0": True, "shadow": True, "junk": True, "active": False, "1": False}),
                 ("MERCURY_CHASE", {"0": True, "00": False, "no": False, "1": False}),
                 ("MERCURY_DFTSMOOTH", {"legacy": True, "0": True, "off": False}),
                 ("MERCURY_TURNAROUND_GUARD_SCOPE_ALL", {"0": True, "0abc": True, " 0": False, "1": False}),
                 ("MERCURY_KEYDOWN_TRACK", {"": True, "0": True, "no": True, "1": False}),
                 ("MERCURY_GEN_CANON", {"": False, "0": True, "false": True, "2": False})]
        for name, want in cases:
            self.assertIn(name, reg)
            for v, off in want.items():
                self.assertEqual(po.active(reg[name], v), off, (name, v))
        self.assertTrue(all(e.get("off_rule") and e.get("off_rule_source") for e in reg.values() if e["kind"] == "default-on-lever"))

    def test_c_atoi_matches_c(self):
        for s, want in (("0", 0), ("  42x", 42), ("-3", -3), ("+5", 5), ("yes", 0), ("", 0), ("0x10", 0), ("\n\t9", 9), ("- 1", 0)):
            self.assertEqual(po.c_atoi(s), want, repr(s))

    # The expected values are glibc's own atoi() outputs, measured through ctypes.
    GLIBC = (("4294967296", 0), ("-4294967296", 0), ("8589934592", 0), ("4294967297", 1), ("99999999999999999999", -1),
             ("+0", 0), ("0x0", 0), (" 1", 1), ("1e0", 1), ("１", 0), ("０", 0), ("00", 0), ("-0", 0), ("yes", 0), ("0 ", 0),
             ("\t0", 0), ("2147483648", -2147483648), ("18446744073709551616", -1), ("-4294967295", 1))

    def test_c_atoi_matches_glibc_including_32_bit_wrap(self):
        for s, want in self.GLIBC:
            self.assertEqual(po.c_atoi(s), want, repr(s))

    def test_lever_set_to_a_multiple_of_2_32_is_an_override(self):          # G-atoi fail-before: CLEAN
        for v in ("4294967296", "-4294967296", "8589934592"):
            r = po.check_env({"MERCURY_BIGBLOCK_AFCTRACK": v})
            self.assertEqual(r["status"], "REJECT", v)
        self.assertEqual(po.check_env({"MERCURY_BIGBLOCK_AFCTRACK": "4294967297"})["status"], "CLEAN")

    def test_declared_negative_control_on_a_lever(self):
        self.assertEqual(po.check_env({self.LEVER: "0"}, negative_control=self.LEVER)["status"], "NEGATIVE-CONTROL")

    def test_values_compare_stripped(self):   # refuter G9 '0 ' case: the inert value with trailing space stays inert
        self.assertEqual(po.check_env({SEAM: "0 "})["status"], "CLEAN")
        self.assertEqual(po.check_env({SEAM: " 1"})["status"], "REJECT")


if __name__ == "__main__":
    unittest.main()

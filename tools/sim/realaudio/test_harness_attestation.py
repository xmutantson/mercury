#!/usr/bin/env python3
"""Tests for the harness identity attestation and the [TxGain] case-B path.

Pure-Python tests run anywhere. The bridge tests run the native bridge in
--dry-run mode (no audio devices); on Linux a missing bridge binary is a
failure, not a skip. Point them at a build with:

    python3 test_harness_attestation.py --bin ./realaudio_bridge_s32_c
"""
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import harness_attestation as HA  # noqa: E402

BRIDGE_BIN = os.path.join(HERE, "realaudio_bridge_s32_c")
REFERENCE_INI = os.path.join(HERE, "txgain_bench_7_13_21.ini")


def fake_modem_log(labels=("RSP", "CMD"), overrides=None, ini_db=0.0):
    """A harness log as log_output writes it, with the modem's own print formats."""
    names = {"MFSK_1S": "MFSK_1S", "MFSK_2S": "MFSK_2S", "OFDM": "OFDM   ",
             "ACK": "ACK    ", "BREAK": "BREAK  "}
    lines = []
    t = 0.0
    for label in labels:
        for key, (prev, new) in sorted((overrides or {}).items()):
            sig, mode = key.rsplit("_", 1)
            t += 0.001
            lines.append("[T+%08.3f] [%s] [TX-GAIN-OVERRIDE] %s  %s  %.4f -> %.4f "
                         "(calibration override, plan \u00a77.13.21)"
                         % (t, label, names[sig], mode, prev, new))
        t += 0.001
        lines.append("[T+%08.3f] [%s] [TX-GAIN] INI: %.1f dB  [RX-GAIN] INI: 0.0 dB"
                     % (t, label, ini_db))
    return "\n".join(lines) + "\n"


BENCH = {"MFSK_1S_WB": 3.62, "ACK_WB": 3.62, "BREAK_WB": 3.62,
         "MFSK_1S_NB": 8.09, "ACK_NB": 8.09, "BREAK_NB": 8.09}
DEFAULTS = {"MFSK_1S_WB": 5.24, "ACK_WB": 5.24, "BREAK_WB": 5.24,
            "MFSK_1S_NB": 5.24 * math.sqrt(5.0), "ACK_NB": 5.24 * math.sqrt(5.0),
            "BREAK_NB": 5.24 * math.sqrt(5.0)}


class TestIdentity(unittest.TestCase):
    def test_sha256_file(self):
        with tempfile.NamedTemporaryFile(delete=False) as handle:
            handle.write(b"attestation")
        try:
            self.assertEqual(HA.sha256_file(handle.name),
                             hashlib.sha256(b"attestation").hexdigest())
        finally:
            os.unlink(handle.name)

    def test_identity_and_verify(self):
        with tempfile.NamedTemporaryFile(delete=False) as handle:
            handle.write(b"bridge")
        try:
            ident = HA.identity(handle.name, __file__, "canon")
            self.assertEqual(ident["bridge_sha256"],
                             hashlib.sha256(b"bridge").hexdigest())
            self.assertEqual(ident["harness_sha256"], HA.sha256_file(__file__))
            self.assertEqual(HA.verify_identity(dict(ident), ident), [])
            bad = dict(ident, bridge_sha256="0" * 64)
            self.assertEqual(HA.verify_identity(bad, ident),
                             ["bridge_sha256_mismatch"])
            self.assertIn("harness_lineage_mismatch",
                          HA.verify_identity(dict(ident, harness_lineage=""), ident))
        finally:
            os.unlink(handle.name)

    def test_lineage_fits_bridge_field(self):
        # The bridge stores the lineage in char[32].
        with self.assertRaises(ValueError):
            HA.identity(__file__, __file__, "x" * 32)
        with self.assertRaises(ValueError):
            HA.identity(__file__, __file__, "")

    def test_py_bridge_gets_no_identity_options(self):
        self.assertFalse(HA.bridge_accepts_identity("/x/realaudio_bridge_s32.py"))
        self.assertTrue(HA.bridge_accepts_identity("/x/realaudio_bridge_s32_c"))


class TestTxGainIni(unittest.TestCase):
    def write(self, text):
        handle = tempfile.NamedTemporaryFile("w", suffix=".ini", delete=False)
        handle.write(text)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        return handle.name

    def test_reference_ini_carries_the_bench_values(self):
        values = HA.read_tx_gain_ini(REFERENCE_INI)
        self.assertEqual(values, BENCH)
        # 3.62 / 5.24 = 0.6908 on WB (the plan quotes 0.690x); NB carries the
        # same factor on its own default (8.09 / 11.717 = 0.6904).
        self.assertAlmostEqual(values["MFSK_1S_WB"] / 5.24, 0.690, delta=1e-3)
        self.assertAlmostEqual(values["MFSK_1S_NB"] / DEFAULTS["MFSK_1S_NB"],
                               0.690, delta=1e-3)

    def test_key_set_mirrors_the_modem_loader(self):
        self.assertEqual(len(HA.TX_GAIN_KEYS), 10)
        self.assertIn("OFDM_WB", HA.TX_GAIN_KEYS)
        self.assertIn("BREAK_NB", HA.TX_GAIN_KEYS)

    def test_other_sections_are_ignored_like_the_pi_ini(self):
        path = self.write("[GUI]\nTxGainDb=0.0\n[TxGain]\n ACK_WB = 3.62 \n; c\n# c\n")
        self.assertEqual(HA.read_tx_gain_ini(path), {"ACK_WB": 3.62})

    def test_fail_closed_inputs(self):
        for text in ("[GUI]\nTxGainDb=1\n",            # no section
                     "[TxGain]\n",                      # empty section
                     "[TxGain]\nHAIL_WB=3.62\n",        # key the modem never reads
                     "[TxGain]\nack_wb=3.62\n",         # keys are case sensitive
                     "[TxGain]\nACK_WB=abc\n",
                     "[TxGain]\nACK_WB=0\n",
                     "[TxGain]\nACK_WB=nan\n"):
            with self.assertRaises(HA.TxGainError, msg=text):
                HA.read_tx_gain_ini(self.write(text))

    def test_settings_file_round_trip(self):
        with tempfile.TemporaryDirectory() as home:
            path = HA.write_modem_settings(home, BENCH)
            self.assertEqual(path, os.path.join(home, ".config", "mercury",
                                                "mercury.ini"))
            with open(path) as handle:
                text = handle.read()
            parsed = HA.parse_ini_text(text)
            self.assertEqual(list(parsed), ["TxGain"])
            self.assertEqual({k: float(v) for k, v in parsed["TxGain"].items()},
                             BENCH)

    def test_resolve_precedence(self):
        env = {HA.TX_GAIN_ENV: "/env.ini"}
        self.assertEqual(HA.resolve_tx_gain_ini("/cli.ini", env), ("/cli.ini", "cli"))
        self.assertEqual(HA.resolve_tx_gain_ini(None, env), ("/env.ini", "env"))
        self.assertEqual(HA.resolve_tx_gain_ini(None, {HA.TX_GAIN_ENV: ""}),
                         (None, None))
        self.assertEqual(HA.resolve_tx_gain_ini(None, {}), (None, None))


class TestTxGainLog(unittest.TestCase):
    def overrides(self, values):
        return {k: (DEFAULTS[k], v) for k, v in values.items()}

    def test_case_b_applied_on_both_peers(self):
        log = fake_modem_log(overrides=self.overrides(BENCH))
        att = HA.check_tx_gain_log(log, BENCH)
        self.assertTrue(att["ok"], att["reasons"])
        self.assertEqual(att["mode"], "ini")
        self.assertEqual(att["override_lines"], 12)
        self.assertEqual(att["applied"]["MFSK_1S_WB"]["RSP"]["applied_db"], -3.212)
        self.assertEqual(att["ini_db"], {"RSP": 0.0, "CMD": 0.0})

    def test_case_b_missing_on_one_peer_fails(self):
        log = (fake_modem_log(labels=("RSP",), overrides=self.overrides(BENCH))
               + fake_modem_log(labels=("CMD",)))
        att = HA.check_tx_gain_log(log, BENCH)
        self.assertFalse(att["ok"])
        self.assertIn("CMD:ACK_WB_override_missing", att["reasons"])

    def test_case_b_wrong_value_fails(self):
        wrong = dict(BENCH, ACK_WB=3.70)
        att = HA.check_tx_gain_log(fake_modem_log(overrides=self.overrides(wrong)), BENCH)
        self.assertFalse(att["ok"])
        self.assertTrue(any("ACK_WB_override_3.7000_expected_3.6200" in r
                            for r in att["reasons"]))

    def test_case_b_non_gui_build_prints_nothing_fails(self):
        att = HA.check_tx_gain_log("[T+0001.000] [RSP] link_status:Idle\n", BENCH)
        self.assertFalse(att["ok"])
        self.assertIn("RSP:tx_gain_ini_db_line_missing", att["reasons"])

    def test_default_path_clean(self):
        att = HA.check_tx_gain_log(fake_modem_log(), {})
        self.assertTrue(att["ok"], att["reasons"])
        self.assertEqual(att["mode"], "default")
        self.assertEqual(att["override_lines"], 0)
        self.assertEqual(att["ini_db"], {"RSP": 0.0, "CMD": 0.0})

    def test_default_path_flags_a_stray_home_ini(self):
        stray = fake_modem_log(overrides={"OFDM_WB": (1.0, 0.8)})
        att = HA.check_tx_gain_log(stray, {})
        self.assertFalse(att["ok"])
        self.assertIn("RSP:OFDM_WB_unexpected_override", att["reasons"])
        att = HA.check_tx_gain_log(fake_modem_log(ini_db=3.0), {})
        self.assertFalse(att["ok"])


@unittest.skipUnless(sys.platform.startswith("linux"), "native bridge is Linux-only")
class TestBridgeDryRun(unittest.TestCase):
    def dry_run(self, *extra):
        self.assertTrue(os.path.isfile(BRIDGE_BIN), "bridge not built: %s" % BRIDGE_BIN)
        with tempfile.TemporaryDirectory() as tmp:
            stats = os.path.join(tmp, "stats.json")
            proc = subprocess.run([BRIDGE_BIN, "--dry-run", "--snr3k-db", "10",
                                   "--statsfile", stats] + list(extra),
                                  capture_output=True)
            self.assertEqual(proc.returncode, 0, proc.stderr.decode())
            with open(stats) as handle:
                return json.load(handle)

    def test_identity_echoed(self):
        ident = HA.identity(BRIDGE_BIN, os.path.join(HERE, "arq_realaudio.py"),
                            "canon")
        stats = self.dry_run("--binary-sha256", "1" * 64,
                             *HA.bridge_identity_argv(ident))
        att = stats["axis_attestation"]
        self.assertEqual(HA.verify_identity(att, ident), [])
        self.assertEqual(att["bridge_sha256"], HA.sha256_file(BRIDGE_BIN))
        self.assertEqual(att["binary_sha256"], "1" * 64)

    def test_absent_identity_is_valid_json_with_zero_hashes(self):
        att = self.dry_run()["axis_attestation"]
        self.assertEqual(att["bridge_sha256"], "0" * 64)
        self.assertEqual(att["harness_sha256"], "0" * 64)
        self.assertEqual(att["harness_lineage"], "")

    def test_lineage_escaping(self):
        att = self.dry_run("--harness-lineage", 'a"b\\c\td')["axis_attestation"]
        self.assertEqual(att["harness_lineage"], 'a"b\\c\td')

    def test_overlong_values_are_truncated_not_overflowed(self):
        att = self.dry_run("--bridge-sha256", "f" * 200,
                           "--harness-lineage", "L" * 100)["axis_attestation"]
        self.assertEqual(att["bridge_sha256"], "f" * 79)
        self.assertEqual(att["harness_lineage"], "L" * 31)


if __name__ == "__main__":
    if "--bin" in sys.argv:
        index = sys.argv.index("--bin")
        BRIDGE_BIN = os.path.abspath(sys.argv[index + 1])
        del sys.argv[index:index + 2]
    unittest.main(verbosity=2)

#!/usr/bin/env python3
"""Rate-table attestation of a twin cell (ticket #446): the gearshift must run table-free
(four '[OPT] table not found' lines per peer, default search order) unless the recipe names a
table. HARNESS_DIR=<dir> selects the harness copy (fail-before check); BOARD_RAW=<dir> also
checks every retained board cell log."""
import glob
import os
import sys
import tempfile
import unittest

HARNESS_DIR = os.environ.get("HARNESS_DIR") or os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HARNESS_DIR)
import arq_realaudio as AR  # noqa: E402

PATHS = ["mercury/effective_rate_table.json", "effective_rate_table.json",
         "mercury/effective_rate_table.synthetic.json", "effective_rate_table.synthetic.json"]


def nf(peer, path, t=2.484):
    return "[T+%08.3f] [%s] [OPT] table not found (path=%s); Gearshift-v2 continues uncalibrated" % (t, peer, path)


def loaded(peer, path, band="WB"):
    return ("[T+0002.500] [%s] [OPT] %s calibration loaded: 12 configs x 4 channels "
            "(valid_cells=40, mode=full, path=%s)" % (peer, band, path))


def write(lines):
    fd, p = tempfile.mkstemp(suffix=".log")
    with os.fdopen(fd, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return p


class RateTableAttestation(unittest.TestCase):
    def check(self, lines, expect=None):
        p = write(lines)
        try:
            return AR.rate_table_attestation(p, expect)
        finally:
            os.remove(p)

    def test_table_free_cell_passes(self):
        r = self.check([nf("RSP", x) for x in PATHS] + [nf("CMD", x) for x in PATHS])
        self.assertTrue(r["ok"], r["reasons"])

    def test_planted_table_in_cell_dir_fails(self):
        r = self.check([nf("RSP", x) for x in PATHS]
                       + [nf("CMD", PATHS[0]), loaded("CMD", "effective_rate_table.json")])
        self.assertFalse(r["ok"])
        self.assertIn("CMD_not_table_free", r["reasons"])

    def test_env_table_path_fails(self):
        r = self.check([nf("RSP", "/tmp/x.json")] + [nf("RSP", x) for x in PATHS]
                       + [nf("CMD", x) for x in PATHS])
        self.assertEqual(r["reasons"], ["RSP_not_table_free"])

    def test_missing_lines_fail(self):
        r = self.check([nf("RSP", x) for x in PATHS])
        self.assertIn("CMD_not_table_free", r["reasons"])

    def test_named_table_passes_when_loaded(self):
        r = self.check([loaded("RSP", "/opt/t.json"), loaded("CMD", "/opt/t.json")], "/opt/t.json")
        self.assertTrue(r["ok"], r["reasons"])
        r = self.check([loaded("RSP", "/opt/t.json"), loaded("CMD", "/opt/u.json")], "/opt/t.json")
        self.assertEqual(r["reasons"], ["CMD_not_expected_table"])

    def test_signature_mismatch_fails(self):
        r = self.check([loaded("RSP", "/opt/t.json"), loaded("CMD", "/opt/t.json"),
                        "[T+0002.600] [CMD] [OPT] WARNING calibration config signature mismatch "
                        "table=a current=b; prior weight x0.50 sigma x2.00 probe-only"], "/opt/t.json")
        self.assertEqual(r["reasons"], ["CMD_not_expected_table"])

    @unittest.skipUnless(os.environ.get("BOARD_RAW"), "BOARD_RAW not set")
    def test_board_cells_are_table_free(self):
        logs = sorted(glob.glob(os.path.join(os.environ["BOARD_RAW"], "*_s[1-9]", "logs", "arq_*.log")))
        self.assertGreater(len(logs), 0)
        bad = [p for p in logs if not AR.rate_table_attestation(p)["ok"]]
        print("board cells checked: %d, not table-free: %d" % (len(logs), len(bad)))
        self.assertEqual(bad, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)

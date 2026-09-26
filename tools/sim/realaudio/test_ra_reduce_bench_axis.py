#!/usr/bin/env python3
"""The vs-bar gate names the axis the twin runs (bench), the frozen width, and
the frozen bar.

bench and steady are honest axes; peak and a missing axis are refused. The
default width limit is the frozen 8 per box (RECIPE_FREEZE_20260923 s3/s8);
--width-limit 24 admits the 2026-09-07 certified width and is recorded. A
bench-axis cell is compared with the RECIPE_FREEZE s2 bar derived from its own
profile and dial (snr3k + 0.35, column P), never with the stamped bar. Optional
census over real twin results: set RA_REDUCE_BOARD_RAW to a directory of
<cell>/result.json.
"""
import contextlib
import glob
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ra_reduce  # noqa: E402


def _cell(seed, psig_mode="bench", width=8, box=8, tag=None, profile="wgn",
          snr3k=19.65, stamped=26173.8):
    return {
        "tag": tag or "c%d" % seed, "arm": "A", "seed": seed, "snr3k": snr3k,
        "snr3k_controlled": snr3k, "profile": profile,
        "bin_md5": "m", "traffic": "random-binary", "connected": True,
        "rx_bytes": 1000, "payload_target": 1000, "good_prefix_bytes": 1000,
        "byte_integrity_ok": True, "uniqueness_ok": True,
        "delivered_full": True, "delivered_full_count_only": True,
        "wall_secs": 100.0, "connected_at_s": 10.0,
        "anatomy": {"ptt_fwd_airtime_s": 50.0, "ptt_duty": 0.5},
        "vara_bar_Bmin": stamped, "psig_mode": psig_mode,
        "spawner_width": width, "box_concurrent_estimate": box,
    }


class BenchAxisGate(unittest.TestCase):
    def setUp(self):
        self.dirs = []

    def tearDown(self):
        for d in self.dirs:
            shutil.rmtree(d, ignore_errors=True)

    def _run(self, cells, extra=()):
        d = tempfile.mkdtemp(prefix="ra_bench_")
        self.dirs.append(d)
        for i, c in enumerate(cells):
            with open(os.path.join(d, "res_%02d.json" % i), "w") as f:
                json.dump(c, f)
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = ra_reduce.main([d] + list(extra))
        return rc, out.getvalue(), err.getvalue()

    def test_bench_axis_is_honest(self):
        self.assertEqual(ra_reduce._axis_width_offense(
            {"psig_mode": "bench", "spawner_width": 8, "box_concurrent_estimate": 8}), [])

    def test_bench_cohort_scores_and_records_the_gate(self):
        rc, out, err = self._run([_cell(i) for i in range(1, 9)])
        self.assertEqual(rc, 0, err)
        red = json.loads(out)
        self.assertEqual(red["n_cells"], 8)
        gate = red["axis_width_gate"]
        self.assertIn("bench", gate["honest_axes"])
        self.assertEqual(gate["width_limit"], 8)
        self.assertEqual(gate["widths"], [["8", "8"]])

    def test_peak_still_refused(self):
        rc, out, err = self._run([_cell(1), _cell(2, psig_mode="peak", tag="pk")])
        self.assertEqual(rc, 2)
        self.assertIn("pk", err)

    def test_missing_axis_still_refused(self):
        c = _cell(2, tag="noaxis")
        del c["psig_mode"]
        rc, out, err = self._run([_cell(1), c])
        self.assertEqual(rc, 2)
        self.assertIn("psig_mode MISSING", err)

    def test_frozen_width_is_the_default(self):
        rc, _, err = self._run([_cell(1, width=12, box=12, tag="w12")])
        self.assertEqual(rc, 2)
        self.assertIn("spawner_width=12 > 8", err)
        rc, _, err = self._run([_cell(1, width=8, box=10, tag="b10")])
        self.assertEqual(rc, 2)
        self.assertIn("box_concurrent_estimate=10 > 8", err)

    def test_certified_width_only_on_request(self):
        rc, out, err = self._run([_cell(1, width=24, box=24)], ["--width-limit", "24"])
        self.assertEqual(rc, 0, err)
        self.assertEqual(json.loads(out)["axis_width_gate"]["width_limit"], 24)
        rc, _, err = self._run([_cell(1, width=25, box=25, tag="w25")], ["--width-limit", "24"])
        self.assertEqual(rc, 2)

    def test_bench_cell_uses_the_frozen_bar(self):
        for profile, bar in (("wgn", 38425.0), ("mpg", 23142.0), ("mpm", 23711.0), ("mpp", 16592.0)):
            row = ra_reduce.reduce_cell(_cell(1, profile=profile))
            self.assertEqual(row["vara_bar_Bmin"], bar, profile)
            self.assertEqual(row["vara_bar_source"], ra_reduce.FROZEN_BAR_ID)
            self.assertEqual(row["vara_bar_stamped_Bmin"], 26173.8)
            self.assertEqual(row["frozen_dial"], 20)
            self.assertAlmostEqual(row["vs_vara_whole"], round(10.0 * 60.0 / bar, 4))

    def test_off_grid_or_unknown_profile_has_no_bar(self):
        self.assertIsNone(ra_reduce.reduce_cell(_cell(1, snr3k=18.0))["vs_vara_whole"])
        self.assertIsNone(ra_reduce.reduce_cell(_cell(1, profile="xyz"))["vs_vara_whole"])

    def test_steady_cell_keeps_the_stamped_bar(self):
        row = ra_reduce.reduce_cell(_cell(1, psig_mode="steady", stamped=18232.0))
        self.assertEqual(row["vara_bar_Bmin"], 18232.0)
        self.assertEqual(row["vara_bar_source"], "stamped")

    def test_census_real_twin_results(self):
        raw = os.environ.get("RA_REDUCE_BOARD_RAW")
        if not raw:
            self.skipTest("RA_REDUCE_BOARD_RAW not set")
        files = sorted(glob.glob(os.path.join(raw, "*", "result.json")))
        self.assertGreater(len(files), 0)
        bench = 0
        barred = 0
        for f in files:
            d = json.load(open(f))
            if d.get("vara_bar_Bmin") is None:
                continue
            reasons = ra_reduce._axis_width_offense(d)
            self.assertEqual(reasons, [], "%s: %s" % (f, reasons))
            bench += d.get("psig_mode") == "bench"
            row = ra_reduce.reduce_cell(d)
            if row["vara_bar_source"] == ra_reduce.FROZEN_BAR_ID:
                barred += 1
                self.assertEqual(row["vara_bar_Bmin"],
                                 float(ra_reduce.FROZEN_BAR_BMIN[d["profile"].upper()][row["frozen_dial"]]))
        self.assertGreater(bench, 0)
        print("census: %d result files, %d bench-axis vs-bar cells, %d with a frozen bar, 0 refused"
              % (len(files), bench, barred))


if __name__ == "__main__":
    unittest.main()

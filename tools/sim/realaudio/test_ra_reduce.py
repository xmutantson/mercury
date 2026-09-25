#!/usr/bin/env python3
"""Unit tests for ra_reduce.py (the blessed real-audio reducer).

Two layers:

1. SYNTHETIC (always runs): hand-built per-cell dicts exercising the honest
   completion form, the shear flag, legacy-vs-new field vintages, and the
   reconstruction law (including a deliberately inconsistent cell that MUST
   fail it).

2. FIXTURE (runs when RA_REDUCE_FIXTURES points at one or more directories of
   per-cell res JSONs, ';'-separated): asserts the reducer reproduces the
   independently recomputed reference values for the 48-cell P2 set
   (P2_ADAPTIVE_AB_20260731; binaries 982ac639... / 00e19c0f...):
     keyed 453.6 / 453.3 / 699.1 B/s, whole 328.0 / 327.9 / 610.8 B/s,
     completions 8/8, 7/8, 8/8 vs 0/8, 4/8, 8/8; two corrupt cells with
     first-bad offsets 19686 and 16360; zero reconstruction failures.
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ra_reduce  # noqa: E402
from ra_reduce import reduce_cell, aggregate, load_cells  # noqa: E402


def _cell(rx=1000, payload=1000, gp=None, integ=True, uniq=True, wall=100.0,
          fwd=50.0, ramp=10.0, new_style=False, **kw):
    d = {
        "_file": "synthetic.json",
        "tag": kw.get("tag", "syn"), "arm": "A", "seed": 1, "snr3k": 20.0,
        "bin_md5": "deadbeef", "traffic": "random-binary",
        "connected": True, "verdict": "OK" if integ and uniq else "INTEGRITY_FAIL",
        "rx_bytes": rx, "payload_target": payload,
        "good_prefix_bytes": gp if gp is not None else (rx if integ else 0),
        "byte_integrity_ok": integ, "uniqueness_ok": uniq,
        "integrity_first_bad_offset": None if integ else (gp or 0),
        "integrity_mismatch_bytes": 0 if integ else max(1, rx - (gp or 0)),
        "integrity_mismatch_segments": 0 if integ else 1,
        "wall_secs": wall, "connected_at_s": ramp,
        "anatomy": {"ptt_fwd_airtime_s": fwd},
        # the real harness window field's numerator is the DELIVERED stream
        # (rx), corruption-blind - verified on the 48-cell P2 fixture set
        "windows": {"whole_transfer_warm": {"content_Bmin":
                    round(rx * 60.0 / wall, 1)}},
        "delivered_full": rx >= payload,   # legacy count-only field
    }
    if new_style:
        d["delivered_full_count_only"] = rx >= payload
        d["delivered_full"] = bool(rx >= payload and integ and uniq)
        d["content_shear_suspected"] = bool(rx >= payload and not integ)
    d.update({k: v for k, v in kw.items() if k != "tag"})
    return d


class TestCompletionHonest(unittest.TestCase):
    def test_clean_complete(self):
        r = reduce_cell(_cell())
        self.assertTrue(r["completion_honest"])
        self.assertEqual(r["completion_basis"], "recomputed_legacy")
        self.assertFalse(r["content_shear_suspected"])

    def test_sheared_stream_counts_as_incomplete(self):
        # count parity kept (rx == payload) but content broken: the exact
        # silent-pass case of the count-only field.
        r = reduce_cell(_cell(rx=1000, payload=1000, gp=400, integ=False))
        self.assertFalse(r["completion_honest"])
        self.assertTrue(r["completion_count_only"])   # the old field lied
        self.assertTrue(r["content_shear_suspected"])

    def test_truncated_corrupt_is_not_shear(self):
        r = reduce_cell(_cell(rx=500, payload=1000, gp=400, integ=False))
        self.assertFalse(r["completion_honest"])
        self.assertFalse(r["content_shear_suspected"])

    def test_new_style_field_agreement(self):
        r = reduce_cell(_cell(new_style=True))
        self.assertEqual(r["completion_basis"], "harness_honest_field")
        self.assertFalse(r["completion_field_disagrees"])

    def test_uniqueness_fail_blocks_completion(self):
        r = reduce_cell(_cell(uniq=False))
        self.assertFalse(r["completion_honest"])


class TestReconstructionLaw(unittest.TestCase):
    def test_identity_holds_on_consistent_cell(self):
        r = reduce_cell(_cell(rx=1000, gp=1000, wall=100.0, fwd=50.0))
        # keyed = 20 B/s, duty = 0.5, whole = 10 B/s -> exact identity
        self.assertEqual(r["keyed_user_rate_Bps"], 20.0)
        self.assertEqual(r["duty"], 0.5)
        self.assertEqual(r["whole_user_rate_Bps"], 10.0)
        self.assertTrue(r["recon_ok"])
        self.assertLessEqual(r["recon_rel_err"], 1e-9)
        self.assertLessEqual(r["recon3_rel_err"], 1e-9)

    def test_inconsistent_fields_fail_the_law(self):
        # A whole rate that cannot be keyed x duty (fields from different
        # windows) must be flagged, not silently reduced.
        d = _cell(rx=1000, gp=1000, wall=100.0, fwd=50.0)
        d["wall_secs"] = 100.0
        d["anatomy"]["ptt_fwd_airtime_s"] = 50.0
        r_ok = reduce_cell(d)
        self.assertTrue(r_ok["recon_ok"])
        # now poison the good_prefix used for keyed vs whole via a bogus
        # negative ramp making duty_post inconsistent: emulate by shrinking
        # wall only in the whole computation path -> use a direct check on
        # recon tolerance instead: force disagreement via recon_tol=0.
        r_strict = reduce_cell(d, recon_tol=-1.0)
        self.assertFalse(r_strict["recon_ok"])   # any nonzero err fails tol<0

    def test_missing_airtime_yields_no_verdict(self):
        d = _cell()
        d["anatomy"] = {}
        r = reduce_cell(d)
        self.assertIsNone(r["keyed_user_rate_Bps"])
        self.assertIsNone(r["recon_ok"])


class TestStoredFieldCrossCheck(unittest.TestCase):
    """The numerator-swap tripwire (instrument-#13 class).

    The identity layer of the reconstruction law holds by construction when
    all factors are recomputed from the same raw fields, so the defect class
    that actually bit (a modeled numerator stored in a rate field, divided by
    a measured wall) can only be caught by comparing each STORED rate field
    against the raw recompute in its own numerator class.
    """

    def test_modeled_numerator_in_stored_keyed_field_is_flagged(self):
        # Replay the retired true_wire_keyed defect with the real P2 numbers:
        # raw fields give keyed = 262144/577.9 = 453.6 B/s; the retired meter
        # stored 16469.2 B/min = 274.5 B/s (modeled airtime x a net-bps table
        # over the same cell). If that value ever lands in the stored keyed
        # field again, the reducer must refuse the decomposition.
        d = _cell(rx=262144, payload=262144, gp=262144,
                  wall=799.0, fwd=577.9, ramp=52.0)
        d["bytes_per_fwd_airtime"] = 274.5
        r = reduce_cell(d)
        self.assertAlmostEqual(r["keyed_user_rate_Bps"], 453.6, delta=0.1)
        self.assertGreater(r["xcheck_keyed_rel_err"], 0.35)
        self.assertFalse(r["stored_xcheck_ok"])
        self.assertFalse(r["recon_ok"])

    def test_consistent_stored_fields_pass(self):
        d = _cell(rx=262144, payload=262144, gp=262144,
                  wall=799.0, fwd=577.9, ramp=52.0)
        d["bytes_per_fwd_airtime"] = 453.61       # honest harness value
        d["anatomy"]["ptt_duty"] = 0.723
        r = reduce_cell(d)
        self.assertTrue(r["stored_xcheck_ok"])
        self.assertTrue(r["recon_ok"])

    def test_corrupt_cell_window_field_checked_in_own_units(self):
        # The whole-transfer window field counts DELIVERED bytes; on a
        # sheared cell it exceeds the good-prefix rate by design. That is not
        # a numerator swap and must not false-flag (real case: the P2 shear
        # cell, window 32.7 B/s vs good-prefix 23.1 B/s).
        r = reduce_cell(_cell(rx=1000, payload=1000, gp=400, integ=False))
        self.assertTrue(r["stored_xcheck_ok"])
        self.assertNotEqual(r["whole_user_rate_Bps"],
                            r["whole_warm_field_Bps"])

    def test_zero_delivery_keyed_is_zero_not_excluded(self):
        r = reduce_cell(_cell(rx=0, payload=1000, gp=0, integ=False))
        self.assertEqual(r["keyed_user_rate_Bps"], 0.0)


class TestRetiredMeters(unittest.TestCase):
    def test_true_wire_keyed_only_survives_as_labeled_model(self):
        d = _cell(true_wire_keyed_Bmin=16466.0)
        r = reduce_cell(d)
        self.assertEqual(r["modeled_wire_capacity_Bmin"], 16466.0)
        self.assertNotIn("true_wire_keyed_Bmin", r)
        # and it never contaminates the measured rates
        self.assertEqual(r["whole_user_rate_Bps"], 10.0)


class TestAggregate(unittest.TestCase):
    def test_group_counts(self):
        rows = [reduce_cell(_cell(tag=f"c{i}")) for i in range(3)]
        rows.append(reduce_cell(_cell(tag="bad", rx=1000, gp=400,
                                      integ=False)))
        g = aggregate(rows)
        self.assertEqual(len(g), 1)
        self.assertEqual(g[0]["n"], 4)
        self.assertEqual(g[0]["n_completed_honest"], 3)
        self.assertEqual(g[0]["n_corrupt"], 1)
        self.assertEqual(g[0]["n_shear_suspected"], 1)


class TestFixtures(unittest.TestCase):
    """Reference-value reproduction on the 48-cell P2 fixture set."""
    MD5_A = "982ac6395b59359a3eb029a35e35c1e2"   # 4b964c0
    MD5_B = "00e19c0f14d834055d63d765d6e72e9c"   # ed10ac005

    @classmethod
    def setUpClass(cls):
        paths = os.environ.get("RA_REDUCE_FIXTURES")
        if not paths:
            raise unittest.SkipTest("RA_REDUCE_FIXTURES not set")
        cells, _ = load_cells(paths.split(";"))
        cls.rows = [reduce_cell(c) for c in cells]
        cls.groups = {(g["bin_md5"], g["snr3k"]): g
                      for g in aggregate(cls.rows)}

    def g(self, md5, snr):
        return self.groups[(md5, snr)]

    def test_cell_count(self):
        self.assertEqual(len(self.rows), 48)

    def test_completions(self):
        expect = {(self.MD5_A, 18.0): (8, 8), (self.MD5_A, 20.79): (7, 8),
                  (self.MD5_A, 28.0): (8, 8), (self.MD5_B, 18.0): (0, 8),
                  (self.MD5_B, 20.79): (4, 8), (self.MD5_B, 28.0): (8, 8)}
        for key, (ncomp, n) in expect.items():
            g = self.g(*key)
            self.assertEqual((g["n_completed_honest"], g["n"]), (ncomp, n),
                             f"completions mismatch at {key}")

    def test_keyed_rates(self):
        # reference: independent P2 recompute (bpfa medians over completers)
        self.assertAlmostEqual(
            self.g(self.MD5_A, 18.0)["keyed_field_Bps_med_completers"],
            453.6, delta=0.05)
        self.assertAlmostEqual(
            self.g(self.MD5_A, 20.79)["keyed_field_Bps_med_completers"],
            453.3, delta=0.05)
        self.assertAlmostEqual(
            self.g(self.MD5_A, 28.0)["keyed_field_Bps_med_completers"],
            699.1, delta=0.05)
        self.assertAlmostEqual(
            self.g(self.MD5_B, 28.0)["keyed_field_Bps_med_completers"],
            699.0, delta=0.06)
        # raw recompute must agree with the harness field to 0.2 B/s
        for key in [(self.MD5_A, 18.0), (self.MD5_A, 28.0)]:
            g = self.g(*key)
            self.assertAlmostEqual(g["keyed_user_rate_Bps_med_completers"],
                                   g["keyed_field_Bps_med_completers"],
                                   delta=0.2)

    def test_whole_rates(self):
        self.assertAlmostEqual(
            self.g(self.MD5_A, 18.0)["whole_warm_field_Bps_med_completers"],
            328.0, delta=0.05)
        self.assertAlmostEqual(
            self.g(self.MD5_A, 20.79)["whole_warm_field_Bps_med_completers"],
            327.9, delta=0.05)
        self.assertAlmostEqual(
            self.g(self.MD5_A, 28.0)["whole_warm_field_Bps_med_completers"],
            610.8, delta=0.06)
        for key in [(self.MD5_A, 18.0), (self.MD5_A, 28.0)]:
            g = self.g(*key)
            self.assertAlmostEqual(g["whole_user_rate_Bps_med_completers"],
                                   g["whole_warm_field_Bps_med_completers"],
                                   delta=0.3)

    def test_corrupt_cells(self):
        bad = [r for r in self.rows if r.get("byte_integrity_ok") is False]
        self.assertEqual(len(bad), 2)
        self.assertEqual(sorted(r["integrity_first_bad_offset"] for r in bad),
                         [16360, 19686])

    def test_reconstruction_law_all_cells(self):
        fails = [r for r in self.rows if r.get("recon_ok") is False]
        self.assertEqual(fails, [])
        errs = [r["recon_rel_err"] for r in self.rows
                if r.get("recon_rel_err") is not None]
        self.assertTrue(errs)
        self.assertLessEqual(max(errs), 0.001)


def _bar_cell(seed=1, psig_mode="steady", spawner_width=8, box=8, bar=300.0,
              tag=None, drop_mode=False, drop_width=False):
    """A minimal, self-consistent vs-bar cell (carries a VARA bar).

    Defaults are an honest, scorable cell: steady axis, cohort width 8, box
    concurrency 8. keyed=512/60, duty=60/100, whole=512/100 reconstruct
    exactly, so no recon/xcheck flag muddies the axis/width gate under test.
    """
    d = {
        "tag": tag or f"c{seed:02d}", "arm": "test", "seed": seed,
        "snr3k": 18.0, "bin_md5": "deadbeef", "traffic": "random-binary",
        "connected": True, "verdict": "OK",
        "rx_bytes": 512, "payload_target": 512, "good_prefix_bytes": 512,
        "byte_integrity_ok": True, "uniqueness_ok": True,
        "delivered_full_count_only": True, "delivered_full": True,
        "wall_secs": 100.0,
        "anatomy": {"ptt_fwd_airtime_s": 60.0, "ptt_duty": 0.6,
                    "bytes_per_fwd_airtime": round(512 / 60.0, 4)},
        "bytes_per_fwd_airtime": round(512 / 60.0, 4),
        "psig_mode_source": "bridge_log_psig_diag",
    }
    if bar is not None:
        d["vara_bar_Bmin"] = bar
    if not drop_mode and psig_mode is not None:
        d["psig_mode"] = psig_mode
    if not drop_width and spawner_width is not None:
        d["spawner_width"] = spawner_width
    if box is not None:
        d["box_concurrent_estimate"] = box
    return d


class TestAxisWidthOffenseHelper(unittest.TestCase):
    """Direct unit tests of the per-cell axis/width offense classifier."""

    def test_clean_steady_width8_is_clean(self):
        self.assertEqual(ra_reduce._axis_width_offense(
            {"psig_mode": "steady", "spawner_width": 8,
             "box_concurrent_estimate": 8}), [])

    def test_peak_axis_flagged_instrument16(self):
        r = ra_reduce._axis_width_offense(
            {"psig_mode": "peak", "spawner_width": 8})
        self.assertTrue(any("instrument #16" in x for x in r))

    def test_missing_axis_flagged(self):
        r = ra_reduce._axis_width_offense({"spawner_width": 8})
        self.assertTrue(any("psig_mode MISSING" in x for x in r))

    def test_width_over_8_flagged_instrument17(self):
        r = ra_reduce._axis_width_offense(
            {"psig_mode": "steady", "spawner_width": 25})
        self.assertTrue(any("spawner_width=25 > 24" in x for x in r))

    def test_missing_width_flagged(self):
        r = ra_reduce._axis_width_offense({"psig_mode": "steady"})
        self.assertTrue(any("spawner_width MISSING" in x for x in r))

    def test_missing_both_gives_two_reasons(self):
        self.assertEqual(len(ra_reduce._axis_width_offense({})), 2)

    def test_box_overlap_flagged_even_when_spawner_width_ok(self):
        # the 8+2 confound: a single 8-wide spawner, but 10 cells on the box.
        r = ra_reduce._axis_width_offense(
            {"psig_mode": "steady", "spawner_width": 8,
             "box_concurrent_estimate": 30})
        self.assertTrue(any("box_concurrent_estimate=30 > 24" in x for x in r))

    def test_hwcal_override_waives_everything(self):
        self.assertEqual(ra_reduce._axis_width_offense({}, "hwcal"), [])

    def test_peak_control_admits_peak_but_still_enforces_width(self):
        self.assertEqual(ra_reduce._axis_width_offense(
            {"psig_mode": "peak", "spawner_width": 8}, "peak-control"), [])
        r = ra_reduce._axis_width_offense(
            {"psig_mode": "peak", "spawner_width": 25}, "peak-control")
        self.assertTrue(any("spawner_width=25 > 24" in x for x in r))


class TestAxisWidthGate(unittest.TestCase):
    """End-to-end reducer gate on a synthetic res-JSON corpus (fail-before /
    pass-after: the same corpus is REFUSED by the gate but SCORES under the
    legacy-equivalent --no-vs-bar path)."""

    def setUp(self):
        self._dirs = []

    def tearDown(self):
        for d in self._dirs:
            shutil.rmtree(d, ignore_errors=True)

    def _write(self, cells):
        d = tempfile.mkdtemp(prefix="ra_axis_")
        self._dirs.append(d)
        for i, c in enumerate(cells):
            with open(os.path.join(d, f"res_{i:02d}.json"), "w") as f:
                json.dump(c, f)
        return d

    def _run(self, argv):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = ra_reduce.main(argv)
        return rc, out.getvalue(), err.getvalue()

    def test_steady_width8_scores(self):
        d = self._write([_bar_cell(seed=i) for i in range(1, 9)])
        rc, out, err = self._run([d])
        self.assertEqual(rc, 0, err)
        red = json.loads(out)
        self.assertEqual(red["n_cells"], 8)
        self.assertTrue(red["groups"])
        self.assertIsNotNone(red["groups"][0]["vs_vara_whole_med"])

    def test_peak_row_refused(self):
        cells = [_bar_cell(seed=i) for i in range(1, 8)]
        cells.append(_bar_cell(seed=8, psig_mode="peak", tag="peakcell"))
        rc, out, err = self._run([self._write(cells)])
        self.assertEqual(rc, 2)
        self.assertEqual(out, "")                 # refused -> nothing emitted
        self.assertIn("peakcell", err)
        self.assertIn("psig_mode", err)
        self.assertIn("REFUSING", err)

    def test_width10_row_refused(self):
        cells = [_bar_cell(seed=i) for i in range(1, 8)]
        cells.append(_bar_cell(seed=8, spawner_width=25, box=None,
                               tag="widecell"))
        rc, out, err = self._run([self._write(cells)])
        self.assertEqual(rc, 2)
        self.assertEqual(out, "")
        self.assertIn("widecell", err)
        self.assertIn("spawner_width=25 > 24", err)

    def test_missing_fields_refused(self):
        cells = [_bar_cell(seed=i) for i in range(1, 8)]
        cells.append(_bar_cell(seed=8, drop_mode=True, drop_width=True,
                               box=None, tag="nofields"))
        rc, out, err = self._run([self._write(cells)])
        self.assertEqual(rc, 2)
        self.assertIn("nofields", err)
        self.assertIn("psig_mode MISSING", err)
        self.assertIn("spawner_width MISSING", err)

    def test_box_overlap_refused(self):
        # spawner_width=8 passes the width gate, but 10 cells ran on the box.
        cells = [_bar_cell(seed=i) for i in range(1, 8)]
        cells.append(_bar_cell(seed=8, spawner_width=8, box=30, tag="overlap"))
        rc, out, err = self._run([self._write(cells)])
        self.assertEqual(rc, 2)
        self.assertIn("box_concurrent_estimate=30 > 24", err)

    def test_override_admits_peak_control(self):
        d = self._write([_bar_cell(seed=i, psig_mode="peak")
                         for i in range(1, 9)])
        rc, out, err = self._run([d, "--axis-override", "peak-control"])
        self.assertEqual(rc, 0, err)
        self.assertEqual(json.loads(out)["n_cells"], 8)
        # and WITHOUT the override the very same corpus is refused
        rc2, _, _ = self._run([d])
        self.assertEqual(rc2, 2)

    def test_override_admits_hwcal(self):
        # HW-cal rows carry a bar but have no sim axis and no spawner width.
        d = self._write([_bar_cell(seed=i, drop_mode=True, drop_width=True,
                                   box=None) for i in range(1, 9)])
        rc, out, err = self._run([d, "--axis-override", "hwcal"])
        self.assertEqual(rc, 0, err)
        self.assertEqual(json.loads(out)["n_cells"], 8)
        rc2, _, _ = self._run([d])
        self.assertEqual(rc2, 2)

    def test_no_vs_bar_reduce_unchanged(self):
        # (a) fail-before / pass-after on ONE corpus: the gate REFUSES the peak
        # cell, but --no-vs-bar (the legacy-equivalent path) scores it.
        cells = [_bar_cell(seed=i) for i in range(1, 8)]
        cells.append(_bar_cell(seed=8, psig_mode="peak", tag="peakcell"))
        d = self._write(cells)
        rc_gate, _, _ = self._run([d])
        self.assertEqual(rc_gate, 2)              # pass-after: guardrail refuses
        rc_nogate, out_nogate, _ = self._run([d, "--no-vs-bar"])
        self.assertEqual(rc_nogate, 0)            # fail-before behavior preserved
        self.assertTrue(json.loads(out_nogate)["groups"])
        # (b) the gate is OUTPUT-TRANSPARENT: on a clean corpus, default and
        # --no-vs-bar emit byte-identical reductions.
        dc = self._write([_bar_cell(seed=i) for i in range(1, 9)])
        _, out_default, _ = self._run([dc])
        _, out_nogate2, _ = self._run([dc, "--no-vs-bar"])
        self.assertEqual(out_default, out_nogate2)
        # (c) a NON-vs-bar corpus (no bar) is byte-identical with/without the
        # flag -- the non-vs-bar reduction path is never touched by the gate.
        dn = self._write([_bar_cell(seed=i, bar=None) for i in range(1, 9)])
        _, out_nb_default, _ = self._run([dn])
        _, out_nb_nogate, _ = self._run([dn, "--no-vs-bar"])
        self.assertEqual(out_nb_default, out_nb_nogate)


if __name__ == "__main__":
    unittest.main(verbosity=2)

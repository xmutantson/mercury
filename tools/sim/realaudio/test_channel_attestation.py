#!/usr/bin/env python3
"""Deterministic guards for bridge-authoritative real-audio coordinates."""

from __future__ import annotations

import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parallel_spawner
import realaudio_bridge_s32 as bridge
from channel_attestation import attested_snr3k


def write_stats(path: Path, **overrides: object) -> None:
    attestation = {
        "cell": "WGN:21",
        "profile": "WGN",
        "commanded_snr": 21.0,
        "realized_snr_offset_db": 4.782343639228,
        "seed": 7,
        "passthrough": False,
        "realized_p_sig": 0.0225,
    }
    attestation.update(overrides)
    path.write_text(json.dumps({"channel_attestation": attestation}),
                    encoding="utf-8")


class ChannelCoordinateTests(unittest.TestCase):
    def test_historical_2p4_split_is_instrument_invalid_and_unscored(self):
        with tempfile.TemporaryDirectory() as tmp:
            stats = Path(tmp) / "bridge.json"
            write_stats(stats)
            snr3k, audit = attested_snr3k(
                stats, expected_cell="WGN:21", expected_profile="wgn",
                expected_seed=7, expected_passthrough=False,
                requested_snr3k=23.4)

        self.assertIsNone(snr3k)
        self.assertFalse(audit["valid"])
        self.assertTrue(audit["instrument_invalid"])
        self.assertAlmostEqual(audit["realized_snr3k_db"], 25.782343639228)
        self.assertAlmostEqual(audit["requested_delta_db"], -2.382343639228)
        self.assertIn("requested_snr3k_mismatch", audit["reasons"])

    def test_bridge_coordinate_is_authoritative_when_assertion_matches(self):
        with tempfile.TemporaryDirectory() as tmp:
            stats = Path(tmp) / "bridge.json"
            write_stats(stats)
            snr3k, audit = attested_snr3k(
                stats, expected_cell="WGN:21", expected_profile="wgn",
                expected_seed=7, expected_passthrough=False,
                requested_snr3k=25.79)

        self.assertTrue(audit["valid"])
        self.assertFalse(audit["instrument_invalid"])
        self.assertAlmostEqual(snr3k, 25.782343639228)
        self.assertNotEqual(snr3k, 25.79)

    def test_missing_or_identity_mismatched_evidence_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "missing.json"
            snr3k, audit = attested_snr3k(
                missing, expected_cell="WGN:21", expected_profile="wgn",
                expected_seed=7, expected_passthrough=False)
            self.assertIsNone(snr3k)
            self.assertTrue(audit["instrument_invalid"])
            self.assertIn("stats_missing", audit["reasons"])

            stats = Path(tmp) / "bridge.json"
            write_stats(stats, seed=8)
            snr3k, audit = attested_snr3k(
                stats, expected_cell="WGN:21", expected_profile="wgn",
                expected_seed=7, expected_passthrough=False)
            self.assertIsNone(snr3k)
            self.assertIn("seed_mismatch", audit["reasons"])

    def test_passthrough_is_attested_but_not_vara_scorable(self):
        with tempfile.TemporaryDirectory() as tmp:
            stats = Path(tmp) / "bridge.json"
            write_stats(stats, passthrough=True)
            snr3k, audit = attested_snr3k(
                stats, expected_cell="WGN:21", expected_profile="wgn",
                expected_seed=7, expected_passthrough=True)
        self.assertTrue(audit["valid"])
        self.assertFalse(audit["instrument_invalid"])
        self.assertFalse(audit["vara_scorable"])
        self.assertIsNone(snr3k)

    def test_faded_profile_requires_explicit_calibrated_snr_for_vara(self):
        with tempfile.TemporaryDirectory() as tmp:
            stats = Path(tmp) / "bridge.json"
            write_stats(stats, profile="MPG")
            snr3k, audit = attested_snr3k(
                stats, expected_cell="WGN:21", expected_profile="mpg",
                expected_seed=7, expected_passthrough=False)
            self.assertTrue(audit["valid"])
            self.assertFalse(audit["instrument_invalid"])
            self.assertFalse(audit["vara_scorable"])
            self.assertIsNone(snr3k)
            self.assertEqual(audit["vara_scoring_disabled_reason"],
                             "faded_profile_not_physically_calibrated")

            snr3k, audit = attested_snr3k(
                stats, expected_cell="WGN:21", expected_profile="mpg",
                expected_seed=7, expected_passthrough=False,
                requested_snr3k=25.782343639228)
            self.assertTrue(audit["vara_scorable"])
            self.assertAlmostEqual(snr3k, 25.782343639228)

    def test_bridge_preserves_fractional_commanded_coordinate(self):
        args = types.SimpleNamespace(profile="wgn", seed=1, passthrough=False)
        stats = {"fwd": {"sig_frames": 2, "sig_sumsq": 0.045}}
        att = bridge.build_channel_attestation(
            args, stats, 37.6, 4.782343639228, "WGN:37.6")
        self.assertEqual(att["commanded_snr"], 37.6)
        self.assertAlmostEqual(att["realized_snr3k"], 42.382343639228)

    def test_cohort_reducer_uses_only_attested_per_run_coordinates(self):
        results = [
            {"tag": "s1", "snr3k": 25.782, "vara_scoring_enabled": True,
             "instrument_invalid": False},
            {"tag": "s2", "snr3k": 25.784, "vara_scoring_enabled": True,
             "instrument_invalid": False},
        ]
        snr3k, audit = parallel_spawner.attested_cohort_coordinate(results)
        self.assertIn(snr3k, (25.782, 25.784))
        self.assertTrue(audit["valid"])

        results[1].update(instrument_invalid=True, snr3k=None,
                          vara_scoring_enabled=False)
        snr3k, audit = parallel_spawner.attested_cohort_coordinate(results)
        self.assertIsNone(snr3k)
        self.assertFalse(audit["valid"])
        self.assertEqual(audit["invalid_tags"], ["s2"])


if __name__ == "__main__":
    unittest.main()

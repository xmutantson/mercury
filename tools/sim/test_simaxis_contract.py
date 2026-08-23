#!/usr/bin/env python3
"""Fast contract tests for the versioned SIMAXIS machinery."""

import math
import unittest

from sim_axis import (AXIS_V0, AXIS_V1, AXIS_V2, AxisError,
                      cn_from_snr3k, headroom_plan, noise_variance,
                      require_single_axis, resolve_axis, validate_attestation)


class CoordinateContractTest(unittest.TestCase):
    def test_default_is_v1_steady_coordinate(self):
        resolved = resolve_axis(
            AXIS_V1, configured_bandwidth_hz=2343.75,
            default_snr3k_db=30.0)
        self.assertEqual(resolved["axis_version"], AXIS_V1)
        self.assertEqual(resolved["snr3k_db"], 30.0)

    def test_two_numeric_coordinates_are_always_rejected(self):
        with self.assertRaises(AxisError):
            resolve_axis(AXIS_V2, snr3k_db=4.8, cn_config_db=5.8720997,
                         configured_bandwidth_hz=2343.75)

    def test_legacy_snr_is_v0_only(self):
        with self.assertRaises(AxisError):
            resolve_axis(AXIS_V1, snr=4.8,
                         configured_bandwidth_hz=2343.75)
        self.assertEqual(resolve_axis(
            AXIS_V0, snr=4.8,
            configured_bandwidth_hz=2343.75)["snr3k_db"], 4.8)

    def test_deprecated_alias_controls_v2(self):
        resolved = resolve_axis(
            AXIS_V2, snr3k=-5.2,
            configured_bandwidth_hz=2343.75)
        self.assertEqual(resolved["input_coordinate"], "snr3k")
        self.assertAlmostEqual(resolved["cn_config_db"], -4.1279003)

    def test_bandwidth_terms(self):
        self.assertAlmostEqual(cn_from_snr3k(0, 2343.75), 1.072099696,
                               places=9)
        self.assertAlmostEqual(cn_from_snr3k(0, 468.75), 8.061799740,
                               places=9)


class PhysicalLawTest(unittest.TestCase):
    def test_cn_and_snr3k_forms_are_identical(self):
        power = 0.03125
        snr3k = -5.2
        bandwidth = 2343.75
        cn = cn_from_snr3k(snr3k, bandwidth)
        via_snr3k = noise_variance(power, snr3k)
        via_cn = power * 24000.0 / (
            10.0 ** (cn / 10.0) * bandwidth)
        self.assertEqual(via_snr3k, via_cn)

    def test_headroom_is_fixed_and_bounded(self):
        variance = noise_variance(0.05, -5.2)
        plan = headroom_plan(0.4, variance, 1_048_576, 1e-9)
        self.assertLess(plan["composite_scale"], 1.0)
        self.assertLessEqual(
            plan["composite_scale"] * plan["headroom_bound"], 0.9)


class ReducerTest(unittest.TestCase):
    @staticmethod
    def row(axis):
        return {
            "axis_version": axis, "input_coordinate": "snr3k_db",
            "reference_mode": "fixed", "reference_id": "r",
            "reference_power": 1.0, "reference_n_samples": 1,
            "configured_bandwidth_hz": 2343.75, "snr3k_db": 0.0,
            "cn_config_db": cn_from_snr3k(0, 2343.75),
            "noise_variance": 8.0, "seed": 1, "composite_scale": 1.0,
            "pre_scale_peak": 0.1, "hard_clip_count": 0,
            "s32_saturation_count": 0, "binary_sha256": "b" * 64,
            "recipe_sha256": "a" * 64,
        }

    def test_rejects_zero_ambiguous_and_mixed_rows(self):
        with self.assertRaises(AxisError):
            require_single_axis([])
        ambiguous = self.row(AXIS_V2)
        del ambiguous["reference_id"]
        with self.assertRaises(AxisError):
            require_single_axis([ambiguous])
        with self.assertRaises(AxisError):
            require_single_axis([self.row(AXIS_V1), self.row(AXIS_V2)])
        self.assertEqual(require_single_axis([self.row(AXIS_V2)]), AXIS_V2)

    def test_v2_attestation_rejects_clip_and_budget_overrun(self):
        row = self.row(AXIS_V2)
        row.update({"reference_mode": "fixed-archived-pcal",
                    "s32_mode": "prescaled",
                    "headroom_budget_exceeded": False})
        self.assertEqual(validate_attestation(row)["axis_version"], AXIS_V2)
        clipped = dict(row, hard_clip_count=1)
        with self.assertRaises(AxisError):
            validate_attestation(clipped)
        overrun = dict(row, headroom_budget_exceeded=True)
        with self.assertRaises(AxisError):
            validate_attestation(overrun)


if __name__ == "__main__":
    unittest.main()

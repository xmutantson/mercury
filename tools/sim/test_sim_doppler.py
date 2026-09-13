#!/usr/bin/env python3
"""Verify the MPP tap's Gaussian Doppler width at one OFDM-symbol lag."""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_channel_relay as relay
from realaudio import test_bridge_c_faithfulness as firmware_reference


SYMBOL_SAMPLES = 1168
SECONDS = 30
SEEDS = (2, 3, 5, 6)


def _add_pairs(accumulator, values):
    left = values[:-SYMBOL_SAMPLES]
    right = values[SYMBOL_SAMPLES:]
    accumulator[0] += float(np.vdot(left, right).real)
    accumulator[1] += float(np.vdot(left, left).real)
    accumulator[2] += float(np.vdot(right, right).real)


def _measure(tap_type):
    accumulator = [0.0, 0.0, 0.0]
    composite_accumulator = [0.0, 0.0, 0.0]
    sample_count = int(SECONDS * relay.FS)
    for seed in SEEDS:
        source = relay.Xoshiro(seed * 2654435761 & 0xFFFFFFFF)
        rng = source.seed_np()
        tap0 = tap_type(1.0, rng)
        tap1 = tap_type(1.0, rng)
        gains0 = np.empty(sample_count, dtype=np.complex64)
        gains1 = np.empty(sample_count, dtype=np.complex64)
        for start in range(0, sample_count, relay.CHUNK_SAMPLES):
            count = min(relay.CHUNK_SAMPLES, sample_count - start)
            gains0[start:start + count] = tap0.advance(count)
            gains1[start:start + count] = tap1.advance(count)
            rng.standard_normal(count)
        _add_pairs(accumulator, gains0)
        _add_pairs(accumulator, gains1)
        _add_pairs(composite_accumulator, gains0 + gains1)

    return _metrics(accumulator), _metrics(composite_accumulator)


def _metrics(accumulator):
    rho = accumulator[0] / math.sqrt(accumulator[1] * accumulator[2])
    symbol_seconds = SYMBOL_SAMPLES / relay.FS
    sigma_eff = math.sqrt(
        -math.log(rho) / (2.0 * math.pi * math.pi * symbol_seconds ** 2))
    return rho, sigma_eff


class DopplerWidthTest(unittest.TestCase):
    def test_firmware_cadence_hold_and_power_normalization(self):
        default_tap = relay.DopplerTap(1.0, np.random.default_rng(7))
        slow_tap = relay.DopplerTap(0.1, np.random.default_rng(8))
        explicit_tap = relay.DopplerTap(
            1.0, np.random.default_rng(9), update=19)
        gains = explicit_tap.advance(20)

        self.assertEqual(default_tap.update, 750)
        self.assertEqual(slow_tap.update, 7500)
        self.assertEqual(explicit_tap.update, 19)
        self.assertTrue(np.all(gains[:19] == gains[0]))
        self.assertNotEqual(gains[19], gains[0])
        power = (2.0 * default_tap.inno_std ** 2 *
                 float(np.sum(relay.GAUS_FIR_COEFFS ** 2)))
        self.assertAlmostEqual(power, 1.0, places=14)

    def test_mpp_width_matches_firmware_reference(self):
        (rho, sigma_eff), (_, composite_sigma) = _measure(relay.DopplerTap)
        (reference_rho, reference_sigma), (_, reference_composite_sigma) = (
            _measure(firmware_reference.DopplerTap))
        relative_error = abs(sigma_eff - reference_sigma) / reference_sigma
        composite_error = (abs(composite_sigma - reference_composite_sigma) /
                           reference_composite_sigma)

        print(f"relay rho(Tsym)={rho:.9f} sigma_eff={sigma_eff:.6f} Hz")
        print(f"firmware reference rho(Tsym)={reference_rho:.9f} "
              f"sigma_eff={reference_sigma:.6f} Hz")
        print(f"sigma relative error={relative_error:.6%}")
        print(f"composite sigma_eff={composite_sigma:.6f} Hz "
              f"relative error={composite_error:.6%}")

        self.assertGreaterEqual(sigma_eff, 0.5 * 0.85)
        self.assertLessEqual(sigma_eff, 0.5 * 1.15)
        self.assertLess(relative_error, 0.05)
        self.assertLess(composite_error, 0.05)
        self.assertEqual(len(relay.GAUS_FIR_COEFFS), 128)
        self.assertEqual(relay.GAUS_FIR_COEFFS.tobytes(),
                         firmware_reference.GAUS_FIR_COEFFS.tobytes())


if __name__ == "__main__":
    unittest.main()

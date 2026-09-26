#!/usr/bin/env python3
"""A declared negative-control cell is never compared with the VARA bar."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arq_realaudio as AR  # noqa: E402


def attestation():
    return {"valid": True, "vara_scorable": True, "vara_scoring_disabled_reason": None,
            "reasons": []}


class NegativeControlGate(unittest.TestCase):
    def test_gate_exists(self):
        self.assertTrue(hasattr(AR, "vara_scoring_gate"),
                        "harness has no vs-bar gate for negative-control cells")

    def test_clean_cell_scores(self):
        att = attestation()
        self.assertTrue(AR.vara_scoring_gate(att, False, {"status": "CLEAN"}))
        self.assertTrue(att["vara_scorable"])

    def test_negative_control_never_scores(self):
        att = attestation()
        pre = {"status": "NEGATIVE-CONTROL",
               "declared_negative_control": "MERCURY_ACK_TIMEOUT_FLOOR_DEFEAT"}
        self.assertFalse(AR.vara_scoring_gate(att, False, pre))
        self.assertFalse(att["vara_scorable"])
        self.assertEqual(att["vara_scoring_disabled_reason"],
                         "negative_control:MERCURY_ACK_TIMEOUT_FLOOR_DEFEAT")

    def test_invalid_cell_does_not_score(self):
        self.assertFalse(AR.vara_scoring_gate(attestation(), True, {"status": "CLEAN"}))

    def test_unavailable_registry_keeps_existing_behaviour(self):
        self.assertTrue(AR.vara_scoring_gate(attestation(), False, {"status": "UNAVAILABLE"}))


if __name__ == "__main__":
    unittest.main()

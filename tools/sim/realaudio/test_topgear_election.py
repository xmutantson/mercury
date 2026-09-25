#!/usr/bin/env python3
"""Regression tests for the authoritative top-gear election log meter."""

import tempfile
import unittest
from pathlib import Path

import capstone_arms as CA


CURRENT_ELECTION_LOG = """\
[T+0158.803] [CMD] [TOPGEAR] queue SET_CONFIG: 16 -> 17 (ACK-seam dispatch; engaged=1 flatness=0.000 snr_dl=23.0 streak=2 fully_acked=1)
[T+0160.476] [RSP] [PHY] Loading configuration 17 (was 16)
[T+0160.879] [RSP] [PHY] Config 17 active: M=64 LDPC_rate=0.875 BW=2344Hz Nc=50 Nsymb=7 nBits=1572
"""

NO_ELECTION_LOG = """\
[T+0158.803] [CMD] [GEARSHIFT] SET_CONFIG: forward=16 reverse=13 (SNR down=23.0 up=23.0) link_status=2
[T+0160.476] [RSP] [PHY] Loading configuration 17 (was 16)
[T+0160.879] [RSP] [PHY] Config 17 active: M=64 LDPC_rate=0.875 BW=2344Hz Nc=50 Nsymb=7 nBits=1572
"""


class TopgearElectionTests(unittest.TestCase):
    def parse(self, text):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "arq.log"
            path.write_text(text, encoding="utf-8")
            return CA.parse_topgear_election(path)

    def test_current_production_event_records_config_17_election(self):
        self.assertEqual(
            self.parse(CURRENT_ELECTION_LOG),
            {
                "elected_16_to_17": True,
                "transitions": [(16, 17)],
                "matched_rows": 1,
            },
        )

    def test_phy_activation_without_authoritative_event_is_not_an_election(self):
        self.assertEqual(
            self.parse(NO_ELECTION_LOG),
            {
                "elected_16_to_17": False,
                "transitions": [],
                "matched_rows": 0,
            },
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)

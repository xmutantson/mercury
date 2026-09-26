#!/usr/bin/env python3
"""The config timeline keys on the config being loaded, not the one being left.

The modem prints "[CFG] load_configuration(N) current=M" before it assigns N
(arq_common.cc load_configuration), so M is the previous config. A timeline keyed
on M lags one transition behind: a pinned cell that loads 16 and later demotes to
15 reads as "reached 16 late, then held" instead of "held 16, demoted to 15".
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import capstone_arms as CA  # noqa: E402

LOG = (
    "[T+0002.100] [RSP] [CFG] load_configuration(100) current=-1 level=FULL backup=NO\n"
    "[T+0002.200] [CMD] [CFG] load_configuration(100) current=-1 level=FULL backup=NO\n"
    "[T+0030.000] [RSP] [CFG] load_configuration(16) current=100 level=FULL backup=NO\n"
    "[T+0030.100] [CMD] [CFG] load_configuration(16) current=100 level=FULL backup=NO\n"
    "[T+0200.000] [RSP] [CFG] load_configuration(15) current=16 level=FULL backup=NO\n"
    "[T+0200.100] [CMD] [CFG] load_configuration(15) current=16 level=FULL backup=NO\n"
)


class ConfigTimelineTarget(unittest.TestCase):
    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix=".log")
        with os.fdopen(fd, "w") as f:
            f.write(LOG)

    def tearDown(self):
        os.unlink(self.path)

    def test_timeline_is_keyed_on_the_loaded_config(self):
        tl = CA.parse_config_timeline(self.path)
        self.assertEqual([c for _t, c in tl], [100, 100, 16, 16, 15, 15])
        self.assertEqual(tl[2], (30.0, 16))

    def test_pinned_demote_is_seen(self):
        tl = CA.parse_config_timeline(self.path)
        v = CA.config_held_verdict(tl, 16, connected_at_s=None, pinned=True)
        self.assertTrue(v["reached_target"])
        self.assertFalse(v["config_held"])
        self.assertEqual(v["demoted_to"], 15)

    def test_pinned_hold_is_seen(self):
        tl = [(t, c) for t, c in CA.parse_config_timeline(self.path) if t < 100]
        v = CA.config_held_verdict(tl, 16, connected_at_s=None, pinned=True)
        self.assertTrue(v["reached_target"])
        self.assertTrue(v["config_held"])


# Retained-cell shape (frontA floor cells): each modem loads 0 then the target at
# start-up, a few seconds apart, and the cell log interleaves them.
INTERLEAVED = (
    "[T+0002.274] [RSP] [CFG] load_configuration(0) current=-1 level=FULL backup=NO\n"
    "[T+0002.500] [RSP] [CFG] load_configuration(10) current=0 level=FULL backup=NO\n"
    "[T+0005.275] [CMD] [CFG] load_configuration(0) current=-1 level=FULL backup=NO\n"
    "[T+0005.499] [CMD] [CFG] load_configuration(10) current=0 level=FULL backup=NO\n"
)
POST_TRANSFER_DROP = (
    "[T+0400.000] [RSP] [CFG] load_configuration(0) current=10 level=FULL backup=NO\n"
    "[T+0400.100] [CMD] [CFG] load_configuration(0) current=10 level=FULL backup=NO\n"
)
MID_TRANSFER_DEMOTE = (
    "[T+0200.000] [RSP] [CFG] load_configuration(9) current=10 level=FULL backup=NO\n"
)


class PerPeerTimeline(unittest.TestCase):
    def _log(self, text):
        fd, path = tempfile.mkstemp(suffix=".log")
        with os.fdopen(fd, "w") as f:
            f.write(text)
        self.addCleanup(os.unlink, path)
        return path

    def test_interleaved_start_up_is_not_a_demote(self):
        path = self._log(INTERLEAVED)
        self.assertEqual([c for _t, c in CA.parse_config_timeline(path, "RSP")], [0, 10])
        v = CA.config_held_both_peers(path, 10, None, pinned=True)
        self.assertTrue(v["config_held"])
        self.assertIsNone(v["demoted_to"])

    def test_post_transfer_drop_is_outside_the_window(self):
        path = self._log(INTERLEAVED + POST_TRANSFER_DROP)
        self.assertTrue(CA.config_held_both_peers(path, 10, None, pinned=True,
                                                  until_s=300.0)["config_held"])
        self.assertFalse(CA.config_held_both_peers(path, 10, None, pinned=True)["config_held"])

    def test_mid_transfer_demote_of_one_peer_is_seen(self):
        path = self._log(INTERLEAVED + MID_TRANSFER_DEMOTE + POST_TRANSFER_DROP)
        v = CA.config_held_both_peers(path, 10, None, pinned=True, until_s=300.0)
        self.assertFalse(v["config_held"])
        self.assertEqual(v["demoted_to"], 9)
        self.assertFalse(v["per_peer"]["RSP"]["config_held"])
        self.assertTrue(v["per_peer"]["CMD"]["config_held"])


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Tests for tools/contamination_endpoint.py: the endpoint of an incomplete cell comes
from the result's last_rx_at_s; without it the cell stays ENDPOINT-UNDERIVED."""
import json
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import contamination_endpoint as CE  # noqa: E402


def make_cell(root, name, result, clip_lines, fwd=(0, 0), rev=(0, 0)):
    d = pathlib.Path(root) / name
    (d / "logs").mkdir(parents=True)
    (d / "result.json").write_text(json.dumps(result))
    (d / "logs" / "bridge_x_stats.json").write_text(json.dumps({
        "fwd": {"input_at_fs": fwd[0], "hard_clips": fwd[1]},
        "rev": {"input_at_fs": rev[0], "hard_clips": rev[1]}}))
    lines = ["[T+0001.000] [CMD] link_status:Idle"]
    for t, peer, n in clip_lines:
        lines.append("[T+%08.3f] [%s] [TX-CLIP] %d samples clipped" % (t, peer, n))
    (d / "logs" / "arq_x.log").write_text("\n".join(lines) + "\n")
    return d


INCOMPLETE = {"whole_session_status": "TRANSFER_DEADLINE", "completion_at_s": None}


class EndpointOfIncompleteCell(unittest.TestCase):
    def test_clip_after_last_byte_is_not_contamination(self):
        with tempfile.TemporaryDirectory() as root:
            d = make_cell(root, "a_s1", dict(INCOMPLETE, last_rx_at_s=50.0),
                          [(60.0, "CMD", 100)], fwd=(100, 0))
            row = CE.classify(d)
            self.assertEqual(row["endpoint_s"], 50.0)
            self.assertEqual(row["endpoint_source"], "last_rx_at_s")
            self.assertFalse(row["endpoint_bounded_contaminated"], row["endpoint_bounded_reasons"])

    def test_clip_before_last_byte_is_contamination(self):
        with tempfile.TemporaryDirectory() as root:
            d = make_cell(root, "a_s1", dict(INCOMPLETE, last_rx_at_s=50.0),
                          [(40.0, "CMD", 100)], fwd=(100, 0))
            row = CE.classify(d)
            self.assertTrue(row["endpoint_bounded_contaminated"])
            self.assertIn("tx_clip-before-endpoint", row["endpoint_bounded_reasons"])
            self.assertIn("fwd-input_at_fs-unexplained", row["endpoint_bounded_reasons"])

    def test_legacy_result_without_the_field_stays_underived(self):
        with tempfile.TemporaryDirectory() as root:
            d = make_cell(root, "a_s1", dict(INCOMPLETE), [(60.0, "CMD", 100)], fwd=(100, 0))
            row = CE.classify(d)
            self.assertIsNone(row["endpoint_s"])
            self.assertIn("endpoint-underived", row["endpoint_bounded_reasons"])

    def test_nothing_delivered_stays_underived(self):
        with tempfile.TemporaryDirectory() as root:
            d = make_cell(root, "a_s1", dict(INCOMPLETE, last_rx_at_s=None),
                          [(60.0, "CMD", 100)], fwd=(100, 0))
            row = CE.classify(d)
            self.assertIn("endpoint-underived", row["endpoint_bounded_reasons"])

    def test_complete_cell_uses_completion(self):
        with tempfile.TemporaryDirectory() as root:
            d = make_cell(root, "a_s1", {"whole_session_status": "COMPLETE", "completion_at_s": 30.0,
                                         "last_rx_at_s": 30.0},
                          [(31.5, "RSP", 5)], rev=(5, 3))
            row = CE.classify(d)
            self.assertEqual(row["endpoint_source"], "completion_at_s")
            self.assertFalse(row["endpoint_bounded_contaminated"])

    def test_window_straddling_the_endpoint_counts_before(self):
        # the line prints up to 1.03 s after the clipped samples were written
        with tempfile.TemporaryDirectory() as root:
            d = make_cell(root, "a_s1", {"whole_session_status": "COMPLETE", "completion_at_s": 30.0},
                          [(30.9, "RSP", 5)], rev=(5, 0))
            row = CE.classify(d)
            self.assertTrue(row["endpoint_bounded_contaminated"])
            self.assertIn("tx_clip-before-endpoint", row["endpoint_bounded_reasons"])


class ClipLogTimeLocation(unittest.TestCase):
    """Bridge clips placed by the clip log against the harness clock."""

    def cell(self, d, events, hard, t0=1000.0, completion=50.0):
        cell = pathlib.Path(d) / "mpg_p40_s4"
        (cell / "logs").mkdir(parents=True)
        (cell / "logs" / "arq_mpg_p40_s4.log").write_text("[T+0001.000] [CMD] hello\n")
        stats = {"fwd": {"input_at_fs": 0, "hard_clips": 0}, "rev": {"input_at_fs": 0, "hard_clips": hard}}
        (cell / "logs" / "bridge_mpg_p40_s4_stats.json").write_text(json.dumps(stats))
        (cell / "logs" / "bridge_mpg_p40_s4_stats.json.clips.jsonl").write_text(
            "".join(json.dumps(e) + "\n" for e in events))
        (cell / "result.json").write_text(json.dumps({
            "whole_session_status": "COMPLETE", "completion_at_s": completion,
            "harness_t0_monotonic_s": t0}))
        return cell

    def ev(self, mono, n):
        return {"dir": "rev", "event": "clip", "t_s": 0, "dur_s": 0.01, "mono_s": mono, "lat_s": 0.05,
                "hard_clips": n, "over_fs_unscaled": n, "input_at_fs": 0, "composite_scale": 0.5}

    def test_clip_before_completion_is_flagged(self):
        with tempfile.TemporaryDirectory() as d:
            row = CE.classify(self.cell(d, [self.ev(1030.0, 7)], 7))
            self.assertTrue(row["endpoint_bounded_contaminated"])
            self.assertIn("rev-hard_clips-before-endpoint", row["endpoint_bounded_reasons"])
            self.assertEqual(row["hard_clip_attribution"], "time-located")

    def test_clip_after_completion_is_not(self):
        with tempfile.TemporaryDirectory() as d:
            row = CE.classify(self.cell(d, [self.ev(1052.0, 7)], 7))
            self.assertFalse(row["endpoint_bounded_contaminated"])
            self.assertEqual(row["rev"]["located"]["hard_post"], 7)

    def test_capture_latency_counts_toward_before(self):
        with tempfile.TemporaryDirectory() as d:
            # emitted 0.03 s after completion, but its samples could be 0.05 s older
            row = CE.classify(self.cell(d, [self.ev(1050.03, 3)], 3))
            self.assertTrue(row["endpoint_bounded_contaminated"])

    def test_totals_disagree_falls_back_to_screen(self):
        with tempfile.TemporaryDirectory() as d:
            row = CE.classify(self.cell(d, [self.ev(1052.0, 7)], 9))
            self.assertEqual(row["hard_clip_attribution"], "screen")
            self.assertTrue(row["endpoint_bounded_contaminated"])


if __name__ == "__main__":
    unittest.main(verbosity=2)

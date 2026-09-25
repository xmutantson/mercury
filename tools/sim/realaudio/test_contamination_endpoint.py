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


if __name__ == "__main__":
    unittest.main(verbosity=2)

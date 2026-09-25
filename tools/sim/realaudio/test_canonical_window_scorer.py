#!/usr/bin/env python3
"""Deterministic regression tests for the fixed-window content scorer.

These tests use synthetic monotonic timestamps.  They do not open ALSA devices,
start Mercury, or sleep for the real ten-minute score windows.
"""
import io
import inspect
import os
import sys
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import capstone_arms as ca  # noqa: E402
import arq_realaudio as ra  # noqa: E402
from canonical_window_scorer import (  # noqa: E402
    DeliveryOracle,
    LinkEventTracker,
    SessionAwareDeliveryOracle,
    audit_campaign_cell,
    score_delivered_sessions,
    score_fixed_windows,
)
from parallel_spawner import (  # noqa: E402
    is_ux_population_result,
    is_valid_result,
)


def delivery(at_s, good_prefix_bytes):
    return {"at_s": float(at_s), "good_prefix_bytes": int(good_prefix_bytes)}


def link(at_s, kind, config=None, batch=None):
    row = {"at_s": float(at_s), "kind": kind}
    if config is not None:
        row["config"] = int(config)
    if batch is not None:
        row["batch"] = int(batch)
    return row


class DeliveryOracleTests(unittest.TestCase):
    def test_nonperiodic_oracle_catches_five_byte_backward_shear(self):
        oracle = DeliveryOracle(
            lambda offset, length: ca.traffic_expected_slice(
                "random-binary", offset, length))
        oracle.feed(1.0, ca.traffic_expected_slice("random-binary", 0, 64))
        # The next receive segment starts five bytes behind the expected offset.
        oracle.feed(2.0, ca.traffic_expected_slice("random-binary", 59, 32))
        self.assertEqual(oracle.first_bad_offset, 64)
        self.assertEqual(oracle.good_prefix_bytes, 64)
        self.assertGreater(oracle.mismatch_bytes, 0)

    def test_live_rx_does_not_generate_expected_bytes_twice(self):
        source = inspect.getsource(ra.rx_thread_fn)
        self.assertNotIn("CA.traffic_expected_slice", source)


class LinkEventTrackerTests(unittest.TestCase):
    def test_ack_is_counted_once_when_clean_and_pattern_lines_both_print(self):
        tracker = LinkEventTracker()
        tracker.observe("CMD", "[CMD-TX] CONFIG_16 batch=25 type=16", 10.0)
        tracker.observe(
            "CMD",
            "[CMD-MFSK-ACK-SACK] CLEAN batch_seq_id=0 arrival_ms=1096",
            11.0)
        tracker.observe("CMD", "[CMD-ACK-PAT] Data ACK pattern detected!", 11.01)
        self.assertEqual(
            [e["kind"] for e in tracker.snapshot()].count("ack"), 1)

    def test_partial_ack_is_not_sustainable_clean_credit(self):
        tracker = LinkEventTracker()
        tracker.observe("CMD", "[CMD-TX] CONFIG_16 batch=25 type=16", 10.0)
        tracker.observe(
            "CMD",
            "[CMD-MFSK-ACK-SACK] PARTIAL batch_seq_id=0 arrival_ms=1096",
            11.0)
        tracker.observe("CMD", "[CMD-ACK-PAT] Data ACK pattern detected!", 11.01)
        self.assertEqual(
            [e["kind"] for e in tracker.snapshot()].count("ack"), 0)
        self.assertEqual(
            [e["kind"] for e in tracker.snapshot()].count("partial_ack"), 1)

    def test_cross_peer_lock_order_does_not_reorder_timestamped_ack_before_break(self):
        tracker = LinkEventTracker()
        tracker.observe("CMD", "[CMD-TX] CONFIG_16 batch=25 type=16", 10.0)
        # Deliberately deliver callbacks in lock-acquisition order rather than
        # event-time order: BREAK callback arrives first, but its timestamp is later.
        tracker.observe("RSP", "[BREAK] Block failure", 12.0)
        tracker.observe(
            "CMD",
            "[CMD-MFSK-ACK-SACK] CLEAN batch_seq_id=0 arrival_ms=900",
            11.0)
        rows = tracker.snapshot()
        self.assertEqual([row["kind"] for row in rows], ["tx", "ack", "break"])

    def test_control_batch_does_not_seed_pending_data_or_synthetic_miss(self):
        tracker = LinkEventTracker()
        tracker.observe("CMD", "[CMD-TX] CONFIG_17 batch=1 type=48", 1.0)
        tracker.observe(
            "CMD", "[CMD-ACK-PAT] Control ACK for code=67 detected!", 2.0)
        tracker.observe("CMD", "[CMD-TX] CONFIG_17 batch=95 type=16", 3.0)
        tracker.observe(
            "CMD", "[CMD-MFSK-ACK-SACK] CLEAN batch_seq_id=4", 4.0)
        rows = tracker.snapshot()
        self.assertEqual([row["kind"] for row in rows], ["tx", "ack"])
        self.assertEqual(rows[0]["config"], 17)
        self.assertEqual(rows[0]["batch"], 95)

    def test_unapplied_set_config_announcement_does_not_change_data_rung(self):
        tracker = LinkEventTracker()
        tracker.observe("CMD", "[CMD-TX] CONFIG_16 batch=25 type=16", 1.0)
        tracker.observe(
            "CMD", "[CMD-MFSK-ACK-SACK] CLEAN batch_seq_id=0 arrival_ms=900", 2.0)
        tracker.observe("CMD", "[CMD-TX] CONFIG_16 batch=25 type=16", 3.0)
        tracker.observe(
            "CMD", "[CMD-MFSK-ACK-SACK] CLEAN batch_seq_id=1 arrival_ms=900", 4.0)
        tracker.observe(
            "CMD", "[GEARSHIFT] SET_CONFIG: forward=17", 20.0)
        score = score_fixed_windows(
            request_at_s=0.0,
            connected_at_by_peer={"CMD": 1.0, "RSP": 1.0},
            observed_until_s=61.0,
            delivery_events=[],
            link_events=tracker.snapshot(),
            horizon_s=60.0,
            steady_warmup_s=30.0,
        )
        self.assertEqual(score["steady"]["status"], "OK")
        self.assertEqual(score["steady"]["sustainable_config"], 16)


class LiveClockTests(unittest.TestCase):
    def test_log_parser_records_each_peer_exactly_and_requires_both(self):
        state = ra.State()
        log = io.StringIO()
        cmd = type("Proc", (), {
            "stdout": io.BytesIO(b"link_status:Connected to TESTB\n")
        })()
        rsp = type("Proc", (), {
            "stdout": io.BytesIO(b"link_status:Connected to TESTA\n")
        })()
        with mock.patch.object(ra.time, "monotonic", return_value=101.25):
            ra.log_output(cmd, "CMD", log, 100.0, state)
        self.assertEqual(state.connected_at_by_peer, {"CMD": 1.25})
        self.assertFalse(state.connected)
        with mock.patch.object(ra.time, "monotonic", return_value=102.75):
            ra.log_output(rsp, "RSP", log, 100.0, state)
        self.assertEqual(
            state.connected_at_by_peer, {"CMD": 1.25, "RSP": 2.75})
        self.assertTrue(state.connected)

    def test_connection_parser_rejects_diagnostic_quote(self):
        state = ra.State()
        log = io.StringIO()
        proc = type("Proc", (), {
            "stdout": io.BytesIO(
                b"[TEST] expected marker link_status:Connected to TESTB\n")
        })()
        with mock.patch.object(ra.time, "monotonic", return_value=101.0):
            ra.log_output(proc, "CMD", log, 100.0, state)
        self.assertEqual(state.connected_at_by_peer, {})
        self.assertFalse(state.connected)

    def test_disconnect_then_reconnect_requires_simultaneous_peer_state(self):
        state = ra.State()
        log = io.StringIO()

        def feed(label, text, now):
            proc = type("Proc", (), {
                "stdout": io.BytesIO((text + "\n").encode())
            })()
            with mock.patch.object(ra.time, "monotonic", return_value=now):
                ra.log_output(proc, label, log, 100.0, state)

        feed("CMD", "link_status:Connected to TESTB", 101.0)
        feed("CMD", "link_status:Disconnected", 101.5)
        feed("RSP", "link_status:Connected to TESTA", 102.0)
        self.assertFalse(state.connected)
        feed("CMD", "link_status:Connected to TESTB", 103.0)
        self.assertTrue(state.connected)
        self.assertEqual(
            state.connected_at_by_peer, {"CMD": 3.0, "RSP": 2.0})

    def test_repeated_connected_status_does_not_move_transition_edge(self):
        state = ra.State()
        log = io.StringIO()

        def feed(label, now):
            proc = type("Proc", (), {
                "stdout": io.BytesIO(
                    b"link_status:Connected to TEST ID= 0\n")
            })()
            with mock.patch.object(ra.time, "monotonic", return_value=now):
                ra.log_output(proc, label, log, 100.0, state)

        feed("RSP", 101.0)
        feed("RSP", 105.0)
        feed("CMD", 106.0)
        self.assertEqual(
            state.connected_at_by_peer, {"RSP": 1.0, "CMD": 6.0})
        self.assertEqual(
            state.first_connected_at_by_peer, {"RSP": 1.0, "CMD": 6.0})

    def test_live_harness_has_no_wall_clock_deadline_calls(self):
        with open(ra.__file__, encoding="utf-8") as source_file:
            source = source_file.read()
        self.assertNotIn("time.time(", source)


class FixedWindowTests(unittest.TestCase):
    def test_invalid_window_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            score_fixed_windows(
                request_at_s=0,
                connected_at_by_peer={},
                observed_until_s=0,
                delivery_events=[],
                link_events=[],
                horizon_s=30,
                steady_warmup_s=30,
            )

    def test_exact_peer_connect_and_fixed_snapshot_boundaries(self):
        request = 100.0
        connected = {"CMD": 105.2, "RSP": 105.8}
        events = [
            delivery(100.0, 0),
            delivery(135.79, 800),
            delivery(135.8, 1000),
            delivery(700.0, 9000),
            delivery(705.8, 10000),
        ]
        link_events = [
            link(110, "tx", 16, 1), link(111, "ack", 16, 1),
            link(118, "tx", 16, 2), link(119, "ack", 16, 2),
            link(126, "tx", 16, 3), link(127, "ack", 16, 3),
        ]
        score = score_fixed_windows(
            request_at_s=request,
            connected_at_by_peer=connected,
            observed_until_s=705.8,
            delivery_events=events,
            link_events=link_events,
        )
        self.assertEqual(score["connected_at_by_peer_s"], connected)
        self.assertEqual(score["connected_at_s"], 105.8)
        self.assertEqual(score["ux"]["window_s"], 600.0)
        self.assertEqual(score["ux"]["good_prefix_bytes"], 9000)
        self.assertEqual(score["steady"]["start_good_prefix_bytes"], 1000)
        self.assertEqual(score["steady"]["end_good_prefix_bytes"], 10000)
        self.assertEqual(score["steady"]["good_prefix_bytes"], 9000)
        self.assertEqual(score["steady"]["window_s"], 570.0)
        self.assertEqual(score["steady"]["sustainable_config"], 16)
        self.assertEqual(score["steady"]["status"], "OK")

    def test_window_endpoint_inclusion_is_start_exclusive_end_inclusive(self):
        score = score_fixed_windows(
            request_at_s=0.0,
            connected_at_by_peer={"CMD": 1.0, "RSP": 1.0},
            observed_until_s=601.0,
            delivery_events=[
                delivery(31.0, 100),
                delivery(31.000001, 200),
                delivery(601.0, 300),
                delivery(601.000001, 400),
            ],
            link_events=[
                link(2, "tx", 16, 1), link(3, "ack", 16, 1),
                link(4, "tx", 16, 2), link(31, "ack", 16, 2),
            ],
        )
        self.assertEqual(score["steady"]["status"], "OK")
        self.assertEqual(score["steady"]["start_good_prefix_bytes"], 100)
        self.assertEqual(score["steady"]["end_good_prefix_bytes"], 300)
        self.assertEqual(score["steady"]["good_prefix_bytes"], 200)

    def test_observation_and_payload_boundaries_are_exact(self):
        common = dict(
            request_at_s=0.0,
            connected_at_by_peer={},
            delivery_events=[],
            link_events=[],
        )
        exact = score_fixed_windows(observed_until_s=600.0, **common)
        early = score_fixed_windows(observed_until_s=599.999999, **common)
        complete_at_end = score_fixed_windows(
            observed_until_s=600.0,
            completion_at_s=600.0,
            **{k: v for k, v in common.items() if k != "observed_until_s"},
        )
        self.assertEqual(exact["ux"]["status"], "OK")
        self.assertEqual(early["ux"]["status"], "INCOMPLETE_SCORE_WINDOW")
        self.assertFalse(complete_at_end["instrument_invalid"])

    def test_early_stall_keeps_full_denominator(self):
        score = score_fixed_windows(
            request_at_s=0.0,
            connected_at_by_peer={"CMD": 1.0, "RSP": 1.0},
            observed_until_s=601.0,
            delivery_events=[delivery(0, 0), delivery(10, 1000)],
            link_events=[
                link(2, "tx", 5, 1), link(3, "ack", 5, 1),
                link(4, "tx", 5, 2), link(5, "ack", 5, 2),
                link(6, "tx", 5, 3), link(7, "ack", 5, 3),
            ],
        )
        self.assertEqual(score["ux"]["window_s"], 600.0)
        self.assertEqual(score["ux"]["good_prefix_bytes"], 1000)
        self.assertAlmostEqual(score["ux"]["content_Bmin"], 100.0)
        self.assertEqual(score["steady"]["window_s"], 570.0)
        self.assertEqual(score["steady"]["good_prefix_bytes"], 0)

    def test_nonconnect_is_zero_ux_and_has_no_steady_window(self):
        score = score_fixed_windows(
            request_at_s=0.0,
            connected_at_by_peer={},
            observed_until_s=600.0,
            delivery_events=[],
            link_events=[],
        )
        self.assertEqual(score["ux"]["status"], "OK")
        self.assertEqual(score["ux"]["good_prefix_bytes"], 0)
        self.assertEqual(score["ux"]["content_Bmin"], 0.0)
        self.assertEqual(score["steady"]["status"], "NO_CONNECT")

    def test_no_sustainable_rung_by_thirty_is_loud(self):
        score = score_fixed_windows(
            request_at_s=0.0,
            connected_at_by_peer={"CMD": 1.0, "RSP": 1.0},
            observed_until_s=601.0,
            delivery_events=[],
            link_events=[
                link(2, "tx", 4, 1), link(3, "ack", 4, 1),
                link(10, "tx", 4, 2),
            ],
        )
        self.assertEqual(score["steady"]["status"], "NOT_STEADY_BY_30")
        self.assertIsNone(score["steady"]["sustainable_config"])

    def test_payload_completion_before_endpoint_invalidates_instrument(self):
        score = score_fixed_windows(
            request_at_s=0.0,
            connected_at_by_peer={"CMD": 1.0, "RSP": 1.0},
            observed_until_s=100.0,
            delivery_events=[delivery(100, 4096)],
            link_events=[],
            payload_target=4096,
            completion_at_s=100.0,
        )
        self.assertTrue(score["instrument_invalid"])
        self.assertIn("payload_too_small", score["instrument_invalid_reasons"])

    def test_input_socket_exhaustion_alone_is_only_diagnostic(self):
        score = score_fixed_windows(
            request_at_s=0.0,
            connected_at_by_peer={"CMD": 1.0, "RSP": 1.0},
            observed_until_s=601.0,
            delivery_events=[delivery(100, 3000)],
            link_events=[],
            payload_target=4096,
            payload_exhausted_at_s=20.0,
        )
        self.assertFalse(score["instrument_invalid"])
        self.assertEqual(score["instrument_invalid_reasons"], [])
        self.assertTrue(score["payload_exhausted_before_required_end"])

    def test_valid_result_requires_connection(self):
        row = {
            "connected": False,
            "byte_integrity_ok": True,
            "uniqueness_ok": True,
            "instrument_invalid": False,
            "vara_scoring_enabled": True,
        }
        self.assertFalse(is_valid_result(row))
        row["connected"] = True
        self.assertTrue(is_valid_result(row))

    def test_nonconnect_zero_remains_in_unconditional_ux_population(self):
        row = {
            "connected": False,
            "fixed_window_score_enabled": True,
            "byte_integrity_ok": True,
            "uniqueness_ok": True,
            "instrument_invalid": False,
            "canonical_fixed_score": {
                "instrument_invalid": False,
                "ux": {
                    "status": "OK",
                    "scorable": True,
                    "content_Bmin": 0.0,
                },
                "steady": {"status": "NO_CONNECT", "scorable": False},
            },
        }
        self.assertTrue(is_ux_population_result(row))
        row["canonical_fixed_score"]["ux"]["status"] = (
            "INCOMPLETE_SCORE_WINDOW")
        self.assertFalse(is_ux_population_result(row))


class Lane9AuditAdapterTests(unittest.TestCase):
    @staticmethod
    def envelope(connected=True):
        cell = {
            "id": "commission-one",
            "score_horizon_s": 60.0,
            "steady_warmup_s": 30.0,
            "fixed_window_protocol_canonical": False,
        }
        peers = {"CMD": 1.0, "RSP": 1.0} if connected else {}
        events = [
            delivery(31.0, 100),
            delivery(61.0, 400),
        ] if connected else []
        links = [
            link(2, "tx", 16, 1), link(3, "ack", 16, 1),
            link(4, "tx", 16, 2), link(5, "ack", 16, 2),
        ] if connected else []
        score = score_fixed_windows(
            request_at_s=0.0,
            connected_at_by_peer=peers,
            observed_until_s=61.0 if connected else 60.0,
            delivery_events=events,
            link_events=links,
            horizon_s=60.0,
            steady_warmup_s=30.0,
        )
        raw = {
            "fixed_window_score_enabled": True,
            "fixed_window_protocol_canonical": False,
            "canonical_fixed_score": score,
            "instrument_invalid": False,
            "byte_integrity_ok": True,
            "uniqueness_ok": True,
            "connected": connected,
        }
        return cell, {"cell_id": cell["id"], "cell": cell, "result": raw}

    def test_adapter_accepts_exact_noncanonical_commission_geometry(self):
        cell, envelope = self.envelope()
        self.assertEqual(
            audit_campaign_cell(cell, envelope, {"campaign_id": "test"}), [])

    def test_adapter_accepts_complete_nonconnect_as_zero_ux(self):
        cell, envelope = self.envelope(connected=False)
        self.assertEqual(
            audit_campaign_cell(cell, envelope, {"campaign_id": "test"}), [])

    def test_adapter_rejects_tampered_byte_delta(self):
        cell, envelope = self.envelope()
        envelope["result"]["canonical_fixed_score"]["steady"][
            "good_prefix_bytes"] += 1
        errors = audit_campaign_cell(cell, envelope, {})
        self.assertIn("steady good-prefix delta mismatch", errors)


class SessionAwareOracleTests(unittest.TestCase):
    """Reproduce the R5 contention-cert cross-session oracle shear and prove the
    session-aware scorer clears it.

    R5 specimen j08_stress_w8_l75_v00_s00005 (cfg100, random-binary traffic):
    75% CPU stress starved the RX capture thread, so the transfer was torn down
    and restarted repeatedly.  The delivered data socket carried session 1's
    24-byte prefix, then -- after a control-socket DISCONNECTED and a fresh
    START_CONNECTION -- session 3's 18432-byte restart from payload offset 0.
    The cumulative oracle compared the restart against expected[24..] and
    reported a false byte-corruption: first_bad_offset=24, rx_bytes=18456,
    integrity FAIL.  Mercury delivered zero wrong bytes within any session.
    """

    TRAFFIC = "random-binary"
    S1 = 24
    S3 = 18432

    def _expected(self):
        return lambda offset, length: ca.traffic_expected_slice(
            self.TRAFFIC, offset, length)

    def _session1(self):
        return ca.traffic_slice(self.TRAFFIC, 0, self.S1)

    def _session3(self):
        return ca.traffic_slice(self.TRAFFIC, 0, self.S3)

    def test_cumulative_oracle_reproduces_the_false_cross_session_shear(self):
        # This is the meter R5 shipped; it must reproduce the false verdict so
        # the fix is anchored to a demonstrated bug, not a hypothetical one.
        oracle = DeliveryOracle(self._expected())
        oracle.feed(1.0, self._session1())
        # A DISCONNECTED tore session 1 down; session 3 restarts from offset 0.
        # The cumulative oracle has no boundary concept, so it keeps counting.
        oracle.feed(2.0, self._session3())
        snap = oracle.snapshot()
        self.assertEqual(snap["received_bytes"], self.S1 + self.S3)  # 18456
        self.assertEqual(snap["first_bad_offset"], self.S1)          # 24
        # Deterministic for this traffic source (SHA-256 counter stream): the
        # 18432-byte restart mismatches expected[24..] everywhere the stream
        # does not coincidentally repeat, ~1/256.  The R5 res JSON recorded
        # 18369 -- a 4-byte difference because the delivered bytes were never
        # persisted, so the drill reconstructed the ledger from the seam log;
        # the shear SIGNATURE (first_bad=24, rx=18456, integrity FAIL) matches.
        self.assertEqual(snap["mismatch_bytes"], 18373)
        self.assertGreater(snap["mismatch_bytes"], 18000)
        self.assertNotEqual(snap["mismatch_bytes"], 0)  # integrity would FAIL

    def test_session_aware_streaming_oracle_clears_the_shear(self):
        oracle = SessionAwareDeliveryOracle(self._expected())
        oracle.feed(1.0, self._session1())
        oracle.mark_disconnected(1.5)
        oracle.feed(2.0, self._session3())
        snap = oracle.snapshot()
        self.assertTrue(snap["byte_integrity_ok"])
        self.assertEqual(snap["aggregate_mismatch_bytes"], 0)
        self.assertIsNone(snap["first_bad_session"])
        self.assertEqual(snap["session_count"], 2)
        self.assertEqual(
            [s["delivered_bytes"] for s in snap["sessions"]],
            [self.S1, self.S3])
        self.assertEqual(snap["total_received_bytes"], self.S1 + self.S3)

    def test_session_aware_posthoc_rederivation_clears_the_shear(self):
        # The authoritative cert meter: re-derive from the persisted delivered
        # stream + the recorded boundary offsets (rx byte count at each
        # DISCONNECTED), never trusting the live boolean.
        delivered = self._session1() + self._session3()
        snap = score_delivered_sessions(
            delivered, [self.S1], self._expected())
        self.assertTrue(snap["byte_integrity_ok"])
        self.assertEqual(snap["session_count"], 2)
        self.assertEqual(
            [s["delivered_bytes"] for s in snap["sessions"]],
            [self.S1, self.S3])

    def test_posthoc_collapses_empty_and_duplicate_boundaries(self):
        # The live path records res["rx"] at EVERY DISCONNECTED, including a
        # session that delivered 0 bytes (session 2 in the R5 ledger) and both
        # peers' DISCONNECTED lines.  Empty/duplicate cuts must collapse.
        delivered = self._session1() + self._session3()
        snap = score_delivered_sessions(
            delivered, [self.S1, self.S1, self.S1 + self.S3], self._expected())
        self.assertTrue(snap["byte_integrity_ok"])
        self.assertEqual(snap["session_count"], 2)

    def test_session_aware_still_catches_a_real_within_session_hole(self):
        # The fix must not blind the meter to genuine corruption.  A within-
        # session backward shift (a dropped/re-delivered batch) still lands a
        # wrong offset inside its own session and must FAIL integrity.
        oracle = SessionAwareDeliveryOracle(self._expected())
        oracle.feed(1.0, ca.traffic_slice(self.TRAFFIC, 0, 64))
        # Next segment starts five bytes behind its session offset.
        oracle.feed(2.0, ca.traffic_slice(self.TRAFFIC, 59, 32))
        snap = oracle.snapshot()
        self.assertFalse(snap["byte_integrity_ok"])
        self.assertIsNotNone(snap["first_bad_session"])
        self.assertEqual(snap["first_bad_session"]["session_index"], 0)
        self.assertEqual(snap["first_bad_session"]["session_offset"], 64)

    def test_posthoc_catches_a_real_within_session_hole(self):
        # Same corruption via the post-hoc path: one continuous session whose
        # bytes deviate from payload[0..] must FAIL.
        delivered = (ca.traffic_slice(self.TRAFFIC, 0, 64)
                     + ca.traffic_slice(self.TRAFFIC, 59, 32))
        snap = score_delivered_sessions(delivered, [], self._expected())
        self.assertFalse(snap["byte_integrity_ok"])
        self.assertEqual(snap["session_count"], 1)
        self.assertEqual(snap["first_bad_session"]["session_offset"], 64)

    def test_clean_single_session_passes_both_meters(self):
        # Negative control: an unbroken 40 KiB transfer must pass on both the
        # cumulative and the session-aware meters (the fix does not manufacture
        # passes for genuinely clean streams).
        clean = ca.traffic_slice(self.TRAFFIC, 0, 40960)
        cumulative = DeliveryOracle(self._expected())
        cumulative.feed(1.0, clean)
        self.assertEqual(cumulative.snapshot()["mismatch_bytes"], 0)
        session = SessionAwareDeliveryOracle(self._expected())
        session.feed(1.0, clean)
        self.assertTrue(session.snapshot()["byte_integrity_ok"])
        self.assertTrue(
            score_delivered_sessions(clean, [], self._expected())[
                "byte_integrity_ok"])


if __name__ == "__main__":
    unittest.main(verbosity=2)

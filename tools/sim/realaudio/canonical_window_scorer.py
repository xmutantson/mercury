#!/usr/bin/env python3
"""Canonical fixed-window scorer and byte-exact partial-delivery oracle.

All timestamps accepted here are from one monotonic clock.  The scorer never
shrinks a denominator to the last delivery, a process exit, or a failed connect:

* UX: ``CONNECT`` request through request + 600 seconds.
* Sustainable rung: good-prefix delta from connected + 30 through connected +
  600 seconds, after the production anchor proof at one config by +30
  (one consecutive clean batch for ROBUST, two for OFDM).

The functions are pure apart from the two live helpers, so campaign reductions
can be tested without waiting ten minutes or opening audio hardware.
"""
import bisect
import re
import threading


_CMD_TX_RE = re.compile(
    r"\[CMD-TX\]\s+CONFIG_(\d+)\s+batch=(\d+)\s+type=(\d+)")
_DATA_FRAME_TYPES = frozenset((0x10, 0x11))
_CLEAN_ACK_RE = re.compile(
    r"\[(?:CMD-MFSK-ACK-SACK|CMD-COMPACT-CONFIRM)\].*\bCLEAN\b")
_PARTIAL_ACK_RE = re.compile(
    r"\[(?:CMD-MFSK-ACK-SACK|CMD-SACK-V2|CMD-COMPACT-CONFIRM)\]"
    r".*(?:\bPARTIAL\b|decoded SACK_RSP)")
_BARE_ACK_RE = re.compile(
    r"\[CMD-ACK-PAT\]\s+Data ACK pattern detected!")
_BREAK_RE = re.compile(r"\[BREAK\]")
_DISCONNECT_RE = re.compile(
    r"^(?:link_status:Disconnected(?:\s|$)|DISCONNECTED(?:\s|$))")


def _round(value):
    return round(float(value), 6)


def _rate_bmin(byte_count, seconds):
    return round(float(byte_count) * 60.0 / float(seconds), 3)


class DeliveryOracle:
    """Verify a nonperiodic delivered stream and timestamp its good prefix.

    ``expected_slice(offset, length)`` must return the exact application bytes
    expected at that absolute offset.  Once a mismatch occurs, later bytes can
    never extend the trustworthy prefix even if they happen to match again.
    """

    def __init__(self, expected_slice):
        self._expected_slice = expected_slice
        self._lock = threading.Lock()
        self.received_bytes = 0
        self.first_bad_offset = None
        self.mismatch_bytes = 0
        self.mismatch_segments = 0
        self.events = []
        self._last_at_s = None

    @property
    def good_prefix_bytes(self):
        return (self.received_bytes if self.first_bad_offset is None
                else self.first_bad_offset)

    def feed(self, at_s, data):
        """Verify one delivered segment and record an event at ``at_s``."""
        at_s = float(at_s)
        data = bytes(data)
        with self._lock:
            if self._last_at_s is not None and at_s < self._last_at_s:
                raise ValueError("delivery timestamps must be monotonic")
            base = self.received_bytes
            expected = self._expected_slice(base, len(data))
            if len(expected) != len(data):
                raise ValueError("expected_slice returned the wrong length")
            if data != expected:
                self.mismatch_segments += 1
                bad_positions = [
                    index for index, (got, want) in enumerate(zip(data, expected))
                    if got != want
                ]
                self.mismatch_bytes += len(bad_positions)
                if self.first_bad_offset is None:
                    self.first_bad_offset = base + bad_positions[0]
            self.received_bytes += len(data)
            self._last_at_s = at_s
            event = {
                "at_s": at_s,
                "segment_base": base,
                "received_bytes": self.received_bytes,
                "good_prefix_bytes": self.good_prefix_bytes,
            }
            self.events.append(event)
            result = dict(event)
            if data != expected:
                # Returned only to the live mismatch-capture path; never retained
                # in the delivery-event timeline.
                result["mismatch_expected"] = expected
            return result

    def snapshot(self):
        with self._lock:
            return {
                "received_bytes": self.received_bytes,
                "first_bad_offset": self.first_bad_offset,
                "mismatch_bytes": self.mismatch_bytes,
                "mismatch_segments": self.mismatch_segments,
                "good_prefix_bytes": self.good_prefix_bytes,
                "events": [dict(row) for row in self.events],
            }


class SessionAwareDeliveryOracle:
    """Byte-exact delivery verifier that segments the stream at session breaks.

    ``DeliveryOracle`` above scores one delivered stream against a single
    expected offset that never rewinds.  That is correct only while the transfer
    is one continuous ARQ session.  When a contiguity guard tears the session
    down -- an out-of-band ``DISCONNECTED`` on the control socket -- and the peer
    re-proposes the transfer from application offset 0, the delivered data socket
    carries a FRESH prefix of the payload.  The cumulative oracle compares that
    fresh prefix against the running expected offset, so a clean restart looks
    like a uniform positional shift: a byte-corruption verdict for delivery that
    was byte-exact on the wire.  That is exactly the false positive the R5
    contention cert hit (``first_bad_offset=24``, ``mismatch_bytes~=18.4k`` on a
    stream that was two clean sessions of 24 and 18432 bytes).

    This scorer resets the expected offset to 0 at every session boundary and
    verifies each session against the payload from offset 0 -- the contract
    Mercury actually honours (each session restarts the transfer; the resume
    layer owns re-proposal).  Integrity holds iff every session delivered a
    byte-exact prefix of its own payload.  A genuine within-session hole,
    reorder, or double-delivery still lands a wrong offset inside its session and
    is caught.

    Boundaries are applied lazily on the next ``feed`` so that back-to-back
    boundary signals (both peers emit ``DISCONNECTED``) and a boundary with no
    following delivery collapse to a single break with no spurious empty session.
    """

    def __init__(self, expected_slice):
        self._expected_slice = expected_slice
        self._lock = threading.Lock()
        self.total_received_bytes = 0
        self.session_index = 0
        self._session_received_bytes = 0
        self._session_first_bad_offset = None
        self._session_mismatch_bytes = 0
        self.aggregate_mismatch_bytes = 0
        self.aggregate_mismatch_segments = 0
        self.first_bad_session = None
        self._finalized_sessions = []
        self._boundary_pending = False
        self._last_at_s = None

    @property
    def byte_integrity_ok(self):
        return self.aggregate_mismatch_bytes == 0

    def mark_disconnected(self, at_s):
        """Signal a session boundary; the next delivered byte starts a fresh
        session at payload offset 0."""
        at_s = float(at_s)
        with self._lock:
            if self._session_received_bytes > 0:
                self._boundary_pending = True
            if self._last_at_s is None or at_s > self._last_at_s:
                self._last_at_s = at_s

    def _finalize_session_locked(self):
        self._finalized_sessions.append({
            "session_index": self.session_index,
            "delivered_bytes": self._session_received_bytes,
            "first_bad_offset": self._session_first_bad_offset,
            "mismatch_bytes": self._session_mismatch_bytes,
        })
        self.session_index += 1
        self._session_received_bytes = 0
        self._session_first_bad_offset = None
        self._session_mismatch_bytes = 0

    def feed(self, at_s, data):
        """Verify one delivered segment against the current session's payload."""
        at_s = float(at_s)
        data = bytes(data)
        with self._lock:
            if self._last_at_s is not None and at_s < self._last_at_s:
                raise ValueError("delivery timestamps must be monotonic")
            if self._boundary_pending:
                self._finalize_session_locked()
                self._boundary_pending = False
            base = self._session_received_bytes
            expected = self._expected_slice(base, len(data))
            if len(expected) != len(data):
                raise ValueError("expected_slice returned the wrong length")
            if data != expected:
                self.aggregate_mismatch_segments += 1
                bad_positions = [
                    index for index, (got, want) in enumerate(zip(data, expected))
                    if got != want
                ]
                self._session_mismatch_bytes += len(bad_positions)
                self.aggregate_mismatch_bytes += len(bad_positions)
                if self._session_first_bad_offset is None:
                    self._session_first_bad_offset = base + bad_positions[0]
                if self.first_bad_session is None:
                    self.first_bad_session = {
                        "session_index": self.session_index,
                        "session_offset": base + bad_positions[0],
                    }
            self._session_received_bytes += len(data)
            self.total_received_bytes += len(data)
            self._last_at_s = at_s
            return {
                "at_s": at_s,
                "session_index": self.session_index,
                "session_offset": base,
                "session_received_bytes": self._session_received_bytes,
                "total_received_bytes": self.total_received_bytes,
            }

    def _current_sessions_locked(self):
        sessions = [dict(row) for row in self._finalized_sessions]
        if self._session_received_bytes > 0:
            sessions.append({
                "session_index": self.session_index,
                "delivered_bytes": self._session_received_bytes,
                "first_bad_offset": self._session_first_bad_offset,
                "mismatch_bytes": self._session_mismatch_bytes,
            })
        return sessions

    def snapshot(self):
        with self._lock:
            return {
                "byte_integrity_ok": self.aggregate_mismatch_bytes == 0,
                "total_received_bytes": self.total_received_bytes,
                "session_count": (
                    len(self._finalized_sessions)
                    + (1 if self._session_received_bytes > 0 else 0)),
                "aggregate_mismatch_bytes": self.aggregate_mismatch_bytes,
                "aggregate_mismatch_segments": self.aggregate_mismatch_segments,
                "first_bad_session": (
                    dict(self.first_bad_session)
                    if self.first_bad_session is not None else None),
                "sessions": self._current_sessions_locked(),
            }


def score_delivered_sessions(delivered_bytes, boundary_offsets, expected_slice):
    """Session-aware integrity re-derivation from a persisted delivered stream.

    ``delivered_bytes`` is the full byte string the data socket carried across
    every ARQ session of one cell.  ``boundary_offsets`` is the list of
    cumulative delivered-byte counts recorded at each control-socket
    ``DISCONNECTED`` (in delivered-stream coordinates).  The stream is split at
    those offsets and each non-empty segment is verified against the payload from
    offset 0 via ``expected_slice``.  This is the authoritative meter the cert's
    booking layer runs over ``rx.bin`` -- it derives the verdict from raw bytes
    rather than trusting the live oracle's boolean.

    Returns the same schema as ``SessionAwareDeliveryOracle.snapshot``.
    """
    delivered = bytes(delivered_bytes)
    total = len(delivered)
    cuts = sorted(
        set(int(offset) for offset in (boundary_offsets or [])
            if 0 < int(offset) < total))
    bounds = [0] + cuts + [total]
    sessions = []
    aggregate_mismatch_bytes = 0
    aggregate_mismatch_segments = 0
    first_bad_session = None
    session_index = 0
    for start, stop in zip(bounds, bounds[1:]):
        if stop <= start:
            continue
        segment = delivered[start:stop]
        expected = expected_slice(0, len(segment))
        if len(expected) != len(segment):
            raise ValueError("expected_slice returned the wrong length")
        bad_positions = [
            index for index, (got, want) in enumerate(zip(segment, expected))
            if got != want
        ]
        first_bad = bad_positions[0] if bad_positions else None
        if bad_positions:
            aggregate_mismatch_bytes += len(bad_positions)
            aggregate_mismatch_segments += 1
            if first_bad_session is None:
                first_bad_session = {
                    "session_index": session_index,
                    "session_offset": first_bad,
                }
        sessions.append({
            "session_index": session_index,
            "delivered_bytes": len(segment),
            "first_bad_offset": first_bad,
            "mismatch_bytes": len(bad_positions),
        })
        session_index += 1
    return {
        "byte_integrity_ok": aggregate_mismatch_bytes == 0,
        "total_received_bytes": total,
        "session_count": len(sessions),
        "aggregate_mismatch_bytes": aggregate_mismatch_bytes,
        "aggregate_mismatch_segments": aggregate_mismatch_segments,
        "first_bad_session": first_bad_session,
        "sessions": sessions,
    }


class LinkEventTracker:
    """Convert live Mercury log lines into event-anchored rung evidence."""

    def __init__(self):
        self.events = []
        self._sequence = 0
        self._lock = threading.Lock()

    def _append(self, at_s, kind, config=None, batch=None):
        event = {
            "at_s": float(at_s),
            "kind": kind,
            "_sequence": self._sequence,
        }
        self._sequence += 1
        if config is not None:
            event["config"] = int(config)
        if batch is not None:
            event["batch"] = int(batch)
        self.events.append(event)

    def observe(self, label, text, at_s):
        """Observe one already-timestamped log line.

        This method records raw markers only. ACK-to-DATA attribution happens
        later, after events from both stdout reader threads are ordered by their
        captured monotonic timestamps. Mutating pending state in callback lock
        order would let a later RSP BREAK erase an earlier CMD ACK.
        """
        at_s = float(at_s)
        with self._lock:
            if _BREAK_RE.search(text):
                self._append(at_s, "break")
            if _DISCONNECT_RE.search(text):
                self._append(at_s, "disconnect")

            if label == "CMD":
                match = _CMD_TX_RE.search(text)
                if match:
                    config, batch, frame_type = map(int, match.groups())
                    # send_batch() also logs CONTROL batches. Only DATA_LONG and
                    # DATA_SHORT frames can seed the pending DATA/ACK tracker.
                    if frame_type in _DATA_FRAME_TYPES:
                        self._append(at_s, "tx", config, batch)
                if _PARTIAL_ACK_RE.search(text):
                    self._append(at_s, "partial_ack_marker")
                elif (_CLEAN_ACK_RE.search(text) or _BARE_ACK_RE.search(text)):
                    self._append(at_s, "ack_marker")

    def snapshot(self):
        with self._lock:
            return normalize_link_events(self.events)


def normalize_link_events(events):
    """Order raw cross-thread markers and bind ACKs to pending DATA.

    BREAK/disconnect wins ties conservatively. Other equal-time events retain
    their source insertion order, which preserves one CMD reader's stdout order.
    """
    rows = sorted(
        (dict(row) for row in events),
        key=lambda row: (
            float(row["at_s"]),
            1 if row["kind"] in ("break", "disconnect") else 0,
            int(row.get("_sequence", 0)),
        ))
    normalized = []
    pending = None
    for row in rows:
        kind = row["kind"]
        at_s = float(row["at_s"])
        if kind == "tx":
            if pending is not None:
                normalized.append({
                    "at_s": at_s,
                    "kind": "miss",
                    "config": pending[0],
                    "batch": pending[1],
                })
            pending = (row.get("config"), row.get("batch"))
            normalized.append({
                key: value for key, value in row.items()
                if key != "_sequence"
            })
        elif kind == "ack_marker":
            if pending is not None:
                normalized.append({
                    "at_s": at_s,
                    "kind": "ack",
                    "config": pending[0],
                    "batch": pending[1],
                })
                pending = None
        elif kind == "partial_ack_marker":
            if pending is not None:
                normalized.append({
                    "at_s": at_s,
                    "kind": "partial_ack",
                    "config": pending[0],
                    "batch": pending[1],
                })
                pending = None
        elif kind in ("break", "disconnect"):
            normalized.append({
                key: value for key, value in row.items()
                if key != "_sequence"
            })
            pending = None
    return normalized


def good_prefix_at(delivery_events, at_s):
    """Return the last trustworthy delivered-byte count at or before ``at_s``."""
    rows = sorted(delivery_events, key=lambda row: row["at_s"])
    times = [float(row["at_s"]) for row in rows]
    index = bisect.bisect_right(times, float(at_s)) - 1
    return int(rows[index]["good_prefix_bytes"]) if index >= 0 else 0


def _rung_state_at(link_events, boundary_s):
    current = None
    pending = None
    streak = 0
    stable_since = None
    for event in sorted(link_events, key=lambda row: row["at_s"]):
        if float(event["at_s"]) > boundary_s:
            break
        kind = event["kind"]
        if kind in ("break", "disconnect"):
            current, pending, streak, stable_since = None, None, 0, None
        elif kind == "config":
            config = event.get("config")
            if config != current:
                current, streak, stable_since = config, 0, None
            pending = None
        elif kind == "tx":
            config = event.get("config")
            if config != current:
                current, streak, stable_since = config, 0, None
            pending = (config, event.get("batch"))
        elif kind == "ack":
            config = event.get("config")
            if config != current:
                current, streak, stable_since = config, 0, None
            streak += 1
            pending = None
            threshold = 1 if (config is not None and config >= 100) else 2
            if streak == threshold:
                stable_since = float(event["at_s"])
        elif kind in ("miss", "partial_ack"):
            streak, stable_since, pending = 0, None, None
    return {
        "config": (
            current if current is not None
            and streak >= (1 if current >= 100 else 2)
            else None),
        "consecutive_acked_batches": streak,
        "stable_since_s": stable_since,
        "pending": pending,
    }


def _rung_disrupted(link_events, start_s, end_s, sustainable_config):
    for event in sorted(link_events, key=lambda row: row["at_s"]):
        at_s = float(event["at_s"])
        if at_s <= start_s or at_s > end_s:
            continue
        if event["kind"] in ("break", "disconnect", "miss", "partial_ack"):
            return True
        if (event["kind"] in ("config", "tx")
                and event.get("config") != sustainable_config):
            return True
    return False


def score_fixed_windows(
        request_at_s,
        connected_at_by_peer,
        observed_until_s,
        delivery_events,
        link_events,
        payload_target=None,
        completion_at_s=None,
        payload_exhausted_at_s=None,
        horizon_s=600.0,
        steady_warmup_s=30.0):
    """Score fixed UX and sustainable-rung windows from monotonic events."""
    request_at_s = float(request_at_s)
    observed_until_s = float(observed_until_s)
    horizon_s = float(horizon_s)
    steady_warmup_s = float(steady_warmup_s)
    if horizon_s <= 0:
        raise ValueError("horizon_s must be positive")
    if steady_warmup_s < 0 or steady_warmup_s >= horizon_s:
        raise ValueError(
            "steady_warmup_s must be nonnegative and smaller than horizon_s")
    peers = {
        str(peer): float(at_s)
        for peer, at_s in (connected_at_by_peer or {}).items()
        if at_s is not None
    }
    connected_at_s = (max(peers["CMD"], peers["RSP"])
                      if "CMD" in peers and "RSP" in peers else None)

    ux_start = request_at_s
    ux_end = request_at_s + horizon_s
    ux_start_bytes = good_prefix_at(delivery_events, ux_start)
    ux_end_bytes = good_prefix_at(delivery_events, ux_end)
    ux_bytes = max(0, ux_end_bytes - ux_start_bytes)
    ux_complete = observed_until_s >= ux_end
    ux = {
        "status": "OK" if ux_complete else "INCOMPLETE_SCORE_WINDOW",
        "scorable": ux_complete,
        "start_s": _round(ux_start),
        "end_s": _round(ux_end),
        "window_s": _round(horizon_s),
        "start_good_prefix_bytes": ux_start_bytes,
        "end_good_prefix_bytes": ux_end_bytes,
        "good_prefix_bytes": ux_bytes,
        "content_Bmin": _rate_bmin(ux_bytes, horizon_s),
    }

    if connected_at_s is None:
        steady = {
            "status": "NO_CONNECT",
            "scorable": False,
            "start_s": None,
            "end_s": None,
            "window_s": _round(horizon_s - steady_warmup_s),
            "start_good_prefix_bytes": None,
            "end_good_prefix_bytes": None,
            "good_prefix_bytes": 0,
            "content_Bmin": 0.0,
            "sustainable_config": None,
            "consecutive_acked_batches_by_30": 0,
            "stable_since_s": None,
        }
        required_end_s = ux_end
    else:
        steady_start = connected_at_s + steady_warmup_s
        steady_end = connected_at_s + horizon_s
        required_end_s = max(ux_end, steady_end)
        start_bytes = good_prefix_at(delivery_events, steady_start)
        end_bytes = good_prefix_at(delivery_events, steady_end)
        byte_delta = max(0, end_bytes - start_bytes)
        rung = _rung_state_at(link_events, steady_start)
        if observed_until_s < steady_end:
            status = "INCOMPLETE_SCORE_WINDOW"
        elif rung["config"] is None:
            status = "NOT_STEADY_BY_30"
        elif _rung_disrupted(
                link_events, steady_start, steady_end, rung["config"]):
            status = "UNSTABLE_AFTER_30"
        else:
            status = "OK"
        steady_seconds = horizon_s - steady_warmup_s
        steady = {
            "status": status,
            "scorable": status == "OK",
            "start_s": _round(steady_start),
            "end_s": _round(steady_end),
            "window_s": _round(steady_seconds),
            "start_good_prefix_bytes": start_bytes,
            "end_good_prefix_bytes": end_bytes,
            "good_prefix_bytes": byte_delta,
            "content_Bmin": _rate_bmin(byte_delta, steady_seconds),
            "sustainable_config": rung["config"],
            "consecutive_acked_batches_by_30":
                rung["consecutive_acked_batches"],
            "stable_since_s": (None if rung["stable_since_s"] is None
                               else _round(rung["stable_since_s"])),
        }

    invalid_reasons = []
    # Socket input exhaustion is diagnostic only: Mercury can queue offered
    # bytes long before they finish crossing the radio. Only full application
    # delivery proves that a finite payload truncated the measurement window.
    if (completion_at_s is not None
            and float(completion_at_s) < required_end_s):
        invalid_reasons.append("payload_too_small")
    if observed_until_s < required_end_s and completion_at_s is None:
        invalid_reasons.append("incomplete_score_window")

    return {
        "clock": "monotonic",
        "horizon_s": _round(horizon_s),
        "steady_warmup_s": _round(steady_warmup_s),
        "request_at_s": _round(request_at_s),
        "connected_at_by_peer_s": {
            peer: _round(at_s) for peer, at_s in sorted(peers.items())
        },
        "connected_at_s": (None if connected_at_s is None
                           else _round(connected_at_s)),
        "observed_until_s": _round(observed_until_s),
        "required_end_s": _round(required_end_s),
        "payload_target": payload_target,
        "completion_at_s": (None if completion_at_s is None
                            else _round(completion_at_s)),
        "payload_exhausted_at_s": (
            None if payload_exhausted_at_s is None
            else _round(payload_exhausted_at_s)),
        "payload_exhausted_before_required_end": bool(
            payload_exhausted_at_s is not None
            and float(payload_exhausted_at_s) < required_end_s),
        "instrument_invalid": bool(invalid_reasons),
        "instrument_invalid_reasons": invalid_reasons,
        "ux": ux,
        "steady": steady,
    }


def _audit_number(value, label, errors):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        errors.append(f"{label} is not numeric")
        return None
    value = float(value)
    if value != value or value in (float("inf"), float("-inf")):
        errors.append(f"{label} is not finite")
        return None
    return value


def _audit_close(got, wanted, label, errors, tolerance=1e-6):
    got_number = _audit_number(got, label, errors)
    if got_number is not None and abs(got_number - float(wanted)) > tolerance:
        errors.append(f"{label} mismatch")


def _audit_byte_window(window, start_s, end_s, label, errors):
    if not isinstance(window, dict):
        errors.append(f"{label} window missing")
        return
    _audit_close(window.get("start_s"), start_s, f"{label} start", errors)
    _audit_close(window.get("end_s"), end_s, f"{label} end", errors)
    seconds = end_s - start_s
    _audit_close(window.get("window_s"), seconds, f"{label} duration", errors)
    start_bytes = window.get("start_good_prefix_bytes")
    end_bytes = window.get("end_good_prefix_bytes")
    delta = window.get("good_prefix_bytes")
    for value, name in (
            (start_bytes, "start bytes"), (end_bytes, "end bytes"),
            (delta, "delta bytes")):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            errors.append(f"{label} {name} invalid")
    if (isinstance(start_bytes, int) and not isinstance(start_bytes, bool)
            and isinstance(end_bytes, int) and not isinstance(end_bytes, bool)
            and isinstance(delta, int) and not isinstance(delta, bool)):
        if end_bytes < start_bytes or delta != end_bytes - start_bytes:
            errors.append(f"{label} good-prefix delta mismatch")
        expected_rate = delta * 60.0 / seconds
        _audit_close(
            window.get("content_Bmin"), round(expected_rate, 3),
            f"{label} content rate", errors, tolerance=0.001)


def audit_campaign_cell(cell, envelope, manifest):
    """Independent Lane-9 adapter for one fixed-window result envelope.

    Returns a list of fail-closed errors.  It does not decide whether a modem
    outcome is fast enough; it proves that the scorer used the exact requested
    clock geometry, byte deltas, connection population, and validity gates.
    """
    errors = []
    if not isinstance(cell, dict) or not isinstance(envelope, dict):
        return ["cell/envelope is not a mapping"]
    raw = envelope.get("result")
    if not isinstance(raw, dict):
        return ["result mapping missing"]
    if envelope.get("cell_id") != cell.get("id"):
        errors.append("cell_id mismatch")
    if envelope.get("cell") != cell:
        errors.append("cell contract mismatch")

    score = raw.get("canonical_fixed_score")
    if not isinstance(score, dict):
        return errors + ["canonical_fixed_score missing"]
    if raw.get("fixed_window_score_enabled") is not True:
        errors.append("fixed-window scorer not enabled")
    expected_protocol = cell.get("fixed_window_protocol_canonical")
    if raw.get("fixed_window_protocol_canonical") is not expected_protocol:
        errors.append("fixed-window canonical flag mismatch")
    if score.get("clock") != "monotonic":
        errors.append("score clock is not monotonic")
    if score.get("instrument_invalid") is not False:
        errors.append("score instrument invalid")
    if score.get("instrument_invalid_reasons") not in ([], None):
        errors.append("score invalid reasons present")
    if raw.get("instrument_invalid") is not False:
        errors.append("raw instrument invalid")
    if raw.get("byte_integrity_ok") is not True:
        errors.append("byte integrity failed")
    if raw.get("uniqueness_ok") is not True:
        errors.append("delivery uniqueness failed")

    horizon = _audit_number(
        cell.get("score_horizon_s"), "cell score horizon", errors)
    warmup = _audit_number(
        cell.get("steady_warmup_s"), "cell steady warmup", errors)
    request = _audit_number(score.get("request_at_s"), "request time", errors)
    observed = _audit_number(
        score.get("observed_until_s"), "observation end", errors)
    if None in (horizon, warmup, request, observed):
        return errors
    if not (horizon > warmup >= 0):
        errors.append("cell window geometry invalid")
        return errors
    _audit_close(score.get("horizon_s"), horizon, "score horizon", errors)
    _audit_close(score.get("steady_warmup_s"), warmup, "score warmup", errors)

    ux = score.get("ux")
    _audit_byte_window(ux, request, request + horizon, "ux", errors)
    if isinstance(ux, dict):
        if ux.get("status") != "OK" or ux.get("scorable") is not True:
            errors.append("ux window incomplete/unscorable")

    peers = score.get("connected_at_by_peer_s")
    peers = peers if isinstance(peers, dict) else {}
    peer_times = {}
    for peer, value in peers.items():
        parsed = _audit_number(
            value, f"{peer} connected time", errors)
        if parsed is not None:
            peer_times[peer] = parsed
            if parsed < request:
                errors.append(f"{peer} connected before request")
    both_connected = "CMD" in peer_times and "RSP" in peer_times
    connected_at = max(peer_times["CMD"], peer_times["RSP"]) if both_connected else None
    if raw.get("connected") is not both_connected:
        errors.append("raw/scorer connection population mismatch")
    if connected_at is None:
        if score.get("connected_at_s") is not None:
            errors.append("aggregate connection time present without both peers")
        steady = score.get("steady")
        if not isinstance(steady, dict):
            errors.append("steady window missing")
        else:
            if steady.get("status") != "NO_CONNECT":
                errors.append("nonconnect steady status mismatch")
            if steady.get("scorable") is not False:
                errors.append("nonconnect steady window is scorable")
            if steady.get("good_prefix_bytes") != 0:
                errors.append("nonconnect steady bytes are nonzero")
        if isinstance(ux, dict) and ux.get("good_prefix_bytes") != 0:
            errors.append("nonconnect UX bytes are nonzero")
        required_end = request + horizon
    else:
        _audit_close(
            score.get("connected_at_s"), connected_at,
            "aggregate connection time", errors)
        steady_start = connected_at + warmup
        steady_end = connected_at + horizon
        steady = score.get("steady")
        _audit_byte_window(
            steady, steady_start, steady_end, "steady", errors)
        if isinstance(steady, dict):
            status = steady.get("status")
            allowed = {"OK", "NOT_STEADY_BY_30", "UNSTABLE_AFTER_30"}
            if status not in allowed:
                errors.append("steady status invalid/incomplete")
            if steady.get("scorable") is not (status == "OK"):
                errors.append("steady scorable/status mismatch")
            if (status == "NOT_STEADY_BY_30"
                    and steady.get("sustainable_config") is not None):
                errors.append("unproven steady rung has a config")
            if (status in ("OK", "UNSTABLE_AFTER_30")
                    and not isinstance(steady.get("sustainable_config"), int)):
                errors.append("proven steady rung config missing")
        required_end = max(request + horizon, steady_end)

    _audit_close(
        score.get("required_end_s"), required_end,
        "required endpoint", errors)
    if observed < required_end:
        errors.append("observation ended before required endpoint")
    completion = score.get("completion_at_s")
    if completion is not None:
        completion_value = _audit_number(
            completion, "payload completion time", errors)
        if completion_value is not None and completion_value < required_end:
            errors.append("payload completed before required endpoint")
    return errors

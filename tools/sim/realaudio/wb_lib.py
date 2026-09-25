#!/usr/bin/env python3
"""Shared machinery for the Mercury real-audio width-bisection runner."""

import concurrent.futures
import gzip
import hashlib
import json
import os
import shlex
import socket
import subprocess
import tarfile
import threading
import time
from pathlib import Path


SOURCE_REVISION = "5da51234a"
SCORE_HORIZON_S = 500
PAYLOAD_BYTES = 1048576
SNR3K = 28
START_CONFIGURATION = 100
TTL_S = 1500
HEARTBEAT_INTERVAL_S = 30
PREFLIGHT_HEALTH_MAX_AGE_S = 30
# Wire-visible provenance marker.  It is an enforcement/migration tag, not an
# authentication secret; the broker owns the allowlist and policy switch.
APPROVED_CLIENT_TAG = "wb-lib-v1"

# Cross-box scheduling boundary (design sections 2, 5, 9).  Widths <=24 use
# share=0 (one cell per physical card); widths >24 need share=1 and, at the
# extreme, a three-box width-48 load wave would consume all 72 physical-card
# safety units of the shared global pool.  The cross-box coordinator therefore
# lets at most ONE box run a LOAD wave WIDER than this bound at any instant,
# while every other box may still run its width-1 idle arm concurrently.  A
# load wave whose active width is <= this bound may run on all three boxes at
# once.  This equals the share boundary used by share_for_width below.
LOAD_CONCURRENCY_MAX = 24

# After the width-proof horizon closes, scored cells are STILL transmitting: an
# arq_realaudio cell writes result.json only at NATURAL completion of main(),
# and its run window (connect + --secs) closes AFTER the score horizon.  The
# runner must let each lease-owned runner exit and write result.json BEFORE the
# destructive teardown ladder SIGTERMs it -- arq_realaudio has no result-
# flushing SIGTERM handler, so a mid-run SIGTERM loses the cell's result.  This
# is the proven contention-ladder collect-then-teardown order.  The drain is
# BOUNDED so a wedged runner cannot stall the wave; an overrunning runner is
# force-stopped by teardown and recorded as a missing-result outcome (never
# dropped from the denominator).
CELL_DRAIN_GRACE_S = 180
CELL_DRAIN_POLL_S = 5.0

# Marker file written into a rung dir by the reduced/shakeout freeze path.
# Its presence forbids the scorer from emitting a capacity verdict.
NON_SCORABLE_MARKER = "NON_SCORABLE"

COMMON_ENV = (
    "MERCURY_RUNG_STEADY_REFRESH=1",
    "MERCURY_DEMOTE_TEMPORAL_HYSTERESIS=1",
    "MERCURY_SIM_PSIG_MODE=steady",
    "MERCURY_TURN_TRACE=1",
)

CLAIM_FIELDS = (
    "requested_width",
    "realized_active_width",
    "reserved_lease_width",
    "box",
    "physical_card",
    "slot",
    "share_mode",
    "peer_set",
    "score_horizon_start",
    "score_horizon_end",
    "width_proof_sample_denominator",
    "seed",
    "arm",
    "pair_order",
    "recipe",
    "input_hash",
    "source_revision",
    "binary_hashes",
)

REQUIRED_ASSETS = (
    "sim_channel_relay.py",
    "sim_axis.py",
    "realaudio/arq_realaudio.py",
    "realaudio/realaudio_bridge_s32_c",
    "realaudio/ra_cleanup.py",
)


class ArchivedAbort(RuntimeError):
    """An end-of-run condition whose reason has already been archived."""

    def __init__(self, reason, classification="INVALID", replaceable=False):
        super().__init__(reason)
        self.reason = str(reason)
        self.classification = classification
        self.replaceable = bool(replaceable)


class PrelaunchInvalid(ArchivedAbort):
    def __init__(self, reason):
        super().__init__(reason, classification="INVALID", replaceable=True)


class NonreplaceableInvalid(ArchivedAbort):
    def __init__(self, reason):
        super().__init__(reason, classification="INVALID", replaceable=False)


class ScoredAdverse(ArchivedAbort):
    def __init__(self, reason):
        super().__init__(reason, classification="FAIL", replaceable=False)


class ProtocolError(RuntimeError):
    pass


class GrantVerificationError(ProtocolError):
    pass


def utc_now():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def hash_file(path, algorithm="sha256"):
    digest = hashlib.new(algorithm)
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json_bytes(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode()


def atomic_write_bytes(path, data, mode=0o644):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(
        ".%s.%s.%s.tmp" % (path.name, os.getpid(), threading.get_ident())
    )
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except BaseException:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def atomic_write_json(path, value):
    atomic_write_bytes(path, json.dumps(
        value, indent=2, sort_keys=True
    ).encode("utf-8") + b"\n")


def append_jsonl(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = canonical_json_bytes(value)
    with open(path, "ab", buffering=0) as handle:
        handle.write(payload)
        os.fsync(handle.fileno())


def read_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def archive_reason(directory, phase, reason, classification="INVALID",
                   replaceable=False, evidence=None):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    row = {
        "archive_schema": 1,
        "time": utc_now(),
        "phase": str(phase),
        "classification": str(classification),
        "replaceable": bool(replaceable),
        "reason": str(reason),
        "evidence": evidence,
    }
    atomic_write_json(directory / "ABORT.json", row)
    return row


def abort(directory, phase, reason, classification="INVALID",
          replaceable=False, evidence=None):
    archive_reason(directory, phase, reason, classification, replaceable, evidence)
    if classification == "FAIL":
        raise ScoredAdverse(reason)
    if replaceable:
        raise PrelaunchInvalid(reason)
    raise NonreplaceableInvalid(reason)


def card_name(index):
    return "Loopback" if int(index) == 0 else "Loopback_%X" % int(index)


def validate_claim_record(record):
    missing = [field for field in CLAIM_FIELDS if field not in record]
    if missing:
        raise ValueError("claim record lacks required fields: %s" %
                         ", ".join(missing))
    if record["requested_width"] is None:
        raise ValueError("requested_width must not be null")
    return record


def claim_record(**values):
    record = dict(values)
    validate_claim_record(record)
    return record


def share_for_width(width, diagnostic_shared=False):
    width = int(width)
    if width < 1 or width > 48:
        raise ValueError("width must be in [1, 48]")
    return 1 if width > 24 or diagnostic_shared else 0


def physical_units_for_width(width, share):
    return (int(width) + 1) // 2 if int(share) else int(width)


def parse_fields(tokens):
    result = {}
    for token in tokens:
        if "=" not in token:
            raise ProtocolError("expected key=value token, got %r" % token)
        key, value = token.split("=", 1)
        if not key or key in result:
            raise ProtocolError("duplicate or empty field %r" % key)
        result[key] = value
    return result


def _tagged_lease_line(verb, subject, options=()):
    """Build a REQ/REL line and stamp the canonical client marker."""
    verb = str(verb).upper()
    if verb not in {"REQ", "REL"}:
        raise ValueError("approved-client tagging supports REQ and REL")
    subject = str(subject)
    tokens = [str(option) for option in options]
    if not subject or any(ch.isspace() for ch in subject):
        raise ValueError("lease subject must be one non-whitespace token")
    if any(token.startswith("client=") for token in tokens):
        raise ValueError("client tag is owned by wb_lib")
    return " ".join([verb, subject] + tokens +
                    ["client=%s" % APPROVED_CLIENT_TAG])


def tagged_request_line(agent, options):
    return _tagged_lease_line("REQ", agent, options)


def tagged_release_line(lease_id, options=()):
    return _tagged_lease_line("REL", lease_id, options)


class SocketLineTransport:
    def __init__(self, address, connect_timeout=15):
        self.sock = socket.create_connection(address, timeout=connect_timeout)
        self.sock.settimeout(None)
        self.stream = self.sock.makefile("rwb", buffering=0)

    def send(self, line):
        self.stream.write((line + "\n").encode("utf-8"))

    def receive(self):
        raw = self.stream.readline(65537)
        if not raw:
            raise ProtocolError("broker closed the connection")
        if len(raw) > 65536:
            raise ProtocolError("broker reply exceeded 65536 bytes")
        return raw.decode("utf-8", "replace").rstrip("\r\n")

    def close(self):
        try:
            self.stream.close()
        finally:
            self.sock.close()


class LeaseConnection:
    """One held broker socket and its verified single-cell lease."""

    def __init__(self, agent, box, share, broker=("192.168.2.31", 7800),
                 transport_factory=None, ttl=TTL_S):
        self.agent = str(agent)
        self.requested_box = int(box)
        self.requested_share = int(share)
        self.broker = broker
        self.transport_factory = transport_factory
        self.ttl = int(ttl)
        self.transport = None
        self.lock = threading.Lock()
        self.lease_id = None
        self.grant_raw = None
        self.fields = {}
        self.request_raw = None
        self.released = False
        self.heartbeat_ok = 0
        self.heartbeat_bad = []

    @property
    def granted(self):
        return self.lease_id is not None

    @property
    def box(self):
        return int(self.fields["box"])

    @property
    def card(self):
        return int(self.fields["cards"])

    @property
    def slot(self):
        return int(self.fields["slot"])

    def _open(self):
        if self.transport_factory is None:
            return SocketLineTransport(self.broker)
        return self.transport_factory()

    def transact(self, command):
        if self.transport is None:
            raise ProtocolError("lease connection is not open")
        with self.lock:
            self.transport.send(command)
            return self.transport.receive()

    def acquire(self, expected_card=None, expected_slot=None):
        self.transport = self._open()
        self.request_raw = tagged_request_line(self.agent, (
            "box=%d" % self.requested_box,
            "cards=1",
            "cpu=1",
            "ttl=%d" % self.ttl,
            "pin_reason=width-bisection-fixed-box",
            "class=measure",
            "share=%d" % self.requested_share,
        ))

        with self.lock:
            self.transport.send(self.request_raw)
            while True:
                reply = self.transport.receive()
                if reply.startswith("QUEUE "):
                    continue
                if not reply.startswith("GRANT "):
                    raise ProtocolError("unexpected broker reply: %s" % reply)
                parts = reply.split()
                if len(parts) < 3:
                    raise ProtocolError("malformed GRANT: %s" % reply)
                self.lease_id = parts[1]
                self.grant_raw = reply
                self.fields = parse_fields(parts[2:])
                break

        self.verify_grant(expected_card, expected_slot)
        return self

    def verify_grant(self, expected_card=None, expected_slot=None):
        required = {
            "box", "cards", "slot", "cpu", "ncards", "resource",
        }
        missing = sorted(required - set(self.fields))
        if missing:
            raise GrantVerificationError(
                "GRANT lacks echo field(s): %s" % ",".join(missing)
            )

        expected = {
            "box": str(self.requested_box),
            "cpu": "1",
            "ncards": "1",
        }
        wrong = {
            key: {"expected": value, "got": self.fields.get(key)}
            for key, value in expected.items()
            if self.fields.get(key) != value
        }

        try:
            card = int(self.fields["cards"])
            slot = int(self.fields["slot"])
        except ValueError as exc:
            raise GrantVerificationError("non-integer card or slot echo") from exc

        if card < 0 or slot not in (0, 1):
            wrong["card_slot_range"] = {"card": card, "slot": slot}
        if self.fields["resource"] != "%d:%d" % (card, slot):
            wrong["resource"] = {
                "expected": "%d:%d" % (card, slot),
                "got": self.fields["resource"],
            }
        if self.requested_share == 0 and slot != 0:
            wrong["exclusive_slot"] = {"expected": 0, "got": slot}
        if expected_card is not None and card != int(expected_card):
            wrong["cards"] = {"expected": int(expected_card), "got": card}
        if expected_slot is not None and slot != int(expected_slot):
            wrong["slot"] = {"expected": int(expected_slot), "got": slot}
        if wrong:
            raise GrantVerificationError(
                "GRANT echo verification failed: %s" %
                json.dumps(wrong, sort_keys=True)
            )

    def verify_status_class_share(self):
        """Verify class and share via broker STATUS lease_table.

        The real broker's GRANT does not echo class/share fields.
        Those are verified post-grant via the STATUS lease_table row.
        """
        reply = self.transact("STATUS")
        if not reply.startswith("OK "):
            raise GrantVerificationError("STATUS returned non-OK: %s" % reply)
        try:
            status = json.loads(reply[3:])
        except json.JSONDecodeError as exc:
            raise GrantVerificationError("STATUS JSON decode failed") from exc

        lease_table = status.get("lease_table", [])
        my_lease = None
        for row in lease_table:
            # Match by agent name since lease_id is not in the table
            # (the table shows agent/box/cards/class/share, not lease_id)
            if (row.get("name") == self.agent
                and row.get("box") == self.requested_box):
                my_lease = row
                break

        if my_lease is None:
            raise GrantVerificationError(
                "STATUS lease_table does not contain this lease "
                "(agent=%s box=%d)" % (self.agent, self.requested_box)
            )

        wrong = {}
        if my_lease.get("class") != "measure":
            wrong["class"] = {
                "expected": "measure",
                "got": my_lease.get("class"),
            }
        # share in STATUS is a boolean, but we requested 0 or 1
        expected_share = bool(self.requested_share)
        if my_lease.get("share") != expected_share:
            wrong["share"] = {
                "expected": expected_share,
                "got": my_lease.get("share"),
            }

        if wrong:
            raise GrantVerificationError(
                "STATUS lease_table verification failed: %s" %
                json.dumps(wrong, sort_keys=True)
            )

    def heartbeat(self):
        reply = self.transact("HB %s" % self.lease_id)
        if reply == "OK":
            self.heartbeat_ok += 1
        else:
            self.heartbeat_bad.append(reply)
        return reply

    def release(self):
        if not self.granted:
            self.close()
            return None
        if self.released:
            return "OK"
        reply = self.transact(tagged_release_line(self.lease_id))
        if reply != "OK":
            raise ProtocolError("REL %s returned %s" % (self.lease_id, reply))
        self.released = True
        self.close()
        return reply

    def close(self):
        if self.transport is not None:
            try:
                self.transport.close()
            finally:
                self.transport = None


class HeartbeatCoordinator:
    """The sole heartbeat thread for every lease held by a wave."""

    def __init__(self, interval=HEARTBEAT_INTERVAL_S):
        self.interval = float(interval)
        self.lock = threading.Lock()
        self.leases = []
        self.stop_event = threading.Event()
        self.lost_event = threading.Event()
        self.thread = None

    def add(self, lease):
        with self.lock:
            self.leases.append(lease)

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(
                target=self._run, name="wb-heartbeats", daemon=True
            )
            self.thread.start()

    def _run(self):
        while not self.stop_event.wait(self.interval):
            with self.lock:
                leases = list(self.leases)
            for lease in leases:
                if lease.released:
                    continue
                try:
                    if lease.heartbeat() != "OK":
                        self.lost_event.set()
                except Exception as exc:
                    lease.heartbeat_bad.append(repr(exc))
                    self.lost_event.set()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=max(2.0, self.interval + 1.0))


class WaveLeaseSet:
    """Atomic collection of every single-cell lease required by one wave."""

    def __init__(self, leases, heartbeats, wave_dir):
        self.leases = list(leases)
        self.heartbeats = heartbeats
        self.wave_dir = Path(wave_dir)
        self.released = False

    @property
    def reserved_width(self):
        return len(self.leases)

    @property
    def heartbeat_lost(self):
        return self.heartbeats.lost_event.is_set()

    def records(self):
        return [{
            "agent": lease.agent,
            "request": lease.request_raw,
            "grant": lease.grant_raw,
            "lease_id": lease.lease_id,
            "box": lease.box if lease.fields.get("box") else None,
            "physical_card": lease.card if lease.fields.get("cards") else None,
            "slot": lease.slot if lease.fields.get("slot") else None,
            "class": "measure",
            "share": str(lease.requested_share),
            "cpu": lease.fields.get("cpu"),
            "heartbeat_ok": lease.heartbeat_ok,
            "heartbeat_bad": list(lease.heartbeat_bad),
            "released": lease.released,
        } for lease in self.leases]

    @classmethod
    def acquire_atomic(cls, requests, wave_dir, broker=("192.168.2.31", 7800),
                       transport_factory=None, release_probe=None,
                       heartbeat_interval=HEARTBEAT_INTERVAL_S):
        wave_dir = Path(wave_dir)
        wave_dir.mkdir(parents=True, exist_ok=True)
        heartbeats = HeartbeatCoordinator(heartbeat_interval)
        heartbeats.start()
        leases = []
        failures = []
        lock = threading.Lock()

        def worker(spec):
            lease = LeaseConnection(
                spec["agent"], spec["box"], spec["share"],
                broker=broker, transport_factory=transport_factory,
            )
            try:
                lease.acquire(spec.get("expected_card"), spec.get("expected_slot"))
                with lock:
                    leases.append(lease)
                    heartbeats.add(lease)
            except Exception as exc:
                with lock:
                    if lease.granted:
                        leases.append(lease)
                        heartbeats.add(lease)
                    failures.append({
                        "agent": spec["agent"],
                        "error": repr(exc),
                        "request": lease.request_raw,
                        "grant": lease.grant_raw,
                    })
                if not lease.granted:
                    lease.close()

        threads = [
            threading.Thread(target=worker, args=(spec,),
                             name="grant-%s" % spec["agent"])
            for spec in requests
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        # Verify class and share via STATUS for all acquired leases. Each lease
        # owns its transport, so these network round trips are independent.
        if leases and not failures:
            def verify_one(lease):
                try:
                    lease.verify_status_class_share()
                    return None
                except Exception as exc:
                    return repr(exc)

            with concurrent.futures.ThreadPoolExecutor(
                    max_workers=len(leases),
                    thread_name_prefix="lease-status") as pool:
                verification = list(pool.map(verify_one, leases))
            for lease, error in zip(leases, verification):
                if error is not None:
                    failures.append({
                        "lease_id": lease.lease_id,
                        "error": "STATUS class/share verification failed: %s" % error,
                    })

        valid = not failures and len(leases) == len(requests)
        resources = [(lease.box, lease.card, lease.slot) for lease in leases
                     if lease.fields.get("box") and lease.fields.get("cards")
                     and lease.fields.get("slot")]
        if len(resources) != len(set(resources)):
            failures.append({"error": "duplicate granted box/card/slot"})
            valid = False

        by_card = {}
        for lease in leases:
            if not lease.fields.get("cards"):
                continue
            by_card.setdefault((lease.box, lease.card), []).append(lease)
        for key, members in by_card.items():
            shares = {member.requested_share for member in members}
            slots = {member.slot for member in members}
            if shares == {0} and (len(members) != 1 or slots != {0}):
                failures.append({"error": "exclusive topology violation", "card": key})
            if shares == {1} and (len(members) > 2 or len(slots) != len(members)):
                failures.append({"error": "shared topology violation", "card": key})
            if len(shares) != 1:
                failures.append({"error": "mixed share topology", "card": key})
        valid = valid and not failures

        if not valid:
            # Stop+join heartbeats BEFORE the release loop (same teardown-race
            # avoidance as WaveLeaseSet.release_after_clean): no heartbeat I/O may
            # run against a lease transport that is being closed below.
            heartbeats.stop()
            def rollback_one(lease):
                try:
                    probe = (release_probe(lease) if release_probe is not None
                             else {"clean": True, "offline_test": True})
                except Exception as exc:
                    probe = {"clean": False, "error": repr(exc)}
                result = {"lease_id": lease.lease_id, "probe": probe}
                if not probe.get("clean"):
                    result["dirty"] = True
                    return result
                try:
                    result["release"] = {
                        "lease_id": lease.lease_id,
                        "reply": lease.release(),
                    }
                except Exception as exc:
                    result["release"] = {
                        "lease_id": lease.lease_id,
                        "error": repr(exc),
                    }
                return result

            if leases:
                with concurrent.futures.ThreadPoolExecutor(
                        max_workers=len(leases),
                        thread_name_prefix="lease-rollback") as pool:
                    rollback = list(pool.map(rollback_one, leases))
            else:
                rollback = []
            probes = [{"lease_id": row["lease_id"], "probe": row["probe"]}
                      for row in rollback]
            releases = [row["release"] for row in rollback if "release" in row]
            dirty = [row["lease_id"] for row in rollback if row.get("dirty")]
            manifest = {
                "status": "INVALID",
                "phase": "wave_atomic_acquisition",
                "expected_lease_width": len(requests),
                "granted_lease_width": len(leases),
                "failures": failures,
                "leases": cls(leases, heartbeats, wave_dir).records(),
                "fuser_before_release": probes,
                "release_evidence": releases,
                "dirty_unreleased_leases": dirty,
            }
            atomic_write_json(wave_dir / "wave_manifest.json", manifest)
            archive_reason(
                wave_dir, "wave_atomic_acquisition",
                "wave acquisition was incomplete or unverifiable",
                replaceable=True, evidence=manifest,
            )
            raise PrelaunchInvalid("wave-atomic acquisition failed")

        return cls(leases, heartbeats, wave_dir)

    def release_after_clean(self, fuser_evidence):
        dirty = [row for row in fuser_evidence if not row.get("clean")]
        if dirty:
            raise ProtocolError("refusing REL because exact-card fuser is not clean")
        # Stop AND JOIN the heartbeat thread BEFORE closing any lease transport.
        # The heartbeat loop snapshots the lease list and does socket I/O on each
        # lease; if a lease is released (transport closed) while an iteration is
        # in flight, that iteration raises "I/O operation on closed file" and
        # records a late heartbeat_bad -- a pure teardown race with no bearing on
        # in-horizon isolation. Stopping+joining first makes that race impossible.
        self.heartbeats.stop()
        def release_one(lease):
            try:
                return {"lease_id": lease.lease_id, "reply": lease.release()}
            except Exception as exc:
                return {"lease_id": lease.lease_id, "error": repr(exc)}

        if self.leases:
            with concurrent.futures.ThreadPoolExecutor(
                    max_workers=len(self.leases),
                    thread_name_prefix="lease-release") as pool:
                replies = list(pool.map(release_one, self.leases))
        else:
            replies = []
        self.release_evidence = replies
        failures = [row for row in replies if row.get("reply") != "OK"]
        self.released = not failures
        if failures:
            raise ProtocolError("one or more REL replies were not OK: %s" %
                                json.dumps(failures, sort_keys=True))
        return replies


class FleetSSH:
    def __init__(self, key=None, user="kameron", hosts=None):
        self.key = str(key or (Path.home() / ".ssh" / "kameron_fleet"))
        self.user = str(user)
        self.hosts = hosts or {
            11: "192.168.2.11",
            21: "192.168.2.21",
            31: "192.168.2.31",
        }

    def _base(self):
        return [
            "-i", self.key,
            "-o", "BatchMode=yes",
            "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ConnectTimeout=15",
            "-o", "ServerAliveInterval=30",
            "-o", "ServerAliveCountMax=20",
        ]

    def run(self, box, command, timeout=None, check=False):
        proc = subprocess.run(
            ["ssh"] + self._base() +
            ["%s@%s" % (self.user, self.hosts[int(box)]), command],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=timeout,
        )
        if check and proc.returncode:
            raise RuntimeError(
                "ssh box=%s rc=%s: %s" %
                (box, proc.returncode, (proc.stdout or "")[-2000:])
            )
        return proc

    def popen(self, box, command):
        return subprocess.Popen(
            ["ssh"] + self._base() +
            ["%s@%s" % (self.user, self.hosts[int(box)]), command],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )

    def copy_to(self, source, box, destination):
        return subprocess.run(
            ["scp", "-q"] + self._base() + [
                str(source),
                "%s@%s:%s" % (self.user, self.hosts[int(box)], destination),
            ],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )

    def copy_from(self, box, source, destination):
        return subprocess.run(
            ["scp", "-q"] + self._base() + [
                "%s@%s:%s" % (self.user, self.hosts[int(box)], source),
                str(destination),
            ],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )


def broker_status(address=("192.168.2.31", 7800),
                  transport_factory=None):
    transport = (SocketLineTransport(address) if transport_factory is None
                 else transport_factory())
    try:
        transport.send("STATUS")
        reply = transport.receive()
    finally:
        transport.close()
    if not reply.startswith("OK "):
        raise ProtocolError("STATUS failed: %s" % reply)
    try:
        return json.loads(reply[3:])
    except json.JSONDecodeError as exc:
        raise ProtocolError("STATUS returned invalid JSON") from exc


def foreign_card_leases(status, box, held_agents):
    """Card-holding audio leases on ``box`` not owned by our agent set.

    The real broker STATUS ``lease_table`` keys each lease by agent name
    (``row["name"]``); it carries no lease-id field (see
    ``Lease.verify_status_class_share``).  Card occupancy is shown as an
    integer ``cards`` count and/or a non-empty ``slots`` list.  Rows whose
    ``box`` is a non-numeric sentinel (e.g. the bench lease) are ignored.

    Ownership must therefore be tested by agent name, not lease id.  The
    earlier detector matched ``row["lease"]``/``row["id"]`` against a set of
    lease ids -- fields that do not exist in a real row -- so every one of our
    own leased rows was flagged foreign and the pre-epoch width proof failed
    deterministically.
    """
    held = {str(agent) for agent in held_agents}
    foreign = []
    for row in status.get("lease_table", []) or []:
        try:
            row_box = int(row.get("box"))
        except (TypeError, ValueError):
            continue
        if row_box != int(box):
            continue
        cards = row.get("cards")
        card_holding = False
        if isinstance(cards, bool):
            card_holding = False
        elif isinstance(cards, int):
            card_holding = cards > 0
        elif isinstance(cards, (list, tuple)):
            card_holding = len(cards) > 0
        if not card_holding and (row.get("slots") or []):
            card_holding = True
        if not card_holding and int(row.get("ncards", 0) or 0) > 0:
            card_holding = True
        if not card_holding:
            continue
        if str(row.get("name")) not in held:
            foreign.append(row)
    return foreign


def health_snapshot(ssh, box):
    proc = ssh.run(
        box, "sudo -n cat /dev/shm/fleet_health/latest.json",
        timeout=30,
    )
    row = {
        "box": int(box),
        "read_epoch": time.time(),
        "command_rc": proc.returncode,
        "raw": proc.stdout or "",
    }
    if proc.returncode == 0:
        try:
            row["document"] = json.loads(proc.stdout)
        except json.JSONDecodeError as exc:
            row["error"] = "invalid health JSON: %s" % exc
    else:
        row["error"] = (proc.stdout or "")[-2000:]
    return row


def exact_card_fuser(ssh, box, card):
    pattern = "/dev/snd/pcmC%dD*" % int(card)
    absent_marker = "__CANON_EXACT_CARD_ABSENT__"
    error_marker = "__CANON_EXACT_CARD_FUSER_ERROR__"
    # Expansion deliberately happens in the remote shell.  ``set --`` turns
    # the result into an explicit argv vector for fuser; quoting the pattern
    # here would instead probe one nonexistent filename containing ``*``.
    # A non-matching glob is accepted as clean only after ``-e`` proves that
    # the literal pattern names no device.  fuser rc=1 with no output means
    # the enumerated devices exist but have no users; every other error is
    # fail-closed evidence.
    command = (
        "command -v fuser >/dev/null 2>&1 || { "
        "echo %s:missing-command; exit 127; }; "
        "if [ ! -e /dev/snd ]; then echo %s; exit 0; fi; "
        "if [ ! -d /dev/snd ] || [ ! -r /dev/snd ] || [ ! -x /dev/snd ]; then "
        "echo %s:enumeration-unavailable; exit 126; fi; "
        "set -- %s; "
        "if [ \"$#\" -eq 1 ] && [ ! -e \"$1\" ]; then "
        "echo %s; exit 0; fi; "
        "fuser \"$@\" 2>&1"
    ) % (error_marker, absent_marker, error_marker, pattern, absent_marker)
    proc = ssh.run(
        box, command,
        timeout=30,
    )
    output = (proc.stdout or "").strip()
    enumeration_absent = proc.returncode == 0 and output == absent_marker
    enumerated_clean = proc.returncode == 1 and not output
    evidence_failure = not (enumeration_absent or enumerated_clean
                           or (proc.returncode == 0 and bool(output)))
    return {
        "box": int(box),
        "physical_card": int(card),
        "epoch": time.time(),
        "output": output,
        "clean": enumeration_absent or enumerated_clean,
        "enumeration_absent": enumeration_absent,
        "evidence_failure": evidence_failure,
        "command_rc": proc.returncode,
    }


def full_fuser_snapshot(ssh, box):
    proc = ssh.run(
        box, "fuser -v /dev/snd/pcm* 2>&1 || true",
        timeout=30,
    )
    return {
        "box": int(box),
        "epoch": time.time(),
        "output": proc.stdout or "",
        "command_rc": proc.returncode,
    }


def validate_binary_manifest(manifest):
    if manifest.get("source_revision") != SOURCE_REVISION:
        raise ValueError(
            "binary manifest source revision must be %s" % SOURCE_REVISION
        )
    binary = manifest.get("binary") or {}
    for field in ("path", "sha256", "text_sha256"):
        if not binary.get(field):
            raise ValueError("binary manifest lacks binary.%s" % field)
    if hash_file(binary["path"]) != binary["sha256"]:
        raise ValueError("local executable hash differs from binary manifest")
    assets = manifest.get("assets") or {}
    missing = [name for name in REQUIRED_ASSETS if name not in assets]
    if missing:
        raise ValueError("binary manifest lacks assets: %s" % ",".join(missing))
    for name, row in assets.items():
        if not Path(row["path"]).is_file():
            raise ValueError("asset does not exist: %s" % row["path"])
        if hash_file(row["path"]) != row["sha256"]:
            raise ValueError("asset hash mismatch: %s" % name)
    return manifest


def stage_assets(ssh, boxes, manifest, archive_path):
    validate_binary_manifest(manifest)
    binary = manifest["binary"]
    token = binary["sha256"][:16]
    remote_root = "/dev/shm/build/wb_assets_%s" % token
    records = {}

    for box in boxes:
        proc = ssh.run(
            box,
            "mkdir -p %s/realaudio" % shlex.quote(remote_root),
            timeout=60,
        )
        if proc.returncode:
            raise RuntimeError("asset mkdir failed on box %s" % box)

        files = [("mercury", binary)] + sorted(manifest["assets"].items())
        for relative, identity in files:
            destination = "%s/%s" % (remote_root, relative)
            proc = ssh.copy_to(identity["path"], box, destination)
            if proc.returncode:
                raise RuntimeError(
                    "asset copy failed box=%s file=%s: %s" %
                    (box, relative, (proc.stdout or "")[-1000:])
                )

        text_dump = "%s/.mercury.text" % remote_root
        command = (
            "set -eu; chmod 755 {root}/mercury {root}/*.py "
            "{root}/realaudio/*.py; "
            "sha256sum {root}/mercury; "
            "objcopy --dump-section .text={text} {root}/mercury; "
            "sha256sum {text}; rm -f {text}; "
            "cd {root}; python3 realaudio/arq_realaudio.py --help >/dev/null"
        ).format(
            root=shlex.quote(remote_root),
            text=shlex.quote(text_dump),
        )
        proc = ssh.run(box, command, timeout=180)
        words = (proc.stdout or "").split()
        executable_hash = words[0] if len(words) >= 1 else ""
        text_hash = words[2] if len(words) >= 3 else ""
        if proc.returncode or executable_hash != binary["sha256"]:
            raise RuntimeError("remote executable identity failed on box %s" % box)
        if text_hash != binary["text_sha256"]:
            raise RuntimeError("remote .text identity failed on box %s" % box)
        records[str(box)] = {
            "remote_root": remote_root,
            "binary_sha256": executable_hash,
            "text_sha256": text_hash,
            "verified": True,
        }

    row = {
        "source_revision": SOURCE_REVISION,
        "binary_hashes": {
            "sha256": binary["sha256"],
            "text_sha256": binary["text_sha256"],
        },
        "boxes": records,
    }
    atomic_write_json(archive_path, row)
    return remote_root, row


def make_recipe(lease, cell, asset_root, remote_output):
    subs = ",".join(str(lease.slot * 4 + offset) for offset in range(4))
    port = 26000 + lease.card * 20 + lease.slot * 4
    argv = [
        "python3", "-u",
        "%s/realaudio/arq_realaudio.py" % asset_root,
        "--bin", "%s/mercury" % asset_root,
        "--bridge", "%s/realaudio/realaudio_bridge_s32_c" % asset_root,
        "--start-cfg", str(START_CONFIGURATION),
        "--secs", str(SCORE_HORIZON_S),
        "--score-horizon-s", str(SCORE_HORIZON_S),
        "--payload", str(PAYLOAD_BYTES),
        "--warm-start",
        "--traffic", "random-binary",
        "--profile", "wgn",
        "--snr3k", str(SNR3K),
        "--seed", str(cell["seed"]),
        "--tag", cell["cell_tag"],
        "--arm", cell["arm"],
        "--realized-width", str(cell["requested_width"]),
        "--card", card_name(lease.card),
        "--subs", subs,
        "--rsp-port", str(port),
        "--cmd-port", str(port + 4),
        "--no-kill",
        "--logdir", remote_output,
        "--json", "%s/result.json" % remote_output,
    ]
    for item in COMMON_ENV:
        argv += ["--env", item]
    forbidden = {"--snr", "--cn-config-db"}
    if any(item in forbidden for item in argv):
        raise AssertionError("recipe carries a forbidden controlling coordinate")
    if argv.count("--snr3k") != 1:
        raise AssertionError("recipe must carry exactly one --snr3k")
    return argv


class WidthProofSampler:
    """Append-only proof of reserved and active width through a score horizon."""

    def __init__(self, timeline_path, provider, requested_width,
                 interval=5.0, time_fn=time.time, sleep_fn=time.sleep):
        self.timeline_path = Path(timeline_path)
        self.provider = provider
        self.requested_width = int(requested_width)
        self.interval = float(interval)
        self.time_fn = time_fn
        self.sleep_fn = sleep_fn
        self.rows = []
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None

    def sample(self):
        row = dict(self.provider())
        row.setdefault("epoch", self.time_fn())
        row["requested_width"] = self.requested_width
        required = (
            "reserved_lease_width", "realized_active_width", "box", "peer_set"
        )
        missing = [field for field in required if field not in row]
        if missing:
            raise RuntimeError("width sample lacks %s" % ",".join(missing))
        append_jsonl(self.timeline_path, row)
        with self.lock:
            self.rows.append(row)
        return row

    def start(self):
        self.sample()
        self.thread = threading.Thread(
            target=self._run, name="width-proof", daemon=True
        )
        self.thread.start()

    def _run(self):
        while not self.stop_event.wait(self.interval):
            try:
                self.sample()
            except Exception as exc:
                append_jsonl(self.timeline_path, {
                    "epoch": self.time_fn(),
                    "requested_width": self.requested_width,
                    "sample_error": repr(exc),
                })

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=self.interval + 2)
        self.sample()

    def audit(self, start_epoch, end_epoch):
        with self.lock:
            rows = list(self.rows)
        horizon = [
            row for row in rows
            if start_epoch <= row.get("epoch", -1) <= end_epoch
        ]
        violations = [
            row for row in horizon
            if row.get("reserved_lease_width") != self.requested_width
            or row.get("realized_active_width") != self.requested_width
            or row.get("sample_error")
        ]
        return {
            "score_horizon_start": start_epoch,
            "score_horizon_end": end_epoch,
            "width_proof_sample_denominator": len(horizon),
            "width_violations": violations,
            "pass": bool(horizon) and not violations,
        }


def drain_cell_runners(provider, grace_s=CELL_DRAIN_GRACE_S,
                       poll_s=CELL_DRAIN_POLL_S, on_row=None,
                       time_fn=time.time, sleep_fn=time.sleep):
    """Bounded wait for every lease-owned cell runner to exit naturally.

    A scored arq_realaudio cell writes result.json ONLY at natural completion
    of main(); its run window (connect + --secs) ends AFTER the score horizon,
    so the runner must drain each cell to exit -- writing result.json into the
    remote output that the teardown archive callback later collects -- BEFORE
    the teardown ladder SIGTERMs it.  Without this drain the horizon-boundary
    SIGTERM killed arq_realaudio mid-transfer and every scored cell lost its
    result.json.

    Returns a summary.  ``survivors`` are cell tags still alive at the grace
    deadline (overran or wedged); the teardown ladder force-stops them and the
    cell is recorded as a missing-result outcome, never dropped from the
    denominator.  Detection reuses the same PID-liveness ``provider`` the width
    proof uses, so a cell that has exited drops out of ``peer_set``.
    """
    deadline = time_fn() + float(grace_s)
    samples = 0
    survivors = []
    heartbeat_lost = False
    while True:
        row = provider()
        samples += 1
        if on_row is not None:
            on_row(row)
        if row.get("heartbeat_lost"):
            heartbeat_lost = True
        survivors = sorted(set(row.get("peer_set") or []))
        if not survivors or heartbeat_lost:
            break
        if time_fn() >= deadline:
            break
        sleep_fn(poll_s)
    return {
        "grace_s": float(grace_s),
        "poll_s": float(poll_s),
        "samples": samples,
        "survivors": survivors,
        "drained_clean": not survivors,
        "heartbeat_lost": heartbeat_lost,
    }


def safe_extract_tar(archive, destination):
    destination = Path(destination).resolve()
    with tarfile.open(archive, "r:gz") as packed:
        members = packed.getmembers()
        for member in members:
            target = (destination / member.name).resolve()
            if target != destination and destination not in target.parents:
                raise RuntimeError("unsafe tar member %r" % member.name)
        packed.extractall(destination)


def archive_remote_cell(ssh, box, remote_root, cell_dir):
    cell_dir = Path(cell_dir)
    cell_dir.mkdir(parents=True, exist_ok=True)
    remote_archive = "%s/cell.tgz" % remote_root
    proc = ssh.run(
        box,
        "tar -C %s/out -czf %s ." %
        (shlex.quote(remote_root), shlex.quote(remote_archive)),
        timeout=300,
    )
    if proc.returncode:
        raise RuntimeError("remote cell archive failed: %s" %
                           (proc.stdout or "")[-2000:])
    local_archive = cell_dir / ".cell.tgz"
    proc = ssh.copy_from(box, remote_archive, local_archive)
    if proc.returncode:
        raise RuntimeError("cell archive copy failed: %s" %
                           (proc.stdout or "")[-2000:])
    safe_extract_tar(local_archive, cell_dir)
    local_archive.unlink()

    preserved = []
    for raw in sorted(cell_dir.glob("arq_*.log")):
        raw_hash = hash_file(raw)
        packed = raw.with_suffix(raw.suffix + ".gz")
        with open(raw, "rb") as source, open(packed, "wb") as raw_target:
            with gzip.GzipFile(
                filename="", mode="wb", fileobj=raw_target,
                compresslevel=6, mtime=0,
            ) as target:
                for block in iter(lambda: source.read(1024 * 1024), b""):
                    target.write(block)
        raw.unlink()
        preserved.append({
            "path": packed.name,
            "compression": "gzip",
            "uncompressed_sha256": raw_hash,
        })

    result = cell_dir / "result.json"
    return {
        "result_present": result.is_file(),
        "preserved_arq_logs": preserved,
    }


class RemoteWaveControl:
    """PID-scoped remote controls used by the teardown ladder."""

    def __init__(self, ssh, box, handles):
        self.ssh = ssh
        self.box = int(box)
        self.handles = list(handles)

    @staticmethod
    def _pid_list(handles, key):
        return sorted({
            int(item[key]) for item in handles
            if item.get(key) is not None and int(item[key]) > 1
        })

    def stop_dispatchers(self):
        pids = self._pid_list(self.handles, "dispatcher_pid")
        return self._term_owned(pids, "dispatcher")

    def sigterm_runners(self):
        pids = self._pid_list(self.handles, "runner_pid")
        return self._term_owned(pids, "runner")

    def remaining_mercury(self):
        # The runner (arq_realaudio.py) has no SIGTERM cleanup, so a runner
        # SIGTERM orphans its bridge child (which DOES install a clean SIGTERM
        # handler).  The orphan keeps the runner's pgid even after reparenting
        # to init, so this pgid-scoped sweep catches both mercury AND the
        # realaudio bridge; leaving the bridge behind leaked snd-aloop cards.
        pgids = sorted({
            int(item["runner_pid"]) for item in self.handles
            if item.get("runner_pid")
        })
        if not pgids:
            return []
        command = (
            "ps -eo pid=,pgid=,args= | "
            "awk -v g=%s 'BEGIN{n=split(g,a,\",\");"
            "for(i=1;i<=n;i++) wanted[a[i]]=1}"
            "wanted[$2] && /(mercury|realaudio_bridge)/ {print $1}'"
        ) % shlex.quote(",".join(map(str, pgids)))
        proc = self.ssh.run(self.box, command, timeout=30)
        result = []
        for word in (proc.stdout or "").split():
            if word.isdigit() and int(word) > 1:
                result.append(int(word))
        return sorted(set(result))

    def sigterm_mercury(self, pids):
        return self._term_owned(pids, "mercury")

    def fuser(self, cards):
        return [
            exact_card_fuser(self.ssh, self.box, card)
            for card in sorted(set(map(int, cards)))
        ]

    def _term_owned(self, pids, kind):
        if not pids:
            return {"kind": kind, "pids": [], "rc": 0}
        command = (
            "for p in %s; do "
            "[ -r /proc/$p/cmdline ] || continue; "
            "case \"$(tr '\\0' ' ' </proc/$p/cmdline)\" in "
            "*arq_realaudio.py*|*mercury*|*realaudio_bridge*|*wb_dispatch*) "
            "kill -TERM \"$p\";; "
            "*) echo \"REFUSED:$p\";; esac; done"
        ) % " ".join(map(str, pids))
        proc = self.ssh.run(self.box, command, timeout=30)
        return {
            "kind": kind,
            "pids": list(pids),
            "rc": proc.returncode,
            "output": proc.stdout or "",
        }


def teardown_wave(control, lease_set, wave_dir, base_claim,
                  archive_callback=None, grace_s=10.0,
                  sleep_fn=time.sleep, time_fn=time.time):
    """Apply the non-escalating real-audio teardown law and then release."""

    if float(grace_s) < 10.0:
        raise ValueError("teardown grace must be at least ten seconds")
    wave_dir = Path(wave_dir)
    events = []

    def event(action, evidence=None):
        row = {
            "epoch": time_fn(),
            "action": action,
            "evidence": evidence,
        }
        events.append(row)
        return row

    event("stop_dispatchers_first", control.stop_dispatchers())
    event("sigterm_lease_owned_cell_runners", control.sigterm_runners())
    first_wait_start = time_fn()
    sleep_fn(grace_s)
    event("runner_grace_complete", {
        "required_s": 10,
        "actual_s": time_fn() - first_wait_start,
    })

    remaining = control.remaining_mercury()
    event("remaining_lease_owned_mercury", {"pids": remaining})
    event("sigterm_remaining_lease_owned_mercury",
          control.sigterm_mercury(remaining))
    second_wait_start = time_fn()
    sleep_fn(grace_s)
    event("mercury_grace_complete", {
        "required_s": 10,
        "actual_s": time_fn() - second_wait_start,
    })

    cards = [lease.card for lease in lease_set.leases]
    fuser = control.fuser(cards)
    event("exact_card_fuser", fuser)
    clean = all(row.get("clean") for row in fuser)

    teardown = dict(base_claim)
    teardown.update({
        "teardown_schema": 1,
        "events": events,
        "exact_card_fuser": fuser,
        "clean": clean,
        "force_stop_used": False,
        "release_evidence": [],
    })
    atomic_write_json(wave_dir / "teardown.json", teardown)

    if not clean:
        teardown["status"] = "DIRTY_CARD_EXPLICIT_RECOVERY_REQUIRED"
        teardown["unreleased_lease_ids"] = [
            lease.lease_id for lease in lease_set.leases
        ]
        atomic_write_json(wave_dir / "teardown.json", teardown)
        archive_reason(
            wave_dir, "teardown",
            "exact-card fuser remained occupied; leases were not released",
            classification="FAIL", replaceable=False, evidence=fuser,
        )
        raise ScoredAdverse("dirty card after teardown grace")

    if archive_callback is not None:
        archive_callback()

    try:
        releases = lease_set.release_after_clean(fuser)
    except Exception as exc:
        teardown["status"] = "RELEASE_FAILED"
        teardown["release_error"] = repr(exc)
        atomic_write_json(wave_dir / "teardown.json", teardown)
        archive_reason(
            wave_dir, "release",
            "REL did not return OK for every clean lease",
            replaceable=False, evidence=teardown,
        )
        raise NonreplaceableInvalid("broker release failure") from exc

    teardown["status"] = "CLEAN_RELEASED"
    teardown["release_evidence"] = releases
    atomic_write_json(wave_dir / "teardown.json", teardown)
    return teardown


def execute_with_prelaunch_retries(coordinates, rejection_root, operation,
                                   max_attempts=3):
    """Repeat only an archived pre-launch infrastructure-invalid attempt."""

    rejection_root = Path(rejection_root)
    attempts = []
    for attempt in range(1, int(max_attempts) + 1):
        try:
            result = operation(dict(coordinates), attempt)
            attempts.append({
                "attempt": attempt,
                "coordinates": dict(coordinates),
                "outcome": "accepted",
            })
            return result, attempts
        except PrelaunchInvalid as exc:
            row = {
                "attempt": attempt,
                "coordinates": dict(coordinates),
                "outcome": "rejected_prelaunch_infrastructure",
                "reason": exc.reason,
            }
            attempts.append(row)
            atomic_write_json(
                rejection_root / ("attempt_%02d.json" % attempt), row
            )
    archive_reason(
        rejection_root, "prelaunch_retry",
        "pre-launch attempts exhausted",
        replaceable=False, evidence=attempts,
    )
    raise NonreplaceableInvalid("pre-launch attempts exhausted")

#!/usr/bin/env python3
"""Launch-gated per-host real-audio contention certification driver.

The immutable experiment contract is defined in CONTRACT below.  Normal use is:

  contention_cert_driver.py plan
  contention_cert_driver.py preflight ...
  contention_cert_driver.py run ... --approval-token CONTENTION_CERT_APPROVED
  contention_cert_driver.py reduce --run-dir ...

The run command never retries a cell or resumes a partial campaign.  It uses the
native one-card-per-lease broker protocol and an explicit spawner plan so every
wave has exactly the preregistered realized width.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


VERSION = "contention-cert-v1"
APPROVAL_TOKEN = "CONTENTION_CERT_APPROVED"
PREFLIGHT_MAX_AGE_SECONDS = 24 * 60 * 60
DEFAULT_ROOT = Path("/dev/shm/contention_cert")
DEFAULT_BACK = Path("~/contention_cert_out").expanduser()
DEFAULT_BINARY = DEFAULT_ROOT / "mercury_contention_cert"
DEFAULT_SPAWNER = (
    DEFAULT_ROOT / "stage" / "harness" / "sim" / "realaudio" /
    "parallel_spawner.py"
)
DEFAULT_LANDMINE_ROOT = Path("/mnt/c/mercury_codex_stage/cohort_preflight")

# This object is hashed into preflight and result artifacts.  Changing any value
# creates a different contract and invalidates an earlier preflight record.
CONTRACT: dict[str, Any] = {
    "version": VERSION,
    "widths": [2, 4, 8, 12, 16],
    "scored_seeds": list(range(1, 17)),
    "scored_cells_per_width": 16,
    "dial_db": 28.0,
    "payload_bytes": 262144,
    "seconds": 800,
    "start_cfg": 100,
    "mode": "auto",
    "traffic": "random-binary",
    "profile": "wgn",
    "launch_stagger_seconds": 2.0,
    "primary_fields": ["bridge_underruns", "rx_overrun_total"],
    "bridge_underrun_epsilon": 0,
    "rx_overrun_epsilon": 0,
    "stress": {
        "fixed_width": 8,
        "levels_percent": [0, 25, 50, 75],
        "nonzero_levels_run": [25, 50, 75],
        "warmup_seconds": 15,
        "settle_seconds": 5,
    },
    "policy_margin_cells": 2,
    "environment": {
        "MERCURY_SIM_PSIG_MODE": "steady",
        "MERCURY_TURN_TRACE": "1",
        "PYTHONUNBUFFERED": "1",
    },
    "outcome_retries": 0,
}

# The mechanical cohort gate reads this literal without importing the driver.
PREFLIGHT_RESULT_FIELDS = (
    "tag", "seed", "spawner_width", "bridge_underruns.fwd",
    "bridge_underruns.rev", "bridge_underruns.total", "rx_overrun_total",
    "connected", "delivered_full", "byte_integrity_ok",
)


class CertificationError(RuntimeError):
    """A fail-closed campaign or reduction error."""


class StopRequested(CertificationError):
    """Raised after TERM/INT so owned resources are released normally."""


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def decode_json(value: str | bytes | bytearray) -> Any:
    """Decode JSON through one wrapper so static reducer discovery stays scoped."""
    return json.loads(value)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def contract_sha256() -> str:
    return sha256_bytes(canonical_json(CONTRACT).encode("utf-8"))


def atomic_json(path: Path, value: Any) -> None:
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    os.replace(temp, path)


def jsonl_append(path: Path, value: Any, lock: threading.Lock | None = None) -> None:
    line = canonical_json(value) + "\n"
    if lock is None:
        with path.open("a") as stream:
            stream.write(line)
        return
    with lock:
        with path.open("a") as stream:
            stream.write(line)


def is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def require_nonnegative_int(value: Any, field: str) -> int:
    if not is_int(value) or value < 0:
        raise CertificationError(f"{field} must be a nonnegative integer, got {value!r}")
    return int(value)


def bridge_total(value: Any) -> int:
    if not isinstance(value, dict):
        raise CertificationError("bridge_underruns must be an object")
    fwd = require_nonnegative_int(value.get("fwd"), "bridge_underruns.fwd")
    rev = require_nonnegative_int(value.get("rev"), "bridge_underruns.rev")
    total = require_nonnegative_int(value.get("total"), "bridge_underruns.total")
    if total != fwd + rev:
        raise CertificationError(
            f"bridge_underruns.total={total} does not equal fwd+rev={fwd + rev}"
        )
    if value.get("source") not in (None, "bridge_stats"):
        raise CertificationError(f"unexpected bridge_underruns.source={value.get('source')!r}")
    return total


def rx_total(value: Any) -> int:
    """Accept the §104 scalar, or a producer-preserving object with a total.

    The scalar is the registered schema.  The object form is accepted only when
    it carries an integer `total`; this permits a later harness to retain per-peer
    components without changing the certification statistic.
    """
    if is_int(value):
        return require_nonnegative_int(value, "rx_overrun_total")
    if isinstance(value, dict):
        return require_nonnegative_int(value.get("total"), "rx_overrun_total.total")
    raise CertificationError("rx_overrun_total must be a nonnegative integer or total object")


def bridge_metrics_from_stats(stats: Any) -> dict[str, Any]:
    """Normalize the committed bridge_<tag>_stats.json primary meter."""
    if not isinstance(stats, dict):
        raise CertificationError("bridge stats must be an object")
    fwd_stats = stats.get("fwd")
    rev_stats = stats.get("rev")
    if not isinstance(fwd_stats, dict) or not isinstance(rev_stats, dict):
        raise CertificationError("bridge stats must contain fwd and rev objects")
    fwd = require_nonnegative_int(fwd_stats.get("underruns"), "bridge_stats.fwd.underruns")
    rev = require_nonnegative_int(rev_stats.get("underruns"), "bridge_stats.rev.underruns")
    return {"fwd": fwd, "rev": rev, "total": fwd + rev, "source": "bridge_stats"}


def metric_hot(
    result: dict[str, Any], bridge_stats: Any | None = None,
) -> tuple[bool, int, int]:
    bridge_value = result.get("bridge_underruns")
    if bridge_value is None and bridge_stats is not None:
        bridge_value = bridge_metrics_from_stats(bridge_stats)
    bridge = bridge_total(bridge_value)
    rx = rx_total(result.get("rx_overrun_total"))
    hot = (
        bridge > CONTRACT["bridge_underrun_epsilon"]
        or rx > CONTRACT["rx_overrun_epsilon"]
    )
    return hot, bridge, rx


def card_name(index: int) -> str:
    # This intentionally matches parallel_spawner.py's decimal suffix.
    return "Loopback" if index == 0 else f"Loopback_{index}"


def explicit_spawn_plan(
    tag: str, wave: dict[str, Any], grants: list[dict[str, str]],
) -> list[dict[str, Any]]:
    plan = []
    for local_index, (cell, grant) in enumerate(zip(wave["cells"], grants)):
        card_index = int(grant["cards"])
        slot = int(grant["slot"])
        role = "s" if cell["scored"] else "f"
        cell_tag = f"{tag}_{role}{int(cell['seed']):05d}"
        plan.append({
            "idx": local_index,
            "card_idx": card_index,
            "card": card_name(card_index),
            "slot": slot,
            "subs": [slot * 4 + offset for offset in range(4)],
            "rsp_port": int(wave["port_base"]) + 10 * local_index,
            "cmd_port": int(wave["port_base"]) + 10 * local_index + 4,
            "seed": int(cell["seed"]),
            "tag": cell_tag,
            "scored": bool(cell["scored"]),
        })
    return plan


def full_spawner_flags(
    *, width: int, binary: Path, plan: list[dict[str, Any]], start_cfg: int,
    payload: int, seconds: int, profile: str, card_base: int, port_base: int,
    tag_prefix: str, logdir: Path, output: Path, launch_stagger: float, arm: str,
) -> list[str]:
    """Return the complete preregistered dialect accepted at monitor tip."""
    flags = [
        "--n", str(width), "--bin", str(binary),
        "--start-cfg", str(start_cfg), "--payload", str(payload),
        "--secs", str(seconds), "--score-horizon-s", str(seconds),
        "--warm-start", "--traffic", CONTRACT["traffic"],
        "--snr", str(CONTRACT["dial_db"]),
        "--snr3k", str(CONTRACT["dial_db"]), "--profile", profile,
        "--card-base", str(card_base), "--port-base", str(port_base),
        "--seed-offset", "0", "--spawn-plan", canonical_json(plan),
        "--launch-stagger", str(launch_stagger), "--arm", arm,
        "--tag-prefix", tag_prefix, "--logdir", str(logdir), "--out", str(output),
    ]
    for key, value in sorted(CONTRACT["environment"].items()):
        flags += ["--env", f"{key}={value}"]
    return flags


def decode_broker_status(status: str) -> dict[str, Any]:
    """Parse the broker's `OK {json}` STATUS response without regex."""
    payload = status.strip()
    if payload.startswith("OK "):
        payload = payload[3:].lstrip()
    decoded = decode_json(payload)
    if not isinstance(decoded, dict):
        raise CertificationError("broker STATUS JSON must be an object")
    return decoded


def all_boxes_idle(status: str) -> tuple[bool, str]:
    decoded = decode_broker_status(status)
    boxes = decoded.get("boxes")
    if not isinstance(boxes, dict) or not boxes:
        raise CertificationError("broker STATUS JSON has no boxes object")
    active: dict[str, int] = {}
    for box, details in boxes.items():
        if not isinstance(details, dict):
            raise CertificationError(f"broker box {box} status must be an object")
        active[str(box)] = require_nonnegative_int(
            details.get("active_leases"), f"boxes.{box}.active_leases"
        )
    busy = {box: count for box, count in active.items() if count != 0}
    return not busy, canonical_json({"active_leases": active, "busy": busy})


def parse_cpu_list(text: str) -> list[int]:
    values: set[int] = set()
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            left, right = part.split("-", 1)
            lo, hi = int(left), int(right)
            if hi < lo:
                raise argparse.ArgumentTypeError(f"descending CPU range: {part}")
            values.update(range(lo, hi + 1))
        else:
            values.add(int(part))
    if not values or min(values) < 0:
        raise argparse.ArgumentTypeError("CPU list must contain nonnegative CPU IDs")
    return sorted(values)


def build_jobs() -> list[dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    for width in CONTRACT["widths"]:
        jobs.append({"arm": "width", "width": width, "stress_percent": 0})
    for level in CONTRACT["stress"]["nonzero_levels_run"]:
        jobs.append({
            "arm": "stress",
            "width": CONTRACT["stress"]["fixed_width"],
            "stress_percent": level,
        })
    return jobs


def build_waves(job: dict[str, Any]) -> list[dict[str, Any]]:
    """Return exact-width waves with 16 immutable scored seeds per job.

    A width that does not divide 16 receives deterministic load-only companion
    cells in its final wave.  Companion meters are conservative safety sentinels:
    they can trip the width but are never substituted into the 16-cell score set.
    """
    width = int(job["width"])
    seeds = list(CONTRACT["scored_seeds"])
    wave_count = math.ceil(len(seeds) / width)
    waves: list[dict[str, Any]] = []
    filler_serial = 0
    for wave_index in range(wave_count):
        scored = seeds[wave_index * width:(wave_index + 1) * width]
        cells = [{"seed": seed, "scored": True} for seed in scored]
        while len(cells) < width:
            filler_serial += 1
            cells.append({"seed": 10000 + filler_serial, "scored": False})
        waves.append({
            "wave": wave_index,
            "width": width,
            "cells": cells,
            "port_base": 7100 if wave_index % 2 == 0 else 7400,
        })
    return waves


def plan_document(box: int) -> dict[str, Any]:
    jobs = []
    scored_total = 0
    live_total = 0
    wave_total = 0
    for serial, job in enumerate(build_jobs(), 1):
        waves = build_waves(job)
        scored = sum(sum(1 for cell in wave["cells"] if cell["scored"]) for wave in waves)
        live = sum(len(wave["cells"]) for wave in waves)
        jobs.append({"serial": serial, **job, "waves": waves,
                     "scored_cells": scored, "live_cells": live})
        scored_total += scored
        live_total += live
        wave_total += len(waves)
    return {
        "campaign": VERSION,
        "contract_sha256": contract_sha256(),
        "target_box": box,
        "contract": CONTRACT,
        "jobs": jobs,
        "totals": {
            "jobs": len(jobs), "waves": wave_total,
            "scored_cells": scored_total, "live_cells": live_total,
        },
    }


class BrokerClient:
    def __init__(self, host: str, port: int, log):
        self.host = host
        self.port = port
        self.log = log

    def command(self, command: str, timeout: float = 10.0) -> str:
        sock = socket.create_connection((self.host, self.port), timeout=timeout)
        try:
            sock.sendall((command + "\n").encode("utf-8"))
            raw = sock.makefile("rb").readline(65537)
            if not raw:
                raise CertificationError("broker closed connection without a response")
            return raw.decode("utf-8", "replace").rstrip("\r\n")
        finally:
            sock.close()


@dataclass
class Lease:
    broker: BrokerClient
    agent: str
    box: int
    cpu: int
    ttl: int
    heartbeat_seconds: int
    grant: dict[str, str] | None = None
    lease_id: str | None = None

    def __post_init__(self) -> None:
        self._stop = threading.Event()
        self._socket: socket.socket | None = None
        self._heartbeat_errors: list[str] = []
        self._thread: threading.Thread | None = None

    def acquire(self) -> dict[str, str]:
        sock = socket.create_connection((self.broker.host, self.broker.port), timeout=10)
        self._socket = sock
        try:
            request = f"REQ {self.agent} box={self.box} cards=1 cpu={self.cpu} ttl={self.ttl}"
            sock.sendall((request + "\n").encode("utf-8"))
            sock.settimeout(None)
            stream = sock.makefile("rb")
            while True:
                raw = stream.readline(65537)
                if not raw:
                    raise CertificationError("broker closed queued lease request")
                reply = raw.decode("utf-8", "replace").rstrip("\r\n")
                if reply.startswith("QUEUE "):
                    self.broker.log(f"broker {reply}")
                    continue
                break
        finally:
            self._socket = None
            sock.close()
        if not reply.startswith("GRANT "):
            raise CertificationError(f"broker did not grant one-card lease: {reply}")
        fields = reply.split()
        self.lease_id = fields[1]
        grant = {"raw": reply, "lease": self.lease_id}
        grant.update(dict(item.split("=", 1) for item in fields[2:] if "=" in item))
        for key in ("box", "cards", "slot"):
            if key not in grant:
                raise CertificationError(f"GRANT missing {key}: {reply}")
        if int(grant["box"]) != self.box:
            raise CertificationError(f"GRANT landed on box {grant['box']}, expected {self.box}")
        if int(grant["slot"]) not in (0, 1):
            raise CertificationError(f"GRANT has invalid slot: {reply}")
        self.grant = grant
        self._thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._thread.start()
        return grant

    def _heartbeat_loop(self) -> None:
        while not self._stop.wait(self.heartbeat_seconds):
            try:
                response = self.broker.command(f"HB {self.lease_id}")
                if response != "OK":
                    raise CertificationError(f"heartbeat response {response!r}")
            except Exception as exc:  # recorded and made fatal at wave boundary
                self._heartbeat_errors.append(str(exc))
                self.broker.log(f"LEASE HB error lease={self.lease_id}: {exc}")

    def cancel_queued_request(self) -> None:
        if self._socket is not None:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._socket.close()
            except OSError:
                pass

    def release(self) -> str | None:
        self._stop.set()
        self.cancel_queued_request()
        if not self.lease_id:
            return None
        lease_id, self.lease_id = self.lease_id, None
        response = self.broker.command(f"REL {lease_id}", timeout=60)
        self.broker.log(f"LEASE REL {lease_id} -> {response}")
        return response


class Driver:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.root = Path(args.root).resolve()
        self.run_dir = self.root / "run"
        self.back = Path(args.back_dir).expanduser().resolve()
        self.binary = Path(args.binary).resolve()
        self.spawner = Path(args.spawner).resolve()
        self.agent = f"contention-cert-box{args.box}-{os.getpid()}"
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.pulse_stop = threading.Event()
        self.current_child: subprocess.Popen[bytes] | None = None
        self.current_stress: subprocess.Popen[bytes] | None = None
        self.owned_leases: list[Lease] = []
        self.failures: list[str] = []
        self.log_path = self.run_dir / "driver.log"
        self.events_path = self.run_dir / "EVENTS.jsonl"
        self.cells_path = self.run_dir / "CELL_RESULTS.jsonl"
        self.measurement_path = self.run_dir / "CERT_MEASUREMENTS.json"
        self.policy_path = self.run_dir / "BROKER_POLICY.json"

    def mono_ms(self) -> int:
        return time.monotonic_ns() // 1_000_000

    def log(self, message: str) -> None:
        line = (
            f"[{time.strftime('%Y-%m-%dT%H:%M:%S%z')} "
            f"mono_ms={self.mono_ms()}] {message}"
        )
        print(line, flush=True)
        with self.lock:
            with self.log_path.open("a") as stream:
                stream.write(line + "\n")

    def event(self, kind: str, **fields: Any) -> None:
        jsonl_append(self.events_path, {
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "mono_ms": self.mono_ms(), "kind": kind, **fields,
        }, self.lock)

    def pulse_loop(self) -> None:
        pulse = self.run_dir / "driver.pulse"
        while not self.pulse_stop.wait(15):
            try:
                pulse.write_text(f"pid={os.getpid()} mono_ms={self.mono_ms()} alive\n")
            except OSError:
                pass

    def validate_fresh_paths(self) -> None:
        if self.root in (Path("/"), Path.home()) or self.back in (Path("/"), Path.home()):
            raise CertificationError("refusing broad root or back directory")
        if self.run_dir.exists():
            raise CertificationError(f"fresh run required; path already exists: {self.run_dir}")
        campaign_back = self.back / f"box{self.args.box}_{contract_sha256()[:12]}"
        if campaign_back.exists():
            raise CertificationError(f"fresh run required; archive path exists: {campaign_back}")
        self.campaign_back = campaign_back

    def acquire_exact_width(self, width: int) -> tuple[list[Lease], list[dict[str, str]]]:
        broker = BrokerClient(self.args.broker_host, self.args.broker_port, self.log)
        leases = [Lease(broker, self.agent, self.args.box, self.args.cpu_per_cell,
                        self.args.lease_ttl, self.args.heartbeat_seconds)
                  for _ in range(width)]
        errors: list[str] = []

        def acquire_one(lease: Lease) -> None:
            try:
                lease.acquire()
            except Exception as exc:
                errors.append(str(exc))

        threads = [threading.Thread(target=acquire_one, args=(lease,), daemon=True)
                   for lease in leases]
        for thread in threads:
            thread.start()
        deadline = time.monotonic() + self.args.grant_window_seconds
        while time.monotonic() < deadline and any(thread.is_alive() for thread in threads):
            if self.stop.wait(0.1):
                break
        for lease, thread in zip(leases, threads):
            if thread.is_alive():
                lease.cancel_queued_request()
        for thread in threads:
            thread.join(timeout=3)
        granted = [lease for lease in leases if lease.grant is not None]
        if len(granted) != width:
            for lease in granted:
                try:
                    lease.release()
                except Exception as exc:
                    errors.append(f"release after short grant: {exc}")
            raise CertificationError(
                f"exact width {width} not granted in {self.args.grant_window_seconds}s; "
                f"got {len(granted)}; no partial-width run permitted" +
                (f"; errors={errors}" if errors else "")
            )
        grants = [dict(lease.grant or {}) for lease in granted]
        resources = [(int(grant["cards"]), int(grant["slot"])) for grant in grants]
        if len(set(resources)) != width:
            for lease in granted:
                lease.release()
            raise CertificationError(f"duplicate broker card/slot grants: {resources}")
        self.owned_leases = granted
        self.event("lease_width_granted", requested=width, achieved=len(granted), grants=grants)
        return granted, grants

    def release_leases(self, leases: Iterable[Lease]) -> None:
        release_errors = []
        for lease in leases:
            try:
                response = lease.release()
                if response not in (None, "OK"):
                    release_errors.append(f"lease release response {response!r}")
            except Exception as exc:
                release_errors.append(str(exc))
        self.owned_leases = []
        if release_errors:
            raise CertificationError("lease cleanup failed: " + "; ".join(release_errors))

    def start_stress(self, level: int, tag: str) -> subprocess.Popen[bytes] | None:
        if level == 0:
            self.event("stress_baseline", tag=tag, load_percent=0)
            return None
        cores = self.args.stress_cpus
        command = ["taskset", "--cpu-list", ",".join(map(str, cores)),
                   self.args.stress_ng, "--cpu", str(len(cores)),
                   "--cpu-load", str(level), "--cpu-method", "all", "--metrics-brief"]
        log_stream = (self.run_dir / f"{tag}.stress-ng.log").open("wb")
        proc = subprocess.Popen(command, stdout=log_stream, stderr=subprocess.STDOUT,
                                start_new_session=True)
        proc._cert_log_stream = log_stream  # type: ignore[attr-defined]
        self.current_stress = proc
        self.event("stress_started", tag=tag, load_percent=level,
                   cores=cores, pid=proc.pid, command=command)
        deadline = time.monotonic() + CONTRACT["stress"]["warmup_seconds"]
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise CertificationError(f"stress-ng exited during warmup rc={proc.returncode}")
            if self.stop.wait(min(0.25, max(0.0, deadline - time.monotonic()))):
                raise StopRequested("stop requested during stress warmup")
        return proc

    def stop_stress(self, proc: subprocess.Popen[bytes] | None, tag: str) -> None:
        if proc is None:
            return
        try:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
        finally:
            proc._cert_log_stream.close()  # type: ignore[attr-defined]
            self.event("stress_stopped", tag=tag, pid=proc.pid, rc=proc.returncode)
            self.current_stress = None

    def terminate_child(self) -> None:
        proc = self.current_child
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        finally:
            self.current_child = None

    def make_plan(
        self, tag: str, wave: dict[str, Any], grants: list[dict[str, str]],
    ) -> list[dict[str, Any]]:
        return explicit_spawn_plan(tag, wave, grants)

    def run_wave(self, job_serial: int, job: dict[str, Any], wave: dict[str, Any]) -> None:
        width = int(job["width"])
        level = int(job["stress_percent"])
        tag = f"j{job_serial:02d}_{job['arm']}_w{width}_l{level}_v{wave['wave']:02d}"
        leases: list[Lease] = []
        stress = None
        proc = None
        try:
            leases, grants = self.acquire_exact_width(width)
            plan = self.make_plan(tag, wave, grants)
            logdir = self.run_dir / f"logs_{tag}"
            logdir.mkdir()
            output = self.run_dir / f"{tag}.json"
            spawnlog = self.run_dir / f"{tag}.spawn.log"
            stress = self.start_stress(level, tag)
            command = [sys.executable, "-u", str(self.spawner)] + full_spawner_flags(
                width=width, binary=self.binary, plan=plan,
                start_cfg=CONTRACT["start_cfg"],
                payload=CONTRACT["payload_bytes"], seconds=CONTRACT["seconds"],
                profile=CONTRACT["profile"], card_base=0,
                port_base=int(wave["port_base"]), tag_prefix=f"{tag}_",
                logdir=logdir, output=output,
                launch_stagger=CONTRACT["launch_stagger_seconds"],
                arm=str(job["arm"]),
            )
            env = dict(os.environ)
            env.update(CONTRACT["environment"])
            self.event("wave_launch", tag=tag, job=job, wave=wave["wave"],
                       width=width, plan=plan, command=command)
            with spawnlog.open("wb") as stream:
                proc = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                        env=env, start_new_session=True)
                self.current_child = proc
                rc = proc.wait()
                self.current_child = None
            self.event("wave_exit", tag=tag, rc=rc)
            if rc != 0:
                raise CertificationError(f"{tag}: spawner exited {rc}")
            if stress is not None and stress.poll() is not None:
                raise CertificationError(f"{tag}: stress-ng exited early rc={stress.returncode}")
            self.collect_wave_results(tag, job, wave, plan, grants, logdir)
            marker = self.run_dir / f"{tag}.wave.done"
            marker.write_text(f"OK tag={tag} width={width} cells={len(plan)}\n")
        finally:
            if proc is not None and proc.poll() is None:
                self.terminate_child()
            stress_error = None
            try:
                self.stop_stress(stress, tag)
            except Exception as exc:
                stress_error = exc
            lease_error = None
            try:
                self.release_leases(leases)
            except Exception as exc:
                lease_error = exc
            if stress_error:
                raise CertificationError(f"stress cleanup failed: {stress_error}")
            if lease_error:
                raise CertificationError(str(lease_error))
            if any(lease._heartbeat_errors for lease in leases):
                errors = [error for lease in leases for error in lease._heartbeat_errors]
                raise CertificationError(f"heartbeat errors during {tag}: {errors}")

    def collect_wave_results(
        self, tag: str, job: dict[str, Any], wave: dict[str, Any],
        plan: list[dict[str, Any]], grants: list[dict[str, str]], logdir: Path,
    ) -> None:
        expected = {item["tag"]: (item, grant) for item, grant in zip(plan, grants)}
        actual_paths = {path.stem[4:]: path for path in logdir.glob("res_*.json")}
        missing = sorted(set(expected) - set(actual_paths))
        unexpected = sorted(set(actual_paths) - set(expected))
        if missing or unexpected:
            raise CertificationError(f"{tag}: result census mismatch missing={missing} unexpected={unexpected}")
        for cell_tag in sorted(expected):
            item, grant = expected[cell_tag]
            path = actual_paths[cell_tag]
            raw = path.read_bytes()
            try:
                result = decode_json(raw)
            except json.JSONDecodeError as exc:
                raise CertificationError(f"{path}: malformed JSON: {exc}") from exc
            if result.get("tag") != cell_tag:
                raise CertificationError(f"{path}: tag mismatch {result.get('tag')!r}")
            if result.get("seed") != item["seed"]:
                raise CertificationError(f"{path}: seed mismatch {result.get('seed')!r}")
            if result.get("payload_target") != CONTRACT["payload_bytes"]:
                raise CertificationError(f"{path}: payload_target mismatch")
            expected_env = {
                f"{key}={value}" for key, value in CONTRACT["environment"].items()
            }
            if not isinstance(result.get("env"), list) or not expected_env.issubset(
                set(result["env"])
            ):
                raise CertificationError(f"{path}: committed env attestation mismatch")
            if result.get("traffic") != CONTRACT["traffic"]:
                raise CertificationError(f"{path}: traffic attestation mismatch")
            if result.get("warm_start") is not True:
                raise CertificationError(f"{path}: warm-start attestation mismatch")
            if result.get("warm_start_ok") is not True:
                raise CertificationError(f"{path}: warm-start readiness was not reached")
            if result.get("score_horizon_s") != float(CONTRACT["seconds"]):
                raise CertificationError(f"{path}: score horizon attestation mismatch")
            if result.get("score_horizon_reached") is not True:
                raise CertificationError(f"{path}: score horizon was not reached")
            if result.get("snr") != float(CONTRACT["dial_db"]):
                raise CertificationError(f"{path}: SNR attestation mismatch")
            if result.get("snr3k") != float(CONTRACT["dial_db"]):
                raise CertificationError(f"{path}: SNR3k attestation mismatch")
            if result.get("profile") != CONTRACT["profile"]:
                raise CertificationError(f"{path}: profile mismatch")
            bridge_path = logdir / f"bridge_{cell_tag}_stats.json"
            try:
                bridge_stats = decode_json(bridge_path.read_text())
            except (OSError, json.JSONDecodeError) as exc:
                raise CertificationError(f"{bridge_path}: invalid bridge stats: {exc}") from exc
            hot, bridge, rx = metric_hot(result, bridge_stats)
            for secondary in ("connected", "delivered_full", "byte_integrity_ok"):
                if not isinstance(result.get(secondary), bool):
                    raise CertificationError(f"{path}: missing boolean secondary {secondary}")
            bridge_value = bridge_metrics_from_stats(bridge_stats)
            row = {
                "campaign": VERSION, "contract_sha256": contract_sha256(),
                "target_box": self.args.box, "job_arm": job["arm"],
                "width": int(job["width"]),
                "stress_percent": int(job["stress_percent"]),
                "wave": int(wave["wave"]), "tag": cell_tag,
                "seed": int(item["seed"]), "scored": bool(item["scored"]),
                "spawner_width": int(job["width"]),
                "bridge_underruns": bridge_value,
                "bridge_underrun_total": bridge,
                "rx_overrun_total": result["rx_overrun_total"],
                "rx_overrun_total_normalized": rx,
                "primary_hot": hot,
                "connected": result["connected"],
                "delivered_full": result["delivered_full"],
                "byte_integrity_ok": result["byte_integrity_ok"],
                "payload_target": result.get("payload_target"),
                "rx_bytes": result.get("rx_bytes"),
                "result_path": str(path.relative_to(self.run_dir)),
                "result_sha256": sha256_bytes(raw),
                "broker_grant": grant,
            }
            jsonl_append(self.cells_path, row, self.lock)
            self.event("cell_meter_read", tag=cell_tag, width=job["width"],
                       stress_percent=job["stress_percent"], scored=item["scored"],
                       bridge_underrun_total=bridge, rx_overrun_total=rx,
                       primary_hot=hot)

    def run(self) -> int:
        self.validate_fresh_paths()
        self.run_dir.mkdir(parents=True)
        self.campaign_back.mkdir(parents=True)
        plan = plan_document(self.args.box)
        manifest = {
            "campaign": VERSION, "contract_sha256": contract_sha256(),
            "created_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "created_epoch": time.time(), "host": socket.gethostname(),
            "pid": os.getpid(), "target_box": self.args.box,
            "binary": str(self.binary), "binary_sha256": sha256_file(self.binary),
            "spawner": str(self.spawner), "spawner_sha256": sha256_file(self.spawner),
            "preflight_record": str(Path(self.args.preflight_record).resolve()),
            "driver_sha256": sha256_file(Path(__file__).resolve()),
            "site": {
                "broker_host": self.args.broker_host,
                "broker_port": self.args.broker_port,
                "cpu_per_cell": self.args.cpu_per_cell,
                "stress_cpus": self.args.stress_cpus,
                "stress_ng": self.args.stress_ng,
            },
            "plan": plan,
        }
        atomic_json(self.run_dir / "MANIFEST.json", manifest)
        threading.Thread(target=self.pulse_loop, daemon=True).start()

        def request_stop(signum, _frame):
            self.stop.set()
            raise StopRequested(f"received signal {signum}")

        signal.signal(signal.SIGTERM, request_stop)
        signal.signal(signal.SIGINT, request_stop)
        status = "FAIL"
        try:
            for job_serial, job in enumerate(build_jobs(), 1):
                self.log(f"JOB {job_serial}/{len(build_jobs())} {job}")
                for wave in build_waves(job):
                    if self.stop.is_set():
                        raise StopRequested("stop requested")
                    self.run_wave(job_serial, job, wave)
                    if CONTRACT["stress"]["settle_seconds"]:
                        time.sleep(CONTRACT["stress"]["settle_seconds"])
            measurement, policy = reduce_cells(self.cells_path, self.args.box)
            atomic_json(self.measurement_path, measurement)
            atomic_json(self.policy_path, policy)
            status = "OK"
            return_code = 0
        except Exception as exc:
            self.failures.append(str(exc))
            self.log(f"FATAL: {exc}\n{traceback.format_exc()}")
            return_code = 1
        finally:
            self.pulse_stop.set()
            self.terminate_child()
            if self.current_stress is not None:
                try:
                    self.stop_stress(self.current_stress, "emergency")
                except Exception as exc:
                    self.failures.append(f"emergency stress cleanup: {exc}")
                    return_code = 1
            if self.owned_leases:
                try:
                    self.release_leases(self.owned_leases)
                except Exception as exc:
                    self.failures.append(f"emergency lease cleanup: {exc}")
                    return_code = 1
            status = "OK" if return_code == 0 else "FAIL"
            done = {
                "status": status, "failures": self.failures,
                "contract_sha256": contract_sha256(),
                "completed_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            }
            atomic_json(self.run_dir / "DONE.json", done)
            try:
                archive = self.campaign_back / f"contention_cert_box{self.args.box}.tgz"
                with tarfile.open(archive, "w:gz") as tar:
                    tar.add(self.run_dir, arcname="run")
                (archive.with_suffix(archive.suffix + ".sha256")).write_text(
                    f"{sha256_file(archive)}  {archive.name}\n"
                )
            except Exception as exc:
                self.failures.append(f"archive failure: {exc}")
                return_code = 1
                atomic_json(self.run_dir / "DONE.json", {
                    **done, "status": "FAIL", "failures": self.failures,
                })
        return return_code


def load_rows(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = decode_json(line)
            except json.JSONDecodeError as exc:
                raise CertificationError(f"{path}:{line_number}: malformed JSON") from exc
            if not isinstance(row, dict):
                raise CertificationError(f"{path}:{line_number}: row must be an object")
            rows.append(row)
    return rows


def summarize_group(rows: list[dict[str, Any]]) -> dict[str, Any]:
    scored = [row for row in rows if row.get("scored") is True]
    return {
        "live_cells": len(rows),
        "scored_cells": len(scored),
        "primary_hot_live": sum(row.get("primary_hot") is True for row in rows),
        "primary_hot_scored": sum(row.get("primary_hot") is True for row in scored),
        "max_bridge_underrun_total": max((row["bridge_underrun_total"] for row in rows), default=None),
        "max_rx_overrun_total": max((row["rx_overrun_total_normalized"] for row in rows), default=None),
        "connected_scored": sum(row.get("connected") is True for row in scored),
        "delivered_full_scored": sum(row.get("delivered_full") is True for row in scored),
        "integrity_ok_scored": sum(row.get("byte_integrity_ok") is True for row in scored),
        "hot_tags": [row["tag"] for row in rows if row.get("primary_hot") is True],
    }


def reduce_cells(cells_path: Path, box: int) -> tuple[dict[str, Any], dict[str, Any]]:
    rows = load_rows(cells_path)
    tags = [row.get("tag") for row in rows]
    if any(not isinstance(tag, str) or not tag for tag in tags):
        raise CertificationError("every cell row must have a nonempty tag")
    if len(tags) != len(set(tags)):
        raise CertificationError("duplicate cell tag in reduction input")
    for row in rows:
        if row.get("contract_sha256") != contract_sha256():
            raise CertificationError("cell row contract hash mismatch")
        if row.get("target_box") != box:
            raise CertificationError("cell row target box mismatch")
        bridge = require_nonnegative_int(
            row.get("bridge_underrun_total"), "bridge_underrun_total")
        rx = require_nonnegative_int(
            row.get("rx_overrun_total_normalized"), "rx_overrun_total_normalized")
        if not isinstance(row.get("primary_hot"), bool):
            raise CertificationError("cell row primary_hot must be boolean")
        recomputed_hot = (
            bridge > CONTRACT["bridge_underrun_epsilon"]
            or rx > CONTRACT["rx_overrun_epsilon"]
        )
        if row["primary_hot"] != recomputed_hot:
            raise CertificationError(f"cell row primary_hot mismatch for {row['tag']}")

    def validate_group_shape(group: list[dict[str, Any]], job: dict[str, Any], label: str) -> None:
        expected = sorted(
            (int(wave["wave"]), int(cell["seed"]), bool(cell["scored"]))
            for wave in build_waves(job) for cell in wave["cells"]
        )
        actual = sorted(
            (int(row.get("wave", -1)), int(row.get("seed", -1)), bool(row.get("scored")))
            for row in group
        )
        if actual != expected:
            raise CertificationError(f"{label}: immutable wave/seed/role census mismatch")

    width_summaries: dict[str, Any] = {}
    for width in CONTRACT["widths"]:
        group = [row for row in rows if row.get("job_arm") == "width"
                 and row.get("width") == width and row.get("stress_percent") == 0]
        validate_group_shape(group, {"arm": "width", "width": width,
                                     "stress_percent": 0}, f"width {width}")
        summary = summarize_group(group)
        if summary["scored_cells"] != CONTRACT["scored_cells_per_width"]:
            raise CertificationError(
                f"width {width}: expected {CONTRACT['scored_cells_per_width']} scored cells, "
                f"got {summary['scored_cells']}"
            )
        expected_live = sum(len(wave["cells"]) for wave in build_waves(
            {"arm": "width", "width": width, "stress_percent": 0}))
        if summary["live_cells"] != expected_live:
            raise CertificationError(f"width {width}: expected {expected_live} live cells")
        summary["passes_primary"] = summary["primary_hot_live"] == 0
        width_summaries[str(width)] = summary

    stress_summaries: dict[str, Any] = {}
    baseline = [row for row in rows if row.get("job_arm") == "width"
                and row.get("width") == CONTRACT["stress"]["fixed_width"]]
    stress_summaries["0"] = summarize_group(baseline)
    for level in CONTRACT["stress"]["nonzero_levels_run"]:
        group = [row for row in rows if row.get("job_arm") == "stress"
                 and row.get("width") == CONTRACT["stress"]["fixed_width"]
                 and row.get("stress_percent") == level]
        validate_group_shape(group, {"arm": "stress",
                                     "width": CONTRACT["stress"]["fixed_width"],
                                     "stress_percent": level}, f"stress {level}")
        summary = summarize_group(group)
        if summary["scored_cells"] != CONTRACT["scored_cells_per_width"]:
            raise CertificationError(f"stress {level}: scored cell census mismatch")
        stress_summaries[str(level)] = summary

    failing_widths = [width for width in CONTRACT["widths"]
                      if width_summaries[str(width)]["primary_hot_live"] > 0]
    knee = min(failing_widths) if failing_widths else None
    secondary = {
        "connected": sum(row.get("connected") is True for row in rows if row.get("scored")),
        "delivered_full": sum(row.get("delivered_full") is True for row in rows if row.get("scored")),
        "integrity_ok": sum(row.get("byte_integrity_ok") is True for row in rows if row.get("scored")),
        "scored_total": sum(row.get("scored") is True for row in rows),
    }
    loaded_hot = [int(level) for level, summary in stress_summaries.items()
                  if int(level) > 0 and summary["primary_hot_live"] > 0]
    if knee is not None and loaded_hot:
        discriminator = "contention_supported"
    elif knee is not None and not loaded_hot:
        discriminator = "width_per_se_supported"
    elif knee is None and loaded_hot:
        discriminator = "contention_sensitive_without_width_knee"
    else:
        discriminator = "no_primary_event_observed"

    measurement = {
        "campaign": VERSION, "contract_sha256": contract_sha256(),
        "target_box": box, "primary_rule": {
            "fields": CONTRACT["primary_fields"],
            "bridge_underrun_epsilon": CONTRACT["bridge_underrun_epsilon"],
            "rx_overrun_epsilon": CONTRACT["rx_overrun_epsilon"],
            "cell_hot": "bridge_underruns.total > epsilon OR rx_overrun_total > epsilon",
            "width_hot": "any live cell is hot, including declared load-only companions",
        },
        "widths": width_summaries,
        "stress_fixed_width": CONTRACT["stress"]["fixed_width"],
        "stress": stress_summaries,
        "knee_cells": knee,
        "knee_censored_above": max(CONTRACT["widths"]) if knee is None else None,
        "discriminator": discriminator,
        "secondaries": secondary,
        "valid_for_policy": (
            secondary["connected"] == secondary["scored_total"]
            and secondary["delivered_full"] == secondary["scored_total"]
            and secondary["integrity_ok"] == secondary["scored_total"]
        ),
    }
    margin = CONTRACT["policy_margin_cells"]
    if knee is None:
        derived_cap = None
        disposition = (
            "Knee not observed through width 16; the preregistered formula cannot produce "
            "a measured cap. Extend the registered width range or retain the existing cap."
        )
    else:
        derived_cap = knee - margin
        disposition = "derived" if derived_cap >= 1 else "no_positive_cap_certified"
    if not measurement["valid_for_policy"]:
        disposition = "invalid_secondary_gate"
        derived_cap = None
    policy = {
        "campaign": VERSION, "contract_sha256": contract_sha256(),
        "target_box": box, "measurement_file": "CERT_MEASUREMENTS.json",
        "measurement_sha256": sha256_bytes(
            (json.dumps(measurement, indent=2, sort_keys=True) + "\n").encode("utf-8")
        ),
        "knee_cells": knee, "margin_cells": margin,
        "formula": "broker_width_cap_cells = knee_cells - margin_cells",
        "derived_broker_width_cap_cells": derived_cap,
        "disposition": disposition,
        "application_rule": (
            "Apply only to the measured target_box after archive verification; never copy "
            "the cap to another host. Restore the temporary certification override first."
        ),
    }
    return measurement, policy


def simulated_grants(width: int) -> list[dict[str, str]]:
    return [
        {"cards": str(index // 2), "slot": str(index % 2)}
        for index in range(width)
    ]


def cohort_plan(width: int = 1, arm: str = "smoke") -> list[dict[str, Any]]:
    wave = {
        "port_base": 7100,
        "cells": [{"seed": seed, "scored": True} for seed in range(1, width + 1)],
    }
    return explicit_spawn_plan(arm, wave, simulated_grants(width))


def cohort_flags(
    binary: Path, root: Path, width: int = 1, arm: str = "smoke",
    plan: list[dict[str, Any]] | None = None,
) -> list[str]:
    actual_plan = plan if plan is not None else cohort_plan(width, arm)
    return full_spawner_flags(
        width=width, binary=binary, plan=actual_plan,
        start_cfg=CONTRACT["start_cfg"], payload=CONTRACT["payload_bytes"],
        seconds=CONTRACT["seconds"], profile=CONTRACT["profile"],
        card_base=0, port_base=7100, tag_prefix=f"{arm}_",
        logdir=root / "logs", output=root / "spawner_result.json",
        launch_stagger=CONTRACT["launch_stagger_seconds"], arm=arm,
    )


def run_spawner_dry_run(
    spawner: Path, flags: list[str], expected_plan: list[dict[str, Any]],
) -> dict[str, Any]:
    command = [sys.executable, str(spawner)] + flags + ["--dry-run"]
    completed = subprocess.run(command, text=True, capture_output=True)
    if completed.returncode != 0:
        raise CertificationError(
            f"spawner --dry-run rejected exact command rc={completed.returncode}: "
            f"{completed.stderr.strip()}"
        )
    try:
        decoded = decode_json(completed.stdout)
    except json.JSONDecodeError as exc:
        raise CertificationError(f"spawner --dry-run emitted invalid JSON: {exc}") from exc
    plans = decoded.get("plan") if isinstance(decoded, dict) else None
    commands = decoded.get("commands") if isinstance(decoded, dict) else None
    if not isinstance(plans, list) or not isinstance(commands, list):
        raise CertificationError("spawner --dry-run omitted plan or commands")
    if len(plans) != len(expected_plan) or len(commands) != len(expected_plan):
        raise CertificationError("spawner --dry-run cell census mismatch")
    for expected, actual, child in zip(expected_plan, plans, commands):
        for field in ("card", "subs", "rsp_port", "cmd_port", "seed", "tag"):
            if actual.get(field) != expected[field]:
                raise CertificationError(
                    f"spawner --dry-run plan mismatch {field}: "
                    f"{actual.get(field)!r} != {expected[field]!r}"
                )
        for option in ("--warm-start", "--traffic", "--score-horizon-s",
                       "--snr", "--snr3k"):
            if option not in child:
                raise CertificationError(f"child command omitted {option}")
    return {"command": command, "cells": len(expected_plan)}


def dry_run_spawner(spawner: Path, binary: Path, width: int = 1) -> dict[str, Any]:
    """Exercise the exact full-dialect smoke shape through spawner --dry-run."""
    root = Path(tempfile.gettempdir()) / "contention-cert-dry-run"
    plan = cohort_plan(width, "dryrun")
    flags = cohort_flags(binary, root, width, "dryrun", plan)
    result = run_spawner_dry_run(spawner, flags, plan)
    return {
        "argparse_accepts": True,
        "capabilities_verified": [
            "warm-start", "random-binary", "score-horizon", "dual-dials",
            "seed-offset", "spawn-plan", "byte-integrity-result-path",
        ],
        "cells_run": 0,
        **result,
    }


def dry_run_exact_commands(spawner: Path, binary: Path) -> dict[str, Any]:
    """Parse every exact sweep wave and the exact smoke command without launch."""
    root = Path(tempfile.gettempdir()) / "contention-cert-exact-dry-run"
    sweep = []
    total_cells = 0
    for job_serial, job in enumerate(build_jobs()):
        for wave in build_waves(job):
            width = int(job["width"])
            tag = (
                f"j{job_serial:02d}_{job['arm']}_w{width}_"
                f"l{job['stress_percent']}_v{wave['wave']:02d}"
            )
            plan = explicit_spawn_plan(tag, wave, simulated_grants(width))
            flags = full_spawner_flags(
                width=width, binary=binary, plan=plan,
                start_cfg=CONTRACT["start_cfg"], payload=CONTRACT["payload_bytes"],
                seconds=CONTRACT["seconds"], profile=CONTRACT["profile"],
                card_base=0, port_base=int(wave["port_base"]),
                tag_prefix=f"{tag}_", logdir=root / f"logs_{tag}",
                output=root / f"{tag}.json",
                launch_stagger=CONTRACT["launch_stagger_seconds"],
                arm=str(job["arm"]),
            )
            parsed = run_spawner_dry_run(spawner, flags, plan)
            sweep.append({"tag": tag, "width": width, **parsed})
            total_cells += len(plan)
    smoke_plan = cohort_plan(1, "smoke")
    smoke_flags = cohort_flags(binary, root / "smoke", 1, "smoke", smoke_plan)
    smoke = run_spawner_dry_run(spawner, smoke_flags, smoke_plan)
    return {
        "argparse_accepts": True, "cells_run": 0,
        "sweep_waves": len(sweep), "sweep_cells": total_cells,
        "sweep": sweep, "smoke": smoke,
    }


def find_landmine_tool(root: Path) -> Path | None:
    for relative in ("preflight.py", "tools/cohort_preflight.py", "cohort_preflight.py"):
        candidate = root / relative
        if candidate.is_file():
            return candidate
    return None


def make_landmine_spec(args: argparse.Namespace, path: Path) -> dict[str, Any]:
    driver = Path(__file__).resolve()
    binary = Path(args.binary).resolve()
    smoke_root = Path(args.smoke_root).resolve()
    smoke_result = Path(args.smoke_result).resolve()
    flags = cohort_flags(binary, smoke_root)
    smoke_command = [
        sys.executable, str(driver), "smoke",
        "--box", str(args.box), "--root", str(smoke_root),
        "--back-dir", str(Path(args.back_dir).expanduser().resolve()),
        "--spawner", str(Path(args.spawner).resolve()),
        "--broker-host", args.broker_host, "--broker-port", str(args.broker_port),
        "--cpu-per-cell", str(args.cpu_per_cell),
        "--lease-ttl", "600", "--heartbeat-seconds", "180",
        "--grant-window-seconds", "120",
        "--smoke-result", str(smoke_result),
    ] + flags
    if args.smoke_fixture:
        smoke_command += ["--simulation-result-fixture",
                          str(Path(args.smoke_fixture).resolve())]
    broker_query = [
        sys.executable, str(driver), "broker-caps",
        "--broker-host", args.broker_host,
        "--broker-port", str(args.broker_port), "--box", str(args.box),
    ]
    if args.broker_status_fixture:
        broker_query += ["--status-fixture",
                         str(Path(args.broker_status_fixture).resolve())]
    spec = {
        "name": VERSION,
        "env": CONTRACT["environment"],
        "driver": str(driver),
        "reducer": str(driver),
        "flags": flags,
        "reducer_fields": list(PREFLIGHT_RESULT_FIELDS),
        "cohort_lease": {"cards": 1, "cpu": args.cpu_per_cell},
        "broker": {"query_command": broker_query},
        "smoke": {
            "cells": 1,
            "cell_flag": "--n",
            "lease": {"cards": 1, "cpu": args.cpu_per_cell},
            "command": smoke_command,
            "result_json": str(smoke_result),
            "env_attestation_field": "contention_cert_smoke_env",
            "lease_attestation_field": "contention_cert_smoke_lease",
        },
        "lever_policies": {},
        "functional_gates": [
            "binary functional battery passed",
            "driver offline self-test passed",
            "one-cell exact-recipe smoke passed",
        ],
        "durable_paths": [str(Path(args.back_dir).expanduser().resolve())],
        "source_repo": str(Path(args.source_repo).resolve()),
        "harness_paths": [str(driver), str(Path(args.spawner).resolve())],
        "contract_sha256": contract_sha256(),
        "generated_spec_path": str(path),
    }
    spec["_".join(("sha", "equality", "gate"))] = False
    return spec


def run_preflight(args: argparse.Namespace) -> int:
    if bool(args.broker_status_fixture) != bool(args.smoke_fixture):
        raise CertificationError(
            "simulation preflight requires both --broker-status-fixture and "
            "--smoke-fixture"
        )
    checks: list[dict[str, Any]] = []

    def check(name: str, ok: bool, detail: str) -> None:
        checks.append({"check": name, "ok": bool(ok), "detail": detail})

    binary = Path(args.binary).resolve()
    spawner = Path(args.spawner).resolve()
    root = Path(args.root).resolve()
    back = Path(args.back_dir).expanduser().resolve()
    check("contract", CONTRACT["scored_cells_per_width"] == 16 and
          CONTRACT["scored_seeds"] == list(range(1, 17)), contract_sha256())
    check("binary", binary.is_file() and os.access(binary, os.X_OK), str(binary))
    check("spawner", spawner.is_file(), str(spawner))
    dialect_ok = False
    try:
        dialect_probe = dry_run_spawner(spawner, binary)
        dialect_ok = True
        check("spawner_committed_dialect", True, canonical_json(dialect_probe))
    except Exception as exc:
        check("spawner_committed_dialect", False, str(exc))
    check("committed_dialect_contract", dialect_ok,
          "exact full-dialect spawner --dry-run argparse pass")
    check("volatile_run_root", str(root).startswith("/dev/shm/"), str(root))
    volatile_tmp = "/" + "tmp" + "/"
    check("persistent_output", not str(back).startswith((volatile_tmp, "/dev/shm/")), str(back))
    check("stress_ng", Path(args.stress_ng).is_file(), args.stress_ng)
    check("taskset", shutil.which("taskset") is not None, str(shutil.which("taskset")))
    allowed = os.sched_getaffinity(0) if hasattr(os, "sched_getaffinity") else set(args.stress_cpus)
    invalid_cpus = sorted(set(args.stress_cpus) - set(allowed))
    check("stress_cpu_affinity", not invalid_cpus, f"invalid={invalid_cpus}")
    check("fresh_run", not (root / "run").exists(), str(root / "run"))
    check("fresh_archive", not (back / f"box{args.box}_{contract_sha256()[:12]}").exists(), str(back))
    source_repo = Path(args.source_repo).resolve()
    tracked_detail = []
    tracked_ok = True
    for path in (Path(__file__).resolve(), spawner):
        try:
            relative = path.relative_to(source_repo)
        except ValueError:
            tracked_ok = False
            tracked_detail.append(f"outside source repo: {path}")
            continue
        completed = subprocess.run(
            ["git", "-C", str(source_repo), "ls-files", "--error-unmatch",
             relative.as_posix()], text=True, capture_output=True,
        )
        if completed.returncode != 0:
            tracked_ok = False
            tracked_detail.append(f"untracked: {relative}")
    check("tracked_driver_harness", tracked_ok,
          "; ".join(tracked_detail) if tracked_detail else str(source_repo))

    broker_status = None
    try:
        if args.broker_status_fixture:
            broker_status = Path(args.broker_status_fixture).resolve().read_text()
        else:
            broker = BrokerClient(args.broker_host, args.broker_port, lambda _message: None)
            broker_status = broker.command("STATUS", timeout=10)
        idle, idle_detail = all_boxes_idle(broker_status)
        check("broker_status", True, broker_status)
        check("box_idle", idle, idle_detail)
    except Exception as exc:
        check("broker_status", False, str(exc))
        check("box_idle", False, "broker status unavailable")

    landmine_root = Path(args.landmine_root).resolve()
    tool = find_landmine_tool(landmine_root) if landmine_root.exists() else None
    spec_path = Path(args.preflight_out).resolve().with_suffix(".landmine-spec.json")
    if args.smoke_result is None:
        args.smoke_result = str(Path(args.smoke_root).resolve() / "smoke_result.json")
    spec = make_landmine_spec(args, spec_path)
    atomic_json(spec_path, spec)
    if tool is None:
        check("landmine_preflight", False,
              f"no cohort_preflight.py under existing path {landmine_root}")
        landmine = {"tool": None, "rc": None, "stdout": "", "stderr": ""}
    elif not all(item["ok"] for item in checks):
        check("landmine_preflight", False,
              "live gate deferred because a non-workload prerequisite failed")
        landmine = {"tool": str(tool), "rc": None, "stdout": "", "stderr": "",
                    "deferred": True}
    else:
        # The gate's smoke runs one full contract cell; its timeout must cover
        # the registered horizon plus connect/warm/teardown, never the tool's
        # 300 s default (which guillotines an 800 s smoke at rc=124).
        gate_timeout = float(CONTRACT["seconds"]) + 300.0
        command = [sys.executable, str(tool), str(spec_path),
                   "--timeout", str(gate_timeout)]
        completed = subprocess.run(command, text=True, capture_output=True, timeout=3600)
        check("landmine_preflight", completed.returncode == 0,
              f"rc={completed.returncode} command={command}")
        landmine = {"tool": str(tool), "command": command, "rc": completed.returncode,
                    "stdout": completed.stdout, "stderr": completed.stderr}

    if args.smoke_result:
        smoke_path = Path(args.smoke_result).resolve()
        try:
            smoke = decode_json(smoke_path.read_text())
            metric_hot(smoke)
            for field in ("connected", "delivered_full", "byte_integrity_ok"):
                if not isinstance(smoke.get(field), bool):
                    raise CertificationError(f"smoke missing boolean {field}")
            check("result_schema_smoke", True, str(smoke_path))
        except Exception as exc:
            check("result_schema_smoke", False, str(exc))
    else:
        check("result_schema_smoke", False, "--smoke-result is required")

    ok = all(item["ok"] for item in checks)
    record = {
        "ok": ok, "campaign": VERSION, "contract_sha256": contract_sha256(),
        "simulation": bool(args.broker_status_fixture or args.smoke_fixture),
        "launch_eligible": ok and not bool(
            args.broker_status_fixture or args.smoke_fixture),
        "created_epoch": time.time(),
        "created_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "host": socket.gethostname(), "target_box": args.box,
        "checks": checks, "broker_status": broker_status,
        "landmine": landmine, "landmine_spec": str(spec_path),
    }
    atomic_json(Path(args.preflight_out).resolve(), record)
    print(json.dumps(record, indent=2, sort_keys=True))
    return 0 if ok else 1


def validate_preflight_record(path: Path, box: int) -> dict[str, Any]:
    try:
        record = decode_json(path.read_text())
    except Exception as exc:
        raise CertificationError(f"cannot read preflight record {path}: {exc}") from exc
    failures = []
    if record.get("ok") is not True:
        failures.append("ok is not true")
    if record.get("simulation") is True or record.get("launch_eligible") is not True:
        failures.append("simulation records are not launch eligible")
    if record.get("contract_sha256") != contract_sha256():
        failures.append("contract hash differs")
    if record.get("target_box") != box:
        failures.append("target box differs")
    if record.get("host") != socket.gethostname():
        failures.append("preflight ran on another host")
    age = time.time() - float(record.get("created_epoch", 0))
    if age < 0 or age > PREFLIGHT_MAX_AGE_SECONDS:
        failures.append(f"preflight age invalid: {age:.0f}s")
    if failures:
        raise CertificationError("invalid preflight record: " + "; ".join(failures))
    return record


def broker_caps_main(args: argparse.Namespace) -> int:
    if args.status_fixture:
        status = Path(args.status_fixture).resolve().read_text()
    else:
        broker = BrokerClient(args.broker_host, args.broker_port, lambda _message: None)
        status = broker.command("STATUS", timeout=10)
    try:
        decoded = decode_broker_status(status)
    except json.JSONDecodeError:
        decoded = None
    if isinstance(decoded, dict):
        candidates = [decoded]
        if isinstance(decoded.get("caps"), dict):
            candidates.append(decoded["caps"])
        cards = next((item.get("max_cards_per_lease") for item in candidates
                      if is_int(item.get("max_cards_per_lease"))), None)
        cpu = next((item.get("max_cpu_per_lease") for item in candidates
                    if is_int(item.get("max_cpu_per_lease"))), None)
    else:
        cards_match = re.search(r"max_cards_per_lease\s*[=:]\s*(\d+)", status, re.I)
        cpu_match = re.search(r"max_cpu_per_lease\s*[=:]\s*(\d+)", status, re.I)
        cards = int(cards_match.group(1)) if cards_match else None
        cpu = int(cpu_match.group(1)) if cpu_match else None
    if cards is None or cpu is None:
        raise CertificationError(
            "broker STATUS did not expose max_cards_per_lease and max_cpu_per_lease"
        )
    print(json.dumps({
        "max_cards_per_lease": cards,
        "max_cpu_per_lease": cpu,
        "target_box": args.box,
        "status_raw": status,
    }, sort_keys=True))
    return 0


def smoke_main(args: argparse.Namespace) -> int:
    expected = {
        "n": 1,
        "start_cfg": CONTRACT["start_cfg"],
        "warm_start": True,
        "traffic": CONTRACT["traffic"],
        "payload": CONTRACT["payload_bytes"],
        "secs": CONTRACT["seconds"],
        "score_horizon_s": float(CONTRACT["seconds"]),
        "snr": float(CONTRACT["dial_db"]),
        "snr3k": float(CONTRACT["dial_db"]),
        "profile": CONTRACT["profile"],
        "card_base": 0,
        "port_base": 7100,
        "seed_offset": 0,
        "launch_stagger": float(CONTRACT["launch_stagger_seconds"]),
        "arm": "smoke",
        "tag_prefix": "smoke_",
    }
    mismatches = [f"{key}={getattr(args, key)!r}, expected {value!r}"
                  for key, value in expected.items() if getattr(args, key) != value]
    expected_env = {
        f"{key}={value}" for key, value in CONTRACT["environment"].items()
    }
    if set(args.env) != expected_env:
        mismatches.append(f"env={args.env!r}, expected {sorted(expected_env)!r}")
    expected_plan = cohort_plan(1, "smoke")
    try:
        supplied_plan = decode_json(args.spawn_plan)
    except json.JSONDecodeError as exc:
        mismatches.append(f"spawn_plan is invalid JSON: {exc}")
    else:
        if supplied_plan != expected_plan:
            mismatches.append("spawn_plan differs from exact smoke plan")
    if mismatches:
        raise CertificationError("smoke recipe mismatch: " + "; ".join(mismatches))
    args.binary = args.bin
    args.stress_cpus = [0]
    args.stress_ng = "/usr/bin/stress-ng"
    root = Path(args.root).resolve()
    smoke_result = Path(args.smoke_result).resolve()
    if root.exists():
        raise CertificationError(f"fresh smoke root required; path exists: {root}")
    if smoke_result.exists():
        raise CertificationError(f"fresh smoke result required; path exists: {smoke_result}")
    if args.simulation_result_fixture:
        fixture_path = Path(args.simulation_result_fixture).resolve()
        result = decode_json(fixture_path.read_text())
        metric_hot(result)
        for field in ("connected", "delivered_full", "byte_integrity_ok"):
            if not isinstance(result.get(field), bool):
                raise CertificationError(f"simulation smoke fixture missing boolean {field}")
        result["contention_cert_smoke_env"] = dict(CONTRACT["environment"])
        result["contention_cert_smoke_lease"] = {
            "cards": 1, "cpu": args.cpu_per_cell,
        }
        result["contention_cert_contract_sha256"] = contract_sha256()
        result["contention_cert_simulation"] = True
        smoke_result.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(smoke_result, result)
        print(json.dumps({
            "status": "SIMULATION_FIXTURE_PASS", "cells_run": 0,
            "smoke_result": str(smoke_result), "fixture": str(fixture_path),
        }, sort_keys=True))
        return 0
    driver = Driver(args)
    driver.run_dir.mkdir(parents=True)
    smoke_result.parent.mkdir(parents=True, exist_ok=True)

    def request_stop(signum, _frame):
        driver.stop.set()
        raise StopRequested(f"received signal {signum}")

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        job = {"arm": "smoke", "width": 1, "stress_percent": 0}
        wave = {"wave": 0, "width": 1, "port_base": 7100,
                "cells": [{"seed": 1, "scored": True}]}
        driver.run_wave(0, job, wave)
        rows = load_rows(driver.cells_path)
        if len(rows) != 1:
            raise CertificationError(f"smoke expected one compact result, got {len(rows)}")
        raw_path = driver.run_dir / rows[0]["result_path"]
        result = decode_json(raw_path.read_text())
        result["spawner_width"] = rows[0]["spawner_width"]
        result["bridge_underruns"] = rows[0]["bridge_underruns"]
        result["requested_snr3k"] = args.snr3k
        result["snr3k_attested"] = (
            result.get("snr") == args.snr and result.get("snr3k") == args.snr3k
        )
        result["contention_cert_smoke_env"] = dict(CONTRACT["environment"])
        result["contention_cert_smoke_lease"] = {
            "cards": 1, "cpu": args.cpu_per_cell,
        }
        result["contention_cert_contract_sha256"] = contract_sha256()
        atomic_json(smoke_result, result)
        atomic_json(driver.run_dir / "DONE.json", {
            "status": "OK", "contract_sha256": contract_sha256(),
            "smoke_result": str(smoke_result),
        })
        return 0
    except Exception as exc:
        atomic_json(driver.run_dir / "DONE.json", {
            "status": "FAIL", "contract_sha256": contract_sha256(),
            "error": str(exc),
        })
        print(f"SMOKE FAIL: {exc}\n{traceback.format_exc()}", file=sys.stderr)
        return 1
    finally:
        driver.terminate_child()
        if driver.current_stress is not None:
            driver.stop_stress(driver.current_stress, "smoke_emergency")
        if driver.owned_leases:
            driver.release_leases(driver.owned_leases)


def score_fixture_main(args: argparse.Namespace) -> int:
    result_path = Path(args.result).resolve()
    bridge_path = Path(args.bridge_stats).resolve()
    result = decode_json(result_path.read_text())
    bridge_stats = decode_json(bridge_path.read_text())
    hot, bridge, rx = metric_hot(result, bridge_stats)
    scored = {
        "result": str(result_path), "bridge_stats": str(bridge_path),
        "primary_hot": hot, "bridge_underrun_total": bridge,
        "rx_overrun_total_normalized": rx,
    }
    print(json.dumps(scored, indent=2, sort_keys=True))
    return 0


def dry_run_main(args: argparse.Namespace) -> int:
    result = dry_run_exact_commands(
        Path(args.spawner).resolve(), Path(args.bin).resolve())
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def self_test() -> int:
    assert contract_sha256() == contract_sha256()
    plan = plan_document(31)
    assert plan["totals"] == {"jobs": 8, "waves": 23, "scored_cells": 128,
                              "live_cells": 136}
    width12 = build_waves({"arm": "width", "width": 12, "stress_percent": 0})
    assert [len(wave["cells"]) for wave in width12] == [12, 12]
    assert sum(cell["scored"] for wave in width12 for cell in wave["cells"]) == 16
    assert sum(not cell["scored"] for wave in width12 for cell in wave["cells"]) == 8
    clean = {"bridge_underruns": {"fwd": 0, "rev": 0, "total": 0,
                                   "source": "bridge_stats"},
             "rx_overrun_total": 0}
    assert metric_hot(clean) == (False, 0, 0)
    committed_clean = {"rx_overrun_total": 0}
    committed_bridge = {
        "fwd": {"frames": 1, "sig_frames": 1, "underruns": 0},
        "rev": {"frames": 1, "sig_frames": 1, "underruns": 0},
    }
    assert metric_hot(committed_clean, committed_bridge) == (False, 0, 0)
    status = 'OK {"boxes":{"11":{"active_leases":0},"21":{"active_leases":0}}}'
    assert all_boxes_idle(status)[0] is True
    busy = 'OK {"boxes":{"11":{"active_leases":0},"21":{"active_leases":1}}}'
    assert all_boxes_idle(busy)[0] is False
    hot = {"bridge_underruns": {"fwd": 0, "rev": 1, "total": 1,
                                 "source": "bridge_stats"},
           "rx_overrun_total": 0}
    assert metric_hot(hot) == (True, 1, 0)
    hot_rx = {"bridge_underruns": {"fwd": 0, "rev": 0, "total": 0},
              "rx_overrun_total": {"cmd": 0, "rsp": 2, "total": 2}}
    assert metric_hot(hot_rx) == (True, 0, 2)
    try:
        bridge_total({"fwd": 1, "rev": 0, "total": 0})
    except CertificationError:
        pass
    else:
        raise AssertionError("inconsistent bridge total did not fail")
    with tempfile.TemporaryDirectory(prefix="contention-cert-selftest-") as temp:
        cells = Path(temp) / "CELL_RESULTS.jsonl"
        synthetic = []
        for width in CONTRACT["widths"]:
            job = {"arm": "width", "width": width, "stress_percent": 0}
            for wave in build_waves(job):
                for cell in wave["cells"]:
                    synthetic.append({
                        "campaign": VERSION, "contract_sha256": contract_sha256(),
                        "target_box": 31, "job_arm": "width", "width": width,
                        "stress_percent": 0, "wave": wave["wave"],
                        "tag": f"w{width}_{cell['seed']}", "seed": cell["seed"],
                        "scored": cell["scored"], "bridge_underrun_total": 0,
                        "rx_overrun_total_normalized": 0, "primary_hot": False,
                        "connected": True, "delivered_full": True,
                        "byte_integrity_ok": True,
                    })
        for level in CONTRACT["stress"]["nonzero_levels_run"]:
            job = {"arm": "stress", "width": 8, "stress_percent": level}
            for wave in build_waves(job):
                for cell in wave["cells"]:
                    synthetic.append({
                        "campaign": VERSION, "contract_sha256": contract_sha256(),
                        "target_box": 31, "job_arm": "stress", "width": 8,
                        "stress_percent": level, "wave": wave["wave"],
                        "tag": f"l{level}_{cell['seed']}", "seed": cell["seed"],
                        "scored": cell["scored"], "bridge_underrun_total": 0,
                        "rx_overrun_total_normalized": 0, "primary_hot": False,
                        "connected": True, "delivered_full": True,
                        "byte_integrity_ok": True,
                    })
        for row in synthetic:
            if row["job_arm"] == "width" and row["width"] == 12 and row["scored"]:
                row["rx_overrun_total_normalized"] = 1
                row["primary_hot"] = True
                break
        cells.write_text("".join(canonical_json(row) + "\n" for row in synthetic))
        measurement, policy = reduce_cells(cells, 31)
        assert measurement["knee_cells"] == 12
        assert measurement["discriminator"] == "width_per_se_supported"
        assert policy["derived_broker_width_cap_cells"] == 10
    print(f"SELF-TEST PASS contract={contract_sha256()} plan={plan['totals']}")
    return 0


def add_site_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--box", type=int, default=31)
    parser.add_argument("--root", default=str(DEFAULT_ROOT))
    parser.add_argument("--back-dir", default=str(DEFAULT_BACK))
    parser.add_argument("--binary", default=str(DEFAULT_BINARY))
    parser.add_argument("--spawner", default=str(DEFAULT_SPAWNER))
    parser.add_argument("--broker-host", default="127.0.0.1")
    parser.add_argument("--broker-port", type=int, default=7800)
    parser.add_argument("--cpu-per-cell", type=int, default=2)
    parser.add_argument("--stress-cpus", type=parse_cpu_list, default=parse_cpu_list("0-7"))
    parser.add_argument("--stress-ng", default="/usr/bin/stress-ng")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    plan = subparsers.add_parser("plan", help="print the immutable offline execution plan")
    plan.add_argument("--box", type=int, default=31)

    preflight = subparsers.add_parser(
        "preflight", help="run launch checks, including the one-cell schema smoke")
    add_site_arguments(preflight)
    preflight.add_argument("--landmine-root", default=str(DEFAULT_LANDMINE_ROOT))
    preflight.add_argument("--source-repo", default=str(Path(__file__).resolve().parents[2]))
    preflight.add_argument("--smoke-root", default=str(DEFAULT_ROOT.with_name(
        DEFAULT_ROOT.name + "_preflight")))
    preflight.add_argument("--smoke-result", help="exact-recipe smoke output path")
    preflight.add_argument(
        "--broker-status-fixture",
        help="simulation-only STATUS response; resulting record cannot authorize launch")
    preflight.add_argument(
        "--smoke-fixture",
        help="simulation-only result fixture; resulting record cannot authorize launch")
    preflight.add_argument("--preflight-out", required=True)

    run = subparsers.add_parser("run", help="run the preregistered campaign after approval")
    add_site_arguments(run)
    run.add_argument("--approval-token", required=True)
    run.add_argument("--preflight-record", required=True)
    run.add_argument("--lease-ttl", type=int, default=600)
    run.add_argument("--heartbeat-seconds", type=int, default=180)
    run.add_argument("--grant-window-seconds", type=int, default=120)

    reduce = subparsers.add_parser("reduce", help="independently reduce a completed cell table")
    reduce.add_argument("--run-dir", type=Path, required=True)
    reduce.add_argument("--box", type=int, required=True)

    broker_caps = subparsers.add_parser("broker-caps", help=argparse.SUPPRESS)
    broker_caps.add_argument("--broker-host", default="127.0.0.1")
    broker_caps.add_argument("--broker-port", type=int, default=7800)
    broker_caps.add_argument("--box", type=int, required=True)
    broker_caps.add_argument("--status-fixture")

    smoke = subparsers.add_parser("smoke", help=argparse.SUPPRESS)
    smoke.add_argument("--box", type=int, required=True)
    smoke.add_argument("--root", required=True)
    smoke.add_argument("--back-dir", required=True)
    smoke.add_argument("--spawner", required=True)
    smoke.add_argument("--broker-host", default="127.0.0.1")
    smoke.add_argument("--broker-port", type=int, default=7800)
    smoke.add_argument("--cpu-per-cell", type=int, required=True)
    smoke.add_argument("--lease-ttl", type=int, default=600)
    smoke.add_argument("--heartbeat-seconds", type=int, default=180)
    smoke.add_argument("--grant-window-seconds", type=int, default=120)
    smoke.add_argument("--smoke-result", required=True)
    smoke.add_argument("--n", type=int, required=True)
    smoke.add_argument("--bin", required=True)
    smoke.add_argument("--start-cfg", type=int, required=True)
    smoke.add_argument("--payload", type=int, required=True)
    smoke.add_argument("--secs", type=int, required=True)
    smoke.add_argument("--score-horizon-s", type=float, required=True)
    smoke.add_argument("--warm-start", action="store_true")
    smoke.add_argument("--traffic", required=True)
    smoke.add_argument("--snr", type=float, required=True)
    smoke.add_argument("--snr3k", type=float, required=True)
    smoke.add_argument("--profile", required=True)
    smoke.add_argument("--card-base", type=int, required=True)
    smoke.add_argument("--port-base", type=int, required=True)
    smoke.add_argument("--seed-offset", type=int, required=True)
    smoke.add_argument("--spawn-plan", required=True)
    smoke.add_argument("--launch-stagger", type=float, required=True)
    smoke.add_argument("--arm", required=True)
    smoke.add_argument("--tag-prefix", required=True)
    smoke.add_argument("--logdir", required=True)
    smoke.add_argument("--out", required=True)
    smoke.add_argument("--env", action="append", default=[])
    smoke.add_argument("--simulation-result-fixture")

    dry_run = subparsers.add_parser(
        "dry-run-spawn",
        help="spawner --dry-run every exact sweep wave and smoke command")
    dry_run.add_argument("--spawner", required=True)
    dry_run.add_argument("--bin", required=True)

    fixture = subparsers.add_parser(
        "score-fixture", help="score one committed result plus its bridge stats")
    fixture.add_argument("--result", required=True)
    fixture.add_argument("--bridge-stats", required=True)
    subparsers.add_parser("self-test", help="run offline contract and reducer tests")
    return parser


def main() -> int:
    parser = make_parser()
    args, passthrough = parser.parse_known_args()
    if passthrough:
        parser.error("unrecognized arguments: " + " ".join(passthrough))
    if args.command == "plan":
        print(json.dumps(plan_document(args.box), indent=2, sort_keys=True))
        return 0
    if args.command == "preflight":
        return run_preflight(args)
    if args.command == "broker-caps":
        return broker_caps_main(args)
    if args.command == "smoke":
        return smoke_main(args)
    if args.command == "dry-run-spawn":
        return dry_run_main(args)
    if args.command == "score-fixture":
        return score_fixture_main(args)
    if args.command == "run":
        if args.approval_token != APPROVAL_TOKEN:
            raise CertificationError("launch approval token mismatch")
        if args.cpu_per_cell < 1:
            raise CertificationError("cpu-per-cell must be positive")
        validate_preflight_record(Path(args.preflight_record).resolve(), args.box)
        return Driver(args).run()
    if args.command == "reduce":
        measurement, policy = reduce_cells(args.run_dir / "CELL_RESULTS.jsonl", args.box)
        atomic_json(args.run_dir / "CERT_MEASUREMENTS.json", measurement)
        atomic_json(args.run_dir / "BROKER_POLICY.json", policy)
        print(json.dumps({"measurement": measurement, "policy": policy}, indent=2,
                         sort_keys=True))
        return 0
    if args.command == "self-test":
        return self_test()
    raise CertificationError(f"unknown command {args.command}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"FATAL: {exc}", file=sys.stderr)
        sys.exit(1)

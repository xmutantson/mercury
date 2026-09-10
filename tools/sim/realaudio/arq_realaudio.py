#!/usr/bin/env python3
"""
arq_realaudio.py - drive TWO stock `-x alsa` Mercury instances through the
real-audio IONOS bridge over snd-aloop.

Mirrors tools/sim_arq_channel.py EXACTLY (same ctrl/data TCP protocol: ctrl on
PORT, data on PORT+1, MYCALL/LISTEN/CONNECT, tx=bytes(range(256))*8 chunks,
rx=recv) - the ONLY change vs the canonical sim harness is the audio transport:
-x sim TCP relay  ->  -x alsa real audio through the bridge.

This is the production copy of the prototype proven in wf_1e5c12f1.

One run = 4 snd-aloop cables (substreams). snd-aloop wires playback (dev0,subN)
to capture (dev1,subN), so each cable is one substream index used on BOTH dev0
(play side) and dev1 (capture side). A run consumes 4 substreams S0..S3 of one
ALSA card `--card` (default "Loopback"):

  Commander  -o hw:<card>,0,<S0> (TX)   -i hw:<card>,1,<S3> (RX)
  Responder  -i hw:<card>,1,<S1> (RX)   -o hw:<card>,0,<S2> (TX)
  Bridge FWD cap hw:<card>,1,<S0> -> impair -> play hw:<card>,0,<S1>  (CMD->RSP)
  Bridge REV cap hw:<card>,1,<S2> -> impair -> play hw:<card>,0,<S3>  (RSP->CMD)

For N concurrent runs each must own a DISJOINT set of 4 substreams. The parallel
spawner allocates them; here we accept the 4 substream indices + card name +
the two TCP control ports so concurrent runs do not collide.

Uses raw hw: (NOT plughw:) on both the bridge AND the modems so NO plug
resample/requantize - Mercury negotiates INT32/2ch/48k and the bridge opens
S32/2ch/48k to match (zero plug conversion on either side).
"""
import argparse
import hashlib
import json
import math
import os
import re
import socket
import subprocess
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))
from ra_cleanup import scoped_cleanup  # concurrency-safe per-run cleanup
from sim_axis import (AXIS_CHOICES, AXIS_V0, AXIS_V2, DEFAULT_AXIS, AxisError,
                      calibration_key, infer_bandwidth,
                      load_and_select_calibration, resolve_axis, sha256_file,
                      sha256_json, validate_attestation)
from sim_channel_relay import parse_cell

CONNECT_RE = re.compile(r"link_status:Connected to")
DISC_RE = re.compile(r"link_status:Disconnected|DISCONNECTED")
NRECV_RE = re.compile(r"stats\.nReceived_data=\s*(\d+)")
# load_configuration(N) current=M : N (group 1) = TARGET config being LOADED
# (becomes active at arq_common.cc:2129); M (group 2) = OUTGOING/previous config,
# printed BEFORE the assignment. The crossing scorer MUST key on the TARGET N, NOT
# on current=M -- keying on M recorded the pre-load config (e.g.
# load_configuration(0) current=102 scored as "102/still-ROBUST"), the BUG-A parse
# bug that produced the all-session no-cross flip-flops and the lying
# wb_configs_seen=[]. See score_climb_canonical.py + CROSSING_GROUNDTRUTH.md (1).
CFG_RE = re.compile(r"load_configuration\((\d+)\)\s+current=(-?\d+)")
BREAK_RE = re.compile(r"\[BREAK\] Block failure")
# AEAD auth-fail markers: the env-gated nonce trace (DEC-AUTHFAIL) AND any
# generic crypto auth-failure / PSK-mismatch the modem already logs.
AUTHFAIL_RE = re.compile(r"NONCE-TRACE\] DEC-AUTHFAIL|auth.?fail|PSK mismatch|decrypt.*fail",
                         re.IGNORECASE)
NONCE_ENC_RE = re.compile(r"NONCE-TRACE\] ENC dir=(\d+) idx=(\d+)")
NONCE_DECOK_RE = re.compile(r"NONCE-TRACE\] DEC-OK dir=(\d+) idx=(\d+)")
ENC_ACT_RE = re.compile(r"\[CRYPTO\] Encryption ACTIVATED")
# Warm-start readiness from the proven campaign harness: the initial Idle line
# precedes the one-time PHY precompute, so only an Idle after PRECOOK may open
# the gate.
PRECOOK_DONE_RE = re.compile(r"\[PRECOOK\] config bundles built")
IDLE_RE = re.compile(r"link_status:Idle")
# PRODUCTION crypto emissions — the lines the modem ACTUALLY logs on the live
# path (the NONCE-TRACE lines above are env-gated debug and are never emitted,
# so the durable counters read all-0 without these). [CRYPTO-TX] is the AEAD
# seal, [CRYPTO-RX] "Decrypted: N -> M bytes OK" is a successful open.
CRYPTO_TX_RE = re.compile(r"\[CRYPTO-TX\] Encrypting \d+ bytes, wire_bsi=\d+ index=(\d+) dir=(\d+)")
CRYPTO_RX_OK_RE = re.compile(r"\[CRYPTO-RX\] Decrypted: \d+ -> \d+ bytes OK")
# Durable cross signal: an RSP-side WB-OFDM DATA frame decoded (batch or arq path).
RXDATA_RE = re.compile(r"\[RX-BATCH-SEQ\]\s+type=DATA|\[RX-DATA\]\s+type=")
RX_OVERRUN_MARKER = "RX-OVERRUN-TOTAL"
RX_OVERRUN_RE = re.compile(r"RX-OVERRUN-TOTAL n=(\d+)$")
SENSITIVE_ENV_RE = re.compile(
    r"(?:PASS|PASSWORD|PSK|SECRET|TOKEN|CREDENTIAL|PRIVATE|API[_-]?KEY)",
    re.IGNORECASE)

TRAFFIC_LEGACY = "legacy"
TRAFFIC_RANDOM_BINARY = "random-binary"
TRAFFIC_CHOICES = (TRAFFIC_LEGACY, TRAFFIC_RANDOM_BINARY)
_LEGACY_CHUNK = bytes(range(256)) * 8
_RANDOM_BINARY_KEY = b"mercury-capstone-incompressible-v1"
_ELF_MAGIC = b"\x7fELF"


class BridgeCommandError(ValueError):
    """Raised when --bridge does not identify a supported bridge program."""


def bridge_command_prefix(bridge_path):
    """Return a fail-closed interpreter/scheduler argv for ``bridge_path``."""
    if not os.path.isfile(bridge_path):
        raise BridgeCommandError(
            f"invalid --bridge {bridge_path!r}: expected a regular file")
    if bridge_path.endswith(".py"):
        return [sys.executable, bridge_path]
    try:
        with open(bridge_path, "rb") as stream:
            is_elf = stream.read(len(_ELF_MAGIC)) == _ELF_MAGIC
    except OSError as exc:
        raise BridgeCommandError(
            f"invalid --bridge {bridge_path!r}: cannot read file: {exc}") from exc
    if not is_elf and not os.access(bridge_path, os.X_OK):
        raise BridgeCommandError(
            f"invalid --bridge {bridge_path!r}: unsupported bridge type; "
            "expected a .py script, an ELF binary, or an executable "
            "without a .py suffix")
    if os.path.basename(bridge_path) == "realaudio_bridge_s32_c":
        return ["sudo", "-n", "/usr/bin/nice", "-n", "-5", bridge_path]
    return [bridge_path]


def redacted_env_items(items):
    """Preserve experimental env settings without emitting credentials."""
    result = []
    for item in items:
        key, separator, value = item.partition("=")
        if separator and SENSITIVE_ENV_RE.search(key):
            result.append(key + "=<redacted>")
        else:
            result.append(item)
    return result


def random_binary_slice(offset, length):
    """Return a stable slice of the proven counter-mode SHA-256 byte stream."""
    if offset < 0 or length < 0:
        raise ValueError("offset and length must be nonnegative")
    if length == 0:
        return b""
    first_block = offset // 32
    last_block = (offset + length - 1) // 32
    data = b"".join(
        hashlib.sha256(
            _RANDOM_BINARY_KEY + counter.to_bytes(8, "little")
        ).digest()
        for counter in range(first_block, last_block + 1)
    )
    start = offset % 32
    return data[start:start + length]


def traffic_slice(traffic, offset, length):
    """Return expected application bytes for a traffic class and offset."""
    if traffic == TRAFFIC_RANDOM_BINARY:
        return random_binary_slice(offset, length)
    if traffic != TRAFFIC_LEGACY:
        raise ValueError(f"unknown traffic class: {traffic}")
    if offset < 0 or length < 0:
        raise ValueError("offset and length must be nonnegative")
    start = offset % len(_LEGACY_CHUNK)
    repeats = (start + length + len(_LEGACY_CHUNK) - 1) // len(_LEGACY_CHUNK)
    return (_LEGACY_CHUNK * repeats)[start:start + length]


def observation_active(now, legacy_start, secs, score_deadline=None):
    """Pure loop predicate: a fixed score deadline supersedes legacy --secs."""
    if score_deadline is not None:
        return now < score_deadline
    return now - legacy_start < secs


def delivery_ends_observation(score_deadline, connected, rx_bytes, payload):
    """Legacy cells end at delivery; fixed-horizon cells never do."""
    return (score_deadline is None and connected and rx_bytes >= payload)


def fixed_score_deadline(t0, connect_issued_at, horizon_s,
                         connected_at_cold=None):
    """Cover both request-horizon UX and a full post-connect score horizon."""
    deadline = t0 + connect_issued_at + horizon_s
    if connected_at_cold is not None:
        deadline = max(deadline, t0 + connected_at_cold + horizon_s)
    return deadline


class ByteIntegrityTracker:
    """Session-aware streaming byte oracle shared by legacy and random-binary
    traffic.

    Each ARQ session restarts the application payload at offset 0.  A single
    logical cell can span several sessions: a disconnect/reconnect drops the
    RF link but leaves the local KISS data socket open, and the responder
    re-delivers the transfer from its beginning on the fresh session.  A byte
    oracle that compares the concatenated wire stream against ONE monotonic
    source offset therefore SHEARS at every session boundary -- session two's
    bytes (delivered from application offset 0) are checked against the source
    at the running total, a false mismatch that reads as corruption.  The
    contention-certification round-5 corrupt-cell drill traced exactly this:
    three sessions of 24 + 0 + 18432 bytes concatenated to 18456 with the shear
    at byte 24, a ~1/256 spurious match rate.

    note_disconnect() realigns the expected-source offset to 0 at each session
    boundary so every session is scored against its own fresh payload prefix,
    while the cumulative mismatch evidence (a genuine mid-session corruption)
    is preserved across boundaries.  A per-session delivered-byte high-water
    mark backs an honest completion check: a cell that only reaches the payload
    target by re-delivering a prefix across sessions has NOT delivered the
    payload in one clean session and must not count as complete.
    """

    _RAW_CAP_BYTES = 4 << 20        # forensic rx-buffer ceiling (per cell)
    _CAP_CHUNKS = 8                 # first N mismatched chunks kept for capbytes
    _CAP_SPAN = 64                  # bytes retained per captured mismatch chunk

    def __init__(self, traffic):
        self.traffic = traffic
        # session_offset realigns to 0 at each disconnect; total_bytes never does.
        self.session_offset = 0
        self.total_bytes = 0
        self.max_session_bytes = 0
        self.mismatch_bytes = 0
        self.mismatch_segments = 0
        self.first_bad_offset = None
        self.disconnects = 0
        self.sessions = 1
        self._delivered_md5 = hashlib.md5()
        self._expected_md5 = hashlib.md5()
        self._cap = []
        self._raw = bytearray()
        self.lock = threading.Lock()

    def feed(self, data):
        data = bytes(data)
        with self.lock:
            expected = traffic_slice(self.traffic, self.session_offset, len(data))
            bad = [i for i, pair in enumerate(zip(data, expected))
                   if pair[0] != pair[1]]
            if bad:
                self.mismatch_bytes += len(bad)
                self.mismatch_segments += 1
                if self.first_bad_offset is None:
                    self.first_bad_offset = self.total_bytes + bad[0]
                if len(self._cap) < self._CAP_CHUNKS:
                    self._cap.append((
                        self.total_bytes,
                        bytes(data[:self._CAP_SPAN]),
                        bytes(expected[:self._CAP_SPAN]),
                    ))
            self._delivered_md5.update(data)
            self._expected_md5.update(expected)
            self.session_offset += len(data)
            self.total_bytes += len(data)
            if self.session_offset > self.max_session_bytes:
                self.max_session_bytes = self.session_offset
            if len(self._raw) < self._RAW_CAP_BYTES:
                self._raw += data[:self._RAW_CAP_BYTES - len(self._raw)]

    def note_disconnect(self):
        """Realign the expected-source offset to 0 for the next ARQ session."""
        with self.lock:
            self.disconnects += 1
            if self.session_offset > 0:
                self.sessions += 1
            self.session_offset = 0

    @property
    def ok(self):
        with self.lock:
            return self.mismatch_bytes == 0

    def snapshot(self):
        with self.lock:
            delivered_md5 = self._delivered_md5.hexdigest()
            expected_md5 = self._expected_md5.hexdigest()
            return {
                "byte_integrity_ok": self.mismatch_bytes == 0,
                "integrity_mismatch_bytes": self.mismatch_bytes,
                "integrity_mismatch_segments": self.mismatch_segments,
                "integrity_first_bad_offset": self.first_bad_offset,
                "integrity_disconnects": self.disconnects,
                "integrity_sessions": self.sessions,
                "max_session_bytes": self.max_session_bytes,
                "total_received_bytes": self.total_bytes,
                "content_delivered_md5": delivered_md5,
                "content_expected_md5": expected_md5,
                "content_md5_ok": delivered_md5 == expected_md5,
            }

    def dump_forensics(self, capbytes_path, rx_bin_path):
        """Persist first-mismatch chunks + the raw received stream, ONLY when a
        content violation was observed.  A clean cell writes nothing; its
        durable proof of byte-identity is the delivered/expected md5 pair in the
        result JSON."""
        with self.lock:
            if self.mismatch_bytes == 0:
                return
            cap = list(self._cap)
            raw = bytes(self._raw)
            first_bad = self.first_bad_offset
            mism = self.mismatch_bytes
            segs = self.mismatch_segments
            total = self.total_bytes
        try:
            with open(capbytes_path, "w") as handle:
                handle.write(
                    f"first_bad={first_bad} total_rx={total} "
                    f"mismatch={mism} segs={segs}\n")
                for base, deliv, exp in cap:
                    handle.write(f"BASE {base} LEN {len(deliv)}\n")
                    handle.write(f"DELIV {deliv.hex()}\n")
                    handle.write(f"EXPEC {exp.hex()}\n")
        except OSError:
            pass
        try:
            with open(rx_bin_path, "wb") as handle:
                handle.write(raw)
        except OSError:
            pass


def is_wb_config(cfg):
    # WB OFDM = ids 0..16; ROBUST MFSK = 100/101/102 (common_defines.h:122-155).
    return cfg is not None and 0 <= cfg <= 16


def parse_rx_overrun_segments(lines):
    """Return every per-session RX overrun total in one merged cell log.

    reset_session_state() exchanges the modem counter back to zero, so a cell
    that reconnects can emit several segments.  A line containing the marker
    but not the production format is rejected instead of being misreported as
    an absent metric.
    """
    segments = []
    for lineno, line in enumerate(lines, 1):
        text = line.rstrip("\r\n")
        if RX_OVERRUN_MARKER not in text:
            continue
        match = RX_OVERRUN_RE.search(text)
        if match is None:
            raise ValueError(
                f"malformed {RX_OVERRUN_MARKER} line {lineno}: {text!r}")
        segments.append(int(match.group(1)))
    return segments


def parse_rx_overrun_metrics(lines):
    segments = parse_rx_overrun_segments(lines)
    return (sum(segments) if segments else None), segments


def read_rx_overrun_metrics(logpath):
    with open(logpath, "r", encoding="utf-8", errors="replace") as logfile:
        return parse_rx_overrun_metrics(logfile)


class State:
    def __init__(self):
        self.connected = False
        self.cmd_connected = False
        self.rsp_connected = False
        self.disconnected = False
        self.rsp_nreceived = 0
        self.cmd_nreceived = 0
        self.breaks = 0
        # Per-side TARGET configs (keyed on N in load_configuration(N), not current=M).
        # configs_seen kept for backward compat = union of both sides.
        self.configs_seen = set()
        self.authfails = 0
        self.enc_activated = False
        # nonce trace: (dir,idx) -> count of ENC emissions; >1 for any key = reuse
        self.enc_nonces = {}
        self.dec_ok = 0
        self.nonce_reuse = 0
        self.rsp_configs_seen = set()
        self.cmd_configs_seen = set()
        self.rsp_cur_cfg = None
        self.cmd_cur_cfg = None
        # Durable cross (RSP side): WB id loaded + WB DATA frames decoded there.
        self.rsp_wb_data_frames = 0      # RX-DATA/BATCH-SEQ DATA while RSP on a WB id
        self.rsp_loaded_wb_id = None     # first WB id (0..16) the RSP load_configuration'd
        self.rsp_max_wb_cfg_with_data = None  # highest WB id an RSP DATA frame decoded on
        self.time_to_cross_s = None      # T+ of first RSP WB DATA decode (durable cross)
        self.precook_done = {}
        self.warm_at = {}
        self.lock = threading.Lock()


def observe_warm_line(st, label, text, event_at):
    """Apply one real modem log line to the two-stage warm readiness gate."""
    with st.lock:
        if PRECOOK_DONE_RE.search(text):
            st.precook_done[label] = True
        if (IDLE_RE.search(text) and st.precook_done.get(label)
                and label not in st.warm_at):
            st.warm_at[label] = event_at


def log_output(proc, label, logfile, t0, st, integrity=None, stop=None):
    try:
        for line in iter(proc.stdout.readline, b''):
            text = line.decode("utf-8", "replace").rstrip()
            event_at = time.time() - t0
            logfile.write(f"[T+{event_at:08.3f}] [{label}] {text}\n")
            logfile.flush()
            observe_warm_line(st, label, text, event_at)
            if CONNECT_RE.search(text):
                with st.lock:
                    st.connected = True
                    if label == "CMD":
                        st.cmd_connected = True
                    else:
                        st.rsp_connected = True
            if DISC_RE.search(text):
                st.disconnected = True
                # Session boundary: realign the RX byte oracle to application
                # offset 0 for the next ARQ session (the responder re-delivers
                # the transfer from its beginning).  Gate on the RSP peer (the
                # side whose data socket feeds the oracle) and skip teardown
                # disconnects raised after the observation window closed.
                if (integrity is not None and label == "RSP"
                        and (stop is None or not stop.is_set())):
                    integrity.note_disconnect()
            m = NRECV_RE.search(text)
            if m:
                v = int(m.group(1))
                with st.lock:
                    if label == "RSP":
                        st.rsp_nreceived = max(st.rsp_nreceived, v)
                    else:
                        st.cmd_nreceived = max(st.cmd_nreceived, v)
            m = CFG_RE.search(text)
            if m:
                target = int(m.group(1))   # the config being LOADED (becomes active)
                with st.lock:
                    st.configs_seen.add(target)
                    if label == "RSP":
                        st.rsp_configs_seen.add(target)
                        st.rsp_cur_cfg = target
                        if is_wb_config(target) and st.rsp_loaded_wb_id is None:
                            st.rsp_loaded_wb_id = target
                    else:
                        st.cmd_configs_seen.add(target)
                        st.cmd_cur_cfg = target
            # Durable cross: an RSP DATA frame decoded while RSP active on a WB id.
            if label == "RSP" and RXDATA_RE.search(text):
                with st.lock:
                    if is_wb_config(st.rsp_cur_cfg):
                        st.rsp_wb_data_frames += 1
                        if st.time_to_cross_s is None:
                            st.time_to_cross_s = round(time.time() - t0, 3)
                        if (st.rsp_max_wb_cfg_with_data is None
                                or st.rsp_cur_cfg > st.rsp_max_wb_cfg_with_data):
                            st.rsp_max_wb_cfg_with_data = st.rsp_cur_cfg
            if BREAK_RE.search(text):
                with st.lock:
                    st.breaks += 1
            if AUTHFAIL_RE.search(text):
                with st.lock:
                    st.authfails += 1
            if ENC_ACT_RE.search(text):
                with st.lock:
                    st.enc_activated = True
            m = NONCE_ENC_RE.search(text)
            if m:
                # (direction, unwrapped index) — per-peer the (key,nonce) pair must
                # be unique. A second ENC of the same (dir,idx) within one peer's
                # log = nonce reuse with (potentially) different plaintext.
                key = (label, int(m.group(1)), int(m.group(2)))
                with st.lock:
                    st.enc_nonces[key] = st.enc_nonces.get(key, 0) + 1
                    if st.enc_nonces[key] > 1:
                        st.nonce_reuse += 1
            if NONCE_DECOK_RE.search(text):
                with st.lock:
                    st.dec_ok += 1
            m = CRYPTO_TX_RE.search(text)
            if m:
                # Production AEAD seal: (peer, direction, unwrapped index). A
                # second [CRYPTO-TX] of the same (dir,idx) within ONE peer's log
                # is a nonce reuse (same key+nonce). The re-seal generation fold
                # keeps a legitimate config-transition re-seal at a DISTINCT index,
                # so a healthy run never double-counts a (dir,idx).
                enc_idx = int(m.group(1)); enc_dir = int(m.group(2))
                key = (label, enc_dir, enc_idx)
                with st.lock:
                    st.enc_nonces[key] = st.enc_nonces.get(key, 0) + 1
                    if st.enc_nonces[key] > 1:
                        st.nonce_reuse += 1
            if CRYPTO_RX_OK_RE.search(text):
                with st.lock:
                    st.dec_ok += 1
    except (ValueError, OSError):
        pass


def tcp_send(port, commands, retries=40, delay=1.0, before_send=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    for attempt in range(retries):
        try:
            sock.connect(("127.0.0.1", port))
            break
        except (ConnectionRefusedError, OSError):
            if attempt == retries - 1:
                raise
            time.sleep(delay)
            sock.close()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
    for c in commands:
        if before_send is not None:
            before_send(c)
        sock.sendall(c.encode())
        time.sleep(0.3)
    return sock


def tx_thread_fn(sock, stop, res, payload_total, traffic=TRAFFIC_LEGACY):
    sock.settimeout(30)
    while not stop.is_set() and res["tx"] < payload_total:
        if traffic == TRAFFIC_LEGACY:
            # Preserve the committed default path byte-for-byte, including its
            # whole-chunk payload overshoot.
            chunk = _LEGACY_CHUNK
        else:
            chunk = traffic_slice(
                traffic, res["tx"], min(len(_LEGACY_CHUNK), payload_total - res["tx"]))
        try:
            sock.sendall(chunk)
            res["tx"] += len(chunk)
        except (socket.timeout, ConnectionError, OSError):
            break
        time.sleep(0.03)


def rx_thread_fn(sock, stop, res, integrity=None):
    sock.settimeout(2)
    while not stop.is_set():
        try:
            d = sock.recv(8192)
            if not d:
                break
            res["rx"] += len(d)
            if integrity is not None:
                integrity.feed(d)
        except socket.timeout:
            continue
        except (ConnectionError, OSError):
            break


def dev(card, devno, sub):
    return f"hw:{card},{devno},{sub}"


def build_arg_parser():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default="/home/kameron/raspeed/mercury")
    ap.add_argument("--bridge", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "realaudio_bridge_s32.py"))
    ap.add_argument("--start-cfg", type=int, default=100)
    ap.add_argument("--no-gearshift", action="store_true",
                    help="omit -g so --start-cfg HOLDS (pins; adds -Q 0 to start direct-WB)")
    ap.add_argument("--secs", type=int, default=120)
    ap.add_argument("--payload", type=int, default=4096)
    ap.add_argument(
        "--score-horizon-s", type=float, default=None,
        help="fixed CONNECT-request observation/score horizon; when set, the "
             "cell does not exit at payload delivery; when omitted, --secs "
             "retains the committed legacy behavior")
    ap.add_argument(
        "--warm-start", action=argparse.BooleanOptionalAction, default=False,
        help="wait for both modem PRECOOK+Idle readiness markers before CONNECT; "
             "the default remains the committed cold-start behavior")
    ap.add_argument("--warm-timeout", type=float, default=120.0)
    ap.add_argument("--passthrough", action="store_true")
    ap.add_argument("--axis", choices=AXIS_CHOICES, default=DEFAULT_AXIS,
                    help="versioned channel axis; campaign default remains v1")
    ap.add_argument("--snr", type=float, default=None,
                    help="historical ambiguous dial; v0 only")
    ap.add_argument("--snr3k-db", type=float, default=None)
    ap.add_argument(
        "--snr3k", type=float, default=None,
        help="DEPRECATED controlling alias for --snr3k-db")
    ap.add_argument("--cn-config-db", type=float, default=None)
    ap.add_argument("--configured-bandwidth-hz", type=float, default=None)
    ap.add_argument("--band-family", choices=("WB", "NB"), default=None)
    ap.add_argument("--calibration-registry", default=None)
    ap.add_argument("--reference-id", default=None)
    ap.add_argument("--reference-power", type=float, default=None)
    ap.add_argument("--reference-class", default=None)
    ap.add_argument("--tx-gain-overrides", default="none")
    ap.add_argument("--sample-rate-hz", type=int, default=48000)
    ap.add_argument("--headroom-n-samples", type=int, default=None)
    ap.add_argument("--headroom-epsilon", type=float, default=None)
    ap.add_argument("--s32-mode",
                    choices=("auto", "prescaled", "s32-hardclip"),
                    default="auto")
    ap.add_argument("--traffic", choices=TRAFFIC_CHOICES, default=TRAFFIC_LEGACY,
                    help="legacy committed ramp or deterministic incompressible bytes")
    ap.add_argument("--cell", default=None)
    ap.add_argument("--profile", default="wgn")
    ap.add_argument("--cfo-hz", type=float, default=0.0)
    ap.add_argument("--phase-noise-deg", type=float, default=0.0)
    ap.add_argument("--fade-depth-db", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--cap-periods", type=int, default=3)
    ap.add_argument("--play-periods", type=int, default=4)
    ap.add_argument("--prime-periods", type=int, default=2)
    ap.add_argument("--tag", default="cell")
    ap.add_argument("--arm", default="legacy",
                    help="A/B arm label (e.g. redesign|legacy); recorded in JSON")
    ap.add_argument("--realized-width", type=int, default=None,
                    help="concurrent live real-audio cells on this box during "
                         "this cell; recorded verbatim for wide-window evidence")
    ap.add_argument("--env", action="append", default=[],
                    help="KEY=VAL env var injected into BOTH mercury instances "
                         "(repeatable); used to carry the redesign env trio")
    ap.add_argument("--logdir", default="/tmp/raionos2/logs")
    ap.add_argument("--json", default=None)
    # Concurrency knobs: distinct card + 4 disjoint substreams + distinct ports.
    ap.add_argument("--card", default="Loopback",
                    help="ALSA snd-aloop card name or index")
    ap.add_argument("--subs", default="0,1,2,3",
                    help="4 substream indices S0,S1,S2,S3 (CMD-tx, RSP-rx, RSP-tx, CMD-rx-impaired)")
    ap.add_argument("--rsp-port", type=int, default=7002)
    ap.add_argument("--cmd-port", type=int, default=7006)
    ap.add_argument("--no-kill", action="store_true",
                    help="do NOT global-pkill mercury/bridge on start (REQUIRED for concurrent runs)")
    ap.add_argument("--encrypt", default=None,
                    help="encryption mode passed to BOTH mercury via -E (e.g. 'fast' or 'strict'); "
                         "omit for plaintext")
    ap.add_argument("--psk", default=None,
                    help="pre-shared key hex passed to BOTH mercury via -K (required with --encrypt)")
    return ap


def main(argv=None):
    ap = build_arg_parser()
    args = ap.parse_args(argv)
    if args.score_horizon_s is not None and args.score_horizon_s <= 0:
        ap.error("--score-horizon-s must be positive")
    if args.warm_timeout <= 0:
        ap.error("--warm-timeout must be positive")
    try:
        bridge_prefix = bridge_command_prefix(args.bridge)
    except BridgeCommandError as exc:
        ap.error(str(exc))

    configured_bandwidth_hz = (
        args.configured_bandwidth_hz
        if args.configured_bandwidth_hz is not None
        else infer_bandwidth(args.start_cfg, args.band_family))
    try:
        resolved_axis = resolve_axis(
            args.axis, snr=args.snr, snr3k=args.snr3k,
            snr3k_db=args.snr3k_db, cn_config_db=args.cn_config_db,
            cell_snr3k_db=(parse_cell(args.cell) if args.cell else None),
            configured_bandwidth_hz=configured_bandwidth_hz,
            default_snr3k_db=30.0)
    except (AxisError, ValueError) as exc:
        ap.error(str(exc))

    try:
        binary_sha256 = sha256_file(args.bin)
    except OSError as exc:
        ap.error("cannot hash --bin: %s" % exc)

    calibration = None
    if args.axis == AXIS_V2:
        missing = [name for name, value in (
            ("--calibration-registry", args.calibration_registry),
            ("--reference-id", args.reference_id),
            ("--reference-power", args.reference_power),
            ("--reference-class", args.reference_class),
            ("--band-family", args.band_family),
            ("--headroom-n-samples", args.headroom_n_samples),
            ("--headroom-epsilon", args.headroom_epsilon),
        ) if value is None]
        if missing:
            ap.error("v2 preflight requires " + ", ".join(missing))
        key = calibration_key(
            binary_sha256=binary_sha256, config=args.start_cfg,
            band_family=args.band_family,
            tx_gain_overrides=args.tx_gain_overrides,
            sample_rate_hz=args.sample_rate_hz,
            waveform_class=args.reference_class)
        try:
            calibration = load_and_select_calibration(
                args.calibration_registry, key, args.reference_id)
        except (AxisError, OSError, ValueError) as exc:
            ap.error("v2 calibration preflight failed: %s" % exc)
        if not math.isclose(args.reference_power,
                            float(calibration["reference_power"]),
                            rel_tol=0.0, abs_tol=1e-15):
            ap.error("--reference-power does not match the registry record")
        if not math.isclose(configured_bandwidth_hz,
                            float(calibration["configured_bandwidth_hz"]),
                            rel_tol=0.0, abs_tol=1e-12):
            ap.error("--configured-bandwidth-hz does not match calibration")

    recipe_sha256 = sha256_json({
        "axis": resolved_axis, "start_cfg": args.start_cfg,
        "reference_id": args.reference_id,
        "reference_power": args.reference_power,
        "reference_class": args.reference_class,
        "tx_gain_overrides": args.tx_gain_overrides,
        "sample_rate_hz": args.sample_rate_hz,
        "profile": args.profile, "cfo_hz": args.cfo_hz,
        "phase_noise_deg": args.phase_noise_deg,
        "seed": args.seed, "s32_mode": args.s32_mode,
        "headroom_n_samples": args.headroom_n_samples,
        "headroom_epsilon": args.headroom_epsilon,
        "passthrough": args.passthrough,
        "env": sorted(args.env),
    })

    os.makedirs(args.logdir, exist_ok=True)
    # Per-cell env injection: each mercury instance inherits the parent env PLUS
    # the KEY=VAL pairs passed via --env. This is how the redesign arm carries
    # MERCURY_INBAND_RATE / MERCURY_CUMULATIVE_ACK / MERCURY_INBAND_A3_DECOUPLE
    # while the legacy arm leaves them unset (empty --env list).
    cell_env = dict(os.environ)
    for kv in args.env:
        if "=" in kv:
            k, v = kv.split("=", 1)
            cell_env[k] = v
    use_robust = args.start_cfg >= 100
    rsp_port = args.rsp_port
    cmd_port = args.cmd_port
    subs = [int(x) for x in args.subs.split(",")]
    assert len(subs) == 4, "need exactly 4 substream indices"
    S0, S1, S2, S3 = subs

    # cable wiring (see module docstring)
    cmd_tx = dev(args.card, 0, S0)   # commander plays into FWD cap
    cmd_rx = dev(args.card, 1, S3)   # commander reads REV-impaired output
    rsp_rx = dev(args.card, 1, S1)   # responder reads FWD-impaired output
    rsp_tx = dev(args.card, 0, S2)   # responder plays into REV cap
    fwd_cap = dev(args.card, 1, S0)
    fwd_play = dev(args.card, 0, S1)
    rev_cap = dev(args.card, 1, S2)
    rev_play = dev(args.card, 0, S3)

    if not args.no_kill:
        # CONCURRENCY-SAFE cleanup: kill ONLY the mercury / bridge processes
        # that own THIS run's ALSA cables (this card + these 4 substreams) or
        # THIS run's TCP ports. A sibling agent's run on a DISJOINT
        # card/subs/port set can never match, so we never reap it. The old
        # global `pkill -9 -f 'mercury -m ARQ'` reaped every concurrent
        # agent's mercury (diagnostic wf_18b9f890). See ra_cleanup.py.
        scoped_cleanup(args.card, subs, [rsp_port, cmd_port], settle=1.5)

    logpath = os.path.join(args.logdir, f"arq_{args.tag}.log")
    logfile = open(logpath, "w")
    st = State()
    procs, sockets = [], []
    stop = threading.Event()
    t0 = time.time()
    bridge = None
    bridge_stats_path = os.path.join(
        args.logdir, f"bridge_{args.tag}_stats.json")
    res = {"tx": 0, "rx": 0}
    integrity = ByteIntegrityTracker(args.traffic)
    connected_at = None
    connected_at_cold = None
    connect_issued_at = None
    warm_offset = 0.0
    warm_snapshot = {}
    warm_start_ok = not args.warm_start
    score_horizon_reached = False

    def mercury_cmd(port, in_dev, out_dev):
        c = [args.bin, "-m", "ARQ", "-s", str(args.start_cfg), "-W",
             "-p", str(port), "-x", "alsa", "-i", in_dev, "-o", out_dev,
             "-n", "-F", "off"]
        if not args.no_gearshift:
            c += ["-g"]
        else:
            c += ["-Q", "0"]      # pin: start direct-WB at start-cfg
        if use_robust:
            c += ["-R"]
        # Encryption: -E <mode> + matching -K <psk-hex> on BOTH peers. The PSK is
        # the shared secret both modems use to derive the X25519 session key; it
        # MUST be identical on commander and responder or KX confirmation fails.
        if args.encrypt:
            c += ["-E", args.encrypt]
            if args.psk:
                c += ["-K", args.psk]
        return c

    def launch(port, in_dev, out_dev, label):
        p = subprocess.Popen(mercury_cmd(port, in_dev, out_dev),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             env=cell_env)
        threading.Thread(target=log_output,
                         args=(p, label, logfile, t0, st, integrity, stop),
                         daemon=True).start()
        return p

    try:
        # 1. bridge first (opens the 4 loopback subdevices for THIS run).
        # A .py bridge retains the Python interpreter. ELF binaries and other
        # executable non-.py bridges execute directly; the imported bridge's
        # proven fleet tune is SCHED_OTHER nice -5 (the ring depths are below).
        bcmd = bridge_prefix + [
                "--fwd-cap", fwd_cap, "--fwd-play", fwd_play,
                "--rev-cap", rev_cap, "--rev-play", rev_play,
                "--axis", args.axis,
                "--configured-bandwidth-hz", str(configured_bandwidth_hz),
                "--profile", args.profile,
                "--cfo-hz", str(args.cfo_hz),
                "--phase-noise-deg", str(args.phase_noise_deg),
                "--fade-depth-db", str(args.fade_depth_db),
                "--seed", str(args.seed),
                "--cap-periods", str(args.cap_periods),
                "--play-periods", str(args.play_periods),
                "--prime-periods", str(args.prime_periods),
                "--binary-sha256", binary_sha256,
                "--recipe-sha256", recipe_sha256,
                "--s32-mode", args.s32_mode,
                "--statsfile", bridge_stats_path]
        input_coordinate = resolved_axis["input_coordinate"]
        if input_coordinate == "snr":
            bcmd += ["--snr", str(resolved_axis["snr3k_db"])]
        elif input_coordinate == "snr3k":
            bcmd += ["--snr3k", str(resolved_axis["snr3k_db"])]
        elif input_coordinate == "snr3k_db":
            bcmd += ["--snr3k-db", str(resolved_axis["snr3k_db"])]
        elif input_coordinate == "cn_config_db":
            bcmd += ["--cn-config-db", str(resolved_axis["cn_config_db"])]
        elif input_coordinate == "cell":
            bcmd += ["--cell", args.cell]
        if calibration is not None:
            bcmd += [
                "--reference-power", str(calibration["reference_power"]),
                "--reference-id", calibration["reference_id"],
                "--reference-n-samples",
                str(calibration["reference_n_samples"]),
                "--calibration-peak", str(calibration["calibration_peak"]),
                "--headroom-n-samples", str(args.headroom_n_samples),
                "--headroom-epsilon", str(args.headroom_epsilon),
            ]
        if args.passthrough:
            bcmd += ["--passthrough"]
        blog = open(os.path.join(args.logdir, f"bridge_{args.tag}.log"), "wb")
        bridge = subprocess.Popen(bcmd, stdout=blog, stderr=blog)
        time.sleep(2.0)

        # 2. responder then commander (raw hw: device strings, no plug).
        rsp = launch(rsp_port, rsp_rx, rsp_tx, "RSP")
        procs.append(rsp)
        time.sleep(3)
        cmd = launch(cmd_port, cmd_rx, cmd_tx, "CMD")
        procs.append(cmd)
        time.sleep(3)

        # Proven two-stage warm gate: both peer processes must report PRECOOK
        # completion and then a running-loop Idle. No fixed warmup sleep can
        # substitute for these process-owned readiness markers.
        if args.warm_start:
            warm_deadline = time.time() + args.warm_timeout
            while time.time() < warm_deadline and len(st.warm_at) < 2:
                if any(p.poll() is not None for p in procs):
                    break
                time.sleep(0.25)
            with st.lock:
                warm_snapshot = dict(st.warm_at)
            warm_start_ok = len(warm_snapshot) == 2
            if warm_start_ok:
                warm_offset = max(warm_snapshot.values())
            logfile.write(
                f"[T+{time.time()-t0:08.3f}] [WARM-START] "
                f"ready={warm_start_ok} warm_at={warm_snapshot} "
                f"offset={warm_offset:.3f}s\n")
            logfile.flush()

        # 3. control + data sockets (identical to canonical sim_arq_channel.py).
        rsp_ctrl = tcp_send(rsp_port, ["MYCALL TESTB\r\n", "LISTEN ON\r\n"])
        sockets.append(rsp_ctrl)
        time.sleep(1)

        cmd_data = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cmd_data.settimeout(5)
        cmd_data.connect(("127.0.0.1", cmd_port + 1))
        sockets.append(cmd_data)
        rsp_data = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rsp_data.settimeout(5)
        rsp_data.connect(("127.0.0.1", rsp_port + 1))
        sockets.append(rsp_data)

        threading.Thread(target=rx_thread_fn,
                         args=(rsp_data, stop, res, integrity),
                         daemon=True).start()
        time.sleep(0.5)

        def stamp_connect_request(command):
            nonlocal connect_issued_at
            if command.lstrip().upper().startswith("CONNECT "):
                connect_issued_at = time.time() - t0

        cmd_ctrl = tcp_send(
            cmd_port, ["MYCALL TESTA\r\n", "CONNECT TESTA TESTB\r\n"],
            before_send=stamp_connect_request)
        sockets.append(cmd_ctrl)

        connect_wait = (args.score_horizon_s
                        if args.score_horizon_s is not None else args.secs)
        connect_deadline = time.time() + min(connect_wait, 90)
        while time.time() < connect_deadline and not st.connected:
            time.sleep(0.5)
        connected_at_cold = (time.time() - t0) if st.connected else None
        connected_at = (connected_at_cold - warm_offset
                        if connected_at_cold is not None else None)

        tx_started = False
        if st.connected:
            threading.Thread(target=tx_thread_fn,
                             args=(cmd_data, stop, res, args.payload, args.traffic),
                             daemon=True).start()
            tx_started = True

        start = time.time()
        score_deadline = (
            fixed_score_deadline(
                t0, connect_issued_at, args.score_horizon_s,
                connected_at_cold)
            if args.score_horizon_s is not None and connect_issued_at is not None
            else None)
        dead = False
        while observation_active(
                time.time(), start, args.secs, score_deadline) and not dead:
            time.sleep(1)
            if score_deadline is not None and st.connected and not tx_started:
                connected_at_cold = time.time() - t0
                connected_at = connected_at_cold - warm_offset
                score_deadline = fixed_score_deadline(
                    t0, connect_issued_at, args.score_horizon_s,
                    connected_at_cold)
                threading.Thread(
                    target=tx_thread_fn,
                    args=(cmd_data, stop, res, args.payload, args.traffic),
                    daemon=True).start()
                tx_started = True
            if delivery_ends_observation(
                    score_deadline, st.connected, res["rx"], args.payload):
                break
            for p in procs:
                if p.poll() is not None:
                    dead = True
        if score_deadline is not None:
            score_horizon_reached = time.time() >= score_deadline
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"[harness] {e}\n")
    finally:
        stop.set()
        for s in sockets:
            try:
                s.close()
            except OSError:
                pass
        for p in procs:
            try:
                p.terminate()
            except OSError:
                pass
        terminate_deadline = time.time() + 10.0
        for p in procs:
            remaining = max(0.0, terminate_deadline - time.time())
            try:
                p.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                sys.stderr.write(
                    "[harness] process did not exit after SIGTERM grace; "
                    "leaving it for scoped recovery: pid=%s\n" % p.pid)
        if bridge:
            try:
                bridge.terminate()
                bridge.wait(timeout=10)
            except subprocess.TimeoutExpired:
                sys.stderr.write(
                    "[harness] bridge did not exit after SIGTERM grace; "
                    "not sending SIGKILL: pid=%s\n" % bridge.pid)
            except OSError:
                pass
        logfile.close()

    dwell_cold = max(1.0, time.time() - t0)
    dwell = max(1.0, dwell_cold - warm_offset)
    integrity_result = integrity.snapshot()
    # Forensic artifacts land under logdir (captured by the certification
    # driver's wholesale run-dir tar) and are written ONLY on a content
    # violation; a clean cell's durable byte-identity proof is the
    # delivered/expected md5 pair recorded in the result JSON.  The round-5
    # corrupt cell was un-auditable because these were never persisted.
    integrity.dump_forensics(
        os.path.join(args.logdir, f"capbytes_{args.tag}.txt"),
        os.path.join(args.logdir, f"rx_{args.tag}.bin"))
    # HONEST COMPLETION: count-only rx>=payload silently passes a multi-session
    # cell that only reaches the target by RE-DELIVERING a prefix across
    # sessions.  Require a single session to have delivered the payload AND the
    # session-aware oracle to be clean.  The legacy count-only value survives
    # under its own name so old and new cells stay distinguishable in audits.
    delivered_full_count_only = res["rx"] >= args.payload
    delivered_full_single_session = (
        integrity_result["max_session_bytes"] >= args.payload)
    delivered_full = bool(
        delivered_full_single_session and integrity_result["byte_integrity_ok"])
    content_shear_suspected = bool(
        delivered_full_count_only and not integrity_result["byte_integrity_ok"])
    try:
        rx_overrun_total, rx_overrun_segments = read_rx_overrun_metrics(logpath)
    except (OSError, ValueError) as e:
        sys.stderr.write(f"[harness] FATAL: RX overrun metric rejected: {e}\n")
        sys.stderr.flush()
        return 2
    if rx_overrun_total is None:
        sys.stderr.write(
            f"[harness] INVALID METRIC: no {RX_OVERRUN_MARKER} lines in "
            f"{logpath}; rx_overrun_total=null\n")
        sys.stderr.flush()
    configs_sorted = sorted(st.configs_seen)
    rsp_configs_sorted = sorted(st.rsp_configs_seen)
    cmd_configs_sorted = sorted(st.cmd_configs_seen)
    # wb_configs_seen is keyed on the TARGET config of load_configuration(N) and is
    # RSP-SIDE ONLY. The CMD adopts WB unilaterally on a (false) bare ACK
    # (arq_commander.cc:2569/7196-7208) and would false-positive a cross; the
    # durable cross is an RSP that actually LOADS + DECODES WB. WB OFDM = ids 0..16.
    wb_seen = [c for c in rsp_configs_sorted if is_wb_config(c)]
    max_config = max(configs_sorted) if configs_sorted else None
    # DURABLE CROSS = RSP loaded a WB id AND decoded >= 1 WB-OFDM DATA frame there
    # AND bytes were delivered. This is the only crossing signal not fooled by the
    # INDIRECT V2-PREV-DELIVERED ROBUST drain or the CMD's unilateral adoption.
    crossed = ((st.rsp_loaded_wb_id is not None)
               and (st.rsp_wb_data_frames >= 1)
               and (res["rx"] > 0))
    # climbed_past_cfg0: the BINDING-CONSTRAINT metric -- did the RSP decode a DATA
    # frame on a WB id >= 1 (i.e. climb THROUGH cfg0)? cfg0 is the lowest WB rung.
    climbed_past_cfg0 = (st.rsp_max_wb_cfg_with_data is not None
                         and st.rsp_max_wb_cfg_with_data >= 1)
    # legacy compat boolean (kept; now derived from the corrected RSP-side WB set).
    climbed_past_robust0 = bool(wb_seen) or any(c > 100 for c in rsp_configs_sorted)
    try:
        with open(bridge_stats_path, "r", encoding="utf-8") as stream:
            bridge_stats = json.load(stream)
        axis_attestation = validate_attestation(
            bridge_stats["axis_attestation"])
    except (OSError, KeyError, ValueError, TypeError, AxisError) as exc:
        sys.stderr.write("[harness] FATAL bridge attestation rejected: %s\n" % exc)
        return 2

    result = {
        "tag": args.tag, "arm": args.arm,
        "env": redacted_env_items(args.env),
        "traffic": args.traffic,
        "passthrough": args.passthrough,
        "snr": (axis_attestation["snr3k_db"]
                if axis_attestation["axis_version"] == AXIS_V0 else None),
        "snr3k": axis_attestation["snr3k_db"],
        "cell": args.cell, "profile": args.profile,
        "seed": args.seed, "start_cfg": args.start_cfg,
        "card": args.card, "subs": subs,
        "rsp_port": rsp_port, "cmd_port": cmd_port,
        "no_gearshift": args.no_gearshift,
        "connected": st.connected,
        "cmd_connected": st.cmd_connected, "rsp_connected": st.rsp_connected,
        "connected_at_s": round(connected_at, 2) if connected_at else None,
        "connected_at_cold_s": (
            round(connected_at_cold, 2) if connected_at_cold else None),
        "warm_start": args.warm_start,
        "warm_start_ok": warm_start_ok,
        "warm_offset_s": round(warm_offset, 2),
        "warm_at_s": {key: round(value, 2)
                      for key, value in sorted(warm_snapshot.items())},
        "tx_bytes": res["tx"], "rx_bytes": res["rx"],
        "payload_target": args.payload,
        "realized_width": args.realized_width,
        "delivered_full": delivered_full,
        "delivered_full_count_only": delivered_full_count_only,
        "delivered_full_single_session": delivered_full_single_session,
        "content_shear_suspected": content_shear_suspected,
        "rsp_nreceived_frames": st.rsp_nreceived,
        "cmd_nreceived_frames": st.cmd_nreceived,
        "breaks": st.breaks,
        "byte_integrity_ok": integrity_result["byte_integrity_ok"],
        "integrity_mismatch_bytes": integrity_result["integrity_mismatch_bytes"],
        "integrity_mismatch_segments": integrity_result["integrity_mismatch_segments"],
        "integrity_first_bad_offset": integrity_result["integrity_first_bad_offset"],
        "integrity_disconnects": integrity_result["integrity_disconnects"],
        "integrity_sessions": integrity_result["integrity_sessions"],
        "max_session_bytes": integrity_result["max_session_bytes"],
        "total_received_bytes": integrity_result["total_received_bytes"],
        "content_delivered_md5": integrity_result["content_delivered_md5"],
        "content_expected_md5": integrity_result["content_expected_md5"],
        "content_md5_ok": integrity_result["content_md5_ok"],
        "rx_overrun_total": rx_overrun_total,
        "rx_overrun_segments": rx_overrun_segments,
        "encrypt": args.encrypt,
        "enc_activated": st.enc_activated,
        "aead_authfails": st.authfails,
        "nonce_enc_count": sum(st.enc_nonces.values()),
        "nonce_reuse_count": st.nonce_reuse,
        "dec_ok_count": st.dec_ok,
        "configs_seen": configs_sorted,
        "rsp_configs_seen": rsp_configs_sorted,
        "cmd_configs_seen": cmd_configs_sorted,
        "max_config_reached": max_config,
        "wb_configs_seen": wb_seen,
        "climbed_past_robust0": climbed_past_robust0,
        # --- durable ROBUST->WB crossing metrics (RSP-side, target-keyed) ---
        "rsp_loaded_wb_id": st.rsp_loaded_wb_id,
        "rsp_wb_data_frames": st.rsp_wb_data_frames,
        "max_wb_cfg_with_data": st.rsp_max_wb_cfg_with_data,
        "time_to_cross_s": st.time_to_cross_s,
        "crossed": crossed,
        "climbed_past_cfg0": climbed_past_cfg0,
        "score_horizon_s": args.score_horizon_s,
        "score_horizon_reached": score_horizon_reached,
        "score_rx_Bmin": (
            round(res["rx"] * 60.0 / args.score_horizon_s, 1)
            if args.score_horizon_s is not None and score_horizon_reached
            else None),
        "rx_bps_wall": round(res["rx"] * 8 / dwell, 1),
        "wall_secs": round(dwell, 1),
        "wall_secs_cold": round(dwell_cold, 1),
    }
    result.update(axis_attestation)
    print(json.dumps(result))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())

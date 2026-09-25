#!/usr/bin/env python3
"""
arq_realaudio.py - drive TWO stock `-x alsa` Mercury instances through the
real-audio IONOS bridge (realaudio_bridge_s32_c) over snd-aloop.

Mirrors tools/sim_arq_channel.py EXACTLY (same ctrl/data TCP protocol: ctrl on
PORT, data on PORT+1, MYCALL/LISTEN/CONNECT, tx=traffic-class source chunks
[Winlink corpus for compressible / keyed-random for incompressible, see
capstone_arms.traffic_slice], rx=recv) - the ONLY change vs the canonical sim
harness is the audio transport:
-x sim TCP relay  ->  -x alsa real audio through the bridge.

This is the production copy of the prototype proven in wf_1e5c12f1.

===  CAPSTONE INTEGRATION EXTENSIONS  (measurement infra only; NO mercury source
     touched) - see capstone_arms.py for the shared registry/anatomy/VARA logic:
  (a) PER-LEVER KILL-SWITCHES: --arm-name PRESET or --arm-spec '{"inband_rate":1}'
      resolves to env toggles applied to BOTH mercury via the existing --env
      path. ALL-OFF injects nothing => reproduces the stock monitor baseline.
  (b) BYTE-INTEGRITY (MANDATORY): the RX side byte-compares every
      delivered segment against the sent deterministic pattern; ANY mismatch
      HARD-FAILS the cell (verdict INTEGRITY_FAIL + loud banner).
  (c) ANATOMY LINE-ITEMS (PRIMARY scoreboard): listen-guard ms, retx rounds,
      demote/BREAK counts, active_fraction, delivered USER-bytes/min - wired from
      the run_inband_ab.py anatomy profiler (compute_inburst) + marker greps.
  (d) VARA-BAR column: the documented client-effective VARA B/min at this SNR3k.

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
import os
import re
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ra_cleanup import scoped_cleanup, terminate_popen_processes
from channel_attestation import attested_snr3k
import bridge_reference
import harness_attestation as HA

HARNESS_LINEAGE = "mercury"
import capstone_arms as CA             # arm-spec + anatomy + VARA (capstone ext)
from raw_evidence import RawEvidence   # per-attempt delivered/sent byte retention
import override_guard                  # registered behaviour-restoring switches

# Set by main(): retains the exact delivered and sent bytes of this attempt on disk
# as they happen, so failed and timed-out attempts keep their raw evidence.
_RAW_EVIDENCE = None


def _raw_rx(data):
    if _RAW_EVIDENCE is not None:
        _RAW_EVIDENCE.rx(data)


def _raw_tx(data):
    if _RAW_EVIDENCE is not None:
        _RAW_EVIDENCE.tx(data)
from canonical_window_scorer import (
    DeliveryOracle,
    LinkEventTracker,
    score_fixed_windows,
)
from warm_gate import (
    DeliveredRateWarmGate,
    WARM_CEILING_S,
    WARM_FLOOR_S,
    WARM_MAX_TX_AHEAD_BYTES,
    WARM_RATE_TOLERANCE,
    WARM_RATE_WINDOW_S,
    WARM_SAMPLE_S,
    WARM_STABLE_SAMPLES,
    scored_delta,
    warm_gate_constants,
)
from eot_log_adapter import apply_terminal_eot_gate

CONNECT_RE = re.compile(r"^link_status:Connected to(?:\s|$)")
DISC_RE = re.compile(
    r"^(?:link_status:Disconnected(?:\s|$)|DISCONNECTED(?:\s|$))")
# Mercury's protocol-level release acknowledgement is written to each control
# socket as ``DISCONNECTED\r``.  It is not normally printed on stdout (the next
# stdout state is ``link_status:Idle``/``Listening``), so stdout alone cannot
# prove that both peers released the channel.
CONTROL_DISCONNECTED = "DISCONNECTED"
TERMINAL_SETTLEMENT_GRACE_S = 30.0
NRECV_RE = re.compile(r"stats\.nReceived_data=\s*(\d+)")
TOSEND_RE = re.compile(r"stats\.ToSend_data:\s*(\d+)")
# group(1) is the TARGET being loaded; group(2) is the OUTGOING config. Keying on
# group(2) meant the newest-adopted config was NEVER recorded (the run ends on it),
# so configs_seen under-reported the climb and max_config_reached was wrong.
# current= can be -1 at first load, so the sign must be allowed.
CFG_RE = re.compile(r"load_configuration\((\d+)\)\s+current=(-?\d+)")
BREAK_RE = re.compile(r"\[BREAK\] Block failure")
# AEAD auth-fail markers: the env-gated nonce trace (DEC-AUTHFAIL) AND any
# generic crypto auth-failure / PSK-mismatch the modem already logs.
AUTHFAIL_RE = re.compile(r"NONCE-TRACE\] DEC-AUTHFAIL|auth.?fail|PSK mismatch|decrypt.*fail",
                         re.IGNORECASE)
NONCE_ENC_RE = re.compile(r"NONCE-TRACE\] ENC dir=(\d+) idx=(\d+)")
NONCE_DECOK_RE = re.compile(r"NONCE-TRACE\] DEC-OK dir=(\d+) idx=(\d+)")
ENC_ACT_RE = re.compile(r"\[CRYPTO\] Encryption ACTIVATED")
# WARM-START readiness: the one-time PHY precompute finishes when the modem prints
# the config-bundle-build line (right before it spawns audio + enters its accept
# loop); the FIRST link_status:Idle after that is emitted by the running main loop
# = accept loop ready. Gate on the bundle line first so the pre-precompute Idle
# (printed once at boot) does not falsely mark the modem warm.
PRECOOK_DONE_RE = re.compile(r"\[PRECOOK\] config bundles built")
IDLE_RE = re.compile(r"link_status:Idle")
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

# The channel's P_sig meter latches to the loud connect burst in its legacy
# 'peak' mode and hot-labels the delivered SNR; only 'steady' measures at the
# data-frame power. A cohort that omits the mode env silently re-poisons its
# axis (instrument #16), so the ACTUAL mode the channel ran is stamped into the
# per-cell result as durable evidence, and the scorer refuses to compare an
# unknown/hot axis to the VARA bar.
PSIG_DIAG_MODE_RE = re.compile(r"\[PSIG_DIAG\]\s+mode=(\S+)")
# MUST track sim_channel_relay.py's own MERCURY_SIM_PSIG_MODE default. Reached
# only when NO durable axis evidence exists (no stats key AND no PSIG_DIAG line
# AND the env is unset). The sim channel now defaults to the honest 'steady'
# axis, so an un-pinned cohort is labelled 'steady' here -- matching what the
# channel actually runs. ('peak' is retained only as an explicit opt-in control.)
_SIM_PSIG_DEFAULT_MODE = "steady"


_STARTUP_PHASE_ORDER = {
    "cooking": 0,
    "warmed_idle": 1,
    "active": 2,
    "done": 3,
}


def _publish_startup_phase(path, phase, t0, tag, **details):
    """Atomically expose this cell's startup phase to the fleet admission gate.

    The outer spawner treats a missing file as ``cooking``.  Atomic replacement
    means it can never consume a partially written event.  Phase progression is
    monotonic by construction; callers publish only at the real state edges.
    """
    if not path:
        return
    if phase not in _STARTUP_PHASE_ORDER:
        raise ValueError(f"unknown startup phase: {phase}")
    event = {
        "version": 1,
        "tag": tag,
        "pid": os.getpid(),
        "phase": phase,
        "phase_order": _STARTUP_PHASE_ORDER[phase],
        "elapsed_s": round(max(0.0, time.monotonic() - t0), 6),
        "wall_time_ns": time.time_ns(),
    }
    event.update(details)
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    tmp = f"{path}.tmp.{os.getpid()}"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(event, handle, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def _resolve_psig_mode(bridge_stats_path, bridge_log_path, passthrough):
    """Resolve the ACTUAL noise-calibration (P_sig) axis the sim channel ran.

    Preference (most authoritative first):
      1. the bridge statsfile 'psig_mode' key (durable, if the bridge wrote one);
      2. the [PSIG_DIAG] mode=<x> line the channel prints into the bridge log
         (== what Channel.process actually ran; the last line is the settled
         mode);
      3. the MERCURY_SIM_PSIG_MODE env this process passes on to the bridge
         (default 'steady', matching sim_channel_relay.py's own default).
    Returns (mode, source). A passthrough cell has no channel/noise at all.
    """
    if passthrough:
        return "passthrough", "passthrough"
    # 1. durable statsfile key (future-proof: read it if the bridge writes one)
    try:
        with open(bridge_stats_path) as fh:
            st = json.load(fh)
        m = st.get("psig_mode") if isinstance(st, dict) else None
        if isinstance(m, str) and m.strip():
            return m.strip().lower(), "bridge_stats"
    except (OSError, ValueError):
        pass
    # 2. the PSIG_DIAG line the channel actually printed (authoritative)
    try:
        found = None
        with open(bridge_log_path, "r", errors="replace") as fh:
            for line in fh:
                mm = PSIG_DIAG_MODE_RE.search(line)
                if mm:
                    found = mm.group(1).strip().lower()
        if found:
            return found, "bridge_log_psig_diag"
    except OSError:
        pass
    # 3. env fallback (the bridge inherits this process's environment)
    env = os.environ.get("MERCURY_SIM_PSIG_MODE")
    if env is not None and env.strip():
        return env.strip().lower(), "env"
    return _SIM_PSIG_DEFAULT_MODE, "env_default"


def _read_bridge_underruns(bridge_stats_path):
    """Surface the per-direction snd-aloop underrun counters the bridge
    (realaudio_bridge_s32_c) already records into its statsfile, so a stormed
    cell can be classified clean-audio vs degraded-audio (the load-invariance /
    cohort-width diagnostic; the audio-path integrity signal). Each pump
    direction ('fwd' == CMD->RSP payload, 'rev' == RSP->CMD ACK/return) bumps
    its 'underruns' on a capture-read error (length<0) or a playback XRUN --
    i.e. a dropped or re-primed (silence-injected + time-shifted) audio period.
    Those counts live at the statsfile top level under 'fwd'/'rev' (bridge
    realaudio_bridge_s32_c stats[name]), NOT inside channel_attestation, so the
    existing attested_snr3k read does not carry them.

    Returns {"fwd": int|None, "rev": int|None, "total": int|None,
    "source": "bridge_stats"|None}. Fail-open (None values) when the statsfile
    is absent/unreadable/malformed -- same contract as _resolve_psig_mode; a
    direct single-cell run before the bridge flushes simply reports None."""
    out = {"fwd": None, "rev": None, "total": None, "source": None}
    try:
        with open(bridge_stats_path) as fh:
            st = json.load(fh)
    except (OSError, ValueError):
        return out
    if not isinstance(st, dict):
        return out
    total = 0
    seen = False
    for direction in ("fwd", "rev"):
        dirstats = st.get(direction)
        if isinstance(dirstats, dict):
            u = dirstats.get("underruns")
            if isinstance(u, int) and not isinstance(u, bool):
                out[direction] = u
                total += u
                seen = True
    if seen:
        out["total"] = total
        out["source"] = "bridge_stats"
    return out


class State:
    def __init__(self):
        self.connected = False
        self.cmd_connected = False
        self.rsp_connected = False
        # First parsed Connected event for each peer, on the harness monotonic
        # T+ clock.  The cell is connected only after BOTH peers have emitted it.
        self.connected_at_by_peer = {}
        self.first_connected_at_by_peer = {}
        self.ever_connected = False
        self.disconnected = False
        # First channel-release event from each endpoint.  M1 settlement is the
        # later of these two timestamps, not receiver byte completion.
        self.disconnected_at_by_peer = {}
        self.disconnect_source_by_peer = {}
        self.rsp_nreceived = 0
        self.cmd_nreceived = 0
        # Legibility marker: last CMD queue depth reported by the modem status block.
        self.last_ToSend_data = 0
        self.breaks = 0
        self.configs_seen = set()
        self.authfails = 0
        self.enc_activated = False
        # nonce trace: (dir,idx) -> count of ENC emissions; >1 for any key = reuse
        self.enc_nonces = {}
        self.dec_ok = 0
        self.nonce_reuse = 0
        # ANATOMY (capstone ext): listen-window ms values printed per batch.
        self.listen_windows = []
        # ENGAGEMENT (capstone STEP 2a): per-process [ENGAGE-SUMMARY] parsed at
        # teardown, keyed by label ("CMD"/"RSP"); summed across both at the end.
        self.engage_by_label = {}
        # WARM-START readiness (metric-honesty fix): a fresh-launched modem pays a
        # one-time PHY precompute (ring pin + per-config bundle build) BEFORE its
        # accept loop runs, so a cold-launch harness charges ~30 s of boot to the
        # connect metric that a persistent field modem has long since finished. We
        # therefore wait until each peer is WARM (precompute done, accept loop up)
        # before starting the transfer clock / issuing CONNECT. A peer is WARM once
        # it has (1) emitted the bundle-build line AND (2) a subsequent
        # `link_status:Idle` printed by its running main loop. The initial
        # `link_status:Idle` printed ONCE before precompute (main.cc prints stats
        # before precook_pin_shared_ring) must NOT count -- gating on precook_done
        # first excludes it. warm_at[label] = wall seconds since t0 when warm.
        self.precook_done = {}     # label -> True once bundle-build line seen
        self.warm_at = {}          # label -> t0-relative wall seconds when warm
        self.link_events = LinkEventTracker()
        self.lock = threading.Lock()


def log_output(proc, label, logfile, t0, st):
    try:
        for line in iter(proc.stdout.readline, b''):
            text = line.decode("utf-8", "replace").rstrip()
            event_at = time.monotonic() - t0
            logfile.write(f"[T+{event_at:08.3f}] [{label}] {text}\n")
            logfile.flush()
            st.link_events.observe(label, text, event_at)
            if CONNECT_RE.search(text):
                with st.lock:
                    already_connected = (
                        st.cmd_connected if label == "CMD"
                        else st.rsp_connected)
                    if not already_connected:
                        st.first_connected_at_by_peer.setdefault(
                            label, event_at)
                        st.connected_at_by_peer[label] = event_at
                    if label == "CMD":
                        st.cmd_connected = True
                    else:
                        st.rsp_connected = True
                    st.connected = st.cmd_connected and st.rsp_connected
                    st.ever_connected = st.ever_connected or st.connected
            if DISC_RE.search(text):
                with st.lock:
                    st.disconnected = True
                    st.disconnected_at_by_peer.setdefault(label, event_at)
                    st.disconnect_source_by_peer.setdefault(
                        label, "stdout:link-release")
                    st.connected_at_by_peer.pop(label, None)
                    if label == "CMD":
                        st.cmd_connected = False
                    else:
                        st.rsp_connected = False
                    st.connected = st.cmd_connected and st.rsp_connected
            m = NRECV_RE.search(text)
            if m:
                v = int(m.group(1))
                with st.lock:
                    if label == "RSP":
                        st.rsp_nreceived = max(st.rsp_nreceived, v)
                    else:
                        st.cmd_nreceived = max(st.cmd_nreceived, v)
            m = TOSEND_RE.search(text)
            if m and label == "CMD":
                with st.lock:
                    st.last_ToSend_data = int(m.group(1))
            m = CFG_RE.search(text)
            if m:
                with st.lock:
                    st.configs_seen.add(int(m.group(1)))   # TARGET, not the config being left
            if BREAK_RE.search(text):
                with st.lock:
                    st.breaks += 1
            # ANATOMY (capstone ext): capture the CMD reverse-ACK listen window ms.
            lw = CA.parse_listen_window(text)
            if lw is not None:
                with st.lock:
                    st.listen_windows.append(lw)
            # ENGAGEMENT (capstone STEP 2a): the per-process [ENGAGE-SUMMARY]
            # atexit line. Store per label; a re-emit (shouldn't happen) keeps the
            # last/max. Requires the graceful SIGTERM teardown below (SIGKILL skips
            # atexit -> no line).
            eng = CA.parse_engage(text)
            if eng is not None:
                with st.lock:
                    st.engage_by_label[label] = eng
            # WARM-START readiness detection (see State comment). Precook line
            # first, then the next running-loop Idle marks the peer warm.
            if PRECOOK_DONE_RE.search(text):
                with st.lock:
                    st.precook_done[label] = True
            if IDLE_RE.search(text):
                with st.lock:
                    if st.precook_done.get(label) and label not in st.warm_at:
                        st.warm_at[label] = event_at
            if AUTHFAIL_RE.search(text):
                with st.lock:
                    st.authfails += 1
            if ENC_ACT_RE.search(text):
                with st.lock:
                    st.enc_activated = True
            m = NONCE_ENC_RE.search(text)
            if m:
                # (direction, unwrapped index) - per-peer the (key,nonce) pair must
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


def control_output(sock, label, logfile, t0, st, stop):
    """Drain and instrument one Mercury control socket.

    Mercury continuously emits status/flow-control records on this full-duplex
    socket.  Leaving them unread can fill its transmit side during a long
    transfer and starve later commands.  Most importantly for M1, the real
    terminal release acknowledgement (``DISCONNECTED``) exists only here.
    """
    pending = b""
    sock.settimeout(1.0)
    try:
        while not stop.is_set():
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            pending += chunk
            while b"\r" in pending:
                raw, pending = pending.split(b"\r", 1)
                raw = raw.lstrip(b"\n")
                text = raw.decode("utf-8", "replace").strip()
                if not text:
                    continue
                event_at = time.monotonic() - t0
                logfile.write(
                    f"[T+{event_at:08.3f}] [{label}-CTRL] {text}\n")
                logfile.flush()
                if text == CONTROL_DISCONNECTED:
                    with st.lock:
                        st.disconnected = True
                        st.disconnected_at_by_peer.setdefault(label, event_at)
                        st.disconnect_source_by_peer.setdefault(
                            label, "control:DISCONNECTED")
                        st.connected_at_by_peer.pop(label, None)
                        if label == "CMD":
                            st.cmd_connected = False
                        else:
                            st.rsp_connected = False
                        st.connected = st.cmd_connected and st.rsp_connected
    except (ValueError, OSError):
        pass


def tx_thread_fn(sock, stop, res, tx_control, traffic, t0, res_lock,
                 input_exhaustion):
    # Stream the TRAFFIC-CLASS source keyed on the absolute TX offset:
    #   compressible ("pg84"/"winlink") -> deterministic Winlink radio-email corpus
    #     (fed to a `-F on` mercury so the PPMd+dict+zstd streaming compressor
    #     ACTUALLY arms; RX decompresses so the delivered bytes ARE original content);
    #   incompressible ("random-binary") -> keyed counter-mode random bytes
    #     (fed to `-F off`; delivered == wire == content).
    # BOTH are NON-REPEATING pure functions of the offset, so the RX verifier
    # (rx_thread_fn) catches any hole/reorder/double-delivery (capstone STEP 2c).
    sock.settimeout(30)
    while not stop.is_set():
        with res_lock:
            offset = res["tx"]
            payload_total = tx_control["target"]
        if offset >= payload_total:
            break
        chunk_len = min(CA.CHUNK_LEN, payload_total - offset)
        chunk = CA.traffic_slice(
            traffic, offset, chunk_len)
        try:
            sock.sendall(chunk)
            _raw_tx(chunk)
            with res_lock:
                res["tx"] += len(chunk)
                if (res["tx"] >= payload_total
                        and input_exhaustion["at_s"] is None):
                    input_exhaustion["at_s"] = time.monotonic() - t0
        except (socket.timeout, ConnectionError, OSError):
            break
        time.sleep(0.03)


def rx_thread_fn(sock, stop, res, integ, traffic, oracle, t0, res_lock,
                 completion):
    """Receive the delivered stream AND byte-verify it against the TRAFFIC source
    (capstone ext (b)). res['rx'] is the running delivered total; the segment just
    received occupies absolute [base, base+len). For the compressible arm the
    delivered bytes are the ORIGINAL content (mercury RX-decompresses before it
    delivers), so they must match the pre-compression corpus at that offset — a
    single C-level bytes compare against traffic_expected_slice() is the fast path;
    a rare mismatch is localized byte-by-byte for the first bad offset. ANY
    mismatch marks the cell INTEGRITY_FAIL."""
    sock.settimeout(2)
    while not stop.is_set():
        try:
            d = sock.recv(8192)
            if not d:
                break
            _raw_rx(d)
            event_at = time.monotonic() - t0
            with res_lock:
                row = oracle.feed(event_at, d)
                # CAPTURE (root-cause): record the first N mismatched chunks
                # (base, delivered, expected) so the mask f=delivered^expected
                # can be identified offline. Bounded to avoid RAM/log bloat.
                exp = row.get("mismatch_expected")
                if exp is not None:
                    cap = integ.get("cap")
                    if cap is not None and len(cap) < 12:
                        cap.append((
                            row["segment_base"], bytes(d), bytes(exp)))
                res["rx"] = row["received_bytes"]
                integ["rx_md5"].update(d)
                integ["first_mismatch_off"] = oracle.first_bad_offset
                integ["mismatch_segments"] = oracle.mismatch_segments
                integ["mismatch_bytes"] = oracle.mismatch_bytes
                # COUNT-crossing instant (legacy; also the run-length trigger,
                # so cell wall-clock behavior is unchanged vs old cohorts):
                # first instant the raw delivered COUNT reached the payload
                # target. Content-blind - a sheared stream crosses on schedule.
                if (completion["payload_target"] is not None
                        and completion["count_at_s"] is None
                        and res["rx"] >= completion["payload_target"]):
                    completion["count_at_s"] = event_at
                # HONEST completion instant (field-level instrument fix,
                # 2026-07-31): first instant the byte-exact contiguous
                # good prefix covered the payload target.
                if (completion["payload_target"] is not None
                        and completion["at_s"] is None
                        and row["good_prefix_bytes"]
                        >= completion["payload_target"]):
                    completion["at_s"] = event_at
        except socket.timeout:
            continue
        except (ConnectionError, OSError):
            break


def _bin_fingerprint(path):
    """md5 + size + mtime of the binary that actually ran this cell.

    An A/B whose arms are distinguished only by a label is not an A/B. The retained
    cell summaries carried `arm: "A"` / `arm: "B"` and an IDENTICAL lever probe, with
    no record of which binary produced them -- so the arm identity could not be checked
    from the artifacts afterwards. Record it at run time, in the cell, always.
    """
    import hashlib, os
    try:
        h = hashlib.md5()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        st = os.stat(path)
        return {"bin_path": os.path.abspath(path), "bin_md5": h.hexdigest(),
                "bin_size": st.st_size, "bin_mtime": int(st.st_mtime)}
    except OSError as e:
        return {"bin_path": path, "bin_md5": None, "bin_error": str(e)}


def delivery_endpoint_times(events):
    """Delivery instants of this attempt, on the harness clock (seconds since t0,
    the clock of the [T+] log lines and of completion_at_s).

    last_rx_at_s is the arrival of the last delivered byte: the endpoint of an
    incomplete cell under the endpoint-bounded contamination rule. For a complete
    cell it is the arrival of the segment that finished the payload or a later one.
    last_good_prefix_advance_at_s is the last instant the byte-exact prefix grew;
    it equals last_rx_at_s whenever delivery stayed byte-exact."""
    last_rx_at_s = None
    last_good_at_s = None
    good = 0
    for ev in events:
        last_rx_at_s = ev["at_s"]
        if ev["good_prefix_bytes"] > good:
            good = ev["good_prefix_bytes"]
            last_good_at_s = ev["at_s"]
    return {"last_rx_at_s": last_rx_at_s,
            "last_good_prefix_advance_at_s": last_good_at_s,
            "delivery_event_count": len(events)}


OPT_NOT_FOUND_RE = re.compile(r"\[(CMD|RSP)\] \[OPT\] table not found \(path=([^)]*)\)")
OPT_LOADED_RE = re.compile(r"\[(CMD|RSP)\] \[OPT\] (?:WB|NB) calibration loaded: .*path=([^)]*)\)")
OPT_OTHER_RE = re.compile(r"\[(CMD|RSP)\] \[OPT\] (?:table empty|calibration metadata lacks|"
                          r"WARNING calibration config signature mismatch|calibration build differs|"
                          r"calibration contains no valid)")
# The modem's search order when MERCURY_RATE_TABLE is unset (mercury
# source/datalink_layer/arq_common.cc:12318-12325 @95723953): table-free = all four absent.
OPT_TABLE_FREE_PATHS = ("mercury/effective_rate_table.json", "effective_rate_table.json",
                        "mercury/effective_rate_table.synthetic.json",
                        "effective_rate_table.synthetic.json")


def rate_table_attestation(log_path, expect_path=None):
    """Per-peer check that the gearshift ran on the rate table the recipe names.

    Table-free (the product configuration): each peer prints exactly the four
    '[OPT] table not found' lines of the default search order and nothing else from
    the loader. With expect_path, each peer must print a calibration-loaded line for
    exactly that path. Anything else (a planted table in the cell directory, a set
    MERCURY_RATE_TABLE, an empty or mismatched table) fails the cell."""
    peers = {p: {"not_found": [], "loaded": [], "other": 0} for p in ("CMD", "RSP")}
    try:
        with open(log_path, "r", errors="replace") as fh:
            for line in fh:
                if "[OPT]" not in line:
                    continue
                m = OPT_NOT_FOUND_RE.search(line)
                if m:
                    peers[m.group(1)]["not_found"].append(m.group(2))
                    continue
                m = OPT_LOADED_RE.search(line)
                if m:
                    peers[m.group(1)]["loaded"].append(m.group(2))
                    continue
                m = OPT_OTHER_RE.search(line)
                if m:
                    peers[m.group(1)]["other"] += 1
    except OSError as exc:
        return {"ok": False, "reasons": ["log_unreadable:%s" % exc], "per_peer": peers,
                "expect_path": expect_path}
    reasons = []
    for p, v in peers.items():
        if expect_path is None:
            if tuple(v["not_found"]) != OPT_TABLE_FREE_PATHS or v["loaded"] or v["other"]:
                reasons.append("%s_not_table_free" % p)
        elif not v["loaded"] or any(x != expect_path for x in v["loaded"]) or v["other"]:
            reasons.append("%s_not_expected_table" % p)
    return {"ok": not reasons, "reasons": reasons, "per_peer": peers, "expect_path": expect_path}


def modem_bandwidth_args(mode, skip_nb_probe):
    """Bandwidth-entry flags for one modem.

    In ARQ mode -W only sets the value that ARQ start-up overwrites: the session
    starts narrowband unless the NB probe count is 0 with automatic bandwidth
    (mercury source/main.cc -W at 7310-7312, overwrite at 9840, condition at
    9808-9811). So the wb arm passes -Q 0 as well; without it a wb cell entered
    exactly like an auto cell."""
    c = []
    if mode == "wb":
        c += ["-W"]
    elif mode == "auto":
        c += ["-M", "auto"]
    elif mode == "nb":
        c += ["-M", "nb"]
    if skip_nb_probe or mode == "wb":
        c += ["-Q", "0"]
    return c


def dev(card, devno, sub):
    return f"hw:{card},{devno},{sub}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default="/home/kameron/raspeed/mercury")
    ap.add_argument("--bridge", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "realaudio_bridge_s32_c"))
    ap.add_argument("--start-cfg", type=int, default=100,
                    help="data-phase start config; -1 omits -s for the automatic stack")
    ap.add_argument("--skip-nb-probe", action="store_true",
                    help="pass -Q 0 (direct-WB start) independently of --no-gearshift")
    ap.add_argument("--force-robust", action="store_true",
                    help="keep -R (MFSK hailing) even with a WB --start-cfg; connect geometry "
                         "is independent of the data-phase config")
    ap.add_argument("--no-gearshift", action="store_true",
                    help="omit -g so --start-cfg HOLDS (pins; does not alter NB probing)")
    ap.add_argument("--mode", default="wb", choices=["wb", "auto", "nb"],
                    help="bandwidth election: wb=force wideband (-W, default, for "
                         "high-SNR WB cells); auto=NB/WB election ON (-M auto, for the "
                         "low-SNR FLOOR cell so Mercury's NB reach is not understated); "
                         "nb=NB-only (-M nb).")
    ap.add_argument("--secs", type=int, default=120)
    ap.add_argument("--payload", type=int, default=4096)
    ap.add_argument(
        "--fixed-window-score", action="store_true",
        help="also emit the legacy sustained-load fixed-window diagnostic; it "
             "does not change or invalidate the primary one-message --secs clock")
    ap.add_argument(
        "--score-horizon-s", type=float, default=600.0,
        help="fixed score horizon (default/canonical=600; shorter values are for "
             "deterministic instrument commissioning only)")
    ap.add_argument(
        "--steady-warmup-s", type=float, default=30.0,
        help="post-connect rung-settling interval (default/canonical=30)")
    ap.add_argument("--warm-start", action=argparse.BooleanOptionalAction, default=True,
                    help="PRE-WARM both peers (wait for the one-time PHY precompute to "
                         "finish and each peer's accept loop to reach link_status:Idle) "
                         "BEFORE starting the transfer clock and issuing CONNECT. This is "
                         "the honest, field-representative metric: a persistent daemon is "
                         "warm at connect time. --no-warm-start reproduces the old "
                         "cold-launch behaviour (connect measured from process launch, "
                         "which charges ~30 s of one-time boot to the connect metric).")
    ap.add_argument("--warm-timeout", type=float, default=120.0,
                    help="max seconds to wait for both peers to warm before proceeding "
                         "anyway (guards against a hang if the precook line never prints).")
    ap.add_argument("--warm-gate", action=argparse.BooleanOptionalAction, default=False,
                    help="observe the post-connect delivered-rate warm gate as a "
                         "SECONDARY diagnostic; it never delays or changes the "
                         "primary one-message whole-session score")
    ap.add_argument("--warm-floor-s", type=float, default=WARM_FLOOR_S)
    ap.add_argument("--warm-ceiling-s", type=float, default=WARM_CEILING_S)
    ap.add_argument("--warm-sample-s", type=float, default=WARM_SAMPLE_S)
    ap.add_argument("--warm-rate-window-s", type=float, default=WARM_RATE_WINDOW_S)
    ap.add_argument("--warm-stable-samples", type=int, default=WARM_STABLE_SAMPLES)
    ap.add_argument("--warm-rate-tolerance", type=float, default=WARM_RATE_TOLERANCE)
    ap.add_argument("--warm-max-tx-ahead-bytes", type=int,
                    default=WARM_MAX_TX_AHEAD_BYTES)
    ap.add_argument("--passthrough", action="store_true")
    ap.add_argument("--snr", type=float, default=30.0)
    ap.add_argument("--cell", default=None)
    ap.add_argument("--profile", default="wgn")
    ap.add_argument("--cfo-hz", type=float, default=0.0)
    ap.add_argument("--phase-noise-deg", type=float, default=0.0)
    ap.add_argument("--fade-depth-db", type=float, default=0.0)
    ap.add_argument("--erase-a2b-burst", type=int, default=0,
                    help="TEST-ONLY: erase the Nth forward signal burst")
    ap.add_argument("--erase-b2a-burst", type=int, default=0,
                    help="TEST-ONLY: erase the Nth reverse signal burst")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--cap-periods", type=int, default=3)
    ap.add_argument("--play-periods", type=int, default=4)
    ap.add_argument("--prime-periods", type=int, default=2)
    ap.add_argument("--tag", default="cell")
    ap.add_argument("--arm", default="legacy",
                    help="A/B arm label (e.g. redesign|legacy); recorded in JSON")
    ap.add_argument("--env", action="append", default=[],
                    help="KEY=VAL env var injected into BOTH mercury instances "
                         "(repeatable); used to carry the redesign env trio")
    ap.add_argument("--wire-stamp", type=int, choices=(0, 1), default=0,
                    help="pass --wire-stamp 0|1 to both modem processes and "
                         "record the exact argv for configuration-pin attestation")
    # ---- capstone ext (a): per-lever kill-switches -------------------------
    ap.add_argument("--arm-name", default=None,
                    help="preset arm name from capstone_arms.ARMS (e.g. all_off, "
                         "inband_only, inband_t1, full_stack). Resolves to env "
                         "toggles applied to BOTH mercury; combines with --arm-spec/--env.")
    ap.add_argument("--arm-spec", default=None,
                    help="JSON dict {lever:bool} of per-lever kill-switches "
                         "(e.g. '{\"inband_rate\":1,\"ack_slot\":1}'); resolves via "
                         "capstone_arms.resolve_arm. ALL-OFF == stock monitor baseline.")
    # ---- capstone ext (d): VARA bar + user metric --------------------------
    ap.add_argument("--snr3k", type=float, default=None,
                    help="required sole controlling coordinate for the v1 steady-SNR3k bridge")
    ap.add_argument("--traffic", default="pg84",
                    help="pg84/winlink (COMPRESSIBLE: feeds a real Winlink email corpus, "
                         "runs mercury -F on so PPMd+dict arms, scores DELIVERED decompressed "
                         "content vs the VARA CLIENT bar) | random-binary (INCOMPRESSIBLE: "
                         "keyed-random bytes, mercury -F off, scores WIRE vs the VARA WIRE bar)")
    ap.add_argument("--force-compress", default="auto", choices=["auto", "on", "off"],
                    help="mercury -F override: auto (default) derives from --traffic "
                         "(compressible=on so the compressor arms; incompressible=off); "
                         "on/off pins it (e.g. to A/B compression itself on one corpus)")
    ap.add_argument("--logdir", default="/tmp/raionos2/logs")
    ap.add_argument("--expect-rate-table", default=None,
                    help="path of the rate table the recipe runs with; omitted = the "
                         "product configuration, table-free (every peer must report the "
                         "four default paths absent)")
    ap.add_argument("--negative-control", default=None,
                    help="declare ONE registered behaviour-restoring switch as this run's "
                         "negative control (see _research/PREFLIGHT_OVERRIDES.json); without "
                         "it any active registered switch refuses the launch")
    ap.add_argument("--json", default=None)
    ap.add_argument("--replay-result", default=None,
                    help="CPU-only trust replay: production result JSON to re-evaluate")
    ap.add_argument("--replay-log", default=None,
                    help="CPU-only trust replay: raw ARQ log paired with --replay-result")
    # Concurrency knobs: distinct card + 4 disjoint substreams + distinct ports.
    ap.add_argument("--card", default="Loopback",
                    help="ALSA snd-aloop card name or index")
    ap.add_argument("--subs", default="0,1,2,3",
                    help="4 substream indices S0,S1,S2,S3 (CMD-tx, RSP-rx, RSP-tx, CMD-rx-impaired)")
    ap.add_argument("--rsp-port", type=int, default=7002)
    ap.add_argument("--cmd-port", type=int, default=7006)
    ap.add_argument("--no-kill", action="store_true",
                    help="do NOT global-pkill mercury/bridge on start (REQUIRED for concurrent runs)")
    # ---- cohort-width provenance (candidate instrument #17) ----------------
    # The spawner is the sole producer of these; a direct single-cell run leaves
    # them unset and is refused for vs-bar scoring unless the scorer is given an
    # explicit --axis-override. Recorded verbatim into the result JSON.
    ap.add_argument("--spawner-width", type=int, default=None,
                    help="cohort width (the spawner's -n) recorded into the "
                         "result JSON so the scorer can enforce width <= 8 on a "
                         "scorable cohort")
    ap.add_argument("--box-concurrent-estimate", type=int, default=None,
                    help="pgrep census of concurrent real-audio cells on this "
                         "box at THIS cell's launch (siblings included, self "
                         "counted); recorded so a scoreboard+fineprobe overlap "
                         "is visible in the per-cell data")
    ap.add_argument("--broker-override-window", default=None,
                    help="identifier of the sanctioned broker measurement-window "
                         "under which a width>cap diagnostic cohort ran (raised "
                         "--global-card-limit + whole-box lease); recorded "
                         "verbatim into the result JSON so a cell run outside the "
                         "standard broker caps is never mistaken for a scoreboard "
                         "cohort")
    ap.add_argument("--startup-phase-file", default=None,
                    help="atomic cell phase event consumed by parallel_spawner's "
                         "cold-PRECOOK admission gate")
    ap.add_argument("--encrypt", default=None,
                    help="encryption mode passed to BOTH mercury via -E (e.g. 'fast' or 'strict'); "
                         "omit for plaintext")
    ap.add_argument("--psk", default=None,
                    help="pre-shared key hex passed to BOTH mercury via -K (required with --encrypt)")
    ap.add_argument("--tx-gain-ini", default=None,
                    help="INI whose [TxGain] section each sim modem loads from a private HOME; "
                         "fails closed unless both peers log the requested gain overrides")
    args = ap.parse_args()
    if args.snr3k is None:
        ap.error("--snr3k is required as the sole controlling channel coordinate")
    if args.cell is not None:
        ap.error("--cell is forbidden: this campaign requires --snr3k as the sole coordinate")
    if args.start_cfg < -1:
        ap.error("--start-cfg must be -1 (automatic) or a real configuration")
    if bool(args.replay_result) != bool(args.replay_log):
        ap.error("--replay-result and --replay-log must be supplied together")
    if args.replay_result:
        with open(args.replay_result, encoding="utf-8") as stream:
            replayed = json.load(stream)
        replayed = apply_terminal_eot_gate(replayed, args.replay_log)
        print(json.dumps(replayed))
        if args.json:
            with open(args.json, "w", encoding="utf-8") as stream:
                json.dump(replayed, stream, indent=1)
        return 3 if replayed.get("whole_session_status") == "VOID" else 0
    try:
        bridge_prefix = bridge_command_prefix(args.bridge)
    except BridgeCommandError as exc:
        ap.error(str(exc))
    try:
        bridge_ref_argv, bridge_ref_requested = bridge_reference.resolve(os.environ)
    except (bridge_reference.BridgeReferenceError, ValueError) as exc:
        ap.error(str(exc))
    try:
        DeliveredRateWarmGate(
            floor_s=args.warm_floor_s,
            ceiling_s=args.warm_ceiling_s,
            sample_s=args.warm_sample_s,
            rate_window_s=args.warm_rate_window_s,
            stable_samples=args.warm_stable_samples,
            tolerance=args.warm_rate_tolerance,
        )
    except ValueError as exc:
        ap.error(str(exc))
    if args.warm_max_tx_ahead_bytes <= 0:
        ap.error("--warm-max-tx-ahead-bytes must be positive")
    if args.fixed_window_score:
        if args.payload <= 0:
            ap.error("--fixed-window-score requires --payload > 0")
        if (args.score_horizon_s <= 0
                or args.steady_warmup_s < 0
                or args.steady_warmup_s >= args.score_horizon_s):
            ap.error(
                "--fixed-window-score requires 0 <= --steady-warmup-s "
                "< --score-horizon-s")

    os.makedirs(args.logdir, exist_ok=True)

    # ---- capstone ext (a): resolve the arm-spec into env KEY=VAL pairs ------
    # Merge order: preset --arm-name, then --arm-spec JSON, then explicit --env
    # (explicit --env always wins so an operator can override). ALL-OFF resolves
    # to [] => nothing injected => stock monitor baseline (verified in smoke).
    arm_spec = {}
    if args.arm_name:
        arm_spec.update(CA.normalize_spec(args.arm_name))
    if args.arm_spec:
        arm_spec.update({k: bool(v) for k, v in json.loads(args.arm_spec).items()})
    arm_env_kv, arm_warnings = CA.resolve_arm(arm_spec) if arm_spec else ([], [])
    # binary probe: which requested RUNTIME levers actually exist in this binary
    probe = CA.probe_binary(args.bin)
    inert_levers = []
    for lever, on in CA.normalize_spec(arm_spec).items():
        if on and CA.LEVERS[lever]["kind"] == "runtime" and probe.get(lever) is False:
            inert_levers.append(f"{lever} ({CA.LEVERS[lever]['probe']} absent from binary - INERT)")

    cell_env = dict(os.environ)
    for kv in list(arm_env_kv) + list(args.env):
        if "=" in kv:
            k, v = kv.split("=", 1)
            cell_env[k] = v
    peer_env = {"RSP": cell_env, "CMD": cell_env}
    # -R is MFSK weak-signal HAILING (the CONNECT handshake); -s is the DATA-phase config.
    # They are independent. Tying them meant --start-cfg 16 silently dropped -R, so the
    # responder never heard a HAIL and the cell never connected (F6, 2026-07-09).
    use_robust = (args.start_cfg >= 100) or args.force_robust
    rsp_port = args.rsp_port
    cmd_port = args.cmd_port
    # ---- PORT-COLLISION PREFLIGHT (universal harness guard) ----------------
    # Each modem binds a CONTROL listener at its base port and a DATA listener
    # at base+1 (arq_common.cc: tcp_socket_data.port = tcp_base_port + 1). So
    # the responder owns the port set {rsp_port, rsp_port+1} and the commander
    # owns {cmd_port, cmd_port+1}. If those two sets intersect, two listeners
    # fight for the same port; the kernel load-balances inbound connections
    # between them, so the harness CONNECT can be delivered to the wrong modem
    # and the commander wedges at Idle. Refuse to launch loudly rather than
    # score a silently-corrupt cell. Disjointness of the two sets is exactly a
    # >=2 separation between the base ports (in EITHER order -- cmd-below-rsp is
    # a legitimate in-tree convention), so cmd_port == rsp_port+1 (the
    # connect-cohort instrument artifact) is rejected here while the safe
    # defaults (7002/7006) and every disjoint arrangement pass.
    rsp_ports = {rsp_port, rsp_port + 1}   # responder {control, data}
    cmd_ports = {cmd_port, cmd_port + 1}   # commander {control, data}
    _overlap = rsp_ports & cmd_ports
    if _overlap or abs(cmd_port - rsp_port) < 2:
        raise SystemExit(
            f"[PORT-GUARD] control/data port-set collision: responder binds "
            f"{sorted(rsp_ports)} (control,data), commander binds "
            f"{sorted(cmd_ports)} (control,data); the base ports must differ "
            f"by >= 2 so the four listeners are pairwise disjoint "
            f"(overlap={sorted(_overlap)}).")
    # ------------------------------------------------------------------------
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
        # THIS run's TCP ports. A concurrent run on a DISJOINT
        # card/subs/port set can never match, so we never reap it. The old
        # global `pkill -9 -f 'mercury -m ARQ'` reaped every concurrent
        # run's mercury (an earlier diagnostic). See ra_cleanup.py.
        scoped_cleanup(args.card, subs, [rsp_port, cmd_port], settle=1.5)

    # OVERRIDE PREFLIGHT: an efficacy/normal run refuses to launch with a registered
    # behaviour-restoring switch active; a negative control must be declared.
    override_preflight = override_guard.check(cell_env, args.negative_control)
    if override_preflight.get("status") == "REJECT":
        msg = "[PREFLIGHT-OVERRIDE] REJECT: " + "; ".join(override_preflight.get("reasons", []))
        sys.stderr.write(msg + "\n")
        if args.json:
            with open(args.json, "w") as f:
                json.dump({"tag": args.tag, "verdict": "PREFLIGHT_REJECT",
                           "instrument_invalid": True,
                           "instrument_invalid_reasons": ["override_preflight_reject"],
                           "override_preflight": override_preflight}, f, indent=1)
        raise SystemExit(3)
    global _RAW_EVIDENCE
    _RAW_EVIDENCE = RawEvidence(args.logdir, args.tag, args.traffic, CA.traffic_slice)
    raw_evidence_record = {"status": "NOT-FINALIZED"}
    logpath = os.path.join(args.logdir, f"arq_{args.tag}.log")
    logfile = open(logpath, "w")
    st = State()
    procs, sockets = [], []
    log_threads = []            # stdout readers, joined at teardown so the final
                                # [ENGAGE-SUMMARY] atexit line is captured.
    control_threads = []        # full-duplex control drains; these observe the
                                # real per-peer DISCONNECTED release acknowledgements.
    stop = threading.Event()
    t0 = time.monotonic()
    _publish_startup_phase(
        args.startup_phase_file, "cooking", t0, args.tag,
        warm_start=bool(args.warm_start))
    bridge = None
    res = {"tx": 0, "rx": 0}
    res_lock = threading.Lock()
    delivery_oracle = DeliveryOracle(
        lambda offset, length: CA.traffic_expected_slice(
            args.traffic, offset, length))
    # at_s = HONEST completion (good-prefix crossing); count_at_s = legacy
    # count crossing.  The primary contract always feeds exactly ONE bounded
    # payload after CONNECT; a warm diagnostic must never manufacture a prefix
    # transfer or grant a second payload.
    completion = {"at_s": None, "count_at_s": None,
                  "payload_target": args.payload}
    input_exhaustion = {"at_s": None}
    # One message per session: exactly --payload bytes are made available once,
    # after the simultaneous two-peer connection is established.
    tx_control = {
        "target": args.payload,
    }
    # byte-integrity accumulator (capstone ext (b)); rx_md5 = running md5 of
    # the delivered stream (durable content-equality record for offline audit)
    integ = {"mismatch_bytes": 0, "mismatch_segments": 0, "first_mismatch_off": None,
             "cap": [], "rx_md5": hashlib.md5()}
    timeline = []          # (t, rx_bytes) samples for compute_inburst
    connected_at = None
    # WARM-START metric bookkeeping (defined here so an early exception in the launch
    # block still leaves the summary math well-defined; warm_offset == 0 => legacy cold).
    warm_offset = 0.0
    warm_snapshot = {}
    connected_at_cold = None
    connect_issued_at = None
    run_end_s = None
    connect_onair_s = None
    connected_at_by_peer = {}
    disconnect_issued_at_s = None
    terminal_settlement_s = None
    observed_until_s = 0.0
    fixed_window_score = None
    # Required warm-gate fields have defaults even on launch/connect failure.
    warm_seconds = None
    under_warmed = False
    # scored_window_start is the PRIMARY CONNECT-issued edge.  The optional
    # post-warm diagnostic has a separate steady_window_start.
    scored_window_start = None
    steady_window_start = None
    warm_rx_bytes = None
    warm_tx_bytes = None
    warm_good_prefix_bytes = None
    warm_breaks = None
    warm_confirmed_rate_Bps = None
    # Machine-readable legibility defaults; set only by the exact branches below.
    connect_fail_reason = None
    instrument_invalid = False
    instrument_invalid_reasons = []
    teardown_survivor_pids = []
    bridge_stats_path = os.path.join(args.logdir, f"bridge_{args.tag}_stats.json")

    # -F (force-compress) resolution: for the COMPRESSIBLE arm we MUST run -F on so
    # the production PPMd+dict+zstd streaming compressor arms (arq_commander.cc:6699
    # force_compress -> compression_enabled -> streaming_enable). The old harness
    # hardcoded -F off, so on the (non-B2F) traffic the compressor stayed asleep and
    # the "compression benefit" was a post-hoc x2.0907 fiction. For INCOMPRESSIBLE
    # we run -F off (delivered == wire == content). Applied to BOTH mercury (the TX
    # commander compresses, the RX responder decompresses before delivery).
    if args.force_compress == "on":
        force_compress_flag = "on"
    elif args.force_compress == "off":
        force_compress_flag = "off"
    else:  # auto: derive from traffic class
        force_compress_flag = "on" if CA.is_compressible_traffic(args.traffic) else "off"

    binary_hasher = hashlib.sha256()
    with open(args.bin, "rb") as binary_stream:
        for binary_chunk in iter(lambda: binary_stream.read(1 << 20), b""):
            binary_hasher.update(binary_chunk)
    binary_sha256 = binary_hasher.hexdigest()
    binary_fingerprint = _bin_fingerprint(args.bin)
    try:
        harness_ident = HA.identity(args.bridge, __file__, HARNESS_LINEAGE)
    except (OSError, ValueError) as exc:
        ap.error("cannot attest bridge/harness identity: %s" % exc)
    if not harness_ident["bridge_program_resolved"]:
        # A wrapper whose program cannot be named: the result could not say which
        # bridge binary ran.
        instrument_invalid = True
        instrument_invalid_reasons.append("bridge_program_unresolved")
    bridge_ident_argv = (HA.bridge_identity_argv(harness_ident)
                         if HA.bridge_accepts_identity(args.bridge) else [])
    tx_gain_ini, tx_gain_ini_source = HA.resolve_tx_gain_ini(
        args.tx_gain_ini, os.environ)
    tx_gain_values = {}
    if tx_gain_ini:
        try:
            tx_gain_values = HA.read_tx_gain_ini(tx_gain_ini)
        except (OSError, HA.TxGainError) as exc:
            ap.error("--tx-gain-ini: %s" % exc)
    tx_gain_settings = {}
    if tx_gain_values:
        for peer in ("RSP", "CMD"):
            peer_home = os.path.abspath(os.path.join(
                args.logdir, "txgain_home_%s_%s" % (args.tag, peer)))
            tx_gain_settings[peer] = HA.write_modem_settings(
                peer_home, tx_gain_values)
            peer_env[peer] = dict(cell_env, HOME=peer_home)
    bridge_recipe_sha256 = hashlib.sha256(json.dumps({
        "axis": "v1-steady-snr3k",
        "configured_bandwidth_hz": 2343.75,
        "snr3k": args.snr3k,
        "profile": args.profile,
        "seed": args.seed,
        "binary_sha256": binary_sha256,
        "mode": args.mode,
        "start_cfg": args.start_cfg,
    }, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()

    def mercury_cmd(port, in_dev, out_dev):
        c = [args.bin, "-m", "ARQ"]
        if args.start_cfg >= 0:
            c += ["-s", str(args.start_cfg)]
        c += ["-p", str(port), "-x", "alsa", "-i", in_dev, "-o", out_dev,
              "-n", "-F", force_compress_flag,
              "--wire-stamp", str(args.wire_stamp)]
        # bandwidth election (capstone STEP 3 FLOOR cell): wb starts direct-WB
        # (-W -Q 0, see modem_bandwidth_args); auto/nb enable the NB/WB election so
        # the low-SNR floor is not understated.
        # -g (gearshift) and -Q 0 (skip the NB probe, start direct-WB) are INDEPENDENT
        # knobs. Welding them together meant a pinned-vs-climbing comparison silently
        # differed by TWO flags, so no single-variable A/B on the gearshift was possible.
        c += modem_bandwidth_args(args.mode, args.skip_nb_probe)
        if not args.no_gearshift:
            c += ["-g"]
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
        command = mercury_cmd(port, in_dev, out_dev)
        logfile.write("[HARNESS-MODEM-ARGV] label=%s argv=%s\n" %
                      (label, json.dumps(command)))
        logfile.flush()
        p = subprocess.Popen(command,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             env=peer_env[label])
        th = threading.Thread(target=log_output, args=(p, label, logfile, t0, st),
                              daemon=True)
        th.start()
        log_threads.append(th)
        return p

    try:
        # 1. bridge first (opens the 4 loopback subdevices for THIS run).
        bcmd = bridge_prefix + [
                "--fwd-cap", fwd_cap, "--fwd-play", fwd_play,
                "--rev-cap", rev_cap, "--rev-play", rev_play,
                "--axis", "v1-steady-snr3k",
                "--configured-bandwidth-hz", "2343.75",
                "--snr3k", str(args.snr3k), "--profile", args.profile,
                "--binary-sha256", binary_sha256,
                "--recipe-sha256", bridge_recipe_sha256,
                "--s32-mode", "auto",
                "--cfo-hz", str(args.cfo_hz),
                "--phase-noise-deg", str(args.phase_noise_deg),
                "--fade-depth-db", str(args.fade_depth_db),
                "--seed", str(args.seed),
                "--cap-periods", str(args.cap_periods),
                "--play-periods", str(args.play_periods),
                "--prime-periods", str(args.prime_periods),
                "--statsfile", bridge_stats_path] + bridge_ident_argv + bridge_ref_argv
        # Burst erasure is a test-only relay feature and is not part of a WGN
        # comparator arm.  Do not pass unsupported no-op flags to the native
        # commissioned ALSA bridge; fail closed if a test explicitly asks for
        # either feature against that bridge.
        if args.erase_a2b_burst or args.erase_b2a_burst:
            raise RuntimeError(
                "test-only directional burst erasure is unsupported by the "
                "native real-audio bridge")
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

        # 2b. WARM-START gate: wait for both peers to finish the one-time PHY
        # precompute and bring their accept loops up to link_status:Idle before
        # starting the transfer clock / issuing CONNECT. A persistent field modem is
        # warm at connect time, so charging its cold-boot precompute (~30 s, printed
        # ONCE per process, never per-connect) to the connect metric is a harness
        # artifact. FIRE PROOF: log the wall time each peer ACTUALLY reached warm (we
        # WAITED for it -- not a fixed sleep) and the offset the clock skips.
        warm_offset = 0.0
        warm_snapshot = {}
        if args.warm_start:
            wstart = time.monotonic()
            proc_died = False
            while (len(st.warm_at) < 2
                   and (time.monotonic() - wstart) < args.warm_timeout):
                if any(p.poll() is not None for p in procs):
                    proc_died = True
                    break
                time.sleep(0.25)
            with st.lock:
                warm_snapshot = dict(st.warm_at)
            if len(warm_snapshot) >= 2:
                warm_offset = max(warm_snapshot.values())
                _publish_startup_phase(
                    args.startup_phase_file, "warmed_idle", t0, args.tag,
                    warm_at_s={
                        peer: round(at_s, 6)
                        for peer, at_s in sorted(warm_snapshot.items())
                    })
                logfile.write(
                    f"[T+{time.monotonic()-t0:08.3f}] [WARM-START] both peers warm: "
                    f"RSP@T+{warm_snapshot.get('RSP'):.3f} "
                    f"CMD@T+{warm_snapshot.get('CMD'):.3f}; transfer/connect clock "
                    f"starts warm (offset={warm_offset:.3f}s of one-time boot skipped)\n")
            else:
                # A warm-gate fallback is an invalid instrument measurement, not a modem result.
                instrument_invalid = True
                instrument_invalid_reasons.append("warm_start_incomplete")
                logfile.write(
                    f"[T+{time.monotonic()-t0:08.3f}] [WARM-START] WARN: only {len(warm_snapshot)}/2 "
                    f"peers warm after {args.warm_timeout:.0f}s (warm_at={warm_snapshot}, "
                    f"proc_died={proc_died}); proceeding COLD-equivalent (offset=0)\n")
            logfile.flush()

        # 3. control + data sockets (identical to canonical sim_arq_channel.py).
        rsp_ctrl = tcp_send(rsp_port, ["MYCALL TESTB\r\n", "LISTEN ON\r\n"])
        sockets.append(rsp_ctrl)
        rsp_ctrl_thread = threading.Thread(
            target=control_output,
            args=(rsp_ctrl, "RSP", logfile, t0, st, stop),
            daemon=True)
        rsp_ctrl_thread.start()
        control_threads.append(rsp_ctrl_thread)
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
                         args=(rsp_data, stop, res, integ, args.traffic,
                               delivery_oracle, t0, res_lock, completion),
                         daemon=True).start()
        time.sleep(0.5)

        def _stamp_connect_request(command):
            nonlocal connect_issued_at
            if command.lstrip().upper().startswith("CONNECT "):
                # Exact clock edge immediately before the CONNECT socket write.
                connect_issued_at = time.monotonic() - t0

        cmd_ctrl = tcp_send(
            cmd_port,
            ["MYCALL TESTA\r\n", "CONNECT TESTA TESTB\r\n"],
            before_send=_stamp_connect_request)
        sockets.append(cmd_ctrl)
        cmd_ctrl_thread = threading.Thread(
            target=control_output,
            args=(cmd_ctrl, "CMD", logfile, t0, st, stop),
            daemon=True)
        cmd_ctrl_thread.start()
        control_threads.append(cmd_ctrl_thread)
        if connect_issued_at is None:
            raise RuntimeError("CONNECT command was not timestamped")
        scored_window_start = connect_issued_at

        def _start_tx():
            nonlocal tx_started
            if tx_started:
                return
            tx_started = True
            threading.Thread(
                target=tx_thread_fn,
                args=(cmd_data, stop, res, tx_control, args.traffic,
                      t0, res_lock, input_exhaustion),
                daemon=True,
            ).start()

        tx_started = False
        # PRIMARY WHOLE-SESSION CONTRACT: one common CONNECT-origin horizon.
        # Connection establishment consumes this same budget; a late connection
        # does not receive a fresh transfer horizon. --fixed-window-score may
        # additionally emit its legacy diagnostic, but cannot change this clock.
        run_end_s = connect_issued_at + float(args.secs)
        dead = False
        while (time.monotonic() - t0) < run_end_s and not st.connected:
            if any(p.poll() is not None for p in procs):
                dead = True
                instrument_invalid = True
                instrument_invalid_reasons.append("process_died_before_connect")
                break
            time.sleep(0.25)

        with st.lock:
            connected_at_by_peer = dict(st.connected_at_by_peer)
        connected_at_cold = (
            max(connected_at_by_peer.values())
            if st.connected and len(connected_at_by_peer) == 2 else None)
        if connected_at_cold is None:
            connect_fail_reason = "deadline"
        else:
            connect_fail_reason = None
            _publish_startup_phase(
                args.startup_phase_file, "active", t0, args.tag,
                connected_at_cold_s=round(connected_at_cold, 6))
            _start_tx()
        # connected_at_cold: seconds from process launch (t0) -- includes the one-time
        # PHY precompute when NOT warm-started (the old cold metric).
        # connected_at (headline): seconds from the WARM clock (t0 + warm_offset) --
        # the field-honest connect time. warm_offset == 0 under --no-warm-start, so
        # the two collapse to the legacy cold number.
        connected_at = (connected_at_cold - warm_offset) if (connected_at_cold is not None) else None
        # on-air handshake time = from CONNECT-issued to Connected (excludes harness
        # socket prologue); the pure PHY connect the Lever-B race guard targets.
        connect_onair_s = (connected_at_cold - connect_issued_at) if (connected_at_cold is not None) else None

        # 4. OPTIONAL SECONDARY warm-gate observer.  It watches the SAME single
        # primary payload.  It never starts traffic, gates primary eligibility,
        # resets counters, extends the deadline, or grants another payload.
        rate_gate = None
        next_sample_s = 0.0
        warm_decision = None
        if connected_at_cold is not None and args.warm_gate:
            rate_gate = DeliveredRateWarmGate(
                floor_s=args.warm_floor_s,
                ceiling_s=args.warm_ceiling_s,
                sample_s=args.warm_sample_s,
                rate_window_s=args.warm_rate_window_s,
                stable_samples=args.warm_stable_samples,
                tolerance=args.warm_rate_tolerance,
            )
        elif connected_at_cold is not None:
            # Diagnostic opt-out remains mechanically explicit in JSON.
            warm_seconds = 0.0
            with res_lock:
                warm_rx_bytes = 0
                warm_tx_bytes = 0
                warm_good_prefix_bytes = 0
            with st.lock:
                warm_breaks = 0

        while (time.monotonic() - t0) < run_end_s and not dead:
            time.sleep(0.25 if args.fixed_window_score else 1.0)
            now_s = time.monotonic() - t0
            if rate_gate is not None and warm_decision is None:
                warm_elapsed_s = now_s - connected_at_cold
                if warm_elapsed_s >= next_sample_s:
                    with res_lock:
                        delivered_now = res["rx"]
                    warm_decision = rate_gate.observe(
                        warm_elapsed_s, delivered_now)
                    next_sample_s += args.warm_sample_s
                if warm_decision is not None:
                    warm_seconds = round(warm_decision.warm_seconds, 3)
                    under_warmed = warm_decision.under_warmed
                    warm_confirmed_rate_Bps = warm_decision.rate_Bps
                    with res_lock:
                        warm_rx_bytes = res["rx"]
                        warm_tx_bytes = res["tx"]
                        warm_good_prefix_bytes = delivery_oracle.snapshot()[
                            "good_prefix_bytes"]
                    with st.lock:
                        warm_breaks = st.breaks
                    if warm_decision.confirmed:
                        steady_window_start = now_s
                        logfile.write(
                            f"[T+{now_s:08.3f}] [WARM-GATE] CONFIRMED "
                            f"warm_seconds={warm_seconds:.3f} rate_Bps="
                            f"{warm_confirmed_rate_Bps:.3f}; SECONDARY suffix "
                            f"starts at rx={warm_rx_bytes} tx={warm_tx_bytes} "
                            f"breaks={warm_breaks}; primary unchanged\n")
                    else:
                        logfile.write(
                            f"[T+{now_s:08.3f}] [WARM-GATE] CEILING "
                            f"warm_seconds={warm_seconds:.3f}; under_warmed=true; "
                            "secondary suffix unavailable; primary unchanged\n")
                    logfile.flush()
            # ANATOMY (capstone ext): sample the delivered-byte timeline once/s
            # for active_fraction (compute_inburst). Only after connect.
            if connected_at_cold is not None:
                with res_lock:
                    timeline.append((
                        time.monotonic(),
                        res["rx"],
                    ))
            # Stop at byte-valid completion.  If the count crossed with a known
            # content mismatch, the attempt is already a VOID integrity result
            # and no additional payload exists to repair it.
            if (completion["at_s"] is not None
                    or (completion["count_at_s"] is not None
                        and integ["mismatch_bytes"] > 0)):
                break
            for p in procs:
                if p.poll() is not None:
                    dead = True
                    instrument_invalid = True
                    instrument_invalid_reasons.append(
                        "process_died_during_primary_transfer")

        # M1 ends channel occupancy only after the completed application
        # transfer releases both endpoints.  Byte delivery is not settlement:
        # the final ACK/control turn may still be on air.  Keep the original
        # CONNECT-origin H deadline unchanged, with a short observation-only
        # grace so a release just beyond H can be recorded and scored at zero.
        if completion["at_s"] is not None and not dead:
            disconnect_issued_at_s = time.monotonic() - t0
            try:
                cmd_ctrl.sendall(b"DISCONNECT\r\n")
                logfile.write(
                    f"[T+{disconnect_issued_at_s:08.3f}] [M1] DISCONNECT issued "
                    "after byte-exact completion; awaiting both endpoint releases\n")
                logfile.flush()
            except OSError as exc:
                instrument_invalid = True
                instrument_invalid_reasons.append("disconnect_request_not_issued")
                logfile.write(
                    f"[T+{time.monotonic()-t0:08.3f}] [M1] DISCONNECT send failed: "
                    f"{exc!r}\n")
                logfile.flush()
            else:
                observe_until_s = run_end_s + TERMINAL_SETTLEMENT_GRACE_S
                while (time.monotonic() - t0) < observe_until_s:
                    with st.lock:
                        releases = dict(st.disconnected_at_by_peer)
                    if len(releases) == 2:
                        terminal_settlement_s = max(releases.values())
                        break
                    time.sleep(0.1)
                if terminal_settlement_s is None:
                    with st.lock:
                        releases = dict(st.disconnected_at_by_peer)
                    if len(releases) == 2:
                        terminal_settlement_s = max(releases.values())
                logfile.write(
                    f"[T+{time.monotonic()-t0:08.3f}] [M1] terminal settlement="
                    f"{terminal_settlement_s!r} releases={releases} "
                    f"sources={dict(st.disconnect_source_by_peer)}\n")
                logfile.flush()
        observed_until_s = time.monotonic() - t0
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"[harness] {e}\n")
    finally:
        # End of the measurement observation interval. Teardown time must never
        # make an early-ended fixed window look complete.
        observed_until_s = max(observed_until_s, time.monotonic() - t0)
        stop.set()
        for s in sockets:
            try:
                s.close()
            except OSError:
                pass
        for th in control_threads:
            th.join(timeout=2)
        # GRACEFUL teardown so mercury's atexit [ENGAGE-SUMMARY] line actually
        # prints (capstone STEP 2a). SIGKILL skips atexit; mercury installs a
        # graceful SIGTERM/SIGINT handler (main.cc install_termination_handlers)
        # that winds the audio threads down and RETURNS from main -> atexit fires.
        # A surviving card holder invalidates the run; never hard-kill it here.
        survivors = terminate_popen_processes(procs, grace_seconds=30)
        teardown_survivor_pids.extend(
            proc.pid for proc in survivors if getattr(proc, "pid", None) is not None)
        # Join the stdout readers so the final [ENGAGE-SUMMARY] line is drained
        # into the log + State before logfile.close().
        for th in log_threads:
            th.join(timeout=5)
        if bridge:
            survivors = terminate_popen_processes([bridge], grace_seconds=30)
            teardown_survivor_pids.extend(
                proc.pid for proc in survivors if getattr(proc, "pid", None) is not None)
        if teardown_survivor_pids:
            logfile.write(
                "[TEARDOWN-REAP-REQUIRED] owned process(es) survived 30 s "
                "SIGTERM grace after the measurement endpoint; the wave "
                "controller must reap this process group before lease release: %s\n" %
                ",".join(str(pid) for pid in teardown_survivor_pids))
            logfile.flush()
        try:
            blog.close()
        except (NameError, OSError):
            pass
        logfile.close()
        # Raw bytes are on disk already (streamed); hash + independent compare now.
        try:
            raw_evidence_record = _RAW_EVIDENCE.finalize(res["rx"], res["tx"])
        except Exception as exc:  # noqa: BLE001 -- recorded, never silent
            raw_evidence_record = {"status": "FINALIZE-FAILED", "error": repr(exc)}

    dwell_cold = max(1.0, time.monotonic() - t0)
    # Legacy launch/warm dwell views remain diagnostic-only. Primary score and
    # duty denominators below use the explicit CONNECT-issued whole-session clock.
    dwell = max(1.0, dwell_cold - warm_offset)
    oracle_snapshot = delivery_oracle.snapshot()
    delivery_times = delivery_endpoint_times(oracle_snapshot["events"])
    rate_table = rate_table_attestation(logpath, args.expect_rate_table)
    if not rate_table["ok"]:
        instrument_invalid = True
        instrument_invalid_reasons.append("rate_table_attestation")
    # SECONDARY-only post-warm suffix.  These values are retained for engineering
    # diagnosis, but are never the primary score and never affect eligibility.
    steady_rx_bytes = scored_delta(res["rx"], warm_rx_bytes, under_warmed)
    steady_tx_bytes = scored_delta(res["tx"], warm_tx_bytes, under_warmed)
    steady_good_prefix_bytes = scored_delta(
        oracle_snapshot["good_prefix_bytes"],
        warm_good_prefix_bytes,
        under_warmed,
    )
    steady_breaks = scored_delta(st.breaks, warm_breaks, under_warmed)
    steady_window_seconds = (
        max(0.0, observed_until_s - steady_window_start)
        if steady_window_start is not None and not under_warmed else None
    )
    # PRIMARY whole-session clock and numerator.  CONNECT establishment and ramp
    # are charged.  Only a byte-valid, unique full payload earns bytes; a valid
    # connect/transfer deadline attempt earns exactly zero while remaining
    # scorable.  Instrument/channel validity is finalized below.
    whole_session_completed_prelim = bool(
        completion["at_s"] is not None
        and (run_end_s is None or completion["at_s"] <= run_end_s)
        and oracle_snapshot["good_prefix_bytes"] >= args.payload
        and integ["mismatch_bytes"] == 0
        and res["rx"] <= res["tx"])
    whole_transfer_complete = bool(
        whole_session_completed_prelim
        and terminal_settlement_s is not None
        and terminal_settlement_s <= run_end_s)
    whole_session_end_s = (
        max(completion["at_s"], terminal_settlement_s)
        if whole_session_completed_prelim and terminal_settlement_s is not None
        else (min(observed_until_s, run_end_s)
              if run_end_s is not None else observed_until_s))
    whole_session_window_s = (
        max(0.0, whole_session_end_s - connect_issued_at)
        if connect_issued_at is not None else None)
    whole_session_credited_bytes = (
        args.payload if whole_transfer_complete else 0)
    scored_rx_bytes = res["rx"]
    scored_tx_bytes = res["tx"]
    scored_good_prefix_bytes = whole_session_credited_bytes
    scored_breaks = st.breaks
    scored_window_seconds = whole_session_window_s
    scored_wall_user_rate_Bps = (
        round(whole_session_credited_bytes / whole_session_window_s, 1)
        if (whole_session_window_s is not None
            and whole_session_window_s > 0
            and not instrument_invalid) else None)
    score_rx_for_math = whole_session_credited_bytes
    score_good_for_math = whole_session_credited_bytes
    if args.fixed_window_score:
        if connect_issued_at is None:
            instrument_invalid = True
            instrument_invalid_reasons.append("connect_request_not_issued")
            fixed_window_score = {
                "clock": "monotonic",
                "instrument_invalid": True,
                "instrument_invalid_reasons": ["connect_request_not_issued"],
                "ux": {"status": "NO_CONNECT_REQUEST"},
                "steady": {"status": "NO_CONNECT_REQUEST"},
            }
        else:
            fixed_window_score = score_fixed_windows(
                request_at_s=connect_issued_at,
                connected_at_by_peer=connected_at_by_peer,
                observed_until_s=observed_until_s,
                delivery_events=oracle_snapshot["events"],
                link_events=st.link_events.snapshot(),
                payload_target=args.payload,
                # runway-exhaustion detection wants the COUNT crossing (any
                # crossing means the cell stopped being loaded), not the
                # honest content completion.
                completion_at_s=completion["count_at_s"],
                payload_exhausted_at_s=input_exhaustion["at_s"],
                horizon_s=args.score_horizon_s,
                steady_warmup_s=args.steady_warmup_s,
            )
            # This sustained-load fixed-window instrument predates the one-message
            # contract and normally calls an early completed payload "too small".
            # Retain it as a diagnostic only; it must never void the primary cell.
            fixed_window_score["diagnostic_only"] = True
    configs_sorted = sorted(st.configs_seen)
    # max_config_reached = the highest config the gearshift CLIMBED to. ROBUST
    # IDs (100/101/102) are the MFSK floor; WB OFDM IDs are 0..16. A climb past
    # the ROBUST floor (any id < 100 appearing, or id>100 within ROBUST) is THE
    # signal. We report both the raw set and the max, plus a climbed-past-floor
    # boolean for quick A/B reading.
    wb_seen = [c for c in configs_sorted if c < 100]
    # NOTE: config ids are NOT ordered by capability. Robust MFSK tiers are 100/101/102
    # and sit BELOW every WB rung (0..17). max() over the mixed set therefore returns 102
    # whenever robust is used -- i.e. on every real run -- and is blind to the WB ceiling.
    # `max_wb_config_reached` is the number everyone actually means.
    max_config = max(configs_sorted) if configs_sorted else None          # raw max; MIXED ids
    max_wb_config = max(wb_seen) if wb_seen else None                     # the real WB ceiling
    climbed_past_robust0 = bool(wb_seen) or any(c > 100 for c in configs_sorted)

    # ---- capstone ext (c): ANATOMY line-items (PRIMARY scoreboard) ---------
    markers = CA.count_markers(logpath)
    _demote_stats = CA.count_real_demotes(logpath)   # one pass, not two
    in_burst_bps, active_frac, active_polls, total_polls, batch_sizes = CA.compute_inburst(timeline)
    # FIX 1 (metric-correction 2026-07-03): TRUE forward PTT-airtime duty from the
    # commander [CMD-TX]->[TX-END] brackets in the combined log. This is the honest
    # duty active_fraction was mis-reading as ~0.088 (batch-landing rate). None when
    # the cell had no bracketed forward TX (ROBUST-only / 0-delivery / truncated log).
    # Parse against the T+ clock and expose CONNECT->endpoint duty explicitly.
    # ptt_duty_steady is the desired whole-session denominator here because its
    # floor is CONNECT issued (not process launch or connection completion).
    ptt = CA.compute_ptt_duty(
        logpath,
        max(1.0, whole_session_end_s),
        connect_issued_at)
    # FIX 2: TRUE reverse-ACK arrival distribution (ms from end-of-TX to ACK landing),
    # per batch, harvested from the arrival_ms=/elapsed= the commander prints on every
    # data-ACK detect. ack_arrival_stats -> median/mean/p10/p90/max/n.
    ack_rows, ack_arrivals_sorted = CA.collect_ack_arrivals(logpath)
    ack_arrival = CA.ack_arrival_stats(ack_arrivals_sorted)
    # listen-guard ms (BUDGET): the CONFIGURED receiving_timeout the commander is
    # ALLOWED to wait, printed per batch (arq_commander.cc:1949). This is a CONSTANT
    # (~3524 ms on a pinned rung) — it is NOT the time actually spent listening. Kept
    # as the budget; the ACTUAL guard is listen_guard_ms_actual below.
    lw = list(st.listen_windows)
    listen_guard_ms_total = int(sum(lw)) if lw else 0
    listen_guard_ms_budget_mean = round(sum(lw) / len(lw), 1) if lw else None
    listen_guard_ms_mean = listen_guard_ms_budget_mean   # back-compat alias (BUDGET)
    listen_guard_events = len(lw)
    # FIX 3: ACTUAL listen guard = the real reverse-ACK wait. When an ACK landed the
    # wait IS its arrival_ms; a window that truly expired without an ACK waited the
    # full budget (counted by markers["rx_timeout"], the real no-ACK misses). We
    # report the arrival-based actual mean (the dominant term) + the real miss count.
    listen_guard_ms_actual_mean = ack_arrival["mean"] if ack_arrival else None
    listen_guard_ms_actual_median = ack_arrival["median"] if ack_arrival else None
    real_reverse_ack_misses = markers.get("rx_timeout", 0)
    # NOTE (metric-correction 2026-07-03): active_frac is BATCH-LANDING RATE, not
    # a data-airtime fraction (RX counter is a per-batch step function, so it reads
    # ~9x too low). listen_guard_frac_est below is DERIVED from it (1-active_frac)
    # and is therefore an ARTIFACT (~0.91) -- it is NOT the real idle fraction. The
    # honest datalink duty/efficiency is datalink_eff_* / data_airtime_frac emitted
    # below (~0.77). Kept only for back-compat with old cohort JSONs; do not gate on it.
    listen_guard_frac_est = round(1.0 - active_frac, 3) if active_frac is not None else None
    win_ms = None
    if timeline and len(timeline) >= 2:
        win_ms = int((timeline[-1][0] - timeline[0][0]) * 1000)
    listen_guard_ms_est = (int(listen_guard_frac_est * win_ms)
                           if (listen_guard_frac_est is not None and win_ms) else None)

    # ---- capstone ext (d): TWO-TRAFFIC VARA bar + delivered B/min ----------
    # delivered_Bmin = the bytes the harness read back off the RX data socket per
    # minute. For the COMPRESSIBLE arm those are the ORIGINAL decompressed content
    # bytes (mercury -F on compressed on the wire; the RX decompressed before it
    # delivered — arq_common.cc:13784) => the REAL delivered-content rate, with NO
    # synthetic multiplier (Mercury's own PPMd+dict ratio is already baked in). For
    # the INCOMPRESSIBLE arm compression is off so delivered == wire == content.
    #   * compressible  -> scored vs the VARA CLIENT bar (wire * per-corpus MEASURED
    #                      LZHUF ratio) = VARA's delivered original-content rate.
    #   * incompressible-> scored vs the VARA WIRE bar (LZHUF no-ops; wire==content),
    #                      removing the old double penalty.
    # Only WGN has a hardware-validated external-SNR map. Faded profiles still
    # require the documented long-window calibration; absent an explicit
    # coordinate, attest their channel structurally but disable VARA scoring.
    # The bridge is the impairment producer and therefore the only coordinate
    # producer. --snr3k is an assertion against its durable evidence, never an
    # independent replacement for that evidence.
    expected_cell = f"{args.profile.upper()}:{args.snr3k:g}"
    snr3k, channel_attestation = attested_snr3k(
        bridge_stats_path,
        expected_cell=expected_cell,
        expected_profile=args.profile,
        expected_seed=args.seed,
        expected_passthrough=args.passthrough,
        requested_snr3k=args.snr3k,
    )
    if not channel_attestation["valid"]:
        instrument_invalid = True
        instrument_invalid_reasons.extend(channel_attestation["reasons"])
        sys.stderr.write(
            "\n[CHANNEL-ATTESTATION INVALID] tag=%s: %s; requested=%r "
            "bridge=%r. VARA scoring disabled.\n\n" % (
                args.tag, ",".join(channel_attestation["reasons"]), args.snr3k,
                channel_attestation.get("realized_snr3k_db")))
        sys.stderr.flush()
    if instrument_invalid and channel_attestation["valid"]:
        channel_attestation["valid"] = False
        channel_attestation["instrument_invalid"] = True
        channel_attestation["vara_scorable"] = False
        channel_attestation["vara_scoring_disabled_reason"] = "runner_precondition_invalid"
        channel_attestation["reasons"].append("runner_precondition_invalid")
        snr3k = None
    compressible = CA.is_compressible_traffic(args.traffic)
    scoring_attested = bool(
        channel_attestation["vara_scorable"] and not instrument_invalid)
    if scoring_attested:
        vara_bar_Bmin, vara_bar_kind, corpus_lzhuf = CA.vara_bar_for_traffic(
            snr3k, args.traffic)
        vara_client = CA.vara_bar(snr3k)
        vara_wire_Bmin = CA.vara_wire(snr3k)
    else:
        vara_bar_Bmin = vara_bar_kind = corpus_lzhuf = None
        vara_client = vara_wire_Bmin = None
    primary_integrity_ok = (integ["mismatch_bytes"] == 0)
    primary_uniqueness_ok = (res["rx"] <= res["tx"])
    whole_session_scorable = bool(
        not instrument_invalid
        and primary_integrity_ok
        and primary_uniqueness_ok)
    scored_wall_user_rate_Bps = (
        round(whole_session_credited_bytes / whole_session_window_s, 1)
        if (whole_session_scorable
            and whole_session_window_s is not None
            and whole_session_window_s > 0) else None)
    mult = CA.user_multiplier(args.traffic)   # DEPRECATED (== 1.0); kept for good_prefix compat
    delivered_Bmin = (
        round(score_rx_for_math / scored_window_seconds * 60.0, 1)
        if (whole_session_scorable and scored_window_seconds) else None
    )   # scored-window delivered content bytes/min
    # C4 (metric-correction 2026-07-04): res["rx"] is the POST-DECOMPRESSION USER
    # bytes off the RX socket, so delivered_Bmin is the USER-CONTENT rate — it
    # equals the on-air WIRE rate ONLY for incompressible (-F off) traffic. For
    # compressible traffic it is inflated by mercury's compression ratio; presenting
    # it as "wire" produced the 109,409 B/min artifact (>3x the cfg16 wire cap).
    # The honestly-named field is user_content_Bmin.
    user_content_Bmin = delivered_Bmin        # HONEST name (post-decompression socket rate)
    delivered_user_Bmin = delivered_Bmin      # NO synthetic multiplier (mercury really compressed)
    # -- MODELED wire-capacity rate (renamed 2026-07-31; was "true_wire_keyed_Bmin").
    # The NUMERATOR is a MODEL: sum over configs of ([CMD-TX]->[TX-END] airtime x
    # CONFIG_NET_BPS[cfg]/8) — what the table says the keyed airtime COULD carry —
    # divided by the MEASURED warm wall. That mix is neither a keyed rate (wall
    # denominator) nor a measured wire rate (modeled numerator); under
    # retransmission it counts re-sent capacity a user byte never fills, so it can
    # read BELOW the measured whole-transfer rate (P2 fixture proof: whole 328 B/s
    # vs "true wire keyed" 274 B/s on byte-clean cfg15 cells — a numerator swap,
    # 1.1945 == 262144/keyed_bytes). Kept ONLY as a labeled capacity model + the
    # impossibility tripwire below; NEVER a scoreboard rate. The MEASURED rates
    # for scoring are keyed_user_rate_Bps and wall_user_rate_Bps (emitted below).
    modeled_wire_capacity_Bmin, modeled_wire_bytes = CA.true_wire_keyed_Bmin(
        ptt, whole_session_window_s or dwell)
    _keyed_bytes_val, modeled_wire_by_config = CA.keyed_wire_bytes(ptt)
    # -- IMPOSSIBILITY GUARD (C4): a rate above the fastest-config wire cap is a bug --
    wire_cap_Bmin = CA.max_wire_cap_Bmin(configs_sorted)   # net_bps(max WB cfg)/8*60; None if ROBUST-only
    # For INCOMPRESSIBLE traffic wire==content, so a user-content rate over the cap is
    # itself impossible; the modeled capacity rate is impossible over the cap for ANY
    # traffic (it is airtime-bounded by construction, so this only fires on an
    # accounting bug). Flag EITHER at the source so a future 109k number is caught here.
    rate_impossible = (
        CA.flag_impossible_rate(modeled_wire_capacity_Bmin, wire_cap_Bmin)
        or (not compressible and CA.flag_impossible_rate(delivered_user_Bmin, wire_cap_Bmin)))
    if rate_impossible:
        sys.stderr.write(
            f"\n[IMPOSSIBLE-RATE] tag={args.tag} arm={args.arm}: reported rate exceeds "
            f"the physical wire cap {wire_cap_Bmin} B/min (max cfg={max_config}). "
            f"modeled_wire_capacity={modeled_wire_capacity_Bmin} user_content={delivered_user_Bmin} "
            f"compressible={compressible}. This is a METRIC BUG (decompressed-content "
            f"mislabeled 'wire' — the 109k-class artifact). DO NOT score this cell's "
            f"throughput as wire.\n\n")
        sys.stderr.flush()
    vs_vara = (
        round(delivered_user_Bmin / vara_bar_Bmin, 4)
        if (vara_bar_Bmin and delivered_user_Bmin is not None) else None)

    # ---- C5: cfg17 HELD-vs-DEMOTED + live measure_variance -----------------
    # target_config = the pinned/attempted WB config (start-cfg when <100; ROBUST
    # start -> None = held-question not applicable). config_held answers the
    # endpoint-floor cfg17-held-vs-demoted-to-cfg16 question the old sweep left open
    # (it recorded only decode16=1.0). measure_variance ties the recovered SNR to the
    # same variance metric that once read 14.4 dB.
    target_config = (args.start_cfg if 0 <= args.start_cfg < 100 else None)
    config_timeline = CA.parse_config_timeline(logpath)
    # config_held filters the config timeline (t0/cold clock) by connect time, so it
    # takes the COLD connect (same clock as the timeline), not the warm headline. For a
    # PINNED session (--no-gearshift + a WB --start-cfg) the target config is loaded ONCE
    # during warm-up -- under --warm-start that is BEFORE the cold connect completes -- so
    # the connect floor would filter that load out and read held=False on a genuinely-held
    # cell (it rejected 18/18 byte-exact cells). pinned drops the floor for pinned cells;
    # a real demote after reaching target still fails held, and gearshift is untouched.
    pinned = bool(args.no_gearshift) and (target_config is not None)
    config_held = CA.config_held_verdict(config_timeline, target_config,
                                         connected_at_cold, pinned=pinned)
    # -- GEOMETRY + ELECTION FIRE-PROOF (records the RUNTIME pilot lattice + cfg17
    #    election so a silently-STOCK / silently-cfg16 cell is detectable in audit) --
    pilot_geometry = CA.parse_pilot_geometry(logpath)
    topgear_election = CA.parse_topgear_election(logpath)
    measure_variance = CA.parse_measure_variance(logpath)

    # ---- PRIMARY whole-session score + explicitly secondary diagnostics ----
    def _win_bps(_n, _s):
        return round(_n * 8.0 / _s, 1) if (_n and _s and _s > 0) else 0.0

    def _win_bmin(_n, _s):
        return round(_n * 60.0 / _s, 1) if (_n and _s and _s > 0) else 0.0

    if not whole_session_scorable:
        if instrument_invalid:
            whole_session_status = "INSTRUMENT_INVALID"
        elif not primary_integrity_ok:
            whole_session_status = "INTEGRITY_FAIL"
        else:
            whole_session_status = "UNIQUENESS_FAIL"
    elif whole_session_completed_prelim and terminal_settlement_s is None:
        whole_session_status = "SETTLEMENT_DEADLINE"
    elif whole_session_completed_prelim:
        whole_session_status = "COMPLETE"
    elif connected_at_cold is None:
        whole_session_status = "CONNECT_DEADLINE"
    else:
        whole_session_status = "TRANSFER_DEADLINE"

    _ss_secs = steady_window_seconds
    windows = {
        "primary_whole_session": {
            "label": "PRIMARY contract: one message, CONNECT issued -> byte-valid completion/common deadline",
            "scorable": whole_session_scorable,
            "clock_start": "CONNECT_issued",
            "deadline_horizon_s": (
                round(run_end_s - connect_issued_at, 3)
                if (run_end_s is not None and connect_issued_at is not None)
                else None),
            "status": whole_session_status,
            "connected": connected_at_cold is not None,
            "completed": whole_session_completed_prelim,
            "byte_complete_s": completion["at_s"],
            "disconnect_issued_s": disconnect_issued_at_s,
            "terminal_settlement_s": terminal_settlement_s,
            "payload_bytes": args.payload,
            "observed_good_prefix_bytes": oracle_snapshot["good_prefix_bytes"],
            "credited_bytes": whole_session_credited_bytes,
            "window_s": (
                round(whole_session_window_s, 3)
                if whole_session_window_s is not None else None),
            "rate_Bps": scored_wall_user_rate_Bps,
            "content_Bmin": delivered_Bmin,
        },
        "steady_state": {
            "label": "SECONDARY diagnostic: post-warm-gate suffix of the same one-message transfer",
            "scorable": False,
            "diagnostic_available": _ss_secs is not None and not under_warmed,
            "window_s": round(_ss_secs, 1) if _ss_secs is not None else None,
            "bps": (_win_bps(steady_rx_bytes or 0, _ss_secs)
                    if _ss_secs is not None else None),
            "content_Bmin": (_win_bmin(steady_good_prefix_bytes or 0, _ss_secs)
                             if _ss_secs is not None else None),
            "rx_bytes": steady_rx_bytes,
            "good_prefix_bytes": steady_good_prefix_bytes,
            "break_events": steady_breaks,
        },
        "whole_transfer_cold": {
            "label": "legacy diagnostic total (launch->end; NOT SCORED)",
            "scorable": False,
            "window_s": round(dwell_cold, 1),
            "bps": _win_bps(res["rx"], dwell_cold),
            "content_Bmin": _win_bmin(res["rx"], dwell_cold),
        },
        "whole_transfer_warm": {
            "label": "legacy diagnostic total (boot skipped; NOT SCORED)",
            "scorable": False,
            "window_s": round(dwell, 1),
            "bps": _win_bps(res["rx"], dwell),
            "content_Bmin": _win_bmin(res["rx"], dwell),
        },
    }
    if fixed_window_score is not None:
        windows["canonical_fixed"] = fixed_window_score
    # decode17 / decode-at-target: byte-faithful delivery WHILE holding the target
    # config. For a pinned cfg17 cell this is the honest "cfg17 decoded" signal the
    # sweep conflated with decode16 (a demote to cfg16 makes config_held False, so
    # decode_at_target is False even if bytes were faithful at cfg16).
    decode_at_target = bool(config_held.get("config_held")) and (integ["mismatch_bytes"] == 0) and (score_rx_for_math > 0)

    # ---- capstone ext (b): byte-integrity verdict --------------------------
    # HARD-FAIL on any corrupted delivered byte. (Truncation vs the payload
    # CEILING is normal for a time-bounded cell and is reported separately as
    # delivered_full - it is NOT an integrity failure. A HOLE/reorder in the
    # delivered stream DOES show as a pattern mismatch and IS caught here.)
    byte_integrity_ok = (integ["mismatch_bytes"] == 0)
    if not byte_integrity_ok and integ.get("cap"):
        # Dump the first mismatched (base, delivered, expected) chunks so the
        # per-byte transform f can be identified offline (root-cause capture).
        try:
            cappath = os.path.join(args.logdir, f"capbytes_{args.tag}.txt")
            with open(cappath, "w") as cf:
                cf.write(f"tag={args.tag} first_bad={integ['first_mismatch_off']} "
                         f"rx={res['rx']} mismatch={integ['mismatch_bytes']} "
                         f"segs={integ['mismatch_segments']}\n")
                for (b, dv, ev) in integ["cap"]:
                    cf.write(f"BASE {b} LEN {len(dv)}\n")
                    cf.write(f"DELIV {dv.hex()}\n")
                    cf.write(f"EXPEC {ev.hex()}\n")
        except OSError:
            pass
    if not byte_integrity_ok:
        sys.stderr.write(
            f"\n[BYTE-INTEGRITY FAIL] tag={args.tag} arm={args.arm}: "
            f"{integ['mismatch_bytes']} corrupted byte(s) in "
            f"{integ['mismatch_segments']} segment(s); first bad offset="
            f"{integ['first_mismatch_off']} of {res['rx']} delivered. "
            f"CELL HARD-FAILED.\n\n")
        sys.stderr.flush()

    # ---- capstone STEP 2c: delivered-byte UNIQUENESS gate ------------------
    # No double-delivery may inflate B/min. TWO independent assertions, BOTH
    # hard-gate the cell (either -> void):
    #   (1) CONTENT (byte_integrity_ok above): the delivered stream must match the
    #       NON-PERIODIC canonical pattern, so ANY hole/reorder/duplicate — of any
    #       size, including a period-256-aligned re-delivered batch the old
    #       periodic pattern was blind to — lands a wrong block index and is caught.
    #   (2) COUNT (here): delivered bytes may never exceed fed bytes (rx <= tx). A
    #       gross re-delivery that pushes the delivered total past everything TX
    #       ever handed the socket is caught even if it were pattern-aligned.
    uniqueness_ok = res["rx"] <= res["tx"]
    if not uniqueness_ok:
        sys.stderr.write(
            f"\n[UNIQUENESS FAIL] tag={args.tag} arm={args.arm}: delivered "
            f"rx_bytes={res['rx']} EXCEEDS fed tx_bytes={res['tx']} — "
            f"double-delivery inflating B/min. CELL HARD-FAILED.\n\n")
        sys.stderr.flush()

    # ---- HONEST COMPLETION (field-level instrument fix, 2026-07-31) --------
    # delivered_full was COUNT-ONLY (rx >= payload): it silently passed a
    # stream that kept byte-count parity while content broke (a shear: bytes
    # deleted mid-stream, every later byte at the wrong offset). The oracle
    # above already catches that; this ANDs it into the completion field
    # itself so a consumer reading delivered_full ALONE can no longer count a
    # sheared cell as complete. The legacy count-only value survives under an
    # explicit name so old and new cells stay distinguishable in audits.
    delivered_full_count_only = bool(
        scored_rx_bytes is not None and scored_rx_bytes >= args.payload)
    delivered_full = bool(
        delivered_full_count_only
        and completion["at_s"] is not None
        and (run_end_s is None or completion["at_s"] <= run_end_s)
        and byte_integrity_ok and uniqueness_ok)
    # Shear signature: content broken while the byte count still reached the
    # payload target — exactly the case the count-only field silently passed.
    content_shear_suspected = bool(
        delivered_full_count_only and not byte_integrity_ok)
    if content_shear_suspected:
        sys.stderr.write(
            f"\n[CONTENT-SHEAR] tag={args.tag} arm={args.arm}: byte count "
            f"reached the payload target ({scored_rx_bytes} >= {args.payload}) but "
            f"content is broken (first bad offset="
            f"{integ['first_mismatch_off']}). The count-only completion field "
            f"would have silently passed this cell.\n\n")
        sys.stderr.flush()
    # Durable content-equality record for offline recompute: md5 over the
    # delivered stream vs md5 over the expected source prefix of equal
    # length. Must agree with the oracle verdict (byte_integrity_ok); a
    # divergence is an instrument bug in the harness itself.
    content_delivered_md5 = integ["rx_md5"].hexdigest()
    _exp_md5 = hashlib.md5()
    _exp_off = 0
    while _exp_off < res["rx"]:
        _exp_n = min(1 << 20, res["rx"] - _exp_off)
        _exp_md5.update(
            CA.traffic_expected_slice(args.traffic, _exp_off, _exp_n))
        _exp_off += _exp_n
    content_expected_md5 = _exp_md5.hexdigest()
    content_md5_ok = (content_delivered_md5 == content_expected_md5)
    if content_md5_ok != byte_integrity_ok:
        sys.stderr.write(
            f"\n[INSTRUMENT-BUG] tag={args.tag}: content md5 verdict "
            f"({content_md5_ok}) disagrees with the streaming oracle "
            f"({byte_integrity_ok}) — the harness itself is broken.\n\n")
        sys.stderr.flush()

    # GOOD-PREFIX bytes (capstone STEP 2c, honest throughput): the length of the
    # correct contiguous NON-PERIODIC prefix = the useful IN-ORDER user bytes the
    # app received before the FIRST content violation. For a clean cell this is
    # rx_bytes; for a cell that re-delivered a batch during retransmission (the
    # double-delivery this pattern uniquely catches) it is first_bad_offset — the
    # bytes past that point are duplicates/garbage the app cannot trust, so they
    # must NOT count toward throughput. Scoring on good_prefix uses EVERY connected
    # cell (no survivorship bias from voiding the retx-hit realizations) while never
    # crediting the inflation.
    good_prefix_bytes = score_good_for_math
    # C4: good_prefix_bytes are POST-DECOMPRESSION user bytes -> this is the
    # good-prefix USER-CONTENT rate (== wire only for incompressible -F off), NOT a
    # wire rate. Honest name = good_prefix_user_content_Bmin. good_prefix_wire_Bmin
    # is retained below ONLY as a DEPRECATED alias (the old misnomer).
    good_prefix_user_content_Bmin = (
        round(good_prefix_bytes / scored_window_seconds * 60.0, 1)
        if scored_window_seconds else None
    )
    good_prefix_wire_Bmin = good_prefix_user_content_Bmin   # DEPRECATED misnomer alias (== user_content)
    good_prefix_user_Bmin = (
        round(good_prefix_user_content_Bmin * mult, 1)
        if good_prefix_user_content_Bmin is not None else None
    )

    # ---- HONEST datalink efficiency / data-airtime fraction (metric-correction
    # 2026-07-03). This REPLACES active_fraction as the duty/efficiency figure.
    #   datalink_eff_delivered = delivered good-prefix wire bytes / (full wall x
    #       net_bps(delivering config)) -- the fraction of the config's WIRE
    #       capacity actually delivered, net of ALL overhead (guard/ACK/turnaround/
    #       retx/preamble/connect). This is the number active_fraction was meant to
    #       be; active_fraction read ~9x too low (batch-landing rate artifact).
    #   datalink_eff_steady = same over CONNECT-issued -> endpoint (the primary
    #       whole-session denominator, including connect establishment).
    # Keyed on max_config (the delivering WB config); None if only ROBUST/unknown.
    eff_full, eff_steady, eff_net_bps = CA.datalink_efficiency(
        good_prefix_bytes, max(1.0, whole_session_end_s),
        connect_issued_at, max_config)

    # ---- GEOMETRY GUARD (metric-correction 2026-07-12) ----------------------
    # CONFIG_NET_BPS (and every meter derived from it: datalink_eff_*,
    # modeled_wire_capacity_Bmin, wire_cap_Bmin, eff_net_bps) is a HARDCODED table captured
    # at the STOCK frame geometry (runtime Ngi=36). It is BLIND to a per-run CP/pilot
    # geometry override (MERCURY_SE_{NGI,DY,NSYMB} escape hatch or a reclaim rung):
    # under an override it reports the STOCK rate while the real config differs
    # (cfg15 3348.4 vs ~3826.7 bps, +14.3%). geom flags whether the table is valid for
    # THIS run; bytes_per_fwd_airtime is the geometry-INDEPENDENT trusted goodput.
    geom = CA.geometry_override_flags(logpath, list(arm_env_kv) + list(args.env))
    fwd_airtime_s = (ptt or {}).get("fwd_airtime_s")
    bytes_per_fwd_air = CA.bytes_per_fwd_airtime(good_prefix_bytes, fwd_airtime_s)
    if geom["net_bps_geometry_blind"]:
        sys.stderr.write(
            f"\n[GEOMETRY-BLIND] tag={args.tag} arm={args.arm}: a frame-geometry override "
            f"was active (runtime Ngi={geom['runtime_ngi']} vs stock {geom['stock_ngi']}; "
            f"se_env_injected={geom['se_env_injected']}). The CONFIG_NET_BPS table is stock-only, "
            f"so net_bps/datalink_eff_*/modeled_wire_capacity_Bmin/wire_cap_Bmin are NOT trustworthy for "
            f"this run. Use delivered_Bmin and bytes_per_fwd_airtime "
            f"({bytes_per_fwd_air} B/s) instead.\n\n")
        sys.stderr.flush()
    # (The stock-geometry consistency assertion — Ngi==36 and a monotonic, self-
    # consistent CONFIG_NET_BPS table — is deterministic, so it lives in test_meters.py
    # rather than the per-run path where compressible/retx cells would false-alarm any
    # keyed-wire vs delivered-goodput comparison.)
    # retx corroboration (anti-fabrication): a second, independent number (requeued
    # frames) from the SAME MIXBATCH-RETX lines that source retx_rounds, so a future
    # printf rename that silently zeroes retx_rounds is visible against this field.
    _retx = CA.count_retx(logpath)

    # ---- capstone STEP 2a: per-lever engagement (summed across CMD+RSP) -----
    engagement = CA.empty_engage()
    for _lbl, _eng in st.engage_by_label.items():
        CA.add_engage(engagement, _eng)
    engage_emitted_by = sorted(st.engage_by_label.keys())

    # AXIS provenance: the noise-calibration (P_sig) mode the channel ACTUALLY
    # ran, read from durable bridge evidence where available (falling back to
    # the env this process passed the bridge). Stamped so the scorer can refuse
    # a hot / unknown axis against the VARA bar (instrument #16).
    psig_mode, psig_mode_source = _resolve_psig_mode(
        bridge_stats_path,
        os.path.join(args.logdir, f"bridge_{args.tag}.log"),
        args.passthrough)
    # Fork-B audio-path integrity (load-invariance / #17 width diagnostic): the
    # per-direction snd-aloop underruns the bridge recorded, surfaced from the
    # same statsfile attested_snr3k reads for psig -- so a stormed cell can be
    # graded clean-audio vs degraded-audio.
    bridge_underruns = _read_bridge_underruns(bridge_stats_path)

    # Bench-twin attestation is fail-closed: the bridge must echo the selected
    # fixed-noise reference and the exact bridge/harness identities.
    try:
        bridge_ref_check = bridge_reference.verify(
            bridge_stats_path, bridge_ref_requested,
            passthrough=args.passthrough)
        if not bridge_ref_check["ok"]:
            raise ValueError("bridge noise reference not applied: "
                             + ",".join(bridge_ref_check["reasons"]))
        with open(bridge_stats_path, encoding="utf-8") as bridge_stats_stream:
            bridge_stats_doc = json.load(bridge_stats_stream)
        bridge_axis_attestation = bridge_stats_doc.get("axis_attestation", {})
        if bridge_ident_argv:
            ident_reasons = HA.verify_identity(
                bridge_axis_attestation, harness_ident)
            if ident_reasons:
                raise ValueError("bridge/harness identity not attested: "
                                 + ",".join(ident_reasons))
    except (OSError, ValueError, TypeError) as exc:
        sys.stderr.write("[harness] FATAL bench-twin attestation rejected: %s\n" % exc)
        return 2
    harness_attestation = dict(
        harness_ident, echoed_by_bridge=bool(bridge_ident_argv))

    try:
        with open(logpath, "r", encoding="utf-8", errors="replace") as fh:
            tx_gain_attestation = HA.check_tx_gain_log(
                fh.read(), tx_gain_values)
    except OSError as exc:
        tx_gain_attestation = {
            "mode": "ini" if tx_gain_values else "default",
            "ok": False,
            "reasons": ["log_unreadable:%s" % exc],
        }
    tx_gain_attestation["ini_path"] = tx_gain_ini
    tx_gain_attestation["ini_source"] = tx_gain_ini_source
    tx_gain_attestation["settings_files"] = tx_gain_settings
    if tx_gain_values and not tx_gain_attestation["ok"]:
        sys.stderr.write("[harness] FATAL tx gain override not applied: %s\n"
                         % ",".join(tx_gain_attestation["reasons"]))
        return 2

    result = {
        "tag": args.tag, "arm": args.arm, "env": args.env,
        **binary_fingerprint,
        "binary_sha256": binary_sha256,
        # -- arm spec (capstone ext a) --
        "arm_spec": arm_spec,
        "arm_env_injected": arm_env_kv,
        "arm_lever_warnings": arm_warnings,     # compile-time/absent/rider levers
        "arm_inert_levers": inert_levers,       # requested runtime levers absent from THIS binary
        "bin_lever_probe": probe,
        "traffic": args.traffic,
        "passthrough": args.passthrough,
        "snr": None, "snr3k_controlled": args.snr3k,
        "axis_version": "v1-steady-snr3k", "cell": args.cell,
        "profile": args.profile,
        "seed": args.seed, "start_cfg": args.start_cfg,
        "erase_a2b_burst": args.erase_a2b_burst,
        "erase_b2a_burst": args.erase_b2a_burst,
        "mode": args.mode,
        "card": args.card, "subs": subs,
        "rsp_port": rsp_port, "cmd_port": cmd_port,
        "no_gearshift": args.no_gearshift,
        "wire_stamp": args.wire_stamp,
        "modem_argv": {
            "CMD": mercury_cmd(cmd_port, cmd_rx, cmd_tx),
            "RSP": mercury_cmd(rsp_port, rsp_rx, rsp_tx),
        },
        # Headline connection means a simultaneous two-peer session was
        # established for this transfer, not that both peers remain connected
        # at teardown.
        "connected": connected_at_cold is not None,
        # -- instrument #11 legibility markers (classification only) --
        "connect_fail_reason": connect_fail_reason,
        "last_ToSend_data": st.last_ToSend_data,
        "instrument_invalid": instrument_invalid,
        "instrument_invalid_reasons": sorted(set(instrument_invalid_reasons)),
        "teardown_survivor_pids": teardown_survivor_pids,
        "teardown_post_grace_reap_required": bool(teardown_survivor_pids),
        "cmd_connected": st.cmd_connected, "rsp_connected": st.rsp_connected,
        "connected_at_s": round(connected_at, 2) if connected_at is not None else None,
        "connected_at_by_peer_cold_s": {
            peer: round(at_s, 6)
            for peer, at_s in sorted(connected_at_by_peer.items())
        },
        "first_connected_at_by_peer_cold_s": {
            peer: round(at_s, 6)
            for peer, at_s in sorted(st.first_connected_at_by_peer.items())
        },
        # -- WARM-START honest-metric fields (Lever A) --
        # warm_start: was the transfer clock started warm? connected_at_s above is
        # the WARM connect (from t0+warm_offset); connected_at_cold_s is the legacy
        # cold connect (from process launch) for the fail-before comparison.
        "warm_start": bool(args.warm_start),
        "warm_offset_s": round(warm_offset, 2),
        "warm_at_s": {k: round(v, 2) for k, v in warm_snapshot.items()},
        # -- optional SECONDARY post-connect warm diagnostic --
        "warm_gate_enabled": bool(args.warm_gate),
        "warm_gate_constants": {
            **warm_gate_constants(),
            "floor_s": args.warm_floor_s,
            "ceiling_s": args.warm_ceiling_s,
            "sample_s": args.warm_sample_s,
            "rate_window_s": args.warm_rate_window_s,
            "stable_samples": args.warm_stable_samples,
            "rate_tolerance_fraction": args.warm_rate_tolerance,
            "max_tx_ahead_bytes": args.warm_max_tx_ahead_bytes,
        },
        "warm_seconds": warm_seconds,
        "under_warmed": bool(under_warmed),
        "scored_window_start": (
            round(scored_window_start, 6)
            if scored_window_start is not None else None
        ),
        "steady_window_start": (
            round(steady_window_start, 6)
            if steady_window_start is not None else None
        ),
        "scored_window_seconds": (
            round(scored_window_seconds, 3)
            if scored_window_seconds is not None else None
        ),
        "warm_confirmed_rate_Bps": (
            round(warm_confirmed_rate_Bps, 3)
            if warm_confirmed_rate_Bps is not None else None
        ),
        "warm_rx_bytes": warm_rx_bytes,
        "warm_tx_bytes": warm_tx_bytes,
        "warm_breaks": warm_breaks,
        "steady_rx_bytes": steady_rx_bytes,
        "steady_tx_bytes": steady_tx_bytes,
        "steady_good_prefix_bytes": steady_good_prefix_bytes,
        "steady_breaks": steady_breaks,
        "steady_window_seconds": (
            round(steady_window_seconds, 3)
            if steady_window_seconds is not None else None),
        # -- PRIMARY one-message whole-session score (flat reducer aliases) --
        "whole_session_status": whole_session_status,
        "whole_session_scorable": whole_session_scorable,
        "whole_session_window_s": (
            round(whole_session_window_s, 3)
            if whole_session_window_s is not None else None),
        "whole_session_good_prefix_bytes": oracle_snapshot["good_prefix_bytes"],
        "whole_session_credited_bytes": whole_session_credited_bytes,
        "whole_session_rate_Bps": scored_wall_user_rate_Bps,
        "whole_session_content_Bmin": delivered_Bmin,
        "scored_rx_bytes": scored_rx_bytes,
        "scored_tx_bytes": scored_tx_bytes,
        "scored_good_prefix_bytes": scored_good_prefix_bytes,
        "connected_at_cold_s": round(connected_at_cold, 2) if connected_at_cold is not None else None,
        "connect_issued_at_s": round(connect_issued_at, 6) if connect_issued_at is not None else None,
        "connect_onair_s": round(connect_onair_s, 2) if connect_onair_s is not None else None,
        "wall_secs_cold": round(dwell_cold, 1),
        "tx_bytes": scored_tx_bytes, "rx_bytes": scored_rx_bytes,
        "tx_bytes_total": res["tx"], "rx_bytes_total": res["rx"],
        "payload_target": args.payload,
        # HONEST completion (2026-07-31): count AND byte-exact content AND
        # uniqueness. The pre-fix count-only value is kept under an explicit
        # legacy name; a completion tally may read delivered_full ONLY.
        "delivered_full": delivered_full,
        "delivered_full_count_only": delivered_full_count_only,
        "content_shear_suspected": content_shear_suspected,
        "content_delivered_md5": content_delivered_md5,
        "content_expected_md5": content_expected_md5,
        "content_md5_ok": content_md5_ok,
        # completion_at_s = GOOD-PREFIX crossing (honest); the legacy count
        # crossing is retained separately for audit.
        "completion_at_s": completion["at_s"],
        "completion_count_at_s": completion["count_at_s"],
        # last delivered byte (endpoint of an incomplete cell, D84) and the last
        # instant the byte-exact prefix grew; harness clock, like completion_at_s.
        "last_rx_at_s": delivery_times["last_rx_at_s"],
        "last_good_prefix_advance_at_s": delivery_times["last_good_prefix_advance_at_s"],
        "delivery_event_count": delivery_times["delivery_event_count"],
        "rate_table_attestation": rate_table,
        "disconnect_issued_at_s": disconnect_issued_at_s,
        "terminal_settlement_s": terminal_settlement_s,
        "terminal_release_at_by_peer_s": dict(st.disconnected_at_by_peer),
        "terminal_release_source_by_peer": dict(st.disconnect_source_by_peer),
        "payload_exhausted_at_s": input_exhaustion["at_s"],
        "fixed_window_score_enabled": bool(args.fixed_window_score),
        "fixed_window_diagnostic_only": bool(args.fixed_window_score),
        "fixed_window_protocol_canonical": bool(
            args.fixed_window_score
            and args.score_horizon_s == 600.0
            and args.steady_warmup_s == 30.0),
        "canonical_fixed_score": fixed_window_score,
        # -- BYTE-INTEGRITY (capstone ext b) --
        "byte_integrity_ok": byte_integrity_ok,
        "integrity_mismatch_bytes": integ["mismatch_bytes"],
        "integrity_mismatch_segments": integ["mismatch_segments"],
        "integrity_first_bad_offset": integ["first_mismatch_off"],
        # -- UNIQUENESS (capstone STEP 2c) --
        "uniqueness_ok": uniqueness_ok,
        "delivered_exceeds_fed": res["rx"] > res["tx"],
        # -- GOOD-PREFIX honest throughput (double-delivery-robust) --
        "good_prefix_bytes": good_prefix_bytes,
        "good_prefix_user_content_Bmin": good_prefix_user_content_Bmin,  # C4 HONEST name (post-decompression)
        "good_prefix_wire_Bmin": good_prefix_wire_Bmin,   # DEPRECATED misnomer alias (== user_content, NOT wire)
        "good_prefix_user_Bmin": good_prefix_user_Bmin,
        # -- ENGAGEMENT (capstone STEP 2a; summed across CMD+RSP) --
        "engagement": engagement,
        "engage_emitted_by": engage_emitted_by,
        # -- ANATOMY line-items (capstone ext c; PRIMARY scoreboard) --
        "anatomy": {
            # ---- FIX 1: TRUE PTT-airtime duty (use THIS as the duty figure) ----
            # Primary duty uses the same CONNECT-issued -> completion/deadline
            # denominator as the whole-session score, including hail/ramp airtime.
            "ptt_duty": (ptt or {}).get("ptt_duty_steady"),
            "whole_session_ptt_duty": (ptt or {}).get("ptt_duty_steady"),
            "ptt_duty_launch_to_end": (ptt or {}).get("ptt_duty"),
            "ptt_duty_steady": (ptt or {}).get("ptt_duty_steady"),
            "ptt_fwd_airtime_s": (ptt or {}).get("fwd_airtime_s"),
            "ptt_n_tx_bursts": (ptt or {}).get("n_tx_bursts"),
            "ptt_duty_msgtx": (ptt or {}).get("ptt_duty_msgtx"),
            "ptt_rev_ack_sends": (ptt or {}).get("rev_ack_sends"),
            "ptt_configs_tx": (ptt or {}).get("configs_tx"),
            # ---- FIX 2: TRUE reverse-ACK arrival (ms from end-of-TX to ACK) ----
            "ack_arrival_ms": ack_arrival,                       # {median,mean,p10,p90,max,n}
            "ack_arrival_median_ms": (ack_arrival or {}).get("median"),
            "ack_arrival_p90_ms": (ack_arrival or {}).get("p90"),
            "ack_arrival_n": (ack_arrival or {}).get("n"),
            "ack_arrivals_sorted_ms": ack_arrivals_sorted,       # raw list (corpus feed)
            # ---- FIX 3: ACTUAL listen guard vs configured BUDGET ----
            "listen_guard_ms_actual_mean": listen_guard_ms_actual_mean,     # real wait to ACK
            "listen_guard_ms_actual_median": listen_guard_ms_actual_median,
            "real_reverse_ack_misses": real_reverse_ack_misses,            # true no-ACK count
            "listen_guard_ms_budget_mean": listen_guard_ms_budget_mean,    # CONFIGURED budget (constant)
            "listen_guard_ms_total": listen_guard_ms_total,
            "listen_guard_ms_mean": listen_guard_ms_mean,        # BACK-COMPAT alias = BUDGET (not actual)
            "listen_guard_events": listen_guard_events,
            "listen_guard_ms_est_from_idle": listen_guard_ms_est,
            "listen_guard_frac_est": listen_guard_frac_est,
            "retx_rounds": markers["retx_round"],
            "retx_frames_requeued": _retx["retx_frames_requeued"],   # Σ R=<n> from the same MIXBATCH-RETX lines (corroborates retx_rounds)
            # markers["demote"] retired: a substring cannot tell a climb from a demote.
            # Direction needs a capability-rank comparison over SET_CONFIG transitions.
            # A substring cannot tell a climb from a demote; direction needs capability rank
            # over the SET_CONFIG trajectory. See capstone_arms.count_real_demotes().
            "real_demotes": _demote_stats["real_demotes"],
            "config_climbs": _demote_stats["climbs"],
            "robust_drops": _demote_stats["robust_drops"],
            "demote_events_UNRELIABLE": markers.get("demote_lines", 0),
            "robust_drops": markers.get("robust_drop", 0),
            "lossless_rolls": markers.get("lossless_roll", 0),
            "break_events": scored_breaks,
            "break_events_total": markers["break_block"],
            "break_any": (bool(scored_breaks) if scored_breaks is not None else None),
            "break_any_total": markers["break_any"],
            "set_config_events": markers["set_config"],
            "rx_timeout_events": markers["rx_timeout"],              # FIX 3: now REAL reverse-ACK misses
            "rsp_rx_timeout_events": markers.get("rsp_rx_timeout", 0),
            "rx_timeout_budget_lines_BROKEN": markers.get("rx_timeout_budget_lines_BROKEN", 0),
            "inband_tag_events": markers["inband_tag"],
            "inband_nobreak_events": markers["inband_nobreak"],
            # HONEST duty/efficiency (use THESE, not active_fraction) --------
            "datalink_eff_delivered": eff_full,      # delivered good-prefix / wire capacity, full wall
            "datalink_eff_steady": eff_steady,       # same, post-connect window
            "eff_net_bps": eff_net_bps,              # net_bps of the delivering config (mercury -l)
            # active_fraction is BATCH-LANDING RATE (per-batch step-counter polled
            # at 1 Hz), ~9x LOWER than the true duty; in_burst_wire_bps is ~9x too
            # HIGH (same root). BROKEN as a duty figure; retained ONLY for back-compat
            # with old cohort JSONs. Use ptt_duty (FIX 1) / datalink_eff_* instead.
            "active_fraction_BROKEN": active_frac,
            # The bare unlabelled `active_fraction` key is RETIRED (2026-07-12): a naive
            # analysis that greps "active_fraction" would read this ~9x-deflated CADENCE as
            # a duty. Emitted ONLY under an explicit label naming what it measures. Use
            # ptt_duty_steady (FIX 1) / datalink_eff_* for duty; NEVER re-derive duty here.
            "active_fraction_CADENCE_NOT_DUTY": active_frac,   # batch-landing cadence, NOT airtime duty
            "active_polls": active_polls,
            "total_polls": total_polls,
            "in_burst_wire_bps": in_burst_bps,
            "num_batch_deliveries": len(batch_sizes),
            # C4 THROUGHPUT (honest labels, re-corrected 2026-07-31).
            # delivered_user_Bmin / user_content_Bmin are the MEASURED post-
            # decompression USER-CONTENT socket rate (== wire ONLY for
            # incompressible -F off). The old "true_wire_keyed_Bmin" is RETIRED
            # (a MODELED numerator — airtime x CONFIG_NET_BPS table — over the
            # MEASURED wall: neither keyed nor wire; under retransmission it
            # reads BELOW the measured whole rate). Its value survives ONLY as
            # modeled_wire_capacity_Bmin, a labeled capacity model and the
            # impossibility tripwire. MEASURED rates for scoring live below:
            # bytes_per_fwd_airtime (in-burst keyed goodput) and the top-level
            # keyed_user_rate_Bps / wall_user_rate_Bps.
            "delivered_user_Bmin": delivered_user_Bmin,   # post-decompression content rate
            "user_content_Bmin": user_content_Bmin,       # HONEST name for the old "wire_Bmin"
            "modeled_wire_capacity_Bmin": modeled_wire_capacity_Bmin,  # capacity MODEL, not a rate
            "modeled_wire_bytes": modeled_wire_bytes,     # MODELED capacity bytes (airtime x table)
            "modeled_wire_by_config": modeled_wire_by_config,  # per-config model split (transparency)
            "wire_cap_Bmin": wire_cap_Bmin,               # physical ceiling (max WB cfg net_bps/8*60)
            "rate_impossible": rate_impossible,           # True => a rate exceeded the wire cap (metric bug)
            "wire_Bmin": user_content_Bmin,               # DEPRECATED misnomer alias (== user_content, NOT wire)
            # ---- GEOMETRY GUARD (2026-07-12): trust-flags for the table-derived wire
            # meters above, + the geometry-INDEPENDENT trusted goodput. When
            # net_bps_geometry_blind is True, DO NOT trust net_bps/datalink_eff_*/
            # modeled_wire_capacity_Bmin/wire_cap_Bmin; use delivered_Bmin + bytes_per_fwd_airtime.
            "geometry_is_stock": geom["geometry_is_stock"],       # CONFIG_NET_BPS table valid for this run?
            "net_bps_geometry_blind": geom["net_bps_geometry_blind"],  # True => table wire meters untrustworthy
            "runtime_ngi": geom["runtime_ngi"],                   # runtime guard-interval Ngi ([36]=stock)
            "se_env_injected": geom["se_env_injected"],           # MERCURY_SE_* geometry override injected?
            "bytes_per_fwd_airtime": bytes_per_fwd_air,           # TRUSTED geometry-independent goodput (B/s of PTT airtime)
        },
        # -- AXIS + WIDTH provenance (instrument #16 / candidate #17) --
        # psig_mode = the noise-calibration axis the channel ACTUALLY ran;
        # psig_mode_source names where it was read (bridge_stats > PSIG_DIAG log
        # > env > env_default). spawner_width / box_concurrent_estimate are the
        # cohort-width fields the spawner stamps; None on a direct single-cell
        # run. The scorer refuses a vs-bar row that is hot/unknown-axis or
        # over-wide/width-unrecorded.
        "psig_mode": psig_mode,
        "psig_mode_source": psig_mode_source,
        "spawner_width": args.spawner_width,
        "box_concurrent_estimate": args.box_concurrent_estimate,
        # Fork-B audio-path integrity: per-direction snd-aloop underruns
        # (fwd/rev/total) from the bridge statsfile. None on a cell whose bridge
        # never flushed a statsfile. The load-invariance (#17 width) discriminator.
        "bridge_underruns": bridge_underruns,
        "bridge_reference_attestation": bridge_ref_check,
        "harness_attestation": harness_attestation,
        "tx_gain_attestation": tx_gain_attestation,
        # Sanctioned broker measurement-window this cell ran under (width>cap
        # diagnostic); None for a normal within-caps cell.
        "broker_override_window": args.broker_override_window,
        # -- TWO-TRAFFIC VARA bar (capstone ext d) --
        "snr3k": snr3k,
        "requested_snr3k": args.snr3k,
        "snr3k_attested": scoring_attested,
        "vara_scoring_enabled": scoring_attested,
        "channel_attestation": channel_attestation,
        "traffic_compressible": compressible,
        "force_compress": force_compress_flag,
        "vara_bar_Bmin": vara_bar_Bmin,        # the bar ACTUALLY scored against
        "vara_bar_kind": vara_bar_kind,        # "client" (compressible) | "wire" (incompressible)
        "corpus_lzhuf_ratio": corpus_lzhuf,    # per-corpus MEASURED LZHUF (client-bar multiplier)
        "vara_wire_Bmin": vara_wire_Bmin,      # reference WIRE bar
        "vara_client_Bmin": vara_client,       # reference legacy client table (wire*2.0907)
        "delivered_Bmin": delivered_Bmin,      # delivered USER-CONTENT bytes/min (post-decompression; == what vs_vara scored)
        "delivered_user_Bmin": delivered_user_Bmin,
        "user_content_Bmin": user_content_Bmin,        # C4 HONEST name (post-decompression socket rate)
        # MODELED capacity (2026-07-31; was "true_wire_keyed_Bmin"): airtime x
        # CONFIG_NET_BPS table over the measured wall. A capacity model and
        # impossibility tripwire ONLY — never a rate, never in a ratio.
        "modeled_wire_capacity_Bmin": modeled_wire_capacity_Bmin,
        "wire_cap_Bmin": wire_cap_Bmin,                # physical ceiling (max WB cfg)
        "rate_impossible": rate_impossible,            # True => reported rate exceeded the wire cap (metric bug)
        # GEOMETRY GUARD (2026-07-12): the table-derived wire meters (net_bps/
        # datalink_eff_*/modeled_wire_capacity_Bmin/wire_cap_Bmin) are STOCK-geometry-only.
        # When net_bps_geometry_blind is True a per-run CP/pilot override was active and
        # those meters are untrustworthy — score on delivered_Bmin + bytes_per_fwd_airtime.
        "geometry_is_stock": geom["geometry_is_stock"],
        "net_bps_geometry_blind": geom["net_bps_geometry_blind"],
        "bytes_per_fwd_airtime": bytes_per_fwd_air,     # TRUSTED geometry-independent goodput (B/s of PTT airtime)
        # -- CANONICAL MEASURED RATES (_research/CANONICAL_METRICS.md) --------
        # keyed_user_rate_Bps: in-burst keyed goodput = good-prefix user bytes
        #   per second of forward PTT airtime (== bytes_per_fwd_airtime).
        # wall_user_rate_Bps: THE whole-transfer contract metric = credited
        #   payload / CONNECT-issued -> valid completion/common deadline.
        # Reconstruction law: wall_user_rate == keyed_user_rate x
        #   (fwd_airtime / wall); ra_reduce.py checks it per cell.
        "keyed_user_rate_Bps": bytes_per_fwd_air,
        "wall_user_rate_Bps": scored_wall_user_rate_Bps,
        "scored_wall_user_rate_Bps": scored_wall_user_rate_Bps,
        "vs_vara": vs_vara,                    # delivered CONTENT vs the traffic-correct VARA bar (client|wire)
        # vs_vara_wire is RETIRED (2026-07-31): it divided the modeled capacity
        # by the measured VARA wire bar — a model-vs-measurement ratio that
        # misled the campaign. Whole-vs-whole scoring is vs_vara; wire-vs-wire
        # for incompressible traffic is wall_user_rate_Bps vs vara_wire_Bmin/60.
        "vara_source": CA.VARA_SOURCE,
        # -- legacy fields (unchanged) --
        "rsp_nreceived_frames": st.rsp_nreceived,
        "cmd_nreceived_frames": st.cmd_nreceived,
        "breaks": scored_breaks,
        "scored_breaks": scored_breaks,
        "breaks_total": st.breaks,
        "encrypt": args.encrypt,
        "enc_activated": st.enc_activated,
        "aead_authfails": st.authfails,
        "nonce_enc_count": sum(st.enc_nonces.values()),
        "nonce_reuse_count": st.nonce_reuse,
        "dec_ok_count": st.dec_ok,
        "configs_seen": configs_sorted,
        "max_config_reached": max_config,          # MIXED robust+WB ids; 102 whenever robust ran
        "max_wb_config_reached": max_wb_config,    # the WB ceiling actually climbed to
        "wb_configs_seen": wb_seen,
        "climbed_past_robust0": climbed_past_robust0,
        # -- C5: cfg17 HELD-vs-DEMOTED + live measure_variance --
        "target_config": target_config,             # pinned/attempted WB config (None if ROBUST/auto)
        # Provenance: WHICH BINARY produced this cell. Without it, "arm A" is a label.
        **_bin_fingerprint(args.bin),
        "config_held": config_held.get("config_held"),   # True=held target, False=demoted, None=n/a
        "config_held_detail": config_held,          # {reached_target, demoted_to, wb_configs_after, pinned, floor_s, ...}
        "config_held_pinned": pinned,               # pinned session => warm-up load credited (the config_held fix)
        "demoted_to": config_held.get("demoted_to"),     # the fallback rung (e.g. 16) if demoted; the C5 answer
        "decode_at_target": decode_at_target,       # byte-faithful WHILE holding target (honest decode17 for a cfg17 cell)
        # -- GEOMETRY + ELECTION FIRE-PROOF (silently-STOCK / silently-cfg16 detector) --
        "pilot_geometry": pilot_geometry,           # {by_config:{cfg:{Nsymb,Dy,...}}, matched_rows} from [PILOT_DIAG]
        "topgear_election": topgear_election,       # {elected_16_to_17, transitions, matched_rows} from [TOPGEAR]
        # -- BOTH WINDOWS (labeled): steady-state envelope + whole-transfer cold/warm UX --
        "windows": windows,
        "measure_variance": measure_variance,       # {measure_variance, recovered_snr_db, source} or None
        "rx_bps_wall": (
            round(score_rx_for_math * 8 / scored_window_seconds, 1)
            if scored_window_seconds else None
        ),
        "wall_secs": (
            round(scored_window_seconds, 1)
            if scored_window_seconds is not None else None
        ),
        "verdict": ("INTEGRITY_FAIL" if not byte_integrity_ok
                    else "UNIQUENESS_FAIL" if not uniqueness_ok
                    else "INSTRUMENT_INVALID" if instrument_invalid else "OK"),
    }
    result = apply_terminal_eot_gate(result, logpath)
    result["raw_evidence"] = raw_evidence_record
    result["override_preflight"] = override_preflight
    print(json.dumps(result))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=1)
    _publish_startup_phase(
        args.startup_phase_file, "done", t0, args.tag,
        connected=bool(result["connected"]), verdict=result["verdict"],
        result_json=os.path.abspath(args.json) if args.json else None)
    return (2 if instrument_invalid else
            3 if result.get("whole_session_status") == "VOID" else 0)


if __name__ == "__main__":
    sys.exit(main())

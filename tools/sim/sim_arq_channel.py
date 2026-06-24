#!/usr/bin/env python3
"""
sim_arq_channel.py — device-free ARQ-loop channel simulator harness.

Drives a REAL two-process Mercury ARQ session through the SIM software channel
backend (`-x sim` + tools/sim/sim_channel_relay.py) at a FIXED, configurable SNR
and sample-loss, with NO audio hardware. This reproduces, in MINUTES, the HW
behavior the IONOS bench shows over ~3h re-measures:

  * over-climb -> collapse  (clean/wgn cells: gearshift rockets up to a high
    config, that config FAILS on the lossy channel, link BREAKs/collapses to
    ROBUST_0 and stalls), and
  * deep-SNR STALL          (very low SNR cells: link pins at ROBUST_0 /
    CONFIG_0 and delivers almost nothing).

WHY this exists: the -m ARQ loopback was near-LOSSLESS because AWGN was only
injected in PLOT_PASSBAND BER mode (telecom_system.cc:292), never through the
ARQ loop. An over-climbed config never failed -> the collapse never reproduced
-> sim-validated gearshift fixes moved ZERO on HW. The SIM backend routes the
full ARQ passband through a relay that adds sustained AWGN + bursty loss, so
the collapse and stall reproduce deterministically.

=== I7 instrumentation (chase-combine delivered-bps A/B) =====================
This harness is the in-process FTRT driver for the chase-combine I7 gate. To
make a delivered-bps measurement that the VARA scoring surface can be compared
against, three things are emitted that the old repeating-pattern TX could not:

  * FIXED INCOMPRESSIBLE payload (--payload, default
    payload_incompressible_64k.bin): high-entropy bytes (NOT bytes(range(256))*8)
    so PPMd/zstd are bypassed and the wire carries RAW PHY rate. The TX thread
    loops this file; the harness records the EXACT byte stream md5.
  * recv_md5 + delivered_bytes: the RX thread accumulates the delivered bytes
    and their streaming md5. md5_match compares recv_md5 against the md5 of the
    same-length PREFIX of the TX stream — a real end-to-end integrity check on
    delivered bytes (not just a byte count).
  * retransmission + chase-fire counters: parsed from the CMD log. n_cmd_retx /
    n_cmd_retx_v2 / n_sack_retx_lines / nResent_data_max are generic ARQ retx
    counts; n_chase_fire keys on the [CHASE] marker (0 against a monitor binary
    that lacks the chase-combine feature; non-zero once feat/chase-combine ships
    its [CHASE] print).

Usage:
  python tools/sim/sim_arq_channel.py --snr 12 --loss 0.02 --secs 180
  python tools/sim/sim_arq_channel.py --snr 2 --loss 0.10 --burst --secs 240   # deep stall
  python tools/sim/sim_arq_channel.py --snr 35 --secs 180                       # clean over-climb

Options:
  --bin PATH        mercury.exe (default: <mercury-repo>/mercury.exe)
  --snr DB          channel SNR3k in dB (referenced to 3 kHz; default 12)
  --cell WGN:N      convenience SNR3k = N + 2.4 (testbed WGN-label mapping)
  --profile P       fading profile: wgn (none) / mpg / mpm / mpp (ITU HF)
  --cfo-hz HZ       carrier frequency offset (default 0)
  --phase-noise-deg D   per-sample phase-noise stddev in deg (default 0.2)
  --loss F          impulse/burst erasure fraction (orthogonal to fading)
  --burst           Gilbert-Elliott bursty impulse dropout (impulse-noise knob)
  --secs N          dwell seconds (default 180)
  --port N          relay TCP port PREFERENCE (default 52100; auto-advanced if busy)
  --ctrl-base N     pin the control/data quad base (default auto-pick from 7002)
  --start-cfg N     mercury -s config (default 100 = ROBUST_0)
  --robust          pass -R (start in robust tier; default on for start-cfg>=100)
  --compress on|off (default off, so we read RAW PHY gearshift behavior)
  --payload PATH    TX payload file (default payload_incompressible_64k.bin)
  --json PATH       write a machine-readable result summary

NOTE: ports are AUTO-PICKED at startup (a free control/data quad {base, base+1,
base+4, base+5} plus a free relay port; base advances on collision). Cleanup is
PORT-SCOPED — only PIDs holding THIS run's ports are killed, never a system-wide
`taskkill /IM mercury.exe`. So two (or N) invocations run CONCURRENTLY without
killing each other's modems or colliding on ports. Use --ctrl-base to pin the
base for a deterministic re-run.

Verdict: prints whether over-climb->collapse and/or deep-SNR stall reproduced,
plus the config-switch timeline (same fields the muething HW harness records:
switch_seq, peak_config, steady_config, final_config), the delivered-byte md5,
and the retx/chase counters.
"""
import argparse, hashlib, json, os, re, socket, subprocess, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
# Migrated into the mercury repo at mercury/tools/sim/ (was tools/ in the outer
# workspace). The mercury binary lives at the mercury repo root.
MERCURY_ROOT = os.path.dirname(os.path.dirname(HERE))   # mercury/tools/sim -> mercury
REPO = MERCURY_ROOT                                      # back-compat alias
DEFAULT_BIN = os.path.join(MERCURY_ROOT, "mercury.exe")
RELAY = os.path.join(HERE, "sim_channel_relay.py")
DEFAULT_PAYLOAD = os.path.join(HERE, "payload_incompressible_64k.bin")

# Control/data ports are AUTO-PICKED at startup (see pick_free_ports) so multiple
# sim cells can run concurrently without colliding — the historical hardcoded
# 7002/7006 quad was the single root cause of three consecutive "port-blocked"
# calibration runs (a live bench experiment owned 7002/7006/52100). These module
# constants are the *defaults the auto-picker probes FIRST* (base=7002), and the
# --ctrl-base override pins the base for determinism. The chosen quad is recorded
# in the result JSON so cleanup is PORT-SCOPED, never a system-wide taskkill.
DEFAULT_CTRL_BASE = 7002          # RSP ctrl; data=+1, CMD ctrl=+4, CMD data=+5
DEFAULT_RELAY_PORT = 52100        # relay listen port (independent of the quad)
RSP_PORT = DEFAULT_CTRL_BASE      # rebound in main() after pick_free_ports
CMD_PORT = DEFAULT_CTRL_BASE + 4  # rebound in main() after pick_free_ports

# ============================================================================
# VIRTUAL-TIME RUN BOUND (host-load-independence)
# ============================================================================
# The modem core clock is VIRTUAL (sim_clock.cc:43-52: virtual_ns = samples *
# 1e9 / 48000; advanced ONLY by sim_clock_add_samples(len) per forwarded chunk
# in audioio.c rx_transfer:1753-1754). Every climb-path deadline (gearshift
# receiving_timeout, PTT turnaround, barrier) reads this virtual clock, NOT the
# wall clock. The trajectory is therefore a function of VIRTUAL time only and is
# host-load-independent BY CONSTRUCTION.
#
# The ONE place that broke that property was this harness's run loop, which
# bounded the whole monitored run by `time.time() - start < args.secs` (REAL
# seconds). Under host CPU saturation fewer VIRTUAL seconds fit the REAL budget,
# so the ROBUST_0->CFG16 climb truncated (e.g. stalled at CONFIG_13) and the
# SAME seed produced a DIFFERENT trajectory idle-vs-hammered. That is the bug.
#
# THE FIX: bound the run by VIRTUAL time (and/or transfer completion), with only
# a GENEROUS real-time watchdog that guards a true wedge and never truncates a
# legit-but-slow climb. Virtual seconds are read from the relay log, which is the
# SINGLE authoritative virtual-time source (sim_channel_relay.py:980,998-1002):
# each forwarded chunk carries vstamp = counters[key]*CHUNK_SAMPLES, the
# monotonic per-direction END sample index. virtual_seconds = vstamp / FS. The
# relay forwards under the conservative-PDES barrier (the two per-direction
# counters stay within BARRIER_K chunks), so we take the MIN of the two
# directions as the conservative virtual-clock FLOOR.
SIM_FS = 48000.0              # passband wire sample rate (sim_channel_relay.py FS)
# Relay stats line (sim_channel_relay.py:998-1002), emitted every 500 chunks:
#   [HH:MM:SS] a2b: 1500 chunks (32.0s) vstamp=1536000 split=+0 P_sig=...
RELAY_VSTAMP_RE = re.compile(r"\b(a2b|b2a):\s+\d+\s+chunks\s+\([\d.]+s\)\s+vstamp=(\d+)")
# Generous real-time watchdog: a healthy climb runs MUCH faster than real time
# (FTRT), so a real run never legitimately exceeds this multiple of the virtual
# budget. It ONLY fires on a true wedge (relay not forwarding / a process hung).
REAL_WATCHDOG_MULT = 20.0     # real-seconds ceiling = 20x the virtual budget ...
REAL_WATCHDOG_FLOOR = 600.0   # ... but never less than this (short virtual runs)
# If the relay's virtual clock does not advance for this many REAL seconds while
# the run has not completed, the channel is wedged (no forwarding) -> abort.
VCLOCK_STALL_REAL_S = 90.0


def read_relay_virtual_seconds(relay_log_path):
    """Return (virtual_seconds, ok) parsed from the relay log's vstamp stats
    lines. virtual_seconds = min(a2b_vstamp, b2a_vstamp) / FS = the conservative
    virtual-clock floor (the slower of the two barrier-locked directions). ok is
    False until BOTH directions have emitted at least one vstamp (pre-CONNECT /
    log-not-yet-written), at which point virtual_seconds is 0.0 / ok False so the
    caller falls back to its grace window rather than the real clock.

    Reads the tail only (the vstamps are monotonic; the last line per direction
    is the latest). Robust to a partially-written/locked log: any read error ->
    (last_known, False)."""
    a2b = b2a = None
    try:
        with open(relay_log_path, "r", errors="replace") as f:
            for line in f:
                m = RELAY_VSTAMP_RE.search(line)
                if not m:
                    continue
                key, stamp = m.group(1), int(m.group(2))
                if key == "a2b":
                    a2b = stamp
                else:
                    b2a = stamp
    except OSError:
        return (0.0, False)
    if a2b is None and b2a is None:
        return (0.0, False)
    # If only one direction has logged a vstamp yet, use it (the barrier keeps
    # the other within BARRIER_K chunks; the floor is conservative either way).
    stamps = [s for s in (a2b, b2a) if s is not None]
    return (min(stamps) / SIM_FS, True)


# Mercury prints "[GEARSHIFT] SET_CONFIG: forward=N ..." on every config change
# (arq_commander.cc:692) and "loaded config N" on load. Track both.
SETCFG_RE = re.compile(r"SET_CONFIG:\s*forward=(\d+)")
LOADED_RE = re.compile(r"loaded config (\d+)")
GEAR_RE = re.compile(
    r"\[GEARSHIFT\]|\[TURBO\]|\[BREAK\]|FRAME UP|SET_CONFIG|loaded config|"
    r"SUPERSHIFT|RE-TRIGGER|DISCONNECT|CONNECTED|\[SIM\]|\[CMD-RETX\]|\[CHASE\]")

# I7 retransmission + chase-fire markers (CMD-side log lines, exact source
# format strings):
#   [CMD-RETX] Sending %d retransmit frames                 (arq_commander.cc:1293)
#   [CMD-RETX-V2] retransmit batch carries original ...     (arq_commander.cc:1347)
#   [CMD-SACK] %d/%d received, %d queued for retransmit     (arq_commander.cc:2973)
#   stats.nReSent_data= %d                                  (arq_common.cc:7440)
#   [CHASE] ...   chase-combine fire marker (feat/chase-combine; absent on monitor)
CMD_RETX_RE      = re.compile(r"\[CMD-RETX\] Sending (\d+) retransmit frames")
CMD_RETX_V2_RE   = re.compile(r"\[CMD-RETX-V2\] retransmit batch")
SACK_RETX_RE     = re.compile(r"\[CMD-SACK\]\s+(\d+)/(\d+) received,\s+(\d+) queued for retransmit")
NRESENT_RE       = re.compile(r"stats\.nReSent_data=\s*(\d+)")
CHASE_RE         = re.compile(r"\[CHASE\]")


def cfg_name(n):
    if 100 <= n <= 102:
        return f"ROBUST_{n - 100}"
    if 0 <= n <= 16:
        return f"CONFIG_{n}"
    return f"CFG{n}"


# GUARD 2: refuse to launch a mercury binary that lacks the GUARD-1 marker.
#
# Why: this harness drives mercury with `-x sim`, a device-free software
# channel. A STALE or wrong binary (e.g. the main-tree mercury.exe pointed at by
# DEFAULT_BIN, which predates the device-free SIM backend) does NOT short-circuit
# the device threads on `-x sim` — it falls through to the real audio device and
# PLAYS MODEM TONES OUT OF THE USER'S PHYSICAL SPEAKERS. That actually happened
# twice. GUARD 1 (source/audioio/audioio.c) compiles the marker "[SIM-AUDIO-GUARD]"
# into every guard-protected binary and aborts-before-render if a device is ever
# touched under -x sim. GUARD 2 makes the leak structurally impossible from the
# harness side: before we spawn ANYTHING, we read the chosen binary and refuse to
# run it unless that marker is present. A non-guard binary is never launched.
SIM_AUDIO_GUARD_MARKER = b"[SIM-AUDIO-GUARD]"


def require_guard_binary(bin_path):
    """Exit(2) WITHOUT launching anything if bin_path lacks the GUARD-1 marker."""
    if not os.path.isfile(bin_path):
        print(f"FATAL [SIM-AUDIO-GUARD]: mercury binary not found: {bin_path}",
              file=sys.stderr)
        print("Refusing to launch. -x sim requires a guard-protected build.",
              file=sys.stderr)
        sys.exit(2)
    try:
        with open(bin_path, "rb") as f:
            blob = f.read()
    except OSError as e:
        print(f"FATAL [SIM-AUDIO-GUARD]: cannot read mercury binary {bin_path}: {e}",
              file=sys.stderr)
        sys.exit(2)
    if SIM_AUDIO_GUARD_MARKER not in blob:
        print(f"FATAL [SIM-AUDIO-GUARD]: binary {bin_path} lacks the SIM audio guard "
              f"({SIM_AUDIO_GUARD_MARKER.decode()}).", file=sys.stderr)
        print("This is a STALE or non-guard build. Under -x sim it could open a real "
              "audio device and LEAK MODEM TONES to your speakers.", file=sys.stderr)
        print("Refusing to launch. Rebuild from a tree containing GUARD 1 "
              "(source/audioio/audioio.c) and point --bin at it.", file=sys.stderr)
        sys.exit(2)
    print(f"[SIM-AUDIO-GUARD] OK: {bin_path} contains the device-open guard marker.")


# ---------------------------------------------------------------------------
# PORT AUTO-PICK — collision-proof concurrent sims (root-cause fix)
# ---------------------------------------------------------------------------
# The harness binds FIVE TCP ports per run:
#   RSP ctrl = base       RSP data = base+1
#   CMD ctrl = base+4     CMD data = base+5    relay = a separate free port
# These were hardcoded (7002/7006 + 52100), so a concurrent run (e.g. a live
# bench experiment) holding them blocked every retry. pick_free_ports bind-probes
# a whole quad at once; on ANY member being busy it advances base by +8 and
# retries (cap PORT_PICK_TRIES). The relay port is probed independently. A
# --ctrl-base override pins the base for deterministic re-runs.
PORT_PICK_TRIES = 20
PORT_PICK_STRIDE = 8              # quad is 6 wide; +8 leaves a 2-port guard gap


def _port_free(port):
    """True iff a fresh TCP socket can bind 127.0.0.1:port RIGHT NOW.

    No SO_REUSEADDR: we want the probe to FAIL if anything (a live modem, a
    sibling experiment, a lingering zombie) already holds the port, so the quad
    we hand out is genuinely free. The probe socket is closed immediately so the
    real owner can bind it microseconds later."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def pick_free_ports(ctrl_base=None, relay_pref=None, tries=PORT_PICK_TRIES):
    """Return (rsp_ctrl, rsp_data, cmd_ctrl, cmd_data, relay_port).

    Probe the quad {base, base+1, base+4, base+5} for a base starting at
    ctrl_base (default DEFAULT_CTRL_BASE); on collision advance by PORT_PICK_STRIDE
    and retry up to `tries`. The relay port is probed from relay_pref upward,
    avoiding the chosen quad. Raises RuntimeError if no free quad/relay is found.

    Determinism: pass ctrl_base to pin the base (it still verifies the quad is
    free and FAILS LOUDLY rather than silently colliding)."""
    base0 = DEFAULT_CTRL_BASE if ctrl_base is None else ctrl_base
    quad = None
    for i in range(tries):
        base = base0 + i * PORT_PICK_STRIDE
        members = (base, base + 1, base + 4, base + 5)
        if all(_port_free(p) for p in members):
            quad = members
            break
    if quad is None:
        raise RuntimeError(
            f"pick_free_ports: no free ctrl/data quad after {tries} tries from "
            f"base {base0} (stride {PORT_PICK_STRIDE}). Ports busy — is a sibling "
            f"sim/bench experiment running? Check netstat for :{base0}..")
    # Relay port: independent, just needs to be free and not collide with the quad.
    relay0 = DEFAULT_RELAY_PORT if relay_pref is None else relay_pref
    relay_port = None
    for i in range(tries * PORT_PICK_STRIDE):
        cand = relay0 + i
        if cand in quad:
            continue
        if _port_free(cand):
            relay_port = cand
            break
    if relay_port is None:
        raise RuntimeError(
            f"pick_free_ports: no free relay port near {relay0}.")
    return (*quad, relay_port)


def _pids_on_ports(ports):
    """Return the set of PIDs holding (LISTENING/ESTABLISHED on) any of `ports`,
    parsed from `netstat -ano` (Windows). Used for PORT-SCOPED teardown so we
    NEVER system-wide-taskkill mercury.exe (a sibling bench run shares the box)."""
    want = {str(p) for p in ports}
    pids = set()
    try:
        out = subprocess.run(["netstat", "-ano", "-p", "TCP"],
                             capture_output=True, text=True, timeout=15).stdout
    except (OSError, subprocess.SubprocessError):
        return pids
    for line in out.splitlines():
        parts = line.split()
        # TCP  127.0.0.1:7002  0.0.0.0:0  LISTENING  12345
        if len(parts) >= 5 and parts[0].upper() == "TCP":
            local = parts[1]
            if ":" in local:
                lport = local.rsplit(":", 1)[1]
                if lport in want:
                    pid = parts[-1]
                    if pid.isdigit() and pid != "0":
                        pids.add(pid)
    return pids


def kill_port_scoped(ports, my_pids):
    """Kill ONLY the PIDs holding our ports (plus our own spawned PIDs). Never a
    /IM mercury.exe sweep — a concurrent bench run on different ports must survive."""
    targets = set(str(p) for p in my_pids if p)
    targets |= _pids_on_ports(ports)
    for pid in targets:
        try:
            subprocess.run(["taskkill", "/F", "/PID", str(pid)],
                          capture_output=True, timeout=10)
        except (OSError, subprocess.SubprocessError):
            pass


class State:
    def __init__(self):
        self.switch_seq = []      # list of config ids in order
        self.lock = threading.Lock()
        self.connected = False
        self.disconnected = False
        self.lines = []
        # I7 retx / chase counters (CMD-side; updated by log_output under lock)
        self.n_cmd_retx = 0           # count of [CMD-RETX] lines
        self.cmd_retx_frames = 0      # sum of frames over [CMD-RETX] lines
        self.n_cmd_retx_v2 = 0        # count of [CMD-RETX-V2] lines
        self.n_sack_retx_lines = 0    # count of [CMD-SACK] ... queued-for-retx lines
        self.sack_retx_frames = 0     # sum of "queued for retransmit" counts
        self.nresent_data_max = 0     # max stats.nReSent_data= seen
        self.n_chase_fire = 0         # count of [CHASE] markers (0 on monitor)


def log_output(proc, label, logfile, t0, st):
    try:
        for line in iter(proc.stdout.readline, b''):
            text = line.decode("utf-8", "replace").rstrip()
            entry = f"[T+{time.time()-t0:08.3f}] [{label}] {text}"
            logfile.write(entry + "\n")
            logfile.flush()
            # Mercury prints "link_status:Connected to <call>" on both peers
            # when the ARQ link establishes (NOT the "[SIM] connected to
            # channel relay" transport line). Match the link-layer connect.
            if "link_status:Connected to" in text:
                st.connected = True
            if "link_status:Disconnected" in text or "DISCONNECTED" in text:
                st.disconnected = True
            # Only the COMMANDER drives config selection + retransmits; track its
            # SET_CONFIG and retx/chase markers.
            if label == "CMD":
                m = SETCFG_RE.search(text)
                if m:
                    cid = int(m.group(1))
                    with st.lock:
                        if not st.switch_seq or st.switch_seq[-1] != cid:
                            st.switch_seq.append(cid)
                            print(f"[T+{time.time()-t0:7.1f}] CONFIG -> {cfg_name(cid)}")
                            sys.stdout.flush()
                mr = CMD_RETX_RE.search(text)
                if mr:
                    with st.lock:
                        st.n_cmd_retx += 1
                        st.cmd_retx_frames += int(mr.group(1))
                if CMD_RETX_V2_RE.search(text):
                    with st.lock:
                        st.n_cmd_retx_v2 += 1
                ms = SACK_RETX_RE.search(text)
                if ms:
                    with st.lock:
                        st.n_sack_retx_lines += 1
                        st.sack_retx_frames += int(ms.group(3))
                mn = NRESENT_RE.search(text)
                if mn:
                    with st.lock:
                        st.nresent_data_max = max(st.nresent_data_max, int(mn.group(1)))
            # chase-combine can fire on either peer's PHY; count globally.
            if CHASE_RE.search(text):
                with st.lock:
                    st.n_chase_fire += 1
            if GEAR_RE.search(text):
                with st.lock:
                    st.lines.append(entry)
    except (ValueError, OSError):
        # readline on a closed/killed pipe at shutdown — expected, ignore.
        pass


# LOAD-TOLERANT socket timeouts. Under heavy host CPU load a CONNECT-clean modem
# peer can be slow to accept/produce because its process is starved of CPU, NOT
# because it is dead. The OLD small timeouts (connect 5s, tx 30s) could declare a
# merely-starved peer dead and END the run -> a different (truncated) trajectory
# under load = the SAME host-load-coupling bug class as the run cap. These are
# generous so a starved-but-alive peer is never falsely killed; true death is
# detected by p.poll() in the monitor loop, and a true channel wedge by the
# virtual-clock stall watchdog -- not by a short socket timeout.
CONNECT_TIMEOUT_S = 30.0          # startup connect (retry loop also present)
TX_SOCK_TIMEOUT_S = 120.0         # TX-FIFO push: generous; timeout -> RETRY not die


def tcp_send(port, commands, label, retries=20, delay=1.0):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(CONNECT_TIMEOUT_S)
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
            sock.settimeout(CONNECT_TIMEOUT_S)
    for c in commands:
        sock.sendall(c.encode())
        time.sleep(0.3)
    return sock


def tx_thread_fn(sock, stop, res, payload, st):
    """Stream the FIXED INCOMPRESSIBLE payload, looping it whole-file per send,
    and accumulate a streaming md5 of the EXACT bytes sent. Whole-file chunks
    keep the loop boundary aligned so the delivered-byte prefix md5 (computed
    deterministically in _md5_of_looped_prefix) matches the TX stream exactly.

    WAIT-FOR-CONNECT: do not pump the data socket until the ARQ link is up.
    Flooding 64 KiB/30 ms into an Idle modem's TX FIFO starves the half-duplex
    HAIL TX (observed: txpeak stays 0.0 and CONNECT never completes), and it
    also dirties the delivered-bps window with pre-connect bytes the modem
    silently drops. Gating on st.connected starts the measured stream at link-up,
    which is exactly the window the I7 delivered-bps A/B wants."""
    sock.settimeout(TX_SOCK_TIMEOUT_S)
    md5 = hashlib.md5()
    # Park until the link connects (or the run ends). Cheap poll; the harness
    # sets st.connected on the "link_status:Connected to" log line.
    while not stop.is_set() and not st.connected:
        time.sleep(0.1)
    while not stop.is_set():
        try:
            sock.sendall(payload)
            res["tx"] += len(payload)
            md5.update(payload)
            res["tx_md5_hex"] = md5.hexdigest()
        except socket.timeout:
            # The modem TX FIFO is backed up (slow/starved peer), NOT dead. Do
            # NOT end TX on a timeout -- retry. A truly dead peer is caught by
            # p.poll() in the monitor loop; a wedged channel by the vclock-stall
            # watchdog. Killing TX here on a load-induced stall would truncate the
            # trajectory under host load = the bug we are fixing.
            continue
        except (ConnectionError, OSError):
            break
        time.sleep(0.03)


def rx_thread_fn(sock, stop, res):
    """Accumulate delivered bytes + their streaming md5 (recv_md5)."""
    # Short timeout is fine here: rx ALREADY continues-on-timeout (a quiet RX
    # gap is normal), so a starved peer is never declared dead by this thread.
    # 2s only sets the stop-flag check cadence; keep it.
    sock.settimeout(2)
    md5 = hashlib.md5()
    while not stop.is_set():
        try:
            d = sock.recv(8192)
            if not d:
                break
            res["rx"] += len(d)
            md5.update(d)
            res["recv_md5_hex"] = md5.hexdigest()
        except socket.timeout:
            continue
        except (ConnectionError, OSError):
            break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=DEFAULT_BIN)
    ap.add_argument("--snr", type=float, default=12.0,
                    help="channel SNR3k in dB (referenced to 3 kHz). "
                         "Overridden by --cell.")
    ap.add_argument("--cell", default=None,
                    help="convenience SNR spec, e.g. WGN:-12 (SNR3k = label+2.4)")
    ap.add_argument("--profile", default="wgn",
                    help="fading profile: wgn (none), mpg/mpm/mpp (ITU HF)")
    ap.add_argument("--cfo-hz", type=float, default=0.0,
                    help="carrier frequency offset in Hz")
    ap.add_argument("--phase-noise-deg", type=float, default=0.2,
                    help="per-sample phase-noise stddev in degrees")
    ap.add_argument("--loss", type=float, default=0.0,
                    help="impulse/burst erasure fraction (orthogonal to fading)")
    ap.add_argument("--burst", action="store_true",
                    help="Gilbert-Elliott bursty impulse dropout (impulse knob)")
    # CONNECT-REACK T1 (connect-testack-handshake.md §5): deterministically erase
    # the Nth RSP->CMD signal burst (2 = first TEST_CONNECTION_ACK). 0=disabled.
    ap.add_argument("--erase-b2a-burst", type=int, default=0,
                    help="TEST-ONLY: drop the Nth RSP->CMD burst (2=first TEST_ACK)")
    # deprecated flat-fade knobs (superseded by --profile); kept for compat.
    ap.add_argument("--fade-hz", type=float, default=0.0, help=argparse.SUPPRESS)
    ap.add_argument("--fade-depth", type=float, default=0.0, help=argparse.SUPPRESS)
    # --secs is now a VIRTUAL-time dwell budget (sim/virtual-time-runbound):
    # the run is bounded by virtual time / transfer completion, not real seconds.
    ap.add_argument("--secs", type=int, default=180,
                    help="VIRTUAL-time DATA-phase dwell budget in seconds (sim_clock, "
                         "parsed from the relay vstamp). CONNECT-DWELL FIX: this "
                         "budget is ANCHORED AT CONNECT — it starts counting only "
                         "AFTER the ARQ link establishes (link_status:Connected), so "
                         "the multi-leg LDPC connect handshake (which under RT pacing "
                         "consumes ~75-85 virtual seconds) does NOT eat the data "
                         "budget. The run is bounded by VIRTUAL DATA time / transfer "
                         "completion, NOT real wall-clock seconds -> host-load-"
                         "independent trajectory. A generous real watchdog "
                         "(REAL_WATCHDOG_MULT x) only guards a true wedge.")
    ap.add_argument("--connect-grace", type=int, default=150,
                    help="VIRTUAL-time CONNECT grace in seconds (CONNECT_UNDER_RT_"
                         "ROOTCAUSE §6.1). Before the link establishes the run is "
                         "bounded ONLY by this grace (NOT --secs) — the modem keeps "
                         "its own max_connection_attempts retry loop running, matching "
                         "the HW teardown discipline (HW imposes no wall guillotine). "
                         "Sized to the RT worst-case multi-leg LDPC handshake "
                         "(~75-85s observed + margin). If the link never connects "
                         "within this grace the run ends as 'connect_timeout'. The "
                         "wedge/vclock-stall/real-watchdog guards still apply.")
    ap.add_argument("--target-bytes", type=int, default=0,
                    help="if >0, the run also COMPLETES as soon as delivered_bytes "
                         ">= this (with md5 match). 0 = no byte target (climb-sim: "
                         "loop the payload until the virtual-time budget is spent).")
    # --port keeps monitor's COLLISION-PROOF auto-pick (DEFAULT_RELAY_PORT, advanced
    # if busy) + --ctrl-base, NOT virtual-time-runbound's fixed 52100, so concurrent
    # sims (now load-independent and therefore run in parallel) never collide.
    ap.add_argument("--port", type=int, default=DEFAULT_RELAY_PORT,
                    help="relay listen port PREFERENCE; auto-advanced if busy. "
                         "Auto-pick keeps concurrent sims collision-proof.")
    ap.add_argument("--ctrl-base", type=int, default=None,
                    help="pin the control-port quad base (RSP=base, RSP-data=base+1, "
                         "CMD=base+4, CMD-data=base+5) for deterministic re-runs. "
                         "Default: auto-pick starting at %d. Probe still verifies the "
                         "quad is free and fails loudly on collision." % DEFAULT_CTRL_BASE)
    ap.add_argument("--start-cfg", type=int, default=100)
    ap.add_argument("--robust", action="store_true", default=None)
    ap.add_argument("--no-gearshift", action="store_true",
                    help="omit -g (and add -Q 0) so --start-cfg HOLDS (no "
                         "climb/demote). PINs a single tier so wire_bps_airtime "
                         "reads a per-frame wire for THAT config (not a "
                         "climb-ramp-diluted mix). See GAP1_AIRTIME_WIRE_VERDICT.md.")
    ap.add_argument("--compress", default="off")
    ap.add_argument("--payload", default=DEFAULT_PAYLOAD,
                    help="TX payload file (default payload_incompressible_64k.bin, "
                         "a fixed high-entropy file so compression is bypassed and "
                         "the wire carries RAW PHY rate for the delivered-bps A/B)")
    ap.add_argument("--barrier-k", type=int, default=1,
                    help="conservative-PDES lockstep window (chunks) passed to the "
                         "relay. K=1 = strict lockstep (default); relax to 4/8 if "
                         "the strict window starves CONNECT/delivery.")
    # --seed is the sim/virtual-time-runbound alias for --relay-seed (same dest):
    # both name the relay channel-RNG seed passthrough. Unified onto one argument
    # so the relay receives "--seed" exactly ONCE (a duplicate would be silently
    # last-wins). Same seed -> bit-identical channel realization -> reproducible A/B.
    ap.add_argument("--relay-seed", "--seed", type=int, default=1, dest="relay_seed",
                    help="relay --seed passthrough (seeds the deterministic AWGN + "
                         "per-key-up turnaround-jitter walk). Use MATCHED seeds across "
                         "a D2-ON vs D2-OFF A/B so both arms see the identical channel "
                         "+ jitter realization. DEFAULT 1. (--seed is an alias.)")
    # ---- FIX9 inter-peer drift / PTT turnaround repro (OPT-IN, passthrough) ---
    # DEFAULT 0 (off) -> byte-identical deterministic A/B. On -> the relay re-times
    # the forwarded per-direction stream by the given ppm sample-clock skew (and
    # injects PTT keying latency at TX onsets), reproducing the HW CFG16
    # half-duplex turnaround de-alignment the conservative-PDES barrier masks
    # (FIX9_ROOTCAUSE.md). The HW bench measured ~-670 ppm relative skew.
    ap.add_argument("--drift-ppm-a2b", type=float, default=0.0,
                    help="relay --drift-ppm-a2b passthrough (sample-clock skew, "
                         "ppm, on A->B). DEFAULT 0 (off). FIX9 repro: -670.")
    ap.add_argument("--drift-ppm-b2a", type=float, default=0.0,
                    help="relay --drift-ppm-b2a passthrough (B->A). DEFAULT 0.")
    ap.add_argument("--ptt-latency-ms", type=float, default=0.0,
                    help="relay --ptt-latency-ms passthrough (TX-onset keying "
                         "latency, ms, per direction). DEFAULT 0 (off).")
    ap.add_argument("--ptt-latency-jitter-ms", type=float, default=0.0,
                    help="relay --ptt-latency-jitter-ms passthrough. DEFAULT 0.")
    # ---- FAITHFUL turnaround-timing window-miss model (SIMFIDELITY M1-M4) ------
    # The CORRECTED drift axis: a turnaround-timing WINDOW-MISS (reverse ACK lands
    # outside the CMD listen window) realized by integer silence insert/drop with
    # SIGNAL samples bit-exact. The right vehicle for the CFG16 reverse-ACK
    # collapse + the FIX9 D2 A/B (D2 widens the window, which fixes a window-miss
    # but NOT the old tone-smear). DEFAULT off -> byte-identical.
    ap.add_argument("--turnaround-drift", action="store_true",
                    help="relay --turnaround-drift passthrough (FAITHFUL window-"
                         "miss model). DEFAULT off. The corrected axis for the "
                         "CFG16 reverse-ACK collapse + D2 A/B.")
    ap.add_argument("--turnaround-ppm-a2b", type=float, default=8.16,
                    help="relay --turnaround-ppm-a2b passthrough (physical crystal "
                         "slip, default +8.16; NOT the -670 [CLK-TX] artifact).")
    ap.add_argument("--turnaround-ppm-b2a", type=float, default=-8.16,
                    help="relay --turnaround-ppm-b2a passthrough (default -8.16).")
    ap.add_argument("--turnaround-jitter-ms", type=float, default=6.0,
                    help="relay --turnaround-jitter-ms passthrough (per-key-up "
                         "turnaround jitter; the dominant trigger). DEFAULT 6.0.")
    ap.add_argument("--wire-stamp", type=int, default=0, choices=(0, 1),
                    help="relay --wire-stamp passthrough. DEFAULT 0 (bare 8192, "
                         "compatible with every shipped -x sim mercury). Set 1 "
                         "ONLY when --bin reads the 8-byte stamp (feat/sim-clock "
                         "modem half); a non-stamp modem silently corrupts the "
                         "wire. See _simcal_takeover/ASSESSMENT.md.")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    # GUARD 2: refuse a non-guard binary BEFORE launching the relay or any
    # mercury process. If args.bin lacks the GUARD-1 marker this exits(2) here
    # and nothing is spawned — the leak is structurally impossible.
    require_guard_binary(args.bin)

    # AUTO-PICK a free control/data quad + relay port (collision-proof). Rebind
    # the module-level RSP_PORT/CMD_PORT the nested launch/socket code uses, and
    # args.port (the relay port). On collision the picker advances the base; a
    # --ctrl-base pins it for deterministic re-runs.
    global RSP_PORT, CMD_PORT
    try:
        RSP_PORT, _rsp_data, CMD_PORT, _cmd_data, relay_port = pick_free_ports(
            ctrl_base=args.ctrl_base, relay_pref=args.port)
    except RuntimeError as e:
        print(f"FATAL: {e}", file=sys.stderr)
        sys.exit(3)
    args.port = relay_port
    chosen_ports = {
        "rsp_ctrl": RSP_PORT, "rsp_data": RSP_PORT + 1,
        "cmd_ctrl": CMD_PORT, "cmd_data": CMD_PORT + 1,
        "relay": relay_port,
    }
    my_ports = [RSP_PORT, RSP_PORT + 1, CMD_PORT, CMD_PORT + 1, relay_port]
    print(f"[PORTS] auto-picked quad: RSP ctrl={RSP_PORT} data={RSP_PORT+1} | "
          f"CMD ctrl={CMD_PORT} data={CMD_PORT+1} | relay={relay_port}")

    # Load the fixed incompressible payload.
    if not os.path.isfile(args.payload):
        print(f"FATAL: payload file not found: {args.payload}", file=sys.stderr)
        sys.exit(2)
    with open(args.payload, "rb") as f:
        payload = f.read()
    if not payload:
        print(f"FATAL: payload file is empty: {args.payload}", file=sys.stderr)
        sys.exit(2)
    payload_md5 = hashlib.md5(payload).hexdigest()

    use_robust = args.robust if args.robust is not None else (args.start_cfg >= 100)

    print("=== SIM ARQ channel ===")
    print(f"bin={args.bin}")
    snr_desc = f"cell={args.cell}" if args.cell else f"SNR3k={args.snr}dB"
    print(f"{snr_desc} profile={args.profile} cfo={args.cfo_hz}Hz "
          f"phase_noise={args.phase_noise_deg}deg loss={args.loss} "
          f"burst={args.burst} dwell={args.secs}s "
          f"start={cfg_name(args.start_cfg)} compress={args.compress} "
          f"barrier_k={args.barrier_k} "
          f"drift_ppm(a2b={args.drift_ppm_a2b},b2a={args.drift_ppm_b2a}) "
          f"ptt_latency_ms={args.ptt_latency_ms}(jit={args.ptt_latency_jitter_ms}) "
          f"turnaround_drift={args.turnaround_drift}"
          f"(ppm a2b={args.turnaround_ppm_a2b},b2a={args.turnaround_ppm_b2a},"
          f"jit={args.turnaround_jitter_ms}ms)")
    print(f"payload={args.payload} ({len(payload)} bytes, md5={payload_md5})\n")

    # PORT-SCOPED pre-clean: kill only whatever lingers on OUR auto-picked ports
    # (a prior crashed run of THIS cell), NOT a system-wide /IM mercury.exe sweep
    # — a concurrent bench experiment on different ports must survive. The picker
    # already verified the quad was free, so this is normally a no-op; it catches
    # a TIME_WAIT/zombie that grabbed a port between probe and launch.
    kill_port_scoped(my_ports, [])
    time.sleep(1)

    logfile = open(os.path.join(MERCURY_ROOT, "sim_arq_channel.log"), "w")
    # CONNECT-DWELL FIX (CONNECT_UNDER_RT_ROOTCAUSE §6.1): the relay log is the
    # SOLE virtual-clock source (read_relay_virtual_seconds). It was a FIXED shared
    # path with NO pre-clean, so:
    #   (1) a concurrent sim wrote into the same file (cross-run vstamp contamination),
    #   (2) a STALE leftover log from a PRIOR run was read at monitor-loop iter 1
    #       BEFORE this run's relay finished `open(args.log,"w")` truncating it. On
    #       a loaded box (the fleet 56-core probe) the relay's interpreter startup
    #       lags the harness's ~9s of launch sleeps, so the first vsecs read returned
    #       the leftover's virtual seconds (observed vsecs=245.3 at real 1s) and the
    #       run INSTANTLY bounded out -> 0% connect. PORT-SCOPE the log (concurrent-
    #       safe) AND pre-clean it (the leftover can never be read).
    relay_log = os.path.join(MERCURY_ROOT,
                             f"sim_channel_relay_{args.port}.log")
    try:
        if os.path.exists(relay_log):
            os.remove(relay_log)
    except OSError:
        pass
    # GAP #1: per-direction airtime breakdown the relay writes on graceful
    # shutdown; the harness reads it below to derive wire_bps_airtime. Scope it
    # to OUR auto-picked port so concurrent cells don't clobber each other's file.
    relay_airtime_json = os.path.join(MERCURY_ROOT,
                                      f"sim_channel_relay_airtime_{args.port}.json")
    try:
        if os.path.exists(relay_airtime_json):
            os.remove(relay_airtime_json)
    except OSError:
        pass
    st = State()
    procs, sockets = [], []
    stop = threading.Event()
    t0 = time.time()
    relay = None
    # Run-bound bookkeeping (hoisted so they exist even if launch raises before
    # the monitor loop). vsecs = last virtual seconds seen; bounded_by = which
    # bound ended the run (virtual_secs / completion / proc_died / vclock_stall /
    # real_watchdog / error).
    vsecs = 0.0
    vclock_ok = False
    bounded_by = "error"
    # CONNECT-DWELL FIX: vstamp at link-up; anchors the DATA virtual budget so the
    # connect handshake's ~75-85 virtual seconds do not dilute rx_bps/dwell. None
    # until the link establishes (or never, if connect times out).
    connect_vsecs = None

    def base_cmd(port, role):
        c = [args.bin, "-m", "ARQ", "-s", str(args.start_cfg), "-W",
             "-p", str(port), "-x", "sim", "-n", "-F", args.compress]
        if not args.no_gearshift:
            c += ["-g"]          # gearshift ON by default; --no-gearshift PINS start-cfg
        else:
            # PIN mode: -Q 0 skips the NB probe so the link starts DIRECT-WB at
            # --start-cfg (main.cc:2858-2859: nb_probe_max==0 && BW_AUTO => start
            # WB, no NB->WB negotiation). Without -Q 0 the session starts NB CFG14
            # and waits for the (now-absent) gearshift probe to upgrade, so CONNECT
            # never reaches the pinned WB tier. Matches the CLAUDE.md -Q 0 recipe.
            # PINs the config so the airtime wire reads a SINGLE-config per-frame
            # rate (GAP #1 / GAP1_AIRTIME_WIRE_VERDICT.md §"climb-run caveat").
            c += ["-Q", "0"]
        if use_robust:
            c += ["-R"]
        return c

    def launch(port, role):
        env = dict(os.environ)
        env["MERCURY_SIM_PORT"] = str(args.port)
        env["MERCURY_SIM_ROLE"] = role
        return subprocess.Popen(base_cmd(port, role), env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    res = {"tx": 0, "rx": 0, "tx_md5_hex": "", "recv_md5_hex": ""}
    try:
        # 1. relay first so the peers can connect immediately.
        relay_cmd = [sys.executable, RELAY, "--port", str(args.port),
                     # --seed and --relay-seed are unified onto args.relay_seed
                     # (see argparse): pass the channel RNG seed to the relay ONCE.
                     # A double "--seed" here would let the relay's argparse take
                     # the LAST one silently -- the kind of collision we resolve.
                     "--seed", str(args.relay_seed),
                     "--snr", str(args.snr), "--loss", str(args.loss),
                     "--profile", args.profile,
                     "--cfo-hz", str(args.cfo_hz),
                     "--phase-noise-deg", str(args.phase_noise_deg),
                     "--barrier-k", str(args.barrier_k),
                     "--wire-stamp", str(args.wire_stamp),
                     "--drift-ppm-a2b", str(args.drift_ppm_a2b),
                     "--drift-ppm-b2a", str(args.drift_ppm_b2a),
                     "--ptt-latency-ms", str(args.ptt_latency_ms),
                     "--ptt-latency-jitter-ms", str(args.ptt_latency_jitter_ms),
                     "--turnaround-ppm-a2b", str(args.turnaround_ppm_a2b),
                     "--turnaround-ppm-b2a", str(args.turnaround_ppm_b2a),
                     "--turnaround-jitter-ms", str(args.turnaround_jitter_ms),
                     "--erase-b2a-burst", str(args.erase_b2a_burst),
                     "--airtime-json", relay_airtime_json,
                     "--log", relay_log]
        if args.turnaround_drift:
            relay_cmd.append("--turnaround-drift")
        if args.cell:
            relay_cmd += ["--cell", args.cell]
        if args.burst:
            relay_cmd.append("--burst")
        relay = subprocess.Popen(relay_cmd)
        time.sleep(1.0)

        # 2. responder (role B), then commander (role A).
        rsp = launch(RSP_PORT, "B")
        procs.append(rsp)
        threading.Thread(target=log_output, args=(rsp, "RSP", logfile, t0, st),
                         daemon=True).start()
        time.sleep(3)
        cmd = launch(CMD_PORT, "A")
        procs.append(cmd)
        threading.Thread(target=log_output, args=(cmd, "CMD", logfile, t0, st),
                         daemon=True).start()
        time.sleep(3)

        # 3. control + data sockets.
        rsp_ctrl = tcp_send(RSP_PORT, ["MYCALL TESTB\r\n", "LISTEN ON\r\n"], "RSP")
        sockets.append(rsp_ctrl)
        time.sleep(1)

        cmd_data = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cmd_data.settimeout(CONNECT_TIMEOUT_S)   # load-tolerant connect
        cmd_data.connect(("127.0.0.1", CMD_PORT + 1))
        sockets.append(cmd_data)
        rsp_data = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rsp_data.settimeout(CONNECT_TIMEOUT_S)   # load-tolerant connect
        rsp_data.connect(("127.0.0.1", RSP_PORT + 1))
        sockets.append(rsp_data)
        threading.Thread(target=tx_thread_fn, args=(cmd_data, stop, res, payload, st),
                         daemon=True).start()
        time.sleep(1)
        threading.Thread(target=rx_thread_fn, args=(rsp_data, stop, res),
                         daemon=True).start()

        cmd_ctrl = tcp_send(CMD_PORT, ["MYCALL TESTA\r\n", "CONNECT TESTA TESTB\r\n"], "CMD")
        sockets.append(cmd_ctrl)

        # ============================================================
        # CONNECT-ANCHORED VIRTUAL-TIME / COMPLETION-BOUND run loop.
        # ============================================================
        # CONNECT-DWELL FIX (CONNECT_UNDER_RT_ROOTCAUSE §6.1): the run is split
        # into a CONNECT phase and a DATA phase, with SEPARATE virtual budgets so
        # the multi-leg LDPC connect handshake (which under RT pacing consumes
        # ~75-85 virtual seconds) does NOT eat the data --secs budget (that was
        # the 0%-connect blocker: --secs 70 expired DURING connect, before any
        # data flowed). The run ends when ANY of:
        #   PRE-CONNECT (st.connected == False):
        #     (a0) the link establishes -> ANCHOR the data budget at this vstamp
        #          (does NOT end the run; transitions to the data phase), OR
        #     (a1) the CONNECT GRACE (--connect-grace virtual seconds) elapses
        #          with no link -> bounded_by="connect_timeout" (matches the HW
        #          teardown discipline: the modem ran its own retry loop to grace).
        #   POST-CONNECT (st.connected == True):
        #     (a)  DATA virtual time (vsecs - connect_vsecs) reaches args.secs, OR
        #     (b)  transfer COMPLETION: delivered_bytes >= --target-bytes [I7].
        #   ALWAYS (both phases):
        #     (c)  a process died, OR
        #     (d)  a GENEROUS real-time watchdog fires (true wedge guard only), OR
        #     (e)  the virtual clock STALLS (relay stopped forwarding = wedge).
        # Bound (d)/(e) NEVER truncate a healthy-but-slow climb. The decisive
        # load-independence property is preserved: same seed -> same virtual data
        # budget anchored at connect -> identical DATA trajectory idle vs hammered.
        start = time.time()
        # the real watchdog must cover BOTH the connect grace AND the data budget
        # (both are virtual-second budgets; under RT, real ~= virtual, FTRT real <<
        # virtual, so the generous mult still only fires on a true wedge).
        real_watchdog_s = max((args.secs + args.connect_grace) * REAL_WATCHDOG_MULT,
                              REAL_WATCHDOG_FLOOR)
        print(f"\n=== monitoring: connect-grace {args.connect_grace}s VIRTUAL, "
              f"then DATA budget {args.secs}s VIRTUAL anchored at connect "
              f"(real watchdog {real_watchdog_s:.0f}s, "
              f"target_bytes={args.target_bytes or 'none'}) ===\n")
        dead = False
        bounded_by = "virtual_secs"     # default expected exit (overwrites hoist)
        last_vsecs = -1.0
        last_vadvance_real = time.time()
        last_report_v = 0.0
        while not dead:
            time.sleep(1)
            now = time.time()
            real_elapsed = now - start

            vsecs, vclock_ok = read_relay_virtual_seconds(relay_log)

            # --- CONNECT-PHASE handling (anchor or grace) --------------------
            if connect_vsecs is None:
                if st.connected:
                    # (a0) link is up -> anchor the DATA budget at the current
                    # vstamp. Do NOT end the run; the data phase starts here.
                    connect_vsecs = vsecs if vclock_ok else 0.0
                    print(f"[CONNECT] link established at vsecs={connect_vsecs:.1f} "
                          f"(real {real_elapsed:.0f}s) -> DATA budget {args.secs}s "
                          f"VIRTUAL starts now")
                    sys.stdout.flush()
                elif vclock_ok and vsecs >= args.connect_grace:
                    # (a1) connect grace exhausted without a link.
                    bounded_by = "connect_timeout"
                    print(f"[BOUND] connect grace exhausted: vsecs={vsecs:.1f} >= "
                          f"{args.connect_grace} (real {real_elapsed:.0f}s), "
                          f"link never established")
                    break
                # else: still connecting, within grace -> fall through to the
                # ALWAYS guards (proc-died / wedge / watchdog) only.

            # --- (a) DATA virtual-time budget (anchored at connect) ----------
            if connect_vsecs is not None and vclock_ok:
                data_vsecs = vsecs - connect_vsecs
                if data_vsecs >= args.secs:
                    bounded_by = "virtual_secs"
                    print(f"[BOUND] data virtual budget reached: data_vsecs="
                          f"{data_vsecs:.1f} >= {args.secs} "
                          f"(total vsecs={vsecs:.1f}, real {real_elapsed:.0f}s)")
                    break

            # --- (b) transfer completion (byte target + md5) -----------------
            if args.target_bytes > 0 and res["rx"] >= args.target_bytes:
                ref = _md5_of_looped_prefix(payload, res["rx"])
                if res.get("recv_md5_hex") == ref:
                    bounded_by = "completion"
                    print(f"[BOUND] transfer complete: delivered={res['rx']} "
                          f">= {args.target_bytes}, md5 match "
                          f"(vsecs={vsecs:.1f}, real {real_elapsed:.0f}s)")
                    break

            # --- (c) a process exited ----------------------------------------
            for i, p in enumerate(procs):
                if p.poll() is not None:
                    print(f"[WARN] {'RSP' if i == 0 else 'CMD'} exited code {p.returncode}")
                    dead = True
                    bounded_by = "proc_died"
            if dead:
                break

            # --- (e) virtual-clock STALL (relay wedged: no forwarding) --------
            if vclock_ok and vsecs > last_vsecs + 1e-6:
                last_vsecs = vsecs
                last_vadvance_real = now
            elif vclock_ok and (now - last_vadvance_real) > VCLOCK_STALL_REAL_S:
                # Virtual time has been frozen for VCLOCK_STALL_REAL_S real
                # seconds while running -> the channel is wedged, not slow.
                bounded_by = "vclock_stall"
                print(f"[BOUND][WEDGE] virtual clock frozen at vsecs={vsecs:.1f} "
                      f"for {now - last_vadvance_real:.0f}s real -> aborting wedge")
                dead = True
                break

            # --- (d) GENEROUS real-time watchdog (true-wedge ceiling only) ----
            if real_elapsed > real_watchdog_s:
                bounded_by = "real_watchdog"
                print(f"[BOUND][WATCHDOG] real {real_elapsed:.0f}s exceeded "
                      f"{real_watchdog_s:.0f}s while vsecs={vsecs:.1f} "
                      f"(< {args.secs}) -> ceiling guard, run was abnormally slow")
                dead = True
                break

            # progress line (every ~10 virtual seconds)
            if vclock_ok and vsecs >= last_report_v + 10.0:
                last_report_v = vsecs
                if connect_vsecs is None:
                    phase = f"CONNECTING ({vsecs:.0f}/{args.connect_grace}s grace)"
                else:
                    phase = f"DATA ({vsecs - connect_vsecs:.0f}/{args.secs}s)"
                print(f"[T+v{vsecs:7.1f}] (real {real_elapsed:6.0f}s) "
                      f"rx={res['rx']}B cfg={cfg_name(st.switch_seq[-1]) if st.switch_seq else '-'} "
                      f"{phase}")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for s in sockets:
            try:
                s.close()
            except OSError:
                pass
        spawned_pids = []
        for p in procs:
            spawned_pids.append(p.pid)
            try:
                p.kill()
            except OSError:
                pass
        if relay:
            spawned_pids.append(relay.pid)
            # GAP #1: give the relay a moment to notice its peer sockets closed
            # (its readers hit "source closed" -> stop.set() -> graceful shutdown,
            # which writes the airtime-json + 'relay done' line) BEFORE the hard
            # terminate(), which would skip that emit. The mercury procs were just
            # killed above, so the relay's source is already gone.
            try:
                relay.wait(timeout=3)
            except (subprocess.TimeoutExpired, OSError):
                pass
            try:
                relay.terminate()
            except OSError:
                pass
        # PORT-SCOPED teardown: kill our spawned PIDs + anything still holding our
        # auto-picked ports. NEVER a /IM mercury.exe sweep — the BK8/sibling bench
        # run shares this machine on different ports and must not be touched.
        kill_port_scoped(my_ports, spawned_pids)
        logfile.close()

    # ---- delivered-byte md5 (I7) ----
    delivered_bytes = res["rx"]
    recv_md5 = res.get("recv_md5_hex") or hashlib.md5(b"").hexdigest()
    # md5 of the same-length PREFIX of the EXACT TX byte stream. The RX side
    # receives a prefix of what TX sent (looped payload), so the integrity
    # reference is md5(tx_stream[:delivered_bytes]). We reconstruct that prefix
    # deterministically from the looped payload (md5 of bytes is position-exact).
    tx_md5 = res.get("tx_md5_hex") or hashlib.md5(b"").hexdigest()
    md5_ref = _md5_of_looped_prefix(payload, delivered_bytes)
    md5_match = (delivered_bytes > 0 and recv_md5 == md5_ref)

    # ---- verdict ----
    seq = st.switch_seq
    names = [cfg_name(c) for c in seq]
    ofdm_ids = [c for c in seq if 0 <= c <= 16]
    final = seq[-1] if seq else None
    # Monotonic config RANK so ROBUST_0/1/2 sort BELOW CONFIG_0..16 (their raw
    # ids are 100-102, numerically above 16). rank: ROBUST_0=0, ROBUST_1=1,
    # ROBUST_2=2, CONFIG_0=3, ... CONFIG_16=19.
    def rank(c):
        return (c - 100) if c >= 100 else (c + 3)
    ranks = [rank(c) for c in seq]

    peak = None
    if seq:
        peak = seq[ranks.index(max(ranks))]

    # steady = most common id in the LAST third of the timeline
    steady = None
    if seq:
        tail = seq[max(0, len(seq) * 2 // 3):]
        steady = max(set(tail), key=tail.count)

    # over-climb -> collapse: reached a HIGH OFDM config (>= CONFIG_10) and then,
    # ANYWHERE later in the timeline, dropped to a LOW/ROBUST config (a big
    # downward step). Scan the whole sequence, not just after the global peak —
    # the HW signature is an oscillation (CONFIG_16 -> ROBUST_0 -> climb again).
    HIGH = rank(13)        # CONFIG_13 rank
    LOW = rank(4)          # CONFIG_4 rank (>= this drop counts as collapse)
    collapsed = False
    seen_high = False
    for r in ranks:
        if r >= rank(10):
            seen_high = True
        elif seen_high and r <= LOW:
            collapsed = True
            break

    # deep-SNR stall: never reached a USABLE OFDM config (>= CONFIG_1) AND
    # throughput is tiny (link pinned at robust / CONFIG_0).
    #
    # RATE DENOMINATOR IS VIRTUAL TIME, not wall time. rx_bps measured against
    # real seconds is host-load-dependent (a hammered run takes more real time
    # for the same delivered bytes -> a LOWER, meaningless bps). The delivered
    # bytes accrue in VIRTUAL time, so the only load-independent rate is
    # bytes*8 / virtual_seconds. Re-read the relay log AFTER teardown for the
    # final vstamp (the log persists past relay.terminate()).
    final_vsecs, final_vok = read_relay_virtual_seconds(relay_log)
    if final_vok and final_vsecs > 0:
        virtual_secs = final_vsecs
    else:
        virtual_secs = vsecs if vsecs > 0 else 0.0
    real_secs = max(1.0, time.time() - t0)
    # CONNECT-DWELL FIX: rx_bps is delivered bytes over the DATA virtual seconds
    # (vstamp since link-up), NOT the whole window — the ~75-85 virtual-second
    # connect handshake carries zero payload and would otherwise halve the rate.
    # If the link never established (connect_vsecs is None) there is no data phase;
    # fall back to the whole virtual window (rx is ~0 anyway).
    if connect_vsecs is not None and virtual_secs > connect_vsecs:
        data_virtual_secs = virtual_secs - connect_vsecs
    else:
        data_virtual_secs = virtual_secs
    dwell = max(1.0, data_virtual_secs)   # VIRTUAL DATA dwell (load-independent)
    rx_bps = res["rx"] * 8 / dwell
    reached_ofdm = any(1 <= c <= 16 for c in seq)
    stalled = (not reached_ofdm) and rx_bps < 200

    # ---- GAP #1: AIRTIME-DERIVED per-frame wire rate ------------------------
    # rx_bps above divides delivered bytes by REAL WALL-CLOCK over the whole
    # window — diluted by the ROBUST->CFGn climb ramp, by idle/turnaround, and by
    # the harness setup time. That made the sim look ~7-10x under HW in the
    # CFG16-hold verdict, but that was an apples-to-oranges comparison artifact:
    # the per-frame CHANNEL wire rate is delivered_bytes*8 / FRAME-AIRTIME, where
    # frame-airtime = the SIGNAL chunks the relay actually carried in the
    # delivering direction (data flows CMD/A -> RSP/B == relay direction a2b).
    # This number maps to the modem's own rbc (Tf airtime model) and to HW
    # (CFG15 rbc=3348, bench-8 sustained 3060). See the relay's --airtime-json.
    # PIN a config (--no-gearshift) to read a SINGLE tier's per-frame wire.
    airtime_secs = None
    wire_bps_airtime = None
    airtime_signal_chunks = None
    airtime_silence_chunks = None
    airtime_total_virtual_s = None
    try:
        if os.path.exists(relay_airtime_json):
            with open(relay_airtime_json) as af:
                at = json.load(af)
            d_dir = at.get("a2b", {})   # CMD(A) -> RSP(B): the delivering direction
            airtime_signal_chunks = d_dir.get("signal_chunks")
            airtime_silence_chunks = d_dir.get("silence_chunks")
            airtime_secs = d_dir.get("signal_airtime_s")
            airtime_total_virtual_s = d_dir.get("total_virtual_s")
            if airtime_secs and airtime_secs > 0:
                wire_bps_airtime = round(res["rx"] * 8 / airtime_secs, 1)
    except (OSError, ValueError, KeyError):
        pass

    print("\n========== SUMMARY ==========")
    print(f"connected         : {st.connected}")
    print(f"config switch_seq : {names}")
    print(f"peak_config       : {cfg_name(peak) if peak is not None else None}")
    print(f"steady_config     : {cfg_name(steady) if steady is not None else None}")
    print(f"final_config      : {cfg_name(final) if final is not None else None}")
    # virtual-time-runbound: dwell/rx_bps are now measured in VIRTUAL seconds
    # (load-independent); bounded_by names which run-bound ended the run.
    print(f"bounded_by        : {bounded_by}")
    print(f"virtual_secs      : {virtual_secs:.1f}s  (real {real_secs:.1f}s, "
          f"FTRT {real_secs/max(1e-6,virtual_secs):.2f}x real/virtual)")
    print(f"connect_vsecs     : "
          f"{('%.1fs' % connect_vsecs) if connect_vsecs is not None else 'NEVER CONNECTED'}"
          f"   data_virtual_secs: {data_virtual_secs:.1f}s")
    print(f"client rx bytes   : {res['rx']}  (~{rx_bps:.0f} bps over "
          f"{dwell:.0f}s DATA-VIRTUAL)")
    # monitor: per-frame CHANNEL wire from the relay airtime-json (HW-representative
    # axis), independent of the climb-ramp-diluted virtual-clock rx_bps above.
    if wire_bps_airtime is not None:
        print(f"wire_bps_airtime  : {wire_bps_airtime} bps  "
              f"(per-frame CHANNEL wire = rx*8 / {airtime_secs:.1f}s frame-airtime; "
              f"sig={airtime_signal_chunks} sil={airtime_silence_chunks} chunks a2b)")
        print(f"                    [HW-representative axis: maps to rbc/HW, NOT the "
              f"climb-ramp-diluted {rx_bps:.0f} bps]")
    else:
        print("wire_bps_airtime  : (unavailable -- relay airtime-json not found)")
    print(f"client tx bytes   : {res['tx']}")
    print(f"delivered_bytes   : {delivered_bytes}")
    print(f"recv_md5          : {recv_md5}")
    print(f"md5_ref(prefix)   : {md5_ref}")
    print(f"md5_match         : {md5_match}")
    print(f"OFDM configs hit  : {sorted(set(ofdm_ids))}")
    print(f"retx: cmd_retx={st.n_cmd_retx} frames={st.cmd_retx_frames} "
          f"v2={st.n_cmd_retx_v2} sack_retx_lines={st.n_sack_retx_lines} "
          f"sack_retx_frames={st.sack_retx_frames} "
          f"nResent_max={st.nresent_data_max} chase_fire={st.n_chase_fire}")
    print()
    print(f"REPRO over-climb->collapse : {collapsed}")
    print(f"REPRO deep-SNR stall       : {stalled}")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({
                "snr": args.snr, "cell": args.cell, "profile": args.profile,
                "cfo_hz": args.cfo_hz, "phase_noise_deg": args.phase_noise_deg,
                "loss": args.loss, "burst": args.burst,
                "secs": args.secs, "start_cfg": args.start_cfg,
                # --seed/--relay-seed are unified onto args.relay_seed; emit both
                # JSON keys (same value) so consumers of either key keep working.
                "relay_seed": args.relay_seed,
                "seed": args.relay_seed,
                "target_bytes": args.target_bytes,
                # auto-picked ports (so cleanup/inspection is PORT-SCOPED, never
                # a system-wide kill — multiple concurrent sims are collision-proof)
                "ports": chosen_ports,
                "drift_ppm_a2b": args.drift_ppm_a2b,
                "drift_ppm_b2a": args.drift_ppm_b2a,
                "turnaround_drift": args.turnaround_drift,
                "turnaround_ppm_a2b": args.turnaround_ppm_a2b,
                "turnaround_ppm_b2a": args.turnaround_ppm_b2a,
                "turnaround_jitter_ms": args.turnaround_jitter_ms,
                "ptt_latency_ms": args.ptt_latency_ms,
                "ptt_latency_jitter_ms": args.ptt_latency_jitter_ms,
                "connected": st.connected,
                "switch_seq": names,
                "peak_config": cfg_name(peak) if peak is not None else None,
                "steady_config": cfg_name(steady) if steady is not None else None,
                "final_config": cfg_name(final) if final is not None else None,
                "rx_bytes": res["rx"], "tx_bytes": res["tx"],
                "rx_bps": round(rx_bps, 1),
                # --- VIRTUAL-time run bound (host-load-independence) ---
                # virtual_secs is the load-INDEPENDENT clock the trajectory and
                # rx_bps are measured against; wall_secs is the real time the run
                # took (varies with host load); bounded_by names which bound ended
                # the run. wall_secs == virtual_secs ONLY by coincidence; a
                # healthy FTRT run has wall_secs < virtual_secs.
                "virtual_secs": round(virtual_secs, 1),
                # CONNECT-DWELL FIX: connect_vsecs = the virtual second at which
                # the link established (None if it never connected); data_virtual_secs
                # = the virtual airtime AFTER connect (the rx_bps denominator). The
                # connect handshake under RT consumes ~75-85 virtual seconds that are
                # now EXCLUDED from the data-rate measurement.
                "connect_vsecs": (round(connect_vsecs, 1)
                                  if connect_vsecs is not None else None),
                "data_virtual_secs": round(data_virtual_secs, 1),
                "wall_secs": round(real_secs, 1),
                "bounded_by": bounded_by,
                "barrier_k": args.barrier_k,
                "wire_stamp": args.wire_stamp,
                "no_gearshift": args.no_gearshift,
                # GAP #1: HW-representative per-frame CHANNEL wire rate (rx*8 /
                # delivering-direction frame-airtime). Undiluted by climb ramp /
                # idle / setup. Reconciles with the modem rbc + HW. None if the
                # relay airtime-json was unavailable. PIN a config (--no-gearshift)
                # to read a single tier's per-frame wire (else it's a config mix).
                "wire_bps_airtime": wire_bps_airtime,
                "airtime_secs": airtime_secs,
                "airtime_signal_chunks": airtime_signal_chunks,
                "airtime_silence_chunks": airtime_silence_chunks,
                "airtime_total_virtual_s": airtime_total_virtual_s,
                # --- I7 delivered-byte integrity ---
                "payload": os.path.basename(args.payload),
                "payload_bytes": len(payload),
                "payload_md5": payload_md5,
                "delivered_bytes": delivered_bytes,
                "tx_md5": tx_md5,
                "recv_md5": recv_md5,
                "recv_md5_ref": md5_ref,
                "md5_match": md5_match,
                # --- I7 retransmission + chase-fire counters ---
                "n_cmd_retx": st.n_cmd_retx,
                "cmd_retx_frames": st.cmd_retx_frames,
                "n_cmd_retx_v2": st.n_cmd_retx_v2,
                "n_sack_retx_lines": st.n_sack_retx_lines,
                "sack_retx_frames": st.sack_retx_frames,
                "nresent_data_max": st.nresent_data_max,
                "n_chase_fire": st.n_chase_fire,
                # --- verdict ---
                "repro_overclimb_collapse": collapsed,
                "repro_deep_stall": stalled,
            }, f, indent=1)
        print(f"\nwrote {args.json}")

    return 0


def _md5_of_looped_prefix(payload, nbytes):
    """md5 of the first `nbytes` of the payload looped end-to-end (the exact
    byte stream the TX thread produces, truncated to the delivered length)."""
    if nbytes <= 0:
        return hashlib.md5(b"").hexdigest()
    md5 = hashlib.md5()
    plen = len(payload)
    full, rem = divmod(nbytes, plen)
    for _ in range(full):
        md5.update(payload)
    if rem:
        md5.update(payload[:rem])
    return md5.hexdigest()


if __name__ == "__main__":
    sys.exit(main())

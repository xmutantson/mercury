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
  --port N          relay base TCP port (default 52100)
  --start-cfg N     mercury -s config (default 100 = ROBUST_0)
  --robust          pass -R (start in robust tier; default on for start-cfg>=100)
  --compress on|off (default off, so we read RAW PHY gearshift behavior)
  --payload PATH    TX payload file (default payload_incompressible_64k.bin)
  --json PATH       write a machine-readable result summary

NOTE: this harness taskkills ALL mercury.exe at start AND teardown (process-
global). Run cells SEQUENTIALLY — two concurrent invocations will kill each
other's modem processes. The calibrator drives one cell at a time for this
reason.

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

RSP_PORT = 7002
CMD_PORT = 7006

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


def tcp_send(port, commands, label, retries=20, delay=1.0):
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
    sock.settimeout(30)
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
        except (socket.timeout, ConnectionError, OSError):
            break
        time.sleep(0.03)


def rx_thread_fn(sock, stop, res):
    """Accumulate delivered bytes + their streaming md5 (recv_md5)."""
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
    # deprecated flat-fade knobs (superseded by --profile); kept for compat.
    ap.add_argument("--fade-hz", type=float, default=0.0, help=argparse.SUPPRESS)
    ap.add_argument("--fade-depth", type=float, default=0.0, help=argparse.SUPPRESS)
    ap.add_argument("--secs", type=int, default=180)
    ap.add_argument("--port", type=int, default=52100)
    ap.add_argument("--start-cfg", type=int, default=100)
    ap.add_argument("--robust", action="store_true", default=None)
    ap.add_argument("--compress", default="off")
    ap.add_argument("--payload", default=DEFAULT_PAYLOAD,
                    help="TX payload file (default payload_incompressible_64k.bin, "
                         "a fixed high-entropy file so compression is bypassed and "
                         "the wire carries RAW PHY rate for the delivered-bps A/B)")
    ap.add_argument("--barrier-k", type=int, default=1,
                    help="conservative-PDES lockstep window (chunks) passed to the "
                         "relay. K=1 = strict lockstep (default); relax to 4/8 if "
                         "the strict window starves CONNECT/delivery.")
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
          f"barrier_k={args.barrier_k}")
    print(f"payload={args.payload} ({len(payload)} bytes, md5={payload_md5})\n")

    os.system("taskkill /F /IM mercury.exe >nul 2>&1")
    time.sleep(1)

    logfile = open(os.path.join(MERCURY_ROOT, "sim_arq_channel.log"), "w")
    relay_log = os.path.join(MERCURY_ROOT, "sim_channel_relay.log")
    st = State()
    procs, sockets = [], []
    stop = threading.Event()
    t0 = time.time()
    relay = None

    def base_cmd(port, role):
        c = [args.bin, "-m", "ARQ", "-s", str(args.start_cfg), "-W",
             "-p", str(port), "-x", "sim", "-n", "-g", "-F", args.compress]
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
                     "--snr", str(args.snr), "--loss", str(args.loss),
                     "--profile", args.profile,
                     "--cfo-hz", str(args.cfo_hz),
                     "--phase-noise-deg", str(args.phase_noise_deg),
                     "--barrier-k", str(args.barrier_k),
                     "--wire-stamp", str(args.wire_stamp),
                     "--log", relay_log]
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
        cmd_data.settimeout(5)
        cmd_data.connect(("127.0.0.1", CMD_PORT + 1))
        sockets.append(cmd_data)
        rsp_data = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rsp_data.settimeout(5)
        rsp_data.connect(("127.0.0.1", RSP_PORT + 1))
        sockets.append(rsp_data)
        threading.Thread(target=tx_thread_fn, args=(cmd_data, stop, res, payload, st),
                         daemon=True).start()
        time.sleep(1)
        threading.Thread(target=rx_thread_fn, args=(rsp_data, stop, res),
                         daemon=True).start()

        cmd_ctrl = tcp_send(CMD_PORT, ["MYCALL TESTA\r\n", "CONNECT TESTA TESTB\r\n"], "CMD")
        sockets.append(cmd_ctrl)

        start = time.time()
        print(f"\n=== monitoring {args.secs}s ===\n")
        dead = False
        while time.time() - start < args.secs and not dead:
            time.sleep(1)
            for i, p in enumerate(procs):
                if p.poll() is not None:
                    print(f"[WARN] {'RSP' if i == 0 else 'CMD'} exited code {p.returncode}")
                    dead = True
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for s in sockets:
            try:
                s.close()
            except OSError:
                pass
        for p in procs:
            try:
                p.kill()
            except OSError:
                pass
        if relay:
            try:
                relay.terminate()
            except OSError:
                pass
        os.system("taskkill /F /IM mercury.exe >nul 2>&1")
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
    dwell = max(1.0, time.time() - t0)
    rx_bps = res["rx"] * 8 / dwell
    reached_ofdm = any(1 <= c <= 16 for c in seq)
    stalled = (not reached_ofdm) and rx_bps < 200

    print("\n========== SUMMARY ==========")
    print(f"connected         : {st.connected}")
    print(f"config switch_seq : {names}")
    print(f"peak_config       : {cfg_name(peak) if peak is not None else None}")
    print(f"steady_config     : {cfg_name(steady) if steady is not None else None}")
    print(f"final_config      : {cfg_name(final) if final is not None else None}")
    print(f"client rx bytes   : {res['rx']}  (~{rx_bps:.0f} bps over {dwell:.0f}s)")
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
                "connected": st.connected,
                "switch_seq": names,
                "peak_config": cfg_name(peak) if peak is not None else None,
                "steady_config": cfg_name(steady) if steady is not None else None,
                "final_config": cfg_name(final) if final is not None else None,
                "rx_bytes": res["rx"], "tx_bytes": res["tx"],
                "rx_bps": round(rx_bps, 1),
                "wall_secs": round(dwell, 1),
                "barrier_k": args.barrier_k,
                "wire_stamp": args.wire_stamp,
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

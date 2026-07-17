#!/usr/bin/env python3
"""
arq_realaudio.py - drive TWO stock `-x alsa` Mercury instances through the
real-audio IONOS bridge (realaudio_bridge_s32.py) over snd-aloop.

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
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ra_cleanup import scoped_cleanup  # concurrency-safe per-run cleanup

CONNECT_RE = re.compile(r"link_status:Connected to")
DISC_RE = re.compile(r"link_status:Disconnected|DISCONNECTED")
NRECV_RE = re.compile(r"stats\.nReceived_data=\s*(\d+)")
CFG_RE = re.compile(r"load_configuration\((\d+)\)\s+current=(\d+)")
BREAK_RE = re.compile(r"\[BREAK\] Block failure")
# AEAD auth-fail markers: the env-gated nonce trace (DEC-AUTHFAIL) AND any
# generic crypto auth-failure / PSK-mismatch the modem already logs.
AUTHFAIL_RE = re.compile(r"NONCE-TRACE\] DEC-AUTHFAIL|auth.?fail|PSK mismatch|decrypt.*fail",
                         re.IGNORECASE)
NONCE_ENC_RE = re.compile(r"NONCE-TRACE\] ENC dir=(\d+) idx=(\d+)")
NONCE_DECOK_RE = re.compile(r"NONCE-TRACE\] DEC-OK dir=(\d+) idx=(\d+)")
ENC_ACT_RE = re.compile(r"\[CRYPTO\] Encryption ACTIVATED")
# PRODUCTION crypto emissions — the lines the modem ACTUALLY logs on the live
# path (the NONCE-TRACE lines above are env-gated debug and are never emitted,
# so the durable counters read all-0 without these). [CRYPTO-TX] is the AEAD
# seal, [CRYPTO-RX] "Decrypted: N -> M bytes OK" is a successful open.
CRYPTO_TX_RE = re.compile(r"\[CRYPTO-TX\] Encrypting \d+ bytes, wire_bsi=\d+ index=(\d+) dir=(\d+)")
CRYPTO_RX_OK_RE = re.compile(r"\[CRYPTO-RX\] Decrypted: \d+ -> \d+ bytes OK")


class State:
    def __init__(self):
        self.connected = False
        self.cmd_connected = False
        self.rsp_connected = False
        self.disconnected = False
        self.rsp_nreceived = 0
        self.cmd_nreceived = 0
        self.breaks = 0
        self.configs_seen = set()
        self.authfails = 0
        self.enc_activated = False
        # nonce trace: (dir,idx) -> count of ENC emissions; >1 for any key = reuse
        self.enc_nonces = {}
        self.dec_ok = 0
        self.nonce_reuse = 0
        self.lock = threading.Lock()


def log_output(proc, label, logfile, t0, st):
    try:
        for line in iter(proc.stdout.readline, b''):
            text = line.decode("utf-8", "replace").rstrip()
            logfile.write(f"[T+{time.time()-t0:08.3f}] [{label}] {text}\n")
            logfile.flush()
            if CONNECT_RE.search(text):
                with st.lock:
                    st.connected = True
                    if label == "CMD":
                        st.cmd_connected = True
                    else:
                        st.rsp_connected = True
            if DISC_RE.search(text):
                st.disconnected = True
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
                with st.lock:
                    st.configs_seen.add(int(m.group(2)))
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


def tcp_send(port, commands, retries=40, delay=1.0):
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


def tx_thread_fn(sock, stop, res, payload_total):
    chunk = bytes(range(256)) * 8  # 2048 bytes varied data (== canonical)
    sock.settimeout(30)
    while not stop.is_set() and res["tx"] < payload_total:
        try:
            sock.sendall(chunk)
            res["tx"] += len(chunk)
        except (socket.timeout, ConnectionError, OSError):
            break
        time.sleep(0.03)


def rx_thread_fn(sock, stop, res):
    sock.settimeout(2)
    while not stop.is_set():
        try:
            d = sock.recv(8192)
            if not d:
                break
            res["rx"] += len(d)
        except socket.timeout:
            continue
        except (ConnectionError, OSError):
            break


def dev(card, devno, sub):
    return f"hw:{card},{devno},{sub}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default="/home/kameron/raspeed/mercury")
    ap.add_argument("--bridge", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "realaudio_bridge_s32.py"))
    ap.add_argument("--start-cfg", type=int, default=100)
    ap.add_argument("--no-gearshift", action="store_true",
                    help="omit -g so --start-cfg HOLDS (pins; adds -Q 0 to start direct-WB)")
    ap.add_argument("--secs", type=int, default=120)
    ap.add_argument("--payload", type=int, default=4096)
    ap.add_argument("--passthrough", action="store_true")
    ap.add_argument("--snr", type=float, default=30.0)
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
    args = ap.parse_args()

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
    res = {"tx": 0, "rx": 0}
    connected_at = None

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
        threading.Thread(target=log_output, args=(p, label, logfile, t0, st),
                         daemon=True).start()
        return p

    try:
        # 1. bridge first (opens the 4 loopback subdevices for THIS run).
        bcmd = [sys.executable, args.bridge,
                "--fwd-cap", fwd_cap, "--fwd-play", fwd_play,
                "--rev-cap", rev_cap, "--rev-play", rev_play,
                "--snr", str(args.snr), "--profile", args.profile,
                "--cfo-hz", str(args.cfo_hz),
                "--phase-noise-deg", str(args.phase_noise_deg),
                "--fade-depth-db", str(args.fade_depth_db),
                "--seed", str(args.seed),
                "--cap-periods", str(args.cap_periods),
                "--play-periods", str(args.play_periods),
                "--prime-periods", str(args.prime_periods),
                "--statsfile", os.path.join(args.logdir, f"bridge_{args.tag}_stats.json")]
        if args.passthrough:
            bcmd += ["--passthrough"]
        if args.cell:
            bcmd += ["--cell", args.cell]
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

        threading.Thread(target=rx_thread_fn, args=(rsp_data, stop, res),
                         daemon=True).start()
        time.sleep(0.5)

        cmd_ctrl = tcp_send(cmd_port, ["MYCALL TESTA\r\n", "CONNECT TESTA TESTB\r\n"])
        sockets.append(cmd_ctrl)

        connect_deadline = time.time() + min(args.secs, 90)
        while time.time() < connect_deadline and not st.connected:
            time.sleep(0.5)
        connected_at = (time.time() - t0) if st.connected else None

        if st.connected:
            threading.Thread(target=tx_thread_fn,
                             args=(cmd_data, stop, res, args.payload),
                             daemon=True).start()

        start = time.time()
        dead = False
        while time.time() - start < args.secs and not dead:
            time.sleep(1)
            if st.connected and res["rx"] >= args.payload:
                break
            for p in procs:
                if p.poll() is not None:
                    dead = True
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
                p.kill()
            except OSError:
                pass
        if bridge:
            try:
                bridge.terminate()
                bridge.wait(timeout=3)
            except Exception:
                try:
                    bridge.kill()
                except Exception:
                    pass
        logfile.close()

    dwell = max(1.0, time.time() - t0)
    configs_sorted = sorted(st.configs_seen)
    # max_config_reached = the highest config the gearshift CLIMBED to. ROBUST
    # IDs (100/101/102) are the MFSK floor; WB OFDM IDs are 0..16. A climb past
    # the ROBUST floor (any id < 100 appearing, or id>100 within ROBUST) is THE
    # signal. We report both the raw set and the max, plus a climbed-past-floor
    # boolean for quick A/B reading.
    wb_seen = [c for c in configs_sorted if c < 100]
    max_config = max(configs_sorted) if configs_sorted else None
    climbed_past_robust0 = bool(wb_seen) or any(c > 100 for c in configs_sorted)
    result = {
        "tag": args.tag, "arm": args.arm, "env": args.env,
        "passthrough": args.passthrough,
        "snr": args.snr, "cell": args.cell, "profile": args.profile,
        "seed": args.seed, "start_cfg": args.start_cfg,
        "card": args.card, "subs": subs,
        "rsp_port": rsp_port, "cmd_port": cmd_port,
        "no_gearshift": args.no_gearshift,
        "connected": st.connected,
        "cmd_connected": st.cmd_connected, "rsp_connected": st.rsp_connected,
        "connected_at_s": round(connected_at, 2) if connected_at else None,
        "tx_bytes": res["tx"], "rx_bytes": res["rx"],
        "payload_target": args.payload,
        "delivered_full": res["rx"] >= args.payload,
        "rsp_nreceived_frames": st.rsp_nreceived,
        "cmd_nreceived_frames": st.cmd_nreceived,
        "breaks": st.breaks,
        "encrypt": args.encrypt,
        "enc_activated": st.enc_activated,
        "aead_authfails": st.authfails,
        "nonce_enc_count": sum(st.enc_nonces.values()),
        "nonce_reuse_count": st.nonce_reuse,
        "dec_ok_count": st.dec_ok,
        "configs_seen": configs_sorted,
        "max_config_reached": max_config,
        "wb_configs_seen": wb_seen,
        "climbed_past_robust0": climbed_past_robust0,
        "rx_bps_wall": round(res["rx"] * 8 / dwell, 1),
        "wall_secs": round(dwell, 1),
    }
    print(json.dumps(result))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())

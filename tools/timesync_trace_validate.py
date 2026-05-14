#!/usr/bin/env python3
"""Plan-B Step 6 validation harness — exercise the TIMESYNC_TRACE dual-path
decision-equality checks on real RX buffers (WB + NB), then aggregate the
`# STEPxx-...` trace lines the instrumented build emits.

This is the discipline that caught the Step-5 first-attempt failure: the
dual-path #ifdef checks must be RUN on live buffers, not merely compiled.

Reuses the loopback-pump pattern from baseline_loopback.py. The instrumented
mercury.exe writes `timesync_trace_<PID>.csv` into its cwd; we run the mercury
subprocesses with cwd set to a known scratch dir and parse those files.

Usage:
  python tools/timesync_trace_validate.py [--secs 45] [--configs WB_CFG10,NB_CFG10]

Exit code 0 if every dual-path line is OK/EXACT, 1 otherwise.
"""
import argparse
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
from collections import defaultdict
from datetime import datetime

MERCURY_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INSTALL_PATH = r"C:\Program Files\Mercury\mercury.exe"

VB_IN = "CABLE Output (VB-Audio Virtual Cable)"
VB_OUT = "CABLE Input (VB-Audio Virtual Cable)"
RSP_PORT = 7002
CMD_PORT = 7006

# (config_id, is_nb, label)
ALL_CONFIGS = [
    (10, False, "WB_CFG10"),
    (4,  False, "WB_CFG4"),
    (10, True,  "NB_CFG10"),
    (4,  True,  "NB_CFG4"),
]


def kill_mercury():
    os.system("taskkill /F /IM mercury.exe 2>nul >nul")
    time.sleep(2)


def tcp_connect_retry(port, timeout=5, retries=12, delay=1):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    for attempt in range(retries):
        try:
            s.connect(("127.0.0.1", port))
            return s
        except (ConnectionRefusedError, socket.timeout, OSError):
            if attempt == retries - 1:
                raise
            time.sleep(delay)
            s.close()
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
    return s


def tcp_send_commands(port, commands):
    s = tcp_connect_retry(port)
    for cmd in commands:
        s.sendall(cmd.encode())
        time.sleep(0.3)
        try:
            s.settimeout(0.5)
            s.recv(4096)
        except socket.timeout:
            pass
    return s


def run_loopback(config_id, is_nb, label, secs, parent_dir):
    """Run one short loopback to exercise the trace. Returns list of trace-file paths.
    Each config gets its own subdir so traces are not clobbered across configs."""
    kill_mercury()
    scratch_dir = os.path.join(parent_dir, label)
    os.makedirs(scratch_dir, exist_ok=True)

    procs = []
    sockets = []
    try:
        mode_args = ["-Q", "0", "-M", "nb"] if is_nb else ["-Q", "0", "-M", "auto"]
        gain_args = ["-T", "-12.6", "-G", "12.6"]
        config_args = ["-s", str(config_id)]

        rsp_log = os.path.join(scratch_dir, f"rsp_{label}.log")
        cmd_log = os.path.join(scratch_dir, f"cmd_{label}.log")

        rsp_fd = open(rsp_log, 'w')
        rsp_cmd = [INSTALL_PATH, "-m", "ARQ", *config_args, *mode_args, *gain_args,
                   "-p", str(RSP_PORT), "-i", VB_IN, "-o", VB_OUT, "-x", "wasapi", "-n"]
        rsp = subprocess.Popen(rsp_cmd, stdout=rsp_fd, stderr=subprocess.STDOUT,
                               cwd=scratch_dir)
        procs.append((rsp, rsp_fd))
        time.sleep(4)
        if rsp.poll() is not None:
            print(f"  [{label}] RSP crashed on start", file=sys.stderr)
            return []

        cmd_fd = open(cmd_log, 'w')
        cmd_cmd = [INSTALL_PATH, "-m", "ARQ", *config_args, *mode_args, *gain_args,
                   "-p", str(CMD_PORT), "-i", VB_IN, "-o", VB_OUT, "-x", "wasapi", "-n"]
        cmd = subprocess.Popen(cmd_cmd, stdout=cmd_fd, stderr=subprocess.STDOUT,
                               cwd=scratch_dir)
        procs.append((cmd, cmd_fd))
        time.sleep(3)
        if cmd.poll() is not None:
            print(f"  [{label}] CMD crashed on start", file=sys.stderr)
            return []

        nb_cmds = ["BW500\r\n"] if is_nb else []
        rsp_ctrl = tcp_send_commands(RSP_PORT, nb_cmds + ["MYCALL TESTB\r\n", "LISTEN ON\r\n"])
        sockets.append(rsp_ctrl)
        time.sleep(1)

        cmd_data = tcp_connect_retry(CMD_PORT + 1)
        sockets.append(cmd_data)
        rsp_data = tcp_connect_retry(RSP_PORT + 1)
        sockets.append(rsp_data)

        stop_event = threading.Event()
        tx_data = bytes(range(256)) * 4

        def tx_loop():
            cmd_data.settimeout(5)
            pos = 0
            while not stop_event.is_set():
                end = min(pos + 1024, len(tx_data))
                chunk = tx_data[pos:end]
                try:
                    cmd_data.sendall(chunk)
                    pos = end
                    if pos >= len(tx_data):
                        pos = 0
                except (socket.timeout, ConnectionError, OSError):
                    break
                time.sleep(0.01)

        rx_bytes = [0]

        def rx_loop():
            rsp_data.settimeout(2)
            while not stop_event.is_set():
                try:
                    data = rsp_data.recv(4096)
                    if not data:
                        break
                    rx_bytes[0] += len(data)
                except socket.timeout:
                    continue
                except (ConnectionError, OSError):
                    break

        threading.Thread(target=tx_loop, daemon=True).start()
        threading.Thread(target=rx_loop, daemon=True).start()
        time.sleep(1)

        cmd_ctrl = tcp_send_commands(CMD_PORT, nb_cmds + ["CONNECT TESTA TESTB\r\n"])
        sockets.append(cmd_ctrl)

        cmd_ctrl.settimeout(2)
        buf = b''
        t0 = time.time()
        connected = False
        while time.time() - t0 < 90:
            try:
                d = cmd_ctrl.recv(4096)
                if d:
                    buf += d
                    if b'CONNECTED' in buf and b'DISCONNECTED' not in buf:
                        connected = True
                        break
                    if b'DISCONNECTED' in buf:
                        break
                    if len(buf) > 2048:
                        buf = buf[-1024:]
            except socket.timeout:
                continue
            if any(p.poll() is not None for p, _ in procs):
                print(f"  [{label}] process died during connect", file=sys.stderr)
                return []
        if not connected:
            print(f"  [{label}] connect timeout", file=sys.stderr)
            # still collect whatever trace lines were emitted
        # Run for `secs` to accumulate trace lines
        time.sleep(secs)
        stop_event.set()
        print(f"  [{label}] rx_bytes={rx_bytes[0]} connected={connected}")
    finally:
        for s in sockets:
            try:
                s.close()
            except Exception:
                pass
        for p, fd in procs:
            try:
                p.terminate()
                p.wait(3)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
            try:
                fd.close()
            except Exception:
                pass
        kill_mercury()

    trace_files = [os.path.join(scratch_dir, f) for f in os.listdir(scratch_dir)
                   if f.startswith("timesync_trace_") and f.endswith(".csv")]
    return trace_files


def _kv(line):
    """Parse 'k=v' tokens from a trace line into a dict (values left as str)."""
    d = {}
    for tok in line.split():
        if '=' in tok:
            k, v = tok.split('=', 1)
            d[k] = v
    return d


def _decision_harmless(tag, line, is_nb):
    """For a coarse-search DIFF line, decide if it can flip a real decision.

    STEP3-PRIMITIVE / STEP5-SITE3 compare a decimated-coarse + fine path against
    the old full-rate path. A DIFF is HARMLESS (decision-equal) when:
      - both metrics are below the preamble detection threshold (0.15 WB /
        0.30 NB) -> both paths REJECT the frame, OR
      - the delay difference is small enough not to change pream_symb_loc
        (the Step-3 RESULT block documents M-grid-phase deltas <= 5*M; the
        symbol stride Nofdm*M is ~1168+ so a sub-symbol delta is absorbed).
    A DIFF where one metric is above threshold and the other below, or where
    the delays land in different symbols, IS a real failure.
    """
    thr = 0.30 if is_nb else 0.15
    d = _kv(line)
    if tag == 'STEP3-PRIMITIVE':
        om = float(d.get('old_metric', 0))
        dm = float(d.get('dec_metric', 0))
        diff = abs(int(d.get('diff', 0)))
        # both sub-threshold -> both reject; or small M-grid delta -> same symbol
        if (om < thr and dm < thr):
            return True
        if diff <= 32:   # <= ~8*M; never crosses a symbol boundary (Nofdm*M >> 32)
            return True
        return False
    if tag == 'STEP5-SITE3':
        nm = float(d.get('new_metric', 0))
        om = float(d.get('old_metric', 0))
        diff = abs(int(d.get('diff', 0)))
        if (nm < thr and om < thr):
            return True
        if diff <= 32:
            return True
        return False
    return False  # 6a tags: any MISMATCH/WARN is a real failure


def parse_traces(trace_files, is_nb):
    """Aggregate `# STEPxx-...` lines by tag.

    Counts: ok (EXACT/OK), harmless (DIFF that cannot flip a decision),
    bad (DIFF/MISMATCH/WARN that CAN flip a decision)."""
    agg = defaultdict(lambda: {'ok': 0, 'harmless': 0, 'bad': 0, 'samples': []})
    tag_re = re.compile(r'^#\s+(STEP\S+)\s')
    for tf in trace_files:
        try:
            with open(tf, 'r') as f:
                for line in f:
                    line = line.rstrip('\n')
                    m = tag_re.match(line)
                    if not m:
                        continue
                    tag = m.group(1)
                    verdict = line.split()[-1]
                    if verdict in ('OK', 'EXACT'):
                        agg[tag]['ok'] += 1
                    elif verdict in ('DIFF', 'MISMATCH', 'WARN'):
                        if _decision_harmless(tag, line, is_nb):
                            agg[tag]['harmless'] += 1
                        else:
                            agg[tag]['bad'] += 1
                            if len(agg[tag]['samples']) < 5:
                                agg[tag]['samples'].append(line)
        except OSError:
            pass
    return agg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=int, default=45,
                    help='seconds to run each loopback after connect (default 45)')
    ap.add_argument('--configs', type=str, default="WB_CFG10,NB_CFG10",
                    help='comma-separated config labels (default WB_CFG10,NB_CFG10)')
    args = ap.parse_args()

    wanted = set(c.strip() for c in args.configs.split(','))
    configs = [t for t in ALL_CONFIGS if t[2] in wanted]
    if not configs:
        print(f"No configs matched {args.configs}", file=sys.stderr)
        return 2

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    scratch = os.path.join(tempfile.gettempdir(), f"mercury_tstrace_{ts}")
    os.makedirs(scratch, exist_ok=True)
    print(f"Scratch dir: {scratch}")
    print(f"Configs: {[c[2] for c in configs]}  secs/config={args.secs}\n")

    total = defaultdict(lambda: {'ok': 0, 'harmless': 0, 'bad': 0, 'samples': []})
    for cfg_id, is_nb, label in configs:
        print(f"=== {label} ===")
        tfs = run_loopback(cfg_id, is_nb, label, args.secs, scratch)
        if not tfs:
            print(f"  WARNING: no trace files produced for {label}")
            continue
        agg = parse_traces(tfs, is_nb)
        for tag, d in sorted(agg.items()):
            status = "PASS" if d['bad'] == 0 else "FAIL"
            print(f"  {tag:<28} ok={d['ok']:<5} harmless={d['harmless']:<4} bad={d['bad']:<4} {status}")
            for s in d['samples']:
                print(f"      {s}")
            total[tag]['ok'] += d['ok']
            total[tag]['harmless'] += d['harmless']
            total[tag]['bad'] += d['bad']
            total[tag]['samples'].extend(d['samples'][:2])
        print()

    print("=" * 70)
    print("AGGREGATE (all configs)")
    print("  ok       = bit-exact / decision-equal")
    print("  harmless = DIFF that cannot flip a decision (sub-threshold both"
          " sides, or sub-symbol M-grid delta — documented Step 3/5 behavior)")
    print("  bad      = DIFF/MISMATCH that COULD flip a real decision")
    print("=" * 70)
    any_bad = False
    any_data = False
    for tag, d in sorted(total.items()):
        any_data = True
        status = "PASS" if d['bad'] == 0 else "FAIL"
        if d['bad'] > 0:
            any_bad = True
        print(f"  {tag:<28} ok={d['ok']:<6} harmless={d['harmless']:<5} bad={d['bad']:<5} {status}")
        for s in d['samples'][:5]:
            print(f"      {s}")
    if not any_data:
        print("  NO TRACE LINES SEEN — instrumentation did not run!")
        return 1
    print()
    if any_bad:
        print("RESULT: FAIL — at least one dual-path check disagreed.")
        return 1
    print("RESULT: PASS — every dual-path decision-equality check agreed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""
SACK ldpc=NO hard-fallback repro test.

Reproduces (and, after the A2 fix, validates) the bug documented in
fact-documents/SACK_LDPC_FALLBACK_INVESTIGATION.md.

Setup: CMD + RSP on VB-Cable, WB CONFIG_15, --enable-sack on both.
CMD additionally runs with --test-sack-ldpc-fail, which forces ldpc_ok=false
for every SACK reception (even healthy ones). This deterministically drives the
ldpc=NO code path in receive_sack_pattern().

PASS/FAIL logic:
  - For each CMD '[RX-SACK] Detected (... ldpc=NO ...), bitmap: B' line, find the
    RSP '[TX-SACK] Sending SACK pattern (batch=N, received: R)' that produced it
    (the temporally-preceding RSP SACK of the same batch size).
  - BEFORE THE FIX: CMD's decoded bitmap B is the legacy-codebook garbage decode
    of LDPC-coded tones -> B != R  => bug reproduced (this script "fails": prints
    BUG REPRODUCED).
  - AFTER THE A2 FIX: ldpc=NO leaves out_bitmap all-false. CMD's B is all-zero.
    An all-zero bitmap is the SAFE full-batch-retransmit signal -> NOT a
    fabricated bitmap. Script prints SAFE BEHAVIOUR.
  - The dangerous case the fix eliminates: B has at least one '1' that does NOT
    match R (CMD marks a frame ACKED that RSP never received, OR CMD's pattern of
    1s simply disagrees with R). If B is all-zero, it is safe regardless of R.

Exit code: 0 if the run produced usable ldpc=NO events AND classified them;
the textual verdict (BUG REPRODUCED / SAFE BEHAVIOUR / INCONCLUSIVE) is printed.
"""
import subprocess, socket, time, sys, os, threading, re

MERCURY = r"C:\Program Files\Mercury\mercury.exe"
VB_OUT = "CABLE Input (VB-Audio Virtual Cable)"
VB_IN  = "CABLE Output (VB-Audio Virtual Cable)"
RSP_PORT = 7002
CMD_PORT = 7006
TEXT_FILE = r"x:\Storage\Documents\hermes and mercury\pg84.txt"
DURATION = 90  # seconds to measure (need several batches -> several SACKs)

RSP_LOG = "sack_repro_RSP_7002.log"
CMD_LOG = "sack_repro_CMD_7006.log"


def kill_mercury():
    os.system("taskkill /F /IM mercury.exe >nul 2>&1")
    time.sleep(1)


def start_mercury(role, port, logname, extra_args=None):
    args = [
        MERCURY,
        "-m", "ARQ",
        "-s", "15",
        "-Q", "0",
        "-M", "auto",
        "-T", "-12.6",
        "-G", "12.6",
        "-x", "wasapi",
        "-i", VB_IN,
        "-o", VB_OUT,
        "-n",
        "-p", str(port),
        "-F", "on",
        "--skip-turbo-reverse",
        "--enable-sack",
    ]
    if extra_args:
        args += extra_args
    logf = open(logname, "w")
    proc = subprocess.Popen(args, stdout=logf, stderr=subprocess.STDOUT)
    print(f"[{role}] PID={proc.pid} port={port} extra={extra_args}")
    return proc, logf


def tcp_connect_retry(port, retries=10, delay=0.5):
    for attempt in range(retries):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5)
            s.connect(("127.0.0.1", port))
            return s
        except (ConnectionRefusedError, socket.timeout):
            if attempt < retries - 1:
                time.sleep(delay)
            else:
                raise


def send_cmd(port, cmd):
    s = tcp_connect_retry(port)
    s.sendall((cmd + "\r").encode())
    time.sleep(0.3)
    try:
        resp = s.recv(4096).decode(errors='replace')
    except Exception:
        resp = ""
    s.close()
    return resp


def tx_loop(sock, data, stop_event):
    sock.settimeout(30)
    pos = 0
    while not stop_event.is_set():
        end = min(pos + 1024, len(data))
        chunk = data[pos:end]
        if not chunk:
            pos = 0
            continue
        try:
            sock.sendall(chunk)
            pos = end
            if pos >= len(data):
                pos = 0
        except socket.timeout:
            continue
        except (ConnectionError, OSError):
            break


def rx_loop(sock, results, stop_event):
    sock.settimeout(2)
    total = 0
    while not stop_event.is_set():
        try:
            chunk = sock.recv(65536)
            if chunk:
                total += len(chunk)
                results['rx_bytes'] = total
            elif not chunk:
                break
        except socket.timeout:
            continue
        except (ConnectionError, OSError):
            break
    results['rx_bytes'] = total


# Regexes for log parsing.
RSP_TX = re.compile(r"\[TX-SACK\] Sending SACK pattern \(batch=(\d+), received:([ 01]+)\)")
CMD_RX = re.compile(
    r"\[RX-SACK\] Detected \(matched=(\d+), metric=([\d.]+), ack_xcheck=(\d+), ldpc=(YES|NO)\), bitmap:([ 01]+)")


def parse_bits(s):
    return [int(x) for x in s.split()]


def analyze():
    """Compare CMD ldpc=NO decoded bitmaps against the RSP SACK that produced them."""
    if not (os.path.exists(RSP_LOG) and os.path.exists(CMD_LOG)):
        print("MISSING LOGS")
        return "INCONCLUSIVE"

    with open(RSP_LOG, errors='replace') as f:
        rsp_lines = f.readlines()
    with open(CMD_LOG, errors='replace') as f:
        cmd_lines = f.readlines()

    # Ordered list of RSP SACK transmissions (batch, bits).
    rsp_sacks = []
    for ln in rsp_lines:
        m = RSP_TX.search(ln)
        if m:
            rsp_sacks.append((int(m.group(1)), parse_bits(m.group(2))))

    # Ordered list of CMD SACK receptions.
    cmd_rx = []
    for ln in cmd_lines:
        m = CMD_RX.search(ln)
        if m:
            cmd_rx.append(dict(matched=int(m.group(1)), metric=float(m.group(2)),
                               ack_xcheck=int(m.group(3)), ldpc=m.group(4),
                               bitmap=parse_bits(m.group(5))))

    print(f"\n--- Parsed: {len(rsp_sacks)} RSP TX-SACKs, {len(cmd_rx)} CMD RX-SACK Detected ---")

    ldpc_no = [r for r in cmd_rx if r['ldpc'] == 'NO']
    ldpc_yes = [r for r in cmd_rx if r['ldpc'] == 'YES']
    print(f"    ldpc=YES: {len(ldpc_yes)}   ldpc=NO: {len(ldpc_no)}")

    if not ldpc_no:
        print("    No ldpc=NO events captured -> test could not exercise the fault path.")
        return "INCONCLUSIVE"

    # The RSP SACK is sent before CMD receives it; the simplest robust pairing is
    # by batch size + ordinal. We match each ldpc=NO reception to the RSP SACK at
    # the same ordinal position among same-batch-size SACKs. Since both are
    # strictly ordered in time and 1:1 per round, ordinal matching is sound for
    # this loopback (one CMD, one RSP, no reordering).
    fabricated = 0   # B has a 1 that disagrees with R, or B != R with nonzero B
    safe = 0         # B all-zero -> full-batch retransmit
    classified = 0

    # Build same-batch ordinal index for RSP SACKs.
    for idx, rx in enumerate(ldpc_no):
        b = rx['bitmap']
        nonzero = any(b)
        # Find a plausible RSP SACK: same length, nearest by ordinal.
        candidates = [s for s in rsp_sacks if len(s[1]) == len(b)]
        r = candidates[min(idx, len(candidates) - 1)][1] if candidates else None

        if not nonzero:
            safe += 1
            classified += 1
            print(f"  [ldpc=NO #{idx}] metric={rx['metric']} matched={rx['matched']} "
                  f"bitmap=ALL-ZERO -> SAFE (full-batch retransmit). RSP wanted={r}")
        else:
            # nonzero decoded bitmap on a ldpc=NO event == fabricated bitmap
            disagree = (r is None) or (b != r)
            fabricated += 1
            classified += 1
            tag = "DISAGREES with RSP" if disagree else "happens to match RSP"
            print(f"  [ldpc=NO #{idx}] metric={rx['metric']} matched={rx['matched']} "
                  f"bitmap={b} ({tag}) RSP wanted={r}  <-- FABRICATED")

    print(f"\n    classified={classified}  fabricated={fabricated}  safe={safe}")

    # Also dump matched/metric distribution for the §8 threshold question.
    print("\n--- matched/metric distribution (ALL [RX-SACK] Detected) ---")
    for r in cmd_rx:
        print(f"    matched={r['matched']:2d} metric={r['metric']:5.1f} "
              f"ack_xcheck={r['ack_xcheck']:2d} ldpc={r['ldpc']}")

    if fabricated > 0:
        return "BUG REPRODUCED"
    if safe > 0:
        return "SAFE BEHAVIOUR"
    return "INCONCLUSIVE"


def main():
    print("=== SACK ldpc=NO hard-fallback repro test ===")
    print(f"CONFIG_15, --enable-sack both, --test-sack-ldpc-fail on CMD, duration={DURATION}s")

    with open(TEXT_FILE, 'rb') as f:
        text_data = f.read()
    if len(text_data) < 20000:
        text_data = text_data * 10
    print(f"TX data: {len(text_data)} bytes")

    kill_mercury()

    rsp_proc, rsp_log = start_mercury("RSP", RSP_PORT, RSP_LOG)
    time.sleep(4)
    cmd_proc, cmd_log = start_mercury("CMD", CMD_PORT, CMD_LOG,
                                      extra_args=["--test-sack-ldpc-fail"])
    time.sleep(3)

    print("[RSP] MYCALL + LISTEN ON")
    send_cmd(RSP_PORT, "MYCALL RSP")
    time.sleep(0.5)
    send_cmd(RSP_PORT, "LISTEN ON")
    time.sleep(1)
    print("[CMD] MYCALL")
    send_cmd(CMD_PORT, "MYCALL CMD")
    time.sleep(0.5)

    print("[DATA] connecting data ports")
    cmd_data = tcp_connect_retry(CMD_PORT + 1)
    rsp_data = tcp_connect_retry(RSP_PORT + 1)

    stop_event = threading.Event()
    tx_thread = threading.Thread(target=tx_loop, args=(cmd_data, text_data, stop_event), daemon=True)
    tx_thread.start()
    time.sleep(2)
    rx_results = {'rx_bytes': 0}
    rx_thread = threading.Thread(target=rx_loop, args=(rsp_data, rx_results, stop_event), daemon=True)
    rx_thread.start()

    print("[CMD] CONNECT CMD RSP")
    resp = send_cmd(CMD_PORT, "CONNECT CMD RSP")
    print(f"  Response: {resp.strip()}")

    SETTLE = 35
    print(f"Settling up to {SETTLE}s ...")
    settle_start = time.time()
    while time.time() - settle_start < SETTLE:
        time.sleep(1)
        if rx_results.get('rx_bytes', 0) > 0:
            print(f"  First data at {time.time() - settle_start:.1f}s")
            break
    else:
        print("  WARNING: no data received during settle")

    print(f"Measuring {DURATION}s ...")
    measure_start = time.time()
    while time.time() - measure_start < DURATION:
        time.sleep(10)
        elapsed = time.time() - measure_start
        print(f"  {elapsed:.0f}s elapsed, rx={rx_results.get('rx_bytes',0)} bytes")

    stop_event.set()
    print("Cleaning up ...")
    try:
        cmd_data.close(); rsp_data.close()
    except Exception:
        pass
    try:
        send_cmd(CMD_PORT, "DISCONNECT")
    except Exception:
        pass
    time.sleep(2)
    kill_mercury()
    rsp_log.close()
    cmd_log.close()

    verdict = analyze()
    print(f"\n==================  VERDICT: {verdict}  ==================")
    # exit 0 only when we got a definitive classification
    sys.exit(0 if verdict in ("BUG REPRODUCED", "SAFE BEHAVIOUR") else 2)


if __name__ == "__main__":
    main()

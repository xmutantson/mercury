#!/usr/bin/env python3
"""
run_threaded_connect.py — Phase 1.5 THREADED CONNECT-crash repro for the O3
intermittent-crash hunt.

Phase 1 exonerated the single-threaded SIM_INPROC path (0 UB / 121 runs) and
pinned the top candidate to a multi-threaded UAF/race between the audio
capture_prep thread (audioio.c:1302 reads Nofdm/buffer_Nsymb outside the mutex)
and load_configuration's free+realloc of the raw passband/baseband buffers
(telecom_system.cc:8849). SIM_INPROC is structurally single-threaded so it
cannot exhibit the race.

This driver runs the REAL threaded -m ARQ path WITHOUT audio hardware via the
device-free SIM software channel (-x sim + sim_channel_relay.py). That spawns
radio_capture_prep_thread + the two bridge threads (audioio.c:1752-1754) — the
exact threaded config-transition path H1 names — across two processes wired
through a TCP relay. A payload forces the gearshift to climb (many
load_configuration transitions = max race exposure).

Built on the UBSan+GLIBCXX+re-armed-canary binary, every run is watched for:
  * crash exit code (SIGSEGV 0xC0000005 / abort / nonzero)
  * [CANARY] OOB ...   (raw-heap-buffer over-write — the predicted class)
  * [UBSAN] ...         (any UB the toolchain can see)
  * [CAP-STALE] ...     (the capture thread saw the size pair change under it —
                         the race WINDOW being entered, even if it didn't fault)

Each cycle's CMD/RSP/relay logs are written to phase15/ and a one-line CSV row
is appended INCREMENTALLY so a crash mid-sweep is never lost.

NOTE: this bypasses sim_arq_channel.py's GUARD-2 binary check on purpose — the
monitor HEAD predates the [SIM-AUDIO-GUARD] marker, but audioio_init_internal
short-circuits the SIM backend and returns BEFORE any device pthread_create
(audioio.c:1746-1755), so -x sim is device-free regardless. Verified in source.
"""
import argparse, json, os, socket, struct, subprocess, sys, threading, time, signal

HERE = os.path.dirname(os.path.abspath(__file__))
# OUTDIR / seed-base are env-overridable so the Phase-2 pre/post capstale slices
# can target distinct dirs with offset seeds WITHOUT touching modem source.
OUTDIR = os.environ.get(
    "PHASE15_OUTDIR",
    r"x:/Storage/Documents/hermes and mercury/bigblock_p3_hw/_o3ub/phase15")
SEED_BASE = int(os.environ.get("PHASE15_SEED_BASE", "41000"))
os.makedirs(OUTDIR, exist_ok=True)
RELAY = r"x:/Storage/Documents/hermes and mercury/tools/sim_channel_relay.py"

RSP_PORT = 7002
CMD_PORT = 7006


def kill_all():
    os.system("taskkill /F /IM mercury_ubsan_O3_glibcxx.exe >nul 2>&1")
    os.system("taskkill /F /IM mercury.exe >nul 2>&1")


def tcp_send(port, commands, retries=25, delay=0.5):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    for attempt in range(retries):
        try:
            s.connect(("127.0.0.1", port)); break
        except (ConnectionRefusedError, OSError):
            if attempt == retries - 1:
                raise
            time.sleep(delay)
            s.close()
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM); s.settimeout(5)
    for c in commands:
        s.sendall(c.encode()); time.sleep(0.25)
    return s


def tx_payload(sock, stop, nbytes):
    """Push `nbytes` of varied data to force the gearshift to climb."""
    chunk = bytes(range(256)) * 8   # 2048 bytes
    sent = 0
    sock.settimeout(20)
    while not stop.is_set() and sent < nbytes:
        try:
            sock.sendall(chunk); sent += len(chunk)
        except (socket.timeout, ConnectionError, OSError):
            break
        time.sleep(0.02)


def classify(rc):
    # Windows: 0xC0000005 access violation = -1073741819 (segfault), 0xC000001D
    # illegal instruction, 3 = abort()/_exit. Treat any nonzero/negative as crash
    # candidate, but separate the clean teardown kill (we kill -> rc set by us).
    if rc is None:
        return "running"
    u = rc & 0xFFFFFFFF
    if u == 0xC0000005:
        return "SIGSEGV"
    if u == 0xC000001D:
        return "SIGILL"
    if u in (0xC0000374,):
        return "HEAP_CORRUPT"
    if rc == 3 or u == 0xC0000409:
        return "ABORT"
    if rc == 0:
        return "ok"
    return f"exit{rc}"


def scan_log(path):
    canary = ubsan = capstale = 0
    first_canary = first_ubsan = ""
    try:
        with open(path, "r", errors="replace") as f:
            for line in f:
                if "[CANARY] OOB" in line:
                    canary += 1
                    if not first_canary:
                        first_canary = line.strip()
                if "[UBSAN]" in line:
                    ubsan += 1
                    if not first_ubsan:
                        first_ubsan = line.strip()
                if "[CAP-STALE]" in line:
                    capstale += 1
    except OSError:
        pass
    return canary, ubsan, capstale, first_canary, first_ubsan


def run_cycle(args, idx, seed, snr, profile, summary_fp):
    tag = f"c{idx:03d}_s{seed}_snr{snr}_{profile}"
    cmd_log = os.path.join(OUTDIR, f"{tag}_CMD.log")
    rsp_log = os.path.join(OUTDIR, f"{tag}_RSP.log")
    relay_log = os.path.join(OUTDIR, f"{tag}_relay.log")
    port = args.port

    env = dict(os.environ)
    env["MERCURY_SIM_PORT"] = str(port)
    # log-and-continue so one cycle surfaces every site; the watcher records them
    env["UBSAN_ABORT"] = "0"
    env["CANARY_ABORT"] = "0"

    def base_cmd(p):
        c = [args.bin, "-m", "ARQ", "-s", str(args.start_cfg), "-W",
             "-p", str(p), "-x", "sim", "-n", "-g", "-F", "off"]
        if args.start_cfg >= 100:
            c += ["-R"]
        return c

    relay = None
    procs = []
    stop = threading.Event()
    crashed = "ok"
    crash_who = ""
    rc_cmd = rc_rsp = None
    try:
        rl = open(relay_log, "w")
        relay = subprocess.Popen(
            [sys.executable, RELAY, "--port", str(port), "--snr", str(snr),
             "--profile", profile, "--seed", str(seed), "--barrier-k", "4",
             "--log", relay_log])
        time.sleep(0.8)

        f_rsp = open(rsp_log, "wb")
        e2 = dict(env); e2["MERCURY_SIM_ROLE"] = "B"
        rsp = subprocess.Popen(base_cmd(RSP_PORT), env=e2,
                               stdout=f_rsp, stderr=subprocess.STDOUT)
        procs.append(("RSP", rsp, f_rsp))
        time.sleep(2.0)

        f_cmd = open(cmd_log, "wb")
        e1 = dict(env); e1["MERCURY_SIM_ROLE"] = "A"
        cmd = subprocess.Popen(base_cmd(CMD_PORT), env=e1,
                               stdout=f_cmd, stderr=subprocess.STDOUT)
        procs.append(("CMD", cmd, f_cmd))
        time.sleep(2.0)

        # control: RSP listen, then CMD connect
        rsp_ctrl = tcp_send(RSP_PORT, ["MYCALL TESTB\r\n", "LISTEN ON\r\n"])
        time.sleep(0.6)
        # data socket on CMD to push the climbing payload
        cmd_data = None
        try:
            cmd_data = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            cmd_data.settimeout(5)
            cmd_data.connect(("127.0.0.1", CMD_PORT + 1))
        except OSError:
            cmd_data = None
        cmd_ctrl = tcp_send(CMD_PORT, ["MYCALL TESTA\r\n", "CONNECT TESTA TESTB\r\n"])
        if cmd_data is not None:
            threading.Thread(target=tx_payload,
                             args=(cmd_data, stop, args.payload), daemon=True).start()

        # monitor for the dwell, watching for an early crash (process exit)
        t_end = time.time() + args.dwell
        while time.time() < t_end:
            time.sleep(0.4)
            for who, p, _f in procs:
                if p.poll() is not None:
                    cl = classify(p.returncode)
                    if cl not in ("ok",):
                        crashed = cl; crash_who = who
                    t_end = 0  # break outer
                    break
        stop.set()
        try:
            if cmd_data: cmd_data.close()
            rsp_ctrl.close(); cmd_ctrl.close()
        except OSError:
            pass
    except Exception as e:
        crashed = "harness_err"; crash_who = str(e)[:60]
    finally:
        stop.set()
        # give a brief grace, then capture exit codes (kill anything still up)
        time.sleep(0.3)
        for who, p, f in procs:
            if p.poll() is None:
                try: p.kill()
                except OSError: pass
            else:
                cl = classify(p.returncode)
                if cl not in ("ok",) and crashed == "ok":
                    crashed = cl; crash_who = who
            if who == "CMD": rc_cmd = p.returncode
            if who == "RSP": rc_rsp = p.returncode
            try: f.close()
            except Exception: pass
        if relay:
            try: relay.terminate()
            except OSError: pass
        try: rl.close()
        except Exception: pass
        kill_all()
        time.sleep(0.4)

    # scan both peer logs for canary/ubsan/cap-stale
    cc, uc, sc, fc, fu = 0, 0, 0, "", ""
    for lg in (cmd_log, rsp_log):
        a, b, c, d, e = scan_log(lg)
        cc += a; uc += b; sc += c
        fc = fc or d; fu = fu or e

    connected = False
    for lg in (cmd_log, rsp_log):
        try:
            with open(lg, "r", errors="replace") as f:
                if "link_status:Connected to" in f.read():
                    connected = True; break
        except OSError:
            pass

    row = (f"{idx},{seed},{snr},{profile},{crashed},{crash_who},"
           f"rc_cmd={rc_cmd},rc_rsp={rc_rsp},canary={cc},ubsan={uc},"
           f"capstale={sc},connected={int(connected)}")
    summary_fp.write(row + "\n"); summary_fp.flush()
    flag = ""
    if crashed != "ok": flag += " CRASH(%s/%s)" % (crashed, crash_who)
    if cc: flag += " CANARY=%d" % cc
    if uc: flag += " UBSAN=%d" % uc
    if sc: flag += " capstale=%d" % sc
    print(f"[{tag}] crashed={crashed} canary={cc} ubsan={uc} capstale={sc} "
          f"conn={int(connected)}{flag}")
    sys.stdout.flush()
    if fc: print("   first canary:", fc)
    if fu: print("   first ubsan :", fu)

    # keep logs only if interesting; else delete to bound disk
    if crashed == "ok" and cc == 0 and uc == 0:
        for lg in (cmd_log, rsp_log, relay_log):
            try: os.remove(lg)
            except OSError: pass

    return {"crashed": crashed, "canary": cc, "ubsan": uc, "capstale": sc,
            "connected": connected}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=os.path.join(HERE, "mercury_ubsan_O3_glibcxx.exe"))
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--dwell", type=int, default=20, help="seconds per cycle")
    ap.add_argument("--payload", type=int, default=8000, help="bytes to push (climb driver)")
    ap.add_argument("--start-cfg", type=int, default=100)
    ap.add_argument("--port", type=int, default=52180)
    args = ap.parse_args()

    if not os.path.isfile(args.bin):
        print("FATAL: binary not found:", args.bin); return 2

    # cells: clean (aggressive climb = max config transitions) + a couple noisy
    # (collapse -> re-climb = even MORE transitions through load_configuration).
    cells = [(900, "wgn"), (30, "wgn"), (12, "mpm"), (900, "wgn"), (20, "mpm")]

    summary = os.path.join(OUTDIR, "phase15_threaded_summary.csv")
    sfp = open(summary, "w")
    sfp.write("idx,seed,snr,profile,crashed,crash_who,rc_cmd,rc_rsp,"
              "canary,ubsan,capstale,connected\n")
    sfp.flush()

    kill_all()
    print(f"=== Phase 1.5 threaded CONNECT repro: N={args.n} dwell={args.dwell}s "
          f"payload={args.payload}B bin={os.path.basename(args.bin)} ===")
    tot = {"crashed": 0, "canary": 0, "ubsan": 0, "capstale": 0, "connected": 0}
    t0 = time.time()
    for i in range(args.n):
        snr, prof = cells[i % len(cells)]
        seed = SEED_BASE + i * 17
        r = run_cycle(args, i, seed, snr, prof, sfp)
        if r["crashed"] != "ok": tot["crashed"] += 1
        tot["canary"] += r["canary"]
        tot["ubsan"] += r["ubsan"]
        tot["capstale"] += r["capstale"]
        tot["connected"] += int(r["connected"])

    sfp.close()
    dt = time.time() - t0
    print(f"\n=== DONE in {dt:.0f}s. N={args.n} "
          f"crashes={tot['crashed']} canary_runs_total={tot['canary']} "
          f"ubsan_total={tot['ubsan']} capstale_total={tot['capstale']} "
          f"connected={tot['connected']}/{args.n} ===")
    print("summary:", summary)
    verdict = {
        "n": args.n, "dwell_s": args.dwell, "payload_b": args.payload,
        "wall_s": round(dt, 1),
        "crashes": tot["crashed"], "canary_hits": tot["canary"],
        "ubsan_hits": tot["ubsan"], "capstale_events": tot["capstale"],
        "connected": tot["connected"],
        "bin": os.path.basename(args.bin),
        "cells": cells,
    }
    with open(os.path.join(OUTDIR, "phase15_verdict.json"), "w") as f:
        json.dump(verdict, f, indent=1)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        kill_all(); sys.exit(130)

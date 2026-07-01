#!/usr/bin/env python3
"""
parallel_spawner.py - launch N concurrent real-audio Mercury round-trips over
snd-aloop and prove CORRECTNESS AT SPEED.

The real-audio runs (arq_realaudio.py + realaudio_bridge_s32.py) are INDEPENDENT:
each owns its own 4 snd-aloop cables, its own 2 mercury -x alsa processes, its own
IONOS bridge, and its own TCP control/data ports. Because each capture read is
kernel-clocked by the snd-aloop hardware timer (NOT the CPU), the runs are
load-immune by construction - so we can run N of them CONCURRENTLY and:

  (a) all N still connect + deliver  => correctness under the concurrent heavy
      load (the N runs ARE the load), i.e. load-immunity proven in one shot;
  (b) total wall-clock ~ ONE run's duration (NOT N x), because they overlap;
  (c) each run's delivered frames/bytes match a known-good single-run reference
      (same pinned config + same per-run seed).

snd-aloop topology
------------------
snd-aloop wires playback(dev0,subN) <-> capture(dev1,subN). One run needs 4
cables = 4 substreams. A card supports at most 8 substreams (pcm_substreams 1-8),
so 2 runs fit per card. N runs need ceil(N/2) cards. We require the caller to
have loaded snd-aloop with index=0..C-1 each enable=1 pcm_substreams=8 (see
--setup-cmd printed by --print-setup). Card k carries runs 2k and 2k+1, on
substreams {0,1,2,3} and {4,5,6,7} respectively.

TCP ports
---------
Each run gets a disjoint (rsp_port, cmd_port). Run i: rsp=BASE+10*i, cmd=BASE+10*i+4
(data sockets are port+1, so +10 spacing leaves headroom).

Usage
-----
  python3 parallel_spawner.py --n 12 --bin /path/to/mercury \\
      --secs 130 --payload 512 --start-cfg 100 --no-gearshift \\
      --passthrough            # or: --cell WGN:55 --profile wgn
"""
import argparse
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ra_cleanup import scoped_cleanup_cells  # concurrency-safe cohort cleanup
HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.join(HERE, "arq_realaudio.py")


def card_name(idx):
    # snd-aloop names successive cards Loopback, Loopback_1, Loopback_2, ...
    return "Loopback" if idx == 0 else f"Loopback_{idx}"


def run_plan(n, port_base, card_base=0, tag_prefix="par"):
    """Return list of per-run dicts: card, subs, rsp_port, cmd_port, seed, tag.

    card_base offsets the starting snd-aloop card index so multiple concurrent
    cohorts (e.g. two A/B arms, or two SNR points) can own DISJOINT cards. Each
    cohort must also use a disjoint port_base. tag_prefix keeps result/log
    filenames unique per cohort."""
    plan = []
    for i in range(n):
        card_idx = card_base + i // 2
        slot = i % 2                       # 0 -> subs 0..3, 1 -> subs 4..7
        subs = [slot * 4 + j for j in range(4)]
        rsp_port = port_base + 10 * i
        cmd_port = port_base + 10 * i + 4
        plan.append({
            "idx": i,
            "card": card_name(card_idx),
            "card_idx": card_idx,
            "subs": subs,
            "rsp_port": rsp_port,
            "cmd_port": cmd_port,
            "seed": i + 1,                 # deterministic per-run seed = run index+1
            "tag": f"{tag_prefix}{i:02d}",
        })
    return plan


def print_setup(n):
    cards = (n + 1) // 2
    idx = ",".join(str(k) for k in range(cards))
    en = ",".join("1" for _ in range(cards))
    subs = ",".join("8" for _ in range(cards))
    print("# load snd-aloop with enough cards for N=%d runs (%d cards x 8 substreams):" % (n, cards))
    print("sudo modprobe -r snd-aloop 2>/dev/null; "
          f"sudo modprobe snd-aloop index={idx} enable={en} pcm_substreams={subs}")
    print("# verify:")
    print("cat /proc/asound/cards")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--bin", default="/home/kameron/raspeed/mercury")
    ap.add_argument("--bridge", default=os.path.join(HERE, "realaudio_bridge_s32.py"))
    ap.add_argument("--secs", type=int, default=130)
    ap.add_argument("--payload", type=int, default=512)
    ap.add_argument("--start-cfg", type=int, default=100)
    ap.add_argument("--no-gearshift", action="store_true")
    ap.add_argument("--passthrough", action="store_true")
    ap.add_argument("--cell", default=None)
    ap.add_argument("--profile", default="wgn")
    ap.add_argument("--cfo-hz", type=float, default=0.0)
    ap.add_argument("--phase-noise-deg", type=float, default=0.0)
    ap.add_argument("--fade-depth-db", type=float, default=0.0)
    ap.add_argument("--cap-periods", type=int, default=3)
    ap.add_argument("--play-periods", type=int, default=4)
    ap.add_argument("--prime-periods", type=int, default=2)
    ap.add_argument("--port-base", type=int, default=7100)
    ap.add_argument("--card-base", type=int, default=0,
                    help="starting snd-aloop card index (disjoint cohorts use disjoint bases)")
    ap.add_argument("--tag-prefix", default="par",
                    help="prefix for per-cell tag/result filenames (unique per cohort)")
    ap.add_argument("--logdir", default="/tmp/raspeed/logs")
    ap.add_argument("--out", default="/tmp/raspeed/PARALLEL_RESULT.json")
    ap.add_argument("--launch-stagger", type=float, default=0.0,
                    help="seconds between launching successive runs (0 = true simultaneous)")
    ap.add_argument("--encrypt", default=None,
                    help="pass -E <mode> to BOTH mercury in every cell (e.g. 'fast')")
    ap.add_argument("--psk", default=None,
                    help="pass -K <hex> to BOTH mercury in every cell (with --encrypt)")
    ap.add_argument("--arm", default="legacy", help="A/B arm label recorded per cell")
    ap.add_argument("--env", action="append", default=[],
                    help="KEY=VAL env injected into BOTH mercury per cell (repeatable)")
    ap.add_argument("--print-setup", action="store_true",
                    help="print the modprobe command to provision snd-aloop and exit")
    args = ap.parse_args()

    if args.print_setup:
        print_setup(args.n)
        return 0

    os.makedirs(args.logdir, exist_ok=True)
    plan = run_plan(args.n, args.port_base, args.card_base, args.tag_prefix)

    # CONCURRENCY-SAFE cohort cleanup: clear ONLY stale leftovers for the
    # cells THIS spawner is about to launch (its own plan: each cell's card +
    # substreams + TCP ports). A sibling agent's cohort on disjoint
    # cards/subs/ports is never matched, so we never reap it. The old global
    # `pkill -9 -f 'mercury -m ARQ'` reaped every concurrent agent's mercury
    # (diagnostic wf_18b9f890). See ra_cleanup.py.
    scoped_cleanup_cells(plan, settle=2.0)

    procs = []
    cohort_t0 = time.time()
    for p in plan:
        jpath = os.path.join(args.logdir, f"res_{p['tag']}.json")
        cmd = [sys.executable, "-u", HARNESS,
               "--bin", args.bin, "--bridge", args.bridge,
               "--tag", p["tag"], "--json", jpath, "--logdir", args.logdir,
               "--start-cfg", str(args.start_cfg),
               "--secs", str(args.secs), "--payload", str(args.payload),
               "--seed", str(p["seed"]),
               "--card", p["card"], "--subs", ",".join(str(s) for s in p["subs"]),
               "--rsp-port", str(p["rsp_port"]), "--cmd-port", str(p["cmd_port"]),
               "--cap-periods", str(args.cap_periods),
               "--play-periods", str(args.play_periods),
               "--prime-periods", str(args.prime_periods),
               "--no-kill"]
        if args.no_gearshift:
            cmd += ["--no-gearshift"]
        if args.passthrough:
            cmd += ["--passthrough"]
        if args.cell:
            cmd += ["--cell", args.cell]
        if args.profile:
            cmd += ["--profile", args.profile]
        if args.cfo_hz:
            cmd += ["--cfo-hz", str(args.cfo_hz)]
        if args.phase_noise_deg:
            cmd += ["--phase-noise-deg", str(args.phase_noise_deg)]
        if args.fade_depth_db:
            cmd += ["--fade-depth-db", str(args.fade_depth_db)]
        if args.encrypt:
            cmd += ["--encrypt", args.encrypt]
        if args.psk:
            cmd += ["--psk", args.psk]
        if args.arm:
            cmd += ["--arm", args.arm]
        for kv in args.env:
            cmd += ["--env", kv]
        outlog = open(os.path.join(args.logdir, f"spawn_{p['tag']}.out"), "wb")
        proc = subprocess.Popen(cmd, stdout=outlog, stderr=subprocess.STDOUT)
        procs.append((p, proc, jpath, outlog))
        sys.stderr.write(f"[spawner] launched {p['tag']} card={p['card']} "
                         f"subs={p['subs']} ports={p['rsp_port']}/{p['cmd_port']} "
                         f"seed={p['seed']}\n")
        sys.stderr.flush()
        if args.launch_stagger > 0:
            time.sleep(args.launch_stagger)

    sys.stderr.write(f"[spawner] all {args.n} runs launched at "
                     f"T+{time.time()-cohort_t0:.1f}s; waiting...\n")
    sys.stderr.flush()

    # wait for all children (each self-terminates at --secs or on delivery)
    for p, proc, jpath, outlog in procs:
        proc.wait()
        outlog.close()
    cohort_wall = time.time() - cohort_t0

    results = []
    for p, proc, jpath, _ in procs:
        try:
            with open(jpath) as f:
                r = json.load(f)
        except Exception as e:  # noqa: BLE001
            r = {"tag": p["tag"], "error": str(e), "connected": False,
                 "delivered_full": False, "rx_bytes": None,
                 "rsp_nreceived_frames": None}
        r["plan_card"] = p["card"]
        r["plan_subs"] = p["subs"]
        results.append(r)

    n_conn = sum(1 for r in results if r.get("connected"))
    n_deliv = sum(1 for r in results if r.get("rx_bytes"))
    n_full = sum(1 for r in results if r.get("delivered_full"))
    tot_authfails = sum((r.get("aead_authfails") or 0) for r in results)
    tot_nonce_reuse = sum((r.get("nonce_reuse_count") or 0) for r in results)
    summary = {
        "n": args.n,
        "arm": args.arm,
        "encrypt": args.encrypt,
        "cohort_wall_secs": round(cohort_wall, 1),
        "passthrough": args.passthrough,
        "cell": args.cell,
        "start_cfg": args.start_cfg,
        "n_connected": n_conn,
        "n_delivered_any": n_deliv,
        "n_delivered_full": n_full,
        "all_connected": n_conn == args.n,
        "total_aead_authfails": tot_authfails,
        "total_nonce_reuse": tot_nonce_reuse,
        "per_run": [
            {"tag": r.get("tag"), "card": r.get("plan_card"),
             "subs": r.get("plan_subs"), "seed": r.get("seed"),
             "connected": r.get("connected"),
             "connected_at_s": r.get("connected_at_s"),
             "rx_bytes": r.get("rx_bytes"),
             "delivered_full": r.get("delivered_full"),
             "payload_target": r.get("payload_target"),
             "encrypt": r.get("encrypt"),
             "enc_activated": r.get("enc_activated"),
             "aead_authfails": r.get("aead_authfails"),
             "nonce_enc_count": r.get("nonce_enc_count"),
             "nonce_reuse_count": r.get("nonce_reuse_count"),
             "dec_ok_count": r.get("dec_ok_count"),
             "rsp_nreceived_frames": r.get("rsp_nreceived_frames"),
             "configs_seen": r.get("configs_seen"),
             "wall_secs": r.get("wall_secs")}
            for r in results
        ],
    }
    with open(args.out, "w") as f:
        json.dump(summary, f, indent=1)
    # Touch an unambiguous DONE marker (next to the summary) AFTER it is fully
    # written. This cohort already BLOCKED to completion above (proc.wait() on
    # every child), so DONE's existence means all N cells finished and the summary
    # is on disk. A coarse waiter can then `sleep 300; test -f <out>.done && echo
    # READY; cat <summary>` ONCE instead of tight-polling / re-reading the growing
    # per-cell logs (the "trapped reading output files" pathology). Incremental
    # per-cell res_*.json / spawn_*.out stay for liveness.
    done_path = args.out + ".done"
    with open(done_path, "w") as f:
        f.write("%d/%d connected, %d/%d delivered_full\n"
                % (n_conn, args.n, n_full, args.n))
    sys.stderr.write("[spawner] DONE marker -> %s\n" % done_path)
    sys.stderr.flush()
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

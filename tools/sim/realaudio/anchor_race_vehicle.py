#!/usr/bin/env python3
"""
anchor_race_vehicle.py — scripted SET_LINK_PARAMS batch-size climb vehicle +
construction-meter parser for the post-turnaround first-frame loss.

WHAT THIS REPRODUCES
--------------------
On a CLEAN channel the gearshift climbs to CONFIG_16 and the batch-size policy
(axis-2) elects a larger batch: it sends SET_LINK_PARAMS raising data_batch_size
(observed 25 -> 30). The responder applies the new batch size while it is still
draining its post-ACK turnaround; the FIRST frame of the first larger batch
(sequence 0) is lost, and the batch is SACK'd one short (frame-0 missing).

Two discard mechanisms live inside that turnaround window; this vehicle measures
BOTH so a prevention fix is not scored against the wrong needle:

  NEEDLE 1  — samples zeroed while rx_mute=1 (capture-prep zero-writer,
              source/audioio/audioio.c). Emitted by the in-binary construction
              meter as [CBC-METER-EAT] / [CBC-METER-SUMMARY] when
              MERCURY_CBC_METER=1. On these device-free cables a full-amplitude
              buffer zeroed while muted is incoming peer signal discarded.
  NEEDLE 2  — a peer frame whose preamble is DETECTED (high correlation metric)
              immediately after the turnaround flush but whose frame read then
              FAILS ([FTR-FAIL] with metric>~0.9), so the receiver re-locks on
              the SECOND frame. This is the receiver re-arm window not being
              sample-anchored to the incoming frame; it is the direct cause of
              the observed sequence-0 loss.

VEHICLE
-------
A cohort of N real-audio cells (arq_realaudio.py, -x alsa: a REAL ALSA capture
thread => the REAL post-TX mute drain, not the in-process sim no-op). Each cell
runs on its own snd-aloop card and disjoint ports so the cohort is card-scoped
and never reaps a sibling. The loss is bimodal run-to-run, so a cohort (n>=8)
is required to read a RATE rather than a single draw.

RECOVERY IS DEFEATED for every cell (the four defeat knobs) so the meter reads
PREVENTION alone with no recovery machinery masking the loss.

USAGE
  python3 anchor_race_vehicle.py --bin ./mercury --n 8 --card-base 0 \
      --secs 500 --cell WGN:40 --outdir /dev/shm/anchor_race

Parse-only (re-score an existing cohort dir without re-running):
  python3 anchor_race_vehicle.py --parse-only --outdir /dev/shm/anchor_race
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ARQ_REALAUDIO = os.path.join(HERE, "arq_realaudio.py")

# Recovery-DEFEAT knobs: measure prevention alone (no recovery masking the loss).
DEFEAT_ENV = {
    "MERCURY_GAP_RECOVER_DEFEAT": "1",
    "MERCURY_CMD_PREV_RETAIN_DEFEAT": "1",
    "MERCURY_GAP_RECOVER_TURNAROUND_DEFEAT": "1",
    "MERCURY_HELD_CUR_DELIVER_DEFEAT": "1",
}

# ---- log tokens (verified against a live run of the base binary) ------------
RE_T          = re.compile(r"\[T\+([0-9.]+)\]")
RE_SIDE       = re.compile(r"\[(RSP|CMD)\]")
RE_CONNECTED  = re.compile(r"link_status:Connected to")
RE_LINKPARAMS = re.compile(r"\[RSP-LINK-PARAMS\] APPLIED batch (\d+) -> (\d+)")
RE_RXBATCHSEQ = re.compile(r"\[RX-BATCH-SEQ\].*seq=(\d+)\s+batch_seq_id=(\d+)")
RE_ACKGATE    = re.compile(r"\[ACK-GATE\] SACK: received (\d+)/(\d+) \(expected (\d+)\)")
RE_CBC_EAT    = re.compile(r"\[CBC-METER-EAT\].*rms=([0-9.]+) pk=([0-9.]+)")
RE_CBC_SUMM   = re.compile(
    r"\[CBC-METER-SUMMARY\] signal_eaten_samples=(\d+) signal_eaten_events=(\d+) "
    r"noise_muted_events=(\d+) total_muted_samples=(\d+) peak=([0-9.]+)")
RE_FTRFAIL    = re.compile(r"\[FTR-FAIL\].*metric=([0-9.]+)")
# Tightened needle-2 tokens (config-scoped FTR-FAIL, sub-peak reject, turnaround flush).
RE_FTRFAIL_CFG = re.compile(r"\[FTR-FAIL\] CONFIG_(\d+) .*metric=([0-9.]+)")
RE_SUBPEAK     = re.compile(r"\[SUBPEAK-REJECT\] trial \d+ metric=([0-9.]+) mean_H=([0-9.]+)")
RE_FLUSH       = re.compile(r"rsp_post_ack_flush_done")

FTR_HIGH_METRIC = 0.9   # a preamble this well-correlated is a real frame, not search noise
TARGET_CFG      = 16    # post-transition operating config (the SET_LINK_PARAMS climb reaches CONFIG_16)
SUBPEAK_MEANH   = 0.5   # mean|H| below which a high-metric lock is a Schmidl-Cox sub-peak (the collapse fingerprint)


def _t(line):
    m = RE_T.search(line)
    return float(m.group(1)) if m else None


def _side(line):
    m = RE_SIDE.search(line)
    return m.group(1) if m else None


def parse_cell(logpath):
    """Parse one arq_<tag>.log. Returns a dict of counts + the fail-before verdict.

    Asserts a NONZERO, EXPECTED number of anchor rows matched (connected +
    RX-BATCH-SEQ seen); a parser that matches nothing fabricates a clean pass,
    so every count is reported next to the row total it was drawn from."""
    res = {
        "log": logpath, "parsed": False, "reason": "",
        "connected": False,
        "linkparams_up": [],        # (t, frm, to) transitions raising batch size
        "linkparams_all": 0,
        "rxbatchseq_rows": 0,
        "batches_seen": 0,
        "slot0_loss_batches": [],   # bsi whose first decoded seq != 0
        "slot0_loss_after_up": 0,   # slot-0 losses on the first batch after an up-transition
        "cbc_eat_rsp": 0, "cbc_eat_cmd": 0, "cbc_eat_max_rms": 0.0,
        "cbc_summary": None,
        "ftrfail_total": 0, "ftrfail_highmetric": 0,
        "ftrfail_highmetric_after_up": 0,   # needle 2 (LOOSE), scoped to a flat 4 s window
        "cbc_eat_after_up": 0,              # needle 1, scoped to the turnaround window
        "ackgate_short": 0, "ackgate_rows": 0,
        # --- tightened needle-2 (F1B_DESIGN §3 step-1): config-scoped, flush->first-
        # decode window, SUBPEAK corroborator, per-transition CONJUNCTION with the
        # frame-0 loss. Every count is reported next to its matched-row denominator.
        "flush_rows": 0, "subpeak_rows": 0, "ftrfail_cfg_rows": 0,
        "needle2_tight_events": 0,          # transitions whose CONJUNCTION fired
        "needle2_tight_detail": [],
    }
    if not os.path.isfile(logpath):
        res["reason"] = "log missing"
        return res

    first_seq = {}    # bsi -> (t, first decoded seq)
    up_events = []     # (t, frm, to)
    ftr_hi = []        # timestamps of high-metric FTR-FAILs
    eat_ts = []        # timestamps of CBC-METER-EAT frame eats
    flush_ts = []      # timestamps of rsp_post_ack_flush_done (turnaround flush)
    subpeak_ev = []    # (t, mean_H) for [SUBPEAK-REJECT]
    ftrfail_cfg = []   # (t, cfg, metric) for config-scoped [FTR-FAIL]
    with open(logpath, "r", errors="replace") as f:
        for line in f:
            if RE_CONNECTED.search(line):
                res["connected"] = True
            if RE_FLUSH.search(line):
                ft = _t(line)
                if ft is not None:
                    flush_ts.append(ft)
            m = RE_SUBPEAK.search(line)
            if m:
                st = _t(line)
                if st is not None:
                    subpeak_ev.append((st, float(m.group(2))))
            m = RE_FTRFAIL_CFG.search(line)
            if m:
                ct = _t(line)
                if ct is not None:
                    ftrfail_cfg.append((ct, int(m.group(1)), float(m.group(2))))
            m = RE_LINKPARAMS.search(line)
            if m:
                res["linkparams_all"] += 1
                frm, to = int(m.group(1)), int(m.group(2))
                if to > frm:
                    t = _t(line)
                    up_events.append((t, frm, to))
                    res["linkparams_up"].append({"t": t, "from": frm, "to": to})
            m = RE_RXBATCHSEQ.search(line)
            if m:
                res["rxbatchseq_rows"] += 1
                seq, bsi = int(m.group(1)), int(m.group(2))
                if bsi not in first_seq:
                    first_seq[bsi] = (_t(line), seq)
            m = RE_ACKGATE.search(line)
            if m:
                res["ackgate_rows"] += 1
                got, tot = int(m.group(1)), int(m.group(2))
                if got < tot:
                    res["ackgate_short"] += 1
            m = RE_CBC_EAT.search(line)
            if m:
                rms = float(m.group(1))
                if _side(line) == "RSP":
                    res["cbc_eat_rsp"] += 1
                else:
                    res["cbc_eat_cmd"] += 1
                if rms > res["cbc_eat_max_rms"]:
                    res["cbc_eat_max_rms"] = rms
                et = _t(line)
                if et is not None:
                    eat_ts.append(et)
            m = RE_CBC_SUMM.search(line)
            if m:
                res.setdefault("cbc_summaries", []).append({
                    "side": _side(line),
                    "signal_eaten_samples": int(m.group(1)),
                    "signal_eaten_events": int(m.group(2)),
                    "noise_muted_events": int(m.group(3)),
                    "total_muted_samples": int(m.group(4)),
                    "peak": float(m.group(5)),
                })
            m = RE_FTRFAIL.search(line)
            if m:
                res["ftrfail_total"] += 1
                if float(m.group(1)) > FTR_HIGH_METRIC:
                    res["ftrfail_highmetric"] += 1
                    ft = _t(line)
                    if ft is not None:
                        ftr_hi.append(ft)

    res["batches_seen"] = len(first_seq)
    for bsi, (t, seq) in sorted(first_seq.items()):
        if seq != 0:
            res["slot0_loss_batches"].append({"bsi": bsi, "t": t, "first_seq": seq})

    # slot-0 loss on the FIRST batch received after an up-transition (the target
    # loss), plus the two construction needles scoped to that same turnaround
    # window so they measure THIS loss, not unrelated background events.
    WIN = 4.0   # seconds after the up-transition = the first-batch turnaround window
    for (ut, frm, to) in up_events:
        if ut is None:
            continue
        after = [(t, seq, bsi) for bsi, (t, seq) in first_seq.items()
                 if t is not None and t >= ut]
        after.sort()
        if after and after[0][1] != 0:
            res["slot0_loss_after_up"] += 1
        res["ftrfail_highmetric_after_up"] += sum(1 for t in ftr_hi if ut <= t <= ut + WIN)
        res["cbc_eat_after_up"] += sum(1 for t in eat_ts if ut <= t <= ut + WIN)

    # ---- Tightened needle-2 (the HONEST prevention needle, F1B_DESIGN §3 step-1) ----
    # Per up-transition CONJUNCTION: window = [first rsp_post_ack_flush_done at/after
    # the up-transition -> arrival (first RX-BATCH-SEQ) of the first decoded batch in
    # that window]; the event fires iff a mechanism token is present IN that window
    # (a high-metric CONFIG_16 [FTR-FAIL] OR a [SUBPEAK-REJECT] mean_H<0.5 cluster)
    # AND that batch's first decoded seq != 0 (frame-0 actually lost). This couples
    # the mechanism to the loss instead of tallying them independently, and scopes the
    # window to the true turnaround (not a blanket 4 s), so a rescue that makes frame-0
    # READ drives it to 0 by construction.
    res["flush_rows"] = len(flush_ts)
    res["subpeak_rows"] = len(subpeak_ev)
    res["ftrfail_cfg_rows"] = len(ftrfail_cfg)
    for (ut, frm, to) in up_events:
        if ut is None:
            continue
        fl = [t for t in flush_ts if t is not None and t >= ut]
        win_start = min(fl) if fl else ut
        after = sorted((t, seq, bsi) for bsi, (t, seq) in first_seq.items()
                       if t is not None and t >= win_start)
        if not after:
            continue
        t_first, seq_first, bsi_first = after[0]
        win_end = t_first
        ftr_hi_cfg = sum(1 for (t, cfg, mt) in ftrfail_cfg
                         if cfg == TARGET_CFG and mt > FTR_HIGH_METRIC and win_start <= t <= win_end)
        subpeak_in = sum(1 for (t, mh) in subpeak_ev
                         if mh < SUBPEAK_MEANH and win_start <= t <= win_end)
        mech = (ftr_hi_cfg > 0) or (subpeak_in > 0)
        lost = (seq_first != 0)
        ev = bool(mech and lost)
        if ev:
            res["needle2_tight_events"] += 1
        res["needle2_tight_detail"].append({
            "ut": ut, "win_start": win_start, "win_end": win_end,
            "bsi": bsi_first, "seq_first": seq_first,
            "ftrfail_hi_cfg16_inwin": ftr_hi_cfg, "subpeak_inwin": subpeak_in,
            "event": ev})

    # anchor-row assertion: a real, connected cell decodes many batches.
    if not res["connected"]:
        res["reason"] = "cell never connected"
        return res
    if res["rxbatchseq_rows"] == 0:
        res["reason"] = "no RX-BATCH-SEQ rows (nothing decoded) — parser/vehicle mismatch"
        return res
    res["parsed"] = True

    # Fail-before verdict for this cell.
    res["eat_reproduced"] = bool(res["slot0_loss_after_up"] > 0)
    res["needle1_mute_eat_fired"] = bool((res["cbc_eat_rsp"] + res["cbc_eat_cmd"]) > 0)
    res["needle2_rearm_ftrfail_fired"] = bool(res["ftrfail_highmetric_after_up"] > 0)  # LOOSE
    res["needle2_tight_fired"] = bool(res["needle2_tight_events"] > 0)                 # TIGHT (graded needle)
    return res


def run_cohort(args):
    os.makedirs(args.outdir, exist_ok=True)
    env = dict(os.environ)
    procs = []
    for i in range(args.n):
        card_idx = args.card_base + i
        card = "Loopback" if card_idx == 0 else f"Loopback_{card_idx}"
        tag = f"anchor_c{card_idx}"
        rsp_port = args.port_base + i * 8
        cmd_port = rsp_port + 4
        envs = []
        for k, v in DEFEAT_ENV.items():
            envs += ["--env", f"{k}={v}"]
        envs += ["--env", "MERCURY_CBC_METER=1"]
        for kv in args.extra_env:
            envs += ["--env", kv]
        if args.signal_rms is not None:
            envs += ["--env", f"MERCURY_CBC_SIGNAL_RMS={args.signal_rms}"]
        cmd = [sys.executable, ARQ_REALAUDIO,
               "--bin", args.bin, "--card", card, "--subs", "0,1,2,3",
               "--rsp-port", str(rsp_port), "--cmd-port", str(cmd_port),
               "--cell", args.cell, "--profile", args.profile,
               "--start-cfg", "100", "--secs", str(args.secs),
               "--payload", str(args.payload), "--seed", str(args.seed_base + i),
               "--no-kill", "--tag", tag, "--logdir", args.outdir,
               "--json", os.path.join(args.outdir, f"{tag}.json")] + envs
        out = open(os.path.join(args.outdir, f"{tag}.out"), "w")
        procs.append((tag, card, subprocess.Popen(cmd, stdout=out, stderr=out)))
        print(f"launched {tag} on {card} ports {rsp_port}/{cmd_port} seed {args.seed_base+i}")
        time.sleep(args.stagger)

    print(f"waiting for {len(procs)} cells (secs={args.secs}) ...")
    for tag, card, p in procs:
        p.wait()
        print(f"cell {tag} exited rc={p.returncode}")
    # arq_realaudio.py writes its merged log as arq_<tag>.log
    return [os.path.join(args.outdir, f"arq_anchor_c{args.card_base+i}.log")
            for i in range(args.n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default="./mercury")
    ap.add_argument("--n", type=int, default=8, help="cohort size (>=8 for the bimodal rate)")
    ap.add_argument("--card-base", type=int, default=0)
    ap.add_argument("--port-base", type=int, default=7002)
    ap.add_argument("--seed-base", type=int, default=1)
    ap.add_argument("--secs", type=int, default=500)
    ap.add_argument("--payload", type=int, default=262144)
    ap.add_argument("--cell", default="WGN:40")
    ap.add_argument("--profile", default="wgn")
    ap.add_argument("--signal-rms", default=None,
                    help="override MERCURY_CBC_SIGNAL_RMS (frame/noise split)")
    ap.add_argument("--stagger", type=float, default=3.0)
    ap.add_argument("--extra-env", action="append", default=[],
                    help="KEY=VAL env passed to the mercury child (repeatable)")
    ap.add_argument("--outdir", default="/dev/shm/anchor_race")
    ap.add_argument("--parse-only", action="store_true",
                    help="re-score arq_*.log already in --outdir; do not launch")
    args = ap.parse_args()

    if args.parse_only:
        logs = sorted(
            os.path.join(args.outdir, fn)
            for fn in os.listdir(args.outdir)
            if re.match(r"arq_anchor_c\d+\.log$", fn))
        if not logs:
            print(f"FATAL: no arq_anchor_c*.log in {args.outdir}", file=sys.stderr)
            sys.exit(2)
    else:
        logs = run_cohort(args)

    cells = [parse_cell(lp) for lp in logs]
    parsed = [c for c in cells if c.get("parsed")]

    agg = {
        "cohort_n": len(cells),
        "cells_connected": sum(1 for c in cells if c["connected"]),
        "cells_parsed": len(parsed),
        "cells_with_linkparams_up": sum(1 for c in parsed if c["linkparams_up"]),
        "cells_eat_reproduced": sum(1 for c in parsed if c.get("eat_reproduced")),
        "cells_needle1_mute_eat": sum(1 for c in parsed if c.get("needle1_mute_eat_fired")),
        "cells_needle2_rearm_ftrfail": sum(1 for c in parsed if c.get("needle2_rearm_ftrfail_fired")),
        "cells_needle2_tight": sum(1 for c in parsed if c.get("needle2_tight_fired")),
        "needle2_tight_events_total": sum(c.get("needle2_tight_events", 0) for c in parsed),
        "cells": cells,
    }
    # denominators printed next to every rate (a matched-nothing parser fabricates a pass).
    print("\n===== ANCHOR-RACE FAIL-BEFORE (recovery DEFEATED) =====")
    print(f"cohort cells          : {agg['cohort_n']}")
    print(f"connected             : {agg['cells_connected']}/{agg['cohort_n']}")
    print(f"parsed (decoded data) : {agg['cells_parsed']}/{agg['cohort_n']}")
    print(f"saw LINK-PARAMS up    : {agg['cells_with_linkparams_up']}/{agg['cells_parsed']}")
    print(f"slot-0 loss reproduced: {agg['cells_eat_reproduced']}/{agg['cells_with_linkparams_up']}"
          "  (post-LINK-PARAMS first-batch frame-0 lost)")
    print(f"needle1 mute-eat fired: {agg['cells_needle1_mute_eat']}/{agg['cells_parsed']}"
          "  ([CBC-METER-EAT] peer frame zeroed while muted)")
    print(f"needle2 re-arm FTRFAIL: {agg['cells_needle2_rearm_ftrfail']}/{agg['cells_parsed']}"
          "  (LOOSE: high-metric [FTR-FAIL] in a flat 4 s window)")
    print(f"needle2 TIGHT (graded) : {agg['cells_needle2_tight']}/{agg['cells_parsed']}"
          f"  (flush->first-decode window, CFG16 FTR-FAIL/SUBPEAK ∧ seq0-lost; "
          f"{agg['needle2_tight_events_total']} transition-events)")
    for c in cells:
        tag = os.path.basename(c["log"])
        if not c.get("parsed"):
            print(f"  {tag:22s} NOT-PARSED: {c['reason']}")
            continue
        print(f"  {tag:22s} conn={c['connected']} up={len(c['linkparams_up'])} "
              f"batches={c['batches_seen']} slot0_loss_after_up={c['slot0_loss_after_up']} "
              f"needle2_TIGHT={c['needle2_tight_events']} "
              f"cbc_eat(rsp/cmd)={c['cbc_eat_rsp']}/{c['cbc_eat_cmd']} "
              f"ftrfail_hi_after_up={c['ftrfail_highmetric_after_up']} "
              f"(rows: flush={c['flush_rows']} subpeak={c['subpeak_rows']} ftrfail_cfg={c['ftrfail_cfg_rows']} "
              f"ftrfail_hi_total={c['ftrfail_highmetric']}/{c['ftrfail_total']}) "
              f"ackgate_short={c['ackgate_short']}/{c['ackgate_rows']}")

    outjson = os.path.join(args.outdir, "anchor_race_result.json")
    with open(outjson, "w") as f:
        json.dump(agg, f, indent=2)
    print(f"\nwrote {outjson}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
build_ack_arrival_corpus.py -- harvest the TRUE reverse-ACK arrival distribution
from every existing Mercury ARQ log and write it to a durable, replayable corpus
(_research/ack_arrival_corpus.json).  (metric-correction 2026-07-03, FIX 4.)

WHY: the T1 deterministic ACK-slot lever used a FIXED slot (pre-fix 1403 ms, fixed
1724 ms) sized to a MODELLED MFSK-ACK airtime.  The real reverse-ACK arrival (ms
from end-of-TX to the ACK landing at the commander) is a DISTRIBUTION with a heavy
retx/partial tail, so a fixed slot clips the tail and the ACK arrives after the slot
already closed -> 0-delivery (the T1-v2 miss: arm-c 0/4 in _research/_t1_duty/).
This corpus is the OFFLINE validation set for a T1-v3 ADAPTIVE slot estimator: any
candidate slot rule can be replayed against these real arrivals at ZERO fleet cost,
and a fixed 1724 ms slot MUST reproduce the miss (fail-before) here.

WHAT it harvests: Mercury already prints the true arrival on every data-ACK detect
  [CMD-MFSK-ACK-SACK] CLEAN|PARTIAL ... arrival_ms=N   (arq_commander.cc)
  [CMD-SACK-V2] decoded SACK_RSP ... arrival_ms=N       (arq_commander.cc)
  [CMD-COMPACT-CONFIRM] CLEAN ... arrival_ms=N          (arq_commander.cc)
  [CMD-ACK-PAT] Data ACK pattern detected! elapsed=Nms  (branch
                    metrics/ack-arrival-elapsed off monitor; the bare clean funnel)
receiving_timer.start() fires at end-of-TX (arq_commander.cc:1952) so every value is
the true end-of-TX -> ACK ms.  Each arrival is keyed to the config + batch_size of the
batch it ACKs (most-recent preceding [CMD-TX] CONFIG_c and [CMD-POST-TX] batch=b).

Handles BOTH log formats: harness-stamped ("[T+SSSSS.sss] [CMD|RSP] <text>") and raw
mercury stdout (no prefix, all lines are the commander's).

REPLAY (documented in the corpus _meta): a fixed slot S "misses" an ACK iff
arrival_ms > S.  miss_rate(S) = fraction of arrivals exceeding S.  The corpus reports
miss_rate at the pre-fix (1403) and fixed-T1 (1724) slots plus a sweep, and an
example ADAPTIVE rule (per-(config,batch_size) p90 + margin) whose miss_rate is far
lower -- the fail-before / pass-after the T1-v3 estimator must reproduce offline.
"""
import glob
import json
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))   # workspace root
OUT = os.path.join(WS, "_research", "ack_arrival_corpus.json")

# lenient line parser: strip an optional harness "[T+SSSSS.sss] [CMD|RSP] " prefix.
PREFIX_RE = re.compile(r"^\[T\+\d+(?:\.\d+)?\]\s+\[(CMD|RSP)\]\s+(.*)$")
CMDTX_RE = re.compile(r"\[CMD-TX\]\s+CONFIG_(\d+)\s+batch=(\d+)")
POSTTX_RE = re.compile(r"\[CMD-POST-TX\]\s+receiving_timeout=(\d+)ms\s+msg_tx_time=(\d+)ms\s+batch=(\d+)")
ARRIVAL_RE = re.compile(
    r"\[(CMD-MFSK-ACK-SACK|CMD-SACK-V2|CMD-COMPACT-CONFIRM|CMD-ACK-PAT)\][^\n]*?"
    r"(?:arrival_ms=(\d+)|elapsed=(\d+)ms)")
KIND_RE = re.compile(r"\b(CLEAN|PARTIAL)\b")
CONTROL_ACK = "Control ACK for code="

# Fixed-slot reference points for the replay.
SLOT_PREFIX = 1403     # pre-fix T1 slot (closed before the SACK ACK landed)
SLOT_FIXED = 1724      # the "fixed" T1 slot (fix/t1-ackslot-sizing @3d900692)
SLOT_SWEEP = [1200, 1403, 1600, 1724, 2000, 2500, 3000, 4000, 5000]


def line_iter(path):
    """Yield (side, text) for each log line, harness-prefix-stripped; raw -> CMD."""
    with open(path, "r", errors="replace") as f:
        for raw in f:
            raw = raw.rstrip("\n")
            m = PREFIX_RE.match(raw)
            if m:
                yield m.group(1), m.group(2)
            else:
                yield "CMD", raw   # raw mercury stdout: commander side


def harvest(path):
    """Return list of {arrival_ms,kind,config,batch_size} for one log."""
    rows = []
    cur_cfg = None
    cur_bsize = None
    for side, txt in line_iter(path):
        if side != "CMD":
            continue
        mc = CMDTX_RE.search(txt)
        if mc:
            cur_cfg = int(mc.group(1))
            continue
        mp = POSTTX_RE.search(txt)
        if mp:
            cur_bsize = int(mp.group(3))
            continue
        if CONTROL_ACK in txt:
            continue
        ma = ARRIVAL_RE.search(txt)
        if ma:
            ms = int(ma.group(2) or ma.group(3))
            km = KIND_RE.search(txt)
            if km:
                kind = km.group(1).lower()
            elif ma.group(1) == "CMD-SACK-V2":
                kind = "sack"
            elif ma.group(1) == "CMD-ACK-PAT":
                kind = "bare"
            else:
                kind = None
            rows.append({"arrival_ms": ms, "kind": kind,
                         "config": cur_cfg, "batch_size": cur_bsize})
    return rows


def dist(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    n = len(xs)

    def pct(p):
        if n == 1:
            return xs[0]
        k = (n - 1) * p
        lo = int(k)
        hi = min(lo + 1, n - 1)
        return round(xs[lo] + (xs[hi] - xs[lo]) * (k - lo), 1)
    return {"n": n, "min": xs[0], "p10": pct(0.10), "median": round(statistics.median(xs), 1),
            "mean": round(statistics.mean(xs), 1), "p90": pct(0.90), "p95": pct(0.95),
            "max": xs[-1]}


def miss_rate(xs, slot):
    xs = [x for x in xs if x is not None]
    if not xs:
        return None
    return round(sum(1 for x in xs if x > slot) / len(xs), 4)


def source_class(path):
    p = path.replace("\\", "/")
    if "climbdiag" in p or "/hw" in p or "ionos" in p.lower():
        return "hardware"
    if "_sim" in p or "cascade_repro" in p:
        return "sim_inprocess"
    return "real_audio_sim"


def main():
    logs = []
    for pat in ("_research/**/*.log",):
        logs += glob.glob(os.path.join(WS, pat), recursive=True)
    logs = sorted(set(logs))
    all_rows = []
    per_log = []
    for lp in logs:
        rows = harvest(lp)
        if not rows:
            continue
        rel = os.path.relpath(lp, WS).replace("\\", "/")
        cls = source_class(lp)
        for r in rows:
            r["_src"] = rel
            r["_class"] = cls
        all_rows += rows
        per_log.append({"log": rel, "class": cls, "n_arrivals": len(rows),
                        "configs": sorted({r["config"] for r in rows if r["config"] is not None}),
                        "batch_sizes": sorted({r["batch_size"] for r in rows if r["batch_size"] is not None}),
                        "arrival_dist": dist([r["arrival_ms"] for r in rows])})

    all_ms = [r["arrival_ms"] for r in all_rows]

    def bucket(key_fn):
        b = {}
        for r in all_rows:
            k = key_fn(r)
            if k is None:
                continue
            b.setdefault(k, []).append(r["arrival_ms"])
        return {str(k): {"dist": dist(v),
                         "miss_rate@1724": miss_rate(v, SLOT_FIXED),
                         "miss_rate@1403": miss_rate(v, SLOT_PREFIX)}
                for k, v in sorted(b.items(), key=lambda kv: str(kv[0]))}

    by_config = bucket(lambda r: r["config"])
    by_config_batch = bucket(lambda r: (r["config"], r["batch_size"])
                             if r["config"] is not None and r["batch_size"] is not None else None)
    by_kind = bucket(lambda r: r["kind"])
    by_class = bucket(lambda r: r["_class"])

    # cfg16-specific slice (the T1-duty pinned config) for the reconciliation.
    cfg16_ms = [r["arrival_ms"] for r in all_rows if r["config"] == 16]

    # ADAPTIVE-rule example: slot = per-(config,batch_size) p90 + 250ms margin,
    # replayed against every arrival (fallback to global p90 when a key is unseen).
    key_p90 = {}
    kb = {}
    for r in all_rows:
        if r["config"] is not None and r["batch_size"] is not None:
            kb.setdefault((r["config"], r["batch_size"]), []).append(r["arrival_ms"])
    for k, v in kb.items():
        key_p90[k] = dist(v)["p90"]
    global_p90 = dist(all_ms)["p90"]
    adaptive_miss = 0
    adaptive_n = 0
    for r in all_rows:
        k = (r["config"], r["batch_size"])
        slot = (key_p90.get(k, global_p90) or global_p90) + 250
        adaptive_n += 1
        if r["arrival_ms"] > slot:
            adaptive_miss += 1
    adaptive_rate = round(adaptive_miss / adaptive_n, 4) if adaptive_n else None

    corpus = {
        "_meta": {
            "purpose": "OFFLINE validation set for the T1-v3 adaptive reverse-ACK-slot "
                       "estimator. Real end-of-TX->ACK arrival ms harvested from existing "
                       "Mercury ARQ logs. A fixed slot S misses an ACK iff arrival_ms>S.",
            "generated_by": "tools/sim/realaudio/build_ack_arrival_corpus.py",
            "arrival_semantics": "ms from end-of-TX (receiving_timer.start at "
                                 "arq_commander.cc:1952) to the data-ACK detect at the commander.",
            "sources": "[CMD-MFSK-ACK-SACK]/[CMD-SACK-V2]/[CMD-COMPACT-CONFIRM] arrival_ms= "
                       "(already in mercury) + [CMD-ACK-PAT] elapsed= (branch "
                       "metrics/ack-arrival-elapsed off monitor 7860d37a).",
            "replay_howto": "load all_arrivals_ms (or a by_config/by_config_batch slice); "
                            "for a candidate slot S, miss_rate = mean(arrival_ms > S). "
                            "Reproduce the T1-v2 miss with S=1724 (see fixed_slot_replay).",
            "n_logs_harvested": len(per_log),
            "n_arrivals_total": len(all_ms),
            "slot_prefix_ms": SLOT_PREFIX, "slot_fixed_ms": SLOT_FIXED,
            "note": "Sources mixed hardware / in-process-sim / real-audio-sim; see "
                    "by_source_class + per_log[].class. Bench-claim guard: HW arrivals are "
                    "real measurements, not modelled.",
        },
        "overall_dist": dist(all_ms),
        "fixed_slot_replay": {
            "miss_rate_by_slot_ms": {str(s): miss_rate(all_ms, s) for s in SLOT_SWEEP},
            "miss_rate@1403_prefix": miss_rate(all_ms, SLOT_PREFIX),
            "miss_rate@1724_fixed": miss_rate(all_ms, SLOT_FIXED),
            "adaptive_p90+250_miss_rate": adaptive_rate,
            "verdict": ("A fixed 1724ms slot misses %.1f%% of all real ACKs (%.1f%% at the "
                        "pre-fix 1403ms slot); the per-(config,batch) p90+250ms adaptive rule "
                        "misses %.1f%% -- the fail-before/pass-after the T1-v3 estimator "
                        "must reproduce offline."
                        % (100 * (miss_rate(all_ms, SLOT_FIXED) or 0),
                           100 * (miss_rate(all_ms, SLOT_PREFIX) or 0),
                           100 * (adaptive_rate or 0))),
        },
        "cfg16_slice": {
            "dist": dist(cfg16_ms),
            "miss_rate@1724": miss_rate(cfg16_ms, SLOT_FIXED),
            "miss_rate@1403": miss_rate(cfg16_ms, SLOT_PREFIX),
            "note": "cfg16 is the T1-duty pinned rung; this slice is the direct "
                    "arm-A/arm-c reconciliation set.",
        },
        "by_config": by_config,
        "by_config_batch": by_config_batch,
        "by_kind": by_kind,
        "by_source_class": by_class,
        "per_log": per_log,
        "all_arrivals_ms": all_ms,
    }

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        json.dump(corpus, f, indent=1)
    print("wrote", OUT)
    print("logs harvested:", len(per_log), " arrivals:", len(all_ms))
    print("overall arrival dist:", corpus["overall_dist"])
    print("fixed-slot replay:", json.dumps(corpus["fixed_slot_replay"], indent=1))
    print("cfg16 slice:", json.dumps(corpus["cfg16_slice"], indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

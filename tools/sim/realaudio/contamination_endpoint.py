#!/usr/bin/env python3
"""Endpoint-bounded contamination flag for twin cells (sidecar to the frozen scorer).

Rule (CONTRACT_RULES section 8, decision D84): a cell is instrument-invalid when a
source clip ([TX-CLIP]), bridge input_at_fs > 0 or bridge hard_clips > 0 occurs
BEFORE the cell endpoint (byte-exact completion, or the last delivered byte for an
incomplete cell). Emissions after the endpoint are recorded, not disqualifying.

What is time-located and what is not:
  * [TX-CLIP] lines carry the harness clock (T+seconds) and the emitting peer. A line is
    a WINDOW report, not a clip instant: the playback thread plays
    continuously (zeros when idle) and prints after 48,000 played samples (mercury
    audioio.c:1453-1468 @95723953), so the clipped samples were written in the 1.0 s
    before the print, plus at most one ALSA buffer (30 ms, audioio.c:1229-1231). The
    harness reads the line after it is printed, so its window is taken as
    [T - WINDOW_S, T] and a window that starts at or before the endpoint counts as
    BEFORE the endpoint.
  * Bridge input_at_fs / hard_clips are per-direction CELL TOTALS; bursts.jsonl has no
    per-burst clip count. They are placed in time only by ACCOUNTING IDENTITY:
      - input_at_fs(dir) is explained when it is <= the summed sample counts of the
        post-endpoint [TX-CLIP] lines of the peer transmitting in that direction
        (fwd = CMD -> RSP, rev = RSP -> CMD). The modem clamps at full scale, so each
        clamped sample arrives at the bridge input at full scale.
      - hard_clips(dir) is explained when input_at_fs(dir) is explained AND
        hard_clips(dir) <= input_at_fs(dir) (the post-channel clip rides on the
        full-scale input samples). This part is a SCREEN, not an identity: a fade
        peak coincident with the source clip cannot be excluded from cell totals.
    Anything unexplained counts as BEFORE the endpoint (conservative).
  * Endpoint of an incomplete cell: the result's last_rx_at_s (arrival of the last
    delivered byte, harness clock = the [T+] clock), written by harnesses that carry
    the harness result field. Results without that field, and cells that delivered nothing, have
    no derivable endpoint: with any flag they stay invalid as ENDPOINT-UNDERIVED.

Usage: contamination_endpoint.py <raw_dir> [--board-reduction RAW_REDUCTION.json] [--json OUT]
"""
import argparse
import json
import pathlib
import re
import sys

CLIP_RX = re.compile(rb"^\[T\+(\d+(?:\.\d+)?)\] \[(CMD|RSP)\] \[TX-CLIP\] (\d+) samples clipped")
DIR_OF_PEER = {"CMD": "fwd", "RSP": "rev"}
# [TX-CLIP] report window: 48,000 played samples at 48 kHz + one 30 ms ALSA buffer.
WINDOW_S = 48000 / 48000.0 + 0.030


def tx_clip_events(arq_log):
    events = []
    raw_mentions = 0
    with open(arq_log, "rb") as f:
        for line in f:
            if b"[TX-CLIP]" not in line:
                continue
            raw_mentions += line.count(b"[TX-CLIP]")
            m = CLIP_RX.match(line)
            if m:
                events.append((float(m.group(1)), m.group(2).decode(), int(m.group(3))))
    return events, raw_mentions


def classify(cell_dir):
    result = json.loads((cell_dir / "result.json").read_text())
    stats_paths = list((cell_dir / "logs").glob("bridge_*_stats.json"))
    arq_paths = list((cell_dir / "logs").glob("arq_*.log"))
    if len(stats_paths) != 1 or len(arq_paths) != 1:
        raise AssertionError("artifact census failed: %s" % cell_dir)
    stats = json.loads(stats_paths[0].read_text())
    events, raw_mentions = tx_clip_events(arq_paths[0])
    if len(events) != raw_mentions:
        raise AssertionError("[TX-CLIP] grammar mismatch in %s: parsed %d of %d"
                             % (arq_paths[0], len(events), raw_mentions))
    complete = (result.get("whole_session_status") == "COMPLETE"
                and result.get("completion_at_s") is not None)
    if complete:
        endpoint, endpoint_source = float(result["completion_at_s"]), "completion_at_s"
    elif result.get("last_rx_at_s") is not None:
        endpoint, endpoint_source = float(result["last_rx_at_s"]), "last_rx_at_s"
    else:
        endpoint, endpoint_source = None, None
    row = {"cell": cell_dir.name, "status": result.get("whole_session_status"),
           "endpoint_s": endpoint, "endpoint_source": endpoint_source,
           "tx_clip_events": len(events),
           "tx_clip_times": [e[0] for e in events]}
    pre = [e for e in events if endpoint is None or e[0] - WINDOW_S <= endpoint]
    post = [e for e in events if endpoint is not None and e[0] - WINDOW_S > endpoint]
    row["tx_clip_pre_endpoint"] = len(pre)
    row["tx_clip_post_endpoint"] = len(post)
    reasons = []
    flagged = bool(events)
    for d in ("fwd", "rev"):
        infs = int(stats.get(d, {}).get("input_at_fs", 0))
        hard = int(stats.get(d, {}).get("hard_clips", 0))
        post_sum = sum(e[2] for e in post if DIR_OF_PEER[e[1]] == d)
        row[d] = {"input_at_fs": infs, "hard_clips": hard, "post_endpoint_clip_samples": post_sum}
        if infs or hard:
            flagged = True
        if endpoint is None:
            continue
        infs_explained = infs <= post_sum
        if infs and not infs_explained:
            reasons.append("%s-input_at_fs-unexplained" % d)
        if hard and not (infs and infs_explained and hard <= infs):
            reasons.append("%s-hard_clips-unexplained" % d)
    if pre:
        reasons.append("tx_clip-before-endpoint")
    if endpoint is None and flagged:
        reasons.append("endpoint-underived")
    row["flagged_any"] = flagged
    row["endpoint_bounded_contaminated"] = bool(reasons)
    row["endpoint_bounded_reasons"] = sorted(set(reasons))
    row["hard_clip_attribution"] = "screen"
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("raw_dir")
    ap.add_argument("--board-reduction")
    ap.add_argument("--json")
    a = ap.parse_args()
    raw = pathlib.Path(a.raw_dir)
    cells = sorted(p.parent for p in raw.glob("*_s[1-9]/result.json"))
    if not cells:
        print("no cells matched under %s" % raw, file=sys.stderr)
        return 2
    rows = [classify(c) for c in cells]
    out = {"raw_dir": str(raw), "cells": len(rows), "rows": rows}
    print("cells parsed: %d" % len(rows))
    print("flagged (any clip evidence, whole cell): %d" % sum(r["flagged_any"] for r in rows))
    print("contaminated under endpoint bound: %d" % sum(r["endpoint_bounded_contaminated"] for r in rows))
    if a.board_reduction:
        board = json.loads(pathlib.Path(a.board_reduction).read_text())
        by_cell = {}
        for br in board["rows"]:
            key = br.get("cell") or br.get("tag")
            by_cell[key] = br
        mine = {r["cell"]: r for r in rows}
        invalid = [k for k, br in by_cell.items() if not br.get("instrument_valid", True)]
        recovered, still, other = [], [], []
        for k in sorted(invalid):
            br = by_cell[k]
            rs = set(br.get("instrument_invalid_reasons", []))
            r = mine.get(k)
            if r is None:
                other.append((k, "not-in-raw"))
                continue
            if rs == {"clips"} and not r["endpoint_bounded_contaminated"]:
                recovered.append(k)
            elif rs - {"clips"}:
                other.append((k, ",".join(sorted(rs - {"clips"}))))
            else:
                still.append((k, ",".join(r["endpoint_bounded_reasons"])))
        out["board_compare"] = {"board_invalid": len(invalid), "recovered": recovered,
                                "still_contaminated": still, "invalid_for_other_reasons": other}
        print("board invalid: %d; recovered by endpoint bound: %d; still contaminated: %d; other reasons: %d"
              % (len(invalid), len(recovered), len(still), len(other)))
        print("recovered:", " ".join(recovered))
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

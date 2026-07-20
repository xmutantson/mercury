#!/usr/bin/env python3
"""
score_climb_canonical.py - THE single canonical scorer for the ROBUST->WB-OFDM
climb and the WB-OFDM crossing. Every climb / crossing analysis should import
score_log() from here (or run this as a CLI). All other climb scorers that key
on `current=M` or on a CMD-side `SET_CONFIG: forward=N` announcement are
DEPRECATED in favour of this one (see HISTORY below).

WHY THIS EXISTS (the scorer-bug graveyard)
==========================================
Two scorer bug classes repeatedly produced FALSE climb verdicts and drove wasted
fixes and misdiagnoses:

  BUG-A "current= keying": parsing `load_configuration(N) current=M` and keying
    the ACTIVE config on `current=M` (the OUTGOING/previous config, printed
    BEFORE the assignment at arq_common.cc:2129) instead of the TARGET `N` (the
    config being loaded, which becomes active at :2129). Result: BLIND to
    inband-adopted WB configs -> `wb_configs_seen=[]` / `crossed=NO` FALSE
    NEGATIVES even when the RSP genuinely crossed to CONFIG_0 and decoded WB DATA.
    Bitten in: arq_realaudio.py CFG_RE, score_sustained_wb.py, wb_configs_seen.

  BUG-B "announcement keying": keying the climb/crossing on a non-durable signal
    -- a CMD-side `[GEARSHIFT] SET_CONFIG: forward=N` ANNOUNCEMENT (announced,
    may never hold or decode), the CMD's unilateral WB adoption on a (false)
    bare ACK, or a ROBUST-tier batch drain -- instead of a real RSP DATA decode
    on a WB id. Bitten in: gearshift_cascade_bench.py, sim_arq_channel.py,
    _revack_ab_parse.py, the bigblock benchers.

THE DURABLE CROSS SIGNAL (the only one this scorer trusts)
==========================================================
A ROBUST->WB crossing is REAL only when the RESPONDER (RSP, the receiver of the
data payload):
  (a) emits `[CFG] load_configuration(N)` with N a WB id 0..16  (TARGET-keyed,
      group 1 -- NOT current=M), AND
  (b) decodes >= 1 WB-OFDM DATA frame WHILE active on that WB id: an
      `[RX-BATCH-SEQ] type=DATA*` (or `[RX-DATA] type=...`) line emitted while
      the RSP's current active config (advanced from its own
      load_configuration(N) TARGETs) is a WB id 0..16, AND
  (c) delivered bytes > 0 end-to-end (the harness rx_bytes; passed in via
      --rx-bytes, or read from the harness JSON).

The CMD adopting WB unilaterally (false bare receive_ack_pattern,
arq_commander.cc:2569/7196-7208) does NOT count; neither does the INDIRECT
V2-PREV-DELIVERED ROBUST-batch drain. Both previously masqueraded as crossings.

THE BINDING-CONSTRAINT METRIC (climb PAST cfg0)
===============================================
Crossing to CONFIG_0 (the lowest WB-OFDM rung, ~66 bps) is necessary but not
sufficient -- the campaign's binding constraint is climbing THROUGH cfg0 up to
cfg1..16. This scorer therefore reports BOTH:
  - crossed              : RSP reached + decoded WB-OFDM CONFIG_0..16 at all
  - max_wb_cfg_with_data : highest WB id the RSP decoded a DATA frame on
  - climbed_past_cfg0    : max_wb_cfg_with_data >= 1
so a run that crosses to cfg0 but never climbs past it reads as
crossed=True, climbed_past_cfg0=False -- the true state (see
faithful_beat_vara_climb_binding / WB_ACQ_FIX_RESULT.md).

USAGE
=====
  # combined [T+..] [SIDE] harness log:
  score_climb_canonical.py --log arq_cell.log [--rx-bytes 425] [--json out.json]
  # two separate RSP/CMD stderr captures:
  score_climb_canonical.py --rsp-log rsp.txt --cmd-log cmd.txt [--json out.json]

HISTORY / SUPERSEDED SCORERS (do not use for climb crossing)
============================================================
  arq_realaudio.py (pre-fix)  : BUG-A current=M. FIXED in-place to import this.
  rescore_climb.py (_dictreval_tmp)        : the offline prototype of THIS file.
  arq_realaudio_comp.py (_dictreval_tmp)   : the live-harness prototype of the fix.
  score_sustained_wb.py (fleet)            : BUG-A; superseded by score_wb_fixed.py.
  score_wb_fixed.py (fleet)                : target-keyed; fold into this canonical.
  gearshift_cascade_bench.py, sim_arq_channel.py, _revack_ab_parse.py,
  bigblock_p3_hw/{run_election_climb,run_recovack_e2e,bb_winrun3,
  _wallb/.../wallb_hw_bench5}.py          : BUG-B announcement-keyed (see headers).
"""
import argparse
import json
import re
import sys

# load_configuration(N) current=M : N (group 1) = TARGET config being LOADED
# (becomes active at arq_common.cc:2129); M (group 2) = OUTGOING/previous config,
# printed BEFORE the assignment. KEY ON N (group 1), NEVER on current=M.
# `current=-1` is the default-init value (length=0 / first load), so allow a sign.
CFG_RE = re.compile(r"load_configuration\((\d+)\)\s+current=(-?\d+)")
# Durable WB DATA decode. Mercury prints either form depending on path/version:
#   [RX-BATCH-SEQ] type=DATA_LONG ...   (arq batch path)
#   [RX-DATA] type=... id=... seq=...   (arq_responder.cc:1491)
RXDATA_RE = re.compile(r"\[RX-BATCH-SEQ\]\s+type=DATA|\[RX-DATA\]\s+type=")
NRECV_RE = re.compile(r"stats\.nReceived_data=\s*(\d+)")
BREAK_RE = re.compile(r"\[BREAK\] Block failure")
TSTAMP_RE = re.compile(r"\[T\+\s*([0-9.]+)\]")
SIDE_RE = re.compile(r"\[(RSP|CMD)\]")


def is_wb_config(cfg):
    # WB OFDM = ids 0..16; ROBUST MFSK = 100/101/102 (common_defines.h:122-155).
    return cfg is not None and 0 <= cfg <= 16


class SideState:
    """Per-side (RSP or CMD) climb state, advanced line by line."""
    def __init__(self):
        self.cur_cfg = None
        self.configs_seen = set()
        self.loaded_wb_id = None       # first WB id 0..16 this side load_configuration'd
        self.wb_data_frames = 0        # DATA frames decoded while active on a WB id
        self.max_wb_cfg_with_data = None  # highest WB id a DATA frame decoded on
        self.first_wb_data_at_s = None
        self.nreceived = 0


def feed(side, text, tstamp):
    """Apply one log line to a SideState. Order matters: a load line sets the
    active cfg BEFORE a DATA line on the same logical event is attributed to it.
    """
    m = CFG_RE.search(text)
    if m:
        target = int(m.group(1))            # TARGET config (NOT current=M)
        side.configs_seen.add(target)
        side.cur_cfg = target
        if is_wb_config(target) and side.loaded_wb_id is None:
            side.loaded_wb_id = target
    if RXDATA_RE.search(text) and is_wb_config(side.cur_cfg):
        side.wb_data_frames += 1
        if side.first_wb_data_at_s is None:
            side.first_wb_data_at_s = tstamp
        if side.max_wb_cfg_with_data is None or side.cur_cfg > side.max_wb_cfg_with_data:
            side.max_wb_cfg_with_data = side.cur_cfg
    mn = NRECV_RE.search(text)
    if mn:
        side.nreceived = max(side.nreceived, int(mn.group(1)))


def score_combined(path):
    rsp, cmd = SideState(), SideState()
    breaks = 0
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            sm = SIDE_RE.search(line)
            if not sm:
                continue
            tm = TSTAMP_RE.search(line)
            tstamp = float(tm.group(1)) if tm else None
            if BREAK_RE.search(line):
                breaks += 1
            feed(rsp if sm.group(1) == "RSP" else cmd, line, tstamp)
    return rsp, cmd, breaks


def score_two_files(rsp_path, cmd_path):
    rsp, cmd = SideState(), SideState()
    breaks = 0
    for path, side, label in ((rsp_path, rsp, "RSP"), (cmd_path, cmd, "CMD")):
        if not path:
            continue
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for i, line in enumerate(f):
                tm = TSTAMP_RE.search(line)
                tstamp = float(tm.group(1)) if tm else float(i)
                if BREAK_RE.search(line):
                    breaks += 1
                feed(side, line, tstamp)
    return rsp, cmd, breaks


def score_lines(lines):
    """In-memory variant for tests: a list of combined `[T+..] [SIDE] ...` lines."""
    rsp, cmd = SideState(), SideState()
    breaks = 0
    for line in lines:
        sm = SIDE_RE.search(line)
        if not sm:
            continue
        tm = TSTAMP_RE.search(line)
        tstamp = float(tm.group(1)) if tm else None
        if BREAK_RE.search(line):
            breaks += 1
        feed(rsp if sm.group(1) == "RSP" else cmd, line, tstamp)
    return rsp, cmd, breaks


def build_result(rsp, cmd, breaks, tag, rx_bytes=None):
    # DURABLE CROSS = RSP loaded a WB id AND decoded >= 1 WB-OFDM DATA frame there.
    # If rx_bytes was supplied, ALSO require bytes delivered > 0 end-to-end (gate c).
    cross_phy = (rsp.loaded_wb_id is not None) and (rsp.wb_data_frames >= 1)
    crossed = cross_phy and (rx_bytes is None or rx_bytes > 0)
    return {
        "tag": tag,
        "scorer": "score_climb_canonical (target-keyed N + RSP durable WB-DATA decode + bytes gate)",
        "rsp_configs_seen": sorted(rsp.configs_seen),
        "cmd_configs_seen": sorted(cmd.configs_seen),
        "wb_configs_seen": sorted(c for c in rsp.configs_seen if is_wb_config(c)),
        "rsp_loaded_wb_id": rsp.loaded_wb_id,
        "rsp_wb_data_frames": rsp.wb_data_frames,
        "max_wb_cfg_with_data": rsp.max_wb_cfg_with_data,
        "time_to_cross_s": rsp.first_wb_data_at_s,
        "crossed": crossed,                                  # reached + decoded WB (cfg0..16)
        "climbed_past_cfg0": (rsp.max_wb_cfg_with_data is not None
                              and rsp.max_wb_cfg_with_data >= 1),
        "rx_bytes": rx_bytes,
        "rsp_nreceived_frames": rsp.nreceived,
        "cmd_nreceived_frames": cmd.nreceived,
        "breaks": breaks,
    }


def score_log(log=None, rsp_log=None, cmd_log=None, tag="climb", rx_bytes=None):
    """Public entry point. Returns the canonical result dict."""
    if log:
        rsp, cmd, breaks = score_combined(log)
    elif rsp_log or cmd_log:
        rsp, cmd, breaks = score_two_files(rsp_log, cmd_log)
    else:
        raise ValueError("need log= OR rsp_log=/cmd_log=")
    return build_result(rsp, cmd, breaks, tag, rx_bytes)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", help="combined [T+..] [SIDE] harness log")
    ap.add_argument("--rsp-log", help="RSP-only stderr capture")
    ap.add_argument("--cmd-log", help="CMD-only stderr capture")
    ap.add_argument("--rx-bytes", type=int, default=None,
                    help="end-to-end delivered bytes (gates crossed on >0)")
    ap.add_argument("--json", default=None)
    ap.add_argument("--tag", default="climb")
    args = ap.parse_args()
    if not (args.log or args.rsp_log or args.cmd_log):
        ap.error("need --log OR --rsp-log/--cmd-log")
    result = score_log(args.log, args.rsp_log, args.cmd_log, args.tag, args.rx_bytes)
    print(json.dumps(result, indent=1))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())

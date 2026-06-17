#!/usr/bin/env python3
"""
turnaround_timing_extract.py — reconstruct the recovery-ACK turnaround timeline
from MERCURY_TURNAROUND_TIMING ([TT]) instrumentation.

PURPOSE (owner directive): pin the recovery-ACK turnaround timing mismatch
EMPIRICALLY. Mercury, built with the env-gated MERCURY_TURNAROUND_TIMING
instrumentation (feat/turnaround-timing-instr), emits high-resolution timing
events on stdout of the form:

    [TT] role=<CMD|RSP> <event> mono_us=<N> utc_us=<N> [k=v ...]
    [TT-ANCHOR] role=<CMD|RSP> mono_us=<N> utc_us=<N>

The CMD and RSP modem processes each have their OWN steady_clock epoch, so
mono_us is NOT comparable across processes. utc_us (system_clock microseconds
since the Unix epoch) IS a shared wall clock — this script uses utc_us as the
MASTER timeline and places every event of both processes on it.

IMPORTANT — `role=` is the INSTANTANEOUS half-duplex role, NOT a stable process
id. Mercury flips this->role between COMMANDER/RESPONDER many times per session
(set_role() at dozens of sites). So this script identifies the PROCESS by the
harness line-prefix label ([CMD]/[RSP], written by sim_arq_channel.py's
log_output) when reading a COMBINED log, OR by which file the line came from
when reading SEPARATE per-process logs. The `role=` field is kept only as an
annotation. The EVENT NAME (cmd_break_*, rsp_break_*, rsp_ack_onair_*) is
unambiguous regardless of role.

INPUT (either form):
  * a single COMBINED harness log (sim_arq_channel.log): each line is
      [T+SSSS.mmm] [CMD|RSP] <modem stdout line>
    pass it with --combined LOG.
  * two SEPARATE per-process stdout logs: --cmd CMD.log --rsp RSP.log.

A recovery turnaround is bracketed CMD-side by cmd_break_fire ... (capture
window opens after cmd_break_ptt_off) ... cmd_break_capture_score* ...
{cmd_break_ack_detected | cmd_break_window_expired}, and RSP-side by
rsp_break_rx_decoded ... rsp_break_ack_key ... rsp_ack_onair_start ...
rsp_ack_onair_done (... rsp_break_ack_done). The script groups events into
turnarounds by proximity on the utc_us timeline (a new cmd_break_fire starts a
new group; RSP events join the nearest preceding/following cmd_break_fire).

KEY MEASUREMENT (per turnaround):
  * capture window  = [W_open, W_score_last]  on the CMD timeline, where
        W_open  = utc_us of the FIRST cmd_break_capture_score (elapsed_ms~=0),
        W_close = W_open + (timeout_ms * 1000),  the SCORED window end.
  * ACK on-air      = [A_start, A_end] = [rsp_ack_onair_start, rsp_ack_onair_done].
  * offset_in       = A_start - W_open      (>0 ACK starts after window opens)
  * offset_out      = A_start - W_close     (>0 ACK starts AFTER window closes = MISS)
  * verdict: ACK_IN_WINDOW if A_start < W_close (the correlator could see it),
             ACK_LATE      if A_start >= W_close (lands past the scored window),
             ACK_ABSENT    if no ACK on-air event found for the turnaround.

Across turnarounds: report whether offset_out is CONSTANT (deterministic
geometry bug) or VARIABLE (jitter), with mean / stdev / min / max in ms.

USAGE:
  # HW (the geometry-faithful run — the [TT] clock and cl_timer are BOTH wall-clock):
  python tools/turnaround_timing_extract.py --vehicle hw \
      --cmd cmd_stdout.log --rsp rsp_stdout.log
  python tools/turnaround_timing_extract.py --vehicle hw --combined combined.log

  # Static sim (instrumentation-coverage / event-order ONLY — see CLOCK CAVEAT):
  python tools/turnaround_timing_extract.py --vehicle sim --combined sim2_run.log

  add --json OUT.json to dump the machine-readable structure,
  add --verbose to print every event of every turnaround.

VEHICLES & THE CLOCK CAVEAT (measured 2026-06-17, feat/turnaround-timing-instr):
  * HW (--vehicle hw, DEFAULT): on real hardware cl_timer (which drives the CMD
    recovery capture window receiving_timer) reads CLOCK_MONOTONIC_RAW and the
    [TT] events read steady_clock — BOTH are the real wall clock — so the
    window-vs-ACK offsets below ARE production-faithful. This is the run the
    owner wants for the geometry; capture each modem's stdout to its own file and
    pass --cmd/--rsp.
  * Static SIM (--vehicle sim): the in-process SIM_INPROC 2-instance stepper
    (mercury -m SIM_INPROC, MERCURY_SIM_2INST=1) drives cl_timer from the VIRTUAL
    sim-clock (timer.cc cl_timer_clock_read -> sim_clock_fill_timespec) while the
    [TT] events still read the WALL steady_clock. The two are DECOUPLED, so the
    wall-clock window-vs-ACK offset is NOT meaningful in the sim — only (a) the
    modem's own elapsed_ms/timeout_ms (virtual, self-consistent) and (b) the
    event ORDER are. The sim also CANNOT reach the natural BREAK (3-consecutive-
    block-failure) recovery handshake: the multi-codeword data TX wedges the
    in-process wire first (the documented SIM_INPROC depth-1 turnaround deadlock).
    Use the env-gated synthetic fire to reach the recovery path deterministically:
        MERCURY_SIM_2INST=1 MERCURY_TURNAROUND_TIMING=1 MERCURY_SIM2_FORCE_BREAK=1 \
        MERCURY_SIM2_SNR3K=900 MERCURY_SIM2_CFG=15 MERCURY_SIM2_SEED=12345 \
        MERCURY_SIM2_PAYLOAD_BYTES=200 mercury -m SIM_INPROC > sim2_run.log 2>&1
    That fires the production BREAK recovery sequence once and emits every [TT]
    recovery event, proving instrumentation coverage; trust the geometry numbers
    only from the HW run.
"""
import argparse
import json
import re
import sys

# [TT] role=CMD <event> mono_us=N utc_us=N [k=v ...]
TT_RE = re.compile(
    r"\[TT\]\s+role=(?P<role>\w+)\s+(?P<event>\S+)\s+"
    r"mono_us=(?P<mono>-?\d+)\s+utc_us=(?P<utc>-?\d+)(?P<rest>.*)$")
# [TT-ANCHOR] role=CMD mono_us=N utc_us=N
ANCHOR_RE = re.compile(
    r"\[TT-ANCHOR\]\s+role=(?P<role>\w+)\s+"
    r"mono_us=(?P<mono>-?\d+)\s+utc_us=(?P<utc>-?\d+)")
# harness combined-log line prefix: [T+SSSS.mmm] [CMD] ...
HARNESS_PREFIX_RE = re.compile(r"^\[T\+[\d.]+\]\s+\[(?P<proc>CMD|RSP)\]\s+(?P<body>.*)$")
KV_RE = re.compile(r"(\w+)=(-?[\d.]+)")

# Events and which role/process they belong to.
CMD_EVENTS = {"cmd_break_fire", "cmd_break_audio_start", "cmd_break_audio_done",
              "cmd_break_ptt_off", "cmd_break_capture_score",
              "cmd_break_ack_detected", "cmd_break_window_expired"}
RSP_EVENTS = {"rsp_break_rx_decoded", "rsp_break_ack_key",
              "rsp_ack_onair_start", "rsp_ack_onair_done", "rsp_break_ack_done"}


class Event:
    __slots__ = ("proc", "role", "name", "utc", "mono", "kv", "raw")

    def __init__(self, proc, role, name, utc, mono, kv, raw):
        self.proc = proc      # process id from harness prefix / file ("CMD"/"RSP"/None)
        self.role = role      # instantaneous this->role tag (annotation only)
        self.name = name
        self.utc = utc        # microseconds, shared wall clock (MASTER)
        self.mono = mono      # microseconds, per-process steady_clock (not cross-proc)
        self.kv = kv          # dict of parsed key=value suffix fields
        self.raw = raw


def parse_kv(rest):
    return {k: (float(v) if "." in v else int(v)) for k, v in KV_RE.findall(rest)}


def parse_line(line, proc_hint):
    """Return an Event or None. proc_hint is the harness/file process label."""
    m = HARNESS_PREFIX_RE.match(line)
    proc = proc_hint
    body = line
    if m:
        proc = m.group("proc")
        body = m.group("body")
    a = ANCHOR_RE.search(body)
    if a:
        return Event(proc, a.group("role"), "__anchor__",
                     int(a.group("utc")), int(a.group("mono")), {}, line.rstrip())
    t = TT_RE.search(body)
    if t:
        return Event(proc, t.group("role"), t.group("event"),
                     int(t.group("utc")), int(t.group("mono")),
                     parse_kv(t.group("rest")), line.rstrip())
    return None


def load(combined=None, cmd=None, rsp=None):
    events = []
    if combined:
        with open(combined, "r", errors="replace") as f:
            for line in f:
                e = parse_line(line, None)
                if e:
                    events.append(e)
    if cmd:
        with open(cmd, "r", errors="replace") as f:
            for line in f:
                e = parse_line(line, "CMD")
                if e:
                    events.append(e)
    if rsp:
        with open(rsp, "r", errors="replace") as f:
            for line in f:
                e = parse_line(line, "RSP")
                if e:
                    events.append(e)
    events.sort(key=lambda e: e.utc)
    return events


def group_turnarounds(events):
    """Split events into recovery turnarounds.

    A turnaround opens at a cmd_break_fire and runs to the next cmd_break_fire
    (or end). All other events whose utc falls in that interval are attached.
    This is robust to the role/process-label noise because the bracketing is by
    EVENT NAME + utc proximity.
    """
    fires = [i for i, e in enumerate(events) if e.name == "cmd_break_fire"]
    groups = []
    if not fires:
        return groups
    for gi, start_idx in enumerate(fires):
        end_utc = events[fires[gi + 1]].utc if gi + 1 < len(fires) else float("inf")
        start_utc = events[start_idx].utc
        grp = [e for e in events
               if start_utc <= e.utc < end_utc and e.name != "__anchor__"]
        groups.append(grp)
    return groups


def first(grp, name):
    for e in grp:
        if e.name == name:
            return e
    return None


def last(grp, name):
    found = None
    for e in grp:
        if e.name == name:
            found = e
    return found


def analyze(grp):
    """Compute the capture-window vs ACK-on-air geometry for one turnaround."""
    fire = first(grp, "cmd_break_fire")
    ptt_off = first(grp, "cmd_break_ptt_off")
    scores = [e for e in grp if e.name == "cmd_break_capture_score"]
    detected = first(grp, "cmd_break_ack_detected")
    expired = first(grp, "cmd_break_window_expired")

    rsp_decoded = first(grp, "rsp_break_rx_decoded")
    ack_key = first(grp, "rsp_break_ack_key")
    ack_start = first(grp, "rsp_ack_onair_start")
    ack_done = first(grp, "rsp_ack_onair_done")

    # Capture window geometry.
    w_open = scores[0].utc if scores else None
    timeout_ms = None
    if scores:
        timeout_ms = scores[0].kv.get("timeout_ms")
    if timeout_ms is None and expired:
        timeout_ms = expired.kv.get("timeout_ms")
    w_close = (w_open + int(timeout_ms) * 1000) if (w_open is not None and timeout_ms is not None) else None
    w_score_last = scores[-1].utc if scores else None

    a_start = ack_start.utc if ack_start else None
    a_end = ack_done.utc if ack_done else None

    def ms(us):
        return None if us is None else round(us / 1000.0, 3)

    res = {
        "t_break_fire_us": fire.utc if fire else None,
        "t_cmd_ptt_off_us": ptt_off.utc if ptt_off else None,
        "t_window_open_us": w_open,
        "timeout_ms": int(timeout_ms) if timeout_ms is not None else None,
        "t_window_close_us": w_close,
        "t_last_score_us": w_score_last,
        "n_scores": len(scores),
        "t_rsp_decoded_us": rsp_decoded.utc if rsp_decoded else None,
        "t_rsp_ack_key_us": ack_key.utc if ack_key else None,
        "t_ack_onair_start_us": a_start,
        "t_ack_onair_end_us": a_end,
        "outcome": ("detected" if detected else "expired" if expired else "unknown"),
    }

    # Deltas (ms).
    if rsp_decoded and ack_start:
        res["d_rsp_decode_to_ack_onair_ms"] = ms(a_start - rsp_decoded.utc)
    if ack_key and ack_start:
        res["d_ack_key_to_onair_ms"] = ms(a_start - ack_key.utc)
    if a_start is not None and a_end is not None:
        res["ack_onair_dur_ms"] = ms(a_end - a_start)
    if ptt_off and w_open is not None:
        res["d_cmd_pttoff_to_window_open_ms"] = ms(w_open - ptt_off.utc)
    if fire and rsp_decoded:
        res["d_break_fire_to_rsp_decode_ms"] = ms(rsp_decoded.utc - fire.utc)

    # THE KEY MEASUREMENT: ACK on-air vs CMD capture window.
    if a_start is not None and w_open is not None:
        res["offset_ackstart_minus_winopen_ms"] = ms(a_start - w_open)
    if a_start is not None and w_close is not None:
        res["offset_ackstart_minus_winclose_ms"] = ms(a_start - w_close)
    if a_end is not None and w_close is not None:
        res["offset_ackend_minus_winclose_ms"] = ms(a_end - w_close)

    # Verdict.
    if a_start is None:
        res["verdict"] = "ACK_ABSENT_OR_NOT_LOGGED"
    elif w_close is None:
        res["verdict"] = "WINDOW_UNKNOWN"
    elif a_start < w_close:
        # ACK begins before the window closes -> the correlator's score loop
        # could overlap it. Distinguish fully-in vs straddling the close.
        if a_end is not None and a_end <= w_close:
            res["verdict"] = "ACK_FULLY_IN_WINDOW"
        else:
            res["verdict"] = "ACK_STRADDLES_WINDOW_CLOSE"
    else:
        res["verdict"] = "ACK_LATE_PAST_WINDOW"

    return res


def summarize(analyses):
    """Constant-vs-variable across turnarounds for the headline offsets."""
    def stats(key):
        vals = [a[key] for a in analyses if a.get(key) is not None]
        if not vals:
            return None
        n = len(vals)
        mean = sum(vals) / n
        var = sum((v - mean) ** 2 for v in vals) / n
        sd = var ** 0.5
        return {"n": n, "mean_ms": round(mean, 3), "stdev_ms": round(sd, 3),
                "min_ms": round(min(vals), 3), "max_ms": round(max(vals), 3),
                "spread_ms": round(max(vals) - min(vals), 3)}

    keys = ["offset_ackstart_minus_winopen_ms",
            "offset_ackstart_minus_winclose_ms",
            "offset_ackend_minus_winclose_ms",
            "d_break_fire_to_rsp_decode_ms",
            "d_rsp_decode_to_ack_onair_ms",
            "ack_onair_dur_ms",
            "d_cmd_pttoff_to_window_open_ms",
            "timeout_ms"]
    summary = {k: stats(k) for k in keys}
    verdicts = {}
    for a in analyses:
        verdicts[a["verdict"]] = verdicts.get(a["verdict"], 0) + 1
    summary["verdict_counts"] = verdicts
    # Constant-vs-variable call on the decisive offset.
    s = summary.get("offset_ackstart_minus_winclose_ms")
    if s and s["n"] >= 2:
        summary["offset_constant"] = bool(s["spread_ms"] <= 5.0)  # <=5ms => deterministic
    return summary


def fmt_us(us, ref):
    if us is None:
        return "      --   "
    return f"{(us - ref) / 1000.0:+10.3f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--combined", help="combined harness log (sim_arq_channel.log)")
    ap.add_argument("--cmd", help="CMD-process stdout log (separate-file mode)")
    ap.add_argument("--rsp", help="RSP-process stdout log (separate-file mode)")
    ap.add_argument("--json", help="write machine-readable result here")
    ap.add_argument("--verbose", action="store_true",
                    help="print every event of every turnaround")
    ap.add_argument("--vehicle", choices=("hw", "sim"), default="hw",
                    help="hw (default): the [TT] wall clock AND cl_timer both read the real "
                         "wall clock, so the wall-clock window-vs-ACK offsets below are "
                         "PRODUCTION-FAITHFUL. sim: the in-process SIM_INPROC stepper drives "
                         "cl_timer (the recovery window) from the VIRTUAL sim-clock while the "
                         "[TT] events read the WALL steady_clock -> the wall-clock offsets are "
                         "NOT meaningful; only the modem's own elapsed_ms/timeout_ms (virtual) "
                         "and the EVENT ORDER are. See the script header / report.")
    args = ap.parse_args()

    if args.vehicle == "sim":
        print("!! VEHICLE=sim: cl_timer reads the VIRTUAL sim-clock; [TT] reads the WALL "
              "clock.\n!! The wall-clock window-vs-ACK offsets below are NOT "
              "production-faithful.\n!! Trust only: (a) the modem's own elapsed_ms/timeout_ms "
              "(virtual) and (b) event ORDER.\n!! Run on HW (--vehicle hw) for the real "
              "wall-clock geometry.\n")

    if not (args.combined or args.cmd or args.rsp):
        ap.error("provide --combined LOG or --cmd/--rsp logs")

    events = load(args.combined, args.cmd, args.rsp)
    tt_events = [e for e in events if e.name != "__anchor__"]
    print(f"Loaded {len(events)} [TT]/[TT-ANCHOR] events "
          f"({len(tt_events)} TT events) from log(s).")

    groups = group_turnarounds(events)
    if not groups:
        print("\nNO recovery turnarounds found (no cmd_break_fire events).")
        print("The static vehicle did not produce a BREAK->recovery handshake.")
        # Still report any ACK-on-air events seen (data-ACK turnarounds), which
        # share the same send_ack_pattern() geometry the recovery ACK uses.
        ack_starts = [e for e in tt_events if e.name == "rsp_ack_onair_start"]
        print(f"\n({len(ack_starts)} rsp_ack_onair_start (data-ACK) events present "
              f"-- same keyer geometry as the recovery ACK.)")
        if args.json:
            with open(args.json, "w") as f:
                json.dump({"turnarounds": [], "summary": None,
                           "n_data_ack_onair": len(ack_starts)}, f, indent=2)
        return

    analyses = [analyze(g) for g in groups]
    print(f"\nFound {len(groups)} recovery turnaround(s).\n")

    for i, (g, a) in enumerate(zip(groups, analyses)):
        ref = a["t_break_fire_us"] or (g[0].utc if g else 0)
        print(f"=== Recovery turnaround #{i} (t=0 at cmd_break_fire, ms) ===")
        print(f"  cmd_break_fire        {fmt_us(a['t_break_fire_us'], ref)}")
        print(f"  cmd_break_ptt_off     {fmt_us(a['t_cmd_ptt_off_us'], ref)}")
        print(f"  WINDOW OPEN  (score0) {fmt_us(a['t_window_open_us'], ref)}  "
              f"timeout_ms={a['timeout_ms']}")
        print(f"  rsp_break_rx_decoded  {fmt_us(a['t_rsp_decoded_us'], ref)}")
        print(f"  rsp_break_ack_key     {fmt_us(a['t_rsp_ack_key_us'], ref)}")
        print(f"  ACK ON-AIR start      {fmt_us(a['t_ack_onair_start_us'], ref)}  <== reverse ACK keys")
        print(f"  ACK ON-AIR end        {fmt_us(a['t_ack_onair_end_us'], ref)}")
        print(f"  WINDOW CLOSE (scored) {fmt_us(a['t_window_close_us'], ref)}  "
              f"(open + timeout)")
        print(f"  last capture score    {fmt_us(a['t_last_score_us'], ref)}  "
              f"(n_scores={a['n_scores']})")
        print(f"  outcome={a['outcome']}   VERDICT={a['verdict']}")
        ow = a.get("offset_ackstart_minus_winopen_ms")
        oc = a.get("offset_ackstart_minus_winclose_ms")
        oe = a.get("offset_ackend_minus_winclose_ms")
        print(f"  ACKstart - WINopen  = {ow} ms")
        print(f"  ACKstart - WINclose = {oc} ms   (>0 => ACK starts AFTER scored window)")
        print(f"  ACKend   - WINclose = {oe} ms   (>0 => ACK ends after scored window)")
        if args.verbose:
            print("  --- raw events (utc-ordered) ---")
            for e in g:
                kv = " ".join(f"{k}={v}" for k, v in e.kv.items())
                print(f"    [{e.proc or '?'}|role={e.role}] {e.name:<24} "
                      f"{(e.utc - ref) / 1000.0:+10.3f}ms {kv}")
        print()

    summary = summarize(analyses)
    print("=== SUMMARY across turnarounds ===")
    for k, v in summary.items():
        print(f"  {k}: {v}")

    # Headline.
    s = summary.get("offset_ackstart_minus_winclose_ms")
    print("\n=== HEADLINE ===")
    if s:
        if s["mean_ms"] >= 0:
            print(f"  ACK lands LATE: ACKstart is on average {s['mean_ms']} ms "
                  f"AFTER the scored window closes (spread {s['spread_ms']} ms).")
        else:
            print(f"  ACK lands IN-WINDOW: ACKstart is on average "
                  f"{-s['mean_ms']} ms BEFORE the window closes "
                  f"(spread {s['spread_ms']} ms).")
        const = summary.get("offset_constant")
        if const is True:
            print(f"  Offset is CONSTANT (spread <= 5 ms) => deterministic CODE "
                  f"GEOMETRY, not channel jitter.")
        elif const is False:
            print(f"  Offset is VARIABLE (spread {s['spread_ms']} ms) => jitter "
                  f"dominates the miss.")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"turnarounds": analyses, "summary": summary}, f, indent=2)
        print(f"\nWrote {args.json}")


if __name__ == "__main__":
    main()

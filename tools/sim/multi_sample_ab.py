#!/usr/bin/env python3
"""
multi_sample_ab.py — DISTRIBUTION-based multi-sample A/B harness for the
realtime SIM ARQ channel.

WHY THIS EXISTS
===============
The realtime sim's CONFIG_0 gearshift is FAITHFULLY nondeterministic: the two
`-x sim` mercury peers are independent OS processes whose virtual clocks advance
under host-load-dependent 2-process concurrency, so the gearshift's climb/demote
decisions at a MARGINAL SNR crossing vary run-to-run (this variance is present
under FTRT too, and a real HF link would exhibit it). A SINGLE-sample A/B at a
marginal crossing is therefore a COIN FLIP — one ON vs one OFF run tells you
nothing. (Verified finding, 2026-06-22.)

The fix is METHODOLOGY, not a code change: run N>=8-10 samples per arm with a
VARIED seed per sample, then compare arms by DISTRIBUTION (deliver-rate, median
rx_bytes, config-trajectory modes), and flag whether the per-arm distributions
OVERLAP (inconclusive) or SEPARATE (a real difference). Equivalently, run at a
NON-MARGINAL SNR where the trajectory is deterministic and a small N suffices.

This wraps the single-shot realtime-sim runner `sim_arq_channel.py` (which itself
launches the relay + two `-x sim` peers and emits a per-run JSON with
delivered_bytes / md5_match / switch_seq / bounded_by / wire_bps_airtime). Each
sample is one full single-shot run with seed = base_seed + i. We set
MERCURY_SIM_REALTIME=1 (wall-clock relay pacing) + the arm's env vars in the
CHILD environment; sim_arq_channel.py does `env = dict(os.environ)` for both
peers AND the relay reads MERCURY_SIM_REALTIME from its own env, so a single
parent-env export reaches all three processes.

Samples run one cell at a time (no two sims contend for the CPU — that would
inflate the per-sample spread artificially). The child auto-picks a free port
quad per run and pre-cleans port-scoped, so even a crashed prior sample can't
poison the next.

LOAD-FAIRNESS (the deconfound, added 2026-06-23). The two arms are INTERLEAVED
per sample (sample 0 runs arm A then B, sample 1 runs B then A, ...) so NEITHER
arm is systematically the loaded 2nd in the session — the A/B run-ORDER bias that
the connect-deafness investigation proved was masquerading as an inband bug
(fact-documents/data-flow-inband-connect-deafness-INVALIDATED.md §3,§6). Every
sample also records FTRT = wall_secs/virtual_secs (faster-than-realtime ratio);
a LOAD-STARVED sample (FTRT > --ftrt-fair-max, default 0.85) is one where the host
fell behind realtime, and a marginal acquisition stalls in that sample REGARDLESS
of the feature under test. The headline DECONFOUNDED verdict (compare_loadfair)
keeps ONLY seeds where BOTH arms ran FTRT-fair, so load can't poison the median.
Use --no-interleave to restore the old sequential order (reintroduces the
confound) and --settle-s to tune the inter-cell port/TIME_WAIT drain delay.

USAGE
=====
  python tools/sim/multi_sample_ab.py \
      --arm "ON:MERCURY_INBAND_RATE=1" --arm "OFF:" \
      --cell WGN:12 --profile mpm \
      --target-bytes 65536 --secs 240 --n 10 --base-seed 1000 \
      --bin ./mercury.exe --out _msab/inband_wgn12

  # null-control validation (two identical legacy arms — must OVERLAP):
  python tools/sim/multi_sample_ab.py \
      --arm "A:" --arm "B:" --cell WGN:12 --profile mpm \
      --target-bytes 65536 --secs 180 --n 3 --base-seed 4000 \
      --out _msab/nullctl

ARM SPEC: "LABEL:env-assignments" where env-assignments is a space- or
comma-separated list of NAME=VALUE pairs (empty after the colon = legacy/no
extra env). Examples:
  "ON:MERCURY_INBAND_RATE=1"
  "A3:MERCURY_INBAND_RATE=1,MERCURY_A3_DECOUPLE=1"
  "OFF:"            (legacy — no extra env)

KEY ARGS
========
  --arm SPEC          repeatable; >=2 arms. SPEC = "LABEL:NAME=VAL[,NAME=VAL...]"
  --n N               samples per arm (default 10; >=8 recommended at a marginal
                      crossing; 3-5 ok at a NON-marginal SNR)
  --base-seed S       sample i of every arm uses seed S+i (same seed set per arm
                      => paired comparison, channel realization held across arms)
  --bin PATH          mercury.exe (must carry the [SIM-AUDIO-GUARD] marker)
  --cell WGN:N | --snr DB   channel (passed through to the relay)
  --profile P         wgn / mpg / mpm / mpp
  --target-bytes B    delivery target; a sample DELIVERS if bounded_by=="completion"
                      AND md5_match. Default 65536 (the fixed 64 KiB payload).
  --secs N            per-sample VIRTUAL dwell budget (passed through)
  --deliver-frac F    rx_bytes >= F*target counts toward the "partial" stat
                      (default 0.999 == full). The deliver-RATE uses completion.
  --out DIR           output dir for per-sample JSONs + the aggregate verdict
  --realtime 0|1      set MERCURY_SIM_REALTIME (default 1 == realtime relay pacing)
  --ctrl-base N       pin the child port-quad base for the FIRST sample; each
                      sample advances by --port-stride to avoid TIME_WAIT reuse
                      (default: let the child auto-pick — already collision-proof)

OUTPUT
======
  <out>/sample_<armidx>_<label>_s<seed>.json   one per sample (the child JSON)
  <out>/aggregate.json                          per-arm stats + comparison verdict
  stdout                                        human-readable distribution table

The verdict reports, per arm: deliver-rate (k/N), rx_bytes {min/median/mean/max},
wire_bps_airtime {median}, collapse-rate, stall-rate, and the config-trajectory
modes (how often a sample SUSTAINS a high OFDM config vs stalls at the crossing).
The COMPARISON flags OVERLAP (the rx_bytes ranges intersect AND the deliver-rate
difference is within binomial noise for this N) vs SEPARATE, and prints the
per-arm spread so the user can judge whether N is sufficient.
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

# ── FTRT (faster-than-realtime ratio) ─────────────────────────────────────────
# FTRT = wall_secs / virtual_secs. A run is FTRT-FAIR (sim kept ahead of, or with,
# realtime) when FTRT <= FTRT_FAIR_MAX; a run is LOAD-STARVED (the host couldn't
# pace the realtime relay — wall clock fell behind virtual time) when FTRT exceeds
# it. EVIDENCE (fact-documents/data-flow-inband-connect-deafness-INVALIDATED.md §1,§3):
# a CONNECTED+CLIMBED run measured FTRT 0.71 (real 286 / virtual 405); a stalled
# 2nd-arm run measured ~1.01. The marginal robust connect acquisition lands in the
# decodable window only with faster-than-realtime headroom, so a load-starved sample
# (FTRT~1.0) stalls REGARDLESS of the feature under test — that confound is what this
# gate excludes from the median. 0.85 sits between the 0.71 healthy and 1.01 starved
# observations, with margin. NOTE: lower FTRT is BETTER (more headroom).
FTRT_FAIR_MAX_DEFAULT = 0.85

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

HERE = os.path.dirname(os.path.abspath(__file__))
SINGLE = os.path.join(HERE, "sim_arq_channel.py")
DEFAULT_PAYLOAD = os.path.join(HERE, "payload_incompressible_64k.bin")
# mercury repo root is two levels up from tools/sim/
MERCURY_ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
DEFAULT_BIN = os.path.join(MERCURY_ROOT, "mercury.exe")


# ── arm-spec parsing ──────────────────────────────────────────────────────────
def parse_arm(spec):
    """'LABEL:NAME=VAL,NAME2=VAL2' -> (label, {NAME: VAL, ...}).

    Everything after the FIRST colon is the env list (space- or comma-separated
    NAME=VALUE pairs). An empty env list (e.g. 'OFF:') = legacy / no extra env."""
    if ":" not in spec:
        raise ValueError(
            f"arm spec must be 'LABEL:env' (got {spec!r}); use 'LABEL:' for no env")
    label, _, envpart = spec.partition(":")
    label = label.strip()
    if not label:
        raise ValueError(f"arm spec has empty label: {spec!r}")
    env = {}
    envpart = envpart.strip()
    if envpart:
        tokens = [t for t in envpart.replace(",", " ").split() if t]
        for tok in tokens:
            if "=" not in tok:
                raise ValueError(
                    f"env token {tok!r} in arm {label!r} is not NAME=VALUE")
            k, _, v = tok.partition("=")
            env[k.strip()] = v.strip()
    return label, env


# ── one sample (one single-shot realtime-sim run) ─────────────────────────────
def run_sample(args, label, arm_env, seed, ctrl_base, sample_idx):
    """Run ONE sim_arq_channel.py sample. Returns (parsed_json_or_None, rc, jpath)."""
    jname = f"sample_{sample_idx:02d}_{label}_s{seed}.json"
    jpath = os.path.join(args.out, jname)
    try:
        if os.path.exists(jpath):
            os.remove(jpath)
    except OSError:
        pass

    cmd = [sys.executable, SINGLE,
           "--bin", args.bin,
           "--profile", args.profile,
           "--seed", str(seed),
           "--secs", str(args.secs),
           "--payload", args.payload,
           "--compress", args.compress,
           "--json", jpath]
    if args.cell:
        cmd += ["--cell", args.cell]
    else:
        cmd += ["--snr", str(args.snr)]
    if args.target_bytes > 0:
        cmd += ["--target-bytes", str(args.target_bytes)]
    if args.start_cfg is not None:
        cmd += ["--start-cfg", str(args.start_cfg)]
    if args.profile != "wgn" and args.phase_noise_deg is not None:
        cmd += ["--phase-noise-deg", str(args.phase_noise_deg)]
    if ctrl_base is not None:
        cmd += ["--ctrl-base", str(ctrl_base)]
    # extra passthrough flags verbatim (e.g. --no-gearshift, --barrier-k K)
    cmd += args.passthrough

    # CHILD ENV: realtime pacing + the arm's env. sim_arq_channel.py copies
    # os.environ for BOTH peers, and the relay reads MERCURY_SIM_REALTIME from
    # its own env, so this single dict reaches all three child processes.
    env = dict(os.environ)
    env["MERCURY_SIM_REALTIME"] = str(args.realtime)
    for k, v in arm_env.items():
        if v == "":
            env.pop(k, None)        # explicit unset for the legacy arm
        else:
            env[k] = v

    t0 = time.time()
    log_path = os.path.join(args.out, jname.replace(".json", ".log"))
    with open(log_path, "w", encoding="utf-8", errors="replace") as lf:
        try:
            rc = subprocess.call(cmd, env=env, stdout=lf,
                                 stderr=subprocess.STDOUT,
                                 timeout=args.sample_timeout)
        except subprocess.TimeoutExpired:
            lf.write(f"\n[MSAB] sample TIMED OUT after {args.sample_timeout}s\n")
            rc = -9
    real_s = round(time.time() - t0, 1)

    parsed = None
    if os.path.exists(jpath):
        try:
            with open(jpath) as f:
                parsed = json.load(f)
        except (OSError, ValueError):
            parsed = None
    return parsed, rc, jpath, real_s


# ── per-sample feature extraction ─────────────────────────────────────────────
def _ftrt(wall, virt):
    """FTRT = wall_secs/virtual_secs (lower = more faster-than-realtime headroom).
    None if either is missing or virtual is zero."""
    try:
        if wall is None or virt is None or float(virt) <= 0:
            return None
        return round(float(wall) / float(virt), 4)
    except (TypeError, ValueError):
        return None


def sample_features(j, target_bytes, deliver_frac, ftrt_fair_max=FTRT_FAIR_MAX_DEFAULT):
    """Reduce a child JSON to the comparison features. Robust to a missing JSON
    (a crashed sample => delivered=0, not delivered, treated as a stall).

    Records FTRT (wall/virtual) and ftrt_fair (FTRT <= ftrt_fair_max). A crashed /
    JSON-less sample has FTRT=None and is treated as NOT ftrt-fair (it cannot be
    load-fairness-verified, so it is excluded from the LOAD-FAIR verdict)."""
    if j is None:
        return {
            "ok_json": False, "connected": False, "delivered": False,
            "partial": False, "rx_bytes": 0, "md5_match": False,
            "bounded_by": "no_json", "wire_bps_airtime": None,
            "final_config": None, "collapse": False, "stall": True,
            "sustained_high": False, "virtual_secs": None, "wall_secs": None,
            "ftrt": None, "ftrt_fair": False,
        }
    rx = int(j.get("rx_bytes", j.get("delivered_bytes", 0)) or 0)
    md5 = bool(j.get("md5_match"))
    bounded = j.get("bounded_by")
    # DELIVERED = the run actually completed the byte target with a faithful md5.
    delivered = bool(bounded == "completion" and md5 and
                     (target_bytes <= 0 or rx >= target_bytes))
    # PARTIAL = reached the deliver_frac of the target (even if not "completion").
    partial = bool(target_bytes > 0 and rx >= deliver_frac * target_bytes and md5)
    # STALL = the run did NOT deliver and ended on a wedge / watchdog / death /
    # the explicit deep-stall verdict.
    stall = bool((not delivered) and
                 (bounded in ("vclock_stall", "real_watchdog", "proc_died",
                              "no_json")
                  or j.get("repro_deep_stall")))
    final = j.get("final_config")
    sustained_high = bool(isinstance(final, str) and final.startswith("CONFIG_")
                          and _cfg_num(final) is not None and _cfg_num(final) >= 10)
    wall = j.get("wall_secs")
    virt = j.get("virtual_secs")
    ftrt = _ftrt(wall, virt)
    ftrt_fair = bool(ftrt is not None and ftrt <= ftrt_fair_max)
    return {
        "ok_json": True,
        "connected": bool(j.get("connected")),
        "delivered": delivered,
        "partial": partial,
        "rx_bytes": rx,
        "md5_match": md5,
        "bounded_by": bounded,
        "wire_bps_airtime": j.get("wire_bps_airtime"),
        "final_config": final,
        "collapse": bool(j.get("repro_overclimb_collapse")),
        "stall": stall,
        "sustained_high": sustained_high,
        "virtual_secs": virt,
        "wall_secs": wall,
        "ftrt": ftrt,
        "ftrt_fair": ftrt_fair,
    }


def _cfg_num(name):
    try:
        if name and name.startswith("CONFIG_"):
            return int(name.split("_", 1)[1])
    except (ValueError, IndexError):
        pass
    return None


# ── per-arm aggregation ───────────────────────────────────────────────────────
def _stats(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return {"n": 0, "min": None, "median": None, "mean": None, "max": None,
                "stdev": None}
    return {
        "n": len(vals),
        "min": round(min(vals), 1),
        "median": round(statistics.median(vals), 1),
        "mean": round(statistics.mean(vals), 1),
        "max": round(max(vals), 1),
        "stdev": round(statistics.pstdev(vals), 1) if len(vals) > 1 else 0.0,
    }


def aggregate_arm(label, env, feats):
    n = len(feats)
    delivered = sum(1 for f in feats if f["delivered"])
    partial = sum(1 for f in feats if f["partial"])
    stalled = sum(1 for f in feats if f["stall"])
    collapsed = sum(1 for f in feats if f["collapse"])
    connected = sum(1 for f in feats if f["connected"])
    sustained = sum(1 for f in feats if f["sustained_high"])
    rx_stats = _stats([f["rx_bytes"] for f in feats])
    wire_stats = _stats([f["wire_bps_airtime"] for f in feats])
    # FTRT distribution across the N samples + the load-starved count.
    ftrt_stats = _stats([f.get("ftrt") for f in feats])
    fair = [f for f in feats if f.get("ftrt_fair")]
    n_fair = len(fair)
    n_starved = sum(1 for f in feats if f.get("ftrt") is not None
                    and not f.get("ftrt_fair"))
    # FTRT-FAIR-only headline stats (load-starved samples excluded from the median).
    rx_fair = _stats([f["rx_bytes"] for f in fair])
    deliv_fair = sum(1 for f in fair if f["delivered"])
    # config-trajectory modes: histogram of final_config across the N samples.
    from collections import Counter
    final_modes = Counter(f["final_config"] for f in feats)
    bounded_modes = Counter(f["bounded_by"] for f in feats)
    final_modes_fair = Counter(f["final_config"] for f in fair)
    return {
        "label": label, "env": env, "n": n,
        "deliver_rate": round(delivered / n, 3) if n else None,
        "delivered": delivered, "partial": partial,
        "connect_rate": round(connected / n, 3) if n else None,
        "stall_rate": round(stalled / n, 3) if n else None,
        "collapse_rate": round(collapsed / n, 3) if n else None,
        "sustained_high_rate": round(sustained / n, 3) if n else None,
        "rx_bytes": rx_stats,
        "wire_bps_airtime": wire_stats,
        "final_config_modes": dict(final_modes),
        "bounded_by_modes": dict(bounded_modes),
        # ── FTRT / load-fairness block ──
        "ftrt": ftrt_stats,
        "n_ftrt_fair": n_fair,
        "n_ftrt_starved": n_starved,
        "ftrt_fair_rate": round(n_fair / n, 3) if n else None,
        "rx_bytes_fair": rx_fair,
        "deliver_rate_fair": round(deliv_fair / n_fair, 3) if n_fair else None,
        "delivered_fair": deliv_fair,
        "final_config_modes_fair": dict(final_modes_fair),
        "_feats": feats,
    }


# ── two-arm comparison verdict ────────────────────────────────────────────────
def compare(a, b, n):
    """Distribution comparison of arms a vs b. OVERLAP => inconclusive at this N."""
    ar, br = a["rx_bytes"], b["rx_bytes"]
    # rx_bytes range overlap (min..max). If the ranges intersect, the
    # distributions overlap on the headline metric.
    rx_overlap = None
    if ar["min"] is not None and br["min"] is not None:
        rx_overlap = not (ar["max"] < br["min"] or br["max"] < ar["min"])
    # deliver-rate gap in percentage points + a crude binomial-noise band.
    da, db = a["deliver_rate"], b["deliver_rate"]
    dr_gap_pp = None
    dr_noise_pp = None
    dr_separated = None
    if da is not None and db is not None and n > 0:
        dr_gap_pp = round((da - db) * 100, 1)
        # 1-sigma binomial SE for a rate p over n, summed in quadrature for two
        # arms, ~doubled to ~2-sigma. A gap inside this band is plausibly noise.
        import math
        se_a = math.sqrt(max(da * (1 - da), 1e-9) / n)
        se_b = math.sqrt(max(db * (1 - db), 1e-9) / n)
        dr_noise_pp = round(2 * math.sqrt(se_a ** 2 + se_b ** 2) * 100, 1)
        dr_separated = abs(dr_gap_pp) > dr_noise_pp
    # median rx delta (relative).
    med_delta_pct = None
    if ar["median"] and br["median"] and br["median"] != 0:
        med_delta_pct = round((ar["median"] - br["median"]) / br["median"] * 100, 1)
    # OVERALL: SEPARATE only if rx ranges do NOT overlap OR the deliver-rate gap
    # clears the binomial-noise band. Else OVERLAP (inconclusive at this N).
    separate = bool((rx_overlap is False) or (dr_separated is True))
    return {
        "arm_a": a["label"], "arm_b": b["label"],
        "n_per_arm": n,
        "deliver_rate_a": da, "deliver_rate_b": db,
        "deliver_rate_gap_pp": dr_gap_pp,
        "deliver_rate_noise_band_pp": dr_noise_pp,
        "deliver_rate_separated": dr_separated,
        "rx_median_a": ar["median"], "rx_median_b": br["median"],
        "rx_median_delta_pct": med_delta_pct,
        "rx_range_a": [ar["min"], ar["max"]], "rx_range_b": [br["min"], br["max"]],
        "rx_ranges_overlap": rx_overlap,
        "verdict": "SEPARATE (a real difference)" if separate
                   else "OVERLAP (inconclusive at this N)",
        "separate": separate,
    }


def compare_loadfair(a, b):
    """LOAD-FAIR paired comparison: keep ONLY seeds where BOTH arms ran FTRT-fair,
    then compare those paired samples. This is the deconfounded verdict — it
    removes the A/B run-order / host-load confound (a load-starved arm stalls
    regardless of feature). Pairs by sample index (same seed set per arm).
    Returns a dict; n_pairs==0 => NO_FAIR_PAIRS (machine too loaded to trust)."""
    fa, fb = a["_feats"], b["_feats"]
    pairs = [(x, y) for x, y in zip(fa, fb)
             if x.get("ftrt_fair") and y.get("ftrt_fair")]
    n_pairs = len(pairs)
    if n_pairs == 0:
        return {
            "arm_a": a["label"], "arm_b": b["label"], "n_fair_pairs": 0,
            "verdict": "NO_FAIR_PAIRS (machine load-starved; A/B not trustworthy "
                       "— need FTRT<=fair-max headroom on BOTH arms)",
            "trustworthy": False,
        }
    rxa = [x["rx_bytes"] for x, _ in pairs]
    rxb = [y["rx_bytes"] for _, y in pairs]
    sa, sb = _stats(rxa), _stats(rxb)
    deliv_a = sum(1 for x, _ in pairs if x["delivered"])
    deliv_b = sum(1 for _, y in pairs if y["delivered"])
    from collections import Counter
    cfg_a = Counter(x["final_config"] for x, _ in pairs)
    cfg_b = Counter(y["final_config"] for _, y in pairs)
    med_delta_pct = None
    if sa["median"] and sb["median"] and sb["median"] != 0:
        med_delta_pct = round((sa["median"] - sb["median"]) / sb["median"] * 100, 1)
    return {
        "arm_a": a["label"], "arm_b": b["label"], "n_fair_pairs": n_pairs,
        "rx_median_a": sa["median"], "rx_median_b": sb["median"],
        "rx_median_delta_pct": med_delta_pct,
        "rx_range_a": [sa["min"], sa["max"]], "rx_range_b": [sb["min"], sb["max"]],
        "deliver_a": deliv_a, "deliver_b": deliv_b,
        "deliver_rate_a": round(deliv_a / n_pairs, 3),
        "deliver_rate_b": round(deliv_b / n_pairs, 3),
        "final_config_modes_a": dict(cfg_a),
        "final_config_modes_b": dict(cfg_b),
        "verdict": ("LOAD-FAIR: compared %d FTRT-fair paired sample(s)" % n_pairs),
        "trustworthy": True,
    }


def _synth_feat(rx, delivered, bounded="completion", connected=True,
                collapse=False, stall=False, final="CONFIG_15", wire=3300.0,
                ftrt=0.75):
    """Build a synthetic per-sample feature dict (selftest helper). Default
    ftrt=0.75 (FTRT-fair); pass ftrt>fair-max to synthesize a load-starved sample."""
    virt = 120.0
    wall = round(virt * ftrt, 1) if ftrt is not None else None
    return {
        "ok_json": True, "connected": connected, "delivered": delivered,
        "partial": delivered, "rx_bytes": rx, "md5_match": delivered,
        "bounded_by": bounded, "wire_bps_airtime": wire, "final_config": final,
        "collapse": collapse, "stall": stall,
        "sustained_high": (isinstance(final, str) and final.startswith("CONFIG_")
                           and (_cfg_num(final) or 0) >= 10),
        "virtual_secs": virt, "wall_secs": wall,
        "ftrt": ftrt, "ftrt_fair": bool(ftrt is not None
                                        and ftrt <= FTRT_FAIR_MAX_DEFAULT),
    }


def interleave_schedule(n_arms, n_samples):
    """Build the (sample_idx, arm_idx) execution ORDER that interleaves arms so
    neither arm is systematically the loaded 2nd in a session. For each sample
    index i we run all arms, but ROTATE the arm order by i (a Latin-square-ish
    rotation): sample 0 = arms [0,1,..], sample 1 = [1,..,0], etc. This balances
    "who runs after whom" across samples, deconfounding A/B run-order from arm-type.
    Returns a flat list of (sample_idx, arm_idx) in execution order."""
    order = []
    for i in range(n_samples):
        rot = i % n_arms
        arm_seq = [(rot + k) % n_arms for k in range(n_arms)]
        for arm_idx in arm_seq:
            order.append((i, arm_idx))
    return order


def selftest():
    """Exercise aggregate_arm + compare on SYNTHETIC data WITH spread (no sim).
    Proves the stats + OVERLAP/SEPARATE logic on a non-degenerate distribution,
    PLUS the FTRT record/gate + interleave-order machinery (the deconfound fix).
    Returns 0 on success, 1 on any assertion failure."""
    ok = True

    def check(name, cond):
        nonlocal ok
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")
        ok = ok and bool(cond)

    # 1) OVERLAP case: two arms with overlapping rx spread + equal deliver-rate.
    n = 10
    feats_a = ([_synth_feat(r, True) for r in (4000, 4200, 4100, 4300, 3900,
                                               4150, 4250, 4050)]
               + [_synth_feat(0, False, bounded="vclock_stall", stall=True,
                              final=None, wire=None)] * 2)
    feats_b = ([_synth_feat(r, True) for r in (3800, 4100, 3950, 4050, 4200,
                                               4000, 3850, 4150)]
               + [_synth_feat(0, False, bounded="vclock_stall", stall=True,
                              final=None, wire=None)] * 2)
    a = aggregate_arm("A", {}, feats_a)
    b = aggregate_arm("B", {}, feats_b)
    check("OVERLAP: deliver_rate A=0.8", a["deliver_rate"] == 0.8)
    check("OVERLAP: deliver_rate B=0.8", b["deliver_rate"] == 0.8)
    check("OVERLAP: A rx median computed", a["rx_bytes"]["median"] is not None)
    check("OVERLAP: stall_rate A=0.2", a["stall_rate"] == 0.2)
    c = compare(a, b, n)
    check("OVERLAP: rx ranges overlap True", c["rx_ranges_overlap"] is True)
    check("OVERLAP: verdict OVERLAP", c["separate"] is False)

    # 2) SEPARATE case: arm A fully delivers high rx, arm B fully stalls.
    feats_hi = [_synth_feat(r, True) for r in
                (60000, 61000, 59000, 62000, 60500, 61500, 59500, 60800,
                 61200, 60200)]
    feats_lo = [_synth_feat(0, False, bounded="vclock_stall", stall=True,
                            final=None, wire=None) for _ in range(10)]
    hi = aggregate_arm("HI", {}, feats_hi)
    lo = aggregate_arm("LO", {}, feats_lo)
    check("SEPARATE: HI deliver_rate 1.0", hi["deliver_rate"] == 1.0)
    check("SEPARATE: LO deliver_rate 0.0", lo["deliver_rate"] == 0.0)
    c2 = compare(hi, lo, 10)
    check("SEPARATE: rx ranges DO NOT overlap", c2["rx_ranges_overlap"] is False)
    check("SEPARATE: deliver-rate clears noise band",
          c2["deliver_rate_separated"] is True)
    check("SEPARATE: verdict SEPARATE", c2["separate"] is True)

    # 3) arm-spec parsing
    lbl, env = parse_arm("ON:MERCURY_INBAND_RATE=1,MERCURY_A3_DECOUPLE=1")
    check("parse_arm label", lbl == "ON")
    check("parse_arm env", env == {"MERCURY_INBAND_RATE": "1",
                                   "MERCURY_A3_DECOUPLE": "1"})
    lbl2, env2 = parse_arm("OFF:")
    check("parse_arm legacy empty env", lbl2 == "OFF" and env2 == {})

    # 4) stats sanity on a known vector
    s = _stats([10, 20, 30])
    check("stats median", s["median"] == 20)
    check("stats mean", s["mean"] == 20)
    check("stats min/max", s["min"] == 10 and s["max"] == 30)

    # 5) FTRT RECORD: sample_features computes ftrt = wall/virtual and ftrt_fair.
    f_fair = sample_features({"rx_bytes": 60000, "md5_match": True,
                              "bounded_by": "completion", "final_config": "CONFIG_16",
                              "wall_secs": 286.0, "virtual_secs": 405.0,
                              "connected": True}, 65536, 0.999)
    check("FTRT record: ftrt computed (0.71 healthy)",
          f_fair["ftrt"] is not None and abs(f_fair["ftrt"] - 0.7062) < 0.01)
    check("FTRT record: 0.71 is ftrt_fair=True", f_fair["ftrt_fair"] is True)
    f_starved = sample_features({"rx_bytes": 0, "md5_match": False,
                                 "bounded_by": "vclock_stall", "final_config": None,
                                 "wall_secs": 410.0, "virtual_secs": 405.0,
                                 "connected": True}, 65536, 0.999)
    check("FTRT record: ftrt~1.01 starved", f_starved["ftrt"] is not None
          and f_starved["ftrt"] > 1.0)
    check("FTRT gate: ftrt~1.01 is ftrt_fair=False",
          f_starved["ftrt_fair"] is False)
    f_nojson = sample_features(None, 65536, 0.999)
    check("FTRT record: missing-JSON ftrt=None, NOT fair",
          f_nojson["ftrt"] is None and f_nojson["ftrt_fair"] is False)

    # 6) FTRT GATE: aggregate + load-fair compare EXCLUDE the load-starved sample.
    #    Arm HI has 4 fair deliveries + 1 starved stall; arm LO has 4 fair stalls
    #    + 1 starved stall. Paired load-fair compare must use only the 4 fair pairs
    #    and must EXCLUDE the starved index entirely.
    hi_feats = ([_synth_feat(r, True, final="CONFIG_16", ftrt=0.7)
                 for r in (60000, 61000, 59000, 62000)]
                + [_synth_feat(0, False, bounded="vclock_stall", stall=True,
                               final=None, wire=None, ftrt=1.05)])  # starved
    lo_feats = ([_synth_feat(0, False, bounded="vclock_stall", stall=True,
                             final="CONFIG_0", wire=None, ftrt=0.7)
                 for _ in range(4)]
                + [_synth_feat(0, False, bounded="vclock_stall", stall=True,
                               final=None, wire=None, ftrt=1.05)])  # starved
    hi = aggregate_arm("HI", {}, hi_feats)
    lo = aggregate_arm("LO", {}, lo_feats)
    check("GATE: HI n_ftrt_fair=4 (1 starved excluded)", hi["n_ftrt_fair"] == 4)
    check("GATE: HI n_ftrt_starved=1", hi["n_ftrt_starved"] == 1)
    check("GATE: HI fair rx median > naive median (starved 0 dropped)",
          hi["rx_bytes_fair"]["median"] > hi["rx_bytes"]["median"])
    check("GATE: HI deliver_rate_fair=1.0 (all 4 fair delivered)",
          hi["deliver_rate_fair"] == 1.0)
    lf = compare_loadfair(hi, lo)
    check("LOAD-FAIR: n_fair_pairs=4 (starved pair excluded)",
          lf["n_fair_pairs"] == 4)
    check("LOAD-FAIR: HI fair deliver=4, LO fair deliver=0",
          lf["deliver_a"] == 4 and lf["deliver_b"] == 0)
    check("LOAD-FAIR: trustworthy True", lf["trustworthy"] is True)

    # 7) NO_FAIR_PAIRS: every sample load-starved => verdict refuses to compare.
    starved_a = [_synth_feat(0, False, bounded="vclock_stall", stall=True,
                             final=None, wire=None, ftrt=1.02) for _ in range(3)]
    starved_b = [_synth_feat(0, False, bounded="vclock_stall", stall=True,
                             final=None, wire=None, ftrt=1.02) for _ in range(3)]
    sa_agg = aggregate_arm("SA", {}, starved_a)
    sb_agg = aggregate_arm("SB", {}, starved_b)
    lf2 = compare_loadfair(sa_agg, sb_agg)
    check("NO_FAIR_PAIRS: all-starved => n_fair_pairs=0", lf2["n_fair_pairs"] == 0)
    check("NO_FAIR_PAIRS: trustworthy False", lf2["trustworthy"] is False)

    # 8) INTERLEAVE ORDER: arms alternate who-runs-first across samples; balanced.
    sched = interleave_schedule(2, 4)
    #   per sample, BOTH arms run; arm that runs FIRST rotates each sample.
    check("INTERLEAVE: 2 arms x 4 samples => 8 cells", len(sched) == 8)
    check("INTERLEAVE: every (sample,arm) pair present once",
          sorted(sched) == sorted([(i, a) for i in range(4) for a in range(2)]))
    #   arm running FIRST per sample alternates 0,1,0,1 (no arm always 2nd)
    first_per_sample = [sched[2 * i][1] for i in range(4)]
    check("INTERLEAVE: first-arm rotates 0,1,0,1 (no systematic 2nd-arm)",
          first_per_sample == [0, 1, 0, 1])
    #   3-arm rotation sanity
    sched3 = interleave_schedule(3, 3)
    check("INTERLEAVE: 3 arms x 3 samples => 9 cells", len(sched3) == 9)
    first3 = [sched3[3 * i][1] for i in range(3)]
    check("INTERLEAVE: 3-arm first rotates 0,1,2", first3 == [0, 1, 2])

    print(f"\nSELFTEST {'OK' if ok else 'FAILED'}")
    return 0 if ok else 1


def fmt_arm(a):
    rx, wire, ft, rxf = (a["rx_bytes"], a["wire_bps_airtime"], a["ftrt"],
                         a["rx_bytes_fair"])
    return (f"  ARM {a['label']:<8} env={a['env'] or '{}'}\n"
            f"    deliver-rate : {a['delivered']}/{a['n']} "
            f"(={a['deliver_rate']})  partial={a['partial']}  "
            f"connect={a['connect_rate']}\n"
            f"    rx_bytes     : min={rx['min']} median={rx['median']} "
            f"mean={rx['mean']} max={rx['max']} stdev={rx['stdev']}\n"
            f"    wire_bps_air : median={wire['median']} "
            f"(min={wire['min']} max={wire['max']})\n"
            f"    FTRT(w/v)    : median={ft['median']} (min={ft['min']} "
            f"max={ft['max']})  fair={a['n_ftrt_fair']}/{a['n']} "
            f"starved={a['n_ftrt_starved']}\n"
            f"    FAIR-ONLY    : rx_median={rxf['median']} (n={rxf['n']})  "
            f"deliver_rate_fair={a['deliver_rate_fair']}  "
            f"final_cfg_fair={a['final_config_modes_fair']}\n"
            f"    stall-rate   : {a['stall_rate']}   collapse-rate: "
            f"{a['collapse_rate']}   sustained-high: {a['sustained_high_rate']}\n"
            f"    final_cfg    : {a['final_config_modes']}\n"
            f"    bounded_by   : {a['bounded_by_modes']}")


def main():
    ap = argparse.ArgumentParser(
        description="Multi-sample distribution-based A/B for the realtime SIM "
                    "ARQ channel.")
    ap.add_argument("--arm", action="append", default=[], metavar="LABEL:env",
                    help="repeatable arm spec 'LABEL:NAME=VAL[,NAME=VAL]'. "
                         "'LABEL:' = legacy/no extra env. Need >=2 arms.")
    ap.add_argument("--n", type=int, default=10,
                    help="samples per arm (>=8 at a marginal crossing).")
    ap.add_argument("--base-seed", type=int, default=1000,
                    help="sample i uses seed base+i (SAME set per arm = paired).")
    ap.add_argument("--bin", default=DEFAULT_BIN)
    ap.add_argument("--cell", default=None, help="e.g. WGN:12 (SNR3k=label+2.4)")
    ap.add_argument("--snr", type=float, default=12.0,
                    help="channel SNR3k dB (used when --cell absent)")
    ap.add_argument("--profile", default="wgn", help="wgn/mpg/mpm/mpp")
    ap.add_argument("--phase-noise-deg", type=float, default=0.2)
    ap.add_argument("--target-bytes", type=int, default=65536,
                    help="delivery target; completion+md5 => DELIVERED. "
                         "Default 65536 (the fixed 64 KiB payload).")
    ap.add_argument("--deliver-frac", type=float, default=0.999,
                    help="rx>=frac*target counts as 'partial' (default full).")
    ap.add_argument("--secs", type=int, default=240,
                    help="per-sample VIRTUAL dwell budget.")
    ap.add_argument("--start-cfg", type=int, default=None,
                    help="passthrough mercury -s start config (default child's).")
    ap.add_argument("--compress", default="off")
    ap.add_argument("--payload", default=DEFAULT_PAYLOAD)
    ap.add_argument("--realtime", type=int, default=1, choices=(0, 1),
                    help="MERCURY_SIM_REALTIME (1 = wall-clock relay pacing).")
    ap.add_argument("--ctrl-base", type=int, default=None,
                    help="pin the FIRST sample's child port-quad base; each "
                         "sample advances by --port-stride. Default: child "
                         "auto-picks (already collision-proof).")
    ap.add_argument("--port-stride", type=int, default=20,
                    help="port-quad advance between samples when --ctrl-base set.")
    ap.add_argument("--sample-timeout", type=int, default=1200,
                    help="hard real-time ceiling per sample (s).")
    ap.add_argument("--ftrt-fair-max", type=float, default=FTRT_FAIR_MAX_DEFAULT,
                    help="FTRT (wall/virtual) at/above which a sample is LOAD-STARVED "
                         "and excluded from the load-fair median (default %.2f; lower "
                         "FTRT = more faster-than-realtime headroom)."
                         % FTRT_FAIR_MAX_DEFAULT)
    ap.add_argument("--no-interleave", action="store_true",
                    help="run all of arm A then all of arm B (LEGACY order; "
                         "reintroduces the A/B run-order confound). Default OFF: "
                         "arms are INTERLEAVED so neither is systematically the 2nd.")
    ap.add_argument("--settle-s", type=float, default=3.0,
                    help="settle delay between samples to let ports/TIME_WAIT drain "
                         "(the child already port-scoped-cleans its own PIDs; this is "
                         "NOT a system-wide kill). Default 3.0s.")
    ap.add_argument("--out", default=None,
                    help="output dir (default _msab/<timestamp>).")
    ap.add_argument("--selftest", action="store_true",
                    help="run the synthetic aggregation/comparison self-test "
                         "(no sim) and exit. Proves the stats + OVERLAP/SEPARATE "
                         "logic on a non-degenerate distribution.")
    ap.add_argument("passthrough", nargs="*",
                    help="extra flags forwarded verbatim to sim_arq_channel.py "
                         "(e.g. --no-gearshift --barrier-k 4).")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if len(args.arm) < 2:
        ap.error("need at least two --arm specs (e.g. --arm 'ON:MERCURY_INBAND_RATE=1' "
                 "--arm 'OFF:')")
    try:
        arms = [parse_arm(s) for s in args.arm]
    except ValueError as e:
        ap.error(str(e))

    if args.out is None:
        args.out = os.path.join(MERCURY_ROOT, "_msab",
                                time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(args.out, exist_ok=True)

    channel = args.cell if args.cell else f"SNR3k={args.snr}dB"
    print("=" * 72)
    print("MULTI-SAMPLE A/B  (realtime SIM, distribution comparison)")
    print(f"  arms      : {[lbl for lbl, _ in arms]}")
    print(f"  channel   : {channel} profile={args.profile}")
    print(f"  target    : {args.target_bytes}B  secs={args.secs}  "
          f"realtime={args.realtime}")
    print(f"  N/arm     : {args.n}   seeds {args.base_seed}..{args.base_seed+args.n-1}")
    print(f"  bin       : {args.bin}")
    print(f"  out       : {args.out}")
    interleaved = not args.no_interleave
    print(f"  order     : {'INTERLEAVED (load-fair)' if interleaved else 'SEQUENTIAL (legacy, run-order confound)'}"
          f"  ftrt_fair_max={args.ftrt_fair_max}  settle={args.settle_s}s")
    print("=" * 72, flush=True)

    started = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    agg = {
        "started_utc": started,
        "channel": channel, "profile": args.profile,
        "target_bytes": args.target_bytes, "secs": args.secs,
        "realtime": args.realtime, "n_per_arm": args.n,
        "base_seed": args.base_seed, "bin": args.bin,
        "interleaved": interleaved, "ftrt_fair_max": args.ftrt_fair_max,
        "arms": [], "comparisons": [], "comparisons_loadfair": [],
        "note": ("Realtime-sim CONFIG_0 gearshift is faithfully nondeterministic "
                 "(2-process concurrency); single samples at a marginal crossing "
                 "are coin flips. Compare by DISTRIBUTION. OVERLAP=inconclusive. "
                 "FTRT=wall/virtual: a LOAD-STARVED sample (FTRT>ftrt_fair_max) "
                 "stalls regardless of feature (A/B run-order confound) and is "
                 "excluded from the LOAD-FAIR verdict — the deconfounded comparison."),
    }

    # ── EXECUTION ORDER: interleave arms per sample so neither is systematically
    #    the loaded 2nd; clean state (port-scoped — child does its own teardown)
    #    + settle between cells. Legacy --no-interleave runs all of A then all of B.
    if interleaved:
        schedule = interleave_schedule(len(arms), args.n)
    else:
        schedule = [(i, arm_idx) for arm_idx in range(len(arms))
                    for i in range(args.n)]

    # per-arm feature lists, indexed by sample so paired (same-seed) comparison holds
    feats_by_arm = [[None] * args.n for _ in arms]

    def _flush_agg():
        arm_results_now = []
        for arm_idx, (label, env) in enumerate(arms):
            done = [f for f in feats_by_arm[arm_idx] if f is not None]
            arm_results_now.append(aggregate_arm(label, env, done))
        agg["arms"] = [{k: v for k, v in r.items() if k != "_feats"}
                       for r in arm_results_now]
        agg["per_sample"] = {r["label"]: r["_feats"] for r in arm_results_now}
        with open(os.path.join(args.out, "aggregate.json"), "w") as fjs:
            json.dump(agg, fjs, indent=2, default=str)
        return arm_results_now

    for cell_n, (i, arm_idx) in enumerate(schedule):
        label, env = arms[arm_idx]
        seed = args.base_seed + i
        ctrl_base = (None if args.ctrl_base is None
                     else args.ctrl_base + i * args.port_stride)
        print(f"  [{cell_n+1}/{len(schedule)}] arm={label} sample={i+1}/{args.n} "
              f"seed={seed} ...", end="", flush=True)
        j, rc, jpath, real_s = run_sample(args, label, env, seed, ctrl_base, i)
        f = sample_features(j, args.target_bytes, args.deliver_frac,
                            ftrt_fair_max=args.ftrt_fair_max)
        feats_by_arm[arm_idx][i] = f
        print(f" rc={rc} {real_s}s  rx={f['rx_bytes']}B "
              f"delivered={f['delivered']} bounded={f['bounded_by']} "
              f"final={f['final_config']} FTRT={f['ftrt']} "
              f"{'FAIR' if f['ftrt_fair'] else 'STARVED'}", flush=True)
        _flush_agg()
        # settle between cells: the child already port-scoped-killed its own PIDs
        # at teardown (sim_arq_channel.py kill_port_scoped) — NEVER a system-wide
        # /IM mercury.exe sweep (would corrupt a concurrent bench run). Just give
        # ports/TIME_WAIT a moment to drain so the next cell starts clean.
        if cell_n + 1 < len(schedule) and args.settle_s > 0:
            time.sleep(args.settle_s)

    arm_results = _flush_agg()

    # ── comparisons (arm0 vs each other arm): full (legacy) + LOAD-FAIR (deconf) ──
    comps, comps_lf = [], []
    for j in range(1, len(arm_results)):
        comps.append(compare(arm_results[0], arm_results[j], args.n))
        comps_lf.append(compare_loadfair(arm_results[0], arm_results[j]))
    agg["comparisons"] = comps
    agg["comparisons_loadfair"] = comps_lf
    agg["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    with open(os.path.join(args.out, "aggregate.json"), "w") as f:
        json.dump(agg, f, indent=2, default=str)

    # ── human-readable verdict ──
    print("\n" + "=" * 72)
    print("DISTRIBUTION SUMMARY")
    print("=" * 72)
    for a in arm_results:
        print(fmt_arm(a))
        print()
    print("-" * 72)
    print("COMPARISON (full distribution — NOT load-deconfounded)")
    print("-" * 72)
    for c in comps:
        print(f"  {c['arm_a']} vs {c['arm_b']}  (N={c['n_per_arm']}/arm)")
        print(f"    deliver-rate : {c['arm_a']}={c['deliver_rate_a']}  "
              f"{c['arm_b']}={c['deliver_rate_b']}  "
              f"gap={c['deliver_rate_gap_pp']}pp  "
              f"(binomial-noise band ~{c['deliver_rate_noise_band_pp']}pp -> "
              f"{'SEPARATED' if c['deliver_rate_separated'] else 'within noise'})")
        print(f"    rx_median    : {c['arm_a']}={c['rx_median_a']}  "
              f"{c['arm_b']}={c['rx_median_b']}  "
              f"delta={c['rx_median_delta_pct']}%")
        print(f"    rx_ranges    : {c['arm_a']}={c['rx_range_a']}  "
              f"{c['arm_b']}={c['rx_range_b']}  "
              f"overlap={c['rx_ranges_overlap']}")
        print(f"    VERDICT      : {c['verdict']}")
        print()
    print("-" * 72)
    print("LOAD-FAIR COMPARISON (FTRT-fair paired samples ONLY — DECONFOUNDED)")
    print("-" * 72)
    for c in comps_lf:
        print(f"  {c['arm_a']} vs {c['arm_b']}  (fair pairs={c['n_fair_pairs']})")
        if not c.get("trustworthy"):
            print(f"    VERDICT      : {c['verdict']}")
            print()
            continue
        print(f"    deliver-rate : {c['arm_a']}={c['deliver_rate_a']}  "
              f"{c['arm_b']}={c['deliver_rate_b']}  "
              f"({c['deliver_a']}/{c['n_fair_pairs']} vs "
              f"{c['deliver_b']}/{c['n_fair_pairs']})")
        print(f"    rx_median    : {c['arm_a']}={c['rx_median_a']}  "
              f"{c['arm_b']}={c['rx_median_b']}  "
              f"delta={c['rx_median_delta_pct']}%")
        print(f"    final_cfg    : {c['arm_a']}={c['final_config_modes_a']}  "
              f"{c['arm_b']}={c['final_config_modes_b']}")
        print(f"    VERDICT      : {c['verdict']}")
        print()
    print(f"wrote {os.path.join(args.out, 'aggregate.json')}")
    print("Guidance: trust the LOAD-FAIR comparison (FTRT-fair pairs). If "
          "n_fair_pairs is small/0 the machine was load-starved — re-run with more "
          "host headroom or a lower load, NOT a different conclusion.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
parallel_spawner.py - launch N concurrent real-audio Mercury round-trips over
snd-aloop and prove CORRECTNESS AT SPEED. (CAPSTONE-extended cohort driver.)

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

===  CAPSTONE INTEGRATION EXTENSIONS (measurement infra; NO mercury source)  ===
  (a) PER-LEVER KILL-SWITCHES: --arm-name / --arm-spec select the lever set for
      THIS cohort (applied to every cell's BOTH mercury via arq_realaudio). One
      cohort = one arm. PAIRED-SEED across arms: run this spawner once per arm
      with the SAME --n (per-run seed = run index+1 is identical across arms) and
      DISJOINT --card-base/--port-base/--tag-prefix.
  (b) BYTE-INTEGRITY: every cell hard-verifies its delivered payload; the cohort
      summary counts integrity failures and RAISES a loud INTEGRITY_FAILURES flag
      (silent false-accept is the one thing that must never pass).
  (c) ANATOMY line-items are the PRIMARY scoreboard: per-cell + cohort
      median/spread of listen-guard ms, retx rounds, demote/BREAK, active_frac,
      delivered USER-bytes/min.
  (d) VARA-BAR column at this cohort's SNR3k.

Also writes <out>.done on completion (marker for tools/wait_done.sh).

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
  # one arm (all-off baseline) on cards 0.., ports 7100..:
  python3 parallel_spawner.py --n 8 --bin /path/mercury --secs 130 \\
      --payload 512 --start-cfg 100 --cell WGN:37.6 --arm-name all_off \\
      --tag-prefix off --out /tmp/OFF.json
  # paired arm (inband+T1) on DISJOINT cards/ports, SAME n => same seed set:
  python3 parallel_spawner.py --n 8 --bin /path/mercury --secs 130 \\
      --payload 512 --start-cfg 100 --cell WGN:37.6 --arm-name inband_t1 \\
      --card-base 4 --port-base 7300 --tag-prefix ibt1 --out /tmp/IBT1.json
"""
import argparse
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ra_cleanup import scoped_cleanup_cells  # concurrency-safe cohort cleanup
import capstone_arms as CA
from warm_gate import (
    DeliveredRateWarmGate,
    WARM_CEILING_S,
    WARM_FLOOR_S,
    WARM_MAX_TX_AHEAD_BYTES,
    WARM_RATE_TOLERANCE,
    WARM_RATE_WINDOW_S,
    WARM_SAMPLE_S,
    WARM_STABLE_SAMPLES,
)
HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.join(HERE, "arq_realaudio.py")


def card_name(idx):
    # snd-aloop names the card index in UPPERCASE HEX, not decimal:
    #   idx 0      -> "Loopback"
    #   idx 1..9   -> "Loopback_1" .. "Loopback_9"   (hex == decimal here)
    #   idx 10..15 -> "Loopback_A" .. "Loopback_F"
    #   idx 16..   -> "Loopback_10", "Loopback_11", ...  ("%X" of the index)
    # A plain decimal "Loopback_%d" is WRONG for idx >= 10 (it asks for
    # "Loopback_10" while the kernel card is "Loopback_A") -> ENODEV, cell fails.
    # Verified on the fleet R730s (24-card pool, 2026-06-29). The kernel `id=`
    # module param can NOT override this (it strips underscores).
    return "Loopback" if idx == 0 else "Loopback_%X" % idx


def _arq_cell_census():
    """Count concurrent real-audio CELLS currently alive on THIS box.

    One arq_realaudio.py process == one cell, so a pgrep of that pattern is the
    clean per-cell census (the bridge path also appears in each child's argv, so
    counting bridge processes would double-count). Counts siblings from ANY
    cohort plus this spawner's already-launched cells. Best-effort: returns None
    if pgrep is unavailable. pgrep excludes its own pid, and the spawner's own
    argv does not carry the harness path, so neither is miscounted.
    """
    try:
        out = subprocess.run(["pgrep", "-f", "arq_realaudio.py"],
                             capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    return sum(1 for ln in out.stdout.split() if ln.strip())


class StartupAdmissionController:
    """Bound cold PRECOOK using real child phase events and host headroom.

    A cell is charged as cooking until its runner publishes the exact
    two-peer warm-completion edge.  Warmed-but-not-connected cells remain
    charged at the measured idle-pair cost; connected cells use a conservative
    active allowance.  Admission also requires observed host busy cores plus
    one cold cell to remain below the spendable host budget.
    """

    PHASE_ORDER = {"cooking": 0, "warmed_idle": 1, "active": 2, "done": 3}

    def __init__(self, *, enabled, host_cores, reserve_cores,
                 max_cooking, cook_cores, warmed_idle_cores, active_cores,
                 poll_s, trace_path):
        self.enabled = bool(enabled)
        self.host_cores = float(host_cores)
        self.reserve_cores = float(reserve_cores)
        self.max_cooking = int(max_cooking)
        self.cook_cores = float(cook_cores)
        self.warmed_idle_cores = float(warmed_idle_cores)
        self.active_cores = float(active_cores)
        self.poll_s = float(poll_s)
        self.trace_path = trace_path
        self.records = []
        self.errors = []
        self.max_observed = {phase: 0 for phase in self.PHASE_ORDER}
        self.max_modeled_cores = 0.0
        self.max_host_busy_cores = None
        self.wait_seconds = 0.0
        self._cpu_prev = None
        self._last_signature = None
        if self.trace_path:
            try:
                os.remove(self.trace_path)
            except FileNotFoundError:
                pass

    @property
    def budget_cores(self):
        return self.host_cores - self.reserve_cores

    def register(self, plan, proc, phase_path):
        self.records.append({
            "tag": plan["tag"], "proc": proc, "path": phase_path,
            "phase": "cooking", "phase_order": 0, "event": None,
        })
        self._trace("launched", self.snapshot(), tag=plan["tag"])

    def _read_phase(self, record):
        if record["proc"].poll() is not None:
            phase, event = "done", {"phase": "done", "source": "process_exit"}
        else:
            try:
                with open(record["path"], encoding="utf-8") as handle:
                    event = json.load(handle)
                phase = event.get("phase")
                if event.get("tag") != record["tag"]:
                    raise ValueError("event tag mismatch")
                if phase not in self.PHASE_ORDER:
                    raise ValueError(f"invalid phase {phase!r}")
            except FileNotFoundError:
                return
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                msg = f"{record['tag']}: {exc}"
                if msg not in self.errors:
                    self.errors.append(msg)
                return
        order = self.PHASE_ORDER[phase]
        if order < record["phase_order"]:
            msg = (f"{record['tag']}: phase regressed "
                   f"{record['phase']}->{phase}")
            if msg not in self.errors:
                self.errors.append(msg)
            return
        record["phase"] = phase
        record["phase_order"] = order
        record["event"] = event

    def _sample_host_busy(self):
        try:
            with open("/proc/stat", encoding="ascii") as handle:
                fields = handle.readline().split()
            values = [int(value) for value in fields[1:]]
            total = sum(values)
            idle = values[3] + (values[4] if len(values) > 4 else 0)
        except (OSError, ValueError, IndexError):
            return None
        current = (total, idle)
        previous, self._cpu_prev = self._cpu_prev, current
        if previous is None:
            return None
        dt = total - previous[0]
        didle = idle - previous[1]
        if dt <= 0:
            return None
        busy_fraction = max(0.0, min(1.0, (dt - didle) / dt))
        return busy_fraction * self.host_cores

    def snapshot(self):
        for record in self.records:
            self._read_phase(record)
        counts = {phase: 0 for phase in self.PHASE_ORDER}
        for record in self.records:
            counts[record["phase"]] += 1
        modeled = (
            counts["cooking"] * self.cook_cores
            + counts["warmed_idle"] * self.warmed_idle_cores
            + counts["active"] * self.active_cores
        )
        busy = self._sample_host_busy()
        for phase, count in counts.items():
            self.max_observed[phase] = max(self.max_observed[phase], count)
        self.max_modeled_cores = max(self.max_modeled_cores, modeled)
        if busy is not None:
            self.max_host_busy_cores = (
                busy if self.max_host_busy_cores is None
                else max(self.max_host_busy_cores, busy))
        return {"counts": counts, "modeled_cores": modeled,
                "host_busy_cores": busy}

    def _can_admit(self, snapshot):
        if not self.enabled:
            return True, "disabled"
        projected_model = snapshot["modeled_cores"] + self.cook_cores
        if snapshot["counts"]["cooking"] >= self.max_cooking:
            return False, "max_cooking"
        if projected_model > self.budget_cores:
            return False, "state_budget"
        busy = snapshot["host_busy_cores"]
        if busy is not None and busy + self.cook_cores > self.budget_cores:
            return False, "host_headroom"
        return True, "admit"

    def wait_to_admit(self, tag):
        started = time.monotonic()
        while True:
            snapshot = self.snapshot()
            allowed, reason = self._can_admit(snapshot)
            signature = (
                tuple(snapshot["counts"].items()), reason,
                None if snapshot["host_busy_cores"] is None
                else round(snapshot["host_busy_cores"], 1))
            if signature != self._last_signature:
                self._trace("decision", snapshot, tag=tag,
                            allowed=allowed, reason=reason)
                self._last_signature = signature
            if allowed:
                self.wait_seconds += time.monotonic() - started
                return snapshot
            time.sleep(self.poll_s)

    def _trace(self, kind, snapshot, **fields):
        if not self.trace_path:
            return
        row = {
            "time_ns": time.time_ns(), "event": kind,
            "counts": snapshot["counts"],
            "modeled_cores": round(snapshot["modeled_cores"], 3),
            "host_busy_cores": (
                round(snapshot["host_busy_cores"], 3)
                if snapshot["host_busy_cores"] is not None else None),
        }
        row.update(fields)
        with open(self.trace_path, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(row, sort_keys=True) + "\n")

    def report(self):
        final = self.snapshot()
        return {
            "enabled": self.enabled,
            "host_cores": self.host_cores,
            "reserve_cores": self.reserve_cores,
            "budget_cores": self.budget_cores,
            "max_cooking": self.max_cooking,
            "cost_model": {
                "cooking_cores_per_cell": self.cook_cores,
                "warmed_idle_cores_per_cell": self.warmed_idle_cores,
                "active_cores_per_cell": self.active_cores,
            },
            "poll_s": self.poll_s,
            "admission_wait_seconds": round(self.wait_seconds, 3),
            "max_phase_counts": self.max_observed,
            "max_modeled_cores": round(self.max_modeled_cores, 3),
            "max_observed_host_busy_cores": (
                round(self.max_host_busy_cores, 3)
                if self.max_host_busy_cores is not None else None),
            "final": final,
            "trace_path": os.path.abspath(self.trace_path),
            "event_errors": self.errors,
        }


def run_plan(n, port_base, card_base=0, tag_prefix="par",
             cells_per_card=2, seed_offset=0, resource_cycle=None):
    """Return list of per-run dicts: card, subs, rsp_port, cmd_port, seed, tag.

    card_base offsets the starting snd-aloop card index so multiple concurrent
    cohorts (e.g. two A/B arms, or two SNR points) can own DISJOINT cards. Each
    cohort must also use a disjoint port_base. tag_prefix keeps result/log
    filenames unique per cohort.

    cells_per_card selects the snd-aloop packing: 2 (default) fits two runs on
    one card (subs {0-3} and {4-7}); 1 gives each run its OWN card (subs {0-3})
    so a card wedge or cross-talk cannot couple two cells -- the 1-cell-per-card
    cohort mechanic (needs N cards, not ceil(N/2)).

    seed_offset shifts the deterministic per-run seed (seed = i + 1 + seed_offset)
    so sequential single-cell invocations can cover a contiguous seed range
    (e.g. seeds 1..8 across 8 `--n 1` launches). Keep it 0 for the default
    paired-seed cohort so paired A/B arms keep identical seed sets."""
    plan = []
    for i in range(n):
        if resource_cycle is not None:
            card_idx, slot = resource_cycle[i]
            subs = [slot * 4 + j for j in range(4)]
        elif cells_per_card == 1:
            card_idx = card_base + i
            subs = [j for j in range(4)]   # own card, subs 0..3
        else:
            card_idx = card_base + i // 2
            slot = i % 2                    # 0 -> subs 0..3, 1 -> subs 4..7
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
            "seed": i + 1 + seed_offset,    # deterministic per-run seed
            "tag": f"{tag_prefix}{i:02d}",
        })
    return plan


def attested_cohort_coordinate(results, tolerance_db=0.05):
    """Reduce a cohort coordinate solely from per-run bridge attestations."""
    invalid_tags = [r.get("tag") for r in results
                    if r.get("instrument_invalid")]
    coordinates = [float(r["snr3k"]) for r in results
                   if r.get("vara_scoring_enabled")
                   and r.get("snr3k") is not None]
    reasons = []
    if invalid_tags:
        reasons.append("instrument_invalid_cells")
    if coordinates and len(coordinates) != len(results):
        reasons.append("partial_attested_coordinate")
    if coordinates and max(coordinates) - min(coordinates) > tolerance_db:
        reasons.append("cohort_coordinate_mismatch")
    if reasons or not coordinates:
        return None, {
            "valid": not invalid_tags and not reasons,
            "vara_scorable": False,
            "reasons": reasons,
            "invalid_tags": invalid_tags,
            "coordinates_db": coordinates,
        }
    coordinate = sorted(coordinates)[len(coordinates) // 2]
    return coordinate, {
        "valid": True,
        "vara_scorable": True,
        "reasons": [],
        "invalid_tags": [],
        "coordinates_db": coordinates,
    }


def is_valid_result(result):
    """One canonical cohort inclusion gate; non-connects never score as valid."""
    return (
        result.get("connected") is True
        and result.get("under_warmed") is not True
        and result.get("byte_integrity_ok") is True
        and result.get("uniqueness_ok") is True
        and result.get("instrument_invalid") is not True
        and result.get("vara_scoring_enabled") is True
    )


def is_ux_population_result(result):
    """Unconditional fixed-window UX population, including nonconnect zeros."""
    score = result.get("canonical_fixed_score")
    ux = score.get("ux") if isinstance(score, dict) else None
    return (
        result.get("fixed_window_score_enabled") is True
        and result.get("under_warmed") is not True
        and result.get("byte_integrity_ok") is True
        and result.get("uniqueness_ok") is True
        and result.get("instrument_invalid") is not True
        and isinstance(score, dict)
        and score.get("instrument_invalid") is not True
        and isinstance(ux, dict)
        and ux.get("status") == "OK"
        and ux.get("scorable") is True
    )


def print_setup(n, cells_per_card=2):
    cards = n if cells_per_card == 1 else (n + 1) // 2
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
    ap.add_argument("--secs", type=int, default=130,
                    help="scored-window duration after the warm gate opens")
    ap.add_argument("--payload", type=int, default=512,
                    help="post-warm scored payload bytes (warm traffic is additional)")
    ap.add_argument(
        "--fixed-window-score", action="store_true",
        help="enable canonical request->+600 UX and connected+30->+600 "
             "sustainable-rung scoring in every child")
    ap.add_argument("--score-horizon-s", type=float, default=600.0)
    ap.add_argument("--steady-warmup-s", type=float, default=30.0)
    ap.add_argument("--warm-start", action=argparse.BooleanOptionalAction, default=True,
                    help="pre-warm both peers to link_status:Idle before the transfer "
                         "clock / CONNECT (honest field-representative metric; default). "
                         "--no-warm-start reproduces the legacy cold-launch metric.")
    ap.add_argument("--warm-gate", action=argparse.BooleanOptionalAction, default=True,
                    help="mechanically gate every cell's scored window on post-CONNECT "
                         "delivered-rate stability (default: enabled)")
    ap.add_argument("--warm-floor-s", type=float, default=WARM_FLOOR_S)
    ap.add_argument("--warm-ceiling-s", type=float, default=WARM_CEILING_S)
    ap.add_argument("--warm-sample-s", type=float, default=WARM_SAMPLE_S)
    ap.add_argument("--warm-rate-window-s", type=float, default=WARM_RATE_WINDOW_S)
    ap.add_argument("--warm-stable-samples", type=int, default=WARM_STABLE_SAMPLES)
    ap.add_argument("--warm-rate-tolerance", type=float, default=WARM_RATE_TOLERANCE)
    ap.add_argument("--warm-max-tx-ahead-bytes", type=int,
                    default=WARM_MAX_TX_AHEAD_BYTES)
    ap.add_argument("--start-cfg", type=int, default=100)
    ap.add_argument("--start-cfg-cycle", default=None,
                    help="optional comma-separated per-cell configuration cycle "
                         "(for example 16,100); overrides --start-cfg by index")
    ap.add_argument("--no-gearshift", action="store_true")
    ap.add_argument("--mode", default="wb", choices=["wb", "auto", "nb"],
                    help="bandwidth election passthrough (wb=-W default; auto=-M auto "
                         "for the FLOOR cell; nb=-M nb)")
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
    ap.add_argument("--resource-cycle", default=None,
                    help="comma-separated exact broker resources card:slot; "
                         "overrides --card-base and must contain exactly -n entries")
    ap.add_argument("--tag-prefix", default="par",
                    help="prefix for per-cell tag/result filenames (unique per cohort)")
    ap.add_argument("--logdir", default="/tmp/raspeed/logs")
    ap.add_argument("--out", default="/tmp/raspeed/PARALLEL_RESULT.json")
    ap.add_argument("--launch-stagger", type=float, default=0.0,
                    help="seconds between launching successive runs (0 = true simultaneous)")
    ap.add_argument("--startup-admission", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="bound concurrent cold PRECOOK from actual cell phase "
                         "events and host headroom (default: enabled)")
    ap.add_argument("--admission-host-cores", type=float,
                    default=float(os.cpu_count() or 1))
    ap.add_argument("--admission-reserve-cores", type=float, default=12.0,
                    help="cores reserved for bridges, audio, OS, and uncertainty")
    ap.add_argument("--admission-max-cooking", type=int, default=20,
                    help="hard cap K on cells whose two peers have not warmed")
    ap.add_argument("--admission-cook-cores", type=float, default=2.0)
    ap.add_argument("--admission-warmed-idle-cores", type=float, default=1.011)
    ap.add_argument("--admission-active-cores", type=float, default=0.5)
    ap.add_argument("--admission-poll-s", type=float, default=0.25)
    ap.add_argument("--encrypt", default=None,
                    help="pass -E <mode> to BOTH mercury in every cell (e.g. 'fast')")
    ap.add_argument("--psk", default=None,
                    help="pass -K <hex> to BOTH mercury in every cell (with --encrypt)")
    ap.add_argument("--arm", default="legacy", help="A/B arm label recorded per cell")
    ap.add_argument("--env", action="append", default=[],
                    help="KEY=VAL env injected into BOTH mercury per cell (repeatable)")
    # ---- capstone ext (a): per-lever kill-switches -------------------------
    ap.add_argument("--arm-name", default=None,
                    help="preset arm from capstone_arms.ARMS applied to EVERY cell "
                         "(e.g. all_off, inband_only, inband_t1, full_stack)")
    ap.add_argument("--arm-spec", default=None,
                    help="JSON {lever:bool} kill-switch set applied to EVERY cell")
    # ---- capstone ext (d): VARA bar + user metric --------------------------
    ap.add_argument("--snr3k", type=float, default=None,
                    help="optional assertion against each bridge-attested SNR3k")
    ap.add_argument("--snr", type=float, default=None,
                    help="channel WGN --snr (dB) forwarded to each cell. When set "
                         "(and no --cell), the bridge treats it as the realized "
                         "SNR3k directly (offset 0), so the requested dB equals the "
                         "attested snr3k label. Pass the same value to --snr3k to "
                         "assert it. Without --snr the child falls back to its own "
                         "--snr default (30 dB).")
    ap.add_argument("--traffic", default="pg84",
                    help="pg84 (USER=wire*LZHUF) | random-binary (USER=wire)")
    ap.add_argument("--fast-af-threshold", type=float, default=CA.DEFAULT_FAST_AF,
                    help="active_fraction cut for the BIMODAL fast/slow-lock split "
                         "(STEP 2b). Raw per-run active_fraction is always emitted "
                         "so the scorer can re-bin at any threshold.")
    ap.add_argument("--print-setup", action="store_true",
                    help="print the modprobe command to provision snd-aloop and exit")
    ap.add_argument("--one-cell-per-card", action="store_true",
                    help="give each run its OWN snd-aloop card (subs 0-3) instead of "
                         "packing two runs per card; needs N cards, not ceil(N/2). "
                         "Isolates cells so a card wedge/cross-talk cannot couple two "
                         "(the 1-cell-per-card cohort mechanic).")
    ap.add_argument("--seed-offset", type=int, default=0,
                    help="add to every per-run seed (seed = run_index + 1 + offset). "
                         "Lets sequential single-cell cohorts span a contiguous seed "
                         "range; keep 0 for paired A/B cohorts (identical seed sets).")
    ap.add_argument("--seed-cycle", default=None,
                    help="optional comma-separated per-cell seeds, cycled by index. "
                         "This permits heterogeneous config plans to use the same "
                         "channel/payload seed for every member of a paired block.")
    args = ap.parse_args()
    try:
        DeliveredRateWarmGate(
            floor_s=args.warm_floor_s,
            ceiling_s=args.warm_ceiling_s,
            sample_s=args.warm_sample_s,
            rate_window_s=args.warm_rate_window_s,
            stable_samples=args.warm_stable_samples,
            tolerance=args.warm_rate_tolerance,
        )
    except ValueError as exc:
        ap.error(str(exc))
    if args.warm_max_tx_ahead_bytes <= 0:
        ap.error("--warm-max-tx-ahead-bytes must be positive")
    if args.startup_admission and not args.warm_start:
        ap.error("--startup-admission requires --warm-start so a real "
                 "two-peer warm-completion event exists")
    if (args.admission_host_cores <= 0
            or args.admission_reserve_cores < 0
            or args.admission_reserve_cores >= args.admission_host_cores
            or args.admission_max_cooking <= 0
            or min(args.admission_cook_cores,
                   args.admission_warmed_idle_cores,
                   args.admission_active_cores) < 0
            or args.admission_poll_s <= 0):
        ap.error("invalid startup-admission budget/cost arguments")
    if args.admission_cook_cores > (
            args.admission_host_cores - args.admission_reserve_cores):
        ap.error("one cooking cell does not fit the startup-admission budget")
    start_cfg_cycle = None
    if args.start_cfg_cycle:
        try:
            start_cfg_cycle = [
                int(item.strip()) for item in args.start_cfg_cycle.split(",")
                if item.strip()
            ]
        except ValueError:
            ap.error("--start-cfg-cycle must contain only integer config IDs")
        if not start_cfg_cycle:
            ap.error("--start-cfg-cycle may not be empty")
    seed_cycle = None
    if args.seed_cycle:
        try:
            seed_cycle = [
                int(item.strip()) for item in args.seed_cycle.split(",")
                if item.strip()
            ]
        except ValueError:
            ap.error("--seed-cycle must contain only integer seeds")
        if not seed_cycle:
            ap.error("--seed-cycle may not be empty")
    if args.fixed_window_score and args.warm_gate:
        ap.error("--fixed-window-score and --warm-gate cannot be combined; "
                 "the canonical fixed-window scorer has a different clock origin")
    if args.fixed_window_score:
        if args.payload <= 0:
            ap.error("--fixed-window-score requires --payload > 0")
        if (args.score_horizon_s <= 0
                or args.steady_warmup_s < 0
                or args.steady_warmup_s >= args.score_horizon_s):
            ap.error(
                "--fixed-window-score requires 0 <= --steady-warmup-s "
                "< --score-horizon-s")

    cells_per_card = 1 if args.one_cell_per_card else 2
    resource_cycle = None
    if args.resource_cycle:
        try:
            resource_cycle = [tuple(int(value) for value in item.split(":"))
                              for item in args.resource_cycle.split(",")]
        except ValueError:
            ap.error("--resource-cycle must contain comma-separated card:slot entries")
        if (len(resource_cycle) != args.n
                or any(len(item) != 2 or item[0] < 0 or item[1] not in (0, 1)
                       for item in resource_cycle)
                or len(set(resource_cycle)) != len(resource_cycle)):
            ap.error("--resource-cycle must contain -n unique card:slot entries")
        if args.one_cell_per_card:
            ap.error("--resource-cycle and --one-cell-per-card are mutually exclusive")

    if args.print_setup:
        print_setup(args.n, cells_per_card)
        return 0

    os.makedirs(args.logdir, exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    plan = run_plan(args.n, args.port_base, args.card_base, args.tag_prefix,
                    cells_per_card=cells_per_card, seed_offset=args.seed_offset,
                    resource_cycle=resource_cycle)
    if start_cfg_cycle:
        for index, cell_plan in enumerate(plan):
            cell_plan["start_cfg"] = start_cfg_cycle[index % len(start_cfg_cycle)]
    if seed_cycle:
        for index, cell_plan in enumerate(plan):
            cell_plan["seed"] = seed_cycle[index % len(seed_cycle)]
    done_marker = args.out + ".done"
    try:
        if os.path.exists(done_marker):
            os.remove(done_marker)
    except OSError:
        pass

    # Resolve the cohort arm-spec ONCE for the summary header (each cell resolves
    # its own too; this is for reporting + the inert-lever warning up front).
    cohort_spec = {}
    if args.arm_name:
        cohort_spec.update(CA.normalize_spec(args.arm_name))
    if args.arm_spec:
        cohort_spec.update({k: bool(v) for k, v in json.loads(args.arm_spec).items()})
    cohort_env_kv, cohort_warnings = CA.resolve_arm(cohort_spec) if cohort_spec else ([], [])
    probe = CA.probe_binary(args.bin)

    # CONCURRENCY-SAFE cohort cleanup: clear ONLY stale leftovers for the
    # cells THIS spawner is about to launch (its own plan: each cell's card +
    # substreams + TCP ports). A concurrent cohort on disjoint
    # cards/subs/ports is never matched, so we never reap it. The old global
    # `pkill -9 -f 'mercury -m ARQ'` reaped every concurrent run's mercury
    # (an earlier diagnostic). See ra_cleanup.py.
    scoped_cleanup_cells(plan, settle=2.0)

    procs = []
    cohort_t0 = time.monotonic()
    admission_trace = os.path.abspath(args.out) + ".startup_admission.jsonl"
    admission = StartupAdmissionController(
        enabled=args.startup_admission,
        host_cores=args.admission_host_cores,
        reserve_cores=args.admission_reserve_cores,
        max_cooking=args.admission_max_cooking,
        cook_cores=args.admission_cook_cores,
        warmed_idle_cores=args.admission_warmed_idle_cores,
        active_cores=args.admission_active_cores,
        poll_s=args.admission_poll_s,
        trace_path=admission_trace,
    )
    for p in plan:
        admission.wait_to_admit(p["tag"])
        jpath = os.path.join(args.logdir, f"res_{p['tag']}.json")
        phase_path = os.path.join(
            args.logdir, f"startup_phase_{p['tag']}.json")
        try:
            os.remove(phase_path)
        except FileNotFoundError:
            pass
        # COHORT-WIDTH provenance (candidate instrument #17): stamp this
        # spawner's width (-n) plus a box-concurrency census taken AT THIS
        # cell's launch (already-launched cohort cells + any sibling cohort's
        # cells, +1 for this cell) into every result JSON, so an 8+2
        # scoreboard+fineprobe overlap is visible in the data and the scorer
        # can enforce width <= 8 on a scorable cohort.
        census = _arq_cell_census()
        box_est = (census + 1) if census is not None else None
        cmd = [sys.executable, "-u", HARNESS,
               "--bin", args.bin, "--bridge", args.bridge,
               "--tag", p["tag"], "--json", jpath, "--logdir", args.logdir,
               "--start-cfg", str(p.get("start_cfg", args.start_cfg)),
               "--secs", str(args.secs), "--payload", str(args.payload),
               "--seed", str(p["seed"]),
               "--card", p["card"], "--subs", ",".join(str(s) for s in p["subs"]),
               "--rsp-port", str(p["rsp_port"]), "--cmd-port", str(p["cmd_port"]),
               "--cap-periods", str(args.cap_periods),
               "--play-periods", str(args.play_periods),
               "--prime-periods", str(args.prime_periods),
               "--traffic", args.traffic,
               "--mode", args.mode,
               "--spawner-width", str(args.n),
               "--startup-phase-file", phase_path,
               "--no-kill"]
        if box_est is not None:
            cmd += ["--box-concurrent-estimate", str(box_est)]
        if args.no_gearshift:
            cmd += ["--no-gearshift"]
        # Current monitor direct-WB recipe: bypass the NB probe only for a
        # concrete WB starting config.  Robust cfg100+ must retain that probe.
        # This has to follow the per-cell plan rather than the cohort default
        # because W48 deliberately alternates cfg16 and cfg100.
        # Pinned ROBUST_3 is a WB payload rung even though its numeric ID is in
        # the robust family. Preserve robust MFSK hailing (-R, selected by the
        # child harness from start_cfg>=100) but bypass the NB payload probe so
        # cfg103 is seated in its designed WB M16x2 geometry on both peers.
        if p.get("start_cfg", args.start_cfg) < 100 or p.get("start_cfg", args.start_cfg) == 103:
            cmd += ["--skip-nb-probe"]
        if args.fixed_window_score:
            cmd += [
                "--fixed-window-score",
                "--score-horizon-s", str(args.score_horizon_s),
                "--steady-warmup-s", str(args.steady_warmup_s),
            ]
        cmd += (["--warm-start"] if args.warm_start else ["--no-warm-start"])
        cmd += (["--warm-gate"] if args.warm_gate else ["--no-warm-gate"])
        cmd += [
            "--warm-floor-s", str(args.warm_floor_s),
            "--warm-ceiling-s", str(args.warm_ceiling_s),
            "--warm-sample-s", str(args.warm_sample_s),
            "--warm-rate-window-s", str(args.warm_rate_window_s),
            "--warm-stable-samples", str(args.warm_stable_samples),
            "--warm-rate-tolerance", str(args.warm_rate_tolerance),
            "--warm-max-tx-ahead-bytes", str(args.warm_max_tx_ahead_bytes),
        ]
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
        if args.arm_name:
            cmd += ["--arm-name", args.arm_name]
        if args.arm_spec:
            cmd += ["--arm-spec", args.arm_spec]
        if args.snr is not None:
            cmd += ["--snr", str(args.snr)]
        if args.snr3k is not None:
            cmd += ["--snr3k", str(args.snr3k)]
        # CONNECT_FAST_CONFIG is part of the requested per-cell control mode,
        # not a cohort-global setting.  A mixed cfg16/cfg100 cohort otherwise
        # silently gives one half the wrong handshake policy: pinned WB cells
        # demote immediately without 16, while robust hailing requires -1.
        # Put the derived value first so an explicit --env remains the final,
        # operator-selected override in arq_realaudio.py's merge order.
        cell_start_cfg = p.get("start_cfg", args.start_cfg)
        connect_fast_cfg = cell_start_cfg if cell_start_cfg < 100 else -1
        cmd += ["--env", f"MERCURY_CONNECT_FAST_CONFIG={connect_fast_cfg}"]
        for kv in args.env:
            cmd += ["--env", kv]
        outlog = open(os.path.join(args.logdir, f"spawn_{p['tag']}.out"), "wb")
        proc = subprocess.Popen(cmd, stdout=outlog, stderr=subprocess.STDOUT)
        procs.append((p, proc, jpath, outlog))
        admission.register(p, proc, phase_path)
        sys.stderr.write(f"[spawner] launched {p['tag']} card={p['card']} "
                         f"subs={p['subs']} ports={p['rsp_port']}/{p['cmd_port']} "
                         f"seed={p['seed']}\n")
        sys.stderr.flush()
        if args.launch_stagger > 0:
            time.sleep(args.launch_stagger)

    sys.stderr.write(f"[spawner] all {args.n} runs launched at "
                     f"T+{time.monotonic()-cohort_t0:.1f}s; waiting...\n")
    sys.stderr.flush()

    # wait for all children (each self-terminates at --secs or on delivery)
    for p, proc, jpath, outlog in procs:
        proc.wait()
        outlog.close()
    cohort_wall = time.monotonic() - cohort_t0
    startup_admission_summary = admission.report()

    results = []
    for p, proc, jpath, _ in procs:
        try:
            with open(jpath) as f:
                r = json.load(f)
        except Exception as e:  # noqa: BLE001
            r = {"tag": p["tag"], "error": str(e), "connected": False,
                 "delivered_full": False, "rx_bytes": None,
                 "warm_gate_enabled": bool(args.warm_gate),
                 "warm_seconds": None, "under_warmed": False,
                 "scored_window_start": None,
                 "byte_integrity_ok": False, "verdict": "MISSING_RESULT",
                 "rsp_nreceived_frames": None}
        r["plan_card"] = p["card"]
        r["plan_subs"] = p["subs"]
        results.append(r)

    n_conn = sum(1 for r in results if r.get("connected"))
    n_deliv = sum(1 for r in results if r.get("rx_bytes"))
    # delivered_full is the HONEST field as of 2026-07-31 (count AND byte-exact
    # content AND uniqueness). For pre-fix cells it is count-only; the explicit
    # count-only tally below keeps the two auditable side by side.
    n_full = sum(1 for r in results if r.get("delivered_full"))
    n_full_count_only = sum(
        1 for r in results
        if r.get("delivered_full_count_only",
                 r.get("delivered_full")))
    tot_authfails = sum((r.get("aead_authfails") or 0) for r in results)
    tot_nonce_reuse = sum((r.get("nonce_reuse_count") or 0) for r in results)

    # ---- capstone ext (b): byte-integrity gate (LOUD) ----------------------
    integrity_fail_tags = [r.get("tag") for r in results
                           if r.get("byte_integrity_ok") is False]
    n_integrity_ok = sum(1 for r in results if r.get("byte_integrity_ok") is True)
    tot_bad_bytes = sum((r.get("integrity_mismatch_bytes") or 0) for r in results)

    # ---- capstone STEP 2c: delivered-byte UNIQUENESS gate (LOUD) ------------
    # A cell is VOID if EITHER byte-integrity OR uniqueness fails. uniqueness_ok
    # may be absent on an errored/missing result -> treat None as not-a-pass but
    # only FLAG an explicit False (a hard rx>tx double-delivery).
    uniqueness_fail_tags = [r.get("tag") for r in results
                            if r.get("uniqueness_ok") is False]
    instrument_invalid_tags = [r.get("tag") for r in results
                               if r.get("instrument_invalid") is True]
    under_warmed_tags = [r.get("tag") for r in results
                         if r.get("under_warmed") is True]
    warm_gate_missing_tags = [r.get("tag") for r in results
                              if (r.get("warm_gate_enabled") is not True
                                  or r.get("warm_seconds") is None
                                  or r.get("scored_window_start") is None)
                              and r.get("under_warmed") is not True]
    nonconnected_tags = [r.get("tag") for r in results
                         if r.get("connected") is not True]
    vara_unscored_tags = [r.get("tag") for r in results
                          if r.get("vara_scoring_enabled") is not True]
    n_uniqueness_ok = sum(1 for r in results if r.get("uniqueness_ok") is True)
    # A cell is scored VALID only if it connected, passed integrity, and passed
    # uniqueness. void_tags is the loud union that any downstream scorer must drop.
    void_tags = sorted(set(integrity_fail_tags) | set(uniqueness_fail_tags)
                       | set(instrument_invalid_tags) | set(vara_unscored_tags)
                       | set(nonconnected_tags) | set(under_warmed_tags)
                       | set(warm_gate_missing_tags))
    valid_results = [r for r in results if is_valid_result(r)]
    # Unlike steady/VARA aggregates, fixed-window UX is unconditional on
    # connection: a complete, instrument-valid nonconnect contributes zero.
    ux_results = [r for r in results if is_ux_population_result(r)]
    ux_rates = [
        float(r["canonical_fixed_score"]["ux"]["content_Bmin"])
        for r in ux_results
    ]
    canonical_ux_summary = ({
        "n_expected": args.n,
        "n_scored": len(ux_results),
        "n_nonconnect": sum(
            1 for r in ux_results if r.get("connected") is not True),
        "all_complete": len(ux_results) == args.n,
        "content_Bmin_median_spread": CA.median_spread(ux_rates),
        "content_Bmin_mean_std": CA.mean_std_spread(ux_rates),
    } if args.fixed_window_score else None)

    # ---- capstone ext (c): anatomy aggregate (PRIMARY scoreboard) ----------
    # Aggregated over VALID (non-void) cells only, so a byte-integrity/uniqueness
    # violation can never pollute the line-items. Both a median/spread and the
    # mean/σ/min/max the verdict cites (STEP 4) are emitted.
    def anat(r, k):
        a = r.get("anatomy") or {}
        return a.get(k)
    # NOTE (2026-07-12): keys must match the anatomy dict EXACTLY or median_spread
    # silently aggregates over all-None and fabricates a clean 0/None. Fixed two stale
    # keys: `demote_events` (retired -> real_demotes) and `active_fraction` (relabelled
    # -> active_fraction_CADENCE_NOT_DUTY, a batch-landing cadence, NOT a duty).
    _ANAT_KEYS = ["listen_guard_ms_total", "listen_guard_ms_mean", "retx_rounds",
                  "retx_frames_requeued",
                  "real_demotes", "break_events", "set_config_events",
                  "rx_timeout_events", "inband_tag_events",
                  "active_fraction_CADENCE_NOT_DUTY",
                  # C4: delivered_user_Bmin / user_content_Bmin = post-decompression
                  # USER-CONTENT rate (== wire ONLY for incompressible). The old
                  # "wire_Bmin" was the content rate mislabeled — no longer scored.
                  # 2026-07-31: true_wire_keyed_Bmin RETIRED (modeled numerator over
                  # measured wall — neither keyed nor wire; _research/
                  # CANONICAL_METRICS.md). Its value survives only as
                  # modeled_wire_capacity_Bmin (capacity model / tripwire).
                  # bytes_per_fwd_airtime = MEASURED in-burst keyed goodput
                  # (good-prefix bytes per second of forward PTT airtime).
                  "delivered_user_Bmin", "user_content_Bmin",
                  "modeled_wire_capacity_Bmin", "bytes_per_fwd_airtime"]
    anatomy_agg = {k: CA.median_spread([anat(r, k) for r in valid_results])
                   for k in _ANAT_KEYS}
    anatomy_mean_std = {k: CA.mean_std_spread([anat(r, k) for r in valid_results])
                        for k in _ANAT_KEYS}

    # ---- capstone STEP 2b: BIMODAL-LOCK scoring ----------------------------
    # Split valid cells fast/slow by active_fraction; report P(fast) + per-mode
    # delivered B/min (the RX/ACK levers move P(fast), not the fast-mode bps).
    bimodal = CA.bimodal_score(valid_results, args.fast_af_threshold)

    # ---- capstone STEP 2a: per-lever ENGAGEMENT aggregate ------------------
    # Sum each lever's fire-count across all cells + count how many cells fired it
    # (proves the lever engaged where it should; a 0 where a fire was expected is
    # the review-R1 silently-dead / merge-misaligned signal).
    engage_total = CA.empty_engage()
    engage_cells_fired = {f: 0 for f in CA.ENGAGE_FIELDS}
    n_engage_emitted = 0
    for r in results:
        eng = r.get("engagement")
        if not eng:
            continue
        if r.get("engage_emitted_by"):
            n_engage_emitted += 1
        for f in CA.ENGAGE_FIELDS:
            v = int(eng.get(f, 0) or 0)
            engage_total[f] += v
            if v > 0:
                engage_cells_fired[f] += 1

    # ---- C4 IMPOSSIBILITY GUARD (cohort): surface any cell whose reported rate
    # exceeded the physical wire cap (decompressed-content mislabeled 'wire' — the
    # 109,409 B/min artifact). Loud, like the integrity/uniqueness gates.
    impossible_rate_tags = [r.get("tag") for r in results
                            if r.get("rate_impossible") is True]

    # ---- C5 CONFIG-HELD TALLY (cohort): the durable cfg17-held-vs-demoted answer
    # the endpoint-floor sweep lacked (it recorded only decode16=1.0). Applies only
    # to pinned WB cells (target_config not None).
    held_cells = [r for r in results if r.get("target_config") is not None]
    n_held = sum(1 for r in held_cells if r.get("config_held") is True)
    n_demoted = sum(1 for r in held_cells if r.get("config_held") is False)
    n_decode_at_target = sum(1 for r in held_cells if r.get("decode_at_target") is True)
    demoted_to_hist = {}
    for r in held_cells:
        if r.get("config_held") is False:
            d = r.get("demoted_to")
            demoted_to_hist[str(d)] = demoted_to_hist.get(str(d), 0) + 1
    config_held_summary = ({
        "target_config": held_cells[0].get("target_config") if held_cells else None,
        "n_cells": len(held_cells),
        "n_held": n_held,
        "n_demoted": n_demoted,
        "n_decode_at_target": n_decode_at_target,   # honest decode17 count for a cfg17 target
        "demoted_to_hist": demoted_to_hist,         # e.g. {"16": 3} = 3 cells demoted to cfg16
    } if held_cells else None)

    # ---- capstone ext (d): TWO-TRAFFIC VARA bar for this cohort -------------
    # C4 (metric-correction 2026-07-04, re-corrected 2026-07-31): the headline is
    # the CONTENT race (delivered_user_Bmin, post-decompression) scored vs the
    # traffic-correct VARA bar — CLIENT (wire×LZHUF) for compressible, WIRE for
    # incompressible. Content-vs-content, whole-vs-whole; fair, but NOT a wire
    # rate for compressible traffic (do not label it 'wire').
    # The old "WIRE race" (true_wire_keyed_Bmin vs the VARA wire bar) is RETIRED:
    # its numerator was a table MODEL (airtime x CONFIG_NET_BPS), not a
    # measurement, over the measured wall — a model-vs-measurement ratio that
    # misled the campaign (see _research/CANONICAL_METRICS.md). The model value
    # survives only as modeled_wire_capacity_Bmin (capacity audit / tripwire).
    snr3k, cohort_channel_audit = attested_cohort_coordinate(results)
    if snr3k is not None:
        vara_bar_Bmin, vara_bar_kind, corpus_lzhuf = CA.vara_bar_for_traffic(
            snr3k, args.traffic)
        vara_client = CA.vara_bar(snr3k)      # legacy client-table reference
        vara_wire_Bmin = CA.vara_wire(snr3k)  # WIRE bar reference
    else:
        vara_bar_Bmin = vara_bar_kind = corpus_lzhuf = None
        vara_client = vara_wire_Bmin = None
    user_med = anatomy_agg["delivered_user_Bmin"]["median"] if anatomy_agg["delivered_user_Bmin"] else None
    user_mean = (anatomy_mean_std["delivered_user_Bmin"]["mean"]
                 if anatomy_mean_std["delivered_user_Bmin"] else None)
    vs_vara_median = round(user_med / vara_bar_Bmin, 4) if (vara_bar_Bmin and user_med) else None
    vs_vara_mean = round(user_mean / vara_bar_Bmin, 4) if (vara_bar_Bmin and user_mean) else None
    # Modeled capacity passthrough (audit only; never a rate, never a ratio).
    mwc_med = (anatomy_agg["modeled_wire_capacity_Bmin"]["median"]
               if anatomy_agg["modeled_wire_capacity_Bmin"] else None)
    mwc_mean = (anatomy_mean_std["modeled_wire_capacity_Bmin"]["mean"]
                if anatomy_mean_std["modeled_wire_capacity_Bmin"] else None)
    # CANONICAL MEASURED RATES (2026-07-31; _research/CANONICAL_METRICS.md):
    # in-burst keyed goodput (== bytes_per_fwd_airtime) + whole-transfer rate.
    keyed_med = (anatomy_agg["bytes_per_fwd_airtime"]["median"]
                 if anatomy_agg["bytes_per_fwd_airtime"] else None)
    wall_rate_agg = CA.median_spread(
        [r.get("wall_user_rate_Bps") for r in valid_results])
    wall_rate_med = wall_rate_agg["median"] if wall_rate_agg else None

    summary = {
        "n": args.n,
        # cohort width (== -n) recorded so a scorable cohort's width can be
        # enforced downstream (candidate instrument #17). Per-cell result JSONs
        # carry spawner_width + box_concurrent_estimate too.
        "spawner_width": args.n,
        "arm": args.arm,
        # -- arm spec (capstone ext a) --
        "arm_name": args.arm_name,
        "arm_spec": cohort_spec,
        "arm_env_injected": cohort_env_kv,
        "arm_lever_warnings": cohort_warnings,
        "arm_inert_levers": [
            f"{lever} ({CA.LEVERS[lever]['probe']} absent from binary)"
            for lever, on in CA.normalize_spec(cohort_spec).items()
            if on and CA.LEVERS[lever]["kind"] == "runtime" and probe.get(lever) is False
        ],
        "bin_lever_probe": probe,
        "encrypt": args.encrypt,
        "cohort_wall_secs": round(cohort_wall, 1),
        "passthrough": args.passthrough,
        "cell": args.cell,
        "traffic": args.traffic,
        "snr3k": snr3k,
        "requested_snr3k": args.snr3k,
        "channel_attestation": cohort_channel_audit,
        "instrument_invalid": bool(instrument_invalid_tags),
        "vara_scoring_enabled": bool(cohort_channel_audit["vara_scorable"]),
        "start_cfg": args.start_cfg,
        "start_cfg_cycle": start_cfg_cycle,
        "seed_cycle": seed_cycle,
        "mode": args.mode,
        "startup_admission": startup_admission_summary,
        "n_connected": n_conn,
        "n_delivered_any": n_deliv,
        "n_delivered_full": n_full,                     # HONEST completions (2026-07-31)
        "n_delivered_full_count_only": n_full_count_only,  # legacy count-only tally (shear-blind)
        "all_connected": n_conn == args.n,
        # -- BYTE-INTEGRITY + UNIQUENESS gates (capstone ext b + STEP 2c) — LOUD --
        "n_valid": len(valid_results),
        "warm_gate": {
            "enabled": bool(args.warm_gate),
            "floor_s": args.warm_floor_s,
            "ceiling_s": args.warm_ceiling_s,
            "sample_s": args.warm_sample_s,
            "rate_window_s": args.warm_rate_window_s,
            "stable_samples": args.warm_stable_samples,
            "rate_tolerance_fraction": args.warm_rate_tolerance,
            "max_tx_ahead_bytes": args.warm_max_tx_ahead_bytes,
            "n_under_warmed": len(under_warmed_tags),
            "under_warmed_tags": under_warmed_tags,
            "n_gate_missing": len(warm_gate_missing_tags),
            "gate_missing_tags": warm_gate_missing_tags,
        },
        "canonical_fixed_ux": canonical_ux_summary,
        "byte_integrity": {
            "all_ok": len(integrity_fail_tags) == 0,
            "n_ok": n_integrity_ok,
            "n_fail": len(integrity_fail_tags),
            "failed_tags": integrity_fail_tags,
            "total_bad_bytes": tot_bad_bytes,
        },
        "uniqueness": {
            "all_ok": len(uniqueness_fail_tags) == 0,
            "n_ok": n_uniqueness_ok,
            "n_fail": len(uniqueness_fail_tags),
            "failed_tags": uniqueness_fail_tags,
        },
        "void_tags": void_tags,
        "instrument_invalid_tags": instrument_invalid_tags,
        "nonconnected_tags": nonconnected_tags,
        "vara_unscored_tags": vara_unscored_tags,
        "INTEGRITY_FAILURES": (
            f"*** {len(integrity_fail_tags)} CELL(S) FAILED BYTE-INTEGRITY: "
            f"{integrity_fail_tags} ({tot_bad_bytes} bad bytes) ***"
            if integrity_fail_tags else None),
        "UNIQUENESS_FAILURES": (
            f"*** {len(uniqueness_fail_tags)} CELL(S) FAILED UNIQUENESS "
            f"(delivered>fed double-delivery): {uniqueness_fail_tags} ***"
            if uniqueness_fail_tags else None),
        # -- ANATOMY aggregate (capstone ext c; PRIMARY scoreboard; VALID cells) --
        "anatomy_median_spread": anatomy_agg,
        "anatomy_mean_std": anatomy_mean_std,
        # -- BIMODAL-LOCK split (capstone STEP 2b) --
        "bimodal": bimodal,
        # -- PER-LEVER ENGAGEMENT aggregate (capstone STEP 2a) --
        "engagement_total": engage_total,
        "engagement_cells_fired": engage_cells_fired,
        "n_engage_emitted": n_engage_emitted,
        # -- TWO-TRAFFIC VARA bar (capstone ext d) --
        "traffic_compressible": CA.is_compressible_traffic(args.traffic),
        "vara_bar_Bmin": vara_bar_Bmin,        # the bar THIS cohort is scored against
        "vara_bar_kind": vara_bar_kind,        # "client" (compressible) | "wire" (incompressible)
        "corpus_lzhuf_ratio": corpus_lzhuf,    # per-corpus MEASURED LZHUF (client-bar multiplier)
        "vara_wire_Bmin": vara_wire_Bmin,      # reference WIRE bar
        "vara_client_Bmin": vara_client,       # reference legacy client table (wire*2.0907)
        "cohort_user_Bmin_median": user_med,   # CONTENT rate (post-decompression; NOT wire for compressible)
        "cohort_user_Bmin_mean": user_mean,
        "cohort_vs_vara_median": vs_vara_median,   # CONTENT vs traffic-correct bar (client|wire)
        "cohort_vs_vara_mean": vs_vara_mean,
        # -- CANONICAL MEASURED RATES (2026-07-31; whole-vs-whole is the verdict) --
        "cohort_keyed_user_rate_Bps_median": keyed_med,   # in-burst keyed goodput (measured)
        "cohort_wall_user_rate_Bps_median": wall_rate_med,  # whole-transfer rate (measured)
        # -- MODELED capacity passthrough (audit/tripwire only; NOT a rate). The
        # old cohort_true_wire_* keys and their vs-VARA ratios are RETIRED: a
        # modeled numerator over a measured wall in a measured-bar ratio
        # (_research/CANONICAL_METRICS.md, instrument #13). --
        "cohort_modeled_wire_capacity_Bmin_median": mwc_med,
        "cohort_modeled_wire_capacity_Bmin_mean": mwc_mean,
        # -- C4 IMPOSSIBILITY GUARD (LOUD) --
        "rate_impossible": {
            "any": len(impossible_rate_tags) > 0,
            "n": len(impossible_rate_tags),
            "tags": impossible_rate_tags,
        },
        "IMPOSSIBLE_RATE": (
            f"*** {len(impossible_rate_tags)} CELL(S) REPORTED A PHYSICALLY-IMPOSSIBLE "
            f"RATE (> wire cap; decompressed-content mislabeled 'wire' — the 109k "
            f"artifact): {impossible_rate_tags} ***"
            if impossible_rate_tags else None),
        # -- C5 CONFIG-HELD tally (cfg17 held-vs-demoted; None if no pinned WB cells) --
        "config_held_summary": config_held_summary,
        "vara_source": CA.VARA_SOURCE,
        "total_aead_authfails": tot_authfails,
        "total_nonce_reuse": tot_nonce_reuse,
        "per_run": [
            {"tag": r.get("tag"), "card": r.get("plan_card"),
             "subs": r.get("plan_subs"), "seed": r.get("seed"),
             "start_cfg": r.get("start_cfg"),
             "connected": r.get("connected"),
             "connected_at_s": r.get("connected_at_s"),
             "warm_seconds": r.get("warm_seconds"),
             "under_warmed": r.get("under_warmed"),
             "scored_window_start": r.get("scored_window_start"),
             "scored_window_seconds": r.get("scored_window_seconds"),
             "warm_rx_bytes": r.get("warm_rx_bytes"),
             "scored_rx_bytes": r.get("scored_rx_bytes"),
             "scored_tx_bytes": r.get("scored_tx_bytes"),
             "scored_good_prefix_bytes": r.get("scored_good_prefix_bytes"),
             "rx_bytes": r.get("rx_bytes"),
             "scored_breaks": r.get("scored_breaks"),
             # delivered_full is HONEST as of 2026-07-31 (count AND byte-exact
             # content AND uniqueness); count-only legacy value rides along.
             "delivered_full": r.get("delivered_full"),
             "delivered_full_count_only": r.get("delivered_full_count_only"),
             "content_shear_suspected": r.get("content_shear_suspected"),
             "payload_target": r.get("payload_target"),
             "verdict": r.get("verdict"),
             "byte_integrity_ok": r.get("byte_integrity_ok"),
             "integrity_mismatch_bytes": r.get("integrity_mismatch_bytes"),
             "integrity_first_bad_offset": r.get("integrity_first_bad_offset"),
             "uniqueness_ok": r.get("uniqueness_ok"),
             "delivered_exceeds_fed": r.get("delivered_exceeds_fed"),
             "engagement": r.get("engagement"),
             "engage_emitted_by": r.get("engage_emitted_by"),
             "tx_bytes": r.get("tx_bytes"),
             "climbed_past_robust0": r.get("climbed_past_robust0"),
             "max_config_reached": r.get("max_config_reached"),      # MIXED robust+WB ids
             "max_wb_config_reached": r.get("max_wb_config_reached"),  # the real WB ceiling
             "real_demotes": r.get("real_demotes"),
             "config_climbs": r.get("config_climbs"),
             # -- C5: cfg17 held-vs-demoted + live measure_variance --
             "target_config": r.get("target_config"),
             "config_held": r.get("config_held"),
             "demoted_to": r.get("demoted_to"),
             "decode_at_target": r.get("decode_at_target"),
             "measure_variance": r.get("measure_variance"),
             "anatomy": r.get("anatomy"),
             "snr3k": r.get("snr3k"),
             "vara_client_Bmin": r.get("vara_client_Bmin"),
             "delivered_user_Bmin": r.get("delivered_user_Bmin"),   # CONTENT rate (post-decompression)
             # canonical measured rates (2026-07-31) + labeled model passthrough
             "keyed_user_rate_Bps": r.get("keyed_user_rate_Bps"),
             "wall_user_rate_Bps": r.get("wall_user_rate_Bps"),
             "modeled_wire_capacity_Bmin": r.get("modeled_wire_capacity_Bmin"),
             "wire_cap_Bmin": r.get("wire_cap_Bmin"),
             "rate_impossible": r.get("rate_impossible"),
             "vs_vara": r.get("vs_vara"),                           # CONTENT vs traffic-correct bar
             "arm_inert_levers": r.get("arm_inert_levers"),
             "encrypt": r.get("encrypt"),
             "enc_activated": r.get("enc_activated"),
             "aead_authfails": r.get("aead_authfails"),
             "nonce_enc_count": r.get("nonce_enc_count"),
             "nonce_reuse_count": r.get("nonce_reuse_count"),
             "dec_ok_count": r.get("dec_ok_count"),
             "rsp_nreceived_frames": r.get("rsp_nreceived_frames"),
             "configs_seen": r.get("configs_seen"),
             # axis + width provenance (instrument #16 / candidate #17)
             "psig_mode": r.get("psig_mode"),
             "psig_mode_source": r.get("psig_mode_source"),
             "spawner_width": r.get("spawner_width"),
             "box_concurrent_estimate": r.get("box_concurrent_estimate"),
             "wall_secs": r.get("wall_secs")}
            for r in results
        ],
    }
    with open(args.out, "w") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))

    # DONE marker for tools/wait_done.sh (write LAST, after the summary is on disk)
    try:
        with open(done_marker, "w") as f:
            f.write(f"done n={args.n} arm={args.arm_name or args.arm} "
                    f"valid={len(valid_results)} "
                    f"integrity_fail={len(integrity_fail_tags)} "
                    f"uniqueness_fail={len(uniqueness_fail_tags)} "
                    f"p_fast={bimodal.get('p_fast')} "
                    f"wall={round(cohort_wall,1)}s\n")
    except OSError:
        pass
    cell_safety_failed = any(
        r.get("byte_integrity_ok") is not True
        or r.get("uniqueness_ok") is not True
        or r.get("instrument_invalid") is True
        for r in results
    )
    return 1 if cell_safety_failed or not cohort_channel_audit["valid"] else 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""ra_reduce.py - the blessed reducer for real-audio per-cell res JSONs.

This is the ONE reduction path for real-audio throughput claims (see
_research/CANONICAL_METRICS.md). It reads per-cell result JSONs written by
tools/sim/realaudio/arq_realaudio.py (old or new field vintage), emits the
canonical per-cell fields, runs the reconstruction self-check on every cell,
and aggregates per (arm, binary, snr3k) group.

Canonical per-cell fields (all MEASURED; no modeled numerator ever becomes a
rate here):

  completion_honest       rx>=payload AND byte_integrity_ok AND uniqueness_ok
                          (the field-level #12 form; recomputed from raw for
                          legacy cells, cross-checked against delivered_full
                          for post-2026-07-31 cells)
  keyed_user_rate_Bps     good_prefix_bytes / ptt_fwd_airtime_s  (in-burst)
  whole_user_rate_Bps     good_prefix_bytes / wall_secs          (THE metric)
  duty                    ptt_fwd_airtime_s / wall_secs
  ramp_s                  connected_at_s (warm hail->connect seconds)
  corruption              byte_integrity_ok / first_bad / mismatch counts /
                          shear flag
  vs_vara_whole           whole rate vs the traffic-correct VARA bar

RECONSTRUCTION LAW (per cell): whole == keyed x duty (identically, from the
same raw fields), and whole == keyed x duty_post_ramp x (1 - ramp/wall).
recon_ok=False (rel err > tolerance) means a factor came from a different
window or numerator class - the cell's decomposition may not be cited.

NOTE ON WHAT THE LAW CAN AND CANNOT CATCH: when all three factors are
recomputed here from the same raw fields (gp, fwd, wall) the identity holds
by construction - its role is to forbid citing any decomposition NOT derived
that way. The tripwire that actually catches a stored-field numerator swap
(the instrument-#13 class: a modeled value smuggled into a rate field) is the
STORED-FIELD CROSS-CHECK: every harness-stored rate field is compared against
the raw recompute in its own numerator class -
  bytes_per_fwd_airtime        vs good_prefix_bytes / ptt_fwd_airtime_s
  anatomy.ptt_duty             vs ptt_fwd_airtime_s / wall_secs
  windows.whole_transfer_warm  vs rx_bytes / wall_secs   (that field's
      numerator is the DELIVERED stream, corruption-blind by design; the
      honest whole rate is always the good-prefix recompute)
Any relative divergence > tolerance sets recon_ok=False and is counted in
n_stored_xcheck_fail. On the 48-cell P2 reference set the worst stored-field
divergence is 9.3e-4 (field rounding); the retired true_wire_keyed value
replayed through this check diverges by ~0.4 and is flagged.

Retired meters (#13): true_wire_keyed_Bmin and its cohort/vara ratios are
NEVER read as rates; the value is passed through ONLY as
modeled_wire_capacity_Bmin for capacity-model audit.

AXIS + WIDTH GUARDRAIL (the vs-bar / scoreboard gate): a cell that carries a
VARA bar is a vs-bar row, and a vs-bar comparison is only meaningful on the
honest (steady) P_sig axis at a cohort width the high-SNR break/storm fraction
is not perturbed by. Two confirmed poisons are refused before any scoreboard is
emitted: a hot/unknown P_sig axis (instrument #16) and an over-wide / unrecorded
cohort width (candidate instrument #17). Refusal is LOUD and lists every
offending cell. Legitimate exceptions get an explicit escape (--axis-override
peak-control | hwcal). A reduction with no bar, or one run with --no-vs-bar, is
byte-identical to the legacy reducer (reduce_cell/aggregate are untouched; the
gate is a pre-emit check).

Usage:
  python3 ra_reduce.py <dir-or-json> [<dir-or-json> ...]
      [--out reduced.json] [--expect-n N] [--recon-tol 0.01]
      [--axis-override {peak-control,hwcal}] [--no-vs-bar]

A parser that matches nothing fabricates a clean answer: the reducer asserts
a nonzero cell count and prints it (use --expect-n to hard-gate).
"""
import argparse
import glob
import json
import os
import statistics
import sys

RECON_TOL_DEFAULT = 0.01   # 1%; stored-field rounding explains <= ~0.3%


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def load_cells(paths):
    """Collect per-cell res JSONs from files/dirs. Returns (cells, files)."""
    files = []
    for p in paths:
        if os.path.isdir(p):
            files.extend(sorted(glob.glob(os.path.join(p, "*.json"))))
        else:
            files.append(p)
    cells = []
    for f in files:
        try:
            with open(f) as fh:
                d = json.load(fh)
        except (OSError, ValueError) as e:
            cells.append({"_file": f, "_load_error": str(e)})
            continue
        if not isinstance(d, dict) or "rx_bytes" not in d:
            # not a per-cell res JSON (cohort summaries etc.) - skip loudly
            cells.append({"_file": f, "_load_error": "not_a_cell_json"})
            continue
        d["_file"] = f
        cells.append(d)
    return cells, files


# --------------------------------------------------------------------------
# AXIS + WIDTH GUARDRAIL (the vs-bar / scoreboard gate)
# --------------------------------------------------------------------------
# A vs-bar row compares a cell's MEASURED rate to the VARA bar. That comparison
# is only meaningful on the honest (steady) P_sig axis and at a cohort width the
# high-SNR link-phase break/storm fraction is not perturbed by. Two confirmed
# poisons must never silently reach the scoreboard:
#   * instrument #16 -- a hot 'peak' P_sig axis (or an unknown/missing one)
#     hot-labels the delivered SNR, so the cell sits at a DIFFERENT true
#     coordinate than its label claims when scored against the bar;
#   * candidate instrument #17 -- a scoreboard scored at a cohort width > 8
#     (a single over-wide spawner) OR with a sibling cohort overlapping on the
#     box (the 8+2 confound) perturbs the break/storm fraction, and an
#     unrecorded width hides it.
# reduce_cell()/aggregate() are unchanged; this is a PRE-EMIT gate, so a
# reduction with no bar (or run with --no-vs-bar) stays byte-identical to the
# legacy reducer.
_AXIS_OVERRIDES = ("peak-control", "hwcal")


def _int_or_none(x):
    try:
        return int(x)
    except (TypeError, ValueError):
        return None


def _axis_width_offense(d, override=None):
    """Return the list of axis/width offense reasons for a vs-bar cell.

    Empty list == the cell is clean and may be scored against the bar.
    `override` admits the two legitimate exceptions:
      'peak-control' : a deliberate peak CONTROL arm (a peak axis is intended);
      'hwcal'        : a CAL-measured hardware row (no sim P_sig axis and no
                       spawner width exist to enforce).
    """
    reasons = []
    if override == "hwcal":
        # A real-hardware calibration row has neither a sim P_sig mode nor a
        # spawner width; the operator asserts it explicitly, so nothing to gate.
        return reasons
    mode = d.get("psig_mode")
    mode_s = mode.strip().lower() if isinstance(mode, str) else mode
    # ---- axis (P_sig mode; instrument #16) ----
    if override == "peak-control":
        if mode_s not in ("steady", "peak"):
            reasons.append(
                "psig_mode=%r (peak-control override admits only steady/peak)"
                % (mode,))
    elif mode_s is None:
        reasons.append("psig_mode MISSING (axis provenance unknown; "
                       "instrument #16 -- pass --axis-override if legitimate)")
    elif mode_s != "steady":
        reasons.append("psig_mode=%r != 'steady' (hot axis, instrument #16)"
                       % (mode,))
    # ---- width (cohort concurrency; candidate instrument #17) ----
    sw = d.get("spawner_width")
    if sw is None:
        sw = d.get("cohort_width")        # tolerate the alternate field name
    if sw is None:
        reasons.append("spawner_width MISSING (cohort width unrecorded; "
                       "candidate instrument #17)")
    else:
        swi = _int_or_none(sw)
        if swi is None:
            reasons.append("spawner_width=%r not an integer" % (sw,))
        elif swi > 8:
            reasons.append("spawner_width=%d > 8 (over-wide cohort, "
                           "candidate instrument #17)" % (swi,))
    # Box concurrency catches the 8+2 sibling-overlap that spawner_width alone
    # misses (spawner_width=8 but two sibling cells ran on the box == width 10).
    bce = _int_or_none(d.get("box_concurrent_estimate"))
    if bce is not None and bce > 8:
        reasons.append("box_concurrent_estimate=%d > 8 (a scoreboard+sibling "
                       "overlap ran on the box; the 8+2 concurrency confound)"
                       % (bce,))
    return reasons


def reduce_cell(d, recon_tol=RECON_TOL_DEFAULT):
    """Canonical per-cell reduction. Pure; raises nothing on missing fields."""
    if "_load_error" in d:
        return {"file": d.get("_file"), "load_error": d["_load_error"],
                "valid": False}
    an = d.get("anatomy") or {}
    win = d.get("windows") or {}
    rx = d.get("rx_bytes") or 0
    payload = d.get("payload_target")
    gp = d.get("good_prefix_bytes")
    if gp is None:
        # legacy fallback: clean cell -> rx; corrupt -> first bad offset
        gp = (d.get("integrity_first_bad_offset")
              if d.get("byte_integrity_ok") is False else rx)
    integ_ok = d.get("byte_integrity_ok")
    uniq_ok = d.get("uniqueness_ok")
    wall = _f(d.get("wall_secs"))
    fwd = _f(an.get("ptt_fwd_airtime_s"))
    ramp = _f(d.get("connected_at_s"))

    # ---- completion (honest form; #12) ----------------------------------
    count_full = (payload is not None) and (rx >= payload)
    honest = bool(count_full and integ_ok is True and uniq_ok is True)
    if "delivered_full_count_only" in d:
        basis = "harness_honest_field"
        field_disagrees = bool(d.get("delivered_full")) != honest
    else:
        basis = "recomputed_legacy"     # pre-fix cell: delivered_full is count-only
        field_disagrees = False

    # ---- corruption ------------------------------------------------------
    shear = d.get("content_shear_suspected")
    if shear is None:
        shear = bool(count_full and integ_ok is False)

    # ---- measured rates (raw recompute; never a modeled numerator) -------
    # keyed gates on gp is not None (a zero-delivery keyed rate is honestly
    # 0.0, not excluded-from-medians)
    keyed = (gp / fwd) if (gp is not None and fwd and fwd > 0) else None
    whole = (gp / wall) if (gp is not None and wall and wall > 0) else None
    rx_whole = (rx / wall) if (wall and wall > 0) else None
    duty = (fwd / wall) if (fwd and wall and wall > 0) else None
    duty_post = (fwd / (wall - ramp)
                 if (fwd and wall and ramp is not None and wall > ramp)
                 else None)
    # harness-side cross-checks (rounded stored fields)
    keyed_field = _f(d.get("bytes_per_fwd_airtime")
                     if d.get("bytes_per_fwd_airtime") is not None
                     else an.get("bytes_per_fwd_airtime"))
    whole_warm_field = None
    ww = (win.get("whole_transfer_warm") or {}).get("content_Bmin")
    if ww:
        whole_warm_field = ww / 60.0

    # ---- reconstruction self-check (the law) -----------------------------
    # Layer 1 - identity: keyed x duty == whole. When all three come from the
    # same raw fields this holds by construction; it exists to forbid citing
    # any decomposition NOT derived that way (mixed windows / numerators).
    recon = None
    recon_err = None
    recon3_err = None
    recon_ok = None
    if keyed is not None and duty is not None and whole:
        recon = keyed * duty
        recon_err = abs(recon - whole) / whole
        if duty_post is not None and wall:
            recon3 = keyed * duty_post * (1.0 - ramp / wall)
            recon3_err = abs(recon3 - whole) / whole
        recon_ok = recon_err <= recon_tol and (
            recon3_err is None or recon3_err <= recon_tol)

    # Layer 2 - stored-field cross-check: the tripwire that catches an
    # instrument-#13-class numerator swap (a modeled value in a stored rate
    # field). Each harness-stored rate field is compared against the raw
    # recompute in ITS OWN numerator class. The whole-transfer window field's
    # numerator is the DELIVERED stream (corruption-blind by design), so it
    # is checked against rx/wall, never gp/wall.
    xcheck_keyed_err = None
    xcheck_duty_err = None
    xcheck_whole_err = None
    if keyed_field is not None and keyed:
        xcheck_keyed_err = abs(keyed_field - keyed) / keyed
    duty_field = _f(an.get("ptt_duty"))
    if duty_field is not None and duty:
        xcheck_duty_err = abs(duty_field - duty) / duty
    if whole_warm_field is not None and rx_whole:
        xcheck_whole_err = abs(whole_warm_field - rx_whole) / rx_whole
    xerrs = [e for e in (xcheck_keyed_err, xcheck_duty_err, xcheck_whole_err)
             if e is not None]
    stored_xcheck_ok = (max(xerrs) <= recon_tol) if xerrs else None
    if stored_xcheck_ok is False:
        recon_ok = False

    # ---- modeled capacity passthrough (audit only; NOT a rate) -----------
    modeled = (d.get("modeled_wire_capacity_Bmin")
               if d.get("modeled_wire_capacity_Bmin") is not None
               else d.get("true_wire_keyed_Bmin"))   # legacy name (#13, retired)

    vara_bar = _f(d.get("vara_bar_Bmin"))
    vs_vara_whole = (round(whole * 60.0 / vara_bar, 4)
                     if (whole and vara_bar) else None)

    return {
        "file": os.path.basename(d.get("_file") or ""),
        "valid": d.get("verdict") not in ("INSTRUMENT_INVALID",)
                 and not d.get("instrument_invalid"),
        "tag": d.get("tag"), "arm": d.get("arm"), "seed": d.get("seed"),
        "snr3k": d.get("snr3k"), "bin_md5": d.get("bin_md5"),
        "traffic": d.get("traffic"),
        "connected": d.get("connected"),
        "verdict": d.get("verdict"),
        # completion (#12 honest form)
        "completion_honest": honest,
        "completion_basis": basis,
        "completion_count_only": count_full,
        "completion_field_disagrees": field_disagrees,
        "completion_at_s": d.get("completion_at_s"),
        # corruption
        "byte_integrity_ok": integ_ok,
        "integrity_first_bad_offset": d.get("integrity_first_bad_offset"),
        "integrity_mismatch_bytes": d.get("integrity_mismatch_bytes"),
        "integrity_mismatch_segments": d.get("integrity_mismatch_segments"),
        "content_shear_suspected": shear,
        "uniqueness_ok": uniq_ok,
        # raw inputs (provenance)
        "rx_bytes": rx, "payload_target": payload,
        "good_prefix_bytes": gp,
        "wall_secs": wall, "ptt_fwd_airtime_s": fwd, "ramp_s": ramp,
        # canonical measured rates
        "keyed_user_rate_Bps": round(keyed, 1) if keyed is not None else None,
        "whole_user_rate_Bps": round(whole, 1) if whole is not None else None,
        "rx_whole_rate_Bps": round(rx_whole, 1) if rx_whole is not None else None,
        "duty": round(duty, 4) if duty is not None else None,
        "duty_post_ramp": round(duty_post, 4) if duty_post is not None else None,
        # harness-side cross-checks
        "keyed_field_Bps": keyed_field,
        "whole_warm_field_Bps": (round(whole_warm_field, 1)
                                 if whole_warm_field is not None else None),
        # reconstruction law (identity layer)
        "recon_whole_Bps": round(recon, 2) if recon is not None else None,
        "recon_rel_err": round(recon_err, 5) if recon_err is not None else None,
        "recon3_rel_err": round(recon3_err, 5) if recon3_err is not None else None,
        # stored-field cross-check layer (the #13-class tripwire)
        "xcheck_keyed_rel_err": (round(xcheck_keyed_err, 5)
                                 if xcheck_keyed_err is not None else None),
        "xcheck_duty_rel_err": (round(xcheck_duty_err, 5)
                                if xcheck_duty_err is not None else None),
        "xcheck_whole_rel_err": (round(xcheck_whole_err, 5)
                                 if xcheck_whole_err is not None else None),
        "stored_xcheck_ok": stored_xcheck_ok,
        "recon_ok": recon_ok,
        # model passthrough (audit only) + bar
        "modeled_wire_capacity_Bmin": modeled,
        "vara_bar_Bmin": vara_bar,
        "vs_vara_whole": vs_vara_whole,
    }


def _med(vals):
    vals = [v for v in vals if v is not None]
    return round(statistics.median(vals), 2) if vals else None


def aggregate(rows):
    """Group by (arm, bin_md5, snr3k); emit the canonical group scoreboard."""
    groups = {}
    for r in rows:
        if r.get("load_error"):
            continue
        key = (r.get("arm"), r.get("bin_md5"), r.get("snr3k"))
        groups.setdefault(key, []).append(r)
    out = []
    for (arm, md5, snr), g in sorted(
            groups.items(), key=lambda kv: (str(kv[0][0]), str(kv[0][2]))):
        comp = [r for r in g if r["completion_honest"]]
        out.append({
            "arm": arm, "bin_md5": md5, "snr3k": snr,
            "n": len(g),
            "n_connected": sum(1 for r in g if r.get("connected")),
            "n_completed_honest": len(comp),
            "n_completed_count_only": sum(
                1 for r in g if r["completion_count_only"]),
            "n_corrupt": sum(1 for r in g
                             if r.get("byte_integrity_ok") is False),
            "n_shear_suspected": sum(
                1 for r in g if r.get("content_shear_suspected")),
            "n_recon_fail": sum(1 for r in g if r.get("recon_ok") is False),
            "n_stored_xcheck_fail": sum(
                1 for r in g if r.get("stored_xcheck_ok") is False),
            "stored_xcheck_rel_err_max": max(
                (e for r in g
                 for e in (r.get("xcheck_keyed_rel_err"),
                           r.get("xcheck_duty_rel_err"),
                           r.get("xcheck_whole_rel_err"))
                 if e is not None), default=None),
            # medians, all cells + honest completers
            "keyed_user_rate_Bps_med": _med(
                [r["keyed_user_rate_Bps"] for r in g]),
            "keyed_user_rate_Bps_med_completers": _med(
                [r["keyed_user_rate_Bps"] for r in comp]),
            "whole_user_rate_Bps_med": _med(
                [r["whole_user_rate_Bps"] for r in g]),
            "whole_user_rate_Bps_med_completers": _med(
                [r["whole_user_rate_Bps"] for r in comp]),
            "whole_warm_field_Bps_med_completers": _med(
                [r["whole_warm_field_Bps"] for r in comp]),
            "keyed_field_Bps_med_completers": _med(
                [r["keyed_field_Bps"] for r in comp]),
            "duty_med": _med([r["duty"] for r in g]),
            "ramp_s_med": _med([r["ramp_s"] for r in g]),
            "vs_vara_whole_med": _med([r["vs_vara_whole"] for r in g]),
            "recon_rel_err_max": max(
                (r["recon_rel_err"] for r in g
                 if r["recon_rel_err"] is not None), default=None),
        })
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("inputs", nargs="+",
                    help="per-cell res JSON files and/or directories of them")
    ap.add_argument("--out", default=None, help="write reduced JSON here")
    ap.add_argument("--expect-n", type=int, default=None,
                    help="hard-gate: exact number of cells expected")
    ap.add_argument("--recon-tol", type=float, default=RECON_TOL_DEFAULT)
    ap.add_argument("--axis-override", choices=_AXIS_OVERRIDES, default=None,
                    help="admit a legitimate off-steady vs-bar cohort: "
                         "'peak-control' (a deliberate peak CONTROL arm) or "
                         "'hwcal' (a CAL-measured hardware row). Without it the "
                         "scoreboard refuses any non-steady / over-wide vs-bar "
                         "row.")
    ap.add_argument("--no-vs-bar", action="store_true",
                    help="reduce WITHOUT the vs-bar axis/width gate (the "
                         "non-vs-bar reduction path; output is byte-identical to "
                         "the legacy reducer). Use for capacity/floor reductions "
                         "not scored against the VARA bar.")
    args = ap.parse_args(argv)

    cells, files = load_cells(args.inputs)
    n_err = sum(1 for c in cells if "_load_error" in c)
    n_ok = len(cells) - n_err
    # A parser that matches nothing fabricates a clean answer - assert & print.
    print(f"[ra_reduce] parsed {n_ok} cell(s) from {len(files)} file(s) "
          f"({n_err} unreadable/non-cell)", file=sys.stderr)
    if n_ok == 0:
        print("[ra_reduce] FATAL: zero cells parsed - refusing to emit a "
              "reduction from nothing", file=sys.stderr)
        return 2
    if args.expect_n is not None and n_ok != args.expect_n:
        print(f"[ra_reduce] FATAL: expected {args.expect_n} cells, "
              f"parsed {n_ok}", file=sys.stderr)
        return 2

    # ---- AXIS + WIDTH GUARDRAIL: refuse to score a poisoned scoreboard ------
    # Any cell that carries a VARA bar is a vs-bar row. It may only be scored on
    # the honest (steady) P_sig axis at a non-perturbed cohort width; otherwise
    # the comparison sits at a different true coordinate than its label. Refuse
    # loudly (listing every offender) unless the operator opts out with
    # --no-vs-bar or admits a legitimate control / HW row with --axis-override.
    # Cells with no bar (non-vs-bar reductions) are never gated, so this path is
    # a no-op there and the emitted reduction is byte-identical to legacy.
    if not args.no_vs_bar:
        offenders = []
        for c in cells:
            if "_load_error" in c:
                continue
            if _f(c.get("vara_bar_Bmin")) is None:
                continue           # not a vs-bar row -> not gated
            reasons = _axis_width_offense(c, args.axis_override)
            if reasons:
                offenders.append((os.path.basename(c.get("_file") or ""),
                                  c.get("tag"), reasons))
        if offenders:
            print("[ra_reduce] FATAL: REFUSING vs-bar / scoreboard scoring -- "
                  "%d cell(s) are off the honest axis or over-wide:"
                  % len(offenders), file=sys.stderr)
            for fname, tag, reasons in offenders:
                print("  - %s (tag=%s): %s"
                      % (fname, tag, "; ".join(reasons)), file=sys.stderr)
            print("[ra_reduce] a vs-bar row is comparable to the VARA bar ONLY "
                  "on a steady P_sig axis at cohort width <= 8. Fix the cohort, "
                  "OR pass --axis-override {peak-control|hwcal} for a legitimate "
                  "control / HW-cal row, OR --no-vs-bar to reduce without the "
                  "bar comparison.", file=sys.stderr)
            return 2

    rows = [reduce_cell(c, recon_tol=args.recon_tol) for c in cells]
    valid_rows = [r for r in rows if not r.get("load_error")]
    recon_fails = [r for r in valid_rows if r.get("recon_ok") is False]
    xcheck_fails = [r for r in valid_rows
                    if r.get("stored_xcheck_ok") is False]
    disagreements = [r for r in valid_rows
                     if r.get("completion_field_disagrees")]
    reduced = {
        "reducer": "ra_reduce.py",
        "spec": "_research/CANONICAL_METRICS.md (2026-07-31)",
        "n_cells": n_ok,
        "n_load_errors": n_err,
        "recon_tol": args.recon_tol,
        "n_recon_fail": len(recon_fails),
        "recon_fail_tags": [r.get("tag") or r.get("file")
                            for r in recon_fails],
        "n_stored_xcheck_fail": len(xcheck_fails),
        "stored_xcheck_fail_tags": [r.get("tag") or r.get("file")
                                    for r in xcheck_fails],
        "n_completion_field_disagreements": len(disagreements),
        "groups": aggregate(valid_rows),
        "cells": rows,
    }
    text = json.dumps(reduced, indent=1)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
        print(f"[ra_reduce] wrote {args.out}", file=sys.stderr)
    else:
        print(text)
    if recon_fails:
        print(f"[ra_reduce] WARNING: {len(recon_fails)} cell(s) FAILED the "
              f"reconstruction law (factor composition != whole rate); their "
              f"decompositions may not be cited", file=sys.stderr)
    if xcheck_fails:
        print(f"[ra_reduce] WARNING: {len(xcheck_fails)} cell(s) FAILED the "
              f"stored-field cross-check (harness-stored rate field diverges "
              f"from raw recompute - suspected numerator swap, the "
              f"instrument-#13 class)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

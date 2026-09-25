#!/usr/bin/env python3
"""Publish-gate for the geometry-aware net-wire meter (G1 / landmine L5).

No wire A/B (SE-reclaim, pilot-thin, LEVER-F, big-block) may lean on a wire meter
until this gate is GREEN. It asserts, against mercury's OWN bit-exact output:

  T1  FIXTURE      net_bps_geom reproduces all four LEVER11 SE-reclaim grid points
                   (mercury rbc, hand-computed) to < 0.05%.
  T2  NO-REGRESS   at the reference geometry net_bps_geom == CONFIG_NET_BPS for
                   every table config (the meter is a no-op on stock runs).
  T3  CONTROL      the incumbent (FULL) and the SE-reclaim lever through the OLD
                   (blind) vs NEW (geometry) meter, CONFIRMING the L5 12-17%
                   deflation with numbers (or refuting it if it is not real).
  T4  KEYED-BYTES  keyed_wire_bytes_geom equals capstone_arms.keyed_wire_bytes on a
                   STOCK run (parity) and is +14.29% on a reclaim geometry.

Every assertion prints the ROW COUNT / DENOMINATOR it used (a parser that matched
nothing fabricates a clean pass). Exit 0 = geometry-correct. Nonzero = the meter is
lying or the L5 claim does not hold as stated.

Run: python tools/sim/realaudio/test_net_wire_meter.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import capstone_arms as ca          # noqa: E402
import net_wire_meter as nw         # noqa: E402

FAIL = []
def check(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAIL.append(msg)


def t1_fixture():
    print("=== T1 FIXTURE: model vs mercury bit-exact rbc (LEVER11 §1) ===")
    rows = nw.LEVER11_FIXTURE
    n = len(rows)
    check(n == 4, f"fixture row count = {n} (expected 4 mercury-generated grid points)")
    if n == 0:
        return
    worst = 0.0
    for label, ngi, dy, nsymb, rbc in rows:
        model = nw.net_bps_geom(nw.FIXTURE_CONFIG, ngi=ngi, nsymb=nsymb,
                                nsymb_ref=nw.FIXTURE_REF_NSYMB,
                                ngi_ref=nw.FIXTURE_REF_NGI)
        err = abs(model - rbc) / rbc * 100.0
        worst = max(worst, err)
        check(err < 0.05, f"{label}: mercury={rbc:.2f} model={model:.2f} err={err:.3f}% (<0.05%)")
    print(f"    worst-case fixture error = {worst:.3f}% over {n} rows")


def t2_no_regression():
    print("=== T2 NO-REGRESSION: net_bps_geom == CONFIG_NET_BPS at reference geometry ===")
    cfgs = sorted(ca.CONFIG_NET_BPS.keys())
    n = len(cfgs)
    check(n >= 17, f"table config count = {n} (expected >=17)")
    mism = 0
    for c in cfgs:
        stock = ca.config_net_bps(c)
        # reference geometry: ref Ngi, and nsymb==nsymb_ref so the pilot ratio is 1.
        geom = nw.net_bps_geom(c, ngi=nw.NGI_REF, nsymb=12, nsymb_ref=12)
        if abs(geom - stock) > 1e-6:
            mism += 1
            print(f"      MISMATCH cfg{c}: stock={stock} geom={geom}")
    check(mism == 0, f"all {n} configs identical at reference geometry ({mism} mismatch)")


def t3_control():
    print("=== T3 CONTROL: incumbent vs SE-reclaim, OLD (blind) vs NEW (geometry) meter ===")
    cfg = nw.FIXTURE_CONFIG
    # Incumbent = stock FULL cfg15 (54/3/12). Lever = RECLAIM-PILOTS (54/5/10).
    d_full = nw.geometry_deflation(cfg, ngi=54, nsymb=12, nsymb_ref=12)
    d_pil  = nw.geometry_deflation(cfg, ngi=54, nsymb=10, nsymb_ref=12)
    d_full_full = nw.geometry_deflation(cfg, ngi=18, nsymb=10, nsymb_ref=12)  # RECLAIM-FULL extreme
    print(f"    incumbent FULL      54/3/12 : old={d_full['old']} new={d_full['new']} "
          f"under_read={d_full['under_read_pct']}% recovered=+{d_full['recovered_pct']}%")
    print(f"    lever RECLAIM-PILOTS 54/5/10 : old={d_pil['old']} new={d_pil['new']} "
          f"under_read={d_pil['under_read_pct']}% recovered=+{d_pil['recovered_pct']}%")
    print(f"    lever RECLAIM-FULL  18/5/10 : old={d_full_full['old']} new={d_full_full['new']} "
          f"under_read={d_full_full['under_read_pct']}% recovered=+{d_full_full['recovered_pct']}%")
    # (a) NEW meter must not move the incumbent (no regression on stock).
    check(abs(d_full['new'] - d_full['old']) < 1e-6,
          f"incumbent unchanged by new meter (old={d_full['old']} new={d_full['new']})")
    # (b) The L5 claim: the blind meter deflates the pilots-only wire lever by 12-17%.
    check(12.0 <= d_pil['recovered_pct'] <= 17.0,
          f"L5 CONFIRMED: pilots-only recovered=+{d_pil['recovered_pct']}% in [12,17] band "
          f"(under_read {d_pil['under_read_pct']}%; denominators labelled)")
    # (c) full-reclaim upper extreme is materially larger (documents the range).
    check(d_full_full['recovered_pct'] > 20.0,
          f"RECLAIM-FULL recovered=+{d_full_full['recovered_pct']}% (>20%, the range top)")


def t4_keyed_bytes():
    print("=== T4 KEYED-BYTES: geometry-aware drop-in parity + reclaim uplift ===")
    # A synthetic PTT anatomy: 100 s of forward airtime keyed at cfg15.
    airtime = {15: 100.0}
    # STOCK run: geometry-aware must equal the blind keyed_wire_bytes exactly.
    blind_total, blind_per = ca.keyed_wire_bytes({"airtime_by_config": airtime})
    geom_total, geom_per = nw.keyed_wire_bytes_geom(airtime)  # no override => stock
    check(blind_total is not None and geom_total is not None,
          f"both meters returned a byte count (blind={blind_total}, geom={geom_total})")
    if blind_total and geom_total:
        check(abs(blind_total - geom_total) < 0.5,
              f"STOCK parity: blind={blind_total} B == geom={geom_total} B (100 s @ cfg15)")
    # RECLAIM-PILOTS geometry: same airtime carries +14.29% more net bytes.
    geom_r_total, _ = nw.keyed_wire_bytes_geom(
        airtime, {15: {"ngi": 54, "nsymb": 10, "nsymb_ref": 12}})
    if geom_total and geom_r_total:
        uplift = (geom_r_total - geom_total) / geom_total * 100.0
        check(13.9 <= uplift <= 14.7,
              f"reclaim uplift = +{uplift:.2f}% (~+14.29% expected; the wire the blind "
              f"meter would have hidden)")


def main():
    print("############ G1 net-wire meter publish-gate ############")
    t1_fixture()
    print()
    t2_no_regression()
    print()
    t3_control()
    print()
    t4_keyed_bytes()
    print()
    if FAIL:
        print(f"GATE RED -- {len(FAIL)} assertion(s) failed. Do NOT trust wire A/Bs:")
        for m in FAIL:
            print("   - " + m)
        return 1
    print("GATE GREEN -- geometry-aware net-wire meter is correct; L5 deflation confirmed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

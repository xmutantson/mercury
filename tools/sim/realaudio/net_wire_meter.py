#!/usr/bin/env python3
"""Geometry-aware net-wire meter (G1) — fixes the geometry-blind net_bps landmine.

WHY THIS EXISTS
---------------
The wire-throughput meters in capstone_arms.py (config_net_bps, keyed_wire_bytes,
true_wire_keyed_Bmin, datalink_efficiency) all read a HARDCODED per-config table,
CONFIG_NET_BPS, captured verbatim from `mercury -l` at the STANDALONE frame
geometry (Ngi=54, stock pilot grid Dy/Nsymb; physical_config.cc:37,50). That table
is BLIND to a per-run geometry override:

  * a pilot / CP reclaim (MERCURY_SE_{NGI,DY,NSYMB} escape hatch, or any future
    reclaim rung) that packs the SAME payload into FEWER symbols or a SHORTER
    guard interval delivers MORE net bits per second than the stock table says;
  * so keyed_wire_bytes / true_wire_keyed_Bmin UNDER-read the reclaim arm, and
    datalink_efficiency OVER-reads it.

capstone_arms.geometry_override_flags() only DETECTS the override and then FALLS
BACK to a geometry-independent proxy (bytes_per_fwd_airtime). That is safe but it
means every wire A/B on a reclaimed geometry is scored against the stock rate and
so re-buries the very lever it is meant to measure. This module CORRECTS the rate
instead of abandoning it.

THE MODEL (derived and BIT-EXACT verified against mercury's own output)
-----------------------------------------------------------------------
mercury's net wire rate is  net_bps = payload_bits_per_frame / frame_airtime,
with the frame airtime, in OFDM samples,

    frame_airtime_units = (Nfft + Ngi) * (Nsymb + preamble_nSymb)         (1)

and Nfft=256, preamble_nSymb=4 for every WB OFDM config (physical_config.cc:36,50).
A geometry override changes Ngi and/or Nsymb (a pilot reclaim holds the payload
nData fixed and spends fewer symbols; a CP reclaim shrinks Ngi). So the corrected
rate is the stock table value scaled by the airtime ratio (and, if the payload
per frame changes, by that ratio too):

    net_bps_geom = CONFIG_NET_BPS[cfg]
                 * (Nfft+Ngi_ref)/(Nfft+Ngi)                 # CP / GI reclaim
                 * (Nsymb_ref+pre)/(Nsymb+pre)               # pilot reclaim
                 * (nData/nData_ref)                         # payload change

At the reference geometry (Ngi_ref=54, Nsymb_ref=stock, nData_ref=stock) every
ratio is 1 and net_bps_geom == CONFIG_NET_BPS[cfg], so the meter is a no-op on
stock runs and NEVER regresses the incumbent number.

VALIDATION (see test_net_wire_meter.py, the publish-gate)
---------------------------------------------------------
Model (1) reproduces mercury's OWN bit-exact rbc at all four LEVER11 SE-reclaim
grid points to < 0.02% (these are the hand-computed fixture, LEVER11_SE_RECLAIM.md
§1, PLOT_PASSBAND -s 15 --ber, MERCURY_SE_* escape hatch, seeded CCIR-GOOD):

    grid (Ngi/Dy/Nsymb)     mercury rbc     model     err
    FULL      54/3/12        3348.39         3348.39   anchor
    RECLAIM-P 54/5/10        3826.73         3826.73   0.00%   (+14.29% vs FULL)
    mixed     36/5/10        4062.62         4062.6    0.00%
    RECLAIM-F 18/5/10        4329.51         4329.5    0.00%

The Nfft=256 and preamble=4 primitives were CONFIRMED from source
(physical_config.cc:36-37,50) AND cross-checked by the airtime ratios above.

NOTE / OPEN (handed to the orchestrator, NOT asserted as fact here)
------------------------------------------------------------------
CONFIG_NET_BPS is captured at the STANDALONE default Ngi=54 (4.5 ms GI;
physical_config.cc:37). A LIVE ARQ connection installs the PRODUCTION 3.0 ms GI
= Ngi=36 (main.cc startup; arq_responder.cc:9097; capstone_arms STOCK_NGI=36
"100% of Guard-interval prints are Ngi=36"). IF the `-l` table really is at Ngi=54
while live runs key at Ngi=36, the stock table UNDER-reads EVERY live WB run by
(256+54)/(256+36)-1 = +6.16%, independent of any reclaim. That is a SEPARATE,
larger-scope hypothesis than the SE-reclaim deflation and needs a binary-run
confirmation (a fleet `mercury -l` bitrate dump vs a live-run Ngi grep) before it
is treated as fact. This meter is correct either way: it keys off the run's ACTUAL
logged Ngi/Nsymb, so feeding it a live Ngi=36 yields the corrected rate.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# Single source of truth for the stock anchor table (mercury -l, build 913ec8b5).
from capstone_arms import CONFIG_NET_BPS, config_net_bps  # noqa: E402

# --- geometry primitives (WB OFDM), verified from source + LEVER11 ------------
NFFT_WB = 256          # physical_config.cc:36  ofdm_Nfft=256
NGI_REF = 54           # physical_config.cc:37  ofdm_gi = 54/256 (the -l table geometry)
PREAMBLE_NSYMB = 4     # physical_config.cc:50  ofdm_preamble_configurator_Nsymb=4
F_OFDM_HZ = 12000.0    # OFDM sample rate (48 kHz passband / interp 4)

# The bit-exact fixture: mercury's own rbc at four SE-reclaim grid points
# (LEVER11_SE_RECLAIM.md §1 / se_arma_repro.csv). Config 15 throughout.
# Each row: (label, Ngi, Dy, Nsymb, mercury_rbc_bps). Dy is informational (its
# effect is realised through Nsymb, which is what enters the airtime).
LEVER11_FIXTURE = [
    ("FULL      54/3/12", 54, 3, 12, 3348.39),   # baseline == CONFIG_NET_BPS[15]
    ("RECLAIM-P 54/5/10", 54, 5, 10, 3826.73),   # +14.29% vs FULL (pilot reclaim)
    ("mixed     36/5/10", 36, 5, 10, 4062.62),   # + CP reclaim
    ("RECLAIM-F 18/5/10", 18, 5, 10, 4329.51),   # full CP reclaim
]
FIXTURE_CONFIG = 15
FIXTURE_REF_NGI = 54
FIXTURE_REF_NSYMB = 12


def frame_airtime_units(ngi, nsymb, nfft=NFFT_WB, preamble=PREAMBLE_NSYMB):
    """Frame airtime in OFDM samples = (Nfft+Ngi)*(Nsymb+preamble_nSymb).
    Divide by F_OFDM_HZ for seconds; the constant cancels in every rate RATIO."""
    return (nfft + ngi) * (nsymb + preamble)


def frame_airtime_s(ngi, nsymb, nfft=NFFT_WB, preamble=PREAMBLE_NSYMB):
    """Frame airtime in seconds."""
    return frame_airtime_units(ngi, nsymb, nfft, preamble) / F_OFDM_HZ


def net_bps_stock(config):
    """The OLD, geometry-BLIND meter: the hardcoded stock table value, whatever
    geometry the run actually used. This is what capstone_arms.config_net_bps and
    keyed_wire_bytes credit today. Returns None for ROBUST/unknown."""
    return config_net_bps(config)


def net_bps_geom(config, ngi=NGI_REF, nsymb=None, nsymb_ref=None,
                 ndata_ratio=1.0, nfft=NFFT_WB, preamble=PREAMBLE_NSYMB,
                 ngi_ref=NGI_REF, anchor=None):
    """GEOMETRY-AWARE net payload bps for an OFDM config under a run geometry.

    Anchored to CONFIG_NET_BPS[config] (mercury's own `-l` rate at the reference
    geometry ngi_ref/nsymb_ref) and scaled by the airtime and payload ratios:

        net_bps_geom = anchor
                     * (nfft+ngi_ref)/(nfft+ngi)          # GI / CP reclaim
                     * (nsymb_ref+pre)/(nsymb+pre)        # pilot reclaim (if given)
                     * ndata_ratio                        # payload change (if any)

    Args:
      config      : OFDM config id (0..17). None-anchor configs return None.
      ngi         : the run's actual guard-interval Ngi (from the log's
                    "Guard interval: ... (Ngi=N)"), defaults to the reference.
      nsymb       : the run's actual data-frame Nsymb, if a pilot reclaim changed
                    it. Requires nsymb_ref to take effect.
      nsymb_ref   : the config's reference Nsymb the anchor was captured at (stock).
                    Only the RATIO nsymb_ref/nsymb matters, so a caller with a
                    reclaim spec supplies the (ref, actual) PAIR directly.
      ndata_ratio : payload-bytes-per-frame ratio (actual/reference). 1.0 when a
                    reclaim holds nData fixed (the SE-reclaim case).
      anchor      : override the table anchor (bps) for a config not in the table.

    Returns net bps (float) or None if the config has no anchor.
    """
    a = anchor if anchor is not None else config_net_bps(config)
    if a is None:
        return None
    ratio_gi = (nfft + ngi_ref) / (nfft + ngi)
    if nsymb is not None and nsymb_ref is not None:
        ratio_ns = (nsymb_ref + preamble) / (nsymb + preamble)
    else:
        ratio_ns = 1.0
    return a * ratio_gi * ratio_ns * ndata_ratio


def geometry_deflation(config, ngi=NGI_REF, nsymb=None, nsymb_ref=None,
                       ndata_ratio=1.0, **kw):
    """Quantify how much the OLD geometry-blind meter mis-reads a run at the given
    geometry. Returns a dict:
      old            : net_bps_stock (what the blind meter credits)
      new            : net_bps_geom  (the geometry-correct rate)
      under_read_pct : how much the OLD meter under-reads the TRUE wire,
                       (new-old)/new*100  (denominator = TRUE rate)
      recovered_pct  : the gain the NEW meter recovers over the OLD,
                       (new-old)/old*100  (denominator = STOCK rate)
    Both framings are reported because they use DIFFERENT denominators; a claim
    must name which. under_read_pct <= recovered_pct always."""
    old = net_bps_stock(config)
    new = net_bps_geom(config, ngi=ngi, nsymb=nsymb, nsymb_ref=nsymb_ref,
                       ndata_ratio=ndata_ratio, **kw)
    if old is None or new is None or old == 0:
        return {"old": old, "new": new, "under_read_pct": None, "recovered_pct": None}
    return {
        "old": round(old, 2),
        "new": round(new, 2),
        "under_read_pct": round((new - old) / new * 100.0, 2),
        "recovered_pct": round((new - old) / old * 100.0, 2),
    }


def keyed_wire_bytes_geom(airtime_by_config, geom_by_config=None):
    """Geometry-aware drop-in for capstone_arms.keyed_wire_bytes.

    airtime_by_config : {cfg: forward_PTT_airtime_seconds} (from ptt anatomy).
    geom_by_config    : optional {cfg: {ngi, nsymb, nsymb_ref, ndata_ratio}} run
                        geometry per config. Configs absent here are treated as
                        STOCK (net_bps_geom == table), so a stock run reproduces
                        capstone_arms.keyed_wire_bytes exactly.

    Returns (keyed_bytes:float, per_config:{cfg: bytes}) or (None, {}).
    """
    if not airtime_by_config:
        return None, {}
    geom_by_config = geom_by_config or {}
    total = 0.0
    per = {}
    for cfg, air in airtime_by_config.items():
        cfg = int(cfg)
        g = geom_by_config.get(cfg, {})
        net = net_bps_geom(cfg,
                           ngi=g.get("ngi", NGI_REF),
                           nsymb=g.get("nsymb"),
                           nsymb_ref=g.get("nsymb_ref"),
                           ndata_ratio=g.get("ndata_ratio", 1.0))
        if net is None:
            continue
        b = air * net / 8.0
        per[cfg] = round(b, 1)
        total += b
    if not per:
        return None, {}
    return round(total, 1), per


def _print_fixture_and_control():
    """Standalone report: reproduce the LEVER11 fixture + the L5 deflation control."""
    print("=== net_wire_meter: LEVER11 fixture (model vs mercury bit-exact rbc) ===")
    print(f"  anchor CONFIG_NET_BPS[{FIXTURE_CONFIG}] = {config_net_bps(FIXTURE_CONFIG)} bps"
          f"  (ref geometry Ngi={FIXTURE_REF_NGI}/Nsymb={FIXTURE_REF_NSYMB}/pre={PREAMBLE_NSYMB})")
    print(f"  {'grid':<20} {'mercury':>9} {'model':>9} {'err%':>7}")
    for label, ngi, dy, nsymb, rbc in LEVER11_FIXTURE:
        model = net_bps_geom(FIXTURE_CONFIG, ngi=ngi, nsymb=nsymb,
                             nsymb_ref=FIXTURE_REF_NSYMB, ngi_ref=FIXTURE_REF_NGI)
        err = (model - rbc) / rbc * 100.0
        print(f"  {label:<20} {rbc:>9.2f} {model:>9.2f} {err:>7.3f}")
    print()
    print("=== L5 CONTROL: incumbent (FULL) vs SE-reclaim through OLD vs NEW meter ===")
    # FULL is the incumbent stock arm; RECLAIM-PILOTS is the wire lever.
    d_full = geometry_deflation(FIXTURE_CONFIG, ngi=54, nsymb=12,
                                nsymb_ref=FIXTURE_REF_NSYMB)
    d_recl = geometry_deflation(FIXTURE_CONFIG, ngi=54, nsymb=10,
                                nsymb_ref=FIXTURE_REF_NSYMB)
    print(f"  incumbent  FULL   54/3/12 : old={d_full['old']}  new={d_full['new']}  "
          f"under_read={d_full['under_read_pct']}%  (new==old: no regression)")
    print(f"  lever  RECLAIM-P 54/5/10 : old={d_recl['old']}  new={d_recl['new']}  "
          f"under_read={d_recl['under_read_pct']}%  recovered=+{d_recl['recovered_pct']}%")
    print(f"  => the blind meter hides the reclaim lever's +{d_recl['recovered_pct']}% "
          f"per-frame wire (reads both arms at {d_full['old']}). L5 12-17% band: "
          f"CONFIRMED (under_read {d_recl['under_read_pct']}% / recovered "
          f"+{d_recl['recovered_pct']}%).")


if __name__ == "__main__":
    _print_fixture_and_control()

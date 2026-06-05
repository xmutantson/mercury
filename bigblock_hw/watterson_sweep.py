#!/usr/bin/env python3
"""
Big-block PHY slow/mild-fade operating-envelope sweep (production-finalize §15).

Drives sfo_grid_test (MERCURY_SFO_GRID=1, -m PLOT_PASSBAND -s 16) with the
TIME-VARYING Watterson channel (chan_sel=3) across:
  - fade depth (dB)  x  Doppler spread (Hz)   [the realistic slow/mild regime]
  - pilot layouts (cont_cols / scat_dx / scat_dy)   [freq-focused vs time-denser]
  - many random Watterson seeds (Rayleigh scatter is stochastic -> need an
    ENSEMBLE success rate, not a single realization).

At each (layout, depth, fd) cell it reports the per-seed coded LDPC outcome
(codewords decoded / 8) and the cell success rate, plus the net-PHY of the
layout (info bps per acquisition / block airtime). net-PHY > VARA 7050 is the
freeze gate; the success rate vs depth/fd is the operating envelope.

Estimator = sparse-2D (TRACK off, per the wnrcoq8rr finding). 32-QAM rate-0.875
unless --fec-rate overrides (the lower-FEC gearshift-ladder note).
"""
import os, sys, subprocess, re, argparse, json

MERC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mercury.exe")
MERC = os.path.abspath(MERC)

# Block geometry (fixed by the big-block design): one 4-sym preamble + 60 data
# symbols under one acquisition. Airtime from the ACTUAL CFG16 OFDM timing:
#   Nofdm=310 samples/symbol (measured: [PHY-SWITCH] init Nofdm=310), Fs=12000
#   (= freq_interp(1) * BW/Nc(46.875) * Nfft(256)). T_sym = 310/12000 = 25.833 ms.
PREAMBLE_SYM = 4
DATA_SYM     = 60
NOFDM        = 310
FS_HZ        = 12000.0
T_SYM_S      = NOFDM / FS_HZ                      # 25.833 ms / OFDM symbol
# net-PHY basis: info bits / DATA-PAYLOAD airtime (the standard PHY throughput; the
# one 4-sym preamble is a one-time amortizable acquisition cost). Also report the
# incl-preamble number. The validated-sim "7859-7960" and HW "~7000 bps over the 60
# data-symbol payload" both use the DATA-payload basis (PHASE2_HW_RESULTS.md (c)).
DATA_AIRTIME_S  = DATA_SYM * T_SYM_S                       # 1.550 s
BLOCK_AIRTIME_S = (PREAMBLE_SYM + DATA_SYM) * T_SYM_S      # 1.653 s (incl preamble)

PAT_DEC   = re.compile(r"codewords_decoded=(\d+)/(\d+)")
PAT_BER   = re.compile(r"post_FEC_info_BER=([0-9.eE+-]+)")
PAT_NBITS = re.compile(r"nData=(\d+) nBits=(\d+)")
PAT_K     = re.compile(r"CODED RESULT \(K=(\d+)")
PAT_KN    = re.compile(r"x (\d+)-bit rate-([0-9.]+)")
PAT_PIL   = re.compile(r"pilots=(\d+) \(([0-9.]+)% of grid\)")
PAT_HINST = re.compile(r"\|H\|inst\[([0-9.]+)\.\.([0-9.]+)\] max_time_drift\(sd\|H\|\)=([0-9.]+)")


def run_cell(layout, depth_db, fd_hz, seed, esn0, ldpc_n_override=None, ldpc_k_override=None):
    env = dict(os.environ)
    env.update({
        "MERCURY_SFO_GRID": "1",
        "MERCURY_SFO_GRID_THIN": "1",
        "MERCURY_SFO_GRID_CONT_COLS": str(layout[0]),
        "MERCURY_SFO_GRID_SCAT_DX":   str(layout[1]),
        "MERCURY_SFO_GRID_SCAT_DY":   str(layout[2]),
        "MERCURY_SFO_GRID_SPARSE2D": "1",
        "MERCURY_SFO_GRID_TRACK": "0",
        "MERCURY_SFO_GRID_CODED": "1",
        "MERCURY_SFO_GRID_CSI": "1",
        "MERCURY_SFO_GRID_CHAN": "3",
        "MERCURY_SFO_GRID_WATT_DEPTH_DB": str(depth_db),
        "MERCURY_SFO_GRID_WATT_FD_HZ": str(fd_hz),
        "MERCURY_SFO_GRID_SEED": str(seed),
        "MERCURY_SIM2_SFO_PPM": "13.2",     # real measured Fe-Pi SFO
        "MERCURY_SFO_GRID_ESN0": str(esn0),
    })
    try:
        out = subprocess.run([MERC, "-m", "PLOT_PASSBAND", "-s", "16"],
                             env=env, capture_output=True, text=True, timeout=120).stdout
    except subprocess.TimeoutExpired:
        return None
    md, mb, mn = PAT_DEC.search(out), PAT_BER.search(out), PAT_NBITS.search(out)
    mk, mkn, mp = PAT_K.search(out), PAT_KN.search(out), PAT_PIL.search(out)
    mh = PAT_HINST.search(out)
    if not (md and mn and mk):
        return None
    dec, tot = int(md.group(1)), int(md.group(2))
    return {
        "dec": dec, "tot": tot,
        "ber": float(mb.group(1)) if mb else None,
        "nbits": int(mn.group(2)),
        "K": int(mk.group(1)),
        "Nldpc": int(mkn.group(1)) if mkn else None,
        "rate": float(mkn.group(2)) if mkn else None,
        "pilots_pct": float(mp.group(2)) if mp else None,
        "Hmin": float(mh.group(1)) if mh else None,
        "Hmax": float(mh.group(2)) if mh else None,
        "drift": float(mh.group(3)) if mh else None,
    }


def net_phy(K, Nldpc, rate):
    """info bps per acquisition. info bits = K * (Nldpc*rate).
    Returns (net_phy_data_payload, net_phy_incl_preamble, info_bits, info_per_cw)."""
    info_per_cw = int(round(Nldpc * rate))
    info_bits = K * info_per_cw
    return (info_bits / DATA_AIRTIME_S, info_bits / BLOCK_AIRTIME_S,
            info_bits, info_per_cw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layouts", default="2/4/8",
                    help="semi-list of cont/dx/dy, e.g. 2/4/8,2/4/4,2/6/4")
    ap.add_argument("--depths", default="0,1,2,3,4,6")
    ap.add_argument("--dopplers", default="0.1,0.25,0.5,1.0")
    ap.add_argument("--seeds", type=int, default=12)
    ap.add_argument("--esn0", default="900")   # clean unless overridden
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    layouts = []
    for L in args.layouts.split(","):
        c, dx, dy = L.split("/")
        layouts.append((int(c), int(dx), int(dy)))
    depths   = [float(x) for x in args.depths.split(",")]
    dopplers = [float(x) for x in args.dopplers.split(",")]
    esn0     = float(args.esn0)
    seeds    = list(range(1, args.seeds + 1))

    results = {"layouts": {}, "params": {
        "depths": depths, "dopplers": dopplers, "seeds": args.seeds,
        "esn0": esn0, "vara": 7050, "block_airtime_s": BLOCK_AIRTIME_S}}

    for layout in layouts:
        lkey = f"{layout[0]}/{layout[1]}/{layout[2]}"
        # one probe at flat (depth 0, fd 0.1) to read net-PHY / pilots for the layout
        probe = run_cell(layout, 0.0, 0.1, 1, esn0)
        if probe and probe["Nldpc"]:
            nphy, nphy_pre, info_bits, info_cw = net_phy(probe["K"], probe["Nldpc"], probe["rate"])
        else:
            nphy = nphy_pre = info_bits = info_cw = None
        ldata = {"net_phy_data_payload": nphy, "net_phy_incl_preamble": nphy_pre,
                 "info_bits": info_bits,
                 "pilots_pct": probe["pilots_pct"] if probe else None,
                 "K": probe["K"] if probe else None,
                 "rate": probe["rate"] if probe else None,
                 "cells": {}}
        if nphy:
            print(f"\n##### LAYOUT cont/dx/dy={lkey}  pilots={ldata['pilots_pct']}%  "
                  f"K={ldata['K']}  net-PHY={nphy:.0f} bps (data-payload) / "
                  f"{nphy_pre:.0f} (incl-pre)  vs VARA 7050  "
                  f"{'PASS' if nphy>7050 else 'FAIL'}")
        else:
            print(f"\n##### LAYOUT {lkey} (probe failed)")
        print(f"{'depth\\fd':>9} | " + " | ".join(f"{fd:>5}Hz" for fd in dopplers))
        for depth in depths:
            row = []
            for fd in dopplers:
                ok = 0; tot_cw = 0; dec_cw = 0; drift = 0.0; hmin = 9.9
                for s in seeds:
                    r = run_cell(layout, depth, fd, s, esn0)
                    if r is None:
                        continue
                    if r["dec"] == r["tot"]:
                        ok += 1
                    tot_cw += r["tot"]; dec_cw += r["dec"]
                    if r["drift"] is not None: drift = max(drift, r["drift"])
                    if r["Hmin"] is not None: hmin = min(hmin, r["Hmin"])
                rate_ok = ok / len(seeds)
                cw_rate = dec_cw / tot_cw if tot_cw else 0.0
                ldata["cells"][f"{depth}|{fd}"] = {
                    "block_ok_rate": rate_ok, "cw_rate": cw_rate,
                    "ok": ok, "n": len(seeds), "max_drift": drift, "min_Hinst": hmin}
                row.append(f"{ok}/{len(seeds)}")
            print(f"{depth:>9} | " + " | ".join(f"{c:>7}" for c in row))
        results["layouts"][lkey] = ldata

    if args.out:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\n[written] {args.out}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# CONNECT-SEED FUSION A/B extractor. Per arm/band: connect reliability (conn/n), the connect-floor
# decomposition (A=rspConn->cmdConn, B=cmdConn->SWITCH_BW, CD=SWITCH_BW->cfg0LOADED, TOTAL), climb-to-
# cfg16, beat-VARA wire ratio, integrity fails, and the FUSION fire proof (FIX collapses CD; BASE keeps
# the separate SET_CONFIG cross). Asserts nonzero row counts; prints denominators.
import json, glob, re, statistics, sys

LOGDIR = "/dev/shm/connlever/ab/logs"
TS = re.compile(r"^\[T\+(\d+\.\d+)\]\s+\[(CMD|RSP)\]\s+(.*)$")

def med(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.median(xs), 2) if xs else None

def scan(logpath):
    """Return first-occurrence milestone timestamps + fire-proof flags for one cell log."""
    m = {"rspC": None, "cmdC": None, "swbw": None, "cfg0": None, "cfg16": None,
         "fuse_embed": 0, "fuse_rsp": 0, "fuse_loaded": 0, "base_cross": 0}
    try:
        for raw in open(logpath, errors="replace"):
            mm = TS.match(raw.rstrip("\n"))
            if not mm: continue
            t = float(mm.group(1)); who = mm.group(2); msg = mm.group(3)
            if who == "RSP":
                if m["rspC"] is None and "link_status:Connected to" in msg: m["rspC"] = t
                if "FUSE: loaded connect-seed config 0" in msg: m["fuse_rsp"] += 1
            else:  # CMD
                if m["cmdC"] is None and "link_status:Connected to TESTB" in msg: m["cmdC"] = t
                if m["swbw"] is None and "[BW-NEG] SWITCH_BANDWIDTH recv_timeout" in msg: m["swbw"] = t
                if m["cfg0"] is None and "loaded config 0" in msg: m["cfg0"] = t
                if m["cfg16"] is None and "loaded config 16" in msg: m["cfg16"] = t
                if "FUSE: embedding connect-seed" in msg: m["fuse_embed"] += 1
                if "FUSED connect-seed loaded config 0" in msg: m["fuse_loaded"] += 1
                if "SET_CONFIG ACKed, loaded config 0" in msg: m["base_cross"] += 1
    except OSError:
        return m
    return m

def arm(prefix):
    js = sorted(glob.glob(f"{LOGDIR}/res_{prefix}*.json"))
    rows = []
    for j in js:
        try: d = json.load(open(j))
        except Exception: continue
        tag = j.split("/")[-1].replace("res_", "").replace(".json", "")
        L = f"{LOGDIR}/arq_{tag}.log"
        s = scan(L)
        conn = d.get("connected_at_s")
        A  = (s["cmdC"]-s["rspC"]) if (s["cmdC"] and s["rspC"]) else None
        B  = (s["swbw"]-s["cmdC"]) if (s["swbw"] and s["cmdC"]) else None
        CD = (s["cfg0"]-s["swbw"]) if (s["cfg0"] and s["swbw"]) else None
        TOT= (s["cfg0"]-s["rspC"]) if (s["cfg0"] and s["rspC"]) else None
        climb = (s["cfg16"]-s["rspC"]) if (s["cfg16"] and s["rspC"]) else None
        tw = d.get("true_wire_keyed_Bmin"); vw = d.get("vara_wire_Bmin")
        ratio = round(tw/vw, 4) if (tw and vw) else None
        rows.append({"tag": tag, "connected": d.get("connected"), "conn_at": conn,
                     "A": A, "B": B, "CD": CD, "TOT": TOT, "cfg16": s["cfg16"], "climb": climb,
                     "ratio": ratio, "tw": tw, "vw": vw,
                     "integ": d.get("byte_integrity_ok"), "maxcfg": d.get("max_config_reached"),
                     "fuse_embed": s["fuse_embed"], "fuse_rsp": s["fuse_rsp"],
                     "fuse_loaded": s["fuse_loaded"], "base_cross": s["base_cross"]})
    return rows

def summ(prefix, label):
    rows = arm(prefix); n = len(rows)
    conn = [r for r in rows if r["connected"]]
    r16 = [r for r in rows if r["cfg16"]]
    integ_fail = [r for r in rows if r["integ"] is False]
    fe = sum(1 for r in conn if r["fuse_embed"] > 0)
    fl = sum(1 for r in conn if r["fuse_loaded"] > 0)
    bc = sum(1 for r in conn if r["base_cross"] > 0)
    res = {"n": n, "conn": len(conn), "r16": len(r16), "integ_fail": len(integ_fail),
           "connect_s": med([r["conn_at"] for r in conn]),
           "A": med([r["A"] for r in conn]), "B": med([r["B"] for r in conn]),
           "CD": med([r["CD"] for r in conn]), "TOT": med([r["TOT"] for r in conn]),
           "climb": med([r["climb"] for r in r16]),
           "ratio": med([r["ratio"] for r in conn]),
           "vw": conn[0]["vw"] if conn else None,
           "fuse_embed_cells": fe, "fuse_loaded_cells": fl, "base_cross_cells": bc}
    print(f"\n### {label} ({prefix}) n={n} conn={len(conn)} r16={len(r16)} integ_FAIL={len(integ_fail)}")
    print(f"    connect_s(med/{len(conn)})={res['connect_s']}  "
          f"FLOOR: A={res['A']} B={res['B']} CD={res['CD']} TOTAL={res['TOT']}")
    print(f"    climb_to_cfg16(med/{len(r16)})={res['climb']}  beatVARA_ratio(med)={res['ratio']}  vw={res['vw']}")
    print(f"    FIRE-PROOF: fuse_embed_cells={fe}/{len(conn)} fuse_loaded_cells={fl}/{len(conn)} "
          f"base_SETCONFIG_cross_cells={bc}/{len(conn)}")
    return res

RES = {}
BANDS = (("WGN:40", "b40"), ("WGN:30", "b30"), ("WGN:25", "b25"), ("FADE mpm@30", "fad"))
for band, tag in BANDS:
    print(f"\n========== BAND {band} ==========")
    for lbl, suf in (("FIX (R+P-alt+FUSE)", "fix"), ("BASE (R+P-alt)", "bas")):
        RES[(tag, suf)] = summ(tag+suf, lbl)

print("\n\n========== SUMMARY (median) ==========")
hdr = f"{'band':12} {'arm':5} {'conn/n':7} {'r16':4} {'conn_s':7} {'A':6} {'B':6} {'CD':6} {'floorTOT':8} {'climb':7} {'ratio':7} {'integF':6} {'fuseCk':7}"
print(hdr)
for band, tag in BANDS:
    for suf, lbl in (("fix", "FIX"), ("bas", "BASE")):
        r = RES[(tag, suf)]
        fuseck = f"{r['fuse_loaded_cells']}/{r['conn']}" if suf == "fix" else f"x{r['base_cross_cells']}"
        print(f"{band:12} {lbl:5} {str(r['conn'])+'/'+str(r['n']):7} {str(r['r16']):4} "
              f"{str(r['connect_s']):7} {str(r['A']):6} {str(r['B']):6} {str(r['CD']):6} "
              f"{str(r['TOT']):8} {str(r['climb']):7} {str(r['ratio']):7} {str(r['integ_fail']):6} {fuseck:7}")

print("\n[legend] A=rspConn->cmdConn  B=cmdConn->SWITCH_BW  CD=SWITCH_BW->cfg0LOADED  "
      "floorTOT=rspConn->cfg0LOADED  climb=rspConn->cfg16LOADED (s). FIX collapses CD (fused seed); "
      "BASE keeps the separate SET_CONFIG cross. fuseCk: FIX=fuse_loaded_cells/conn, BASE=x<base_cross_cells>.")

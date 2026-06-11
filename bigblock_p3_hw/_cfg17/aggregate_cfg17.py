#!/usr/bin/env python3
# Aggregate CFG17 fleet sweep CSVs -> per-channel, per-arm decode-fraction vs EsN0.
# Input: concatenated CSV lines "channel,esn0,seed,arm,ok,tot" (DONE lines ignored).
# Output: per-channel waterfall table + the headline JSON fields.
import sys, json, collections, glob, os

rows = []
for path in sys.argv[1:]:
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("DONE") or ln.startswith("==="):
                continue
            p = ln.split(",")
            if len(p) != 6:
                continue
            ch, esn0, seed, arm, ok, tot = p
            if ok == "ERR":
                continue
            try:
                rows.append((ch, float(esn0), int(seed), arm, int(ok), int(tot)))
            except ValueError:
                continue

# agg[(ch,esn0,arm)] = (sum_ok, sum_tot, nseeds, full_decode_seeds)
agg = collections.defaultdict(lambda: [0, 0, 0, 0])
for ch, e, s, arm, ok, tot in rows:
    a = agg[(ch, e, arm)]
    a[0] += ok; a[1] += tot; a[2] += 1
    if ok == tot and tot > 0:
        a[3] += 1

channels = sorted(set(k[0] for k in agg))
arms = ["uniform", "pasls", "stack", "stacknvfix", "genie"]

out = {"channels": {}}
for ch in channels:
    esn0s = sorted(set(k[1] for k in agg if k[0] == ch))
    chrec = {"esn0_rows": [], "waterfall_dB": {}}
    for e in esn0s:
        row = {"esn0": e}
        for arm in arms:
            if (ch, e, arm) in agg:
                so, st, ns, fd = agg[(ch, e, arm)]
                row[arm] = {"frac": round(so / st, 3) if st else None,
                            "full_seeds": f"{fd}/{ns}"}
        chrec["esn0_rows"].append(row)
    # waterfall = lowest EsN0 where decode-fraction >= 0.99 (essentially all CWs)
    for arm in arms:
        wf = None
        for e in esn0s:
            if (ch, e, arm) in agg:
                so, st, ns, fd = agg[(ch, e, arm)]
                if st and so / st >= 0.99:
                    wf = e; break
        if any((ch, e, arm) in agg for e in esn0s):
            chrec["waterfall_dB"][arm] = wf
    out["channels"][ch] = chrec

print(json.dumps(out, indent=2))

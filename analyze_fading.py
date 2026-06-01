#!/usr/bin/env python3
# Summarize the baud-scaling fading sweep: AWGN vs fading cliff per baud, the
# per-2x scaling under each channel, and the deepest usable rung under fading.
import csv, sys, os

import glob
_cand = [r"C:\Users\kamer\AppData\Local\Temp\fading_results\cliffs_fast.csv",
         r"C:\Users\kamer\AppData\Local\Temp\fading_results\cliffs.csv",
         "/tmp/fading_results/cliffs_fast.csv", "/tmp/fading_results/cliffs.csv"]
CSV = next((p for p in _cand if os.path.exists(p)), _cand[0])
rows = []
with open(CSV) as f:
    for r in csv.DictReader(f):
        try: r["cliff_dB"] = float(r["cliff_dB"])
        except: r["cliff_dB"] = None
        r["K"] = int(r["K"]); r["Nfft"] = int(r["Nfft"])
        rows.append(r)

Ks = sorted(set(r["K"] for r in rows))
chans = ["awgn","moderate","poor","pooredge"]
clab = {"awgn":"AWGN","moderate":"moderate 0.5Hz/1ms","poor":"poor 1Hz/2ms","pooredge":"edge 2Hz/2ms"}

def cliff(K,ch):
    for r in rows:
        if r["K"]==K and r["channel"]==ch: return r["cliff_dB"]
    return None

print("="*86)
print("BAUD-SCALING CLIFF (SNR3k dB, deeper=better): AWGN vs FADING")
print("="*86)
hdr = f"{'K':>2}{'Nfft':>6}{'T_fft':>8}"
for ch in chans: hdr += f"{clab[ch]:>20}"
print(hdr)
Tfft = {1:21.3,2:42.7,4:85.3,8:170.7}
for K in Ks:
    line = f"{K:>2}{256*K:>6}{Tfft.get(K,0):>7.1f}m"
    for ch in chans:
        c = cliff(K,ch)
        line += f"{('NA' if c is None else f'{c:.0f}'):>20}"
    print(line)
print()

print("Fading PENALTY vs AWGN (dB lost; 0 = fading-immune, negative = cliff backed off):")
hdr = f"{'K':>2}"
for ch in chans[1:]: hdr += f"{clab[ch]:>20}"
print(hdr)
for K in Ks:
    base = cliff(K,"awgn")
    line = f"{K:>2}"
    for ch in chans[1:]:
        c = cliff(K,ch)
        if base is None or c is None: line += f"{'NA':>20}"
        else:
            pen = c - base   # c and base are negative dB; if fading cliff is higher (less negative) => positive penalty (lost depth)
            line += f"{pen:+.0f} dB{'':>14}" if False else f"{(f'{pen:+.0f}'):>20}"
    print(line)
print("  (penalty = fading_cliff - awgn_cliff; +N = lost N dB of depth to fading)")
print()

print("Per-2x baud scaling (dB deepened per doubling) under each channel:")
print("  AWGN target (P0): ~+3.5 dB/2x. If fading holds, fading curve ~parallel.")
for ch in chans:
    seq = [(K,cliff(K,ch)) for K in Ks if cliff(K,ch) is not None]
    deltas = []
    for i in range(1,len(seq)):
        d = seq[i-1][1] - seq[i][1]   # deeper => more negative => positive gain
        deltas.append(f"{seq[i-1][0]}->{seq[i][0]}:{d:+.0f}")
    mean = None
    if len(seq)>=2:
        mean = (seq[0][1]-seq[-1][1])/ (len(seq)-1)
    print(f"  {clab[ch]:<22} cliffs={[c for _,c in seq]}  steps=[{', '.join(deltas)}]  mean/2x={'NA' if mean is None else f'{mean:+.1f} dB'}")
print()

print("DEEPEST USABLE RUNG under fading (most-negative cliff that still holds):")
for ch in chans:
    best = None
    for K in Ks:
        c = cliff(K,ch)
        if c is not None and (best is None or c < best[1]): best = (K,c)
    if best: print(f"  {clab[ch]:<22} K={best[0]} (Nfft={256*best[0]}) @ {best[1]:.0f} dB SNR3k")

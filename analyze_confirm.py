#!/usr/bin/env python3
# High-frame (40-frame) confirmation analysis: per (K, channel), find the cliff =
# deepest SNR with BER below a reliability threshold (FEC waterfall: BER<1e-3 ~ the
# point past which LDPC fully recovers). Reports the BER waterfall and the
# high-confidence cliff, so we can separate true K=8 saturation from 3-frame noise.
import csv, os

_cand = [r"C:\Users\kamer\AppData\Local\Temp\fading_confirm\confirm.csv",
         "/tmp/fading_confirm/confirm.csv"]
CSV = next((p for p in _cand if os.path.exists(p)), _cand[0])

rows=[]
with open(CSV) as f:
    for r in csv.DictReader(f):
        try:
            r["K"]=int(r["K"]); r["esn0"]=float(r["esn0_dB"]); r["BER"]=float(r["BER"])
        except: continue
        rows.append(r)

Ks=sorted(set(r["K"] for r in rows))
chans=["awgn","poor","pooredge"]
clab={"awgn":"AWGN","poor":"poor 1Hz/2ms","pooredge":"edge 2Hz/2ms"}
THR=1e-3   # reliability threshold (FEC waterfall knee)

def series(K,ch):
    s=[(r["esn0"],r["BER"]) for r in rows if r["K"]==K and r["channel"]==ch]
    return sorted(s)

print("="*78)
print(f"HIGH-FRAME (40fr) BER WATERFALL + cliff @ BER<{THR:g}")
print("="*78)
for K in Ks:
    print(f"\n--- K={K} (Nfft={256*K}) ---")
    for ch in chans:
        s=series(K,ch)
        if not s: continue
        wf=" ".join(f"{e:.0f}:{b:.1e}" for e,b in s)
        # cliff = deepest esn0 with BER<THR AND all-higher-esn0 also <THR (monotone-ish)
        cliff=None
        for i,(e,b) in enumerate(s):           # ascending esn0
            if b<THR and all(bb<THR for _,bb in s[i:]):
                cliff=e; break
        print(f"  {clab[ch]:<14} cliff={('NA' if cliff is None else f'{cliff:.0f}')+' dB':<8}  [{wf}]")

print("\n"+"="*78)
print("AWGN vs FADING cliff (high-frame) + penalty")
print("="*78)
def cliff_of(K,ch):
    s=series(K,ch)
    for i,(e,b) in enumerate(s):
        if b<THR and all(bb<THR for _,bb in s[i:]): return e
    return None
print(f"{'K':>2}{'AWGN':>8}{'poor':>8}{'edge':>8}{'  poor pen':>11}{'  edge pen':>11}")
for K in Ks:
    a=cliff_of(K,"awgn"); p=cliff_of(K,"poor"); e=cliff_of(K,"pooredge")
    pp = "NA" if (a is None or p is None) else f"{p-a:+.0f}"
    ep = "NA" if (a is None or e is None) else f"{e-a:+.0f}"
    print(f"{K:>2}{(str(int(a)) if a is not None else 'NA'):>8}{(str(int(p)) if p is not None else 'NA'):>8}{(str(int(e)) if e is not None else 'NA'):>8}{pp:>11}{ep:>11}")

# per-2x scaling (AWGN + poor) from high-frame cliffs
print("\nPer-2x scaling (high-frame cliffs):")
for ch in ["awgn","poor"]:
    seq=[(K,cliff_of(K,ch)) for K in Ks if cliff_of(K,ch) is not None]
    if len(seq)>=2:
        steps=[f"{seq[i-1][0]}->{seq[i][0]}:{seq[i-1][1]-seq[i][1]:+.0f}" for i in range(1,len(seq))]
        mean=(seq[0][1]-seq[-1][1])/(len(seq)-1)
        print(f"  {clab[ch]:<14} {[c for _,c in seq]}  steps=[{', '.join(steps)}]  mean/2x={mean:+.1f} dB")

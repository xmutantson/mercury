#!/usr/bin/env python3
"""Measure real Fe-Pi clock SFO (ppm) from a HW recording of the big-block.

Method (the task's "fit the preamble/pilot drift across the block -> ppm"):
The TX wav is a KNOWN, fixed-length passband block (the 60-symbol data + 4-sym
preamble at 48 kHz, bracketed by 0.25 s silence). When RPi1 plays it and RPi2
records it, the recovered block's DURATION is scaled by exactly the ratio of the
two independent 48 kHz crystals:  T_rx / T_tx = f_tx / f_rx = 1 + SFO.
We recover the block span by cross-correlating the recording against the known TX
passband signal (a matched filter) at the head and again near the tail, and from
the spacing between the two correlation peaks vs the known TX spacing we get the
sample-rate ratio -> ppm. We also report a coarse energy-span estimate.
"""
import sys, wave
import numpy as np

def load(p):
    w = wave.open(p,'rb'); n=w.getnframes(); sr=w.getframerate(); ch=w.getnchannels()
    x = np.frombuffer(w.readframes(n), dtype='<i2').astype(np.float64); w.close()
    if ch>1: x=x[::ch]
    return x, sr

def bandpass_energy(x, sr, f0=1500, bw=2400):
    # crude: rms in sliding windows after removing DC
    return x - np.mean(x)

def matched_peak(rx, tmpl, lo, hi):
    """Return the lag (within [lo,hi]) maximizing normalized xcorr of tmpl into rx."""
    best_lag=lo; best=-1e18
    tn = tmpl/ (np.linalg.norm(tmpl)+1e-9)
    for lag in range(lo, hi):
        seg = rx[lag:lag+len(tmpl)]
        if len(seg)<len(tmpl): break
        s = seg/(np.linalg.norm(seg)+1e-9)
        c = abs(np.dot(s, tn))
        if c>best: best=c; best_lag=lag
    return best_lag, best

def main():
    tx_path, rx_path = sys.argv[1], sys.argv[2]
    tx, sr = load(tx_path)
    rx, sr2 = load(rx_path)
    tx = tx - np.mean(tx); rx = rx - np.mean(rx)
    # Locate TX block content (energy>15% of TX peak) — gives TX head/tail anchors.
    win=512
    txenv = np.array([np.sqrt(np.mean(tx[i:i+win]**2)) for i in range(0,len(tx)-win,win)])
    thr = txenv.max()*0.15
    on = np.where(txenv>thr)[0]
    tx_head = on[0]*win; tx_tail = (on[-1]+1)*win
    # Template snippets from TX: first 4096 samp of block (preamble), last 4096 of block.
    head_tmpl = tx[tx_head:tx_head+4096]
    tail_tmpl = tx[tx_tail-4096:tx_tail]
    tx_span = (tx_tail-4096) - tx_head   # distance between head-anchor and tail-anchor in TX
    # Find head template in rx (search whole first 3 s), then tail near expected.
    head_lag, head_c = matched_peak(rx, head_tmpl, 0, min(len(rx)-4096, int(3.0*sr)))
    # tail expected around head_lag + tx_span (+- a few hundred samp for ppm)
    exp = head_lag + tx_span
    tail_lag, tail_c = matched_peak(rx, tail_tmpl, max(0,exp-1500), min(len(rx)-4096, exp+1500))
    rx_span = tail_lag - head_lag
    ratio = rx_span / tx_span
    ppm = (ratio - 1.0)*1e6
    # energy-span cross-check
    rxenv = np.array([np.sqrt(np.mean(rx[i:i+win]**2)) for i in range(0,len(rx)-win,win)])
    rthr = rxenv.max()*0.30
    ron = np.where(rxenv>rthr)[0]
    print(f"TX block: head={tx_head} tail={tx_tail} span(anchors)={tx_span} samp ({tx_span/sr*1000:.1f} ms)")
    print(f"RX match: head_lag={head_lag} (c={head_c:.3f})  tail_lag={tail_lag} (c={tail_c:.3f})")
    print(f"RX span (between anchors) = {rx_span} samp vs TX {tx_span}")
    print(f"==> clock ratio f_tx/f_rx = {ratio:.7f}  SFO = {ppm:+.1f} ppm")
    if len(ron):
        print(f"energy-span x-check: rx block ~{(ron[-1]-ron[0])*win} samp "
              f"({(ron[-1]-ron[0])*win/sr*1000:.1f} ms) starting ~{ron[0]*win/sr:.3f}s")
    print(f"RX peak={np.max(np.abs(rx)):.0f} ({np.max(np.abs(rx))/32768*100:.2f}% FS)")

if __name__=='__main__':
    main()

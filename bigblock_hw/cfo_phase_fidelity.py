#!/usr/bin/env python3
"""Standalone numeric proof: does the 641-tap Type-III Hilbert SSB injector
(cl_sim_cfo) add a FIXED per-subcarrier phase distortion when enabled, vs an
IDEAL FFT-domain analytic shift?  This isolates whether the estimate-vs-payload
gap (within-symbol non-linear pilot phase residual ~0.109 rad, CFO-MAGNITUDE-
INDEPENDENT) is an injector artifact rather than a production PHY defect.

Reproduces the FIR exactly (build_hilbert + process), compares to an ideal
analytic (scipy.signal.hilbert / FFT one-sided) shift, on a multi-tone passband
spanning the OFDM band edges (328..2672 Hz @ fs=48k).
"""
import numpy as np

fs = 48000.0
HILB_LEN = 641
DELAY = (HILB_LEN - 1) // 2

def build_hilbert():
    a0, a1, a2, a3 = 0.35875, 0.48829, 0.14128, 0.01168
    htap = np.zeros(DELAY + 1)
    for k in range(1, DELAY + 1, 2):
        ideal = 2.0 / (np.pi * k)
        n = DELAY - k
        th = 2.0 * np.pi * n / (HILB_LEN - 1)
        w = a0 - a1*np.cos(th) + a2*np.cos(2*th) - a3*np.cos(3*th)
        htap[k] = ideal * w
    return htap

def fir_analytic(x, htap):
    """Mirror process(): xd = delayed real; xh = sum c*(a-b) over odd k."""
    N = len(x)
    xd = np.zeros(N); xh = np.zeros(N)
    for i in range(N):
        xd[i] = x[i - DELAY] if i - DELAY >= 0 else 0.0
        s = 0.0
        for k in range(1, DELAY + 1, 2):
            a = x[i-(DELAY-k)] if i-(DELAY-k) >= 0 else 0.0
            b = x[i-(DELAY+k)] if i-(DELAY+k) >= 0 else 0.0
            s += htap[k]*(a - b)
        xh[i] = s
    return xd, xh

def fir_shift(x, cfo, htap):
    xd, xh = fir_analytic(x, htap)
    n = np.arange(len(x))
    ph = 2*np.pi*cfo*n/fs
    return xd*np.cos(ph) - xh*np.sin(ph)

def ideal_shift(x, cfo):
    """Ideal: one-sided FFT analytic signal, then frequency shift."""
    from numpy.fft import fft, ifft
    N = len(x); X = fft(x); H = np.zeros(N)
    H[0] = 1; H[N//2] = 1; H[1:N//2] = 2
    xa = ifft(X*H)              # analytic signal
    n = np.arange(N)
    return np.real(xa*np.exp(1j*2*np.pi*cfo*n/fs))

# Build a multi-tone passband across the OFDM band; measure per-tone phase after
# shifting, comparing FIR vs ideal. Per-subcarrier phase error = the artifact.
htap = build_hilbert()
band = np.linspace(328.0, 2672.0, 13)   # 13 tones across the OFDM occupancy
N = 8192
n = np.arange(N)
phs = np.random.RandomState(7).uniform(0, 2*np.pi, len(band))
x = sum(np.cos(2*np.pi*f*n/fs + p) for f, p in zip(band, phs))

for cfo in [0.5, 1.0, 4.0]:
    yf = fir_shift(x, cfo, htap)
    yi = ideal_shift(x, cfo)
    # measure each tone's phase at f+cfo via single-bin DFT (skip FIR group delay)
    g = DELAY
    def tone_phase(y, f):
        w = 2*np.pi*(f+cfo)/fs
        s = np.sum(y[g:]*np.exp(-1j*w*np.arange(g, N)))
        return np.angle(s), np.abs(s)
    perr = []
    for f in band:
        pf, mf = tone_phase(yf, f)
        pi, mi = tone_phase(yi, f)
        d = pf - pi
        while d > np.pi: d -= 2*np.pi
        while d < -np.pi: d += 2*np.pi
        perr.append(d)
    perr = np.array(perr)
    print(f"cfo={cfo:>4}: FIR-vs-ideal per-tone phase err  "
          f"RMS={np.sqrt(np.mean(perr**2)):.4f} rad  "
          f"span=[{perr.min():.3f}..{perr.max():.3f}]  "
          f"(nonlinear part after removing slope: "
          f"{np.sqrt(np.mean((perr-np.polyval(np.polyfit(band,perr,1),band))**2)):.4f})")

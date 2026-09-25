#!/usr/bin/env python3
"""
sim_channel_relay.py — device-free software channel for the SIM ARQ loopback.

Sits between two Mercury processes launched with `-x sim`. Each peer opens ONE
TCP connection to this relay and sends a 1-byte role tag ('A' commander /
'B' responder). The relay then carries BOTH directions over that connection:

    peer-A TX passband  --recv--> relay --+
                                          |  Watterson fade + CFO + AWGN
    peer-B TX passband  --recv--> relay --+
                                          |
    relay --send--> peer-A RX passband  <-+   (B's TX, channel-impaired)
    relay --send--> peer-B RX passband  <-+   (A's TX, channel-impaired)

i.e. each peer hears the OTHER peer's transmission through the channel, exactly
like an RF link / VB-Cable, but with deterministic, configurable noise & fading
and NO audio hardware. Wire units = little-endian float64 mono passband @48kHz,
the same samples Mercury's tx_transfer/rx_transfer move (audioio.c).

=== Channel model (per relayed chunk, in this order) =========================

  1. (Inc 4) Multipath fading — Watterson HF channel, complex baseband:
       x_real -> analytic x(t) (Hilbert FIR)
              -> y(t) = g0(t)*x(t) + g1(t)*x(t - dtau)      (2 Rayleigh taps)
              -> y(t) * exp(j*2*pi*cfo*t + j*phase_noise)   (CFO + phase jitter)
              -> Re(y(t))                                    (back to passband)
     Each tap gain g0,g1 is an independent zero-mean complex Gaussian process
     with a Gaussian Doppler power spectrum (Watterson, ITU-R F.1487). Profiles
     (delay spread / Doppler) follow the ITU/ARSFI standard "mid-latitude"
     classes:
        MPG  GOOD      dtau=0.5 ms  fd=0.1 Hz
        MPM  MODERATE  dtau=1.0 ms  fd=0.5 Hz
        MPP  POOR      dtau=2.0 ms  fd=1.0 Hz
        MPD  DISTURBED dtau=4.0 ms  fd=2.0 Hz
     The "wgn" profile disables fading (single unit tap, no Doppler) for a pure
     AWGN reference channel. Collapse on a fading channel now emerges from the
     Rayleigh nulls (the multipath the modem actually sees on HF), NOT from a
     synthetic burst-erasure proxy.

  2. (Inc 3) AWGN at a fixed channel SNR referenced to a 3 kHz noise bandwidth
     (SNR3k — the same axis the IONOS bench + muething_results.json use). The
     per-sample noise is generated to MATCH the modem's own BER harness so a
     sim cell at SNR3k reproduces the BER-harness noise at the equivalent
     Es/N0. See "Noise calibration" below.

  3. (orthogonal knob) Optional impulse / burst erasure (Gilbert-Elliott).
     This is NO LONGER the multipath proxy — it is an independent impulse-noise
     knob (lightning crashes, key-clicks). Off unless --burst is given. Deep
     fades come from the Watterson taps in stage 1.

=== Noise calibration (Inc 3) ===============================================

The modem's BER harness adds AWGN like this (telecom_system.cc:411-422 +
awgn.cc:65):

    f_nyquist = sampling_frequency / 2            # 24000 for 48 kHz passband
    sigma = sqrt(2 * P_sig * f_nyquist / (SNR_lin * BW_noise))
    noise[n] = (sigma / sqrt(2)) * N(0,1)          # awgn.cc ampl_val = sigma/sqrt(2)

    => per-sample real-noise stddev  = sqrt( P_sig * f_nyquist
                                             / (SNR_lin * BW_noise) )
    => per-sample real-noise power   = P_sig * f_nyquist / (SNR_lin * BW_noise)

P_sig is the MEAN-SQUARE (true power, not RMS) of the TX passband. The relay
measures it from the live TX stream with a sticky peak-hold so silent inter-
frame gaps keep carrying the noise floor (RF realism: band noise does not
vanish when a station stops keying). BW_noise = 3000 Hz makes the SNR axis the
channel SNR-in-3-kHz (SNR3k); the modem's own harness uses BW_noise = its
signal bandwidth (2343.75 Hz WB), so the two share the SAME noise PSD when
SNR3k = EsN0_harness + 10*log10(BW_signal/3000). The validator
tools/test_sim_relay_noise.py checks the noise PSD match numerically.

  --cell WGN:N   convenience: measured IONOS WGN-label -> SNR3k mapping.
                 The low/mid region is N + 4.8 dB; independent endpoint
                 self-noise limits the result toward 49.7 dB at very high N.

=== Reproducing the HW pathologies ==========================================
  * clean    : --snr 30                          (over-climb to CONFIG_16, no collapse)
  * wgn10    : --cell WGN:10 --profile mpm        (climbs then collapses on fades)
  * wgn-10   : --cell WGN:-10 --profile mpp       (deep-SNR STALL on ROBUST/CONFIG_0)
  * CFG16 reverse-ACK collapse (the bench-7 turnaround window-miss):
       --snr 40 --profile wgn --turnaround-drift
    The FAITHFUL drift path (TurnaroundDrift, SIMFIDELITY_ROOTCAUSE.md): the
    reverse MFSK ACK lands OUTSIDE the CMD's fixed listen window because
    accumulated per-key-up jitter + the PHYSICAL ±8 ppm slip walk its arrival
    INDEX out of the window (the ACK TONES are bit-exact; only their wire time
    shifts — integer silence insert/drop in the turnaround gaps). This is the
    RIGHT axis for the climb->CFG16->ACK-decay->D3->demote->0-byte trajectory
    and the FIX-9 D2 A/B. The old --drift-ppm-* tone-resampler is the WRONG axis
    (tone-smear, ~80x-too-large −670 ppm artifact) — SUPERSEDED, kept only for
    back-compat; see the DriftResampler header.

Usage:
    python tools/sim/sim_channel_relay.py --port 52100 --snr 12 [--profile mpm]
            [--cfo-hz 0.0] [--phase-noise-deg 0.2] [--cell WGN:-10]
            [--turnaround-drift]  # FAITHFUL CFG16 reverse-ACK collapse (default skew axis)
            [--burst --loss 0.02] [--seed 1] [--log relay.log]

Then launch two Mercury -x sim processes with MERCURY_SIM_ROLE A / B and the
same MERCURY_SIM_PORT. See tools/sim/sim_arq_channel.py for the full harness.
"""

import argparse
import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time

import numpy as np

CHUNK_SAMPLES = 1024          # MUST match SIM_CHUNK_SAMPLES in audioio.c
CHUNK_BYTES = CHUNK_SAMPLES * 8
STAMP_BYTES = 8               # <Q LE uint64 relay-stamped virtual clock (opt-in)
# Forwarded wire format. TWO modes, selected by --wire-stamp (DEFAULT OFF):
#
#  (default, --wire-stamp 0) BARE: the relay sends bare CHUNK_BYTES (8192) doubles
#    per chunk, exactly like the committed 1f7118e relay. The modem's virtual clock
#    is the LOCAL-ADD model (audioio.c rx_transfer -> sim_clock_add_samples): one
#    sample of virtual time per double the modem demods. The conservative-PDES
#    BARRIER (below) still bounds the inter-peer clock split because, under
#    local-ADD, each peer's virtual time tracks the chunks the relay forwarded TO
#    it, so bounding the relay's per-direction forward-count split bounds the
#    inter-peer clock split. This mode is COMPATIBLE with every shipped -x sim
#    mercury binary (all read bare CHUNK_BYTES).
#
#  (--wire-stamp 1) STAMPED (Q3, sim-arq-channel.md §10.5b/§11.2): each forwarded
#    chunk is "<Q vstamp" (8-byte LE uint64, the monotonic per-direction END sample
#    index after this chunk) PREPENDED to the CHUNK_BYTES payload => 8 + CHUNK_BYTES
#    bytes. This RELAY-DRIVEN clock requires a modem whose sim_rx_bridge_thread
#    reads the 8-byte stamp FIRST and calls sim_clock_set_samples(stamp) (the
#    feat/sim-clock modem half, fact-doc §11.2). A modem that does NOT read the
#    stamp gets silent 8-byte/chunk framing corruption -> garbage. STAY OFF until
#    the paired modem build (with the "[SIM] RX bridge first vstamp=" canary) is
#    the binary under test. See _simcal_takeover/ASSESSMENT.md.
#
#  INBOUND from a peer is ALWAYS bare CHUNK_BYTES in both modes (the modem TX bridge
#  never stamps; the relay owns the clock).

FS = 48000.0                  # passband wire sample rate (audioio.c)
F_NYQUIST = FS / 2.0          # 24000 Hz  (matches telecom_system.cc f_nyquist)
BW_NOISE = 3000.0             # SNR3k reference noise bandwidth (Hz)
CENTER_HZ = 1500.0            # OFDM center frequency (modem default)

WGN_TO_SNR3K = 4.8            # measured low/mid IONOS WGN-label intercept (dB)
IONOS_ENDPOINT_SNR3K_CEILING = 49.7  # independently added endpoint self-noise


def ionos_wgn_to_snr3k(label_db):
    """Map an IONOS WGN dial label to measured external SNR3k.

    Four physical sweeps (both directions) found an approximately unit-slope
    low/mid-region map with a 4.8 dB intercept.  Fifteen-second captures then
    measured 43.53 dB at WGN:40, explained by an independent 49.7 dB endpoint
    self-noise ceiling.  Noise powers add, so the mapping is intentionally
    nonlinear only near the clean end.  Direct ``--snr`` remains an exact
    SNR3k coordinate and does not use this compatibility mapping.
    """
    requested = float(label_db) + WGN_TO_SNR3K
    endpoint = IONOS_ENDPOINT_SNR3K_CEILING
    return -10.0 * math.log10(10.0 ** (-requested / 10.0) +
                              10.0 ** (-endpoint / 10.0))

# ITU-R F.1487 / ARSFI mid-latitude HF channel profiles.
#   delay spread dtau (s), Doppler spread fd (Hz, 2-sigma Gaussian).
PROFILES = {
    "wgn": None,                              # no fading (pure AWGN reference)
    "flat": None,                             # alias of wgn (the legacy flat
                                              # bench; regression control for
                                              # the fm_* profile family)
    "mpg": {"dtau": 0.5e-3, "fd": 0.1},       # GOOD
    "mpm": {"dtau": 1.0e-3, "fd": 0.5},       # MODERATE
    "mpp": {"dtau": 2.0e-3, "fd": 1.0},       # POOR
    "mpd": {"dtau": 4.0e-3, "fd": 2.0},       # DISTURBED (firmware MPD,
                                              # intMode=4: 4.0 ms delay ->
                                              # D=192 samples @48 kHz; fd=2.0 Hz
                                              # -> update=375 samples exact,
                                              # 7.8125 ms = 64x fd, matches
                                              # HFSim_BFD_2_03.ino:2269).
}


# ---------------------------------------------------------------------------
# Deterministic Gaussian source (per-direction RNG, reproducible).
# ---------------------------------------------------------------------------
class Xoshiro:
    """Deterministic Gaussian source (Box-Muller on a splitmix64 stream)."""
    def __init__(self, seed):
        self.s = (seed * 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        self._spare = None

    def _u64(self):
        self.s = (self.s + 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        z = self.s
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
        return (z ^ (z >> 31)) & 0xFFFFFFFFFFFFFFFF

    def uniform(self):
        return (self._u64() >> 11) * (1.0 / 9007199254740992.0)

    def gauss(self):
        if self._spare is not None:
            v = self._spare
            self._spare = None
            return v
        u1 = self.uniform()
        if u1 < 1e-15:
            u1 = 1e-15
        u2 = self.uniform()
        mag = math.sqrt(-2.0 * math.log(u1))
        self._spare = mag * math.sin(2.0 * math.pi * u2)
        return mag * math.cos(2.0 * math.pi * u2)

    def seed_np(self):
        """Spin off a numpy Generator seeded from this stream (independent)."""
        return np.random.default_rng(self._u64())


# ---------------------------------------------------------------------------
# Hilbert analytic-signal FIR (causal, continuous across chunks).
# ---------------------------------------------------------------------------
def _hilbert_fir(numtaps):
    """Type-III FIR Hilbert transformer (odd length, antisymmetric).
    h[n] for the 90-degree phase shifter; paired with a (numtaps-1)/2 sample
    delay on the in-phase branch so I and Q are time-aligned. Windowed (Hann).
    """
    if numtaps % 2 == 0:
        numtaps += 1
    m = (numtaps - 1) // 2
    n = np.arange(numtaps) - m
    h = np.zeros(numtaps)
    odd = n % 2 != 0
    h[odd] = 2.0 / (math.pi * n[odd])
    win = np.hanning(numtaps)
    return h * win, m


class AnalyticFilter:
    """Streaming real->analytic converter. Maintains the FIR history so the
    output is continuous (no per-chunk edge transients)."""
    def __init__(self, numtaps=129):
        self.h, self.delay = _hilbert_fir(numtaps)
        self.ntaps = len(self.h)
        # history holds the last (ntaps-1) input samples for overlap.
        self.hist = np.zeros(self.ntaps - 1)

    def process(self, x):
        # full convolution context = history + this chunk
        buf = np.concatenate((self.hist, x))
        # Q = Hilbert(x): 'valid' region aligns to the current chunk samples.
        q = np.convolve(buf, self.h, mode="full")
        # The valid output for the current chunk starts at index (ntaps-1).
        start = self.ntaps - 1
        q_chunk = q[start:start + len(x)]
        # I = x delayed by self.delay so it aligns with Q's group delay.
        # buf[-(len(x)+delay):-delay] is the delayed in-phase branch.
        i_full = buf
        # delayed-in-phase: take samples ending `delay` before the chunk end.
        idx_end = len(buf) - self.delay
        idx_start = idx_end - len(x)
        if idx_start < 0:
            # not enough history yet (first chunk): pad with zeros
            pad = -idx_start
            i_chunk = np.concatenate((np.zeros(pad), buf[0:idx_end]))[:len(x)]
        else:
            i_chunk = buf[idx_start:idx_end]
        # update history
        self.hist = buf[-(self.ntaps - 1):]
        return i_chunk + 1j * q_chunk


# ---------------------------------------------------------------------------
# IONOS firmware 128-tap Gaussian Doppler FIR (ARSFI HFSim_BFD_2_03.ino.src:497-540).
# Designed for a 64 Hz update rate, Fc=0.5856 Hz, ~Gaussian shape (-9.1 dB @ 1 Hz,
# -37.1 dB @ 2 Hz). DC-normalized (sum h = 1.0). Lifted verbatim from the firmware
# (extracted programmatically, NOT hand-transcribed) so the relay's Doppler power
# spectrum is the SAME Gaussian PSD the bench hardware produced. See the firmware
# comment block: "File: 128 Tap Adj Gauss LPF Rev2.ih_fir / Sample Freq 64 Hz,
# Fc=.5856 Hz, Window Off, Num Taps 128, ~Gauss -.900, N Poles=9".
GAUS_FIR_COEFFS = np.array([
    1.1755592671332046e-11, 2.0188004956137427e-10, 1.7236333623946176e-09, 9.815423109243151e-09,
    4.219820040519088e-08, 1.4693429486234634e-07, 4.338503956649552e-07, 1.122118806000393e-06,
    2.604091536729611e-06, 5.522713327023963e-06, 1.0857390465642334e-05, 2.0011412649247494e-05,
    3.4893068706162795e-05, 5.7982477259392286e-05, 9.237678934582388e-05, 0.00014180767284007714,
    0.00021062669000635658, 0.0003037561533907518, 0.0004266051229102419, 0.000584952237938201,
    0.0007847989367901134, 0.001032198204838678, 0.001333065243166745, 0.001692977321877409,
    0.002116970561258896, 0.002609341477645015, 0.003173460865247882, 0.00381160700116864,
    0.004524824309342139, 0.005312812558055807, 0.006173850455688009, 0.007104756211217515,
    0.008100886298035966, 0.00915617235512072, 0.010263194925878348, 0.01141329161173599,
    0.012596696236577472, 0.013802704802828978, 0.015019863385625786, 0.016236172665450826,
    0.01743930354214716, 0.01861681819809535, 0.019756391073966928, 0.020846024470695095,
    0.021874253876569306, 0.02283033861665188, 0.023704434009553566, 0.02448774186995001,
    0.02517263689027796, 0.025752767148979578, 0.026223127704178725, 0.026580106921515002,
    0.02682150583616039, 0.026946531447567135, 0.026955765379786157, 0.02685110980161695,
    0.026635712883536795, 0.026313876369100774, 0.025890948056565766, 0.025373202123371387,
    0.024767710285281262, 0.024082206768594926, 0.02332494999439922, 0.022504583735910823,
    0.02163000032189053, 0.020710208229666707, 0.019754206149457228, 0.018770865316331563,
    0.01776882160593159, 0.016756378583127243, 0.01574142238665159, 0.01473134903422507,
    0.013733004447687326, 0.012752637231260112, 0.011795863992392318, 0.010867646776884902,
    0.009972282000446704, 0.009113400098909145, 0.008293974989634013, 0.007516342337046732,
    0.006782225544921226, 0.006092768355660706, 0.005448572920512081, 0.004849742212182516,
    0.004295925680163602, 0.003786367096475506, 0.003319953602663025, 0.002895265044807468,
    0.002510622769190125, 0.002164137144270543, 0.0018537531721837, 0.001577293652557479,
    0.001332499460868328, 0.001117066600796574, 0.0009286797833839573, 0.0007650423737761393,
    0.0006239026277671727, 0.0005030762143350815, 0.0004004650862100223, 0.0003140728178353742,
    0.00024201657867910556, 0.00018253594974316996, 0.00013399882249807025, 9.49046426887381e-05,
    6.388527699708468e-05, 3.970378899074398e-05, 2.1251412801076355e-05, 7.5430092767428655e-06,
    -2.2887192936932196e-06, -8.999992637884094e-06, -1.3243447148502327e-05, -1.557613368708468e-05,
    -1.6467811426850445e-05, -1.63093528492861e-05, -1.5421097939455242e-05, -1.4061018704922785e-05,
    -1.243257788819571e-05, -1.0692187735418308e-05, -8.956195575428226e-06, -7.3073424710351094e-06,
    -5.8006591112396305e-06, -4.468779263172823e-06, -3.3266653970160216e-06, -2.3757534892377295e-06,
    -1.6075344987453848e-06, -1.0065986371058506e-06, -5.531753923550552e-07, -2.252074190014706e-07,
], dtype=np.float64)


# ---------------------------------------------------------------------------
# Gaussian-Doppler tap generator (Watterson). Produces a complex Gaussian
# process with a Gaussian-shaped Doppler power spectrum, FAITHFUL to the IONOS
# firmware: a fresh complex-Gaussian innovation is drawn every UPDATE samples
# (UPDATE = FS / (64*fd), the firmware's "64x the Doppler rate" cadence —
# MPG 156.25ms / MPM 31.25ms / MPP 15.625ms / MPD 7.812ms), pushed through the
# 128-tap GAUS_FIR_COEFFS (per I and Q rail), and HELD CONSTANT (zero-order hold)
# between updates — exactly as the firmware holds mixIQ.gain() until the next
# QuadGauss12FIR128() update (HFSim_BFD_2_03.ino.src:2260-2276, :668). The earlier
# Lorentzian (1st-order IIR) PSD had heavy tails that mis-shaped the deep fast-fade
# nulls; the Gaussian FIR matches the bench hardware's null statistics, which is
# what the live SKIP-VAR / FTR pilot-residual gate actually reads.
#
# UNIT TAP POWER (INV-4): the FIR steady-state output power per rail =
# innovation_var_per_rail * sum(h^2). To get a unit-power complex tap
# (E[|g|^2] = E[g_I^2] + E[g_Q^2] = 1), each rail's innovation variance is set to
# 0.5 / sum(h^2) so each rail FIR output has variance 0.5. This keeps mean|h|^2 = 1
# so the AWGN SNR3k calibration is undisturbed.
# ---------------------------------------------------------------------------
_FIR_SUMSQ = float(np.sum(GAUS_FIR_COEFFS * GAUS_FIR_COEFFS))   # ~0.019412
_FIR_NH = GAUS_FIR_COEFFS.size                                  # 128


class DopplerTap:
    def __init__(self, fd_hz, rng_np, update=None):
        self.fd = fd_hz
        self.rng = rng_np
        # update interval: 64x the Doppler rate (firmware cadence). NOT capped to
        # a chunk — the slow profiles' true update spans MANY chunks (MPG 7500
        # samples / 156ms, MPM 1500 / 31ms), and the advance() ZOH loop tracks the
        # span across chunk boundaries via self.pos. (The old code clamped this to
        # CHUNK_SAMPLES=1024, which ran MPG ~7x and MPM ~1.5x too fast = a too-wide
        # Doppler bandwidth on the slow profiles — a latent fidelity bug.)
        # fd<=0 -> static unit tap.
        if update is None:
            if fd_hz > 0:
                update = max(1, int(round(FS / (fd_hz * 64.0))))
            else:
                update = CHUNK_SAMPLES
        self.update = update
        # per-rail innovation std so the FIR output is unit-power complex.
        if fd_hz > 0:
            self.inno_std = math.sqrt(0.5 / _FIR_SUMSQ)
        else:
            self.inno_std = 0.0
        # FIR delay lines for the I and Q rails (most-recent-first, like the
        # firmware fir_float: delayx[0] = newest innovation).
        self.fir_i = np.zeros(_FIR_NH, dtype=np.float64)
        self.fir_q = np.zeros(_FIR_NH, dtype=np.float64)
        if fd_hz > 0:
            # prime the FIR with `_FIR_NH` updates so the very first emitted gain
            # is already steady-state (no startup transient of zero gains).
            for _ in range(_FIR_NH):
                self._fir_update()
            self.g_hold = self._fir_output()
        else:
            self.g_hold = self._static_white()
        self.pos = 0          # sample position within the current update span

    def _static_white(self):
        # unit-variance complex Gaussian (var(real)=var(imag)=1/2) for fd<=0.
        r = self.rng.standard_normal()
        i = self.rng.standard_normal()
        return (r + 1j * i) / math.sqrt(2.0)

    def _fir_update(self):
        """Push one fresh Gaussian innovation into the I and Q FIR delay lines
        (firmware QuadGauss12FIR128 -> fir_float: shift, insert newest at [0])."""
        self.fir_i = np.roll(self.fir_i, 1)
        self.fir_q = np.roll(self.fir_q, 1)
        self.fir_i[0] = self.rng.standard_normal() * self.inno_std
        self.fir_q[0] = self.rng.standard_normal() * self.inno_std

    def _fir_output(self):
        """Current shaped complex tap gain = (h . delay_i) + j (h . delay_q)."""
        gi = float(np.dot(GAUS_FIR_COEFFS, self.fir_i))
        gq = float(np.dot(GAUS_FIR_COEFFS, self.fir_q))
        return gi + 1j * gq

    def advance(self, n):
        """Return an array of n complex tap gains, advancing internal state.
        Zero-order hold: the shaped gain is constant within an update span and
        steps to a new shaped value at each 64x-fd update boundary (firmware
        behavior — no per-sample interpolation)."""
        out = np.empty(n, dtype=np.complex128)
        k = 0
        while k < n:
            if self.fd <= 0:
                # static tap (no Doppler): constant unit gain
                out[k:] = self.g_hold
                self.pos = (self.pos + (n - k)) % self.update
                break
            span = self.update - self.pos
            take = min(span, n - k)
            out[k:k + take] = self.g_hold          # ZOH within the update span
            k += take
            self.pos += take
            if self.pos >= self.update:
                self.pos = 0
                self._fir_update()
                self.g_hold = self._fir_output()
        return out


# ---------------------------------------------------------------------------
# Inter-peer sample-clock drift model — OLD TONE-RESAMPLER AXIS (OPT-IN, kept
# behind --drift-ppm-*, but SUPERSEDED — do NOT use as the default vehicle).
# ---------------------------------------------------------------------------
# *** SUPERSEDED 2026-06-10 (SIMFIDELITY_ROOTCAUSE.md). READ THIS FIRST. ***
# This DriftResampler models the collapse as a continuous LINEAR-INTERPOLATION
# fractional resampler driven by the −670 ppm [CLK-TX] number. BOTH premises are
# now known wrong:
#   (1) WRONG AXIS: linear interp is a 2-tap low-pass that SMEARS the narrowband
#       MFSK ACK tones, so the CMD correlator drops on tone QUALITY (the ACK
#       lands IN the window but correlates weakly). The REAL HW failure is a
#       TURNAROUND-TIMING WINDOW-MISS: the reverse ACK is BIT-CLEAN (bench-7
#       bitmap 0x01ffffff, CRC-valid) but lands OUTSIDE the CMD's fixed listen
#       window (peak_metric=0.0). Different mechanism — which is exactly why D2
#       (a wider window) was "unprovable" against this model: a wider window
#       can't restore a smeared correlation.
#   (2) WRONG MAGNITUDE: −670/−817 ppm is a [CLK-TX] software ARTIFACT (a
#       10 s-window producer-push-rate quantization, SIMFIDELITY §1), ~80× the
#       PHYSICAL ±8.16 ppm crystal skew (CLOCK_VERDICT.md §2).
# The CORRECTED, DEFAULT-faithful model is `TurnaroundDrift` (--turnaround-drift)
# below: integer silence insert/drop that re-positions the ACK by WINDOW-MISS
# with the tones BIT-EXACT, ±8 ppm slip + per-key-up jitter. Use that for the
# CFG16 collapse repro and the D2 A/B. DriftResampler is retained only so the old
# --drift-ppm-* invocations still parse (it is the WRONG axis; never the default).
#
# (Original rationale below, preserved for history — its −670 framing is the
# artifact the SUPERSEDED note above corrects.)
#
# WHY this exists (FIX9_ROOTCAUSE.md, bench-5): the two RPis' soundcards run at
# INDEPENDENT sample rates. The HW logs measured [CLK-TX] drift=-670.7 ppm and
# [CLK-RX] drift=-193.0 ppm — hundreds of ppm of RELATIVE skew. Over an 8.5 s
# 25-frame CONFIG_16 OFDM batch that is ~5.7 ms (~274 samples) of timing slip per
# batch, ACCUMULATING across the retransmit loop until the half-duplex frame
# boundaries de-align past the receiver's search window — the CMD's MFSK ACK-SACK
# correlator then sees pure silence (peak_metric=0.0), times out, and BREAKs at
# CFG16 (the climb-collapse). The committed relay's conservative-PDES BARRIER
# (counters[key] split <= K chunks) bounds the a2b/b2a virtual-clock split to
# K*1024 samples BY DESIGN, so it STRUCTURALLY MASKS this drift (FIX9 §3): the
# default sim is a single-clock, barrier-locked channel and the over-climb
# regime it reproduces never carries the physical sample-rate skew that breaks
# the turnaround on HW. This model adds that skew back, OPT-IN, so the FIX9
# D2/D3 turnaround fixes have a failing-first off-bench vehicle.
#
# CONTRACT (default OFF == byte-identical, the PDES determinism A/B baseline):
#   * ppm == 0.0  -> the resampler is a strict pass-through: input chunk in,
#     SAME chunk out, no float reconstruction, no state. A seeded WGN cell is
#     bit-for-bit identical with the model present-but-off (validated:
#     tools/sim/test_sim_relay_drift.py [1], and a full 2-process cell md5).
#   * The drift is applied AFTER ch.process() (the calibrated Watterson+AWGN+CFO
#     channel math): the noise/tap RNG advances per INBOUND chunk exactly as
#     before — drift only RE-TIMES the already-channel-impaired forwarded audio.
#     So the noise PSD / fade realization / SNR calibration are untouched; only
#     the sample-grid the receiving peer's clock tracks is skewed.
#
# MODEL: a continuous per-direction fractional resampler. ppm defines the
# forwarded-stream rate change rate_out/rate_in = (1 + ppm/1e6). It consumes the
# per-direction stream of 1024-sample channel-output blocks and re-emits it as a
# drifted stream STILL chunked into exactly-1024-sample wire chunks (the modem
# reads bare SIM_CHUNK_SAMPLES doubles; chunk SIZE must never change). Because in
# and out rates differ, one input block does NOT map to one output block: a
# positive ppm (faster sink clock) occasionally yields an EXTRA output chunk; a
# negative ppm occasionally HOLDS one back (buffered). That extra/held chunk is
# precisely the sample the physical clock skew adds/drops, and — because each
# forwarded chunk advances the receiving peer's local-ADD virtual clock by 1024
# samples (audioio.c rx_transfer -> sim_clock_add_samples) — it de-aligns that
# peer's clock against the transmitting peer's batch boundaries, exactly the HW
# mechanism. Linear interpolation between input samples (one-tap history carried
# across chunks for continuity); cheap and sufficient at ppm scale (the band is
# heavily oversampled — OFDM occupies <2.4 kHz of the 24 kHz Nyquist).
class DriftResampler:
    """Per-direction fractional resampler modelling sample-clock skew.

    ppm == 0 -> strict identity pass-through (byte-identical, no float work).
    ppm != 0 -> resample the forwarded stream by (1 + ppm/1e6), emitting only
    whole 1024-sample wire chunks and carrying the fractional read phase + the
    one-sample interpolation history across chunks (continuous, no per-chunk
    edge transient)."""

    def __init__(self, ppm):
        self.ppm = float(ppm)
        self.enabled = (self.ppm != 0.0)
        # ratio = input samples consumed per output sample produced.
        # rate_out/rate_in = 1 + ppm/1e6  =>  in/out step = 1/(1+ppm/1e6).
        self.step = 1.0 / (1.0 + self.ppm / 1.0e6) if self.enabled else 1.0
        self.read_pos = 0.0          # fractional read index into the input stream
        self.consumed = 0            # whole input samples already discarded
        self.buf = np.empty(0, dtype=np.float64)   # un-consumed input tail
        self.last_sample = 0.0       # input sample just before self.buf[0]
        self.out_carry = []          # produced output samples not yet a full chunk
        self.in_total = 0            # diag: total input samples seen
        self.out_total = 0           # diag: total output samples produced

    def process(self, chunk):
        """Feed one CHUNK_SAMPLES-length list/array of channel-output samples.
        Returns a list of zero-or-more CHUNK_SAMPLES-length lists (whole wire
        chunks) ready to forward. ppm==0 -> returns [chunk] unchanged."""
        if not self.enabled:
            return [list(chunk)]
        x = np.asarray(chunk, dtype=np.float64)
        self.in_total += x.size
        # Append new input to the working buffer. Indexing convention: index -1
        # is self.last_sample (the sample just before buf[0]); buf[i] is input
        # sample (consumed + i). read_pos is an absolute fractional input index.
        self.buf = np.concatenate((self.buf, x))
        # Produce output samples while we have enough input to interpolate the
        # next read position (need buf up to floor(read_pos)+1 relative to base).
        base = self.consumed                     # absolute index of buf[0]
        n_buf = self.buf.size
        out = self.out_carry
        # highest input index we can interpolate to = base + n_buf - 1
        last_idx = base + n_buf - 1
        rp = self.read_pos
        while rp <= last_idx:
            i0 = int(math.floor(rp))
            frac = rp - i0
            # sample at i0 and i0+1 (relative to base); i0 may be base-1 => use
            # last_sample (history) for the left tap on the very first sample.
            li = i0 - base
            if li < 0:
                s0 = self.last_sample
            else:
                s0 = self.buf[li]
            ri = li + 1
            if ri < n_buf:
                s1 = self.buf[ri]
            elif ri == n_buf:
                # need the next input sample we don't have yet -> stop, wait.
                break
            else:
                break
            out.append((1.0 - frac) * s0 + frac * s1)
            rp += self.step
        self.read_pos = rp
        # Discard fully-consumed input from the buffer (keep one sample of
        # history before the current read position for the next left tap).
        keep_from = int(math.floor(self.read_pos)) - base   # first idx still needed
        drop = keep_from - 1                                 # keep one history sample
        if drop > 0:
            drop = min(drop, n_buf)
            self.last_sample = self.buf[drop - 1]
            self.buf = self.buf[drop:]
            self.consumed += drop
        # Emit whole 1024-sample chunks from the output carry.
        chunks = []
        while len(out) >= CHUNK_SAMPLES:
            chunks.append(out[:CHUNK_SAMPLES])
            del out[:CHUNK_SAMPLES]
        self.out_carry = out
        self.out_total += sum(len(c) for c in chunks)
        return chunks


# ---------------------------------------------------------------------------
# PTT / half-duplex turnaround latency (OPT-IN, DEFAULT OFF).
# ---------------------------------------------------------------------------
# WHY (FIX9_ROOTCAUSE.md §2 Rank-1.3, §3 point 2): on real HW the half-duplex
# turnaround carries a physical keying + AGC-settle + capture-flush latency at
# every TX onset that the sim idealizes to zero (the SIM_INPROC/relay PTT spins
# collapse to deterministic clock advances whose exit predicate always holds).
# That zero-latency turnaround is half of why the sim can't reproduce the
# CMD(2.6s)/RSP(12s) post-TX budget de-sync. This model re-injects it.
#
# CAN THE RELAY DETECT A DIRECTION SWITCH? YES. The reader already classifies
# each INBOUND chunk as `silent` (raw pre-channel mean-square < SILENCE_EPS ==
# the modem TX bridge's memset(0) idle fill) vs carrying signal. A TX ONSET on a
# direction is the rising edge silent->signal: the peer just keyed up a real
# burst (HAIL / OFDM batch / MFSK ACK). The relay keys PTT latency to THAT edge.
#
# MODEL: on a detected onset for a direction, DELAY the burst by inserting
# ptt_latency_ms (+/- uniform jitter) worth of channel-noise SILENCE chunks
# ahead of the first signal chunk, then forward the burst. The injected silence
# is real channel output (noise floor continues), so the receiver simply sees
# the burst ARRIVE LATER relative to its own post-TX search window — the keying
# delay. Per-onset jitter (seeded) varies the turnaround like real keyer/AGC
# variation. DEFAULT 0 ms == no insertion == byte-identical.
class PttLatencyModel:
    """Per-direction TX-onset keying-latency injector. latency_ms==0 -> inert."""

    def __init__(self, latency_ms, jitter_ms, rng):
        self.latency_ms = max(0.0, float(latency_ms))
        self.jitter_ms = max(0.0, float(jitter_ms))
        self.enabled = (self.latency_ms > 0.0)
        self.rng = rng                  # Xoshiro (deterministic jitter)
        self.prev_silent = True         # link starts idle (no burst in flight)
        self.n_onsets = 0
        self.n_delay_chunks = 0

    def onset_delay_chunks(self, silent):
        """Given THIS inbound chunk's silent flag, return how many channel-noise
        silence chunks to inject BEFORE forwarding it (0 unless this is a rising
        silent->signal edge and the model is enabled). Updates edge state."""
        inject = 0
        if self.enabled and self.prev_silent and (not silent):
            self.n_onsets += 1
            ms = self.latency_ms
            if self.jitter_ms > 0.0:
                # symmetric uniform jitter in [-jitter, +jitter], clamped >= 0
                ms = max(0.0, ms + (2.0 * self.rng.uniform() - 1.0) * self.jitter_ms)
            inject = int(round(ms * FS / 1000.0 / CHUNK_SAMPLES))
            self.n_delay_chunks += inject
        self.prev_silent = silent
        return inject


# ---------------------------------------------------------------------------
# CONNECT-REACK T1 — deterministic single-burst eraser (TEST-ONLY).
# ---------------------------------------------------------------------------
# Counts SIGNAL BURSTS on a direction (each silent->signal rising edge is a new
# burst, mirroring the PTT onset detector) and zeros the RAW channel input of
# the Nth burst's signal chunks. Zeroing pre-channel input means the far end
# sees the calibrated noise floor (ch.process(zeros)) exactly where that burst
# would have been -> a clean ERASURE of one whole reverse transmission, the
# deterministic "CMD missed the single TEST_ACK" failure the re-ACK fix heals.
# target_burst==0 -> disabled (identity pass-through, byte-identical).
class BurstEraser:
    def __init__(self, target_burst):
        self.target = int(target_burst)        # 1-based; 0 = disabled
        self.enabled = self.target > 0
        self.prev_silent = True                # link starts idle
        self.burst_idx = 0                     # signal bursts seen so far
        self.n_erased_chunks = 0
        self.last_onset = 0                    # burst idx if this chunk was an edge

    def maybe_erase(self, samples, silent):
        """Given a raw inbound CHUNK and its silent flag, return the (possibly
        zeroed) samples. Advances the burst index on each silent->signal edge."""
        if not self.enabled:
            return samples
        self.last_onset = 0                    # set to new burst idx on an edge
        if (not silent) and self.prev_silent:
            self.burst_idx += 1                # a new burst keyed up
            self.last_onset = self.burst_idx
        self.prev_silent = silent
        if (not silent) and self.burst_idx == self.target:
            self.n_erased_chunks += 1
            return [0.0] * CHUNK_SAMPLES       # erase this burst's signal
        return samples


# ---------------------------------------------------------------------------
# FAITHFUL turnaround-timing de-alignment model (the DEFAULT drift path).
# ---------------------------------------------------------------------------
# WHY this SUPERSEDES DriftResampler (SIMFIDELITY_ROOTCAUSE.md §1-§3, derived
# from bench-7's HW logs): the real HW collapse is NOT a tone/grid distortion.
# The reverse MFSK ACK+SACK is transmitted INTACT and FULL-CONTENT (bench-7 RSP
# bitmap = 0x01ffffff = 25/25, CRC-valid) but the CMD's correlator reports
# peak_metric=0.0 mask=1111111111111111 — the ACK simply does NOT land in the
# CMD's fixed `receiving_timeout` listen window. It is a TURNAROUND-TIMING
# WINDOW-MISS: accumulated half-duplex PTT/capture-flush/scheduling jitter (the
# DOMINANT trigger) plus the small PHYSICAL ±8.16 ppm crystal slip (CLOCK_VERDICT
# §2 — NOT the −670/−817 ppm [CLK-TX] number, which §1 proves is a 10 s-window
# producer-push-rate QUANTIZATION ARTIFACT, ~80× too large) walks the ACK's
# ARRIVAL INDEX out of the window, with no per-frame timing recovery to absorb it.
# CFG15 collapses the SAME way once de-aligned; the forward OFDM stays healthy.
#
# WHY DriftResampler (the OLD --drift-ppm-* path) is the WRONG axis (kept behind
# its flag, NOT the default — N1): it is a continuous LINEAR-INTERPOLATION
# fractional resampler. Linear interp is a 2-tap low-pass that SMEARS the
# narrowband MFSK ACK tones, so the CMD correlator drops on tone QUALITY (a
# metric sag, ACK lands IN the right window but correlates poorly) — a DIFFERENT
# failure axis than HW. That is exactly why D2 (a wider listen window + fatter
# turnaround geometry) was "unprovable in sim": a wider window cannot restore a
# smeared correlation. The faithful model fails by WINDOW-MISS, which is what D2
# addresses, so D2 becomes sim-provable.
#
# MODEL (M1-M4 of SIMFIDELITY_ROOTCAUSE §3.2):
#   M1 (tone fidelity): every SIGNAL sample passes BIT-EXACT. Timing skew is
#      realized ONLY by inserting/dropping WHOLE SILENCE samples in the
#      turnaround GAPS (where the wire already carries the noise floor / zeros),
#      shifting subsequent chunk boundaries. The ACK's tones are never touched —
#      only their wire arrival INDEX moves. (Assert: the ACK samples that DO
#      arrive are bit-identical to the un-drifted ACK.)
#   M2 (accumulating ±8 ppm crystal slip): a cumulative timing offset that grows
#      with samples forwarded at the PHYSICAL ppm rate (default ±8.16, sign per
#      direction), NEVER reset per-batch (no per-frame timing recovery).
#   M3 (per-key-up jitter, the DOMINANT trigger): at each TX-onset edge
#      (silent->signal, i.e. each PTT key-up) add a bounded SEEDED random extra
#      offset (a few ms; the PTT/capture-flush/scheduling jitter). Seeded =>
#      reproducible A/B; variable per key-up => the ACK position RANDOM-WALKS,
#      matching the bench's non-monotone mask decay.
#   M4: the relay relaxes the PDES barrier in drift-mode ONLY (main()); drift-OFF
#      stays byte-identical (N2).
#
# REALIZATION (how integer insert/drop in silence keeps 1024-sample wire chunks
# AND preserves every signal sample): a streaming silence-aware re-timer. The
# model maintains (a) a fractional sample-offset accumulator `acc` (advanced by
# M2 per sample + M3 per onset), and (b) a flat per-direction output sample
# stream into which inbound samples flow. When `acc` crosses an integer N>0 it
# owes N inserted samples (sink slower than source -> ACK arrives LATER); N<0 it
# owes N dropped samples. The owed insert/drop is APPLIED inside SILENCE runs
# only: on a silent inbound chunk the model pads (insert) or skips (drop) silence
# samples; a SIGNAL chunk is copied verbatim with ZERO insert/drop, so signal
# samples are bit-exact. Output is re-chunked to exactly CHUNK_SAMPLES; a partial
# tail is carried. The injected silence samples are real channel output (the
# reader feeds ch.process(zeros) noise floor), so the receiver sees the SAME
# tones at a SHIFTED time — exactly the HW failure axis.
class TurnaroundDrift:
    """Per-direction FAITHFUL timing-de-alignment re-timer (M1-M3).

    Re-positions the reverse-ACK (and every burst) by an INTEGER number of wire
    samples relative to the receiver's listen window, realizing the offset by
    insert/drop of WHOLE SILENCE samples in turnaround gaps. SIGNAL samples pass
    BIT-EXACT (M1). The offset = a cumulative ±ppm crystal slip (M2) + a bounded
    seeded per-onset jitter (M3). enabled==False -> strict identity pass-through
    (byte-identical, the N2 contract)."""

    def __init__(self, ppm, jitter_ms, rng, enabled=True):
        self.ppm = float(ppm)
        self.jitter_ms = max(0.0, float(jitter_ms))
        self.rng = rng                 # Xoshiro (deterministic per-onset jitter)
        self.enabled = bool(enabled) and (self.ppm != 0.0 or self.jitter_ms > 0.0)
        # acc = signed fractional sample offset still owed to the OUTPUT (the wire
        # the receiver clocks on). acc>0 => insert silence (ACK arrives later);
        # acc<0 => drop silence (ACK arrives earlier). NEVER reset (M2).
        self.acc = 0.0
        self.prev_silent = True        # link starts idle
        self.out = []                  # flat output-sample carry (re-chunked)
        # diagnostics
        self.n_onsets = 0
        self.in_total = 0
        self.out_total = 0
        self.n_inserted = 0            # silence samples inserted (cumulative)
        self.n_dropped = 0             # silence samples dropped (cumulative)
        self.max_abs_offset = 0.0      # peak |acc| reached (samples)

    def _emit_chunks(self):
        """Pull whole CHUNK_SAMPLES blocks from the flat output carry."""
        chunks = []
        out = self.out
        while len(out) >= CHUNK_SAMPLES:
            chunks.append(out[:CHUNK_SAMPLES])
            del out[:CHUNK_SAMPLES]
        return chunks

    def process(self, chunk, silent):
        """Feed one CHUNK_SAMPLES channel-output block + its inbound-silent flag.
        Returns a list of zero-or-more CHUNK_SAMPLES wire blocks. enabled==False
        -> returns [chunk] verbatim (identity)."""
        if not self.enabled:
            return [list(chunk)]
        x = list(chunk)
        n = len(x)
        self.in_total += n

        # M2: accumulate the physical crystal slip over the samples we forward.
        self.acc += n * (self.ppm / 1.0e6)

        # M3: at a silent->signal rising edge (PTT key-up) add a bounded seeded
        # jitter. Symmetric uniform in [-jitter, +jitter] samples. This is the
        # DOMINANT trigger that random-walks the ACK arrival index.
        onset = (self.prev_silent and not silent)
        if onset:
            self.n_onsets += 1
            if self.jitter_ms > 0.0:
                jsamp = (2.0 * self.rng.uniform() - 1.0) * self.jitter_ms \
                        * FS / 1000.0
                self.acc += jsamp
        self.prev_silent = silent
        if abs(self.acc) > self.max_abs_offset:
            self.max_abs_offset = abs(self.acc)

        if not silent:
            # SIGNAL chunk: tones BIT-EXACT (M1 — never interpolate/alter them).
            # At a key-up ONSET with a POSITIVE owed offset, realize the
            # turnaround LATENCY by inserting whole SILENCE samples AHEAD of the
            # burst (the physical "keyed up late" — the burst arrives later in the
            # window). This is still M1-clean: the inserted samples are silence,
            # the burst samples are copied verbatim. A NEGATIVE owed offset (drop)
            # cannot eat signal, so it stays in `acc` and is paid down in the next
            # silence gap (you can only compress idle time, not a live burst).
            if onset and self.acc >= 1.0:
                k = int(math.floor(self.acc))
                # silence value = the channel-output noise floor of THIS edge is
                # in the burst itself; use a near-zero pad (the gap noise already
                # advanced the RNG). A flat low pad is spectrally inert pre-burst.
                self.out.extend([0.0] * k)
                self.acc -= k
                self.n_inserted += k
            self.out.extend(x)
        else:
            # SILENCE chunk: this is where we pay down the owed offset. Insert
            # extra silence samples (acc>=+1) or drop silence samples (acc<=-1).
            # Take the silence VALUE from the channel-output silence itself (the
            # noise floor), so injected/retained silence is statistically the
            # same band noise — never a hard zero discontinuity in a faded cell.
            if self.acc >= 1.0:
                k = int(math.floor(self.acc))
                # insert k silence samples by repeating the chunk's mean-noise
                # tail value pattern; cheap + spectrally inert at noise level.
                pad = [x[i % n] for i in range(k)] if n else [0.0] * k
                self.out.extend(pad)
                self.out.extend(x)
                self.acc -= k
                self.n_inserted += k
            elif self.acc <= -1.0:
                k = min(int(math.floor(-self.acc)), n)
                # drop the first k silence samples of this chunk (advance output
                # past them); the remaining (n-k) silence samples still flow.
                self.out.extend(x[k:])
                self.acc += k
                self.n_dropped += k
            else:
                self.out.extend(x)

        chunks = self._emit_chunks()
        self.out_total += sum(len(c) for c in chunks)
        return chunks


# ---------------------------------------------------------------------------
# Narrow-FM audio-passband bandpass (radio audio filter the bidirectional
# passband_probe is meant to DISCOVER).
#
# WHY (sim-fidelity): a real narrow-FM data link is the radio's mic/speaker
# AUDIO path, band-limited by the rig's audio filters to ~300-2900 Hz (VARA FM
# narrow occupies 2580 Hz, 300-2880 Hz). The raw relay Channel had NO
# band-limiting, so Iris's chirp passband_probe (200-4600 Hz) saw the whole band
# flat. Config-selectable edges (narrow 300-2900 / wide 300-6300 / off) so the
# 6 kHz direct-discriminator lane can reuse the SAME mechanism.
#
# NOTE (probe interaction, verified 2026-07-04): band-limiting the SIGNAL is the
# physically-correct radio-audio-filter model, but it does NOT by itself change
# what Iris's CURRENT passband_probe discovers, because that probe estimates its
# noise floor from the out-of-CHIRP-band deconvolution bins (>4700/<150 Hz) where
# the reference chirp has ~zero energy -> the floor is a deconvolution artifact
# (~-170 dB) and the dual-threshold OR-branch (tone >= floor+10) re-detects every
# attenuated tone. See the return report / fact doc: the true root of the 4150 Hz
# over-discovery is in iris passband_probe.cc, not the channel. This filter is
# still the correct sim-fidelity model of the radio audio path (band-limits the
# real OFDM signal) and is a prerequisite for the fair comparison.
#
# HOW: linear-phase windowed-sinc (Blackman) FIR bandpass applied to the SIGNAL
# only (before the broadband AWGN), streaming with a persistent history tail
# (overlap-save, same pattern as AnalyticFilter) so it is continuous across the
# 1024-sample chunks. Passband gain normalized to 1.0 at band center so the SNR3k
# calibration is undisturbed. numpy-only. Default OFF -> the Mercury HF paths
# that share this Channel are byte-identical unless a caller opts in.
# ---------------------------------------------------------------------------
BANDPASS_PRESETS = {
    "off":    (0.0, 0.0),
    "narrow": (300.0, 2900.0),   # narrow-FM audio path (VARA FM narrow occupancy)
    "wide":   (300.0, 6300.0),   # 6 kHz direct-discriminator (9600 data-port) lane
}
BANDPASS_DEFAULT_TAPS = 511


def resolve_bandpass(mode, lo_override=None, hi_override=None, taps=None):
    """Resolve an audio-bandpass spec to (lo_hz, hi_hz, taps). `mode` is a
    BANDPASS_PRESETS key (off/narrow/wide); explicit lo/hi override (>0) wins."""
    lo, hi = BANDPASS_PRESETS.get((mode or "off").lower(), (0.0, 0.0))
    if lo_override is not None and lo_override > 0.0:
        lo = float(lo_override)
    if hi_override is not None and hi_override > 0.0:
        hi = float(hi_override)
    return lo, hi, int(taps if taps else BANDPASS_DEFAULT_TAPS)


def _fir_bandpass_taps(numtaps, flo, fhi, fs):
    """Linear-phase windowed-sinc bandpass FIR (Type-I, odd length, Blackman),
    gain-normalized to 1.0 at band center. Returns (h, group_delay)."""
    if numtaps % 2 == 0:
        numtaps += 1
    m = (numtaps - 1) // 2
    n = np.arange(numtaps) - m

    def ideal_lp(fc_hz):
        wc = 2.0 * fc_hz / fs
        return wc * np.sinc(wc * n)

    h = (ideal_lp(fhi) - ideal_lp(flo)) * np.blackman(numtaps)
    fc = 0.5 * (flo + fhi)
    gain = float(np.sum(h * np.cos(2.0 * math.pi * fc / fs * n)))
    if abs(gain) > 1e-12:
        h = h / gain
    return h, m


class StreamingFIR:
    """Streaming FIR with a persistent input-history tail (overlap-save) so the
    output is continuous across chunks. Group delay = (ntaps-1)/2 samples."""
    def __init__(self, h):
        self.h = np.asarray(h, dtype=np.float64)
        self.ntaps = len(self.h)
        self.hist = np.zeros(self.ntaps - 1)

    def process(self, x):
        x = np.asarray(x, dtype=np.float64)
        buf = np.concatenate((self.hist, x))
        y = np.convolve(buf, self.h, mode="full")
        start = self.ntaps - 1
        out = y[start:start + len(x)]
        self.hist = buf[-(self.ntaps - 1):]
        return out


# ===========================================================================
# FM RADIO CHANNEL (the fm_* profile family) — REAL modulate/demodulate chain.
# ===========================================================================
# WHY (sim-fidelity, 2026-07-09): the flat bench profile ("wgn"/"flat") adds
# WHITE noise to AUDIO and never touches an FM modulator. A real FM data link
# is audio -> [radio TX: pre-emphasis? limiter] -> FM modulator -> RF (AWGN at
# a CARRIER-to-noise ratio) -> discriminator -> [radio RX: de-emphasis?] ->
# audio. Three physical effects follow that the flat profile CANNOT produce:
#
#   1. TRIANGULAR NOISE. An FM discriminator operating above threshold with
#      white RF noise at its input produces output noise whose power spectral
#      density rises as f^2 (+6 dB/octave). Textbook result (Haykin,
#      "Communication Systems"; Carlson, "Communication Systems"; measured on
#      any discriminator with no carrier modulation). So on the flat "9600"
#      data port (no de-emphasis) the HIGH audio frequencies are the noisy
#      ones — the flat-white bench INVERTS the per-carrier reliability
#      ranking. De-emphasis (a matched RX 1/f roll-off) flattens it back and
#      buys real SNR — that is the entire purpose of the emphasis pair.
#   2. THE FM THRESHOLD / CLICK KNEE. Below ~10 dB CNR the discriminator
#      emits impulsive 2*pi phase-slip "clicks" (Rice 1948, "Statistical
#      Properties of a Sine Wave Plus Random Noise"; Taub & Schilling ch. FM
#      threshold): audio SNR collapses much faster than CNR. The flat bench
#      has no knee at all.
#   3. THE DEVIATION LIMITER. Every FM transmitter hard-limits peak deviation
#      (FCC/TIA-603 occupied-bandwidth rules; the limiter sits AFTER
#      pre-emphasis, before a splatter LPF — Sexauer K3VIX,
#      repeater-builder.com/tech-info/fm-theory-discussion.html). A
#      high-PAPR OFDM signal clips in it; DFT-spread (SC-FDMA) exists
#      precisely to survive it. The flat bench never engages a limiter.
#
# EMPHASIS STANDARD (researched 2026-07-09, do not "fix" to 75 us): land-
# mobile / amateur NBFM uses a +6 dB/octave TX pre-emphasis rising from a
# ~300 Hz corner across 300-3000 Hz with matched RX de-emphasis (TIA-603;
# FCC commercial practice per K3VIX: "the FCC specifies that the pre-emphasis
# be a 6 dB per octave rising response beginning at 300 Hz", most rigs start
# 200-300 Hz). 75 us / 50 us corners (2122 / 3183 Hz) are BROADCAST FM
# (ITU-R BS.450) and are the WRONG constants for a voice-band NBFM rig: a
# 75 us corner would leave 300-2000 Hz un-emphasized. Default corner here is
# 300 Hz (tau ~= 531 us) — which is also the corner Iris's TX pre-compensation
# assumes (iris/source/ofdm/ofdm_mod.cc fm_tx_deemph_gain, ofdm_config
# fm_preemph_corner_hz = 300).
#
# FILTER DESIGN: the pre/de-emphasis pair is the bilinear-transform design of
# GNU Radio gr-analog fm_emph.py (fm_preemph / fm_deemph): de-emphasis
# H(s) = (1/tau)/(s + 1/tau); pre-emphasis H(s) = (s + w_cl)/(s + w_ch) with
# the upper flatten pole w_ch (default 0.925*fs/2), gain-normalized to 0 dB
# at DC. A matched pair is transparent end-to-end up to f_ch (validated in
# tools/sim/test_fm_channel_sim.py V3).
#
# WHY REAL MOD/DEMOD, NOT AN ANALYTICAL NOISE MODEL: modulating
# s[n] = exp(j*2*pi*kf*cumsum(m)/fs), adding complex AWGN at the target CNR,
# band-limiting to the Carson IF bandwidth, and discriminating
# mhat[n] = arg(s[n]*conj(s[n-1]))*fs/(2*pi*kf) produces the triangular
# noise, the threshold clicks, the capture behavior, and the limiter
# distortion FOR FREE with the right cross-terms. (iris/tools/
# fm_channel_relay.py models these analytically — injected f^2-shaped noise +
# a Rice click generator; this chain SUPERSEDES that approach for FM cells.)
#
# === CNR / SNR UNITS — READ BEFORE USING (do NOT reuse the WGN label) ======
# "WGN:40"/--snr is an AUDIO-domain SNR3k and is REFUSED for fm_* profiles.
# The honest knob for an FM channel is the RF CARRIER-to-NOISE ratio at the
# discriminator input:
#
#     CNR_dB = 10*log10( P_carrier / (N0 * B_if) )
#
# with P_carrier = 1 (constant envelope), N0 the complex-baseband noise PSD,
# and B_if the IF (Carson) bandwidth B = 2*(dev_hz + audio_hi_hz). The
# resulting AUDIO SNR is a DERIVED quantity that depends on deviation,
# emphasis, and bandwidth (above threshold it tracks CNR dB-for-dB; the
# mapping is characterized in test_fm_channel_sim.py V2). Select with
# --profile fm_dataport --fm-cnr-db 20   (spelled "fm_dataport:cnr20" in
# harness cell strings). Passing --cell/--snr-schedule with an fm_* profile
# is a hard argparse error, so the old audio label can never be silently
# reinterpreted as a CNR.
#
# PROFILES (the three real radio paths + the legacy control):
#   fm_mic             mic/speaker path: pre-emph + limiter + splatter LPF ->
#                      FM -> AWGN(CNR) -> IF BPF -> discriminator -> de-emph
#                      -> 300-3000 Hz audio. Emphasis pair MATCHED (net ~flat,
#                      noise ~flat after de-emphasis).
#   fm_dataport        flat "9600" data port: TX injects AFTER pre-emphasis,
#                      RX taps the discriminator BEFORE de-emphasis. Amplitude
#                      response flat, but the noise is STILL TRIANGULAR (+6
#                      dB/oct): HIGH carriers are the weak ones. Limiter still
#                      present (deviation limiting is a transmitter law, not a
#                      courtesy of the mic path). DC passes (discriminator
#                      taps are DC-coupled; radio CFO appears as an audio DC
#                      offset = cfo_hz/dev_hz — a REAL effect Iris must eat).
#   fm_dataport_deemph data port where the radio DOES de-emphasize its RX
#                      audio out (some rigs do): flat-injected TX, de-emph RX.
#   flat               alias of "wgn": TODAY'S bench, unchanged — kept as the
#                      regression control (V5 byte-identity).
#
# UNKEYED-CARRIER GAPS (--fm-unkeyed-blast, default ON): the modem TX bridge
# fills inter-frame gaps with exact zeros (mean-square < SILENCE_EPS — same
# convention as the reader's silent flag). A real half-duplex peer UNKEYS
# there: the receiving discriminator loses the carrier and outputs the full
# no-quieting noise blast (squelch-open roar), NOT silence. That blast is
# what a real acquisition correlator has to reject. --fm-keyup-hang-chunks
# keeps the carrier up briefly across intra-burst pauses (PTT hang).
#
# All state is streaming (persistent FIR/IIR history, phase accumulator,
# discriminator sample memory) so the chain is continuous across the
# 1024-sample wire chunks — same pattern as AnalyticFilter/StreamingFIR.
# ===========================================================================
FM_PROFILES = {
    #                  TX emph  RX de-emph  dev(Hz)  audio lo-hi (Hz)  TX splatter LPF
    "fm_mic":             {"preemph": True,  "deemph": True,  "dev_hz": 2500.0,
                           "audio_lo_hz": 300.0, "audio_hi_hz": 3000.0, "tx_splatter": True},
    "fm_dataport":        {"preemph": False, "deemph": False, "dev_hz": 5000.0,
                           "audio_lo_hz": 0.0, "audio_hi_hz": 6000.0, "tx_splatter": False},
    "fm_dataport_deemph": {"preemph": False, "deemph": True,  "dev_hz": 5000.0,
                           "audio_lo_hz": 0.0, "audio_hi_hz": 6000.0, "tx_splatter": False},
}

FM_EMPH_CORNER_HZ_DEFAULT = 300.0    # TIA-603 / LMR practice; == Iris pre-comp corner
FM_IF_TAPS_DEFAULT = 257
FM_AUDIO_TAPS_DEFAULT = 1201


def _fm_audio_fir(numtaps, flo, fhi, fs):
    """Audio-path FIR for the FM chain whose SPEC edges are FLAT-passband
    points. The raw windowed-sinc design (_fir_bandpass_taps) places its
    -6 dB cutoff exactly AT the requested edges, which droops a data carrier
    sitting near a spec edge (a 300-3000 Hz radio audio path is FLAT at
    300/3000 and rolls off OUTSIDE the band). Widen the design edges by half
    the Blackman transition width (~5.98*fs/N) so the requested band is all
    passband. Validated by test_fm_channel_sim.py V3a (400/2800 Hz tones)."""
    tw = 5.98 * fs / numtaps
    lo_d = max(0.0, flo - tw / 2.0) if flo > 0.0 else 0.0
    hi_d = min(0.98 * fs / 2.0, fhi + tw / 2.0)
    h, gd = _fir_bandpass_taps(numtaps, lo_d, hi_d, fs)
    return StreamingFIR(h)


class OnePoleIIR:
    """Streaming 1st-order IIR  y[n] = b0*x[n] + b1*x[n-1] + p1*y[n-1].
    Coefficient design below follows gr-analog fm_emph.py (bilinear transform,
    prewarped corners)."""

    def __init__(self, b0, b1, p1):
        self.b0, self.b1, self.p1 = float(b0), float(b1), float(p1)
        self.x1 = 0.0
        self.y1 = 0.0

    def process(self, x):
        # scipy-free streaming DF1; chunk sizes are small (1024) and this is
        # a 1st-order loop, so a Python loop over numpy would dominate — use
        # lfilter-style recursion via numpy where possible. The recursion on
        # y forces a sequential loop; keep it in C-speed via list/float ops.
        out = np.empty(len(x), dtype=np.float64)
        b0, b1, p1 = self.b0, self.b1, self.p1
        x1, y1 = self.x1, self.y1
        for i, xi in enumerate(x):
            y = b0 * xi + b1 * x1 + p1 * y1
            out[i] = y
            x1 = xi
            y1 = y
        self.x1, self.y1 = x1, y1
        return out


def design_deemph(fs, corner_hz):
    """gr-analog fm_deemph: bilinear transform of H(s) = w_c/(s + w_c),
    w_c = 2*pi*corner_hz (tau = 1/w_c). 0 dB at DC, -6 dB/octave above."""
    w_c = 2.0 * math.pi * corner_hz
    w_ca = 2.0 * fs * math.tan(w_c / (2.0 * fs))     # prewarp
    k = -w_ca / (2.0 * fs)
    p1 = (1.0 + k) / (1.0 - k)
    b0 = -k / (1.0 - k)
    return OnePoleIIR(b0, b0, p1)                    # zero at z=-1


def design_preemph(fs, corner_hz, fh=-1.0):
    """gr-analog fm_preemph: bilinear transform of H(s) = (s+w_cl)/(s+w_ch),
    w_cl = 2*pi*corner_hz, upper flatten pole w_ch = 2*pi*fh (default
    0.925*fs/2). Gain-normalized to 0 dB at DC."""
    if fh <= 0.0 or fh >= fs / 2.0:
        fh = 0.925 * fs / 2.0
    w_cl = 2.0 * math.pi * corner_hz
    w_ch = 2.0 * math.pi * fh
    w_cla = 2.0 * fs * math.tan(w_cl / (2.0 * fs))
    w_cha = 2.0 * fs * math.tan(w_ch / (2.0 * fs))
    kl = -w_cla / (2.0 * fs)
    kh = -w_cha / (2.0 * fs)
    z1 = (1.0 + kl) / (1.0 - kl)
    p1 = (1.0 + kh) / (1.0 - kh)
    b0 = (1.0 - kl) / (1.0 - kh)
    # normalize to 0 dB at DC (H(z=1) = 1)
    g = abs(1.0 - p1) / (b0 * abs(1.0 - z1))
    return OnePoleIIR(g * b0, g * b0 * -z1, p1)


class StreamingFIRC:
    """Complex-capable streaming FIR (overlap-save history), same pattern as
    StreamingFIR but with complex128 state for the IF filter."""

    def __init__(self, h):
        self.h = np.asarray(h, dtype=np.float64)
        self.ntaps = len(self.h)
        self.hist = np.zeros(self.ntaps - 1, dtype=np.complex128)

    def process(self, x):
        x = np.asarray(x, dtype=np.complex128)
        buf = np.concatenate((self.hist, x))
        y = np.convolve(buf, self.h, mode="full")
        start = self.ntaps - 1
        out = y[start:start + len(x)]
        self.hist = buf[-(self.ntaps - 1):]
        return out


class FmRadioChannel:
    """Streaming FM radio link: audio in -> TX radio -> RF AWGN at CNR ->
    discriminator RX -> audio out. One instance per direction. See the block
    comment above for the physics and the CNR unit definition."""

    def __init__(self, prof, args, np_rng):
        self.rng = np_rng
        self.drive = float(getattr(args, "fm_drive", 1.0) or 1.0)
        self.preemph_on = bool(prof["preemph"])
        self.deemph_on = bool(prof["deemph"])
        self.limiter_on = int(getattr(args, "fm_limiter", 1)) != 0
        dev = float(getattr(args, "fm_dev_hz", 0.0) or 0.0)
        self.dev_hz = dev if dev > 0.0 else prof["dev_hz"]
        lo = float(getattr(args, "fm_audio_lo_hz", 0.0) or 0.0)
        hi = float(getattr(args, "fm_audio_hi_hz", 0.0) or 0.0)
        self.audio_lo = lo if lo > 0.0 else prof["audio_lo_hz"]
        self.audio_hi = hi if hi > 0.0 else prof["audio_hi_hz"]
        # IF (Carson) bandwidth: B = 2*(dev + f_max). CNR is referenced to it.
        ifbw = float(getattr(args, "fm_if_bw_hz", 0.0) or 0.0)
        self.if_bw_hz = ifbw if ifbw > 0.0 else 2.0 * (self.dev_hz + self.audio_hi)
        cnr_db = getattr(args, "fm_cnr_db", None)
        if cnr_db is None:
            raise ValueError("fm_* profile requires --fm-cnr-db (RF CNR in the "
                             "IF bandwidth; NOT the audio SNR3k / WGN label)")
        self.cnr_db = float(cnr_db)
        cnr_lin = 10.0 ** (self.cnr_db / 10.0)
        # complex AWGN per-sample variance: carrier power = 1 (constant
        # envelope), CNR = 1/(N0*B_if), N0 = sigma^2/fs
        #   => sigma^2 = fs / (cnr_lin * B_if)
        self.noise_var = FS / (cnr_lin * self.if_bw_hz)
        self.noise_std_rail = math.sqrt(self.noise_var / 2.0)   # per I/Q rail

        corner = float(getattr(args, "fm_emph_corner_hz", 0.0) or 0.0)
        self.emph_corner_hz = corner if corner > 0.0 else FM_EMPH_CORNER_HZ_DEFAULT
        self.pre = design_preemph(FS, self.emph_corner_hz) if self.preemph_on else None
        self.de = design_deemph(FS, self.emph_corner_hz) if self.deemph_on else None

        # TX splatter LPF (post-limiter, FCC >=12 dB/oct above ~3 kHz; we use
        # the same windowed-sinc as the audio filters — steeper, documented).
        atap = int(getattr(args, "fm_audio_taps", FM_AUDIO_TAPS_DEFAULT)
                   or FM_AUDIO_TAPS_DEFAULT)
        self.tx_splatter = None
        if prof["tx_splatter"]:
            self.tx_splatter = _fm_audio_fir(atap, self.audio_lo, self.audio_hi, FS)
        # RX audio filter (radio audio path / data-port LPF). lo=0 -> lowpass.
        self.rx_audio = _fm_audio_fir(atap, self.audio_lo, self.audio_hi, FS)
        # IF bandpass at Carson bandwidth: symmetric complex lowpass +-B/2.
        itap = int(getattr(args, "fm_if_taps", FM_IF_TAPS_DEFAULT)
                   or FM_IF_TAPS_DEFAULT)
        hif, _gd = _fir_bandpass_taps(itap, 0.0, self.if_bw_hz / 2.0, FS)
        self.if_fir = StreamingFIRC(hif)

        # FM modulator / discriminator state
        self.kf_w = 2.0 * math.pi * self.dev_hz / FS   # rad/sample per unit m
        self.phase = 0.0
        self.prev_s = 1.0 + 0.0j          # discriminator one-sample memory
        # CFO (RF) — reuse the relay-wide --cfo-hz; discriminator turns it
        # into an audio DC offset of cfo_hz/dev_hz (a real radio-netting
        # error). Applied as an RF rotation, so capture/clicks see it too.
        self.cfo_w = 2.0 * math.pi * float(getattr(args, "cfo_hz", 0.0)) / FS
        self.cfo_phase = 0.0

        # optional flat Rayleigh fading of the RF envelope (VHF mobile):
        # reuses the validated IONOS Gaussian-FIR Doppler tap (155fdc1).
        fd = float(getattr(args, "fm_fade_doppler_hz", 0.0) or 0.0)
        self.fade_tap = DopplerTap(fd, np_rng) if fd > 0.0 else None

        # unkeyed-carrier gap model (squelch-open blast) + PTT hang
        self.unkeyed_blast = int(getattr(args, "fm_unkeyed_blast", 1)) != 0
        self.hang_chunks = max(0, int(getattr(args, "fm_keyup_hang_chunks", 2)))
        self.hang_left = 0                # chunks of carrier-hold remaining
        # final audio rail clip (sound-card / radio audio stage rail). The
        # no-carrier blast swings ~fs/(2*pi*dev) >> 1; a real audio output
        # rails instead. 0 = off.
        self.audio_clip = float(getattr(args, "fm_audio_clip", 2.0))
        # diagnostics
        self.n_clipped = 0                # limiter-clipped samples (TX)
        self.n_unkeyed_chunks = 0

    def describe(self):
        return (f"dev={self.dev_hz:.0f}Hz audio={self.audio_lo:.0f}-"
                f"{self.audio_hi:.0f}Hz B_if={self.if_bw_hz:.0f}Hz "
                f"CNR={self.cnr_db:.1f}dB preemph={self.preemph_on} "
                f"deemph={self.deemph_on} corner={self.emph_corner_hz:.0f}Hz "
                f"limiter={self.limiter_on} drive={self.drive} "
                f"unkeyed_blast={self.unkeyed_blast}(hang={self.hang_chunks})")

    def process(self, x):
        """One CHUNK of TX audio in -> channel-impaired RX audio out (same
        length). All filters/accumulators are streaming across chunks."""
        x = np.asarray(x, dtype=np.float64)
        n = x.size

        # --- keyed-carrier detection (modem idle gaps are exact zeros) -----
        ms = float(np.mean(x * x)) if n else 0.0
        silent = ms < 1e-12               # same convention as reader SILENCE_EPS
        if not silent:
            self.hang_left = self.hang_chunks
            keyed = True
        elif self.hang_left > 0:
            self.hang_left -= 1
            keyed = True                  # PTT hang: carrier up, no modulation
        else:
            keyed = not self.unkeyed_blast   # blast mode: carrier truly drops

        # --- TX radio: [pre-emph] -> limiter -> [splatter LPF] -------------
        a = x * self.drive
        if self.pre is not None:
            a = self.pre.process(a)
        if self.limiter_on:
            clipped = np.abs(a) > 1.0
            self.n_clipped += int(np.count_nonzero(clipped))
            a = np.clip(a, -1.0, 1.0)
        if self.tx_splatter is not None:
            a = self.tx_splatter.process(a)

        # --- FM modulate: s = exp(j*2*pi*kf*cumsum(m)/fs) -------------------
        ph = self.phase + np.cumsum(a) * self.kf_w
        self.phase = float(ph[-1]) % (2.0 * math.pi)
        if keyed:
            s = np.exp(1j * ph)
        else:
            self.n_unkeyed_chunks += 1
            s = np.zeros(n, dtype=np.complex128)   # carrier unkeyed: no signal

        # --- RF: [fading] -> CFO -> complex AWGN at CNR -> IF bandpass ------
        if self.fade_tap is not None:
            s = s * self.fade_tap.advance(n)
        if self.cfo_w != 0.0:
            idx = np.arange(1, n + 1)
            cph = self.cfo_phase + self.cfo_w * idx
            s = s * np.exp(1j * cph)
            self.cfo_phase = float(cph[-1]) % (2.0 * math.pi)
        noise = (self.rng.standard_normal(n) + 1j * self.rng.standard_normal(n)) \
            * self.noise_std_rail
        s = s + noise
        s = self.if_fir.process(s)

        # --- discriminator: mhat = arg(s[n] conj(s[n-1])) * fs/(2 pi kf) ----
        sprev = np.concatenate(([self.prev_s], s[:-1]))
        self.prev_s = complex(s[-1])
        d = np.angle(s * np.conj(sprev))
        audio = d / self.kf_w             # back to modulation units

        # --- RX radio: [de-emph] -> audio filter -> rail clip ---------------
        if self.de is not None:
            audio = self.de.process(audio)
        audio = self.rx_audio.process(audio)
        out = audio / self.drive
        if self.audio_clip > 0.0:
            out = np.clip(out, -self.audio_clip, self.audio_clip)
        return out


# ---------------------------------------------------------------------------
# Per-direction channel.
# ---------------------------------------------------------------------------
# ===========================================================================
# Deterministic scripted CHANNEL-OUTAGE vehicle (P0 cross-session reconnect seam).
#
# WHY: the P0 silent-corruption reconnect-splice only reproduces when the modem
# tears an ESTABLISHED session down (a fat prefix already delivered onto the
# persistent app socket) and then RE-CONNECTS cross-session. A natural WGN draw
# either delivers-but-never-reconnects (clean SNR) or thrashes-but-never-delivers
# (low SNR), so neither reliably manufactures a CROSS-SESSION splice on a FAT
# prefix. This vehicle forces one deterministically: after the transfer has run
# for MERCURY_SIM_OUTAGE_AFTER_S virtual seconds (a fat prefix is on the socket),
# the channel goes SILENT (pure zeros, both directions) for
# MERCURY_SIM_OUTAGE_DUR_S seconds. The modem sees a real loss of signal ->
# link-timeout (arq_common.cc:6024, effective ~30 s) -> teardown -> a FRESH
# START_CONNECTION reconnect. DUR_S MUST EXCEED the modem link-timeout so the
# reconnect is a NEW cross-session session (a short outage = within-session
# recovery = the WRONG bug). After DUR_S the channel restores and the resumed
# transfer's first delivery lands far-forward -> the reconnect splice, if the bug
# is live, is exercised on a fat prefix.
#
# FAITHFULNESS: this is a pure CHANNEL event -- the relay/bridge zeroes what the
# far modem HEARS. No modem code path is bypassed or poked; the modem runs its
# real carrier-loss / BREAK / link-timeout / reconnect logic. It lives inside
# Channel.process(), so BOTH the TCP relay (-x sim) AND the real-audio snd-aloop
# bridge (realaudio_bridge_s32.py, which imports Channel) pick it up, and a single
# process-wide controller shared by the two per-direction Channel objects keeps
# the outage window synchronized across the forward + reverse directions.
#
# ENV KNOBS (all default OFF; fully inert / byte-identical when none is set):
#   MERCURY_SIM_OUTAGE_AFTER_S      PRIMARY trigger: virtual seconds of transfer
#                                   before the 1st outage (target ~180-220 so a
#                                   fat >=~50 KB prefix is delivered at a
#                                   delivering SNR). Fires synchronously in both
#                                   directions (both clocks advance together).
#   MERCURY_SIM_OUTAGE_DUR_S        outage length in seconds (MUST exceed the
#                                   modem link-timeout ~30 s; target 40-60).
#   MERCURY_SIM_OUTAGE_AFTER_BYTES  alternative trigger: cumulative SIGNAL wire
#                                   bytes (float64 samples x8) across both
#                                   directions before the 1st outage. A coarse
#                                   progress proxy (NOT decoded payload); prefer
#                                   AFTER_S. 0 = unused.
#   MERCURY_SIM_OUTAGE_COUNT        number of outages (default 1). Outage k>=2
#                                   re-arms at k x threshold.
# ===========================================================================
_OUTAGE_SIG_EPS_MS = 1e-9   # inbound mean-square above this == signal-bearing chunk


class _OutageController:
    """Process-wide scripted channel-outage state, shared by both per-direction
    Channel objects (forward + reverse) so the black-out window is synchronized.
    Thread-safe: gate() is called from the two reader/pump threads."""

    def __init__(self, after_s, after_bytes, dur_s, count, log=None):
        self.after_s = float(after_s)
        self.after_bytes = int(after_bytes)
        self.dur_s = float(dur_s)
        self.count = max(1, int(count))
        self.fired = 0
        self.active = False
        self.start_samp = 0
        self.clock_samp = 0          # shared virtual clock (max over directions)
        self.sig_bytes = 0           # cumulative signal wire bytes (both dirs)
        self._lock = threading.Lock()
        self._log = log

    def _emit(self, msg):
        if self._log:
            try:
                self._log(msg)
                return
            except Exception:
                pass
        sys.stderr.write(msg + "\n")
        sys.stderr.flush()

    def gate(self, n, is_signal, dir_samp):
        """Advance the shared clock + progress accumulator; return True iff the
        channel is CURRENTLY blacked out (the caller must mute this chunk to
        silence). Time-based so both directions black out together."""
        with self._lock:
            if dir_samp > self.clock_samp:
                self.clock_samp = dir_samp
            if is_signal:
                self.sig_bytes += n * 8
            vs = self.clock_samp / FS
            if self.active:
                if (self.clock_samp - self.start_samp) / FS >= self.dur_s:
                    self.active = False
                    self.fired += 1
                    self._emit("[OUTAGE] END #%d at t=%.2fs (dur=%.2fs sig_bytes=%d)"
                               % (self.fired, vs,
                                  (self.clock_samp - self.start_samp) / FS,
                                  self.sig_bytes))
                    return False
                return True
            if self.fired >= self.count:
                return False
            k = self.fired + 1
            trig = ((self.after_s > 0.0 and vs >= self.after_s * k + self.dur_s * self.fired)
                    or (self.after_bytes > 0 and self.sig_bytes >= self.after_bytes * k))
            if trig:
                self.active = True
                self.start_samp = self.clock_samp
                self._emit("[OUTAGE] START #%d at t=%.2fs (sig_bytes=%d dur_s=%.1f "
                           "after_s=%s after_bytes=%s)"
                           % (k, vs, self.sig_bytes, self.dur_s,
                              self.after_s, self.after_bytes))
                return True
            return False


_outage_singleton = None
_outage_built = False
_outage_build_lock = threading.Lock()


def _get_outage_controller():
    """Lazily build the process-wide outage controller from MERCURY_SIM_OUTAGE_*.
    Returns None (FULLY INERT -- Channel.process is byte-identical) unless BOTH a
    trigger (AFTER_S or AFTER_BYTES) AND DUR_S are set. The two per-direction
    Channel objects in a process share the one controller so the outage is
    synchronized across directions."""
    global _outage_singleton, _outage_built
    if _outage_built:
        return _outage_singleton
    with _outage_build_lock:
        if _outage_built:
            return _outage_singleton
        after_s = float(os.environ.get("MERCURY_SIM_OUTAGE_AFTER_S", "0") or 0)
        after_bytes = int(float(os.environ.get("MERCURY_SIM_OUTAGE_AFTER_BYTES", "0") or 0))
        dur_s = float(os.environ.get("MERCURY_SIM_OUTAGE_DUR_S", "0") or 0)
        count = int(float(os.environ.get("MERCURY_SIM_OUTAGE_COUNT", "1") or 1))
        if dur_s > 0.0 and (after_s > 0.0 or after_bytes > 0):
            _outage_singleton = _OutageController(after_s, after_bytes, dur_s, count)
            sys.stderr.write("[OUTAGE] armed: after_s=%s after_bytes=%s dur_s=%s count=%d\n"
                             % (after_s, after_bytes, dur_s, count))
            sys.stderr.flush()
        else:
            _outage_singleton = None      # inert
        _outage_built = True
        return _outage_singleton


class Channel:
    """Per-direction channel state (independent noise/fade per link)."""

    def __init__(self, args, rng_seed):
        self.snr_db = args.snr            # SNR3k in dB
        # ---- OPT-IN time-varying SNR schedule -------------------------
        # Parse '<virt_s>:<WGN_label>' edges keyed to the per-direction
        # virtual clock (sample_clock/FS). Sorted ascending by time. The
        # t=0 edge (if present) sets the starting SNR; later edges fire in
        # process() when the direction's virtual clock crosses them. The
        # Each label uses the measured affine-plus-endpoint-floor mapping,
        # exactly like --cell.
        self.snr_schedule = []            # list of (virt_s, snr3k_db)
        if getattr(args, "snr_schedule", None):
            for tok in args.snr_schedule.split(","):
                tok = tok.strip()
                if not tok:
                    continue
                ts, lbl = tok.split(":")
                self.snr_schedule.append(
                    (float(ts), ionos_wgn_to_snr3k(lbl)))
            self.snr_schedule.sort(key=lambda e: e[0])
            if self.snr_schedule and self.snr_schedule[0][0] <= 0.0:
                self.snr_db = self.snr_schedule[0][1]
        self._sched_idx = 0               # next un-fired edge
        self.loss = args.loss
        self.burst = args.burst
        self.profile = args.profile
        self.cfo_hz = args.cfo_hz
        self.phase_noise_deg = args.phase_noise_deg

        self.rng = Xoshiro(rng_seed)       # Python RNG for AWGN + burst
        np_rng = self.rng.seed_np()        # numpy RNG for taps + phase noise

        # ---- FM radio profile family (fm_*) ----------------------------
        # Real modulate/demodulate chain (see FM RADIO CHANNEL block above).
        # For every NON-fm profile this is None and NOTHING below changes:
        # no extra RNG draws, no state, no code-path difference (V5
        # byte-identity: tools/sim/test_fm_channel_sim.py).
        self.fm = None
        if self.profile in FM_PROFILES:
            self.fm = FmRadioChannel(FM_PROFILES[self.profile], args, np_rng)

        # ---- Inc 3 noise calibration ----------------------------------
        # P_sig is measured live (mean-square of TX passband). Start from a
        # sticky reference until real TX power is seen so the handshake's
        # first frames already sit in the calibrated noise floor.
        self.snr_lin = 10.0 ** (self.snr_db / 10.0)
        self.p_sig = max(args.sig_ref, 1e-6) ** 2   # sig_ref is an RMS; P=RMS^2
        self.p_sig_measured = False
        self.noise_std = self._noise_std_from_psig(self.p_sig)

        self.peak_diag = 0.0
        self.peak_ms = 0.0                 # sticky peak mean-square (power)

        # ---- HONEST-AXIS P_sig meter (default: steady) -------------------------
        # 'steady' (the DEFAULT) references the AWGN to the STEADY data-burst power:
        # P_sig = running median of active-chunk mean-squares, so a cell's SNR label
        # == the true in-band snr3k of the OFDM/MFSK DATA payload. The minority
        # (boosted) connect burst is a small fraction of active chunks and cannot
        # dominate a median, so it is ignored.
        #   MERCURY_SIM_PSIG_MODE=fix + MERCURY_SIM_PSIG_FIX=<power> -> hard-pin
        #        P_sig to a commanded value.
        # 'peak' is the LEGACY meter: it latches P_sig to the loudest chunk -- the
        # boosted connect burst -- so every subsequent DATA frame is measured
        # ~8-9 dB BELOW the label. That is a meter artifact, not a receiver loss; it
        # silently poisons the axis of any cohort that runs it. It is retained ONLY
        # as an explicit opt-in (MERCURY_SIM_PSIG_MODE=peak) for deliberate control
        # arms; it is no longer the default.
        self.psig_mode = os.environ.get("MERCURY_SIM_PSIG_MODE", "steady").strip().lower()
        try:
            self.psig_fix = float(os.environ.get("MERCURY_SIM_PSIG_FIX", "0") or 0.0)
        except ValueError:
            self.psig_fix = 0.0
        self._active_ms = []               # mean-square of every signal-bearing chunk
        self._psig_active_floor = 1e-7     # below this a chunk is treated as silence
        self._psig_recompute_every = 8     # median refresh cadence (active chunks)
        self._psig_since_recompute = 0
        self._psig_diag_samples = 0
        self._psig_diag_next = 0
        if self.psig_mode == "fix" and self.psig_fix > 0.0:
            self.p_sig = self.psig_fix
            self.p_sig_measured = True
            self.noise_std = self._noise_std_from_psig(self.p_sig)

        # ---- narrow-FM audio bandpass (radio audio filter) ------------
        # Default OFF -> the Mercury HF paths that share this Channel are
        # byte-identical. Enabled (narrow/wide) by the Iris real-audio matrix.
        bp_lo = float(getattr(args, "bandpass_lo_hz", 0.0) or 0.0)
        bp_hi = float(getattr(args, "bandpass_hi_hz", 0.0) or 0.0)
        bp_taps = int(getattr(args, "bandpass_taps", BANDPASS_DEFAULT_TAPS) or BANDPASS_DEFAULT_TAPS)
        self.bandpass_lo_hz = bp_lo
        self.bandpass_hi_hz = bp_hi
        if bp_hi > bp_lo > 0.0:
            h, _gd = _fir_bandpass_taps(bp_taps, bp_lo, bp_hi, FS)
            self.bandpass = StreamingFIR(h)
        else:
            self.bandpass = None

        # ---- Inc 4 Watterson taps -------------------------------------
        prof = PROFILES.get(self.profile)
        self.fading = prof is not None
        if self.fading:
            self.dtau_samp = max(1, int(round(prof["dtau"] * FS)))
            self.analytic = AnalyticFilter(numtaps=129)
            self.tap0 = DopplerTap(prof["fd"], np_rng)
            self.tap1 = DopplerTap(prof["fd"], np_rng)
            # delay line for the second (delayed) tap, in complex baseband.
            self.delay_buf = np.zeros(self.dtau_samp, dtype=np.complex128)
            # two equal-power taps -> normalize so mean |h|^2 = 1
            # (each tap unit-power; sum-power 2 -> scale by 1/sqrt(2))
            self.tap_scale = 1.0 / math.sqrt(2.0)
        else:
            self.analytic = None

        # ---- CFO + phase-noise rotation --------------------------------
        self.phase = 0.0                   # running carrier phase (rad)
        self.cfo_w = 2.0 * math.pi * self.cfo_hz / FS
        self.pn_std = math.radians(self.phase_noise_deg)
        self.np_rng_pn = np_rng

        # Gilbert-Elliott burst (orthogonal impulse knob)
        self.ge_state = 0
        self.sample_clock = 0
        # Deterministic scripted channel-outage vehicle (P0 cross-session
        # reconnect seam). Shared process-wide controller (None unless armed
        # via MERCURY_SIM_OUTAGE_*); inert when unset.
        self._outage = _get_outage_controller()

    def _noise_std_from_psig(self, p_sig):
        # per-sample real-noise stddev matching the BER harness (see header).
        #   var = P_sig * f_nyquist / (SNR_lin * BW_noise)
        var = p_sig * F_NYQUIST / (self.snr_lin * BW_NOISE)
        return math.sqrt(max(var, 0.0))

    def process(self, samples):
        x = np.asarray(samples, dtype=np.float64)
        n = x.size

        # --- FM radio profile family: the WHOLE channel is the FM chain ----
        # (audio -> TX radio -> RF AWGN at CNR -> discriminator -> audio).
        # The HF stages below (P_sig-calibrated audio AWGN, Watterson taps,
        # audio bandpass, burst/loss) do not apply to an RF FM link and are
        # bypassed. Non-fm profiles take the unchanged path below.
        if self.fm is not None:
            out = self.fm.process(x)
            self.sample_clock += n
            return out.tolist()

        # diag: track peak |signal| and update sticky TX power (mean-square).
        amax = float(np.max(np.abs(x))) if n else 0.0
        if amax > self.peak_diag:
            self.peak_diag = amax
        ms = float(np.mean(x * x)) if n else 0.0
        # peak_ms is ALWAYS tracked (diagnostic: the latched connect-burst power),
        # but whether it drives the noise floor depends on the meter mode.
        peak_advanced = ms > self.peak_ms
        if peak_advanced:
            self.peak_ms = ms
        if self.psig_mode == "steady":
            # STEADY axis: P_sig = median of active-chunk power. The connect burst is
            # a small minority of active chunks, so the median settles on the DATA
            # power and the data frames are measured AT the label.
            if ms > self._psig_active_floor:
                self._active_ms.append(ms)
                self._psig_since_recompute += 1
                if self._psig_since_recompute >= self._psig_recompute_every:
                    self._psig_since_recompute = 0
                    self.p_sig = float(np.median(self._active_ms))
                    self.p_sig_measured = True
                    self.noise_std = self._noise_std_from_psig(self.p_sig)
        elif self.psig_mode == "fix":
            pass  # hard-pinned in __init__; never track the peak
        elif peak_advanced:
            # legacy 'peak' mode: sticky peak-hold, byte-identical to the old path so
            # only update from chunks with real signal energy (silent gaps don't drag
            # P_sig -- and thus the noise floor -- toward 0).
            self.p_sig = self.peak_ms
            self.p_sig_measured = True
            self.noise_std = self._noise_std_from_psig(self.p_sig)
        # periodic clean-axis diagnostic (stdout -> bridge log): proves the meter is
        # pinned to the DATA power and is ignoring the connect-burst peak.
        self._psig_diag_samples += n
        if self._psig_diag_samples >= self._psig_diag_next:
            self._psig_diag_next = self._psig_diag_samples + 5 * 48000
            _med = float(np.median(self._active_ms)) if self._active_ms else 0.0
            _mean = float(np.mean(self._active_ms)) if self._active_ms else 0.0
            print(f"[PSIG_DIAG] mode={self.psig_mode} p_sig_used={self.p_sig:.6f} "
                  f"peak_ms_seen={self.peak_ms:.6f} active_median_ms={_med:.6f} "
                  f"active_mean_ms={_mean:.6f} n_active={len(self._active_ms)} "
                  f"noise_std={self.noise_std:.6f} snr_label={self.snr_db:.1f}",
                  flush=True)

        # --- 1. Watterson multipath fading (complex baseband) ----------
        if self.fading:
            z = self.analytic.process(x)                 # analytic signal
            g0 = self.tap0.advance(n) * self.tap_scale
            g1 = self.tap1.advance(n) * self.tap_scale
            # delayed branch via persistent delay line
            zd = np.empty(n, dtype=np.complex128)
            d = self.dtau_samp
            if n >= d:
                zd[:d] = self.delay_buf
                zd[d:] = z[:n - d]
                self.delay_buf = z[n - d:].copy()
            else:
                # chunk shorter than the delay (only on tiny tails): shift buf
                zd[:] = self.delay_buf[:n]
                self.delay_buf = np.concatenate(
                    (self.delay_buf[n:], z))[-d:]
            y = g0 * z + g1 * zd
        else:
            # no fading: keep it real-valued (pure AWGN reference path)
            y = x.astype(np.complex128)

        # --- CFO + phase-noise rotation --------------------------------
        # CFO is a DETERMINISTIC accumulating phase ramp (a constant frequency
        # offset). Phase noise is a BOUNDED per-sample jitter (RMS = pn_std),
        # added directly — NOT a cumsum/Wiener walk. A 0.2-deg/sample random
        # WALK would accumulate to ~0.2*sqrt(Nsamp) ~ tens of degrees of
        # unbounded drift over one frame and destroy MFSK/OFDM phase coherence
        # (it broke even a clean SNR3k=35 handshake). Real oscillator phase
        # noise is bounded; model it as additive zero-mean jitter on the
        # instantaneous phase.
        if self.cfo_hz != 0.0 or self.pn_std > 0.0:
            idx = np.arange(n)
            cfo_ph = self.phase + self.cfo_w * (idx + 1)   # accumulating ramp
            if self.pn_std > 0.0:
                jitter = self.np_rng_pn.standard_normal(n) * self.pn_std
            else:
                jitter = 0.0
            y = y * np.exp(1j * (cfo_ph + jitter))
            self.phase = float(cfo_ph[-1]) % (2.0 * math.pi)

        # back to real passband
        out = np.real(y)

        # --- 1b. narrow-FM audio bandpass (radio audio filter) ----------
        # Band-limit the SIGNAL only (before broadband receiver noise). Models
        # the radio's audio filter on the real OFDM signal. Off unless a caller
        # opts in (Mercury HF paths unaffected).
        if self.bandpass is not None:
            out = self.bandpass.process(out)

        # --- 2. AWGN at calibrated SNR3k (matches BER harness) ----------
        if self.noise_std > 0.0:
            # use the deterministic Xoshiro stream for reproducibility
            noise = np.fromiter((self.rng.gauss() for _ in range(n)),
                                dtype=np.float64, count=n)
            out = out + self.noise_std * noise

        # --- 3. orthogonal impulse / burst erasure (NOT multipath) ------
        if self.burst and self.loss > 0.0:
            p_g2b = self.loss * 0.05      # good->bad
            p_b2g = 0.03                  # bad->good (mean bad run ~33 samples)
            for i in range(n):
                if self.ge_state == 0:
                    if self.rng.uniform() < p_g2b:
                        self.ge_state = 1
                else:
                    out[i] = 0.0
                    if self.rng.uniform() < p_b2g:
                        self.ge_state = 0
        elif self.loss > 0.0:
            for i in range(n):
                if self.rng.uniform() < self.loss:
                    out[i] = 0.0

        self.sample_clock += n

        # --- OPT-IN SNR schedule: fire any edges the per-direction virtual
        # clock has now crossed. Recompute snr_lin + noise_std from the new
        # SNR3k via the SAME calibrated path (_noise_std_from_psig), so a
        # degrade edge raises the AWGN floor for all subsequent chunks. The
        # t=0 edge was applied in __init__; this only handles t>0 edges.
        if self.snr_schedule:
            virt_s = self.sample_clock / FS
            fired = False
            while (self._sched_idx < len(self.snr_schedule)
                   and virt_s >= self.snr_schedule[self._sched_idx][0]):
                self.snr_db = self.snr_schedule[self._sched_idx][1]
                self._sched_idx += 1
                fired = True
            if fired:
                self.snr_lin = 10.0 ** (self.snr_db / 10.0)
                self.noise_std = self._noise_std_from_psig(self.p_sig)

        # --- Deterministic scripted channel outage (P0 cross-session seam) ---
        # If armed (MERCURY_SIM_OUTAGE_*), the shared controller decides whether
        # the channel is currently blacked out; if so the forwarded output is
        # muted to pure silence so the far modem sees a real loss of signal ->
        # BREAK / link-timeout -> cross-session reconnect. No-op unless armed.
        # Placed AFTER the channel math so the RNG/tap state still advances.
        if self._outage is not None and self._outage.gate(
                n, ms > _OUTAGE_SIG_EPS_MS, self.sample_clock):
            out = np.zeros(n, dtype=np.float64)

        return out.tolist()


class Peer:
    def __init__(self, sock, role):
        self.sock = sock
        self.role = role
        self.alive = True


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except OSError:
            # Peer went away mid-recv. On Windows a forcibly-closed socket raises
            # ConnectionResetError (WinError 10054) instead of returning b'' the
            # way a graceful FIN does on POSIX. Treat BOTH as a clean close so the
            # reader thread exits quietly (no daemon-thread traceback) and the
            # scheduler tears down via the same `raw is None` path.
            return None
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def send_all(sock, data):
    try:
        sock.sendall(data)
        return True
    except OSError:
        return False


def parse_cell(cell):
    """Map ``WGN:N`` through the measured IONOS label compatibility axis."""
    s = cell.strip().upper()
    if s.startswith("WGN:"):
        return ionos_wgn_to_snr3k(s[4:])
    # bare number: treat as a literal SNR3k
    return float(s)


def main():
    ap = argparse.ArgumentParser(description="SIM ARQ channel relay (Watterson + SNR3k)")
    ap.add_argument("--port", type=int, default=52100)
    ap.add_argument("--snr", type=float, default=12.0,
                    help="channel SNR in dB referenced to 3 kHz (SNR3k). "
                         "Overridden by --cell.")
    ap.add_argument("--cell", default=None,
                    help="IONOS-compatible WGN label, e.g. WGN:-12; mapped "
                         "to measured external SNR3k (direct --snr is literal)")
    ap.add_argument("--snr-schedule", default=None,
                    help="OPT-IN time-varying SNR3k: comma list of "
                         "'<virt_s>:<WGN_label>' edges keyed to the per-direction "
                         "virtual clock, e.g. '0:40,30:18' (clean WGN:40 until 30 "
                         "virtual-s, then degrade to WGN:18). Labels use the "
                         "measured IONOS map, same as --cell. When set, "
                         "overrides --snr/--cell as the t=0 floor. Default off "
                         "(byte-identical static channel).")
    ap.add_argument("--sig-ref", type=float, default=0.15,
                    help="initial TX passband RMS reference for the noise floor "
                         "before live TX power is measured (default 0.15)")
    ap.add_argument("--profile",
                    choices=list(PROFILES.keys()) + list(FM_PROFILES.keys()),
                    default="wgn",
                    help="channel profile: wgn/flat (none), mpg/mpm/mpp (ITU "
                         "HF Watterson), or the FM radio family fm_mic / "
                         "fm_dataport / fm_dataport_deemph (REAL FM mod/demod "
                         "chain; requires --fm-cnr-db, refuses --cell/--snr "
                         "audio labels — see the FM RADIO CHANNEL block)")
    # ---- FM radio profile family (fm_*) knobs --------------------------
    # ALL default-inert for non-fm profiles (only read when an fm_* profile
    # is selected). CNR is the RF carrier-to-noise ratio in the IF (Carson)
    # bandwidth — an RF axis, deliberately NOT the audio SNR3k/WGN label.
    ap.add_argument("--fm-cnr-db", type=float, default=None,
                    help="RF carrier-to-noise ratio (dB) in the IF bandwidth "
                         "at the discriminator input. REQUIRED for fm_* "
                         "profiles; an error for others.")
    ap.add_argument("--fm-dev-hz", type=float, default=0.0,
                    help="peak FM deviation (Hz); 0 = profile default "
                         "(fm_mic 2500, fm_dataport* 5000)")
    ap.add_argument("--fm-drive", type=float, default=1.0,
                    help="TX audio drive: linear gain ahead of the radio; "
                         "|drive*audio| = 1.0 hits rated deviation / the "
                         "limiter (default 1.0)")
    ap.add_argument("--fm-limiter", type=int, default=1, choices=(0, 1),
                    help="deviation limiter (clip at rated deviation, "
                         "post-pre-emphasis). Real radios ALWAYS have one; "
                         "0 is a diagnostics-only bypass (default 1)")
    ap.add_argument("--fm-emph-corner-hz", type=float, default=0.0,
                    help="pre/de-emphasis corner Hz; 0 = default 300 "
                         "(TIA-603 LMR practice, tau~531us; 75us/2122Hz is "
                         "BROADCAST FM and wrong for NBFM)")
    ap.add_argument("--fm-if-bw-hz", type=float, default=0.0,
                    help="IF bandwidth Hz for the CNR reference + IF filter; "
                         "0 = Carson auto: 2*(dev + audio_hi)")
    ap.add_argument("--fm-audio-lo-hz", type=float, default=0.0,
                    help="radio audio path low edge Hz (0 = profile default)")
    ap.add_argument("--fm-audio-hi-hz", type=float, default=0.0,
                    help="radio audio path high edge Hz (0 = profile default)")
    ap.add_argument("--fm-if-taps", type=int, default=FM_IF_TAPS_DEFAULT,
                    help=f"IF complex FIR taps (default {FM_IF_TAPS_DEFAULT})")
    ap.add_argument("--fm-audio-taps", type=int, default=FM_AUDIO_TAPS_DEFAULT,
                    help=f"audio FIR taps (default {FM_AUDIO_TAPS_DEFAULT})")
    ap.add_argument("--fm-fade-doppler-hz", type=float, default=0.0,
                    help="optional flat Rayleigh RF fading Doppler spread Hz "
                         "(Gaussian PSD, the validated IONOS FIR tap); 0=off")
    ap.add_argument("--fm-unkeyed-blast", type=int, default=1, choices=(0, 1),
                    help="1 (default): carrier UNKEYS in modem idle gaps -> "
                         "squelch-open discriminator noise blast (the real "
                         "half-duplex acquisition environment). 0: carrier "
                         "always keyed (quiet gaps).")
    ap.add_argument("--fm-keyup-hang-chunks", type=int, default=2,
                    help="PTT hang: chunks (~21ms each) the carrier stays "
                         "keyed after the last signal chunk (default 2)")
    ap.add_argument("--fm-audio-clip", type=float, default=2.0,
                    help="RX audio rail clip (sound-card/audio-stage rail; "
                         "bounds the unkeyed blast). 0 = off. Default 2.0")
    ap.add_argument("--cfo-hz", type=float, default=0.0,
                    help="carrier frequency offset in Hz (default 0)")
    ap.add_argument("--phase-noise-deg", type=float, default=0.2,
                    help="per-sample phase-noise stddev in degrees (default 0.2)")
    ap.add_argument("--audio-bandpass", choices=list(BANDPASS_PRESETS.keys()),
                    default="off",
                    help="narrow-FM audio bandpass (radio audio filter): off "
                         "(default, full band -- Mercury HF), narrow (300-2900 Hz, "
                         "VARA FM narrow), wide (300-6300 Hz). Band-limits the "
                         "SIGNAL only. Applied to BOTH directions.")
    ap.add_argument("--bandpass-lo-hz", type=float, default=0.0,
                    help="explicit bandpass low edge Hz (>0 overrides preset)")
    ap.add_argument("--bandpass-hi-hz", type=float, default=0.0,
                    help="explicit bandpass high edge Hz (>0 overrides preset)")
    ap.add_argument("--bandpass-taps", type=int, default=BANDPASS_DEFAULT_TAPS,
                    help=f"FIR bandpass taps (default {BANDPASS_DEFAULT_TAPS})")
    ap.add_argument("--loss", type=float, default=0.0,
                    help="impulse/burst erasure fraction (0..1); orthogonal to "
                         "fading (NOT the multipath proxy)")
    ap.add_argument("--burst", action="store_true",
                    help="Gilbert-Elliott bursty impulse dropout (else memoryless)")
    # retained for backward-compat with old harness invocations (flat fade);
    # superseded by --profile. If --fade-hz>0 and profile=wgn, warn + ignore.
    ap.add_argument("--fade-hz", type=float, default=0.0,
                    help="(DEPRECATED) old flat-fade Hz; use --profile instead")
    ap.add_argument("--fade-depth", type=float, default=0.0,
                    help="(DEPRECATED) old flat-fade depth; use --profile instead")
    ap.add_argument("--seed", type=int, default=1)
    # ---- CONNECT-REACK T1: deterministic single-ACK loss (test-only) ---------
    # Erase (zero the channel input of) the Nth SIGNAL BURST on the b2a
    # (RSP->CMD) direction, 1-based. Burst #1 is the START_CONNECTION ACK,
    # burst #2 is the FIRST TEST_CONNECTION_ACK. --erase-b2a-burst 2 thus
    # deterministically loses the single TEST_ACK while the START_CONNECTION ACK
    # still lands -> the responder reaches CONNECTED, then every duplicate
    # TEST_CONNECTION must be re-answered (connect-testack-handshake.md §5/T1).
    # DEFAULT 0 == disabled == byte-identical channel.
    ap.add_argument("--erase-b2a-burst", type=int, default=0,
                    help="TEST-ONLY: zero the Nth RSP->CMD signal burst (1-based; "
                         "2 = first TEST_CONNECTION_ACK). DEFAULT 0 (disabled).")
    # ---- Inter-peer sample-clock drift + PTT turnaround model (OPT-IN) -------
    # DEFAULT OFF (0). On == FIX9 mechanism repro (sample-rate skew that
    # de-aligns the half-duplex CFG16 turnaround); see DriftResampler /
    # PttLatencyModel above. Drift is applied AFTER the calibrated channel math
    # (noise/taps stay calibrated); ppm 0 == strict byte-identical pass-through.
    ap.add_argument("--drift-ppm-a2b", type=float, default=0.0,
                    help="sample-clock drift (ppm) on the A->B forwarded stream. "
                         "rate_out/rate_in = 1 + ppm/1e6. DEFAULT 0 (off, "
                         "byte-identical). FIX9 HW measured ~-670 ppm relative "
                         "skew that de-aligns the CFG16 half-duplex turnaround.")
    ap.add_argument("--drift-ppm-b2a", type=float, default=0.0,
                    help="sample-clock drift (ppm) on the B->A forwarded stream. "
                         "DEFAULT 0 (off). Set independently from a2b (the two HW "
                         "soundcards drift independently: CLK-TX -670, CLK-RX -193).")
    ap.add_argument("--ptt-latency-ms", type=float, default=0.0,
                    help="half-duplex keying latency injected at each TX onset "
                         "(silent->signal edge) per direction, in ms. DEFAULT 0 "
                         "(off, byte-identical). Models the radio PTT + AGC-settle "
                         "+ capture-flush turnaround the sim idealizes to zero.")
    ap.add_argument("--ptt-latency-jitter-ms", type=float, default=0.0,
                    help="symmetric uniform jitter (ms) on --ptt-latency-ms per "
                         "onset (seeded). DEFAULT 0. Requires --ptt-latency-ms>0.")
    # ---- FAITHFUL turnaround-timing de-alignment model (SIMFIDELITY M1-M4) ----
    # This is the CORRECTED drift mechanism (SIMFIDELITY_ROOTCAUSE.md): a
    # turnaround-timing WINDOW-MISS (the reverse ACK lands outside the CMD's fixed
    # listen window due to accumulated PTT/scheduling jitter + the PHYSICAL ±8 ppm
    # slip), realized by INTEGER insert/drop of WHOLE SILENCE samples — SIGNAL
    # samples (the ACK tones) pass BIT-EXACT. Enable with --turnaround-drift. This
    # is the DEFAULT faithful path; the old --drift-ppm-* tone-resampler is kept
    # behind its flag but is the WRONG axis (tone-smear, not window-miss).
    ap.add_argument("--turnaround-drift", action="store_true",
                    help="enable the FAITHFUL turnaround-timing window-miss model "
                         "(M1-M4, SIMFIDELITY_ROOTCAUSE). Injects ±ppm crystal "
                         "slip + per-key-up jitter as integer silence insert/drop "
                         "(tones bit-exact), and relaxes the PDES barrier. The "
                         "RIGHT axis for the CFG16 reverse-ACK collapse + D2 A/B.")
    ap.add_argument("--turnaround-ppm-a2b", type=float, default=8.16,
                    help="PHYSICAL crystal slip (ppm) on A->B for --turnaround-drift "
                         "(default +8.16, CLOCK_VERDICT DIR1). NOT the -670 [CLK-TX] "
                         "artifact (~80x too large; a producer-push quantization).")
    ap.add_argument("--turnaround-ppm-b2a", type=float, default=-8.16,
                    help="PHYSICAL crystal slip (ppm) on B->A for --turnaround-drift "
                         "(default -8.16, CLOCK_VERDICT DIR2; reciprocal sign).")
    ap.add_argument("--turnaround-jitter-ms", type=float, default=6.0,
                    help="bounded SEEDED per-PTT-key-up turnaround jitter (ms, "
                         "symmetric uniform) for --turnaround-drift — the DOMINANT "
                         "trigger that random-walks the ACK arrival index out of "
                         "the CMD window. DEFAULT 6.0. Calibrated to bench-7.")
    ap.add_argument("--idle-bigstep", type=int, default=1,
                    help="Q3 FTRT speed-up (EXPERIMENTAL, default OFF=1): when BOTH "
                         "directions are inbound-silent (modem idle gaps), coalesce up "
                         "to this many silence chunks into one forwarded chunk per "
                         "direction, advancing both virtual clocks by the SAME step "
                         "(phase-locked). Only helps when paired with a flat-out modem "
                         "TX-idle (audioio sim_tx_idle_pace yield); that pairing breaks "
                         "CONNECT (see sim-ftrt-speedup-floor.md §7/§8), so it is OFF by "
                         "default. 1 = strict 1:1 (CONNECT-safe). >1 enables coalescing.")
    ap.add_argument("--barrier-k", type=int, default=1,
                    help="conservative-PDES window-barrier credit (chunks). "
                         "Neither direction's cumulative stamp counter may run "
                         "more than K chunks ahead of the other, bounding the "
                         "a2b/b2a virtual-clock split to K*1024 samples "
                         "(~K*21ms). K=1 = strict lockstep (default); relax to "
                         "4-8 if it throttles throughput materially.")
    ap.add_argument("--realtime", type=int, default=None, choices=(0, 1),
                    help="V2 WALL-CLOCK real-time pacing (sim-virtual-testbed-design "
                         "§2.6; fact-documents/data-flow-sim-realtime-pacing.md). When 1, "
                         "the forwarder releases AT MOST one chunk per direction per "
                         "1024/48000 s (=21.333 ms) of WALL clock, so the capture ring is "
                         "fed at true 48 kHz instead of as-fast-as-the-host-computes (the "
                         "FTRT cheat). This DECOUPLES the capture ring from decode (a late "
                         "reverse-ACK now arrives late in wall time and can MISS its window "
                         "-> the turnaround miss EMERGES). RT forces --idle-bigstep OFF and "
                         "supersedes the PDES barrier (wall-clock IS the inter-peer "
                         "coupling). DEFAULT: env MERCURY_SIM_REALTIME (0/1), else 0 "
                         "(FTRT, byte-identical to baseline). RT pacing changes only WHEN a "
                         "chunk is released, never its CONTENT, so a render is bit-identical "
                         "modulo arrival time; with RT off the scheduler is the original "
                         "flat-out loop (N2 byte-identity preserved).")
    ap.add_argument("--wire-stamp", type=int, default=0, choices=(0, 1),
                    help="prepend the 8-byte <Q per-direction END-sample stamp to "
                         "each forwarded chunk (8192->8200 B). DEFAULT 0 (bare "
                         "8192, compatible with every shipped -x sim mercury). "
                         "Set 1 ONLY when the binary under test reads the stamp "
                         "(feat/sim-clock modem half, '[SIM] RX bridge first "
                         "vstamp=' canary) — a non-stamp modem silently corrupts "
                         "8 bytes/chunk. See _simcal_takeover/ASSESSMENT.md.")
    ap.add_argument("--log", default=None)
    ap.add_argument("--airtime-json", default=None,
                    help="write the GAP-#1 per-direction airtime breakdown (signal "
                         "vs silence forwarded-chunk counts, virtual airtime seconds) "
                         "to this path on shutdown. The harness reads it to derive the "
                         "HW-representative per-frame wire rate (delivered_bytes*8 / "
                         "airtime_secs). No-op when unset (back-compatible).")
    args = ap.parse_args()
    if args.barrier_k < 1:
        ap.error("--barrier-k must be >= 1")

    # ---- fm_* profile argument hygiene (the CNR/SNR units firewall) ------
    # The WGN/SNR3k label is an AUDIO-domain axis; fm_* profiles are driven
    # by RF CNR. Refuse every combination that could silently reinterpret
    # one as the other (this project has produced an 8.4x metric error from
    # exactly such a silent unit reuse).
    fm_on = args.profile in FM_PROFILES
    if fm_on:
        if args.fm_cnr_db is None:
            ap.error(f"--profile {args.profile} requires --fm-cnr-db (RF CNR "
                     "in the IF bandwidth). --cell/--snr audio labels are NOT "
                     "accepted for FM profiles.")
        if args.cell:
            ap.error("--cell (audio SNR3k/WGN label) cannot be combined with "
                     "an fm_* profile. Use --fm-cnr-db (RF CNR).")
        if getattr(args, "snr_schedule", None):
            ap.error("--snr-schedule is an audio-SNR3k axis and is not "
                     "supported for fm_* profiles (Phase 1).")
        if args.audio_bandpass != "off" or args.bandpass_lo_hz > 0.0 \
                or args.bandpass_hi_hz > 0.0:
            ap.error("--audio-bandpass is the flat-bench radio-audio-filter "
                     "model; fm_* profiles own their audio filters "
                     "(--fm-audio-lo-hz/--fm-audio-hi-hz). Use those.")
        if args.burst or args.loss > 0.0:
            ap.error("--burst/--loss (audio-domain erasure) are not wired "
                     "into fm_* profiles (Phase 1).")
    elif args.fm_cnr_db is not None:
        ap.error("--fm-cnr-db only applies to fm_* profiles (got --profile "
                 f"{args.profile}). For flat profiles use --snr/--cell.")

    drift_on = (args.drift_ppm_a2b != 0.0 or args.drift_ppm_b2a != 0.0)
    ptt_on = (args.ptt_latency_ms > 0.0)
    turn_on = bool(args.turnaround_drift)
    # V2 real-time pacing (default-off byte-identical). CLI --realtime wins; else
    # the MERCURY_SIM_REALTIME env (the SAME env the modem bridges read so a probe
    # can flip BOTH halves with one variable); else 0 (FTRT baseline). Read once
    # here, never mid-run, so there is no torn read.
    if args.realtime is not None:
        realtime_on = bool(args.realtime)
    else:
        realtime_on = (os.environ.get("MERCURY_SIM_REALTIME", "0") not in ("0", "", None))
    if (args.ptt_latency_jitter_ms > 0.0) and not ptt_on:
        ap.error("--ptt-latency-jitter-ms requires --ptt-latency-ms > 0")
    # The faithful turnaround model is the CORRECTED axis; the old tone-resampler
    # is the WRONG axis. They model the same physical event two different (and
    # incompatible) ways — refuse to run both at once so an A/B is unambiguous.
    if turn_on and drift_on:
        ap.error("--turnaround-drift (faithful window-miss) cannot be combined "
                 "with --drift-ppm-* (the old tone-resampler axis). Use one.")
    # The idle big-step COALESCES runs of silence (drops chunks) and is gated on
    # split==0. That defeats BOTH the drift resampler (it needs the CONTINUOUS
    # per-direction sample stream — dropped silence chunks would skip input), the
    # PTT onset-edge detector (a coalesced silence run hides the rising edge), and
    # the faithful turnaround model (it pays the timing offset down IN the silence
    # runs the big-step would elide). They are conceptually incompatible; refuse
    # the combination rather than silently produce a wrong model.
    if (drift_on or ptt_on or turn_on) and args.idle_bigstep > 1:
        ap.error("--drift-ppm-* / --ptt-latency-ms / --turnaround-drift cannot be "
                 "combined with --idle-bigstep > 1 (coalescing drops/merges chunks "
                 "and breaks the continuous drift stream + onset-edge detection)")

    # --wire-stamp phase-lock vs --idle-bigstep (sim-arq-channel.md §11.2 / fix
    # decision item D). The big-step COALESCES a run of silent chunks and forwards
    # only the LAST one's stamp. Under --wire-stamp both peers SET their clock to
    # that stamp, so a coalesced run produces a multi-hundred-ms FORWARD lurch in
    # virtual time across the handshake — which can overshoot a connect window and
    # silently break the very thing the phase-lock fixes. Refuse the combination
    # so a future sweep cell cannot enable both at once. Default --idle-bigstep 1
    # is OFF, so the failing connect path is already safe.
    if args.wire_stamp and args.idle_bigstep > 1:
        ap.error("--wire-stamp 1 cannot be combined with --idle-bigstep > 1: the "
                 "big-step coalesces silent chunks and forwards only the last "
                 "stamp, lurching the relay-stamped virtual clock forward across "
                 "the handshake (overshoots connect windows). Use --idle-bigstep 1.")

    if args.cell:
        args.snr = parse_cell(args.cell)

    # Resolve the audio-bandpass preset into the numeric edges the Channel reads.
    bp_lo, bp_hi, bp_taps = resolve_bandpass(
        args.audio_bandpass, args.bandpass_lo_hz, args.bandpass_hi_hz, args.bandpass_taps)
    args.bandpass_lo_hz = bp_lo
    args.bandpass_hi_hz = bp_hi
    args.bandpass_taps = bp_taps

    logf = open(args.log, "w") if args.log else sys.stdout

    def log(msg):
        ts = time.strftime("%H:%M:%S")
        logf.write(f"[{ts}] {msg}\n")
        logf.flush()

    if (args.fade_hz > 0 or args.fade_depth > 0) and args.profile == "wgn":
        log("WARN: --fade-hz/--fade-depth are deprecated and ignored; "
            "use --profile {mpg,mpm,mpp} for multipath fading.")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(4)
    prof = PROFILES.get(args.profile)
    if fm_on:
        prof_s = "FM radio chain (see FM RADIO CHANNEL header)"
    else:
        prof_s = "none" if prof is None else f"dtau={prof['dtau']*1e3:.1f}ms fd={prof['fd']}Hz"
    log(f"relay listening on 127.0.0.1:{args.port} "
        f"SNR3k={args.snr:.2f}dB ({'cell '+args.cell if args.cell else 'snr'}) "
        f"profile={args.profile} ({prof_s}) cfo={args.cfo_hz}Hz "
        f"phase_noise={args.phase_noise_deg}deg "
        f"burst={args.burst} loss={args.loss} seed={args.seed} "
        f"barrier_k={args.barrier_k} "
        f"drift_ppm(a2b={args.drift_ppm_a2b},b2a={args.drift_ppm_b2a}) "
        f"ptt_latency_ms={args.ptt_latency_ms}(jit={args.ptt_latency_jitter_ms}) "
        f"turnaround_drift={turn_on}(ppm a2b={args.turnaround_ppm_a2b},"
        f"b2a={args.turnaround_ppm_b2a},jit={args.turnaround_jitter_ms}ms) "
        f"wire={'STAMPED(8200,needs feat/sim-clock modem)' if args.wire_stamp else 'BARE(8192,compatible)'}")
    if getattr(args, "snr_schedule", None):
        log(f"SNR-SCHEDULE (opt-in, keyed to per-direction virtual clock): "
            f"{args.snr_schedule}  (IONOS measured label->SNR3k map; "
            f"overrides --snr/--cell as the t=0 floor)")
    if realtime_on:
        log("NOTE: V2 WALL-CLOCK real-time pacing ENABLED (MERCURY_SIM_REALTIME) — "
            "forwarder releases one chunk/direction per 21.333 ms wall clock (true "
            "48 kHz drip); idle-bigstep FORCED OFF, PDES barrier superseded by the "
            "wall clock. The capture ring is decoupled from decode -> the turnaround "
            "window-miss can EMERGE. Signal CONTENT is bit-exact (only release TIME "
            "is paced); RT-OFF is byte-identical to baseline.")
    if realtime_on:
        log("NOTE: V2 WALL-CLOCK real-time pacing ENABLED (MERCURY_SIM_REALTIME) — "
            "forwarder releases one chunk/direction per 21.333 ms wall clock (true "
            "48 kHz drip); idle-bigstep FORCED OFF, PDES barrier superseded by the "
            "wall clock. The capture ring is decoupled from decode -> the turnaround "
            "window-miss can EMERGE. Signal CONTENT is bit-exact (only release TIME "
            "is paced); RT-OFF is byte-identical to baseline.")
    if turn_on:
        log("NOTE: FAITHFUL turnaround-drift model ENABLED (SIMFIDELITY M1-M4) — "
            "integer silence insert/drop re-times bursts (signal samples "
            "BIT-EXACT), ±ppm crystal slip + per-key-up jitter walk the reverse "
            "ACK out of the CMD window; PDES barrier RELAXED (drift-mode only). "
            "This is the corrected window-miss vehicle (NOT the tone-smear axis).")
    elif drift_on or ptt_on:
        log("NOTE: inter-peer drift/PTT model ENABLED — the conservative-PDES "
            "barrier no longer makes this a byte-identical deterministic A/B; "
            "this is the FIX9 turnaround-de-alignment repro vehicle.")

    # Accept exactly two peers (A and B).
    peers = {}
    while len(peers) < 2:
        sock, _ = srv.accept()
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        tag = recv_exact(sock, 1)
        if tag is None:
            sock.close()
            continue
        role = tag.decode("ascii", "ignore")
        peers[role] = Peer(sock, role)
        log(f"peer connected role='{role}' ({len(peers)}/2)")

    a = peers.get("A")
    b = peers.get("B")
    if a is None or b is None:
        log("ERROR: need both role A and role B; got " + ",".join(peers.keys()))
        return 1

    # Independent channel per direction (A->B and B->A): independent RNGs,
    # tap realizations, and fade phase (RF links are reciprocal in mean but
    # not sample-wise).
    ch_a2b = Channel(args, args.seed * 2654435761 & 0xFFFFFFFF)
    ch_b2a = Channel(args, args.seed * 40503 + 7 & 0xFFFFFFFF)
    if ch_a2b.fm is not None:
        log(f"NOTE: FM RADIO profile '{args.profile}' ENABLED — real "
            f"mod/demod chain per direction: {ch_a2b.fm.describe()}. The "
            f"SNR3k number in the line above is IGNORED for this profile; "
            f"the operating point is --fm-cnr-db={args.fm_cnr_db} (RF CNR "
            f"in B_if). Expect triangular discriminator noise, the ~10dB "
            f"CNR threshold knee, deviation limiting, and squelch-open "
            f"noise blasts in unkeyed gaps (--fm-unkeyed-blast).")

    # Per-direction drift resampler + PTT-latency model (OPT-IN; identity/inert
    # when the ppm / latency are 0 -> byte-identical default). Applied AFTER
    # ch.process() in the reader. Independent seeds for the PTT jitter so the two
    # directions' keyer jitter is uncorrelated (like two physical radios).
    drift = {"a2b": DriftResampler(args.drift_ppm_a2b),
             "b2a": DriftResampler(args.drift_ppm_b2a)}
    ptt = {"a2b": PttLatencyModel(args.ptt_latency_ms, args.ptt_latency_jitter_ms,
                                  Xoshiro(args.seed * 2246822519 & 0xFFFFFFFF)),
           "b2a": PttLatencyModel(args.ptt_latency_ms, args.ptt_latency_jitter_ms,
                                  Xoshiro(args.seed * 3266489917 & 0xFFFFFFFF))}
    # CONNECT-REACK T1 single-burst eraser (TEST-ONLY). Only the b2a (RSP->CMD)
    # direction can carry a TEST_ACK; a2b is disabled (target 0 = inert).
    eraser = {"a2b": BurstEraser(0),
              "b2a": BurstEraser(args.erase_b2a_burst)}
    # FAITHFUL turnaround-timing model (M1-M3; SIMFIDELITY_ROOTCAUSE §3.2). Per
    # direction: ±ppm crystal slip + seeded per-key-up jitter realized as integer
    # silence insert/drop (signal samples bit-exact). enabled only with
    # --turnaround-drift. Independent jitter seeds per direction (two radios).
    turn = {"a2b": TurnaroundDrift(args.turnaround_ppm_a2b, args.turnaround_jitter_ms,
                                   Xoshiro(args.seed * 2718281829 & 0xFFFFFFFF),
                                   enabled=turn_on),
            "b2a": TurnaroundDrift(args.turnaround_ppm_b2a, args.turnaround_jitter_ms,
                                   Xoshiro(args.seed * 3141592653 & 0xFFFFFFFF),
                                   enabled=turn_on)}

    stop = threading.Event()
    counters = {"a2b": 0, "b2a": 0}      # per-direction chunk counts
    # ---- GAP #1 airtime accounting (sim-fidelity throughput breakdown) -------
    # Split each direction's FORWARDED chunks into SIGNAL (the modem was actively
    # keying a frame on the wire — real OFDM/MFSK airtime) vs SILENCE (inter-frame
    # gaps, PTT turnaround, ACK-wait idle — the modem TX bridge floods memset(0)).
    # The classifier is the SAME `silent` flag the reader already computes on the
    # RAW pre-channel modem samples (an idle TX bridge sends exact zeros, so the
    # INBOUND raw mean-square is < SILENCE_EPS). This lets the harness derive the
    # per-frame CHANNEL wire rate = delivered_bytes*8 / (signal_chunks*1024/FS) —
    # the airtime-budget number that maps to the modem's own rbc and to HW — and
    # separate it from the wall-clock rate that is diluted by climb ramp + idle.
    # Coalesced (bigstep) silence advances the SILENCE count by the same step so
    # the airtime fraction is invariant to coalescing. NOTE: the drift/turnaround
    # model re-times the FORWARDED stream AFTER ch.process(), but the `silent` flag
    # is computed on the RAW INBOUND pre-channel sample (reader), so the SIGNAL vs
    # SILENCE classification is unaffected by drift — the airtime fraction reflects
    # the modem's keying, the intended axis.
    sig_chunks = {"a2b": 0, "b2a": 0}    # forwarded chunks carrying frame airtime
    sil_chunks = {"a2b": 0, "b2a": 0}    # forwarded chunks carrying inter-frame idle
    # Relay-stamped shared clock (Q3, sim-arq-channel.md §10.5b). The relay is the
    # SINGLE authoritative virtual-time source: each forwarded chunk carries an
    # 8-byte LE END-sample-index stamp; both peers SET g_sim_samples to
    # max(current, stamp) on RX (audioio.c). The stamp is the PER-DIRECTION END
    # index (counters[key]*CHUNK_SAMPLES): a peer's virtual clock is driven by
    # what the channel actually carried TO it in its RX direction, so the
    # commander's ACK-timeout window and the responder's reply share the b2a
    # timeline that carries the reply (kills the §10.4 turnaround wall) and a
    # silence flood can no longer multiply virtual time (kills the §9.6 idle warp,
    # because the consumer SETs to the relay count instead of ADDing locally).
    # NOTE (§10.7): shared-SUM (2x fast) and shared-MAX (tracks the fast
    # data-flood direction) were both A/B-falsified — they broke CONNECT by
    # running the commander clock too fast. Per-direction keeps the commander on
    # the slow b2a (reply) channel, which is the correct coupling.

    # ---- Conservative-PDES window barrier (Q3 lockstep, sim-arq-channel.md
    # §11.7 residual) -----------------------------------------------------
    # PRIOR BUG: the two directions were forwarded by two INDEPENDENT free-running
    # daemon threads. OS scheduling let one direction's pump out-run the other by
    # hundreds of virtual seconds (measured a2b=597s vs b2a=288s on a WGN:0 cell).
    # Because each peer's virtual clock = the relay's PER-DIRECTION RX stamp
    # (counters[key]*CHUNK_SAMPLES), that pump divergence IS an inter-peer
    # virtual-clock divergence: it slides the commander's CONFIG_0 post-TX
    # reverse-preamble search window past the actual ACK arrival (acquisition
    # starvation) and, run-to-run, drives the >10x delivered-rate variance that
    # swamped the floor A/Bs.
    #
    # FIX: ONE round-robin scheduler forwards at most one chunk per direction per
    # round, gated by a per-direction CREDIT so neither direction's cumulative
    # chunk counter runs more than K chunks ahead of the other. This bounds the
    # a2b/b2a stamp split to K*CHUNK_SAMPLES (~K*21ms), making the two peers'
    # virtual clocks advance in lockstep regardless of OS scheduling. Only the
    # SCHEDULING changes — the per-direction Watterson+AWGN channel math
    # (ch.process), the 8-byte per-direction END-sample stamp, and the wire
    # contract are all unchanged.
    #
    # NO-DEADLOCK: the modem's TX bridge injects silence chunks on TX gaps
    # (audioio.c sim_tx_bridge), so BOTH directions ALWAYS produce chunks. The
    # barrier only ever blocks waiting for the BEHIND direction to supply its
    # next chunk, which it always eventually does — the credit gate can never
    # starve both directions at once (one of them is always <= other + K).
    BARRIER_K = max(1, args.barrier_k)
    # M4 (SIMFIDELITY §3.2): the faithful turnaround model inserts/drops whole
    # SILENCE samples per direction, which changes the per-direction chunk COUNT
    # slightly (an inserted silence run emits an extra chunk; a dropped run emits
    # one fewer). With strict K=1 the barrier throttles whichever direction ran a
    # chunk ahead, re-coupling the clocks and re-absorbing the offset (it would
    # mask the de-alignment, FIX9 §3). So in --turnaround-drift mode ONLY, give a
    # MODEST extra credit so the insert/drop can compound across a batch WITHOUT
    # pinning a large inter-peer split (a large split de-syncs the CONNECT/climb
    # FSM — the physical de-alignment is only a few-hundred-samples << 1 chunk per
    # batch, NOT seconds). K=4 (~85 ms) is comfortably above the per-batch offset
    # and well below the ~1.3 s that desyncs CONNECT. drift-OFF keeps the user K
    # (default strict 1) => N2 byte-identity untouched (gated on turn_on; applied
    # AFTER --barrier-k so an explicit larger --barrier-k still wins).
    if turn_on:
        BARRIER_K = max(BARRIER_K, 4)
    # V2: under real-time pacing the per-direction wall clock IS the inter-peer
    # coupling (both directions release at 48 kHz wall-clock), so the K-chunk PDES
    # credit barrier is SUPERSEDED — give generous credit so the barrier never
    # throttles the wall pacer (which would re-introduce a host-compute coupling on
    # top of the wall coupling). Wall-clock keeps the split bounded by scheduling
    # jitter (sub-chunk), tighter than the K-chunk barrier (data-flow §5 INV-3).
    if realtime_on:
        BARRIER_K = max(BARRIER_K, 64)
    # Bounded inbound queues: a reader thread per direction blocks on the socket,
    # applies the channel, and hands the processed chunk to the forwarder. The
    # small maxsize back-pressures a reader whose direction is K-credit-blocked so
    # it cannot buffer the whole link ahead (which would re-decouple the clocks).
    inq = {"a2b": queue.Queue(maxsize=BARRIER_K + 1),
           "b2a": queue.Queue(maxsize=BARRIER_K + 1)}
    chans = {"a2b": ch_a2b, "b2a": ch_b2a}

    # Idle big-step (Q3 FTRT speed-up, sim-ftrt-speedup-floor.md §8). The modem's
    # TX-bridge idle path now floods SILENCE flat-out (audioio.c sim_tx_idle_pace
    # yields instead of Sleep(1)) so the relay receives idle chunks far faster than
    # real time. Forwarding them 1:1 would (a) FLOOD the receiving peer's capture
    # ring + prep correlator faster than real time and (b) let the two peers' connect
    # FSMs desync — that BROKE CONNECT (clean SNR35 -> 0 bytes). FIX: when BOTH
    # directions' next ready chunk is INBOUND-SILENT (the modem sent zeros — pure
    # inter-frame gap), the relay COALESCES: it forwards ONE physical chunk per
    # direction but advances BOTH per-direction counters by the SAME step S, so:
    #   * virtual time jumps S*1024 samples on BOTH peers IDENTICALLY (clocks stay
    #     phase-locked — the FSM-interleave the K=1 barrier alone didn't protect),
    #   * the receiving peer's pipeline gets only ~1/S the physical chunks (it is
    #     NOT flooded; it processes a real-time-rate trickle of noise),
    #   * the channel math (ch.process) still runs on EVERY received chunk in the
    #     reader (taps/RNG advance identically), so the noise/fade realization is
    #     byte-for-byte unchanged vs forwarding 1:1 — only WHICH processed chunks
    #     get a wire slot changes, and a coalesced run is pure noise floor anyway.
    # The instant EITHER direction carries signal energy, coalescing stops and the
    # frame is forwarded chunk-by-chunk normally (the HAIL/data is never skipped).
    # Disable with --idle-bigstep 1 to recover strict 1:1 (determinism A/B baseline).
    BIGSTEP = max(1, args.idle_bigstep)
    SILENCE_EPS = 1e-12   # inbound raw mean-square below this == modem idle silence
    # V2: real-time pacing REQUIRES strict 1:1 forwarding — coalescing idle silence
    # (bigstep) collapses a real wall-clock turnaround gap into one chunk, which is
    # exactly the FTRT time-compression RT removes. Force BIGSTEP=1 in RT (data-flow
    # §2.3 / §5 INV-4). (Production/FTRT path keeps the user --idle-bigstep.)
    if realtime_on:
        BIGSTEP = 1
    # V2 wall-clock release period: one CHUNK_SAMPLES block = 1024/48000 s of real
    # time per direction. The forwarder gates each direction independently on this
    # deadline so the capture ring is fed at true 48 kHz (sim-virtual-testbed-design
    # §2.1). monotonic clock; lazily initialized per direction on first forward.
    RT_CHUNK_PERIOD_S = CHUNK_SAMPLES / FS   # 21.333 ms
    rt_next_release = {"a2b": None, "b2a": None}   # per-direction wall deadline

    def reader(src, key, ch):
        """Block on the socket, run the channel, enqueue (processed_out, silent).
        Channel math + call order per direction are byte-for-byte the same as the
        old pump (one Channel, one thread, sequential ch.process per direction).
        `silent` flags that the INBOUND raw (pre-channel) was all-zero — i.e. the
        modem emitted an idle silence chunk, eligible for big-step coalescing."""
        while not stop.is_set():
            raw = recv_exact(src.sock, CHUNK_BYTES)
            if raw is None:
                log(f"{key}: source closed")
                stop.set()
                # unblock the forwarder if it is parked on a queue.get
                try:
                    inq[key].put_nowait(None)
                except queue.Full:
                    pass
                break
            samples = list(struct.unpack(f"<{CHUNK_SAMPLES}d", raw))
            # inbound idle detection on the RAW (pre-channel) modem samples: an
            # idle TX bridge sends memset(0). Cheap max-abs test.
            silent = True
            for v in samples:
                if v > SILENCE_EPS or v < -SILENCE_EPS:
                    silent = False
                    break
            # --- CONNECT-REACK T1 (TEST-ONLY): deterministically erase one whole
            # reverse burst (the single TEST_ACK). Runs on the RAW pre-channel
            # samples so the far end sees the calibrated noise floor in its place.
            # `silent` is recomputed from the erased samples below so airtime
            # accounting / onset edges treat the erased burst as silence. -------
            samples = eraser[key].maybe_erase(samples, silent)
            if eraser[key].enabled and eraser[key].last_onset:
                log(f"BURST-ERASER {key}: burst #{eraser[key].last_onset} onset "
                    f"(target={eraser[key].target}, "
                    f"erased_chunks={eraser[key].n_erased_chunks})")
            if not silent:
                silent = True
                for v in samples:
                    if v > SILENCE_EPS or v < -SILENCE_EPS:
                        silent = False
                        break
            # --- PTT keying latency (OPT-IN): on a silent->signal onset, inject
            # ptt_latency_ms of channel-noise SILENCE ahead of the burst so the
            # far end sees it arrive late (the half-duplex turnaround the sim
            # idealizes to zero). The injected silence is REAL channel output
            # (ch.process on zeros -> the calibrated noise floor) and advances the
            # channel RNG, so it is part of the per-direction stream the drift
            # resampler re-times. DEFAULT off -> n_inject==0, no-op. ----------
            n_inject = ptt[key].onset_delay_chunks(silent)
            wire = []                    # ordered list of CHUNK_SAMPLES blocks
            if n_inject > 0:
                zeros = [0.0] * CHUNK_SAMPLES
                for _ in range(n_inject):
                    sil_out = ch.process(zeros)          # noise-floor chunk
                    # injected PTT silence is always silent for the turnaround
                    # re-timer (it pays offset down there too); drift then turn.
                    for dc in drift[key].process(sil_out):
                        wire.extend(turn[key].process(dc, True))
            out = ch.process(samples)   # ALWAYS advance channel state (determinism)
            # --- inter-peer sample-clock drift (OPT-IN): re-time the forwarded
            # stream. ppm==0 -> identity (returns [out], byte-identical). --------
            # --- FAITHFUL turnaround re-timer (M1-M3, OPT-IN): integer silence
            # insert/drop that re-positions bursts; SIGNAL chunks bit-exact.
            # turnaround-OFF -> identity. drift and turnaround are mutually
            # exclusive (validated in main), so the chain is one or the other. -----
            for dc in drift[key].process(out):
                wire.extend(turn[key].process(dc, silent))
            # Enqueue every produced wire chunk in order (default off: exactly one
            # == the un-drifted `out`, so the queue cadence is unchanged). The
            # barrier/forwarder advances counters[key] by 1 per chunk regardless,
            # so N drift/PTT chunks are simply N forwarded chunks.
            for wc in wire:
                while not stop.is_set():
                    try:
                        inq[key].put((wc, silent), timeout=0.2)
                        break
                    except queue.Full:
                        continue
                if stop.is_set():
                    break

    CLOSED, FORWARDED, WOULDBLOCK = -1, 1, 0
    WIRE_STAMP = bool(args.wire_stamp)

    def _account(key, n, silent):
        """Attribute n forwarded chunks on `key` to SIGNAL (frame airtime) or
        SILENCE (inter-frame idle / turnaround) per the RAW-inbound silence flag.
        Pure bookkeeping for the GAP-#1 airtime breakdown; does not touch the wire
        or the barrier."""
        if silent:
            sil_chunks[key] += n
        else:
            sig_chunks[key] += n

    def _send_stamped(key, out):
        """Forward `out` to the destination peer. In --wire-stamp mode prepend the
        8-byte <Q per-direction END-sample stamp (relay-driven clock, needs the
        feat/sim-clock modem half); otherwise send bare CHUNK_BYTES (compatible
        with every shipped -x sim modem, local-ADD clock). The per-direction
        counter (and thus the barrier) advances IDENTICALLY in both modes — only
        the wire framing differs. Returns FORWARDED on success, CLOSED on close."""
        dst = b if key == "a2b" else a
        ch = chans[key]
        stamp = counters[key] * CHUNK_SAMPLES
        payload = struct.pack(f"<{CHUNK_SAMPLES}d", *out)
        packed = (struct.pack("<Q", stamp) + payload) if WIRE_STAMP else payload
        if not send_all(dst.sock, packed):
            log(f"{key}: dest closed")
            stop.set()
            return CLOSED
        if counters[key] % 500 == 0:
            dr = drift[key]
            pt = ptt[key]
            drift_s = ""
            if dr.enabled:
                # forwarded/input sample ratio realised so far (sanity vs ppm).
                ratio = (dr.out_total / dr.in_total) if dr.in_total else 0.0
                drift_s = (f" drift_ppm={dr.ppm:+.1f} out/in={ratio:.6f} "
                           f"(slip={dr.out_total - dr.in_total:+d}smp)")
            if pt.enabled:
                drift_s += f" ptt_onsets={pt.n_onsets} ptt_inj={pt.n_delay_chunks}ch"
            tr = turn[key]
            if tr.enabled:
                drift_s += (f" turn_ppm={tr.ppm:+.2f} onsets={tr.n_onsets} "
                            f"ins={tr.n_inserted} drop={tr.n_dropped} "
                            f"offset={tr.acc:+.1f}smp peak={tr.max_abs_offset:.1f}smp")
            log(f"{key}: {counters[key]} chunks "
                f"({counters[key]*CHUNK_SAMPLES/FS:.1f}s) "
                f"vstamp={stamp} split={counters['a2b']-counters['b2a']:+d} "
                f"P_sig={ch.p_sig:.5f} noise_std={ch.noise_std:.6f} "
                f"txpeak={ch.peak_diag:.4f}{drift_s}")
        return FORWARDED

    def forward_one(key):
        """If `key` has credit AND a processed chunk ready, stamp + send it.
        NON-blocking on the queue: returns WOULDBLOCK if the reader hasn't
        supplied the next chunk yet (so the scheduler can service the OTHER
        direction up to its credit), CLOSED on socket close, FORWARDED on send.
        The credit gate (counters[key] - counters[other] < K) is checked by the
        caller; here we only re-assert the no-deadlock contract via the queue."""
        other = "b2a" if key == "a2b" else "a2b"
        if counters[key] - counters[other] >= BARRIER_K:
            return WOULDBLOCK            # credit-blocked: wait for `other`
        # V2 real-time gate (root-cause pacing): release this direction's next
        # chunk only when WALL time crosses its per-direction 48 kHz deadline. Not
        # yet due -> WOULDBLOCK so the scheduler yields (the 0.5 ms reader-fill
        # sleep) and re-polls; the capture ring is therefore fed at true 48 kHz,
        # NOT as-fast-as-the-host-computes (the FTRT cheat removed). We check the
        # deadline BEFORE consuming the queue so a not-yet-due chunk is left in the
        # queue (no peek-and-discard, no reorder). RT off -> this whole block is
        # skipped and the forward is the original flat-out path (byte-identical).
        if realtime_on:
            now = time.monotonic()
            dl = rt_next_release[key]
            if dl is None:
                dl = now                 # first chunk of this direction: due now
            if now < dl:
                return WOULDBLOCK        # not yet due — wall-clock paced
        try:
            out, silent = inq[key].get_nowait()
        except queue.Empty:
            return WOULDBLOCK            # reader hasn't produced the chunk yet
        if out is None:                  # reader signalled close
            stop.set()
            return CLOSED
        if realtime_on:
            # Advance the deadline by exactly one chunk period from the SCHEDULED
            # release time (dl), not from `now`, so transient host scheduling jitter
            # does not accumulate into permanent slow drift. But if we have fallen
            # MORE than one period behind (host briefly oversubscribed), re-anchor
            # to `now` so we do not then burst-catch-up faster than real time (that
            # would re-introduce FTRT). Net: the long-run release rate is 48 kHz.
            now2 = time.monotonic()
            base = dl if (now2 - dl) < RT_CHUNK_PERIOD_S else now2
            rt_next_release[key] = base + RT_CHUNK_PERIOD_S
        counters[key] += 1
        _account(key, 1, silent)
        return _send_stamped(key, out)

    def try_bigstep():
        """If BOTH directions have a SILENT chunk ready AND the clocks are level
        (split==0, so a symmetric jump respects the K=1 barrier), coalesce a run
        of idle silence: pull up to BIGSTEP silent chunks from EACH direction,
        forward only the LAST of each (advancing both counters by the SAME drained
        count so the two virtual clocks jump IDENTICALLY), and drop the rest. This
        advances virtual time fast WITHOUT flooding either receive pipeline and
        WITHOUT desyncing the two connect FSMs (they jump together). Returns the
        number of chunks coalesced per direction (0 = not eligible this pass).
        Every dropped chunk's channel math already ran in the reader, so the noise
        realization is unchanged — only its wire slot is elided."""
        if BIGSTEP <= 1:
            return 0
        if counters["a2b"] != counters["b2a"]:
            return 0                     # not level: let the barrier re-level first
        # Peek both heads by getting one from each. If either is empty or NOT
        # silent, push back what we took and bail (forward path handles it).
        try:
            a_out, a_sil = inq["a2b"].get_nowait()
        except queue.Empty:
            return 0
        try:
            b_out, b_sil = inq["b2a"].get_nowait()
        except queue.Empty:
            # only a2b had one — hand it to the normal forward path by sending it.
            if a_out is None:
                stop.set(); return -1
            counters["a2b"] += 1
            _account("a2b", 1, a_sil)
            return -1 if _send_stamped("a2b", a_out) == CLOSED else 0
        if a_out is None or b_out is None:
            stop.set(); return -1
        if not (a_sil and b_sil):
            # at least one direction carries SIGNAL — forward both 1:1, no skip.
            counters["a2b"] += 1
            _account("a2b", 1, a_sil)
            if _send_stamped("a2b", a_out) == CLOSED:
                return -1
            counters["b2a"] += 1
            _account("b2a", 1, b_sil)
            if _send_stamped("b2a", b_out) == CLOSED:
                return -1
            return 0
        # both silent: keep draining additional silent chunks (up to BIGSTEP) that
        # are ALREADY queued, per direction, dropping all but the last.
        a_last, a_n = a_out, 1
        b_last, b_n = b_out, 1
        while a_n < BIGSTEP:
            try:
                o, s = inq["a2b"].get_nowait()
            except queue.Empty:
                break
            if o is None: stop.set(); return -1
            if not s:
                # signal arrived — forward the buffered silence then this signal.
                counters["a2b"] += a_n
                _account("a2b", a_n, True)        # coalesced run was all-silence
                if _send_stamped("a2b", a_last) == CLOSED: return -1
                counters["a2b"] += 1
                _account("a2b", 1, False)         # the arriving signal chunk
                if _send_stamped("a2b", o) == CLOSED: return -1
                a_last, a_n = None, 0
                break
            a_last, a_n = o, a_n + 1
        while b_n < BIGSTEP:
            try:
                o, s = inq["b2a"].get_nowait()
            except queue.Empty:
                break
            if o is None: stop.set(); return -1
            if not s:
                counters["b2a"] += b_n
                _account("b2a", b_n, True)        # coalesced run was all-silence
                if _send_stamped("b2a", b_last) == CLOSED: return -1
                counters["b2a"] += 1
                _account("b2a", 1, False)         # the arriving signal chunk
                if _send_stamped("b2a", o) == CLOSED: return -1
                b_last, b_n = None, 0
                break
            b_last, b_n = o, b_n + 1
        # forward the (possibly remaining) coalesced silence tail per direction.
        if a_n > 0:
            counters["a2b"] += a_n
            _account("a2b", a_n, True)            # coalesced silence tail
            if _send_stamped("a2b", a_last) == CLOSED: return -1
        if b_n > 0:
            counters["b2a"] += b_n
            _account("b2a", b_n, True)            # coalesced silence tail
            if _send_stamped("b2a", b_last) == CLOSED: return -1
        return max(a_n, b_n, 1)

    def scheduler():
        """Conservative-PDES window barrier + idle big-step. Each pass first tries
        try_bigstep() (coalesce a level, both-silent idle run — the FTRT speed-up
        for turnaround-dominated cells). Otherwise it forwards the BEHIND direction
        first (round-robin on ties), advancing a direction only while its counter
        is within BARRIER_K of the other's. With K=1 this is strict lockstep (the
        two counters never differ by more than 1), bounding the a2b/b2a clock split
        to 1*CHUNK_SAMPLES (~21ms); the symmetric big-step keeps split==0 so it
        respects the barrier. The loop only sleeps when NEITHER direction can
        progress this pass — both can never be credit-blocked at once, so this is a
        pure reader-fill wait, never a deadlock."""
        rr = 0
        while not stop.is_set():
            r = try_bigstep()
            if r < 0:
                return                   # CLOSED during big-step
            progressed = (r > 0)
            if not progressed:
                # behind direction first; alternate the tie-break each pass.
                cands = sorted(("a2b", "b2a"),
                               key=lambda k: (counters[k], 0 if k == ("a2b", "b2a")[rr % 2] else 1))
                for key in cands:
                    rf = forward_one(key)
                    if rf == CLOSED:
                        return
                    if rf == FORWARDED:
                        progressed = True
            rr += 1
            if not progressed and not stop.is_set():
                # No chunk ready on the direction(s) with credit — yield briefly
                # so the reader thread(s) can fill. (Pure I/O wait, not a stall:
                # both peers always emit chunks via the TX-bridge silence path.)
                time.sleep(0.0005)

    ta = threading.Thread(target=reader, args=(a, "a2b", ch_a2b), daemon=True)
    tb = threading.Thread(target=reader, args=(b, "b2a", ch_b2a), daemon=True)
    tsched = threading.Thread(target=scheduler, daemon=True)
    ta.start()
    tb.start()
    tsched.start()

    try:
        while not stop.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        stop.set()

    for p in (a, b):
        try:
            p.sock.close()
        except OSError:
            pass
    log(f"relay done. a2b={counters['a2b']} b2a={counters['b2a']} chunks "
        f"(final split={counters['a2b']-counters['b2a']:+d}, barrier_k={args.barrier_k}, "
        f"wire_stamp={int(WIRE_STAMP)})")

    # ---- GAP #1 airtime breakdown ------------------------------------------
    # Per direction: where did the simulated wall-clock (virtual channel time) go?
    #   SIGNAL chunks  -> real frame airtime (OFDM/MFSK keyed on the wire)
    #   SILENCE chunks -> inter-frame idle + PTT turnaround + ACK-wait
    # virtual airtime seconds = signal_chunks * CHUNK_SAMPLES / FS. The harness
    # divides delivered app bytes by the DELIVERING direction's airtime to get the
    # per-frame CHANNEL wire rate (maps to the modem's rbc and to HW), separate
    # from the wall-clock rate diluted by climb ramp + idle.
    def _airtime_s(key):
        return sig_chunks[key] * CHUNK_SAMPLES / FS

    def _total_s(key):
        return counters[key] * CHUNK_SAMPLES / FS

    for key in ("a2b", "b2a"):
        tot = counters[key]
        frac = (sig_chunks[key] / tot) if tot else 0.0
        log(f"airtime {key}: signal={sig_chunks[key]} silence={sil_chunks[key]} "
            f"total={tot} chunks | signal_airtime={_airtime_s(key):.2f}s "
            f"total_virtual={_total_s(key):.2f}s signal_frac={frac:.3f}")

    if args.airtime_json:
        try:
            with open(args.airtime_json, "w") as af:
                json.dump({
                    "fs": FS,
                    "chunk_samples": CHUNK_SAMPLES,
                    "barrier_k": args.barrier_k,
                    "wire_stamp": int(WIRE_STAMP),
                    "a2b": {
                        "signal_chunks": sig_chunks["a2b"],
                        "silence_chunks": sil_chunks["a2b"],
                        "total_chunks": counters["a2b"],
                        "signal_airtime_s": _airtime_s("a2b"),
                        "total_virtual_s": _total_s("a2b"),
                    },
                    "b2a": {
                        "signal_chunks": sig_chunks["b2a"],
                        "silence_chunks": sil_chunks["b2a"],
                        "total_chunks": counters["b2a"],
                        "signal_airtime_s": _airtime_s("b2a"),
                        "total_virtual_s": _total_s("b2a"),
                    },
                }, af, indent=1)
            log(f"wrote airtime breakdown -> {args.airtime_json}")
        except OSError as e:
            log(f"WARN: could not write airtime-json {args.airtime_json}: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

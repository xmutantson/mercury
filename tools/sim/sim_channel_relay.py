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
        MPG  GOOD     dtau=0.5 ms  fd=0.1 Hz
        MPM  MODERATE dtau=1.0 ms  fd=0.5 Hz
        MPP  POOR     dtau=2.0 ms  fd=1.0 Hz
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
SNR3k = EsN0_harness + 10*log10(3000/BW_signal). The validator
tools/test_sim_relay_noise.py checks the noise PSD match numerically.

  --cell WGN:N   convenience: SNR3k = N + 2.4 dB  (testbed WGN-label mapping;
                 channel SNR3k = WGN_label + 2.4, MEMORY testbed_wgn_snr3k_mapping)

=== Reproducing the HW pathologies ==========================================
  * clean    : --snr 30                          (over-climb to CONFIG_16, no collapse)
  * wgn10    : --cell WGN:10 --profile mpm        (climbs then collapses on fades)
  * wgn-10   : --cell WGN:-10 --profile mpp       (deep-SNR STALL on ROBUST/CONFIG_0)

Usage:
    python tools/sim/sim_channel_relay.py --port 52100 --snr 12 [--profile mpm]
            [--cfo-hz 0.0] [--phase-noise-deg 0.2] [--cell WGN:-10]
            [--burst --loss 0.02] [--seed 1] [--log relay.log]

Then launch two Mercury -x sim processes with MERCURY_SIM_ROLE A / B and the
same MERCURY_SIM_PORT. See tools/sim/sim_arq_channel.py for the full harness.
"""

import argparse
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

WGN_TO_SNR3K = 2.4            # channel SNR3k = WGN_label + 2.4 dB (testbed map)

# ITU-R F.1487 / ARSFI mid-latitude HF channel profiles.
#   delay spread dtau (s), Doppler spread fd (Hz, 2-sigma Gaussian).
PROFILES = {
    "wgn": None,                              # no fading (pure AWGN reference)
    "mpg": {"dtau": 0.5e-3, "fd": 0.1},       # GOOD
    "mpm": {"dtau": 1.0e-3, "fd": 0.5},       # MODERATE
    "mpp": {"dtau": 2.0e-3, "fd": 1.0},       # POOR
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
# Gaussian-Doppler tap generator (Watterson). Produces a complex Gaussian
# process with a Gaussian-shaped Doppler power spectrum, updated at a coarse
# rate (every UPDATE samples) and linearly interpolated per sample. The coarse
# samples are first-order low-pass filtered white complex Gaussian; the LPF
# time constant is set so the 3 dB Doppler bandwidth matches fd.
# ---------------------------------------------------------------------------
class DopplerTap:
    def __init__(self, fd_hz, rng_np, update=None):
        self.fd = fd_hz
        self.rng = rng_np
        # update interval: ~64x the Doppler rate (>> Nyquist for fd), bounded.
        if update is None:
            if fd_hz > 0:
                update = max(1, int(FS / (fd_hz * 64.0)))
            else:
                update = CHUNK_SAMPLES
        self.update = min(update, CHUNK_SAMPLES)
        self.dt = self.update / FS
        # Gaussian-Doppler: model as 1st-order IIR on complex white noise.
        # 3 dB bandwidth ~ fd -> alpha = exp(-2*pi*fd*dt). Scale the white
        # innovation so the steady-state tap variance is 1 (unit-power tap).
        if fd_hz > 0:
            self.alpha = math.exp(-2.0 * math.pi * fd_hz * self.dt)
        else:
            self.alpha = 1.0
        self.inno = math.sqrt(max(0.0, 1.0 - self.alpha * self.alpha))
        # state: current and next coarse tap value (complex), interp between.
        self.g_prev = self._white()
        self.g_next = self.alpha * self.g_prev + self.inno * self._white()
        self.pos = 0          # sample position within the current update span

    def _white(self):
        # unit-variance complex Gaussian: var(real)=var(imag)=1/2
        r = self.rng.standard_normal()
        i = self.rng.standard_normal()
        return (r + 1j * i) / math.sqrt(2.0)

    def advance(self, n):
        """Return an array of n complex tap gains, advancing internal state."""
        out = np.empty(n, dtype=np.complex128)
        k = 0
        while k < n:
            if self.fd <= 0:
                # static tap (no Doppler): constant unit gain
                out[k:] = self.g_prev
                self.pos = (self.pos + (n - k)) % self.update
                break
            span = self.update - self.pos
            take = min(span, n - k)
            # linear interpolation g_prev -> g_next across the update span
            t0 = self.pos
            frac = (np.arange(t0, t0 + take) + 0.5) / self.update
            out[k:k + take] = (1.0 - frac) * self.g_prev + frac * self.g_next
            k += take
            self.pos += take
            if self.pos >= self.update:
                self.pos = 0
                self.g_prev = self.g_next
                self.g_next = self.alpha * self.g_prev + self.inno * self._white()
        return out


# ---------------------------------------------------------------------------
# Per-direction channel.
# ---------------------------------------------------------------------------
class Channel:
    """Per-direction channel state (independent noise/fade per link)."""

    def __init__(self, args, rng_seed):
        self.snr_db = args.snr            # SNR3k in dB
        self.loss = args.loss
        self.burst = args.burst
        self.profile = args.profile
        self.cfo_hz = args.cfo_hz
        self.phase_noise_deg = args.phase_noise_deg

        self.rng = Xoshiro(rng_seed)       # Python RNG for AWGN + burst
        np_rng = self.rng.seed_np()        # numpy RNG for taps + phase noise

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

    def _noise_std_from_psig(self, p_sig):
        # per-sample real-noise stddev matching the BER harness (see header).
        #   var = P_sig * f_nyquist / (SNR_lin * BW_noise)
        var = p_sig * F_NYQUIST / (self.snr_lin * BW_NOISE)
        return math.sqrt(max(var, 0.0))

    def process(self, samples):
        x = np.asarray(samples, dtype=np.float64)
        n = x.size

        # diag: track peak |signal| and update sticky TX power (mean-square).
        amax = float(np.max(np.abs(x))) if n else 0.0
        if amax > self.peak_diag:
            self.peak_diag = amax
        ms = float(np.mean(x * x)) if n else 0.0
        # sticky power: only update from chunks with real signal energy so
        # silent gaps don't drag P_sig (and thus the noise floor) toward 0.
        if ms > self.peak_ms:
            self.peak_ms = ms
            self.p_sig = self.peak_ms
            self.p_sig_measured = True
            self.noise_std = self._noise_std_from_psig(self.p_sig)

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
    """--cell WGN:-12  ->  SNR3k dB.  channel SNR3k = WGN_label + 2.4."""
    s = cell.strip().upper()
    if s.startswith("WGN:"):
        return float(s[4:]) + WGN_TO_SNR3K
    # bare number: treat as a literal SNR3k
    return float(s)


def main():
    ap = argparse.ArgumentParser(description="SIM ARQ channel relay (Watterson + SNR3k)")
    ap.add_argument("--port", type=int, default=52100)
    ap.add_argument("--snr", type=float, default=12.0,
                    help="channel SNR in dB referenced to 3 kHz (SNR3k). "
                         "Overridden by --cell.")
    ap.add_argument("--cell", default=None,
                    help="convenience SNR spec, e.g. WGN:-12 (SNR3k = label+2.4)")
    ap.add_argument("--sig-ref", type=float, default=0.15,
                    help="initial TX passband RMS reference for the noise floor "
                         "before live TX power is measured (default 0.15)")
    ap.add_argument("--profile", choices=list(PROFILES.keys()), default="wgn",
                    help="fading profile: wgn (none), mpg/mpm/mpp (ITU HF)")
    ap.add_argument("--cfo-hz", type=float, default=0.0,
                    help="carrier frequency offset in Hz (default 0)")
    ap.add_argument("--phase-noise-deg", type=float, default=0.2,
                    help="per-sample phase-noise stddev in degrees (default 0.2)")
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
    ap.add_argument("--wire-stamp", type=int, default=0, choices=(0, 1),
                    help="prepend the 8-byte <Q per-direction END-sample stamp to "
                         "each forwarded chunk (8192->8200 B). DEFAULT 0 (bare "
                         "8192, compatible with every shipped -x sim mercury). "
                         "Set 1 ONLY when the binary under test reads the stamp "
                         "(feat/sim-clock modem half, '[SIM] RX bridge first "
                         "vstamp=' canary) — a non-stamp modem silently corrupts "
                         "8 bytes/chunk. See _simcal_takeover/ASSESSMENT.md.")
    ap.add_argument("--log", default=None)
    args = ap.parse_args()
    if args.barrier_k < 1:
        ap.error("--barrier-k must be >= 1")

    if args.cell:
        args.snr = parse_cell(args.cell)

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
    prof_s = "none" if prof is None else f"dtau={prof['dtau']*1e3:.1f}ms fd={prof['fd']}Hz"
    log(f"relay listening on 127.0.0.1:{args.port} "
        f"SNR3k={args.snr:.2f}dB ({'cell '+args.cell if args.cell else 'snr'}) "
        f"profile={args.profile} ({prof_s}) cfo={args.cfo_hz}Hz "
        f"phase_noise={args.phase_noise_deg}deg "
        f"burst={args.burst} loss={args.loss} seed={args.seed} "
        f"barrier_k={args.barrier_k} "
        f"wire={'STAMPED(8200,needs feat/sim-clock modem)' if args.wire_stamp else 'BARE(8192,compatible)'}")

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

    stop = threading.Event()
    counters = {"a2b": 0, "b2a": 0}      # per-direction chunk counts
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
            out = ch.process(samples)   # ALWAYS advance channel state (determinism)
            while not stop.is_set():
                try:
                    inq[key].put((out, silent), timeout=0.2)
                    break
                except queue.Full:
                    continue

    CLOSED, FORWARDED, WOULDBLOCK = -1, 1, 0
    WIRE_STAMP = bool(args.wire_stamp)

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
            log(f"{key}: {counters[key]} chunks "
                f"({counters[key]*CHUNK_SAMPLES/FS:.1f}s) "
                f"vstamp={stamp} split={counters['a2b']-counters['b2a']:+d} "
                f"P_sig={ch.p_sig:.5f} noise_std={ch.noise_std:.6f} "
                f"txpeak={ch.peak_diag:.4f}")
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
        try:
            out, _silent = inq[key].get_nowait()
        except queue.Empty:
            return WOULDBLOCK            # reader hasn't produced the chunk yet
        if out is None:                  # reader signalled close
            stop.set()
            return CLOSED
        counters[key] += 1
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
            return -1 if _send_stamped("a2b", a_out) == CLOSED else 0
        if a_out is None or b_out is None:
            stop.set(); return -1
        if not (a_sil and b_sil):
            # at least one direction carries SIGNAL — forward both 1:1, no skip.
            counters["a2b"] += 1
            if _send_stamped("a2b", a_out) == CLOSED:
                return -1
            counters["b2a"] += 1
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
                if _send_stamped("a2b", a_last) == CLOSED: return -1
                counters["a2b"] += 1
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
                if _send_stamped("b2a", b_last) == CLOSED: return -1
                counters["b2a"] += 1
                if _send_stamped("b2a", o) == CLOSED: return -1
                b_last, b_n = None, 0
                break
            b_last, b_n = o, b_n + 1
        # forward the (possibly remaining) coalesced silence tail per direction.
        if a_n > 0:
            counters["a2b"] += a_n
            if _send_stamped("a2b", a_last) == CLOSED: return -1
        if b_n > 0:
            counters["b2a"] += b_n
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
    return 0


if __name__ == "__main__":
    sys.exit(main())

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
        f"drift_ppm(a2b={args.drift_ppm_a2b},b2a={args.drift_ppm_b2a}) "
        f"ptt_latency_ms={args.ptt_latency_ms}(jit={args.ptt_latency_jitter_ms}) "
        f"turnaround_drift={turn_on}(ppm a2b={args.turnaround_ppm_a2b},"
        f"b2a={args.turnaround_ppm_b2a},jit={args.turnaround_jitter_ms}ms) "
        f"wire={'STAMPED(8200,needs feat/sim-clock modem)' if args.wire_stamp else 'BARE(8192,compatible)'}")
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

#!/usr/bin/env python3
"""
test_sim_relay_drift.py — validate the SIM relay inter-peer drift / PTT model.

The drift model (DriftResampler) and PTT-latency model (PttLatencyModel) in
sim_channel_relay.py re-time the per-direction forwarded audio to reproduce the
HW CFG16 half-duplex turnaround de-alignment (FIX9_ROOTCAUSE.md: independent
RPi soundcard sample-clock skew, ~-670 ppm relative, that the conservative-PDES
barrier structurally masks). This test asserts the model's CONTRACTS:

  [1] DEFAULT OFF == byte-identical pass-through. ppm==0 -> the resampler returns
      the input chunk verbatim; latency_ms==0 -> no injection. This is the PDES
      determinism A/B baseline (a seeded WGN cell must be bit-for-bit identical
      with the model present-but-off).

  [2] DRIFT SANITY: the forwarded-stream out/in sample ratio matches the
      requested (1 + ppm/1e6) to high precision, over a long stream, for both
      signs and a range of magnitudes (incl the FIX9 -670 ppm).

  [3] CHANNEL-MATH UNTOUCHED: applying drift AFTER ch.process() does not change
      the calibrated channel. The model operates on the channel OUTPUT, so the
      noise/tap RNG (driven per inbound chunk inside ch.process) advances
      identically with drift on vs off -> the per-sample noise variance / SNR
      calibration are preserved (the noise/taps "stay calibrated" contract).

  [4] DRIFT CONTINUITY: the resampled stream is a faithful (interpolated)
      time-scaling of the input, not a glitchy chunk-edge artifact. A pure tone
      in -> a pure tone out at the drifted frequency, low residual.

  [5] PTT ONSET DETECTION + INJECTION: a silent->signal rising edge triggers
      ptt_latency_ms of injected silence-ahead chunks; no edge -> no injection;
      jitter varies the per-onset delay; signal->signal and signal->silent edges
      do NOT inject.

Run:  python tools/sim/test_sim_relay_drift.py
Exits 0 on PASS, 1 on FAIL.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_channel_relay as R


class Args:
    """Minimal stand-in for argparse.Namespace the Channel ctor reads."""
    def __init__(self, **kw):
        self.snr = 10.0
        self.loss = 0.0
        self.burst = False
        self.profile = "wgn"
        self.cfo_hz = 0.0
        self.phase_noise_deg = 0.0
        self.sig_ref = 0.15
        self.__dict__.update(kw)


def _drive(ppm, nchunks, seed=7):
    """Push nchunks of a deterministic signal through a DriftResampler; return
    (concatenated output samples, in_total, out_total)."""
    dr = R.DriftResampler(ppm)
    rng = np.random.default_rng(seed)
    out = []
    in_total = 0
    for c in range(nchunks):
        x = rng.standard_normal(R.CHUNK_SAMPLES) * 0.1
        in_total += x.size
        for wc in dr.process(x.tolist()):
            out.extend(wc)
    return np.asarray(out), in_total, dr.out_total


def main():
    ok = True

    print("=== FIX9 drift / PTT model validation ===\n")

    # ---- [1] DEFAULT OFF == byte-identical pass-through --------------------
    print("[1] ppm==0 / latency==0 -> byte-identical pass-through")
    dr0 = R.DriftResampler(0.0)
    rng = np.random.default_rng(123)
    identical = True
    for _ in range(50):
        x = (rng.standard_normal(R.CHUNK_SAMPLES) * 0.3).tolist()
        outs = dr0.process(x)
        if len(outs) != 1 or outs[0] != x:
            identical = False
            break
    pt0 = R.PttLatencyModel(0.0, 0.0, R.Xoshiro(1))
    inj_off = all(pt0.onset_delay_chunks(s) == 0
                  for s in (True, False, False, True, False))
    status = "OK" if (identical and inj_off) else "FAIL"
    if status == "FAIL":
        ok = False
    print(f"    resampler identity={identical}  ptt no-inject={inj_off}  [{status}]")

    # ---- [2] DRIFT SANITY: out/in ratio matches 1 + ppm/1e6 ---------------
    print("\n[2] forwarded out/in sample ratio == 1 + ppm/1e6")
    for ppm in (-670.0, -193.0, +250.0, +1000.0, -50.0):
        # long stream so the fractional accumulation converges; whole-chunk
        # emission truncates a partial tail, so allow a small tolerance that
        # shrinks with stream length.
        nch = 4000
        _, n_in, n_out = _drive(ppm, nch)
        ratio = n_out / n_in
        want = 1.0 + ppm / 1.0e6
        # tail truncation can drop up to <CHUNK_SAMPLES out + the un-emitted
        # carry; bound the error by a few chunks over the total.
        tol = 3.0 * R.CHUNK_SAMPLES / n_in
        err = abs(ratio - want)
        status = "OK" if err < tol else "FAIL"
        if status == "FAIL":
            ok = False
        print(f"    ppm={ppm:+8.1f}  out/in={ratio:.8f} want={want:.8f} "
              f"err={err:.2e} tol={tol:.2e}  [{status}]")

    # ---- [3] CHANNEL-MATH UNTOUCHED (noise variance preserved) ------------
    # Drive the SAME channel realization (same seed) twice: once forwarding the
    # channel output un-drifted, once through the drift resampler. The channel
    # output (and thus its noise variance) is IDENTICAL in both — drift only
    # re-times it. Assert the drift resampler's INPUT stream variance == the
    # channel's calibrated noise+signal variance (the model never touches it).
    print("\n[3] drift applied AFTER channel -> channel noise var preserved")
    P_SIG = 0.02
    snr3k = 6.0
    ch_ref = R.Channel(Args(snr=snr3k, profile="wgn", phase_noise_deg=0.0), 555)
    ch_drift = R.Channel(Args(snr=snr3k, profile="wgn", phase_noise_deg=0.0), 555)
    dr = R.DriftResampler(-670.0)
    resid_ref, resid_in = [], []
    for c in range(400):
        t = (np.arange(R.CHUNK_SAMPLES) + c * R.CHUNK_SAMPLES) / R.FS
        sig = math.sqrt(2.0 * P_SIG) * np.sin(2.0 * math.pi * R.CENTER_HZ * t)
        out_ref = np.asarray(ch_ref.process(sig.tolist()))
        out_drift = np.asarray(ch_drift.process(sig.tolist()))
        # the two channels (same seed) must produce IDENTICAL output (the drift
        # is downstream and does not feed back into ch.process).
        resid_ref.append(out_ref - sig)
        resid_in.append(out_drift - sig)
        _ = dr.process(out_drift.tolist())   # drift consumes but never alters ch
    var_ref = float(np.var(np.concatenate(resid_ref)))
    var_in = float(np.var(np.concatenate(resid_in)))
    ch_identical = (ch_ref.noise_std == ch_drift.noise_std)
    var_match = (abs(var_in - var_ref) / var_ref < 1e-9)
    status = "OK" if (ch_identical and var_match) else "FAIL"
    if status == "FAIL":
        ok = False
    print(f"    channel noise_std identical={ch_identical}  "
          f"input noise var {var_in:.3e} == ref {var_ref:.3e} ({var_match})  [{status}]")

    # ---- [4] DRIFT CONTINUITY: tone in -> drifted tone out, low residual ---
    # A 1500 Hz tone resampled by (1+ppm/1e6) should appear at f*(1+ppm/1e6).
    # Measure the dominant output frequency via a long FFT and confirm the shift
    # direction + that the output is a clean tone (no broadband resample noise).
    print("\n[4] drift continuity: pure tone -> clean drifted tone")
    ppm = -670.0
    dr = R.DriftResampler(ppm)
    f0 = 1500.0
    out = []
    nch = 3000
    for c in range(nch):
        t = (np.arange(R.CHUNK_SAMPLES) + c * R.CHUNK_SAMPLES) / R.FS
        x = np.sin(2.0 * math.pi * f0 * t)
        for wc in dr.process(x.tolist()):
            out.extend(wc)
    out = np.asarray(out)
    # input tone freq f0 maps to output freq f0/step == f0*(1+ppm/1e6) because
    # the OUTPUT is sampled at the nominal FS but represents (1/step)x time.
    win = np.hanning(out.size)
    spec = np.abs(np.fft.rfft(out * win))
    freqs = np.fft.rfftfreq(out.size, 1.0 / R.FS)
    f_peak = freqs[int(np.argmax(spec))]
    # The resampler steps the read pointer by `step = 1/(1+ppm/1e6)` INPUT samples
    # per OUTPUT sample, so output index k reads input at k*step: a tone at input
    # freq f0 becomes f0*step = f0/(1+ppm/1e6) at the output (sampled at nominal
    # FS). This is the FREQUENCY shift, which is the INVERSE of the duration/
    # sample-count scaling (out/in == 1+ppm/1e6, test [2]) — both self-consistent:
    # FEWER output samples (shorter audio) <=> HIGHER pitch, exactly faster-clock
    # playback. For ppm=-670 the tone moves UP to ~1501 Hz.
    f_want = f0 * dr.step          # == f0 / (1 + ppm/1e6)
    # spectral purity: peak bin energy fraction of total (a clean tone ~ all in
    # a few bins).
    purity = float(spec[int(np.argmax(spec))] / (np.sum(spec) + 1e-12))
    df = abs(f_peak - f_want)
    # FFT bin width
    binw = R.FS / out.size
    status = "OK" if (df < 3 * binw and purity > 0.01) else "FAIL"
    if status == "FAIL":
        ok = False
    print(f"    tone {f0}Hz ppm{ppm:+.0f} -> peak {f_peak:.3f}Hz want {f_want:.3f}Hz "
          f"(df={df:.3f}, binw={binw:.3f}) purity={purity:.4f}  [{status}]")

    # ---- [5] PTT ONSET DETECTION + INJECTION ------------------------------
    print("\n[5] PTT latency injects on silent->signal edge only")
    latency_ms = 100.0
    expect_chunks = int(round(latency_ms * R.FS / 1000.0 / R.CHUNK_SAMPLES))
    pt = R.PttLatencyModel(latency_ms, 0.0, R.Xoshiro(42))
    # idle, idle, ONSET(signal), signal, signal, silent, ONSET(signal)
    seq = [True, True, False, False, False, True, False]
    injects = [pt.onset_delay_chunks(s) for s in seq]
    # injection only on indices 2 and 6 (the two rising edges)
    edges_only = (injects[2] == expect_chunks and injects[6] == expect_chunks and
                  all(injects[i] == 0 for i in (0, 1, 3, 4, 5)))
    onsets_ok = (pt.n_onsets == 2)
    # jitter: vary per onset, bounded by +/- jitter
    ptj = R.PttLatencyModel(latency_ms, 40.0, R.Xoshiro(99))
    jvals = []
    state = True
    for _ in range(40):
        # alternate idle->signal repeatedly to make 20 onsets
        jvals.append(ptj.onset_delay_chunks(False) if state else
                     ptj.onset_delay_chunks(True))
        state = not state
    jchunks = [v for v in jvals if v > 0]
    lo = int(round((latency_ms - 40.0) * R.FS / 1000.0 / R.CHUNK_SAMPLES))
    hi = int(round((latency_ms + 40.0) * R.FS / 1000.0 / R.CHUNK_SAMPLES))
    jitter_bounded = all(lo <= v <= hi for v in jchunks) and len(set(jchunks)) > 1
    status = "OK" if (edges_only and onsets_ok and jitter_bounded) else "FAIL"
    if status == "FAIL":
        ok = False
    print(f"    edges_only={edges_only} (inj@onset={expect_chunks}ch) "
          f"n_onsets={pt.n_onsets} jitter_bounded={jitter_bounded} "
          f"jvals_seen={sorted(set(jchunks))}  [{status}]")

    print("\n" + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

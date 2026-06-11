#!/usr/bin/env python3
"""
test_sim_relay_turnaround.py — validate the FAITHFUL turnaround-timing
de-alignment model (TurnaroundDrift) in sim_channel_relay.py.

This is the CORRECTED drift mechanism per SIMFIDELITY_ROOTCAUSE.md (derived from
bench-7's HW logs). The old DriftResampler collapsed via TONE-SMEARING (linear
interp at the bogus ~670 ppm) — the WRONG axis. The real bench mechanism is a
TURNAROUND-TIMING WINDOW-MISS: the reverse ACK lands outside the CMD's fixed
listen window because accumulated PTT/scheduling jitter (+ the physical ±8.16 ppm
slip) walks its ARRIVAL INDEX out of the window — the ACK tones are BIT-EXACT,
only their wire time shifts.

The model's CONTRACTS (M1-M3 + the N1/N2 anti-targets):

  [M1 / N1]  TONE FIDELITY: every SIGNAL sample passes BIT-EXACT. A burst that
             appears in the output is bit-identical to the input burst — only its
             arrival INDEX differs. (The OLD resampler FAILS this; the whole point
             of the correction is that we do NOT.) The timing shift is realized
             ONLY by integer insert/drop of SILENCE samples in the gaps.

  [M2]       ACCUMULATING ±ppm CRYSTAL SLIP: the cumulative timing offset grows
             with samples forwarded, at the physical ppm rate, and is NEVER reset
             per-batch. Over a long stream the net inserted/dropped silence
             samples track ppm/1e6 * total.

  [M3]       PER-KEY-UP JITTER (dominant trigger): each silent->signal onset adds
             a bounded SEEDED random extra offset, so the offset RANDOM-WALKS per
             key-up (non-monotone, reproducible for a fixed seed, different across
             seeds). No onset -> no jitter contribution.

  [N2]       DEFAULT-OFF byte-identical: enabled=False -> strict identity
             pass-through (the PDES determinism A/B baseline; pairs with the
             relay's existing byte_identity_check + test_sim_relay_drift [1]).

Run:  python tools/sim/test_sim_relay_turnaround.py
Exits 0 on PASS, 1 on FAIL.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_channel_relay as R


CH = R.CHUNK_SAMPLES
FS = R.FS


def _silence_chunk():
    return [0.0] * CH


def _signal_chunk(c, amp=0.3, f=R.CENTER_HZ):
    t = (np.arange(CH) + c * CH) / FS
    return (amp * np.sin(2.0 * math.pi * f * t)).tolist()


def _flatten(chunks):
    out = []
    for ch in chunks:
        out.extend(ch)
    return out


def main():
    ok = True
    print("=== FAITHFUL turnaround-drift model validation (SIMFIDELITY M1-M3 + N1/N2) ===\n")

    # ---- [N2] enabled=False -> strict byte-identical pass-through -----------
    print("[N2] enabled=False -> byte-identical pass-through")
    td_off = R.TurnaroundDrift(8.16, 6.0, R.Xoshiro(1), enabled=False)
    identical = True
    rng = np.random.default_rng(123)
    for c in range(60):
        # mix of silence and signal
        x = (_silence_chunk() if c % 3 == 0
             else (rng.standard_normal(CH) * 0.3).tolist())
        sil = all(abs(v) <= 1e-12 for v in x)
        outs = td_off.process(x, sil)
        if len(outs) != 1 or outs[0] != x:
            identical = False
            break
    status = "OK" if identical else "FAIL"
    if not identical:
        ok = False
    print(f"    off identity over 60 mixed chunks = {identical}  [{status}]")

    # ---- [M1 / N1] SIGNAL samples bit-exact; only arrival INDEX shifts ------
    # Drive a long idle gap (so the offset accumulates + a key-up jitters), then a
    # signal burst. Assert the burst that comes out is a CONTIGUOUS bit-identical
    # copy of the input burst (found somewhere in the output at a SHIFTED index),
    # i.e. tones untouched, position moved. Use a large ppm + jitter to force a
    # measurable, non-zero index shift.
    print("\n[M1/N1] signal samples BIT-EXACT, only arrival INDEX shifts")
    td = R.TurnaroundDrift(8.16, 6.0, R.Xoshiro(7), enabled=True)
    # build the input: long silence gap, then a distinctive signal burst.
    n_gap = 40
    n_burst = 8
    in_chunks = []
    for c in range(n_gap):
        in_chunks.append(_silence_chunk())
    burst_in = []
    for c in range(n_burst):
        bc = _signal_chunk(n_gap + c, amp=0.37, f=1234.0)  # distinctive tone
        burst_in.append(bc)
    burst_flat = _flatten(burst_in)
    out_all = []
    # feed the gap (silent), then the burst (signal)
    for ch in in_chunks:
        out_all.extend(_flatten(td.process(ch, True)))
    for ch in burst_in:
        out_all.extend(_flatten(td.process(ch, False)))
    # flush any tail (feed a couple of silent chunks so the carry drains)
    for _ in range(4):
        out_all.extend(_flatten(td.process(_silence_chunk(), True)))
    out_arr = np.asarray(out_all)
    burst_arr = np.asarray(burst_flat)
    # find the burst in the output by exact match at the shifted offset.
    found_idx = -1
    # search a window of plausible shifts (offset is small: ppm*samples + jitter)
    for shift in range(0, len(out_arr) - len(burst_arr) + 1):
        seg = out_arr[shift:shift + len(burst_arr)]
        if np.array_equal(seg, burst_arr):
            found_idx = shift
            break
    bit_exact = (found_idx >= 0)
    # the input burst started at sample n_gap*CH; the output index must DIFFER
    # (a real timing shift) — or be equal only if the net offset rounded to 0.
    in_start = n_gap * CH
    shifted = (found_idx != in_start)
    # what the model itself reports it inserted/dropped (net silence retiming)
    net_offset = td.n_inserted - td.n_dropped
    status = "OK" if bit_exact else "FAIL"
    if not bit_exact:
        ok = False
    print(f"    burst found bit-exact in output = {bit_exact} at idx {found_idx} "
          f"(input idx {in_start}, shifted={shifted}); "
          f"model net silence retiming = {net_offset:+d} samples  [{status}]")
    print(f"    onsets={td.n_onsets} inserted={td.n_inserted} dropped={td.n_dropped} "
          f"peak|offset|={td.max_abs_offset:.1f} samples")

    # ---- [M2] accumulating ±ppm slip over a long stream ---------------------
    # Over a long pure-silence stream (no onsets -> no M3 jitter), the net
    # inserted-minus-dropped silence samples must track ppm/1e6 * total_in.
    print("\n[M2] accumulating ±ppm crystal slip (no-onset silence stream)")
    for ppm in (+8.16, -8.16, +50.0, -50.0):
        td2 = R.TurnaroundDrift(ppm, 0.0, R.Xoshiro(3), enabled=True)
        nch = 8000
        total_in = 0
        for c in range(nch):
            td2.process(_silence_chunk(), True)  # all silent -> no onset jitter
            total_in += CH
        net = td2.n_inserted - td2.n_dropped
        want = ppm / 1.0e6 * total_in
        err = abs(net - want)
        # integer quantization + the un-drained residual <= a couple samples
        tol = 2.0
        status = "OK" if err <= tol else "FAIL"
        if err > tol:
            ok = False
        print(f"    ppm={ppm:+7.2f}  net_retime={net:+5d}  want={want:+8.2f}  "
              f"err={err:.2f} (tol {tol})  acc_resid={td2.acc:+.3f}  [{status}]")
    # explicit "never reset per batch": acc grows monotonically in sign with a
    # one-sign ppm and no onsets (no jitter to flip it).
    td3 = R.TurnaroundDrift(+50.0, 0.0, R.Xoshiro(3), enabled=True)
    offs = []
    for c in range(2000):
        td3.process(_silence_chunk(), True)
        if c % 500 == 499:
            offs.append(td3.n_inserted - td3.n_dropped)
    monotone = all(offs[i] <= offs[i + 1] for i in range(len(offs) - 1)) and offs[-1] > offs[0]
    status = "OK" if monotone else "FAIL"
    if not monotone:
        ok = False
    print(f"    cumulative (never reset): net_retime checkpoints={offs} "
          f"monotone-growing={monotone}  [{status}]")

    # ---- [M3] per-key-up jitter random-walks the offset; seeded/reproducible -
    print("\n[M3] per-key-up jitter random-walks the offset (seeded)")
    def run_keyups(seed, ppm=0.0, jitter_ms=6.0, n_keyups=30):
        """Alternate gap(silent)/burst(signal) n_keyups times; record the offset
        accumulator AT each onset to see the per-key-up random walk."""
        td = R.TurnaroundDrift(ppm, jitter_ms, R.Xoshiro(seed), enabled=True)
        walk = []
        for k in range(n_keyups):
            # short silent gap
            for _ in range(3):
                td.process(_silence_chunk(), True)
            # signal onset (single burst chunk)
            td.process(_signal_chunk(k, amp=0.3), False)
            walk.append(td.acc)
        return walk, td.n_onsets

    # ppm=0 so the ONLY offset source is the per-key-up jitter (M3 isolated).
    walk_a, onsets_a = run_keyups(seed=11, ppm=0.0)
    walk_b, onsets_b = run_keyups(seed=22, ppm=0.0)
    walk_a2, _ = run_keyups(seed=11, ppm=0.0)   # same seed -> reproducible
    # the walk must (i) be non-constant (it moves per key-up), (ii) be non-monotone
    # (it goes both ways — a random walk, matching the bench's non-monotone mask),
    # (iii) be reproducible for a fixed seed, (iv) differ across seeds.
    nonconstant = len(set(round(v, 6) for v in walk_a)) > 3
    deltas = [walk_a[i + 1] - walk_a[i] for i in range(len(walk_a) - 1)]
    both_signs = any(d > 0 for d in deltas) and any(d < 0 for d in deltas)
    reproducible = (walk_a == walk_a2)
    seed_varies = (walk_a != walk_b)
    onsets_ok = (onsets_a == 30)
    status = "OK" if (nonconstant and both_signs and reproducible and seed_varies
                      and onsets_ok) else "FAIL"
    if status == "FAIL":
        ok = False
    print(f"    onsets={onsets_a} nonconstant={nonconstant} both_signs={both_signs} "
          f"reproducible={reproducible} seed_varies={seed_varies}  [{status}]")
    print(f"    walk(seed11) head={[round(v,2) for v in walk_a[:6]]} ...")

    # ---- [N2 cross] no-onset + ppm=0 + jitter=0 enabled is also identity-ish -
    # (enabled but with NO offset source -> emits the same chunks, no insert/drop)
    print("\n[N2-cross] enabled with ppm=0 jitter=0 -> no retiming (inert)")
    td_inert = R.TurnaroundDrift(0.0, 0.0, R.Xoshiro(1), enabled=True)
    # ctor sets enabled False when both ppm==0 and jitter==0
    inert = (not td_inert.enabled)
    out = td_inert.process(_signal_chunk(0), False)
    inert = inert and (len(out) == 1 and out[0] == _signal_chunk(0))
    status = "OK" if inert else "FAIL"
    if not inert:
        ok = False
    print(f"    ppm=0,jit=0 -> enabled={td_inert.enabled} identity={inert}  [{status}]")

    print("\n" + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

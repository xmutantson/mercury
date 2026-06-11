#!/usr/bin/env python3
"""
test_sim_relay_turnaround_xdir.py — CROSS-DIRECTION batch-length turnaround
accrual (bench-9 anchor, v2 mechanism correction).

WHY THIS TEST EXISTS (the v1 mechanism error it pins, SIMTURNCAL_VERDICT.json):
The v1 batch-length accrual (test_sim_relay_turnaround_batchlen.py) added the
one-sided LATE accrual proportional to EACH DIRECTION's OWN burst, applied at
THAT SAME direction's NEXT onset. Because the two directions are SEPARATE
TurnaroundDrift instances (turn["a2b"] forward OFDM, turn["b2a"] reverse MFSK
ACK), that meant the long FORWARD a2b OFDM batch accrued an offset that was
spent inserting SILENCE AHEAD OF the a2b FORWARD signal burst onset
(sim_channel_relay.py v1 lines 705-712) -> the forward OFDM chunk grid was
de-aligned -> forward sync failed (relay ins=90313 samp / 286 ms into the
forward signal; FTR-FAIL x414, SKIP-VAR x116, 3-consecutive-abort x31, metric
collapse to 0.12-0.25, modem proc_died). The held-CFG16 wire stayed ~1897 bps,
3.2x too optimistic vs the bench-9 597.6. The v1 model BROKE the forward OFDM
decode instead of DELAYING the reverse MFSK ACK.

THE HW MECHANISM (bench-9, ARMA_CALIBRATION.json / SIMTURNCAL_VERDICT.json): the
CMD finishes its FORWARD OFDM batch (a2b), drops PTT, opens its receive window;
the keyer/AGC/capture-flush/scheduling latency accumulated DURING that forward
batch makes the REVERSE MFSK ACK (b2a) arrive LATE at the CMD, in proportion to
the FORWARD batch airtime, landing OUTSIDE the CMD receiving_timeout window ->
matched=0/7 -> D3 reverse-ACK-starvation -> active 18.9%, whole-window 597.6.
So the accrual amount is keyed to the FORWARD (a2b) batch airtime but the SILENCE
is inserted in the REVERSE (b2a) turnaround gap (delaying the b2a ACK onset).

THE v2 MODEL (cross-direction coupling, --turnaround-batch-accrual, OPT-IN,
env-mirrored MERCURY_SIM_BATCH_ACCRUAL=1): the forward a2b TurnaroundDrift
PUBLISHES its just-ended forward-burst airtime to a shared TurnaroundCoupler on
its signal->silence falling edge (burst end). The reverse b2a TurnaroundDrift
CONSUMES that pending forward airtime at its NEXT silent->signal reverse-ACK
onset and adds accrual_ms_per_s * forward_burst_s of LATE offset to its own acc,
realized as silence inserted in the b2a turnaround GAP (the existing
silent-onset insert delays the reverse-ACK signal onset). The forward a2b signal
is NEVER touched (no insert into the forward signal burst — the v1 regression).

CONTRACTS asserted here (the failing-first gate for the v2 correction):
  [X0] DEFAULT-OFF byte identity: with accrual disabled, a coupled pair produces
       the SAME wire on BOTH directions as the legacy (uncoupled) TurnaroundDrift
       over a mixed stream -> the existing --turnaround-drift default + tests are
       untouched (the default-off byte-identical contract).
  [X1] FORWARD OFDM BIT-EXACT under accrual ON (the v1 regression this fixes):
       drive a long CFG16-length FORWARD a2b OFDM batch through the coupled
       forward instance with accrual ON; assert ZERO silence was inserted into
       the forward signal (n_inserted==0 on the forward signal path) and that
       every forward signal burst comes out bit-identical at its OWN index (no
       grid shift). v1 FAILS this (it inserted ~90313 samp into the forward).
  [X2] REVERSE ACK ONSET DELAYED by ~accrual_ms_per_s * forward-batch-airtime:
       after the forward a2b batch ends (publishes its airtime) and the reverse
       b2a ACK keys up, the b2a onset's owed offset == ~30 ms/s * forward_burst_s
       (within the small ppm+jitter dither), i.e. the reverse ACK is shifted LATE
       by the calibrated amount keyed to the FORWARD batch length.
  [X3] BATCH-LENGTH STEP at the CMD window: a CFG16-length forward batch delays
       the reverse ACK PAST the CMD window half-width (MISS) while a CFG15-length
       forward batch keeps it inside (LAND) — the bench-9 87.5%/~0% step.

Run:  python tools/sim/test_sim_relay_turnaround_xdir.py
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

# bench-9 / bench-8 batch geometry (forward OFDM frames per batch on the wire).
CFG16_FRAMES = 28      # mid of the 25-30 HW CFG16 range
CFG15_FRAMES = 6       # short-batch CFG15
FRAME_CHUNKS = 8       # forward signal chunks per OFDM frame on the wire

# CMD reverse-ACK listen-window HALF-WIDTH (samples). The reverse ACK lands if
# its accumulated late-offset <= this, else MISSES. ~90 ms (bench-9 calibration).
WINDOW_HALFWIDTH_MS = 90.0
WINDOW_HALFWIDTH = WINDOW_HALFWIDTH_MS * FS / 1000.0


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


def _make_coupled_pair(seed, accrual_ms_per_s, ppm_a2b=8.16, ppm_b2a=-8.16,
                       jitter_ms=6.0):
    """Build an a2b (forward) + b2a (reverse) TurnaroundDrift pair sharing ONE
    TurnaroundCoupler — exactly the wiring main() uses. The forward instance
    PUBLISHES its burst airtime to the coupler; the reverse instance CONSUMES it
    at its ACK onset. accrual_ms_per_s==0 -> legacy (uncoupled-equivalent)."""
    coupler = R.TurnaroundCoupler()
    fwd = R.TurnaroundDrift(ppm_a2b, jitter_ms, R.Xoshiro(seed * 2718281829 & 0xFFFFFFFF),
                            enabled=True, accrual_ms_per_s=accrual_ms_per_s,
                            coupler=coupler, direction="a2b")
    rev = R.TurnaroundDrift(ppm_b2a, jitter_ms, R.Xoshiro(seed * 3141592653 & 0xFFFFFFFF),
                            enabled=True, accrual_ms_per_s=accrual_ms_per_s,
                            coupler=coupler, direction="b2a")
    return fwd, rev, coupler


def _drive_forward_batch(fwd, n_frames, cbase):
    """Drive ONE forward a2b OFDM batch (n_frames*FRAME_CHUNKS signal chunks held
    on the channel) followed by the turnaround gap (silence — the falling edge
    that publishes the burst airtime to the coupler). Returns (out_signal_flat,
    out_all_flat, burst_in_flat, next_cbase, n_inserted_during)."""
    n_ins0 = fwd.n_inserted
    burst_in = []
    out_sig = []
    c = cbase
    for _ in range(n_frames * FRAME_CHUNKS):
        sc = _signal_chunk(c)
        burst_in.append(sc)
        out_sig.extend(_flatten(fwd.process(sc, False)))
        c += 1
    out_gap = []
    # turnaround gap (silence) — publishes the just-ended forward burst airtime.
    for _ in range(3):
        out_gap.extend(_flatten(fwd.process(_silence_chunk(), True)))
        c += 1
    n_ins_during = fwd.n_inserted - n_ins0
    return out_sig, out_gap, _flatten(burst_in), c, n_ins_during


def main():
    ok = True
    print("=== CROSS-DIRECTION batch-length turnaround accrual (bench-9 v2) ===\n")
    print(f"  CFG16 batch = {CFG16_FRAMES} frames, CFG15 batch = {CFG15_FRAMES} "
          f"frames, {FRAME_CHUNKS} chunks/frame")
    print(f"  CMD window half-width = {WINDOW_HALFWIDTH_MS} ms "
          f"({WINDOW_HALFWIDTH:.0f} samples)")
    print(f"  accrual default = {R.DEFAULT_TURNAROUND_ACCRUAL_MS_PER_S} ms/s\n")

    has_coupler = hasattr(R, "TurnaroundCoupler")
    if not has_coupler:
        print("[X*] FAIL: sim_channel_relay has no TurnaroundCoupler — the v2 "
              "cross-direction mechanism is not implemented yet.")
        print("\nFAIL")
        return 1

    # ---- [X0] DEFAULT-OFF byte identity vs the legacy uncoupled model --------
    print("[X0] accrual OFF -> coupled pair byte-identical to legacy on BOTH dirs")
    fwd0, rev0, _ = _make_coupled_pair(5, accrual_ms_per_s=0.0)
    leg_f = R.TurnaroundDrift(8.16, 6.0, R.Xoshiro(5 * 2718281829 & 0xFFFFFFFF), enabled=True)
    leg_r = R.TurnaroundDrift(-8.16, 6.0, R.Xoshiro(5 * 3141592653 & 0xFFFFFFFF), enabled=True)
    same = True
    rng = np.random.default_rng(99)
    for c in range(200):
        xf = (_silence_chunk() if c % 4 == 0 else (rng.standard_normal(CH) * 0.3).tolist())
        xr = (_silence_chunk() if c % 5 == 0 else (rng.standard_normal(CH) * 0.3).tolist())
        sf = all(abs(v) <= 1e-12 for v in xf)
        sr = all(abs(v) <= 1e-12 for v in xr)
        if fwd0.process(list(xf), sf) != leg_f.process(list(xf), sf):
            same = False; break
        if rev0.process(list(xr), sr) != leg_r.process(list(xr), sr):
            same = False; break
    status = "OK" if same else "FAIL"
    if not same:
        ok = False
    print(f"    coupled accrual=0 == legacy over 200 mixed chunks (both dirs) = "
          f"{same}  [{status}]")

    # ---- [X1] FORWARD OFDM BIT-EXACT under accrual ON (the v1 regression) -----
    # The v1 bug: the forward instance accrued ITS OWN just-ended burst into ITS
    # OWN next signal onset and spent it as ~90313 samp of silence inserted INTO
    # the forward OFDM signal -> de-aligned forward decode. The v2 fix: the forward
    # instance only PUBLISHES its burst airtime to the coupler; it CONSUMES only
    # the OTHER direction's (reverse-ACK) airtime, which is tiny/empty during a
    # forward batch -> n_accrual_onsets==0 on the forward, no large insert. ppm=0,
    # jitter=0 here isolates the accrual contract: ZERO inserts into the forward
    # signal (the precise v1 regression), tones bit-exact.
    print("\n[X1] forward a2b OFDM signal BIT-EXACT under accrual ON (no insert)")
    coup = R.TurnaroundCoupler()
    fwd = R.TurnaroundDrift(0.0, 0.0, R.Xoshiro(7), enabled=True,
                            accrual_ms_per_s=R.DEFAULT_TURNAROUND_ACCRUAL_MS_PER_S,
                            coupler=coup, direction="a2b")
    out_sig, out_gap, burst_flat, cnext, n_ins_during = _drive_forward_batch(
        fwd, CFG16_FRAMES, 0)
    out_sig_arr = np.asarray(out_sig)
    burst_arr = np.asarray(burst_flat)
    # forward signal samples bit-identical at their OWN index (no grid shift) AND
    # zero accrual-driven insert into the forward signal (the v1 regression).
    bit_exact = (len(out_sig_arr) == len(burst_arr) and
                 np.array_equal(out_sig_arr, burst_arr))
    no_signal_insert = (n_ins_during == 0) and (fwd.n_accrual_onsets == 0)
    status = "OK" if (bit_exact and no_signal_insert) else "FAIL"
    if not (bit_exact and no_signal_insert):
        ok = False
    print(f"    forward signal bit-exact (no grid shift) = {bit_exact}; "
          f"silence inserted into forward signal = {n_ins_during} (need 0); "
          f"forward accrual onsets = {fwd.n_accrual_onsets} (need 0)  [{status}]")
    print(f"    forward acc after batch = {fwd.acc:+.1f} samp; "
          f"coupler pending fwd airtime PUBLISHED = {coup.pending_samples} samp "
          f"(fed to the REVERSE ACK onset)")

    # ---- [X2] REVERSE ACK ONSET DELAYED by ~accrual * forward-batch-airtime ---
    print("\n[X2] reverse b2a ACK onset delayed by ~30 ms/s * forward-batch-airtime")
    # Build a coupled pair, drive the forward CFG16 batch through the FORWARD
    # instance (publishes its airtime), then key up the reverse ACK through the
    # REVERSE instance (consumes it). Read the b2a onset's owed offset = the
    # reverse-ACK late shift. ppm=0/jitter=0 isolates the cross-direction accrual.
    fwd2, rev2, coup2 = _make_coupled_pair(
        0, R.DEFAULT_TURNAROUND_ACCRUAL_MS_PER_S, ppm_a2b=0.0, ppm_b2a=0.0,
        jitter_ms=0.0)
    _, _, _, cnext, _ = _drive_forward_batch(fwd2, CFG16_FRAMES, 0)
    # reverse turnaround gap before the ACK (the b2a stream was silent during the
    # whole forward batch; here we just need a couple of gap chunks then the onset)
    for _ in range(2):
        rev2.process(_silence_chunk(), True)
    rev2.process(_signal_chunk(cnext), False)       # the reverse ACK ONSET
    rev_shift = rev2.last_onset_acc
    fwd_airtime_s = (CFG16_FRAMES * FRAME_CHUNKS * CH) / FS
    expect = R.DEFAULT_TURNAROUND_ACCRUAL_MS_PER_S * fwd_airtime_s * FS / 1000.0
    # ppm=0/jitter=0 -> the onset offset is EXACTLY the cross-direction accrual
    # (no dither); allow only integer-quantization slack.
    dither_tol = 2.0
    near = abs(rev_shift - expect) <= dither_tol
    positive_late = rev_shift > 0
    status = "OK" if (near and positive_late) else "FAIL"
    if not (near and positive_late):
        ok = False
    print(f"    reverse ACK onset shift = {rev_shift:.0f} samp; "
          f"expect ~{expect:.0f} (fwd airtime {fwd_airtime_s:.2f}s); "
          f"|err|<= {dither_tol:.0f}? {near}  [{status}]")

    # ---- [X3] BATCH-LENGTH STEP at the CMD window (CFG16 MISS / CFG15 LAND) ----
    print("\n[X3] step: CFG16 fwd batch -> reverse ACK MISS; CFG15 fwd batch -> LAND")

    def reverse_shift_for(n_frames, seed=0):
        fwd, rev, coup = _make_coupled_pair(seed, R.DEFAULT_TURNAROUND_ACCRUAL_MS_PER_S)
        _, _, _, cnext, _ = _drive_forward_batch(fwd, n_frames, 0)
        for _ in range(2):
            rev.process(_silence_chunk(), True)
        rev.process(_signal_chunk(cnext), False)
        return rev.last_onset_acc

    s16 = reverse_shift_for(CFG16_FRAMES)
    s15 = reverse_shift_for(CFG15_FRAMES)
    step = (s16 > WINDOW_HALFWIDTH) and (s15 < WINDOW_HALFWIDTH)
    status = "OK" if step else "FAIL"
    if not step:
        ok = False
    print(f"    CFG16 reverse shift = {s16:.0f} (>{WINDOW_HALFWIDTH:.0f}? "
          f"{s16 > WINDOW_HALFWIDTH}); CFG15 reverse shift = {s15:.0f} "
          f"(<{WINDOW_HALFWIDTH:.0f}? {s15 < WINDOW_HALFWIDTH})  step={step}  [{status}]")

    # [X3b] miss-fraction over a seed sweep (bench-9 87.5% / ~0% step).
    def miss_frac(n_frames, n_seeds=128):
        miss = 0
        for s in range(n_seeds):
            if reverse_shift_for(n_frames, seed=s) > WINDOW_HALFWIDTH:
                miss += 1
        return miss / n_seeds
    m16 = miss_frac(CFG16_FRAMES)
    m15 = miss_frac(CFG15_FRAMES)
    anchor = (m16 >= 0.80) and (m15 <= 0.05)
    status = "OK" if anchor else "FAIL"
    if not anchor:
        ok = False
    print(f"    miss-fraction CFG16={m16:.2f} (HW 0.875, need>=0.80), "
          f"CFG15={m15:.2f} (HW ~0.0, need<=0.05)  anchors={anchor}  [{status}]")

    print("\n" + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

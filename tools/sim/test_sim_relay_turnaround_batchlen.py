#!/usr/bin/env python3
"""
test_sim_relay_turnaround_batchlen.py — BATCH-LENGTH-DEPENDENT turnaround
window-miss calibration (bench-9 anchor).

WHY THIS TEST EXISTS (the model error it pins):
The existing TurnaroundDrift (M1-M3, test_sim_relay_turnaround.py) realizes the
turnaround offset as a CUMULATIVE ppm slip (M2) plus a SYMMETRIC, ZERO-MEAN,
per-key-up jitter (M3, uniform in [-J,+J]). That offset is a zero-mean random
walk: its expected magnitude after n key-ups grows only as ~sigma*sqrt(n). So a
CFG16 batch (25-30 forward frames, one reverse-ACK turnaround) accumulates only
~sqrt(3)=1.7x more |offset| than a short CFG15 batch — NOT enough to make the
CFG16 reverse-ACK MISS its window while the CFG15 ACK lands. The HW ground truth
(bench-9, ARMA_CALIBRATION.json) is a STEP, not a 1.7x:

    CFG16 held under +/-8 ppm + PTT/capture jitter -> matched=0/7 on 14/16
       reverse-ACK turnarounds = 87.5% FULL WINDOW MISS, active 18.9%,
       whole-window wire 597.6 bps.
    CFG15 short batches under the SAME drift -> reverse-ACK lands in window on
       essentially every batch (bench-8: sustained 3060 bps, NOT halved,
       d2_reverse_ack_fires=0, 100% high-rung).

The physical cause (SIMFIDELITY_ROOTCAUSE / bench-9): a LONGER forward batch
holds the half-duplex channel longer, so the keyer/AGC/capture-flush/scheduling
turnaround latency ACCUMULATES WITHIN THE BATCH and pushes the single
end-of-batch reverse-ACK turnaround SYSTEMATICALLY LATE (one-sided), in
proportion to how long the batch was. The miss probability is therefore
BATCH-LENGTH-DEPENDENT. A zero-mean symmetric walk cannot produce that.

THE NEW MODEL (--turnaround-batch-accrual, OPT-IN, env-mirrored): each forward
SIGNAL-burst run (the just-held batch) contributes a ONE-SIDED accumulating
turnaround offset proportional to its run length (samples held on the channel),
applied as inserted silence AHEAD OF the next (reverse) burst, so the reverse
ACK's arrival index walks LATE in proportion to the forward batch length. Long
CFG16 batch -> offset crosses the CMD window half-width -> ACK MISSES; short
CFG15 batch -> offset stays inside -> ACK LANDS. The existing ppm (M2) +
symmetric jitter (M3) ride on top (small dither); the accrual term is the
systematic batch-length axis.

CONTRACTS asserted here:
  [B0] DEFAULT-OFF byte identity: accrual disabled -> TurnaroundDrift is
       UNCHANGED (same insert/drop as the legacy model) so the existing
       --turnaround-drift default and its test are untouched.
  [B1] THE BUG (current model is NOT batch-length-dependent enough): with
       accrual OFF, the end-of-batch offset for a 30-frame batch is < 2x a
       6-frame batch -> cannot produce a STEP. (Demonstrates the failure the
       fix repairs; informational, never the gate.)
  [B2] BATCH-LENGTH STEP (the fix): with accrual ON and the bench-9-calibrated
       knobs, the accumulated late-offset for a CFG16-length batch exceeds the
       CMD window half-width (MISS) while a CFG15-length batch stays below it
       (LAND). The ratio is >> the sqrt(n) the symmetric walk gives.
  [B3] MONOTONE in batch length: longer held batch -> larger late-offset,
       strictly (one-sided accrual, never cancels).
  [B4] CALIBRATION ANCHORS: with the default calibrated knobs the per-batch
       reverse-ACK MISS fraction is ~>=0.80 for CFG16-length batches and
       ~<=0.05 for CFG15-length batches (the bench-9 87.5% / ~0% step), over a
       seed sweep.

Run:  python tools/sim/test_sim_relay_turnaround_batchlen.py
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
# CFG16 held batches are 25-30 frames; CFG15 short batches are ~6 frames. Each
# OFDM frame is one forward signal burst; the batch ends with ONE reverse-ACK
# turnaround. We model "n forward signal chunks held, then a reverse onset".
CFG16_FRAMES = 28      # mid of the 25-30 HW range
CFG15_FRAMES = 6       # short-batch CFG15
FRAME_CHUNKS = 8       # ~ forward signal chunks per OFDM frame on the wire (>=1)

# CMD reverse-ACK listen-window HALF-WIDTH in samples. The reverse ACK lands in
# the window if |accumulated late-offset| <= this, else it MISSES. Calibrated so
# a CFG16-length accrual exceeds it and a CFG15-length accrual does not (the
# bench-9 step). Expressed in ms for readability.
WINDOW_HALFWIDTH_MS = 90.0
WINDOW_HALFWIDTH = WINDOW_HALFWIDTH_MS * FS / 1000.0


def _silence_chunk():
    return [0.0] * CH


def _signal_chunk(c, amp=0.3, f=R.CENTER_HZ):
    t = (np.arange(CH) + c * CH) / FS
    return (amp * np.sin(2.0 * math.pi * f * t)).tolist()


def _batch_endoffset(td, n_frames, seed_chunkbase=0):
    """Drive ONE forward batch of n_frames*FRAME_CHUNKS signal chunks (the modem
    holding the channel), then a short turnaround gap, then the reverse ACK
    ONSET. Return the signed timing offset (samples) the model has accumulated AT
    the reverse-ACK onset = how far LATE/early the ACK arrival index is shifted.
    Positive = LATE (toward a window miss)."""
    c = seed_chunkbase
    # forward batch: a continuous run of signal chunks (one held burst).
    for _ in range(n_frames * FRAME_CHUNKS):
        td.process(_signal_chunk(c), False)
        c += 1
    # short turnaround gap (silence) — the half-duplex switch.
    for _ in range(2):
        td.process(_silence_chunk(), True)
        c += 1
    # the reverse-ACK ONSET: process a single signal chunk (the rising edge) and
    # read the model's owed offset AT the onset (jitter + batch-length accrual
    # applied, before it is spent on inserted silence) = the reverse-ACK shift.
    td.process(_signal_chunk(c), False)
    return td.last_onset_acc


def _miss_fraction(make_td, n_frames, n_seeds=64):
    """Fraction of batches whose reverse-ACK end-offset exceeds the window
    half-width (a MISS) over a seed sweep."""
    miss = 0
    for s in range(n_seeds):
        td = make_td(s)
        off = _batch_endoffset(td, n_frames)
        if abs(off) > WINDOW_HALFWIDTH:
            miss += 1
    return miss / n_seeds


def main():
    ok = True
    print("=== BATCH-LENGTH-DEPENDENT turnaround window-miss (bench-9 anchor) ===\n")
    print(f"  CFG16 batch = {CFG16_FRAMES} frames, CFG15 batch = {CFG15_FRAMES} "
          f"frames, {FRAME_CHUNKS} chunks/frame")
    print(f"  CMD window half-width = {WINDOW_HALFWIDTH_MS} ms "
          f"({WINDOW_HALFWIDTH:.0f} samples)\n")

    has_accrual = hasattr(R.TurnaroundDrift(8.16, 6.0, R.Xoshiro(1)), "accrual_ms_per_s")

    # ---- [B0] accrual-OFF byte identity with the legacy model ---------------
    # An accrual-OFF TurnaroundDrift must produce the SAME wire as the legacy
    # (pre-fix) model for an identical input + seed: the default path is unchanged.
    print("[B0] accrual-OFF -> legacy TurnaroundDrift behaviour unchanged")
    if has_accrual:
        td_off = R.TurnaroundDrift(8.16, 6.0, R.Xoshiro(5),
                                   enabled=True, accrual_ms_per_s=0.0)
        td_leg = R.TurnaroundDrift(8.16, 6.0, R.Xoshiro(5), enabled=True)
        same = True
        rng = np.random.default_rng(99)
        for c in range(200):
            x = (_silence_chunk() if c % 4 == 0
                 else (rng.standard_normal(CH) * 0.3).tolist())
            sil = all(abs(v) <= 1e-12 for v in x)
            o1 = td_off.process(list(x), sil)
            o2 = td_leg.process(list(x), sil)
            if o1 != o2:
                same = False
                break
        status = "OK" if same else "FAIL"
        if not same:
            ok = False
        print(f"    accrual=0 matches legacy over 200 mixed chunks = {same}  [{status}]")
    else:
        print("    SKIP (accrual knob not present yet) [pending fix]")

    # ---- [B1] THE BUG: current (symmetric) model is NOT a batch-length step --
    print("\n[B1] current symmetric-jitter model: NOT batch-length-dependent (the bug)")

    def make_legacy(seed):
        # +/-8.16 ppm + 6 ms symmetric jitter, accrual OFF (current default).
        if has_accrual:
            return R.TurnaroundDrift(8.16, 6.0, R.Xoshiro(seed + 1),
                                     enabled=True, accrual_ms_per_s=0.0)
        return R.TurnaroundDrift(8.16, 6.0, R.Xoshiro(seed + 1), enabled=True)

    # mean |end-offset| over seeds for long vs short batch under the legacy model.
    def mean_abs_off(make_td, n_frames, n_seeds=64):
        vals = [abs(_batch_endoffset(make_td(s), n_frames)) for s in range(n_seeds)]
        return float(np.mean(vals))

    off16_leg = mean_abs_off(make_legacy, CFG16_FRAMES)
    off15_leg = mean_abs_off(make_legacy, CFG15_FRAMES)
    ratio_leg = off16_leg / max(off15_leg, 1e-9)
    # the legacy model's long/short ratio is small (ppm-dominated ~ linear but
    # tiny, jitter ~ sqrt(n)); it is NOWHERE near a window-crossing STEP. We just
    # SHOW it (informational). The defining bug: the legacy CFG16 offset does NOT
    # reliably exceed the window while CFG15 stays under (no step).
    leg_miss16 = _miss_fraction(make_legacy, CFG16_FRAMES)
    print(f"    legacy mean|off| CFG16={off16_leg:.0f}  CFG15={off15_leg:.0f}  "
          f"ratio={ratio_leg:.2f}x  CFG16-miss-frac={leg_miss16:.2f}")
    print(f"    (HW needs CFG16 miss ~0.875 / CFG15 ~0.0 — legacy cannot deliver "
          f"this STEP; it is ~sqrt(n) bounded)")

    # ---- [B2]/[B3]/[B4] THE FIX: batch-length accrual produces the step ------
    print("\n[B2/B3/B4] batch-length accrual -> CFG16 MISS / CFG15 LAND step")
    if not has_accrual:
        print("    FAIL: TurnaroundDrift has no batch-length accrual knob "
              "(accrual_ms_per_s) — fix not implemented yet.")
        print("\nFAIL")
        return 1

    def make_accrual(seed):
        # default-calibrated accrual knob (the relay's default for the new mode);
        # ppm + symmetric jitter ride on top as small dither.
        return R.TurnaroundDrift(8.16, 6.0, R.Xoshiro(seed + 1), enabled=True,
                                 accrual_ms_per_s=R.DEFAULT_TURNAROUND_ACCRUAL_MS_PER_S)

    # [B3] MONOTONE: longer held batch -> strictly larger late-offset.
    offs = []
    for nf in (CFG15_FRAMES, 12, 20, CFG16_FRAMES):
        o = _batch_endoffset(make_accrual(0), nf)
        offs.append(o)
    monotone = all(offs[i] < offs[i + 1] for i in range(len(offs) - 1))
    status = "OK" if monotone else "FAIL"
    if not monotone:
        ok = False
    print(f"    [B3] late-offset vs frames {[6,12,20,CFG16_FRAMES]} = "
          f"{[round(v) for v in offs]}  strictly-increasing={monotone}  [{status}]")

    # [B2] STEP: CFG16 end-offset > window, CFG15 end-offset < window (one seed
    # at the calibrated knob — the systematic accrual dominates the dither).
    off16 = _batch_endoffset(make_accrual(0), CFG16_FRAMES)
    off15 = _batch_endoffset(make_accrual(0), CFG15_FRAMES)
    step = (off16 > WINDOW_HALFWIDTH) and (off15 < WINDOW_HALFWIDTH)
    status = "OK" if step else "FAIL"
    if not step:
        ok = False
    print(f"    [B2] CFG16 off={off16:.0f} (>{WINDOW_HALFWIDTH:.0f}? "
          f"{off16 > WINDOW_HALFWIDTH}), CFG15 off={off15:.0f} "
          f"(<{WINDOW_HALFWIDTH:.0f}? {off15 < WINDOW_HALFWIDTH})  step={step}  [{status}]")

    # [B4] CALIBRATION ANCHORS: miss-fraction over a seed sweep matches bench-9
    # (CFG16 ~>=0.80 miss; CFG15 ~<=0.05 miss).
    miss16 = _miss_fraction(make_accrual, CFG16_FRAMES, n_seeds=128)
    miss15 = _miss_fraction(make_accrual, CFG15_FRAMES, n_seeds=128)
    anchor_ok = (miss16 >= 0.80) and (miss15 <= 0.05)
    status = "OK" if anchor_ok else "FAIL"
    if not anchor_ok:
        ok = False
    print(f"    [B4] miss-fraction CFG16={miss16:.2f} (HW 0.875, need >=0.80), "
          f"CFG15={miss15:.2f} (HW ~0.0, need <=0.05)  anchors={anchor_ok}  [{status}]")

    print("\n" + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

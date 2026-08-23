#!/usr/bin/env python3
"""
realaudio_bridge_s32.py - bit-faithful real-audio IONOS bridge over snd-aloop.

This is the production copy of the prototype proven in wf_1e5c12f1 (a real modem
round-trip CONNECTS + DELIVERS over it, and it is LOAD-IMMUNE: pinned ROBUST_0
idle vs stress-ng --cpu 24 gave BIT-IDENTICAL delivered frames+bytes because the
audio clock is the snd-aloop HW timer, not the CPU).

It matches Mercury's ACTUAL `-x alsa` wire format EXACTLY so the ONLY thing
between the two modems is Channel.process():

  Mercury -x alsa negotiates  INT32 / 48000 Hz / 2ch  (audioio.c:564,981
  conf.buf.format = FFAUDIO_F_INT32; confirmed in run logs:
  "format=32 (INT32) / 48000Hz / 2ch"). TX scales the internal double passband by
  INT_MAX (audioio.c:788/806 `clamped * INT_MAX`); RX scales back by /INT_MAX
  (audioio.c:1210/1220/1231 `buffer[i] / INT_MAX`). Both stereo channels carry
  the SAME mono sample (audioio.c:807-808).

So this bridge opens RAW hw:Loopback (NO `plug` - no resample/requantize) at
S32_LE/2ch/48k, takes channel 0, divides by INT_MAX to recover the exact float64
passband Mercury's RX would see, runs Channel.process() VERBATIM (lifted from
sim_channel_relay.py - the same DSP the TCP relay uses), re-scales by INT_MAX,
clamps, duplicates to both channels, and writes. The float64 units are now
identical to the TCP float64 wire (sim_channel_relay.py header lines 18-19), so
the channel math sees exactly what it sees in the -x sim path.

Latency: small playback ring (PLAY_PERIODS) primed with PRIME_PERIODS of silence,
to keep added round-trip latency low (turnaround-sensitive ARQ). The CAPTURE read
is kernel-clocked by the snd-aloop hardware timer => load-immune.

snd-aloop cabling: playback (dev 0, sub N) is internally wired to capture
(dev 1, sub N). So one run uses 4 cables (subs):
  FORWARD  cap hw:<card>,1,<fs>  -> impair -> play hw:<card>,0,<fp>   (CMD->RSP)
  REVERSE  cap hw:<card>,1,<rs>  -> impair -> play hw:<card>,0,<rp>   (RSP->CMD)
Mercury commander/responder sit on the OTHER ends of those 4 sub-cables.

--passthrough disables Channel.process entirely (perfect bit-exact cable) for
the P0 round-trip reference.

Import note: Channel/PROFILES/ionos_wgn_to_snr3k live in the parent tools/sim package
(sim_channel_relay.py). We add the parent dir to sys.path so this file can live
in tools/sim/realaudio/ while reusing the canonical channel DSP verbatim.
"""
import argparse
import os
import sys
import threading
import time
import types

import numpy as np
import alsaaudio

# Channel DSP lives in the parent tools/sim/ package. Add it to sys.path so the
# bridge can be run from anywhere and still import the canonical relay verbatim.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT_SIM = os.path.dirname(_HERE)               # tools/sim
sys.path.insert(0, _PARENT_SIM)
sys.path.insert(0, _HERE)
from sim_channel_relay import Channel, PROFILES, ionos_wgn_to_snr3k  # noqa: E402
from sim_axis import (AXIS_CHOICES, AXIS_V2, DEFAULT_AXIS,
                      AxisError, headroom_plan, infer_bandwidth, resolve_axis,
                      sha256_json)  # noqa: E402

PERIOD = 1024            # frames per ALSA period (~21.3 ms @48k); == CHUNK_SAMPLES
RATE = 48000
INT_MAX = 2147483647.0   # matches audioio.c INT_MAX scaling (both directions)


def chan_args(a):
    return types.SimpleNamespace(
        axis=a.axis, snr=a.snr3k_db, snr3k_db=a.snr3k_db,
        cn_config_db=a.cn_config_db,
        configured_bandwidth_hz=a.configured_bandwidth_hz,
        input_coordinate=a.input_coordinate,
        reference_power=a.reference_power,
        reference_id=a.reference_id,
        reference_n_samples=a.reference_n_samples,
        binary_sha256=a.binary_sha256, recipe_sha256=a.recipe_sha256,
        loss=a.loss, burst=a.burst, profile=a.profile,
        cfo_hz=a.cfo_hz, phase_noise_deg=a.phase_noise_deg,
        sig_ref=a.sig_ref, fade_depth_db=a.fade_depth_db)


def open_cap(dev, periods):
    return alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NORMAL,
                         rate=RATE, channels=2,
                         format=alsaaudio.PCM_FORMAT_S32_LE,
                         periodsize=PERIOD, periods=periods, device=dev)


def open_play(dev, periods):
    return alsaaudio.PCM(alsaaudio.PCM_PLAYBACK, alsaaudio.PCM_NORMAL,
                         rate=RATE, channels=2,
                         format=alsaaudio.PCM_FORMAT_S32_LE,
                         periodsize=PERIOD, periods=periods, device=dev)


def pump(name, cap_dev, play_dev, ch, stop, stats, passthrough,
         cap_periods, play_periods, prime_periods, composite_scale,
         s32_mode, headroom):
    cap = open_cap(cap_dev, cap_periods)
    play = open_play(play_dev, play_periods)
    silence = np.zeros(PERIOD * 2, dtype="<i4").tobytes()
    for _ in range(prime_periods):
        play.write(silence)
    underruns = 0
    nframes = 0
    sig_frames = 0          # frames carrying non-zero input (signal energy)
    pre_scale_peak = 0.0
    hard_clip_count = 0
    s32_saturation_count = 0
    clip_denominator = 0
    while not stop.is_set():
        length, data = cap.read()          # BLOCKING, kernel-clocked
        if length <= 0:
            if length < 0:
                underruns += 1
            continue
        st = np.frombuffer(data, dtype="<i4")
        # de-interleave stereo -> channel 0 (both channels identical per audioio.c)
        if st.size >= length * 2:
            ch0 = st[0:length * 2:2]
        else:
            ch0 = st[:length]
        if passthrough:
            oi32 = ch0.astype("<i4")
        else:
            xf = ch0.astype(np.float64) / INT_MAX
            if np.any(ch0):
                sig_frames += length
            out = ch.process(xf.tolist())
            o = np.asarray(out, dtype=np.float64)
            if o.size:
                pre_scale_peak = max(pre_scale_peak,
                                     float(np.max(np.abs(o))))
            o *= composite_scale
            hard_clip_count += int(np.count_nonzero(np.abs(o) > 1.0))
            clip_denominator += int(o.size)
            o *= INT_MAX
            s32_saturation_count += int(
                np.count_nonzero(np.abs(o) > INT_MAX))
            np.clip(o, -INT_MAX, INT_MAX, out=o)
            oi32 = o.astype("<i4")
        # re-interleave to stereo (duplicate ch0 to both, like audioio.c:807-808)
        stereo = np.empty(oi32.size * 2, dtype="<i4")
        stereo[0::2] = oi32
        stereo[1::2] = oi32
        buf = stereo.tobytes()
        try:
            rc = play.write(buf)
            if rc is not None and rc < 0:
                underruns += 1
        except alsaaudio.ALSAAudioError:
            # playback underrun (XRUN): the ring drained before this write.
            # Recover: re-prime with one period of silence then re-write, so a
            # transient GIL/GC stall does not kill the bridge or wedge the cable.
            underruns += 1
            try:
                play.write(np.zeros(PERIOD * 2, dtype="<i4").tobytes())
                play.write(buf)
            except alsaaudio.ALSAAudioError:
                pass
        nframes += length
    applied = ch.attestation()
    applied.update({
        "composite_scale": composite_scale,
        "pre_scale_peak": pre_scale_peak,
        "hard_clip_count": hard_clip_count,
        "s32_saturation_count": s32_saturation_count,
        "clip_event_denominator": clip_denominator,
        "s32_mode": s32_mode,
        "headroom_k": headroom.get("headroom_k"),
        "headroom_n_samples": headroom.get("headroom_n_samples"),
        "headroom_epsilon": headroom.get("headroom_epsilon"),
    })
    stats[name] = {"frames": nframes, "sig_frames": sig_frames,
                   "underruns": underruns, "axis_attestation": applied}
    try:
        cap.close()
        play.close()
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fwd-cap", default="hw:Loopback,1,0")
    ap.add_argument("--fwd-play", default="hw:Loopback,0,1")
    ap.add_argument("--rev-cap", default="hw:Loopback,1,2")
    ap.add_argument("--rev-play", default="hw:Loopback,0,3")
    ap.add_argument("--passthrough", action="store_true",
                    help="perfect bit-exact cable (no Channel.process)")
    ap.add_argument("--axis", choices=AXIS_CHOICES, default=DEFAULT_AXIS)
    ap.add_argument("--snr", type=float, default=None,
                    help="historical v0-only dial")
    ap.add_argument("--snr3k-db", type=float, default=None)
    ap.add_argument("--snr3k", type=float, default=None,
                    help="DEPRECATED controlling alias for --snr3k-db")
    ap.add_argument("--cn-config-db", type=float, default=None)
    ap.add_argument("--configured-bandwidth-hz", type=float, default=None)
    ap.add_argument("--band-family", choices=("WB", "NB"), default=None)
    ap.add_argument("--reference-power", type=float, default=None)
    ap.add_argument("--reference-id", default=None)
    ap.add_argument("--reference-n-samples", type=int, default=None)
    ap.add_argument("--calibration-peak", type=float, default=None)
    ap.add_argument("--headroom-n-samples", type=int, default=None)
    ap.add_argument("--headroom-epsilon", type=float, default=None)
    ap.add_argument("--s32-mode", choices=("auto", "prescaled", "s32-hardclip"),
                    default="auto")
    ap.add_argument("--binary-sha256", default=None)
    ap.add_argument("--recipe-sha256", default=None)
    ap.add_argument("--cell", default=None)
    ap.add_argument("--profile", choices=list(PROFILES.keys()), default="wgn")
    ap.add_argument("--cfo-hz", type=float, default=0.0)
    ap.add_argument("--phase-noise-deg", type=float, default=0.0)
    ap.add_argument("--fade-depth-db", type=float, default=0.0)
    ap.add_argument("--loss", type=float, default=0.0)
    ap.add_argument("--burst", action="store_true")
    ap.add_argument("--sig-ref", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--cap-periods", type=int, default=3)
    ap.add_argument("--play-periods", type=int, default=4)
    ap.add_argument("--prime-periods", type=int, default=2)
    ap.add_argument("--statsfile", default=None)
    a = ap.parse_args()
    bandwidth = (a.configured_bandwidth_hz
                 if a.configured_bandwidth_hz is not None
                 else infer_bandwidth(None, a.band_family))
    try:
        resolved = resolve_axis(
            a.axis, snr=a.snr, snr3k=a.snr3k, snr3k_db=a.snr3k_db,
            cn_config_db=a.cn_config_db,
            cell_snr3k_db=(ionos_wgn_to_snr3k(a.cell.split(":", 1)[1])
                           if a.cell else None),
            configured_bandwidth_hz=bandwidth, default_snr3k_db=30.0)
    except (AxisError, ValueError, IndexError) as exc:
        ap.error(str(exc))
    a.snr3k_db = resolved["snr3k_db"]
    a.cn_config_db = resolved["cn_config_db"]
    a.configured_bandwidth_hz = resolved["configured_bandwidth_hz"]
    a.input_coordinate = resolved["input_coordinate"]
    if a.axis == AXIS_V2:
        if (a.reference_power is None or not a.reference_id
                or not a.reference_n_samples):
            ap.error("v2 requires reference power/id/sample denominator")
        if a.calibration_peak is None:
            ap.error("v2 requires --calibration-peak")
        if a.s32_mode == "auto":
            a.s32_mode = "prescaled"
    else:
        if a.s32_mode == "prescaled":
            ap.error("prescaled S32 is canonical only for v2")
        if a.s32_mode == "auto":
            a.s32_mode = "s32-hardclip"
    if a.s32_mode == "prescaled":
        try:
            headroom = headroom_plan(
                a.calibration_peak,
                a.reference_power * 24000.0 /
                (10.0 ** (a.snr3k_db / 10.0) * 3000.0),
                a.headroom_n_samples, a.headroom_epsilon)
        except AxisError as exc:
            ap.error(str(exc))
    else:
        headroom = {"composite_scale": 1.0, "headroom_k": None,
                    "headroom_n_samples": a.headroom_n_samples,
                    "headroom_epsilon": a.headroom_epsilon}
    composite_scale = headroom["composite_scale"]
    if a.binary_sha256 is None:
        a.binary_sha256 = "unattested"
    if a.recipe_sha256 is None:
        a.recipe_sha256 = sha256_json({
            "axis": a.axis, "snr3k_db": a.snr3k_db,
            "cn_config_db": a.cn_config_db,
            "configured_bandwidth_hz": a.configured_bandwidth_hz,
            "reference_id": a.reference_id, "reference_power": a.reference_power,
            "s32_mode": a.s32_mode, "composite_scale": composite_scale,
        })

    cargs = chan_args(a)
    ch_fwd = Channel(cargs, (a.seed * 2654435761) & 0xFFFFFFFF)
    ch_rev = Channel(cargs, (a.seed * 40503 + 7) & 0xFFFFFFFF)
    mode = ("PASSTHROUGH" if a.passthrough else
            f"axis={a.axis} SNR3k={a.snr3k_db:.2f} "
            f"CNcfg={a.cn_config_db:.2f} profile={a.profile} "
            f"s32={a.s32_mode} scale={composite_scale:.12g}")
    sys.stderr.write(f"[bridge_s32] {mode} cfo={a.cfo_hz} pn={a.phase_noise_deg} "
                     f"seed={a.seed} rings cap={a.cap_periods} play={a.play_periods} "
                     f"prime={a.prime_periods} cables "
                     f"fwd[{a.fwd_cap}->{a.fwd_play}] rev[{a.rev_cap}->{a.rev_play}]\n")
    sys.stderr.flush()

    stop = threading.Event()
    stats = {}

    # On SIGTERM (the harness shuts the bridge down with terminate()), set stop
    # so the pump threads exit their loop and record their stats, instead of
    # being killed mid-loop and leaving the statsfile empty (which blinds us to
    # the underrun count - the single most important health signal).
    import signal as _signal

    def _on_term(signum, frame):  # noqa: ARG001
        stop.set()
    _signal.signal(_signal.SIGTERM, _on_term)

    tf = threading.Thread(target=pump, args=(
        "fwd", a.fwd_cap, a.fwd_play, ch_fwd, stop, stats, a.passthrough,
        a.cap_periods, a.play_periods, a.prime_periods, composite_scale,
        a.s32_mode, headroom))
    tr = threading.Thread(target=pump, args=(
        "rev", a.rev_cap, a.rev_play, ch_rev, stop, stats, a.passthrough,
        a.cap_periods, a.play_periods, a.prime_periods, composite_scale,
        a.s32_mode, headroom))
    tf.start()
    tr.start()
    try:
        while tf.is_alive() and tr.is_alive() and not stop.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    stop.set()
    tf.join(timeout=3)
    tr.join(timeout=3)
    if "fwd" in stats and "rev" in stats:
        fwd_att = stats["fwd"]["axis_attestation"]
        rev_att = stats["rev"]["axis_attestation"]
        aggregate = dict(fwd_att)
        aggregate["pre_scale_peak"] = max(
            fwd_att["pre_scale_peak"], rev_att["pre_scale_peak"])
        for key in ("hard_clip_count", "s32_saturation_count",
                    "clip_event_denominator", "noise_n_samples"):
            aggregate[key] = fwd_att.get(key, 0) + rev_att.get(key, 0)
        aggregate["seed"] = a.seed
        aggregate["direction_attestations"] = {
            "fwd": fwd_att, "rev": rev_att,
        }
        budget = aggregate.get("headroom_n_samples")
        aggregate["headroom_budget_exceeded"] = bool(
            budget is not None
            and aggregate["clip_event_denominator"] > budget)
        stats["axis_attestation"] = aggregate
    sys.stderr.write(f"[bridge_s32] stats={stats}\n")
    if a.statsfile:
        import json
        with open(a.statsfile, "w") as f:
            json.dump(stats, f)


if __name__ == "__main__":
    main()

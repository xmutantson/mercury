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

PERIOD = 1024            # frames per ALSA period (~21.3 ms @48k); == CHUNK_SAMPLES
RATE = 48000
INT_MAX = 2147483647.0   # matches audioio.c INT_MAX scaling (both directions)


def chan_args(a):
    return types.SimpleNamespace(
        snr=a.snr, loss=a.loss, burst=a.burst, profile=a.profile,
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
         cap_periods, play_periods, prime_periods):
    cap = open_cap(cap_dev, cap_periods)
    play = open_play(play_dev, play_periods)
    silence = np.zeros(PERIOD * 2, dtype="<i4").tobytes()
    for _ in range(prime_periods):
        play.write(silence)
    underruns = 0
    nframes = 0
    sig_frames = 0          # frames carrying non-zero input (signal energy)
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
            o *= INT_MAX
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
    stats[name] = {"frames": nframes, "sig_frames": sig_frames,
                   "underruns": underruns}
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
    ap.add_argument("--snr", type=float, default=30.0)
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
    if a.cell:
        a.snr = ionos_wgn_to_snr3k(a.cell.split(":", 1)[1])

    cargs = chan_args(a)
    ch_fwd = Channel(cargs, (a.seed * 2654435761) & 0xFFFFFFFF)
    ch_rev = Channel(cargs, (a.seed * 40503 + 7) & 0xFFFFFFFF)
    mode = "PASSTHROUGH" if a.passthrough else f"SNR3k={a.snr:.2f} profile={a.profile}"
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
        a.cap_periods, a.play_periods, a.prime_periods))
    tr = threading.Thread(target=pump, args=(
        "rev", a.rev_cap, a.rev_play, ch_rev, stop, stats, a.passthrough,
        a.cap_periods, a.play_periods, a.prime_periods))
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
    sys.stderr.write(f"[bridge_s32] stats={stats}\n")
    if a.statsfile:
        import json
        with open(a.statsfile, "w") as f:
            json.dump(stats, f)


if __name__ == "__main__":
    main()

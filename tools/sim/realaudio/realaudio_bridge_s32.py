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

Channel attestation ([C1]): at teardown the bridge writes a `channel_attestation`
object into the statsfile (alongside the per-direction frames/underruns) that
records what channel the session ACTUALLY ran -- cell / profile / commanded_snr /
realized_snr3k / realized_snr_offset_db / seed / passthrough / realized_p_sig (mean-square input
signal power on the forward/payload direction). This makes a --passthrough or
wrong-profile bridge VISIBLE to the honest-baseline runner gate (a default-deny
evidence manifest, HONEST_BASELINE_RUNNER_DESIGN.md C1/C3) instead of silently
scoring a bit-exact cable as a real channel. Run `--self-test` (or `--dry-run`)
for a no-ALSA check that the statsfile carries a well-formed attestation; both
work on hosts without alsaaudio/snd-aloop (the power/attestation math is exercised
on a synthetic sig_ref-amplitude tone through the SAME Channel.process path).

Import note: Channel/PROFILES/ionos_wgn_to_snr3k live in the parent tools/sim package
(sim_channel_relay.py). We add the parent dir to sys.path so this file can live
in tools/sim/realaudio/ while reusing the canonical channel DSP verbatim.
"""
import argparse
import atexit
import json
import math
import os
import signal
import sys
import threading
import time
import types

import numpy as np
try:
    import alsaaudio
except ImportError:
    # Not installed on non-Linux hosts (e.g. the Windows dev box). The live
    # snd-aloop path needs it, but --self-test / --dry-run must run anywhere, so
    # the import is optional and only the ALSA path asserts its presence.
    alsaaudio = None

# Channel DSP lives in the parent tools/sim/ package. Add it to sys.path so the
# bridge can be run from anywhere and still import the canonical relay verbatim.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT_SIM = os.path.dirname(_HERE)               # tools/sim
sys.path.insert(0, _PARENT_SIM)
sys.path.insert(0, _HERE)
from sim_channel_relay import (  # noqa: E402
    Channel, PROFILES, ionos_wgn_to_snr3k, BANDPASS_PRESETS,
    BANDPASS_DEFAULT_TAPS, resolve_bandpass)

PERIOD = 1024            # frames per ALSA period (~21.3 ms @48k); == CHUNK_SAMPLES
RATE = 48000
INT_MAX = 2147483647.0   # matches audioio.c INT_MAX scaling (both directions)


def chan_args(a):
    # Resolve the audio-bandpass spec (narrow-FM radio audio filter) into the
    # numeric edges Channel reads. Off (default) -> no filter -> byte-identical
    # to the pre-bandpass bridge (Mercury real-audio path unaffected).
    bp_mode = getattr(a, "audio_bandpass", "off")
    bp_lo, bp_hi, bp_taps = resolve_bandpass(
        bp_mode, getattr(a, "bandpass_lo_hz", 0.0),
        getattr(a, "bandpass_hi_hz", 0.0), getattr(a, "bandpass_taps", None))
    return types.SimpleNamespace(
        snr=a.snr, loss=a.loss, burst=a.burst, profile=a.profile,
        cfo_hz=a.cfo_hz, phase_noise_deg=a.phase_noise_deg,
        sig_ref=a.sig_ref, fade_depth_db=a.fade_depth_db,
        bandpass_lo_hz=bp_lo, bandpass_hi_hz=bp_hi, bandpass_taps=bp_taps)


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
    sig_frames = 0          # input samples in signal-bearing periods
    sig_sumsq = 0.0         # sum of x^2 over those samples (for realized_p_sig)
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
        # realized_p_sig attestation: measure the mean-square of the INPUT
        # passband (float64, same units Channel.process sees == the P_sig that,
        # offset by realized_snr_offset_db, sets the realized SNR) over
        # signal-bearing periods. Done in BOTH modes so the attestation is always
        # populated; the passthrough OUTPUT still uses ch0 verbatim (bit-exact).
        xf = ch0.astype(np.float64) / INT_MAX
        if np.any(ch0):
            sig_frames += length
            sig_sumsq += float(np.dot(xf, xf))
        if passthrough:
            oi32 = ch0.astype("<i4")
        else:
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
        # Incremental publish (NIT A): keep stats[name] CURRENT every iteration so a
        # mid-run durable flush (SIGTERM / atexit / 0.5 s cadence) captures a real
        # per-direction snapshot -> the persisted [C1] channel_attestation carries a
        # genuine realized_p_sig even if the bridge is torn down before its final write.
        stats[name] = {"frames": nframes, "sig_frames": sig_frames,
                       "sig_sumsq": sig_sumsq, "underruns": underruns}
    stats[name] = {"frames": nframes, "sig_frames": sig_frames,
                   "sig_sumsq": sig_sumsq, "underruns": underruns}
    try:
        cap.close()
        play.close()
    except Exception:
        pass


# Fields of the [C1] channel_attestation and their required JSON types. bool is
# a subclass of int, so the validator special-cases int/float to exclude bool.
_ATT_SPEC = (
    ("cell", str), ("profile", str), ("commanded_snr", float),
    ("realized_snr3k", float), ("realized_snr_offset_db", float), ("seed", int),
    ("passthrough", bool), ("realized_p_sig", float),
)


def build_channel_attestation(a, stats, commanded_snr, snr_offset, cell_label):
    """[C1] channel-attestation: record what channel the session ACTUALLY ran so
    a --passthrough or wrong-profile bridge is no longer invisible to the runner
    gate (HONEST_BASELINE_RUNNER_DESIGN.md C3, a default-deny evidence manifest).

    realized_p_sig is the FORWARD (payload) direction's mean-square input power
    over signal-bearing periods -- the P_sig that, offset by
    realized_snr_offset_db, defines the realized SNR the modem experienced. The
    forward direction carries the CMD->RSP payload whose SNR is compared to VARA.
    """
    fwd = stats.get("fwd", {})
    nsamp = fwd.get("sig_frames", 0)
    ssq = fwd.get("sig_sumsq", 0.0)
    p_sig = (ssq / nsamp) if nsamp > 0 else 0.0
    return {
        "cell": str(cell_label),
        "profile": a.profile.upper(),
        # Preserve fractional dial coordinates: rounding would attest a
        # different channel from the one the bridge actually configured.
        "commanded_snr": float(commanded_snr),
        "realized_snr3k": float(commanded_snr + snr_offset),
        "realized_snr_offset_db": float(snr_offset),
        "seed": int(a.seed),
        "passthrough": bool(a.passthrough),
        "realized_p_sig": float(p_sig),
    }


def validate_channel_attestation(att):
    """Assert a channel_attestation is well-formed (all [C1] fields present and
    correctly typed). Raises AssertionError with a specific message on the first
    violation. Used by --self-test and available to the runner gate."""
    assert isinstance(att, dict), "channel_attestation is missing / not an object"
    for key, typ in _ATT_SPEC:
        assert key in att, f"channel_attestation missing field '{key}'"
        val = att[key]
        if typ is bool:
            assert isinstance(val, bool), \
                f"field '{key}' must be bool, got {type(val).__name__}"
        elif typ is int:
            assert isinstance(val, int) and not isinstance(val, bool), \
                f"field '{key}' must be int, got {type(val).__name__}"
        elif typ is float:
            assert isinstance(val, (int, float)) and not isinstance(val, bool), \
                f"field '{key}' must be a number, got {type(val).__name__}"
        else:
            assert isinstance(val, typ), \
                f"field '{key}' must be {typ.__name__}, got {type(val).__name__}"
    assert att["profile"] in ("WGN", "MPG", "MPM", "MPP"), \
        f"profile '{att['profile']}' is not an uppercase channel label"
    assert att["realized_p_sig"] >= 0.0, "realized_p_sig must be >= 0"
    return True


# ── Durable statsfile flush (NIT A) ───────────────────────────────────────────
# The bridge used to write its statsfile ONLY at main() end -- AFTER the pump
# threads join (up to ~6 s) -- so an older runner's event-driven teardown killed the
# bridge BEFORE that final write, and the [C1] channel_attestation was LOST -> the
# gate saw ATTEST_MISSING and returned HARNESS_INVALID on an otherwise-good
# session (C1 lost).  Persist the statsfile INCREMENTALLY (0.5 s cadence, from the
# main monitor loop) and on SIGTERM/atexit, via an atomic temp + os.replace, so the
# file at --statsfile is ALWAYS a COMPLETE prior snapshot during graceful teardown.
# Mirrors kiss_data_pump.py's durable manifest (FIX-C, OUTER commit 2c93f66).
_STATS_SINK = {"path": None, "stats": None, "args": None,
               "commanded_snr": None, "snr_offset": None, "cell_label": None}
_stats_flush_lock = threading.Lock()


def _atomic_write_json(path, obj):
    """Write obj to path atomically (temp + os.replace); a reader never sees a
    partial file.  On a hard kill mid-write the temp is orphaned but `path` keeps
    the last complete snapshot."""
    tmp = f"{path}.tmp.{os.getpid()}"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=2)
        f.flush()
        try:
            os.fsync(f.fileno())
        except OSError:
            pass
    os.replace(tmp, path)


def _register_stats_sink(path, stats, args, commanded_snr, snr_offset, cell_label):
    """Bind the live stats dict + attestation params for incremental / SIGTERM /
    atexit flush.  `stats` is mutated in place by the pump threads, so the flusher
    always sees the CURRENT per-direction counters (the pump publishes them into
    stats[name] every iteration) -> a mid-run snapshot carries a real
    realized_p_sig, not a 0 that would trip ATTEST_P_SIG_INSANE."""
    _STATS_SINK.update(path=path, stats=stats, args=args, commanded_snr=commanded_snr,
                       snr_offset=snr_offset, cell_label=cell_label)


def flush_stats():
    """Persist the current stats + a freshly-built [C1] channel_attestation to the
    statsfile (best-effort, atomic).  Safe from a signal handler / atexit and
    re-entrant (a re-entrant call during an in-flight flush is dropped; the
    in-flight write completes)."""
    path = _STATS_SINK["path"]
    stats = _STATS_SINK["stats"]
    if not path or stats is None:
        return
    if not _stats_flush_lock.acquire(blocking=False):
        return
    try:
        snap = dict(stats)          # shallow copy; per-direction fwd/rev dicts referenced whole
        snap["channel_attestation"] = build_channel_attestation(
            _STATS_SINK["args"], snap, _STATS_SINK["commanded_snr"],
            _STATS_SINK["snr_offset"], _STATS_SINK["cell_label"])
        _atomic_write_json(path, snap)
    except (OSError, ValueError, TypeError):
        pass
    finally:
        _stats_flush_lock.release()


def _dry_pump(ch, a, n_periods=8):
    """In-memory pump for --dry-run / --self-test (NO ALSA, no snd-aloop). Feeds a
    synthetic sig_ref-amplitude tone through the SAME Channel.process + power
    measurement the live pump uses, so the emitted realized_p_sig is a genuinely
    measured value (~sig_ref**2) rather than a fabricated constant. Returns a
    per-direction stats dict shaped exactly like the live pump's."""
    t = np.arange(PERIOD, dtype=np.float64) / RATE
    tone = np.sin(2.0 * np.pi * 1500.0 * t)          # modem center freq (Hz)
    amp = max(a.sig_ref, 1e-9) * math.sqrt(2.0)      # RMS sig_ref -> peak amp
    sig_frames = 0
    sig_sumsq = 0.0
    nframes = 0
    for _ in range(n_periods):
        xf = amp * tone                              # float64 passband, ~sig_ref RMS
        if not a.passthrough:
            ch.process(xf.tolist())                  # exercise the real channel
        sig_frames += PERIOD
        sig_sumsq += float(np.dot(xf, xf))
        nframes += PERIOD
    return {"frames": nframes, "sig_frames": sig_frames,
            "sig_sumsq": sig_sumsq, "underruns": 0, "dry_run": True}


def _resolve_cell(a):
    """Return (commanded_snr, snr_offset, cell_label, realized_snr3k) from the
    args, matching main(). --cell is the IONOS dial label (WGN:N) and follows
    the measured affine-plus-endpoint-floor map. A direct --snr (no --cell) is
    already the realized SNR3k (offset 0)."""
    if a.cell:
        commanded = float(a.cell.split(":", 1)[1])
        realized = ionos_wgn_to_snr3k(commanded)
        offset = realized - commanded
        return commanded, offset, a.cell, realized
    commanded = float(a.snr)
    return commanded, 0.0, f"{a.profile.upper()}:{a.snr:g}", commanded


def self_test():
    """No-ALSA self-test: run the bridge's dry path across both passthrough
    polarities and a WGN + MPG cell + the direct --snr path, and assert each
    statsfile carries a well-formed [C1] channel_attestation that faithfully
    records the session it ran. Exits 0 on PASS, 1 on FAIL. Runs anywhere
    (no alsaaudio / snd-aloop needed)."""
    import json
    import tempfile
    cases = [
        # (cell, profile, passthrough)
        ("WGN:40", "wgn", False),
        ("WGN:15", "mpg", False),
        (None,     "wgn", False),      # --snr direct path (offset must be 0)
        ("WGN:30", "wgn", True),       # a passthrough bridge MUST be visible
    ]
    ok = True
    for cell, profile, passthrough in cases:
        a = types.SimpleNamespace(
            snr=30.0, cell=cell, profile=profile, cfo_hz=0.0,
            phase_noise_deg=0.0, fade_depth_db=0.0, loss=0.0, burst=False,
            sig_ref=0.15, seed=7, passthrough=passthrough)
        commanded, offset, label, _snr3k = _resolve_cell(a)
        a.snr = _snr3k
        cargs = chan_args(a)
        ch_fwd = Channel(cargs, (a.seed * 2654435761) & 0xFFFFFFFF)
        ch_rev = Channel(cargs, (a.seed * 40503 + 7) & 0xFFFFFFFF)
        stats = {"fwd": _dry_pump(ch_fwd, a), "rev": _dry_pump(ch_rev, a)}
        stats["channel_attestation"] = build_channel_attestation(
            a, stats, commanded, offset, label)
        # round-trip through the real statsfile write/read path.
        fd, path = tempfile.mkstemp(prefix="bridge_selftest_", suffix=".json")
        os.close(fd)
        with open(path, "w") as f:
            json.dump(stats, f, indent=2)
        with open(path) as f:
            loaded = json.load(f)
        os.unlink(path)
        att = loaded.get("channel_attestation")
        try:
            validate_channel_attestation(att)
            assert att["passthrough"] == passthrough, \
                "passthrough polarity not recorded"
            assert att["profile"] == profile.upper(), "profile not recorded"
            assert att["seed"] == 7, "seed not recorded"
            expect_offset = (_snr3k - commanded) if cell else 0.0
            assert abs(att["realized_snr_offset_db"] - expect_offset) < 1e-9, \
                "realized_snr_offset_db wrong"
            if cell:
                assert att["cell"] == cell, "cell label not recorded"
                assert att["commanded_snr"] == float(cell.split(":")[1]), \
                    "commanded_snr wrong"
            # non-passthrough sessions carry real signal -> ~sig_ref**2 power.
            assert att["realized_p_sig"] > 0.0, "no signal power measured"
            assert abs(att["realized_p_sig"] - 0.15 ** 2) < 0.5 * 0.15 ** 2, \
                f"realized_p_sig implausible: {att['realized_p_sig']}"
            # the existing per-direction health fields must survive intact.
            for d in ("fwd", "rev"):
                assert "underruns" in loaded[d] and "frames" in loaded[d], \
                    f"{d} health fields missing"
            sys.stderr.write(
                f"[bridge_s32] self-test OK  cell={cell} profile={profile} "
                f"passthrough={passthrough} "
                f"p_sig={att['realized_p_sig']:.5f} offset={att['realized_snr_offset_db']}\n")
        except AssertionError as e:
            ok = False
            sys.stderr.write(f"[bridge_s32] self-test FAIL cell={cell} "
                             f"profile={profile} passthrough={passthrough}: {e}\n")
    if ok:
        sys.stderr.write("[bridge_s32] SELF-TEST PASS\n")
        sys.exit(0)
    sys.stderr.write("[bridge_s32] SELF-TEST FAIL\n")
    sys.exit(1)


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
    # type=str.lower so an uppercase channel label (e.g. --profile MPM, natural
    # for a cell like MPM:15) is accepted: argparse applies type BEFORE choices,
    # so "MPM" -> "mpm" passes. A case mismatch used to make the bridge exit 2 at
    # launch with NO statsfile written -> the attestation read stats_missing and
    # the whole faded cell went instrument-invalid while the AWGN/lowercase path
    # worked. Normalizing the case here repairs that instrument at the root.
    ap.add_argument("--profile", type=str.lower, choices=list(PROFILES.keys()),
                    default="wgn")
    ap.add_argument("--cfo-hz", type=float, default=0.0)
    ap.add_argument("--phase-noise-deg", type=float, default=0.0)
    ap.add_argument("--fade-depth-db", type=float, default=0.0)
    # narrow-FM audio bandpass (radio audio filter). Default OFF so the Mercury
    # real-audio path (arq_realaudio.py, which never passes this) is unchanged.
    # The IRIS_AUDIO_BANDPASS env var provides a fallback so the Iris matrix can
    # enable it without editing the (inner-repo) runner.
    ap.add_argument("--audio-bandpass", choices=list(BANDPASS_PRESETS.keys()),
                    default=os.environ.get("IRIS_AUDIO_BANDPASS", "off"),
                    help="off (default) / narrow (300-2900 Hz) / wide (300-6300 Hz)")
    ap.add_argument("--bandpass-lo-hz", type=float, default=0.0,
                    help="explicit bandpass low edge Hz (>0 overrides preset)")
    ap.add_argument("--bandpass-hi-hz", type=float, default=0.0,
                    help="explicit bandpass high edge Hz (>0 overrides preset)")
    ap.add_argument("--bandpass-taps", type=int, default=BANDPASS_DEFAULT_TAPS,
                    help=f"FIR bandpass taps (default {BANDPASS_DEFAULT_TAPS})")
    ap.add_argument("--loss", type=float, default=0.0)
    ap.add_argument("--burst", action="store_true")
    ap.add_argument("--sig-ref", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--cap-periods", type=int, default=3)
    ap.add_argument("--play-periods", type=int, default=4)
    ap.add_argument("--prime-periods", type=int, default=2)
    ap.add_argument("--statsfile", default=None)
    ap.add_argument("--dry-run", action="store_true",
                    help="no-ALSA in-memory run: build+write the statsfile "
                         "(with channel_attestation) from a synthetic signal")
    ap.add_argument("--self-test", action="store_true",
                    help="run the no-ALSA dry path across both passthrough "
                         "polarities + WGN/MPG cells and assert the statsfile "
                         "carries a well-formed [C1] channel_attestation; exit 0/1")
    a = ap.parse_args()

    if a.self_test:
        self_test()          # exits 0/1

    # Resolve the commanded (IONOS dial-label) SNR, the realized offset applied,
    # the realized SNR3k the channel runs at, and the cell label to attest.
    commanded_snr, snr_offset, cell_label, a.snr = _resolve_cell(a)

    cargs = chan_args(a)
    ch_fwd = Channel(cargs, (a.seed * 2654435761) & 0xFFFFFFFF)
    ch_rev = Channel(cargs, (a.seed * 40503 + 7) & 0xFFFFFFFF)

    # --dry-run: exercise the whole build+attest+write path with NO ALSA (runs
    # on any host). A single in-memory synthetic cohort -> statsfile.
    if a.dry_run:
        import json
        import tempfile
        stats = {"fwd": _dry_pump(ch_fwd, a), "rev": _dry_pump(ch_rev, a)}
        stats["channel_attestation"] = build_channel_attestation(
            a, stats, commanded_snr, snr_offset, cell_label)
        statsfile = a.statsfile
        if not statsfile:
            fd, statsfile = tempfile.mkstemp(prefix="bridge_dry_", suffix=".json")
            os.close(fd)
        with open(statsfile, "w") as f:
            json.dump(stats, f, indent=2)
        sys.stderr.write(f"[bridge_s32] DRY-RUN wrote {statsfile}\n"
                         f"[bridge_s32] channel_attestation="
                         f"{stats['channel_attestation']}\n")
        return

    if alsaaudio is None:
        sys.stderr.write("[bridge_s32] alsaaudio unavailable; this host cannot "
                         "run the live snd-aloop bridge. Use --self-test or "
                         "--dry-run for a no-ALSA check.\n")
        sys.exit(2)
    _bp = chan_args(a)
    _bp_s = (f" bandpass={a.audio_bandpass}({_bp.bandpass_lo_hz:.0f}-{_bp.bandpass_hi_hz:.0f}Hz)"
             if _bp.bandpass_hi_hz > _bp.bandpass_lo_hz > 0 else " bandpass=off")
    mode = ("PASSTHROUGH" if a.passthrough
            else f"SNR3k={a.snr:.2f} profile={a.profile}{_bp_s}")
    sys.stderr.write(f"[bridge_s32] {mode} cfo={a.cfo_hz} pn={a.phase_noise_deg} "
                     f"seed={a.seed} rings cap={a.cap_periods} play={a.play_periods} "
                     f"prime={a.prime_periods} cables "
                     f"fwd[{a.fwd_cap}->{a.fwd_play}] rev[{a.rev_cap}->{a.rev_play}]\n")
    sys.stderr.flush()

    stop = threading.Event()
    stats = {}

    # NIT A: durable statsfile.  Bind the live stats + attestation params so the
    # flusher persists a COMPLETE [C1] snapshot incrementally + on SIGTERM/atexit
    # (see the durable-flush section above), instead of a single write at main() end
    # that an interrupted teardown can lose (-> C1 lost).
    if a.statsfile:
        _register_stats_sink(a.statsfile, stats, a, commanded_snr, snr_offset, cell_label)
        atexit.register(flush_stats)

    # On SIGTERM (the harness shuts the bridge down with terminate()), set stop so
    # the pump threads exit their loop AND flush the CURRENT stats immediately, so a
    # complete C1 snapshot is durable before the harness releases the card instead of
    # leaving the statsfile empty (which blinded the runner gate
    # to the whole attestation, and to the underrun count, the key ring-health signal).
    def _on_term(signum, frame):  # noqa: ARG001
        stop.set()
        flush_stats()
    signal.signal(signal.SIGTERM, _on_term)

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
            flush_stats()            # incremental durable snapshot (0.5 s cadence)
    except KeyboardInterrupt:
        pass
    stop.set()
    tf.join(timeout=3)
    tr.join(timeout=3)
    # [C1] final attest of what channel ACTUALLY ran, alongside per-direction health.
    stats["channel_attestation"] = build_channel_attestation(
        a, stats, commanded_snr, snr_offset, cell_label)
    sys.stderr.write(f"[bridge_s32] stats={stats}\n")
    if a.statsfile:
        flush_stats()


if __name__ == "__main__":
    main()

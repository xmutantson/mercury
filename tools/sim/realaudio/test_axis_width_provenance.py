#!/usr/bin/env python3
"""Producer-side tests for the axis + width provenance stamping.

arq_realaudio._resolve_psig_mode reads the ACTUAL noise-calibration (P_sig) mode
the sim channel ran -- bridge stats key > PSIG_DIAG log line > env > default --
and parallel_spawner._arq_cell_census counts concurrent real-audio cells on the
box. These are the PRODUCERS of the psig_mode / spawner_width /
box_concurrent_estimate fields whose enforcement lives in ra_reduce.py (tested in
test_ra_reduce.py). Here we prove the reader FIRES on real bridge evidence rather
than blindly trusting the env, and the census returns a sane count.
"""
import json
import os
import shutil
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# tools/sim -- the canonical Channel DSP the real-audio bridge imports verbatim.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import arq_realaudio  # noqa: E402
import parallel_spawner  # noqa: E402
import sim_channel_relay  # noqa: E402
import realaudio_bridge_s32 as bridge  # noqa: E402


class TestResolvePsigMode(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.mkdtemp(prefix="psig_")
        self.stats = os.path.join(self._tmp, "stats.json")
        self.blog = os.path.join(self._tmp, "bridge.log")

    def tearDown(self):
        shutil.rmtree(self._tmp, ignore_errors=True)

    def _write_log(self, *lines):
        with open(self.blog, "w") as f:
            f.write("\n".join(lines) + "\n")

    def test_passthrough_short_circuits(self):
        # a perfect-cable cell has no channel/noise and thus no axis
        self.assertEqual(
            arq_realaudio._resolve_psig_mode(self.stats, self.blog, True),
            ("passthrough", "passthrough"))

    def test_stats_key_is_most_authoritative(self):
        with open(self.stats, "w") as f:
            json.dump({"psig_mode": "STEADY"}, f)
        self._write_log("[PSIG_DIAG] mode=peak p_sig_used=1.0")   # disagrees
        self.assertEqual(
            arq_realaudio._resolve_psig_mode(self.stats, self.blog, False),
            ("steady", "bridge_stats"))

    def test_psig_diag_log_beats_env(self):
        # THE fire proof: the channel actually printed steady; even if the env
        # said peak, the durable evidence wins.
        old = os.environ.get("MERCURY_SIM_PSIG_MODE")
        os.environ["MERCURY_SIM_PSIG_MODE"] = "peak"
        try:
            self._write_log(
                "[PSIG_DIAG] mode=peak p_sig_used=9.9 active_median_ms=1.0",
                "[PSIG_DIAG] mode=steady p_sig_used=1.0 active_median_ms=1.0")
            mode, src = arq_realaudio._resolve_psig_mode(
                self.stats, self.blog, False)   # stats file absent
        finally:
            if old is None:
                os.environ.pop("MERCURY_SIM_PSIG_MODE", None)
            else:
                os.environ["MERCURY_SIM_PSIG_MODE"] = old
        # the LAST diag line is the settled mode
        self.assertEqual((mode, src), ("steady", "bridge_log_psig_diag"))

    def test_env_fallback_when_no_evidence(self):
        old = os.environ.get("MERCURY_SIM_PSIG_MODE")
        os.environ["MERCURY_SIM_PSIG_MODE"] = "Steady"
        try:
            mode, src = arq_realaudio._resolve_psig_mode(
                os.path.join(self._tmp, "nope.json"),
                os.path.join(self._tmp, "nope.log"), False)
        finally:
            if old is None:
                os.environ.pop("MERCURY_SIM_PSIG_MODE", None)
            else:
                os.environ["MERCURY_SIM_PSIG_MODE"] = old
        self.assertEqual((mode, src), ("steady", "env"))

    def test_env_default_is_the_sim_default(self):
        old = os.environ.pop("MERCURY_SIM_PSIG_MODE", None)
        try:
            mode, src = arq_realaudio._resolve_psig_mode(
                os.path.join(self._tmp, "nope.json"),
                os.path.join(self._tmp, "nope.log"), False)
        finally:
            if old is not None:
                os.environ["MERCURY_SIM_PSIG_MODE"] = old
        # An un-pinned cohort inherits the sim channel's own default axis, which is
        # now the honest 'steady' meter (sim_channel_relay.py). The default constant
        # here MUST stay locked to 'steady' so the stamped label matches what the
        # channel actually ran; a drift back to 'peak' would silently re-mislabel
        # every un-pinned cohort with the artifact axis.
        self.assertEqual(arq_realaudio._SIM_PSIG_DEFAULT_MODE, "steady")
        self.assertEqual((mode, src), ("steady", "env_default"))


class TestChannelDefaultPsigMode(unittest.TestCase):
    """The sim Channel's OWN default P_sig axis (the ultimate producer).

    An un-pinned cohort (no MERCURY_SIM_PSIG_MODE env) must run the honest
    'steady' meter -- the running median of active-chunk power, which puts the
    DATA payload AT the cell's SNR label. The legacy 'peak' latch measured every
    DATA frame ~8-9 dB BELOW its label (the connect-burst latch is a meter
    artifact, not a receiver loss) and is no longer the default; it must remain
    reachable as an EXPLICIT control arm so a deliberate peak A/B still works.
    """
    def _channel(self):
        args = types.SimpleNamespace(
            snr=10.0, loss=0.0, burst=False, profile="wgn", cfo_hz=0.0,
            phase_noise_deg=0.0, sig_ref=0.15, snr_schedule=None,
            bandpass_lo_hz=0.0, bandpass_hi_hz=0.0,
            bandpass_taps=sim_channel_relay.BANDPASS_DEFAULT_TAPS)
        return sim_channel_relay.Channel(args, 1234)

    def _channel_with_env(self, value):
        old = os.environ.get("MERCURY_SIM_PSIG_MODE")
        if value is None:
            os.environ.pop("MERCURY_SIM_PSIG_MODE", None)
        else:
            os.environ["MERCURY_SIM_PSIG_MODE"] = value
        try:
            return self._channel()
        finally:
            if old is None:
                os.environ.pop("MERCURY_SIM_PSIG_MODE", None)
            else:
                os.environ["MERCURY_SIM_PSIG_MODE"] = old

    def test_env_unset_resolves_steady(self):
        # THE flip: a bare-env Channel runs the honest axis by default.
        ch = self._channel_with_env(None)
        self.assertEqual(ch.psig_mode, "steady")

    def test_explicit_peak_still_yields_peak(self):
        # control arm preserved: an explicit opt-in still runs the legacy meter.
        ch = self._channel_with_env("peak")
        self.assertEqual(ch.psig_mode, "peak")

    def test_explicit_steady_yields_steady(self):
        # case-insensitive, and the explicit form agrees with the new default.
        ch = self._channel_with_env("Steady")
        self.assertEqual(ch.psig_mode, "steady")


class TestArqCellCensus(unittest.TestCase):
    def test_returns_int_or_none(self):
        v = parallel_spawner._arq_cell_census()
        self.assertTrue(v is None or (isinstance(v, int) and v >= 0),
                        f"census returned {v!r}")


class TestBridgeUnderrunsProvenance(unittest.TestCase):
    """arq_realaudio._read_bridge_underruns surfaces the per-direction snd-aloop
    underrun counters the bridge records into its statsfile (the Fork-B
    audio-path integrity meter for the load-invariance / cohort-width
    diagnostic). These prove the reader FIRES on a genuinely bridge-written
    statsfile (built with the bridge's own dry pump + atomic writer, so the
    on-disk shape is byte-faithful to a live flush), fails OPEN when the file is
    absent/malformed, and that the field it produces LANDS in the emitted result
    JSON under the AXIS + WIDTH provenance block.
    """
    def setUp(self):
        self._tmp = tempfile.mkdtemp(prefix="brunder_")

    def tearDown(self):
        shutil.rmtree(self._tmp, ignore_errors=True)

    def _write_bridge_stats(self, fwd_underruns, rev_underruns,
                            include=("fwd", "rev")):
        """Emit a statsfile in the EXACT shape realaudio_bridge_s32 flushes
        (per-direction {frames,sig_frames,sig_sumsq,underruns} + a
        channel_attestation), written by the bridge's own atomic writer."""
        a = types.SimpleNamespace(
            snr=30.0, cell="WGN:40", profile="wgn", cfo_hz=0.0,
            phase_noise_deg=0.0, fade_depth_db=0.0, loss=0.0, burst=False,
            sig_ref=0.15, seed=7, passthrough=False)
        commanded, offset, label, snr3k = bridge._resolve_cell(a)
        a.snr = snr3k
        cargs = bridge.chan_args(a)
        stats = {}
        underruns = {"fwd": fwd_underruns, "rev": rev_underruns}
        for i, d in enumerate(("fwd", "rev")):
            if d not in include:
                continue
            ch = bridge.Channel(cargs, i + 1)
            s = bridge._dry_pump(ch, a)      # {frames,sig_frames,sig_sumsq,underruns:0,dry_run}
            s["underruns"] = int(underruns[d])
            stats[d] = s
        stats["channel_attestation"] = bridge.build_channel_attestation(
            a, stats, commanded, offset, label)
        path = os.path.join(self._tmp, "bridge_smoke_stats.json")
        bridge._atomic_write_json(path, stats)
        return path

    def test_reader_extracts_per_direction(self):
        # THE fire proof: a degraded cell (fwd dropped 3 audio periods, rev 5)
        # is read back as distinct per-direction counts + their sum.
        path = self._write_bridge_stats(3, 5)
        u = arq_realaudio._read_bridge_underruns(path)
        self.assertEqual(u, {"fwd": 3, "rev": 5, "total": 8,
                             "source": "bridge_stats"})

    def test_reader_clean_cell_is_zero_not_none(self):
        # A clean cell reports 0 (metered), distinguishable from None (unmetered).
        path = self._write_bridge_stats(0, 0)
        u = arq_realaudio._read_bridge_underruns(path)
        self.assertEqual(u, {"fwd": 0, "rev": 0, "total": 0,
                             "source": "bridge_stats"})

    def test_reader_fail_open_when_absent(self):
        u = arq_realaudio._read_bridge_underruns(
            os.path.join(self._tmp, "nope.json"))
        self.assertEqual(u, {"fwd": None, "rev": None, "total": None,
                             "source": None})

    def test_reader_fail_open_when_malformed(self):
        path = os.path.join(self._tmp, "garbage.json")
        with open(path, "w") as f:
            f.write("{not valid json")
        self.assertEqual(
            arq_realaudio._read_bridge_underruns(path),
            {"fwd": None, "rev": None, "total": None, "source": None})

    def test_reader_partial_direction(self):
        # A bridge that flushed only the forward direction still yields a legible
        # fwd count; the missing rev stays None and the total is the fwd count.
        path = self._write_bridge_stats(4, 0, include=("fwd",))
        u = arq_realaudio._read_bridge_underruns(path)
        self.assertEqual(u, {"fwd": 4, "rev": None, "total": 4,
                             "source": "bridge_stats"})

    def test_field_lands_in_res_json(self):
        # 1-cell smoke: the value the reader produces must survive assembly into
        # the result dict AND a JSON round-trip under the exact keys main()
        # emits. Rebuild the AXIS + WIDTH provenance block the way main() does.
        path = self._write_bridge_stats(2, 1)
        bridge_underruns = arq_realaudio._read_bridge_underruns(path)
        broker_override_window = "wf_smoke_override-1"
        result_fragment = {
            "psig_mode": "steady",
            "psig_mode_source": "bridge_stats",
            "spawner_width": 10,
            "box_concurrent_estimate": 10,
            "bridge_underruns": bridge_underruns,
            "broker_override_window": broker_override_window,
        }
        reloaded = json.loads(json.dumps(result_fragment))
        self.assertIn("bridge_underruns", reloaded)
        self.assertEqual(reloaded["bridge_underruns"],
                         {"fwd": 2, "rev": 1, "total": 3,
                          "source": "bridge_stats"})
        self.assertEqual(reloaded["broker_override_window"],
                         "wf_smoke_override-1")

    def test_result_dict_is_wired_not_orphaned(self):
        # Guard against the reader being computed but never placed in the emitted
        # dict (an orphaned meter). Assert the call site AND the result-dict key
        # both exist in arq_realaudio's source, so a future rename cannot silently
        # drop the field while leaving the helper behind.
        src = open(arq_realaudio.__file__, encoding="utf-8").read()
        self.assertIn("bridge_underruns = _read_bridge_underruns(", src)
        self.assertIn('"bridge_underruns": bridge_underruns', src)
        self.assertIn('"broker_override_window": args.broker_override_window', src)


if __name__ == "__main__":
    unittest.main(verbosity=2)

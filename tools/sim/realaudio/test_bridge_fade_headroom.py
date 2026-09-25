#!/usr/bin/env python3
"""Fade headroom and the clip log of realaudio_bridge_s32_c (file I/O mode, no sound card).

fade : on a fading profile the receiver gain is capped at 1/2, so a 1500 Hz tone whose
       peaks sit at 0.6 of full scale no longer clips where the fade lifts it; on WGN
       the gain is unchanged (no pad without fading).
clip log : with the headroom law off the same fading run clips, and the per-chunk clip
       log adds up to the bridge's own clip total.

BRIDGE_BIN selects the bridge under test (default ./realaudio_bridge_s32_c).
"""
import json
import math
import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BIN = Path(os.environ.get("BRIDGE_BIN", str(HERE / "realaudio_bridge_s32_c")))
FS = 48000
CLIP_RE = re.compile(r"vector clip hard=(\d+) over_fs_unscaled=(\d+) den=(\d+) composite_scale=([0-9.eE+-]+)")


def tone_s32(seconds, peak):
    n = int(seconds * FS)
    x = peak * np.sin(2.0 * math.pi * 1500.0 * np.arange(n) / FS)
    v = np.round(x * 2147483647.0).astype("<i4")
    return np.repeat(v, 2)


def run(profile, *opts, clip_log=None, seconds=30.0, peak=0.6):
    with tempfile.TemporaryDirectory(prefix="fade_hr_") as td:
        pi, po = Path(td) / "in.s32", Path(td) / "out.s32"
        tone_s32(seconds, peak).tofile(pi)
        argv = [str(BIN), "--vector-in", str(pi), "--vector-out", str(po), "--profile", profile,
                "--snr", "40", "--seed", "7", *opts]
        if clip_log:
            argv += ["--clip-log", str(Path(td) / "clips.jsonl")]
        cp = subprocess.run(argv, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=300)
        err = cp.stderr.decode(errors="replace")
        if cp.returncode != 0:
            raise RuntimeError("bridge exited %d: %s" % (cp.returncode, err.strip()))
        m = CLIP_RE.search(err)
        if not m:
            raise RuntimeError("no vector clip line in: %s" % err)
        events = []
        if clip_log:
            events = [json.loads(l) for l in (Path(td) / "clips.jsonl").read_text().splitlines() if l.strip()]
        return {"hard": int(m.group(1)), "over": int(m.group(2)), "den": int(m.group(3)),
                "scale": float(m.group(4)), "events": events}


@unittest.skipUnless(BIN.is_file(), "bridge binary not built")
class FadeHeadroom(unittest.TestCase):
    def test_fading_gain_is_capped_and_the_tone_does_not_clip(self):
        r = run("mpp")
        self.assertEqual(r["scale"], 0.5)
        self.assertEqual(r["hard"], 0, r)

    def test_wgn_keeps_unit_gain(self):
        r = run("wgn")
        self.assertEqual(r["scale"], 1.0)
        self.assertEqual(r["hard"], 0, r)

    def test_clip_log_adds_up_to_the_clip_total(self):
        r = run("mpp", "--headroom-rms", "0", clip_log=True)
        self.assertGreater(r["hard"], 0, "headroom off: the fading run must clip")
        self.assertEqual(sum(e["hard_clips"] for e in r["events"]), r["hard"])
        self.assertEqual(sum(e["over_fs_unscaled"] for e in r["events"]), r["over"])
        self.assertTrue(all(e["event"] == "clip" and e["dir"] == "vec" for e in r["events"]))
        self.assertTrue(all(e["hard_clips"] or e["over_fs_unscaled"] for e in r["events"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)

#!/usr/bin/env python3
"""Directed harness tests with stub modem and bridge programs (Linux; no audio, no modem).

1. A busy modem port is refused before anything launches: the cell reads
   INSTRUMENT_INVALID with port_unavailable:<port>, and the stub modem never runs.
2. A modem that never opens its control port makes tcp_send give up; that
   exception is recorded (harness_exception:ConnectionRefusedError) and the
   cell reads INSTRUMENT_INVALID, never a modem outcome.
"""
import json
import os
import socket
import stat
import subprocess
import sys
import tempfile
import textwrap
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.join(HERE, "arq_realaudio.py")

STUB_MODEM = textwrap.dedent("""\
    #!%s
    import os, sys, time
    marker = os.environ.get("STUB_MODEM_MARKER")
    if marker:
        with open(marker, "a") as f:
            f.write(" ".join(sys.argv[1:]) + "\\n")
    time.sleep(float(os.environ.get("STUB_MODEM_SLEEP_S", "120")))
    """) % sys.executable

STUB_BRIDGE = textwrap.dedent("""\
    import time
    time.sleep(120)
    """)


def free_base_port():
    """A base port P with P..P+5 free right now (the cell uses P, P+1, P+4, P+5)."""
    for _ in range(50):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        base = s.getsockname()[1]
        s.close()
        if base + 5 > 65535:
            continue
        ok = True
        for p in range(base, base + 6):
            t = socket.socket()
            try:
                t.bind(("0.0.0.0", p))
            except OSError:
                ok = False
            finally:
                t.close()
        if ok:
            return base
    raise RuntimeError("no free port block")


@unittest.skipIf(os.name == "nt", "the harness runs on the Linux fleet")
class HarnessPreconditions(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="h445_")
        self.modem = os.path.join(self.tmp, "stub_modem")
        with open(self.modem, "w") as f:
            f.write(STUB_MODEM)
        os.chmod(self.modem, os.stat(self.modem).st_mode | stat.S_IXUSR)
        self.bridge = os.path.join(self.tmp, "stub_bridge.py")
        with open(self.bridge, "w") as f:
            f.write(STUB_BRIDGE)
        self.marker = os.path.join(self.tmp, "modem_launched.txt")

    def run_cell(self, base, tag, timeout):
        out = os.path.join(self.tmp, tag + ".json")
        env = dict(os.environ, STUB_MODEM_MARKER=self.marker, STUB_MODEM_SLEEP_S="120")
        for k in ("MERCURY_SIM_PSIG_MODE", "MERCURY_TX_GAIN_INI"):
            env.pop(k, None)
        argv = [sys.executable, HARNESS, "--bin", self.modem, "--bridge", self.bridge,
                "--snr3k", "20", "--profile", "wgn", "--secs", "5", "--payload", "1000",
                "--no-warm-start", "--traffic", "random-binary", "--force-compress", "off",
                "--mode", "auto", "--start-cfg", "-1", "--tag", tag, "--card", "99",
                "--rsp-port", str(base), "--cmd-port", str(base + 4), "--no-kill",
                "--logdir", os.path.join(self.tmp, "logs_" + tag), "--json", out]
        proc = subprocess.run(argv, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=timeout)
        self.assertTrue(os.path.exists(out), "no result JSON; harness output:\n" + proc.stdout[-3000:])
        with open(out) as f:
            return json.load(f), proc.stdout

    def test_busy_port_refused_before_launch(self):
        base = free_base_port()
        holder = socket.socket()
        holder.bind(("0.0.0.0", base))  # a plain socket on the port, as the stale client held it
        try:
            result, output = self.run_cell(base, "busyport", timeout=240)
        finally:
            holder.close()
        reasons = result.get("instrument_invalid_reasons") or []
        self.assertIn("port_unavailable:%d" % base, reasons, output[-2000:])
        self.assertTrue(result.get("instrument_invalid"))
        self.assertEqual(result.get("whole_session_status"), "INSTRUMENT_INVALID")
        self.assertFalse(os.path.exists(self.marker), "the modem was launched on a busy port")

    def test_modem_that_never_listens_is_an_instrument_failure(self):
        base = free_base_port()
        result, output = self.run_cell(base, "nolisten", timeout=300)
        reasons = result.get("instrument_invalid_reasons") or []
        self.assertIn("harness_exception:ConnectionRefusedError", reasons, output[-2000:])
        self.assertTrue(result.get("instrument_invalid"))
        self.assertEqual(result.get("whole_session_status"), "INSTRUMENT_INVALID")
        self.assertEqual((result.get("harness_exception") or {}).get("type"),
                         "ConnectionRefusedError")
        self.assertTrue(os.path.exists(self.marker), "the stub modem was never launched")


if __name__ == "__main__":
    unittest.main()

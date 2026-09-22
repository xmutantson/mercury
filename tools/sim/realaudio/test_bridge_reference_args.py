#!/usr/bin/env python3
"""Unit tests for bridge_reference: the harness-side noise-reference options.

With --bin, also launches the bridge in --dry-run through the harness prefix
(sudo -n nice for the production name) with MERCURY_SIM_PSIG_MODE=fix and, in a
second pass, with the default (bench) mode and checks that the attested mode and reference come from the
command line, and that a launch without the options is rejected.
"""
import argparse
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bridge_reference as BR  # noqa: E402


def check(cond, msg):
    print("[%s] %s" % ("PASS" if cond else "FAIL", msg))
    return bool(cond)


def unit():
    ok = True
    ref = repr(BR.OFDM_REF)
    argv, req = BR.resolve({})
    ok &= check(argv == ["--reference-mode", "bench", "--ofdm-ref", ref] and req["psig_mode"] == "bench",
                "unset -> bench with the OFDM WB reference")
    argv, req = BR.resolve({"MERCURY_SIM_PSIG_MODE": "steady"})
    ok &= check(req["psig_mode"] == "bench", "steady -> bench")
    argv, req = BR.resolve({"MERCURY_SIM_PSIG_MODE": "per-class"})
    ok &= check(argv == ["--reference-mode", "geometry"], "per-class -> geometry (optional mode)")
    argv, req = BR.resolve({"MERCURY_SIM_PSIG_MODE": "legacy-median"})
    ok &= check(req["psig_mode"] == "legacy-median", "legacy-median kept")
    argv, req = BR.resolve({"MERCURY_SIM_PSIG_MODE": "fix", "MERCURY_SIM_PSIG_FIX": "0.0206"})
    ok &= check(argv == ["--reference-mode", "fix", "--psig-fix", "0.0206"], "fix carries --psig-fix")
    argv, req = BR.resolve({"MERCURY_SIM_OFDM_REF": "0.02"})
    ok &= check(argv == ["--reference-mode", "bench", "--ofdm-ref", "0.02"], "MERCURY_SIM_OFDM_REF overrides the reference")
    for bad in ({"MERCURY_SIM_PSIG_MODE": "fix"}, {"MERCURY_SIM_PSIG_MODE": "median"},
                {"MERCURY_SIM_OFDM_REF": "-1"}):
        try:
            BR.resolve(bad)
            ok &= check(False, "rejects %r" % bad)
        except BR.BridgeReferenceError:
            ok &= check(True, "rejects %r" % bad)
    with tempfile.TemporaryDirectory() as td:
        sf = os.path.join(td, "s.json")
        _, req = BR.resolve({"MERCURY_SIM_PSIG_MODE": "fix", "MERCURY_SIM_PSIG_FIX": "0.0206"})
        json.dump({"psig_mode": "fix", "psig_mode_source": "cli",
                   "axis_attestation": {"reference_power": 0.0206}}, open(sf, "w"))
        ok &= check(BR.verify(sf, req)["ok"], "verify accepts the attested fixed reference")
        json.dump({"axis_attestation": {"reference_power": 0.0225,
                                        "reference_mode": "steady-active-chunk-median"}}, open(sf, "w"))
        v = BR.verify(sf, req)
        ok &= check(not v["ok"] and "bridge_did_not_attest_reference_mode" in v["reasons"],
                    "verify rejects a bridge that does not attest its mode (the pre-change bridge)")
        json.dump({"psig_mode": "geometry", "psig_mode_source": "default",
                   "axis_attestation": {"reference_power": 0.0206}}, open(sf, "w"))
        ok &= check(not BR.verify(sf, req)["ok"], "verify rejects a mode mismatch")
        ok &= check(not BR.verify(os.path.join(td, "missing.json"), req)["ok"], "verify rejects a missing statsfile")
        _, req = BR.resolve({})
        good = {"psig_mode": "bench", "psig_mode_source": "cli", "axis_attestation": {"reference_power": BR.OFDM_REF},
                "bench": {"ofdm_reference_power": BR.OFDM_REF, "ofdm_reference_source": "cli",
                          "fwd_noise_variance": 0.0016, "rev_noise_variance": 0.0016},
                "waveform_kinds": {"fwd": {"ofdm-wb": {"transmissions": 3, "signal_power": BR.OFDM_REF * 1.02}}, "rev": {}}}
        json.dump(good, open(sf, "w"))
        v = BR.verify(sf, req)
        ok &= check(v["ok"] and not v["warnings"] and abs(v["ofdm_wb_level_delta_db"] - 0.086) < 1e-3,
                    "verify accepts bench and attests the OFDM WB level delta (%s)" % v.get("ofdm_wb_level_delta_db"))
        bad = json.loads(json.dumps(good))
        bad["bench"]["rev_noise_variance"] = 0.0017
        json.dump(bad, open(sf, "w"))
        ok &= check(not BR.verify(sf, req)["ok"], "verify rejects different forward / reverse noise")
        bad = json.loads(json.dumps(good))
        bad["bench"]["ofdm_reference_source"] = "default"
        json.dump(bad, open(sf, "w"))
        ok &= check(not BR.verify(sf, req)["ok"], "verify rejects an OFDM reference not from the command line")
        off = json.loads(json.dumps(good))
        off["waveform_kinds"]["fwd"]["ofdm-wb"]["signal_power"] = BR.OFDM_REF * 1.2
        json.dump(off, open(sf, "w"))
        v = BR.verify(sf, req)
        ok &= check(v["ok"] and v["warnings"], "verify warns when the measured OFDM WB level is off the reference")
    return ok


def live(binpath):
    import subprocess
    ok = True
    prefix = ["sudo", "-n", "/usr/bin/nice", "-n", "-5"] if os.path.basename(binpath) == "realaudio_bridge_s32_c" else []
    for envmode in ({"MERCURY_SIM_PSIG_MODE": "fix", "MERCURY_SIM_PSIG_FIX": "0.0206"}, {}):
        env = dict(os.environ, **envmode)
        env.pop("MERCURY_SIM_PSIG_MODE", None) if not envmode else None
        argv, req = BR.resolve(env)
        with tempfile.TemporaryDirectory() as td:
            for tag, extra, want in (("with_options", argv, True), ("without_options", [], not envmode and False)):
                sf = os.path.join(td, tag + ".json")
                p = subprocess.run(prefix + [binpath, "--dry-run", "--snr3k-db", "10", "--statsfile", sf] + extra,
                                   env=env, capture_output=True, text=True)
                v = BR.verify(sf, req) if p.returncode == 0 else {"ok": False, "reasons": [p.stderr.strip()]}
                ok &= check(v["ok"] == want, "live mode=%s %s prefix=%s verify_ok=%s %s" % (
                    req["psig_mode"], tag, prefix[:1], v["ok"], v["reasons"]))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin")
    a = ap.parse_args()
    ok = unit()
    if a.bin:
        ok &= live(a.bin)
    print("RESULT %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

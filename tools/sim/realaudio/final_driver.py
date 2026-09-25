#!/usr/bin/env python3
"""Run the immutable dial-40 column-E fade completion cohort on fleet audio."""
import hashlib
import json
import os
import subprocess
import sys
import threading
import time

ROOT = os.environ.get("FA_ROOT", "/dev/shm/fade_arq_final")
HARNESS = ROOT + "/harness/arq_realaudio.py"
BRIDGE = ROOT + "/realaudio_bridge_s32_c"
TX_GAIN_INI = ROOT + "/harness/txgain_bench_calibrated.ini"
sys.path.insert(0, ROOT + "/harness")
import wb_lib


def log(message):
    print("[fade_arq_final %s] %s" % (time.strftime("%H:%M:%S"), message),
          flush=True)


def file_hash(path, name):
    digest = hashlib.new(name)
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


class LocalShell:
    def run(self, box, command, timeout=None, check=False):
        return subprocess.run(["bash", "-c", command], text=True,
                              stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, timeout=timeout)


def pulse(path, stop):
    while not stop.wait(15):
        with open(path, "a", encoding="utf-8") as stream:
            stream.write("%s pulse\n" % time.strftime("%FT%T%z"))


def launch(cell, card, index, outroot, width, cohort):
    label = cell["label"]
    rundir = os.path.join(outroot, label)
    logdir = os.path.join(rundir, "logs")
    os.makedirs(logdir, exist_ok=True)
    rsp_port = int(cohort.get("port_base", 9200)) + index * 20
    cmd_port = rsp_port + 4
    binary = cohort["bin"]
    seconds = int(cell.get("secs", cohort.get("secs", 560)))
    payload = int(cell.get("payload", cohort.get("payload", 183717)))
    argv = [
        sys.executable, "-u", HARNESS,
        "--bin", binary, "--bridge", BRIDGE,
        "--tx-gain-ini", TX_GAIN_INI,
        "--start-cfg", "-1", "--mode", "auto",
        "--secs", str(seconds), "--payload", str(payload),
        "--score-horizon-s", str(seconds), "--no-warm-start",
        "--snr3k", "37.3", "--profile", cell["profile"],
        "--seed", str(cell["seed"]),
        "--traffic", "random-binary", "--force-compress", "off",
        "--tag", label, "--arm", cohort["agent"], "--wire-stamp", "0",
        "--card", str(card), "--subs", "0,1,2,3",
        "--rsp-port", str(rsp_port), "--cmd-port", str(cmd_port),
        "--no-kill", "--spawner-width", str(width),
        "--box-concurrent-estimate", str(width),
        "--logdir", logdir, "--json", os.path.join(rundir, "result.json"),
    ]
    env = dict(os.environ)
    env["MERCURY_GEARSHIFT_V2"] = "active"
    env["MERCURY_SIM_PSIG_MODE"] = "bench"
    env.pop("MERCURY_SIM_PSIG_FIX", None)
    env.pop("MERCURY_CONNECT_FAST_CONFIG", None)
    env["PYTHONPATH"] = ROOT + "/harness"
    hashes = {
        "binary_md5": file_hash(binary, "md5"),
        "binary_sha256": file_hash(binary, "sha256"),
        "bridge_sha256": file_hash(BRIDGE, "sha256"),
        "harness_sha256": file_hash(HARNESS, "sha256"),
        "tx_gain_ini_sha256": file_hash(TX_GAIN_INI, "sha256"),
    }
    with open(os.path.join(rundir, "argv.json"), "w", encoding="utf-8") as stream:
        json.dump({"argv": argv, "cell": cell, "card": card, "width": width,
                   "hashes": hashes, "psig_mode": "bench",
                   "column": "E", "dial_db": 40.0, "snr3k_db": 37.3,
                   "source_commit": cohort.get("source_commit"),
                   "source_tree": cohort.get("source_tree"),
                   "recipe": cohort.get("recipe"),
                   "exported": {key: env[key] for key in sorted(env)
                                if key.startswith("MERCURY_")}}, stream, indent=1)
    harness_log = open(os.path.join(rundir, "harness.log"), "w", encoding="utf-8")
    process = subprocess.Popen(argv, stdout=harness_log,
                               stderr=subprocess.STDOUT, env=env)
    log("LAUNCH %s card=%s profile=%s seed=%s" %
        (label, card, cell["profile"], cell["seed"]))
    return {"label": label, "process": process, "log": harness_log,
            "rundir": rundir, "logdir": logdir, "card": card,
            "started": time.time(), "cap": seconds + 380}


def grade(handle, wave_index, width):
    try:
        with open(os.path.join(handle["rundir"], "result.json"),
                  encoding="utf-8") as stream:
            result = json.load(stream)
    except Exception as exc:
        result = {"_error": repr(exc)}
    reference = result.get("bridge_reference_attestation", {})
    identity = result.get("harness_attestation", {})
    gain = result.get("tx_gain_attestation", {})
    underruns = result.get("bridge_underruns", {})
    channel = result.get("channel_attestation", {})
    attested = bool(
        reference.get("ok") and
        reference.get("attested", {}).get("psig_mode") == "bench" and
        identity.get("echoed_by_bridge") and
        gain.get("ok") and gain.get("mode") == "ini" and
        channel.get("valid") and
        result.get("requested_snr3k") == 37.3 and
        underruns.get("total") == 0 and
        result.get("force_compress") == "off")
    return {
        "label": handle["label"], "wave": wave_index,
        "card": handle["card"], "width": width,
        "rc": handle["process"].returncode,
        "wall_s": round(time.time() - handle["started"], 1),
        "attested": attested,
        "rx_bytes": result.get("rx_bytes"),
        "good_prefix_bytes": result.get("whole_session_good_prefix_bytes"),
        "delivered_full": result.get("delivered_full"),
        "byte_integrity_ok": result.get("byte_integrity_ok"),
        "whole_session_status": result.get("whole_session_status"),
        "terminal_eot_verified": result.get("terminal_eot_verified"),
        "force_compress": result.get("force_compress"),
        "reference_ok": reference.get("ok"),
        "channel_valid": channel.get("valid"),
        "tx_gain_ok": gain.get("ok"),
        "bridge_underruns": underruns,
    }


def run_wave(wave_index, cells, cohort, outroot, rows):
    width = len(cells)
    box = int(cohort.get("box", 31))
    wave_dir = os.path.join(outroot, "wave%d" % wave_index)
    os.makedirs(wave_dir, exist_ok=True)
    pre_fuser = subprocess.run(
        ["bash", "-c", "fuser /dev/snd/pcm* 2>&1 || true"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    with open(os.path.join(wave_dir, "fuser_before_grant.txt"), "w",
              encoding="utf-8") as stream:
        stream.write(pre_fuser.stdout)
    requests = [{"agent": "%s_w%d_%02d" % (cohort["agent"], wave_index, i),
                 "box": box, "share": 0} for i in range(width)]
    leases = wb_lib.WaveLeaseSet.acquire_atomic(
        requests, os.path.join(wave_dir, "leases"))
    records = leases.records()
    with open(os.path.join(wave_dir, "leases.json"), "w", encoding="utf-8") as stream:
        json.dump(records, stream, indent=1)
    cards = [record["physical_card"] for record in records]
    if len(cards) != len(set(cards)):
        raise RuntimeError("broker granted two cells on one physical card")
    handles = []
    first_row = len(rows)
    try:
        for index, cell in enumerate(cells):
            handles.append(launch(cell, cards[index], index, outroot, width, cohort))
            time.sleep(3)
        for handle in handles:
            remaining = handle["cap"] - (time.time() - handle["started"])
            try:
                handle["process"].wait(timeout=max(1, remaining))
            except subprocess.TimeoutExpired:
                log("HARDCAP %s -> SIGTERM" % handle["label"])
                handle["process"].terminate()
                try:
                    handle["process"].wait(timeout=30)
                except subprocess.TimeoutExpired:
                    handle["process"].terminate()
                    time.sleep(15)
            handle["log"].close()
            row = grade(handle, wave_index, width)
            rows.append(row)
            log("CELL %s rc=%s rx=%s full=%s attested=%s" %
                (row["label"], row["rc"], row["rx_bytes"],
                 row["delivered_full"], row["attested"]))
    finally:
        for handle in handles:
            if handle["process"].poll() is None:
                handle["process"].terminate()
        time.sleep(12)
        shell = LocalShell()
        evidence = [wb_lib.exact_card_fuser(shell, box, card) for card in cards]
        with open(os.path.join(wave_dir, "fuser_evidence.json"), "w",
                  encoding="utf-8") as stream:
            json.dump(evidence, stream, indent=1)
        release = leases.release_after_clean(evidence)
        with open(os.path.join(wave_dir, "wave_summary.json"), "w",
                  encoding="utf-8") as stream:
            json.dump({"release": repr(release), "cards": cards,
                       "width": width}, stream, indent=1)
        log("WAVE %d RELEASED cards=%s" % (wave_index, cards))
    invalid = [row["label"] for row in rows[first_row:]
               if row["rc"] != 0 or not row["attested"]]
    if invalid:
        raise RuntimeError("unattested or failed cells: %s" % ",".join(invalid))


def main():
    with open(sys.argv[1], encoding="utf-8") as stream:
        cohort = json.load(stream)
    outroot = sys.argv[2]
    os.makedirs(outroot, exist_ok=True)
    stop = threading.Event()
    threading.Thread(target=pulse, args=(os.path.join(outroot, "PULSE"), stop),
                     daemon=True).start()
    rows = []
    completed = False
    try:
        for wave_index, cells in enumerate(cohort["waves"]):
            run_wave(wave_index, cells, cohort, outroot, rows)
            with open(os.path.join(outroot, "cohort_summary.json"), "w",
                      encoding="utf-8") as stream:
                json.dump({"rows": rows}, stream, indent=1)
        completed = True
    finally:
        stop.set()
        with open(os.path.join(outroot, "cohort_summary.json"), "w",
                  encoding="utf-8") as stream:
            json.dump({"rows": rows}, stream, indent=1)
        marker = "DONE" if completed else "FAILED"
        with open(os.path.join(outroot, marker), "w", encoding="utf-8") as stream:
            stream.write("%s %s\n" % (marker.lower(), time.strftime("%FT%T%z")))


if __name__ == "__main__":
    main()

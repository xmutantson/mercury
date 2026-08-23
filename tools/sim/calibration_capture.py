#!/usr/bin/env python3
"""Capture and register fixed Pcal production-wire references.

Input captures are little-endian float64 mono samples at the declared sample
rate.  Statistics are computed in streaming blocks over an explicit steady
window; the tool never loads a capture wholesale.  ``synthesize-suite`` exists
only to create the deterministic presubmit fixtures used by
``test_simaxis_calibration.py``.  Campaign registries use ``capture`` with raw
samples emitted by the production TX wire.
"""

import argparse
import json
import math
import os
import re
import shutil
import sys

import numpy as np

from sim_axis import (NB_CONFIGURED_BANDWIDTH_HZ,
                      WB_CONFIGURED_BANDWIDTH_HZ, calibration_key,
                      sha256_file, sha256_json)


BLOCK_SAMPLES = 1 << 18
REGISTRY_SCHEMA = "simaxis-pcal-registry-v1"
SENSITIVE_ENV_RE = re.compile(
    r"(?:PASS|PASSWORD|PSK|SECRET|TOKEN|CREDENTIAL|PRIVATE|API[_-]?KEY)",
    re.IGNORECASE)


def measure_f64le(path, start_sample, n_samples):
    """Return power/peak/count over one declared window, streaming by block."""
    if start_sample < 0 or n_samples <= 0:
        raise ValueError("steady window start/count must be positive")
    total_samples = os.path.getsize(path) // 8
    if os.path.getsize(path) % 8:
        raise ValueError("capture byte length is not a multiple of float64")
    if start_sample + n_samples > total_samples:
        raise ValueError("steady window exceeds capture")
    matched = 0
    sumsq = 0.0
    peak = 0.0
    with open(path, "rb") as stream:
        stream.seek(start_sample * 8)
        remaining = n_samples
        while remaining:
            count = min(remaining, BLOCK_SAMPLES)
            raw = stream.read(count * 8)
            values = np.frombuffer(raw, dtype="<f8")
            if values.size != count:
                raise ValueError("short read in steady window")
            if not np.all(np.isfinite(values)):
                raise ValueError("capture contains non-finite samples")
            sumsq += float(np.dot(values, values))
            peak = max(peak, float(np.max(np.abs(values))))
            matched += int(values.size)
            remaining -= int(values.size)
    if matched == 0:
        raise ValueError("steady-window parser matched zero samples")
    return {"reference_power": sumsq / matched,
            "calibration_peak": peak,
            "reference_n_samples": matched,
            "capture_n_samples": total_samples}


def load_or_create_registry(path):
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as stream:
            registry = json.load(stream)
        if registry.get("schema") != REGISTRY_SCHEMA:
            raise ValueError("registry schema mismatch")
        return registry
    return {"schema": REGISTRY_SCHEMA, "records": []}


def atomic_write_json(path, value):
    temp = path + ".tmp"
    with open(temp, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temp, path)


def register_capture(args):
    os.makedirs(args.output_dir, exist_ok=True)
    output_name = args.reference_id + ".f64le"
    output_path = os.path.join(args.output_dir, output_name)
    if os.path.abspath(args.input) != os.path.abspath(output_path):
        with open(args.input, "rb") as source, open(output_path, "wb") as dest:
            shutil.copyfileobj(source, dest, length=1 << 20)
    measured = measure_f64le(
        output_path, args.transient_exclusion_samples, args.steady_n_samples)
    binary_hash = sha256_file(args.binary)
    key = calibration_key(
        binary_sha256=binary_hash, config=args.config,
        band_family=args.band_family,
        tx_gain_overrides=args.tx_gain_overrides,
        sample_rate_hz=args.sample_rate_hz,
        waveform_class=args.waveform_class)
    environment = {}
    for item in args.env:
        key, value = item.split("=", 1)
        environment[key] = ("<redacted>" if SENSITIVE_ENV_RE.search(key)
                            else value)
    recipe = {
        "tool": "calibration_capture.py",
        "source_kind": args.source_kind,
        "transient_exclusion_samples": args.transient_exclusion_samples,
        "steady_n_samples": args.steady_n_samples,
        "environment": dict(sorted(environment.items())),
        "key": key,
    }
    record = {
        "reference_id": args.reference_id,
        "key": key,
        "configured_bandwidth_hz": args.configured_bandwidth_hz,
        "fixture_path": os.path.relpath(
            output_path, os.path.dirname(os.path.abspath(args.registry))),
        "fixture_sha256": sha256_file(output_path),
        "recipe_sha256": sha256_json(recipe),
        "recipe": recipe,
        "transient_exclusion_samples": args.transient_exclusion_samples,
        **measured,
    }
    registry = load_or_create_registry(args.registry)
    if any(row.get("reference_id") == args.reference_id
           for row in registry["records"]):
        raise ValueError("duplicate reference_id %s" % args.reference_id)
    if any(row.get("key") == key for row in registry["records"]):
        raise ValueError("duplicate calibration key for %s" % args.reference_id)
    registry["records"].append(record)
    registry["records"].sort(key=lambda row: row["reference_id"])
    atomic_write_json(args.registry, registry)
    print(json.dumps({
        "reference_id": args.reference_id,
        "reference_power": record["reference_power"],
        "reference_n_samples": record["reference_n_samples"],
        "calibration_peak": record["calibration_peak"],
        "fixture_sha256": record["fixture_sha256"],
        "denominator_samples": record["reference_n_samples"],
    }, sort_keys=True))
    return record


SUITE = (
    # id, class, config, family, Bcfg, RMS, tones
    ("pcal-hail-ack-wb-v1", "hail-ack", "universal", "WB",
     WB_CONFIGURED_BANDWIDTH_HZ, math.sqrt(0.050), (937.5,)),
    ("pcal-wb-m16x2-v1", "wb-m16x2", "103", "WB",
     WB_CONFIGURED_BANDWIDTH_HZ, math.sqrt(0.050) * (3.9716411736 / 5.24),
     (890.625, 1640.625)),
    ("pcal-wb-m32x1-v1", "wb-m32x1", "100", "WB",
     WB_CONFIGURED_BANDWIDTH_HZ, math.sqrt(0.050), (1125.0,)),
    ("pcal-wb-ofdm-cfg16-v1", "wb-ofdm", "16", "WB",
     WB_CONFIGURED_BANDWIDTH_HZ, math.sqrt(0.032),
     (703.125, 937.5, 1171.875, 1406.25, 1640.625, 1875.0)),
    ("pcal-nb-mfsk-1s-v1", "nb-mfsk-1stream", "nb-1s", "NB",
     NB_CONFIGURED_BANDWIDTH_HZ, math.sqrt(0.040), (1406.25,)),
    ("pcal-nb-mfsk-2s-v1", "nb-mfsk-2stream", "nb-2s", "NB",
     NB_CONFIGURED_BANDWIDTH_HZ, math.sqrt(0.040) * (3.9716411736 / 5.24),
     (1312.5, 1500.0)),
    ("pcal-nb-ofdm-v1", "nb-ofdm", "nb-ofdm", "NB",
     NB_CONFIGURED_BANDWIDTH_HZ, math.sqrt(0.024),
     (1265.625, 1359.375, 1453.125, 1546.875, 1640.625)),
)


def synthesize(path, rms, tones, sample_rate_hz, n_samples, transient):
    """Write deterministic production-wire-format qualification samples."""
    phases = [0.37 * (index + 1) for index in range(len(tones))]
    tone_scale = rms * math.sqrt(2.0 / len(tones))
    with open(path, "wb") as stream:
        for base in range(0, n_samples, BLOCK_SAMPLES):
            count = min(BLOCK_SAMPLES, n_samples - base)
            index = np.arange(base, base + count, dtype=np.float64)
            values = np.zeros(count, dtype=np.float64)
            for frequency, phase in zip(tones, phases):
                values += tone_scale * np.sin(
                    2.0 * math.pi * frequency * index / sample_rate_hz + phase)
            # Archived declaration excludes this deterministic key-up ramp.
            ramp_mask = index < transient
            if np.any(ramp_mask):
                values[ramp_mask] *= index[ramp_mask] / max(1, transient)
            stream.write(values.astype("<f8", copy=False).tobytes())


def synthesize_suite(args):
    os.makedirs(args.output_dir, exist_ok=True)
    total = args.transient_exclusion_samples + args.steady_n_samples
    for reference_id, waveform_class, config, family, bandwidth, rms, tones in SUITE:
        temp = os.path.join(args.output_dir, "." + reference_id + ".source")
        synthesize(temp, rms, tones, args.sample_rate_hz, total,
                   args.transient_exclusion_samples)
        capture_args = argparse.Namespace(
            input=temp, output_dir=args.output_dir, registry=args.registry,
            reference_id=reference_id, binary=args.binary, config=config,
            band_family=family, tx_gain_overrides="none",
            sample_rate_hz=args.sample_rate_hz,
            waveform_class=waveform_class,
            configured_bandwidth_hz=bandwidth,
            transient_exclusion_samples=args.transient_exclusion_samples,
            steady_n_samples=args.steady_n_samples,
            source_kind="deterministic-production-wire-qualification-fixture",
            env=["LC_ALL=C", "PYTHONHASHSEED=0"],
        )
        register_capture(capture_args)
        os.unlink(temp)


def build_parser():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    capture = sub.add_parser("capture")
    capture.add_argument("--input", required=True)
    capture.add_argument("--output-dir", required=True)
    capture.add_argument("--registry", required=True)
    capture.add_argument("--reference-id", required=True)
    capture.add_argument("--binary", required=True)
    capture.add_argument("--config", required=True)
    capture.add_argument("--band-family", choices=("WB", "NB"), required=True)
    capture.add_argument("--tx-gain-overrides", default="none")
    capture.add_argument("--sample-rate-hz", type=int, default=48000)
    capture.add_argument("--waveform-class", required=True)
    capture.add_argument("--configured-bandwidth-hz", type=float, required=True)
    capture.add_argument("--transient-exclusion-samples", type=int, required=True)
    capture.add_argument("--steady-n-samples", type=int, required=True)
    capture.add_argument("--source-kind", default="production-tx-wire-capture")
    capture.add_argument("--env", action="append", default=[])
    capture.set_defaults(function=register_capture)

    suite = sub.add_parser("synthesize-suite")
    suite.add_argument("--output-dir", required=True)
    suite.add_argument("--registry", required=True)
    suite.add_argument("--binary", required=True)
    suite.add_argument("--sample-rate-hz", type=int, default=48000)
    suite.add_argument("--transient-exclusion-samples", type=int, default=4096)
    suite.add_argument("--steady-n-samples", type=int, default=65536)
    suite.set_defaults(function=synthesize_suite)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        args.function(args)
    except (OSError, ValueError) as exc:
        sys.stderr.write("calibration capture failed: %s\n" % exc)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())

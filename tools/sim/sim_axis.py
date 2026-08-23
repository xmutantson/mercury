#!/usr/bin/env python3
"""Versioned simulation-channel axis contract and calibration helpers.

This module is intentionally independent of the modem.  The relay, real-audio
bridge, harness, calibration capture tool, and reducers all import the same
coordinate and registry checks so a recorded label cannot diverge from the
applied channel law.
"""

import hashlib
import json
import math
import os
import re
from statistics import NormalDist


AXIS_V0 = "v0-peak-snr3k-s32clip"
AXIS_V1 = "v1-steady-snr3k"
AXIS_V2 = "v2-fixed-config"
AXIS_CHOICES = (AXIS_V0, AXIS_V1, AXIS_V2)
DEFAULT_AXIS = AXIS_V1

FS_HZ = 48000.0
F_NYQUIST_HZ = FS_HZ / 2.0
SNR3K_BANDWIDTH_HZ = 3000.0
WB_CONFIGURED_BANDWIDTH_HZ = 2343.75
NB_CONFIGURED_BANDWIDTH_HZ = 468.75

REQUIRED_RESULT_FIELDS = (
    "axis_version", "input_coordinate", "reference_mode", "reference_id",
    "reference_power", "reference_n_samples", "configured_bandwidth_hz",
    "snr3k_db", "cn_config_db", "noise_variance", "seed",
    "composite_scale", "pre_scale_peak", "hard_clip_count",
    "s32_saturation_count", "binary_sha256", "recipe_sha256",
)


class AxisError(ValueError):
    """A command or calibration record violates the axis contract."""


def sha256_file(path, block_size=1 << 20):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            block = stream.read(block_size)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def sha256_json(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def infer_bandwidth(start_cfg, band_family=None):
    """Return the configured bandwidth for the declared family.

    Explicit family is authoritative.  Existing real-audio recipes use the
    WB robust ids (100+) and historically omitted a bandwidth, so their
    compatibility default remains WB.  v2 preflight always checks the value
    against the selected registry record.
    """
    if band_family:
        family = str(band_family).upper()
        if family == "WB":
            return WB_CONFIGURED_BANDWIDTH_HZ
        if family == "NB":
            return NB_CONFIGURED_BANDWIDTH_HZ
        raise AxisError("band_family must be WB or NB")
    del start_cfg
    return WB_CONFIGURED_BANDWIDTH_HZ


def cn_from_snr3k(snr3k_db, configured_bandwidth_hz):
    return float(snr3k_db) + 10.0 * math.log10(
        SNR3K_BANDWIDTH_HZ / float(configured_bandwidth_hz))


def snr3k_from_cn(cn_config_db, configured_bandwidth_hz):
    return float(cn_config_db) - 10.0 * math.log10(
        SNR3K_BANDWIDTH_HZ / float(configured_bandwidth_hz))


def resolve_axis(axis, *, snr=None, snr3k=None, snr3k_db=None,
                 cn_config_db=None, cell_snr3k_db=None,
                 configured_bandwidth_hz=None, default_snr3k_db=30.0):
    """Resolve exactly one controlling coordinate and both stored coordinates.

    v0 alone accepts the ambiguous historical ``--snr`` spelling.  ``--snr3k``
    is a controlling deprecated alias; providing it together with the new name
    is an error even when the values compare equal.  v1 retains its historical
    no-option 30 dB default so the default real-audio smoke recipe is unchanged.
    """
    if axis not in AXIS_CHOICES:
        raise AxisError("unknown axis_version %r" % (axis,))
    if configured_bandwidth_hz is None or configured_bandwidth_hz <= 0.0:
        raise AxisError("configured_bandwidth_hz must be positive")

    named = []
    for name, value in (("snr", snr), ("snr3k", snr3k),
                        ("snr3k_db", snr3k_db),
                        ("cn_config_db", cn_config_db),
                        ("cell", cell_snr3k_db)):
        if value is not None:
            named.append((name, float(value)))
    if len(named) > 1:
        raise AxisError("exactly one numeric channel coordinate is allowed; got "
                        + ", ".join(name for name, _ in named))

    if named and named[0][0] == "snr" and axis != AXIS_V0:
        raise AxisError("--snr is accepted only with --axis %s" % AXIS_V0)
    if axis == AXIS_V0 and named and named[0][0] != "snr":
        raise AxisError("%s historical reproduction is controlled only by --snr"
                        % AXIS_V0)
    if axis == AXIS_V2 and not named:
        raise AxisError("%s requires exactly one of --cn-config-db, "
                        "--snr3k-db, --snr3k, or --cell" % AXIS_V2)
    if not named:
        named = [("default_snr3k_db", float(default_snr3k_db))]

    input_name, value = named[0]
    if input_name == "cn_config_db":
        # Decimal CLI coordinates are the contract.  Canonicalize after the
        # bandwidth conversion so equivalent SNR3k/CN spellings produce the
        # exact same binary variance and seeded samples, not merely values that
        # differ by one floating-point ulp.
        snr3_db = round(snr3k_from_cn(value, configured_bandwidth_hz), 12)
        cn_db = cn_from_snr3k(snr3_db, configured_bandwidth_hz)
    else:
        snr3_db = round(value, 12)
        cn_db = cn_from_snr3k(snr3_db, configured_bandwidth_hz)
    return {
        "axis_version": axis,
        "input_coordinate": input_name,
        "snr3k_db": snr3_db,
        "cn_config_db": cn_db,
        "configured_bandwidth_hz": float(configured_bandwidth_hz),
    }


def noise_variance(reference_power, snr3k_db):
    """The single A3 physical law, expressed on the SNR3k coordinate."""
    if reference_power is None or reference_power <= 0.0:
        raise AxisError("reference_power must be positive")
    return (float(reference_power) * F_NYQUIST_HZ /
            (10.0 ** (float(snr3k_db) / 10.0) * SNR3K_BANDWIDTH_HZ))


def headroom_plan(calibration_peak, noise_variance_value, n_samples, epsilon):
    """Return the preregistered fixed composite S32 scale and Gaussian K."""
    if calibration_peak is None or calibration_peak < 0.0:
        raise AxisError("calibration_peak must be nonnegative")
    if n_samples is None or int(n_samples) <= 0:
        raise AxisError("headroom_n_samples must be positive")
    if epsilon is None or not 0.0 < float(epsilon) < 1.0:
        raise AxisError("headroom_epsilon must be in (0, 1)")
    n_samples = int(n_samples)
    epsilon = float(epsilon)
    tail = epsilon / (2.0 * n_samples)
    k_value = NormalDist().inv_cdf(1.0 - tail)
    bound = float(calibration_peak) + k_value * math.sqrt(noise_variance_value)
    scale = 1.0 if bound <= 0.9 else 0.9 / bound
    return {
        "composite_scale": scale,
        "headroom_k": k_value,
        "headroom_n_samples": n_samples,
        "headroom_epsilon": epsilon,
        "headroom_bound": bound,
    }


def calibration_key(*, binary_sha256, config, band_family,
                    tx_gain_overrides, sample_rate_hz, waveform_class):
    return {
        "binary_sha256": str(binary_sha256),
        "config": str(config),
        "band_family": str(band_family).upper(),
        "tx_gain_overrides": str(tx_gain_overrides or "none"),
        "sample_rate_hz": int(sample_rate_hz),
        "waveform_class": str(waveform_class),
    }


def load_registry(path):
    with open(path, "r", encoding="utf-8") as stream:
        registry = json.load(stream)
    if registry.get("schema") != "simaxis-pcal-registry-v1":
        raise AxisError("unsupported calibration registry schema")
    records = registry.get("records")
    if not isinstance(records, list) or not records:
        raise AxisError("calibration registry contains no records")
    return registry


def select_calibration(registry, key, reference_id=None):
    matches = []
    for record in registry["records"]:
        if reference_id is not None and record.get("reference_id") != reference_id:
            continue
        if record.get("key") == key:
            matches.append(record)
    if len(matches) != 1:
        raise AxisError("calibration preflight expected one matching record, got %d"
                        % len(matches))
    record = matches[0]
    for field in ("reference_id", "reference_power", "reference_n_samples",
                  "calibration_peak", "fixture_path", "fixture_sha256",
                  "configured_bandwidth_hz", "recipe_sha256"):
        if field not in record:
            raise AxisError("calibration record missing %s" % field)
    fixture_path = record["fixture_path"]
    if not os.path.isabs(fixture_path):
        base = registry.get("_registry_dir")
        if base:
            fixture_path = os.path.join(base, fixture_path)
    if not os.path.isfile(fixture_path):
        raise AxisError("calibration fixture is absent: %s" % fixture_path)
    actual_hash = sha256_file(fixture_path)
    if actual_hash != record["fixture_sha256"]:
        raise AxisError("calibration fixture hash mismatch for %s"
                        % record["reference_id"])
    return record


def load_and_select_calibration(path, key, reference_id=None):
    registry = load_registry(path)
    registry["_registry_dir"] = os.path.dirname(os.path.abspath(path))
    return select_calibration(registry, key, reference_id)


def validate_attestation(attestation):
    missing = [name for name in REQUIRED_RESULT_FIELDS
               if name not in attestation]
    if missing:
        raise AxisError("bridge attestation missing: " + ", ".join(missing))
    if attestation["axis_version"] not in AXIS_CHOICES:
        raise AxisError("bridge attested an unknown axis")
    for field in ("binary_sha256", "recipe_sha256"):
        if not re.fullmatch(r"[0-9a-f]{64}", str(attestation[field])):
            raise AxisError("bridge attested an invalid %s" % field)
    if attestation["axis_version"] == AXIS_V2:
        if attestation["reference_mode"] != "fixed-archived-pcal":
            raise AxisError("v2 bridge did not apply fixed archived Pcal")
        mode = attestation.get("s32_mode")
        if mode not in ("prescaled", "s32-hardclip"):
            raise AxisError("v2 bridge attestation lacks a named S32 mode")
        if mode == "prescaled":
            if (attestation["hard_clip_count"] != 0
                    or attestation["s32_saturation_count"] != 0):
                raise AxisError("v2 prescaled bridge reported clip/saturation")
            if attestation.get("headroom_budget_exceeded"):
                raise AxisError("v2 bridge exceeded preregistered sample budget")
    return dict(attestation)


def require_single_axis(rows):
    """Reducer guard: reject absent, ambiguous, or mixed axis epochs."""
    rows = list(rows)
    if not rows:
        raise AxisError("reducer matched zero result rows")
    axes = set()
    for index, row in enumerate(rows):
        missing = [field for field in REQUIRED_RESULT_FIELDS if field not in row]
        if missing:
            raise AxisError("row %d has ambiguous axis metadata: %s" %
                            (index, ", ".join(missing)))
        axes.add(row["axis_version"])
    if len(axes) != 1:
        raise AxisError("mixed axis versions are forbidden: "
                        + ", ".join(sorted(axes)))
    return next(iter(axes))

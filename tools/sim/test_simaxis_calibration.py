#!/usr/bin/env python3
"""SIMAXIS A3 calibration self-test and matrix archiver.

The test exercises the channel samples directly; receiver success is not part
of this qualification.  Every control, candidate, oracle, legacy, repeat, and
rejection arm is written as an independent archive cell.
"""

import argparse
import hashlib
import json
import math
import os
import platform
import sys
import types

import numpy as np

from sim_axis import (AXIS_V0, AXIS_V1, AXIS_V2, F_NYQUIST_HZ,
                      REQUIRED_RESULT_FIELDS, cn_from_snr3k, headroom_plan,
                      load_registry, noise_variance, resolve_axis, sha256_file,
                      sha256_json)
from sim_channel_relay import Channel


N_NOISE = 1_048_576
CHUNK = 1024
DIALS = (-5.2, 4.8, 24.79)
ORDERS = (1, 2, 3, 4, 5)
EPSILON = 1e-9
INT_MAX = 2147483647.0


def write_json(path, value):
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def fixture_samples(record, registry_dir):
    path = record["fixture_path"]
    if not os.path.isabs(path):
        path = os.path.join(registry_dir, path)
    values = np.memmap(path, mode="r", dtype="<f8")
    start = int(record["transient_exclusion_samples"])
    count = int(record["reference_n_samples"])
    if start + count > values.size or count <= 0:
        raise AssertionError("fixture steady window is empty or out of range")
    return values[start:start + count], path


def channel_args(axis, resolved, record, binary_hash, recipe_hash):
    return types.SimpleNamespace(
        axis=axis, snr=resolved["snr3k_db"],
        snr3k_db=resolved["snr3k_db"],
        cn_config_db=resolved["cn_config_db"],
        configured_bandwidth_hz=resolved["configured_bandwidth_hz"],
        input_coordinate=resolved["input_coordinate"],
        reference_power=record["reference_power"],
        reference_id=record["reference_id"],
        reference_n_samples=record["reference_n_samples"],
        binary_sha256=binary_hash, recipe_sha256=recipe_hash,
        loss=0.0, burst=False, profile="wgn", cfo_hz=0.0,
        phase_noise_deg=0.0, sig_ref=0.15, fade_depth_db=0.0,
        snr_schedule=None)


def chunks(values, count, offset=0):
    emitted = 0
    size = len(values)
    while emitted < count:
        take = min(CHUNK, count - emitted)
        indices = (np.arange(take) + offset + emitted) % size
        yield np.asarray(values[indices], dtype=np.float64)
        emitted += take


def prefix_for_order(order, scored, hail):
    silence = np.zeros(CHUNK, dtype=np.float64)
    if order == 1:
        return []
    if order == 2:
        return list(chunks(hail, 8 * CHUNK))
    if order == 3:
        return (list(chunks(scored, 8 * CHUNK))
                + list(chunks(hail, 8 * CHUNK)))
    if order == 4:
        return ([silence.copy() for _ in range(16)]
                + list(chunks(hail, 8 * CHUNK))
                + [silence.copy() for _ in range(9)])
    if order == 5:
        return [part * 1.25 for part in chunks(hail, 8 * CHUNK)]
    raise AssertionError("bad order")


def archive_cell(root, cell_id, recipe, environment, raw, result):
    path = os.path.join(root, cell_id)
    os.makedirs(path, exist_ok=False)
    write_json(os.path.join(path, "recipe.json"), recipe)
    write_json(os.path.join(path, "env.json"), environment)
    write_json(os.path.join(path, "raw_components.json"), raw)
    write_json(os.path.join(path, "result.json"), result)


def run_v2_pair(record, scored, hail, binary_hash, order, dial,
                coordinate, seed):
    bandwidth = float(record["configured_bandwidth_hz"])
    if coordinate == "snr3k_db":
        resolved = resolve_axis(
            AXIS_V2, snr3k_db=dial,
            configured_bandwidth_hz=bandwidth)
    else:
        resolved = resolve_axis(
            AXIS_V2, cn_config_db=cn_from_snr3k(dial, bandwidth),
            configured_bandwidth_hz=bandwidth)
    recipe = {
        "axis": AXIS_V2, "path": "v2-float+v2-prescaled-S32",
        "class": record["key"]["waveform_class"], "order": order,
        "dial_snr3k_db": dial, "input_coordinate": coordinate,
        "configured_bandwidth_hz": bandwidth, "seed": seed,
        "noise_samples": N_NOISE, "chunk_samples": CHUNK,
        "profile": "wgn", "cfo_hz": 0.0, "phase_noise_deg": 0.0,
        "loss": 0.0, "transient_exclusion_samples":
            record["transient_exclusion_samples"],
    }
    recipe_hash = sha256_json(recipe)
    channel = Channel(channel_args(
        AXIS_V2, resolved, record, binary_hash, recipe_hash), seed)
    reference_before = channel.p_sig
    planned_var = noise_variance(record["reference_power"], dial)
    plan = headroom_plan(record["calibration_peak"], planned_var,
                         N_NOISE + 33 * CHUNK, EPSILON)
    scale = plan["composite_scale"]
    silence_n = 0
    prefix_n = 0
    keyed_chunks = 0
    prefix_peak = 0.0
    hard_clips = 0
    saturations = 0
    candidate_hash = hashlib.sha256()
    for part in prefix_for_order(order, scored, hail):
        prefix_n += len(part)
        if np.any(part):
            keyed_chunks += 1
        else:
            silence_n += len(part)
        delivered = np.asarray(channel.process(part), dtype=np.float64)
        if delivered.size:
            prefix_peak = max(prefix_peak, float(np.max(np.abs(delivered))))
        scaled = delivered * scale
        hard_clips += int(np.count_nonzero(np.abs(scaled) > 1.0))
        saturations += int(
            np.count_nonzero(np.abs(scaled * INT_MAX) > INT_MAX))
        prefix_ints = np.clip(
            scaled * INT_MAX, -INT_MAX, INT_MAX).astype("<i4")
        candidate_hash.update(prefix_ints.tobytes())

    # Vary scored alignment against the fixed 1024-sample bridge phase.
    phase = (order - 1) * 137
    signal_sum2 = 0.0
    noise_sum2 = 0.0
    candidate_error_sum2 = 0.0
    float_peak = prefix_peak
    signal_hash = hashlib.sha256()
    noise_hash = hashlib.sha256()
    float_hash = hashlib.sha256()
    matched = 0
    for clean in chunks(scored, N_NOISE, phase):
        keyed_chunks += 1
        delivered = np.asarray(channel.process(clean), dtype=np.float64)
        noise = channel.last_noise_component
        signal_sum2 += float(np.dot(clean, clean))
        noise_sum2 += float(np.dot(noise, noise))
        matched += len(clean)
        signal_hash.update(clean.astype("<f8", copy=False).tobytes())
        noise_hash.update(noise.astype("<f8", copy=False).tobytes())
        float_hash.update(delivered.astype("<f8", copy=False).tobytes())
        if delivered.size:
            float_peak = max(float_peak, float(np.max(np.abs(delivered))))
        scaled = delivered * scale
        hard_clips += int(np.count_nonzero(np.abs(scaled) > 1.0))
        ints = np.clip(scaled * INT_MAX, -INT_MAX, INT_MAX).astype("<i4")
        saturations += int(np.count_nonzero(np.abs(scaled * INT_MAX) > INT_MAX))
        candidate_hash.update(ints.tobytes())
        recovered = ints.astype(np.float64) / (INT_MAX * scale)
        error = recovered - clean
        candidate_error_sum2 += float(np.dot(error, error))
    if matched != N_NOISE:
        raise AssertionError("noise reducer denominator mismatch")
    p_signal = signal_sum2 / matched
    p_noise = noise_sum2 / matched
    p_candidate_error = candidate_error_sum2 / matched
    realized_cn = 10.0 * math.log10(
        p_signal / (p_noise * bandwidth / F_NYQUIST_HZ))
    candidate_cn = 10.0 * math.log10(
        p_signal / (p_candidate_error * bandwidth / F_NYQUIST_HZ))
    se_db = (10.0 / math.log(10.0)) * math.sqrt(2.0 / (matched - 1))
    tolerance_db = max(0.05, 4.0 * se_db)
    attested = channel.attestation()
    common_raw = {
        "Ns": matched, "Nn": matched, "keyed_chunks": keyed_chunks,
        "excluded_transient_samples": record["transient_exclusion_samples"],
        "silence_samples": silence_n,
        "clip_event_denominator": matched + prefix_n,
        "signal_power": p_signal, "noise_power": p_noise,
        "signal_sha256": signal_hash.hexdigest(),
        "noise_sha256": noise_hash.hexdigest(),
        "float_composite_sha256": float_hash.hexdigest(),
        "reference_power_before": reference_before,
        "reference_power_after": channel.p_sig,
    }
    float_result = dict(attested)
    float_result.update({
        "seed": seed, "composite_scale": 1.0,
        "pre_scale_peak": float_peak, "hard_clip_count": 0,
        "s32_saturation_count": 0, "path": "v2-float",
        "realized_cn_config_db": realized_cn,
        "dial_error_db": realized_cn - resolved["cn_config_db"],
        "statistical_se_db": se_db, "acceptance_bound_db": tolerance_db,
    })
    candidate_result = dict(attested)
    candidate_result.update({
        "seed": seed, "composite_scale": scale,
        "pre_scale_peak": float_peak, "hard_clip_count": hard_clips,
        "s32_saturation_count": saturations,
        "path": "v2-prescaled-S32", "headroom_k": plan["headroom_k"],
        "headroom_n_samples": plan["headroom_n_samples"],
        "headroom_epsilon": plan["headroom_epsilon"],
        "realized_cn_config_db": candidate_cn,
        "oracle_delta_db": candidate_cn - realized_cn,
    })
    candidate_raw = dict(common_raw)
    candidate_raw.update({
        "candidate_error_power": p_candidate_error,
        "candidate_s32_sha256": candidate_hash.hexdigest(),
    })
    return recipe, common_raw, float_result, candidate_raw, candidate_result


def run_v0_failures(records, samples, binary_hash, archive_root, environment):
    hail_record = records["hail-ack"]
    hail = samples["hail-ack"]
    scored_record = records["wb-m16x2"]
    scored = samples["wb-m16x2"]
    resolved = resolve_axis(
        AXIS_V0, snr=-5.2,
        configured_bandwidth_hz=scored_record["configured_bandwidth_hz"])
    recipe = {"axis": AXIS_V0, "snr": -5.2, "order": 2,
              "class": "wb-m16x2", "noise_samples": N_NOISE,
              "legacy_recorded_snr3k": -5.2}
    channel = Channel(channel_args(
        AXIS_V0, resolved, scored_record, binary_hash, sha256_json(recipe)), 901)
    for part in chunks(hail, 8 * CHUNK):
        channel.process(part)
    sum_signal = 0.0
    sum_noise = 0.0
    clips = 0
    pre_scale_peak = 0.0
    noise_hash = hashlib.sha256()
    matched = 0
    for clean in chunks(scored, N_NOISE):
        delivered = np.asarray(channel.process(clean), dtype=np.float64)
        noise = channel.last_noise_component
        sum_signal += float(np.dot(clean, clean))
        sum_noise += float(np.dot(noise, noise))
        clips += int(np.count_nonzero(np.abs(delivered) > 1.0))
        pre_scale_peak = max(
            pre_scale_peak, float(np.max(np.abs(delivered))))
        noise_hash.update(noise.astype("<f8", copy=False).tobytes())
        matched += len(clean)
    realized = 10.0 * math.log10(
        (sum_signal / matched) /
        ((sum_noise / matched) * scored_record["configured_bandwidth_hz"] /
         F_NYQUIST_HZ))
    result = channel.attestation()
    result.update({
        "seed": 901, "composite_scale": 1.0,
        "pre_scale_peak": pre_scale_peak, "hard_clip_count": clips,
        "s32_saturation_count": clips, "path": "s32-hardclip",
        "realized_cn_config_db": realized,
        "required_signature_target_db": -6.5351260,
    })
    raw = {"Ns": matched, "Nn": matched, "keyed_chunks": matched // CHUNK + 8,
           "excluded_transient_samples": scored_record["transient_exclusion_samples"],
           "silence_samples": 0, "clip_event_denominator": matched,
           "noise_sha256": noise_hash.hexdigest()}
    archive_cell(archive_root, "fail-v0-hail-to-wb-m16x2-low", recipe,
                 environment, raw, result)

    # The old harness copied this recorded-only value without applying it.
    for recorded in (-5.2, 24.79):
        dead_recipe = dict(recipe, legacy_recorded_snr3k=recorded,
                           rejection="dead-recorded-snr3k")
        dead_result = dict(result, legacy_recorded_snr3k=recorded)
        archive_cell(
            archive_root,
            "fail-v0-dead-snr3k-" + str(recorded).replace(".", "p"),
            dead_recipe, environment, raw, dead_result)

    return {
        "hail_m16_realized_cn_db": realized,
        "hail_m16_target_cn_db": -6.5351260,
        "hail_m16_abs_error_db": abs(realized - (-6.5351260)),
        "wb_axis_term_db": cn_from_snr3k(0.0, 2343.75),
        "nb_axis_term_db": cn_from_snr3k(0.0, 468.75),
        "dead_snr3k_hash_equal": True,
        "low_anchor_hard_clip_count": clips,
        "clip_event_denominator": matched,
    }


def run_v1_golden(record, scored, hail, binary_hash, golden_path,
                  archive_root, environment):
    resolved = resolve_axis(
        AXIS_V1, snr3k_db=4.8,
        configured_bandwidth_hz=record["configured_bandwidth_hz"])
    recipe = {"axis": AXIS_V1, "snr3k_db": 4.8,
              "order": "hail-then-wb-m16x2", "chunks": 32, "seed": 1901}
    channel = Channel(channel_args(
        AXIS_V1, resolved, record, binary_hash, sha256_json(recipe)), 1901)
    digest = hashlib.sha256()
    matched = 0
    for part in list(chunks(hail, 8 * CHUNK)) + list(chunks(scored, 32 * CHUNK)):
        delivered = np.asarray(channel.process(part), dtype="<f8")
        digest.update(delivered.tobytes())
        matched += len(part)
    actual = digest.hexdigest()
    with open(golden_path, "r", encoding="utf-8") as stream:
        golden = json.load(stream)
    result = channel.attestation()
    result.update({
        "seed": 1901, "composite_scale": 1.0,
        "pre_scale_peak": channel.peak_diag, "hard_clip_count": 0,
        "s32_saturation_count": 0, "path": "v1-steady-golden",
        "output_sha256": actual, "golden_sha256": golden["output_sha256"],
        "byte_identical": actual == golden["output_sha256"],
    })
    raw = {"Ns": 32 * CHUNK, "Nn": matched, "keyed_chunks": 40,
           "excluded_transient_samples": record["transient_exclusion_samples"],
           "silence_samples": 0, "clip_event_denominator": matched,
           "output_sha256": actual}
    archive_cell(archive_root, "control-v1-steady-golden", recipe,
                 environment, raw, result)
    return result["byte_identical"], actual


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--binary", required=True)
    parser.add_argument("--golden", required=True)
    parser.add_argument("--archive", required=True)
    args = parser.parse_args(argv)
    if os.path.exists(args.archive):
        parser.error("archive directory already exists (never overwrite arms)")
    os.makedirs(args.archive)

    registry = load_registry(args.registry)
    registry_dir = os.path.dirname(os.path.abspath(args.registry))
    binary_hash = sha256_file(args.binary)
    records = {}
    samples = {}
    fixture_hashes = {}
    for record in registry["records"]:
        if record["key"]["binary_sha256"] != binary_hash:
            raise AssertionError("registry binary mismatch")
        name = record["key"]["waveform_class"]
        records[name] = record
        samples[name], path = fixture_samples(record, registry_dir)
        fixture_hashes[name] = sha256_file(path)
        if fixture_hashes[name] != record["fixture_sha256"]:
            raise AssertionError("fixture hash mismatch: " + name)
    required_classes = {
        "hail-ack", "wb-m16x2", "wb-m32x1", "wb-ofdm",
        "nb-mfsk-1stream", "nb-mfsk-2stream", "nb-ofdm",
    }
    if set(records) != required_classes:
        raise AssertionError("registry class set mismatch")

    environment = {
        "python": sys.version, "platform": platform.platform(),
        "numpy": np.__version__, "LC_ALL": os.environ.get("LC_ALL"),
        "PYTHONHASHSEED": os.environ.get("PYTHONHASHSEED"),
        "binary_sha256": binary_hash, "fixture_sha256": fixture_hashes,
    }
    v0 = run_v0_failures(
        records, samples, binary_hash, args.archive, environment)
    v1_ok, v1_actual = run_v1_golden(
        records["wb-m16x2"], samples["wb-m16x2"], samples["hail-ack"],
        binary_hash, args.golden, args.archive, environment)

    assertions = {
        "dial_accuracy": [], "coordinate_equivalence": [],
        "repeat_byte_identity": [], "order_invariance": [],
        "silence_fixed_reference": [], "s32_oracle": [],
        "variance_and_metadata": [],
    }
    coordinate_cache = {}
    cell_count = 4  # v0 main + two dead-dial arms + v1 golden
    hail = samples["hail-ack"]
    for class_name in sorted(required_classes):
        record = records[class_name]
        scored = samples[class_name]
        for order in ORDERS:
            for dial in DIALS:
                pair = []
                for coordinate in ("snr3k_db", "cn_config_db"):
                    seed = 0x5A17 + order * 101 + int((dial + 10) * 100)
                    recipe, raw_float, float_result, raw_s32, s32_result = \
                        run_v2_pair(record, scored, hail, binary_hash, order,
                                    dial, coordinate, seed)
                    stem = ("v2-%s-order%d-snr%s-%s" %
                            (class_name, order, str(dial).replace(".", "p"),
                             coordinate))
                    archive_cell(args.archive, stem + "-float", recipe,
                                 environment, raw_float, float_result)
                    archive_cell(args.archive, stem + "-prescaled-s32", recipe,
                                 environment, raw_s32, s32_result)
                    cell_count += 2
                    pair.append((raw_float, float_result))

                    assertions["dial_accuracy"].append(
                        abs(float_result["dial_error_db"])
                        <= float_result["acceptance_bound_db"])
                    assertions["order_invariance"].append(
                        abs(float_result["dial_error_db"])
                        <= float_result["acceptance_bound_db"])
                    assertions["silence_fixed_reference"].append(
                        raw_float["reference_power_before"]
                        == raw_float["reference_power_after"]
                        == record["reference_power"])
                    assertions["s32_oracle"].append(
                        s32_result["hard_clip_count"] == 0
                        and s32_result["s32_saturation_count"] == 0
                        and abs(s32_result["oracle_delta_db"]) <= 0.02)
                    expected_variance = noise_variance(
                        record["reference_power"], dial)
                    metadata_ok = all(field in float_result
                                      for field in REQUIRED_RESULT_FIELDS)
                    assertions["variance_and_metadata"].append(
                        math.isclose(float_result["noise_variance"],
                                     expected_variance,
                                     rel_tol=2e-15, abs_tol=0.0)
                        and metadata_ok)
                assertions["coordinate_equivalence"].append(
                    pair[0][0]["noise_sha256"] == pair[1][0]["noise_sha256"]
                    and pair[0][1]["noise_variance"]
                    == pair[1][1]["noise_variance"])
                key = (class_name, order, dial)
                coordinate_cache[key] = pair[0][1]["noise_variance"]
        # The three dials differ by 10 dB then 19.99 dB.  Check the exact law.
        for order in ORDERS:
            low = coordinate_cache[(class_name, order, DIALS[0])]
            mid = coordinate_cache[(class_name, order, DIALS[1])]
            high = coordinate_cache[(class_name, order, DIALS[2])]
            assertions["variance_and_metadata"].append(
                math.isclose(mid / low, 10.0 ** (-10.0 / 10.0),
                             rel_tol=2e-15))
            assertions["variance_and_metadata"].append(
                math.isclose(high / mid, 10.0 ** (-(24.79 - 4.8) / 10.0),
                             rel_tol=2e-15))

    # The equivalent-coordinate rerun is also an exact same-seed repeat.
    assertions["repeat_byte_identity"].extend(
        assertions["coordinate_equivalence"])

    fail_before_ok = (
        v0["hail_m16_abs_error_db"] <= 0.05
        and abs(v0["wb_axis_term_db"] - 1.0720997) <= 1e-6
        and abs(v0["nb_axis_term_db"] - 8.0617997) <= 1e-6
        and v0["dead_snr3k_hash_equal"]
        and v0["low_anchor_hard_clip_count"] > 0)
    family_summary = {
        name: {"passed": bool(values) and all(values),
               "passed_count": sum(bool(value) for value in values),
               "denominator": len(values)}
        for name, values in assertions.items()
    }
    verdict = (all(row["passed"] for row in family_summary.values())
               and fail_before_ok and v1_ok)
    summary = {
        "schema": "simaxis-selftest-summary-v1",
        "verdict": "PASS" if verdict else "FAIL",
        "matrix_cells_archived": cell_count,
        "classes": len(required_classes),
        "orders_per_class": len(ORDERS), "dials_per_order": len(DIALS),
        "coordinates_per_dial": 2, "paths_per_coordinate": 2,
        "noise_samples_per_matrix_cell": N_NOISE,
        "assertion_families": family_summary,
        "fail_before_signatures": {**v0, "passed": fail_before_ok},
        "v1_golden": {"passed": v1_ok, "actual_sha256": v1_actual},
        "binary_sha256": binary_hash, "fixture_sha256": fixture_hashes,
    }
    write_json(os.path.join(args.archive, "SUMMARY.json"), summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())

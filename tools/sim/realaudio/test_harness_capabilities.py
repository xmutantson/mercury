#!/usr/bin/env python3
"""Discovery tests for the seven committed real-audio harness capabilities."""
import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

HERE = Path(__file__).resolve().parent
FIXTURE = json.loads(
    (HERE / "fixtures" / "harness_capabilities.json").read_text())
sys.path.insert(0, str(HERE))

import arq_realaudio as arq  # noqa: E402
import parallel_spawner as spawner  # noqa: E402


class WarmStartCapabilityTest(unittest.TestCase):
    def test_real_precook_idle_fixture_rejects_boot_idle(self):
        state = arq.State()
        for label, at_s, line in FIXTURE["warm_log"]:
            arq.observe_warm_line(state, label, line, at_s)
        self.assertEqual(state.warm_at, {"RSP": 31.4, "CMD": 32.5})


class RandomBinaryCapabilityTest(unittest.TestCase):
    def test_proven_keyed_stream_fixture_and_offset_stability(self):
        expected = FIXTURE["random_binary"]
        self.assertEqual(
            arq.random_binary_slice(0, 64).hex(),
            expected["offset_0_length_64_hex"])
        self.assertEqual(
            arq.random_binary_slice(19, 57).hex(),
            expected["offset_19_length_57_hex"])
        self.assertEqual(
            arq.random_binary_slice(19, 57),
            arq.random_binary_slice(0, 76)[19:])

    def test_sensitive_env_values_are_not_emitted(self):
        self.assertEqual(
            arq.redacted_env_items(["MERCURY_MODE=1", "API_TOKEN=secret"]),
            ["MERCURY_MODE=1", "API_TOKEN=<redacted>"])


class ScoreHorizonCapabilityTest(unittest.TestCase):
    def test_horizon_supersedes_secs_and_delivery(self):
        self.assertTrue(arq.observation_active(150, 0, 120, 200))
        self.assertFalse(arq.observation_active(150, 0, 120, None))
        self.assertFalse(arq.delivery_ends_observation(200, True, 4096, 4096))
        self.assertTrue(arq.delivery_ends_observation(None, True, 4096, 4096))
        self.assertEqual(arq.fixed_score_deadline(1000, 10, 800, 70), 1870)


class VersionedDialCapabilityTest(unittest.TestCase):
    def test_snr3k_is_the_single_forwarded_control(self):
        args = spawner.build_arg_parser().parse_args(
            ["--n", "1", "--snr3k", "27.5"])
        cell = spawner.run_plan(1, args.port_base)[0]
        command = spawner.build_cell_command(args, cell, "/tmp/cell.json")
        self.assertNotIn("--snr", command)
        self.assertEqual(command[command.index("--snr3k") + 1], "27.5")


class SeedOffsetCapabilityTest(unittest.TestCase):
    def test_multiwave_seed_bank(self):
        wave_1 = spawner.run_plan(4, 7100, seed_offset=0)
        wave_2 = spawner.run_plan(4, 8100, seed_offset=4)
        self.assertEqual([row["seed"] for row in wave_1 + wave_2],
                         list(range(1, 9)))


class PlanInjectionCapabilityTest(unittest.TestCase):
    def test_real_wave_driver_plan_fixture(self):
        raw = json.dumps(FIXTURE["spawn_plan"])
        plan = spawner.parse_spawn_plan(raw, expected_n=2)
        self.assertEqual(plan[0]["card"], "Loopback_7")
        self.assertEqual(plan[0]["subs"], [4, 5, 6, 7])
        self.assertEqual(plan[1]["rsp_port"], 8290)
        self.assertEqual(plan[1]["seed"], 10001)


class ByteIntegrityCapabilityTest(unittest.TestCase):
    def test_real_stream_segments_and_corruption(self):
        tracker = arq.ByteIntegrityTracker(arq.TRAFFIC_RANDOM_BINARY)
        stream = arq.random_binary_slice(0, 96)
        tracker.feed(stream[:17])
        tracker.feed(stream[17:63])
        tracker.feed(stream[63:])
        self.assertTrue(tracker.snapshot()["byte_integrity_ok"])

        corrupt = arq.ByteIntegrityTracker(arq.TRAFFIC_RANDOM_BINARY)
        damaged = bytearray(stream)
        damaged[41] ^= 0x80
        corrupt.feed(damaged)
        snap = corrupt.snapshot()
        self.assertFalse(snap["byte_integrity_ok"])
        self.assertEqual(snap["integrity_mismatch_bytes"], 1)
        self.assertEqual(snap["integrity_mismatch_segments"], 1)
        self.assertEqual(snap["integrity_first_bad_offset"], 41)
        self.assertEqual(snap["integrity_disconnects"], 0)
        self.assertEqual(snap["integrity_sessions"], 1)
        self.assertEqual(snap["max_session_bytes"], 96)
        self.assertEqual(snap["total_received_bytes"], 96)
        self.assertFalse(snap["content_md5_ok"])


class CertDriverArgparseIntegrationTest(unittest.TestCase):
    def test_extended_cert_spawn_command_dry_run(self):
        command = [
            sys.executable, str(HERE / "parallel_spawner.py"),
            "--dry-run", "--n", "2", "--bin", "/tmp/mercury",
            "--start-cfg", "100", "--payload", "262144", "--secs", "800",
            "--profile", "wgn", "--launch-stagger", "2", "--arm", "contention",
            "--warm-start", "--traffic", "random-binary",
            "--score-horizon-s", "800", "--snr3k-db", "28",
            "--seed-offset", "0",
            "--spawn-plan", json.dumps(FIXTURE["spawn_plan"], separators=(",", ":")),
        ]
        completed = subprocess.run(
            command, check=False, capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        dry_run = json.loads(completed.stdout)
        self.assertEqual(len(dry_run["commands"]), 2)
        for child in dry_run["commands"]:
            for flag in ("--warm-start", "--traffic", "--score-horizon-s",
                         "--snr3k-db"):
                self.assertIn(flag, child)


class GoldenNoFlagIdentityTest(unittest.TestCase):
    def test_default_child_argv_uses_native_tune(self):
        args = spawner.build_arg_parser().parse_args(["--n", "1"])
        cell = spawner.run_plan(1, args.port_base)[0]
        actual = spawner.build_cell_command(args, cell, "/tmp/res.json")
        expected = [
            sys.executable, "-u", spawner.HARNESS,
            "--bin", "/home/kameron/raspeed/mercury",
            "--bridge", spawner.default_bridge_path(),
            "--tag", "par00", "--json", "/tmp/res.json",
            "--logdir", "/tmp/raspeed/logs", "--start-cfg", "100",
            "--secs", "130", "--payload", "512", "--seed", "1",
            "--card", "Loopback", "--subs", "0,1,2,3",
            "--rsp-port", "7100", "--cmd-port", "7104",
            "--cap-periods", "4", "--play-periods", "5",
            "--prime-periods", "2", "--no-kill",
            "--profile", "wgn", "--arm", "legacy",
        ]
        self.assertEqual(actual, expected)
        cell_defaults = arq.build_arg_parser().parse_args([])
        self.assertEqual(cell_defaults.traffic, arq.TRAFFIC_LEGACY)
        self.assertFalse(cell_defaults.warm_start)
        self.assertIsNone(cell_defaults.score_horizon_s)
        self.assertIsNone(cell_defaults.snr)
        self.assertIsNone(cell_defaults.snr3k)
        self.assertEqual(arq.traffic_slice("legacy", 0, 4096),
                         bytes(range(256)) * 16)

    def test_result_schema_is_legacy_plus_documented_additions(self):
        tree = ast.parse((HERE / "arq_realaudio.py").read_text())
        result_dicts = [
            node.value for node in ast.walk(tree)
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == "result"
                    for target in node.targets)
            and isinstance(node.value, ast.Dict)
        ]
        self.assertEqual(len(result_dicts), 1)
        actual = {ast.literal_eval(key) for key in result_dicts[0].keys}
        legacy = set(FIXTURE["legacy_result_keys"])
        additive = set(FIXTURE["additive_result_keys"])
        self.assertEqual(actual, legacy | additive)


if __name__ == "__main__":
    unittest.main()

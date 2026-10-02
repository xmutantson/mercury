#!/usr/bin/env python3
"""Offline selected-peer telemetry CLI and launch-environment contracts."""

import argparse
import contextlib
import io
import unittest
from unittest import mock

import sim_arq_channel as harness


class SelectedPeerTelemetryTests(unittest.TestCase):
    def parse(self, argv):
        parser = argparse.ArgumentParser()
        harness.add_rro_telemetry_arguments(parser)
        return parser.parse_args(argv)

    def test_cli_defaults_leave_peer_unselected(self):
        args = self.parse([])
        self.assertIsNone(args.rro_telemetry_peer)
        self.assertEqual(args.rro_telemetry_port, 38429)
        self.assertEqual(args.rro_telemetry_version, 2)

    def test_explicit_a_and_b_use_default_v2_port_for_selected_peer(self):
        for selected in ("A", "B"):
            with self.subTest(selected=selected):
                args = self.parse(["--rro-telemetry-peer", selected])
                for role in ("A", "B"):
                    env = harness.peer_telemetry_environment(
                        {}, role, args.rro_telemetry_peer,
                        args.rro_telemetry_port, args.rro_telemetry_version)
                    if role == selected:
                        self.assertEqual(env, {
                            "MERCURY_RRO_TELEMETRY": "1",
                            "MERCURY_RRO_TELEMETRY_VERSION": "2",
                            "MERCURY_RRO_UDP_PORT": "38429",
                        })
                    else:
                        self.assertEqual(env, {"MERCURY_RRO_TELEMETRY": "0"})

    def test_inherited_global_enable_cannot_mix_peers(self):
        inherited = {
            "MERCURY_RRO_TELEMETRY": "1",
            "MERCURY_RRO_TELEMETRY_VERSION": "invalid",
            "MERCURY_RRO_UDP_PORT": "invalid",
            "MERCURY_SIM_REALTIME": "1",
            "MERCURY_RATE_TABLE": "effective_rate_table.json",
            "UNRELATED": "unchanged",
        }
        original = dict(inherited)
        for selected in ("A", "B"):
            for role in ("A", "B"):
                with self.subTest(selected=selected, role=role):
                    env = harness.peer_telemetry_environment(
                        inherited, role, selected, 40000, 1)
                    self.assertIsNot(env, inherited)
                    self.assertEqual(env["MERCURY_RRO_TELEMETRY"],
                                     "1" if role == selected else "0")
                    self.assertEqual(env["MERCURY_SIM_REALTIME"], "1")
                    self.assertEqual(env["MERCURY_RATE_TABLE"],
                                     "effective_rate_table.json")
                    self.assertEqual(env["UNRELATED"], "unchanged")
                    if role == selected:
                        self.assertEqual(env["MERCURY_RRO_TELEMETRY_VERSION"], "1")
                        self.assertEqual(env["MERCURY_RRO_UDP_PORT"], "40000")
                    else:
                        self.assertEqual(env["MERCURY_RRO_TELEMETRY_VERSION"],
                                         "invalid")
                        self.assertEqual(env["MERCURY_RRO_UDP_PORT"], "invalid")
        self.assertEqual(inherited, original)

    def test_unselected_preserves_all_inherited_settings(self):
        for inherited in ({}, {
                "MERCURY_RRO_TELEMETRY": "1",
                "MERCURY_RRO_TELEMETRY_VERSION": "1",
                "MERCURY_RRO_UDP_PORT": "50000",
                "UNRELATED": "unchanged"}):
            for role in ("A", "B"):
                with self.subTest(inherited=inherited, role=role):
                    env = harness.peer_telemetry_environment(
                        inherited, role, None, 40000, 2)
                    self.assertEqual(env, inherited)
                    self.assertIsNot(env, inherited)

    def test_valid_port_boundaries_and_versions_serialize_exactly(self):
        for port in (1, 38429, 65535):
            for version in (1, 2):
                with self.subTest(port=port, version=version):
                    args = self.parse([
                        "--rro-telemetry-peer", "B",
                        "--rro-telemetry-port", str(port),
                        "--rro-telemetry-version", str(version)])
                    env = harness.peer_telemetry_environment(
                        {}, "B", args.rro_telemetry_peer,
                        args.rro_telemetry_port, args.rro_telemetry_version)
                    self.assertEqual(env["MERCURY_RRO_TELEMETRY"], "1")
                    self.assertEqual(env["MERCURY_RRO_UDP_PORT"], str(port))
                    self.assertEqual(env["MERCURY_RRO_TELEMETRY_VERSION"],
                                     str(version))

    def test_invalid_cli_options_fail_before_guard_probe_or_launch(self):
        invalid = [
            ["--rro-telemetry-peer", "C"],
            ["--rro-telemetry-peer", "a"],
            *[["--rro-telemetry-port", value]
              for value in ("0", "-1", "65536", "1.5", "invalid")],
            *[["--rro-telemetry-version", value]
              for value in ("0", "3", "invalid")],
        ]
        for argv in invalid:
            with self.subTest(argv=argv), \
                    mock.patch.object(harness.sys, "argv", ["harness", *argv]), \
                    mock.patch.object(harness, "require_guard_binary") as guard, \
                    mock.patch.object(harness, "pick_free_ports") as ports, \
                    mock.patch.object(harness.subprocess, "Popen") as popen, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    harness.main()
                self.assertEqual(error.exception.code, 2)
                guard.assert_not_called()
                ports.assert_not_called()
                popen.assert_not_called()

    def test_environment_helper_rejects_invalid_domains(self):
        invalid = [
            {"role": "C"}, {"selected_peer": "C"},
            *[{"port": value} for value in (0, 65536, True, "38429")],
            *[{"version": value} for value in (0, 3, True, "2")],
        ]
        for override in invalid:
            with self.subTest(override=override):
                kwargs = {"role": "A", "selected_peer": "A"}
                kwargs.update(override)
                with self.assertRaises(ValueError):
                    harness.peer_telemetry_environment({}, **kwargs)

    def test_selected_peer_still_requires_audio_guard_before_any_probe(self):
        class GuardStopped(Exception):
            pass

        argv = ["harness", "--bin", "unqualified.exe",
                "--rro-telemetry-peer", "B", "--rro-telemetry-port", "40000",
                "--rro-telemetry-version", "1", "--start-cfg", "10",
                "--wire-stamp", "1", "--secs", "30"]
        with mock.patch.object(harness.sys, "argv", argv), \
                mock.patch.object(harness, "require_guard_binary",
                                  side_effect=GuardStopped) as guard, \
                mock.patch.object(harness, "pick_free_ports") as ports, \
                mock.patch.object(harness.subprocess, "Popen") as popen:
            with self.assertRaises(GuardStopped):
                harness.main()
            guard.assert_called_once_with("unqualified.exe")
            ports.assert_not_called()
            popen.assert_not_called()


class QaRunSafetyTests(unittest.TestCase):
    def parse(self, argv):
        parser = argparse.ArgumentParser()
        harness.add_qa_safety_arguments(parser)
        return parser.parse_args(argv)

    @staticmethod
    def process(pid, exited=False):
        process = mock.Mock(pid=pid)
        process.poll.return_value = 0 if exited else None
        process.kill.side_effect = lambda: setattr(process.poll, "return_value", -1)
        return process

    def test_safety_defaults_preserve_existing_run_policy(self):
        args = self.parse([])
        self.assertFalse(args.owned_process_cleanup_only)
        self.assertIsNone(args.wall_timeout_secs)
        with mock.patch.object(harness.time, "sleep") as sleep, \
                mock.patch.object(harness.time, "monotonic") as clock:
            deadline = harness.WallDeadline()
            self.assertIsNone(deadline.remaining())
            self.assertEqual(deadline.socket_timeout(30), 30)
            deadline.pause(3)
            sleep.assert_called_once_with(3)
            clock.assert_not_called()

    def test_wall_timeout_valid_bounds(self):
        for seconds in (0.1, 1, 120, 600):
            with self.subTest(seconds=seconds):
                args = self.parse([
                    "--owned-process-cleanup-only", "--wall-timeout-secs",
                    str(seconds)])
                self.assertTrue(args.owned_process_cleanup_only)
                self.assertEqual(args.wall_timeout_secs, seconds)

    def test_invalid_wall_options_fail_before_any_guard_or_probe(self):
        invalid = [
            ["--wall-timeout-secs", "120"],
            *[["--owned-process-cleanup-only", "--wall-timeout-secs", value]
              for value in ("0", "-1", "600.1", "nan", "inf", "invalid")],
        ]
        for argv in invalid:
            with self.subTest(argv=argv), \
                    mock.patch.object(harness.sys, "argv", ["harness", *argv]), \
                    mock.patch.object(harness, "require_guard_binary") as guard, \
                    mock.patch.object(harness, "_port_free") as ports, \
                    mock.patch.object(harness.subprocess, "Popen") as popen, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    harness.main()
                self.assertEqual(error.exception.code, 2)
                guard.assert_not_called()
                ports.assert_not_called()
                popen.assert_not_called()

    def test_owned_ports_are_exact_and_never_auto_advance(self):
        with mock.patch.object(harness, "_port_free", return_value=True) as probe:
            self.assertEqual(harness.pick_owned_run_ports(7100, 52000),
                             (7100, 7101, 7104, 7105, 52000))
            self.assertEqual([call.args[0] for call in probe.call_args_list],
                             [7100, 7101, 7104, 7105, 52000])
        with mock.patch.object(harness, "_port_free", return_value=False) as probe:
            with self.assertRaisesRegex(RuntimeError, "occupied"):
                harness.pick_owned_run_ports(7100, 52000)
            probe.assert_called_once_with(7100)
        with mock.patch.object(harness, "_port_free") as probe:
            for base, relay in ((7100, 7101), (65531, 52000), (0, 52000)):
                with self.subTest(base=base, relay=relay), \
                        self.assertRaises(RuntimeError):
                    harness.pick_owned_run_ports(base, relay)

    def test_busy_owned_port_fails_without_discovering_or_killing_foreign_pid(self):
        argv = ["harness", "--bin", "qualified.exe",
                "--owned-process-cleanup-only", "--ctrl-base", "7100",
                "--port", "52000"]
        with mock.patch.object(harness.sys, "argv", argv), \
                mock.patch.object(harness, "require_guard_binary"), \
                mock.patch.object(harness, "_port_free", return_value=False), \
                mock.patch.object(harness, "_pids_on_ports",
                                  return_value={"99999"}) as discovery, \
                mock.patch.object(harness, "kill_port_scoped") as legacy_cleanup, \
                mock.patch.object(harness.subprocess, "Popen") as popen, \
                mock.patch.object(harness.subprocess, "run") as subprocess_run, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                harness.main()
            self.assertEqual(error.exception.code, 3)
            discovery.assert_not_called()
            legacy_cleanup.assert_not_called()
            popen.assert_not_called()
            subprocess_run.assert_not_called()

    def test_cleanup_and_deadline_kill_only_retained_live_handles(self):
        stop = mock.Mock()
        owned = harness.OwnedProcessHandles(stop)
        relay, modem, exited, foreign = [self.process(pid, exited=(pid == 3))
                                        for pid in (1, 2, 3, 99999)]
        for process in (relay, modem, exited):
            self.assertIs(owned.retain(process), process)
        with mock.patch.object(harness, "_pids_on_ports",
                               return_value={"99999"}) as discovery, \
                mock.patch.object(harness.subprocess, "run") as subprocess_run:
            owned.expire()
            owned.kill_all()
            relay.kill.assert_called_once()
            modem.kill.assert_called_once()
            exited.kill.assert_not_called()
            foreign.kill.assert_not_called()
            stop.set.assert_called_once()
            discovery.assert_not_called()
            subprocess_run.assert_not_called()
        late = self.process(4)
        with self.assertRaises(harness.WallTimeout):
            owned.retain(late)
        late.kill.assert_called_once()

    def test_wall_math_uses_monotonic_startup_budget_not_virtual_or_civil_time(self):
        clock = [100.0]

        def advance(seconds):
            clock[0] += seconds

        with mock.patch.object(harness.time, "monotonic", side_effect=lambda: clock[0]), \
                mock.patch.object(harness.time, "sleep", side_effect=advance) as sleep, \
                mock.patch.object(harness.time, "time", return_value=-1000000):
            deadline = harness.WallDeadline(120)
            self.assertEqual(deadline.expires_at, 220)
            deadline.pause(3)
            self.assertEqual(deadline.remaining(), 117)
            clock[0] = 219.5
            self.assertEqual(deadline.socket_timeout(30), 0.5)
            with self.assertRaises(harness.WallTimeout):
                deadline.pause(3)
            self.assertEqual(clock[0], 220)
            self.assertEqual(sleep.call_args.args, (0.5,))

    def test_control_connect_retries_cannot_extend_wall_deadline(self):
        clock = [100.0]
        sock = mock.Mock()

        def timed_out_connect(address):
            clock[0] += 5
            raise OSError("connect exceeded remaining wall budget")

        sock.connect.side_effect = timed_out_connect
        with mock.patch.object(harness.time, "monotonic", side_effect=lambda: clock[0]), \
                mock.patch.object(harness.time, "sleep") as sleep, \
                mock.patch.object(harness.socket, "socket", return_value=sock) as factory:
            deadline = harness.WallDeadline(5)
            with self.assertRaises(harness.WallTimeout):
                harness.tcp_send(7100, ["LISTEN ON\r\n"], "RSP", deadline=deadline)
            factory.assert_called_once()
            sock.settimeout.assert_called_once_with(5)
            sock.connect.assert_called_once_with(("127.0.0.1", 7100))
            sock.close.assert_called_once()
            sock.sendall.assert_not_called()
            sleep.assert_not_called()

    def test_data_connect_timeout_is_reported_as_wall_timeout(self):
        clock = [100.0]
        sock = mock.Mock()

        def timed_out_connect(address):
            clock[0] = 101
            raise OSError("socket timeout")

        sock.connect.side_effect = timed_out_connect
        with mock.patch.object(harness.time, "monotonic", side_effect=lambda: clock[0]):
            deadline = harness.WallDeadline(1)
            with self.assertRaises(harness.WallTimeout):
                deadline.connect(sock, ("127.0.0.1", 7101))
            sock.settimeout.assert_called_once_with(1)

    def test_actual_launch_path_selects_one_peer_and_finally_cleans_exact_handles(self):
        for selected in ("A", "B"):
            with self.subTest(selected=selected):
                processes = [self.process(pid) for pid in (1, 2, 3)]
                argv = ["harness", "--bin", "qualified.exe",
                        "--rro-telemetry-peer", selected,
                        "--owned-process-cleanup-only", "--wall-timeout-secs", "120",
                        "--start-cfg", "10", "--no-gearshift", "--wire-stamp", "1",
                        "--log", "offline-source-log.txt",
                        "--json", "offline-mocked-result.json"]
                with contextlib.ExitStack() as stack:
                    source_open = mock.mock_open(read_data=b"payload")
                    for patch in (
                            mock.patch.object(harness.sys, "argv", argv),
                            mock.patch.dict(harness.os.environ, {
                                "MERCURY_RRO_TELEMETRY": "1",
                                "MERCURY_RRO_TELEMETRY_VERSION": "1",
                                "MERCURY_RRO_UDP_PORT": "50000"}),
                            mock.patch.object(harness, "require_guard_binary"),
                            mock.patch.object(harness, "_port_free", return_value=True),
                            mock.patch.object(harness.os.path, "isfile", return_value=True),
                            mock.patch.object(harness.os.path, "exists", return_value=False),
                            mock.patch("builtins.open", source_open),
                            mock.patch.object(harness.threading, "Thread"),
                            mock.patch.object(harness.WallDeadline, "pause"),
                            mock.patch.object(harness, "tcp_send",
                                              side_effect=harness.WallTimeout),
                            mock.patch.object(harness, "read_relay_virtual_seconds",
                                              return_value=(0, False)),
                            contextlib.redirect_stdout(io.StringIO())):
                        stack.enter_context(patch)
                    result_dump = stack.enter_context(mock.patch.object(harness.json, "dump"))
                    popen = stack.enter_context(mock.patch.object(
                        harness.subprocess, "Popen", side_effect=processes))
                    subprocess_run = stack.enter_context(mock.patch.object(
                        harness.subprocess, "run"))
                    timer = stack.enter_context(mock.patch.object(harness.threading, "Timer"))
                    legacy_cleanup = stack.enter_context(mock.patch.object(
                        harness, "kill_port_scoped"))
                    discovery = stack.enter_context(mock.patch.object(harness, "_pids_on_ports"))
                    self.assertEqual(harness.main(), 0)
                    source_open.assert_any_call("offline-source-log.txt", "w",
                                                encoding="utf-8")
                    self.assertEqual(popen.call_count, 3)
                    for call in popen.call_args_list[1:]:
                        env = call.kwargs["env"]
                        role = env["MERCURY_SIM_ROLE"]
                        self.assertEqual(env["MERCURY_RRO_TELEMETRY"],
                                         "1" if role == selected else "0")
                        if role == selected:
                            self.assertEqual(env["MERCURY_RRO_TELEMETRY_VERSION"], "2")
                            self.assertEqual(env["MERCURY_RRO_UDP_PORT"], "38429")
                        self.assertEqual(env["MERCURY_CONNECT_FAST_CONFIG"], "10")
                        command = call.args[0]
                        self.assertEqual(command[:10], [
                            "qualified.exe", "-m", "ARQ", "-s", "10", "-W",
                            "-p", str(harness.RSP_PORT if role == "B"
                                      else harness.CMD_PORT), "-x", "sim"])
                        self.assertEqual(command[-4:], ["--wire-stamp", "1", "-Q", "0"])
                    for process in processes:
                        process.kill.assert_called_once()
                    self.assertGreater(timer.call_args.args[0], 119)
                    self.assertLessEqual(timer.call_args.args[0], 120)
                    timer.return_value.start.assert_called_once()
                    timer.return_value.cancel.assert_called_once()
                    result_dump.assert_called_once()
                    result = result_dump.call_args.args[0]
                    self.assertEqual(result["bounded_by"], "wall_timeout")
                    self.assertEqual(result["rro_telemetry_peer"], selected)
                    self.assertEqual(result["rro_telemetry_port"], 38429)
                    self.assertEqual(result["rro_telemetry_version"], 2)
                    self.assertTrue(result["owned_process_cleanup_only"])
                    self.assertEqual(result["wall_timeout_secs"], 120)
                    subprocess_run.assert_not_called()
                    discovery.assert_not_called()
                    legacy_cleanup.assert_not_called()


class SourceLogEncodingTests(unittest.TestCase):
    def test_unicode_source_line_keeps_pipe_draining_and_reaches_parsers(self):
        source = ("[PRECOOK] config bundles built: NB↔WB adopt switches\n"
                  "[SIM] RX bridge first vstamp=1024\n"
                  "link_status:Connected to TESTB\n"
                  "[GEARSHIFT] SET_CONFIG: forward=10\n"
                  "[CMD-RETX] Sending 2 retransmit frames\n")
        process = mock.Mock(stdout=io.BytesIO(source.encode("utf-8")))
        stored = io.BytesIO()
        source_log = io.TextIOWrapper(stored, encoding="utf-8")
        terminal_bytes = io.BytesIO()
        terminal = io.TextIOWrapper(terminal_bytes, encoding="cp1252")
        state = harness.State()
        with contextlib.redirect_stdout(terminal), \
                mock.patch.object(harness, "cfg_name", return_value="CONFIG_10 ↔ WB"):
            harness.log_output(process, "CMD", source_log, 0, state)
        source_log.flush()
        terminal.flush()
        self.assertIn("NB↔WB", stored.getvalue().decode("utf-8"))
        self.assertIn("[CMD-RETX] Sending 2", stored.getvalue().decode("utf-8"))
        self.assertEqual(process.stdout.tell(), len(source.encode("utf-8")))
        self.assertTrue(state.connected)
        self.assertEqual(state.first_vstamp["CMD"], 1024)
        self.assertEqual(state.switch_seq, [10])
        self.assertEqual(state.cmd_retx_frames, 2)
        self.assertIn("\\u2194", terminal_bytes.getvalue().decode("cp1252"))
        source_log.close()
        terminal.close()

    def test_closed_terminal_does_not_abandon_source_pipe(self):
        source = ("[GEARSHIFT] SET_CONFIG: forward=10\n"
                  "[SIM] RX bridge first vstamp=1024\n"
                  "link_status:Connected to TESTB\n")
        process = mock.Mock(stdout=io.BytesIO(source.encode("utf-8")))
        terminal = io.StringIO()
        terminal.close()
        source_log = io.StringIO()
        state = harness.State()
        with contextlib.redirect_stdout(terminal):
            harness.log_output(process, "CMD", source_log, 0, state)
        self.assertTrue(state.connected)
        self.assertEqual(state.first_vstamp["CMD"], 1024)
        self.assertIn("link_status:Connected", source_log.getvalue())


if __name__ == "__main__":
    unittest.main()

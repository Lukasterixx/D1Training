"""The D1's command timing without the arm: the report clock keeps commands out of the stall window (F-238), the phase
analysis finds that window, and the rate test's locked and swing blocks run against a pretend arm that stalls there."""
from __future__ import annotations

import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np

import d1_hardware
import d1_ik
from d1_hardware import GATE_OPEN_S, ReportClock, REPORT_PERIOD_S, STALL_WINDOW_S, stall_wait_s
from demos.combiner_door.robot import arm_command_rate_test as rate_test
from demos.combiner_door.robot import arm_gap_phase, arm_link
from demos.combiner_key.robot import extract_demo_draw, holster_dog


def sends(clock, reports, end_s, step_s=0.001):
    """Step a clock through time with the reports it would see; the times it sends."""
    out = []
    for now in np.arange(0.0, end_s, step_s):
        seen = [r for r in reports if r <= now]
        if clock.due(float(now), seen[-1] if seen else None):
            clock.sent(float(now))
            out.append(float(now))
    return np.array(out)


def phases(times, reports):
    reports = np.array(reports)
    return np.array([t - reports[reports <= t][-1] for t in times if (reports <= t).any()])


class ReportClockTests(unittest.TestCase):
    def test_one_command_a_report_at_its_offset(self):
        reports = list(np.arange(0.05, 20.0, REPORT_PERIOD_S))
        times = sends(ReportClock(offset_s=0.02), reports, 20.0)
        self.assertAlmostEqual(len(times), len(reports), delta=1)
        self.assertTrue(np.all(np.abs(phases(times, reports) - 0.02) < 0.003))

    def test_it_stays_out_of_the_window_while_the_reports_jitter(self):
        rng = np.random.default_rng(0)
        reports = list(np.cumsum(REPORT_PERIOD_S + rng.normal(0.0, 0.0016, 400)))
        got = phases(sends(ReportClock(offset_s=0.02), reports, reports[-1]), reports)
        self.assertLess(got.max(), STALL_WINDOW_S[0])

    def test_it_never_outruns_the_arms_ten_hertz(self):
        rng = np.random.default_rng(1)
        reports = list(np.cumsum(rng.uniform(0.02, 0.2, 300)))  # far worse than the arm
        times = sends(ReportClock(offset_s=0.02), reports, reports[-1])
        self.assertGreaterEqual(np.diff(times).min(), 1.0 / d1_hardware.MAX_COMMAND_HZ - 1e-9)

    def test_it_carries_on_through_a_gap_in_the_reports(self):
        reports = [0.1, 0.211, 0.322]  # then nothing
        times = sends(ReportClock(offset_s=0.02), reports, 1.5)
        later = np.diff(times[times > 0.4])
        self.assertTrue(np.all(np.abs(later - REPORT_PERIOD_S) < 0.003), later)

    def test_a_late_loop_skips_the_slot_rather_than_drift_to_the_window(self):
        clock = ReportClock(offset_s=0.02, slack_s=0.04)
        self.assertTrue(clock.due(1.03, 1.0))
        self.assertFalse(clock.due(1.08, 1.0))  # 60 ms past the slot: wait for the next
        self.assertTrue(clock.due(1.135, 1.0))

    def test_a_slot_on_the_pacing_floor_waits_for_it_rather_than_being_dropped(self):
        # 2026-11-06, the one locked skip: reports at 0 and 0.210, a slot at 0.230 only just inside 100 ms of the last send.
        clock = ReportClock(offset_s=0.02)
        clock.sent(0.1305)
        self.assertFalse(clock.due(0.2302, 0.210))
        self.assertTrue(clock.due(0.2306, 0.210))

    def test_nothing_is_due_before_the_first_report(self):
        self.assertFalse(ReportClock().due(5.0, None))

    def test_a_clock_in_the_window_has_to_be_asked_for(self):
        with self.assertRaises(ValueError):
            ReportClock(offset_s=0.103)
        with self.assertRaises(ValueError):
            ReportClock(offset_s=0.06, slack_s=0.04)
        ReportClock(offset_s=0.103, allow_window=True)


class GateTests(unittest.TestCase):
    def gated(self, now, report_s):
        return now + stall_wait_s(now, report_s)

    def test_a_command_clear_of_the_window_goes_at_once(self):
        for phase in (GATE_OPEN_S, 0.05, STALL_WINDOW_S[0]):
            self.assertEqual(stall_wait_s(10.0 + phase, 10.0), 0.0)

    def test_every_moment_of_the_cycle_comes_out_clear_and_soon(self):
        for phase in np.arange(0.0, 3 * REPORT_PERIOD_S, 0.0005):
            wait = stall_wait_s(10.0 + phase, 10.0)
            out = (phase + wait) % REPORT_PERIOD_S
            self.assertLess(wait, 0.04)
            self.assertTrue(GATE_OPEN_S - 1e-9 <= out <= STALL_WINDOW_S[0] + 1e-9, (phase, wait))

    def test_with_no_report_or_an_old_one_it_does_not_hold_the_command(self):
        self.assertEqual(stall_wait_s(10.0, None), 0.0)
        self.assertEqual(stall_wait_s(12.0, 10.0), 0.0)

    def test_a_ten_hertz_sender_never_lands_in_the_window(self):
        reports = np.arange(0.0, 30.0, REPORT_PERIOD_S)
        now, sent = 0.2, []
        while now < 29.0:
            report = reports[reports <= now][-1]
            now = self.gated(now, report)
            sent.append(now)
            now = max(now + 0.0, sent[-1] + 0.1)
        got = phases(sent, list(reports))
        self.assertLess(got.max(), STALL_WINDOW_S[0] + 1e-6)
        self.assertGreater(len(sent) / 28.8, 8.5)

    def test_pace_holds_a_command_but_never_the_release_path(self):
        client = d1_hardware.D1Client.__new__(d1_hardware.D1Client)
        client._min_command_interval_s, client._last_motion_tx = 0.0, 0.0
        client._last_servo_rx = time.monotonic() - 0.100
        started = time.monotonic()
        client._pace()
        self.assertGreater(time.monotonic() - started, 0.02)
        client.report_gate = False
        client._last_servo_rx = time.monotonic() - 0.100
        started = time.monotonic()
        client._pace()
        self.assertLess(time.monotonic() - started, 0.01)


class SwingTests(unittest.TestCase):
    def test_it_starts_at_the_middle_and_stays_in_reach(self):
        self.assertEqual(rate_test.swing_offset(0.0, 10.0), 0.0)
        got = [rate_test.swing_offset(t, 10.0) for t in np.arange(0.0, 5.0, 0.01)]
        self.assertAlmostEqual(max(got), 10.0, delta=0.4)
        self.assertAlmostEqual(min(got), -10.0, delta=0.4)

    def test_it_steps_no_more_than_its_pace_and_never_jumps(self):
        got = np.array([rate_test.swing_offset(t, 10.0) for t in np.arange(0.0, 20.0, 0.111)])
        self.assertLess(np.abs(np.diff(got)).max(), rate_test.SWING_DEG_S * 0.111 + 1e-6)

    def test_a_held_block_does_not_swing(self):
        self.assertEqual(rate_test.swing_offset(3.0, 0.0), 0.0)

    def test_only_the_swing_and_gate_sets_move_the_arm(self):
        moving = {name for name, blocks in rate_test.BLOCK_SETS.items()
                  if any(len(block) > 4 and block[4] for block in blocks)}
        self.assertEqual(moving, {"swing", "gate"})


class PhaseAnalysisTests(unittest.TestCase):
    def test_it_counts_cycles_in_and_out_of_the_window(self):
        reports = np.array([0.0, 0.111, 0.222, 0.633, 0.744])
        commands = np.array([0.05, 0.161, 0.322])  # the third is 100 ms after the report at 0.222
        summary = arm_gap_phase.summarise(arm_gap_phase.cycle_rows(reports, commands))
        self.assertEqual((summary["cycles"], summary["skips"]), (4, 1))
        self.assertEqual((summary["in_window"], summary["in_window_skipped"], summary["outside_skipped"]), (1, 1, 0))
        self.assertAlmostEqual(summary["longest_s"], 0.411)

    def test_only_the_commands_with_the_grippers_angle_count(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run / "feedback_timing").mkdir()
            rows = [{"topic": "current_servo_angle", "src": 10.0 + i * 0.111, "wall": 110.0 + i * 0.111}
                    for i in range(5)]
            (run / "feedback_timing" / "samples.jsonl").write_text("\n".join(map(json.dumps, rows)))
            sent = [{"t": 110.1, "kind": "command", "sent": '{"data": {"angle6": 1}}'},
                    {"t": 110.2, "kind": "command", "sent": '{"data": {"angle5": 1}}'},
                    {"t": 110.3, "kind": "command", "sent": '{"data": {"angle6": 1}}'}]
            (run / "arm_link.jsonl").write_text("\n".join(map(json.dumps, sent)))
            _, reports, commands = arm_gap_phase.live_run(run)
        self.assertEqual(commands.tolist(), [110.1, 110.3])
        self.assertTrue(np.all((reports >= 110.1) & (reports <= 110.3)))


class LinkLockTests(unittest.TestCase):
    def test_locked_it_goes_by_the_reports_and_not_the_clock(self):
        client = mock.Mock(servo_report_s=10.0)
        link = arm_link.Link(client, False, None, lock_reports=True)
        self.assertFalse(link.command_due(10.01, None, 0.1))
        self.assertTrue(link.command_due(10.021, None, 0.1))
        link.report_clock.sent(10.021)
        self.assertFalse(link.command_due(10.105, 10.021, 0.1))  # 100 ms on: the clock's slot, in the window

    def test_it_is_locked_unless_told_otherwise(self):
        self.assertIsNotNone(arm_link.Link(mock.Mock(), False, None).report_clock)

    def test_unlocked_it_keeps_the_clock(self):
        link = arm_link.Link(mock.Mock(), False, None, lock_reports=False)
        self.assertTrue(link.command_due(5.0, None, 0.1))
        self.assertFalse(link.command_due(5.05, 5.0, 0.1))
        self.assertTrue(link.command_due(5.11, 5.0, 0.1))


class DemoDrawTests(unittest.TestCase):
    def setUp(self):
        self.moves = holster_dog.demo_plan()

    def test_a_fast_stretch_is_slowed_and_the_path_kept(self):
        points = np.zeros((4, 6))
        points[:, 0] = [0.0, 2.0, 10.0, 12.0]
        got = extract_demo_draw.retimed(points)
        self.assertLessEqual(np.abs(np.diff(got, axis=0)).max() / 0.1, extract_demo_draw.MAX_SPEED_DEG_S + 1e-6)
        self.assertEqual((got[0, 0], got[-1, 0]), (0.0, 12.0))
        self.assertGreater(len(got), 4)

    def test_a_slow_path_keeps_its_time(self):
        points = np.zeros((5, 6))
        points[:, 1] = np.arange(5.0)
        self.assertEqual(len(extract_demo_draw.retimed(points)), 5)

    def test_every_path_is_one_the_link_takes(self):
        where = None
        for m in self.moves:
            if "path" not in m:
                continue
            path = np.asarray(m["path"])
            self.assertIsNone(arm_link.check_path(path, 0.1, path[0] if where is None else where), m["name"])
            where = path[-1]

    def test_it_joins_up_with_the_lock_left_out(self):
        by_name = {m["name"]: m for m in self.moves}
        for a, b in (("hold", "clear"), ("clear", "unclear"), ("unclear", "align"), ("align", "enter"), ("enter", "back")):
            gap = np.abs(np.asarray(by_name[a]["path"][-1]) - np.asarray(by_name[b]["path"][0])).max()
            self.assertLess(gap, 2.0, (a, b))

    def test_it_ends_where_it_began_and_the_way_back_exists(self):
        names = [m["name"] for m in self.moves]
        self.assertEqual((names[0], names[-1]), ("turn", "fold"))
        self.assertIn("enter", names)
        self.assertEqual(self.moves[-1]["reverse_of"], "turn")

    def test_the_jaws_shut_only_between_close_and_open(self):
        names = [m["name"] for m in self.moves]
        shut = [m["name"] for m in self.moves if m["gripper"] < 0]
        self.assertEqual(shut, names[names.index("close"):names.index("open")])


class FoldBackTests(unittest.TestCase):
    def test_it_is_the_zero_tests_way_back_and_only_that(self):
        moves = holster_dog.fold_back_plan()
        self.assertEqual([m["name"] for m in moves], ["home", "tuck", "fold"])
        np.testing.assert_allclose(moves[-1]["goal"], holster_dog.FOLD_SERVO_DEG)
        self.assertTrue(all(m["speed_deg_s"] == holster_dog.ZERO_SPEED_DEG_S for m in moves))

    def test_from_where_the_stop_left_it_every_path_is_one_the_link_takes(self):
        start = np.array([-7.2, 3.1, -0.8, 54.9, 14.2, -13.0])
        where = start
        for m in holster_dog.fold_back_plan():
            path = holster_dog.move_path(m, where)
            self.assertIsNone(arm_link.check_path(path.tolist(), holster_dog.PERIOD_S, where))
            where = path[-1]
        np.testing.assert_allclose(where, holster_dog.FOLD_SERVO_DEG, atol=0.01)

    def test_a_stop_on_the_way_goes_on_forward(self):
        moves = holster_dog.fold_back_plan()
        self.assertEqual([m["name"] for m in holster_dog.recovery(moves, {}, 1, np.zeros(6))], ["tuck", "fold"])

    def test_it_needs_only_a_fresh_report(self):
        arm = mock.Mock()
        arm.latest.return_value = (np.array([0.0, 0, 0, 55, 0, 0]), 50.0, 0.01)
        self.assertIsNone(holster_dog.check_report(arm))
        self.assertIsNotNone(holster_dog.check_start(arm))


class ConsoleDryRunTests(unittest.TestCase):
    def test_the_consoles_dry_run_goes_at_the_real_pace_so_b_can_land_in_it(self):
        with mock.patch.object(holster_dog.time, "sleep") as slept:
            holster_dog.SimArm().tick()
            slept.assert_not_called()
            holster_dog.SimArm(realtime=True).tick()
            slept.assert_called_once_with(holster_dog.PERIOD_S)

    def test_the_remote_takes_ros_down_and_its_thread_with_it(self):
        remote = holster_dog.Remote.__new__(holster_dog.Remote)
        stopped = threading.Event()
        remote.rclpy = mock.Mock()
        remote.rclpy.spin.side_effect = lambda node: (stopped.wait(2.0), (_ for _ in ()).throw(RuntimeError("shut down")))
        remote.rclpy.try_shutdown.side_effect = stopped.set
        remote.node = None
        remote.thread = threading.Thread(target=remote._spin, daemon=True)
        remote.thread.start()
        remote.close()
        self.assertFalse(remote.thread.is_alive())


class ConsoleStdinTests(unittest.TestCase):
    def runner(self, text, **kw):
        with mock.patch("sys.stdin", io.StringIO(text)):
            made = holster_dog.Runner(mock.Mock(), kw.pop("live", False), False, Path("."), [], **kw)
            time.sleep(0.05)
        return made

    def test_the_console_closing_is_a_stop(self):
        self.assertEqual(self.runner("", console=True).keys.get(timeout=1), "a")

    def test_its_lines_are_read_in_a_dry_run_too(self):
        made = self.runner("r\n", console=True)
        self.assertEqual([made.keys.get(timeout=1), made.keys.get(timeout=1)], ["r", "a"])

    def test_a_terminal_closing_is_not(self):
        self.assertTrue(self.runner("", live=True).keys.empty())

    def test_a_dry_run_off_the_console_reads_nothing(self):
        self.assertTrue(self.runner("a\n").keys.empty())


class ZeroTestTests(unittest.TestCase):
    def setUp(self):
        self.moves = holster_dog.zero_plan()
        self.names = [m["name"] for m in self.moves]
        self.fold = np.asarray(holster_dog.FOLD_SERVO_DEG)

    def played(self):
        """Every joint path in order, as the link would play it."""
        out, where, played = [], self.fold, {}
        for m in self.moves:
            if "follow" in m:
                out.append(m["follow"])
                where = m["follow"][-1]
            else:
                played[m["name"]] = holster_dog.move_path(m, where, played)
                out.append(played[m["name"]])
                where = played[m["name"]][-1]
        return out

    def test_it_ends_folded_by_home_and_tuck(self):
        self.assertEqual(self.names[-3:], ["home", "tuck", "fold"])
        self.assertEqual(float(self.moves[-2]["goal"][2]), self.fold[2] - 30.0)
        np.testing.assert_array_equal(self.moves[-1]["goal"], self.fold)

    def test_the_follow_starts_and_ends_on_zero_at_a_few_degrees_a_command(self):
        goals = holster_dog.zero_circle()
        np.testing.assert_allclose(goals[[0, -1]], 0.0, atol=1e-9)
        per_report = np.abs(np.diff(goals, axis=0)).max() / holster_dog.PERIOD_S * d1_hardware.REPORT_PERIOD_S
        self.assertLess(per_report, 4.0)
        self.assertGreater(per_report, 2.0)
        self.assertLessEqual(np.abs(goals).max(), holster_dog.ZERO_CIRCLE_DEG)

    def test_every_path_is_one_the_link_takes(self):
        where = self.fold
        for path in self.played():
            self.assertIsNone(arm_link.check_path(path, holster_dog.PERIOD_S, where)) if len(path) > 1 else None
            where = path[-1]

    def test_the_cpu_model_keeps_it_off_the_body_all_the_way(self):
        from demos.cup.pick_demo import grasp

        joints, _ = d1_ik.load_urdf()
        margin = min(float(grasp.proxy_margin(joints, np.asarray(d1_ik.from_servo_deg(q)[0])))
                     for path in self.played() for q in path)
        self.assertGreater(margin, 0.04)        # the fold itself is 46 mm

    def test_a_stop_anywhere_goes_on_through_home_tuck_and_fold(self):
        for k in range(self.names.index("home") + 1):
            got = [m["name"] for m in holster_dog.recovery(self.moves, {}, k, np.zeros(6))]
            self.assertEqual(got, ["home", "tuck", "fold"], self.names[k])

    def test_a_stop_on_the_way_back_finishes_it(self):
        got = [m["name"] for m in holster_dog.recovery(self.moves, {}, self.names.index("tuck"), self.fold)]
        self.assertEqual(got, ["tuck", "fold"])

    def test_the_dry_run_goes_out_follows_and_folds(self):
        with tempfile.TemporaryDirectory() as tmp:
            arm = holster_dog.SimArm()
            with mock.patch.object(holster_dog, "ZERO_CIRCLE_S", 6.0), mock.patch.object(holster_dog.time, "sleep"):
                moves = holster_dog.zero_plan()
                done = holster_dog.Runner(arm, False, False, Path(tmp), moves).run(moves)["moves"]
        self.assertEqual([d["move"] for d in done], self.names)
        circle = done[self.names.index("circle")]
        self.assertLess(max(circle["off_goal_max_deg"]), 8.0)
        self.assertLess(np.abs(arm.latest()[0] - self.fold).max(), holster_dog.ARRIVED_DEG)


class DemoDrawRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.moves = holster_dog.demo_plan()
        self.names = [m["name"] for m in self.moves]
        self.played = {}
        where = np.asarray(d1_hardware.REST_SERVO_DEG[:6], dtype=float)
        for m in self.moves:
            self.played[m["name"]] = holster_dog.move_path(m, where, self.played)
            where = self.played[m["name"]][-1]

    def stopped_in(self, name):
        k = self.names.index(name)
        path = self.played[name]
        return holster_dog.recovery(self.moves, self.played, k, path[len(path) // 2])

    def test_stopped_in_the_draw_it_retraces_with_the_jaws_as_they_were(self):
        got = self.stopped_in("draw")
        self.assertEqual([m["name"] for m in got],
                         ["undo draw", "undo close", "undo on", "undo lower", "undo reach", "undo turn"])
        self.assertGreater(got[1]["gripper"], 0)  # the jaws opened once the key is back at its start
        self.assertLess(got[0]["gripper"], 0)     # and shut on it until then

    def test_stopped_while_clear_of_the_mouth_it_retraces_through_the_mouth(self):
        got = [m["name"] for m in self.stopped_in("unclear")]
        self.assertEqual(got[:6], ["undo unclear", "undo clear", "undo hold", "undo exit", "undo draw", "undo close"])

    def test_stopped_on_the_way_back_it_finishes_the_way_back(self):
        got = [m["name"] for m in self.stopped_in("back")]
        self.assertEqual(got, ["back", "open", "off", "raise", "unreach", "fold"])

    def test_every_recovery_ends_at_the_start(self):
        start = self.played["turn"][0]
        for name in self.names:
            last = self.stopped_in(name)[-1]
            end = holster_dog.move_path(last, last.get("points", [start])[0], self.played)[-1]
            self.assertLess(np.abs(end - start).max(), 1e-6, name)


class StallingArm:
    """A D1 that behaves as F-238 says: reports every 111 ms, unless a command lands in the window before one."""

    report_gate = True

    def __init__(self):
        self.q = list(d1_hardware.REST_SERVO_DEG[:6]) + [8.0]
        self.next_report = time.monotonic() + d1_hardware.REPORT_PERIOD_S
        self.report = None
        self.sent = []
        self._last_rx = 0.0

    def wait_for_feedback(self, timeout_s=5.0):
        return list(self.q)

    def poll(self, timeout_s=0.0):
        time.sleep(min(timeout_s, 0.001))
        now = time.monotonic()
        if now >= self.next_report:
            self.report = self._last_rx = now
            self.next_report = now + d1_hardware.REPORT_PERIOD_S
            return True
        return False

    @property
    def servo_report_s(self):
        return self.report

    def get_joint_angles(self):
        return list(self.q[:6])

    def get_gripper_units(self):
        return self.q[6]

    def set_all_joint_angles(self, angles, mode=0, gripper=None):
        now = time.monotonic()
        self.sent.append(list(angles))
        self.q[:6] = list(angles)
        if self.report is not None and 0.098 <= now - self.report <= 0.106:
            self.next_report = now + 0.105  # the report it was about to send is gone


class LaggingArm(StallingArm):
    """The real arm takes time to get there: it was still 3.5 deg out after a swing's return (2026-11-06)."""

    LAG_S = 0.3

    def __init__(self):
        super().__init__()
        self.pending = []

    def poll(self, timeout_s=0.0):
        now = time.monotonic()
        while self.pending and self.pending[0][0] <= now:
            self.q[:6] = self.pending.pop(0)[1]
        return super().poll(timeout_s)

    def set_all_joint_angles(self, angles, mode=0, gripper=None):
        self.sent.append(list(angles))
        self.pending.append((time.monotonic() + self.LAG_S, list(angles)))


class RateTestDryRuns(unittest.TestCase):
    def test_the_gate_is_off_for_the_stall_blocks_and_on_for_the_gated(self):
        seen = []
        arm = StallingArm()
        original = arm.set_all_joint_angles
        arm.set_all_joint_angles = lambda *a, **k: (seen.append(arm.report_gate), original(*a, **k))[1]
        self.run_blocks((("plain", 10.0, 1), ("gated", 10.0, 1, None, 0.0, True)), block_s=0.4, arm=arm)
        self.assertEqual((seen[0], seen[-1]), (False, True))

    def run_blocks(self, blocks, block_s=1.0, arm=None, between_s=0.05):
        arm = arm or StallingArm()
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(rate_test, "BETWEEN_S", between_s), \
                mock.patch.object(d1_hardware, "D1Client", lambda: arm), \
                mock.patch("demos.combiner_door.robot.arm_link.bridge_status", lambda: None):
            self.assertEqual(rate_test.run(Path(tmp), block_s, blocks), 0)
            return arm, json.loads((Path(tmp) / "rate_test.json").read_text())["blocks"]

    def test_the_locked_blocks_send_a_command_a_report_and_the_control_stalls(self):
        _, blocks = self.run_blocks((("lock_20ms", 9.0, 1, 20.0), ("control", 9.0, 1, 103.0)), block_s=1.5)
        locked, control = (np.diff(b["reports"]) for b in blocks)
        self.assertLess(locked.max(), 0.14)
        self.assertAlmostEqual(len(blocks[0]["commands"]), len(blocks[0]["reports"]), delta=2)
        self.assertGreater(control.max(), 0.16)

    def test_a_swing_records_what_it_sent_and_returns_to_the_middle(self):
        arm, blocks = self.run_blocks((("swing", 9.0, 1, 20.0, 10.0),))
        self.assertEqual(len(blocks[0]["sent"]), len(blocks[0]["commands"]))
        self.assertGreater(max(abs(v - d1_hardware.REST_SERVO_DEG[0]) for v in blocks[0]["sent"]), 5.0)
        self.assertEqual(arm.sent[-1], list(d1_hardware.REST_SERVO_DEG[:6]))
        self.assertTrue(blocks[0]["joint1"])

    def test_the_arm_still_returning_from_a_swing_does_not_stop_the_test(self):
        # Ending 0.3 s (the arm's lag) after the swing's peak leaves it ~10 deg out when the return is sent.
        swing = ("swing", 9.0, 1, 20.0, 10.0)
        peak_s = 10.0 / rate_test.SWING_DEG_S
        _, blocks = self.run_blocks((swing, swing), block_s=peak_s + LaggingArm.LAG_S, arm=LaggingArm(), between_s=0.25)
        self.assertEqual(len(blocks), 2)

    def test_a_held_arm_that_moves_stops_the_test(self):
        arm = StallingArm()
        arm.get_joint_angles = lambda: [v + 5.0 for v in arm.q[:6]]
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(rate_test, "BETWEEN_S", 0.05), \
                mock.patch.object(d1_hardware, "D1Client", lambda: arm), \
                mock.patch("demos.combiner_door.robot.arm_link.bridge_status", lambda: None):
            self.assertEqual(rate_test.run(Path(tmp), 0.5, (("held", 9.0, 1, 20.0),)), 1)


if __name__ == "__main__":
    unittest.main()

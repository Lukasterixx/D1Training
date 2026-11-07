"""The arm half of the reach (`demos/combiner_key/robot/reach_arm.py`): against the model the policy trained on, and the real arm."""
import json
import math
import struct
import unittest
from pathlib import Path

import numpy as np
import torch

import d1_hardware
import d1_ik
from demos.combiner_key.robot import hand_error, leg_policy, lowlevel, reach_arm
from position_only.workspace import load_urdf
from unifp_isaaclab import interface
from unifp_train import task_cfg
from unifp_train.arm_link import ArmLink

DATA = Path(__file__).resolve().parent / "data"
REPO = Path(__file__).resolve().parent.parent
DEPLOYED = REPO / "checkpoints/reach_force_sticky_model_66600.npz"
DT = 0.02


def targets():
    limits = np.array([[-2.0, 2.0]] * 6)
    return reach_arm.ArmTargets(interface.DEFAULT_DOF_POS[12:18], interface.ACTION_SCALE, interface.CLIP_ACTIONS,
                                limits[:, 0], limits[:, 1])


def reports_at(q, stamp=1.0):
    r = reach_arm.ArmReports(np.zeros(6))
    r.report(stamp, q)
    return r


class TestTheWire(unittest.TestCase):
    def test_joint1_and_joint4_are_flipped_and_no_others(self):
        wire = reach_arm.to_servo_deg(np.radians([10.0, 10.0, 10.0, 10.0, 10.0, 10.0]))
        np.testing.assert_allclose(wire, [-10.0, 10.0, 10.0, -10.0, 10.0, 10.0])

    def test_it_agrees_with_d1_ik_both_ways(self):
        q = np.array([0.3, -0.7, 1.1, -0.2, 0.5, -1.4])
        np.testing.assert_allclose(reach_arm.to_servo_deg(q), d1_ik.to_servo_deg(q))
        np.testing.assert_allclose(reach_arm.from_servo_deg(d1_ik.to_servo_deg(q)), q)
        np.testing.assert_allclose(np.ravel(d1_ik.from_servo_deg(reach_arm.to_servo_deg(q))), q)


class TestTheWindow(unittest.TestCase):
    def test_it_is_the_tested_window(self):
        self.assertTrue(reach_arm.in_window(0.020))
        self.assertTrue(reach_arm.in_window(0.080))

    def test_it_stays_clear_of_where_the_arm_drops_commands(self):
        self.assertFalse(any(reach_arm.in_window(t / 1000) for t in range(98, 107)))
        self.assertFalse(reach_arm.in_window(0.005))


class TestTargets(unittest.TestCase):
    def test_it_is_the_environments_arithmetic(self):
        action = torch.tensor([[1.0] * 12 + [0.5, -1.0, 2.0, -3.0, 4.0, 0.0]])
        expected = interface.joint_targets(action)[0, 12:18].numpy()
        np.testing.assert_allclose(targets().target(action[0, 12:].numpy()), expected, atol=1e-6)

    def test_past_the_clip_it_is_the_clip(self):
        self.assertAlmostEqual(reach_arm.ArmTargets(np.zeros(6), 0.25, 100.0, [-99.0] * 6, [99.0] * 6).target([1e6] * 6)[0], 25.0)

    def test_it_is_clamped_to_the_limits(self):
        np.testing.assert_allclose(targets().target([100.0] * 6), [2.0] * 6)
        np.testing.assert_allclose(targets().target([-100.0] * 6), [-2.0] * 6)

    def test_limits_out_of_order_are_refused(self):
        with self.assertRaises(ValueError):
            reach_arm.ArmTargets(np.zeros(6), 0.25, 100.0, [1.0] * 6, [-1.0] * 6)


class TestReportsAgainstTheLink(unittest.TestCase):
    """The same reports through `ArmLink.observe`, what the policy trained on, and through `ArmReports`, what the dog would run."""

    def test_they_read_the_same(self):
        rng = np.random.default_rng(3)
        link = ArmLink(1, "cpu", DT, feedback_hz=9.0, quantum_deg=0.0, seed=5)
        link.reset(None, torch.zeros(1, 6))
        ours = reach_arm.ArmReports(np.zeros(6))
        q = np.zeros(6)
        compared = 0
        for k in range(600):
            link.clock += DT
            q = q + rng.normal(0.0, 0.01, 6)
            before = float(link.last_report[0])
            seen_q, seen_dq = link.observe(torch.as_tensor(q, dtype=torch.float32).unsqueeze(0), torch.zeros(1, 6))
            stamp = float(link.last_report[0])
            if stamp != before:
                ours.report(stamp, seen_q[0].numpy().astype(float))
                compared += 1
                np.testing.assert_allclose(ours.q, seen_q[0].numpy(), atol=1e-6)
                if ours.count > 1:
                    np.testing.assert_allclose(ours.dq, seen_dq[0].numpy(), rtol=2e-3, atol=2e-3)
        self.assertGreater(compared, 50)

    def test_the_first_report_has_no_speed(self):
        r = reports_at(np.ones(6))
        np.testing.assert_array_equal(r.dq, np.zeros(6))

    def test_held_between_reports_and_a_repeat_changes_nothing(self):
        r = reports_at(np.zeros(6), 1.0)
        r.report(1.111, np.full(6, 0.111))
        np.testing.assert_allclose(r.dq, np.ones(6))
        self.assertFalse(r.report(1.111, np.full(6, 5.0)))
        self.assertFalse(r.report(1.0, np.full(6, 5.0)))
        np.testing.assert_allclose(r.q, np.full(6, 0.111))
        np.testing.assert_allclose(r.dq, np.ones(6))

    def test_a_skip_is_differenced_over_the_real_interval(self):
        r = reports_at(np.zeros(6), 1.0)
        r.report(1.222, np.full(6, 0.222))
        np.testing.assert_allclose(r.dq, np.ones(6))


class TestArrivalStamps(unittest.TestCase):
    def test_reports_delivered_together_do_not_read_as_a_huge_speed(self):
        r = reports_at(np.zeros(6), 1.0)
        r.report(1.001, np.full(6, 0.02))
        np.testing.assert_allclose(r.dq, np.ones(6))           # over the 20 ms floor, as the link: not 20 rad/s

    def test_the_floor_is_the_links(self):
        link = ArmLink(1, "cpu", DT, feedback_hz=9.0)
        link.reset(None, torch.zeros(1, 6))
        self.assertEqual(reach_arm.MIN_REPORT_INTERVAL_S, link.policy_dt)


class TestTheFold(unittest.TestCase):
    """The real fold is at or past the training limits: J2 -1.5865, J3 +1.6005 at rest, against +-1.57."""

    FOLD = reach_arm.from_servo_deg(d1_hardware.REST_SERVO_DEG)

    def limits(self):
        return np.array(task_cfg.JOINT_POS_LIMITS[12:18])

    def holding(self):
        lim = self.limits()
        arm = reach_arm.ArmTargets(interface.DEFAULT_DOF_POS[12:18], interface.ACTION_SCALE, interface.CLIP_ACTIONS, lim[:, 0], lim[:, 1])
        return arm.target(self.FOLD / interface.ACTION_SCALE)

    def test_the_clamp_cannot_reach_it(self):
        self.assertGreater(float(np.abs(self.holding() - self.FOLD).max()), 0.02)

    def test_holding_at_the_fold_is_not_counted_as_lead(self):
        watch = reach_arm.ArmWatch()
        self.assertIsNone(watch.check(1.05, reports_at(self.FOLD), self.holding()))

    def test_a_report_past_the_limits_is_passed_on_as_it_came(self):
        self.assertEqual(reports_at(self.FOLD).q[2], self.FOLD[2])


class TestWatch(unittest.TestCase):
    def test_a_normal_step_passes(self):
        self.assertIsNone(reach_arm.ArmWatch().check(1.05, reports_at(np.zeros(6)), np.radians([5.0] * 6)))

    def test_no_report_yet_stops_it(self):
        self.assertIn("no report", reach_arm.ArmWatch().check(1.0, reach_arm.ArmReports(np.zeros(6)), np.zeros(6)))

    def test_an_arm_gone_quiet_stops_it(self):
        self.assertIn("no report for", reach_arm.ArmWatch().check(1.4, reports_at(np.zeros(6)), np.zeros(6)))

    def test_a_target_far_past_the_arm_stops_it_at_once_and_says_which_joint(self):
        target = np.zeros(6)
        target[4] = math.radians(70.0)
        self.assertIn("joint 5", reach_arm.ArmWatch().check(1.05, reports_at(np.zeros(6)), target))

    def test_a_lead_past_the_limit_stops_it_only_if_it_stays(self):
        watch, target = reach_arm.ArmWatch(max_step_deg=1e9), np.zeros(6)
        target[3] = math.radians(35.0)
        r = reach_arm.ArmReports(np.zeros(6))
        for k in range(50):
            now = 1.0 + 0.02 * k
            r.report(now - 0.01, np.zeros(6))
            why = watch.check(now, r, target)
            if why:
                break
        self.assertIn("joint 4", why)
        self.assertAlmostEqual(0.02 * k, reach_arm.LEAD_HOLD_S, delta=0.03)

    def test_a_lead_that_comes_back_starts_the_count_again(self):
        watch, r = reach_arm.ArmWatch(max_step_deg=1e9), reach_arm.ArmReports(np.zeros(6))
        far, near = np.radians([35.0] + [0.0] * 5), np.zeros(6)
        for k in range(60):
            now = 1.0 + 0.02 * k
            r.report(now - 0.01, np.zeros(6))
            self.assertIsNone(watch.check(now, r, far if k % 20 < 15 else near))

    def test_a_big_step_stops_it(self):
        watch, r = reach_arm.ArmWatch(), reports_at(np.radians([10.0] * 6))
        self.assertIsNone(watch.check(1.05, r, np.radians([-5.0] * 6)))
        self.assertIn("stepped", watch.check(1.06, r, np.radians([20.0] * 6)))

    def test_nan_stops_it(self):
        self.assertIn("not finite", reach_arm.ArmWatch().check(1.05, reports_at(np.zeros(6)), np.full(6, np.nan)))

    def test_what_the_policy_asks_is_a_soft_stop_and_a_dead_feed_is_not(self):
        lead, gap, bad = reach_arm.ArmWatch(), reach_arm.ArmWatch(), reach_arm.ArmWatch()
        lead.check(1.05, reports_at(np.zeros(6)), np.radians([70.0] * 6))
        gap.check(1.4, reports_at(np.zeros(6)), np.zeros(6))
        bad.check(1.05, reports_at(np.zeros(6)), np.full(6, np.nan))
        self.assertTrue(lead.soft)
        self.assertFalse(gap.soft)
        self.assertFalse(bad.soft)

    def test_stopped_stays_stopped_until_reset(self):
        watch, r = reach_arm.ArmWatch(), reports_at(np.zeros(6))
        watch.check(1.05, r, np.full(6, np.nan))
        self.assertIsNotNone(watch.check(1.05, r, np.zeros(6)))
        watch.reset()
        self.assertIsNone(watch.check(1.05, r, np.zeros(6)))


class TestGrace(unittest.TestCase):
    """The policy's first second, on an empty history: the Follower limits the arm then, not these checks."""

    def begun(self):
        watch = reach_arm.ArmWatch()
        watch.begin(1.0)
        return watch

    def test_the_handover_transient_passes(self):
        self.assertIsNone(self.begun().check(1.05, reports_at(np.zeros(6)), np.radians([64.0] * 6)))

    def test_it_ends(self):
        watch, r = self.begun(), reports_at(np.zeros(6), 2.0)
        self.assertIn("ahead", watch.check(2.05, r, np.radians([64.0] * 6)))

    def test_a_step_right_after_it_is_measured_from_the_last_target(self):
        watch, r = self.begun(), reports_at(np.radians([60.0] * 6), 2.0)
        self.assertIsNone(watch.check(1.9, r, np.radians([60.0] * 6)))
        self.assertIn("stepped", watch.check(2.05, r, np.radians([38.0] * 6)))

    def test_a_dead_feed_stops_it_inside_the_grace(self):
        self.assertIn("no report for", self.begun().check(1.4, reports_at(np.zeros(6), 1.0), np.zeros(6)))

    def test_a_bad_number_stops_it_inside_the_grace(self):
        self.assertIn("not finite", self.begun().check(1.05, reports_at(np.zeros(6)), np.full(6, np.inf)))

    def test_a_step_of_exactly_the_limit_passes(self):
        watch, r = self.begun(), reports_at(np.radians([60.0] * 6), 2.0)
        watch.check(1.9, r, np.radians([60.0] * 6))
        self.assertIsNone(watch.check(2.05, r, np.radians([60.0 - reach_arm.MAX_STEP_DEG + 0.01] * 6)))

    def test_with_no_begin_there_is_no_grace(self):
        self.assertIn("ahead", reach_arm.ArmWatch().check(1.05, reports_at(np.zeros(6)), np.radians([64.0] * 6)))

    def test_a_policy_running_away_is_stopped_the_tick_the_grace_ends(self):
        watch, r = self.begun(), reports_at(np.zeros(6), 2.0)
        self.assertIn("ahead", watch.check(2.05, r, np.radians([1400.0] * 6)))


class TestOnTheRealArmsRuns(unittest.TestCase):
    """The stops have to leave a good run alone, or the first thing they do on the dog is end it."""

    def replay(self, tc, cmd, tr, rep):
        """(seconds, URDF-order deg, per command) and (seconds, deg, per report) through the watch, on the dog's order of events."""
        watch, reports = reach_arm.ArmWatch(), reach_arm.ArmReports(np.radians(rep[0]))
        i, stops, gaps = 0, [], []
        for t, target in zip(tc, cmd):
            while i < len(tr) and tr[i] <= t:
                if reports.stamp is not None:
                    gaps.append(tr[i] - reports.stamp)
                reports.report(tr[i], np.radians(rep[i]))
                i += 1
            if reports.stamp is None:
                continue
            why = watch.check(t, reports, np.radians(target))
            if why:
                stops.append((t, why))
                watch.reset()
        return stops, gaps

    def test_the_six_joint_draw_with_the_key_passes(self):
        d = json.loads((DATA / "d1_locked_draw.json").read_text())["draw"]
        cmd = np.array([d[f"J{j}"]["cmd_deg"] for j in range(1, 7)]).T
        rep = np.array([d[f"J{j}"]["rep_deg"] for j in range(1, 7)]).T
        stops, gaps = self.replay(np.array(d["tc_ms"]) / 1000, cmd, np.array(d["tr_ms"]) / 1000, rep)
        self.assertEqual(stops, [])
        self.assertGreater(max(gaps), 0.2)                     # the skip is in there, and it does not trip the watch

    def test_the_swing_passes_including_its_skip(self):
        blocks = json.loads((DATA / "d1_locked_swing.json").read_text())["blocks"]
        for name, b in blocks.items():
            pad = lambda v: np.c_[np.array(v), np.zeros((len(v), 5))]
            stops, _ = self.replay(np.array(b["t_cmd_ms"]) / 1000, pad(b["cmd_deg"]), np.array(b["t_rep_ms"]) / 1000, pad(b["rep_deg"]))
            self.assertEqual(stops, [], name)

    def test_every_real_command_was_inside_the_window(self):
        d = json.loads((DATA / "d1_locked_draw.json").read_text())["draw"]
        tc, tr = np.array(d["tc_ms"]) / 1000, np.array(d["tr_ms"]) / 1000
        since = [t - tr[tr <= t][-1] for t in tc if (tr <= t).any()]
        inside = float(np.mean([reach_arm.in_window(s) for s in since]))
        self.assertGreater(inside, 0.97)


class ThePressForce(unittest.TestCase):
    def test_nothing_outside_its_window(self):
        f = reach_arm.PressForce([4.0], 10.0, 20.0)
        self.assertEqual([f.at(t) for t in (0.0, 10.0, 20.0, 25.0)], [0.0, 0.0, 0.0, 0.0])

    def test_it_ramps_on_and_off(self):
        f = reach_arm.PressForce([4.0], 10.0, 20.0)
        self.assertAlmostEqual(f.at(10.5), 2.0)
        self.assertAlmostEqual(f.at(15.0), 4.0)
        self.assertAlmostEqual(f.at(19.5), 2.0)

    def test_levels_share_the_time_and_ramp_between(self):
        f = reach_arm.PressForce([0.0, 2.0, 4.0, 6.0], 0.0, 20.0)
        self.assertEqual(f.at(4.0), 0.0)
        self.assertAlmostEqual(f.at(5.5), 1.0)
        self.assertAlmostEqual(f.at(9.0), 2.0)
        self.assertAlmostEqual(f.at(17.0), 6.0)

    def test_it_refuses_what_the_policy_never_trained_on(self):
        for levels in ([8.5], [-1.0], []):
            with self.assertRaises(ValueError):
                reach_arm.PressForce(levels, 0.0, 20.0)
        with self.assertRaises(ValueError):
            reach_arm.PressForce([1.0, 2.0, 3.0], 0.0, 5.0)

    def test_the_slot_is_trainings_and_the_ceiling_too(self):
        self.assertEqual(interface.CMD_EE_FORCE.stop - 1, leg_policy.CMD_EE_FORCE_Z)
        self.assertEqual(task_cfg.GRIPPER_FORCE_RANGE_N[1], reach_arm.MAX_FORCE_N)

    def test_the_force_is_down_and_in_the_right_slot(self):
        meta = {"num_commands": 15, "cmd_ee_radius": 3}
        commands = leg_policy.reach_command(meta, (0.5, 0.1, 0.2), (0.1, 0.0, 0.0), 4.0)
        self.assertEqual(commands[leg_policy.CMD_EE_FORCE_Z], -4.0)
        self.assertEqual(float(np.abs(commands[6:11]).sum()), 0.0)


class TheJaws(unittest.TestCase):
    def test_the_units_are_the_arms(self):
        import d1_hardware
        self.assertEqual((reach_arm.JAWS_OPEN_UNITS, reach_arm.JAWS_SHUT_UNITS), (d1_hardware.GRIPPER_UNITS_OPEN, d1_hardware.GRIPPER_UNITS_CLOSED))

    def test_closing_runs_from_open_to_shut_and_stays(self):
        self.assertEqual(reach_arm.jaws_closing(0.0), reach_arm.JAWS_OPEN_UNITS)
        self.assertAlmostEqual(reach_arm.jaws_closing(reach_arm.CLOTH_CLOSE_S), reach_arm.JAWS_SHUT_UNITS)
        self.assertAlmostEqual(reach_arm.jaws_closing(100.0), reach_arm.JAWS_SHUT_UNITS)

    def test_a_hold_carries_no_gripper_and_the_rest_do(self):
        sent = []
        sock = type("Sock", (), {"sendall": lambda self, b: sent.append(json.loads(b))})()
        arm = reach_arm.ArmSender(sock)
        arm.gripper = -19.8
        arm.follow(np.zeros(6))
        arm.keepalive()
        arm.path([[0.0] * 6, [1.0] * 6])
        arm.hold()
        self.assertEqual([m.get("gripper") for m in sent], [-19.8, -19.8, -19.8, None])


class TheGoalPath(unittest.TestCase):
    START = (0.5, math.pi / 5, 0.0)

    def path(self, *via, settle=3.0):
        return reach_arm.GoalPath(self.START, via, settle)

    def test_the_ranges_are_trainings(self):
        self.assertEqual(reach_arm.GOAL_RADIUS_M, interface.EE_GOAL_RADIUS_RANGE)
        self.assertEqual(reach_arm.GOAL_PITCH_RAD, interface.EE_GOAL_PITCH_RANGE)
        self.assertEqual(reach_arm.GOAL_YAW_RAD, interface.EE_GOAL_YAW_RANGE)

    def test_it_holds_slides_holds_and_comes_back(self):
        far = (0.45, 0.0, 0.3)
        p = self.path((far, 3.0, 4.0))
        self.assertEqual(p.at(0.0).tolist(), list(self.START))
        self.assertEqual(p.at(2.99).tolist(), list(self.START))
        np.testing.assert_allclose(p.at(4.5), (np.array(self.START) + np.array(far)) / 2)    # in spherical coordinates, as `EeGoalTrajectory`
        np.testing.assert_allclose(p.at(6.0), far)
        np.testing.assert_allclose(p.at(9.99), far)
        np.testing.assert_allclose(p.at(13.0), self.START)
        np.testing.assert_allclose(p.at(100.0), self.START)
        self.assertEqual(p.duration, 3.0 + 3.0 + 4.0 + 3.0 + reach_arm.BACK_HOLD_S)

    def test_a_goal_never_jumps(self):
        p = self.path(((0.35, 1.0, 1.2), 2.0, 1.0), ((0.55, -0.7, -1.2), 2.0, 1.0))
        goals = np.array([p.at(k * 0.02) for k in range(int(p.duration / 0.02) + 10)])
        self.assertLess(np.abs(np.diff(goals, axis=0)).max(), 0.03)

    def test_outside_what_it_trained_on_is_refused(self):
        for bad in ((0.25, 0.0, 0.0), (0.6, 0.0, 0.0), (0.5, 1.1, 0.0), (0.5, -0.8, 0.0), (0.5, 0.0, 1.3)):
            with self.assertRaises(ValueError):
                self.path((bad, 3.0, 1.0))

    def test_the_keep_out_is_trainings(self):
        self.assertEqual(reach_arm.GOAL_KEEP_OUT_LOWER, interface.EE_GOAL_COLLISION_LOWER)
        self.assertEqual(reach_arm.GOAL_KEEP_OUT_UPPER, interface.EE_GOAL_COLLISION_UPPER)
        self.assertEqual(reach_arm.GOAL_UNDERGROUND_M, interface.EE_GOAL_UNDERGROUND_LIMIT)
        self.assertEqual(reach_arm.GOAL_CHECK_SAMPLES, interface.EE_GOAL_COLLISION_SAMPLES)

    def test_a_goal_through_the_dog_or_the_floor_is_refused(self):
        for bad in ((0.3, -0.78, 0.7), (0.58, -0.78, 0.0)):    # inside the keep-out box, 0.16 m ahead and 0.21 under the centre; under the floor's limit
            with self.assertRaises(ValueError):
                self.path((bad, 3.0, 1.0))
        self.assertTrue(reach_arm.goal_is_clear(self.START, (0.45, 0.0, 0.0)))

    def test_every_goal_offered_is_clear(self):
        policies = json.loads((REPO / "demos/robot_console/policies.json").read_text())
        modes = next(p for p in policies if p["id"] == "reach_66600")["modes"]
        seen = 0
        for words in modes.values():
            now = self.START
            for k, word in enumerate(words):
                if word == "--reach_via":
                    r, p, y = float(words[k + 1]), math.radians(float(words[k + 2])), math.radians(float(words[k + 3]))
                    self.assertTrue(reach_arm.goal_is_clear(now, (r, p, y)), words)
                    now = (r, p, y)
                    seen += 1
            self.assertTrue(reach_arm.goal_is_clear(now, self.START))
        self.assertGreaterEqual(seen, 7)

    def test_a_slide_faster_than_the_floor_is_refused(self):
        with self.assertRaises(ValueError):
            self.path(((0.45, 0.0, 0.0), 1.0, 1.0))


class TheSticks(unittest.TestCase):
    def test_each_way_round(self):
        w = reach_arm.WalkSticks(0.3, 0.2, 0.5)
        v = lambda **k: w.velocity({"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, **k})   # noqa: E731
        self.assertEqual(v(ly=1.0), (0.3, 0.0, 0.0))             # up: forward
        self.assertEqual(v(ly=-1.0), (-0.3, 0.0, 0.0))
        self.assertEqual(v(lx=1.0), (0.0, -0.2, 0.0))            # right: -y in the base frame
        self.assertEqual(v(rx=1.0), (0.0, 0.0, -0.5))            # right: clockwise, -z
        self.assertEqual(v(rx=-1.0), (0.0, 0.0, 0.5))

    def test_a_resting_stick_asks_nothing_and_a_pushed_one_walks(self):
        w = reach_arm.WalkSticks(0.3, 0.2, 0.5)
        self.assertEqual(w.velocity({"lx": 0.19, "ly": -0.19, "rx": 0.19, "ry": 1.0}), (0.0, 0.0, 0.0))
        for k, name in enumerate(("ly", "lx", "rx")):
            for size in (0.2, 0.21, 0.5, 0.99):
                sticks = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, name: size}
                self.assertGreater(abs(w.velocity(sticks)[k]), leg_policy.WALK_DEAD_ZONE[k])      # never told to creep

    def test_the_gaits_least_is_the_clocks(self):
        for least, dead in zip(reach_arm.WALK_LEAST, leg_policy.WALK_DEAD_ZONE):
            self.assertGreater(least, dead)
        with self.assertRaises(ValueError):
            reach_arm.WalkSticks(0.05, 0.2, 0.5)

    def test_the_bytes(self):
        raw = bytearray(40)
        raw[4:24] = struct.pack("<5f", 0.25, -0.5, 0.75, 1.0, -1.0)      # lx, rx, ry, L2, ly
        self.assertEqual(lowlevel.remote_sticks(raw), {"lx": 0.25, "ly": -1.0, "rx": -0.5, "ry": 0.75})

    def test_bad_bytes_stand_still(self):
        raw = bytearray(40)
        raw[4:24] = struct.pack("<5f", math.nan, math.inf, 9.0, 0.0, -9.0)
        self.assertEqual(lowlevel.remote_sticks(raw), {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0})
        self.assertEqual(lowlevel.remote_sticks(bytearray(3)), {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0})

    def test_a_thumb_resting_at_the_edge_does_not_flicker(self):
        w = reach_arm.WalkSticks(0.3, 0.2, 0.5)
        g = np.random.default_rng(1)
        on = [bool(w.velocity({"lx": 0.0, "ly": 0.2 + float(v), "rx": 0.0, "ry": 0.0})[0]) for v in g.normal(0.0, 0.02, 500)]
        self.assertLessEqual(sum(a != b for a, b in zip(on, on[1:])), 1)
        self.assertEqual(w.velocity({"lx": 0.0, "ly": 0.1, "rx": 0.0, "ry": 0.0})[0], 0.0)       # let go, it stops
        self.assertEqual(w.velocity({"lx": 0.0, "ly": 0.15, "rx": 0.0, "ry": 0.0})[0], 0.0)      # and does not start again below the edge

    def test_it_never_asks_past_training(self):
        self.assertEqual(reach_arm.WALK_MOST, (task_cfg.LIN_VEL_X_RANGE[1], task_cfg.LIN_VEL_Y_RANGE[1], task_cfg.ANG_VEL_YAW_RANGE[1]))
        for most in ((0.7, 0.2, 0.5), (0.3, 0.5, 0.5), (0.3, 0.2, 0.7)):
            with self.assertRaises(ValueError):
                reach_arm.WalkSticks(*most)


class TheGaitClock(unittest.TestCase):
    """`leg_policy.gait_step` against the training's own."""

    def test_the_constants(self):
        self.assertEqual(leg_policy.GAIT_CYCLE_S, interface.CYCLE_TIME)
        self.assertEqual(leg_policy.WALK_DEAD_ZONE, (interface.LIN_VEL_X_CLIP, interface.LIN_VEL_Y_CLIP, interface.ANG_VEL_YAW_CLIP))

    def test_it_runs_as_training_runs_it(self):
        g = np.random.default_rng(3)
        ours, theirs = 0.0, torch.zeros(1)
        for k in range(600):
            v = (0.0, 0.0, 0.0) if (k // 40) % 3 == 0 else tuple(g.uniform(-0.3, 0.3, 3))
            ours = leg_policy.gait_step(ours, v, interface.POLICY_DT)
            commands = torch.zeros(1, 15)
            commands[0, :3] = torch.tensor(v)
            theirs = interface.gait_step(theirs, commands)
            self.assertAlmostEqual(ours, float(theirs), places=4)

    def test_the_frame_shows_the_phase_as_training_does(self):
        meta = leg_policy.LegPolicy(DEPLOYED).meta
        o = leg_policy.LegObservation(meta)
        o.gait_phase = 0.3
        frame = o.frame([1, 0, 0, 0], np.zeros(3), np.zeros(12), np.zeros(12), np.zeros(6), np.zeros(6), np.zeros(15))
        self.assertAlmostEqual(frame[59], math.sin(2 * math.pi * 0.3))
        self.assertAlmostEqual(frame[60], math.cos(2 * math.pi * 0.3))
        o.reset()
        self.assertEqual(o.gait_phase, 0.0)

    def test_the_velocity_is_where_the_policy_reads_it(self):
        meta = leg_policy.LegPolicy(DEPLOYED).meta
        c = leg_policy.reach_command(meta, (0.5, 0.6, 0.1), (0.3, -0.2, 0.4))
        self.assertEqual(c[[interface.CMD_LIN_VEL_X, interface.CMD_LIN_VEL_Y, interface.CMD_ANG_VEL_YAW]].tolist(), [0.3, -0.2, 0.4])
        self.assertEqual(c[interface.CMD_EE_RADIUS:interface.CMD_EE_YAW + 1].tolist(), [0.5, 0.6, 0.1])
        np.testing.assert_allclose(leg_policy.reach_command(meta, (0.5, 0.6, 0.1)), np.r_[0, 0, 0, c[3:]])


class TheHandSum(unittest.TestCase):
    """`hand_error.py` against numbers that were recorded, not derived: the mount counted twice read 8.4 cm too high and nobody's test said so (2026-11-07)."""

    def test_the_simulators_own_first_step(self):
        # `folded_start_probe`, `66600` at 1 N.m: arm at zero, pitch -1.3 deg, base 0.252 m: the simulator recorded 2.7 cm to the goal.
        joints = load_urdf()[0]
        hand = hand_error.hand_point(joints, np.radians([0.0, 0.0, 1.17, 0.0, 0.0, 0.0]), 0.252, 0.0, math.radians(-1.3))
        goal = hand_error.goal_point((0.5, math.pi / 5, 0.0))
        self.assertAlmostEqual(float(np.linalg.norm(hand - goal)) * 100, 2.7, delta=0.3)

    def test_the_goal_is_the_tasks(self):
        np.testing.assert_allclose(hand_error.GOAL_CENTRE_M, interface.EE_GOAL_CENTER_OFFSET)
        np.testing.assert_allclose(hand_error.goal_point((0.5, 0.0, 0.0)), [0.5, 0.0, 0.49])

    def test_a_real_run_reads_as_recorded(self):
        rows = REPO / "results/week_08/runs/20261106T140642_reach_stand_live/policy_rows.npz"
        if not rows.exists():
            self.skipTest("the run's frames")
        data = dict(np.load(rows))
        meta = leg_policy.LegPolicy(DEPLOYED).meta
        t, _, off = hand_error.offsets(data, meta)
        last = np.linalg.norm(off[t > t[-1] - 3], axis=1).mean() * 100
        self.assertAlmostEqual(last, 5.0, delta=0.2)                     # F-257's table, `handgoal.txt`


if __name__ == "__main__":
    unittest.main()

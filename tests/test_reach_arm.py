"""The arm half of the reach (`demos/combiner_key/robot/reach_arm.py`): against the model the policy trained on, and the real arm."""
import json
import math
import unittest
from pathlib import Path

import numpy as np
import torch

import d1_hardware
import d1_ik
from demos.combiner_key.robot import reach_arm
from unifp_isaaclab import interface
from unifp_train import task_cfg
from unifp_train.arm_link import ArmLink

DATA = Path(__file__).resolve().parent / "data"
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


if __name__ == "__main__":
    unittest.main()

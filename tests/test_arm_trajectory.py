"""The D1 firmware trajectory model: trapezoids toward each setpoint, lossy replanning, dead time."""
import math
import unittest

import torch

from position_only.core import TrapezoidTracker

DT = 0.005  # physics step
A, B, V = 12.6, 17.3, 1.25  # fitted accel/decel (rad/s^2), a measured speed ceiling (rad/s)


def run(tracker, seconds):
    trace = []
    for _ in range(int(round(seconds / DT))):
        trace.append((tracker.step(DT).clone(), tracker.velocity.clone()))
    return torch.stack([p for p, _ in trace]), torch.stack([v for _, v in trace])


def single(dim=1, envs=1, **kw):
    tracker = TrapezoidTracker(envs, dim, A, B, V, device="cpu", **kw)
    tracker.reset(None, torch.zeros(envs, dim))
    return tracker


class SingleStep(unittest.TestCase):
    def test_a_30_degree_step_follows_the_analytic_trapezoid(self):
        tracker = single()
        goal = math.radians(30.0)
        tracker.command(torch.tensor([True]), torch.tensor([[goal]]))
        pos, vel = run(tracker, 1.5)
        pos, vel = pos[:, 0, 0], vel[:, 0, 0]
        la, lb = V * V / (2 * A), V * V / (2 * B)
        expected = V / A + (goal - la - lb) / V + V / B
        arrived = int(torch.nonzero((pos - goal).abs() < 1e-6)[0]) * DT + DT
        self.assertAlmostEqual(arrived, expected, delta=3 * DT)
        self.assertAlmostEqual(float(vel.max()), V, places=5)
        # Cruise is reached after v/a, give or take a physics step.
        first_cruise = int(torch.nonzero(vel >= V - 1e-6)[0]) * DT + DT
        self.assertAlmostEqual(first_cruise, V / A, delta=2 * DT)
        self.assertLessEqual(float(pos.max()), goal + 1e-6)

    def test_it_parks_on_the_goal_without_chattering(self):
        tracker = single()
        tracker.command(torch.tensor([True]), torch.tensor([[0.3]]))
        pos, vel = run(tracker, 2.0)
        tail_pos, tail_vel = pos[-100:, 0, 0], vel[-100:, 0, 0]
        self.assertTrue(torch.all(tail_pos == tail_pos[0]))
        self.assertAlmostEqual(float(tail_pos[0]), 0.3, places=6)
        self.assertTrue(torch.all(tail_vel == 0.0))

    def test_a_short_move_is_triangular_and_does_not_overshoot(self):
        tracker = single()
        goal = math.radians(3.0)
        tracker.command(torch.tensor([True]), torch.tensor([[goal]]))
        pos, vel = run(tracker, 0.6)
        self.assertLess(float(vel.max()), V)
        self.assertLessEqual(float(pos.max()), goal + 1e-6)
        self.assertAlmostEqual(float(pos[-1, 0, 0]), goal, places=6)

    def test_a_negative_move_is_the_mirror_image(self):
        up, down = single(), single()
        up.command(torch.tensor([True]), torch.tensor([[0.5]]))
        down.command(torch.tensor([True]), torch.tensor([[-0.5]]))
        pu, _ = run(up, 1.2)
        pd, _ = run(down, 1.2)
        self.assertTrue(torch.allclose(pu, -pd, atol=1e-6))


class Replanning(unittest.TestCase):
    """F-035: streaming a new setpoint each cycle lost speed; restarting from rest reproduces it."""

    def cruise_then_replan(self, retention):
        tracker = single(retention=retention)
        tracker.command(torch.tensor([True]), torch.tensor([[2.0]]))
        run(tracker, 0.3)                       # well into cruise
        self.assertAlmostEqual(float(tracker.velocity), V, places=5)
        tracker.command(torch.tensor([True]), torch.tensor([[3.0]]))
        return float(tracker.velocity)

    def test_retention_zero_restarts_the_plan_from_rest(self):
        self.assertEqual(self.cruise_then_replan(0.0), 0.0)

    def test_retention_one_keeps_the_planned_speed(self):
        self.assertAlmostEqual(self.cruise_then_replan(1.0), V, places=5)

    def test_a_reversal_brakes_then_returns_without_overshooting_the_new_goal(self):
        tracker = single(retention=1.0)
        tracker.command(torch.tensor([True]), torch.tensor([[2.0]]))
        run(tracker, 0.3)
        tracker.command(torch.tensor([True]), torch.tensor([[0.1]]))
        pos, vel = run(tracker, 2.0)
        self.assertAlmostEqual(float(pos[-1, 0, 0]), 0.1, places=6)
        self.assertGreaterEqual(float(pos[:, 0, 0].min()), 0.1 - 1e-6)


class DeadTime(unittest.TestCase):
    def test_nothing_moves_until_the_dead_time_has_passed(self):
        tracker = single(dead_time=0.040)
        tracker.command(torch.tensor([True]), torch.tensor([[0.5]]))
        pos, _ = run(tracker, 0.1)
        waited = pos[:8, 0, 0]
        self.assertTrue(torch.all(waited == 0.0))
        self.assertGreater(float(pos[10, 0, 0]), 0.0)

    def test_the_latest_pending_setpoint_wins(self):
        tracker = single(dead_time=0.040)
        tracker.command(torch.tensor([True]), torch.tensor([[0.5]]))
        run(tracker, 0.02)
        tracker.command(torch.tensor([True]), torch.tensor([[-0.5]]))
        pos, _ = run(tracker, 2.0)
        self.assertAlmostEqual(float(pos[-1, 0, 0]), -0.5, places=6)
        self.assertLessEqual(float(pos[:, 0, 0].max()), 0.0)


class Vectorised(unittest.TestCase):
    def test_environments_and_joints_are_independent(self):
        tracker = TrapezoidTracker(2, 3, A, B, [1.25, 1.29, 1.21], device="cpu")
        tracker.reset(None, torch.zeros(2, 3))
        target = torch.tensor([[0.4, -0.4, 0.0], [9.9, 9.9, 9.9]])
        tracker.command(torch.tensor([True, False]), target)
        pos, _ = run(tracker, 1.5)
        self.assertTrue(torch.allclose(pos[-1, 0], target[0]))
        self.assertTrue(torch.all(pos[:, 1] == 0.0))

    def test_per_joint_speed_ceilings_apply(self):
        tracker = TrapezoidTracker(1, 2, A, B, [1.21, 1.29], device="cpu")
        tracker.reset(None, torch.zeros(1, 2))
        tracker.command(torch.tensor([True]), torch.tensor([[2.0, 2.0]]))
        _, vel = run(tracker, 0.5)
        self.assertAlmostEqual(float(vel[:, 0, 0].max()), 1.21, places=5)
        self.assertAlmostEqual(float(vel[:, 0, 1].max()), 1.29, places=5)

    def test_reset_parks_at_rest_and_drops_pending_setpoints(self):
        tracker = single(dead_time=0.05)
        tracker.command(torch.tensor([True]), torch.tensor([[1.0]]))
        tracker.reset(None, torch.tensor([[0.2]]))
        pos, vel = run(tracker, 0.3)
        self.assertTrue(torch.all(pos == 0.2))
        self.assertTrue(torch.all(vel == 0.0))

    def test_invalid_parameters_are_refused(self):
        with self.assertRaises(ValueError):
            TrapezoidTracker(1, 1, 0.0, B, V)
        with self.assertRaises(ValueError):
            TrapezoidTracker(1, 1, A, B, V, retention=1.5)
        with self.assertRaises(ValueError):
            TrapezoidTracker(1, 1, A, B, V, dead_time=-0.01)


if __name__ == "__main__":
    unittest.main()


class FittedModel(unittest.TestCase):
    """The constants the simulator runs are the ones the committed hardware fit chose."""

    def test_motor_model_matches_the_committed_fit(self):
        import json
        from pathlib import Path

        import motor_model as mm

        fit = json.loads((Path(__file__).resolve().parent.parent
                          / "results/week_01/figures/d1_arm_response.json").read_text())["chosen"]
        self.assertAlmostEqual(mm.D1_COMMAND_DEAD_TIME_S, fit["dead_time_s"], places=6)
        self.assertAlmostEqual(mm.D1_PLAN_ACCEL_RAD_S2, fit["accel_rad_s2"], delta=0.05)
        self.assertAlmostEqual(mm.D1_PLAN_DECEL_RAD_S2, fit["decel_rad_s2"], delta=0.05)
        self.assertEqual(mm.D1_REPLAN_VELOCITY_RETENTION, fit["retention"])
        # The fit's own evidence that the old figures were artefacts: a 127 ms dead time fits far worse.
        rms = {round(row["dead_time_s"], 3): row["rms_deg"] for row in json.loads(
            (Path(__file__).resolve().parent.parent / "results/week_01/figures/d1_arm_response.json")
            .read_text())["dead_time_profile"]}
        self.assertGreater(rms[0.127], 5 * rms[0.01])

    def test_profiles(self):
        from motor_model import arm_trajectory

        self.assertIsNone(arm_trajectory("none"))
        plan = arm_trajectory("measured")
        self.assertEqual(set(plan), {"dead_time_s", "accel_rad_s2", "decel_rad_s2", "replan_velocity_retention"})
        with self.assertRaises(ValueError):
            arm_trajectory("estimated")


class ManifestCondition(unittest.TestCase):
    def test_a_planner_change_is_a_mismatch_and_a_float32_roundtrip_is_not(self):
        import numpy as np

        from position_only import manifest as mf

        manifest = mf.build("development", episodes=2)
        conditions = dict(manifest["conditions"])
        plan = dict(conditions["arm_trajectory"])
        conditions["arm_trajectory"] = dict(plan, velocity_limits_rad_s=[float(np.float32(v))
                                                                          for v in plan["velocity_limits_rad_s"]])
        self.assertEqual(mf.mismatches(manifest, conditions), [])
        conditions["arm_trajectory"] = None                          # the arm without a planner
        self.assertEqual(len(mf.mismatches(manifest, conditions)), 1)
        conditions["arm_trajectory"] = dict(plan, replan_velocity_retention=1.0)
        self.assertEqual(len(mf.mismatches(manifest, conditions)), 1)

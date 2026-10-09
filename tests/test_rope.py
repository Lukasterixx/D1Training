"""The rope-pull task's plant, rewards and scoring, without a simulator.

`unifp_train/rope.py` is the task's only new physics -- a tension-only tether and an anchor that gives way -- and a
sign error there would train a policy to push on a rope. These pin it case by case. The environment wiring
(`rope_env.py`) needs Isaac Lab and is exercised by `run_unifp_train.py smoke --task rope` instead.
"""
from __future__ import annotations

import math
import unittest

import numpy as np
import torch

from unifp_isaaclab import interface
from unifp_train import rope, rope_cfg, rope_eval, rope_rewards
from unifp_train.rewards import TaskState

X = torch.tensor([[1.0, 0.0, 0.0]])
STILL = torch.zeros(1, 3)


def tied(length=1.0, stiffness=2000.0, damping=10.0, n=1):
    """A rope tied to a tool at the origin, pulled along +x: the anchor is `length` out along -x."""
    tether = rope.RopeTether(n, "cpu")
    full = lambda v: torch.full((n,), float(v))
    tether.engage(torch.arange(n), torch.zeros(n, 3), X.expand(n, 3), length=full(length), stiffness=full(stiffness),
                  damping=full(damping))
    return tether


def at(x=0.0, y=0.0, z=0.0):
    return torch.tensor([[x, y, z]])


class RopeTetherTest(unittest.TestCase):
    def test_tied_taut_with_no_tension(self):
        tether = tied()
        torch.testing.assert_close(tether.anchor, at(x=-1.0))
        force = tether.step(at(), STILL)
        torch.testing.assert_close(force, torch.zeros(1, 3))
        self.assertTrue(bool(tether.slack[0]))

    def test_pulling_away_stretches_it_and_it_pulls_back(self):
        tether = tied()
        force = tether.step(at(x=0.01), STILL)
        torch.testing.assert_close(force, at(x=-20.0))
        torch.testing.assert_close(tether.applied_by_robot(), at(x=20.0))
        self.assertAlmostEqual(float(tether.stretch[0]), 0.01, places=6)
        self.assertFalse(bool(tether.slack[0]))

    def test_moving_toward_the_anchor_slackens_it(self):
        tether = tied()
        force = tether.step(at(x=-0.05), STILL)
        torch.testing.assert_close(force, torch.zeros(1, 3))
        self.assertTrue(bool(tether.slack[0]))

    def test_the_damper_never_pushes(self):
        """Stretched 1 mm but moving back at 1 m/s: the damper would say -8 N; the rope says 0."""
        tether = tied()
        force = tether.step(at(x=0.001), at(x=-1.0))
        torch.testing.assert_close(force, torch.zeros(1, 3))

    def test_force_runs_along_the_rope_not_the_pull_axis(self):
        """A tool 10 cm to the side of a 1 m rope's line is pulled back along the rope, which now leans."""
        tether = tied()
        tether.step(at(x=0.01, y=0.1), STILL)
        direction = torch.tensor([[1.01, 0.1, 0.0]]) / math.hypot(1.01, 0.1)
        torch.testing.assert_close(tether.direction, direction)
        stretch = math.hypot(1.01, 0.1) - 1.0
        torch.testing.assert_close(tether.force_on_tool, -2000.0 * stretch * direction, rtol=1e-4, atol=1e-4)

    def test_sideways_it_holds_only_tension_over_length(self):
        """Across the line a taut rope is a pendulum: T / L per metre, nothing like the ring's 2,000 N/m."""
        tether = tied(length=1.0)
        tether.step(at(x=0.02), STILL)
        tension = float(tether.tension[0])
        tether.step(at(x=0.02, y=0.01), STILL)
        lateral = float(tether.force_on_tool[0, 1])
        self.assertAlmostEqual(-lateral / 0.01, tension / 1.02, delta=0.05 * tension)

    def test_a_snap_drops_the_tension_to_zero_at_once(self):
        tether = tied()
        tether.step(at(x=0.02), STILL)
        self.assertAlmostEqual(float(tether.tension[0]), 40.0, places=4)
        tether.release(torch.tensor([0]), torch.tensor([rope.SNAP]), torch.tensor([0.0]), torch.tensor([0]))
        force = tether.step(at(x=0.02), STILL)
        torch.testing.assert_close(force, torch.zeros(1, 3))
        # ... and pulling further only drags the anchor: the rope has gone.
        force = tether.step(at(x=0.10), at(x=0.5))
        torch.testing.assert_close(force, torch.zeros(1, 3))
        self.assertGreater(float(tether.paid_out[0]), 0.07)

    def test_a_slip_holds_its_residual_and_pays_out(self):
        tether = tied()
        tether.step(at(x=0.02), STILL)
        tether.release(torch.tensor([0]), torch.tensor([rope.SLIP]), torch.tensor([0.25]), torch.tensor([0]))
        tether.step(at(x=0.05), STILL)
        self.assertAlmostEqual(float(tether.tension[0]), 10.0, places=4)
        self.assertAlmostEqual(float(tether.stretch[0]), 10.0 / 2000.0, places=6)
        # The anchor moved toward the tool by what it paid out.
        self.assertAlmostEqual(float(tether.anchor[0, 0]), -1.0 + 0.05 - 0.005, places=5)
        # Easing off below the residual, the rope holds again where it is.
        tether.step(at(x=0.048), STILL)
        self.assertAlmostEqual(float(tether.tension[0]), 2000.0 * 0.003, places=3)

    def test_a_fade_lowers_the_capacity_linearly(self):
        tether = tied()
        tether.step(at(x=0.02), STILL)
        tether.release(torch.tensor([0]), torch.tensor([rope.FADE]), torch.tensor([0.0]), torch.tensor([4]))
        seen = []
        for _ in range(6):
            tether.step(at(x=0.02), STILL)
            seen.append(round(float(tether.tension[0]), 3))
        self.assertEqual(seen, [40.0, 30.0, 20.0, 10.0, 0.0, 0.0])

    def test_reset_unties_it(self):
        tether = tied()
        tether.step(at(x=0.02), STILL)
        tether.reset(torch.tensor([0]))
        self.assertFalse(bool(tether.engaged[0]))
        torch.testing.assert_close(tether.step(at(x=0.5), STILL), torch.zeros(1, 3))

    def test_environments_are_independent(self):
        tether = tied(n=2)
        tether.release(torch.tensor([1]), torch.tensor([rope.SNAP]), torch.tensor([0.0]), torch.tensor([0]))
        force = tether.step(torch.tensor([[0.01, 0.0, 0.0], [0.01, 0.0, 0.0]]), torch.zeros(2, 3))
        torch.testing.assert_close(force[0], torch.tensor([-20.0, 0.0, 0.0]))
        torch.testing.assert_close(force[1], torch.zeros(3))


def state(n=1, **fields):
    """A TaskState with only what the rope terms read; everything else zero."""
    zeros = lambda *shape: torch.zeros(n, *shape)
    base = TaskState(
        base_lin_vel_b=zeros(3), base_ang_vel_b=zeros(3), root_pos_w=zeros(3), dof_pos=zeros(20), dof_vel=zeros(20),
        last_dof_vel=zeros(20), torques=zeros(20), actions=zeros(18), last_actions=zeros(18), commands=zeros(15),
        gait_phase=zeros(), ref_leg_pos=zeros(12), stance_mask=zeros(4), contact_mask=zeros(4), feet_pos_w=zeros(4, 3),
        feet_contact_forces=zeros(4, 3), feet_air_time=zeros(4), penalised_contact_forces=zeros(3, 3),
        ee_pos_w=zeros(3), ee_goal_w=zeros(3), thigh_pos_w=zeros(4, 3), feet_vel_w=zeros(4, 3),
        last_contacts=torch.zeros(n, 4, dtype=torch.bool), base_yaw_quat=torch.tensor([[1.0, 0.0, 0.0, 0.0]] * n),
        ee_force_measured=zeros(3), ee_force_cmd=zeros(3), base_force_measured=zeros(3), base_force_cmd=zeros(3),
        gripper_force_kp=torch.full((n, 3), 200.0), base_force_kd=torch.full((n, 3), 200.0))
    defaults = dict(fixture_engaged=zeros(), rope_held=zeros(), fixture_applied_w=zeros(3), fixture_command_w=zeros(3),
                    fixture_axis_w=X.expand(n, 3), tool_vel_w=zeros(3), rope_tv_n=zeros(), rope_slack_fraction=zeros(),
                    rope_recovering=zeros(), base_lin_vel_w=zeros(3), rope_fallen=zeros(),
                    rope_arm_line_w=X.expand(n, 3).clone(), rope_line_w=X.expand(n, 3).clone(), rope_wobble_n2=zeros())
    defaults.update(fields)
    for key, value in defaults.items():
        setattr(base, key, value)
    return base


class RopeRewardTest(unittest.TestCase):
    def test_ripple_is_squared_capped_and_gated_on_an_intact_anchor(self):
        held = torch.ones(3)
        s = state(3, rope_tv_n=torch.tensor([0.0, 2.0, 100.0]), fixture_engaged=held)
        torch.testing.assert_close(rope_rewards.rope_ripple(s), torch.tensor([0.0, 1.0, rope_cfg.RIPPLE_CAP]))
        s = state(3, rope_tv_n=torch.tensor([0.0, 2.0, 100.0]), fixture_engaged=torch.zeros(3))
        torch.testing.assert_close(rope_rewards.rope_ripple(s), torch.zeros(3))

    def test_wobble_is_scaled_capped_and_gated(self):
        s = state(3, rope_wobble_n2=torch.tensor([0.0, 4.0, 1000.0]), fixture_engaged=torch.tensor([1.0, 1.0, 1.0]))
        torch.testing.assert_close(rope_rewards.rope_wobble(s), torch.tensor([0.0, 1.0, rope_cfg.RIPPLE_CAP]))
        s = state(1, rope_wobble_n2=torch.tensor([4.0]), fixture_engaged=torch.zeros(1))
        self.assertEqual(float(rope_rewards.rope_wobble(s)[0]), 0.0)

    def test_upset_charges_only_while_recovering(self):
        s = state(2, base_lin_vel_w=torch.tensor([[0.3, 0.4, 5.0], [0.3, 0.4, 0.0]]),
                  base_ang_vel_b=torch.tensor([[2.0, 0.0, 9.0], [0.0, 0.0, 0.0]]), rope_recovering=torch.tensor([1.0, 0.0]))
        torch.testing.assert_close(rope_rewards.rope_upset(s), torch.tensor([0.25 + 0.25 * 4.0, 0.0]))

    def test_force_tracking_still_slopes_far_from_the_command(self):
        term = rope_rewards.TERMS["fixture_force_tracking"]
        on = state(1, fixture_engaged=torch.ones(1), fixture_applied_w=at(x=30.0), fixture_command_w=at(x=30.0))
        self.assertAlmostEqual(float(term(on)[0]), 1.0, places=6)
        short = [float(term(state(1, fixture_engaged=torch.ones(1), fixture_applied_w=at(x=f),
                                  fixture_command_w=at(x=50.0)))[0]) for f in (20.0, 25.0)]
        # 5 N closer at 25-30 N short is worth at least 0.01 (the hook task's term: 0.006).
        self.assertGreater(short[1] - short[0], 0.01)

    def test_taut_arm_pays_for_the_arm_along_the_rope_while_pulling(self):
        lines = torch.tensor([[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
        s = state(3, rope_arm_line_w=lines, fixture_engaged=torch.tensor([1.0, 1.0, 0.0]),
                  fixture_command_w=torch.tensor([[20.0, 0.0, 0.0]] * 3))
        reward = rope_rewards.rope_taut_arm(s)
        self.assertAlmostEqual(float(reward[0]), 1.0, places=5)
        self.assertAlmostEqual(float(reward[1]), math.exp(-(math.pi / 4) / rope_cfg.TAUT_ARM_SIGMA_RAD), places=5)
        self.assertEqual(float(reward[2]), 0.0)
        idle = state(1, fixture_engaged=torch.ones(1), fixture_command_w=at(x=2.0))
        self.assertEqual(float(rope_rewards.rope_taut_arm(idle)[0]), 0.0)

    def test_every_term_has_a_weight_and_back(self):
        self.assertEqual(set(rope_rewards.TERMS), set(rope_cfg.WEIGHTS))


class RopeScoringTest(unittest.TestCase):
    RATE = 200.0

    def test_tugs_are_prominent_peaks(self):
        level = 40.0
        signal = np.array([40, 41, 40, 52, 30, 40, 39, 41, 20, 45, 40, 40], dtype=float)
        # Prominence is the height over the higher of the two troughs: 52 stands 12 over 40 (a tug, >= 10 N); 41 at
        # index 7 stands 11 over 30 and 21 over 20 (a tug); 45 stands only 5 over the 40 after it; 41 at index 1, 1.
        self.assertEqual(rope_eval.count_tugs(signal, level), 2)
        self.assertEqual(rope_eval.count_tugs(np.full(50, 40.0), level), 0)
        ripple = 40.0 + np.sin(np.arange(200) * 0.7)
        self.assertEqual(rope_eval.count_tugs(ripple, level), 0)

    def test_tugs_against_a_ramping_command(self):
        """Counted against the command as it ramps; nothing counts while under 10 N is commanded."""
        command = np.r_[np.full(6, 8.0), np.full(9, 20.0)]
        tension = np.array([8, 0, 8, 0, 8, 0, 20, 30, 20, 20, 35, 10, 20, 20, 20], dtype=float)
        self.assertEqual(rope_eval.count_tugs(tension, command), 2)

    def test_steadiness_finds_a_shake(self):
        t = np.arange(400) / self.RATE
        tension = 40.0 + 3.0 * np.sin(2 * math.pi * 6.0 * t)
        stats = rope_eval.steadiness(tension, 40.0, self.RATE)
        self.assertAlmostEqual(stats["dominant_hz"], 6.0, delta=0.5)
        self.assertAlmostEqual(stats["dominant_amp_n"], 3.0, delta=0.4)
        self.assertAlmostEqual(stats["std_n"], 3.0 / math.sqrt(2), delta=0.1)
        self.assertGreater(stats["hf_rms_n"], 1.0)
        self.assertEqual(stats["tugs"], 0)  # 6 N peak to trough is under 25% of 40 N: a shake, not a tug
        tugging = rope_eval.steadiness(40.0 + 8.0 * np.sin(2 * math.pi * 6.0 * t), 40.0, self.RATE)
        self.assertGreaterEqual(tugging["tugs"], 10)  # 12 cycles in 2 s, the first and last without both troughs
        self.assertEqual(stats["slack_fraction"], 0.0)
        steady = rope_eval.steadiness(np.full(400, 40.0), 40.0, self.RATE)
        self.assertEqual((steady["std_n"], steady["hf_rms_n"], steady["tugs"]), (0.0, 0.0, 0))

    def test_a_rope_going_slack_is_a_tug_and_slack(self):
        tension = np.tile(np.r_[np.full(30, 40.0), np.zeros(10)], 10)
        stats = rope_eval.steadiness(tension, 40.0, self.RATE)
        self.assertEqual(stats["tugs"], 9)  # the first plateau has no trough before it
        self.assertAlmostEqual(stats["slack_fraction"], 0.25, places=6)

    def test_settle_time(self):
        speed = np.r_[np.full(10, 0.5), np.zeros(40)]
        self.assertAlmostEqual(rope_eval.settle_time(speed, np.zeros(50), 0.02), 0.2, places=6)
        self.assertIsNone(rope_eval.settle_time(np.full(50, 0.5), np.zeros(50), 0.02))

    def test_score_reads_a_synthetic_episode(self):
        """One episode: engaged at step 10, three 20-step levels, released at step 70, standing still throughout."""
        levels, hold, window, recovery = (20.0, 35.0, 50.0), 20, 10, 30
        steps = 100
        stage = np.zeros((steps, 1))
        level_at = np.zeros((steps, 1))
        for k, lvl in enumerate(levels):
            stage[10 + k * hold:10 + (k + 1) * hold] = k + 1
            level_at[10 + k * hold:10 + (k + 1) * hold] = lvl
        stage[70:] = 3
        released = np.zeros((steps, 1))
        released[70:] = 1.0
        tension = np.repeat(level_at[:, :, None], 4, axis=2)
        tension[70:] = 0.0
        trace = {key: np.zeros((steps, 1)) for key in rope_eval.FIELDS}
        trace.update(stage=stage, level=level_at, engaged=(stage > 0).astype(float), released=released,
                     tension_sub=tension, height=np.full((steps, 1), 0.31), estimate_along=np.full((steps, 1), np.nan),
                     base_xy=np.zeros((steps, 1, 2)), tool_b=np.zeros((steps, 1, 3)), arm_loads=np.full((steps, 1, 6), 0.5),
                     feet_fz=np.full((steps, 1, 4), 45.0), feet_xy=np.zeros((steps, 1, 4, 2)))
        rows = [{"release": "snap_noticed", "stiffness": 1000.0, "sphere": (0.5, 0.0, 0.0)}]
        result = rope_eval.score(trace, rows, levels=levels, hold_steps=hold, window_steps=window, recovery_steps=recovery)
        episode = result["episodes"][0]
        self.assertEqual([lv["held"] for lv in episode["levels"]], [True, True, True])
        self.assertEqual([lv["std_n"] for lv in episode["levels"]], [0.0, 0.0, 0.0])
        self.assertEqual(episode["levels"][2]["front_share"], 0.5)
        self.assertFalse(episode["recovery"]["fell"])
        self.assertEqual(episode["recovery"]["settle_s"], 0.0)
        self.assertEqual(result["summary"]["by_level"][0]["held"], 1)
        self.assertEqual(result["summary"]["recovery"][0]["fell"], 0)


class RopeEvalPlanTest(unittest.TestCase):
    def test_the_frozen_set(self):
        """Frozen 2026-10-07: 9 placements x 2 stiffnesses x 3 releases, 20/35/50 N for 4 s, released at the end."""
        plan = rope_cfg.eval_plan()
        self.assertEqual(len(plan), 54)
        self.assertEqual(rope_cfg.EVAL_LEVELS_N, (20.0, 35.0, 50.0))
        self.assertEqual((rope_cfg.EVAL_HOLD_S, rope_cfg.EVAL_WINDOW_S, rope_cfg.EVAL_RECOVERY_S), (4.0, 2.0, 4.0))
        self.assertEqual(rope_cfg.EVAL_STIFFNESS, (1000.0, 4000.0))
        self.assertEqual({r["release"] for r in plan}, {"snap_noticed", "snap_unnoticed", "slip_noticed"})
        heights = sorted({round(rope_cfg.GOAL_CENTRE_HEIGHT_M + s[0] * math.sin(s[1]), 6)
                          for s in rope_cfg.eval_grip_spheres()})
        self.assertEqual(heights, [0.25, 0.4, 0.55])

    def test_the_held_out_test_set(self):
        """Frozen 2026-10-07 before any policy was run on it; nothing in it repeats the development set."""
        plan = rope_cfg.test_plan()
        self.assertEqual(len(plan), 48)
        self.assertEqual(rope_cfg.TEST_LEVELS_N, (15.0, 30.0, 45.0))
        self.assertEqual({r["length"] for r in plan}, {0.5, 1.5})
        self.assertEqual({r["stiffness"] for r in plan}, {2000.0, 6000.0})
        self.assertEqual({r["elevation"] for r in plan}, {-0.15, 0.15})
        self.assertFalse({r["release"] for r in plan} & {r["release"] for r in rope_cfg.eval_plan()})
        self.assertFalse({tuple(r["sphere"]) for r in plan} & {tuple(r["sphere"]) for r in rope_cfg.eval_plan()})

    def test_weights_are_scaled_by_the_policy_step(self):
        scaled = rope_cfg.scaled_weights()
        for name, weight in rope_cfg.WEIGHTS.items():
            self.assertAlmostEqual(scaled[name], weight * interface.POLICY_DT)


if __name__ == "__main__":
    unittest.main()

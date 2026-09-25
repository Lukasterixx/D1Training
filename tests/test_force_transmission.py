"""The force-transmission task's contact, command schedule, rewards and scoring, without a simulator.

`unifp_train/fixture.py` is the only new physics in the task -- a unilateral spring with friction
across it -- and a sign error there would train a policy to push on a handle it is meant to pull.
These pin its behaviour case by case. The environment wiring (`hook_env.py`) needs Isaac Lab and is
exercised by `run_unifp_train.py smoke --task hook` instead.
"""
from __future__ import annotations

import math
import unittest

import numpy as np
import torch

from unifp_isaaclab import interface
from unifp_train import fixture, hook_cfg, hook_eval, hook_rewards
from unifp_train.rewards import TaskState


NEVER = fixture.ContactFixture.NEVER_M
X = torch.tensor([[1.0, 0.0, 0.0]])


def make_fixture(n=1, mu=2.0, allowance=5.0, slip=0.05, release=0.03, backstop=NEVER, damping=10.0,
                 bar=None, lift=NEVER, axis=X):
    """A pad by default (isotropic friction, releases on backing off); pass `bar` for a claw."""
    fx = fixture.ContactFixture(n, "cpu", damping=damping)
    ids = torch.arange(n)
    bar = torch.zeros(n, 3) if bar is None else bar.expand(n, 3)
    full = lambda v: torch.full((n,), float(v))
    fx.engage(ids, torch.zeros(n, 3), axis.expand(n, 3), stiffness=full(2000.0), mu=full(mu),
              allowance_n=full(allowance), slip_radius_m=full(slip), release_m=full(release),
              backstop_m=full(backstop), bar=bar, lift_release_m=full(lift))
    return fx


def claw(**kwargs):
    """A claw over a horizontal bar running along y, pulled along +x: the bar is under it (+z)."""
    return make_fixture(release=NEVER, backstop=0.03, bar=torch.tensor([[0.0, 1.0, 0.0]]), lift=0.02,
                        **kwargs)


def at(x=0.0, y=0.0, z=0.0):
    return torch.tensor([[x, y, z]])


STILL = torch.zeros(1, 3)


class ContactFixtureTest(unittest.TestCase):
    def test_loading_along_the_axis_is_resisted(self):
        fx = make_fixture()
        force = fx.step(at(x=0.01), STILL)
        torch.testing.assert_close(force, at(x=-20.0))
        torch.testing.assert_close(fx.applied_by_robot(), at(x=20.0))
        self.assertAlmostEqual(float(fx.penetration_m[0]), 0.01, places=6)

    def test_a_claw_backing_off_meets_the_door_and_stays_hooked(self):
        fx = claw()
        torch.testing.assert_close(fx.step(at(x=-0.02), STILL), torch.zeros(1, 3))
        force = fx.step(at(x=-0.04), STILL)
        # 1 cm past the 3 cm gap: the door pushes the claw back along +d, and the robot is pushing
        # the door, the opposite of the pull it was asked for.
        torch.testing.assert_close(force, at(x=20.0))
        torch.testing.assert_close(fx.applied_by_robot(), at(x=-20.0))
        fx.step(at(x=-0.30), STILL)
        self.assertTrue(bool(fx.engaged[0]))
        self.assertFalse(bool(fx.lost[0]))

    def test_a_claw_rests_its_weight_on_the_bar_without_slipping(self):
        fx = claw()
        # No pull at all, the claw 1 cm down into the bar: the bar carries 20 N and nothing slides,
        # where the pad's friction (5 N allowance, no normal load) would have let it go.
        force = fx.step(at(z=-0.01), STILL)
        torch.testing.assert_close(force, at(z=20.0))
        torch.testing.assert_close(fx.anchor, torch.zeros(1, 3))
        self.assertTrue(bool(fx.engaged[0]))

    def test_a_claw_lifted_off_its_bar_is_unhooked(self):
        fx = claw()
        torch.testing.assert_close(fx.step(at(z=0.015), STILL), torch.zeros(1, 3))
        self.assertTrue(bool(fx.engaged[0]))
        fx.step(at(z=0.03), STILL)
        self.assertTrue(bool(fx.lost[0]))
        self.assertEqual(int(fx.lost_how[0]), fixture.ContactFixture.LOST_LIFTED_OFF)

    def test_a_claw_slides_along_its_bar_and_off_the_end(self):
        fx = claw()
        # 10 N of pull: 2 x 10 + 5 = 25 N of friction along the bar, 1.25 cm of spring.
        force = fx.step(at(x=0.005, y=0.02), STILL)
        self.assertAlmostEqual(float(force[0, 1]), -25.0, places=3)
        self.assertAlmostEqual(float(fx.anchor[0, 1]), 0.02 - 0.0125, places=6)
        fx.step(at(x=0.005, y=0.08), STILL)
        self.assertTrue(bool(fx.lost[0]))
        self.assertEqual(int(fx.lost_how[0]), fixture.ContactFixture.LOST_SLID_OFF)

    def test_the_weight_on_a_bar_adds_to_its_friction(self):
        fx = claw()
        # 0 N of pull but 20 N resting on the bar: 2 x 20 + 5 = 45 N before it slides.
        force = fx.step(at(y=0.02, z=-0.01), STILL)
        torch.testing.assert_close(force, at(y=-40.0, z=20.0))
        torch.testing.assert_close(fx.anchor, torch.zeros(1, 3))

    def test_a_pad_backing_off_is_slack_then_lost(self):
        fx = make_fixture()
        torch.testing.assert_close(fx.step(at(x=-0.01), STILL), torch.zeros(1, 3))
        self.assertTrue(bool(fx.engaged[0]))
        fx.step(at(x=-0.04), STILL)
        self.assertFalse(bool(fx.engaged[0]))
        self.assertTrue(bool(fx.lost[0]))
        # A lost contact stays lost, and pushes nothing, even back on the anchor.
        torch.testing.assert_close(fx.step(at(x=0.01), STILL), torch.zeros(1, 3))

    def test_damping_never_pulls_the_tool_in(self):
        fx = make_fixture()
        force = fx.step(at(x=0.0005), at(x=-1.0))
        self.assertEqual(float(force[0, 0]), 0.0)

    def test_a_pad_holds_a_sideways_load_inside_the_cap(self):
        fx = make_fixture()
        force = fx.step(at(x=0.01, y=0.01), STILL)
        torch.testing.assert_close(force, at(x=-20.0, y=-20.0))
        torch.testing.assert_close(fx.anchor, torch.zeros(1, 3))

    def test_a_pad_past_the_cap_slides_then_comes_off(self):
        fx = make_fixture(mu=0.5, allowance=0.0, slip=0.02)
        force = fx.step(at(x=0.01, y=0.01), STILL)
        # 20 N normal, so 10 N across at most; the contact point follows the tool half-way.
        self.assertAlmostEqual(float(force[0, 1]), -10.0, places=4)
        self.assertAlmostEqual(float(fx.anchor[0, 1]), 0.005, places=6)
        self.assertAlmostEqual(float(fx.anchor[0, 0]), 0.0, places=9)
        fx.step(at(x=0.01, y=0.04), STILL)
        self.assertTrue(bool(fx.lost[0]))

    def test_a_press_is_the_same_contact_pointing_the_other_way(self):
        fx = make_fixture(mu=0.5, allowance=0.0, slip=0.02, damping=0.0, axis=at(z=-1.0))
        force = fx.step(at(z=-0.005), STILL)
        torch.testing.assert_close(force, at(z=10.0))
        torch.testing.assert_close(fx.applied_by_robot(), at(z=-10.0))

    def test_unengaged_environments_feel_nothing(self):
        fx = fixture.ContactFixture(2, "cpu", damping=10.0)
        torch.testing.assert_close(fx.step(torch.ones(2, 3), torch.ones(2, 3)), torch.zeros(2, 3))
        self.assertFalse(bool(fx.lost.any()))

    def test_a_reattaching_pad_detaches_then_presses_again(self):
        fx = fixture.ContactFixture(1, "cpu", damping=0.0)
        full = lambda v: torch.full((1,), float(v))
        fx.engage(torch.tensor([0]), torch.zeros(1, 3), X, stiffness=full(2000.0), mu=full(0.6),
                  allowance_n=full(1.0), slip_radius_m=full(0.04), release_m=full(0.03), backstop_m=full(NEVER),
                  bar=torch.zeros(1, 3), lift_release_m=full(NEVER), reattach=torch.tensor([True]))
        fx.step(at(x=-0.05), STILL)                                  # backed off the button
        self.assertFalse(bool(fx.engaged[0]))
        self.assertFalse(bool(fx.lost[0]))
        self.assertTrue(bool(fx.detached[0]))
        self.assertEqual(float(fx.risk[0]), 1.0)
        torch.testing.assert_close(fx.step(at(x=-0.01), STILL), torch.zeros(1, 3))   # hovering, off it
        force = fx.step(at(x=0.005, y=0.01), STILL)                 # pressed back on, 1 cm off-centre
        self.assertTrue(bool(fx.engaged[0]))
        torch.testing.assert_close(force, at(x=-10.0))
        self.assertTrue(bool(fx.ever_detached[0]))
        fx.step(at(x=-0.05), STILL)
        fx.step(at(x=0.005, y=0.06), STILL)                          # beside the button: no press
        self.assertFalse(bool(fx.engaged[0]))

    def test_risk_rises_toward_each_way_of_losing_it(self):
        fx = claw()                                                    # a 2 cm hook
        fx.step(at(z=0.01), STILL)
        self.assertAlmostEqual(float(fx.risk[0]), 0.5, places=5)     # half-way to lifting off
        pad = make_fixture()
        pad.step(at(x=-0.015), STILL)
        self.assertAlmostEqual(float(pad.risk[0]), 0.5, places=5)    # half-way to backing off

    def test_a_ring_holds_any_sideways_load_and_cannot_come_off(self):
        fx = make_fixture(mu=1.0e3, allowance=1.0e3, slip=NEVER, release=NEVER, backstop=0.03)
        force = fx.step(at(x=0.005, y=0.05, z=-0.05), STILL)
        torch.testing.assert_close(force, at(x=-10.0, y=-100.0, z=100.0))
        fx.step(at(x=-0.2, y=0.3), STILL)
        self.assertTrue(bool(fx.engaged[0]))
        self.assertAlmostEqual(float(fx.risk[0]), 0.0, places=5)

    def test_bars_are_perpendicular_to_the_pull(self):
        axis = torch.tensor([[0.8, 0.0, 0.6], [-0.6, 0.8, 0.0]])
        for vertical in (False, True):
            bar = fixture.bar_directions(axis, torch.tensor([vertical, vertical]))
            torch.testing.assert_close((bar * axis).sum(-1), torch.zeros(2), atol=1e-6, rtol=0)
            torch.testing.assert_close(bar.norm(dim=-1), torch.ones(2))
            if not vertical:
                torch.testing.assert_close(bar[:, 2], torch.zeros(2), atol=1e-6, rtol=0)


class ForceLevelScheduleTest(unittest.TestCase):
    def schedule(self, n=2):
        return fixture.ForceLevelSchedule(n, "cpu", hold_s=(1.0, 1.0), ramp_n_per_s=25.0,
                                          zero_prob=0.0, dt=interface.POLICY_DT)

    def test_ramps_at_the_rate_and_idles_at_zero(self):
        sched = self.schedule()
        sched.set_levels(torch.tensor([0]), torch.tensor([50.0]), hold_steps=100)
        active = torch.tensor([True, False])
        for _ in range(10):
            value = sched.step(active, ceiling=60.0)
        self.assertAlmostEqual(float(value[0]), 5.0, places=4)
        self.assertEqual(float(value[1]), 0.0)

    def test_frontier_draws_sit_near_the_ceiling(self):
        sched = fixture.ForceLevelSchedule(4000, "cpu", hold_s=(1.0, 1.0), ramp_n_per_s=25.0,
                                           zero_prob=0.0, dt=interface.POLICY_DT, frontier_prob=1.0,
                                           frontier_fraction=0.7)
        sched.draw(torch.arange(4000), ceiling=20.0)
        self.assertGreaterEqual(float(sched.level.min()), 14.0)
        self.assertLessEqual(float(sched.level.max()), 20.0)

    def test_draws_stay_under_the_ceiling(self):
        sched = self.schedule(n=500)
        sched.draw(torch.arange(500), ceiling=15.0)
        self.assertLessEqual(float(sched.level.max()), 15.0)
        self.assertGreater(float(sched.level.max()), 10.0)

    def test_staircase_climbs_in_order_and_stays_on_the_top_step(self):
        sched = self.schedule(n=1)
        sched.use_staircase((10.0, 20.0), hold_steps=3)
        active = torch.tensor([True])
        levels = []
        for _ in range(9):
            sched.step(active, ceiling=0.0)
            levels.append(float(sched.level[0]))
        self.assertEqual(levels, [10.0] * 3 + [20.0] * 6)


class AxesAndPlacementsTest(unittest.TestCase):
    def test_a_pull_points_back_at_the_robot(self):
        axis = fixture.pull_axes(at(x=0.5, y=0.5, z=0.4), at(), yaw_noise_rad=0.0, elevation_rad=(0.0, 0.0))
        torch.testing.assert_close(axis, at(x=-1.0, y=-1.0) / math.sqrt(2.0))

    def test_a_press_points_away_unless_it_points_down(self):
        away = fixture.press_axes(at(x=0.5, z=0.4), at(), yaw_noise_rad=0.0, elevation_rad=(0.0, 0.0),
                                  down_prob=0.0)
        torch.testing.assert_close(away, at(x=1.0))
        down = fixture.press_axes(at(x=0.5, z=0.4), at(), yaw_noise_rad=0.0, elevation_rad=(0.0, 0.0),
                                  down_prob=1.0)
        torch.testing.assert_close(down, at(z=-1.0))

    def test_handles_are_clear_of_the_body(self):
        torch.manual_seed(0)
        local = interface.sphere2cart(fixture.sample_handles(4000, "cpu"))
        clear = (local[:, 0] >= hook_cfg.HANDLE_CLEAR_AHEAD_M) | (local[:, 1].abs() >= hook_cfg.HANDLE_CLEAR_SIDE_M)
        self.assertTrue(bool(clear.all()))
        heights = hook_cfg.GOAL_CENTRE_HEIGHT_M + local[:, 2]
        self.assertGreater(float(heights.min()), 0.09)
        self.assertLess(float(heights.max()), 0.75)

    def test_evaluation_placements_sit_at_their_stated_heights(self):
        spheres = torch.tensor(hook_cfg.eval_handle_spheres())
        local = interface.sphere2cart(spheres)
        heights = sorted({round(float(h), 6) for h in hook_cfg.GOAL_CENTRE_HEIGHT_M + local[:, 2]})
        self.assertEqual(heights, [round(h, 6) for h in hook_cfg.EVAL_HEIGHTS_M])
        np.testing.assert_allclose(local[:, :2].norm(dim=-1).numpy(), hook_cfg.EVAL_REACH_M, atol=1e-6)


class RewardTermsTest(unittest.TestCase):
    def state(self, applied, command, engaged=1.0, torques=None):
        n = applied.shape[0]
        blank = torch.zeros(n, 1)
        state = TaskState(*([blank] * 30))
        state.torques = torch.zeros(n, 20) if torques is None else torques
        state.fixture_engaged = torch.full((n,), engaged)
        state.fixture_applied_w = applied
        state.fixture_command_w = command
        state.fixture_lost = torch.zeros(n)
        state.fixture_axis_w = at(x=1.0).expand(n, 3)
        return state

    def test_force_tracking_is_one_on_target_and_zero_off_the_fixture(self):
        on = hook_rewards.fixture_force_tracking(self.state(at(x=30.0), at(x=30.0)))
        self.assertAlmostEqual(float(on[0]), 1.0, places=6)
        wrong_way = hook_rewards.fixture_force_tracking(self.state(at(y=30.0), at(x=30.0)))
        self.assertLess(float(wrong_way[0]), 0.1)
        # The right pull plus the arm's weight resting on the handle costs a quarter of that weight.
        resting = hook_rewards.fixture_force_tracking(self.state(at(x=30.0, z=-12.0), at(x=30.0)))
        expected = 0.5 * math.exp(-3.0 / 3.0) + 0.5 * math.exp(-3.0 / 12.0)
        self.assertAlmostEqual(float(resting[0]), expected, places=5)
        off = hook_rewards.fixture_force_tracking(self.state(at(x=30.0), at(x=30.0), engaged=0.0))
        self.assertEqual(float(off[0]), 0.0)

    def test_tool_speed_is_charged_only_on_a_fixture(self):
        state = self.state(at(), at())
        state.tool_vel_w = at(x=0.3, y=0.4)
        state.fixture_active = torch.ones(1)
        self.assertAlmostEqual(float(hook_rewards.fixture_tool_speed(state)[0]), 0.25, places=6)
        state.fixture_active = torch.zeros(1)
        self.assertEqual(float(hook_rewards.fixture_tool_speed(state)[0]), 0.0)

    def test_torque_margin_charges_only_past_seventy_percent(self):
        torques = torch.zeros(1, 20)
        torques[0, 14] = 1.7          # Joint3 at its limit
        torques[0, 12] = 0.5 * 3.3    # Joint1 at half
        cost = hook_rewards.arm_torque_margin(self.state(at(), at(), torques=torques))
        self.assertAlmostEqual(float(cost[0]), 0.09, places=5)


class ScoringTest(unittest.TestCase):
    def test_sustained_force_is_the_last_level_held_without_a_gap(self):
        levels, hold, window = (10.0, 20.0, 30.0, 40.0), 5, 2
        steps = hold * len(levels) + 3
        stage = np.zeros((steps, 1))
        applied = np.zeros((steps, 1))
        for k, level in enumerate(levels):
            stage[3 + k * hold: 3 + (k + 1) * hold] = k + 1
            # Holds 10 and 20, drops 30 (to half), then "holds" 40 -- which must not count.
            applied[3 + k * hold: 3 + (k + 1) * hold] = level * (0.5 if level == 30.0 else 1.0)
        trace = {key: np.zeros((steps, 1)) for key in hook_eval.FIELDS}
        trace["arm_loads"] = np.zeros((steps, 1, 6))
        trace["arm_limit_gap"] = np.ones((steps, 1, 6))
        trace.update(stage=stage, applied_along=applied, engaged=(stage > 0).astype(float))
        trace["estimate_along"][:] = np.nan
        trace["estimate_err"][:] = np.nan
        result = hook_eval.score(trace, levels=levels, hold_steps=hold, window_steps=window,
                                 handles_per_repeat=1)
        episode = result["episodes"][0]
        self.assertEqual(episode["sustained_n"], 20.0)
        self.assertEqual([row["held"] for row in episode["levels"]], [True, True, False, True])


if __name__ == "__main__":
    unittest.main()

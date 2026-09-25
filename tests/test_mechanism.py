"""The goal-commanded task's mechanism, goal, rewards and scoring, without a simulator.

`unifp_train/mechanism.py` is the task's only new physics -- a one-degree-of-freedom plant with
stick-slip, a snapping latch and hard stops, driven through a 3-D grasp spring -- and a sign error
there would train a policy that pushes a drawer shut. These pin its behaviour case by case. The
environment wiring (`mech_env.py`) needs Isaac Lab and is exercised by
`run_unifp_train.py smoke --task mechanism` instead.
"""
from __future__ import annotations

import math
import unittest

import numpy as np
import torch

from unifp_isaaclab import interface
from unifp_train import hook_rewards, mech_cfg, mech_eval, mech_rewards, mechanism
from unifp_train.rewards import TaskState

DT = interface.SIM_DT
X = torch.tensor([[1.0, 0.0, 0.0]])
Y = torch.tensor([[0.0, 1.0, 0.0]])
Z = torch.tensor([[0.0, 0.0, 1.0]])


def make(n=1, hinge=False, opening=X, axis=Z, radius=0.3, travel=0.10, mass=1.0, spring_n=0.0, spring_k=0.0,
         damping=0.0, static=0.0, kinetic=0.0, latch=0.0, grasp_k=2000.0, grip=150.0, point=None):
    mech = mechanism.Mechanism(n, "cpu", grasp_damping=10.0)
    full = lambda v: torch.full((n,), float(v))
    mech.grasp(torch.arange(n), torch.zeros(n, 3) if point is None else point, hinge=torch.full((n,), hinge),
               opening=opening.expand(n, 3), axis=axis.expand(n, 3), radius=full(radius), travel=full(travel),
               mass=full(mass), spring_n=full(spring_n), spring_k=full(spring_k), damping=full(damping),
               friction_static=full(static), friction_kinetic=full(kinetic), latch_n=full(latch),
               grasp_k=full(grasp_k), grip_n=full(grip))
    return mech


def push(mech, force_n, steps=1):
    """Hold the tool where the grasp spring drives the handle along its path with `force_n`,
    following it at its own speed (so the grasp's damping adds nothing)."""
    for _ in range(steps):
        tangent = mech.tangent()
        tool = mech.position() + (force_n / mech.grasp_k).unsqueeze(-1) * tangent
        mech.step(tool, mech.v.unsqueeze(-1) * tangent, DT)
    return mech


def at(x=0.0, y=0.0, z=0.0):
    return torch.tensor([[x, y, z]])


class MechanismTest(unittest.TestCase):
    def test_nothing_moves_until_stiction_is_passed(self):
        mech = push(make(static=10.0, kinetic=6.0), torch.tensor([9.0]), steps=100)
        self.assertEqual(float(mech.s[0]), 0.0)
        push(mech, torch.tensor([11.0]), steps=20)
        self.assertGreater(float(mech.s[0]), 0.0)
        self.assertGreater(float(mech.v[0]), 0.0)

    def test_a_latch_holds_then_lets_go_at_once(self):
        mech = push(make(static=2.0, kinetic=1.0, latch=20.0), torch.tensor([21.0]), steps=100)
        self.assertEqual(float(mech.s[0]), 0.0)
        self.assertTrue(bool(mech.latched[0]))
        push(mech, torch.tensor([23.0]))
        self.assertFalse(bool(mech.latched[0]))
        self.assertTrue(bool(mech.released[0]))
        # Released, the same force meets only kinetic friction: 22 N on 1 kg.
        push(mech, torch.tensor([23.0]), steps=10)
        self.assertGreater(float(mech.v[0]), 0.5 * 22.0 * 10 * DT)

    def test_the_spring_shuts_it_and_the_latch_catches_again(self):
        mech = push(make(spring_n=5.0, spring_k=100.0, static=1.0, kinetic=1.0, latch=10.0),
                    torch.tensor([30.0]), steps=100)
        self.assertGreater(float(mech.s[0]), 0.02)
        push(mech, torch.tensor([0.0]), steps=400)
        self.assertEqual(float(mech.s[0]), 0.0)
        self.assertTrue(bool(mech.latched[0]))

    def test_the_stops_keep_it_inside_its_travel(self):
        mech = push(make(travel=0.05), torch.tensor([40.0]), steps=200)
        self.assertAlmostEqual(float(mech.s[0]), 0.05, places=6)
        self.assertEqual(float(mech.v[0]), 0.0)
        shut = push(make(), torch.tensor([-40.0]), steps=50)
        self.assertEqual(float(shut.s[0]), 0.0)

    def test_friction_stops_a_motion_and_never_reverses_it(self):
        mech = make(static=3.0, kinetic=3.0, travel=1.0)
        mech.v[:] = 0.5
        push(mech, torch.tensor([0.0]), steps=100)
        self.assertEqual(float(mech.v[0]), 0.0)
        self.assertGreater(float(mech.s[0]), 0.0)

    def test_a_hinge_swings_on_its_circle_and_starts_along_its_opening(self):
        mech = make(hinge=True, opening=-X, axis=Z, radius=0.3, travel=0.3 * math.pi / 2, point=at(0.5, 0.0, 0.4))
        self.assertTrue(torch.allclose(mech.position(), at(0.5, 0.0, 0.4), atol=1e-6))
        self.assertTrue(torch.allclose(mech.tangent(), -X, atol=1e-6))
        for s in (0.05, 0.2, 0.3 * math.pi / 2):
            s = torch.tensor([s])
            p, t = mech.position(s), mech.tangent(s)
            self.assertAlmostEqual(float((p - mech.centre).norm()), 0.3, places=5)
            self.assertAlmostEqual(float((t * (p - mech.centre)).sum()), 0.0, places=5)
            self.assertAlmostEqual(float(t.norm()), 1.0, places=5)
            self.assertAlmostEqual(float(p[0, 2]), 0.4, places=6)
        # A quarter turn later the handle has swung 0.3 m back and 0.3 m toward its hinge.
        end = mech.position(torch.tensor([0.3 * math.pi / 2]))
        self.assertTrue(torch.allclose(end, mech.centre + 0.3 * -X, atol=1e-5))

    def test_the_grasp_holds_the_tool_on_the_path_and_drives_along_it(self):
        mech = make()
        on_tool = mech.step(at(y=0.01), torch.zeros(1, 3), DT)
        self.assertTrue(torch.allclose(on_tool, at(y=-20.0), atol=1e-4))
        self.assertAlmostEqual(float(mech.drive_n[0]), 0.0, places=6)
        mech = make()
        on_tool = mech.step(at(x=0.01), torch.zeros(1, 3), DT)
        self.assertAlmostEqual(float(mech.drive_n[0]), 20.0, places=4)
        self.assertTrue(torch.allclose(mechanism.Mechanism.applied_by_robot(mech), at(x=20.0), atol=1e-4))

    def test_tearing_the_handle_out_ends_the_grasp(self):
        mech = make(latch=500.0, grip=150.0)
        mech.step(at(x=0.08), torch.zeros(1, 3), DT)
        self.assertTrue(bool(mech.torn[0]))
        self.assertFalse(bool(mech.grasped[0]))
        self.assertEqual(float(mech.force_on_tool.norm()), 0.0)

    def test_the_stiffest_lightest_grasp_settles_at_the_physics_step(self):
        # 3,000 N/m on 0.3 kg: the handle pulled toward a fixed tool 1 cm ahead must settle, not ring.
        mech = make(mass=0.3, grasp_k=3000.0, travel=1.0)
        tool = at(x=0.01)
        excursions = []
        for step in range(600):
            mech.step(tool, torch.zeros(1, 3), DT)
            excursions.append(abs(float(mech.s[0]) - 0.01))
        self.assertLess(max(excursions[-100:]), 1e-4)
        self.assertLess(max(excursions), 0.011)

    def test_ungrasped_environments_feel_nothing(self):
        mech = make(n=2)
        mech.reset(torch.tensor([1]))
        on_tool = mech.step(at(x=0.05).expand(2, 3), torch.zeros(2, 3), DT)
        self.assertEqual(float(on_tool[1].norm()), 0.0)
        self.assertGreater(float(on_tool[0].norm()), 0.0)


class ResistanceAndAxesTest(unittest.TestCase):
    def test_the_profile_peaks_at_exactly_the_requested_force(self):
        torch.manual_seed(0)
        n = 200
        peak = 5.0 + 75.0 * torch.rand(n)
        latch = torch.rand(n) < 0.5
        profile = mechanism.resistance_profile(peak, latch=latch, preload_fraction=0.8 * torch.rand(n),
                                               kinetic_fraction=0.5 + 0.5 * torch.rand(n), weights=torch.rand(n, 3))
        travel = 0.02 + 0.2 * torch.rand(n)
        mech = mechanism.Mechanism(n, "cpu", grasp_damping=10.0)
        full = lambda v: torch.full((n,), float(v))
        mech.grasp(torch.arange(n), torch.zeros(n, 3), hinge=torch.zeros(n, dtype=torch.bool),
                   opening=X.expand(n, 3), axis=Z.expand(n, 3), radius=full(1.0), travel=travel, mass=full(1.0),
                   spring_n=profile["spring_closed_n"],
                   spring_k=(profile["spring_open_n"] - profile["spring_closed_n"]) / travel,
                   damping=full(0.0), friction_static=profile["friction_static"],
                   friction_kinetic=profile["friction_kinetic"], latch_n=profile["latch_n"],
                   grasp_k=full(2000.0), grip_n=full(150.0))
        self.assertTrue(torch.allclose(mech.peak_resistance(), peak, atol=1e-3))
        self.assertTrue(bool((profile["latch_n"][~latch] == 0.0).all()))
        self.assertTrue(bool((profile["friction_kinetic"] <= profile["friction_static"] + 1e-6).all()))

    def test_hinge_axes_are_perpendicular_to_the_opening(self):
        torch.manual_seed(1)
        opening = torch.randn(100, 3)
        opening[:5] = torch.tensor([0.0, 0.0, 1.0])
        opening = opening / opening.norm(dim=-1, keepdim=True)
        for vertical in (True, False):
            axis = mechanism.hinge_axes(opening, torch.full((100,), vertical), torch.rand(100) < 0.5)
            self.assertLess(float((axis * opening).sum(-1).abs().max()), 1e-5)
            self.assertTrue(torch.allclose(axis.norm(dim=-1), torch.ones(100), atol=1e-5))
        level = torch.tensor([[-1.0, 0.0, 0.0]])
        door = mechanism.hinge_axes(level, torch.tensor([True]), torch.tensor([False]))
        self.assertTrue(torch.allclose(door, Z, atol=1e-6))


class TravelGoalTest(unittest.TestCase):
    def test_slides_at_its_speed_holds_then_moves_on(self):
        goal = mechanism.TravelGoal(1, "cpu", speed_m_s=(0.1, 0.1), hold_s=(0.1, 0.1), first_fraction=(1.0, 1.0),
                                    dt=0.02)
        travel = torch.tensor([0.1])
        active = torch.tensor([True])
        goal.start(torch.tensor([0]), travel)
        goal.step(active, travel)
        self.assertAlmostEqual(float(goal.ref[0]), 0.002, places=6)
        for _ in range(49):
            goal.step(active, travel)
        self.assertAlmostEqual(float(goal.ref[0]), 0.1, places=5)
        for _ in range(10):
            goal.step(active, travel)
        self.assertTrue(0.0 <= float(goal.target[0]) <= 0.1)
        self.assertAlmostEqual(float(goal.first_target[0]), 0.1, places=6)

    def test_an_evaluation_plan_opens_fully_and_stays(self):
        goal = mechanism.TravelGoal(1, "cpu", speed_m_s=(0.05, 0.2), hold_s=(1.0, 3.0), first_fraction=(0.6, 1.0),
                                    dt=0.02)
        goal.use_plan(1.0, 0.1)
        travel = torch.tensor([0.03])
        goal.start(torch.tensor([0]), travel)
        for _ in range(500):
            goal.step(torch.tensor([True]), travel)
        self.assertAlmostEqual(float(goal.ref[0]), 0.03, places=6)
        self.assertAlmostEqual(float(goal.target[0]), 0.03, places=6)

    def test_inactive_environments_hold_still(self):
        goal = mechanism.TravelGoal(1, "cpu", speed_m_s=(0.1, 0.1), hold_s=(0.1, 0.1), first_fraction=(1.0, 1.0),
                                    dt=0.02)
        goal.start(torch.tensor([0]), torch.tensor([0.1]))
        goal.step(torch.tensor([False]), torch.tensor([0.1]))
        self.assertEqual(float(goal.ref[0]), 0.0)


class RewardTermsTest(unittest.TestCase):
    def state(self, error=0.0, speed=0.0, limit=0.2, force=0.0, held=1.0):
        blank = torch.zeros(1, 1)
        state = TaskState(*([blank] * 30))
        state.torques = torch.zeros(1, 20)
        state.fixture_engaged = torch.tensor([held])
        state.mech_error = torch.tensor([error])
        state.mech_speed = torch.tensor([speed])
        state.mech_speed_limit = torch.tensor([limit])
        state.mech_torn = torch.zeros(1)
        state.mech_force = torch.tensor([force])
        state.mech_drive = torch.zeros(1)
        state.mech_travel = torch.tensor([0.12])
        state.mech_command = torch.zeros(1)
        return state

    def test_progress_is_one_on_the_reference_and_zero_when_not_held(self):
        self.assertAlmostEqual(float(mech_rewards.mech_progress(self.state())[0]), 1.0, places=6)
        two_cm = float(mech_rewards.mech_progress(self.state(error=-0.02))[0])
        self.assertAlmostEqual(two_cm, 0.5 * math.exp(-1.0) + 0.5 * math.exp(-0.25), places=6)
        self.assertEqual(float(mech_rewards.mech_progress(self.state(held=0.0))[0]), 0.0)

    def test_overspeed_charges_only_past_the_limit(self):
        self.assertEqual(float(mech_rewards.mech_overspeed(self.state(speed=0.15))[0]), 0.0)
        self.assertAlmostEqual(float(mech_rewards.mech_overspeed(self.state(speed=-0.7))[0]), 0.25, places=6)

    def test_push_pays_for_force_toward_the_reference_and_never_for_lagging(self):
        state = self.state(error=-0.05)
        state.mech_drive = torch.tensor([50.0])
        behind = float(mech_rewards.mech_push(state)[0])
        self.assertAlmostEqual(behind, 1.0 - (0.5 * math.exp(-2.5) + 0.5 * math.exp(-0.625)), places=5)
        state.mech_drive = torch.tensor([-50.0])
        self.assertEqual(float(mech_rewards.mech_push(state)[0]), 0.0)
        # On the reference it pays nothing, and for any drive under the cap, closing the gap is worth
        # more than pushing from behind: total reward rises as the error shrinks.
        for drive in (20.0, 60.0, 99.0):
            totals = []
            for error in (-0.03, -0.02, -0.01, -0.005, 0.0):
                s = self.state(error=error)
                s.mech_drive = torch.tensor([drive])
                totals.append(mech_cfg.WEIGHTS["mech_progress"] * float(mech_rewards.mech_progress(s)[0])
                              + mech_cfg.WEIGHTS["mech_push"] * float(mech_rewards.mech_push(s)[0]))
            self.assertEqual(totals, sorted(totals))

    def test_a_short_mechanism_is_scored_against_its_own_travel(self):
        # A 1.5 cm button not pressed at all scores what a drawer 12 cm short does, near nothing; a
        # drawer's widths are the fixed 2 and 8 cm.
        button = self.state(error=-0.015)
        button.mech_travel = torch.tensor([0.015])
        expected = 0.5 * math.exp(-0.015 / 0.0045) + 0.5 * math.exp(-1.0)
        self.assertAlmostEqual(float(mech_rewards.mech_progress(button)[0]), expected, places=5)
        drawer = self.state(error=-0.02)
        self.assertAlmostEqual(float(mech_rewards.mech_progress(drawer)[0]),
                               0.5 * math.exp(-1.0) + 0.5 * math.exp(-0.25), places=6)
        # And pushing from behind still never beats getting there, on the short one too.
        for drive in (20.0, 60.0, 99.0):
            totals = []
            for error in (-0.015, -0.01, -0.005, -0.002, 0.0):
                s = self.state(error=error)
                s.mech_travel = torch.tensor([0.015])
                s.mech_drive = torch.tensor([drive])
                totals.append(mech_cfg.WEIGHTS["mech_progress"] * float(mech_rewards.mech_progress(s)[0])
                              + mech_cfg.WEIGHTS["mech_push"] * float(mech_rewards.mech_push(s)[0]))
            self.assertEqual(totals, sorted(totals))

    def test_overforce_charges_only_past_the_command_and_its_margin(self):
        state = self.state()
        state.mech_command = torch.tensor([40.0])
        state.mech_drive = torch.tensor([48.0])
        self.assertEqual(float(mech_rewards.mech_overforce(state)[0]), 0.0)
        state.mech_drive = torch.tensor([80.0])
        self.assertAlmostEqual(float(mech_rewards.mech_overforce(state)[0]), 9.0, places=5)

    def test_effort_is_the_squared_force_per_hundred_newtons(self):
        self.assertAlmostEqual(float(mech_rewards.mech_effort(self.state(force=60.0))[0]), 0.36, places=6)

    def test_the_arm_margin_is_the_pull_tasks_term(self):
        self.assertIs(mech_rewards.TERMS["arm_torque_margin"], hook_rewards.arm_torque_margin)
        self.assertEqual(set(mech_rewards.TERMS), set(mech_cfg.WEIGHTS))


class ScoringTest(unittest.TestCase):
    def test_capacity_is_the_last_level_opened_without_a_gap(self):
        steps, envs = 60, 2
        trace = {key: np.zeros((steps, envs)) for key in mech_eval.FIELDS}
        trace["arm_loads"] = np.zeros((steps, envs, 6))
        trace["estimate_drive"][:] = np.nan
        trace["travel"][:] = 0.1
        trace["grasped"][5:] = 1.0
        # Episode 0 opens (reaches 0.09 at step 45); episode 1 stalls at 3 cm.
        trace["s"][5:, 0] = np.linspace(0.0, 0.11, steps - 5).clip(max=0.1)
        trace["s"][5:, 1] = 0.03
        trace["drive"][5:, 0] = 12.0
        trace["drive"][20, 0] = 18.0
        trace["drive"][5:, 1] = 15.0
        result = mech_eval.score(trace, ["drawer"], [10.0, 20.0], placements_per_level=1)
        first, second = result["episodes"]
        self.assertTrue(first["opened"])
        self.assertFalse(second["opened"])
        self.assertEqual(first["peak_drive_n"], 18.0)
        opened_at = int(np.flatnonzero(trace["s"][:, 0] >= 0.09)[0])
        self.assertAlmostEqual(first["open_time_s"], (opened_at - 5) * interface.POLICY_DT, places=6)
        self.assertEqual(result["summary"]["kinds"]["drawer"]["capacity_n"], 10.0)

    def test_the_plan_covers_every_kind_level_and_placement(self):
        handles, kinds, levels = mech_eval.build_plan(["latch", "door"], [10.0, 20.0], "cpu")
        self.assertEqual(len(handles), 2 * 2 * 15)
        names = list(mech_cfg.EVAL_KINDS)
        self.assertEqual(int(kinds[0]), names.index("latch"))
        self.assertEqual(int(kinds[-1]), names.index("door"))
        self.assertEqual(float(levels[15]), 20.0)


if __name__ == "__main__":
    unittest.main()

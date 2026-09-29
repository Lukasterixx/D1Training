"""The pressing task's button, tools and task layer (`unifp_train/pressing.py`), without a simulator.

Each contact case is pinned: the free approach, the preload, the spring's travel, the click, the stop, the panel beside
the cap, a recessed cap that the bare jaws cannot follow and a probe can, friction, going through the panel; then the
task layer pressing a button it has estimated right, too near and too far, and one it cannot press.
"""
from __future__ import annotations

import math
import unittest

import torch

from unifp_train import pressing

N = torch.tensor([[1.0, 0.0, 0.0]])          # the button's axis: into the panel along +x
K = pressing.CONTACT_K_N_PER_M


def button(n=1, cap_radius=0.010, proud=0.003, travel=0.005, preload=3.0, stiffness=1200.0, damping=0.0,
           snap_n=0.0, snap_at=0.6, mu=0.4):
    b = pressing.Button(n, "cpu")
    full = lambda v: torch.full((n,), float(v))
    b.place(torch.arange(n), face=torch.zeros(n, 3), axis=N.expand(n, 3), cap_radius=full(cap_radius),
            proud=full(proud), travel=full(travel), preload_n=full(preload), stiffness=full(stiffness),
            damping=full(damping), snap_n=full(snap_n), snap_at=full(snap_at), mu=full(mu))
    return b


def press_to(b, front, radius=0.01, lateral=0.0, steps=200, dt=0.005):
    """Move the sphere's front from 5 mm short of the face to `front` (m past the cap's rest face), slowly; the force."""
    force = None
    start = -0.005
    for i in range(steps):
        x = start + (front - start) * (i + 1) / steps
        centre = torch.tensor([[x - radius, lateral, 0.0]])
        vel = torch.tensor([[(front - start) / (steps * dt), 0.0, 0.0]])
        force = b.step(centre, vel, torch.tensor([radius]), dt)
    return force


class ButtonTest(unittest.TestCase):
    def test_nothing_touches_short_of_the_cap(self):
        b = button()
        force = press_to(b, -0.001)
        self.assertLess(float(force.norm()), 1e-9)
        self.assertEqual(float(b.s[0]), 0.0)

    def test_below_the_preload_the_cap_does_not_move(self):
        b = button(preload=3.0)
        press_to(b, 0.0001)                                     # 1.6 N into the contact spring
        self.assertEqual(float(b.s[0]), 0.0)
        self.assertAlmostEqual(float(b.normal_n[0]), K * 0.0001, delta=0.3)

    def test_through_its_travel_the_force_is_the_springs(self):
        b = button(preload=3.0, stiffness=1200.0)
        force = press_to(b, 0.003, steps=2000, dt=0.005)       # slow: damping negligible
        s = float(b.s[0])
        self.assertGreater(s, 0.0025)
        self.assertLess(s, 0.003)
        self.assertAlmostEqual(-float(force[0, 0]), 3.0 + 1200.0 * s, delta=0.1)
        self.assertFalse(bool(b.pressed()[0]))

    def test_at_its_stop_it_is_pressed_and_the_stop_pushes_back(self):
        b = button(travel=0.005)
        # The contact spring compresses by resistance / K before the cap bottoms: 9 N / 16,000 N/m = 0.56 mm.
        force = press_to(b, 0.0055, steps=2000)
        self.assertLess(float(b.s[0]), 0.005)
        self.assertTrue(bool(b.pressed()[0]))                   # within the 0.5 mm tolerance
        force = press_to(b, 0.0060, steps=2000)
        self.assertAlmostEqual(float(b.s[0]), 0.005, places=6)
        self.assertAlmostEqual(-float(force[0, 0]), K * 0.001, delta=0.2)
        self.assertAlmostEqual(float(b.full_travel_n()[0]), 3.0 + 1200.0 * 0.005, places=5)

    def test_the_click_rises_then_lets_go(self):
        b = button(preload=2.0, stiffness=800.0, snap_n=4.0, snap_at=0.6, travel=0.005)
        forces, positions = [], []
        start, end, steps, dt = -0.002, 0.0048, 3000, 0.005
        for i in range(steps):
            x = start + (end - start) * (i + 1) / steps
            f = b.step(torch.tensor([[x - 0.01, 0.0, 0.0]]), torch.tensor([[(end - start) / (steps * dt), 0.0, 0.0]]),
                       torch.tensor([0.01]), dt)
            forces.append(-float(f[0, 0]))
            positions.append(float(b.s[0]))
        peak = max(forces)
        at_peak = positions[forces.index(peak)]
        # The peak is the click's, just before 60% of the travel, and the force falls away after it.
        self.assertAlmostEqual(peak, 2.0 + 800.0 * at_peak + 4.0, delta=0.3)
        self.assertLess(at_peak, 0.6 * 0.005 + 1e-4)
        self.assertGreater(at_peak, 0.6 * 0.005 - pressing.SNAP_WIDTH_M)
        after = [f for f, s in zip(forces, positions) if 0.0032 < s < 0.0034]
        self.assertLess(max(after), peak - 3.0)
        self.assertAlmostEqual(float(b.operating_n()[0]), 2.0 + 800.0 * 0.003 + 4.0, places=5)

    def test_off_the_cap_the_tool_presses_the_panel_and_the_cap_stays(self):
        b = button(cap_radius=0.010, proud=0.003)
        force = press_to(b, 0.004, lateral=0.02)               # 1 mm into the panel's face
        self.assertEqual(float(b.s[0]), 0.0)
        self.assertFalse(bool(b.touched_cap[0]))
        self.assertAlmostEqual(-float(force[0, 0]), K * 0.001, delta=0.5)

    def test_the_cap_returns_when_the_tool_withdraws(self):
        b = button()
        press_to(b, 0.004, steps=1000)
        self.assertGreater(float(b.s[0]), 0.003)
        press_to(b, -0.004, steps=10)
        self.assertEqual(float(b.s[0]), 0.0)

    def test_friction_opposes_sliding_across_the_cap(self):
        b = button(mu=0.5)
        press_to(b, 0.002, steps=500)
        f = b.step(torch.tensor([[0.002 - 0.01, 0.0, 0.0]]), torch.tensor([[0.0, 0.05, 0.0]]), torch.tensor([0.01]), 0.005)
        self.assertLess(float(f[0, 1]), 0.0)
        self.assertAlmostEqual(-float(f[0, 1]), 0.5 * float(b.normal_n[0]) * math.tanh(0.05 / 0.02), delta=0.05)

    def test_driving_through_the_panel_is_damage(self):
        b = button()
        press_to(b, 0.003 + pressing.FACE_DEPTH_M + 0.001, lateral=0.03, steps=50)
        self.assertTrue(bool(b.pierced[0]))

    def push_with(self, b, radius, force_n=15.0, seconds=3.0, dt=0.005, admittance=1e-4):
        """Push the sphere in along the axis with a steady `force_n` through a finite admittance; where its front ends."""
        front = torch.tensor([-0.006])
        vel = torch.zeros(1, 3)
        for _ in range(int(seconds / dt)):
            centre = torch.stack((front - radius, torch.zeros(1), torch.zeros(1)), dim=-1)
            contact = b.step(centre, vel, torch.tensor([radius]), dt)
            step = admittance * (force_n + contact[:, 0]) * dt / 0.02
            vel = torch.stack((step / dt, torch.zeros(1), torch.zeros(1)), dim=-1)
            front = front + step
        return float(front[0])

    def test_the_bare_jaws_cannot_follow_a_small_recessed_cap_and_the_probe_can(self):
        small = dict(cap_radius=0.004, proud=-0.002, travel=0.003, preload=2.0, stiffness=1000.0)
        bare, probe = pressing.TOOLS["bare"], pressing.TOOLS["probe"]
        reach = pressing.reachable_front(torch.tensor([bare.radius_m]), torch.tensor([0.004]), torch.tensor([-0.002]))
        self.assertLess(float(reach[0]), 0.0)                    # resting on the rim, short of even the cap at rest
        b = button(**small)
        front = self.push_with(b, bare.radius_m)
        self.assertFalse(bool(b.touched_cap[0]))
        self.assertFalse(bool(b.pressed()[0]))
        self.assertAlmostEqual(front, float(reach[0]), delta=0.0015)    # held on the rim, where the geometry says
        self.assertTrue(math.isinf(float(pressing.reachable_front(torch.tensor([probe.radius_m]), torch.tensor([0.004]),
                                                                   torch.tensor([-0.002]))[0])))
        b = button(**small)
        self.push_with(b, probe.radius_m)
        self.assertTrue(bool(b.pressed()[0]))

    def test_the_physics_rate_peak_is_kept_until_taken(self):
        b = button()
        press_to(b, 0.006, steps=20)
        peak = float(b.take_peak()[0])
        self.assertGreaterEqual(peak, float(b.normal_n[0]))
        self.assertEqual(float(b.peak_n[0]), 0.0)


class PressTaskLayerTest(unittest.TestCase):
    """The task layer against a stand-in hand: along the axis it moves under the commanded and contact forces with a
    finite admittance, across it tracks the goal -- the insertion tests' hand."""

    def run_loop(self, *, face_error=0.0, lateral_error=0.0, seconds=8.0, admittance=1e-4, max_n=20.0, hold_n=12.0,
                 **button_kwargs):
        dt = 0.005
        b = button(**button_kwargs)
        layer = pressing.PressTaskLayer(1, "cpu")
        # The layer's estimate of the face is `face_error` short of the truth (positive: the button is further), and
        # `lateral_error` to the side of it.
        layer.start(torch.arange(1), face=torch.tensor([[-face_error, lateral_error, 0.0]]), axis=N,
                    standoff=torch.tensor([0.01]), travel=b.travel.clone(), overdrive=torch.tensor([0.004]),
                    speed=torch.tensor([0.02]), touch_n=torch.tensor([3.0]), max_n=torch.tensor([max_n]),
                    hold_n=torch.tensor([hold_n]), hold_s=torch.tensor([1.5]))
        radius = torch.tensor([0.01])
        tip = torch.tensor([[-face_error - 0.01, lateral_error, 0.0]])
        vel = torch.zeros(1, 3)
        pressed_for, longest, peak, peak_push = 0.0, 0.0, 0.0, 0.0
        for _ in range(int(seconds / dt)):
            goal, force_cmd = layer.step(tip, dt)
            contact = b.step(tip - radius * N, vel, radius, dt)
            new = tip + 0.3 * (goal - tip)
            if int(layer.phase[0]) <= pressing.HOLDING:
                new[:, 0] = tip[:, 0] + admittance * (force_cmd[:, 0] + contact[:, 0]) * dt / 0.02
            vel = (new - tip) / dt
            tip = new
            pressed_for = pressed_for + dt if bool(b.pressed()[0]) else 0.0
            longest = max(longest, pressed_for)
            peak = max(peak, float(b.normal_n[0]))
            peak_push = max(peak_push, float(layer.push[0]))
        return b, layer, longest, peak, peak_push

    def test_a_well_estimated_button_is_pressed_held_and_released(self):
        b, layer, longest, peak, peak_push = self.run_loop()
        self.assertGreaterEqual(longest, 1.0)
        self.assertEqual(int(layer.phase[0]), pressing.DONE)
        self.assertFalse(bool(b.pressed()[0]))                   # released: the cap is back up
        self.assertLessEqual(peak_push, 20.0 + 1e-6)

    def test_a_button_further_than_estimated_is_still_pressed(self):
        _, _, longest, _, _ = self.run_loop(face_error=0.003)
        self.assertGreaterEqual(longest, 1.0)

    def test_a_button_nearer_than_estimated_is_pressed_and_found_by_the_stall(self):
        _, layer, longest, _, _ = self.run_loop(face_error=-0.003)
        self.assertGreaterEqual(longest, 1.0)
        self.assertTrue(bool(layer.touched[0]))

    def test_the_push_is_light_until_something_is_touched(self):
        layer = pressing.PressTaskLayer(1, "cpu")
        layer.start(torch.arange(1), face=torch.zeros(1, 3), axis=N, standoff=torch.tensor([0.01]),
                    travel=torch.tensor([0.005]), overdrive=torch.tensor([0.004]), speed=torch.tensor([0.05]),
                    touch_n=torch.tensor([3.0]), max_n=torch.tensor([20.0]), hold_n=torch.tensor([12.0]),
                    hold_s=torch.tensor([1.5]))
        tip = torch.tensor([[-0.01, 0.0, 0.0]])                  # a hand that does not move
        pushes = []
        for _ in range(40):                                      # 0.2 s: too soon to have stalled
            _, cmd = layer.step(tip, 0.005)
            pushes.append(float(cmd[0, 0]))
        self.assertLessEqual(max(pushes), 3.0 + 1e-6)
        self.assertFalse(bool(layer.touched[0]))

    def test_a_button_stiffer_than_the_cap_is_not_pressed_and_the_push_stays_capped(self):
        b, layer, longest, peak, peak_push = self.run_loop(stiffness=6000.0, preload=5.0, max_n=20.0, hold_n=14.0)
        self.assertEqual(longest, 0.0)
        self.assertLessEqual(peak_push, 20.0 + 1e-6)
        self.assertLess(peak, 30.0)

    def test_a_tool_that_misses_the_cap_presses_the_panel_and_never_the_button(self):
        b, _, longest, _, _ = self.run_loop(lateral_error=0.015)
        self.assertEqual(longest, 0.0)
        self.assertFalse(bool(b.touched_cap[0]))


class EvaluationTest(unittest.TestCase):
    """The frozen sets and the scoring, on synthetic traces."""

    def test_the_frozen_sets_are_the_sizes_they_were_frozen_at(self):
        from unifp_train import press_eval
        for kind, size in (("panel", 96), ("force", 48), ("recessed", 72)):
            plan, rows = press_eval.build_plan(kind, "cpu")
            self.assertEqual(len(rows), size, kind)
            self.assertEqual(plan["face"].shape, (size, 3))
            self.assertEqual(sorted({r["tool"] for r in rows}), sorted(pressing.TOOL_NAMES))
        _, rows = press_eval.build_plan("force", "cpu")
        self.assertEqual(sorted({r["force_n"] for r in rows}), [5.0, 10.0, 20.0, 30.0])

    def trace(self, steps=200, envs=1):
        from unifp_train import press_eval
        t = {key: [[0.0] * envs for _ in range(steps)] for key in press_eval.FIELDS}
        import numpy as np
        return {key: np.array(value, dtype=float) for key, value in t.items()}

    def fill(self, trace, key, start, stop, value=1.0, env=0):
        trace[key][start:stop, env] = value

    def test_held_a_second_is_success_and_less_is_not_held(self):
        from unifp_train import press_eval
        for stop, expected in ((80, "success"), (60, "not held")):
            tr = self.trace()
            self.fill(tr, "engaged", 5, 200)
            self.fill(tr, "on_cap", 12, stop)
            self.fill(tr, "over_cap", 0, 200)
            self.fill(tr, "pressed", 20, stop)
            result = press_eval.score_episode(tr, 0)
            self.assertEqual(result["reason"], expected)
            self.assertAlmostEqual(result["time_to_press_s"], 15 * 0.02, places=6)

    def test_the_failure_reasons(self):
        from unifp_train import press_eval
        tr = self.trace()
        self.assertEqual(press_eval.score_episode(tr, 0)["reason"], "not started")
        self.fill(tr, "engaged", 5, 200)
        self.assertEqual(press_eval.score_episode(tr, 0)["reason"], "missed")
        self.fill(tr, "on_cap", 20, 30)
        self.fill(tr, "over_cap", 0, 30)                          # then the tool leaves the cap while pushing
        self.assertEqual(press_eval.score_episode(tr, 0)["reason"], "slid off")
        self.fill(tr, "over_cap", 0, 200)
        self.assertEqual(press_eval.score_episode(tr, 0)["reason"], "short")
        self.fill(tr, "pressed", 40, 70)                          # at full travel for 0.6 s, on the cap throughout
        self.assertEqual(press_eval.score_episode(tr, 0)["reason"], "not held")
        self.fill(tr, "over_cap", 80, 120, 0.0)                    # ... and then off it, still pushing
        self.assertEqual(press_eval.score_episode(tr, 0)["reason"], "slid off")
        self.fill(tr, "fallen", 150, 151)
        self.assertEqual(press_eval.score_episode(tr, 0)["reason"], "fell")

    def test_the_force_error_is_applied_against_commanded_while_pushing(self):
        from unifp_train import press_eval
        tr = self.trace()
        self.fill(tr, "engaged", 0, 200)
        self.fill(tr, "push_n", 0, 200, 10.0)
        self.fill(tr, "applied_n", 0, 200, 7.0)
        self.fill(tr, "phase", 100, 200, float(pressing.RELEASING))  # released: no longer counted
        self.fill(tr, "applied_n", 100, 200, 0.0)
        self.assertAlmostEqual(press_eval.score_episode(tr, 0)["force_rmse_n"], 3.0, places=5)


class RewardTest(unittest.TestCase):
    def test_once_started_the_stillness_term_prices_only_motion_across_the_axis(self):
        from types import SimpleNamespace

        from unifp_train import press_rewards
        v = torch.tensor([[0.3, 0.1, 0.0], [0.3, 0.1, 0.0]])
        state = SimpleNamespace(tool_vel_w=v, fixture_axis_w=N.expand(2, 3), fixture_engaged=torch.tensor([1.0, 0.0]),
                                press_holding=torch.tensor([1.0, 1.0]))
        cost = press_rewards.press_tool_speed(state)
        self.assertAlmostEqual(float(cost[0]), 0.01, places=6)
        self.assertAlmostEqual(float(cost[1]), 0.10, places=6)

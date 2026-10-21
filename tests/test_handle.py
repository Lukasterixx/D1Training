"""The door handle and its task layer (`unifp_train/handle.py`), without a simulator.

The lever's geometry and its inverse, the spring's preload holding the lever on its rest stop, the angle a steady foot
force turns it to, the end stop, the door's face, the foot sliding off the bar's end, and the task layer: the goal it
hands a position-only policy, and the force it commands a force-aware one -- along the arc, bounded by its cap.
"""
from __future__ import annotations

import math
import unittest

import torch

from unifp_train import foot_goals, handle, leg_cfg

DT = 0.005


def lever(n=1, rest=0.5, full=2.0, theta_max=math.radians(50.0), latch=0.7, length=0.12, radius=0.01,
          standoff=0.06, damping=0.01, friction=0.0, mu=0.6, inertia=1e-3):
    """A lever pointing +y (the robot's left) on a door 0.6 m ahead at 0.30 m, facing the robot (-x)."""
    h = handle.LeverHandle(n, "cpu")
    full_of = lambda v: torch.full((n,), float(v))
    h.place(torch.arange(n), door=torch.tensor([[0.6, -0.2, 0.30]]).expand(n, 3),
            normal=torch.tensor([[-1.0, 0.0, 0.0]]).expand(n, 3), u0=torch.tensor([[0.0, 1.0, 0.0]]).expand(n, 3),
            standoff=full_of(standoff), length=full_of(length), inner=full_of(0.03), radius=full_of(radius),
            inertia=full_of(inertia), rest_nm=full_of(rest), full_nm=full_of(full), damping=full_of(damping),
            friction=full_of(friction), theta_max=full_of(theta_max), theta_latch=full_of(latch * theta_max),
            mu=full_of(mu))
    return h


def push_down(h, force_n, reach=0.09, steps=600):
    """Hold a steady downward force on the bar at `reach`, by servoing a foot onto it: the foot sits where the contact
    spring carries `force_n` (a stiff force-controlled foot)."""
    clearance = torch.tensor([handle.FOOT_RADIUS_M + 0.01])
    reach_t = torch.tensor([reach])
    for _ in range(steps):
        rest = handle.foot_on_bar(h.pivot, h.u0, h.theta, reach_t, clearance)
        sink = force_n / handle.CONTACT_K_N_PER_M
        foot = rest - sink * handle.lever_top(h.u0, h.theta)
        h.step(foot, torch.zeros(1, 3), DT)
    return h


class Geometry(unittest.TestCase):
    def test_measured_theta_inverts_foot_on_bar(self):
        h = lever()
        reach, clearance = torch.tensor([0.09]), torch.tensor([0.032])
        for degrees in (0.0, 10.0, 35.0, 60.0):
            theta = torch.tensor([math.radians(degrees)])
            foot = handle.foot_on_bar(h.pivot, h.u0, theta, reach, clearance)
            got = handle.measured_theta(foot, h.pivot, h.u0, reach, clearance)
            self.assertAlmostEqual(float(got), float(theta), places=5)

    def test_lever_turns_down(self):
        h = lever()
        down = handle.lever_dir(h.u0, torch.tensor([math.radians(40.0)]))
        self.assertLess(float(down[0, 2]), -0.6)
        self.assertGreater(float(down[0, 1]), 0.7)


class Plant(unittest.TestCase):
    def test_preload_holds_the_lever_on_its_stop(self):
        h = lever(rest=1.0)
        push_down(h, force_n=0.8 / 0.09)          # 0.8 N·m against a 1.0 N·m preload
        self.assertLess(abs(float(h.theta)), 0.005)

    def test_a_steady_force_turns_it_to_the_spring_balance(self):
        h = lever(rest=0.5, full=2.0, theta_max=math.radians(50.0), damping=0.02)
        torque = 1.25                              # half way up the spring: 25 degrees
        push_down(h, force_n=torque / 0.09, steps=1200)
        expected = (torque - 0.5) / ((2.0 - 0.5) / math.radians(50.0))
        self.assertAlmostEqual(float(h.theta), expected, delta=math.radians(3.0))
        self.assertFalse(bool(h.latch_open()))

    def test_the_end_stop_holds_and_the_latch_opens(self):
        h = lever(full=2.0, theta_max=math.radians(50.0), latch=0.7)
        push_down(h, force_n=40.0, steps=1200)
        self.assertLess(float(h.theta), math.radians(52.0))
        self.assertGreater(float(h.theta), math.radians(48.0))
        self.assertTrue(bool(h.latch_open()))
        self.assertGreater(float(h.bar_n), 35.0)

    def test_the_door_face_pushes_a_foot_back_out(self):
        h = lever()
        foot = torch.tensor([[0.6 - handle.FOOT_RADIUS_M + 0.004, -0.5, 0.30]])   # 4 mm into the face, off the bar
        force = h.step(foot, torch.zeros(1, 3), DT)
        self.assertAlmostEqual(float(force[0, 0]), -handle.CONTACT_K_N_PER_M * 0.004, delta=1.0)
        self.assertAlmostEqual(float(h.bar_n), 0.0)

    def test_a_fast_strike_is_bounded_by_the_spring(self):
        h = lever()
        foot = handle.foot_on_bar(h.pivot, h.u0, torch.zeros(1), torch.tensor([0.09]), torch.tensor([0.032 - 0.001]))
        # 1 mm in, closing at 5 m/s: unbounded, the damping alone would be 250 N.
        _, _, _, f_n, _ = h._bar_contact(foot, torch.tensor([[0.0, 0.0, -5.0]]), handle.FOOT_RADIUS_M)
        self.assertAlmostEqual(float(f_n), 2.0 * handle.CONTACT_K_N_PER_M * 0.001, delta=1.0)

    def test_overlap_is_capped(self):
        h = lever()
        foot = handle.foot_on_bar(h.pivot, h.u0, torch.zeros(1), torch.tensor([0.09]), torch.tensor([0.0]))
        force = h.step(foot, torch.zeros(1, 3), DT)
        self.assertLessEqual(float(force.norm()), handle.CONTACT_K_N_PER_M * handle.MAX_OVERLAP_M * 1.5)

    def test_a_foot_past_the_end_of_the_bar_does_not_touch_it(self):
        h = lever(length=0.12)
        foot = handle.foot_on_bar(h.pivot, h.u0, torch.zeros(1), torch.tensor([0.12 + 0.04]), torch.tensor([0.02]))
        h.step(foot, torch.zeros(1, 3), DT)
        self.assertFalse(bool(h.on_bar))

    def test_inactive_handles_do_nothing(self):
        h = handle.LeverHandle(2, "cpu")
        force = h.step(torch.zeros(2, 3), torch.zeros(2, 3), DT)
        self.assertEqual(float(force.abs().sum()), 0.0)


class Layer(unittest.TestCase):
    def _layer(self, force_law: bool, max_n=20.0):
        h = lever()
        layer = handle.HandleTaskLayer(1, "cpu", kp=leg_cfg.FORCE_LAW_KP_N_PER_M, ki=leg_cfg.FORCE_LAW_KI_N_PER_M_S)
        full = lambda v: torch.full((1,), float(v))
        layer.start(torch.arange(1), pivot=h.pivot, u0=h.u0, reach=full(0.09), clearance=full(0.032),
                    theta_end=full(math.radians(55.0)), descent=full(0.06), descend_speed=full(0.1),
                    turn_speed=full(1.0), max_n=full(max_n), touch_n=full(4.0), hold_s=full(0.5), release_s=full(0.5),
                    force_law=torch.tensor([force_law]))
        return h, layer

    def test_position_only_gets_a_goal_and_no_force(self):
        h, layer = self._layer(force_law=False)
        foot = layer.pre_contact()
        for _ in range(100):
            goal, force = layer.step(foot, 0.02)
            self.assertEqual(float(force.abs().sum()), 0.0)
        # 2 s in: down 6 cm (0.6 s) and round the arc at 1 rad/s to the end (0.96 rad): holding at the end.
        self.assertEqual(int(layer.phase), handle.HOLDING)
        self.assertAlmostEqual(float(layer.theta_ref), math.radians(55.0), places=4)

    def test_force_law_pushes_along_the_arc_and_is_capped(self):
        h, layer = self._layer(force_law=True, max_n=20.0)
        stuck = layer.pre_contact() - torch.tensor([[0.0, 0.0, 0.06]])    # the foot resting on the unturned bar
        magnitudes = []
        for _ in range(100):
            _, force = layer.step(stuck, 0.02)
            magnitudes.append(float(force.norm()))
        self.assertAlmostEqual(max(magnitudes), 20.0, places=4)
        # Along the arc's tangent at the reference: into the bar's top, mostly downward.
        direction = force / force.norm()
        self.assertAlmostEqual(float((direction * -handle.lever_top(layer.u0, layer.theta_ref)).sum()), 1.0, places=5)

    def test_touch_then_turn(self):
        h, layer = self._layer(force_law=True, max_n=20.0)
        layer.touch_s[:] = 0.5
        stuck = layer.pre_contact() - torch.tensor([[0.0, 0.0, 0.06]])
        forces, refs = [], []
        for _ in range(120):
            _, force = layer.step(stuck, 0.02)
            forces.append(float(force.norm()))
            refs.append(float(layer.theta_ref))
        # 0.6 s down to the bar, then 0.5 s waiting there at the touch force, then the turn at up to the cap.
        self.assertAlmostEqual(max(forces[31:52]), 4.0, places=4)
        self.assertAlmostEqual(max(refs[31:52]), 0.0, places=6)
        self.assertGreater(refs[-1], 0.05)
        self.assertAlmostEqual(max(forces), 20.0, places=4)

    def test_release_returns_to_the_pre_contact_point(self):
        h, layer = self._layer(force_law=True)
        foot = layer.pre_contact()
        for _ in range(250):
            goal, force = layer.step(foot, 0.02)
        self.assertEqual(int(layer.phase), handle.DONE)
        self.assertLess(float((goal - layer.pre_contact()).norm()), 1e-5)
        self.assertEqual(float(force.abs().sum()), 0.0)


def key(friction=1.5, detent=0.0, inertia=6e-4, gravity=True, mu=0.7):
    """Lukas's lever key in a lock on a door 0.6 m ahead facing the robot (-x), its shaft 0.42 m up, pointing to the
    robot's right (-y) when level; it starts 45 degrees up."""
    h = handle.LeverHandle(1, "cpu")
    one = lambda v: torch.full((1,), float(v))
    h.place(torch.arange(1), door=torch.tensor([[0.6, 0.0, 0.42]]), normal=torch.tensor([[-1.0, 0.0, 0.0]]),
            u0=torch.tensor([[0.0, -1.0, 0.0]]), standoff=one(leg_cfg.KEY_AIM_OFF_DOOR_M), length=one(0.16),
            inner=one(0.0), radius=one(leg_cfg.KEY_TOP_M), inertia=one(inertia), rest_nm=one(0.0), full_nm=one(0.0),
            damping=one(0.01), friction=one(friction), theta_max=one(leg_cfg.KEY_THETA_END_RAD),
            theta_latch=one(leg_cfg.KEY_THETA_UNLOCK_RAD), mu=one(mu), theta_min=one(leg_cfg.KEY_THETA_START_RAD),
            pad=True, detent_nm=one(detent), detent_from=one(leg_cfg.KEY_THETA_UNLOCK_RAD - leg_cfg.KEY_DETENT_SPAN_RAD),
            gravity_nm=one(leg_cfg.KEY_GRAVITY_NM if gravity else 0.0))
    return h


def press_key(h, force_n, reach=0.10, steps=600):
    """A stiff force-controlled foot on the key's platform at `reach`: it sits where the contact spring carries
    `force_n` along the platform's normal."""
    clearance, reach_t = torch.tensor([handle.FOOT_RADIUS_M + leg_cfg.KEY_TOP_M]), torch.tensor([reach])
    for _ in range(steps):
        rest = handle.foot_on_bar(h.pivot, h.u0, h.theta, reach_t, clearance)
        foot = rest - (force_n / handle.CONTACT_K_N_PER_M) * handle.lever_top(h.u0, h.theta)
        h.step(foot, torch.zeros(1, 3), DT)
    return h


class Key(unittest.TestCase):
    def test_it_starts_pointing_up_on_its_stop(self):
        h = key()
        self.assertAlmostEqual(float(h.theta), leg_cfg.KEY_THETA_START_RAD, places=6)
        up = handle.lever_dir(h.u0, h.theta)
        self.assertGreater(float(up[0, 2]), 0.7)           # up
        self.assertLess(float(up[0, 1]), -0.7)             # and to the robot's right

    def test_the_lock_holds_below_its_friction(self):
        h = press_key(key(friction=1.5), force_n=12.0, reach=0.10)        # 1.2 N·m + its weight's 0.025, under 1.5
        self.assertLess(abs(float(h.theta) - leg_cfg.KEY_THETA_START_RAD), math.radians(0.5))
        self.assertFalse(bool(h.latch_open()))

    def test_it_turns_past_the_unlock_to_its_stop_and_stays_there(self):
        h = press_key(key(friction=1.5, detent=1.0), force_n=30.0, reach=0.10, steps=1500)
        self.assertGreater(float(h.theta), math.radians(43.0))
        self.assertLess(float(h.theta), math.radians(47.0))
        self.assertTrue(bool(h.latch_open()))
        away = torch.tensor([[0.3, -0.3, 0.3]])                         # the foot gone: no return spring
        for _ in range(400):
            h.step(away, torch.zeros(1, 3), DT)
        self.assertGreater(float(h.theta), math.radians(43.0))

    def test_the_detent_needs_more(self):
        light = press_key(key(friction=1.0, detent=0.0), force_n=17.0, steps=1500)
        cammed = press_key(key(friction=1.0, detent=1.0), force_n=17.0, steps=1500)   # 1.7 N·m: past 1.0, not 2.0
        self.assertTrue(bool(light.latch_open()))
        self.assertFalse(bool(cammed.latch_open()))
        self.assertGreater(float(cammed.theta), math.radians(5.0))      # turned as far as the cam

    def test_the_platform_pushes_back_along_its_normal(self):
        h = key()
        top = handle.lever_top(h.u0, h.theta)
        # 2 mm into the platform's top, 0.1 m along the key, 35 mm off the door's face.
        foot = handle.foot_on_bar(h.pivot, h.u0, h.theta, torch.tensor([0.10]),
                                  torch.tensor([handle.FOOT_RADIUS_M + leg_cfg.KEY_TOP_M - 0.002]))
        f, _, touching, f_n, along = h._pad_contact(foot, torch.zeros(1, 3), handle.FOOT_RADIUS_M)
        self.assertTrue(bool(touching))
        self.assertAlmostEqual(float(f_n), handle.CONTACT_K_N_PER_M * 0.002, delta=0.5)
        self.assertAlmostEqual(float((f[0] / f[0].norm() * top[0]).sum()), 1.0, places=4)
        self.assertAlmostEqual(float(along), 0.10, places=4)

    def test_a_foot_past_the_platforms_edges_does_not_touch_it(self):
        h = key()
        clearance = torch.tensor([handle.FOOT_RADIUS_M + leg_cfg.KEY_TOP_M - 0.002])
        beyond_tip = handle.foot_on_bar(h.pivot, h.u0, h.theta, torch.tensor([0.16 + 0.03]), clearance)
        off_front = handle.foot_on_bar(h.pivot, h.u0, h.theta, torch.tensor([0.10]), clearance) \
            + 0.05 * h.n                                                  # 85 mm off the door: 34 mm past the edge
        for foot in (beyond_tip, off_front):
            _, _, touching, _, _ = h._pad_contact(foot, torch.zeros(1, 3), handle.FOOT_RADIUS_M)
            self.assertFalse(bool(touching))

    def test_the_layer_comes_down_onto_the_tilted_platform_and_turns_it_through_90_degrees(self):
        h = key()
        layer = handle.HandleTaskLayer(1, "cpu", kp=leg_cfg.FORCE_LAW_KP_N_PER_M, ki=leg_cfg.FORCE_LAW_KI_N_PER_M_S)
        one = lambda v: torch.full((1,), float(v))
        layer.start(torch.arange(1), pivot=h.pivot, u0=h.u0, reach=one(0.10),
                    clearance=one(handle.FOOT_RADIUS_M + leg_cfg.KEY_TOP_M),
                    theta_end=one(leg_cfg.KEY_THETA_END_RAD + math.radians(6.0)), descent=one(0.05),
                    descend_speed=one(0.1), turn_speed=one(1.0), max_n=one(30.0), touch_n=one(4.0), hold_s=one(0.5),
                    release_s=one(0.5), force_law=torch.tensor([False]),
                    theta_start=one(leg_cfg.KEY_THETA_START_RAD))
        pre = layer.pre_contact()
        on_pad = handle.foot_on_bar(h.pivot, h.u0, h.theta, one(0.10), one(handle.FOOT_RADIUS_M + leg_cfg.KEY_TOP_M))
        # The pre-contact point is 5 cm off the platform along its normal: up and toward the shaft.
        self.assertAlmostEqual(float(((pre - on_pad) * handle.lever_top(h.u0, h.theta)).sum()), 0.05, places=5)
        for _ in range(120):          # 0.5 s down, 1.68 rad at 1 rad/s round, then the 0.5 s hold: 2.4 s is in it
            layer.step(pre, 0.02)
        self.assertEqual(int(layer.phase), handle.HOLDING)
        self.assertAlmostEqual(float(layer.theta_ref), leg_cfg.KEY_THETA_END_RAD + math.radians(6.0), places=4)


class Goals(unittest.TestCase):
    def test_free_goals_are_admissible_or_down(self):
        torch.manual_seed(0)
        goals = foot_goals.FootGoals(512, "cpu")
        downs = 0
        for _ in range(400):
            goals.step()
            # The drawn targets (what the walk slides to), each admissible or a foot-down goal.
            ok = foot_goals.admissible(foot_goals.sphere_to_local(goals.goal)) | goals.down
            self.assertTrue(bool(ok.all()))
            downs += int(goals.down.sum())
        self.assertGreater(downs, 0)

    def test_the_handle_contact_points_are_inside_the_goal_ranges(self):
        # A contact at the eval set's extremes, pre-contact 8 cm up: radius and pitch within the trained ranges + 0.2.
        for x, y, z in ((0.55 - 0.065, -0.18, 0.28 + 0.032 + 0.08), (0.61 - 0.05, -0.08, 0.32 + 0.032 + 0.05)):
            sphere = foot_goals.local_to_sphere(torch.tensor([[x, y, z]]))
            self.assertLessEqual(float(sphere[0, 0]), leg_cfg.GOAL_RADIUS_RANGE[1] + 0.02)
            self.assertLessEqual(float(sphere[0, 1]), leg_cfg.GOAL_PITCH_RANGE[1] + 0.2)


if __name__ == "__main__":
    unittest.main()

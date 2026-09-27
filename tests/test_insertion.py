"""The insertion task's slot and task layer (`unifp_train/insertion.py`), without a simulator.

A sign error in a contact model trains a policy that pushes a key through the door or pulls it out while
told to push it in, so each contact case is pinned here: the free slide, the face, the chamfer, the walls,
a blade turned too far, a tilted key jamming, the detent, and the task layer finding a slot it has
mis-estimated.
"""
from __future__ import annotations

import math
import unittest

import torch

from unifp_train import insertion

N = torch.tensor([[1.0, 0.0, 0.0]])          # the slot's axis: into the door along +x
UP = torch.tensor([[0.0, 0.0, 1.0]])


def slot(n=1, half_u=0.006, half_w=0.0025, chamfer=0.002, depth=0.018, detent_depth=0.012, detent_n=0.0, mu=0.3):
    s = insertion.Slot(n, "cpu")
    full = lambda v: torch.full((n,), float(v))
    s.place(torch.arange(n), mouth=torch.zeros(n, 3), axis=N.expand(n, 3), roll=full(0.0), half_u=full(half_u),
            half_w=full(half_w), chamfer=full(chamfer), depth=full(depth), detent_depth=full(detent_depth),
            detent_n=full(detent_n), mu=full(mu))
    return s


def blade(roll=0.0):
    """The key's long direction for a blade rolled `roll` from level, pointing along +x."""
    u, _ = insertion.roll_frame(N, torch.tensor([roll]))
    return u


def drive(s, points, axis=N, long=None, speed=0.02, dt=0.005):
    """Step the key through `points` (each (1, 3)), moving at `speed` along +x; the last force."""
    long = blade() if long is None else long
    force = None
    for p in points:
        force = s.step(p, torch.tensor([[speed, 0.0, 0.0]]), axis, long, dt)
    return force


def along(x, y=0.0, z=0.0, start=-0.01, steps=60):
    return [torch.tensor([[start + (x - start) * i / (steps - 1), y, z]]) for i in range(steps)]


class FrameTest(unittest.TestCase):
    def test_a_level_blade_lies_across_and_an_upright_one_vertical(self):
        u, w = insertion.roll_frame(N, torch.tensor([0.0]))
        self.assertAlmostEqual(float(u[0, 2]), 0.0, places=6)
        self.assertAlmostEqual(abs(float(u[0, 1])), 1.0, places=6)
        u, _ = insertion.roll_frame(N, torch.tensor([math.pi / 2]))
        self.assertAlmostEqual(abs(float(u[0, 2])), 1.0, places=6)

    def test_a_rolled_blade_is_wider_across_the_slot(self):
        wide_u, wide_w = insertion.blade_extent(0.005, 0.0015, torch.tensor([math.radians(10.0)]))
        self.assertGreater(float(wide_w), 0.0015 + 0.0005)
        self.assertGreater(float(wide_u), 0.005 * math.cos(math.radians(10.0)) - 1e-9)


class SlotTest(unittest.TestCase):
    def test_a_centred_key_slides_in_freely_and_the_seat_stops_it(self):
        s = slot()
        force = drive(s, along(0.012))
        self.assertTrue(bool(s.entered[0]))
        self.assertLess(float(force.norm()), 1e-6)                      # nothing touches
        force = drive(s, along(0.0185, start=0.012, steps=5))
        self.assertLess(float(force[0, 0]), -1.0)                        # the seat pushes back out
        self.assertTrue(bool(s.seated()[0]))

    def test_a_key_outside_the_mouth_presses_on_the_face(self):
        s = slot()
        force = drive(s, along(0.001, y=0.004))                          # 4 mm off: beyond 1 mm + 2 mm chamfer
        self.assertFalse(bool(s.entered[0]))
        self.assertAlmostEqual(float(force[0, 0]), -insertion.CONTACT_K_N_PER_M * 0.001
                               - insertion.CONTACT_C_N_S_PER_M * 0.02, places=3)

    def test_the_chamfer_turns_a_push_into_a_centring_force(self):
        s = slot()
        force = drive(s, along(0.0015, y=0.002))                         # 2 mm off: on the chamfer, past the clearance
        self.assertTrue(bool(s.entered[0]))
        u = s.u[0]
        self.assertLess(float((force[0] * u).sum()) * float(u[1]), 0.0)   # toward the centre (-y)
        self.assertLess(float(force[0, 0]), 0.0)                         # and back out along the axis

    def test_a_blade_turned_too_far_does_not_fit(self):
        s = slot()
        drive(s, along(0.001), long=blade(math.radians(40.0)))
        self.assertFalse(bool(s.entered[0]))
        s = slot()
        drive(s, along(0.001), long=blade(math.radians(5.0)))
        self.assertTrue(bool(s.entered[0]))

    def test_a_tilted_key_jams_on_friction_where_a_straight_one_slides(self):
        # 1 mm clearance a side over 13 mm of channel admits ~9 degrees; 15 wedges the blade.
        tilted_axis = insertion.unit(torch.tensor([[1.0, math.tan(math.radians(15.0)), 0.0]]))
        straight, tilted = slot(), slot()
        drive(straight, along(0.015))
        drive(tilted, along(0.015), axis=tilted_axis)
        self.assertTrue(bool(tilted.entered[0]))
        self.assertGreater(float(tilted.normal_sum_n[0]), 10.0)          # the blade is wedged
        self.assertLess(float(straight.normal_sum_n[0]), 1e-6)
        self.assertGreater(float(tilted.axial_n[0]), 0.3 * float(tilted.normal_sum_n[0]) - 1e-6)

    def test_the_detent_holds_then_clicks(self):
        s = slot(detent_n=15.0)
        force = drive(s, along(0.0118))
        self.assertGreater(-float(force[0, 0]), 10.0)
        self.assertFalse(bool(s.clicked[0]))
        drive(s, along(0.0125, start=0.0118, steps=3))
        self.assertTrue(bool(s.clicked[0]))
        force = drive(s, along(0.0118, start=0.0125, steps=3), speed=0.0)
        self.assertLess(abs(float(force[0, 0])), 1e-6)                  # clicked: it does not hold again

    def test_the_face_is_a_panel_and_driving_through_it_is_damage_not_a_catapult(self):
        s = slot()
        force = drive(s, along(0.05, y=0.004))                           # 5 cm into the face
        self.assertTrue(bool(s.pierced[0]))
        self.assertLessEqual(-float(force[0, 0]), insertion.CONTACT_K_N_PER_M * insertion.FACE_DEPTH_M
                             + insertion.CONTACT_C_N_S_PER_M * 0.02 + 1e-6)
        far = slot()
        force = drive(far, along(0.01, y=0.3))                           # beside the door, not in front of it
        self.assertEqual(float(force.norm()), 0.0)
        self.assertFalse(bool(far.pierced[0]))

    def test_face_friction_opposes_sliding_across_it(self):
        s = slot()
        drive(s, along(0.001, y=0.004))
        force = s.step(torch.tensor([[0.001, 0.004, 0.0]]), torch.tensor([[0.0, 0.1, 0.0]]), N, blade(), 0.005)
        self.assertLess(float(force[0, 1]), 0.0)
        self.assertAlmostEqual(abs(float(force[0, 1])), 0.3 * insertion.CONTACT_K_N_PER_M * 0.001, delta=0.2)


class TaskLayerTest(unittest.TestCase):
    def run_loop(self, offset_y=0.0, offset_z=0.0, seconds=12.0, dt=0.005, admittance=0.0001):
        """A kinematic hand: across the axis it goes where the goal is; along it, it moves with the net force."""
        s = slot()
        layer = insertion.InsertionTaskLayer(1, "cpu")
        mouth_true = torch.tensor([[0.0, offset_y, offset_z]])
        s.mouth[:] = mouth_true
        layer.start(torch.arange(1), mouth=torch.zeros(1, 3), axis=N, roll=torch.zeros(1),
                    standoff=torch.tensor([0.01]), depth=torch.tensor([0.018]), speed=torch.tensor([0.02]),
                    search_n=torch.tensor([6.0]), max_n=torch.tensor([30.0]))
        tip = torch.tensor([[-0.01, 0.0, 0.0]])
        vel = torch.zeros(1, 3)
        found = None
        for i in range(int(seconds / dt)):
            goal, force_cmd = layer.step(tip, dt)
            contact = s.step(tip, vel, N, blade(), dt)
            # Across the axis the hand tracks the goal with a finite stiffness and yields to contact.
            new = tip + 0.3 * (goal - tip) + admittance * contact * dt / 0.02
            new[:, 0] = tip[:, 0] + admittance * (force_cmd[:, 0] + contact[:, 0]) * dt / 0.02
            vel = (new - tip) / dt
            tip = new
            if found is None and bool(s.entered[0]):
                found = i * dt
        return s, layer, found

    def test_a_well_estimated_slot_goes_straight_in_to_the_seat(self):
        s, layer, found = self.run_loop()
        self.assertTrue(bool(s.seated()[0]))
        self.assertLess(found, 1.0)
        self.assertLess(float(layer.theta[0]), 1e-6)                       # no search needed

    def test_a_mis_estimated_slot_is_found_by_the_spiral_and_then_seated(self):
        s, layer, found = self.run_loop(offset_y=0.004, offset_z=-0.002)
        self.assertIsNotNone(found)
        self.assertGreater(float(layer.theta[0]), 1.0)                     # it searched
        self.assertTrue(bool(s.seated()[0]))

    def test_the_push_is_capped_low_on_the_face_and_the_law_does_not_wind_up(self):
        s = slot()
        layer = insertion.InsertionTaskLayer(1, "cpu")
        layer.start(torch.arange(1), mouth=torch.zeros(1, 3), axis=N, roll=torch.zeros(1),
                    standoff=torch.tensor([0.0]), depth=torch.tensor([0.018]), speed=torch.tensor([0.05]),
                    search_n=torch.tensor([6.0]), max_n=torch.tensor([30.0]))
        tip = torch.zeros(1, 3)                                              # held on the face
        for _ in range(400):
            _, force_cmd = layer.step(tip, 0.005)
        self.assertAlmostEqual(float(force_cmd[0, 0]), 6.0, places=4)
        # A hand held short of the door is behind its reference, which keeps running...
        self.assertGreater(float(layer.ref[0] - layer.s[0]), 0.015)
        # ...but the integral alone never asks for more than the face's cap.
        self.assertLessEqual(float(layer.ki * layer.integral[0]), 6.0 + 1e-6)

    def test_once_home_it_holds_the_seat_with_a_light_push(self):
        s, layer, _ = self.run_loop()
        self.assertTrue(bool(layer.home[0]))
        goal, force_cmd = layer.step(torch.tensor([[0.012, 0.0, 0.0]]), 0.005)   # the hand lets the key drift out 6 mm
        self.assertAlmostEqual(float(force_cmd[0, 0]), layer.hold_n, places=5)
        self.assertAlmostEqual(float(goal[0, 0]), 0.018, places=5)                # still asked to be at the seat

    def test_going_in_clears_the_integral_and_brings_the_reference_back(self):
        layer = insertion.InsertionTaskLayer(1, "cpu")
        layer.start(torch.arange(1), mouth=torch.zeros(1, 3), axis=N, roll=torch.zeros(1),
                    standoff=torch.tensor([0.0]), depth=torch.tensor([0.018]), speed=torch.tensor([0.05]),
                    search_n=torch.tensor([6.0]), max_n=torch.tensor([30.0]))
        for _ in range(200):
            layer.step(torch.zeros(1, 3), 0.005)
        _, force_cmd = layer.step(torch.tensor([[0.0045, 0.0, 0.0]]), 0.005)   # the key drops in
        self.assertTrue(bool(layer.inserted[0]))
        self.assertLessEqual(float(layer.ref[0] - layer.s[0]), layer.lead_m + 1e-9)
        self.assertLessEqual(float(force_cmd[0, 0]), (layer.kp + layer.ki * 0.005) * layer.lead_m + 1e-6)



class PoseCorrectionTest(unittest.TestCase):
    """The outer loops against a stand-in policy that tracks its command with a steady bias (F-118)."""

    def loops(self, n=1, **gains):
        base = {"tip_gain": 1.5, "tip_max_m": 0.05, "elevation_gain": 1.5, "elevation_max_rad": math.radians(20.0),
                "roll_gain": 1.5, "roll_max_rad": math.radians(20.0)}
        base.update(gains)
        return insertion.PoseCorrection(n, "cpu", **base)

    def test_the_tip_loop_puts_a_biased_hands_key_on_its_goal(self):
        c = self.loops()
        goal = torch.tensor([[0.5, 0.02, 0.8]])
        approach = insertion.unit(torch.tensor([[1.0, 0.1, -0.12]]))   # the hand ~9 degrees off, as the policy arrives
        bias = torch.tensor([[0.012, -0.008, 0.015]])                    # and centimetres off where it is sent
        mask = torch.tensor([True])
        for _ in range(250):                                             # 5 s at the policy's 50 Hz
            jaw = c.jaw_for_tip(goal, approach, insertion.KEY_LENGTH_M)
            tip = jaw + bias + insertion.KEY_LENGTH_M * approach
            c.update_tip(mask, goal, tip, 0.02)
        self.assertLess(float((tip - goal).norm()), 1e-4)

    def test_across_the_axis_it_leaves_the_depth_alone(self):
        c = self.loops()
        c.update_tip(torch.tensor([True]), torch.tensor([[0.01, 0.003, 0.0]]), torch.zeros(1, 3), 0.02, across=N)
        self.assertEqual(float(c.offset[0, 0]), 0.0)
        self.assertGreater(float(c.offset[0, 1]), 0.0)

    def test_nothing_integrates_outside_its_mask_and_the_offset_is_clamped(self):
        c = self.loops(n=2, tip_max_m=0.01)
        for _ in range(500):
            c.update_tip(torch.tensor([True, False]), torch.tensor([[1.0, 0.0, 0.0]] * 2), torch.zeros(2, 3), 0.02)
        self.assertAlmostEqual(float(c.offset[0].norm()), 0.01, places=6)
        self.assertEqual(float(c.offset[1].norm()), 0.0)

    def test_the_attitude_loops_cancel_a_steady_elevation_and_roll_bias(self):
        c = self.loops()
        want_elevation, want_roll = torch.tensor([0.05]), torch.tensor([math.pi / 2 - 0.02])
        mask = torch.tensor([True])
        for _ in range(250):
            elevation = want_elevation + c.elevation + math.radians(7.0)     # the hand 7 degrees high of its command
            roll = insertion.wrap_half_turn(want_roll + c.roll - math.radians(5.0))
            c.update_attitude(mask, want_elevation, elevation, want_roll, roll, 0.02)
        self.assertLess(abs(float(want_elevation + c.elevation + math.radians(7.0) - want_elevation)), 1e-3)
        # Across the half-turn wrap (the wanted roll is near +90 degrees, the blade's near -90 is the same axis).
        self.assertAlmostEqual(float(c.roll), math.radians(5.0), places=3)


class CompliantGripTest(unittest.TestCase):
    def test_a_force_at_the_tip_tilts_the_key_toward_it_against_the_spring(self):
        grip = insertion.CompliantGrip(1, "cpu")
        force = torch.tensor([[1.0, -1.5, 0.0]])       # 1 N along the hand's x, 1.5 N along -y, at the tip
        for _ in range(2000):
            grip.step(force, insertion.KEY_LENGTH_M, 0.005)
        axis, long = grip.axes_local()
        self.assertGreater(float(axis[0, 0]), 0.0)     # toward +x
        self.assertLess(float(axis[0, 1]), 0.0)        # and toward -y
        # At rest the spring carries the moment: about x, L * 1.5 N / k; about y, L * 1 N / k (small angles).
        self.assertAlmostEqual(float(grip.tilt[0, 0]), insertion.KEY_LENGTH_M * 1.5 / grip.stiffness, delta=2e-3)
        self.assertAlmostEqual(float(grip.tilt[0, 1]), insertion.KEY_LENGTH_M * 1.0 / grip.stiffness, delta=2e-3)
        self.assertAlmostEqual(float((axis * long).sum()), 0.0, places=6)

    def test_it_tilts_no_further_than_its_limit(self):
        grip = insertion.CompliantGrip(1, "cpu")
        for _ in range(2000):
            grip.step(torch.tensor([[0.0, -100.0, 0.0]]), insertion.KEY_LENGTH_M, 0.005)
        self.assertAlmostEqual(float(grip.tilt[0, 0]), insertion.GRIP_MAX_RAD, places=6)

    def drive(self, compliant: bool, tilt_deg: float = 0.0, jitter_m: float = 0.0):
        """A hand `tilt_deg` off the slot's axis (across the blade's narrow side) and jittering `jitter_m` across it at
        10 Hz -- as the policies this task starts from do while holding still (2026-09-27) -- driven in at 20 mm/s with
        the key's tip starting on the axis. The peak contact and the final depth."""
        s = slot(half_u=0.006, half_w=0.0025)          # 1 mm clearance each side
        t = math.radians(tilt_deg)
        z_h = torch.tensor([[math.cos(t), 0.0, math.sin(t)]])
        y_h = torch.tensor([[0.0, 1.0, 0.0]])
        x_h = torch.cross(y_h, z_h, dim=-1)
        R = torch.stack((x_h[0], y_h[0], z_h[0]), dim=-1)    # world <- hand, columns the hand's axes
        grip = insertion.CompliantGrip(1, "cpu") if compliant else None
        start = torch.tensor([[-0.01, 0.0, 0.0]]) - insertion.KEY_LENGTH_M * z_h
        v, dt, peak = 0.02, 0.005, 0.0
        for i in range(int(0.03 / (v * dt))):
            time = i * dt
            wobble = jitter_m * math.sin(2 * math.pi * 10.0 * time)
            wobble_v = jitter_m * 2 * math.pi * 10.0 * math.cos(2 * math.pi * 10.0 * time)
            jaw = start + torch.tensor([[v * time, 0.0, wobble]])
            vel = torch.tensor([[v, 0.0, wobble_v]])
            if grip:
                def probe(axis_l, long_l):
                    axis, long = axis_l @ R.T, long_l @ R.T
                    force, normal = s.probe(jaw + insertion.KEY_LENGTH_M * axis, vel, axis, long)
                    return force @ R, normal > 0.0
                grip.settle(probe, insertion.KEY_LENGTH_M, dt)
            axis_l, long_l = grip.axes_local() if grip else (torch.tensor([[0.0, 0.0, 1.0]]), torch.tensor([[0.0, 1.0, 0.0]]))
            axis, long = axis_l @ R.T, long_l @ R.T
            s.step(jaw + insertion.KEY_LENGTH_M * axis, vel, axis, long, dt)
            peak = max(peak, float(s.normal_sum_n[0]))
            if bool(s.seated()[0]):
                break
        return peak, float(s.tip_depth[0])

    def test_a_jittering_hand_bangs_a_rigid_key_into_the_walls_and_not_a_compliant_one(self):
        rigid_peak, _ = self.drive(compliant=False, jitter_m=0.005)
        soft_peak, soft_depth = self.drive(compliant=True, jitter_m=0.005)
        self.assertGreater(rigid_peak, 150.0)          # past the task's damage threshold (measured 160 N)
        self.assertLess(soft_peak, 30.0)               # measured 22 N
        self.assertGreater(soft_depth, 0.017)

    def test_a_compliant_key_goes_home_off_axis_with_less_force(self):
        rigid_peak, rigid_depth = self.drive(compliant=False, tilt_deg=6.0)
        soft_peak, soft_depth = self.drive(compliant=True, tilt_deg=6.0)
        self.assertGreater(soft_depth, 0.017)
        self.assertLess(soft_peak, 0.5 * rigid_peak)



class RewardTest(unittest.TestCase):
    def test_once_started_the_stillness_term_prices_only_motion_across_the_axis(self):
        from types import SimpleNamespace

        from unifp_train import insert_rewards
        v = torch.tensor([[0.3, 0.1, 0.0], [0.3, 0.1, 0.0]])             # 0.3 m/s along the axis (+x), 0.1 across
        state = SimpleNamespace(tool_vel_w=v, fixture_axis_w=N.expand(2, 3), fixture_engaged=torch.tensor([1.0, 0.0]),
                                ins_holding=torch.tensor([1.0, 1.0]))
        cost = insert_rewards.ins_tool_speed(state)
        self.assertAlmostEqual(float(cost[0]), 0.01, places=6)          # inserting: only the 0.1 across
        self.assertAlmostEqual(float(cost[1]), 0.10, places=6)          # holding the pre-insertion pose: all of it

    def test_with_true_targets_the_roll_is_scored_against_the_slot_not_the_shifted_command(self):
        from types import SimpleNamespace

        from unifp_isaaclab import interface
        from unifp_train import rewards
        commands = torch.zeros(2, interface.NUM_COMMANDS)
        # The loop has shifted the command to 30 degrees; the blade is level (0), which is what the slot wants.
        commands[:, interface.CMD_EE_ORN_R:interface.CMD_EE_ORN_P + 1] = rewards.encode_roll(torch.full((2,), math.radians(30.0)))
        state = SimpleNamespace(ee_approach_w=N.expand(2, 3), ee_jaw_w=torch.tensor([[0.0, 1.0, 0.0]] * 2),
                                base_forward_w=N.expand(2, 3), commands=commands,
                                roll_true=torch.zeros(2), roll_true_mask=torch.tensor([True, False]))
        score = rewards.tracking_ee_orn_roll(state)
        self.assertAlmostEqual(float(score[0]), 1.0, places=5)            # on the slot's roll
        self.assertLess(float(score[1]), 0.9)                             # scored against the command, 30 degrees off


if __name__ == "__main__":
    unittest.main()

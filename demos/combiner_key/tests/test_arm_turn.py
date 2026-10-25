"""The key turned by the arm, without a simulator: the hand's frame on the platform, the contact sphere, the push's
targets, the stance, and the arm IK and its quasi-static push over the whole turn from that stance."""
from __future__ import annotations

import math
import unittest

import numpy as np

from demos.combiner_key import arm_turn as at
from demos.combiner_key import geometry as kg
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6
from unifp_train import leg_cfg


class Hand(unittest.TestCase):
    def test_the_hand_frame_is_a_rotation_at_every_angle(self):
        for theta in (-45.0, -10.0, 30.0, 45.0):
            r = at.hand_rotation(theta)
            self.assertTrue(np.allclose(r.T @ r, np.eye(3), atol=1e-9))
            self.assertAlmostEqual(float(np.linalg.det(r)), 1.0, places=9)

    def test_it_comes_in_pitched_down_onto_the_platform_with_the_fingers_along_the_key(self):
        for theta in (-45.0, 0.0, 45.0):
            r = at.hand_rotation(theta)
            top, n = np.asarray(kg.lever_top_box(theta)), np.asarray(kg.OUT_OF_DOOR_BOX)
            pitch = math.radians(at.HAND_PITCH_DEG)
            self.assertAlmostEqual(float(r[:, 2] @ -n), math.cos(pitch), places=9)      # into the door
            self.assertAlmostEqual(float(r[:, 2] @ -top), math.sin(pitch), places=9)    # down onto the platform
            self.assertAlmostEqual(abs(float(r[:, 1] @ np.asarray(kg.lever_dir_box(theta)))), 1.0, places=9)

    def test_the_turn_is_the_hand_rotated_about_the_shaft(self):
        a, b = at.hand_rotation(-45.0), at.hand_rotation(45.0)
        rel = b @ a.T
        axis = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]])
        self.assertAlmostEqual(float(np.arccos((np.trace(rel) - 1) / 2)), math.radians(90.0), places=9)
        self.assertGreater(abs(float(axis / np.linalg.norm(axis) @ np.asarray(kg.OUT_OF_DOOR_BOX))), 0.999)

    def test_the_contact_sphere_touches_the_platform_at_the_tip_edge(self):
        for theta in (-45.0, 45.0):
            r = at.hand_rotation(theta)
            top = np.asarray(kg.lever_top_box(theta))
            lowest = r @ at.contact_link6() - at.CONTACT_RADIUS_M * top
            np.testing.assert_allclose(lowest, r @ np.asarray(at.TIP_EDGE_LINK6), atol=1e-12)


class Targets(unittest.TestCase):
    def test_the_edge_presses_into_rests_on_and_clears_the_platform(self):
        for theta in (-45.0, 0.0, 45.0):
            top = np.asarray(kg.lever_top_box(theta))
            on = at.platform_point(theta)
            depth = lambda where: float((on - at.tip_target(where, theta)) @ top)
            self.assertAlmostEqual(depth("pressed"), at.PRESS_DEPTH_M, places=12)
            self.assertAlmostEqual(depth("resting"), 0.0, places=12)
            self.assertAlmostEqual(depth("above"), -at.CLEARANCE_M, places=12)

    def test_the_push_is_on_the_platform(self):
        """The edge's point is on the platform's top, inside its extent (`leg_cfg.KEY_PAD_BOX` and the bar under it)."""
        u_lo, u_hi = leg_cfg.KEY_PAD_BOX[0]
        n_lo, n_hi = leg_cfg.KEY_BAR_BOX[1][0], leg_cfg.KEY_PAD_BOX[1][1]
        self.assertTrue(u_lo < at.PUSH_REACH_M < u_hi)
        self.assertTrue(n_lo < at.CONTACT_OFF_DOOR_M < n_hi)

    def test_the_jaw_centre_is_where_link6_puts_it(self):
        r = at.hand_rotation(10.0)
        tip = at.tip_target("pressed", 10.0)
        link6 = tip - r @ np.asarray(at.TIP_EDGE_LINK6)
        np.testing.assert_allclose(at.jaw_centre(tip, r), link6 + r @ np.asarray(JAW_CENTRE_LINK6), atol=1e-12)

    def test_the_turn_runs_from_the_start_stop_to_the_end_stop(self):
        """The arm's lever (v4 in the changed lock, 2026-10-24): in level, round 45 degrees clockwise, unlocked at 30."""
        angles = [at.turn_angle(u) for u in np.linspace(0, 1, 21)]
        self.assertEqual(angles[0], kg.ARM_LEVER.insert_deg)
        self.assertEqual(angles[-1], kg.ARM_LEVER.end_deg)
        self.assertEqual((at.START_DEG, at.END_DEG, at.UNLOCK_DEG), (0.0, 45.0, 30.0))
        self.assertTrue(all(b >= a for a, b in zip(angles, angles[1:])))

    def test_the_key_shape_is_the_plants(self):
        """v3 is the foot's plant's own (`leg_cfg`); the arm's (v4) is v3 10 mm longer between the shaft and the platform."""
        v3, v4 = kg.LEVER_V3, kg.LEVER_V4
        self.assertEqual(v3.top_m, leg_cfg.KEY_TOP_M)
        self.assertEqual(v3.bar_box, leg_cfg.KEY_BAR_BOX)
        self.assertEqual(v3.pad_box, leg_cfg.KEY_PAD_BOX)
        self.assertEqual((at.KEY_BAR_BOX, at.KEY_PAD_BOX), (kg.ARM_LEVER.bar_box, kg.ARM_LEVER.pad_box))
        self.assertAlmostEqual(v4.bar_box[0][1] - v3.bar_box[0][1], 0.010, places=9)
        for i in (0, 1):
            self.assertAlmostEqual(v4.pad_box[0][i] - v3.pad_box[0][i], 0.010, places=9)
        self.assertTrue(v4.pad_box[0][0] < at.PUSH_REACH_M < v4.pad_box[0][1])

    def test_lukas_lock_as_a_torque(self):
        self.assertAlmostEqual(at.lock_torque_nm(5.0), 0.65, places=9)


class Stance(unittest.TestCase):
    def test_the_lock_is_ahead_and_to_the_left_for_any_box_yaw(self):
        for yaw_deg in (-6.0, 0.0, 4.0):
            yaw = math.radians(180.0 + yaw_deg)
            rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            lock = np.array([0.7, 0.25, kg.LOCK[2]])
            normal = np.append(rot @ np.array([1.0, 0.0]), 0.0)
            right = np.append(rot @ np.array([0.0, 1.0]), 0.0)
            base = at.stance_base(lock, normal, right)
            facing = -normal[:2]
            left = np.array([-facing[1], facing[0]])
            rel = lock[:2] - base[:2]
            self.assertAlmostEqual(float(rel @ facing), at.ARM_DOOR_AHEAD_M, places=9)
            self.assertAlmostEqual(float(rel @ left), at.LOCK_LEFT_OF_BASE_M, places=9)


class Reach(unittest.TestCase):
    def test_the_arm_pushes_the_whole_turn_from_the_stance(self):
        """IK over -45..45 at the stance, the trunk clear, and the quasi-static push at least 1.3 N·m (the CPU scan's
        worst over the stance's spread, 2026-10-21) -- twice Lukas's 5 N lock."""
        for height in (0.28, 0.31, 0.33):
            rows = at.arc_rows(base_height_m=height, step_deg=15.0)
            self.assertTrue(all(r.solved for r in rows), height)
            self.assertTrue(all(r.trunk_clear for r in rows), height)
            self.assertGreater(min(r.capacity_nm for r in rows), 1.3, height)
            self.assertGreater(min(r.capacity_nm for r in rows), 2 * at.lock_torque_nm(5.0), height)

    def test_every_corner_of_the_walks_window_pushes_the_whole_turn(self):
        """The walk stops in `ACCEPT_*` (the real door pull's way) and may drift to the edges of `PUSH_*`; each corner of
        those must still do the push, and the walk's window must lie inside them."""
        self.assertTrue(at.PUSH_DOOR_AHEAD_M[0] <= at.ACCEPT_DOOR_AHEAD_M[0] < at.ACCEPT_DOOR_AHEAD_M[1] <= at.PUSH_DOOR_AHEAD_M[1])
        self.assertTrue(at.PUSH_LOCK_LEFT_M[0] <= at.ACCEPT_LOCK_LEFT_M[0] < at.ACCEPT_LOCK_LEFT_M[1] <= at.PUSH_LOCK_LEFT_M[1])
        for door in at.PUSH_DOOR_AHEAD_M:
            for left in at.PUSH_LOCK_LEFT_M:
                for height in (0.28, 0.33):
                    rows = at.arc_rows(door, left, height, step_deg=15.0)
                    self.assertTrue(all(r.solved and r.trunk_clear for r in rows), (door, left, height))
                    self.assertGreater(min(r.capacity_nm for r in rows), 1.3, (door, left, height))

    def test_the_window_holds_the_stance_and_rejects_a_turned_robot(self):
        self.assertTrue(at.in_window(at.ARM_DOOR_AHEAD_M, at.LOCK_LEFT_OF_BASE_M, 0.0))
        self.assertFalse(at.in_window(at.ARM_DOOR_AHEAD_M, at.LOCK_LEFT_OF_BASE_M, 5.0))
        self.assertFalse(at.in_window(0.40, at.LOCK_LEFT_OF_BASE_M, 0.0))


if __name__ == "__main__":
    unittest.main()

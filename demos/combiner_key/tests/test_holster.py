"""Lukas's key holster, without a simulator: the saved registration (where the holster and the key in it are on the
D1's base), the colliders against the key, and the draw on the CPU arm model (`holster.check`)."""
from __future__ import annotations

import math
import unittest

import numpy as np

from demos.combiner_key import geometry as kg
from demos.combiner_key import holster
from demos.unifp.decoupled_arm import hand_rotation


class Registration(unittest.TestCase):
    def setUp(self):
        self.record = holster.load()
        self.key = holster.key_in_d1_base(self.record)

    def test_the_key_frame_is_a_rotation(self):
        rot = self.key[:3, :3]
        self.assertTrue(np.allclose(rot.T @ rot, np.eye(3), atol=1e-6))
        self.assertAlmostEqual(float(np.linalg.det(rot)), 1.0, places=5)

    def test_the_key_is_the_mirrored_one_the_simulation_uses(self):
        self.assertTrue(self.record["key_mirrored"])
        self.assertTrue(kg.KEY_MIRRORED)
        # The mirrored key's (u, n, t) is left-handed, as `geometry.KEY_AXES_LINK6` has it.
        axes = holster.key_axes(self.key)
        rows = np.asarray(kg.KEY_AXES_LINK6)
        self.assertAlmostEqual(float(np.cross(axes["u"], axes["n"]) @ axes["t"]),
                               float(np.cross(rows[0], rows[1]) @ rows[2]), places=5)
        self.assertLess(self.record["bar_fit_mm"]["median"], 0.1)

    def test_the_holster_is_on_the_right_beside_the_base_and_the_key_slides_out_forward_and_up(self):
        axes = holster.key_axes(self.key)
        self.assertLess(axes["origin"][1], -0.06)                      # the robot's right of the arm's axis
        out = axes["out"]
        self.assertGreater(out[0], 0.7)
        self.assertAlmostEqual(math.degrees(math.asin(float(out[2]))), 40.8, delta=0.5)


class Colliders(unittest.TestCase):
    def test_the_holster_s_boxes_stand_off_the_key_s_by_less_than_half_a_millimetre_and_never_overlap_it(self):
        key = holster.boxes_in(np.eye(4), holster.KEY_BOXES)
        gaps = []
        for name, (lo, hi) in holster.HOLSTER_BOXES.items():
            grid = np.stack(np.meshgrid(*[np.linspace(a, b, 9) for a, b in zip(lo, hi)], indexing="ij"), -1)
            gap = float(holster.box_distance(grid.reshape(-1, 3), key).min())
            self.assertGreaterEqual(gap, 0.0, name)
            gaps.append(gap)
        self.assertLess(min(gaps), 0.0005)

    def test_the_key_can_only_slide_out_toward_its_shaft(self):
        key = np.eye(4)
        holster_boxes = holster.boxes_in(np.eye(4), holster.HOLSTER_BOXES)
        bar = holster.KEY_BOXES["bar"]
        for direction, should_hit in ((np.array(holster.SLIDE_OUT_KEY), False), (-np.array(holster.SLIDE_OUT_KEY), True),
                                      (np.array([1.0, 0, 0]), True), (np.array([-1.0, 0, 0]), True),
                                      (np.array([0, 0, 1.0]), True), (np.array([0, 0, -1.0]), True)):
            moved = key.copy()
            moved[:3, 3] = 0.002 * direction
            grid = np.stack(np.meshgrid(*[np.linspace(a, b, 15) for a, b in zip(*bar)], indexing="ij"), -1).reshape(-1, 3)
            inside = holster.box_distance(grid + moved[:3, 3], holster_boxes) < 0.0
            self.assertEqual(bool(inside.any()), should_hit, direction)


class Draw(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out = holster.check()

    def test_the_grip_is_what_the_control_loop_builds(self):
        axes = holster.key_axes(holster.key_in_d1_base())
        self.assertTrue(np.allclose(holster.grip_rotation(axes), hand_rotation(-axes["n"], axes["u"], 0.0,
                                                                                 holster.GRIP_ROLL), atol=1e-9))

    def test_the_draw_solves_within_the_loop_s_limits_from_the_root_of_the_bar(self):
        self.assertTrue(self.out["solved"], self.out["rows"][-1])
        rows = self.out["rows"]
        self.assertEqual(rows[-1]["move"], "out")
        self.assertAlmostEqual(rows[-1]["f"], 1.0)
        self.assertLessEqual(max(r["range_use_max"] for r in rows), holster.IK_SOFT + 1e-6)
        self.assertLess(max(r["gravity_use_max"] for r in rows), 0.5)

    def test_nothing_touches_on_the_way(self):
        rows = self.out["rows"]
        self.assertGreater(min(r["trunk_proxy_margin_m"] for r in rows), 0.0)
        self.assertGreater(min(r["open_hand_to_key_mm"] for r in rows if r["open_hand_to_key_mm"] is not None), 5.0)
        self.assertGreater(min(r["arm_to_holster_mm"] for r in rows), 5.0)
        self.assertGreater(min(r["key_to_arm_base_mm"] for r in rows), 5.0)
        self.assertGreater(min(r["key_to_arm_links_mm"] for r in rows), 5.0)
        self.assertGreater(self.out["unfold_clearance_mm"], 5.0)
        self.assertGreater(self.out["unfold_trunk_proxy_margin_m"], 0.0)

    def test_the_insertion_s_own_roll_does_not_draw_it(self):
        self.assertFalse(holster.check(roll=-holster.GRIP_ROLL)["solved"])


if __name__ == "__main__":
    unittest.main()

"""The held mechanism's viewer overlay (`unifp_train/mech_vis.py`): where each part is drawn, from the plant alone."""
import math
import unittest

import torch

from unifp_train import mech_vis
from unifp_train.mechanism import Mechanism

V = mech_vis


def _mech(hinge: bool, opening, axis=(0.0, 0.0, 1.0), radius=0.25, travel=0.12):
    """Two environments: one grasped at (0.5, 0, 0.4) in front of a robot at the origin, one never grasped."""
    m = Mechanism(2, "cpu", grasp_damping=10.0)
    one = lambda v: torch.tensor([v])
    m.grasp(torch.tensor([0]), torch.tensor([[0.5, 0.0, 0.4]]), hinge=torch.tensor([hinge]),
            opening=torch.tensor([opening]), axis=torch.tensor([axis]), radius=one(radius), travel=one(travel),
            mass=one(1.0), spring_n=one(0.0), spring_k=one(0.0), damping=one(0.0), friction_static=one(0.0),
            friction_kinetic=one(0.0), latch_n=one(0.0), grasp_k=one(2000.0), grip_n=one(150.0))
    return m


BASE = torch.zeros(2, 3)
SHOWN = torch.tensor([True, False])


class DrawerTest(unittest.TestCase):
    """A drawer pulled toward the robot (−x)."""

    def setUp(self):
        self.m = _mech(False, (-1.0, 0.0, 0.0))

    def test_closed_front_behind_the_handle_and_cabinet_behind_that(self):
        t, _, s, idx = V.marker_poses(self.m, BASE, torch.zeros(2), SHOWN)
        handle_x = 0.5
        self.assertAlmostEqual(float(t[0, V.HANDLE_SLOT, 0]), handle_x, places=5)
        self.assertGreater(float(t[0, V.FRONT_SLOT, 0]), handle_x)           # the front is on the far side
        self.assertGreater(float(t[0, V.FIXED_SLOT, 0]), float(t[0, V.FRONT_SLOT, 0]))
        self.assertEqual(int(idx[0, V.FIXED_SLOT]), V.FIXED)
        self.assertEqual(int(idx[0, V.FRONT_SLOT]), V.BODY + V.ORANGE)
        # Closed: no sides yet; the never-grasped environment draws nothing.
        self.assertTrue(bool((s[0, V.SIDES_SLOT] <= V.HIDDEN).all()))
        self.assertTrue(bool((s[1] <= V.HIDDEN).all()))

    def test_open_drawer_moves_the_front_and_shows_its_sides(self):
        self.m.s[0] = 0.12
        t, _, s, idx = V.marker_poses(self.m, BASE, torch.full((2,), 0.12), SHOWN)
        self.assertAlmostEqual(float(t[0, V.HANDLE_SLOT, 0]), 0.38, places=5)
        self.assertAlmostEqual(float(s[0, V.SIDES_SLOT, 0]), 0.12, places=5)     # the sides as long as the travel
        # The sides span from the cabinet's face to the front's back face.
        face = 0.5 + V.PROUD_M + V.PLATE_M
        self.assertAlmostEqual(float(t[0, V.SIDES_SLOT, 0]), face - 0.06, places=5)
        self.assertEqual(int(idx[0, V.FRONT_SLOT]), V.BODY + V.GREEN)            # >= 0.9 of the travel: opened
        self.assertAlmostEqual(float(t[0, V.GOAL_SLOT, 0]), 0.38, places=5)

    def test_over_budget_and_torn_are_red(self):
        self.m.drive_n[0] = 81.0
        self.assertEqual(int(V.state_colour(self.m)[0]), V.RED)
        self.m.drive_n[0] = 0.0
        self.m.torn[0] = True
        self.assertEqual(int(V.state_colour(self.m)[0]), V.RED)

    def test_force_arrow_from_the_handle_along_the_force(self):
        self.m.force_on_tool[0] = torch.tensor([10.0, 0.0, 0.0])
        t, _, s, _ = V.marker_poses(self.m, BASE, torch.zeros(2), SHOWN)
        self.assertAlmostEqual(float(s[0, V.FORCE_SLOT, 0]), 10.0 * V.FORCE_M_PER_N, places=6)
        self.assertAlmostEqual(float(t[0, V.FORCE_SLOT, 0]), 0.5 + 5.0 * V.FORCE_M_PER_N, places=6)
        self.m.force_on_tool[0] = torch.tensor([0.5, 0.0, 0.0])
        _, _, s, _ = V.marker_poses(self.m, BASE, torch.zeros(2), SHOWN)
        self.assertTrue(bool((s[0, V.FORCE_SLOT] <= V.HIDDEN).all()))


class ButtonTest(unittest.TestCase):
    def test_housing_beyond_the_travel_and_no_bar(self):
        m = _mech(False, (1.0, 0.0, 0.0), travel=0.015)
        t, _, s, _ = V.marker_poses(m, BASE, torch.zeros(2), SHOWN)
        self.assertGreater(float(t[0, V.FIXED_SLOT, 0]), 0.5 + 0.015)
        self.assertTrue(bool((s[0, V.HANDLE_SLOT] <= V.HIDDEN).all()))
        self.assertTrue(bool((s[0, V.SIDES_SLOT] <= V.HIDDEN).all()))


class DoorTest(unittest.TestCase):
    """A door with a vertical hinge, pulled toward the robot."""

    def setUp(self):
        self.m = _mech(True, (-1.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0), radius=0.25, travel=0.25 * math.radians(40))

    def test_leaf_runs_from_the_axis_past_the_handle_and_swings(self):
        for frac in (0.0, 1.0):
            self.m.s[0] = frac * self.m.travel[0]
            t, q, s, idx = V.marker_poses(self.m, BASE, torch.zeros(2), SHOWN)
            p = self.m.position()[0]
            centre = self.m.centre[0]
            leaf = t[0, V.FRONT_SLOT]
            along = (p - centre) / 0.25
            # The leaf's middle is half its length out from the axis along the handle's radius, set back off the handle.
            expect = centre + along * 0.5 * (0.25 + V.LEAF_PAST_M)
            offset = leaf - expect
            self.assertAlmostEqual(float((offset * along).sum()), 0.0, places=5)
            self.assertAlmostEqual(float(offset.norm()), V.PROUD_M + 0.5 * V.PLATE_M, places=5)
            # The leaf is set back on the far side from the robot.
            n = torch.linalg.cross(along, torch.tensor([0.0, 0.0, 1.0]))
            toward = n * torch.sign((n * (BASE[0] - p)).sum())
            self.assertLess(float(((leaf - p) * toward).sum()), -0.01)
            self.assertAlmostEqual(float(s[0, V.FRONT_SLOT, 0]), 0.25 + V.LEAF_PAST_M, places=5)
            self.assertAlmostEqual(float(s[0, V.PIN_SLOT, 0]), V.LEAF_HEIGHT_M + 0.04, places=5)
        # Open 40 degrees: the handle has swung toward the robot.
        self.assertLess(float(self.m.position()[0, 0]), 0.5)

    def test_doorway_stays_where_the_leaf_was_closed(self):
        t0, *_ = V.marker_poses(self.m, BASE, torch.zeros(2), SHOWN)
        self.m.s[0] = self.m.travel[0]
        t1, *_ = V.marker_poses(self.m, BASE, torch.zeros(2), SHOWN)
        self.assertTrue(torch.allclose(t0[0, V.FIXED_SLOT], t1[0, V.FIXED_SLOT]))
        self.assertFalse(torch.allclose(t0[0, V.FRONT_SLOT], t1[0, V.FRONT_SLOT]))

    def test_path_follows_the_arc(self):
        t, *_ = V.marker_poses(self.m, BASE, torch.zeros(2), SHOWN)
        mids = t[0, V.PATH_SLOT:V.PATH_SLOT + V.PATH_SEGMENTS]
        radii = (mids - self.m.centre[0]).norm(dim=-1)
        # Chord midpoints sit just inside the radius.
        self.assertTrue(bool(((radii > 0.249) & (radii <= 0.25 + 1e-6)).all()))

    def test_frames_are_finite(self):
        t, q, s, _ = V.marker_poses(self.m, BASE, torch.zeros(2), SHOWN)
        for value in (t, q, s):
            self.assertTrue(bool(torch.isfinite(value).all()))
        self.assertTrue(torch.allclose(q.norm(dim=-1), torch.ones(2, V.MARKERS_PER_ENV), atol=1e-4))


if __name__ == "__main__":
    unittest.main()

"""CPU tests for the decoupled door-pull baseline's arm (`decoupled_arm.py`). No simulator.

    python -m unittest demos.unifp.tests.test_decoupled -v
"""
import math
import random
import unittest

import numpy as np

from demos.combiner.pull import PROVEN_GRASPS, Handle, PullParams
from demos.unifp import decoupled_arm, props, script
from position_only.workspace import load_urdf

#: About where the standing robot's base sits above the ground in the door pull, metres. The script's points are
#: from the ground; the arm's IK works in the base's frame.
BASE_HEIGHT_M = 0.30


def box_pose(site):
    yaw = math.radians(site.yaw_deg)
    pose = np.eye(4)
    pose[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    pose[:3, 3] = site.root_m
    return pose


class HandRotation(unittest.TestCase):
    def test_matches_the_scripted_grips_handle_at_every_door_angle(self):
        site = props.BoxSite(distance_m=0.72, bearing_deg=3.0, yaw_deg=184.0)
        for door in (0.0, 30.0, 90.0):
            for pitch, roll in ((0.0, 1), (40.0, -1), (50.0, -1), (20.0, 1)):
                want = Handle(box_pose(site), PullParams(pitch_deg=pitch, roll=roll)).grasp(0.0, door)[1]
                got = decoupled_arm.hand_rotation(site.approach_unit(door), site.lever_axis_unit(0.0, door),
                                                  pitch, roll)
                np.testing.assert_allclose(got, want, atol=1e-9, err_msg=f"door {door}, pitch {pitch}, roll {roll}")

    def test_is_a_rotation(self):
        rot = decoupled_arm.hand_rotation((-1.0, 0.1, -0.2), (0.0, 1.0, 0.0), 40.0, -1)
        np.testing.assert_allclose(rot.T @ rot, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(rot)), 1.0, places=12)

    def test_pitch_tips_the_approach_down(self):
        rot = decoupled_arm.hand_rotation((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), 40.0, 1)
        self.assertAlmostEqual(rot[2, 2], -math.sin(math.radians(40.0)), places=12)


class GraspChoice(unittest.TestCase):
    def test_tries_every_candidate_once_nearest_level_first(self):
        order = decoupled_arm.GRASP_ORDER
        self.assertEqual(order[0][0], 0.0)
        self.assertEqual([p for p, _ in order], sorted(p for p, _ in order))
        self.assertEqual(len(order), len(set(order)))
        self.assertTrue(set(PROVEN_GRASPS) <= set(order))

    def test_the_door_pulls_placements_have_a_grasp(self):
        """Every placement the door pull draws admits one of the scripted grip's grasps, at grasp and standoff.

        Not guaranteed by anything else: the scripted grip was planned with the dog lying, at another height."""
        joints, links = load_urdf()
        rng = random.Random(1)
        timing = script.DoorTiming()
        chosen = []
        for _ in range(6):
            site = props.sample_door_site(rng)
            arm = decoupled_arm.ArmIK(joints, links)
            point = np.asarray(site.grasp_point_m(0.0, 0.0)) - [0.0, 0.0, BASE_HEIGHT_M]
            grasp = arm.choose_grasp(point, site.approach_unit(0.0), site.lever_axis_unit(0.0, 0.0), timing.standoff_m)
            self.assertNotIn("none", arm.grasp_note, msg=site)
            chosen.append(grasp)
        self.assertTrue(chosen)


class Solve(unittest.TestCase):
    def test_warm_start_reaches_the_grasp_pose(self):
        joints, links = load_urdf()
        site = props.BoxSite(distance_m=0.72, bearing_deg=0.0, yaw_deg=180.0)
        arm = decoupled_arm.ArmIK(joints, links)
        point = np.asarray(site.grasp_point_m(0.0, 0.0)) - [0.0, 0.0, BASE_HEIGHT_M]
        pitch, roll = arm.choose_grasp(point, site.approach_unit(0.0), site.lever_axis_unit(0.0, 0.0), 0.12)
        rot = decoupled_arm.hand_rotation(site.approach_unit(0.0), site.lever_axis_unit(0.0, 0.0), pitch, roll)
        step = arm.solve(point, rot)
        self.assertEqual(step.status, "pose")
        self.assertLess(step.position_error_m, 1e-3)
        # The next, nearby target warm-starts from it and converges without reseeding.
        nearer = point + [0.0, 0.005, 0.0]
        self.assertEqual(arm.solve(nearer, rot).status, "pose")

    def test_a_target_out_of_reach_costs_a_bounded_solve(self):
        """The loop's worst case: the pose and then the position, each capped, and no seeded search."""
        import time
        joints, links = load_urdf()
        site = props.BoxSite(distance_m=0.72, bearing_deg=0.0, yaw_deg=180.0)
        arm = decoupled_arm.ArmIK(joints, links)
        point = np.asarray(site.grasp_point_m(0.0, 0.0)) - [0.0, 0.0, BASE_HEIGHT_M]
        grasp = arm.choose_grasp(point, site.approach_unit(0.0), site.lever_axis_unit(0.0, 0.0), 0.12)
        rot = decoupled_arm.hand_rotation(site.approach_unit(0.0), site.lever_axis_unit(0.0, 0.0), *grasp)
        start = time.perf_counter()
        step = arm.solve(point + [0.25, 0.0, 0.0], rot)
        elapsed = time.perf_counter() - start
        self.assertIn(step.status, ("position", "hold"))
        # Two capped solves; generous against a slower machine, and an order below the 486 ms the seeded search cost.
        self.assertLess(elapsed, 0.05)

    def test_an_unreachable_point_holds_rather_than_jumps(self):
        joints, links = load_urdf()
        arm = decoupled_arm.ArmIK(joints, links)
        before = arm.q.copy()
        step = arm.solve((3.0, 0.0, 0.0), None)
        self.assertEqual(step.status, "hold")
        np.testing.assert_array_equal(step.q, before)


if __name__ == "__main__":
    unittest.main()

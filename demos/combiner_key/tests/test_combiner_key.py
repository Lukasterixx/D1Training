"""The combiner key demo's geometry, without a simulator: the key's frames, the mirrored mesh, the stances, and the arm
IK reaching the insertion from the insertion stance."""
from __future__ import annotations

import math
import unittest

import numpy as np

from demos.combiner_key import geometry as kg
from demos.combiner_key.asset import key_mesh_points
from unifp_train import leg_cfg


def key_prim_rotation(theta_deg):
    u, n, t = kg.key_rotation_box(theta_deg)
    return np.column_stack([n, u, t])


class Frames(unittest.TestCase):
    def test_the_key_frame_is_a_rotation_at_every_angle(self):
        for theta in (-45.0, 0.0, 30.0, 45.0):
            r = key_prim_rotation(theta)
            self.assertTrue(np.allclose(r.T @ r, np.eye(3), atol=1e-9))
            self.assertAlmostEqual(float(np.linalg.det(r)), 1.0, places=9)

    def test_it_turns_clockwise_as_the_robot_sees_it(self):
        # The robot looks along -X (into the door); its right is +Y. Starting up and right, the lever goes down.
        up = np.asarray(kg.lever_dir_box(kg.INSERT_DEG))
        down = np.asarray(kg.lever_dir_box(kg.TURN_END_DEG))
        self.assertGreater(up[1], 0.7)
        self.assertGreater(up[2], 0.7)
        self.assertGreater(down[1], 0.7)
        self.assertLess(down[2], -0.7)
        # Clockwise seen looking along -X is a positive turn about -X.
        axis = np.cross(up, down)
        self.assertGreater(float(axis @ np.array([-1.0, 0.0, 0.0])), 0.9)

    def test_the_key_in_the_hand_is_a_rotation(self):
        rows = np.asarray(kg.KEY_AXES_LINK6)
        prim = np.column_stack([rows[1], rows[0], rows[2]])
        self.assertAlmostEqual(float(np.linalg.det(prim)), 1.0, places=9)

    def test_the_leg_plant_and_the_demo_agree_on_the_key(self):
        self.assertAlmostEqual(math.radians(kg.INSERT_DEG), leg_cfg.KEY_THETA_START_RAD, places=9)
        self.assertAlmostEqual(math.radians(kg.UNLOCK_DEG), leg_cfg.KEY_THETA_UNLOCK_RAD, places=9)
        self.assertAlmostEqual(kg.ADAPTER.face_off_door, leg_cfg.KEY_ADAPTER_FACE_M, places=9)
        self.assertAlmostEqual(leg_cfg.KEY_BAR_BOX[1][0], kg.ADAPTER.face_off_door, places=9)


class Mesh(unittest.TestCase):
    def test_the_mirrored_key_points_along_its_lever_with_the_plate_toward_the_robot(self):
        points, counts, indices = key_mesh_points()
        p = np.asarray(points)
        self.assertEqual(len(indices), 3 * len(counts))
        # x off the door: the shaft behind the bar's back face (x < 0), the plate out to 50 mm.
        self.assertAlmostEqual(float(p[:, 0].min()), -0.010, places=4)
        self.assertAlmostEqual(float(p[:, 0].max()), 0.050, places=4)
        # y along the lever: 15 mm behind the shaft to 160 mm out.
        self.assertAlmostEqual(float(p[:, 1].min()), -0.015, places=3)
        self.assertAlmostEqual(float(p[:, 1].max()), 0.160, places=3)
        # The platform's top 10 mm above the axis; the plate (x > 16 mm) is in the lever's outer part.
        self.assertAlmostEqual(float(p[:, 2].max()), 0.010, places=4)
        plate = p[p[:, 0] > 0.03]
        self.assertGreater(float(plate[:, 1].min()), 0.045)

    def test_faces_still_face_out_after_the_mirror(self):
        points, _, indices = key_mesh_points()
        p = np.asarray(points)
        tri = p[np.asarray(indices).reshape(-1, 3)]
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        volume = float(np.einsum("ij,ij->i", tri[:, 0], normals).sum() / 6.0)
        self.assertGreater(volume, 4.0e-5)          # 43 cm^3, positive: outward faces


class Stances(unittest.TestCase):
    def test_the_turn_stance_has_the_lock_to_the_right(self):
        ahead, left = kg.TURN_STANCE
        self.assertAlmostEqual(ahead, kg.LOOK_DOOR_AHEAD_M - kg.TURN_DOOR_AHEAD_M, places=9)
        self.assertAlmostEqual(left, -kg.LOCK[1] + kg.LOCK_RIGHT_OF_BASE_M, places=9)
        self.assertGreater(kg.INSERT_STANCE[0], 0.0)
        self.assertLess(kg.INSERT_STANCE[0], kg.TURN_STANCE[0])

    def test_the_turn_stance_is_inside_the_trained_keys(self):
        self.assertTrue(leg_cfg.KEY_DOOR_AHEAD_M[0] <= kg.TURN_DOOR_AHEAD_M <= leg_cfg.KEY_DOOR_AHEAD_M[1])
        self.assertTrue(leg_cfg.KEY_AXIS_SIDE_M[0] <= -kg.LOCK_RIGHT_OF_BASE_M <= leg_cfg.KEY_AXIS_SIDE_M[1])


class Reach(unittest.TestCase):
    def test_the_arm_reaches_the_insertion_level_and_rolled(self):
        from position_only.workspace import load_urdf
        from demos.cup.pick_demo.grasp import arm_clear
        from demos.unifp import decoupled_arm

        joints, links = load_urdf()
        th = math.radians(kg.INSERT_DEG)
        e_u = np.array([0.0, -math.cos(th), -math.sin(th)])       # the robot's frame: right, then up
        face = kg.INSERT_FACE_AHEAD_M
        jaw = kg.KEY_ORIGIN_LINK6[2] - kg.JAW_CENTRE_Z_M
        for base_z in (0.27, 0.31):
            seated = np.array([face - jaw, -kg.LOCK_RIGHT_OF_BASE_M, kg.LOCK[2] - base_z]) + kg.KEY_GRASP_U_M * e_u
            arm = decoupled_arm.ArmIK(joints, links)
            arm.reset(np.zeros(6))
            rot = decoupled_arm.hand_rotation(np.array([1.0, 0.0, 0.0]), e_u, 0.0, kg.GRASP_ROLL)
            for point in (seated - np.array([0.025, 0.0, 0.0]), seated):
                step = arm.solve(point, rot)
                self.assertEqual(step.status, "pose")
                self.assertTrue(bool(arm_clear(joints, step.q)[0]))


if __name__ == "__main__":
    unittest.main()

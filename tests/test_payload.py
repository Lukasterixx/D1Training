"""`payload.py`: the MID-360, its mount, the wrist RealSense and its bracket. Numpy only (no simulator)."""
import unittest

import numpy as np

import payload
from dock import read_binary_stl


def _inside(points, bounds, tolerance=0.002):
    low = np.array([b[0] for b in bounds]) - tolerance
    high = np.array([b[1] for b in bounds]) + tolerance
    return np.all((points >= low) & (points <= high), axis=1)


class TrunkParts(unittest.TestCase):
    def test_the_meshes_lie_inside_their_colliders(self):
        lidar = read_binary_stl(payload.MESH_DIR / "mid360.stl").reshape(-1, 3)
        mount = read_binary_stl(payload.MESH_DIR / "mid360_mount.stl").reshape(-1, 3)
        self.assertTrue(_inside(lidar, payload.LIDAR_CAGE_BOX_B).all())
        either = _inside(mount, payload.LIDAR_CAGE_BOX_B) | _inside(mount, payload.MOUNT_PLATE_BOX_B)
        self.assertTrue(either.all(), f"{(~either).sum()} mount vertices outside both boxes")

    def test_the_lidar_is_65_mm_on_the_front_of_the_trunk(self):
        lidar = read_binary_stl(payload.MESH_DIR / "mid360.stl").reshape(-1, 3)
        size = lidar.max(axis=0) - lidar.min(axis=0)
        self.assertAlmostEqual(size[0], 0.065, delta=0.001)
        self.assertAlmostEqual(size[1], 0.065, delta=0.001)
        # Ahead of the arm's mount (x = -0.019) and above the trunk's top (0.057 m at the dock).
        self.assertGreater(lidar[:, 0].min(), 0.14)
        self.assertGreater(lidar[:, 2].min(), 0.09)

    def test_masses_say_where_they_come_from(self):
        parts = payload.all_parts(np.eye(4))
        self.assertEqual([p.name for p in parts], ["mid360", "mid360_mount", "d435i", "wrist_bracket"])
        sources = {p.name: p.mass_source.split(":")[0] for p in parts}
        self.assertEqual(sources, {"mid360": "published", "mid360_mount": "estimated", "d435i": "published",
                                   "wrist_bracket": "estimated"})
        self.assertEqual({p.name: p.body for p in parts},
                         {"mid360": "base", "mid360_mount": "base", "d435i": "Link6", "wrist_bracket": "Link6"})


class Folding(unittest.TestCase):
    def test_a_point_mass_moves_the_centre_and_adds_by_the_parallel_axis_rule(self):
        point = payload.Part("p", "base", 1.0, "test", (1.0, 0.0, 0.0), tuple(np.zeros(9)))
        mass, com, tensor = payload.combine(1.0, (-1.0, 0.0, 0.0), np.zeros((3, 3)), [point])
        self.assertEqual(mass, 2.0)
        np.testing.assert_allclose(com, 0.0)
        # Two unit masses 1 m either side of the centre on x: I_yy = I_zz = 2, I_xx = 0.
        np.testing.assert_allclose(tensor, np.diag([0.0, 2.0, 2.0]), atol=1e-12)

    def test_principal_axes_round_trip(self):
        rng = np.random.default_rng(0)
        a = rng.normal(size=(3, 3))
        tensor = a @ a.T
        values, axes = payload.principal(tensor)
        np.testing.assert_allclose(payload.tensor_from_principal(values, axes), tensor, atol=1e-12)

    def test_the_gripper_gets_the_camera_and_the_bracket(self):
        camera, bracket = payload.wrist_parts()
        self.assertAlmostEqual(camera.mass_kg + bracket.mass_kg, 0.0987, places=4)
        # The camera's box is the D435's housing, on the camera side of the gripper (Link6 -x, F-138).
        self.assertLess(camera.com[0], -0.03)
        self.assertEqual(camera.colliders[0].size, payload.D435_SIZE_M)


class ArmClearance(unittest.TestCase):
    def test_the_folded_arm_clears_the_lidar_and_the_proxy_knows_it_is_there(self):
        from position_only import workspace

        joints, _ = workspace.load_urdf()
        self.assertTrue(workspace.clear_of_body(joints, np.zeros((1, 6)), 0.3)[0])
        # The proxy refuses a reach that passes through the lidar's cage: some sampled configurations clear the trunk
        # box alone and are now refused.
        q = workspace.sample_configs(joints, 4000, np.random.default_rng(0))
        saved = workspace.CLEARANCE_BOXES_B
        try:
            workspace.CLEARANCE_BOXES_B = ()
            trunk_only = workspace.clear_of_body(joints, q, 0.3)
        finally:
            workspace.CLEARANCE_BOXES_B = saved
        with_lidar = workspace.clear_of_body(joints, q, 0.3)
        self.assertFalse((with_lidar & ~trunk_only).any())
        self.assertGreater((trunk_only & ~with_lidar).sum(), 0)


if __name__ == "__main__":
    unittest.main()

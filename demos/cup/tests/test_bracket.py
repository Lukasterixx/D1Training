"""The wrist camera's bracket: the saved mount, the bracket mesh and Intel's case agree with one another.

These check the committed results of `pick_demo.bracket`, not the registration itself, which needs Lukas's CAD
export (kept outside the repository; its checksums are in `assets/bracket/registration.json`). They need
`trimesh`, which the Isaac environment has.
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))   # the repository root

from demos.cup.pick_demo import bracket, camera, camera_body  # noqa: E402

HAVE_TRIMESH = importlib.util.find_spec("trimesh") is not None


class SavedMountTests(unittest.TestCase):
    def setUp(self):
        self.mount = camera.load_mount(camera.DEFAULT_MOUNT_PATH)

    def test_the_default_mount_says_it_comes_from_the_cad(self):
        """Not measured and not aligned by eye: a run log has to be able to tell the three apart."""
        self.assertIn("from CAD, not measured", self.mount.source)
        _, source, _ = camera.resolve_mount(None)
        self.assertIn("from CAD, not measured", source)

    def test_the_camera_looks_30_deg_down_the_gripper(self):
        """The bracket's design angle: the optical axis tilts from the approach axis towards Link6 +x, which is
        down when the gripper is level, and image right is Link6 -y."""
        rot = self.mount.rotation
        np.testing.assert_allclose(rot[:, 2], [math.sin(math.radians(30.0)), 0.0, math.cos(math.radians(30.0))],
                                   atol=1e-4)
        np.testing.assert_allclose(rot[:, 0], [0.0, -1.0, 0.0], atol=1e-4)

    def test_the_registration_record_matches_the_saved_mount(self):
        record = json.loads(bracket.REGISTRATION_PATH.read_text())
        np.testing.assert_allclose(np.array(record["mount_pose_link6"]), self.mount.pose, atol=1e-6)
        self.assertAlmostEqual(record["camera_in_cad"]["bracket_hole_spacing_m"],
                               record["camera_in_cad"]["intel_hole_spacing_m"], delta=1e-5)


@unittest.skipUnless(HAVE_TRIMESH, "needs trimesh (the Isaac environment)")
class BracketMeshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import trimesh

        cls.mount = camera.load_mount(camera.DEFAULT_MOUNT_PATH)
        cls.points, cls.faces = bracket.bracket_mesh_link6()
        to_optical = np.linalg.inv(cls.mount.pose)
        cls.optical = cls.points @ to_optical[:3, :3].T + to_optical[:3, 3]     # the bracket in the optical frame
        intel = trimesh.load(str(camera_body.MESH_PATH), force="mesh")
        cls.case_back_mesh_z = float(intel.vertices[:, 2].min())

    def test_the_bracket_plate_lies_on_the_case_back(self):
        """Intel's case, at the saved mount, has its back face on the bracket's plate."""
        back = camera_body.MESH_TO_OPTICAL @ np.array([0.0, 0.0, self.case_back_mesh_z, 1.0])
        # Behind the case's outline only: the bracket's side lugs reach forward beside the case.
        lo, hi = (np.array(v) for v in camera_body.housing_bounds_m())
        behind = np.all((self.optical[:, :2] > lo[:2] + 0.001) & (self.optical[:, :2] < hi[:2] - 0.001), axis=1)
        self.assertGreater(int(behind.sum()), 100)
        self.assertAlmostEqual(float(self.optical[behind, 2].max()), back[2], delta=2e-5)

    def test_the_cases_mounting_holes_line_up_with_the_brackets(self):
        """Intel's two M3 holes (45 mm apart on the case's back) are on the axes of the bracket's 3 mm holes."""
        for side in (-1.0, 1.0):
            hole = camera_body.MESH_TO_OPTICAL @ np.array([side * bracket.CASE_HOLE_X_M, 0.0, 0.0, 1.0])
            radial = np.linalg.norm(self.optical[:, :2] - hole[:2], axis=1)
            ring = self.optical[(radial > 0.0010) & (radial < 0.0020)]
            centre, radius = bracket.circle_fit(ring[:, :2])
            self.assertLess(np.linalg.norm(centre - hole[:2]), 5e-5)
            self.assertAlmostEqual(radius, 0.0015, delta=5e-5)

    def test_the_bracket_stays_out_of_the_cameras_view(self):
        model = camera.CAMERAS["d435"]
        z = self.optical[:, 2]
        ahead = z > camera_body.NEAR_CLIP_PAST_HOUSING_M
        self.assertFalse(ahead.any(), "part of the bracket is in front of the camera")

    def test_the_bracket_sits_on_top_of_the_gripper(self):
        """With the gripper level, Link6 -x is up: the bracket is above the roll axis, astride the jaw plane, and
        between the wrist flange and the jaws."""
        self.assertLess(self.points[:, 0].max(), -0.03)
        self.assertAlmostEqual(float(self.points[:, 1].mean()), 0.0, delta=0.002)
        self.assertGreater(self.points[:, 2].min(), -0.001)
        self.assertLess(self.points[:, 2].max(), 0.08)


if __name__ == "__main__":
    unittest.main()

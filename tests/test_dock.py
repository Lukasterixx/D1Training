"""The expansion dock and rails drawn on the Go2 (dock.py): where the meshes are, and that they are only drawn.

Plain numpy and XML, so it runs on any Python; weld.py's use of the same meshes needs Isaac and is checked by rendering
(`demos/cup/run_camera_body_view.sh --look_at dock`).
"""
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import dock
from mount import ARM_MOUNT_POS_M

URDF = Path(__file__).resolve().parent.parent / "description" / "go2_d1.urdf"


class DockMeshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.meshes = {name: dock.read_binary_stl(dock.MESH_DIR / stl) for name, stl, _ in dock.DOCK_MESHES}
        cls.bounds = {name: (tri.reshape(-1, 3).min(0), tri.reshape(-1, 3).max(0)) for name, tri in cls.meshes.items()}

    def test_the_rails_tops_are_where_the_arm_stands(self):
        # The D1's base frame is its bottom face (mount.py), so the rails' tops are its z.
        self.assertAlmostEqual(self.bounds["expansion_dock_rails"][1][2], ARM_MOUNT_POS_M[2], delta=0.0005)

    def test_nothing_of_the_dock_is_above_the_rails(self):
        self.assertLess(self.bounds["expansion_dock"][1][2], ARM_MOUNT_POS_M[2])

    def test_the_rails_are_centred_across_the_dog(self):
        # Centred at y = +/-29 mm (F-117), which is also the D1's hole spacing across.
        lo, hi = self.bounds["expansion_dock_rails"]
        self.assertAlmostEqual(lo[1] + hi[1], 0.0, delta=0.0005)
        self.assertAlmostEqual(hi[1], 0.029 + 0.0065, delta=0.001)

    def test_the_dock_sits_on_the_trunk_in_base_link(self):
        # Registered to base_link (not left in the CAD's frame): above the trunk's top, over its middle.
        lo, hi = self.bounds["expansion_dock"]
        self.assertAlmostEqual(lo[2], 0.057, delta=0.001)
        self.assertGreater(lo[0], -0.125)
        self.assertLess(hi[0], 0.075)
        self.assertLess(max(-lo[1], hi[1]), 0.06)

    def test_the_d1s_holes_are_over_the_rail_screws_fore_and_aft(self):
        # The real arm's holes are 130 mm apart along the rail (F-126) and sit over the rails' four screws, at
        # x = -84.5 and +46.6 mm in Unitree's CAD (F-125). Across, the rails' screws are at y = +/-29 mm; the measured
        # base's mesh scales Unitree's 58 mm to 69.6 there, which nobody measured (F-126), so across is not tested.
        for screw_x, side in ((-0.0845, -1.0), (0.0466, 1.0)):
            self.assertAlmostEqual(ARM_MOUNT_POS_M[0] + side * 0.065, screw_x, delta=0.0015)
        lo, hi = self.bounds["expansion_dock_rails"]
        for side in (-1.0, 1.0):
            self.assertTrue(lo[0] < ARM_MOUNT_POS_M[0] + side * 0.065 < hi[0])

    def test_the_measured_bases_holes_are_over_the_rail_screws_across_too(self):
        # Across the rail the real holes are 58 mm apart, as Unitree's, over the screws at y = +/-29 mm (Lukas,
        # 2026-10-06). Found in the mesh itself: the vertical walls through the flange's bottom 3 mm, past the cylinder.
        tri = dock.read_binary_stl(Path(__file__).resolve().parent.parent / "d1_arm" / "meshes" / "base_link_measured.STL")
        normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        walls = tri[(np.abs(normal[:, 2]) < 0.05 * np.linalg.norm(normal, axis=1)) & (tri[:, :, 2].max(1) <= 0.0031)]
        xy = walls.reshape(-1, 3)[:, :2]
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                hole = xy[(np.sign(xy[:, 0]) == sx) & (np.sign(xy[:, 1]) == sy) & (np.abs(xy[:, 0]) > 0.06)]
                centre = (hole.max(0) + hole.min(0)) / 2
                screw = np.array([ARM_MOUNT_POS_M[0] + centre[0], sy * 0.029])
                self.assertAlmostEqual(centre[1], screw[1], delta=0.0005)
                self.assertAlmostEqual(screw[0], -0.0845 if sx < 0 else 0.0466, delta=0.0015)

    def test_a_truncated_stl_is_refused(self):
        data = (dock.MESH_DIR / dock.DOCK_MESHES[1][1]).read_bytes()
        with tempfile.NamedTemporaryFile(suffix=".stl") as handle:
            handle.write(data[:-10])
            handle.flush()
            with self.assertRaises(ValueError):
                dock.read_binary_stl(handle.name)


class DockUrdfTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ET.parse(URDF).getroot()
        cls.link = next(link for link in cls.robot.findall("link") if link.get("name") == "expansion_dock")

    def test_it_is_fixed_to_the_trunk_with_no_offset(self):
        joint = next(j for j in self.robot.findall("joint") if j.find("child").get("link") == "expansion_dock")
        self.assertEqual(joint.get("type"), "fixed")
        self.assertEqual(joint.find("parent").get("link"), "base_link")
        self.assertEqual(joint.find("origin").get("xyz"), "0 0 0")
        self.assertEqual(joint.find("origin").get("rpy"), "0 0 0")

    def test_it_is_drawn_only(self):
        self.assertIsNone(self.link.find("collision"))
        self.assertIsNone(self.link.find("inertial"))
        meshes = [v.find("geometry/mesh").get("filename").rsplit("/", 1)[1] for v in self.link.findall("visual")]
        self.assertEqual(meshes, [stl for _, stl, _ in dock.DOCK_MESHES])

    def test_its_colours_are_the_sims(self):
        colours = {m.get("name"): m.find("color").get("rgba") for m in self.robot.findall("material")
                   if m.find("color") is not None}
        for name, _, rgb in dock.DOCK_MESHES:
            np.testing.assert_allclose([float(v) for v in colours[name].split()], [*rgb, 1.0])


if __name__ == "__main__":
    unittest.main()

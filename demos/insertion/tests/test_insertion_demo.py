"""The insertion demo's key, slot and placement, without a simulator.

    python -m unittest discover -s demos/insertion/tests -t .
"""
from __future__ import annotations

import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import torch

from demos.insertion import geometry, key
from unifp_train import insert_cfg, insertion

ROOT = Path(__file__).resolve().parents[3]


class KeyTest(unittest.TestCase):
    def test_the_blade_is_the_trainings_key_and_clears_the_fingers(self):
        centre, size = key.blade_box()
        self.assertAlmostEqual(size[0] / 2, insertion.KEY_HALF_W_M)          # thin along x
        self.assertAlmostEqual(size[1] / 2, insertion.KEY_HALF_U_M)          # long along the jaw axis
        tip = centre[2] + size[2] / 2
        self.assertAlmostEqual(tip, geometry.JAW_CENTRE_Z_M + insertion.KEY_LENGTH_M)
        root = centre[2] - size[2] / 2
        self.assertGreater(root, 0.1256)                                       # the fingers end at z = 0.1256

    def test_the_urdf_gets_one_blade_on_link6_and_absolute_meshes(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = key.write_key_urdf(ROOT / "d1_arm/d1.urdf", Path(tmp) / "d1_key.urdf")
            root = ET.parse(out).getroot()
            boxes = {link.get("name"): [c for c in link.findall("collision") if c.find("geometry/box") is not None]
                     for link in root.iter("link")}
            self.assertEqual(len(boxes["Link6"]), 1)
            self.assertEqual(sum(len(v) for v in boxes.values()), 1)
            for mesh in root.iter("mesh"):
                self.assertTrue(Path(mesh.get("filename")).is_file())


    def test_the_compliant_key_hangs_on_two_limited_joints_at_the_jaw_centre(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = key.write_key_urdf(ROOT / "d1_arm/d1.urdf", Path(tmp) / "d1_key.urdf", compliant=True)
            root = ET.parse(out).getroot()
            boxes = {link.get("name"): [c for c in link.findall("collision") if c.find("geometry/box") is not None]
                     for link in root.iter("link")}
            self.assertEqual(len(boxes["key"]), 1)                              # the blade is on the key, only
            self.assertEqual(sum(len(v) for v in boxes.values()), 1)
            joints = {j.get("name"): j for j in root.iter("joint")}
            rx, ry = (joints[name] for name in key.KEY_JOINTS)
            self.assertEqual((rx.find("parent").get("link"), rx.find("child").get("link")), ("Link6", "key_gimbal"))
            self.assertEqual((ry.find("parent").get("link"), ry.find("child").get("link")), ("key_gimbal", "key"))
            self.assertAlmostEqual(float(rx.find("origin").get("xyz").split()[2]), geometry.JAW_CENTRE_Z_M, places=6)
            for joint in (rx, ry):
                self.assertAlmostEqual(float(joint.find("limit").get("upper")), insertion.GRIP_MAX_RAD, places=5)
            # The blade, in the key's frame (the pivot), still ends at the key's length.
            box = boxes["key"][0]
            z = float(box.find("origin").get("xyz").split()[2])
            length = float(box.find("geometry/box").get("size").split()[2])
            self.assertAlmostEqual(z + length / 2, insertion.KEY_LENGTH_M, places=6)
            # The URDF draws no key: the env hangs `asset.build_key_usd` on the blade's link, so it tilts as one piece.
            self.assertEqual(root.find("link[@name='key']").findall("visual"), [])
            self.assertEqual(key.key_link(True), ("key", -geometry.JAW_CENTRE_Z_M))
            self.assertEqual(key.key_link(False), ("Link6", 0.0))

    def test_the_drawn_key_is_a_closed_mesh_inside_the_collider_past_the_fingers(self):
        tris = key.key_mesh()
        (_, _, cz), (sx, sy, sz) = key.blade_box()
        for x, y, z in (p for tri in tris for p in tri):
            if z > geometry.KEY_ROOT_Z_M + 1e-9:
                self.assertLessEqual(abs(x), sx / 2 + 1e-9)
                self.assertLessEqual(abs(y), sy / 2 + 1e-9)
                self.assertLessEqual(z, cz + sz / 2 + 1e-9)
        # Outward-facing and closed: the signed volume is the ring's plus the extruded profile's.
        volume = sum(a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                     + a[2] * (b[0] * c[1] - b[1] * c[0]) for a, b, c in tris) / 6
        thickness = 2 * geometry.GEOMETRY.key_half_w
        ring = math.pi * (key.BOW_OUTER_R_M ** 2 - key.BOW_INNER_R_M ** 2) * thickness
        top = key._top_edge()
        strip = sum((z1 - z0) * ((y0 + y1) / 2 + geometry.GEOMETRY.key_half_u)
                    for (z0, y0), (z1, y1) in zip(top, top[1:])) * thickness
        self.assertAlmostEqual(volume, ring + strip, delta=0.02 * (ring + strip))


class SlotTest(unittest.TestCase):
    def test_the_physical_slot_is_inside_the_trainings_ranges(self):
        g = geometry.GEOMETRY
        self.assertTrue(insert_cfg.CLEARANCE_M[0] <= g.clearance <= insert_cfg.CLEARANCE_M[1])
        self.assertTrue(insert_cfg.CHAMFER_M[0] <= g.chamfer <= insert_cfg.CHAMFER_M[1])
        self.assertTrue(insert_cfg.DEPTH_M[0] <= g.depth <= insert_cfg.DEPTH_M[1])
        self.assertAlmostEqual(g.half_u, insertion.KEY_HALF_U_M + g.clearance)
        self.assertEqual(insert_cfg.EVAL_CONDITIONS["clearance_m"], g.clearance)
        self.assertEqual(insert_cfg.EVAL_CONDITIONS["depth_m"], g.depth)

    def test_the_door_is_placed_with_its_axes_on_the_true_slots(self):
        torch.manual_seed(0)
        axis = insertion.unit(torch.randn(8, 3))
        u, w = insertion.roll_frame(axis, torch.rand(8) * math.pi - math.pi / 2)
        q = geometry.frame_quat(axis, u, w)
        self.assertTrue(torch.allclose(q.norm(dim=-1), torch.ones(8), atol=1e-5))
        # Rotating the slot frame's x, y, z by q gives back the axis, u and w.
        def rotate(q, v):
            w_, xyz = q[:, :1], q[:, 1:]
            t = 2 * torch.cross(xyz, v, dim=-1)
            return v + w_ * t + torch.cross(xyz, t, dim=-1)
        e = torch.eye(3).unsqueeze(0).expand(8, 3, 3)
        self.assertTrue(torch.allclose(rotate(q, e[:, 0]), axis, atol=1e-5))
        self.assertTrue(torch.allclose(rotate(q, e[:, 1]), u, atol=1e-5))
        self.assertTrue(torch.allclose(rotate(q, e[:, 2]), w, atol=1e-5))

    def test_the_asset_builds_with_its_colliders(self):
        try:
            from pxr import Usd, UsdPhysics
        except ImportError:
            self.skipTest("needs pxr (the Isaac environment)")
        from demos.insertion import asset
        with tempfile.TemporaryDirectory() as tmp:
            path = asset.build_slot_usd(Path(tmp) / "slot.usd")
            stage = Usd.Stage.Open(str(path))
            colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
            names = {p.GetName() for p in colliders}
            self.assertEqual(len(colliders), 5 + 4 + 4 + 4)
            self.assertTrue({"Seat", "ChamferTop", "WallLeft", "FaceRight", "PanelBottom"} <= names)

    def test_the_scenery_floor_and_key_are_visual_only(self):
        try:
            from pxr import Usd, UsdPhysics
        except ImportError:
            self.skipTest("needs pxr (the Isaac environment)")
        from demos.insertion import asset
        with tempfile.TemporaryDirectory() as tmp:
            stage = Usd.Stage.Open(str(asset.build_scenery_usd(Path(tmp) / "scenery.usd")))
            stages = [stage, Usd.Stage.Open(str(asset.build_floor_usd(Path(tmp) / "floor.usd", wall_m=0.52))),
                      Usd.Stage.Open(str(asset.build_key_usd(Path(tmp) / "key.usd", compliant=True)))]
            for prim in (p for st in stages for p in st.Traverse()):
                for api in (UsdPhysics.CollisionAPI, UsdPhysics.RigidBodyAPI, UsdPhysics.MassAPI):
                    self.assertFalse(prim.HasAPI(api), prim.GetPath())

    def test_the_escutcheon_frames_the_keyway_and_hides_the_doors_hole(self):
        from demos.insertion import asset
        # The escutcheon's opening takes the whole mouth, chamfer and all, across any roll; the door's square hole is
        # inside the escutcheon's rim, so no edge of it shows.
        g = geometry.GEOMETRY
        self.assertGreater(asset.ESCUTCHEON_INNER_R_M, math.hypot(g.half_u + g.chamfer, g.half_w + g.chamfer) + 0.004)
        self.assertLess(asset.ESCUTCHEON_INNER_R_M, 0.95 * asset.ESCUTCHEON_OUTER_R_M / math.sqrt(2))
        self.assertLess(asset.ESCUTCHEON_OUTER_R_M, g.half_u + g.margin)       # the lock's face fills the opening


if __name__ == "__main__":
    unittest.main()

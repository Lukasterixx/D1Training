"""Tests for the browser console's server side. Numpy only; no DDS, no browser.

The page builds its scene graph from `/model.json` by composing, per joint,
translate(xyz) · R(rpy) · Rot(axis, q). A browser cannot run here, so the same
composition is done in numpy from the same JSON and checked against the solver's
FK -- if these agree, the rendered arm and the IK share one model.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import d1_ik
from d1_ui import server
from position_only.workspace import rpy_matrix, sample_configs


def _rot_axis(axis, q):
    axis = np.asarray(axis, dtype=float); axis /= np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * (k @ k)


def _page_fk(spec, q_by_joint):
    """What the page does: walk the joints, composing fixed origin then joint rotation."""
    frames = {spec["root"]: (np.eye(3), np.asarray(spec.get("mount", [0, 0, 0]), dtype=float))}
    pending = list(spec["joints"])
    while pending:
        for j in list(pending):
            if j["parent"] not in frames:
                continue
            R_p, p_p = frames[j["parent"]]
            p = p_p + R_p @ np.asarray(j["xyz"])
            R = R_p @ rpy_matrix(*j["rpy"])
            if j["type"] == "revolute":
                R = R @ _rot_axis(j["axis"], q_by_joint.get(j["name"], 0.0))
            frames[j["child"]] = (R, p)
            pending.remove(j)
    return frames


class ModelJsonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = server.build_model()
        cls.joints, _ = d1_ik.load_urdf()

    def test_page_scene_graph_matches_solver_fk(self):
        """The decisive check: the page's transform composition equals workspace.forward."""
        rng = np.random.default_rng(7)
        for q in sample_configs(self.joints, 6, rng):
            frames = _page_fk(self.model["d1"], dict(zip(d1_ik.ARM_JOINTS, q)))
            R, p = frames[self.model["tool"]["body"]]
            page_tool = p + R @ np.asarray(self.model["tool"]["offset"])
            solver_tool, solver_R = d1_ik.tool_pose(self.joints, q)
            np.testing.assert_allclose(page_tool, solver_tool, atol=1e-9)
            np.testing.assert_allclose(R, solver_R, atol=1e-9)

    def test_d1_chain_is_complete_and_mounted(self):
        d1 = self.model["d1"]
        self.assertEqual(d1["arm_joints"], [f"Joint{i}" for i in range(1, 7)])
        self.assertEqual(d1["mount"], [0.0, 0.0, 0.08])
        names = {j["name"] for j in d1["joints"]}
        self.assertTrue({"Joint1", "Joint6", "Joint7_1", "Joint7_2"} <= names)
        for link in ("base_link", "Link1", "Link6", "Link7_1"):
            self.assertTrue(d1["links"][link], f"{link} has no visual")
            self.assertTrue(d1["links"][link][0]["mesh"].startswith("/meshes/d1/"))

    def test_go2_chain_excludes_the_arm_and_covers_twelve_motors(self):
        go2 = self.model["go2"]
        names = {j["name"] for j in go2["joints"]}
        self.assertFalse(any(n.startswith("Joint") for n in names))
        self.assertNotIn("arm_mount_joint", names)
        self.assertTrue(set(go2["motor_order"]) <= names)
        self.assertEqual(len(go2["motor_order"]), 12)
        self.assertTrue(go2["links"]["base_link"][0]["mesh"].endswith("base.dae"))

    def test_servo_signs_and_limits_are_the_measured_ones(self):
        self.assertEqual(self.model["d1"]["servo_sign"], [-1.0, 1.0, 1.0, -1.0, 1.0, 1.0])
        lows, highs = self.model["d1"]["servo_limits_deg"]
        self.assertTrue(all(lo < hi for lo, hi in zip(lows, highs)))

    def test_sphere_sits_above_the_mount(self):
        sph = self.model["sphere"]
        self.assertGreater(sph["min_z"], self.model["d1"]["mount"][2])
        self.assertGreater(sph["center"][2], sph["min_z"])


class TargetValidationTests(unittest.TestCase):
    """validate_target is pure on the model, so it is exercised without an ArmServer."""

    def setUp(self):
        self.srv = server.ArmServer.__new__(server.ArmServer)
        self.srv.model = server.build_model(0.40)

    def test_rejects_below_the_mount_plane(self):
        ok, why = self.srv.validate_target([0.2, 0.0, 0.05])
        self.assertFalse(ok); self.assertIn("mount plane", why)

    def test_rejects_outside_the_sphere(self):
        c = self.srv.model["sphere"]["center"]
        ok, why = self.srv.validate_target([c[0] + 0.9, c[1], c[2] + 0.1])
        self.assertFalse(ok); self.assertIn("sphere", why)

    def test_rejects_garbage(self):
        self.assertFalse(self.srv.validate_target([1.0, float("nan"), 0.3])[0])
        self.assertFalse(self.srv.validate_target([1.0, 2.0])[0])

    def test_accepts_a_point_on_the_upper_sphere(self):
        c = self.srv.model["sphere"]["center"]; r = self.srv.model["sphere"]["radius"]
        ok, why = self.srv.validate_target([c[0] + r * 0.6, c[1], c[2] + r * 0.8])
        self.assertTrue(ok, why)


if __name__ == "__main__":
    unittest.main()

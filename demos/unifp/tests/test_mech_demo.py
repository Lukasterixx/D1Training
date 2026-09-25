"""CPU tests for the mechanism-policy combiner demo's claw, force law and roll command (`demos/unifp/mech.py`).

The claw and the law act at the physics and policy rates on a real articulated box, and a sign or a
frame error in either shows up as a demo that pulls the door shut or pushes the lever up. These pin
them without a simulator.

    python -m unittest demos.unifp.tests.test_mech_demo -v
"""
from __future__ import annotations

import math
import unittest

try:
    import torch
except ImportError:  # pragma: no cover - the system Python has no torch
    torch = None


@unittest.skipIf(torch is None, "needs torch")
class ClawTest(unittest.TestCase):
    def setUp(self):
        from demos.unifp import mech
        self.claw = mech.ClawCoupling(2, "cpu", stiffness=2000.0, damping=10.0, grip_n=150.0)
        self.claw.engage(torch.tensor([0]), torch.zeros(1, 3))

    def test_the_claw_pulls_the_tool_back_to_its_anchor(self):
        force = self.claw.force(torch.tensor([[0.01, 0.0, 0.0]] * 2), torch.zeros(2, 3),
                                torch.zeros(2, 3), torch.zeros(2, 3))
        self.assertTrue(torch.allclose(force[0], torch.tensor([-20.0, 0.0, 0.0]), atol=1e-4))
        # The second environment never hooked the bar: no force.
        self.assertEqual(float(force[1].norm()), 0.0)

    def test_damping_resists_the_relative_motion_only(self):
        moving = torch.tensor([[0.0, 0.2, 0.0]] * 2)
        force = self.claw.force(torch.zeros(2, 3), moving, torch.zeros(2, 3), moving)
        self.assertEqual(float(force[0].norm()), 0.0)

    def test_the_lever_tears_out_above_the_grip(self):
        self.claw.force(torch.tensor([[0.08, 0.0, 0.0]] * 2), torch.zeros(2, 3), torch.zeros(2, 3), torch.zeros(2, 3))
        self.assertTrue(bool(self.claw.torn[0]))
        self.assertFalse(bool(self.claw.engaged[0]))
        again = self.claw.force(torch.tensor([[0.01, 0.0, 0.0]] * 2), torch.zeros(2, 3),
                                torch.zeros(2, 3), torch.zeros(2, 3))
        self.assertEqual(float(again[0].norm()), 0.0)


@unittest.skipIf(torch is None, "needs torch")
class ForceLawTest(unittest.TestCase):
    def law(self, **kwargs):
        from demos.unifp import mech
        return mech.PlaneForceLaw(1, "cpu", **{"kp": 400.0, "ki": 1000.0, "max_n": 80.0, "bleed_s": 1.0, **kwargs})

    def test_projection_keeps_what_the_mechanism_can_do_and_drops_the_rest(self):
        from demos.unifp import mech
        jac = torch.tensor([[[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]]])      # moves in x and y only
        self.assertTrue(torch.allclose(mech.project_onto(torch.tensor([[0.3, -0.2, 0.5]]), jac),
                                       torch.tensor([[0.3, -0.2, 0.0]]), atol=1e-6))
        # Columns that are not orthonormal (a lever and a door at an angle) project the same way.
        skew = torch.tensor([[[1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]])
        self.assertTrue(torch.allclose(mech.project_onto(torch.tensor([[0.3, -0.2, 0.5]]), skew),
                                       torch.tensor([[0.3, -0.2, 0.0]]), atol=1e-5))

    def test_the_integral_escalates_while_the_handle_stays_behind(self):
        law = self.law()
        lag = torch.tensor([[0.03, 0.0, 0.0]])
        first = float(law.step(lag, torch.tensor([True]), 0.02)[0, 0])
        self.assertAlmostEqual(first, 400.0 * 0.03 + 1000.0 * 0.03 * 0.02, places=4)
        for _ in range(50):
            later = float(law.step(lag, torch.tensor([True]), 0.02)[0, 0])
        self.assertGreater(later, first + 20.0)

    def test_the_command_is_capped_in_magnitude(self):
        law = self.law()
        for _ in range(500):
            force = law.step(torch.tensor([[0.2, 0.2, 0.0]]), torch.tensor([True]), 0.02)
        self.assertAlmostEqual(float(force.norm()), 80.0, places=3)

    def test_the_integral_never_pushes_against_the_lag(self):
        law = self.law()
        for _ in range(20):
            law.step(torch.tensor([[0.05, 0.0, 0.0]]), torch.tensor([True]), 0.02)
        force = law.step(torch.tensor([[-0.01, 0.0, 0.0]]), torch.tensor([True]), 0.02)
        self.assertLess(float(force[0, 0]), 0.0)

    def test_the_integral_bleeds_once_the_handle_has_arrived(self):
        law = self.law()
        for _ in range(50):
            law.step(torch.tensor([[0.05, 0.0, 0.0]]), torch.tensor([True]), 0.02)
        held = float(law.integral.norm())
        for _ in range(100):
            law.step(torch.tensor([[0.001, 0.0, 0.0]]), torch.tensor([True]), 0.02)
        self.assertLess(float(law.integral.norm()), 0.3 * held)

    def test_an_inactive_environment_is_commanded_nothing(self):
        law = self.law()
        force = law.step(torch.tensor([[0.05, 0.0, 0.0]]), torch.tensor([False]), 0.02)
        self.assertEqual(float(force.norm()), 0.0)


@unittest.skipIf(torch is None, "needs torch")
class RollTest(unittest.TestCase):
    def test_jaws_across_a_level_bar_are_upright(self):
        from demos.unifp import mech
        approach = torch.tensor([[1.0, 0.0, 0.0]])
        forward = torch.tensor([[1.0, 0.0, 0.0]])
        upright = mech.jaw_roll(approach, torch.tensor([[0.0, 0.0, 1.0]]), forward)
        self.assertAlmostEqual(abs(float(upright[0])), math.pi / 2, places=5)
        level = mech.jaw_roll(approach, torch.tensor([[0.0, 1.0, 0.0]]), forward)
        self.assertAlmostEqual(float(level[0]), 0.0, places=5)

    def test_it_is_the_quantity_training_tracked(self):
        try:
            from unifp_train import rewards
        except ImportError:  # pragma: no cover
            self.skipTest("unifp_train needs torch")
        from demos.unifp import mech
        torch.manual_seed(0)
        approach = mech.unit(torch.randn(64, 3))
        jaw = mech.unit(torch.cross(approach, torch.randn(64, 3), dim=-1))
        forward = torch.tensor([[1.0, 0.0, 0.0]]).expand(64, 3)
        blank = torch.zeros(64, 1)
        state = rewards.TaskState(*([blank] * 30))
        state.ee_approach_w, state.ee_jaw_w, state.base_forward_w = approach, jaw, forward
        self.assertTrue(torch.allclose(mech.jaw_roll(approach, jaw, forward), rewards.tool_roll(state), atol=1e-5))


class LipGeometryTest(unittest.TestCase):
    """The L-claw's lips (`demos/unifp/claw.py`): where they are, and what they allow. numpy only."""

    def test_the_finger_frame_box_lands_where_the_hand_frame_says(self):
        import numpy as np
        from demos.unifp import claw
        for finger, spec in claw.FINGERS.items():
            centre, size = claw.lip_box_link6(finger)
            xyz, rpy, size_f = claw.lip_box_finger(finger)
            rot = claw.rpy_matrix(*spec["rpy"])
            back = rot @ xyz + np.asarray(spec["xyz"])
            self.assertTrue(np.allclose(back, centre, atol=1e-9))
            # The box's axes are Link6's: its rotation in Link6 is the identity.
            self.assertTrue(np.allclose(rot @ claw.rpy_matrix(*rpy), np.eye(3), atol=1e-6))
            self.assertTrue(np.allclose(size, size_f))

    def test_the_lips_pass_each_other_and_each_is_fixed_to_its_own_finger(self):
        from demos.unifp import claw
        c1, s1 = claw.lip_box_link6("Link7_1")
        c2, s2 = claw.lip_box_link6("Link7_2")
        x1 = (c1[0] - s1[0] / 2, c1[0] + s1[0] / 2)
        x2 = (c2[0] - s2[0] / 2, c2[0] + s2[0] / 2)
        self.assertGreaterEqual(x1[0] - x2[1], claw.LIP_CLEARANCE_M - 1e-9)   # disjoint halves of the width
        # Link7_1 sits at -y: its lip starts over its own end (y < -8.6 mm) and reaches 20 mm past its inner face.
        self.assertLess(c1[1] - s1[1] / 2, -claw.FINGER_INNER_Y_M)
        self.assertAlmostEqual(c1[1] + s1[1] / 2, -claw.FINGER_INNER_Y_M + claw.LIP_LENGTH_M, places=9)
        self.assertGreater(c2[1] + s2[1] / 2, claw.FINGER_INNER_Y_M)
        self.assertAlmostEqual(c2[1] - s2[1] / 2, claw.FINGER_INNER_Y_M - claw.LIP_LENGTH_M, places=9)
        # Beyond the fingertips, not across them.
        self.assertGreater(c1[2] - s1[2] / 2, claw.FINGER_END_Z_M)

    def test_at_full_close_each_lip_overbites_the_other_finger_without_touching_it(self):
        import struct
        from pathlib import Path
        import numpy as np
        from demos.unifp import claw

        def link6_vertices(finger):
            path = Path(__file__).resolve().parents[3] / "d1_arm" / "meshes" / f"{finger}.STL"
            data = path.read_bytes()
            count = struct.unpack("<I", data[80:84])[0]
            tri = np.frombuffer(data[84:84 + 50 * count],
                                dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"].reshape(-1, 3)
            spec = claw.FINGERS[finger]
            return tri.astype(float) @ claw.rpy_matrix(*spec["rpy"]).T + np.asarray(spec["xyz"])

        for lip, other in (("Link7_1", "Link7_2"), ("Link7_2", "Link7_1")):
            centre, size = claw.lip_box_link6(lip)
            lo, hi = centre - size / 2, centre + size / 2
            verts = link6_vertices(other)       # jaws shut: zero travel
            inside = np.all((verts > lo) & (verts < hi), axis=1)
            self.assertFalse(inside.any(), f"{lip}'s lip cuts into {other} at full close")
            # And it does reach over the other finger's end: an overbite, not a stop short of it.
            under = (np.abs(verts[:, 0] - centre[0]) < size[0] / 2) & (np.abs(verts[:, 1] - centre[1]) < size[1] / 2)
            self.assertTrue(under.any())
        self.assertEqual(claw.LIP_STOP_TRAVEL_M, 0.0)

    def test_what_the_lips_allow(self):
        from demos.unifp import claw
        self.assertAlmostEqual(claw.entry_gap_m(claw.MAX_TRAVEL_M), 0.0372, places=6)
        self.assertLess(claw.entry_gap_m(0.012), 0.018)      # the friction grip's 41.2 mm opening admits no bar

    def test_the_urdf_gets_a_lip_on_each_finger_and_absolute_meshes(self):
        import tempfile
        import xml.etree.ElementTree as ET
        from pathlib import Path
        from demos.unifp import claw
        source = Path(__file__).resolve().parents[3] / "d1_arm" / "d1.urdf"
        with tempfile.TemporaryDirectory() as tmp:
            out = claw.write_claw_urdf(source, Path(tmp) / "d1_claw.urdf")
            root = ET.parse(out).getroot()
            for link in root.iter("link"):
                boxes = [c for c in link.findall("collision") if c.find("geometry/box") is not None]
                self.assertEqual(len(boxes), 1 if link.get("name") in claw.FINGERS else 0)
            for mesh in root.iter("mesh"):
                self.assertTrue(Path(mesh.get("filename")).is_file(), mesh.get("filename"))


if __name__ == "__main__":
    unittest.main()

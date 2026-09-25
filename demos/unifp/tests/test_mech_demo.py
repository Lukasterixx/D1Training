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


if __name__ == "__main__":
    unittest.main()

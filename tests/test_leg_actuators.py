"""The leg task's motor models (`unifp_train/leg_actuators.py`) and the geared knee they derive from the URDF."""
import re
import unittest
from pathlib import Path

import torch

import motor_model
from unifp_train import leg_actuators

URDF = Path(__file__).resolve().parents[1] / "description" / "go2_d1.urdf"


def urdf_limit(joint: str) -> tuple[float, float]:
    text = URDF.read_text()
    block = text[text.index(f'<joint name="{joint}"'):]
    limit = re.search(r"<limit [^>]*>", block[:block.index("</joint>")]).group(0)
    return float(re.search(r'effort="([^"]+)"', limit).group(1)), float(re.search(r'velocity="([^"]+)"', limit).group(1))


class GearedKnee(unittest.TestCase):
    def test_the_urdf_has_the_same_ratio_in_torque_and_speed(self):
        for leg in ("FL", "FR", "RL", "RR"):
            thigh, calf = urdf_limit(f"{leg}_thigh_joint"), urdf_limit(f"{leg}_calf_joint")
            self.assertAlmostEqual(calf[0] / thigh[0], motor_model.GO2_KNEE_RATIO, places=6)
            self.assertAlmostEqual(thigh[1] / calf[1], motor_model.GO2_KNEE_RATIO, places=2)

    def test_the_geared_envelope_is_the_hip_s_scaled(self):
        knee, hv, r = motor_model.GO2_HV_KNEE, motor_model.GO2_HV, motor_model.GO2_KNEE_RATIO
        self.assertAlmostEqual(knee.Y2, 44.85, delta=0.01)            # near the URDF's 45.43
        self.assertAlmostEqual(knee.X2, 15.65, delta=0.01)            # near the URDF's 15.70
        # The knee's limit at a speed equals the hip's at r times that speed, times r.
        for speed in (0.0, 5.0, 7.5, 12.0, 15.0):
            v = torch.tensor([speed])
            knee_limit = motor_model.unitree_effort_limit(torch.tensor([1.0]), v, knee.Y1, knee.Y2, knee.X1, knee.X2)
            hip_limit = motor_model.unitree_effort_limit(torch.tensor([1.0]), v * r, hv.Y1, hv.Y2, hv.X1, hv.X2)
            self.assertAlmostEqual(knee_limit.item(), r * hip_limit.item(), places=4)


class Models(unittest.TestCase):
    def test_every_model_is_described(self):
        for name in leg_actuators.LEG_ACTUATORS:
            d = leg_actuators.describe(name)
            self.assertEqual(d["gains"], [25.0, 0.5])
            self.assertIn("knee_nm", d)

    def test_the_knee_split_covers_the_legs(self):
        self.assertEqual(sorted(leg_actuators.HIP_THIGH + leg_actuators.KNEES), sorted(leg_actuators.LEGS))
        self.assertEqual(len(leg_actuators.KNEES), 4)

    def test_delay_needs_a_delay_buffer(self):
        with self.assertRaises(ValueError):
            leg_actuators.swap_legs(object(), "dcmotor", 2)
        with self.assertRaises(ValueError):
            leg_actuators.swap_legs(object(), "unknown")


if __name__ == "__main__":
    unittest.main()

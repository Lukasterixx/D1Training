"""CPU checks for the parts adapted from unitree_rl_lab: motor envelope and deploy manifest."""
import unittest

import torch

from motor_model import GO2_HV, clip_unitree_effort, dc_motor_effort_bounds, joint_friction, unitree_effort_limit
from position_only.deploy import GO2_SDK_JOINT_ORDER, build_manifest


def go2_limit(effort, velocity):
    return unitree_effort_limit(torch.tensor([effort]), torch.tensor([velocity]),
                                GO2_HV.Y1, GO2_HV.Y2, GO2_HV.X1, GO2_HV.X2).item()


class UnitreeEnvelopeTests(unittest.TestCase):
    def test_full_torque_below_the_knee_and_driving_versus_braking(self):
        self.assertAlmostEqual(go2_limit(30.0, 5.0), 20.2, places=5)    # torque with motion: Y1
        self.assertAlmostEqual(go2_limit(-30.0, 5.0), 23.4, places=5)   # torque against motion: Y2
        self.assertAlmostEqual(go2_limit(30.0, 0.0), 23.4, places=5)    # at rest counts as not driving

    def test_limit_falls_linearly_to_zero_at_no_load_speed(self):
        midpoint = (GO2_HV.X1 + GO2_HV.X2) / 2
        self.assertAlmostEqual(go2_limit(30.0, midpoint), GO2_HV.Y1 / 2, places=4)
        self.assertAlmostEqual(go2_limit(30.0, -GO2_HV.X2), 0.0, places=5)   # speed magnitude, either sign
        self.assertEqual(go2_limit(30.0, 40.0), 0.0)                        # never negative past X2

    def test_clip_keeps_sign_and_small_demands(self):
        effort = torch.tensor([30.0, -30.0, 5.0])
        velocity = torch.tensor([5.0, 5.0, 5.0])
        clipped = clip_unitree_effort(effort, velocity, GO2_HV.Y1, GO2_HV.Y2, GO2_HV.X1, GO2_HV.X2)
        torch.testing.assert_close(clipped, torch.tensor([20.2, -23.4, 5.0]))

    def test_friction_opposes_motion(self):
        friction = joint_friction(torch.tensor([-1.0, 0.0, 1.0]), 1.6, 0.16, 0.01)
        torch.testing.assert_close(friction, torch.tensor([-1.76, 0.0, 1.76]))

    def test_stock_dc_motor_gives_less_torque_mid_range(self):
        """The difference that motivated the port: at 13.5 rad/s the stock model allows 12.9 N·m."""
        _, top = dc_motor_effort_bounds(torch.tensor([13.5]), 23.5, 23.5, 30.0)
        self.assertAlmostEqual(top.item(), 23.5 * (1 - 13.5 / 30), places=4)
        self.assertAlmostEqual(go2_limit(30.0, 13.49), 20.2, places=4)


def action(name, joints, offset=0.0):
    return {"name": name, "joints": joints, "raw_clip": [-1.0, 1.0], "scale": [0.25] * len(joints),
            "offset": [offset] * len(joints), "target_limits": [[-1.0, 1.0]] * len(joints)}


class DeployManifestTests(unittest.TestCase):
    LEGS = [f"{leg}_{part}_joint" for part in ("hip", "thigh", "calf") for leg in ("FL", "FR", "RL", "RR")]
    ARM = [f"Joint{i}" for i in range(1, 7)]

    def test_maps_policy_outputs_to_motor_ids_on_each_bus(self):
        manifest = build_manifest(
            0.02, [action("legs", self.LEGS), action("arm", self.ARM)],
            [{"name": "base_ang_vel", "dim": 3, "scale": None, "clip": None, "history_length": 1},
             {"name": "joint_pos", "dim": 18, "scale": None, "clip": None, "history_length": 2}],
            [{"name": "base_legs", "model": "UnitreeActuator", "joints": self.LEGS, "stiffness": [25.0] * 12,
              "damping": [0.5] * 12, "effort_limit": [23.4] * 12, "velocity_limit": [30.0] * 12, "envelope": None}],
        )
        legs, arm = manifest["actions"]["legs"], manifest["actions"]["arm"]
        # Isaac order FL, FR, RL, RR hips; SDK order is FR, FL, RR, RL with hip/thigh/calf grouped.
        self.assertEqual(legs["motor_ids"][:4], [3, 0, 9, 6])
        self.assertEqual([GO2_SDK_JOINT_ORDER[i] for i in legs["motor_ids"]], self.LEGS)
        self.assertEqual((legs["bus"], arm["bus"], arm["motor_ids"]), ("go2_legs", "d1_arm", list(range(6))))
        self.assertEqual((legs["policy_output_indices"], arm["policy_output_indices"]), ([0, 12], [12, 18]))
        self.assertEqual(manifest["observations"]["joint_pos"]["input_indices"], [3, 39])
        self.assertEqual(manifest["policy"], {"input_width": 39, "output_width": 18})

    def test_rejects_unknown_joints_mixed_buses_and_length_mismatches(self):
        with self.assertRaises(ValueError):
            build_manifest(0.02, [action("legs", ["Joint7_1"])], [], [])
        with self.assertRaises(ValueError):
            build_manifest(0.02, [action("mixed", ["FL_hip_joint", "Joint1"])], [], [])
        bad = action("arm", self.ARM)
        bad["scale"] = [0.25]
        with self.assertRaises(ValueError):
            build_manifest(0.02, [bad], [], [])


if __name__ == "__main__":
    unittest.main()

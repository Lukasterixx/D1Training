"""CPU checks for the parts adapted from unitree_rl_lab: motor envelope and deploy manifest."""
import unittest

import torch

from motor_model import (
    D1_EFFORT_LIMIT_NM, D1_VELOCITY_LIMIT_RAD_S, GO2_HV, clip_unitree_effort, dc_motor_effort_bounds,
    interface_timing, joint_friction, unitree_effort_limit,
)
from position_only.core import SampleAndHold
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


class InterfaceTimingTests(unittest.TestCase):
    def test_estimated_profile_matches_d1_rates_under_a_50_hz_policy(self):
        timing = interface_timing("estimated", 50.0, "unitree")
        # Feedback is 6 steps, not 5, because the arm's measured angle cycle is 111 ms (9.0 Hz), while
        # commands are still modelled at the driver's 10 Hz streaming rate. See F-020.
        self.assertEqual(timing, {"leg_delay_physics_steps": (0, 2), "arm_command_hold_steps": 5,
                                  "arm_feedback_period_steps": 6})
        # The stock DCMotor legs have no delay buffer, so the profile must not promise one.
        self.assertEqual(interface_timing("estimated", 50.0, "dc_motor")["leg_delay_physics_steps"], (0, 0))
        self.assertEqual(set(interface_timing("none", 50.0, "unitree").values()), {(0, 0), 1})
        with self.assertRaises(ValueError):
            interface_timing("measured", 50.0, "unitree")

    def test_d1_limits_cover_the_six_arm_joints(self):
        self.assertEqual(list(D1_EFFORT_LIMIT_NM.values()), [3.3, 3.3, 1.7, 1.7, 1.7, 1.7])
        self.assertEqual(list(D1_VELOCITY_LIMIT_RAD_S), [f"Joint{i}" for i in range(1, 7)])


class SampleAndHoldTests(unittest.TestCase):
    def make(self, period=5, envs=2):
        hold = SampleAndHold(envs, 1, period, sample_dt=0.02)
        hold.reset(None, torch.zeros(envs, 1), generator=torch.Generator().manual_seed(0))
        return hold

    def test_samples_every_period_at_each_environments_phase(self):
        hold = self.make()
        hold.phase[:] = torch.tensor([0, 3])
        sampled = []
        for step in range(10):
            due = hold.update(torch.full((2,), step), torch.full((2, 1), float(step)))
            sampled.append(due.tolist())
        self.assertEqual([s[0] for s in sampled], [step % 5 == 0 for step in range(10)])
        self.assertEqual([s[1] for s in sampled], [(step + 3) % 5 == 0 for step in range(10)])
        # Env 1 last sampled at step 7 and holds that value.
        self.assertEqual(hold.value[:, 0].tolist(), [5.0, 7.0])

    def test_rate_is_difference_over_interval_and_zero_until_second_sample(self):
        hold = self.make(envs=1)
        hold.phase[:] = 0
        hold.update(torch.tensor([0]), torch.tensor([[0.3]]))
        self.assertEqual(hold.rate.item(), 0.0)                  # first sample after reset: no velocity yet
        hold.update(torch.tensor([5]), torch.tensor([[0.4]]))
        self.assertAlmostEqual(hold.rate.item(), 0.1 / (5 * 0.02), places=5)

    def test_repeat_update_in_one_step_is_harmless_and_reset_restores_hold(self):
        hold = self.make(envs=1)
        hold.phase[:] = 0
        hold.update(torch.tensor([0]), torch.tensor([[1.0]]))
        hold.update(torch.tensor([5]), torch.tensor([[2.0]]))
        rate = hold.rate.clone()
        hold.update(torch.tensor([5]), torch.tensor([[9.0]]))   # observations recomputed in the same step
        torch.testing.assert_close(hold.value, torch.tensor([[2.0]]))
        torch.testing.assert_close(hold.rate, rate)
        hold.reset(torch.tensor([0]), torch.tensor([[0.5]]))
        self.assertEqual((hold.value.item(), hold.rate.item(), bool(hold.primed.item())), (0.5, 0.0, False))
        self.assertTrue(0 <= hold.phase.item() < 5)


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
            timing={"leg_delay_physics_steps": (0, 2), "arm_command_hold_steps": 5},
        )
        legs, arm = manifest["actions"]["legs"], manifest["actions"]["arm"]
        # Isaac order FL, FR, RL, RR hips; SDK order is FR, FL, RR, RL with hip/thigh/calf grouped.
        self.assertEqual(legs["motor_ids"][:4], [3, 0, 9, 6])
        self.assertEqual([GO2_SDK_JOINT_ORDER[i] for i in legs["motor_ids"]], self.LEGS)
        self.assertEqual((legs["bus"], arm["bus"], arm["motor_ids"]), ("go2_legs", "d1_arm", list(range(6))))
        self.assertEqual((legs["policy_output_indices"], arm["policy_output_indices"]), ([0, 12], [12, 18]))
        self.assertEqual(manifest["observations"]["joint_pos"]["input_indices"], [3, 39])
        self.assertEqual(manifest["policy"], {"input_width": 39, "output_width": 18})
        self.assertEqual(manifest["timing"], {"leg_delay_physics_steps": [0, 2], "arm_command_hold_steps": 5})
        self.assertEqual(legs["hold_steps"], 1)

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

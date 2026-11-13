"""The key held by the arm while the foot turns it (`unifp_train/keyhold.py`, `keyhold_env.py`): the grip's geometry and
the arm's reach to the raised box, on the CPU model. No simulator."""
from __future__ import annotations

import math
import unittest

import numpy as np
import torch

from unifp_train import arm_kin, keyhold, leg_cfg

import json
from pathlib import Path

#: The stance the episodes start from with the arm out (`demos/combiner_key/assets/keyhold_settled_arm_out.json`).
_STANCE = json.loads((Path(__file__).resolve().parents[1] / "demos/combiner_key/assets/keyhold_settled_arm_out.json")
                     .read_text())[0]
SETTLED_BASE_M = _STANCE["base_z"]
PITCH = _STANCE["pitch"]                     # nose down, rad


def base_rotation() -> np.ndarray:
    c, s = math.cos(PITCH), math.sin(PITCH)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def placement(door_ahead, side, height, yaw):
    """As `leg_env._place_keys` mounts a key: the door point on the shaft's axis, the normal out of it, the lever's
    level direction (env frame of a robot at the origin facing +x)."""
    normal = np.array([-math.cos(yaw), -math.sin(yaw), 0.0])
    along = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    door = np.array([door_ahead, 0.0, height]) + (side / along[1]) * along
    return door, normal, -along


class Geometry(unittest.TestCase):
    def test_hand_rotation_is_the_decoupled_arms(self):
        from demos.unifp.decoupled_arm import hand_rotation as reference

        g = np.random.default_rng(0)
        for _ in range(20):
            approach = g.normal(size=3)
            bar = np.cross(approach, g.normal(size=3))
            expected = reference(approach, bar, 0.0, leg_cfg.KEYHOLD_GRIP_ROLL)
            got = keyhold.hand_rotation(torch.tensor(approach)[None], torch.tensor(bar)[None])[0].numpy()
            np.testing.assert_allclose(got, expected, atol=1e-9)

    def test_the_bar_turns_down(self):
        bar = keyhold.bar_at(torch.tensor([[0.0, -1.0, 0.0]]), torch.tensor([math.radians(45.0)]))[0]
        self.assertAlmostEqual(float(bar[2]), -math.sqrt(0.5), places=6)


class Reach(unittest.TestCase):
    def setUp(self):
        self.kin = arm_kin.ArmKinematics("cpu", torch.float64)
        self.seeds = torch.tensor(leg_cfg.KEYHOLD_IK_SEEDS, dtype=torch.float64)

    def solve(self, door, normal, u0, turn_deg):
        grip = keyhold.grip_point(torch.tensor(door)[None], torch.tensor(normal)[None])
        bar = keyhold.bar_at(torch.tensor(u0)[None], torch.tensor([math.radians(turn_deg)], dtype=torch.float64))
        rot = keyhold.hand_rotation(-torch.tensor(normal)[None], bar)
        r = torch.tensor(base_rotation())
        p_b = (grip - torch.tensor([[0.0, 0.0, SETTLED_BASE_M]], dtype=torch.float64)) @ r       # r^T (p), row vectors
        q, ok = self.kin.inverse(p_b, r.T @ rot, self.seeds, soft=True)
        return q[0].numpy(), bool(ok[0])

    def test_every_frozen_key_is_held_from_the_arm_out_stance_through_the_turn(self):
        c = leg_cfg
        for ahead in c.KEYHOLD_EVAL_DOOR_AHEAD_M:
            for side in c.KEYHOLD_EVAL_AXIS_SIDE_M:
                for height in c.KEYHOLD_EVAL_AXIS_HEIGHT_M:
                    for yaw in (-c.KEYHOLD_EVAL_DOOR_YAW_RAD, c.KEYHOLD_EVAL_DOOR_YAW_RAD):
                        door, normal, u0 = placement(ahead, side, height, yaw)
                        q0, ok0 = self.solve(door, normal, u0, 0.0)
                        q45, ok45 = self.solve(door, normal, u0, 45.0)
                        with self.subTest(ahead=ahead, side=side, height=height, yaw=yaw):
                            self.assertTrue(ok0 and ok45)
                            # The turn is Joint6 alone, the way KEYHOLD_J6_SIGN says.
                            np.testing.assert_allclose(q45[:5], q0[:5], atol=0.02)
                            self.assertAlmostEqual(q45[5] - q0[5], c.KEYHOLD_J6_SIGN * math.radians(45.0), delta=0.02)

    def test_the_jaw_centre_lands_on_the_grip(self):
        door, normal, u0 = placement(0.50, -0.105, 0.45, 0.0)
        q, ok = self.solve(door, normal, u0, 0.0)
        self.assertTrue(ok)
        _, p = self.kin.tool(torch.tensor(q)[None])
        expected = keyhold.grip_point(torch.tensor(door)[None], torch.tensor(normal)[None])[0].numpy()
        np.testing.assert_allclose(base_rotation() @ p[0].numpy() + [0.0, 0.0, SETTLED_BASE_M], expected, atol=4e-3)


if __name__ == "__main__":
    unittest.main()

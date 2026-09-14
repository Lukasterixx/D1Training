"""CPU checks for frame correctness; these do not run PhysX."""
import math
import unittest

import torch

from position_only.core import point_in_world, tracking_reward, world_to_body


class PositionFrameTests(unittest.TestCase):
    def test_goal_stays_fixed_when_base_translates_and_yaws(self):
        goal_w = torch.tensor([[1.0, 2.0, 3.0]])
        root_w = torch.tensor([[0.0, 2.0, 2.0]])
        yaw90 = torch.tensor([[math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]])
        body_goal = world_to_body(goal_w, root_w, yaw90)
        torch.testing.assert_close(body_goal, torch.tensor([[0.0, -1.0, 1.0]]), atol=1e-6, rtol=0)
        torch.testing.assert_close(point_in_world(root_w, yaw90, body_goal), goal_w)
        torch.testing.assert_close(goal_w, torch.tensor([[1.0, 2.0, 3.0]]))

    def test_environment_translation_does_not_change_observation(self):
        origins = torch.tensor([[0.0, 0.0, 0.0], [3.0, -3.0, 0.0]])
        goals = origins + torch.tensor([0.3, 0.0, 0.7])
        roots = origins + torch.tensor([0.0, 0.0, 0.4])
        quat = torch.tensor([[1.0, 0.0, 0.0, 0.0]]).repeat(2, 1)
        result = world_to_body(goals, roots, quat)
        torch.testing.assert_close(result[0], result[1])

    def test_tip_offset_rotates_with_link(self):
        root = torch.tensor([[1.0, 0.0, 0.0]])
        quat = torch.tensor([[math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0]])
        tip = point_in_world(root, quat, torch.tensor([0.0, 0.0, 0.1]))
        torch.testing.assert_close(tip, torch.tensor([[1.1, 0.0, 0.0]]), atol=1e-6, rtol=0)
        torch.testing.assert_close(point_in_world(root, -quat, torch.tensor([0.0, 0.0, 0.1])), tip)

    def test_tracking_reward_has_metric_units_and_decreases(self):
        reward = tracking_reward(torch.tensor([0.0, 0.04, 0.08]), 0.04)
        torch.testing.assert_close(reward, torch.tensor([1.0, math.exp(-1), math.exp(-4)]))
        with self.assertRaises(ValueError):
            tracking_reward(torch.tensor([0.0]), 0)


if __name__ == "__main__":
    unittest.main()

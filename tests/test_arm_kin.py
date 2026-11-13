"""`unifp_train.arm_kin` and `pose_goals` against `position_only.workspace`, the numpy original, with no simulator."""
import math
import unittest

import numpy as np
import torch

from position_only.workspace import forward, load_urdf
from unifp_train import arm_kin, pose_goals
from unifp_train.arm_kin import ArmKinematics, rotation_angle


def draw(count, scale=(2.3, 1.5, 1.5, 2.3, 1.5, 2.3), seed=0):
    gen = torch.Generator().manual_seed(seed)
    return (torch.rand(count, 6, generator=gen, dtype=torch.float64) * 2 - 1) * torch.tensor(scale, dtype=torch.float64)


class Forward(unittest.TestCase):
    def test_matches_the_numpy_chain(self):
        kin = ArmKinematics(dtype=torch.float64)
        joints, _ = load_urdf(str(arm_kin.URDF))
        q = draw(500)
        frames, _, _ = forward(joints, q.numpy())
        rotation, position = kin.link6(q)
        np.testing.assert_allclose(position.numpy(), frames["Link6"][1], atol=1e-12)
        np.testing.assert_allclose(rotation.numpy(), frames["Link6"][0], atol=1e-12)

    def test_the_wrist_is_spherical(self):
        """The wrist centre ignores Joint4-6, and the jaw centre stays a fixed distance from it, to Joint6's 1.1 mm miss."""
        kin = ArmKinematics(dtype=torch.float64)
        q = draw(500)
        other = q.clone()
        other[:, 3:] = draw(500, seed=1)[:, 3:]
        self.assertLess(float((kin.wrist(q) - kin.wrist(other)).abs().max()), 1e-12)
        rotation, tool = kin.tool(q)
        local = torch.einsum("nji,nj->ni", rotation, tool - kin.wrist(q))
        self.assertLess(float((local - kin.tool_from_wrist).norm(dim=-1).max()), 2 * arm_kin.WRIST_AXIS_MISS_M)


class Inverse(unittest.TestCase):
    def test_round_trip(self):
        kin = ArmKinematics(dtype=torch.float64)
        seeds = torch.tensor([[0, 0, 0], [0, -0.8, 0.8], [0, 0.8, -0.8], [0, 0.5, 0.8], [0, -0.5, -0.8]], dtype=torch.float64)
        q = draw(1000, (1.2, 1.2, 1.2, 1.5, 1.2, 1.5))
        rotation, position = kin.tool(q)
        solved, ok = kin.inverse(position, rotation, seeds)
        self.assertGreater(float(ok.double().mean()), 0.98)
        got_r, got_p = kin.tool(solved)
        self.assertLess(float((got_p - position).norm(dim=-1)[ok].max()), 4e-3)
        self.assertLess(math.degrees(float(rotation_angle(got_r, rotation)[ok].max())), 0.1)


class Goals(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(0)
        self.goals = pose_goals.PoseGoalTrajectory(256, "cpu")

    def test_rotation_vector_round_trip(self):
        rotation = self.goals.kin.tool(draw(500, (1.2, 1.2, 1.2, 1.5, 1.2, 1.5)).float())[0]
        vector = pose_goals.rotvec_from_matrix(self.goals.neutral.T @ rotation)
        keep = vector.norm(dim=-1) < pose_goals.MAX_TURN_RAD
        back = self.goals.neutral @ pose_goals.matrix_from_rotvec(vector[keep])
        self.assertLess(math.degrees(float(rotation_angle(back, rotation[keep]).max())), 0.2)

    def test_episodes_start_on_the_all_zero_pose(self):
        self.assertLess(float(self.goals.current_rotvec.abs().max()), 1e-5)
        sphere, _ = self.goals.pose_of(torch.zeros(1, 6))
        np.testing.assert_allclose(self.goals.current[0].numpy(), sphere[0].numpy(), atol=1e-6)

    def test_every_commanded_pose_has_its_joint_angles(self):
        """The whole slide is reachable by construction: the command always equals the forward kinematics of joints in the box."""
        low = torch.tensor([b[0] for b in pose_goals.JOINT_BOX]) - 1e-5
        high = torch.tensor([b[1] for b in pose_goals.JOINT_BOX]) + 1e-5
        for _ in range(400):
            sphere = self.goals.step()
            q = self.goals.q_current
            self.assertTrue(bool(((q >= low) & (q <= high)).all()))
            expect, rotation = self.goals.pose_of(q)
            np.testing.assert_allclose(sphere.numpy(), expect.numpy(), atol=1e-5)
            self.assertLess(float(rotation_angle(self.goals.current_rotation, rotation).max()), 2e-3)

    def test_goals_stay_in_range_and_slide_within_the_arm(self):
        previous = self.goals.q_current.clone()
        worst = 0.0
        for _ in range(800):
            self.goals.step()
            worst = max(worst, float((self.goals.q_current - previous).abs().max()))
            previous = self.goals.q_current.clone()
        # A joint's slide speed stays under `SPEED_RAD_S`'s top, in rad per 20 ms step, and well under the Follower's cap.
        self.assertLess(worst, pose_goals.SPEED_RAD_S[1] * 0.02 * 1.05)
        self.assertLess(worst / 0.02, math.radians(45.0))
        self.assertLess(self.goals.misses, 0.02 * self.goals.draws)
        # The range holds a slide's ends; between them a straight line in joint space may bow a little past it, reachably.
        self.assertLess(float(self.goals.goal[:, 0].max()), pose_goals.RADIUS_RANGE[1] + 1e-3)
        self.assertLess(float(self.goals.current[:, 0].max()), pose_goals.RADIUS_RANGE[1] + 0.03)

    def test_primitives_are_reachable_and_what_they_say(self):
        """Each kind is drawn; a translation keeps its attitude, a turn its position; every commanded pose has an arm solution."""
        goals = pose_goals.PoseGoalTrajectory(512, "cpu", mix=pose_goals.TASK_MIX)
        box_low, box_high = goals.low - 0.05, goals.high + 0.05
        seeds = goals.q_current.clone()
        unsolved = total = 0
        for step in range(1500):
            sphere = goals.step()
            ids = goals.cartesian.nonzero().flatten()
            if step % 25 == 0 and len(ids):
                centre = torch.tensor((0.0, 0.0, 0.19))
                position = pose_goals.interface.sphere2cart(sphere[ids]) + centre
                q, good = goals.kin.track(position, goals.current_rotation[ids], goals.q_current[ids], box_low, box_high, iterations=8)
                unsolved += int((~good).sum())
                total += len(ids)
        self.assertGreater(min(goals.kinds[k] for k in ("translate", "rotate", "arc")), 20, goals.kinds)
        self.assertGreater(total, 1000)
        # Checked at three points along a segment; between them a pose may graze a limit, not leave the arm's reach.
        self.assertLess(unsolved, 0.02 * total)
        moving = goals.cartesian & (goals.seg_theta == 0.0)
        self.assertGreater(int(moving.sum()), 0)
        t = torch.clip(goals.timer / goals.traj_steps, 0.0, 1.0)
        kept = rotation_angle(goals.current_rotation[moving], goals.seg_r0[moving])
        self.assertLess(math.degrees(float(kept.max())), 0.5)
        del t, seeds

    def test_the_naive_control_draws_over_all_orientations(self):
        naive = pose_goals.PoseGoalTrajectory(512, "cpu", naive=True)
        for _ in range(600):
            naive.step()
        turn = torch.rad2deg(naive.current_rotvec.norm(dim=-1))
        self.assertGreater(float(turn.quantile(0.9)), 130.0)


if __name__ == "__main__":
    unittest.main()

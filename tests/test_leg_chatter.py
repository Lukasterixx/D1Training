"""The leg targets' chatter measure (`unifp_train/leg_chatter.py`): a target flipping every step, a smooth one, resets."""
import unittest

import torch

from unifp_train import leg_chatter

NAMES = [f"{leg}_{part}_joint" for leg in ("FL", "FR", "RL", "RR") for part in ("hip", "thigh", "calf")]


def run(targets, steps=None, upright=None, skip=1):
    """Feed `targets` (steps, envs, joints) through a tracker; `steps` the episode step counter per policy step."""
    t, n, j = targets.shape
    c = leg_chatter.LegChatter(n, j, "cpu", skip_steps=skip)
    for k in range(t):
        s = torch.full((n,), k) if steps is None else steps[k]
        u = torch.ones(n, dtype=torch.bool) if upright is None else upright[k]
        c.step(targets[k], s, u)
    return c.summary(NAMES)


class Chatter(unittest.TestCase):
    def test_a_target_flipping_every_step(self):
        a = 0.1
        flip = torch.tensor([a if k % 2 else -a for k in range(50)]).view(50, 1, 1).expand(50, 3, 12).clone()
        out = run(flip)
        self.assertAlmostEqual(out["rms_d1_rad"]["FL_thigh"], 2 * a, places=5)
        self.assertAlmostEqual(out["rms_d2_rad"]["FL_thigh"], 4 * a, places=5)
        self.assertAlmostEqual(out["d2_over_d1"]["RR_thigh"], 2.0, places=4)
        self.assertEqual(out["envs_rattling"], 3)

    def test_a_smooth_motion_has_no_second_difference(self):
        ramp = (0.01 * torch.arange(50.0)).view(50, 1, 1).expand(50, 2, 12).clone()
        out = run(ramp)
        self.assertAlmostEqual(out["rms_d1_rad"]["FL_hip"], 0.01, places=6)
        self.assertAlmostEqual(out["rms_d2_rad"]["FL_hip"], 0.0, places=6)
        self.assertEqual(out["envs_rattling"], 0)

    def test_the_first_steps_of_an_episode_and_a_fallen_base_do_not_count(self):
        # A jump at a reset (the step counter back to 0) and a flail while not upright must not count.
        targets = torch.zeros(20, 1, 12)
        targets[10:] = 1.0                              # the new episode starts at a different pose
        targets[15:20:2] += 0.5                         # flailing while fallen
        steps = [torch.tensor([k if k < 10 else k - 10]) for k in range(20)]
        upright = [torch.tensor([k < 15]) for k in range(20)]
        out = run(targets, steps, upright)
        self.assertEqual(out["rms_d1_rad"]["FR_calf"], 0.0)
        self.assertEqual(out["steps_counted"], 9 + 4)    # steps 1-9, then 11-14
        self.assertEqual(out["envs_counted"], 1)

    def test_an_episode_s_settling_is_left_out(self):
        targets = torch.zeros(60, 1, 12)
        targets[:30:2] = 0.5                             # a settling transient over the first 30 steps
        self.assertGreater(run(targets)["rms_d1_rad"]["FL_hip"], 0.1)
        out = run(targets, skip=leg_chatter.SKIP_STEPS)
        self.assertEqual(out["rms_d1_rad"]["FL_hip"], 0.0)
        self.assertEqual(out["steps_counted"], 30)       # steps 30-59

    def test_the_worst_joint_is_named(self):
        targets = torch.zeros(30, 1, 12)
        targets[1::2, 0, NAMES.index("RR_thigh_joint")] = 0.3
        out = run(targets)
        self.assertEqual(out["worst_joint"], "RR_thigh")
        self.assertEqual(out["envs_rattling"], 1)


if __name__ == "__main__":
    unittest.main()

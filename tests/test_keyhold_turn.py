"""`--key_turn_follow_weight` (F-283): the key's run ahead of the task layer's reference and the turn's reward as the arc
has it. No simulator: `unifp_train.keyhold`'s pure functions on synthetic turns shaped like the recorded ones (key 12 of
the staged run, 2026-11-12: the reference descends from -17 deg, waits at 0, then turns 0 -> 40 deg at about 50 deg/s;
the key went 0 -> 45 deg in 0.1 s at first contact)."""
import math
import unittest

import torch

from unifp_train import keyhold, leg_cfg

DEG = math.pi / 180.0
FREE = leg_cfg.KEYHOLD_TURN_AHEAD_FREE_RAD
LATCH = 30 * DEG


def reference(steps: int = 120) -> torch.Tensor:
    """The layer's reference angle per 20 ms step: descending -17 -> 0 deg (0.45 s), at 0 for 0.6 s, 0 -> 40 at 50 deg/s."""
    t = torch.arange(steps) * 0.02
    descend = -17 * DEG + 37.5 * DEG * t
    arc = (t - 1.05).clamp(min=0.0) * 50 * DEG
    return torch.where(t < 0.45, descend, arc.clamp(max=40 * DEG)).clamp(max=40 * DEG)


def jolt(ref: torch.Tensor) -> torch.Tensor:
    """The key as the policies turn it: at its start until the foot lands (0.4 s), then 45 deg in 0.1 s, at its stop."""
    t = torch.arange(len(ref)) * 0.02
    return ((t - 0.40).clamp(min=0.0, max=0.10) / 0.10) * 45 * DEG


def follow(ref: torch.Tensor) -> torch.Tensor:
    """The key as the arc asks: on the reference, 3 deg behind it, never below its start."""
    return (ref - 3 * DEG).clamp(min=0.0)


class TurnAheadTest(unittest.TestCase):
    def test_not_ahead_at_the_start_while_the_reference_descends(self):
        zero = torch.zeros(5)
        self.assertTrue(torch.equal(keyhold.turn_ahead(zero, torch.linspace(-17, -1, 5) * DEG, zero), zero))

    def test_ahead_past_the_free_band(self):
        ahead = keyhold.turn_ahead(torch.tensor([45 * DEG]), torch.tensor([0.0]), torch.tensor([0.0]))
        self.assertAlmostEqual(float(ahead), 45 * DEG - FREE, places=6)
        within = keyhold.turn_ahead(torch.tensor([9 * DEG]), torch.tensor([0.0]), torch.tensor([0.0]))
        self.assertEqual(float(within), 0.0)

    def test_the_jolt_is_priced_and_following_is_not(self):
        ref = reference()
        zero = torch.zeros_like(ref)
        priced = lambda key: float((keyhold.turn_ahead(key, ref, zero) ** 2).sum() * 0.02)
        self.assertEqual(priced(follow(ref)), 0.0)
        # The traces' jolts collect 0.13-0.50 rad^2 s (2026-11-13); this synthetic one about the same.
        self.assertGreater(priced(jolt(ref)), 0.1)


class FollowTurnTest(unittest.TestCase):
    def test_progress_capped_at_the_reference(self):
        theta, _ = keyhold.follow_turn(torch.tensor([45 * DEG]), torch.tensor([5 * DEG]), torch.tensor([0.0]),
                                       torch.tensor([LATCH]), torch.tensor([True]))
        self.assertAlmostEqual(float(theta), 5 * DEG, places=6)

    def test_latch_paid_only_once_the_reference_reaches_it(self):
        open_ = torch.tensor([True, True, False])
        _, paid = keyhold.follow_turn(torch.full((3,), 45 * DEG), torch.tensor([25 * DEG, 31 * DEG, 31 * DEG]),
                                      torch.zeros(3), torch.full((3,), LATCH), open_)
        self.assertEqual(paid.tolist(), [False, True, False])

    def test_the_jolt_pays_less_than_following(self):
        """The turn's reward over an episode, as the env weighs it (per second x 20 ms): `handle_progress` (1.5, the
        angle over the latch, 0..1) and `handle_open` (2.0) while the layer presses or holds. Before F-283's fix the
        jolt earns more; with it, its early lead earns nothing extra and the price makes it strictly worse."""
        ref = reference()
        zero, latch = torch.zeros_like(ref), torch.full_like(ref, LATCH)
        dt, weight = 0.02, -15.0

        def reward_before(key):  # to 2026-11-13: the key's own angle and latch
            return dt * float((1.5 * (key / LATCH).clamp(0, 1) + 2.0 * (key >= LATCH).float()).sum())

        def reward_after(key):
            theta, paid = keyhold.follow_turn(key, ref, zero, latch, key >= LATCH)
            price = weight * dt * float((keyhold.turn_ahead(key, ref, zero) ** 2).sum())
            return dt * float((1.5 * (theta / LATCH).clamp(0, 1) + 2.0 * paid.float()).sum()) + price

        self.assertGreater(reward_before(jolt(ref)), reward_before(follow(ref)) + 0.5)   # why the policies jolt
        self.assertLess(reward_after(jolt(ref)), reward_after(follow(ref)) - 0.5)


if __name__ == "__main__":
    unittest.main()

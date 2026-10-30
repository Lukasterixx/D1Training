"""How much a policy's leg targets chatter: each leg joint's step-to-step target change, over an evaluation.

The dog's rattle under round 4's foot-press policy (F-196) was its leg targets flipping almost every policy step: the FL
and RR thighs 0.23 / 0.16-0.18 rad rms a step on the dog, 0.27 / 0.22 in simulation on light legs at 15-20 ms, against
0.002-0.012 for policies that stand still. `leg_eval` measured it offline from a shadow dump over the 10 s stand. This
accumulates the same quantity in any environment, every evaluation, without dumping anything:

- `d1`: a joint's target change from one policy step to the next (rad). A rattle and a fast deliberate motion (a step of
  the gait) both make it large.
- `d2`: the change of that change. A target flipping back and forth every step makes it about twice `d1`; a smooth motion
  at 50 Hz makes it a small fraction of `d1`. The ratio is what tells a rattle from a gait.

Counted only from an episode's `SKIP_STEPS`th policy step on (the settling from the spawn is not a rattle; the dog
battery's dump skipped the same 0.6 s), and only while the base is upright, so resets, settling and fallen robots do not
count. Simulator-free (torch only), so the arithmetic is unit-tested on the CPU.
"""
from __future__ import annotations

import torch

#: An environment counts as rattling when any leg joint's own rms `d1` passes this (rad). Between the still policies'
#: 0.002-0.012 and the rattling ones' 0.13-0.27 (F-196, F-225).
RATTLE_RAD = 0.1
#: Policy steps at an episode's start left out (0.6 s at 50 Hz, as `logs/unifp_train/press/summarize.py`'s window).
SKIP_STEPS = 30


class LegChatter:
    """Per-environment, per-joint sums of the squared step-to-step change of the leg targets."""

    def __init__(self, num_envs: int, num_joints: int, device, skip_steps: int = SKIP_STEPS):
        self.skip_steps = max(int(skip_steps), 1)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        self._prev = zeros(num_envs, num_joints)
        self._prev_d1 = zeros(num_envs, num_joints)
        self.sum_d1 = zeros(num_envs, num_joints)
        self.sum_d2 = zeros(num_envs, num_joints)
        self.n_d1 = zeros(num_envs)
        self.n_d2 = zeros(num_envs)

    def step(self, targets: torch.Tensor, steps_in_episode: torch.Tensor, upright: torch.Tensor) -> None:
        """`targets` (envs, joints) this policy step; `steps_in_episode` the policy steps already taken in the episode."""
        d1 = targets - self._prev
        ok1 = ((steps_in_episode >= self.skip_steps) & upright).float()
        ok2 = ((steps_in_episode >= self.skip_steps + 1) & upright).float()
        self.sum_d1 += d1.square() * ok1.unsqueeze(1)
        self.sum_d2 += (d1 - self._prev_d1).square() * ok2.unsqueeze(1)
        self.n_d1 += ok1
        self.n_d2 += ok2
        self._prev = targets.clone()
        self._prev_d1 = d1

    def summary(self, joint_names) -> dict:
        """Pooled and per-environment rms of `d1` and `d2`, per joint (rad), with the rattling environments' share."""
        names = [str(n).removesuffix("_joint") for n in joint_names]
        n1, n2 = self.n_d1.clamp(min=1).unsqueeze(1), self.n_d2.clamp(min=1).unsqueeze(1)
        counted = self.n_d1 > 0
        per_env = (self.sum_d1 / n1).sqrt()[counted]
        pooled = lambda s, n: (s.sum(0) / n.sum().clamp(min=1)).sqrt()
        rms_d1 = pooled(self.sum_d1, self.n_d1)
        rms_d2 = pooled(self.sum_d2, self.n_d2)
        q = lambda p: (per_env.quantile(p, dim=0) if len(per_env) else torch.full_like(rms_d1, float("nan")))
        r = lambda t: {k: round(float(v), 5) for k, v in zip(names, t.tolist())}
        worst = int(rms_d1.argmax())
        return {
            "what": "each leg joint's position-target change from one policy step to the next (d1, rad) and the change "
                    "of that change (d2), from an episode's skip_steps-th step on with the base upright; a target "
                    "flipping every step makes d2 about twice d1, a smooth motion a small fraction of it",
            "skip_steps": self.skip_steps,
            "envs_counted": int(counted.sum()),
            "steps_counted": int(self.n_d1.sum()),
            "rms_d1_rad": r(rms_d1),
            "rms_d2_rad": r(rms_d2),
            "d2_over_d1": r(rms_d2 / rms_d1.clamp(min=1e-9)),
            "per_env_rms_d1_rad": {"median": r(q(0.5)), "p90": r(q(0.9))},
            "worst_joint": names[worst],
            "worst_joint_rms_d1_rad": round(float(rms_d1[worst]), 5),
            "rattle_rad": RATTLE_RAD,
            "envs_rattling": int((per_env > RATTLE_RAD).any(dim=1).sum()) if len(per_env) else 0,
        }

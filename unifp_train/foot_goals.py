"""The front-right foot's goal generator: UniFP's end-effector goal walk, moved from the arm to the foot.

UniFP's arm goal (`unifp_isaaclab.task.EeGoalTrajectory`) is drawn in (radius, pitch, yaw) about a centre 0.49 m above
the ground under the robot, slid to in spherical coordinates over 1-3 s, held for 0.5-2 s, and redrawn. This is the
same walk about the front-right thigh joint (`leg_cfg.GOAL_CENTRE`), with the foot's ranges and its own keep-out
rules: above the ground, outside the trunk, right of the centreline and ahead of the hip. A share of draws
(`leg_cfg.DOWN_GOAL_PROB`) put the foot back on the ground at its standing footfall, so standing on four legs again is
part of the task. Goals are carried in UniFP's three command channels at UniFP's scales, so no width changes.

Like `hook_env.HeldGoals`, chosen environments can be *held* on a fixed goal (`hold_mask`, `hold_goal`), which is how
the handle episodes hand the foot their scripted approach and the task layer's goal. Plain torch, no Isaac imports.
"""
from __future__ import annotations

import torch

from unifp_isaaclab import interface

from . import leg_cfg


def sphere_to_local(sphere: torch.Tensor) -> torch.Tensor:
    """(radius, pitch, yaw) about the goal centre -> (x, y, z) in the yaw frame from the ground under the base."""
    centre = torch.as_tensor(leg_cfg.GOAL_CENTRE, dtype=sphere.dtype, device=sphere.device)
    return centre + interface.sphere2cart(sphere)


def local_to_sphere(local: torch.Tensor) -> torch.Tensor:
    centre = torch.as_tensor(leg_cfg.GOAL_CENTRE, dtype=local.dtype, device=local.device)
    return interface.cart2sphere(local - centre)


def admissible(local: torch.Tensor) -> torch.Tensor:
    """True where a goal (yaw frame, from the ground under the base) keeps clear of the ground, trunk and left leg."""
    x, y, z = local.unbind(-1)
    (bx0, bx1), (by0, by1), (bz0, bz1) = leg_cfg.GOAL_TRUNK_BOX
    in_trunk = (x > bx0) & (x < bx1) & (y > by0) & (y < by1) & (z > bz0) & (z < bz1)
    return (z >= leg_cfg.GOAL_MIN_HEIGHT_M) & (y <= leg_cfg.GOAL_MAX_LEFT_M) & (x >= leg_cfg.GOAL_MIN_AHEAD_M) & ~in_trunk


class FootGoals:
    """One goal trajectory per robot for the front-right foot, in (radius, pitch, yaw) about `leg_cfg.GOAL_CENTRE`."""

    def __init__(self, num_envs: int, device="cpu", generator: torch.Generator | None = None, *,
                 down_goal: tuple[float, float, float] = leg_cfg.DOWN_GOAL,
                 pitch_range: tuple[float, float] = leg_cfg.GOAL_PITCH_RANGE) -> None:
        """`down_goal` and `pitch_range` are the handle task's unless given (the key task's are `leg_cfg.KEY_*`)."""
        self.num_envs, self.device, self.generator = num_envs, torch.device(device), generator
        self.down_goal, self.pitch_range = tuple(down_goal), tuple(pitch_range)
        #: The yaw range goals are drawn in (the press task's is wider, `foot_press_cfg.GOAL_YAW_RANGE`).
        self.yaw_range = tuple(leg_cfg.GOAL_YAW_RANGE)
        zeros = lambda *shape: torch.zeros(*shape, device=self.device)
        self.start, self.goal, self.current = zeros(num_envs, 3), zeros(num_envs, 3), zeros(num_envs, 3)
        self.timer = zeros(num_envs)
        self.traj_steps = self._uniform(leg_cfg.TRAJ_TIME_S, num_envs) / interface.POLICY_DT
        self.total_steps = self.traj_steps + self._uniform(leg_cfg.HOLD_TIME_S, num_envs) / interface.POLICY_DT
        #: True while the current goal is a foot-down goal (the standing footfall).
        self.down = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.hold_mask = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.hold_goal = zeros(num_envs, 3)
        #: Unused: the arm generator's roll, kept so UniFP's viewer overlay reads this class as it reads that one.
        self.roll_range = None
        self.reset()

    def _uniform(self, span, count: int) -> torch.Tensor:
        return torch.rand(count, device=self.device, generator=self.generator) * (span[1] - span[0]) + span[0]

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        index = slice(None) if env_ids is None else env_ids
        init = torch.as_tensor(leg_cfg.INIT_GOAL, device=self.device)
        self.start[index] = init
        self.current[index] = init
        self.timer[index] = 0.0
        self.down[index] = False
        self.hold_mask[index] = False
        # The first goal is drawn at once, so every episode opens with a lift.
        ids = torch.arange(self.num_envs, device=self.device) if env_ids is None else env_ids
        if len(ids) > 0:
            self.goal[ids] = init
            self._resample(ids, allow_down=False)

    def _resample(self, env_ids: torch.Tensor, allow_down: bool = True) -> None:
        self.start[env_ids] = self.goal[env_ids].clone()
        count = len(env_ids)
        draw = torch.zeros(count, 3, device=self.device)
        pending = torch.arange(count, device=self.device)
        for _ in range(12):
            n = len(pending)
            candidate = torch.stack((self._uniform(leg_cfg.GOAL_RADIUS_RANGE, n),
                                     self._uniform(self.pitch_range, n),
                                     self._uniform(self.yaw_range, n)), dim=-1)
            draw[pending] = candidate
            pending = pending[~admissible(sphere_to_local(candidate))]
            if len(pending) == 0:
                break
        if len(pending) > 0:
            draw[pending] = torch.as_tensor((0.30, 0.10, 0.0), device=self.device)
        down = torch.zeros(count, dtype=torch.bool, device=self.device)
        if allow_down:
            down = torch.rand(count, device=self.device, generator=self.generator) < leg_cfg.DOWN_GOAL_PROB
            # A foot already down is not sent down again.
            down &= ~self.down[env_ids]
            if bool(down.any()):
                jitter = (torch.rand(int(down.sum()), 2, device=self.device, generator=self.generator) * 2 - 1) \
                    * leg_cfg.DOWN_JITTER_M
                spot = torch.as_tensor(self.down_goal, device=self.device).expand(int(down.sum()), 3).clone()
                spot[:, :2] += jitter
                draw[down] = local_to_sphere(spot)
        self.goal[env_ids] = draw
        self.down[env_ids] = down
        self.timer[env_ids] = 0.0

    def step(self) -> torch.Tensor:
        """Advance one policy step; return the commanded goal (radius, pitch, yaw)."""
        t = torch.clip(self.timer / self.traj_steps, 0.0, 1.0)
        self.current = torch.lerp(self.start, self.goal, t[:, None])
        self.timer += 1
        due = ((self.timer > self.total_steps) & ~self.hold_mask).nonzero(as_tuple=False).flatten()
        if len(due) > 0:
            self._resample(due)
        self.current = torch.where(self.hold_mask.unsqueeze(-1), self.hold_goal, self.current)
        return self.current

    def goal_local(self) -> torch.Tensor:
        """The current goal in the yaw frame from the ground under the base."""
        return sphere_to_local(self.current)

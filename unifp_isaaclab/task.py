"""The moving end-effector goal UniFP's task generates, reproduced so the policy is given work.

The policy's command vector carries a goal in spherical coordinates that walks around on its own:
a target is drawn inside a reach annulus, the commanded goal slides to it over 1-3 s, holds for
0.5-2 s, and a new one is drawn from wherever it stopped. Feeding a fixed goal instead is a
different task -- it removes the tracking transient the policy spends most of its time on -- so
the generator is reproduced here rather than approximated.

Reproduced from `_resample_ee_goal`, `collision_check` and `update_curr_ee_goal` in the ported
Isaac Gym environment. Two details that are easy to lose:

  * **The trajectory and hold durations are drawn once**, in `_init_buffers`, and never redrawn.
    Each robot keeps the same cadence for the whole run.
  * **The goal sphere is centred at ground level**, under the robot's x,y, offset by
    `EE_GOAL_CENTER_OFFSET` in the *yaw-only* base frame. It does not pitch, roll or bob with the
    base -- so as the robot walks, the goal is dragged along the ground with it rather than
    carried by the body.

The keep-out box is a crude model of the robot's own body: goals inside it, or below
`EE_GOAL_UNDERGROUND_LIMIT`, are rejected and redrawn up to ten times.
"""
from __future__ import annotations

import math

import torch

from . import interface


def goal_sphere_center(root_pos_w: torch.Tensor, base_yaw_quat: torch.Tensor) -> torch.Tensor:
    """Centre of the goal sphere in world coordinates: the robot's x,y at z=0, plus the offset."""
    centre = torch.cat((root_pos_w[:, :2], torch.zeros_like(root_pos_w[:, 2:3])), dim=1)
    offset = torch.as_tensor(interface.EE_GOAL_CENTER_OFFSET, dtype=root_pos_w.dtype,
                             device=root_pos_w.device).expand_as(centre)
    return centre + interface.quat_apply(base_yaw_quat, offset)


def sphere_goal_to_world(goal_sphere: torch.Tensor, root_pos_w: torch.Tensor,
                         base_yaw_quat: torch.Tensor) -> torch.Tensor:
    """(radius, pitch, yaw) in the yaw-only base frame -> a world position."""
    local = interface.sphere2cart(goal_sphere)
    return goal_sphere_center(root_pos_w, base_yaw_quat) + interface.quat_apply(base_yaw_quat, local)


def trajectory_samples(goals: "EeGoalTrajectory", samples: int,
                       base_yaw_quat: torch.Tensor, centre: torch.Tensor) -> torch.Tensor:
    """The path the goal is following, as `samples` points from its start to its goal.

    The interpolation is done in **spherical** coordinates and converted afterwards, because that
    is where the goal itself is interpolated (`EeGoalTrajectory.advance`). Lerping the Cartesian
    endpoints instead would draw a straight line through space while the goal swept an arc, and
    the drawing would disagree with the thing it is drawing at every point but the two ends.

    Returns (N, `samples`, 3) in the same frame as `centre`.
    """
    t = torch.linspace(0.0, 1.0, samples, device=goals.start.device)
    spherical = torch.lerp(goals.start[..., None], goals.goal[..., None], t)     # (N, 3, S)
    flat = spherical.permute(0, 2, 1).reshape(-1, 3)                             # (N*S, 3)
    quat = base_yaw_quat.repeat_interleave(samples, dim=0)                       # (N*S, 4)
    cart = interface.quat_apply(quat, interface.sphere2cart(flat))
    return cart.view(-1, samples, 3) + centre.unsqueeze(1)


class EeGoalTrajectory:
    """UniFP's end-effector goal generator, one trajectory per robot."""

    def __init__(self, num_envs: int, device: str = "cpu", generator: torch.Generator | None = None,
                 roll_range: tuple[float, float] | None = None):
        self.num_envs = num_envs
        self.device = device
        self.generator = generator
        #: This repository's addition (F-099). When a range is given, a commanded gripper roll is
        #: drawn with each goal and slid along the same timer, so the hand is asked to arrive
        #: turned rather than to turn after arriving. Left as `None` the roll stays zero and every
        #: buffer below behaves exactly as upstream's does, which is what keeps a run made without
        #: it comparable with the released checkpoints.
        self.roll_range = roll_range
        self.start = torch.zeros(num_envs, 3, device=device)
        self.goal = torch.zeros(num_envs, 3, device=device)
        self.current = torch.zeros(num_envs, 3, device=device)
        self.start_roll = torch.zeros(num_envs, device=device)
        self.goal_roll = torch.zeros(num_envs, device=device)
        self.current_roll = torch.zeros(num_envs, device=device)
        self.timer = torch.zeros(num_envs, device=device)
        # Drawn once, as upstream draws them: each robot keeps its cadence for the whole run.
        self.traj_steps = self._uniform(interface.EE_GOAL_TRAJ_TIME_S, (num_envs,)) / interface.POLICY_DT
        self.total_steps = self.traj_steps + self._uniform(interface.EE_GOAL_HOLD_TIME_S,
                                                           (num_envs,)) / interface.POLICY_DT
        self.check_t = torch.linspace(0.0, 1.0, interface.EE_GOAL_COLLISION_SAMPLES, device=device)
        self.reset()

    def _uniform(self, span, shape) -> torch.Tensor:
        low, high = span
        draw = torch.rand(*shape, device=self.device, generator=self.generator)
        return draw * (high - low) + low

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        """Start every selected robot on upstream's fixed opening move.

        Not a random goal: upstream's `is_init` branch puts the start at `init_pos_start` and the
        goal at `init_pos_end`, so every episode opens with the same short sweep from pitch pi/5
        down to level at a 0.50 m radius. Keeping it means the first second of a rollout here is
        comparable with the first second of one in Isaac Gym.
        """
        index = slice(None) if env_ids is None else env_ids
        start = torch.as_tensor(interface.EE_GOAL_INIT_START, device=self.device)
        end = torch.as_tensor(interface.EE_GOAL_INIT_END, device=self.device)
        self.start[index] = start
        self.goal[index] = end
        self.current[index] = start
        self.timer[index] = 0.0
        # Upstream has no opening roll, so every episode starts level and the first draw turns it.
        self.start_roll[index] = 0.0
        self.goal_roll[index] = 0.0
        self.current_roll[index] = 0.0

    def _collides(self, start: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """True where the straight line from `start` to `goal` passes through the keep-out box."""
        samples = torch.lerp(start[..., None], goal[..., None], self.check_t)   # (N, 3, S)
        cart = interface.sphere2cart(samples.permute(2, 0, 1).reshape(-1, 3))
        cart = cart.reshape(len(self.check_t), -1, 3)
        upper = torch.as_tensor(interface.EE_GOAL_COLLISION_UPPER, device=self.device)
        lower = torch.as_tensor(interface.EE_GOAL_COLLISION_LOWER, device=self.device)
        inside = torch.all(cart < upper, dim=-1) & torch.all(cart > lower, dim=-1)
        underground = cart[..., 2] < interface.EE_GOAL_UNDERGROUND_LIMIT
        return torch.any(inside, dim=0) | torch.any(underground, dim=0)

    def _resample(self, env_ids: torch.Tensor) -> None:
        """Draw a new goal for each selected robot, starting from where its last one ended."""
        self.start[env_ids] = self.goal[env_ids].clone()
        pending = env_ids
        for _ in range(10):
            draw = torch.stack((
                self._uniform(interface.EE_GOAL_RADIUS_RANGE, (len(pending),)),
                self._uniform(interface.EE_GOAL_PITCH_RANGE, (len(pending),)),
                self._uniform(interface.EE_GOAL_YAW_RANGE, (len(pending),)),
            ), dim=-1)
            self.goal[pending] = draw
            pending = pending[self._collides(self.start[pending], self.goal[pending])]
            if len(pending) == 0:
                break
        if self.roll_range is not None:
            # Drawn uniformly and unconditionally: unlike a position, every roll is reachable from
            # every position solution, because a jaw axis repeats every 180 degrees and Joint6
            # spans 242 at its soft limits (F-099). So there is nothing to reject and no rejection
            # loop -- which is the practical reason this is the orientation worth commanding.
            # From where the roll actually *is*, not from the previous goal: the two differ by a
            # half turn whenever the last slide crossed the wrap, and starting from the goal makes
            # the command jump by pi at every resample.
            self.start_roll[env_ids] = self.current_roll[env_ids].clone()
            self.goal_roll[env_ids] = self._uniform(self.roll_range, (len(env_ids),))
        self.timer[env_ids] = 0.0

    def step(self) -> torch.Tensor:
        """Advance one policy step and return the commanded goal, (N, 3) as (radius, pitch, yaw)."""
        t = torch.clip(self.timer / self.traj_steps, 0.0, 1.0)
        self.current = torch.lerp(self.start, self.goal, t[:, None])
        if self.roll_range is not None:
            # Interpolated the short way round the half-turn: a jaw axis is an axis, so sliding
            # from +80 to -80 degrees is a 20 degree turn, not a 160 degree one.
            delta = torch.remainder(self.goal_roll - self.start_roll + math.pi / 2, math.pi) - math.pi / 2
            self.current_roll = torch.remainder(
                self.start_roll + delta * t + math.pi / 2, math.pi) - math.pi / 2
        self.timer += 1
        due = (self.timer > self.total_steps).nonzero(as_tuple=False).flatten()
        if len(due) > 0:
            self._resample(due)
        return self.current

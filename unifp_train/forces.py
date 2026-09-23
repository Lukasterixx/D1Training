"""UniFP's external-force schedule: what pushes the tool, and what the policy is told about it.

The task is called *unified position/force*, and this module is the "force" half. Without it the
environment is a whole-body reaching-and-walking task and nothing more; the force command channel
sits at zero and the end-effector reward degenerates to plain position tracking.

Two channels run on the same schedule and with the same numbers, independently drawn:

  * **command** -- written into `commands[9:12]`, so the policy is *told* to push with a force
    nothing is actually applying. The end-effector reward turns that command into a position
    target displaced past the goal, so obeying it means leaning into a wall that is not there.
  * **external** -- a real wrench applied to the tool, which the policy is *not* told about. It
    reaches the actor only through the state it disturbs, and the critic through the privileged
    observation; recovering it from the observation history is the adaptation module's job.

One push is: draw a peak force and a duration, ramp linearly from zero to the peak over that
duration, hold for `settling` steps, ramp back down over the same duration, then wait for the next
interval. Every draw also rolls whether this environment is "freed" -- 20% of the time the force is
suppressed until the next draw, which is what keeps zero-force episodes in the mix.

Deliberate departures from upstream, both of them about *when* this runs rather than what it does:

  * Upstream calls `_push_gripper` inside its decimation loop, so the schedule is evaluated four
    times per policy step against an `episode_length_buf` that only changes once. The ramps are a
    pure function of that counter, so three of the four calls recompute the same numbers; the one
    place it is not idempotent is the draw, which is redrawn up to four times with the last one
    winning -- distributionally identical to drawing once. This module is stepped once per policy
    step. The one behaviour that is genuinely lost: upstream can re-fire a push within the same
    policy step if a finished push happens to redraw an interval that the current step divides
    (about a 0.4% chance per finished push).
  * Upstream's base push is dead code -- `_push_robot_base` exists but its call in `step()` is
    commented out, so `commands[12:15]` and the base wrench are zero for the entire run. Nothing
    here implements it either; see `task_cfg`.

No Isaac imports, so the schedule can be stepped and checked without a simulator.
"""
from __future__ import annotations

import math

import torch

from unifp_isaaclab import interface

from . import task_cfg


def steps_from_seconds(seconds: float) -> int:
    """Seconds to policy steps, rounded the way legged_gym rounds them (`ceil(t / dt)`)."""
    return int(math.ceil(seconds / interface.POLICY_DT))


class PushSchedule:
    """One channel of the push: draw, ramp up, hold, ramp down, rest, repeat.

    Everything is a function of the per-environment episode step counter, which is what makes the
    schedule reproducible and testable. `step()` is called once per policy step with that counter
    and returns the force now acting, (N, 3), in whatever frame the caller drew the peak in --
    world for the external channel, the yaw-only base frame for the command channel.
    """

    def __init__(self, num_envs: int, device, *,
                 interval_s: tuple[float, float] = task_cfg.PUSH_INTERVAL_S,
                 duration_s: tuple[float, float] = task_cfg.PUSH_DURATION_S,
                 settling_s: float = task_cfg.SETTLING_TIME_GRIPPER_S,
                 force_range: tuple[float, float] = task_cfg.GRIPPER_FORCE_RANGE_N,
                 forced_prob: float = task_cfg.GRIPPER_FORCED_PROB,
                 clear_on_finish: bool = True,
                 generator: torch.Generator | None = None) -> None:
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.interval_min = steps_from_seconds(interval_s[0])
        self.interval_max = steps_from_seconds(interval_s[1])
        self.duration_min = float(steps_from_seconds(duration_s[0]))
        self.duration_max = float(steps_from_seconds(duration_s[1]))
        self.settling = float(steps_from_seconds(settling_s))
        self.force_low, self.force_high = force_range
        self.forced_prob = forced_prob
        # Upstream's command channel zeroes its held force when a push finishes and its external
        # channel does not. It makes no difference -- the down-ramp lands exactly on zero on the
        # step the finish test fires -- but the two are kept distinct rather than unified, because
        # "makes no difference" is a claim about float arithmetic and not a design decision.
        self.clear_on_finish = clear_on_finish
        self._generator = generator

        self.current = torch.zeros(num_envs, 3, device=self.device)
        self.target = torch.zeros(num_envs, 3, device=self.device)
        self.duration = torch.zeros(num_envs, device=self.device)
        self.end_time = torch.zeros(num_envs, device=self.device)
        self.active = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        #: Redrawn on every draw and *not* cleared on episode reset -- it does not need to be,
        #: because a reset puts the counter at 0, which every interval divides, so the first step
        #: of every episode draws afresh.
        self.freed = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.interval = self._draw_intervals(num_envs)

    # --- drawing ----------------------------------------------------------------------------

    def _rand(self, *shape: int) -> torch.Tensor:
        return torch.rand(*shape, device=self.device, generator=self._generator)

    def _draw_intervals(self, count: int) -> torch.Tensor:
        return torch.randint(self.interval_min, self.interval_max, (count,),
                             device=self.device, generator=self._generator)

    def _draw(self, ids: torch.Tensor, episode_length: torch.Tensor) -> None:
        count = int(ids.numel())
        self.freed[ids] = self._rand(count) > self.forced_prob
        span = self.force_high - self.force_low
        # Axis by axis, in x, y, z order, because that is the order upstream draws them in. Three
        # calls and one call of width three consume the same numbers in a different arrangement,
        # and drawing them the same way is what lets `tests/test_unifp_train.py` drive this and a
        # transcription of upstream's own code from one seed and compare the results directly.
        for axis in range(3):
            self.target[ids, axis] = self._rand(count, 1).view(count) * span + self.force_low

        duration = self._rand(count) * (self.duration_max - self.duration_min) + self.duration_min
        # Never let a push outlast its own slot: ramp up, hold and ramp down have to fit inside
        # the interval, so the duration is capped at half of what is left after the hold.
        duration = torch.minimum(duration, (self.interval[ids].to(duration.dtype) - self.settling) / 2)
        self.duration[ids] = duration
        self.end_time[ids] = episode_length[ids].to(duration.dtype) + duration
        self.active[ids] = True

    # --- stepping ---------------------------------------------------------------------------

    def step(self, episode_length: torch.Tensor) -> torch.Tensor:
        """Advance one policy step and return the force now acting, (N, 3)."""
        due = (episode_length % self.interval) == 0
        if bool(due.any()):
            self._draw(due.nonzero(as_tuple=False).flatten(), episode_length)

        if bool(self.active.any()):
            now = episode_length.to(self.current.dtype)
            floor = torch.zeros_like(self.duration)
            # Inactive environments have `duration == 0`. `torch.where` evaluates both branches,
            # so the denominator is guarded rather than the result checked: the guarded value is
            # only ever used by the branch that is thrown away.
            per_step = self.target / torch.where(self.active, self.duration,
                                                 torch.ones_like(self.duration)).unsqueeze(-1)

            rising = self.active & (episode_length < self.end_time.to(torch.int32))
            elapsed = torch.clamp(now - (self.end_time - self.duration), floor, self.duration)
            self.current = torch.where(rising.unsqueeze(-1), per_step * elapsed.unsqueeze(-1),
                                       self.current)

            falling = self.active & (episode_length > (self.end_time + self.settling).to(torch.int32))
            elapsed = torch.clamp(now - (self.end_time + self.settling), floor, self.duration)
            self.current = torch.where(falling.unsqueeze(-1),
                                       self.target - per_step * elapsed.unsqueeze(-1), self.current)

            finished = self.active & (
                episode_length >= (self.end_time + self.settling + self.duration).to(torch.int32))
            if bool(finished.any()):
                ids = finished.nonzero(as_tuple=False).flatten()
                self.active[ids] = False
                self.target[ids] = 0.0
                self.end_time[ids] = 0.0
                self.duration[ids] = 0.0
                if self.clear_on_finish:
                    self.current[ids] = 0.0
                self.interval[ids] = self._draw_intervals(int(ids.numel()))

        if bool(self.freed.any()):
            self.active[self.freed] = False
            self.target[self.freed] = 0.0
            self.current[self.freed] = 0.0
            self.end_time[self.freed] = 0.0
            self.duration[self.freed] = 0.0

        return self.current

    def reset(self, env_ids: torch.Tensor) -> None:
        """Clear the schedule for environments that just started a new episode."""
        self.current[env_ids] = 0.0
        self.target[env_ids] = 0.0
        self.end_time[env_ids] = 0.0
        self.duration[env_ids] = 0.0
        self.active[env_ids] = False


class GripperForces:
    """Both channels of the gripper push, stepped together."""

    def __init__(self, num_envs: int, device, generator: torch.Generator | None = None) -> None:
        self.commanded = PushSchedule(num_envs, device, clear_on_finish=True, generator=generator)
        self.external = PushSchedule(num_envs, device, clear_on_finish=False, generator=generator)

    def step(self, episode_length: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Return `(commanded, external)`, each (N, 3)."""
        return self.commanded.step(episode_length), self.external.step(episode_length)

    def reset(self, env_ids: torch.Tensor) -> None:
        self.commanded.reset(env_ids)
        self.external.reset(env_ids)

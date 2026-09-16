"""Frame calculations shared by the task and CPU checks (SI units, wxyz)."""
from __future__ import annotations

import torch


def rotate(quat: torch.Tensor, vectors: torch.Tensor) -> torch.Tensor:
    """Rotate vectors by unit quaternions, without changing the input tensors."""
    xyz = quat[..., 1:]
    cross = 2.0 * torch.linalg.cross(xyz, vectors, dim=-1)
    return vectors + quat[..., :1] * cross + torch.linalg.cross(xyz, cross, dim=-1)


def world_to_body(points_w, root_pos_w, root_quat_w):
    inverse = root_quat_w.clone()
    inverse[..., 1:] *= -1
    return rotate(inverse, points_w - root_pos_w)


def point_in_world(body_pos_w, body_quat_w, offset_b):
    return body_pos_w + rotate(body_quat_w, offset_b.expand_as(body_pos_w))


def tracking_reward(error_m: torch.Tensor, std_m: float) -> torch.Tensor:
    if std_m <= 0:
        raise ValueError("Tracking reward width must be positive.")
    return torch.exp(-torch.square(error_m / std_m))


class SampleAndHold:
    """Per-environment sample-and-hold of a signal every `period` steps, at a random phase.

    Models an interface slower than the policy (the D1's 10 Hz feedback and commands under a 50 Hz
    policy). Each environment samples when (step + phase) % period == 0; `rate` is the change between
    consecutive samples divided by the sample interval, which is how velocity is recovered from an
    angle-only feed. `update` is idempotent within a step, so recomputing observations is harmless.
    """

    def __init__(self, num_envs: int, dim: int, period: int, sample_dt: float, device="cpu"):
        if period < 1:
            raise ValueError("Sample period must be at least one step.")
        self.period, self.sample_dt = period, sample_dt
        self.value = torch.zeros(num_envs, dim, device=device)
        self.rate = torch.zeros(num_envs, dim, device=device)
        self.phase = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.last_step = torch.full((num_envs,), -1, dtype=torch.long, device=device)
        self.primed = torch.zeros(num_envs, dtype=torch.bool, device=device)

    def reset(self, env_ids, value: torch.Tensor, generator: torch.Generator | None = None):
        """Hold `value` (rows for `env_ids`) with zero rate until the first sample; draw new phases."""
        ids = slice(None) if env_ids is None else env_ids
        count = self.value[ids].shape[0]
        self.value[ids] = value
        self.rate[ids] = 0.0
        self.phase[ids] = torch.randint(0, self.period, (count,), device=self.phase.device, generator=generator)
        self.last_step[ids] = -1
        self.primed[ids] = False

    def update(self, step: torch.Tensor, signal: torch.Tensor) -> torch.Tensor:
        """Sample `signal` where due at `step` (per-environment episode step). Returns the mask sampled."""
        due = ((step + self.phase) % self.period == 0) & (step != self.last_step)
        fresh = due & self.primed
        self.rate[fresh] = (signal[fresh] - self.value[fresh]) / (self.period * self.sample_dt)
        self.rate[due & ~self.primed] = 0.0
        self.value[due] = signal[due]
        self.last_step[due] = step[due]
        self.primed |= due
        return due


class TrapezoidTracker:
    """The D1 firmware's motion planner: rest-to-rest trapezoids toward each commanded angle.

    The D1 does not snap to a setpoint. Given a joint angle it plans a trapezoid -- accelerate, cruise
    at its speed ceiling, decelerate -- and its servo loop follows the plan. Fitted to the six recorded
    30 deg single-joint steps (F-033 sweeps, 12 legs, 234 samples; `arm_response.py`), that plan
    reaches cruise in about a tenth of a second and stops a little harder than it starts.

    Two behaviours the simulation otherwise lacks:

    - **Finite acceleration.** PhysX's stiff position drive is limited only by torque, so the simulated
      arm reaches its speed ceiling almost instantly. The plan here limits the *target* instead, and
      the drive follows it.
    - **Lossy replanning.** A new setpoint mid-motion does not continue the old plan at its current
      speed. Streaming a waypoint every feedback cycle (F-035) covered 4.9-5.3 deg per 111 ms where a
      velocity-preserving planner gives 7.8; restarting from rest reproduces the hardware.
      `retention` is the fraction of planned velocity a new setpoint keeps (0 = restart from rest).

    `dead_time` delays each setpoint before it replaces the goal; the latest pending setpoint wins, as
    a firmware with one setpoint register would behave. Everything is per environment and per joint;
    `accel`, `decel` and `vmax` broadcast over the last dimension.

    Braking is exact in discrete time: each step's speed is capped at the largest from which the joint
    can still stop on its goal, so the plan lands on the goal at the physics rate without creeping up
    to it, passing it, or chattering about it.
    """

    def __init__(self, num_envs: int, dim: int, accel, decel, vmax, dead_time: float = 0.0,
                 retention: float = 0.0, device="cpu"):
        if not 0.0 <= retention <= 1.0:
            raise ValueError(f"Velocity retention must be in [0, 1], got {retention}")
        if dead_time < 0.0:
            raise ValueError(f"Dead time cannot be negative, got {dead_time}")
        as_row = lambda v: torch.as_tensor(v, dtype=torch.float32, device=device).expand(dim).clone()
        self.accel, self.decel, self.vmax = as_row(accel), as_row(decel), as_row(vmax)
        if (self.accel <= 0).any() or (self.decel <= 0).any() or (self.vmax <= 0).any():
            raise ValueError("Acceleration, deceleration and speed limits must be positive.")
        self.dead_time, self.retention = float(dead_time), float(retention)
        self.position = torch.zeros(num_envs, dim, device=device)
        self.velocity = torch.zeros(num_envs, dim, device=device)
        self.goal = torch.zeros(num_envs, dim, device=device)
        self.pending = torch.zeros(num_envs, dim, device=device)
        self.timer = torch.full((num_envs,), -1.0, device=device)

    def reset(self, env_ids, position: torch.Tensor):
        """Park at `position` (rows for `env_ids`) at rest, with no pending setpoint."""
        ids = slice(None) if env_ids is None else env_ids
        self.position[ids] = position
        self.goal[ids] = position
        self.pending[ids] = position
        self.velocity[ids] = 0.0
        self.timer[ids] = -1.0

    def command(self, mask: torch.Tensor, target: torch.Tensor):
        """Send `target` to the environments in `mask`; it takes effect after the dead time."""
        if not mask.any():
            return
        self.pending[mask] = target[mask]
        self.timer[mask] = self.dead_time
        if self.dead_time == 0.0:
            self._activate(mask)

    def _activate(self, mask):
        self.goal[mask] = self.pending[mask]
        self.velocity[mask] *= self.retention
        self.timer[mask] = -1.0

    def step(self, dt: float) -> torch.Tensor:
        """Advance the plan by `dt` and return the planned position."""
        waiting = self.timer >= 0.0
        if waiting.any():
            self.timer[waiting] -= dt
            due = waiting & (self.timer <= 1e-9)
            if due.any():
                self._activate(due)

        error = self.goal - self.position
        direction = torch.sign(error)
        distance = error.abs()
        heading = self.velocity * direction          # speed toward the goal; negative = moving away

        # Toward the goal (or at rest): the largest speed this step from which the joint can still
        # stop on the goal, u*dt + u^2/(2*decel) <= distance, solved for u. Accelerating is capped by
        # it, so braking begins exactly when it must and the plan lands on the goal without creeping
        # up to it or passing it.
        reachable = self.decel * (torch.sqrt(dt * dt + 2.0 * distance / self.decel) - dt)
        toward = torch.minimum(torch.minimum(heading.clamp(min=0.0) + self.accel * dt, self.vmax), reachable)
        # Away from the goal (a replan that kept speed and reversed): brake back through zero.
        away = heading + self.decel * dt
        speed = torch.where(heading < 0.0, torch.minimum(away, torch.zeros_like(away)), toward)
        self.velocity = direction * speed

        step = self.velocity * dt
        arrive = (direction != 0) & (heading >= 0.0) & (step.abs() >= distance - 1e-9)
        self.position = torch.where(arrive, self.goal, self.position + step)
        self.velocity = torch.where(arrive | (direction == 0), torch.zeros_like(self.velocity), self.velocity)
        return self.position


def box_edges(ranges, thickness: float = 0.004) -> tuple[torch.Tensor, torch.Tensor]:
    """The 12 edges of an axis-aligned box as thin cuboids: centres (12, 3) and sizes (12, 3).

    `ranges` is ((x0, x1), (y0, y1), (z0, z1)). Each edge runs along one axis at a pair of the other
    two axes' bounds, so drawing the cuboids gives a wireframe of the box.
    """
    bounds = torch.tensor(ranges, dtype=torch.float32)
    centre, length = bounds.mean(dim=1), bounds[:, 1] - bounds[:, 0]
    centres, sizes = [], []
    for axis in range(3):
        others = [a for a in range(3) if a != axis]
        for first in bounds[others[0]]:
            for second in bounds[others[1]]:
                point = centre.clone()
                point[others[0]], point[others[1]] = first, second
                size = torch.full((3,), thickness)
                size[axis] = length[axis] + thickness
                centres.append(point)
                sizes.append(size)
    return torch.stack(centres), torch.stack(sizes)


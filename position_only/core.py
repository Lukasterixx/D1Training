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


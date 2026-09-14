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

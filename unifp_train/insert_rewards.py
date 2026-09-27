"""Reward terms of the insertion task (`insert_env.py`), weighted in `insert_cfg.WEIGHTS`."""
from __future__ import annotations

import torch

from . import hook_rewards, insert_cfg
from .rewards import TaskState


def ins_progress(state: TaskState) -> torch.Tensor:
    """The key tip's depth against the task layer's reference: two exponentials, a sharp one and a broad one."""
    error = state.ins_error.abs()
    reward = 0.5 * torch.exp(-error / insert_cfg.PROGRESS_SIGMA_FINE_M) \
        + 0.5 * torch.exp(-error / insert_cfg.PROGRESS_SIGMA_COARSE_M)
    return reward * state.fixture_engaged


def ins_reach(state: TaskState) -> torch.Tensor:
    """The jaw centre's precision: to its goal before the task layer starts, across the axis after. UniFP's own position
    term is `exp(-2 error)` -- 82% at 10 cm -- which cannot tell a hand that finds a slot from one that misses it by a
    hand's width, and a policy asked for an approach as well will spend position on it (the third run, 2026-09-26)."""
    error = state.ins_reach_error
    reward = 0.5 * torch.exp(-error / insert_cfg.REACH_SIGMA_FINE_M) + 0.5 * torch.exp(-error / insert_cfg.REACH_SIGMA_COARSE_M)
    return reward * state.ins_active


def ins_seated(state: TaskState) -> torch.Tensor:
    """1.0 while the key is home."""
    return state.ins_seated


def ins_lateral(state: TaskState) -> torch.Tensor:
    """The force on the key across the estimated axis, squared per (`LATERAL_SCALE_N`)^2."""
    return torch.square(state.ins_lateral_n / insert_cfg.LATERAL_SCALE_N) * state.fixture_engaged


def ins_overforce(state: TaskState) -> torch.Tensor:
    """Contact force beyond `SAFE_N`, squared per (10 N)^2."""
    excess = (state.ins_contact_n - insert_cfg.SAFE_N).clamp(min=0.0)
    return torch.square(excess / 10.0) * state.fixture_engaged


def ins_tool_speed(state: TaskState) -> torch.Tensor:
    """Squared speed of the tool while the goal is held, m^2/s^2 -- once the task layer has started, only across its
    axis. Every policy this task starts from jitters its arm at ~10 Hz by a few millimetres while holding still
    (0.23-0.44 m/s median tool speed, 2026-09-27), which no loop on its average can remove and a 1 mm slot cannot take
    (F-104 saw the same habit on a pad). Along the axis the key has to move: priced there too (v6-v9), the term taught
    v9 to stop at a 12 N detent it had pushed through 800 iterations earlier."""
    v = state.tool_vel_w
    along = (v * state.fixture_axis_w).sum(-1, keepdim=True) * state.fixture_axis_w
    v = torch.where(state.fixture_engaged.unsqueeze(-1) > 0.5, v - along, v)
    return torch.sum(torch.square(v), dim=-1) * state.ins_holding


def ins_damage(state: TaskState) -> torch.Tensor:
    """1.0 on the step the contact force passed `DAMAGE_N`."""
    return state.mech_torn


TERMS = {
    "ins_progress": ins_progress,
    "ins_reach": ins_reach,
    "ins_seated": ins_seated,
    "ins_lateral": ins_lateral,
    "ins_overforce": ins_overforce,
    "ins_damage": ins_damage,
    "ins_tool_speed": ins_tool_speed,
    "fixture_force_tracking": hook_rewards.fixture_force_tracking,
    "arm_torque_margin": hook_rewards.arm_torque_margin,
}

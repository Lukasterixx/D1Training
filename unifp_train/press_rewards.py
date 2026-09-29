"""Reward terms of the pressing task (`press_env.py`), weighted in `press_cfg.WEIGHTS`."""
from __future__ import annotations

import torch

from . import hook_rewards, press_cfg
from .rewards import TaskState


def press_progress(state: TaskState) -> torch.Tensor:
    """The tip's depth against the task layer's reference while it presses: a sharp exponential and a broad one."""
    error = state.press_error.abs()
    reward = 0.5 * torch.exp(-error / press_cfg.PROGRESS_SIGMA_FINE_M) \
        + 0.5 * torch.exp(-error / press_cfg.PROGRESS_SIGMA_COARSE_M)
    return reward * state.press_pressing


def press_reach(state: TaskState) -> torch.Tensor:
    """The tool tip's precision: to its goal before the layer starts, across the axis after (the insertion's
    `ins_reach`, for the same reason: UniFP's own position term cannot tell a tip on the cap from one beside it)."""
    error = state.press_reach_error
    reward = 0.5 * torch.exp(-error / press_cfg.REACH_SIGMA_FINE_M) \
        + 0.5 * torch.exp(-error / press_cfg.REACH_SIGMA_COARSE_M)
    return reward * state.press_active


def press_pressed(state: TaskState) -> torch.Tensor:
    """1.0 while the cap is at its full travel."""
    return state.press_pressed


def press_lateral(state: TaskState) -> torch.Tensor:
    """The force on the tool across the estimated axis, squared per (`LATERAL_SCALE_N`)^2."""
    return torch.square(state.press_lateral_n / press_cfg.LATERAL_SCALE_N) * state.fixture_engaged


def press_overforce(state: TaskState) -> torch.Tensor:
    """Contact force beyond `SAFE_N`, squared per (10 N)^2."""
    excess = (state.press_contact_n - press_cfg.SAFE_N).clamp(min=0.0)
    return torch.square(excess / 10.0) * state.fixture_engaged


def press_tool_speed(state: TaskState) -> torch.Tensor:
    """Squared speed of the tool while its goal is held -- once the layer has started, only across its axis, along
    which the tool has to move (`insert_rewards.ins_tool_speed`, and F-104: a pad on a button needs a still tool)."""
    v = state.tool_vel_w
    along = (v * state.fixture_axis_w).sum(-1, keepdim=True) * state.fixture_axis_w
    v = torch.where(state.fixture_engaged.unsqueeze(-1) > 0.5, v - along, v)
    return torch.sum(torch.square(v), dim=-1) * state.press_holding


def press_damage(state: TaskState) -> torch.Tensor:
    """1.0 on the step the contact force passed `DAMAGE_N` or the tool went through the panel."""
    return state.mech_torn


TERMS = {
    "press_progress": press_progress,
    "press_reach": press_reach,
    "press_pressed": press_pressed,
    "press_lateral": press_lateral,
    "press_overforce": press_overforce,
    "press_damage": press_damage,
    "press_tool_speed": press_tool_speed,
    "fixture_force_tracking": hook_rewards.fixture_force_tracking,
    "arm_torque_margin": hook_rewards.arm_torque_margin,
}

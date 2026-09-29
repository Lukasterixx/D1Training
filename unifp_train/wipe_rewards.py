"""Reward terms of the wiping task (`wipe_env.py`), weighted in `wipe_cfg.WEIGHTS`."""
from __future__ import annotations

import torch

from . import hook_rewards, wipe_cfg
from .rewards import TaskState


def wipe_track(state: TaskState) -> torch.Tensor:
    """The pad's precision: to its pre-contact goal before the layer starts, in the plane on the path after (the
    insertion's `ins_reach`, which UniFP's `exp(-2 error)` cannot replace at centimetres)."""
    error = state.wipe_track_error
    reward = 0.5 * torch.exp(-error / wipe_cfg.TRACK_SIGMA_FINE_M) + 0.5 * torch.exp(-error / wipe_cfg.TRACK_SIGMA_COARSE_M)
    return reward * state.wipe_active


def wipe_contact(state: TaskState) -> torch.Tensor:
    """1.0 while the pad presses the surface as it wipes: the plan's contact-during-motion criterion."""
    return state.wipe_contact


def wipe_overforce(state: TaskState) -> torch.Tensor:
    """Normal force beyond `SAFE_N`, squared per (10 N)^2."""
    excess = (state.wipe_normal_n - wipe_cfg.SAFE_N).clamp(min=0.0)
    return torch.square(excess / 10.0) * state.fixture_engaged


def wipe_tool_speed(state: TaskState) -> torch.Tensor:
    """The pad's squared velocity error while its goal is held, m^2/s^2: against standing still before contact, against
    the path's reference velocity while wiping. A jittering hand bounces a pad off the surface (F-104, F-119)."""
    return torch.sum(torch.square(state.wipe_vel_error_w), dim=-1) * state.wipe_holding


def wipe_damage(state: TaskState) -> torch.Tensor:
    """1.0 on the step the normal force passed `DAMAGE_N` or the pad went through the panel."""
    return state.mech_torn


TERMS = {
    "wipe_track": wipe_track,
    "wipe_contact": wipe_contact,
    "wipe_overforce": wipe_overforce,
    "wipe_damage": wipe_damage,
    "wipe_tool_speed": wipe_tool_speed,
    "fixture_force_tracking": hook_rewards.fixture_force_tracking,
    "arm_torque_margin": hook_rewards.arm_torque_margin,
}

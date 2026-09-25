"""The force-transmission task's own reward terms, on top of UniFP's 27.

Kept out of `rewards.py` for the reason `task_cfg.EXTENSION_WEIGHTS` gives: that module's `TERMS` is
the port's fidelity record against a recorded Isaac Gym rollout, and a new task's terms are not
part of what "reproduces upstream" means. Every term here reads the `fixture_*` fields of
`rewards.TaskState`, which only `hook_env` fills in.

UniFP's own end-effector term keeps working unchanged: its target is the goal displaced by
(measured + commanded force) / k, and with the goal on the fixture's anchor and the measured force
being the fixture's reaction, that target sits on the anchor exactly when the robot applies the
commanded force. So it already pays for the force, softly; `fixture_force_tracking` pays for it
directly, at the resolution of a few newtons.
"""
from __future__ import annotations

import torch

from unifp_isaaclab import interface

from . import hook_cfg
from .rewards import NUM_LEG_JOINTS, TaskState


def force_error(applied: torch.Tensor, command: torch.Tensor, axis: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """(|error along the axis|, |force across it|), newtons."""
    along = (applied * axis).sum(-1)
    across = (applied - along.unsqueeze(-1) * axis).norm(dim=-1)
    return (along - (command * axis).sum(-1)).abs(), across


def fixture_force_tracking(state: TaskState) -> torch.Tensor:
    """How close the force applied along the fixture's axis is to the one commanded, while engaged.

    The error is the along-axis miss plus `FORCE_LATERAL_WEIGHT` of whatever goes across the axis.
    Two exponentials: the fine one only pays in the last few newtons, the coarse one still slopes
    30 N away, where a policy starting from a bent arm will be.
    """
    along, across = force_error(state.fixture_applied_w, state.fixture_command_w, state.fixture_axis_w)
    error = along + hook_cfg.FORCE_LATERAL_WEIGHT * across
    reward = 0.5 * torch.exp(-error / hook_cfg.FORCE_SIGMA_FINE_N) \
        + 0.5 * torch.exp(-error / hook_cfg.FORCE_SIGMA_COARSE_N)
    return reward * state.fixture_engaged


def arm_torque_ratio(state: TaskState) -> torch.Tensor:
    """|torque| / effort limit for the six arm joints, (N, 6)."""
    limits = torch.as_tensor(interface.TORQUE_LIMITS[NUM_LEG_JOINTS:interface.NUM_ACTIONS],
                             dtype=state.torques.dtype, device=state.torques.device)
    return state.torques[:, NUM_LEG_JOINTS:interface.NUM_ACTIONS].abs() / limits


def arm_torque_margin(state: TaskState) -> torch.Tensor:
    """Squared excess of each arm joint's load over `ARM_TORQUE_MARGIN` of its limit, while engaged.

    A saturated joint scores (1 - 0.7)^2 = 0.09. Charged only on a fixture: in free space the arm
    saturating is how it moves fast, and UniFP's own `torque_limits` term already prices that.
    """
    excess = (arm_torque_ratio(state) - hook_cfg.ARM_TORQUE_MARGIN).clamp(min=0.0)
    return torch.sum(torch.square(excess), dim=1) * state.fixture_engaged


def fixture_lost(state: TaskState) -> torch.Tensor:
    """1.0 on the step the claw came off the handle or the pad off the button."""
    return state.fixture_lost


def fixture_seat(state: TaskState) -> torch.Tensor:
    """Squared nearness to losing the contact: 0 seated, 1 on the edge of lifting or sliding off.

    Not gated on `fixture_engaged`: a pad detached from its button (training only) scores the full 1,
    and a robot with no fixture scores 0 because its risk is 0.
    """
    return torch.square(state.fixture_risk)


def fixture_tool_speed(state: TaskState) -> torch.Tensor:
    """Squared speed of the tool while it is on a fixture (or a pad is off its button), m^2/s^2."""
    return torch.sum(torch.square(state.tool_vel_w), dim=-1) * state.fixture_active


TERMS = {
    "fixture_force_tracking": fixture_force_tracking,
    "arm_torque_margin": arm_torque_margin,
    "fixture_lost": fixture_lost,
    "fixture_seat": fixture_seat,
    "fixture_tool_speed": fixture_tool_speed,
}

"""The rope-pull task's reward terms, on top of UniFP's 27 (`rope_cfg.WEIGHTS` has what each is for).

Every term reads fields of `rewards.TaskState` that only `rope_env` fills in:

  * `fixture_engaged` -- the rope held **and its anchor intact** (1.0) -- gates the force objective;
  * `rope_held` -- the rope held, released or not -- gates the arm's margin and the tool's speed;
  * `fixture_applied_w`, `fixture_command_w`, `fixture_axis_w` -- as the hook task's, the axis being the commanded
    direction, so the hook task's force arithmetic (`hook_rewards.force_error`) scores the pull;
  * `rope_tv_n` -- the tension's total variation over the step's physics substeps, against the command's, N;
  * `rope_wobble_n2` -- the mean over those substeps of the force error's squared deviation from its running mean, N^2;
  * `rope_slack_fraction` -- the share of those substeps the rope was slack with a pull commanded;
  * `rope_recovering` -- 1.0 inside `RECOVERY_WINDOW_S` of a release;
  * `base_lin_vel_w`, `rope_fallen`;
  * `rope_arm_line_w` (shoulder to jaw centre) and `rope_line_w` (jaw centre toward the anchor).
"""
from __future__ import annotations

import torch

from . import hook_cfg, hook_rewards, rope_cfg
from .rewards import TaskState


def arm_torque_margin(state: TaskState) -> torch.Tensor:
    """The hook task's margin term, charged while the rope is held at all."""
    excess = (hook_rewards.arm_torque_ratio(state) - hook_cfg.ARM_TORQUE_MARGIN).clamp(min=0.0)
    return torch.sum(torch.square(excess), dim=1) * state.rope_held


def fixture_tool_speed(state: TaskState) -> torch.Tensor:
    return torch.sum(torch.square(state.tool_vel_w), dim=-1) * state.rope_held


def rope_force_tracking(state: TaskState) -> torch.Tensor:
    """The hook task's force term with a third, broad exponential: along-axis miss + 1/4 of the force across it."""
    along, across = hook_rewards.force_error(state.fixture_applied_w, state.fixture_command_w, state.fixture_axis_w)
    error = along + hook_cfg.FORCE_LATERAL_WEIGHT * across
    fine, coarse, broad = rope_cfg.FORCE_SHARES
    reward = (fine * torch.exp(-error / rope_cfg.FORCE_SIGMA_FINE_N) + coarse * torch.exp(-error / rope_cfg.FORCE_SIGMA_COARSE_N)
              + broad * torch.exp(-error / rope_cfg.FORCE_SIGMA_BROAD_N))
    return reward * state.fixture_engaged


def rope_ripple(state: TaskState) -> torch.Tensor:
    """Squared, capped total variation of the tension error over the step, in `RIPPLE_SCALE_N` units."""
    scaled = torch.square(state.rope_tv_n / rope_cfg.RIPPLE_SCALE_N).clamp(max=rope_cfg.RIPPLE_CAP)
    return scaled * state.fixture_engaged


def rope_wobble(state: TaskState) -> torch.Tensor:
    """Squared deviation of the force error from its running mean, over the step, in `WOBBLE_SCALE_N` units, capped."""
    scaled = (state.rope_wobble_n2 / rope_cfg.WOBBLE_SCALE_N ** 2).clamp(max=rope_cfg.RIPPLE_CAP)
    return scaled * state.fixture_engaged


def rope_slack(state: TaskState) -> torch.Tensor:
    return state.rope_slack_fraction * state.fixture_engaged


def rope_upset(state: TaskState) -> torch.Tensor:
    """Horizontal base speed squared plus a quarter of the roll/pitch rates squared, while recovering."""
    speed = torch.sum(torch.square(state.base_lin_vel_w[:, :2]), dim=-1)
    rates = torch.sum(torch.square(state.base_ang_vel_b[:, :2]), dim=-1)
    return (speed + 0.25 * rates) * state.rope_recovering


def rope_fallen(state: TaskState) -> torch.Tensor:
    return state.rope_fallen


def rope_taut_arm(state: TaskState) -> torch.Tensor:
    """How nearly the arm's line (shoulder to jaw centre) lies along the rope's (jaw centre to anchor)."""
    arm, line = state.rope_arm_line_w, state.rope_line_w
    cos = (arm * line).sum(-1) / (arm.norm(dim=-1) * line.norm(dim=-1)).clamp(min=1e-9)
    angle = torch.acos(cos.clamp(-1.0, 1.0))
    pulling = state.fixture_command_w.norm(dim=-1) >= rope_cfg.TAUT_ARM_MIN_COMMAND_N
    return torch.exp(-angle / rope_cfg.TAUT_ARM_SIGMA_RAD) * state.fixture_engaged * pulling


TERMS = {
    "fixture_force_tracking": rope_force_tracking,
    "arm_torque_margin": arm_torque_margin,
    "fixture_tool_speed": fixture_tool_speed,
    "rope_ripple": rope_ripple,
    "rope_slack": rope_slack,
    "rope_upset": rope_upset,
    "rope_fallen": rope_fallen,
    "rope_taut_arm": rope_taut_arm,
    "rope_wobble": rope_wobble,
}

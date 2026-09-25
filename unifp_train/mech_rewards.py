"""The goal-commanded task's own reward terms, on top of UniFP's and the arm-margin term.

Kept out of `rewards.py` for the reason `task_cfg.EXTENSION_WEIGHTS` gives. Every term reads the
`mech_*` fields of `rewards.TaskState`, which only `mech_env` fills in, and is zero wherever the
robot is not holding a handle.

UniFP's own end-effector term keeps running, with its target made rigid (`mech_cfg.STIFF_KP`): the
goal is the reference point on the path, and the tool is asked to be on it. That term is broad --
`exp(-2 x error)`, 0.96 at 2 cm -- so `mech_progress` is what resolves the last centimetres.
"""
from __future__ import annotations

import torch

from . import hook_rewards, mech_cfg
from .rewards import TaskState


def mech_progress(state: TaskState) -> torch.Tensor:
    """How close the handle is to its reference along the path, while held.

    Two exponentials, as `hook_rewards.fixture_force_tracking` has: the fine one pays in the last
    couple of centimetres, which is all a button's travel is; the coarse one still slopes at a
    drawer's full 20 cm.
    """
    return _progress(state.mech_error, state.mech_travel) * state.fixture_engaged


def _progress(error: torch.Tensor, travel: torch.Tensor | None = None) -> torch.Tensor:
    """0.5 exp(-|e|/fine) + 0.5 exp(-|e|/coarse); for a short mechanism both widths shrink with its travel.

    v3: at the fixed 2 and 8 cm, a 1.5 cm button or a 3 cm latch that has not moved at all still scores
    0.65 or 0.45 -- so `mech_push`, which pays in proportion to what is lost, paid little for pushing
    on exactly the mechanisms with the stiffest latches, and v2 hovered at their thresholds (a 10 N latch
    driven with 9.7-10 N). The widths are capped at `PROGRESS_FINE_PER_TRAVEL` and
    `PROGRESS_COARSE_PER_TRAVEL` of the travel, which leaves a drawer's or a door's unchanged.
    """
    error = error.abs()
    fine = torch.full_like(error, mech_cfg.PROGRESS_SIGMA_FINE_M)
    coarse = torch.full_like(error, mech_cfg.PROGRESS_SIGMA_COARSE_M)
    if travel is not None and mech_cfg.PROGRESS_SCALES_WITH_TRAVEL:
        span = travel.clamp(min=mech_cfg.PATH_MIN_TRAVEL_M)
        fine = torch.minimum(fine, mech_cfg.PROGRESS_FINE_PER_TRAVEL * span)
        coarse = torch.minimum(coarse, mech_cfg.PROGRESS_COARSE_PER_TRAVEL * span)
    return 0.5 * torch.exp(-error / fine) + 0.5 * torch.exp(-error / coarse)


def mech_push(state: TaskState) -> torch.Tensor:
    """Newtons driven toward the reference, per `PUSH_SCALE_N`, times how far from it the handle is.

    The dense half of "push until it moves" (v2). `mech_progress` pays nothing for pushing harder on a
    mechanism that has not yet given: the reward is flat until it breaks free, and a policy that makes
    30 N never finds that 40 N would open it (v1 stalled there). This pays for the force, in the
    direction of the reference, scaled by ``1 - progress`` so it vanishes on the reference. With
    `mech_progress` at 4.0 and this at 2.0 per 50 N, getting closer is worth more than pushing from
    behind for any drive under 100 N (the cap), so sitting short to collect it never pays.
    """
    toward = (torch.sign(-state.mech_error) * state.mech_drive).clamp(0.0, mech_cfg.PUSH_CAP_N)
    return toward / mech_cfg.PUSH_SCALE_N * (1.0 - _progress(state.mech_error, state.mech_travel)) * state.fixture_engaged


def mech_torn(state: TaskState) -> torch.Tensor:
    """1.0 on the step the handle was torn out of the claw."""
    return state.mech_torn


def mech_overspeed(state: TaskState) -> torch.Tensor:
    """Squared handle speed above the limit, (m/s)^2, while held: the lunge when a latch gives."""
    excess = (state.mech_speed.abs() - state.mech_speed_limit).clamp(min=0.0)
    return torch.square(excess) * state.fixture_engaged


def mech_overforce(state: TaskState) -> torch.Tensor:
    """Squared drive beyond the commanded force plus `OVERFORCE_MARGIN_N`, per (10 N)^2, while held.

    The hierarchical variant's force limit (law v2): with the progress reward in the loop, law v1 learned to push
    past its command when a mechanism would not move -- 113-134 N against an 80 N cap (F-108) -- so the cap was not
    a limit. This makes the command one: the task layer sets the most it will ask for, and the policy is charged
    for going past it.
    """
    excess = (state.mech_drive.abs() - state.mech_command.abs() - mech_cfg.OVERFORCE_MARGIN_N).clamp(min=0.0)
    return torch.square(excess / 10.0) * state.fixture_engaged


def mech_effort(state: TaskState) -> torch.Tensor:
    """The grasp force squared, in units of `EFFORT_SCALE_N`, while held."""
    return torch.square(state.mech_force / mech_cfg.EFFORT_SCALE_N) * state.fixture_engaged


TERMS = {
    "mech_progress": mech_progress,
    "mech_push": mech_push,
    "arm_torque_margin": hook_rewards.arm_torque_margin,
    "mech_torn": mech_torn,
    "mech_overspeed": mech_overspeed,
    "mech_effort": mech_effort,
    "mech_overforce": mech_overforce,
}

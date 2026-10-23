"""The front-right-leg task's reward terms (`leg_env.py`).

Where a term is UniFP's it is called, not copied (`rewards.py`): the foot's tracking is UniFP's end-effector term with
the foot as the end effector, and the effort and smoothness terms are UniFP's as they stand (they read the twelve leg
joints). What is new is what standing on three legs needs -- the support feet down and still, the lifted foot up -- and
the handle's outcome terms. The position and force variants read the same terms; they differ in what the state carries
(`ee_force_measured` and `ee_force_cmd` are zero for the position variant), which is what makes the target the bare
goal for one and `goal + (external + command) / k` for the other.

Every term reads `rewards.TaskState` plus the attributes `leg_env` sets on it: `goal_down` (the foot's goal is on the
ground), `soft_limits` (N-independent, (20, 2)), `support_contact_forces`, `support_feet_vel_w`, `handle_*`.
"""
from __future__ import annotations

import torch

from unifp_isaaclab import interface

from . import leg_cfg, rewards


def foot_tracking(state: rewards.TaskState) -> torch.Tensor:
    """UniFP's end-effector term on the foot: `exp(-2 |foot - target|_1)`, the target displaced by any force."""
    return rewards.tracking_ee_force_world(state)


def foot_tracking_fine(state: rewards.TaskState) -> torch.Tensor:
    """The same target at the scale of a door handle's bar: `exp(-|e|_2 / 4 cm)`."""
    error = (state.ee_pos_w - rewards.ee_target(state)).norm(dim=-1)
    return torch.exp(-error / leg_cfg.FOOT_FINE_SIGMA_M)


def foot_tracking_mid(state: rewards.TaskState) -> torch.Tensor:
    """The same target at 12 cm: `exp(-|e|_2 / 12 cm)`, the gradient between UniFP's broad term and the fine one."""
    error = (state.ee_pos_w - rewards.ee_target(state)).norm(dim=-1)
    return torch.exp(-error / leg_cfg.FOOT_MID_SIGMA_M)


def alive(state: rewards.TaskState) -> torch.Tensor:
    return rewards.alive(state)


def support_contact(state: rewards.TaskState) -> torch.Tensor:
    """+1 for each support foot down (-1 up); the lifted foot +1 clear of the ground (-1 not) unless its goal is down.

    "Clear" is geometric since v3 (2026-10-20): the foot sphere's centre `FOOT_CLEAR_M` up and under 1 N of contact.
    In v1 and v2 it was "under the 5 N contact threshold", and both runs learnt to rest the foot on the floor unloaded,
    the base leaning back 15 degrees: the reward for lifting, without lifting.
    """
    contact = state.contact_mask
    support = contact[:, list(leg_cfg.SUPPORT_FEET)]
    support_score = torch.where(support > 0.5, 1.0, -1.0).sum(dim=1)
    fr_down = contact[:, leg_cfg.FR_FOOT_INDEX] > 0.5
    fr_clear = (state.ee_pos_w[:, 2] > leg_cfg.FOOT_CLEAR_M) & \
        (state.feet_contact_forces[:, leg_cfg.FR_FOOT_INDEX].norm(dim=-1) < 1.0)
    fr_score = torch.where(state.goal_down, torch.where(fr_down, 1.0, -0.3), torch.where(fr_clear, 1.0, -1.0))
    return (support_score + fr_score) / 4.0


def foot_lift(state: rewards.TaskState) -> torch.Tensor:
    """While the foot's goal is up: how far up toward the goal's height the foot is, 0..1 (since v3)."""
    floor = 0.022
    goal_h = (state.ee_goal_w[:, 2] - floor).clamp(min=1e-3)
    lift = ((state.ee_pos_w[:, 2] - floor).clamp(min=0.0) / goal_h).clamp(max=1.0)
    return torch.where(state.goal_down, torch.zeros_like(lift), lift)


def base_still(state: rewards.TaskState) -> torch.Tensor:
    error = torch.sum(torch.square(state.base_lin_vel_b[:, :2]), dim=1)
    return torch.exp(-error / leg_cfg.TRACKING_SIGMA)


def base_yaw_still(state: rewards.TaskState) -> torch.Tensor:
    return torch.exp(-torch.square(state.base_ang_vel_b[:, 2]) / leg_cfg.TRACKING_SIGMA)


def orientation(state: rewards.TaskState) -> torch.Tensor:
    return torch.sum(torch.square(state.projected_gravity_b[:, :2]), dim=1)


def tilt_excess(state: rewards.TaskState) -> torch.Tensor:
    """The base's tilt from upright past `leg_cfg.TILT_EXCESS_RAD`, rad: a hinge where `orientation`'s square is soft."""
    tilt = torch.acos((-state.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
    return (tilt - leg_cfg.TILT_EXCESS_RAD).clamp(min=0.0)


def base_height(state: rewards.TaskState) -> torch.Tensor:
    return torch.square(state.root_pos_w[:, 2] - leg_cfg.BASE_HEIGHT_TARGET)


def dof_pos_limits(state: rewards.TaskState) -> torch.Tensor:
    """UniFP's soft-limit penalty on the legs, with the front-right leg's limits at 0.95 of its range."""
    soft = state.soft_limits
    below = -(state.dof_pos - soft[:, 0]).clip(max=0.0)
    above = (state.dof_pos - soft[:, 1]).clip(min=0.0)
    return torch.sum((below + above)[:, :12], dim=1)


def support_slip(state: rewards.TaskState) -> torch.Tensor:
    """Horizontal speed of each support foot while it is down, summed."""
    speed = state.support_feet_vel_w[:, :, :2].norm(dim=-1)
    down = state.contact_mask[:, list(leg_cfg.SUPPORT_FEET)]
    return (speed * down).sum(dim=1)


def support_stance(state: rewards.TaskState) -> torch.Tensor:
    """Mean over the support feet of exp(-(d / sigma)^2), d each foot's horizontal distance, in the base's yaw frame,
    from where it stands when the robot stands settled (`leg_cfg.NOMINAL_FEET_B`). The key task's (2026-10-21): the
    handle policies stood on three legs with the right-rear foot 13-20 cm forward of its place, put there while the
    robot dropped from its spawn, and nothing asked for it back."""
    rel = state.feet_pos_w - state.root_pos_w.unsqueeze(1)
    nominal = torch.as_tensor(leg_cfg.NOMINAL_FEET_B, dtype=rel.dtype, device=rel.device)
    error = []
    for i in leg_cfg.SUPPORT_FEET:
        local = interface.quat_rotate_inverse(state.base_yaw_quat, rel[:, i])
        error.append((local[:, :2] - nominal[i]).norm(dim=-1))
    d = torch.stack(error, dim=1)
    return torch.exp(-torch.square(d / leg_cfg.STANCE_SIGMA_M)).mean(dim=1)


def stance_error(feet_pos: torch.Tensor, root_pos: torch.Tensor, yaw_quat: torch.Tensor) -> torch.Tensor:
    """Each foot's horizontal distance from its settled place (base yaw frame), m, (N, 4)."""
    rel = feet_pos - root_pos.unsqueeze(1)
    nominal = torch.as_tensor(leg_cfg.NOMINAL_FEET_B, dtype=rel.dtype, device=rel.device)
    return torch.stack([(interface.quat_rotate_inverse(yaw_quat, rel[:, i])[:, :2] - nominal[i]).norm(dim=-1)
                        for i in range(4)], dim=1)


def support_hip_pos(state: rewards.TaskState) -> torch.Tensor:
    index = list(leg_cfg.SUPPORT_HIP_INDICES)
    default = torch.as_tensor(interface.DEFAULT_DOF_POS, dtype=state.dof_pos.dtype, device=state.dof_pos.device)
    return torch.sum(torch.square(state.dof_pos[:, index] - default[index]), dim=1)


def feet_contact_forces(state: rewards.TaskState) -> torch.Tensor:
    magnitude = torch.norm(state.feet_contact_forces, dim=-1)
    return torch.sum((magnitude - leg_cfg.MAX_CONTACT_FORCE).clip(min=0.0), dim=1)


# --- the handle (both variants) ----------------------------------------------------------------------------------

def handle_contact(state: rewards.TaskState) -> torch.Tensor:
    """The foot on the bar (overlapping it) while the layer presses or holds: a contact flag, not a force."""
    return state.handle_on_bar * state.handle_pushing


def handle_progress(state: rewards.TaskState) -> torch.Tensor:
    """How far the lever is down toward the latch, 0..1, while the layer presses or holds."""
    return (state.handle_theta / state.handle_latch.clamp(min=1e-3)).clamp(0.0, 1.0) * state.handle_pushing


def handle_open(state: rewards.TaskState) -> torch.Tensor:
    return state.handle_open * state.handle_pushing


def handle_slip(state: rewards.TaskState) -> torch.Tensor:
    return state.handle_slip


def approach_contact(state: rewards.TaskState) -> torch.Tensor:
    """How far (cm) the foot or the knee is pressed into the bar or the door before the press has begun."""
    return state.handle_approach_cm


def door_push(state: rewards.TaskState) -> torch.Tensor:
    """How far (cm) the foot and the knee are pressed into the door's face."""
    return state.handle_door_cm


def press_excess(state: rewards.TaskState) -> torch.Tensor:
    """How far (cm) the peak contact goes past the press limit (`Go2D1LegEnvCfg.press_limit_n`), through the plant's
    stiffness: a force limit stated as geometry, as `door_push` is."""
    return state.handle_press_excess_cm


TERMS = {
    "foot_tracking": foot_tracking,
    "foot_tracking_mid": foot_tracking_mid,
    "foot_tracking_fine": foot_tracking_fine,
    "alive": alive,
    "support_contact": support_contact,
    "foot_lift": foot_lift,
    "base_still": base_still,
    "base_yaw_still": base_yaw_still,
    "lin_vel_z": rewards.lin_vel_z,
    "ang_vel_xy": rewards.ang_vel_xy,
    "orientation": orientation,
    "tilt_excess": tilt_excess,
    "base_height": base_height,
    "collision": rewards.collision,
    "dof_pos_limits": dof_pos_limits,
    "torque_limits": rewards.torque_limits,
    "torques": rewards.torques,
    "dof_vel": rewards.dof_vel,
    "dof_acc": rewards.dof_acc,
    "action_rate": rewards.action_rate,
    "support_slip": support_slip,
    "support_hip_pos": support_hip_pos,
    "support_stance": support_stance,
    "feet_contact_forces": feet_contact_forces,
    "dof_vel_arm": rewards.dof_vel_arm,
    "handle_contact": handle_contact,
    "handle_progress": handle_progress,
    "handle_open": handle_open,
    "handle_slip": handle_slip,
    "door_push": door_push,
    "approach_contact": approach_contact,
    "press_excess": press_excess,
}

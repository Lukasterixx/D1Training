"""The 153-wide privileged observation the critic reads, and the frame stacks around both obs.

The actor's 76-wide observation is `unifp_isaaclab.interface.single_obs` — the same function the
playback port uses, so a policy trained here and a policy trained upstream see the identical
contract by construction. This module adds the half that only training needs: what the *critic*
is allowed to know.

The critic's extra knowledge is the point of the architecture. It sees the base's linear velocity,
where the tool actually is, the forces actually acting, the randomised masses and friction, and
the per-actuator strength -- none of which the actor gets. The actor has to infer them from 32
frames of history, and the adaptation module is trained to do exactly that.

Layout confirmed block by block against a recorded Isaac Gym rollout (every block exact); the
offsets below are not a reading of the source but a measurement of it.

Three blocks are **all zeros in playback and in this task's training**: `mass_params` is zero
unless base/leg/gripper mass randomisation is on, the 17-wide leg-mass slice inside it is zero
regardless because `randomize_leg_mass` is off upstream, and `motor_strength - 1` is zero unless
motor randomisation is on. They are kept at full width anyway -- the critic's input dimension is
part of the contract, and silently narrowing it would make checkpoints incompatible.
"""
from __future__ import annotations

import math

import torch

from unifp_isaaclab import interface

#: Widths, in order. Sums to 153.
PRIVILEGED_LAYOUT = (
    ("base_lin_vel", 3),        # x2.0, body frame -- the actor never sees this
    ("ee_pos_sphere", 3),       # where the tool tip actually is, goal-sphere coordinates
    ("ee_force", 3),            # x0.01, measured external force at the tool, yaw frame
    ("base_force", 3),          # x0.01, measured external force at the base, yaw frame
    ("leg_ref_diff", 12),       # legs against the gait reference
    ("mass_params", 22),        # added base mass, 17 leg masses, gripper mass, CoM offset
    ("friction", 1),
    ("motor_strength", 18),     # per-actuator scale, minus 1
    ("stance_mask", 4),
    ("contact_mask", 4),
    ("projected_gravity", 3),
    ("base_ang_vel", 3),        # x0.25
    ("dof_pos", 18),            # minus default
    ("dof_vel", 18),            # x0.05
    ("actions", 18),
    ("gait_phase", 2),          # sin, cos
    ("commands", 15),           # x COMMANDS_SCALE
    ("ee_goal_offset_sphere", 3),
)

NUM_PRIVILEGED_OBS = sum(width for _, width in PRIVILEGED_LAYOUT)   # 153
CRITIC_FRAME_STACK = 3
NUM_CRITIC_OBS = NUM_PRIVILEGED_OBS * CRITIC_FRAME_STACK            # 459

#: Width of the adaptation module's supervised target: the first four blocks of the privileged
#: observation, unstacked. Upstream builds it as a separate `obs_pred` tensor from the same four
#: expressions, so slicing the privileged observation is the same thing computed once instead of
#: twice -- and the slice is checked against the recorded `obs_pred` in `tests/test_unifp_train.py`.
#: These four are what the actor has to infer from history: how fast the base is moving, where the
#: tool actually is, and the two external forces.
NUM_ESTIMATES = sum(width for name, width in PRIVILEGED_LAYOUT[:4])  # 12

#: Per-axis scales for the two spherical blocks, matching the command scales for radius/pitch/yaw.
EE_SPHERE_SCALE = (0.5, 1.0, 1.3)

OBS_SCALE_EE_FORCE = 0.01
OBS_SCALE_BASE_FORCE = 0.01


def offsets() -> dict[str, tuple[int, int]]:
    """Slice bounds of each block, for anyone reading a recorded critic observation."""
    out, start = {}, 0
    for name, width in PRIVILEGED_LAYOUT:
        out[name] = (start, start + width)
        start += width
    return out


def _sphere_about(point_w: torch.Tensor, centre_w: torch.Tensor,
                  base_yaw_quat: torch.Tensor) -> torch.Tensor:
    """(radius, pitch, yaw) of a world point about the goal-sphere centre, in the yaw-only frame."""
    local = interface.quat_rotate_inverse(base_yaw_quat, point_w - centre_w)
    scale = torch.as_tensor(EE_SPHERE_SCALE, dtype=local.dtype, device=local.device)
    return interface.cart2sphere(local) * scale


def privileged_obs(
    base_lin_vel_b: torch.Tensor,
    ee_pos_w: torch.Tensor,
    ee_goal_w: torch.Tensor,
    goal_centre_w: torch.Tensor,
    base_yaw_quat: torch.Tensor,
    ee_force_w: torch.Tensor,
    base_force_w: torch.Tensor,
    gripper_force_kp: torch.Tensor,
    ee_force_cmd: torch.Tensor,
    leg_ref_diff: torch.Tensor,
    mass_params: torch.Tensor,
    friction: torch.Tensor,
    motor_strength: torch.Tensor,
    stance_mask: torch.Tensor,
    contact_mask: torch.Tensor,
    projected_gravity: torch.Tensor,
    base_ang_vel_b: torch.Tensor,
    dof_pos: torch.Tensor,
    dof_vel: torch.Tensor,
    actions: torch.Tensor,
    gait_phase: torch.Tensor,
    commands: torch.Tensor,
) -> torch.Tensor:
    """Build the 153-wide privileged observation.

    `dof_pos` and `dof_vel` are the full 20 in UniFP's DOF order; the jaws are dropped here.
    `commands` is the raw 15, scaled here. `motor_strength` is the raw per-actuator scale --
    the *minus one* is applied here, as upstream applies it.
    """
    default = torch.as_tensor(interface.DEFAULT_DOF_POS, dtype=dof_pos.dtype, device=dof_pos.device)
    command_scale = torch.as_tensor(interface.COMMANDS_SCALE, dtype=commands.dtype,
                                    device=commands.device)

    # Both forces are expressed in the yaw-only base frame, which is what makes them comparable
    # with a force *command*: the command is given in that frame too.
    ee_force_local = interface.quat_rotate_inverse(base_yaw_quat, ee_force_w)
    base_force_local = interface.quat_rotate_inverse(base_yaw_quat, base_force_w)

    # The goal the end-effector reward actually tracks: displaced from the commanded goal by
    # whatever force is acting, through the virtual stiffness. See rewards.tracking_ee_force_world.
    offset = ee_force_w + interface.quat_apply(base_yaw_quat, ee_force_cmd)
    displaced_goal = offset / gripper_force_kp + ee_goal_w

    return torch.cat((
        base_lin_vel_b * interface.COMMANDS_SCALE[0],
        _sphere_about(ee_pos_w, goal_centre_w, base_yaw_quat),
        ee_force_local * OBS_SCALE_EE_FORCE,
        base_force_local * OBS_SCALE_BASE_FORCE,
        leg_ref_diff,
        mass_params,
        friction,
        motor_strength[:, :interface.NUM_ACTIONS] - 1.0,
        stance_mask,
        contact_mask,
        projected_gravity,
        base_ang_vel_b * interface.OBS_SCALE_ANG_VEL,
        (dof_pos - default)[:, :interface.NUM_ACTIONS] * interface.OBS_SCALE_DOF_POS,
        dof_vel[:, :interface.NUM_ACTIONS] * interface.OBS_SCALE_DOF_VEL,
        actions,
        torch.sin(2 * math.pi * gait_phase).unsqueeze(-1),
        torch.cos(2 * math.pi * gait_phase).unsqueeze(-1),
        commands * command_scale,
        _sphere_about(displaced_goal, goal_centre_w, base_yaw_quat),
    ), dim=-1)

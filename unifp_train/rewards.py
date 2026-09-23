"""UniFP's 27 reward terms, ported for Isaac Lab and checked against the environment that trained.

Each function here is one `_reward_<name>` from the Isaac Gym environment, taking an explicit
`TaskState` instead of reading `self`. That shape is deliberate: it lets
`tests/test_unifp_train.py` build a state out of a recorded Isaac Gym rollout and compare every
term against the value that environment actually accumulated, which is the only way 27 terms get
ported without a sign or an index error hiding inside a weighted sum.

Two conventions worth stating once:

  * **Terms are unscaled.** The weight and the `dt` live in `task_cfg.scaled_weights()`.
  * **Every quantity is post-step**, as `compute_reward` sees it — the state at the *end* of the
    step being rewarded, after the four physics substeps. `last_*` fields are the previous step's.

Slices follow UniFP's DOF order throughout: `[:12]` is the legs, `[12:18]` the arm, `18:` the
held jaws. Several terms are named for the whole robot and act on the legs only
(`torques`, `dof_vel`, `dof_acc`, `action_rate`); that is upstream's behaviour, not a transcription
slip, and the arm has its own `_arm` variants.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import torch

from unifp_isaaclab import interface

from . import task_cfg

NUM_LEG_JOINTS = 12


@dataclass
class TaskState:
    """Everything the reward terms read, at the end of the step being rewarded."""

    base_lin_vel_b: torch.Tensor        # (N, 3) body frame
    base_ang_vel_b: torch.Tensor        # (N, 3) body frame
    root_pos_w: torch.Tensor            # (N, 3) world
    dof_pos: torch.Tensor               # (N, 20) UniFP order
    dof_vel: torch.Tensor               # (N, 20)
    last_dof_vel: torch.Tensor          # (N, 20) previous step
    torques: torch.Tensor               # (N, 20) applied joint efforts
    actions: torch.Tensor               # (N, 18)
    last_actions: torch.Tensor          # (N, 18) previous step
    commands: torch.Tensor              # (N, 15) raw, unscaled
    gait_phase: torch.Tensor            # (N,)
    ref_leg_pos: torch.Tensor           # (N, 12) from gait.reference_leg_pos
    stance_mask: torch.Tensor           # (N, 4) FL, FR, RL, RR
    contact_mask: torch.Tensor          # (N, 4) measured contact, same order
    feet_pos_w: torch.Tensor            # (N, 4, 3) world
    feet_contact_forces: torch.Tensor   # (N, 4, 3) world
    feet_air_time: torch.Tensor         # (N, 4) seconds since last touchdown
    penalised_contact_forces: torch.Tensor  # (N, K, 3) thighs, calves, trunk
    ee_pos_w: torch.Tensor              # (N, 3) tool tip, world
    ee_goal_w: torch.Tensor             # (N, 3) commanded goal, world
    thigh_pos_w: torch.Tensor           # (N, 4, 3) world, same leg order as the feet
    feet_vel_w: torch.Tensor            # (N, 4, 3) world linear velocity
    last_contacts: torch.Tensor         # (N, 4) bool, previous step's measured contact
    base_yaw_quat: torch.Tensor         # (N, 4) w-first, yaw only
    ee_force_measured: torch.Tensor     # (N, 3) external force at the tool, world
    ee_force_cmd: torch.Tensor          # (N, 3) commanded tool force, yaw frame
    base_force_measured: torch.Tensor   # (N, 3) external force at the base, world
    base_force_cmd: torch.Tensor        # (N, 3) commanded base force, yaw frame
    gripper_force_kp: torch.Tensor      # (N, 3) virtual stiffness, N/m
    base_force_kd: torch.Tensor         # (N, 3) virtual damping for the base
    dt: float = interface.POLICY_DT


def _walking(state: TaskState) -> torch.Tensor:
    """True where any velocity command is outside its dead zone -- upstream's `get_walking_cmd_mask`."""
    return (
        (state.commands[:, interface.CMD_LIN_VEL_X].abs() > interface.LIN_VEL_X_CLIP)
        | (state.commands[:, interface.CMD_LIN_VEL_Y].abs() > interface.LIN_VEL_Y_CLIP)
        | (state.commands[:, interface.CMD_ANG_VEL_YAW].abs() > interface.ANG_VEL_YAW_CLIP)
    )


# --- staying alive and upright -----------------------------------------------------------------

def alive(state: TaskState) -> torch.Tensor:
    return torch.ones(state.dof_pos.shape[0], device=state.dof_pos.device)


def lin_vel_z(state: TaskState) -> torch.Tensor:
    return torch.square(state.base_lin_vel_b[:, 2])


def ang_vel_xy(state: TaskState) -> torch.Tensor:
    return torch.sum(torch.square(state.base_ang_vel_b[:, :2]), dim=1)


def base_height(state: TaskState) -> torch.Tensor:
    """Squared error to the target height, against **world z**.

    Upstream reads `root_states[:, 2]` directly rather than a terrain-relative height, even though
    the config asks for a height scan. On flat ground the two agree; on its own rough terrain they
    do not, and the reward is the world-frame one.
    """
    return torch.square(state.root_pos_w[:, 2] - task_cfg.BASE_HEIGHT_TARGET)


def collision(state: TaskState) -> torch.Tensor:
    return torch.sum(
        1.0 * (torch.norm(state.penalised_contact_forces, dim=-1) > 0.1), dim=1)


# --- tracking the commands ----------------------------------------------------------------------

def tracking_ang_vel(state: TaskState) -> torch.Tensor:
    error = torch.square(state.commands[:, interface.CMD_ANG_VEL_YAW] - state.base_ang_vel_b[:, 2])
    return torch.exp(-error / task_cfg.TRACKING_SIGMA)


def ref_dof_leg(state: TaskState) -> torch.Tensor:
    """How closely the legs hold the gait's reference trot posture."""
    error = torch.sum(torch.abs(state.dof_pos[:, :NUM_LEG_JOINTS] - state.ref_leg_pos), dim=1)
    return torch.exp(-error * 0.1)


def stand_still(state: TaskState) -> torch.Tensor:
    """Rewards holding the default leg pose, but **only while told to stand**."""
    error = torch.sum(
        torch.abs(state.dof_pos - torch.as_tensor(
            interface.DEFAULT_DOF_POS, dtype=state.dof_pos.dtype,
            device=state.dof_pos.device))[:, :NUM_LEG_JOINTS], dim=1)
    reward = torch.exp(-error * 0.05)
    return torch.where(_walking(state), torch.zeros_like(reward), reward)


# --- effort and smoothness ------------------------------------------------------------------------

def torques(state: TaskState) -> torch.Tensor:
    return torch.sum(torch.square(state.torques)[:, :NUM_LEG_JOINTS], dim=1)


def torque_limits(state: TaskState) -> torch.Tensor:
    limits = torch.as_tensor(interface.TORQUE_LIMITS, dtype=state.torques.dtype,
                             device=state.torques.device)
    return torch.sum(
        (state.torques.abs() - limits * task_cfg.SOFT_TORQUE_LIMIT).clip(min=0.0), dim=1)


def dof_vel(state: TaskState) -> torch.Tensor:
    return torch.sum(torch.square(state.dof_vel)[:, :NUM_LEG_JOINTS], dim=1)


def dof_vel_arm(state: TaskState) -> torch.Tensor:
    return torch.sum(torch.square(state.dof_vel)[:, NUM_LEG_JOINTS:interface.NUM_ACTIONS], dim=1)


def dof_acc(state: TaskState) -> torch.Tensor:
    change = (state.last_dof_vel - state.dof_vel)[:, :NUM_LEG_JOINTS] / state.dt
    return torch.sum(torch.square(change), dim=1)


def dof_acc_arm(state: TaskState) -> torch.Tensor:
    change = (state.last_dof_vel - state.dof_vel)[:, NUM_LEG_JOINTS:interface.NUM_ACTIONS] / state.dt
    return torch.sum(torch.square(change), dim=1)


def action_rate(state: TaskState) -> torch.Tensor:
    return torch.sum(torch.square(state.last_actions - state.actions)[:, :NUM_LEG_JOINTS], dim=1)


def action_rate_arm(state: TaskState) -> torch.Tensor:
    return torch.sum(
        torch.square(state.last_actions - state.actions)[:, NUM_LEG_JOINTS:interface.NUM_ACTIONS],
        dim=1)


def dof_pos_limits(state: TaskState) -> torch.Tensor:
    """How far the actuated joints have pushed past their **soft** limits."""
    soft = torch.as_tensor(task_cfg.soft_joint_pos_limits(), dtype=state.dof_pos.dtype,
                           device=state.dof_pos.device)
    below = -(state.dof_pos - soft[:, 0]).clip(max=0.0)
    above = (state.dof_pos - soft[:, 1]).clip(min=0.0)
    return torch.sum((below + above)[:, :interface.NUM_ACTIONS], dim=1)


def hip_pos(state: TaskState) -> torch.Tensor:
    """Keeps the hips near their default -- what stops the robot splaying sideways."""
    index = list(task_cfg.HIP_INDICES)
    default = torch.as_tensor(interface.DEFAULT_DOF_POS, dtype=state.dof_pos.dtype,
                              device=state.dof_pos.device)
    return torch.sum(torch.square(state.dof_pos[:, index] - default[index]), dim=1)


# --- the two objectives ---------------------------------------------------------------------------

def tracking_ee_force_world(state: TaskState) -> torch.Tensor:
    """The end-effector term, and the one that makes this a *unified* position/force task.

    The target is not the goal but `goal + (measured force + commanded force) / k`: with no force
    it is plain tool-tip position tracking, and with a force command it asks the arm to reach
    *past* the goal by however far a spring of stiffness `k` would be compressed by that force.
    Which is how a position-controlled arm is asked for a force without a force sensor in the loop.

    `k` is `gripper_force_kps`, randomised per episode in training (the config's range is
    degenerate at 200 N/m, so it is effectively constant).
    """
    error = torch.sum(torch.abs(state.ee_pos_w - ee_target(state)), dim=1)
    return torch.exp(-error / task_cfg.TRACKING_EE_SIGMA * 2)


def ee_target(state: TaskState) -> torch.Tensor:
    """Where the tool tip is actually asked to be: the goal, displaced by the force term.

    Split out of `tracking_ee_force_world` so the viewer overlay can draw *this* rather than a
    second copy of the same expression. UniFP draws it as a magenta sphere, and a magenta sphere
    that has quietly drifted from the reward it claims to show is worse than no sphere at all.
    """
    offset = state.ee_force_measured + interface.quat_apply(state.base_yaw_quat, state.ee_force_cmd)
    return offset / state.gripper_force_kp + state.ee_goal_w


def tracking_lin_vel_force_world(state: TaskState) -> torch.Tensor:
    """The same construction for the base: velocity tracking, displaced by any base force.

    The `non_stop_sign` mask is upstream's and is not cosmetic -- when every command is inside its
    dead zone the *whole* offset velocity target is zeroed, so a base force cannot smuggle in a
    walk command while the robot is being told to stand still.
    """
    local = interface.quat_rotate_inverse(state.base_yaw_quat, state.base_force_measured)
    offset = local + state.base_force_cmd
    target = (offset / state.base_force_kd)[:, :2] + state.commands[:, :2]
    moving = (
        (target[:, 0].abs() > interface.LIN_VEL_X_CLIP)
        | (target[:, 1].abs() > interface.LIN_VEL_Y_CLIP)
        | (state.commands[:, interface.CMD_ANG_VEL_YAW].abs() > interface.ANG_VEL_YAW_CLIP)
    )
    target = target * moving.unsqueeze(1)
    error = torch.sum(torch.square(target - state.base_lin_vel_b[:, :2]), dim=1)
    return torch.exp(-error / task_cfg.TRACKING_SIGMA)


# --- the feet -------------------------------------------------------------------------------------

def feet_contact_number(state: TaskState) -> torch.Tensor:
    """+1 per foot whose measured contact agrees with the gait, -0.3 per foot that disagrees."""
    agrees = state.contact_mask == state.stance_mask
    return torch.mean(torch.where(agrees, 1.0, -0.3), dim=1)


def feet_air_time(state: TaskState) -> torch.Tensor:
    """Rewards long steps, paid once per touchdown.

    **This term has side effects**, as upstream's does: it advances `feet_air_time` and rewrites
    `last_contacts` on the `TaskState` in place. It must be called exactly once per step, and the
    caller must carry those two buffers across steps. Calling it twice per step silently changes
    the gait being rewarded -- which is why the recorded reference records episode-sum deltas
    rather than re-calling the reward functions.
    """
    contact = state.feet_contact_forces[:, :, 2] > 1.0
    filtered = torch.logical_or(contact, state.last_contacts)
    state.last_contacts = contact
    first_contact = (state.feet_air_time > 0.0) * filtered
    state.feet_air_time += state.dt
    reward = torch.sum((state.feet_air_time - 0.5) * first_contact, dim=1)
    reward = reward * _walking(state)
    state.feet_air_time = state.feet_air_time * (~filtered)
    return reward


def feet_height(state: TaskState) -> torch.Tensor:
    """Penalises the **front** feet never leaving the ground, and only while walking.

    Upstream takes `feet_indices[:2]`, which is FL and FR. Clamped at most 0, so it is a penalty
    that vanishes once the higher front foot passes 10 cm.
    """
    highest = torch.max(state.feet_pos_w[:, :2, 2], dim=-1)[0]
    reward = torch.clamp(highest - 0.10, max=0.0)
    return torch.where(_walking(state), reward, torch.zeros_like(reward))


def feet_height_high(state: TaskState) -> torch.Tensor:
    """Penalises lifting any foot above 20 cm, while walking."""
    highest = torch.max(state.feet_pos_w[:, :, 2], dim=-1)[0]
    reward = torch.clamp(highest - 0.20, min=0.0)
    return torch.where(_walking(state), reward, torch.zeros_like(reward))


def feet_pos_xy(state: TaskState) -> torch.Tensor:
    """Mean horizontal distance from each foot to its own thigh -- keeps the stance tucked in."""
    difference = torch.norm(state.feet_pos_w[:, :, :2] - state.thigh_pos_w[:, :, :2], dim=2)
    return torch.mean(difference, dim=1)


def feet_drag(state: TaskState) -> torch.Tensor:
    """Penalises moving a foot while it is loaded: contact force times foot speed, summed."""
    speed = torch.abs(state.feet_vel_w).sum(dim=-1)
    force = torch.norm(state.feet_contact_forces, dim=-1)
    return (force * speed).sum(dim=-1)


def feet_contact_forces(state: TaskState) -> torch.Tensor:
    """Penalises foot contact forces above `MAX_CONTACT_FORCE`."""
    magnitude = torch.norm(state.feet_contact_forces, dim=-1)
    return torch.sum((magnitude - task_cfg.MAX_CONTACT_FORCE).clip(min=0.0), dim=1)


#: Every term the task uses, by the name its weight is keyed under.
TERMS = {
    name: globals()[name] for name in task_cfg.REWARD_WEIGHTS
}

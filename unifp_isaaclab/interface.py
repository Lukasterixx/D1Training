"""The policy interface of the UniFP Go2+D1 task, written down so a second simulator can meet it.

`model_48800.pt` was trained in Isaac Gym against a specific contract: a 76-wide observation in a
specific order with specific scales, stacked 32 deep, and 18 actions read as joint-position offsets
in Isaac Gym's DOF order. None of that is recoverable from the weights. This module is that
contract, and nothing else -- no Isaac Lab imports, so it can be checked against the recorded
Isaac Gym interface (`unifp_go2d1/dump_interface.py`) with no simulator running on either side.

Every constant here was **read out of the running Isaac Gym environment**, not copied from the
config, because several of them are derived: the torque limits come from the URDF rather than the
config, the DOF order is Isaac Gym's own asset ordering, and `p_gains`/`d_gains` are the config's
name-keyed dicts already resolved per joint. `tests/test_unifp_interface.py` re-checks them against
a recorded rollout.

Two things a reader should not have to discover the hard way:

  * **The DOF order is not Isaac Lab's.** Isaac Gym orders the 20 joints leg-by-leg
    (FL hip/thigh/calf, FR..., RL..., RR...) then the arm; Isaac Lab groups by joint level
    (all four hips, then all four thighs, ...). `permutation_from()` builds the gather index,
    and it is the single most dangerous thing to get wrong here: a wrong permutation gives a
    policy that limps rather than one that errors.
  * **The arm joints are renamed.** UniFP's asset calls them `d1_Joint1..6` purely to force Isaac
    Gym's DOF sort (see `unifp_go2d1/README.md`); the Isaac Lab model calls them `Joint1..6`.
    `ISAACLAB_NAMES` is the translation.
"""
from __future__ import annotations

import math

import torch

# --- DOF layout, in Isaac Gym's order. 12 legs, 6 arm joints, 2 held jaws. ---------------------

DOF_NAMES = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "d1_Joint1", "d1_Joint2", "d1_Joint3", "d1_Joint4", "d1_Joint5", "d1_Joint6",
    "d1_Joint7_1", "d1_Joint7_2",
)

NUM_DOF = 20
NUM_ACTIONS = 18          # the first 18 of DOF_NAMES; the two jaws are held, not commanded
NUM_GRIPPER_JOINTS = 2

#: UniFP's `d1_` prefix exists only to control Isaac Gym's DOF sort. Isaac Lab's model uses the
#: D1 URDF's own names.
ISAACLAB_NAMES = tuple(n[3:] if n.startswith("d1_") else n for n in DOF_NAMES)

# --- Control. Read from the environment, not the config: the torque limits are the URDF's. -----

DEFAULT_DOF_POS = (
    0.1, 0.8, -1.5,
    -0.1, 0.8, -1.5,
    0.1, 1.0, -1.5,
    -0.1, 1.0, -1.5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0,
)

#: Stiffness for the 18 actioned joints (N·m/rad; N/m for the prismatic jaws below).
P_GAINS = (25.0,) * 12 + (60.0, 60.0, 40.0, 40.0, 40.0, 40.0)
D_GAINS = (0.5,) * 12 + (1.0, 1.0, 0.8, 0.8, 0.8, 0.8)

#: The two jaws are not actions. UniFP holds them at their default with a separate PD.
GRIPPER_STIFFNESS = 800.0
GRIPPER_DAMPING = 8.0

#: Per-DOF effort limit, from the URDF. PhysX clips at this, so stiffness buys tracking, never
#: strength. Note the calf's 45.43 N·m -- it is geared, and is not the hip/thigh's 23.7.
TORQUE_LIMITS = (
    23.7, 23.7, 45.43,
    23.7, 23.7, 45.43,
    23.7, 23.7, 45.43,
    23.7, 23.7, 45.43,
    3.3, 3.3, 1.7, 1.7, 1.7, 1.7,
    15.0, 15.0,
)

ACTION_SCALE = 0.25
DECIMATION = 4
SIM_DT = 0.005
POLICY_DT = SIM_DT * DECIMATION      # 0.02 s, 50 Hz
CYCLE_TIME = 0.64                    # gait period, seconds

# --- Observation. 76 wide, stacked 32 deep into the 2432 the adaptation encoder reads. ---------

NUM_SINGLE_OBS = 76
FRAME_STACK = 32
NUM_OBS = NUM_SINGLE_OBS * FRAME_STACK
NUM_LATENT = 64
CLIP_OBSERVATIONS = 100.0
CLIP_ACTIONS = 100.0

OBS_SCALE_ANG_VEL = 0.25
OBS_SCALE_DOF_POS = 1.0
OBS_SCALE_DOF_VEL = 0.05

#: Slice bounds of the single observation, for anyone reading a trace.
OBS_SLICES = {
    "body_orientation": (0, 2),      # roll, pitch (rad)
    "base_ang_vel": (2, 5),          # body frame, x0.25
    "dof_pos": (5, 23),              # minus default, 18 joints
    "dof_vel": (23, 41),             # x0.05, 18 joints
    "actions": (41, 59),             # previous actions, raw
    "gait_phase": (59, 61),          # sin, cos
    "commands": (61, 76),            # x COMMANDS_SCALE
}

# --- Commands. 15 wide. -------------------------------------------------------------------------

#: Index names for the command vector, as UniFP's environment numbers them.
CMD_LIN_VEL_X, CMD_LIN_VEL_Y, CMD_ANG_VEL_YAW = 0, 1, 2
CMD_EE_RADIUS, CMD_EE_PITCH, CMD_EE_YAW = 3, 4, 5
CMD_EE_ORN_R, CMD_EE_ORN_P, CMD_EE_ORN_Y = 6, 7, 8      # never written by the task; always 0
CMD_EE_FORCE = slice(9, 12)
CMD_BASE_FORCE = slice(12, 15)
NUM_COMMANDS = 15

COMMANDS_SCALE = (
    2.0, 2.0, 0.25,          # lin vel x, y; ang vel yaw
    0.5, 1.0, 1.3,           # ee goal radius, pitch, yaw
    0.5, 0.5, 0.5,           # ee orientation (unused by this task)
    0.01, 0.01, 0.01,        # ee force command, N
    0.01, 0.01, 0.01,        # base force command, N
)

#: Below these the command counts as "not walking" and the gait phase is pinned to 0.
LIN_VEL_X_CLIP = 0.1
LIN_VEL_Y_CLIP = 0.1
ANG_VEL_YAW_CLIP = 0.2

# --- End-effector goal. ---------------------------------------------------------------------------

#: The goal sphere's centre: the robot's x,y at *ground level* (z is dropped, not taken from the
#: base), yaw-rotated by this offset. So the centre is 0.49 m above the ground under the robot --
#: roughly the shoulder height of the D1 when the Go2 stands. Deliberately z-invariant: the goal
#: does not rise and fall with the base.
EE_GOAL_CENTER_OFFSET = (0.0, 0.0, 0.49)

EE_GOAL_RADIUS_RANGE = (0.30, 0.58)
EE_GOAL_PITCH_RANGE = (-math.pi / 4, math.pi / 3)
EE_GOAL_YAW_RANGE = (-2 * math.pi / 5, 2 * math.pi / 5)
EE_GOAL_INIT_START = (0.50, math.pi / 5, 0.0)
EE_GOAL_INIT_END = (0.50, 0.0, 0.0)
EE_GOAL_TRAJ_TIME_S = (1.0, 3.0)
EE_GOAL_HOLD_TIME_S = (0.5, 2.0)
EE_GOAL_COLLISION_UPPER = (0.17, 0.14, -0.10)
EE_GOAL_COLLISION_LOWER = (-0.48, -0.14, -0.55)
EE_GOAL_UNDERGROUND_LIMIT = -0.40
EE_GOAL_COLLISION_SAMPLES = 10

#: The controlled point: the CAD pincer tip. Same body and offset as the Isaac Lab position-only
#: task (`position_only/tool_point.py`), which is where UniFP's `ee_gripper_link` was placed.
#: CAD, not measured on the arm (F-013).
TOOL_BODY = "Link7_1"
TOOL_OFFSET_M = (0.0547, 0.0060, 0.0170)

#: Peak end-effector force the task commands, N. Upstream's B2Z1 uses ±60; the D1 cannot make or
#: resist that (see unifp_go2d1/README.md), so force tracking here is a few-newton problem.
EE_FORCE_RANGE_N = (-8.0, 8.0)


def permutation_from(names: list[str]) -> list[int]:
    """Gather index that reorders `names` into UniFP's DOF order.

    `joint_pos[:, permutation_from(robot.joint_names)]` is the vector UniFP's observation expects.
    Raises rather than silently dropping a joint, because a quietly wrong permutation is the one
    failure mode of this port that produces plausible-looking motion.
    """
    index = {name: i for i, name in enumerate(names)}
    missing = [n for n in ISAACLAB_NAMES if n not in index]
    if missing:
        raise KeyError(f"articulation is missing {missing}; it has {sorted(names)}")
    if len(names) != NUM_DOF:
        raise ValueError(f"expected {NUM_DOF} joints, got {len(names)}: {names}")
    return [index[name] for name in ISAACLAB_NAMES]


def body_roll_pitch(quat_wxyz: torch.Tensor) -> torch.Tensor:
    """Roll and pitch of the base, as UniFP's `euler_from_quat` computes them.

    Isaac Lab stores quaternions w-first, Isaac Gym w-last; this takes Isaac Lab's order. The
    pitch is a clipped asin, so it saturates at ±pi/2 rather than wrapping -- which matters only
    when the robot is on its side, but that is exactly when a fall is being recorded.
    """
    w, x, y, z = quat_wxyz.unbind(-1)
    roll = torch.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = torch.asin(torch.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return torch.stack((roll, pitch), dim=-1)


def yaw_from_quat(quat_wxyz: torch.Tensor) -> torch.Tensor:
    """Base yaw, for the yaw-only frame the goal sphere and the force commands live in."""
    w, x, y, z = quat_wxyz.unbind(-1)
    return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quat(yaw: torch.Tensor) -> torch.Tensor:
    """A w-first quaternion for a rotation about z only."""
    half = 0.5 * yaw
    zero = torch.zeros_like(yaw)
    return torch.stack((torch.cos(half), zero, zero, torch.sin(half)), dim=-1)


def quat_apply(quat_wxyz: torch.Tensor, vec: torch.Tensor) -> torch.Tensor:
    """Rotate `vec` by `quat_wxyz` (w-first)."""
    w = quat_wxyz[..., 0:1]
    xyz = quat_wxyz[..., 1:4]
    t = 2.0 * torch.cross(xyz, vec, dim=-1)
    return vec + w * t + torch.cross(xyz, t, dim=-1)


def quat_rotate_inverse(quat_wxyz: torch.Tensor, vec: torch.Tensor) -> torch.Tensor:
    """Rotate `vec` into the frame `quat_wxyz` describes."""
    conj = quat_wxyz.clone()
    conj[..., 1:4] = -conj[..., 1:4]
    return quat_apply(conj, vec)


def sphere2cart(sphere: torch.Tensor) -> torch.Tensor:
    """(radius, pitch, yaw) -> (x, y, z), UniFP's convention: pitch is elevation above the x-y plane."""
    radius, pitch, yaw = sphere.unbind(-1)
    return torch.stack((
        radius * torch.cos(pitch) * torch.cos(yaw),
        radius * torch.cos(pitch) * torch.sin(yaw),
        radius * torch.sin(pitch),
    ), dim=-1)


def cart2sphere(cart: torch.Tensor) -> torch.Tensor:
    """(x, y, z) -> (radius, pitch, yaw)."""
    radius = torch.norm(cart, dim=-1)
    pitch = torch.asin(cart[..., 2] / radius)
    yaw = torch.atan2(cart[..., 1], cart[..., 0])
    return torch.stack((radius, pitch, yaw), dim=-1)


def single_obs(
    quat_wxyz: torch.Tensor,
    base_ang_vel_b: torch.Tensor,
    dof_pos: torch.Tensor,
    dof_vel: torch.Tensor,
    prev_actions: torch.Tensor,
    gait_phase: torch.Tensor,
    commands: torch.Tensor,
) -> torch.Tensor:
    """Build UniFP's 76-wide single observation.

    `dof_pos` and `dof_vel` are the full 20 in UniFP's DOF order; the two jaws are dropped here,
    as upstream drops them. `commands` is the raw 15, unscaled -- the scaling happens here, so a
    caller never has to remember which entries are newtons and which are radians.
    """
    default = torch.as_tensor(DEFAULT_DOF_POS, dtype=dof_pos.dtype, device=dof_pos.device)
    scale = torch.as_tensor(COMMANDS_SCALE, dtype=commands.dtype, device=commands.device)
    return torch.cat((
        body_roll_pitch(quat_wxyz),
        base_ang_vel_b * OBS_SCALE_ANG_VEL,
        (dof_pos - default)[:, :NUM_ACTIONS] * OBS_SCALE_DOF_POS,
        dof_vel[:, :NUM_ACTIONS] * OBS_SCALE_DOF_VEL,
        prev_actions,
        torch.sin(2 * math.pi * gait_phase).unsqueeze(-1),
        torch.cos(2 * math.pi * gait_phase).unsqueeze(-1),
        commands * scale,
    ), dim=-1)


def joint_targets(actions: torch.Tensor) -> torch.Tensor:
    """Joint position targets for the 18 actioned joints, as UniFP's torque law reads them.

    UniFP never forms a target explicitly -- it goes straight to
    `kp * (action*scale + default - q) - kd * qd`. Isaac Lab's PD actuator takes a target, so the
    same quantity is written out here: `action * ACTION_SCALE + default`. Identical arithmetic,
    provided the actuator's gains and effort limits are UniFP's (they are, in `robot.py`) and the
    motor-strength randomisation is off (it is, in playback).
    """
    default = torch.as_tensor(DEFAULT_DOF_POS[:NUM_ACTIONS], dtype=actions.dtype, device=actions.device)
    return actions * ACTION_SCALE + default


def gait_step(phase: torch.Tensor, commands: torch.Tensor) -> torch.Tensor:
    """Advance the gait phase one policy step, pinning it to 0 while the robot is told to stand.

    UniFP advances the phase *before* the observation that uses it, and zeroes it whenever every
    velocity command is inside its dead zone. The zeroing is not cosmetic: it is how the policy is
    told to stop stepping, and a phase that keeps running while the command is zero makes the
    robot march on the spot.
    """
    walking = (
        (commands[:, CMD_LIN_VEL_X].abs() > LIN_VEL_X_CLIP)
        | (commands[:, CMD_LIN_VEL_Y].abs() > LIN_VEL_Y_CLIP)
        | (commands[:, CMD_ANG_VEL_YAW].abs() > ANG_VEL_YAW_CLIP)
    )
    phase = torch.remainder(phase + POLICY_DT / CYCLE_TIME, 1.0)
    return torch.where(walking, phase, torch.zeros_like(phase))

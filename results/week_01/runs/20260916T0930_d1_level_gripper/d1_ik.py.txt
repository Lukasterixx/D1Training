"""Damped least-squares inverse kinematics for the D1 arm, in plain numpy.

This is the solver half of Cartesian control. It has no Isaac, torch or DDS
dependency, so the same code runs in the task, in a test on the system Python,
and on the Go2's Jetson beside the real arm -- which is the point: one kinematic
model, not one per place it is used.

Kinematics come from `position_only.workspace`, which parses `d1_arm/d1.urdf`.
`forward()` already returns each arm joint's axis and origin in the Go2 base
frame, so the geometric Jacobian below is exact rather than a finite difference.
The controlled point defaults to the task's: the Link7_1 pincer tip (F-013).

Frames. Everything here is the **Go2 base frame**, the same frame the task uses,
with the arm mounted at `workspace.MOUNT_B`. On a sitting robot that frame is
still the base's, not the world's.

Angles. Radians and URDF joint order (`Joint1..Joint6`) throughout. The arm's
wire protocol is degrees indexed by servo id 0..5; `to_servo_deg` and
`from_servo_deg` convert, and are the only place that mapping is written down.

**Unvalidated:** that servo *i* reports URDF `Joint{i+1}`'s angle with the same
zero and sign. The URDF limits match the driver's table joint for joint, which
is consistent with it, but the arm's measured resting pose sits outside both
(F-023), so the zero point is not established. Cartesian accuracy here is only
as good as that assumption and has never been checked against a measurement of
the real arm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from position_only.tool_point import TOOL_BODY, TOOL_OFFSET_M
from position_only.workspace import (
    ARM_JOINTS,
    SOFT_LIMIT_FACTOR,
    clear_of_body,
    forward,
    load_urdf,
    tool_position,
)

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

# Servo id on the wire -> URDF joint name. Servo 6 is the gripper and is not an
# IK degree of freedom.
SERVO_TO_JOINT = {i: name for i, name in enumerate(ARM_JOINTS)}

# Sign of each servo's angle relative to its URDF joint. **Every entry measured**
# on 2026-09-16 by commanding one joint at a time and watching which way it went
# (F-030, F-034). Two are inverted; the rest match the URDF.
#
#   J0  INVERTED  a +y (left) target swung the arm right; a +30 deg command
#                 swivels it right, which the corrected model now predicts.
#   J1  matches   the arm rose and reached forward over the head as predicted,
#   J2  matches   which also rules out a 180 deg mount yaw (that flips x too).
#   J3  INVERTED  +30 deg was predicted to roll the forearm left-side-UP; it
#                 rolled left-side-DOWN.
#   J4  matches   +30 deg pitched the wrist down, tip dropping ~106 mm.
#   J5  matches   +30 deg rolled the wrist left-side-down, as predicted.
#
# The J3/J5 pair is the reason this needed measuring rather than assuming: the
# model had them counter-rotating, and on hardware they roll the same way.
SERVO_SIGN = np.array([-1.0, 1.0, 1.0, -1.0, 1.0, 1.0])


# The controlled tool's own axes, in `TOOL_BODY`'s link frame, derived from the
# URDF: the fingers point along the approach axis and separate along the jaw axis.
# `position_only.workspace.tip_offsets` gives the approach direction in Link6; the
# two finger joints sit at -/+y there, which fixes the jaw axis.
TOOL_APPROACH_LOCAL = np.array([1.0, 0.0051, 0.0])
TOOL_APPROACH_LOCAL /= np.linalg.norm(TOOL_APPROACH_LOCAL)
TOOL_JAW_LOCAL = np.array([0.0, -0.0006, 1.0])
TOOL_JAW_LOCAL -= TOOL_APPROACH_LOCAL * (TOOL_APPROACH_LOCAL @ TOOL_JAW_LOCAL)
TOOL_JAW_LOCAL /= np.linalg.norm(TOOL_JAW_LOCAL)
TOOL_UP_LOCAL = np.cross(TOOL_APPROACH_LOCAL, TOOL_JAW_LOCAL)
_TOOL_TRIAD = np.column_stack([TOOL_APPROACH_LOCAL, TOOL_JAW_LOCAL, TOOL_UP_LOCAL])

WORLD_UP = np.array([0.0, 0.0, 1.0])


def tool_axes(rot):
    """(approach, jaw, up) of the tool in the base frame, for rotations (N, 3, 3)."""
    rot = np.asarray(rot, dtype=float).reshape(-1, 3, 3)
    return (rot @ TOOL_APPROACH_LOCAL, rot @ TOOL_JAW_LOCAL, rot @ TOOL_UP_LOCAL)


def tool_attitude_deg(rot):
    """(elevation, roll) of the tool in degrees, for rotations (N, 3, 3).

    Elevation is the approach axis above horizontal; roll is rotation about that
    axis away from level. Both zero means the gripper is level.
    """
    approach, _, up = tool_axes(rot)
    elevation = np.degrees(np.arcsin(np.clip(approach[:, 2], -1.0, 1.0)))
    # "Level up" is world up with the approach component removed; roll is the
    # signed angle from it to the tool's own up, measured about the approach axis.
    level_up = WORLD_UP - approach * (approach @ WORLD_UP)[:, None]
    norm = np.linalg.norm(level_up, axis=1, keepdims=True)
    level_up = np.divide(level_up, norm, out=np.zeros_like(level_up), where=norm > 1e-9)
    sin = np.einsum("ni,ni->n", np.cross(level_up, up), approach)
    cos = np.einsum("ni,ni->n", level_up, up)
    return elevation, np.degrees(np.arctan2(sin, cos))


def level_rotation(rot_current, fallback_heading=None):
    """The nearest level attitude to `rot_current`, keeping its heading.

    Level means the approach axis lies in the horizontal plane and the tool is not
    rolled about it. Heading -- which way it points in that plane -- is deliberately
    left at whatever the arm already has, so this constrains 2 rotational degrees of
    freedom rather than 3 and leaves the solver a redundancy to work with. A fully
    specified orientation would make most of the workspace unreachable.
    """
    rot_current = np.asarray(rot_current, dtype=float).reshape(-1, 3, 3)
    approach, _, _ = tool_axes(rot_current)
    heading = approach.copy()
    heading[:, 2] = 0.0
    norm = np.linalg.norm(heading, axis=1, keepdims=True)
    # Straight up or down: the heading is undefined, so take one from the caller
    # (the direction out from the arm base is the sensible default).
    degenerate = (norm < 1e-6).ravel()
    if degenerate.any():
        default = np.array([1.0, 0.0, 0.0]) if fallback_heading is None else np.asarray(fallback_heading, float)
        default = default.copy(); default[2] = 0.0
        default /= max(np.linalg.norm(default), 1e-9)
        heading[degenerate] = default
        norm = np.linalg.norm(heading, axis=1, keepdims=True)
    a_des = heading / norm
    u_des = np.broadcast_to(WORLD_UP, a_des.shape)
    t_des = np.cross(u_des, a_des)
    m_des = np.stack([a_des, t_des, u_des], axis=-1)     # columns
    return m_des @ _TOOL_TRIAD.T


def joint_limits(joints, soft: float = SOFT_LIMIT_FACTOR):
    """(low, high) arrays in radians for the six arm joints.

    `soft` shrinks each range about its midpoint, matching the task's
    `soft_joint_pos_limit_factor`. Pass 1.0 for the raw URDF limits.
    """
    lows, highs = [], []
    for name in ARM_JOINTS:
        low, high = joints[name]["limits"]
        mid, half = (low + high) / 2.0, (high - low) / 2.0 * soft
        lows.append(mid - half)
        highs.append(mid + half)
    return np.array(lows), np.array(highs)


def to_servo_deg(q_rad) -> list[float]:
    """URDF joint angles (radians) -> the six servo angles the arm takes, in degrees.

    Applies `SERVO_SIGN`, so this and `from_servo_deg` are the only two places the
    hardware's sign convention is written down.
    """
    q = np.asarray(q_rad, dtype=float).reshape(-1)[:6]
    return [float(a) for a in q * RAD2DEG * SERVO_SIGN]


def from_servo_deg(angles_deg) -> np.ndarray:
    """The arm's reported servo angles (degrees) -> URDF joint angles (radians), shape (1, 6).

    `SERVO_SIGN` is its own inverse (every entry is +/-1), so this is the exact
    inverse of `to_servo_deg`.
    """
    a = np.asarray(angles_deg, dtype=float).reshape(-1)[:6]
    return (a * SERVO_SIGN * DEG2RAD)[None, :]


def servo_limits_deg(joints, soft: float = SOFT_LIMIT_FACTOR):
    """(low, high) limits expressed in **servo** degrees, signs applied.

    A sign flip swaps a joint's low and high, which matters the moment any joint's
    URDF range stops being symmetric about zero. The D1's are all symmetric today,
    so this is defensive -- but clamping in the wrong space is exactly the kind of
    bug that hides until a limit is asymmetric.
    """
    lows, highs = joint_limits(joints, soft)
    a = lows * RAD2DEG * SERVO_SIGN
    b = highs * RAD2DEG * SERVO_SIGN
    return np.minimum(a, b), np.maximum(a, b)


def jacobian(axes, origins, point):
    """Geometric Jacobian (N, 6, 6) of `point` for revolute joints.

    Rows 0:3 are linear, rows 3:6 angular. `axes` and `origins` (N, 6, 3) come
    straight out of `workspace.forward`, so this needs no differentiation:
    for a revolute joint, moving it sweeps the point about its axis.
    """
    lever = point[:, None, :] - origins            # (N, 6, 3) joint -> point
    linear = np.cross(axes, lever)                 # z_i x (p - o_i)
    return np.concatenate([linear.transpose(0, 2, 1), axes.transpose(0, 2, 1)], axis=1)


def rotation_error(rot_current, rot_target):
    """Angular error vector (N, 3): the axis-angle taking `rot_current` to `rot_target`."""
    err = rot_target @ rot_current.transpose(0, 2, 1)
    # Axis-angle of a rotation matrix, guarded at 0 and pi where the skew part vanishes.
    cos = np.clip((np.trace(err, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos)
    skew = np.stack([err[:, 2, 1] - err[:, 1, 2],
                     err[:, 0, 2] - err[:, 2, 0],
                     err[:, 1, 0] - err[:, 0, 1]], axis=-1)
    small = angle < 1e-8
    near_pi = angle > math.pi - 1e-6
    scale = np.where(small, 0.5, angle / (2.0 * np.sin(np.where(small | near_pi, 1.0, angle))))
    out = skew * scale[:, None]
    if near_pi.any():
        # At pi the skew part is zero; recover the axis from the symmetric part.
        idx = np.flatnonzero(near_pi)
        sym = (err[idx] + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diagonal(sym, axis1=1, axis2=2), 0.0, None))
        axis = axis / np.clip(np.linalg.norm(axis, axis=-1, keepdims=True), 1e-12, None)
        out[idx] = axis * angle[idx, None]
    return out


@dataclass
class IKResult:
    """Outcome of a solve. `q` is always within limits, converged or not."""

    q: np.ndarray            # (6,) radians
    position_error_m: float
    rotation_error_rad: float
    iterations: int
    converged: bool
    clear_of_body: bool      # collision proxy against the Go2 trunk and the ground
    elevation_deg: float = 0.0   # approach axis above horizontal; 0 is level
    roll_deg: float = 0.0        # rotation about the approach axis; 0 is level

    @property
    def servo_deg(self) -> list[float]:
        return to_servo_deg(self.q)


def solve(
    joints,
    target_pos,
    target_rot=None,
    q0=None,
    *,
    level: bool = False,
    body: str = TOOL_BODY,
    offset=TOOL_OFFSET_M,
    damping: float = 0.05,
    max_iterations: int = 200,
    position_tol_m: float = 1e-4,
    rotation_tol_rad: float = 1e-3,
    max_step_rad: float = 0.2,
    soft: float = SOFT_LIMIT_FACTOR,
    base_height: float | None = None,
) -> IKResult:
    """Solve for joint angles putting the controlled point at `target_pos`.

    Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e. The damping is what
    keeps it finite through the shoulder and wrist singularities, at the cost of
    slowing down near them -- the standard trade, and the same `dls` method the
    simulator's controller uses.

    `target_rot` (3, 3) adds orientation to the objective; omit it for
    position-only control, which is what the task needs and what leaves the
    redundancy free. `level=True` instead asks for the gripper to finish level --
    approach axis horizontal, no roll -- while leaving its heading free, so it
    costs two degrees of freedom rather than three. The target attitude is
    re-derived from the current one each iteration, which is what keeps the
    heading unconstrained. Each iteration is clamped to `max_step_rad` and then to the
    joint limits, so the returned `q` is always commandable.

    `base_height` enables the collision proxy: pass the base's height above the
    ground and the result reports whether the arm clears the trunk and floor.
    """
    lows, highs = joint_limits(joints, soft)
    q = np.clip(np.asarray(q0, dtype=float).reshape(1, 6) if q0 is not None
                else ((lows + highs) / 2.0)[None, :], lows, highs)
    if level:
        # Reach the point first, then level from there. Levelling is a much harder
        # objective to satisfy from a distant seed: solving it in one stage lands
        # 24% of sphere targets, staged it lands 51%, and the difference is the
        # solver, not the geometry.
        reach = solve(joints, target_pos, q0=q[0], body=body, offset=offset,
                      damping=damping, max_iterations=max_iterations,
                      position_tol_m=position_tol_m, max_step_rad=max_step_rad, soft=soft)
        if reach.converged:
            q = reach.q[None, :].copy()
    target_pos = np.asarray(target_pos, dtype=float).reshape(1, 3)
    if target_rot is not None:
        if level:
            raise ValueError("pass either target_rot or level=True, not both")
        target_rot = np.asarray(target_rot, dtype=float).reshape(1, 3, 3)

    pos_err = rot_err = float("inf")
    used = 0
    for used in range(1, max_iterations + 1):
        frames, axes, origins = forward(joints, q)
        point = tool_position(frames, body, offset)
        jac = jacobian(axes, origins, point)

        error = target_pos - point
        pos_err = float(np.linalg.norm(error))
        # Re-derived every iteration: the levelled attitude is relative to wherever
        # the tool is now pointing, so the heading follows the solver instead of
        # fighting it.
        want = level_rotation(frames[body][0], fallback_heading=point[0]) if level else target_rot
        if want is None:
            jac, err_vec = jac[:, :3, :], error
            rot_err = 0.0
        else:
            ang = rotation_error(frames[body][0], want)
            rot_err = float(np.linalg.norm(ang))
            err_vec = np.concatenate([error, ang], axis=-1)

        if pos_err < position_tol_m and rot_err < rotation_tol_rad:
            break

        rows = jac.shape[1]
        jjt = jac @ jac.transpose(0, 2, 1) + (damping ** 2) * np.eye(rows)
        dq = np.einsum("nji,njk,nk->ni", jac, np.linalg.inv(jjt), err_vec)

        longest = np.max(np.abs(dq))
        if longest > max_step_rad:
            dq *= max_step_rad / longest
        q = np.clip(q + dq, lows, highs)

    converged = pos_err < position_tol_m and rot_err < rotation_tol_rad
    clear = True
    if base_height is not None:
        clear = bool(clear_of_body(joints, q, base_height)[0])
    frames, _, _ = forward(joints, q)
    elevation, roll = tool_attitude_deg(frames[body][0])
    return IKResult(q[0].copy(), pos_err, rot_err, used, converged, clear,
                    float(elevation[0]), float(roll[0]))


def traversal_configs(q_start, q_goal, samples: int = 41, rate_rad_s: float = 1.22):
    """The configurations the arm actually passes through on its way to `q_goal`.

    Not a straight line in joint space. Every joint slews at roughly the same
    ~1.2 rad/s ceiling (F-033), so the joints with less to do arrive first and
    stop while the rest keep going. The path therefore bends. `rate_rad_s` cancels
    out of the shape -- it only sets which joint finishes when -- so the default is
    the measured value and the result is a plain (samples, 6) array.
    """
    q_start = np.asarray(q_start, dtype=float).reshape(6)
    q_goal = np.asarray(q_goal, dtype=float).reshape(6)
    delta = q_goal - q_start
    duration = float(np.max(np.abs(delta))) / rate_rad_s if rate_rad_s > 0 else 0.0
    if duration <= 0.0:
        return q_start[None, :].copy()
    t = np.linspace(0.0, duration, samples)[:, None]
    travelled = np.minimum(rate_rad_s * t, np.abs(delta))
    return q_start + np.sign(delta) * travelled


def path_clearance(joints, q_start, q_goal, base_height, samples: int = 41):
    """Is the whole traversal clear of the trunk and the ground, not just its end?

    `solve` only ever reports on the configuration it converged to. A move between
    two perfectly clear poses can still sweep the arm straight through the dog:
    sampling random clear pairs, 20 of 600 had a colliding path, the worst spending
    58% of it inside the body proxy. Endpoint clearance is not path clearance.

    Returns (clear, first_bad_fraction) -- the fraction along the path where it
    first fails, so a caller can say where rather than only that.
    """
    configs = traversal_configs(q_start, q_goal, samples)
    ok = clear_of_body(joints, configs, base_height)
    if bool(ok.all()):
        return True, None
    return False, float(np.flatnonzero(~ok)[0]) / max(1, len(ok) - 1)


def tool_pose(joints, q, body: str = TOOL_BODY, offset=TOOL_OFFSET_M):
    """Forward kinematics: (position (3,), rotation (3, 3)) of the controlled point."""
    q = np.asarray(q, dtype=float).reshape(1, 6)
    frames, _, _ = forward(joints, q)
    return tool_position(frames, body, offset)[0], frames[body][0][0]


__all__ = [
    "ARM_JOINTS", "SERVO_TO_JOINT", "TOOL_BODY", "TOOL_OFFSET_M", "IKResult", "joint_limits", "jacobian",
    "rotation_error", "solve", "tool_pose", "to_servo_deg", "from_servo_deg",
    "traversal_configs", "path_clearance", "level_rotation", "tool_axes", "tool_attitude_deg",
    "SERVO_SIGN", "servo_limits_deg",
    "load_urdf", "RAD2DEG", "DEG2RAD",
]

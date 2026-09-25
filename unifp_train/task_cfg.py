"""UniFP's task constants for the Go2+D1, as numbers rather than as a config class hierarchy.

`unifp_isaaclab.interface` already owns everything the *policy* imposes — DOF order, observation
layout and scales, gains, timing. This module owns what the *task* imposes: reward weights and
their shaping constants, command ranges, episode length and termination. Together they are the
whole contract the Isaac Gym environment implements.

Values are from `unifp_go2d1/go2d1_pos_force_config.py`, which is the retarget of UniFP's released
B2Z1 config and carries its own account of what was changed from upstream and why.

**Reward scales here are the config's, not the environment's.** `_prepare_reward_function`
multiplies every scale by `dt` on startup, so a scale read out of a running Isaac Gym environment
is 50x smaller than the one written here. `scaled_weights()` does that multiply explicitly, in one
place, because applying it twice is a silent 50x error in every reward term at once.
"""
from __future__ import annotations

import math

from unifp_isaaclab import interface

#: Reward weights, exactly as the config declares them. Terms whose weight is zero are omitted --
#: upstream declares nine of those, and `_prepare_reward_function` drops them before training, so
#: they are not part of the task.
REWARD_WEIGHTS = {
    # --- the objectives ---
    "tracking_ee_force_world": 2.0,       # the end-effector term: position, or force through a spring
    "tracking_lin_vel_force_world": 2.0,  # the same idea for base velocity
    "tracking_ang_vel": 1.0,
    "feet_contact_number": 2.0,
    "alive": 1.5,
    "ref_dof_leg": 1.0,
    "feet_air_time": 1.0,
    "feet_height": 1.0,
    "stand_still": 0.5,
    # --- regularisation ---
    "lin_vel_z": -1.5,
    "ang_vel_xy": -0.02,
    "base_height": -2.0,
    "collision": -5.0,
    "dof_pos_limits": -10.0,
    "torque_limits": -0.005,
    "torques": -5.0e-6,
    "dof_vel": -8.0e-4,
    "dof_acc": -2.5e-7,
    "dof_vel_arm": -2.0e-4,
    "dof_acc_arm": -4.5e-7,
    "action_rate": -0.02,
    "action_rate_arm": -0.045,
    "hip_pos": -0.5,
    "feet_drag": -0.0008,
    "feet_contact_forces": -0.001,
    "feet_pos_xy": -0.5,
    "feet_height_high": -15.0,
}

# --- this repository's extensions to the task -------------------------------------------------
#
# Kept in their own dictionary rather than folded into `REWARD_WEIGHTS`, because that dictionary is
# the *port's fidelity record*: `tests/test_unifp_train.py` asserts its key set equals the term set
# of a recorded Isaac Gym rollout, term for term. Adding anything to it would either break that
# test or, worse, quietly redefine what "reproduces upstream" means. What upstream's task is and
# what we added to it are different questions and stay in different places.

#: The gripper roll objective (F-095, F-099). Off unless the environment is configured for it.
#:
#: Upstream declares `tracking_ee_orn` and `tracking_ee_orn_ry` at weight zero and implements
#: neither -- there is no `_reward_tracking_ee_orn*` anywhere in its source, so setting a weight on
#: those names raises rather than trains. This is a new term, not a revived one, and it constrains
#: **one** rotational degree of freedom for the reason F-099 measured: over this goal sphere 85.7%
#: of goals admit a position and 8.2% admit a freely chosen orientation, so a full-pose objective
#: would score mostly unreachable targets. Roll is the one that costs no workspace.
EXTENSION_WEIGHTS = {
    "tracking_ee_orn_roll": 1.0,
}

#: Width of the roll reward, radians. `exp(-|error| / sigma)`: 1.0 on target, 0.82 at 10 degrees,
#: 0.21 at 45. Untuned -- it and the weight above are the first things to sweep, and neither has
#: been trained with yet.
TRACKING_EE_ROLL_SIGMA = 0.5

#: Commanded roll range. A jaw axis is an *axis*, so a commanded jaw direction repeats every 180
#: degrees and this half-open interval covers every distinguishable one. 0 is jaws level (the cup
#: on a table); +/- pi/2 is jaws upright (across a lever's horizontal bar).
EE_ROLL_RANGE_RAD = (-math.pi / 2, math.pi / 2)


def scaled_extension_weights() -> dict[str, float]:
    """`EXTENSION_WEIGHTS` multiplied by the policy timestep, as the environment applies them."""
    return {name: weight * interface.POLICY_DT for name, weight in EXTENSION_WEIGHTS.items()}


# --- the controlled point ----------------------------------------------------------------------
#
# **This is not `interface.TOOL_BODY`, and the difference is deliberate.** `unifp_isaaclab.interface`
# records the contract the released checkpoints were *trained* against -- the tip of the Link7_1
# pincer, one finger -- and it must keep saying so, or a playback of `model_48800` would be scored
# against a point that model never tracked. What follows is what this repository's *new* training
# runs control, and it is a different point for two measured reasons:
#
#   * **It does not move when the jaws do** (F-094). `Joint7_1` slides the fingertip up to 30 mm
#     along the hand, and both jaws are dropped from the observation, so opening them to grasp
#     moved the controlled point for a reason the policy could not see: 0.91 cm of tracking error
#     with the jaws shut against 3.50 cm fully open, on the same goal. The two fingers take
#     opposite joint coordinates, so their midpoint is invariant to jaw travel exactly.
#   * **It lies on the roll axis** (F-096, F-099). `Joint6` rolls about Link6's own z through its
#     origin, and this point sits on that axis, so commanding a roll does not disturb the position
#     at all -- which is what makes one rotational degree of freedom free rather than a trade.
#
# The value is `demos/cup/pick_demo/grasp.JAW_CENTRE_LINK6`, which both scripted demos already use
# as the jaw centre. It is 5.6 mm from the true midpoint of the finger roots, along the palm, and
# that idealisation is what puts it exactly on the axis; the pads are 26 mm deep, so the point sits
# inside the volume a grasp closes on. CAD, not measured on the arm -- F-013's caveat applies to it
# exactly as it applied to the fingertip.
#
# **`position_only` still controls the fingertip.** Changing it there would invalidate that task's
# frozen manifests and its target box, so the two tasks now measure different points; that
# divergence is a decision for the P0-P4 ladder, not something to make silently here.
TOOL_BODY = "Link6"
TOOL_OFFSET_M = (0.0, 0.0, 0.1051)

#: Shaping constants the reward terms read.
TRACKING_SIGMA = 0.25          # velocity tracking width
TRACKING_EE_SIGMA = 1.0        # end-effector tracking width
SIGMA_FORCE = 1.0 / 50.0       # force tracking width
BASE_HEIGHT_TARGET = 0.30      # m, the Go2's measured standing height
MAX_CONTACT_FORCE = 100.0      # N, above which foot contact is penalised
SOFT_DOF_POS_LIMIT = 0.8
SOFT_TORQUE_LIMIT = 0.9
CYCLE_TIME = interface.CYCLE_TIME

#: Joint position limits from the URDF, in UniFP's DOF order. Measured identical between the
#: Isaac Gym asset and the Isaac Lab weld on 2026-09-20 (F-087), all twenty joints.
JOINT_POS_LIMITS = (
    (-1.0472, 1.0472), (-1.5708, 3.4907), (-2.7227, -0.8378),
    (-1.0472, 1.0472), (-1.5708, 3.4907), (-2.7227, -0.8378),
    (-1.0472, 1.0472), (-0.5236, 4.5379), (-2.7227, -0.8378),
    (-1.0472, 1.0472), (-0.5236, 4.5379), (-2.7227, -0.8378),
    (-2.35, 2.35), (-1.57, 1.57), (-1.57, 1.57),
    (-2.35, 2.35), (-1.57, 1.57), (-2.35, 2.35),
    (0.0, 0.03), (-0.03, 0.0),
)

#: Hip joints, in UniFP's DOF order: the first of each leg's three.
HIP_INDICES = (0, 3, 6, 9)

#: Bodies whose contact is penalised, as regexes against the Isaac Lab model's body names.
PENALISED_CONTACT_BODIES = (".*_thigh", ".*_calf", "base")

#: Episode length. Upstream's `env.episode_length_s`; also the reason a 1500-step playback trace
#: from the Isaac Gym side contains a reset partway through.
EPISODE_LENGTH_S = 20.0

#: Base velocity command ranges, and the dead zone inside which the robot is told to stand.
LIN_VEL_X_RANGE = (-0.6, 0.6)
LIN_VEL_Y_RANGE = (-0.4, 0.4)
ANG_VEL_YAW_RANGE = (-0.6, 0.6)
COMMAND_RESAMPLING_TIME_S = 5.0
ZERO_VEL_CMD_PROB = 0.3
#: Declared by the config and never read by the environment -- upstream's `_resample_commands`
#: only ever uses `ZERO_VEL_CMD_PROB`. Kept so the config comparison is complete, not because it
#: does anything.
ZERO_VEL_CMD_PROB_AFTER_FORCE = 0.8

# --- external forces -------------------------------------------------------------------------
#
# UniFP pushes the *gripper* in two independent channels, both on the same schedule and both with
# the same numbers:
#
#   * the **command** channel writes into `commands[9:12]`, so the policy is *told* to produce a
#     force it is not given. That is the force half of "unified position/force".
#   * the **external** channel applies a real wrench to the tool, so the policy has to survive a
#     force it is *not* told about. That is the disturbance half.
#
# Both are gated behind `FORCE_START_ITERATION`: the first 8,000 iterations are position-only.
#
# The base push (`_push_robot_base`) exists in upstream's source but its call in `step()` is
# commented out, so no base force is ever applied and `commands[12:15]` stay zero for the whole
# run. `unifp_go2d1` kept it that way so the two stacks match, and so does this. The base force
# constants below are therefore the ones the *rewards* read, not a schedule.

#: Seconds between the starts of two pushes, drawn per push. Both channels, both targets.
PUSH_INTERVAL_S = (3.5, 9.0)
#: Seconds to ramp a push up, and again to ramp it down.
PUSH_DURATION_S = (1.0, 3.0)
#: Seconds the force is held at its peak between the two ramps.
SETTLING_TIME_GRIPPER_S = 1.0
#: Probability that a drawn push is actually applied. The other 20% of envs are "freed": their
#: force is zeroed until the next draw, which is how the task keeps zero-force episodes in the mix.
GRIPPER_FORCED_PROB = 0.8
#: Peak force per axis, newtons, drawn uniformly. Upstream's B2Z1 uses +/-60 N; the D1's strongest
#: joint makes about 7 N at the tool, so `unifp_go2d1` cut this to a band the arm can actually work
#: in. Raising it does not make the arm stronger, it only saturates the torque limits.
GRIPPER_FORCE_RANGE_N = (-8.0, 8.0)

#: Virtual stiffness the end-effector force reward converts a force command into a position offset
#: through, N/m. Upstream randomises it per episode from `gripper_force_kp_range`, which is
#: `[200, 200]` -- a degenerate range, so it is a constant and is written as one here.
GRIPPER_FORCE_KP = 200.0
#: The same for the base velocity term, from `base_force_kd_range = [200, 200]`.
BASE_FORCE_KD = 200.0

#: Iterations of position-only training before any external force is applied.
FORCE_START_ITERATION = 8000
#: Rollout steps per environment per iteration. Only used to turn `FORCE_START_ITERATION` into the
#: step count the environment actually counts, which is how upstream gates it
#: (`global_steps > force_start_step * 24`). Keep it equal to the agent config's.
NUM_STEPS_PER_ENV = 24


def soft_joint_pos_limits() -> tuple[tuple[float, float], ...]:
    """The limits the `dof_pos_limits` penalty actually uses.

    Upstream shrinks each joint's range about its midpoint by `SOFT_DOF_POS_LIMIT`, so the penalty
    starts biting before the hard stop. Recomputed here rather than stored, because it is derived
    from the URDF limits and drifting the two apart would be invisible.
    """
    out = []
    for lower, upper in JOINT_POS_LIMITS:
        middle = 0.5 * (lower + upper)
        half = 0.5 * (upper - lower) * SOFT_DOF_POS_LIMIT
        out.append((middle - half, middle + half))
    return tuple(out)


def scaled_weights() -> dict[str, float]:
    """Reward weights multiplied by the policy timestep, as the environment applies them."""
    return {name: weight * interface.POLICY_DT for name, weight in REWARD_WEIGHTS.items()}

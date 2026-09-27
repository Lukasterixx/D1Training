"""Constants of the insertion task: a held key pushed into a keyed slot on a door, force-aware.

The hierarchical recipe that worked for the mechanisms (F-106, F-108): a task layer (`insertion.InsertionTaskLayer`)
runs the insertion from an estimate of the slot -- the goal to the estimated mouth, a PI force law along the estimated
axis, a light push and a spiral search while the key is on the door's face -- and the whole-body policy is trained with
that layer in the loop, the slot's true pose differing from the estimate by a drawn error it never observes. What the
policy has to learn is the body's part: arrive with the approach and roll it is commanded (a slot admits a few degrees),
keep a light push on a face while sliding across it, yield across the axis when a chamfer pushes, and push through a
detent without lunging into the seat.

Every number is invented or taken from the slot sizes of common enclosure keys; none is measured. Reward weights are
per second, as elsewhere (`scaled_weights`).
"""
from __future__ import annotations

import math

from unifp_isaaclab import interface

from . import insertion

#: Fraction of episodes with a slot. The rest reach through free space as UniFP's task does, no pushes.
INSERTION_FRACTION = 0.8

# --- where the slot is --------------------------------------------------------------------------------------------
#
# In the robot's yaw frame, measured from the ground under the base: the mouth's height, its distance ahead and to
# the side. A level approach is reachable with the jaw centre 0.65-0.95 m up and 0.30-0.50 m ahead of the arm's
# mount (arm-only IK, 2026-09-26 log), and the jaw centre is the key's length plus the standoff behind the mouth.
MOUTH_HEIGHT_M = (0.66, 0.92)
MOUTH_AHEAD_M = (0.46, 0.60)
MOUTH_SIDE_M = (-0.08, 0.08)
#: The slot's axis elevation (a door leaning back or forward) and its roll (the key's blade orientation), rad.
AXIS_ELEVATION_RAD = (math.radians(-10.0), math.radians(10.0))
ROLL_RAD = (-math.pi / 2, math.pi / 2)
#: The key starts this far short of the estimated mouth.
STANDOFF_M = (0.008, 0.02)

# --- the slot's geometry --------------------------------------------------------------------------------------------
#
# Half sizes of the channel (the key blade is `insertion.KEY_HALF_U_M` x `KEY_HALF_W_M`): the clearance per side is
# drawn, so the policy meets tight and loose slots.
CLEARANCE_M = (0.0006, 0.0015)
CHAMFER_M = (0.0015, 0.0025)
DEPTH_M = (0.014, 0.022)
DETENT_FRACTION = (0.5, 0.8)    # of the depth
FRICTION_MU = (0.15, 0.45)

# --- the curriculum -------------------------------------------------------------------------------------------------
#
# One level in [0, 1] scales how wrong the estimate is and how stiff the detent: at 1, the slot is up to
# `LATERAL_ERROR_M` off across the axis, its axis `TILT_ERROR_RAD` and its roll `ROLL_ERROR_RAD` off, and the
# detent needs up to `DETENT_N`. Raised by `CURRICULUM_STEP` when the seated fraction passes `PROMOTE_SEATED`, at
# most once per `CURRICULUM_MIN_ITERATIONS`.
LATERAL_ERROR_M = 0.005
#: ... and, from v6 (2026-09-27), how much easier the slot is below level 1: this much more clearance per side and
#: chamfer at level 0, shrinking linearly to the ranges above at level 1, so the first episodes can seat at all.
CLEARANCE_EASY_EXTRA_M = 0.0025
CHAMFER_EASY_EXTRA_M = 0.003
TILT_ERROR_RAD = math.radians(3.0)
ROLL_ERROR_RAD = math.radians(5.0)
DETENT_N = 25.0
CURRICULUM_START = 0.2
CURRICULUM_STEP = 0.1
PROMOTE_SEATED = 0.7
CURRICULUM_MIN_ITERATIONS = 50
CURRICULUM_EMA = 0.05

# --- the task layer -------------------------------------------------------------------------------------------------
REFERENCE_SPEED_M_S = (0.01, 0.03)
SEARCH_N = (4.0, 8.0)
MAX_N = (25.0, 40.0)
FORCE_LAW_KP_N_PER_M = 400.0
FORCE_LAW_KI_N_PER_M_S = 1000.0
FORCE_LAW_BLEED_S = 1.0
#: The task layer starts once the hand has settled at its pre-insertion pose -- slower than this, m/s -- with the key's
#: tip within `ENGAGE_CONVERGED_M` of it when the outer loops are on (`ENGAGE_NEAR_M` when off). By the deadline,
#: `ENGAGE_MAX_WAIT_STEPS` policy steps after the goal arrived, it starts anyway if the tip is within `ENGAGE_NEAR_M`;
#: otherwise the episode gets no slot and is a free reach.
ENGAGE_SETTLE_SPEED_M_S = 0.05
ENGAGE_DELAY_STEPS = (10, 20)
ENGAGE_MAX_WAIT_STEPS = 150
ENGAGE_NEAR_M = 0.03
ENGAGE_CONVERGED_M = 0.003

# --- the outer loops (`insertion.PoseCorrection`) ----------------------------------------------------------------------
#
# Integral gains, 1/s, and their clamps, on the tip's miss, the approach's elevation and the blade's roll. At 1.5/s the
# time constant (0.7 s) is slower than the policy's own settling, so the loop corrects its steady miss rather than
# chasing its transient. 0 turns a loop off; the policies before 2026-09-26's v6 were trained and scored without them.
TIP_CORRECTION_GAIN = 1.5
TIP_CORRECTION_MAX_M = 0.05
ELEVATION_CORRECTION_GAIN = 1.5
ROLL_CORRECTION_GAIN = 1.5
#: The attitude loops' clamps were 20° each until 2026-09-27, when v7 `model_20200` was found holding a level blade at
#: 0.86 m with its roll 30° off and its elevation 8-11° off, both loops at their clamps: widened, it seats 59/96 frozen
#: slots against 55 (the level blades at 0.86 m 5/16 against 0). The tip's clamp stays: at 8 cm it only added damage.
ELEVATION_CORRECTION_MAX_RAD = math.radians(30.0)
ROLL_CORRECTION_MAX_RAD = math.radians(40.0)

# --- safety ------------------------------------------------------------------------------------------------------------
#: Contact force above which the episode ends as a failure: a key rammed into a door, or a door broken.
DAMAGE_N = 150.0

# --- the reward --------------------------------------------------------------------------------------------------------
WEIGHTS = {
    # The key's depth against the task layer's reference.
    "ins_progress": 4.0,
    # The jaw centre's precision (`insert_rewards.ins_reach`): to the pre-insertion goal, then across the axis.
    "ins_reach": 4.0,
    # Seated: the key home.
    "ins_seated": 2.0,
    # Force across the estimated axis, squared per (`LATERAL_SCALE_N`)^2: a key dragged sideways in its slot jams it,
    # and on the face it is friction fought for nothing.
    "ins_lateral": -1.0,
    # Every contact force above `SAFE_N`, squared per (10 N)^2: ramming the door.
    "ins_overforce": -1.0,
    # Ended by `DAMAGE_N` or a key driven through the door; paid once.
    "ins_damage": -500.0,
    # The tool's squared speed while the goal is held (from v6, 2026-09-27): hold still.
    "ins_tool_speed": -8.0,
    # The pull task's force tracking along the estimated axis (the hierarchical recipe, F-108).
    "fixture_force_tracking": 4.0,
    # The arm's motors near their limits while in contact: the body's job.
    "arm_torque_margin": -4.0,
}
LATERAL_SCALE_N = 10.0
REACH_SIGMA_FINE_M = 0.01
REACH_SIGMA_COARSE_M = 0.04
SAFE_N = 40.0
PROGRESS_SIGMA_FINE_M = 0.002
PROGRESS_SIGMA_COARSE_M = 0.008

#: The critic's privileged block, the same 22-wide slot the other tasks use.
PRIVILEGED_WIDTH = 22


def scaled_weights(weights: dict[str, float] | None = None) -> dict[str, float]:
    return {name: weight * interface.POLICY_DT for name, weight in (WEIGHTS if weights is None else weights).items()}


# --- evaluation ------------------------------------------------------------------------------------------------------
#
# Fixed slots, frozen 2026-09-26 before any policy was scored on them: three heights of the mouth straight ahead at
# 0.52 m, the estimate off across the axis by `EVAL_LATERAL_M` in four directions (along +/-u and +/-w), a 1.5° tilt
# and 3° roll error, a detent of `EVAL_DETENT_N`. One episode each; a horizontal and a vertical blade. Everything else
# at the middle of its training range.
EVAL_HEIGHTS_M = (0.70, 0.78, 0.86)
EVAL_AHEAD_M = 0.52
EVAL_LATERAL_M = (0.0, 0.0015, 0.003, 0.0045)
EVAL_DIRECTIONS_RAD = (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)
EVAL_ROLLS_RAD = (0.0, math.pi / 2)
EVAL_TILT_RAD = math.radians(1.5)
EVAL_ROLL_ERROR_RAD = math.radians(3.0)
EVAL_DETENT_N = 12.0
EVAL_CONDITIONS = {"clearance_m": 0.001, "chamfer_m": 0.002, "depth_m": 0.018, "detent_fraction": 0.65, "mu": 0.3,
                   "standoff_m": 0.012, "speed_m_s": 0.02, "search_n": 6.0, "max_n": 30.0}
EVAL_EPISODE_S = 16.0
#: Also by detent, at 1.5 mm off: how hard a click the policy pushes through.
EVAL_DETENT_LEVELS_N = (0.0, 10.0, 20.0, 30.0)


def key_half_sizes() -> tuple[float, float]:
    return insertion.KEY_HALF_U_M, insertion.KEY_HALF_W_M

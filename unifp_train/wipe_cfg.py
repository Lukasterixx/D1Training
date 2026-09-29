"""Constants of the wiping task: a pad held in the jaws, wiped along a path on a flat panel at a set force.

The plan's wiping task (`docs/thesis_b_plan.md`): "planar fixture with known normal; slow prescribed path; rigid pad
initially", success "at least 90% path completion and at least 80% contact during motion". The hierarchical recipe
again (F-106, F-108, F-120): a task layer (`wiping.WipeTaskLayer`) runs the wipe from an estimate of the surface -- the
pad to a standoff over the path's start, a PI force law along the estimated normal capped at the wiping force, a
reference point along the path with a PI force across the normal to push through friction, a lift at the end -- and
the whole-body policy is trained with that layer in the loop, the true surface off the estimate by a drawn offset and
tilt it never observes. What the policy has to learn is the body's part: hold a steady force on a surface that is not
quite where it was said to be while the hand moves across it, without the tool's jitter (F-104, F-119) breaking the
contact.

The panels face the robot (the combiner box's door): wiping a table top needs the hand pointing down, which the D1
mostly cannot command at this reach (F-099).

Every number is invented; none is measured. Reward weights are per second, as elsewhere (`scaled_weights`).
"""
from __future__ import annotations

import math

from unifp_isaaclab import interface

from . import wiping

#: Fraction of episodes with a surface to wipe. The rest reach through free space as UniFP's task does, no pushes.
WIPE_FRACTION = 0.8

# --- where the surface is -----------------------------------------------------------------------------------------
#
# In the robot's yaw frame, from the ground under the base: the path's centre on the surface -- its height, its
# distance ahead and to the side. The pad's centre is `wiping.PAD_LENGTH_M` beyond the jaw centre, so the jaw works
# 5-6 cm nearer than the surface: the insertion's reachable box for a level approach (`insert_cfg.MOUTH_*`).
CENTRE_HEIGHT_M = (0.66, 0.90)
CENTRE_AHEAD_M = (0.46, 0.58)
CENTRE_SIDE_M = (-0.05, 0.05)
#: The panel's lean: the elevation of its inward normal, rad.
NORMAL_ELEVATION_RAD = (math.radians(-10.0), math.radians(10.0))
#: The panel's half sizes: larger than any path, so its edge is not what ends a wipe.
PANEL_HALF_M = 0.25
#: The pad starts this far off the estimated surface.
STANDOFF_M = (0.01, 0.02)

# --- the path --------------------------------------------------------------------------------------------------------
LINE_PROB = 0.5
LINE_LENGTH_M = (0.08, 0.18)
LINE_ANGLE_RAD = (math.radians(-30.0), math.radians(30.0))
ZIGZAG_LENGTH_M = (0.06, 0.10)
ZIGZAG_SPACING_M = (0.02, 0.035)
ZIGZAG_STROKES = 3
#: Kinds, as `WipeEnv._kind` records them.
KINDS = ("line", "zigzag")

# --- the contact and the task layer -------------------------------------------------------------------------------
FORCE_N = (3.0, 15.0)
SPEED_M_S = (0.03, 0.06)
APPROACH_SPEED_M_S = 0.02
FRICTION_MU = (0.2, 0.6)
#: How far past the estimated surface the normal law's reference runs: more than the offset error, so a surface further
#: than estimated is still reached and pressed at the full force.
OVERDRIVE_M = 0.015
FORCE_LAW_KP_N_PER_M = 400.0
FORCE_LAW_KI_N_PER_M_S = 1000.0
#: The in-plane law pushing through friction: gains, and its cap as a multiple of the wiping force (above any mu drawn).
PATH_LAW_KP_N_PER_M = 200.0
PATH_LAW_KI_N_PER_M_S = 400.0
PATH_LAW_CAP_RATIO = 1.0
#: As the insertion: start once the hand has settled at the pre-contact pose with the pad converged, or by the deadline if
#: near; otherwise the episode gets no surface and is a free reach.
ENGAGE_SETTLE_SPEED_M_S = 0.05
ENGAGE_DELAY_STEPS = (10, 20)
ENGAGE_MAX_WAIT_STEPS = 150
ENGAGE_NEAR_M = 0.03
ENGAGE_CONVERGED_M = 0.003

# --- the curriculum -------------------------------------------------------------------------------------------------
#
# One level in [0, 1] scales how wrong the estimate is: at 1 the surface is up to `OFFSET_ERROR_M` nearer or further than
# estimated and tilted up to `TILT_ERROR_RAD` from it. Raised by `CURRICULUM_STEP` when the successful fraction passes
# `PROMOTE_SUCCESS`, at most once per `CURRICULUM_MIN_ITERATIONS`.
OFFSET_ERROR_M = 0.01
TILT_ERROR_RAD = math.radians(4.0)
CURRICULUM_START = 0.2
CURRICULUM_STEP = 0.1
PROMOTE_SUCCESS = 0.6
CURRICULUM_MIN_ITERATIONS = 50
CURRICULUM_EMA = 0.05

# --- the outer loops (`insertion.PoseCorrection`) --------------------------------------------------------------------
#
# The tip loop on the pad's centre (before contact; while wiping the in-plane law carries the miss), the elevation loop
# on the approach. A disc has no roll: the jaws are commanded level.
TIP_CORRECTION_GAIN = 1.5
TIP_CORRECTION_MAX_M = 0.05
ELEVATION_CORRECTION_GAIN = 1.5
ELEVATION_CORRECTION_MAX_RAD = math.radians(30.0)

# --- success and safety --------------------------------------------------------------------------------------------
#: The plan's criteria.
SUCCESS_COMPLETION = 0.9
SUCCESS_CONTACT = 0.8
#: A contact loss shorter than this is not counted as an event (debounce).
LOSS_DEBOUNCE_S = 0.1
#: Normal force above which the episode ends as a failure.
DAMAGE_N = 80.0

# --- the reward --------------------------------------------------------------------------------------------------------
WEIGHTS = {
    # The pad centre's precision: to the pre-contact goal, then in the plane on the path's reference point.
    "wipe_track": 4.0,
    # In contact while wiping.
    "wipe_contact": 3.0,
    # The pull task's force tracking, along the estimated normal (the hierarchical recipe, F-108).
    "fixture_force_tracking": 4.0,
    # Normal force above `SAFE_N`, squared per (10 N)^2.
    "wipe_overforce": -1.0,
    # Ended by `DAMAGE_N` or the pad through the panel; paid once.
    "wipe_damage": -500.0,
    # The pad's squared velocity error while its goal is held: still before contact, the path's speed while wiping.
    "wipe_tool_speed": -4.0,
    # The arm's motors near their limits while in contact.
    "arm_torque_margin": -4.0,
}
TRACK_SIGMA_FINE_M = 0.01
TRACK_SIGMA_COARSE_M = 0.04
SAFE_N = 30.0

#: The critic's privileged block, the same 22-wide slot the other tasks use.
PRIVILEGED_WIDTH = 22


def scaled_weights(weights: dict[str, float] | None = None) -> dict[str, float]:
    return {name: weight * interface.POLICY_DT for name, weight in (WEIGHTS if weights is None else weights).items()}


# --- evaluation ------------------------------------------------------------------------------------------------------
#
# Fixed surfaces, frozen 2026-09-29 before any policy was scored on them: a panel straight ahead at `EVAL_AHEAD_M`,
# the path centred at three heights; a 16 cm horizontal stroke or a three-stroke zigzag (12 cm strokes 3 cm apart);
# the surface nearer, where or further than estimated by `EVAL_OFFSETS_M`; two wiping forces. The surface's tilt from
# the estimate alternates between `EVAL_TILT_RAD` about its horizontal and its vertical axis. At the slow end of the
# training speeds.
EVAL_HEIGHTS_M = (0.70, 0.78, 0.86)
EVAL_AHEAD_M = 0.52
EVAL_LINE_LENGTH_M = 0.16
EVAL_ZIGZAG_LENGTH_M = 0.12
EVAL_ZIGZAG_SPACING_M = 0.03
EVAL_OFFSETS_M = (-0.008, 0.0, 0.008)
EVAL_FORCES_N = (5.0, 10.0)
EVAL_TILT_RAD = math.radians(3.0)
EVAL_CONDITIONS = {"standoff_m": 0.015, "speed_m_s": 0.03, "mu": 0.4}
EVAL_EPISODE_S = 32.0

#: The pad, for reference in run records.
PAD = {"radius_m": wiping.PAD_RADIUS_M, "length_m": wiping.PAD_LENGTH_M, "points": wiping.PAD_POINTS}

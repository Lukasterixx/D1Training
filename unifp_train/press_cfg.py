"""Constants of the pressing task: a spring-loaded push button pressed with the bare jaws or a probe, force-aware.

The plan's calibration task (`docs/thesis_b_plan.md`): "spring-loaded button with travel and spring/damping parameters;
bare end effector, then rounded probe", success "full button travel and engaged for 1 s". The hierarchical recipe of
the mechanisms and the key (F-106, F-108, F-120): a task layer (`pressing.PressTaskLayer`) runs the press from an
estimate of the button and its datasheet -- the tool's tip to a standoff, a PI force law along the estimated axis, a
steady hold at full travel, a release -- and the whole-body policy is trained with that layer in the loop, the true
button off the estimate by a drawn error it never observes. What the policy has to learn is the body's part: arrive
with the approach it is commanded, push along the axis without sliding off the cap, hold a steady force against the
stop without lunging when the click lets go, and let go cleanly.

The buttons are panel buttons, pressed away from the robot (the combiner box's door is vertical): a button on a top
face pressed down needs an approach pointing down, which the D1 mostly cannot command at this reach (F-099).

Every number is invented or taken from common pushbutton datasheets (travel 3-8 mm, operating force 2-30 N); none is
measured. Reward weights are per second, as elsewhere (`scaled_weights`).
"""
from __future__ import annotations

import math

from unifp_isaaclab import interface

from . import pressing

#: Fraction of episodes with a button. The rest reach through free space as UniFP's task does, no pushes.
BUTTON_FRACTION = 0.8
#: The tools drawn per episode (`pressing.TOOLS`), equally often.
TOOLS = pressing.TOOL_NAMES

# --- where the button is -------------------------------------------------------------------------------------------
#
# In the robot's yaw frame, from the ground under the base: the cap's height, its distance ahead and to the side. The
# key insertion's reachable box for a level approach (`insert_cfg.MOUTH_*`), a little lower and nearer because the
# bare jaws reach 50 mm less far than the key.
FACE_HEIGHT_M = (0.62, 0.92)
FACE_AHEAD_M = (0.40, 0.58)
FACE_SIDE_M = (-0.08, 0.08)
#: The panel's lean: the axis's elevation, rad.
AXIS_ELEVATION_RAD = (math.radians(-10.0), math.radians(10.0))
#: The tool's tip starts this far short of the estimated cap.
STANDOFF_M = (0.008, 0.02)

# --- the button -------------------------------------------------------------------------------------------------------
CAP_RADIUS_M = (0.006, 0.020)
#: How far the cap stands proud of the panel (negative: recessed). Redrawn up to pressable for the episode's tool
#: (`pressing.reachable_front`): the recessed buttons only the probe can press are a declared evaluation (`recessed`),
#: not something to learn by ramming a rim.
PROUD_M = (-0.002, 0.008)
TRAVEL_M = (0.003, 0.008)
PRELOAD_N = (1.0, 6.0)
STIFFNESS_N_PER_M = (300.0, 3000.0)
DAMPING_N_S_PER_M = (0.5, 5.0)
SNAP_AT = (0.4, 0.7)
FRICTION_MU = (0.2, 0.6)

# --- the curriculum -------------------------------------------------------------------------------------------------
#
# One level in [0, 1] scales how wrong the estimate is, how hard the click and how far the datasheet's force is off: at
# 1 the cap is up to `LATERAL_ERROR_M` across the axis from the estimate, its face `DEPTH_ERROR_M` along it, its axis
# `TILT_ERROR_RAD` off, the click up to `SNAP_N`, the rated force wrong by up to `RATING_ERROR`. Raised by
# `CURRICULUM_STEP` when the pressed fraction passes `PROMOTE_PRESSED`, at most once per `CURRICULUM_MIN_ITERATIONS`.
LATERAL_ERROR_M = 0.006
DEPTH_ERROR_M = 0.003
TILT_ERROR_RAD = math.radians(4.0)
SNAP_N = 4.0
RATING_ERROR = 0.2
CURRICULUM_START = 0.2
CURRICULUM_STEP = 0.1
PROMOTE_PRESSED = 0.7
CURRICULUM_MIN_ITERATIONS = 50
CURRICULUM_EMA = 0.05

# --- the task layer -------------------------------------------------------------------------------------------------
REFERENCE_SPEED_M_S = (0.01, 0.03)
#: How far past the estimated full travel the reference runs: more than the depth error, so a button further than
#: estimated is still bottomed.
OVERDRIVE_M = 0.004
#: The light push until something is touched, N.
TOUCH_N = 3.0
#: The press's cap and the hold, as multiples of the datasheet's operating and full-travel forces (with its error).
MAX_MARGIN = (1.6, 2.4)
HOLD_MARGIN = (1.2, 1.6)
#: Never more than this, whatever the datasheet says.
MAX_N_LIMIT = 60.0
HOLD_S = 1.5
FORCE_LAW_KP_N_PER_M = 400.0
FORCE_LAW_KI_N_PER_M_S = 1000.0
#: As the insertion: start once the hand has settled at the pre-press pose with the tip converged, or by the deadline if
#: near; otherwise the episode gets no button and is a free reach.
ENGAGE_SETTLE_SPEED_M_S = 0.05
ENGAGE_DELAY_STEPS = (10, 20)
ENGAGE_MAX_WAIT_STEPS = 150
ENGAGE_NEAR_M = 0.03
ENGAGE_CONVERGED_M = 0.003

# --- the outer loops (`insertion.PoseCorrection`) --------------------------------------------------------------------
#
# The insertion's (F-120): the tip's miss integrated into the jaw goal, the approach's elevation into its command. No
# roll loop -- a sphere has no roll -- and the roll is commanded level.
TIP_CORRECTION_GAIN = 1.5
TIP_CORRECTION_MAX_M = 0.05
ELEVATION_CORRECTION_GAIN = 1.5
ELEVATION_CORRECTION_MAX_RAD = math.radians(30.0)

# --- success and safety --------------------------------------------------------------------------------------------
#: The plan's criterion: full travel held this long.
SUCCESS_HOLD_S = 1.0
#: Contact force above which the episode ends as a failure: a button or a panel broken.
DAMAGE_N = 100.0

# --- the reward --------------------------------------------------------------------------------------------------------
WEIGHTS = {
    # The tip's depth against the task layer's reference, while it presses.
    "press_progress": 4.0,
    # The tip's precision: to the pre-press goal, then across the axis.
    "press_reach": 4.0,
    # Pressed: the cap at its full travel.
    "press_pressed": 3.0,
    # Force across the estimated axis, squared per (`LATERAL_SCALE_N`)^2: a tool sliding off the cap.
    "press_lateral": -1.0,
    # Contact force above `SAFE_N`, squared per (10 N)^2.
    "press_overforce": -1.0,
    # Ended by `DAMAGE_N` or a tool driven through the panel; paid once.
    "press_damage": -500.0,
    # The tool's squared speed while its goal is held, across the axis once the layer has started (the insertion's).
    "press_tool_speed": -8.0,
    # The pull task's force tracking along the estimated axis (the hierarchical recipe, F-108).
    "fixture_force_tracking": 4.0,
    # The arm's motors near their limits while in contact.
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
# Fixed buttons, frozen 2026-09-29 before any policy was scored on them. Every set is straight ahead at `EVAL_AHEAD_M`
# at three heights, with each tool; the estimate's depth error alternates +/- `EVAL_DEPTH_ERROR_M`, its tilt
# `EVAL_TILT_RAD` turns with the lateral error's direction, and its rated force alternates +/- `EVAL_RATING_ERROR`.
#
#   * `panel`: a common panel button (`EVAL_BUTTON`), the estimate off across the axis by each of `EVAL_LATERAL_M` in
#     four directions -- the precision half of the task;
#   * `force`: the same cap at `EVAL_FORCE_LEVELS_N` of full-travel force, 2 mm off in two directions -- the force half,
#     the calibration the plan uses pressing for;
#   * `recessed`: a small cap 2 mm below the panel (`EVAL_RECESSED_BUTTON`), which the bare jaws' 10 mm tip cannot reach
#     and the probe can -- the plan's access limitation, declared in advance: the bare jaws are expected to fail it.
EVAL_HEIGHTS_M = (0.70, 0.78, 0.86)
EVAL_AHEAD_M = 0.50
EVAL_LATERAL_M = (0.0, 0.002, 0.004, 0.006)
EVAL_DIRECTIONS_RAD = (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)
EVAL_DEPTH_ERROR_M = 0.002
EVAL_TILT_RAD = math.radians(2.0)
EVAL_RATING_ERROR = 0.1
EVAL_BUTTON = {"cap_radius_m": 0.010, "proud_m": 0.003, "travel_m": 0.005, "preload_n": 3.0,
               "stiffness_n_per_m": 1200.0, "damping": 2.0, "snap_n": 2.0, "snap_at": 0.6, "mu": 0.4}
EVAL_FORCE_LEVELS_N = (5.0, 10.0, 20.0, 30.0)
EVAL_FORCE_LATERAL_M = 0.002
EVAL_RECESSED_BUTTON = {"cap_radius_m": 0.004, "proud_m": -0.002, "travel_m": 0.003, "preload_n": 2.0,
                        "stiffness_n_per_m": 1000.0, "damping": 1.0, "snap_n": 1.0, "snap_at": 0.6, "mu": 0.4}
EVAL_RECESSED_LATERAL_M = (0.0, 0.001, 0.002)
EVAL_CONDITIONS = {"standoff_m": 0.012, "speed_m_s": 0.02, "max_margin": 2.0, "hold_margin": 1.4}
EVAL_EPISODE_S = 16.0

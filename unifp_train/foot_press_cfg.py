"""Constants for the foot-press task (`foot_press.py`, `foot_press_env.py`): plates anywhere the front-right foot reaches,
facing any way it can push them, pressed with a commanded force.

Lukas, 2026-10-21: develop the force-aware foot pushing "into a policy trained to go in many positions and directions
just like UniFP so it can also be used for pressing etc." UniFP's arm policy is general -- any goal in its sphere, any
force command -- and is shown on tasks it was never trained on. The key policy (F-179) is not: its contact is one key at
one place. This task keeps UniFP's free-space goal walk with pushes and adds, in a share of episodes, a flat plate
placed where the foot can reach it and facing the shoulder within 60 degrees, rigid or sprung, that a task layer
approaches, finds by the foot stalling, and presses with a profile of normal (and some tangential) forces, sliding
along it in some episodes. The stance is the key's (settled spawn, sport stands, the support feet held to their
places), so the same handover applies.

Every number here is a choice, not a measurement, unless it says otherwise.
"""
from __future__ import annotations

import math

# --- where plates go -----------------------------------------------------------------------------------------------
#
# A plate is a rectangle with a face, a unit normal out of it and two in-plane axes, in the yaw frame of a robot
# standing at the origin facing +x (from the ground under the base). It is drawn from a table built once per process
# (`foot_press.PlateTable`): a contact point the foot sphere's centre reaches (a kinematic sample of the FR leg with the
# base level at both `REACH_HEIGHTS_M`), a normal within `NORMAL_TILT_MAX_RAD` of the direction from that point to the
# FR thigh joint, the foot able to stand off it and to push past it, and the whole plate clear of the floor, the trunk,
# the head and every leg as the robot stands.

#: Contact points are drawn uniformly in this box (yaw frame from the ground) and kept between these distances from the
#: FR thigh joint (`SHOULDER_M`): the leg is 0.426 m at full stretch, and a foot far out can neither hold its place nor
#: push (since 2026-10-21 22:50; the first table's median was 0.31 m, see `foot_press.PlateTable`).
CONTACT_BOX_M = ((0.10, 0.52), (-0.46, 0.04), (0.02, 0.52))
CONTACT_DIST_M = (0.13, 0.30)
#: The free-space goals' yaw about the goal centre (`leg_cfg.GOAL_YAW_RANGE` is -0.65 to 0.25): plates to the foot's right
#: lie past -1 rad.
GOAL_YAW_RANGE = (-1.2, 0.25)
#: Half the candidates are drawn ahead of the FR foot instead.
AHEAD_SHARE = 0.5
AHEAD_BOX_M = ((0.25, 0.52), (-0.30, -0.02), (0.04, 0.52))
#: The FR leg's reach is sampled at these base heights (the settled stance's 0.272 m, the sport stands' 0.32 m); a point
#: counts as reached within `REACH_TOL_M` of the sample at both.
REACH_HEIGHTS_M = (0.272, 0.32)
REACH_TOL_M = 0.015
#: The FR thigh joint the normals face, yaw frame from the ground (`leg_cfg.GOAL_CENTRE`, at a 0.30 m base).
SHOULDER_M = (0.19, -0.14, 0.30)
NORMAL_TILT_MAX_RAD = math.radians(60.0)
#: Shares of plates turned into vertical walls (the normal made horizontal) and level surfaces (straight up or down).
WALL_SNAP_PROB = 0.25
LEVEL_SNAP_PROB = 0.15
#: The foot has to reach this far into the plate (room to push) and this far off it (the layer's furthest standoff).
PUSH_ROOM_M = 0.02
STANDOFF_REACH_M = 0.05
#: Half-sizes of the plate (m, uniform) and where on it the contact point lies, as a share of each half-size.
PLATE_HALF_M = (0.03, 0.15)
CONTACT_OFFSET_SHARE = 0.6
#: The slab behind the face: a sphere whose centre is further behind the face than this has gone through it.
PLATE_THICKNESS_M = 0.02
#: Keep-outs as the robot stands (yaw frame from the ground): the trunk with the D1 folded on it and the head with its
#: RealSense (to 0.36 m ahead), and every leg's links (hip -> knee -> foot at the settled stance) by this clearance.
TRUNK_KEEPOUT_M = ((-0.32, 0.38), (-0.15, 0.15), (0.13, 0.52))
LEG_CLEARANCE_M = 0.04
#: The contact point's foot-centre height at least (a plate's face 1 cm off the floor, at the lowest); it is also ahead of
#: and right of where the foot's free goals may go (`leg_cfg.GOAL_MIN_AHEAD_M`, `GOAL_MAX_LEFT_M`).
MIN_CONTACT_HEIGHT_M = 0.032
TABLE_SIZE = 20000
TABLE_SEED = 20261022

# --- what a plate is -----------------------------------------------------------------------------------------------

#: Share of rigid plates (the foot's penalty contact, 8,000 N/m, `handle.CONTACT_K_N_PER_M`); the rest are sprung: a
#: spring of `SOFT_K_N_PER_M` (log-uniform) over a travel of `SOFT_TRAVEL_M`, then the rigid stop (a button, a pedal, a
#: printed part flexing, a door giving).
RIGID_PROB = 0.4
SOFT_K_N_PER_M = (400.0, 8000.0)
SOFT_TRAVEL_M = (0.005, 0.03)
PLATE_MU = (0.4, 0.9)
#: The foot's tangential contact sticks: a spring of this stiffness to where it touched down, slipping past mu x normal
#: (a regularised Coulomb friction would let the foot creep under any sideways force).
TANGENT_K_N_PER_M = 4000.0
TANGENT_C_N_S_PER_M = 20.0
SUBSTEPS = 4

# --- what the layer is told, and how wrong it is ---------------------------------------------------------------------

EST_ERROR_PLANE_M = 0.010
EST_ERROR_NORMAL_M = 0.010
EST_ERROR_TILT_RAD = math.radians(4.0)

# --- the task layer --------------------------------------------------------------------------------------------------
#
# The scripted approach is the key's (lift straight up, to a waypoint, onto the pre-contact point, settle, the outer loop
# on the foot's miss), the waypoint `WAYPOINT_OUT_M` further out along the estimated normal. The layer then moves the
# reference in along the normal at `APPROACH_SPEED_M_S` up to `OVER_TRAVEL_M` past the estimated face, a light touch
# force commanded (force-aware only), and calls contact when the foot stalls -- `DETECT_LAG_M` behind the reference and
# slower than `DETECT_SPEED_M_S` -- or `DETECT_WAIT_S` after the reference stopped. Where the foot stalled is the press's
# origin. It then runs the profile: up to three levels of normal force, each ramped to and held, some with a tangential
# part, the last sliding along the plate in some episodes; then it ramps the force off, backs out to the standoff, and the
# foot goes back to its footfall. The force-aware layer commands the force and keeps the goal at the origin (UniFP's
# formulation does the rest), the goal following the foot along the normal as it sinks into a sprung plate; the
# position-only layer commands the goal `force / FOOT_FORCE_KP` past it, the target the force-aware reward scores before
# contact, and keeps it there.

STANDOFF_M = (0.03, 0.05)
WAYPOINT_OUT_M = 0.06
APPROACH_SPEED_M_S = (0.03, 0.06)
OVER_TRAVEL_M = 0.04
TOUCH_N = 3.0
DETECT_LAG_M = 0.012
DETECT_SPEED_M_S = 0.015
DETECT_WAIT_S = 0.5
#: ... and only once the reference is within `DETECT_FROM_M` of the estimated face, the foot within `DETECT_ACROSS_M` of
#: the reference's line (since 2026-10-21 22:15).
DETECT_FROM_M = 0.02
DETECT_ACROSS_M = 0.025
#: While it presses, the force-aware layer's goal follows the foot along the normal with this lag, at most this far into
#: the plate and this far out of it from where contact was called (since 2026-10-21 22:20, `PressLayer.step`).
FOLLOW_TAU_S = 0.1
FOLLOW_MAX_M = 0.04
FOLLOW_OUT_M = 0.01
#: The profile: 1-3 segments, each a level drawn in `FORCE_LEVELS_N`, ramped to over `RAMP_S` and held `SEG_HOLD_S`.
FORCE_LEVELS_N = (3.0, 35.0)
SEGMENTS = (1, 3)
RAMP_S = (0.3, 1.0)
SEG_HOLD_S = (0.8, 2.0)
#: In this share of episodes every segment has a tangential part, up to `TANGENT_RATIO` of its normal level, one way.
TANGENT_PROB = 0.3
TANGENT_RATIO = 0.3
#: In this share the last segment slides the goal along the plate once its ramp is done.
SLIDE_PROB = 0.3
SLIDE_SPEED_M_S = (0.02, 0.04)
SLIDE_DIST_M = (0.02, 0.06)
RELEASE_FORCE_S = 0.3
RELEASE_S = 0.8
#: The lift's first leg goes straight up off the floor to the waypoint's height (within these), then across to it; the
#: return goes out to the waypoint, across at its height to above the footfall, and straight down (since 2026-10-21 23:35).
LIFT_MIN_M = 0.05
LIFT_MAX_M = 0.35
RETURN_OUT_S = 0.8
RETURN_OVER_S = 0.8
RETURN_DOWN_S = 1.0

# --- episodes, rewards and limits ----------------------------------------------------------------------------------

EPISODE_LENGTH_S = 20.0
#: Contact past this peak force is penalised per cm of its overlap (`press_excess`, both variants): above every command.
PRESS_LIMIT_N = 45.0
#: Weights (before the `dt` multiply), added to `leg_cfg.REWARD_WEIGHTS` in press episodes. `press_force` is the
#: force-aware variant's only (the plan's P0 rule: no force term for a policy given no force).
PLATE_WEIGHTS = {
    "handle_contact": 1.0,         # the foot on the plate while the layer presses
    "handle_slip": -3.0,           # off the plate after touching it, while pressing
    "door_push": -1.0,             # the knee or the head into the plate, per cm
    "approach_contact": -2.0,      # the foot, knee or head into the plate before the press, per cm
}
PRESS_FORCE_WEIGHT = 2.0
PRESS_FORCE_SIGMA_N = 6.0
PRESS_EXCESS_WEIGHT = -4.0
#: The base's tilt (UniFP's `orientation`, -1 in `leg_cfg.REWARD_WEIGHTS`), in every episode of this task: round 2's
#: model_3000 reared 32-40 degrees through every approach and press (since 2026-10-21 23:35).
ORIENTATION_WEIGHT = -4.0

# --- the frozen evaluation -----------------------------------------------------------------------------------------
#
# 48 plates (12 of each kind: tilted, wall, level facing up, level facing down) drawn once from a table built with
# `EVAL_TABLE_SEED`, x rigid / sprung (1,000 N/m over 25 mm) x three levels, one segment each (ramped over 0.5 s, held
# 3 s), no tangential part, no slide; the estimate's errors drawn once at the training ranges, the rest mid-range.
EVAL_TABLE_SEED = 20261023
EVAL_PLATES_PER_KIND = 12
EVAL_STIFFNESS = ((8000.0, 0.0), (1000.0, 0.025))
EVAL_FORCES_N = (8.0, 16.0, 28.0)
EVAL_RAMP_S = 0.5
EVAL_HOLD_S = 3.0
#: The force is scored over the hold's last `EVAL_SCORE_S`: held within max(`EVAL_TOL_N`, `EVAL_TOL_SHARE` x level).
EVAL_SCORE_S = 2.0
EVAL_TOL_N = 3.0
EVAL_TOL_SHARE = 0.2
EVAL_EPISODE_S = 17.0
EVAL_SEED = 20261022
#: The "slide" set (2026-10-22 02:40): the same 48 plates, rigid, 16 N, each pressed twice -- with a tangential part of
#: `EVAL_TANGENT_RATIO` of the level one way along the plate, and sliding `EVAL_SLIDE_M` at `EVAL_SLIDE_SPEED_M_S` while
#: the level is held; the way drawn once per plate.
EVAL_SLIDE_LEVEL_N = 16.0
EVAL_TANGENT_RATIO = 0.25
EVAL_SLIDE_M = 0.04
EVAL_SLIDE_SPEED_M_S = 0.025
#: The "dog" set (2026-10-23): the live program's two hands (`demos/combiner_key/robot/press_layer.PLATES`, rows of the
#: training table: palm_up, the palm facing up under the foot; palm_facing, the palm 0.42 m ahead facing the dog), rigid
#: and sprung (`EVAL_STIFFNESS`) at the program's 5 N cap and 8 N, `EVAL_DOG_REPEATS` times each with their own estimate
#: errors: 288 presses.
EVAL_DOG_ROWS = (14027, 10851)
EVAL_DOG_LEVELS_N = (5.0, 8.0)
EVAL_DOG_REPEATS = 36
KINDS = ("tilted", "wall", "level_up", "level_down")

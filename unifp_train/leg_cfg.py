"""Constants for the front-right-leg task (`leg_env.py`): the foot's goals, its pushes, the arm's held camera pose, the
door handles, the task layer, the rewards and the frozen evaluation.

The task is Lukas's of 2026-10-19: train UniFP's position/force formulation with the Go2's front-right *foot* as the
end effector instead of the D1's hand, standing on the other three legs while the arm holds the wrist camera on the
foot's workspace, and see whether it can turn a lever door handle at the Go2's own body height. Two policies are
trained the same way and differ only as the plan's P0 and P1 do (`docs/thesis_b_plan.md`, "Controlled training and
testing"): **position** gets no force command, no force-tracking target, no force in its critic and no force in its
estimator's target; **force** gets UniFP's: a commanded force, an external one it is not told about, the foot's target
displaced by both through a virtual spring, and the estimator supervised on the external force.

Every number here is a choice, not a measurement, unless it says otherwise.
"""
from __future__ import annotations

import math

from unifp_isaaclab import interface

# --- the robot ------------------------------------------------------------------------------------------------------

#: The controlled point: the centre of the front-right foot's collision sphere (`handle.FOOT_CENTRE_LOCAL_M`, from
#: `description/go2_d1.urdf`).
FOOT_BODY = "FR_foot"
KNEE_BODY = "FR_calf"
#: Leg indices in UniFP's DOF order: FR is joints 3-5, foot 1 of (FL, FR, RL, RR).
FR_JOINTS = (3, 4, 5)
FR_FOOT_INDEX = 1
SUPPORT_FEET = (0, 2, 3)
#: The support legs' hips, for `hip_pos` (FR's hip has to move to reach sideways).
SUPPORT_HIP_INDICES = (0, 6, 9)

#: The arm's held pose, Joint1..Joint6 (rad): the arm folded upright, the wrist pitched forward, the camera high and
#: looking down at the handle region. Chosen on 2026-10-20 by a search over the CPU kinematics
#: (`position_only.workspace.forward`) and the wrist camera's CAD mount (F-138) for the pose that keeps seven points of
#: the handles this task draws (pivots, bar ends, pre-contact points, a turned contact) furthest inside a 640 x 480
#: image at fx 615, the shoulder and elbow lightly loaded; then checked in Isaac Lab on 8 drawn handles after a 2 s
#: settle: joints within 0.025 rad of target, Joint2 / Joint3 holding 0.66 / 0.95 N·m (20% / 29% of 3.33), the camera
#: 0.89 m up at 0.6-0.77 m from the handle points, every point inside the image (u 170-394, v 105-249). The first
#: choice (-0.30, -0.25, 0.75, 0, 0, 0), from a search that assumed a 0.30 m base, put every handle above the image.
ARM_LOOK_POSE = (-0.30, -0.20, -0.60, 0.0, 1.30, 0.0)
#: Per-episode jitter on that pose (rad, uniform +-), so the legs are not trained against one arm posture.
ARM_POSE_JITTER = (0.15, 0.10, 0.15, 0.0, 0.15, 0.0)

#: The support legs' soft position limits are UniFP's 0.8 of range; the front-right leg's are 0.95, because reaching
#: forward at hip height needs the thigh near -1.1 rad, inside the hard -1.571 but past 0.8's -1.06.
SOFT_DOF_POS_LIMIT_FR = 0.95

# --- the foot's goals in free space ---------------------------------------------------------------------------------
#
# UniFP's goal generator with the arm's sphere moved to the front-right shoulder: a goal drawn in (radius, pitch, yaw)
# about `GOAL_CENTRE`, in the base's yaw frame from the ground under the base, slid to over `TRAJ_TIME_S` and held for
# `HOLD_TIME_S`. The three command channels and their scales are UniFP's, so no observation width changes.

GOAL_CENTRE = (0.19, -0.14, 0.28)          # the FR thigh joint at the standing height
GOAL_RADIUS_RANGE = (0.16, 0.38)
GOAL_PITCH_RANGE = (-0.95, 0.55)
#: Yaw to the left stops at +0.25: past +0.2, 39% of drawn goals were more than 3 cm from anything the leg reaches with
#: the base level (a kinematic sample of the FR leg, 2026-10-20), against 15% overall.
GOAL_YAW_RANGE = (-0.65, 0.25)
TRAJ_TIME_S = (1.0, 2.5)
HOLD_TIME_S = (0.5, 2.0)
#: The opening goal: just ahead of where the foot stands, so the first move is the lift.
INIT_GOAL = (0.24, -1.20, 0.0)
#: Goals must keep clear of the ground, the trunk and the other front leg (yaw frame, from the ground under the base).
GOAL_MIN_HEIGHT_M = 0.06
GOAL_MAX_LEFT_M = 0.02
GOAL_MIN_AHEAD_M = 0.12
GOAL_TRUNK_BOX = ((-0.30, 0.30), (-0.13, 0.13), (0.15, 0.50))
#: Of the goals drawn, this share puts the foot back down at its standing footfall (+- `DOWN_JITTER_M`), so the
#: policy also learns to stand on four legs again.
DOWN_GOAL_PROB = 0.12
DOWN_GOAL = (0.19, -0.14, 0.022)
DOWN_JITTER_M = 0.03

# --- UniFP's pushes, at the foot ------------------------------------------------------------------------------------

#: Peak force per axis, N (UniFP's schedule otherwise: `task_cfg.PUSH_*`). The arm's band is +-8 N because the D1 makes
#: ~7 N; the leg makes tens of newtons at the foot, and the handles below need up to ~40.
FOOT_FORCE_RANGE_N = (-35.0, 35.0)
#: The virtual spring the force-aware target is displaced through, N/m: `goal + (external + command) / k`. UniFP uses
#: 200 for the arm's +-8 N (4 cm); at the foot's +-35 N that would be 17 cm a side, so it is doubled.
FOOT_FORCE_KP = 400.0

# --- episodes -------------------------------------------------------------------------------------------------------

EPISODE_LENGTH_S = 16.0
#: Base height and orientation limits that end an episode (UniFP's tilt limits, plus a floor for the trunk).
MIN_BASE_HEIGHT_M = 0.14
#: `--obs_noise` (training, the actor's frame only): uniform noise of these half-widths on the raw quantities, before
#: UniFP's scales -- unitree_rl_lab's Go2 velocity task (`tasks/locomotion/robots/go2/velocity_env_cfg.py`, the
#: policy group's Unoise; Isaac Lab adds an observation's noise before its scale), which the walking policy that runs
#: on the dog (rima's) trained with. Its gravity vector's 0.05 stands here on UniFP's roll and pitch (rad).
OBS_NOISE = {"roll_pitch": 0.05, "ang_vel": 0.2, "dof_pos": 0.01, "dof_vel": 1.5}
#: `tilt_excess` (off unless weighted, `run_unifp_train.py --tilt_excess_weight`): the tilt from upright it starts at.
#: 2026-10-23: the foot-press policy sat back 17-25 degrees nose up within 0.3 s of every start, waiting, in simulation as
#: on the dog (F-188), where the live program stops at 15.
TILT_EXCESS_RAD = 0.1745

# --- the door handles -----------------------------------------------------------------------------------------------
#
# A lever handle at the Go2's body height on a door ahead of it, drawn in the base's yaw frame at reset (from the
# ground under the base). The lever points left or right of its spindle; the foot pushes its outer part down.

#: The door's face, metres ahead of the base (its normal faces the robot, turned by `DOOR_YAW_RAD`). 0.54-0.62 until
#: 2026-10-20 03:30: the robot settles ~4 cm behind where it spawns, and from there 48% of the frozen set's
#: pre-contact points lay more than 1 cm beyond anything the FR leg reaches with the base level (a kinematic sample).
DOOR_AHEAD_M = (0.46, 0.54)
DOOR_YAW_RAD = (-0.25, 0.25)
#: The spindle's height above the ground and its sideways position (yaw frame).
SPINDLE_HEIGHT_M = (0.26, 0.34)
#: Where the contact point (`REACH_FRACTION` of the lever out) lands sideways. -0.20 to -0.06 until 2026-10-20 03:30:
#: at hip height the FR foot cannot come inboard of y ~ -0.10 (its leg plane is 9.55 cm outboard of the hip's ab/ad
#: axis and the hip stops at 60 degrees), so contact points at -0.08 were out of reach by 4-7 cm.
CONTACT_SIDE_M = (-0.22, -0.11)
#: Share of levers pointing to the robot's left of their spindle (the rest point right).
LEVER_LEFT_PROB = 0.5
LEVER_LENGTH_M = (0.10, 0.14)
LEVER_INNER_M = 0.03
LEVER_RADIUS_M = (0.008, 0.012)
LEVER_STANDOFF_M = (0.05, 0.065)
LEVER_INERTIA_KG_M2 = (5e-4, 2e-3)
#: Return spring: torque at rest (preload) and at the end stop, N·m. Domestic lever sets need roughly 0.5-2.5 N·m;
#: up to 4 so the leg's strength is exercised.
REST_NM = (0.2, 0.9)
FULL_NM = (0.8, 4.0)
SPINDLE_DAMPING = (0.005, 0.03)
SPINDLE_FRICTION_NM = (0.02, 0.12)
THETA_MAX_RAD = (math.radians(35.0), math.radians(60.0))
#: The latch is drawn back once the lever is this share of its travel down.
LATCH_FRACTION = (0.60, 0.85)
FOOT_MU = (0.4, 0.9)
#: Where along the lever the layer aims the foot, as a share of the estimated length.
REACH_FRACTION = 0.78
#: The layer aims the foot this far off the bar's centreline toward the robot; negative is toward the door. +0.008 at
#: first, away from the door; but the policy's foot runs 2-3 cm toward the robot of its goal as it descends, met the
#: bar in front of its centreline and was pushed off it toward the robot. Aimed door-side, an overshoot is caught by
#: the door's face with the foot on the bar's door-side shoulder, and the usual undershoot still lands on the bar.
#: Its scripted approach comes down onto the pre-contact point from this far up and back.
AIM_TOWARD_ROBOT_M = 0.0           # -0.012 for one stage-2 start: the held foot, 2-3 cm door-side of its goal, met the door
#: Since 2026-10-20 07:00 the waypoint is 12 cm back: the policy swings the lifted foot up to 10 cm ahead of its goal,
#: and a path to a waypoint 4 cm back crossed the bar (49 N and 35 N on the lift, then damage).
WAYPOINT_UP_M = 0.06               # 0.05, and 0.12 back, until recipe j (2026-10-20 10:45)
WAYPOINT_BACK_M = 0.15
#: The lift's first leg rises straight up to this height (m, the foot sphere's centre), over this share of `LIFT_S`.
LIFT_FIRST_HEIGHT_M = 0.14
LIFT_FIRST_SHARE = 0.3
#: The scripted approach's two slides, s: to the waypoint (1.2 until 07:10, the foot kicked ahead), then onto the point.
LIFT_S = 2.0
OVER_S = 0.8

# --- what the task layer is told, and how wrong it is ----------------------------------------------------------------

#: Error of the handle's estimate (uniform +-): the pivot sideways along the door and up, and in depth (along the
#: door's normal); the lever's length; the stop angle; the rated torque.
EST_ERROR_SIDE_M = 0.015
EST_ERROR_UP_M = 0.010
EST_ERROR_DEPTH_M = 0.008
EST_ERROR_LENGTH = 0.10
EST_ERROR_THETA = 0.10
EST_ERROR_RATING = 0.30

# --- the task layer -------------------------------------------------------------------------------------------------

#: The pre-contact point is this far above the estimated bar's resting top (0.05-0.08 until 2026-10-20 03:50).
DESCENT_M = (0.04, 0.06)
DESCEND_SPEED_M_S = (0.04, 0.07)     # 0.08-0.15 until 2026-10-20 04:10: the foot's bias moved faster than the loop
TURN_SPEED_RAD_S = (0.6, 1.2)
#: Past the estimated end stop, so a stop that is further than estimated is still reached.
OVERDRIVE_RAD = math.radians(6.0)
HOLD_S = 2.0
RELEASE_S = 1.0
#: The outer loop on the foot's measured position (both variants): gain 1/s, and the most it may shift the goal, m.
FOOT_CORRECTION_GAIN = 2.0           # 4.0 for one stage-2 start (2026-10-20 06:00): ~99% of approaches never settled within
#: 1.5 cm -- an integral loop at 4/s round a policy that takes ~0.35 s to follow a moved goal circles rather than settles
FOOT_CORRECTION_MAX_M = 0.07
#: The force law (force-aware only): N per m of arc lag, N per m·s of its integral; a light touch force.
FORCE_LAW_KP_N_PER_M = 500.0
FORCE_LAW_KI_N_PER_M_S = 1500.0
TOUCH_N = 4.0
#: The reference waits this long at the bar's top, the force law capped at `TOUCH_N`, before it turns the lever.
TOUCH_HOLD_S = 0.6
#: Its cap: this multiple of the rated force at the stop (rated torque / estimated reach), within the limits.
MAX_MARGIN = 1.8
#: 60 N at most until 2026-10-20 08:30; stage 1 trained the command channel to +-35 N a side.
MAX_N_LIMITS = (12.0, 40.0)
#: The layer starts once the foot has settled near the pre-contact point: closer than this and slower than that, or
#: at the deadline if within `ENGAGE_NEAR_M`; a foot not there by the deadline gets no handle this episode.
ENGAGE_CONVERGED_M = 0.02
ENGAGE_SETTLE_SPEED_M_S = 0.10
ENGAGE_NEAR_M = 0.04
ENGAGE_MAX_WAIT_STEPS = 150
ENGAGE_DELAY_STEPS = (10, 30)

# --- success and failure (both policies, the plan's common limits) --------------------------------------------------

#: Success: the latch held drawn back this long without a break, no damage, no fall.
SUCCESS_OPEN_S = 1.0
#: Damage: any contact force on the foot or knee above this, at the physics rate. The leg can make it; a handle should
#: not need it, and a door handle loaded like this is being forced rather than turned.
DAMAGE_N = 120.0
#: A slip-off: in contact with the bar after touching it, then off it for longer than this while pressing or holding.
SLIP_S = 0.1

# --- rewards --------------------------------------------------------------------------------------------------------

#: Weights before the `dt` multiply, as `task_cfg.REWARD_WEIGHTS` holds UniFP's.
REWARD_WEIGHTS = {
    # the objectives
    # UniFP's end-effector term on the foot, exp(-2 |e|_1), target as the variant says. 2.0 in the first run
    # (2026-10-20, v1): so broad that a lifted foot anywhere earns half of it, and tracking sat at ~1.0 of 2.0 from
    # iteration 80 to 328 with the foot ~0.2 m off its goal. Kept, lowered, with a 12 cm term carrying the gradient.
    "foot_tracking": 1.0,
    "foot_tracking_mid": 3.0,      # exp(-|e|_2 / 12 cm), since v2
    "foot_tracking_fine": 2.0,     # exp(-|e|_2 / 4 cm): what makes the foot precise enough for a 2 cm bar
    "alive": 1.0,
    "support_contact": 1.0,        # the three support feet down; FR clear of the ground unless its goal is down
    "foot_lift": 1.5,              # the foot raised toward its goal's height (v3)
    "base_still": 1.0,             # UniFP's velocity term at zero command
    "base_yaw_still": 0.5,
    # regularisation (UniFP's weights where the term is UniFP's)
    "lin_vel_z": -1.5,
    "ang_vel_xy": -0.05,
    "orientation": -1.0,
    "base_height": -2.0,
    "collision": -5.0,
    "dof_pos_limits": -10.0,
    "torque_limits": -0.005,
    "torques": -5.0e-6,
    "dof_vel": -8.0e-4,
    "dof_acc": -2.5e-7,
    "action_rate": -0.02,
    "support_slip": -1.0,
    "support_hip_pos": -0.5,
    "feet_contact_forces": -0.001,
    "dof_vel_arm": -2.0e-4,
}
#: Added in episodes with a handle, for both policies: how far the lever is down toward the latch, the latch held
#: drawn back, losing the bar after touching it, and leaning on the door's face (per N).
HANDLE_WEIGHTS = {
    "handle_contact": 1.0,         # the foot on the bar while the layer presses or holds (geometric)
    "handle_progress": 1.5,
    "handle_open": 2.0,
    "handle_slip": -3.0,
    "door_push": -1.0,             # per cm of overlap with the door's face (foot or knee)
    "approach_contact": -2.0,      # per cm of overlap with the bar or the door before the press (recipe j)
}
TRACKING_SIGMA = 0.25
FOOT_FINE_SIGMA_M = 0.04
FOOT_MID_SIGMA_M = 0.12
#: The lifted foot counts as clear of the ground with its sphere's centre this high (2.8 cm of clearance).
FOOT_CLEAR_M = 0.05
BASE_HEIGHT_TARGET = 0.29
MAX_CONTACT_FORCE = 150.0

# --- the frozen evaluation ------------------------------------------------------------------------------------------

EVAL_EPISODE_S = 11.0
#: 3 door distances x 3 contact sides x 2 lever directions x 4 spring strengths x 2 heights = 144 handles; the
#: estimate's errors drawn from a fixed seed at the training ranges, the rest at their mid-range.
EVAL_DOOR_AHEAD_M = (0.47, 0.50, 0.53)
EVAL_CONTACT_SIDE_M = (-0.20, -0.16, -0.12)
EVAL_LEVER_LEFT = (False, True)
EVAL_FULL_NM = (1.0, 2.0, 3.0, 4.0)
EVAL_SPINDLE_HEIGHT_M = (0.28, 0.32)
EVAL_SEED = 20261020


def scaled_weights(names: dict[str, float]) -> dict[str, float]:
    return {name: weight * interface.POLICY_DT for name, weight in names.items()}


# === the combiner box's key (Lukas, 2026-10-21) ======================================================================
#
# The second object for the foot: Lukas's lever key (`combinerbox_lever v3.stl`, `assets/combiner_key/`) in the box's
# quarter-turn cam lock, turned by the front-right foot from 45 degrees above horizontal (its flat top facing up) to 45
# degrees below, clockwise as the robot sees it; the door unlocks past 30 degrees below. In the handle's convention
# (`handle.py`: theta from horizontal, positive down, about the door's normal) the key turns from -45 to +45 degrees
# and unlocks at +30. `--task leg --leg_object key`. Everything about the stance changed with it (below), because the
# handle policies Lukas watched kept their feet where they had put them while the robot dropped from its spawn.

# --- the stance -------------------------------------------------------------------------------------------------------

#: The D1 folded in its rest pose against its mechanical stop (`d1_ik.from_servo_deg(d1_hardware.REST_SERVO_DEG)`), as
#: the combiner door demo starts it: the camera is on the head now, so the arm only has to stay out of the way. Its
#: jitter leaves Joint2 alone (it rests on its stop).
ARM_REST_POSE = (-0.0367, -1.5865, 1.6005, 0.0541, 0.0716, 0.0541)
ARM_REST_JITTER = (0.10, 0.0, 0.05, 0.10, 0.10, 0.10)
#: The robot standing settled under zero actions with the arm at rest (Isaac Lab, 2026-10-21, 8 robots after 6 s): the
#: base 0.272 m up, level to 0.3 degrees, the knees bent to -1.80 / -1.83 rad under load (their targets are -1.5). The
#: key task's episodes start here instead of dropping from UniFP's 0.35 m spawn: the handle policies used the 0.1 s
#: their feet were in the air to put the right-rear foot 13-20 cm forward under the body, and support_slip then kept
#: it there (`stance_probe`, 2026-10-21).
SETTLED_BASE_HEIGHT_M = 0.272
SETTLED_LEG_POS = (0.1112, 0.7431, -1.8014, -0.1108, 0.7434, -1.8007,
                   0.1045, 0.9905, -1.8329, -0.1037, 0.9905, -1.8324)
#: Where the feet stand then, in the base's yaw frame from the base (x forward, y left), m: FL, FR, RL, RR.
NOMINAL_FEET_B = ((0.236, 0.170), (0.236, -0.1705), (-0.2115, 0.168), (-0.2115, -0.168))
#: The front-right foot's standing footfall, from the ground under the base (the handle task's `DOWN_GOAL` is 4.5 cm
#: behind and 3 cm inboard of it).
KEY_DOWN_GOAL = (0.235, -0.1705, 0.022)
#: `support_stance`: exp(-(d / sigma)^2) per support foot, d its horizontal distance from its nominal place.
STANCE_SIGMA_M = 0.06
#: 2.0 for the first stage-1 start (2026-10-21 00:47, stopped at 437 when its position-only twin had died of the shared
#: GPU's memory): the support feet's median distance from their places fell to 2.4 cm, but in each robot one foot still
#: stood ~15 cm off -- it stepped it to make its tripod rather than leaning its body over the other three.
STANCE_WEIGHT = 4.0
#: Free-space goals reach up to the key: its pad, 0.37-0.51 m up at 0.25-0.3 m ahead of the FR thigh joint, is at 0.3-0.7
#: rad of pitch from the goal centre; the handle task's 0.55 stops short of it.
KEY_GOAL_PITCH_RANGE = (-0.95, 0.80)

# --- the box and the lock ---------------------------------------------------------------------------------------------

#: The cam lock's axis above the floor: midway up the box's free (left) edge, `demos/combiner/geometry.GEOMETRY.lock`
#: (0.4175 m; the lock's height is from Lukas's photo, its distance from the edge measured). Training draws it +-1.5 cm
#: (the floor, the robot's stance height).
KEY_AXIS_HEIGHT_M = 0.4175
KEY_AXIS_HEIGHT_JITTER_M = 0.015
#: The key's geometry, from the STL (metres; Lukas's v3 print): a 10 mm round shaft with two wards 15 mm across, 10 mm
#: long into the lock; a bar 15 mm thick (along the shaft) and 20 mm tall about the shaft's axis, from 15 mm behind the
#: axis to 160 mm out; and a 2 mm plate 35 mm wide past the bar from 48 to 148 mm out, its top flush with the bar's.
#: The flat top -- 100 mm along the key, 50 mm off the door -- is the platform the foot presses.
KEY_SHAFT_LENGTH_M = 0.010
#: The slot, adjusted so the key slides in (`demos/combiner_key`): a printed lock adapter over the cam lock, 16 mm thick
#: and 1 mm off the door, its keyway the key's profile plus 1 mm a side behind a lead-in. A seated key's bar stands on
#: the adapter's face, so the key is 17 mm further off the door than it would be in the bare lock.
KEY_ADAPTER_FACE_M = 0.017
KEY_BAR_BOX = ((-0.015, 0.160), (0.017, 0.032), (-0.010, 0.010))     # (along the key, off the door, up its top side)
KEY_PAD_BOX = ((0.048, 0.148), (0.032, 0.067), (0.004, 0.010))
KEY_TOP_M = 0.010                    # the flat top above the axis
KEY_MASS_KG = 0.053                  # 43 cm^3 of PLA
KEY_COM_M = 0.069                    # its centre of mass along the key from the axis (the STL's)
KEY_INERTIA_KG_M2 = (4e-4, 1.2e-3)   # the key's ~4e-4 about the shaft, plus the cam's
#: The turn: from 45 degrees up to 45 degrees down (the end stop), unlocking past 30 degrees down. Not the turn Lukas asked
#: for, which starts horizontal (F-176): `run_unifp_train.py --key_turn_deg 0 45 30`. Kept as the default because every
#: key checkpoint up to F-178 trained on it.
KEY_THETA_START_RAD = math.radians(-45.0)
KEY_THETA_END_RAD = math.radians(45.0)
KEY_THETA_UNLOCK_RAD = math.radians(30.0)
#: The lock's resistance, N·m: a Coulomb friction that sticks below it (no return spring: a cam lock stays where it is
#: turned), plus a cam detent over the 20 degrees before the unlock. Not measured: a guess at a stiff quarter-turn lock,
#: to 3.5 N·m in all so the leg's strength is exercised (35 N at the pad's middle).
KEY_FRICTION_NM = (0.3, 2.5)
KEY_DETENT_NM = (0.0, 1.0)
KEY_DETENT_SPAN_RAD = math.radians(20.0)
KEY_DAMPING = (0.005, 0.02)
KEY_MU = (0.5, 0.9)                  # the foot's rubber on the printed top
#: Where the layer puts the foot: along the key (the pad runs 48-148 mm out), and off the door's face (the pad's middle is
#: 49.5 mm off it on the adapter; the foot sphere then clears the face by 28 mm). Inboard of the pad's middle, because at the start
#: the key points 45 degrees up and its pad's middle puts the foot's centre 0.51 m up, 0.24 m above the FR thigh joint
#: of the settled stance: about what the leg reaches with the base level (0.21 at the thigh's -90 degree stop).
KEY_REACH_M = (0.075, 0.110)
KEY_AIM_OFF_DOOR_M = 0.050
#: The robot's stance against the lock, in the base's yaw frame at reset (from the ground under the base): the door's
#: face ahead, and the lock's axis to the side, so the pad's middle lies in front of the FR foot. The FR foot's plane
#: is 0.17 m to the right; the pad's middle is 0.07-0.10 m right of the axis over the turn.
KEY_DOOR_AHEAD_M = (0.40, 0.47)     # the head's front at ~0.36: from 4 cm clear; the knee has to stay off the face
KEY_DOOR_YAW_RAD = (-0.15, 0.15)
KEY_AXIS_SIDE_M = (-0.11, -0.05)
#: The estimate the layer is told (the combiner door demo's look places the box to a median 2.2 mm and 0.12 deg, F-155;
#: these are ~4x that), and the resistance it is told, wrong by +-30%.
KEY_EST_ERROR_SIDE_M = 0.010
KEY_EST_ERROR_UP_M = 0.010
KEY_EST_ERROR_DEPTH_M = 0.008
KEY_EST_ERROR_RATING = 0.30
#: The key's own weight about its shaft at horizontal, N·m (it helps the push).
KEY_GRAVITY_NM = KEY_MASS_KG * 9.81 * KEY_COM_M
#: After the release the foot goes back to its footfall: to the waypoint, then down.
RETURN_UP_S = 0.8
RETURN_DOWN_S = 1.2
KEY_EPISODE_LENGTH_S = 18.0
KEY_EVAL_EPISODE_S = 18.0

# --- the key's frozen evaluation --------------------------------------------------------------------------------------

#: 3 door distances x 3 axis sides x 3 lock frictions x 2 detents x 2 heights = 108 keys; the estimate's errors drawn
#: once from `KEY_EVAL_SEED` at the training ranges, the rest at mid-range.
KEY_EVAL_DOOR_AHEAD_M = (0.41, 0.435, 0.46)
KEY_EVAL_AXIS_SIDE_M = (-0.10, -0.08, -0.06)
KEY_EVAL_FRICTION_NM = (0.5, 1.5, 2.5)
KEY_EVAL_DETENT_NM = (0.0, 1.0)
KEY_EVAL_AXIS_HEIGHT_M = (0.405, 0.430)
KEY_EVAL_SEED = 20261021
#: The key's approach: the pre-contact point this far off the platform along its normal (up and toward the shaft at
#: the start), and the waypoint before it toward the robot and a little down -- not the handle's 15 cm back and 6 cm
#: up, which at the key's height is above anything the leg reaches: the FR foot rises highest only well forward of its
#: thigh joint (0.21 m up at 0.21 m ahead with the thigh on its -90 degree stop, 0.14 m up at 0.05 m ahead).
KEY_DESCENT_M = (0.02, 0.035)
KEY_WAYPOINT_BACK_M = 0.07
KEY_WAYPOINT_UP_M = -0.01
#: The force-aware layer's cap on its command for a key: 1.8x the rated force, within these (the stage-1 command channel
#: is +-35 N a side, ~60 N along a diagonal).
KEY_MAX_N_LIMITS = (15.0, 55.0)
#: The robot's head against the door's face, as a sphere in the base frame (the Go2's head colliders reach x 0.34; the
#: head RealSense's case ~0.36): the analytic plant checks the foot, the knee and this.
HEAD_POINT_B = (0.30, 0.0, 0.03)
HEAD_RADIUS_M = 0.065

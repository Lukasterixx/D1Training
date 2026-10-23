"""The combiner door on the real Go2 and D1, as numbers: the attempt `node.py` runs, with no ROS in it.

It is the simulated demo's sequence (`demos/combiner_door/env.py`) on the same modules: the look and its fix
(`perception.lidar_estimate`: the tag for where, the MID-360's face fit for which way), the door script
(`demos.unifp.script.door_phases`), the arm's inverse kinematics (`decoupled_arm.ArmIK`), the lidar's door reading
(`mid360.door_angle`). What differs is the robot:

- **The legs are Unitree's sport mode**, not the clocked-gait policy the simulator walks with (that policy has only ever
  run in Isaac Sim). P2Dingo measured sport mode on this dog (its CLAUDE.md, 2026-10): standing still at 0.12-0.15 m/s
  forward, walking at 0.18-0.21, not turning on the spot at 0.30-0.42 rad/s, not side-stepping at 0.25 m/s, and
  carrying on ~75 mm after a stop from 0.21 m/s. So a command is either zero or lifted to those floors
  (`sport_command`, as P2Dingo's docking lifts its own), and the walk in stops early by the carry (`CARRY_S`).
- **The odometry is the robot's own**: sport mode's leg-and-IMU estimate, or GLIM's when P2Dingo's stack runs. Not the
  simulator's truth. After the walk in the lidar fits the door again from the stance (`refine_fix`), which takes the
  walk's odometry error out of the box's distance and heading.
- **The arm is the D1**, through `arm_link.py`: a smooth 10 Hz command towards each step's goal (`arm_link.Profile`),
  never far ahead of where the arm last reported itself. It starts and ends folded in its rest pose
  (`d1_hardware.REST_SERVO_DEG`). A phase that only waits for the arm ends when the arm is there (`DWELL_PHASES`), and
  the hand lets go of the lever from where it measures itself to be, straight back off it (`ungrasp_phases`).
- **An attempt stops at a stage** (`STAGES`), backs off and folds, so it can be tried one step at a time: look, unfold,
  walk in, reach, grip, pull.

Frames. `odom` is the odometry's. The look's frame is the base where it stood (yaw only). The script's frame is the
door pull's stance: straight ahead of the look, where the box's root is `NOMINAL_DISTANCE_M` away (the simulated
placements' middle), so the operator need only stand the dog roughly in front of the box. Its z is set so the box's
root is at the simulated plinth's height (`props.POST_TOP_M`), so every height the door script and the box geometry use
is the box's own, whatever the floor. The base's height is a constant (`BASE_HEIGHT_M`): it cancels, since the box is
placed from the base through the camera. Poses are 4x4 matrices, points (n, 3) arrays. numpy only.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

import d1_hardware
import d1_ik
from demos.combiner.apriltag import box_pose
from demos.combiner.geometry import GEOMETRY
from demos.combiner_door import front_camera, head_camera, mid360, perception
from demos.unifp import decoupled_arm, props, script as demo_script

# --- the robot ------------------------------------------------------------------------------------------------------

#: The MID-360's points in `base_link`: P2Dingo's static `livox_frame -> base_link` (-0.176, 0, -0.140), level
#: (`split/start_p2dingo.sh`). The simulator's mount (`mid360.ORIGIN_BASE_M`) agrees to 3 mm.
LIDAR_IN_BASE_M = (0.176, 0.0, 0.140)
#: The base's height above the floor, standing. Only gives poses a z; it cancels (module docstring).
BASE_HEIGHT_M = 0.32
#: The head camera's optical frame in `base_link`: P2Dingo's mount (`GO2_REALSENSE_MOUNT`, the colour sensor), level.
CAMERA_POSE_BASE = head_camera.optical_pose_base()
#: The Go2's own front camera's optical frame in `base_link` (`front_camera`: the URDF's mount): the look's camera since
#: 2026-10-22 (`node.py --camera front`, the default). Fixed in the trunk, so its frames need no arm pose, and the look no
#: arm move. Its lens model is NOMINAL until calibrated (`front_camera.NOMINAL`).
FRONT_CAMERA_POSE_BASE = front_camera.optical_pose_base()
#: The nearest the box's root can be for the front camera to see the whole tag, standing (the base 0.30-0.34 m up and
#: pitched +-2 deg, the tag within 0.25 m of straight ahead and 15 deg of square, 20 px to spare): its vertical field is
#: only +-27 deg (`front_camera.ONE_STAND`), and the tag, 0.46 m up the box, leaves the top of the image closer in. The
#: door is then ~0.79 m from the base. Every look must start at least this far back: the door pull's look band is
#: 0.95-1.14 m (the walk in is at most `WALK_IN_RANGE_M`), its no-walk band (0.65-0.76 m) cannot be looked from, nor
#: the key's 0.6 m from the door.
FRONT_LOOK_MIN_ROOT_M = 0.95
#: The camera on the D1's wrist instead (Lukas moved the D435i there, 2026-10-21): its colour optical frame in Link6,
#: the bracket's CAD (`demos/cup/pick_demo/assets/mounts/wrist_mount.json`: 30 degrees below the approach axis; not
#: measured on the arm). Its pose in the base comes from the arm's measured joints at each frame (`wrist_camera_base`).
WRIST_MOUNT_LINK6 = None
#: Where the arm holds the wrist camera for the look (`look_pose_servo_deg`): its optical centre this far (ahead, left,
#: up) of the base, looking at `LOOK_AT_BASE_M` with the image's rows level. 4 cm left, the camera's own offset off the
#: arm's plane: the arm then reaches it nearly in its plane (Joint1 -3, Joint4 -5, Joint6 +3 degrees; centred it needs
#: Joint1 +45, the CPU model, 2026-10-21). Aimed low enough that the tag is in the upper half of the image (rows 50-370
#: of 720 with the box's root 0.9-1.3 m ahead, 335-546 at 0.75): the fingers stand in the lower right of the wrist
#: camera's view whatever the arm does (the bracket's 30 degrees), and the jaws are shut while it looks. Aimed at the
#: tag's height, the first live look had a finger across it and found it in 0 of 26 frames (2026-10-21).
LOOK_CAMERA_IN_BASE_M = (0.25, 0.04, 0.40)
LOOK_AT_BASE_M = (0.80, 0.04, 0.05)

#: Sport mode's floors and caps (module docstring). The lateral and yaw floors are where P2Dingo's docking moved it.
WALK_FLOOR = (0.18, 0.20, 0.45)        # vx m/s, vy m/s, wz rad/s
WALK_CAP = (0.25, 0.25, 0.60)
#: How far the base carries on after a stop, as seconds at the speed it was going: 75 mm from 0.21 m/s.
CARRY_S = 0.36
WALK_IN_MPS = 0.20
#: Close enough to the path's pose to stand still rather than lift a command to the gait's floor.
HOLD_XY_M = 0.03
HOLD_YAW_RAD = math.radians(4.0)

#: The arm link moves its command smoothly towards the goal this sends it (`arm_link.Profile`): at most this speed on
#: its longest joint, accelerating and slowing at `arm_link.ACCEL_DEG_S2`, never more than `arm_link.LEAD_DEG` ahead of
#: the arm. The Cartesian phases move at the door script's pace, well under it; the unfold and the fold, in joint
#: space, at `JOINT_SPEED_DEG_S`. The arm slews at ~70 deg/s (F-033). A dry run's arm is `arm_link.SimulatedArm`.
ARM_SPEED_DEG_S = 30.0
JOINT_SPEED_DEG_S = 40.0
ARM_STEP_DEG = ARM_SPEED_DEG_S * 0.1
#: When the IK cannot meet the hand's pose for a step it falls back to the jaw centre alone, a different wrist; this
#: many steps in a row the goal holds where it was instead (a missed step in a transit), so the arm does not lurch to
#: the other wrist and back.
POSE_MISS_HOLD_STEPS = 5
#: A joint-space move (the unfold, the fold) is over when the arm is this close to its goal on every joint; its phase
#: waits for that past its clock, up to `JOINT_TIMEOUT_S`, then aborts. On the clock alone the fold ended 23 deg short
#: of the rest pose (2026-10-20).
JOINT_ARRIVED_DEG = 2.0
JOINT_TIMEOUT_S = 20.0
#: Phases that only wait for the arm to get to a goal that stands still, and the joint moves: each ends as soon as
#: every joint is within `ARRIVED_DEG` of its goal on a report under `ARRIVED_REPORT_S` old, once it has run
#: `DWELL_MIN_S`; else on its clock as before. The arm's own steady-state error is 0.2 deg (mode 0, F-033). On the clock
#: alone the stand, the settle and the arrival took 7 s of the live pull's reach (2026-10-20).
DWELL_PHASES = {"stand", "settle", "arrive"}
ARRIVED_DEG = 1.0
#: The fold is the attempt's end, where the arm link then holds the arm where it is: it ends early only this close.
FOLD_ARRIVED_DEG = 0.5
ARRIVED_REPORT_S = 0.25
DWELL_MIN_S = 0.3
#: The jaws, from servo 6's report (units). The grip goes on past its clock until they stop closing (stalled on the
#: lever: under `JAW_STALL_UNITS` in `JAW_STALL_S`). The hand leaves the lever only once they have opened: within
#: `JAW_OPEN_TOLERANCE_UNITS` of the command (~1.7 mm a finger), or stopped more than half way open. Past
#: `JAW_WAIT_S` of waiting, the grip goes on and a release aborts, the hand where it is. Without the report, the clock.
JAW_STALL_UNITS = 0.5
JAW_STALL_S = 0.3
JAW_OPEN_TOLERANCE_UNITS = 4.0
JAW_WAIT_S = 2.0
#: The commandable fold: the rest pose with J1 and J2 at the limits the arm takes commands to (F-023, F-027).
FOLD_SERVO_DEG = tuple(float(np.clip(v, -90.0, 90.0)) for v in d1_hardware.REST_SERVO_DEG)
#: Gripper, servo units: the scripted grip's 63.2 mm opening and its shut, through `d1_hardware`'s linear map, which
#: is unmeasured between its ends (F-063). Shut clamps to the pads touching; on the 40 mm lever they stall near 6.8.
GRIPPER_OPEN_UNITS = d1_hardware.finger_travel_to_gripper_units(demo_script.JAW_LEVER_OPEN_M)
GRIPPER_SHUT_UNITS = d1_hardware.finger_travel_to_gripper_units(demo_script.JAW_BAR_SHUT_M)

# --- the attempt ----------------------------------------------------------------------------------------------------

#: How far an attempt goes. `watch` is no attempt: the look runs continuously and nothing is commanded.
STAGES = ("watch", "look", "unfold", "walk", "reach", "grip", "pull")
#: Seconds. The look is two seconds of the camera's 15 Hz and the lidar's 10 Hz with the robot standing.
LOOK_S = 2.0
#: The unfold and the fold: ~100 deg of J1 and J2, 3.2 s at `JOINT_SPEED_DEG_S`. Each ends when the arm is there
#: (`ARRIVED_DEG`), and past its clock waits for it (`JOINT_ARRIVED_DEG`).
UNFOLD_S = 5.0
FOLD_S = 5.0
#: The walk in's clock is its least: it ends when its steps are done (`Attempt._walk_in`), which takes longer.
WALK_IN_S = 0.5
#: Standing after the walk in (which ends with `WALK_IN_COAST_S` stood) while the lidar's frames for the refit come in.
WALK_SETTLE_S = 1.0
#: The unfold stage's pause, unfolded, and the grip stage's squeeze.
PAUSE_S = 1.0
#: Where the box must be from the stance, as the simulated demo placed it (`props.BOX_DISTANCE_RANGE_M`, the handle
#: 0.477-0.517 m from the robot; `BOX_BEARING_RANGE_DEG` +/-12; `BOX_YAW_JITTER_DEG` +/-8): outside it the attempt
#: refuses. The walk in sets the distance; one shorter than `WALK_IN_RANGE_M` is not walked, and the box must then
#: already be in range.
DISTANCE_RANGE_M = props.BOX_DISTANCE_RANGE_M
NOMINAL_DISTANCE_M = round(sum(DISTANCE_RANGE_M) / 2, 4)
#: Without a walk in, the box may stand this far off: the arm reaches the lever with the level grasp across it (on the
#: CPU model the level grasp solves with the base 0.10 m short of the stance to 0.04 m past it). Wider than the
#: simulated placements, which is all the door pull has been run through physics at; the record says when it is used.
#: A walk shorter than `WALK_IN_RANGE_M[0]` is below what sport mode can do (it carries on 75 mm after a stop).
ACCEPT_DISTANCE_M = (0.65, 0.76)
BEARING_LIMIT_DEG = max(abs(v) for v in props.BOX_BEARING_RANGE_DEG)
YAW_LIMIT_DEG = max(abs(v) for v in props.BOX_YAW_JITTER_DEG)
WALK_IN_RANGE_M = (0.10, 0.45)
#: The box's tag is printed on its door, so the lidar's face is where the tag is (`mid360.FaceFit.depth`, 0.6 mm in the
#: model). A tag this far in front of the face behind it is on something else: P2Dingo's charging dock carries the same
#: tag (36h11 id 0, 100 mm) on a stand in front of a wall.
FACE_BEHIND_TAG_LIMIT_M = 0.03
#: Nor is a face that carries on past the door's edges (0.315 m either side of the tag, 0.37 m above it): a wall. The
#: lidar's points on the face's line beyond them, as a fraction of those on it round the tag; the box against a wall
#: has that wall 0.3 m behind its door, off the line. P2Dingo's dock's stand stands 2 cm off its wall (2026-10-20).
BEYOND_EDGES_U_M = (0.40, 0.75)          # either side, along the face
BEYOND_TOP_DZ_M = (0.45, 0.75)           # above the tag's centre, within 0.25 m of it along the face
BEYOND_LIMIT = 0.25
#: Detections that put the box this close are one tag's (`make_fix`). The look's own scatter is centimetres; P2Dingo's
#: dock and the box stand metres apart in any room the two share.
TAG_GROUP_M = 0.25
#: The refit after the walk in may move the box this much and no more; past it something is wrong. 8 cm since the walk
#: in is sent short (`ODOM_WALK_SCALE`): the fix then puts the box 13% of the walk further than the dog will find it, and
#: the refit takes that out too (43 mm over a 0.33 m walk, 2026-10-21; 59 mm at the longest walk in, 0.45 m).
REFIT_LIMIT_M = 0.08
REFIT_LIMIT_DEG = 6.0
#: The walk in (`Attempt._walk_in`): straight at `WALK_IN_MPS`, its heading held, stopped early by the carry it has
#: seen (`CARRY_S` to begin with); after each stop it stands `WALK_IN_COAST_S`, measures how far the dog carried on,
#: and takes a corrective step (at least `WALK_IN_MIN_STEP_S` long, `WALK_IN_MAX_STEPS` of them) while it is still
#: more than `WALK_IN_TOLERANCE_M` short. In classic walk the first live walk in stopped 8 cm short, having carried on
#: next to nothing (2026-10-20). Then the attempt re-anchors where the dog stopped (`reanchor`) and checks the arm can
#: reach from there: wherever the walk ends, the box must be within `ACCEPT_DISTANCE_M` of the base.
WALK_IN_TOLERANCE_M = 0.03
#: Sport mode's odometry counts this share of a walk: the refit after the walk in put the door 12-14% of the walk nearer
#: than the look did in every door pull and key walk up on the dog (13 mm over 0.11 m to 63 mm over 0.44 m, 2026-10-20/21).
#: The walk in is sent this much shorter on the odometry, so that the dog walks the whole of it. Without it a 0.36 m walk
#: in left the box 0.63 m off, under `ACCEPT_DISTANCE_M` (Lukas's first door pull with the wrist camera, 2026-10-21).
ODOM_WALK_SCALE = 0.87
WALK_IN_COAST_S = 1.0
WALK_IN_MIN_STEP_S = 0.3
WALK_IN_MAX_STEPS = 3
WALK_IN_TIMEOUT_S = 20.0
WALK_IN_YAW_GAIN = 1.5
WALK_IN_YAW_CAP = 0.3
#: The pull (`Attempt._pull_twist`): the base and the door turn together about the hinge, which in the base's own
#: frame is one constant twist (the hinge stays where it is in it). It is streamed for the whole pull at the gait's
#: slowest usable speed (its largest part at its `WALK_FLOOR`, none past its `WALK_CAP`), never stopped and restarted,
#: and the script's door angle follows how far the base has actually turned, not the clock: so the hand's target stays
#: on the door's arc and nearly still relative to the base. On the clock, lifted to the floors only when behind, the
#: base and the arm moved in spurts and the door swung back and forth (2026-10-20). After the pull the base stands.
PULL_STALL_S = 2.0
PULL_STALL_DEG = 0.5
#: Through the hold the door goes on following the base's turn, as far as the gait carries it past the pull (at most
#: this much): the door is free and the hand holds it. Held at the pull's angle while the base carried on, the hand's
#: goal swung 6.6 deg at 30 deg and 8.2 deg at 60 deg in the dry model, near the loaded-arm check's 10.
HOLD_FOLLOW_DEG = 10.0
#: Phases the arm stays folded through, moves through in joint space, or holds the home point through (orientation
#: free).
JOINT_PHASES = {"unfold", "fold"}
POSITION_ONLY_PHASES = {"walk_in", "walk_settle", "pause", "home"}
#: Phases a live attempt run with `--step` waits at, for the operator to press Enter. Without it, one Enter starts the
#: attempt and it runs through.
CONFIRM_BEFORE = ("unfold", "walk_in", "approach", "grip", "pull", "release", "fold")
#: Phases from which the hand may be at the lever, so a recovery opens the jaws and backs out before it folds; from any
#: other, a recovery only folds.
AT_THE_LEVER_PHASES = {"approach", "settle", "enter", "arrive", "grip", "squeeze", "pull", "hold", "release", "ungrasp"}
#: Phases in which the hand holds the handle, and in which the base is walked (or may still be carrying on).
HOLDING_PHASES = {"grip", "squeeze", "pull", "hold"}
WALKING_PHASES = {"walk_in", "pull", "hold", "release", "ungrasp"}
#: Letting go (`ungrasp_phases`): the hand keeps the orientation it measured itself at as the release began.
RELEASE_HELD_PHASES = {"release", "ungrasp"}

#: Aborts: what an attempt will not carry on through.
ODOM_STALE_S = 0.3
#: The arm reports at ~9 Hz and has gone quiet for half a second while commanded (2026-10-20); the arm link only
#: commands on a fresh report, so a quiet arm is not driven blind.
ARM_STALE_S = 1.0
#: The arm loaded: while it holds the handle, a joint this far from where the IK wants it for `ARM_LAG_S`. Each command
#: is stepped from the arm's own pose (`ARM_STEP_DEG`), so the gap from the goal, not from the command, is the measure:
#: holding, the goal moves at the door's pace and an arm that follows stays within a step or two of it.
ARM_LAG_DEG = 10.0
ARM_LAG_S = 0.6
TILT_LIMIT_DEG = 12.0
#: The lidar's door against the script's, during the pull and hold: the door not following the hand.
DOOR_DISAGREE_DEG = 25.0
DOOR_DISAGREE_S = 1.0
#: The IK without a solution this long while holding.
IK_HOLD_S = 0.5


def wrist_mount() -> np.ndarray:
    """`WRIST_MOUNT_LINK6`, loaded once."""
    global WRIST_MOUNT_LINK6
    if WRIST_MOUNT_LINK6 is None:
        from demos.cup.pick_demo.camera import resolve_mount

        WRIST_MOUNT_LINK6 = np.asarray(resolve_mount(None)[0].pose, dtype=float)
    return WRIST_MOUNT_LINK6


def wrist_camera_base(joints, servo_deg) -> np.ndarray:
    """The wrist camera's colour optical frame in `base_link` (4x4), by forward kinematics of `servo_deg`."""
    q = d1_ik.from_servo_deg(servo_deg)[0]
    pos, rot = d1_ik.tool_pose(joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
    return pose(rot, pos) @ wrist_mount()


def look_pose_servo_deg(joints, links) -> np.ndarray | None:
    """The arm's joints (servo degrees) holding the wrist camera at `LOOK_CAMERA_IN_BASE_M`, aimed at `LOOK_AT_BASE_M`
    with the image level: of the solutions clear of the trunk proxy and holdable, the one nearest the fold. None if
    there is none."""
    from demos.cup.pick_demo.grasp import SEEDS, arm_clear, holdable

    cam = np.asarray(LOOK_CAMERA_IN_BASE_M, dtype=float)
    z = np.asarray(LOOK_AT_BASE_M, dtype=float) - cam
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    camera = pose(np.column_stack([x, np.cross(z, x), z]), cam)
    link6 = camera @ invert(wrist_mount())
    fold = d1_ik.from_servo_deg(FOLD_SERVO_DEG)[0]
    best = None
    for seed in [fold, np.zeros(6), *SEEDS]:
        result = d1_ik.solve(joints, link6[:3, 3], link6[:3, :3], q0=np.asarray(seed, dtype=float), body="Link6",
                             offset=(0.0, 0.0, 0.0), max_iterations=300)
        q = np.asarray(result.q, dtype=float)
        if not (result.converged and bool(arm_clear(joints, q.reshape(1, 6))[0]) and holdable(joints, links, q)):
            continue
        servo = np.asarray(d1_ik.to_servo_deg(q), dtype=float)
        if np.any(np.abs(servo) > 90.0 + 1e-6):
            continue
        cost = float(np.max(np.abs(servo - np.asarray(FOLD_SERVO_DEG))))
        if best is None or cost < best[0]:
            best = (cost, servo)
    return None if best is None else best[1]


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Z-Y-X: yaw, then pitch, then roll, as Unitree's IMU reports them."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return rot_z(yaw) @ ry @ rx


def pose(rot=None, pos=(0.0, 0.0, 0.0)) -> np.ndarray:
    out = np.eye(4)
    if rot is not None:
        out[:3, :3] = rot
    out[:3, 3] = pos
    return out


def invert(p) -> np.ndarray:
    p = np.asarray(p, dtype=float)
    out = np.eye(4)
    out[:3, :3] = p[:3, :3].T
    out[:3, 3] = -p[:3, :3].T @ p[:3, 3]
    return out


def base_pose(x: float, y: float, yaw: float, roll: float = 0.0, pitch: float = 0.0) -> np.ndarray:
    """The base in `odom` from a planar pose and the IMU's tilt, at `BASE_HEIGHT_M`."""
    return pose(rpy_matrix(roll, pitch, yaw), (x, y, BASE_HEIGHT_M))


def planar(p) -> tuple[float, float, float]:
    p = np.asarray(p)
    return float(p[0, 3]), float(p[1, 3]), math.atan2(float(p[1, 0]), float(p[0, 0]))


def level(p) -> np.ndarray:
    """The planar part of a pose: its position and its yaw, level."""
    x, y, yaw = planar(p)
    return pose(rot_z(yaw), (x, y, float(np.asarray(p)[2, 3])))


def tilt_deg(p) -> float:
    """How far the base's up axis is from vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.asarray(p)[2, 2])))))


def lidar_to_base(points_lidar) -> np.ndarray:
    return np.asarray(points_lidar, dtype=float).reshape(-1, 3) + np.asarray(LIDAR_IN_BASE_M)


def transform(p, points) -> np.ndarray:
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points @ np.asarray(p)[:3, :3].T + np.asarray(p)[:3, 3]


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# --- the look ---------------------------------------------------------------------------------------------------------

@dataclass
class Fix:
    """Where the look puts the box: the script's frame in `odom`, the `BoxSite` in it, and how it was made."""
    script_odom: np.ndarray
    site: props.BoxSite
    pose_script: np.ndarray            # the box's full pose the fit gave, script frame
    fit: mid360.FaceFit
    detections: int
    lidar_points: int
    tag_only_yaw_deg: float            # the tag's own heading against the one used
    walk_in_m: float = 0.0             # from the look to the stance, straight ahead
    tag_tilt_deg: float = 0.0          # how far from upright the tag's own pose has the box: the camera's (or the
                                       # IMU's) error, since the box stands upright
    beyond_edges: float = 0.0          # the face's points past the door's edges, as a fraction of those round the tag
    other_tags: int = 0                # other id-0 tags the look saw and set aside (`make_fix`): the dock's, usually

    def on_a_door(self) -> bool:
        """Whether the lidar's face round the tag is the box's door: a face just behind the tag that stops at the door's
        edges (`in_range`'s guard against P2Dingo's dock, which carries the same tag on a stand)."""
        return (self.fit.ok and abs(self.fit.depth) <= FACE_BEHIND_TAG_LIMIT_M
                and self.beyond_edges <= BEYOND_LIMIT)

    @property
    def box_odom(self) -> np.ndarray:
        return self.script_odom @ perception.site_pose(self.site)

    def in_range(self, check_tag: bool = True) -> tuple[bool, str]:
        """Whether the box is where the simulated demo placed it from the stance, by a walk in the robot can make; and
        if not, how to move the dog before trying again (the walk in is straight, so it cannot fix the rest).

        `check_tag`: also whether the tag is on the box's door (the look's guard against P2Dingo's dock). Off after the
        walk in: the refit's face then measures the correction it made, not the tag's place on its door (3.5 cm on the
        dog, 2026-10-20, which the guard took for a stand)."""
        s = self.site
        yaw_off = math.degrees(wrap(math.radians(s.yaw_deg) - math.pi))
        x, y, _ = s.root_m
        look_distance = math.hypot(x + self.walk_in_m, y)
        far = NOMINAL_DISTANCE_M + WALK_IN_RANGE_M[1]
        problems = []
        if check_tag and not self.fit.ok:
            return False, f"no door face round the tag in the lidar ({self.fit.why}): not the box, or not in view"
        if check_tag and not self.on_a_door():
            return False, (f"the tag is not on a door the box's size: the lidar's face is {100 * self.fit.depth:+.0f} cm "
                           f"from it and carries on past the door's edges ({self.beyond_edges:.0%}). A tag on a stand "
                           "in front of a wall? (P2Dingo's dock carries the same one)")
        if self.walk_in_m > WALK_IN_RANGE_M[1] or look_distance > far:
            problems.append(f"forward {look_distance - (far - 0.10):.2f} m")
        elif self.walk_in_m == 0.0 and s.distance_m < ACCEPT_DISTANCE_M[0]:
            problems.append(f"back {NOMINAL_DISTANCE_M - s.distance_m:.2f} m")
        elif self.walk_in_m == 0.0 and s.distance_m > ACCEPT_DISTANCE_M[1]:
            problems.append(f"back {NOMINAL_DISTANCE_M + WALK_IN_RANGE_M[0] + 0.05 - look_distance:.2f} m, or "
                            f"forward {look_distance - NOMINAL_DISTANCE_M:.2f} m")
        if abs(s.bearing_deg) > BEARING_LIMIT_DEG:
            problems.append(f"{'left' if y > 0 else 'right'} {abs(y):.2f} m")
        if abs(yaw_off) > YAW_LIMIT_DEG:
            problems.append(f"turn {'left' if yaw_off > 0 else 'right'} {abs(yaw_off):.0f} deg")
        text = (f"box {s.distance_m:.2f} m from the stance at {s.bearing_deg:+.1f} deg, door {yaw_off:+.1f} deg off "
                f"square, walk in {self.walk_in_m:.2f} m")
        if not DISTANCE_RANGE_M[0] - 0.005 <= s.distance_m <= DISTANCE_RANGE_M[1] + 0.005:
            text += f" (outside the simulated {DISTANCE_RANGE_M[0]:.2f}-{DISTANCE_RANGE_M[1]:.2f} m)"
        return not problems, text + ("" if not problems else "; move the dog " + ", ".join(problems))


def make_fix(look_odom, frames, scans, stance_for=None, fix_class=None) -> Fix | None:
    """The fix from a look: `frames` [(base pose in odom, [TagDetection])], `scans` [(base pose in odom, base-frame
    points)], the base standing at `look_odom` when it began (its planar part is used). None without a detection.

    `stance_for(box_look)`: the stance (x, y, yaw) in the look's frame, from the box's level pose there (default: the
    door pull's, straight ahead with its root `NOMINAL_DISTANCE_M` off); `fix_class`: a `Fix` with another task's range
    check (`demos/combiner_key/robot/key_plan.py`).

    More than one tag: the box's tag and P2Dingo's charging dock's are both 36h11 id 0, 100 mm, and the front camera's
    ~100 degree field often holds both (2026-10-22). Averaged, they would put the box between them. So the detections
    are grouped by where they put the box (`TAG_GROUP_M`), a fix is made from each group, and the one on a door
    (`Fix.on_a_door`) is kept, the most detected if several are; none on a door, the most detected, which `in_range`
    then refuses. `other_tags` says how many were set aside."""
    look_odom = level(look_odom)
    look_inv = invert(look_odom)
    # A frame is (base pose, detections) from the head camera, or (base pose, detections, the camera in the base) from
    # the front camera's or the wrist's.
    poses_look = [look_inv @ f[0] @ box_pose(f[2] if len(f) > 2 else CAMERA_POSE_BASE, d) for f in frames for d in f[1]]
    if not poses_look:
        return None
    cloud_look = (np.concatenate([transform(look_inv @ base, pts) for base, pts in scans])
                  if scans else np.zeros((0, 3)))
    groups = tag_groups(poses_look)
    if len(groups) == 1:
        return _fix_from(look_odom, poses_look, cloud_look, stance_for, fix_class)
    fixes = [_fix_from(look_odom, [poses_look[i] for i in group], cloud_look, stance_for, fix_class)
             for group in groups]
    doors = [f for f in fixes if f.on_a_door()]
    chosen = max(doors or fixes, key=lambda f: f.detections)
    chosen.other_tags = len(fixes) - 1
    return chosen


def tag_groups(poses) -> list[list[int]]:
    """Indices of `poses` grouped by where they put the box: within `TAG_GROUP_M` of a group's first, largest first."""
    groups: list[list[int]] = []
    for i, p in enumerate(poses):
        for group in groups:
            if np.linalg.norm(p[:3, 3] - poses[group[0]][:3, 3]) <= TAG_GROUP_M:
                group.append(i)
                break
        else:
            groups.append([i])
    return sorted(groups, key=len, reverse=True)


def _fix_from(look_odom, poses_look, cloud_look, stance_for, fix_class) -> Fix:
    """`make_fix`'s fix from one tag's poses in the look's frame."""
    # The box as the script will have it (the tag's centre, the lidar's heading, level), from the look: its root
    # gives the walk. Not the tag's own pose, whose tilt (the tag's heading error's cousin) moves its root along the
    # floor by the tag's height times it.
    _, level_look, _ = perception.lidar_estimate(poses_look, cloud_look)
    root = level_look[:3, 3]
    # The stance: straight ahead, where the box's root is the nominal distance off; no walk if that is too short.
    if stance_for is not None:
        sx, sy, syaw = stance_for(level_look)
        walk = math.hypot(sx, sy)
    else:
        walk = float(root[0]) - math.sqrt(max(NOMINAL_DISTANCE_M ** 2 - float(root[1]) ** 2, 0.0))
        walk = ODOM_WALK_SCALE * walk if walk >= WALK_IN_RANGE_M[0] else 0.0     # as the odometry will count it
        sx, sy, syaw = walk, 0.0, 0.0
    script_look = pose(rot_z(syaw), (sx, sy, float(root[2]) - props.POST_TOP_M))
    script_odom = look_odom @ script_look
    to_script = invert(script_look)
    poses = [to_script @ p for p in poses_look]
    cloud = transform(to_script, cloud_look)
    tag_site, tag_pose = perception.estimate(poses)
    site, pose_script, fit = perception.lidar_estimate(poses, cloud)
    fix = (fix_class or Fix)(script_odom, site, pose_script, fit, len(poses), len(cloud),
                             math.degrees(wrap(math.radians(tag_site.yaw_deg - site.yaw_deg))), walk, tilt_deg(tag_pose))
    fix.beyond_edges = beyond_edges(cloud, fix)
    return fix


def beyond_edges(points, fix: Fix, params: mid360.FaceParams = mid360.DOOR) -> float:
    """The face's points past the door's edges and top (`BEYOND_*`), as a fraction of its points round the tag: about
    nothing for the box, about one for a wall. 0 without a fit."""
    if not fix.fit.ok or fix.fit.points == 0:
        return 0.0
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    centre = perception.tag_centre(fix.pose_script)
    psi = fix.fit.heading
    h, f = np.array([math.cos(psi), math.sin(psi)]), np.array([math.sin(psi), -math.cos(psi)])
    q, dz = points[:, :2] - centre[:2], points[:, 2] - centre[2]
    u, d = q @ f, q @ h
    on_line = np.abs(d - fix.fit.depth) <= params.inlier_m
    sides = (np.abs(u) >= BEYOND_EDGES_U_M[0]) & (np.abs(u) <= BEYOND_EDGES_U_M[1]) & (dz >= params.z_min_above_tag) \
        & (dz <= params.z_max_above_tag)
    top = (np.abs(u) <= 0.25) & (dz >= BEYOND_TOP_DZ_M[0]) & (dz <= BEYOND_TOP_DZ_M[1])
    window, line = face_inliers(points, fix, params)
    return float((on_line & (sides | top)).sum()) / max(int(line.sum()), 1)


def refine_fix(fix: Fix, scans) -> tuple[Fix, mid360.FaceFit]:
    """The fix again from the stance after the walk in: the door's face fitted round where the fix puts the tag, and
    the box moved along its normal by how far behind that the face is and turned to its heading, about the tag. That
    takes the odometry's error over the walk (and the look's own along the normal) out of where the arm reaches; the
    tag's place along the face, which the lidar cannot see, is kept. The fix unchanged without a face."""
    if not scans:
        return fix, mid360.FaceFit(False, why="no lidar")
    cloud = np.concatenate([transform(invert(fix.script_odom) @ base, pts) for base, pts in scans])
    centre = perception.tag_centre(perception.site_pose(fix.site))
    fit = mid360.face_line(cloud, centre, math.radians(fix.site.yaw_deg) + math.pi)
    if not fit.ok:
        return fix, fit
    normal = np.array([math.cos(fit.heading), math.sin(fit.heading), 0.0])
    pose_script = perception.pose_from_tag_and_heading(centre + fit.depth * normal, fit.heading - math.pi)
    return type(fix)(fix.script_odom, perception.site_from_pose(pose_script), pose_script, fit, fix.detections,
                     len(cloud), fix.tag_only_yaw_deg, fix.walk_in_m, fix.tag_tilt_deg), fit


def reanchor(fix: Fix, base_odom) -> Fix:
    """The fix with its script frame moved to where the base stands (planar; the frame's height kept): the stance is
    where the dog stopped, and the box is placed from there."""
    x, y, yaw = planar(base_odom)
    frame = pose(rot_z(yaw), (x, y, float(fix.script_odom[2, 3])))
    to_new = invert(frame) @ np.asarray(fix.script_odom)
    pose_script = to_new @ np.asarray(fix.pose_script)
    site = perception.site_from_pose(to_new @ perception.site_pose(fix.site))
    return type(fix)(frame, site, pose_script, fix.fit, fix.detections, fix.lidar_points, fix.tag_only_yaw_deg, 0.0,
                     fix.tag_tilt_deg, fix.beyond_edges)


def fix_change(before: Fix, after: Fix) -> tuple[float, float]:
    """How far a refit moved the box's root (m) and turned it (deg)."""
    a, b = np.asarray(before.site.root_m), np.asarray(after.site.root_m)
    return float(np.linalg.norm((b - a)[:2])), abs(math.degrees(wrap(math.radians(after.site.yaw_deg
                                                                                   - before.site.yaw_deg))))


def face_segment(fix: Fix) -> np.ndarray | None:
    """The fitted face as a two-point segment (script frame) at the tag's height, for drawing; None without a fit."""
    if not fix.fit.ok:
        return None
    centre = perception.tag_centre(fix.pose_script)
    normal = np.array([math.cos(fix.fit.heading), math.sin(fix.fit.heading)])
    along = np.array([normal[1], -normal[0]])
    mid = centre[:2] + fix.fit.depth * normal
    half = fix.fit.span / 2
    return np.array([[*(mid - half * along), centre[2]], [*(mid + half * along), centre[2]]])


def face_inliers(points, fix: Fix, params: mid360.FaceParams = mid360.DOOR):
    """(in the face's window, near its line) masks for `points` (script frame), the window laid as the fit lays it."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    centre = perception.tag_centre(fix.pose_script)
    psi = fix.fit.heading if fix.fit.ok else math.radians(fix.site.yaw_deg) + math.pi
    h, f = np.array([math.cos(psi), math.sin(psi)]), np.array([math.sin(psi), -math.cos(psi)])
    q, dz = points[:, :2] - centre[:2], points[:, 2] - centre[2]
    u, d = q @ f, q @ h
    p = params
    window = ((dz >= p.z_min_above_tag) & (dz <= p.z_max_above_tag) & (np.abs(u) <= p.half_width_m)
              & (d >= p.depth_min_m) & (d <= p.depth_max_m)
              & ~((np.abs(u) < p.handle_half_width_m) & (dz >= p.handle_z_m[0]) & (dz <= p.handle_z_m[1])))
    line = window & (np.abs(d - (fix.fit.depth if fix.fit.ok else 0.0)) <= p.inlier_m)
    return window, line


# --- the base -------------------------------------------------------------------------------------------------------

def sport_command(v, error_xy_m: float, error_yaw_rad: float):
    """A walking command sport mode will act on: zero when the base is within `HOLD_*` of where it should be, else the
    command scaled up until its largest part reaches that part's floor (P2Dingo's `liftOutOfDeadZone`), then capped."""
    if error_xy_m < HOLD_XY_M and abs(error_yaw_rad) < HOLD_YAW_RAD:
        return (0.0, 0.0, 0.0)
    v = [float(c) for c in v]
    reach = max(abs(c) / floor for c, floor in zip(v, WALK_FLOOR))
    if 0.0 < reach < 1.0:
        v = [c / reach for c in v]
    return tuple(max(-cap, min(cap, c)) for c, cap in zip(v, WALK_CAP))


# --- the phases -----------------------------------------------------------------------------------------------------

def home_at(base) -> tuple[float, float, float]:
    """The door script's home point relative to a base at (x, y, yaw) in the script's frame."""
    x, y, z = demo_script.HOME_POINT
    c, s = math.cos(base[2]), math.sin(base[2])
    return (base[0] + c * x - s * y, base[1] + s * x + c * y, z)


def scaled_timing(speed_scale: float = 1.0, door_deg: float = props.DOOR_PULL_DEG) -> demo_script.DoorTiming:
    """The door script's clock, every phase `speed_scale` times as long; a pull to `door_deg` takes its share of the
    full pull's time, so the base walks it at the same pace."""
    base = demo_script.DoorTiming()
    fields = {name: getattr(base, name) * speed_scale for name in
              ("stand_s", "approach_s", "settle_s", "enter_s", "arrive_s", "grip_s", "pull_s", "hold_s", "release_s",
               "back_s")}
    fields["pull_s"] *= door_deg / props.DOOR_PULL_DEG
    return demo_script.DoorTiming(**fields, standoff_m=base.standoff_m, door_deg=door_deg)


def attempt_phases(stage: str, timing: demo_script.DoorTiming | None = None, fold: bool = True) -> list:
    """The phases after the look of an attempt that goes as far as `stage`, then lets go and folds (`fold`).

    The door script's phases are the simulated demo's (`door_phases`) up to the hold. From `reach` the hand then backs
    straight off the lever and goes home; from `grip` the jaws squeeze for `PAUSE_S` first and open; the pull opens
    them at the end of its hold (`ungrasp_phases`, which replaces the script's release and its straight line home)."""
    if stage not in STAGES[2:]:
        raise ValueError(f"an attempt's stage is one of {STAGES[2:]}, not {stage!r}")
    timing = timing or demo_script.DoorTiming()
    Phase = demo_script.Phase
    opened, shut = demo_script.JAW_LEVER_OPEN_M, demo_script.JAW_BAR_SHUT_M
    home = lambda st, u: home_at(st["base"])
    phases = [Phase("unfold", UNFOLD_S, path=home, jaw_m=opened)]
    if stage == "unfold":
        phases.append(Phase("pause", PAUSE_S, path=home, jaw_m=opened))
    if STAGES.index(stage) >= STAGES.index("walk"):
        phases += [Phase("walk_in", WALK_IN_S, path=home, jaw_m=opened, base=lambda st, u: (0.0, 0.0, 0.0)),
                   Phase("walk_settle", WALK_SETTLE_S, path=home, jaw_m=opened)]
    if STAGES.index(stage) >= STAGES.index("reach"):
        door = demo_script.door_phases(timing)
        names = [p.name for p in door]
        phases += door[:names.index({"reach": "arrive", "grip": "grip", "pull": "pull"}[stage]) + 1]
        if stage == "pull":
            # The script's hold, its door at the angle the base has actually turned it to (`HOLD_FOLLOW_DEG`).
            site = lambda st: st["box_site"]
            turned = lambda st: st.get("door_deg", timing.door_deg)
            phases.append(Phase("hold", timing.hold_s, jaw_m=shut, measure=True, servo=True,
                                path=lambda st, u: site(st).grasp_point_m(0.0, turned(st)),
                                approach=lambda st: site(st).approach_unit(turned(st)),
                                object_axis=lambda st: site(st).lever_axis_unit(0.0, turned(st))))
        if stage == "grip":
            site = lambda st: st["box_site"]
            phases.append(Phase("squeeze", PAUSE_S, target=lambda st: site(st).grasp_point_m(0.0, 0.0), jaw_m=shut,
                                approach=lambda st: site(st).approach_unit(0.0),
                                object_axis=lambda st: site(st).lever_axis_unit(0.0, 0.0), servo=True))
        phases += ungrasp_phases(timing, release=stage != "reach")
    if fold:
        phases.append(Phase("fold", FOLD_S, path=home, jaw_m=opened))
    return phases


def ungrasp_phases(timing: demo_script.DoorTiming | None = None, release: bool = True) -> list:
    """Letting go of the lever, then home.

    - "release" (unless the jaws never shut, `release`): they open where the hand is.
    - "ungrasp": the hand backs straight off the lever along its own approach (Link6's +z), by the standoff and at the
      enter's pace, its orientation held. The fingertips stand 20 mm past the jaw centre, in the 27 mm gap behind the
      lever and beside its 63 mm hub; straight out, they clear the lever's front after 29 mm without moving towards the
      hub.
    - "home": from there, clear of the door, to the home point where the base now stands.

    Both from the hand's own pose as the release begins (`Attempt._held_pose`, the measured joints), not the script's:
    after the pull the hand is where the lever is, which the door's load and the gait's carry can have moved off the
    goal. The script's "back" went straight from the lever to the home point, which crossed in front of the hub with
    the fingers still behind the lever, and the fingers pushed the door on the way out (2026-10-20)."""
    timing = timing or demo_script.DoorTiming()
    Phase = demo_script.Phase
    opened = demo_script.JAW_LEVER_OPEN_M
    held = lambda key: (lambda st: st["held"]()[key])
    hand = dict(approach=held("approach"), object_axis=held("axis"), servo=True)
    phases = [Phase("release", timing.release_s, target=held("hand"), jaw_m=opened, **hand)] if release else []
    return phases + [Phase("ungrasp", timing.enter_s, target=held("back"), jaw_m=opened, **hand),
                     Phase("home", timing.back_s, path=lambda st, u: home_at(st["base"]), jaw_m=opened)]


def recovery_phases(timing: demo_script.DoorTiming | None = None) -> list:
    """After an abort, if the operator chooses: let go of the lever from where the hand is (`ungrasp_phases`) and fold.
    The base stands throughout."""
    return ungrasp_phases(timing) + [demo_script.Phase("fold", FOLD_S, path=lambda st, u: home_at(st["base"]),
                                                       jaw_m=demo_script.JAW_LEVER_OPEN_M)]


# --- one 10 Hz step ---------------------------------------------------------------------------------------------------

@dataclass
class Observation:
    """What the robot knows at a step."""
    now_s: float
    base_odom: np.ndarray               # 4x4
    odom_age_s: float
    arm_servo_deg: np.ndarray | None    # (6,), None without feedback
    arm_age_s: float
    scan: tuple | None = None           # (base pose in odom, base-frame points): a lidar frame new since the last step
    door_reading: mid360.DoorReading = field(default_factory=lambda: mid360.NO_READING)
    web_teleop: bool = False
    gripper_units: float | None = None  # servo 6's report, None without one
    body_height_m: float | None = None  # sport mode's `body_height`, None without one


@dataclass
class Command:
    """What a step asks for. `arm_servo_deg` None: nothing to the arm (it holds where it is)."""
    phase: str
    arm_servo_deg: list | None = None
    gripper_units: float | None = None
    base: tuple = (0.0, 0.0, 0.0)
    walking: bool = False
    hand_target_script: tuple | None = None
    approach_script: tuple | None = None
    object_axis_script: tuple | None = None
    base_target_script: tuple | None = None
    ik_status: str = ""
    #: The arm link's speed for this step's move (`ARM_SPEED_DEG_S`, `JOINT_SPEED_DEG_S` in joint space).
    speed_deg_s: float = ARM_SPEED_DEG_S
    abort: str = ""
    note: str = ""
    finished: bool = False
    #: A posture for sport mode, sent once each time it changes: "" (none), "lie" (StandDown), "stand" (StandUp),
    #: "balance" (BalanceStand). The door pull asks for none; the key's insertion lies down for it.
    posture: str = ""


@dataclass
class Attempt:
    """One attempt from a fix: the phases on a clock, the arm's IK, the walk and the aborts. `step` at 10 Hz."""
    fix: Fix
    stage: str
    timing: demo_script.DoorTiming = field(default_factory=demo_script.DoorTiming)
    fold: bool = True

    #: The phase sets the clock, the IK and the aborts read: the door pull's (a task with other phases gives its own).
    joint_phases = JOINT_PHASES
    dwell_phases = DWELL_PHASES
    position_only_phases = POSITION_ONLY_PHASES
    holding_phases = HOLDING_PHASES
    at_the_lever_phases = AT_THE_LEVER_PHASES
    walking_phases = WALKING_PHASES
    release_held_phases = RELEASE_HELD_PHASES
    #: Whether the lidar reads the door's angle through the door's phases (`node.py`): the door pull's attempts only.
    reads_door = True
    #: How far the refit after the walk in may move the box before it aborts (`REFIT_LIMIT_*`).
    refit_limit_m = REFIT_LIMIT_M
    refit_limit_deg = REFIT_LIMIT_DEG

    def __post_init__(self):
        joints, links = d1_ik.load_urdf()
        self._joints = joints
        self.arm = decoupled_arm.ArmIK(joints, links)
        self.script = demo_script.DemoScript(self._make_phases())
        self.script.reset(demo_script.HOME_POINT, demo_script.JAW_LEVER_OPEN_M)
        self.look_fix = self.fix
        self.refit: mid360.FaceFit | None = None
        self.status = ""
        self.last_sent: list | None = None
        self.last_goal: list | None = None
        self.last_command: Command | None = None
        self._joint_goal: tuple | None = None
        self._previous_base = None
        self._settle_scans: list = []
        self._lag_since = self._door_since = self._ik_since = None
        self._joint_waiting = False
        self._joint_wait_s = 0.0
        self._walk: dict | None = None
        #: Where the hand measured itself as the release began (`_held_pose`), and its orientation (Link6, script
        #: frame), held while the jaws open and it backs off the lever (`RELEASE_HELD_PHASES`).
        self._held: dict | None = None
        self.held_rotation: np.ndarray | None = None
        self._measured: np.ndarray | None = None
        self._base_s: np.ndarray | None = None
        self._jaws: list = []                      # (t, units): servo 6's recent reports
        self._jaw_from: float | None = None        # the jaws' report as a release began
        self._pull: dict | None = None
        self._pose_misses = 0
        self.events: list[dict] = []
        self._prepare()

    def _make_phases(self) -> list:
        """The attempt's phases after the look (`attempt_phases`)."""
        return attempt_phases(self.stage, self.timing, self.fold)

    def _prepare(self) -> None:
        """Before anything moves: the grasp, for a stage that reaches the lever."""
        if STAGES.index(self.stage) >= STAGES.index("reach"):
            self._choose_grasp()

    def _choose_grasp(self) -> None:
        """The grasp (pitch, roll) for the attempt, planned with the base at the stance before anything moves: the
        search tries every grasp from many seeds when the first fails, which is no thing to do inside the 10 Hz loop."""
        to_base = invert(pose(None, (0.0, 0.0, BASE_HEIGHT_M - float(self.fix.script_odom[2, 3]))))
        site = self.fix.site
        self.arm.choose_grasp(transform(to_base, site.grasp_point_m())[0], np.asarray(site.approach_unit(0.0)),
                              np.asarray(site.lever_axis_unit(0.0, 0.0)), self.timing.standoff_m)
        self.events.append({"phase": "grasp", "choice": self.arm.grasp_note})

    @property
    def phase(self) -> str:
        return "done" if self.script.finished else self.script.phases[self.script.index].name

    def next_phase(self) -> str:
        """The phase after this one, or "done"."""
        nxt = self.script.index + 1
        return self.script.phases[nxt].name if nxt < len(self.script.phases) else "done"

    def base_script(self, base_odom) -> np.ndarray:
        return invert(self.fix.script_odom) @ np.asarray(base_odom)

    def state(self, base_odom) -> dict:
        st = {"box_site": self.fix.site, "base": planar(self.base_script(base_odom)), "held": self._held_pose}
        if self._pull is not None and self._pull["done"]:
            st["door_deg"] = self._followed_door_deg(st["base"])
        return st

    def _followed_door_deg(self, base) -> float:
        """After the pull: the door's angle as the base has turned it, the pull's at least and `HOLD_FOLLOW_DEG` past it
        at most; frozen once the hand begins to let go."""
        pull = self._pull
        if self._held is None:
            turned = abs(math.degrees(wrap(base[2] - pull["yaw0"])))
            pull["followed"] = min(max(turned, self.timing.door_deg), self.timing.door_deg + HOLD_FOLLOW_DEG)
        return pull.get("followed", self.timing.door_deg)

    def step(self, obs: Observation, dt: float = 0.1) -> Command:
        base_s = self.base_script(obs.base_odom)
        st = self.state(obs.base_odom)
        before = self.phase
        self._base_s = base_s
        if obs.arm_servo_deg is not None:
            self._measured = np.asarray(obs.arm_servo_deg, dtype=float)
        if obs.gripper_units is not None and obs.arm_age_s <= ARRIVED_REPORT_S:
            self._jaws = [j for j in self._jaws if obs.now_s - j[0] <= 1.0] + [(obs.now_s, float(obs.gripper_units))]
        if before == "walk_settle" and obs.scan is not None:
            self._settle_scans.append(obs.scan)
        step_dt, held = self._clock(obs, dt)
        if held:
            return Command(before, abort=held)
        if before == "pull":
            step_dt, why = self._pull_clock(st["base"], dt)
            if why:
                return Command(before, abort=why)
        cmd = self.script.update(step_dt, st)
        if cmd.finished:
            return Command("done", finished=True)
        if before == "walk_settle" and cmd.phase != "walk_settle":
            note = self._refit()
            if note.startswith("abort:"):
                return Command(cmd.phase, abort=note[len("abort:"):].strip())
            # The stance is where the dog stopped: the box placed from there, and the arm must reach it.
            self.fix = reanchor(self.fix, obs.base_odom)
            ok, text = self.fix.in_range(check_tag=False)
            self.events.append({"phase": "reanchor", "ok": ok, "text": text})
            if not ok:
                return Command(cmd.phase, abort=f"after the walk in: {text}")
            base_s = self.base_script(obs.base_odom)
            st = self.state(obs.base_odom)
        if before != cmd.phase:
            why = self._on_phase_change(before, cmd.phase, obs)
            if why:
                return Command(cmd.phase, abort=why)
            base_s = self.base_script(obs.base_odom)
            st = self.state(obs.base_odom)
        out = Command(cmd.phase, hand_target_script=tuple(cmd.point), approach_script=tuple(cmd.approach),
                      object_axis_script=tuple(cmd.object_axis), base_target_script=cmd.base_pose,
                      speed_deg_s=JOINT_SPEED_DEG_S if cmd.phase in self.joint_phases else ARM_SPEED_DEG_S)
        why = self._abort(obs, cmd.phase)
        if why:
            out.abort = why
            return out
        if obs.arm_servo_deg is not None:
            measured = np.asarray(obs.arm_servo_deg, dtype=float)
            goal = self._goal_servo(cmd, base_s, measured, st)
            self.last_goal = [float(v) for v in goal]
            out.arm_servo_deg = list(self.last_goal)       # the arm link steps towards it
            out.gripper_units = d1_hardware.finger_travel_to_gripper_units(cmd.jaw_m)
            out.ik_status = self.status
        out.base, out.walking = self._base(cmd, st["base"], dt)
        if before != cmd.phase:
            self._previous_base = None
            self.events.append({"t": round(obs.now_s, 3), "phase": cmd.phase})
        self.last_command = out
        return out

    def _on_phase_change(self, before: str, phase: str, obs: Observation) -> str:
        """As the script goes from `before` to `phase`, before the new phase's first goal: another task's hook (the key
        insertion's refit and plan once lying down). An abort's reason, or ""."""
        return ""

    def _grasp_rotation(self, to_base):
        """Link6's rotation (base frame) for the attempt's grasp on the shut door, or None without a grasp."""
        if self.arm.grasp is None:
            return None
        site = self.fix.site
        approach = to_base[:3, :3] @ np.asarray(site.approach_unit(0.0))
        bar = to_base[:3, :3] @ np.asarray(site.lever_axis_unit(0.0, 0.0))
        return decoupled_arm.hand_rotation(approach, bar, *self.arm.grasp)

    def _clock(self, obs: Observation, dt: float) -> tuple[float, str]:
        """How far to move the script's clock this step, and an abort's reason. Ends a phase that waits for the arm as
        soon as the arm is there (`DWELL_PHASES`, `JOINT_PHASES`); holds a phase past its clock while the walk in, a
        joint move or the jaws are not done (`JOINT_ARRIVED_DEG`, `JAW_*`), up to its timeout."""
        self._joint_waiting = False
        if self.script.finished or obs.arm_servo_deg is None:
            return dt, ""
        current = self.script.phases[self.script.index]
        remaining = current.duration_s - self.script.elapsed_s
        at_clock_end = remaining <= dt + 1e-9
        if current.name not in ("walk_in", "release", "grip") and current.name not in self.joint_phases:
            self._joint_wait_s = 0.0
        if current.name == "walk_in":
            # The walk in runs until its corrective steps are done, past its clock if need be.
            if self._walk is None or self._walk["state"] == "done" or not at_clock_end:
                return dt, ""
            return self._wait(dt, remaining, WALK_IN_TIMEOUT_S, "the walk in did not finish")
        goal = self._joint_goal[1] if (current.name in self.joint_phases and self._joint_goal is not None
                                       and self._joint_goal[0] == current.name) else self.last_goal
        off = float(np.max(np.abs(np.asarray(goal) - np.asarray(obs.arm_servo_deg)))) if goal is not None else math.inf
        fresh = obs.arm_age_s <= ARRIVED_REPORT_S
        if (current.name in self.dwell_phases or current.name in self.joint_phases) and goal is not None:
            close = FOLD_ARRIVED_DEG if current.name == "fold" else ARRIVED_DEG
            if fresh and off <= close and self.script.elapsed_s >= DWELL_MIN_S and not at_clock_end:
                self._joint_wait_s = 0.0
                return remaining + 1e-6, ""                     # there: on to the next phase now
        if not at_clock_end:
            return dt, ""
        if current.name in self.joint_phases:
            if goal is None or off <= JOINT_ARRIVED_DEG:
                self._joint_wait_s = 0.0
                return dt, ""
            return self._wait(dt, remaining, JOINT_TIMEOUT_S,
                              f"the arm did not reach the {current.name}: still {off:.0f} deg off")
        if current.name == "grip" and self._jaws and not self._jaws_stalled(obs.now_s):
            # Still closing: the pull waits for the grip, up to `JAW_WAIT_S`, then goes on.
            return self._wait(dt, remaining, JAW_WAIT_S, "")
        if current.name == "release" and self._jaws and not self._jaws_open(obs.now_s):
            reading = self._jaws[-1][1]
            return self._wait(dt, remaining, JAW_WAIT_S, f"the jaws did not open: servo 6 at {reading:.1f} units, "
                                                         f"sent {GRIPPER_OPEN_UNITS:.1f}; the hand is still on the lever")
        self._joint_wait_s = 0.0
        return dt, ""

    def _wait(self, dt: float, remaining: float, limit_s: float, why: str) -> tuple[float, str]:
        """Hold the phase at its end (its last goal and jaw command, not the step before them) this step; past
        `limit_s` of holding, `why` (an abort, or "" to go on)."""
        self._joint_wait_s += dt
        if self._joint_wait_s > limit_s:
            self._joint_wait_s = 0.0
            return (0.0, why) if why else (dt, "")
        self._joint_waiting = True
        return max(0.0, remaining - 1e-6), ""

    def _jaws_stalled(self, now: float) -> bool:
        recent = [u for t, u in self._jaws if now - t <= JAW_STALL_S]
        spanned = self._jaws and now - self._jaws[0][0] >= JAW_STALL_S
        return bool(spanned and recent and max(recent) - min(recent) < JAW_STALL_UNITS)

    def _jaws_open(self, now: float) -> bool:
        reading = self._jaws[-1][1]
        if reading >= GRIPPER_OPEN_UNITS - JAW_OPEN_TOLERANCE_UNITS:
            return True
        start = self._jaw_from if self._jaw_from is not None else GRIPPER_SHUT_UNITS
        return self._jaws_stalled(now) and reading >= start + 0.5 * (GRIPPER_OPEN_UNITS - start)

    def sent(self, command: Command) -> None:
        """What the node actually sent the arm, which the lag check compares the arm against."""
        if command.arm_servo_deg is not None:
            self.last_sent = list(command.arm_servo_deg)

    def recover(self) -> None:
        """Swap what is left of the attempt for `recovery_phases` (let go from where the hand measures itself, back off,
        home, fold) if it may be at the lever (`AT_THE_LEVER_PHASES`); else for the fold alone."""
        last = self.last_command
        if last is None or last.hand_target_script is None or last.phase not in self.at_the_lever_phases:
            phases = [demo_script.Phase("fold", FOLD_S, path=lambda st, u: home_at(st["base"]),
                                        jaw_m=demo_script.JAW_LEVER_OPEN_M)]
        else:
            phases = self._recovery_phases()
        self.script = demo_script.DemoScript(phases)
        self.script.reset(last.hand_target_script if last and last.hand_target_script else demo_script.HOME_POINT,
                          demo_script.JAW_BAR_SHUT_M)
        self._joint_goal = self._held = self.held_rotation = None
        self._joint_wait_s = 0.0
        self._lag_since = self._door_since = self._ik_since = None
        self.events.append({"phase": "recover"})

    def _recovery_phases(self) -> list:
        """What an abort at the lever turns into (`recovery_phases`)."""
        return recovery_phases(self.timing)

    def _held_pose(self) -> dict:
        """The hand as the release begins, script frame, from the arm's measured joints (else its last goal): where
        `ungrasp_phases` let go of the lever from. Captured once, at the first call."""
        if self._held is None:
            servo = self._measured if self._measured is not None else (
                self.last_goal if self.last_goal is not None else d1_ik.to_servo_deg(self.arm.q))
            base_s = self._base_s if self._base_s is not None else self.fix.script_odom @ np.eye(4)
            self._capture(servo, base_s)
        return self._held

    def _capture(self, servo_deg, base_s) -> None:
        from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6

        q = d1_ik.from_servo_deg(servo_deg)[0]
        jaw_b, link6_b = d1_ik.tool_pose(self._joints, q, body="Link6", offset=JAW_CENTRE_LINK6)
        hand = transform(base_s, [jaw_b])[0]
        rot = base_s[:3, :3] @ link6_b
        approach = rot[:, 2]                        # Link6 +z: the way the jaws point
        back = hand - self.timing.standoff_m * approach
        self.held_rotation = rot
        self._jaw_from = self._jaws[-1][1] if self._jaws else None
        self._held = {"hand": tuple(float(v) for v in hand), "back": tuple(float(v) for v in back),
                      "approach": tuple(float(v) for v in approach), "axis": tuple(float(v) for v in rot[:, 0])}
        self.events.append({"phase": "let_go", "hand_script": [round(float(v), 4) for v in hand],
                            "approach": [round(float(v), 3) for v in approach],
                            "jaws_units": None if self._jaw_from is None else round(self._jaw_from, 2)})

    # --- the arm ------------------------------------------------------------------------------------------------

    def _goal_servo(self, cmd, base_s, measured, st):
        """Where the arm should go, servo degrees: the fold, or a joint goal fixed for the phase (the IK's solution for
        its end point, from where the arm is), or the IK for this step's jaw centre (position only through
        `POSITION_ONLY_PHASES`). Folded through the look's pause (`unfold` not yet begun) it holds."""
        to_base = invert(base_s)
        if cmd.phase == "fold":
            self.status = "joint"
            self._joint_goal = ("fold", np.asarray(FOLD_SERVO_DEG))
            return self._joint_goal[1]
        if cmd.phase in self.joint_phases:
            if self._joint_goal is None or self._joint_goal[0] != cmd.phase:
                demo = self.script
                end = demo.phase_end_point(demo.phases[demo.index], st)
                grasp, note = self.arm.grasp, self.arm.grasp_note
                self.arm.reset(d1_ik.from_servo_deg(measured)[0])
                self.arm.grasp, self.arm.grasp_note = grasp, note
                q = self._solve_joint_goal(cmd.phase, transform(to_base, end)[0], to_base)
                self._joint_goal = (cmd.phase, np.asarray(d1_ik.to_servo_deg(q)))
            self.status = "joint"
            return self._joint_goal[1]
        if self._joint_goal is not None:
            # Out of a joint move: the IK carries on from where the arm is, its grasp choice kept.
            grasp, note = self.arm.grasp, self.arm.grasp_note
            self.arm.reset(d1_ik.from_servo_deg(measured)[0])
            self.arm.grasp, self.arm.grasp_note = grasp, note
            self._joint_goal = None
        point_b = transform(to_base, cmd.point)[0]
        rot = self._grasp_rotation(to_base) if cmd.phase in ("walk_in", "walk_settle") else None
        if cmd.phase in self.position_only_phases:
            # The walk keeps the grasp's orientation when it has one; else, as the pause and the way home, the jaw
            # centre alone.
            step = self.arm.solve(point_b, rot) if rot is not None else None
            if step is None or step.status != "pose":
                step = self.arm.solve(point_b, None)
        else:
            letting_go = self.held_rotation is not None and cmd.phase in self.release_held_phases
            if letting_go:
                # Letting go: the hand keeps the orientation it had when the release began.
                rot = to_base[:3, :3] @ self.held_rotation
            else:
                approach = to_base[:3, :3] @ np.asarray(cmd.approach)
                bar = to_base[:3, :3] @ np.asarray(cmd.object_axis)
                if self.arm.grasp is None:
                    grasp_b = transform(to_base, self.fix.site.grasp_point_m())[0]
                    self.arm.choose_grasp(grasp_b, approach, bar, self.timing.standoff_m)
                rot = decoupled_arm.hand_rotation(approach, bar, *self.arm.grasp)
            seed = self.arm.q.copy()
            step = self.arm.solve(point_b, rot)
            if step.status == "pose" or self.last_goal is None:
                self._pose_misses = 0
            elif letting_go:
                # Never another wrist with the fingers round the lever: the goal holds, and `IK_HOLD_S` of it aborts.
                self.arm.q = seed
                self.status = "hold"
                return np.asarray(self.last_goal)
            elif self._pose_misses < POSE_MISS_HOLD_STEPS:
                # Missed the pose this step: keep the last goal and the IK's seed, rather than switch wrists.
                self._pose_misses += 1
                self.arm.q = seed
                self.status = "pose_held"
                return np.asarray(self.last_goal)
        self.status = step.status
        return np.asarray(d1_ik.to_servo_deg(step.q))

    def _solve_joint_goal(self, phase: str, end_b, to_base) -> np.ndarray:
        """The joints a joint phase goes to, for its end point (base frame), from where the arm is. Unfolding for a
        reach: straight to the hand's grasp orientation, so the walk and the stand that follow need no wrist turn (it
        was a 94 deg one between the walk and the stand)."""
        rot = self._grasp_rotation(to_base) if phase == "unfold" else None
        solved = self.arm.solve(end_b, rot) if rot is not None else None
        if solved is None or solved.status != "pose":
            solved = self.arm.solve(end_b, None)
        return solved.q

    # --- the base -----------------------------------------------------------------------------------------------

    def _base(self, cmd, actual, dt):
        if cmd.phase == "walk_in":
            return self._walk_in(actual, dt)
        if cmd.phase == "pull":
            pull = self._pull
            if pull is None or pull["done"]:
                return (0.0, 0.0, 0.0), False
            return pull["twist"], True
        if cmd.phase == "hold" or cmd.base_pose is None:
            return (0.0, 0.0, 0.0), False
        desired = cmd.base_pose
        v = demo_script.base_velocity_command(desired, self._previous_base, actual, dt)
        self._previous_base = desired
        shaped = sport_command(v, math.hypot(desired[0] - actual[0], desired[1] - actual[1]),
                               wrap(desired[2] - actual[2]))
        return shaped, any(abs(c) > 0 for c in shaped)

    def _pull_twist(self, base) -> dict:
        """The pull's constant twist, from where the base stands as it begins (module constants' note)."""
        angle = math.radians(GEOMETRY.hinge_axis_z * self.timing.door_deg)
        hx, hy = self.fix.site.hinge_xy_m()
        c, s = math.cos(base[2]), math.sin(base[2])
        dx, dy = hx - base[0], hy - base[1]
        h = (c * dx + s * dy, -s * dx + c * dy)                 # the hinge in the base's frame
        w = math.copysign(1.0, angle)
        twist = [w * h[1], -w * h[0], w]                        # rotation about the hinge, at 1 rad/s
        reach = max(abs(v) / f for v, f in zip(twist, WALK_FLOOR))
        twist = [v / reach for v in twist]                      # its largest part at its floor
        over = max(abs(v) / cap for v, cap in zip(twist, WALK_CAP))
        if over > 1.0:
            twist = [v / over for v in twist]
        self.events.append({"phase": "pull", "twist": [round(v, 3) for v in twist],
                            "hinge_in_base_m": [round(v, 3) for v in h], "seconds": round(abs(angle / twist[2]), 2)})
        return {"twist": tuple(twist), "yaw0": base[2], "angle": angle, "done": False, "best": 0.0, "since": 0.0}

    def _pull_clock(self, base, dt):
        """How far to move the script's clock this step: to where the base's turn has got the door. Returns (dt, abort
        reason)."""
        if self._pull is None:
            self._pull = self._pull_twist(base)
        pull = self._pull
        turned = wrap(base[2] - pull["yaw0"])
        u = max(0.0, turned / pull["angle"])
        current = self.script.phases[self.script.index]
        if u >= 1.0:
            pull["done"] = True
            return current.duration_s - self.script.elapsed_s + 1e-6, ""
        if u > pull["best"] + math.radians(PULL_STALL_DEG) / abs(pull["angle"]):
            pull["best"], pull["since"] = u, 0.0
        else:
            pull["since"] += dt
            if pull["since"] > PULL_STALL_S:
                return 0.0, f"the base stopped turning {math.degrees(turned):.0f} deg into the pull"
        return max(0.0, u * current.duration_s - self.script.elapsed_s), ""

    def _walk_in(self, actual, dt):
        """The walk in to the stance (`WALK_IN_*`): walk, stop early by the carry, stand and measure, step again if
        short. Returns (command, walking)."""
        stand = ((0.0, 0.0, 0.0), False)
        remaining = -actual[0]
        heading = max(-WALK_IN_YAW_CAP, min(WALK_IN_YAW_CAP, -WALK_IN_YAW_GAIN * actual[2]))
        walk = ((WALK_IN_MPS, 0.0, heading), True)
        w = self._walk
        if w is None:
            w = self._walk = {"state": "walking" if remaining > WALK_IN_TOLERANCE_M else "done", "t": 0.0,
                              "min_t": 0.0, "carry_s": CARRY_S, "steps": 0, "stop_x": actual[0]}
        if w["state"] == "walking":
            w["t"] += dt
            if w["t"] < w["min_t"] or remaining > WALK_IN_MPS * w["carry_s"]:
                return walk
            w.update(state="coasting", t=0.0, stop_x=actual[0])
            return stand
        if w["state"] == "coasting":
            w["t"] += dt
            if w["t"] < WALK_IN_COAST_S:
                return stand
            carried = actual[0] - w["stop_x"]
            w["carry_s"] = min(max(carried / WALK_IN_MPS, 0.0), 0.8)
            self.events.append({"phase": "walk_in", "short_m": round(remaining, 3), "carried_m": round(carried, 3),
                                "step": w["steps"]})
            reach = WALK_IN_MPS * (w["carry_s"] + WALK_IN_MIN_STEP_S)
            if remaining > WALK_IN_TOLERANCE_M and w["steps"] < WALK_IN_MAX_STEPS and remaining > 0.7 * reach:
                w.update(state="walking", t=0.0, min_t=WALK_IN_MIN_STEP_S, steps=w["steps"] + 1)
                return walk
            w["state"] = "done"
        return stand

    def _refit(self) -> str:
        """After the walk in: `refine_fix` on the settle's lidar frames. Too big a change aborts."""
        refined, fit = refine_fix(self.fix, self._settle_scans)
        self.refit = fit
        if not fit.ok:
            self.events.append({"phase": "refit", "used": False, "why": fit.why})
            return f"refit: no face ({fit.why}); the look's fix kept"
        moved_m, turned_deg = fix_change(self.fix, refined)
        self.events.append({"phase": "refit", "used": True, "moved_mm": round(1000 * moved_m, 1),
                            "turned_deg": round(turned_deg, 2)})
        if moved_m > self.refit_limit_m or turned_deg > self.refit_limit_deg:
            return (f"abort: the lidar puts the door {1000 * moved_m:.0f} mm and {turned_deg:.1f} deg from where the "
                    "look did")
        self.fix = refined
        return f"refit: moved {1000 * moved_m:.0f} mm, turned {turned_deg:.1f} deg"

    # --- the aborts ---------------------------------------------------------------------------------------------

    def _abort(self, obs: Observation, phase: str) -> str:
        now = obs.now_s
        if obs.web_teleop:
            return "the website's joysticks are on"
        if obs.odom_age_s > ODOM_STALE_S and phase in self.walking_phases:
            return f"odometry {obs.odom_age_s:.2f} s old"
        if tilt_deg(obs.base_odom) > TILT_LIMIT_DEG:
            return f"base tilted {tilt_deg(obs.base_odom):.0f} deg"
        if obs.arm_servo_deg is None or obs.arm_age_s > ARM_STALE_S:
            return f"arm feedback {obs.arm_age_s:.2f} s old"
        holding = phase in self.holding_phases
        if holding and self.last_goal is not None:
            lag = float(np.max(np.abs(np.asarray(self.last_goal) - np.asarray(obs.arm_servo_deg))))
            if lag > ARM_LAG_DEG:
                self._lag_since = now if self._lag_since is None else self._lag_since
                if now - self._lag_since >= ARM_LAG_S:
                    return f"the arm is {lag:.0f} deg from where the IK wants it: loaded"
            else:
                self._lag_since = None
        if phase in ("pull", "hold") and obs.door_reading.ok:
            commanded = self.commanded_door_deg()
            if abs(obs.door_reading.angle_deg - commanded) > DOOR_DISAGREE_DEG:
                self._door_since = now if self._door_since is None else self._door_since
                if now - self._door_since >= DOOR_DISAGREE_S:
                    return (f"the lidar has the door at {obs.door_reading.angle_deg:.0f} deg, the script at "
                            f"{commanded:.0f}")
            else:
                self._door_since = None
        if (holding or phase in self.release_held_phases) and self.status == "hold":
            self._ik_since = now if self._ik_since is None else self._ik_since
            if now - self._ik_since >= IK_HOLD_S:
                return "no IK solution for the hand"
        else:
            self._ik_since = None
        return ""

    def commanded_door_deg(self) -> float:
        """The door's angle as the script has it now."""
        phase = self.phase
        if phase == "pull":
            current = self.script.phases[self.script.index]
            return self.timing.door_deg * min(1.0, self.script.elapsed_s / max(current.duration_s, 1e-9))
        if self._pull is None:
            return 0.0
        return self._pull.get("followed", self.timing.door_deg) if self._pull["done"] else self.timing.door_deg


def release(fix: Fix, base_odom, arm_servo_deg, timing: demo_script.DoorTiming | None = None) -> Attempt:
    """An attempt that only lets go, from wherever the arm is, with no look: `recovery_phases` (the jaws opened where
    they are, the hand straight back off the lever with its orientation held, home, fold). For an arm left holding the
    lever (an attempt quit after an abort). `fix` gives the script frame (the base where it stands)."""
    timing = timing or demo_script.DoorTiming()
    attempt = Attempt(fix, "unfold", timing)
    q = d1_ik.from_servo_deg(arm_servo_deg)[0]
    attempt._capture(arm_servo_deg, attempt.base_script(base_odom))
    attempt.script = demo_script.DemoScript(recovery_phases(timing))
    attempt.script.reset(attempt._held["hand"], demo_script.JAW_BAR_SHUT_M)
    attempt.arm.reset(q)
    return attempt


def step_towards(measured, goal, max_step_deg: float = ARM_STEP_DEG) -> list[float]:
    """`goal` capped to one step from `measured` along the longest joint, servo degrees
    (`d1_hardware.CartesianMover.step_towards`)."""
    measured = np.asarray(measured, dtype=float)
    delta = np.asarray(goal, dtype=float) - measured
    longest = float(np.max(np.abs(delta))) if delta.size else 0.0
    if longest > max_step_deg:
        delta *= max_step_deg / longest
    return [float(v) for v in measured + delta]

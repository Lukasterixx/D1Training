"""The key turned by the D1 on the real Go2, as numbers: the attempt `combiner_door/robot/node.py --task key` runs. numpy
only, no ROS, no torch.

The simulated push (`demos/combiner_key/arm_turn.py`, F-183) on the door pull's robot program (`combiner_door/robot/
plan.py`): its look and fix (the head RealSense's tag, the MID-360's face fit), its straight walk in with sport mode's
carry, its refit from the stance, its arm IK at 10 Hz through the arm link, its aborts and its record. What is the key's:

- **The stance**: the dog stands roughly in front of the *lock* (not the box's middle) and looks; the walk in is straight
  ahead until the door's face is `WALK_TO_DOOR_M` off (the middle of `arm_turn.ACCEPT_DOOR_AHEAD_M`). A straight walk
  cannot move it sideways, so the look's range check wants the lock 4-12 cm left of the base already
  (`arm_turn.ACCEPT_LOCK_LEFT_M`) and says how far to move the dog if not. After the walk and the refit the stance is
  accepted anywhere the push works from (`arm_turn.PUSH_*`): close enough, as the door pull accepts its range.
- **The phases** (`key_phases`): unfold, walk in, settle; `key_out` -- a joint-space move to the hand 8 cm out from the
  platform, pitched 60 degrees down onto it (the pose from the scripted grip's seeded search, checked clear of the
  trunk); `key_above`, `key_touch` (the fingertips' edge 4 mm into the platform, 0.13 m out from the shaft); `turn`
  (the hand pose rotated rigidly about the shaft, -45 to +45 degrees); `key_hold` (resting on the key at its stop);
  `key_lift`, `key_clear` (straight up, clear of the head camera), home, fold.
- **The stages** (`STAGES`): look, unfold, walk, reach (to `key_above` and back), touch (onto the key and off it, no
  turn), turn.
- **The robot's own check** that the key turned: the fingertips' edge on the estimated key (forward kinematics of the
  measured joints) past the unlock at the hold, *and* the hand never more than `arm_turn`'s track limit behind its
  command through the turn (`turn_check`). A turn the hand falls that far behind for `STALL_S` aborts: the key is not
  turning (14 and 17 N in simulation), and the recovery lifts the hand off and folds.
- **The recovery** (`recovery_phases`): from where the hand measures itself, up off the platform along its normal and
  back out, its orientation held, then home and fold.

Not checked anywhere but on the robot: the real key's place and handedness against `demos/combiner_key/geometry.py`
(the simulation's key points right, toward the box's middle, its platform up; the dry run draws it in RViz to compare),
and what the real D1 pushes at its published limits.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

import d1_ik
from demos.combiner_door import head_camera, perception
from demos.combiner_door.robot import plan
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6, _chain_points, _seeds, path_clear, solve_link6
from demos.unifp import decoupled_arm, script as demo_script

from .. import arm_turn as at
from .. import geometry as kg

STAGES = ("watch", "look", "unfold", "walk", "reach", "touch", "turn")
#: Where the walk up aims: the door's face this far ahead at the lock, the lock this far to the base's left (the
#: middle of where the push works, `arm_turn.PUSH_*`), facing the door. (The simulation stops its walk in the near part,
#: `ACCEPT_*`, for the simulated gait's drift once stopped; sport mode stands where it stops, after its carry.)
#: 1 cm nearer than the middle (0.45): the dry walks up, with the odometry 10-15% short, landed 1-3 cm past where they aimed
#: (the gait's carry and its settling back), and at 0.45 one 10% short stopped 0.48-0.51 m off, outside the push's area.
WALK_TO_DOOR_M = 0.44
STANCE_LOCK_LEFT_M = round(sum(at.PUSH_LOCK_LEFT_M) / 2, 4)
#: Sport mode's odometry reports this share of a walk up: the refit after the walk put the door 43, 42, 58 and 63 mm
#: nearer than the look's fix in Lukas's four live walks up (2026-10-21), 12-15% of the 0.34-0.47 m walked -- growing
#: with the walk, as an odometry's scale error does (the look itself passed its lidar-against-tag check, under 3 cm).
#: The walk up is planned this much shorter on the odometry, so that the dog walks the whole of it. Estimated from four
#: runs; revisit as more come in.
ODOM_WALK_SCALE = plan.ODOM_WALK_SCALE
#: The walk up is over once the odometry has the base within this of where it was sent, facing to
#: `WALK_UP_ACCEPT_YAW_DEG`. (Superseded: a corrective step only for more than `WALK_UP_MIN_STEP_M`; now `_walk_up`'s
#: rule, the door pull's: when the gap is over 70% of the shortest step the gait makes.) A step at sport mode's
#: 0.18 m/s floor goes ~5 cm before its carry, so a smaller gap is overshot and settled back from -- the walk hunted at
#: the door six times in a row so (Lukas's first two live walks up). The refit's box must then be where the push works.
WALK_UP_ACCEPT_M = 0.04
WALK_UP_ACCEPT_YAW_DEG = 4.0
WALK_UP_MIN_STEP_M = 0.09
#: How long the gait carries on after a stop, at the speed it had: what the walk up first stops early by, then learnt
#: from each stop (as the door pull's walk in does). `plan.CARRY_S` (0.36 s, P2Dingo's 75 mm from 0.21 m/s in sport
#: mode's default gait) stopped the walk up 5-7 cm short every time on the dog, which this program walks in classic
#: walk: it carried on 2-3 cm from 0.18-0.25 m/s and settled back 1-1.5 cm (Lukas's live walks up, 2026-10-21; the
#: last stopped 6.9 cm short of where it was sent, under the then 9 cm a corrective step needed, and the refit had it
#: 0.52 m from the door).
WALK_UP_CARRY_S = 0.12
#: The refit may move the box this far before the attempt aborts (the door pull's is 6 cm): the walk's leftover error is
#: the refit's to take out, and the refit's box must still be where the push works.
REFIT_LIMIT_M = 0.10
#: Within this of the stance an axis is left alone rather than lifted to sport mode's floor (which made the first walk
#: up hunt across the stance at +-0.18 m/s, the dry run, 2026-10-21).
WALK_UP_HOLD_M = 0.02
WALK_UP_HOLD_DEG = 2.0
#: The walk up (Lukas, 2026-10-21, after the first look refused a dog 9 cm off: "it needs a walk up stage. I don't want
#: to position it myself"): from wherever the look finds the box within these -- the stance up to this far ahead
#: (or back), across and turned -- to the stance, closed on the odometry: forward, sideways and turning at once, as
#: P2Dingo's docking walks its diagonal sidesteps, each axis lifted out of sport mode's dead zone (`plan.WALK_FLOOR`),
#: the sideways floor raised when a sideways command does not move the dog (P2Dingo's: no side-step at 0.25 m/s from a
#: standstill), stopped early by the gait's carry, and stepped again until the stance is in `arm_turn.ACCEPT_*`.
WALK_UP_AHEAD_M = (-0.20, 0.70)
WALK_UP_SIDE_M = 0.40
WALK_UP_TURN_DEG = 25.0
WALK_UP_GAIN = 1.5
WALK_UP_YAW_GAIN = 1.5
WALK_UP_LAT_CAP = 0.40
WALK_UP_LAT_STALL_S = 1.5
WALK_UP_LAT_STALL_M = 0.01
WALK_UP_LAT_STEP = 0.05
WALK_UP_MAX_STEPS = 2
WALK_UP_TIMEOUT_S = 18.0      # under the attempt clock's limit on a walk in (`plan.WALK_IN_TIMEOUT_S`)
#: The box's door this far off square to the robot at most: the push was planned square (the walk keeps the look's
#: heading), and checked on the CPU for a box square to the base.
YAW_LIMIT_DEG = 4.0
#: The jaws shut through the push (no finger travel).
JAW_SHUT_M = 0.0
#: Where the hand (the jaw centre) waits through the unfold, the walk up and on the way home: (ahead, left, up) of the
#: base, in the base's own frame -- its height from where the base is, not the script frame's (whose box root is
#: assumed 0.358 m below the base: on the dog it was 0.391, and the hand sent to "0.24 m above the base" went to 0.207,
#: where the trunk proxy refused the move out to the key, 2026-10-21). The door pull's home holds the fingertips 0.42 m
#: ahead of the base, which is where the door is at the key's stance: walking up, the D1 hit the door and pushed the dog
#: back (Lukas's first live walk up). Here the fingertips are ~0.22 m ahead, the arm clear of the trunk proxy from
#: every IK start tried (the fold, the door pull's home, the dog's two home poses, zeros), and the move out to the
#: push's first pose clear.
KEY_HOME_IN_BASE = (0.20, 0.0, 0.30)
#: The phases' lengths at `speed_scale` 1, s.
ABOVE_S, TOUCH_S, HOLD_S, LIFT_S, CLEAR_S, HOME_S = 2.0, 2.0, 1.5, 1.5, 3.0, 3.0
TURN_S = at.turn_seconds()
#: How long the hand may be past `arm_turn.TRACK_LIMIT_MM` behind its command in the turn before it aborts.
STALL_S = 0.6
#: Asked before with `--step`.
CONFIRM_BEFORE = ("unfold", "walk_in", "key_out", "key_touch", "turn", "fold")
#: The turn planned before the hand goes in (`KeyAttempt._plan_arc`): the arm's IK every `ARC_STEP_DEG` of the key from
#: -45 to +45, each warm-started from the last, accepted only if no joint moves more than `ARC_JUMP_DEG` between two.
#: Warm-started from its own last solution alone, the 10 Hz loop's IK flipped the wrist mid-turn (J4 +84, J5 +60, J6
#: -94 degrees in one step, the dry run's first try, 2026-10-21): the push's hand passes near the D1's wrist singularity.
#: Through the push the loop's IK is seeded from the plan at the key's commanded angle, so it stays on that branch.
#: The hand's pitch onto the platform, planned per attempt: which one gives a turn clear of the wrist's singularity
#: depends on the base's height over the box (the CPU over 9 stances, 2026-10-21: at 0.32 m 60 degrees turned 9/9 and 50
#: none; at 0.353 m, the dog's, 50 turned 9/9 and 60 5/9 with Joint5 within 0.6 degrees of 0; at 0.38 m 50 5/9, 60 none).
#: `arm_turn.HAND_PITCH_DEG` (60) is the simulation's (its walker stood 0.31 m).
PITCHES_DEG = (50.0, 55.0, 60.0, 65.0, 70.0)
ARC_STEP_DEG = 2.5
ARC_JUMP_DEG = 20.0
ARC_SUBSTEPS = 10
#: The turn retimed from that plan (`KeyAttempt._retime_turn`): each step as long as the key at `TURN_KEY_DEG_S` or its
#: fastest joint at `TURN_JOINT_DEG_S` needs, whichever is longer, eased in and out (1.5x at the middle). Near 0 degrees the
#: wrist passes close to its singularity (Joint5 ~12 degrees) and Joints 4 and 6 swing ~90 degrees each way over 20 of
#: the key: at the simulated turn's 15 degrees/s of key that asked 66 degrees/s of them, past the arm link's 30
#: (`plan.ARM_SPEED_DEG_S`), and the dry run's arm fell 39 degrees behind. At 18 (27 at the middle) it kept up but
#: lagged 9.7 degrees, at the 10 that aborts as loaded (`plan.ARM_LAG_DEG`); at 15 (22.5) it lags less.
TURN_KEY_DEG_S = 10.0
TURN_JOINT_DEG_S = 15.0
#: The turn waits for the arm: while a joint is more than `TURN_CATCH_UP_DEG` behind its goal the turn's clock holds,
#: (its stall abort is the key's progress, below). The plan is for the stance the look predicts; the
#: dog stops a few cm and degrees off it, and near the wrist's singularity that changes the joints' speeds (a dry walk
#: up with the box 6 degrees off square lagged 11 degrees, past the 10 that aborts as loaded).
#: On the dog (Lukas's first live turn, 2026-10-21) the key followed the command to 1-2 degrees from -45 to +14 while
#: Joint4, the wrist joint the singularity makes fast, ran 5-7 degrees behind; at 5 degrees and 3 s of holds in all, the
#: turn aborted mid-way as "not turning". Now the clock holds only past `TURN_CATCH_UP_DEG`, and the turn aborts only
#: when the fingers' measure of the key has not advanced `TURN_PROGRESS_DEG` in `TURN_STALL_S` -- a key that is not
#: turning, not an arm that is slow.
TURN_CATCH_UP_DEG = 8.0
TURN_PROGRESS_DEG = 2.0
TURN_STALL_S = 3.0
ARC_PHASES = {"key_above", "key_touch", "turn", "key_hold", "key_lift"}
#: The way off the key (`key_clear`, the recovery's too): from above the platform straight up, the hand's orientation
#: held, until the jaw centre is this far above the base; only then home. The head camera's case and bracket
#: (`head_camera.CLEARANCE_BOX_BASE_M`) stand 4 cm past the trunk proxy's front, 7-11 cm above the base. Backing off along
#: the hand's approach (`key_back`, 8 cm) and then straight to home, the fingertips went through the case on the dog
#: after Lukas's first full turn (2026-10-21): at +45 degrees the approach points right and down, so backing off moved the
#: wrist 5 cm towards the camera, and the line home from there passed in front of it at its height. Straight up from
#: above the key to 0.20 m, the replayed turn's arm keeps 31 mm from the case on the way up and 80 on the way home (the
#: clearance proxy's margins, `camera_margin`; that run's way home had -30).
KEY_CLEAR_ABOVE_BASE_M = 0.20
#: The box the arm's links keep off (`camera_margin`): the dry tests hold the whole attempt to it.
CAMERA_BOX_B = head_camera.CLEARANCE_BOX_BASE_M


# --- the look -------------------------------------------------------------------------------------------------------

def push_stance(box) -> tuple[float, float, float]:
    """The push's stance (x, y, yaw) in the frame `box` is given in: facing the door, its face `WALK_TO_DOOR_M` ahead at
    the lock and the lock `STANCE_LOCK_LEFT_M` to the left (`arm_turn.stance_base`)."""
    box = np.asarray(box)
    lock = box[:3, :3] @ np.asarray(kg.LOCK) + box[:3, 3]
    normal = box[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX)
    right = box[:3, :3] @ np.asarray(kg.LEVER_LEVEL_BOX)
    base = lock + WALK_TO_DOOR_M * normal + STANCE_LOCK_LEFT_M * right
    return float(base[0]), float(base[1]), math.atan2(-normal[1], -normal[0])


def stance_for(box_look) -> tuple[float, float, float]:
    """Where the walk up is sent (x, y, yaw) in the look's frame, from the box's level pose there: the push's stance --
    facing the door, its face `WALK_TO_DOOR_M` ahead at the lock and the lock `STANCE_LOCK_LEFT_M` to the left
    (`arm_turn.stance_base`) -- as the odometry will report it (`ODOM_WALK_SCALE`)."""
    x, y, yaw = push_stance(box_look)
    return ODOM_WALK_SCALE * x, ODOM_WALK_SCALE * y, yaw


def lock_from_base(fix: plan.Fix) -> tuple[float, float, float]:
    """The lock's point on the door from the stance (the script frame's origin, facing +x): ahead m, left m, and the
    door's yaw off square (deg)."""
    lock = perception.site_pose(fix.site) @ np.array([*kg.LOCK, 1.0])
    off_square = math.degrees(plan.wrap(math.radians(fix.site.yaw_deg) - math.pi))
    return float(lock[0]), float(lock[1]), off_square


def lock_from(fix: plan.Fix, base) -> tuple[float, float, float]:
    """The lock from a base at (x, y, yaw) in the script frame: ahead m, left m, and the base's yaw from facing the
    door (deg)."""
    lock = perception.site_pose(fix.site) @ np.array([*kg.LOCK, 1.0])
    x, y, yaw = base
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = lock[0] - x, lock[1] - y
    facing = math.radians(fix.site.yaw_deg) - math.pi
    return c * dx + s * dy, -s * dx + c * dy, math.degrees(plan.wrap(facing - yaw))


class KeyFix(plan.Fix):
    """The door pull's fix with the key's range check."""

    #: The stance (x, y, yaw) in the look's frame, set by `make_key_fix` (not kept through the refit, which needs it no
    #: more: the walk is over by then).
    stance_look: tuple = (0.0, 0.0, 0.0)

    def in_range(self, check_tag: bool = True) -> tuple[bool, str]:
        """Whether the push can be made: at the look (`check_tag`), the tag on a box's door and a walk up the robot can
        make (`WALK_UP_*`); after the walk up, the lock where the push works (`arm_turn.PUSH_*`). And how to move the
        dog if not."""
        if check_tag and not self.fit.ok:
            return False, f"no door face round the tag in the lidar ({self.fit.why}): not the box, or not in view"
        if check_tag and (abs(self.fit.depth) > plan.FACE_BEHIND_TAG_LIMIT_M or self.beyond_edges > plan.BEYOND_LIMIT):
            return False, (f"the tag is not on a door the box's size: the lidar's face is {100 * self.fit.depth:+.0f} cm "
                           f"from it and carries on past the door's edges ({self.beyond_edges:.0%})")
        problems = []
        if check_tag:
            x, y, yaw = self.stance_look
            turn = math.degrees(yaw)
            if x > WALK_UP_AHEAD_M[1]:
                problems.append(f"forward {x - WALK_UP_AHEAD_M[1] + 0.05:.2f} m")
            if x < WALK_UP_AHEAD_M[0]:
                problems.append(f"back {WALK_UP_AHEAD_M[0] - x + 0.05:.2f} m")
            if abs(y) > WALK_UP_SIDE_M:
                problems.append(f"{'left' if y > 0 else 'right'} {abs(y) - WALK_UP_SIDE_M + 0.05:.2f} m")
            if abs(turn) > WALK_UP_TURN_DEG:
                problems.append(f"turn {'left' if turn > 0 else 'right'} {abs(turn) - WALK_UP_TURN_DEG + 5:.0f} deg")
            text = (f"the push's stance {x:.2f} m ahead, {y:+.2f} m left, turned {turn:+.0f} deg: a walk up of "
                    f"{math.hypot(x, y):.2f} m")
            return not problems, text + ("" if not problems else "; move the dog " + ", ".join(problems))
        ahead, left, off_square = lock_from_base(self)
        if not at.PUSH_DOOR_AHEAD_M[0] <= ahead <= at.PUSH_DOOR_AHEAD_M[1]:
            problems.append(f"{'back' if ahead < at.PUSH_DOOR_AHEAD_M[0] else 'forward'} {abs(ahead - WALK_TO_DOOR_M):.2f} m")
        lo, hi = at.PUSH_LOCK_LEFT_M
        if not lo <= left <= hi:
            problems.append(f"{'left' if left > hi else 'right'} {abs(left - STANCE_LOCK_LEFT_M):.2f} m")
        if abs(off_square) > YAW_LIMIT_DEG:
            problems.append(f"turn {'left' if off_square > 0 else 'right'} {abs(off_square):.0f} deg")
        text = (f"lock {ahead:.2f} m ahead, {left:+.2f} m left of the stance, door {off_square:+.1f} deg off square, "
                "after the walk")
        return not problems, text + ("" if not problems else "; move the dog " + ", ".join(problems))


def make_key_fix(look_odom, frames, scans) -> KeyFix | None:
    """The look's fix with the script frame at the push's stance (`stance_for`): the walk up's target."""
    stance = {}

    def stance_from(box_look):
        stance["look"] = stance_for(box_look)
        return stance["look"]

    fix = plan.make_fix(look_odom, frames, scans, stance_for=stance_from, fix_class=KeyFix)
    if fix is not None:
        fix.stance_look = stance["look"]
    return fix


# --- the phases ------------------------------------------------------------------------------------------------------

def _box(st) -> np.ndarray:
    return perception.site_pose(st["box_site"])


def hand_rotation(st, theta_deg: float) -> np.ndarray:
    """Link6's axes, script frame, with the key at `theta_deg`, from the fix, at the attempt's planned pitch (`st`'s
    "pitch", `arm_turn.HAND_PITCH_DEG` without one)."""
    return _box(st)[:3, :3] @ at.hand_rotation(theta_deg, st.get("pitch", at.HAND_PITCH_DEG))


def jaw_point(st, where: str, theta_deg: float) -> tuple[float, float, float]:
    """The jaw centre (the IK's point), script frame, for the fingertips' edge at `where` (`arm_turn.tip_target`)."""
    box = _box(st)
    tip = box[:3, :3] @ at.tip_target(where, theta_deg) + box[:3, 3]
    return tuple(float(v) for v in at.jaw_centre(tip, hand_rotation(st, theta_deg)))


def key_phases(stage: str, speed_scale: float = 1.0, fold: bool = True) -> list:
    """The phases after the look of an attempt that goes as far as `stage`, then off the key, home and folded."""
    if stage not in STAGES[2:]:
        raise ValueError(f"a key attempt's stage is one of {STAGES[2:]}, not {stage!r}")
    Phase = demo_script.Phase
    k = speed_scale
    home = lambda st, u: key_home(st["base"], st["base_z"])
    z = lambda deg: (lambda st, *u: tuple(hand_rotation(st, deg)[:, 2]))
    x = lambda deg: (lambda st, *u: tuple(hand_rotation(st, deg)[:, 0]))
    push = lambda name, seconds, where, deg: Phase(name, seconds * k, target=lambda st: jaw_point(st, where, deg),
                                                   jaw_m=JAW_SHUT_M, approach=z(deg), object_axis=x(deg), servo=True)
    start, end = at.START_DEG, at.END_DEG
    phases = [Phase("unfold", plan.UNFOLD_S, path=home, jaw_m=JAW_SHUT_M)]
    if stage == "unfold":
        phases.append(Phase("pause", plan.PAUSE_S, path=home, jaw_m=JAW_SHUT_M))
    index = STAGES.index(stage)
    if index >= STAGES.index("walk"):
        phases += [Phase("walk_in", plan.WALK_IN_S, path=home, jaw_m=JAW_SHUT_M, base=lambda st, u: (0.0, 0.0, 0.0)),
                   Phase("walk_settle", plan.WALK_SETTLE_S, path=home, jaw_m=JAW_SHUT_M)]
    if index >= STAGES.index("reach"):
        last = end if stage == "turn" else start
        phases.append(push("key_out", plan.UNFOLD_S, "out", start))
        phases.append(push("key_above", ABOVE_S, "above", start))
        if index >= STAGES.index("touch"):
            phases.append(push("key_touch", TOUCH_S, "pressed", start))
        if stage == "turn":
            angle = lambda u: at.turn_angle(u)
            phases += [Phase("turn", TURN_S * k, path=lambda st, u: jaw_point(st, "pressed", angle(u)), jaw_m=JAW_SHUT_M,
                             approach_path=lambda st, u: tuple(hand_rotation(st, angle(u))[:, 2]),
                             object_axis_path=lambda st, u: tuple(hand_rotation(st, angle(u))[:, 0]), servo=True),
                       push("key_hold", HOLD_S, "resting", end)]
        if index >= STAGES.index("touch"):
            phases.append(push("key_lift", LIFT_S, "above", last))
        phases.append(Phase("key_clear", CLEAR_S * k, target=lambda st: cleared(st, jaw_point(st, "above", last)),
                            jaw_m=JAW_SHUT_M, approach=z(last), object_axis=x(last), servo=True))
        phases.append(Phase("home", HOME_S * k, path=home, jaw_m=JAW_SHUT_M))
    if fold:
        phases.append(Phase("fold", plan.FOLD_S, path=home, jaw_m=JAW_SHUT_M))
    return phases


def cleared(st, point) -> tuple[float, float, float]:
    """`point` (script frame) raised straight up to `KEY_CLEAR_ABOVE_BASE_M` above the base, if below it."""
    x, y, z = point
    return (float(x), float(y), max(float(z), st["base_z"] + KEY_CLEAR_ABOVE_BASE_M))


def camera_margin(joints, q) -> float:
    """How far (m) the arm at `q` (radians) is from the head camera's box (`CAMERA_BOX_B`): the smallest gap between its
    link segments and the box, less the clearance proxy's link radius (`grasp.proxy_margin`'s measure). Negative: in."""
    from position_only.workspace import LINK_RADIUS_M

    points = _chain_points(joints, q)[0]
    low, high = np.array(CAMERA_BOX_B)[:, 0], np.array(CAMERA_BOX_B)[:, 1]
    outside = np.linalg.norm(np.maximum(np.maximum(low - points, points - high), 0.0), axis=1)
    return float(outside.min()) - LINK_RADIUS_M


def key_home(base, base_z: float) -> tuple[float, float, float]:
    """`KEY_HOME_IN_BASE` for a base at (x, y, yaw) and height `base_z` in the script frame."""
    x, y, z = KEY_HOME_IN_BASE
    c, s = math.cos(base[2]), math.sin(base[2])
    return (base[0] + c * x - s * y, base[1] + s * x + c * y, base_z + z)


def recovery_phases(speed_scale: float = 1.0) -> list:
    """After an abort at the key: from where the hand measures itself, up off the platform along its normal, then
    straight up clear of the head camera (`cleared`), its orientation held; then home and fold. The base stands
    throughout."""
    Phase = demo_script.Phase
    held = lambda key: (lambda st: st["held"]()[key])
    hand = dict(approach=held("approach"), object_axis=held("axis"), servo=True)
    home = lambda st, u: key_home(st["base"], st["base_z"])
    return [Phase("key_up", LIFT_S * speed_scale, target=held("up"), jaw_m=JAW_SHUT_M, **hand),
            Phase("key_clear", CLEAR_S * speed_scale, target=lambda st: cleared(st, st["held"]()["up"]),
                  jaw_m=JAW_SHUT_M, **hand),
            Phase("home", HOME_S * speed_scale, path=home, jaw_m=JAW_SHUT_M),
            Phase("fold", plan.FOLD_S, path=home, jaw_m=JAW_SHUT_M)]


# --- the walk up ------------------------------------------------------------------------------------------------------

class WalkUp:
    """The walk up to the stance (the script frame's origin, facing +x) through the `walk_in` phase, on all three axes at
    once (`WALK_UP_*`): the key's, and the door pull's after the key (`DoorAfterKey`). A mixin for a `plan.Attempt`."""

    def _base(self, cmd, actual, dt):
        if cmd.phase == "walk_in":
            return self._walk_up(actual, dt)
        return super()._base(cmd, actual, dt)

    def _close_enough(self, base) -> bool:
        """Standing at `base` (script frame, the odometry's), whether the walk up is done (`WALK_UP_ACCEPT_*`)."""
        x, y, yaw = base
        return math.hypot(x, y) <= WALK_UP_ACCEPT_M and abs(math.degrees(plan.wrap(yaw))) <= WALK_UP_ACCEPT_YAW_DEG

    def _walk_up(self, actual, dt):
        """The walk up to the stance (`WALK_UP_*`): walk, stop early by the carry (the distance left within what the
        gait carries on at this speed, the turn within its own), stand and measure, step again if not close enough
        (`_close_enough`). Returns (command, walking)."""
        stand = ((0.0, 0.0, 0.0), False)
        w = self._walk
        if w is None:
            w = self._walk = {"state": "done" if self._close_enough(actual) else "walking", "t": 0.0, "steps": 0,
                              "total": 0.0, "lat_floor": plan.WALK_FLOOR[1], "cmd": (0.0, 0.0, 0.0),
                              "lat_from": actual, "lat_t": 0.0, "carry_s": WALK_UP_CARRY_S, "stop_at": actual,
                              "stop_speed": 0.0}
            self.events.append({"phase": "walk_up", "from": [round(v, 3) for v in actual]})
        x, y, yaw = actual
        if w["state"] == "walking":
            w["t"] += dt
            w["total"] += dt
            vx, vy, wz = w["cmd"]
            left_m = math.hypot(x, y)
            there = (left_m <= max(WALK_UP_HOLD_M, math.hypot(vx, vy) * w["carry_s"])
                     and abs(math.degrees(plan.wrap(yaw))) <= max(WALK_UP_HOLD_DEG, math.degrees(abs(wz) * w["carry_s"])))
            if (w["t"] >= plan.WALK_IN_MIN_STEP_S and there) or w["total"] > WALK_UP_TIMEOUT_S:
                w.update(state="coasting", t=0.0, stop_at=actual, stop_speed=math.hypot(vx, vy))
                return stand
            w["cmd"] = self._walk_up_command(actual, w, dt)
            return w["cmd"], True
        if w["state"] == "coasting":
            w["t"] += dt
            if w["t"] < plan.WALK_IN_COAST_S:
                return stand
            ok = self._close_enough(actual)
            # The carry this stop shows, for the next: how far on it went from where it stopped, at the speed it had.
            carried = math.hypot(actual[0] - w["stop_at"][0], actual[1] - w["stop_at"][1])
            if w["stop_speed"] > 0.05:
                w["carry_s"] = min(max(carried / w["stop_speed"], 0.0), 0.5)
            self.events.append({"phase": "walk_up", "step": w["steps"], "close_enough": ok, **self._walk_up_note(actual),
                                "left_m": round(math.hypot(*actual[:2]), 3), "carried_m": round(carried, 3),
                                "carry_s": round(w["carry_s"], 2), "lat_floor": round(w["lat_floor"], 2)})
            # A step is worth taking when the gap is over 70% of the shortest the gait makes (`plan._walk_in`'s rule).
            shortest = plan.WALK_FLOOR[0] * (w["carry_s"] + plan.WALK_IN_MIN_STEP_S)
            worth = (math.hypot(*actual[:2]) > 0.7 * shortest
                     or abs(math.degrees(plan.wrap(actual[2]))) > WALK_UP_ACCEPT_YAW_DEG)
            if not ok and worth and w["steps"] < WALK_UP_MAX_STEPS and w["total"] <= WALK_UP_TIMEOUT_S:
                w.update(state="walking", t=0.0, steps=w["steps"] + 1, cmd=(0.0, 0.0, 0.0), lat_from=actual, lat_t=0.0)
                return stand
            w["state"] = "done"          # close enough, or out of steps: the refit's range check decides
        return stand

    def _walk_up_command(self, actual, w, dt):
        """Towards the stance (the script frame's origin, facing +x) on all three axes at once, in the base's frame,
        scaled up until its largest part (against each axis's floor) leaves sport mode's dead zone, then capped. The
        sideways floor goes up a step when a mostly sideways command has not moved the dog for `WALK_UP_LAT_STALL_S`."""
        x, y, yaw = actual
        c, s = math.cos(yaw), math.sin(yaw)
        ex, ey = c * -x + s * -y, -s * -x + c * -y
        near = math.hypot(ex, ey) < WALK_UP_HOLD_M
        turned = abs(math.degrees(plan.wrap(-yaw))) < WALK_UP_HOLD_DEG
        v = [0.0 if near else WALK_UP_GAIN * ex, 0.0 if near else WALK_UP_GAIN * ey,
             0.0 if turned else WALK_UP_YAW_GAIN * plan.wrap(-yaw)]
        if near and turned:
            return (0.0, 0.0, 0.0)
        floors = (plan.WALK_FLOOR[0], w["lat_floor"], plan.WALK_FLOOR[2])
        caps = (plan.WALK_CAP[0], max(plan.WALK_CAP[1], w["lat_floor"]), plan.WALK_CAP[2])
        ratios = [abs(a) / f for a, f in zip(v, floors)]
        reach = max(ratios)
        if 0.0 < reach < 1.0:
            v = [a / reach for a in v]
        over = max(abs(a) / cap for a, cap in zip(v, caps))
        if over > 1.0:
            v = [a / over for a in v]
        if ratios[1] >= max(ratios[0], ratios[2]):
            w["lat_t"] += dt
            fx, fy, fyaw = w["lat_from"]
            moved = abs(-math.sin(fyaw) * (x - fx) + math.cos(fyaw) * (y - fy))
            if moved > WALK_UP_LAT_STALL_M:
                w["lat_from"], w["lat_t"] = actual, 0.0
            elif w["lat_t"] > WALK_UP_LAT_STALL_S and w["lat_floor"] < WALK_UP_LAT_CAP:
                w["lat_floor"] = min(WALK_UP_LAT_CAP, w["lat_floor"] + WALK_UP_LAT_STEP)
                w["lat_from"], w["lat_t"] = actual, 0.0
                self.events.append({"phase": "walk_up", "lat_floor_raised_to": w["lat_floor"]})
        else:
            w["lat_from"], w["lat_t"] = actual, 0.0
        return tuple(float(a) for a in v)

    def _walk_up_note(self, actual) -> dict:
        """What the record says of where a stop left the base (`actual`, script frame), beside how far it is off."""
        return {}


# --- the attempt -----------------------------------------------------------------------------------------------------

@dataclass
class KeyAttempt(WalkUp, plan.Attempt):
    """The door pull's attempt (`plan.Attempt`: the clock, the walk in, the refit, the IK, the aborts) with the key's
    phases, stance, recovery and check. `step` at 10 Hz."""
    speed_scale: float = 1.0

    joint_phases = {"unfold", "key_out", "fold"}
    dwell_phases = frozenset()
    position_only_phases = {"walk_in", "walk_settle", "pause", "home"}
    #: The arm's lag abort (`plan.ARM_LAG_DEG`) and, in the turn, the hand's (`STALL_S`).
    holding_phases = {"key_touch", "turn", "key_hold"}
    #: Where an abort lifts the hand off the key before folding.
    at_the_lever_phases = {"key_above", "key_touch", "turn", "key_hold", "key_lift", "key_clear"}
    walking_phases = {"walk_in"}
    release_held_phases = {"key_up", "key_clear"}
    refit_limit_m = REFIT_LIMIT_M
    confirm_before = CONFIRM_BEFORE
    reads_door = False

    def _make_phases(self) -> list:
        return key_phases(self.stage, self.speed_scale, self.fold)

    def state(self, base_odom) -> dict:
        """The parent's, and the base's height in the script frame (`key_home`)."""
        st = super().state(base_odom)
        st["base_z"] = float(self.base_script(base_odom)[2, 3])
        st["pitch"] = self._pitch
        return st

    def _prepare(self) -> None:
        # The push's hand: `object_axis` is Link6's x, so the grasp is (pitch 0, roll +1) of `hand_rotation`.
        self.arm.grasp, self.arm.grasp_note = (0.0, 1), "the key push's hand (pitch 0, roll +1)"
        self._turn = {"lag_mm": 0.0, "since": None, "measured_deg": None, "verified": None}
        self._blocked = ""
        self._arc: list | None = None
        self._arc_deg = at.START_DEG
        self._turn_times: np.ndarray | None = None
        self._push_plan: dict | None = None
        self._pitch = at.HAND_PITCH_DEG
        self._turn_held_s = 0.0
        self._turn_best = (None, -math.inf)          # (when, the fingers' furthest measure of the key in the turn)
        self.camera_gaps_mm: dict[str, float] = {}   # the arm's smallest gap to the head camera, per phase
        self._planned = STAGES.index(self.stage) < STAGES.index("reach")

    def _grasp_rotation(self, to_base):
        return None             # unfolded and walked in with the jaw centre alone: the push's hand comes at `key_out`

    def _recovery_phases(self) -> list:
        return recovery_phases(self.speed_scale)

    def _solve_joint_goal(self, phase: str, end_b, to_base) -> np.ndarray:
        """`key_out`: the planned first push pose (`_plan_push`) solved again where the base now stands, the planned
        pose the only seed (one solve, a few ms: the arm is streaming, and the arm link holds it after 0.5 s without a
        command; planning the whole turn again here took 34-78 ms on the PC, several times that on the dog); the
        joint-space move to it checked clear of the trunk. The parent's for the unfold and the fold."""
        if phase != "key_out":
            return super()._solve_joint_goal(phase, end_b, to_base)
        start = self.arm.q.copy()
        chosen = self._push_plan
        if chosen is None:
            self._blocked = self._blocked or "the push was not planned"
            return start
        result = solve_link6(self.arm.joints, self.arm.links, end_b, self._push_rotation(to_base, at.START_DEG),
                             [chosen["q"]], self.arm.offset)
        q = np.asarray(result.q, dtype=float) if result is not None else chosen["q"]
        clear = bool(path_clear(self.arm.joints, start, q)[0])
        turn_s = self._retime_turn()
        self.events.append({"phase": "key_out", "solved_here": result is not None, "path_clear": clear,
                            "turn_s": round(turn_s, 2)})
        if not clear:
            self._blocked = "the move to the key's first pose crosses the trunk proxy"
            return start
        self.arm.q = q
        return q

    def _st(self) -> dict:
        return {"box_site": self.fix.site, "pitch": self._pitch}

    def _plan_push(self, stance_s) -> None:
        """Before anything moves, from the stance (`stance_s`, the base where the walk up should leave it, script
        frame): for each hand pitch in `PITCHES_DEG`, the first push pose from the scripted grip's seeds whose whole turn
        traces without a flip; of those, the one whose wrist stays furthest from its singularity (the largest smallest
        |Joint5|) kept, and its pitch used for the attempt. None: `_blocked` says why, and the attempt stops at its first
        step."""
        to_base = plan.invert(stance_s)
        candidates, solved = [], 0
        for pitch in PITCHES_DEG:
            self._pitch = pitch
            end_b = plan.transform(to_base, [jaw_point(self._st(), "out", at.START_DEG)])[0]
            rot = self._push_rotation(to_base, at.START_DEG)
            for seed in _seeds(end_b, [self.arm.q]):
                result = solve_link6(self.arm.joints, self.arm.links, end_b, rot, [seed], self.arm.offset)
                if result is None:
                    continue
                solved += 1
                q = np.asarray(result.q, dtype=float)
                traced = self._plan_arc(q, to_base)
                if traced is not None:
                    candidates.append({"q": q, "arc": traced[0], "min_j5_deg": traced[1], "pitch_deg": pitch})
                    break
        self.events.append({"phase": "plan_push", "poses_solved": solved,
                            "turns": {c["pitch_deg"]: round(c["min_j5_deg"], 1) for c in candidates}})
        if not candidates:
            self._pitch = at.HAND_PITCH_DEG
            self._blocked = ("no arm pose in front of the key from which the whole turn solves without a wrist flip, at "
                             f"any of the hand's pitches {PITCHES_DEG} ({solved} poses solved)")
            return
        self._push_plan = max(candidates, key=lambda c: c["min_j5_deg"])
        self._pitch = self._push_plan["pitch_deg"]
        self._arc = self._push_plan["arc"]

    def _push_rotation(self, to_base, theta_deg: float) -> np.ndarray:
        """Link6's rotation, base frame, for the push with the key at `theta_deg`."""
        hand = to_base[:3, :3] @ hand_rotation(self._st(), theta_deg)
        return decoupled_arm.hand_rotation(hand[:, 2], hand[:, 0], 0.0, 1)

    def _solve_pressed(self, to_base, deg: float, q0):
        point = plan.transform(to_base, [jaw_point(self._st(), "pressed", deg)])[0]
        result = d1_ik.solve(self.arm.joints, point, self._push_rotation(to_base, deg), q0=q0, body="Link6",
                             offset=self.arm.offset, max_iterations=200)
        ok = (result.position_error_m <= decoupled_arm.POSE_ACCEPT_M
              and result.rotation_error_rad <= decoupled_arm.POSE_ACCEPT_RAD)
        return (np.asarray(result.q, dtype=float) if ok else None)

    def _plan_arc(self, q_out, to_base):
        """([(key deg, q)], the smallest |Joint5| on it, deg) of the pressed hand over the whole turn from the first push
        pose `q_out`, or None. A step that jumps a joint more than `ARC_JUMP_DEG` is traced again in `ARC_SUBSTEPS`
        steps, each held to a `ARC_SUBSTEPS`-th of it: fast near the wrist's singularity is allowed, a flip is not."""
        arc, q = [], np.asarray(q_out, dtype=float)
        degs = np.arange(at.START_DEG, at.END_DEG + 1e-6, ARC_STEP_DEG)
        for k, deg in enumerate(degs):
            nxt = self._solve_pressed(to_base, float(deg), q)
            if nxt is None:
                return None
            if arc and float(np.degrees(np.max(np.abs(nxt - q)))) > ARC_JUMP_DEG:
                for sub in np.linspace(degs[k - 1], deg, ARC_SUBSTEPS + 1)[1:]:
                    step_q = self._solve_pressed(to_base, float(sub), q)
                    if step_q is None or float(np.degrees(np.max(np.abs(step_q - q)))) > ARC_JUMP_DEG / ARC_SUBSTEPS * 2:
                        return None
                    q = step_q
                    arc.append((float(sub), q))
                continue
            q = nxt
            arc.append((float(deg), q))
        return arc, float(min(abs(np.degrees(row[1][4])) for row in arc))

    def _retime_turn(self) -> float:
        """The turn's phase replaced by one timed from the plan (`TURN_*_DEG_S`); its length, s."""
        import dataclasses

        degs = np.array([d for d, _ in self._arc])
        qs = np.degrees(np.array([q for _, q in self._arc]))
        seconds = np.maximum(np.abs(np.diff(degs)) / TURN_KEY_DEG_S,
                             np.max(np.abs(np.diff(qs, axis=0)), axis=1) / TURN_JOINT_DEG_S)
        self._turn_times = np.concatenate([[0.0], np.cumsum(seconds)])
        names = [p.name for p in self.script.phases]
        if "turn" in names:
            i = names.index("turn")
            self.script.phases[i] = dataclasses.replace(
                self.script.phases[i], duration_s=float(self._turn_times[-1]) * self.speed_scale,
                path=lambda st, u: jaw_point(st, "pressed", self.turn_deg(u)),
                approach_path=lambda st, u: tuple(hand_rotation(st, self.turn_deg(u))[:, 2]),
                object_axis_path=lambda st, u: tuple(hand_rotation(st, self.turn_deg(u))[:, 0]))
        return float(self._turn_times[-1]) * self.speed_scale

    def turn_deg(self, progress: float) -> float:
        """The key's commanded angle at `progress` through the turn: the plan's timing, eased in and out (else
        `arm_turn.turn_angle`'s)."""
        if self._turn_times is None:
            return at.turn_angle(progress)
        u = min(1.0, max(0.0, progress))
        s = u * u * (3 - 2 * u)
        return float(np.interp(s * self._turn_times[-1], self._turn_times, [d for d, _ in self._arc]))

    def _goal_servo(self, cmd, base_s, measured, st):
        """The parent's, its IK seeded through the push from the planned turn at the key's commanded angle."""
        if cmd.phase == "turn":
            self._arc_deg = self.turn_deg(cmd.progress)
        elif cmd.phase == "key_hold":
            self._arc_deg = at.END_DEG
        if self._arc is not None and cmd.phase in ARC_PHASES and self._joint_goal is None:
            nearest = min(self._arc, key=lambda row: abs(row[0] - self._arc_deg))
            self.arm.q = nearest[1].copy()
        return super()._goal_servo(cmd, base_s, measured, st)

    def _capture(self, servo_deg, base_s) -> None:
        """The parent's held pose, and the key's way off: up the platform's normal (in the hand, `(-cos p, 0, -sin p)`
        of Link6); `recovery_phases` then raises it clear of the camera."""
        super()._capture(servo_deg, base_s)
        p = math.radians(self._pitch)
        up = self.held_rotation @ np.array([-math.cos(p), 0.0, -math.sin(p)])
        raised = np.asarray(self._held["hand"]) + at.CLEARANCE_M * up
        self._held["up"] = tuple(float(v) for v in raised)

    # --- what the robot measures of the key ------------------------------------------------------------------------

    def hand_now(self, servo_deg, base_s) -> tuple[np.ndarray, np.ndarray]:
        """(the jaw centre, the fingertips' edge), script frame, by forward kinematics of `servo_deg`."""
        q = d1_ik.from_servo_deg(servo_deg)[0]
        pos, link6 = d1_ik.tool_pose(self.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
        jaw = pos + link6 @ np.asarray(JAW_CENTRE_LINK6)
        edge = pos + link6 @ np.asarray(at.TIP_EDGE_LINK6)
        return plan.transform(base_s, [jaw])[0], plan.transform(base_s, [edge])[0]

    def key_deg_measured(self, edge_script) -> float:
        """The key's angle the fingertips' edge implies, resting on the estimated platform `PUSH_REACH_M` out
        (`handle.measured_theta`, in numpy)."""
        box = perception.site_pose(self.fix.site)
        r = box[:3, :3].T @ (np.asarray(edge_script) - (box[:3, :3] @ np.asarray(kg.LOCK) + box[:3, 3]))
        polar = math.atan2(-r[2], r[1])                      # in the turn's plane: (+Y the key level, -Z down)
        return math.degrees(polar + math.atan2(at.KEY_TOP_M, at.PUSH_REACH_M))

    def turn_check(self) -> dict:
        """The robot's own check: the hand's worst lag behind its command in the turn, the key's angle at the hold, and
        whether that says unlocked (`arm_turn.TRACK_LIMIT_MM`, the simulation's unlock angle)."""
        return dict(self._turn)

    def key_deg_shown(self) -> float:
        """The key's angle to draw: measured while the fingers are on it, the start before, the last measure after."""
        last = self._turn.get("measured_deg")
        return at.START_DEG if last is None else max(at.START_DEG, min(at.END_DEG, last))

    def step(self, obs: plan.Observation, dt: float = 0.1) -> plan.Command:
        if not self._planned:
            # The push planned once, at the first step: the base's height in the script frame is known, and nothing has
            # been sent to the arm yet (the plan takes ~0.3 s on the PC, more on the dog; the arm link holds an arm left
            # without a command for 0.5 s once it has had one).
            # The stance it plans from is the push's for the box as the look has it (the walk is sent short of it by
            # the odometry's error, `ODOM_WALK_SCALE`, and the refit puts the box where the dog then stands).
            self._planned = True
            x, y, yaw = push_stance(perception.site_pose(self.fix.site))
            stance = plan.pose(plan.rot_z(yaw), (x, y, float(self.base_script(obs.base_odom)[2, 3])))
            self._plan_push(stance)
        before = self.phase
        held = False
        if before == "turn" and obs.arm_servo_deg is not None and self.last_goal is not None:
            measured = self._turn.get("measured_deg")
            when, best = self._turn_best
            if when is None or (measured is not None and measured >= best + TURN_PROGRESS_DEG):
                self._turn_best = (obs.now_s, measured if measured is not None else best)
            elif obs.now_s - when > TURN_STALL_S:
                return plan.Command(before, abort=(f"the key is not turning: the fingers have it at {measured:+.0f} deg, "
                                                   f"not {TURN_PROGRESS_DEG:.0f} deg further in {TURN_STALL_S:.0f} s"))
            lag_deg = float(np.max(np.abs(np.asarray(self.last_goal) - np.asarray(obs.arm_servo_deg))))
            if lag_deg > TURN_CATCH_UP_DEG:
                held, self._turn_held_s = True, self._turn_held_s + dt
        out = super().step(obs, 0.0 if held else dt)
        if obs.arm_servo_deg is not None:
            self._camera_gap(before, out.phase, obs.arm_servo_deg)
        if obs.arm_servo_deg is None or self._base_s is None or out.abort or out.finished:
            return out
        jaw, edge = self.hand_now(obs.arm_servo_deg, self._base_s)
        on_key = out.phase in ("key_touch", "turn", "key_hold")
        if on_key:
            self._turn["measured_deg"] = round(self.key_deg_measured(edge), 2)
        if out.phase == "turn" and out.hand_target_script is not None:
            lag = 1000.0 * float(np.linalg.norm(np.asarray(out.hand_target_script) - jaw))
            self._turn["lag_mm"] = round(max(self._turn["lag_mm"], lag), 1)
            if lag > at.TRACK_LIMIT_MM:
                self._turn["since"] = obs.now_s if self._turn["since"] is None else self._turn["since"]
                if obs.now_s - self._turn["since"] >= STALL_S:
                    out.abort = (f"the key is not turning: the hand is {lag:.0f} mm behind its command "
                                 f"(the key at {self._turn['measured_deg']:+.0f} deg by the fingers)")
            else:
                self._turn["since"] = None
        if before == "key_hold" and out.phase != "key_hold":
            measured = self._turn["measured_deg"]
            self._turn["verified"] = bool(measured is not None and measured >= kg.UNLOCK_DEG
                                          and self._turn["lag_mm"] < at.TRACK_LIMIT_MM)
            self._turn["held_s"] = round(self._turn_held_s, 2)
            self.events.append({"phase": "turn_check", **self.turn_check()})
        return out

    def _camera_gap(self, before: str, phase: str, servo_deg) -> None:
        """The arm's smallest `camera_margin` through each phase, from the measured joints: an event as the phase ends."""
        gap = 1000.0 * camera_margin(self.arm.joints, d1_ik.from_servo_deg(servo_deg)[0])
        self.camera_gaps_mm[before] = round(min(self.camera_gaps_mm.get(before, math.inf), gap), 1)
        if phase != before:
            self.events.append({"phase": "camera_gap", "of": before, "mm": self.camera_gaps_mm[before]})

    def _walk_up_note(self, actual) -> dict:
        ahead, left, off = lock_from(self.fix, actual)
        return {"lock_ahead_m": round(ahead, 3), "lock_left_m": round(left, 3), "yaw_off_deg": round(off, 1)}

    def _abort(self, obs: plan.Observation, phase: str) -> str:
        if self._blocked:
            return self._blocked
        return super()._abort(obs, phase)


# --- on to the door pull ---------------------------------------------------------------------------------------------

#: The door pull's stages after the key (`node.py --then_door`): each walks over first.
DOOR_STAGES = plan.STAGES[plan.STAGES.index("walk"):]


def door_fix(fix: plan.Fix) -> plan.Fix:
    """The door pull's fix from the key's (after its refit, the script frame where the dog stands at the key), its
    script frame moved to the door pull's stance: square to the door, the box's root `plan.NOMINAL_DISTANCE_M` straight
    ahead (the simulated demo's middle placement). From the key's stance that is ~0.09 m back and ~0.18 m right."""
    box = perception.site_pose(fix.site)
    normal = box[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX)
    normal = normal[:2] / np.linalg.norm(normal[:2])
    x, y = box[:2, 3] + plan.NOMINAL_DISTANCE_M * normal
    plain = plan.Fix(fix.script_odom, fix.site, fix.pose_script, fix.fit, fix.detections, fix.lidar_points,
                     fix.tag_only_yaw_deg, 0.0, fix.tag_tilt_deg, fix.beyond_edges)
    return plan.reanchor(plain, np.asarray(fix.script_odom) @ plan.base_pose(float(x), float(y),
                                                                              math.atan2(-normal[1], -normal[0])))


@dataclass
class DoorAfterKey(WalkUp, plan.Attempt):
    """The door pull straight after the key's turn, from where the key left the dog: no new look, and no backing off to
    walk in straight. The arm folded, the dog walks over to the door pull's stance on all three axes at once (the key's
    walk up, `WalkUp`: back and sideways), settles, and the door pull's refit and range check run there as after its
    own walk in; then its unfold and the rest of its phases (`plan.attempt_phases`). The door pull's unfold puts the
    hand 0.40 m ahead of the base: at the key's stance, with the door 0.44 m off, that is in the door, so the arm
    unfolds only once the dog is over."""

    def _make_phases(self) -> list:
        if self.stage not in DOOR_STAGES:
            raise ValueError(f"the door pull after the key is one of {DOOR_STAGES}, not {self.stage!r}")
        phases = plan.attempt_phases(self.stage, self.timing, self.fold)
        walk = [p for p in phases if p.name in ("walk_in", "walk_settle")]
        return walk + [p for p in phases if p.name not in ("walk_in", "walk_settle")]

    def _prepare(self) -> None:
        super()._prepare()
        self._folded_walk = True                 # the arm holds the fold until the unfold begins

    def _goal_servo(self, cmd, base_s, measured, st):
        if cmd.phase == "unfold":
            self._folded_walk = False
        if self._folded_walk:
            self.status = "joint"
            return np.asarray(plan.FOLD_SERVO_DEG)
        return super()._goal_servo(cmd, base_s, measured, st)

    def _walk_up_note(self, actual) -> dict:
        x, y, yaw = actual
        site = plan.reanchor(self.fix, np.asarray(self.fix.script_odom) @ plan.base_pose(x, y, yaw)).site
        return {"box_distance_m": round(site.distance_m, 3), "box_bearing_deg": round(site.bearing_deg, 1),
                "door_yaw_deg": round(site.yaw_deg, 1)}

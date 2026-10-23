"""The key inserted into the box's cam lock by the D1 on the real Go2, as numbers: the attempt `combiner_door/robot/
node.py --task insert` runs. numpy only, no ROS, no torch.

The simulated insertion (`demos/combiner_key/env.py`: the hand level, rolled so the key goes in at 45 degrees up, the
jaws across the bar 25 mm out from the shaft) on the key turn's robot program (`key_plan.py`: its look, walk up, refit,
the head camera's box) and the door pull's (`combiner_door/robot/plan.py`: the arm link at 10 Hz, the aborts, the
record). What is the insertion's:

- **The dog walks up first, the arm folded, and the key is handed in at the stance** (since 2026-10-22): carried out in
  front the key and hand shadow the door from the lidar, and the refit after a walk with the key in hand turned the door
  6.9 deg and aborted. So `walk_in` and `walk_settle` (with the refit) come before `unfold`, the arm held at the fold.
- **The key is handed in** (`load`): the arm unfolds to `KEY_CARRY_IN_BASE`, in front of the dog, the jaws open and the
  hand already as it will insert; a 3 s countdown in the log while the key is put there and held, then the jaws close on
  the bar. Closed on nothing, they open and count down again (three tries); the key slipping out later aborts.
- **It lies down to insert** (Lukas, 2026-10-21: "Make the dog sit, and then insert while it is sitting down"): sport
  mode's StandDown, P2Dingo's "sit" (the trunk lowered level over its planted feet), not Unitree's `Sit` trick, which
  tips the chest up. Standing, the base is 0.35-0.39 m over the box's root on the dog and the lock only 3-6 cm above it:
  the level, rolled hand solves from no stance there (the CPU model, 2026-10-21, 0.40-0.70 m from the door and 0.10 m
  left to 0.20 m right). Lying, the base ~0.20 m lower, it solves from 29-46 stances with the shoulder at 34-67% of its
  torque. The drop is measured (sport mode's `body_height`). **No lidar refit lying** (Lukas, 2026-10-22: the held key
  obscures the door): the fix is carried through the lie-down by the odometry, plus whatever part of the measured
  sit-back (`LIE_BACK_M`, 4 cm straight back) the odometry did not report (`_replan_lying`).
- **The insertion is by touch, not by position.** The cam lock's own keyway has under 0.5 mm of play round the key (no
  printed lead-in, Lukas, 2026-10-21); nothing the robot measures puts the key that close (the look's depth was 3-11 mm
  off in simulation; the D1 sags ~16 mm holding it out there). So: the key to 25 mm out on the keyway's axis
  (`key_pre`, the hand's outer loop taking out the sag the encoders see); in along the axis at `IN_SPEED_M_S`
  until the key stops following its command (`key_in`, contact: the measured depth `CONTACT_LAG_M` behind more than it
  was at the start); if that is short of seated -- the shaft on the lock's face, not in it -- a spiral across the
  keyway at the touched depth, pressed `PRESS_M` into it, until the key advances `ENTER_M` (`key_search`, out to
  `SEARCH_R_M`); then in to seated (`key_seat`), the jaws opened (`release`) and the hand straight back out
  (`key_back`). Every depth and miss is the forward kinematics of the arm's encoders with the key where the hand was
  given it: a key that turns or slides in the jaws is not seen.
- **The stages** (`STAGES`): load (hand it in, take it back), walk (walk up and refit first, then hand it in and take it
  back), lie (and lie down, plan the insertion, stand), reach (and the key to 25 mm out and back), touch (and in until it touches), insert.
  Anything short of `insert` ends with the key handed back (`unload`, a countdown, `drop`) before the fold.

Not checked anywhere but on the robot: StandDown with the D1 on the back and the key in the hand; the real keyway's
place against `geometry.py` (the turn found the key's platform where the model has it, to the platform's ~1 cm); what
force the D1's position servos push with at a few mm of lag; whether a printed key slides into a cam lock from a
spiral at all.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

import d1_hardware
import d1_ik
from demos.combiner_door import perception
from demos.combiner_door.robot import plan
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6, _seeds, path_clear, solve_link6
from demos.unifp import decoupled_arm, script as demo_script

from .. import geometry as kg
from . import key_plan as kp

STAGES = ("watch", "look", "load", "walk", "lie", "reach", "touch", "insert")

# --- the stance ------------------------------------------------------------------------------------------------------

#: Where the dog should lie: the seated key's back face this far ahead of the base and the lock this far to its right,
#: facing the door. At the dog's lying height (0.25 m down, the base ~0.10-0.14 m over the box's root) the CPU model's
#: insertion solves over all of 0.40-0.60 m ahead and 0.10 m left to 0.20 m right, the shoulder at 40-87%: 0.48 keeps it
#: under ~60%.
INSERT_SEATED_AHEAD_M = 0.48
INSERT_LOCK_RIGHT_M = 0.10
#: StandDown sets the dog down this far back of where it stood (Lukas: "sitting down usually moves the robot abit
#: backwards as it sits"; the lidar had the seated key 0.49 m ahead standing and 0.53 lying, and the door 4.3 degrees
#: more turned, 2026-10-21, one run): the walk up stops this much nearer. The turn is not allowed for (its sign from
#: one run).
LIE_BACK_M = 0.04
#: After the walk and its refit the lock must be in this window from the base (seated plane ahead, lock right) and the
#: door square to `key_plan.YAW_LIMIT_DEG`. Lying, no window: the insertion's own plan from there decides
#: (`_plan_insert`). (A window of 0.45-0.55 m and 5-15 cm lying aborted the first lie-down, the dog 0.53 m and 3 cm off
#: after it sat back and turned, where the plan solves.)
ACCEPT_SEATED_AHEAD_M = (0.38, 0.56)
ACCEPT_LOCK_RIGHT_M = (0.0, 0.20)

# --- the hand and the key ---------------------------------------------------------------------------------------------

#: Where the hand waits (the jaw centre, ahead, left, up of the base, its own frame), rolled as it inserts: the key handed
#: in there, carried through the walk and the lie-down, and handed back. The key's shaft tip ~0.33 m ahead, clear of the
#: door at the stance (0.48 m).
KEY_CARRY_IN_BASE = (0.28, 0.0, 0.25)
#: The jaws round the key's bar (20 mm, `geometry.KEY_BAR_BOX`'s top to bottom), and opened to take it or let it go.
JAW_OPEN_M = demo_script.JAW_LEVER_OPEN_M
JAW_SHUT_M = demo_script.JAW_BAR_SHUT_M
#: Servo 6 within this of its fully shut report means nothing between the jaws, for `SLIP_S` while the key should be
#: there. Closed on the key on the dog it stopped at -1.8 units, then read -19.4 (shut: -19.8) as the walk began, the
#: key out of the jaws (Lukas's third live run, 2026-10-21). `BAR_UNITS` is that hold, for dry runs.
GRIP_EMPTY_UNITS = 6.0
BAR_UNITS = -1.8
SLIP_S = 0.5
#: The hand-in without Enter (Lukas, 2026-10-21: "can you make it automatically grasp so I dont have to press enter
#: while holding the tool in place"): the jaws open and still for `LOAD_S`, a countdown in the log, then they close; not
#: held, they open and count down again, `LOAD_TRIES` in all. Handing it back, a countdown of `HAND_BACK_S` before the
#: jaws open.
LOAD_S = 3.0              # Lukas: "3 second countdown is plenty"
LOAD_TRIES = 3
HAND_BACK_S = 3.0
#: The cam lock as Lukas measured it (2026-10-21, callipers, `results/week_06/screenshots/2026-10-21_cam_lock_calipers.jpg`):
#: a chrome cylinder in a 27.5 mm black bezel, its keyway the printed adapter's outline (the shaft and its two wards, at
#: 45 degrees; Lukas: "exactly the same as the printed adapter"), with under 0.5 mm of play by his earlier answer; the
#: lock's face 4.5 mm proud of the door, the keyway 8.7 mm deep (4.2 mm of it behind the door's face).
LOCK_FACE_OFF_DOOR_M = 0.0045
KEYWAY_DEPTH_M = 0.0087
#: The key's shaft past the bar's back face (the STL's, `leg_cfg.KEY_SHAFT_LENGTH_M`).
SHAFT_M = 0.010
#: From the shaft's tip on the lock's face to seated: the keyway's depth, the shaft being longer (it bottoms out before
#: the bar meets the face). So seated, the key's back face is 5.8 mm off the door. (The turn's model, the printed
#: adapter's, had it 17 mm off; the turn's fingers found that good only to the platform's ~1 cm.)
TRAVEL_M = min(SHAFT_M, KEYWAY_DEPTH_M)
SEATED_OFF_DOOR_M = LOCK_FACE_OFF_DOOR_M + SHAFT_M - TRAVEL_M
#: The key in the robot's hand: its shaft ON the gripper's roll axis (Link6's z), the jaws on the bar's root round the
#: shaft (Lukas, 2026-10-23: how he has been handing it in; with the simulation's grip, the bar 25 mm out from the shaft
#: (`geometry.KEY_GRASP_U_M`), every insertion went in that far along the lever, "to the top right"). The key's origin
#: as far out along the axis as the simulation's (`geometry.KEY_ORIGIN_LINK6`); the touch finds the depth.
KEY_IN_JAWS_LINK6 = (0.0, 0.0, kg.KEY_ORIGIN_LINK6[2])
#: The jaw centre from the key's origin, along the key and out of the door (`env._key_point`'s, the grip above).
JAW_FROM_KEY = (KEY_IN_JAWS_LINK6[0], KEY_IN_JAWS_LINK6[2] - kg.JAW_CENTRE_Z_M)

# --- the insertion ----------------------------------------------------------------------------------------------------

#: The wrist camera looks at the lock (`key_view`) with the key's back face this far out: the bezel ~17 cm from the
#: camera, ~150 px across in the 1280 x 720 image, the key mostly out of its way (the bracket's CAD mount, 30 degrees
#: below the approach). At each place the hand looks from, the first `VIEW_SETTLE_S` after it gets there is not looked
#: at (the arm settling); frames after it are, and their median offset is used once `VIEW_MIN_FRAMES` from one place
#: found the bezel (`demos/combiner_key/keyhole.py`).
VIEW_OFF_M = 0.085
VIEW_SETTLE_S, VIEW_MIN_FRAMES = 0.8, 3
#: A place moves on (or `key_view` ends) after `VIEW_TRIES` frames found nothing, or `VIEW_PLACE_S` after it settled:
#: the dog's node searched ~1 frame a second (Lukas's live run, 2026-10-22, 13:29 UTC), so a clock alone saw one frame.
VIEW_TRIES, VIEW_PLACE_S = 3, 5.0
#: `key_view`'s own clock: an upper bound; finding the bezel, or giving up on it, ends it.
VIEW_S = VIEW_SETTLE_S + VIEW_PLACE_S + 2.0
#: **The lock's first look** (`lock_look`). In that live run the held key and the jaws were where the model put the
#: bezel at `key_view`, the bezel nowhere in the picture, and the key then missed the lock. Lukas: "the search stage
#: should happen right after the robot sits down while the arm is folded - tilt the wrist up and pan left and right
#: using the base". So, lying, the arm folded and empty: the wrist camera aimed at where the model puts the bezel
#: (servo 5, the wrist's pitch, up from the fold; servo 1, the arm's base, round: `aim_folded`), then panned by servo 1
#: through `LOOK_PAN_DEG`, each place held as `key_view`'s are. The first place with `VIEW_MIN_FRAMES` finds moves the
#: box model across the door onto the bezel (`_use_look`) and the insertion is planned again; then back to the fold,
#: and on to `unfold` and the key. The image's search looks `LOOK_WINDOW_M` round the model's place (the sit-back's
#: centimetres); places nearer the head camera than `LOOK_MARGIN_M` are left out. From ~0.45 m a degree of the wrist
#: mount's error is ~8 mm on the door, which `key_view`, close, and the touch then take up.
LOOK_PAN_DEG = (0.0, 15.0, -15.0, 30.0, -30.0)
#: 0.15 m since 2026-10-23: from the fold the bezel was 66-77 mm right and 35-39 mm below the model's place in two runs
#: running, just past the first 80 mm (Lukas: "the search needs to be widened"); the lock against the tag in the same
#: frames, 247-251 mm left of its centre and 35-36 mm below, matches the box model (251 and 40), so the offset is in where
#: the box was put against the wrist camera (the look, the sit-back, the wrist camera's mount), not in the box.
LOOK_WINDOW_M = 0.15
LOOK_MARGIN_M = 0.02
LOOK_ARRIVED_DEG = 1.5
#: The look also records the lock's range by the bezel's size in the picture against the model's (`range_m`,
#: `model_range_m`): ~2 cm a pixel of radius at 0.48 m, and on the test renders 2% long (11 mm), so it is not used to
#: move the box; the touch finds the depth (`IN_SLACK_M`).
LOOK_S = len(LOOK_PAN_DEG) * (VIEW_SETTLE_S + VIEW_PLACE_S + 3.0) + 6.0
#: Where the vision's offset from the keyway that the touch found is kept between runs: added to what the camera sees
#: (its mount is the bracket's CAD, never measured). The mean of the runs' residuals, each capped at `CALIBRATION_MAX_M`.
CALIBRATION_PATH = "logs/combiner_key_calibration/keyhole.json"
CALIBRATION_MAX_M = 0.006
#: The key's back face this far out along the keyway's axis before it goes in: its shaft's tip 16 mm off the lock's face.
PRE_OFF_M = 0.025
#: In along the axis this fast (`key_in`).
IN_SPEED_M_S = 0.002
#: Contact: the key's measured depth this much further behind its command than it was in `key_in`'s first second (the
#: arm's sag along the axis), for `CONTACT_HOLD_S`.
CONTACT_LAG_M = 0.003
CONTACT_HOLD_S = 0.3
#: Seated (by touch): stopped again at least the shaft's length less this past where the key met the lock's face; or,
#: straight in, stopped with its back face nearer seated than half the keyway's depth. A key driven this far past where either
#: should have stopped it aborts.
SEATED_TOL_M = 0.002
OVERRUN_M = 0.005
#: `key_in` goes on until it touches, up to this far past where the model seats it: the lock's depth after lying down
#: is only the odometry's and the sit-back's (both live runs on 2026-10-22 added 44 mm of sit-back the odometry did not
#: see, and both keys stopped 5 mm past seated touching nothing). A touch more than half the keyway's depth past the
#: model's seated is the lock's face, the door that much further (straight in it would have bottomed there instead,
#: which cm of error make the unlikely case).
IN_SLACK_M = 0.04
#: The search (`key_search`): at the touched depth less `PRESS_M`, an Archimedean spiral across the keyway (`PITCH`
#: between turns, at `SEARCH_SPEED_M_S` along it) out to `SEARCH_R_M`; in when the key advances `ENTER_M` past where it
#: touched (under `PRESS_M`: over the keyway the key goes to its command, `PRESS_M` in). ~85 s to search all of it.
PRESS_M = 0.002
SEARCH_PITCH_M = 0.0004
SEARCH_SPEED_M_S = 0.0015
SEARCH_R_M = 0.004
ENTER_M = 0.0012
BACK_OUT_M = 0.06
#: The hand's outer loop (the simulation's, `env.HAND_LOOP_*`): the jaw centre's miss as the encoders put it, integrated
#: into the command at this gain, capped; along the axis too only before the key is going in.
LOOP_GAIN = 1.5
LOOP_MAX_M = 0.03

# --- lying down -------------------------------------------------------------------------------------------------------

#: How far StandDown lowers the base, for dry runs: 0.249 m on the dog with the D1 on and the key in the hand (sport
#: mode's body height 0.315 to 0.067, 2026-10-21; P2Dingo's model, `go2_omniverse/posture.py`, has ~0.18). A live run
#: measures it.
LIE_DROP_EST_M = 0.25
#: Lying: the base this much lower than it stood, still to `LIE_STILL_M` over a second; standing again: back to within
#: `STAND_TOL_M`. Each within its timeout.
LIE_MIN_DROP_M = 0.10
LIE_STILL_M = 0.005
STAND_TOL_M = 0.03
POSTURE_TIMEOUT_S = 10.0

#: The phases' lengths, s (the reactive ones are caps: they end when their condition is met).
GRIP_S, PRE_S, IN_S, SEARCH_S, SEAT_S, RELEASE_S, BACK_S, HOME_S, DROP_S = 1.0, 2.0, 45.0, 120.0, 12.0, 1.0, 3.0, 3.0, 1.5
LIE_S, LIE_SETTLE_S, STAND_S, STAND_SETTLE_S = 3.0, 1.5, 3.0, 1.0
#: Asked before with `--step`.
CONFIRM_BEFORE = ("unfold", "walk_in", "lie_down", "key_go", "key_in", "key_search", "release", "stand_up", "fold")
#: What the log says through the hand-in and the hand-back countdowns.
LOAD_TEXT = ("put the key in the open jaws (the fingers on the bar right at the shaft, so the shaft points ahead on the "
             "gripper's axis, the lever up and to the right at 45 degrees, its platform facing up-left) and hold it "
             "there: they close in {n} s")
HAND_BACK_TEXT = "take hold of the key: the jaws open in {n} s"
REACTIVE = {"key_in", "key_search", "key_seat", "key_back"}


# --- the look --------------------------------------------------------------------------------------------------------

def _frames(box):
    """The keyway's (seated origin, out of the door, along the key at insertion, its top side), in `box`'s frame."""
    box = np.asarray(box)
    rot = box[:3, :3]
    seated = box @ np.array([kg.LOCK[0] + SEATED_OFF_DOOR_M, kg.LOCK[1], kg.LOCK[2], 1.0])
    return (seated[:3], rot @ np.asarray(kg.OUT_OF_DOOR_BOX), rot @ np.asarray(kg.lever_dir_box(kg.INSERT_DEG)),
            rot @ np.asarray(kg.lever_top_box(kg.INSERT_DEG)))


def insert_stance(box) -> tuple[float, float, float]:
    """The insertion's stance (x, y, yaw) in the frame `box` is given in: facing the door, the seated key's back face
    `INSERT_SEATED_AHEAD_M` ahead and the lock `INSERT_LOCK_RIGHT_M` to the right."""
    seated, out, _, _ = _frames(box)
    right = np.asarray(box)[:3, :3] @ np.asarray(kg.LEVER_LEVEL_BOX)
    base = seated + (INSERT_SEATED_AHEAD_M - LIE_BACK_M) * out - INSERT_LOCK_RIGHT_M * right
    return float(base[0]), float(base[1]), math.atan2(-out[1], -out[0])


def seated_from(fix: plan.Fix, base) -> tuple[float, float, float]:
    """The seated key's origin from a base at (x, y, yaw), script frame: ahead m, right m, and the base's yaw from
    facing the door (deg)."""
    seated, out, _, _ = _frames(perception.site_pose(fix.site))
    x, y, yaw = base
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = seated[0] - x, seated[1] - y
    facing = math.atan2(-out[1], -out[0])
    return c * dx + s * dy, -(-s * dx + c * dy), math.degrees(plan.wrap(facing - yaw))


class InsertFix(kp.KeyFix):
    """The key turn's fix (its walk-up range at the look) with the insertion's window after the walk."""

    def in_range(self, check_tag: bool = True) -> tuple[bool, str]:
        if check_tag:
            return super().in_range(check_tag)
        ahead, right, off = seated_from(self, (0.0, 0.0, 0.0))
        problems = []
        lo, hi = ACCEPT_SEATED_AHEAD_M
        if not lo <= ahead <= hi:
            problems.append(f"{'back' if ahead < lo else 'forward'} {abs(ahead - INSERT_SEATED_AHEAD_M + LIE_BACK_M):.2f} m")
        lo, hi = ACCEPT_LOCK_RIGHT_M
        if not lo <= right <= hi:
            problems.append(f"{'right' if right > hi else 'left'} {abs(right - INSERT_LOCK_RIGHT_M):.2f} m")
        if abs(off) > kp.YAW_LIMIT_DEG:
            problems.append(f"turn {'left' if off > 0 else 'right'} {abs(off):.0f} deg")
        text = f"the seated key {ahead:.2f} m ahead, {right:+.2f} m right, door {off:+.1f} deg off square"
        return not problems, text + ("" if not problems else "; move the dog " + ", ".join(problems))


def keyhole_prediction(fix):
    """Where the model puts the bezel's front centre, out of the door, along the key and up its top side, in `fix`'s
    script frame: what `keyhole.find_bezel` searches round (an attempt's, and `node.py --stage keyhole`'s)."""
    box = perception.site_pose(fix.site)
    _, out, u, t = _frames(box)
    centre = box @ np.array([kg.LOCK[0] + LOCK_FACE_OFF_DOOR_M, kg.LOCK[1], kg.LOCK[2], 1.0])
    return centre[:3], out, u, t


def make_insert_fix(look_odom, frames, scans) -> InsertFix | None:
    """The look's fix with the script frame at the insertion's stance (`insert_stance`, as the odometry will report it,
    `key_plan.ODOM_WALK_SCALE`): the walk up's target."""
    stance = {}

    def stance_from(box_look):
        x, y, yaw = insert_stance(box_look)
        stance["look"] = (kp.ODOM_WALK_SCALE * x, kp.ODOM_WALK_SCALE * y, yaw)
        return stance["look"]

    fix = plan.make_fix(look_odom, frames, scans, stance_for=stance_from, fix_class=InsertFix)
    if fix is not None:
        fix.stance_look = stance["look"]
    return fix


# --- the hand ---------------------------------------------------------------------------------------------------------

def carry_rotation(yaw: float) -> np.ndarray:
    """Link6's axes, in a frame turned `yaw` from the base's, holding the key as it inserts into a door square ahead."""
    th = math.radians(kg.INSERT_DEG)
    c, s = math.cos(yaw), math.sin(yaw)
    ahead = np.array([c, s, 0.0])
    u = np.array([s * math.cos(th), -c * math.cos(th), -math.sin(th)])       # right and up, at 45 degrees
    return decoupled_arm.hand_rotation(ahead, u, 0.0, kg.GRASP_ROLL)


def carry_point(base, base_z: float) -> tuple[float, float, float]:
    """`KEY_CARRY_IN_BASE` for a base at (x, y, yaw) and height `base_z`, script frame."""
    x, y, z = KEY_CARRY_IN_BASE
    c, s = math.cos(base[2]), math.sin(base[2])
    return (base[0] + c * x - s * y, base[1] + s * x + c * y, base_z + z)


def jaw_for(box, off: float, a: float = 0.0, b: float = 0.0) -> np.ndarray:
    """The jaw centre (script frame) holding the key with its back face `off` out of seated along the keyway's axis and
    `a`, `b` across it (along the key, up its top side)."""
    seated, out, u, t = _frames(box)
    origin = seated + off * out + a * u + b * t
    return origin + JAW_FROM_KEY[0] * u + JAW_FROM_KEY[1] * out


def key_in_hand(joints, servo_deg, base_s) -> tuple[np.ndarray, np.ndarray]:
    """(the key's origin, its (u, n, t) axes as columns), script frame, by forward kinematics of `servo_deg`, the key
    where the hand was given it (`KEY_IN_JAWS_LINK6`, `geometry.KEY_AXES_LINK6`)."""
    q = d1_ik.from_servo_deg(servo_deg)[0]
    pos, rot = d1_ik.tool_pose(joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
    origin = pos + rot @ np.asarray(KEY_IN_JAWS_LINK6)
    axes = rot @ np.asarray(kg.KEY_AXES_LINK6).T
    return plan.transform(base_s, [origin])[0], base_s[:3, :3] @ axes


def stage_phases(stage: str, attempt) -> list:
    """The phases after the look of an insertion attempt to `stage` (`STAGES`). The insertion's points come from the
    attempt's own state (`InsertAttempt._target`), which its step moves."""
    if stage not in STAGES[2:]:
        raise ValueError(f"an insertion attempt's stage is one of {STAGES[2:]}, not {stage!r}")
    Phase = demo_script.Phase
    index = STAGES.index(stage)
    carry = lambda st, u: carry_point(st["base"], st["base_z"])
    carried_ = carried()
    key = dict(approach=lambda st: tuple(attempt.key_axes()[0]), object_axis=lambda st: tuple(attempt.key_axes()[1]),
               servo=True)
    at_key = lambda name, seconds, jaw=JAW_SHUT_M: Phase(name, seconds, path=lambda st, u: attempt._target(), jaw_m=jaw,
                                                         **key)
    # The walk up and its lidar refit come FIRST, the arm folded, and the key is handed in at the stance: carried out in
    # front (`KEY_CARRY_IN_BASE`, ~10 cm ahead of and above the MID-360) the hand and key shadow the door, and the refit
    # after the walk turned it 6.9 deg and aborted (Lukas's live run, 2026-10-22). The arm holds the fold until `unfold`
    # (`InsertAttempt._folded_walk`).
    # Lying down follows the walk, the arm still folded and empty, and the wrist camera looks for the lock from there
    # (`lock_look`, Lukas 2026-10-23); the key is handed in after it, lying.
    phases = []
    if index >= STAGES.index("walk"):
        phases += [Phase("walk_in", plan.WALK_IN_S, path=carry, jaw_m=JAW_SHUT_M, base=lambda st, u: (0.0, 0.0, 0.0)),
                   Phase("walk_settle", plan.WALK_SETTLE_S, path=carry, jaw_m=JAW_SHUT_M)]
    if index >= STAGES.index("lie"):
        phases += [Phase("lie_down", LIE_S, path=carry, jaw_m=JAW_SHUT_M, **carried_),
                   Phase("lie_settle", LIE_SETTLE_S, path=carry, jaw_m=JAW_SHUT_M, **carried_),
                   Phase("lock_look", LOOK_S, path=carry, jaw_m=JAW_SHUT_M, **carried_)]
    phases += [Phase("unfold", plan.UNFOLD_S, path=carry, jaw_m=JAW_OPEN_M)] + load_phases()
    if index >= STAGES.index("reach"):
        phases += [Phase("key_go", plan.UNFOLD_S, path=lambda st, u: attempt._target(), jaw_m=JAW_SHUT_M),
                   at_key("key_view", VIEW_S),
                   Phase("key_pre", PRE_S, target=lambda st: attempt._target(), jaw_m=JAW_SHUT_M, **key)]
        if index >= STAGES.index("touch"):
            phases.append(at_key("key_in", IN_S))
        if stage == "insert":
            phases += [at_key("key_search", SEARCH_S), at_key("key_seat", SEAT_S), at_key("release", RELEASE_S, JAW_OPEN_M)]
        phases += [at_key("key_back", BACK_S, JAW_OPEN_M if stage == "insert" else JAW_SHUT_M),
                   Phase("home", HOME_S, path=carry, jaw_m=JAW_OPEN_M if stage == "insert" else JAW_SHUT_M, **carried_)]
    if stage == "insert":
        return phases + stand_phases(JAW_OPEN_M) + [Phase("fold", plan.FOLD_S, path=carry, jaw_m=JAW_OPEN_M)]
    if index >= STAGES.index("lie"):
        phases += stand_phases()
    return phases + hand_back_phases()


def carried() -> dict:
    """A phase's hand held as it carries the key (`carry_rotation`), whatever way the base faces."""
    return dict(approach=lambda st: tuple(carry_rotation(st["base"][2])[:, 2]),
                object_axis=lambda st: tuple(-carry_rotation(st["base"][2])[:, 0]), servo=True)


def stand_phases(jaw_m: float = JAW_SHUT_M) -> list:
    Phase = demo_script.Phase
    carry = lambda st, u: carry_point(st["base"], st["base_z"])
    return [Phase("stand_up", STAND_S, path=carry, jaw_m=jaw_m, **carried()),
            Phase("stand_settle", STAND_SETTLE_S, path=carry, jaw_m=jaw_m, **carried())]


def load_phases() -> list:
    """The key handed in: the jaws open and still while it is put there (`LOAD_S`), then closed on it."""
    Phase = demo_script.Phase
    carry = lambda st, u: carry_point(st["base"], st["base_z"])
    return [Phase("load", LOAD_S, path=carry, jaw_m=JAW_OPEN_M, **carried()),
            Phase("grip", GRIP_S, path=carry, jaw_m=JAW_SHUT_M, **carried())]


def hand_back_phases(jaw_m: float = JAW_SHUT_M) -> list:
    """The key handed back: to the carry point, a countdown (`HAND_BACK_S`), the jaws opened, folded."""
    Phase = demo_script.Phase
    carry = lambda st, u: carry_point(st["base"], st["base_z"])
    return [Phase("unload", HOME_S, path=carry, jaw_m=jaw_m, **carried()),
            Phase("hand_back", HAND_BACK_S, path=carry, jaw_m=jaw_m, **carried()),
            Phase("drop", DROP_S, path=carry, jaw_m=JAW_OPEN_M, **carried()),
            Phase("fold", plan.FOLD_S, path=carry, jaw_m=JAW_OPEN_M)]


# --- the attempt -----------------------------------------------------------------------------------------------------

class Places:
    """A look from one place after another (`key_view`'s one; `lock_look`'s aim and pans): the frames the node finds
    filed under the place they were taken from, a place left after `VIEW_TRIES` frames found nothing or `VIEW_PLACE_S`
    after it settled, the look over at the first place with `VIEW_MIN_FRAMES` finds (`used`) or after the last."""

    def __init__(self, places=()):
        self.places = list(places)
        self.reset()

    def reset(self) -> None:
        self.i, self.arrived, self.frames, self.used, self.done = 0, None, {}, None, not self.places

    def settled(self, now_s: float) -> bool:
        return not self.done and self.arrived is not None and now_s - self.arrived >= VIEW_SETTLE_S

    def add(self, found) -> bool:
        """`found` filed under the place now; not while the arm is on its way to one."""
        if self.done or self.arrived is None:
            return False
        self.frames.setdefault(self.i, []).append(found)
        return True

    def found(self) -> list:
        return [f for f in self.frames.get(self.used, []) if f.ok] if self.used is not None else []

    def step(self, now_s: float, there: bool, events: list, name: str) -> bool:
        """One step: `there`, whether the arm is at the place now. True once the look is over."""
        if self.done:
            return True
        if not there:
            self.arrived = None
            return False
        if self.arrived is None:
            self.arrived = now_s
        here = self.frames.get(self.i, [])
        hits = [f for f in here if f.ok]
        if len(hits) >= VIEW_MIN_FRAMES:
            self.used, self.done = self.i, True
            return True
        if (len(here) >= VIEW_TRIES and not hits) or now_s - self.arrived >= VIEW_SETTLE_S + VIEW_PLACE_S:
            events.append({"phase": name, "place": self.i, "frames": len(here), "found": len(hits),
                           "why_not": sorted({f.why for f in here if not f.ok})[:2]})
            if self.i + 1 >= len(self.places):
                self.done = True
                return True
            self.i, self.arrived = self.i + 1, None
        return False


def aim_folded(joints, target_base, servo=None) -> np.ndarray:
    """Servo degrees that point the wrist camera's optical axis at `target_base` (base frame) from the fold: the fold
    with servo 1 (the arm's base: the camera's heading) and servo 5 (the wrist's pitch: its elevation) turned. Each
    turns the camera about 1:1 (`servo 1 +30` heads it 30 degrees right, `servo 5 -30` lifts it 30), so a few steps of
    that, each from the camera's forward kinematics, converge."""
    servo = np.array(plan.FOLD_SERVO_DEG if servo is None else servo, dtype=float)
    target = np.asarray(target_base, dtype=float)
    for _ in range(6):
        cam = plan.wrist_camera_base(joints, servo)
        z, to = cam[:3, 2], target - cam[:3, 3]
        heading = math.atan2(z[1], z[0]) - math.atan2(to[1], to[0])
        lift = math.asin(np.clip(to[2] / np.linalg.norm(to), -1, 1)) - math.asin(np.clip(z[2], -1, 1))
        servo[0] = float(np.clip(servo[0] + math.degrees(heading), -90.0, 90.0))
        servo[4] = float(np.clip(servo[4] - math.degrees(lift), -90.0, 90.0))
    return servo


@dataclass
class InsertAttempt(kp.WalkUp, plan.Attempt):
    """The door pull's attempt (`plan.Attempt`) with the key turn's walk up (`key_plan.WalkUp`) and the insertion's
    phases (`stage_phases`), lie-down, touch and search. `step` at 10 Hz."""

    #: A dry run: no lock to touch, so a key that goes all the way in untouched is taken as seated.
    dry: bool = False
    #: Where the camera's calibration against the touch is kept (`CALIBRATION_PATH` on the dog), or None: not kept.
    calibration_path: str | None = None

    joint_phases = {"unfold", "key_go", "fold"}
    dwell_phases = frozenset()
    position_only_phases = {"walk_in", "walk_settle"}
    #: The arm's lag abort (`plan.ARM_LAG_DEG`) while the key presses on the lock.
    holding_phases = {"key_in", "key_search", "key_seat"}
    #: Where an abort backs the key out along the keyway's axis before anything else.
    at_the_lever_phases = {"key_view", "key_pre", "key_in", "key_search", "key_seat", "release", "key_back"}
    walking_phases = {"walk_in"}
    release_held_phases = frozenset()
    refit_limit_m = kp.REFIT_LIMIT_M
    confirm_before = CONFIRM_BEFORE
    #: Not the door pull: no door reading through `grip` and `release`, which share its phases' names. With it, the
    #: lidar's door-angle thread ran back to back through the 43 s wait to hand the key in, and the odometry went 1.4
    #: and 1.8 s stale (Lukas's first two live runs, 2026-10-21).
    reads_door = False

    def _make_phases(self) -> list:
        return stage_phases(self.stage, self)

    def _prepare(self) -> None:
        self.arm.grasp, self.arm.grasp_note = (0.0, kg.GRASP_ROLL), "the key's grip (pitch 0, roll -1)"
        self._blocked = ""
        self._body0: float | None = None          # sport mode's body height as the attempt began, standing
        self._drop = 0.0                           # how far below that the base is now
        self._heights: list = []                   # (t, body height) of the last second
        self._posture = ""
        self._posture_phase, self._posture_t = "", 0.0
        self._grips = 0
        self._slip_since: float | None = None
        self._in_t0 = 0.0
        self.camera_gaps_mm: dict[str, float] = {}
        self._folded_walk = True                   # the arm holds the fold until `unfold` begins (the walk comes first)
        self._stand_odom: np.ndarray | None = None  # the base in odom as it began to lie down
        self._loop = np.zeros(3)
        self._q_pre: np.ndarray | None = None
        self._ins = {"off": VIEW_OFF_M, "a": 0.0, "b": 0.0}
        self._view = Places([(VIEW_OFF_M, 0.0, 0.0)])   # `key_view`: one place, the key `VIEW_OFF_M` out on the axis
        self._look = Places()                      # `lock_look`: servo degrees, the aim then the pans (`_plan_look`)
        self._look_base: np.ndarray | None = None  # the lying base (script frame) the look was planned from
        self._now = 0.0
        self.keyhole_frames: list = []             # `keyhole.Bezel`s the node adds through `key_view`
        self._vision = None                        # (the median Bezel's centre, its camera), script frame
        self._search_ab = (0.0, 0.0)               # where the spiral is centred, along u and t
        self._face_off = 0.0                       # the lock's face out of the model's, by the touch
        self.calibration = _load_calibration(self.calibration_path)
        self._spiral = 0.0                         # the spiral's angle, rad
        self._lag0: float | None = None
        self._contact_since: float | None = None
        self._measured_key: tuple | None = None    # (off, a, b) by the encoders
        self._recovery_target: dict | None = None
        self.check = {"held": None, "lie_drop_m": None, "plan": None, "look": None, "vision": None,
                      "contact_off_mm": None,
                      "entered_at_mm": None,
                      "searched_r_mm": 0.0, "seated": None, "measured_off_mm": None, "released": False}
        self.search_path: list = []

    # --- frames -------------------------------------------------------------------------------------------------

    def base_script(self, base_odom) -> np.ndarray:
        """The parent's, lowered by how far the base has dropped (sport mode's body height: the odometry's pose keeps
        the standing height)."""
        out = super().base_script(base_odom).copy()
        out[2, 3] -= getattr(self, "_drop", 0.0)
        return out

    def state(self, base_odom) -> dict:
        st = super().state(base_odom)
        st["base_z"] = float(self.base_script(base_odom)[2, 3])
        return st

    def key_axes(self):
        """(into the door, along the key at insertion), script frame: the hand's approach and bar at the key."""
        _, out, u, _ = _frames(perception.site_pose(self.fix.site))
        return -out, u

    def _grasp_rotation(self, to_base):
        return carry_rotation(0.0)                  # base frame: the key held as it inserts, ahead

    def _target(self) -> tuple[float, float, float]:
        """The jaw centre for the insertion's state (`_ins`), or the planned first pose's before it is planned."""
        point = jaw_for(perception.site_pose(self.fix.site), self._ins["off"], self._ins["a"], self._ins["b"])
        return tuple(float(v) for v in point)

    # --- the step -----------------------------------------------------------------------------------------------

    def step(self, obs: plan.Observation, dt: float = 0.1) -> plan.Command:
        self._now = obs.now_s
        if obs.body_height_m is not None:
            if self._body0 is None:
                self._body0 = float(obs.body_height_m)
            self._drop = max(0.0, self._body0 - float(obs.body_height_m))
            self._heights = [h for h in self._heights if obs.now_s - h[0] <= 1.0] + [(obs.now_s, obs.body_height_m)]
        before = self.phase
        base_s = self.base_script(obs.base_odom)
        if obs.arm_servo_deg is not None:
            origin, _ = key_in_hand(self.arm.joints, obs.arm_servo_deg, base_s)
            seated, out, u, t = _frames(perception.site_pose(self.fix.site))
            rel = origin - seated
            self._measured_key = (float(rel @ out), float(rel @ u), float(rel @ t))
        step_dt, why = self._react(before, obs, dt)
        if why:
            return plan.Command(before, abort=why)
        out = super().step(obs, step_dt)
        out.posture = self._posture_for(out.phase)
        empty = obs.gripper_units is not None and obs.gripper_units <= plan.GRIPPER_SHUT_UNITS + GRIP_EMPTY_UNITS
        if before == "grip" and out.phase != "grip" and not out.abort:
            self._grip_check(obs, empty, out)
        elif self.check["held"] and out.phase not in ("grip", "release", "drop", "fold", "done") and not out.abort:
            # The key must stay between the jaws until they let it go.
            self._slip_since = (obs.now_s if self._slip_since is None else self._slip_since) if empty else None
            if self._slip_since is not None and obs.now_s - self._slip_since >= SLIP_S:
                self.check["held"] = False
                self.events.append({"phase": "slipped", "gripper_units": obs.gripper_units, "in": out.phase})
                out.abort = (f"the key is no longer between the jaws (servo 6 at {obs.gripper_units:.1f} units, shut "
                             f"{plan.GRIPPER_SHUT_UNITS:.1f}): it slipped out or fell")
        if out.phase in ("load", "hand_back") and not out.abort:
            current = self.script.phases[self.script.index] if not self.script.finished else None
            left = math.ceil(current.duration_s - self.script.elapsed_s) if current is not None else 0
            out.note = (LOAD_TEXT if out.phase == "load" else HAND_BACK_TEXT).format(n=max(left, 0))
        if obs.arm_servo_deg is not None:
            kp.KeyAttempt._camera_gap(self, before, out.phase, obs.arm_servo_deg)
        return out

    def _grip_check(self, obs: plan.Observation, empty: bool, out: plan.Command) -> None:
        """After the jaws closed: the key held, or the jaws opened for another try (`LOAD_TRIES`), or an abort."""
        self.check["held"] = not empty
        self._grips += 1
        self.events.append({"phase": "grip_check", "gripper_units": obs.gripper_units, "held": not empty,
                            "try": self._grips})
        if not empty:
            return
        if self._grips >= LOAD_TRIES:
            out.abort = (f"the jaws closed on nothing {self._grips} times (servo 6 at {obs.gripper_units:.1f} units, "
                         f"shut {plan.GRIPPER_SHUT_UNITS:.1f}): no key in the hand")
            return
        # Again: open, count down, close. The script had gone on into the next phase this step: back to its start,
        # behind the new hand-in, and nothing of it sent.
        here = self.script.index
        self.script.phases[here:here] = load_phases()
        self.script.elapsed_s = 0.0
        out.phase, out.arm_servo_deg, out.gripper_units = "load", None, None
        out.base, out.walking = (0.0, 0.0, 0.0), False
        out.note = "nothing between the jaws: opening them again"

    def _posture_for(self, phase: str) -> str:
        """The posture asked of sport mode from this phase on: lying from `lie_down`, up at `stand_up`, balanced
        after."""
        self._posture = {"lie_down": "lie", "stand_up": "stand", "stand_settle": "balance"}.get(phase, self._posture)
        return self._posture

    def _skip(self) -> float:
        current = self.script.phases[self.script.index]
        return current.duration_s - self.script.elapsed_s + 1e-6

    def _react(self, phase: str, obs: plan.Observation, dt: float) -> tuple[float, str]:
        """This step's clock and the insertion's state: the posture phases held until the body has got there; the
        reactive phases' points moved, ended when their condition is met. Returns (dt for the script, abort)."""
        if self.script.finished:
            return dt, ""
        current = self.script.phases[self.script.index]
        elapsed = self.script.elapsed_s
        if phase in ("lie_down", "stand_up"):
            if self._posture_phase != phase:
                self._posture_phase, self._posture_t = phase, obs.now_s
            heights = [h for _, h in self._heights]
            still = len(heights) >= 5 and max(heights) - min(heights) <= LIE_STILL_M
            there = (self._drop >= LIE_MIN_DROP_M) if phase == "lie_down" else (self._drop <= STAND_TOL_M)
            if there and still and elapsed >= 1.0:
                if phase == "lie_down":
                    self.check["lie_drop_m"] = round(self._drop, 4)
                    self.events.append({"phase": "lain", "drop_m": round(self._drop, 4)})
                return self._skip(), ""
            if obs.now_s - self._posture_t > POSTURE_TIMEOUT_S:
                what = "lie down" if phase == "lie_down" else "stand up"
                return 0.0, (f"the dog did not {what}: the body {self._drop:.2f} m below where it stood after "
                             f"{POSTURE_TIMEOUT_S:.0f} s")
            return (min(dt, max(0.0, current.duration_s - elapsed - 1e-6))), ""
        if phase == "key_view":
            done = self._view.step(obs.now_s, True, self.events, "key_view")
            return (self._skip(), "") if done else (dt, "")
        if phase == "lock_look":
            return self._look_step(obs, dt)
        if self._measured_key is None or phase not in REACTIVE:
            return dt, ""
        off_m, a_m, b_m = self._measured_key
        ins = self._ins
        if phase == "key_in":
            ins["off"] -= IN_SPEED_M_S * dt
            if self._stopped(off_m, obs):
                # Stopped with its back face within half the keyway's depth of seated: straight in. Otherwise on the
                # lock's face: near the shaft's length out, or past seated where the door is further than modelled.
                self._found(off_m, seated=-TRAVEL_M / 2 < off_m < TRAVEL_M / 2)
                return self._skip(), ""
            if self.dry and ins["off"] < -OVERRUN_M:
                self._found(off_m, seated=True)
                self.events.append({"phase": "dry", "note": "no lock to touch: taken as seated"})
                return self._skip(), ""
            if ins["off"] < -IN_SLACK_M or current.duration_s - elapsed <= dt + 1e-9:
                return 0.0, (f"the key went {1000 * IN_SLACK_M:.0f} mm past where it should seat without touching "
                             "anything: no lock there?")
            return dt, ""
        if phase == "key_search":
            contact = self.check["contact_off_mm"]
            if self.check["seated"] or contact is None:
                return self._skip(), ""
            contact /= 1000.0
            if off_m <= contact - ENTER_M:
                self.check["entered_at_mm"] = [round(1000 * ins["a"], 2), round(1000 * ins["b"], 2)]
                self.events.append({"phase": "entered", "across_mm": self.check["entered_at_mm"],
                                    "off_mm": round(1000 * off_m, 2)})
                return self._skip(), ""
            r = SEARCH_PITCH_M * self._spiral / (2 * math.pi)
            if r > SEARCH_R_M:
                return 0.0, (f"no keyway within {1000 * SEARCH_R_M:.0f} mm of where the look put it: the key touched "
                             f"{1000 * contact:.1f} mm out of seated and never went further")
            self._spiral += SEARCH_SPEED_M_S * dt / max(r, SEARCH_PITCH_M / 2)
            r = SEARCH_PITCH_M * self._spiral / (2 * math.pi)
            a0, b0 = self._search_ab
            ins.update(off=contact - PRESS_M, a=a0 + r * math.cos(self._spiral), b=b0 + r * math.sin(self._spiral))
            self.check["searched_r_mm"] = round(1000 * r, 2)
            self.search_path.append((ins["a"], ins["b"]))
            return dt, ""
        if phase == "key_seat":
            if self.check["seated"]:
                return self._skip(), ""
            contact = self.check["contact_off_mm"] / 1000.0
            ins["off"] -= IN_SPEED_M_S * dt
            if self._stopped(off_m, obs):
                # Seated by touch: stopped again, the keyway's depth (less the tolerance) past where it met the face.
                advanced = contact - off_m
                if advanced >= TRAVEL_M - SEATED_TOL_M:
                    self._seated(off_m)
                    return self._skip(), ""
                self.check.update(seated=False, measured_off_mm=round(1000 * off_m, 2))
                return 0.0, (f"the key stuck {1000 * (TRAVEL_M - advanced):.1f} mm short of seated: in "
                             f"{1000 * advanced:.1f} mm past the lock's face, the keyway {1000 * TRAVEL_M:.1f} mm deep")
            if ins["off"] < contact - TRAVEL_M - OVERRUN_M or current.duration_s - elapsed <= dt + 1e-9:
                self.check.update(seated=False, measured_off_mm=round(1000 * off_m, 2))
                return 0.0, (f"the key did not stop at seated: {1000 * (contact - off_m):.1f} mm past the lock's face "
                             f"and its command {1000 * (contact - ins['off']):.1f}")
            return dt, ""
        if phase == "key_back":
            ins["off"] += BACK_OUT_M / current.duration_s * dt
            return dt, ""
        return dt, ""

    def _stopped(self, off_m: float, obs: plan.Observation) -> bool:
        """Whether the key has stopped following its command along the axis: its measured depth `CONTACT_LAG_M` further
        behind than in the phase's first second (the arm's sag), for `CONTACT_HOLD_S`."""
        lag = off_m - self._ins["off"]
        if self._lag0 is None:
            if obs.now_s - self._in_t0 >= 1.0:
                self._lag0 = lag
            return False
        if lag - self._lag0 > CONTACT_LAG_M:
            self._contact_since = obs.now_s if self._contact_since is None else self._contact_since
            return obs.now_s - self._contact_since >= CONTACT_HOLD_S
        self._contact_since = None
        return False

    def _found(self, off_m: float, seated: bool) -> None:
        self.check["contact_off_mm"] = round(1000 * off_m, 2)
        if seated:
            self._seated(off_m)
        self.events.append({"phase": "touched", "off_mm": round(1000 * off_m, 2), "seated": seated,
                            "commanded_off_mm": round(1000 * self._ins["off"], 2)})
        # Hold where it touched rather than drive on into the face.
        self._ins["off"] = off_m - (0.0 if seated else PRESS_M)
        if not seated:
            self._face_off = off_m - TRAVEL_M
        if not seated and self._vision is not None:
            # The touch says where the lock's face is: the bezel the camera saw, carried along its ray to that depth.
            self._search_ab = self._vision_ab(off_m - TRAVEL_M)
            self._ins["a"], self._ins["b"] = self._search_ab
            self.events.append({"phase": "vision_at_touch", "face_off_model_mm": round(1000 * (off_m - TRAVEL_M), 2),
                                "search_centre_mm": [round(1000 * v, 2) for v in self._search_ab]})

    def _seated(self, off_m: float) -> None:
        self.check.update(seated=True, measured_off_mm=round(1000 * off_m, 2))
        self.events.append({"phase": "seated", "off_mm": round(1000 * off_m, 2)})
        if self._vision is not None and self._measured_key is not None:
            # Where the key went in against where the camera put the keyway (at the face's depth the touch found, without
            # the kept calibration): the next run's correction.
            seen = np.asarray(self._vision_ab(self._face_off, calibrated=False))
            residual = np.asarray(self._measured_key[1:]) - seen
            self.check["vision_residual_mm"] = [round(1000 * v, 2) for v in residual]
            self.calibration = _save_calibration(self.calibration_path, self.calibration, residual)

    # --- the camera ---------------------------------------------------------------------------------------------------

    def keyhole_prediction(self):
        """Where the model puts the bezel's front centre, out of the door, along the key and up its top side (script
        frame): what `keyhole.find_bezel` searches round."""
        return keyhole_prediction(self.fix)

    def _looking(self):
        return {"key_view": self._view, "lock_look": self._look}.get(self.phase)

    def wants_keyhole(self, now_s: float) -> bool:
        """Whether the node should look for the keyhole in its next frame: in `lock_look` or `key_view`, the arm at
        one of its places and settled there."""
        looking = self._looking()
        return looking is not None and looking.settled(now_s)

    def keyhole_window_m(self, default: float) -> float:
        """How far round the model's place the image's own search looks (`keyhole.find_bezel`'s `wide_m`): as wide as
        the first look's (`LOOK_WINDOW_M`) through it, and through `key_view` when it found nothing to move the model by;
        `default` (the node's `--keyhole_wide_mm`) at `key_view` after it did."""
        corrected = (self.check.get("look") or {}).get("found", 0) >= VIEW_MIN_FRAMES
        if self.phase == "lock_look" or (self.phase == "key_view" and not corrected):
            return max(default, LOOK_WINDOW_M)
        return default

    def add_keyhole(self, found) -> None:
        """A `keyhole.Bezel` from a frame of `lock_look` or `key_view` (the node's image thread), filed under the place
        it was taken from; dropped if the arm has started for the next."""
        looking = self._looking()
        if looking is not None and looking.add(found):
            self.keyhole_frames.append(found)

    def _vision_ab(self, face_off_m: float, calibrated: bool = True) -> tuple[float, float]:
        """The keyway's place across its axis (along u, along t, from the model's) by the camera, its found centre carried
        along its ray to a lock face `face_off_m` out of the model's; plus the kept calibration."""
        centre, camera = self._vision
        model, out, u, t = self.keyhole_prediction()
        ray = centre - camera
        at = centre + ray * ((face_off_m - float((centre - model) @ out)) / float(ray @ out))
        rel = at - model
        cal = self.calibration["offset_m"] if calibrated else (0.0, 0.0)
        return float(rel @ u) + cal[0], float(rel @ t) + cal[1]

    def _use_vision(self) -> None:
        """At the end of `key_view`: the median of the frames from the place that found the bezel (one camera pose, so
        the median ray is a ray), if one did; the key's first pose moved onto it, else back on the model's axis."""
        frames = [f for fs in self._view.frames.values() for f in fs]
        found = self._view.found()
        record = {"frames": len(frames), "found": len(found), "why_not": sorted({f.why for f in frames if not f.ok})[:3]}
        if len(found) >= VIEW_MIN_FRAMES:
            centre = np.median(np.array([f.centre for f in found]), axis=0)
            camera = np.median(np.array([f.camera for f in found]), axis=0)
            self._vision = (centre, camera)
            depth = float(np.median([f.d for f in found]))
            a, b = self._vision_ab(depth)
            if depth:
                # Its size put the lock this far out of the model's plane (only past what size can be trusted to,
                # `keyhole.WIDE_DEPTH_TRUST`): the box moved along the door's normal, so seated is seated again.
                self._move_box(depth * self.keyhole_prediction()[1])
                record["out_mm"] = round(1000 * depth, 1)
            self._ins["a"], self._ins["b"] = a, b
            self._search_ab = (a, b)
            spread = np.ptp(np.array([[f.a, f.b] for f in found]), axis=0)
            record.update(offset_mm=[round(1000 * a, 2), round(1000 * b, 2)],
                          spread_mm=[round(1000 * v, 2) for v in spread],
                          support=round(float(np.median([f.support for f in found])), 2),
                          calibration_mm=[round(1000 * v, 2) for v in self.calibration["offset_m"]])
        self.check["vision"] = record
        self.events.append({"phase": "keyhole", **record})

    def _move_box(self, shift) -> None:
        """The box model moved by `shift` (script frame, m)."""
        import dataclasses

        move = plan.pose(None, tuple(float(v) for v in shift))
        self.fix = dataclasses.replace(self.fix, site=perception.site_from_pose(move @ perception.site_pose(
            self.fix.site)), pose_script=move @ np.asarray(self.fix.pose_script))

    def _look_step(self, obs: plan.Observation, dt: float) -> tuple[float, str]:
        """`lock_look`: the arm to each of its places in turn (servo degrees), there when within `LOOK_ARRIVED_DEG`; when
        the look is over, the box model moved onto what it found (`_use_look`), then back to the fold, and on."""
        look = self._look
        measured = None if obs.arm_servo_deg is None else np.asarray(obs.arm_servo_deg, dtype=float)
        fresh = measured is not None and obs.arm_age_s <= plan.ARRIVED_REPORT_S
        if not look.done:
            there = fresh and float(np.max(np.abs(np.asarray(look.places[look.i]) - measured))) <= LOOK_ARRIVED_DEG
            if not look.step(obs.now_s, there, self.events, "lock_look"):
                return dt, ""
            self._use_look()
            if self._blocked:
                return 0.0, self._blocked
        home = fresh and float(np.max(np.abs(np.asarray(plan.FOLD_SERVO_DEG) - measured))) <= LOOK_ARRIVED_DEG
        return (self._skip(), "") if home else (dt, "")

    def _use_look(self) -> None:
        """At the end of `lock_look`: the box model moved across the door (along u and t) onto the median of the finds
        from the place that found the bezel, and the insertion planned again from the lying base; or left as it was."""
        import dataclasses

        look = self._look
        frames = [f for fs in look.frames.values() for f in fs]
        found = look.found()
        record = {"phase": "lock_look", "places": len(look.places), "places_tried": min(look.i + 1, len(look.places)),
                  "frames": len(frames), "found": len(found), "why_not": sorted({f.why for f in frames if not f.ok})[:3]}
        if len(found) >= VIEW_MIN_FRAMES:
            centre = np.median(np.array([f.centre for f in found]), axis=0)
            camera = np.median(np.array([f.camera for f in found]), axis=0)
            model, out, u, t = self.keyhole_prediction()
            # The finds' centres are where their size puts them along their rays (`keyhole._image_find`): across the
            # door from there, which holds whatever the door's depth; the depth itself is left to the touch.
            ranges = [f.image["range_m"] for f in found if f.image and "range_m" in f.image]
            if ranges:
                record.update(range_m=round(float(np.median(ranges)), 4),
                              model_range_m=found[0].image.get("model_range_m"),
                              out_mm=round(1000 * float(np.median([f.d for f in found])), 1))
            a, b = float((centre - model) @ u), float((centre - model) @ t)
            # Out of the door's plane only when the size disagrees grossly (`keyhole.WIDE_DEPTH_TRUST`, ~3 cm from here,
            # where the size's range runs 2-3% long): `key_view`, close, measures it again.
            depth = float((centre - model) @ out)
            self._move_box(a * u + b * t + depth * out)
            record.update(place=look.used, pan_deg=LOOK_PAN_DEG[look.used] if look.used < len(LOOK_PAN_DEG) else None,
                          moved_mm=[round(1000 * a, 1), round(1000 * b, 1)],
                          spread_mm=[round(1000 * v, 1) for v in np.ptp(np.array([[f.a, f.b] for f in found]), axis=0)],
                          image_r_px=round(float(np.median([np.sqrt(np.prod(f.image["axes_px"])) for f in found
                                                             if f.image and f.image.get("ok")] or [0.0])), 1))
            self.check["look"] = record
            self.events.append(record)
            if self._look_base is not None:
                self._plan_insert(self._look_base)
            return
        self.check["look"] = record
        self.events.append(record)

    def _plan_look(self, base_s) -> None:
        """`lock_look`'s places from the lying base: the folded wrist camera aimed at the model's bezel (`aim_folded`),
        then servo 1 turned by each of `LOOK_PAN_DEG`; places nearer the head camera than `LOOK_MARGIN_M`, or whose move
        from the last crosses the trunk proxy, left out."""
        self._look_base = np.asarray(base_s).copy()
        centre = self.keyhole_prediction()[0]
        aim = aim_folded(self.arm.joints, plan.transform(plan.invert(base_s), [centre])[0])
        places, left_out, last = [], [], np.asarray(plan.FOLD_SERVO_DEG, dtype=float)
        for pan in LOOK_PAN_DEG:
            servo = aim.copy()
            servo[0] = float(np.clip(servo[0] + pan, -90.0, 90.0))
            q = d1_ik.from_servo_deg(servo)[0]
            margin = kp.camera_margin(self.arm.joints, q)
            clear = bool(path_clear(self.arm.joints, d1_ik.from_servo_deg(last)[0], q)[0])
            if margin < LOOK_MARGIN_M or not clear:
                left_out.append({"pan_deg": pan, "camera_mm": round(1000 * margin, 1), "path_clear": clear})
                continue
            places.append(servo)
            last = servo
        self._look = Places(places)
        self.events.append({"phase": "plan_look", "aim_servo_deg": [round(float(v), 1) for v in aim],
                            "places": len(places), "left_out": left_out})

    # --- the phases' hooks ------------------------------------------------------------------------------------------

    def _on_phase_change(self, before: str, phase: str, obs: plan.Observation) -> str:
        if phase == "lie_down":
            self._stand_odom = np.asarray(obs.base_odom).copy()
        if before == "lie_settle":
            return self._replan_lying(obs)
        if phase == "key_view":
            self._view.reset()
        if before == "key_view":
            self._use_vision()
            self._ins["off"] = PRE_OFF_M
        if phase in ("key_in", "key_seat"):
            self._lag0, self._contact_since, self._in_t0 = None, None, obs.now_s
        if phase == "key_back":
            if self._measured_key is not None:
                self._ins["off"] = self._measured_key[0]   # back out from where the key is, not where it was sent
            self.check["released"] = self.check["released"] or before == "release"
        return ""

    def _replan_lying(self, obs: plan.Observation) -> str:
        """Lying down: the fix carried to where the base lies by the odometry, the window checked, and the insertion
        planned from there (`_plan_insert`). No lidar refit: lying, the key held out in front shadows the door (Lukas,
        2026-10-22: "we cant trust the lidar after sitting down, as the tool obscures it"). The standing refit, before the
        key is handed in, is the last lidar word; what the odometry misses of the sit-back (~2 cm and a few degrees in the
        first lie-downs) is left to the keyhole view and the touch.

        In its place, the sit-back as measured (`LIE_BACK_M`, the first lie-down: ~4 cm straight back, which sport mode's
        odometry did not see, and ~4.5 deg of turn, which it did): whatever part of `LIE_BACK_M` the odometry did not
        report between standing and lying is added, back along the standing heading. The odometry goes on reporting the
        base that far ahead of where it is for the rest of the attempt, so the box is moved as far ahead in its frame
        (the base-relative box is then the true one), and the fix re-anchored on the odometry's own pose."""
        import dataclasses

        event = {"phase": "refit_lying", "used": False, "why": "skipped: the key in the hand shadows the door lying down"}
        if self._stand_odom is not None:
            x0, y0, yaw0 = plan.planar(self._stand_odom)
            x1, y1, _ = plan.planar(obs.base_odom)
            seen_back = -((x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0))
            extra = max(0.0, LIE_BACK_M - seen_back)
            drift = plan.pose(None, (extra * math.cos(yaw0), extra * math.sin(yaw0), 0.0))
            self.fix = dataclasses.replace(self.fix, script_odom=drift @ np.asarray(self.fix.script_odom))
            event.update(odometry_back_mm=round(1000 * seen_back, 1), sit_back_added_mm=round(1000 * extra, 1))
        self.events.append(event)
        self.fix = plan.reanchor(self.fix, obs.base_odom)
        ahead, right, off = seated_from(self.fix, (0.0, 0.0, 0.0))
        text = f"the seated key {ahead:.2f} m ahead, {right:+.2f} m right, door {off:+.1f} deg off square"
        self.events.append({"phase": "reanchor_lying", "text": text})
        self._plan_insert(self.base_script(obs.base_odom))
        if self._blocked:
            self._blocked = f"{self._blocked} ({text})"
        elif any(p.name == "lock_look" for p in self.script.phases):
            self._plan_look(self.base_script(obs.base_odom))
        return self._blocked

    def _plan_insert(self, base_s) -> None:
        """From the lying base (`base_s`, script frame): the key's poses on the keyway's axis -- 60 and 25 mm out, at
        the touch, seated, and seated `SEARCH_R_M` off across it either way -- solved with the hand rolled as it holds
        the key, clear of the trunk proxy and the head camera, holdable; each warm-started from the last. The first
        (25 mm out) is `key_go`'s goal. `_blocked` says why not."""
        to_base = plan.invert(base_s)
        box = perception.site_pose(self.fix.site)
        into, bar = self.key_axes()
        rot = decoupled_arm.hand_rotation(to_base[:3, :3] @ into, to_base[:3, :3] @ bar, 0.0, kg.GRASP_ROLL)
        q, rows = None, []
        r = SEARCH_R_M
        for name, (off, a, b) in (("view", (VIEW_OFF_M, 0, 0)), ("pre", (PRE_OFF_M, 0, 0)),
                                  ("touch", (TRAVEL_M, 0, 0)), ("seated", (0.0, 0, 0)), ("across+", (TRAVEL_M, r, r)),
                                  ("across-", (TRAVEL_M, -r, -r))):
            point = plan.transform(to_base, [jaw_for(box, off, a, b)])[0]
            seeds = _seeds(point, [self.arm.q] if q is None else [q])
            result = solve_link6(self.arm.joints, self.arm.links, point, rot, seeds, self.arm.offset)
            if result is None:
                self._blocked = (f"lying down, the arm cannot hold the key {name} ({1000 * off:.0f} mm out of seated): "
                                 "no IK pose clear of the trunk and holdable from here")
                self.events.append({"phase": "plan_insert", "failed": name})
                return
            q = np.asarray(result.q, dtype=float)
            camera = kp.camera_margin(self.arm.joints, q)
            torque = float(np.max(np.abs(_gravity(self.arm, q))))
            rows.append({"pose": name, "camera_mm": round(1000 * camera, 1), "shoulder_share": round(torque, 2)})
            if camera < 0.0:
                self._blocked = f"lying down, holding the key {name} the arm would be {-1000 * camera:.0f} mm into the head camera"
                return
            if name == "view":
                self._q_pre = q                     # `key_go`'s goal
        self.check["plan"] = rows
        self.events.append({"phase": "plan_insert", "poses": rows})

    def _solve_joint_goal(self, phase: str, end_b, to_base) -> np.ndarray:
        if phase != "key_go":
            return super()._solve_joint_goal(phase, end_b, to_base)
        start = self.arm.q.copy()
        if self._q_pre is None:
            self._blocked = self._blocked or "the insertion was not planned"
            return start
        clear = bool(path_clear(self.arm.joints, start, self._q_pre)[0])
        self.events.append({"phase": "key_go", "path_clear": clear})
        if not clear:
            self._blocked = "the move to the key's first pose crosses the trunk proxy"
            return start
        self.arm.q = self._q_pre.copy()
        return self._q_pre

    def _across(self, v) -> np.ndarray:
        """`v` (script frame) without its part along the keyway's axis."""
        _, out, _, _ = _frames(perception.site_pose(self.fix.site))
        v = np.asarray(v, dtype=float)
        return v - (v @ out) * out

    def _goal_servo(self, cmd, base_s, measured, st):
        """The parent's, its point moved by the hand's outer loop at the key: the jaw centre's miss as the encoders put
        it, integrated (all of it on the way to `key_pre`'s point, across the axis only once the key goes in). Folded
        until `unfold` begins: the walk up comes first."""
        import dataclasses

        if cmd.phase == "unfold":
            self._folded_walk = False
        if cmd.phase == "lock_look":
            # The wrist camera's first look: the aim and the pans, the arm still folded; the fold again once it is over.
            self.status = "joint"
            look = self._look
            return np.asarray(plan.FOLD_SERVO_DEG if look.done else look.places[look.i], dtype=float)
        if self._folded_walk:
            self.status = "joint"
            return np.asarray(plan.FOLD_SERVO_DEG)

        if cmd.phase in self.at_the_lever_phases and cmd.phase != "release":
            q = d1_ik.from_servo_deg(measured)[0]
            pos, rot = d1_ik.tool_pose(self.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
            jaw = plan.transform(base_s, [pos + rot @ np.asarray(JAW_CENTRE_LINK6)])[0]
            miss = np.asarray(cmd.point) - jaw
            if cmd.phase != "key_view":
                miss = self._across(miss)
            # Integrated only where the target holds still: in `key_view` once the arm has settled there (all of the
            # miss: the arm's sag), and across the axis while the key goes in. Not while `key_pre` moves the hand in:
            # the arm's lag behind a moving target wound the loop up along the axis, and the key then ran 6 mm past
            # its command into the keyway (the dry tests, 2026-10-21).
            settled = cmd.phase != "key_view" or self._view.settled(self._now)
            if cmd.phase not in ("key_pre", "key_back") and settled:
                self._loop = self._loop + LOOP_GAIN * 0.1 * miss
                norm = float(np.linalg.norm(self._loop))
                if norm > LOOP_MAX_M:
                    self._loop *= LOOP_MAX_M / norm
            cmd = dataclasses.replace(cmd, point=tuple(np.asarray(cmd.point) + self._loop))
        return super()._goal_servo(cmd, base_s, measured, st)

    def _walk_up_note(self, actual) -> dict:
        ahead, right, off = seated_from(self.fix, actual)
        return {"seated_ahead_m": round(ahead, 3), "lock_right_m": round(right, 3), "yaw_off_deg": round(off, 1)}

    def _abort(self, obs: plan.Observation, phase: str) -> str:
        if self._blocked:
            return self._blocked
        return super()._abort(obs, phase)

    # --- recovery ---------------------------------------------------------------------------------------------------

    def recover(self) -> None:
        """After an abort: the key straight back out along its axis if it is at the lock (from where the encoders put
        the hand, jaws as they were), home; then, if it is still held, handed back (a countdown, `drop`); folded; stood up if
        lying."""
        last = self.last_command
        released = self.check["released"] or (last is not None and last.phase == "release")
        held = bool(self.check["held"]) and not released
        Phase = demo_script.Phase
        carry = lambda st, u: carry_point(st["base"], st["base_z"])
        jaw = JAW_SHUT_M if held else JAW_OPEN_M
        phases = []
        if last is not None and last.phase in self.at_the_lever_phases and self._measured is not None:
            base_s = self._base_s if self._base_s is not None else np.eye(4)
            q = d1_ik.from_servo_deg(self._measured)[0]
            pos, rot = d1_ik.tool_pose(self.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
            hand = plan.transform(base_s, [pos + rot @ np.asarray(JAW_CENTRE_LINK6)])[0]
            approach = base_s[:3, :3] @ rot[:, 2]
            axis = base_s[:3, :3] @ rot[:, 0]
            back = tuple(float(v) for v in hand - BACK_OUT_M * approach)
            phases.append(Phase("key_out", BACK_S, target=lambda st: back, jaw_m=jaw, approach=lambda st: tuple(approach),
                                object_axis=lambda st: tuple(-axis), servo=True))
        if last is not None and last.phase not in ("unfold", "grip", "drop", "fold"):
            phases.append(Phase("home", HOME_S, path=carry, jaw_m=jaw, **carried()))
        if self._posture == "lie":
            phases += stand_phases(jaw)
        phases += hand_back_phases(jaw) if held else [Phase("fold", plan.FOLD_S, path=carry, jaw_m=JAW_OPEN_M)]
        start = last.hand_target_script if last and last.hand_target_script else demo_script.HOME_POINT
        self.script = demo_script.DemoScript(phases)
        self.script.reset(start, jaw)
        self._joint_goal = self._held = self.held_rotation = None
        self._joint_wait_s = 0.0
        self._lag_since = self._door_since = self._ik_since = None
        self._blocked = ""
        self.events.append({"phase": "recover", "phases": [p.name for p in phases]})

    def insert_check(self) -> dict:
        """What the robot measures of the insertion: the key held, the drop lying down, the plan's poses, where the key
        touched and entered, how far the search went, seated or not, and the key's depth now."""
        out = dict(self.check)
        if self._measured_key is not None:
            out["key_mm"] = [round(1000 * v, 2) for v in self._measured_key]
        return out


def _gravity(arm, q) -> np.ndarray:
    """Each joint's gravity torque at `q` as a share of its limit."""
    from position_only.workspace import EFFORT_LIMIT_NM, gravity_torques

    return np.abs(gravity_torques(arm.joints, arm.links, np.asarray(q).reshape(1, 6)))[0] / EFFORT_LIMIT_NM


def _load_calibration(path) -> dict:
    """The kept correction of the camera's keyhole (`CALIBRATION_PATH`): {"offset_m": (along u, along t), "runs": [...]}."""
    import json
    from pathlib import Path

    if path and Path(path).is_file():
        data = json.loads(Path(path).read_text())
        return {"offset_m": tuple(data.get("offset_m", (0.0, 0.0))), "runs": data.get("runs", [])}
    return {"offset_m": (0.0, 0.0), "runs": []}


def _save_calibration(path, calibration: dict, residual) -> dict:
    """`calibration` with a run's residual added (capped at `CALIBRATION_MAX_M`), its offset the runs' mean; written to
    `path` if there is one."""
    import json
    import time
    from pathlib import Path

    residual = np.asarray(residual, dtype=float)
    norm = float(np.linalg.norm(residual))
    if norm > CALIBRATION_MAX_M:
        residual = residual * (CALIBRATION_MAX_M / norm)
    runs = list(calibration["runs"]) + [{"utc": time.strftime("%Y%m%dT%H%M%S", time.gmtime()),
                                         "residual_m": [round(float(v), 5) for v in residual]}]
    offset = tuple(float(v) for v in np.mean([r["residual_m"] for r in runs], axis=0))
    out = {"offset_m": offset, "runs": runs}
    if path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps({"what": "the wrist camera's keyhole against where the key went in by touch "
                                                  "(insert_plan.py); offset_m along the key and up its top side",
                                          **out}, indent=2))
    return out

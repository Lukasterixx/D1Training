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
  torque. The drop is measured (sport mode's `body_height`). The fix is carried through the lie-down by the odometry,
  plus whatever part of the measured sit-back (`LIE_BACK_M`, 4 cm straight back) the odometry did not report, then
  **refitted on the lidar lying** (`_replan_lying`; Lukas, 2026-10-24: "you may do the lidar fit again"). It was off
  from 2026-10-22, when the key was handed in before the dog lay down and shadowed the door; since then the key comes
  after `lock_look`, so the arm is folded and empty through `lie_settle`.
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
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6, _seeds, arm_clear, holdable, solve_link6
from demos.unifp import decoupled_arm, script as demo_script

from .. import geometry as kg
from .. import holster
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
#: The holster's grip on the dog holds the shaft this far along the bar (toward the platform) from the jaw centre, not
#: on it. The draw puts the jaws within 1.4 mm of its planned grip by the encoders (Lukas's live runs, 2026-10-24 05:08
#: and 05:11 UTC), yet they close on the bar's free end past the shaft, the shaft just beyond the fingers, "consistently"
#: (his photos). The key then went in to the right of the keyhole. Taking the draw that much deeper instead ran the
#: wrist motor into the shoulder motor on the dog (Lukas: "can we instead add the offset the insertion phase itself").
#: So the insertion holds the jaws this far short of the shaft. 18 mm (half a 26 mm finger and the 10 mm shaft's radius,
#: read off the photos) pressed 8 mm left of the keyhole (Lukas, 2026-10-24), so 10 mm; Lukas's trials then found 6 mm
#: ("I find this to be the successful offset after a few tests": `--shaft_along_mm 6`). The hand-in's grip is at the
#: shaft (`KEY_IN_JAWS_LINK6`). A run nudges where the key lands on top of this (`InsertAttempt.key_right_m`,
#: `key_up_m`; `node.py --key_right_mm`, `--key_up_mm`).
#: Since 2026-10-25 the dog's draw takes the bar that much further along instead (`DOG_GRIP_ALONG_M`), so the jaws close
#: round the shaft (Lukas's photo: "It grabs the lever like this each time... it would be good to grab it more
#: cleanly", the fingers past the bar's end): the shaft on the jaws' axis, as the turn (Joint6 alone) takes it. On the
#: CPU model the v5 draw solves with the grip 6 and 10 mm along, every joint within 0.920 and 0.887 of its range
#: (0.968 at the shaft). (On v4 the jaws 18 mm along ran the wrist into the shoulder on the dog; v5's lever is 15 mm
#: further out.)
#: Then 2 mm back up the lever's axis, away from the holster (Lukas, 2026-10-25: "make the arm grab the lever from its
#: holster 2mm further up its axis (away from the holster)"): 4 mm along, so the shaft sits 2 mm along the bar from
#: the jaws' axis (`HOLSTER_SHAFT_ALONG_M`), and the insertion and its turn take the key there, off Joint6's axis.
DOG_GRIP_ALONG_M = 0.004
HOLSTER_SHAFT_ALONG_M = 0.006 - DOG_GRIP_ALONG_M
#: The holster key's landing nudges by default (`InsertAttempt.key_right_m`, `key_up_m`), to the robot's left and up,
#: m; `--key_right_mm`, `--key_up_mm` replace them. With the grip 4 mm along the key hit the keyway's bottom right and
#: did not go in (Lukas, 2026-10-25: "can you make it insert the key up 2mm and to the left 2mm"), so they were 2 and
#: 2; then "expose this as a launch argument" and "undo the -2, 2 default, make it 0,0": 0, set per run instead.
HOLSTER_KEY_LEFT_M = 0.0
HOLSTER_KEY_UP_M = 0.0
#: And this much further onto the bar along the hand's approach: with the grip 6 mm along, the fingertips stopped
#: 3.5 mm short of the bar's back face, where they are meant to be (Lukas's photo and measure, 2026-10-25: "There is a
#: 3.5mm gap to where the end of the pincers are supposed to be"). The CPU model: the draw solves, every joint within
#: 0.944 of its range, the open hand 29.6 mm off the key as it slides on.
DOG_GRIP_ONTO_M = 0.0035
#: The lock taken this much further from the robot (into the door) than the box's model and the door's fit put it
#: (`InsertAttempt._box`; the door's fit, the stance and the look's correction of the model are not moved). Lukas,
#: 2026-10-24: "it thinks the door is about a cm closer than it actually is ... and hence it always starts its
#: insertion wiggle very early". His 1 cm was not taken up: the key went straight in and stopped at 1.41, 0.40 and
#: -1.43 mm out of the model's seated in his runs of 05:54, 05:57 and 06:14 UTC (the 6 mm grip; he called it
#: "successful"), and stalled at -2.6 in the 06:16 one, so the model's depth is within ~3 mm. 10 mm deeper, a seated
#: key would stop 10 mm short of the model's seated and be taken for one on the face. The early wiggle was the wrist's
#: search running from the start of `key_in`, 25 mm out (`WRIST_START_M`). A run sets it (`InsertAttempt.key_in_m`,
#: `node.py --key_in_mm`).
#: The default since 2026-10-25 is the grip's 3.5 mm. Until then the holster's grip held the key 3.5 mm further out of the jaws than
#: `KEY_IN_JAWS_LINK6` (the fingertips short of the bar's back face, Lukas's measure), and the key seated straight in at
#: -1.4 to +2.2 mm of the model's seated (seven runs, 05:54-22:53 UTC). With the grip onto the bar (`DOG_GRIP_ONTO_M`),
#: Lukas's run of 23:27 UTC stopped at -4.61 mm, past the half keyway's -4.35 that is taken as seated, and searched
#: across the face for a minute: the lock is that much further in than the box's model has it.
LOCK_DEEPER_M = 0.0035


def key_in_jaws_link6(along_m: float = 0.0, roll: int = kg.GRASP_ROLL, up_m: float = 0.0) -> tuple[float, float, float]:
    """The key's origin in Link6 with its shaft `along_m` along the bar and `up_m` up its top side from the jaw centre:
    Link6's x is `roll` times the bar and y is `-roll` times the top side (`geometry.KEY_AXES_LINK6`, roll -1; the
    holster's +1 is that turned 180 deg about the shaft)."""
    return (roll * along_m, KEY_IN_JAWS_LINK6[1] - roll * up_m, KEY_IN_JAWS_LINK6[2])

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
#: Without the spiral (`InsertAttempt.spiral` False): how long the key stays pressed where it touched, the wrist's
#: search turning it (10 of its 2 s periods), before the search gives up.
PRESS_ONLY_S = 20.0
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
CONFIRM_BEFORE = ("unfold", "h_turn", "walk_in", "lie_down", "key_go", "key_in", "key_search", "key_turn", "release",
                  "h_return",
                  "stand_up",
                  "fold")
#: Where the key comes from (`InsertAttempt.key_source`): Lukas's holster beside the arm's base on the robot's right
#: (`demos/combiner_key/holster.py`, F-189, F-198), drawn out by the arm once the dog lies in front of the lock (Lukas,
#: 2026-10-24: "the unholstering can happen once the dog has sat down in front ot the keyhole"); or handed in.
KEY_SOURCES = ("holster", "hand")
#: The holster draw's phases (`holster_phases`) and their lengths: Joint1 turned at the fold, the arm unfolded to above
#: the waiting point and down to it (joint goals, `holster.draw_plan`), the open jaws slid onto the bar's root, closed
#: (`grip`, checked as the hand-in's), the key drawn out along the holster's axis.
H_TURN_S, H_REACH_S, H_LOWER_S, H_ON_S, H_DRAW_S = 4.0, 4.0, 2.0, 3.0, 5.0
#: The walk up's finish for the insertion (`InsertAttempt._close_enough`), and how many times it walks up again when the
#: refit after it puts the stance out of range (`_out_of_range_after_walk`). Lukas's run of 2026-10-24 23:41 UTC (the
#: look 9 degrees off square, his others 1-3): the walk up stopped 2.0 degrees off (the key's 4 accepted it), the refit
#: turned the door 2.2 more, and the range check (`key_plan.YAW_LIMIT_DEG`, 4) aborted at 4.0 ("its never done it
#: before?").
INSERT_ACCEPT_YAW_DEG = 2.0
INSERT_REWALKS = 1
#: The draw is `holster.py`'s for v5 (Lukas's holster with a mouth, 2026-10-24): `holster.SLIDE_M` along the axis
#: (`h_draw`), then `holster.EXIT_M` out through the mouth at `holster.EXIT_DEG` (`h_exit`), held `H_OUT_S` at the end
#: (`h_out`) so the arm is there before the key goes to the lock (Lukas: "the lever isnt pulled fully out of the
#: holster before it is moved towards the lock"); Joint1 and Joint5 to `holster.DRAW_SOFT_J1_J5` of their range out
#: of the holster and back (`WIDE_PHASES`). How far the encoders put the jaws along the axis is recorded
#: (`check["drawn_mm"]`, the plan's 93.1 mm). (v4's draw on the dog was 100 mm along the axis; an abort short of 96
#: mm was removed after Lukas's run of 06:10 UTC: "It has cleared it by 2cm".)
H_EXIT_S, H_OUT_S, H_CLEAR_S = 2.0, 3.0, 2.0
HOLSTER_PHASES = ("h_turn", "h_reach", "h_lower", "h_on", "grip", "h_draw", "h_exit", "h_out")
#: The draw's own outer loop (`_goal_servo`): the jaw centre's miss by the encoders, integrated across the holster's
#: axis through `h_draw` (the target moving along it) and all of it through `h_out` (still), and held through `h_exit`
#: and `h_clear`; its value at the end of `h_out` is where the return starts from. Lukas's run of 2026-10-24 21:54 UTC:
#: at the end of a 1.5 s `h_out` the encoders put the jaws 88.7 mm out of the planned 93.1 and 4.2 mm off the axis
#: (the arm's sag, 4-5 mm off all through the draw), so the lever's end, with the grip's 6 mm, was at the mouth's lip.
#: Then `h_clear` lifts the key `holster.CLEAR_M` away from the mouth (`holster.clear_direction`) before `key_go`.
DRAW_LOOP_PHASES = ("h_draw", "h_exit", "h_out", "h_clear")
RETURN_LOOP_PHASES = ("h_unclear", "h_align", "h_enter", "h_back")
#: Its gain, 1/s: slower than the insertion's `LOOP_GAIN`, which integrates only once the arm has settled. At 1.5 the
#: arm's lag behind the command made it overshoot (a 1.5-degree shoulder sag: +1.5 to -1.3 mm across through
#: `h_out`); at 0.8 it ends within 0.8 mm, against 3.0-3.6 mm without it (the dry tests, 2026-10-25).
DRAW_LOOP_GAIN = 0.8
DRAW_LOOP_MAX_M = 0.015
#: The key put back in the holster once it has turned the lock (Lukas, 2026-10-24: "make it so that the arm puts the
#: lever back in the holster after unlocking the door"; the key comes out of the lock turned, "Pull out at 45 deg"):
#: `key_back` with the jaws shut, straight out along the keyway's axis; `h_return`, a joint move to where the draw
#: ended (the IK's joints at the end of `h_clear`); then the draw played the other way: down from the lift to the exit
#: (`h_unclear`), into the mouth (`h_enter`) and
#: down the channel to the grip (`h_back`), the jaws open (`h_open`), the hand off the bar to the waiting point
#: (`h_off`), up and to the turned fold (`h_raise`, `h_unreach`), as the holster's recovery goes. Then the dog stands
#: and the arm folds. In the holster's simulation the mouth took the key in from a draw planned up to 5 mm off it
#: (8 of 8 at 0, 3 and 5 mm, 2026-10-24); on the dog the hand is stiffer than the simulation's spring grip.
#: `InsertAttempt.reholster` False (`node.py --leave_key`): the key let go in the lock instead and the hand backed away.
#: `h_back` is slower than the draw: Lukas's run of 2026-10-24 22:24 UTC aborted in it on the arm's lag (Joints 1, 4,
#: 5 and 6 10-24 degrees behind a moving target, the key going in), which the way back no longer aborts on (Lukas:
#: "can you make it not abort when returning? it did a successful holstering and stopped mid putting it back").
H_RETURN_S, H_ENTER_S, H_BACK_S, H_OPEN_S = 4.0, 4.0, 8.0, 1.0
#: Held at the exit (`h_align`) before going in, the draw's loop integrating all of the miss as through `h_out`; then
#: in more slowly, the loop across the axis (Lukas, 2026-10-25: "when returning the key, theres a very close call where
#: it begins contact with the upper angled wall of the mouth before sliding in... so it doesnt catch an edge"). His run
#: of 22:53 UTC: `h_enter` began with the arm still coming down from the lift, and with the draw's end value alone the
#: key reached the channel 3.6 mm toward its top wall (the walls 0.4 mm off the bar there); down the channel it ran
#: within 1-2 mm of the axis.
H_ALIGN_S = 2.0
#: Then folded before the dog stands (`h_unturn`: Joint1 back with the elbow still open; `h_fold`: the elbow shut; the
#: unfold's first leg undone), and held folded through the stand: on the way back of the run of 22:19 UTC the stand's
#: phases took the arm to the carry pose in front of the dog, and the fold from there dragged the fingers through the
#: lidar's box (the chain's proxy 33 mm inside it). On the CPU model this way back keeps the hand 63.6 mm off the
#: lidar and 20.2 mm off the holstered key (straight to the fold: 11.5).
RETURN_PHASES = ("h_return", "h_unclear", "h_align", "h_enter", "h_back", "h_open", "h_off", "h_raise", "h_unreach",
                 "h_unturn", "h_fold")
#: The return's phases at the holster, where an abort opens the jaws and slides off the bar (`_recover_at_holster`);
#: before it (`h_return`, `h_enter`: the key out of the channel) it is handed back.
RETURN_AT_HOLSTER = ("h_back", "h_open", "h_off", "h_raise", "h_unreach")
#: Folding after the key is back: an abort there finishes the fold (`h_fold`) and stands, the arm held folded.
RETURN_FOLDING = ("h_unturn", "h_fold")
#: Joint1 and Joint5 to `holster.DRAW_SOFT_J1_J5` of their range: out of the holster and back.
WIDE_PHASES = {"h_draw", "h_exit", "h_out", "h_clear", "h_return", "h_unclear", "h_align", "h_enter", "h_back", "h_open",
               "h_off"}
#: The wrist's search while the key is pushed in (Lukas, 2026-10-24: "rotating the wrist 10 degrees in either direction
#: while pushing so its more likely to go in"): the hand rolled about the key's shaft by `WRIST_SEARCH_DEG` times a sine
#: of period `WRIST_SEARCH_PERIOD_S` through `key_in` and `key_search`; held at the angle the key went in at for the
#: seat. The jaw centre and the key's origin are on the roll's axis, so the shaft does not move: only the wards turn.
WRIST_SEARCH_DEG = 10.0
WRIST_SEARCH_PERIOD_S = 2.0
#: The search begins with the key this far off the lock's face by the model (`TRAVEL_M` out of seated), or once it has
#: touched: from the start of `key_in`, 16 mm before the face, it looked to Lukas like a door 1 cm nearer than it is.
WRIST_START_M = 0.004
#: Each side of the search is as wide as the plan can hold (`_plan_insert`): the first of these, rolled that way, whose
#: touch, seat and spiral edge all solve. The holster's grip runs Joint6 near its end at the lock, and the higher the
#: lock over the lying base the further: on the CPU model -10 deg needs more than `HOLSTER_J6_SOFT` from a dog standing
#: 0.39 m over the box's root (the dog stands 0.35-0.39; Lukas's live run of 2026-10-24 13:22 aborted on it).
WRIST_SEARCH_STEPS_DEG = (10.0, 7.5, 5.0, 2.5, 0.0)
WRIST_SEARCH_PHASES = ("key_in", "key_search")
#: The holster's grip is the hand-in's turned 180 deg, which puts Joint6 near the end of its range at the lock: 2.08 rad
#: at the touch against the IK's soft limit of 2.12 (0.9 of the URDF's 2.35), and 2.27 rolled -10 deg. The holster
#: grip's attempt lets Joint6 alone go to `HOLSTER_J6_SOFT` of its range (2.30 rad, 2.7 deg inside the hard limit
#: `d1_hardware` clamps to); the other joints keep 0.9.
HOLSTER_J6_SOFT = 0.98
#: What the log says through the hand-in and the hand-back countdowns.
LOAD_TEXT = ("put the key in the open jaws (the fingers on the bar right at the shaft, so the shaft points ahead on the "
             "gripper's axis, the lever level and to the right, its platform facing up) and hold it "
             "there: they close in {n} s")
HAND_BACK_TEXT = "take hold of the key: the jaws open in {n} s"
REACTIVE = {"key_in", "key_search", "key_seat", "key_close", "key_turn", "key_back"}
#: The turn once seated (Lukas, 2026-10-24: "I have a feeling the arm can turn the lock at its unholstering grip: after
#: inserting, try rotating 45 degrees before backing away"): Joint6 alone turned from the angle the key went in at
#: (Lukas: "assume the key is in the roll axis perfectly, so you only need to rotate the wrist": the jaw centre stays
#: where it was seated, `_target`'s `pivot`), clockwise as the robot sees it (the lever down at its right: `geometry.lever_dir_box`), by
#: `InsertAttempt.turn_deg` (`ARM_LEVER.end_deg`, the v4 lock's 45; it unlocks at its 30), over `TURN_RAMP_S`, then
#: held to `TURN_S`. Then the jaws open and the hand backs away: the key stays in the lock, turned. The plan
#: (`_plan_insert`) takes the first of the turn, 5 deg less, ... down to `TURN_MIN_DEG` whose seated pose with Joint6
#: turned that far beyond the wrist search's widest the arm holds; none: no turn. The lock holding the key back shows as the arm's
#: lag (`plan.ARM_LAG_DEG`): an abort, the jaws opened where they are and the hand backed away, the key left in it.
TURN_RAMP_S, TURN_S = 4.0, 5.0
TURN_MIN_DEG = 30.0
#: The unlock's turn is commanded this much past `turn_deg` (`InsertAttempt.turn_extra_deg`, `node.py
#: --turn_extra_deg`): on the dog a 45-degree command did not turn the key the full 45 (Lukas, 2026-10-25: "if it is
#: commanded 45, lets command it 50 to compensate"; then "make the default overturn 10 degrees actually": 55). The
#: plan's step-down (5 deg at a time to `TURN_MIN_DEG`) starts from the sum. The lock's turn back to level is not
#: extended.
UNLOCK_OVERTURN_DEG = 10.0
#: **Locking** (`InsertAttempt.lock`; `node.py --lock`, and the door close's chain, `--then_close`). Lukas, 2026-10-25:
#: "insert it at -45 degrees since it is already unlocked and then push the key in while wiggling it. instead of
#: turning it instantly, push until the arm travels no further (door needs some force to fight the spring) and then
#: holding this position begin rotating from -45 to 0". The unlock turned the cylinder `ARM_LEVER.end_deg` clockwise
#: from level (`geometry.Lever`: positive clockwise as the robot sees it) and the key came out there, so the keyway
#: takes the key only at that angle: the whole insertion (the plan, the wiggle about it) is rolled `lock_from_deg`
#: (45, Lukas's -45 in his sense) and the turn is back to level. Between the seat and the turn, `key_close`: the key
#: pressed straight on to `LOCK_BEYOND_FACE_M` beyond the box's front face, or until it stops following its command
#: (`CONTACT_LAG_M` for `CONTACT_HOLD_S`), at `key_in`'s pace; held there, pressing, through the turn, which goes to
#: level whatever angle the wiggle had the key in at. Until 2026-10-25 it pushed the key round the door's closing arc
#: until it stopped (aborting after `CLOSE_MAX_DEG`), which on the dog with the door already shut went on until the IK
#: ran out, the box moving (`LOCK_BEYOND_FACE_M`'s note). The phase's clock is still `CLOSE_S`.
CLOSE_LOCK_FROM_HINGE_M = abs(kg.LOCK[1] - plan.GEOMETRY.hinge[1])
CLOSE_SPEED_DEG_S = math.degrees(IN_SPEED_M_S / CLOSE_LOCK_FROM_HINGE_M)
CLOSE_MAX_DEG = 15.0
CLOSE_S = CLOSE_MAX_DEG / CLOSE_SPEED_DEG_S + 4.0
#: Locked: the encoders put the key this near level after the turn back (`lock_check`): past the lock's unlock angle
#: (`ARM_LEVER.unlock_deg`) from the end, the way back.
LOCKED_DEG = kg.ARM_LEVER.end_deg - kg.ARM_LEVER.unlock_deg
#: Locking, the key is pressed straight in until its shaft's tip is `LOCK_BEYOND_FACE_M` beyond the box's front face, as
#: the lidar saw it lying down before the arm came out (`_refit_lying`; without a face there, the standing refit and
#: the sit-back), then turned (`key_close`, `_press_off`). Lukas, 2026-10-25: "it kept pushing until the arm was fully
#: outstretched, even moving the box. we need a better condition to start turning. make it go a certain distance beyond
#: the front face of the box according to the lidar (scanned before the arm covers the lidar)", and "expose this as a
#: launch argument" (`node.py --lock_beyond_face_mm`). In that run the close had pushed the door shut, the key went 37 mm
#: past seated in `key_in`, was taken as pushing the door shut (the handover then), and `key_close` carried it on round
#: the arc for 35 s until the IK ran out. Seated, the tip is 7.7 mm beyond the face (the keyway's 4.2 behind it and
#: `LOCK_DEEPER_M`). A key that stops first, past half the keyway out of seated, is in: pressed as far as it goes, it
#: turns there. Further out it touched the lock's face, and the search follows. No abort for going on.
LOCK_BEYOND_FACE_M = 0.012
#: The press is at its depth once its command has been there this long (the arm's lag along the axis is 1.5-3 mm).
PRESS_DWELL_S = 1.0


# --- the look --------------------------------------------------------------------------------------------------------

def _frames(box):
    """The keyway's (seated origin, out of the door, along the key at insertion, its top side), in `box`'s frame."""
    box = np.asarray(box)
    rot = box[:3, :3]
    seated = box @ np.array([kg.LOCK[0] + SEATED_OFF_DOOR_M, kg.LOCK[1], kg.LOCK[2], 1.0])
    return (seated[:3], rot @ np.asarray(kg.OUT_OF_DOOR_BOX), rot @ np.asarray(kg.lever_dir_box(kg.ARM_LEVER.insert_deg)),
            rot @ np.asarray(kg.lever_top_box(kg.ARM_LEVER.insert_deg)))


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


def keyhole_prediction(fix, box=None):
    """Where the model puts the bezel's front centre, out of the door, along the key and up its top side, in `fix`'s
    script frame: what `keyhole.find_bezel` searches round (an attempt's, with its `_box`; and `node.py --stage
    keyhole`'s)."""
    box = perception.site_pose(fix.site) if box is None else box
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
    th = math.radians(kg.ARM_LEVER.insert_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    ahead = np.array([c, s, 0.0])
    u = np.array([s * math.cos(th), -c * math.cos(th), -math.sin(th)])       # right and up, at 45 degrees
    return decoupled_arm.hand_rotation(ahead, u, 0.0, kg.GRASP_ROLL)


def carry_point(base, base_z: float) -> tuple[float, float, float]:
    """`KEY_CARRY_IN_BASE` for a base at (x, y, yaw) and height `base_z`, script frame."""
    x, y, z = KEY_CARRY_IN_BASE
    c, s = math.cos(base[2]), math.sin(base[2])
    return (base[0] + c * x - s * y, base[1] + s * x + c * y, base_z + z)


def jaw_for(box, off: float, a: float = 0.0, b: float = 0.0, along_m: float = 0.0, bar=None, up_m: float = 0.0,
            top=None) -> np.ndarray:
    """The jaw centre (script frame) holding the key with its back face `off` out of seated along the keyway's axis and
    `a`, `b` across it (along the key, up its top side), the shaft `along_m` along the bar and `up_m` up its top side
    from the jaws; the bar and its top side along `bar` and `top` (the wrist's search rolls them about the shaft) or the
    keyway's u and t."""
    seated, out, u, t = _frames(box)
    origin = seated + off * out + a * u + b * t
    bar = u if bar is None else np.asarray(bar, dtype=float)
    top = t if top is None else np.asarray(top, dtype=float)
    return origin + JAW_FROM_KEY[0] * u - along_m * bar - up_m * top + JAW_FROM_KEY[1] * out


def key_in_hand(joints, servo_deg, base_s, along_m: float = 0.0, roll: int = kg.GRASP_ROLL,
                up_m: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """(the key's origin, its (u, n, t) axes as columns), script frame, by forward kinematics of `servo_deg`, the key
    where the hand holds it (`key_in_jaws_link6`; `geometry.KEY_AXES_LINK6`, turned 180 deg about the shaft for roll
    +1)."""
    q = d1_ik.from_servo_deg(servo_deg)[0]
    pos, rot = d1_ik.tool_pose(joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
    origin = pos + rot @ np.asarray(key_in_jaws_link6(along_m, roll, up_m))
    axes = rot @ np.diag([-roll, -roll, 1.0]) @ np.asarray(kg.KEY_AXES_LINK6).T
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
    key = dict(approach=lambda st: tuple(attempt.key_axes()[0]), object_axis=lambda st: tuple(attempt.key_bar()),
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
    if getattr(attempt, "key_source", "hand") == "holster":
        phases += holster_phases(attempt)
    else:
        phases += [Phase("unfold", plan.UNFOLD_S, path=carry, jaw_m=JAW_OPEN_M)] + load_phases()
    if index >= STAGES.index("reach"):
        phases += [Phase("key_go", plan.UNFOLD_S, path=lambda st, u: attempt._target(), jaw_m=JAW_SHUT_M),
                   at_key("key_view", VIEW_S),
                   Phase("key_pre", PRE_S, target=lambda st: attempt._target(), jaw_m=JAW_SHUT_M, **key)]
        if index >= STAGES.index("touch"):
            phases.append(at_key("key_in", IN_S))
        put_back = stage == "insert" and getattr(attempt, "reholster", False) and \
            getattr(attempt, "key_source", "hand") == "holster"
        if stage == "insert":
            phases += [at_key("key_search", SEARCH_S), at_key("key_seat", SEAT_S)]
            if getattr(attempt, "lock", False):
                phases.append(at_key("key_close", CLOSE_S))
            phases.append(at_key("key_turn", TURN_S))
            if put_back:
                # The key, turned, out of the lock in the hand and back into the holster (`RETURN_PHASES`).
                return (phases + [at_key("key_back", BACK_S, JAW_SHUT_M)] + return_phases(attempt)
                        + stand_phases(JAW_OPEN_M) + [Phase("fold", plan.FOLD_S, path=carry, jaw_m=JAW_OPEN_M)])
            phases.append(at_key("release", RELEASE_S, JAW_OPEN_M))
        phases += [at_key("key_back", BACK_S, JAW_OPEN_M if stage == "insert" else JAW_SHUT_M),
                   Phase("home", HOME_S, path=carry, jaw_m=JAW_OPEN_M if stage == "insert" else JAW_SHUT_M, **carried_)]
    if stage == "insert":
        return phases + stand_phases(JAW_OPEN_M) + [Phase("fold", plan.FOLD_S, path=carry, jaw_m=JAW_OPEN_M)]
    if index >= STAGES.index("lie"):
        phases += stand_phases()
    return phases + hand_back_phases()


def holster_phases(attempt) -> list:
    """The key drawn out of the holster (`HOLSTER_PHASES`): its points the holster's, fixed in the base's frame, put in
    the script frame each step from where the base is (`InsertAttempt._holster_point`)."""
    Phase = demo_script.Phase
    point = lambda name: (lambda st, u: attempt._holster_point(name))
    line = lambda a, b: (lambda st, u: attempt._holster_point(a, b, u * u * (3.0 - 2.0 * u)))
    hand = dict(approach=lambda st: attempt._holster_axis("approach"), object_axis=lambda st: attempt._holster_axis("bar"),
                servo=True)
    opened, shut = holster.JAW_OPEN_M, JAW_SHUT_M
    return [Phase("h_turn", H_TURN_S, path=point("pre"), jaw_m=opened, **hand),
            Phase("h_reach", H_REACH_S, path=point("pre"), jaw_m=opened, **hand),
            Phase("h_lower", H_LOWER_S, path=point("pre"), jaw_m=opened, **hand),
            Phase("h_on", H_ON_S, path=line("pre", "grip"), jaw_m=opened, **hand),
            Phase("grip", GRIP_S, path=point("grip"), jaw_m=shut, **hand),
            Phase("h_draw", H_DRAW_S, path=line("grip", "out"), jaw_m=shut, **hand),
            Phase("h_exit", H_EXIT_S, path=line("out", "exit"), jaw_m=shut, **hand),
            Phase("h_out", H_OUT_S, path=point("exit"), jaw_m=shut, **hand),
            Phase("h_clear", H_CLEAR_S, path=line("exit", "clear"), jaw_m=shut, **hand)]


def return_phases(attempt) -> list:
    """The key put back (`RETURN_PHASES`): to where the draw ended, then the draw undone, the jaws open, off the bar and
    folded back as the unfold came (the holster's points as `holster_phases`' are)."""
    Phase = demo_script.Phase
    point = lambda name: (lambda st, u: attempt._holster_point(name))
    line = lambda a, b: (lambda st, u: attempt._holster_point(a, b, u * u * (3.0 - 2.0 * u)))
    hand = dict(approach=lambda st: attempt._holster_axis("approach"), object_axis=lambda st: attempt._holster_axis("bar"),
                servo=True)
    opened, shut = holster.JAW_OPEN_M, JAW_SHUT_M
    return [Phase("h_return", H_RETURN_S, path=point("clear"), jaw_m=shut, **hand),
            Phase("h_unclear", H_CLEAR_S, path=line("clear", "exit"), jaw_m=shut, **hand),
            Phase("h_align", H_ALIGN_S, path=point("exit"), jaw_m=shut, **hand),
            Phase("h_enter", H_ENTER_S, path=line("exit", "out"), jaw_m=shut, **hand),
            Phase("h_back", H_BACK_S, path=line("out", "grip"), jaw_m=shut, **hand),
            Phase("h_open", H_OPEN_S, path=point("grip"), jaw_m=opened, **hand),
            Phase("h_off", H_ON_S, path=line("grip", "pre"), jaw_m=opened, **hand),
            Phase("h_raise", H_LOWER_S, path=point("pre"), jaw_m=opened, **hand),
            Phase("h_unreach", H_REACH_S, path=point("pre"), jaw_m=opened, **hand),
            Phase("h_unturn", H_TURN_S, path=point("pre"), jaw_m=opened, **hand),
            Phase("h_fold", H_LOWER_S, path=point("pre"), jaw_m=opened, **hand)]


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
    #: Where the key comes from (`KEY_SOURCES`): drawn out of the holster lying down, or handed in.
    key_source: str = "hand"
    #: Where the key lands at the lock, moved from where the model of the grip puts it: to the robot's right and up, m
    #: (Lukas's trial and error on the dog, `node.py --key_right_mm`, `--key_up_mm`). The grip's model is the shaft
    #: `HOLSTER_SHAFT_ALONG_M` along the bar from the jaws for the holster, on their axis handed in; the nudges move
    #: the shaft in it the other way (`shaft_along`, `shaft_up`), so the jaws go that way and the wrist's search still
    #: turns about the shaft. None: the key source's default (`HOLSTER_KEY_LEFT_M`, `HOLSTER_KEY_UP_M` from the holster,
    #: 0 handed in).
    key_right_m: float | None = None
    key_up_m: float | None = None
    #: The lock taken this much further in (toward the door) than `LOCK_DEEPER_M` puts it, m (`node.py --key_in_mm`).
    key_in_m: float = 0.0
    #: The turn once seated, deg clockwise (`TURN_S`); 0: none, the key released where it went in.
    turn_deg: float = kg.ARM_LEVER.end_deg
    #: Unlocking, commanded this much past `turn_deg` (`UNLOCK_OVERTURN_DEG`); not with no turn (`turn_deg` 0).
    turn_extra_deg: float = UNLOCK_OVERTURN_DEG
    #: From the holster, the key put back in it after the turn (`RETURN_PHASES`); False: let go in the lock.
    reholster: bool = True
    #: Lock the door instead (`CLOSE_SPEED_DEG_S`'s note): the key in at `lock_from_deg` clockwise (the angle the unlock
    #: left the cylinder at), pressed to `lock_beyond_face_m` beyond the box's front face (`key_close`), turned back to
    #: level holding it there. `turn_deg` is then not used.
    lock: bool = False
    lock_from_deg: float = kg.ARM_LEVER.end_deg
    #: Locking, how far beyond the box's front face (the lidar's) the key's tip is pressed before it turns, m
    #: (`LOCK_BEYOND_FACE_M`, `node.py --lock_beyond_face_mm`).
    lock_beyond_face_m: float = LOCK_BEYOND_FACE_M
    #: The spiral across the lock's face when the key stops on it (`key_search`); False keeps the key pressed where it
    #: touched, the wrist's search (the wiggle) alone, for `PRESS_ONLY_S` (Lukas, 2026-10-25: "I want no_wiggle to
    #: still wiggle but not apply the translational spiralling"; `node.py --no_spiral`, alias `--no_wiggle`).
    spiral: bool = True

    joint_phases = {"unfold", "key_go", "fold", "h_turn", "h_reach", "h_lower", "h_return", "h_raise", "h_unreach",
                    "h_unturn", "h_fold"}
    dwell_phases = frozenset()
    position_only_phases = {"walk_in", "walk_settle"}
    #: The arm's lag abort (`plan.ARM_LAG_DEG`) while the key presses on the lock.
    holding_phases = {"key_in", "key_search", "key_seat", "key_close", "key_turn", "key_back"}
    #: Where an abort backs the key out along the keyway's axis before anything else.
    at_the_lever_phases = {"key_view", "key_pre", "key_in", "key_search", "key_seat", "key_close", "key_turn", "release",
                           "key_back"}
    walking_phases = {"walk_in"}
    release_held_phases = frozenset()
    refit_limit_m = kp.REFIT_LIMIT_M
    confirm_before = CONFIRM_BEFORE
    #: Not the door pull: no door reading through `grip` and `release`, which share its phases' names. With it, the
    #: lidar's door-angle thread ran back to back through the 43 s wait to hand the key in, and the odometry went 1.4
    #: and 1.8 s stale (Lukas's first two live runs, 2026-10-21).
    reads_door = False
    grasp_raise_m = 0.0                 # not the handle (`plan.GRASP_RAISE_M`)
    handle_jaws = False                 # the key's and the holster's jaw openings (`plan.Attempt.gripper_units`)

    @property
    def record_key(self) -> str | None:
        """Where the node's record keeps this attempt's events: locking, beside an unlock's (`node._write_run`)."""
        return "lock_attempt_events" if self.lock else None

    def _make_phases(self) -> list:
        return stage_phases(self.stage, self)

    def _prepare(self) -> None:
        if self.key_source not in KEY_SOURCES:
            raise ValueError(f"key_source is one of {KEY_SOURCES}, not {self.key_source!r}")
        # The holster's grip holds the key turned 180 deg about its shaft from the hand-in's (`holster.GRIP_ROLL` +1:
        # the insertion's roll -1 does not slide onto the bar there), so the whole attempt holds the hand that way: at the
        # lock the lever then points as the hand-in's does.
        self.grip_roll = holster.GRIP_ROLL if self.key_source == "holster" else kg.GRASP_ROLL
        holstered = self.key_source == "holster"
        if self.key_right_m is None:
            self.key_right_m = -HOLSTER_KEY_LEFT_M if holstered else 0.0
        if self.key_up_m is None:
            self.key_up_m = HOLSTER_KEY_UP_M if holstered else 0.0
        self.shaft_along = (HOLSTER_SHAFT_ALONG_M if holstered else 0.0) - float(self.key_right_m)
        self.shaft_up = -float(self.key_up_m)
        self.lock_deeper = LOCK_DEEPER_M + float(self.key_in_m)
        self.arm.grasp, self.arm.grasp_note = (0.0, self.grip_roll), f"the key's grip (pitch 0, roll {self.grip_roll:+d})"
        self.arm.soft = ((d1_ik.SOFT_LIMIT_FACTOR,) * 5 + (HOLSTER_J6_SOFT,) if self.key_source == "holster"
                         else d1_ik.SOFT_LIMIT_FACTOR)
        self._soft = self.arm.soft
        self._draw = (holster.draw_plan(grip_along_m=DOG_GRIP_ALONG_M, grip_onto_m=DOG_GRIP_ONTO_M)
                      if self.key_source == "holster" else None)
        # The roll the key goes in at (`lock`: the angle the unlock left the cylinder at), which the wiggle is about.
        self._roll0 = float(self.lock_from_deg) if self.lock else 0.0
        self._close = {"deg": 0.0, "held": False}  # `key_close`: the door's closing turn about its hinge, deg
        self._wrist = {"theta": self._roll0, "t0": None, "held": False}
        self._wrist_span = (WRIST_SEARCH_DEG, WRIST_SEARCH_DEG)   # (+, -) degrees, as far as `_plan_insert` solves
        self._blocked = ""
        self._body0: float | None = None          # sport mode's body height as the attempt began, standing
        self._drop = 0.0                           # how far below that the base is now
        self._heights: list = []                   # (t, body height) of the last second
        self._posture = ""
        self._posture_phase, self._posture_t = "", 0.0
        self._grips = 0
        self._rewalks = 0                          # walks up again after the refit (`_out_of_range_after_walk`)
        self._slip_since: float | None = None
        self._in_t0 = 0.0
        self.camera_gaps_mm: dict[str, float] = {}
        self._folded_walk = True                   # the arm holds the fold until `unfold` begins (the walk comes first)
        self._stand_odom: np.ndarray | None = None  # the base in odom as it began to lie down
        self._lie_scans: list = []                 # (base in odom, points) through `lie_settle`, for the lying refit
        self._loop = np.zeros(3)
        self._hloop = np.zeros(3)                  # the draw's outer loop (`DRAW_LOOP_PHASES`), script frame
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
        self.check = {"key_source": self.key_source, "key_right_mm": round(1000 * self.key_right_m, 2),
                      "key_up_mm": round(1000 * self.key_up_m, 2), "key_in_mm": round(1000 * self.key_in_m, 2),
                      "lock_deeper_mm": round(1000 * self.lock_deeper, 2), "shaft_along_mm": round(1000 * self.shaft_along, 2),
                      "shaft_up_mm": round(1000 * self.shaft_up, 2), "drawn_mm": None, "turn_deg": None,
                      "turned_deg": None, "left_in_lock": False, "held": None, "lie_drop_m": None, "plan": None, "look": None,
                      "vision": None, "spiral": self.spiral, "wrist_search_deg": None, "wrist_at_entry_deg": None,
                      "contact_off_mm": None,
                      "entered_at_mm": None,
                      "searched_r_mm": 0.0, "seated": None, "measured_off_mm": None, "released": False,
                      "lock": bool(self.lock), "lock_from_deg": self._roll0 if self.lock else None, "closed_deg": None,
                      "closed": None}
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
        _, out, u, _ = _frames(self._box())
        return -out, u

    def key_bar(self) -> np.ndarray:
        """The bar's axis the hand is held to at the key: `key_axes`'s, rolled about the way in by the wrist's search
        angle (`WRIST_SEARCH_DEG`)."""
        into, bar = self.key_axes()
        return decoupled_arm.axis_angle(into, math.radians(self._wrist["theta"])) @ bar

    def _box(self) -> np.ndarray:
        """The box's pose (script frame) the insertion works to: the fix's, moved `lock_deeper` into the door along its
        normal, so the lock's face, the seated key and the bezel are that much further from the robot."""
        box = perception.site_pose(self.fix.site).copy()
        box[:3, 3] -= self.lock_deeper * (box[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX))
        closing = getattr(self, "_close", {"deg": 0.0})["deg"]
        if closing:
            # `key_close`: the door, and the lock and key in it, turned that far shut about the hinge.
            hinge = np.asarray(plan.GEOMETRY.hinge)
            turn = plan.pose(plan.rot_z(math.radians(-plan.GEOMETRY.hinge_axis_z * closing)))
            box = box @ plan.pose(None, hinge) @ turn @ plan.pose(None, -hinge)
        return box

    def _offset_axes(self) -> tuple[np.ndarray, np.ndarray]:
        """The bar and its top side the shaft's offset in the jaws is taken along (`jaw_for`): rolled with the wrist's
        search, but not with the turn (`pivot`, the angle it began at): the turn is Joint6's alone, about the jaw
        centre."""
        into, bar = self.key_axes()
        t = _frames(self._box())[3]
        turn = decoupled_arm.axis_angle(into, math.radians(self._wrist.get("pivot", self._wrist["theta"])))
        return turn @ bar, turn @ t

    def _holster_point(self, a: str, b: str | None = None, f: float = 0.0) -> tuple:
        """A point of the holster draw (`holster.draw_plan`'s `pre`, `grip`, `out`; `f` of the way from `a` to `b`), in
        the script frame from where the base is now. The holster rides on the base, lying or standing."""
        d = self._draw
        p = d[a] if b is None else d[a] + f * (d[b] - d[a])
        base_s = self._base_s if self._base_s is not None else np.eye(4)
        return tuple(float(v) for v in plan.transform(base_s, [p])[0])

    def _holster_axis(self, which: str) -> tuple:
        """The draw's hand, as the approach (along the key's shaft) or the bar (along the lever) in the script frame."""
        axes = self._draw["axes"]
        v = -axes["n"] if which == "approach" else axes["u"]
        base_s = self._base_s if self._base_s is not None else np.eye(4)
        return tuple(float(x) for x in base_s[:3, :3] @ v)

    def _grasp_rotation(self, to_base):
        return carry_rotation(0.0)                  # base frame: the key held as it inserts, ahead

    def _target(self) -> tuple[float, float, float]:
        """The jaw centre for the insertion's state (`_ins`), or the planned first pose's before it is planned."""
        bar, top = self._offset_axes()
        point = jaw_for(self._box(), self._ins["off"], self._ins["a"], self._ins["b"],
                        self.shaft_along, bar, self.shaft_up, top)
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
        if before == "lie_settle" and obs.scan is not None:
            self._lie_scans.append(obs.scan)
        base_s = self.base_script(obs.base_odom)
        if obs.arm_servo_deg is not None:
            origin, _ = key_in_hand(self.arm.joints, obs.arm_servo_deg, base_s, self.shaft_along, self.grip_roll,
                                    self.shaft_up)
            seated, out, u, t = _frames(self._box())
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
        elif (self.check["held"] and not self.check["released"]
              and out.phase not in ("grip", "release", "h_open", "drop", "fold", "done") and not out.abort):
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
        if self.key_source == "holster":
            out.abort = (f"the jaws closed on nothing at the holster (servo 6 at {obs.gripper_units:.1f} units, shut "
                         f"{plan.GRIPPER_SHUT_UNITS:.1f}): no key there, or the hand is off the bar")
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
        self._wrist_step(phase, obs.now_s)
        if self._measured_key is None or phase not in REACTIVE:
            return dt, ""
        off_m, a_m, b_m = self._measured_key
        ins = self._ins
        if phase == "key_in":
            ins["off"] -= IN_SPEED_M_S * dt
            if self._stopped(off_m, obs):
                # Stopped with its back face within half the keyway's depth of seated: straight in. Otherwise on the
                # lock's face: near the shaft's length out, or past seated where the door is further than modelled.
                # Locking, any stop past half the keyway out is in: the press's end (`LOCK_BEYOND_FACE_M`).
                self._found(off_m, seated=(off_m < TRAVEL_M / 2) if self.lock else -TRAVEL_M / 2 < off_m < TRAVEL_M / 2)
                return self._skip(), ""
            if self.dry and ins["off"] < -OVERRUN_M:
                self._found(off_m, seated=True)
                self.events.append({"phase": "dry", "note": "no lock to touch: taken as seated"})
                return self._skip(), ""
            if self.lock and (ins["off"] <= self._press_off() or current.duration_s - elapsed <= dt + 1e-9):
                # Locking, at the press's depth (or the phase's end) without a stop: in, and no abort however far that
                # is (`LOCK_BEYOND_FACE_M`); `key_close` holds it there, then the turn.
                self._found(off_m, seated=True)
                return self._skip(), ""
            if not self.lock and (ins["off"] < -IN_SLACK_M or current.duration_s - elapsed <= dt + 1e-9):
                return 0.0, (f"the key went {1000 * IN_SLACK_M:.0f} mm past where it should seat without touching "
                             "anything: no lock there?")
            return dt, ""
        if phase == "key_search":
            contact = self.check["contact_off_mm"]
            if self.check["seated"] or contact is None:
                return self._skip(), ""
            contact /= 1000.0
            if off_m <= contact - ENTER_M:
                self._hold_wrist()
                self.check["entered_at_mm"] = [round(1000 * ins["a"], 2), round(1000 * ins["b"], 2)]
                self.events.append({"phase": "entered", "across_mm": self.check["entered_at_mm"],
                                    "off_mm": round(1000 * off_m, 2)})
                return self._skip(), ""
            if not self.spiral:
                # `--no_spiral`: pressed where it touched, `PRESS_M` in, the wrist's search alone.
                if elapsed >= PRESS_ONLY_S:
                    return 0.0, (f"the key touched {1000 * contact:.1f} mm out of seated and never went further in "
                                 f"{PRESS_ONLY_S:.0f} s of the wrist's search (no spiral)")
                a0, b0 = self._search_ab
                ins.update(off=contact - PRESS_M, a=a0, b=b0)
                return dt, ""
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
        if phase == "key_close":
            c = self._close
            if not self.check["seated"] or c["held"]:
                return self._skip(), ""
            # Locking: straight in to the press's depth (`LOCK_BEYOND_FACE_M`), or as far as the key goes, then the turn.
            target = self._press_off()
            ins["off"] = max(target, ins["off"] - IN_SPEED_M_S * dt)
            c["at_s"] = c.get("at_s", 0.0) + (dt if ins["off"] <= target + 1e-9 else 0.0)
            stopped = self._stopped(off_m, obs)
            if stopped or c["at_s"] >= PRESS_DWELL_S or current.duration_s - elapsed <= dt + 1e-9:
                c["held"] = True
                how = "stopped" if stopped else "at depth" if c["at_s"] >= PRESS_DWELL_S else "on the clock"
                beyond = self._beyond_face(off_m)
                self.check.update(closed=True, pressed_mm_beyond_face=round(1000 * beyond, 1), pressed=how)
                self.events.append({"phase": "pressed", "how": how, "off_mm": round(1000 * off_m, 2),
                                    "beyond_face_mm": round(1000 * beyond, 1),
                                    "target_beyond_face_mm": round(1000 * self.lock_beyond_face_m, 1)})
                return self._skip(), ""
            return dt, ""
        if phase == "key_turn":
            if not self._turn or not self.check["seated"] or (self.lock and not self._close["held"]):
                return self._skip(), ""
            w = self._wrist
            if "turn_from" not in w:
                w["turn_from"] = w["pivot"] = w["theta"]
                if not self.lock:
                    ins["off"] = off_m              # no press while it turns: where the key stopped seated (locking,
                                                    # the press holds the door shut against its spring)
            f = min(1.0, elapsed / TURN_RAMP_S)
            # Locking, to level (`_roll0 + _turn`, 0 for the whole turn) from wherever the key went in.
            turn = (self._roll0 + self._turn - w["turn_from"]) if self.lock else self._turn
            w["theta"] = w["turn_from"] + turn * f * f * (3.0 - 2.0 * f)
            return dt, ""
        if phase == "key_back":
            ins["off"] += BACK_OUT_M / current.duration_s * dt
            return dt, ""
        return dt, ""

    def _wrist_step(self, phase: str, now_s: float) -> None:
        """The wrist's search angle this step: a sine through `WRIST_SEARCH_PHASES` until the key goes in, then held;
        not begun until the key is `WRIST_START_M` off the lock's face, or has touched it."""
        w = self._wrist
        if w["held"] or phase not in WRIST_SEARCH_PHASES:
            return
        near = (self._measured_key is not None and self._measured_key[0] <= TRAVEL_M + WRIST_START_M)
        if w["t0"] is None and not (near or phase != "key_in" or self.check["contact_off_mm"] is not None):
            return
        if w["t0"] is None:
            w["t0"] = now_s
        wave = math.sin(2 * math.pi * (now_s - w["t0"]) / WRIST_SEARCH_PERIOD_S)
        w["theta"] = self._roll0 + wave * (self._wrist_span[0] if wave >= 0.0 else self._wrist_span[1])

    def _hold_wrist(self) -> None:
        """The key is in: the wrist kept at the angle it went in at."""
        if not self._wrist["held"]:
            self._wrist["held"] = True
            self.check["wrist_at_entry_deg"] = round(self._wrist["theta"], 2)
            self.events.append({"phase": "wrist_held", "deg": round(self._wrist["theta"], 2)})

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

    def _press_off(self) -> float:
        """Locking, the key's depth (its back face from seated, out of the door positive) whose tip is
        `lock_beyond_face_m` beyond the box's front face (the fix's, which the lidar puts there lying down)."""
        return SHAFT_M - SEATED_OFF_DOOR_M + self.lock_deeper - self.lock_beyond_face_m

    def _beyond_face(self, off_m: float) -> float:
        """How far beyond the box's front face the key's tip is at depth `off_m`."""
        return SHAFT_M - SEATED_OFF_DOOR_M + self.lock_deeper - off_m

    def _seated(self, off_m: float) -> None:
        self._hold_wrist()
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
        return keyhole_prediction(self.fix, self._box())

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
            clear = bool(plan.joint_path_clear(self.arm.joints, d1_ik.from_servo_deg(last)[0], q)[0])
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
        if self._draw is not None:
            self.arm.soft = holster.draw_soft(self._soft) if phase in WIDE_PHASES else self._soft
        if phase == "h_draw":
            self._hloop = np.zeros(3)
        if before == "h_out":
            self.check["draw_loop_mm"] = [round(1000 * v, 1) for v in self._hloop]
            self._draw["loop"] = self._hloop.copy()
            return self._drawn_check(obs)
        if before == "h_clear":
            self._draw["exit_q"] = np.asarray(self.arm.q, dtype=float).copy()     # `h_return`'s goal
        if phase == "h_unclear":
            self._hloop = np.asarray(self._draw.get("loop", np.zeros(3)), dtype=float).copy()
        if before == "h_back" and obs.arm_servo_deg is not None:
            self.check["returned_mm"] = round(1000 * self._along_holster(obs.arm_servo_deg), 1)
        if before == "h_fold":
            self._folded_walk = True                 # held folded from here (the stand, the last fold)
        if before == "h_open":
            self.check["released"] = True
            self.check["reholstered"] = True
            self.events.append({"phase": "reholstered", "returned_mm": self.check.get("returned_mm")})
        if phase == "lie_down":
            self._stand_odom = np.asarray(obs.base_odom).copy()
        if before == "lie_settle":
            return self._replan_lying(obs)
        if phase == "key_view":
            self._view.reset()
        if before == "key_view":
            self._use_vision()
            self._ins["off"] = PRE_OFF_M
        if phase in ("key_in", "key_seat", "key_close"):
            self._lag0, self._contact_since, self._in_t0 = None, None, obs.now_s
        if before == "key_turn" and self._turn and obs.arm_servo_deg is not None:
            _, axes = key_in_hand(self.arm.joints, obs.arm_servo_deg, self.base_script(obs.base_odom), self.shaft_along,
                                  self.grip_roll, self.shaft_up)
            _, _, u, t = _frames(self._box())
            self.check["turned_deg"] = round(math.degrees(math.atan2(-(axes[:, 0] @ t), axes[:, 0] @ u)), 1)
            self.events.append({"phase": "turned", "deg": self.check["turned_deg"], "planned_deg": self._turn})
        if phase == "key_back":
            if self._measured_key is not None:
                self._ins["off"] = self._measured_key[0]   # back out from where the key is, not where it was sent
            self.check["released"] = self.check["released"] or before == "release"
        return ""

    def _drawn_check(self, obs: plan.Observation) -> str:
        """After `h_out`: how far the encoders put the jaws along the holster's axis from the grip (`check["drawn_mm"]`),
        recorded only."""
        servo = obs.arm_servo_deg if obs.arm_servo_deg is not None else self._measured
        if servo is None or self._draw is None:
            return ""
        self.check["drawn_mm"] = round(1000 * self._along_holster(servo), 1)
        self.events.append({"phase": "drawn", "drawn_mm": self.check["drawn_mm"]})
        return ""

    def _along_holster(self, servo) -> float:
        """How far the encoders put the jaw centre out along the holster's axis from the grip, m."""
        q = d1_ik.from_servo_deg(servo)[0]
        pos, rot = d1_ik.tool_pose(self.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
        return float((pos + rot @ np.asarray(JAW_CENTRE_LINK6) - self._draw["grip"]) @ self._draw["axes"]["out"])

    def _replan_lying(self, obs: plan.Observation) -> str:
        """Lying down: the fix carried to where the base lies by the odometry and refitted on the lidar, and the
        insertion planned from there (`_plan_insert`).

        First the sit-back as measured (`LIE_BACK_M`, the first lie-down: ~4 cm straight back, which sport mode's
        odometry did not see, and ~4.5 deg of turn, which it did): whatever part of `LIE_BACK_M` the odometry did not
        report between standing and lying is added, back along the standing heading. The odometry goes on reporting the
        base that far ahead of where it is for the rest of the attempt, so the box is moved as far ahead in its frame
        (the base-relative box is then the true one).

        Then the lidar's word on the rest (`plan.refine_fix` on `lie_settle`'s scans, each from its base lowered by the
        drop: the odometry keeps the standing height): the door's face moved along its normal and turned, the tag's place
        along it and its height kept. The arm is folded and empty through `lie_settle` (the key comes after
        `lock_look`), so nothing of the robot's shadows the door. (Off from 2026-10-22 to -24, while the key was handed in
        before the dog lay down: "we cant trust the lidar after sitting down, as the tool obscures it".) Past the
        standing refit's limits it aborts; with no face found the sit-back alone stands. Then the fix is re-anchored on
        the odometry's own pose."""
        import dataclasses

        event = {"phase": "refit_lying", "used": False}
        if self._stand_odom is not None:
            x0, y0, yaw0 = plan.planar(self._stand_odom)
            x1, y1, _ = plan.planar(obs.base_odom)
            seen_back = -((x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0))
            extra = max(0.0, LIE_BACK_M - seen_back)
            drift = plan.pose(None, (extra * math.cos(yaw0), extra * math.sin(yaw0), 0.0))
            self.fix = dataclasses.replace(self.fix, script_odom=drift @ np.asarray(self.fix.script_odom))
            event.update(odometry_back_mm=round(1000 * seen_back, 1), sit_back_added_mm=round(1000 * extra, 1))
        scans = []
        for base, points in self._lie_scans:
            lowered = np.asarray(base, dtype=float).copy()
            lowered[2, 3] -= self._drop
            scans.append((lowered, points))
        refined, fit = plan.refine_fix(self.fix, scans)
        event["scans"] = len(scans)
        if not fit.ok:
            event["why"] = f"no face ({fit.why}); the sit-back alone"
        else:
            moved_m, turned_deg = plan.fix_change(self.fix, refined)
            event.update(used=True, moved_mm=round(1000 * moved_m, 1), turned_deg=round(turned_deg, 2))
            if moved_m > self.refit_limit_m or turned_deg > self.refit_limit_deg:
                self.events.append(event)
                return (f"abort: lying, the lidar puts the door {1000 * moved_m:.0f} mm and {turned_deg:.1f} deg from "
                        "where the odometry carried it")
            self.fix = refined
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
        the touch, seated, and the spiral's edge (`SEARCH_R_M` off across the keyway) either way -- solved with the hand
        as it holds the key, clear of the trunk proxy and the head camera, holdable; each warm-started from the last.
        The first (25 mm out) is `key_go`'s goal. Then, each way, the wrist's search as far as `WRIST_SEARCH_STEPS_DEG`
        allows (`_rolled`). `_blocked` says why not; a side of the search that holds no roll only narrows it
        (`_wrist_span`, `check["wrist_search_deg"]`)."""
        to_base = plan.invert(base_s)
        box = self._box()
        into, bar = self.key_axes()
        # Locking, every pose with the key rolled `_roll0` (the angle the unlock left the cylinder at).
        roll = decoupled_arm.axis_angle(into, math.radians(self._roll0))
        bar, top = roll @ bar, roll @ _frames(box)[3]
        rot = decoupled_arm.hand_rotation(to_base[:3, :3] @ into, to_base[:3, :3] @ bar, 0.0, self.grip_roll)
        r = SEARCH_R_M
        q, rows, solved = None, [], {}
        places = {"view": (VIEW_OFF_M, 0, 0), "pre": (PRE_OFF_M, 0, 0), "touch": (TRAVEL_M, 0, 0), "seated": (0.0, 0, 0),
                  "across+": (TRAVEL_M, r, r), "across-": (TRAVEL_M, -r, -r)}
        for name, (off, a, b) in places.items():
            point = plan.transform(to_base, [jaw_for(box, off, a, b, self.shaft_along, bar, self.shaft_up, top)])[0]
            seeds = _seeds(point, [self.arm.q] if q is None else [q])
            result = solve_link6(self.arm.joints, self.arm.links, point, rot, seeds, self.arm.offset, self.arm.soft)
            if result is None:
                self._blocked = (f"lying down, the arm cannot hold the key {name} ({1000 * off:.0f} mm out of seated): "
                                 "no IK pose clear of the trunk and holdable from here")
                self.events.append({"phase": "plan_insert", "failed": name})
                return
            q = np.asarray(result.q, dtype=float)
            row = self._pose_row(name, 0.0, q)
            if row["camera_mm"] < 0.0:
                self._blocked = (f"lying down, holding the key {name} the arm would be {-row['camera_mm']:.0f} mm into "
                                 "the head camera")
                return
            rows.append(row)
            solved[name] = q
            if name == "view":
                self._q_pre = q                     # `key_go`'s goal
        span = []
        for sign, tag in ((1.0, "+"), (-1.0, "-")):
            for amp in WRIST_SEARCH_STEPS_DEG:
                side = [self._rolled(solved[base], sign * amp, f"{name}{tag}", places[base], base_s)
                        for base, name in (("touch", "touch_wrist"), ("seated", "seated_wrist"), (f"across{tag}", "across_wrist"))]
                if all(row is not None for row in side):
                    span.append(amp)
                    rows += side
                    break
        self._wrist_span = (span[0], span[1])
        self.check["wrist_search_deg"] = [-span[1], span[0]]
        self._turn = 0.0
        # Unlocking, clockwise from where the key went in; locking, back to level (the other way): each checked from the
        # wrist search's far side that way, Joint6 alone.
        sign = -1.0 if self.lock else 1.0
        full = float(self.lock_from_deg if self.lock else
                     self.turn_deg + (self.turn_extra_deg if self.turn_deg > 0.0 else 0.0))
        amp = full
        while amp > 0.0 and amp >= min(TURN_MIN_DEG, full) - 1e-9:
            reach = amp + (span[1] if self.lock else span[0])
            row = self._rolled(solved["seated"], sign * reach, f"turned{sign * amp:g}")
            if row is not None:
                self._turn = sign * amp
                rows.append(row)
                break
            amp -= 5.0
        self.check["turn_deg"] = self._turn
        self.check["plan"] = rows
        self.events.append({"phase": "plan_insert", "poses": rows, "wrist_search_deg": [-span[1], span[0]]})

    def _pose_row(self, name: str, deg: float, q) -> dict:
        camera = kp.camera_margin(self.arm.joints, q)
        torque = float(np.max(np.abs(_gravity(self.arm, q))))
        return {"pose": name, "deg": deg, "camera_mm": round(1000 * camera, 1), "shoulder_share": round(torque, 2),
                "j6_rad": round(float(q[5]), 3)}

    def _rolled(self, q, deg: float, name: str, place=None, base_s=None) -> dict | None:
        """`q` with the hand rolled `deg` about the key's shaft, or None if that leaves the joint limits (`arm.soft`),
        the trunk proxy's clearance, the holdable or the head camera's clearance. With the key's shaft on Joint6's axis
        (the hand-in's grip, no nudge) the roll is Joint6 alone, `deg` the other way: no IK (a pose out of reach cost the dog's
        planner ~20 s of failing seeds, Lukas's run of 2026-10-24 13:34). With it off the axis (`shaft_along`,
        `shaft_up`: the holster's grip, the nudges) the jaws swing round the shaft: the IK for the key at `place` (off, a, b) from the lying base
        `base_s`, seeded from `q` alone."""
        rolled = np.asarray(q, dtype=float).copy()
        rolled[5] -= math.radians(deg)
        if (self.shaft_along or self.shaft_up) and place is not None:
            to_base = plan.invert(base_s)
            box = self._box()
            into, bar = self.key_axes()
            turn = decoupled_arm.axis_angle(into, math.radians(self._roll0 + deg))
            bar = turn @ bar
            point = plan.transform(to_base, [jaw_for(box, *place, self.shaft_along, bar, self.shaft_up,
                                                     turn @ _frames(box)[3])])[0]
            rot = decoupled_arm.hand_rotation(to_base[:3, :3] @ into, to_base[:3, :3] @ bar, 0.0, self.grip_roll)
            result = solve_link6(self.arm.joints, self.arm.links, point, rot, [rolled], self.arm.offset, self.arm.soft)
            if result is None:
                return None
            rolled = np.asarray(result.q, dtype=float)
        lows, highs = d1_ik.joint_limits(self.arm.joints, self.arm.soft)
        if not (lows[5] <= rolled[5] <= highs[5]):
            return None
        if not (bool(arm_clear(self.arm.joints, rolled)[0]) and holdable(self.arm.joints, self.arm.links, rolled)):
            return None
        row = self._pose_row(name, deg, rolled)
        return row if row["camera_mm"] >= 0.0 else None

    def _solve_joint_goal(self, phase: str, end_b, to_base) -> np.ndarray:
        goals = {"h_turn": "turned", "h_reach": "above", "h_lower": "q_pre", "h_raise": "above", "h_unreach": "turned"}
        if phase in goals and self._draw is not None:
            q = np.asarray(self._draw[goals[phase]], dtype=float).copy()
            if phase in ("h_turn", "h_unreach"):
                # The fold's own wrist and elbow as the arm reports them (the commandable fold, not the rest pose's),
                # the elbow opened as `holster.unfold_stages` has it.
                folded = d1_ik.from_servo_deg(plan.FOLD_SERVO_DEG)[0]
                q[1:] = folded[1:]
                q[2] += holster.TURN_ELBOW_RAD
            self.arm.q = q.copy()                # the IK through `h_on` carries on from the waiting point's branch
            return q
        if phase == "h_return":
            return self._return_goal()
        if phase in RETURN_FOLDING:
            q = np.asarray(d1_ik.from_servo_deg(plan.FOLD_SERVO_DEG)[0], dtype=float)
            if phase == "h_unturn":
                q[2] += holster.TURN_ELBOW_RAD           # Joint1 home first, the elbow still open
            self.arm.q = q.copy()
            return q
        if phase != "key_go":
            return super()._solve_joint_goal(phase, end_b, to_base)
        start = self.arm.q.copy()
        if self._q_pre is None:
            self._blocked = self._blocked or "the insertion was not planned"
            return start
        # The move the node sends (`plan.joint_path`, each joint paced on its own), not a straight line.
        clear = bool(plan.joint_path_clear(self.arm.joints, start, self._q_pre)[0])
        key_margin = key_path_margin(self.arm.joints, plan.joint_path_rad(start, self._q_pre), self.grip_roll,
                                     self.shaft_along, self.shaft_up)
        self.events.append({"phase": "key_go", "path_clear": clear, "key_margin_mm": round(1000 * key_margin, 1)})
        if not clear:
            self._blocked = "the move to the key's first pose crosses the trunk proxy"
            return start
        if key_margin < 0.0:
            self._blocked = (f"on the move to the key's first pose the key in the hand would pass {-1000 * key_margin:.0f} "
                             "mm into the trunk or the lidar")
            return start
        self.arm.q = self._q_pre.copy()
        return self._q_pre

    def _return_goal(self) -> np.ndarray:
        """`h_return`'s goal: the joints where the draw ended (`exit_q`), if the move there (`plan.joint_path`) keeps the
        arm off the trunk proxy and the held key off the trunk and the lidar; else blocked, the key handed back."""
        start = self.arm.q.copy()
        goal = self._draw.get("exit_q") if self._draw is not None else None
        if goal is None:
            self._blocked = self._blocked or "the draw's end was not recorded: nowhere to put the key back from"
            return start
        clear = bool(plan.joint_path_clear(self.arm.joints, start, goal)[0])
        key_margin = key_path_margin(self.arm.joints, plan.joint_path_rad(start, goal), self.grip_roll,
                                     self.shaft_along, self.shaft_up)
        self.events.append({"phase": "h_return", "path_clear": clear, "key_margin_mm": round(1000 * key_margin, 1)})
        if not clear:
            self._blocked = "the move back to the holster crosses the trunk proxy"
            return start
        if key_margin < 0.0:
            self._blocked = (f"on the move back to the holster the key in the hand would pass {-1000 * key_margin:.0f} "
                             "mm into the trunk or the lidar")
            return start
        self.arm.q = np.asarray(goal, dtype=float).copy()
        return self.arm.q.copy()

    def _across(self, v) -> np.ndarray:
        """`v` (script frame) without its part along the keyway's axis."""
        _, out, _, _ = _frames(self._box())
        v = np.asarray(v, dtype=float)
        return v - (v @ out) * out

    def _goal_servo(self, cmd, base_s, measured, st):
        """The parent's, its point moved by the hand's outer loop at the key: the jaw centre's miss as the encoders put
        it, integrated (all of it on the way to `key_pre`'s point, across the axis only once the key goes in). Folded
        until `unfold` (or the holster draw's `h_turn`) begins: the walk up comes first."""
        import dataclasses

        if cmd.phase in ("unfold", "h_turn"):
            self._folded_walk = False
        if cmd.phase == "lock_look":
            # The wrist camera's first look: the aim and the pans, the arm still folded; the fold again once it is over.
            self.status = "joint"
            look = self._look
            return np.asarray(plan.FOLD_SERVO_DEG if look.done else look.places[look.i], dtype=float)
        if self._folded_walk:
            self.status = "joint"
            return np.asarray(plan.FOLD_SERVO_DEG)

        if cmd.phase in DRAW_LOOP_PHASES + RETURN_LOOP_PHASES and self._draw is not None:
            # The draw's outer loop (`DRAW_LOOP_PHASES`): across the holster's axis while the hand moves along it, all
            # of the miss while it holds at the exit.
            q = d1_ik.from_servo_deg(measured)[0]
            pos, rot = d1_ik.tool_pose(self.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
            jaw = plan.transform(base_s, [pos + rot @ np.asarray(JAW_CENTRE_LINK6)])[0]
            miss = np.asarray(cmd.point) - jaw
            out = base_s[:3, :3] @ np.asarray(self._draw["axes"]["out"])
            if cmd.phase in ("h_draw", "h_enter", "h_back"):
                miss = miss - float(miss @ out) * out
            # All of it where the target holds still (`h_out`, `h_align`); across the axis while the hand moves along
            # it (the arm's lag is along its path: up to 24 degrees on the way back at `h_back`'s first 5 s); not while
            # it comes down from the lift (`h_unclear`), the draw's end value held.
            if cmd.phase in ("h_draw", "h_out", "h_align", "h_enter", "h_back"):
                self._hloop = self._hloop + DRAW_LOOP_GAIN * 0.1 * miss
                norm = float(np.linalg.norm(self._hloop))
                if norm > DRAW_LOOP_MAX_M:
                    self._hloop *= DRAW_LOOP_MAX_M / norm
            cmd = dataclasses.replace(cmd, point=tuple(np.asarray(cmd.point) + self._hloop))
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

    def _close_enough(self, base) -> bool:
        """The walk up's finish (`key_plan.WalkUp`), turned within `INSERT_ACCEPT_YAW_DEG`, not the key's 4: the
        refit after it may turn the door a couple of degrees more, against the range check's `key_plan.YAW_LIMIT_DEG`."""
        x, y, yaw = base
        return (math.hypot(x, y) <= kp.WALK_UP_ACCEPT_M
                and abs(math.degrees(plan.wrap(yaw))) <= INSERT_ACCEPT_YAW_DEG)

    def _out_of_range_after_walk(self, text: str) -> str:
        """The refit after the walk up put the stance out of range: up to `INSERT_REWALKS` times, the stance again from
        the refitted box (`insert_stance`, as the odometry counts it: `key_plan.ODOM_WALK_SCALE`) and the walk up and
        its refit again, standing, the arm folded; then the abort."""
        if self._rewalks >= INSERT_REWALKS:
            return text + (f" (after {self._rewalks} more walk up)" if self._rewalks else "")
        x, y, yaw = insert_stance(perception.site_pose(self.fix.site))
        k = kp.ODOM_WALK_SCALE
        self.fix = plan.reanchor(self.fix, np.asarray(self.fix.script_odom) @ plan.base_pose(k * x, k * y, yaw))
        self._rewalks += 1
        self._walk = None
        self._settle_scans = []
        walk = [p for p in self._make_phases() if p.name in ("walk_in", "walk_settle")]
        self.script = demo_script.DemoScript(walk + self.script.phases[self.script.index:])
        self.script.reset(demo_script.HOME_POINT, JAW_SHUT_M)
        self.events.append({"phase": "rewalk", "why": text, "to": [round(k * x, 3), round(k * y, 3),
                                                                  round(math.degrees(yaw), 1)]})
        return ""

    def _walk_up_note(self, actual) -> dict:
        ahead, right, off = seated_from(self.fix, actual)
        return {"seated_ahead_m": round(ahead, 3), "lock_right_m": round(right, 3), "yaw_off_deg": round(off, 1)}

    def _abort(self, obs: plan.Observation, phase: str) -> str:
        if self._blocked:
            return self._blocked
        return super()._abort(obs, phase)

    def _lag_joints(self, phase: str) -> tuple:
        """Joint6 left out while the wrist searches: its sine (±10 deg in 2 s) runs up to ~17 deg ahead of the D1,
        which pauses its reports and motion (F-200), and a gap there is the roll's, not a load's. Lukas's run of
        2026-10-24 13:34 aborted 21 s into `key_in` on it ("the arm is 16 deg from where the IK wants it: loaded"),
        servo 6 commanded 101.5 and at 119. The key's depth (`_stopped`) is on the roll's axis, so the touch still
        sees the face."""
        if phase in WRIST_SEARCH_PHASES and not self._wrist["held"]:
            return (0, 1, 2, 3, 4)
        return super()._lag_joints(phase)

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
        if last is not None and last.phase in RETURN_FOLDING:
            # The key back in the holster, the arm folding: the fold finished (`h_fold`, then held folded), stood up.
            phases = [Phase("h_fold", H_LOWER_S, path=carry, jaw_m=JAW_OPEN_M)]
            if self._posture == "lie":
                phases += stand_phases(JAW_OPEN_M)
            self.script = demo_script.DemoScript(phases + [Phase("fold", plan.FOLD_S, path=carry, jaw_m=JAW_OPEN_M)])
            self.script.reset(last.hand_target_script or demo_script.HOME_POINT, JAW_OPEN_M)
            self._joint_goal = self._held = self.held_rotation = None
            self._joint_wait_s = 0.0
            self._lag_since = self._door_since = self._ik_since = None
            self._blocked = ""
            self.events.append({"phase": "recover", "phases": [p.name for p in self.script.phases]})
            return
        if last is not None and last.phase in HOLSTER_PHASES + RETURN_AT_HOLSTER and self._draw is not None and not (
                held and last.phase == "h_draw" and self.script.finished):
            # At the holster: the jaws opened (the key, if half out, stays in the holster's sleeve), the hand slid off the
            # bar's root along the holster's axis to the waiting point's distance beyond where it is, then the way it came.
            self.events.append({"phase": "recover_at_holster", "from": last.phase})
            return self._recover_at_holster(last)
        if last is not None and last.phase in self.at_the_lever_phases and self._measured is not None:
            base_s = self._base_s if self._base_s is not None else np.eye(4)
            q = d1_ik.from_servo_deg(self._measured)[0]
            pos, rot = d1_ik.tool_pose(self.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
            hand = plan.transform(base_s, [pos + rot @ np.asarray(JAW_CENTRE_LINK6)])[0]
            approach = base_s[:3, :3] @ rot[:, 2]
            axis = base_s[:3, :3] @ rot[:, 0]
            if held and (last.phase == "key_turn" or (last.phase == "key_back" and self.reholster)):
                # Turned, or part way, or held back coming out: the jaws opened where they are, the key left in the
                # lock, and the hand backed away empty.
                here = tuple(float(v) for v in hand)
                phases.append(Phase("key_letgo", RELEASE_S, target=lambda st: here, jaw_m=JAW_OPEN_M,
                                    approach=lambda st: tuple(approach), object_axis=lambda st: tuple(-axis), servo=True))
                held, jaw = False, JAW_OPEN_M
                self.check["left_in_lock"] = True
                self.events.append({"phase": "left_in_lock"})
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

    def _recover_at_holster(self, last) -> None:
        Phase = demo_script.Phase
        hand = dict(approach=lambda st: self._holster_axis("approach"), object_axis=lambda st: self._holster_axis("bar"),
                    servo=True)
        base_s = self._base_s if self._base_s is not None else np.eye(4)
        if self._measured is not None:
            q = d1_ik.from_servo_deg(self._measured)[0]
            pos, rot = d1_ik.tool_pose(self.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
            here_b = pos + rot @ np.asarray(JAW_CENTRE_LINK6)
        else:
            here_b = self._draw["grip"]
        off_b = here_b + holster.PRE_OUT_M * self._draw["axes"]["out"]
        to_s = lambda p: tuple(float(v) for v in plan.transform(base_s, [p])[0])
        here_s, off_s = to_s(here_b), to_s(off_b)
        phases = [Phase("h_letgo", RELEASE_S, target=lambda st: here_s, jaw_m=holster.JAW_OPEN_M, **hand),
                  Phase("h_off", H_ON_S, target=lambda st: off_s, jaw_m=holster.JAW_OPEN_M, **hand),
                  Phase("h_raise", H_LOWER_S, path=lambda st, u: off_s, jaw_m=holster.JAW_OPEN_M, **hand),
                  Phase("h_unreach", H_REACH_S, path=lambda st, u: off_s, jaw_m=holster.JAW_OPEN_M, **hand)]
        if last.phase in ("h_turn", "h_reach", "h_lower", "h_raise", "h_unreach"):
            # Before the jaws were round the bar, or after they left it: up and folded, nothing to let go of.
            phases = phases[2:] if last.phase != "h_turn" else []
        if self._posture == "lie":
            phases += stand_phases(holster.JAW_OPEN_M)
        phases.append(Phase("fold", plan.FOLD_S, path=lambda st, u: off_s, jaw_m=holster.JAW_OPEN_M))
        self.script = demo_script.DemoScript(phases)
        self.script.reset(here_s, holster.JAW_OPEN_M)
        self._joint_goal = self._held = self.held_rotation = None
        self._joint_wait_s = 0.0
        self._lag_since = self._door_since = self._ik_since = None
        self._blocked = ""
        self.events.append({"phase": "recover", "phases": [p.name for p in phases]})

    def unlock_check(self) -> dict:
        """Whether the door may be pulled after this insertion (`node.py --then_door`), by what the robot measured: the
        key seated, turned at least the lock's `geometry.ARM_LEVER.unlock_deg` by the encoders, and the jaws empty
        (put back in the holster, or let go in the lock). {"verified", "why", and the check}."""
        check = self.insert_check()
        turned = check.get("turned_deg")
        unlock = kg.ARM_LEVER.unlock_deg
        why = ""
        if not check.get("seated"):
            why = "the key did not seat"
        elif turned is None or turned < unlock:
            why = f"the key turned {turned} deg, under the {unlock:g} the lock unlocks at"
        elif not (check.get("reholstered") or check.get("released") or check.get("left_in_lock")):
            why = "the key is still in the hand"
        return {"verified": not why, "why": why, **check}

    def lock_check(self) -> dict:
        """Whether this insertion locked the door, by what the robot measured: the key seated, pressed to its depth or as
        far as it went (`key_close`), the key turned back to within `LOCKED_DEG` of level by the encoders, and the jaws empty
        or the key back in the holster. {"verified", "why", and the check}."""
        check = self.insert_check()
        turned = check.get("turned_deg")
        why = ""
        if not check.get("seated"):
            why = "the key did not seat"
        elif not check.get("closed"):
            why = "the key was not pressed home"
        elif turned is None or abs(turned) > LOCKED_DEG:
            why = f"the key is at {turned} deg, not within {LOCKED_DEG:g} of level"
        elif not (check.get("reholstered") or check.get("released") or check.get("left_in_lock")):
            why = "the key is still in the hand"
        return {"verified": not why, "why": why, **check}

    def insert_check(self) -> dict:
        """What the robot measures of the insertion: the key held, the drop lying down, the plan's poses, where the key
        touched and entered, how far the search went, seated or not, and the key's depth now."""
        out = dict(self.check)
        if self._measured_key is not None:
            out["key_mm"] = [round(1000 * v, 2) for v in self._measured_key]
        return out


def key_path_margin(joints, configs, roll: int, along_m: float = 0.0, up_m: float = 0.0) -> float:
    """How far the held key's bar and shaft stay outside the trunk's box and the lidar's (`BODY_BOX_B`,
    `CLEARANCE_BOXES_B`) through the joint move's `configs` (radians; `plan.joint_path_rad`, as the node sends it), m;
    negative: that far in. The key is where the hand holds it (`key_in_jaws_link6`, `along_m`, `up_m`), its
    bar from 15 mm behind the shaft to 160 mm (and `holster.LEVER_EXTRA_M`) along the lever, which is `roll` times
    Link6's x (the hand's grip roll: -1 the hand-in's, `geometry.KEY_AXES_LINK6`; +1 the holster's)."""
    from position_only.workspace import BODY_BOX_B, CLEARANCE_BOXES_B

    boxes = [np.array(b, dtype=float) for b in (BODY_BOX_B, *CLEARANCE_BOXES_B)]
    length = 0.160 + holster.LEVER_EXTRA_M
    along = np.linspace(-0.015, length, 12)
    worst = np.inf
    for q in np.asarray(configs, dtype=float):
        pos, rot = d1_ik.tool_pose(joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
        origin = pos + rot @ np.asarray(key_in_jaws_link6(along_m, roll, up_m))
        pts = origin + np.outer(along, roll * rot[:, 0])
        for box in boxes:
            lo, hi = box[:, 0], box[:, 1]
            outside = np.linalg.norm(np.maximum(np.maximum(lo - pts, pts - hi), 0.0), axis=1)
            inside = np.minimum(pts - lo, hi - pts).min(axis=1)
            worst = min(worst, float(np.where(outside > 0, outside, -np.maximum(inside, 0.0)).min()))
    return worst


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

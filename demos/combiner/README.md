# Combiner-box scene

```bash
./demos/combiner/run_combiner_demo.sh                             # no arguments: --turn --handle_torque_nm 0.3
./demos/combiner/run_combiner_demo.sh --seed 7                    # the scene alone; the arm rests
./demos/combiner/run_combiner_demo.sh --headless --episodes 5 --capture
./demos/combiner/run_combiner_demo.sh --sweep_deg 180
./demos/combiner/run_combiner_demo.sh --door_angle_deg 90 --capture
./demos/combiner/run_combiner_demo.sh --turn                      # find the tag, grip the lever, pull the door open
./demos/combiner/run_combiner_demo.sh --turn --method press       # only push the lever down (F-071's runs)
./demos/combiner/run_combiner_demo.sh --turn --headless --capture --no_console --ui_feed_port 0 \
    --handle_torque_nm 0.2 0.3 0.4 0.5                            # torque sweep, one attempt per spring
```

With **no arguments** the launcher runs the tag-guided grip-and-pull in the viewer against a
0.3 N·m spring (`--turn --handle_torque_nm 0.3`): on the earlier proxy box the grip turned
0.4 N·m in simulation and lost the bar at 0.5 (F-073), so not the 0.8 N·m the push managed. Each attempt holds its end
pose until `R`, or `--attempt_s` (90 s) after it began, then the next placement starts. Any
argument at all, even just `--seed 42` or `--no_console`, hands the choice back and nothing is
added. The reach console's camera window outlines the door's tag in magenta, with its id and
range (`d1_ui/server.py --apriltag`, which the launcher passes).

Uses the same `env_isaaclab` environment and welded Go2+D1 as the cup demo, with the same
measured D1 trajectory model and force drives. **The dog stands** (since 2026-10-19): the
position-only task's stance, its legs held at the robot config's default pose under zero
action, spawned at 0.30 m; the base settles 0.274 m up after sliding about 6 cm back. The
cup pick's folded-leg pose put the arm's mount 0.17 m up, and the real box's handle 0.46 m
above it, which the arm reached at 2 of 12 default placements on the CPU model. No walking
policy or hardware link is needed.

The box is **Lukas's real combiner** (photos of 2026-10-19), a Shoals "PV array d.c
isolator": a light-grey cabinet standing on the floor, **830 mm tall, 630 mm wide and
300 mm deep**, its door hinged on the right edge. Measured: those outside dimensions; the
black-and-red rotary isolator handle's centre **160 mm below the top**; the tag's top edge
**150 mm below that centre**; the quarter-turn cam lock midway up the **left edge** (the side
that swings open), its centre **64 mm in** from it. The tag is tag36h11 id 0 printed from
`dock_tag_id0_100mm_A4.pdf`: a **100 mm** black square cut out with its 12.5 mm white border,
125 mm in all (`apriltag.py`). Assumed: the handle and the tag on the door's centreline (the
photo puts both within a few centimetres of it), wall and door thickness, the interior
(mounting plate, two rails of fuse-holder proxies), the decals' places, and every mass. The
cam lock is drawn, not simulated. The housing is hollow; the door and handle have colliders
and are separate rigid links. The lever starts horizontal, points from its spindle toward the
hinge as the isolator's bar does, and turns downward about the spindle through 0–60°. Its
shape (18 mm bar, 65 mm proud, 105 mm long) is the earlier proxy's, not the isolator's, so the
grasp results stay comparable; the photo suggests a longer, flatter grip. Its return spring brings it back when released. Turning it at least **45°
releases the latch** (Lukas, 2026-09-19), allowing the door to swing outward through 0–110°.

The return spring is sized by the **torque it needs at 45°**, `--handle_torque_nm`
(default **0.4 N·m**, lowered from the first version's 0.94 so the arm can turn it).
Its rest angle is −15°, below the stop, so the lever rests preloaded against the
stop with a quarter of that torque, and needs 1.25× it at the 60° stop. The lever's
own weight helps a downward push by 0.029 N·m at horizontal. All assumed.

The latch is a task approximation: `latch.py` constrains the door hinge to 0–0.5°
of play until the **measured** handle angle reaches 45°. It checks every physics
step (200 Hz), independently of keyboard input, so robot contact can also release
it. Once open, releasing the handle leaves the door free. Closing it within 0.5°
with the lever returned below 10° re-engages the latch. The separate release/return
thresholds prevent chatter. A sliding bolt and internal cam are not simulated;
the standalone USD needs this runtime controller for the latch behavior.

Focus the Isaac viewport to use these keys:

| Key | Action |
| --- | --- |
| R | Reset the dog, reset the door, and sample a new box position and yaw |
| H | Toggle holding the handle down at 50° (past the release) / releasing it to its return spring |
| O | Pull the door toward 90° for inspection; refused while latched |
| C | Pull the door closed for inspection |
| F | Release both inspection drives, leaving the return spring and latch active |

To inspect: press **H**, wait for the lever to turn down, then **O**. Once open,
press **H** again to release the handle, then **C** to close and re-latch. O/C
apply a bounded pull torque (maximum 0.6 Nm); they do not teleport the door or
bypass the latch. These are inspection controls, not robot opening actions. The
door is otherwise unpowered, with slight hinge damping. Without `--turn` the arm
holds its zero pose.

## Working the handle (`--turn`)

The plan's B path with the IK reference in place of the learned controller: the
wrist RealSense finds the door's AprilTag, the tag gives the box and so the handle,
and a programmed sequence works it (`sequence.py`; planned in `pull.py` or
`press.py`; run in the simulator by `turn_run.py`):

1. **Search.** The camera looks from stops at 0, −20, −40, −60, +20, +40, +60° of
   heading, the camera about 0.37 m above the arm's mount looking down at a point 0.10 m
   above it (the tag's height, standing), its view turned 15° right and 8° down so the tag
   sits up and to the left, clear of the shut fingers that fill the image's lower right
   (`press.FINGERS_PX`), until three frames show the tag. A stop whose first three frames
   show nothing is left at once.
2. **Close look** at the tag from 0.30 m out and 0.16 m above it, aimed 10° right of and 5°
   below it for the same reason (centred, the fingers hid a corner and the estimate came out
   37 mm off, 2026-10-19); three more frames give the box
   pose the rest is planned on. Before a grip the camera is turned about its view, in
   quarter turns, to face the way the grasp will hold the wrist, which saves the wrist up to
   2 rad on the way in.
3. **`--method pull` (default): grip, turn, pull.** The jaws open and close across
   the lever 90 mm from the spindle, shut to the real jaws' 2 mm past the URDF's stop
   (F-063). Before closing, the arm takes out what its joints fall short of the plan,
   read from joint feedback (F-074). It turns the lever to a commanded 52°, cracks the
   door 10° with the lever down, lets the lever back up while holding it, pulls the
   door to `--door_deg` (60) or as far as the arm reaches with 15% of each joint's
   torque to spare, then opens the jaws and backs off. The planner tries 5 approach
   pitches × 2 wrist rolls. Of those whose static turning ceiling clears the handle's
   torque, a grasp the jaws have been seen to hold (`PROVEN_GRASPS`) goes first, then
   the one that opens the door furthest. If the arm cannot get onto the lever, it
   backs off and takes the next grasp (up to 3). `--grasp PITCH ROLL` forces one.
4. **`--method press`: push only.** The gripper comes in level, jaws shut, lays its
   fingers across the top of the lever 80 mm out, follows its arc to 52°, 4 mm inside
   the lever, holds 2 s and withdraws. The door stays shut.

Every arc runs on a clock, not on arrival: against a strong spring the arm falls
short, and how far short is the measurement. Each arc goes as fast as its joints
allow, at most 0.6 rad/s per joint, up to 45°/s of lever and 30°/s of door. Where
the wrist has to swing, the lever turns slowly; where the arm barely moves, it
turns quickly. The jaws open 41 mm onto the 18 mm bar, not the pick's 77 mm. The sequence never sees the lever's
angle, the latch or the door. The runner reads them from the simulator, along with
PhysX's joint torques, finger positions and contact forces, and how far the jaws
have slid from the point they gripped.

**On the earlier proxy box** (360 × 400 × 160 mm, lever 0.30 m up, hinge on the left) with the
dog lying, the grip opened the door against springs up to 0.4 N·m and lost the bar at 0.5
(F-073). Only two grasps held, both wrist roll −1: pitch 40° and 50°. At 0.3 N·m every attempt
with one of them opened the door, 17 of 17 at 17 placements, and every other grasp lost the bar
(F-075). A whole attempt took 18–25 s from its start to letting go (F-076). The push turned up to
1.15 N·m but opened nothing (F-071). None of that has been re-measured on the real box.
**Standing at the real box**, on the CPU model at 30 seeded default placements, the search
sees the whole tag, the close look solves, and a grip-and-pull plans at all 30, opening the
door 20–40° in the plan with static turning ceilings of 0.82–0.96 N·m; the push plans at 28.
The level grasp (pitch 0, roll +1, no record) solves at all 30 and goes first at 22; pitch 40
roll −1 solves at only 6, pitch 50 at none (2026-10-19). The
grip's weak direction is the URDF's two independent finger drives, where the real
jaws are one servo. Coupling them with a PhysX mimic joint broke the articulation
in this Isaac Lab, so the fingers stay as the URDF has them.

Detection is OpenCV's AprilTag 36h11 dictionary and `solvePnP` (IPPE square) through
the intrinsics the renderer used (F-070); it is not the AprilTag library `apriltag_ros`
uses on the robot. `--mount_calibration sim` (default) locates the tag with the camera
pose the renderer used, a perfect hand-eye calibration, so a torque test is not
also a perception test; `none` uses the requested mount (F-052's ~11 mm offset).

Several `--handle_torque_nm` values try each placement once per spring, in order.
Each attempt writes `attempt_NNN/` with:

- `trace.csv`: commanded lever and door angles and the simulator's; the latch; the arm's
  commands, feedback, drive targets and true angles; PhysX joint torques; the finger
  positions, targets and forces; the grip slip; the largest arm-link contact and which
  link; the spring torque;
- annotated look frames;
- with `--capture`, overview and wrist images halfway through each hold.

Its row in `run.json` has:

- the box-pose error at each look;
- the grasp chosen and the static torque ceiling it predicted;
- the grasp offset split into perception and arm error;
- the peak and held lever angle and the torque held against;
- whether the latch released;
- for the pull: the door's peak, held and final angles, and the grip's slip and shift.

Results so far are F-071 to F-076, all in simulation and all on the earlier proxy box with the dog lying.

Placement is randomized on the first episode and every reset, reproducibly with
`--seed`. Defaults are an enclosure centre 0.62–0.70 m from the dog's spawn,
within ±45 degrees of forward, facing the dog with ±10 degrees of yaw variation.
Use `--box_range MIN MAX`, `--sweep_deg`, and `--yaw_jitter_deg` to change these.
`--sweep_deg 180` covers the full circle. Height stays fixed. Defaults put the
handle in the standing arm's workspace (CPU numbers above); at 0.70–0.78 m the pull plans
at 22 of 30 and the push at 12. Door-swing clearance has not been validated: the door is
630 mm wide and swings toward the dog's right, and the planner models the dog's trunk, not
the door. Wider placements may require moving the dog.

The outline, the handle's height, the tag's place and the lock's place are **measured**
(above). The masses (8 kg enclosure, 0.6 kg door, 0.12 kg lever) and the handle's geometry are
**demo assumptions**. A 630 × 825 mm steel door weighs several kilograms, not 0.6; the door's
mass has not been changed with the outline and is unvalidated. The body stands 5 mm off the
floor so the door's lower edge clears the ground plane; its top is at the measured 830 mm.
The lever has an 18 mm diameter grip,
65 mm projection and 105 mm length. Its return spring is set by `--handle_torque_nm`
(see above), with 0.08 Nm·s/rad damping and a drive limit 20% above the most the
spring can ask for. No product CAD is needed.

Each run writes `logs/combiner_demo/<stamp>_combiner_seedN/run.json`, `env.yaml`
and `assets/combiner_box.usda` (built with the first `--handle_torque_nm`). `run.json` includes sampled poses, handle positions,
door/handle angles, latch transitions, inspection commands, and final robot positions. `--capture` adds an
overview PNG and a wrist-camera PNG (`wrist_NNN.png`) per placement after up to two seconds. `--headless` finishes after
one five-second placement by default; `--episodes N --episode_s S` runs a batch.
The viewer stays open for manual resets unless `--episodes` is supplied.
`--robot_usd PATH` reuses an existing welded robot asset.
`--door_angle_deg` and `--handle_angle_deg` set explicit initial inspection poses;
an initially open door starts unlatched, and the lever spring still returns it to its stop.

## Wrist RealSense and the reach console

The wrist carries the cup pick's RealSense, set up the same way (`wrist_camera.py`, and
`wrist_camera_cfg`/`camera_body_cfg` from `demos/cup/pick_demo/scene.py`): the D435
preset's colour intrinsics (datasheet-derived, not a calibration) at the saved wrist mount
(`demos/cup/pick_demo/assets/mounts/wrist_mount.json`: since 2026-10-09 the bracket's CAD,
30 deg down from the gripper, `demos/cup/pick_demo/bracket.py`; from CAD, not measured). It renders
RGB and ideal depth with the 20 mm near clip, and Intel's D435 case is drawn at the mount and the
bracket on Link6 (visual only). The tag search's stops look 32 deg down because of that pitch
(`press.SearchParams`). The flags match the pick's: `--camera`, `--calibration FILE`,
`--mount FILE|none`, `--no_camera_body`. `run.json` `camera` records the model, the mount
and its provenance, and the model's intrinsics beside the rendered ones (F-070).

The launcher also brings up the reach console, as `run_pick_demo.sh` does, through the shared
`demos/cup/d1_ui/beside_sim.sh`. It runs `demos/cup/d1_ui/server.py --mode sim` on
http://localhost:8090, following this run's feed on localhost:8765: the arm, the legs and
the wrist camera. A viewer run opens the tab once the simulator publishes; `--headless`
does not. The console stops when the launcher exits. It only watches: commands are refused
in sim mode. There is no cup here, so its YOLO is off (`--detect none`), which also keeps
it off the shared GPU. `D1_UI_ARGS='--detect all'` turns it back on. Use `--no_console` for
the simulator alone and `--ui_feed_port 0` to stop publishing.

**At rest the wrist camera sees part of the door but not the whole tag.** Standing, with the
arm at its zero pose, the camera is 0.85 m up and looks 30° down (the bracket's pitch).
Projecting the door face for 2,000 default placements (CPU model, level base) puts some of it in
the image at all 2,000 and the whole printed tag at none, so `--turn` still searches. Not
checked in a live frame. (Lying, at the earlier proxy, it saw none of the box.)

The asset root is at floor level, +X out of the door, +Y toward the hinge (the right,
looking at the door), +Z up. `DoorHinge` opens outward about enclosure +Z; `HandleJoint`
lowers the lever about the door's −X spindle (its joint frames are turned half a turn about
Z). `Handle/HandleGrasp` follows both joints, 70 mm along the lever from the spindle.
`geometry.BoxGeometry.hinge_side` mirrors the whole door (−1 is the earlier proxy's left hinge). The enclosure, door and lever are three rigid bodies.
Joint frames and the fixed mount use [USD Physics conventions](https://openusd.org/dev/api/usd_physics_page_front.html).

CPU placement/CLI tests: `python -m unittest discover -s demos/combiner/tests -v`.
`test_turn.py` checks the tag pattern against OpenCV's dictionary, pose recovery from
rendered views, the push plan against the lever's arc, and runs the whole sequence
against rendered tags (numpy and OpenCV only).
With `pxr`, this also checks the USD's joints, colliders, moving grasp frame and the measured
layout. `env_isaaclab` has no importable `pxr` of its own; Isaac Sim's bundled one works with
`L=$(ls -d $CONDA_PREFIX/lib/python3.11/site-packages/isaacsim/extscache/omni.usd.libs-*)`,
`PYTHONPATH=$L LD_LIBRARY_PATH=$L/bin:$CONDA_PREFIX/lib`.
With `torch` (the Isaac environment), it also tests the controller's joint-limit
writes and inspection commands against an articulation test double, including
reversed joint ordering. Live PhysX behavior and rendered appearance still need
a GPU smoke run and an H → O → H → C inspection cycle.

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
0.3 N·m spring (`--turn --handle_torque_nm 0.3`): the grip turned 0.4 N·m in simulation and
lost the bar at 0.5 (F-073), so not the 0.8 N·m the push managed. Each attempt holds its end
pose until `R`, or `--attempt_s` (90 s) after it began, then the next placement starts. Any
argument at all, even just `--seed 42` or `--no_console`, hands the choice back and nothing is
added. The reach console's camera window outlines the door's tag in magenta, with its id and
range (`d1_ui/server.py --apriltag`, which the launcher passes).

Uses the same `env_isaaclab` environment and welded Go2+D1 as the cup demo.
The dog holds the **same folded-leg resting pose**, with the same measured D1
trajectory model and force drives. No walking policy or hardware link is needed.

The procedural enclosure follows the supplied photo: light-grey cabinet, yellow
electrical warning, six black cable glands underneath, and a silver door lever
in place of the right-hand keyhole. A short stand supports the cabinet. A 60 mm
tag36h11 AprilTag (id 0) sits on the door above the handle (`apriltag.py`).
Inside are simple mounting rails and fuse-block proxies. The housing is hollow;
the door and handle have colliders and are separate rigid links. The lever starts
horizontal, extends toward the hinge, and turns downward about its spindle through
0–60°. Its return spring brings it back when released. Turning it at least **45°
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
stays at rest.

## Working the handle (`--turn`)

The plan's B path with the IK reference in place of the learned controller: the
wrist RealSense finds the door's AprilTag, the tag gives the box and so the handle,
and a programmed sequence works it (`sequence.py`; planned in `pull.py` or
`press.py`; run in the simulator by `turn_run.py`):

1. **Search.** The camera looks from stops at 0, −20, −40, −60, +20, +40, +60° of
   heading, about 0.24 m above the arm's mount, until three frames show the tag. A
   stop whose first three frames show nothing is left at once.
2. **Close look** straight at the tag from 0.25 m; three more frames give the box
   pose the rest is planned on. Before a grip the camera is turned about its view
   to face the way the grasp will hold the wrist, which saves Joint6 up to about a
   radian on the way in.
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

In simulation the grip opens the door against springs up to 0.4 N·m and loses the
bar at 0.5 (F-073). Only two grasps have held, both wrist roll −1: pitch 40° and 50°. At
0.3 N·m every attempt with one of them opened the door, 17 of 17 at 17 placements, and
every other grasp lost the bar (F-075). On the CPU model about 6% of default placements
reach neither grasp, and there the planner falls back to one with no record. A whole attempt takes 18–25 s from its start
to letting go, where it took 29–49 s. The rest is mostly the arm's own speed (F-076). The push turns up to 1.15 N·m but opens nothing (F-071). The
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

Results so far are F-071 to F-076, all in simulation.

Placement is randomized on the first episode and every reset, reproducibly with
`--seed`. Defaults are an enclosure centre 0.62–0.70 m from the dog's spawn,
within ±45 degrees of forward, facing the dog with ±10 degrees of yaw variation.
Use `--box_range MIN MAX`, `--sweep_deg`, and `--yaw_jitter_deg` to change these.
`--sweep_deg 180` covers the full circle. Height stays fixed. Defaults put the
handle in the arm's workspace: on the CPU model the `--turn` search, close look
and push all plan at 300 of 300 seeded default placements, and some grip-and-pull
grasp at 30 of 30. Door-swing clearance
has not been validated. Wider placements may require moving the dog.

Dimensions (360 × 400 × 160 mm cabinet), masses (8 kg enclosure, 0.6 kg door,
0.12 kg lever),
mounting height (100 mm), and handle geometry are **demo assumptions**, not
measurements of the photographed product. The lever has an 18 mm diameter grip,
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
(`demos/cup/pick_demo/assets/mounts/wrist_mount.json`, aligned by eye, not measured). It renders
RGB and ideal depth with the 20 mm near clip, and Intel's D435 case is drawn at the mount
(visual only). The flags match the pick's: `--camera`, `--calibration FILE`,
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

**At rest the wrist camera does not see the box.** With the arm at its zero pose it looks
level and straight ahead from about 0.73 m up. The cabinet's top is at 0.50 m and the
colour camera's vertical half-angle is 21.3°. Projecting the whole cabinet for 2,000
default placements puts none of it in the image; at best it reaches 54 px below the bottom
edge (level base assumed). One live frame shows only the floor grid and background. The
camera shows the box only once the arm moves to a pose that looks down at it, and nothing
here does that yet.

The asset root is at floor level, +X out of the door, +Y toward the handle, +Z up.
`DoorHinge` opens outward (joint +Z is enclosure -Z); `HandleJoint` turns about
the door's +X spindle. `Handle/HandleGrasp` follows both joints, 70 mm along the
lever from the spindle. The enclosure, door and lever are three rigid bodies.
Joint frames and the fixed mount use [USD Physics conventions](https://openusd.org/dev/api/usd_physics_page_front.html).

CPU placement/CLI tests: `python -m unittest discover -s demos/combiner/tests -v`.
`test_turn.py` checks the tag pattern against OpenCV's dictionary, pose recovery from
rendered views, the push plan against the lever's arc, and runs the whole sequence
against rendered tags (numpy and OpenCV only).
With `pxr`, this also checks the USD's joints, colliders and moving grasp frame.
With `torch` (the Isaac environment), it also tests the controller's joint-limit
writes and inspection commands against an articulation test double, including
reversed joint ordering. Live PhysX behavior and rendered appearance still need
a GPU smoke run and an H → O → H → C inspection cycle.

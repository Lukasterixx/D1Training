# The combiner door: found by the front camera, swung open by walking, checked by the lidar

The combiner box's door pull without UniFP, with the robot finding the box itself: the Go2's own front camera finds the
AprilTag (since 2026-10-22; the head RealSense before, below), the MID-360 gives the door's heading, the dog walks in, grips the handle above the tag and walks the door
open round its hinge, and the lidar reads the door's angle all the way.

On the real dog, the same sequence is `robot/` (`robot/README.md`: `d1dog` on the dog, `d1view` for RViz here).

```bash
./demos/combiner_door/run_combiner_door.sh                            # watch one attempt: sets up the environment itself
./demos/combiner_door/run_combiner_door.sh --attempts 16 --headless    # measure

# or, in an environment already set up:
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

./demos/combiner_door/run_combiner_door.py                            # watch one attempt (one scene, no markers)
./demos/combiner_door/run_combiner_door.py --attempts 16 --headless    # measure: the door pull's 16 placements
python -m unittest demos.combiner_door.tests.test_combiner_door        # the geometry, without a simulator
```

It is the door pull's runner (`demos/unifp/run_demo.py`) with `--task door --controller decoupled --perception head`, so
the box, its placements, the door script, the walk round the hinge and the record are the UniFP door pull's and its
decoupled baseline's (`demos/unifp/README.md`). Then `./dashboard.py record <out dir> --title "..."`.

## What happens in an attempt

| phase | what | sensors |
| --- | --- | --- |
| `spawn` (4 s) | the robot lands and stands, the arm folded in the real D1's rest pose | |
| `look` (1 s) | the attempt is anchored where the dog now stands; it looks, still folded | camera: the tag; lidar: frames kept |
| `unfold` (3.5 s) | the arm unfolds in joint space to the home point, high and forward (none if no tag was found) | |
| `walk_in` | straight ahead at a steady speed until the odometry has it at the stance, 0.18 m on | |
| `walk_settle` (1.5 s) | stands | lidar: door read from here on |
| the door script | stand, approach, settle, enter, arrive, grip, pull (walk the door to 90°), hold, release, back | lidar: door read at 10 Hz |

- **Why it starts back.** From the door pull's stance the tag runs off the top of the front camera's image: its field is
  ~100 x 53 degrees, and P2Dingo's walking policy stands the base 0.31 m up. From 0.40 m back
  (`perception.LOOK_BACK_M`) it is whole at all 200 placements tried, the base 0.30-0.33 m up and pitched +-2 deg. (The
  head RealSense needed 0.18 m: 0.22 m from the door at the stance it cut the tag off at a quarter of them.)
- **Why it starts folded.** The real arm rests folded against its mechanical stop
  (`d1_hardware.REST_SERVO_DEG`, J1 and J2 1-3 deg inside the URDF's limits here). It unfolds only once the look has
  found the box, standing, in a straight line in joint space, each 10 Hz command at most 4 deg from where the arm is
  (as `d1_hardware`'s approach steps the real arm). Handed a Cartesian goal from folded, UniFP turned the hand down
  onto the head (F-149); this path stays clear of the body proxy all the way.
- **Why it is anchored after landing.** The simulator drops the robot from above its standing height, and P2Dingo's
  walking policy turns ~15° while it lands. The real robot starts standing, so the attempt starts from where it stands
  (`env._anchor`): the script's frame and the box are placed from there, as the door pull places them from its spawn.

## The box: the tag for where, the lidar for which way (P2Dingo's docking)

As P2Dingo's docking makes its fix (`dock_detector_node.cpp`, `dock_wall.hpp`): a 100 mm tag's heading shows only as
the difference between its left and right edges in the image, so it is its weakest number, and on the robot was ~10° off
at a metre; its centre is good to a couple of centimetres. So:

1. **Camera** (`head_camera.py`, `perception.py`). The measured D435i at 640x480 on P2Dingo's head mount
   (`base_link` (0.3392, 0.03215, 0.0927), straight ahead, from the bracket's CAD) finds the tag in every frame of the
   look (OpenCV's 36h11 decoder, IPPE). Its centre, through the mount and the odometry, averaged: the **position**.
2. **Lidar** (`mid360.py`). The MID-360's frames over the same look, added up, and the door's face fitted as a line
   round the tag's centre: P2Dingo's wall fit ported (a window along the face, a RANSAC line within 25° of the tag's own
   heading, total least squares twice, refitted along the line found): the **heading**. The tag's heading only seeds the
   window; where no face is found it stands, and the record says so.
3. The box placed off the tag's centre at that heading, level, on its known plinth.

The record keeps the tag-only estimate beside the one used (`look.tag_only` in `summary.json`).

## The door: read by the lidar while it opens

Every 10 Hz lidar frame from the walk's end on, the same face fit is laid where the tag would be at the last reading's
angle (and across the door's travel if the face is not there), then once more where the tag is at the angle found; the
face's heading against the shut door's is the door's angle (`mid360.door_angle`). Nothing steers on it: it is the check
the plan's "verify" step asks for. `trace.csv` has `lidar_door_deg` beside the simulator's `door_deg` and the script's
`commanded_door_deg`; `summary.json` has each attempt's error and `verified_open` (the lidar's reading at the end of the
hold at 85° or more) beside `success` (the simulator's).

## The robot

- **Legs:** P2Dingo's clocked-gait walking policy (`--walker p2dingo_gait`, `checkpoints/p2dingo_go2_gait_v7_model_7000.pt`),
  trained on a bare Go2. The 2024 policy the door pull's baseline uses (`--walker rescue_2024`) cannot be stopped from
  a walk without turning in place and leaning up to 26°, and falls; P2Dingo's stops level.
- **Arm:** inverse kinematics on the jaw centre through the measured D1 model, bounded to run at 10 Hz (F-152).
- **Front camera** (`front_camera.py`): the Go2's own, at the URDF's mount, a 1280x720 fisheye (`CALIBRATED`: OpenCV's
  fisheye model on an A3 ChArUco board, 0.22 px, F-206; `calibrate_front_camera.py`). Isaac renders a wide pinhole (1714x966) that `distort_render`
  remaps through the lens model, so the detector sees what the dog streams. **The USD camera needs a real focal
  length** (`USD_FOCAL_LENGTH`): with Isaac Lab's default (aperture 1.0, focal 0.35 here) Isaac reports the intrinsics
  asked for and draws the scene ~1.35x too large, the box 17 cm too near. P2Dingo's head bracket is drawn; the RealSense
  it held is on the wrist since 2026-10-21 and is not drawn there.

## What it does and does not show

**With the front camera (2026-10-22, seed 1; F-185):** 16 of 16 doors opened, the box placed 6.1 mm off (median, 7.8
max), no falls. The head RealSense's results below are F-155's.

**Results (2026-10-20, 16 environments, seeds 1 and 2; F-155):** the arm starting folded, the lever as measured (a 140
x 40 x 17 mm slot, 27 mm off the door).

| same 16 placements | doors open | falls |
| --- | --- | --- |
| **this demo** | **16/16, 16/16** | 0 |
| the pose given, the same stack (`run_demo.py --task door --controller decoupled`) | 16/16, 16/16 | 0 |
| this demo before the measured lever, arm starting at home (F-154) | 15/16, 16/16 | 0 |
| UniFP + wrist servo, before the measured lever (F-146) | 8/16, 7/16 | 0 |

- **Look:** the tag was found 32/32. The box was placed a median 2.2-2.3 mm (max 3.0) and 0.12 deg (max 0.64) off.
- **Door:** read on every frame of the grip, pull and hold, with a median error of 0.33-0.39 deg (p90 at most 2.1 deg).
- **Verdict:** the lidar's open-or-not at the end of the hold agreed with the simulator 32/32.
- **The grip:** the jaws squeeze the lever's flat top and bottom at 14 N through the pull (about 10 N on the earlier
  round bar).

- **Simulation only.** No perception noise beyond the lidar's range noise (3 cm, the datasheet's); the camera renders an
  ideal picture of a printed tag; the lidar is a 1° grid ray caster, not the MID-360's pattern, and has no motion
  distortion. The odometry is the simulator's.
- **The placements are the door pull's**, relative to the stance: the handle 0.477-0.517 m off (the box 0.6705-0.7105 m
  with the lever measured 27 mm off the door; 0.70-0.74 m before), square to ±8°. The demo does
  not search for a box out of view, or walk to one from afar.
- Not on the robot: P2Dingo's mount is its CAD's, the D435i's intrinsics the bench camera's, and neither the walking
  policy nor the lidar fit has carried the arm on hardware.

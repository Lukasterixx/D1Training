# The combiner door on the real dog

The simulated combiner door demo (`../README.md`) as a program on the Go2's Jetson. It runs the same sequence on the same
code (`plan.py` imports the sim's perception, door script, IK and lidar fit):

1. look with the head RealSense (the tag) and the MID-360 (the door's face);
2. unfold the arm from its rest pose;
3. walk in to the stance;
4. refit the door with the lidar;
5. reach, grip, pull the door open while walking round the hinge;
6. let go: open the jaws, back the hand straight off the lever (the ungrasp), go home, fold.

RViz on this PC shows all of it through the dog's P2Dingo zenoh link. **Nothing moves without `--live`**, and a live
run waits for one Enter, once it has shown its plan, before anything moves.

```bash
# on this PC (the helpers are in ~/.bashrc; they find the dog as sshuni does)
d1update                          # copy this working tree to the dog's ~/D1Training (it is a clone of this repo)
d1view                            # RViz here, over the zenoh link (P2Dingo's operator stack)
d1dog                             # watch: where the box is and whether the dog stands right for it; nothing moves
d1dog --stage pull                # the whole attempt as a DRY RUN: the ghost robot does it in RViz, the dog stands still
d1dogdoor                         # the same, the door pull to 60 deg (`d1dog --stage pull --door_deg 60`); --live to run it
d1dog --stage unfold --live       # LIVE: unfold the arm, pause, fold it again
d1fetch                           # copy the dog's records (logs/combiner_door_robot/) back here
```

`d1dog` is `run_on_dog.sh` over SSH. It starts the arm link (`arm_link.py`, in P2Dingo's arm image) and the robot
program (`node.py`, in P2Dingo's ROS image) beside P2Dingo's containers. On the way out it sends StopMove and stops
the arm link, which leaves the arm holding where it is. `d1dog --help` lists the options (`--stage`, `--live`, `--yes`,
`--speed_scale` (default 1.0, the sim's pace), `--door_deg` (default 60), `--odom sport|tf`, `--no_fold`,
`--ignore_range`).

## Stages: try it one step at a time

| `--stage` | what happens (each ends by letting go and folding) | moves |
| --- | --- | --- |
| `watch` (default) | the look, over and over: the box, the stance, IN or OUT OF RANGE and what to move | nothing |
| `look` | one 2 s look and the plan from it | nothing |
| `unfold` | look, unfold the arm to the home point, pause 1 s, fold (needs no box) | arm |
| `walk` | look, unfold, walk in to the stance, settle, the lidar refit, fold | arm, legs |
| `reach` | ... reach to the standoff, settle, enter, arrive at the lever, ungrasp, home, fold | arm, legs |
| `grip` | ... close on the lever, squeeze 1 s, release, ungrasp, home, fold | arm, legs |
| `pull` | the whole door script: grip, walk the door open `--door_deg`, hold, release, ungrasp, home, fold | arm, legs, door |

**Letting go** is the same at the end of every stage that reaches the lever, in a recovery (`r`) and in `--release`:

- **release:** the jaws open where the hand is. The hand is held at the pose the arm *measures* itself at as it begins,
  not at the IK's goal: after the pull the door's load and the gait's carry can have the hand off its goal, and the
  lever is where the jaws are. The hand does not move until servo 6 reports the jaws open (within 4 units of the
  command, or stopped more than half way open). If they have not opened 2 s after the release's clock, it stops with
  the hand where it is.
- **ungrasp:** the hand backs straight off the lever along its own approach, by the 12 cm standoff, at the enter's pace,
  its orientation held. The fingertips stand 20 mm past the jaw centre, in the 27 mm gap behind the lever and beside its
  63 mm hub; going straight out, they clear the lever's front after 29 mm and never move towards the hub. If the IK
  cannot hold the hand's orientation, the hand stops rather than turn its wrist.
- **home**, from 12 cm out, then the fold.

Before (2026-10-20, live), the door script's release went straight from the lever to the home point. That crossed in
front of the hub with the fingers still behind the lever, and they pushed the door on the way out. On the CPU model the
same path runs the fingers 8 mm into the lever and to the hub's rim; the ungrasp keeps them 11.5 mm off the lever and
38 mm off the hub (`test_letting_go_keeps_the_fingers_off_the_lever_and_its_hub`).

**What waits for what.** A phase that only waits for the arm (the stand, the settle, the arrival, the unfold, the fold)
ends as soon as every joint is within 1 deg of its goal (the fold: 0.5 deg) on a fresh report, after at least 0.3 s,
and otherwise runs its clock as before. The grip waits for the jaws to stop closing (up to 2 s), the walk in for its last
step, the release for the jaws. On the CPU model with the arm link's profile and a model of the servo, a pull from Enter
to the start of the pull takes about 12.6 s (the look excluded); the live pull at the old clocks took 27 s.

Each stage and `--live` combination: run it dry first and watch the ghost in RViz.

## Before a live run

- **The box, not the dock, in view.** The box's tag is 36h11 id 0, 100 mm: the same tag as P2Dingo's charging dock.
  The look refuses a tag whose lidar face carries on past where the box's door ends (a wall behind a stand), but keep
  the dock out of view. RViz's stance turns green and its text reads IN RANGE when the dog stands right.
- **Standing.** Stand the dog up (the website or the remote); the program refuses to walk a dog that is not standing.
- **Nothing else commanding.**
  - **Legs:** P2Dingo's nav stack must not be running: its `cmdvel_to_unitree` streams Move(0, 0, 0) at 20 Hz and
    would fight the walk.
  - **Joysticks:** the website's must be off (the program aborts if they switch on).
  - **Arm:** P2Dingo's arm bridge must not be streaming; the arm link refuses to start if it is. Press E-STOP in the
    website's arm panel, or `docker stop p2dingo-arm` on the dog.
- **Clear space** round the arm (it unfolds up and forward) and round the dog's path (it walks in up to 0.45 m; the
  pull walks it sideways round the hinge).
- **The Unitree remote in hand.**

**At the keyboard during a live run.** It looks, prints the plan, and waits for one Enter; then it runs through to
the end and exits by itself (`--step` makes it wait before each step that moves something; `--yes` not at all).

- **Enter** starts the attempt.
- **a** + Enter stops at once: the base stands and the arm holds where it is, grip included.
- **r** + Enter, after a stop: recovers. If the hand may be at the lever it opens the jaws and backs out first; it
  always ends by folding the arm.
- **q** + Enter or Ctrl-C quits with the arm where it is.

**If you quit with the arm still holding the lever:** `d1dog --release --live`. It does no look and no attempt: it opens
the jaws where they are, backs the hand straight out along its own approach (orientation held), then goes home and folds.
`d1dog --release` (without `--live`) shows it first as the ghost.

## Where it differs from the simulation

- **The legs are Unitree's sport mode** (`/api/sport/request`), not the clocked-gait policy the sim walks with. That
  policy has only ever run in Isaac Sim; P2Dingo walks the real dog through sport mode.
- **Sport mode's dead zones.** P2Dingo measured sport mode on this dog: still at 0.12-0.15 m/s forward, no turn on the
  spot at 0.30-0.42 rad/s, no side-step at 0.25 m/s, and 75 mm of carry after a stop from 0.21 m/s. So every command
  is either zero or lifted to those floors (`plan.sport_command`), and the walk in stops early by the carry.
  - The sim's pull walks the base at 0.12 m/s sideways and 0.22 rad/s. The real one walks it in lifted bursts.
- **The odometry is sport mode's own** (`/sportmodestate`: leg and IMU), or GLIM's with `--odom tf` when P2Dingo's
  stack runs. After the walk the lidar fits the door again from the stance and moves the box along its normal (at most
  6 cm and 6 deg, or the attempt stops). That takes the walk's odometry error out of where the arm reaches.
- **The arm starts and ends folded** in its rest pose. It unfolds in a straight line in joint space, at most 4 deg a
  10 Hz command from where it last reported itself (as `d1_hardware`'s approach steps it).
- **The sim's pace** (`--speed_scale` 1.0; the first live runs were at 1.5): a 2 s approach to the standoff, a 2.5 s
  enter, a 0.8 s grip, but each dwell only as long as the arm takes to get there (above).
- **A smaller pull:** 60 deg by default (`--door_deg`; the first live pulls were 30), against the sim's 90. The dog
  walks about 0.7 m round the hinge, ending about 0.7 m to the side and turned 60 deg: clear that space. Through the
  hold the door follows how far the dog has actually turned (up to 10 deg past the pull, as far as the gait carries
  on), so the hand stays still relative to the dog.
- **The stance is set from the box.** Stand the dog square to the box (within 8 deg), the tag within 12 deg of
  straight ahead, and either:
  - 0.80-1.15 m from the box's root: it walks straight in until the root is 0.69 m away (the sim's placements);
  - or 0.65-0.76 m: it does not walk. The arm reaches the lever from anywhere in that band, but the sim only ran
    0.67-0.71 m, and the record says when it is outside.
- **The walk in is closed-loop, in classic walk.**
  - ClassicWalk goes out once when you press Enter.
  - The dog walks straight at 0.2 m/s with its heading held, and stops early by the carry it has seen.
  - It then stands 1 s, measures, and takes up to three corrective steps while more than 3 cm short.
  - The attempt then re-anchors where the dog actually stopped: the lidar refit places the box from there, and it
    goes on only if the box is 0.65-0.76 m off.
- **The pull is one continuous turn about the hinge.** The base and the door turn together, which in the base's frame is a
  constant twist. It is streamed at the gait's slowest usable speed for the whole pull (about 0.1 m/s back, 0.2 m/s
  sideways, 0.36-0.39 rad/s: 60 deg in about 3 s), and the script's door angle follows how far the dog has actually turned. The
  hand then stays on the door's arc, nearly still relative to the dog. The base stands for the hold and while the hand
  lets go.
- **The arm moves smoothly** (`arm_link.Profile`): a steady 10 Hz stream, 60 deg/s^2 up to 30 deg/s and back down into
  the goal, never more than 10 deg ahead of the arm. For a reach, the unfold already turns the hand to the grasp's
  orientation. When the IK misses the hand's pose for a step, the goal holds rather than switching wrists.
- **Joint moves wait for the arm.** The unfold and the fold go at 40 deg/s in joint space (the Cartesian phases at
  most 30), and end once the arm is there; past their clock they wait until it is within 2 deg (20 s at most).

## What stops an attempt

Any of these stops the base and holds the arm, then waits for `r` or `q`:

- the arm's feedback older than 1 s, or the odometry older than 0.3 s while walking or letting go;
- the base tilted past 12 deg;
- the website's joysticks switching on;
- the walk ending where the arm cannot reach the lever (the box, placed again from there, outside 0.65-0.76 m, 12 deg of bearing or 8 deg of square), or not finishing in 20 s;
- the lidar refit moving the box more than 6 cm or 6 deg;
- while holding the lever, a joint more than 10 deg from where the IK wants it for 0.6 s (the arm loaded), or no IK
  solution for 0.5 s;
- during the pull, the lidar's door angle more than 25 deg from the script's for 1 s;
- the jaws not reported open 2 s after the release's clock, or, while letting go, no IK solution for the hand's
  orientation for 0.5 s (the hand stays where it is, on or round the lever).

The arm link holds the arm if the program goes quiet for 0.5 s or disconnects, and it never releases the arm: release
is its only software stop, and it drops an extended arm (F-028).

## RViz (`d1view`)

- **Go2 + D1:** the robot as it is, the legs from `/lf/lowstate` and the arm from its feedback.
- **Plan (ghost):** what the program commands. In a dry run the whole attempt walks and reaches here.
- **Combiner door:**
  - the box where the look puts it, the door at the lidar's angle (live) or the script's (dry);
  - the measured slot lever and the tag;
  - the stance (green in range, red not) and the base's path round the hinge;
  - the grasp point and the hand's target;
  - the lidar's face fit and the camera's field;
  - the status in words.
- **Door face:** the MID-360's points in the fit's window, green on the face's line.
- **Head camera:** the RealSense frame with the tag outlined.

It is P2Dingo's `scripts/p2dingo-rviz.sh` with this directory's `combiner_door_robot.rviz`, this repo mounted into
the rviz container (`opc-compose.yaml`) and P2Dingo's static-TF helper. It finds the dog's zenoh listener on its WiFi
dongle (the address `sshuni` caches) before P2Dingo's own search.

**It needs two changes in the P2Dingo checkout (made 2026-10-20, uncommitted there):**

- the `/combiner_door/...` topics in both zenoh allow-lists (`zenoh/robot_bridge.json5`, `zenoh/opc_bridge.json5`);
  the dog's copy is deployed and its zenoh container restarted;
- an `OPC_COMPOSE_EXTRA` hook in `scripts/p2dingo-rviz.sh`, which lets this view add its compose override.

## Files

| file | what |
| --- | --- |
| `plan.py` | the attempt as numbers: the look's fix, the stance, the stages' phases, the arm's steps, the walk's shaping, the aborts. No ROS. |
| `node.py` | the ROS 2 node: sensors in, sport mode and the arm link out (live), RViz's topics, the record |
| `arm_link.py` | the D1 through `d1_hardware.D1Client` behind a local socket; `--observe` in dry runs |
| `viz.py` | the robot model's transforms from `description/go2_d1.urdf`, the markers, the clouds, the camera frame |
| `run_on_dog.sh`, `view_on_pc.sh` | the two launchers (`d1dog`, `d1view`) |
| `combiner_door_robot.rviz`, `opc-compose.yaml` | the RViz config and the rviz container's mount |
| `../tests/test_robot.py` | the look, the range check, the refit, a dry attempt through every stage, the shaping, the plumbing |

## What has been run, and what has not (2026-10-20)

**Run on the dog, dry:**

- **Watch:** the camera (1280x720, fx 907) and the lidar read; the arm link reporting the folded arm; RViz here over
  zenoh drawing all of the above.
- **A dry pull to 30 deg:**
  - every phase ran in order;
  - the IK held the hand's pose throughout, apart from transit steps that fell back to the position alone;
  - the arm ended commanded to the fold.

**The dock, not the box.** The dog was lying in front of P2Dingo's charging dock, so every look found the dock's tag.
The dry pull planned against it with `--ignore_range`.

**Not run:**

- anything live: no arm or leg command has been sent;
- the combiner box itself in view;
- sport-mode walking with the arm out;
- the gripper's units on the lever, which come from a linear map measured only at its ends (F-063);
- the pull's lifted base commands against a real door.

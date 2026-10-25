# The combiner door on the real dog

The simulated combiner door demo (`../README.md`) as a program on the Go2's Jetson. It runs the same sequence on the same
code (`plan.py` imports the sim's perception, door script, IK and lidar fit):

1. look with the Go2's own front camera (the tag) and the MID-360 (the door's face), the arm folded (below: "The look's
   camera"; `--camera wrist` is the RealSense on the D1's wrist, aimed by the arm first, as from 2026-10-21);
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

- **release:** the jaws open where the hand is. Across the door, the hand is held where the arm *measures* itself as
  it begins, not at the IK's goal: after the pull the door's load and the gait's carry can have the hand off its goal,
  and the lever is where the jaws are. In height and wrist, it keeps what it was commanded on the handle. The arm sags
  11-15 mm below its goal reaching out there, so a goal at the measured height pressed the lever down while the jaws
  still held it, and the hand dropped onto it once they opened. The jaws open slowly, from where they grip the lever
  to fully open over 1.5 s (`LET_GO_RAMP_S`), and the hand stays still with them open until 2 s (`LET_GO_S`). It does
  not move until servo 6 reports the jaws open (within 4 units of the command, or stopped more than half way open).
  If they have not opened 4 s after the release's clock, it stops with the hand where it is.
- **ungrasp:** the hand backs straight off the lever along its own approach, by the 12 cm standoff, at the enter's pace,
  its orientation held. The fingertips stand 20 mm past the jaw centre, in the 27 mm gap behind the lever and beside its
  63 mm hub; going straight out, they clear the lever's front after 29 mm and never move towards the hub. If the IK
  cannot hold the hand's orientation, the hand stops rather than turn its wrist.
- **home**, from 12 cm out, then the fold in two joint moves: **tuck** (the shoulder and wrist to the fold, the elbow
  still 30 deg open), then **fold** (the elbow shut). Straight to the fold the elbow got there first and the shoulder
  swung the shut forearm back onto the lidar (both live pulls of 2026-10-25; on the CPU model 29 mm into the cage's
  box). Through the tuck the arm stays 45 mm or more off it.

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

## The look's camera (2026-10-22)

The box's tag is found by the **Go2's front camera** (`--camera front`, the default; `../front_camera.py`). It is fixed
in the trunk, so the look needs no arm move and nothing between the tag and the base but the URDF's mount. The wrist
RealSense's look went through the bracket's CAD mount and the arm's joints and put the box 3-5 cm low (2026-10-21);
the insertion still uses the wrist RealSense, for its keyhole view only.

- **P2Dingo's `p2dingo-front-camera` must run** (`cd ~/P2Dingo && docker compose up -d front_camera`; it is in the
  boot autostart). `run_on_dog.sh` refuses without it. The node reads `/go2_front_camera/image_full/compressed` (1280x720
  JPEG, local to the dog). RViz's "Camera" panel shows the front camera's frame with the tag outlined; through the
  insertion's key phases, the wrist RealSense's.
- **The lens is calibrated** (`front_camera.CALIBRATED`, 2026-10-24, F-206): an A3 ChArUco board
  (`charuco_9x6_40mm_A3.pdf`) filmed through P2Dingo's web app, fitted by `calibrate_front_camera.py`. On the 0.61 m
  taped stand it reads 0.616 m; its pitch there (6.8° against the lidar's 4.7°) is not explained. The lidar's face
  fit and the refit correct the door's depth and heading as before, not its height.
- **Stand further back than before: the box's root at least 0.98 m ahead** (`plan.FRONT_LOOK_MIN_ROOT_M`; the door ~0.82 m
  from the base; 0.95 before the lens was calibrated). The field is only 25 degrees above the axis, and the tag leaves
  the top of it closer in. For the door pull that is 0.98-1.14 m (the walk in is at most 0.45 m); the old no-walk band (0.65-0.76 m) cannot be looked from. A watch that
  sees no tag says so.
- **The dock in view is no longer averaged in.** The front camera often sees the box's tag and P2Dingo's dock's (both
  36h11 id 0) at once. The look groups the detections by where they put the box and keeps the group whose lidar face is
  the box's door (`plan.make_fix`, `Fix.other_tags`).

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
the end and exits by itself (`--step` makes it wait before each step that moves something; `--yes` not at all). One
Enter covers the whole chain: the stages it goes on to (`--then_door`, `--then_close`, a walk to a new look) do not ask
again (`started`).

- **Enter** starts the attempt.
- **a** + Enter stops at once: the base stands and the arm holds where it is, grip included.
- **r** + Enter, after a stop: recovers. If the hand may be at the lever it opens the jaws and backs out first; it
  always ends by folding the arm.
- **q** + Enter or Ctrl-C quits with the arm where it is.

The remote does the same with two buttons, as in every live program here (`lowlevel.RemoteButtons`, read from
`/lf/lowstate`): **A** is Enter, and **r** + Enter once stopped; **B** is **a** + Enter. A counts only pressed on its own
(L1/L2 + A are sport mode's); B counts with anything held. Not yet tried on the dog: the remote's bits have been read
there on `/lowstate` (the foot press), not on `/lf/lowstate`.

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
- **The stance is set from the box.** Since 2026-10-25 it is 6 cm further from the door than the sim's and 10 cm
  toward the hinge (`plan.STANCE_*`): the handle about straight ahead, the box's root 0.75 m ahead and 0.10 m to the
  dog's left. Near the end of a pull the door swings on past the dog's turn (15-30 deg on the dog), and from the sim's
  stance it reached the dog's face after 22.5 deg; from this one, after 38 (30-46 across the tolerances below). Stand
  the dog square to the box (within 8 deg), in line with the handle (within 5 cm of the stance's line: RViz's text says
  how far left or right to move it), and either:
  - 0.87-1.20 m from the box's root (with the front camera: 0.98-1.20 m, above): it walks straight in until the root is
    0.75 m away;
  - or 0.71-0.80 m (only with `--camera wrist`): it does not walk. The arm reaches the lever with its full pose from
    0.03 to 0.11 m behind the sim's stance and 0.05 to 0.15 m right of it (the CPU model); the sim only ran its own.
- **The walk in is closed-loop, in classic walk.**
  - ClassicWalk goes out once when you press Enter.
  - The dog walks straight at 0.2 m/s with its heading held, and stops early by the carry it has seen.
  - It then stands 1 s, measures, and takes up to three corrective steps while more than 3 cm short.
  - The attempt then re-anchors where the dog actually stopped: the lidar refit places the box from there, and it
    goes on only if the box is 0.71-0.80 m off.
- **The pull is one continuous turn about the hinge.** The base and the door turn together, which in the base's frame is a
  constant twist. It is streamed at the gait's slowest usable speed for the whole pull (about 0.1 m/s back, 0.2 m/s
  sideways, 0.36-0.39 rad/s: 60 deg in about 3 s), and the script's door angle follows how far the dog has actually turned. The
  hand then stays on the door's arc, nearly still relative to the dog. The base stands for the hold and while the hand
  lets go.
- **The arm moves smoothly** (`arm_link.Profile`): a steady 10 Hz stream, 60 deg/s^2 up to 30 deg/s and back down into
  the goal, never more than 10 deg ahead of the arm. For a reach, the unfold already turns the hand to the grasp's
  orientation. When the IK misses the hand's pose for a step, the goal holds rather than switching wrists.
- **Joint moves wait for the arm.** The unfold, the tuck and the fold go at 40 deg/s in joint space (the Cartesian phases at
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
- **Camera:** the look camera's frame with the tag outlined (the front camera's; the wrist RealSense's through the
  insertion's key phases).

It is P2Dingo's `scripts/p2dingo-rviz.sh` with this directory's `combiner_door_robot.rviz`, this repo mounted into
the rviz container (`opc-compose.yaml`) and P2Dingo's static-TF helper. It finds the dog's zenoh listener on its WiFi
dongle (the address `sshuni` caches) before P2Dingo's own search.

**It needs two changes in the P2Dingo checkout (made 2026-10-20, uncommitted there):**

- the `/combiner_door/...` topics in both zenoh allow-lists (`zenoh/robot_bridge.json5`, `zenoh/opc_bridge.json5`);
  the dog's copy is deployed and its zenoh container restarted;
- an `OPC_COMPOSE_EXTRA` hook in `scripts/p2dingo-rviz.sh`, which lets this view add its compose override.

## The camera on the wrist (`--camera wrist`, the default since 2026-10-21)

Lukas moved the D435i from the head to the D1's wrist bracket. Each tag frame is placed with the camera's pose at
that moment: forward kinematics of the arm's measured joints and the bracket's mount
(`demos/cup/pick_demo/assets/mounts/wrist_mount.json`, the CAD: 30 degrees below the approach axis; not measured). A
frame taken while the arm moves (0.5 degrees between frames), or without fresh arm feedback, is not used.
- **Live**: before the look the arm goes to the look pose (`plan.look_pose_servo_deg`: the camera 0.25 m ahead,
  4 cm left and 0.40 m up of the base, aimed at a point 0.80 m ahead, with the image level; the tag is in view with
  the box's root 0.7-1.4 m ahead). The run's one Enter (none with `--yes`) now comes before that move.
- **Dry**: the camera looks from wherever the real arm holds it; nothing is moved to aim it.
- `--camera head` is the old fixed mount, for a camera put back there.

The tag's place now carries the arm's and the bracket's errors (the head mount was rigid): likely millimetres to a
centimetre. The lidar's refit still takes out the door's depth and heading. The key's insertion finds the lock itself
with the camera close up (`../../combiner_key/keyhole.py`).

## Out of range: the approach (2026-10-24)

A box the look sees but finds out of the task's range is no longer refused (Lukas: "make it so that the box is never
out of range as long as the dog sees the qr code"). Live, the dog walks to a **staging pose** first (`approach.py`): the
task's own stance (the door pull's, the key's push, the insertion's) moved straight back until the box's root is 0.75 m
ahead (`approach.STAGE_ROOT_M`; for the door pull that is its stance itself), square to the box, and the root no more
than 0.10 m to the side (`approach.LOOK_SIDE_M`: the tag on the door's centreline in view; the key's and the insertion's
stances, in front of the lock 0.35 m to the side, saw no tag from 0.75 m, F-220). Lukas, 2026-10-25: the
distance the close looked at the open door from, which saw the tag in both runs, for every look the program walks to. A
look there that finds no tag walks on back to 1.05 m (`approach.STAGE_FAR_ROOT_M`, the lens model's band) once and
looks again. It walks forward or back, sideways and turning at once (the key's
walk up, with the odometry's 13% under-count allowed for), the arm left where it is, and stops within 8 cm and 6 degrees.
Then it looks again, and the task's own walk in, refit and range check carry on from that look. Up to three approaches;
at most 3 m each. Not walked to: a tag the lidar says is not on a door the box's size (the charging dock). A dry run
says how far a live one would walk, and stops. The way there is not checked for obstacles.

The same walk takes the dog back after the key (`--then_door`, `approach.door_after_key`, 2026-10-25): standing 1.5 s
first, classic walk sent again, the arm folded, to the door pull's staging pose from the key's fix; then the door pull's
own look there. The unlocked door has swung open by then (~10 degrees), so the key's fix is not used for the pull.

## Closing and locking (`--then_close`, `--lock`, 2026-10-25)

Lukas: "a chain of events that close the door - first by grabbing the handle again, then pushing it closed while moving,
then scanning the door again (it has a spring in the last 10 degrees of closing that makes it stay open 5-10 degrees)
then sit down in front of the keyhole, unholster the lever, insert it ... and then push the key in while wiggling it".

```bash
d1dog --stage pull --then_close --live                                 # open, close, lock
d1doginsert --then_door pull --then_close --live                       # unlock, open, close, lock
d1doginsert --lock --live                                              # lock alone (the door pushed shut by the key)
```

1. **The close** (`key_plan.DoorClose`, `plan.close_phases`), reached from a look at the tag as the pull is (Lukas,
   2026-10-25: "this phase should use the apriltag to grab it just like on the open"):
   - The dog backs up, folded, to the door pull's staging pose in front of the open door (the pull's box turned about
     the hinge to the lidar's last angle; `approach.ApproachAttempt`), the root 0.75 m ahead (`approach.STAGE_ROOT_M`);
     a look there that finds no tag backs up to 1.05 m and looks again. The look's
     guard against the dock is its hinge within 0.30 m of the pull's (`plan.OPEN_DOOR_HINGE_LIMIT_M`), not the door's
     edges, which the box's side carries on past near 90 deg open (F-215). From where the pull leaves it the front camera
     cannot see the whole tag: it stands 0.2-0.4 m from the lens and runs off the top of the image; 0.3-0.4 m further
     back it is whole at 60-76 deg.
   - There it looks. The tag's enclosure pose is the box whose shut door stands where the open door does
     (`plan.about_hinge`), so the pull's range check, approach, walk up on three axes, settle and lidar refit (now of
     the open door's face) go on from that look unchanged. The door's angle is the look's heading against the pull's
     box; the box itself is the look's turned back about the hinge by it (the hinge stays put, so the push's centre
     is where the look has it whatever the odometry did). The record's `close_from_look` has the angle and how far the
     look moved the hinge from the pull's (`hinge_moved_mm`: the odometry's error through the pull and the walk back).
     After an unlock (`--task insert --stage insert --then_door pull --then_close`) the pull took the door hanging ajar
     on its spring as shut; the close then takes shut from the unlock's look instead, turned about the pull's hinge, when
     the two differ by 0-20 deg (`plan.shut_reference`; the record's `close_reference`).
   - `unfold`, `walk_in`, `walk_settle`, `stand`, then the pull's reach onto the handle where the look has it
     (`approach`, `settle`, `enter`, `arrive`, `grip`); the hand faces the way the base has turned.
   - The close's stance is 5 cm nearer the door than the pull's (`plan.CLOSE_STANCE_NEARER_M`, `plan.CloseFix`).
   - While the hand holds the door (`push`, `hold`), the base is placed on the hinge's arc by its own turn, not by the
     odometry's position, which fell 24 cm short of it over a 70-deg push and took the hand's goal out of reach (F-216).
   - `push`: the pull's turn about the hinge reversed, from wherever the base stands to shut (`CLOSE_TO_DEG`, 0) and on
     past it by 15 deg of the base's turn (`CLOSE_PAST_DEG`): the door lags the base (16-19 deg by the lidar, and pushed
     to 5 it stopped ~20 short of shut, Lukas). Past shut, the base stalling or the arm loaded is the door on its frame:
     the push ends there, done. The spring then holds it 5-10 deg open, which the lock's key pushes shut. The hand keeps
     its grip in the base's frame through the push and the `hold`; then the pull's letting go, `tuck` and `fold`. A door under 15 deg (the lidar's after the pull, or the look's) is not pushed.
   - Before (the close of 2026-10-25 12:35, `20261025T013412_pull_close_live`) the close reached from the pull's fix
     through the odometry and the lidar's door angle (`door_look`, `door_align`): 15 cm short of the handle and 7 cm
     under it (Lukas). By the hinge's geometry the pull's 60 deg turn should have carried the base 12.6 cm further than
     the odometry counted; the jaws stood as high in the base as at the pull's grip (0.305 m against 0.308), so the
     7 cm was the handle standing higher against the trunk than at the pull, which no plan from the pull's fix sees.
2. **Looking again**: the dog backs up to the insertion's staging pose (the door now 5-10 deg open on its spring) and
   looks; the insertion's range check, approach and walk up go on from that look.
3. **The lock** (`InsertAttempt.lock`, the insertion's whole sequence lying down, the key from the holster):
   - The key goes in at `--lock_from_deg` (45, the angle the unlock turned the cylinder to: `geometry.Lever` counts
     clockwise as the robot sees it positive, so this is Lukas's "-45"), the wiggle +-10 deg about it.
   - The key is pressed straight in at `key_in`'s 2 mm/s until its tip is `--lock_beyond_face_mm` (12; seated, 7.7)
     beyond the box's front face as the lidar saw it lying down, or until it stops in the keyway (`key_close`); no
     abort for going on. Until 2026-10-25 a seated key was pushed round the door's arc until it stopped, which with
     the door already shut pushed the box until the IK ran out (F-221). A door left ajar needs the argument raised.
   - Holding it there, `key_turn` turns the key back to level, then it comes out and goes back in the holster.
   - The record's `lock` (`lock_check`): seated, closed, the encoders' key within 15 deg of level, the key back.

In the dry tests the close looks at the open door (64-88 deg, the box up to 12 cm and 1.5 deg off where the pull's
odometry has it), walks up, refits on the open door's face, sends the jaws to within 1 mm of the real handle, holds them
within 3 mm of their goal through the push with the hand's whole pose and folds at the end; the lock pushes a door 8
and 10 deg open shut and turns the key to level, stops at once on a shut door (0.6 deg), and aborts on one that has not
stopped after 15 deg. The close from the look and the lock are not run on the dog (the dry tests' tag is seen by the
head camera's model; the front camera's view of the open door is only the geometry above). Not modelled: the door's
spring against the touch (the dry door stays put until `key_close`), and whether the real door's lidar angle is good past
80 deg (the pulls of 2026-10-24 had readings of 118-132 among 77-89).

## The key, turned by the arm (`--task key`)

The same program runs the key's push (`demos/combiner_key/robot/key_plan.py`, `d1dogkey`): its own stance (in front of
the lock), phases, planned and retimed turn, stall abort and lift-off. See `demos/combiner_key/robot/README.md`.

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

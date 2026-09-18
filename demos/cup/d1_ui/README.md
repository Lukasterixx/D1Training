# D1 reach console

A browser page that draws the Go2 and the D1 from their URDFs, moves the joints
live from the arm's own feedback, and turns a click on a sphere around the arm
into a Cartesian target for the real hardware.

## Sim or hardware

The console works out which one it is looking at before anything else:

1. **A simulator is publishing on this PC** (`./demos/cup/run_pick_demo.sh` or `./run_sim.sh`; both start the feed in
   `demos/cup/d1_ui/sim_feed.py` on localhost:8765). This is **sim mode**: the page draws the simulator's arm, fingers
   and legs, and the camera window shows the rendered wrist RealSense. SEND, PARK, RELEASE and LIVE are
   disabled on the page and refused by the server; clicking the sphere still previews the IK.
2. **Otherwise, on the dog** (the arm's NIC `enP8p1s0` exists): **hardware mode**, everything below.
3. Neither: sim mode, waiting for a simulator to start.

The header shows **SIM** or **HARDWARE**; hover over it to see why.

```bash
./demos/cup/run_pick_demo.sh       # terminal 1: the simulator (its feed starts with it)
./demos/cup/run_ui.sh              # terminal 2: sees the simulator, serves here in sim mode
                         # open http://localhost:8090; Ctrl-C stops the console, not the simulator
./demos/cup/run_ui.sh sim          # sim mode here even before a simulator is up (it waits)
./demos/cup/run_ui.sh robot        # deploy to the dog even though a simulator is running here
```

Sim mode runs in `env_isaaclab` (numpy, torch, ultralytics, OpenCV) without starting Isaac. The simulator
only copies joints and frames while a console is reading. Close the simulator and start another, and the
console picks the new one up.

## The camera window

A floating window over the 3D view (drag its bar, resize from the corner, `–` hides the image; where you
leave it is remembered in this browser). It shows the wrist camera with YOLO's boxes and mask outlines
**drawn onto the frame they were detected in** by the server, streamed as MJPEG from `/camera.mjpg`
(`/camera.jpg` is one frame). The footer lists the detections and YOLO's time per frame.

- Source: the simulator's rendered wrist camera in sim mode; on the dog, the first RealSense pyrealsense2
  finds (colour, 640x480 at 30 fps).
- Detector: the pick demo's stock `generated/yolo/yolo11s-seg.pt`, on the GPU when there is one. Only
  cups are boxed by default: `--detect cup,bottle`, `--detect all`, or `--detect none` for frames only.
  `--camera none` turns the window off; `--camera-fps` caps the rate (15).
- It degrades rather than fails, and says why in the window: no pyrealsense2 or no camera, no frames; no
  ultralytics, frames without boxes.

**Not yet run on the dog.** The RealSense source is written to librealsense's documented API and has never
opened a camera. Whether pyrealsense2, ultralytics and torch are installed on the Jetson is unknown.
`demos/cup/run_ui.sh` deploys `demos/cup/pick_demo/` and the weights (21 MB, copied once) for it.

## Bench mode: the arm on this PC

The arm and the RealSense can be plugged into the workstation instead of the Go2. `./demos/cup/run_ui.sh bench`
runs the hardware console here, finding the arm's interface itself (anything on 192.168.123.0/24, or
`D1_ARM_IFACE`). There is no `rt/lowstate` without the dog, so the legs are not drawn.

```bash
./demos/cup/run_ui.sh bench --pick-calibration demos/cup/pick_demo/assets/calibration/d435i_238222076237_640x480.json
```

Needs `cyclonedds` and `pyrealsense2` in the Isaac env (`pip install --no-deps`, so numpy 1.26 is left
alone).

## The scripted pick

The PICK button runs `pick_demo.hardware`, which drives the same `PickSequence`, grasp planner and
perception the simulated pick runs — YOLO finds a cup, depth and forward kinematics place it, `d1_ik`
plans a top-down grasp, and the arm is driven through it. What it is not:

- **The gripper closes on an unverified scale.** Servo 6 is commanded, carried as `angle6` on the same
  message as the arm pose. The URDF's 0–30 mm of finger travel maps onto the span the arm actually
  reaches — 0 to 50.2, measured (F-060), not the 65 the protocol advertises — with linearity between
  the endpoints assumed. Nobody has measured what a unit is in *millimetres of jaw gap*, and nobody has
  yet watched which end is open, so a run records the units it asked for and claims no width.
  `--pick-no-gripper` goes back to reach-descend-lift with the jaw untouched.
- **Not calibrated.** The wrist mount is assumed until the bracket is measured, so the cup's position
  carries whatever error the mount does. The page says so under the button.
- **Not accurate in any measured sense.** There is no ground truth on a bench, so a run reports what it
  did and saw, never how close it got.

**Nothing states where the table is.** Every pose the pick uses — the survey it looks from, the headings
it sweeps, the grasp — is placed relative to the arm's own mount, and the cup is measured by the camera in
that frame. There used to be a height to type, the arm's mount plane above the surface, and it was the one
number on this rig that nobody could measure: it moved three times in one day, and a wrong value did not
fail loudly, it tilted the plane the grasp was planned against. `--pick-up` remains — world up in the base
frame, `[0, 0, 1]` for an arm standing upright — because gravity is what a top-down grasp is defined
against.

**Dry run first.** With LIVE off, PICK runs the whole pipeline — real camera, real depth, real YOLO,
real planning — against a *virtual* arm pose that slews toward each waypoint, so the sequence runs to
the end instead of stalling on a stationary arm. Nothing is sent. The result says `feedback: virtual
(dry run)` so it can never be mistaken for evidence the arm followed the plan. With LIVE on, the same
code drives the real arm; STOP interrupts it and leaves the arm **holding**, never released (F-028).

Every pick writes `logs/pick_hardware/<stamp>_pick_hw/` with `run.json` and `events.json`, so
`./dashboard.py record` takes it like any other run.

## Measuring the gripper

The console has a **gripper jog**: a slider over servo 6's range and SET GRIPPER, LIVE-gated like every
other command. It exists because nothing in this repository knows what a unit of servo 6 is. Command a
value, put a ruler across the fingers, write down both — that pair of numbers is the calibration the arm
has never had, and it settles two open questions at once: what width the pick is really asking for, and
whether the real pads close further than the CAD's 17.26 mm (which decides whether any rim or wall grasp
is possible at all).

The fingers close under the drive's own effort. Keep hands out of the jaws.

### What it needs, and what it remembers

Nothing on the command line, and nothing in the page either. A console started with no flags finds the
one calibration stored in `demos/cup/pick_demo/assets/calibration/`, loads the saved wrist mount, and streams depth
(hardware mode; `--no-pick-depth` if the USB bandwidth is wanted elsewhere). There is no height to set:
the field that used to hold one, and the `demos/cup/pick_demo/assets/bench.json` that remembered it, are gone.

The button still refuses what cannot be worked around: sim mode has no arm to command, and a camera
without depth cannot place a cup. Those say so in the panel.

## The wrist mount editor

Where the camera sits on the wrist is a guess until someone measures the bracket, and that guess is
what turns a pixel into a point in the base frame. The **Wrist mount** panel moves it.

The RealSense's CAD (Intel's own D435 case, `third_party/realsense2_description`) hangs off the Link6
node in the 3D view, so it follows the arm as the real camera does. The small axes at its origin are the
**colour optical frame** — red x right, green y down, blue z forward — which is the frame depth is
deprojected in, not the case's centre.

**MOVE IN 3D** puts a gizmo on it: three arrows to slide it and three rings to turn it, both along the
**wrist's own axes** (Link6, not the world), with a label at the end of each axis reading how far along
it the camera sits, in cm, and the angle of the saved triple that belongs to it. Only the named axes are
offered — three.js's free-rotation handles are removed — so every drag is about one axis of the wrist.
While a ring is being dragged its label reads the turn that drag has applied, which *is* about that one
axis; at rest it reads `roll`, `pitch` or `yaw`, named because those are a ZYX sequence rather than three
independent turns, and one ring can move more than one of them.

The panel keeps the same six numbers for an exact value: `move` in **cm** across x y z, `turn` in
**degrees** as roll · pitch · yaw about the same three axes, the Link6 frame in the URDFs' `ZYX`
convention. The saved file is still metres and degrees. Changed numbers turn amber until saved.

Every change is sent to the server immediately, so when a pick is configured here the next frame is
deprojected through the new mount and you can watch the estimate move. Nothing is written until SAVE,
which asks for a name and writes `demos/cup/pick_demo/assets/mounts/<name>.json`. REVERT goes back to whatever
the console started with. A mount cannot move while a pick is running.

`demos/cup/pick_demo/assets/mounts/wrist_mount.json` is **the default everywhere**: save under that name and the
console, the pick and the body renders all load it on the next launch with nothing to pass, so the 3D
scene, the perception and the simulator agree. Any other name is kept but has to be named explicitly.
Every launch prints which mount it resolved, and `--pick-mount none` forces the four-number placeholder
back for a run that means to use it.

The file is six degrees of freedom with its provenance attached, and both ends take an explicit one:

```
./demos/cup/run_ui.sh bench --pick-mount demos/cup/pick_demo/assets/mounts/wrist_mount.json
python demos/cup/run_pick_demo.py --mount demos/cup/pick_demo/assets/mounts/wrist_mount.json
python demos/cup/run_camera_body_view.py --mount demos/cup/pick_demo/assets/mounts/wrist_mount.json
```

**What it is not.** Dragging a model until it looks right is an *alignment*, not a calibration. The
file records `"measured": false` and says in `method` exactly how it was arrived at, and
`pick_demo.camera.load_mount` carries that into the mount's own `source` string, so a run log that
cites it says "aligned by eye, not measured". A real extrinsic needs a hand-eye procedure against a
target; nothing here sets `measured` true. Treat the saved numbers as the bracket you *intended*,
good enough to stop the simulator and the controller disagreeing, and not as evidence about the
camera's true pose.

On the dog the panel works too — `run_ui.sh deploy` carries the case mesh — but the live clearance
warning needs `trimesh` to read it, which the Jetson has no reason to carry, so there it is simply not
offered rather than wrongly reported as clear.

The editor works without an arm or a camera — on a workstation it simply edits and saves a file, and
says so — because that is where the simulator runs.

## Launching it on the dog

```bash
./demos/cup/run_ui.sh              # deploy to the dog, start, print the URL (if no simulator is running here)
./demos/cup/run_ui.sh status       # is it up, what does it see, is it LIVE
./demos/cup/run_ui.sh stop         # stop the server (the arm is not touched)
./demos/cup/run_ui.sh restart      # stop, re-deploy, start
./demos/cup/run_ui.sh logs         # tail the server log on the dog
```

Then open **http://100.99.23.36:8090**.

It **must** run on the Go2's Jetson payload: CycloneDDS only reaches the arm from
the Jetson's arm-facing NIC (`enP8p1s0`, 192.168.123.x). Running it on the
workstation cannot see the arm. `demos/cup/run_ui.sh` copies the server and the meshes over
and starts it there; it re-deploys on every `start`, so editing a file here and
re-running is the normal workflow.

The remote copy lives under `/tmp/d1train`, which **does not survive a reboot of
the dog**. Re-run `./demos/cup/run_ui.sh` after one.

Overrides: `D1_UI_HOST` (default `$GO2_ROBOT`), `D1_UI_PORT` (8090),
`D1_UI_REMOTE_DIR` (`/tmp/d1train`). Arguments after the subcommand are forwarded
to `server.py`:

```bash
./demos/cup/run_ui.sh start --sphere-radius 0.45      # bigger reach sphere
./demos/cup/run_ui.sh start --no-legs                 # skip the Go2 rt/lowstate subscription
./demos/cup/run_ui.sh start --max-joint-step-deg 8    # per-cycle fallback step cap
```

### By hand, without the script

```bash
sshuni                                   # or: ssh unitree@100.99.23.36
cd /tmp/d1train && python3 demos/cup/d1_ui/server.py --port 8090
```

Ctrl-C stops it. Killing it from another ssh session needs care: `pkill -f
demos/cup/d1_ui/server.py` typed inside an inline `ssh '...'` matches the ssh shell running
it and kills the connection instead. Put it in a script file, or use
`./demos/cup/run_ui.sh stop`.

## Just starting it

`./demos/cup/run_ui.sh` with no arguments works out where it is and starts the console, and opens the page:

| what it finds | what it runs |
| --- | --- |
| this machine is the dog (Jetson, or it owns the deploy address) | hardware mode here, legs drawn |
| a simulator publishing on this PC | sim mode, following it |
| an arm on this PC's own 192.168.123.x NIC | bench mode |
| none of those, but the dog answers over ssh | deploy and start it there |

The arm being on a local NIC is the signal that it is *not* on the dog, and it is checked before the
ssh probe so a double-click starts at once instead of waiting out a timeout. Every path starts in DRY
RUN. `sim`, `bench` and `robot` force one when the guess is wrong.

## Stopping it

`sim` and `bench` run in the foreground here, and Ctrl-C stops them. A console whose terminal was
closed keeps the port, though, and the next start then cannot bind — the server now says which process
is holding it rather than raising. To clear it:

```
./demos/cup/run_ui.sh stop-local      # a console on THIS PC (sim or bench)
./demos/cup/run_ui.sh stop            # the console on the dog
```

Neither touches the arm: it holds whatever pose it was in.

## Using it

Drag to orbit, wheel to zoom. **Click the sphere** to pick a target; the server
solves IK and shows the result. Nothing moves until you press **SEND**.

- **LIVE** — the arm switch. Off by default, and the state lives in the server, so
  reloading the page cannot arm it. While off, every button rehearses.
- **SEND** — run the approach. Single-shot: one waypoint per pass, so the arm runs
  its own smooth trajectory (F-035). The gripper is held **level** at the goal
  (approach axis horizontal, no roll, heading free) for the end-effector camera.
- **STOP** — cancel. The arm is commanded to its measured pose and decelerates in
  place. It is **not** released, because on this arm release is a fall (F-028).
- **PARK** — bounded joint-space move back to the folded pose.
- **RELEASE** — drops motor torque. Refused unless the arm is folded; forcing it
  needs a second confirmation. From an extended pose the arm falls to its stop.

The small red marker is the server's forward kinematics of the tool point. If it
sits on the pincer tip, the drawing and the solver agree — it is the quickest
check that the page is telling the truth.

## What it refuses, and what it does not know

Targets are rejected below the mount plane, outside the sphere, when IK does not
converge, when the solution folds into the trunk/ground proxy, when the *path*
there sweeps through it (F-036 — endpoint clearance is not path clearance), or
when the point is reachable but **not with the gripper level** (F-037).

Levelling costs workspace: about **51%** of the sphere is reachable with the
gripper level against **89%** without. `./demos/cup/run_ui.sh start --no-level` trades the
level attitude back for the larger workspace. The preview always shows the pitch
and roll it would finish at, so a refusal says which constraint bit.

Level is commanded exactly (0.00°); the arm lands about **1.4° low**, consistently.
That bias is likely the unmeasured joint zero offset (F-023) rather than anything
the controller should correct.

The proxy is a capsule chain against the Go2's **standing** trunk box and a flat
ground. The dog is usually sitting. It knows nothing about the room, cabling, the
payload, or a camera on the end effector, and it has never been checked against a
real collision. **It is not a substitute for watching the arm.**

Every executed command is appended to `/tmp/d1_ui_log.jsonl` on the dog.

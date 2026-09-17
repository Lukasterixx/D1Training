# D1 reach console

A browser page that draws the Go2 and the D1 from their URDFs, moves the joints
live from the arm's own feedback, and turns a click on a sphere around the arm
into a Cartesian target for the real hardware.

## Sim or hardware

The console works out which one it is looking at before anything else:

1. **A simulator is publishing on this PC** (`./run_pick_demo.sh` or `./run_sim.sh`; both start the feed in
   `d1_ui/sim_feed.py` on localhost:8765). This is **sim mode**: the page draws the simulator's arm, fingers
   and legs, and the camera window shows the rendered wrist RealSense. SEND, PARK, RELEASE and LIVE are
   disabled on the page and refused by the server; clicking the sphere still previews the IK.
2. **Otherwise, on the dog** (the arm's NIC `enP8p1s0` exists): **hardware mode**, everything below.
3. Neither: sim mode, waiting for a simulator to start.

The header shows **SIM** or **HARDWARE**; hover over it to see why.

```bash
./run_pick_demo.sh       # terminal 1: the simulator (its feed starts with it)
./run_ui.sh              # terminal 2: sees the simulator, serves here in sim mode
                         # open http://localhost:8090; Ctrl-C stops the console, not the simulator
./run_ui.sh sim          # sim mode here even before a simulator is up (it waits)
./run_ui.sh robot        # deploy to the dog even though a simulator is running here
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
`run_ui.sh` deploys `pick_demo/` and the weights (21 MB, copied once) for it.

## Launching it on the dog

```bash
./run_ui.sh              # deploy to the dog, start, print the URL (if no simulator is running here)
./run_ui.sh status       # is it up, what does it see, is it LIVE
./run_ui.sh stop         # stop the server (the arm is not touched)
./run_ui.sh restart      # stop, re-deploy, start
./run_ui.sh logs         # tail the server log on the dog
```

Then open **http://100.99.23.36:8090**.

It **must** run on the Go2's Jetson payload: CycloneDDS only reaches the arm from
the Jetson's arm-facing NIC (`enP8p1s0`, 192.168.123.x). Running it on the
workstation cannot see the arm. `run_ui.sh` copies the server and the meshes over
and starts it there; it re-deploys on every `start`, so editing a file here and
re-running is the normal workflow.

The remote copy lives under `/tmp/d1train`, which **does not survive a reboot of
the dog**. Re-run `./run_ui.sh` after one.

Overrides: `D1_UI_HOST` (default `$GO2_ROBOT`), `D1_UI_PORT` (8090),
`D1_UI_REMOTE_DIR` (`/tmp/d1train`). Arguments after the subcommand are forwarded
to `server.py`:

```bash
./run_ui.sh start --sphere-radius 0.45      # bigger reach sphere
./run_ui.sh start --no-legs                 # skip the Go2 rt/lowstate subscription
./run_ui.sh start --max-joint-step-deg 8    # per-cycle fallback step cap
```

### By hand, without the script

```bash
sshuni                                   # or: ssh unitree@100.99.23.36
cd /tmp/d1train && python3 d1_ui/server.py --port 8090
```

Ctrl-C stops it. Killing it from another ssh session needs care: `pkill -f
d1_ui/server.py` typed inside an inline `ssh '...'` matches the ssh shell running
it and kills the connection instead. Put it in a script file, or use
`./run_ui.sh stop`.

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
gripper level against **89%** without. `./run_ui.sh start --no-level` trades the
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

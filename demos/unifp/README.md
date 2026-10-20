# The cup and combiner demos on UniFP's trained whole-body policy

The scripted demos in [`demos/cup/`](../cup) and [`demos/combiner/`](../combiner) drive the arm with
an IK reference while the dog lies on the floor (the cup) or stands still (the combiner, since
2026-10-19). These run the same two tasks under the trained UniFP position/force policy
(`checkpoints/unifp_go2d1_isaaclab_model_56000.pt`) with the dog **standing**, the cup on a table and
the box on a 10 mm plinth. Since 2026-10-19 the box is Lukas's real one (830 × 630 × 300 mm, handle
0.67 m up, hinged on the right; `demos/combiner/README.md`); every result below was measured on the
earlier proxy box (lever 0.30 m up on a 0.38 m post, hinged on the left) and has not been re-run.

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

# watch one attempt
./demos/unifp/run_demo.py --task cup --num_envs 1 --attempts 1 --wrist
./demos/unifp/run_demo.py --task combiner --num_envs 1 --attempts 1 --wrist

# measure: sixteen placements at once
./demos/unifp/run_demo.py --task cup --attempts 16 --headless --wrist
./demos/unifp/run_demo.py --task combiner --attempts 16 --headless --wrist

# where the jaws point when the policy settles on a goal -- the feasibility check
./demos/unifp/orientation_probe.py --checkpoint checkpoints/unifp_go2d1_isaaclab_model_56000.pt --headless
```

Then, as for every run in this repository: `./dashboard.py record <out dir> --title "..."`.

## What it does

16 placements per cell, same seed, same placements. **The environment count is a condition, not a
detail** — see F-098 and the warning below.

| | UniFP alone | with `--wrist` |
| --- | --- | --- |
| cup picked up and carried, `--num_envs 16` | 0/16 | **12/16** |
| cup picked up and carried, `--num_envs 1` | 0/16 | **7/16** |
| combiner door opened past 30°, `--num_envs 16` | 2/16 | **11/16** |
| combiner door opened past 30°, `--num_envs 1` | 3/16 | **8/16** |
| falls | 0 | 0 |
| tool-tip tracking at the grasp point | 1.7 cm (cup), 2.8 cm (lever) | |

> **Do not quote a per-placement result, and always state the environment count.** The two
> configurations above draw the same sixteen placements from the same seed and disagree on eleven
> of them, in both directions; agreement is 5 of 16, worse than two independent draws would give.
> Each configuration reproduces *itself* exactly, to the decimal. Rendering, the sequential reset
> path and grip margin were each tested and eliminated; no mechanism has been found (F-098). The
> binary criteria are what hid it — the continuous measures underneath (lever angle reached, door
> angle, cup height) show it at a glance, and are what a training change should be scored on.

[F-094](../../results/findings.md) to [F-097](../../results/findings.md) are the findings; the
[Week 2 log](../../results/week_02/notes.md) is the narrative.

## The three things that decide whether it works

1. **UniFP commands no orientation.** `CMD_EE_ORN_R/P/Y` are declared in `unifp_isaaclab/interface.py`
   and the task never writes them, so the hand points wherever the goal happens to put it — jaw axis
   4° to 87° from horizontal over the workspace, swinging a median 20° at a goal that is not moving.
   `--wrist` replaces the policy's `Joint6` action with a servo (`wrist.py`) that levels the jaws.
   **That is a different controller**, and a result from it must never be quoted as UniFP's.
   `--wrist_joints 56` adds the wrist pitch, which levels the jaws no better and costs 15 cm of reach.
2. **The controlled point is the tip of one finger, and the jaws are not in the observation.** Opening
   them to grasp slides that point up to 30 mm for a reason the policy cannot see: 0.91 cm of tracking
   error with the jaws shut, 3.50 cm fully open. The launcher puts the measured travel back into the
   commanded goal, which recovers all of it (0.73 cm). `--no_jaw_compensation` measures the cost.
3. **The furniture heights are measured, not chosen.** The hand arrives 40–47° nose-down at goals near
   the middle of the goal sphere and level only at pitch 20–45°, so the table is 0.60 m and the box's
   handle 0.68 m up (plus the arm's rise) — the grasp points land where the probe says the approach is
   flat and tracking is 0.9 cm. The proxy box stood on a 0.38 m post for that; the real box's handle is
   already 0.67 m up, so it stands on a 10 mm plinth. On the real box, a 45° latch-side
   `--stance_deg` carries the door's pull out to a 0.61 m radius, past the trained 0.58.
   `props.py` carries the reasoning.

## What a result here does and does not show

- **No perception runs.** The cup's and the box's pose are given. The scripted demos find them with a
  wrist RealSense and a detector, and aiming that camera needs the wrist control this policy does not
  have, so what is compared is the manipulation phase from first motion to release.
- **The arm is not the measured D1.** UniFP's port drives it with UniFP's own PD — peak 1.77 rad/s
  against the D1's measured 1.21–1.29 (F-033) and the ~0.8 a 10 Hz stream achieves through its
  firmware planner (F-046). No timing here transfers to the bench, and no speed comparison against the
  scripted demos holds until both run the same arm model (F-097).
- **The postures differ too.** These stand with the object lifted into the goal sphere; the scripted
  demos lie down with it on the floor. Success rates are not comparable across that.
- Simulation only, one checkpoint, one seed, 16 placements per cell.

## The combiner under the mechanism-trained policy (`--mech`)

```bash
./demos/unifp/run_demo.py --task combiner --mech --num_envs 1 --attempts 1 --door_torque_nm 8    # watch one
./demos/unifp/run_demo.py --task combiner --mech --attempts 16 --headless --door_torque_nm 16    # measure
```

`--mech` runs the policy of F-108 — trained to open mechanisms it is never told the resistance of, with a task
layer's force law in the loop — the way it was trained, which the plain demo would waste:

| what | why |
| --- | --- |
| an **L-lip claw**: a 20 mm lip turned inward just beyond each fingertip, the two on opposite halves of the finger's width so they pass each other and overbite at full close — real colliders (`claw.py`, the robot rebuilt with them in `generated/claw/`), the jaws approaching fully open and closing fully onto the bar | Lukas's claw design; without a claw a friction grip is what gives first (F-073). PhysX decides whether the bar is held; `mech_env` only watches whether it is in the loop. `--claw_model spring` is the first model: a spring from the jaw centre to the lever that holds in every direction |
| while the claw holds: the **force law** — PI on how far the handle lags the script, 80 N cap, integral bled once it arrives — along the joint each phase drives (the lever to turn it, lever and door to crack it, the door to open it), the goal on the handle along that joint and on the script's reference elsewhere (`mech_env.LAW_JOINTS`, `LAW_GOAL`) | how the policy was trained to be commanded; one joint at a time as training had one |
| the door's force capped at 15 N while the latch still holds | a task layer that can see the lever should not yank a latched door |
| a **roll command** puts the jaws across the bar | the policy has the roll objective; no wrist servo |
| a **goal correction** slides the commanded goal by the jaw centre's steady miss during the reach's holds | this policy arrives 5 cm off (its training grasped wherever it stopped); corrected, 2.5 mm |
| the jaw-centre tool point, the arm's 0.01 kg·m² armature, turn the lever to its stop (60°); with the lip claw hook it 80 mm out and keep it at the stop through the pull, rolling the jaws to its measured angle while the door opens (the spring claw: 95 mm, eased back) | its training conditions; pushing the lever down is its weakest move; the lip claw is 26 mm wide (at 95 mm half of it hangs off the 105 mm lever) and a lever turned by the pull slides out of it |

**Where the robot stands** (`--stance_deg`, F-113): the box turned about the grasp point, so the reach is unchanged and
only the directions the lever and the door move in turn. Positive is the latch side, where the lever's late turn draws
the handle toward the robot: the policy turns a stiffer lever there (lip claw best at +15°: 13/16 opened at a 1.2 N·m
lever, against 2/16 square). Negative is the hinge side, where the opening door comes toward the robot and the pull goes
into the lips: no late escapes at a 16 N·m closer from −30° (square: 13 of 16). Each side makes the other phase harder;
at +45° the forearm lands on the box. Pick the side by which of the two is stiff.

**The lever-trained checkpoint** (`checkpoints/unifp_go2d1_mech_lever_model_20497.pt`, F-114) moves the best side to the
hinge side, where it takes a stiff lever and a stiff door together: with the lip claw, 12/16 at a 1.2 N·m lever (the
default checkpoint 2) and 9/16 with a 16 N·m door closer as well (the default 4 at its best stance). It is worse from the
latch side and needs `--correct_entry`, which keeps the goal correction on through the entry, across the approach:

```bash
./demos/unifp/run_demo.py --task combiner --mech --num_envs 1 --attempts 1 --correct_entry --stance_deg -30 \
    --handle_torque_nm 1.2 --door_torque_nm 16 --checkpoint checkpoints/unifp_go2d1_mech_lever_model_20497.pt
```

`--post_top_m` and `--box_distance_m` raise the box and bring it nearer; `--post_top_m 0.53 --box_distance_m 0.56 0.60`
puts the handle at 0.83 m, above the robot's shoulder, with every goal inside the trained sphere.

`--door_torque_nm` adds a door closer (torque at 45°); `--claw` and `--goal_correction` give any controller the same
claw and correction, and `--no_force_law` keeps them and drops the law. Measured with the **L-lip claw**, lips
overbiting beyond the fingertips, 16 placements, lever 0.4 N·m (F-112; `results/week_02/figures/combiner_mech_sweep_overbite.png`):

| door closer | `--mech` | `--mech --no_force_law` | UniFP + wrist, same claw, correction and script |
| --- | --- | --- | --- |
| free | **16/16** (door 46°) | 1/16 | 13/16 |
| 4 N·m | **16/16** (45°) | 0/16 | 6/16 |
| 8 N·m | **16/16** (42°) — also 16/16 on untuned placements | 0/16 | 0/16 |
| 12 N·m | **16/16** (43°); the lever came out of the claw late in 5 | 0/16 | 0/16 |
| 16 N·m | **16/16** (35°) — also on untuned placements; the lever came out late in 13 (11 untuned) | 0/16 | 0/16 |

From 12 N·m the door peaks past 30° every time, then the lever often leaves the claw near the end of the pull and the
door swings back: once the door has swung, the pull runs along the lip face rather than into it, and the policy
commands no direction for the hand. A lip claw wants its pull going into the lips. The first lip model, with the lips
across the fingertips so the jaws stopped short of the bar (F-111, `combiner_mech_sweep_lips.png`), measured within a
few degrees of this with the force law; the full close helped the old controller on a free door (13/16 against 11/16)
but not beyond it.

With the first, spring claw (F-110; `combiner_mech_sweep.png`):

| door closer | `--mech` | `--mech --no_force_law` | UniFP + wrist, same claw, correction and script |
| --- | --- | --- | --- |
| free | **16/16** (door 58°) | 1/16 | 9/16 |
| 4 N·m | **16/16** (44°) | 0/16 | 3/16 |
| 8 N·m | **16/16** (41°) — also 16/16 at one environment | 0/16 | 0/16 |
| 12 N·m | **16/16** (40°) | 0/16 | 0/16 |
| 16 N·m | **16/16** (35°) | 0/16 | 0/16 |

No falls or tears anywhere. What it costs: the claw peaks near 100–118 N at 12–16 N·m (its limit 150), and the
door ends 35–40° open, not the scripted 50°, inside the script's clock. With the lip claw the finger contact force
peaks at a median 118–124 N at 12–16 N·m. What neither shows: the lips (4 mm thick,
rigid) and the gripper's drives (the URDF's 15 N) are models of parts not yet made or measured, the box's springs and
the latch are models, the lever is still only 0.4 N·m — pushing it down is the limit, not pulling the door — and
nothing here has run on the robot.

## The door pull (`--task door`)

```bash
./demos/unifp/run_demo.py --task door --wrist --num_envs 1 --attempts 1        # watch one
./demos/unifp/run_demo.py --task door --wrist --attempts 16 --headless         # measure
```

The combiner box with its door **unlocked** and the handle never turned: the hand grips the handle and
the robot **walks the door open to 90°** (`--door_deg`). The reach onto the bar is the combiner's. Then
`script.door_phases` runs the door's angle up on the clock (7 s, `--pull_s`) and moves two things with
it: the jaw centre along the handle's arc, and the base round the hinge with the door
(`props.BoxSite.base_pose_m`). Robot and door turn as one body, so the handle stays put in the robot's
frame and the goal never moves in the goal sphere. The robot also stays out of the door's sweep: the
footprint keeps at least 0.19 m from the door and the box on the planned path (`test_unifp_demos.DoorPull`).
On the real box (hinge on the robot's right) the base steps right while turning left, ending 0.88 m
right, 0.25 m forward and turned 90°, facing the open door's outer face.

The walk is UniFP's own base command, which the other demos hold at zero. Each policy step, `env.py`
turns the script's base pose into (vx, vy, yaw rate) with `script.base_velocity_command`: the path's
rate fed forward, the base's measured pose error fed back at 1.5/s, clipped to the trained ranges. Over
the swing the command is a steady 0.07 m/s back, 0.13 m/s right and 0.22 rad/s left. Those are just
outside the dead zones (0.1 m/s, 0.2 rad/s) inside which UniFP is told to stand, so a much slower
`--pull_s` would stall the gait. The latch is `LatchController(locked=False)`; the lever's spring still acts.

**Results (2026-10-19, `--wrist`, 16 environments).** No falls in 116 attempts. The base's peak error
from its path is 3–11 cm (median 5.5) and up to 11° of yaw, and it ends turned 87–95°. When the grip holds,
the door is at 87–105° when the hold ends. **The grip holds about half the time:** 8 of 16 and 7 of 16 on
seeds 1 and 2. Of the 17 failures, 4 doors barely moved (under 6°), 12 lost the handle at 15–39°, and
one swung past 108° and back to 65°. Reach misses play a part (the failures' median miss at arrive was
12 mm, against 5 mm for the successes, on a bar the open jaws clear by 11.5 mm a side). Others began
while the jaws closed on a well-placed bar (attempt 1, seed 1: 3 mm off, then the hand pitched up 30° and
backed 24 mm off the bar during the grip). It is a friction pinch on an 18 mm bar, pulled out along the jaws. Tried,
on the same placements: `--goal_correction` 8/16; a 2 s grip 10/16 and 7/16; `--wrist_joints 56` 3/16;
the lip claw (`--claw`, without the mechanism policy's goal correction and roll) 2/16. Outcomes flip
between settings at the same placement, as F-098 found for the combiner, so quote rates, not
placements. F-146.

### The non-UniFP baseline (`--controller decoupled`)

```bash
./demos/unifp/run_demo.py --task door --controller decoupled --attempts 16 --headless
./demos/unifp/run_demo.py --task door --controller decoupled --attempts 16 --headless --no_object --free_space --park_arm   # the walk alone
```

The same scene, placements, script, walk feedback, latch, jaws and success criterion, with a conventional stack in
place of UniFP (`decoupled.py`, `decoupled_arm.py`):
- **Legs:** a walking policy trained on a bare Go2 (`--walker`): P2Dingo's clocked gait by default since 2026-10-20
  (`checkpoints/p2dingo_go2_gait_v7_model_7000.pt`), or `rescue_2024`, the checkpoint `sim.py` plays back. Each gets its
  own observation (height scan included; the gait clock for P2Dingo's), its DCMotor legs, self-collision off and the
  foot friction it trained on. With UniFP's self-collision and friction the 2024 policy tracked a third of its command
  and fell.
- **Arm:** IK on the jaw centre and the scripted grip's hand orientation, nearest-level grasp, re-solved every
  10 Hz command in the base's current frame. It runs through the D1 firmware planner into the measured servo model.
- **Jaws:** UniFP's drive.

Neither half knows about the other. `trace.csv` carries the IK's outcome (`ik_status`), and `summary.json` per-attempt
counts and the chosen grasp.

**Results (2026-10-19, 16 environments):** 3/16 and 6/16 doors open on seeds 1 and 2 (9 of 32), against UniFP's 8/16
and 7/16 with `--wrist` (15 of 32), with no falls. The arm IK is bounded to run at 10 Hz (≤ 7 ms a command). Its first
version re-ran a seeded search on every miss, up to 486 ms a command, which stalled the viewer. That version opened 15
of 32 and is kept as `--seeded_ik_loop` for comparison (F-152).
- **Arm:** the jaw centre arrives within 3 mm.
- **Legs:** the base trails its path by up to 0.27–0.72 m (median 0.39), as badly with no box and the arm parked.
- **How it fails:** the door is pulled past 66° in 14 of 16, but the grip goes when the base lags out of the arm's
  reach, and the free door swings back.

**On P2Dingo's walking policy (2026-10-20, the default since):** 16/16 and 16/16 on seeds 1 and 2, no falls (F-153).
- The base trails its path by 0.10–0.12 m at worst (median attempt).
- The 2024 policy, stopped from a walk faster than 0.06 m/s, turns in place and leans up to 26°; P2Dingo's stops level.
- The walking policy was the stack's weak half, and with it fixed the stack beats UniFP + wrist's 15 of 32 on this task.

It is still a stack on its own plant (legs and arm both differ from UniFP's), not a same-plant comparison. F-151, F-152,
F-153. The same stack finding the box itself, with a head camera and the lidar: `demos/combiner_door/`.

## The wrist camera

Every run draws the wrist RealSense in its printed bracket on the gripper, as fitted to the real arm (Lukas,
2026-10-19), at the saved mount (`demos/cup/pick_demo/assets/mounts/wrist_mount.json`, from the bracket's CAD).
`--no-camera_body` leaves it out. It is visual only, with no mass, no collider and no rendering, so results are
unchanged: the door pull's seed-1 run with it reproduced the one without, door angle for door angle. The policies were
trained without its weight. Nothing in the simulator would show it hitting the box, so
`demos/combiner/tests/test_turn.py` checks it: at every grasp the planner can choose it stays 30 mm off the door
and 25 mm off the lever, spindle and rosette (proxy lever shape).

## The pieces

| file | |
| --- | --- |
| `run_demo.py` | the launcher: builds the scene, loads the checkpoint, runs the attempts, writes `run.json`/`trace.csv`/`summary.json` |
| `env.py` | `UniFPDemoEnv`, a subclass of the trained `Go2D1PosForceEnv` — furniture, scripted goal, commanded jaws, standing by command, or walking where the script names a base pose |
| `script.py` | the demos as clock-driven phase lists. Every arc runs on a clock, not on arrival, and the sequence never reads the thing it is working (both from the scripted demos) |
| `props.py` | table, post, placements, and where the grasp points land in UniFP's goal sphere |
| `wrist.py` | the roll servo, and why it takes the joint it takes |
| `mech.py`, `mech_env.py` | `--mech` and `--claw`: the claw, the task layer's force law, the roll command, the goal correction (plain torch, then the environment) |
| `claw.py` | the L-lip claw's geometry and the lip-carrying copy of the D1 URDF (numpy only) |
| `orientation_probe.py` | where the jaws point when the policy settles: the feasibility measurement that comes before a demo |
| `tests/` | `python -m unittest discover -s demos/unifp/tests -t .` — geometry, the phase machine, the servo's algebra, and that every commanded goal stays inside the trained sphere |

## Controls worth knowing about

The demo's failures confound easily — a missed grasp looks the same whether the arm could not follow
the path or met the object badly — so the launcher carries the separations that were needed to
diagnose it:

| flag | what it isolates |
| --- | --- |
| `--no_object` | the object moved aside, the furniture left: how well the arm follows the demo's path |
| `--free_space` | the furniture moved aside too |
| `--jaw_hold TRAVEL_M` | the jaws pinned open or shut for the whole run (F-094's sweep) |
| `--command_tip` | the tool point commanded at the target instead of the jaw centre |
| `--zero_actions` | no policy at all: how much of any result is the script |
| `--speed_scale`, `--hold_scale` | every phase, or just the two holds, scaled on the clock |

With the viewer (no `--headless`), the run defaults to **one environment**, and further attempts follow one after
another. Headless runs default to `--attempts` at once, which is how the rates above were measured. The count changes
results (F-098), so state it with any rate. The goal overlay (yellow goal, magenta scored goal, blue tool point, cyan
sphere centre) is off by default since 2026-10-19 so the hand can be seen; `--markers` draws it.

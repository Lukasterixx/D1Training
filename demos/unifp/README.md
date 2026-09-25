# The cup and combiner demos on UniFP's trained whole-body policy

The scripted demos in [`demos/cup/`](../cup) and [`demos/combiner/`](../combiner) drive the arm with
an IK reference while the dog lies on the floor. These run the same two tasks under the trained
UniFP position/force policy (`checkpoints/unifp_go2d1_isaaclab_model_56000.pt`) with the dog
**standing**, the cup on a table and the box on a post.

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
   the middle of the goal sphere and level only at pitch 20–45°, so the table is 0.60 m and the post
   0.38 m — the grasp points land where the probe says the approach is flat and tracking is 0.9 cm.
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
| a **claw** hooks the lever bar when the grip closes, if the jaw centre is within 2.5 cm of it (`mech.ClawCoupling`, 2,000 N/m, torn out above 150 N) | the training's grasp, and Lukas's "the pincers become claws"; without it a friction grip is what gives first (F-073) |
| while the claw holds: the **force law** — PI on how far the handle lags the script, 80 N cap, integral bled once it arrives — along the joint each phase drives (the lever to turn it, lever and door to crack it, the door to open it), the goal on the handle along that joint and on the script's reference elsewhere (`mech_env.LAW_JOINTS`, `LAW_GOAL`) | how the policy was trained to be commanded; one joint at a time as training had one |
| the door's force capped at 15 N while the latch still holds | a task layer that can see the lever should not yank a latched door |
| a **roll command** puts the jaws across the bar | the policy has the roll objective; no wrist servo |
| a **goal correction** slides the commanded goal by the jaw centre's steady miss during the reach's holds | this policy arrives 5 cm off (its training grasped wherever it stopped); corrected, 2.5 mm |
| the jaw-centre tool point, the arm's 0.01 kg·m² armature, turn the lever to its stop (60°) and hook it 95 mm out | its training conditions; pushing the lever down is its weakest move, and at 52° and 75 mm the lever stalled short of the latch |

`--door_torque_nm` adds a door closer (torque at 45°); `--claw` and `--goal_correction` give any controller the same
claw and correction, and `--no_force_law` keeps them and drops the law. Measured, 16 placements, lever 0.4 N·m
(week 2 log, 2026-09-25; `results/week_02/figures/combiner_mech_sweep.png`):

| door closer | `--mech` | `--mech --no_force_law` | UniFP + wrist, same claw, correction and script |
| --- | --- | --- | --- |
| free | **16/16** (door 58°) | 1/16 | 9/16 |
| 4 N·m | **16/16** (44°) | 0/16 | 3/16 |
| 8 N·m | **16/16** (41°) — also 16/16 at one environment | 0/16 | 0/16 |
| 12 N·m | **16/16** (40°) | 0/16 | 0/16 |
| 16 N·m | **16/16** (35°) | 0/16 | 0/16 |

No falls or tears anywhere. What it costs: the claw peaks near 100–118 N at 12–16 N·m (its limit 150), and the
door ends 35–40° open, not the scripted 50°, inside the script's clock. What it does not show: the claw, the box's
springs and the latch are models, the lever is still only 0.4 N·m — pushing it down is the limit, not pulling the
door — and nothing here has run on the robot.

## The pieces

| file | |
| --- | --- |
| `run_demo.py` | the launcher: builds the scene, loads the checkpoint, runs the attempts, writes `run.json`/`trace.csv`/`summary.json` |
| `env.py` | `UniFPDemoEnv`, a subclass of the trained `Go2D1PosForceEnv` — furniture, scripted goal, commanded jaws, standing by command |
| `script.py` | the demos as clock-driven phase lists. Every arc runs on a clock, not on arrival, and the sequence never reads the thing it is working (both from the scripted demos) |
| `props.py` | table, post, placements, and where the grasp points land in UniFP's goal sphere |
| `wrist.py` | the roll servo, and why it takes the joint it takes |
| `mech.py`, `mech_env.py` | `--mech` and `--claw`: the claw, the task layer's force law, the roll command, the goal correction (plain torch, then the environment) |
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

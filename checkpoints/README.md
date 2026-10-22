# Trained policies

The two UniFP position/force controllers for the welded Go2 + D1, kept here rather than under
`logs/` because `logs/` is gitignored and these are deliverables: they are what you copy to
another machine to run the task.

Both drive the **same** 18-action interface and load through the same code path
(`unifp_train.eval.EvaluablePolicy`), which reads UniFP's single-module layout and rsl-rl 5.x's
split-actor layout alike. Verify what you copied against `SHA256SUMS`.

## The key-turning leg policy, exported for the dog's shadow mode (2026-10-21)

`leg_key_force_s2walker_model_2104.npz` is the force-aware front-right-leg key policy fine-tuned from the walking
policy's stances (seed 2, `model_2104`, F-167; from `logs/unifp_train/20261020T203809_train_seed2_key_force_s2_stage3_walker/`,
which is gitignored), exported by `demos/combiner_key/robot/export_policy.py` for the dog's numpy runner
(`demos/combiner_key/robot/`). It holds the weights and every constant of the policy's interface, and the export
checked numpy against the checkpoint (actions to 3.3e-6). It is not a checkpoint: `EvaluablePolicy` does not load it,
and it cannot be trained further. Trained on UniFP's ideal-PD legs with armature 2e-4, no delay and no noise (F-168):
for shadow mode, not for driving the legs.

`leg_key_force_ft045_model_2604.npz` (2026-10-21 evening) is the same export of the fine-tune on Lukas's turn (F-179):
force-aware, from seed 2's `model_1605`, 1000 iterations on the turn from horizontal to 45 degrees down with a light lock,
a 30 N press limit, the geared-knee legs with a 5-15 ms delay and armature drawn per environment, and 80% of its
episodes started from sport-mode stands (`logs/unifp_train/20261021T053626_train_seed2_key_ft045_force_s2/`, gitignored).
Its `meta.trained_on` holds the training run's own settings. Numpy against the checkpoint: actions to 4.9e-6.

`leg_press_force_r4_model_5999.npz` (2026-10-22) is the same export of the general foot-press policy (`--leg_object
press`, the week-6 log's "A general foot-press policy"): force-aware, the round-4 lineage's final (from F-179's
`model_2604` through rounds 2-4, ~3400 iterations of plates anywhere the foot reaches and UniFP's free-space goals with
pushes, on the same legs, delay, armature and sport-mode stands; `logs/unifp_train/20261021T140237_train_seed3_press_r4_force_s3/`,
gitignored). It takes a foot goal and a force command from a task layer; its waiting command (the foot on its
footfall) is the key policy's. Numpy against the checkpoint: actions to 8.6e-6.

## A walking policy: P2Dingo's clocked gait (2026-10-20)

`p2dingo_go2_gait_v7_model_7000.pt` is not a UniFP controller: it is the 12-leg velocity tracker P2Dingo's sim installs
(`go2_p2dingo_gait` v7), copied for the decoupled door pull's legs (`demos/unifp/decoupled.py`, `--walker p2dingo_gait`)
and the combiner door demo (`demos/combiner_door/`), where it is the default. It trained on a bare Go2, observes a gait
clock (`demos/unifp/p2dingo_gait_clock.py`, which must not change) and loads through `decoupled.WalkingPolicy`, not
`EvaluablePolicy`. Provenance: `third_party/p2dingo/NOTICE.md`.

## The robot with its lidar and wrist camera (since 2026-10-19): none of these trained on it

The real robot carries a MID-360 in its mount on the trunk and the RealSense in its bracket on the gripper (`payload.py`).
`run_position_only.py` and `run_unifp_train.py` now build that robot by default (`--payload physical`). Every checkpoint
here was trained without it, so **re-score them with `--payload visual`** to reproduce the numbers below. On the payload
robot the reach default `..._cad333_model_58400.pt` scores 1.69 / 2.33 cm median / p90 on the validation set, against
1.50 / 1.96 on the same day without it, with no falls (F-147). Lukas retrains on another machine.

The numbers below for `58400` (1.40 / 1.93 cm) do not reproduce from this repository: the arm URDF they were scored with
(sha256 `1509b0bc…`, on Victor's machine) is not the committed `d1_arm/d1.urdf` (`a848ee41…`). The committed one gives
1.50 / 1.96 (F-148).

## On the CAD-length robot with a 3.33 N·m shoulder and elbow (since 2026-10-09)

The simulated D1 changed again on 2026-10-09:
- **Lengths** (F-139): its arm took Unitree's D1-550 CAD lengths, so the forearm is 6.8 mm longer and the controlled
  point moves a median 6 mm.
- **Torque** (F-140): Joint2 and Joint3 are 3.33 N·m, where Joint3 was 1.7. That is Lukas's call from the CAD's
  identical servo housings, and it is not measured.

Every checkpoint was scored on its frozen sets on the new robot, and each lineage was fine-tuned on it, with an
identical run on the previous robot as the control (Week 4 log, 2026-10-09). What to use:

| use | file | on the new robot | why |
| --- | --- | --- | --- |
| reach (UniFP demos' default) | **`unifp_go2d1_isaaclab_cad333_model_58400.pt`** (new) | 1.40 / 1.93 cm median / p90 (`57499` here: 1.78 / 2.19); cup demo 10/16 (`57499`: 6/16) | `57499` fine-tuned 900 iterations on this robot; its control never got the p90 below 2.24 |
| rope pull | **`unifp_go2d1_rope_cad333_model_14600.pt`** (new) | held out: 21.9 / 31.9 / 45.2 N for 15 / 30 / 45 N, 0 of 192 releases fallen (`..._polish_14600` here: 21.3 / 35.6 / 51.1 N) | F-133's v3 recipe re-run on this robot from hook v4 `13000`, chosen by the rope rule |
| mechanism controller | `unifp_go2d1_mech_law_model_17499.pt` (unchanged) | 491 of 512, 4 falls; standing combiner demo 16/16 | its pinned-rate fine-tune `18000` scores 497 with no fall but opens 13/16 in the demo |
| lever | `unifp_go2d1_mech_lever_measured_model_21496.pt` (unchanged) | levers 336, test 492; **lever demo 9/16 with 2 falls** (16/16 at 1.7 N·m) | every fine-tune lost the claw's bar in all 16 demo attempts |
| insertion | `unifp_go2d1_insertion_seed3_measured_model_21400.pt` (unchanged) | lateral 76 (1 damaged), detent 79 (2); demo 44/48, 44/48, 89/96 | every fine-tuned checkpoint scored lower (F-121's decline, on both robots) |

The new rope policy pulls differently from every earlier one. It leans 3.6–5.4° rather than ~20–24°, so the arm
makes more of the force, with Joint2 and Joint3 at their 3.33 N·m limits. Both joints are a Fashion Star RX18-U100H-M,
3.14 N·m rated and 4.90 continuous (F-142), so Unitree's 1.7 N·m for the elbow is not the motor's limit. Whether the
firmware caps it lower is not measured: check the elbow's torque before running this on hardware.

```bash
./run_unifp_train.py rope_eval --task rope --headless --rope_set test --checkpoint checkpoints/unifp_go2d1_rope_cad333_model_14600.pt
./run_unifp_train.py eval --headless --tool_point fingertip --manifest results/manifests/unifp_isaaclab_validation.json \
    --checkpoint checkpoints/unifp_go2d1_isaaclab_cad333_model_58400.pt
```

One seed each, simulation only. The sections below describe each checkpoint on the robot it was trained on.

## On the measured robot (since 2026-10-06)

Since 2026-09-30 the simulated D1 has the base Lukas measured on the real arm: Joint1 at 76.0 mm, not Unitree's
53.3 mm. Its mount is on the dock's rails midway between their screws, at (−0.019, 0, 0.0855) m (F-125, F-126). Every
checkpoint below this section was trained on Unitree's base, at (0, 0, 0.08) or F-117's (0.0025, 0, 0.0855). On the
measured robot the shoulder is 2–3 cm from where those policies learnt it, and they miss by about that much (F-118).

Three were fine-tuned on the measured robot and beat their originals there. They are the ones to use (F-127; Week 4
log, 2026-10-05/06):

| file | from | fine-tune | measured robot, frozen sets | measured robot, PhysX demo |
| --- | --- | --- | --- | --- |
| `unifp_go2d1_isaaclab_measured_model_57499.pt` | `..._isaaclab_model_56000.pt` | 1,500 iterations, UniFP recipe, fingertip | 1.70 / 2.16 cm median / p90 (56000: 4.35 / 4.86; on its own robot 1.51 / 2.17) | cup 2/16, box 1/16: the scenes, not the policy (below) |
| `unifp_go2d1_mech_lever_measured_model_21496.pt` | `..._mech_lever_model_20497.pt` | 1,000 iterations, lever recipe | levers 343/360 (20497: 280); held-out test 505/512 | lever demo **16/16** (20497: 10/16; on its own robot 9/16) |
| `unifp_go2d1_insertion_seed3_measured_model_21400.pt` | `..._insertion_seed3_model_21200.pt` | 200 of 800 iterations, pinned 1e-5 | lateral 73/96, detent 68/96 (21200: 33, 29) | **46/48, 45/48, 85/96** at 2 mm (21200: 39, 40, 78; on its own robot 43, 43, 80) |

- **The mechanism controller stays `unifp_go2d1_mech_law_model_17499.pt`.** Unretrained, it opens the combiner box 15/16
  at a 16 N·m door on the measured robot (16/16 on its own). Its 1,000-iteration fine-tune scored worse on every saved
  checkpoint: 481–443 of 512 against 498, and 12/16 in the demo.
- **`demos/unifp/run_demo.py`'s reach default is now the measured-robot checkpoint.** With the scenes re-placed with
  the arm (F-128), its cup demo picks 7/16 (12/16 on the old robot). Its box demo opens 0–1/16 as it was. With
  `--wrist_joints 56 --goal_correction --turn_deg 55 --ease_deg 30` it opens 9/16 and 3/16 at seeds 1 and 2, against
  11 and 9 on the old robot (F-129). That is a different controller from plain `--wrist`. Use the mechanism
  controller for the box (16/16).
- Lever `21496` still needs `--correct_entry`; without it, it misses the bar (0/16), as `20497` did (F-114).

```bash
./demos/unifp/run_demo.py --task combiner --mech --attempts 16 --headless --correct_entry --stance_deg -30 \
    --handle_torque_nm 1.2 --door_torque_nm 16 --checkpoint checkpoints/unifp_go2d1_mech_lever_measured_model_21496.pt
./demos/insertion/run_demo.py --checkpoint checkpoints/unifp_go2d1_insertion_seed3_measured_model_21400.pt \
    --compliant_key --attempts 48 --num_envs 16 --headless --lateral_mm 2
```

One seed each, simulation only. None of the three was checked against a control continued on the old robot, so the
part of each gain due to extra training, rather than the robot, is not known.

## On Unitree's base (before 2026-09-30)

| file | trained on | best measured | falls |
| --- | --- | --- | --- |
| `unifp_go2d1_isaaclab_model_56000.pt` | Isaac Lab 0.54.3 / Isaac Sim 5.1, this repository's `unifp_train` | **1.5 cm** median tool-tip error | 0/50 |
| `unifp_go2d1_isaacgym_model_48800.pt` | Isaac Gym Preview 4 / legged_gym, via `unifp_go2d1/` | 3.9 cm median tool-tip error | 0/50 |
| `unifp_go2d1_mech_law_model_17499.pt` | Isaac Lab 0.54.3 / Isaac Sim 5.1, `unifp_train --task mechanism --force_law` | 504/512 held mechanisms opened to 80 N (F-108); the standing combiner demo 16/16 to a 16 N·m door closer (F-110) | 0 |
| `unifp_go2d1_mech_lever_model_20497.pt` | the same, continued on short levers turned down, high handles and a price on force across the path | combiner with the L-lip claw from the hinge side: 12/16 at a 1.2 N·m lever (17499: 2), 9/16 with a 16 N·m door as well (17499 at its best stance: 4); loses the latch side and the button (F-114) | 3 in 864 demo attempts |
| `unifp_go2d1_insertion_model_20200.pt` | the same simulator, `unifp_train --task insertion` with the compliant grip and the task layer's outer loops (v7) | a key into a 1 mm keyed slot: 55/96 frozen slots in the training plant; in PhysX with the compliant key 13-14/16 at estimate errors of 0-4 mm (F-120) | 0 in 256 (frozen sets and demo) |
| `unifp_go2d1_insertion_model_21400.pt` | the same, continued (v9, v10) with the learning rate held at 1e-5, a slower curriculum and stillness priced only across the slot's axis | 71/96 frozen lateral slots and 71/96 detent slots in the plant; in PhysX 40/48, 36/48 and 83/96 at a 2 mm estimate error (F-121); as good a second | 0 in 384 |
| `unifp_go2d1_insertion_seed3_model_21200.pt` | the recipe from v5 with seed 3 (400 iterations), continued 800 at the learning rate of 1e-5 and the slower curriculum | **the insertion policy to use**: 75/96 frozen lateral slots (1 damaged), 71/96 detent slots; in PhysX 43/48, 43/48 and 80/96 on three sets of draws, no damage (F-122) | 0 in 384 |

Both figures are from the same 50 frozen episodes in the same simulator
(`results/manifests/unifp_isaaclab_validation.json`, content `94e576a6…`), forces active, with
every episode's schedule digest reproduced. See [F-092](../results/findings.md) for the comparison
and [F-093](../results/findings.md) for the one region where both are weak — goals low and in
front track at about three times the error of the rest of the workspace.

## Which to use

`unifp_go2d1_isaaclab_model_56000.pt`. It is better on 50 of 50 paired episodes and it is steady:
its tool-tip error varies by 0.4 cm over time against the Isaac Gym policy's 1.8 cm. The Isaac Gym
checkpoint is kept as an independent reference trained on a different simulator, which is what
makes the comparison checkable rather than self-reported.

Do not pick a checkpoint by training return. On this task the return cannot distinguish a 1.5 cm
policy from a 60 cm one (F-090, reconfirmed in F-092); selection is by frozen-manifest evaluation.

## Running one on another machine

Everything needed is in this repository except the simulator and the Go2 mesh, which is fetched
from Isaac Sim's asset server. `generated/` is gitignored and the welded USD is rebuilt from
`d1_arm/d1.urdf` on first run, so there is nothing else to copy.

> **Since 2026-09-24 the task controls a different point.** New runs put the controlled point at
> the jaw centre (`Link6 + (0, 0, 0.1051)`), which does not move when the jaws open and lies on the
> roll axis (F-094, F-096, F-099). These checkpoints were trained on the Link7_1 **fingertip**, and
> the two points are 2.4 cm apart, so **the table above only reproduces with
> `--tool_point fingertip`**. Without it the same policy on the same frozen set reads 3.5 cm rather
> than 1.5, the manifest reports a condition mismatch, and both are correct — it is a different
> measurement, not a regression.

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

# watch it, with UniFP's overlay: goal, force-displaced goal, tool tip, trajectory, force arrows
./run_unifp_train.py play --num_envs 4 --steps 200000 --seed 42 --force_start_iteration 0 \
    --checkpoint checkpoints/unifp_go2d1_isaaclab_model_56000.pt

# score it on the frozen set -- reproduces the table above
./run_unifp_train.py eval --headless --tool_point fingertip \
    --manifest results/manifests/unifp_isaaclab_validation.json \
    --checkpoint checkpoints/unifp_go2d1_isaaclab_model_56000.pt

# tracking error as a function of where the goal is
./unifp_train/workspace_map.py --headless \
    --checkpoint checkpoints/unifp_go2d1_isaaclab_model_56000.pt
```

A different GPU will not reproduce the numbers bit for bit — PhysX is not deterministic across
devices — but the manifest re-checks every episode's schedule digest and the environment
conditions on each run, so a machine that has drifted says so rather than quietly reporting
different numbers.

## The mechanism policy: `unifp_go2d1_mech_law_model_17499.pt`

A different controller from the two above, not a better copy of them. It holds a handle and is given the goal
plus a force command from a task layer's PI law on how far the handle lags its reference (F-106, F-108); it was
trained on invented mechanisms (slides and hinges with springs, stiction and snapping latches), the jaw-centre
tool point, the roll objective, and **0.01 kg·m² of arm armature** (F-102) — run it with anything else and it is
not the policy that was measured. Given only a goal, with no force command, it gives way (F-105, F-110).

```bash
# the standing combiner demo, as it is meant to be run (claw, force law, roll command, goal correction)
./demos/unifp/run_demo.py --task combiner --mech --num_envs 1 --attempts 1 --door_torque_nm 8
./demos/unifp/run_demo.py --task combiner --mech --attempts 16 --headless --door_torque_nm 16

# the mechanism evaluation it was scored on: held-out test set, the law with its integral bleed
./run_unifp_train.py mech_eval --task mechanism --force_law --force_law_bleed 1.0 --mech_test --headless \
    --checkpoint checkpoints/unifp_go2d1_mech_law_model_17499.pt
```

It is `logs/unifp_train/20260924T155754_train_seed1_mech_law_v1/model_17499.pt`, byte for byte (`SHA256SUMS`),
the end of a chain on this task — pull task v4 `model_13000` (F-103) → `mech_v1` → `v2` → `v3` → this, ~4,350
iterations on mechanisms, one seed; the runs are under `results/week_02/runs/` (`*_mech_*`). Simulation only; the
arm's motors run at their limits while it pulls, which a real D1 may not tolerate.

## The lever-trained mechanism policy: `unifp_go2d1_mech_lever_model_20497.pt`

`unifp_go2d1_mech_law_model_17499.pt` continued 3,000 iterations (F-114): 1,500 with 30% of mechanisms short levers
turned down about a spindle pointing at the robot (`--lever_fraction 0.3`), then 1,500 more with lever handles up to
~0.96 m (`--lever_high`) and a price on force across the mechanism's path (`--offpath_weight 2.0`), which a claw cannot
hold. It is `logs/unifp_train/20260925T111115_train_seed1_mech_law_lever_high/model_20497.pt`, byte for byte.

Not a replacement for `17499`: it is better on levers square to the box and from the door's hinge side, worse from
the latch side, and it lost the button in the held-out set (88/128 against 119). It also reaches in a posture that
needs the demo's goal correction kept on through the entry, `--correct_entry`; without it the lip claw misses the
bar. The combination it was measured best in:

```bash
./demos/unifp/run_demo.py --task combiner --mech --num_envs 1 --attempts 1 --correct_entry --stance_deg -30 \
    --handle_torque_nm 1.2 --door_torque_nm 16 --checkpoint checkpoints/unifp_go2d1_mech_lever_model_20497.pt
```

## The insertion policies

**Use `unifp_go2d1_insertion_seed3_model_21200.pt`.** It is `logs/unifp_train/20260926T214320_train_seed3_ins_recipe_seed3_cont/model_21200.pt`, byte for byte: the recipe run from
v5 with seed 3 for 400 iterations, then continued as v9 was (learning rate 1e-5, a promotion at most every 200
iterations), and chosen as the best of that continuation's evaluated checkpoints (F-122). The two below are kept
because F-120 and F-121 are about them.

### `unifp_go2d1_insertion_model_21400.pt`

`model_21400` is `logs/unifp_train/20260926T170707_train_seed1_ins_v10/model_21400.pt`, byte for byte: `model_20200`
continued 800 iterations as v9 (to its `model_21000`) and 400 more as v10, the learning rate held at rsl-rl's 1e-5 so
the resume does not throw it, the curriculum promoted at most every 200 iterations, and the stillness term priced only
across the slot's axis once the key is going in. It was chosen as the best of the checkpoints evaluated on the frozen
sets. Later ones decline (F-121), so do not take a later checkpoint of the same run on trust. It descends from seed
1, the best of three seeds of the recipe: seeds 2 and 3 reached 36 and 32 of 96 in the same 400 iterations, against 28
for the untrained policy under the same task layer (F-120's note). Everything below about
the task layer and the grip holds for it too. `model_20200` is kept because F-120's evidence is about it.

### `unifp_go2d1_insertion_model_20200.pt`

The mechanism lineage continued on key insertion (`--task insertion`, F-116, F-119, F-120): v5 `model_19998`, then 200
iterations of v7 with a compliant grip on the key, the task layer's outer loops on the measured tip, elevation and
roll, the arm's action noise cut to 0.15, and an easier slot below curriculum level 1. It is
`logs/unifp_train/20260926T141624_train_seed1_ins_v7/model_20200.pt`, byte for byte.

**It only works inside its task layer, holding the key compliantly.** With the loops off it starts 15 of 96 slots and
seats none, because the roll loop carries its roll. With a rigid key the training plant damages 86 of 96, and PhysX
seats 5 of 16. The grip it assumes is 2 N·m/rad about the jaw centre, ±10°: a hardware requirement, not yet built.

```bash
./demos/insertion/run_demo.py --checkpoint checkpoints/unifp_go2d1_insertion_seed3_model_21200.pt --compliant_key \
    --num_envs 1 --attempts 1 --lateral_mm 2
```

## The rope-pull policies

### `unifp_go2d1_rope_cad333_model_14600.pt`

For the robot as modelled since 2026-10-09 (CAD lengths, a 3.33 N·m shoulder and elbow). It is
`logs/unifp_train/20261009T172054_train_seed1_rope_v3_c33/model_14600.pt`, byte for byte: hook v4 `model_13000`
continued 1,600 iterations on `--task rope` with the v3 recipe and `--curriculum_rule ema`, on that robot. It was chosen
by the rope rule (falls, then episodes with any tug, then force error, then holds) over all ten saved checkpoints on the
development set: no falls in any checkpoint, no tug in 108 episodes, 25.1 / 35.0 / 48.9 N for 20 / 35 / 50.

On the held-out set, run once: 21.9 / 31.9 / 45.2 N for 15 / 30 / 45 N, all 576 holds held, and no fall in 192 releases.
The polish below, unretrained on this robot, over-pulls by ~6 N at every level. Polishing it on this robot brought the
sinking after unnoticed snaps back (2–19 of 36, three seeds), so it was not used. See the CAD-robot section above for the
posture and the hardware caveat.

### On the measured robot, 1.7 N·m elbow

**Use `unifp_go2d1_rope_v3_polish_model_14600.pt`** (F-136): `unifp_go2d1_rope_v3_model_14400.pt`, below, continued 200
iterations at half the exploration noise. It is `logs/unifp_train/20261007T152825_train_seed1_rope_v3_polish/model_14600.pt`,
byte for byte. On the held-out rope set it pulls 17.4 / 30.0 / 44.3 N for 15 / 30 / 45 N (std 0.04–0.11 N, no tug at 30
or 45 N) and falls in none of 96 releases. Same commands as below, with this file.

### `unifp_go2d1_rope_v3_model_14400.pt`

Trained on the measured robot (2026-10-07) for Lukas's question: a steady pull on a rope with the arm taut along it and
the legs making the force, and no fall when the rope suddenly gives. It is
`logs/unifp_train/20261007T080223_train_seed1_rope_v3/model_14400.pt`, byte for byte: hook v4 `model_13000` (F-103)
continued 1,400 iterations on `--task rope` with the v3 recipe (`unifp_train/README.md`, "The rope-pull task"), chosen
by a rule written before the last checkpoints were scored (F-133). Run it with `--task rope`, which also sets the arm's
0.01 kg·m² armature (F-102).

```bash
./run_unifp_train.py rope_eval --task rope --headless --repeats 2 --checkpoint checkpoints/unifp_go2d1_rope_v3_model_14400.pt
./run_unifp_train.py rope_eval --task rope --headless --rope_set test --checkpoint checkpoints/unifp_go2d1_rope_v3_model_14400.pt
./run_unifp_train.py play --task rope --num_envs 4 --steps 3000 --checkpoint checkpoints/unifp_go2d1_rope_v3_model_14400.pt
```

On the held-out rope set it holds 15 / 30 / 45 N in 96 of 96 episodes each, with the tension's std 0.03–0.28 N, and
no fall in 96 releases. It pulls 5–7 N over the command and still snatches once as the rope comes taut (less on a
10 N/s ramp). It runs Joint2 and Joint3 at their torque limits, as every pull policy here has: what a real D1 does
under that is unmeasured. Simulation only.

## Provenance

`unifp_go2d1_isaacgym_model_48800.pt` was trained by this project using UniFP's unmodified method
on the retargeted Go2+D1 robot, through the port in `unifp_go2d1/`. UniFP is BSD 3-Clause; see
[third_party/UniFP/NOTICE.md](../third_party/UniFP/NOTICE.md). The weights are this project's
output, not a redistribution of anything from upstream — UniFP publishes no Go2+D1 checkpoint.

`unifp_go2d1_isaaclab_model_56000.pt` came from the 60,000-iteration run of 2026-09-21, UniFP's
configuration unchanged, 4,096 environments, seed 1, 43.5 h. Its full training trace is committed
under `results/week_02/runs/20260921T121141_train_seed1_full_after_f080/`.

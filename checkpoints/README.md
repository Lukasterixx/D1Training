# Trained policies

The two UniFP position/force controllers for the welded Go2 + D1, kept here rather than under
`logs/` because `logs/` is gitignored and these are deliverables: they are what you copy to
another machine to run the task.

Both drive the **same** 18-action interface and load through the same code path
(`unifp_train.eval.EvaluablePolicy`), which reads UniFP's single-module layout and rsl-rl 5.x's
split-actor layout alike. Verify what you copied against `SHA256SUMS`.

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

## Provenance

`unifp_go2d1_isaacgym_model_48800.pt` was trained by this project using UniFP's unmodified method
on the retargeted Go2+D1 robot, through the port in `unifp_go2d1/`. UniFP is BSD 3-Clause; see
[third_party/UniFP/NOTICE.md](../third_party/UniFP/NOTICE.md). The weights are this project's
output, not a redistribution of anything from upstream — UniFP publishes no Go2+D1 checkpoint.

`unifp_go2d1_isaaclab_model_56000.pt` came from the 60,000-iteration run of 2026-09-21, UniFP's
configuration unchanged, 4,096 environments, seed 1, 43.5 h. Its full training trace is committed
under `results/week_02/runs/20260921T121141_train_seed1_full_after_f080/`.

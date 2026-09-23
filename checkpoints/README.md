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

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

# watch it, with UniFP's overlay: goal, force-displaced goal, tool tip, trajectory, force arrows
./run_unifp_train.py play --num_envs 4 --steps 200000 --seed 42 --force_start_iteration 0 \
    --checkpoint checkpoints/unifp_go2d1_isaaclab_model_56000.pt

# score it on the frozen set -- reproduces the table above
./run_unifp_train.py eval --headless \
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

## Provenance

`unifp_go2d1_isaacgym_model_48800.pt` was trained by this project using UniFP's unmodified method
on the retargeted Go2+D1 robot, through the port in `unifp_go2d1/`. UniFP is BSD 3-Clause; see
[third_party/UniFP/NOTICE.md](../third_party/UniFP/NOTICE.md). The weights are this project's
output, not a redistribution of anything from upstream — UniFP publishes no Go2+D1 checkpoint.

`unifp_go2d1_isaaclab_model_56000.pt` came from the 60,000-iteration run of 2026-09-21, UniFP's
configuration unchanged, 4,096 environments, seed 1, 43.5 h. Its full training trace is committed
under `results/week_02/runs/20260921T121141_train_seed1_full_after_f080/`.

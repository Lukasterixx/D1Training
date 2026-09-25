# unifp_go2d1 — UniFP's position/force task on the Go2 + D1

UniFP's released B2Z1 whole-body position/force environment, retargeted to this repository's
welded Go2 + D1. It is the "reproduce a bounded example, then adapt it" step the
[Thesis B plan](../docs/thesis_b_plan.md) asks for before H1 work, and it runs on a separate
legacy stack (Isaac Gym Preview 4, Python 3.8) that must not touch the shared Isaac Lab
installation.

**Not validated.** As of the first port there is one thing established: the environment builds
and PPO optimises on it. Nothing here has produced a trained policy, been evaluated against the
frozen manifest, or been compared with hardware. Every physical number below is either taken
from this repository's Isaac Lab model or scaled from upstream by hand.

## Layout

| File | Role |
| --- | --- |
| `build_asset.py` | generates `resources/robots/go2d1/go2d1.urdf` + meshes in the UniFP clone |
| `go2d1_pos_force_config.py` | the task config, copied into the clone as-is |
| `port_env.py` | generates the environment from UniFP's, by asserted substitution |
| `launch_training.py` | training entry point, plus the wall-clock knobs |
| `install.sh` | runs all of the above against a UniFP checkout, and registers the task |
| `run_training.sh` | launches a detached training run |
| `supervise_training.sh` | watchdog: resumes from the newest checkpoint after a crash or hang |
| `find_checkpoint.py` | newest checkpoint across every run directory of an experiment |
| `stop_training.sh` | stops training and supervisor deliberately (leaves the `STOP` file) |
| `progress.py`, `watch_progress.sh` | live status readout: progress, ETA, curriculum, GPU, health |
| `play_policy.py` | watch or trace a trained policy, with forces on and the camera following |
| `eval_manifest.py` | frozen evaluation manifests: build, load, hash, condition check |
| `evaluate.py` | run a manifest's episodes and summarise what the policy did |
| `run_eval.py` | CLI for both: `build` a manifest, `eval` a checkpoint or the baseline |
| `run_metadata.py` | writes the `run.json` the evidence record reads |

Nothing is edited inside the clone by hand. `install.sh` is idempotent — change a file here and
re-run it.

## Setup, from nothing

```bash
conda create -n unifp python=3.8 -y && conda activate unifp
pip install "numpy==1.23.5"
pip install torch==2.3.1 torchvision==0.18.1 --index-url https://download.pytorch.org/whl/cu121
pip install -e ~/thesis_b_legacy/isaacgym/python           # Isaac Gym Preview 4, extracted
pip install tensorboard pyyaml matplotlib wandb pydelatin scipy "setuptools<70"
pip install "params_proto==2.12.1"                          # 3.x is Python >= 3.9 only

git clone https://github.com/unified-force/UniFP.git ~/thesis_b_legacy/UniFP
cd ~/thesis_b_legacy/UniFP && git checkout 68847a070f88d731058c3d8476929bc3b205f5bd
~/D1Training/unifp_go2d1/install.sh
```

Every Isaac Gym shell also needs `export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH`
and `unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH`. **Run from the
UniFP root**: `cfg.asset.file` is a relative path, so running from `legged_gym/scripts` fails
to load the robot and then dies on a missing body name, which is not what the error says.

Two things upstream needs that its README does not say: `params_proto` must be pinned below
3.0, and `wandb.init` is called unconditionally, so set `WANDB_MODE=disabled` (or log in)
before training. TensorBoard logging is unaffected.

## Training

```bash
cd ~/thesis_b_legacy/UniFP
python ~/D1Training/unifp_go2d1/launch_training.py --tf32 --no-wandb \
    --task=go2d1_pos_force --num_envs 4096 --headless
```

Checkpoints land in `~/thesis_b_legacy/UniFP/logs/go2d1_pos_force/<date>_/` every 200
iterations, so the run can be stopped at any point and the last checkpoint is still usable.
Resume with `--resume --load_run <dir> --checkpoint <n>`.

Measured on the RTX 3500 Ada (12 GB), 4096 environments: 2.44 s/iteration with `--tf32`
(3.28 s without), so upstream's default 60,000-iteration schedule is about 41 hours. Forces do
not start until iteration 8,000 (`commands.force_start_step`), about 5.5 hours in; everything
before that is position tracking and locomotion.

## Surviving a crash

A 42-hour run will not be watched. `supervise_training.sh` watches it instead:

```bash
setsid nohup ./unifp_go2d1/supervise_training.sh --attach <training pid> &   # watch a running job
setsid nohup ./unifp_go2d1/supervise_training.sh &                          # or start and watch one
./unifp_go2d1/stop_training.sh                                              # stop both, on purpose
tail -f ~/thesis_b_legacy/UniFP/logs/supervisor.log
```

## Watching it

```bash
./unifp_go2d1/watch_progress.sh          # a desktop terminal window if there is a display,
                                         # otherwise right here -- so it works over SSH too
./unifp_go2d1/watch_progress.sh --here   # always in this terminal
./unifp_go2d1/watch_progress.sh --once   # print once and exit
```

It reports progress against the **checkpoints**, not the log, so it stays correct across a resume
(the log file changes, the checkpoint numbering does not), and it reads whether the force
curriculum has started, whether the supervisor is watching, how many resumes have happened, and
whether the process is alive but quiet. It writes nothing, so it is safe to run as often as you
like.

The desktop window is launched with a scrubbed environment deliberately: with ROS or a conda env
sourced, `LD_LIBRARY_PATH` puts snap libraries ahead of the system ones and `gnome-terminal` dies
on a symbol lookup in `/snap/core20` instead of opening.

It resumes from the newest checkpoint across every run directory, restarts a job that has hung
without writing a checkpoint for an hour, falls back one checkpoint if a restart dies without
progress, and gives up after five consecutive failures rather than burning the GPU in a crash
loop. `stop_training.sh` leaves a `STOP` file, which is how the supervisor tells a deliberate
stop from a crash — delete it before starting again. Neither survives a reboot; that would need a
systemd user unit.

**Resuming UniFP correctly needs two corrections, both in `launch_training.py`.** They are easy to
miss because nothing errors without them:

- `learn(num_learning_iterations=N)` runs `current_learning_iteration + N`, so `--max_iterations`
  on a resume is an amount to **add**, not a target. The supervisor asks for exactly the
  iterations still owed; the launcher prints where the run will actually end.
- `env.global_steps` is zeroed by `_init_buffers` on every launch, and it is what gates the force
  curriculum (`global_steps > force_start_step * num_steps_per_env`). Left alone, a resume at
  iteration 20,000 drops **silently** back to position-only training for another 8,000 iterations.
  The launcher sets it from the resumed iteration and says which side of the curriculum the run
  is on. Measured: with the gate shut no force is applied (0.000 N peak), past it forces are
  (0.394 N peak in 12 steps).

`--force-start-step N` overrides the curriculum start. It changes the experiment — upstream's
value is 8000 — so the launcher prints it as a deviation and `run.json` records it.

`--tf32` is the only wall-clock knob that is not also a change to the experiment. It is a
reduced-precision arithmetic change, so a TF32 run is statistically equivalent to an FP32 one
but not bit-identical — record which was used. Everything else that would shorten a run
(fewer environments, fewer iterations, an earlier force curriculum, smaller terrain) changes
what is being trained and belongs in the log as a deviation.

## Watching a trained policy

```bash
python unifp_go2d1/play_policy.py --task=go2d1_pos_force \
    --load_run <run> --checkpoint 48800 --forces
```

Upstream's `play_*.py` works, but shows a force policy **with the forces off**: the curriculum is
gated on `env.global_steps`, which starts at 0 in a fresh process, so a play session never reaches
`force_start_step` and the robot is never pushed — the same defect as F-082, which was fixed for
resumed training but lives in the play path too. `--forces` winds `global_steps` past the gate, the
camera follows the robot, and `--out DIR` writes a `run.json` and `trace.csv` so a rollout can be
recorded with `./dashboard.py record`.

## Evaluating a policy

```bash
# freeze a set of episodes, recording the schedule each one receives
python unifp_go2d1/run_eval.py build --role development --episodes 50 --seed 20260920 \
    --task=go2d1_pos_force --headless

# the baseline and a checkpoint on the same frozen set
python unifp_go2d1/run_eval.py eval --manifest results/manifests/unifp_development.json \
    --zero --task=go2d1_pos_force --headless
python unifp_go2d1/run_eval.py eval --manifest results/manifests/unifp_development.json \
    --load_run <run> --checkpoint 48800 --task=go2d1_pos_force --headless
```

An episode here cannot be stated the way the Isaac Lab task's can: this task *generates* its
schedule as it runs (velocity commands on a timer, a goal trajectory, force pushes on their own
intervals) through dozens of random draws. So the set is frozen by one seed and one environment
count, and each episode carries a **digest of the schedule it actually received**, recomputed on
every run — `schedule_mismatches` in the result is the check that the episodes are the same
episodes. Two consequences worth knowing: episodes run in parallel, one per environment, so the
environment count is part of the frozen conditions (the draws are batched); and termination is
recorded but not acted on, so one episode's fall cannot shift the schedule of the next.

The physics is **not** bitwise reproducible — GPU PhysX is not deterministic, and the same
controller on the same manifest gives fall counts that differ by one and medians that move by
about ±0.1 cm. Repeat any comparison finer than that. Results in F-086.

## What the task rewards

The released config sets `tracking_ee_sphere = 0` and puts the weight on
`tracking_ee_force_world`. That is not a position-free task: the term rewards the tool tip for
reaching `goal + (measured force + commanded force) / k`, so with no force it is EE position
tracking, and with a force command it asks for that force through a virtual stiffness.
`tracking_lin_vel_force_world` does the same for base velocity against base force. This is the
"unified" formulation, and it is why a position-only ablation means removing force inputs,
force latents and force objectives — not setting the force command to zero.

## What differs from upstream, and why

| | B2Z1 (upstream) | Go2+D1 (here) |
| --- | --- | --- |
| DOFs / actions | 19 / 17 (12 legs + 5 arm; wrist roll and gripper held) | 20 / 18 (12 legs + 6 arm; two jaws held) |
| Single obs / privileged obs | 73 / 149 | 76 / 153 |
| Leg gains | 300–500 / 7.5–12.5 | 25 / 0.5 |
| Arm gains | 64–128 / 1.5–3.0 | 40–60 / 0.8–1.0 |
| Held-joint gains | 64 N·m/rad, 1.5 | 800 N/m, 8 (prismatic jaws) |
| Base height target | 0.50 m | 0.30 m |
| EE goal sphere | centre +0.2 m x, 0.8 m z; r ∈ [0.35, 0.95] | centre 0 x, 0.49 m z; r ∈ [0.30, 0.58] |
| **EE force command** | **±60 N** | **±8 N** |
| Base force command | ±50 N | ±20 N |
| Added base mass | 0–15 kg | 0–3 kg |

The force range is the change that matters most. The D1's strongest joint is 3.3 N·m, which is
about 7 N at a 0.45 m moment arm, so upstream's ±60 N is an instruction this arm can only fail.
Force tracking on this robot is a few-newton problem, and any comparison against UniFP's
reported numbers has to say so.

*2026-09-24 (F-103):* true of UniFP's task — a force in any direction at wherever the goal is — and
not of a force into a fixture the whole body can lean on. Trained for that (`unifp_train/hook_env.py`),
the same architecture pulls 60 N in simulation by lining the arm up with the pull and leaning. The
7 N figure is the arm alone in a bent reach.

Three mechanical changes were forced by Isaac Gym rather than chosen:

- **Arm joints are renamed `Joint<n>` → `d1_Joint<n>`** in the generated URDF. Isaac Gym orders
  DOFs by walking the base link's child subtrees in alphabetical order of the joint that starts
  each one, so `Joint1` placed the arm between the front and rear legs. The prefix sorts it
  after `RR_hip_joint` and restores the [12 legs | 6 arm | 2 gripper] layout the environment
  slices by index. Link and body names are untouched.
- **The mass model is rebuilt into the URDF.** `description/go2_d1.urdf` drops every
  `<inertial>` on purpose (it is a drawing; `weld.py` owns the mass model). Isaac Gym then
  derives mass from collision geometry × `density = 0.005`, which NaNs the articulation on the
  first step. `build_asset.py` reassembles the same model `weld.py` builds — Go2 inertials from
  the Go2 description, D1 shells from `d1_arm/d1.urdf`, servo masses, and the remainder of
  ARM_MASS_KG on the arm base — and the loaded articulation weighs 18.171 kg before
  randomisation, with 2.165 kg at the arm base and 0.987 kg of moving arm.
- **The arm's visual meshes are pre-rotated.** `flip_visual_attachments` is one flag per asset, but
  the Go2's `.dae` meshes are y-up (they need it) and the D1's `.STL` meshes are z-up (they must not
  have it), so neither setting renders both halves correctly. `build_asset.py` writes a `_visflip`
  copy of each arm mesh rotated by Rx(−90°) and points only `<visual>` at it; `<collision>` keeps
  the original. Measured to be visual-only: identical masses and every body within 0.0000 mm after a
  2 s settle either way (F-084). Worth re-checking the first time this model is rendered anywhere
  new — nothing errors, and the numbers stay right while the picture is wrong.
- **`ee_gripper_link`** is added at the CAD pincer tip (`position_only/tool_point.py`), because
  UniFP indexes the controlled point and applies the EE force by body name. It is the same
  controlled point as the Isaac Lab position-only task: CAD, not measured on the arm (F-013).

Two upstream details worth knowing: `_push_robot_base` is commented out in the released
`step()`, so base forces are configured but never applied; and `randomize_leg_mass` is off, so
the 17-wide leg-mass block of the privileged observation is always zeros.

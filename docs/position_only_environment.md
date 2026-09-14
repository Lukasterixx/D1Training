# Position-only starter environment

Status: initial implementation, **not yet validated in Isaac/PhysX**.
The GPU prerequisite check passes (14 September 2026, second session), but no
simulator run has been made yet. Progress is tracked in the
[Week 1 record](../results/week_01/notes.md). This task begins the free-space stage of the
[Thesis B plan](thesis_b_plan.md); it is not a trained P0 controller or a
contact-task benchmark.

## Implemented

- Flat-ground, floating-base Go2 with welded D1; existing robot factory and
  mass model reused. Training assets rebuild inside each run directory.
- Eighteen policy actions: 12 named leg joints followed by `Joint1`–`Joint6`.
  Jaw joints remain at their closed initial target. No IK controller overwrites
  policy actions. Targets are clipped to soft joint limits.
- One static target per 10 s episode. Targets are sampled relative to each
  environment origin and remain fixed in world coordinates. The actor receives
  the target and measured interaction point in the **current base frame**.
- Asymmetric actor-critic, following unitree_rl_lab. Actor, 66 scalars: base
  angular velocity, projected gravity, 18 joint positions and velocities,
  target position, measured point position, previous 18 actions. There is no
  base linear velocity, because the Go2 has no sensor for it. Critic, 87
  scalars: those inputs, noise-free, plus base linear velocity and 18 joint
  torques. No force commands, force measurements, force reward or tool
  descriptors in either.
- Go2 legs use unitree_rl_lab's measured motor envelope by default
  (`--leg_actuator unitree`: explicit 25/0.5 PD; 20.2 N·m driving and 23.4 N·m
  braking up to 13.5 rad/s, falling to zero at 30 rad/s). `--leg_actuator
  dc_motor` restores Isaac Lab's stock model for comparison. The arm stays on
  implicit 4000/400 drives: no D1 servo curve is published, and an explicit
  PD that stiff is not stable at 200 Hz. Solver iterations are 8 position / 4
  velocity (the stock Go2 uses 4/0).
- `--robustness unitree` adds unitree_rl_lab's observation noise and
  randomisation: friction 0.3–1.2, restitution 0–0.15, base mass −1 to +3 kg,
  base CoM offset, and ±0.5 m/s pushes every 5–10 s. The default `none` stays
  deterministic for bring-up. `--self_collisions` lets the arm hit the Go2 body;
  it is off until the weld has been checked for overlapping collision shapes.
- Every run writes `params/deploy.yaml`
  ([`deploy.py`](../position_only/deploy.py)): policy output index → motor id on
  the Go2 or D1 bus, action scale/offset/clip and limits, PD gains, actuator
  models and the actor observation layout. This is the contract a robot-side
  controller must reproduce.
- Position tracking, uprightness, motion/action regularisation and failures;
  timeout, low base, excessive tilt, base contact and workspace exit resets.
- Separate contact sensor paths for the base and nested D1 links. Arm forces
  are diagnostic data, excluded from the policy. Sensor coverage is asserted
  during launch; force calibration remains a GPU validation task.
- Fresh stochastic PPO configuration for RSL-RL 5.x; explicit checkpoint
  selection; per-run configuration, source snapshots, model hashes and logs.

Code: [`env_cfg.py`](../position_only/env_cfg.py),
[`mdp.py`](../position_only/mdp.py), [`agent.py`](../position_only/agent.py),
[`deploy.py`](../position_only/deploy.py),
[`run_position_only.py`](../run_position_only.py),
[`motor_model.py`](../motor_model.py) and
[`unitree_actuators.py`](../unitree_actuators.py). The last three adapt Apache-2.0
unitree_rl_lab code; see [third_party/unitree_rl_lab](../third_party/unitree_rl_lab/NOTICE.md).

## Run sequence

From the repository root, in a clean shell with the existing Isaac environment
activated. Avoid sourcing system ROS into this Python process.

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

python run_position_only.py check
python run_position_only.py smoke --num_envs 1 --steps 600
python run_position_only.py smoke --headless --num_envs 4 --steps 600
python run_position_only.py train --headless --num_envs 64 --iterations 100 --seed 42
```

`check` returns nonzero if GPU prerequisites are unavailable. It does not start
Isaac or install/change dependencies. Passing it establishes prerequisites,
not that asset downloads, rendering, cloning or physics will work. The first
asset build may require NVIDIA cloud access; `--robot_usd /absolute/path/model.usd`
can reuse an existing welded model with all of its references resolvable.

Add `--leg_actuator dc_motor`, `--robustness unitree` or `--self_collisions` to
change the model; each choice is recorded under `sim2real` in `run.json`. Run the
first smoke test with the defaults, then repeat it with `--leg_actuator dc_motor`
so any change in standing behaviour can be attributed to the motor model.

`smoke` uses zero policy actions, meaning default joint targets. It checks
dimensions, action order, arm sensor bodies, finite observations/rewards and
stepping with automatic resets. It reports failure/time-limit counts and final
position errors as diagnostics. Reaching success is not expected from zero
actions. Inspect the visible run for self-collision, posture and target location;
then deliberately test each failure/reset condition before the PPO pilot.

The short `train` command uses 153,600 transitions. Training starts fresh unless
`--checkpoint` is supplied. After a successful pilot, use the **exact new task
checkpoint path** for playback or resumption:

```bash
python run_position_only.py smoke --num_envs 1 --steps 600 --checkpoint /path/to/position_only/model_99.pt
python run_position_only.py train --headless --num_envs 64 --iterations 100 --checkpoint /path/to/position_only/model_99.pt
```

`--iterations` means additional iterations when resuming. Do not use the old
12-action locomotion checkpoint. Preserve the matching task config and
`--tip_offset` when loading; shape compatibility alone does not establish
behavioural compatibility.

Output goes to `logs/position_only/<UTC timestamp>_<mode>_seed<seed>/`, ignored
by Git. Record each run as evidence with
`./dashboard.py record logs/position_only/<run> --title "..."` (see
[results/README.md](../results/README.md)). `run.json` records completion/failure, versions, seed, action order,
source hashes/snapshot and USD/checkpoint hashes; `env.yaml` and `agent.json`
record configuration. Smoke mode adds `smoke.json`; training adds TensorBoard
events and checkpoints. Referenced cloud assets still need to be cached and
pinned for a final reproducible archive; the root USD hash alone is insufficient.

## Provisional choices to validate

| Choice | Current value | Required validation |
| --- | --- | --- |
| Interaction point | `Link6` origin, offset `(0, 0, 0)` | Register actual grasp/contact point; use `--tip_offset X Y Z` in Link6 coordinates, metres |
| Target box | x 0.24–0.36, y −0.08–0.08, z 0.66–0.78 m above each world environment origin | Collision-aware workspace and joint/effort feasibility; do not relabel proxy-frame errors as tool-tip errors |
| Arm action range | Default angles ±1 rad, also clamped to soft limits | Validate reachable workspace and extend through a documented curriculum |
| Physics/policy rate | 200/50 Hz | Timestep/solver convergence and actual D1 command-rate/latency model |
| Arm mass and PD gains | Existing 3.152 kg mass model; 4000/400 gains | Step/load response and sensitivity; no identified hardware equivalence claimed |
| Leg motor model | unitree_rl_lab Go2 envelope, explicit 25/0.5 PD, zero delay | Standing/stepping comparison with `dc_motor`; latency (`min_delay`/`max_delay`) left at zero as in unitree_rl_lab until measured |
| Randomisation | Deterministic by default; `--robustness unitree` for noise and randomisation | Enable after nominal reaching works; push size chosen for locomotion, so check it against the 0.75 m workspace termination |
| Self-collision | Off by default | Visible run with `--self_collisions`: no contact forces at rest from overlapping weld shapes |
| Reward widths | 15 cm coarse, 4 cm fine | Inspect learning and physical errors; widths are not success thresholds |

The free-space task has no orientation objective yet. It also lacks commanded
walking, trajectory interpolation, fixtures/tools, calibrated force logging,
arm contact failure rules and comprehensive self-collision monitoring. Those
belong to the subsequent stages in the plan. Base-motion regularisation and
the 0.75 m workspace boundary make this a local stance/reaching stage.

## Checks completed in the initial session

```bash
python -m unittest discover -s tests -v
python -m compileall -q position_only run_position_only.py tests
```

Five CPU tests pass: world-fixed target transform under base motion,
environment translation invariance, rotated interaction-point offset including
quaternion sign equivalence, and position-reward units/shape, plus a
synthetic CPU rollout exercising a real RSL-RL 5.0.1 PPO update with 18 actions:
losses/parameters remained finite and actor weights changed. This validates
algorithm configuration compatibility, **not robot learning**. CLI help and
prerequisite reporting were also checked.

Updated in the second session for the unitree_rl_lab changes. The PPO test now
uses 66 actor and 87 critic inputs and asserts each network's input width.
`tests/test_sim2real.py` checks the Go2 envelope, friction sign, the stock
`DCMotor` comparison and the deploy manifest's motor mapping. The ported
envelope matches the original unitree_rl_lab function exactly over an
801 × 1001 torque–speed grid. None of this runs PhysX. The actuator class,
new observation groups, randomisation terms and deploy export first execute
in the smoke test.

No simulator launch or training run is recorded as passing. The first session
could not see a GPU (`cuda_available=false`, no `/dev/nvidia*`). In the second
session the RTX 4080 was visible and `python run_position_only.py check`
exited 0. Smoke and pilot runs were deferred because another experiment queue
occupied the GPU.

## Next implementation after GPU smoke validation

Build a fixed episode-manifest evaluator that logs terminal measurements
**before automatic reset**, counts every failed trial, reports per-seed
RMS/percentile/dwell/success metrics, and rejects mismatched checkpoint/task
configuration. Add vectorised IK reference playback on the same cases. Then
add smooth trajectories and a calibrated spring-loaded pressing fixture.
Until this exists, `smoke.json` is an interface diagnostic, not thesis evidence.

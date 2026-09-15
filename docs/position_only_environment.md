# Position-only starter environment

Status: runs in Isaac Sim; **interface validated, reaching not**. On 15 September 2026 smoke runs, the
deliberate `verify` checks (F-009), three short PPO pilots and a workspace analysis ran. They fixed the arm drive
type (F-010) and showed that the target box starts next to the arm (F-011). Progress is tracked in the
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
  the target and measured interaction point in the **current base frame**. The
  interaction point is the tip of the Link7_1 pincer (`position_only/tool_point.py`).
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
  dc_motor` restores Isaac Lab's stock model for comparison. Solver iterations
  are 8 position / 4 velocity (the stock Go2 uses 4/0).
- D1 servo model (`--arm_actuator d1_servo`, default). Implicit 4000/400 **force**
  drives stand in for the D1's internal position loop. The URDF import authors
  acceleration drives, which PhysX scales by joint inertia. With those the arm
  sagged 0.11 rad at rest inside its torque limits (F-010), so `d1_servo` sets
  the drive type to force and `implicit` keeps the import for comparison. An explicit PD that stiff is
  not stable at 200 Hz, and Unitree publishes no D1 torque–speed curve for an
  envelope. Torque limits are set explicitly to Unitree's published 3.3/3.3/
  1.7/1.7/1.7/1.7 N·m. Speed limits are set to the URDF's 1.05 rad/s (Joints 1–3)
  and 1.73 rad/s (Joints 4–6); these are **unverified**. Unitree's own URDF has
  zeros, and the values were filled in during the Rescue port with no source.
  `run.json` records the limits PhysX actually applies (`arm_limits_in_physx`,
  `leg_limits_in_physx`) and the drive types on the stage (`drive_type_in_usd`).
- Interface timing (`--latency estimated`, default; `none` for exact 50 Hz
  interfaces). Go2 leg commands are delayed 0–2 physics steps (0–10 ms), drawn
  per episode by the Unitree actuator's delay buffer. That is an estimate from
  unitree_rl_lab's 1 kHz command loop, not a measurement. D1 arm targets reach
  the servos every 5 policy steps (10 Hz, the SDK's streaming rate), at a random
  phase per environment. The actor's arm joint angles come from 10 Hz samples,
  and its arm joint velocities are differenced from them, because the D1 reports
  angles only. The critic keeps exact state.
- `--robustness unitree` adds unitree_rl_lab's observation noise and
  randomisation: friction 0.3–1.2, restitution 0–0.15, base mass −1 to +3 kg,
  base CoM offset, and ±0.5 m/s pushes every 5–10 s. The default `none` stays
  deterministic for bring-up. Self-collisions are on by default, as in
  unitree_rl_lab (`--no-self_collisions` to disable). Week 1 measured no resting
  contact from the weld, and a positive control showed that with them off the arm
  passes through the trunk. Arm-to-body contact is not yet a termination or penalty.
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
python run_position_only.py smoke --headless --num_envs 1 --steps 600
python run_position_only.py smoke --num_envs 1 --steps 600          # visible: posture and target marker
python run_position_only.py verify --headless --num_envs 8
python run_position_only.py view --num_envs 16                     # real-time replay with target, tip and box markers
python run_position_only.py train --headless --num_envs 64 --iterations 100 --seed 42
python -m position_only.workspace --out results/week_NN/figures     # CPU: workspace, tool point, start distance
```

`check` returns nonzero if GPU prerequisites are unavailable. It does not start
Isaac or install/change dependencies. Passing it establishes prerequisites,
not that asset downloads, rendering, cloning or physics will work. The first
asset build may require NVIDIA cloud access; `--robot_usd /absolute/path/model.usd`
can reuse an existing welded model with all of its references resolvable.

Add `--leg_actuator dc_motor`, `--arm_actuator implicit`, `--latency none`,
`--robustness unitree` or `--no-self_collisions` to change the model; each choice is
recorded under `sim2real` in `run.json`, with the timing written into
`params/deploy.yaml`. Run the first smoke test with the defaults. If it fails or
behaves oddly, repeat with `--latency none --leg_actuator dc_motor --arm_actuator
implicit` (the pre-port configuration), then re-enable one change at a time.

`smoke` uses zero policy actions, meaning default joint targets. It checks
dimensions, action order, arm sensor bodies, finite observations/rewards and
stepping with automatic resets. It reports failure/time-limit counts and final
position errors as diagnostics, and writes `smoke_trace.csv`: base height, tilt,
drift, base x/y/yaw, leg and arm joint deviation, leg torque and arm contact force per step.
Reaching success is not expected from zero actions. Under zero actions the robot
drops from 0.42 m and settles at 0.266 m (Week 1).

`verify` needs at least 6 environments and writes `verify.json`. It checks the arm
command hold and feedback sampling, and the leg delays. It induces time limit, low base,
tilt, base contact and workspace exit, each in its own environment, and checks that only
that environment resets. It compares Link6 and controlled-point kinematics, link masses and the rest posture
with the CPU model in `position_only/workspace.py`, and commands the arm into the trunk
as a self-collision positive control. A pass is about those mechanisms only.

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
record configuration. Smoke mode adds `smoke.json` and `smoke_trace.csv`, verify
adds `verify.json`; training adds TensorBoard
events and checkpoints. Referenced cloud assets still need to be cached and
pinned for a final reproducible archive; the root USD hash alone is insufficient.

## Provisional choices to validate

| Choice | Current value | Required validation |
| --- | --- | --- |
| Interaction point | Link7_1 pincer tip: end-face centre `(0.0547, 0.0060, 0.0170)` m in Link7_1 coordinates (`tool_point.py`); `--tip_body`, `--tip_offset` | CAD-derived, not measured on the arm; matches the model to 0.9 µm in simulation. Grasping would need the point between the pincers. `--tip_body Link6 --tip_offset 0 0 0` is the pre-15-September point |
| Target box | x 0.24–0.36, y −0.08–0.08, z 0.66–0.78 m above each world environment origin | **Move before P0 counts** (F-013). At least 99.8% reachable for the pincer tip, but every reset slides the robot 7.4 cm back, carrying the tip into the box: zero actions meet 5 cm for 12.9% of targets (256 environments). Report a zero-action baseline with every reach metric |
| Arm action range | Default angles ±1 rad, also clamped to soft limits | Validate reachable workspace and extend through a documented curriculum |
| Physics/policy rate | 200/50 Hz | Timestep/solver convergence and actual D1 command-rate/latency model |
| Arm mass and PD gains | Existing 3.152 kg mass model (matches PhysX exactly); 4000/400 force-drive gains | Step/load response and sensitivity; J3 still rests 0.010 rad off (unexplained); no identified hardware equivalence claimed |
| Leg motor model | unitree_rl_lab Go2 envelope, explicit 25/0.5 PD | Standing matches `dc_motor` to 0.1 mm (Week 1), so the comparison needs motion. Both inherit the USD's unlabelled joint speed limits (30.1 / 15.7 rad/s) |
| Latency | Legs 0–2 physics steps; D1 commands and feedback 10 Hz, random phase | Measure Go2 command-to-motion delay and D1 command/feedback latency and smoothing on hardware (Thesis C); compare `--latency none` in training as a sensitivity run |
| D1 limits | Published torques; URDF speed limits (1.05/1.73 rad/s, unverified) | Measure joint speed. PhysX applies exactly these values (Week 1 smoke), with `d1_servo` and with `implicit` |
| Randomisation | Deterministic by default; `--robustness unitree` for noise and randomisation | Enable after nominal reaching works; push size chosen for locomotion, so check it against the 0.75 m workspace termination |
| Self-collision | On by default | No resting contact measured, and the positive control passes (Week 1). Arm-to-body contact is neither a termination nor a penalty yet |
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
`DCMotor` comparison, the latency profiles, 10 Hz sample-and-hold (phases,
differenced velocity, reset, repeated updates in one step) and the deploy
manifest's motor mapping and timing. 33 tests pass. The ported
envelope matches the original unitree_rl_lab function exactly over an
801 × 1001 torque–speed grid. None of this runs PhysX. The actuator class,
held arm action, sampled arm feedback, randomisation terms and deploy export
first execute in the smoke test.

Before 15 September no simulator launch or training run was recorded as passing. The first session
could not see a GPU (`cuda_available=false`, no `/dev/nvidia*`). In the second
session the RTX 4080 was visible and `python run_position_only.py check`
exited 0. Smoke and pilot runs were deferred because another experiment queue
occupied the GPU.

### Third session (15 September 2026)

First simulator runs, all recorded in the [Week 1 record](../results/week_01/notes.md):
- **Smoke:** 8 runs across the leg, arm, latency and self-collision options.
- **`verify`:** passes 20/20 with the defaults (F-009).
- **PPO pilots:** 64 environments for 100 iterations run at about 3,600–3,800 steps/s, adding 2.7–3.0 GB of GPU
  memory.
- **Workspace analysis:** covered by `tests/test_workspace.py`.

The runner now exits nonzero on failure; before, `SimulationApp.close()` hid exceptions behind exit status 0.
37 CPU tests pass.

## Next implementation after GPU smoke validation

Build a fixed episode-manifest evaluator that logs terminal measurements
**before automatic reset**, counts every failed trial, reports per-seed
RMS/percentile/dwell/success metrics next to a zero-action baseline, and rejects
mismatched checkpoint/task configuration. Add vectorised IK reference playback on the same cases. Then
add smooth trajectories and a calibrated spring-loaded pressing fixture.
Until this exists, `smoke.json` is an interface diagnostic, not thesis evidence.

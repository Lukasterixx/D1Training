# Week 1 — 14–20 September 2026

**Status:** in progress
**Focus:** Audit the model, runtime and candidate code; bring up the free-space stance-and-reach task

## Planned

From the [Thesis B plan](../../docs/thesis_b_plan.md#schedule-aligned-with-appendix-a) and the Appendix A Gantt chart.

- **Training and implementation:** audit model, runtime and candidate code; bring up the stance-and-reach task.
- **Testing and evidence:** record versions; verify welded articulation, frames, action order, resets and sensor
  coverage; capture baseline playback runs.
- **Gantt activities:** codebases, *testing*; position-only baseline, *check*; contact tasks, *build tasks + fixtures*.
- **Gate focus:** G0.

## Checklist

- [x] Thesis A timeline and experimental matrix reviewed into the plan
- [x] Codebase candidates reviewed at pinned revisions ([investigation](../../docs/codebase_investigation.md))
- [x] Position-only task scaffold: 18 actions, world-fixed targets, 69 observations, PPO config
- [x] CPU frame and PPO-API tests pass (F-003)
- [x] Results record and dashboard set up
- [x] `run_position_only.py check` passes on the GPU (2026-09-14, see log)
- [x] unitree_rl_lab (via MaiRo) reviewed; motor model, critic split, noise/randomisation and deploy manifest adopted (F-005, F-006)
- [ ] Headless smoke run, 1 environment (defaults: `--leg_actuator unitree`, `--robustness none`)
- [ ] Headless smoke run, 1 environment, `--leg_actuator dc_motor`: compare standing posture and failure resets
- [ ] `params/deploy.yaml` from the smoke run checked: motor ids, limits, 66-wide actor layout
- [ ] Headless smoke run, 4 environments
- [ ] Visible 1-environment run inspected: posture, target marker location (screenshot)
- [ ] Visible run with `--self_collisions`: no resting contact from overlapping weld shapes (screenshot)
- [ ] Deliberate reset tests: time limit, low base, tilt, base contact, workspace exit, partial reset
- [ ] End-effector frame registered: `Link6` origin versus the physical grasp point (`--tip_offset`)
- [ ] Target box checked against the reachable, collision-free workspace
- [ ] Playback reference: bare Go2 and Go2+D1, with forward, lateral and yaw commands
- [ ] Short PPO pilot: 64 envs × 24 steps × 100 iterations (153,600 transitions); throughput and memory recorded

## Environment

| Item | Value | Source |
| --- | --- | --- |
| GPU | NVIDIA GeForce RTX 4080, 16 GB; driver 580.159.03, CUDA 13.0 | `nvidia-smi`, 2026-09-14 |
| Isaac Sim | 5.1.0.0 | [codebase investigation](../../docs/codebase_investigation.md#local-implementation-audit) |
| Isaac Lab | package 0.54.3; `~/IsaacLab` at `4df6560e187f` (`training-checkpoints-develop`) | same |
| Isaac Lab RL / RSL-RL | 0.5.0 / 5.0.1 | same |
| Python / PyTorch | 3.11.15 / 2.7.0+cu128 (`env_isaaclab`) | same |
| Task physics / policy rate | 200 Hz / 50 Hz (decimation 4) | `position_only/env_cfg.py` |

## Log

### 2026-09-14 · planning and task scaffold (first session)

- Reviewed Thesis A §§3.1–3.4, Tables 3–4 and Appendices A–C into the
  [ten-week plan](../../docs/thesis_b_plan.md), including gates G0–G3 and the P0–P4 matrix.
- Reviewed six codebases from source at pinned revisions. The recommendation (F-004) is to build on the local Isaac Lab
  model and treat the Isaac Gym codebases as references.
- Wrote the [position-only environment](../../docs/position_only_environment.md) and `run_position_only.py`
  (`check` / `smoke` / `train`). Each run writes `run.json` with versions, source hashes, action order and USD hash.
- CPU tests: 5 pass (F-003). That session could not see a GPU (`nvidia-smi` failed, no `/dev/nvidia*`), so no
  simulator run was possible.

### 2026-09-14 · results record and GPU bring-up (second session)

- Set up this record: `results/` with a folder per week, [findings](../findings.md), [gates](../gates.md), and
  `./dashboard.py` for viewing and for recording runs.
- The GPU is visible in this session. `python run_position_only.py check` exited 0: CUDA available, 1 GPU,
  torch 2.7.0+cu128, isaacsim 5.1.0.0, isaaclab 0.54.3, isaaclab-rl 0.5.0, rsl-rl-lib 5.0.1, D1 URDF present.
  This establishes prerequisites only. No simulator has been launched for this task yet.
- The GPU is shared. A Rescue experiment queue (another session) held 6.6 GB and ~90% utilisation:
  `exp_Control` trained 14:07–14:57, and five more arms (Com, Riser, StairFwd, Placement, Thigh, ~50 min
  each plus benchmarks) follow. Lukas chose not to run the smoke tests or the PPO pilot alongside it. They
  are still to do, with the commands in the [environment guide](../../docs/position_only_environment.md#run-sequence).
  When the pilot runs, note any other GPU jobs, because they contaminate the throughput and memory figures.
- Root disk had 7.5 GB free (97% used). Each run rebuilds its ~10 MB robot USD inside its run folder, and
  pilot checkpoints are small, but long runs will need log housekeeping.
- Recorded scalars take about 0.4 KB per iteration (≈1.6 MB for a 4,000-iteration run), measured by recording
  two existing Rescue logs into a scratch copy of this tree. Nothing from that test was added here.

### 2026-09-14 · unitree_rl_lab review and sim-to-real changes (second session)

Reviewed `~/mairo-rl-lab-rinam` (commit `a179aa0`), recommended for sim-to-real transfer from accurate motor
models. It is the MaiRo course container around Unitree's official `unitree_rl_lab`. The sim-to-real parts are
upstream Unitree code; MaiRo's own changes are course exercises. Source-level notes are in the
[codebase investigation](../../docs/codebase_investigation.md#source-level-findings-to-act-on) (F-006).

Adopted into the position-only task before any P0 training, so P0–P4 all share them:

| Change | Detail | Default |
| --- | --- | --- |
| Go2 leg motor model | `UnitreeActuator` ported to `motor_model.py` and `unitree_actuators.py` (Apache-2.0 notice in `third_party/`): explicit PD 25/0.5, envelope Y1 20.2 / Y2 23.4 N·m, X1 13.5 / X2 30 rad/s | `--leg_actuator unitree`; `dc_motor` for comparison. Playback keeps `dc_motor` |
| Actor/critic split | Actor 66 inputs with no base linear velocity (not measurable on the Go2); critic 87 with base linear velocity and joint torques | Always |
| Observation noise and randomisation | Unitree's Go2 noise levels; friction, restitution, base mass and CoM, ±0.5 m/s pushes | `--robustness none` until nominal reaching works |
| Solver and self-collision | 8/4 solver iterations (stock 4/0); self-collision switchable | 8/4; `--self_collisions` off pending a weld-overlap check |
| Deploy manifest | Each run writes `params/deploy.yaml`: policy output → motor id per bus, scale/offset/clip/limits, gains, actuator models, actor layout | Always |

Checks, all CPU only:

- The ported envelope equals the original unitree_rl_lab clipping function exactly: max abs difference 0.0 over
  801 × 1001 torque–speed points.
- Envelope comparison (F-005). At 13.5 rad/s the stock `DCMotor` allows 12.9 N·m of driving torque; the Unitree
  model allows 20.2. At 20 rad/s braking, stock allows 23.5 N·m against the Unitree model's 14.2.
- 28 tests pass in `env_isaaclab` (7 new in `tests/test_sim2real.py`; the PPO test now asserts 66/87 input widths).

Not yet validated: the actuator class, observation groups, randomisation terms, solver settings and deploy export
have not run in Isaac Sim. They first execute in the smoke tests above. The earlier `check` result predates these
changes but does not depend on them. Not adopted: MaiRo's C++ controller (Go2 legs only, `unitree_sdk2`), kept
as the Thesis C reference; its base policy (12 actions, different observations), which cannot initialise the
18-action actor; and any D1 motor parameters, since none are published.

## Results

Runs recorded this week appear under **Runs** below these notes, with their curves.

## Findings this week

- [F-003](../findings.md): CPU frame maths and PPO configuration work (confirmed, interface only).
- [F-004](../findings.md): primary implementation choice (provisional until the end of Week 2).
- [F-005](../findings.md): stock Isaac Lab Go2 motor model versus Unitree's measured envelope (confirmed, model comparison).
- [F-006](../findings.md): what unitree_rl_lab/MaiRo offers for sim-to-real, and what it does not show (provisional).

## Issues and risks

- The GPU is shared with the Rescue experiment queue (about 4 h remaining at 15:00 on 2026-09-14). The first
  simulator validation of this task (G0) waits on GPU time. Check `nvidia-smi` before launching.
- The task configuration changed (explicit leg actuators, split observations, deploy export) before its first
  simulator run. If the smoke test fails, retry with `--leg_actuator dc_motor` to separate the motor-model change
  from interface faults.
- The `Link6` origin stands in for the interaction point. Reach errors are proxy-frame errors until the tip
  offset is registered.
- `Metrics/ee_position/position_error_m` is sampled at each reset during training. It is a diagnostic, not the
  G1a evaluation, which needs the frozen-manifest evaluator.

## Next week

Workspace validation against an IK reference, the first PPO pilots with reward-term inspection, a pressing-fixture
prototype, and selection of the primary implementation.

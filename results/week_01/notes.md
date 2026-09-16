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

**Revised 16 September.** The [plan was rewritten](../../docs/thesis_b_plan.md) that day, outside this record,
making a physical AprilTag-guided combiner-box demonstration the Thesis B deliverable and adding gates G4–G7.
Its Week 1 row asks for two tracks:

- **Simulation and learning:** close remaining G0 checks; resolve and reset-test the target workspace; start the
  frozen evaluator; trace UniFP and the position-only reference. The workspace item is done (F-016); the frozen
  evaluator and the reference traces are not started.
- **Hardware, camera and deliverable:** bring up RealSense/tags and D1 telemetry; measure basic arm response and
  box loads; record access and fixture needs. **None of this has been started in this repository**, and no camera,
  tag or D1 telemetry code exists here. It is the larger half of the revised Week 1.

## Checklist

- [x] Thesis A timeline and experimental matrix reviewed into the plan
- [x] Codebase candidates reviewed at pinned revisions ([investigation](../../docs/codebase_investigation.md))
- [x] Position-only task scaffold: 18 actions, world-fixed targets, 69 observations, PPO config
- [x] CPU frame and PPO-API tests pass (F-003)
- [x] Results record and dashboard set up
- [x] `run_position_only.py check` passes on the GPU (2026-09-14, see log)
- [x] unitree_rl_lab (via MaiRo) reviewed; motor model, critic split, noise/randomisation and deploy manifest adopted (F-005, F-006)
- [x] D1 servo limits and interface latency model added (`--arm_actuator d1_servo`, `--latency estimated`; F-007)
- [x] Headless smoke run, 1 environment (defaults: `--leg_actuator unitree`, `--robustness none`) (2026-09-15, see log)
- [x] Headless smoke run, 1 environment, `--leg_actuator dc_motor`: standing posture within 0.1 mm of the default, 0 failure resets in both
- [x] `params/deploy.yaml` from the smoke run checked: motor ids, limits, 66-wide actor layout, timing block
- [x] `run.json` `arm_limits_in_physx` checked against 3.3/1.7 N·m and 1.05/1.73 rad/s (exact)
- [x] Held arm targets visible in a trace: arm target changes only every 5 policy steps (`verify`, 8 envs, F-009)
- [x] Smoke run with `--latency none --leg_actuator dc_motor --arm_actuator implicit` (pre-port configuration) for comparison
- [x] Headless smoke run, 4 environments
- [ ] Visible 1-environment run inspected: posture, target marker location (screenshot)
- [x] Self-collision check: no resting contact from overlapping weld shapes (measured headless, with a positive control; a screenshot is optional)
- [x] Deliberate reset tests: time limit, low base, tilt, base contact, workspace exit, partial reset (`verify`, F-009)
- [x] Controlled point chosen: the Link7_1 pincer tip (CAD end-face centre; `--tip_body`, `--tip_offset`), matched in simulation to 0.9 µm (F-013). Not measured on the physical arm
- [x] Target box checked against the reachable, collision-free workspace: reachable, but zero actions meet 5 cm for 12.9% of targets (F-011, then F-013)
- [x] Playback reference: bare Go2 and Go2+D1, with forward, lateral and yaw commands (F-012)
- [x] Short PPO pilot: 64 envs × 24 steps × 100 iterations (153,600 transitions); throughput and memory recorded (three pilots)
- [x] Rescue flat ablation recorded as external evidence (F-008)
- [ ] Gait-quality metrics (foot vs neutral point, front–rear spacing, backward after request, turn tracking at 0.2/0.5/1.0 rad/s, pitch wobble) added to the P0 evaluator design for G1b
- [x] Move the target box (or change the reset height) so zero actions do not meet the 5 cm criterion, before P0 training (F-013): box moved forward and down, spawn lowered to 0.30 m; zero actions now score 0/256 (2026-09-16, F-014, F-015, F-016)
- [ ] Grow the target range beyond the first box: base-motion targets and a growth schedule for plan stage 6 (the 12 × 16 × 12 cm stage-5 box is set; `workspace.py` scores candidates against the measured stance)
- [ ] Explain the 0.010 rad residual at J3 with force drives (F-010)
- [ ] Decide whether playback (`run_sim.sh`) should also get force arm drives; it would change the F-012 reference

## Environment

| Item | Value | Source |
| --- | --- | --- |
| GPU | NVIDIA GeForce RTX 4080, 16 GB; driver 580.159.03, CUDA 13.0 | `nvidia-smi`, 2026-09-14 |
| Isaac Sim | 5.1.0.0 | [codebase investigation](../../docs/codebase_investigation.md#local-implementation-audit) |
| Isaac Lab | package 0.54.3; `~/IsaacLab` at `4df6560e187f` (`training-checkpoints-develop`) | same |
| Isaac Lab RL / RSL-RL | 0.5.0 / 5.0.1 | same |
| Python / PyTorch | 3.11.15 / 2.7.0+cu128 (`env_isaaclab`) | same |
| Task physics / policy rate | 200 Hz / 50 Hz (decimation 4) | `position_only/env_cfg.py` |
| CPU / RAM | AMD Ryzen 9 9950X3D, 16 cores; 31 GB | Isaac Sim startup report, 2026-09-15 |
| Root disk | 34 GB free (85% used); `logs/` holds 292 MB. Was 3.6 GB free on 2026-09-15; freed outside this record | `df -h`, 2026-09-16 |
| GPU during Week 1 runs | No other compute job: every launch checked `nvidia-smi --query-compute-apps` first | 2026-09-15, 2026-09-16 |

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
18-action actor. (Superseded the same day: a D1 servo and latency model was then added; see the next entry.)

### 2026-09-14 · D1 servo model and interface latency (second session)

Lukas asked for the D1 motor model and latency to be brought over as well. The MaiRo/unitree_rl_lab repo has
neither. It contains no D1 or arm files, and its delay support is set to zero for the Go2. So the mechanisms
came from that repo, and the D1 numbers from the best available sources, each labelled in `motor_model.py`:

| Quantity | Value used | Source and confidence |
| --- | --- | --- |
| D1 joint torque limits | 3.3, 3.3, 1.7, 1.7, 1.7, 1.7 N·m | Unitree D1-550 published spec (README). Published |
| D1 joint speed limits | 1.05 rad/s (J1–3), 1.73 rad/s (J4–6) | `d1_arm/d1.urdf`. **Unverified**: Unitree's `d1_description` has `velocity="0"`; filled in during the Rescue port (2026-07-16) with no source. No published D1-550 joint speed found online |
| D1 torque–speed curve | None: implicit stiff drive with the limits above | Not published by Unitree; an explicit 4000/400 PD is unstable at 200 Hz |
| D1 command rate | Arm targets held 5 policy steps (10 Hz), random phase | D1 SDK streaming setpoints at ~10 Hz (Rescue `d1_sdk`, reproduced from the SDK) |
| D1 feedback | Joint angles sampled at 10 Hz, random phase; velocity differenced from samples | D1 SDK publishes angles only at 10 Hz |
| D1 firmware smoothing | Not modelled | SDK exposes a smoothing mode; its filter is undocumented |
| Go2 leg command delay | 0–2 physics steps (0–10 ms), drawn per episode | Estimated from unitree_rl_lab's controller: 50 Hz policy thread, 1 kHz command publish. Not measured |

Implementation: `HeldJointPositionAction` (arm command hold) and `SampledJointFeedback` (arm angle/velocity feedback)
in `position_only/mdp.py`; `SampleAndHold` in `position_only/core.py`; the leg delay uses the Unitree actuator's delay
buffer. The actor's layout is unchanged at 66 (legs then arm, positions then velocities) and the critic still sees
exact state. New flags: `--arm_actuator {d1_servo,implicit}` and `--latency {estimated,none}`. Both are recorded in
`run.json` and the deploy file's `timing` block. `run.json` also records the arm limits PhysX actually applies.

Checks (CPU only): 33 tests pass, including the latency profiles and sample-and-hold phases, differenced velocity,
reset, and repeated updates within one step. Nothing here has run in Isaac Sim. The first smoke run exercises all
of it, with a pre-port comparison run listed in the checklist.

### 2026-09-15 · Rescue flat ablation: training with the arm attached degrades the gait (external evidence)

Lukas ran a locomotion ablation in the Rescue project. It is recorded here because it is direct evidence that
attaching the D1 makes locomotion learning harder. Evidence, provenance and definitions are in
[external/rescue_flat_ablation](external/rescue_flat_ablation/README.md), copied because nothing was committed in
Rescue.

**Setup.** These are Rescue's flat velocity-tracking policies: 12 leg actions, with the D1 welded on but passive
(folded, held at zero). Each run removes one factor Rescue's task added to P2Dingo's. Every policy trained for 2000
iterations from scratch with 4096 environments, seed 42 unless noted. All were benchmarked on the same welded
Go2+D1 with the measured motor (Rescue `measure_bench.py`, seed 1, 200 robots, flat ground).

![Rear-foot placement and front–rear spacing per configuration](figures/rescue_flat_ablation_gait.png)

| | P2Dingo flat | all removed | **no arm** | Rescue, seed 42 | Rescue, seed 2 | no MaiRo DR | stock motor | 8 cm clearance |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Arm on the training robot | no | no | no | yes | yes | yes | yes | yes |
| Front / rear foot vs neutral point, 0.5 m/s (cm) | −1 / −2 | −2 / −1 | **−4 / −3** | −12 / +18 | −9 / +3 | −7 / +14 | −10 / +17 | −10 / +17 |
| Same-side front–rear spacing, 1.0 m/s (cm) | 66 | 58 | **54** | 32 | 36 | 32 | 35 | 32 |
| Stride / stance, 0.5 m/s | 28 cm / 346 ms | 21 / 253 | 17 / 189 | 25 / 234 | 12 / 131 | 28 / 274 | 23 / 223 | 27 / 250 |
| Forward tracking, 0.5 / 1.0 m/s | 98 / 100 % | 95 / 97 | 106 / 102 | 100 / 107 | 104 / 102 | 101 / 104 | 98 / 107 | 101 / 105 |
| Turn tracking, 0.2 / 0.5 / 1.0 rad/s | 7 / 98 / 104 % | 58 / 99 / 99 | 51 / 124 / 109 | – / – / 76 | 75 / 106 / 105 | – / – / 89 | – / – / 99 | – / – / 102 |
| Drift turning at 1.0 rad/s (m/s) | 0.055 | 0.059 | 0.028 | 0.060 | 0.030 | 0.067 | 0.053 | 0.048 |
| Backward within 1 s of a 1.0 m/s request | 0 % | 0 | 0 | 57 | 0 | 0 | **99.5** | 0 |
| Pitch wobble, 1.0 m/s (deg) | 0.73 | 0.70 | 0.46 | 1.17 | 0.17 | 0.93 | 0.95 | 0.94 |

(– = not measured at that turn rate. Unrounded values: [summary.csv](external/rescue_flat_ablation/summary.csv).)

What it shows:

- "All removed" reproduces P2Dingo (rear foot −1 cm against −2 cm; spacing 58 against 66 cm), so no difference
  between the two tasks was missed.
- Of the single removals, only taking the arm off the **training** robot puts the feet back under the body. Both
  arm-free runs land within 4 cm of the neutral point with 54–58 cm spacing. Three of the four runs that kept the
  arm (no DR, stock motor, 8 cm clearance) land the rear feet 14–17 cm ahead of it, with 32–35 cm spacing. Rescue
  seed 42 is the same at +18 cm. Rescue seed 2 avoids that only with a 12 cm stride and 131 ms stance, a shuffle.
  In all, 2 of 2 arm-free runs walk well and 0 of 5 arm-on runs do.
- The randomisation, the motor model and the clearance target did not cause it. The stock-motor run also backed
  away after a forward request in 99.5% of robots (one seed).
- A policy that never trained with the arm walks the welded robot better than every policy that did. This is
  consistent with F-001, now with foot-placement and turn metrics.

What it does not show: why. Rescue's hypothesis is that the arm's added height and pitch inertia make "feet under
the body" the cheapest pitch control during learning; that is untested. It covers one task (locomotion with a
passive arm, not whole-body control), flat ground only, and one training seed per configuration (the five arm-on
runs are not independent replicates of one setting). No 0.2 rad/s turn is tracked well by any policy (7–75%).

For this project (F-008), P0 is exactly "arm on the training robot", so the plan now asks locomotion evaluations
(G1b) to report gait quality alongside tracking.

### 2026-09-15 · First simulator runs: smoke, deliberate checks, arm drive fix, pilots, workspace, playback (third session)

The GPU was free (742 MiB display only, no compute processes). Every launch below first checked
`nvidia-smi --query-compute-apps` and found nothing else running, so the throughput figures are uncontaminated. All
commands ran from the repository root in `env_isaaclab` with ROS variables unset.

#### Runner fix: failed runs exited 0

The first two `python run_position_only.py smoke --headless --num_envs 1 --steps 600` attempts exited 0 but wrote no run
folder. Isaac Sim's `SimulationApp.close()` ends the process with status 0 before Python can print an exception, so any
error inside `run()` looked like a success. The runner now prints the traceback and exits 1, and it flushes output
before shutdown. The hidden error was `AttributeError: type object 'FlatSceneCfg' has no attribute 'terrain'`:
`configclass` turns mutable class attributes into default factories. The scene config now reads them from an instance.
Neither attempt created a run folder, so neither could be recorded.

#### Smoke runs (zero actions: default joint targets)

| Run | Configuration | Failure / time-limit resets | Settled base height | Arm joint deviation (mean) |
| --- | --- | --- | --- | --- |
| [first simulator run](#/week/1/run/20260915T093352_794571Z_smoke_seed42) | defaults, 1 env | 0 / 1 | (no posture trace yet) | |
| [defaults](#/week/1/run/20260915T093730_338089Z_smoke_seed42) | Unitree legs, `d1_servo`, estimated latency | 0 / 1 | 0.2661 m | 0.110 rad |
| [`dc_motor`](#/week/1/run/20260915T093743_674698Z_smoke_seed42) | stock DCMotor legs | 0 / 1 | 0.2661 m | 0.110 rad |
| [pre-port](#/week/1/run/20260915T093756_424913Z_smoke_seed42) | `dc_motor`, `implicit` arm, no latency | 0 / 1 | 0.2661 m | 0.110 rad |
| [4 envs](#/week/1/run/20260915T093809_039560Z_smoke_seed42) | defaults | 0 / 4 | 0.2666 m | 0.110 rad |
| [force arm drives](#/week/1/run/20260915T095815_165623Z_smoke_seed42) | defaults after the drive fix (below) | 0 / 1 | 0.2663 m | 0.018 rad |
| [4 envs, contact logged](#/week/1/run/20260915T100526_698612Z_smoke_seed42) | self-collisions off | 0 / 4 | 0.2664 m | 0.018 rad |
| [4 envs, `--self_collisions`](#/week/1/run/20260915T100540_450156Z_smoke_seed42) | self-collisions on | 0 / 4 | 0.2664 m | 0.018 rad |

Posture statistics exclude the first 50 steps. `smoke_trace.csv` holds the per-step trace; only the first
smoke run (09:33 UTC) predates it.

![Zero-action posture: leg motor models overlap; acceleration against force arm drives](figures/smoke_posture.png)

What they show:

- **Interface.** One articulation with 20 joints and 18 actions in the declared order, 66 actor and 87 critic inputs,
  arm sensor bodies resolved, finite observations and rewards, automatic resets. Articulation mass 18.171 kg.
- **Deploy manifest.** All 12 leg motor ids match unitree_sdk2's FR, FL, RR, RL order (FL_hip → 3, FR_hip → 0,
  RL_hip → 9, RR_hip → 6, …). The arm maps to ids 0–5, the actor layout is 66 wide, and the timing block holds
  0–2 leg delay steps and 5-step arm hold and feedback periods.
- **Arm limits in PhysX** exactly 3.3/3.3/1.7/1.7/1.7/1.7 N·m and 1.05/1.05/1.05/1.73/1.73/1.73 rad/s, with
  `d1_servo` **and** with `implicit`: the URDF import already carries these limits, so `d1_servo` restates them.
- **Leg limits in PhysX.** Both leg models inherit the Go2 USD's joint speed limits (30.1 rad/s hip and thigh,
  15.7 rad/s calf), and explicit actuators give PhysX a 1e9 N·m torque cap because the envelope does the clipping.
  The deploy file's leg `effort_limit` (23.7 / 45.43 N·m) is the USD value, which the Unitree envelope does not use.
  None of these speed limits carries a source label in `motor_model.py`.
- **Stance.** Spawned at 0.42 m, the robot drops for about 7 steps, lands with a peak tilt of 18.9° and slides 7 cm.
  It settles by 2 s at 0.266 m, tilt 1.3°, legs 0.35 rad from their default angles, peak leg torque 8.7 N·m. Every
  episode starts with this drop.
- **Leg models.** Unitree and DCMotor standing traces agree to 0.1 mm in height. Standing never reaches the joint
  speeds where the two envelopes differ (F-005), so a smoke run cannot compare them; that needs motion.
- **Gripper.** It resets to 1.5 mm open: its default of 0 lies outside the 0.9-scaled soft limits of its
  0–30 mm stroke.

What they do not show: standing is a PD response to default targets, not a learned stance, and nothing here reaches
for anything.

#### Deliberate checks: new `verify` mode

`python run_position_only.py verify --headless --num_envs 8` induces each mechanism and states what it expects
(`position_only/verify.py`, output `verify.json`):

| Run | Result | What changed |
| --- | --- | --- |
| [verify 1](#/week/1/run/20260915T094240_086775Z_verify_seed42) | 11/16 | Two errors in the checks themselves: joints compared against the unclamped gripper default, and a base placed 7 mm into the ground, too shallow to outlast the contact sensor's 3-substep history |
| [verify 2](#/week/1/run/20260915T094418_958379Z_verify_seed42) | 16/16 | Timing, terminations and partial resets |
| [verify 3](#/week/1/run/20260915T095136_775722Z_verify_seed42) | 18/19 | Arm model added: kinematics and masses match; **the arm sags at rest** |
| [verify 4](#/week/1/run/20260915T095318_759126Z_verify_seed42) | 18/19 | Sag diagnostics (below) |
| [verify, force drives](#/week/1/run/20260915T095752_008164Z_verify_seed42) | 19/19 | `d1_servo` with force drives |
| [verify, acceleration drives](#/week/1/run/20260915T095803_471192Z_verify_seed42) | 18/19 | `--arm_actuator implicit`, same check code |
| [verify, self-collision control, off](#/week/1/run/20260915T100737_413430Z_verify_seed42) | 20/20 | Positive control added |
| [verify, self-collision control, on](#/week/1/run/20260915T100751_257895Z_verify_seed42) | 20/20 | `--self_collisions` |

- **Arm command hold.** Targets change exactly every 5 policy steps, at per-environment phases 0–3 in 8 envs. Legs
  update every step.
- **Arm feedback.** Samples change exactly every 5 steps at their own phases. A fresh sample equals the simulator's
  angle (0.0 rad error), and differenced velocity matches (sample − previous) / 0.1 s to 1.2e-7 rad/s, held between
  samples. Leg delays drawn: 2, 0, 1, 0, 1, 0, 2, 1 physics steps.
- **Terminations and partial resets.** Each condition was induced in its own environment:
  - episode length set to 499 → `time_out`;
  - base placed at 0.10 m → `low_base`;
  - base rolled 60° → `bad_orientation`;
  - base centre placed at ground level → `base_contact` and `low_base`;
  - base moved 0.80 m → `outside_workspace`.

  Only those environments reset, to the default root pose and the soft-clamped default joints (error 0.0), with a new
  target inside the box and the arm hold back at default. Three control environments carried on (episode length 61,
  same target, 0.5 mm drift), and nothing terminated in the 5 steps after the resets.
- **Arm model.** PhysX link masses and centres of mass equal the weld mass model (0 g, < 1 µm). At 8 random arm poses,
  Link6 in the base frame equals URDF forward kinematics to 0.6 µm.

#### Finding: the arm sagged because the URDF import made acceleration drives (F-010)

Verify 3 found J2, J3 and J5 standing 0.013, 0.080 and 0.110 rad off target, although PhysX reported joint loads of
1.20, 1.10 and 0.29 N·m, below the 3.3/1.7/1.7 N·m limits. Verify 4 ruled out the landing: joint friction and armature
were 0, and reseating the arm exactly on target while standing returned it to the same sag within 0.2 s. A scratch
probe read the drive attributes from the stage: `Joint1`–`Joint6` are `acceleration` drives and the Go2 legs are
`force`. PhysX scales an acceleration drive's gains by joint inertia, so the arm's 4000 N·m/rad behaved like roughly
90, 14 and 3 N·m/rad at J2, J3 and J5. The probe (not a run folder) was reproduced by the two recorded verify runs:

| Arm drives | J2 / J3 / J5 deviation at rest | Link6 droop | Joint load J2 / J3 / J5 |
| --- | --- | --- | --- |
| acceleration (URDF import, `--arm_actuator implicit`) | 0.013 / 0.080 / 0.110 rad | 35.5 mm | 1.20 / 1.10 / 0.29 N·m |
| force (`d1_servo`, now) | 0.003 / 0.010 / 0.0002 rad | 4.1 mm | 1.18 / 1.10 / 0.30 N·m |

**Change:** `make_robot_cfg(..., arm_actuator="d1_servo")` sets `JointDrivePropertiesCfg(drive_type="force")`, and
`run.json` records the drive types read from the stage. Playback's arm is unchanged: it still uses the import's
acceleration drives. The 35.5 mm droop matches F-002's 3.3 cm at stiffness 4000, so F-002 is superseded. With force
drives J3 still carries its load at 0.010 rad (about 105 N·m/rad); that residual is unexplained. The 5 mm pass
threshold was set after the probe, so the check separates the two drive types but does not bound what is still
unexplained.

#### Self-collision

With `--self_collisions`, arm contact force at rest was 0.0 N over 600 steps × 4 envs, and posture matched the
self-collisions-off run to 4 decimals. A zero means "no overlap" only if the sensor can see arm-to-body contact, so
`verify` now commands the arm to angles (0, 1.0, 0.88, 0, −0.16, 0). Forward kinematics put the fingertips 7 cm inside
the trunk at that pose. With self-collisions **off**, the arm reached the pose (0.002 rad) with 0 N contact: it passed
through the body. With them **on**, the body stopped it 0.344 rad short, with 16.2–20.2 N peak arm contact in all
8 envs. The touch did not trigger the base-contact termination.

**Change:** self-collisions are now on by default (`--no-self_collisions` to disable), as in unitree_rl_lab. The
documented condition for turning them on (no resting contact from the weld) is met. Arm-to-body contact is not a
termination or a penalty in the task yet.

#### PPO pilots: `train --headless --num_envs 64 --iterations 100 --seed 42`

| Pilot | Arm drives / self-collisions | Steps/s (mean over iterations 1–99) | Training time | GPU memory added (peak) | Peak tilt-failure share | Final mean reward / episode length | Episode-end reach error, last 20 iterations |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [pilot 1](#/week/1/run/20260915T094508_663721Z_train_seed42) | acceleration / off | 3,787 | 40.8 s (47 s wall) | +2.72 GB (3,347 MiB) | 23% | 16.15 / 500 | 0.065–0.157 m |
| [pilot 2](#/week/1/run/20260915T095829_331585Z_train_seed42) | force / off | 3,605 | 42.8 s | +2.72 GB (3,328 MiB) | 37% | 14.79 / 495 | 0.081–0.215 m |
| [pilot 3](#/week/1/run/20260915T100923_720849Z_train_seed42) | force / on (new defaults) | 3,675 | 41.9 s | +3.02 GB (3,632 MiB) | 37% | 14.79 / 495 | 0.081–0.215 m |

Collection took about 0.36 s per iteration and learning 0.035 s. GPU memory is whole-GPU `nvidia-smi` sampled every
2 s against a 610–630 MiB baseline (`gpu_memory.csv` in each run).

What they show:

- **Pipeline.** PPO runs end to end on the GPU with the 66/87 split, saves checkpoints and logs every term. Early tilt
  failures from exploration noise fall to 0% by iteration 79 in all three pilots.
- **Self-collisions** cost no measurable throughput and changed nothing in this pilot. Pilots 2 and 3 are identical
  in all 2,200 learning scalars, so the arm never touched the body in 100 iterations of this seed.
- **Reproducibility.** That identity also shows a repeated seeded run on this machine reproduces exactly.

What they do not show: any reaching. The reach metric is a training-time value at episode end, averaged over
whichever environments reset. It is no better than zero actions (4.4–11.7 cm), because the targets start next to the
arm (F-011). Differences between pilots 1 and 2 come from a single seed.

#### Workspace and tool point (CPU, validated against the simulator)

`python -m position_only.workspace --out results/week_01/figures` samples 400,000 arm poses inside the 0.9 soft
limits. It uses URDF kinematics and the weld mass model, both confirmed in simulation above, and a box-shaped
collision proxy for the Go2 body.

![Workspace and start distances (regenerated for the pincer tip later the same day)](figures/d1_workspace.png)

*Correction, later on 2026-09-15:* the start distances in this subsection assume the base at the environment origin.
Under zero actions the robot settles 7.4 cm back, and the controlled point is now the pincer tip, so these Link6
figures are superseded (F-013 and the next entry). The figure file now shows the pincer tip; the origin-stance Link6
values remain in `d1_workspace.json` under `by_base_height`.

- **Reachable.** At every base height from 0.22 to 0.38 m, at least 99.9% of the target box is reachable by Link6
  (within 1.5 cm), clear of the body proxy and statically holdable. Every sampled pose is holdable: the largest
  static load stays inside the torque limits, and 0.99 kg of the arm moves.
- **Starts next to the arm.** The arm's zero pose, where every episode starts, puts Link6 at (0.280, −0.001, 0.500) m
  in the base frame. At the zero-action stance height of 0.266 m that is inside the top of the box. Targets are
  0.4–15.5 cm from it (mean 8.3 cm), and 14% are within the 5 cm success radius. At 0.30 m base height the share is
  5%, and from 0.34 m it is 0% (F-011). Zero actions already earn a coarse reach reward of about
  exp(−(0.083/0.15)²) ≈ 0.74 of its maximum.
- **Tool point.** From the finger meshes (CAD, not measured on the arm), the closed fingertip centre is at
  (0.0007, −0.0005, 0.1251) m in the Link6 frame, and the pad centre at (0.0004, 0.0000, 0.1241) m. Link6's z axis
  points forward at the zero pose, so the fingertips sit 12.5 cm ahead of the controlled point. Whether P0 controls
  the fingertip, a tool or Link6 is still open.

#### Playback reference (F-012)

`./run_sim.sh --headless --no_ros2 --selftest 10 --selftest_command VX VY WZ --selftest_out logs/playback [--no_arm]`
uses the new self-test options: a command vector, mean base velocity, and a run folder `dashboard.py record` can
store. The walking checkpoint is `logs/rsl_rl/unitree_go2_rough/2024-04-06_02-37-07/model_7850.pt` (sha256
`7e684601…`) with DCMotor legs. The first runs' `run.json` named the policy wrongly; the label was corrected to this
path and hash after the runs. Each run is 10 s after a 1 s settle, one robot.

| Command | Go2 + D1, 18.17 kg | Bare Go2, 15.02 kg |
| --- | --- | --- |
| Forward 1.0 m/s | [0.955 m/s (96%)](#/week/1/run/20260915T100336_284175Z_playback_arm_+1.0_+0.0_+0.0), max tilt 7.8°, min height 0.328 m | [0.972 m/s (97%)](#/week/1/run/20260915T100410_249895Z_playback_bare_+1.0_+0.0_+0.0), 8.0°, 0.363 m |
| Lateral 0.5 m/s | [0.436 m/s (87%)](#/week/1/run/20260915T100348_230691Z_playback_arm_+0.0_+0.5_+0.0), 6.4°, 0.376 m | [0.409 m/s (82%)](#/week/1/run/20260915T100420_122466Z_playback_bare_+0.0_+0.5_+0.0), 5.6°, 0.403 m |
| Yaw 1.0 rad/s | [0.923 rad/s (92%)](#/week/1/run/20260915T100400_365939Z_playback_arm_+0.0_+0.0_+1.0), **31.2°**, **0.227 m**, drift 0.15 m/s | [0.782 rad/s (78%)](#/week/1/run/20260915T100430_024793Z_playback_bare_+0.0_+0.0_+1.0), 16.1°, 0.323 m, drift 0.04 m/s |

- **Forward** reproduces F-001: 96% of command and 7.8° peak tilt, against 95% and 7.3° pre-term.
- **Turning** is where the arm shows: twice the peak tilt, a 10 cm lower base and four times the sideways drift.
- **Arm and gripper.** The IK arm target was held to 3.4–3.8 cm (acceleration drives, as in F-002), and the gripper
  opened to 59–60 mm of the commanded 60 in every arm run.

Single 10 s runs with one checkpoint that never trained with the arm: a reference, not a P0 result.

#### Code and document changes

- **Runner (`run_position_only.py`).** Nonzero exit on failure, posture trace, leg and arm limits and drive types in
  `run.json`, and the `verify` mode.
- **Task config.** `position_only/env_cfg.py` gets the scene fix and self-collisions on; `flat_env_cfg.py` gives
  `d1_servo` force arm drives.
- **New modules.** `position_only/verify.py` and `position_only/workspace.py`.
- **Playback.** `sim.py` and `main.py` gain the self-test command and run folder.
- **Record.** `evidence/record.py` summarises `verify.json` and `selftest.json`; `results/config.json` adds
  `logs/playback`.
- **Tests.** `tests/test_workspace.py` is new. 37 tests pass in `env_isaaclab`, and `test_evidence` passes on the
  system Python.
- **Docs.** [Environment guide](../../docs/position_only_environment.md): status, arm drives, self-collisions,
  `verify`, provisional choices. [README](../../README.md) "Known behaviour" now says the droop table used
  acceleration drives. [Plan](../../docs/thesis_b_plan.md) status updated.

### 2026-09-15 · Controlled point moved to the Link7_1 pincer tip; zero-action baseline measured (third session, continued)

Lukas asked what "tool point" meant and chose one of the pincers for now.

**What it is.** The tool point is the one point on the end effector that the task steers: its distance to the target
is the reach error, the reach rewards are computed from it, the actor observes it (`tip_position`), and G1a's 5 cm
criterion applies to it. Until now it was the Link6 origin, the wrist frame, which sits 12.5 cm behind the fingertips.

**Choice.** The tip of the Link7_1 pincer: the centre of its 26 × 7 mm end face, at (0.0547, 0.0060, 0.0170) m in
Link7_1's own frame, from the CAD finger mesh (`workspace.pincer_tip`). At the arm's zero pose this is the pincer on
the robot's right, at (0.406, −0.014, 0.500) m in the base frame. The point is attached to the pincer rather than to
Link6, so it follows the finger when the jaw opens. It is now the default (`position_only/tool_point.py`;
`--tip_body Link7_1 --tip_offset 0.0547 0.0060 0.0170`). `--tip_body Link6 --tip_offset 0 0 0` restores the old
point, and `run.json` records the body and offset.

| Run | Result |
| --- | --- |
| [verify, pincer tip](#/week/1/run/20260915T103642_229904Z_verify_seed42) | 20/21: the new tool-point kinematics check failed at 1.12 mm |
| [verify, per-env diagnostics](#/week/1/run/20260915T104019_667798Z_verify_seed42) | 20/21: only env 6 was off. Its random arm pose sat inside the Go2 trunk (self-collisions now on), with 602 N arm contact in one physics step. The other 7 matched to ≤ 1 µm |
| [verify, new defaults](#/week/1/run/20260915T104104_347118Z_verify_seed42) | **21/21**: random poses now clear the body proxy; the pincer tip matches URDF kinematics to 0.9 µm |
| [smoke, 8 envs](#/week/1/run/20260915T103656_126177Z_smoke_seed42) | final error 3.4–13.3 cm (after the 10 s reset) |
| [smoke, 8 envs, base position logged](#/week/1/run/20260915T104159_671108Z_smoke_seed42) | the robot settles 7.4 cm **behind** the environment origin (x −0.065 to −0.094 m, y 0.000 m, yaw 0.0°) |
| [zero-action baseline, 256 envs](#/week/1/run/20260915T104334_162270Z_smoke_seed42) | final error at 9 s: 1.2–14.8 cm, mean 8.2 cm; **33 of 256 (12.9%, 95% interval 9.3–17.6%) within 5 cm** |

Two probes in the scratchpad (not run folders) located the 1.12 mm. At rest, and with the jaw opened 10 and 20 mm per
pincer, the simulator's pincer poses relative to Link6 match the URDF joint transforms to 0.0 mm. The error needed the
arm teleported into the trunk. That is a contact effect from the check itself, not a kinematics error.

**The workspace model had the stance wrong.** The first pincer-tip estimate put the base at the environment origin,
and predicted targets 4.6–21.8 cm from the start with 0.5% within 5 cm. The simulator disagreed, 3.4 cm in the first
8 envs, so the smoke trace now logs base x, y and yaw. Every reset drops the robot from 0.42 m and it slides 7.4 cm
back, which carries the pincer tip to (0.331, −0.014, 0.766) m from the origin: inside the box. With the measured
stance, `python -m position_only.workspace --out results/week_01/figures --baseline_smoke …` predicts 13.1% within
5 cm and a mean of 8.7 cm. The simulator measured 12.9% and 8.2 cm.

![Pincer tip workspace; start distances from the model and the simulator](figures/d1_workspace.png)

What it shows:

- **Controlled point.** The pincer tip is implemented, and matches the model to 0.9 µm in simulation. The target
  box stays at least 99.8% reachable for it.
- **Free successes remain.** Switching from Link6 to the pincer tip did not remove them. At the real stance, the
  Link6 origin would have started 12.3 cm from its targets (1.8% within 5 cm), and the pincer tip starts 8.7 cm away
  (13%). So F-011's numbers were wrong about the stance, and its conclusion still holds for the new point (F-013).
  The box has to move, or the reset has to stop the slide, before G1a.

What it does not show:

- **Hardware.** The tip is CAD geometry; it has not been measured on the arm (Thesis C registration).
- **Grasping.** A pincer tip suits pressing or touching; grasping would use the point between the pincers.
- **Earlier runs.** PPO pilots 1–3 and every run before this entry used Link6.

### 2026-09-15 · Visual replay of the target box issue (third session, continued)

Lukas asked to see the issue. `python run_position_only.py view --num_envs 16` (new mode) runs zero-action episodes in
real time in the viewer until the window closes, printing one line per episode. The command term now draws markers
whenever a run has a viewer:

- **Target:** green once the pincer tip is within 5 cm, red otherwise.
- **Pincer tip:** blue dot.
- **Target box:** orange wireframe.
- **Spawn position:** grey ground plate.

| Overview (16 robots) | Close-up |
| --- | --- |
| ![Zero-action replay: boxes, targets, pincer tips and spawn plates](figures/replay_target_box_overview.png) | ![A resting pincer tip inside the box next to a green target](figures/replay_target_box_closeup.png) |

(Viewport captures by Claude from the desktop, cropped to leave out a desktop menu; not Lukas's screenshots.)

[Replay run](#/week/1/run/20260915T105401_574747Z_view_seed42):

- **Result.** 42 episodes × 16 robots. **111 of 672 targets (16.5%, 95% interval 13.9–19.5%) were within 5 cm at 9 s
  with no arm motion**, 0–6 per episode, mean error 8.4 cm. The base slid 7.2 cm back in each reset episode (7.5 cm
  after the initial spawn). This agrees with the headless 256-robot measurement (12.9%, 9.3–17.6%). Pooled: 144 of
  928, 15.5%.
- **How it ended.** The process was killed with SIGKILL (exit 137) after about 7 minutes; no out-of-memory message was
  found, and the cause was not logged. `run.json` had not yet stored the episodes, so they were recovered from the
  console log (rounded to 0.1 cm) and the status set to `killed`, with that explanation in its `error` field. `view`
  now saves each episode as it finishes.
- **Camera.** The camera opened on a wide world view instead of the side view set in the config. The viewer panel's
  Follow Mode showed World, so `origin_type="env"` did not take effect.

Questions raised while watching:

- **The arms do not move** because this is the zero-action baseline: no policy, joints held at their default angles.
  A trained policy is meant to move the pincer tip onto the target.
- **The box is small on purpose.** Plan stage 5 starts with a small workspace, and stage 6 grows it: trajectories,
  then base movement, then contact. The current box also surrounds the resting tip and never needs the base to move
  (F-013). Its replacement, a start-pose exclusion and a growth schedule are open design choices. The ±1 rad arm
  action clip may need widening for a larger range.

### 2026-09-16 · Target box moved off the resting tip; the reset drop replaced by a standing spawn (fourth session)

The Week 1 blocker from F-013: the target box surrounded the pincer tip's resting position, so 12.9% of
targets were met with no arm motion at all and the reach metric could not be told apart from doing nothing.
Lukas chose a forward-and-down box and asked for the reset drop to be fixed in the same change.

#### The backward slide is the posture, not the drop

A spawn-height sweep (8 envs, 250 steps, zero actions; the five runs below) separated the two. Lowering the
spawn cuts the tilt transient sharply but leaves the slide almost untouched:

| Spawn | Peak tilt | Lowest base | Settled base x | Run |
| --- | --- | --- | --- | --- |
| 0.42 m | 18.6° | 0.197 m | −7.5 cm | [sweep 0.42](#/week/1/run/20260915T233821_522424Z_smoke_seed42) |
| 0.34 m | 8.4° | — | −4.9 cm | [sweep 0.34](#/week/1/run/20260915T233830_692754Z_smoke_seed42) |
| 0.32 m | 6.7° | — | −5.5 cm | [sweep 0.32](#/week/1/run/20260915T233839_655054Z_smoke_seed42) |
| 0.30 m | 4.3° | 0.260 m | −5.5 cm | [sweep 0.30](#/week/1/run/20260915T233847_549394Z_smoke_seed42) |
| 0.28 m | 4.3° | — | −5.4 cm | [sweep 0.28](#/week/1/run/20260915T233855_377869Z_smoke_seed42) |

Even spawning at 0.28 m, essentially the settled height, the robot still slides ~5 cm back and overshoots to
−8.8 cm before recovering. So the slide is the zero-action posture settling under its own leg PD, not the
drop. F-013 attributed it to the drop; that attribution was wrong (F-014).

Two scratchpad probes (16 envs, 12 s, no time limit, one process per height; not run folders) measured the
converged stance and separated its spread:

| Spawn | Converged base x | Base height | Tilt | Pincer tip (env frame) | Settled by | Peak tilt | Lowest base |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.42 m | −7.461 cm | 0.2667 m | 1.30° | (0.3435, −0.0142, 0.7514) m | 2.6 s | 18.83° | 0.197 m |
| 0.30 m | −5.551 cm | 0.2737 m | 1.03° | (0.3601, −0.0141, 0.7604) m | 3.6 s | 5.11° | 0.260 m |

The spread of the settled base x **across environments** is 0.22 cm (sd) at 0.42 m and 0.005 cm at 0.30 m.
F-013 reported a 2.9 cm spread; that range was the settling transient over time, not variation between
environments, which is small (F-014).

The deciding number is the lowest base height. Dropping from 0.42 m takes the base to 0.197 m, only 4.7 cm
above the `low_base` termination at 0.15 m, before it recovers. Spawning at 0.30 m keeps it at 0.260 m.
With randomisation on (`--robustness unitree`: pushes, mass and CoM changes) the drop is close enough to the
limit to terminate episodes at reset for reasons that have nothing to do with the task. `SPAWN_HEIGHT_M = 0.30`
is now the task default; `--spawn_height` overrides it, and `run.json` records it. Playback (`run_sim.sh`)
keeps `flat_env_cfg.ROBOT_START_POS`, so F-012's reference is unchanged.

#### The CPU model puts the resting tip 1.6 cm out

The workspace model predicted the resting tip at (0.3502, −0.0139, 0.7732) m; the simulator measures
(0.3601, −0.0141, 0.7604) m, 1.6 cm away. The model has neither the arm's residual sag (F-010) nor the base's
1.0° resting tilt. The box is placed against the **measured** point, and `task_space.ZERO_ACTION_TIP_M` records
it (F-015).

#### The new box

`TARGET_RANGES = ((0.36, 0.48), (−0.08, 0.08), (0.50, 0.62))` m from the environment origin: same
12 × 16 × 12 cm as before, moved forward and down so it sits below the resting tip rather than around it.
Chosen from a CPU search over box centres that scored 0% free successes and ≥99% reachable, then re-scored
against the measured stance.

| | Old box | New box |
| --- | --- | --- |
| Start distance, model (min/mean/max) | 0.4 / 9.8 / 18.3 cm | 14.0 / 21.9 / 30.2 cm |
| Within 5 cm of the resting tip | 8.6% | **0.0%** |
| Reachable, clear and holdable | 100% | 99.1% |
| Simulator zero-action baseline | 12.9% (33/256) | **0.0% (0/256)** |

- [verify, 8 envs](#/week/1/run/20260915T235632_928071Z_verify_seed42): **23/23**, up from 21. Two new checks
  measure the resting tip in the simulator and require every point of the box, not just the drawn targets, to
  be further than 5 cm from it: minimum distance to the box 14.3 cm, 0 of 8 sampled targets within 5 cm, and
  the resting tip within 3.1 mm of the recorded stance.
- [Zero-action baseline, 256 envs](#/week/1/run/20260915T235714_363411Z_smoke_seed42): final error at 9 s
  13.5–28.7 cm, mean 21.5 cm, **0 of 256 within 5 cm**, 0 failure resets. The base settles at 0.2737 m and
  −0.0550 m, matching the probe.

![Pincer tip workspace against the new target box](figures/d1_workspace.png)

A second `verify` run from the committed configuration ([23/23](#/week/1/run/20260916T000418_054709Z_verify_seed42))
reproduces the first. CPU tests: 40 pass in `env_isaaclab`, 16 in `tests/test_evidence.py` on the system Python.

What it shows:

- **Reaching is no longer free.** Zero actions score 0/256 where they scored 33/256. Every target needs at
  least 13.5 cm of tool-point motion, so a reach metric now measures reaching.
- **Episodes start standing.** Peak tilt at reset falls from 18.6° to 4.3°, and the lowest base height rises
  from 0.197 m to 0.260 m, well clear of the 0.15 m termination.
- **The box is still a small, fixed-target workspace**, as plan stage 5 asks: 12 × 16 × 12 cm, one target per
  episode, and 99.1% of it reachable, clear of the body proxy and statically holdable.

What it does not show:

- **No policy has been trained against this box.** The three PPO pilots used the old box, the old spawn and
  the Link6 control point, so their numbers do not carry over. Reaching ability is still unmeasured.
- **The slide remains.** The base still settles 5.6 cm behind where it spawns and takes 3.6 s to get there,
  inside a 10 s episode. This moves the robot relative to a world-fixed target for the first third of every
  episode. It is measured and repeatable, not removed.
- **Still not the G1a evaluation.** This is a zero-action snapshot at 9 s, not the frozen-manifest evaluator
  with a 1 s dwell.
- **The box never requires the base to move**, and it is not a validated choice of difficulty — it is the
  nearest placement that removes free successes while staying reachable.

#### Code changes

| Change | Detail |
| --- | --- |
| `position_only/task_space.py` (new) | `SPAWN_HEIGHT_M`, `ZERO_ACTION_BASE_OFFSET_M`, `ZERO_ACTION_TIP_M`, `TARGET_RANGES`, with no imports, so the task and the CPU tools read one copy. `mdp.py` and `workspace.py` each held their own copy of the box before |
| `env_cfg.make_cfg` | `spawn_height` and `target_ranges` arguments; the spawn height overrides the init state after `make_robot_cfg`, leaving playback's `ROBOT_START_POS` alone |
| `run_position_only.py` | `--spawn_height`; `run.json` records `reset.spawn_height_m` and `target_box_env_frame_m` |
| `verify.py` | `target_box_needs_arm_motion` and `resting_tip_matches_recorded_stance` (23 checks) |
| `workspace.py` | Reports and plots the measured resting tip beside the model's, with the model error; histogram bins follow the data instead of stopping at 22 cm |

## Results

Runs recorded this week appear under **Runs** below these notes, with their curves: 11 smoke, 11 verify, 3 PPO pilots,
6 playback runs and 1 viewer replay on 2026-09-15. Figures: [smoke posture](figures/smoke_posture.png),
[D1 workspace](figures/d1_workspace.png) and the replay captures. External evidence: [Rescue flat ablation](external/rescue_flat_ablation/README.md).

## Findings this week

- [F-001](../findings.md): superseded by F-012 (forward walking with the arm reproduced; lateral and yaw added).
- [F-002](../findings.md): superseded by F-010 (the droop figures came from acceleration drives).
- [F-003](../findings.md): CPU frame maths and PPO configuration work (confirmed, interface only).
- [F-004](../findings.md): primary implementation choice (provisional until the end of Week 2).
- [F-005](../findings.md): stock Isaac Lab Go2 motor model versus Unitree's measured envelope (confirmed, model comparison).
- [F-006](../findings.md): what unitree_rl_lab/MaiRo offers for sim-to-real, and what it does not show (provisional).
- [F-007](../findings.md): the D1 and latency model rests on published torques, SDK timing and two estimates (provisional).
- [F-008](../findings.md): in Rescue's flat locomotion ablation, training with the arm attached degraded the gait (provisional).
- [F-009](../findings.md): the task's interface, timing, terminations and partial resets pass deliberate checks in Isaac Sim (confirmed, mechanism only).
- [F-010](../findings.md): the URDF import made the D1's joints acceleration drives, so the arm sagged 0.11 rad inside its limits; now force drives (confirmed).
- [F-011](../findings.md): superseded by F-013 (its start distances assumed the base at the environment origin).
- [F-012](../findings.md): playback reference; the arm leaves forward and lateral walking intact but doubles tilt when turning at 1 rad/s (provisional).
- [F-013](../findings.md): with the pincer tip as the controlled point, zero actions still meet 5 cm for 12.9% of targets (confirmed; its mechanism corrected by F-014, and the box it describes replaced by F-016).
- [F-014](../findings.md): the backward slide at reset is the zero-action posture settling, not the drop; it is repeatable between environments to 0.005 cm (confirmed).
- [F-015](../findings.md): the CPU workspace model puts the resting pincer tip 1.6 cm from where the simulator rests it, missing the arm's sag and the base's 1° tilt (confirmed).
- [F-016](../findings.md): moving the target box below the resting tip removes the free successes — zero actions score 0 of 256, and the spawn drop is gone (confirmed).

## Issues and risks

- **The target box never needs the base to move.** The free successes are gone (F-016: 0 of 256), but the box is a
  small fixed volume in front of a standing robot, so P0 is still a stance-and-reach task. Base-motion targets are
  plan stage 6 and are not designed yet.
- **The box's difficulty is not validated.** It is the nearest placement that removes free successes while staying
  reachable (F-016), chosen from a CPU search, not from any evidence about what a policy can learn. If PPO cannot
  make 13.5–28.7 cm reaches, the box, the ±1 rad action clip and the reward scales all become suspects at once.
- **The pincer tip is CAD geometry.** It has not been measured on the arm. Grasping tasks would need the point between
  the pincers instead.
- **Two arm models are now in use.** Playback (`run_sim.sh`, F-001/F-012) and Rescue's training keep the import's
  acceleration arm drives, while the task uses force drives (F-010). Comparisons across them must say which one.
  Whether the floppier arm contributed to F-008 is untested.
- **Unexplained J3 residual.** With force drives J3 still sits 0.010 rad off at rest, about 4 mm at Link6.
- **Arm-to-body contact is not penalised.** Self-collisions now stop the arm, but the touch does not terminate the
  episode or cost reward, so a policy could learn to lean the arm on the body.
- **The base still slides back at reset, and the slide is the posture, not the drop** (F-014). The drop is gone:
  the spawn is 0.30 m, peak tilt 4.3° and the lowest base height 0.260 m against the 0.15 m termination. But the base
  still settles 5.6 cm behind where it spawns and takes 3.6 s of a 10 s episode to get there, moving the robot
  relative to a world-fixed target for the first third of the episode. It is repeatable between environments to
  0.005 cm, so it is a fixed offset rather than noise, and the target box is placed against the settled stance.
  Lowering the spawn further does not help: at 0.28 m the slide is unchanged.
- **Geometry must be measured, not modelled** (F-015). The CPU workspace model puts the resting tool point 1.6 cm
  from where the simulator rests it, missing the arm's sag and the base's 1° tilt. `verify` fails if the simulator
  drifts more than 2 cm from the recorded stance, but the recorded stance is one configuration; changing the arm
  gains, the leg model or the mass would move it and the box would need re-placing.
- **Unverified estimates.** The D1 speed limits and the leg delay range are unmeasured, and the 10 Hz arm hold means
  arm targets update five times less often. Any reaching result under `--latency estimated` depends on those
  assumptions until they are measured.
- **Arm-on training degrades gaits** in Rescue's locomotion task (F-008). P0's moving stage can hit the same failure
  while still tracking velocity well, so tracking metrics alone will not catch it. Yaw is where the arm shows first
  in playback (F-012).
- **Training metrics are diagnostics.** `Metrics/ee_position/position_error_m` is sampled at episode end during
  training. The G1a evaluation needs the frozen-manifest evaluator, with a zero-action baseline beside every result.
- **Disk.** 34 GB free as of 2026-09-16, up from 3.6 GB; the space was freed outside this record. Each run still
  rebuilds its ~10 MB USD, so long training runs need log housekeeping.
- Resolved on 2026-09-15: the GPU queue (the GPU was free all session), and "configuration changed before its first
  simulator run" (it has now run; see the log).
- Resolved on 2026-09-16: "the target box does not test reaching" (F-016: zero actions now score 0 of 256) and
  "every episode starts with a drop" (the spawn is now the standing height). Both left residues, listed above.

## Next week

The box and reset are done (F-016), so the simulation track's remaining Week 1 items carry forward: workspace
validation against an IK reference, a frozen-manifest evaluator reporting a zero-action baseline (now 0%) and
G1b's gait metrics, and the first PPO pilots against the new box with reward-term inspection. The three Week 1
pilots predate the box, the spawn and the pincer tip, so P0 training starts from scratch.

The 16 September replan adds a second track that has not begun: RealSense and AprilTag bring-up, D1 telemetry,
arm response and box-load measurements, and the access and fixture needs that Week 2's frame calibration and
G4 deployment contract depend on. Week 2 also has to freeze the task, interface and tolerance manifests and
select the implementation from bounded reproductions.

# Week 1 — 14–20 September 2026

**Status:** in progress
**Focus:** Finish environment/evaluator work; begin method reproduction, camera setup and hardware measurements

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Close remaining G0 checks, validate workspace/reset changes, start the frozen evaluator, and trace UniFP plus the Go2+D1 position-only reference.
- **Hardware and camera:** Bring up RealSense/AprilTags and D1 telemetry; measure basic arm response and the real box mechanism; establish robot, fixture, GPU and storage access.
- **Evidence and deliverable:** Record reproduction attempts and interface measurements. Freeze the task and deployment contract in Week 2.
- **Gate focus:** G0; preparation for G1a and G4.

**Where the two tracks stand, 16 September.** Simulation: the workspace and reset were resolved (F-016) and the
frozen evaluator is written and measured (F-018); the UniFP and Go2+D1 reference traces have not been started,
and neither repository is cloned here. Hardware and camera: **the D1 half is started, the camera half is not**.
This PC has no RealSense (no Intel USB device, no `pyrealsense2`), no serial device and no wired link to the
robot. The arm, however, is reachable through the Go2: later on 16 September the dog was up on Tailscale with
the D1 welded and powered, and its interface was measured from the Jetson payload (F-020 to F-023, sixth-session
log below). RealSense, AprilTags, the box mechanism and any camera/tag/tool frame work remain not started, and
Week 2's G4 still depends on them.

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
- [x] Visible 1-environment run inspected (2026-09-16): `view --episode` replays a pinned manifest episode; seed 43 on development ep94. It found two things 300 evaluated episodes had not — the robot walks 20 cm to its target (F-043) and oscillates at 5–8 Hz while holding (F-044)
- [x] Model the arm's measured command latency and ramp (2026-09-17, F-045, F-046). Fitted from the raw samples, the ~127 ms and ~220 ms figures were feedback-sampling artefacts: 10 ms dead time, 15.5/17.4 rad/s², restart from rest on each setpoint. `--arm_trajectory measured` is the default; `verify` 25/25; manifests re-frozen
- [ ] Retrain the three seeds under `--arm_trajectory measured` and re-measure falls, precision, oscillation and base travel (F-046)
- [ ] Hardware: re-send an identical setpoint at 10 Hz through one 30° step to settle whether the D1 restarts its plan on an unchanged setpoint (F-045, F-046)
- [ ] G4 contract: decide whether the deploy stack streams the policy's arm output at 10 Hz, given streaming costs the arm a third of its speed (F-046)
- [ ] Price base translation, once the timing model is right (F-043)
- [x] Self-collision check: no resting contact from overlapping weld shapes (measured headless, with a positive control; a screenshot is optional)
- [x] Deliberate reset tests: time limit, low base, tilt, base contact, workspace exit, partial reset (`verify`, F-009)
- [x] Controlled point chosen: the Link7_1 pincer tip (CAD end-face centre; `--tip_body`, `--tip_offset`), matched in simulation to 0.9 µm (F-013). Not measured on the physical arm
- [x] Target box checked against the reachable, collision-free workspace: reachable, but zero actions meet 5 cm for 12.9% of targets (F-011, then F-013)
- [x] Playback reference: bare Go2 and Go2+D1, with forward, lateral and yaw commands (F-012)
- [x] Short PPO pilot: 64 envs × 24 steps × 100 iterations (153,600 transitions); throughput and memory recorded (three pilots)
- [x] Rescue flat ablation recorded as external evidence (F-008)
- [x] Frozen-manifest evaluator written, with development/validation/test manifests and the zero-action baseline measured on all three (2026-09-16, F-018)
- [ ] Gait-quality metrics (foot vs neutral point, front–rear spacing, backward after request, turn tracking at 0.2/0.5/1.0 rad/s, pitch wobble) added to the P0 evaluator design for G1b
- [x] Run the evaluator against a trained checkpoint (2026-09-16, F-019): `model_1499` scores 100/100 on the development manifest against 0/100 for zero actions. The truncation and fall paths are still untested by a real failure, because this policy never fell
- [x] Cartesian (IK) controller for the arm: `d1_ik.py` solver and `d1_hardware.py` client (2026-09-16, F-025). **Executed on hardware**: an 11.3 cm Cartesian move tracked to a 6.0 mm residual, which is the arm settling rather than the solver (F-026). Absolute Cartesian accuracy still unvalidated pending the J1/J2 zero
- [~] Hardware and camera half of the revised Week 1: D1 telemetry and arm response **done** on the real arm via the Go2 (F-020, F-021, F-022, F-023, 2026-09-16); RealSense/tags, box loads and fixture needs still not started
- [x] Move the target box (or change the reset height) so zero actions do not meet the 5 cm criterion, before P0 training (F-013): box moved forward and down, spawn lowered to 0.30 m; zero actions now score 0/256 (2026-09-16, F-014, F-015, F-016)
- [x] Price the squat: `base_height_l2` at weight −50 against the measured stance; retrained seed 42 and re-evaluated (2026-09-16, F-040). Squat gone (0 of 100 episodes below 0.20 m), tracking 30% better, 1 fall in 100 at a corner target
- [x] Three seeds (42/43/44) at the frozen budget, evaluated on development and validation (2026-09-16, F-041). **G1a fails on falls**: 3% / 0% / 3% against ≤1%. Failures are far-reach tilt terminations; seed 43 is clean
- [x] Reduce the tilt failures at far reach (2026-09-16, F-042): `base_angular_motion_l2` at −0.2 — rotation had never been priced while translation was. Falls 4 → 1 across 300 development episodes, seed 44 fixed outright, leg saturation effectively gone
- [ ] Draw a **fresh validation manifest** on a new RNG stream, versioned beside the existing one, and measure G1a on it. The current validation set is contaminated: F-041's diagnosis was read off its episodes and F-042's reward term followed from that diagnosis (F-042)
- [ ] Diagnose the remaining tip-while-holding failure (seed 42 ep62, far bottom corner, 4.92 s dwell then tipped). The arm's missing command latency (F-021) and acceleration ramp (F-035) bear on holding behaviour (F-042)
- [ ] Grow the target range beyond the first box: base-motion targets and a growth schedule for plan stage 6 (the 12 × 16 × 12 cm stage-5 box is set; `workspace.py` scores candidates against the measured stance)
- [x] D1 joint speed limits measured on hardware and `motor_model.py` updated: 1.20–1.29 rad/s on every joint, no 1.05/1.73 split (2026-09-16, F-033)
- [x] Validate the servo signs on hardware with an observer (2026-09-16, F-034): J0 and J3 inverted, J4/J5 correct, J1/J2 indirect. `SERVO_SIGN = [-1,1,1,-1,1,1]`
- [ ] Give `params/deploy.yaml` a per-joint sign before G4 freezes the deployment contract — it would currently mirror two joints (F-034)
- [ ] Measure the joint **zero offsets** (signs are done; zeros are not — F-023, F-025)
- [ ] Explain the 0.010 rad residual at J3 with force drives (F-010)
- [ ] Decide whether playback (`run_sim.sh`) should also get force arm drives; it would change the F-012 reference

### Added by the 16 September plan revision

- [x] Frozen-manifest evaluator started, including zero-action reference and terminal metrics (F-018, F-019)
- [ ] UniFP and one position-only reference reproduction attempts started with a setup time budget
- [ ] RealSense stream and AprilTag detections recorded
- [x] D1 feedback/command timing and basic joint response measured on hardware (2026-09-16): angle feedback 111.0 ms / 9.00 Hz and status 100.5 ms / 9.96 Hz (F-020); J0 step latency 126–138 ms, peak 1.15 rad/s, steady-state error ≤0.1° (F-021). One joint, unloaded
- [ ] Box latch/lever loads, travel, geometry and tool engagement measured
- [~] Robot access recorded (2026-09-16): `sshuni` → `unitree@100.99.23.36` over Tailscale, arm at 192.168.123.100 on `enP8p1s0`, dog's host `python3` carries `cyclonedds` 0.10.2. Fixture availability and recording storage budget still open

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

### 2026-09-16 · revised B/C plan: reuse, tagged box demonstration and C refinements

- Revised the [plan](../../docs/thesis_b_plan.md) at Lukas's request: target a complete
  AprilTag-guided real Go2+D1 box-opening sequence in B, with markerless perception and
  broader refinements in C. First integrated attempt is targeted Week 6; Weeks 7–9
  support repeatability and comparisons, followed by the Week 10 report/demonstration.
- B prioritises pressing for force calibration and the box/lever as the application.
  Additional tasks and autonomous tool changes are conditional. Source reuse centres on
  UniFP, the Go2+D1 position-only reference, UMI-on-Legs and the existing robot interfaces.
- Updated all ten weekly plans and the [gate record](../gates.md), adding G4–G7 for
  measured observations, physical reaching, contact and integrated repeated trials.
  Task/interface definitions move to Week 2 and contact calibration to Week 3.
- This is a planning/documentation change. Existing dated logs, checked items and gate
  statuses are retained. No new simulation, training, camera or hardware result is claimed;
  workspace/reset findings F-014–F-016 remain separately recorded experimental evidence.

### 2026-09-16 · Frozen-manifest evaluator, and the zero-action baseline G1a is read against (fifth session)

The revised plan's Week 1 asks to "start frozen evaluator". It is written and running. Hardware is not
available on this machine today, so the hardware and camera half of Week 1 stays untouched.

#### What it measures

`run_position_only.py eval --manifest <file>` runs every episode of a frozen manifest under one
controller with deterministic actions, and reports what the plan's measurement section asks for:
reach success with a continuous dwell, survival, falls, error norm, RMS, 95th percentile, time to
reach, base tilt and height, joint-limit and effort figures, with transient and final-two-second
tracking kept apart. Without `--checkpoint` it evaluates zero actions, which is the baseline every
reach number has to be read against.

Three rules are enforced in code rather than left to the writer, because each one can quietly flatter
a result. They have unit tests (`tests/test_evaluate.py`, 29 tests) built around exactly those cases:

- **Failed episodes stay in the success denominator.** `error_all_episodes` is the reported figure;
  `error_surviving_episodes_only` exists but is labelled "for diagnosis only".
- **Truncated traces are not padded.** An episode that ends early keeps the samples it produced; the
  missing tail is not filled with the last good value, and truncated episodes are counted separately.
- **The dwell must be continuous.** Four separate 0.5 s visits do not add up to G1a's 1 s.

A terminated environment is reset inside Isaac Lab's `step()`, so the state read after that step
belongs to the next episode. The terminating step's sample is discarded and the episode's last valid
sample is the one before it, 20 ms earlier. An episode that ends early cannot be a success however
close the tool point got, because G1a requires surviving to the end.

#### Three manifests, and a guard on the conditions

`manifest` mode writes a versioned, hashed episode set: `development` (free to look at),
`validation` (checkpoint selection only) and `test` (untouched until the final result), 100 episodes
each, drawn from independent RNG streams. The file fixes the targets *and* the conditions the
episodes run under — spawn height, target box, robustness, latency, both actuator models,
self-collisions, tool point, episode length, success radius and dwell. Loading re-checks the hash, so
a hand-edited manifest is rejected.

| Role | Seed | `content_sha256` |
| --- | --- | --- |
| development | 20260916 | `e2e0d3e6a566…` |
| validation | 20260917 | `8754dc71f8a0…` |
| test | 20260918 | `27ef0f8569ea…` |

Every evaluation compares the run's actual conditions against the manifest's and records any
mismatch in `eval.json`. [A deliberate check](#/week/1/run/20260916T005431_232893Z_eval_seed42) ran
the development manifest at the old 0.42 m spawn with the old `implicit` arm drives: both deviations
were named, and the mismatched run reported **RMS 16.6 cm against the correct 21.7 cm**. A sagging
arm drops the tool point toward a box that sits below it, so the wrong configuration looks 5 cm
*better*. That is the whole reason for the guard.

#### The zero-action baseline

100 episodes per manifest, zero actions, 50 environments per batch, about 14 s each:

| Manifest | Success | Falls | Truncated | RMS | 95th pct | Final 2 s | Lowest base | Peak tilt |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| [development](#/week/1/run/20260916T005258_970617Z_eval_seed42) | **0/100** | 0 | 0 | 21.7 cm | 23.1 cm | 21.7 cm | 0.260 m | 5.11° |
| [validation](#/week/1/run/20260916T005318_737159Z_eval_seed42) | **0/100** | 0 | 0 | 21.6 cm | 23.1 cm | 21.6 cm | 0.260 m | 5.11° |
| [test](#/week/1/run/20260916T005338_474757Z_eval_seed42) | **0/100** | 0 | 0 | 22.2 cm | 23.6 cm | 22.1 cm | 0.260 m | 5.11° |

No episode reached within 5 cm at any point, in any of the 300. G1a's own thresholds are checked in
`eval.json` (`g1a.passed`), and this correctly reports `false`. The three sets agree to 0.6 cm of RMS,
so the development/validation/test split is balanced rather than three different difficulties.

#### Finding: the arm's commanded torque saturates while the robot stands still (F-017)

The first evaluation reported the arm at its effort limit in 100% of steps, which sits oddly against
F-010's measured joint loads of about 1.1 N·m against 1.7–3.3 N·m limits. Both are true, because they
are different quantities. The arm is an `ImplicitActuator`, where PhysX runs the PD itself and Isaac
Lab's `applied_torque` is only its own Python-side estimate clipped to the limit — the class calls
them "approximate torques … since PhysX does not expose this quantity". Measured at rest, zero
actions, settled:

| Joint | Isaac Lab `computed_torque` | `applied_torque` (clipped) | Limit | PhysX reaction torque |
| --- | --- | --- | --- | --- |
| Joint2 | −4.46 N·m | −3.30 N·m | 3.3 N·m | 1.10 N·m |
| Joint3 | −25.80 N·m | −1.70 N·m | 1.7 N·m | 1.08 N·m |
| Joint5 | 2.87 N·m | 1.70 N·m | 1.7 N·m | 0.29 N·m |

With 4000 N·m/rad standing in for the D1's servo loop, J3's 0.011 rad residual alone demands 44.9 N·m
before damping, against a 1.7 N·m limit. So the commanded torque saturates on four of six arm joints
with the robot doing nothing at all. An "effort saturation" metric built on `applied_torque` would
report 100% saturation for a motionless robot and tell a reader nothing about a policy. The evaluator
now reports `arm_commanded_effort_at_limit_frac` (named as commanded, with the caveat in the JSON)
beside `peak_arm_joint_torque_nm` from PhysX's reaction torques. The legs use an explicit actuator, so
their figures are the model's own clipped output and need no such caveat.

What it shows:

- **The measurement G1a needs exists**, with its rules tested and the zero-action reference measured
  on all three manifests.
- **The comparison is protected**: manifests are hashed, conditions are checked, and a wrong
  configuration is caught rather than silently reported.

What it does not show:

- **No policy has been evaluated.** Every number here is zero actions. The evaluator has never been
  run against a trained checkpoint, so its behaviour on a moving robot is untested — in particular
  the truncation and fall paths have not been exercised by a real failure, only by unit tests.
- **One simulator seed.** Per-seed spread across training seeds 42/43/44 is a G3 item, not done.
- **The arm's gains are a modelling choice**, not an identified servo loop (F-002, F-007). The
  saturation result describes this model, not D1 hardware.

### 2026-09-16 · First policy trained against the revised box: it reaches by squatting (fifth session, continued)

The revised box (F-016) had never been trained against, so its difficulty was unvalidated. It is
learnable, and the way the policy learns it is the result worth keeping.

#### Throughput first

The pilots measured 3,787 steps/s at 64 environments, but the box, the spawn and the controlled point
have all changed since. A [probe](#/week/1/run/20260916T013722_481886Z_train_seed42) of
`train --headless --num_envs 2048 --iterations 20 --seed 42` gave **86,331 steps/s**, 0.57 s per
iteration, so 1500 iterations costs about 15 minutes rather than the pilots' 153,600 transitions.

#### The candidate run

`python run_position_only.py train --headless --num_envs 2048 --iterations 1500 --seed 42`
([run](#/week/1/run/20260916T013817_908786Z_train_seed42)): 73,728,000 transitions, 13 min 27 s,
88,020 steps/s, peak GPU 4,673 MiB of 16,376. Tilt failures behaved as the pilots did — 19.2% of
episodes terminating on `bad_orientation` at iteration 20, 0% by iteration 300 — and from iteration
300 every episode ran to its time limit.

| Iteration | Mean reward | Episode-end reach error | Mean episode length | `bad_orientation` |
| --- | --- | --- | --- | --- |
| 20 | 10.65 | 0.471 m | 452.7 | 19.2% |
| 300 | 32.33 | 0.0107 m | 500.0 | 0.0% |
| 600 | 33.66 | 0.0079 m | 500.0 | 0.0% |
| 900 | 33.88 | 0.0059 m | 500.0 | 0.0% |
| 1500 | 34.00 | 0.0060 m | 500.0 | 0.0% |

Those are training-time diagnostics sampled at reset, which the plan says do not pass G1.

#### The frozen-manifest measurement

`eval --headless --num_envs 50 --manifest results/manifests/development.json --checkpoint
…/model_1499.pt` ([run](#/week/1/run/20260916T015224_579606Z_eval_seed42)), development manifest
`e2e0d3e6a566…`, no condition mismatches:

| | Zero actions | model_1499 |
| --- | --- | --- |
| Success (5 cm, 1 s dwell, survived) | 0/100 | **100/100** (Wilson 96.3–100%) |
| Falls | 0 | 0 |
| Truncated | 0 | 0 |
| RMS error | 21.7 cm | 2.11 cm |
| 95th percentile | 23.1 cm | 1.37 cm |
| Final 2 s mean | 21.7 cm | 0.52 cm |
| Mean time to reach | — | 0.135 s |
| Mean max dwell | 0.0 s | 9.85 s |
| Lowest base height | 0.260 m | **0.159 m** |
| Peak tilt | 5.11° | 17.38° |
| Legs at effort limit | 0.0% of steps | 9.8% of steps |

RMS (2.11 cm) exceeds the 95th percentile (1.37 cm) because RMS is dominated by the first 0.135 s,
before the tool point arrives; the final-2 s figure is the steady-state one.

#### It passes the criterion, and it should not be the P0 baseline

G1a's arithmetic is met on this manifest. The posture is not a stance-and-reach:

- **All 100 episodes drop the base below 0.20 m**, median 0.172 m, against the `low_base`
  termination at 0.15 m. The margin is **9 mm at worst, 22 mm at the median**.
- **The squat tracks the target.** Correlation between target height and lowest base height is
  **+0.50**, and between target *distance* and lowest base height **−0.50**: lower and further
  targets are met by squatting further. Lowest-half targets average 0.169 m of base height,
  highest-half 0.175 m.
- **It reaches in 0.10–0.18 s.** Arm joints are limited to 1.05–1.73 rad/s and arm commands are held
  for 5 policy steps at 10 Hz, so one or two arm commands have issued by then. The body is doing the
  early work, not the arm.
- **The legs pay for it**: saturated for up to 30.1% of an episode's steps, peak 23.4 N·m, exactly
  the Unitree envelope's Y2 limit (F-005).

The mechanism is available because the reward has no base-height term at all: `upright`
(`flat_orientation_l2`, −1.0) penalises tilt and `base_motion` (−0.2) penalises velocity, but nothing
opposes a slow, level squat. F-016 moved the box forward and *down* to remove the free successes, and
lowering the whole arm by crouching is the cheapest way to follow it. `alive` (+0.5) against
`failure` (−2.0) then teaches the policy to hover just above the termination rather than on it.

What it shows:

- **The revised box is learnable**, and the evaluator works against a trained checkpoint, not only
  zero actions. This clears the Week 1 worry that the box's difficulty was unvalidated.
- **The zero-action baseline separates cleanly**: 0/100 against 100/100 on the same frozen episodes.

What it does not show:

- **Not a G1a pass.** One seed, one manifest, `--robustness none`. G1a wants three seeds, and G3 wants
  matched budgets; neither is done. The `development` manifest is the one that may be looked at, so a
  number measured on it cannot be the reported result.
- **Not a usable P0 baseline as configured.** A policy living 9 mm above its fall threshold with legs
  at their torque limit for a third of an episode will not survive G5's "no falls or limit violations"
  on hardware, and it makes P0's reaching largely leg work, which would confound the P0–P4 comparisons
  those policies exist to make.
- **The truncation and fall paths are still untested by a real failure.** This policy never fell.
- **Squatting is not itself cheating.** The plan allows stance and posture changes for whole-body
  coordination. The objection is the 9 mm margin, the leg saturation and the confound, not the crouch.

### 2026-09-16 · First hardware session: D1 arm telemetry, J0 step response and the protocol's real behaviour (sixth session)

The Go2 was reachable over Tailscale (`sshuni` → `unitree@100.99.23.36`) with the D1 welded and powered, so the
hardware half of Week 1 started. The dog was **sitting** throughout and nothing on the legs was touched: arm only.

**Route in.** The arm answers at `192.168.123.100` on the Jetson payload's `enP8p1s0` (0.65 ms, 0% loss). There is
no serial link and no RealSense on this route; the arm speaks CycloneDDS on domain 0. Two paths exist to it — the
C++ samples in `~/d1Arm` on the dog, and the VIP-Rescue driver `unitree-d1-control` (pinned at `b9d3b8a`, the same
commit `VIP-Rescue/Docker/unitree-d1-control` points to, mounted into the `vip-arm` container). The `vip-arm`
container was **not running**; `vip-livox`, `vip-realsense`, two zenoh bridges and the `p2v-*` services were. The
dog's host `python3` already has `cyclonedds` 0.10.2 — the same version the arm's message headers were generated
with — so the measurements were taken with small standalone scripts against the driver's protocol, without
starting the container or disturbing the running stack. Commands are **JSON strings** on `rt/arm_Command`
(`ArmString_`), not the `SetServoAngle_` struct, and angles are **degrees**.

**Telemetry, read-only** ([run](#/week/1/run/20260916T043123_d1_telemetry)). Nothing was published during this
capture. Over 120 s, `current_servo_angle` delivered 1080 samples at **8.9986 Hz, median period 111.016 ms,
stdev 0.516 ms**. A 45 s capture split `rt/arm_Feedback` into two streams: `funcode 1`, the seven joint angles,
at **8.9852 Hz / 111.009 ms**, and `funcode 3`, the enable/power/error status, at **9.9614 Hz / 100.455 ms**.
So the angle cycle is 111 ms and the 10 Hz cycle carries status only (F-020). The servo floats are **float32**
on the wire: the shipped driver's dataclass uses Python's bare `float` (float64), and its "primary" angle path
would fall back to JSON parsing rather than decode them. Angles quantise to 0.1° with a resting noise stdev of
≤0.047°, which sets the resolution floor for everything below. Resting pose
**[2.1, −90.9, 91.7, −3.1, 4.1, 3.1, 40.8]°** — J1 and J2 sit outside the driver's own limit table (F-023).

**J0 step response** ([run](#/week/1/run/20260916T043641_d1_j0_step)). Lukas approved powering the arm and
moving **J0 only** (base rotation, ±135°, resting 2.0°), confirming that enabling holds the current pose. Steps
of ±5°, ±10° and ±20° about rest, each held 4 s, then J0 returned to 2.0° and the motors released. Measured on
both angle topics with agreeing results:

| Step | Command → motion | Peak rate | Settle | Steady-state error |
| --- | --- | --- | --- | --- |
| ±5° | 132.9–138.2 ms | 46.0–46.7 °/s (0.80–0.82 rad/s) | 133–138 ms | ≤0.10° |
| ±10° | 126.3–129.4 ms | 50.6–51.3 °/s (0.88–0.90 rad/s) | 238–353 ms | ≤0.08° |
| ±20° | 126.4–127.9 ms | 64.5–65.9 °/s (1.13–1.15 rad/s) | 349–350 ms | ≤0.10° |

Latency clustered at 126–138 ms across all six excursions. Peak rate was **still climbing at the largest step**,
so J0 never saturated and 1.15 rad/s is a lower bound — already above the 1.05 rad/s `d1_arm/d1.urdf` carries
with no recorded source (F-021). The joint holds within 0.1° and dithers ±0.1°, one quantisation step.

**What it does not show.** One joint, one posture, unloaded, with the arm folded so J0 turns the lightest inertia
it ever sees. Nothing here transfers to J1/J2 working against gravity or to a loaded gripper. Both numbers are
bounded by the 111 ms feedback cycle: a 20° move spans about three samples, so the peak rate is a lower bound and
the 127 ms latency an upper bound (true latency is that minus up to one sample period). This is interface
characterisation, **not** reaching ability and not a sim-to-real comparison.

**The protocol does not behave as its driver documents** (F-022). `funcode 5 {"mode": 1}` ("enable all") never set
`enable_status`, alone or as the per-joint `funcode 4`; the flag went 0→1 100 ms after the first *motion* command,
so the arm **enables itself implicitly** when a `funcode 1` arrives. Release (`funcode 5 {"mode": 0}`) works,
in 99.6 ms. `funcode 6 {"power": 1}` works, in 79.3 ms. `funcode 6 {"power": 0}` **does not**: three attempts, the
last sending 14,208 power-off commands over 5 s, left `power_status` at 1. `arm_control.py` documents that call as
an emergency stop and the GUI exposes it as an emergency power-off button, so **there is currently no software
stop for this arm** — the working abort is release, which drops torque but leaves the arm powered and
backdrivable. This matters beyond Thesis B: the driver is shared with VIP-Rescue. Raised with Lukas.

**State left behind.** J0 is back at its 2.0° resting angle, the motors are released (`enable_status=0`) and the
whole arm is within 0.1° of the pose it started in. `power_status` is **1** and could not be cleared in software;
it was 0 before this session. Mechanically that is the same released, backdrivable state as before — no joint is
holding torque — but the arm is energised until it is power-cycled.

**Model updated.** `motor_model.py` `D1_FEEDBACK_HZ` 10.0 → **9.0**, now labelled measured, which moves
`arm_feedback_period_steps` from 5 to 6 at the 50 Hz policy rate: simulated arm feedback had been refreshing one
policy step sooner than hardware allows. `tests/test_sim2real.py` updated to match; all 69 tests pass in
`env_isaaclab`. **Runs recorded before this change, including the F-019 policy, were trained and evaluated at
5 steps** and are not directly comparable to runs made after it. `D1_COMMAND_HZ` is unchanged at 10.0 and still
carries an unmeasured label — the arm's maximum accepted command rate was not tested, only that single commands
are honoured. The velocity limits are unchanged, with the hardware disagreement recorded against them.

**`verify` has not been re-run against this change.** `nvidia-smi` could not reach the driver in this session, as
in the first session of 2026-09-14, so no simulator could be launched. The 23/23 G0 pass of 2026-09-16 was measured
with `arm_feedback_period_steps = 5`, and one of those checks reads the arm feedback period directly. Run
`python run_position_only.py verify --headless --num_envs 8` before the next training run and record the result.

### 2026-09-16 · Is there a higher-rate mode? Full DDS surface enumerated (sixth session, continued)

Lukas asked whether the D1 has a higher-Hz mode that can be unlocked. Answered read-only, publishing nothing.

**The arm's complete DDS surface**, from the builtin `DCPSPublication`/`DCPSSubscription` readers across 36
participants ([`dds_topics.txt`](#/week/1/run/20260916T043123_d1_telemetry)): `rt/arm_Command`, `rt/arm_Feedback`,
`current_servo_angle`, `arm_zero`, plus three struct topics the JSON samples never use — `set_servo_angle` and
`set_servo_angle_control` (`SetServoAngle_` = `{seq, id, angle, delay_ms}`) and `set_servo_dumping`
(`SetServoDumping_` = `{seq, id, power}`, i.e. per-servo damping). **There is no `rt/arm_sdk` and no
`arm_LowCmd`**, so the per-joint kp/kd topic that `unitree-d1-control/haptic_control.md` hoped for is not on this
firmware, and the gravity-compensated teach mode that doc plans cannot be built the way it assumes. Watched for
20 s with the arm idle, all four struct topics were silent — no hidden internal high-rate stream.

**No published source describes a faster mode.** The community D1 SDK extension lists both feedback topics at
10 Hz, and Unitree told the Caltech group that 10 Hz is the arm's control cycle. That group tried what this
question is asking: 100 Hz commands "appeared to move more smoothly", and mode-0 `execution_time` was edited from
0.04 s to 0.1 s and 0.01 s without curing the shaking. Within minutes their arm stopped reaching commanded
positions and then stopped responding entirely, with servos "burning hot" on the Go2's own supply; they concluded
it was defective. Unitree support told them the D1 R&D team had been lost and its data deleted. The D1 is
discontinued. That is one arm they judged defective, so it is not proof that high-rate commanding kills a healthy
D1 — but it is the only public account of trying, and there is no replacement path for ours (F-024).

**What is actually adjustable** is per-command, not per-rate: `delay_ms` (funcode 1), and `execution_time` and
`mode` 0/1 (funcode 2, "small smoothing of 10 Hz data" versus "large smoothing of trajectory-use").
`set_servo_dumping` is undocumented and untested. Port 22 is open on the arm itself (192.168.123.100) and the
Caltech author describes SSHing in to edit execution time, so an on-device configuration surface probably exists;
it was **not** explored here.

**What this means for the task.** The command side is not rate-limited — 14,208 commands in 5 s were accepted
(F-022) — but feedback stays at 111 ms whatever the command rate, so closed-loop arm control is capped by the
observation rate regardless. Design for 9 Hz observation. No high-rate command test was run, and none should be
without deciding the risk is worth it.

### 2026-09-16 · Cartesian (IK) controller for the arm, validated read-only against hardware (sixth session, continued)

Lukas asked for an IK controller for hardware testing. `d1_ik_controller.py` already does this in simulation, but
it is Isaac- and torch-bound and `isaaclab` is not on the dog, so the solver could not follow the arm. It does,
though, keep a deliberate `KinematicsBackend` seam "if this ever needs to talk to a real arm again", and
`DirectD1` defines the client seam. Both are now filled for hardware.

**Two new modules.** [`d1_ik.py`](../../d1_ik.py) is a damped least-squares solver in plain numpy, built on the
FK already in `position_only/workspace.py` rather than a second copy of the kinematics — `forward()` returns each
arm joint's axis and origin in the base frame, so the geometric Jacobian is analytic. It defaults to the task's
controlled point (the Link7_1 pincer tip, F-013), handles position-only and full-pose targets, clamps every
iterate to the URDF soft limits, and optionally checks `clear_of_body`. [`d1_hardware.py`](../../d1_hardware.py)
speaks the measured protocol (F-020, F-022) and mirrors `DirectD1`'s method names, so simulator and hardware
differ only in transport. Neither needs Isaac; `d1_hardware` imports CycloneDDS lazily, so the solver and its
tests run on the system Python.

**Tests:** 19 new, 88 in the full suite in `env_isaaclab`, all passing. The analytic Jacobian matches central
differences to 1e-6 (linear) and 1e-5 (angular); position-only IK converges on 20+/25 random reachable targets
and full-pose on 12+/15; every solution lies inside the soft limits even for targets 1.5 m outside the workspace;
unreachable targets fail without NaN; the collision proxy refuses to command; and each commanded step respects
the step cap. `tests/test_d1_ik.py` and `tests/test_d1_hardware.py` need only numpy, so they run without Isaac.

**Against the arm, read-only** ([run](#/week/1/run/20260916T0500_d1_ik_bringup)). Deployed to the dog and run
there, since DDS needs the arm's subnet. FK of the measured resting pose puts the tool point at
**[0.1366, −0.0097, 0.2069] m** in the Go2 base frame, and the CLI correctly flags J1/J2 as outside the soft
limits (F-023). A dry-run approach to [0.30, 0.0, 0.30] planned in 8 iterations to 0.10 mm and rehearsed as **15
bounded 5° steps** to 0.05 mm, with the error falling monotonically. As a cross-check, the solver's zero-pose tool
point sits 1.6 cm from the simulated resting tip — the same gap F-015 attributes to sag and base tilt, so the new
FK agrees with the existing CPU model.

**Not done, deliberately.** **The arm was not moved.** Lukas approved J0-only motion earlier in the session;
unfolding the arm from its rest pose to a Cartesian target is a much larger motion with the robot sitting, and
that is a separate decision. Everything above is a software test or FK of a measured joint vector.

**The honest limit** (F-025): the solver is only as accurate as the URDF, and it assumes servo *i* reports
`Joint{i+1}` with the same zero and sign. The arm rests outside the limits both the URDF and the driver give
(F-023), so the zero point is not established, and a constant joint offset would shift every Cartesian number
without failing a single test here. There is no external measurement of the tool point — that is the camera and
tag work G4 needs and Week 1 has not started. So treat the output as joint-space commands that are safe to send,
not as calibrated Cartesian positions. One gotcha recorded in the module: `from __future__ import annotations`
must not be added to `d1_hardware.py`, because CycloneDDS resolves an `IdlStruct`'s field types by name at
runtime and PEP 563 breaks it.

### 2026-09-16 · The IK controller on the real arm: an 11 cm Cartesian move, and two defects it exposed (sixth session, continued)

Lukas gave permission to run a safe target. **Target [0.20, 0.00, 0.30] m** in the base frame, 11.3 cm up and
forward of the folded rest pose. Up was chosen deliberately: it is the one direction clear of both the trunk
(top at z = 0.06) and the ground whatever the dog's sitting posture, which is exactly what `clear_of_body` cannot
vouch for, since it models the standing trunk box.

**A defect found before moving anything.** The dry run exposed that `set_all_joint_angles` clamped to the
solver's **soft** limits (±81°). The arm rests at J1 = −90.9°, outside them, so the first command of any approach
would have been clamped to −81° — a 9.9° jump that silently defeats the step cap the whole safety argument rests
on. The wire now clamps to the **hard** URDF limits, which is the clamp's actual job (reject what the arm will
refuse), with the soft band left to the solver. Three tests pin it.

**The move** ([run](#/week/1/run/20260916T0530_d1_ik_execute)). The arm tracked it, in bounded 3° steps with the
error falling monotonically: tool **[0.1366, −0.0097, 0.2069] → [0.1984, −0.0013, 0.2943] m**, an achieved
displacement of **107.4 mm against 113.0 mm commanded**. Then it stopped improving at **6.03 mm** and ran all 200
cycles.

**Second defect: no stall detection.** 186 of those 200 cycles made no progress and kept commanding. `run()` now
stops on `reached` / `stalled` / `max_cycles` and says which.

**What the 6 mm actually is.** Not the solver: re-solving from the measured pose converges to **0.025 mm** and
asks for [+0.31, +0.26, −0.72, −0.06, −0.42, +0.03]°. The arm had simply landed up to **0.7°** from the last
command. The return leg explained it — parking to a fixed joint vector with the command held **constant**, the
worst joint error fell **3.00 → 2.20 → 1.20 → 0.40°** over four consecutive cycles. The arm was still settling.
A loop that re-solves every 111 ms re-commands before the arm arrives, so the residual is substantially settling
lag rather than a fixed deadband (F-026). The fix for accuracy is to hold and let it settle, not to tighten the
tolerance; below about 1 cm a continuously re-solving loop at this rate is not meaningful. 6 mm is well inside
G1a's 5 cm, so this does not threaten the position-only gate.

**A new primitive.** `approach_joints` (CLI `park`): a bounded joint-space move. Needed because the rest pose is
outside the joint limits, so IK cannot express it — solving for its tool point returns a different, less folded
configuration (J1 −74° against −91°).

**The arm is back as found.** Parked to the limit, then released; tool within **1.1 mm** of where it started.
Released, it settled *past* the commanded −89.9°/+89.9° to **−90.8°/+91.6°** — its mechanical rest. That makes an
encoder zero-point error less likely than a limit table 1–2° tighter than the mechanism (F-027), which is mildly
good news for F-025's caveat, though it is not proof and G4 still needs the zero measured externally.

27 tests now cover the solver, the client and the mover (11 + 16); 96 in the full suite in `env_isaaclab`, all passing.

### 2026-09-16 · Second target, faster — and the arm dropped (sixth session, continued)

A larger, faster move: **[0.25, 0.08, 0.32] m**, 183.8 mm of travel with a lateral component (J0 2.5° → 19.6°),
at 6°/step ≈ 54°/s against 3°/step ≈ 27°/s before, under the 65°/s of F-021. Before running it, a
settle-and-measure step was added to `run()`, specifically to test what F-026 had predicted: that holding the
command would shrink the residual.

**It reached, and the prediction was wrong** ([run](#/week/1/run/20260916T0600_d1_ik_faster)). The approach hit
its 5 mm tolerance at 4.78 mm, in 99 steps rather than the 10 the dry run predicted. Then, with **no commands
published for 3 s**, the error did not decay: it oscillated **5.93–6.68 mm** across 17 samples and finished at
**6.27 mm — worse than the loop end**. The settled joints sit up to **0.7°** from the last command, the same
joints and magnitude as the first move. So the residual is a persistent per-joint offset with a ±0.1° dither,
not settling lag (F-029, superseding F-026). What F-026 had read as settling was real but different: on the park
leg the arm was closing on its folded **mechanical stop**, where it does keep creeping in; in free space it does
not. Speed did not degrade accuracy — the faster move landed in the same 6 mm band. Treat ~6 mm as the practical
floor for open-loop joint commands to this arm; closing it needs a measured tool position (G4) or per-joint
calibration, not a tighter tolerance.

**Then the arm dropped, and it was this code that dropped it** (F-028). After the settle the CLI published
release (`funcode 5 {"mode": 0}`) as its normal end-of-move action, with the arm extended and holding at
J1 = −30.1°, J2 = +57.7°. The next reading, with nothing commanding it, was J1 = −90.9°, J2 = +92.8°: **J1 fell
60.8° and J2 35.1°**, an uncommanded drop to the folded mechanical stop. Lukas saw it and said so. The arm
reports `error_status = 0` and is back at the session-start rest pose within 0.4° per joint and ~1 mm at the
tool, so no damage is evident — but nothing has been inspected mechanically, and what it passed through on the
way down is not known.

**Why it happened.** The D1 has **no holding brake**. Release is a torque-off, so on an extended arm the only
working software stop *is* a fall. That refines F-022, which called release "the working stop" — it works, but
it does not hold. The first execution made it look safer than it is: release at J1 = −49.9° left the arm in
place, so holding on release is **pose-dependent**, not a property of the arm. The direct cause was mine:
`d1_hardware.py` released at the end of every `--execute` path, which is exactly backwards — releasing is an
emergency action, not a way to finish a move.

**Fixed.** The CLI now leaves the arm energised and holding and says so; `release` warns and refuses on an
unfolded arm without `--force`; `is_safe_to_release()` gates on J1 ≤ −80° and J2 ≥ +80°, a conservative folded
test bounded by the two observations (fell from −30°, held at −50°) rather than a measured threshold. Three
tests pin it against both real poses. 32 tests on the solver, client and mover.

**For the gates.** G5 requires "abort/hold behaviour demonstrated". Between this and F-022, **the D1 has no
software stop that holds position at all**: power-off is ignored and release is a fall. An abort must either
leave the arm energised and holding, or be planned from a pose where a fall is acceptable. That has to be
designed for, not discovered on the day.

### 2026-09-16 · Up-and-left: the arm went right, and the servo sign was wrong (sixth session, continued)

A third target, **[0.22, +0.16, 0.36] m** — 242 mm, up 15 cm with J0 swinging to 37.6°, nearly double the
previous move's lateral component. Before running it I stated the prediction out loud, because the servo-to-URDF
sign mapping is exactly what F-025 flags as unvalidated and Lukas was watching: **+y should be the dog's left**.

**It moved right** ([run](#/week/1/run/20260916T0630_d1_ik_left)). Lukas reported it immediately.

**Diagnosis.** Two explanations fitted: an inverted J0 sign, or the arm being physically mounted 180° about
vertical from what the model assumes. They differ in a single observation — a mount rotation flips **+x** as well,
so the arm would reach over the *tail* rather than the head. Lukas confirmed it reached **forward over the head**,
which leaves only the sign. `weld.py` mounts with identity rotation (`Quatf(1,0,0,0)`) and `workspace.forward`
does the same, and the two agree with each other to the 1.6 cm of F-015 — so the model is self-consistent and it
is the hardware mapping that is wrong (F-030).

**Fixed at the one place the mapping lives.** `d1_ik.SERVO_SIGN = [-1, 1, 1, 1, 1, 1]`, applied in
`to_servo_deg`/`from_servo_deg`. Applying it exposed a **latent unit bug**: `step_towards` compared a solver
result in URDF radians against a measured pose in servo degrees — harmless only while every sign is +1, and a
mirrored joint the moment one is not. It is now servo-space throughout, and clamping moved to
`servo_limits_deg`, because a sign flip swaps a joint's low and high. Four tests pin it, including the exact
regression: a +y target must command a **negative** J0.

**Re-run** ([run](#/week/1/run/20260916T0700_d1_sign_fix)). Same target, servo 0 now commanded **−37.30°**
against +37.63° before. Settled at 6.26 mm — a fourth point in the same 6–7 mm band (F-029), across two
directions and two speeds. **The corrected direction has not yet been visually confirmed.**

**What is still wrong, and it is bigger than the arm.** `position_only/deploy.py` maps `Joint1..Joint6` to motor
ids 0–5 **by index with no sign convention at all**, so `params/deploy.yaml` — the deployment contract G4 is meant
to freeze — carries the same error. A policy trained in simulation and deployed through it would mirror its J0.
The manifest needs a per-joint sign and every joint's sign needs measuring first. Left as a deliberate decision
rather than changed here, since it alters the contract. **J3–J5 signs remain unvalidated**: no motion so far
isolates a wrist joint, so orientation targets cannot be trusted yet.

**Process note.** This was found because the prediction was stated before the move and someone was watching. The
same check should be run for each remaining joint before G4 freezes anything.

**Arm state:** parked folded, released at the folded pose per the F-028 rule, settled and stable at
[+2.8, −90.8, +91.2, −2.9, +5.1, +2.5]°, `error_status = 0`. Full traces captured for every run this time.

### 2026-09-16 · Per-joint sweeps: the command path was the story, then the arm stopped responding (sixth session, continued)

Lukas asked for the remaining hardware checks, including the wrist signs. The autonomous half — per-joint speed
limits, latency and tracking — needs no observer, so it went first, with a `sweep` subcommand added to
`d1_hardware.py` (a pure `analyse_sweep` so the timing maths is unit-tested against synthetic traces, plus a
Jacobian-derived prediction of what each sweep should do to the tool, for a watcher to check).

**The first results were nonsense**, and two things were wrong. One was mine: the outbound analysis window had no
end, so it swallowed the return leg and read the steady-state error as the full amplitude. The other was the
finding ([run](#/week/1/run/20260916T0730_d1_sweeps)): peak rate came out at **14.5°/s, identical across joints
and directions**, when F-021 had measured J0 at 65°/s. The difference was the command path. F-021 used funcode 1;
the sweep and the whole Cartesian controller used **funcode 2 mode 1**. Same joint, same 30°, same pose:

| path | peak rate | latency | settle | steady-state error | hold band |
| --- | --- | --- | --- | --- | --- |
| funcode 2, mode 1 | 13.5 °/s | 202 ms | not reached in 3 s | **−5.09°** | 13.1° |
| funcode 2, mode 0 | **69.3 °/s** | 93 ms | **538 ms** | **−0.20°** | **0.00°** |
| funcode 1 | **70.5 °/s** | 61 ms | **505 ms** | **−0.20°** | 0.10° |

Unitree documents mode 0 as "small smoothing of 10 Hz data" and mode 1 as "large smoothing of trajectory-use".
Mode 1 is a slow interpolator, so commanding it once per feedback cycle re-commands a waypoint the arm is still
slewing toward, and it never arrives. **Everything this session had attributed to the hardware was this**: the
99–117 step approaches against a predicted 10–14, the stalls, and the "0.5–0.7° tracking offset". Mode 0 is now
the default and the same Cartesian move takes **13 steps** (F-031, superseding F-029). Lukas, watching, confirmed
the fast path was much better and asked to keep it.

A residual did survive: the loop still stalls around **4.95 mm**, the arm sitting 0.3–0.7° from the last command,
and neither silence nor streaming the final command for 3 s closes it — while a single 30° command lands within
0.20°. That points to a small-increment deadband. It was being measured when the arm stopped.

**Then the arm stopped responding** (F-032). Sweeps of 0.5, 1, 2, 5 and 10° on J0 all returned peak **0.0 °/s**
with the joint never leaving its start angle, on **both** command paths. Throughout it reported
`power_status = 1`, `enable_status = 1`, `error_status = 0`, with unbroken 9 Hz feedback — healthy in telemetry,
inert in fact. This is the failure the Caltech report describes (F-024) and it is the first time this project has
seen it. The trigger is not established. A plausible contributor is ours: the `settle --hold-stream` loop re-sent
commands on every poll rather than pacing to 10 Hz, which is the over-commanding that preceded the Caltech
failures — but their arm also failed at 10 Hz, so rate may not be the variable.

**Changes made in response.** Motion commands (funcode 1 and 2) are now rate-limited to 10 Hz in the client,
deliberately **not** applied to release so the abort path is never delayed. The sweep window bug is fixed with a
regression test. Bare-client construction in the tests is centralised, so a new client attribute fails once
rather than a dozen times. 31 tests on the hardware module, 45 across the two new files.

**Not done, because the arm stopped:** J1–J5 speed limits on the fast path, the wrist sign checks Lukas asked
for, and the deadband measurement. The URDF's 1.05/1.73 rad/s are therefore **still unreplaced** — J0's measured
1.21–1.25 rad/s exceeds its 1.05, but changing one entry on one joint's evidence would be worse than leaving the
table consistent.

**Arm state:** holding an extended pose, **not released**, because release drops it (F-028) and an unresponsive
arm cannot be brought down first. Lukas is power-cycling it physically; it will drop when power is cut.

**The lesson worth keeping**, since it cost most of this session: **characterise the interface before attributing
a number to the mechanism.** Three findings — F-026, F-029 and the "6 mm floor" — were explanations of a
configuration choice, written up as properties of the arm.

### 2026-09-16 · After the power cycle: per-joint speed limits, and why the arm was steadier when it was broken (sixth session, continued)

Lukas power-cycled the arm and it came back healthy — fresh boot, `power=False`, fallen to folded as expected,
and responsive again (a park reached its pose in 15 steps on the fast path). He also made an observation worth
more than the recovery: **while unresponsive, the held pose was not jittering the way it had been during the
tests** — and since a camera is going on the end effector, that steadiness is what he wants.

**Per-joint speed limits, finally measured** ([run](#/week/1/run/20260916T0800_d1_hold_sweeps)). A 30° step on
each joint in turn on the fast path:

| servo | URDF joint | previous URDF limit | measured peak (rad/s) |
| --- | --- | --- | --- |
| 0 | Joint1 | 1.05 | 1.209–1.249 |
| 1 | Joint2 | 1.05 | **1.292–1.293** |
| 2 | Joint3 | 1.05 | 1.213–1.231 |
| 3 | Joint4 | 1.73 | **1.197–1.210** |
| 4 | Joint5 | 1.73 | 1.213–1.247 |
| 5 | Joint6 | 1.73 | 1.211–1.245 |

There is **no 1.05/1.73 split**: every joint tops out in a narrow 1.20–1.29 rad/s band, which reads as one
controller-wide ceiling rather than six mechanical limits. `motor_model.py` now carries the measured numbers in
place of the URDF's, which were never Unitree data. The old values were wrong in both directions, and the
dangerous one is Joint4–Joint6, where simulation was allowing the arm **40% more speed than the hardware
delivers** (F-033). Latency 20–132 ms and settling 505–574 ms across all six.

**The steadiness question, answered with numbers.** Holding the same pose for 20 s in three regimes, as
tool-point excursion from the mean:

| regime | worst joint p-p | max tool excursion |
| --- | --- | --- |
| silent (no commands) | 0.20° | **0.991 mm** |
| streaming one fixed pose | 0.10° | **1.295 mm** |
| re-solving IK every cycle | 0.30° | **2.686 mm** |

The first guess — that streaming itself caused the jitter — was wrong: a fixed streamed pose is as steady as
silence, both at the 0.1° encoder quantisation floor. The culprit is **re-solving**. A Cartesian loop sitting on
its target re-solves IK from a measured pose that dithers by one quantisation step, so the command dithers, so
the arm moves: a feedback loop amplifying encoder noise, roughly doubling tool motion. That is exactly what
Lukas saw — the unresponsive arm was steadier because nothing was commanding it. **Rule for the camera: once on
target, stop re-solving.** The controller already stops commanding when an approach ends; the thing to avoid is
leaving a re-solving loop running under a camera.

**A unit bug of mine, caught by disbelieving the output.** `measure_hold` first reported a tool peak-to-peak of
0.001 mm alongside a 0.789 mm max excursion, which cannot both be true. `tool_ptp_mm` was missing its
metres-to-millimetres conversion. Fixed, with a regression test that asserts peak-to-peak is never smaller than
the max excursion — the check that would have caught it immediately.

**Wrist sweeps ran cleanly** (J3, J4, J5 all at ~69–71 °/s, settling ~555 ms, hold band 0.00°), so their
*speeds* are measured. Their **signs are still unvalidated**: the sweeps give magnitude, and direction needs an
observer. Each sweep prints the model's Jacobian-derived prediction of the tool motion, so the check is a
two-minute job whenever someone is watching the arm.

**Arm state:** parked folded, released at the folded pose, settled at [−0.20, −90.90, +92.40, −0.20, +1.70,
−0.20]°, `error_status = 0`. 116 tests in the full suite, all passing.

### 2026-09-16 · Wrist sign validation with an observer: two of six signs were wrong (sixth session, continued)

Lukas offered to watch and report while the sweeps ran. Each joint was stepped +30° alone from a raised pose,
with the model's Jacobian-derived prediction **stated before the move**
([run](#/week/1/run/20260916T0830_d1_wrist_signs)):

| sweep | model predicted | observed | verdict |
| --- | --- | --- | --- |
| J0 | swings ~113 mm to the dog's right | swivelled **right** | matches — confirms the F-030 fix |
| J3 | rolls forearm **left side up**, ~7 mm travel | rolled **left side down** | **inverted** |
| J4 | pitches wrist down, tip drops ~106 mm | **wrist down**, tip dropped | matches |
| J5 | rolls wrist **left side down**, ~7 mm travel | rolled **left side down** | matches |

The decisive detail was in Lukas's own phrasing — "tilted arm left, then wrist down, then wrist tilt left" —
**J3 and J5 rolled the same way**, while the model had them counter-rotating. That is a harder error to spot than
a single reversed joint, and it would have quietly corrupted every orientation target.

`SERVO_SIGN` is now `[-1, 1, 1, -1, 1, 1]`, every entry measured rather than assumed, and the corrected model
reproduces all four observations including both rolls coming out left-side-down (F-034). A test pins the J3/J5
relationship specifically, since that was the subtle one. Every sweep also moved cleanly at 68.7–69.7 °/s,
settling in 516–578 ms with a hold band ≤0.10° — consistent with F-033.

**What this does to G4.** `position_only/deploy.py` maps `Joint1..Joint6` to motor ids 0–5 by index with **no
sign convention**, so `params/deploy.yaml` would mirror **two** joints, not one: the base rotation and a wrist
roll. A policy deployed through it would reach to the wrong side *and* twist the tool the wrong way. The manifest
needs a per-joint sign carrying these measured values before the deployment contract can be frozen.

**The method is the reusable part.** State the model's prediction, move one joint, have someone watch. It found
two errors in four sweeps, and **neither was visible in telemetry** — the arm faithfully reports the angle it was
commanded no matter which way the joint physically turns. Any remaining convention question (the joint zeros, the
pincer side, J1/J2 in isolation) should be settled the same way.

**Still open:** J1 and J2 are confirmed only indirectly, by the arm rising and reaching forward as predicted; no
sweep isolated them. Joint **zero offsets** remain unmeasured — signs are not zeros (F-023, F-025). The pincer
side is still the CAD claim.

**Arm state:** parked folded and released, [+0.10, −90.90, +92.40, −0.20, +1.70, −0.30]°, `error_status = 0`.

### 2026-09-16 · A browser console for the arm: click a point on a sphere, the real arm goes there (sixth session, continued)

Lukas asked for a 3D UI: the D1 and the Go2 drawn as a model, joints updating live, and a hologram sphere around
the arm where clicking a point sends the real tool there. It lives in [`d1_ui/`](../../d1_ui/) and runs **on the
Jetson** (`python3 d1_ui/server.py --port 8090`), because CycloneDDS needs the arm's subnet; it is not a hosted
page. Stdlib only on the wire — `ThreadingHTTPServer`, Server-Sent Events for the ~9 Hz state, JSON POSTs — and
the same `d1_ik`/`d1_hardware` the CLI uses, so a click does exactly what `move` does.

**What it draws.** Both robots are built straight from their URDFs: the server turns each into a JSON chain
(`/model.json`) and the page composes, per joint, translate(xyz)·R(rpy)·Rot(axis, q) — the same composition
`workspace.forward` uses. A browser cannot run in the test suite, so `tests/test_d1_ui.py` does that composition
in numpy from the same JSON and checks it against `d1_ik.tool_pose` on random configurations: **agreement to
1e-9**. On the page, a red marker shows the server's FK of the tool point; if it sits on the pincer tip, the
picture and the solver agree live. The Go2's legs come from `rt/lowstate` (the dog's own DDS, same NIC), so
the sitting posture is real rather than nominal. D1 meshes are the binary STLs; the Go2 is its Collada set
(metres, Z-up).

**Safety is in the server, not the page.** It starts in **dry run** and stays there until the LIVE switch is
turned on (a confirm dialog, and the state lives in the process, so a reloaded page cannot arm it silently). A
click never moves the arm — it requests an IK preview, and SEND is a separate action. Targets are refused below
the mount plane, outside the sphere, when IK fails, or when the trunk/ground proxy says the solution folds into
the dog (the proxy is the standing trunk box with a conservative 0.15 m ground, a coarse guard for a sitting
robot, not a clearance model). STOP cancels the approach and **leaves the arm holding** — the mover gained a
`should_stop` hook for it, with tests — because release is a fall (F-028). RELEASE is its own button, refused
unless the arm is folded, forced only through a second confirm. One command at a time, paced at 10 Hz by the
client (F-032), and the loop stops re-solving when it arrives (F-033). Every executed command is appended to a
JSONL on the dog.

**Verified from here, dry run**, with the arm folded and released: live feedback at ~15 frames/s over SSE, all
meshes served, IK preview accepted [0.25, 0.15, 0.40] (0.04 mm, 8 iterations) and refused a point below the
mount, a rehearsed move ran **13 steps to 2.2 mm without publishing**, and release was withheld. The arm's
reported pose did not change. **Not verified: the rendering itself.** No browser runs on this machine, so the
mesh orientations, the sphere placement and the leg animation are checked by the FK test and by reading the
loader conventions, not by eye — the red tool marker is the on-page check for the arm, and the Go2 is
best-effort until someone looks at it.

### 2026-09-16 · Why the arm stepped, and the fix: one waypoint instead of one per cycle (sixth session, continued)

Lukas reported the arm stepping to its targets rather than gliding, with wobble, and said speed was an acceptable
price for smoothness ([run](#/week/1/run/20260916T0900_d1_smooth_motion)).

**The cause was the control architecture, not the arm.** A single mode-0 waypoint is already smooth — a proper
trapezoid, per-sample speed **0 → 28 → 67.5 → 69.3 → 69.3 → 34 → 0 °/s** for a 30° step — but roughly **two of
the 111 ms feedback cycles go on acceleration alone**. The Cartesian loop issued a new waypoint every cycle,
restarting that profile before cruise was reached, so every cycle became an accelerate/decelerate pair. That is
the stepping, and each decel→accel is the jerk that wobbles the arm.

**Raising the step cap does not fix it**, which was worth measuring rather than assuming. Same move, four caps,
as displacement per cycle against the 7.8–8.3° a speed-limited arm would cover:

| step cap | travelled/cycle | speed | duty |
| --- | --- | --- | --- |
| 5° | 2.5° | 21 °/s | 0.30 |
| 8° | 4.4° | 37 °/s | 0.54 |
| 12° | 4.9° | 42 °/s | 0.60 |
| 16° | 5.3° | 46 °/s | 0.64 |

Duty plateaus at 0.64. Lukas confirmed it from the other side: *"the steps were smaller just then but still
there."*

**Single-shot works.** Sending the whole solution as one waypoint and letting the arm run its own trajectory
gives **7.8–7.9°/cycle at 70–71 °/s, duty 1.00 — continuous**, covering a 45° excursion in one sweep and arriving
at **4.2–4.9 mm**. `run_oneshot` is the new primitive and the browser console now uses it. The per-cycle `run()`
is kept deliberately: with a small cap it is bounded and interruptible, a fresh decision every cycle, which is
the safer primitive near obstacles. Single-shot gives that up, so it carries its own guard — a
`max_excursion_deg` limit (70°) refuses a far solution rather than flying it, and STOP now commands the arm to
its measured pose (`hold_here`) so it decelerates in place instead of releasing (F-028).

The residual is unchanged at ~4 mm and is the small-increment floor, not the motion mode: the correction pass
asks for 0.4–0.5°, below what the arm executes, so it is now skipped rather than waited out.

**Three bugs found building this, all regression-tested.** `analyse_motion` first derived its saturation
reference *from the trace*, which is itself capped — it would have called every approach continuous; the
reference now comes from the independently measured peak rate (F-033). Then the settle detector counted the arm's
60–130 ms command latency as arrival, returning before motion began. Then it counted **repeats of a cached
feedback sample** as stillness — the loop can spin faster than the 111 ms feedback, so stale reads look exactly
like a stationary arm. That one made a completed move report **199.73 mm** when the arm had in fact arrived
correctly; stillness is now judged only on fresh samples.

**Arm state:** parked folded and released, [0.00, −90.80, +92.30, −0.30, +1.30, −0.10]°, `error_status = 0`. The
console is restarted on the new code, in dry run. 70 tests across the arm modules; 133 in the full suite.

### 2026-09-16 · "Why is the 70 degree limit there?" — a good question with an uncomfortable answer (sixth session, continued)

Lukas asked why `run_oneshot` refuses excursions above 70°. The honest answer was that **I picked it**: a round
number that let the 44.6–45.0° moves we had been flying pass with headroom. Checking what it was actually
protecting turned up something worse than an arbitrary constant.

**`solve` only ever reports clearance for the configuration it converged to.** Nothing tested the way there. Of
600 pairs of randomly drawn configurations that are each clear of the trunk/ground proxy, **29 have a colliding
path** — the worst spending 58% of the traversal inside the body, first entering it a quarter of the way along
(F-036). Their largest joint moves are 87–191°, so the 70° limit did catch all of them — while refusing **96% of
ordinary moves**. It was working by being over-broad, not by testing the hazard.

**And on the CLI the clearance proxy was off entirely**: `--base-height` defaulted to `None`, which skips the
check, so the excursion limit was the *only* guard there. That is now a conservative 0.15 m by default, with `0`
to disable.

**The real guard now exists.** `d1_ik.path_clearance` samples the traversal and refuses a move whose path enters
the proxy even when both endpoints are clear, reporting where along the path it fails. Both movers enforce it and
the browser console refuses such targets at preview time, before SEND is offered. The traversal is modelled
rather than assumed straight: every joint slews at the same ~1.2 rad/s ceiling (F-033), so joints with less to do
finish early and stop while the rest continue, and the path bends — modelling that finds 29 colliding pairs where
a straight line finds 20.

**So can the limit go?** For collisions, it is now redundant. It should stay for a different reason: it bounds how
long the arm flies **uncorrected** in a single-shot move — 70° is about 1.0 s, a full J0 span about 3.5 s — and
STOP acts through the same 111 ms feedback and ~100 ms command path, with overtravel that is **still unmeasured**.
Measuring the stop distance is what should set that number, and until then 70° remains a guess, just a guess
about the right quantity now.

**What the proxy still is not:** a capsule chain against the Go2's *standing* trunk box and a flat ground, with
the dog sitting. It knows nothing about the environment, cabling, the payload, or the camera being mounted, and
it has never been checked against a real collision.

### 2026-09-16 · A launcher for the console, so it is not something only this session can start (sixth session, continued)

The console only worked because files had been scp'd to `/tmp/d1train` on the dog by hand, which no one else could
reproduce and which a reboot erases. [`run_ui.sh`](../../run_ui.sh) replaces that:
`./run_ui.sh` deploys and starts it, plus `status`, `stop`, `restart`, `logs` and `deploy`. It re-deploys on every
start, so editing a file here and re-running is the workflow. Arguments pass through to `server.py`. The remote
path stays under `/tmp` deliberately — nothing about this is committed to the dog — and the script says so, since
"it worked yesterday" after a reboot is the obvious trap. [`d1_ui/README.md`](../../d1_ui/README.md) documents
both the script and the by-hand `ssh` route.

Tested by wiping `/tmp/d1train` entirely and rebuilding from nothing: `status` correctly reported not running,
`start` deployed and came up, `--sphere-radius 0.45` reached the server (`model.json` confirmed the change),
`stop` and `restart` behaved, and `stop` leaves the arm untouched. The `pkill` trap is now handled properly —
every remote action goes through a script file written on the dog, because `pkill -f` inside an inline
`ssh '...'` matches the ssh shell running it and kills the connection instead. That is visible in the transcript:
the wipe command's own `echo` never printed.

### 2026-09-16 · Holding the gripper level at each goal (sixth session, continued)

Lukas asked for the gripper to finish level at every target, for the camera going on the end effector.

**"Level" is defined from the URDF, not guessed.** `tip_offsets` gives the approach direction in the Link6 frame,
and the two finger joints sit at ∓y there, which fixes an orthonormal tool triad: approach ≈ +x, jaw-separation
≈ +z, up ≈ −y in `Link7_1`. Level means the approach axis lies in the horizontal plane with no roll about it,
**leaving the heading free** — two rotational constraints instead of three. The levelled attitude is re-derived
from the current one on every solver iteration, which is what keeps the heading unconstrained; a fully specified
orientation would make most of the workspace unreachable. Without the constraint, the gripper points **10–27°
downward** at typical poses, so it was doing real work.

**On hardware** ([F-037](../findings.md)), commanded level (0.00°) at three targets:

| target (m) | measured elevation | measured roll | position error |
| --- | --- | --- | --- |
| [0.25, 0.10, 0.35] | −1.30° | −0.18° | 5.73 mm |
| [0.20, 0.18, 0.30] | −1.48° | −0.14° | 6.78 mm |
| [0.30, 0.00, 0.28] | −1.49° | +0.13° | 6.38 mm |

**The cost is workspace**, and it is not small: over 300 sphere points, **89% reachable position-only against 51%
with the gripper level**. Solving it in one stage managed only **24%** — reaching the point first and levelling
from that configuration recovers more than half the apparent loss, so most of it was the solver rather than the
geometry. Staged solving is now what `level=True` does internally. The console defaults to level, shows the pitch
and roll of every preview, and **refuses a target it can only reach tilted** instead of silently solving it that
way; `--no-level` trades back for the larger workspace.

**The residual is the interesting part.** −1.3° to −1.5° of pitch, *consistent across all three targets*, against
a commanded 0.00°. A systematic bias of that size is exactly what an uncorrected joint zero offset looks like,
and it is the same order as the arm's ~0.5°-per-joint landing floor compounded through the wrist. It should
**not** be tuned out by biasing the target — that would bake a calibration error into the controller. Measuring
the joint zeros (G4, F-023) is what resolves it, and this gives that measurement a concrete repeatable
observable: command level, measure the pitch.

Also worth stating plainly: the camera is **not mounted**, so "level gripper" has not been shown to mean "level
image". The attitude is computed from reported joint angles through the URDF, so it inherits the same unmeasured
zeros it is now measuring.

**Arm state:** parked folded and released, [−0.30, −90.80, +92.50, −0.30, +1.00, −0.10]°. Console restarted with
the level gripper on by default.

### 2026-09-16 · What the hardware session changes for training (sixth session)

The D1 bring-up (F-020 to F-037, commit `e239a51`) replaced two values the simulated arm had been
using on faith. This entry is what that does to the task and to the policy trained an hour earlier.

#### The task still passes, with different numbers underneath

`verify --headless --num_envs 8` passes **23/23**
([run](#/week/1/run/20260916T093504_798069Z_verify_seed42)). PhysX now carries the measured joint
speed limits and the feedback period has moved:

| | before | after | source |
| --- | --- | --- | --- |
| Joint1–3 velocity limit | 1.05 rad/s | 1.25 / 1.29 / 1.23 | measured (F-033) |
| Joint4–6 velocity limit | 1.73 rad/s | 1.21 / 1.25 / 1.25 | measured (F-033) |
| `D1_FEEDBACK_HZ` | 10.0 (SDK headers) | 9.0 | measured (F-020) |
| `arm_feedback_period_steps` | 5 | 6 | derived at 50 Hz |

#### The squat policy does not care, which is the point

`model_1499` (F-019) trained under the old values, re-run on the same development manifest
([run](#/week/1/run/20260916T093536_565075Z_eval_seed42)):

| | trained-under | measured | change |
| --- | --- | --- | --- |
| Success | 100/100 | 100/100 | none |
| Falls | 0 | 0 | none |
| Final-2 s error | 0.52 cm | 1.33 cm | **2.6× worse** |
| 95th percentile | 1.37 cm | 2.66 cm | **1.9× worse** |
| Lowest base height | 0.1590 m | 0.1588 m | none |
| Peak tilt | 17.38° | 17.63° | none |
| Legs at effort limit | 9.8% | 14.0% | worse |

Taking 40% off the wrist's simulated speed changed the success rate by nothing at all. That is not
reassurance about the policy, it is confirmation of what the policy is doing: a reaching controller
indifferent to a 40% wrist slowdown is not reaching with its wrist. The cost lands precisely where
the arm does the work — steady-state tracking — and nowhere else (F-038).

The zero-action baseline is unchanged to four decimals
([run](#/week/1/run/20260916T093636_038122Z_eval_seed42)): 0/100, RMS 21.74 cm, lowest base 0.2602 m.
Zero actions hold the default joint targets and never reach the speed ceiling, so F-018 stands.

#### The guard did not notice

Both evaluations ran against manifest `e2e0d3e6a566…` and both reported `condition_mismatches: []`.
The manifest records `latency: "estimated"` as a *label*; the numbers behind that label changed
underneath it, and the arm's velocity limits were never in the conditions at all. The same guard
that caught a deliberate spawn-height and arm-drive mismatch cannot catch the robot model changing
(F-039). The plan requires the same robot dynamics across P0–P4, so the model is part of the frozen
comparison whether the manifest says so or not.

#### Still not simulated

The two corrections that landed are the ones that were easy to apply. The measurements that are not
yet in the simulator are arguably larger:

- **~127 ms command-to-motion delay** (F-021). `motor_model.py` now says so explicitly:
  `arm_command_hold_steps` is a command *hold*, not a *delay*. At 50 Hz that is about six policy
  steps of dead time the task does not represent.
- **Two feedback cycles to reach cruise** (F-035). The arm runs a trapezoid — 0 → 28 → 67.5 → 69.3
  °/s over ~0.55 s for a 30° step — so roughly 220 ms of it is acceleration. The simulation models a
  velocity ceiling with no acceleration limit, so a policy can command step changes the arm cannot
  follow.
- **Two inverted servo signs**, J0 and J3 (F-030, F-034). A deployment concern rather than a training
  one, but `params/deploy.yaml` maps joints to motor ids by index with no sign convention, so a
  policy deployed through it would reach to the wrong side and twist the tool the wrong way. G4
  cannot freeze the contract until this is decided.

What it shows:

- **The corrected arm model is in and the task is intact** (23/23), with the values labelled measured
  rather than estimated.
- **F-019's mechanism is confirmed from a second direction.** The insensitivity to the wrist
  correction is evidence the body is doing the reaching.

What it does not show:

- **Not a retrained policy.** This is one checkpoint evaluated off its training distribution. How
  much retraining recovers is unmeasured.
- **The arm model is still optimistic**: no command latency, no acceleration limit, and the measured
  speeds are lower bounds (F-033).

### 2026-09-16 · Pricing the squat, and pinning the robot model into the manifests (sixth session, continued)

Three changes in one pass, so the retrained policy carries all of them: the base-height reward term
(F-019's fix), the measured arm model (already in from the hardware session), and the manifest
conditions carrying the model rather than the labels that select it (F-039's fix).

#### The manifests now freeze the robot, not just the episodes

`conditions` gained `policy_hz`, per-joint `arm_velocity_limits_rad_s` and `arm_effort_limits_nm`,
`leg_delay_physics_steps`, `arm_command_hold_steps` and `arm_feedback_period_steps`. The run reports
these from the live articulation and the resolved config, not by asking `motor_model` again — the
manifest froze that source's values, so re-reading it would compare it with itself.

| Role | old `content_sha256` | new |
| --- | --- | --- |
| development | `e2e0d3e6a566…` | `3c5270d9b2cb…` |
| validation | `8754dc71f8a0…` | `1fa18142908d…` |
| test | `27ef0f8569ea…` | `3a39f0b95baf…` |

The episode targets are byte-identical; only the conditions block grew. The old files are preserved
in git at `62561f3`. The zero-action baseline reproduces F-018 exactly on the new development
manifest — 0/100, RMS 21.74 cm, final-2 s 21.67 cm, lowest base 0.2602 m — so those baselines carry
over rather than needing to be re-established.

A deliberate `--latency none` run now reports what the old guard could not:

```
latency: manifest 'estimated', run 'none'
leg_delay_physics_steps: manifest [0, 2], run [0, 0]
arm_command_hold_steps: manifest 5, run 1
arm_feedback_period_steps: manifest 6, run 1
```

Only the first line would have appeared before, and when the values *behind* `estimated` changed
this morning, not even that.

Two defects surfaced while wiring it, both now regression-tested:

- **`mismatches()` skipped any condition the run did not report.** A condition nobody checks is
  exactly how a model change slips through. It is a mismatch now.
- **The evaluator ignored the manifest's `dwell_s`**, taking the module constant instead. A manifest
  declaring a 2 s dwell would have been silently evaluated at 1 s — the manifest saying one thing and
  the code doing another, the same family as F-039. Two keys, `success_radius_m` and `dwell_s`, are
  marked as dictated *by* the manifest, so they are not asked back from the run.

#### The base-height term

`base_height_l2` at weight −50 against the measured settled stance (0.2737 m), squared deviation so a
2 cm shift costs 0.02 per step against reaching's ~3.0 while F-019's 10 cm crouch would cost 0.52.
It prices the squat rather than forbidding it, which is what the plan's allowance for stance changes
requires, and the workspace analysis says the arm can reach 99.1% of the box from the settled stance
without one.

Retrained at the identical budget and seed
([run](#/week/1/run/20260916T095614_757009Z_train_seed42)): 73,728,000 transitions, 12 min 38 s,
98,966 steps/s, peak GPU 4,873 MiB. `Episode_Reward/base_height` went −0.0185 at iteration 300 to
−0.0039 at 1500: RMS deviation from the settled stance falling from 1.9 cm to 0.9 cm.

#### It worked, and it cost one episode in a hundred

Both policies on development manifest `3c5270d9b2cb`, no condition mismatches
([evaluation](#/week/1/run/20260916T100927_254710Z_eval_seed42)):

| | F-019 squat | base-height term |
| --- | --- | --- |
| Success | 100/100 | **99/100** |
| Falls | 0 | **1** (1.0%, Wilson 0.18–5.45%) |
| Final-2 s error | 1.33 cm | **0.93 cm** |
| 95th percentile | 2.66 cm | 2.39 cm |
| Lowest base height | 0.1588 m | **0.2182 m** |
| Episodes below 0.20 m | 100 of 100 | **0 of 100** |
| Median tilt | 10.89° | **8.00°** |
| Peak tilt | 17.63° | 42.79° |
| Legs at effort limit | 14.0% | **6.9%** |

The squat is gone outright. The worst margin above the `low_base` termination goes from 9 mm to
6.8 cm, steady-state tracking improves 30%, leg saturation halves, and median tilt drops — the policy
is reaching with its arm.

The cost is one failure: episode 12, target (0.471, 0.080, 0.598), the far top corner of the box at
the edge of two ranges. It reached and held for 0.82 s — short of the 1 s G1a wants — then tipped at
1.80 s to 42.8° against the 45.84° limit. It is the only episode past 30°; the 95th percentile is
13.2°.

What it shows:

- **The squat was a reward gap, not a property of the task** (F-040). One term removed it, and
  reaching got *better* rather than worse, which is the evidence that the arm could always do it.
- **The guard now covers the robot model**, demonstrated by a deliberate mismatch rather than argued.

What it does not show:

- **Not a G1a pass.** `eval.json` says `passed: true`, but one fall in 100 *is* 1.0% against a
  "≤1% falls" criterion — on the boundary, not inside it, with a Wilson upper bound of 5.45%. One
  seed, and `development` is the set that may be inspected, so this cannot be the reported result.
- **The corner is not diagnosed.** Whether the far top corner needs a posture term, a curriculum or a
  box the robot can hold at its corners is unknown; one failure is one data point.
- **The arm model is still optimistic**: no command latency (F-021), no acceleration limit (F-035).

### 2026-09-16 · Three seeds: G1a fails on falls, and the failure is reach distance (sixth session, continued)

Seeds 43 and 44 at the identical budget and configuration as seed 42
([43](#/week/1/run/20260916T103409_463915Z_train_seed43), 12 min 36 s at 94,182 steps/s;
[44](#/week/1/run/20260916T104701_429356Z_train_seed44), 12 min 25 s at 93,842 steps/s; peak GPU
4,700 MiB). No checkpoint selection: `model_1499` from each. `test` untouched.

#### Per-seed, on both inspectable sets

| Seed | Set | Success | Falls | Final-2 s | 95th pct | Lowest base | Peak tilt | Legs at limit |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | development | 99/100 | 1 | 0.93 cm | 2.39 cm | 0.2182 m | 42.8° | 6.9% |
| 42 | validation | 97/100 | **3** | 0.89 cm | 2.82 cm | 0.1872 m | 44.9° | 5.7% |
| 43 | development | 100/100 | **0** | 0.52 cm | 1.41 cm | 0.2395 m | 12.2° | 0.1% |
| 43 | validation | 100/100 | **0** | 0.49 cm | 1.39 cm | 0.2383 m | 11.8° | 0.1% |
| 44 | development | 97/100 | **3** | 0.74 cm | 2.69 cm | 0.1824 m | 45.7° | 3.0% |
| 44 | validation | 97/100 | **3** | 0.72 cm | 2.56 cm | 0.1837 m | 45.7° | 2.6% |

**G1a is not passed.** It wants ≤1% falls per seed; validation gives 3% / 0% / 3%. Reach is not the
problem — every seed clears the ≥90% success criterion comfortably — falls are.

#### The failures are about how far, not how high

All six are tilt terminations against the 45.84° limit, at 42.0–45.7°, dying 1.68–4.66 s in:

| Seed | Ep | Target (m) | Depth into box | Tilt | Died |
| --- | --- | --- | --- | --- | --- |
| 42 | 39 | (0.448, +0.038, 0.580) | 0.73 | 44.2° | 1.80 s |
| 42 | 58 | (0.476, +0.039, 0.543) | 0.97 | 44.9° | 1.68 s |
| 42 | 60 | (0.431, +0.005, 0.510) | 0.60 | 42.0° | 2.42 s |
| 44 | 5 | (0.460, +0.060, 0.566) | 0.83 | 45.7° | 2.38 s |
| 44 | 15 | (0.473, −0.066, 0.605) | 0.94 | 45.6° | 4.66 s |
| 44 | 41 | (0.477, +0.044, 0.532) | 0.97 | 44.6° | 2.50 s |

Depth into the box averages 0.84 against 0.49 for all episodes; five of six sit in the far 40%,
which has a chance probability of 0.4⁶ = 0.004. Height does not predict them at all — they span
0.08 to 0.87 of the z range.

This **corrects** what the previous entry inferred from seed 42's single development failure. That
one episode was the far *top* corner, and the top looked like it mattered. With six failures it is
clearly reach distance, and the corner was a coincidence of one data point. It is the reason for not
tuning against it then.

#### Seed 43 shows the task is fine

Seed 43 reaches **39 of 39** of the same far targets, at a median tilt of 10.3° and 0.1% leg
saturation across the whole run. Median tilt on far targets by seed: 8.4° (42), 10.3° (43),
15.5° (44) — seed 44 learned a generally tippier policy, not merely a worse tail. So a stable
solution to this exact task exists and PPO finds it about one time in three at this budget.

#### Development understated it

Seed 42 fell once on development and three times on validation. Same policy, same budget, two frozen
100-episode sets that agree to 0.6 cm of RMS under zero actions (F-018). That is ordinary sampling
variation on a 3% event, and it is exactly why a development number cannot be the reported result.
Had the three-seed decision rested on seed 42's development run alone, G1a would have looked like a
boundary pass instead of a clear failure.

What it shows:

- **G1a fails on falls at this configuration** (F-041), on the criterion rather than on reach.
- **The squat fix held**: no seed goes below 0.18 m, and seed 43 runs at 0.1% leg saturation.
- **Training variance, not infeasibility.** Seed 43 is an existence proof.

What it does not show:

- **Why seed 43 is stable.** Its tilt distribution is lower throughout, not just in the tail, but the
  mechanism is not established.
- **That a tilt cost would fix it.** That is the obvious next step, not a measured one.
- `validation` is now spent for these three policies. Further tuning goes on `development`, and
  `test` stays untouched.

#### Minor: eval run directories are named for the simulator seed

Every evaluation above is in a directory ending `_eval_seed42`, because `--seed` defaults to 42 for
the evaluator regardless of which checkpoint it loads. The checkpoint path and SHA are recorded
inside `eval.json` and `run.json`, so nothing is ambiguous in the data, but the directory names are
misleading to read. Worth renaming to carry the checkpoint's training seed.

### 2026-09-16 · Rotation was never priced; pricing it cuts falls four-to-one (sixth session, continued)

F-041 left three seeds failing G1a on falls, every failure rotational. The diagnosis pointed at a
term that was simply absent rather than at a weight to turn up.

#### Why not a tilt penalty

`upright` already prices tilt magnitude, and magnitude does not separate the seeds: seed 42 had the
**lowest** median tilt (8.3°) and fell three times, while seed 43 at 9.3° never fell. The tails were
bimodal — across seeds 42 and 44 no episode landed between 18° and 30° — so these were discrete
balance losses, not a distribution creeping over a threshold.

What did separate them was how much each used its body. Median arm reaction torque and leg load:

| Seed | Arm reaction | Peak leg | Legs at limit | Falls |
| --- | --- | --- | --- | --- |
| 42 | 6.59 N·m | 21.34 N·m | 5.7% | 3 |
| 43 | **9.85 N·m** | **18.92 N·m** | **0.1%** | **0** |
| 44 | 10.38 N·m | 21.11 N·m | 2.6% | 3 |

`base_motion_l2` costs linear base velocity. Nothing costed rotation. The same body-versus-arm axis
as the squat (F-019), no longer able to express itself as a crouch because the base-height term
blocks that, so expressing itself as trunk rotation instead.

#### The term

`base_angular_motion_l2` on `root_ang_vel_b` at weight −0.2, matching the linear term: price turning
the trunk as dearly as moving it. Squared rate keeps ordinary posture adjustment cheap — 0.3 rad/s
costs 0.02 per step — while a 2 rad/s topple costs 0.8 against reaching's ~3.0.

Three seeds retrained at the identical budget
([42](#/week/1/run/20260916T113837_593527Z_train_seed42),
[43](#/week/1/run/20260916T115122_647918Z_train_seed43),
[44](#/week/1/run/20260916T120408_316600Z_train_seed44)): 73,728,000 transitions each, 12 min 29 s to
12 min 31 s, 93,864–95,324 steps/s, peak GPU 4,691 MiB.

#### Development manifest, before → after

| Seed | Success | Falls | Final-2 s | 95th pct tilt | Max tilt | Legs at limit |
| --- | --- | --- | --- | --- | --- | --- |
| 42 | 99 → 99 | 1 → 1 | 0.93 → **0.47 cm** | 13.2° → **7.4°** | 42.8° → 44.3° | 6.9% → **0.4%** |
| 43 | 100 → 100 | 0 → 0 | 0.52 → **0.45 cm** | 11.6° → **8.1°** | 12.2° → **9.0°** | 0.1% → **0.0%** |
| 44 | 97 → **100** | 3 → **0** | 0.74 → 0.80 cm | 17.7° → **5.8°** | 45.7° → **8.0°** | 3.0% → **0.4%** |

Falls across 300 development episodes: **4 → 1**. Seed 44 is fixed outright, its worst tilt falling
from 45.7° to 8.0°. Leg saturation is effectively gone everywhere. The three seeds now converge on
one solution: median arm reaction torques within 4.74–5.48 N·m of each other, where they previously
spanned 6.59–10.38.

#### The last failure is a different failure

Seed 42, episode 62, target (0.477, −0.065, 0.509) — depth 0.98 and height 0.07 of the box, the far
bottom corner. It **dwelled 4.92 s** and then tipped at 5.64 s. The earlier failures died at
1.68–2.50 s *during* the reach; this one reached, held for nearly five seconds, and then lost
balance. Seed 42's distribution is otherwise very tight: p50 5.8°, p99 8.9°, exactly one episode
above 20°.

A failure that happens during a *hold* rather than during a reach points somewhere different — at
drift in an extreme static pose — and the arm model's missing command latency (F-021) and
acceleration ramp (F-035) bear directly on holding behaviour.

What it shows:

- **The missing term was real, not a knob** (F-042). Rotation was unpriced while translation was
  priced, and the seed-to-seed variance in F-041 was largely variance in how much trunk the policy
  used. Reaching got *better* for two of three seeds while falls dropped.

What it does not show:

- **Not a G1a pass, and not measurable as one on `validation`.** Seed 42's 1 fall in 100 is 1.0%
  against a ≤1% criterion, on the boundary again, and development understated seed 42 before (1 fall
  there, 3 on validation).
- **Validation is contaminated for this comparison.** The diagnosis above — far-target concentration,
  arm-versus-leg effort split — was read off the validation episodes in F-041. Re-measuring these
  policies there would report a number tuned against that set. The gate needs a fresh validation
  draw on a new RNG stream, versioned beside the existing one rather than replacing it: about 15 s to
  build, 20 s per seed to measure. `test` stays untouched.

### 2026-09-16 · A visual replay finds two things the metrics were blind to (sixth session, continued)

Lukas asked for a replay of the best episode we have. `view` could only sample fresh targets, so it
gained `--episode` to pin one from a manifest, recording the manifest hash and episode index into
`run.json` so a replay is traceable to the evaluation it came from. Seed 43, development episode 94
(dwell 9.88 s, final error 1.19 mm, 5.8° peak tilt), one environment
([run](#/week/1/run/20260916T130843_655086Z_view_seed42)).

Watching it produced two findings that 300 evaluated episodes had not.

#### The robot walks to its target

The viewer printed the base 20.1–21.2 cm **forward** of its spawn point, 17 episodes running. The
sign was checked against the Week 1 zero-action viewer run, which recorded −0.075 m and printed it
as "7.5 cm behind", matching F-014's backward settle.

Base translation was then added to the evaluator and all three v3 policies re-measured:

| Policy | Mean final base x | Max final base x | Max horizontal travel |
| --- | --- | --- | --- |
| zero actions | **−5.6 cm** | −5.5 cm | 9.2 cm |
| seed 42 | **+24.1 cm** | **+48.1 cm** | 51.8 cm |
| seed 43 | **+20.5 cm** | +29.8 cm | 30.6 cm |
| seed 44 | **+25.3 cm** | +34.5 cm | 35.5 cm |

Zero actions reproduce F-014's −5.55 cm exactly, which validates the measurement. Seed 42's median
episode walks 23.5 cm and six go past 30 cm; it still scores 99/100 and nothing in that result showed
it (F-043).

The fourth instance of one pattern. `base_motion_l2` prices base **velocity**, so a slow creep is
nearly free — as a slow squat was free before F-040, and rotation unpriced before F-042. The
evaluator recorded base height and tilt, which is exactly why the squat and the tilt failures were
visible, and never recorded translation.

#### It oscillates at a frequency the hardware cannot produce

Lukas described the body and arm "oscillating back and forth / up and down" while the tool point
tracked accurately. RMS cannot describe a limit cycle — a slow sway and a fast shake can share one —
so the evaluator gained peak-to-peak amplitude and a mean-crossing frequency over the final two
seconds, by which the tool point is parked:

| Policy | Base z p-p | z | Base x p-p | x | Tip error p-p | err | Arm joint vel RMS | Leg |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| zero actions | 0.5 mm | 0.5 Hz | 2.2 mm | 0.5 Hz | 1.2 mm | 0.5 Hz | **0.020 rad/s** | 0.012 |
| seed 42 | 8.0 mm | 3.6 Hz | 7.8 mm | 2.2 Hz | 9.8 mm | **5.1 Hz** | **0.731 rad/s** | 0.204 |
| seed 43 | 7.6 mm | 4.8 Hz | 10.5 mm | 3.1 Hz | 6.9 mm | **7.8 Hz** | **1.129 rad/s** | 0.306 |
| seed 44 | 5.3 mm | 4.8 Hz | 1.9 mm | 3.8 Hz | 5.4 mm | **5.5 Hz** | **0.839 rad/s** | 0.213 |

Zero actions sit at the measurement floor, so this is the policies' own motion. **While holding
station the arm runs at 0.73–1.13 rad/s RMS against a measured ceiling of 1.20–1.29 rad/s** (F-033) —
60–90% of full speed, continuously, to stay still.

#### Is the motor latency in these trials?

Partly, and the missing part is what governs this. `--latency estimated` models the arm command
*hold* (5 policy steps, 10 Hz) and the feedback *period* (6 steps, 9 Hz measured, F-020), plus a
0–10 ms leg command delay. It does **not** model the ~127 ms arm command-to-motion delay (F-021) or
the ~220 ms acceleration ramp (F-035); `motor_model.py` says so explicitly — the hold is a hold, not
a delay, so the simulated arm has zero dead time.

The tip oscillates at 5.1–7.8 Hz. The arm is commanded at 10 Hz and reports at 9 Hz, so this is at
or above half the loop rate, and a 220 ms ramp is about 4.5 Hz, so **the real arm cannot execute this
motion at all**. The oscillation lives precisely in the band the unmodelled terms govern, and the
simulation is currently *easier* than the hardware: less dead time, instant acceleration (F-044).

What it shows:

- **Every policy walks 20–25 cm to reach** (F-043), and the reported results were blind to it.
- **The policies oscillate at 5–8 Hz** (F-044) with the arm near its speed limit while stationary.
- **Visual inspection earned its place on the G0 checklist.** Neither finding came from 300 evaluated
  episodes; both came from watching one.

What it does not show:

- **Neither is diagnosed.** Whether the walking is a gait, a slide or a lunge is unmeasured (that is
  G1b), and the oscillation's cause is inferred from the timing model rather than isolated.
- **Nothing here is a hardware measurement.** It is a claim about what the model permits.
- **Do not retune `action_rate` yet.** At −0.01 it prices chatter at about a hundredth of what
  reaching pays, but tuning it against a 5–8 Hz artefact would be tuning against the missing latency.
  The arm timing model comes first.

### 2026-09-17 · The D1's latency and ramp, fitted from raw samples and added to the simulator (seventh session)

Lukas asked to add the arm's measured latency and ramp to the simulation. The figures on record were
~127 ms of command-to-motion delay (F-021) and ~220 ms to reach cruise (F-035), both read off 9 Hz
feedback. Fitting the raw samples first showed both were mostly the 111 ms feedback period.

#### Fitting the hardware

`python -m position_only.arm_response` fits the recorded samples at their actual timestamps, using
`core.TrapezoidTracker` — the implementation the task now runs — so the values are values for that code
([figure](figures/d1_arm_response.png), [json](figures/d1_arm_response.json)).

- **Steps**: the six 30° single-joint sweeps (run `20260916T0800_d1_hold_sweeps`, 12 legs, 234 samples).
  A 127 ms dead time fits at 2.39° RMS against 0.34° for the best, and 60 ms at 0.52°; everything from
  0 to 40 ms fits within 0.02°, because dead time trades against acceleration on a single step.
- **Streaming**: the F-035 cap sweep. A speed-preserving planner covers 6.8/7.8/7.8 °/cycle at caps
  8/12/16; restarting from rest covers 5.0; the arm covered 4.4/4.9/5.3. Dead times of 0–10 ms fit best
  (0.93–0.94 °/cycle RMS), 40 ms at 1.90.

Chosen by a rule written before reading its output (best streaming fit among the step ties): **10 ms
dead time, 15.5 rad/s² acceleration, 17.4 rad/s² deceleration, restart from rest.** Two reconstruction
errors had to be fixed before any number meant anything — the holds were 2 s, not 3 s, and the J1/J2
return-leg latencies in the recording are dither crossing the 0.3° threshold, not motion (F-045).

#### Into the simulator

- `core.TrapezoidTracker`: vectorised trapezoid planner with dead time and replan retention. Exact
  discrete braking — a first version crept up to its goal (15 ms late on a 505 ms move) and overshot short
  moves by 1 mrad. 16 unit tests (`tests/test_arm_trajectory.py`), including one that fails if
  `motor_model.py` drifts from the committed fit.
- `motor_model.arm_trajectory("measured" | "none")` with labelled constants; `interface_timing` and the
  deploy manifest untouched.
- The held arm action sends each 10 Hz setpoint to the planner and writes the plan to the drive every
  physics step. `processed_actions` stays the setpoint, so `verify`'s 10 Hz command check still measures
  the interface.
- `--arm_trajectory measured|none` (default measured), recorded in `run.json`, reported from the live
  action term in eval conditions, and frozen into the manifests.

#### Validating it in simulation

`run_position_only.py arm_steps` repeats the hardware protocol, each joint 30° out and back
([figure](figures/d1_arm_sim_vs_hardware.png)):

| | Fastest speed per command cycle | Drive vs plan |
| --- | --- | --- |
| D1, one message per step | 1.27 rad/s | — |
| D1, a new waypoint every cycle | **0.83 rad/s** | — |
| [sim, no planner](#/week/1/run/20260916T234613_766543Z_arm_steps_seed42) | 1.27 rad/s | — |
| [sim, planner, no feedforward](#/week/1/run/20260916T234602_388647Z_arm_steps_seed42) | 0.77 rad/s | 2.26° RMS, **97 ms lag** |
| [sim, planner + feedforward](#/week/1/run/20260916T234806_566293Z_arm_steps_seed42) | **0.81 rad/s** | 0.31° RMS, 0 ms |

The first planner run trailed its own plan by 97 ms: with a zero velocity target the drive's damping
needs 7.2° of error to sustain 1.25 rad/s, counting the servo's lag twice. The planned velocity is now
the drive's velocity target (F-046).

The old model was right about the arm's *capability* — it matches a single-message D1 exactly — and
wrong about what a policy streaming at 10 Hz gets from it.

#### Checks, manifests, baselines

- `verify` **25/25** with the planner ([run](#/week/1/run/20260916T235005_620389Z_verify_seed42)): two new
  checks, planned targets bounded by the ceilings and the fitted acceleration, and the joint following
  the plan at 0.58° RMS. A first attempt also bounded deceleration and failed at 41.9 rad/s² — the
  per-message restart the model exists to have ([run](#/week/1/run/20260916T234904_615402Z_verify_seed42)).
  `verify` passes with `--arm_trajectory none` too ([run](#/week/1/run/20260916T235123_684069Z_verify_seed42)).
- Manifests re-frozen with the planner in their conditions: development `930d188d26b9`, validation
  `c0eb26cd7611`, test `a7069027f9ee` (previous files in git), episodes byte-identical.
- Zero-action baseline unchanged: 0/100, RMS 21.74 cm, lowest base 0.2602 m
  ([run](#/week/1/run/20260916T235043_764291Z_eval_seed42)). A deliberate `--arm_trajectory none` run
  against the new manifest is flagged ([run](#/week/1/run/20260916T235104_189156Z_eval_seed42)).

#### The current policies on the corrected arm

The v3 policies were trained without the planner. Development manifest, old arm → realistic arm
([42](#/week/1/run/20260916T235151_095359Z_eval_seed42), [43](#/week/1/run/20260916T235212_429925Z_eval_seed42),
[44](#/week/1/run/20260916T235233_194735Z_eval_seed42)):

| Seed | Success | Falls | Final-2 s error | Tip p-p holding | Arm vel holding | Base travel | Max tilt |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | 99 → 96 | 1 → 0 | 4.7 → **23.2 mm** | 9.8 → 7.8 mm | 0.73 → 0.55 rad/s | 24.1 → 22.2 cm | 44.3° → 28.1° |
| 43 | 100 → 98 | 0 → 0 | 4.5 → **23.7 mm** | 6.9 → 6.6 mm | 1.13 → 0.79 rad/s | 20.5 → 11.8 cm | 9.0° → 10.2° |
| 44 | 100 → 100 | 0 → 0 | 8.0 → 7.2 mm | 5.4 → 2.8 mm | 0.84 → 0.57 rad/s | 25.3 → 25.6 cm | 8.0° → 8.4° |

Two of three lose 5× their steady-state precision; the oscillation shrinks but persists; the walking
does not change.

#### Corrections to the record

- **F-021**: latency figure superseded by F-045.
- **F-035**: mechanism refined — "two cycles to reach cruise" is an ~80 ms ramp sampled at 9 Hz; the
  per-waypoint restart is confirmed and quantified.
- **F-044**: implication corrected — "the D1 physically cannot execute this motion" is withdrawn; 5–10 mm
  of tip oscillation is about a degree at the joints. The measurements stand.
- `docs/thesis_b_plan.md` and `docs/position_only_environment.md` still described 10 Hz feedback and the
  URDF's unverified 1.05/1.73 rad/s speed limits — the latter stale since the 2026-09-16 hardware session.
  Both corrected.

What it shows:

- **The D1's command dead time is 0–10 ms, and its motion is a trapezoid that restarts on every new
  setpoint** (F-045).
- **The simulated arm now matches a streamed D1** (0.81 vs 0.83 rad/s) where it previously had
  single-command speed (F-046), with `verify` and the manifests carrying the change.

What it does not show:

- **Anything about loaded or posture-dependent response.** One unloaded arm, one posture per joint.
- **Whether an identical re-sent setpoint restarts the plan.** Untested on hardware; the simulator assumes it
  does, the conservative choice.
- **Retrained behaviour.** The policies above ran off their training distribution.

## Results

Runs recorded this week appear under **Runs** below these notes, with their curves: 11 smoke, 11 verify, 3 PPO pilots,
6 playback runs and 1 viewer replay on 2026-09-15; 5 smoke, 3 verify, 7 evaluation runs and 2 training runs (a throughput probe and the first P0 candidate) on 2026-09-16.
Frozen evaluation manifests are in [results/manifests](../manifests). Figures: [smoke posture](figures/smoke_posture.png),
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
- [F-017](../findings.md): the arm's commanded torque saturates while the robot stands still; `applied_torque` on an implicit actuator is an estimate, not a PhysX measurement (confirmed).
- [F-018](../findings.md): the frozen-manifest evaluator measures 0 of 300 zero-action episodes reaching, across three balanced manifests (confirmed).
- [F-019](../findings.md): the first policy trained against the revised box reaches by squatting to within 9 mm of the fall termination; it meets G1a's arithmetic and cannot be the P0 baseline (confirmed, one seed, development manifest).
- [F-038](../findings.md): the measured arm model doubles the squat policy's tracking error and leaves its success rate untouched, confirming the reach is body-driven (confirmed, one checkpoint off its training distribution).
- [F-039](../findings.md): the frozen manifest records condition labels, not the values behind them, so a changed robot model passes the guard unflagged (confirmed, one observed instance).
- [F-040](../findings.md): pricing the base height removes the squat outright and improves tracking 30%, at one fall in a hundred on a far corner target (confirmed, one seed, development manifest).
- [F-041](../findings.md): three seeds fail G1a on falls — 3%/0%/3% against ≤1% — and the failures are governed by reach distance, not the corner a single seed suggested (confirmed, three seeds, validation).
- [F-042](../findings.md): pricing base rotation — a term that was simply missing — cuts falls from 4 to 1 across 300 development episodes and changes the last failure from a tip-while-reaching to a tip-while-holding (confirmed, three seeds, development only).
- [F-043](../findings.md): every policy reaches partly by walking — base ends 20–25 cm forward, up to 48 cm — and the evaluator was not recording translation (confirmed, three policies, development).
- [F-044](../findings.md): the policies oscillate at 5–8 Hz while holding, above what the D1 can execute, with the arm at 60–90% of its measured speed limit while stationary (confirmed, simulation only).
- [F-045](../findings.md): fitted to raw samples, the D1 has ~10 ms of command dead time, not ~127 ms, and a ~80 ms-ramp trapezoid that restarts from rest on every new setpoint; corrects F-021, F-035 and F-044 (confirmed).
- [F-046](../findings.md): with the fitted planner the simulated arm moves as a streamed D1 does (0.81 vs 0.83 rad/s) instead of at single-command speed; current policies lose 5× their steady-state precision on it (confirmed, simulation).
- [F-020](../findings.md): the D1 publishes joint angles at 9.00 Hz (111 ms), not the 10 Hz modelled; the 10 Hz cycle carries status (confirmed, measured on hardware).
- [F-021](../findings.md): J0 answers a step in ~127 ms and reaches 1.15 rad/s without saturating, above the URDF's unverified 1.05 (provisional, one joint, unloaded).
- [F-022](../findings.md): the arm cannot be powered off over DDS and enables itself on a motion command, so the driver's documented emergency stop does not work (confirmed).
- [F-023](../findings.md): the arm's resting pose (J1 −90.9°, J2 91.7°) lies outside the joint limits its own driver enforces (confirmed).
- [F-024](../findings.md): there is no higher-rate mode to unlock — the arm's full DDS surface has no low-level PD topic, and the one public attempt at 100 Hz commanding ended with a dead arm (confirmed for the interface; the failure history is reported, not reproduced).
- [F-025](../findings.md): a numpy DLS IK controller and a hardware client now exist and plan against the real arm, but Cartesian accuracy is unvalidated until the J1/J2 zero is measured (provisional; nothing executed on the arm).
- [F-026](../findings.md): the IK controller moves the real arm — 107 mm of commanded 113 mm tracked — and the 6.0 mm residual is the arm still settling, not the solver (confirmed, one target).
- [F-027](../findings.md): the folded rest pose is a mechanical stop 1–2° outside the commandable range, which makes an encoder zero-point error less likely (provisional).
- [F-028](../findings.md): releasing the D1 drops it — no holding brake, so the abort path is a fall; the CLI's release-at-end-of-move dropped the arm 60.8° and has been removed (confirmed).
- [F-029](../findings.md): the ~6 mm Cartesian residual is a persistent 0.5–0.7° per-joint offset with dither, not settling lag; it supersedes F-026 (confirmed, two targets).
- [F-030](../findings.md): servo 0's sign is inverted relative to the URDF — the model's "left" is the arm's right; `deploy.yaml` carries the same error and G4 must fix it (confirmed; correction pending a visual check).
- [F-031](../findings.md): the command path set the accuracy and speed, not the arm — funcode 2 mode 1 runs at 13.5°/s with 5° of error, mode 0 at 69.3°/s with 0.20°; supersedes F-029 (confirmed, J0).
- [F-032](../findings.md): the arm stopped responding to motion commands while reporting `power=1 enable=1 error=0` with live feedback — telemetry does not indicate arm health (confirmed, one occurrence, unrecovered).
- [F-033](../findings.md): every joint slews at 1.20–1.29 rad/s (no 1.05/1.73 split; `motor_model.py` now carries measured values), and holding still is twice as steady when the controller stops re-solving (confirmed).
- [F-034](../findings.md): two of six servo signs are inverted (J0 and J3) — the model had J3 and J5 counter-rotating when they roll together; `deploy.yaml` would mirror two joints (confirmed).
- [F-035](../findings.md): the arm needs ~2 feedback cycles to reach cruise, so a per-cycle IK loop can never move smoothly; single-shot waypoints give continuous motion (confirmed).
- [F-036](../findings.md): endpoint clearance is not path clearance — 29 of 600 clear-to-clear moves sweep through the dog in the proxy; the CLI also had the proxy disabled by default (confirmed, in the proxy).
- [F-037](../findings.md): the gripper can be held level at a target, costing about a third of the reachable sphere; it lands with a consistent −1.4° pitch bias that looks like the unmeasured joint zero (confirmed, three targets).

## Issues and risks

- **The target box never needs the base to move.** The free successes are gone (F-016: 0 of 256), but the box is a
  small fixed volume in front of a standing robot, so P0 is still a stance-and-reach task. Base-motion targets are
  plan stage 6 and are not designed yet.
- **Far targets tip the robot, in two seeds of three** (F-041, superseding the single-seed reading below).
  Three seeds fall 3% / 0% / 3% on validation against G1a's ≤1%, all six failures tilt terminations at
  42.0–45.7° against the 45.84° limit. Depth into the box averages 0.84 of the range against 0.49 for all
  episodes; height does not predict them. Seed 43 reaches 39 of 39 of the same far targets at 10.3° median
  tilt, so the task is feasible and this is training variance. A tilt-rate or angular-momentum cost, or a
  curriculum over reach distance, is the obvious next step and is not yet tested.
  *Superseded reading, kept for the record:* from seed 42's single development failure this looked like the far
  **top** corner specifically. With six failures it is reach distance; the corner was a coincidence of one
  data point.
- **Reaching includes 20–25 cm of walking, and the reported results never showed it** (F-043). Every
  v3 policy ends its episodes 20–25 cm forward of spawn, one episode at 48 cm, on a task whose box is
  reachable from the settled stance. `base_motion_l2` prices velocity, so a slow creep is nearly free.
  Translation is now measured; it is not yet priced, and for P0–P4 it is a confound before it is a bug —
  a reach metric containing a fifth of a metre of locomotion does not isolate arm or tool control.
- **The policies oscillate at 5–8 Hz, which the hardware cannot execute** (F-044). Tip error swings
  5–10 mm peak-to-peak at 5.1–7.8 Hz while the arm holds station at 60–90% of its measured speed limit.
  The D1 is commanded at 10 Hz, reports at 9 Hz and needs ~220 ms to reach cruise, so this is an artefact
  of the timing model omitting the ~127 ms command delay (F-021) and the acceleration ramp (F-035). Any
  transfer claim from these policies is void until the arm model carries both. Do not retune `action_rate`
  first — that would tune against the artefact.
  *Corrected 2026-09-17 (F-045):* those figures were feedback-sampling artefacts, and the D1 can execute
  oscillations this small. The arm model now carries the fitted planner (F-046); under it the oscillation
  shrinks but persists, and retraining is the next measurement.
- **Streaming the arm at 10 Hz costs it a third of its speed** (F-046). The D1 restarts its motion plan on
  every new setpoint, so a controller streaming at 10 Hz gets ~0.8 rad/s rather than the arm's 1.27. The
  simulator now models this, and the current policies lose 5× their steady-state precision under it. Whether
  an identical re-sent setpoint also restarts the plan is untested; the simulator assumes it does. Deployment
  design (G4) has to decide how the policy's arm output reaches the D1.
- **The manifest freezes labels, not the robot model** (F-039). `latency: "estimated"` stayed `estimated`
  while the numbers behind it changed, and the arm's velocity limits were never in the conditions at all, so
  two evaluations across a changed arm both reported no mismatch. Resolve the values into the conditions before
  any policy of record exists; it costs three new manifest hashes and ~15 s per zero-action re-measurement now,
  and a great deal more later.
- **The arm model is still optimistic where it was not corrected.** The ~127 ms command-to-motion delay
  (F-021) and the two-feedback-cycle acceleration ramp (F-035) are measured and not simulated; the task models
  a velocity ceiling with no acceleration limit and a command *hold* rather than a *delay*. The measured speeds
  are lower bounds (F-033).
- **Two servo signs are inverted and `params/deploy.yaml` has no sign convention** (F-030, F-034). A policy
  deployed through the current manifest would reach to the wrong side and twist the tool the wrong way. G4
  cannot freeze the deployment contract until that is decided.
- **Nothing prices the squat.** The reward set has no base-height term, so a policy can lower the tool
  point by crouching instead of moving the arm, and the first one trained against the revised box does
  exactly that: every episode below 0.20 m, 9 mm from the `low_base` termination at worst, legs
  saturated up to 30% of an episode (F-019). The plan permits stance changes, so the fix is to price
  the squat rather than forbid it, and to re-measure. Until then no reach number from that policy is a
  baseline, and G1a stays not started.
  **Answered on 2026-09-16 (F-040):** a `base_height_l2` term at weight −50 removed it — 0 of 100
  episodes below 0.20 m, worst margin 6.8 cm rather than 9 mm, tracking 30% better and leg saturation
  halved. It exposed a rarer failure in its place: 1 of 100 episodes tips at the far top corner of the
  box, which is on G1a's ≤1% fall boundary rather than inside it.
- **The box's difficulty is not validated.** It is the nearest placement that removes free successes while staying
  reachable (F-016), chosen from a CPU search, not from any evidence about what a policy can learn. If PPO cannot
  make 13.5–28.7 cm reaches, the box, the ±1 rad action clip and the reward scales all become suspects at once.
  **Answered on 2026-09-16 (F-019):** PPO makes the reaches, reaching 100/100 on the development manifest.
  The open question is no longer whether it can be learned but how it is being learned.
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
- **Unverified estimates**, now partly measured. The arm feedback rate is no longer an estimate: it is 9.00 Hz
  (F-020), and `arm_feedback_period_steps` moved 5 → 6 on 2026-09-16, so runs recorded before that date — the
  F-019 policy included — used the old value. Still unmeasured: the D1 **speed limits** (J0 already exceeds the
  URDF's 1.05 rad/s without saturating, F-021, and J1–J5 are untouched), the **command rate** the arm will accept,
  and the **leg delay range**. The ~127 ms command→motion lag that F-021 measured is not modelled at all —
  `arm_command_hold_steps` is a hold, not a delay. Any reaching result under `--latency estimated` still depends
  on those assumptions.
- **There is no software stop for the D1** (F-022). `funcode 6 {"power": 0}` is ignored by the arm — 14,208
  consecutive commands did not clear `power_status` — so the emergency power-off that `arm_control.py` documents
  and the VIP-Rescue GUI exposes is inert. The only working abort is release (`funcode 5 {"mode": 0}`), which
  drops torque but leaves the arm powered and backdrivable; beyond that it is a physical power cut. The arm also
  **enables itself on any `funcode 1` motion command**, with no arming step, on a DDS domain shared with the
  Rescue stack. Before any Thesis B hardware run (G4 onward), the abort path has to be release plus a reachable
  physical cut, and a person must be able to reach the latter. Raised with Lukas; it affects VIP-Rescue too.
- **The arm was left energised.** `power_status` was 0 at the start of 2026-09-16 and is 1 now, because
  power-off cannot be commanded. The arm is released and holding no torque, but it stays energised until
  power-cycled.
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

Follow the revised [Week 2 plan](../week_02/notes.md): finish bounded method reproductions,
validate the P0 candidate, freeze the box/task and deployment interfaces, calibrate
camera/tag/tool frames, and begin recorded-state inference replay. Outstanding Week 1
checks remain open until their evidence is recorded.

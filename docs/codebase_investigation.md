# Codebases for Thesis B

Initial inspection: 14 September 2026. Sources are the authors' repositories,
project pages and actual source/configuration files. None of the external
repositories has been installed or run during this investigation. “Available”
here means code can be inspected, not that its reported results were reproduced.

## Recommendation

Use the existing Isaac Lab/Go2+D1 model as the primary implementation. Prioritise
the Go2+D1 Deep-WBC adaptation for robot-specific comparisons, UMI-on-Legs for
trajectory tracking/evaluation, and UniFP for the later force-aware extension.
Reproduce selected external examples in separate environments; importing their
entire older simulator and PPO stacks would make it harder to isolate changes
to this robot and the thesis ablations. This is an engineering recommendation
from the compatibility findings below, not a measured performance ranking.

## Shortlist and verified availability

| Codebase | Verified fit and available material | Compatibility and initial decision |
| --- | --- | --- |
| [Isaac Lab](https://github.com/isaac-sim/IsaacLab) | Existing local task managers, Go2 asset/config, reach examples and RSL-RL wrapper | Primary implementation; inspect the actual installed API rather than assuming the README's version label |
| [Go2+D1 Deep-WBC adaptation](https://github.com/nayon007/Loco-Manipulation-with-RL-for-Go2-D1-Robot) | Same named robot; Go2+D1 URDF, `go2d1.py`, `go2d1_config.py`, training/play scripts and modified PPO | Closest embodiment reference; Isaac Gym/Python 3.8. Audit model/gains and reproduce before trusting README performance claims |
| [Deep Whole-Body Control](https://github.com/MarkFzp/Deep-Whole-Body-Control) | Original whole-body learning reference; `legged_gym`, custom `rsl_rl`, WidowGo1 assets | Useful policy/curriculum reference; different arm/base and older simulator. Retain per-file licence notices for any adapted code |
| [UMI-on-Legs](https://github.com/real-stanford/umi-on-legs) | `mani-centric-wbc`, deployment components, documented checkpoint/data downloads, rollout and evaluation commands | Strong trajectory/evaluation reference; Isaac Gym and ARX5-related hardware/software. Checkpoint is not compatible with D1 without retraining |
| [UniFP / UnifiedForce](https://github.com/unified-force/UniFP) | B2Z1 force/position environment, training/play scripts and PPO components | Closest H1 methods reference; Isaac Gym Preview 4/Python 3.8. Training release is checked off, but ROS 2 deployment, MuJoCo transfer and imitation data collection remain unchecked in the README |
| [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) via the MaiRo RL Lab container (`~/mairo-rl-lab-rinam`, commit `a179aa0`) | Unitree's own Isaac Lab tasks: measured motor envelopes, Go2 velocity task with randomisation and an asymmetric critic, `deploy.yaml` export, ONNX export and a C++ `unitree_sdk2` controller that replays the observation and action managers on the robot. MaiRo adds a Go2 base policy and course exercises | Same simulator family as this repo (Isaac Sim 5.1); Apache-2.0. **Adopted in part** (motor model, noise/randomisation values, asymmetric critic, deploy manifest). The C++ controller is legs-only and kept as the Thesis C reference |
| [Visual Whole-Body Control](https://github.com/Ericonaldo/visual_wholebody) | Separate low-level walking/EE tracking and high-level visuomotor components; linked low-level weights | Secondary reference; Python 3.8/Isaac Gym. High-level vision is outside immediate scope; root licence is CC BY-NC 4.0 |

Sources for the platform requirements and release status:
[Go2+D1 README](https://github.com/nayon007/Loco-Manipulation-with-RL-for-Go2-D1-Robot/blob/main/README.md),
[UMI starter](https://github.com/real-stanford/umi-on-legs/blob/main/mani-centric-wbc/docs/starter.md),
[UniFP README](https://github.com/unified-force/UniFP/blob/main/README.md),
[Visual Wholebody licence](https://github.com/Ericonaldo/visual_wholebody/blob/main/LICENCE).
UniFP lists BSD-3-Clause and UMI lists MIT at their repository roots; check
individual assets and submodules as part of any actual import. No external
source code has been vendored in this first step.

## Source-level findings to act on

**Go2+D1 adaptation:** the inspected
[configuration](https://github.com/nayon007/Loco-Manipulation-with-RL-for-Go2-D1-Robot/blob/54d6c40b6d952106d90472591bdd625e0af42e6a/legged_gym/legged_gym/envs/go2d1/go2d1_config.py)
sets 18 actions, 10 s episodes, action clipping at 1.0, an observation history,
separate arm/leg action scales, and name-specific arm/leg gains. Its arm
stiffness is 5.0 and damping 0.5, unlike the local implicit-drive 4000/400
settings. Those numbers are not transferable without checking units, explicit
versus implicit control, timestep, torque clipping and asset inertials. Compare
the complete actuation path. The README's successful-training statements do not
substitute for tracking-error/failure measurements on this repo's robot.

**Deep-WBC:** the inspected
[WidowGo1 configuration](https://github.com/MarkFzp/Deep-Whole-Body-Control/blob/8159e4ed8695b2d3f62a40d2ab8d88205ac5021a/legged_gym/legged_gym/envs/widowGo1/widowGo1_config.py)
contains 18 actions, EE trajectory/hold ranges, collision exclusion bounds,
privileged/history inputs, distinct arm/leg reward terms and control heads.
Its default `RESUME = True` and zero scales on some wrist actions make it a
poor configuration to copy blindly. Start by tracing target sampling,
observations, action ordering and curriculum. The
[authors' project page](https://manipulation-locomotion.github.io/) describes
advantage mixing and online adaptation; reproducing those methods requires the
custom training code, not simply relabelling ordinary PPO as Deep-WBC.

**UMI-on-Legs:** the
[starter guide](https://github.com/real-stanford/umi-on-legs/blob/d75c9c182d8044dadf53043612da2ffbf1936a97/mani-centric-wbc/docs/starter.md)
documents rollout/evaluation commands, position/orientation error, survival and
power metrics, and comparisons including body-space targets and no trajectory
preview. This makes it useful for testing the choice of command frame and
future trajectory observations. Begin with its provided example on its own
robot; then adapt the task interface and evaluation ideas to Go2+D1.

**UniFP:** the released
[environment configuration](https://github.com/unified-force/UniFP/blob/68847a070f88d731058c3d8476929bc3b205f5bd/legged_gym/envs/b2/b2z1_pos_force_config.py)
and the README identify the relevant B2Z1 force/position environment and
`ppo_cse_pf` components. Trace force-command sampling, simulated external forces,
history-based estimation and observation routing before implementing H1.
Remove force inputs, force-derived latents and force objectives deliberately
for a position-only ablation. Setting only the force command to zero does not
establish that a policy is force-unaware. The released training code is a
reference for method design; D1 dynamics and contact fixtures still need their
own validation.

**unitree_rl_lab / MaiRo** (inspected 14 September 2026, local clone). It was recommended for
sim-to-real transfer from accurate motor models. Findings from source:

- `UnitreeActuator` is Isaac Lab's `DelayedPDActuator`: explicit PD torques, then a
  measured envelope. Full torque holds until a knee speed, then falls linearly to
  zero at the no-load speed. Torque with the motion (Y1) is capped separately from
  torque against it (Y2), and smoothed Coulomb and viscous friction are subtracted.
  The Go2 motor is Y1 20.2 N·m, Y2 23.4 N·m, X1 13.5 rad/s, X2 30 rad/s. Friction
  and armature values are given for G1/H1 motors but not the Go2. Isaac Lab's stock
  Go2 `DCMotor` allows 12.9 N·m at 13.5 rad/s where the Unitree model allows 20.2 N·m.
  It also allows 23.5 N·m of braking at 20 rad/s where the Unitree model allows
  14.2 N·m (F-005).
- The shipped base policy (`rinam-base-policy/params/env.yaml`) trained with
  `min_delay = max_delay = 0` and zero Go2 friction. So "accurate motor model" means
  the torque–speed envelope, not latency or friction identification.
- The Go2 velocity task keeps base linear velocity out of the actor. The critic gets
  it plus joint torques. Noise is ±0.2 rad/s angular velocity, ±0.05 gravity,
  ±0.01 rad joint position and ±1.5 rad/s joint velocity. Randomisation covers
  friction, restitution, base mass and CoM, and pushes. Solver iterations are 8/4
  with self-collisions on.
- `export_deploy_cfg.py` writes `params/deploy.yaml`: joint-to-SDK index map, gains,
  default pose, action scale/offset/clip, and observation scale/clip/history. The C++
  controller (`deploy/include/isaaclab/...`) rebuilds the same managers from that file
  and runs the ONNX policy at `step_dt`. This parity, rather than the motor model, is
  what protects against silent sim/real mismatches in observation processing.
- Sim-to-real evidence in the repo is demo footage (`doc/sim.gif`, `doc/real.gif`).
  No tracking or transfer metrics are published. MaiRo's own diff is course material:
  a gallop-gait reward, flat terrain, ±1.0 m/s pushes and README changes.
- No D1 content: the repo has no arm model, D1 parameters or latency values. Its
  changelog adds delay and friction *support*; the Go2 policy uses neither. The D1's
  servos close their own position loop, and Unitree publishes no D1 torque–speed
  curve (searched 14 September 2026: retail spec sheets list range, reach, payload
  and power only). So the D1 model adopted here combines published torque limits,
  URDF speed limits (unverified) and the D1 SDK's 10 Hz command and feedback timing.
  Leg latency uses the actuator's delay buffer, ranged from the deploy loop's
  1 kHz structure (F-007).

## Reproducible source references

These default-branch revisions were resolved through GitHub during inspection.
They are candidate pins for a later isolated checkout, not locally reproduced
experiments. If upstream changes, retain these revisions in the comparison log.

| Repository | Inspected revision |
| --- | --- |
| Go2+D1 adaptation | [`54d6c40b6d95`](https://github.com/nayon007/Loco-Manipulation-with-RL-for-Go2-D1-Robot/commit/54d6c40b6d952106d90472591bdd625e0af42e6a) |
| Deep-WBC | [`8159e4ed8695`](https://github.com/MarkFzp/Deep-Whole-Body-Control/commit/8159e4ed8695b2d3f62a40d2ab8d88205ac5021a) |
| UMI-on-Legs | [`d75c9c182d80`](https://github.com/real-stanford/umi-on-legs/commit/d75c9c182d8044dadf53043612da2ffbf1936a97) |
| UniFP | [`68847a070f88`](https://github.com/unified-force/UniFP/commit/68847a070f88d731058c3d8476929bc3b205f5bd) |
| MaiRo RL Lab (unitree_rl_lab inside) | `a179aa0` (local clone of `github.com/vick-l1m/mairo-rl-lab-rinam`) |

## Local implementation audit

The local installation is Isaac Sim `5.1.0.0`, Python `3.11.15`, PyTorch
`2.7.0+cu128`, Isaac Lab package `0.54.3`, Isaac Lab RL `0.5.0`, and RSL-RL
`5.0.1`. `/home/lukas/IsaacLab` is at
`4df6560e187f2cc66685b41b21b259f4485d0c22` (`training-checkpoints-develop`),
with untracked local extensions. Package version labels are not a substitute
for the Isaac Lab repository revision. Do not overwrite or upgrade this shared
installation as part of reproducing an Isaac Gym paper.

| Existing component | Reuse | Work required for thesis training |
| --- | --- | --- |
| `weld.py`, D1 URDF | Single articulation and existing mass model | Audit inertials/limits, version referenced assets, check physical responses |
| `flat_env_cfg.py` | Robot factory, flat terrain, initial stance, reset events | Playback has 12 actions, frozen velocity commands and disabled episode terminations; new task supplies its own RL managers |
| `agent_cfg.py` and walking checkpoint | Playback/diagnostic comparison | Existing checkpoint/network has different observations/actions; fresh stochastic PPO actor needed for training |
| `d1_ik_controller.py`, `d1_direct.py` | Diagnostic position controller and interface assumptions | Current client broadcasts environment 0 feedback/targets; it must be vectorised before use as a parallel IK baseline |
| Contact sensor | Feet/base diagnostics in old task | D1 links are nested; new task declares a separate arm sensor and checks its resolved bodies |
| `ros2.py` | Later observation/command parity and RViz inspection | Keep bridge and lidar outside high-throughput training; verify deployment interface later |

## Bounded reproduction experiments

| Priority | Experiment | Evidence to collect / stop condition |
| --- | --- | --- |
| 1 | Local Isaac Lab task, then original playback | Smoke/reset/frame checks, joint mapping, model mass, tracking/fall traces; stop PPO work if physics/interface checks fail. **2026-09-15: launched; checks pass (F-009); playback replayed with forward, lateral and yaw commands (F-012)** |
| 2 | Go2+D1 adaptation in its own Isaac Gym environment | Reproduce advertised launch; record exact assets/gains/revision and actual reaching/locomotion metrics; time-box setup to two working days |
| 3 | UMI example checkpoint and its supplied trajectory | Verify downloads and rollout, reproduce reported metric definitions; keep original embodiment clearly labelled |
| 4 | Deep-WBC source trace and a pilot only if useful | Identify useful curriculum/advantage-mixing features; avoid a second long reproduction unless it resolves a specific baseline failure |
| 1b | unitree_rl_lab parts adopted into this repo | Smoke with `--leg_actuator unitree` and `dc_motor`; inspect `params/deploy.yaml`; confirm explicit-actuator stability with the welded arm. Revert to `dc_motor` if the explicit legs are unstable. **2026-09-15: done; explicit legs stand stably, matching `dc_motor` to 0.1 mm, and the manifest checks out ([Week 1](../results/week_01/notes.md))** |
| 5 | UniFP example when force work begins | Confirm released training starts; audit force-estimation supervision and position-only ablation before porting |

Record each attempt as source-only / installed / launched / checkpoint replayed /
short training passed / independently evaluated. Store setup time, throughput,
dependencies, missing assets and errors. Select the implementation at the end
of Week 2; continue method investigation through approximately Week 6 without
letting it delay the local baseline.

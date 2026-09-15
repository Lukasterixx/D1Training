# Rescue flat ablation (2026-09-14/15)

Evidence copied from Lukas's Rescue project, where the experiments were run. It is recorded here
because it bears on training a Go2 with the D1 arm attached (finding F-008). At the time of copying,
none of this was committed in Rescue (`~/Rescue` HEAD `46e2411`, 2026-07-17, with `training/`
untracked), so these copies are the versioned record.

## What was run

Locomotion velocity-tracking policies from Rescue's flat task (`go2_rescue/flat_env_cfg.py`). They
have 12 leg actions and leg-only joint observations. The D1 is welded on but passive: folded, PD-held
at zero and never randomised. It is not the D1Training position-only task.

| Configuration | What changed from the Rescue flat task | Run (`~/IsaacLab/logs/rsl_rl/…`) | Seed |
| --- | --- | --- | --- |
| Rescue, seed 42 | Unchanged: arm welded on, measured Go2 motor, MaiRo randomisation, 10 cm clearance target, symmetric CoM range | `go2_rescue_flat/2026-09-14_21-06-31_flat_arm`, checkpoint 2000 of 3000 | 42 |
| Rescue, seed 2 | Unchanged | `go2_rescue_flat/2026-09-15_10-05-40_abl_Rescue_s2` | 2 |
| no MaiRo DR | Friction fixed 0.8/0.6, no restitution, no pushes, no joint-velocity reset, no tip-over termination | `…_08-28-45_abl_FlatNoDR` | 42 |
| no arm | Bare Go2 as the training robot (measured motor kept) | `…_08-56-00_abl_FlatNoArm` | 42 |
| stock motor | Isaac Lab's stock `DCMotor` legs instead of the measured curve | `…_09-16-43_abl_FlatNoAct` | 42 |
| 8 cm clearance | Swing-clearance target 8 cm instead of 10 cm | `…_09-40-26_abl_FlatClear08` | 42 |
| all removed | All of the above removed, plus P2Dingo's 0..+5 cm CoM range; should reproduce P2Dingo | `…_08-09-04_abl_FlatP2D` | 42 |
| P2Dingo flat | P2Dingo's own flat policy, the reference | `go2_p2dingo_flat/2026-09-12_15-06-39`, checkpoint 1999 | — |

Every ablation trained for 2000 iterations from scratch with 4096 environments and 24 steps per
environment per iteration: about 196.6 M transitions. The actions were clipped at ±20.
`training_params/<run>/` holds each run's exact Isaac Lab `env.yaml` and `agent.yaml`, and
`checkpoints.json` holds the SHA-256 of every benchmarked checkpoint.

## How it was measured

`rescue_source/measure_bench.py`, seed 1, 200 robots per terrain, `--flat_only`. Every policy ran
on the **same welded Go2+D1 with the measured motor**: the robot Rescue's sim drives, with no base-mass
or CoM randomisation. The text outputs in `bench/` define every column. The ones used here:

- **Foot vs neutral point**: touchdown position relative to Raibert's neutral point v·T_stance/2, in the
  yaw frame. 0 means the foot lands where the stance centres under the hip; + means it lands ahead.
- **Same-side spacing**: front foot ahead of the same-side rear foot at the front touchdown. The hips are
  38.7 cm apart.
- **Backward after request**: % of robots moving below −0.05 m/s within 1 s of a 1.0 m/s forward command.
- **Pitch wobble**: pitch minus its own 0.5 s moving average, as a standard deviation.
- **Turn tracking**: in-place yaw rate as % of the command. The 0.2 and 0.5 rad/s values come from
  separate runs in `bench/2026-09-15_turn_rates/`.

## Files

| Path | Contents |
| --- | --- |
| `bench/2026-09-14_flat/` | P2Dingo flat and Rescue seed 42 (checkpoints 2000 and 2999), 0.5 and 1.0 m/s |
| `bench/2026-09-15_flat_ablation/` | The six 2026-09-15 runs, 0.5 and 1.0 m/s |
| `bench/2026-09-15_turn_rates/` | 0.2 and 0.5 rad/s turns for five policies, including Rescue's installed stair policy (model_7700) |
| `training_params/` | `env.yaml` and `agent.yaml` for each training run |
| `checkpoints.json` | SHA-256 and size of each benchmarked checkpoint (the weights stay in `~/IsaacLab/logs`) |
| `rescue_source/` | Snapshots of `experiments.py`, `measure_bench.py`, `bench_table.py`, `go2_actuators.py` and Rescue's `training/README.md` (the ablation section starts at "## Flat ground") |
| `summarise.py` | Rebuilds `summary.csv` and `../../figures/rescue_flat_ablation_gait.png` from `bench/` |
| `summary.csv` | The table in the Week 1 notes, unrounded |

The numbers were cross-checked on 2026-09-15: every value in Rescue's README table and in its
chat summary matches the JSON here.

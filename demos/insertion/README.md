# Insertion demo

Combiner boxes often have no handle: the door has a key slot, and a removable handle is pushed into it and turned.
This demo is the push: the standing Go2 + D1 holds a key and inserts it into a keyed slot on a door, force-aware,
from an estimate of where the slot is (Lukas, 2026-09-26). The turn is the lever work in `demos/unifp` (F-110..F-114).

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

./demos/insertion/run_demo.py --checkpoint <insertion-trained model.pt> --compliant_key --num_envs 1 --attempts 1 --lateral_mm 2   # watch one
./demos/insertion/run_demo.py --checkpoint <model.pt> --compliant_key --attempts 16 --headless --lateral_mm 3                     # measure
```

Then `./dashboard.py record <out dir> --title "..."`; runs go to `logs/insertion_demo/`. The insertion-trained policy to
use is `checkpoints/unifp_go2d1_insertion_seed3_model_21200.pt`, with `--compliant_key` (it was trained holding the key
compliantly; with a rigid key it seats far fewer, see `checkpoints/README.md`).

## What runs

| piece | what it is |
| --- | --- |
| the key | a flat blade, 10 × 3 mm, its tip 70 mm beyond the jaw centre along the approach, its long side along the jaw axis (`key.py`, a copy of the D1 URDF, the robot welded into `generated/insertion/`). Rigid (a collider on `Link6`), or with `--compliant_key` on its own link hung from the jaw centre by two sprung revolute joints, 2 N·m/rad, ±10° -- the PhysX form of the training's `insertion.CompliantGrip` |
| the slot | a channel 1 mm wider than the blade each side, 18 mm deep to a seat, a 2 mm 45° chamfer round the mouth, in a lock body set in a door panel (`asset.py`, all box and convex colliders, 1 mm contact offsets); placed per attempt, kinematic |
| the estimate | what the task layer believes: the true slot moved `--lateral_mm` across its axis in a random direction, its axis tilted `--tilt_deg`, its roll off `--roll_error_deg` -- the error a tag seen from the approach might leave |
| the task layer | `unifp_train.insertion.InsertionTaskLayer`, shared with training: the hand to the pre-insertion pose (key tip 12 mm short of the estimated mouth, approach along the estimated axis, roll the blade's); then a PI force law along the axis on how far the tip lags a reference depth, capped at `--search_n` while the key is on the door's face and spiralling across it if it stalls, at `--max_n` once it is in; across the axis the goal yields to the tip once it has touched. Its outer loops (`insertion.PoseCorrection`, `--tip_correction`, `--attitude_correction`; on by default) integrate the measured miss of the tip, the approach's elevation and the roll into the commands |
| the policy | an insertion-trained checkpoint (`unifp_train --task insertion`): position, force, roll and the new **approach** command (`rewards.encode_approach`, the channel UniFP declares and never writes) |

The environment is the training's (`env.py` subclasses `unifp_train.insert_env.Go2D1InsertEnv`) with PhysX doing the
contact: no analytic force is applied, and the analytic slot is stepped only to measure depth.

**Seated** is the key's tip reaching the seat in the true slot, which PhysX only allows through the channel.

## Where it stands

**2026-09-27 (F-119, F-120, F-121):** keys seat. With the task layer's outer loops, the key held in a compliant grip,
and the policy trained with low arm noise, then more slowly at a pinned learning rate, the best checkpoint
(`checkpoints/unifp_go2d1_insertion_seed3_model_21200.pt`, F-122) seats 75/96 frozen 1 mm slots in the training plant
and 43 of 48 attempts in PhysX at a 2 mm estimate error, with no damage and no falls. What it does worst is a level blade, the higher the worse (13, 9 and 6 of 16
frozen slots at 0.70, 0.78 and 0.86 m, against 15-16 upright): the approach's elevation error tilts a level key along
its tightest side. The Week 2 log of 2026-09-27 has the path there. The section below is the state on 2026-09-26, kept
as it was.

### 2026-09-26 (F-116)

**No key has been seated yet.** The demo, the training plant, the task layer and a frozen evaluation work; the policy
does not. After 2,500 iterations of training (`unifp_train --task insertion`, run v5) the policy arrives with its approach
9-10° off and its key's tip centimetres from the slot: 0 of 16 seated in PhysX at estimate errors of 0, 2 and 4 mm, 0 of
192 frozen slots in the training plant. The untrained mechanism policy (`checkpoints/unifp_go2d1_mech_law_model_17499.pt`)
reaches the door more often but rams it. `--tip_correction 1.5` (the jaw goal placed for the *tip*, from the hand's
measured approach, and an integral of the tip's miss) gets the key to the door and searching in 6-10 of 16, still none
in. The v5 checkpoint is `logs/unifp_train/20260925T164048_train_seed1_ins_v5/model_19998.pt` on the training machine; it
is not in `checkpoints/`, because it does not do the task. What to try next is in F-116.

## What this does not show

- **A real key's grip.** A rigid key is part of the hand. The compliant key tilts on ideal springs about the jaw
  centre, and nothing yet built holds a key like that. A real grip can also slip.
- **The detent.** The training's plant has one; this lock has none.
- **The turn after the push.**
- **A real estimate.** The estimate here is only ever a known error added to the truth.
- **A measured tip.** The loops read the key's tip and the hand's attitude from the simulator; a robot would have
  to measure them (the arm's kinematics, F-118, or a wrist camera).
- **Anything on hardware.**

The slot's sizes are the middle of the training's ranges, not a measured lock.

## The pieces

| file | |
| --- | --- |
| `run_demo.py` | the launcher: builds the key robot and the door, draws the attempts, runs the policy, writes `run.json` / `summary.json` / `trace.csv` |
| `env.py` | the training's insertion environment with the door and key in PhysX |
| `asset.py` | the door panel and lock body with the keyed slot, as USD |
| `key.py` | the D1 URDF with the key's blade on `Link6`, or on its own sprung link (`compliant=True`) |
| `geometry.py` | the sizes, shared with the training's plant, and the door's placement |
| `tests/` | `python -m unittest discover -s demos/insertion/tests -t .` |

Training and its frozen evaluation are in `unifp_train/` (`insertion.py`, `insert_env.py`, `insert_cfg.py`,
`insert_rewards.py`, `insert_eval.py`); see its README.

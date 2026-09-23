# unifp_isaaclab — UniFP's trained Go2+D1 policy, run in Isaac Sim

`unifp_go2d1/` trains UniFP's position/force policy on the legacy stack (Isaac Gym Preview 4,
Python 3.8). This package runs the resulting checkpoint — `model_48800`, the one F-072 and F-075
are about — on **this** repository's Isaac Lab welded Go2+D1 instead. Nothing is retrained and
nothing is fine-tuned.

It exists because every question after "the policy works" needs it. The policy was measured in the
simulator it was trained in, which cannot distinguish a controller that has learned the task from
one that has learned that simulator. Running it unchanged on a differently-built model of the same
robot is the cheapest test that separates those, and it is the step before hardware can be argued
for at all.

## What it is not

**It is not a hardware result, and it is not evidence that the policy transfers.** A sim-to-sim
agreement rules out one failure mode. A disagreement does not implicate the policy on its own,
because the two sides differ in ways that have nothing to do with it — see
[the deviation list in `robot.py`](robot.py), which is the first thing to read before drawing a
conclusion from a gap. The largest are: the robot is built from a different file on each side
(a generated URDF against a welded USD), and the ground here is a flat plane where UniFP's is
flat-but-rough trimesh.

## Layout

| File | Role |
| --- | --- |
| `interface.py` | the contract `model_48800` imposes: DOF order, observation layout and scales, gains, timing. No Isaac imports. |
| `policy.py` | the checkpoint rebuilt as plain PyTorch — no Isaac Gym, no `b2_gym_learn` — plus the 32-frame observation history. |
| `robot.py` | the Isaac Lab articulation, controlled UniFP's way, **and the list of what is not reproduced**. |
| `task.py` | UniFP's moving end-effector goal generator. |
| `rollout.py` | the 50 Hz policy loop over a 200 Hz torque loop. |
| `../run_unifp_isaaclab.py` | the launcher: `check` (no simulator) and `play`. |
| `../unifp_go2d1/dump_interface.py` | records the interface out of the *training* stack, which is what makes the port checkable. |

## How the port is checked

The two halves are verified separately, with no simulator on either side, against 128 policy steps
recorded out of the running Isaac Gym environment
(`tests/data/unifp_interface_48800.npz`, from `Sep18_11-10-15_/model_48800.pt`):

- **the contract** — `interface.single_obs` rebuilds UniFP's 76-wide observation from the same raw
  state, and every constant matches what the environment actually had; and
- **the network** — `policy.UniFPPolicy` produces the same 18 actions from the same observations.

Both agree to float32 rounding (`< 1e-5`). Run them with
`python -m unittest tests.test_unifp_interface`. The network tests need the checkpoint itself
(24 MB, not in this repository) and skip without it; point `UNIFP_CHECKPOINT` at it or leave it
where `unifp_go2d1` put it.

This split matters for reading a result. If these tests pass, the port describes the policy
correctly, and any difference in Isaac Sim is the *physics* differing — not the observation, the
action mapping or the weights.

## Running it

```bash
conda activate env_isaaclab
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

# no simulator: the checkpoint loads and matches the interface
./run_unifp_isaaclab.py check

# watch it walk
./run_unifp_isaaclab.py play --command 0.5 0 0

# headless and recorded
./run_unifp_isaaclab.py play --headless --steps 1500 --command 0.5 0 0 \
    --out logs/unifp_isaaclab/$(date -u +%Y%m%dT%H%M%S)_play48800
./dashboard.py record logs/unifp_isaaclab/<run> --title "..."
```

`--command VX VY WZ` holds a base velocity command. Inside the dead zone
(|vx| < 0.1, |vy| < 0.1, |wz| < 0.2) the policy is being told to **stand**, and the gait phase is
pinned to zero — that is the trained behaviour, not a bug.

The two force flags are different things, and upstream keeps them separate for a reason:

- `--ee_force_cmd FX FY FZ` is the force the policy is **asked to produce** at the tool tip. It
  enters the observation; nothing external is applied. Trained range is ±8 N.
- `--ee_force_ext FX FY FZ` is a force **actually applied** to the tool tip as an external
  wrench — a disturbance to reject. The policy never observes it directly; it can only infer it
  through the adaptation module, which is what `force_est_n` in the trace reports.

## What it found, 2026-09-20

`model_48800`, flat ground, no external forces, no randomisation, one robot, a held velocity
command, compared over 18 s (F-077):

| condition | | Isaac Gym | Isaac Lab |
| --- | --- | --- | --- |
| zero actions | base height | 23.8 cm | 27.6 cm |
| standing | tool-tip error L1 | 2.6 cm | 8.7 cm |
| walking, 0.5 m/s | tool-tip error L1 | 3.3 cm | 69.7 cm (fallen) |

**Standing transfers; walking does not** — told to walk, the robot is on its side within 0.5 s,
while the same command with zero actions leaves it standing. And the standing result is
**marginal**: raising the PhysX solver iterations to 8/4 makes the same policy fall immediately
while barely moving the passive robot, so it must not be quoted on its own. The walking failure
survives that change; the standing success does not. Read `robot.py`'s deviation list before
attributing any of this to the policy — the foot colliders differ between the two models, and
that is the first suspect for a pair of stacks that agree standing and disagree walking.

Getting this far took four bug fixes, three of which were silent (F-076): the joint state was
never written to the sim, `self_collisions = 0` in legged_gym means *enabled*, and matching Isaac
Gym's zero armature made UniFP's arm gains non-integrable here — `Joint4` left its ±2.35 rad limit
and reached −127 rad. Each one looked exactly like "the policy does not transfer".

## Comparing the two stacks

`trace.csv` here opens with the same columns as `unifp_go2d1/play_policy.py --out` writes, so the
two can be read side by side without a translation step. For the comparison to be about the
simulators rather than the terrain, dump the Isaac Gym side on flat ground too:

```bash
python unifp_go2d1/play_policy.py --task=go2d1_pos_force --load_run <run> --checkpoint 48800 \
    --headless --flat_terrain --out <dir>
```

Remember that neither side is bitwise reproducible (F-075: the same controller on the same
manifest moves medians by about ±0.1 cm run to run in Isaac Gym), so a difference smaller than
that is nothing.

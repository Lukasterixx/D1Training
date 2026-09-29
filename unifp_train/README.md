# unifp_train — UniFP's task, trained natively in Isaac Lab

`unifp_isaaclab/` takes a checkpoint trained on the legacy Isaac Gym stack and runs it here.
This package is the other direction: rebuild the **task** in Isaac Lab so a policy can be trained
on the stack it will be evaluated on.

The motivation is F-088. The Isaac Gym policy holds a stance on the Isaac Lab model but falls
within 0.5 s when told to walk, and whether it stands at all turns on a PhysX solver setting. That
is not a defect to debug so much as a reason not to move trained weights between these two
simulators at all.

## Status

| piece | state |
| --- | --- |
| Observation contract, DOF order, control law, robot | **done**, reused from `unifp_isaaclab/` (F-087: exact to 6e-8 / 9.5e-6) |
| End-effector goal generator | **done**, reused from `unifp_isaaclab/task.py` |
| Gait clock: stance mask, reference leg pose (`gait.py`) | **done and verified** against the training environment |
| Task constants: weights, ranges, limits (`task_cfg.py`) | **done**, all 27 weights checked against the running environment |
| All 27 reward terms (`rewards.py`) | **done and verified** to 4.8e-07, four of them unexercised (below) |
| Privileged 153-wide critic observation (`observations.py`) | **done and verified** to 3.5e-07, every one of the 18 blocks |
| The environment (`env.py`, `env_cfg.py`) — `DirectRLEnv`, resets, terminations, velocity command sampling | **runs**, and its 27 reward terms agree with the Isaac Gym recording in aggregate (below) |
| External force schedule (`forces.py`) | **done and verified** step-for-step against a transcription of upstream's `_push_gripper` |
| The wrench and the command channel, wired into the environment | **done**, and measured to move the arm |
| Force curriculum (`cfg.force_start_step`, 8,000 iterations) | **done** |
| Adaptation-module actor-critic (`models.py`) | **done and verified** — `model_48800`'s weights load into it and it reproduces the reference loader's actions exactly |
| The extra estimator loss on rsl-rl 5.x (`algorithm.py`, `agent.py`) | **done**, runs and the loss falls; not compared against upstream's optimiser |
| Training | **fixed, and a full run completed** (F-091, F-092): 60,000 iterations in 43.5 h with no collapse, the end-effector term above the Isaac Gym reference throughout, and `kl_first_minibatch` never above 1.6e-09. Previously (F-091): resetting an environment blanked an observation the policy had already acted on, which pinned the learning-rate schedule to its floor for the whole run. On probe D's configuration the rate now sits at 1.98e-04 rather than the 1e-5 floor and the end-effector term goes 0.618 → 1.465. Verified over 300 iterations, one seed — **not** over a full run |
| Evaluation against the frozen manifests (`eval.py`) | **done** (F-092): the natively trained policy beats the ported Isaac Gym one, **1.5 cm against 3.9 cm** on the same 50 frozen episodes in the same simulator, better on 50 of 50 paired episodes, no falls. Supersedes F-090, which measured a run made before the F-091 fix and stopped at 44% |

## How the port is checked

The same method that made the playback port trustworthy: record the contract out of the *running*
Isaac Gym environment and compare, with no simulator on either side.
`unifp_go2d1/dump_interface.py` writes 160 steps of full state plus **each of the 27 reward terms
separately**, taken as the change in the environment's own `episode_sums` — not by re-evaluating
the reward functions, because `_reward_feet_air_time` mutates state and a second call per step
would change the very run being recorded.

`tests/test_unifp_train.py` then checks every term against its own source. Worst error across the
terms that actually fire: **4.8e-07**.

Two conventions were established by measurement rather than assumed, and both are easy to get
wrong silently:

- **A reward is computed from post-step state** — the reward recorded at step `t` uses the state
  recorded at `t+1`. Scanning the offset gives 1e-10 at `t+1` and 1e-2 at either neighbour.
- **`feet_air_time` and `last_contacts` are the exception**, because they are mutated inside the
  reward itself, so the value at index `t` is what step `t`'s reward starts from.

### What the verification does not cover

Four terms — `collision`, `dof_pos_limits`, `feet_height_high`, `stand_still` — are identically
zero throughout the reference rollout: the robot never collides, never reaches a joint limit,
never lifts a foot past 20 cm and is never commanded to stand. Their arithmetic runs, but agreeing
with a column of zeros is not evidence. A rollout that provokes them is needed before anything
rests on those four. `test_unexercised_terms_are_declared` fails if a future fixture starts
exercising one, so the list cannot quietly go stale.

`collision` is weaker still: the fixture carries no contact forces for the penalised bodies, so
the test feeds it zeros. It is transcribed from upstream, not verified.

## The environment against the one it was ported from

`./run_unifp_train.py smoke` builds it and steps it; `play` drives it with an existing UniFP
checkpoint, which exercises the whole loop — observation, action mapping, physics, reward —
against a policy already known to work on this robot. Both write a run directory with **all 27
reward terms as per-step means**, which is what makes the comparison below possible: a single
total can agree while two terms are wrong in opposite directions.

| `model_48800`, flat ground, no forces | reward / step | tool-tip error, median |
| --- | --- | --- |
| Isaac Gym, 1 env x 160 steps (`tests/data/unifp_task_48800.npz`) | 0.17586 | 2.0 cm |
| **Isaac Lab port**, 16 envs x 400 steps | **0.17519** (−0.4%) | 3.8 cm |
| Isaac Lab port, zero actions | 0.09042 | 41.5 cm |

Term by term (`results/week_01/figures/unifp_task_terms.png`), 23 of the 27 agree to better than
0.0005 per step. The four that do not:

- **`stand_still` +0.00267** and **`dof_pos_limits` −0.00021** are two of the four terms the
  reference rollout never exercises — 160 steps of one environment drew no standing command,
  where 16 environments over 400 steps draw plenty. This is the fixture being small, not a
  disagreement, and it is larger than the total gap: without it the port sits 1.9% *below*
  Isaac Gym rather than 0.4%.
- **`tracking_ee_force_world` −0.00148** and **`tracking_lin_vel_force_world` −0.00124**, the two
  objectives, are each about 3–4% lower. The policy tracks slightly worse on the Isaac Lab model,
  which is F-088 in a much milder form than the standing/walking playback showed.
- **`action_rate_arm` −0.00107**, four times upstream's magnitude: the arm chatters more here.
  The rotor inertia this port has to add (`unifp_isaaclab/robot.py`, `ARM_ARMATURE`) and a
  different integrator are the obvious suspects, and neither has been separated from the other.
  *2026-09-24 (F-102):* it is the armature. At 2e-4 kg·m² the wrist's explicit PD is in a
  numerical limit cycle — torque sign flipping on 98–99.8% of physics steps at 75–90% of its limits,
  with zero actions — and at 0.01 it is still. Arm torque read out of this configuration is mostly
  that oscillation; `--task hook` runs the arm at 0.01.

**What this is and is not.** It is not a step-for-step verification: the two stacks draw their own
velocity commands and goal trajectories, so these are two different samples of the same task, and
the Isaac Gym one is a single environment. It *is* enough to catch a wrong environment, which the
earlier version of this table was not — the version that reported 0.141 against 0.176 was reading
a bug, not a simulator difference. See below.

### The bug that table found

The tool tip was measured in the simulation frame and its goal in the environment frame. With one
environment at the origin the two agree; with 16 on a 2.5 m grid they differ by metres, and
`tracking_ee_force_world` — `exp(-2 * error)` — is then exactly zero for every environment, worth
0.0000 in the log and nothing in the gradient. Nothing else in the task changes, so the only
symptom was a total reward that looked plausibly low and was read as a simulator difference.

It was found by training, not by testing: the per-term log printed
`Episode_Reward/tracking_ee_force_world: 0.0000` for twelve consecutive iterations while every
other term moved. `run_unifp_train.py` now reports the median tool-tip error and refuses a run
where it exceeds a metre, because a metre is not a tracking error, it is a frame error.

## How the external forces are checked

`forces.py` is stepped against a transcription of upstream's own `_push_gripper`, both driven from
one seed, for 1200 steps across 12 environments — agreement to 1e-6 through 35 pushes, 29 of them
run to completion, with 14% of environment-steps in the "freed" state where a drawn push is
suppressed. That is a differential test against the source, not against a restatement of it.

Two things it does not cover:

- **Stepping rate.** Upstream evaluates the schedule inside its decimation loop, four times per
  policy step against a counter that changes once. The ramps are a pure function of that counter
  so three of those calls recompute the same numbers, and the draw is simply redrawn with the last
  winning — but upstream can also re-fire a push within one policy step when a finished push
  redraws an interval the current step divides, about 0.4% of the time. This runs once per step
  and cannot.
- **The wrench reaching the physics.** Measured separately, half the environments pushed and half
  not, from the same reset: 8 N down moves the tool tip 3.3 mm down; 8 N forward moves it 409 mm
  forward and 335 mm up. The asymmetry is the arm, not the port — at ±8 N the wrist torque limits
  saturate in the weak direction, which is the band `unifp_go2d1` chose deliberately.

The one Isaac Gym run with forces on ([recorded 1 env, 1500 steps](../results/week_01/runs/20260920T0945_play48800_forces))
has a commanded force active 45% of the time. This schedule averages 64%, but a single environment
over that window is a wide distribution — 5th to 95th percentile 0.37 to 0.89 — and 0.45 sits at
its 12th percentile. Consistent, and too small a sample to say more.

## The adaptation module

`models.UniFPActor` is the encoder/actor/decoder network as one rsl-rl `MLPModel`: the encoder
takes all 32 stacked frames to a 64-wide latent, the actor body takes the newest frame plus that
latent, and the decoder is trained to reproduce the four privileged quantities the actor never
sees. `algorithm.UniFPPPO` adds the estimator loss, interleaved per mini-batch exactly where
upstream puts it, without reimplementing PPO.

The architecture is verified rather than asserted: `model_48800`'s weights load into `UniFPActor`
with `strict=True`, the remaining `critic_body` loads into a stock `MLPModel`, and the loaded
actor reproduces `unifp_isaaclab.policy.UniFPPolicy` — itself checked against the running Isaac
Gym environment — to **0.0** on actions and 7e-09 on the decoder output. Shapes alone would not
settle this; an encoder concatenated on the wrong side of the newest frame has identical shapes.

What is *not* checked is the optimisation: that the estimator loss falls (3.63 to 3.44 over 12
iterations) and that PPO runs at 28k steps/s says the pieces are connected, not that the two
together reproduce upstream's learning dynamics.

## Screening a run before you spend forty hours on it

`./unifp_train/kl_drift.py` reads the first 300 iterations of any run and reports whether the KL
divergence is holding or climbing.

The long run of 2026-09-20 collapsed three times, and each collapse was only visible in advance as
the KL creeping upward over thousands of iterations. Measured across nine configurations, the
**drift** over the first 300 iterations predicts the loss of end-effector tracking with r = −0.74,
while the mean KL predicts almost nothing. So an eight-hour question became a thirteen-minute one.

Read it as a screen and not a verdict. Each of those nine points is a single run, r = −0.74
explains about half the variance, and the table already contains a counter-example: the
`learning_rate_scale 0.1` run has the third-lowest drift and the *worst* tracking loss. A run that
fails the screen is worth stopping; a run that passes it still has to be watched.

All nine of those configurations were measured **before** the F-091 fix, when most of the KL being
screened was spurious, so the r = −0.74 calibration is from the broken regime. The screen itself
held up: re-running the five-seed scan with the fix moved every seed from the amber and red bands
into the green one, median drift +51% → +8%, and the end-effector term from −20.9% to +21.0%.
Six of six configurations run since the fix pass it; six of seven before it did not.

**`Loss/kl_first_minibatch` is the standing check.** The first mini-batch of the first epoch has
taken no gradient step since the rollout, so its KL is the numerical floor and nothing else. It
should be ~0 on every run. When it is not, the update is not seeing the policy that acted, and
`--diagnose_storage` prints which samples disagree and whether they are the ones whose episode
ended. That is how F-091 was found: it read 0.063 with both optimisers switched off.

The KL itself is logged because rsl-rl computes it for its adaptive learning rate and then throws
it away, which leaves the rate as the only visible trace — and a rate pinned at its floor is
equally consistent with a KL of 0.006 sitting harmlessly inside the schedule's dead band and a KL
of 0.5. `algorithm.UniFPPPO` wraps the actor's KL function for the duration of the update rather
than reimplementing the loop, so the schedule sees exactly what it saw before.

## Evaluating a checkpoint

```
./run_unifp_train.py manifest --role validation --num_envs 50 --seed 20260921 --headless
./run_unifp_train.py eval --manifest results/manifests/unifp_isaaclab_validation.json \
  --checkpoint <path> --headless          # or --zero_actions for the baseline
```

Both a UniFP checkpoint and an rsl-rl one load through the same path, so the policy trained on the
legacy stack and one trained here are scored by identical code in identical episodes — which is
the only way the question "is training natively better than porting the weights" has an answer.
It currently does not: F-090.

The manifest is **specific to this simulator**. `results/manifests/unifp_validation.json` freezes
its episodes by a seed and an environment count, and verifies each by a digest of the schedule the
environment realised; that works because every draw comes from the global torch RNG in an order
fixed by the *Isaac Gym* code path. This environment draws the same quantities from different code
in a different order, so a shared seed does not give shared episodes and those digests can never
match. Hence a separate format, `unifp_isaaclab_eval_manifest_v1`, which refuses to load the other
one rather than failing later at the digest check.

**Select checkpoints by this, not by training return.** Between iterations 20,000 and 26,400 of the
first training run the mean return never left 130–137 while evaluated tracking went from 6.2 cm to
60.8 cm — worse than doing nothing. The reward does not see it.

## Why the reward weights carry a `dt`

`legged_gym`'s `_prepare_reward_function` multiplies every scale by the timestep on startup, so a
scale read out of a running environment is 50x smaller than the one in the config.
`task_cfg.REWARD_WEIGHTS` holds the config's numbers and `scaled_weights()` does the multiply in
one place. Applying it twice is a 50x error in every term at once and would look like a learning
rate problem.

## The force-transmission task (`--task hook`)

UniFP's task asks for a few newtons wherever the goal is. This one engages the tool with a fixture — a
claw through a ring or over a bar, or a pad on a button — and asks for tens of newtons into it, far more
than the D1's motors hold in a bent reach (9.4 N median, F-101). Meeting it means lining the arm up with
the pull and letting the legs and body make the force. Same 76/153-wide observations and 18 actions, so
a UniFP checkpoint resumes into it.

| module | what |
| --- | --- |
| `fixture.py` | the contact (unilateral spring along the axis; ring, claw-over-bar or pad across it), the force-level schedule, axis and handle samplers — plain torch |
| `hook_cfg.py` | every constant, with the reason for each and the version that changed it |
| `hook_env.py` | `Go2D1HookEnv`: goal to the handle, engage once settled, hold the goal on the anchor, ramp the force command along the axis; the critic's `mass_params` block carries the fixture |
| `hook_rewards.py` | `fixture_force_tracking` (along-axis miss + ¼ of the sideways load), `arm_torque_margin`, `fixture_lost`, `fixture_seat` |
| `hook_eval.py` | the staircase experiment and its scoring |

```
./run_unifp_train.py smoke --task hook --num_envs 16 --headless                        # zero actions
./run_unifp_train.py train --task hook --num_envs 4096 --iterations 1500 --headless \
    --resume_from <UniFP or hook checkpoint> [--press_fraction 0.35] [--force_ceiling 30]
./run_unifp_train.py hook_eval --task hook --checkpoint <model.pt> --headless \
    [--fixture_kind ring|bar] [--press] [--eval_levels 20 40 60 80 100]
```

The arm runs at 0.01 kg·m² of armature here, not the 2e-4 of UniFP's task: at 2e-4 the wrist's explicit
PD is in a numerical limit cycle and no arm torque means anything (F-102). Evaluations on UniFP's frozen
free-space manifests work with `eval --task hook --fixture_fraction 0` (the environment draws nothing
extra, so the schedules replay) and report the armature as a condition mismatch, correctly.

What it is not: the fixture is an anchored virtual spring — nothing opens, latches or moves — and the arm
runs at its published torque limits while pulling hard, which a real servo may not tolerate (F-103).

## The goal-commanded task (`--task mechanism`)

The pull task commands a force; on a real box nobody knows that number. Here the claw already holds a
handle, the command is only where the handle should go — a reference sliding along the mechanism's
path, as a task layer that knows the geometry from tags would plan it — and the resistance is drawn per
episode and never observed. The force is the policy's to find, and the arm-margin term makes the body
supply what the arm cannot. Same widths again, so a UniFP or pull-task checkpoint resumes into it.

| module | what |
| --- | --- |
| `mechanism.py` | the plant: a slide or a hinge, spring and preload, stick-slip friction, damping, a latch that snaps, stops; the 3-D grasp spring that tears out above its limit; the reference schedule — plain torch |
| `mech_cfg.py` | every constant and the reason for it; the four evaluation mechanisms |
| `mech_env.py` | `Go2D1MechEnv`: goal to the handle, grasp once settled, build a mechanism around the tool and shorten its path until it fits, hold the goal on the path's reference; UniFP's virtual spring made rigid, no pushes |
| `mech_rewards.py` | `mech_progress` (handle against its reference; widths shrink with a short mechanism's travel), `mech_push` (newtons toward the reference × progress lost — what makes it escalate, F-107), `arm_torque_margin` (the pull task's), `mech_torn`, `mech_overspeed` (the lunge after a latch lets go), `mech_effort` |
| `mech_eval.py` | drawer, latch, door and button at 15 placements × 10–80 N (plus held-out `lid` and `bolt` on request); opened, capacity, and how the force was made |

```
./run_unifp_train.py smoke --task mechanism --num_envs 16 --steps 400 --headless        # zero actions
./run_unifp_train.py train --task mechanism --num_envs 4096 --iterations 2500 --headless \
    --resume_from <UniFP, pull or mechanism checkpoint> [--peak_ceiling 30] [--peak_ceiling_max 80]
./run_unifp_train.py mech_eval --task mechanism --checkpoint <model.pt> --headless \
    [--mech_kinds latch button lid bolt] [--mech_levels 10 30 50 70]
```

**Two ways to give it the goal.** Without `--force_law` the policy gets the goal alone and has to find the
force itself (F-105, F-107). With `--force_law` a task-layer PI law turns the handle's lag behind its reference
into a force command (`mech_env.set_force_law`, `mech_cfg.FORCE_LAW_*`) and the goal sits on the handle, as the
pull task trained: in `mech_eval` that is the hierarchical baseline for a force-following policy (F-106); in
`train` the law runs inside the loop and the pull task's force-tracking term comes back
(`mech_cfg.LAW_WEIGHT_CHANGES`). Evaluate a policy the way it was trained — a law-trained policy with
`--force_law`, a goal-only one without. `--force_law_bleed 1.0` lets the law's integral decay once the handle
has arrived (without it the integral holds its force against a stop; F-108's torn buttons), and
`--law_variant v2` adds a price on drive past the command (`mech_overforce`, a cap drawn per episode) and a
heavier price on the lunge after a release; `v3` also drops the outcome reward, a pure force-follower. Neither makes
the cap a hard limit (F-109): v3 keeps closest to it and gives up some capacity for it.

`mech_eval --mech_test` is the held-out test condition (`mech_cfg.TEST_*`: other placements, twice the mass, a
softer grasp, a faster reference, more damping), frozen before any policy was scored on it; the default set is
the development set every reward change was read against.

**Levers** (F-114), all off by default so earlier commands draw what they drew. `--lever_fraction 0.3` makes that share
of mechanisms short levers turned *down* about a level spindle pointing at the robot (`mechanism.lever_axes`,
`mech_cfg.LEVER_*`: 5–12 cm, 40–75°, the spindle turned ±45° for where the robot stands, 3–35 N, never at the
curriculum's frontier). `--lever_high` draws their handles up to ~0.96 m, above the shoulder, where turning a lever
down is a pull. `--offpath_weight 2.0` prices force across the mechanism's path (`mech_offpath`, per (20 N)²): a claw
lets go under it and this task's ball-joint grasp does not (F-115). The lever tests: `mech_eval --mech_kinds lever
lever_spindle_side lever_tip_side` (the combiner's handle square and 30° either side, 5–40 N), and with `--lever_high`
the same at 0.6–0.9 m. `across_p95_n` in every `mech_eval` is the force across the path during the push, a proxy for
what a claw would have to hold; `Mechanism/grasp_error_m` in training is how far from the handle the grasp was made.

**The virtual spring is rigid here** (`mech_cfg.STIFF_KP`). UniFP's target is `goal + force / 200`, which
pays the policy to give way — right for its pushes, wrong for a latch — so a policy trained on this task
is a different controller from UniFP's, not a fine-tuned copy of it. Same 0.01 kg·m² arm armature as the
pull task (F-102).

What it is not: every mechanism number is invented, since there was no hardware to measure; the grasp is
a ball joint closed wherever the tool stopped (no torque through the handle, so no knob or lever turned about the
wrist, and no reach precision or claw geometry: F-115); and the robot stands — nothing here walks while it opens.

## The insertion task (`--task insertion`)

A held key into a keyed slot on a door, from an estimate of where the slot is (2026-09-26; the demo is
`demos/insertion/`). The same hierarchical recipe as the mechanisms: a task layer (`insertion.InsertionTaskLayer`)
drives the insertion and the whole-body policy is trained with it in the loop.

| file | |
| --- | --- |
| `insertion.py` | the slot (a penalty-contact model of a chamfered channel, a panel face, a detent and a seat, the force acting at the key's tip), the task layer (a PI force law along the estimated axis, capped low on the face with a spiral search, higher once in; across the axis the goal yields to the tip after a touch), its outer loops (`PoseCorrection`) and the compliant grip (`CompliantGrip`) -- plain torch, `tests/test_insertion.py` |
| `insert_cfg.py` | where slots are, their sizes, the curriculum on the estimate's error and the detent, the task layer's gains, the rewards, the frozen evaluation |
| `insert_env.py` | `Go2D1InsertEnv`: the goal to the pre-insertion pose with the approach commanded, the slot placed off the estimate once the hand is there, the layer's goal and force each step |
| `insert_rewards.py` | depth against the reference, the jaw centre's precision, seated, force across the axis, contact above a safe force, damage, the tool's speed while the goal is held |
| `insert_eval.py` | the frozen slots (`--ins_set lateral`: three heights × four estimate errors × four directions × two blade rolls; `detent`: four detents) and their scoring |

```bash
./run_unifp_train.py smoke --task insertion --num_envs 64 --steps 600 --headless
./run_unifp_train.py train --task insertion --num_envs 4096 --iterations 5000 --headless --tool_point jaw_centre \
    --resume_from <model.pt> --ins_compliant_grip --resume_arm_std 0.15 --entropy_coef 0.002     # v7, 2026-09-27
./run_unifp_train.py ins_eval --task insertion --headless --checkpoint <model.pt> [--ins_set detent] [--ins_compliant_grip]
./run_unifp_train.py play --task insertion --num_envs 32 --steps 900 --headless --checkpoint <model.pt> --ins_debug 0
```

**The approach objective.** A key goes in only along the slot's axis, so the hand's approach is commanded: the
elevation in `CMD_EE_ORN_Y` (1 + elevation when on, 0 when off), the azimuth the goal's own bearing, which is the
slot's axis when the robot stands on it. It is on only in insertion episodes, whose slots sit where a level approach
is reachable (0.65-0.95 m up): over the whole goal sphere a commanded orientation is mostly unreachable (F-099).

**What made it trainable** (2026-09-27, after five runs that did not learn it, F-116):

- **Outer loops** (`insertion.PoseCorrection`, on by default; `--ins_tip_gain`, `--ins_attitude_gain`, 0 = off). The
  policy never observes its own hand (F-118), so it arrives steadily off. The task layer integrates the measured
  miss of the key's tip, the approach's elevation and the blade's roll into the commands; with them on, the layer
  waits for the tip to converge (3 mm) before it starts, and starts anyway at the deadline if within 3 cm.
- **A compliant grip** (`insertion.CompliantGrip`, `--ins_compliant_grip`): the key tilts about the jaw centre on a
  2 N·m/rad spring, up to 10°, settled against the contact each physics step before it is committed (`Slot.probe`).
  A rigid key off the axis or in a jittering hand bangs the walls (160 N at a 5 mm jitter against 22 N compliant).
  It is a hardware requirement this sets, not a free choice: the key has to be held so it can tilt.
- **Less arm noise in training** (`--resume_arm_std`). Checkpoints of the pull lineage carry an action-noise std of
  0.7-0.9 on the arm, ~0.2 rad of random target per step: nothing inserts under it, so nothing is learned.
- **Holding still** (`ins_tool_speed`): every starting policy jitters its arm at ~10 Hz by a few millimetres while
  holding a pose (F-104 saw the habit on a pad). And an **easier slot** below curriculum level 1 (up to 2.5 mm more
  clearance and 3 mm more chamfer), so the first episodes can seat.

What it is not: the slot is a model -- a penalty contact, one friction coefficient, a key rigid or sprung in a rigid
hand -- and the demo inserts the same key in PhysX to check it; the estimate's error is drawn, not measured.

## The pressing task (`--task press`)

The plan's force-calibration task (2026-09-29): a spring-loaded push button on a panel facing the robot, pressed with
the bare jaws or a probe, from an estimate of where the button is and a datasheet of its force. The same hierarchical
recipe as the key: a task layer (`pressing.PressTaskLayer`) drives the press, and the whole-body policy is trained with
it in the loop.

| file | |
| --- | --- |
| `pressing.py` | the tools (`TOOLS`: the closed jaws' end faces as a 10 mm sphere 20 mm past the jaw centre; a 6 mm ball 70 mm out), the button (a penalty contact on the tool's sphere: a cap on preload, spring, damping and a click, stopping at its travel; the panel beside it; the rim of its hole, which keeps a wide tip out of a small recessed cap), and the task layer (the tip to a standoff, a PI force law along the estimated axis capped light until something is touched, a steady hold at full travel, a release) -- plain torch, `tests/test_pressing.py` |
| `press_cfg.py` | where buttons are, their sizes and forces, the curriculum on the estimate's error and the click, the task layer's margins, the rewards, the frozen evaluation |
| `press_env.py` | `Go2D1PressEnv`: the goal to the pre-press pose with the approach commanded, the button placed off the estimate once the hand is there, the layer's goal and force each step, the insertion's outer loops on the tool's tip and the approach's elevation |
| `press_rewards.py` | depth against the reference, the tip's precision, pressed, force across the axis, contact above a safe force, damage, the tool's speed while its goal is held |
| `press_eval.py` | the frozen buttons (`--press_set panel`: three heights x four estimate errors x four directions x two tools; `force`: 5-30 N full-travel force; `recessed`: a small cap 2 mm below the panel) and their scoring |

```bash
./run_unifp_train.py smoke --task press --num_envs 64 --steps 600 --headless
./run_unifp_train.py train --task press --num_envs 4096 --iterations 400 --headless --tool_point jaw_centre \
    --resume_from checkpoints/unifp_go2d1_insertion_seed3_model_21200.pt --resume_arm_std 0.15 --entropy_coef 0.002 \
    --fixed_learning_rate 1e-5                                   # the pilot of 2026-09-29
./run_unifp_train.py press_eval --task press --headless --checkpoint <model.pt> [--press_set force|recessed]
```

**Success** is the plan's: the cap held at its full travel (within 0.5 mm) for 1 s without a break, no damage, no fall.
Every episode also reports the force error (applied along the axis against the layer's command, RMS while it pushes),
the physics-rate peak force, the time to full travel, and one failure reason: fell, damaged, not started, missed, slid
off, short, not held (`press_eval`).

**Where it stands (2026-09-29, F-123).** Before any training on it, the key's and the combiner's policies hold 27 and
29 of 96 frozen `panel` buttons, sliding off the cap. A 400-iteration pilot from the key's policy (the command above)
holds 49/96 `panel`, 20/48 `force` (none at 20-30 N) and 23/72 `recessed` (the probe; the bare jaws 0, as declared).
It is a pilot, one seed, not a deliverable.

**Resume at a pinned learning rate.** From another task's checkpoint, rsl-rl's adaptive rate restarts at 1e-3 and
the first update replaces the policy (KL 197 here, 243-400 for the insertion recipe; Week 2 log, v8). An adaptive-rate
pilot from the insertion checkpoint never got a hand to its pre-press pose for 300 iterations and ended at 0/96. At
`--fixed_learning_rate 1e-5` the first update's KL is 0.015 and the policy keeps what it had (Week 3 log, 2026-09-29).

**Reading the training log.** For the first ~40 iterations the only episodes that end are the ones rsl-rl started
part-way through (`init_at_random_ep_len`). They end before their button engages, so `Press/missed_fraction` reads 1.0
and `Press/success_fraction` is absent. That is not a defect. `Press/success_fraction` (a continuous 1 s hold) also
stays near 0 in training while `Press/ever_pressed_fraction` climbs: the exploration noise makes the cap flicker at its
stop. The curriculum promotes on the former, so as built it does not promote; the frozen evaluation (deterministic) is
the measure.

**What the layer knows.** The estimate of the cap's face and axis, and a datasheet: the travel exactly and the forces
within a rating error. It sizes its press cap (1.6-2.4x the operating force, at most 60 N) and its hold (1.3-1.6x the
full-travel force) from them. It finds the bottom two ways: at the estimated full travel, or stalled at the full cap
(a button nearer than estimated). It never learns the button's stiffness, only where its tip is.

**The recessed set is an access test, declared in advance.** A 10 mm tip cannot follow an 8 mm cap 2 mm into its hole
(`pressing.reachable_front`), so the bare jaws are expected to fail every episode of it, and the probe is what makes
the button pressable. That is the plan's "bare-finger inability is an access limitation", measured rather than argued.

What it is not: the button, the panel and the tools are models. There is one penalty stiffness and one friction
coefficient, the click is a ramp, and the tools are spheres, where the real closed jaws are two flat 12 mm fingers.
Nothing is PhysX (a PhysX check, as `demos/insertion` is for the key, has not been built). The panel is always
vertical-ish: a button pressed down onto a top face needs an approach pointing down, which the D1 mostly cannot command
at this reach (F-099).

## The wiping task (`--task wipe`)

The plan's wiping task (2026-09-29): a pad held in the jaws, wiped along a slow path on a panel facing the robot at a
set force, from an estimate of where the panel is.

| file | |
| --- | --- |
| `wiping.py` | the pad (a rigid 50 mm disc 45 mm past the jaw centre, meeting the surface at its centre and eight rim points, so a tilted pad rides on its rim and the force acts at the centre of pressure), the surface (a penalty spring along its normal, damping, Coulomb friction per point, an edge), the paths (a straight stroke, a zigzag of strokes, resampled evenly by arc length), coverage, and the task layer (a PI force along the estimated normal capped at the wiping force; once the pad has stalled on the surface, a reference point along the path never more than 2 cm ahead of the pad's progress, and a PI force across the normal to push through friction; a lift at the end) -- plain torch, `tests/test_wiping.py` |
| `wipe_cfg.py` | where panels and paths are, forces and speeds, the curriculum on the estimate's offset and tilt, the rewards, the frozen evaluation |
| `wipe_env.py` | `Go2D1WipeEnv`: the goal to the pre-contact pose with the approach commanded into the panel, the true panel placed off the estimate once the hand is there, the layer's goal and force each step, coverage and contact bookkeeping |
| `wipe_rewards.py` | the pad's precision (to its goal, then in the plane), in contact while wiping, force tracking along the normal, overforce, damage, the pad's velocity error against the path's |
| `wipe_eval.py` | the frozen surfaces (three heights x a 16 cm stroke or a 3 x 12 cm zigzag x the surface 8 mm nearer, where estimated or 8 mm further x 5 N or 10 N, tilted 3 degrees from the estimate) and their scoring |

```bash
./run_unifp_train.py smoke --task wipe --num_envs 64 --steps 900 --headless
./run_unifp_train.py train --task wipe --num_envs 4096 --iterations 400 --headless --tool_point jaw_centre \
    --resume_from checkpoints/unifp_go2d1_insertion_seed3_model_21200.pt --resume_arm_std 0.15 --entropy_coef 0.002 \
    --fixed_learning_rate 1e-5                                   # the pilot of 2026-09-29
./run_unifp_train.py wipe_eval --task wipe --headless --checkpoint <model.pt>
```

**Success** is the plan's: at least 90% of the path wiped and contact during at least 80% of the motion, no damage,
no fall. A path sample counts as wiped when the pad's centre of pressure passes within 2 cm of it while pressing with at
least 1 N (`wiping.WIPE_RADIUS_M`, `CONTACT_MIN_N`). "The motion" runs from when the layer found the surface to the
path's end. Every episode also reports the normal force's RMS error against the command and its physics-rate peak,
the friction carried, contact-loss events (losses under 0.1 s do not count), the time to finish, and one failure
reason: fell, damaged, not started, no contact, incomplete, contact lost (`wipe_eval`).

**Where it stands (2026-09-29, F-124).** The key's policy, untrained for wiping, keeps contact for the whole motion
and wipes 90% of the path in 14/36 frozen episodes. The pad is tilted ~8° to the panel and presses with its rim. A
400-iteration pilot scores 16/36, with the tilt barely changed. It is a pilot, one seed.

**Why the layer has a force along the path too.** UniFP's controller yields to the force it measures: its target is
`goal + (measured + commanded) / 200 N/m`. A 4 N friction on a 10 N wipe puts the target 2 cm behind the goal. With
the reference kept within 2 cm of the pad, the pad would then stop. The in-plane PI turns the pad's miss into the force
that pushes through the friction, as the normal law turns a depth lag into the press.

What it is not: the pad is rigid and flat (the plan's "rigid pad initially"). Tilted 4 degrees to the surface it wipes
with its rim, and with an ideal hand that still covers 97-100% of a 16 cm stroke (`tests/test_wiping.py`). A sponge,
or a pad on a gimbal as the key has its compliant grip, is the next tool. The surface is a model, and the panel faces
the robot: wiping a table top needs the hand pointing down (F-099).

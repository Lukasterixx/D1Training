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

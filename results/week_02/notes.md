# Week 2 — 21–27 September 2026

**Status:** in progress
**Focus:** Select reusable methods; validate P0 candidate; freeze task/interface definitions and calibrate camera/tool frames

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Finish bounded UniFP and position-only reproduction attempts; select port versus retarget; validate candidate reaching and define slow pose trajectories plus pressing/box fixtures.
- **Hardware and camera:** Calibrate camera-to-base, tag-to-task and tool transforms; record real observations and replay them through robot-side inference with actuation disabled.
- **Evidence and deliverable:** Freeze sequence phases, box opening extent, tolerances, force bounds, comparison applicability, tool splits and evaluation manifests before substantive runs; calibrate contact definitions by Week 3.
- **Gate focus:** G0; candidate G1a; G4 begins. Implementation and scope decision.

## Checklist

- [ ] Reproduction levels and implementation choice recorded; legacy setup kept within its time budget
- [ ] Revised target workspace and zero-action baseline evaluated on frozen cases
- [ ] Candidate P0 evaluated with 100 frozen episodes; reaching claims distinguished from PPO diagnostics
- [ ] Task-command schema frozen: pose/force frames, units, timestamps, interpolation and tool registration
- [ ] Actual box sequence, loads, opening measurement, phase timeouts and tolerances documented
- [ ] Core pressing/box tasks and tool conditions selected; applicability and held-out cases frozen
- [ ] Camera/tag/tool calibration and recorded-state inference replay completed or blockers recorded
- [ ] Physical trial manifests and training budgets defined from pilot costs

## Log

### 2026-09-23 — Chasing an observation from the viewer: the low-front corner of the workspace

Lukas, watching both policies, noticed the arm flops and fails to track on trajectories going low
and in front. The frozen manifest cannot answer that — one median per episode, and an episode
sweeps the whole workspace — so `unifp_train/workspace_map.py` holds the goal still instead: 243
grid points over three radii x nine pitches x nine yaws, one environment each, steady state read
over the last 200 of 600 steps, standing, forces off.

**The observation is right.** Pitch is elevation, so the bottom of the range is low and in front;
at radius 0.45, pitch −45 degrees, the goal is 32 cm forward and 17 cm off the ground.

| | native `model_56000` | ported `model_48800` |
| --- | --- | --- |
| median over 243 goals | 1.5 cm | 3.2 cm |
| lowest pitch band | **3.9 cm** | **4.5 cm** |
| everything above it | 1.4 cm | 3.1 cm |
| worst single goal | 6.4 cm (pitch −45, yaw 0) | 7.7 cm |

**The two failures are not the same failure**, and the spread over time separates them. Median
standard deviation of the error: **native 0.4 cm, ported 1.8 cm** — and at the ported policy's
worst goal, 5.1 cm of swing on a 7.7 cm error. The ported policy really does flop, across most of
the workspace rather than only low down. The native one is steady to half a centimetre everywhere,
including where it is least accurate; its low-goal failure is a **systematic under-reach**, the tip
sitting 2.0 cm forward and 2.1 cm above the goal.

**Two explanations tested and rejected.** It is not a joint limit — arm-joint saturation at the
worst goals is 0.52–0.80 of travel. And it is not under-exposure in training: the goal generator
does command the lowest pitch band less than uniform, 6.6% against 11.1%, because goals are
interpolated between random endpoints and so spend less time at the extremes — but the **highest**
band gets 6.6% as well and tracks at 1.5 cm. The exposure deficit is symmetric and the accuracy
deficit is not.

What does correlate is geometry: error against goal height r = −0.49, against distance from the
task's own keep-out box r = −0.37. Goals within 10 cm of that box average 3.9 cm; beyond 35 cm,
1.3 cm. The worst shell is immediately outside the volume the task declares off-limits, reached by
working down and forward past the front legs.

**What this does not measure.** The probe holds the goal still, so it says nothing about tracking
*lag* on a moving goal — which is what is actually on screen during a sweep, and may be part of
what looked like flopping on the native policy. Standing only; low goals while walking are not
measured. One checkpoint per policy.

[F-082](../findings.md) records it. The practical reading: the deliverable policy has a bounded
weak region — roughly three times the error in the bottom tenth of the workspace, steady, with no
instability anywhere — which is a thing to site tasks around rather than a defect to chase.

### 2026-09-23 — Evaluated: the natively trained policy wins, 1.5 cm against 3.9

The run finished all 60,000 iterations in 43.5 h, `status: complete`, 301 checkpoints. Ten
frozen-manifest evaluations against the same 50-episode set F-079 used, **zero schedule or
condition mismatches** in any of them, so every comparison below is paired episode for episode.

| policy | tracking (median) | p90 | falls | base vel err | train return |
| --- | --- | --- | --- | --- | --- |
| zero actions | 38.1 cm | 44.8 | 0/50 | 0.259 | — |
| Isaac Gym `model_48800`, ported | 3.9 cm | 5.1 | 0/50 | 0.067 | ~154 |
| Isaac Lab 8,000 | 1.5 cm | 2.0 | **5/50** | 0.047 | ~173 |
| Isaac Lab 16,000 | 2.4 cm | 3.0 | 0/50 | 0.049 | ~162 |
| Isaac Lab 24,000 | 2.2 cm | 2.6 | 0/50 | 0.043 | ~164 |
| Isaac Lab 32,000 | 1.6 cm | 2.0 | 0/50 | 0.042 | ~166 |
| Isaac Lab 40,000 | 1.7 cm | 2.5 | 0/50 | 0.039 | ~168 |
| Isaac Lab 48,000 | 1.8 cm | 2.4 | 0/50 | 0.037 | ~168 |
| **Isaac Lab 56,000** | **1.5 cm** | 2.2 | **0/50** | 0.035 | ~168 |
| Isaac Lab 59,999 | 1.7 cm | 2.1 | 0/50 | 0.039 | ~169 |

Per-episode, against the ported policy: **50 of 50 episodes better** for every checkpoint but
40,000 (49 of 50), median paired gain +1.5 to +2.6 cm, sign test p ≈ 2e-15. The zero-action
baseline reproduces F-079's 38.1 cm to the millimetre, which is the check that the set has not
moved under us.

**This reverses F-079.** That finding concluded porting beat native training, 3.9 cm against
6.2 cm. It was measuring a run made with the F-080 defect and stopped at 44%. Fix the defect, let
it finish, and the same task on the same manifest in the same simulator gives 1.5 cm. F-078 built
the native path on the argument that a policy should be trained on the stack it will be evaluated
on; that argument pays.

**F-079's other finding survives intact, and rather pointedly.** The training return still cannot
select a checkpoint: the **highest**-return checkpoint in the ladder is iteration 8,000 at ~173,
and it is the only one that falls — five times in fifty. Return moves over 162–173 while falls go
5 → 0 and tracking 2.4 → 1.5 cm, in no useful relation. What has changed is the cost of choosing
badly: this run contains no catastrophic checkpoints, where the previous one held a 60.8 cm
policy at a perfectly ordinary return.

**What this does not say.** One run, one seed, one 50-episode set, one simulator. No interval is
attached to any median, so 1.5 against 1.6 against 1.7 inside the plateau is not an ordering — the
paired test supports the gap to the *ported* policy, not the ranking within. And this manifest is
Isaac Lab's; it is not comparable episode-for-episode with F-075's 2.6 cm on the Isaac Gym side.

[F-081](../findings.md) records it; F-079 is marked superseded.

### 2026-09-23 — The full run at 80%: still no collapse, and a KL artefact that is not one

48,155 of 60,000, about 8h 50m left, finishing today around 18:50.

| iterations | `tracking_ee_force_world` | | | mean return | | |
| --- | --- | --- | --- | --- | --- | --- |
| | p0 (defect) | Isaac Gym | **this run** | p0 | Isaac Gym | **this run** |
| 8,000–12,000 | 0.925 | 1.600 | 1.598 | 125.6 | 151.6 | **154.8** |
| 12,000–20,000 | 0.887 | 1.608 | **1.673** | 92.1 | 152.6 | **160.9** |
| 20,000–30,000 | 1.189 | 1.643 | **1.706** | 129.3 | 154.2 | **163.7** |
| 30,000–40,000 | — | 1.663 | **1.750** | — | 156.0 | **167.2** |
| 40,000–48,100 | — | 1.673 | **1.764** | — | 156.9 | **167.8** |

**Zero collapse episodes** by the rule that found three in p0 (smoothed term below 80% of its
running maximum for more than 150 iterations). The term has risen in every window since the
curriculum opened and sits at its highest, above the Isaac Gym reference throughout. Episode
length 999 of 1000.

**The KL spikes.** From iteration 45,269 the run has produced 37 iterations with a mean KL above
0.1, peaking at 13 — against a target of 0.01 and a run median of 0.0133. They are not scattered:
they come in two bursts of exactly 18 consecutive iterations (45,269–45,286 and 47,826–47,843)
plus one isolated at 45,617. Reporting the *mean* KL over a window containing one makes it look
like the run is diverging; the median does not move at all.

Nothing behavioural moves with them:

| | at a spike | 20 iterations either side |
| --- | --- | --- |
| σ | 0.53974 | 0.53940 |
| episode length | 998.72 | 998.76 |
| `tracking_ee_force_world` | 1.7714 | 1.7681 |
| `kl_first_minibatch` | 5.2e-10 | 2.2e-10 |
| value loss | 0.0398 | 0.0276 |
| learning rate | 1e-05 (floor) | 2.07e-05 |

The tracking term averages 1.7643 before the first spike and 1.7648 since. So the schedule sees a
huge KL, cuts to the floor for the duration, and the policy comes out the other side unchanged.

**It is not F-080 returning.** `kl_first_minibatch` stays at ~1e-10 through the bursts — the first
mini-batch of each update still sees exactly the policy that acted, so the KL is produced by real
gradient steps within the update and not by a broken pairing. (It is no longer *exactly* 0 as it
was for the first 44,000 iterations; 1.6e-09 is eight orders below the defect's 0.063 and is
floating-point noise.)

**Has it stopped improving?** On the training term, yes — at about iteration 40,000. Gains per
2,000-iteration window ran +0.0087, +0.0095, +0.0057, +0.0074 up to 40,000 and then −0.0006,
−0.0018, +0.0019, +0.0020, −0.0006: flat, oscillating about 1.765. A linear fit over the last
10,000 iterations gives +0.00035 per 1,000, so the remaining 9,185 iterations project to +0.0032,
**+0.18%**.

The Isaac Gym run did no better over the same stretch: it **lost** 1.74% over its last 12,000
iterations (1.674 → 1.646), dipping to 1.557 at 54,000–57,000. That is independent corroboration
of F-072's finding that most of that run's 39.85 h bought nothing.

**This is not a reason to stop early, and the reason why is F-079.** A plateau in the training term
says nothing about the evaluated policy: F-079 measured the return holding at 130–137 while frozen
tracking swung from 6.2 cm to 60.8 cm on the same run. Stopping on the strength of a flat training
curve would be reasoning from precisely the signal that was shown not to carry this information.
Checkpoints are written every 200 iterations, so finishing costs only time and forecloses nothing;
what it buys is the removal of F-079's largest scope caveat, that it compared a policy stopped at
44% against a complete one.

**Untested hypothesis** for the record: a burst of 18 iterations is 432 environment-steps, and an
episode is 1,000. One environment stuck in a pathological state — observations at the ±100 clip,
where the network is most sensitive — would dominate the batch mean until its episode times out,
which fits the duration, the contiguity and the complete absence of a behavioural signature. Not
checked, and not worth interrupting the run for. If it matters later it can be tested on a
checkpoint by looking at the per-sample KL distribution rather than its mean.

### 2026-09-22 — The full run at 25%: no collapse, and ahead of Isaac Gym

15,116 of 60,000 iterations, 11 hours in, ETA Wednesday 23 September about 18:50. The force
curriculum opened on schedule at 8,000.

| iterations | `tracking_ee_force_world` | | | mean return | | |
| --- | --- | --- | --- | --- | --- | --- |
| | p0 (defect) | Isaac Gym | **this run** | p0 | Isaac Gym | **this run** |
| 0–1,000 | 1.037 | 1.292 | **1.492** | 114.2 | 120.3 | **145.6** |
| 1,000–4,000 | 1.518 | 1.632 | **1.801** | 151.7 | 146.6 | **170.2** |
| 4,000–8,000 | 0.932 | 1.690 | **1.841** | 139.0 | 153.1 | **172.9** |
| 8,000–12,000 | 0.925 | 1.600 | 1.598 | 125.6 | 151.6 | **154.8** |
| 12,000–15,100 | 0.975 | 1.604 | **1.660** | 111.9 | 152.6 | **160.0** |

**p0 had collapsed by iteration 5,399 and never recovered**; this run passed that point without
one and is above the Isaac Gym reference in every window. Since the curriculum opened the term has
risen monotonically — 1.563, 1.582, 1.606, 1.640, 1.642, 1.666, 1.670 over 1,000-iteration windows
— against Gym's flat 1.60. The dip at 8,000 is the forces arriving and Isaac Gym shows the same
one.

Health, over all 15,127 iterations so far:

- `Loss/kl_first_minibatch` **maximum 0**. The F-080 standing check has not wobbled once.
- Learning rate steady at 5.06e-05 against upstream's 7.59e-05 median; **0% of iterations at the
  1e-5 floor since iteration 9,000** (p0: 100% from iteration 200).
- Mean KL 0.0124–0.0131 across every window, no drift (−1.5% since the curriculum opened). The
  300-iteration drift screen read +4.3%, reproducing the probe exactly.

**None of this is an evaluation.** These are training-time terms sampled at reset, and F-079
established that on this task the training return cannot distinguish a 6.2 cm policy from a
60.8 cm one. Whether this run beats the ported policy's 3.9 cm is not addressed by anything above
and needs the frozen manifest. The GPU is busy with the run itself, so no evaluation has been made.

### 2026-09-21 — A full run, on UniFP's own configuration, with the pairing fixed

```
./run_unifp_train.py train --num_envs 4096 --iterations 60000 --seed 1 --headless \
    --run_name full_after_f080
```

No departures: 4,096 environments, seed 1, the force curriculum opening at iteration 8,000, 24
steps per environment per iteration — 5.9 billion environment-steps, the same budget the Isaac Gym
run of F-072 had. About 43 hours at the measured 2.59 s/iteration, against that run's 39.85 h.
Checkpoints every 200 iterations, roughly 10.5 GB.

It is deliberately the faithful configuration and not probe B's 1,024-environment variant. The
argument for the smaller batch came from the defect-era measurements retired above, and running
UniFP's own numbers means the comparison with F-072 and with the stopped p0 run carries a
simulator difference and nothing else.

**First 67 iterations**, against the p0 run at the same point:

| | p0 (defect present) | this run |
| --- | --- | --- |
| `kl_first_minibatch` | not logged; 0.063 measured later at `model_4000` | **0, exactly** |
| mean KL | — | 0.0139 |
| learning rate, median | reached the floor at iteration 130 and stayed | **4.44e-04** |
| iterations at the 1e-5 floor | 100% from iteration 200 onward | **0%** |
| mini-batches above 0.02 / below 0.005 | 42% / 3% | 8.1% / 12.8% |

The controller is correcting in both directions rather than only downward, which is the thing that
was never true of any run before today.

**What it is for.** F-079 compared a 26,466-iteration Isaac Lab policy against a complete Isaac Gym
one and found porting ahead, 3.9 cm against 6.2 cm. Both halves of that comparison were
handicapped — one by being unfinished, both by the defect. This run answers the question the
honest way, and it will be judged the same way F-079 judged the others: by frozen-manifest
evaluation of several checkpoints, never by training return, which F-079 showed cannot tell a
6.2 cm policy from a 60.8 cm one.

**Watch `Loss/kl_first_minibatch`.** It should stay at 0 for the whole run. If it leaves 0 the
rollout/update pairing has broken again and the run is not worth finishing; `--diagnose_storage`
says which samples disagree.

### 2026-09-21 — The collapse has a cause: resetting an environment blanked an observation the policy had already acted on

Everything below this entry treats the learning-rate schedule's saturation as a symptom of a
stiff network. It was not. Most of the KL the schedule was regulating was never produced by
learning, and the schedule was doing exactly what it should with a corrupted input.

**Start from the two long runs side by side**, which no earlier entry had done — both still have
their full TensorBoard traces, so it cost no GPU at all:

| | Isaac Lab (p0, 26,468 iters) | Isaac Gym (60,000 iters) |
| --- | --- | --- |
| iterations with the rate at the 1e-5 floor, 200–1,000 | **100.0%** | **0%** |
| … 1,000–10,000 | 100.0% | 1.3% |
| … 10,000–26,400 | 100.0% | 38.5% |
| median rate, 1,000–10,000 | 1e-05 | 7.59e-05 |

The rate reaches the floor at iteration 130 and never leaves for the remaining 26,268 iterations.
Upstream's never touches it until late. And across the twelve probe runs, 42% of the faithful
config's mini-batches were **above** the schedule's upper threshold against 3% below, so it was
asking to go lower and could not: saturated, not merely sitting low. That distinction had not
been drawn and it is the one that matters.

**The estimator-loss gap is a red herring** and the record already warned about it: Lab logs
13–63x upstream's, but `algorithm.py` documents that upstream divides its logged value by
`adaptation_batch_size` (64). Corrected, Lab's is 0.2–1.0x upstream's. Noted here so it is not
chased again.

**Three measurements, each cheap, that turned the question over.**

*The quadratic law holds in-run.* With the adaptive schedule neutralised — a new
`--fixed_learning_rate`, which holds the rate by refusing the schedule's writes rather than using
rsl-rl's `schedule="fixed"`, because that skips the KL computation entirely — the KL at iteration
0, where every condition has identical weights and an identical rollout, scales as the square of
the rate: a tenfold cut gives **0.0098** of the KL against 0.01 predicted. So the earlier "a
tenfold cut is worth only 27%" was not a property of the optimiser. It was the schedule: **mean
KL under an adaptive controller is a regulated quantity, the setpoint, not a measurement of the
policy.** Every probe-to-probe comparison of mean KL in the entries below was comparing a
controlled variable, which is exactly why nothing moved it.

*The null test.* Set both optimisers to zero — `--fixed_learning_rate 0 --estimator_learning_rate 0`
— so no parameter can move and any KL reported is spurious by construction. At a fresh
initialisation: 1.4e-05, near enough nothing. Resumed from `model_4000`: **0.0666**. From
`model_10000`: **1.042**. Against a target of 0.01, with the optimiser switched off.

*Which mini-batch.* The first mini-batch of the first epoch has taken zero gradient steps since
the rollout, so its KL is the numerical floor and nothing else. It is now logged on every run as
`Loss/kl_first_minibatch`. It read **0.063** at `model_4000` with everything frozen.

**The defect.** `ObsHistory.append` returns `self.buffer.reshape(...)` — a view of the ring
buffer — and `ObsHistory.reset` zeroed that buffer **in place**. `DirectRLEnv.step()` resets
terminated environments *after* the policy has acted on the observation and *before* rsl-rl
copies the transition into storage, so an environment that ended had its already-used observation
retroactively replaced by 2432 zeros. The update then evaluated the policy on zeros and compared
it against the distribution the rollout had produced from the real state.

`--diagnose_storage` recomputes the action mean from every stored observation and compares it
with the mean stored beside it. Before the fix, at 256 environments, over three iterations:

| | iter 1 | iter 2 | iter 3 |
| --- | --- | --- | --- |
| samples disagreeing | 3 | 6 | 7 |
| samples whose episode ended | 3 | 6 | 7 |
| stored observation all zero | 3 | 6 | 7 |
| disagree **not** done / done **not** disagree | 0 / 0 | 0 / 0 | 0 / 0 |

Exact set equality, three times. Offsets of ±1 step are far worse than offset 0, so it was never
an off-by-one. After the fix, at 4,096 environments: **0 of 98,304 samples disagree**, with ~100
episodes still ending per iteration.

**The fix** is two lines in `ObsHistory.reset`: rebind rather than write in place, so a reset
cannot reach back into a stack already handed out. Upstream is immune because it rebuilds its
stack with `torch.stack` every step, which allocates. Two regression tests cover it, and both
fail against the old code.

**Probe D re-run with nothing else changed** — 4,096 environments, seed 1, 300 iterations:

| | before | after |
| --- | --- | --- |
| mean KL | 0.0226 | 0.0142 |
| `kl_first_minibatch` | — | **0** |
| mini-batches above 0.02 | 42% | **4.9%** |
| mini-batches below 0.005 | 2.8% | 9.8% |
| learning rate, median | 1e-05 (floor) | **1.98e-04** |
| iterations at the floor | 57% | **0%** |
| KL drift over 300 iterations | +180% | **+4.3%** |
| `tracking_ee_force_world`, last 50 | 0.618 | **1.465** |
| return, last 50 | 92.6 | **141.0** |
| value loss, last 50 | 0.128 | 0.036 |

The rate now sits where upstream's sits and the controller moves in both directions. On the
end-effector term this beats every one of the nine probe conditions, including probe B's 1.235 —
which was measured on seed 7, the one seed in five that did not degrade; this is seed 1, the
worst of them.

**The five-seed scan re-taken**, seed for seed, same configuration, screened with this
repository's own `kl_drift.py` — the instrument that produced the original column:

| seed | KL drift, before | after | `tracking_ee_force_world`, before | after |
| --- | --- | --- | --- | --- |
| 7 | +10% | **+8%** | +6.9% | **+24.4%** |
| 42 | +37% | **+5%** | −20.9% | **+17.4%** |
| 3 | +51% | **+5%** | −21.3% | **+26.9%** |
| 1 | +74% | **+12%** | −19.9% | **+21.0%** |
| 2 | +90% | **+10%** | −30.5% | **+13.3%** |
| probe D, 4096 envs | +180% | **+4%** | −41.4% | **+38.1%** |

**Four of five seeds degraded; none does now, and every one gains.** Median drift +51% → +8%,
median end-effector change −20.9% → +21.0%. Six of six configurations run since the fix sit in the
green band of the screen; six of seven before it did not. Every one reports `kl_first_minibatch`
of exactly 0.

**What this is not.** It is **not** a demonstration that the collapse is cured. The first collapse
in the long run appeared at iteration 5,399 and nothing has been run past 300 iterations since the
fix, so what six of six seeds show is entry into the regime that preceded health rather than the
one that preceded collapse — the necessary condition, not the result. A small residual KL (~5e-4, decaying) remains in the frozen null test in mini-batches
after the first and is unexplained — it is twenty times below the schedule's target and the direct
storage check is exactly zero, so it is not stale observations, but it is not nothing either.

**What it retires.** The factor of two-to-five-point-five between the stacks' policy movement per
unit learning rate was read as a property of the trained weights, and five mechanisms were killed
against it. It was mostly not learning at all. The five-seed scan, the nine probes and the whole
2x2 were all measured under the defect, at a learning rate held ten to twenty times below what
the controller would have chosen, on gradients computed partly from blanked observations.
**"Degradation is the rule" was measured on a broken optimiser loop**, and re-taking it above
reverses it on every seed.
Every number in the entries below stands as a record of what those runs did; none of them should
now be read as evidence about what this task trains like in Isaac Lab.

Also worth keeping: across the collapse, **every** reward term degrades and none improves, so the
policy was never trading the objective against a penalty — it was getting worse at everything at
once, within ~300 iterations across all 27 terms. And the first collapse is at iteration 5,399,
not the 12,000 assumed below.

[F-080](../findings.md) records the result.

### 2026-09-21 — Visual check of the ported policy

`./run_unifp_train.py play --num_envs 4 --steps 30000 --seed 42 --force_start_iteration 0`
with the viewer on, so Lukas could watch `model_48800` drive the welded Go2+D1 directly. Run
through the **training** environment rather than `run_unifp_isaaclab.py play`: the usual playback
path is the harness flagged above, which would have shown a robot on its side within half a second
of a walk command rather than the policy.

`run_unifp_train.py` gained a follow-camera for non-headless runs — `origin_type="asset_root"` on
the robot, offset over its right shoulder — because a fixed world camera loses a walking quadruped
in seconds.

No measurements taken and no run directory written: the session was stopped from outside partway
through, and `play` mode only wrote its `run.json` after the loop. That gap is now closed — both
`play` and `smoke` write the record before stepping and update it after, so an interrupted run
still leaves a note. Nothing here is evidence; it is a look.

### 2026-09-21 — Running down the F-077 contradiction: the policy walks, the playback harness does not

F-079 and F-077 could not both be right, so the difference was bisected rather than left as a
caveat. Both results reproduce exactly — `rollout.py` at a sustained 0.5 m/s gives 0.75558 m and
161 fall-steps, identical to the digit to the run recorded in week 1 — so neither is noise.

| condition | harness | tool-tip error | base height | falls |
| --- | --- | --- | --- | --- |
| 1 env, 0.5 m/s held, no force | `unifp_train` env | **4.1 cm** | 0.302 m | **0** |
| 1 env, 0.5 m/s held, no force | `unifp_isaaclab/rollout.py` | 0.755 m (L1) | 0.154 m | rolls over at step 21 |

Eliminated along the way: the **command protocol** (a held 0.5 and 0.6 m/s both walk fine in the
training env, so it is not that F-077 used a sustained command), the **environment count** (one
environment walks as well as fifty), **external forces** (off in both), and the **ground**, sim
timestep, solver iterations, armature, USD and spawn height, which are shared code.

**Three candidate causes tested, all three wrong, and each made the harness worse.**

- *Foot friction.* Isaac Lab's default `RigidBodyMaterialCfg` is 0.5, and the robot's colliders
  inherit it; the ground carries UniFP's 1.0 from `ground_cfg()` but the feet do not. That is a
  genuine latent discrepancy — and setting it to 1.0 left the walking error unchanged at 0.755 m.
- *Gait-clock ordering.* `rollout.py` advances the clock and the goal **before** the observation;
  UniFP's `post_physics_step` advances them after, so the playback's phase leads the state by one
  policy step. Correcting it degraded **standing** from 0.140 m to 0.417 m.
- *The same with the goal initialised* before the first observation, since advancing at the end
  otherwise leaves a zero goal in the first frame: still 0.417 m.

All three were reverted and the original numbers reproduce to the digit. That every perturbation
breaks this harness is the most informative thing found: it holds the policy on a stability
knife-edge, which is exactly what F-077 itself reported when 8/4 solver iterations flipped its
standing result from success to total failure. The training environment is not in that regime.

**Which side to believe is not symmetric**, and that is what settles it rather than the bisection.
The training environment is verified against the Isaac Gym original at every layer — observations
to 6e-8, actions to 9.5e-6, all 27 reward terms to 4.8e-07, privileged observations to 3.5e-07,
total reward to 0.4%, step ordering checked line by line against
`legged_robot_go2d1_pos_force.py`. `rollout.py` is a hand-written 50 Hz loop whose only
verification is that it runs, and it has now been shown to violate UniFP's documented ordering and
to run the feet at half the intended friction.

**The ported policy does not crouch in Isaac Lab; it stands slightly taller.** Worth recording
because the failing harness's 0.154 m invites the opposite reading — that number is a robot lying
on its side at 2.4 rad of roll, not a posture.

| | base height, median |
| --- | --- |
| Isaac Gym, `model_48800`, recorded rollout | 0.2924 m |
| Isaac Lab, `model_48800`, 1 env, held 0.5 m/s | **0.3024 m** |
| Isaac Lab, `model_48800`, 50 episodes, sampled commands and forces | **0.3031 m** |
| Isaac Lab, zero actions | 0.2749 m |

Both sit within a centimetre of the task's 0.30 m target, and the policy holds itself about 2.8 cm
above the passive resting height. The comparison is if anything conservative: both stacks measure
**world z** — upstream reads `root_states[:, 2]` directly rather than a terrain-relative height,
so this port does the same — and the Isaac Gym fixture was recorded on the trimesh terrain, whose
0 to 5 cm of height variation is inside that 0.2924 m. Its true height above ground is lower
again, so the real gap is larger than the table shows and runs the same way.

That fits the known model difference rather than contradicting it: passively the two robots differ
by about 3.8 cm (zero actions settle at 23.8 cm in Isaac Gym against 27.5 cm here, F-077), and
under the policy they differ by about one. The controller narrows the gap, which is mild evidence
that it transfers cleanly — it reaches the same commanded posture on a model that rests 4 cm
higher.

So F-077's locomotion claim is **superseded**: the finding is annotated and the walking number
should not be cited. Its standing and solver-sensitivity results are untouched. The specific defect
in `rollout.py` remains unidentified, which is recorded as an open issue rather than closed by the
supersession — the harness is still used by `run_unifp_isaaclab.py`, and anything measured with it
carries this doubt.

### 2026-09-21 — Evaluating what was trained: the ported policy wins, and the reward cannot pick a checkpoint

Rather than start a fourth training run on a configuration nothing had validated, the question
was turned around: is the policy already in hand any good? `unifp_train/eval.py` and
`./run_unifp_train.py eval` answer it, scoring any checkpoint against a frozen manifest **in
Isaac Lab** — so the Isaac Gym-trained policy and the natively-trained one are measured in the
same simulator, on the same episodes, by the same code.

```
./run_unifp_train.py manifest --role validation --num_envs 50 --seed 20260921 --headless
./run_unifp_train.py eval --manifest results/manifests/unifp_isaaclab_validation.json \
  --checkpoint <path> --headless
```

**Why a new manifest rather than the Isaac Gym one.** `results/manifests/unifp_validation.json`
freezes its episodes by one seed and one environment count, and verifies each by a digest of the
schedule the environment realised. That works because every draw comes from the global torch RNG
in a fixed order — a property of the *Isaac Gym* code path. This environment draws the same
quantities from different code in a different order, so the same seed gives different episodes and
those digests can never match. A separate manifest says so; reusing the file and ignoring the
mismatch would not.

The digest machinery earns its keep immediately: **all twelve runs reproduced their frozen
schedules exactly**, which is what makes this a paired comparison and not merely a similar one.

| policy | goal tracking, unforced | p90 | falls | training return |
| --- | --- | --- | --- | --- |
| zero actions | 38.1 cm | 44.8 | 0/50 | — |
| Isaac Lab, 4,400 | 5.7 cm | 8.1 | 7/50 | ~159 |
| Isaac Lab, 12,000 | 49.2 cm | 56.3 | 9/50 | ~96 |
| Isaac Lab, 16,000 | 38.8 cm | 41.0 | **26/50** | ~17 |
| Isaac Lab, 20,000 | 8.0 cm | 10.6 | 0/50 | ~132 |
| **Isaac Lab, 22,000** | **6.2 cm** | 8.6 | 1/50 | ~130 |
| Isaac Lab, 24,000 | 7.3 cm | 10.2 | 0/50 | ~137 |
| Isaac Lab, 26,400 (last) | **60.8 cm** | 65.7 | 1/50 | ~134 |
| **Isaac Gym `model_48800`, ported** | **3.9 cm** | **5.1** | **0/50** | ~154 |

**Two results, in [F-079](../findings.md).** The ported weights beat the natively trained policy
in Isaac Lab's own simulator on Isaac Lab's own frozen set — 3.9 cm and no falls against 6.2 cm
and one. And between iterations 20,000 and 26,400 the training return never leaves the 130–137
band while evaluated tracking swings from 6.2 cm to 60.8 cm, so **the reward cannot select a
checkpoint**: the last checkpoint of the run is the worst of its final five and is beaten by an
inert robot.

That is the answer to the question the whole day was really about. F-078 built the native training
path on the argument that a policy should be trained on the stack it will be evaluated on. Measured
rather than argued, that does not pay here — and the collapses were not cosmetic after all, which
is what the evaluation was run to find out.

**It also puts F-077 in question**, and the finding is annotated to say so. The same `model_48800`
that F-077 reports falling within half a second when told to walk takes **zero falls** across
fifty 20-second episodes here, tracking sampled velocity commands to 0.067 m/s — and the
zero-action baseline's 0.259 m/s error shows those commands are not trivial. The likeliest
difference is protocol, a sustained maximum-speed command against sampled ones, but it is not run
down and should not be asserted.

### 2026-09-21 — Continuing the UniFP training port from week 1

The Isaac Lab training run started on 20 September ([week 1](../week_01/notes.md), run
[p0](#/week/2/run/20260920T044925_train_seed1_p0)) is the subject of everything below. The task
port itself, its verification against the Isaac Gym recording and the launch of that run are in
week 1; what follows is what happened to it.

### 2026-09-21 — The run is unstable: the task port holds, the learning does not

At iteration 23,500 of 60,000 (39%). The task port is doing its job and the *learning* is not, and
those are separable claims here because the two runs can be read against each other iteration by
iteration.

**For the first ~4,500 iterations the two stacks agree closely**, which is the strongest evidence
yet that the task port is right — return 158.9 against 150.3, `tracking_lin_vel_force_world` 1.881
against 1.858, `tracking_ee_force_world` 1.492 against 1.691. That is a live, 4,500-iteration
agreement on a learning curve, not a single rollout.

**After that this run oscillates violently and the Isaac Gym one does not.** Over iterations
5,000–24,000:

| | Isaac Gym (F-072) | Isaac Lab (this run) |
| --- | --- | --- |
| return, median | 152.8 | 129.1 |
| return, minimum | 143.2 | **−80.7** (iteration 16,694) |
| iterations with return < 100 | **0** of 19,001 | **4,124** of 18,583 (22%) |
| iterations with the robot falling inside 10 s | **0** | 1,073 |

Three separate collapses (around 12,000, 15,000–16,000 and 18,000), each recovered from. At
iteration 16,000 the mean episode length was 291 steps — the robot falling in under six seconds.
At 16,694 it stayed upright for the full episode and still scored −80.7, so that one is the
penalties winning rather than a fall: violent arm motion, not a stumble.

It has since been **stable for ~4,500 iterations** — the last 2,000 have a median return of 133.5
and a minimum of 124.8, with the action noise coming back down (0.81 → 0.65) and
`tracking_ee_force_world` back to 1.25. So it is recovering, from a worse place than it should
ever have been in, to a level still 13% below Isaac Gym on return and 24% below on the main
objective.

**The measurable difference is the learning rate schedule.** rsl-rl and legged_gym both adapt it
from the KL divergence with the same bounds and the same `desired_kl = 0.01`. In the Isaac Gym run
the rate oscillates, median 7.59e-05 over the first 8,000 iterations, at the 1e-5 floor 44% of the
time. In this run it reaches the floor at **iteration 130 and never once rises above it in 23,000
iterations** — the KL is persistently over twice the target, the schedule has no headroom left to
damp anything, and the entropy bonus is then free to inflate the action noise, which is what each
collapse looks like from the inside.

One confirmed structural difference between the two PPO implementations, though not yet shown to
be the cause: legged_gym clips the actor and critic gradients **jointly** to norm 1.0
(`clip_grad_norm_(self.actor_critic.parameters(), ...)`) and rsl-rl clips them **separately**, so
the actor here takes larger steps than upstream's for the same gradients. The value loss is small
for most of this run, which weakens that explanation rather than supporting it — separate and
joint clipping converge when the critic's gradient is small. Nothing else found so far differs:
the KL formula, the advantage normalisation, the entropy coefficient, the epochs, the mini-batch
count and the batch size all match.

Neither `rsl_rl` nor this port logs the KL itself, so all of the above is inferred from the rate
it drives. That is the first thing to fix before another run.

Lukas's call was to let it run and watch. It is running; checkpoints every 200 iterations mean
`model_4400` (the pre-collapse peak) and the current stable region are both kept.

### 2026-09-21 — Chasing the collapse: five hypotheses eliminated, one mechanism left standing

> **Superseded as an explanation (2026-09-21, [F-080](../findings.md)).** Every run in this
> entry was made with the stale-observation defect present, so its learning rate was held at
> the 1e-5 floor and part of its gradient came from blanked observations. The numbers are a
> faithful record of what those runs did; they are not evidence about how this task trains.


**Eliminated, each for a stated reason rather than by not looking:**

- *Gradient clipping.* legged_gym clips the actor and critic jointly, rsl-rl clips them
  separately, so this run takes larger steps for the same gradients. It cannot be the cause:
  Adam's update is `m̂ / √v̂`, which is invariant to a uniform rescaling of the gradient, so
  clipping barely moves the step size once the moments have adapted. A real difference in the
  port, and a harmless one.
- *The KL formula.* rsl-rl uses `torch.distributions.kl_divergence(Normal(old), Normal(new))`
  summed over dimensions; legged_gym writes the same expression by hand. Identical.
- *Advantage normalisation, entropy coefficient, epochs, mini-batches, batch size.* All match.
- *Early termination.* Upstream terminates on `|pitch| > 1.0 or |roll| > 0.8` and its
  contact-termination list is empty — the same condition this port implements. Isaac Gym's
  episode length never dropping below 964 is its policy never falling, not a different MDP.
- *The adaptation module perturbing the shared encoder.* This one looked like the answer and is
  worth recording because of how it failed. The estimator optimiser runs at a **fixed** 1e-5 and
  updates the encoder the actor reads, so it puts a floor under the KL that no reduction in PPO's
  rate can get below. Measured offline against the recorded observations and a checkpoint of the
  live run, a fresh Adam gave KL 0.024 after **one** step and 0.21 after twenty — ten times the
  threshold. But a fresh Adam moves every parameter by exactly ±lr on its first step, which is not
  what a running optimiser does. Warming the moments 200 steps first and holding the loss at the
  level the live run actually sits at (5e-4) gives KL **0.0013** after twenty steps — and
  upstream's own checkpoint gives **0.0061**, more than this run's. So the effect is real, small,
  and *larger* in the stack that does not collapse.

**What is left, and it fits every number.** The KL divergence scales as `18·Δμ² / (2σ²)` — with
the inverse square of the policy's own noise. This run's action noise fell to **0.540** by
iteration 4,365; the Isaac Gym run's never went below about 0.66 and was **0.741** at that same
iteration. At σ = 0.54 the same policy change produces 1.88x the KL, so the adaptive schedule
floors the learning rate to compensate — and with the rate floored, the surrogate's pull on σ
weakens against the *fixed* entropy bonus, so σ drifts back up. It went 0.540 → 0.61 → 0.72 →
**0.81**, and the three collapses land on that climb; each recovery coincides with σ falling
again (0.65 now). The run over-sharpened first and everything after is the schedule and the
entropy term fighting over the consequences.

Note what the pinned learning rate does and does not prove. The schedule only *raises* the rate
when KL < 0.005, so a permanently floored rate means the KL is never that low — not that it is
high. It can sit harmlessly inside the dead band. That distinction is why the rate alone was
never going to settle this.

**Instrumented.** `unifp_train/algorithm.py` now records the KL that rsl-rl computes and discards:
the mean, and the fraction of mini-batches on each side of the two thresholds the schedule acts
on. It wraps the actor's KL function for the duration of the update rather than reimplementing
the loop, so the schedule sees exactly what it saw before. First reading, at iteration 42 of a
probe: mean KL **0.0143**, 15% of mini-batches above the upper threshold, 10% below the lower —
more pushing the rate down than up, which is how it reaches the floor and stays.

`run_unifp_train.py` gained `--std_type`, `--init_std`, `--entropy_coef` and
`--learning_rate_scale`. Each defaults to UniFP's own value, so the faithful port is what runs
unless one is named, and naming one is recorded in the run's `run.json` under
`departures_from_unifp`.

**Probe A — the faithful configuration, instrumented.** 500 iterations, 1024 environments,
seed 7, run alongside the long job (which slowed from 2.60 to 4.44 s/iteration for the duration;
the cost is the probe's own length, not the alarming ETA the readout shows while it runs).

| | |
| --- | --- |
| mean KL over the run | **0.0172** against a target of 0.010 |
| iterations with mean KL above the upper threshold (0.02) | 127 of 500 |
| iterations with mean KL **below** the lower threshold (0.005) | **0 of 500** |
| KL, start to end | 0.0135 → **0.0237**, rising |
| action noise, start to end | 1.000 → 0.910 |

This kills my own leading hypothesis and replaces it with a better one. The noise stayed **high**
at 0.91 through the whole probe and the KL still reached 0.024, so `σ` is not what drives it —
the story about the policy over-sharpening was wrong, or at least not the mechanism.

What the probe does show is sharper: **at the minimum learning rate the KL is already above target
and climbing.** The schedule's floor is not a formality here, it is binding, and a schedule
pressed against its bound is not regulating anything — which is why the KL drifts upward unchecked
across a run, and an unregulated KL is what a policy collapse looks like beforehand. Upstream sits
at a median rate of 7.59e-05 with its KL inside the band, so its schedule *is* regulating; this
port's policy moves roughly seven times as far per unit of learning rate, and the schedule
responds by flooring the rate and then losing its grip.

The remedy that follows from the measurement rather than from a guess is to give the schedule room
below 1e-5. rsl-rl hard-codes the clamp inside `PPO.update()`, so `algorithm.ScaledAdam` scales
what the optimizer does with whatever rate it is handed: at `--learning_rate_scale 0.25` the
clamp `[1e-5, 1e-2]` becomes an effective `[2.5e-6, 2.5e-3]` and the schedule works inside it
unchanged.

**Probe B — the same run with that one change.** Identical seed, environments and length; the only
difference is `--learning_rate_scale 0.25`.

| 500 iterations, 1024 envs, seed 7 | A: UniFP as-is | B: rate range moved down 4x |
| --- | --- | --- |
| mean KL | 0.0172 | **0.0132** |
| KL, start → end | 0.0135 → 0.0237 (rising) | 0.0145 → 0.0147 (**flat**) |
| iterations above the upper threshold | **127** of 500 | **0** of 500 |
| nominal rate at the 1e-5 floor | 57% of iterations | **23%** |
| longest unbroken stretch at the floor | 209 iterations | **18** |
| median nominal rate | 1.00e-05 | **5.06e-05** |
| final return | 106.4 | **114.3** |
| final `tracking_ee_force_world` | 1.153 | **1.315** |
| final action noise | 0.910 | 0.900 |

Two things worth drawing out. The first is that **B's schedule behaves like upstream's** — bouncing
off the floor rather than sitting on it, with a longest stretch of 18 iterations against upstream's
217 and a median rate of 5.06e-05 against upstream's 7.59e-05. The pathology is gone, and what
replaces it is recognisably the thing that works in the stack that does not collapse.

The second is that lowering the effective rate made learning **faster**, not slower: return 114.3
against 106.4 and end-effector tracking 1.315 against 1.153. That is the tell that the updates
were outside the trust region rather than merely large — a policy that overshoots and comes back
makes less progress per iteration than one that steps inside it.

**What this does not establish.** 500 iterations, one seed, and 1024 environments rather than the
4,096 the long run uses — and the scale matters here, because the long run is at the floor 99.5%
of the time against probe A's 57%, so the saturation is *worse* at full size and 0.25 may not be
enough. Nothing here shows the collapse is fixed: the first collapse in the long run took about
12,000 iterations to appear, so a 500-iteration probe can only show that the schedule regulates
again, which is the precondition, not the result. And the factor of roughly seven between this
port's policy movement per unit rate and upstream's is still unexplained — `ScaledAdam` treats
the symptom competently without saying what causes it.

### 2026-09-21 — Stopped the first run at 44% and started the fixed one

```
kill -INT $(cat logs/unifp_train/20260920T044925_train_seed1_p0/train.pid)
nohup ./run_unifp_train.py train --num_envs 4096 --iterations 60000 --seed 1 \
  --learning_rate_scale 0.1 --run_name p1_lrscale01 --headless > logs/unifp_train/p1.log 2>&1 &
```

The first run stopped at **iteration 26,466 of 60,000** with 133 checkpoints kept
([run](#/week/1/run/20260920T044925_train_seed1_p0)). Nothing is thrown away: it is the control
for the fix, and the only evidence that the collapse exists at all.

**The stop did not record itself, which is a defect worth naming.** `SIGINT` never reached Python —
Isaac Sim installs its own signal handling — so the script's shutdown path never ran and
`run.json` was left saying `"running"` with no final iteration count. The file has been completed
by hand from the log and the checkpoint directory, and says so in a `record_note` field rather
than pretending it wrote itself. `run_unifp_train.py` now installs explicit `SIGINT`/`SIGTERM`
handlers so the next stop records itself.

**The new run uses `--learning_rate_scale 0.1`, not the 0.25 that was tested.** The probes ran at
1,024 environments and the real run uses 4,096, where the saturation is much worse — 99.5% of
iterations at the rate floor against probe A's 57%. A 4x reduction moved 1,024 environments from
57% to 23%; 4,096 starts far deeper, so it needs more room. The ceiling is not the binding end:
`[1e-5, 1e-2]` scaled by 0.1 is an effective `[1e-6, 1e-3]`, and upstream's highest rate all run
was 6.7e-04, so there is no risk of capping the schedule from above.

0.1 is therefore an extrapolation rather than a tested value, which is why it is checked at
iteration 300 rather than trusted for forty hours: if the schedule is still pinned, the run gets
stopped and restarted lower. That check is cheap now only because the KL is instrumented — the
first run had no way to tell.

### 2026-09-21 — The check failed, and failing told us more than passing would have

The 300-iteration check stopped the run, which is what it was for. `learning_rate_scale 0.1` is
confirmed active — `Loss/effective_lr` is 1.00e-06 against a nominal 1.00e-05, exactly the tenth
asked for — and it changed almost nothing:

| new run, first 300 iterations, 4,096 envs, effective rate **1e-6** | |
| --- | --- |
| mean KL | 0.0165 |
| iterations above the upper threshold (0.02) | 54 of 300 |
| iterations below the lower threshold (0.005) | **0** |
| median nominal rate | 1.00e-05, **still on the floor** |

**Correction to the first version of this entry.** It put 0.0172 in a column headed "old run" and
read a 4% difference off the pair. That number is probe A's, measured at **1,024** environments;
the stopped run predates the KL instrumentation entirely and has no KL to compare with. The two
were not the same configuration and the comparison should not have been drawn. The claim it
supported — that a tenfold rate cut barely moved the KL — is not established by it, and probe D
was run to get the baseline that was missing.

What does hold without any comparison is the last row. The schedule is still sitting on its floor
at an effective rate of 1e-6, with not one mini-batch in 300 iterations falling below the lower
threshold: it is asking to go lower still, ten times below where the first run was already stuck.
A control that is saturated at a tenth of the input is not a control whose input is the problem.

What is left is the one optimizer that `learning_rate_scale` does **not** touch. The adaptation
module has its own Adam, hard-wired at 1e-5, updating the encoder the actor reads. Upstream runs
PPO at a median 7.59e-05 against that 1e-5, so the policy objective outweighs the estimator on
the shared encoder by about **7.6 to 1**. The first run here had them **equal**, at 1e-5 apiece.
The run just stopped made it *worse* — PPO at 1e-6 against an unscaled estimator at 1e-5, a ratio
of 1 to 10, with the estimator now the dominant author of the encoder the policy reads.

That ratio is a measured difference between the two stacks rather than another guess, and it fits
the rest: the earlier offline probe found the estimator's own contribution to the KL small on a
*converged* estimator with a warm optimizer, which is the regime late in a run, not the regime
where the collapses started.

Probe C tests it the only way that settles it — `--estimator_learning_rate 0`, everything else
faithful, 4,096 environments. Probe D supplies the baseline that turned out to be missing: the
same thing with nothing changed at all, which no earlier run had, because the KL instrumentation
postdates the long run.

### 2026-09-21 — It is the batch size, and the schedule was never the disease

> **Superseded as an explanation (2026-09-21, [F-080](../findings.md)).** Every run in this
> entry was made with the stale-observation defect present, so its learning rate was held at
> the 1e-5 floor and part of its gradient came from blanked observations. The numbers are a
> faithful record of what those runs did; they are not evidence about how this task trains.


Five conditions, 300 iterations each, seed 1.

| condition | envs | mean KL | iterations below 0.005 | rate at floor | return | `tracking_ee_force_world` |
| --- | --- | --- | --- | --- | --- | --- |
| D: UniFP unchanged | 4096 | 0.0226 | 0 of 300 | 57% | 95.6 | 0.618 |
| C: adaptation module off | 4096 | **0.0226** | 0 of 300 | 57% | 105.5 | 0.718 |
| p1: PPO rate x0.1 | 4096 | 0.0165 | 0 of 300 | 52% | 87.8 | 0.776 |
| A: UniFP unchanged | 1024 | 0.0141 | 0 of 300 | 28% | 101.5 | 1.052 |
| B: PPO rate x0.25 | 1024 | 0.0135 | 0 of 300 | **3%** | 109.1 | **1.235** |

**The adaptation module is exonerated outright.** C and D agree to four decimal places on the mean
KL. Disabling the thing entirely changes nothing, so the shared encoder was never the mechanism,
and the 7.6-to-1 ratio argument in the entry above — which read well and fitted the numbers then
available — was wrong.

**The 10x rate cut is worth 27%, not the 4% claimed above.** With D as the proper baseline the
figure is 0.0226 → 0.0165. Still sharply sublinear, so a rate-independent floor is real, but the
number in the previous entry came from comparing against a 1,024-environment probe and is
withdrawn.

**What the table actually says is in the `envs` column.** At 4,096 every condition sits at 57%
floor occupancy no matter what is changed; at 1,024 the *same unchanged config* sits at 28%, and
with the rate cut at 3% — the schedule fully back in control. Bigger batches mean less gradient
noise, which means Adam's `m̂ / √v̂` stays nearer one, which means more policy movement per
iteration than the schedule can regulate away. The learning rate was the symptom; the batch size
is the input.

And the end-effector term follows it exactly: **1.235 at 1,024 environments against 0.618 at
4,096**, on a quarter of the samples. That is not sample efficiency, it is optimisation.

**The run that follows from this** is probe B's configuration at full length: 1,024 environments,
`--learning_rate_scale 0.25`, 60,000 iterations. It is also cheaper — about 0.9 s/iteration
against 2.6, so roughly 15 hours rather than 42 — and at 1.47 billion environment-steps it still
clears the ~1 billion that F-072 found was all the Isaac Gym run had needed.

Two things it is not. It is **not** upstream's configuration: UniFP trains at 4,096 and this is a
deliberate, recorded departure on two axes at once, so the comparison with F-072 now carries a
batch-size difference as well as a simulator one. And 500 iterations of probe B is **not**
evidence that the collapse is cured — the first collapse took about 12,000 iterations to appear,
and all that has been shown is that the schedule regulates again, which is the precondition and
not the result.

### 2026-09-21 — The batch-size conclusion was confounded with the seed, and it does not survive

> **Superseded as an explanation (2026-09-21, [F-080](../findings.md)).** Every run in this
> entry was made with the stale-observation defect present, so its learning rate was held at
> the 1e-5 floor and part of its gradient came from blanked observations. The numbers are a
> faithful record of what those runs did; they are not evidence about how this task trains.


The run started on that conclusion was stopped at iteration 435 by its own check. It did not
reproduce probe B at all: mean KL **0.0231** against probe B's 0.0135, the rate at the floor
**57%** against 3%, nothing below the lower threshold, and `tracking_ee_force_world` falling
1.085 → 0.749 → 0.463 across the first 435 iterations.

The reason is embarrassing and worth writing down plainly:

| probe | envs | seed |
| --- | --- | --- |
| A, B | 1024 | 7 |
| C, D, p1 | 4096 | 1 |

**Every 1,024-environment probe used seed 7 and every 4,096-environment run used seed 1.** Batch
size and seed were perfectly aliased across the whole comparison, so the `envs` column of that
table could just as well have been headed `seed`. The new run supplies the cell that breaks the
tie — 1,024 environments at seed **1** — and it lands with the seed-1 group, not the
1,024-environment group. On that evidence the effect I attributed to batch size is the seed.

That also puts probe B back in question. A and B share seed 7, so the A-versus-B comparison is
still controlled and `learning_rate_scale 0.25` really did improve *that* pair. What is not
established is that it generalises, and a second seed is the first thing that failed to reproduce
it.

Probes E and F fill in the missing cells — 4,096 at seed 7, and 1,024 at seed 1 with nothing
changed — to separate the two properly instead of arguing about it. What the 2x2 cannot fix is
that every cell is still a single run: with seed variance this large, one run per condition was
never going to support the conclusions drawn from it, and three configurations have now been
launched on the strength of exactly that.

**The completed 2x2**, UniFP's configuration unchanged in every cell, 300 iterations each:

| envs | seed | mean KL | rate at floor | return | `tracking_ee_force_world` |
| --- | --- | --- | --- | --- | --- |
| 1024 | 7 | 0.0141 | 28% | 101.5 | **1.052** |
| 1024 | 1 | 0.0178 | 50% | 99.0 | 0.878 |
| 4096 | 7 | 0.0194 | 56% | 102.3 | 0.670 |
| 4096 | 1 | 0.0226 | 57% | 95.6 | 0.618 |

Both effects are real and roughly additive on the KL: moving from 1,024 to 4,096 environments
costs about +0.005 in each seed row, and changing the seed costs about +0.0035 in each batch
column. So the batch-size claim was **overstated rather than wrong** — and on the end-effector
term, which is the objective the task is named for, the batch effect is the larger and the more
consistent of the two: −0.38 and −0.26 across the rows against −0.17 and −0.05 for the seed.

What does not survive is the *intervention*. `learning_rate_scale 0.25` took seed 7 from 28% floor
occupancy to 3%; on seed 1 it managed 50% to 38%, with a slightly **worse** mean KL (0.0189
against 0.0178). One seed is not a result, and this is the second time that has been the lesson
today.

**Where this leaves the collapse: not fixed, and now characterised rather than explained.** Every
one of the eight conditions has zero iterations below the schedule's lower threshold, so the
saturation is a property of this port across batch sizes, seeds, learning rates and with the
adaptation module switched off entirely. It is not caused by any of them. The factor of roughly
seven between this port's policy movement per unit learning rate and upstream's remains
unexplained, and four separate mechanisms have been proposed and killed by measurement:
gradient clipping, the KL formula, the shared encoder, and the learning-rate floor.

### 2026-09-21 — Measuring the factor instead of inferring it, and the estimator comes back

> **Superseded as an explanation (2026-09-21, [F-080](../findings.md)).** Every run in this
> entry was made with the stale-observation defect present, so its learning rate was held at
> the 1e-5 floor and part of its gradient came from blanked observations. The numbers are a
> faithful record of what those runs did; they are not evidence about how this task trains.


Everything above reasons about policy movement from the learning rate it drives. That movement is
a property of the weights and can be measured directly: load both stacks' checkpoints **at
matched iterations**, feed both the same recorded observations, perturb every encoder and
actor-body parameter by ±2e-4 — one iteration's worth of Adam at the 1e-5 floor, same random sign
pattern for both — and read the KL that comes out. No simulator, no optimizer, no inference.

| iteration | KL, Isaac Lab | KL, Isaac Gym | ratio | mean abs Δμ, Lab vs Gym | σ, Lab vs Gym | ‖W‖, Lab vs Gym |
| --- | --- | --- | --- | --- | --- | --- |
| 200 | 0.00066 | 0.00033 | 2.0x | 0.0062 / 0.0042 | 0.874 / 0.937 | 47 / 43 |
| 1,000 | 0.00200 | 0.00068 | 2.9x | 0.0097 / 0.0056 | 0.796 / 0.803 | 50 / 51 |
| 4,000 | 0.01345 | 0.00243 | **5.5x** | 0.0174 / 0.0100 | 0.553 / 0.753 | 56 / 63 |
| 10,000 | 0.00621 | 0.00269 | 2.3x | 0.0121 / 0.0094 | 0.631 / 0.707 | 62 / 75 |
| 20,000 | 0.01262 | 0.00327 | 3.9x | 0.0222 / 0.0108 | 0.716 / 0.711 | 70 / 81 |
| 26,400 | 0.01031 | 0.00386 | 2.7x | 0.0168 / 0.0109 | 0.652 / 0.688 | 73 / 84 |

**The factor is real, it is 2 to 5.5 rather than seven, and it is present from iteration 200** —
before any collapse, so it is not their consequence. It decomposes cleanly: the Isaac Lab network
moves its mean about 1.7x further for the same parameter step (which enters the KL squared, so
~3x) and carries lower action noise (which enters as 1/σ², another ~1.9x at iteration 4,000).
It is **not** the weight magnitude — the norms are comparable and Isaac Gym's are *larger* late on.

**Two further measurements turn this from a description into a mechanism.**

*KL is exactly quadratic in the step size.* Scaling the perturbation by 0.316 gives 0.1002 of the
KL against 0.0999 predicted; by 0.1 gives 0.0100 against 0.0100. The network is in its linear
regime, so a tenfold smaller optimizer step **must** produce a hundredfold smaller KL.

Run p1 cut PPO's rate tenfold and the KL fell by 27%. Those two facts cannot both be about the
same optimizer, so most of the movement in p1 was not PPO's.

*The encoder carries 45–78% of the sensitivity* in both stacks. The encoder is exactly what the
adaptation module's own optimizer writes to — at a fixed 1e-5 that `learning_rate_scale` never
touched.

**So the estimator is back, and the earlier exoneration was wrong for a specific reason.** Probes
C and D agreed to four decimals because with PPO running at 1e-5 it *corrects* the estimator's
perturbation within the same iteration — they are interleaved twenty times per update, and the
surrogate loss pushes back on anything the estimator does that hurts the policy. Cut PPO to 1e-6
and it can no longer keep up, which is why p1's KL stayed high while its own steps got ten times
smaller. "Disabling it changes nothing" was measured at the one operating point where the effect
is masked.

Probe G tests the correction: scale **both** optimizers by 0.1, preserving their ratio.

**Probe G came back at mean KL 0.0162 against probe D's 0.0226 — not the hundredfold collapse the
quadratic law predicts.** The prediction was naive and the reason is obvious in hindsight: the
schedule *reacts*. Scaling the optimizer does not lower the KL, it lowers the rate at which a
given KL is produced, and the schedule promptly raises the nominal rate to compensate — median
3.43e-05 against D's 1.00e-05, which is very nearly the 10x it was scaled by. An adaptive
controller cannot be moved by changing its gain.

What G did buy is headroom: the rate is off the floor 62% of the time against D's 43%, the mean
KL is back inside the dead band rather than above it, and return and tracking both improved
(103.0 against 95.6, 0.764 against 0.618).

### 2026-09-21 — The drift is the collapse, and it is visible in 300 iterations

> **Superseded as an explanation (2026-09-21, [F-080](../findings.md)).** Every run in this
> entry was made with the stale-observation defect present, so its learning rate was held at
> the 1e-5 floor and part of its gradient came from blanked observations. The numbers are a
> faithful record of what those runs did; they are not evidence about how this task trains.


The mean KL was the wrong statistic all along. Every one of the nine conditions run today starts
in the same place — mean KL between 0.0138 and 0.0149 over the first fifty iterations — and what
separates them is whether it then **drifts**:

| config | envs | seed | KL drift over 300 iters | `tracking_ee_force_world` change |
| --- | --- | --- | --- | --- |
| PPO x0.25 | 1024 | 7 | **−11%** | **+16.8%** |
| unchanged | 1024 | 7 | +10% | +6.9% |
| PPO x0.1 | 4096 | 1 | +30% | −34.2% |
| BOTH x0.1 | 4096 | 1 | +64% | −20.6% |
| unchanged | 1024 | 1 | +74% | −19.9% |
| estimator OFF | 4096 | 1 | +116% | −24.1% |
| unchanged | 4096 | 7 | +119% | −32.9% |
| PPO x0.25 | 1024 | 1 | +133% | −16.8% |
| unchanged | 4096 | 1 | **+180%** | **−41.4%** |

**r = −0.742** between the two columns across nine conditions. The collapse that took 12,000
iterations to become visible in the long run is legible in **300**, which turns an eight-hour
question into a thirteen-minute one.

It also settles the batch-versus-seed argument that the mean-KL 2x2 could not. Holding the seed
fixed, moving 1,024 → 4,096 environments costs +109% and +106% of drift; holding the batch fixed,
changing the seed costs +64% and +61%. **Both are real and the batch size is the larger of the
two** — which is close to the claim made earlier today and then retracted too far. The mean KL
was simply too blunt to show it.

**What the whole day establishes about the factor.** It is 2 to 5.5x, not seven; it is a property
of the *weights* — the Isaac Lab network moves its output about 1.7x further for an identical
parameter step, at comparable weight norms — and it is there from iteration 200, before anything
collapses. Four candidate mechanisms were proposed and killed by measurement (gradient clipping,
the KL formula, the learning-rate floor, and the estimator's shared encoder — the last twice, in
both directions). None of the interventions tried removes the drift at 4,096 environments; the
best of them halve it.

What has **not** been established is why two networks of the same architecture, trained on the
same task, end up with different sensitivity. The most likely answer is the one thing that was
never a candidate because it is not a bug: they are trained in different physics, so they learn
different functions, and this one is stiffer. That would make the drift a property of the Isaac
Lab model rather than a defect in the port — which is a claim this evidence supports but does not
demonstrate.

### 2026-09-21 — Five seeds: the degradation is the rule, and seed 7 was the exception

> **Superseded as an explanation (2026-09-21, [F-080](../findings.md)).** Every run in this
> entry was made with the stale-observation defect present, so its learning rate was held at
> the 1e-5 floor and part of its gradient came from blanked observations. The numbers are a
> faithful record of what those runs did; they are not evidence about how this task trains.


Screening five seeds at 1,024 environments with nothing changed, 300 iterations each, using the
drift indicator above:

| seed | KL drift | `tracking_ee_force_world` change | final ee | return |
| --- | --- | --- | --- | --- |
| **7** | +10% | **+6.9%** | 1.019 | 104.6 |
| 42 | +37% | −20.9% | 0.754 | 93.6 |
| 3 | +51% | −21.3% | 0.745 | 77.1 |
| 1 | +74% | −19.9% | 0.786 | 96.6 |
| 2 | +90% | −30.5% | 0.670 | 91.7 |

Median drift **+51%** (range +10% to +90%, sd 31); median tracking change **−20.9%**; **four of
five seeds degrade**.

**This retires today's remaining conclusion and explains why it was drawn.** Probes A and B were
*both* seed 7 — the one seed in five that does not degrade. "1,024 environments is better" and
"`learning_rate_scale 0.25` helps" were both measured on the single lucky draw, which is also why
neither reproduced when a second seed was tried. The batch size does still help on the median
(−20.9% against −37% across 4,096's two seeds), but it helps a degrading run degrade less; it
does not prevent it.

Counting every faithful run measured today — five seeds at 1,024 and two at 4,096 — **six of seven
degrade**. The collapse is the typical behaviour of this port rather than an unlucky episode, and
the Isaac Gym run does not do it at all: its end-effector term holds between 1.6 and 1.7 for the
full 60,000 iterations, never once below 143 on return.

So: the task port is verified, the training port runs, and what it trains reliably gets worse at
the objective the task is named for. Nothing tried today prevents that, the mechanism is measured
(the Isaac Lab network is 2–5.5x stiffer per parameter step, from iteration 200 onward) and its
cause is not established.

**What is worth doing next is not another training run.** Committing fifteen hours to one seed has,
on this evidence, about a four-in-five chance of producing a degrading run — and it is still not
known whether degradation in this metric matters for the thing the thesis actually asks. The
stopped run left `model_26400`, which recovered from three collapses and was scoring 134 against
upstream's 154. Evaluating it against `model_48800` on the frozen manifests answers F-077's real
question — does training natively beat porting the weights — and costs no GPU-hours gambled on
a run that will probably degrade.


<!-- Dated entries (### YYYY-MM-DD · topic). For each: the exact command or change, the outcome in numbers,
     and what it does and does not show. Record runs with ./dashboard.py record. -->

## Results

Twenty runs on 2026-09-21, all under **Runs** below: nine probe configurations diagnosing the
training instability carried over from week 1, a five-seed drift scan, and twelve frozen-manifest
evaluations. Frozen set: [unifp_isaaclab_validation.json](../manifests/unifp_isaaclab_validation.json),
50 episodes, content `94e576a6…`, specific to the Isaac Lab simulator and not interchangeable with
the Isaac Gym manifest of the same task.

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-082](../findings.md): both policies track worst at goals low and in front, confirming an
  observation from the viewer — but for opposite reasons. The ported policy oscillates across the
  workspace (error sd 1.8 cm); the native one is steady everywhere (0.4 cm) and under-reaches
  downward by a systematic 4 cm in the lowest pitch band against 1.4 cm elsewhere. Not a joint
  limit, and not under-exposure in training.
- [F-081](../findings.md): with the F-080 fix and a complete 60,000-iteration run, training
  natively in Isaac Lab **beats porting the Isaac Gym weights** — 1.5 cm against 3.9 cm on the same
  50 frozen episodes in the same simulator, better on 50 of 50 paired episodes (p ≈ 2e-15), no
  falls. This reverses [F-079](../findings.md), which measured a defective run stopped at 44%.
  The training return still cannot select a checkpoint: the highest-return one is the only faller.
- [F-080](../findings.md): the Isaac Lab training collapse has a located cause — `ObsHistory.reset`
  blanked, in place, an observation the policy had already acted on, for every environment whose
  episode ended. With both optimisers frozen the port reported a KL of 0.063 at `model_4000` and
  1.055 at `model_10000` against a target of 0.01; after the fix the first mini-batch reports
  exactly 0 and 0 of 98,304 stored samples disagree. On probe D's configuration the learning rate
  goes from the floor 57% of iterations to 0%, and the end-effector term from 0.618 to 1.465.
  Re-taking the five-seed scan reverses its conclusion: 0 of 5 seeds degrade against 4 of 5, and
  every seed now gains on the end-effector term. **Not** verified over a full run — 300 iterations,
  and the first collapse took 5,399.
- [F-079](../findings.md): training UniFP's task natively in Isaac Lab does not beat porting the Isaac Gym weights (3.9 cm against 6.2 cm on the same frozen episodes in the same simulator), and the training return cannot select a checkpoint — it holds at 130–137 while evaluated tracking swings tenfold (confirmed on one run, one seed, 50 episodes; the Isaac Lab run was stopped at 44% of its schedule).
- [F-077](../findings.md): **locomotion claim superseded** — the policy walks (4.1 cm, 0 falls, 0.302 m base height) under F-077's own condition in the verified training environment; the failure is an artefact of the `rollout.py` playback harness, whose specific defect is not yet identified. Standing and solver sensitivity stand.

## Issues and risks

- **Every native training number on this stack predates the F-080 fix** (2026-09-21). The
  26,468-iteration run, the nine probes, the 2x2 and the five-seed scan were all measured with the
  learning rate pinned to its floor by a spurious KL. The five-seed scan has been re-taken and
  reverses (0 of 5 degrade against 4 of 5), but the 26,468-iteration run and F-079's Isaac Lab
  column have not: those checkpoints all come from the affected run and no full-length run has
  been made since.
- **A residual spurious KL of ~5e-4 is unexplained** (2026-09-21). In the frozen null test the
  first mini-batch reports exactly 0 but later ones do not, with no parameter able to move. It is
  twenty times below the schedule's target and the direct storage check is exactly zero at 98,304
  samples, so it is not stale observations. `Loss/kl_first_minibatch` is logged on every run as
  the standing check; if it stops being ~0, this is the thing to chase.
- **`unifp_isaaclab/rollout.py` is not trustworthy and the reason is not known** (2026-09-21).
  It fails to walk where the verified training environment succeeds under an identical condition,
  it advances the gait clock a step ahead of UniFP's documented order, and it runs the robot's
  colliders at Isaac Lab's default 0.5 friction rather than UniFP's 1.0. Correcting either of
  those makes it *worse*, so neither is the defect. `run_unifp_isaaclab.py` uses this harness, so
  F-076's interface results — which are verified offline against recorded Isaac Gym data and do
  not depend on the rollout loop — stand, but any *behavioural* number measured through it should
  be re-taken in the training environment before it is cited.

## Next week

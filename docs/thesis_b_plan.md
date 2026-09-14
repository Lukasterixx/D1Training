# Thesis B testing and training plan

Working draft, 14 September 2026. Based on Thesis A §§3.1–3.4, Table 3,
Table 4, and Appendices A–C. Week numbers refer to the ten-week Thesis B block
in Appendix A, not confirmed calendar deadlines. Numerical gates below are
**proposed engineering targets**, to be agreed and frozen before comparisons.

## Immediate objective

Establish a reproducible Go2+D1 simulation and a position-only whole-body
baseline, then add force awareness and tool conditioning in controlled stages.
Keep the current walking-policy-plus-IK controller as an engineering reference.
The thesis P0 baseline must eventually control the legs and arm through the
same policy architecture used for P1–P4; a separate IK controller is insufficient
for that comparison.

The initial implementation is a free-space stance-and-reach task. This is a
preparation step towards P0, not the completed contact-task baseline. The
[environment guide](position_only_environment.md) separates implemented features
from remaining validation. The [codebase review](codebase_investigation.md)
contains verified repositories, initial findings and reproduction experiments.

## Schedule aligned with Appendix A

The chart approximately places codebase investigation in Weeks 1–6, baseline
validation by Weeks 4–5, task/metric freeze by Week 6, force-aware policy
selection around Week 8, tool comparison around Week 9, and reporting in Week
10. Work may overlap, but dependent training passes its preceding gate first.

| Week | Training and implementation | Testing, evidence and deliverable |
| --- | --- | --- |
| 1 | Audit model, runtime and candidate code; bring up stance-and-reach task | Record versions; verify welded articulation, frames, action order, resets and sensor coverage; capture baseline playback runs |
| 2 | Validate reaching workspace; run first short PPO pilots; build a pressing fixture | Compare held-out targets with IK reference; inspect reward terms, joint saturation and learning stability; select primary implementation |
| 3 | Expand P0 to slow trajectories and controlled base movement; begin force-command and sensor plumbing | Add wiping fixture; test contact calibration and force sign; force-aware work remains exploratory until P0 passes |
| 4 | Tune P0; add first position-only pressing/wiping episodes | Freeze candidate P0 checkpoint and evaluation cases; run baseline gate; diagnose failures rather than extending training blindly |
| 5 | Repeat P0 across seeds; begin P1/P3 force-aware pilots after gate; register probe/pad | Complete codebase comparison; audit tool mass, tip transform and collision geometry; decide whether scraper/lever remain feasible |
| 6 | Freeze task definitions, metrics and training budgets; continue force-aware curriculum | Deliver reproducible task suite and matched P0/P1 and P2/P3 comparisons; document any scope reduction |
| 7 | Train tool-conditioned P4 and unconditioned P3 on identical tool distributions | Begin controlled ablations; hold out tool parameters and contact conditions; collect failures and learning curves |
| 8 | Select force-aware policy on validation results; finish main training | Run matched trials and randomisation tests; freeze policies for final test set |
| 9 | Complete multi-tool comparison; only targeted reruns for identified faults | Analyse seed variation, failure modes and sensitivity; write results and limitations |
| 10 | Reserve compute time for reproducibility fixes | Finish Thesis B report, figures, experiment archive and Thesis C handover |

Physical system identification, shadow testing, low-force robot trials and
sim-to-real results remain principally in Thesis C, as the appendix schedules.
Record actuator and interface requirements during Thesis B to avoid designing
a controller that cannot later be executed on the D1.

## First work package: simulation and position-only control

1. **Audit the existing robot and playback reference.** Verify one articulation,
   12 leg joints, six arm joints and two jaw joints; map by names. Check mass,
   inertia, joint/effort limits and root pose after import. Reproduce bare-Go2,
   Go2+D1 with folded arm, and Go2+D1 with arm motion. Include forward, lateral
   and yaw commands: the thesis reports lateral instability, while the README
   reports good forward walking. These are different tests, not interchangeable
   evidence. Neither establishes that whole-body learning is necessary by itself.
2. **Register frames.** Verify `world`, environment origin, Go2 base, D1 mount,
   `Link6`, grasp point and eventual tool tip. Keep a world target fixed while
   translating/yawing the base. Draw target and measured point. The initial
   zero-offset `Link6` point is a proxy; measure its offset to the physical
   interaction point before publishing end-effector/tool-tip results.
3. **Validate dynamics and contact.** Check arm step responses and effort
   saturation under gravity. The repo's mass distribution and high arm PD gains
   are modelling choices, not identified hardware parameters. Compare timestep
   and gain sensitivity. Compare the Go2 leg models (unitree_rl_lab's measured
   envelope, the task default, against Isaac Lab's stock `DCMotor`) on standing
   and stepping, then keep one fixed across P0–P4. Verify contact sensing on nested D1 links, known static
   loads and a prescribed collision. Confirm forces propagate through the weld.
4. **Bring up the RL interface.** First one visible environment, then four, then
   64 headless environments. Exercise time-limit and fall resets, partial resets
   and different targets per environment. Verify finite observations/rewards,
   correct 18-action order, closed gripper targets, isolated clones and stable
   physics. Zero-action stepping checks the interface, not reaching ability.
5. **Train and assess stance-and-reach.** Begin with a small collision-checked
   workspace, a fixed target per episode and no random disturbances. Compare
   joint/IK diagnostic reference with the learned policy on identical targets.
   Inspect actual position error, uprightness and failures alongside reward.
6. **Grow P0 gradually.** Add smooth world-frame line/circle trajectories, then
   modest base repositioning and locomotion, then position-only fixture contact.
   Add orientation tracking where tool alignment requires it. Here
   “position-only” means no force-control inputs/objective; it need not prohibit
   orientation commands. Match this pose interface in force-aware variants.

The current task uses 200 Hz physics and 50 Hz joint-target updates. The existing
D1 interface code assumes 10 Hz commands. Confirm the actual hardware path;
introduce per-arm command holding, latency and action-history observations
before calling a policy transfer-ready. Test 50 Hz versus the measured command
rate as a modelling sensitivity experiment, not an undocumented change between
ablations.

## Gates and measurement definitions

| Gate | Proposed criterion | If it fails |
| --- | --- | --- |
| G0: environment works | Correct articulation/action mapping; finite state; deliberate timeout/fall and partial reset tests pass; no clone interference; target-frame checks pass | Fix model/interface before PPO tuning |
| G1a: free-space baseline | On 100 frozen 10 s episodes per seed: ≥90% reach within 5 cm for ≥1 continuous second; episode must survive to its end; ≤1% falls | Check goal feasibility, resets, actuators and reward balance; reduce workspace before adding complexity |
| G1b: moving baseline | Same declared error/survival criteria on slow trajectories, plus reported commanded-versus-measured base velocity | Separate trajectory timing errors from locomotion failure; maintain a standing manipulation scope if necessary |
| G2: contact suite | Fixture travel/force signs and contact signals calibrated; repeatable initial states; explicit success/failure definitions; no hidden force inputs to P0/P2 | Fix measurement and freeze tasks before comparing policies |
| G3: experiment ready | At least three independent training seeds; matched budgets and test cases; complete configs/checkpoints/metrics saved | Report exploratory results only until matched runs exist |

For free space, report Cartesian error norm in metres; RMS error
`sqrt(mean(||p_measured - p_target||²))`; 95th percentile; time to reach;
continuous dwell; fall rate; base tilt/height; and joint-limit or effort
saturation. Report transient and final-two-second tracking separately. Count
failed episodes in success-rate denominators; do not hide them by averaging
only surviving trajectories. Treat truncated error traces separately rather
than filling the missing tail with favourable values.

For contact tasks, additionally record normal force RMSE and peak, tangential
load, contact duration/loss events, path completion, completion time and fixture
state. Sample peak forces at physics rate, or retain a within-control-step
maximum; a 50 Hz sample can miss contact spikes. Define contact onset, debounce,
the duration of a “brief” loss, orientation tolerance and force limits during
fixture calibration. Use the same definitions for all policies.

## Contact-task progression

| Task | Initial simulation | Thesis A success requirement to preserve |
| --- | --- | --- |
| Pressing, first priority | Spring-loaded button with travel and spring/damping parameters; bare end effector, then rounded probe | Full button travel and engaged for 1 s; report force error/peak, time and failure reason |
| Wiping, second priority | Planar fixture with known normal; slow prescribed path; rigid pad initially, compliance only after validation | ≥90% path completion and ≥80% contact during motion |
| Scraping, conditional | Prescribed tangential resistance and plastic tool model | ≥90% path completion with at most one brief contact loss |
| Lever, conditional | Hinge with limited resisting torque and an explicit tool engagement model | Commanded angle reached while engagement is maintained |

Begin pressing/wiping implementation early, but train free-space reaching first.
Before Week 6, decide whether the full four-task set is achievable. If reducing
scope, retain pressing and wiping, document the reduction allowed by Appendix C,
and keep the core comparisons. Do not equate bare-finger inability to engage a
particular lever fixture with a general control failure; declare task
applicability and tool-access constraints in advance.

## Controlled training and testing

Preserve Appendix C's policy IDs:

| Policy | Force aware | External tool | Explicit tool descriptors | Main comparison |
| --- | --- | --- | --- | --- |
| P0 | No | No | No | Bare position-only baseline |
| P1 | Yes | No | No | P1 versus P0: force awareness |
| P2 | No | Yes | No | P2 versus P0: tool benefit with position control |
| P3 | Yes | Yes | No | P3 versus P2: force awareness with tools |
| P4 | Yes | Yes | Yes | P4 versus P3: tool conditioning |

Use the same robot dynamics, fixtures, trajectories, reset distributions, joint
controllers, failure limits and evaluation force references. Give all variants
the same geometric task definition at the registered interaction point. Tool
registration is common infrastructure; explicit tool ID, mass, inertia,
geometry/compliance descriptors distinguish P4. Do not accidentally give only
P4 an accurate target frame. Distinguish multi-tool training from generalisation
to held-out tool properties or identities.

P0/P2 receive no desired force, measured/estimated force, force-estimator latent
or force-tracking reward. Ground-truth force may be logged externally and used
for the common failure limits. Actors receive only deployable inputs (no base
linear velocity). All variants share one privileged critic (base linear velocity
and joint torques, following unitree_rl_lab), adopted before any P0 training.
Force-aware critics may add force terms only where the matching actor does.
Match network capacity and history where practical, or include a capacity
control. A reference desired force can exist in the evaluator without being
an input to the position-only controller.

Use training seeds 42, 43 and 44 initially; add two more if the compute budget
allows. Separate development targets, validation targets used for checkpoint
selection, and an untouched final test manifest. Fix the test manifest's initial
states, targets/trajectories, fixture parameters, tool settings and disturbances
so every policy sees matched cases. Evaluate deterministic policy actions.
Report per-seed results and uncertainty across seeds; repeated episodes from one
trained policy are not independent training replicates.

Run a short pilot first: 64 environments × 24 rollout steps × 100 PPO updates
= **153,600 transitions**. Measure simulation throughput, memory and update time
on the actual GPU. Select a shared transition budget for the substantive runs
after that measurement; do not assume equal iterations imply equal compute
when environment counts differ. Record all tuning and pretraining transitions.
Increase curriculum difficulty only after held-out performance improves. If
warm-starting, give matched variants the same starting policy and account for
its training cost. The old 12-action walking checkpoint cannot load unchanged
into the new 18-action actor.

Archive code revision and source snapshot, dependency versions, asset hashes
and referenced asset revisions, complete configuration, RNG seeds, target
manifest, checkpoint hash, transition count, wall time, learning curves,
per-episode metrics and failure videos. Capture terminal metrics before Isaac
Lab automatically resets an environment. Freeze evaluation before large runs.

## Practical learning alongside development

| Period | Skills to practise | Evidence of understanding |
| --- | --- | --- |
| Weeks 1–2 | Isaac Lab manager-based tasks; observation/action/command flow; frame transforms | Explain one full policy step and demonstrate an independent per-environment reset |
| Weeks 2–3 | PPO rollouts, returns, entropy, KL and reward scaling | Read a learning curve together with actual error/fall metrics; diagnose one failed pilot |
| Weeks 3–5 | Contact sensing, fixture dynamics, force projection and curriculum | Calibrated contact trace and force-sign checks on a known fixture |
| Weeks 5–7 | Matched ablations, multi-tool data splits and reproducibility | Frozen experiment manifest and repeatable P0/P1 evaluation |
| Weeks 8–10 | Statistical reporting and failure analysis | Per-seed plots, uncertainty, examples of failure modes and a documented limitation list |

## First-session status and next concrete actions

- Completed: Thesis A/timeline review, repository audit, initial codebase review,
  position-only task and launch scaffold, CPU frame/reward checks and a synthetic
  CPU PPO update against the installed RSL-RL API.
- GPU execution remains unverified. The first session saw no NVIDIA device. A
  second session on the same day found the RTX 4080 and the prerequisite check
  passed, but smoke runs were deferred while another experiment queue used the
  GPU. Current status lives in the [weekly record](../results/week_01/notes.md).
- Next: run the documented preflight and single/four-environment smoke checks
  in a GPU-enabled session; register the end-effector point and validate the
  workspace; complete G0; run the short PPO pilot; then implement the frozen
  episode evaluator and pressing fixture.
- No physics results, learned checkpoints, contact-task success rates or
  sim-to-real claims have been produced by this first-session work.

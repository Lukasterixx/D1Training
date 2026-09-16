# Thesis B and C implementation and evaluation plan

Revised 16 September 2026 following Lukas's request for a real-robot and camera
demonstration in Thesis B, with refinements such as removing AprilTags in Thesis C.
This supersedes the timing in Thesis A Appendix A and the 14 September draft;
it retains the research questions and P0–P4 comparisons from §§3.1–3.4 and Appendix C.
B weeks follow `results/config.json` (Week 1: 14–20 September 2026); these are
working targets, not confirmed university submission dates. Numerical gates are
**proposed engineering targets**; freeze task-specific values by Week 2 and
contact calibration by Week 3, before substantive comparisons.

## Immediate objective

Deliver a repeatable, AprilTag-guided combiner-box demonstration on the physical
Go2+D1 by the end of B: locate the fixture, approach, engage a pre-attached tool,
operate the latch/lever, open the door/lid to a declared extent, withdraw and
verify completion. Target the first complete sequence in Week 6, leaving Weeks
7–9 for integration fixes, comparisons and repeated trials, and Week 10 for the
final demonstration and report. A lever-angle result alone does not establish
that the box was opened.

Reuse released methods: UniFP for force-aware learning, a Go2+D1/Deep-WBC
reference for position-only control, UMI-on-Legs for task-frame trajectories,
and Unitree RL Lab plus the existing D1 client for deployment. Learning means
reproducing a bounded example, tracing its inputs and losses, then adapting it.
The contribution is the tool-aware integration and controlled evaluation on
this robot; each supporting module need not be a new method.

Develop simulation, hardware and perception concurrently. The existing
walking-policy-plus-IK controller is an engineering reference for calibration
and camera-to-arm tests. P0–P4 must use the same learned whole-body architecture
with the declared force/tool differences; IK demonstrations do not pass those
comparisons. Physical actuation remains gated by the corresponding simulation
and interface evidence, rather than by completion of every later experiment.

The initial implementation is a free-space stance-and-reach task. This is a
preparation step towards P0, not the completed contact-task baseline. The
[environment guide](position_only_environment.md) separates implemented features
from remaining validation. The [codebase review](codebase_investigation.md)
contains verified repositories, initial findings and reproduction experiments.

## Thesis B scope and completion

- **Core demonstration:** one specified box in a bounded workspace, AprilTags
  on the fixture/moving parts as needed, a repeatably registered pre-attached
  lever tool, and a programmed high-level task sequence supplying pose and
  force commands to the learned controller. Whole-body coordination may use
  stance/posture changes; long-distance autonomous navigation is outside B.
- **Core experiments:** pressing as the force-calibration task, the box/lever
  sequence as the application, and P0–P4 on declared applicable tasks. Aim for
  at least two tool configurations with held-out geometry/mass variations for
  P3/P4; different tools on different tasks alone do not demonstrate tool
  generalisation. Keep three training seeds for the reduced simulation matrix
  and representative matched physical comparisons where applicable.
- **Evidence:** repeated end-to-end trials, measured force and tool-tip errors,
  failures by sequence phase, and a reproducible archive. All trials count,
  including aborts and interventions. A single selected video is insufficient.
- **C refinements:** markerless perception, broader placement/lighting ranges,
  learned visual trajectories, assisted tool changes and wider physical trials.
  Wiping/scraping are extensions once the core sequence and comparisons work.

In Weeks 1–2, measure the real latch/lever loads, required travel and opening
geometry against D1 reach and capability. Freeze the actual sequence and its
completion measurement. If the original mechanism is infeasible, document a
representative fixture modification and the resulting limitation; do not
silently substitute pressing and call the box demonstration complete.

## Reuse and implementation choices

Source availability was reviewed on 16 September; a source review is not a
reproduced result. Record revisions, licences, adaptations and reproduction
levels in the [codebase investigation](codebase_investigation.md).

| Component | Reuse | Adaptation and decision |
| --- | --- | --- |
| Force-aware controller | [UniFP](https://github.com/unified-force/UniFP): force/position formulation, history encoder, estimator supervision, curriculum and PPO changes | Reproduce a short upstream training example, then adapt to D1 timing, dynamics and tool frames. ROS 2 deployment and imitation collection are still unchecked in its release checklist; do not make them dependencies |
| P0 whole-body reference | [Go2+D1 Deep-WBC adaptation](https://github.com/nayon007/Loco-Manipulation-with-RL-for-Go2-D1-Robot) | Audit the 18-action model, gains, history and arm/leg objectives. The current plain-PPO scaffold is not already a reproduction of Deep-WBC or UniFP |
| Trajectory interface | [UMI-on-Legs](https://github.com/real-stanford/umi-on-legs) | Reuse task-frame pose sequences, timing/preview concepts and evaluation; its arm interface and checkpoints need replacement/adaptation |
| Robot execution | [Unitree RL Lab](https://github.com/unitreerobotics/unitree_rl_lab), local D1 client and IK reference | Extend the Go2 deployment path to both buses; implement measured-state kinematics, matching observation normalisation and timing |
| B perception | [RealSense ROS](https://github.com/realsenseai/realsense-ros) and [AprilTag ROS 2](https://github.com/christianrauch/apriltag_ros) | Calibrate camera-to-robot and tag-to-task transforms; track moving parts separately; quantify pose error, latency and visibility |
| C perception | [FoundationPose](https://github.com/NVlabs/FoundationPose); optional [Grounding DINO](https://github.com/IDEA-Research/GroundingDINO)/[SAM 2](https://github.com/facebookresearch/sam2) masks | Test recorded box/handle RGB-D footage before closed-loop use. A mask alone is not a 6D pose, and one rigid pose does not represent the whole articulated box |
| C learned task policy | [Diffusion Policy](https://github.com/real-stanford/diffusion_policy), UMI data/interface patterns | Collect time-aligned images, tool poses, actions and force references in B; train a task-specific policy in C if useful. Motion demonstrations alone do not label desired force |

Default to the existing Isaac Lab integration. Time-box initial legacy Isaac
Gym setup to two working days per chosen reproduction, with a total investigation
decision at the end of Week 2. Reproduce UniFP and one position-only reference;
avoid rebuilding every paper. Decide whether to port the method or retarget its
original stack from actual launch/adaptation evidence. Keep legacy environments
isolated from the shared Isaac Lab installation. Record failures and unresolved
dependencies; an upstream checkpoint is not a Go2+D1 checkpoint.

## Camera, task and controller interface

The B execution path is:

`RealSense + AprilTags → task/handle pose → programmed task sequence → tool-tip pose and force commands → learned whole-body policy → Go2 and D1 interfaces`

Measured encoders/IMU and the force estimator feed controller state construction;
task observations also determine phase changes and completion. Logging observes
this loop in simulation and on hardware. The force instrument supplies calibration
and evaluation ground truth, not a hidden actor input to P0/P2.

Freeze the interface in Week 2: frame names, metres/radians/newtons, quaternion
ordering, tool transform, pose/force command axes, timestamps, update rates,
interpolation and stale-data handling. Publish the same contract from scripted
tasks and any later learned visual policy. Register tag-to-handle geometry and
observe door/latch motion; observing only a fixed enclosure cannot verify opening.

Use camera-relative task estimates or measured base localisation to transform
world/task goals as the robot moves. Compute tip pose from measured joint states
and calibrated kinematics at their actual sample times. The present simulation
uses exact root/link poses for these terms despite sampled arm encoders; replace
that shortcut or quantify its effect before substantive transfer training. Model
pose noise, update rate, latency and dropouts from measurements. On stale vision,
stop advancing the task and follow a validated hold/retreat action while the
balance controller continues running. Reacquire before resuming.

## Thesis B weekly schedule

Weeks overlap across controller learning, camera integration and hardware work.
The dates below assume regular robot/fixture access and available GPU sessions;
confirm access in Week 1. Maintain evidence and write methods/results every week.

| Week | Reuse, simulation and learning | Hardware, camera and deliverable |
| --- | --- | --- |
| 1 · 14–20 Sep | Close remaining G0 checks; resolve/reset-test target workspace; start frozen evaluator; trace UniFP and position-only reference | Bring up RealSense/tags and D1 telemetry; measure basic arm response and box loads; record access/fixture needs |
| 2 · 21–27 Sep | Select implementation from bounded reproductions; validate P0 candidate; define slow pose trajectories and pressing/box fixtures | Freeze task/interface/tolerance and comparison manifests; calibrate tool/camera/tag frames; replay observations through robot-side inference with actuation disabled (G4) |
| 3 · 28 Sep–4 Oct | Adapt history-based force estimator and force curriculum; calibrate contact suite (G2); evaluate P0 trajectories | Validate camera-to-IK reference; complete G4 and candidate G1 checks; first bounded learned free-space trials when G5 entry conditions hold |
| 4 · 5–11 Oct | Repeatable simulated pressing and box phases; P1/P3 pilots; start matched seeds once configuration is stable | G5 free-space evidence; first instrumented low-force contact; compare estimated/measured forces and update dynamics/timing (G6) |
| 5 · 12–18 Oct | Register tool variants; train P2/P3/P4; reproduce lever engagement and opening in simulation | Validate physical box phases; connect AprilTag targets to learned control; measure task success separately from force-estimator accuracy |
| 6 · 19–25 Oct | Resolve transfer failures; maintain matched training/configuration records | First complete tagged-box sequence with a force-aware tool policy; record every attempt. Pilot only until G7 repeated-trial criterion passes |
| 7 · 26 Oct–1 Nov | Complete P0–P4 training on the reduced suite; held-out tool variations and seed comparisons | Improve repeatability and test declared box placements; collect representative matched physical comparisons |
| 8 · 2–8 Nov | Select on validation and freeze final checkpoints/configurations; satisfy G3 before comparative claims | Freeze demo setup; run untouched physical test manifest and G7; archive force, camera, joint and task-phase traces |
| 9 · 9–15 Nov | Final test analysis; only documented fault-driven reruns | Integration/repeatability buffer; count all failures; report sim-to-real performance gap and remaining limitations |
| 10 · 16–22 Nov | Finish Thesis B report, figures and reproducibility archive | Final tagged-box demonstration and handover: calibrated system, successful/failed trials and prioritised C refinements |

### Scope decisions and contingency

- **End of Week 2:** freeze the achievable box mechanism, core tasks, interfaces
  and implementation. Scope additional tasks from measured loads and pilot costs.
- **End of Week 4:** require a useful P0, calibrated force experiment and a viable
  hardware/perception path. If behind, defer wiping, scraping, autonomous tool
  changes and large-workspace locomotion; keep the tagged-box sequence central.
  If proprioceptive force estimation fails, diagnose timing/observability and
  reduce speed/load. A sensor-assisted diagnostic controller must be labelled as
  a method change and does not validate the sensorless thesis claim.
- **End of Week 6:** if the complete sequence has not run, stop adding capabilities
  and spend Weeks 7–9 on the recorded blocking phases. Preserve AprilTags and the
  programmed task interface. Record any remaining demonstration shortfall in B;
  moving it into C is recovery work, not polish or a completed B outcome.
- **Week 8 freeze:** no new architecture or perception dependency. A substantive
  post-freeze fix requires a versioned configuration and rerunning affected
  comparisons. Never relax success thresholds after seeing final test results.

## Thesis C refinement and overflow

Keep the working tagged pipeline as the reference. These are relative C weeks;
calendar dates depend on the confirmed C timetable. Complete any declared B
shortfall first and adjust the extension scope accordingly.

| C period | Work | Evidence / stop condition |
| --- | --- | --- |
| Weeks 1–2 | Reproduce B from its archive; test FoundationPose or another markerless pose method on recorded RGB-D; collect calibration/reference geometry | Compare pose errors, latency and failures with independent/tag reference measurements; meet the same task-derived tolerances before deployment |
| Weeks 3–4 | Replace tag observations behind the frozen task interface; evaluate occlusions and reacquisition | Repeat the same box-placement trials with markerless control. Tags may provide evaluation ground truth but must not feed the markerless controller |
| Weeks 5–7 | Broader tool/placement/contact tests; optional Diffusion Policy task sequence or assisted tool change; wiping/scraping if justified | Add one extension at a time with a matched baseline. Train visual policies on development demonstrations and hold out final test cases |
| Weeks 8–10 | Complete broader physical comparisons, analysis, thesis writing and presentation | Clearly separate B tagged results, markerless results, learned-task-policy results and any unresolved limitations |

Removing tags is an engineering/research extension with its own validation,
not a guaranteed cosmetic change. If markerless pose accuracy is insufficient,
retain the tagged system for the core control evaluation and report the limit.

## First work package: simulation and position-only control

1. **Audit the existing robot and playback reference.** Verify one articulation,
   12 leg joints, six arm joints and two jaw joints; map by names. Check mass,
   inertia, joint/effort limits and root pose after import. Reproduce bare-Go2,
   Go2+D1 with folded arm, and Go2+D1 with arm motion. Include forward, lateral
   and yaw commands: the thesis reports lateral instability, while the README
   reports good forward walking. These are different tests, not interchangeable
   evidence. Neither establishes that whole-body learning is necessary by itself.
   Rescue's flat ablation (F-008) found locomotion policies trained with the arm
   attached gathered their feet under the body, while policies trained without it
   walked the welded robot well.
2. **Register frames.** Verify `world`, environment origin, Go2 base, D1 mount,
   camera, tag, fixture, gripper and tool tip. Keep a world target fixed while
   translating/yawing the base. Draw target and measured point. The current
   Link7_1 pincer point is CAD-derived (F-013); measure the physical interaction
   point and each attached tool transform in B Weeks 1–2.
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
6. **Grow P0 gradually.** Add smooth task-frame line/circle trajectories and
   orientation tracking, then position-only fixture contact and modest base
   repositioning where required. Extensive locomotion need not block standing
   manipulation. Here
   “position-only” means no force-control inputs/objective; it need not prohibit
   orientation commands. Match this pose interface in force-aware variants.

The current task uses 200 Hz physics and 50 Hz policy steps. Its default
`--latency estimated` profile holds D1 arm targets for 5 policy steps (the SDK's
10 Hz streaming rate), samples arm joint angles at 10 Hz with velocities
differenced from them, and delays Go2 leg commands by 0–10 ms. These are
estimates from interface code (the D1 SDK's rates and unitree_rl_lab's 1 kHz
command loop), not hardware measurements. Keep one latency profile across P0–P4.
Run `--latency none` as a declared sensitivity experiment, not an undocumented
change between ablations. Measure D1 command-to-motion latency, firmware
smoothing and joint speed before calling a policy transfer-ready.
Schedule those hardware measurements in B Weeks 1–2; validate the resulting
model in Weeks 3–4. The existing 4000/400 simulated D1 gains are not hardware
commands. Measure actual response rather than copying gains from the Z1.

## Gates and measurement definitions

| Gate | Proposed criterion | If it fails |
| --- | --- | --- |
| G0: environment works | Correct articulation/action mapping; finite state; deliberate timeout/fall and partial reset tests pass; no clone interference; target-frame checks pass | Fix model/interface before PPO tuning |
| G1a: free-space baseline | On 100 frozen 10 s episodes per seed: ≥90% reach within 5 cm for ≥1 continuous second; episode must survive to its end; ≤1% falls | Check goal feasibility, resets, actuators and reward balance; reduce workspace before adding complexity |
| G1b: moving baseline | Same declared error/survival criteria on slow trajectories, plus reported commanded-versus-measured base velocity and gait quality (foot placement against the neutral point, front–rear spacing, backward motion after a forward request, turn tracking at several rates, pitch wobble) | Separate trajectory timing errors from locomotion failure; compare against a locomotion policy trained without the arm (F-008); maintain a standing manipulation scope if necessary |
| G2: contact suite | Fixture travel/force signs and contact signals calibrated; repeatable initial states; explicit success/failure definitions; no hidden force inputs to P0/P2 | Fix measurement and freeze tasks before comparing policies |
| G3: experiment ready | At least three independent training seeds; matched budgets and test cases; complete configs/checkpoints/metrics saved | Report exploratory results only until matched runs exist |
| G4: measured observations and deployment contract | By Weeks 2–3: calibrated camera/tag/tool frames; measured timing; real-state FK and matching normalisation/action mapping on both buses; recorded-state replay and disabled-actuation inference agree within frozen tolerances; target transforms remain correct under base motion; deliberate stale-vision test passes | Fix state construction and interfaces before learned robot actuation; continue independent simulation work |
| G5: physical free-space control | Entry: G4 and candidate G1a, plus G1b for any commanded moving-base case. Exit by Week 4: bounded physical reaching/pose trajectories meet task-specific position/orientation tolerances without falls or limit violations; abort/hold behaviour demonstrated | Reduce workspace/speed and identify model or timing mismatch before contact. Multi-seed G3 comparisons can finish later |
| G6: physical force/contact control | Entry: G2, G5 and successful simulated task cases. Exit by Weeks 4–5: repeatable instrumented low-force contact; force estimate error, peak force and contact loss within predeclared task-specific bounds; tool retained and aborts verified | Diagnose observability, actuation or fixture mismatch; do not treat simulated force estimates as physical measurements |
| G7: integrated box demonstration | Entry: G6 and validated simulated sequence. First complete sequence targeted Week 6; final by Weeks 8–10: at least 16/20 successful physical trials across four frozen box placements × five repeats, no falls or force-limit violations. Camera-derived targets drive the learned force-aware tool policy through opening and withdrawal without intervention | Record failure phase and all attempts; use integration buffer. Any unmet B outcome remains explicitly incomplete |

G1a's 5 cm radius is a free-space learning gate, not a lever engagement tolerance.
By Week 2, declare position/orientation tolerances from tool/fixture clearances,
force and estimator-error bounds, box opening extent, allowable contact loss,
phase timeouts and the supported placement range. Finalise calibrated contact
definitions in Week 3 before contact comparisons. If a criterion is infeasible,
revise the task and version its manifest before evaluation, retaining the old
result. The [gate record](../results/gates.md) tracks evidence, not calendar promises.

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
| Pressing, B calibration/comparison | Spring-loaded button with travel and spring/damping parameters; bare end effector, then rounded probe | Full button travel and engaged for 1 s; report force error/peak, time and failure reason |
| Lever and box opening, B application | Measured latch/lever and door/lid articulation, resisting load and tool engagement; implement each phase before joining the sequence | Preserve Table 4's commanded angle with engagement. Additionally verify latch release, declared door/lid opening, tool withdrawal and final state for the box demonstration |
| Wiping, B extension or C | Planar fixture with known normal; slow prescribed path; rigid pad initially | ≥90% path completion and ≥80% contact during motion |
| Scraping, C unless core completes early | Prescribed tangential resistance and plastic tool model | ≥90% path completion with at most one brief contact loss |

Pressing provides an early force experiment while the box fixture is developed
in parallel. This changes the earlier pressing/wiping-first task selection to
serve the requested box demonstration. Document the reduced suite under Appendix
C's scope provision at the Week 2 decision. Declare task applicability in advance:
bare-finger inability to engage a lever is an access limitation, not proof of
inferior control. Use pressing for matched bare/tool comparisons and identical
applicable tool distributions for P3/P4. If tool diversity is insufficient in B,
label H3 evidence preliminary and complete the broader study in C.

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

Adopting UniFP's history/estimation method changes the current plain-PPO scaffold.
Freeze the common architecture before comparative P0 training; preliminary
scaffold pilots are not automatically the P0 baseline. Remove force-supervised
latents as well as explicit force inputs from P0/P2. An upstream reproduction
and a modified Go2+D1 method must be labelled separately.

For low-level comparisons, hold the high-level task sequence, perception source
and command timing fixed. Separate controller-only tests with registered targets
from end-to-end camera trials. Report tagged and markerless results separately;
when C changes perception, first compare it using the same controller checkpoint.

Use training seeds 42, 43 and 44 initially; add two more if the compute budget
allows. Separate development targets, validation targets used for checkpoint
selection, and an untouched final test manifest. Fix the test manifest's initial
states, targets/trajectories, fixture parameters, tool settings and disturbances
so every policy sees matched cases. Evaluate deterministic policy actions.
Report per-seed results and uncertainty across seeds; repeated episodes from one
trained policy are not independent training replicates.

The initial short pilot is 64 environments × 24 rollout steps × 100 PPO updates
= **153,600 transitions**. Measure simulation throughput, memory and update time
on the actual GPU after method/contact changes; initial scaffold pilots already
ran on 15 September. Select a shared transition budget before substantive runs;
do not assume equal iterations imply equal compute
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

On hardware, also retain synchronised RGB-D/tag observations, calibration files,
reported joint states, commands, force-instrument readings, estimator outputs,
phase transitions, abort reasons and manual interventions. Record actual sensor
rates; an estimated force trace cannot establish its own accuracy. Plan GPU and
disk capacity from pilot measurements, including camera recordings and retained
checkpoints. P0–P4 × three seeds is already 15 runs before tuning and reruns;
reduce task breadth before removing the evidence needed for comparisons.

## Practical learning alongside development

| Period | Skills to practise | Evidence of understanding |
| --- | --- | --- |
| B Weeks 1–2 | Reproduce a released controller; trace observations/actions and losses; camera/task transforms | A recorded reproduction attempt, annotated method mapping and a calibrated tag-to-tool target |
| B Weeks 3–4 | UniFP history/force supervision; contact sensing; hardware timing | Force-sign calibration, estimator error against instrumentation, and replay parity across sim/real processing |
| B Weeks 5–6 | Tool registration; sequence execution; transfer diagnosis | Complete tagged-box attempt with synchronised observations and a phase-by-phase failure diagnosis |
| B Weeks 7–10 | Matched ablations, held-out tools, statistical reporting and reproducibility | Per-seed plots, repeated physical trials, uncertainty and a reproducible handover |
| C | Markerless pose estimation; optionally demonstration learning | Same-task comparison against the tagged/scripted reference with independently measured errors |

## Recorded status and next actions

The [Week 1 record](../results/week_01/notes.md) contains the completed simulator
checks, arm-drive correction, short PPO pilots, playback and CAD tool-point
verification. These establish useful infrastructure, not validated reaching,
force-aware performance or physical transfer. Ongoing source edits alone do not
pass a gate.

The target/reset issue F-013 recorded was resolved on 16 September with fresh
evidence (F-016): the target box moved forward and down, below where the tool
point rests, and the reset drop was replaced by a standing spawn. Zero actions now
score 0 of 256 within 5 cm, against 12.9% before, so a reach metric on this task
measures reaching; `verify` passes 23/23. Two corrections came with it: the
backward slide at reset is the zero-action posture settling rather than the drop,
and it is repeatable between environments to 0.005 cm (F-014); and the CPU
workspace model puts the resting tool point 1.6 cm from where the simulator rests
it, so task geometry is set from simulator measurements (F-015). No policy has
been trained against the new box, and its difficulty is unvalidated.

Next: complete the frozen-manifest evaluator and remaining G0 checks; record a
bounded upstream reproduction; validate the revised workspace and P0 candidate;
bring up AprilTags and robot telemetry; measure the arm and box mechanism; freeze
the deployment/task contract in Week 2. New gates G4–G7 are planned work, not
evidence that hardware or perception already works.

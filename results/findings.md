# Findings

One entry per conclusion, numbered in order found and never deleted. A later
result that changes a conclusion gets a new entry, and the old one is marked
`superseded` or `retracted` with a link. Status vocabulary is in the
[results README](README.md#conventions).

### F-001 — The walking policy carries the welded 3.152 kg D1 at 1 m/s forward

- **Status:** superseded
- **Superseded by:** F-012, below (2026-09-15: the forward result reproduced this term, with lateral and yaw added)
- **Week:** pre-term
- **Date:** 2026-07-30
- **Evidence:** [README results table](../README.md#results): 10 s at a commanded 1.0 m/s on flat ground via
  `./run_sim.sh --headless --selftest 10`. Bare Go2: 9.56 m (96% of commanded), max tilt 5.9°. Go2+D1 at
  3.152 kg: 9.51 m (95%), max tilt 7.3°. Base mass raised to 6.0 kg: 9.10 m (91%), 9.3°.
- **Scope:** one 10 s forward run per configuration, taken in the pre-term testbed. Lateral and yaw commands
  were not tested, and Thesis A reports lateral instability. The `--arm_mass` model adds mass at the
  arm base, not at the end effector. Raw logs were not archived, and this has not been re-run this term.
- **Implication:** a welded payload does not by itself break forward walking. That does not show whole-body
  learning is unnecessary. Week 1 playback should add lateral and yaw commands before this is relied on.

### F-002 — Arm end-effector droop is set by PD stiffness, not by the IK solver

- **Status:** superseded
- **Superseded by:** F-010, below (2026-09-15: these droop figures were measured with the URDF import's
  acceleration drives, whose gains PhysX scales by joint inertia; "not the IK solver" still stands)
- **Week:** pre-term
- **Date:** 2026-07-30
- **Evidence:** [README "Known behaviour"](../README.md#known-behaviour): holding one IK target, EE error was
  30.3 cm at stiffness 100, 19.6 cm at 400, 13.1 cm at 800 (Rescue's value) and 3.3 cm at 4000. PhysX still
  clamps effort at the published 3.3/1.7 Nm.
- **Scope:** one target, and the raw logs were not archived. 4000/400 is a modelling choice standing in for the
  D1's internal servo loop, not an identified hardware parameter.
- **Implication:** the 18-action task uses the same 4000/400 arm gains. Gain sensitivity is on the G0/dynamics
  checklist, and a policy trained against these gains may not transfer without actuator identification (Thesis C).

### F-003 — The position-only task's frame maths and PPO configuration work on CPU

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-14
- **Evidence:** `python -m unittest discover -s tests -v` in `env_isaaclab`: 5 tests pass (re-run 2026-09-14).
  They cover world-fixed targets under base translation and yaw, invariance to environment offset,
  tip-offset rotation including the quaternion sign, reward units, and one synthetic PPO update through
  RSL-RL 5.0.1 with 69 observations and 18 actions: losses finite, actor weights changed.
- **Scope:** interface and algorithm-API compatibility only. No PhysX, no robot learning.
- **Implication:** the simulator smoke test is the next unverified layer (G0).

### F-004 — Primary implementation: the local Isaac Lab Go2+D1 model; external codebases as references

- **Status:** provisional
- **Week:** 1
- **Date:** 2026-09-14
- **Evidence:** [codebase investigation](../docs/codebase_investigation.md): source-level review of the Go2+D1
  Deep-WBC adaptation, Deep-WBC, UMI-on-Legs, UniFP and Visual Whole-Body Control at pinned revisions. All are
  Isaac Gym / Python 3.8 stacks. None was installed or run.
- **Scope:** an engineering recommendation from compatibility, not a measured performance ranking.
- **Implication:** the selection is due at the end of Week 2. Reproduction attempts are time-boxed and recorded
  as source-only / installed / launched / checkpoint replayed / short training passed / independently evaluated.

### F-005 — Unitree's Go2 envelope allows 56% more driving torque at speed, but less braking, than Isaac Lab's stock model

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-14
- **Evidence:** both limit functions evaluated on CPU (`motor_model.py`, `tests/test_sim2real.py`). Stock
  `DCMotor` (23.5 N·m saturation, 30 rad/s) against unitree_rl_lab's Go2 envelope (Y1 20.2, Y2 23.4 N·m;
  X1 13.5, X2 30 rad/s). Driving-torque limit at 0 / 5 / 10 / 13.5 / 20 / 25 rad/s: stock 23.5 / 19.6 / 15.7 /
  12.9 / 7.8 / 3.9 N·m, Unitree 23.4 / 20.2 / 20.2 / 20.2 / 12.2 / 6.1 N·m. Braking at 20 rad/s: stock 23.5 N·m,
  Unitree 14.2 N·m. The port reproduces the original unitree_rl_lab function exactly (0.0 max difference on an
  801 × 1001 grid).
- **Scope:** a comparison of two models' torque limits. It does not show which matches a real Go2, or how
  much the difference changes standing, stepping or reaching. The Unitree values are the manufacturer's;
  this project has not measured them. Neither model includes latency.
- **Implication:** the leg model is a controlled variable. The position-only task defaults to the Unitree
  envelope and must keep one model across P0–P4. The smoke test compares both.

### F-006 — unitree_rl_lab offers deploy parity and a motor envelope, not demonstrated transfer metrics

- **Status:** provisional
- **Week:** 1
- **Date:** 2026-09-14
- **Evidence:** source review of `~/mairo-rl-lab-rinam` at `a179aa0`
  ([codebase investigation](../docs/codebase_investigation.md#source-level-findings-to-act-on)). The shipped base
  policy trained with zero actuator delay and zero Go2 friction (`params/env.yaml`). The actor excludes base linear
  velocity. `deploy.yaml` plus the C++ controller replay the observation and action processing on the robot. The
  repo's sim-to-real evidence is demo footage only.
- **Scope:** source reading only; nothing was installed, trained or deployed from that repo. "Good sim to real"
  is the recommendation, not a measured result in the repo.
- **Implication:** adopted the envelope, the deployable-actor/privileged-critic split, the noise and
  randomisation values (opt-in) and a two-bus deploy manifest. Latency and D1 servo behaviour still need
  measurement (Thesis C system identification). The estimated model added the same day is F-007.

### F-007 — The D1 servo and latency model rests on published torques, SDK timing and two unverified estimates

- **Status:** provisional
- **Week:** 1
- **Date:** 2026-09-14
- **Evidence:** the MaiRo/unitree_rl_lab repo has no D1 content and no non-zero latency (repository search; base
  policy `params/env.yaml` `min_delay = max_delay = 0`). Unitree's `~/Downloads/d1/d1_description` URDF has
  `effort="0" velocity="0"` on every joint. `d1_arm/d1.urdf` speed limits (1.05/1.73 rad/s) first appear in the
  Rescue port commit of 2026-07-16 without a source. A web search on 2026-09-14 found D1 retail specifications
  giving range, reach, payload and power, but no joint speed or torque curve. Rescue's `d1_sdk` records 10 Hz angle
  feedback and ~10 Hz streamed setpoints. unitree_rl_lab's Go2 controller publishes commands from a 1 kHz loop
  fed by a 50 Hz policy thread.
- **Scope:** the torque limits are published; the 10 Hz rates come from SDK code. The D1 speed limits and the
  0–10 ms leg delay are unverified. Firmware smoothing, transport latency and the servos' torque–speed
  behaviour are unmodelled.
- **Implication:** `--latency estimated` and `--arm_actuator d1_servo` are the task defaults and must stay fixed
  across P0–P4. `--latency none` is a declared sensitivity run. Thesis C system identification should measure D1
  joint speed, command-to-motion latency and smoothing, plus Go2 command latency, and replace the labelled
  values in `motor_model.py`.

### F-008 — In Rescue's flat locomotion ablation, training with the arm attached degraded the gait

- **Status:** provisional
- **Week:** 1
- **Date:** 2026-09-15
- **Evidence:** [Week 1 log, 2026-09-15](week_01/notes.md) and
  [external/rescue_flat_ablation](week_01/external/rescue_flat_ablation/README.md). Eight locomotion policies were
  benchmarked on the same welded Go2+D1 with the measured motor (Rescue `measure_bench.py`, 200 robots, flat).
  Trained without the arm: P2Dingo flat, "all removed", "no arm". Their rear feet land −2, −1 and −3 cm from the
  neutral point at 0.5 m/s, with 66, 58 and 54 cm same-side spacing at 1.0 m/s. Trained with the arm: the two
  unchanged Rescue seeds and the no-randomisation, stock-motor and 8 cm-clearance runs. Their rear feet land +18,
  +3, +14, +17 and +17 cm ahead, with 32, 36, 32, 35 and 32 cm spacing; the +3 cm run does it with a 12 cm,
  131 ms shuffling stride. Turn tracking at 1.0 rad/s: 99–109% without the arm, 76–105% with it. The stock-motor
  arm run backed away after a forward request in 99.5% of robots.
- **Scope:** Rescue's velocity-tracking task, with 12 leg actions and a passive, folded arm; not whole-body control.
  Flat ground only. 2000 iterations from scratch. One training seed per configuration: seed 42, plus seed 2 for
  the unchanged task, so configuration differences share a seed and are not independent replicates. The mechanism
  is not established; Rescue suggests the arm's added height and pitch inertia. Measured in the Rescue sim, not
  this repo's.
- **Implication:** attaching the arm to the training robot is not free, and randomisation, the motor model and
  the clearance target were ruled out as the cause here. Thesis B's P0 trains with the arm on, so its moving
  stage (G1b) must report gait quality: foot placement against the neutral point, front–rear spacing, backward
  motion after a forward request, turn tracking at 0.2/0.5/1.0 rad/s and pitch wobble. Tracking alone would
  pass a gathered-feet gait. A locomotion policy trained without the arm, which walks the welded robot well, is
  a candidate reference or warm start for the leg half of P0. This supports F-001.

### F-009 — The position-only task's interface, timing, terminations and partial resets pass deliberate checks in Isaac Sim

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-15
- **Evidence:** [Week 1 log, 2026-09-15](week_01/notes.md) and `python run_position_only.py verify --headless --num_envs 8`
  (defaults, 20/20 checks, run `20260915T100737_413430Z_verify_seed42`). Arm targets change exactly every 5 policy
  steps and arm feedback is sampled every 5, each at its own phase per environment. A fresh sample equals the
  simulator's angle (0.0 rad error), and the differenced arm velocity matches to 1.2e-7 rad/s. Legs update every
  step, and leg command delays of 0, 1 and 2 physics steps all occur. Time limit, low base, tilt, base contact and
  workspace exit were each induced in their own environment. Only the expected term fired; base contact also
  trips the height limit, as it must on flat ground. Only that environment reset, to the default root pose and the
  soft-limit-clamped default joints (0.0 error), with a new target inside the box. Three control environments carried
  on: episode length 61, same target, 0.5 mm base drift. Link6's position matches URDF forward kinematics to 0.6 µm,
  and PhysX link masses and centres of mass match the weld mass model exactly. Smoke runs confirm 66/87 observation
  widths and the deploy manifest's unitree_sdk2 motor order. PhysX applies the arm's 3.3/1.7 N·m and 1.05/1.73 rad/s
  limits.
- **Scope:** mechanism checks in one configuration (defaults) with zero or random arm actions, one seed. Clone
  isolation is shown only as control environments being unaffected. This says nothing about standing, stepping or
  reaching ability.
- **Implication:** G0's action mapping, finite state, reset and target-frame items have evidence. The tool point and
  a visual inspection remain.

### F-010 — The URDF import made the D1's joints acceleration drives, which let the arm sag 0.11 rad inside its torque limits

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-15
- **Evidence:** [Week 1 log, 2026-09-15](week_01/notes.md). Drive types read from the stage: `Joint1`–`Joint6`
  `acceleration`, Go2 legs `force`. The robot stood with zero actions, 8 environments per run, with the same check
  code in both runs. With acceleration drives (`--arm_actuator implicit`, run `20260915T095803_471192Z_verify_seed42`),
  J2/J3/J5 held 0.013/0.080/0.110 rad off target and Link6 sat 35.5 mm from where the target angles put it. With force
  drives (`d1_servo`, run `20260915T095752_008164Z_verify_seed42`) they held 0.003/0.010/0.0002 rad, 4.1 mm. PhysX's
  joint loads were 1.20/1.10/0.29 N·m in both, below the 3.3/1.7/1.7 N·m limits. Joint friction and armature are
  zero. Reseating the arm exactly on target while the robot stands returns it to the same sag within 0.2 s. PhysX
  scales an acceleration drive's gains by the joint's effective inertia, so 4000 N·m/rad behaved like roughly 90,
  14 and 3 N·m/rad at J2, J3 and J5.
- **Scope:** static hold at the arm's zero pose on a standing robot. Dynamic tracking was not compared. The remaining
  0.010 rad at J3 with force drives (about 105 N·m/rad effective) is unexplained. Playback (`run_sim.sh`) and Rescue's
  training still use the import's acceleration drives; whether that affected F-008 is untested.
- **Implication:** `--arm_actuator d1_servo` (default) now authors force drives, and `implicit` keeps the import for
  comparison. F-002's droop figures belong to acceleration drives and do not describe the task. The first PPO pilot
  (`20260915T094508_663721Z_train_seed42`) predates the fix.

### F-011 — The target box is reachable, but it starts next to the arm: 14% of targets are already within 5 cm

- **Status:** superseded
- **Superseded by:** F-013, below (2026-09-15: these start distances assumed the base at the environment origin, but every
  reset slides the robot 7.4 cm back; the controlled point is now the pincer tip)
- **Week:** 1
- **Date:** 2026-09-15
- **Evidence:** [workspace figure](week_01/figures/d1_workspace.png) and
  [d1_workspace.json](week_01/figures/d1_workspace.json) from `python -m position_only.workspace`. The kinematics
  and masses were validated in simulation (F-009). At the arm's zero pose, where every episode starts, Link6 is at
  (0.280, −0.001, 0.500) m in the base frame. At the zero-action stance height of 0.266 m that puts it inside the
  top of the box. Distance from there to box targets: mean 8.3 cm, range 0.4–15.5 cm, 14% within 5 cm. At a
  0.30 m base height it is 5%, and 0% from 0.34 m. At every base height from 0.22 to 0.38 m, at least 99.9% of the
  box is reachable, clear of the body proxy and statically holdable. Zero-action smoke runs ended 4.4–11.7 cm from
  their targets. Over pilot 1's last 20 iterations the training-time episode-end error was 6.5–15.7 cm (8.1–21.5 cm in pilots 2 and 3).
- **Scope:** Link6 origin, not a tool point: the fingertips are 12.5 cm further along Link6's z axis. Static poses; a
  box-shaped collision proxy rather than meshes; base level and at the environment origin.
- **Implication:** as configured, part of reaching success is free and most of the 15 cm coarse reward needs no
  arm motion. The box also never asks the base to move. Before G1a: move or widen the box, decide the tool point, and
  report a zero-action baseline next to every reach metric. The pilots' reach metric cannot be told apart from
  zero actions.

### F-012 — Playback reference: the welded D1 leaves forward and lateral walking intact but doubles tilt when turning at 1 rad/s

- **Status:** provisional
- **Week:** 1
- **Date:** 2026-09-15
- **Evidence:** [Week 1 log, 2026-09-15](week_01/notes.md): six runs of `./run_sim.sh --headless --no_ros2 --selftest 10
  --selftest_command VX VY WZ`, walking checkpoint `model_7850.pt`. Mean base velocity over 10 s as a share of
  the command, then max tilt, Go2+D1 against bare Go2: forward 1.0 m/s 96% / 7.8° against 97% / 8.0°; lateral
  0.5 m/s 87% / 6.4° against 82% / 5.6°; yaw 1.0 rad/s 92% / 31.2° against 78% / 16.1°. While turning, the arm
  robot's base dropped to 0.227 m (bare 0.323 m) and drifted at 0.15 m/s (bare 0.04 m/s). This reproduces F-001's
  forward result (95%, 7.3°).
- **Scope:** one 10 s run per command, one robot and one checkpoint, trained without the arm (unitree_go2_rough,
  DCMotor legs). The arm runs on the import's acceleration drives, holding an IK target. Max tilt is a single
  extreme value.
- **Implication:** a welded arm does not break forward or lateral walking, but turning is where it shows first.
  G1b's gait metrics should include yaw at more than one rate (as F-008 does). Playback's arm drives differ from the
  task's (F-010), so this is a reference for the walking checkpoint, not for P0.

### F-013 — With the pincer tip as the controlled point, zero actions still meet the 5 cm criterion for 13% of targets, because every reset slides the robot 7.4 cm back

- **Status:** confirmed, with its mechanism corrected by [F-014](#f-014) (2026-09-16: the slide is the
  zero-action posture settling, not the drop, and the 2.9 cm figure was the settling transient over time
  rather than variation between environments). The 12.9% measurement stands, and the box it describes was
  replaced the same week ([F-016](#f-016))
- **Week:** 1
- **Date:** 2026-09-15
- **Evidence:** [Week 1 log, 2026-09-15](week_01/notes.md). The controlled point is now the tip of the Link7_1 pincer:
  the centre of its end face, at (0.0547, 0.0060, 0.0170) m in Link7_1's frame, from the CAD mesh. `verify` passes
  21/21 (run `20260915T104104_347118Z_verify_seed42`). The command term's point matches URDF kinematics at the
  simulator's arm and jaw positions to 0.9 µm. Zero-action baseline (run `20260915T104334_162270Z_smoke_seed42`,
  256 environments, measured at 9 s with no resets): final error 1.2–14.8 cm, mean 8.2 cm, with 33 of 256 (12.9%;
  Wilson 95% interval 9.3–17.6%) within 5 cm. After the drop from 0.42 m the base settles 7.5 cm behind the
  environment origin (x −0.064 to −0.095 m, y 0.000 m, yaw 0.0°). That puts the pincer tip at (0.331, −0.014,
  0.766) m, inside the box. The workspace model with that stance predicts 13.1% within 5 cm and a mean of 8.7 cm.
  With the base at the origin it had predicted 0.5% and 13.2 cm. For Link6 at the measured stance it gives 1.8%
  and 12.3 cm, not F-011's 14% and 8.3 cm. Addendum, same day: the 42-episode viewer replay (16 robots,
  run `20260915T105401_574747Z_view_seed42`) gave 111 of 672 (16.5%; 13.9–19.5%). Pooled with the 256-robot run,
  144 of 928 (15.5%).
- **Scope:** zero actions only, one snapshot at 9 s. The robot is static by then, but this is not G1a's 1 s dwell
  test. The pincer tip is CAD-derived, not measured on the arm. One set of sampled targets, default configuration
  (force arm drives, self-collisions on). PPO pilots 1–3 used Link6.
- **Implication:** switching to the pincer tip did not remove the free successes, because the reset slide carries the
  tip into the box. Before G1a, move the box or change the reset height, and report the zero-action baseline (12.9%
  here) next to every reach metric. Workspace estimates must use the measured stance, not the environment origin.

### F-014 — The robot's backward slide at reset is the zero-action posture settling, not the drop from 0.42 m

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). A spawn-height sweep (8 envs, 250 steps, zero
  actions) at 0.42 / 0.34 / 0.32 / 0.30 / 0.28 m: peak tilt 18.6 / 8.4 / 6.7 / 4.3 / 4.3 deg, settled base x
  −7.5 / −4.9 / −5.5 / −5.5 / −5.4 cm. Spawning at 0.28 m, essentially the settled height of 0.274 m, the
  base still overshoots to −8.8 cm before recovering to −5.4 cm. Two 16-env, 12 s probes with the time limit
  disabled (scratchpad, not run folders) give the converged stance: at 0.42 m, base x −7.461 cm, height
  0.2667 m, tilt 1.30 deg, settled by 2.6 s; at 0.30 m, −5.551 cm, 0.2737 m, 1.03 deg, settled by 3.6 s. The
  spread of settled base x **across environments** is 0.22 cm (sd) at 0.42 m and 0.005 cm at 0.30 m; across
  time in the last 2 s it is 0.005 cm and 0.020 cm.
- **Scope:** zero actions, flat ground, `--robustness none`, one seed, the default leg model. Why the posture
  is not a static equilibrium in x is not established; the default Go2 joint angles are front thigh 0.8 and
  rear thigh 1.0 rad, so the stance is not front-to-back symmetric. No policy was acting.
- **Implication:** lowering the spawn does not remove the slide, so F-013's explanation of the free successes
  was wrong in mechanism while right in effect. The slide has to be designed around rather than removed: the
  base settles 5.6 cm behind where it spawns and takes 3.6 s of a 10 s episode to get there, moving the robot
  relative to a world-fixed target. It is repeatable between environments to 0.005 cm, so it is a fixed offset
  rather than a source of variance. F-013's 2.9 cm "spread" was the settling transient over time.

### F-015 — The CPU workspace model puts the resting pincer tip 1.6 cm from where the simulator rests it

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). At the measured zero-action stance the workspace
  model (URDF forward kinematics at the arm's zero pose, base treated as level) predicts the pincer tip at
  (0.3502, −0.0139, 0.7732) m from the environment origin. A 16-env, 12 s settle measures
  (0.3601, −0.0141, 0.7604) m: 1.62 cm away, +0.99 cm in x and −1.28 cm in z. The model has neither the arm's
  residual sag under force drives (F-010, 0.010 rad at J3) nor the base's 1.03 deg resting tilt, which over
  the tip's ~0.49 m lever accounts for about 1 cm of the z error. `verify`'s in-simulator measurement agrees
  with the 12 s probe to 3.1 mm.
- **Scope:** the zero pose on a standing robot under zero actions, one configuration. The model's *kinematics*
  are not in question: F-013 matched the command term's tip to URDF forward kinematics to 0.9 µm at the
  simulator's own joint angles. This is the gap between the commanded pose and the pose the robot holds.
- **Implication:** task geometry that has to be right, such as where the target box sits relative to the
  resting tool point, must be set from a simulator measurement, not from the model. `task_space.py` records
  the measured stance and `verify` fails if the simulator drifts more than 2 cm from it. The model stays
  useful for reachability, where 1.6 cm does not change the answer.

### F-016 — Moving the target box below the resting tip removes the free successes: zero actions score 0 of 256

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). The box moved from
  ((0.24, 0.36), (−0.08, 0.08), (0.66, 0.78)) to ((0.36, 0.48), (−0.08, 0.08), (0.50, 0.62)) m from the
  environment origin, the same 12 × 16 × 12 cm moved forward and down, chosen from a CPU search over box
  centres scoring 0% free successes and ≥99% reachable, then re-scored against the measured stance (F-015).
  Distance from the resting tip to the box: 14.0 / 21.9 / 30.2 cm (min/mean/max) against 0.4 / 9.8 / 18.3 cm
  before; 0.0% within 5 cm against 8.6%; 99.1% of the box reachable, clear of the body proxy and statically
  holdable. Simulator zero-action baseline (run `20260915T235714_363411Z_smoke_seed42`, 256 envs, measured at
  9 s, no resets): final error 13.5–28.7 cm, mean 21.5 cm, **0 of 256 within 5 cm**, 0 failure resets.
  `verify` passes 23/23 (run `20260915T235632_928071Z_verify_seed42`) including two new checks: the minimum
  distance from the measured resting tip to any point of the box is 14.3 cm, and 0 of 8 drawn targets are
  within 5 cm. The spawn height also moved to 0.30 m, which cuts the reset tilt peak from 18.6 to 4.3 deg and
  raises the lowest base height from 0.197 m to 0.260 m, against the `low_base` termination at 0.15 m.
- **Scope:** zero actions only, one snapshot at 9 s, one seed, default configuration. This says what the task
  no longer gives away, not what a policy can do: no policy has been trained against this box, and the three
  Week 1 PPO pilots used the old box, the old spawn and the Link6 control point, so their numbers do not
  carry over. Not the G1a evaluation, which needs the frozen-manifest evaluator and a 1 s dwell. The box is
  the nearest placement that removes free successes while staying reachable, not a validated choice of
  difficulty, and it still never requires the base to move.
- **Implication:** a reach metric measured on this task now measures reaching; every target needs at least
  13.5 cm of tool-point motion. This clears the Week 1 blocker in F-013 and unblocks P0 training that is
  meant to count. The zero-action baseline must still be reported beside every reach metric, and it is now
  0%. Playback (`run_sim.sh`) keeps its 0.42 m spawn, so F-012 is unchanged.

### F-017 — The arm's commanded torque saturates while the robot stands still, because its gains are 4000 N·m/rad against 1.7–3.3 N·m limits

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). Zero actions, settled, 8 environments. Isaac Lab's
  `computed_torque` for Joint2/Joint3/Joint5 is −4.46 / −25.80 / 2.87 N·m; `applied_torque`, which is that
  estimate clipped to the effort limit, is −3.30 / −1.70 / 1.70 N·m against limits of 3.3 / 1.7 / 1.7 N·m. Four
  of six arm joints sit at the clip. PhysX's own incoming joint reaction torques at the same instant are
  1.10 / 1.08 / 0.29 N·m, reproducing F-010's 1.20 / 1.10 / 0.29. J3's 0.011 rad residual times 4000 N·m/rad is
  44.9 N·m, less 400 × 0.048 rad/s of damping, giving the 25.8 N·m demand. The arm is an `ImplicitActuator`,
  whose `compute()` stores "approximate torques … since PhysX does not expose this quantity explicitly".
- **Scope:** one static pose, zero actions, default gains, `--arm_actuator d1_servo` force drives. The two
  quantities are not comparable: `applied_torque` is Isaac Lab's Python-side PD estimate, and the reaction
  torque is PhysX's constraint force at the joint, not the drive torque either. Nothing here measures what a
  real D1 servo does; 4000/400 is a modelling choice standing in for its internal loop (F-002, F-007).
- **Implication:** an "effort saturation" metric built on `applied_torque` would report 100% saturation for a
  motionless robot, which says nothing about a policy. The evaluator reports
  `arm_commanded_effort_at_limit_frac` named as a commanded figure, with the caveat carried in `eval.json`,
  beside `peak_arm_joint_torque_nm` from PhysX. The legs use an explicit actuator, so their figures are the
  model's own clipped output and carry no such caveat. More broadly the arm is torque-limited rather than
  PD-tracked in this model, so its effective bandwidth is set by the 1.7–3.3 N·m limits, not by the gains;
  Thesis C actuator identification should replace both.

### F-018 — The frozen-manifest evaluator measures 0 of 300 zero-action episodes reaching, across three balanced manifests

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). `run_position_only.py eval` over
  `development` / `validation` / `test` manifests, 100 episodes each, zero actions, deterministic, 50
  environments per batch (runs `20260916T005258_970617Z`, `…5318_737159Z`, `…5338_474757Z`). Success
  0/100 in each (Wilson 95% 0.0–3.7%), 0 falls, 0 truncated, all 300 surviving to the 10 s limit. RMS error
  21.7 / 21.6 / 22.2 cm, 95th percentile 23.1 / 23.1 / 23.6 cm, final-2 s mean 21.7 / 21.6 / 22.1 cm, lowest
  base 0.260 m, peak tilt 5.11°. No episode entered the 5 cm radius at any step. Manifests are hashed and
  their conditions checked against the run: a deliberate mismatch run at the old 0.42 m spawn with `implicit`
  arm drives was flagged on both counts and reported RMS 16.6 cm, 5 cm *better* than the correct
  configuration, because a sagging arm falls toward a box that sits below it.
- **Scope:** zero actions only, one simulator seed, `--robustness none`. This measures the task and the
  evaluator, not a policy: no checkpoint has been evaluated, so the truncation and fall paths have been
  exercised only by unit tests (`tests/test_evaluate.py`), not by a real failing episode. Per-seed spread
  across training seeds is a G3 item and is not done.
- **Implication:** G1a now has the measurement it requires, and the zero-action reference it must be read
  against is 0% on every manifest. The three sets agree to 0.6 cm of RMS, so development, validation and test
  are balanced rather than three difficulties, and checkpoint selection on `validation` cannot leak into the
  `test` figure. Any future reach claim reports success rate with failed episodes in the denominator, the
  zero-action baseline beside it, and the manifest hash it was measured on.


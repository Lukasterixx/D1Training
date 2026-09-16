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


### F-019 — The first policy trained against the revised box reaches by squatting to within 9 mm of the fall termination

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). Training
  (`train --headless --num_envs 2048 --iterations 1500 --seed 42`, run `20260916T013817_908786Z`):
  73,728,000 transitions, 13 min 27 s, 88,020 steps/s, peak GPU 4,673 MiB; episode-end reach error
  0.60 cm at iteration 1500, every episode reaching its time limit from iteration 300. Frozen-manifest
  evaluation of `model_1499` on `development` (`e2e0d3e6a566…`, run `20260916T015224_579606Z`, no
  condition mismatches): **100/100 successes** (5 cm, 1 s continuous dwell, surviving to the end;
  Wilson 95% 96.3–100%), 0 falls, 0 truncated, RMS 2.11 cm, 95th percentile 1.37 cm, final-2 s mean
  0.52 cm, mean time to reach 0.135 s, mean max dwell 9.85 s. Against the same manifest zero actions
  score 0/100 at RMS 21.7 cm (F-018). Posture: **every one of the 100 episodes** drops the base below
  0.20 m, median 0.172 m, minimum 0.159 m, against the `low_base` termination at 0.15 m — a 9 mm worst
  margin and 22 mm median. Correlation of lowest base height with target height **+0.50** and with
  target distance **−0.50**; lowest-half targets average 0.169 m of base height against 0.175 m for
  the highest half. Peak tilt 17.4° (limit 45.8°). Legs at their effort limit for 9.8% of steps on
  average and up to 30.1% in one episode, peak 23.4 N·m, the Unitree envelope's Y2 value (F-005).
  The reward set has no base-height term: `upright` (`flat_orientation_l2`, −1.0) penalises tilt,
  `base_motion` (−0.2) penalises velocity, `alive` is +0.5 and `failure` −2.0.
- **Scope:** one seed, one manifest, `--robustness none`, default configuration, deterministic policy
  actions. `development` is the manifest that may be inspected, so this is a development number and
  not a reported result. Three seeds (G1a) and matched budgets (G3) are not done. Arm joint speed
  limits (1.05/1.73 rad/s) and the 10 Hz arm command hold are the unverified estimates of F-007, so
  the claim that the body does the early work rests on those limits being roughly right. No hardware.
- **Implication:** the revised box of F-016 is learnable, and the evaluator now works against a
  checkpoint rather than only zero actions. But the numeric G1a criterion is met by a posture that
  cannot serve as the P0 baseline: a robot holding 9–22 mm above its fall threshold with legs
  saturated for up to a third of an episode will not meet G5's "no falls or limit violations" on
  hardware, and it makes P0's reaching substantially leg work, confounding the P0–P4 comparisons that
  are supposed to isolate force awareness and tool use. Squatting is not itself disallowed — the plan
  permits stance and posture changes — so the fix is to price it, not to forbid it: add a base-height
  reward term or nominal-height penalty, and re-measure. Until then G1a stays "not started" and no
  reach number from this policy is quoted as a baseline. Report the zero-action reference (0/100)
  beside any figure taken from it.

### F-020 — The D1 publishes joint angles at 9.00 Hz (111 ms), not the 10 Hz the timing model assumes

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [D1 arm telemetry, read-only (120 s)](#/week/1/run/20260916T043123_d1_telemetry), measured on the
  physical arm over CycloneDDS from the Go2's Jetson payload (`enP8p1s0`, arm at 192.168.123.100), with the
  arm unpowered and released. Over 120 s, `current_servo_angle` delivered 1080 samples at
  **8.9986 Hz, median period 111.016 ms, stdev 0.516 ms** (min 110.18, max 122.38). A separate 45 s capture
  split `rt/arm_Feedback` into its two streams: `funcode 1` (the seven joint angles) at **8.9852 Hz,
  111.009 ms, stdev 0.420 ms**, and `funcode 3` (`enable_status`/`power_status`/`error_status`) at
  **9.9614 Hz, 100.455 ms, stdev 0.170 ms**. The angle streams on the two topics share one 111 ms cycle;
  the 10 Hz cycle exists, but it carries status, not angles. Angles are quantised to 0.1° and the resting
  per-joint noise stdev is ≤0.047°.
- **Scope:** one arm, one session, arm at rest and unpowered; rates were not re-checked while the arm was
  moving or under load. Arrival times are host-side (`time.monotonic()` after a DDS waitset wake), so they
  include transport and scheduling; the period's 0.5 ms stdev bounds that jitter. This measures the arm's
  publish cycle, not end-to-end observation age in a control loop.
- **Implication:** `motor_model.py` carries `D1_FEEDBACK_HZ = 10.0` labelled from the SDK headers and never
  measured; the true angle cycle is 11% slower. At the task's 50 Hz policy rate this changes
  `arm_feedback_period_steps` from `round(50/10) = 5` to `round(50/9) = 6`, so the `estimated` latency profile
  has been refreshing simulated arm feedback one policy step sooner than the hardware can. The status cycle
  being a genuine 10 Hz is why the SDK-derived number looked right. Replaces the estimate behind F-007.

### F-021 — J0 answers a step command in ~127 ms and reaches 1.15 rad/s, above the URDF's unverified 1.05 rad/s

- **Status:** provisional
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [D1 J0 step response on hardware](#/week/1/run/20260916T043641_d1_j0_step). With the Go2 sitting
  and the arm folded, J0 (base rotation, limits ±135°, resting 2.0°) was stepped ±5°, ±10° and ±20° about its
  resting angle and returned, six commanded excursions in all. Command→first detected motion (>0.3°, against a
  ≤0.05° noise floor) was **126.3–138.2 ms** on all six, measured independently on both angle topics with
  agreeing results. Peak rate rose with step size: **46.7 °/s (0.815 rad/s)** at 5°, **51.3 °/s (0.895 rad/s)**
  at 10°, **65.9 °/s (1.151 rad/s)** at 20°. Settling took 132.9–352.5 ms, steady-state error was ≤0.10°, and
  the joint dithers ±0.1° while holding. `funcode 6 power=1` was acknowledged in 79.3 ms and
  `funcode 5 mode=0` (release) in 99.6 ms.
- **Scope:** **one joint, one posture, no payload.** J0 rotates the folded arm about the base, the lightest
  inertia case; nothing here transfers to J1/J2 under gravity or to a loaded gripper. Both the latency and the
  velocity are bounded by the 111 ms feedback cycle of F-020: a 20° move spans only ~3 samples, so 1.15 rad/s
  is a **lower bound** on peak speed and 127 ms an **upper bound** on latency (true latency is that minus up to
  one sample period). Peak rate still climbing at the largest step means the joint had not saturated, and
  20° was the agreed ceiling for this session, so the actual speed limit was not reached.
- **Implication:** `D1_VELOCITY_LIMIT_RAD_S` carries 1.05 rad/s for the first three joints from `d1_arm/d1.urdf`
  with "no recorded source" (F-007). The first hardware number for J0 is already above it and not yet saturated,
  so the URDF value is more likely an underestimate than a conservative bound — simulated arm motion may be
  slower than the hardware allows. The ~127 ms command→motion delay is a transport-and-controller lag that the
  current `estimated` profile does not model at all: it models a command *hold* (`arm_command_hold_steps`), not a
  *delay*. Do not update the velocity limits from a single joint; measure J1–J5 with larger steps before changing
  the number. See [F-022](#f-022) for why this was measured with a manual restore rather than a power-off.

### F-022 — The arm cannot be powered off over DDS, and enable is implicit: the documented emergency stop does not work

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [D1 J0 step response on hardware](#/week/1/run/20260916T043641_d1_j0_step), status stream sampled
  at 10 Hz throughout. `funcode 5 {"mode": 1}` ("enable all", per the driver and
  `~/d1Arm/src/joint_enable_control.cpp`) **never set `enable_status`**: sent alone with power already on, and
  again as `funcode 4 {"id": 0, "mode": 1}`, the status stayed 0. `enable_status` went 0→1 at t=8.108 s, 100 ms
  after the first `funcode 1` motion command at t=8.010 s — the arm enables itself when a motion command
  arrives. Release (`funcode 5 {"mode": 0}`) does work, clearing the flag in 99.6 ms. `funcode 6 {"power": 1}`
  set `power_status` in 79.3 ms, but `funcode 6 {"power": 0}` was ignored in three separate attempts, the last
  sending **14,208 consecutive power-off commands over 5 s with `power_status` never leaving 1**.
- **Scope:** one arm and one firmware build, over DDS from the Go2 payload (192.168.123.222). The driver's
  README states the arm firmware expects the PC at 192.168.123.162; that was not satisfied here, yet power-on,
  release and all motion commands were honoured, so a source-address rule does not explain power-off being
  dropped. A physical power cut was not attempted. Whether a host at .162 can power the arm off is untested.
- **Implication:** `unitree-d1-control/src/arm_control.py` documents `power(on=False)` as "Can be used as an
  emergency stop", and the GUI exposes it as an emergency power-off button. On this hardware that path is
  inert, so **there is currently no software stop for the D1** — the working stop is release
  (`funcode 5 {"mode": 0}`), which drops torque but leaves the motors powered and the arm backdrivable, or a
  physical power cut. Any hardware protocol for Thesis B (G4 onward) must treat release, not power-off, as the
  software abort, and must not rely on a software stop as the only guard. The implicit-enable behaviour also
  means a stray `funcode 1` message energises the arm with no separate arming step, which matters for anything
  that shares this DDS domain. Flagged to Lukas for the VIP-Rescue driver, which is shared with that project.

### F-023 — The arm's resting pose lies outside the joint limits its own driver enforces

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [D1 arm telemetry, read-only (120 s)](#/week/1/run/20260916T043123_d1_telemetry): over 120 s the
  folded arm reported a stable resting pose of **[2.1, −90.9, 91.7, −3.1, 4.1, 3.1, 40.8]°** (J0–J5 plus the
  gripper), each joint varying by ≤0.2° across the whole capture. `JOINT_LIMITS_DEG` in
  `unitree-d1-control/src/arm_control.py` gives J1 as (−90, 90) and J2 as (−90, 90), so the measured J1 (−90.9°)
  and J2 (+91.7°) sit 0.9° and 1.7° **outside** the limits the driver validates against, and
  `move_joint`/`move_joints` raise `ValueError` for either value.
- **Scope:** the arm's own encoder readings at one resting posture, with the Go2 sitting; no independent
  measurement of true joint angle, and no check of whether the offset is a zero-point calibration error, a
  mechanical hard stop beyond the soft limit, or a genuinely wrong limit table. Only the resting pose was
  examined, so the other end of each joint's range is untested.
- **Implication:** commanding the arm back to the pose it physically rests in is rejected by the driver, so any
  "return to rest" built on `move_joints` fails for this arm, and a full-pose command that includes J1/J2 near
  their limits cannot be expressed. The same ±90° figures appear in the D1 spec table this repository quotes.
  Before the weld geometry or workspace maths (F-011, F-013, F-015) is checked against hardware, the zero-point
  and true range of J1/J2 need measuring — the simulated arm's limits come from `d1_arm/d1.urdf`, which may
  carry the same offset.

### F-024 — There is no higher-rate mode to unlock on the D1: the 111 ms cycle is the interface, and the one public attempt to push past it ended with a dead arm

- **Status:** confirmed (for the interface surface); the failure history is **reported, not reproduced**
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** DDS discovery against the live arm ([run](#/week/1/run/20260916T043123_d1_telemetry), `dds_topics.txt`,
  via the builtin `DCPSPublication`/`DCPSSubscription` readers, 36 participants) enumerates the arm's **complete**
  topic surface: `rt/arm_Command`, `rt/arm_Feedback`, `current_servo_angle`, `arm_zero`, `set_servo_angle`,
  `set_servo_angle_control` (all `SetServoAngle_` = `{seq, id, angle, delay_ms}`) and `set_servo_dumping`
  (`SetServoDumping_` = `{seq, id, power}`). **No `rt/arm_sdk` and no `arm_LowCmd`** — the per-joint kp/kd topic
  that `unitree-d1-control/haptic_control.md` hypothesised does not exist on this firmware. Watched read-only for
  20 s with the arm idle, all four struct topics were **silent**, so there is no hidden internal high-rate stream.
  Published sources agree on 10 Hz and document no alternative: the
  [D1 SDK extension](https://github.com/2Nitrogen/unitree_d1_sdk_extension) states `current_servo_angle` and
  `rt/arm_Feedback` publish at 10 Hz, and the Caltech SURF report
  ([Zeng 2025](https://eloisezeng.people.caltech.edu/documents/32338/2025_SURF_Final_Report.pdf)) records Unitree
  stating "10 Hz is the control cycle of the arm". That report also tried exactly this: commands at **100 Hz
  "appeared to move more smoothly"**, and editing the mode-0 `execution_time` from 0.04 s to 0.1 s and 0.01 s did
  not cure the shaking. Within minutes, at every rate tried, their arm stopped reaching commanded positions and
  then stopped responding; servos ran "burning hot" on the Go2's own supply, and they concluded the arm was
  defective and unfixable. Unitree's support told them the D1 R&D team had been lost and its data deleted; the D1
  is discontinued. Against all that, our own arm's angle cycle is a **hardware-fixed 111.0 ms, stdev 0.52 ms**
  over 120 s (F-020) — a firmware cadence, not a negotiable DDS QoS setting.
- **Scope:** one arm, one firmware, read-only discovery; nothing was published to the struct topics and **no
  high-rate command test was run here**. The Caltech failures are one group with one arm they independently judged
  defective, so they do **not** establish that high-rate commanding damages a healthy D1 — but they are the only
  public account of trying, and the outcome was total loss of the arm. Port 22 is open on the arm (192.168.123.100)
  and the same report describes SSHing in to edit command execution time, so an on-device configuration surface
  probably exists; it was not explored, and its contents and risks are unknown. `set_servo_dumping` is
  undocumented and untested.
- **Implication:** plan for **9 Hz observation**, not for a faster arm. The command side is not rate-limited —
  anything can be published faster, and 14,208 commands in 5 s were accepted without complaint (F-022) — but
  **feedback stays at 111 ms regardless**, so closed-loop arm control is capped by the observation rate no matter
  how fast commands go. The genuine knobs are per-command, not per-rate: `delay_ms` on funcode 1, `execution_time`
  and `mode` 0/1 on funcode 2 ("small smoothing of 10 Hz data" versus "large smoothing of trajectory-use"). These
  shape a single move and are the right place to look for smoothness. Given a discontinued arm, welded to the dog,
  with no replacement path and one public account of it dying under high-rate commanding, raising the command rate
  is not worth the risk to this thesis: the sim-to-real contract should hold the arm at the rate the hardware
  actually publishes. This supports, rather than weakens, the F-020 change to `D1_FEEDBACK_HZ`.

### F-025 — A Cartesian (IK) controller now drives the real arm's protocol, but its Cartesian accuracy is unvalidated

- **Status:** provisional
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [D1 IK controller bring-up](#/week/1/run/20260916T0500_d1_ik_bringup). `d1_ik.py` is a damped
  least-squares solver in plain numpy built on the FK already in `position_only/workspace.py`, so the task, the
  tests and the hardware share one kinematic model; `forward()` returns each joint's axis and origin in the base
  frame, which makes the geometric Jacobian analytic rather than a finite difference. `d1_hardware.py` speaks the
  arm's measured protocol and mirrors `DirectD1`'s client seam, so the simulator's controller and the hardware
  differ only in transport. 19 new tests (88 in the full suite, all passing): the analytic Jacobian matches
  central differences to 1e-6 (linear) and 1e-5 (angular); position-only IK converges on 20+/25 random reachable
  targets and full-pose on 12+/15; solutions always lie inside the URDF soft limits, including for targets 1.5 m
  outside the workspace; unreachable targets report failure without NaN; every commanded step respects the step
  cap. Against the arm itself, read-only: FK of the measured resting pose [2.10, −90.90, 91.70, −3.10, 4.10,
  3.10]° puts the tool point at **[0.1366, −0.0097, 0.2069] m** in the Go2 base frame, and a dry-run approach to
  [0.30, 0.0, 0.30] planned in 8 iterations to 0.10 mm and rehearsed as **15 bounded 5° steps** converging to
  0.05 mm. As a cross-check on the FK, the zero-pose tool point sits 1.6 cm from the simulated resting tip —
  exactly the offset F-015 already attributes to arm sag and base tilt.
- **Scope:** **nothing here was executed on the arm; it did not move.** Every number above is either a software
  test or forward kinematics of a measured joint vector. The solver's accuracy is the URDF's accuracy, and it
  rests on an assumption that is **not established**: that servo *i* reports URDF `Joint{i+1}` with the same zero
  and sign. The URDF's per-joint limits match the driver's table, which is consistent with it, but the arm rests
  outside both (F-023), so the zero point is unknown and a constant joint offset would move every Cartesian
  number without failing any test here. No external measurement of the tool point exists — that needs the camera
  and tag work (G4), which is not started. The FK also models a rigid arm: the real one sags, which is the
  mechanism behind the 1.6 cm of F-015. The `clear_of_body` proxy assumes the Go2's **standing** trunk box, so it
  does not describe clearances while the robot is sitting.
- **Implication:** Cartesian targets can now be planned and rehearsed against the arm's real configuration, and
  the same solver is available to the task, so hardware testing no longer needs hand-picked joint angles. Treat
  the output as **joint-space commands that are safe to send**, not as a calibrated Cartesian position: until the
  zero point is measured, "move the tool to (x, y, z)" means "to where the URDF thinks that is". Establishing the
  J1/J2 zero (F-023) is the prerequisite for quoting any Cartesian error on hardware, and is the first thing G4
  needs. Approach moves should keep the step cap: from the folded rest pose an uncapped solution is a single
  large swing, which with the robot sitting is exactly the motion that would hit the dog.

### F-026 — The IK controller moves the real arm; the 6 mm residual is the arm settling, not the solver

- **Status:** superseded
- **Superseded by:** [F-029](#f-029) (2026-09-16, same day: a settle-and-hold test shows the residual does **not**
  settle out, so the "settling lag" explanation below is wrong. The measurement that the solver is not at fault,
  and the 0.7 deg joint tracking error, both stand)
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [D1 IK controller: first execution on hardware](#/week/1/run/20260916T0530_d1_ik_execute). With the
  Go2 sitting, the Cartesian controller was given a target 11.3 cm up and forward of the folded rest pose —
  chosen so clearance did not depend on the sitting posture, which the `clear_of_body` proxy cannot model. The
  arm tracked it: tool point **[0.1366, −0.0097, 0.2069] → [0.1984, −0.0013, 0.2943] m**, an achieved
  displacement of **107.4 mm against 113.0 mm commanded**, through bounded 3° steps with the error falling
  monotonically. It then stopped improving at **6.03 mm** and ran all 200 cycles. Re-solving from the measured
  pose shows the solver is not at fault: IK converges to **0.025 mm** and asks for
  [+0.31, +0.26, −0.72, −0.06, −0.42, +0.03]°, while the arm had landed up to **0.7° from the last command**
  (commanded J2 68.2°, measured 68.9°). The return leg then showed *why*: parking to a fixed joint vector, with
  the command held **constant**, the worst joint error fell **3.00 → 2.20 → 1.20 → 0.40°** over four consecutive
  111 ms cycles. The arm was still settling. The Cartesian loop re-solves every cycle, so it re-commands before
  the arm has arrived, and the residual is substantially **settling lag rather than a fixed deadband**.
- **Scope:** one target, one direction, one arm, unloaded, with the robot sitting and the arm near the folded end
  of its range. 6.03 mm is the residual of *this* loop at *this* pose, not a spec for the arm: the settling
  constant was measured incidentally on the return leg, not in a designed step-and-hold test, and no test held a
  Cartesian command still to see where it eventually settles. The Cartesian numbers are FK of measured joint
  angles, so they inherit the unvalidated zero point of F-025 and F-023 — the *displacement* is better evidence
  than the absolute position, since a constant joint offset largely cancels in a difference. No external
  measurement of the tool point exists.
- **Implication:** Cartesian control of the physical arm works and is safe to use for testing, with the step cap
  as the guard. For accuracy, the loop should **hold and let the arm settle** rather than re-solve continuously —
  a settle-then-measure step is worth more than a tighter tolerance, and a tolerance below about 1 cm is not
  meaningful for a continuously re-solving loop at this rate. This is well inside G1a's 5 cm reach criterion, so
  it does not threaten the position-only gate, but it does set the floor for anything finer, and G6's force work
  will need the settling behaviour characterised properly. Two defects were found by running this and are fixed:
  the wire clamp used the solver's *soft* limits, so the first command from the −90.9° rest pose would have
  jumped 9.9° and silently defeated the step cap; and the approach had no stall detection, so it spent 186 cycles
  commanding a correction the arm could not take. Both now have tests.

### F-027 — The arm's folded rest pose is a mechanical stop outside its commandable range, not an encoder offset

- **Status:** provisional
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [first execution on hardware](#/week/1/run/20260916T0530_d1_ik_execute). Parking the arm back to
  the folded pose could only command J1/J2 to **−89.9°/+89.9°**, the URDF hard limits, because the measured rest
  pose (−90.9°/+91.7°) lies outside them (F-023). After the command was released, the arm **settled past the
  commanded values** to **−90.8°/+91.6°** — back to where it had been resting all session, to within 0.1°. The
  tool returned to 1.1 mm of its original position.
- **Scope:** one observation, on release, at one end of two joints' travel. Nothing here measures the true joint
  angle, and the other end of each range is untested. "Mechanical stop" is inferred from the arm settling
  repeatably to the same place with no torque commanded; it was not confirmed by touch or by the vendor.
- **Implication:** this makes an encoder zero-point error less likely than a limit table that is simply 1–2°
  tighter than the mechanism, which is the better news of the two for F-025: it suggests URDF angles and reported
  angles share a zero, and that the Cartesian numbers are not carrying a hidden constant offset. It is **not
  proof** — G4 still needs the zero measured externally. Practically: a "return to rest" cannot be commanded, only
  approached to the limit and then released, and `approach_joints` exists because IK cannot express a pose outside
  the joint limits at all.

### F-028 — Releasing the D1 drops it: there is no holding brake, and the abort path is a fall

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [D1 IK controller, second target](#/week/1/run/20260916T0600_d1_ik_faster). After a Cartesian move
  the arm settled, measured and holding, at **[19.60, −30.10, +57.70, −4.30, −12.60, +2.80]°**. `d1_hardware.py`
  then published `funcode 5 {"mode": 0}` (release) as its normal end-of-move action. The next command's opening
  feedback read **[19.60, −90.90, +92.80, −1.20, +0.30, +3.30]°**: with nothing commanding it, **J1 fell 60.8°
  and J2 35.1°**, the arm dropping under gravity to its folded mechanical stop. Lukas saw it drop and said so.
  The arm reports `error_status = 0` and returned to its session-start rest pose within 0.4° per joint and 2 mm
  at the tool, so no damage is evident. Contrast the first execution earlier the same day, where release at
  [3.30, −49.90, +68.90]° left the arm in place: holding on release is **pose-dependent**, not a property of the
  arm, and that earlier observation made it look safer than it is.
- **Scope:** one drop, from one pose, on one arm. The threshold between "settles" and "falls" was not measured —
  `is_safe_to_release()` now uses J1 ≤ −80° and J2 ≥ +80° as a conservative folded test, which is a guess bounded
  by two observations (fell from J1 = −30°, held at J1 = −50°), not a measurement. Whether the fall damaged
  anything is not established by `error_status` alone; nothing was inspected mechanically, and it is not known
  what the arm passed through on the way down.
- **Implication:** this refines [F-022](#f-022), which called release "the working stop". It works, but it is a
  **torque-off, not a hold** — the D1 has no brake, so on an extended arm the software stop *is* a controlled
  fall. Combined with power-off being ignored, **the D1 has no software stop that holds position at all.** For
  G5's "abort/hold behaviour demonstrated" that is the key constraint: an abort must either leave the arm
  energised and holding, or be planned as a fall from a pose where a fall is acceptable. The direct cause here
  was mine: `d1_hardware.py` released at the end of every `--execute` path, which is exactly backwards —
  releasing is an emergency action, not a way to finish a move. Fixed: the CLI now leaves the arm holding and
  says so, `release` warns and refuses on an unfolded arm without `--force`, and three tests pin the guard
  against the two poses actually observed. Any future hardware protocol should treat "leave it energised" as the
  default end state and park the arm low before releasing.

### F-029 — The ~6 mm Cartesian residual is a persistent offset with dither, not settling lag

- **Status:** superseded
- **Superseded by:** [F-031](#f-031) (2026-09-16, same day: the residual and the 0.5-0.7 deg offset were
  artifacts of commanding funcode 2 **mode 1**, a slow interpolator. On mode 0 the same joint settles to 0.20 deg
  in 538 ms. The observation that holding does not close the gap stands; its explanation does not)
- **Week:** 1
- **Date:** 2026-09-16
- **Supersedes:** [F-026](#f-026)
- **Evidence:** [D1 IK controller, second target](#/week/1/run/20260916T0600_d1_ik_faster). A second, larger and
  faster move — 184 mm of travel to [0.25, 0.08, 0.32] m with a lateral component, at 6°/step (~54°/s) against
  3°/step before — reached its 5 mm tolerance at **4.78 mm**. A settle-and-measure step then **stopped commanding
  entirely for 3 s** and watched. The error did not decay: it oscillated between **5.93 and 6.68 mm** across 17
  feedback samples and finished at **6.27 mm**, worse than the 4.78 mm the loop ended on. The settled joints
  [19.60, −30.10, +57.70, −4.30, −12.60, +2.80]° sit up to **0.7°** from the last command
  [20.1, −30.1, +57.0, −4.2, −12.9, +2.8]° — the same joints and the same magnitude as the first move, which
  reached 6.03 mm. So the offset is repeatable and does not wash out with time.
- **Scope:** two targets, one arm, unloaded, both in the same general region of the workspace; the oscillation
  band is 17 samples at one pose. The Cartesian figures are FK of measured joint angles and inherit the
  unvalidated zero point (F-025), though a constant joint offset largely cancels in the *displacement*. What
  F-026 read as settling was real but different: on the park leg the arm was closing on its folded **mechanical
  stop**, where it does keep creeping in; in free space it does not.
- **Implication:** the arm holds a commanded pose to roughly **0.5–0.7° per joint, with a dither of about
  ±0.1°** (consistent with the hold dither in F-021), which is **~6 mm at this reach** and does not improve by
  waiting. Treat ~6 mm as the practical Cartesian floor for open-loop joint commands to this arm, not a number to
  be tuned away: holding longer does not help, and tightening the tolerance below about 1 cm only makes the loop
  spin. Closing it would need either an outer loop correcting on a *measured* tool position — which needs the
  camera and tags of G4 — or per-joint offset calibration. Well inside G1a's 5 cm reach criterion, so the
  position-only gate is unaffected; it matters for G6's force work and for any claim about fine positioning.
  Speed did not degrade accuracy: the faster move landed in the same 6 mm band as the slower one.

### F-030 — Servo 0's sign is inverted relative to the URDF: the model's "left" is the arm's right

- **Status:** confirmed (the error); the correction is **provisional** pending a visual check of the corrected move
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [D1 IK: up-and-left target](#/week/1/run/20260916T0630_d1_ik_left). A target at
  **[0.22, +0.16, 0.36] m** — +y, which the URDF frame and this repository's own convention both call the robot's
  **left** (`tool_point.py` anchors it: the Link7_1 pincer sits at negative y and is described as on the robot's
  right) — was solved to servo 0 = **+37.63°** and commanded. Lukas, watching, reported the arm moved to the
  **right**. The same runs reached **forward over the head** and rose as predicted, which is what rules out the
  competing explanation: a 180° mount yaw would have flipped +x as well, putting the arm over the tail. Both
  `weld.py` (`LocalRot0/1 = Quatf(1,0,0,0)`, offset (0,0,0.08)) and `workspace.forward` mount the arm with
  identity rotation, and they agree with each other to the 1.6 cm of F-015, so this is not an FK bug — the model
  is self-consistent and disagrees with the hardware.
- **Scope:** **J0 only.** J1 and J2 are confirmed consistent by the same observation (the arm rose and reached
  forward as predicted). **J3, J4 and J5 are unvalidated** — no motion run so far isolates a wrist joint, so their
  signs remain assumptions, and any orientation target depends on them. The zero *offsets* are still unmeasured
  (F-023, F-025); this finding is about sign, not zero. The pincer side was not confirmed on hardware either, so
  the `tool_point.py` "robot's right" claim, which anchors the y convention, is still CAD-derived. The correction
  has been applied and re-run — servo 0 was commanded **−37.30°** for the same target — but the resulting
  direction has not yet been visually confirmed.
- **Implication:** this is the failure mode [F-025](#f-025) warned about, now observed: servo *i* does **not**
  report URDF `Joint{i+1}` with the same sign. Every Cartesian *y* through the hardware path was mirrored. Fixed
  at the conversion layer: `d1_ik.SERVO_SIGN = [-1, 1, 1, 1, 1, 1]`, applied in `to_servo_deg`/`from_servo_deg`,
  the only two places the mapping is written down. Fixing it exposed a latent unit bug — `step_towards` compared a
  solver result in URDF radians against a measured pose in servo degrees, which is harmless only while every sign
  is +1; it is now servo-space throughout, and joint clamping moved to `servo_limits_deg` because a sign flip
  swaps a joint's low and high. Four tests pin the behaviour, including the exact regression.
  **The wider problem is not fixed:** `position_only/deploy.py` maps `Joint1..Joint6` to motor ids 0–5 **by index
  with no sign convention at all**, so `params/deploy.yaml` — the deployment contract G4 is meant to freeze —
  carries the same error. A policy trained in simulation and deployed through that manifest would mirror its J0
  on hardware. The manifest needs a per-joint sign, and every joint's sign needs measuring before G4 can freeze
  anything; that is left as a deliberate decision rather than changed here, because it alters the contract.

### F-031 — The command path, not the arm, set the accuracy and speed: funcode 2 mode 1 is 5x slower and 25x less accurate than mode 0

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Supersedes:** [F-029](#f-029)
- **Evidence:** [per-joint sweeps](#/week/1/run/20260916T0730_d1_sweeps). Same joint (J0), same 30° amplitude,
  same pose, three command paths:

  | path | peak rate | latency | settle | steady-state error | hold band |
  | --- | --- | --- | --- | --- | --- |
  | funcode 2, mode 1 | 13.5 °/s (0.236 rad/s) | 202 ms | not reached in 3 s | **−5.09°** | 13.1° |
  | funcode 2, mode 0 | **69.3 °/s (1.209 rad/s)** | 93 ms | **538 ms** | **−0.20°** | **0.00°** |
  | funcode 1 | **70.5 °/s (1.231 rad/s)** | 61 ms | **505 ms** | **−0.20°** | 0.10° |

  Unitree documents mode 0 as "small smoothing of 10 Hz data" and mode 1 as "large smoothing of trajectory-use".
  Mode 1 is a slow interpolator: commanding it once per feedback cycle re-commands a waypoint the arm is still
  slewing toward, so it never arrives. Everything previously attributed to the hardware was this — the
  approach taking **99–117 steps instead of the predicted 10–14**, the stalls, and the "0.5–0.7° tracking
  offset". Switching the default to mode 0 cut the same Cartesian move to **13 steps**.
- **Scope:** one joint, one amplitude, one pose for the three-way comparison; J1–J5 were swept only on mode 1
  before the arm stopped responding (F-032), so their speed limits remain unmeasured. A residual **did** survive
  the switch: the loop stalls around 4.95 mm, the arm sitting 0.3–0.7° from the last command, and neither
  silence nor streaming the final command for 3 s closed it — consistent with a small-increment deadband, since
  a single 30° command lands within 0.20°. That was being measured when the arm became unresponsive, so the
  deadband is **hypothesised, not established**.
- **Implication:** F-029's "persistent per-joint offset" was an artifact of the command mode, and its "~6 mm
  practical floor" was measured through a slow interpolator. `set_all_joint_angles` now defaults to mode 0, with
  mode 1 still reachable for comparison and two tests pinning the default. For J0 the measured peak is
  **1.21–1.25 rad/s**, comfortably above the 1.05 rad/s `d1_arm/d1.urdf` carries with no recorded source
  (F-007) — but the URDF value is **not** updated here, because the same comparison for J1–J5 was never taken.
  The general lesson is the one that cost most of this session: **characterise the interface before attributing
  a number to the mechanism.** Three findings (F-026, F-029, and the 6 mm "floor") were explanations of a
  configuration choice.

### F-032 — The arm stopped responding to motion commands while still reporting healthy

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** after a sequence of Cartesian moves and single-joint sweeps, the arm ceased to move for **any**
  commanded step. Sweeps of 0.5, 1, 2, 5 and 10° on J0 all returned peak 0.0 °/s with the joint never leaving its
  start angle, on **both** command paths (funcode 2 mode 0 and funcode 1). Throughout, the arm reported
  `power_status = 1`, `enable_status = 1`, `error_status = 0`, and its 9 Hz angle feedback continued unbroken —
  it looked healthy in telemetry while being inert. The last successful motion was a Cartesian move minutes
  earlier. This reproduces the failure described in the Caltech SURF report (F-024): *"the arm would only
  partially move toward the commanded positions. Shortly after, the arm would not respond to any commands."*
- **Scope:** one occurrence, not yet reproduced or cleared; as of writing the arm has not been power-cycled, so
  it is not known whether the state is recoverable. The trigger is **not established**. A plausible contributor
  is ours: the `settle --hold-stream` loop re-sent `set_all_joint_angles` on every poll rather than pacing to the
  arm's 10 Hz cycle, which is the same class of over-commanding that preceded the Caltech failures. That is a
  hypothesis; the arm had also been commanded almost continuously for roughly an hour, and the Caltech arm failed
  at 10 Hz too, so rate may not be the variable at all.
- **Implication:** **telemetry does not indicate arm health.** `error_status = 0` with live feedback is
  consistent with a completely inert arm, so any hardware protocol must verify motion against *measured
  movement*, not against the status word — G4's "measured timing" and G5's abort demonstration both need a
  liveness check that commands a small motion and confirms it happened. Command pacing is now enforced in the
  client: motion commands (funcode 1 and 2) are rate-limited to 10 Hz, deliberately not applied to release so the
  abort path is never delayed, with two tests covering both. Whether that prevents a recurrence is unknown. Until
  the arm is recovered and this is understood, **hardware sessions should be short, paced, and bounded**, and the
  arm should be parked folded rather than left extended, because an unresponsive arm cannot be brought down and
  removing power drops it (F-028).

### F-033 — Every D1 joint slews at ~1.2–1.3 rad/s, and holding still is twice as steady when the controller stops re-solving

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [hold stability and per-joint sweeps](#/week/1/run/20260916T0800_d1_hold_sweeps), all on the fast
  command path (funcode 2 mode 0, F-031), 30° steps, peak rate from consecutive feedback samples:

  | servo | URDF joint | previous URDF limit | measured peak (rad/s) | measured peak (°/s) |
  | --- | --- | --- | --- | --- |
  | 0 | Joint1 | 1.05 | 1.209–1.249 | 69.3–71.6 |
  | 1 | Joint2 | 1.05 | **1.292–1.293** | 74.0–74.1 |
  | 2 | Joint3 | 1.05 | 1.213–1.231 | 69.5–70.5 |
  | 3 | Joint4 | 1.73 | **1.197–1.210** | 68.6–69.3 |
  | 4 | Joint5 | 1.73 | 1.213–1.247 | 69.5–71.5 |
  | 5 | Joint6 | 1.73 | 1.211–1.245 | 69.4–71.3 |

  Latency was 20–132 ms and settling 505–574 ms across all six. **Holding still**, measured over 20 s in three
  regimes at the same pose, reported as tool-point excursion from the mean:

  | regime | worst joint p-p | tool p-p in z | max excursion |
  | --- | --- | --- | --- |
  | silent (no commands) | 0.20° | 1.480 mm | **0.991 mm** |
  | streaming one fixed pose | 0.10° | 1.457 mm | **1.295 mm** |
  | re-solving IK every cycle | 0.30° | 2.849 mm | **2.686 mm** |

- **Scope:** one arm, unloaded, one posture per joint, and the peaks are **lower bounds** — the 111 ms feedback
  cycle (F-020) means a 30° move spans only about four samples. Hold figures are FK of reported angles, so they
  inherit the 0.1° encoder quantisation; a still arm cannot read better than roughly ±1 mm here, which is why the
  silent and fixed-stream regimes are indistinguishable. Nothing external measured the tool, and the camera
  itself is not mounted, so this predicts steadiness rather than demonstrating it.
- **Implication:** the 1.05/1.73 split was never Unitree data (`d1_description` ships `velocity="0"`) and the
  measurement shows no such split — every joint tops out in a narrow 1.20–1.29 rad/s band, which reads as one
  controller-wide ceiling rather than six mechanical limits. `motor_model.py` now carries the measured values.
  The old numbers were wrong in **both** directions, and the dangerous one is Joint4–Joint6, where simulation
  allowed the arm **40% more speed than the hardware delivers** — a policy trained against that would command
  wrist motion the real arm cannot follow. For the end-effector camera Lukas is mounting: **stop re-solving once
  on target.** Re-solving feeds the encoder's 0.1° quantisation back into the command and roughly doubles tool
  motion, while going silent or streaming one fixed pose both sit at the measurement floor. The controller
  already stops commanding when the approach ends, so the rule is to keep it that way and never leave a
  re-solving loop running under a camera. This is also what Lukas saw by eye while the arm was unresponsive
  (F-032): it was steadier precisely because nothing was commanding it.

### F-034 — Two of the six servo signs are inverted, and the model had J3 and J5 counter-rotating when they roll together

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [wrist sign validation](#/week/1/run/20260916T0830_d1_wrist_signs). Each joint was stepped +30°
  alone from a raised pose, with the model's Jacobian-derived prediction stated **before** the move, and Lukas
  watching:

  | sweep | model predicted | observed | verdict |
  | --- | --- | --- | --- |
  | J0 | swings ~113 mm to the dog's right | swivelled **right** | matches (confirms the F-030 fix) |
  | J3 | rolls the forearm **left side up**, ~7 mm travel | rolled **left side down** | **inverted** |
  | J4 | pitches the wrist down, tip drops ~106 mm | **wrist down**, tip dropped | matches |
  | J5 | rolls the wrist **left side down**, ~7 mm travel | rolled **left side down** | matches |

  The decisive detail was in Lukas's own words — "tilted arm left, then wrist down, then wrist tilt left" —
  i.e. **J3 and J5 rolled the same way**, while the model had them counter-rotating. With
  `SERVO_SIGN = [-1, 1, 1, -1, 1, 1]` the model reproduces all four observations, including both rolls coming
  out left-side-down. Every sweep also moved cleanly at 68.7–69.7 °/s, settling in 516–578 ms with a hold band
  of ≤0.10°.
- **Scope:** direction only, one amplitude, one pose, one arm, and the verdicts rest on a human observation of
  which way a part moved rather than on any measurement of the tool. The two roll judgements (J3, J5) were the
  hardest to read — each translates only ~7 mm — and both were called the same way, which is self-consistent but
  does mean a single misread would flip a conclusion. J1 and J2 are confirmed only indirectly, by the arm rising
  and reaching forward as predicted (F-030); no sweep isolated them. Signs are **not** zero offsets: the zero
  point of every joint is still unmeasured (F-023, F-025).
- **Implication:** the servo-to-URDF mapping now has **every sign measured** rather than assumed, and two of six
  were wrong. That is the substantive point for [F-030](#f-030)'s open question: `position_only/deploy.py` maps
  `Joint1..Joint6` to motor ids 0–5 **by index with no sign convention**, so `params/deploy.yaml` would mirror
  **two** joints, not one — the base rotation and a wrist roll. A policy deployed through that manifest would
  reach to the wrong side and twist the tool the wrong way. The manifest needs a per-joint sign before G4 can
  freeze the deployment contract, and it should carry these measured values. Orientation targets through
  `d1_ik.solve` are now trustworthy in sign, having previously been wrong on J3. The method is the reusable part:
  state the model's prediction, move one joint, have someone watch. It found two errors in four sweeps, and
  neither would have shown up in telemetry, because the arm faithfully reports the angle it was commanded
  regardless of which way the joint physically turns.

### F-035 — The arm needs two feedback cycles to reach cruise, so a per-cycle IK loop can never move it smoothly

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [smooth-motion investigation](#/week/1/run/20260916T0900_d1_smooth_motion). Lukas reported the arm
  stepping rather than gliding, with wobble. A single mode-0 waypoint is in fact smooth — a proper trapezoid,
  per-sample speed **0 → 28 → 67.5 → 69.3 → 69.3 → 34 → 0 °/s** over ~0.55 s for a 30° step — but it spends
  **about two of the 111 ms feedback cycles just accelerating**. The Cartesian loop issues a new waypoint every
  cycle, which restarts that profile before cruise is reached, so every cycle is an accelerate/decelerate pair.
  Raising the per-cycle step cap does not fix it. Measured on the same move at four caps, as displacement per
  command cycle against the 7.8–8.3° a speed-limited arm would cover:

  | step cap | travelled/cycle | speed | duty |
  | --- | --- | --- | --- |
  | 5° | 2.5° | 21 °/s | 0.30 |
  | 8° | 4.4° | 37 °/s | 0.54 |
  | 12° | 4.9° | 42 °/s | 0.60 |
  | 16° | 5.3° | 46 °/s | 0.64 |

  Duty plateaus at 0.64 — Lukas confirmed by eye that the steps got smaller but never went away. Sending the
  whole solution as **one waypoint** instead reaches **7.8–7.9°/cycle at 70–71 °/s, duty 1.00 (continuous)**,
  covering a 45° excursion in one sweep and arriving at **4.2–4.9 mm**.
- **Scope:** one arm, two directions, one pose pair, unloaded, and "smooth" is Lukas's visual judgement plus the
  duty figure — no accelerometer or external measurement. Duty is *inferred* from displacement per cycle against
  a separately measured peak rate (F-033), because 111 ms feedback cannot resolve motion within a cycle. The
  residual ~4 mm is unchanged and is the small-increment floor, not the motion mode: the follow-up pass is now
  skipped outright because its 0.4–0.5° correction is below what the arm will execute.
- **Implication:** point-to-point Cartesian motion should be **single-shot, then corrected**, not servoed per
  cycle: solve once, send one waypoint, let the arm run its own trajectory, re-solve from where it arrived.
  `run_oneshot` does this and is what the browser console now uses. The per-cycle `run()` is kept for a small
  step cap where bounded, interruptible motion matters more than smoothness — it is the safer primitive near
  obstacles, since each cycle is a fresh decision. Single-shot gives that up, so it carries its own guard: a
  `max_excursion_deg` limit (70° default) refuses a far solution outright rather than flying it, and STOP
  commands the arm to its measured pose so it decelerates in place rather than releasing (F-028). Two bugs found
  while building it, both now regression-tested: the settle detector counted the arm's 60–130 ms command latency
  as arrival, and then counted **repeats of a cached feedback sample** as stillness — the second made the arm
  appear to stop 200 mm short when it was in fact still flying and arrived correctly.

### F-036 — Endpoint clearance is not path clearance: 29 of 600 clear-to-clear moves sweep the arm through the dog

- **Status:** confirmed (in the collision proxy; not tested against the physical robot)
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** Lukas asked why `run_oneshot` carries a 70° excursion limit, which was an arbitrary number of
  mine. Checking what it was actually protecting exposed a real gap: `solve` reports `clear_of_body` **only for
  the configuration it converged to**, and nothing ever tested the way there. Sampling 600 pairs of randomly
  drawn configurations that are *each* clear of the trunk/ground proxy, **29 have a colliding path**; the worst
  spends **58% of the traversal inside the body**, first entering it 25% of the way along. Their largest joint
  moves run 87–191°, so the 70° limit did refuse all of them — but it refuses **96% of ordinary moves** too, so
  it was working by being over-broad rather than by testing the hazard. The traversal is also not a straight line
  in joint space: every joint slews at the same ~1.2 rad/s ceiling (F-033), so joints with less to do arrive
  early and stop while the rest continue, and the path bends. Modelling that bend finds 29 colliding pairs where
  a straight-line approximation finds 20.
- **Scope:** this is the **collision proxy**, not the robot: a capsule chain against the Go2's *standing* trunk
  box plus a flat ground, and the dog is sitting. It knows nothing about the environment, cabling, the payload,
  or the camera Lukas is mounting, and it has never been checked against a real collision. The 29/600 rate comes
  from random configuration pairs across the whole workspace, which is far more extreme than clicking successive
  points on the sphere — today's real moves were 44.6° and 45.0°. No collision was induced on hardware.
- **Implication:** the guard now tests the actual hazard. `d1_ik.path_clearance` samples the modelled traversal
  and refuses a move whose path enters the proxy even when both endpoints are clear, and it reports **where**
  along the path it fails. Both movers enforce it, and the browser console refuses such targets at preview time,
  before SEND is ever offered. A second gap closed with it: the CLI's `--base-height` defaulted to `None`, which
  **disabled the clearance proxy entirely**, leaving the excursion limit as the only guard there; it now defaults
  to a conservative 0.15 m, with `0` to disable. With a path check in place the excursion limit no longer carries
  the collision argument and could be relaxed well above 70°. It should be **kept for a different reason**: it
  bounds how long the arm flies uncorrected in a single-shot move (70° ≈ 1.0 s, a full J0 span ≈ 3.5 s), which
  matters because STOP acts through the same 111 ms feedback and ~100 ms command path and its overtravel is
  **still unmeasured**. Measuring that stop distance is what should set the limit.

### F-037 — The gripper can be held level at a target, at the cost of about a third of the reachable sphere

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** Lukas asked for the gripper to finish level at each goal, for the end-effector camera. "Level" is
  defined from the URDF rather than guessed: `tip_offsets` gives the approach direction in the Link6 frame and the
  two finger joints sit at ∓y there, which fixes an orthonormal tool triad — approach ≈ +x, jaw-separation ≈ +z,
  up ≈ −y in the `Link7_1` frame. Level means the approach axis lies in the horizontal plane with no roll about
  it, **leaving the heading free**, so it costs two rotational degrees of freedom rather than three. The target
  attitude is re-derived from the current one on every solver iteration, which is what keeps the heading
  unconstrained. Without the constraint the gripper points **10–27° downward** at typical poses. With it,
  solutions come out at |elevation| ≤ 0.02° and position error ≤ 0.1 mm. On hardware, across three targets:

  | target (m) | commanded | measured elevation | measured roll | position error |
  | --- | --- | --- | --- | --- |
  | [0.25, 0.10, 0.35] | level | **−1.30°** | −0.18° | 5.73 mm |
  | [0.20, 0.18, 0.30] | level | **−1.48°** | −0.14° | 6.78 mm |
  | [0.30, 0.00, 0.28] | level | **−1.49°** | +0.13° | 6.38 mm |

  The reachability cost was measured over 300 points on the console's sphere: **89% reachable position-only
  versus 51% with the gripper level**. Solving it in one stage reached only 24% — reaching the point first and
  levelling from that configuration recovers more than half the apparent loss, so most of it was the solver, not
  the geometry. Staged solving is now what `level=True` does internally.
- **Scope:** three targets on one arm, unloaded, with the robot sitting; the camera is **not mounted**, so its
  orientation relative to the gripper is unmodelled and "level gripper" is not yet demonstrated to mean "level
  image". Attitude is computed from the arm's reported joint angles through the URDF, so it inherits the
  unmeasured joint zero offsets (F-023, F-025) — an independent measurement of the tool's actual attitude does
  not exist. The 51% figure is for this sphere (r = 0.40 m about the shoulder) and would change with its radius.
- **Implication:** `solve(level=True)`, `--level` on the CLI, and the default in the browser console, which now
  reports the attitude of every preview and **refuses a target it can only reach tilted** rather than silently
  solving it that way — `--no-level` restores the larger workspace. The measured residual is the interesting
  part: **−1.3° to −1.5° of pitch, consistent across all three targets**, against a commanded 0.00°. A systematic
  bias of that size is what an uncorrected joint zero offset would look like, and it is the same order as the
  arm's ~0.5°-per-joint landing floor (F-031) compounded through the wrist. It should **not** be tuned out by
  biasing the target: that would bake a calibration error into the controller. Measuring the joint zeros (G4)
  is what resolves it, and this gives that measurement a concrete, repeatable observable.

### F-038 — The measured arm model doubles the squat policy's tracking error and leaves its success rate untouched, because the reach is body-driven

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). The hardware session replaced two simulated arm
  values: joint speed limits went from the URDF's unverified 1.05/1.05/1.05/1.73/1.73/1.73 rad/s to the
  measured 1.25/1.29/1.23/1.21/1.25/1.25 (F-033), and `D1_FEEDBACK_HZ` from 10.0 to the measured 9.0, moving
  `arm_feedback_period_steps` from 5 to 6 (F-020). `verify` passes **23/23** under the new values
  (run `20260916T093504_798069Z`), with PhysX carrying the measured limits. `model_1499` from the seed-42
  candidate (F-019), trained under the *old* values, re-evaluated on the same development manifest
  (run `20260916T093536_565075Z`):

  | | trained-under model | measured model |
  | --- | --- | --- |
  | Success | 100/100 | **100/100** |
  | Falls | 0 | 0 |
  | Final-2 s mean error | 0.52 cm | **1.33 cm** |
  | 95th percentile | 1.37 cm | **2.66 cm** |
  | RMS | 2.11 cm | 2.50 cm |
  | Mean dwell | 9.85 s | 9.76 s |
  | Lowest base height | 0.1590 m | 0.1588 m |
  | Peak tilt | 17.38° | 17.63° |
  | Legs at effort limit | 9.8% of steps | **14.0% of steps** |

  The zero-action baseline on the same manifest is unchanged to four decimals (0/100, RMS 21.74 cm, lowest base
  0.2602 m; run `20260916T093636_038122Z`), as expected — zero actions hold the default joint targets and never
  approach the speed ceiling or read arm feedback, so F-018's baselines stand.
- **Scope:** one checkpoint, one seed, one manifest, `--robustness none`. This is a policy evaluated *off* its
  training distribution, not a policy trained under the measured model; how much of the degradation retraining
  would recover is not measured. The measured speed limits are lower bounds (F-033), and the ~127 ms
  command-to-motion delay (F-021) and the two-cycle acceleration ramp (F-035) are still not simulated at all,
  so the arm model remains optimistic in ways this comparison does not capture.
- **Implication:** the correction that mattered most on paper — Joint4–Joint6 losing 40% of their simulated
  speed — cost the policy **nothing in success rate**, and that is the diagnosis, not the reassurance. A
  position-only reaching policy whose success is indifferent to a 40% wrist slowdown is not reaching with its
  wrist. It shows up exactly where the arm does the work: the steady-state final-2 s error rises 2.6× and the
  95th percentile 1.9×, while the posture, the dwell and the success rate barely move. This is independent
  confirmation of [F-019](#f-019)'s mechanism from a direction that experiment could not supply. Practically:
  the F-019 checkpoint is stale and should not be retrained until the base-height term lands, so that the reward
  fix and the measured arm model are both in the run that becomes the P0 candidate of record.

### F-039 — The frozen manifest records condition labels, not the values behind them, so a changed robot model passes the guard unflagged

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). The manifest's `conditions` block records
  `latency: "estimated"`, `leg_actuator`, `arm_actuator`, `self_collisions`, the spawn height, target box, tool
  point, episode length, success radius and dwell. The arm's joint speed limits and the feedback rate are not
  among them; they reach the simulation from `motor_model.py` through `flat_env_cfg.py:398`
  (`velocity_limit_sim`) and `position_only/env_cfg.py:231` (`period_steps`). When both changed on 2026-09-16
  (F-020, F-033), two evaluations run against the same `development` manifest `e2e0d3e6a566…` — one before, one
  after — both reported **`condition_mismatches: []`**, while the policy's final-2 s error moved from 0.52 cm
  to 1.33 cm (F-038). The label `estimated` was still `estimated`; the numbers behind it were not.
- **Scope:** one observed instance, on the timing and velocity values. Whether other unrecorded values can move
  the same way has not been audited; the effort limits, gains, masses, physics rate and solver counts are all
  candidates and none of them is in the manifest either. The hash guard works as designed — it protects the
  *episode set*, and this is a gap in what the *conditions* cover, not a defect in the hashing.
- **Implication:** the guard that caught a deliberate spawn-height and arm-drive mismatch (F-018) cannot catch a
  change to the robot model itself, and the plan requires "the same robot dynamics ... for all policies" across
  P0–P4, so the robot model is part of the frozen comparison whether or not the manifest says so. Two policies
  trained months apart could be compared on the same manifest hash across a silently different arm. The fix is
  to resolve the values into the conditions — feedback Hz, command hold, leg delay range, per-joint velocity and
  effort limits — rather than the labels that select them, which makes a model change a loud mismatch instead of
  a silent one. Doing so changes all three manifest hashes and requires re-measuring the zero-action baselines,
  which is about 15 s each and is cheap **now**, before any policy of record exists. It will not be cheap later.

### F-040 — Pricing the base height removes the squat and improves tracking, at one fall in a hundred

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). A `base_height_l2` reward term was added at
  weight −50 against the measured settled stance (0.2737 m, `task_space.ZERO_ACTION_BASE_OFFSET_M`), squared
  deviation so that small adjustments stay cheap and a collapse does not. Retrained at the same budget and seed
  as F-019 (2048 envs × 1500 iterations = 73,728,000 transitions, 12 min 38 s, 98,966 steps/s, peak GPU
  4,873 MiB; run `20260916T095614_757009Z`) under the measured arm model (F-020, F-033). Evaluated on the
  re-frozen development manifest `3c5270d9b2cb` with no condition mismatches
  (run `20260916T100927_254710Z`), against the F-019 policy re-run on the same model (F-038):

  | | F-019 squat | with base-height term |
  | --- | --- | --- |
  | Success | 100/100 | **99/100** (Wilson 94.6–99.8%) |
  | Falls | 0 | **1** (1.0%, Wilson 0.18–5.45%) |
  | Final-2 s mean error | 1.33 cm | **0.93 cm** |
  | 95th percentile | 2.66 cm | 2.39 cm |
  | RMS | 2.50 cm | 2.51 cm |
  | Lowest base height | 0.1588 m | **0.2182 m** |
  | Episodes below 0.20 m | 100 of 100 | **0 of 100** |
  | Median episode tilt | 10.89° | **8.00°** |
  | Peak tilt | 17.63° | 42.79° |
  | Legs at effort limit | 14.0% of steps | **6.9% of steps** |

  Training-time `Episode_Reward/base_height` fell from −0.0185 at iteration 300 to −0.0039 at 1500, i.e. an RMS
  deviation from the settled stance of 1.9 cm falling to 0.9 cm. The single failure is episode 12, target
  (0.471, 0.080, 0.598) — the far top corner of the box, at the edge of two ranges. It reached and dwelled for
  0.82 s, short of the 1 s needed, then tipped at 1.80 s to 42.8° against the 45.84° `bad_orientation` limit.
  It is the only episode past 30°; the next highest is 13.2° at the 95th percentile.
- **Scope:** one seed, one manifest, `--robustness none`, `development` (the inspectable set), deterministic
  policy actions. `eval.json` reports `g1a.passed: true`, but on 100 episodes one fall *is* 1.0% against a
  "≤1% falls" criterion, so this sits exactly on the boundary rather than inside it, and the Wilson interval
  puts the true fall rate as high as 5.45%. G1a additionally wants three seeds (G3), and a development-set
  number cannot be the reported result. The arm's commanded-effort figure stays near 1.0 and remains a
  commanded figure (F-017).
- **Implication:** the squat was a reward gap, not a property of the task. Pricing the base height removed it
  outright — no episode now goes below 0.20 m, against every episode before, and the worst margin above the
  `low_base` termination goes from 9 mm to 6.8 cm — while steady-state tracking *improved* by 30% and leg
  saturation halved. The policy is reaching with its arm. What it exposed is a different, rarer failure: from a
  standing posture the far top corner of the box is near the tilt limit, and one episode in a hundred tips
  there. That is a better failure to have than a 9 mm floor margin, because it is localised to a known corner
  rather than present in every episode, but it is not yet a G1a pass and must not be reported as one. Next:
  three seeds, then decide whether the corner needs a posture term, a curriculum, or a box whose corners the
  robot can hold. Do not tune against `validation` or `test`.

### F-041 — Three seeds fail G1a on falls: two of three tip at far targets, and the development set understated it

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). Seeds 43 and 44 trained at the identical budget and
  configuration as seed 42 (2048 envs × 1500 iterations = 73,728,000 transitions each; 12 min 36 s at
  94,182 steps/s and 12 min 25 s at 93,842 steps/s; peak GPU 4,700 MiB; runs `20260916T103409_463915Z_train_seed43`,
  `20260916T104701_429356Z_train_seed44`). Each seed's `model_1499` evaluated on the development and validation
  manifests, no condition mismatches on any run:

  | Seed | Set | Success | Falls | Final-2 s | 95th pct | Lowest base | Peak tilt | Legs at limit |
  | --- | --- | --- | --- | --- | --- | --- | --- | --- |
  | 42 | development | 99/100 | 1 | 0.93 cm | 2.39 cm | 0.2182 m | 42.8° | 6.9% |
  | 42 | validation | 97/100 | **3** | 0.89 cm | 2.82 cm | 0.1872 m | 44.9° | 5.7% |
  | 43 | development | 100/100 | **0** | 0.52 cm | 1.41 cm | 0.2395 m | 12.2° | 0.1% |
  | 43 | validation | 100/100 | **0** | 0.49 cm | 1.39 cm | 0.2383 m | 11.8° | 0.1% |
  | 44 | development | 97/100 | **3** | 0.74 cm | 2.69 cm | 0.1824 m | 45.7° | 3.0% |
  | 44 | validation | 97/100 | **3** | 0.72 cm | 2.56 cm | 0.1837 m | 45.7° | 2.6% |

  G1a wants ≤1% falls per seed. On validation that is 3% / 0% / 3%: **two of three seeds fail**. All six
  failures are tilt terminations against the 45.84° `bad_orientation` limit, at 42.0–45.7°, dying 1.68–4.66 s
  into the episode. Their targets sit at x-fraction 0.60–0.97 of the box's 0.36–0.48 m depth (mean 0.84, against
  0.49 for all episodes); by height they are spread across 0.08–0.87 of the range. Five of six lie in the far
  40% of the box, for which the chance probability is 0.4⁶ = 0.004. Seed 43 reaches **39 of 39** of those same
  far targets successfully, at a median tilt of 10.3°. Median tilt per seed on far targets is 8.4° (42),
  10.3° (43) and 15.5° (44).
- **Scope:** three seeds, one budget, one configuration, `--robustness none`, deterministic actions,
  `model_1499` from each run with no checkpoint selection. The `test` manifest is untouched. The per-seed fall
  counts are 3 of 100, so each carries a Wilson 95% interval of roughly 1.0–8.5%; the claim that seeds 42 and 44
  exceed 1% is firmer than any particular rate. Why seed 43's solution is stable and the others' are not is not
  established — only that its tilt distribution is lower throughout, not merely in the tail.
- **Implication:** **G1a is not passed.** The squat fix (F-040) held — no seed goes below 0.18 m and the stance
  term is doing its work — but removing the crouch moved the failure to tilt, and at the frozen budget PPO finds
  a stable solution roughly one time in three. Two corrections to what F-040 recorded from a single seed: the
  failures are governed by reach **distance**, not by the far top corner as that one data point suggested, and
  height does not predict them; and seed 42's development result (1 fall) understated its validation result
  (3 falls), which is exactly why a development number cannot be reported. Seed 43 proves the task is not the
  problem — a policy exists that reaches every far target at 12° of tilt with 0.1% leg saturation — so this is
  training variance, not infeasibility, and the fix belongs in the objective or the curriculum rather than in
  the box. Adding a tilt-rate or angular-momentum cost, or curriculum over reach distance, is the obvious next
  step; do not tune it against `validation`, which is now spent for these three policies, and do not touch
  `test`.

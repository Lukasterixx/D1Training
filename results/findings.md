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

- **Status:** superseded in its latency figure by [F-045](#f-045) (2026-09-17: the ~127 ms is mostly the wait for the next 111 ms feedback sample; fitted to the raw samples the command dead time is 0–10 ms, and a 127 ms dead time fits 7× worse). Its speed observation was already replaced by F-033
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

- **Status:** confirmed, with its mechanism refined by [F-045](#f-045) (2026-09-17: "two feedback cycles to reach cruise" is how a ~80 ms ramp looks when sampled every 111 ms; the per-waypoint restart it describes is confirmed and quantified — a speed-preserving planner would cover 7.8°/cycle, restarting from rest covers 5.0, the arm covered 4.9–5.3)
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

### F-042 — Pricing base rotation cuts falls from four in three hundred to one, and changes what the last failure is

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). `base_motion_l2` priced only linear base velocity;
  rotation was unpriced, and every G1a failure was rotational (F-041). A `base_angular_motion_l2` term on
  `root_ang_vel_b` was added at weight −0.2, matching the linear term. Seeds 42/43/44 retrained at the identical
  budget (73,728,000 transitions each, 12 min 29 s to 12 min 31 s, 93,864–95,324 steps/s, peak GPU 4,691 MiB;
  runs `20260916T113837_593527Z`, `…115122_647918Z`, `…120408_316600Z`) and evaluated on the **development**
  manifest `3c5270d9b2cb`, no condition mismatches:

  | Seed | Success | Falls | Final-2 s | 95th pct tilt | Max tilt | Legs at limit |
  | --- | --- | --- | --- | --- | --- | --- |
  | 42 | 99 → **99** | 1 → **1** | 0.93 → **0.47 cm** | 13.2° → **7.4°** | 42.8° → 44.3° | 6.9% → **0.4%** |
  | 43 | 100 → **100** | 0 → **0** | 0.52 → **0.45 cm** | 11.6° → **8.1°** | 12.2° → **9.0°** | 0.1% → **0.0%** |
  | 44 | 97 → **100** | 3 → **0** | 0.74 → 0.80 cm | 17.7° → **5.8°** | 45.7° → **8.0°** | 3.0% → **0.4%** |

  Falls across the 300 development episodes go **4 → 1**. Seed 44's failures are eliminated outright and its
  worst tilt falls from 45.7° to 8.0°. Leg saturation is effectively gone. Training-time `bad_orientation` for
  seed 42 fell 0.83% → 0.19%. The one remaining failure has changed character: seed 42 episode 62, target
  (0.477, −0.065, 0.509) at depth 0.98 and height 0.07 of the box — the far bottom corner — **dwelled 4.92 s**
  and then tipped at 5.64 s. The earlier failures died at 1.68–2.50 s *during* the reach; this one reached,
  held the target for nearly five seconds, and then lost balance. Seed 42's distribution is otherwise extremely
  tight: p50 5.8°, p99 8.9°, exactly one episode above 20°.
- **Scope:** three seeds, one budget, **development only**. The `validation` result is deliberately not
  measured here, for the reason in the implication. This is not a G1a attempt. The arm model still lacks the
  measured command latency (F-021) and acceleration ramp (F-035), both of which bear on a failure that happens
  during a hold.
- **Implication:** the missing term was real, not a tuning knob: rotation had never been priced while translation
  was, and the seed-to-seed variance in F-041 was largely variance in how much the trunk was used. All three
  seeds now converge to a similar calm solution, with median arm reaction torques within 4.74–5.48 N·m of each
  other where they previously spanned 6.59–10.38. **This still cannot be called a G1a pass.** Seed 42's 1 fall
  in 100 is 1.0% against a ≤1% criterion — the boundary again — and development understated seed 42 before
  (1 fall there, 3 on validation). More importantly, the diagnosis that motivated this term — the far-target
  concentration and the arm-versus-leg effort split — was read off the **validation** episodes in F-041, so
  re-measuring these policies on validation would report a number tuned against that set. Validation is
  contaminated for this comparison. The gate needs a **fresh validation draw** (a new RNG stream, versioned
  alongside the existing one rather than replacing it), which costs about 15 s to build and 20 s per seed to
  measure. `test` remains untouched and must stay so.

### F-043 — Every policy reaches partly by walking: the base ends 20–25 cm forward, and no metric was watching

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). A viewer replay of the best episode we have
  (seed 43, development episode 94, run `20260916T130843_655086Z_view_seed42`) reported the base 20.1–21.2 cm
  **forward** of its spawn point across 17 consecutive episodes. The sign was checked against the Week 1
  zero-action viewer run, which recorded `mean_base_x_from_spawn_m: -0.075` and printed it as "7.5 cm behind",
  matching F-014's backward settle. Base translation was then added to the evaluator and all three v3 policies
  re-measured on the development manifest, 100 episodes each:

  | Policy | Mean final base x | Max final base x | Max horizontal travel |
  | --- | --- | --- | --- |
  | zero actions | **−5.6 cm** | −5.5 cm | 9.2 cm |
  | seed 42 | **+24.1 cm** | **+48.1 cm** | 51.8 cm |
  | seed 43 | **+20.5 cm** | +29.8 cm | 30.6 cm |
  | seed 44 | **+25.3 cm** | +34.5 cm | 35.5 cm |

  The zero-action figure reproduces F-014's −5.55 cm settle exactly, which validates the measurement. Seed 42's
  distribution: minimum 16.7 cm, median 23.5 cm, maximum 48.1 cm, with 6 of 100 episodes past 30 cm. That policy
  scores 99/100 with 1 fall and nothing in its reported result showed the walking.
- **Scope:** three policies, one manifest, deterministic actions. Displacement is measured from the spawn point
  in the environment frame; it does not separate walking from sliding or from a single lunge, and no gait
  quality is assessed (that is G1b). Whether the same happens under `--robustness unitree` is untested.
- **Implication:** the task is described as free-space stance-and-reach and F-016 recorded that the box "never
  requires the base to move", but every policy moves it 20–25 cm and one episode moves it half a metre. The
  workspace analysis says 99.1% of the box is reachable from the settled stance (F-016), so this is not
  necessary — it is the fourth instance of one pattern. `base_motion_l2` prices base **velocity**, so a slow
  creep is nearly free, exactly as a slow squat was free before F-040 and rotation was unpriced before F-042.
  The evaluator recorded base height and tilt — which is why the squat and the tilt failures were visible — and
  never recorded translation, so every G1a number produced today was blind to it. Base displacement is now in
  the per-episode record and the `eval.json` summary. For P0–P4 this is a confound before it is a bug: a reach
  metric that includes a fifth of a metre of locomotion does not isolate arm or tool control, which is what
  those comparisons exist to measure. It was found by watching, not by measuring, which is the argument for the
  G0 visual-inspection item rather than against it.

### F-044 — The policies oscillate at 5–8 Hz while holding, above what the D1 can execute, with the arm at 60–90% of its speed limit

- **Status:** confirmed in its measurements; its implication corrected by [F-045](#f-045) (2026-09-17: the ~127 ms delay and ~220 ms ramp it cites were feedback-sampling artefacts, so "the D1 physically cannot execute this motion" is withdrawn — 5–10 mm of tip oscillation is about a degree at the joints, well within reach. The oscillation itself and the arm running near its speed ceiling while holding stand)
- **Week:** 1
- **Date:** 2026-09-16
- **Evidence:** [Week 1 log, 2026-09-16](week_01/notes.md). Lukas, watching the replay, reported the body and
  arm "oscillating back and forth / up and down" while the tool point tracked accurately. Amplitude and
  frequency were added to the evaluator over the final two seconds of each episode, by which the tool point is
  parked, so what remains is a limit cycle rather than progress. Frequency is taken from mean crossings, two
  per cycle. Development manifest, 100 episodes each:

  | Policy | Base z p-p | z | Base x p-p | x | Tip error p-p | err | Arm joint vel RMS | Leg |
  | --- | --- | --- | --- | --- | --- | --- | --- | --- |
  | zero actions | 0.5 mm | 0.5 Hz | 2.2 mm | 0.5 Hz | 1.2 mm | 0.5 Hz | **0.020 rad/s** | 0.012 |
  | seed 42 | 8.0 mm | 3.6 Hz | 7.8 mm | 2.2 Hz | 9.8 mm | **5.1 Hz** | **0.731 rad/s** | 0.204 |
  | seed 43 | 7.6 mm | 4.8 Hz | 10.5 mm | 3.1 Hz | 6.9 mm | **7.8 Hz** | **1.129 rad/s** | 0.306 |
  | seed 44 | 5.3 mm | 4.8 Hz | 1.9 mm | 3.8 Hz | 5.4 mm | **5.5 Hz** | **0.839 rad/s** | 0.213 |

  Zero actions sit at the measurement floor, so these are the policies' own motion. While nominally holding
  station the arm joints run at 0.73–1.13 rad/s RMS against a **measured** ceiling of 1.20–1.29 rad/s (F-033):
  60–90% of full speed, continuously, to stay still. `Episode_Reward/action_rate` is −0.0206 at weight −0.01,
  so mean `action_rate_l2` ≈ 2.06 across 18 actions — an RMS action change of 0.34 every 20 ms.
- **Scope:** simulation only, one manifest, `--robustness none`, deterministic actions. Mean crossings give a
  dominant frequency, not a spectrum, and a 2 s window at 50 Hz resolves roughly 0.5–12 Hz. No hardware was
  involved and nothing here measures a real D1's response.
- **Implication:** the tip oscillates at 5.1–7.8 Hz. The D1 accepts commands at 10 Hz and publishes angles at
  9 Hz (F-020), so this sits at or above half the loop rate — and the arm needs about 220 ms to reach cruise
  (F-035), roughly 4.5 Hz, so it **physically cannot execute this motion**. The behaviour is a simulation
  artefact, and specifically an artefact of what the timing model leaves out: `--latency estimated` models the
  command *hold* and the feedback *period* but not the ~127 ms command-to-motion delay (F-021) or the
  acceleration ramp, so the simulated arm has less dead time than the hardware and can chatter in a band the
  real one cannot even be commanded in. Two consequences. Any transfer claim from these policies is void until
  the arm model carries its measured latency and ramp, which raises that task above further G1a tuning. And
  `action_rate` at −0.01 prices chatter at roughly a hundredth of what reaching pays, the same structural gap as
  the free squat (F-019), unpriced rotation (F-042) and velocity-priced translation (F-043) — but the term
  should not be retuned until the timing model is right, or it will be tuned against an artefact.

### F-045 — Fitted to its raw samples, the D1 has ~10 ms of command dead time, not ~127 ms: an ~80 ms-ramp trapezoid that restarts from rest on every new setpoint

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [fit figure](week_01/figures/d1_arm_response.png) and
  [d1_arm_response.json](week_01/figures/d1_arm_response.json) from `python -m position_only.arm_response`, which
  simulates `core.TrapezoidTracker` — the implementation the task now uses — against the recorded hardware.
  **Steps:** the six 30° single-joint sweeps on funcode 2 mode 0 (run `20260916T0800_d1_hold_sweeps`, 12 legs,
  234 deduplicated samples at their actual timestamps). Dead time was profiled with acceleration and
  deceleration refitted at each value:

  | Dead time | Accel (rad/s²) | Decel (rad/s²) | Step RMS | Streaming RMS, restart from rest |
  | --- | --- | --- | --- | --- |
  | 0 ms | 12.5 | 17.4 | 0.339° | 0.94 °/cycle |
  | **10 ms** | **15.5** | **17.4** | **0.342°** | **0.93 °/cycle** |
  | 20 ms | 21.0 | 17.3 | 0.348° | 1.18 |
  | 40 ms | 67.0 | 17.1 | 0.359° | 1.90 |
  | 60 ms | 400 (bound) | 46.3 | 0.518° | — |
  | 127 ms | 400 (bound) | 400 (bound) | **2.387°** | — |

  **Streaming:** the F-035 cap sweep, a waypoint every 111 ms at the measured angle plus the cap. At caps 8/12/16
  a planner that keeps its speed on a new setpoint covers 6.8/7.8/7.8 °/cycle; restarting from rest covers
  5.0/5.0/5.0; the arm covered 4.4/4.9/5.3. Restarting from rest is the best replanning rule at every dead time.
  The chosen model — 10 ms, 15.5 and 17.4 rad/s², restart from rest — is the best streaming fit among dead
  times within 0.01° of the best step fit, a rule fixed in code before its output was read. Two corrections to
  the reconstruction were needed before the fit meant anything: the holds were 2 s, not the 3 s first assumed,
  and the recorded return-leg latencies for J1 and J2 are artefacts — the settled joint dithers ±0.1° around a
  point 0.3° from the commanded target, which trips the 0.3° detection threshold with no motion, so return
  commands are placed one hold after the outbound command instead.
- **Scope:** one arm, unloaded, sitting robot, one posture per joint, 30° steps, one command path. The feedback
  transport age is folded into the dead time. On single steps dead time and acceleration trade off, so 0–10 ms
  is the supported range, not 10 ms a measurement. At a 5° streaming cap the model covers 4.3 °/cycle against
  2.5 measured, where the loop's IK re-solve and the arm's ~0.7° landing deadband (F-026) come in. Whether
  re-sending an *identical* setpoint restarts the plan was never tested; every streamed waypoint in F-035 was
  new.
- **Implication:** the ~127 ms command-to-motion "latency" (F-021) and the ~220 ms "ramp" (F-035) were both
  mostly the 111 ms feedback period, so the loop's dominant real latency is that period — which the simulation
  already modelled — and F-044's claim that the D1 cannot execute the policies' oscillation is withdrawn. What the
  simulation did lack is the restart: every new setpoint costs the arm its speed, so a controller streaming at
  10 Hz gets roughly two-thirds of the arm's single-command speed. The fitted values are in `motor_model.py`
  with their labels, and `tests/test_arm_trajectory.py` fails if they drift from this fit's output. F-044 cited both
  earlier figures; its measurements stand and its implication is corrected above.

### F-046 — Under the task's 10 Hz stream the simulated arm now moves as a streamed D1 does, where the old model gave it single-command speed; current policies lose 5× their steady-state precision on it

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [comparison figure](week_01/figures/d1_arm_sim_vs_hardware.png) and
  [d1_arm_sim_vs_hardware.json](week_01/figures/d1_arm_sim_vs_hardware.json). `--arm_trajectory measured` (now the
  default) runs F-045's planner in the arm action at the physics rate; `processed_actions` stays the 10 Hz setpoint
  the firmware receives. `run_position_only.py arm_steps` repeats the hardware protocol in simulation, each joint
  30° out and back with the robot standing:

  | | Fastest speed per command cycle | Drive vs plan |
  | --- | --- | --- |
  | D1, one message per step (F-033) | 1.27 rad/s | — |
  | D1, a new waypoint every cycle (F-035) | **0.83 rad/s** | — |
  | sim, no planner (the model until today) | 1.27 rad/s | — |
  | sim, planner without feedforward | 0.77 rad/s | RMS 2.26°, **lag 97 ms** |
  | sim, planner with velocity feedforward | **0.81 rad/s** | RMS 0.31°, lag 0 ms |

  The first planner run trailed its own plan by 97 ms: against a zero velocity target the drive's 400 N·m·s/rad
  damping needs 7.2° of position error to sustain 1.25 rad/s, which counts the servo's lag twice, since the fitted
  trapezoid already is the joint's measured motion. The planned velocity is now the drive's velocity target.
  `verify` passes **25/25** with the planner (run `20260916T235005_620389Z`), adding two checks: planned targets
  stay under each joint's ceiling and never speed up faster than the fitted acceleration, and the joint follows
  the plan (0.58° RMS). A first attempt bounded deceleration too and failed at 41.9 rad/s² — the per-message
  restarts the model is built to have — so only speeding up is bounded (run `…234904_615402Z`). `verify` also
  passes with `--arm_trajectory none` (run `…235123_684069Z`). The manifests now carry the planner's resolved
  values; rebuilt as development `930d188d26b9`, validation `c0eb26cd7611`, test `a7069027f9ee` with byte-identical
  episodes, the zero-action baseline is unchanged (0/100, RMS 21.74 cm, lowest base 0.2602 m), and a deliberate
  `--arm_trajectory none` run against them is flagged. The three v3 policies, trained without the planner, on the
  development manifest under it:

  | Seed | Success | Falls | Final-2 s error | Tip p-p while holding | Arm joint vel holding | Base travel | Max tilt |
  | --- | --- | --- | --- | --- | --- | --- | --- |
  | 42 | 99 → 96 | 1 → 0 | 4.7 → **23.2 mm** | 9.8 → 7.8 mm, 6.5 Hz | 0.73 → 0.55 rad/s | 24.1 → 22.2 cm | 44.3° → 28.1° |
  | 43 | 100 → 98 | 0 → 0 | 4.5 → **23.7 mm** | 6.9 → 6.6 mm, 4.7 Hz | 1.13 → 0.79 rad/s | 20.5 → 11.8 cm | 9.0° → 10.2° |
  | 44 | 100 → 100 | 0 → 0 | 8.0 → 7.2 mm | 5.4 → 2.8 mm, 5.3 Hz | 0.84 → 0.57 rad/s | 25.3 → 25.6 cm | 8.0° → 8.4° |
- **Scope:** simulation. The sim steps start from the arm's default pose on a standing robot, the hardware from
  varied poses on a sitting one, so gravity loading differs. The planner restarts on *every* 10 Hz message,
  including an identical re-send — the conservative reading, since a simulated arm that ignored identical re-sends
  would reward a policy for saturating its actions to hold full speed, and that may not exist on the hardware. The
  policies are evaluated off their training distribution, one manifest, one simulator seed. At the end of a move
  the joint overshoots its plan by up to 1.5°.
- **Implication:** the old arm model was right about the D1's *capability* and wrong about what a streaming
  controller gets from it: it gave a policy streaming at 10 Hz the speed of a single command, half again what the
  hardware delivers. Two of three current policies lose 5× their steady-state precision on the corrected arm — their
  fine tracking depended on corrections the streamed D1 cannot make — while their oscillation shrinks but persists
  and their walking is unchanged. They need retraining under the planner before any further G1a reading. Two
  hardware questions follow, both cheap: whether an identical re-sent setpoint restarts the D1's plan (a single
  30° step with the setpoint re-sent at 10 Hz answers it), and, for the G4 deployment contract, whether the deploy
  stack should stream the policy's arm output at 10 Hz at all, given that streaming costs the arm a third of its
  speed.

### F-047 — Retrained on the realistic arm, the policies recover most of their precision but send bigger commands the arm's planner absorbs: holding shake doubles, and G1a still fails on falls

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [Week 1 log, 2026-09-17](week_01/notes.md). Seeds 42/43/44 retrained under `--arm_trajectory measured`
  at the unchanged budget (2048 envs × 1500 iterations, 73,728,000 transitions each; 13 min 06 s to 13 min 59 s;
  mean 90,810 steps/s; peak GPU 4,959 MiB; runs `20260917T001239_982420Z`, `…002655_508750Z`, `…004051_857200Z`).
  Development manifest `930d188d26b9`, no condition mismatches. Against the same seeds trained without the planner,
  evaluated on the realistic arm (F-046):

  | Seed | Success | Falls | Final-2 s error | Median tip p-p holding | Arm vel holding | Base travel | Max tilt |
  | --- | --- | --- | --- | --- | --- | --- | --- |
  | 42 | 96 → 97 | 0 → **3** | 23.2 → **11.5 mm** | 6.6 → **12.2 mm** | 0.55 → 0.66 rad/s | 22.2 → 28.3 cm | 28.1° → 44.8° |
  | 43 | 98 → 100 | 0 → 0 | 23.7 → **6.7 mm** | 6.2 → **11.5 mm** | 0.79 → 0.79 rad/s | 11.8 → 5.4 cm | 10.2° → 10.7° |
  | 44 | 100 → 99 | 0 → **1** | 7.2 → 10.4 mm | 4.9 → **14.6 mm** | 0.57 → 0.74 rad/s | 25.6 → 11.4 cm | 8.4° → 43.5° |

  (Median tip p-p for the "without planner" column is from those seeds on the old arm; the evaluator's pooled
  means are inflated by a fall's last two seconds, up to 418 mm, so medians are reported.) The shake is typical:
  77/99, 77/100 and 94/100 episodes exceed 10 mm peak-to-peak while holding. Seed 42's three falls are at depth
  0.92–1.00 of the box, dying at 1.62–3.20 s — F-041's far-reach pattern; seed 44's one fall came after holding for
  7.30 s. Training-time `Episode_Reward/action_rate` rose from −0.021 to −0.032, −0.032 and −0.033: the retrained
  policies change their commands about 50% more per step.

  **Diagnostic** — the same retrained checkpoints with the planner switched off (runs `…005629_356785Z`,
  `…005649_629717Z`, `…005710_296304Z`, each flagged as a condition mismatch, deliberately): median tip p-p while
  holding **12.2 → 35.1, 11.5 → 67.1, 14.6 → 48.2 mm**; final-2 s error 11.5 → 23.5, 6.7 → 44.3, 10.4 → 32.9 mm;
  falls 3 → 1, **0 → 46**, 1 → 7.
- **Scope:** three seeds, development manifest only, one simulator seed, `--robustness none`. The planner-off
  runs are policies evaluated off their training distribution, used as a diagnostic, not a result. `validation`
  remains contaminated for this line of work (F-042). Nothing here is hardware.
- **Implication:** **G1a still fails** — seed 42 falls 3 times in 100 against ≤1%. Retraining recovers most of the
  steady-state precision the corrected arm took away (two seeds from ~23 mm to 6.7 and 11.5 mm), so the precision
  loss in F-046 was mostly a training-distribution mismatch. But the policies adapted to an arm that only partly
  follows each command by sending bigger, more frequent command changes; the planner absorbs most of them, leaving
  12–15 mm of typical holding shake, twice the old-arm policies. The diagnostic refutes the tempting explanation that
  the planner's restarts *cause* the shake: without the planner the same commands shake 3–6× more and one seed
  falls in almost half its episodes. Two consequences. First, `action_rate` at −0.01 can now be retuned — F-044
  held it back until the arm model was right, and the model is now fitted — since command chatter is the thing to
  price. Second, these policies depend on the firmware restarting exactly as modelled; the untested case of an
  identical re-sent setpoint (F-045) is now a safety question for any physical trial, not a modelling detail.

### F-048 — Pricing command changes at −0.05 removes the falls and the walking across three seeds, but not the shake, which is now mostly the body bobbing

- **Status:** confirmed
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [Week 1 log, 2026-09-17](week_01/notes.md). A `--reward_weight TERM=WEIGHT` override was added to the
  runner, recorded in `run.json` and applied before `env.yaml` is written. `action_rate` was swept on seed 42 under
  the realistic arm at the unchanged budget (runs `20260917T011444_487339Z`, `…012822_678050Z`,
  `…014131_782753Z`), with the selection rule fixed before any result: among weights scoring ≥95/100 with ≤1 fall on
  `development`, lowest median tip shake while holding, ties to the smaller magnitude.

  | Weight (seed 42) | Success | Falls | Settled error | Median tip shake | Base travel | Max tilt |
  | --- | --- | --- | --- | --- | --- | --- |
  | −0.01 | 97 | 3 | 11.5 mm | 12.2 mm | 28.3 cm | 44.8° |
  | **−0.05** | 100 | 0 | 4.4 mm | **8.4 mm** | 7.8 cm | 11.2° |
  | −0.1 | 100 | 0 | 5.0 mm | 10.6 mm | 2.0 cm | 11.7° |
  | −0.3 | 100 | 0 | 8.2 mm | 11.2 mm | 11.5 cm | 12.0° |

  The rule selected −0.05; at −0.3 reaching slowed (0.27 s against 0.17). Seeds 43 and 44 were then trained at −0.05
  (runs `20260917T015804_638974Z`, `…021141_560571Z`; 13 min 00–21 s; peak GPU 4,999 MiB). Development manifest
  `930d188d26b9`, no condition mismatches:

  | | Falls /300 | Success per seed | Mean settled error | Median tip shake per seed | Mean base travel |
  | --- | --- | --- | --- | --- | --- |
  | v3, old arm | 1 | 99 / 100 / 100 | 5.8 mm | 6.6 / 6.2 / 4.9 mm | 23.3 cm |
  | v4, realistic arm, −0.01 | 4 | 97 / 100 / 99 | 9.5 mm | 12.2 / 11.5 / 14.6 mm | 15.0 cm |
  | **v5, realistic arm, −0.05** | **0** | **100 / 100 / 100** | 7.7 mm | **8.4 / 16.3 / 12.5 mm** | **3.6 cm** |

  Splitting the shake while holding (medians over the final 2 s): v5 base height swings 2.5 / 30.0 / 23.6 mm
  peak-to-peak at 2.5 / 1.8 / 1.8 Hz, against 7.4 / 7.6 / 4.9 mm for v3; tip error ripples at 6.5 / 8.0 / 9.8 Hz;
  leg joint velocity while holding is 0.24 / 0.49 / 0.42 rad/s against 0.21–0.31 for v3. Training-time
  `action_rate_l2` fell from about 3.2 to about 1.0.
- **Scope:** development manifest only, one simulator seed, `--robustness none`. Seed 42 chose the weight, so its
  row is optimistic; seeds 43 and 44 are the fair test of it. `validation` is contaminated for this line of work
  (F-042), so none of this is a G1a measurement. Mean-crossing frequency over 2 s resolves roughly 0.5–12 Hz.
- **Implication:** at −0.05 all three seeds score 100/100 with no falls and walk 1–8 cm instead of 20–25, so G1a's
  numbers are met on `development` by every seed for the first time — a candidate, to be measured on a fresh
  validation draw. The shake is **not** fixed: seed 42's 8.4 mm, which selected the weight, did not carry to seeds
  43 and 44 (16.3, 12.5 mm), which is the selection optimism the rule's own scope anticipated. What remains is mostly
  the body bobbing 2–3 cm at about 2 Hz, the body-versus-arm pattern of F-019, F-042 and F-043 once more, now
  vertical: a ±1.5 cm bob at 2 Hz costs roughly 0.01 per step between `base_height` and `base_motion`, against about
  3.0 for reaching. The faster tip ripple sits near the 10 Hz command rate.

### F-049 — From a lying Go2 the D1 can pick a floor-standing cup only from above, and a scripted YOLO, depth and IK pick lifts one in simulation wherever that grasp is reachable

- **Status:** provisional
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [Week 1 log, 2026-09-17](week_01/notes.md).
  CPU model, 300,000 configurations inside the soft limits: no grasp with the approach within 15° of horizontal puts
  the jaw centre within 8 cm of the floor at any base height from 0.12 to 0.27 m. Pointing down, IK finds grasp,
  pregrasp and a clear path for a 10–12 cm cup 35–45 cm ahead of the base at base heights 0.10–0.14 m. In Isaac Sim
  the robot lies at 8.5 cm, 7.3° nose-up, with Unitree's lie-down leg targets. The pick runs stock `yolo11s-seg`
  (COCO, no fine-tuning) on a wrist camera with the D435 preset, a rim-circle fit through forward kinematics of the
  9 Hz feedback angles, a top-down grasp from `d1_ik`, and the task's own arm interface (10 Hz setpoints, F-045
  planner). On one code version it lifted a 55 × 100 mm mug 11.8–11.9 cm in **8 of 9** configurations: three cup
  positions, 45° handle, seed 7, D405 preset, measured mount, and a marginal far position. The cup ended 3–8 mm from
  the jaw centre. The failure was the handle turned 90°, lying across the jaw axis, which blocked the descent. From
  a 40 cm look YOLO detected the cup in every frame (confidence 0.95); estimates were 2.8–4.2 mm off horizontally
  and 6.4–7.5 mm low (F-052).
- **Scope:** simulation only: ideal rendering, one untextured white mug on a plain floor, one lighting, best-case
  depth noise, a solid collider, the URDF's CAD gripper with independent finger drives (F-051) and an assumed camera
  mount. Nine runs, one per configuration. The lying posture is Unitree's example target, not the real robot's
  measured StandDown.
- **Implication:** a demonstration is geometrically plausible if the cup is about 55 mm wide (open jaws 77 mm in the
  CAD) and about 10 cm tall, stands 35–45 cm ahead of the base centre, and has its handle turned away from or towards
  the robot. The handle's direction is not perceived, so the operator must place it. The wrist camera must look from
  at least 18 cm on a D435i, with its targets kept above the gripper in the image. Nothing here is evidence for the
  hardware pick. It depends first on the unvalidated joint zero (F-023), a hand-eye calibration of the real bracket,
  and the real gripper's coupling, force and timing.

### F-050 — The trunk/ground clearance proxy assumes a level base: under the lying robot's 7° pitch it put the floor 5–6 cm too high where the cup stood, and a nose-down pitch would put it too low

- **Status:** confirmed (in the proxy; the pitch is the simulated lying posture)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [Week 1 log, 2026-09-17](week_01/notes.md). `workspace.clear_of_body`, and through it
  `d1_ik.path_clearance`, test the ground as z > −base_height in the base frame. Lying at 8.5 cm with 7.3° nose-up
  pitch, the first pick ([run](#/week/1/run/20260917T024745_428318Z_pick_seed42)) rejected every grasp of a cup
  whose rim was 10 cm above the real floor: 44 cm ahead that plane sits about 5.6 cm above the floor. The same
  chain tested against the floor plane along gravity (`pick_demo.grasp.arm_clear`) accepted the plan, and the next
  run executed it without floor contact ([run](#/week/1/run/20260917T024959_791290Z_pick_seed42)).
  `tests/test_pick_demo.py` holds a case the level proxy rejects and the gravity test accepts.
- **Scope:** the tilt is simulated; the real robot's sitting or lying pitch has not been measured. The proxy itself
  is unchanged.
- **Implication:** with the base nose-up the proxy refuses reachable poses near the floor ahead. Nose-down it
  **accepts poses that reach into the floor ahead**, which is the unsafe direction. The hardware mover and the
  browser console use the level proxy with a fixed 0.15 m base height for a sitting robot. Before either trusts
  ground clearance near the floor, measure the base attitude, or pass gravity the way `arm_clear` does.

### F-051 — The URDF gripper's two independent finger drives do not centre what they grasp: closed fully, one finger pushed the cup 11 mm and drove the other to its open stop

- **Status:** confirmed (simulation, one cup)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [Week 1 log, 2026-09-17](week_01/notes.md). The fingers are two prismatic joints with separate
  implicit drives (4000 N/m) capped at the URDF's 15 N.
  - **Commanded fully closed** ([run](#/week/1/run/20260917T025256_661501Z_pick_seed42)): both fingers closed
    together to about 21 mm of travel. Then finger 1 went on to 6.8 mm while finger 2 was driven back to its −30 mm
    open stop, and the cup moved 11 mm along the jaw axis, ending 12 mm from the jaw centre. Two equal forces at
    their cap hold an object but exert nothing to centre it.
  - **Commanded to 4 mm under the measured diameter** ([run](#/week/1/run/20260917T025517_631413Z_pick_seed42)):
    fingers settled at 16.7 and 20.9 mm and the cup ended 3.8 mm from the jaw centre. Every final-set success used
    this.
- **Scope:** one 55 mm cylinder collider, one friction pair; the grip force was not measured.
- **Implication:** simulated grasps in this repo must command a width, not "close". The real D1 drives both fingers
  from one servo, a coupling the model lacks; a PhysX mimic joint would add it. Grip force and closing speed on the
  hardware are unmeasured, so no grip result transfers yet.

### F-052 — The rendered wrist camera sits 11 mm and 1.1° from the pose its authored offset and forward kinematics give it, constant in the Link6 frame, which biased simulated cup estimates by about 3 mm across and 7 mm in height

- **Status:** confirmed (measured); cause unexplained
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [Week 1 log, 2026-09-17](week_01/notes.md).
  - **Everything upstream checks out.** The camera prim's local transform is authored as requested:
    (−0.055, 0, 0.035) m, 20° pitch. Isaac Lab reports intrinsics identical to the D435 preset (fx 616.18,
    principal point 320, 240). Forward kinematics of the simulator's joint angles match PhysX's Link6 pose to 0.0 mm.
  - **The reported camera pose does not.** It sits at (−48.8, −0.9, 26.2) mm in the Link6 frame, rotated 1.1°, the
    same at the look pose and the pregrasp pose ([run](#/week/1/run/20260917T030530_523746Z_pick_seed42)).
  - **The image is rendered from that pose.** Using it as the mount
    ([run](#/week/1/run/20260917T032602_826979Z_pick_seed42)) takes the first-look error from 2.9 / −7.3 mm to
    1.4 / +0.3 mm, and the re-look from 2.3 / −7.5 mm to 0.05 / +0.2 mm.
  - **Not the inertia frame.** A diagnostic with Link6's principal axes overridden to identity left the offset
    unchanged to 0.01 mm.
- **Scope:** Link6 of this welded asset only; other links and whether Link6's visual meshes share the offset were
  not checked.
- **Implication:** simulated perception from an arm-mounted camera here carries a fixed extrinsic error that the
  authored offset does not reveal. The real camera needs a hand-eye calibration anyway; in simulation, measure the
  mount from the render (`--mount_calibration sim`) before quoting camera accuracy. Any G4 camera-frame check done in
  simulation must not assume the authored offset. Isolate the cause before simulated camera numbers are used as
  evidence.

### F-053 — The bench D435i's measured colour intrinsics differ from the datasheet preset the pick has always used: 55.6° horizontal rather than 69.4°, and a principal point 14 px off centre

- **Status:** confirmed (measured)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** `pick_demo/assets/calibration/d435i_238222076237_640x480.json`, captured with
  `python -m pick_demo.realsense calibrate` from the camera on the bench (D435I 238222076237, firmware 5.13.0.55,
  enumerated at USB 2.1).
  - **Colour, 640×480:** fx 607.11, fy 607.37, cx 323.00, cy 254.29 — a 55.6° × 43.1° field of view.
  - **`camera.CAMERAS["d435"]`, the preset every recorded pick used:** fx = fy = 616.18, principal point (320, 240),
    derived by scaling the datasheet's 69.4° at 1920×1080 down to 480 rows.
  - **The scaling model is wrong for this mode.** 640×480 colour is not a resized crop of the 16:9 1920×1080 stream,
    so the preset's field of view is 14° too wide, not merely mis-scaled.
  - **Depth, 640×480:** fx 387.75 (79.1° horizontal, against the datasheet's 87°), stereo baseline 50.05 mm
    (confirming the nominal 50 mm), depth scale 1.000 mm/unit, depth origin 14.857 mm from the colour sensor.
  - **A pick on the measured model still succeeds** in simulation
    ([run](#/week/1/run/20260917T070917_995149Z_pick_seed42)): lift 11.9 cm, cup axis 4 mm from the jaw centre.
- **Scope:** one camera, one resolution, one firmware. Intrinsics are per-resolution, so this says nothing about the
  424×240 or 1280×720 modes, and nothing about the other D435i units. `min_depth` and `max_depth` in the derived
  `CameraModel` are still the disparity-search limit and the datasheet's, not measurements.
- **Implication:** the 14.3 px error in cy is about 9 mm of vertical cup-position error at 40 cm, on the same order
  as the mount error F-052 found — so a real pick that uses the preset carries a perception bias before the bracket
  is even considered. Hardware perception must use `--calibration`, and any simulated camera number quoted as
  applying to the bench camera must say which model produced it. This does **not** retract the earlier simulated
  picks: they were self-consistent, rendering and estimating through the same preset. It means their accuracy
  figures describe a camera that does not exist.

### F-054 — A solid camera housing drawn at the wrist blinded the camera it represented: the optical centre sits 4.2 mm behind the front glass, so a closed front face fills the image

- **Status:** confirmed (measured); fixed
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:**
  - **The failure** ([run](#/week/1/run/20260917T070343_357515Z_pick_seed42)): the RealSense body drawn as one
    90 × 25 × 25 mm cuboid at `WristMount`. All 48 frames across six viewpoints came back flat housing grey, no
    detection reached the 0.4 confidence gate, and the sequence ended at "no cup found from any viewpoint".
  - **Why the near plane did not save it.** The render's near clip is 10 mm and the housing's front face is 4.2 mm
    ahead of the optical origin, so the face was expected to be clipped. It was not; relying on that was the error.
  - **The fix** ([run](#/week/1/run/20260917T070917_995149Z_pick_seed42)): the housing rebuilt as an open-fronted
    shell of five 2 mm panels, with nothing drawn on the optical axis. Same command, same seed: the pick succeeded,
    lift 11.9 cm, axis gap 4 mm.
- **Scope:** the simulated body only. It says nothing about whether the real bracket occludes the real camera, which
  is a question for the bench.
- **Implication:** anything mounted at or near a simulated camera's own frame has to be checked against that
  camera's frustum rather than against a near-plane assumption. `camera_body.view_obstruction` is that check and the
  tests hold it at zero parts for every preset and for the measured model. The physical lesson is the same one a
  lens hood embodies: the optical centre is *inside* the body, so the body must be open where it looks.

### F-055 — The hand-built camera body was mirrored along its long axis: Intel's CAD puts the colour lens 12.5 mm from one end, not 34.8, so the bracket clearance number was wrong by 22 mm

- **Status:** confirmed (CAD, cross-checked against the bench camera)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [Week 1 log, 2026-09-17](week_01/notes.md). Intel's own D435 case mesh
  (`realsense2_description` 4.58.3, Apache-2.0, `meshes/d435.dae`; provenance in
  `third_party/realsense2_description/NOTICE.md`) registered into the colour optical frame by that
  package's `urdf/_d435.urdf.xacro`. The registration checks itself: the mesh's three concentric
  colour-lens parts land at x = 0.00 mm, its left imager barrel at +15.25 mm and its right at
  +65.25 mm, against nominal extrinsics of 0, +15 and +65 mm and measured values on D435I
  238222076237 of 0, +14.857 and +64.90 mm. `tests/test_pick_demo.py` holds those numbers.
- **Scope:** geometry only. The mesh is nominal CAD of the D435 case, which the D435i shares
  (`_d435i.urdf.xacro` includes `_d435.urdf.xacro` unchanged and adds IMU frames); it is not a
  measurement of the case on the bench, and it says nothing about where the bracket actually holds it.
  The mount remains assumed (F-054's scope note still applies).
- **Implication:** `camera_body` had `left_x = -COLOUR_FROM_LEFT_IMAGER_M`, placing the depth origin to
  the *left* of colour. librealsense's `depth.get_extrinsics_to(colour)` returns
  `p_colour = R p_depth + t`, so `t_x = +14.857 mm` is the depth origin's position *in the colour
  frame*: it is to the **right**. With the sign corrected and the stereo pair no longer assumed centred
  on the case, the case runs from −12.5 mm to +77.4 mm about the colour lens, where the old geometry had
  it from −34.8 mm to +55.2 mm. In Link6 the case moved from y ∈ [−55.2, +34.8] mm to
  y ∈ [−77.4, +12.5] mm: 22 mm along the jaw axis, at the default mount, [visible between the old and
  new renders](#/week/1/run/20260917T075946_132477Z). Any bracket sizing done against
  [the earlier close-ups](#/week/1/run/20260917T071238_076387Z) should be redone. The five-panel shell
  of F-054 is replaced by the CAD mesh; F-054's conclusion about the front face stands.
- **What this also exposed:** `clearance_report` checks the case against the Link6 shell in x and z
  only, because those are the extents `grasp.py` carries (`PALM_X_RANGE_M`, `PALM_Z_M`). At the default
  mount the optical x axis maps to Link6 −y, so the case's 90 mm long axis lies along the one direction
  the report does not check — which is why its numbers barely moved when a 22 mm error was corrected
  (x went from [−73.9, −41.8] mm to [−73.8, −41.8] mm). The report now returns `housing_y_link6_m` and
  an explicit `y_checked: false`, but nothing yet checks it: the shell's y extent is not in `grasp.py`.

### F-056 — The rendered wrist camera ignores the principal point and the second focal length it is given: it reports (320, 240) and fx = fy whatever the calibration says

- **Status:** confirmed (measured from the simulator's own reported intrinsics)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** `camera_model_vs_sim.intrinsics_sim` in
  [the pick on the measured calibration](#/week/1/run/20260917T070917_995149Z_pick_seed42) and
  [the pick on the d435 preset](#/week/1/run/20260917T070804_174822Z_pick_seed42). Asked for
  fx 607.11, fy 607.37, (cx, cy) = (323.00, 254.29), the simulator reports fx = fy = 607.24 and
  (320.00, 240.00). Asked for the preset's fx 616.20 at (320, 240) it reports 616.18 at (320, 240).
  `PinholeCameraCfg.from_intrinsic_matrix` does compute and pass
  `horizontal_aperture_offset`/`vertical_aperture_offset`; the rendered camera does not apply them.
- **Scope:** Isaac Lab as installed here (the version in each run's `run.json` packages block), pinhole
  projection, one camera. Not checked against a newer Isaac Lab, and not checked for the overview
  camera or for depth-only configurations.
- **Implication:** every simulated pick since the calibration landed has deprojected with cy = 254.29
  against an image the renderer centred at 240.00 — a systematic 14.3 px vertical offset between model
  and render, about 9 mm at 40 cm, in the same direction and of the same order as the −6.5 mm height
  error that run reports at every look. F-053 said the *preset* was 14.3 px off in cy; this says passing
  the measured calibration does not fix it, because the renderer discards it. It is also a candidate
  contribution to F-052's unexplained 11 mm / 1.1° render-vs-model pose gap. Until it is resolved,
  `--mount_calibration sim` treats the symptom in the mount and leaves the intrinsics mismatched; a
  simulated hand-eye result is not evidence the model and the renderer agree.

### F-057 — The camera-body occlusion guard was asked about the model's eye while the renderer's eye sits 11 mm away inside the case, so it certified a clear view of a marker that was plainly in shot

- **Status:** confirmed (measured)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** [the aborted pick of 07:14](#/week/1/run/20260917T071420_080024Z_pick_seed42) and
  [the pick at 07:09](#/week/1/run/20260917T070917_995149Z_pick_seed42), whose saved wrist frames carry a
  146 × 146 px olive square in every frame, pixel-identical as the arm moves, while the same runs'
  `view_obstruction` reports `[]`. The square appears only in runs launched with `--camera_body`; the
  otherwise identical [07:08 run](#/week/1/run/20260917T070804_174822Z_pick_seed42) with
  `--no_camera_body` has none. Solving the square's geometry from the two runs that used different
  intrinsics (the two shapes agree under one camera model only) gives a 2.7 mm cube at ~12.7 mm depth:
  a `lens_colour_*` marker, which `camera_body` places at 6.55 mm. Converting that run's
  `camera_model_vs_sim` position error into the optical frame puts the rendered eye at
  (+0.9, +8.85, −6.23) mm — 8.9 mm below and 6.2 mm behind the optical origin, which is inside the case.
- **Scope:** one mount and one arm pose. The eye offset was resolved in the optical frame at the
  observation pose only; F-052 reports it constant in Link6 but that was not re-derived here.
- **Implication:** the guard was not wrong about the frame it was given — it was given the wrong one.
  `view_obstruction` now takes the eye offset as an argument and both runners report it twice, at the
  model's eye and at F-052's rendered eye. At the rendered eye the case *is* in shot and the guard now
  says so, which is why the CAD body cannot be carved into a clear view: a cone wide enough for an eye
  anywhere within 11 mm removes 28% of the mesh. Drawing the body and looking through the wrist camera
  are mutually exclusive until F-052 is resolved; `--no_camera_body` is the setting for a pick that
  needs the wrist view, and the body belongs in `run_camera_body_view.sh`, which looks from outside.

### F-058 — The bench pick stops at grasp planning, not at perception: it sees the cup at 0.92–0.96 confidence and refuses it as too wide, and the width it believes depends on how much of the rim it saw

- **Status:** confirmed (measured, three runs on one mug)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** three LIVE hardware picks minutes apart, same mug, arm on the bench
  ([1](#/week/1/run/20260917T075204_004370Z_pick_hw), [2](#/week/1/run/20260917T075242_436661Z_pick_hw),
  [3](#/week/1/run/20260917T075300_152735Z_pick_hw)).
  - **Detection is not the problem.** YOLO returned the cup at 0.93–0.96 confidence every time, the rim
    circle fitted to 0.56–0.76 mm RMS on 6,600–9,400 points, and the three frames within a run agreed on
    the radius to better than 1 mm. The arm moved: 7, 22 and 34 commands were sent, reaching viewpoints.
  - **Every run ended at the same gate**, `plan_top_down_grasp`'s width check, before any pregrasp move.
  - **The width tracked the rim coverage, not the cup:**

    | run | rim coverage | diameter | outcome |
    |---|---|---|---|
    | 2 | 63° | 83 mm | refused |
    | 1 | 66° | 79 mm | refused |
    | 3 | 126° | 70 mm | refused |

    A circle fitted to a short arc is ill-conditioned and reads too wide. The simulated picks that
    succeeded saw 192°. Run 3 also reported the cup as 157 mm tall, which no mug is, so coverage is not
    the only thing wrong with a sliver look.
  - **The mug was never measured with a ruler**, so which of 70–83 mm is right is still open.
- **Scope:** one mug, one bench session, an uncalibrated wrist mount (so absolute position carries the
  mount's error), and estimates from a single viewpoint per run.
- **Implication:** "the arm sees the cup but does not try to grab it" is the planner declining a grasp it
  believes impossible, not a failure to reach. Two changes follow: a first look now needs
  ≥ 100° of rim before its circle may set the grasp width (`CupPerception.MIN_RIM_COVERAGE_DEG`), and the
  width gate no longer refuses at a fixed 67.2 mm — see F-059. Any cup width quoted from a single look
  should carry its rim coverage beside it.

### F-059 — The gripper's pads bottom out 17.26 mm apart, so it cannot pinch a cup wall; the fixed 10 mm width margin, not the jaws, was refusing a cup the jaws can hold

- **Status:** confirmed (from the CAD meshes)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** finger mesh vertices in the Link6 frame at zero travel (`workspace.gripper_points`):
  Link7_1 spans y −29.65…−8.63 mm and Link7_2 +8.63…+29.64 mm, so the pads' inner faces meet at ±8.63 mm
  and the **closed jaw gap is 17.26 mm**, open 77.2 mm. This confirms the figure `grasp.py` carried as a
  comment.
  - **A wall pinch is out.** Grasping a cup by one wall — one finger inside, one outside — needs the jaws
    to close below the wall thickness. A mug wall is 5–8 mm, so the fingers would straddle it with about
    5 mm of slack a side and never touch it. No closing motion grips anything thinner than 17.3 mm.
  - **The cup was already inside the jaws' range.** At 70 mm it sits between the 17.26 mm closed gap and
    the 77.2 mm open one. What refused it was `plan_top_down_grasp`'s fixed `OPEN_GAP_M - 0.010`, a
    software margin guarding the descent: the jaw centre may stray `line_tolerance_m` (6 mm) from the
    descent line, and a 70 mm cup leaves only 3.6 mm a side.
  - **The margin is now derived rather than fixed.** `descent_tolerance_for` spends the available
    clearance on a finer descent — 6.0 mm for a 55 mm cup (unchanged, so every earlier simulated pick
    plans identically), 2.6 mm for a 70 mm one — and refuses only a cup that leaves no room at all
    (≥ 75.2 mm). Holding 2.6 mm over the 18 cm descent needs 36 segments, one more than the old cap of
    32, so `line_max_segments` is now a parameter at 64.
- **Scope:** the **URDF CAD** gripper, not a measurement of the real one — `grasp.py` has always said the
  gripper geometry is CAD. The real D1's closed gap has never been measured, and neither have servo 6's
  units, so the real jaws may close further than this.
- **Correction, 2026-09-17 (same day):** they do. Lukas backdrove the powered-down gripper by hand and
  reports that **the pincers can be pushed until they touch**, and pulled about 10 mm further apart than
  the commanded maximum, with no change in resistance at either end. So the real finger rails span a
  wider range than the URDF models, and the CAD's zero travel is not the real mechanical stop.
  The measured claim above still stands — the *CAD* pads meet at ±8.63 mm — but two conclusions drawn
  from it do not, and are withdrawn here:
  - **"A wall pinch is out" is withdrawn.** It follows only for the modelled gripper. A real jaw that
    closes to touching can pinch a 5–8 mm cup wall, so a rim grasp is back to being an open question
    about reach and control rather than one settled by geometry.
  - **"No rim or wall grasp is worth planning against the modelled gripper" stands as written**, but it
    is now a statement about the model's limits, not about what the arm can do.
  It also puts a correctness problem in front of any real grasp: `grip_travel_m` subtracts
  `CLOSED_GAP_M` = 17.2 mm when working out how far to close, because the CAD jaws start 17.2 mm apart.
  If the real jaws start at zero, that arithmetic asks the fingers to travel about 8.6 mm each too far
  and would drive them into the cup rather than onto it. **No computed grip width should be trusted on
  hardware until the jaw gap is measured against servo 6 at a few points.**
- **Update, 2026-09-17 (same day, after F-063 and the wall-grasp work):** both halves of this are now
  settled, and neither the way this finding guessed. The real jaws **do** shut to touching, under command
  and not just by hand (F-063), so a wall pinch is available on the arm. And a wall grasp *is* worth
  planning against the modelled gripper, in the sense this finding did not consider: with the fingers
  *inside* the cup their outer faces span 39.2-99.2 mm, so a cup too wide to straddle can be held from
  within without the pads ever needing to meet. Both are implemented (`GraspParams.wall_grasp`) and the
  pinch lifts a 90 mm cup in simulation (F-065). The sentence below stands only as what was believed of
  the modelled gripper before anyone looked inside a cup.
- **Implication (superseded by F-065):** no rim or wall grasp is worth planning against the modelled gripper. Whether one is
  possible on the real arm turns on a single unmade measurement — command servo 6 across its range and
  measure the jaw gap — which is the same experiment that would unblock commanding the gripper at all.
  Until then a hardware pick positions but cannot close. The wide-cup path is validated by unit tests
  only; no simulated pick has yet run with a 70 mm cup.
- **Update, 2026-09-17 (same day):** two of the closing sentences above have been overtaken by work, and
  the measured claims are unaffected. A simulated pick **has** now run with a 70 mm cup and succeeded
  ([run](#/week/1/run/20260917T091408_329082Z_pick_seed42)): 5.6 mm a side, descent held to 4.6 mm over
  18 waypoints, lift 11.9 cm, cup axis 6 mm from the jaw centre. And a hardware pick now **does** command
  the gripper, carried as `angle6` on the same message as the arm pose, on the protocol's own scale
  (`d1_hardware.GRIPPER_UNITS_RANGE`, 0–65) — so "positions but cannot close" is no longer true of the
  code. It remains true that **nobody has measured what a unit is**: the conversion used is the
  simulator's own (finger travel × 2000), the console has a jog control for measuring the real thing, and
  until someone puts a ruler across the fingers at a known command no run may quote a jaw width in
  millimetres.

### F-060 — Servo 6 saturates at 50.2 of the protocol's advertised 65, and tracks a command with a steady +0.2 offset below that

- **Status:** confirmed (measured, nine commands); direction of travel not yet observed
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** the console's gripper jog on the bench arm, first commands ever sent to servo 6 on this
  arm (`/tmp/d1_ui_bench.log`, and `/tmp/d1_ui_log.jsonl` via the server's command record):

  | commanded | settled at |
  |---|---|
  | 40.0 | 40.2 |
  | 4.0 | 4.2 |
  | 0.0 | 0.1 – 0.2 |
  | 65.0 | **50.2**, three times |

  - **The advertised range is not the reachable one.** `d1_direct` carries the vendor claim of a 65 mm
    jaw. Commanded 65 the servo stops at 50.2 and stays there, identically on three separate commands
    minutes apart, so the usable span is 0 – 50.2.
  - **Below saturation it tracks closely**, with a repeatable +0.2 offset (40 → 40.2, 4 → 4.2), the same
    sign and size at both ends of the range tested.
- **Scope:** one arm, one firmware (5.13.0.55), nine commands, no load in the jaws, and **no ruler**. The
  jaw gap in millimetres at any of these values is still unmeasured, and so is which end of the span is
  open — nobody has yet watched the fingers while a command was sent.
- **Note, same day:** the 50.2 ceiling is a limit of the *command path*, not of the mechanism. Lukas
  backdrove the powered-down fingers and found roughly 10 mm of opening beyond where a command leaves
  them, and closure all the way to touching, with no change in resistance at either end (see the
  correction on F-059). So servo 6's commandable span covers only part of the physical rail, and
  whatever a unit turns out to be worth, the arm does not offer the whole stroke through this interface.
- **Implication:** `finger_travel_to_gripper_units` now maps the URDF's 0–30 mm of finger travel onto the
  measured 0–50.2 span rather than the advertised 0–65, so "fully open" asks for what the arm can
  actually give; linearity between the endpoints is assumed, not shown. The command clamp stays at the
  protocol's 0–65 so the jog control can still probe past the span. **Before anything is grasped in
  earnest, check by eye which end is open**: the mapping assumes zero travel is a closed jaw, following
  the simulator's convention, and if the arm reads servo 6 the other way round a pick would open on the
  cup at the moment it means to close. This finding measures the command scale, not the jaw: no run may
  quote a width in millimetres until someone measures one.

### F-061 — One malformed DDS sample killed the console's state thread, and the page went on showing six-minute-old joint angles while still reporting the arm as connected

- **Status:** confirmed (observed once, cause identified); fixed
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** `/tmp/d1_ui_bench.log` during a bench session.
  - `Exception in thread d1-ui-state` … `AttributeError: 'InvalidSample' object has no attribute
    'servo0_data_'`, raised out of `D1Client.poll` at 18:52. A DDS reader returns more than data: when an
    instance's state changes, it yields a sample carrying only that notification, and cyclonedds-python
    surfaces those as `InvalidSample`. `poll` read `servo0_data_` off one.
  - **The thread died and nothing noticed.** Six minutes later `/state` still answered
    `connected: true` with `feedback_age_s: 358.9`, and the page was drawing the arm's last known pose.
    A gripper command sent in that window moved the fingers about a centimetre, observed by eye, while
    the reported value never changed.
  - **`connected` was the second half of it.** It read
    `servo is not None and (self.mode == "hardware" or age < 1.0)` — in hardware mode the age was
    ignored entirely, so a frozen cache reported as a live arm by construction.
  - **Version-dependent.** The same code ran for weeks on the dog's cyclonedds 0.10.2 without this; the
    workstation has 11.0.1, installed 2026-09-17 to reach the arm from the PC.
- **Scope:** one occurrence, on the workstation. What triggers an invalid sample on this arm's topics was
  not established, so the frequency is unknown; the guard does not depend on knowing.
- **Implication:** the fix is in two places, because either alone leaves the failure silent. `poll` skips
  samples that carry no data (checking `sample_info.valid_data` and the attribute), and the console's
  state loop catches everything, logs once and keeps going — a console that says it has lost the arm is
  far better than one that quietly freezes. `connected` now requires feedback newer than 1 s in every
  mode. **Anything that reads a pose off this console and acts on it must check `feedback_age_s`**: this
  bug made the page confidently wrong about where a real arm was, which is the worst failure a
  teleoperation surface has.

### F-062 — Servo 6's scale is monotonic and the code's first mapping ran the wrong way: more units is a wider jaw, not a narrower one

- **Status:** confirmed (observed on the arm)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** commanded and watched on the bench arm.
  - **At 0.1–0.2 units the pincers stand as far apart as the rails permit** (Lukas, looking at the arm).
  - **Commanded 65 from there, they closed by about a centimetre** — seen by eye, not inferred, and
    initially invisible in the telemetry because the feedback thread had died (F-061). Repeated with
    feedback live, the command settles at 49.8–50.2.
  - So units increase as the jaw closes, and the commandable span (0 → ~50) covers roughly 10 mm of
    finger motion.
  - **The first mapping was inverted.** `finger_travel_to_gripper_units` followed the simulator's
    convention, where travel counts opening from a shut jaw, and mapped zero travel to zero units. On
    this arm zero units is *open*, so a pick would have opened the jaw at the moment the sequence meant
    to close it on the cup.
- **The commandable window is inset at both ends, and it is a clamp rather than a stop.** Lukas: the
  gripper "still doesn't close fully or open fully", and after pushing it by hand past either extreme
  "the servo is able to move it out of that extreme to whatever I commanded, but only within that
  clamped range". So the servo has authority over the whole rail — it can pull the fingers back in from
  outside the window — while the command path will not take them out of it. The window is therefore a
  limit in the firmware or the protocol's interpretation of servo 6, not a mechanical end stop, and it
  sits inside the rails at both ends.
- **Scope:** one arm, one firmware, five commanded values, and eyes rather than a ruler. The gap in
  millimetres at any unit is still unmeasured, and so is whether the relationship is linear. Whether the
  window is fixed in the servo's own coordinates or re-derived at power-on has not been tested, and
  nothing has yet been commanded outside 0–65 to see whether the clamp is in the arm or in this client.
- **Correction, same day, after F-063's sweep.** Two claims above came from watching the fingers while
  the console's feedback was dead (F-061), when the numbers being quoted alongside them were stale, and
  the sweep that followed contradicts both. **Withdrawn:** that 0 units is the fingers at the far end of
  their rails, and that the positive half of the scale closes the jaw. A ladder run in one process from
  -19.8 up to +50, watched throughout, opens monotonically the whole way: "it started closed and then
  got up to the old clamped extent". What stands is the part this finding was really about -- the first
  mapping had the sense reversed, so a pick would have opened the jaw where it meant to close. The
  endpoints are corrected in F-063; the lesson is that an observation paired with dead telemetry is
  worth less than it looks, and three inferences here were built on one.

- **Implication:** the conversion now maps full travel to the open end and zero travel to the closed end,
  and a test states the direction as behaviour — closing on a cup must ask for *more* units than holding
  the jaws open — so the inversion cannot come back silently. The commandable span does not reach a shut
  jaw: the fingers can be pushed further closed by hand than any command takes them (F-059 correction),
  so "zero travel" asks for as shut as this interface goes, not for shut.

### F-063 — The gripper's closing half is negative, outside the range the vendor driver advertises: the jaw runs from −19.8 (pads touching) to +50.2 (widest a command reaches)

- **Status:** confirmed (probed on the arm, closure watched by eye)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** `logs/gripper_probe/close_probe.json` and `open_probe.json`, stepping servo 6 past the
  advertised range with an abort on any error, stale feedback, or two steps without movement.
  - **Upward, from 50.2:** commanded 55.2 and 60.2, moved 0.00 both times, error 0 throughout. The
    arm accepts the command and simply does not act on it, so the clamp is in the **arm**, not in this
    client — which had been the open question.
  - **Downward, from 0.2:** −4.8 → −4.60, −9.8 → −9.60, −14.8 → −14.50, −19.8 → −19.50, then −24.8 →
    −19.80 and −29.8 → −19.70. It tracks to about −19.8 and clamps there.
  - **At −19.7 the pads touch.** Observed directly: "it just closed fully". Nothing had ever commanded
    below zero, because `d1_direct` carries the vendor's advertised 0–65 and `d1_hardware` clamped to
    it, so every command that would have shut the fingers was floored at 0 — two thirds of the way open.
  - **The scale is monotonic.** A ladder of −19.8, −10, 0, +10, +25, +50 run in a single process and
    watched throughout: "it started closed and then got up to the old clamped extent". So more units is
    a wider jaw, all the way, and +50.2 is the widest a command reaches — about a centimetre short of
    the physical rail end, which the powered-down fingers can still be pushed to by hand.
  - **An earlier reading of this was wrong and is withdrawn** (see the correction on F-062): that 0 was
    the widest point and that the positive half closed the jaw. Both came from watching the fingers
    while the console's feedback was dead (F-061) and the numbers quoted beside them were stale.
- **Scope:** one arm, one firmware (5.13.0.55), two probes in 5-unit steps and one six-point ladder,
  with the jaw judged by eye and not with a ruler. The millimetres per unit, the linearity, and whether the window is fixed in the
  servo's coordinates or re-derived at power-on are all still unmeasured.
- **Implication:** the usable jaw command is **−19.8 (touching) to +50.2 (widest)**, a 70-unit span, and
  the closing half of it is negative — the half the vendor's advertised range excludes and that nothing
  in this repository could previously reach. `GRIPPER_UNITS_RANGE` now spans
  −25 to 65 so the closing half can be reached at all. It also settles the question F-059's correction
  raised: the real jaws **do** close to touching, so a rim or wall pinch is mechanically available on
  this arm even though the CAD says the pads stop 17.26 mm apart. `grip_travel_m` still subtracts that
  CAD gap and so remains wrong for hardware until the jaw gap is measured against servo 6 with a ruler.

### F-064 — A command sent before the arm has discovered the writer is lost silently, and live feedback is no guarantee that it has

- **Status:** confirmed (measured, two losses in eight commands; fixed)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** single-shot `d1_hardware gripper --units N --execute` invocations against the bench arm.
  - **Two of eight vanished.** Commanded 0 from −10.1 and commanded 50 from −19.5: the arm stayed
    exactly where it was, error 0, feedback live and under 0.1 s old throughout. No error was raised
    anywhere; the write succeeds locally and reaches nobody.
  - **Feedback proves the wrong direction.** DDS discovery is per-endpoint, so the arm's *writer* can be
    matched to our reader — feedback flowing, joint angles printing, `connected` true — while our
    *writer* is not yet matched to the arm's reader. Each of the lost commands came from a process that
    had already printed live joint angles.
  - **It made a measurement look like physics.** A six-step sweep with one step silently dropped read as
    a non-monotonic jaw, which is where the two-sided opening model in F-062 and F-063 came from.
  - **After waiting for the match, four of four landed**, alternating −19.8 and 50.
- **Scope:** one arm, one session, cyclonedds 11.0.1 on the workstation. The loss rate will depend on
  timing and on how long the process lives before its first write, so two in eight is an observation,
  not a rate.
- **Implication:** `D1Client.wait_for_writer` polls `get_matched_subscriptions` before the first command
  and the CLI calls it after `wait_for_feedback`, warning if the match never arrives. Every short-lived
  process is exposed — `move`, `park`, `gripper` — while a long-running one like the console risks only
  its first command. More generally: **on this arm, "the command had no effect" and "the command was
  never delivered" look identical**, and a sequence of one-shot invocations is a bad way to measure
  anything. Sweeps belong in a single process, which is what `gripper --ladder` is for.

### F-065 — A cup too wide for the jaws can be picked up by its wall: in simulation a pinch of a 90 mm cup lifts it 11.9 cm, and it only works because the pads may shut past the URDF's stop

- **Status:** confirmed in simulation only; never attempted on the arm
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** four viewer runs on one 90 mm cup with a 5 mm wall, 10 cm tall, at (0.42, 0.03) m, the Go2
  lying down at 8.5 cm ([1](#/week/1/run/20260917T100010_537902Z_pick_seed42),
  [2](#/week/1/run/20260917T100229_264853Z_pick_seed42),
  [3](#/week/1/run/20260917T101241_612167Z_pick_seed42),
  [4](#/week/1/run/20260917T101442_504536Z_pick_seed42)).
  - **The pinch lifts the cup.** One finger 30 mm inside it and one outside, the jaw centre on the wall
    39.8 mm from the axis, descending with 7.0 mm either side of the wall and then shut to 2 mm on it:
    **lift 11.9 cm**, the cup's axis ending **0.8–1 mm** from where the plan put it, twice in a row.
  - **It works only because the modelled pads were allowed to meet.** The URDF stops each finger at zero
    travel, where the CAD pads are still 17.26 mm apart (F-059) — wider than any cup wall. The arm itself
    shuts until they touch (F-063), so the simulated finger limits are widened to match and the plan
    commands −7.6 mm of travel. Held to the URDF's own stop (`--pinch_closed_gap 0.0172`) the planner
    refuses the pinch and falls back to the inside-out grasp, which is the honest behaviour for that
    gripper rather than a fallback of convenience.
  - **Pressing on the near edge of the wall estimate grips nothing** (run 2). The inside-out grasp opened
    the fingers to the rim fit (42.1 mm) minus a whole assumed 5 mm wall, against an inside really at
    40.0 mm: they reached 39.0 mm, touched nothing, and the arm lifted away from a cup it had gone 30 mm
    into. The rim circle is fitted to points lying anywhere between the lip's inner and outer edges, so
    the inside is in [fit − wall, fit]; the descent must assume the near end and the press the far one.
  - **A steeper first look sees more rim.** At 65° elevation the camera sits at x = 0.26 m, inside the
    Go2's own 0.30 m body box — the arm folds back over the dog and peers down at its head. At 75° it
    stands at x = 0.36 m, past the nose. On the same cup from the same 0.35 m the steeper view returned
    **230° of rim at 0.93 confidence** against 170° at 0.91. Steeper still is worse, not better: see the
    update below.
- **Scope:** simulation, one cup, one position, one seed. The cup is a ring of 24 boxes on a disc, not a
  vessel; its wall thickness is known to the simulator and **assumed** by the planner
  (`wall_thickness_m`, 5 mm), which perception cannot see. The jaw gap the pinch shuts to is the 2 mm
  Lukas **stated**, not a gauge reading, and `grip_travel_m` still subtracts the CAD's closed gap on
  hardware (F-063). Nothing here was measured on the arm, and no wall grasp has been attempted on it.
- **Update, 2026-09-17 (same day), on the viewpoint and on what reaches the arm:**
  - **A cup seen exactly end-on is not detected at all.** Pushing the first look to 85° elevation — the
    only elevation that carries the *jaws*, as opposed to the camera, past the front of the trunk — gave
    **zero detections from three viewpoints running**, where 75° had given 0.93 confidence. Seen straight
    down a cup is a ring, and a first look only accepts the `cup` label. The run recovered by falling
    through to a 75° look at a further floor point and succeeded there
    ([run](#/week/1/run/20260917T102640_482962Z_pick_seed42), lift 11.9 cm, rim coverage 359°).
  - **The first look is now a survey pose** (`grasp.plan_survey`), described by a camera height and a
    pitch rather than by a floor point at a fixed distance. It holds the floor from **0.23 m to 0.96 m**
    in one frame, against the ~0.20 m patch a near-vertical look sees, and the close scan stays as the
    fallback. Where the wrist sits stopped being a constraint: looking down *and* forwards, a wrist over
    the dog's back is useful height rather than a fault.
  - **Two things stood between the plan and the servo, and neither needed a ruler.**
    `finger_travel_to_gripper_units` took `abs` of its argument, so the pinch's −7.6 mm (a jaw shut past
    the CAD's stop) became −2.07 units, about a quarter open — the arm would have opened its jaws on the
    wall. And the plan's 19 mm descent gap is CAD travel +0.9 mm, which that mapping sends to −17.7
    units, nearly shut, because the travel-to-gap relation on the arm is not the CAD's. The sign is
    fixed; the descent is now **as wide as the cup allows**, so a pinch commands only the two ends of the
    scale that F-063 measured (+50.2 down, −19.8 shut) and no intermediate opening at all.
- **Implication:** the width gate that stopped every bench pick (F-058) is no longer the end of the
  attempt — a cup between about 75 mm and the fingers' reach now gets a wall grasp instead of a refusal.
  A pinch no longer needs the unmeasured millimetres per unit — it uses only the two calibrated ends —
  so what stands between this and the bench is the cup's own numbers, not the gripper's. The ruler is
  still owed for everything else: it would replace both the stated 2 mm here and the CAD gap inside
  `grip_travel_m`, which an ordinary outside grasp still depends on. The wall
  thickness is the other assumption worth attacking, and it is the one perception could in principle
  measure — a rim seen from two viewpoints has an inner edge as well as an outer one.

### F-066 — Four live bench picks "stalled" without the arm being at fault: the sequence charged its own planning time to the arm's move deadline, and a trembling hold never counted as an arrival

- **Status:** confirmed (four LIVE runs on the bench arm, one of them decisive)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** four executed picks on the bench D1, mug at 0.29 m
  ([1](#/week/1/run/20260917T105029_482472Z_pick_hw), [2](#/week/1/run/20260917T105145_214493Z_pick_hw),
  [3](#/week/1/run/20260917T105326_457783Z_pick_hw), [4](#/week/1/run/20260917T111006_406673Z_pick_hw)).
  Every one ended at an arrival test, and Lukas reports hearing the arm working while it "doesn't move".
  - **Run 4 is decisive: one command sent in 10.09 s.** At the loop's 10 Hz that should be about a
    hundred. `commands_sent: 1`, `covered 0% of the move, still moving`, every joint 0.3–1.2 rad from
    target. The arm had been told where to go exactly once, and was declared timed out on the next cycle.
  - **The mechanism.** `Motion` set its deadline when the move was *constructed*, and the planning that
    produces the move runs in the same `update` call — `plan_survey`, or a grasp plan that is 18
    waypoints of inverse kinematics. Seconds of the arm's budget were spent before it was commanded at
    all. The deadline now starts on the first `update` that actually commands the waypoint.
  - **The other half is the stillness test.** Run 3 reached its pregrasp to **0.008 rad** — 0.46°, well
    inside the 0.03 rad tolerance — and still timed out, because a final waypoint also had to go still:
    a spread under 0.004 rad across 0.35 s. An arm holding a pose against gravity trembles by more than
    that, audibly. Holding inside tolerance for 0.8 s now counts as arrival; the spread test stays as
    the quick path.
  - **What the runs show about the rest of the pick, which worked.** The survey pose found the mug from
    its first look at **0.93 confidence with 133° of rim**, and the planner chose a **wall pinch** on it:
    25 mm inside the cup, jaws 77 mm apart going down, shut to 2 mm — commanding only the two calibrated
    ends of servo 6 (+50.2, −19.8), with the gripper reported at 49.8 before the first command.
- **Scope:** one arm, one bench session, four runs, and a defect in this repository rather than a
  property of the hardware. No grasp was attempted: all four ended before the descent.
- **Implication:** a timeout in this sequence was not evidence about the arm, and the three earlier runs
  were read that way for an hour. The failure message now carries per-joint errors, the fraction of the
  move covered and whether the arm is still moving, which is what separated "trembling at the target"
  from "never commanded". Any future arrival timeout should be read from those numbers before the arm is
  suspected. It also leaves an open question worth a measurement: run 2 was 0.555 rad short with a full
  gripper stroke in the same message, so whether the firmware paces a coordinated move to its slowest
  axis is still unknown.

### F-067 — On the lying Go2, the survey pose over the dog's back cannot see past its head: a cup straight ahead is hidden, and a half-hidden one was measured 74 mm too high

- **Status:** confirmed (simulation)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** simulated picks, seed 42, D435 preset, the saved wrist mount, no camera housing (Week 1 log,
  "Searching for the cup by pivoting the arm").
  - **Hidden.** The survey stands the camera 5 cm ahead of the arm's mount, 0.40 m up, tilted 45° down. From
    there the head covers a cup 0.42 m straight ahead except for its rim: nothing detected in eight frames, nor
    from the four pivot stops, and the close scan found it after 28.7 s
    ([run](#/week/1/run/20260917T130232_291448Z_pick_seed42)). The survey had never run in simulation before
    this; it was developed on the bench, where there is no dog.
  - **Half-hidden, and worse.** Across 11 cups spread over ±45°
    ([run](#/week/1/run/20260917T130336_209998Z_pick_seed42)), the three within ±5° were found only by the
    close scan. One at −4° was detected from the survey, but with no rim to fit: the silhouette estimate put
    the rim 74 mm too high. The jaws closed 7 cm above the cup, and the sequence still ended `done` with no lift.
  - **The trunk proxy agrees.** From the old camera, at the trunk box's front edge, the line of sight to the
    rim is 7.6 cm up and to the cup's middle 3.8 cm, against the box's 6 cm top.
  - **Moving the camera past the head fixes it.** Candidate poses were scored on the CPU model for 76 test
    cups at 0.35–0.50 m and ±45°, each needing to be whole in frame, clear of the fingers and in sight past
    the trunk from one of the five pivot stops. The old pose sees 26 of them; 0.50 m up, 20 cm ahead of the
    mount and 60° down (`grasp.SURVEY_PAST_THE_HEAD`) sees 63. With that pose the default cup is found on the
    first look and lifted in 13.9 s ([run](#/week/1/run/20260917T131112_984198Z_pick_seed42)). The same 11
    placements went 11 of 11 ([run](#/week/1/run/20260917T131152_212357Z_pick_seed42)), with every first look
    a rim fit within 1.7 mm horizontally, and the median pick 13.5 s against 17.3 s.
- **Scope:** simulation only, one seed, one lighting, a plain floor and one white 55 mm cup, placed 0.40–0.48 m
  out. The rendered head is the Go2 model's, and the CPU proxy is a box. Cups at 0.35–0.40 m are predicted to
  stay hidden at some bearings, and no run tested that. The bench survey is unchanged, and the new pose's
  reachability from a table-mounted arm is not checked. With the new pose the cup axis ended 1.8–3.0 mm from
  where the plan put the jaw, against 0.2–1.0 mm before; the cause is not isolated.
- **Implication:** a first look for a robot with a body in front of its camera has to be planned with that
  body in the line of sight, not just with the floor in frame. A half-hidden object is the dangerous case,
  because it gets detected and then measured wrongly. That argues for two checks the pick does not have: a
  sanity bound on an estimate's height against the floor (74 mm is most of the cup), and a grasp check after
  the lift. On hardware nothing would catch episode 3's `done`.

### F-068 — Pivoting the survey pose on Joint1 to ±45° finds cups the straight-ahead look never sees

- **Status:** confirmed (simulation; not run on the arm)
- **Week:** 1
- **Date:** 2026-09-17
- **Evidence:** the same runs as F-067 (Week 1 log, "Searching for the cup by pivoting the arm").
  - **What the search does.** The survey pose is turned on Joint1 alone to 0, ±22.5° and ±45° of view about
    gravity, stopping to look at each. The stops are no further apart than half the 54.9° image, so any
    heading in ±45° falls in the middle half of some frame. Joint1 is solved for each heading, because the
    base lies 7.3° nose-up and 45° of view needs 47.3° of Joint1; the stops landed at 22.44° and 44.91°.
  - **With it.** Every pick of a cup at 40–45° was found at a ±22.5° stop and lifted, 11.6–11.9 cm: 11 picks
    of 6 placements, under both survey poses. For example, a cup 40° left
    ([run](#/week/1/run/20260917T125945_370832Z_pick_seed42)) and one 45° right
    ([old pose](#/week/1/run/20260917T130137_781875Z_pick_seed42),
    [new pose](#/week/1/run/20260917T131805_371086Z_pick_seed42)).
  - **Without it.** The same cup 40° left was never seen with `--sweep_deg 0`, by the survey or any of the six
    close-scan views, under either survey pose
    ([old](#/week/1/run/20260917T130048_072754Z_pick_seed42), [new](#/week/1/run/20260917T131716_886055Z_pick_seed42)).
  - **Cost.** A stop that sees nothing takes 2.4–2.5 s of simulated time (the move, 0.3 s of settling, eight
    frames), and 3.6 s after the 67.5° swing from +45° to −22.5°. A cup at 40–45° was picked in 15.5–23.4 s,
    against 13.0–15.0 s for one the survey sees.
- **Scope:** simulation, one seed, one white cup on a plain floor, 11 random placements at 0.40–0.48 m that
  reached ±42°, plus single runs at 40° and 45°. The search is off on the bench (`sweep_deg` defaults to 0 in
  `PickSequence`) and has not moved the real arm. Stop-and-look is assumed; a continuous sweep was not tried,
  because the camera pose would come from 9 Hz feedback that lags a moving arm.
- **Implication:** the region a lying robot can pick from is limited by what its first look sees, and pivoting
  roughly doubles that across (the straight survey frames about ±25°). Before the bench, the new survey pose's
  reach from the bench mount and the swing's clearance of whatever is beside the arm need checking.

### F-069 — The pick needs no floor height: with every pose placed from the arm's own mount, the same 11 placements are picked identically, and the fingertips stop inside the cup instead of above a stated plane

- **Status:** confirmed (simulation; not run on the arm)
- **Week:** 1
- **Date:** 2026-09-18
- **Evidence:** Week 1 log, "The pick no longer asks how high the floor is".
  - **What was removed.** A clearance proxy that kept every link 3 cm above a plane at `z = -base_height`,
    and the floor-relative aiming of the survey, the pivots and the close scan. The height was the
    simulator's true root pose on the Go2 and an operator's number on the bench, where it moved three times
    on 2026-09-17 (0.02 → 0.0 → −0.09 m). `ground_under`, which let a measured cup base lower it but never
    raise it, existed to soften exactly that.
  - **What holds the fingers out of the surface instead.** They are placed from the rim the camera measured:
    35 mm below it for an outside grasp, less for a wall grasp, so on a cup at least 35 mm deep they stop
    inside the cup, above its base, wherever it stands.
  - **Identical in simulation.** The same 11 placements across ±45°, seed 42, picked
    [11 of 11](#/week/1/run/20260917T224400_609091Z_pick_seed42) as they were with a floor height
    ([2026-09-17](#/week/1/run/20260917T131152_212357Z_pick_seed42)): every cup found by the same look, lifts
    within 0.4 mm, cup axis to jaw centre within 0.3 mm, time per pick within 0.2 s, median 13.48 s in both.
    Also [6 of 6](#/week/1/run/20260917T224041_060489Z_pick_seed42) with the cup ahead and
    [11 of 11](#/week/1/run/20260917T224941_941015Z_pick_seed42) with the continuous sweep: 28 picks, 28
    successes, with no close scan to fall back on.
  - **No closer to the floor.** On the CPU model, across every planned waypoint and traversal of a 28-cup
    sector sweep, the lowest point of the arm's link segments is 6.5 cm above the real floor; the proxy that
    was removed wanted 3 cm. 26 of those 28 plans are identical in mode, tilt and geometry to the old
    planner's, and the same 2 are refused.
  - **It also unblocks a cup the stated floor refused.** On a bench-like rig a 45 mm-deep cup now plans, with
    the fingertips 13.6 mm above the table, inside the cup.
  - **Across the whole graspable sector, not just the ring the earlier runs sampled.** 20 placements drawn
    over 0.34-0.52 m and the full ±45° ([run](#/week/1/run/20260917T234929_503276Z_pick_seed42)): 17 of 20.
    All 5 at 0.34-0.40 m -- the near end, where the arm reaches lowest and the removed proxy bound hardest --
    were found and picked, as were all 11 in the old 0.40-0.48 m band. The 3 failures are all at 0.495-0.520 m:
    two grasps refused as unreachable and one lift that stopped 0.078 rad short. None of them is a clearance
    failure.
- **Scope:** simulation and the CPU model, one seed, one white cup, one lighting, a plain floor; 11 random
  placements at 0.40–0.48 m reaching ±42°, and 20 more over 0.34–0.52 m across the full ±45°. Nothing has run on the arm since the change, and the bench's own
  survey pose moves (the default grid now stands the camera 0.26 m above the mount, 5 cm ahead). The
  35 mm assumption is stated, not enforced: nothing refuses a saucer, and a cup shallower than the fingers'
  reach would have them driven into whatever it stands on.
- **Implication:** the one quantity in this pipeline that nothing on either robot could measure is no longer
  needed, so a bench console starts with nothing to configure and a wrong number can no longer refuse
  reachable cups or plan against a floor below the table. What the arm is trusted to avoid is now what the
  camera actually sees, which makes the wrist mount's unmeasured error the next thing that matters.

### F-070 — The simulator does not render the calibrated camera: it averages fx and fy and centres the principal point, so the F-053 simulated pick perceived through intrinsics 14.3 px off its own images

- **Status:** confirmed (simulation)
- **Week:** 1
- **Date:** 2026-09-18
- **Evidence:** [Week 1 log, 2026-09-18](week_01/notes.md), "The simulated camera is not the calibration".
  - **What was rendered.** The [F-053 pick](#/week/1/run/20260917T070917_995149Z_pick_seed42), the one simulated
    run on the bench D435i's calibration, records the rendered camera in every look (`intrinsics_sim` in
    `events.json`): fx = fy = 607.24, principal point (320.0, 240.0). The calibration it perceived through is
    fx 607.11, fy 607.37, principal point (323.00, 254.29).
  - **Why.** `PinholeCameraCfg.from_intrinsic_matrix` cannot express either difference, and says so at spawn:
    "Camera non square pixels are not supported by Omniverse. The average of f_x and f_y are used" and "Camera
    aperture offsets are not supported by Omniverse. c_x and c_y will be half of width and height". Seen with
    the same Isaac Lab install while building VIP-Rescue's rescue sim, which spawns this camera the same way.
  - **Every other simulated pick is unaffected.** They used the `d435` preset, whose principal point is the
    image centre and whose fx = fy: rendered and modelled intrinsics match exactly (616.18, 320, 240) in all
    four runs checked, including [F-069's](#/week/1/run/20260917T234929_503276Z_pick_seed42).
- **Scope:** one simulated run; the size of the resulting error was not separated from the mount's (F-052),
  which biased the same run's estimates in the same direction. The real camera is untouched: this is about
  what the renderer can draw, not about the calibration.
- **Implication:** F-053's "a pick on the measured model still succeeds in simulation" holds, but its camera
  was not the measured one below the image centre: 14.3 px in cy is about 9 mm at 40 cm, the error F-053 was
  written to warn about. A simulated pick on a calibration should perceive through the intrinsics the renderer
  reports (`wrist.data.intrinsic_matrices`), not the file's, and cannot test the principal-point part of a
  calibration at all. VIP-Rescue's rescue sim publishes the rendered intrinsics in its `camera_info` for this
  reason.

### F-071 — In simulation the lying dog's arm pushes the combiner lever to 45° and holds it against a spring needing up to 1.15 N·m there (box straight ahead) or 0.9 N·m (bearing −43°); the base yaw joint saturates first

- **Status:** confirmed (simulation only)
- **Week:** 1
- **Date:** 2026-09-19
- **Evidence:** [Week 1 log, 2026-09-19](week_01/notes.md), "AprilTag-guided lever push on the combiner";
  [figure](week_01/figures/combiner_torque_sweep.png).
  - **Runs.** [Straight ahead at 0.66 m, 0.3–1.2 N·m](#/week/1/run/20260919T015634_664610Z_combiner_seed42),
    [its 1.05–1.15 refinement](#/week/1/run/20260919T020150_725196Z_combiner_seed42) and
    [the seed-42 box at bearing −43°, 0.4–1.1 N·m](#/week/1/run/20260919T020325_562869Z_combiner_seed42): one
    attempt per spring, the spring sized by the torque it needs at 45° (`--handle_torque_nm`), the latch
    releasing at 45°. The arm finds the tag, pushes the lever with its fingers along the lever's arc to a
    commanded 52°, and holds 2 s.
  - **Result.** Held at 45° or past (mean of the last 0.5 s): up to **1.15 N·m** straight ahead (45.2°; 1.2
    peaked at 45.8° and fell to 37.8°) and **0.9 N·m** at −43° (46.0°; 1.0 peaked at 45.5° and held 43.4°; 1.1
    never reached 45°). Below that the lever stays within 1–3° of the commanded 52°.
  - **Mechanism.** PhysX's projected joint torques put Joint1 (base yaw, 3.3 N·m) at 100% of its limit from 0.8
    N·m on at both placements, the joint the static model (`press.press_capacity`) names. The finger force at
    the limit, 10.6 N and 8.5–8.9 N, is near the model's 8.7 and 8.2 N. The model's torque ceiling, 0.69 and 0.65
    N·m, is low because it assumes the planned 80 mm contact: under load the arm's torque on the lever divided by
    the finger force grows to 105–108 mm, the lever's end.
- **Scope:** simulation with the D1's published torque limits and stiff implicit drives (4000 N·m/rad), dog
  lying, arm only, IK reference rather than the learned controller, perfect simulated hand-eye calibration, two
  placements, no repeats and no noise. The real servos' behaviour under load, their torque limits and joint zeros
  are unmeasured (F-007, F-023), and so is the real handle's torque. Part of the margin comes from the fingers
  sliding to the end of the lever, which on a real handle is also how a pusher slips off.
- **Implication:** the lowered 0.4 N·m default spring is 35–45% of what the simulated arm turns, and the
  original 0.94 N·m is inside it straight ahead but close to the limit off-axis. The number to compare against
  is the real handle's measured torque at 45°, which is still the open Week 1 item. If the real handle needs
  more, the base yaw is what to relieve (a push closer to vertical, a contact further out, or turning the base),
  and a tool that holds the contact at the lever's end would make the margin deliberate rather than accidental.

### F-072 — The door's AprilTag locates the combiner lever to within 2.6 mm from a close look at 0.25 m, in 19 of 19 simulated attempts

- **Status:** confirmed (simulation only)
- **Week:** 1
- **Date:** 2026-09-19
- **Evidence:** the four runs of F-071 and [the smoke run](#/week/1/run/20260919T015408_437274Z_combiner_seed42),
  19 attempts. tag36h11 id 0, 60 mm, on the door above the handle; OpenCV `DICT_APRILTAG_36h11` and `solvePnP`
  IPPE_SQUARE through the rendered intrinsics; box pose = camera pose from joint feedback and the mount x
  tag pose x the registered tag-to-box transform, averaged over three frames.
  - Every attempt found the tag (at the 0° stop straight ahead, the −40° stop off-axis): 114 detections, 0.003–0.30 px
    reprojection error.
  - The planned lever contact point, which is what the push depends on, was off by **1.5 mm median, 2.6 mm
    max** from the close look (0.25 m, 149–150 px tag) and 2.7 / 3.9 mm from the search look (91–95 px). The
    whole box pose was off by 0.6–10.2 mm and 0.13–1.66°: most of that is rotation error carried to the box's
    origin on the floor, 0.3 m from the tag.
- **Scope:** the renderer draws the tag perfectly; there is no blur, noise, glare or lighting variation. The
  camera pose came from the renderer (`--mount_calibration sim`), a perfect hand-eye calibration; with the saved
  mount aligned by eye it would carry F-052's ~11 mm offset. The first sweep's errors are measured against the
  box pose at the end of each attempt rather than at the look; the base moved at most 0.34 mm where it was
  recorded. OpenCV's detector is not the AprilTag library `apriltag_ros` uses on the robot.
- **Implication:** in simulation the tag is not the limiting error for the push: 2.6 mm is small against an
  18 mm lever and 26 mm fingers. On the robot the hand-eye calibration and the tag's registration to the handle
  are what will set this number, and both still have to be measured (plan, Weeks 1–2).

### F-073 — Gripping the combiner's lever, the lying dog's arm turns it past 45° and pulls the door open 37–59° against springs up to 0.4 N·m; from 0.5 N·m the grip, not the arm, gives way

- **Status:** confirmed (simulation only), with its scope narrowed by [F-075](#f-075) (2026-09-19): every
  attempt below used grasp pitch 40°, roll −1. Lukas's viewer run of the launcher default opened the door at
  only 5 of 9 placements at 0.3 N·m, and all 4 failures had taken a different grasp. The 0.2–0.4 N·m range
  holds for the two grasps F-075 names, not for any grasp the planner might choose.
- **Week:** 1
- **Date:** 2026-09-19
- **Evidence:** [Week 1 log, 2026-09-19](week_01/notes.md), "The arm grips the combiner's handle and pulls the
  door open"; [lever turned](week_01/figures/combiner_grip_pull_lever_turned.png),
  [door open](week_01/figures/combiner_grip_pull_door_open.png).
  - **Method.** AprilTag search and close look as F-072. The jaws close across the lever 90 mm from the
    spindle, shut to the real jaws' 2 mm past the URDF's stop. The arm then turns the lever to a commanded
    52°, cracks the door 10°, lets the lever back up while holding it, pulls the door to 60° or as far as it
    reaches with 15% torque to spare, and lets go. Grasp chosen from 10 approach/roll candidates by door
    reach among those strong enough for the handle; every attempt used pitch 40°, roll −1.
  - **Sweeps.** Straight ahead at 0.66 m
    ([0.2–0.8 N·m](#/week/1/run/20260919T031732_047629Z_combiner_seed42)): the latch released and the door
    was held open at 47.5°, 46.3° and 46.2° for 0.2, 0.3 and 0.4. At 0.5, 0.6 and 0.8 the lever stopped at
    43.3°, 42.6° and 37.4°, the latch held, and the grip was lost. At bearing −43°
    ([0.2–0.4 N·m](#/week/1/run/20260919T032242_084031Z_combiner_seed42)) all three opened it, to 52.2°,
    49.0° and 50.2°.
  - **Placements at 0.3 N·m.** 10 of 11 attempts over 6 placements opened the door. The failure was an
    approach blocked by an open finger touching something the planner does not model (Joint3 0.31 rad short).
    With a fallback to the next grasp, [4 of 4 random placements](#/week/1/run/20260919T034312_785457Z_combiner_seed7)
    succeeded, one on its second grasp.
  - **Why the grip fails first.** At 0.5 N·m the push the lever needs is ~6 N, 77% of it along the jaw axis
    for this grasp. The static arm ceiling was 0.78 N·m, and the push method turned 1.15 N·m (F-071). The
    URDF's two finger drives are independent. A sideways load pushes one fully open; successful attempts also
    ended with the pair shifted 16–37 mm, the bar hooked rather than pinched, sliding 20–75 mm along it.
    Squeezing inside the bar made it worse. A PhysX mimic coupling froze one finger and sent the arm
    diverging, so it was not used.
- **Scope:** simulation, lying dog, arm only, IK reference, perfect simulated hand-eye calibration, a light
  undamped door. Not validated: the real gripper, which is one servo and whose force and coupling are
  unmeasured, and the real handle's torque. The 0.4 N·m limit is this simulated gripper's.
- **Implication:** the grip-and-pull works on the proxy box at 0.3–0.4 N·m, so that is the demonstrator's
  default (0.3). What sets the real limit is the D1 gripper's squeeze and its coupling under a sideways load,
  which a bench measurement on the real lever would settle. If it is short, turn by pushing (F-071) and grip
  only to pull the door. A faithful simulated gripper needs the jaws coupled, which this Isaac Lab did not
  accept as a mimic joint.

- **Followed by:** F-145 (2026-10-19): on Lukas's real box (handle 0.67 m up, hinged on the right) the dog stands, and
  in three attempts at 0.3 N·m no grip turned the lever past 45°: the standing base tilts under the arm and the jaws
  close 9–20 mm off the bar. This finding holds for the proxy box and the lying dog.

### F-074 — The simulated D1 settles short of its joint targets, Joint2 by 0.0135 rad at the lever (7–9 mm at the jaws); one step of feedback correction takes it to under 0.001 rad

- **Status:** confirmed (simulation)
- **Week:** 1
- **Date:** 2026-09-19
- **Evidence:** [Week 1 log, 2026-09-19](week_01/notes.md), same entry. Trace columns `q_cmd`, `q_drive` (the drive
  target Isaac Lab sends) and `q_true`:
  - With the drive target equal to the command, the joints held Joint2 0.0135 rad and the wrist joints
    0.002–0.005 rad short at the grasp pose, carrying 2.5 N·m on Joint2. At the search poses, with little
    torque, the shortfall was still 0.002–0.009 rad.
  - The jaw centre was 7–9 mm from the lever point; perception accounted for 2 mm.
  - After one correction (targets plus the shortfall read from 9 Hz joint feedback), the residual was
    0.0002–0.0008 rad in all 17 grasps since.
- **Scope:** simulation with force drives at 4000 N·m/rad; the cause is not isolated (not the firmware
  planner, which lands on its goal, nor the soft limits). The real arm's steady error is only known for one
  unloaded joint (≤0.1°, F-021).
- **Implication:** open-loop IK poses in this simulator carry ~1 cm at the tool, which F-015 also saw. Any
  contact task planned from IK here (grasps, presses) should close the loop on joint feedback before
  contact, and the same correction is cheap on the real arm.

### F-075 — Only two grasps hold the combiner's lever in simulation, both wrist roll −1 (pitch 40° and 50°); ranked first, they opened the door at 17 of 17 placements at 0.3 N·m, where the old choice managed 5 of 9

- **Status:** confirmed (simulation only)
- **Week:** 1
- **Date:** 2026-09-19
- **Evidence:** [Week 1 log, 2026-09-19](week_01/notes.md), "The grip-and-pull made faster, and the grasps that
  hold the lever".
  - **The failure.** [Lukas's viewer run](#/week/1/run/20260919T043704_225852Z_combiner_seed42) of the launcher
    default (0.3 N·m, seed 42) finished 9 attempts and opened the door at 5. The planner ranked grasps by how far
    the door would open. All 5 successes took pitch 40°, roll −1. The 4 failures took pitch 20°, roll +1 three
    times and pitch 60°, roll −1 once. In each, the lever stalled at 43.3–44.7° and the jaws lost the bar
    (slip 160–225 mm).
  - **The record over every grip-and-pull attempt at 0.2–0.4 N·m.** Every attempt that gripped is counted:
    the runs before today's, the four below and Lukas's run. Excluded are the two attempts that squeezed
    2 mm inside the bar, a grip command since reverted.

    | Grasp | Door opened | Lever peak |
    | --- | --- | --- |
    | pitch 40°, roll −1 | 39 of 39 | 47.6–52.2° |
    | pitch 50°, roll −1 | 11 of 11 | 45.5–47.9° |
    | pitch 20°, roll +1 | 0 of 5 | 43.0–43.6°, then lost |
    | pitch 60°, roll −1 | 0 of 1 | 44.7°, then lost |

  - **Why ranking alone was not enough.** The pitch-40 grasp does not solve everywhere: on the CPU model it
    plans at 57 of 100 random default placements (seed 11, handle 0.3 N·m). Pitch 50°, roll −1 plans at 37 of
    the other 43, and it was not in the candidate list. At the remaining 6 neither plans, and the planner falls
    back to a grasp with no record: pitch 60°, roll −1 at 3 and pitch 0°, roll +1 at 3.
    [Forcing it](#/week/1/run/20260919T055829_532757Z_combiner_seed42) (`--grasp 50 -1`) at the 9 seed-42
    placements opened the door at all 6 where it solves; the other 3 are placements where pitch 40 solves.
  - **Validation.** With pitch 50 added and the two grasps ranked first (40, then 50):
    [9 of 9](#/week/1/run/20260919T060339_055356Z_combiner_seed42) at Lukas's placements, and
    [8 of 8](#/week/1/run/20260919T061317_145569Z_combiner_seed2026) at seed 2026, placements not used to choose
    the grasps. 12 attempts took pitch 40 and 5 took pitch 50.
- **Scope:** simulation only, with the URDF's independent finger drives (F-073), 0.3 N·m for the validation, and
  default placements (0.62–0.70 m, ±45°), about 6% of which reach neither grasp. This is a record, not a model: the static ceiling rates every
  candidate strong enough, and why roll −1 at 40–50° holds while pitch 20, roll +1 does not is unexplained.
  Pitch 50 turns the lever to only 45.5–47.9° against a 45° release, so it has the less margin of the two.
  Other grasps were not all tried: pitch 0, 80 and 40 at roll +1 have no record. Nothing here says which grasp
  the real jaws hold.
- **Implication:** the planner's static torque model cannot pick a grasp for this gripper, so grasp choice now
  follows the record (`pull.PROVEN_GRASPS`), and the record needs extending as placements widen. On the real arm
  the list starts empty. The first bench grips should try 40° and 50° at roll −1 before anything else, and
  record each grasp's outcome the same way.

- **Followed by:** F-145 (2026-10-19): standing at the real box only pitch 40 roll −1 of these solves often (6 of 30
  default placements on the CPU model, pitch 50 at none); taken in all three real-box attempts, it lost the bar or
  stalled short of 45° each time, for a reason this finding's record could not see (the base moving under it).

### F-076 — The grip-and-pull now finishes in 18–25 s in simulation, down from 29–49 s; what remains is mostly the arm's own speed

- **Status:** confirmed (simulation only)
- **Week:** 1
- **Date:** 2026-09-19
- **Evidence:** [Week 1 log, 2026-09-19](week_01/notes.md), same entry. From the start of an attempt to letting
  go of the handle, at 0.3 N·m:
  - Before: 28.9–49.2 s at Lukas's 9 placements (median 37.8 s over the 4 successes without a second grasp).
  - After: 18.1–24.8 s at the same 9 and 17.7–25.0 s at 8 fresh ones, all 17 opened.
  - Median phase times after, at the two seeds: search 5.5 / 2.7 s, close look 2.3 / 2.5 s, onto the lever
    3.0 / 3.4 s, align and grip 1.7 s, turn 3.3 / 3.5 s, crack and let the lever up 1.8 / 2.5 s, pull and hold
    2.8 / 3.5 s, let go 0.6 s.
  - What changed:
    - **The arcs.** They are streamed at the pace their joints allow: up to 0.6 rad/s, 45°/s of lever and
      30°/s of door. At a fixed 15°/s the wrist trailed its command by 0.4 rad.
    - **The waits.** Holds went from 1.0 and 1.5 s to 0.3 and 0.5 s; the alignment wait from 0.8 to 0.4 s.
      The jaws open 41 mm instead of 77 mm, so gripping takes 0.8 s instead of 1.5 and letting go 0.6 instead
      of 1.5.
    - **The search.** Its stops sweep one side, then the other, instead of alternating sides. It leaves an
      empty stop after 3 frames instead of 8.
    - **The close look.** It turns the camera about its view toward the coming grasp.
  - Measured separately at the 7 seed-42 placements that both runs opened:
    - **Arcs, waits and jaws:** [run](#/week/1/run/20260919T054947_045656Z_combiner_seed42), median 23.9 s.
    - **Search order and close-look roll, added on top:** median 20.4 s. Most of the drop is the search
      order: the search median fell from 10.1 to 5.5 s. The camera roll moved the onto-the-lever median only
      from 3.2 to 3.0 s.
- **Scope:** simulated arm and firmware model. The grip held at these speeds in every attempt that used a
  grasp from F-075, but faster arcs were not tried at 0.4 N·m. Not validated:
  - **The firmware assumption.** The simulated firmware (F-045) restarts its plan at every 10 Hz setpoint,
    even an unchanged one. That holds long moves to about 0.7 rad/s, against the 1.2–1.3 rad/s ceiling
    measured for a single command (F-033). Whether the real firmware restarts on a repeated identical setpoint
    has not been measured, and the pick's hardware loop does repeat them.
  - **The real jaws' closing speed.** The simulated fingers close at about 23 mm/s.
  - **The real camera's latency.**
- **Implication:** most of the remaining time is set by joint speed:
  - point-to-point moves at the firmware model's ~0.7 rad/s;
  - the turn, where the pitch-40 grasp passes near the wrist's Joint5 = 0 singularity and Joint4 swings
    about 2 rad for 52° of lever;
  - the search, for a box off to the side.

  One bench measurement matters most: time a single long waypoint against the same waypoint re-sent at
  10 Hz. If the real firmware does not restart on a repeated target, sending each target once could cut
  every move by up to 40%.

### F-077 — Crouching onto the combiner lever with the arm held still would turn about 1 N·m (up to 1.5) by the static model: the arm's shoulder, not body weight, sets the limit

- **Status:** provisional (static model only; nothing simulated)
- **Week:** 2
- **Date:** 2026-09-21
- **Evidence:** [Week 2 log, 2026-09-21](week_02/notes.md);
  [`combiner_crouch_push_study.py`](week_02/figures/combiner_crouch_push_study.py).
  - **Method.** The dog stands level (base 0.274 m). The gripper comes straight down over the lever, 6.5 cm out
    from the spindle. The arm holds still and the body drops 8.3 cm for 52°. The ceiling is
    `press.press_capacity`, gravity plus JᵀF against the published joint limits; its force matched the
    simulated push within 20% in F-071. It is taken over 439 placements where the box clears the dog.
  - **Result.** Median 0.94 N·m, 90th percentile 1.17, best 1.51 (a push of 14.5 N median, 23 N best). Joint2
    limits 70% of placements and Joint3 the rest. The best stances put the box beside the dog, with the lever
    about 0.20 m to the side of the mount. From the lying pose the top-down push does not solve.
- **Scope:** static, base level, rigid stance, published limits. Not validated: a standing or crouching leg
  controller in the scene, the palm-on-bar contact of the D1 gripper, the real servos under load, and the
  exact clearances around the dog's legs. Sitting was not studied.
- **Implication:** the dog's weight is ample: 15 N per N·m against about 180 N. The force still passes through
  the arm, and the shoulder carries it about 0.2 m out, so crouching gains about 2.5× over the grip-and-pull
  (F-073) and at most ~1.3× over the lying push (F-071). A much larger torque needs the force to go around the
  arm's motors: a longer moment arm (the plan's pre-attached lever tool) or the legs pushing directly.

### F-078 — UniFP's released B2Z1 training starts on Isaac Gym Preview 4, in under a day of setup, with two dependency pins its README does not give

- **Status:** confirmed (launched and optimising; not a reproduction of its results)
- **Week:** 1
- **Date:** 2026-09-18
- **Evidence:** [upstream B2Z1 smoke](#/week/1/run/Sep18_10-45-06_), 64 environments, 3 iterations, on a
  **spare machine** (RTX 3500 Ada Laptop 12 GB, Ubuntu 22.04), not the RTX 4080 PC the rest of this week's
  runs used. Isaac Gym Preview 4 from `~/Downloads/IsaacGym_Preview_4_Package.tar` into a fresh
  `unifp` conda environment: Python 3.8, torch 2.3.1+cu121, numpy 1.23.5. GPU PhysX initialises and the
  bundled `gymtorch` extension builds against torch 2.3.1 without patching. Training reports
  2,973 steps/s and all 40 reward/loss scalars. Setup took about 40 minutes of the two-working-day budget
  the plan sets, with three avoidable stops:
  - `params_proto` must be pinned **below 3.0** (`2.12.1` used). 3.x uses `tuple[...]` subscripting and
    fails to import on Python 3.8, which is the only Python Isaac Gym Preview 4 supports.
  - `wandb.init` is called unconditionally by `on_policy_runner.learn`, so a machine without a wandb login
    cannot train at all until `WANDB_MODE=disabled` is set. TensorBoard logging is unaffected.
  - `cfg.asset.file` is a path relative to the **working directory**, so the README's
    `cd legged_gym/scripts && python train_b2z1posforce.py` cannot find the robot. The URDF load failure
    is reported as a parse error and then surfaces as `KeyError: 'ee_gripper_link'`, which names neither
    cause. Run from the repository root.
- **Scope:** 3 iterations at 64 environments on the released B2Z1 task. This establishes that the stack
  installs and the released training loop optimises **here**. It is not a reproduction of UniFP's reported
  policy or numbers, and no upstream checkpoint was obtained or replayed.
- **Implication:** the legacy Isaac Gym path is viable on a spare machine without touching the shared
  Isaac Lab installation, so the Week 2 port-versus-retarget decision can be made from launch evidence
  rather than from reading. Records the three pins any repeat of this needs.

### F-079 — The Go2+D1 retarget of UniFP trains, once the mass model is rebuilt into the URDF: the RViz drawing has no inertials, and Isaac Gym silently gives the robot almost no mass

- **Status:** confirmed (measured in the simulator, both the failure and the fix)
- **Week:** 1
- **Date:** 2026-09-18
- **Evidence:** first launch of the port [NaNed at reset](#/week/1/run/Sep18_10-54-25_) — 11 of 64
  environments had NaN in `dof_pos` and 18 in `root_states` before a single policy step, spreading to
  31 and 34 after one. It is not self-collision (disabling it changes nothing) and not the observation
  maths (every observation term is finite when the state is). `description/go2_d1.urdf` drops every
  `<inertial>` block on purpose — it is a drawing for RViz, and `weld.py` owns the simulated mass model
  (description/README.md). Isaac Lab never reads it. Isaac Gym does, and with no inertials it derives
  mass from collision geometry times `asset_options.density`, which legged_gym leaves at **0.005**.
  `unifp_go2d1/build_asset.py` reassembles the same model `weld.py` builds — Go2 inertials from the Go2
  description (15.019 kg), D1 shells from `d1_arm/d1.urdf` (0.719 kg), servo masses (0.345 kg) and the
  rest of ARM_MASS_KG on the arm base — and the same command then [trains](#/week/1/run/Sep18_11-02-34_).
  The loaded articulation weighs **18.171 kg**, with 2.165 kg at the arm base and 0.987 kg of moving arm:
  the arm base figures match `weld.py`'s own print, and the total matches `articulation_mass_kg` recorded
  for the Isaac Lab smoke runs ([example](#/week/1/run/20260915T093352_794571Z_smoke_seed42)) exactly.
  Two further changes were forced by Isaac Gym rather than chosen, both documented in
  [unifp_go2d1/README.md](../unifp_go2d1/README.md): arm joints renamed `Joint<n>` → `d1_Joint<n>` (Isaac
  Gym orders DOFs alphabetically by the joint starting each base subtree, which otherwise interleaves the
  arm between the front and rear legs and breaks every fixed-index slice in the environment), and an
  `ee_gripper_link` body added at the CAD pincer tip so the environment can index and push the controlled
  point.
- **Scope:** the environment constructs, steps and optimises, and the model's mass matches the Isaac Lab
  model's. That is an interface and mass-model result. **No policy has been trained or evaluated**, the
  gains and goal ranges below are engineering choices rather than measurements, and nothing here has been
  compared with the real robot.
- **Implication:** the port is a runnable retarget rather than a rewrite — the method (PPO variant, history
  encoder, estimator supervision, force curriculum, reward terms) is upstream's and unchanged. It also
  says that `description/go2_d1.urdf` must not be handed to any importer that reads inertials; its
  docstring now says so, and the port regenerates its own asset from it.

### F-080 — UniFP's force commands are eight times what a D1 can produce: the port runs at ±8 N, not ±60 N

- **Status:** provisional (an arithmetic bound from published torque limits; no force has been measured
  on this arm, in simulation or on hardware). **Superseded in part by [F-103](#f-103) (2026-09-24):** the
  bound stands for the arm alone in a bent reach and for UniFP's free-space task, but its implication
  ("force tracking on this robot is a few-newton problem") does not hold for a force into a fixture the
  body can lean on — a policy trained for that pulls 60 N in simulation.
- **Week:** 1
- **Date:** 2026-09-18
- **Evidence:** UniFP's released B2Z1 config commands end-effector forces uniformly in
  `[-60, 60] N` per axis (`max_push_force_xyz_gripper_cmd`) against a Z1, and base forces in `[-50, 50] N`
  against a ~60 kg B2. The D1's published effort limits are 3.3 N·m on J1/J2 and 1.7 N·m on J3–J6
  (`motor_model.py`, and confirmed exact in PhysX in Week 1). At the ~0.45 m moment arm the port's goal
  sphere uses, the strongest joint supports about **7 N** at the tool tip. The port therefore commands
  `[-8, 8] N` at the end effector and `[-20, 20] N` at the base (the same fraction of body weight as
  upstream's ±50 N on a B2).
- **Scope:** a static torque-over-lever bound using published limits and one nominal moment arm. It
  ignores posture, the arm's own weight, and whatever the legs contribute by leaning. It has not been
  checked against a measured stall force, and the jaw/tool force path is not covered at all.
- **Implication:** force tracking on this robot is a **few-newton** problem. Any comparison against
  UniFP's reported force-tracking numbers has to say so, and the H1 force curriculum, the force-sign
  calibration fixture (Week 3) and the instrumentation chosen to measure ground truth all need to resolve
  single newtons rather than tens. Measuring the arm's actual achievable tip force, in simulation and then
  on the bench, would move this to confirmed and should happen before the force comparisons are framed.

### F-081 — A full-schedule UniFP run costs about 41 hours of this GPU, and its force curriculum does not begin until hour 5.5

- **Status:** confirmed (measured throughput; the schedule length is upstream's default)
- **Week:** 1
- **Date:** 2026-09-18
- **Evidence:** the Go2+D1 task at 4096 environments on the RTX 3500 Ada,
  [FP32](#/week/1/run/Sep18_11-03-27_): 30.0k steps/s, 3.28 s/iteration (collection 1.80 s, PPO update
  1.47 s). [With TF32 matmuls](#/week/1/run/Sep18_11-06-23_): 40.2k steps/s, 2.44 s/iteration (collection
  1.74 s, update 0.70 s) — the update is 45% of the iteration because a history encoder consumes a
  32-frame stack of 76 observations per sample, compressing it to a 64-wide latent that is concatenated
  with the current observation, so the actor MLP itself is only 140 wide. 7.2 GB of 12.3 GB of VRAM, 88% GPU utilisation. `LeggedRobotCfgPPO.runner.max_iterations`
  is **60,000** and the B2Z1 config does not override it, so the released schedule is 5.9 × 10⁹ policy
  steps: **41 hours** with TF32, 55 without. (Measured on completion: 39.85 h, so this estimate was
  out by −2.8% — F-083.) `commands.force_start_step = 8000` gates every external force,
  and `global_steps` counts policy steps at 24 per iteration, so forces begin at **iteration 8,000** —
  5.5 hours in. Everything before that is locomotion and position tracking.
- **Scope:** throughput on one machine, one task, one environment count. TF32 is a reduced-precision
  change: a TF32 run is statistically equivalent to an FP32 one but not bit-identical, so runs must record
  which was used. Nothing here says the schedule is long enough, or that 60,000 iterations is what UniFP
  actually trained for — the number is a base-class default the released config leaves alone.
- **Implication:** one seed of the full schedule is a two-day GPU commitment and three seeds are a week, so
  the seed budget for any UniFP-derived comparison has to be planned rather than assumed. Checkpoints land
  every 200 iterations (~8 minutes), so a run can be cut short at any point and still yield a usable policy
  — which is the practical way to buy a shorter experiment without editing the curriculum.

### F-082 — Resuming a UniFP run silently restarts its force curriculum, and `--max_iterations` on a resume means "train this many more", not "train up to this"

- **Status:** confirmed (both behaviours read from the source and the first measured directly)
- **Week:** 1
- **Date:** 2026-09-18
- **Evidence:** found while making the 42-hour run (F-081) survive a crash without a human present.
  Two separate defects, neither of which raises an error:
  - **The force curriculum restarts.** External forces are gated on `env.global_steps >
    commands.force_start_step * num_steps_per_env`, and `global_steps` is set to 0 by
    `_init_buffers` on every launch while the policy and optimiser are restored from the
    checkpoint. A crash at iteration 20,000 — 12,000 iterations into force training — would
    resume into **8,000 more iterations of position-only training** with nothing in the log
    saying so. Measured directly on the Go2+D1 task with the gate moved to iteration 1: at
    `global_steps = 0` the peak applied force over 12 steps is **0.000 N**; at `global_steps =
    5000` it is **0.394 N**. `launch_training.py` now sets `global_steps = resumed_iteration ×
    num_steps_per_env` after the checkpoint loads, and prints which side of the curriculum the
    run is on.
  - **Iterations are additive.** `OnPolicyRunner.learn` computes `tot_iter =
    current_learning_iteration + num_learning_iterations`, so `--max_iterations 60000` on a
    resume at iteration 20,000 trains to 80,000. The supervisor asks for exactly the iterations
    still owed, and a resume from 3 asking for 6 was verified to stop at 9.
  - A third, smaller one: `get_load_path(load_run=-1)` takes the alphabetically last run
    directory, which is an empty one if a restart died before saving (and is wrong across a month
    boundary, as its own TODO says). `find_checkpoint.py` scans every run directory and orders by
    the iteration in the filename instead.
- **Scope:** read from UniFP's source at `68847a070f88` and exercised on the Go2+D1 port. The force
  measurement is one task, 64 environments, 12 steps either side of the gate — it establishes that
  the gate follows `global_steps`, not what a resumed policy would have learned. No long run has
  yet been crashed and resumed in anger.
- **Implication:** any resumed UniFP-derived run before this fix has to be treated as having an
  unknown curriculum, and any comparison across runs has to state whether it was resumed. It also
  generalises: this environment keeps curriculum state on the env rather than in the checkpoint,
  so **anything else added to the curriculum needs the same treatment**, and the checkpoint is not
  a complete description of training state.

### F-083 — The Go2+D1 UniFP run finished all 60,000 iterations in 39.85 h, and the curve says most of it was wasted: learning is over by ~10,000, and a late destabilisation means the last checkpoint is not the best one

- **Status:** confirmed for what it measures (training-time scalars of one completed run); **no policy
  evaluation has been run**, so nothing here is a statement about how well the policy reaches or
  tracks force
- **Week:** 1
- **Date:** 2026-09-20
- **Evidence:** [the run](#/week/1/run/Sep18_11-10-15_), 4096 environments, seed 1, TF32, forces from
  iteration 8,000. Curves: [unifp_go2d1_training.png](week_01/figures/unifp_go2d1_training.png).
  - **It completed cleanly.** 60,000 iterations, 5,898,240,000 policy steps, 143,461.8 s = **39.85
    hours**, 301 checkpoints, **zero errors or NaN in 40 hours of log**, and **zero supervisor
    restarts** — the process that started it was the one that finished it. Predicted 41 h from a
    25-iteration benchmark (F-081); out by −2.8%. `model_60000.pt` loads, stores `iter = 60000`,
    has 2,074,161 finite parameters and an 18-wide action head.
  - **Learning is over early.** Mean return is 154.6 by iteration 6k–8k and 154.4 at 58k–60k.
    Measuring only after the curriculum starts, so the comparison is like-for-like: 150.7 at
    8k–10k against 154.4 at the end — **+3.7 return for 52,000 iterations, 35 hours and 87% of the
    compute.**
  - **The force curriculum costs tracking and never fully gives it back.** Implied tip L1 error
    (from the tracking term; see Scope): ≥8.2 cm just before forces, ≥11.3 cm immediately after,
    best ≥8.8 cm around iteration 48,800, ≥9.3 cm at the end.
  - **A destabilisation at 55,000–57,500, twice, unexplained.** Return 157.1 → 139.0 → 127.3;
    mean episode length 1001.9 → **929.0**, so episodes were terminating early (falls), having been
    pinned at ~1000 for the previous 45,000 iterations; collision penalty −0.0142 → **−0.1432**, ten
    times worse; value-function loss 0.027 → **0.190**; action noise std 0.664 → 0.730 as the
    adaptive-KL schedule widened exploration in response. It partially recovers by 60,000 (return
    155.4, episode length 1001.6) but does not return to the previous level. The learning rate did
    **not** spike (1.0–1.5 × 10⁻⁵ throughout) and no curriculum stage changes after iteration 8,000,
    so the trigger is not identified.
  - **So the final checkpoint is not the best one.** By mean training return over the 200 iterations
    preceding each checkpoint, `model_48800.pt` scores 157.5 against `model_60000.pt`'s 155.3
    (−1.4%), with implied tip error 8.8 cm against 9.3 cm.
- **Scope:** one seed, training-time scalars sampled at episode reset — the same class of number
  [results/README.md](README.md#conventions) warns is not an evaluation. The "tip error" is a
  Jensen lower bound inferred from `rew_tracking_ee_force_world` (weight 2.0, sigma 1.0), it is an
  **L1** error summed over three axes, and its target is the **force-displaced** goal, so it mixes
  position and force tracking and is not a reach error. It must not be compared with the 6.7–11.5 mm
  of the Isaac Lab position-only policies (F-047): different task, different target, plus velocity
  commands on rough terrain. Picking `model_48800` on training return is itself selection on the
  training signal, which is exactly what F-019 refused to do for G1a.
- **Implication:** three things, in order of how much they change the plan.
  1. **The schedule is not justified by the curve.** Whatever budget the next UniFP-derived runs
     get, 60,000 iterations is not it — the same return was reached by ~10,000. At 2.4 s/iteration
     that turns a three-seed comparison from five days of GPU into about one. Any decision to keep
     the full schedule now needs a reason beyond "it is upstream's default", and the default is a
     base-class value the released B2Z1 config never overrides (F-081).
  2. **"Train to the end, take the last checkpoint" is not safe here.** A run can destabilise after
     45,000 stable iterations. Reporting any UniFP-derived policy must state which checkpoint and
     how it was chosen.
  3. **The evaluator is now the blocking piece.** There is no frozen-manifest equivalent for this
     task, so neither the plateau, the checkpoint choice, nor the effect of the force curriculum can
     be settled — only observed in training reward. That, not more training, is what the next work
     on this should be.

### F-084 — The Go2 and the D1 disagree about which way is up, and one Isaac Gym flag cannot satisfy both: every rendering of this robot before 2026-09-20 was wrong, and none of the physics was

- **Status:** confirmed (both halves seen in the viewer, and the physics equivalence measured)
- **Week:** 1
- **Date:** 2026-09-20
- **Evidence:** found by Lukas looking at the first policy rollout and saying the meshes were
  sideways. `asset_options.flip_visual_attachments` is a single flag for a whole asset, but this
  robot is merged from two sources with opposite mesh conventions:
  - the Go2's `.dae` visual meshes are y-up and **need** the flip — without it the dog renders on
    its side with its legs splayed flat;
  - the D1's SolidWorks `.STL` meshes are already z-up and **must not** be flipped — with the flag
    on, the dog is right and the arm lies on its side.
  The port shipped with the flag off (the dog wrong), then on (the arm wrong). The fix is to give
  each arm mesh a `_visflip` copy pre-rotated by the inverse of the flip, Rx(−90°): (x,y,z) →
  (x,z,−y), and point only `<visual>` at it. `<collision>` keeps the original file.
  **The physics is untouched, and that was measured rather than assumed:** loading the asset with
  the flag on and off gives identical body masses and every body within **0.0000 mm** after a 2 s
  settle. The Go2's collisions are all box/cylinder/sphere primitives, and although the D1's
  collisions *are* meshes, the flag does not touch them.
- **Scope:** an appearance fault in this port's asset, fixed in `unifp_go2d1/build_asset.py`. The
  equivalence test is one 2 s settle under gravity with no actions, which is enough to show the
  flag does not enter the physics pipeline, not a general claim about every Isaac Gym asset option.
- **Implication:** **the 39.85-hour training run (F-083) is unaffected** — the policy never saw a
  visual mesh. What is affected is evidence made of pictures: any screenshot or video of this task
  taken before 2026-09-20 shows a robot in the wrong shape and should not be used or shown.
  More generally, a merged robot inherits a mesh convention per source, and a per-asset flag is the
  wrong shape of control for it — worth checking the first time this model is rendered in any new
  simulator, because nothing errors and the numbers all stay right.

### F-085 — First numbers off a trained Go2+D1 UniFP policy: 4.6 cm median tool-tip error undisturbed, 6.3 cm while being pushed, in a single clean rollout

- **Status:** provisional (one rollout, one seed, one checkpoint, no held-out set, no baseline)
- **Week:** 1
- **Date:** 2026-09-20
- **Evidence:** [playback](#/week/1/run/20260920T0945_play48800_forces) of `model_48800.pt` from
  [the completed run](#/week/1/run/Sep18_11-10-15_), 1500 steps, one environment, randomisation and
  observation noise off, external forces **on**. Discarding the first 100 steps of spawn transient,
  tip-to-goal **L1** error over 1400 steps: median 5.9 cm overall, **4.6 cm in the 528 steps with no
  force applied** and **6.3 cm in the 872 steps with a force commanded or measured**; p90 11.3 cm,
  worst 28.0 cm. Base height median 29.4 cm against the 30 cm target (min 24.2, max 38.5). Force
  commanded up to 9.6 N and measured at the gripper up to 8.1 N (the per-axis range is ±8 N, so a
  three-axis command can exceed that in magnitude).
- **Scope:** **one rollout of one checkpoint of one seed**, with randomisation and noise switched
  off — the easiest conditions the task offers, and not the distribution the policy was trained on.
  No zero-action baseline, no held-out episode set, no repetition. L1 over three axes, to the
  **force-displaced** goal, so it is not a reach error. `model_48800` was picked on training return
  (F-083), which is selection on the training signal. Nothing here is a gate result.
- **Implication:** it is the first evidence that the policy does something — the training reward
  alone could not say whether 9 cm meant "tracking loosely" or "not tracking". It also shows the
  training-reward figure in F-083 (≥9.2 cm L1 lower bound) is a **population average under
  randomisation, noise and pushes**, roughly twice the error of a clean rollout, so the two numbers
  must not be quoted against each other. It does not move G1a or any other gate, and it will not
  until the task has a frozen evaluator with a zero-action reference — still the blocking piece.

### F-086 — The trained Go2+D1 UniFP policy evaluated on frozen episode sets: 2.6 cm median tool-tip error against 26 cm for zero actions, no falls against 19 in 50, and the checkpoint training return preferred is the better one on held-out episodes too

- **Status:** confirmed on two frozen 50-episode sets, one of them held out (single seed, single training run)
- **Week:** 1
- **Date:** 2026-09-20
- **Evidence:** a frozen-manifest evaluator built for this task (`unifp_go2d1/eval_manifest.py`,
  `evaluate.py`, `run_eval.py`) with manifests
  [`unifp_development.json`](manifests/unifp_development.json) (`8bb8d258…`) and
  [`unifp_validation.json`](manifests/unifp_validation.json) (`895ecd9d…`), 50 episodes of 20 s
  each, randomisation and observation noise off, external forces **on**. Median over episodes of
  the per-episode median:

  | | zero actions | model_48800 | model_60000 |
  | --- | --- | --- | --- |
  | falls, development | 15 of 50 | **0** | **0** |
  | falls, validation | 19 of 50 | **0** | **0** |
  | tool-tip error, force-free, development | 26.9 cm | **2.53 cm** | 2.99 cm |
  | tool-tip error, force-free, validation | 25.6 cm | **2.60 cm** | 2.99 cm |
  | unified tracking, development | 32.2 cm | 3.06 cm | 3.35 cm |
  | unified tracking, validation | 33.0 cm | 3.05 cm | 3.42 cm |
  | force-estimator error, validation | — | 1.22 N | 1.39 N |
  | base velocity error, development | 0.393 m/s | 0.050 m/s | 0.055 m/s |

  Runs: development [zero](#/week/1/run/20260920T100239_development_zero),
  [48800](#/week/1/run/20260920T100350_development_48800),
  [60000](#/week/1/run/20260920T100451_development_60000); validation
  [zero](#/week/1/run/20260920T101004_validation_zero),
  [48800](#/week/1/run/20260920T101104_validation_48800),
  [60000](#/week/1/run/20260920T101205_validation_60000).
  - **The policy is doing the task, by a wide margin over doing nothing.** Zero actions fall in
    30–38% of episodes and sit ~26 cm from the goal in the rest; the policy never falls in 100
    episodes across both sets and tracks to 2.6 cm.
  - **F-083's checkpoint call holds up on held-out episodes.** `model_48800` was picked purely on
    training return, and it beats `model_60000` by 0.39–0.46 cm on both sets. Re-running the same
    evaluation three times gives 2.53 / 2.58 / 2.65 cm, so the run-to-run spread is about
    ±0.1 cm and the gap is three to four times it. That is a real difference, and a small one.
  - **The frozen schedule verified itself.** Every run reported `schedule_mismatches: []` —
    the goal, velocity and force schedule each episode received was identical to the one recorded
    in the manifest, for the baseline and for both checkpoints. The episodes really are the same
    episodes.
- **Scope:** **one training seed and one training run**, so this measures this policy, not the
  method. Randomisation and noise are off — the easiest conditions the task offers, not the
  training distribution. "Tool-tip error" is Euclidean to the commanded goal on steps with no force
  commanded or applied; "unified tracking" is to the force-displaced target the reward actually
  uses. The force numbers are UniFP's simulated admittance, **not measured contact force** — there
  is no force sensor in this loop, and `force realised` is meaningless for the fallen baseline
  (a robot on its back is far from the goal, and the projection times a 200 N/m stiffness produces
  tens of newtons that mean nothing). GPU physics is not bitwise deterministic, so identical
  re-runs differ: the fall count for zero actions was 16 when the manifest was built and 15 when it
  was evaluated. No gate is defined for this task, so this passes nothing.
- **Implication:** the plateau, checkpoint choice and force behaviour in F-083 and F-085 are now
  measured rather than inferred, and the two can be compared: the clean-rollout figure in F-085
  (4.6 cm median L1) and this (2.6 cm median Euclidean) are the same policy under the same
  conditions in different metrics, while the ≥9.2 cm from training reward is a population average
  under randomisation and noise — roughly four times looser than the quiet-conditions number, which
  is the size of the gap between "training reward" and "evaluation" for this task. It also gives
  the shorter-schedule question from F-083 a way to be settled: a 15,000-iteration run can now be
  compared with this one on the same frozen sets rather than argued about. The obvious next
  measurements are the same policy **with** randomisation and noise on, and a second training seed.

### F-087 — UniFP's trained policy now runs on the Isaac Lab model, and its observation and action interface is reproduced exactly; but Isaac Lab cannot integrate UniFP's arm gains without rotor inertia the training stack does not have

- **Status:** confirmed for the interface (exact, automated); the armature deviation is confirmed and measured
- **Week:** 1
- **Date:** 2026-09-20
- **Evidence:** `unifp_isaaclab/` and `run_unifp_isaaclab.py` run a UniFP checkpoint on this
  repository's welded Go2+D1 with no retraining. The checkpoint is rebuilt as plain PyTorch, so
  neither Isaac Gym nor `b2_gym_learn` is needed. Verification does not rely on the simulator:
  `unifp_go2d1/dump_interface.py` records 128 policy steps out of the *running* Isaac Gym
  environment — raw state, the 76-wide observation UniFP built from it, and the 18 actions — and
  `tests/test_unifp_interface.py` (21 tests) checks the port against them with no simulator on
  either side.

  | check | result |
  | --- | --- |
  | observation rebuilt from the same raw state | max error **6e-8** (every block exactly 0 except the gait phase) |
  | actions from the same observations | max error **9.5e-6** on actions averaging 1.86 |
  | 32-frame history against the environment's own stacking | **exactly 0**, from step 1 on |
  | DOF order, gains, torque limits, command scales, timing | identical to what the environment had |
  | articulation mass | **18.171 kg** (Isaac Lab) against **18.172 kg** (Isaac Gym) |
  | joint position and effort limits, all 20 joints | identical |

  - **The DOF permutation is load-bearing, not a formality.** Isaac Gym orders the joints leg by
    leg; Isaac Lab orders them by tree level and puts the arm's `Joint1` at index **8**, inside the
    leg block. The measured gather index is `[0,4,9,1,5,10,2,6,11,3,7,12,8,13,14,15,16,17,18,19]`.
    Getting it wrong produces a policy that limps rather than one that errors.
  - **Rotor inertia had to be added, and this is a real departure from the training stack.** The
    D1's wrist links have inertias near 1e-5 kg·m². UniFP's arm stiffness of 40 N·m/rad on that
    inertia is a natural frequency near 1600 rad/s, which an explicit PD evaluated every 5 ms
    cannot integrate. Isaac Gym's articulation solver absorbs it; Isaac Lab's does not. Measured
    with `armature = 0`, matching Isaac Gym: `Joint4` left its ±2.35 rad limit entirely and reached
    **−127.8 rad** with its torque saturated, and the robot was thrown over
    ([run](#/week/1/run/20260920T011858_stand_zero)). With `armature = 2e-4 kg·m²` the joint stays
    inside its limit and zero actions settle at **27.6 cm**
    ([run](#/week/1/run/20260920T012721_stand_zero)), close to the 27.37 cm this repository
    measured for its own Isaac Lab task (`position_only/task_space.py`) — though that task spawns
    at 0.30 m rather than 0.35 m and drives the arm differently, so the agreement is corroborating,
    not like-for-like.
  - **Two further port bugs are recorded because each looked exactly like a failure to transfer:**
    spawning an articulation does not place its joints (the robot landed on straight legs until the
    default joint state was written to the simulation), and `self_collisions = 0` in legged_gym
    means self-collision *enabled* — its own comment says "1 to disable, 0 to enable".
- **Scope:** this is an interface and model result, not a behavioural one — what F-088 measures is
  separate. The equality claims are between this port and the Isaac Gym environment as it runs
  **in playback** (no observation noise, no domain randomisation, motor strength 1.0); nothing here
  says the two simulators agree about physics. The mass agreement is total articulation mass, not
  per-link inertia. The armature value was chosen to make the integration stable and is not
  measured from the D1 — it is an order of magnitude larger than the wrist links' own inertia, and
  any comparison of arm behaviour across the two stacks has to carry it.
- **Implication:** a checkpoint from the legacy stack can now be run, measured and compared on the
  Isaac Lab model without retraining, and the interface half of any disagreement is ruled out by
  test rather than by argument. It also puts a number on something the plan assumed was free:
  moving a controller between these two simulators is not a matter of loading the weights, and
  three of the four problems found today were silent — the robot simply behaved badly.

### F-088 — The trained UniFP policy holds a stance on the Isaac Lab model but cannot walk there, and whether it stands at all turns on a PhysX solver setting that leaves the passive robot untouched

> **The locomotion claim below is superseded (2026-09-21). It is an artefact of the playback
> harness, not a property of the policy.** Run in `unifp_train`'s environment under the *identical*
> condition this finding used — one environment, a sustained 0.5 m/s command, no external force,
> `model_48800` — the policy tracks that command to 0.05 m/s, holds its base at 0.302 m and
> **falls zero times**, at 4.1 cm of tool-tip error. `unifp_isaaclab/rollout.py` under the same
> condition rolls the robot over within 0.4 s (roll reaching 2.4 rad) and reproduces the 0.75558 m
> and 161 fall-steps recorded here, to the digit.
>
> Which side to believe is not symmetric. The training environment is verified against the Isaac
> Gym original — observations to 6e-8, actions to 9.5e-6 (F-087), all 27 reward terms to 4.8e-07,
> the privileged observation to 3.5e-07 and total reward to 0.4% (F-089) — and its step ordering
> was checked line by line against `legged_robot_go2d1_pos_force.py`. `rollout.py` is a
> hand-written 50 Hz loop whose only verification is that it runs.
>
> **The cause in `rollout.py` was not found.** Three candidates were tested and all three were
> wrong, each making the harness *worse* rather than better: the foot friction (Isaac Lab's
> default material is 0.5, where UniFP's terrain is 1.0, and the robot's colliders inherit the
> default) left walking unchanged; restoring UniFP's documented physics → clock → observation
> order, which `rollout.py` genuinely violates by advancing the gait clock one step early,
> degraded **standing** from 0.140 m to 0.417 m. That every perturbation breaks it is itself the
> most informative result: this harness holds the policy on a stability knife-edge, which is
> consistent with the solver-iteration sensitivity recorded below, and the training environment
> does not. The standing and solver-sensitivity results here still stand; the walking result
> should not be cited.

- **Status:** confirmed as a negative transfer result for locomotion; the standing result is
  explicitly **marginal** and should not be quoted on its own
- **Week:** 1
- **Date:** 2026-09-20
- **Evidence:** `model_48800` run unchanged on both stacks, flat ground, no external forces, no
  randomisation or noise, one robot, a held velocity command, 1500 policy steps. Compared over
  steps 100–1000 (18 s) — the Isaac Gym side **resets at 20 s** (`episode_length_s`), so a longer
  window would average one continuous run against one-and-a-half episodes. Medians over the window:

  | condition | | Isaac Gym | Isaac Lab |
  | --- | --- | --- | --- |
  | zero actions | base height | 23.8 cm | 27.6 cm |
  | | steps below 15 cm | 0 | 0 |
  | standing command | tool-tip error L1 | **2.6 cm** | **8.7 cm** (p90 6.3 / 15.9) |
  | | base height | 29.9 cm | 31.2 cm |
  | | steps below 15 cm | 0 | 0 |
  | walking, 0.5 m/s | tool-tip error L1 | **3.3 cm** | **69.7 cm** |
  | | base height | 31.9 cm | 15.4 cm |
  | | steps below 15 cm | 0 | 122 of 900 (13.6%) |

  Runs: Isaac Gym [zero](#/week/1/run/20260920T0110_gym_flat_stand_zero),
  [standing](#/week/1/run/20260920T0112_gym_flat_stand_48800),
  [walking](#/week/1/run/20260920T0114_gym_flat_walk_48800); Isaac Lab
  [zero](#/week/1/run/20260920T012721_stand_zero),
  [standing](#/week/1/run/20260920T012721_stand_48800),
  [walking](#/week/1/run/20260920T012721_walk_48800),
  [zero with a walk command](#/week/1/run/20260920T012721_walk_zero).
  Figure: [unifp_sim2sim.png](week_01/figures/unifp_sim2sim.png).
  - **Standing survives the change of simulator; walking does not.** Told to stand, the policy
    holds the robot up for the full rollout in both simulators and tracks the moving goal to 8.7 cm
    against 2.6 cm — about three times worse, but the same behaviour. Told to walk at 0.5 m/s, it
    is on its side within **0.5 s** and never recovers, while the same command with zero actions
    leaves the robot standing — so the fall is the policy acting, not the command.
  - **The standing result is marginal, and that is the most important number here.** Raising the
    PhysX solver iterations from 4/0 (what `legged_gym` uses, and what Isaac Lab's stock Go2 ships)
    to 8/4 (what this repository's own tasks use) makes the same policy fall immediately and stay
    down for **100%** of the window ([run](#/week/1/run/20260920T013414_stand_48800)), while the
    passive robot barely moves — zero actions settle at 27.2 cm against 27.6 cm
    ([run](#/week/1/run/20260920T013414_stand_zero)). A setting that does not change whether the
    robot can stand decides whether the controller can.
  - **The numbers survived the simulator being rebuilt underneath them.** The runs above were taken
    on isaaclab **0.47.2**; the stack was then restored to the 0.54.3 / isaaclab-rl 0.5.0 /
    rsl-rl 5.0.1 combination this repository targets, and all four conditions were re-run
    ([standing](#/week/1/run/20260920T024333_lab0543_stand_48800),
    [walking](#/week/1/run/20260920T024333_lab0543_walk_48800),
    [zero](#/week/1/run/20260920T024333_lab0543_stand_zero)). Every median and p90 in the table
    above is identical to the digit, so the Isaac Lab minor version is behaviourally neutral here
    and the conclusion does not rest on the version that happened to be installed. Those runs are
    the canonical copies: they carry the simulator versions in `run.json`, which the earlier ones
    do not.
  - **The Isaac Lab side reproduced exactly on a re-run** — 8.74 cm median, 15.92 cm p90, 0 falls,
    every reported digit identical ([run](#/week/1/run/20260920T014015_confirm_stand_48800)). That
    is a property of a single-environment run with a fixed seed here, and it is worth stating next
    to F-086's ±0.1 cm spread on the Isaac Gym side: the numbers in this table are repeatable, but
    repeatable is not the same as robust, as the solver row below shows.
  - **The walking failure, by contrast, is not a solver artefact.** At 8/4 the walking policy is
    also down, for 90.9% of the window against 13.6% at 4/0. Both settings agree that it falls;
    only the standing result moves. So the two halves of this finding carry different weight — the
    negative locomotion result is robust to the one physics knob tested, and the positive standing
    result is not.
- **Scope:** **one checkpoint, one training seed, one rollout per condition**, and neither
  simulator is bitwise reproducible (F-086 measures ±0.1 cm run to run on the Isaac Gym side alone;
  nothing establishes the spread here, and with a single rollout per cell these numbers carry no
  error bar at all). The two robots are built from different files — a generated URDF against a
  welded USD — the foot colliders differ (`replace_cylinder_with_capsule` against whatever
  `go2.usd` ships), the ground is a flat plane against UniFP's flat-but-rough trimesh, and the
  Isaac Lab side carries the added rotor inertia of F-087. Any of those could account for the gap;
  this does not separate them. The goal trajectories are **not** the same episodes — each stack
  draws its own — so these are distributions over a shared goal distribution, not paired
  comparisons. Nothing here involves hardware, and no gate covers it.
- **Implication:** the reproduction is not portable in the form it exists in, and the Thesis B plan
  should not assume a UniFP-derived policy can be moved onto this repository's Isaac Lab task by
  loading the weights. The useful reading is narrower and still worth having: the whole-body
  *posture* behaviour transfers and the *locomotion* does not, which points at foot contact as the
  first thing to investigate rather than the policy or the observation. It also argues that any
  policy this project intends to deploy should be trained on the stack it will be evaluated on —
  or that the two stacks be reconciled deliberately, with the zero-action standing height
  (23.8 cm against 27.6 cm) as the first target, since that gap involves no controller at all.

### F-089 — UniFP's whole task now runs in Isaac Lab and scores within 0.4% of the stack it was ported from, term by term; the number that established this also exposed a frame bug that had zeroed the main objective

- **Status:** confirmed for the task's reward structure (27 terms, aggregate); the learning side runs but is not compared against upstream's optimiser
- **Week:** 1
- **Date:** 2026-09-20
- **Evidence:** `unifp_train/` is UniFP's position/force task as an Isaac Lab `DirectRLEnv`,
  including the external-force schedule, the force curriculum, the adaptation-module actor-critic
  and its extra PPO loss on rsl-rl 5.0.1. Four separate checks, each against the training stack
  rather than against a restatement of it:
  - **The task.** `model_48800` driven through the Isaac Lab environment scores **0.17519**
    reward/step against **0.17586** in the Isaac Gym recording — −0.4%
    ([run](#/week/1/run/20260920T041756_play_seed42_baseline_48800), figure
    `results/week_01/figures/unifp_task_terms.png`). 23 of the 27 terms agree to better than
    0.0005/step. Tool-tip error 3.8 cm against 2.0 cm; zero actions score 0.09042 at 41.5 cm
    ([run](#/week/1/run/20260920T041744_smoke_seed42_baseline_zero)).
  - **The force schedule.** `unifp_train/forces.py` matches a transcription of upstream's
    `_push_gripper` to 1e-6 step-for-step over 1200 steps x 12 environments, through 35 pushes,
    29 of which run to completion and redraw their intervals.
  - **The wrench.** Half the environments pushed and half not from the same reset: 8 N down moves
    the tool tip 3.3 mm down, 8 N forward moves it 409 mm forward and 335 mm up. At ±8 N the wrist
    torque limits saturate in the arm's weak direction, so the force is at the edge of what the
    arm alone can resist.
  - **The network.** `model_48800`'s weights load into `models.UniFPActor` with `strict=True` and
    the loaded model reproduces the verified reference loader's actions to **0.0** and the
    decoder's output to 7e-09. The remaining `critic_body` loads into a stock rsl-rl `MLPModel`,
    so the checkpoint splits exactly along the seam between the two libraries.
  - **Training runs**: 1024 envs at 28k steps/s, estimator loss 3.63 → 3.44 and mean episodic
    return 3.3 → 13.8 over 12 iterations
    ([run](#/week/1/run/20260920T040910_train_seed1_smoke2)).
- **What the number found.** Before this, the same comparison read 0.141 against 0.176 and was
  written up as a simulator difference. It was a bug: the tool tip was measured in the simulation
  frame and its goal in the environment frame, so on Isaac Lab's environment grid they differed by
  metres and `tracking_ee_force_world` — `exp(-2 * error)` — was **exactly zero in every
  environment**. One of the three objectives contributed nothing to the reward or the gradient and
  nothing else in the task changed. It corrupted a second thing too, found only once training ran
  properly: the privileged observation's `ee_pos_sphere` block is built from the same two
  positions, so the adaptation module's supervised target was a radius measured across the
  environment grid. Its loss was 3.44 in the last smoke run before the fix and **0.006** in the
  first iterations after it — a 500x drop that no reward number would have shown.
  The bug surfaced only when the per-term training log showed that
  term at 0.0000 for twelve consecutive iterations while every other term moved; the aggregate
  reward comparison that preceded it had been explicitly labelled "a sanity check, not a
  verification", and it was right to be. `run_unifp_train.py` now reports the median tool-tip
  error and refuses a run above one metre.
- **Scope:** this is **not** a step-for-step verification of the environment. The two stacks draw
  their own velocity commands and goal trajectories, so the comparison is two samples of the same
  task — and the Isaac Gym sample is a *single environment over 160 steps*, which is why two of
  the four disagreeing terms (`stand_still`, `dof_pos_limits`) are terms that sample never
  exercises. Removing `stand_still` alone moves the port from 0.4% below Isaac Gym to 1.9% below.
  The two objectives are each 3–4% lower here and `action_rate_arm` is four times larger, which is
  consistent with F-087's added rotor inertia and F-088's transfer gap but does not separate them.
  The force comparison against a real Isaac Gym forced run is one environment over 1500 steps and
  is consistent only in the weak sense that its 45% duty cycle sits at the 12th percentile of this
  schedule's distribution. The learning side has been run, not validated: nothing compares the
  estimator's optimisation against upstream's, and **no policy has been trained to completion or
  evaluated against the frozen manifests**.
- **The run this enabled:** a 60,000-iteration training run started 2026-09-20 04:49 UTC at
  upstream's 4096 environments, seed 1, force curriculum at iteration 8,000
  ([run](#/week/1/run/20260920T044925_train_seed1_p0)). 39k steps/s, 2.53 s/iteration, about 42 h
  — the same order as the 39.85 h Isaac Gym run of F-083, on a laptop GPU with 5.5 GB of its
  12 GB spare. Watch it with `./unifp_train/watch_progress.sh`. **No result yet**: this entry
  records that it is running, not what it produced.
- **Implication:** F-088 said a UniFP-derived policy should be trained on the stack it will be
  evaluated on rather than ported. This makes that possible: the task the Isaac Gym run optimised
  is now available in Isaac Lab, agreeing on the objective it is optimising to within a fraction
  of a percent, with a network that provably loads the existing checkpoint. The remaining work is
  a training run and an evaluation on the frozen manifests — which, at 28k steps/s and 1024
  environments, is a comparable wall-clock cost to the 39.85 h Isaac Gym run of F-083. It also
  adds a methodological point worth carrying: the aggregate-reward check that preceded this one
  was correctly labelled as insufficient and was insufficient in exactly the way predicted, and
  what caught the bug was logging the reward **per term** rather than in total.

### F-090 — Training UniFP's task natively in Isaac Lab does not beat porting the Isaac Gym weights, and the training reward cannot tell you which checkpoint is any good

> **Superseded by [F-092](#f-092) (2026-09-23): its conclusion is reversed.** A complete
> 60,000-iteration run made after the F-091 fix reaches **1.5 cm with no falls** on this very
> manifest, against the ported policy's 3.9 cm, and beats it on 50 of 50 paired episodes. The
> checkpoints below came from a defective run stopped at 44%; read this entry as the record of
> what that run produced, not as a comparison between the two training paths. The finding that
> training return cannot select a checkpoint survives and is reconfirmed in F-092.
>
> **Explanation superseded by [F-091](#f-091) (2026-09-21); measurements stand.** The Isaac Lab
> checkpoints evaluated below came from a run made with a stale-observation defect that pinned the
> learning-rate schedule to its floor for its entire length. The evaluation itself is unaffected —
> it suppresses resets and reads observations immediately — so every number in the table is what
> those checkpoints do. But the closing explanation, that "the native path is not yet competitive
> and its instability is unexplained", no longer holds: the instability is explained and fixed, and
> what a corrected run produces is untested. Read the comparison as a statement about the
> checkpoints that exist, not about the training path.

- **Status:** superseded by F-092; measurements stand
- **Week:** 2
- **Date:** 2026-09-21
- **Evidence:** `unifp_train/eval.py` and `./run_unifp_train.py eval` score a policy against a
  frozen manifest **in Isaac Lab**, so the Isaac Gym-trained policy and the natively-trained one
  are measured in the same simulator on the same episodes. `results/manifests/unifp_isaaclab_validation.json`
  (50 episodes, seed 20260921, content `94e576a6…`) freezes the set; each episode carries a digest
  of the schedule the environment realised, and **every one of the twelve runs below reproduced
  its digest exactly**, which is what makes the comparison paired rather than merely similar.
  Forces are active throughout, terminations are recorded but not acted on.

  | policy | goal tracking, unforced (median) | p90 | falls | training return |
  | --- | --- | --- | --- | --- |
  | zero actions | 38.1 cm | 44.8 | 0/50 | — |
  | Isaac Lab, iteration 4,400 | 5.7 cm | 8.1 | 7/50 | ~159 |
  | Isaac Lab, iteration 12,000 | 49.2 cm | 56.3 | 9/50 | ~96 |
  | Isaac Lab, iteration 16,000 | 38.8 cm | 41.0 | **26/50** | ~17 |
  | Isaac Lab, iteration 20,000 | 8.0 cm | 10.6 | 0/50 | ~132 |
  | **Isaac Lab, iteration 22,000** | **6.2 cm** | 8.6 | 1/50 | ~130 |
  | Isaac Lab, iteration 24,000 | 7.3 cm | 10.2 | 0/50 | ~137 |
  | Isaac Lab, iteration 26,400 (last) | **60.8 cm** | 65.7 | 1/50 | ~134 |
  | **Isaac Gym `model_48800`, ported** | **3.9 cm** | **5.1** | **0/50** | ~154 |

- **The two results.**
  - **Porting wins.** The best checkpoint this training run ever produced tracks at 6.2 cm with one
    fall in fifty; the Isaac Gym policy run in the same simulator on the same episodes tracks at
    **3.9 cm with none**. F-089 built the native training path on the argument that a policy should
    be trained on the stack it will be evaluated on. On this evidence that argument does not pay:
    the ported weights are better here, in Isaac Lab, on Isaac Lab's own frozen set.
  - **The training reward is not a checkpoint selector.** Between iterations 20,000 and 26,400 the
    mean training return sits between 130 and 137 throughout, while evaluated tracking swings from
    6.2 cm to 60.8 cm — a factor of ten that the reward does not register at all. The final
    checkpoint of the run is the **worst** of the last five, and is worse than doing nothing
    (60.8 cm against the zero-action 38.1 cm). Anyone taking the last checkpoint of this run, or
    choosing by training curve, would have shipped a policy beaten by an inert robot.
- **Scope:** one training run, one seed, one frozen set of 50 episodes, evaluated in one simulator.
  The per-episode spread is not reported here beyond the p90 and no confidence interval is
  attached to any cell, so two numbers within a centimetre or so of each other should not be
  ordered. The Isaac Lab run was **stopped at 26,466 of 60,000 iterations**, so this compares a
  partially trained policy against a fully trained one — the comparison favours Isaac Gym partly
  for that reason, and a completed run might close some of the gap. Nothing here is hardware, and
  no gate covers it. The manifest is specific to this simulator and cannot be compared
  episode-for-episode with `unifp_validation.json` on the Isaac Gym side (F-086's 2.6 cm), because
  the two stacks draw their schedules in different orders; the numbers are comparable in kind, not
  as paired episodes.
- **Tension with F-088, unresolved.** F-088 reports that this same `model_48800` "falls within
  0.5 s when told to walk" on the Isaac Lab model. Here it runs 50 episodes of 20 s with sampled
  velocity commands — the zero-action baseline's 0.259 m/s velocity error shows the commands are
  not trivial — tracks them to 0.067 m/s, and **falls zero times**. Both cannot be the whole
  story. The likeliest difference is protocol: F-088 commanded a single sustained walk for 30 s
  through `unifp_isaaclab/rollout.py`, where this samples from the task's own command distribution
  and resamples every 5 s. Until that is run down, F-088's locomotion claim should be read as
  specific to a sustained maximum-speed command rather than to walking in general.
- **Implication:** the immediate deliverable for Thesis B is the **ported** policy, not a natively
  trained one — it is the best controller measured on this stack and it exists today. The native
  training path is not wasted (the task port is verified to 0.4%, F-089, and its best checkpoints
  are within 2 cm), but it is not yet competitive and its instability is unexplained. Two things
  follow for how any future run is used: checkpoints must be selected **by frozen-manifest
  evaluation and not by training return**, and the evaluation is cheap enough — twelve runs in
  under an hour — that there is no reason not to.

> **Numbering note.** This finding was written as F-080 and renumbered on 2026-09-23, when the
> UniFP branch merged into `main` and the two lines of work turned out to have allocated
> F-067 to F-077 independently. The run directories and run names from that day still read
> `full_after_f080` and `after the F-080 fix`; they mean this finding.

### F-091 — The Isaac Lab training collapse was a stale-observation defect in the port: environments that reset had their stored observation blanked after the policy had acted on it, which fed the learning-rate controller garbage and pinned it to its floor for the entire run

- **Status:** confirmed — mechanism isolated, cause located in source, fix verified to remove it
  exactly and to reverse the 300-iteration degradation on five seeds; the effect on a full-length
  run is **not yet measured**
- **Week:** 2
- **Date:** 2026-09-21
- **Evidence:**
  - **The defect.** `ObsHistory.append` (`unifp_isaaclab/policy.py`) returns
    `self.buffer.reshape(...)` — a *view* of its ring buffer. `ObsHistory.reset` zeroed that same
    buffer **in place**. `DirectRLEnv.step()` runs `_reset_idx` *after* the policy has acted on
    the observation but *before* rsl-rl copies the transition into the rollout storage, so every
    environment whose episode ended had its already-used observation retroactively replaced by
    2432 zeros. The update then computed the policy on zeros and compared it with the action
    distribution the rollout had produced from the real state.
  - **It is a port defect, not upstream behaviour.** UniFP rebuilds its stack with `torch.stack`
    every step, which allocates, so its observation is never aliased to the history it came from.
  - **Measured, with both optimisers frozen** (`--fixed_learning_rate 0 --estimator_learning_rate 0`,
    so no parameter can move and any KL reported is spurious by construction):

    | resumed from | KL, first mini-batch — before | after |
    | --- | --- | --- |
    | fresh initialisation | 1.4e-05 | — |
    | `model_4000` | **0.0632** | **0** |
    | `model_10000` | **1.0553** | — |

    The first mini-batch of the first epoch has taken zero gradient steps since the rollout, so
    its KL is the implementation's numerical floor and nothing else. Against a `desired_kl` of
    0.01 the port was reporting 6x to 100x the target with nothing moving. The spurious KL grows
    through training because a more state-dependent policy responds more violently to having its
    input blanked, which is why it is invisible at initialisation.
  - **Direct check on the pairing.** Recomputing the action mean from every stored observation and
    comparing it with the mean stored beside it: **before**, at 4096 environments, the samples that
    disagreed were *exactly* the samples whose episode ended (3/3, 6/6, 7/7 across three
    iterations, with zero disagreements that were not resets and zero resets that did not
    disagree), and their stored observation was all-zero. **After**, 0 of 98,304 samples disagree
    while ~100 episodes still end per iteration. Offsets of ±1 step are far worse than offset 0
    both before and after, so this was never an off-by-one.
  - **Effect on the faithful configuration** (4096 environments, seed 1, 300 iterations, probe D's
    exact configuration, nothing else changed):

    | | before | after |
    | --- | --- | --- |
    | mean KL | 0.0226 | 0.0142 |
    | mini-batches above the upper threshold | 42% | **4.9%** |
    | mini-batches below the lower threshold | 2.8% | 9.8% |
    | learning rate, median | 1e-05 (the floor) | **1.98e-04** |
    | iterations with the rate at the floor | 57% | **0%** |
    | KL drift over 300 iterations | +180% | **+4.3%** |
    | `tracking_ee_force_world`, last 50 | 0.618 | **1.465** |
    | mean return, last 50 | 92.6 | **141.0** |

    The rate now sits where Isaac Gym's sits (its median over iterations 200–1,000 is 1.13e-04),
    and the controller moves in both directions again instead of only down.
  - **The saturation this explains.** In the 26,468-iteration run the learning rate reached the
    1e-5 floor at iteration 130 and stayed there for **100.0% of every iteration afterwards**,
    while the Isaac Gym run of the same task spent **0%** of iterations 200–1,000 at the floor
    (median 1.13e-04) and only 38.5% late on. 42% of the port's mini-batches were above the
    schedule's upper threshold against 3% below, so the controller was asking for a rate below
    1e-5 and could not have one.
  - **The five-seed scan re-taken.** The 2026-09-21 scan concluded that degradation was the rule
    (four of five seeds, six of seven runs overall). Re-run seed for seed with the fix and nothing
    else changed — 1,024 environments, 300 iterations, UniFP's configuration — and screened with
    the repository's own `kl_drift.py`, which is what produced the original column:

    | seed | KL drift, before | after | `tracking_ee_force_world` change, before | after |
    | --- | --- | --- | --- | --- |
    | 7 | +10% | **+8%** | +6.9% | **+24.4%** |
    | 42 | +37% | **+5%** | −20.9% | **+17.4%** |
    | 3 | +51% | **+5%** | −21.3% | **+26.9%** |
    | 1 | +74% | **+12%** | −19.9% | **+21.0%** |
    | 2 | +90% | **+10%** | −30.5% | **+13.3%** |
    | probe D (4,096 envs, seed 1) | +180% | **+4%** | −41.4% | **+38.1%** |

    **Four of five seeds degraded before; none of five does now, and every one gains.** Median
    drift +51% → +8%, median end-effector change −20.9% → +21.0%. Every run made since the fix
    reports `kl_first_minibatch` of exactly 0.
- **What this retires.** The week-2 entries treated the "factor of roughly seven between this
  port's policy movement per unit learning rate and upstream's" as a property of the trained
  weights, and killed five candidate mechanisms against it. The factor was not a property of the
  weights: most of the KL being regulated was not produced by learning at all. Two specific
  conclusions go with it. **Mean KL under an adaptive schedule is a regulated quantity** — the
  controller's setpoint, not the policy's sensitivity — so the probe-to-probe comparisons of mean
  KL were comparing a controlled variable, which is why they were so insensitive to interventions.
  And the in-run KL *is* exactly quadratic in the learning rate, as the offline perturbation said:
  at a fresh initialisation, where the defect is negligible, a tenfold smaller fixed rate gives
  0.0098 of the KL against 0.01 predicted. The earlier "a tenfold cut is worth only 27%" was the
  defect's rate-independent KL swamping the part that scales.
- **Scope:** the fix is verified to restore the rollout/update pairing **exactly** (0 of 98,304
  samples disagree), to return the learning-rate controller to Isaac Gym's regime, and to reverse
  the 300-iteration degradation on all five seeds it was measured on. It is **not** verified over
  a full run: the first collapse in the long run appeared at iteration 5,399 and nothing has been
  run past 300 iterations since the fix, so what this shows is six of six configurations entering
  the regime that preceded health rather than the one that preceded collapse — a necessary
  condition, not the result. A small residual KL (~5e-4, decaying) remains in the
  frozen null test in mini-batches after the first and is not explained; it is twenty times below
  the schedule's target and the direct storage check is exactly zero, so it is not stale
  observations. Nothing here is hardware and no gate covers it.
- **Implication:** every native training result on this stack — the 26,468-iteration run, the nine
  probe configurations, the five-seed scan, and therefore F-090's Isaac Lab column — was produced
  with the policy's learning rate held ten to twenty times below what its own KL controller would
  have chosen, on gradients computed partly from blanked observations. **They should not be read
  as evidence about what this task trains like in Isaac Lab.** F-090's headline comparison stands
  as a statement about the checkpoints that exist, but its explanation ("the native path is not
  yet competitive and its instability is unexplained") is superseded here. The five-seed
  conclusion that "degradation is the rule" was measured entirely under the defect and should be
  re-taken before it is relied on. Whether a corrected run beats the ported policy is now an open
  question that a single full-length run can answer.

### F-092 — With the stale-observation defect fixed, training UniFP's task natively in Isaac Lab beats porting the Isaac Gym weights by a factor of two and a half, on every episode of the frozen set

- **Status:** confirmed on one frozen validation set of 50 episodes, one training run, one seed
- **Week:** 2
- **Date:** 2026-09-23
- **Supersedes:** [F-090](#f-090), which reached the opposite conclusion from a run made with the
  F-091 defect and stopped at 44% of its schedule.
- **Evidence:** a complete 60,000-iteration run of UniFP's configuration unchanged — 4,096
  environments, seed 1, 5.9 billion environment-steps, 43.5 h — with the F-091 fix in place, then
  scored against the same frozen manifest F-090 used
  (`results/manifests/unifp_isaaclab_validation.json`, 50 episodes, content `94e576a6…`). All ten
  evaluations reproduced every episode's schedule digest: **zero schedule or condition mismatches**,
  so the comparison is paired episode for episode rather than merely like for like.

  | policy | goal tracking, unforced (median) | p90 | falls | base vel err | training return |
  | --- | --- | --- | --- | --- | --- |
  | zero actions | 38.1 cm | 44.8 | 0/50 | 0.259 | — |
  | **Isaac Gym `model_48800`, ported** | **3.9 cm** | 5.1 | 0/50 | 0.067 | ~154 |
  | Isaac Lab, iteration 8,000 | 1.5 cm | 2.0 | **5/50** | 0.047 | ~173 |
  | Isaac Lab, iteration 16,000 | 2.4 cm | 3.0 | 0/50 | 0.049 | ~162 |
  | Isaac Lab, iteration 24,000 | 2.2 cm | 2.6 | 0/50 | 0.043 | ~164 |
  | Isaac Lab, iteration 32,000 | 1.6 cm | 2.0 | 0/50 | 0.042 | ~166 |
  | Isaac Lab, iteration 40,000 | 1.7 cm | 2.5 | 0/50 | 0.039 | ~168 |
  | Isaac Lab, iteration 48,000 | 1.8 cm | 2.4 | 0/50 | 0.037 | ~168 |
  | **Isaac Lab, iteration 56,000** | **1.5 cm** | 2.2 | **0/50** | 0.035 | ~168 |
  | Isaac Lab, iteration 59,999 (last) | 1.7 cm | 2.1 | 0/50 | 0.039 | ~169 |

- **The comparison is paired, and it is not close.** Against the ported policy episode by episode:

  | checkpoint | episodes where Isaac Lab is better | median paired gain | sign test |
  | --- | --- | --- | --- |
  | 8,000 | 50/50 | +2.56 cm | p = 2e-15 |
  | 16,000 | 50/50 | +1.54 cm | p = 2e-15 |
  | 32,000 | 50/50 | +2.28 cm | p = 2e-15 |
  | 40,000 | 49/50 | +2.30 cm | p = 9e-14 |
  | 56,000 | 50/50 | +2.48 cm | p = 2e-15 |
  | 59,999 | 50/50 | +2.34 cm | p = 2e-15 |

  Every checkpoint from 16,000 onward lands between 1.5 and 2.4 cm with **no falls in 50
  episodes**, so this is a property of the whole plateau and not one lucky checkpoint. Base
  velocity tracking improves with it, 0.035–0.049 m/s against the ported policy's 0.067.
- **The training return still cannot select a checkpoint**, and F-090's warning survives its own
  supersession. The **highest**-return checkpoint here is iteration 8,000 at ~173 — and it is the
  only one that falls, five times in fifty. Return varies over 162–173 across the ladder while
  falls go 5 → 0 and tracking goes 2.4 → 1.5 cm, in no particular relation. Selection by frozen
  manifest remains mandatory; what has changed is that the penalty for getting it wrong is now
  smaller, because there are no catastrophic checkpoints in this run to select by mistake.
- **Scope:** one training run, one seed, one frozen set of 50 episodes, one simulator. No
  confidence interval is attached to any median, so checkpoints within a few millimetres of each
  other — 1.5, 1.6, 1.7 cm — should not be ordered against one another; what the paired test
  supports is the gap to the *ported* policy, not the ranking inside the plateau. Nothing here is
  hardware and no gate is claimed. This manifest is specific to Isaac Lab and cannot be compared
  episode-for-episode with `unifp_validation.json` on the Isaac Gym side (F-086's 2.6 cm): the two
  stacks draw their schedules in different orders, so those numbers are comparable in kind only.
  The `estimator_err_n` column is recorded but not analysed here.
- **Implication:** the argument F-089 built the native training path on — that a policy should be
  trained on the stack it will be evaluated on — pays after all. F-090 concluded the opposite and
  was measuring a broken optimiser loop. For Thesis B the deliverable controller is now a
  **natively trained** one, `model_56000` of this run, at 1.5 cm with no falls in 50 episodes
  against the ported policy's 3.9 cm. The ported policy keeps its role as an independent reference
  measured in the same simulator on the same episodes, which is what makes the claim checkable.

### F-093 — Both policies track worst at goals low and in front, but for opposite reasons: the ported one oscillates across the whole workspace, the natively trained one is steady everywhere and simply under-reaches downward

- **Status:** confirmed on a 243-point held-goal sweep, standing, forces off; one checkpoint per policy
- **Week:** 2
- **Date:** 2026-09-23
- **Prompted by:** Lukas's observation, watching both policies in the viewer, that the arm "flops
  and fails to track" on trajectories going low and in front.
- **Why the frozen manifest could not show this.** F-092's evaluation reports one median per
  episode, and an episode sweeps the goal all over the workspace, so a region the policy handles
  badly is averaged in with regions it handles perfectly. `unifp_train/workspace_map.py` holds the
  goal still instead: one environment per grid point, three radii x nine pitches x nine yaws, the
  same goal for 600 steps, steady state read over the last 200. The goal is spherical about a
  centre riding with the base and **pitch is elevation**, so the bottom of the range is low and in
  front — at radius 0.45 and pitch −45 degrees, 32 cm forward and 17 cm off the ground.
- **The observation is correct, and it is the lowest pitch band in both policies:**

  | | native `model_56000` | ported `model_48800` |
  | --- | --- | --- |
  | median error over 243 goals | 1.5 cm | 3.2 cm |
  | lowest pitch band (−45 deg) | **3.9 cm** | **4.5 cm** |
  | everything above it | 1.4 cm | 3.1 cm |
  | ratio | **2.8x** | 1.5x |
  | worst single goal | 6.4 cm (pitch −45, yaw 0) | 7.7 cm |

- **But the two failures are different, and the difference is in the spread over time.** The
  standard deviation of the error across the measurement window:

  | | native | ported |
  | --- | --- | --- |
  | median over all goals | **0.4 cm** | **1.8 cm** |
  | lowest pitch band | 0.5 cm | 2.5 cm |
  | worst goal | 0.1 cm at a 6.4 cm error | 5.1 cm at a 7.7 cm error |

  The ported policy genuinely oscillates — its tip error swings by a couple of centimetres
  continuously, over most of the workspace and not only low down, which is what "flops" describes.
  The natively trained policy does not: it is steady to within half a centimetre everywhere,
  including where it is least accurate. Its low-goal failure is a **systematic under-reach**, the
  tip sitting a median 2.0 cm forward and 2.1 cm above the goal in the base frame.
- **It is not a joint limit.** Arm-joint saturation at the worst goals is 0.52–0.80 of travel
  (1.0 = against a stop), so nothing is jammed.
- **It is not under-exposure during training.** Because goals are interpolated between a random
  start and a random goal, the generator commands the extremes of the pitch range less often than
  a uniform draw: the lowest band gets 6.6% of commanded time against 11.1% uniform. But the
  **highest** band gets 6.6% too, and the policy tracks high goals at 1.5 cm. The exposure deficit
  is symmetric; the accuracy deficit is not, so exposure does not explain it.
- **What does correlate**, for the native policy, is where the goal is in space: error against goal
  height gives r = −0.49, and against distance from the task's own keep-out box r = −0.37. Goals
  within 10 cm of that box average 3.9 cm; goals more than 35 cm from it average 1.3 cm. The
  region the arm handles worst is the shell immediately outside the volume the task has declared
  off-limits, reached by working down and forward past the front legs.
- **Scope:** one checkpoint per policy, one seed, held goals only. **The probe measures steady
  state on a stationary goal and therefore does not measure tracking lag on a moving one**, which
  is what is actually on screen when watching a trajectory — some of what looks like flopping
  during a sweep may be lag this does not capture. Standing only, with the velocity command held
  at zero; low-goal tracking while walking is not measured. Forces are off throughout. Nothing
  here is hardware, and no gate covers it.
- **Implication:** the deliverable policy has a known weak region, and it is a bounded one — a
  steady 4 cm under-reach in the lowest tenth of the goal workspace, against 1.4 cm everywhere
  else, with no instability anywhere. That is a usable characterisation rather than a defect to
  chase: any task placed in the bottom of the workspace should expect roughly three times the
  tracking error, and any pressing or box task should be sited with that in mind. If it later
  needs fixing, the evidence points at the reach geometry rather than at the learning — the error
  is a steady offset, not a control failure.

### F-094 — UniFP's controlled point is the tip of one finger and the jaws are not in its observation, so opening them to grasp costs 2.6 cm of tracking; putting the travel back into the goal recovers all of it

- **Status:** confirmed (simulation only)
- **Week:** 2
- **Date:** 2026-09-23
- **Evidence:** [Week 2 log, 2026-09-23](week_02/notes.md), "Recreating the cup and combiner demos on the
  UniFP policy". Four placements of the cup demo's own path, run in clear air with the object and the
  furniture moved aside, the jaws held at a fixed opening for the whole run, error read at the end of a
  1.2 s hold on the grasp point (`--no_object --free_space --jaw_hold`).

  | jaw travel per finger | gap between the pads | tracking error at the grasp point |
  | --- | --- | --- |
  | 0 mm (the pose it trained in) | 17.2 mm | [**0.91 cm**](#/week/2/run/20260923T111858_cup_unifp_no_object_free_jaw0mm_seed1) |
  | 15 mm | 47.2 mm | [1.44 cm](#/week/2/run/20260923T111919_cup_unifp_no_object_free_jaw15mm_seed1) |
  | 30 mm (fully open) | 77.2 mm | [**3.50 cm**](#/week/2/run/20260923T111940_cup_unifp_no_object_free_jaw30mm_seed1) |
  | 30 mm, travel added back into the commanded goal | 77.2 mm | [**0.73 cm**](#/week/2/run/20260923T112102_cup_unifp_no_object_free_jaw30mm_seed1) |

  The 0.91 cm at the trained jaw pose is the same goal's 1.0 cm in the orientation probe and in F-093's
  workspace map, so the jaws are the whole of the difference.
  - **The mechanism.** `interface.TOOL_BODY` is `Link7_1` — one pincer — and `interface.single_obs` takes
    the first 18 of the 20 joints, dropping both jaws. `Joint7_1` is a prismatic joint that slides that
    pincer up to 30 mm along the hand's jaw axis. So a grasp moves the controlled point by a distance the
    policy has no observation of, and it tracks to where its trained geometry puts the tip. The residual is
    the travel, and it is lateral: 2.58 cm of the 3.50 cm total is across the approach.
  - **Ruled out.** Not the table — the same 3.0 cm appears with the table moved aside and the forearm
    contact sensor reading 0.0 N
    ([free space](#/week/2/run/20260923T111200_cup_unifp_no_object_free_seed1) against
    [table present](#/week/2/run/20260923T111221_cup_unifp_no_object_seed1)). Not the jaw-centre
    conversion — commanding the tool point straight at the target gives the same
    [3.6 cm](#/week/2/run/20260923T111330_cup_unifp_no_object_free_tipgoal_seed1). Not settling time — the
    error does not decay over a 4.8 s hold. Not the base, which drifts 4.7 cm and then holds, correlating
    with the error at r = +0.18.
- **Scope:** simulation only, one checkpoint (`unifp_go2d1_isaaclab_model_56000`), one goal region
  (radius 0.45–0.55, pitch 0–20°), four placements per row, standing with the velocity command zeroed and
  the force curriculum off. The correction is exact only because the demo knows the jaw travel and the
  geometry; nothing here says the policy could learn it. Whether the same offset appears on the Isaac Gym
  checkpoint was not tested.
- **Implication:** any manipulation task built on this policy must either put the jaw state in the
  observation and retrain, move the tool point onto the hand's own axis (`Link6` plus the jaw centre, where
  the travel cancels), or correct the commanded goal by the measured travel as the demo launcher now does
  (`UniFPDemoEnv.jaw_compensation`). Untreated it is 2.6 cm of error at exactly the moment a grasp is
  decided, against the 11 mm a 55 mm cup leaves inside the open jaws.

### F-095 — UniFP tracks a tool-tip position to about a centimetre and leaves the hand pointing wherever the goal happens to put it, which is why the policy alone picks up the cup 0 times in 16 and opens the box 2

- **Status:** confirmed (simulation only)
- **Week:** 2
- **Date:** 2026-09-23
- **Evidence:** [Week 2 log, 2026-09-23](week_02/notes.md), "Recreating the cup and combiner demos on the
  UniFP policy".
  - **The hand frame over the workspace.**
    [105 goals held still for 4 s each](#/week/2/run/20260923T102440_probe_seed1_orientation_probe)
    (`demos/unifp/orientation_probe.py`), standing, forces off. Median tool-tip error **1.2 cm**, worst
    3 cm, no falls — and the jaw axis anywhere from **4° to 87°** from horizontal (median 57°), the approach
    from **71° nose-down to 19° up** (median −30°). At a goal that is not moving at all the jaw axis still
    swings a median of **20°**, p90 46°, worst 93°; only **12 of 105** goals hold it steadier than 10°.
  - **Why.** `interface.py` declares `CMD_EE_ORN_R/P/Y` in the command vector and the task never writes
    them. Six arm joints, three numbers commanded, three free, and no reward term that would hold them.
  - **What it does to the demos.** 16 placements each, standing, object pose given, final configuration:
    the cup [0 of 16 picked up](#/week/2/run/20260923T112548_cup_unifp_seed1) with the jaw axis 72° from
    level at the moment the jaws close, and the box
    [2 of 16 opened](#/week/2/run/20260923T112735_combiner_unifp_seed1), latch released at 6, median door
    angle 0.5°. Position was never the problem in either: 1.6 cm and 2.8 cm at the grasp point.
  - **The hand's direction is a property of the goal, not a choice.** Approach elevation runs 40–47°
    nose-down at goals near the sphere's equator and reaches level only at pitch 20–45°. The first cup
    demo put the table at 0.42 m, where the hand arrives 38° nose-down, and the cup
    [rolled out of the jaws on every lift](#/week/2/run/20260923T112152_cup_unifp_wrist_seed1); raising the
    table to 0.60 m, chosen off the probe, is what made the pick possible at all.
- **Scope:** simulation only, one checkpoint, standing with a zero velocity command and the scheduled force
  pushes off. The probe holds each goal still, so it says nothing about the hand while the goal is moving or
  while the robot walks. "0 of 16" is this demo script at this siting; a different approach direction or a
  different object might suit the hand the policy happens to present. No perception ran in any of it.
- **Implication:** the plan already calls for this — "position-only … need not prohibit orientation
  commands. Match this pose interface in force-aware variants"
  ([thesis_b_plan.md](../docs/thesis_b_plan.md)) — and this is the measurement of what its absence costs.
  UniFP as ported is a reaching controller, not a manipulation controller, and the gap is not accuracy.
  Adding an orientation command and objective to `unifp_train` is the fix; F-096 is what can be done without
  one.

### F-096 — Servoing the wrist roll alone levels the jaws and lets the standing dog pick the cup up 12 times in 16 and open the box 11; adding the wrist pitch costs 15 cm of reach and fixes nothing more

- **Status:** confirmed (simulation only)
- **Week:** 2
- **Date:** 2026-09-23
- **Evidence:** [Week 2 log, 2026-09-23](week_02/notes.md), "Recreating the cup and combiner demos on the
  UniFP policy". `demos/unifp/wrist.py` replaces the policy's actions for chosen wrist joints with a
  two-vector orientation servo, from the standoff hold onward; the policy keeps the legs and the rest of
  the arm and sees the substituted actions in its own observation.

  | | jaw axis from level, at the grasp | tracking error, clear air | cup | box |
  | --- | --- | --- | --- | --- |
  | UniFP alone | 67–72° | 1.7 cm | [0/16](#/week/2/run/20260923T112548_cup_unifp_seed1) | [2/16](#/week/2/run/20260923T112735_combiner_unifp_seed1) |
  | + roll (`Joint6`) | **1.4°** cup, 79° lever | 5.3 cm cup, 3.5 cm lever | [**12/16**](#/week/2/run/20260923T112611_cup_unifp_wrist_seed1) | [**11/16**](#/week/2/run/20260923T112832_combiner_unifp_wrist_seed1) |
  | + roll and pitch (`Joint5`) | 1.0° | 15.5 cm | [3/4 knocked over](#/week/2/run/20260923T111642_cup_unifp_wrist_seed1) | not run |

  - **Why the roll is nearly free and the pitch is not.** `Joint6` rotates about the approach axis and the
    jaw centre lies *on* that axis, so rolling sweeps the jaw axis through every direction perpendicular to
    the approach — it can always bring the jaws level — while barely moving the point the grasp is placed
    by. `Joint5` swings the tool point about 20 cm per radian, and taking it from the policy takes away part
    of how the policy was reaching.
  - **What the roll still costs.** 5.3 cm against 1.7 in clear air on the cup, 3.5 against 2.8 on the lever.
    It is a decaying transient — 5.6 cm at the start of the approach, 2.4 cm by the time the jaws close on
    one traced attempt — and it scales with how far the jaws are open, because the F-094 correction rides on
    the jaw axis and rolling swings it: 30 mm of travel on the cup, 12 mm on the lever, 3.6 cm and 0.7 cm of
    cost.
  - **Two earlier servos that did not work**, kept because the reason is the design rather than the tuning:
    a rate command (`error × gain × dt`) leads UniFP's PD by a thousandth of what it takes to saturate the
    1.7 N·m limit, and the wrist barely turned
    ([4.5 cm, jaws still 62° off](#/week/2/run/20260923T110359_cup_unifp_wrist_seed1)); a full-authority
    servo on both wrist joints levelled the jaws and cost
    [8–9 cm](#/week/2/run/20260923T111350_cup_unifp_wrist_no_object_free_seed1).
  - **No falls** in any run on this page, either controller, either demo.
- **Scope:** simulation only, one checkpoint, 16 placements per cell, standing, object pose given, no
  perception. **Every figure in the table above was measured at `--num_envs 16`**, which F-098 later
  established is a condition rather than a detail: at `--num_envs 1`, the same placements and seed
  give 0/16 and 7/16 for the cup and 3/16 and 8/16 for the combiner. The comparison this finding
  makes survives — the servo beats the policy alone by a wide margin in both configurations — but
  the absolute rates do not transfer between them, and the per-placement outcomes are not
  reproducible at all. The cup is 55 mm against 77 mm jaws and the lever's spring is the default
  0.4 N·m. The
  substituted actions are fed back into the observation, so the policy compensates through what it still
  owns — but this is **not UniFP**, and a number from it must not be quoted as one. Of the 12 cup picks the
  median carry tilt is 21°, so the cup is held at an angle, not upright; one of 16 was knocked over. The
  five combiner failures were not diagnosed.
- **Implication:** a position-only whole-body policy can be made to manipulate by taking back exactly the
  one joint whose axis passes through the grasp, and no more. That is a usable stopgap and it is also the
  argument for the real fix: the roll servo is reconstructing, open-loop and at a measurable cost in
  tracking, the orientation term F-095 says the task should carry. Which joint to take is not a free choice
  — the measured difference between `Joint6` and `Joint5` is a factor of three in tracking error.

### F-097 — The UniFP demo opens the box in 8.3 s against the scripted arm's 18–25, and the difference is the arm model rather than the controller: it drives the D1 at 1.77 rad/s where the measured joint ceiling is 1.21–1.29

- **Status:** confirmed (simulation only)
- **Week:** 2
- **Date:** 2026-09-23
- **Evidence:** [Week 2 log, 2026-09-23](week_02/notes.md), "Recreating the cup and combiner demos on the
  UniFP policy". Same script, every phase duration scaled, 16 placements each, UniFP plus the roll servo.

  | clock | cup | combiner |
  | --- | --- | --- |
  | 1.0× | [12/16 in 13.5 s](#/week/2/run/20260923T113030_cup_unifp_wrist_seed1) | [11/16 in 20.8 s](#/week/2/run/20260923T113054_combiner_unifp_wrist_seed1) |
  | 0.6× | [8/16 in 8.1 s](#/week/2/run/20260923T113153_cup_unifp_wrist_seed1) | [9/16 in 12.5 s](#/week/2/run/20260923T113211_combiner_unifp_wrist_seed1) |
  | 0.4× | [5/16 in 5.4 s](#/week/2/run/20260923T113251_cup_unifp_wrist_seed1) | [**11/16 in 8.3 s**](#/week/2/run/20260923T113307_combiner_unifp_wrist_seed1) |

  The combiner is as reliable at 8.3 s as at 20.8 s. The cup is not: at 0.6× more attempts close on the cup
  (15 of 16 lifted) but it tips past 45° in the jaws on the way up, and at 0.4× the pick goes altogether.
  - **Why the comparison does not hold.** Peak arm joint speed is **1.75–1.79 rad/s** in every one of these
    runs. The D1's measured single-command ceiling is 1.21–1.29 rad/s (F-033) and a 10 Hz setpoint stream
    through its firmware planner manages about 0.8 (F-046), which is what the scripted demos simulate
    (`--arm_actuator d1_servo --arm_trajectory measured --latency estimated`). UniFP's port drives the arm
    with UniFP's own PD and neither model. F-076 attributes the scripted demo's remaining 18–25 s mostly to
    the arm's own speed, so the like-for-like comparison is roughly 2.2× slower than 8.3 s — about 18 s, and
    no advantage at all.
- **Scope:** simulation only, 16 placements per cell, one checkpoint, standing with the object pose
  given, and **all of it at `--num_envs 16`** — see F-098, which makes that a condition. The success
  columns above should be read as "unchanged" or "degraded" within one configuration, not as rates.
  The script is clock-driven, so these are commanded durations, not times the arm chose; the arm falling
  short is what the success column measures. Timing excludes the 1 s spawn settle and all perception, where
  the scripted demo's 18–25 s is from the start of an attempt to letting go and includes its tag search.
  The two demos are not the same task either: these stand with the object on a table or a post, the scripted
  ones lie down with it on the floor.
- **Implication:** no speed claim can be made for UniFP over the scripted arm until both run the same arm
  model. Running the demo under `position_only`'s measured D1 profile is the experiment that would settle
  it, and it is worth doing before any of this reaches the thesis as a comparison. What the sweep does show,
  independent of the arm model, is that the box sequence has no timing margin problem — it tolerates a
  2.5× faster clock at the same success rate — while the cup pick does not.

### F-098 — The demos' success rate is a property of the simulator configuration, not of the placement: the same sixteen placements disagree on eleven of them between one environment and sixteen, while each configuration reproduces itself exactly

- **Status:** confirmed (simulation only)
- **Week:** 2
- **Date:** 2026-09-23
- **Evidence:** [Week 2 log, 2026-09-23](week_02/notes.md), "The demo numbers depend on how many
  environments are simulated". Found by running the combiner demo in the viewer for Lukas at
  `--num_envs 1`, which opened the door at [1 of 4](#/week/2/run/20260923T124649_combiner_unifp_wrist_seed1)
  where the recorded 16-environment run had opened 11 of 16.

  | controller | task | `--num_envs 16` | `--num_envs 1` |
  | --- | --- | --- | --- |
  | UniFP alone | cup | 0/16 | [0/16](#/week/2/run/20260923T131701_cup_unifp_seed1) |
  | + roll servo | cup | 12/16 | [**7/16**](#/week/2/run/20260923T132040_cup_unifp_wrist_seed1) |
  | UniFP alone | combiner | 2/16 | [3/16](#/week/2/run/20260923T131105_combiner_unifp_seed1) |
  | + roll servo | combiner | 11/16 | [**8/16**](#/week/2/run/20260923T125405_combiner_unifp_wrist_seed1) |

  - **Each configuration reproduces itself exactly.** Re-running the 16-environment command gave
    [11 of 16 again](#/week/2/run/20260923T125221_combiner_unifp_wrist_seed1), with the median door
    angle, lever angle and latch count identical to the decimal. This is not run-to-run noise.
  - **The per-placement outcome is not reproducible.** The two configurations draw the *same* 16
    placements from the same seed, and they disagree on **11 of them** — 7 opened only in the
    batched run, 4 only in the single. Agreement is 5 of 16, which is *worse* than two independent
    draws at these rates would give. Which placements succeed carries no information.
  - **Not the rendering.** Headless at one environment reproduced the viewer's
    [1 of 4](#/week/2/run/20260923T125045_combiner_unifp_wrist_seed1) on the same four placements.
  - **Not the reset path, which was the leading suspect.** Sixteen environments over 32 attempts
    runs batch 2 through exactly the sequential reset the single-environment run uses sixteen
    times: [batch 1 gave 11 of 16 and batch 2, after a full reset, 12](#/week/2/run/20260923T130839_combiner_unifp_wrist_seed1).
  - **Not grip margin.** The hypothesis was that the grip sits at the edge of what the arm can turn
    against the 0.4 N·m spring, so small differences decide it. Dropping to the 0.3 N·m the scripted
    demo was validated at made the single-environment case
    [*worse*, 4 of 16](#/week/2/run/20260923T130051_combiner_unifp_wrist_seed1), where the batched
    case was [unchanged at 11](#/week/2/run/20260923T130655_combiner_unifp_wrist_seed1). A weaker
    spring cannot make a marginal grip harder, so that explanation is dead.
- **Scope:** simulation only, 16 attempts per cell, one seed, one checkpoint. With n = 16, 8/16
  against 11/16 is about 1.2 standard errors — the *aggregate* difference between the two
  configurations is not established, and this finding does not claim environment count as a
  mechanism. What is established is the disagreement pattern at the placement level and the exact
  repeatability within a configuration. The three eliminations above are each a single experiment.
  No mechanism was identified; F-088 is precedent for this robot being decided by numerical
  configuration (a PhysX solver setting changes whether the policy stands at all while leaving the
  passive robot untouched), but that is an analogy, not evidence.
- **Implication:** every success rate from these demos must state its environment count, and
  per-placement results must not be quoted at all. F-096 and F-097 are amended accordingly. The
  binary criteria are what hid this — "door past 30°" is a threshold on a continuous quantity, and
  the lever angle it thresholds shows the effect immediately (single-environment failures all stall
  in a narrow 33–42° band, none past it). **Before any retraining is evaluated against these
  tasks, the criteria should move to the continuous measures and the denominator should grow**, or
  a training change will be scored against a metric that a change of environment count can move by
  as much as the change itself.

### F-099 — The D1 has essentially no dexterous workspace at this reach: 86% of UniFP's goal sphere admits a position, 43% a levelled hand and 8% a freely chosen orientation, so a full-pose objective would train mostly on unreachable targets — while roll alone costs nothing

- **Status:** confirmed (kinematics; solver-dependent, see scope)
- **Week:** 2
- **Date:** 2026-09-24
- **Evidence:** [Week 2 log, 2026-09-24](week_02/notes.md), "What to constrain, measured rather than
  argued". `unifp_train/pose_feasibility.py`, damped least squares over 147 goals of UniFP's own
  sphere — 3 radii x 7 pitches x 7 yaws — with 8 uniformly random orientations at each
  ([run](#/week/2/run/20260923T140424_ik_seed1_pose_feasibility)).

  | constrained | goals that solve |
  | --- | --- |
  | position only — what the task does today | **85.7%** |
  | position + roll (1 rotational DOF) | **85.7%** — no loss, analytic |
  | position + level (2 rotational DOF) | 42.9% |
  | position + a freely chosen orientation | **8.2%** |

  - **The sharper figure.** Of 147 goals, **none** admitted all eight orientations tried and **81
    admitted none of them**. A full-pose reward would spend most of training scoring targets no
    action attains — not a harder task but a corrupted one, since the term becomes noise on the
    return.
  - **Why roll is free, and it is an argument rather than a measurement.** A grasp's jaw axis is an
    *axis*: the two fingers are interchangeable, so a commanded jaw direction repeats every 180°.
    `Joint6` spans 269° hard and 242° at the soft limits, and an interval wider than 180° contains
    a representative of any commanded roll wherever the arm is sitting. So roll is reachable from
    *any* position solution; and with the controlled point moved onto the roll axis (F-094, F-096)
    reaching it does not disturb the position either.
  - **Consistent with the demos.** The roll servo of F-096 needed exactly this one degree of
    freedom, and got the jaw axis to 1.4° for the cup and 79° for the lever from the same rule.
- **Scope:** **the arm alone, with its base fixed.** UniFP has the whole body, and a policy free to
  pitch and shift the trunk has more to work with, so 8.2% is a lower bound for the whole-body
  system and the true figure is unmeasured. Damped least squares with one seed and a 300-iteration
  budget: a convergence failure is not a proof of infeasibility, and the solver's own staging of
  the level case lifts it from 24% to 51% (`d1_ik.solve`), so these are solver-dependent numbers
  rather than a kinematic certificate. The orientations are sampled uniformly over SO(3), which is
  harsher than a task would ask for. What survives all of that is the ordering and its size: the
  gap between 85.7% and 8.2% is far too large for the solver to account for, and "0 of 147 goals
  accept every orientation" is a statement about the arm.
- **Implication:** read "orientation tracking" in the plan's stage 6 as **one** degree of freedom.
  [docs/thesis_b_plan.md](../docs/thesis_b_plan.md) is corrected to say so. An orientation term
  added to `unifp_train` should command the gripper's roll about its approach axis and nothing
  else; a full-pose variant would need the goal sphere shrunk to the dexterous subset first, and
  that subset has not been mapped. Nothing here argues against the *force* half of the task: the
  constraint is kinematic redundancy, not reward capacity, and dropping force would free neither.

### F-100 — Commanding the gripper's roll works: 8 h of training takes the roll error from 62° to 5.7° for about 0.75 cm of tool-tip tracking, with no falls and no collapse

- **Status:** confirmed (simulation only, one run, 11,000 of 60,000 iterations)
- **Week:** 2
- **Date:** 2026-09-24
- **Evidence:** [Week 2 log, 2026-09-24](week_02/notes.md), "Eight hours of it: the roll objective
  works". 11,000 iterations at 4,096 environments, seed 1, jaw-centre controlled point and the
  roll objective, force curriculum at its default 8,000 iterations; 8 h 03 m, `status: complete`,
  56 checkpoints ([run](#/week/2/run/20260923T145001_train_seed1_roll_jaw_8h)).
  - **The comparison that matters is matched-iteration**, each policy measured on the point it was
    trained on, both with zero condition mismatches:

    | at iteration 11,000 | tracking | roll error | falls |
    | --- | --- | --- | --- |
    | [reference, position only, fingertip](#/week/2/run/20260924T022346_eval_seed1_ref11000) | **4.28 cm** | 62.3° (uncontrolled) | 1/50 |
    | [this run, roll + jaw centre](#/week/2/run/20260924T022236_eval_seed1_ladder10999) | **5.03 cm** | **5.7°** | **0/50** |

  - **The checkpoint ladder**, 50 frozen episodes each:

    | checkpoint | tracking | roll | falls |
    | --- | --- | --- | --- |
    | [4,000](#/week/2/run/20260924T022033_eval_seed1_ladder4000) | 3.46 cm | 2.9° | 3/50 |
    | [6,000](#/week/2/run/20260924T022057_eval_seed1_ladder6000) | 3.20 cm | 3.5° | 4/50 |
    | [**8,000**](#/week/2/run/20260924T022122_eval_seed1_ladder8000) — last position-only | **2.85 cm** | 2.5° | **5/50** |
    | [9,000](#/week/2/run/20260924T022146_eval_seed1_ladder9000) | 5.46 cm | 5.6° | 0/50 |
    | [10,000](#/week/2/run/20260924T022211_eval_seed1_ladder10000) | 5.86 cm | 6.9° | 1/50 |
    | [10,999](#/week/2/run/20260924T022236_eval_seed1_ladder10999) | 5.03 cm | 5.7° | 0/50 |

  - **F-092's curriculum signature reproduced exactly.** The best-tracking checkpoint is iteration
    8,000 and it is the one that falls most — 5 of 50, the same iteration and the same count F-092
    reported in the earlier run ("Isaac Lab 8,000 | 1.5 cm | **5/50** ... the only one that
    falls"). The force curriculum trades tracking for robustness at exactly the point it opens.
    Two independent runs, different task, same signature.
  - **Roll is learned early and cheaply.** It is under 3° by iteration 4,000, well before the
    curriculum, and degrades only to ~6° once the wrenches start. Training-curve position deficit
    against the reference ran 6.1% at iteration 1,500, closing to 2.4% by 6,000, widening to ~5%
    when the curriculum opened and closing again to 3.4% by 10,100.
  - **A criterion set wrongly, and stated because it changes how the result reads.** The
    pre-registered tracking bar was "better than 3.56 cm", which is the *released* `model_56000`
    measured at the jaw centre — a policy with 52,000 iterations of force training against this
    run's 3,000. That bar was unmeetable at 11,000 iterations however good the change was. Against
    the matched-iteration reference the deficit is 0.75 cm.
- **Scope:** simulation only. **One run, one seed, 11,000 of 60,000 iterations** — a diagnostic,
  not a deliverable. The roll weight (1.0) and width (sigma = 0.5 rad) are the first values tried
  and nothing has been swept. 50 frozen episodes per evaluation, so 0/50 against 1/50 falls is not
  a difference. The two policies in the matched table control points 2.4 cm apart and so are not
  measuring the same quantity; what the comparison supports is "adding roll did not break
  position tracking", not a ranking. The reference itself improves from 4.28 cm at 11,000 to
  1.5 cm at 56,000, so neither policy is near its own ceiling here and the 0.75 cm gap may narrow
  or widen with the remaining 49,000 iterations. Nothing here is measured on hardware, and the arm
  model is still not the measured D1 (F-097).
- **Implication:** the orientation gap F-095 identified can be closed by training, at a cost small
  enough to be worth paying, and F-099's argument for constraining *one* rotational degree of
  freedom survives contact with a trained policy. Continue this run to 60,000 rather than starting
  over — the resume path now restores the force curriculum (F-082 was still live in
  `run_unifp_train.py` until today). Select the final checkpoint on the frozen set and a ladder,
  never on the last iteration alone: the best-tracking checkpoint here is the one that falls.
  *Note, 2026-09-24 later:* the resume path could not have run when this was written — `train()` used
  `task_cfg` without importing it, so any `--resume_from` raised `NameError` before the first iteration
  ([failed run](#/week/2/run/20260924T070931_train_seed1_hook_train_smoke)). Fixed the same day; the first
  resume to succeed is [this smoke run](#/week/2/run/20260924T070954_train_seed1_hook_train_smoke).

### F-101 — Routing the pull through the arm's joints raises what the D1 holds from ~9 N to 40–59 N, but only with the base pitched to line the arm up; exact alignment promises 100–200 N and loses most of it to 2° of joint error

- **Status:** provisional (static model only: published joint limits, the weld's mass model, gravity + JᵀF;
  no servo loop, no structure, no simulator)
- **Week:** 2
- **Date:** 2026-09-24
- **Evidence:** [Week 2 log, 2026-09-24](week_02/notes.md), "Force through the structure";
  [`force_transmission_study.py`](week_02/figures/force_transmission_study.py),
  [figure](week_02/figures/force_transmission_study.png),
  [numbers](week_02/figures/force_transmission_study.json). The controlled point is the jaw centre
  (`Link6 + (0, 0, 0.1051)`, the UniFP task's since F-094).
  - **Typical postures** (97,470 body-clear configurations whose jaw centre lies in UniFP's goal shell,
    ahead of the base): a horizontal pull holds **5.6 / 9.4 / 14.7 N** (p10 / median / p90); a pull
    toward the mount 8.2 / 10.9 / 14.5 N; the arm's best direction 19.0 / 22.9 / 30.8 N and its worst
    1.6 / 2.4 / 4.0 N.
  - **Best posture for a horizontal pull** on a handle at 0.10–0.70 m, the arm posture optimised at each
    base height (0.24 / 0.30 / 0.34 m) and pitch (−15° / 0 / +15°, positive nose-down). Scored
    *robustly*: the 10th percentile when every joint is off by 2° (s.d.). Best over base height:
    **nose-down 15°: 43.8–59.1 N** for handles at 0.30–0.70 m (33 N at 0.55 m); **level: 18–33 N**,
    peaking at 0.40 m; **nose-up 15°: at most 21 N**.
  - **Exact alignment is fragile.** Nominal (zero-error) capacities reach 90–204 N at near-singular
    postures, and the same postures score 37–49 N robustly. At the best standing posture (base 0.30 m,
    nose-down 15°) the capacity is 62.4 N exact, **51.9 N at 2° joint error, 35.3 N at 5°, 21.4 N at
    10°** (p10); a 10° error in the pull's direction leaves 30.9 N at worst.
  - **Push and pull differ in stability, not in torque.** With the tool pinned (a hooked handle, a
    button under friction) and the arm's null space loaded, tension is stable at every force at both
    postures analysed. Compression buckles at a force proportional to servo stiffness: **43.6 / 108 /
    215 / 430 N at 0.1 / 0.25 / 0.5 / 1× the simulator's kp** (60/40 N·m/rad) at the best posture,
    32 / 81 / 162 / 324 N at a level posture.
  - **The body is not the limit.** A rigid 18.17 kg robot on its feet tips over its front feet at
    172 / 86 / 57 N for a horizontal pull at 0.2 / 0.4 / 0.6 m, 244 / 122 / 81 N leaning back 8 cm, and
    slides at 89 N on a μ = 0.5 floor.
- **Scope:** static and arm-only: no dynamics, no servo loop, no gearbox or link strength, the base
  treated as a rigid mount at three heights and pitches, the body-collision proxy ignoring pitch against
  the ground. The published torque limits and the weld's mass model are unverified on the arm (F-007,
  F-023). Robust capacity is a stochastic hill-climb from four seeds per cell, so neighbouring cells
  scatter by a few newtons (the 0.55 m dip is likely the optimiser). Buckling uses the simulator's kp; the
  real servo's stiffness is unmeasured, and its overload behaviour (F-028: torque-off drops the arm) is not
  modelled at all. Corrects my own earlier arithmetic to Lukas (2026-09-24, in conversation): "F ≈ τ/d, so
  2 cm of alignment gives ~85 N" ignored gravity and the fact that no posture puts the line through every
  joint at once to 2°.
- **Implication:** the force-transmission redesign has a real margin — about 4–6× the bent-arm pull —
  and it comes from the body's pitch and height choosing the arm's line, which is what a whole-body
  controller is for. It is not unbounded: ~50 N is the practical target for a pull, and a policy
  holding far more is exploiting a singular posture a real arm will not keep. Pulling is the strong case;
  pushing needs the real servo stiffness measured before any compression figure is believed. Measure on
  the bench before relying on any of it: a luggage scale on the straightened, powered arm.

### F-102 — The UniFP port's arm is in a numerical limit cycle: with zero actions its wrist flips torque every physics step at 75–90% of its limits and Joint3 sits pinned at its limit; a geared servo's rotor inertia (0.01 kg·m²) stops it and the holding torques then match the static model

- **Status:** confirmed (simulation; measured at the physics rate, one environment)
- **Week:** 2
- **Date:** 2026-09-24
- **Evidence:** [Week 2 log, 2026-09-24](week_02/notes.md), "Force through the structure";
  [`arm_chatter_probe.py`](week_02/figures/arm_chatter_probe.py) and its outputs
  ([2e-4](week_02/figures/arm_chatter_probe.json), [0.002](week_02/figures/arm_chatter_probe_armature_0.002.json),
  [0.01](week_02/figures/arm_chatter_probe_armature_0.01.json), [legs](week_02/figures/arm_chatter_probe_legs.json)).
  One standing robot, zero actions, UniFP's control law (`IdealPDActuator`, kp 60/40, kd 1.0/0.8, every
  5 ms), 560 physics steps after a 0.2 s settle.
  - **Found by an evaluation, not looked for.** A zero-action run of the new pull task
    ([run](#/week/2/run/20260924T070514_hook_eval_seed1_hook_eval_zero_loads)) reported the arm at 100% of
    its limit with nothing loading it.
  - **At the port's armature, 2e-4 kg·m² on every joint:** Joint4 / 5 / 6 change torque sign on
    **99.8 / 98.4 / 99.8%** of physics steps, at a mean 75 / 82 / 90% of their limits, and turn at 1.6–1.7
    rad/s RMS while nominally still. Joint3 sits at **−100% of its limit on every step**. Joint1 flips on
    73% of steps at 20%.
  - **At 0.002:** Joint4 and Joint6 (the rolls) still flip on 100% of steps; Joint5 is quiet.
  - **At 0.01:** Joint4 and Joint6 carry 0% and do not move; Joint2 and Joint3 hold the default pose with
    **41% and 69%** of their limits, against the static model's **32% and 63%** (F-101's model).
  - **The legs do not do it**: at 2e-4, sign flips ≤ 1.6% on all twelve, nothing saturated.
  - **Mechanism:** the explicit damper's kd·dt/I on the wrist links is about 19 against the ~2 an explicit
    integrator tolerates; the torque limit clips the resulting oscillation into a period-2 limit cycle.
    `unifp_isaaclab/robot.py` added the 2e-4 because at 0 the arm escaped its limits outright; that was
    enough to stop the escape and not enough to stop the oscillation.
- **Scope:** zero actions and one robot; the limit cycle under a policy's commands was not traced at the
  physics rate. 0.01 kg·m² is an estimate of a geared servo's reflected inertia (a ~1e-6 kg·m² rotor
  through ~100:1), not a measurement. Isaac Gym, where UniFP trained originally, was not probed.
  Consequence observed: the warm-start policy `model_10999`, trained on the chattering arm, did worse on the
  pull staircase once the arm was fixed — sustained force median 10 → 0 N, hooks lost 35 → 47 of 60
  ([before](#/week/2/run/20260924T070325_hook_eval_seed1_hook_eval_roll10999),
  [after](#/week/2/run/20260924T070831_hook_eval_seed1_hook_eval_roll10999_arm01)).
- **Implication:** every Isaac Lab UniFP result so far (F-087–F-100, including the released
  `model_56000`) was trained and scored on an arm whose wrist chatters at its limits, which is the likely
  reason `action_rate_arm` read four times upstream's magnitude (unifp_train/README). Tracking numbers
  are not invalidated by this, but **any claim about arm torque, load or force is**. The force-transmission
  task uses 0.01 on the arm (`hook_cfg.ARM_ARMATURE_KG_M2`); UniFP's own task keeps 2e-4 so the released
  checkpoints still reproduce. Whether UniFP's task should move to 0.01 too is a decision for the P0–P4
  ladder, and the D1's real reflected inertia is worth a bench estimate (a step response unpowered and
  backdriven).

### F-103 — Trained for it, the whole-body policy pulls 60 N through the D1 on a ring or a bar — six times what the arm holds in a bent reach — with the arm drawn straight along the pull and its motors at their limits

- **Status:** provisional (simulation only; one seed; a chain of four fine-tuning runs, figures from
  checkpoint 13,000 of a run still going when written; no hardware)
- **Week:** 2
- **Date:** 2026-09-24
- **Evidence:** [Week 2 log, 2026-09-24](week_02/notes.md), "Force through the structure";
  [figure](week_02/figures/hook_staircase.png). The task is `--task hook` (`unifp_train/hook_env.py`): UniFP's
  observation, actions and 27 rewards, plus a virtual fixture the tool engages and a commanded force into
  it, arm armature 0.01 (F-102). The policy is UniFP's `model_10999` (roll + jaw centre, F-100) fine-tuned
  through v1–v4 of the task, 2,000 iterations in all; what changed between versions, and why each earlier
  one stalled, is in the log.
  - **The experiment** (`run_unifp_train.py hook_eval`): 15 handle placements (0.2–0.6 m high × three
    bearings, 0.45 m reach) × 4 repeats, a horizontal pull straight back toward the robot at 2,000 N/m,
    commanded up a staircase 10 → 60 N, each level held 2.5 s and read over its last 1 s. *Sustained* is
    the highest level held (≥ 80%, contact and robot intact) with every lower level held too.
  - **On a ring** (the claw through a loop; cannot come off):

    | policy | sustained, median (p10–p90) | applied at the 60 N step | lost | fell |
    | --- | --- | --- | --- | --- |
    | [zero actions](#/week/2/run/20260924T084631_hook_eval_seed1_ring_zero) | 0 N | 0.5 N | 0 | 0 |
    | [UniFP `model_10999`](#/week/2/run/20260924T084249_hook_eval_seed1_ring_warm10999) | 10 N (0–21) | 9.4 N | 0 | 0 |
    | [v1 after 411 iterations](#/week/2/run/20260924T084441_hook_eval_seed1_ring_v1_11410) | 10 N (10–11) | 14.1 N | 0 | 0 |
    | [v4 at 12,600](#/week/2/run/20260924T083755_hook_eval_seed1_hook4_eval_v4_model_12600) | 35 N (0–60) | 45.6 N | 0 | 0 |
    | [**v4 at 13,000**](#/week/2/run/20260924T085620_hook_eval_seed1_ring_v4_13000) | **60 N (60–60)** | **60.7 N** | 0 | 0 |

    At 13,000 the applied force follows the command at every step (11.8 / 24.5 / 33.3 / 42.6 / 52.1 /
    60.7 N for 10–60 — overshooting by 1.8–4.5 N below 50 N), 58–60 of 60 episodes hold each level, and
    every handle height sustains 60 N. 60 N is the top of the staircase, so the ceiling is not measured
    there; [a longer staircase](#/week/2/run/20260924T090751_hook_eval_seed1_ring_v4_13000_to100) (20, 40,
    60, 70, 80, 90, 100 N — past anything trained) finds it: applied force levels off at **~70 N median**
    (67.1 / 70.4 / 72.1 / 70.9 N for 70–100), sustained 85 N median (p10 70, p90 100), from 75 N at a 0.2 m
    handle to 100 N at 0.6 m. Past 60 N the base pitch turns nose-up (−2 to −7°): the body leaning back
    against tipping, which F-101's rigid-body limits put at 86 N level and ~120 N leaning back for a 0.4 m
    pull.
  - **On a bar** (the claw must also stay seated; it can lift or slide off):
    [v4 at 13,000](#/week/2/run/20260924T090231_hook_eval_seed1_bar_v4_13000) sustains 60 N median, 54 of 60
    episodes hold 60 N, **5 of 60 lose the claw**, none fall.
  - **How the body does it.** Base pitch rises nose-down to +9–10° by 20–30 N and returns to level at 60 N;
    base height drops 2 cm (0.332 → 0.309 m). F-101's static study found the nose-down pitch the one that
    lines the arm up. The sideways load on the fixture at 12,600 was 4–6.5 N median, about the arm's weight.
  - **Where the load goes** ([paths run](#/week/2/run/20260924T085947_hook_eval_seed1_ring_v4_13000_paths)):
    through the arm's motors. No arm link touches anything (0 N median and p90, on a sensor that matched all
    nine D1 bodies), the trunk is untouched, and no arm joint comes within 0.01 rad of a limit. Joint1–3 sit
    at their torque limits, and Joint5 at 60 N: the pull draws the arm nearly straight along its own line,
    where the saturated torques balance the residual moments. That is F-101's tension case — stable and
    self-aligning — which is why the static *robust* estimate (15–20 N at a 0.2 m handle, 2° of random
    joint error) is pessimistic here: under tension the error is not random, the load pulls the arm into line.
  - **The force estimate follows**: the adaptation module reads 11.0 → 57.9 N for 11.8 → 60.7 N applied;
    UniFP's own reads 4.3–7.6 N at every level (trained on ±8 N).
  - **Cost to reaching:** on the frozen roll + jaw-centre free-space set, 7.1 cm median tracking against
    4.5 cm for `model_10999` on the same arm, 0 of 50 falls each
    ([v4](#/week/2/run/20260924T090428_eval_seed1_free_v4_13000),
    [warm start](#/week/2/run/20260924T082548_eval_seed1_free_warm10999_arm01)).
- **Scope:** simulation only, one seed, one chain of runs selected by looking at evaluations along the way
  (so the staircase is a development set, not a held-out one). The fixture is a virtual anchored spring:
  nothing moves, no door opens, the ring and bar are contact models, not geometry, and the claw does not
  exist yet. One pull direction, straight back; one stiffness, 2,000 N/m. The arm runs **at its published
  torque limits** for the whole pull; the real servos' continuous rating, heating and overload behaviour
  are unmeasured (F-028: a torque-off drops the arm), and 0.01 kg·m² of armature is an estimate. The arm
  contact sensor has not been shown to read a known contact (no positive control). Not validated on
  hardware. The ground is μ = 1.0 (UniFP's terrain); on a μ = 0.5 floor the robot slides at ~89 N, so
  the 70 N plateau is near what a real floor would allow, not comfortably inside it.
- **Later training changed the bar result** (F-104, 2026-09-24): continuing from v4 with presses added
  (v5–v9) kept the ring at 60 N but lost claws off bars in 28 of 60 episodes against 5. The checkpoint this
  finding describes, v4 `model_13000`, is the pull result.
- **Implication:** the redesign's premise holds in simulation, with room: the whole-body policy finds the
  posture and makes the force, six times the bent-arm figure and past F-101's robust static ceiling. What
  limits it now is not the task or the learning but the servo: a policy that holds 60 N with three joints
  saturated would trip or cook a real D1. Next, in order: measure the servos' stall and continuous torque on
  the bench (and the straight-arm pull with a luggage scale); raise `arm_torque_margin` until the arm keeps
  a margin, and see what force survives; then the press case, held-out placements and seeds.

### F-104 — The policy that pulls 60 N through a ring cannot yet press a button: its tool is never still and it lets a fixture carry the arm's weight; rewarding a still tool and letting the pad carry the weight at first makes pressing learnable, from 0 to 6 of 60 presses kept

- **Status:** provisional (simulation only; one seed). **Pushing is not solved**: v9 was stopped when its
  press-loss rate had plateaued, and its final checkpoint is worse on presses than its best (below)
- **Week:** 2
- **Date:** 2026-09-24
- **Evidence:** [Week 2 log, 2026-09-24](week_02/notes.md), "Force through the structure", v5–v9. The press
  staircase is the pull staircase pushed the other way: a pad on a button straight away from the robot at
  the same 15 placements, 2,000 N/m, lost if the pad backs off 3 cm or the button leaves the 4 cm pad
  (`hook_eval --press`).
  - **The pull policy has no push**: v4's `model_13506` lost [60 of 60 pads](#/week/2/run/20260924T092559_hook_eval_seed1_press_v4_13506),
    a median 3.0 s in, before the first 10 N level finished.
  - **Adding presses to training (35% of fixture episodes) did not by itself teach it**: 60 of 60 lost after
    ~330 iterations on a bare 2 cm button ([v5](#/week/2/run/20260924T093943_hook_eval_seed1_press_v5_13800)),
    ~290 with a 4 cm pad tool ([v6](#/week/2/run/20260924T094922_hook_eval_seed1_press_v6_14000)), ~60 with a
    slide-allowance curriculum (v7), ~370 with pads re-engaging in training
    ([v8](#/week/2/run/20260924T101002_hook_eval_seed1_press_v8_14400)).
  - **Why, measured.** 59 of 60 v6 pads [slid off a median 5 steps (0.1 s) after landing](#/week/2/run/20260924T095030_hook_eval_seed1_press_v6_14000_modes):
    4 cm of sideways tool motion in 0.1 s. Recording the direction on v8
    ([run](#/week/2/run/20260924T101134_hook_eval_seed1_press_v8_14400_slide)): a median 1.7 cm *down* by the
    time the pad went (down the main direction in 25 of 60) and 2.8 cm *across*. Two habits the pull training
    built and a ring or bar hides: the tool is never still (the policy saturates Joint1–3 even in free space),
    and the arm's weight rests on the fixture (a ring carries it; a pad at rest holds 1 N).
  - **v9 targets both**: a `fixture_tool_speed` penalty (−5.0 × squared tool speed while on a fixture), and the
    pad's resting friction on the press curriculum (15 N at the widest allowance, to the real 1 N at 4 cm). In
    training the fraction of press episodes that lost their pad fell from 0.86 to 0.40–0.50 within 200
    iterations, where v5–v8 had never left 0.95–1.0. On the strict staircase at
    [`model_14600`](#/week/2/run/20260924T102839_hook_eval_seed1_press_v9_14600): **6 of 60 presses keep the pad
    through the staircase**, 9 of 60 hold 10 and 20 N, 2 hold 30 N; while pressing it pushes 20–24 N whatever it
    is asked for, leaning back 4–5°.
  - **It did not continue.** Over the next ~700 iterations the training press-loss rate swung between 0.39 and
    0.59 without trending, the curriculum never tightened below 15 cm, and v9 was stopped at 15,293
    ([run](#/week/2/run/20260924T101344_train_seed1_hook_push_v9)). Its final checkpoint on the strict staircase:
    [60 of 60 pads slid off, 5 of 60 held 10 N](#/week/2/run/20260924T105445_hook_eval_seed1_press_v9_15293) —
    worse than at 14,600.
  - **What it cost the pull.** On a ring nothing: [60 N sustained (p10 60), 60 of 60 at 60 N](#/week/2/run/20260924T105459_hook_eval_seed1_ring_v9_15293),
    and closer tracking than v4 (11.0 / 19.8 / 27.4 / 37.3 / 46.9 / 56.3 N for 10–60). On a bar a lot:
    [40 N sustained, 28 of 60 claws lifted off](#/week/2/run/20260924T105534_hook_eval_seed1_bar_v9_15293) against
    5 for v4 at 13,000. Free-space reaching [6.3 cm median, p90 10.2](#/week/2/run/20260924T105627_eval_seed1_free_v9_15293),
    0 falls.
- **Scope:** simulation only, one seed, a development set. The press curriculum had not yet tightened below its
  starting 15 cm allowance when these numbers were taken, so training presses were far easier than evaluation
  ones. The simulator's arm gains keep the compression-buckling force (F-101: ~430 N at these gains) far above
  anything commanded, so this says nothing about buckling on the real servos, which is the risk F-101 names for
  pushing. The pad is a contact model, not a designed tool.
- **Implication:** pushing is a different skill from pulling, not a sign flip: it needs a still tool, and the
  pull-trained controller does not have one. Fine-tuning a pull policy toward it stalls halfway and erodes the
  claw-on-bar skill. The next attempt should not be another tweak on this chain: either a press policy trained
  on its own from the UniFP warm start with the steadiness term from the first iteration, or the steadiness term
  added to the pull task alone first and measured there (on a ring it cost nothing and tightened the tracking;
  a real D1 on a 10 Hz firmware loop cannot move the way these policies do, so it is wanted anyway). For the
  pull deliverable, use v4 at 13,000, not the later checkpoints.

### F-105 — Given only where a held handle should go, no existing policy finds the force: the 60 N pull policy drives at most ~30 N, and a policy trained on the outcome alone stops at the same ~30 N because nothing pays for trying harder until the mechanism gives

- **Status:** provisional (simulation only; one seed; invented mechanisms)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log](week_02/notes.md), "The goal is the input" (2026-09-24) and its v2–v3 continuation
  (2026-09-25). The goal-commanded task (`--task mechanism`, `unifp_train/mech_*.py`, `mechanism.py`): the claw
  already holds a handle on a one-degree-of-freedom mechanism (slide or hinge; spring, preload, stick-slip, a latch
  that snaps; drawn per episode, never observed), and the only command is a reference point sliding along the path.
  Evaluation (`mech_eval`): drawer, latch, door and button, 15 placements × peaks of 10–80 N, 480 episodes;
  *capacity* = highest peak with 12 of 15 opened, every lower one too.
  - Given only the goal: [zero actions](#/week/2/run/20260924T132157_mech_eval_seed1_mech_eval_zero) open 0;
    [UniFP `model_10999`](#/week/2/run/20260924T132244_mech_eval_seed1_mech_eval_unifp_10999) 47 (capacity 10 N on
    the latch only); [the pull policy v4 `model_13000`](#/week/2/run/20260924T132220_mech_eval_seed1_mech_eval_v4_13000)
    — 60 N when *told* to pull (F-103) — 25, capacity 0, largest drive 14–30 N. It still gives way by the
    compliance it was trained with (UniFP's target is `goal + F / 200`).
  - Trained on the outcome alone (`mech_v1`: handle-at-reference reward, UniFP's target made rigid, arm-margin,
    lunge and effort terms; warm-started from the pull policy): capacity 10–20 N and 79–89 opened
    ([`model_13200`](#/week/2/run/20260924T133901_mech_eval_seed1_mech_eval_v1_13200),
    [`model_13600`](#/week/2/run/20260924T135440_mech_eval_seed1_mech_eval_v1_13600)); the drive levels off at
    ~30 N at every resistance, and the curriculum stalled at a 45 N ceiling for 500 iterations
    ([run](#/week/2/run/20260924T132337_train_seed1_mech_v1)).
  - The saturation is a load, not a limit cycle: a physics-rate probe of signed arm torque
    ([`mech_arm_probe_v1_13600_drawer60.json`](week_02/figures/mech_arm_probe_v1_13600_drawer60.json)) has
    Joint2/Joint3 at −0.96/+0.98 of their limits, sign flips on 0.2%/0.1% of steps, steady within each robot.
- **Scope:** simulation only; the mechanisms, grasp and resistance ranges are invented (no hardware to measure);
  the grasp is a ball joint; the robot stands. One training seed, one evaluation seed.
- **Implication:** commanding the outcome instead of the force is not free. A reward that pays only for the
  outcome is flat below the breakaway force, and the policy never learns that more force would have worked; it
  needs a dense term for the push itself (F-107), or the escalation has to live outside the policy (F-106).

### F-106 — The goal can be turned into a force by the task layer: the pull policy, commanded by a three-line PI law on the handle's lag, opens 337 of 480 held mechanisms — 70 N drawers, 60 N latches — with no retraining, insensitive to the gains, and fails safe beyond its capacity; it inherits the pull task's directions

- **Status:** provisional (simulation only; one seed; invented mechanisms)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "The baseline this has to beat". The task layer knows
  the path (tags) and the handle's position and commands `F = kp lag + ki ∫lag` along the path (integral reset
  when it would push the other way, clamped to 80 N), with the goal on the handle as the pull task trained
  (`mech_eval --force_law`; gains chosen before the first run).
  - [kp 400, ki 1000, 80 N](#/week/2/run/20260924T140553_mech_eval_seed1_mech_eval_v4_13000_forcelaw): drawer 111 of
    120 (capacity 70 N), latch 100 (60 N), door 77 (40 N), button 49 (30 N): **337 of 480**, no falls, no tears. Drive
    reaches 67–75 N at 80 N peaks.
  - Gains: [ki 500](#/week/2/run/20260924T140754_mech_eval_seed1_mech_eval_v4_forcelaw_ki500) 325,
    [ki 2000](#/week/2/run/20260924T140909_mech_eval_seed1_mech_eval_v4_forcelaw_ki2000) 349; capacities within 10 N.
  - [Beyond capacity](#/week/2/run/20260924T141332_mech_eval_seed1_mech_eval_v4_forcelaw_stuck) (100 and 150 N,
    120 episodes): no falls, no tears, 72–77 N held for 14 s against the 80 N cap.
  - The lunge when a latch lets go at 40–70 N: 0.55–0.90 m/s against a 0.10 m/s reference, the same at every gain.
  - [Held-out directions](#/week/2/run/20260924T142833_mech_eval_seed1_mech_eval_v4_forcelaw_heldout): a lifted lid
    0 of 120 (2–5 N upward whatever it is asked), a sideways bolt 18 (capacity 10 N); pushing tops out at ~37–40 N
    and its force estimator reads ~5 N there.
- **Scope:** as F-105. The law reads the true handle position; a real task layer would have it from tags or the
  tool's own kinematics, with latency. The force-following policy was trained on pulls only (F-103, F-104).
- **Implication:** the "goal is the input" interface does not need an end-to-end policy: a force-tracking
  whole-body policy plus integral action in the task layer already opens most of these mechanisms, and is the
  baseline any learned goal policy has to beat. What it lacks is what its force policy lacks — other directions,
  pushing, and a gentler release — which is where training on the mechanisms can help.

### F-107 — With the push paid for, a policy given only the goal learns to escalate: it opens 358 of 480 held mechanisms with 70–80 N capacity on drawers, latches and doors — more than the hierarchical baseline — but it cannot push a button, leans the wrong way to try, and has no force limit of its own

- **Status:** provisional (simulation only; one seed; invented mechanisms)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "The goal is the input, v2–v3". Same task and evaluation
  as F-105.
  - **v2** adds `mech_push`: newtons driven toward the reference × (1 − progress), weight 2.0 against 4.0 for the
    progress term, so closing the gap always pays more than pushing from behind (tested). Within 800 iterations the
    drive follows the resistance — 10 → 69 N on the drawer for 10 → 80 N peaks, where v1's was flat at 30 N
    ([`model_14600`](#/week/2/run/20260924T144617_mech_eval_seed1_mech_eval_v2_14600)); at 1,200,
    [204 opened](#/week/2/run/20260924T150554_mech_eval_seed1_mech_eval_v2_15000), the door to 40 N capacity. Short
    mechanisms were left behind: a 3 cm latch or a 1.5 cm button cannot lag by more than its travel, so the push
    paid half — 8 of 15 latches at 10 N driven with 9.7–10.1 N and never moved.
  - **v3** scales the progress widths (and so the push term) to a short mechanism's travel. At 1,000 iterations,
    [`model_16000`](#/week/2/run/20260924T155944_mech_eval_seed1_mech_eval_v3_16000): drawer 111 (capacity 70 N),
    latch 120 (80 N), door 114 (80 N), button 13: **358 of 480**, no falls, no tears; the hierarchical baseline
    (F-106) 337.
  - **Pushing fails**: the button's drive is flat at ~21 N from 30 N up, pushed with the base 7.6° nose-*up*
    (leaning away; `model_15400`), where the baseline leans 12° into it and drives 38 N.
  - **No limit**: [beyond capacity](#/week/2/run/20260924T160232_mech_eval_seed1_mech_eval_v3_16000_stuck) it opens
    11–12 of 15 pull mechanisms at 100 N, drives 128–132 N at 150 N, and tears 3 of 15 drawers out of the claw.
  - [Held-out directions](#/week/2/run/20260924T160120_mech_eval_seed1_mech_eval_v3_16000_heldout): lid 0 of 120
    (lifts ~1.7 cm, lets it fall, stops trying), sideways bolt 31. [Free space](#/week/2/run/20260924T160339_eval_seed1_free_mech_v3_16000):
    3.3 cm quiet tracking, 3 of 50 falls (UniFP's walking-and-push manifest, which this policy does not train on).
- **Scope:** as F-105. The warm-start chain is pull v4 → v1 → v2 → v3, about 2,850 iterations on this task; the
  reward was changed twice on the way, each time after reading an evaluation, and the evaluation set is a
  development set, not a frozen test.
- **Implication:** "the goal is the input" is learnable end to end for pulls, and the body does the work: the
  arm's motors are at their limits (F-105) and the drive reaches 80+ N. But the outcome reward has to pay for the
  push as it happens, not only when it succeeds; the policy does not infer which way to lean from a goal that
  moves 1.5 cm; and a deployed version needs a force limit that this one does not have.

### F-108 — Trained with the task layer's force law in the loop, the whole-body policy opens 504 of 512 held-out mechanisms — every drawer, latch and door to 80 N, and the button it could not push before — leaning back into pulls and forward onto pushes; it does not stay under the law's force cap, and nothing it does can lift a lid

- **Status:** provisional (simulation only; one seed; invented mechanisms; the test set is frozen but was built
  on the same day as the policy)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "The hierarchical version trained with the law in the
  loop". `mech_law_v1`: the F-106 law writes the force command in training, the goal sits on the handle, the pull
  task's force-tracking term (4.0) replaces `mech_push`; warm-started from `mech_v3` `model_16000` (F-107), 1,500
  iterations ([run](#/week/2/run/20260924T155754_train_seed1_mech_law_v1)).
  - **Held-out test set** (`mech_eval --mech_test`: other placements, twice the mass, a softer grasp, a faster
    reference, more damping; frozen before any policy was scored on it; the final checkpoint named in advance):
    [final `model_17499`](#/week/2/run/20260924T171443_mech_eval_seed1_mech_test_law_v1_17499) opens drawer, latch
    and door 128 of 128 each and the button 120 of 128 — **504 of 512**, capacity 80 N on all four, no falls, no
    tears — against [338 for the untrained pull policy + law](#/week/2/run/20260924T170003_mech_eval_seed1_mech_test_v4_forcelaw)
    (F-106) and [346 for the goal-only policy](#/week/2/run/20260924T170120_mech_eval_seed1_mech_test_v3_16000) (F-107).
  - The development set: [480 of 480](#/week/2/run/20260924T171328_mech_eval_seed1_mech_dev_law_v1_17499); the button
    rose from 31 to 95 opened between 400 and 800 iterations as the base pitch at ≥ 40 N went from −6.6° (leaning
    back) to +3.1° (leaning in), and the drive from 25 to 50 N.
  - Costs and failures: 12 buttons (development set) torn out of the claw 2.5–10 s *after* being fully pressed, as
    the law's integral holds its force against the stop; lunges of 0.5–1.2 m/s when a latch or button lets go;
    the arm's motors at their limits throughout; [beyond capacity](#/week/2/run/20260924T171744_mech_eval_seed1_mech_eval_law_v1_17499_stuck)
    it opens every 100 N mechanism and drives 113–134 N at 150 N, past the law's 80 N cap (13 of 15 buttons torn
    there); [held out](#/week/2/run/20260924T171636_mech_eval_seed1_mech_eval_law_v1_17499_heldout) bolt 54 of 120,
    lid 0 of 120.
  - Why the lid fails: the arm alone holds a median 2.8 N upward (p99 11.8 N) over reachable postures, a third of
    any other direction ([`direction_capacity.json`](week_02/figures/direction_capacity.json)); leaning adds
    nothing upward.
- **Scope:** simulation only; invented mechanisms; the grasp is a ball joint; the robot stands; one training seed;
  the law reads the true handle position. The warm-start chain (pull v4 → v1 → v2 → v3 → this) is long, and
  whether the end-to-end steps are needed is being tested (log, "Control"; answered in the note below). The arm is simulated at its published
  torque limits, saturated for most of every push; a real D1 may not hold that.
- **Implication:** for "the goal is the input", the best of today's designs is hierarchical: a task layer that
  turns the handle's lag into a force command (integral action, and the direction to lean), and a whole-body
  policy trained on mechanisms *with that layer in the loop*. It opens nearly everything up to 80 N that pulls,
  slides or pushes. Before hardware it needs a force limit it actually respects, an integral that stops holding
  once the handle arrives, a softer release, and an honest answer about the servos at their limits. Lifting is a
  property of the D1, not of the controller; a box whose latch must be lifted needs a different grasp or tool.
- **Note, 2026-09-25 (later the same day):** the control answers the scope question. The same hierarchical training
  started from the plain pull policy (v4 `model_13000`, 1,500 iterations, no end-to-end stage) opens
  [493 of 512 on the test set](#/week/2/run/20260924T183123_mech_eval_seed1_mech_test_law_from_v4_14499) — every drawer,
  latch and door to 80 N, 109 of 128 buttons (capacity 30 N), 3 falls on doors — with the law's integral bled
  (`--force_law_bleed 1.0`). With the bleed, this finding's own policy opens
  [503 of 512 with none torn](#/week/2/run/20260924T172551_mech_eval_seed1_mech_test_law_v1_17499_bleed) and
  [480 of 480 on the development set with none torn](#/week/2/run/20260924T172436_mech_eval_seed1_mech_dev_law_v1_17499_bleed).
  So the end-to-end stage is not needed for pulls; it may help pushing, but the longer chain also had ~2,850 more
  iterations, so that is not a compute-matched comparison.
- **Note, 2026-09-25 — "does not keep to the cap" is about peaks, not holding.** Over the last 5 s held against the
  unopenable 150 N drawers, latches and doors, the median drive is 57.7 N (p90 64.7) for this policy, 60.6 N for law v2
  at 14,800 and 67.3 N for the untrained baseline — all under the 80 N cap
  ([mech_trace of the stuck run](#/week/2/run/20260924T171744_mech_eval_seed1_mech_eval_law_v1_17499_stuck)). The
  113–134 N are the largest *transient* drive, while force is being built; they are what opens the 100 N mechanisms
  and tears the 150 N buttons. A limit that matters for not breaking things has to bound the peaks.
- **Note, 2026-09-25 — a second seed.** The law-in-loop recipe from the pull policy, rerun with seed 2 (1,500
  iterations, [run](#/week/2/run/20260924T201420_train_seed2_mech_law_from_v4_seed2)): on the test set
  [450 of 512](#/week/2/run/20260924T212713_mech_eval_seed1_mech_test_law_from_v4_seed2) — every drawer, latch and door
  to 80 N again, but 66 of 128 buttons (capacity 20 N) against seed 1's 109. **The pulls reproduce; the push does not
  yet**, and this finding's 120 of 128 buttons came from the longer chain, not the recipe alone.
  Continued 1,000 more iterations ([run](#/week/2/run/20260924T212757_train_seed2_mech_law_from_v4_seed2_cont)), seed 2
  reaches [480 of 512](#/week/2/run/20260924T221608_mech_eval_seed1_mech_test_law_from_v4_seed2_cont) with 97 buttons:
  pushing is slower and seed-dependent, not absent.

### F-109 — Making the task layer's force cap a limit trades against capacity: with the outcome reward in training the policy treats the command as advice (146 of 180 mechanisms needing 60–80 N opened under a 40 N budget); as a pure force-follower it keeps closer to the cap (56 of 180) but loses the heavy door and an unseen direction, and no variant bounds the transient peaks

- **Status:** provisional (simulation only; one seed each; invented mechanisms)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "Law v2" and "law v3". Three policies from the same start
  (the law-in-loop control, F-108 note), final checkpoints, law with the integral bleed:
  - control `model_14499` (outcome reward + force tracking): [test 493 of 512](#/week/2/run/20260924T183123_mech_eval_seed1_mech_test_law_from_v4_14499);
    under a 40 N budget [146 of 180](#/week/2/run/20260924T192147_mech_eval_seed1_mech_budget40_law_from_v4_14499)
    mechanisms needing 60–80 N opened; held-out bolt 112 of 120 with 21 torn out of the claw.
  - law v2 (+ over-force price −1.0 per (10 N)², lunge price ×4, cap drawn per episode 30–100 N, bleed in training):
    [test 477](#/week/2/run/20260924T191856_mech_eval_seed1_mech_test_law_v2); budget
    [113 of 180](#/week/2/run/20260924T191944_mech_eval_seed1_mech_budget40_law_v2); at an 80 N cap
    [56 of 60](#/week/2/run/20260924T191922_mech_eval_seed1_mech_stuck_law_v2) 100 N mechanisms opened; bolt
    [116, none torn](#/week/2/run/20260924T192009_mech_eval_seed1_mech_heldout_law_v2).
  - law v3 (v2 without the outcome reward, over-force ×4): [test 457](#/week/2/run/20260924T201135_mech_eval_seed1_mech_test_law_v3)
    (door 104, button 113); budget [56 of 180](#/week/2/run/20260924T201222_mech_eval_seed1_mech_budget40_law_v3); at an
    80 N cap [15 of 60](#/week/2/run/20260924T201200_mech_eval_seed1_mech_stuck_law_v3), holding a median 39 N against
    a 150 N mechanism (v2 61 N) but peaking at a median 105 N (v2 115 N, `mech_law_v1` 120 N); bolt
    [41](#/week/2/run/20260924T201247_mech_eval_seed1_mech_heldout_law_v3).
- **Scope:** one seed per variant; 1,000 iterations each from the same checkpoint; "peak" is the largest drive in the
  push window, which includes the grasp spring's damping; the budget runs cap the *command*, not the measured force.
- **Implication:** a force budget set by the task layer is the right interface for hardware — the thesis's "force-aware"
  half — but a policy trained to open things will use the command as advice. Dropping the outcome reward mostly fixes
  that at a price in capacity and generality; neither bounds peaks. A peak limit should be enforced where the force is
  measured (a clamp or an abort in the task layer on the estimated force) rather than hoped for from a penalty.

### F-110 — In the standing combiner demo the mechanism policy, given a claw and the task layer's force law, opens the box in 16 of 16 placements against door closers up to 16 N·m (≈ 60 N at the handle), where UniFP with the same claw and script opens 9 of 16 free doors and none from 8 N·m; the lever turn, not the door, is what limits it

- **Status:** provisional (simulation only; one policy seed; the claw, springs and latch are models)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "The standing combiner demo under the mechanism policy".
  `run_demo.py --task combiner --mech` (`demos/unifp/mech.py`, `mech_env.py`): the F-108 policy (`mech_law_v1`
  `model_17499`) with its training's tool point and armature, a roll command, a claw that hooks the lever bar when
  the grip closes (2,000 N/m, torn out above 150 N), a goal correction during the reach's holds, and while the claw
  holds, the PI force law on the handle's lag along one box joint per phase (hybrid: goal on the handle along that
  joint, on the script's reference elsewhere), the door's force capped at 15 N while the latch holds; the script turns
  the lever to its stop and the claw hooks it 95 mm out. 16 placements per point, lever 0.4 N·m.
  - Door closer 0 / 4 / 8 / 12 / 16 N·m: `--mech` [16](#/week/2/run/20260925T000233_combiner_mech_law_seed1_door0_mechlaw),
    [16](#/week/2/run/20260925T000543_combiner_mech_law_seed1_door4_mechlaw), [16](#/week/2/run/20260925T000850_combiner_mech_law_seed1_door8_mechlaw),
    [16](#/week/2/run/20260925T001156_combiner_mech_law_seed1_door12_mechlaw), [16](#/week/2/run/20260925T001503_combiner_mech_law_seed1_door16_mechlaw)
    of 16 (door 58 / 44 / 41 / 40 / 35°); the same without the law 1 / 0 / 0 / 0 / 0; UniFP `model_56000` + wrist servo
    with the same claw, correction and script [9](#/week/2/run/20260925T000439_combiner_unifp_wrist_claw_corr_seed1_door0_oldclaw),
    [3](#/week/2/run/20260925T000747_combiner_unifp_wrist_claw_corr_seed1_door4_oldclaw), 0, 0, 0. No falls or tears in any run.
  - Placements not used for tuning (seed 2): [16/16 at 8 N·m](#/week/2/run/20260925T002850_combiner_mech_law_seed2_seed2_door8),
    [16/16 at 16 N·m](#/week/2/run/20260925T002953_combiner_mech_law_seed2_seed2_door16); one environment at a time
    [16/16 at 8 N·m](#/week/2/run/20260925T002051_combiner_mech_law_seed1_defaults_env1).
  - What the configuration needed, each found by a failed run: the goal correction (the policy arrives 5.1 cm off, a
    steady offset; 2.7 mm corrected); one joint per phase and a re-projected integral (both joints at once wound the law
    to 80 N on a 5 N lever); and the lever turned to its stop with the claw 95 mm out (pushing the lever down the policy
    delivers 6–10 N whatever it is commanded, and at 52° and 75 mm the lever stalled at 40–44°, short of the 45° latch).
  - Cost: peak claw force a median 47–103 N across the closers; the door ends 35–40° open at 12–16 N·m, not the
    scripted 50°.
- **Scope:** simulation; the claw is a spring-damper standing in for a claw tool that does not exist yet; the latch is a
  joint limit and the springs are drives; the task layer reads the box's joint angles (as tags on the lever and the
  door would report them) and corrects the reach with the arm's own kinematics; the configuration was tuned on the seed-1
  placements; the arm's motors run at their limits while it pulls (F-108); one policy seed.
- **Implication:** the goal-commanded design works end to end on the thesis's own box in simulation, and the part the
  new policy adds — pulling a resisting door with the body — is exactly where the old controller fails. The weak link
  has moved to the lever: pushing a short lever down is the D1's and this policy's weak direction, so the real box's
  lever torque, and whether its latch needs lifting or pressing, decide what the hardware demo can do. Measure them.
- **Note, 2026-09-25 (later):** the claw here is a spring that holds in every direction. Modelled as the L-lip geometry Lukas is
  building (F-111), every door still opens to 16 N·m, but the lever has to be hooked further in (80 mm) and kept against its stop,
  and from 12 N·m the lever comes out of the claw near the end of the pull.

### F-111 — Modelled as geometry, Lukas's L-lip claw opens the combiner box 16 of 16 against door closers to 16 N·m under the mechanism policy, but it only holds a pull that goes into its lips: from 12 N·m the swinging door turns the pull along the lip face and the lever comes out near the end of the pull, and a claw hooked near the lever's end loses it at once

- **Status:** superseded
- **Superseded by:** [F-112](#f-112), below (2026-09-25: this model put the lips *across* the fingertips, which stops the jaws
  at a 20 mm gap; Lukas's lips are beyond the fingertips and overbite, so the jaws close fully. Remodelled that way the
  results stand within a few degrees and a few escapes, so this finding's conclusion carries over to F-112)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "The L-lip claw as geometry". Each finger gets a 20 mm × 4 mm lip at its
  tip, turned inward, the two on opposite halves of the 26 mm finger width so they pass each other (`demos/unifp/claw.py`,
  colliders in a copy of the D1 URDF); PhysX holds the bar or does not. `run_demo.py --task combiner --mech` (lip claw by
  default), 16 placements, lever 0.4 N·m:
  - Final configuration — lever hooked at 80 mm, turned to its 60° stop and kept there through the pull, jaws rolled to the
    lever's measured angle while the door opens: door closer 0/4/8/12/16 N·m **16/16/16/16/16** opened (door 47/45/44/39/36°),
    the bar out of the claw in 0/0/0/5/16 ([16 N·m](#/week/2/run/20260925T033800_combiner_mech_law_seed1_door16_flaw)); goal
    only 0 at every closer; UniFP + wrist servo with the same claw, correction and script 11/10/1/0/0. Untuned placements:
    [16/16 at 8 N·m](#/week/2/run/20260925T034105_combiner_mech_law_seed2_seed2_door8_flaw),
    [16/16 at 16 N·m](#/week/2/run/20260925T034206_combiner_mech_law_seed2_seed2_door16_flaw). No falls.
  - Hooked at 95 mm (the spring claw's setting), [the bar left the claw in 4 of 4](#/week/2/run/20260925T025731_combiner_mech_law_seed1_lips_first):
    half the claw's width hangs past the 105 mm lever's end.
  - With the lever eased back to 5° before the pull, the stiff door turned the lever to its stop under jaws kept at the
    script's angle and the bar came out in 7 and 15 of 16 at 8 and 12 N·m ([12 N·m](#/week/2/run/20260925T030936_combiner_mech_law_seed1_door12_lipslaw)).
  - The late escapes are not the fingers being pushed open (a 50 N finger drive: [14 of 16 still](#/week/2/run/20260925T032210_combiner_mech_law_seed1_lips_hold2_door16_jaw50)):
    the bar stays pressed on the lips but drifts across the loop and out along the lever as the door swings
    ([recorded](#/week/2/run/20260925T032339_combiner_mech_law_seed1_lips_where_door16)); the policy commands no hand direction.
- **Scope:** the lip thickness, its rigid attachment, the gripper's 15 N / 800 N/m drives and PhysX's default contact friction
  are assumptions; one policy seed; the lever torque is the demo's 0.4 N·m; everything else as F-110.
- **Implication:** the claw design works for this task in simulation and changes what the task layer must do: hook the lever
  well inboard of its end, keep the lever against its stop while the door is pulled, and keep the pull going *into* the lips —
  turn the hand with the door (a wrist servo; the policy has no approach command) or stop the pull before the door has turned
  it aside. The claw's width against the lever's length, and the gripper's real drive force, are worth checking on the part.

### F-112 — With the lips modelled beyond the fingertips as Lukas specified, overbiting so the jaws close fully onto the bar, the L-lip claw opens the combiner box 16 of 16 against door closers to 16 N·m under the mechanism policy, within 5° of the lips-across model; from 12 N·m the lever still works out of the claw late in the pull, and the full close grips the bar harder

- **Status:** provisional (simulation only; lips, overbite clearance and finger drives are models of parts not yet made or measured)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "The L-lip claw corrected". Each lip is a 20 mm × 4 mm box 0.5 mm
  beyond the fingers' end faces (`demos/unifp/claw.py`, `LIP_FACE_Z_M` = 126.1 mm in Link6), from its finger's outer edge
  to 20 mm past its inner face, on opposite halves of the 26 mm finger width. A CPU test checks against the STL meshes
  that at full close each lip passes over the other finger's end without touching it. `run_demo.py --task combiner --mech`,
  16 placements, lever 0.4 N·m, hooked at 80 mm and kept against its stop, jaws closed past the URDF stop onto the bar:
  - Door closer 0/4/8/12/16 N·m: **16/16/16/16/16** opened (door 46/45/42/43/35°), bar out of the claw in 1/0/1/5/13
    ([16 N·m](#/week/2/run/20260925T041951_combiner_mech_law_seed1_door16_olaw)), median peak finger contact 59/26/92/118/124 N.
    Goal only: 1/0/0/0/0. UniFP + wrist servo with the same claw, correction and script: 13/6/0/0/0, bar out 1–2 of 16.
    Untuned placements: [16/16 at 8 N·m](#/week/2/run/20260925T042258_combiner_mech_law_seed2_seed2_door8_olaw) (out 0),
    [16/16 at 16 N·m](#/week/2/run/20260925T042400_combiner_mech_law_seed2_seed2_door16_olaw) (out 11). No falls.
  - Against F-111's lips-across model with the force law: door within 5° at every closer, escapes 5 and 13 against 5 and 16
    at 12 and 16 N·m (untuned, 16 N·m: 11 against 10). At 8 N·m the fingers now press the bar at a median 12 N while the
    lever turns (7 N before), and through the pull the bar's measured offset across the claw is 9.8 mm (17.3 mm before).
- **Scope:** the lip thickness, the 0.5 mm overbite clearance, the rigid attachment, the gripper's 15 N / 800 N/m drives and
  PhysX's default contact friction are assumptions; one policy seed; the lever torque is the demo's 0.4 N·m; everything else
  as F-110. Whether the bar leaves the claw is `mech_env`'s loop test (out for 0.2 s), not a contact measurement.
- **Implication:** F-111's implications stand with the correct geometry: hook the lever well inboard of its end, keep it
  against its stop through the pull, and keep the pull going into the lips. For the part, the overbite
  costs nothing in this task and gives back the squeeze. The late escapes at stiff doors come from the pull direction, not
  from where the lips are, so a wrist that turns the hand with the door, or a shorter pull, is the next thing to try.
- **Note, 2026-09-25 (later):** standing 15–30° to the door's hinge side removes those late escapes (none of 16 at −30°,
  even at 16 N·m) by turning the door's swing into a pull toward the robot; see F-113.

### F-113 — Where the robot stands against the combiner box trades the lever against the door: 30° to the latch side the mechanism policy turns a stiffer lever (spring claw: 14/16 latches released at 1.2 N·m against 0 square; the stiffness at which half release doubles), 15–30° to the hinge side the lip claw holds a 16 N·m door with no late escapes (0/16 against 13), and each side makes the other half of the task harder

- **Status:** provisional (simulation only; one policy seed, one set of placements, modelled lever spring, door closer, claws)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "Standing to the side of the box" ([figure](week_02/figures/combiner_stance_sweep.png),
  [door figure](week_02/figures/combiner_stance_door.png)). `run_demo.py --task combiner --mech --stance_deg`, the box turned about
  the grasp point so the reach is unchanged; positive is the latch side, where the lever's late turn draws the handle
  toward the robot, negative the hinge side, where the opening door comes toward it. 16 placements per point, the same
  draws at every stance:
  - Free door, lever spring 0.4–1.6 N·m at 45°, spring claw (holds in every direction), latch released: square
    16/6/0/0; +30° 15/15/14/8 ([1.2 N·m](#/week/2/run/20260925T050653_combiner_mech_law_spring_seed1_stp30_lev1.2_spring)); −30°
    16/15/8/1; +15° 5 and 2 at 1.2 and 1.6. The force held along the lever's arc at 1.2/1.6 N·m: square 10.6/12.4 N,
    +30° ≥15.8/16.9 N (36–50% more). Half the latches release up to about 0.8 N·m square and 1.6 N·m at +30°.
  - The same with the L-lip claw, latch released / opened: square 16/16, 10/10, 3/2, 0/0; +15° 15/15 at 0.8 N·m and
    14/13 at 1.2 ([run](#/week/2/run/20260925T053320_combiner_mech_law_seed1_stp15_lev1.2_lips)), 4/3 at 1.6; +30° 13/12, 15/10,
    11/10, 9/2 with the bar out of the claw in 4–15 of 16. The policy reaches along its own line to the handle, so the
    bar sits about as askew in the claw as the stance angle (median 18° at +15°, 33° at +30°, 5–6° square).
  - +45°: the forearm presses on the box in 15 of 16 attempts and the lever barely moves (4 of 16 released at any torque).
  - Lip claw, lever 0.4 N·m, door closers 12 and 16 N·m, opened / bar out: −30° 15/0 and 15/0
    ([16 N·m](#/week/2/run/20260925T054157_combiner_mech_law_seed1_stm30_door16_lips)); −15° 15/1 and 15/3; square 16/5 and 16/13;
    +15° 16/9 and 10/13; +30° 5/9 and 0/14. The one hinge-side failure per run is a latch that did not release.
  - The body's lean through the turn (spring claw): 6–9 cm back and 2–3 cm toward the latch side, rolled 5–7° that way,
    about 5° nose-up, from 2–3 cm forward before it — the same shape at every stance and lever torque.
- **Scope:** one policy seed and one set of placements; the lever spring and door closer are models; the "force held" is
  the spring's torque at the furthest angle reached, a floor where the lever reached its stop; the stiff-door runs used
  the 0.4 N·m lever and the lever runs a free door, so no run had both stiff; the robot stands still at each stance (it did
  not walk there); −30° takes the door pull's end up to 3 mm inside the trained 0.30 m goal radius.
- **Implication:** where the robot stands is a task-layer decision as real as the goal. The mechanism's geometry says
  which way each phase moves (the lever in the door's plane toward the latch side, the door toward the hinge side), and
  the policy's strength is along its own axis, not across it; standing so the stiff phase moves toward the robot uses
  that. For the real box, the lever torque and the door force decide the stance: a stiff lever wants the latch side, a
  stiff door the hinge side, both a robot that repositions between the turn and the pull. The lean Lukas saw is real
  (back and toward the latch side in every turn) but it is the policy's pull posture following the lever's arc, not
  evidence that it has found the lever's axis. The lip claw's tolerance to a bar askew, and an approach the task layer can
  command, are what limit the latch-side stance with that claw.

### F-114 — Trained on short levers turned down, the mechanism policy turns levers it could not (frozen lever test 32 → 118 of 120 square); in the combiner demo that shows only once the task layer corrects its entry and the training prices force across the path, and then the best stance moves to the hinge side, where it opens a 1.2 N·m lever against a 16 N·m door closer 9 of 16 (the old policy 4 at its best)

- **Status:** provisional (simulation only; one seed; modelled levers, springs and claws)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "Training the mechanism policy on levers"
  ([lever test figure](week_02/figures/mech_lever_eval.png), [demo figure](week_02/figures/combiner_lever_policy.png)).
  From `model_17499` (v0, F-108): v1 1,500 iterations with 30% of mechanisms short levers (5–12 cm, 40–75°, turned down
  about a level spindle pointing at the robot, ±45° for where it stands, 3–35 N;
  [run](#/week/2/run/20260925T085153_train_seed1_mech_law_lever)); v2 1,500 more with lever handles up to ~0.96 m and a
  price on force across the path ([run](#/week/2/run/20260925T111115_train_seed1_mech_law_lever_high)). Frozen tests, each
  fixed before it scored a policy, v0 / v1 / v2:
  - lever (the combiner's handle, 8 cm, 60°, ≤ 0.6 m), square / spindle side / tip side, of 120: 32/26/86 → 118/99/120
    ([v1](#/week/2/run/20260925T100416_mech_eval_seed1_mech_lever_18998)) → 98/89/117
    ([v2](#/week/2/run/20260925T122454_mech_eval_seed1_mech_lever_20497)); at 0.6–0.9 m, of 96: 24/34/36 → 72/79/72 → 73/77/56.
    v1 leans its weight onto a 40 N lever, pitched 10–12° nose-down, driving 46–48 N (v0 stalled at 15–25 N), and the lever
    lunges free at 1.0–1.4 m/s.
  - held-out mechanisms, of 512: 503 → 510 → [469](#/week/2/run/20260925T122546_mech_eval_seed1_mech_test_law_lever_high_20497_bleed)
    (v2's button 88 of 128; drawers, latches, doors 126–128).
  - the combiner demo, L-lip claw: v1 hooks the lever in 0–1 of 16 (a grasp 36–42 mm off; F-115); with the goal corrected
    through the entry it hooks every time but loses the bar in every attempt. v2, entry corrected, opened of 16 v0 → v2:
    hinge side (−30°) 15/11/2 → 16/16/12 at 0.4/0.8/1.2 N·m; square 16/10/2 → 16/11/11; latch side (+30°) 12/10/10 → 0/0/0;
    lever 1.2 N·m with a 16 N·m door closer, 4 (v0 at +15°) → [9](#/week/2/run/20260925T132419_combiner_mech_law_seed1_both_v2_stm30_door16)
    (v2 at −30°). Spring claw square: 16/6/0/0 → 15/14/13/10 at 0.4–1.6 N·m. v2 fell 3 times in 864 demo attempts (v0 none).
- **Scope:** one seed per stage; the levers, springs, door closer and claws are models; the demo's lever is one geometry
  at one height (0.68 m, raised once to 0.83 m); v2's demo runs use `--correct_entry`, which v0 did not need and which
  left v0 unchanged where tried (5 settings); the 40 N lever-test ceiling is a floor on v1's capacity.
- **Implication:** the lever skill is learnable and it transfers to the demo, but through the task layer: it needs the
  entry corrected, and it changes where the robot should stand — to the hinge side, where the door also wants it
  (F-113), so one stance now takes a stiff lever and a stiff door. The price is the latch side, the button and three
  falls. Which checkpoint to use depends on the box (`checkpoints/README.md`); the real lever torque and door force
  still decide it. Next, if this is pursued: a claw-like grasp in training rather than a price on its symptom (F-115).

### F-115 — The mechanism task's grasp is a ball-joint spring closed wherever the hand stops, so the training and its frozen tests cannot see the two things that decided the combiner demo: how precisely the hand arrives, and whether the force goes where a claw can hold it

- **Status:** provisional (simulation only)
- **Week:** 2
- **Date:** 2026-09-25
- **Evidence:** [Week 2 log, 2026-09-25](week_02/notes.md), "Training the mechanism policy on levers". v1 (F-114) improved on
  every frozen test and got worse in the demo: its jaw centre arrived a median 36–42 mm off with the lip claw (v0 2–5 mm)
  because it reaches pitched ~7° nose-up with its base ~9 cm back, set for a pull before it holds anything, and its lateral
  tracking of the moving grasp point degrades from that posture; the bar meets a lip tip and the hand stops 31–38 mm
  short. With the goal corrected through the entry (`--correct_entry`, 1.7–2.6 mm) it hooks every time and then loses the
  bar in every attempt, leaning back and dragging it toward the lips. The training's grasp error, newly logged
  (`Mechanism/grasp_error_m`), is 4.5–6 cm throughout v2: nothing has ever asked for a precise arrival. The force across
  the path, re-scored from saved traces (`across_p95_n`), is 8–18 N for v0 at every lever level and 36–50 N for v1 on
  heavy levers; v2's price brings it to 12–25 N on high levers and ~halves it on the held-out kinds.
- **Scope:** the claw-holdability proxy (`across_p95_n`) did not predict v1's losses at the lightest levers, where the
  lean along the spindle did; one demo, one claw design.
- **Implication:** a frozen test that the demo contradicts is testing the wrong thing. For a policy meant for a claw,
  the training and its tests need the claw: a grasp that closes only when the tool is at the handle and lets go when the
  force leaves what the claw holds (tool-conditioned, the thesis's third stage), and a test that fails a policy for it.
  Until then, the combiner demo with the lip claw is the check that matters, and a reach-precision and across-force
  report belongs next to every lever-test number.

### F-116 — A force-aware key insertion was built end to end -- slot model, task layer with a force law and a spiral search, an approach objective, a PhysX demo -- and did not train in the time given: in 2,500 iterations the mechanism policy learned not to ram the door but still arrives 9-10° and centimetres off, so 90% of attempts never reach the slot and none of 192 frozen slots is seated

- **Status:** provisional (simulation only; one training seed; the negative is about this recipe in this budget, not the task)
- **Week:** 2
- **Date:** 2026-09-26
- **Evidence:** [Week 2 log, 2026-09-26](week_02/notes.md), "Force-aware insertion of a handle into a key slot" ([figure](week_02/figures/insertion_training.png)).
  `unifp_train/insertion.py` (a 10 × 3 mm key rigid in the hand, its tip 70 mm beyond the jaw centre; a slot 0.6-1.5 mm
  wider each side with a chamfer, detent and seat; a task layer: goal to the estimated mouth, PI force along the
  estimated axis, capped low with a spiral search on the face) and an approach objective in `CMD_EE_ORN_Y` (on only
  where a level approach is reachable, 0.65-0.95 m up). Five runs from `model_17499`; the first four each found a flaw in
  the plant or the layer (a reference that let the hand hover, an unbounded face, a flat position reward traded for
  the approach, a soft face read as "in"), fixed with tests. The fifth, 2,500 iterations
  ([run](#/week/2/run/20260925T164048_train_seed1_ins_v5)):
  - training-time: 90% of insertion episodes missed the pre-insertion pose throughout; of the rest none seated, damage
    fell from over half to ~10% and peak contact from ~190 to ~50 N; the curriculum never promoted;
  - frozen evaluation, `model_19998`: [0 of 96](#/week/2/run/20260925T184708_ins_eval_seed1_ins_eval_lateral_v5b) seated across
    estimate errors (17 started, 9 entered and jammed, the approach 9.5° off against ~9° the channel admits) and
    [0 of 96](#/week/2/run/20260925T184734_ins_eval_seed1_ins_eval_detent_v5b) across detents; untrained
    [0 of 96](#/week/2/run/20260925T184800_ins_eval_seed1_ins_eval_lateral_17499b) (12 damaged, 3 falls);
  - PhysX demo, 16 attempts each: 0 seated at estimate errors of 0, 2 and 4 mm; the tip stopped 7-23 mm short of the door
    with no contact ([2 mm](#/week/2/run/20260925T184540_insertion_policy_seed1_v5_lat2)); with a task-layer tip correction
    the layer started in 6 and 10 of 16 (v5, untrained) and searched, and still [0 seated](#/week/2/run/20260925T184831_insertion_policy_seed1_v5_lat2_tipcorr).
- **Scope:** the slot is invented (sizes from common enclosure keys, not a measured lock), the key is rigid in a rigid
  hand, the estimate's error is drawn; one seed, one 2,500-iteration budget, one reward design at the end of four fixes.
- **Implication:** the precision the task asks for -- the key's tip within millimetres and a few degrees, 70 mm beyond
  the jaw centre -- is an order of magnitude past what this policy was built for, and asking for it from the first
  episode starves the curriculum: almost no episode reaches the part that pays. The next attempt should start where the
  policy can succeed (a wide slot, the layer starting from 5 cm) and tighten, keep the tip correction in the loop, and
  give the key a compliant grip or wrist so a tilted key can align, which is how insertion is usually made to work.
  The demo, the plant, the layer and a frozen evaluation are in place for it (`demos/insertion/`).
- **Note, 2026-09-27:** the next attempt did what this implication asked (outer loops on the measured key, a
  compliant grip, low arm noise) and seated keys: F-120, with why this recipe could not learn in F-119.

### F-117 — On the Go2 EDU the D1's base sits on the expansion dock's rails 85.5 mm above `base_link`, not 80, and -- with its rear screws 1 cm ahead of the rails' rear screws -- 20.5 mm behind it, not over it: the mount is (−0.0205, 0, 0.0855) m

- **Status:** superseded in part (2026-10-05)
- **Superseded by:** F-125 for the fore-aft place. The real arm's holes are 130 mm apart and sit over the rails' screws,
  so the mount is (−0.019, 0, 0.0855) m; F-126 for the base, which is not Unitree's mesh. The rails' height, 85.5 mm,
  and their screws' positions stand. Originally: provisional (from Unitree's CAD and the D1's mesh; which pair of
  screws Lukas's "1 cm forward" refers to is to be confirmed on the robot).
- **Week:** 2
- **Date:** 2026-09-26
- **Evidence:** [Week 2 log, 2026-09-26](week_02/notes.md), "Where the D1 sits on the Go2 EDU". `description/mount_from_cad.py` on
  Unitree's "GO2 simplified, with expansion dock" STEP (2023-09-11), registered to the URDF by the hip-pitch axes (385.8 mm
  apart against 386.8) and the hip-roll axes (±46.5 mm, exact): the dock's T-slot rails centred at y = ±29.0 mm, x −115.8 to
  +53.0 mm, tops at z = 85.5 mm; their four M3 fastening screws at x = −84.5 and +46.5 mm, y = ±29.0 mm. The D1's four M4
  holes at (±54, ±29) mm in its base frame (origin on its bottom face). The 58 mm across matches; fore-aft 108 mm against
  131 mm, so only one pair can be 1 cm ahead, and the front pair would put a D1 screw past the rails' end.
- **Scope:** the CAD is Unitree's "simplified" model; the registration is good to about 1 mm; the D1 is assumed square
  to the dog (its x fore-aft), which the 58 mm match requires; nothing measured on the robot yet.
- **Implication:** the weld `(0, 0, 0.08)` (`weld.py`, `flat_env_cfg.ARM_MOUNT_Z`, `description/build_go2_d1_urdf.py`) is
  2 cm too far forward and 5.5 mm too low. Moving it changes the robot every trained policy was trained on: it needs
  the G0 checks rerun (`run_position_only.py verify`), and the reach-limit and goal-sphere numbers ported from Rescue
  rechecked, before anything trained on it is compared with what came before.
- **Note, 2026-09-26 (later):** Lukas chose the other pair -- "Let's put it 1cm in front of the front screws" -- so the D1's
  front screws are 1 cm ahead of the rails' front screws and its origin is at **(0.0025, 0, 0.0855) m**, not the −0.0205
  above; in the CAD that puts its front screws 3.5 mm past the rails' end, to check on the robot. Applied everywhere
  from one constant (`mount.py`); the G0 checks pass at it ([verify](#/week/2/run/20260926T011051_054410Z_verify_seed42), 25 of 25).
  Policies trained before this were trained on (0, 0, 0.08).
- **Note, 2026-10-01 (re-checked):** [Week 3 log, 2026-10-01](week_03/notes.md) and `week_03/figures/d1_dock_holes.png`.
  The re-downloaded STEP (same sha256) and the committed meshes give the same screw and hole centres. The rail screws
  are the only four-hole pattern on the rails. Every model of the robot welds the D1 at `ARM_MOUNT_POS_M`, square. At
  that mount the D1's holes sit **+10.0 mm** (front pair) and **+33.0 mm** (rear pair) ahead of the rail screws, and
  0 mm across. No mount lines up all four (108 against 131 mm). This holds for the D1 base in `d1_arm/`; a new base
  with a different hole pattern has not been checked.

### F-118 — The reaching policy does not correct a shift of the arm's mount: its settled tool-tip error grows by about the whole shift (1.51 cm at the trained mount, 1.89 cm at the measured one, 5.5–6.0 cm at ±5 cm), because the actor observes joint angles, not its tip

- **Status:** provisional (simulation, one policy, one seed, the 50-episode frozen manifest)
- **Week:** 2
- **Date:** 2026-09-26
- **Evidence:** [Week 2 log, 2026-09-26](week_02/notes.md), "How sensitive the policy is to the mount", and
  `week_02/figures/mount_sensitivity.png`. UniFP `model_56000` (trained at the legacy weld (0, 0, 0.08) m) on
  `unifp_isaaclab_validation.json`, fingertip tool point, with the arm welded elsewhere (`run_unifp_train.py --mount_pos`).
  Settled error (`goal_tracking_quiet_m`), median:
  - legacy: 1.51 cm, p90 2.17;
  - the measured mount (+2.5 mm x, +5.5 mm z): **1.89 cm**, p90 2.72;
  - x −5/−3/−1.5/+1.5/+3/+5 cm: 5.54/3.47/2.17/2.39/3.78/5.99 cm;
  - z −4/−2/−1/+1/+2/+4 cm: 3.73/2.04/1.48/2.19/3.02/4.86 cm;
  - y ±2 cm: 2.93/2.68 cm.

  No falls at any mount. Beyond about 2 cm the median grows 0.85–1.1 cm per cm of shift. Every point lies on or above the
  policy's own miss with the whole shift added at right angles. The actor's observation (`unifp_isaaclab.interface.single_obs`)
  has no tip position; the measured tip goes only to the critic. The mechanism policy `model_17499` (force law,
  `--mech_test`, 512 episodes) opens 503/504/503/498 of 512 at the legacy mount, the measured mount, z +2 cm and x −3 cm,
  with only the button changing. That test grasps wherever the tool settles (`mech_env._engage_due`, F-115), so it
  cannot see a reach error.
- **Scope:**
  - Translations only; the D1 is assumed square to the dog, and a 1° yaw at 0.5 m of reach is about 9 mm.
  - Nothing on hardware.
  - No policy trained at the new mount.
  - The up/down asymmetry (1 cm lower is as good as the trained mount) is read as the policy's tip settling a few mm
    high of its goal (a fit to medians puts it about 6 mm high and 3 mm ahead). That is an inference; per-episode
    miss vectors are not recorded.
- **Implication:**
  - The mount has to be right to within millimetres on the robot. Each millimetre of error is roughly a millimetre at
    the tip, and the robot cannot see it: its tip estimate uses the same kinematics.
  - Policies trained before 2026-09-26 are about 4 mm worse on the median at the measured mount (1.51 → 1.89 cm). They
    should be retrained or fine-tuned at the new mount before new results are compared with old, and old results kept
    labelled with the legacy mount.
  - A goal measured by the wrist camera and carried into the base frame through the same kinematics would share and
    cancel a pure translation of the mount; a body camera's or a map's goal would not (reasoning, not tested).
  - Precision that needs the tip itself, such as the insertion's 1 mm clearance (F-116), has to be closed outside this
    policy: a task layer on a measured tip (`demos/insertion --tip_correction`), or the tip given to the actor, which
    changes the interface.

### F-119 — No policy this work starts from holds its hand still, and the pull lineage trains its arm under ~0.2 rad of noise: a millimetre task can neither be done nor learned through them as they are

- **Status:** provisional (simulation; measured on three policies in the insertion task's pre-insertion hold)
- **Week:** 2
- **Date:** 2026-09-27
- **Evidence:** [Week 2 log, 2026-09-27](week_02/notes.md), "Insertion, continued".
  - **The jitter.** Holding a still goal, the median tool speed over ~45 environments was 0.31-0.38 m/s for v5
    ([run](#/week/2/run/20260926T140440_play_seed1_ins_still_v5)), 0.23-0.31 for the mechanism `model_17499`
    ([run](#/week/2/run/20260926T140455_play_seed1_ins_still_mech17499)) and 0.37-0.44 for UniFP `model_56000`
    ([run](#/week/2/run/20260926T140509_play_seed1_ins_still_unifp56000)). The base moves at 0.01 m/s and the arm's joints at
    1.2-2.3 rad/s ([trace](#/week/2/run/20260926T140341_play_seed1_ins_v5_loops_motion)): a ~10 Hz arm jitter of a few
    millimetres, the habit F-104 saw on a pad. Keys it carries into a 1 mm channel are banged into the walls:
    under the outer loops, 76 of 96 frozen slots were damaged at 0-7 mm deep, the contact jumping from ~0 to over
    150 N within one policy step ([run](#/week/2/run/20260926T140053_ins_eval_seed1_ins_eval_lateral_v5_loops_newmount)). In
    the plant a 5 mm jitter puts 160 N on a rigid key; a key tilted 6° only 9 N (`tests/test_insertion.py`).
  - **The noise.** The pull lineage's checkpoints (v5 `model_19998`) carry an action-noise std of 0.69-0.89 on the six
    arm actions, against 0.48-0.62 on the legs: ~0.2 rad of random arm target every step at the 0.25 rad action
    scale. v6, trained through it, found no insertion in any training rollout
    ([run](#/week/2/run/20260926T140712_train_seed1_ins_v6)).
- **Scope:** the simulated D1's servo model. F-044 already argued this band of arm motion is one the real D1 cannot
  execute (its command-to-motion dead time and ramp). So the jitter is at least partly a simulation artefact, and on
  hardware it may be smaller or replaced by something else. Not measured on the robot.
- **Implication:** precision tasks through these policies need the jitter absorbed (a compliant grip, F-120) or
  priced (a stillness term, `ins_tool_speed`, in v6-v9), and training at a low arm noise (`--resume_arm_std`).
  Frozen evaluations are deterministic and do not see the training noise at all. A policy that looks fine there can
  still be one that no training rollout could ever have taught to succeed.

### F-120 — With the task layer closing its loops on the measured key, the key held in a compliant grip and the arm's training noise cut, the whole-body policy learns key insertion in 200 iterations: 55 of 96 frozen 1 mm slots in the plant, 13-14 of 16 in PhysX

- **Status:** provisional (simulation only; one seed; the grip is a model of hardware not yet built)
- **Week:** 2
- **Date:** 2026-09-27
- **Evidence:** [Week 2 log, 2026-09-27](week_02/notes.md), "Insertion, continued" ([figure](week_02/figures/insertion_v7.png)).
  - **The outer loops** (`insertion.PoseCorrection`): v5's approach missed by 7.9° in elevation, the tip by 10 mm, and
    only 33 of 96 keys entered the mouth. With the loops: tip 4 mm, roll 0.5°, and 93 entered. The elevation loop
    saturates, because v5 barely answers its elevation command.
  - **The compliant grip** (`insertion.CompliantGrip`, 2 N·m/rad about the jaw centre, ±10°, settled implicitly
    against the probed contact each physics step; PhysX: two sprung revolute joints). Untrained with it, v5 went from
    0/96 to 32/96 on the frozen lateral set ([run](#/week/2/run/20260926T142228_ins_eval_seed1_ins_eval_lateral_v5_grip)) and to
    23/96 without the loops ([run](#/week/2/run/20260926T142540_ins_eval_seed1_ins_eval_lateral_v5_grip_noloops)).
  - **Training** (v7 from v5: the grip, the loops, arm noise 0.15, entropy 0.002, a stillness term, an easier slot
    below curriculum level 1). Training rollouts went from 0.32 seated to 0.92 within 100 iterations, and the
    curriculum left level 0.2 for the first time. `model_20200` (`checkpoints/unifp_go2d1_insertion_model_20200.pt`):
    - frozen lateral set [55/96](#/week/2/run/20260926T143156_ins_eval_seed1_ins_eval_lateral_v7_20200_hold) (20 damaged);
      detent set [56/96](#/week/2/run/20260926T151148_ins_eval_seed1_ins_eval_detent_v7_20200);
    - PhysX, compliant key: [13/16](#/week/2/run/20260926T150702_insertion_policy_seed1_v7_20200_compliant_lat0),
      [14/16](#/week/2/run/20260926T143506_insertion_policy_seed1_v7_20200_compliant_lat2) and
      [13/16](#/week/2/run/20260926T150841_insertion_policy_seed1_v7_20200_compliant_lat4) at estimate errors of 0, 2 and 4 mm,
      0-1 damaged, 1.7-2.0 s to seat;
    - rigid key: [5/16](#/week/2/run/20260926T151020_insertion_policy_seed1_v7_20200_rigid_lat2) in PhysX and
      [0/96](#/week/2/run/20260926T151432_ins_eval_seed1_ins_eval_lateral_v7_20200_rigid) in the plant (86 damaged);
    - loops off: [0/96](#/week/2/run/20260926T151557_ins_eval_seed1_ins_eval_lateral_v7_20200_noloops), the roll 30° off: the
      roll loop carries it.
- **Scope:**
  - Simulation, one seed; the slot is invented and the estimate's error is drawn.
  - The loops need the tip and the hand's attitude measured: on a robot, from the arm's kinematics (subject to
    F-118) or a wrist camera.
  - The plant's damage (a summed penalty force past 150 N) is harsher than PhysX's.
  - What still fails is specific: at 0.86 m with the blade level the approach starts 7-9° off in elevation, against
    the grip's 10°, and most of the damage is there; at 0.70 m with the blade upright the tip is still 3-4 cm off with
    the tip loop at its 5 cm clamp. Both poses are reachable by the arm alone.
  - Training on past this checkpoint did not improve the frozen metric: v7's later checkpoints scored 47-51. A
    resume without a low learning rate threw v8 back to 2/96, because rsl-rl restarts its adaptive rate at 1e-3.
- **Implication:**
  - The key has to be held so it can tilt, about 2 N·m/rad over ±10° about the grip point. That is a hardware
    requirement for the combiner-box insertion, to design into the key holder or the claw.
  - The task layer (outer loops, force law, search, home hold) is part of the controller, not an add-on.
  - Supersedes F-116's "did not train" for this recipe; F-116 stands for the rigid key without the loops.
- **Note, 2026-09-27 (later):** the attitude loops' clamps were widened (roll 20 → 40°, elevation 20 → 30°) after a
  trace found `model_20200` holding a level blade at 0.86 m with its roll 30° off. With them it seats
  [59/96](#/week/2/run/20260926T153613_ins_eval_seed1_ins_eval_lateral_v7_20200_roll40) frozen slots (the 0.86 m level
  blades 5/16 against 0). They are the defaults from then on.
- **Note, 2026-09-27 (later still):** the training's part of this does not reproduce as well as it reads. Under the
  task layer as it now stands, v5 with no training seats 28/96. The recipe from v5 with seeds 1, 2 and 3 reached
  64, 42 and 20 at 200 iterations and 68, 36 and 32 at 400. Seed 1, this finding's, is the good one. The grip and the
  loops are what reliably take the task from 0; how much the training adds is seed-dependent. Week 2 log,
  "Does it reproduce?".

### F-121 — Continued from F-120's policy with the learning rate pinned and a slower curriculum, the insertion policy reaches 71 of 96 frozen 1 mm slots and 36-40 of 48 in PhysX with no damage; but every run peaks within a few hundred iterations and then declines while its training rollouts hold, losing the push through a detent

- **Status:** provisional (simulation; three continuations of one lineage, one seed each)
- **Week:** 2
- **Date:** 2026-09-27
- **Evidence:** [Week 2 log, 2026-09-27](week_02/notes.md), "Insertion, continued" (v8-v11).
  - **The resume shock.** rsl-rl restarts its adaptive learning rate at the configured 1e-3 on every launch, and the
    runs it resumes had sat at its 1e-5 floor. With the arm's noise at 0.15 the first update's KL was 243 (v7) and 286
    (v8). v8, resumed from v7's last checkpoint, spent ~100 iterations driving the arm into its joint limits, and its
    `model_21000` seated [2/96](#/week/2/run/20260926T152235_ins_eval_seed1_ins_eval_lateral_v8_21000). With `--fixed_learning_rate 1e-5` the first
    KL was 0.016 (v9).
  - **The best checkpoint**, v10 `model_21400` (`checkpoints/unifp_go2d1_insertion_model_21400.pt`), with the wider
    attitude clamps of F-120's note:
    - frozen lateral set [71/96](#/week/2/run/20260926T173120_ins_eval_seed1_ins_eval_lateral_v10_21400), 1 damaged; frozen detent set
      [71/96](#/week/2/run/20260926T173249_ins_eval_seed1_ins_eval_detent_v10_21400), 3 damaged;
    - PhysX, compliant key, 2 mm, 48 attempts: [40/48](#/week/2/run/20260926T180118_insertion_policy_seed2_v10_21400_compliant_lat2_n48_s2) and, on draws no
      checkpoint was chosen on, [36/48](#/week/2/run/20260926T180557_insertion_policy_seed3_v10_21400_compliant_lat2_n48_s3); 0 damaged, peak force ~13 N;
    - on the same seed-2 draws, v7 `model_20200` seated 34/48 and v9 `model_21000` 37/48;
    - by case on the lateral set, 12-16 of 16 everywhere except a level blade at 0.86 m, 0 of 16.
  - **The decline.** On the frozen lateral set: v9 55, 67, 49, 27 at 20,600, 21,000, 21,400, 21,800; v10 71, 52, 55 at
    21,400, 21,800, 22,200. Training rollouts held at ~0.86 seated throughout. The keys stop at the frozen set's detent:
    v9 `model_21000` pushed through 22/20/17/11 of 24 at 0/10/20/30 N, `model_21800` 23/11/5/1. Taking the stillness
    term off the slot's axis (v10) did not stop it. A trace at a 10 N detent
    ([run](#/week/2/run/20260926T175915_ins_eval_seed1_ins_eval_detent_v10_21800_trace77)) shows the layer commanding 30 N and
    the key not moving, the contact at 16 N, the jaw 30-37 mm from its loop-shifted goal.
- **Scope:** the checkpoint was selected on the frozen lateral set it is scored on; the detent set and a fresh PhysX
  seed are the checks on that. One seed per run. The mechanism of the decline is not established: the chase between
  the policy's rewards and the loops' shifted commands is being tested (v11, `--ins_true_targets`).
- **Implication:**
  - Resume an insertion policy with its learning rate pinned (or saved with the checkpoint).
  - Choose checkpoints by frozen evaluation, detent set included; the last is not the best, and the training
    metrics do not show the decline.
  - The remaining gap is a level blade high up (0.86 m), where the policy holds the blade 30° off in roll.
- **Note, 2026-09-27:** `model_21400` descends from seed 1, the best of three seeds of the recipe (see F-120's note);
  seeds 2 and 3 are being continued the same way to see whether the line, not the seed, is what reaches ~70/96.

### F-122 — The insertion recipe reproduces across three seeds once each is continued at a pinned learning rate: all three peak at 61-75 of 96 frozen 1 mm slots and then decline; the best, 75/96 and 71/96 detent, seats 43 of 48 in PhysX on two sets of draws with no damage

- **Status:** provisional (simulation; three seeds; checkpoints chosen on the frozen lateral set)
- **Week:** 2
- **Date:** 2026-09-27
- **Evidence:** [Week 2 log, 2026-09-27](week_02/notes.md), "Does it reproduce?". The recipe (from v5 `model_19998`: the
  compliant grip, the outer loops with the wider clamps, the home hold, arm noise 0.15, stillness across the axis)
  with seeds 1, 2 and 3. After 400 iterations: 68, 36 and 32 of 96, against 28 for v5 with no training under the same
  task layer ([run](#/week/2/run/20260926T203032_ins_eval_seed1_ins_eval_lateral_v5_grip_now)). Continued at a learning rate of
  1e-5 with a promotion at most every 200 iterations:
  - seed 1's line (v9, v10): peak 71/96 (F-121);
  - seed 2: 59, 61, 40 at 20,800, 21,200, 21,596;
  - seed 3: 71, **75**, 48.

  Seed 3's `model_21200` (`checkpoints/unifp_go2d1_insertion_seed3_model_21200.pt`):
  [75/96](#/week/2/run/20260926T225333_ins_eval_seed1_ins_eval_lateral_recipe_s3c_21200) lateral, 1 damaged;
  [71/96](#/week/2/run/20260926T225504_ins_eval_seed1_ins_eval_detent_recipe_s3c_21200) detent, 4 damaged; PhysX
  [43/48](#/week/2/run/20260926T225544_insertion_policy_seed2_s3c_21200_compliant_lat2_n48_s2) and [43/48](#/week/2/run/20260926T225659_insertion_policy_seed3_s3c_21200_compliant_lat2_n48_s3),
  0 damaged, peak force ~13 N. Each line peaks 0-800 iterations after its curriculum reaches level 1.0.
- **Scope:**
  - Simulation only, with an invented slot and a modelled grip.
  - The peaks are chosen on the frozen lateral set; the detent set and PhysX are the independent checks.
  - The 0.86 m level blade is still the case that fails.
  - Why each line declines is not known. Held against the curriculum's level-1 hypothesis in a running test.
- **Implication:** F-120's single seed overstated how reliably the first 400 iterations help, but not what the recipe
  reaches. With a pinned-rate continuation and checkpoint selection by evaluation, every seed got to 61-75/96. The
  procedure to use is train, evaluate every 400 iterations on both frozen sets, and keep the peak.
- **Note, 2026-09-27 (later):** "peaks and then declines" is noisier than it reads. Evaluated every 200 iterations,
  seed 3's line scores 71, 54, 75, 51, 48 from 20,800 to 21,596: swings of ~20 between neighbouring checkpoints,
  under a downward trend. Holding the curriculum at 0.8 does not stop the trend (58 and 30 at 21,200 and 21,599),
  and the arm's action noise does not collapse (0.12-0.21 throughout). The cause is not known. Choose on both frozen
  sets and PhysX together, evaluated every 200 iterations.
- **Note, 2026-09-27 (last):** the swings are whole cases flipping. Each (height, blade) case's 16 frozen slots differ only
  in the estimate's error and tend to succeed or fail together: a level blade at 0.86 m went 6, 12, 2, 16 of 16 over
  600 iterations of a continuation, so the frozen sets' effective sample is nearer six cases than 96 slots. PhysX,
  drawing heights and rolls continuously, is the better measure for choosing between good checkpoints.
  On 96 PhysX attempts each (seed 4), the best checkpoints were v10 `model_21400` 83/96 and seed 3's `model_21200`
  80/96, then v9 `model_21000` 77, seed 2's `model_21200` 72; all with no damage and no falls (Week 2 log, "The final
  choice").


### F-123 — Pressing is built as a thesis task, and the policies this work already has cannot do it: the insertion and mechanism policies hold 27 and 29 of 96 frozen panel buttons for a second, mostly with the probe, and fail by sliding off the cap; the bare jaws cannot reach a small recessed button at all, as declared

- **Status:** provisional (simulation only; analytic button; untrained-for-the-task checkpoints; one frozen set each)
- **Week:** 3
- **Date:** 2026-09-29
- **Evidence:** [Week 3 log, 2026-09-29](week_03/notes.md). `--task press` (`unifp_train/pressing.py`, `press_*.py`): a
  spring-loaded button (preload, spring, damping, a tactile click, a stop, the rim of its hole) pressed with the bare
  jaws (a 10 mm sphere 20 mm past the jaw centre) or a probe (a 6 mm ball 70 mm out). A task layer drives it from an
  estimate and a datasheet; success is the plan's "full travel held 1 s". Frozen sets `panel`, `force` and `recessed`
  were fixed before any policy was scored. Unchanged checkpoints:
  - `panel`: [zero actions 0/96](#/week/3/run/20260929T012154_press_eval_seed1_press_panel_zero) (none started);
    [insertion 21200 27/96](#/week/3/run/20260929T012129_press_eval_seed1_press_panel_ins21200_b) (probe 27/48,
    bare 0/48; 47 slid off); [mechanism 17499 29/96](#/week/3/run/20260929T012218_press_eval_seed1_press_panel_mech17499)
    (probe 18, bare 11; 66 slid off). Neither holds any button at 0.86 m (0/32 each).
  - `force`: [insertion 10/48](#/week/3/run/20260929T012244_press_eval_seed1_press_force_ins21200), only at 5 and
    10 N; [mechanism 7/48](#/week/3/run/20260929T012309_press_eval_seed1_press_force_mech17499) with **25 damaged**
    past 100 N, every 20–30 N button among them.
  - `recessed`: [insertion 15/72](#/week/3/run/20260929T012335_press_eval_seed1_press_recessed_ins21200): probe
    15/36, bare 0/36, and every bare episode "missed" (never touched the cap).
  - Why they slide: in the bare jaws' hold the tool carries about 8 N across the axis against 20 N along it. At the
    friction limit (μ = 0.4) it slides onto the panel.
- **Scope:**
  - Simulation only, and the plant is analytic, not PhysX.
  - The button, its forces and the friction are invented. The bare jaws are a sphere, where the real end faces are two
    flat fingers.
  - Neither checkpoint was trained on pressing; this is a baseline, not the task's capability.
  - `recessed` measures the declared access limitation: the geometry, not the controller, decides the bare result.
- **Implication:** the task needs its own training. Its first obstacle is the one F-104 met with a pad: the force has to
  go along the button's axis while the hand holds still across it. The probe's longer reach helps rather than hurts.
  The mechanism policy's 100 N+ presses on 20–30 N buttons are F-109's missing force limit showing up on a new
  fixture.
- **Note, 2026-09-29 (later): a 400-iteration pilot** from the insertion checkpoint at a pinned 1e-5 learning rate
  ([run](#/week/3/run/20260929T020153_train_seed1_press_pilot_c)) roughly doubles every frozen set:
  - [panel 49/96](#/week/3/run/20260929T022152_press_eval_seed1_press_panel_pilot_c) (bare 17, probe 32);
  - [force 20/48](#/week/3/run/20260929T022219_press_eval_seed1_press_force_pilot_c) (5 N 12/12, 10 N 8/12, 20–30 N
    0/24);
  - [recessed 23/72](#/week/3/run/20260929T022244_press_eval_seed1_press_recessed_pilot_c) (probe 23/36 with 6
    damaged; bare 0/36).

  Sliding off the cap is still the main failure. One seed, and the curriculum never promoted. The same pilot at
  rsl-rl's adaptive rate scored [0/96](#/week/3/run/20260929T020127_press_eval_seed1_press_panel_pilot_b): its first
  update's KL was 197, the resume problem F-121's log records.

### F-124 — Wiping is built as a thesis task; the insertion policy, untrained for it, keeps the pad on the panel for the whole motion but wipes 90% of the path in only 14 of 36 frozen episodes, because a rigid pad held 8° off the surface presses with its rim

- **Status:** provisional (simulation only; analytic surface; an untrained-for-the-task checkpoint; one frozen set)
- **Week:** 3
- **Date:** 2026-09-29
- **Evidence:** [Week 3 log, 2026-09-29](week_03/notes.md). `--task wipe` (`unifp_train/wiping.py`, `wipe_*.py`):
  - The pad is a rigid 50 mm disc 45 mm past the jaw centre, meeting the panel at nine points. It follows a straight or
    zigzag path at a set force on a panel that is nearer, further and tilted from its estimate.
  - A task layer holds the force along the estimated normal and runs a reference along the path, with a force to push
    through friction.
  - Success is the plan's: ≥ 90% of the path wiped (the centre of pressure within 2 cm of it while pressing) and ≥ 80%
    of the motion in contact. One frozen set of 36 was fixed before any policy was scored.
  - Results: [zero actions 0/36](#/week/3/run/20260929T012401_wipe_eval_seed1_wipe_zero);
    [insertion 21200 14/36](#/week/3/run/20260929T012439_wipe_eval_seed1_wipe_ins21200);
    [mechanism 17499 9/36](#/week/3/run/20260929T012520_wipe_eval_seed1_wipe_mech17499).
  - The insertion policy finished every path by the layer's measure and kept contact throughout. The normal force was
    within 2.2 N RMS of the command, peaking at 13 N on 5–10 N commands. All 22 failures were "incomplete".
  - Why, from the [diagnostic re-run](#/week/3/run/20260929T012649_wipe_eval_seed1_wipe_ins21200_diag): the pad is
    tilted a median 8.2° (p90 15°) to the true panel while wiping, and the contact sits 9.6 mm (p90 21 mm) off the
    path. Incomplete episodes average 9.2° against 5.6° for the successful ones.
  - With an ideal hand the same layer and pad cover 97–100% of a 16 cm stroke on panels up to 4° off
    (`tests/test_wiping.py`).
- **Scope:**
  - Simulation only, and the surface is analytic.
  - The pad is rigid by the plan's choice. Its stiffness and friction are invented.
  - The checkpoint was trained on keys, not wiping.
  - The 2 cm wiping radius is a declared definition, not a measurement of what a real pad cleans.
- **Implication:** on a fixed panel, the rigid pad's coverage is limited by the hand's attitude, and the command
  channel cannot fix all of it. Only the approach's elevation is commanded; its azimuth follows the goal's bearing,
  which swings as the goal runs along the stroke. A compliant or gimballed pad (the next tool the plan names, as the
  key needed its compliant grip, F-120) should decouple coverage from attitude. It is also the natural second tool for
  the P3/P4 tool comparison on wiping.
- **Note, 2026-09-29 (later): a 400-iteration pilot** at a pinned 1e-5 rate
  ([run](#/week/3/run/20260929T022310_train_seed1_wipe_pilot_c)) learns the training distribution: success from 0 to
  56% of episodes, contact during motion from 16% to 68%. On the frozen set it scores
  [16/36](#/week/3/run/20260929T024352_wipe_eval_seed1_wipe_pilot_c_eval) against 14, within noise. All 20 failures are
  still incomplete, and the pad's median tilt is 6.8° against 7.4°. This is consistent with the implication above:
  training the rigid-pad task does not remove the tilt.

### F-125 — The D1 is bolted to the rails' own screw positions, 19 mm further back and 5.5 mm higher than modelled; its resting tip moves (−22.6, 0, +7.5) mm and the task still needs the arm to move

- **Status:** superseded
- **Superseded by:** F-126, below (2026-09-30: the arm's base is taller than the model's, so the stance and task
  numbers here are replaced; the mount, x = -19 mm on the rail tops, stands)
- **Week:** 3
- **Date:** 2026-09-30
- **Numbered:** F-078 on main (de1442c, 2026-09-30); renumbered F-125 when merged into this branch, whose F-078 to
  F-093 are the UniFP findings (e9eefdd).
- **Evidence:** [Week 3 log, 2026-09-30](week_03/notes.md);
  [verify at the rail mount, 25/25](#/week/3/run/20260930T053155_769482Z_verify_seed42);
  [`stance_probe.py`](week_03/figures/stance_probe.py) with its
  [control](week_03/figures/stance_probe_old_mount.json) and [result](week_03/figures/stance_probe_rail_mount.json);
  [workspace model at the rail mount](week_03/figures/d1_workspace_rail_mount.json).
  - **Mount.** The real arm's holes are 13 cm apart along the rail (Lukas, measured). The rails' own screws are
    131.0 mm apart in Unitree's rail CAD, at x = −84.5 and +46.6 mm, and the rail tops are at z = 85.5 mm. So the
    base is centred at x = −19 mm and rests on the rail tops: `weld.D1_MOUNT_POS = (−0.019, 0, 0.0855)`,
    replacing (0, 0, 0.08), which had no measurement behind it.
  - **Stance.** Week 1's probe (16 envs, zero actions, 12 s, last 2 s) reproduced `task_space` exactly at the old
    mount. At the new one the base settles at (−0.0571, 0, 0.2736) m, 1.6 mm further back, and 0.25° more level
    (0.78°). The resting pincer tip is at (0.3375, −0.0141, 0.7679) m, (−22.6, 0, +7.5) mm from before. Peak tilt
    at reset is 4.75° (was 5.11°), and the lowest base height is 0.2597 m (was 0.2602).
  - **Task.** The target box is unchanged in the environment frame, so the frozen manifests keep their hashes.
    It is now 15.2 cm from the resting tip (verify; 14.3 cm before), with 0 of 8 sampled targets within 5 cm.
    The CPU model puts 98.92% of it reachable, clear and holdable at the settled height (99.06% before), and
    misses the measured tip by 1.3 cm (1.6 cm in F-015).
- **Scope:** simulation, zero actions, `run_position_only` defaults. Not validated: the mount on the robot itself
  (a ruler reading of the hole spacing, and the assumption that the hole pattern is centred on the arm's base
  origin), and the D1 model's base mesh, which has its holes 108 mm apart, not 130. The 256-env zero-action
  baseline of F-016 was not re-run.
- **Implication:** every checkpoint and result before 2026-09-30 is at the old mount, so an evaluation of one now
  runs it on a slightly different robot; `run.json`'s `arm_mount_m` separates them from here on. The arm's reach
  over the box is essentially unchanged, but scenes placed at fixed spots in the base frame (the cup and combiner
  demos) are now 19 mm further from the arm, and four of their tests no longer hold (Week 3 log).
- **Note, 2026-10-05 (merge):** on the `mechanism-demo` branch the mount this replaced was F-117's (0.0025, 0, 0.0855),
  not (0, 0, 0.08): the rails' height agreed, the fore-aft place did not, because F-117 had only Unitree's 108 mm hole
  pattern to place. That branch's checkpoints (UniFP, mechanism, insertion, the press and wipe pilots) were trained at
  (0, 0, 0.08) or at F-117's mount, all on Unitree's base; none at this one.

### F-126 — The real D1 base is 22.7 mm taller and 21 mm wider than Unitree's model, so the whole arm sits 22.7 mm higher; the resting tip is now (0.3378, −0.0141, 0.7906) m and the box stays 98.99% reachable

- **Status:** provisional (four dimensions measured by hand; the rest of the arm is taken as the model on one
  check and the published reach)
- **Week:** 3
- **Date:** 2026-09-30
- **Numbered:** F-079 on main (de1442c, 2026-09-30); renumbered F-126 when merged into this branch, whose F-078 to
  F-093 are the UniFP findings (e9eefdd).
- **Evidence:** [Week 3 log, 2026-09-30](week_03/notes.md), including Lukas's photograph of the base;
  [verify with the measured base, 25/25](#/week/3/run/20260930T065528_498258Z_verify_seed42);
  [stance probe](week_03/figures/stance_probe_measured_base.json);
  [workspace model](week_03/figures/d1_workspace_measured_base.json); [`d1_arm/measured_base.py`](../d1_arm/measured_base.py).
  - **Measured (Lukas).** The base is one continuous cylinder on a flange, 126 mm across (model 105.0), with
    holes 130 mm apart along the rail (model 108). From its bottom to the rotating plate it is 76 mm (model 53.3).
    The rotating plate to the shoulder pivot matches the model's 56.3 mm. The plan dimensions agree on one factor,
    1.20, but the height does not (1.43), so the real base is a different part, not an enlarged copy.
  - **The rest of the arm.** The model's chain reaches 554.9 mm from the shoulder to the wrist flange against
    Unitree's published 550 mm (excluding the jaw), and every link mesh agrees with its joint offsets. A uniform
    1.2 scale error would put the flange reach at about 458 mm.
  - **Model.** `base_link_measured.STL` is Unitree's base scaled 1.20 in plan, with the cylinder stretched to
    76.0 mm and the flange kept. Joint1 moves 53.3 -> 76.0 mm and the base's centre of mass is carried with it.
    D1Training and Rescue share the files.
  - **Sim.** The base settles as before, (-0.0571, 0, 0.2736) m, 0.78°. The resting tip is 22.7 mm higher, at
    (0.3378, -0.0141, 0.7906) m. verify: 25/25, Link6 kinematics to 0.74 µm, resting tip 2.3 mm from the
    recorded value, box 17.4 cm from the tip. The CPU model finds the box 98.99% reachable, clear and holdable.
- **Scope:** the across-rail hole spacing and the flange's shape were not measured (the mesh scales them, to 69.6 mm
  and 141.6 × 129.6 mm), nor where the real base's mass sits. The arm above the rotating plate was checked at one
  dimension (the shoulder height) and against the published reach, not link by link.
- **Implication:** on the real robot the arm, with every target and reach, is 22.7 mm higher than every simulation
  before today had it. That comes on top of F-125's mount, (-19, 0, +5.5) mm, for a total of (-19.0, 0, +28.2) mm
  at the shoulder. Low level poses close in are lost: level at 0.30 m out, the arm now stops 22.5 mm short of a
  point 0.28 m above its base (`tests/test_d1_ik.py`). Every checkpoint trained earlier saw the smaller base as well
  as the old mount.
- **Note, 2026-10-06 (across the rail, measured):**
  - Lukas: on the real arm the holes are 58 mm apart across the rail, as on Unitree's, not the 69.6 mm the mesh
    scaled them to.
  - `measured_base.py` now keeps Unitree's across-rail size in the flange and its lip at the mounting tabs (scale 1.0
    beyond |x| = 51.5 mm on Unitree's base, blended from 1.20 inside 48 mm). The holes are at (±64.8, ±29.0) mm and
    sit over the rails' four screws: 0.0 mm across, 0.7 mm along (`week_04` log, `d1_dock_holes.png`).
  - Side effects: the holes are drawn 5.4 × 4.5 mm ovals (their size was never measured), and four internal bolt holes
    in the blend are drawn as teardrops.
  - The cylinder, its 76 mm and the centre of mass are unchanged. [G0](#/week/4/run/20261006T005827_176584Z_verify_seed42)
    is 25/25 and identical to the digit to the run before (resting tip, box distance, Link6 to 0.74 µm), so F-127's
    policies stand. Only the flange's outline is still unmeasured.


### F-127 — On the measured robot the released policies miss by the shoulder's move (reach 4.35 cm, insertion 33/96); fine-tuned there, reach is back to 1.70 cm and insertion and the lever demo beat their old-robot scores, but continuing the mechanism policy made it worse, and the UniFP cup and box demos fail with either policy

- **Status:** provisional (simulation; one seed per fine-tune; no control continued on the old robot)
- **Week:** 4
- **Date:** 2026-10-06
- **Evidence:** [Week 4 log, 2026-10-05](week_04/notes.md), "Retraining the demos' policies on the measured robot",
  every run linked there. The robot is F-125's mount and F-126's base. Each released checkpoint was scored on the
  measured robot, fine-tuned there with its own lineage's recipe, and re-scored on the same frozen sets and PhysX
  demos. The rows below give frozen sets first, then PhysX demos.
  - **Reach** (`56000` → `57499`, 1,500 iterations, fingertip). Frozen sets: settled error median/p90 4.35/4.86 →
    **1.70/2.16 cm** (on its own robot 1.51/2.17); 0 falls. PhysX, cup / box with wrist roll: 5/16, 0/16 →
    2/16, 1/16 (on its own robot 12/16, 11/16).
  - **Mechanism** (`17499`, 1,000 iterations). Frozen sets: held-out test 498/512 → 481, 474, 443 at
    18000/18400/18498. PhysX, combiner at a 16 N·m door: **15/16** unretrained; 12/16 at 18000 (16/16 on its own
    robot).
  - **Lever** (`20497` → `21496`, 1,000 iterations). Frozen sets: levers 280/360 → **343/360** (304 on its own robot);
    held-out test 505/512. PhysX, lever demo (−30°, 1.2 N·m, 16 N·m door): 10/16 → **16/16** (9/16 on its own robot).
  - **Insertion** (`21200` → `21400`, 200 iterations, pinned 1e-5). Frozen sets: lateral 33 → **73**/96, detent 29 →
    **68**/96 (75, 71 on its own robot); then down to 33/32 by 21999. PhysX at 2 mm: 39/48, 40/48, 78/96 → **46/48,
    45/48, 85/96** (43, 43, 80 on its own robot).
  - **Press and wipe pilots** (400 iterations from `21200`, as 2026-09-29). Frozen sets: press panel 63/96, force
    19/48, recessed 31/72; wipe 23/36 (old-robot pilots: 49, 20, 23; 16).
- **Scope:**
  - Simulation only, one seed per fine-tune.
  - There is no control continued on the old robot, so "better than on its own robot" is not shown to be the robot
    rather than more training. The mechanism and lever results especially move in opposite directions under the same
    adaptive-rate recipe.
  - The cup and box failures were read from their summaries. In the box, the jaws arrive 1.7 cm off and the latch
    releases 7 times in 16, but the door does not move (median 0.5°, against 45° on the old robot). In the cup, 7 are
    picked and 5 spilt. They were not traced further. The cause given below, scenes tuned for the old arm, is an
    inference shared with the four failing scripted-demo tests.
- **Implication:**
  - For the measured robot use `checkpoints/..._isaaclab_measured_model_57499.pt`, `..._mech_lever_measured_model_21496.pt`
    and `..._insertion_seed3_measured_model_21400.pt`. Keep `..._mech_law_model_17499.pt` for the combiner.
  - Every result before 2026-09-30 was on a different robot. F-118 holds: the policies cannot correct a moved
    shoulder themselves, so the real arm's geometry has to be right in the model before any transfer.
  - The UniFP cup and box demos need their scenes re-placed relative to the arm before they say anything about the
    policy.
  - A mechanism fine-tune should try the insertion lineage's pinned rate (F-121) before concluding the policy cannot
    be adapted.
- **Note, 2026-10-06 (scenes re-placed, F-128):** "the scenes, not the policy" holds only in part. With the cup's
  table moved with the arm, `57499` picks 7/16 (2 before; 12 on the old robot). The box fails with its post re-placed
  too (0–6/16), with the door not moving after the latch releases: that failure is not the placement.

### F-128 — Re-placed with the arm, the scripted demos come back to their old-robot results (lying cup pick 11/11, lying combiner 8/9 and 8/8, standing combiner 16/16), and three of the four failing tests pass; the fourth's 5 mm limit is inside the tag estimator's noise, and the UniFP box demo fails for a reason that is not its placement

- **Status:** provisional (simulation; one seed per demo)
- **Week:** 4
- **Date:** 2026-10-06
- **Evidence:** [Week 4 log, 2026-10-06](week_04/notes.md), "The scripted scenes re-placed with the arm", every run
  linked there.
  - **The rule.** Every scene was set against the legacy arm. The measured arm's shoulder stands (−19.0, 0, +28.2) mm
    from it (`mount.ARM_SHIFT_FROM_LEGACY_M`; Link6 moves by exactly that under forward kinematics).
  - **Where it went.**
    - Scene objects moved by it; ones standing on the real floor, horizontally only.
    - The arm's pose grids above the mount (the cup survey, the combiner search stops) rose 22.7 mm with the base.
    - UniFP's furniture moved as far as its goal sphere, which is fixed to the base, allows: the cup's table the whole
      way, the box's post in height only.
  - **PhysX on the measured robot, re-placed** (old-robot result in brackets):
    - lying cup pick 11/11 (11/11, F-069);
    - lying combiner grip-and-pull 8/9 and 8/8 (9/9, 8/8, F-075);
    - standing combiner, mechanism `17499`, 16/16 (16/16; 15/16 before re-placing);
    - lever `21496` 16/16;
    - UniFP cup 7/16 (12/16; 2/16 before);
    - UniFP box 0/16 (11/16; 1/16 before).
  - **Tests:** 647 of 648. The survey, pivot and wall-grasp tests pass again.
  - **The tag-pose test** fails at 5.18 mm against 5.0. Over 24 placements around it the same sequence gives median
    2.9 mm, p90 6.4–6.7 mm, max 8–9 mm, the same with the box at the old arm-relative height and on the floor. The
    error follows the tag's estimated rotation (1.9° → 12.1 mm, 0.15° → 0.8 mm) over the 0.35 m from tag to box origin.
- **Scope:**
  - Simulation, one seed per demo.
  - The lying combiner's plans note the door is opened only to 44–48° where the arm stops reaching: the taller arm
    reaches a floor box less far, which no re-placement gives back.
  - The UniFP box's failure was characterised (arrival within ~2 cm, latch 0–8 of 16, door median 0.5°), not
    diagnosed. Placements within 4 mm gave 3/16 and 0/16.
- **Implication:**
  - The scripted demos are usable on the measured robot.
  - The combiner tag test's limit should follow the estimator (about 10 mm), or the close look should be improved.
    Either way it is a statement about the estimator, not the robot.
  - The UniFP box with the position-only policy needs its own investigation. The mechanism controller opens the same
    box 16/16 and is the one to demonstrate with.

### F-129 — On the measured robot the UniFP box demo fails at the lever, not the door: the turn stalls short of the latch and the jaws slide off the bar; servoing the wrist's pitch, with goal correction, a 55° turn and the lever eased only to 30°, opens 12 of 32 doors against 1 as it was and 20 on the old robot

- **Status:** provisional (simulation; two seeds of 16)
- **Week:** 4
- **Date:** 2026-10-06
- **Evidence:** [Week 4 log, 2026-10-06](week_04/notes.md), "The UniFP box demo's door pull on the measured robot",
  35 runs linked there.
  - **The old robot, reproduced** at `9b403c6` with the legacy mount and `56000`: 11/16 and 9/16 at seeds 1 and 2.
    Seed 1 matches the recorded run to its latch count and median door.
  - **The measured robot,** `57499` with the roll servoed: 0/16 and 1/16.
  - **Where it fails.** In the turn the lever stalls at a median 32.5° (latch at 45°), the hand's pitch swings 54°
    against 25°, and the jaws drift 3.1 cm sideways against 1.7. Held there, the grip slides along the bar and the
    lever springs back.
  - **The recipe** `--turn_deg 55 --goal_correction --wrist_joints 56 --ease_deg 30`: 9/16 and 3/16, no falls.
    - The pitch servo with correction and a 60° turn releases the latch 15 of 16.
    - Easing the lever only to 30° instead of 0° keeps the bar in the jaws: 3 → 9 at seed 1.
  - **Did not help:**
    - a slower turn;
    - the box moved 19 mm in;
    - a higher post;
    - the earlier fine-tune checkpoints;
    - keeping the goal correction through the lever phases (worse).
- **Scope:**
  - Simulation, two seeds.
  - The recipe is a different controller from the `--wrist` baseline: the pitch servoed too, and the goal corrected.
    It is not made the default.
  - At seed 2 the lever still stalls at 30–44° in about half the attempts. Nothing tried in the script moves it.
- **Implication:**
  - The UniFP box demo can be shown on the measured robot only with the recipe, at about half the old robot's rate.
  - Closing the rest needs the policy to push the lever harder: a fine-tune for it, or the force channel the reach
    policy was trained with (±8 N, F-080), which the demo does not drive.
  - The mechanism controller (16/16 on the same box) is the demo for opening it.

### F-130 — On a rope, the hook task's 60 N pull policy holds a settled pull steadily, but snatches the rope about three times every time it is tied on, pulls 7–10 N over its command, and falls over backward in 104 of 108 releases

- **Status:** provisional (simulation; one checkpoint on a rope-task evaluation set frozen the same day, 108 episodes)
- **Week:** 4
- **Date:** 2026-10-07
- **Evidence:** [Week 4 log, 2026-10-07](week_04/notes.md), "A steady pull on a rope, and the rope letting go".
  - **The task** (`--task rope`, `unifp_train/rope*.py`, `tests/test_rope.py`): the claw holds a rope at the jaw
    centre for the whole episode. The rope is tension-only along its own line (nothing across it: a 1 m rope at 40 N
    holds the tool sideways at 40 N/m, against the hook task's ring at 2,000), stepped at 200 Hz. Its anchor can give
    way (snap, slip to a residual, fade), and a task layer may notice and stop pulling.
  - **The frozen set** (`rope_cfg.EVAL_*`): 9 grips × 1,000 / 4,000 N/m × 3 releases (snap noticed after 0.3 s, snap
    never noticed, slip to 20% noticed), a 1 m rope pulled straight back at 20 / 35 / 50 N for 4 s each, released at
    the end of the 50 N hold. Two scoring changes, both made before any trained policy was scored (log): the whole
    hold is scored as well as its last 2 s, and a *tug* is a tension peak standing 25% of the command (≥ 3 N) above
    the troughs either side.
  - **Hook v4 `model_13000`** ([run](#/week/4/run/20261007T062950_rope_eval_seed1_rope_eval_v4_13000)), which on the
    measured robot still holds 60 N on a ring ([run](#/week/4/run/20261007T060919_hook_eval_seed1_ring_v4_13000_measured)):
    - holds 108 / 108 / 108 at 20 / 35 / 50 N, at **29.9 / 43.4 / 56.9 N**;
    - settled, the tension's std is 0.6 / 0.2 / 0.2 N (median, 200 Hz);
    - but **tugs in 107 of 108 first holds** (321 tugs; peak 36.9 N on a 20 N command), in 17 at 35 N and 5 at 50 N.
      The snatches come as the rope is tied on and the command ramps;
    - Joint1–3 at their torque limits at every level; the arm 8–12° off the rope's line; the front feet carrying
      57–73% of the vertical load, rising with force;
    - **falls in 36 / 33 / 35 of 36 releases** (snap noticed / unnoticed / slip). The base runs back at up to 0.8 m/s,
      the rear sinks 10 cm, and the robot pitches nose-up over 1 rad and goes over on its back 0.7 s after the snap.
  - **Zero actions** ([run](#/week/4/run/20261007T063225_rope_eval_seed1_rope_eval_zero)): holds none (0.3–0.4 N, the
    arm resting on the rope), so nothing to recover from; 0 falls.
- **Scope:**
  - Simulation only. The rope is a spring with no mass, sag or wave along it, at 1,000–4,000 N/m where a real 1 m
    rope is ~10⁵ N/m. The anchor is a model, and the floor is μ = 1.0.
  - One checkpoint, trained on Unitree's base, evaluated on the measured one (F-126).
  - "Tug" is a definition made here; the log has the reasoning and the first definition it replaced.
- **Implication:**
  - Steadiness and recovery have to be trained for: the pull the hook task taught is steady once settled, but snatches
    while loading up, and the robot cannot survive the rope letting go.
  - At 50 N the feet push about 50 N back against the ground. A snap leaves that push unopposed, so the base
    accelerates backward at ~2.7 m/s² and rotates nose-up until the legs react. The policy has to react within a few
    hundred milliseconds or fall.

### F-131 — Trained against the rope letting go, the pull policy learns not to fall within 400 iterations; but if the first update is allowed to throw away its straight-arm posture, it gets there by no longer pulling

- **Status:** provisional (simulation; one seed per variant; the development rope set)
- **Week:** 4
- **Date:** 2026-10-07
- **Evidence:** [Week 4 log, 2026-10-07](week_04/notes.md), v1 and v2.
  - **v1** ([run](#/week/4/run/20261007T063306_train_seed1_rope_v1)): hook v4 `13000` resumed on the rope task, half the
    rope episodes releasing, rsl-rl's adaptive learning rate.
    - The first update's KL was 0.93: the rate starts at 1e-3 on a resume and is cut to its floor within the update,
      after the first minibatches have already moved the policy. The elbow (Joint3) went from −0.64 to −1.03 rad to
      +0.83.
    - At 400 iterations ([run](#/week/4/run/20261007T065244_rope_eval_seed1_rope_eval_v1_13400)) **0 of 108 releases fell** (baseline 104),
      but the tension capped at 24–25 N whatever was commanded (35 N held 13 of 108, 50 N none). The arm sat 35° off
      the rope with the base 15° nose-up. At 800 ([run](#/week/4/run/20261007T071226_rope_eval_seed1_rope_eval_v1_13800)) the cap was ~19 N.
    - The tension-ripple term (−1.0 on the total variation within each 20 ms) was the largest penalty in training,
      −1.3 to −1.4 per episode-second. Mostly it was exploration noise, which an arm drawn along the rope turns into
      tension and a bent one does not.
  - **v2** ([run](#/week/4/run/20261007T071643_train_seed1_rope_v2)): the same start with the rate scaled by 0.1
    (first-update KL 0.012), the ripple term at −0.2, and a term paying for the arm lying along the rope. At 200
    iterations ([run](#/week/4/run/20261007T072651_rope_eval_seed1_rope_eval_v2_13200)) it pulled 20.5 / 32.0 / 44.6 N for 20 / 35 / 50 N
    with the arm 4–7° off the rope.
- **Scope:** one seed each, and v1 and v2 differ in several things at once, so the share each change had is not
  separated (the matched controls on v3 are in F-133). Simulation only.
- **Implication:** on a resume to a new task, protect what the starting policy already does (a scaled or pinned rate,
  as F-121 found for insertion). A recovery objective added to a force objective can otherwise be met by giving up the
  force: an easy way out that looks like success on a falls count alone. Score both together.

### F-132 — The rope policy's deterministic over-pull is learnt compensation for its own exploration noise: with the training noise on it meets the command to 1–4 N, without it pulls up to 27% over

- **Status:** provisional (simulation; measured on one checkpoint, consistent with three others)
- **Week:** 4
- **Date:** 2026-10-07
- **Evidence:** [Week 4 log, 2026-10-07](week_04/notes.md), "Why it over-pulls".
  - v2 `13600` on the development set, deterministic ([run](#/week/4/run/20261007T074726_rope_eval_seed1_rope_eval_v2_13600)): **25.6 / 43.0 /
    63.6 N** for 20 / 35 / 50 N. With actions sampled at its own noise, as in training
    ([run](#/week/4/run/20261007T075125_rope_eval_seed1_rope_eval_v2_13600_stochastic)): **23.6 / 36.9 / 51.3 N**. Its force estimate agrees
    with the tension both ways (26.1 / 40.5 / 59.0 and 23.9 / 37.9 / 52.7 N), so it is not misreading the force.
  - In training the frontier's relative error read 0.11–0.15 while the deterministic policy was 27% over at 50 N.
  - The noise had grown back by entropy, from the 0.6 set on the arm to 0.77–1.01, with the legs at 0.6–1.2. With
    the noise on, the tension's std is 5–7 N and the feet lift ~100 times in 4 s.
  - Hook v4 `13000`, trained with an arm noise of ~1.6, over-pulls by 50% at 20 N, which fits.
  - Cutting the noise (arm 0.3, legs 0.4, entropy coefficient 0.002; v3) brings the deterministic pull to within 2–5 N
    of 20 / 35 / 50 N ([`14000`](#/week/4/run/20261007T085326_rope_eval_seed1_rope_eval_v3_14000)).
- **Scope:** the mechanism is inferred from the stochastic/deterministic contrast on one checkpoint. v3 also added a
  term against wobble, so its accuracy is not the noise cut's alone.
- **Implication:** for a force-commanded policy deployed deterministically, the training noise is part of the force
  calibration. Train force-tracking fine-tunes at the noise they will be judged without (0.3–0.4 here), or the
  deployed policy will push harder than told, by an amount that drifts as entropy grows the noise.

### F-133 — The whole-body policy can be trained to hold a steady pull on a rope and to survive the rope letting go: in three seeds it holds 20–50 N with no repeated tugging and falls in none of 972 releases, by crouching and leaning into the rope with the arm drawn straight along it; the legs make about two-thirds of the force

- **Status:** provisional (simulation only; three seeds of one recipe; the held-out set run once, on one checkpoint)
- **Week:** 4
- **Date:** 2026-10-07
- **Evidence:** [Week 4 log, 2026-10-07](week_04/notes.md), v3 onward; [figure](week_04/figures/rope_pull.png).
  - **The recipe** (v3; `unifp_train/README.md`, "The rope-pull task"): hook v4 `13000` continued on `--task rope`, the
    learning rate scaled 0.1, the exploration noise cut (arm 0.3, legs 0.4, entropy 0.002), and three terms added to
    the hook task's force objective: the arm along the rope, the force error's wobble about its own 0.3 s mean, and a
    broad third width on the force term (F-131 and F-132 for why).
  - **Seed 1, the chosen checkpoint `14400`** (picked by a rule written before the last checkpoints were scored):
    - on the development set ([run](#/week/4/run/20261007T091308_rope_eval_seed1_rope_eval_v3_14400)): 20 / 35 / 50 N held in 108 / 108 / 108,
      at 26.1 / 40.4 / 55.2 N, tension std 0.49 / 0.06 / 0.02 N, no tug at 35 or 50 N, tie-on tugs in 39 of 108 first
      holds (hook v4: 107 of 108, about three each); **0 of 108 releases fallen** (hook v4: 104);
    - on the **held-out test set, run once** ([run](#/week/4/run/20261007T095046_rope_eval_seed1_rope_eval_v3_14400_test)): other grips, ropes
      of 0.5 and 1.5 m at 2,000 and 6,000 N/m, pulls tilted ±0.15 rad, 15 / 30 / 45 N, and three releases never trained
      or tuned on. All 288 holds held at 22.3 / 34.6 / 50.4 N, std 0.28 / 0.09 / 0.03 N, no tug at 45 N, 18 at 30 N.
      **0 of 96 releases fallen**, the base moving 7–8 cm and settling in 0.7–1.2 s, no foot lifted.
  - **Seeds 2 and 3, the same recipe:** 0 falls at every checkpoint scored (400, 600 and 1,000 iterations); at 600
    iterations all 324 holds held in all three seeds, at 1,000 iterations 322–324; no tug at 50 N. Across the three seeds' nine checkpoints at 400 iterations or more,
    **0 of 972 releases fallen**. Seed 3 is slower to the posture (20 / 31 / 41 N at 400 iterations).
  - **How:** crouched 4–9 cm and pitched about 15° nose-down into the rope, the arm 3–7° off its line, Joint2 and
    Joint3 at their torque limits. When the rope goes, the release spends the lean bringing the body back to level
    (pitch to −0.4° / −1.1°, two seeds) instead of past it (hook v4 pulled level and went to −56°).
  - **The legs' share:** with its leg actions zeroed ([run](#/week/4/run/20261007T095241_rope_eval_seed1_rope_eval_v3_14400_legsfrozen)) the
    same policy pulls 17–20 N whatever is asked, against 55 N with them.
  - **Further:** 70 N held as steadily (74.9 N, std 0.12 N) and survived when it snaps, 108 / 108
    ([run](#/week/4/run/20261007T100020_rope_eval_seed1_rope_eval_v3_14400_to70)).
- **Scope:**
  - **Not perfect.**
    - It pulls 5–7 N over the command.
    - It still snatches once as the rope comes taut on a 25 N/s ramp (7 of 108 first holds on a 10 N/s ramp,
      [run](#/week/4/run/20261007T095827_rope_eval_seed1_rope_eval_v3_14400_ramp10)).
    - At 20,000 N/m a minority of holds oscillate (20 of 108 at 35–50 N).
    - On μ = 0.5, unnoticed snaps take it below the strict 0.15 m line in 22 of 36, upright, none tipped
      ([run](#/week/4/run/20261007T095631_rope_eval_seed1_rope_eval_v3_14400_mu05)).
    - Training on past 1,400 iterations, seed 1's last checkpoint showed the same crouch in 8 of 36 unnoticed snaps
      on μ = 1.0.
  - **The model.** The rope is a massless spring at 500–6,000 N/m in training, against a real rope's ~10⁵, and the
    anchor is a model. The force is the plant's, not a measurement. "Tug" is a definition made here.
  - **The arm.** It runs two joints at their published torque limits for the whole pull, which a real D1 may not
    tolerate (F-103).
  - **The development set** chose the checkpoint and guided three recipe versions; the test set was run once.
  - Simulation only. Nothing here has been on hardware.
- **Implication:**
  - Lukas's question has a yes, in simulation. A whole-body policy can hold a steady rope pull with the body making
    the force and survive the resistance vanishing. The behaviour that makes both possible is one posture, crouched
    and leaning in with the arm in line with the rope, and three seeds find it.
  - Before hardware:
    - measure the D1 servos' stall and continuous torque, and retrain with a heavier arm margin (queued overnight);
    - put a compliant element in the rope;
    - have the task layer ramp at ≤ 10 N/s and drop its command when the force estimate collapses, since unnoticed
      releases are the weak case.

### F-134 — In the rope-pull recipe, the releases teach the recovery, the cut in exploration noise removes the repeated tugging, the wobble term halves the snatch as the rope comes taut, and the term paying for a taut arm adds little once the starting posture is kept

- **Status:** provisional (simulation; one seed per control, 600 iterations, development set)
- **Week:** 4
- **Date:** 2026-10-07
- **Evidence:** [Week 4 log, 2026-10-07](week_04/notes.md), "Controls". Each control is the v3 recipe (F-133) minus one
  thing, seed 1, scored at `13599` against v3's `13600`. 108 releases each.
  - **No releases in training** ([run](#/week/4/run/20261007T115601_rope_eval_seed1_rope_eval_rope_v3_ctl_norelease_13599)): the pull is as
    steady and accurate (24.3 / 36.2 / 50.7 N, no tug at 35 or 50 N), but **46 of 108 releases fall** (snaps 24 and
    22 of 36, slips 0), against 0 for v3 and 104 for the hook v4 start.
  - **No taut-arm term** ([run](#/week/4/run/20261007T122545_rope_eval_seed1_rope_eval_rope_v3_ctl_notaut_13599)): 0 falls, 322 of 324 held,
    the arm 5–9° off the rope against v3's 2–6°.
  - **No wobble term** ([run](#/week/4/run/20261007T125525_rope_eval_seed1_rope_eval_rope_v3_ctl_nowobble_13599)): 0 falls, but tugs as the
    rope is tied on in **108 of 108** first holds (v3: 61), 6 of 108 50 N holds missed. At 35 and 50 N one tug in 216
    holds, where v2 at the same iteration ([run](#/week/4/run/20261007T074726_rope_eval_seed1_rope_eval_v2_13600)) had repeated tugging at 35 N
    in 38 of 108. That control differs from v2 mainly in the exploration noise (0.3 / 0.4 against ~1).
- **Scope:** one seed per control, so differences of a few degrees or a few episodes are within seed variation; the
  large ones (46 against 0 falls, 108 against 61 snatches, 38 against 1 tug trains) are not. v2 differs from the
  no-wobble control in its noise and in being stopped early, so the noise attribution rests on that pair.
- **Implication:** for this task the levers are the disturbance in training (releases), the action noise the policy
  is trained at (for steadiness, and for accuracy, F-132), and protecting the starting posture on a resume. A reward
  term for the posture itself was not needed. The tie-on snatch is the part training only halved; the command's ramp
  (10 N/s) does more for it.

### F-135 — Three variants of the rope recipe aimed at hardware do not improve it: a heavier price on arm torque does not unload the arm's shoulder and elbow, it only lowers the pull; training on ropes to 20,000 N/m fixes the oscillation there but costs force and brings back sideways falls; a corrected curriculum changes nothing consistent

- **Status:** provisional (simulation; one seed each, 1,000 iterations from hook v4 `13000`)
- **Week:** 4
- **Date:** 2026-10-08
- **Evidence:** [Week 4 log, 2026-10-07](week_04/notes.md), "Overnight". Each is the v3 recipe (F-133) with one change.
  - **Arm margin ×3** (`arm_torque_margin` −12; [run](#/week/4/run/20261007T134620_rope_eval_seed1_rope_eval_rope_v3_margin12_13999)):
    Joint2 and Joint3 at 100% of their limits at every level, as in v3. The pull falls to 21.4 / 30.6 / 41.3 N for
    20 / 35 / 50 N, 242 of 324 held; 0 falls.
  - **Ropes of 500–20,000 N/m in training** ([run](#/week/4/run/20261007T143650_rope_eval_seed1_rope_eval_rope_v3_stiff20k_13999_k10k20k)): on
    20,000 N/m the worst-case std drops to 0.9 / 4.8 N at 35 / 50 N (v3: 6.4 / 19.7). On the development set 291 of
    324 held (v3: 323), and unnoticed snaps from low grips now tip sideways (roll 43–46°) in 1 of 108 there and 2 of 108
    on the stiff ropes.
  - **Windowed curriculum** ([run](#/week/4/run/20261007T152706_rope_eval_seed1_rope_eval_rope_v3_window_13999)): at 600 iterations as v3 (0
    falls, mean error 1.3 N); at 1,000 fewer tie-on tugs (24 of 108) but 301 of 324 held and one 35° roll.
- **Scope:** one seed and 1,000 iterations each; longer training might reconcile the stiff-rope trade. Joint torque is
  the simulator's PD clipped at the published limits, and the real servos' behaviour at the limit is unmeasured.
- **Implication:**
  - The saturated shoulder and elbow are a property of holding tens of newtons on a rope with this arm, not of the
    reward. With the rope's line 3–6° off the arm's, ~50 N puts ~2 N·m on Joint2 (limit 3.3) before the arm's own
    weight, and Joint3's limit is 1.7. Whether that is acceptable is a bench measurement (stall and continuous torque,
    heating, F-028's dropped arm), not a training question.
  - A stiff tether is better met with compliance in the line than by training for it.

### F-136 — A short polish of the rope policy at half the exploration noise brings its pull onto the command, held-out included, with no loss of recovery, as F-132 predicted

- **Status:** provisional (simulation; one seed, one polish run; the held-out set run once on the polished checkpoint)
- **Week:** 4
- **Date:** 2026-10-08
- **Evidence:** [Week 4 log, 2026-10-07](week_04/notes.md), "A low-noise polish". F-133's `14400` continued 400 iterations
  with the arm's noise at 0.15 and the legs' at 0.2 (from 0.3 / 0.4), otherwise the same recipe
  ([run](#/week/4/run/20261007T152825_train_seed1_rope_v3_polish)).
  - **Development set, `14600`** ([run](#/week/4/run/20261007T153901_rope_eval_seed1_rope_eval_rope_v3_polish_14600)): 22.0 / 34.7 / 50.1 N for
    20 / 35 / 50 N (`14400`: 26.1 / 40.4 / 55.2), tie-on tugs in 19 of 108 first holds (39), 0 of 108 releases fallen,
    324 of 324 held. First by the selection rule on every count.
  - **Held-out set, run once on `14600`** ([run](#/week/4/run/20261007T155042_rope_eval_seed1_rope_eval_rope_v3_polish_14600_test)): 17.4 / 30.0 /
    44.3 N for 15 / 30 / 45 N (`14400`: 22.3 / 34.6 / 50.4), std 0.04 / 0.11 / 0.07 N, no tug at 30 or 45 N, tie-on
    tugs in 75 of 96 first holds (61) but smaller (peak 19.7 N on 15 N, against 24.1), **0 of 96 releases fallen**.
  - At `14799` the pull drifts back up (24.6 / 39.6 / 54.6 N) and one unnoticed snap crouches to 0.1496 m (upright).
- **Scope:** one seed; the polish was chosen and stopped on the development set. The held-out set has now been used once
  for each of two checkpoints, never to choose between them. The arm's Joint2 and Joint3 are still at their limits.
- **Implication:** the deliverable rope policy is `checkpoints/unifp_go2d1_rope_v3_polish_model_14600.pt`. For force
  tracking judged deterministically, train the last stretch at the noise it will run without. A short low-noise polish
  removes most of the bias F-132 found, and continuing it lets the bias creep back.

### F-137 — Unitree's D1 CAD agrees with its D1-550 URDF and with the base Lukas measured, not with `d1.urdf`, whose arm is the older `d1_description`'s: the forearm is 6.8 mm short, and the jaw centre sits 6.0 mm from where the CAD's lengths put it (median of random poses; max 8.6)

- **Status:** provisional (Unitree's CAD and URDFs compared with each other; the real arm's links not measured)
- **Week:** 4
- **Date:** 2026-10-08
- **Evidence:** [Week 4 log, 2026-10-08](week_04/notes.md), "Unitree's D1 CAD against the URDF";
  `python3 d1_arm/compare_cad.py <D1-550 .stp> <d1_550_description.urdf>`, the files listed with their hashes in
  `third_party/unitree_d1_cad/NOTICE.md` (from support.unitree.com's D1 page, read through the API it loads from).
  - Unitree links two URDFs (`d1_description`, from which `d1.urdf` descends, and `d1_550_description`) and two STEP
    models, which are the same arm. The page names the arm the D1-550.
  - From the CAD, joint axes as lines of coaxial cylindrical faces: Joint2 130.80 mm above the base's bottom face;
    upper arm 270.00; elbow offset 41.32; forearm (Joint3 → Joint5 along the roll line) 204.67; Joint5 → fingertips
    206.38. The D1-550 URDF matches every arm length to 0.1 mm (the shoulder +0.8).
  - `d1.urdf`: shoulder +1.50 (the measured 76 mm base under `d1_description`'s 56.3), upper arm −0.70, elbow offset
    +0.68, **forearm −6.79**, Joint5 → fingertips +1.72.
  - The CAD's base: holes 130 × 58 mm, housing ~126 mm across, the rotating plate 74.0–75.5 mm above the bottom. That is
    what Lukas measured (F-126: 130 × 58, 126, 76).
  - The jaw centre (`Link6 + (0, 0, 0.1051)`), `d1.urdf` against a copy with the CAD's lengths in its own frames, over
    20,000 random configurations within the joint limits: median 6.0 mm apart, p90 7.3, max 8.6. At q = 0, 5.1 mm short
    along the forearm and 1.5 mm high.
  - The shoulder and elbow servos have identical coaxial cylinders in the CAD, and the D1-550 URDF gives the elbow
    3.33 N·m; Unitree's spec table on the same page gives it 1.7, as `d1.urdf` does.
- **Scope:** the CAD is Unitree's simplified model. Only its base has been checked against the real arm, and agrees.
  No link of the real arm was measured here. Masses are not compared: neither URDF carries the motors, and the STEP has
  no materials. The elbow torque is unresolved: an identical housing does not prove an identical gearbox.
- **Implication:**
  - Every result that rests on `d1.urdf`'s arm carries a model error of about 6 mm at the controlled point: the
    simulation, the trained policies, the scripted demos and `d1_ik.py`'s IK on the real arm.
  - Before correcting it, measure the real forearm between the Joint3 and Joint5 axes (CAD 204.7 mm, `d1.urdf` 197.9).
    Then patch `d1.urdf`'s five origins, or adopt `d1_550_description`, and retrain as after F-126 (F-127).
  - Weigh the elbow's torque on the bench as well: if it is 3.3 N·m, the rope pull's saturated elbow (F-135) is half
    the constraint the simulation says.
- **Followed by:** F-139 (2026-10-09): Lukas asked for `d1.urdf` to be patched to the CAD without the caliper reading;
  done, and the policies re-scored.
- **Followed by:** F-142 (2026-10-18): the shoulder and elbow are the same part, a Fashion Star RX18-U100H-M; the
  elbow torque is no longer a housing-versus-gearbox question.

### F-138 — From Lukas's bracket CAD, the wrist camera looks 30° down from the gripper's approach axis, its colour optical frame at (−49.1, 32.8, 61.3) mm in Link6: 13.3 mm and 30° from the mount aligned by eye; with it the D1 cannot point the camera within ~19° of level from the tag search's height

- **Status:** provisional (the bracket's design, registered to Unitree's CAD and Intel's case; not measured on the arm)
- **Week:** 4
- **Date:** 2026-10-09
- **Evidence:** [Week 4 log, 2026-10-09](week_04/notes.md), "The wrist camera's bracket from its CAD";
  `python -m demos.cup.pick_demo.bracket <mount_only.stl> <d435i_with_mount.stl> <d435i_with_mount_and_arm.stl> --write`
  on Lukas's Fusion 360 export (checksums in `demos/cup/pick_demo/assets/bracket/registration.json`).
  - Link6 in the CAD: the wrist roll line from 6 rings of the wrist flange (centres within 0.11 mm), the roll fixed by
    the jaw axis, the origin along the roll line by fitting `d1.urdf`'s Link6 mesh to the CAD's gripper (median 1.38 mm,
    a shallow minimum: +3–4% cost at ±1 mm).
  - The camera: the D435i case's front normal is 30.003° below the approach axis, yaw −0.004°. The bracket's plate lies
    on the case's back and has two 3.0 mm holes 45.000 mm apart, the D435's M3 mounting holes (Intel's: 45.000 mm).
    Intel's case placed there sits on the export's case back to 0.14 mm (median).
  - The mount, colour optical frame in Link6: xyz (−49.05, 32.81, 61.33) mm, rpy (−30, 0, −90)°; the optical axis is
    (0.500, 0, 0.866). The saved default it replaced (2026-09-17, aligned by eye): (−61, 34, 67) mm, (0, 0, −90)°.
  - Clearances: the case is 3.0 mm from `d1.urdf`'s Link6 mesh and 2.8 mm from the CAD's own hand; the bracket is 1.6 mm
    clear of the Link6 mesh (it rests on the CAD's D1-550 hand, which that mesh is not). The case and the bracket stay
    out of the colour view; the closed fingertips enter its right edge (u ≈ 580 of 640) and the jaw centre sits just
    outside (u ≈ 642).
  - The combiner's tag search (`press.plan_search`): looking 10° down at the tag's height, as with the camera along the
    gripper, no stop solves; 19° or more down from 0.32 m above the mount, they do. Re-aimed 20° down, the combiner's
    scripted sequence tests pass (20 of 21 at once, the close-look roll test re-pinned: it saves at most 0.18 rad now,
    against > 0.5). The cup pick's 78 geometry tests pass with the new mount unchanged.
  - [Render](#/week/4/run/20261009T065528_402141Z) (`./demos/cup/run_camera_body_view.sh`): the bracket drawn on Link6
    and Intel's case at the mount, as in the CAD.
- **Scope:** the design, not the part. The printed bracket's tolerances, how the camera seats in it and where the
  slide is clamped are not measured; nor is Link6's origin along the roll axis on the real arm (`d1.urdf`'s hand is not
  the D1-550's, F-137). A hand-eye calibration on the real arm is what would measure the extrinsics. The D435i model in
  the export is not Intel's (0.8 mm longer, 1 mm shallower) and was used only for orientation and handedness.
- **Implication:**
  - Every simulated wrist-camera run from 2026-10-09 uses this mount (`wrist_mount.json`, loaded by default); runs
    before it used the one aligned by eye, which pointed the camera along the gripper.
  - With the camera pitched 30° down, looks have to be planned at least ~20° down from the D1's reach at a lying dog's
    height; a level look at a target is out of reach.
  - Check the printed bracket on the bench with a hand-eye calibration before trusting millimetres from the camera.
- **Followed by:** Lukas, 2026-10-19: the bracket is printed and fitted to the real arm, the RealSense on top of the
  pincer. Its seating is still unmeasured; the hand-eye check above stands. Every demo now draws it (Week 6 log).

### F-139 — Given Unitree's CAD lengths, the D1's controlled point moves 6 mm and the policies in use barely notice: the frozen sets move by at most 4 episodes, the closed-loop reach not at all; one scripted demo, already on the edge of a tolerance, falls off it

- **Status:** provisional (simulation; the CAD's lengths, not the real arm's; one seed per policy)
- **Week:** 4
- **Date:** 2026-10-09
- **Evidence:** [Week 4 log, 2026-10-09](week_04/notes.md), "The D1 given Unitree's CAD lengths".
  - `d1_arm/d1.urdf`: five joint origins (Joint2 z, Joint3 y and z, Joint4 y, Joint5 z, Joint6 x) put every joint axis at
    q = 0 where F-137's CAD has it. `compare_cad.py` rows match to 0.01 mm. The forearm's roll line is now 0.50 mm off
    Joint1's axis, where it was 1.40. The closed fingertips are 206.38 mm past Joint5. `d1_description`'s links, meshes,
    inertials, limits and hand are kept.
  - The jaw centre moves a median 6.0 mm over 20,000 random configurations within the limits (p90 7.4, max 8.8). At
    q = 0 it moves (+5.07, +0.90, −1.48) mm. G0 passes 25/25, with Link6 and the tool point on URDF kinematics to 0.64
    and 0.72 µm. The resting tip moved 5.4 mm and was re-measured. All 674 unit tests pass after three
    forward-kinematics pins were updated.
  - The checkpoints in use, on their frozen sets, on the old arm and the corrected one the same day. The old-arm runs
    reproduce F-127 and F-136 exactly, so the evaluations are deterministic and the differences are the arm's:
    - reach `57499`: 1.70 / 2.16 → 1.71 / 2.16 cm;
    - mechanism `17499`: 498 → 497 of 512;
    - lever `21496`: 343 → 344 of 360 levers, 505 → 502 of 512 on the test;
    - insertion `21400`: lateral 73 → 77, detent 68 → 67, with detent damage 4 → 8;
    - rope `14600`: held out 17.4 / 30.0 / 44.3 → 16.8 / 29.6 / 43.9 N, no fall in either;
    - press pilot: 63 / 19 / 31 → 64 / 21 / 30; wipe pilot: 23 → 22 of 36.
  - The CPU models: the target box is 98.71% reachable, clear and holdable (98.99%). The arm-alone force capacities
    agree to 1%. The best robust standing pull is 45.0 N against 49.2.
  - The PhysX demos, on both arms the same day (checkpoints in use, 2026-10-06's commands): UniFP cup 7 → 10/16, UniFP
    box 0 → 1/16, its scripted fix 12 → 11/32, standing combiner 16 → 16/16, lever demo 16 → 16/16, insertion 46/45/88 →
    47/46/88, lying cup pick 11 → 11/11. The walking self-test does not change (forward 94.9 → 95.6%, no falls).
  - **One regression: the lying combiner, 16/17 → 10/17.** Every failure is the −40° search stop never arriving. The arm
    settles there 0.031 rad off on Joint4, where the old arm settled 0.0291, and the sequence's arrival tolerance is
    0.03 rad. Every attempt that saw the tag earlier opened its door. Self-collisions off fail identically.
- **Scope:** the lengths are Unitree's simplified CAD, not the real arm's, and the hand is still `d1_description`'s, whose
  jaws are 5 mm shorter than the D1-550 URDF's. Its tips are placed on the CAD's. The shoulder height disagrees by 1.5 mm
  with Lukas's 2026-09-30 ruler reading, and the URDF follows the CAD. One seed per policy; nothing on hardware.
- **Implication:**
  - Unlike F-126's base, this correction does not need the policies retrained to keep them working. They close the
    loop on the measured tool point, and 6 mm is inside what they absorb.
  - Insertion is the one to watch. Its detent damage doubles, which a millimetre task can show from a 6 mm model
    change.
  - The lying combiner's 0.03 rad arrival tolerance was 0.0009 rad from failing at one search stop on the old arm. Any
    small change in the arm tips it. At 0.035 rad (patched in memory, on the 3.33 N·m robot) the demo opens 17/17.
    Loosening it, for search stops or for all motions, or moving the stop, is a choice for the demo; it shares the
    tolerance with the real arm's pick.
  - Before the next hardware session, settle the shoulder height (base bottom → shoulder axis: CAD 130.8 mm, ruler
    132.3) and the forearm (Joint3 → Joint5: CAD 204.7, old model 197.9) with calipers. `d1_ik.py` now plans with the
    CAD's.

### F-140 — With the shoulder and elbow at 3.33 N·m, the policies trained against a 1.7 N·m elbow over-pull: the rope policy by ~6 N at every level, and the mechanism controller falls for the first time

- **Status:** provisional (simulation; the 3.33 N·m is Unitree's D1-550 URDF and Lukas's call, not measured; one seed per
  fine-tune, each with an old-arm control)
- **Week:** 4
- **Date:** 2026-10-09
- **Evidence:** [Week 4 log, 2026-10-09](week_04/notes.md), "Then the shoulder and elbow: 3.33 N·m".
  - The change: Joint2 3.3 → 3.33 and Joint3 1.7 → 3.33 N·m, in the URDF, `motor_model`, the UniFP interface and the
    CPU workspace model at once, on top of F-139's lengths. A test keeps the four in agreement. G0 25/25. All 675 unit
    tests pass. Two Isaac Gym fidelity tests now state the departure.
  - The checkpoints in use, unretrained, against the same lengths at 1.7 N·m:
    - rope `14600`: held out 16.8 / 29.6 / 43.9 → **21.3 / 35.6 / 51.1 N** for 15 / 30 / 45 N, development
      21.7 / 34.3 / 49.8 → 26.8 / 40.3 / 56.0; no fall in either;
    - mechanism `17499`: 497 → 491 of 512, **4 falls** (none before), all on the button's two highest levels;
    - lever `21496`: 344 → 336 levers, 502 → 492 on the test;
    - insertion `21400`: lateral 77 → 76, detent 67 → 79 with damage 8 → 2;
    - reach `57499`: 1.71 → 1.78 cm median; press panel 64 → 57; wipe 22 → 21.
  - The arm-alone static capacities rise: lift median 2.5 → 5.8 N, push down 9.1 → 12.3, best robust standing pull 45 →
    65 N. The binding joint moves from Joint3 to Joint1 and Joint2.
  - The PhysX demos, unretrained, against the same lengths at 1.7 N·m: lever demo 16/16 → **9/16 with 2 falls**, UniFP
    box's scripted fix 11/32 → 5/32, UniFP cup 10 → 6/16; standing combiner 16/16, insertion 47/46/88 → 44/44/89 and
    the lying cup pick 11/11 hold. The walking self-test does not change.
  - Each lineage was fine-tuned on this robot with its own recipe, and identically on the old arm as a control. Every
    saved checkpoint was scored on the frozen sets, the candidates in the demos:
    - **reach:** `58400` reaches 1.40 / 1.93 cm (`57499` here 1.78 / 2.19; the control at best 1.57 / 2.24). Cup demo
      10/16 against 6/16; the UniFP box's scripted fix 0/32 against 5/32. Adopted.
    - **mechanism (pinned 1e-5 rate):** `18000` opens 497 of 512 with no fall (`17499` here 491 with 4), while the
      control degrades (437–483 against 498). The standing combiner demo opens 13/16 against `17499`'s 16/16. Not
      adopted.
    - **lever:** the fine-tunes open up to 352 of 360 levers against 336 but fall 3–12 times. They lose the L-claw's
      bar in every lever-demo attempt (0/16 each, `21496` 9/16 with 2 falls). Not adopted.
    - **insertion:** every checkpoint of both runs is below `21400` (here 76 / 79 seated). F-121's decline, not the
      robot. Not adopted.
    - **rope (polish, three seeds):** the pull comes back onto the command at 35 and 50 N. But every one of six
      checkpoints sinks in 2–19 unnoticed snaps (`14600`: none), so by the rule `14600` stays. F-133's full v3 recipe,
      re-run on this robot, does better (F-141).
    - **press and wipe pilots:** 69 / 19 / 30 and 23. The old-arm repeat scored 34 / 10 / 16 against F-127's 63 / 19 /
      31 for the same run, so pilot training varies more between runs than the robots do.
- **Scope:** one seed per policy; the motor's real limit is unmeasured. An identical housing does not prove an identical
  gearbox (F-137).
- **Implication:**
  - A policy that leans on a saturating joint learns the saturation. Raise the limit and it pushes harder than it was
    told to.
  - Fine-tuning recovers reach, and the mechanism controller's frozen score; the controls show both gains are the
    robot's. But the lever demo stays at 9/16 with 2 falls, which no fine-tune repaired. A frozen set that closes the
    grasp wherever the hand stops cannot select for a claw's entry.
  - `mech_cfg.LEVER_PEAK_N` was drawn against the 1.7 N·m capacities and now understates what the arm can turn.
  - If the bench later shows 1.7 N·m, F-135's saturated elbow stands, and these policies would under-pull on the real arm.
- **Followed by:** F-142 (2026-10-18): the shoulder and elbow are the same motor, a Fashion Star RX18-U100H-M
  (datasheet: 3.14 N·m rated, 4.90 continuous), so the 3.33 N·m model stands; the firmware's limit is still unmeasured.

### F-141 — Retrained for the 3.33 N·m shoulder and elbow, the rope policy holds 30 and 45 N on the command with no fall in 192 releases, by pulling with the arm: it leans 4–5° where every earlier rope policy leaned 20–24°

- **Status:** provisional (simulation; one seed; the 3.33 N·m is not measured)
- **Week:** 4
- **Date:** 2026-10-10
- **Evidence:** [Week 4 log, 2026-10-09](week_04/notes.md), "Rope, retrained".
  - The run: F-133's v3 recipe re-run from hook v4 `13000` on the CAD-length, 3.33 N·m robot
    ([run](#/week/4/run/20261009T172054_train_seed1_rope_v3_c33)), 2,000 iterations.
  - All ten saved checkpoints were scored on the development set: no fall in any. From `14400` on, at most one
    episode of 108 with a tug.
  - The rope rule chose `14600`: 25.1 / 35.0 / 48.9 N for 20 / 35 / 50, no tug.
  - On the held-out set, run once ([run](#/week/4/run/20261009T190302_rope_eval_seed1_c33_rope_v3_test_14600)): 21.9 /
    31.9 / 45.2 N for 15 / 30 / 45 N, all 576 holds held, 0 of 192 releases fallen. Tie-on tugs in 28 / 20 / 0 first
    holds.
  - The policy it replaces, on the same robot: 21.3 / 35.6 / 51.1 N, tugs in 147 first holds at 15 N.
  - Its posture: pitch 3.6–5.4° nose-down, the front feet carrying 0.58–0.64 of the weight, Joint2 and Joint3 at their
    3.33 N·m limits. F-136's polish on the 1.7 N·m robot, on the same set: 20–24°, 0.51–0.53.
- **Scope:** one seed. No polish was tried on it: on this robot polishing brought back sinking after unnoticed snaps
  (F-140). The legs' share of the force was not measured here.
- **Implication:**
  - How the policy splits the pull between arm and legs follows the arm's torque limit. Here the cheaper solution was
    the arm, at its new limit.
  - So the elbow's real limit decides which rope policy is right for hardware. If it is 1.7 N·m, this one would demand
    twice the elbow torque the arm has, and lean too little to make up the force. Measure the elbow before a physical
    pull.
  - Use `checkpoints/unifp_go2d1_rope_cad333_model_14600.pt` on the 3.33 N·m model, and
    `unifp_go2d1_rope_v3_polish_model_14600.pt` on the 1.7 N·m one.
- **Followed by:** F-142 (2026-10-18): the shoulder and elbow are the same motor, a Fashion Star RX18-U100H-M, so the
  1.7 N·m case above is not the motor's limit; this is the rope policy for hardware unless the firmware caps the elbow.

### F-142 — The D1's shoulder and elbow are the same motor, a Fashion Star RX18-U100H-M bus servo. Its datasheet gives 3.14 N·m rated, 4.90 N·m continuous and 9.80 N·m stall at 12 V, so Unitree's 1.7 N·m for Joint3 is not the motor's limit, and the model's 3.33 N·m sits between rated and continuous

- **Status:** confirmed for the part (Lukas, from his D1, 2026-10-18); the torques are the vendor's datasheet, not
  measured on the arm, and the limit Unitree's firmware lets the servo deliver is unknown
- **Week:** 5
- **Date:** 2026-10-18
- **Evidence:** [Week 5 log, 2026-10-18](week_05/notes.md), "The shoulder and elbow motors identified".
  - Lukas: "I can confirm now that I have the d1 arm that the shoulder and elbow motors at the same RX18-U100H-M".
  - Fashion Star, *UART Bus Servo Actuator, 100KG Series*, datasheet V1.1.08
    ([PDF](https://files.seeedstudio.com/products/Fashionstar/Fashion%20Star_U100H_Datasheet_EN_V1.1.08.pdf), linked from
    [Seeed's FashionStar page](https://wiki.seeedstudio.com/fashionstar_servo/)). The model code reads R dual-shaft,
    X brushless, 18 the 63 × 34 × 47 mm case, U UART/TTL, 100 kg·cm, H 12 V, M 12-bit magnetic absolute encoder.
  - At 12 V: max stall 9.80 N·m, max continuous 4.90, rated 3.14 at 35 rpm (3.67 rad/s), no load 42 rpm. 392:1
    stainless-steel gears, 230 g, input 9.0–12.6 V, peak current 13 A.
  - The overload graph (25 °C ambient), read off by eye: about 3,000 s at 1.7 N·m, ~900 s at 3.3, ~300 s at 5.6.
  - The arm's measured speed ceiling, 1.20–1.29 rad/s on every joint (F-033), is about a third of the rated speed.
- **Scope:** identifies the motor, not the limit the arm enforces. Unitree's D1 page gives a 24 V supply and the servo
  is rated 9.0–12.6 V, so the voltage it sees and any current or power limit Unitree configures are unknown. Joint1
  and Joint4–6 were not identified. No torque was measured.
- **Implication:**
  - The caveat in F-137 and F-140 is resolved: an identical housing did mean an identical motor, so one limit belongs on
    Joint2 and Joint3. `motor_model.D1_EFFORT_LIMIT_NM` keeps 3.33 N·m, Unitree's D1-550 URDF value, a little above
    the rated 3.14 and below the continuous 4.90. Nothing is retrained.
  - Unitree's published 1.7 N·m for Joint3 is not this motor's limit. Unless the firmware caps the elbow lower, the
    1.7 N·m-elbow policies model a weaker arm than this one, and `unifp_go2d1_rope_cad333_model_14600.pt` (F-141) is
    the rope policy for hardware.
  - F-141's pull holds Joint2 and Joint3 at their 3.33 N·m limits. By the overload graph the servo sustains that for
    about 15 minutes at 25 °C, so a rope hold of seconds is well inside it.
  - The bench check F-137 and F-141 ask for narrows to one question: does the elbow deliver ~3.3 N·m on the arm, or
    does the firmware cap it? A pull to stall against a spring scale at a known lever arm answers it.

### F-143 — Lukas's real combiner box puts its handle 0.67 m up, 0.46 m above the lying dog's arm mount: from there the grip-and-pull plans at 2 of 12 default placements and the push at none. Standing, on the CPU model, the tag search, the close look and a grip-and-pull plan at all 30 of 30, opening the door 20–40° in the plan

- **Status:** provisional (CPU kinematic model, base level; the box is measured, the arm's reach is the model's)
- **Week:** 6
- **Date:** 2026-10-19
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), "The combiner demo on Lukas's real box".
  - The box, measured by Lukas: 830 × 630 × 300 mm, standing on the floor, hinged on the right; the rotary isolator
    handle's centre 160 mm below the top; the 100 mm tag (`dock_tag_id0_100mm_A4.pdf`, 125 mm cut) with its top edge
    150 mm below that; the cam lock midway up the left edge, 64 mm in. In `demos/combiner/geometry.py`.
  - Lying (base 0.085 m up, 7.3° nose-up), default placements 0.62–0.70 m: the pull plans at 2 of 12, the push at 0
    of 12. Pulled in to 0.56–0.60 m the pull plans at every bearing tried, but opens the door only 20–26°.
  - Standing (the position-only task's zero-action stance: base 0.2736 m up, 5.7 cm back), 30 seeded default
    placements: some search stop sees the tag, the close look solves, and a grip-and-pull plans at all 30, the door to
    20–40° in the plan, static ceilings 0.82–0.96 N·m at 45°; the push at 28. At 0.70–0.78 m: pull 22, push 12.
  - The grasp that plans: pitch 0 roll +1 at all 30 (first choice at 22), pitch 40 roll −1 at 6, pitch 50 at none.
- **Scope:** reachability on the CPU model only; the live runs are F-144 and F-145. Door-swing clearance past the
  dog is not modelled. Masses (a 0.6 kg door) are still the proxy's.
- **Implication:** the scripted combiner demo now stands the dog (`demos/common/resting.py` posture "standing"); the
  lying dog cannot work this box. F-071 to F-076 stay records of the earlier proxy and the lying dog.

### F-144 — Standing, the shut fingers fill the wrist image's lower right, and the 100 mm tag ran into them: the close look's box estimate came out 9.6 and 36.9 mm off, worse than the search look's 3–4 mm, and one placement's tag was hidden from every search stop. Aiming both looks so the tag sits up and to the left brought the close look to 2.2–3.5 mm and found the tag at 3 of 3 placements

- **Status:** provisional (simulation, three placements, rendered camera)
- **Week:** 6
- **Date:** 2026-10-19
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), same entry.
  - [smoke 1](#/week/6/run/20261019T012022_709892Z_combiner_seed42): close-look frames show a finger over the tag's lower-right corner; box errors 9.55 mm / 0.53° and
    36.91 mm / 5.44°; at the third placement no stop of seven found the tag (frames show the finger over it).
  - The fingers' edge in the 640 × 480 image, jaws shut, read off the frames: (448, 225) to (349, 470) px
    (`press.FINGERS_PX`). The 125 mm sheet spans ~265 px at the old 0.29 m close look.
  - [smoke 2](#/week/6/run/20261019T013204_701444Z_combiner_seed42): the close look aimed 10° right and 5° down (tag centre near (210, 185) px): 2.94 and 2.24 mm. Search
    unchanged: the hidden placement stayed hidden.
  - [smoke 3](#/week/6/run/20261019T013846_816348Z_combiner_seed42): search stops also turned 15° right and 8° down, from 0.35 m above the mount: the tag found at all three
    (−20°, +20°, −20°); close-look errors 2.76, 3.24, 3.47 mm.
  - On the CPU model with the finger wedge as a keep-out: the search sees the tag at 60 of 60 seeded placements
    (44 of 60 looking straight at the target), the close look at 60 of 60; quarter-turn camera rolls stay clear,
    45° ones clipped 5 of 19.
- **Scope:** the simulator's rendered fingers and the bracket's CAD mount. The real camera's view of the real jaws
  has not been checked.
- **Implication:** a larger tag needs the looks aimed off-centre; check one real wrist frame with the jaws shut to
  confirm where the fingers are before trusting `FINGERS_PX` on hardware.

### F-145 — Standing on held legs, the dog's base tilts 1.3–1.5° under the arm between the close look and the grip, which moves the lever 17–21 mm in the base frame the plan is in, against 1–4.5 mm of perception error: the jaws close off the bar, and in three attempts at 0.3 N·m the lever reached 13.6–37.6° and no door opened

- **Status:** provisional (simulation, three attempts, one spring)
- **Week:** 6
- **Date:** 2026-10-19
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), same entry; [smoke 3](#/week/6/run/20261019T013846_816348Z_combiner_seed42).
  - The grasp offset at the moment the jaws close, split (new in `turn_run.grasp_offsets`): perception alone
    [−0.8, 0.0, 2.4], [3.1, 1.3, 3.3], [3.0, −0.1, −0.1] mm; the lever's motion in the base frame since the close
    look [−9.1, 0.9, 14.4], [1.0, 0.6, 5.0], [−11.3, 1.1, 17.4] mm; base tilt since the look 1.26°, 0.32°, 1.47°.
    The arm itself reached its plan to 0.1 mm.
  - Outcomes: peak lever 13.6°, 37.6°, 26.5°; grip slip 157, 26, 143 mm (the jaws came off the bar in the first
    and third); latch never released; door 0.2–0.5°. All three took pitch 40 roll −1.
  - [smoke 1](#/week/6/run/20261019T012022_709892Z_combiner_seed42) and [smoke 2](#/week/6/run/20261019T013204_701444Z_combiner_seed42) failed the same way (lever 17–41°, slips 136–157 mm), their perception split then hidden in the
    single offset of ~19 mm.
  - The base stayed up: 0.273 m at the end of every attempt.
- **Scope:** the scripted IK sequence; legs held by PD at zero action with no balance controller; three
  placements; one spring (0.3 N·m). Even the attempt with a 5 mm shift stalled at 37.6°, so tilt is not the only
  limit.
- **Implication:** with the dog standing, an open-loop plan in the base frame is not enough. The next step is to
  correct the box pose for the base's measured tilt (the IMU's gravity vector, which the sequence already reads)
  at the align step before the jaws close, and for the arcs after it, or to re-look at the tag from the standoff.

### F-146 — UniFP walks the unlocked combiner door 90° open, its base carried round the hinge by velocity commands, with no fall in 116 attempts and the door at 87–105° whenever the grip holds; the friction grip on the handle holds in only about half the attempts (15 of 32)

- **Status:** provisional (simulation; UniFP plus the wrist roll servo; two seeds of 16 placements)
- **Week:** 6
- **Date:** 2026-10-19
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), "The door pull".
  - The demo (`demos/unifp/run_demo.py --task door --wrist`): Lukas's real box, unlocked, the lever never turned. The
    combiner's reach onto the bar, then 7 s in which the script swings the commanded door 0→90°. The jaw centre
    follows the handle's arc; the base follows the same rotation about the hinge
    (`BoxSite.base_pose_m`) through UniFP's own velocity command, fed forward and fed back
    (`script.base_velocity_command`): a steady 0.07 m/s back, 0.13 m/s right, 0.22 rad/s left.
  - [seed 1](#/week/6/run/20261019T024909_door_unifp_wrist_seed1_door16) and [seed 2](#/week/6/run/20261019T025447_door_unifp_wrist_seed2_door16_s2), 16 environments each: 8 and 7 of 16 at 85° or more when the hold ends (87.4–104.6°);
    0 falls. The base's peak pose error 3–11 cm (median 5.5 cm), yaw error up to 11°, final yaw 87–95°.
  - Failures (17): 4 doors under 6°, 12 lost the handle at 15–39°, one swung past 108° and back to 65°. Median
    jaw-centre miss at arrive 12 mm for the failures, 5 mm for the successes. Attempt 1 of seed 1 arrived 3 mm off,
    then during the 0.8 s grip the hand pitched up 30° and backed 24 mm off the bar, dragging the door 8° open,
    and the bar slipped out once the walk began. Several doors that lost the handle ended shut, cause not
    established (the upper arm carries no contact sensor).
  - The same seed-1 placements, one change each: [goal correction](#/week/6/run/20261019T024957_door_unifp_wrist_corr_seed1_door16) 8/16 (arrive miss 5.9 → 4.2 mm), [2 s grip, seed 1](#/week/6/run/20261019T025326_door_unifp_wrist_seed1_door16_grip_s2.0) 10/16,
    [wrist pitch servoed](#/week/6/run/20261019T025238_door_unifp_wrist_seed1_door16_wrist_joints56) 3/16, [lip claw](#/week/6/run/20261019T025131_door_unifp_wrist_claw_seed1_door16) 2/16 (arrive miss 79 mm: the lip claw without the mechanism policy's correction and
    roll). [2 s grip, seed 2](#/week/6/run/20261019T025535_door_unifp_wrist_seed2_door16_s2grip_s2.0) 7/16. Outcomes flip at the same placement between settings. [first, 4 envs](#/week/6/run/20261019T024810_door_unifp_wrist_seed1_door_first): 2/4.
- **Scope:** simulation, `--wrist` (a different controller from UniFP alone), a free 0.6 kg door with no closer,
  the proxy lever's shape, the object pose given, no perception. 16 environments per run (F-098: the count is
  a condition). Footprint clearance from the door is checked on the planned path only (≥ 0.19 m).
- **Implication:**
  - Walking while holding works with the deliverable policy as it is, through the command it was trained on: a
    co-rotating base keeps the goal fixed in the goal sphere, and nothing fell.
  - The grasp is the limit. A pinch on a horizontal bar, pulled out along the jaws, rests on friction alone, and
    the hand tips while the jaws close. A grasp that takes the pull in the fingers' bending (a hook behind the bar,
    or the lip claw with the goal correction and roll it was tuned with) is the next thing to try. Settings changes
    to the friction grip did not move the rate beyond 7–10 of 16.


### F-147 — Adding the real robot's lidar and wrist camera costs the reach policy 1.9 mm median and 3.7 mm p90 of settled error, with no falls: the trunk gains 0.42 kg and the gripper's last link 0.099 kg (+59%)

- **Status:** provisional (simulation; one checkpoint, the 50-episode validation set; two masses estimated)
- **Week:** 6
- **Date:** 2026-10-19
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), "The lidar and the wrist camera, in every demo and every
  training setup".
  - The parts (`payload.py`): MID-360, 0.265 kg, published; its mount, 0.155 kg, estimated (CAD volume x PLA); the D435i,
    0.072 kg, published; its bracket, 0.027 kg, estimated. The lidar and mount are cut out of Lukas's Go2 CAD and
    registered to `base_link` by fitting the trunk mesh (0.34 mm median residual).
  - Folded in by the weld (`payload_physics`): Go2 `base` 6.921 → 7.341 kg; D1 `Link6` 0.168 → 0.267 kg, its centre of
    mass 16.5 mm toward the camera. Box colliders on both. G0 verify passes on both robots ([physical](#/week/6/run/20261019T075759_109111Z_verify_seed42), [visual](#/week/6/run/20261019T075818_960184Z_verify_seed42)); the
    [first, failed](#/week/6/run/20261019T075646_286594Z_verify_seed42) run's mass check predated the camera and was extended.
  - `unifp_go2d1_isaaclab_cad333_model_58400.pt`, `run_unifp_train.py eval --tool_point fingertip --manifest
    results/manifests/unifp_isaaclab_validation.json`, seed 20260921: [payload physical](#/week/6/run/20261019T075856_eval_seed1_payload_physical) 1.69 / 2.33 cm median / p90; [payload visual](#/week/6/run/20261019T075915_eval_seed1_payload_visual)
    1.50 / 1.96; [no lidar drawn](#/week/6/run/20261019T080030_eval_seed1_payload_none) 1.50 / 1.96, identical to the digit. 0 falls of 50 in each.
  - On the CPU model the folded arm clears the lidar by 14 cm, and 174 of 18,511 random arm poses that clear the trunk
    (0.9%) pass through the lidar or its mount's hull.
- **Scope:** one checkpoint, simulation. The mount's and bracket's masses are solid-PLA estimates (a print with infill is
  lighter). The D435i's and lidar's colliders are boxes. The demos keep the payload visual-only.
- **Implication:** the payload is small for reaching, as the gripper's +59% might have suggested otherwise; Lukas retrains
  on the payload robot (`--payload physical`, now the default for both training launchers). Weigh the mount and bracket
  first, and replace the estimates in `payload.py`.

### F-148 — The CAD-length checkpoints were scored on an arm URDF that is not in the repository: `58400` gives 1.50 / 1.96 cm on the committed `d1.urdf`, not the recorded 1.40 / 1.93

- **Status:** confirmed (the hashes and the scores); the content of the missing URDF is not known
- **Week:** 6
- **Date:** 2026-10-19
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), same entry.
  - All 42 runs recorded in Week 4 on the CAD-length robot (2026-10-09, Victor's machine) record `d1_urdf_sha256`
    `1509b0bc…`. The committed `d1_arm/d1.urdf` (ab7752c, 2026-10-13, and the working tree) is `a848ee41…`; the previous
    commit's is `dad072b0…`. No file with `1509b0bc…` exists on Lukas's machine.
  - The same checkpoint, manifest, seed and options on the committed URDF: 1.50 / 1.96 cm ([payload visual](#/week/6/run/20261019T075915_eval_seed1_payload_visual), and [no lidar drawn](#/week/6/run/20261019T080030_eval_seed1_payload_none) without the
    lidar drawn), against the recorded 1.40 / 1.93. F-139 found these evaluations deterministic to the digit.
- **Scope:** the difference could be the URDF or other code between 054983a and now; only the URDF hash is known to
  differ. The demos' numbers on that robot (cup 10/16 etc.) may move too.
- **Implication:** before retraining, find the `1509b0bc…` URDF on Victor's machine and diff it against the committed
  one, so the new policies are trained on the arm the record describes. Until then the Week 4 numbers on the
  CAD-length robot are not reproducible from the repository.

### F-149 — Switched on with the arm folded, the reach policy unfolds and reaches all 27 goals with no fall, but at low or far goals it turns the hand down before lifting the shoulder and strikes the Go2's head (11 of 27, none from the zero pose), pitching the base up to 26° nose-down

- **Status:** provisional (simulation; one seed; standing, no forces; UniFP's PD arm, not the D1 servo)
- **Week:** 6
- **Date:** 2026-10-19
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), "Switching the reach policy on with the arm folded".
  - `demos/unifp/folded_start_probe.py`, `unifp_go2d1_isaaclab_cad333_model_58400.pt`. 27 held goals run in pairs,
    from the trained zero pose and from the folded rest pose [2.1, −90.0, 88.9, −2.3, 4.9, 2.9]°, after a 1 s
    standing hold ([run](#/week/6/run/20261019T075835_probe_seed1_folded_start)).
  - Both starts: 0 falls, 27 of 27 reached and held within 5 cm.
  - Folded against zero: time to 5 cm median 1.10 s against 0.46 s (max 1.68 against 1.18 s). Final error median
    1.07 against 1.16 cm.
  - Contact with the dog's head (`Head_lower`): 11 of 27 folded envs, none from zero. It comes 0.20–0.66 s after
    handover and lasts up to 1.04 s. Affected: every 0°-pitch goal except 0.35 m straight ahead, and all 0.55 m /
    20° goals. None at 40°.
  - Peak base pitch 8–26° against 3–13°. J1 sits at its torque limit for the whole first 2 s.
  - Without the hold, from training's spawn-drop reset ([run](#/week/6/run/20261019T075729_probe_seed1_folded_start)):
    13 of 27 hit. At the peak contact J1/J2 were still −69…−82° / +54…+78°, while J3 had turned 29–68°.
- **Scope:** simulation of UniFP's port, which drives the arm with its own PD and torque limits, faster than the
  real D1 (2.25 rad/s peak against 1.21–1.29, F-033). The timings do not transfer (F-097).
  - J1/J2 start 1–3° short of the real rest pose (URDF limits).
  - The contact magnitudes are PhysX impact values, not loads.
  - The real head's position relative to the simulated collider is not checked, and the dock and MID-360 have no
    colliders.
  - One seed, 27 goals, standing only.
- **Implication:**
  - Handing a folded arm straight to the policy is not safe to try first on hardware. The motion it produces most
    often is a wrist turn that brings the hand down onto the head.
  - Either bring the arm to the zero pose with the existing joint-space path first (the step-capped
    `approach_joints`, F-025/F-027), which is the start the policy was trained from, or limit the first goals to
    high pitch (≥ 40° was contact-free here).
  - The first option also removes the unfold's base pitch and the 0.6 s of extra tracking time.

### F-150 — The arm folds during the door pull's walk because UniFP commands only the tip's position, and the elbow and wrist are the policy's choice: alone it folds in 13 of 16 attempts, holding or not; with the wrist servo it stays in a forward line whenever the grip holds, and folds only after losing the handle

- **Status:** provisional (simulation; one seed; 16 placements per condition)
- **Week:** 6
- **Date:** 2026-10-19
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), "Why the arm folds during the door pull's walk". Seed 1 of
  F-146's door pull, with the goal (sphere coordinates) and the arm's joints traced.
  - [With `--wrist`](#/week/6/run/20261019T085418_door_unifp_wrist_seed1_door16_armtrace), 8/16:
    - The 8 successes never fold: J2 −22 to +3° through the 90° walk, 0.2–2.1 cm tracking.
    - Folding (J2 −65 to −88°, J3 ~110°, J5 at its −134.6° limit) appears only after the handle is lost: attempts
      1 and 3, and 4 transiently. The tracking error jumps to 12–22 cm as the door drifts shut while the script
      walks on.
    - The arm then stays folded, 5–10 cm off a goal (r 0.56 m, pitch 22°, yaw −12°) that the successful attempts
      track to ~1 cm extended.
  - [UniFP alone](#/week/6/run/20261019T085634_door_unifp_seed1_door16_nowrist_armtrace), 4/16:
    - J2 passes −40° in 13 of 16 attempts, including while holding, and the arm is folded at the end in 5;
    - one of those 5 is a success (door 90°, J2 −59°, J3 107°);
    - no joint is at a limit.
  - The goal: radius 0.57–0.62 m at the grip (trained 0.30–0.58), 0.50–0.58 m during the walk.
- **Scope:** one seed, 16 placements each, simulation. The proxy lever's shape, a free door, the object pose given.
  The upper arm has no contact sensor. Which configuration an attempt ends in is not predicted here, only counted.
- **Implication:**
  - **UniFP's formulation:** the goal is three numbers for a six-joint arm, and its orientation channels are
    declared and never written. The arm's configuration is therefore not commanded, it depends on history (as in
    F-149), and a disturbance can leave it in a folded branch that still tracks within a few cm.
  - **Our use:** the friction grip's losses (F-146) and a script that walks on regardless are the trigger.
  - **In the demo:** stop or re-grip when the fingers lose contact or the tracking error passes a few cm. Stand so
    the handle sits nearer the middle of the trained radius.
  - **To keep the line by design, train it:** command the hand's orientation (the unused channels, as the roll
    objective began to), or add a posture term to the reward.

### F-151 — A decoupled baseline for the door pull opens 7 of 16 doors, against UniFP's 8 with the wrist servo and 4 alone, on the same placements; its arm IK lands within 3 mm, but its walking policy trails the path by up to 0.39 m (median attempt), and the grip goes when the base lags out of the arm's reach

- **Status:** superseded
- **Superseded by:** [F-152](#f-152) (2026-10-19, same day). The 7 of 16 stands, but for an arm IK loop that re-ran a
  seeded search on every miss, up to 486 ms a command, which cannot run at 10 Hz. The baseline's loop is now bounded,
  and opens 9 of 32 on two seeds where this one opens 15. The rest of this finding (the walk, how it fails, the scope)
  stands.
- **Week:** 6
- **Date:** 2026-10-19
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), "A non-UniFP baseline for the door pull".
  - `run_demo.py --task door --controller decoupled`: the go2_omniverse walking checkpoint `model_7850` on the legs,
    with its own 235-wide observation, DCMotor legs, self-collision off and the trained foot friction. IK on the arm
    every 10 Hz command, through the D1 firmware planner.
  - The scene, placements, script, walk feedback, latch, jaws and success criterion are F-146's door pull.
  - [16 placements, seed 1](#/week/6/run/20261019T102047_door_decoupled_walk_ik_seed1_door16_decoupled): 7 doors at
    ≥ 85° at the end of the hold, 0 falls. Arrival 2.7–3.5 mm. Maximum base path error 0.27–0.72 m (median 0.39).
  - Same placements under UniFP: [with the wrist servo](#/week/6/run/20261019T085418_door_unifp_wrist_seed1_door16_armtrace)
    8/16 (path error 3–11 cm), [alone](#/week/6/run/20261019T085634_door_unifp_seed1_door16_nowrist_armtrace) 4/16.
  - [Walk-only control](#/week/6/run/20261019T103049_door_decoupled_walk_ik_no_object_free_seed1_door16_decoupled_walk_only):
    free space, arm parked, same 16 walks. Pull-phase path error 0.146 m (0.19 m with the door), roll up to 26°,
    no falls. The walk's quality is the policy's own.
  - Failures: 14 of 16 doors pass 66°. In 6 of the 9 failures the grip let go with the door past 79°, and the free
    door swung back. Through the pull the IK finds no solution near the target 11–59% of the time.
  - Bringing up the port needed four fixes, each recorded in the log:
    - the grasp nearest level, to match the script's level entry;
    - self-collision off;
    - the trained foot friction;
    - and before them, ruling out the box and plinth.
  - After them, a constant (−0.2, −0.33, 0.2) command tracks (−0.166, −0.267, 0.234) at 12.4° tilt, against the
    native environment's (−0.184, −0.355, 0.204) at 10.4°.
- **Scope:**
  - **Different plant:** each controller runs on the plant it was built for, so this is not a same-plant
    comparison. Legs: DCMotor, self-collision off and 0.8/0.6 foot friction here, against UniFP's PD, self-collision
    on and 1.0. Arm: the measured D1 here, UniFP's faster PD there (F-097).
  - **Naive stack:** an untuned decoupled stack, with a walking policy that never carried the arm. The 25% shortfall
    in sideways speed against native playback is attributed to the arm's drive model, not tested.
  - **Sample:** one seed, 16 placements, a free door, the box's pose given, simulation.
- **Implication:**
  - On this task UniFP's advantage is in coordination, not in reaching. The decoupled arm is more accurate. Its
    failures come from the legs lagging while the arm cannot ask them to wait, which a whole-body policy handles by
    construction.
  - So the baseline is a fair reference for "UniFP vs. a conventional stack". It is not evidence that the arm needs
    learning.
  - Cheap upgrades a stronger baseline would add:
    - a walking policy trained with the arm (F-008's arm-on runs);
    - a supervisor that slows or stops the walk when the IK saturates or the base path error passes a few cm;
    - a re-grip when finger contact drops.

### F-152 — The decoupled door-pull baseline's arm IK, bounded to run at 10 Hz (≤ 7 ms a command), opens 9 doors of 32 on two seeds, against 15 for the same IK with an unbounded seeded search on every miss (up to 486 ms a command); UniFP with the wrist servo opened 15 on the same two seeds

- **Status:** superseded
- **Superseded by:** [F-153](#f-153) (2026-10-20), as the decoupled baseline's number: on P2Dingo's walking policy the same
  bounded IK opens 32 of 32. What this finding measured on the 2024 walking policy (the IK's cost, the 15 against 9 of 32,
  where the two loops part) stands.
- **Week:** 6
- **Date:** 2026-10-19
- **Supersedes:** [F-151](#f-151)
- **Evidence:** [Week 6 log, 2026-10-19](week_06/notes.md), "The door pull hitches: the baseline's IK search".
  - Lukas saw the one-environment viewer stall every half second while pulling. The cause was the first version's
    IK. Every 10 Hz command that missed re-ran the scripted grip's seeded search (13 seeds x up to 300 iterations),
    which cost 486 ms on the development PC, against 0.2 ms for a warm start. The pull misses 43-49% of its commands
    because the walk lags.
  - The bounded loop: a warm start capped at 30 iterations, a pose accepted within 2 mm and 0.02 rad, else the jaw
    centre alone. Worst case 7.3 ms a command. One environment
    ([run](#/week/6/run/20261019T113038_door_decoupled_walk_ik_seed1_timing_decoupled_final_1env)): step wall time
    median 18 ms, max 36 ms, none over 50 ms. UniFP with the wrist servo
    ([run](#/week/6/run/20261019T104640_door_unifp_wrist_seed1_timing_unifp_wrist_1env)): 15 and 21 ms.
  - Doors opened:

    | arm IK loop | seed 1 | seed 2 | of 32 |
    | --- | --- | --- | --- |
    | seeded search on every miss (the first version) | [7](#/week/6/run/20261019T104918_door_decoupled_walk_ik_seed1_door16_decoupled_seededik_s1) | [8](#/week/6/run/20261019T105837_door_decoupled_walk_ik_seed2_door16_decoupled_seededik_s2) | 15 |
    | **bounded (the baseline now)** | [3](#/week/6/run/20261019T112933_door_decoupled_walk_ik_seed1_door16_decoupled_final_s1) | [6](#/week/6/run/20261019T111204_door_decoupled_walk_ik_seed2_door16_decoupled_bounded_nostandoffseed_s2) | **9** |

  - The first version's 7 reproduces F-151 attempt for attempt, and the bounded seed-1 run reproduces its
    configuration's earlier run likewise.
  - Same seeds under UniFP with the wrist servo: 8 and 7 of 16 (F-146).
  - Where they part: once an attempt, around 13.8 s as the walk starts to lag, the seeded search finds the pose on
    another branch, and the bounded loop falls back to the jaw centre (attempts 1, 3, 8, 10, 11 on seed 1).
  - Bounded versions of the search all opened 3/16 and 7/16 on seeds 1 and 2, the same attempts each time:
    - [one seed a command at 100 iterations](#/week/6/run/20261019T111651_door_decoupled_walk_ik_seed1_door16_decoupled_amortizedik_s1);
    - [the current joints and one seed at 300](#/week/6/run/20261019T112336_door_decoupled_walk_ik_seed1_door16_decoupled_rescueik_s1);
    - [the full search once, at the first oriented command](#/week/6/run/20261019T112013_door_decoupled_walk_ik_seed1_door16_decoupled_plannedik_s1);
    - [100 iterations instead of 30](#/week/6/run/20261019T111329_door_decoupled_walk_ik_seed1_door16_decoupled_bounded100_nostandoffseed_s1).

    So did a [standoff warm start](#/week/6/run/20261019T104734_door_decoupled_walk_ik_seed1_door16_decoupled_boundedik),
    with 4 and 4.
- **Scope:**
  - Two seeds, 32 placements. The cost figures are the development PC's, single-threaded numpy; none were measured
    on the Jetson Orin.
  - The rescue the unbounded search finds was located (when, which attempts) but not explained (which seed, which
    branch).
  - Wall time does not change a simulated result. The search made the viewer stall but did not alter the outcome;
    what makes it unusable is the robot's 10 Hz loop.
- **Implication:**
  - The real-time decoupled baseline opens 9 of 32 against UniFP's 15 of 32 with the wrist servo; quote that pair.
    F-151's 7 of 16 needs a planner the robot could not run in its loop.
  - Running that search asynchronously, in a planner thread beside the 10 Hz loop, is how a robot would get its
    rescues back, with a delay. That would be a stronger baseline, not built here.
  - For deployment, the controllers are cheap either way. UniFP is 0.11 ms of inference per 20 ms step on one
    development-PC core, the walking policy 0.03 ms, the bounded IK 7 ms per 100 ms command at worst. What the
    viewer showed was the simulator stalling on the first version's IK.

### F-153 — On P2Dingo's walking policy the decoupled door pull opens 32 doors of 32 on two seeds, against 9 on the 2024 walking policy and UniFP's 15 with the wrist servo; the 2024 policy, stopped from a walk faster than 0.06 m/s, turns in place and leans up to 26 deg, and P2Dingo's stops level

- **Status:** provisional (simulation; two seeds of 16; each controller on its own plant, as in F-151)
- **Week:** 6
- **Date:** 2026-10-20
- **Supersedes:** [F-152](#f-152) (as the baseline's number)
- **Evidence:** [Week 6 log, 2026-10-20](week_06/notes.md), "The combiner door demo".
  - The same door pull, placements and bounded arm IK as F-152, with `--walker p2dingo_gait`: P2Dingo's installed policy
    (`go2_p2dingo_gait` v7 `model_7000.pt`, a clocked gait after UniFP's, trained on a bare Go2). Results:
    [seed 1](#/week/6/run/20261020T051554_door_decoupled_walk_ik_seed1_door16_decoupled_p2dwalker_s1) 16/16,
    [seed 2](#/week/6/run/20261020T051945_door_decoupled_walk_ik_seed2_door16_decoupled_p2dwalker_s2) 16/16, no falls,
    doors 85.8-102.8 deg at the end of the hold. Arrival 5.0-5.1 mm; worst base path error, median attempt, 0.10-0.12 m
    (0.37-0.39 on the 2024 policy).
  - Same seeds: the 2024 policy 3 and 6 (F-152); UniFP + wrist 8 and 7 (F-146).
  - Why the 2024 policy fails, measured in a probe of the port, free space, arm parked or out:
    - stopped from a steady 0.06 m/s it stands (roll 0.7 deg);
    - from 0.08 m/s, 0.10, or a ramp down from 0.10, it keeps turning in place at ~5 deg/s and leans 10-26 deg;
    - native playback leans too (9-14 deg after 0.06 m/s, 20.6 deg tilt walking 0.15 m/s).
  - P2Dingo's, the same probe: stopped from 0.15 or 0.30 m/s, 1.0-1.7 deg of roll, and turning ~2 deg/s for a few
    seconds, which settles (as after landing from the spawn: ~15 deg over 3-4 s, then still).
- **Scope:**
  - Neither walking policy trained with the arm.
  - The legs run each policy's own conditions: DCMotor, self-collision off, 0.8/0.6 foot friction. UniFP's are not
    these (F-151).
  - Two seeds, 32 placements, a free door, the pose given.
- **Implication:**
  - The decoupled stack's weak half was its walking policy, not its arm and not the decoupling. With a policy that
    stops cleanly it beats UniFP + wrist on this task, 32 to 15. The door pull's comparison should be quoted against
    this baseline, which is now its default.
  - UniFP's coordination advantage (F-151) is not visible here.
  - The 2024 policy should not be used to walk the arm: `sim.py` still plays it back.


### F-154 — Finding the box itself with the head RealSense and the MID-360 (P2Dingo's docking method) and verifying the door with the lidar, the combiner door demo opens 31 doors of 32: the box placed within 7.4 mm and 0.34 deg, the door read to a median 0.5 deg while it swings, the lidar's open/not-open agreeing with the simulator 32 of 32

- **Status:** provisional (simulation; two seeds of 16; ideal rendering and a grid lidar)
- **Week:** 6
- **Date:** 2026-10-20
- **Evidence:** [Week 6 log, 2026-10-20](week_06/notes.md), "The combiner door demo". `demos/combiner_door/`, P2Dingo's
  walking policy (F-153). Results: [seed 1](#/week/6/run/20261020T051305_door_combiner_door_seed1_combiner_door16_s1)
  15/16, [seed 2](#/week/6/run/20261020T051658_door_combiner_door_seed2_combiner_door16_s2) 16/16, no falls. The given
  pose on the same placements: 32/32 (F-153).
  - **The look.** The RealSense on P2Dingo's head mount (`base_link` (0.3392, 0.03215, 0.0927), straight ahead), 0.15 m
    behind the door pull's stance: the tag whole in the picture at 200 of 200 sampled placements, against 149 from the
    stance itself.
    - The tag was found in 32/32 attempts.
    - The tag's centre gives the position and the MID-360's line through the door's face around it gives the heading
      (P2Dingo's `dock_wall` fit, ported).
    - The box was placed a median 2.2-2.4 mm off (max 7.4) and 0.14-0.20 deg off (max 0.34). The tag's own heading
      would have given a max 0.50 deg and 8.8 mm.
  - **The door.** Read at 10 Hz from the walk's end by the same face fit, laid at the tag's place at the last reading:
    - a reading on every frame of grip, pull and hold;
    - error against the simulator a median 0.46-0.49 deg, 90th percentile at most 2.9 deg;
    - the reading at the end of the hold called the door open (85 deg or more) or not in agreement with the simulator
      32/32;
    - the one failure, a door at 74.5 deg, was read at 74.4 deg and not verified.
  - Making it work (all runs recorded, the log has each):
    - the walk in had to be straight, slow and stopped by odometry;
    - the 2024 walking policy could not stand after it (F-153);
    - the attempt is anchored after the robot lands, since the spawn drop turned it.
- **Scope:**
  - Simulation: an ideal render of a printed tag, a 1-deg grid lidar with 3 cm range noise and no motion distortion,
    and the simulator's odometry.
  - The tag's heading at 0.37 m in an ideal render is already good, so the lidar's heading gains little here
    (0.50 → 0.34 deg max). On the robot P2Dingo measured the tag's heading ~10 deg off at a metre, which is where the
    lidar earns its place.
  - The placements are the door pull's (0.70-0.74 m, square to within ±5 deg); there is no search for a box out of view.
  - The mount and the bracket are from P2Dingo's working tree (uncommitted there; `third_party/p2dingo/NOTICE.md`).
- **Implication:**
  - The sequence the week's plan asks for runs end to end in simulation without a given pose: locate, approach,
    engage, actuate, open, verify.
  - The lidar's door reading is accurate enough to be the verify step. It could also close the loop (re-grip, or stop
    the walk when the door lags its command), which this does not do.
  - On hardware, three things this has not tested: the camera's intrinsics on the head, the face fit on real returns,
    and P2Dingo's walking policy carrying the arm.

### F-155 — With the arm starting folded in the real D1's rest pose and the handle as Lukas measured it (a 140 × 40 × 17 mm slot, 27 mm off the door), the combiner door demo opens 32 doors of 32 on two seeds, as does the door pull given the pose; the unfold, in joint space, tilts the base under 0.5 deg and touches nothing

- **Status:** provisional (simulation; two seeds of 16; ideal rendering and a grid lidar)
- **Week:** 6
- **Date:** 2026-10-20
- **Evidence:** [Week 6 log, 2026-10-20](week_06/notes.md), "The arm starts folded, and the handle as measured".
  - Results:
    - this demo: [seed 1](#/week/6/run/20261020T072044_door_combiner_door_seed1_cd16_gap27b_s1) 16/16,
      [seed 2](#/week/6/run/20261020T072351_door_combiner_door_seed2_cd16_gap27b_s2) 16/16;
    - the pose given: [seed 1](#/week/6/run/20261020T072654_door_decoupled_walk_ik_seed1_door16_decoupled_gap27b_s1)
      16/16, [seed 2](#/week/6/run/20261020T072754_door_decoupled_walk_ik_seed2_door16_decoupled_gap27b_s2) 16/16;
    - no falls.
  - **The fold.** The arm starts at `d1_hardware.REST_SERVO_DEG`, its J1 and J2 inside the URDF's limits. It stays
    folded through the landing and the look. It unfolds after the look, in a straight line in joint space to the
    home point's IK solution, each 10 Hz command at most 4 deg from the measured pose.
    - In a 4-attempt smoke, J1 went from -89.7 to +8.2 deg in 3 s.
    - The base tilted under 0.5 deg; nothing touched.
    - The body proxy finds the path clear throughout (`d1_ik.path_clearance`).
  - **The lever.**
    - A slot, 140 × 40 × 17 mm, turning about its near end, its back 27 mm off the door; it was an 18 mm bar 56.5 mm
      off.
    - The jaws open to 63.2 mm onto it (11.6 mm clear above and below) and squeeze it 14 N through the pull (about 10 N
      on the bar).
    - The placements moved 29.5 mm closer so the handle keeps its distance from the robot; the look moved back as far.
  - **The look and the lidar, against F-154:**
    - the tag found 32/32;
    - the box placed a median 2.2-2.3 mm and 0.12 deg off (max 3.0 mm, 0.64 deg);
    - the door read to a median 0.33-0.39 deg (p90 at most 2.1);
    - the verdict agreed with the simulator 32/32.
  - **A stale box.** The door runner cached the box's USD by the handle's torque alone, so the first runs with the
    changes simulated the old bar. They are recorded as such:
    - the folded start alone, 16/16 twice, valid as that;
    - the measured gap aimed at a box that did not have it, 6-7/16 and 13-14/16, invalid.
    - The cache is now keyed by the geometry and the builder's code. The cached box was byte-identical to one built from
      the committed geometry (81b9c4a), so F-146 to F-154 were run on the box they describe.
- **Scope:**
  - Simulation, as F-154: an ideal render, a grid lidar, the simulator's odometry, P2Dingo's walking policy.
  - Measured on the lever: its outline and its gap. Assumed: where it turns, its mass, the jaw's grip on its flat faces
    (PhysX friction).
  - The real arm's rest pose is 1-3 deg outside the URDF's limits here.
- **Implication:**
  - Folded at rest is a safe start for this sequence in simulation: the unfold is one joint-space move, after the box
    is found.
  - The measured lever does not cost the decoupled grip anything. A flat top and bottom grips firmer than the round
    bar did.
  - The cache bug is a reminder: a measurement change must be traced through to the simulated object, not only to the
    scripts.

### F-156 — The combiner door's robot program runs on the dog: its sensors, arm feedback and a dry run of the whole attempt all work, drawn in RViz over P2Dingo's zenoh link; the real dog walks on Unitree's sport mode, not the walking policy the simulation uses; and P2Dingo's charging dock carries the box's tag, which the look took for the box until it checked the lidar face's extent

- **Status:** provisional (one session on the dog, 2026-10-20, dry runs only; the dock in view, not the box)
- **Week:** 6
- **Date:** 2026-10-20
- **Evidence:** [Week 6 log, 2026-10-20](week_06/notes.md), "The combiner door on the real dog". `demos/combiner_door/robot/`
  (its README). Nine runs on the dog, all recorded:
  - watch runs, from [the first](#/week/6/run/20261020T063824_watch_dry) to
    [the face-extent guard](#/week/6/run/20261020T064744_watch_dry);
  - [a dry pull to 30 deg](#/week/6/run/20261020T065323_pull_dry).
  - **Sensors and arm.**
    - The head D435i at 1280x720 (fx 907.1, fy 906.0) and the MID-360 (20,064 points a frame) were read.
    - The arm link read the arm folded at [-1.4, -89.7, 90.0, 2.7, 11.1, -4.0] deg with the gripper at 67.1 units,
      P2Dingo's arm bridge disconnected.
    - RViz on the PC drew the robot, the arm, the box, the lidar's face points and the camera through the dog's zenoh
      bridge (two allow-list lines added there).
  - **The look, against P2Dingo's dock:**
    - the tag found 30 times in 2 s at about 1 m;
    - its own pose had the box tilted 5.2 deg, which the levelled fix ignores;
    - the lidar's face heading within 1.4 deg of the tag's, fitting at 19 mm rms;
    - the dog lying, pitched 6.2 deg.
  - **The dock's tag.** The face 20 mm behind the tag did not tell the dock from the box. The face carrying on past the
    door's edges did: 141% of its points round the tag lay outside the box's outline, where the synthetic box gives 0.
  - **The dry pull.**
    - All 14 phases ran in order, against the dock's tag with `--ignore_range`.
    - The ghost walked in to within 8 mm of the stance with the gait's carry modelled.
    - The IK held the hand's pose (transit steps fell back to the position).
    - The base was commanded only at sport mode's floors; the arm ended commanded to the fold.
  - **Walking.** P2Dingo walks the real Go2 through sport mode (`/api/sport/request`); its clocked-gait policy, which
    F-153 to F-155 walk with, has only run in Isaac Sim. P2Dingo measured sport mode's dead zones (still at 0.12-0.15
    m/s, no turn on the spot at 0.30-0.42 rad/s, 75 mm of carry from 0.21 m/s).
- **Scope:**
  - Dry: nothing was commanded to the legs or the arm.
  - The box was not in view.
  - The dog was lying, so its pose for the look was not a standing one.
  - One room, one session.
- **Implication:**
  - The program is ready for its first live steps, in the README's order: unfold, then walk, reach, grip, pull.
  - **Walking will not match the simulation.** The pull's base path, 0.12 m/s sideways and 0.22 rad/s, lies inside
    sport mode's dead zones, so on the robot it is walked in lifted bursts. The pull is the step most likely to differ
    from simulation, and F-154's "P2Dingo's walking policy carrying the arm" was never on the table for hardware.
  - Keep the dock out of view; the extent check is the guard.

### F-157 — Two earlier designs do not fit the lever as measured: the L-lip claw's lip tips stand 37.2 mm apart fully open, so the 40 mm lever cannot enter between them from the front; and the turning demo's 60-80 deg grasps put the wrist RealSense into the door, 27 mm behind the lever

- **Status:** confirmed (geometry, CPU)
- **Week:** 6
- **Date:** 2026-10-20
- **Evidence:** [Week 6 log, 2026-10-20](week_06/notes.md), "The arm starts folded, and the handle as measured".
  - **The claw.** `demos/unifp/claw.py`: the lips reach 20 mm in from each finger, 37.2 mm between their tips with the
    jaws fully open. The lever is 40 mm tall. UniFP's claw runs (F-07x) were on the 18 mm bar.
  - **The wrist camera.** `demos/combiner/tests/test_turn.py::test_camera_and_bracket_clear_the_door_lever_and_spindle_at_every_grasp`:
    - at the 60 and 80 deg grasps the camera or its bracket comes within 0.05-19 mm of the door or the lever;
    - pitched 40 deg or less, 14 mm or more off the lever and 20 mm or more off the door.
    - The camera has no collider in the demos, so a simulated run would not show the collision.
- **Scope:** CAD and measured outlines; the lever's pivot assumed at its near end.
- **Implication:**
  - The lip claw needs lips under 18.6 mm deep for the lever to pass between them at all with the jaws fully open
    (77.2 mm), and shallower for any clearance; or a sideways entry.
  - The turning demo's planner still lists the 60-80 deg grasps; with the wrist camera fitted they should be dropped.
    Not changed here: Lukas's call.

### F-158 — Letting go after the pull, the door script's straight line from the lever to the home point runs the fingers into the lever and its 63 mm hub, as Lukas saw on the dog; backing the hand straight off along its own approach first keeps them 11.5 mm off the lever and 38 mm off the hub

- **Status:** provisional (CPU model, and one live observation; the ungrasp has not run on the dog)
- **Week:** 6
- **Date:** 2026-10-20
- **Evidence:** [Week 6 log, 2026-10-20](week_06/notes.md), "The combiner door on the real dog", the last block.
  - **Live** ([the first live pull](#/week/6/run/20261020T090355_pull_live) and Lukas's runs after it): returning
    home from the lever, the fingers caught the lever's hub and pushed the door. The hub is 63 mm across (Lukas,
    photographed); it is assumed to fill the 27 mm gap behind the lever, centred on the spindle.
  - **Model:** `demos/combiner_door/tests/test_robot.py::test_letting_go_keeps_the_fingers_off_the_lever_and_its_hub`.
    - A dry pull to 30 deg, with the arm link's profile driving a model servo.
    - The fingers are sampled through their CAD volume (`grasp.FINGER_*`) and measured against the slot lever and the
      hub at the script's door angle.
    - **The script's release then "back":** 8.2 mm into the lever and 0.7 mm into the hub. The fingertips stand 20 mm
      past the jaw centre, in the gap behind the lever. With the door turned 30 deg relative to the base, the line home
      runs 11 cm along the door towards the hub for every 5 cm out from it.
    - **The ungrasp:** the jaws opened where the hand measures itself, then 12 cm straight back along Link6's +z, its
      orientation held. Never closer than 11.5 mm to the lever (the open jaws' clearance) or 38 mm to the hub.
- **Scope:**
  - The hub's depth and centring are assumed, from the photo.
  - The door is at the script's angle; the live door read 40-51 deg against 30 commanded.
  - The fingers only (not the wrist camera or Link6).
  - The simulated demos' door script (`demos/unifp/script.door_phases`) still goes home straight from the lever. Their
    asset has a 24 mm spindle and a 60 mm rosette 6 mm deep, not the hub.
- **Implication:**
  - Leave a held handle along the approach the hand came in on, by more than the fingers' 29 mm past the lever's front,
    before any sideways move.
  - The simulated door pulls may also drag the lever on the way home. Their scores are taken at the end of the pull, so
    F-146 to F-155's door counts stand. Whether the released door was pushed back has not been checked.

### F-159 — Trained UniFP-style with the front-right foot as the end effector, the Go2 stands on three legs and tracks foot goals at body height to ~5 cm, but turns a lever door handle only when it is light: after a handle stage, 17 (position-only) and 12 (force-aware) of 144 frozen handles held 1 s, all but one on the weakest spring; the foot slips off a 2 cm round bar or overloads before the lever reaches its stop, so the force-aware variant shows no advantage in success

- **Status:** superseded by [F-160](#f-160) (2026-10-20, two hours after it was written: the second seed reverses
  the steadier-hold observation below and widens the gap in success; the seed-1 numbers stand)
- **Week:** 6
- **Date:** 2026-10-20
- **Evidence:** [Week 6 log, 2026-10-19/20](week_06/notes.md), "The front-right foot as the end effector: a door handle
  at body height"; `unifp_train/README.md`, "The front-right-leg task".
  - The task (`--task leg`): UniFP's environment with the FR foot sphere's centre as the controlled point, its goal
    walk about the FR thigh joint, 12 actions (the legs), the D1 held at a look pose aiming the wrist RealSense at the
    handle region, UniFP's pushes on the foot. Variants as the plan's P0/P1: *position* (no force command, the push a
    disturbance to reject, no force in critic or estimator target) and *force* (the target `goal + (external +
    command) / 400 N/m`, the estimator on the external force).
  - Stage 1, free space ([force](#/week/6/run/20261019T132551_train_seed1_leg_force_v3_s1),
    [position](#/week/6/run/20261019T151333_train_seed1_leg_position_v3_s1)), 4096 envs: three-legged standing from
    iteration ~35, foot error at reset ~5 cm from ~400 (training-time). Two earlier force runs did not lift the foot
    at all (v1, v2): "lifted" meant "under 5 N", and they rested it on the floor unloaded. At full +-35 N pushes
    (from 1800) the force-aware run's episodes fell from 776 to 622 steps; both variants' `model_1200` were used.
  - Zero-shot on the 144 frozen handles (3 door distances x 3 sides x 2 lever directions x 4 springs x 2 heights,
    estimate errors drawn once), final plant:
    [force](#/week/6/run/20261019T175951_leg_eval_seed1_leg_eval_force_stage1_1200_j) 0/144 (108 engaged, 48 past
    120 N, mostly slipped), [position](#/week/6/run/20261019T180115_leg_eval_seed1_leg_eval_position_stage1_1200_j)
    0/144 (124 engaged, 54 damaged).
  - Stage 2, recipe j ([force](#/week/6/run/20261019T175803_train_seed1_leg_force_stage2_j),
    [position](#/week/6/run/20261019T175803_train_seed1_leg_position_stage2_j)), 1500 iterations from `model_1200`,
    handles in 60% of episodes, then the frozen set:

    | final checkpoint (`model_2699`) | latch held 1 s | opened at all | damaged | slipped (any) | 1 / 2 / 3 / 4 N·m held | peak median (p95) |
    | --- | --- | --- | --- | --- | --- | --- |
    | [position-only](#/week/6/run/20261019T223051_leg_eval_seed1_leg_eval_position_stage2_j_2699) | **17** | 48 | 24 | 136 | 16 / 1 / 0 / 0 | 80 N (128) |
    | [force-aware](#/week/6/run/20261019T222952_leg_eval_seed1_leg_eval_force_stage2_j_2699) | **12** | 61 | 48 | 132 | 12 / 0 / 0 / 0 | 86 N (138) |

    Neither fell; both engaged all 144. The force needed at the latch is 9 / 17 / 25 / 32 N for the four
    springs; the force held while the latch is open is ~11 N for both. The lever reaches its end stop almost never.
  - Every bar 20 mm in radius instead of 8-12 (a diagnostic, not the frozen set):
    [position](#/week/6/run/20261019T223120_leg_eval_seed1_leg_eval_position_stage2_j_2699_bar20mm) 22 held, 108
    damaged; [force](#/week/6/run/20261019T223022_leg_eval_seed1_leg_eval_force_stage2_j_2699_bar20mm) 14 held, 122
    damaged; slipping as the failure falls from 74 / 59 episodes to 2 / 3, peaks rise to ~130 N median for both.
  - Holding with the latch open (14 force-aware, 23 position-only episodes): on the bar 0.68 / 0.58 of the hold,
    contact force std/mean 0.88 / 1.10, lever angle std 1.4° / 2.6°. On handle 64 (1 N·m) the force-aware policy
    held the lever at 45° on a steady ~6 N where the position-only one bounced 0-50 N and the lever between 30° and
    42° (`results/week_06/figures/leg_handle_eval_stage2_seed1.png`).
  - The wrist camera at the look pose (`leg_cfg.ARM_LOOK_POSE`, 0.89 m up) sees the door, the rose and the FR foot,
    but a fixed shape on its own link -- almost certainly the D1's closed fingers -- covers ~40% of the image on the
    right, and for a lever pointing right hides the bar and the foot through the press
    (`results/week_06/figures/leg_handle_frames_force_handle64.png`).
- **Scope:**
  - One seed per variant; a second (stage 2 only) was started 2026-10-20 09:40. 12 against 17 is within what a seed
    can move.
  - The handle is a torch plant: a capsule bar on a sprung spindle, penalty contact at the foot sphere and the knee,
    nothing in PhysX. Its stiffness, damping, friction and every handle number are invented, and the contact model was
    changed twice (damping bounded by the spring; 16,000 -> 8,000 N/m) because it produced artefacts. No PhysX check.
  - The scripted approach, the outer loop on the foot's measured position, the touch-then-turn reference and the
    force law are part of what was evaluated; so is the 120 N damage force, which is a declared limit, not a measured
    handle strength. The task layer is told the handle's pose with drawn errors; no image was used to find it.
  - The recipe was revised against training-time handle numbers, never the frozen set, but ten starts on one task is
    a search, and the result is that search's.
- **Implication:**
  - The leg can be made a manipulator in simulation -- three-legged stance, foot goals at body height, a light lever
    turned and held -- but turning door handles at 2-4 N·m is not demonstrated. The limits found are the foot's
    precision on a thin round bar (~1.6 cm off centre at μ 0.6 starts outside the friction cone; engagement is a median
    ~1.1 cm off in training), and the force once the foot stays on.
  - The force-aware formulation's argument for this task -- a bounded force into an end stop -- was never tested,
    because neither policy brings the lever to its stop. A steadier hold is the only difference seen, and it is small.
  - Before more training: a paddle-shaped or wider lever (or a foot pad), a contact-location estimate to centre the
    foot, and a PhysX check of the plant. For the camera: a look pose that keeps the fingers out of the view of the bar,
    or the fingers opened, before any perception claim.

### F-160 — Over two seeds of the handle stage, the position-only foot policy turns more lever handles than the force-aware one, 60 of 288 frozen handles held 1 s against 15, with a threefold spread between seeds; the force-aware policies hold ~12 N and slip off, the position-only ones push to ~60 N, turn springs up to 3 N·m and pass the 120 N damage force more often

- **Status:** provisional (simulation; two seeds of stage 2 from one stage-1 checkpoint per variant; the analytic
  handle plant of F-159). Its scope's "the policies deliver ~12 N whatever is asked" is corrected by
  [F-161](#f-161) (2026-10-21): the force law asked for little more; the rest stands.
- **Week:** 6
- **Date:** 2026-10-20
- **Supersedes:** [F-159](#f-159)
- **Evidence:** [Week 6 log, 2026-10-19/20](week_06/notes.md), "The front-right foot as the end effector: a door handle
  at body height". Task, stage 1 and recipe j as in F-159; the second seed is the same command with `--seed 2`
  ([force](#/week/6/run/20261019T223505_train_seed2_leg_force_stage2_j_s2),
  [position](#/week/6/run/20261019T223505_train_seed2_leg_position_stage2_j_s2)).
  - The 144 frozen handles, final checkpoints (`model_2699`), latch held 1 s:

    | policy | seed 1 | seed 2 | of 288 | held at 1 / 2 / 3 / 4 N·m, seed 2 | damaged, seed 1 / 2 | slipped as the failure, seed 1 / 2 |
    | --- | --- | --- | --- | --- | --- | --- |
    | position-only | [17](#/week/6/run/20261019T223051_leg_eval_seed1_leg_eval_position_stage2_j_2699) | [43](#/week/6/run/20261020T003041_leg_eval_seed1_leg_eval_position_stage2_j_s2_2699) | **60** | 28 / 11 / 4 / 0 | 24 / 59 | 74 / 31 |
    | force-aware | [12](#/week/6/run/20261019T222952_leg_eval_seed1_leg_eval_force_stage2_j_2699) | [3](#/week/6/run/20261020T003011_leg_eval_seed1_leg_eval_force_stage2_j_s2_2699) | **15** | 3 / 0 / 0 / 0 | 48 / 6 | 59 / 126 |

    No falls in any. The latch needs 9 / 17 / 25 / 32 N at the foot. Force held with the latch open, seed 2:
    position-only 12 / 23 / 34 / 60 N by spring, force-aware ~12 N. Peak force median, seed 2: 92 N (p95 138)
    position-only, 59 N (p95 115) force-aware.
  - Holding with the latch open, seed 2: position-only 57 episodes, on the bar the whole hold, contact force
    std/mean 0.53, lever std 1.1°; force-aware 3 episodes. Seed 1 had shown the force-aware policy the steadier
    (0.68 against 0.58 of the hold on the bar, std/mean 0.88 against 1.10): the difference is the seed's, not the
    variant's. Figures: `results/week_06/figures/leg_handle_eval_stage2_seed1.png`, `..._seed2.png` (handle 64: the
    seed-2 position-only policy holds the lever at 42° on a steady ~21 N, the force-aware one touches at ~5 N and
    slips off).
  - Seed 2's position-only policy passes the damage force mostly on levers pointing left (55 of 72) and succeeds
    mostly on levers pointing right (31 of 43).
- **Scope:**
  - Two seeds of stage 2, each from the same stage-1 checkpoint; stage 1 itself was one seed per variant, and its
    force-aware run had degraded under full-strength pushes before `model_1200` was taken (F-159). A seed spread of
    17-43 and 3-12 says a two-seed difference of this size is suggestive, not established.
  - F-159's scope applies: invented handle numbers, a torch plant, no PhysX check, a task layer and recipe found over
    ten training starts, the 120 N damage force declared rather than measured.
  - The force-aware variant's weakness may be its task layer as much as its policy: the PI law asks for at most
    1.8x the rated force within 12-40 N, through UniFP's 400 N/m virtual spring, and the policies deliver ~12 N
    whatever is asked; the position-only variant is driven by a goal past the stop and pushes as hard as that takes.
- **Implication:**
  - On this task, as built, force awareness does not help the foot turn handles; position-only does better in both
    seeds. Do not quote the force-aware variant as the better leg controller, and do not quote F-159's steadier hold.
  - The force-aware policy's failure is under-pushing and slipping, not over-pushing; its claimed advantage (a bounded
    force into an end stop) was never reached. Testing it needs levers the foot can stay on (wider bars or a pad), and
    a force law the policy actually follows -- check commanded against applied force before retraining.
  - The position-only policy's cost is force: 59 damaged on seed 2, and up to 60 N held at 4 N·m without opening.

### F-161 — The force-aware handle policies of F-160 held ~12 N because their force law asked for little more, not because they ignored larger commands: the law raises its force only from the foot's lag behind the reference, a foot slipping off the 2 cm bar does not lag, and the largest command in an episode had a median of 16.5 N (seed 1) and 9.1 N (seed 2) against caps of 19-40 N

- **Status:** confirmed (a re-reading of F-160's recorded frozen traces; no new simulation)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key in the cam lock", the 12 N paragraph. The traces of
  [seed 1](#/week/6/run/20261019T222952_leg_eval_seed1_leg_eval_force_stage2_j_2699) and
  [seed 2](#/week/6/run/20261020T003011_leg_eval_seed1_leg_eval_force_stage2_j_s2_2699)'s force-aware evaluations,
  every step the layer pressed or held with the foot on the bar:

  | command | steps, seed 1 / 2 | delivered along the turn, median, seed 1 / 2 | contact force, median |
  | --- | --- | --- | --- |
  | 0-8 N | 4015 / 6694 | 5.6 / 3.1 N | 6.6 / 5.0 N |
  | 8-16 N | 2132 / 680 | 14.7 / 13.7 N | 19.0 / 15.6 N |
  | 16-24 N | 465 / 155 | 10.8 / 21.3 N | 14.6 / 24.4 N |
  | 24-41 N | 51 / 14 | 12-20 / 27-76 N | 16-27 / 28-77 N |

  The foot was off its goal by a median 17 mm (seed 1) and 50 mm (seed 2) while pressing: at the 400 N/m virtual
  spring, 7-20 N of the force is set by position error alone. Seed 2's handles failed by slipping in 126 of 144.
- **Scope:** F-160's handles, plant, layer and checkpoints; the reading is of the delivered force along the lever's
  turning direction (`drive_n`) against the layer's command, at the policy rate.
- **Implication:**
  - The handle's force-aware result was set by the bar and the law, not by a limit of the leg or of the policy's
    force channel: up to 16 N the policy delivered roughly what it was asked. A surface the foot stays on lets the
    law build force; on the key's flat platform (F-163) the same law asks for a median 21 N per episode (p90 32).
  - Keep reading commanded against delivered force before reading success: the law's lag-only build-up is a design
    choice of the layer, and on a slipping contact it stays low by construction.

### F-162 — The handle policies Lukas watched stood with a rear foot 13-20 cm out of place because they put it there while the robot dropped from its 0.35 m spawn, and nothing asked for it back; starting every episode standing settled and rewarding each support foot's distance from its place keeps the worst support foot within 3-4 cm in free space (force-aware), though the position-only run came apart from iteration ~600

- **Status:** provisional (simulation; one seed of stage 1 per variant; training-time numbers except the probe)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key in the cam lock".
  - The probe (a scratch script, not recorded): recipe j seed 1's final handle checkpoints on 64 robots against zero
    actions. Zero actions settle the feet at FL (+0.23, +0.17), RL (-0.22, +0.17), RR (-0.22, -0.17) m from the base,
    base 0.272 m. Under both policies the right-rear foot was 13-20 cm off that by 0.1 s (five policy steps) and the
    left-rear 12-13 cm: the right-rear forward under the body, where a tripod over the other three has room for the
    centre of mass (with the feet in place it sits on the FL-RR diagonal). For the first ~0.08 s of an episode the feet
    are in the air. Once down, `support_slip` paid for keeping them there.
  - The fix, in the key task (`--leg_object key`): every episode starts standing settled (the base 0.272 m, the knees
    as gravity bends them at the default targets, measured), `support_stance` (exp(-(d / 6 cm)^2) per support foot,
    weight 4) and the front-right foot's footfall where it actually stands. Stage 1 at weight 2 (stopped at 463): the
    median support foot 2.4 cm from its place, the worst ~15 cm. At weight 4,
    [force-aware](#/week/6/run/20261020T140313_train_seed1_key_force_s1_stage1b): the worst support foot 3.6-4.0 cm
    (per-100-iteration means, 400-1300), the foot's free-space error 5.2-6.9 cm.
    [Position-only](#/week/6/run/20261020T140352_train_seed1_key_position_s1_stage1b), identical to it until the pushes
    and the force channel start at 500: the worst support foot 7.0 cm at 600-700, 15 cm from 700, the foot's error
    6.4 cm at 700 rising to 24 cm by 1300.
  - Turning the key (F-163) the robots do move a foot: over the frozen keys' episodes the worst support foot stands a
    median 8.9 cm (force-aware) and 10.7 cm (position-only) from its place.
- **Scope:** training-time stance numbers sampled at reset over each iteration's ending episodes; one seed; the
  settled state measured once (8 robots, zero actions, the arm folded at rest). The stance reward's weight was chosen
  from one stage-1 start.
- **Implication:**
  - Lukas's reading was right: the spawn's drop gave the policies a free step, and the reward let them keep it. Spawn
    standing, and pay for the stance, in any leg-manipulation training.
  - The position-only stage 1 is unstable under foot pushes with the stance reward in this recipe (the force-aware
    one, which yields to the push by design, is not); its stage 2 started from both variants' `model_600`, before it
    came apart. That is a training-recipe result about P0, not yet repeated on a second seed.
  - "Feet near the normal stance" holds in free space, not while turning the key: there a support foot moves ~9 cm.
    A tighter stance during the turn would need more weight there, or a body that leans further.

### F-163 — Turning Lukas's lever key with the front-right foot from 45 degrees up to 45 down, the force-aware policy unlocks 84 of 108 frozen keys with no damage and the foot back on its footfall, against 20 for the position-only one (seed 1); the force-aware foot also pushes less hard (82 N against 142 N median peak), and neither turns any zero-shot

- **Status:** superseded by [F-166](#f-166) (2026-10-21, the second seed: it confirms the ranking and widens it; the
  seed-1 numbers stand)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key in the cam lock". The key task
  (`--task leg --leg_object key`, F-162's stance fixes): the key's plant (the foot sphere against the key's bar and
  100 x 50 mm platform, a lock friction that sticks, a cam detent before the unlock, end stop at +45 degrees, the
  key on a lock adapter 17 mm off the door at the real box's lock, 0.4175 m up), the handle task's layer starting at
  -45 degrees, the foot returned to its footfall afterwards. Stage 1 (1300 iterations, 2048 envs) and stage 2 (from
  both variants' `model_600`, 60% key episodes, 1000 iterations, recipe j's settings); the 108 frozen keys
  (`leg_eval.build_key_plan`), one episode each, the last checkpoints (`model_1607`, `model_1615`):
  [force](#/week/6/run/20261020T174654_leg_eval_seed1_key_eval_force_s1_final),
  [position](#/week/6/run/20261020T174805_leg_eval_seed1_key_eval_position_s1_final).

  | | force-aware (P1) | position-only (P0) |
  | --- | --- | --- |
  | unlocked, foot back down, no damage, no fall | **84** | **20** |
  | at lock friction 0.5 / 1.5 / 2.5 N·m (36 each) | 36 / 35 / 13 | 18 / 2 / 0 |
  | past 120 N | 14 | 79 |
  | peak force, median (p95) | 82 N (136) | 142 N (218) |
  | time to unlock, median | 1.6 s | 2.5 s |
  | zero-shot from stage 1 | 0 | 0 |

  No falls. Training-time (`figures/key_training_curves_seed1.png`): the force-aware policy's key episodes unlocked
  ~20-30% for its first 300 stage-2 iterations and ~95% by its last; the position-only one's stayed under ~15%, its
  foot 7-9 cm short of the pre-contact point when the layer engaged. The force-aware law asked a median 21 N per
  episode (p90 32) and the foot delivered ~16-20 N along the turn of commands of 15-45 N (F-161's handle: 9-16 N
  asked).
- **Scope:**
  - One seed; the handle study's seeds spread threefold (F-160). Stage 1 for both from `model_600`, chosen on
    training-time numbers because the position-only run came apart after it (F-162); stage 2 stopped at 1000
    iterations for time, where the force-aware run was still improving.
  - Not PhysX: the key's turn and its contact with the foot are the training's plant, its friction and detent
    invented (0.3-2.5 + 0-1 N·m, to exercise the leg). The lock's place on the box is Lukas's measurement and photo;
    the key's shape is his STL, mirrored (F-164). The estimate the layer is told is the training's (+-1 cm, +-30% on
    the lock's resistance); the stance the policy starts from is its own settled one.
  - The 120 N damage force is declared, not measured; the force-aware policy still meets the platform at 82 N median
    where the lock needs ~5-37 N.
- **Implication:**
  - On a contact the foot can stay on, the force-aware formulation is the better leg controller here, the reverse of
    the handle (F-160, F-161): the flat platform lets the force law build force, and the position-only policy, driven
    by a goal past the stop, overloads (79 of 108 past 120 N) instead of turning.
  - Quote it with its seed count until seed 2 is in. The impact peaks are the force-aware policy's weak point: what
    damages a printed key or lock is the meeting, not the push.

### F-164 — Without learning, the D1 puts Lukas's key on the cam lock's keyway axis to 1.5 mm and seats it when the box's place is known exactly; from the head camera's and lidar's look it seated 0 of 12, because the tag's range is off by 3-11 mm along the door's normal and the position-aimed push stalls; the key as drawn is handed the wrong way for the turn asked for, and the D1 at the lock's reach has no torque to spare

- **Status:** provisional (simulation, PhysX; one development attempt with the box given, twelve with the look over
  three runs at four placements)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key in the cam lock", the demo's development and
  camera runs; `demos/combiner_key/` (README, 9 CPU tests).
  - The method: the combiner door demo's stack (P2Dingo's walking policy to the stance, the decoupled baseline's arm IK
    at 10 Hz through the D1's planner and servo model), the key held in a capped spring grip, a pre-insertion point
    15 mm short along the estimated keyway axis rolled to 45 degrees, the IK warm-started from a seeded search there,
    an outer loop on the hand's measured miss (the encoders' forward kinematics on a real arm), then straight in to
    4 mm past seated. The slot adjusted: a printed adapter over the cam lock, its keyway the key's outline plus
    1.5 mm a side behind a 5 mm 45-degree lead-in.
  - Box given ([run](#/week/6/run/20261020T161133_force_seed1_pipeline_test_insert_given7)): 25 mm out and 0.5 / 0.7 mm
    across before going in; seated 0.16 mm out, 1.5 mm across, roll -0.8 deg, tilt 0.7 deg; still seated after letting
    go. The six attempts before it (recorded) fixed the grip (a soft one let the key pivot 48 degrees on the adapter)
    and found the arm's limit (below).
  - With the look ([force](#/week/6/run/20261020T175404_force_seed1_key_demo_force_s1),
    [position](#/week/6/run/20261020T175841_position_seed1_key_demo_position_s1),
    [v2](#/week/6/run/20261020T180321_force_seed1_key_demo_force_s1_v2); the insertion does not depend on the leg policy):
    tags found 12/12; the lock's estimate off -3.0 / +2.5 / +9.9 / +11.4 mm along the door's normal and under 0.6 mm
    across, heading 0.1-1.1 deg. Where the depth was within ~3 mm the key reached the keyway on its axis (1.1-1.4 mm
    across) and stalled 2.7-7.8 mm short, tilted 2-4 deg: the push is the grip's spring stretched by how far past the
    seat the IK aims, 0.6-4 N as the depth error moves it. Where the depth was 10-11 mm off it missed the lead-in.
    Taking the depth from the lidar's face fit did not help (it read the face 1.8-2.6 mm in front of every estimate);
    aiming 15 mm deeper put the IK out of what it solves.
  - The arm's margin: on the CPU model the insertion poses need 78-84% of the worst joint's limit. The leg policy's
    training robot carries the wrist RealSense and bracket (99 g) on Link6; with it and the key the D1 saturated at the
    shoulder and sagged 16 mm, and an outer loop could not recover it. The demo uses the robot without (the camera is
    on the head in this task).
  - The key (`combinerbox_lever v3.stl`): with its shaft in the lock and its platform facing up, its lever points to
    the robot's left, off the box past its edge. The turn asked for -- the front-right foot pushing it clockwise, down,
    toward the box's middle -- needs the mirror image; the simulation uses it mirrored.
- **Scope:** PhysX contact on boxes (a square shaft in a rectangular keyway behind the lead-in; the cylinder's own
  keyway not modelled; the key held by a spring, not by friction at the fingers); the perception simulated (an ideal
  picture of a printed tag, a grid ray caster for the lidar); four box placements; the D1's servo model and the
  combiner door demo's robot.
- **Implication:**
  - Lateral alignment is solved by IK and the outer loop; the open problem is along the axis. A push limited by force
    rather than aimed by position (the D1 in a compliant mode along the approach, or a spring in the gripper), or a
    depth the robot measures itself (the hand touching the adapter's face before the insertion), is the next step;
    the lidar as used here is not it.
  - Print the key mirrored (or turn the requirement round) before trying the turn on the real box.
  - The D1 is near its torque limit at the lock's height and reach: keep the wrist camera off the arm for this task,
    and do not add mass to the hand.

### F-165 — In the staged PhysX demo the key-trained leg policies keep their frozen-evaluation ranking only on the leg actuator they trained on: on the decoupled stack's DCMotor legs the force-aware policy turns none and falls from 84 to 3 of 108 frozen keys, while the position-only one is unchanged; and neither can take over from the walking policy's stance, which leaves a support foot ~16 cm from where the leg policy's training always started it

- **Status:** provisional (simulation; seed 1's final policies; four demo attempts per condition, the box given in the
  diagnostics); its reading of why the force-aware policy fell on the DCMotor legs corrected by
  [F-168](#f-168) (2026-10-21: the stock group's zero leg armature, not its torque limits or the estimator; with the
  trained armature the same group gives 84, and seed 2 holds 104 even at zero)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key in the cam lock", the demo's camera runs and the
  diagnostics after them.
  - The turn in the demo's scene from the leg policy's own settled stance (a teleport, `--handover settled`):

    | legs | force-aware | position-only |
    | --- | --- | --- |
    | DCMotor (the walking policy's; head camera and lidar look) | [0/4 engaged](#/week/6/run/20261020T180759_force_inserted_settled_seed1_key_demo_force_s1_settled) | [4/4 unlocked, 3 of them past 120 N](#/week/6/run/20261020T181112_position_inserted_settled_seed1_key_demo_position_s1_settled) |
    | UniFP's ideal PD (the leg policy's; box given) | [**4/4**, peaks 76-88 N](#/week/6/run/20261020T182511_force_inserted_settled_unifplegs_seed1_key_demo_force_s1_settled_unifplegs_given) | [0/4 turned, all past 120 N](#/week/6/run/20261020T183653_position_inserted_settled_unifplegs_seed1_key_demo_position_s1_settled_unifplegs_given) |
    | UniFP's ideal PD; head camera and lidar look (1.1-10.3 mm off) | [**3/4**, peaks 70-87 N (the fourth 181 N)](#/week/6/run/20261020T201437_force_inserted_settled_unifplegs_seed1_key_demo_force_s1_settled_unifplegs_camera) | [0/4, all past 120 N](#/week/6/run/20261020T201806_position_inserted_settled_unifplegs_seed1_key_demo_position_s1_settled_unifplegs_camera) |

  - The frozen evaluation with only the legs' actuator swapped (`--leg_actuator dcmotor`): force-aware
    [3/108](#/week/6/run/20261020T181728_leg_eval_seed1_key_eval_force_s1_final_dcmotor) (engaged 2, 105 past 120 N),
    position-only [20/108](#/week/6/run/20261020T182100_leg_eval_seed1_key_eval_position_s1_final_dcmotor), against 84
    and 20 on the trained actuator (F-163). The DCMotor is Isaac Lab's Go2 model: 23.5 N·m on every joint with a
    torque-speed curve, the same 25 / 0.5 gains; UniFP's ideal PD has the knees at the Go2's 45.43 N·m.
  - The handover itself (`--handover crouch`: the legs eased over 1.5 s from the walking policy's last action to the
    default targets, then the leg policy), on either actuator: the walk arrives within 1.2-1.7 cm, but easing moves the
    base back over feet that stay where the walking policy left them, so the leg policy starts with its worst support
    foot 15.6-15.9 cm from its place and the door 0.54-0.56 m ahead (it trained at 0.40-0.47): it engaged 0 of 4
    ([run](#/week/6/run/20261020T184836_force_inserted_unifplegs_seed1_key_demo_force_s1_crouch_unifplegs_given) on
    ideal-PD legs, and 0 in the earlier development runs on DCMotor).
  - The robot's own check, the turn as the foot measured it at the end of the hold, read "past 30 degrees" on all four
    position-only attempts on ideal-PD legs whose keys never turned: the foot had slid down past the platform.
  - The head RealSense cannot see the key or the foot from the turn's stance: rendered, it sees only the door, the key
    being ~10 cm ahead of it and ~48 degrees to its right, outside the D435's +-34.5 degrees
    ([frames](#/week/6/run/20261020T202459_leg_eval_seed1_key_frames_force_s1_v2),
    `figures/key_frames_force_key54.png`).
- **Scope:** four attempts per demo condition; seed 1; the demo's other differences from training (self-collision
  off, the payload not simulated) not separated; why the force-aware policy is the sensitive one is a reading, not a
  test (its estimator infers the external force from its own dynamics, so an actuator that answers torque differently
  looks like a push).
- **Implication:**
  - F-163's force-aware advantage is conditional on the actuator model. Before anything goes to the dog, train and
    evaluate the leg policy on an actuator model checked against the real Go2's motors (and randomise it), and say
    which actuator every comparison used.
  - The staged sequence is missing its handover: train the leg policy from the stances the walking policy (or the
    sport mode, F-156) actually leaves -- recorded stops, randomised feet -- or have it step to its stance first.
  - Do not verify the unlock from the foot's position alone; the lidar on the door (F-154) or the door opening is
    the check.

### F-166 — Over two seeds, the force-aware foot policy unlocks Lukas's key in 190 of 216 frozen keys with no damage and the foot back on its footfall, the position-only one in 43; the force-aware one does it at every lock friction tried, pressing a median 50-82 N, and the position-only one only on the lightest lock, past 120 N in 113 of 216

- **Status:** provisional (simulation; two seeds of stage 2 from one stage-1 checkpoint per variant; the analytic key
  plant; stage 2 cut to 1000 iterations; on the trained actuator only -- F-165); its comparison conditional on the legs' armature,
  [F-168](#f-168) (2026-10-21: at 0.01 kg·m² with knees at 0.037, force-aware 169 and position-only 136 of 216); and its turn (45 degrees up to 45 down) is not the one Lukas asked for, which starts horizontal
  ([F-176](#f-176), 2026-10-21 afternoon)
- **Week:** 6
- **Date:** 2026-10-21
- **Supersedes:** [F-163](#f-163)
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key in the cam lock". Task, stage 1 and stage 2 as in
  F-163; the second seed is the same command with `--seed 2`
  ([force](#/week/6/run/20261020T181513_train_seed2_key_force_s2_stage2b),
  [position](#/week/6/run/20261020T181612_train_seed2_key_position_s2_stage2b)), scored at the checkpoints its
  interrupt saved (`model_1605`, `model_1614`), no selection.

  | 108 frozen keys | seed 1 | seed 2 | of 216 | at lock friction 0.5 / 1.5 / 2.5 N·m, seed 2 (36 each) | past 120 N, seed 1 / 2 | peak force median, seed 1 / 2 |
  | --- | --- | --- | --- | --- | --- | --- |
  | force-aware | [84](#/week/6/run/20261020T174654_leg_eval_seed1_key_eval_force_s1_final) | [106](#/week/6/run/20261020T201142_leg_eval_seed1_key_eval_force_s2_final) | **190** | 36 / 36 / 34 | 14 / 1 | 82 / 50 N |
  | position-only | [20](#/week/6/run/20261020T174805_leg_eval_seed1_key_eval_position_s1_final) | [23](#/week/6/run/20261020T201252_leg_eval_seed1_key_eval_position_s2_final) | **43** | 23 / 0 / 0 | 79 / 34 | 142 / 83 N |

  Success: past the unlock (30 degrees down), the foot back on its footfall, no contact past 120 N, no fall. No falls in
  any. The force law's largest command per episode, median (p90): 21 (32) N on seed 1, 26 (45) N on seed 2. Time
  from engaging to unlocked, median: force-aware 1.6 / 2.4 s, position-only 2.5 / 2.0 s. The worst support foot over
  an episode, median: 8.9 / 8.4 cm (force-aware), 10.7 / 7.5 cm (position-only). Zero-shot from stage 1: 0 of 108
  for both. Figures: `figures/key_eval_stage2_seed{1,2}.png`, `figures/key_training_curves_seed{1,2}.png`.
- **Scope:**
  - Two seeds of stage 2 from the same stage-1 checkpoint per variant (`model_600`, chosen on training-time numbers
    because the position-only stage 1 came apart after it, F-162); stage 1 itself one seed. The seeds agree here
    (84 / 106 and 20 / 23), unlike the handle's (F-160).
  - F-163's scope applies: the analytic plant with invented lock numbers, the training's estimate errors, the policy's
    own settled stance to start from, the 120 N damage force declared.
  - On UniFP's ideal-PD legs, as trained. On Isaac Lab's Go2 DCMotor legs seed 1's force-aware policy falls to 3 of 108
    and the position-only one stays at 20 (F-165): the ranking is conditional on the actuator model.
- **Implication:**
  - On a contact the foot can stay on, the force-aware formulation is clearly the better leg controller in this
    setting: it turns harder locks, pushes less hard and damages less, on both seeds. This is the reverse of the
    handle (F-160), whose failure was the bar and the force law, not the formulation (F-161).
  - The position-only policy, driven by a goal past the stop, turns only the 0.5 N·m lock and overloads on the rest.
  - The next questions are the actuator (F-165) and the handover from the walking policy (F-165), not the policy's
    skill on its own trained conditions.

### F-167 — Fine-tuned for 500 iterations with half its episodes starting from 32 stances the walking policy actually leaves, the force-aware key policy starts from those stances at rest and unlocks 97 of 108 frozen keys (26 before the fine-tune), keeping 102 of 108 from its own; but in the staged demo's live handover it still fails (0 of 12), so what blocks the sequence is the handover itself, not the stance alone

- **Status:** provisional (simulation; one fine-tune of one policy, force-aware seed 2; a diagnostic evaluation with the
  stances drawn per episode)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key in the cam lock", the handover paragraph.
  - The stances: 32 attempts of the demo on ideal-PD legs, box given, the robot's legs, height, roll and pitch recorded
    at the turn's stance as the walking policy left it (`demos/combiner_key/assets/walker_stances.json`): the base
    0.306-0.315 m up (settled: 0.272), rear thighs 0.60-0.67 rad (settled: 0.99), front knees -1.56 to -1.65 rad
    (settled: -1.80), nose 2.5 degrees down.
  - The fine-tune ([run](#/week/6/run/20261020T203809_train_seed2_key_force_s2_stage3_walker)): from seed 2's force-aware `model_1605`,
    500 iterations, `--spawn_states` (half the episodes from a recorded stance, the joints jittered +-0.03 rad), the
    stage-2 recipe with the pushes at +-10 N throughout. Training-time key episodes unlocked: 85-97%.
  - Frozen keys: from its own settled stance [102/108](#/week/6/run/20261020T210248_leg_eval_seed1_key_eval_force_s2_stage3_walker) (106 before);
    every episode from a recorded stance at rest (`--eval_spawn_states`):
    [97/108](#/week/6/run/20261020T211501_leg_eval_seed1_key_eval_force_s2_stage3_walkerstarts), against
    [26/108](#/week/6/run/20261020T211611_leg_eval_seed1_key_eval_force_s2_stage2_walkerstarts) for the policy before the fine-tune (45 never
    engaged).
  - The demo with a direct handover (`--handover direct`: the leg policy takes over at once, no crouch), ideal-PD legs:
    box given [0/8](#/week/6/run/20261020T210357_force_inserted_unifplegs_seed1_key_demo_stage3_direct_given) (2 unlocked, 5 fell, all past 120 N); head camera
    and lidar [0/4](#/week/6/run/20261020T210652_force_inserted_unifplegs_seed1_key_demo_stage3_direct_camera) (3 fell); the whole sequence with the arm's
    insertion [0/4](#/week/6/run/20261020T211008_force_unifplegs_seed1_key_demo_stage3_full_camera) (no key seated, F-164).
  - Not the motion: the direct handover with the robot's joint and base velocities zeroed at the switch
    ([run](#/week/6/run/20261020T211833_force_inserted_unifplegs_seed1_key_demo_stage3_direct_still_given), a diagnostic) is 0/8 too (4 fell, the foot never at the pre-contact point, contact peaks
    99-390 N on the way). From the settled stance the same scene turns 4/4 (F-165, seed 1's policy).
- **Scope:** one fine-tune, force-aware only (no matched position-only run); the recorded stances from one walking
  policy on ideal-PD legs; the diagnostic draws a stance per episode; what in the demo's scene breaks a start from
  those stances is not separated -- the box's real colliders round the approach (the lock adapter's block is not in
  the training plant), self-collision off and the payload not simulated are the candidates; the robot's motion at the
  switch is ruled out.
- **Implication:**
  - The stance mismatch of F-165 is trainable: half an hour of fine-tuning took the policy from 26 to 97 of 108 from
    the walking policy's stances, at little cost from its own.
  - The next step for the sequence is the moment of handover: train from the walking policy's states in motion (its
    joint velocities, a gait still settling), with self-collision on in the demo or off in training, and the payload
    simulated in both.


### F-168 — The key policies' collapse on the stock DCMotor legs (F-165: 84 to 3 of 108) was the legs' armature, 0 in the stock group against the trained 2e-4 kg·m², not its 23.5 N·m knee or its speed curve; on every torque model tried, including Unitree's envelope with and without a geared knee, the force-aware policies unlock 82-85 (seed 1), 104-106 (seed 2) and 94-104 (the walker fine-tune) of 108; a 0-10 ms command delay costs seed 1 17 keys and the fine-tune 24-41, and a realistic armature narrows F-166's force-aware lead from 190 against 43 of 216 to 169 against 136

- **Status:** provisional (simulation; frozen policies evaluated off their training conditions, not policies trained on
  each model; the analytic key plant; one stage-1 checkpoint per variant)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "Which leg motor model the key policies hold on". The frozen
  key set (108 keys, F-166) with only the legs' model changed (`unifp_train/leg_actuators.py`), the trained model's
  re-run reproducing F-166 and F-167 exactly (84, 106, 102, 20, 23):

  | legs (all kp 25 / kd 0.5) | force s1 | force s2 | fine-tune | position s1 | position s2 |
  | --- | --- | --- | --- | --- | --- |
  | trained: UniFP ideal PD, URDF limits (23.7 / 45.43 N·m knee), armature 2e-4 | 84 | 106 | 102 | 20 | 23 |
  | knee cap + DCMotor speed curve (23.7 / 23.5, linear to 0 at 30 rad/s), armature 2e-4 | [84](#/week/6/run/20261020T232931_leg_eval_seed1_key_act_force_s1_knee23curve) | 106 | 103 | 20 | 23 |
  | Isaac Lab's stock `DCMotor` (23.5 on every joint, 30 rad/s), armature 0 as loaded | [**3**](#/week/6/run/20261020T233521_leg_eval_seed1_key_act_force_s1_dcmotor_rerun) | 104 | 94 | 20 | 20 |
  | the stock `DCMotor` with armature 2e-4 | [**84**](#/week/6/run/20261020T233521_leg_eval_seed1_key_act_force_s1_dcmotor_armature) | | | | |
  | unitree_rl_lab's GO2HV envelope, the knee as the hip (20.2 / 23.4 N·m) | 85 | 106 | 102 | 22 | 20 |
  | the GO2HV envelope with the knee geared 1.917 (38.7 / 44.9 N·m, 0 at 15.65 rad/s) | [82](#/week/6/run/20261020T225436_leg_eval_seed1_key_act_force_s1_unitree_geared) | 106 | 104 | 21 | 20 |
  | trained + 0-10 ms delay | 67 | 107 | [61](#/week/6/run/20261021T002503_leg_eval_seed1_key_act_force_s2walker_unifp_d2) | 16 | 29 |
  | geared + 0-10 ms delay | 66 | 107 | 80 | 24 | 27 |
  | trained, armature 0.003 / 0.01 / 0.01 with knees 0.037 | 92 / 96 / 81 | 103 / 102 / 88 | 102 / 105 / 106 | 37 / 60 / 69 | 31 / 60 / 67 |
  | geared, armature 0.003 / 0.01 / 0.01 with knees 0.037 | 91 / 92 / 83 | 102 / 101 / 88 | 105 / 107 / 107 | 35 / 65 / 68 | 32 / 58 / 66 |

  The knee cap alone and the speed curve alone also give 84 / 106 / 20 / 23. The run record's `leg_joints_loaded` has
  the stock group's leg armature at 0.0 (it sets none, so the robot file's stays) and every other model's at 2e-4;
  PhysX's velocity limit is the URDF's (30.1 / 30.1 / 15.70 rad/s) in all. On the stock group, seed 1's policy, which
  crouches to 0.21 m and then rears on three legs, sinks instead (0.197 to 0.170 m over 1.2-2.0 s, the support knees
  clipped at 23.5 N·m with 24-32 demanded) and 103 of 108 end at ~2.0 s with the foot striking the key (median
  184 N); seed 2's, its support knees clipped for 0.58 s an episode, still unlocks 104.
- **Scope:**
  - Frozen policies, each evaluated on models it did not train on; a policy trained on a model may rank differently.
  - The armature values are references, not measurements: unitree_rl_lab gives the same GO2HV motor 0.01 kg·m² where it
    drives the H1's joints (`UNITREE_H1_CFG`) and leaves the Go2's at the robot file's 0; 0.037 is 0.01 times 1.917²
    for a geared knee. The geared knee itself is derived from the URDF (`motor_model.GO2_HV_KNEE`), not measured. The
    delay is `motor_model.GO2_LEG_DELAY_PHYSICS_STEPS`' estimate.
  - Why zero armature breaks seed 1's rear-up and not seed 2's is not separated.
  - Joint torques are the last physics substep of each 50 Hz step; the analytic key plant throughout (F-163's scope).
- **Implication:**
  - F-165's "the ranking is conditional on the actuator model" holds, but the parameter is the armature, not the
    torque limits: within the torque models tried at the trained armature, the force-aware policies' results move by
    at most 2 keys.
    F-165's reading of why is corrected; F-166's comparison is conditional on the armature.
  - Before the real dog: identify the legs' armature and the control loop's delay (neither is measured), train with
    both randomised around them, and repeat F-166's comparison there. The torque model to train on can be Unitree's
    envelope with the knee geared, once the knee is checked (`sysid/go2_knee/`, F-169).
  - Say which armature every leg comparison used.

### F-169 — The force-aware key policies crouch to ~0.22 m and then rear the base ~35° nose-up on three legs to reach the key's platform at 0.42 m, holding the standing knees at a median 12-14 N·m with peaks of 28-33 N·m; the pressing leg's joints carry at most 19 N·m; whether the real knee has that margin turns on its reduction, which the simulators disagree on (23.5 N·m in Isaac Lab's and unitree_rl_lab's models, 45.43 in the URDF)

- **Status:** provisional (simulation, the trained leg model; the real knee not measured)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "Which leg motor model the key policies hold on". Joint
  torques logged in the frozen key evaluation on the trained model (`leg_eval.TORQUE_FIELDS`):

  | policy | base before lifting | while the FR foot is up (~10-12 s an episode): pitch, base | largest support knee: median, time above 20 N·m, episode peak median / max | pressing leg's max hip / thigh / knee |
  | --- | --- | --- | --- | --- |
  | [force s1](#/week/6/run/20261020T225436_leg_eval_seed1_key_act_force_s1_unifp) | 0.215 m | 35° nose-up, 0.28 m | 13.9 N·m, 6.7%, 23.2 / 32.8 N·m | 8.0 / 7.5 / 18.9 N·m |
  | [force s2](#/week/6/run/20261020T230912_leg_eval_seed1_key_act_force_s2_unifp) | 0.219 m | 34°, 0.28 m | 12.0, 11.0%, 27.7 / 28.0 | 5.9 / 6.9 / 10.7 |
  | [fine-tune](#/week/6/run/20261020T232518_leg_eval_seed1_key_act_force_s2walker_unifp) | 0.230 m | 26°, 0.30 m | 12.6, 4.7%, 23.9 / 23.9 | 9.8 / 10.4 / 16.3 |
  | [position s1](#/week/6/run/20261020T230252_leg_eval_seed1_key_act_position_s1_unifp) | 0.237 m | 25°, 0.30 m | 12.5, 3.5%, 22.1 / 35.7 | 10.9 / 10.0 / 23.7 |

  With more than 40 N on the foot, seed 1's pressing knee carries a median 1.9 N·m (max 8.8): the leg pushes nearly
  straight, the force through its joints. The key's platform is at ~0.42 m, above the front hips' standing height
  (~0.30 m), so the foot reaches it from above only with the base reared.
- **Scope:** the trained ideal-PD legs (no torque-speed limit); the torques sampled at 50 Hz; the analytic key plant; one
  key height (the real box's).
- **Implication:**
  - If the knee is geared as the URDF says, the standing knees run at about a third of their peak through the rear-up
    and peak at about 70% of it; if it is capped at the hip motor's ~23.5 N·m, they run at over half of it, above
    20 N·m for 0.5-1.2 s an episode, with peaks past it.
    `sysid/go2_knee/` (read-only, LowState `tau_est` with a unit check against the robot's weight) settles which before
    any test on the dog; nothing in it has run yet.
  - A 35° rear-up on three legs with the D1 on the back is a fall risk the simulator has not been asked about (fixed
    mass, no randomisation); a lower key, or a stand the dog can lean on, would remove it.

### F-170 — From the Go2 sport mode's classic stand at rest (feet 41 x 28 cm, 3-4 cm from the trained places), the walker-stance fine-tune unlocks 96-99 of 108 frozen keys at base heights 0.28-0.32 m and 74 at 0.34 m; the policies trained only from their own stance lose 9-34 keys at 0.28-0.32 m and more at 0.34 m, and the seed-2 position-only policy falls in 20-58 of 108

- **Status:** provisional (simulation; a start at rest, not the switch out of sport mode; the classic stand's height not
  measured)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "Which leg motor model the key policies hold on". Lukas
  measured the classic stand's feet at 41 x 28 cm, "almost directly under the shoulders"
  (`demos/combiner_key/classic_stance.py` solves the legs for the feet's sphere centres there, level; its kinematics
  reproduce `leg_cfg.NOMINAL_FEET_B` to 2.5 mm). The trained stance is 44.8 x 34 cm under a 0.272 m base. Every
  frozen key episode starts from the stand at rest, its joints jittered 0.03 rad, on the trained legs:

  | policy | trained stance | 0.28 m | 0.30 m | 0.32 m | 0.34 m |
  | --- | --- | --- | --- | --- | --- |
  | force s1 | 84 | 73 | 75 | 50 | 20 |
  | force s2 | 106 | 91 | 80 | 93 | 65 |
  | fine-tune (F-167) | 102 | [97](#/week/6/run/20261020T234136_leg_eval_seed1_key_classic_force_s2walker_28cm) | [99](#/week/6/run/20261020T234402_leg_eval_seed1_key_classic_force_s2walker_30cm) | [96](#/week/6/run/20261020T234409_leg_eval_seed1_key_classic_force_s2walker_32cm) | [74](#/week/6/run/20261020T234414_leg_eval_seed1_key_classic_force_s2walker_34cm) |
  | fine-tune, geared knee | 104 | | [98](#/week/6/run/20261020T235228_leg_eval_seed1_key_classic_force_s2walker_30cm_unitree_geared) | [97](#/week/6/run/20261020T235434_leg_eval_seed1_key_classic_force_s2walker_32cm_unitree_geared) | |
  | position s1 | 20 | 24 | 24 | 22 | 21 |
  | position s2 | 23 | 12 (50 fell) | 14 (27 fell) | 13 (20 fell) | 7 (58 fell) |

- **Scope:** a start at rest with zero velocities and an empty history, not the robot leaving sport mode (on the dog
  the motors pass from the sport controller to low-level commands, which rima's node does by ramping to its own default
  stance); level ground, the trained legs (no delay, armature 2e-4); the sport mode's stand modelled from two spacings
  and a guessed height.
- **Implication:**
  - The classic stand is close enough to start from for the policy fine-tuned from the walking policy's stops (~0.31 m),
    up to 0.32 m; the base height is the variable to measure (`sportmodestate.body_height`, which the door demo's robot
    program already reports).
  - A real handover should start the leg policy from the classic stand as it is, not ramp to another stance first,
    and the switch out of sport mode itself is still to be simulated.

### F-171 — The key policy's runner for the dog (numpy, no torch) reproduces the training environment: through four recorded Isaac episodes (3,596 steps) its frames match to 1.0e-7, its actions to 1.1e-6 and its joint targets to 1.2e-7 rad; as a ROS 2 node in shadow mode it holds the 50 Hz step (p99 20.2 ms, inference p99 ~1 ms on the PC) and publishes nothing; the dog's own conventions (motor order, IMU frame) are taken from rima's working node and are untested here

- **Status:** confirmed (software, on the PC); not run on the dog
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The leg policy's shadow mode for the dog".
  `demos/combiner_key/robot/`: `export_policy.py` (the walker fine-tune `model_2104` to
  `checkpoints/leg_key_force_s2walker_model_2104.npz`, numpy against the checkpoint on 64 random stacks: actions
  3.3e-6, estimates 1.3e-6), `leg_policy.py`, `shadow.py`, `check_shadow.py`.
  - `check_shadow.py --sim` on a key evaluation with `--leg_shadow_dump 4`
    ([run](#/week/6/run/20261021T013654_leg_eval_seed1_key_shadow_dump_force_s2walker), 102 of 108 as before): the raw
    state mapped to Unitree's joint order by name, then the dog's pipeline from each reset -- frames 1.0e-7 (every
    block), actions 1.1e-6 (|a| up to 4.2), targets 1.2e-7 rad, the waiting stage's command 4.9e-7.
  - The node under ROS 2 against a fake 500 Hz LowState (localhost, domain 77; the classic stand at 0.30 m with a 0.01
    rad sway): [20 s](#/week/6/run/20261021T014808_leg_shadow), 1001 steps, period p99 20.2 ms, state age p99 2.2 ms,
    inference median / p99 0.52 / 1.11 ms; `check_shadow --dog`: frames against `interface.single_obs` 3.5e-18, actions
    against the checkpoint in torch 1.2e-6, the stand from the joints 0.41 x 0.28 m under 0.30 m, the still-robot
    prediction within 0.012 rad RMS (the sway). With the foot asked up
    ([run](#/week/6/run/20261021T014859_leg_shadow)): 1.8e-6, 0.045 rad RMS.
  - `tests/test_leg_shadow.py` (10 tests): the export of a random actor, the frame against `single_obs` on random
    states, Unitree target order, the waiting command, the estimates' decoding, the foot kinematics, the node offline,
    and a source scan: no publisher, no client, no socket write, no torch.
- **Scope:** software on the PC. On the dog nothing has run: the motor order (FR, FL, RR, RL), the gyroscope as the body
  rate and the quaternion as w, x, y, z are rima's node's (`~/rima_summerschool`), which runs Isaac Lab policies on this
  dog; the arm's angles come through the door demo's arm link and `d1_ik.from_servo_deg`; the dog's CPU is not timed.
  Open loop by design: the sport controller holds the robot, so the shadow run shows the policy's response to the real
  observation, not what it would do with the legs.
- **Implication:**
  - A shadow run on the dog (`run_shadow_on_dog.sh`, read on the PC with `check_shadow.py --dog`) can now check what only
    the dog can: the joints' signs and zeros (the stand from the encoders against the tape's 41 x 28 cm), the IMU's frame
    (level, and the pitch's sign when the nose is pushed down), the arm's angles, and the policy's response against a
    still model in the same pose. It also measures the classic stand's base height, which F-170 lacked.
  - The runner is the policy's; driving the legs needs a separate, commanding program with its own safety design, after
    the fine-tune F-168 asks for.
- **Follow-up (2026-10-21):** run on the dog, with the conventions above checked there: [F-173](#f-173).

### F-172 — With the key starting 45 degrees down and needing to turn to 90 (straight down, unlocking at 85), the trained key policies fail zero-shot (1-16 of 108), though the walker fine-tune does turn it in 100: what fails is the force at the vertical end stop, past 120 N in 105 of 108; ending the task layer's path short of the stop and holding 0.3 s instead of 2 s gets it to 33, so the new turn needs retraining, not only a layer change

- **Status:** provisional (simulation; frozen policies off their training turn; the analytic key plant; the unlock at 85
  degrees chosen, not measured)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key turned from 45 to 90 degrees down". The frozen key
  set with `--key_turn_deg 45 90 85` (start, end stop, unlock; degrees below horizontal), keys at their trained
  lock friction and detent:

  | policy | zero-shot | path to 3° short of the stop (`--key_overdrive_deg -3`) | and a 0.3 s hold (`--key_hold_s 0.3`) |
  | --- | --- | --- | --- |
  | force s1 | [16](#/week/6/run/20261021T021850_leg_eval_seed1_key_turn4590_force_s1) (83 past 120 N) | 11 | 9 |
  | force s2 | [1](#/week/6/run/20261021T021855_leg_eval_seed1_key_turn4590_force_s2) (46 never reach the pad) | 2 | |
  | walker fine-tune | [3](#/week/6/run/20261021T021900_leg_eval_seed1_key_turn4590_force_s2walker) (100 unlocked, 105 past 120 N) | 17 | [**33**](#/week/6/run/20261021T022958_leg_eval_seed1_key_turn4590_force_s2walker_od-3_hold03) (55 past 120 N, 16 stalled) |
  | position s1 | 11 | 6 | |
  | position s2 | [12](#/week/6/run/20261021T022137_leg_eval_seed1_key_turn4590_position_s2) (94 unlocked, 84 past 120 N) | 10 | [27](#/week/6/run/20261021T023004_leg_eval_seed1_key_turn4590_position_s2_od-3_hold03) |

  For the walker fine-tune zero-shot, every first contact past 120 N is while engaged, with the key at a median 90.8
  degrees: at the stop, in the layer's hold (79 of 105) or the press, with 15-20 N commanded and a median peak of
  142 N. It unlocks a median 2.0 s after engaging. The base rears about as on the trained turn (24 against 26 degrees
  nose-up, F-169).
- **Scope:** zero-shot only, no policy trained on this turn; the end stop's 90 degrees and the unlock's 85 are this
  study's choices; why the vertical stop overloads the foot (the pad's face turned sideways under a foot that pushes
  down, against a rigid stop) is a reading of where and when, not a test.
- **Implication:**
  - The policies can drive the key into the new range: the walker fine-tune unlocks 100 of 108. But they do it with
    the trained turn's pressing, which at a vertical stop exceeds the damage force. A fine-tune on the new turn
    (`--key_turn_deg 45 90 85` in training, from the current checkpoints) is the way to make it work.
  - For a cam lock that stays where it is turned, the layer's 2 s hold is not needed. A retrained policy should be
    trained with the short hold too.
  - If the policy is retrained anyway, it is the point to add F-168's delay and armature randomisation.

### F-173 — On the real dog, the key policy's shadow runner holds the 50 Hz step (period p99 20.7 ms, inference 3.1 ms on its CPU) and builds the training code's observation from the robot (frames to 1e-17, joint targets within 0.013-0.078 rad RMS of a still model's); pushing the nose and the front-right shoulder confirms the conventions F-171 took from rima's node: pitch positive nose down, roll positive right side down, FR first, the quaternion as w, x, y, z; the sport mode's stand from the encoders is 38.7 x 31.7 cm under 0.32-0.33 m, 2-4 cm off the tape

- **Status:** confirmed (one 90 s run on the dog, open loop, with the method stated); the stand's spacing against the
  tape is unresolved
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key policy in shadow mode on the real dog";
  [record](#/week/6/run/20261021T044358_leg_shadow) (`summary.json`, `check_shadow.json`, the arm link's log);
  [`figures/leg_shadow_dog_20261021.png`](week_06/figures/leg_shadow_dog_20261021.png)
  (`demos/combiner_key/robot/plot_shadow.py`). `run_shadow_on_dog.sh --policy
  checkpoints/leg_key_force_s2walker_model_2104.npz --seconds 90` beside P2Dingo's containers, the robot in sport mode's
  stand, the D1 folded and observed only; then `check_shadow.py --dog` on the PC (without the `.pt`, which is on the
  training machine: no torch comparison).
  - 4,472 steps, one reset; period median / p99 20.0 / 20.7 ms; `/lowstate` age p99 1.6 ms; inference median / p99
    3.1 / 4.6 ms; the arm measured in every step.
  - Frames against `interface.single_obs` from the recorded raw state 1.4e-17; the dog's numpy actions against the PC's
    2.7e-15; against a still robot in the first second's pose, targets 0.013-0.078 rad RMS per joint (max 0.28, FR
    calf, during the pushes) and the estimator's outputs as the still model's.
  - Nose pushed down: pitch -1.7 → +3.9°; the front knees' `tau_est` 4.0 → 11.4 (FR) and 9.1 N·m (FL). FR shoulder
    pressed three times: roll -1.8..-2.4 → +1.6..+2.0°; the FR knee 6.3-7.1 → 12.4-13.2 N·m (the others at most
    9.0), the FR foot sensor 38-45 → 77-84, the diagonal RL's 41-44 → 24-28. The policy's roll and pitch from the
    quaternion equal the firmware's `rpy` in every step; the gyroscope's x and y correlate with their rates at 0.91 and
    0.94.
  - The stand from the joints (`foot_local`, the URDF's numbers): 38.7 x 31.7 cm in every quiet window; the base 0.333 m
    above the feet before the pushes, 0.319 m after; the rear-left foot 18.3-19.4 cm from the centre line, the others
    13.7-16.0 cm. Lukas's tape: 41 x 28 cm.
- **Scope:** open loop: sport mode held the robot and the policy's targets never acted. One run, one robot, one
  operator's pushes, level-looking floor not checked (roll -2.1° at the start). The runner's waiting stage only (the
  foot on its footfall); the `--foot_goal` run is not done. The foot sensors are raw units. The stand is computed
  from encoders and URDF kinematics, not measured: the 2-4 cm against the tape could be the stand sport mode left, the
  tape, or a hip's zero (~0.1 rad at the rear left).
- **Implication:**
  - The dog-side observation needs no change before the policy drives the legs: what remains is the policy (F-168's
    fine-tune) and the handover, not the interface.
  - The real classic stand sits at 0.32-0.33 m, the top of the range F-170 found the walker fine-tune holds (96 of 108
    at 0.32 m, 74 at 0.34). F-170's start should be re-run at 0.32 and 0.33 m with this spacing.
  - From that stand, with an empty history, the policy's targets would fold three knees (would-be knee torques FR +0.5,
    FL -3.0, RR -8.5, RL -7.6 N·m where sport mode holds +6.4 to +7.7), which reads as its drop into the crouch at an
    episode's start (F-169). A handover would begin with that drop; it should be simulated from this stand before the
    legs are driven.
- **Update (2026-10-21, the second run, [F-174](#f-174)):** sport mode's stand is not the same each time: 38.4 x 27.8 cm
  under 0.337 m then, so the 31.7 cm width above was the stand, not the encoders; the rear-left foot was again 2.3-3.3 cm
  further out than the others, so a hip zero at the rear left is still possible.

### F-174 — A second shadow run on the dog, with the front-right foot asked 10 cm up and ahead, repeats F-173's agreement (frames to 4e-18, targets within 0.004-0.069 rad RMS of a still model's); open loop, the policy cycles the FR knee's target 0.42 rad at 4.3 Hz, as a still model does; sport mode's stand differs between stands (38.4 x 27.8 cm under 0.337 m this time); and the shadow loop stalls once for 0.18 s at 30 s and once for 0.36-0.40 s at 60 s in both runs

- **Status:** confirmed (two 90 s runs on the dog, open loop); the stall's cause not found
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key policy in shadow mode on the real dog", the second
  run; [record](#/week/6/run/20261021T050842_leg_shadow) (`d1dogshadow --foot_goal 0.30 -0.17 0.12`, the robot not
  touched; `check_shadow.json`, the arm link's log);
  [`figures/leg_shadow_dog_20261021_foot_goal.png`](week_06/figures/leg_shadow_dog_20261021_foot_goal.png).
  - 4,475 steps, one reset, period p99 20.6 ms, inference median 3.1 ms; frames against `interface.single_obs`
    3.5e-18, numpy dog against PC 2.9e-15; against a still robot in the first second's pose, targets 0.004-0.069 rad
    RMS per joint and the estimator's outputs equal to the third decimal.
  - The FR calf target after 20 s: 0.42 rad peak to peak, dominant 4.3 Hz; the still model's (`check_shadow`'s
    `static_prediction`, the policy fed its own actions with the robot still): 0.40 rad, the same rate of reversals.
    The PC's fake LowState at a 0.30 m stand (F-171's lift run) put the FR knee target a median 0.22 rad off.
  - The stand from the encoders: feet 38.4 x 27.8 cm, the base 0.337 m above them, pitch -1.35°, roll -0.4°; the
    rear-left foot 15.9 cm from the centre line, the others 12.6-13.6. Knee `tau_est` 5.8, 8.0, 11.0, 11.7 N·m.
  - Steps over 30 ms, both runs: 30.16-30.17 s (0.178-0.185 s) and 60.34-60.37 s (0.359-0.402 s) after the first
    LowState; no others. The runs started 25 min apart.
- **Scope:** open loop; the lifting regime is the waiting stage's command with a raised footfall, not the key task.
  The cycle is the policy's response to a foot that never lifts; closed loop the foot would move, so it shows nothing
  about the leg's real motion. The stall is seen, not explained: it could be the node (it keeps every step in Python
  lists) or the LowState stream (the node does not record a step whose state is over 0.1 s old).
- **Implication:**
  - Shadow mode has checked what it can on a robot that stands: the interface is the training code's in both regimes.
    The open-loop cycle is a reminder that the shadow targets are not a preview of motion.
  - Before any program commands the legs, find the 30 s / 60 s stall (a longer run would show whether it recurs and
    grows) and give the program a watchdog for a missed step.
  - Compare the stand with the tape again in the same session: sport mode's stand changes from one to the next.
- **Update (2026-10-21, [F-175](#f-175)):** the stall is the node's own 30 s save of its whole record inside the loop,
  not the LowState stream; fixed with a per-second flush of the new steps.

### F-175 — The shadow loop's stalls on the dog (0.18 s at 30 s, 0.4 s at 60 s, F-174) were the node itself: every 30 s its loop saved the whole record so far (compressed, with a per-step summary), 150 ms per 1,500 steps on the dog's CPU; flushing only each second's new steps instead costs at most 2.9 ms there, and on the PC the fixed loop's longest step is 20.4 ms over 65 s

- **Status:** confirmed (cause reproduced on the PC and timed on the dog's CPU; the fixed node not yet run on the dog)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The shadow loop's stalls".
  - `save()` on the dog (`p2dingo-web`, the second dog run's rows): 150 / 302 / 449 ms at 1,500 / 3,000 / 4,500 steps,
    70% of it `np.savez_compressed`, a quarter the summary; the stalls were 178-185 and 359-402 ms (the save plus the
    next step's wait). On the PC: 49 / 98 / 147 ms.
  - The unfixed node on the PC against a fake 500 Hz LowState: gaps of 40 ms at 29.6 s and 83 ms at 59.7 s
    ([run](#/week/6/run/20261021T051927_leg_shadow)), so the cause is in the node.
  - Fixed (`shadow.py`: `flush()` each second right after a step, the full save once at the end; SIGTERM and SIGHUP end
    the loop so the final save runs; `run_shadow_on_dog.sh` waits 10 s before SIGKILL): 65 s on the PC, period p99
    20.2 ms, max 20.4 ms, no gap ([run](#/week/6/run/20261021T052125_leg_shadow)); SIGTERM at 8 s saved the whole record
    ([run](#/week/6/run/20261021T052245_leg_shadow)); SIGKILL at 5.5 s left 5 parts, 252 contiguous steps. The flush on
    the dog: median 2.4 ms, max 2.9 ms over 89 flushes, flat over the run. `tests/test_leg_shadow.py`: the parts
    concatenated are the saved record.
- **Scope:** the node's own loop. The fix is in the repo, not yet on the dog (`d1update`); the dog's step timing with it
  is measured only in pieces (the flush in a container, inference in the runs). The LowState stream showed no gap of its
  own in either dog run, which does not rule one out on a longer run.
- **Implication:**
  - Rerun a shadow run over 60 s on the dog after `d1update`: no step should exceed ~25 ms.
  - The rule for any program that commands the legs: nothing in its control loop may grow with the run's length
    (records, logs, plots), and it needs a watchdog for a missed step whatever the cause.

### F-176 — The turn Lukas asked for is 45 degrees, from horizontal (the platform up) to 45 down, unlocking past 30; every key policy so far trained on twice that, from 45 up (a misreading since 2026-10-21 morning). On his turn the force-aware seed-2 policy unlocks 108 of 108 zero-shot, but on the realistic legs and from the classic stand together 52, and with the leg delay the dog's measured latency implies, the policies need a fine-tune before they drive the legs

- **Status:** provisional (simulation; frozen policies off their training turn; the analytic key plant)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The turn Lukas meant, and what his lever can take". Lukas's
  request, 2026-10-21: "turn it 45 degrees from horixontal (pad facing up) to 45 degrees below horizontal clockwise. the
  door unlocks when the key turns past the 30 degree mark"; later "the 0 to -45" twice. Frozen key set with
  `--key_turn_deg 0 45 30` (108 keys; "realistic" is the geared knee, armature 0.01 / knees 0.037 and a 0-10 ms delay;
  the classic stand the tape's 41 x 28 cm at 0.30 m):

  | policy | trained legs | + delay | geared + armature | realistic | classic stand | classic + realistic |
  | --- | --- | --- | --- | --- | --- | --- |
  | force s2 | [**108**](#/week/6/run/20261021T043112_leg_eval_seed1_key_turn045_force_s2) | 105 | 97 | [89](#/week/6/run/20261021T044222_leg_eval_seed1_key_turn045_force_s2_real) | 84 | [52](#/week/6/run/20261021T044238_leg_eval_seed1_key_turn045_force_s2_classic30_real) |
  | walker fine-tune | [79](#/week/6/run/20261021T043118_leg_eval_seed1_key_turn045_force_s2walker) | 43 | 69 | 48 | 44 | 33 |
  | position s2 | [53](#/week/6/run/20261021T043348_leg_eval_seed1_key_turn045_position_s2) | 41 | 86 | 81 | 33 (25 fell) | 33 |
  | force s1 / position s1 | [94](#/week/6/run/20261021T043107_leg_eval_seed1_key_turn045_force_s1) / [23](#/week/6/run/20261021T043341_leg_eval_seed1_key_turn045_position_s1) | | | | | |

  On the trained turn, the walker fine-tune with the realistic legs unlocks [78](#/week/6/run/20261021T045028_leg_eval_seed1_key_act_force_s2walker_real),
  from the classic stand with them [49](#/week/6/run/20261021T045033_leg_eval_seed1_key_act_force_s2walker_classic30_real); with a
  0-5 ms delay alone [101](#/week/6/run/20261021T045038_leg_eval_seed1_key_act_force_s2walker_unifp_d1) (102 without, 61 at 0-10 ms).
  The delay's damage is on the key (the foot settling on it, pressing, lifting off), no falls. The dog's own latency
  before a command leaves is 3.4 ms median, 5.3 ms p99 (F-173's run; the motors' part unmeasured).
- **Scope:** zero-shot evaluations of policies trained on the longer turn; one seed per variant on Lukas's turn; the
  delay's range an estimate whose low end the dog's measurement rules out.
- **Implication:**
  - F-163 to F-172 hold for the turn they state (45 up to 45 down), which is not Lukas's task. The shorter turn is
    easier as trained (108 of 108) but not as deployed.
  - Before the policy drives the legs, fine-tune it on Lukas's turn with the delay, the armature, the geared knee and
    the sport mode's stand (F-179 when it reports). The code's default turn stays the trained one; `--key_turn_deg 0 45 30`
    sets Lukas's.

### F-177 — On a light lock (0.2-0.5 N·m, as Lukas's lever needs) the force-aware seed-2 policy turns the key with a peak of at most 37 N when the task layer stops 5 degrees short of the end stop and holds 0.3 s, but on the realistic legs from the sport mode's stand its peaks reach 144 N (p90) and 342 N, above what the 2.5 mm PLA platform is estimated to take; a lower force cap changes nothing, the excess is impact

- **Status:** provisional (simulation; the plant's contact stiffness sets the peaks; the lever's strength estimated, not tested)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The turn Lukas meant, and what his lever can take". Lukas's
  turn with `--key_eval_lock_nm 0.2 0.3 0.5 --key_eval_detent_nm 0 0.2`; "soft" `--key_overdrive_deg -5 --key_hold_s 0.3`;
  "cap" `--key_max_force_n 8 15`:

  | policy, setting | unlocked, no damage (120 N), foot down | peak median / p90 / max | on the stop, median | past 50 N |
  | --- | --- | --- | --- | --- |
  | force s2, light | [106](#/week/6/run/20261021T045510_leg_eval_seed1_key_turn045_force_s2_light) | 33 / 41 / 43 N | 24 N | 0 |
  | force s2, light, soft | [106](#/week/6/run/20261021T045516_leg_eval_seed1_key_turn045_force_s2_light_soft) | 22 / 29 / 37 N | 15 N | 0 |
  | force s2, light, soft, cap | [106](#/week/6/run/20261021T045521_leg_eval_seed1_key_turn045_force_s2_light_soft_cap) | the same | 15 N | 0 |
  | force s2, light, soft, cap, classic stand + realistic legs | [72](#/week/6/run/20261021T050258_leg_eval_seed1_key_turn045_force_s2_light_soft_cap_classic30_real) (21 damaged, 2 fell) | 32 / 144 / 342 N | 18 N | 36 |
  | walker fine-tune, light / soft / soft + cap | [82](#/week/6/run/20261021T045744_leg_eval_seed1_key_turn045_force_s2walker_light) / 89 / 90 | 34-42 / 136-147 / 208-237 N | 23-29 N | 39-44 |
  | position s2, light / soft | [102](#/week/6/run/20261021T050022_leg_eval_seed1_key_turn045_position_s2_light) / [106](#/week/6/run/20261021T050031_leg_eval_seed1_key_turn045_position_s2_light_soft) | 41 / 70 / 144; 38 / 48 / 54 N | 24-26 N | 24; 5 |

  The lever, from Lukas's STL (`demos/combiner_key/assets/combinerbox_lever_v3.stl`): the bar 15 x 20 mm on a 10 mm
  shaft; the platform a 2.5 mm plate, 35 mm wide and 100 mm long, beside the bar, which the layer aims at ~18 mm from
  its root. Rough printed-PLA estimates: the plate ~70-160 N there (less at its edge), the shaft ~3-6 N·m in torsion,
  i.e. force x ~0.09 m on the end stop. Lukas's lock turned under a half-full 1 L bottle resting on the platform (~5 N
  at ~0.1 m, at most ~0.5 N·m).
- **Scope:** the peaks are the analytic plant's (sphere against boxes at 8000 N/m, physics rate); a real rubber foot on
  PLA may differ either way; the strength figures are textbook ranges for printed PLA, not a test of Lukas's part.
- **Implication:**
  - In the trained conditions the soft end is enough; on the dog's conditions it is not: train the press limit in
    (`--key_press_limit_n`, F-179) rather than cap the command.
  - Test with a sacrificial or stronger lever first: a 4-5 mm platform is ~3x stronger in bending, a metal pin through
    the shaft protects it on the stop.

### F-178 — From the dog's measured stand (38.7 x 31.7 cm under 0.32-0.33 m, F-173) the trained key policies do worse than from the tape's: on the trained turn the walker fine-tune unlocks 84-85 of 108, on Lukas's turn the best 52, and on his turn with the realistic legs 15-24 with 8-17 falls

- **Status:** provisional (simulation; the stand solved symmetric, the rear-left foot's 3-4 cm offset not in it)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The turn Lukas meant, and what his lever can take".
  `demos/combiner_key/classic_stance.py --front_back 0.387 --left_right 0.317 --heights 0.32 0.33 --out_dir
  demos/combiner_key/assets/dog_stand_20261021` (feet 4.4 cm front, 2.0 cm rear from the trained places), every frozen
  episode from it at rest:

  | policy | trained turn, 0.32 / 0.33 m | Lukas's turn, 0.32 / 0.33 m | Lukas's turn, 0.33 m, realistic legs |
  | --- | --- | --- | --- |
  | walker fine-tune | [85](#/week/6/run/20261021T050835_leg_eval_seed1_key_dogstand_force_s2walker_32cm) / [84](#/week/6/run/20261021T050840_leg_eval_seed1_key_dogstand_force_s2walker_33cm) | [40](#/week/6/run/20261021T051645_leg_eval_seed1_key_dogstand_turn045_force_s2walker_32cm) / [32](#/week/6/run/20261021T051651_leg_eval_seed1_key_dogstand_turn045_force_s2walker_33cm) | [24](#/week/6/run/20261021T051927_leg_eval_seed1_key_dogstand_turn045_force_s2walker_33cm_real) (8 fell) |
  | force s2 | [73](#/week/6/run/20261021T050556_leg_eval_seed1_key_dogstand_force_s2_32cm) / [69](#/week/6/run/20261021T050827_leg_eval_seed1_key_dogstand_force_s2_33cm) | [35](#/week/6/run/20261021T051359_leg_eval_seed1_key_dogstand_turn045_force_s2_32cm) / [52](#/week/6/run/20261021T051404_leg_eval_seed1_key_dogstand_turn045_force_s2_33cm) | [15](#/week/6/run/20261021T051622_leg_eval_seed1_key_dogstand_turn045_force_s2_33cm_real) (17 fell) |
  | force s1 | 28 / 36 | | |
  | position s1 / s2 | 22 / 24; 17 (10 fell) / 18 | | |
- **Scope:** at rest, not the switch out of sport mode; one measured stand; the trained legs except where stated.
- **Implication:** the start from sport mode's real stand belongs in training (F-179 trains from 40 such stands).

### F-179 — Fine-tuned for 1000 iterations on Lukas's turn with his light lock, a 30 N press limit, the geared-knee legs with a 5-15 ms delay and armature drawn per environment, and most episodes started from sport-mode stands, the force-aware key policy unlocks 100-101 of 108 from the dog's measured stand on those legs with nothing past 60 N (peak median 15 N, max 35 N), and holds 95-105 across delays of 0-15 ms, armature 0.003-0.02 kg·m², three motor models and the taped stand; it fails at a 20 ms delay (65) and at UniFP's 2e-4 armature (2); the matched position-only fine-tune ends at 27-30

- **Status:** provisional (simulation; one seed per variant; the analytic key plant)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The fine-tune on Lukas's turn". Both variants from seed 2's
  finals, 1000 iterations each in parallel (~6.5 s an iteration):
  [force](#/week/6/run/20261021T053626_train_seed2_key_ft045_force_s2) (`model_1605` to `model_2604`),
  [position](#/week/6/run/20261021T053727_train_seed2_key_ft045_position_s2) (`model_1614` to `model_2613`). The frozen key set on
  Lukas's turn with a light lock (0.2 / 0.35 / 0.5 N·m x detent 0 / 0.15), the layer ending 5 degrees short of the stop
  with a 0.3 s hold, damage at 60 N; "dog" is the measured stand (F-173) at rest on the geared knee, armature 0.01 /
  knees 0.037 and a 5-15 ms delay; "nominal" the trained-on legs of F-166 from the settled stance:

  | checkpoint | dog, 0.33 m | dog, 0.32 m | nominal |
  | --- | --- | --- | --- |
  | force-aware before (`model_1605`) | [14](#/week/6/run/20261021T073527_leg_eval_seed1_key_ft045eval_base_force_dog33) (30 fell) | [25](#/week/6/run/20261021T073739_leg_eval_seed1_key_ft045eval_base_force_dog32) (24 fell) | [106](#/week/6/run/20261021T073748_leg_eval_seed1_key_ft045eval_base_force_nominal) |
  | force-aware at 2000 | [25](#/week/6/run/20261021T074205_leg_eval_seed1_key_ft045eval_ftmid_force_dog33) (78 past 60 N) | [27](#/week/6/run/20261021T074332_leg_eval_seed1_key_ft045eval_ftmid_force_dog32) | [3](#/week/6/run/20261021T074337_leg_eval_seed1_key_ft045eval_ftmid_force_nominal) |
  | **force-aware final** (`model_2604`) | [**101**](#/week/6/run/20261021T073137_leg_eval_seed1_key_ft045eval_ft_force_dog33): 0 past 60 N, 0 fell, peak 15 / 24 / 34 N (median / p90 / max) | [**100**](#/week/6/run/20261021T073142_leg_eval_seed1_key_ft045eval_ft_force_dog32): 0, 0, 16 / 24 / 35 N | [0](#/week/6/run/20261021T073147_leg_eval_seed1_key_ft045eval_ft_force_nominal) (102 past 60 N) |
  | position-only before (`model_1614`) | [7](#/week/6/run/20261021T073852_leg_eval_seed1_key_ft045eval_base_position_dog33) (24 fell) | [15](#/week/6/run/20261021T074017_leg_eval_seed1_key_ft045eval_base_position_dog32) | [106](#/week/6/run/20261021T074102_leg_eval_seed1_key_ft045eval_base_position_nominal) |
  | position-only at 2000 | [86](#/week/6/run/20261021T074507_leg_eval_seed1_key_ft045eval_ftmid_position_dog33) | [94](#/week/6/run/20261021T074613_leg_eval_seed1_key_ft045eval_ftmid_position_dog32) | [105](#/week/6/run/20261021T074637_leg_eval_seed1_key_ft045eval_ftmid_position_nominal) |
  | position-only final (`model_2613`) | [30](#/week/6/run/20261021T073437_leg_eval_seed1_key_ft045eval_ft_position_dog33) (75 past 60 N) | [27](#/week/6/run/20261021T073442_leg_eval_seed1_key_ft045eval_ft_position_dog32) | [0](#/week/6/run/20261021T073447_leg_eval_seed1_key_ft045eval_ft_position_nominal) |

  Around "dog, 0.33 m", one thing changed at a time (the force-aware final; the position-only checkpoint at 2000):

  | change | force-aware final | position-only at 2000 |
  | --- | --- | --- |
  | delay 0-5 / 5 / 15 ms | [105](#/week/6/run/20261021T091755_leg_eval_seed1_key_ft045sweep_ft_force_delay0to5) / [103](#/week/6/run/20261021T091800_leg_eval_seed1_key_ft045sweep_ft_force_delay5) / [95](#/week/6/run/20261021T091806_leg_eval_seed1_key_ft045sweep_ft_force_delay15), max 36-38 N | 96 / 93 / 77, max 35 / 316 / 345 N |
  | delay 20 ms | [65](#/week/6/run/20261021T092151_leg_eval_seed1_key_ft045sweep_ft_force_delay20) (24 past 60 N, 3 fell, max 391 N) | 77 (1 fell) |
  | armature 0.003 / knees 0.011; 0.02 / 0.073 | [102](#/week/6/run/20261021T092159_leg_eval_seed1_key_ft045sweep_ft_force_armlow); [98](#/week/6/run/20261021T092207_leg_eval_seed1_key_ft045sweep_ft_force_armhigh) (max 32; 42 N) | 88 (7 fell); 91 |
  | armature 2e-4 (UniFP's, as F-166 trained) | [**2**](#/week/6/run/20261021T092648_leg_eval_seed1_key_ft045sweep_ft_force_arm2e-4) (95 past 60 N, max 333 N) | 67 |
  | ideal-PD legs (armature 0.01 / 0.037, delay kept) | [97](#/week/6/run/20261021T092655_leg_eval_seed1_key_ft045sweep_ft_force_idealpd) (max 36 N) | 84 |
  | GO2HV with the knee as the hip | [100](#/week/6/run/20261021T092700_leg_eval_seed1_key_ft045sweep_ft_force_unitreeknee) (max 37 N) | 98 |
  | the taped stand, 41 x 28 cm at 0.30 m | [95](#/week/6/run/20261021T093004_leg_eval_seed1_key_ft045sweep_ft_force_tape30) (one at 60 N) | 101 (max 214 N) |

  Training-time (sampled at reset, the training conditions): the force-aware run's mean peak per key episode fell from
  65-96 N in its first 200 iterations to 40-44 N, with 2-5% past 120 N; the position-only run's from 145-175 N to 59-67 N
  by ~2000, then rose again to 85-98 N (15-28% past 120 N).
- **Scope:**
  - One seed per variant, one fine-tune. The position-only checkpoint at 2000 is reported because its final regressed;
    picking it by these numbers would be choosing on the test set, so the matched comparison is the finals (101 / 100
    against 30 / 27).
  - The analytic key plant, its contact stiffness setting the peaks; Lukas's lock from one bottle test (0.15-0.6 N·m
    trained, 0.2-0.5 evaluated); the stand at rest, not the switch out of sport mode; the delay's motor part
    unmeasured (the dog-side part 3.4 ms median, 5.3 ms p99, F-173).
  - The "nominal" failure is the armature alone: the same legs with 2e-4 give 2, with 0.01 give 97 on the ideal motor model.
- **Implication:**
  - In simulation the fine-tuned force-aware policy is a candidate for the dog: gentle (no contact past 42 N in any
    condition within the training ranges but one taped-stand episode at 60 N) and robust within them. Its limits are the loop's total delay (degrades past
    15 ms) and an armature near zero, which a geared motor does not have.
  - Next on the dog: its shadow run (`checkpoints/leg_key_force_ft045_model_2604.npz`), measuring the motors'
    command-to-motion delay, then a commanding program with its own safety design and a sacrificial lever.
  - On these conditions the force-aware formulation is the better leg controller again, as in F-166, now with the
    realistic legs and Lukas's turn.

### F-180 — A general force-aware foot-press policy -- trained on flat plates anywhere the front-right foot reaches (0.13-0.30 m from its thigh joint, facing it within 60 degrees, rigid or sprung, 3-35 N profiles) plus UniFP's free-space goals with pushes, on the realistic legs and sport-mode stands -- holds 197 of 288 frozen presses from the dog's stand with the force's median error 2.3 N and 5 presses past 60 N, against 100 for the matched position-only policy; a second seed with the final recipe reproduces it (186 against 58); never trained on the key, it unlocks 107 of 108 keys on F-179's evaluation, more than the fine-tuned key specialist (101)

- **Status:** provisional (simulation; two seeds; analytic plates; the layer told the plate by an estimate)
- **Week:** 6
- **Date:** 2026-10-22
- **Evidence:** [Week 6 log, 2026-10-21/22](week_06/notes.md), "A general foot-press policy". `--task leg --leg_object
  press`; both variants from F-179's finals. The frozen set v2: 48 plates (12 tilted, 12 walls, 12 level facing up, 12
  facing down) x rigid / sprung 1,000 N/m x 8 / 16 / 28 N, held 3 s; success is the force within max(3 N, 20%) over the
  hold's last 2 s with no slip, fall or damage (120 N). The realistic legs (geared knee, armature 0.01 / 0.037, 5-15 ms
  delay); "dog" from the measured stand at 0.33 m.

  | final | dog | settled | force bias / MAE | past 60 N | slide set (96) | key zero-shot, 0.33 / 0.32 m |
  | --- | --- | --- | --- | --- | --- | --- |
  | force-aware, round 4 (seed 3, `model_5999`) | [197](#/week/6/run/20261021T165637_leg_eval_seed1_press_r4_force_5999_real) | [207](#/week/6/run/20261021T165640_leg_eval_seed1_press_r4_force_5999_real_settled) | -0.9 / 2.3 N | 5 | [56](#/week/6/run/20261021T170129_leg_eval_seed1_press_r4_force_5999_slide) | [107](#/week/6/run/20261021T170829_leg_eval_seed1_press_r4_force_5999_key_dog33) / [107](#/week/6/run/20261021T171634_leg_eval_seed1_press_r4_force_5999_key_dog32), peak median 7 N |
  | position-only, round 4 | [100](#/week/6/run/20261021T165752_leg_eval_seed1_press_r4_position_5999_real) | [86](#/week/6/run/20261021T165757_leg_eval_seed1_press_r4_position_5999_real_settled) | 0.0 / 5.6 N | 11 | [30](#/week/6/run/20261021T180258_leg_eval_seed1_press_r4_position_5999_slide_b) | [72](#/week/6/run/20261021T171232_leg_eval_seed1_press_r4_position_5999_key_dog33) / [69](#/week/6/run/20261021T172039_leg_eval_seed1_press_r4_position_5999_key_dog32) |
  | force-aware, replication (seed 4, `model_6003`) | [186](#/week/6/run/20261021T231704_leg_eval_seed1_press_rep2_force_6003_real) | [203](#/week/6/run/20261021T231707_leg_eval_seed1_press_rep2_force_6003_real_settled) | +1.4 / 2.5 N | 23 | [50](#/week/6/run/20261021T231713_leg_eval_seed1_press_rep2_force_6003_slide) | [104](#/week/6/run/20261021T232049_leg_eval_seed1_press_rep2_force_6003_key_dog33) / [103](#/week/6/run/20261021T232257_leg_eval_seed1_press_rep2_force_6003_key_dog32) |
  | position-only, replication (`model_6012`) | [58](#/week/6/run/20261021T231849_leg_eval_seed1_press_rep2_position_6012_real) | [63](#/week/6/run/20261021T231855_leg_eval_seed1_press_rep2_position_6012_real_settled) | -1.6 / 6.5 N | 63 | [24](#/week/6/run/20261021T231900_leg_eval_seed1_press_rep2_position_6012_slide) | [0](#/week/6/run/20261021T232052_leg_eval_seed1_press_rep2_position_6012_key_dog33) / [1](#/week/6/run/20261021T232304_leg_eval_seed1_press_rep2_position_6012_key_dog32) |

  Across eight changes to the legs and stance, one at a time (delay 0-5 / 15 / 20 ms, armature 0.003 and 0.02, the
  ideal-PD legs, the knee as the hip, the taped stand), the force-aware finals hold 143-229 (round 4) and 126-218
  (replication), the position-only ones 43-95 and 19-78; 15 ms of delay costs the force-aware policy 31 and 7 presses,
  20 ms 54 and 60, the lowest armature 50 and 37. Zero-shot the F-179 key policy holds [0 of 288](#/week/6/run/20261021T115222_leg_eval_seed1_press_r2_keyft_force_real).
  Against the command the force-aware policy is on the line for rigid and sprung plates alike (`figures/foot_press_eval_r4.png`);
  the position-only one over-presses light commands and under-presses heavy ones, as an impedance does.
- **Scope:**
  - Simulation. The plates are analytic penalty contacts on the foot sphere, knee and head (the thighs and calves pass
    through them); until 10:55 on 2026-10-22, the finals' plant, a plate had a face and no edges. One frozen set of
    288 presses and one of 96 slides; one checkpoint per seed, the finals, not picked.
  - Round 4's lineage ran ~3400 iterations with the layer changed three times on the way (rounds 1-4, all recorded);
    the replication is the final recipe in one go.
  - The weak kind is plates facing down (22 and 34 of 72): pushing up, the foot slips off narrow plates or under-presses.
    The base rears ~30 degrees through a press. The stand is at rest, not the switch out of sport mode.
- **Implication:**
  - UniFP's formulation, given contact during training and a layer that finds the plate by the foot stalling, makes a
    general pressing controller of the front-right leg: one policy for walls, tilted panels, pedals and overhead plates
    in its reach, and the key besides. The force-aware variant is the better one again, by ~2x on plates and on the key.
  - The candidate for the dog is round 4's force-aware final, `checkpoints/leg_press_force_r4_model_5999.npz` (shadow
    mode first, as F-179's). It needs a task layer on the dog (none exists): the estimate of the plate, the approach, the
    stall detection and the profile, and a force cap by direction (F-182).

### F-181 — Driving a UniFP-style policy to press needs four things of its task layer, each found by a failure: a contact goal that follows the foot into the surface (held fixed, the target `goal + (external + command) / k` is a 400 N/m spring and a 1,000 N/m plate gets 71% of the command), a first ramp that starts at the force the target already asked for (starting from the touch force, the target jumped back by the foot's lag and 116 of 288 presses lost the plate), contact called by a stall only near the expected face with the foot on its line, and an outer loop on the foot's miss split by direction along the plate (integrated everywhere it cancels a tangential command: 0.43 N held of 4; off, the foot holds 3.0 N but slides 3 mm of 4 cm; split, 3.05 N and the full 4 cm)

- **Status:** provisional (simulation; the analytic plate; the numbers are round 4's and round 3's checkpoints)
- **Week:** 6
- **Date:** 2026-10-22
- **Evidence:** [Week 6 log, 2026-10-21/22](week_06/notes.md), "A general foot-press policy" (rounds 1-4 and the slide
  set). The spring: derived (`foot_press.PressLayer`), seen in round 1b's first iterations and removed before any
  evaluation. The ramp: round 3's [`model_4000`](#/week/6/run/20261021T135838_leg_eval_seed1_press_r3_force_4000_real)
  (116 slips, 42 of them within the force tolerance), after the fix round 4's [`model_4400`](#/week/6/run/20261021T143624_leg_eval_seed1_press_r4_force_4400_real)
  (3 slips). The stall: round 1, contact called in the air in every episode (stopped at ~50 iterations). The loop, on round 4's
  final on the slide set: [integrated everywhere](#/week/6/run/20261021T170129_leg_eval_seed1_press_r4_force_5999_slide),
  [off](#/week/6/run/20261021T181441_leg_eval_seed1_press_r4_force_5999_slide_nohold),
  [split](#/week/6/run/20261021T181955_leg_eval_seed1_press_r4_force_5999_slide_hybrid); the replication, trained with
  the split loop, [3.1 N and 3.6 cm](#/week/6/run/20261021T231713_leg_eval_seed1_press_rep2_force_6003_slide).
- **Scope:** one plate model, one virtual stiffness (400 N/m), one frozen slide set of 96. The layer reads only the foot's
  position (kinematics) and its estimate of the plate; nothing here uses a force sensor or the policy's own force
  estimate.
- **Implication:** the force channel of a UniFP-style policy is a target offset, not a force loop: what makes it press
  with a commanded force on any surface stiffness is the layer's goal following the foot along the normal, and what lets
  it push along a surface is not to correct its position there. A layer for the dog needs the same four rules (all in
  `unifp_train/foot_press.py`, plain torch).

### F-182 — Pressing up on a plate above the front-right foot at 28 N tips the Go2 over its unsupported corner: a fine-tune that learned to push the full command there (round 5, on plates with edges) falls in 5 and 11 of 288 frozen presses, all on plates facing down at 28 N, rolling ~45 degrees, at 33-42 N of contact -- where round 4's policy, under-pressing them by ~6 N, stands; the force a task layer commands has to be capped by direction, and round 4's final stays the candidate

- **Status:** provisional (simulation; one fine-tune; the tipping force not computed, only seen)
- **Week:** 6
- **Date:** 2026-10-22
- **Evidence:** [Week 6 log, 2026-10-21/22](week_06/notes.md), round 5. Both variants from round 4's finals, 1200 iterations
  on the final layer and plant (the split in-press loop, the release from the foot, the plate as a slab with edges):
  [force](#/week/6/run/20261021T234224_train_seed3_press_r5_force_s3), [position](#/week/6/run/20261021T234340_train_seed3_press_r5_position_s3).
  The force-aware final [167](#/week/6/run/20261022T012835_leg_eval_seed1_press_r5_force_7198_real) (5 fell) and
  [190](#/week/6/run/20261022T012838_leg_eval_seed1_press_r5_force_7198_real_settled) (11 fell), plates facing down 7 and 9
  of 72; across the sweep it fell in 2-86 episodes (86 at 20 ms of delay). Round 4's force-aware final on the same layer
  and plant: [196](#/week/6/run/20261021T234043_leg_eval_seed1_press_r4_force_5999_real_edges) (0 fell, 2 damaged),
  [207](#/week/6/run/20261022T014725_leg_eval_seed1_press_r4_force_5999_final_layer_settled) (0 fell, 0 damaged), on the slide set
  [56 of 96](#/week/6/run/20261022T014728_leg_eval_seed1_press_r4_force_5999_final_layer_slide) with nothing past 34 N; its
  position-only twin [100](#/week/6/run/20261022T014841_leg_eval_seed1_press_r4_position_5999_final_layer_real) /
  [86](#/week/6/run/20261022T014846_leg_eval_seed1_press_r4_position_5999_final_layer_settled) /
  [30](#/week/6/run/20261022T014959_leg_eval_seed1_press_r4_position_5999_final_layer_slide). Round 5's position-only final
  [98](#/week/6/run/20261022T013202_leg_eval_seed1_press_r5_position_7198_real) and, on the key,
  [95](#/week/6/run/20261022T013424_leg_eval_seed1_press_r5_position_7198_key_dog33) of 108.
- **Scope:** the falls' mechanism is read off their traces (the plate pushes the lifted foot down, outside the three-foot
  support); no static tipping force was computed. One fine-tune of one lineage.
- **Implication:**
  - A general foot policy can be commanded into forces the robot cannot stand: pushing the front-right foot up against
    something loads the corner it is not standing on. The task layer, not the policy, has to know the feasible force by
    direction (and the frozen set's 28 N upward presses test refusal rather than tracking).
  - Fine-tuning a policy that already presses well onto a changed plant cost it safety in 1200 iterations; round 4's final
    (`checkpoints/leg_press_force_r4_model_5999.npz`) is safe on the final layer and plant without retraining.

### F-184 — On the real dog the D1 turned Lukas's lever key from 45 degrees up to its stop, verified by the robot's own check, from a walk up and a lidar refit; the two live turns before it aborted mid-turn (+25 and +14 degrees) on a clock that waited for the arm's wrist, and on the way home after the turn the hand hit the head camera, which the arm's clearance proxy does not hold

- **Status:** provisional (one turn to the stop on the dog; the unlock seen by Lukas, the angle the fingers' forward
  kinematics on the estimated key, not measured on the key)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The first live turn" and "The key turned to its stop on the
  dog, the way home past the head camera, and on to the door pull";
  [the turn to the stop](#/week/6/run/20261021T082236_key_turn_live), the turns stopped at
  [+25](#/week/6/run/20261021T081246_key_turn_live) and [+14 degrees](#/week/6/run/20261021T081629_key_turn_live), two
  live reaches ([1](#/week/6/run/20261021T080820_key_reach_live), [2](#/week/6/run/20261021T081113_key_reach_live));
  `demos/combiner_key/robot/key_plan.py`,
  `demos/combiner_key/tests/test_key_robot.py`.
  - The turn to the stop: pitch 70 degrees planned (Joint5 >= 24.0 from its singularity), the walk up 0.34 m in one stop,
    the refit 50.0 mm and 1.2 degrees, the lock 0.46 m ahead and 8 cm left; the turn 12.1 s, the fingers' measure -44 to
    +45.6 degrees, the hand at most 17.7 mm off its path, the worst joint 9.8 degrees behind the arm link's command (the
    loaded abort is 10 for 0.6 s).
  - The two turns before it stopped at +25 and +14 degrees, the key following its command (to 1-2 degrees in the
    second), on a 3 s budget of clock holds for a joint 5-6 degrees behind; now the clock holds past 8 degrees and the turn aborts only when the fingers'
    measure has not advanced 2 degrees in 3 s.
  - The camera: from the record's measured joints against the D435i's case and bracket (`head_camera.CLEARANCE_BOX_BASE_M`,
    the proxy's 3 cm link radius), the turn kept 28 mm, the back-off along the approach (`key_back`) ended at -1 and the
    line home -30, the fingertips in the case; the fold 142 mm. Rising straight up from above the key to 0.20 m over the
    base first (`key_clear`) keeps 31 mm on the replayed lift-off and 80 home; dry over four boxes and 0.30-0.37 m base
    heights, 45 and 79.
- **Scope:** one turn to the stop and two partial (five live walks up, refit 44-57 mm each), one box, one key; the lock's 5 N felt, not measured; whether the
  door then opened was not tried; the D1's push not measured (its joints' lag is); `key_clear` checked on the CPU model
  and replayed joints only, not yet run on the dog. The door pull's own phases are not checked against the camera.
- **Implication:**
  - F-183's arm-only unlock carries to the robot: the scripted push, planned off the wrist's singularity and retimed to the
    arm link's speed, turns the real key -- with a joint within 0.2 degrees of the loaded abort, so little margin for a
    stiffer lock or a lower stance.
  - The head camera has to be in any clearance check for moves in front of the base (the key's, the door pull's, a
    policy's): the arm's proxy (`position_only.workspace`) predates its mount.
  - Next: `key_clear` on the dog, then the door pull straight after the turn (`node.py --then_door pull`).
- **Update (2026-10-21, evening):** both done on the dog in [one run](#/week/6/run/20261021T085022_key_turn_door_pull_live).
  The key turned to 46.1 degrees, verified. `key_clear` kept 29.3 mm (proxy margin) from the head camera, and nothing
  touched it. The door pull followed from where the key left the dog: walked over in one stop, refit 15 mm, pulled,
  released and folded. A [run before it](#/week/6/run/20261021T084501_key_turn_door_pull_live) also turned and
  verified the key (45.4), but its pull aborted with the arm loaded, the door not opening. Two verified turns since the
  lag fix, both to the stop.

### F-183 — In simulation the D1 alone turns Lukas's lever key from 45 degrees up to 45 down against his 5 N lock (taken at 0.13 m, 0.65 N·m) in 11 of 11 attempts from the door demo's head-camera and lidar look and a walked stance, its fingers pressing the key's platform with the hand pitched 60 degrees down; it turns locks up to 11 N (4/4), 1 of 4 at 14 N and none at 17, where the arm's quasi-static push ceiling (10-11.5 N there, Joint1 binding at +-45 degrees) puts the limit; its fingertip measure of the turn reads a stalled key as turned unless the hand's lag behind its command is checked

- **Status:** provisional (simulation; the lock is the leg turn's analytic plant against a sphere at the fingertips;
  the arm at its published torque limits; one seed)
- **Week:** 6
- **Date:** 2026-10-21
- **Evidence:** [Week 6 log, 2026-10-21](week_06/notes.md), "The key turned by the arm (simulation)";
  `demos/combiner_key/arm_turn.py`, `arm_env.py`, `run_arm_turn.py`, `plot_arm_turn.py`, `tests/test_arm_turn.py`;
  [`figures/arm_turn_demo_5n.png`](week_06/figures/arm_turn_demo_5n.png),
  [`figures/arm_turn_lock_sweep.png`](week_06/figures/arm_turn_lock_sweep.png).
  - CPU (`python -m demos.combiner_key.arm_turn`): a level hand does not solve at the key; pitched 60 degrees the whole
    turn solves at 72 of 72 stance, height and wrist-camera combinations (door 0.42-0.48 m, lock 6-10 cm left of the
    base, base 0.28-0.33 m), trunk clear; the worst angle's ceiling 1.31-1.50 N·m.
  - Simulation, the head camera and MID-360 look, the box spread as in the key demo: 5 N
    [8/8](#/week/6/run/20261021T061331_lock0.65nm_seed1_placements) plus 3 single runs, all to the stop (45.2
    degrees), the hand at most 8-10 mm behind its command, the push 4.8 N along the arc; 8 N
    [4/4](#/week/6/run/20261021T061751_lock1.04nm_seed1_sweep) (44.6-44.9); 11 N
    [4/4](#/week/6/run/20261021T062130_lock1.43nm_seed1_sweep) (42.2-42.8, joints at their limits in 21-35% of push
    steps); 14 N [1/4](#/week/6/run/20261021T062509_lock1.82nm_seed1_sweep); 17 N
    [0/4](#/week/6/run/20261021T062849_lock2.21nm_seed1_sweep).
  - The fingertip edge's measure at the hold read 45 degrees on the stalled keys (the fingers had slid off). Lag behind
    the command in the turn: at most 20 mm on every key that turned, 96-160 mm on every stalled one.
- **Scope:** the arm's published limits (the firmware's real delivery is not measured, F-142); the D1's 10 Hz streaming
  as `motor_model` models it (it makes the contact ripple 3-6 N about its mean); contact is one 5 mm sphere at the
  fingertips' edge, the rest of the hand passing through the key; the lock a constant friction from one felt number at
  an assumed lever arm, the unlock at 30 degrees the leg model's; the wrist camera's mass not on the simulated arm (on
  the CPU it helps); the stance 2-4 cm off its target (the gait's drift), within what the CPU scan covers; one seed, 27
  attempts.
- **Implication:**
  - The arm can be the key's actuator for a lock like Lukas's: about twice the force it needs, without the leg policy's
    handover, crouch or 76-88 N contact peaks (F-166) -- but with little margin past ~11 N, and at its published
    torques, which a real D1 may not deliver.
  - Any robot version needs the lag check (or the key's angle measured otherwise) before it calls the door unlocked,
    and a hold that rests on the key rather than pressing it into its stop (pressed, Joint1 sat at 100%).
  - Before the real dog: where the 5 N was felt, the D1's torque at its limits, and a dry run in the combiner door's
    robot program with the RViz ghost.
- **Update (2026-10-21, same day):** the walk now stops in a window (the door 0.42-0.45 m ahead, the lock 4-12 cm left,
  facing to 3 degrees) instead of hunting for the stance's point, as the real door pull accepts a range: 16 more
  attempts at 5 N, [8/8](#/week/6/run/20261021T064727_lock0.65nm_seed1_window) and
  [8/8](#/week/6/run/20261021T065244_lock0.65nm_seed1_window_near) unlocked to the stop, the walk over in under 2.1 s
  rather than 12. 27 of 27 at 5 N in all.
- **Update (2026-10-21, for the robot program, `demos/combiner_key/robot/key_plan.py`):** on the CPU model with the arm
  link's 30 deg/s profile the scripted turn flipped the D1's wrist mid-turn (the push's hand passes near its singularity,
  Joint5 6-12 deg) and asked 66 deg/s of Joints 4 and 6; planned before anything moves on the branch furthest from the
  singularity and retimed to at most 22.5 deg/s a joint, it turns to the stop in 12.7-15.5 s (dry, 4 of 5 placements;
  the fifth told to move the dog). The simulation's 9 s turn needs the D1's full 1.2 rad/s. Not run on the dog.

### F-186 — The cam lock's bezel found from the wrist RealSense's image alone (a dark disc with light all round it, an ellipse through its rim, the gripper masked) is right in 754 of 772 and 565 of 566 frames of two hand-held clips (daylight; dim red light; heavy motion blur in places), wrong in 3, and finds something in 4 of 91 frames with the lock out of view; centring `keyhole.find_bezel`'s 12 mm rim search on it keeps it right with the model's place 20-40 mm off (95% of frames), where the search round the model alone is right in 0-2%

- **Status:** provisional (two hand-held clips, ground truth from a checked track; no arm pose; not run on the dog)
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "The keyhole found in the wrist image first";
  `results/week_06/keyhole_clips/ground_truth.json` and `eval.json`; `demos/combiner_key/bezel_image.py`,
  `demos/combiner_key/robot/eval_keyhole_clips.py`, `demos/combiner_key/tests/test_bezel_image.py`.
  - Image alone, no prior: clip 1 (daylight, 869 frames) right 754, wrong 3, missed 15, 1/66 found with the lock out of
    view; clip 2 (dim red, 601 frames) right 565, wrong 0, missed 1, 3/25. Right: centre within 0.3 radii, radius
    within 25%. The wrong: the bezel ~10 px in radius and blurred, beaten by a tag's cell or clutter.
  - `find_bezel` against a model E mm off, every 5th labelled frame (152 + 114): 0 and 10 mm, both ways 146-147 and 102;
    20 mm, model-only 2 and 3, image-first 146 and 102; 30 and 40 mm, 0 and 0 against 144 and 101-102.
  - Time on the dev PC: the image alone over every size 81-87 ms a frame median; `find_bezel` with the 45 mm window 40 ms
    (19 ms without). On the dog's CPU (the p2dingo-web image): 0.6-0.9 s over every size, `find_bezel` 144 ms (71 ms
    without); the 8 test frames give the same answers there.
- **Scope:** two clips of one box in one room, the arm moved by hand (blur the dog's held view lacks), recorded through
  the website's encoder; the "model" for the `find_bezel` numbers made up from each frame's truth (exact intrinsics, a
  flat door, the circle's tilt from its ellipse), so the plane and pose errors of a real look are not in them; the
  gripper mask fitted to these clips; the ground truth is a track of the same candidates, so frames the candidate stage
  never found (31 + 10, blur or the rim off the edge) are left unlabelled rather than counted as misses.
- **Implication:**
  - The wrist mount's centimetres of error no longer have to be inside the model's 12 mm for the camera to find the lock.
  - The dog's dry `--stage keyhole` run, the arm held still at `key_view`, is the next check: found rate, and the image's
    find against the rim search's.
  - A learned detector is not needed for this box and light; it would earn its place for other boxes, the key over the
    bezel near contact, or outdoor glare, as a candidate source checked by the same gates.

### F-185 — Through the Go2's own front camera (a fisheye fitted on one stand, not calibrated) the combiner door demo's look places the box within 6 mm in simulation and opens 16 doors of 16, as the head RealSense did (F-155); Isaac renders a USD camera whose focal length is 0.35 ~1.35x too large while reporting the requested intrinsics; on the real dog the one-stand lens fit (f 790 px, k1 -0.09) puts the tag 0.609 m off against a taped 0.61 and agrees with the lidar's tilt to 0.02 degrees

- **Status:** provisional (simulation, one seed of 16; the lens fitted on one stand; the look not yet run on the dog)
- **Week:** 6
- **Date:** 2026-10-22
- **Evidence:** [Week 6 log, 2026-10-22](week_06/notes.md), "The look moved to the Go2's front camera";
  [the 16 placements](#/week/6/run/20261022T104403_door_combiner_door_seed1_combiner_door), the debugging runs listed there;
  `demos/combiner_door/front_camera.py`, `demos/combiner_door/tests/test_combiner_door.py` (`FrontCamera`),
  `demos/combiner_door/tests/test_robot.py` (`FrontCameraTests`, the box and the dock in one look).
  - Simulation: tag 16/16, box 6.1 mm median / 7.8 max, 0.13 deg, jaws 6.0 mm at the lever, 16/16 open, no falls; the
    look from 0.40 m back (P2Dingo's walking policy stands at 0.314 m; from 0.18 m back the tag was off the image).
  - The render bug: with Isaac Lab's default aperture (1.0) the front camera's focal length was 0.35 and the box came out
    175 mm too near, independent of render size and aspect; the head RealSense (focal 0.95) in the same scene 8-15 mm.
    Focal 1.8 (aperture 5.2): 5 mm apart.
  - The lens: 20 frames on the dog, camera taped 0.61 m from the box's front; fitted to that distance and the box's 0.63 m
    width (0.609, 0.629); the lidar's plane 0.603 m and tilt 4.72 deg against the camera's 4.7 not fitted to.
- **Scope:** simulation with ideal rendering through the same lens model it is fitted with, so the lens's own error is not
  in the simulated number; one seed; the lens from one stand at one distance near the image's centre (principal point
  and k2-k4 assumed); the dog's look through it not yet run (the one live try was in a dark room).
- **Implication:**
  - The front camera can replace the wrist RealSense for the box's look: no arm move, no CAD mount or arm pose in the
    chain, the arm folded. The wrist RealSense is left the insertion's keyhole view.
  - The dog must start further back: the box's root at least 0.95 m ahead (the door pull's look band 0.95-1.14 m).
  - Any wide simulated camera built with `PinholeCameraCfg.from_intrinsic_matrix`'s default aperture should be checked
    against a known distance before its numbers are trusted.
  - A chessboard calibration should replace the one-stand fit before trusting heights off the image's centre.


### F-187 — A program to drive the dog's legs with the general foot-press policy onto an operator's hand: its numpy task layer reproduces the training's to 1e-9, its LowCmd CRC matches Unitree's C, and its whole live sequence (lie down, release sport mode, stand up, press at most 5 N, lie down, give sport mode back) runs over ROS 2 against a kinematic stand-in with every command's CRC right and none sent while sport mode held the legs; dry on the dog it walks the sequence and reads the arm; it has not driven the real legs

- **Status:** confirmed (software, on the PC and dry on the dog); not run live
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "The foot-press policy driving the legs: the program, not yet
  run live". `demos/combiner_key/robot/press_layer.py`, `lowlevel.py`, `press_live.py`, `run_press_on_dog.sh`,
  `fake_go2.py`; `tests/test_press_live.py` (29 tests).
  - The layer against `unifp_train.foot_press.PressLayer` on closed-loop kinematic foot paths (a rigid plate facing up, a
    wall with three segments, a 1,000 N/m plate, nothing there): goals, forces and levels to 1e-9 in float64. In
    float32 (training's) the torch layer's timers end a hold one step (20 ms) later.
  - The CRC: `crc32_core` from unitree_ros2's `motor_crc.cpp` compiled with gcc gives 279564839 and 1264208000 on two
    word sets and 736096623 on a whole `LowCmd` through `motor_crc.h`'s struct (812 bytes), as `lowlevel` does.
  - Over ROS 2 (localhost, domain 77) against `fake_go2.py`: [the whole sequence](#/week/6/run/20261023T013017_leg_press_live)
    and [from the dog's lying pose](#/week/6/run/20261023T013325_leg_press_live), 12,510 and 12,520 LowCmd at 500 Hz,
    0 failing their CRC, 0 while sport mode held the legs, the requests CheckMode, StandDown, ReleaseMode, CheckMode,
    SelectMode, CheckMode. A [first run](#/week/6/run/20261023T012947_leg_press_live) stopped at the lie-down, as it
    should with a robot that does not lie (the fake's bug).
  - Dry on the dog, lying ([rehearsal](#/week/6/run/20261023T013526_leg_press_rehearse)): every stage walked by the
    pretend foot, the stall at the palm, 5.0 N held, home; inference 3.0-3.7 ms; the arm link's angles read. Two
    earlier rehearsals found that sport mode's lying calves (-2.78 to -2.85 rad) are past the URDF's -2.72
    ([stopped](#/week/6/run/20261023T013144_leg_press_rehearse); the limit stop now applies only while the policy
    drives, which would otherwise have stopped every live stand-up) and that the decoder's force estimate on a lying
    dog reads 19-59 N ([released](#/week/6/run/20261023T013422_leg_press_rehearse); not used in rehearsals now).
- **Scope:** no dynamics anywhere: the stand-in follows targets kinematically, so nothing here says how the policy moves
  the real legs, whether the handover's ramps (unitree_rl_lab's FixStand gains, from the measured lying pose to the
  measured stand) hold the dog, or how this firmware's motion switcher behaves. The remote's key bits are Unitree's
  documented layout, not yet read off this dog. The task layer's frame moves with the base (no odometry); training held
  the plate in the world. The dog's three rules (wait for the operator at the pre-contact point, no press without a
  stall, a 5 N cap) are not in training.
- **Implication:**
  - The live test with a hand can be run: `d1dogpress --rehearse` first (B on the remote should print), then
    `d1dogpress --live` with someone beside the dog and room around it.
  - The decoder's foot-force estimate is not a contact sensor off its training states (19 N at rest lying, up to 59 N);
    the 25 N release on it is a backstop, and what it reads while the real foot presses is a measurement to make.

### F-188 — The first live run of the foot-press program: stood up from lying to sport mode's stance and handed to the policy as the gains reached kp 25, the dog sat back within 0.24 s (11 policy steps; rear knees folded from -1.86 to -2.43 rad, base 0.273 to 0.212 m, pitch to 20 degrees nose up) and the tilt stop put it in damping; the motors delivered the PD torque asked of them, and what the policy took over was a robot sinking through a crouch it never trained from

- **Status:** confirmed (one live run on the dog)
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "The first live run: the dog sat back".
  [The run](#/week/6/run/20261023T014425_leg_press_live) (`d1dogpress --live`, sport mode `mcf`):
  - The stand-up on unitree_rl_lab's FixStand gains (kp 60/80/80) to sport mode's stance (0.333 m, calves -1.48 to
    -1.54) reached 0.298 m, the rear calves 0.18 rad short and the base 4.5 degrees nose up; blending to kp 25 / kd 0.5
    with that stance as the target, it sank to 0.273 m, 6 degrees nose up, still sinking (~0.15 m/s) when the policy
    started.
  - The policy's first step asked FL hip +0.37, RR thigh +0.75, RL thigh +0.55 rad off the measured joints. Fed the
    same pose level and still it asks 0.54 rad; from the trained settled stance, 0.25. Over 10 steps the rear knees
    folded 0.6 rad against targets ~0.9 rad more extended (23 N·m asked, 20-27 N·m measured) and the rear sat down.
  - `tau_est` against kp (target - q) - kd dq of the command being sent: slopes 0.89-1.07 on every joint fitted, in
    the stand-up hold, the blend and the policy's 0.24 s.
  - Standing in sport mode the IMU reads level (shadow runs: pitch +0.2 and -1.35 degrees), so the 4.5-6 degrees were
    the rear sagging, not an offset.
  - A Ctrl-C at the damping prompt: rclpy's own SIGINT handler shut its context, the LowCmd writer died ("publisher's
    context is invalid") and no damping could have been sent; harmless only because the motors were already damping.
- **Scope:** one run, 0.24 s of policy. It does not say whether the policy holds the real dog from the trained settled
  stance: only that this start was outside every trained start (0.30-0.34 m sport stands and the 0.272 m settled stance,
  all level and at rest) and that the policy's response from it was large. The rear's collapse is read off the joints;
  the feet were not observed.
- **Implication:**
  - The handover now ends where training starts: the stand-up goes to the policy's default targets, the gains are
    blended with those kept, and the policy starts only once every joint is still and the stance is within 0.2 rad and
    5 degrees of the trained settled one (otherwise the dog lies down on the stand-up gains). Checked against the
    stand-in only ([stand only](#/week/6/run/20261023T015053_leg_press_live)).
  - Ctrl-C is a KeyboardInterrupt again (`SignalHandlerOptions.NO`): [mid-policy](#/week/6/run/20261023T015445_leg_press_live)
    it gives damping and the handback on Enter; a writer that cannot publish now says so.
  - Next on the dog: `d1dogpress --live --presses 0` (the policy holding the stand only, 10 s) before any lift.

### F-189 — With Lukas's key holster beside the D1's base re-angled to hold the lever 41 degrees above level, the arm takes the key by the bar's root (the dog's insertion grip) and draws it out along the holster 99-99.5 mm (75 needed) in 25 of 25 simulated attempts, with the holster as planned or up to 6 mm off per axis; at the first export's 35 degrees the CPU arm model draws it only 70-75 mm before Joint1's limit; the key the holster holds is v3 mirrored, and it comes out turned 180 degrees about its shaft from how the insertion holds it

- **Status:** provisional (simulation)
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "The key holster beside the D1's base".
  - Registration (`demos/combiner_key/holster.py`, `assets/holster/registration.json`): the base flange's holes
    58.0 x 130.0 mm and its underside put the CAD frame on `base_link` (the folded arm's extent settles the 180°); the
    mirrored v3's bar fits the export's by a rotation to 0.026 mm median; v3 as drawn fits only as a reflection.
  - The first export, the CPU model: the root grip needs the elbow at 92% of its range; the draw stops at 70-75 mm
    (Joint1 at its limit, the wrist pitch beside its axis); 20-30 mm along the bar, 90-105 mm. Tilted about the cup,
    4-8° steeper draws 95-115 mm from the root with every joint at 88-92%; 12° or more and the grip does not solve.
  - The second export (40.8°), the CPU model: 100 mm drawn, every joint within 0.895 of its range, the shoulder at 32%
    of its torque, the open fingers ≥ 11.5 mm off the bar, the key ≥ 11.6 mm off the base; the insertion's own roll does
    not slide onto the bar.
  - Isaac (`run_holster.py`): [one attempt](#/week/6/run/20261023T015209_seed1_roll_fixed) (jaws 0.3 mm and 0.14° off
    the plan, the grip's spring error ≤ 2.1 mm, torque ≤ 60%), [8 as planned](#/week/6/run/20261023T015351_seed1_batch8_err0mm),
    [8 up to 3 mm off](#/week/6/run/20261023T015450_seed1_batch8_err3mm), [8 up to 6 mm off](#/week/6/run/20261023T015549_seed1_batch8_err6mm)
    (jaws up to 4.7 mm off across the bar, spring error ≤ 5.6 mm, fingers ≤ 5.9 N, Joint3 at its torque limit for
    0.6 s in one). A [first run](#/week/6/run/20261023T014641_seed1_first) failed on a reflected registration (the hand
    asked for was the solved one turned 180°); fixed before the 25.
- **Scope:** PhysX on boxes (the holster's nine, the key's bar, platform, shaft and wards) on the CPU model's
  kinematics, the robot standing on the walking policy; the grip is the key demo's spring (600 N/m, 10 N cap), which
  absorbs misalignment a rigid real grip would push into the holster; no wrist RealSense mass; the holster's mounting,
  flex and print, and the real arm's joint zeros (F-023), not modelled. The 0 mm batch is deterministic (repeatability,
  not spread). Getting from the drawn pose to the insertion, and the insertion with the hand rolled 180°, not checked.
- **Implication:**
  - The 41° holster can be printed for the root grip; the 35° one cannot be drawn from the root.
  - Before the dog: check the folded arm's jaws against the printed holster dry (the grip needs Joint2 at 89% and Joint3
    at 88% of their ranges), and decide how the key gets from the hand rolled +1 to the insertion's roll -1: the insertion
    planned for roll +1, or a regrasp.

### F-191 — Under the policy's own default targets at kp 25 / kd 0.5 the real dog does not settle into the stance the policy trained from: it comes to rest 8.1 degrees nose up at 0.261 m with the rear thighs 0.16-0.20 rad lower (0.272 m, level in training), and from that rest, or from the stand-up hold at kp 80, the policy's first step asks ~0.5 rad of the rear thighs where it asks 0.25 from the trained stance -- close to what it asked in F-188 before the rear gave way

- **Status:** confirmed (one live run on the dog; the policy's response computed offline, open loop); see
  [F-194](#f-194): the policy's ~0.5 rad first step starts the crouch it takes from every start in simulation
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "The second live run: the stand only".
  [The run](#/week/6/run/20261023T020839_leg_press_live) (`d1dogpress --live --presses 0`): the stand-up to the default
  targets on kp 80 held 0.303 m, 4.6 degrees nose up; the blend to kp 25 dropped it 16 mm in its last 0.2 s (the
  "momentary collapse") and it sank to rest at 0.261 m, 8.1 degrees, joints within 0.01 rad from 12.4 s on. The settle
  gate refused the policy ("not still": the velocities read 0.04-0.10 rad/s at rest; now judged from positions) and the
  dog lay down on the stand-up gains. Knee torques at rest: front 5.6 / 4.6 N·m, rear 11.0 / 9.1 (the trained settled
  stance implies 7.5 front, 8.3 rear). Foot loads from `tau_est` through each leg's Jacobian: 138 N in total at rest
  at kp 25 (front share 0.35), 242 N at the kp-80 hold (0.49), against 171 N (0.46) in the trained stance -- a spread
  that one weight cannot make, so the torques at rest are not gravity's alone. The policy's first open-loop step:
  0.51 rad from the kp-25 rest (0.45 levelled), 0.46 from the kp-80 hold, 0.62 from F-188's start, 0.25 from the trained
  settled stance.
- **Scope:** one stand on one floor. Friction in the joints is the likeliest reason the dog stops where it does (its
  load sums differ by ~100 N between rising and sinking; the training's leg model has 0.01 N·m) but it is not measured,
  and the weight's distribution is not settled by these numbers. Open loop, the first step says how hard the policy
  reacts, not whether it would hold the dog.
- **Implication:**
  - The policy should not be started from this handover: on the real legs its trained start is not reachable by
    holding its default targets, and its reaction from the stances that are reachable is the one that preceded F-188.
  - Either the handover puts the joints at the trained settled angles themselves (stiff gains, then an instant switch
    to the policy, as the training's settled spawn starts) or the policy is fine-tuned on starts like these and on leg
    friction; which one is open.

### F-190 — The D1 model's 10 Hz setpoints, each planned from rest, make slow arm moves stop-start as Lukas sees on the real arm: through the holster draw at its first timing the fastest joint sat near-stopped in 28% of 20 ms samples; pacing the draw by its fastest joint at 0.6 rad/s (and the unfold twice as fast) cuts that to 8% and the sequence from 18.5 to 12.3 s with tracking unchanged, where speeding it up evenly lags 37 mm at 2x and draws 2 of 8 at 3x

- **Status:** provisional (simulation)
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "The holster draw made faster". Six batches of 8 (the holster
  up to 3 mm off), `run_holster.py --speed --joint_speed`: [1x even](#/week/6/run/20261023T020550_seed1_speed1_err3mm)
  near-stopped 28%, ripple 0.99, lag 6.2 mm, grip error 3.6 mm; [2x even](#/week/6/run/20261023T020648_seed1_speed2_err3mm)
  17%, 37 mm, 17 mm, torque at its limit 8.9% of steps; [3x even](#/week/6/run/20261023T020729_seed1_speed3_err3mm) 2/8
  drawn; [paced 0.6](#/week/6/run/20261023T020935_seed1_paced0.6_err3mm) 8%, 0.52, 7.9 mm, 3.2 mm, 0.2%;
  [0.8](#/week/6/run/20261023T021020_seed1_paced0.8_err3mm) 6%, 9.9 mm, 3.7 mm; [1.0](#/week/6/run/20261023T021101_seed1_paced1.0_err3mm)
  5%, 14 mm, 9.5 mm. All paced batches 8/8.
- **Scope:** the stop-start is `motor_model`'s fitted firmware planner (velocity retention 0, from the F-035 sweep;
  provisional there), not measured here on the arm; Lukas's observation agrees with it. The spring grip absorbs lag
  that a rigid real grip would not. One scene, one seed.
- **Implication:** time the D1's Cartesian moves by its joints, fast enough that the leading joint moves more than
  ~2.5° a command and under its ~1.25 rad/s, rather than evenly along the line; `holster.joint_paced` does it for the
  draw and could do it for the insertion's moves. Worth a check on the real arm: stream a slow and a paced move and
  compare the 9 Hz feedback.

### F-192 — Simulating the dog's handover (lying, the stand-up on kp 80, the blend to kp 25) reproduces its kp-80 hold (0.306 m and 4.9 degrees nose up, the dog 0.303 and 4.6) and the direction of its kp-25 rest, but sinks further (0.215 m against 0.261); from every stance that handover gives, at rest or sinking at up to 0.18 m/s, round 4's force-aware final presses 197-203 of 288 frozen presses without a fall, as it does from its trained stands (196): in simulation the start stance does not explain F-188, and 1 N·m of leg joint friction costs more (197 -> 139)

- **Status:** provisional (simulation); its scope's last point and its implication superseded by [F-194](#f-194): the
  simulated policy does sit back as the dog did, from every start
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "The handover simulated, round 4's final from the dog's
  starts, and a first fine-tune for them that tipped"; `unifp_train/leg_handover.py` (`run_unifp_train.py
  leg_handover`), `leg_env` `leg_friction_range` and spawn velocities, `demos/combiner_key/assets/dog_stand_20261023/`.
  - The handover, 64 robots on the realistic legs, leg friction drawn 0-4 and 0-2 N·m:
    [lying slumped](#/week/6/run/20261023T022841_leg_handover_seed1_handover_friction0to4), [lying held](#/week/6/run/20261023T023034_leg_handover_seed1_handover_lie_held_friction0to2),
    [with the sink's velocities](#/week/6/run/20261023T023526_leg_handover_seed1_handover_moving_friction0to2). The hold is 0.303-0.306 m and 4.7-4.9° nose up
    with or without friction. The rest is 0.209-0.215 m and 11.8-13.5° nose up. Every joint moves off the trained settled
    stance the way the dog's did (front hips +0.07 / -0.06 against +0.069 / -0.062; rear thighs -0.27 / -0.28 against
    -0.16 / -0.20).
  - Round 4's final (`model_5999`) on the frozen v2 set, final layer and plant, realistic legs, every episode from:
    - [the dog's kp-25 rest](#/week/6/run/20261023T023140_leg_eval_seed1_press_r4_force_5999_dogrest): 197, 0 fell;
    - [the simulated hold](#/week/6/run/20261023T023220_leg_eval_seed1_press_r4_force_5999_simhold): 202, 0;
    - [the sink at rest](#/week/6/run/20261023T023301_leg_eval_seed1_press_r4_force_5999_simsink): 197, 0;
    - [the sink in motion](#/week/6/run/20261023T023608_leg_eval_seed1_press_r4_force_5999_simmoving): 203, 0;
    - against its classic stand, 196 ([today's control](#/week/6/run/20261023T032947_leg_eval_seed1_press_r4_force_5999_real_control), the same per plate
      kind as yesterday's).
  - With leg friction at 1 N·m (1.9 at the knees):
    - [the dog's rest](#/week/6/run/20261023T023342_leg_eval_seed1_press_r4_force_5999_dogrest_fric1): 139, 0 fell, force bias -2.3 N;
    - [the sink in motion](#/week/6/run/20261023T023650_leg_eval_seed1_press_r4_force_5999_simmoving_fric1): 165, 2 fell, on walls.
  - On a floor of μ 0.5: 215 and 195, 0 fell ([rest](#/week/6/run/20261023T023732_leg_eval_seed1_press_r4_force_5999_dogrest_mu05),
    [moving](#/week/6/run/20261023T023814_leg_eval_seed1_press_r4_force_5999_simmoving_mu05)).
- **Scope:** simulation only.
  - The lying pose's hips and thighs are from another day's LowState.
  - PhysX's joint friction slows the simulated sink but does not stop it where the dog stopped. Neither it nor
    anything else here holds the robot at 0.261 m, so the depth of the dog's rest is not explained.
  - The friction values are a bracket, not a measurement.
  - One evaluation seed, deterministic.
  - It does not say what made the real dog sit back in F-188: simulated starts sinking as fast do not.
- **Implication:**
  - The settle gate's 0.20 rad and 5° (F-188's fix) refuses the dog's own rest, a start round 4's final handles in
    simulation as well as its trained ones.
  - A policy fine-tuned for the start stance is not what simulation says is missing. What the dog does that the
    simulated legs do not is: the friction F-191 suspects, or something else in its legs.
  - Measuring the legs' friction is the next useful number, e.g. the torque at which each joint starts to move under
    a slow kp ramp.

### F-193 — Fine-tuning round 4's foot-press final on the dog's handover starts brings back round 5's tipping on upward presses (19-33 of 288 fell, peaks to 67 N median, with leg friction 0-2 N·m) unless the training caps the upward force; with it capped at 16 N the fine-tune stands (0 falls on the dog's plate kinds) but presses no better than round 4 on walls and plates facing up (79-126 against 80-129 of 144) and transfers worse to the key (88 against 107), and its first step from the dog's rest is round 4's (0.51 rad); exported for the dog with a start gate that admits that rest

- **Status:** provisional (simulation; one seed per arm, deterministic evaluations); its implication's last point
  superseded by [F-194](#f-194): the stand-only test would stop on the policy's own crouch
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "The handover simulated, round 4's final from the dog's
  starts, and a first fine-tune for them that tipped" and "The fine-tune with the upward force capped: no falls, no
  better than round 4; exported for the dog".
  - The first fine-tune: [1000 iterations](#/week/6/run/20261023T023944_train_seed3_press_ft1_force_s3) with friction 0-2 N·m.
    - Falls: [5](#/week/6/run/20261023T033151_leg_eval_seed1_press_ft1_force_6400_real) by 6400, then
      [19](#/week/6/run/20261023T032202_leg_eval_seed1_press_ft1_force_6600_dogrest)-[32](#/week/6/run/20261023T031511_leg_eval_seed1_press_ft1_force_6998_dogrest) from the dog's rest, all on
      plates facing down.
    - Also [37](#/week/6/run/20261023T032653_leg_eval_seed1_press_ft1_force_6600_key_dog33)-[90](#/week/6/run/20261023T032006_leg_eval_seed1_press_ft1_force_6998_key_dog33) of 108 on the key.
  - The capped ones, 600 iterations each: [ft2a](#/week/6/run/20261023T033438_train_seed3_press_ft2a_force_s3) without friction and
    [ft2b](#/week/6/run/20261023T033538_train_seed3_press_ft2b_force_s3) with 0-1 N·m. ft2a's final against round 4's:
    - [185 / 0](#/week/6/run/20261023T042611_leg_eval_seed1_press_ft2a_force_6598_real) on the classic stand (196 / 0);
    - [203 / 0](#/week/6/run/20261023T042653_leg_eval_seed1_press_ft2a_force_6598_dogrest) from the dog's rest (197 / 0);
    - [148 / 0](#/week/6/run/20261023T042734_leg_eval_seed1_press_ft2a_force_6598_dogrest_fric1) from it with friction (139 / 0);
    - [148 / 0](#/week/6/run/20261023T042816_leg_eval_seed1_press_ft2a_force_6598_simmoving_fric1) from the moving sink with friction (165 / 2);
    - [211 / 2](#/week/6/run/20261023T044513_leg_eval_seed1_press_ft2a_force_6598_real_settled) from the settled stance (207 / 0);
    - [53 / 0](#/week/6/run/20261023T044635_leg_eval_seed1_press_ft2a_force_6598_slide) of 96 on the slide set (56 / 1);
    - [88](#/week/6/run/20261023T044715_leg_eval_seed1_press_ft2a_force_6598_key_dog33) of 108 on the key (107).
    - ft2b fell on upward presses in three of the four conditions.
  - The first step, open loop, from the dog's rest: 0.50-0.52 rad for every capped checkpoint, against round 4's
    0.51 and 0.25 from the trained settled stance.
  - Export: `checkpoints/leg_press_force_ft2a_model_6598.npz`, its `start_gate` 0.402 rad and pitch -15.9 to +1.0°.
- **Scope:**
  - Simulation, the frozen v2 set and the dog's starts.
  - Friction is a bracket, not a measurement (F-192).
  - The cap's 16 N is a choice under round 5's 33-42 N tipping, not a computed limit.
  - One seed per arm; the checkpoints differ from each other by ±20 presses.
  - The dog's own presses (5 N at most, `palm_up` and `palm_facing`) are not in the frozen set: its 8 N, upward-facing
    and wall rows are the nearest.
- **Implication:**
  - A fine-tune of a general foot policy must keep F-182's directional cap in its training, or it learns to push
    upward presses past where the robot stands.
  - For the start stance, no fine-tune improved on round 4 in simulation. The export is useful for the gate it
    carries: it lets the live program start from the dog's own rest, the test that says whether the start or the
    legs' dynamics made F-188.

### F-194 — The foot-press policy sits back in simulation just as the dog did in F-188: within 0.14-0.30 s of every start, still waiting on the foot-down command, its base drops ~12 cm to 0.22 m and pitches 17-25 degrees nose up on ~21 N·m at the support knees (the dog: 0.273 -> 0.212 m, 20 degrees at the stop, 20-27 N·m), and it presses from that posture (each episode's largest tilt median 30-32 degrees, up to 42); the live program's 15-degree tilt stop fired on the policy's trained posture, not on a fall

- **Status:** confirmed in simulation (the traces of evaluations already recorded); one live run to compare
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "Why the dog sat back: the policy's own crouch". The
  per-step traces (`leg_trace.npz`) of round 4's final:
  - from [the classic stand](#/week/6/run/20261023T032947_leg_eval_seed1_press_r4_force_5999_real_control): base 0.338 -> 0.219 m in the first 0.5 s, pitch
    22° nose up, past 15° in 288 of 288 at 0.30 s (median); one episode -12.8° at 0.18 s, -14.6° at 0.24 s, -21.4° at
    0.42 s, all before its approach starts at 0.48 s;
  - from [the dog's kp-25 rest](#/week/6/run/20261023T023140_leg_eval_seed1_press_r4_force_5999_dogrest): 0.271 -> 0.215 m, 25°, past 15° at 0.18 s in 99%;
  - from [the simulated hold](#/week/6/run/20261023T023220_leg_eval_seed1_press_r4_force_5999_simhold): 0.313 -> 0.218 m, 23°, at 0.26 s in 100%;
  - from [the sink in motion](#/week/6/run/20261023T023608_leg_eval_seed1_press_r4_force_5999_simmoving): 25°, at 0.14 s in 100%.
  - While waiting (0.2-0.48 s): support-knee torque median 20-21 N·m (p90 23-24).
  - Through the episode: median pitch by stage -13 to -19° waiting, -24° lifting, -26 to -30° pressing, -21° returning,
    -9° with the foot back down. Each episode's largest tilt: median 30-32°, p95 37-38°, at most 42°.
  - ft2a's final, the exported fine-tune ([from the dog's rest](#/week/6/run/20261023T042653_leg_eval_seed1_press_ft2a_force_6598_dogrest)): 15.6°
    median in the first 0.5 s, past 15° in 73%.
  - F-188's dog: 0.273 -> 0.212 m, rear knees -1.86 -> -2.43 rad, 20.3° nose up at 0.24 s when the stop fired,
    20-27 N·m measured.
- **Scope:**
  - The comparison is with one live run's 0.24 s. Whether the real dog would have held the crouch, as the simulated
    one does, was not seen: the stop cut it.
  - The posture is what the training rewarded: UniFP's orientation term at -4 costs a 25° crouch ~0.7 a second, against
    ~10 a second of positive reward; the base height term costs the 12 cm drop 0.01 a second.
  - Found late: F-192 and F-193 judged the simulated starts by falls, not by tilt.
- **Implication:**
  - Under the live program's 15° stop every run of round 4's final or ft2a's is stopped within ~0.3 s, from any start.
  - F-192's "nothing modelled here reproduces it" and F-193's stand-only test are superseded.
  - The fine-tune Lukas chose (2026-10-23) targets a level base: the tilt penalised past a hinge, long waits on the
    foot-down command, each candidate scored on how many episodes pass 15°.

### F-195 — Fine-tuned for a level base, the foot-press policy stops the crouch but loses the dog's own presses: penalising the tilt everywhere (ft3a, ft3c) levels it (6-9 degrees) and drops the dog's two hands to 75-174 of 288 with up to 30 tip-overs and 87 contacts past 60 N; penalising it only while standing (ft4) passes no 10 s stand past 15 degrees (round 4: 284 of 288) but over-presses the palm under the foot (42-64 of 144, up to 29 past 120 N), where round 4's final presses all 144

- **Status:** provisional (simulation; one seed per recipe)
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md): "The fine-tune for a level base", "ft3a trained on (ft3c)",
  "Level only while standing (ft4)".
  - The dog's hands (`--plate_set dog`, 288 presses at 5 and 8 N from the dog's rest):
    - [round 4's final](#/week/6/run/20261023T084229_leg_eval_seed1_press_r4_force_5999_hands): 280, palm_up 144 of 144, 0 fell, 5 past 60 N;
    - [ft3a](#/week/6/run/20261023T084352_leg_eval_seed1_press_ft3a_force_6998_hands): 75;
    - [ft3c](#/week/6/run/20261023T084438_leg_eval_seed1_press_ft3c_force_7997_hands): 174, 30 fell on palm_up, 87 past 60 N;
    - [ft4](#/week/6/run/20261023T092414_leg_eval_seed1_press_ft4_force_6998_hands): 178, palm_up 64, 29 past 120 N.
  - The 10 s stand: [round 4](#/week/6/run/20261023T061421_leg_eval_seed1_press_r4_force_5999_dogrest_stand10) 284 of 288 past 15° while waiting;
    [ft3a](#/week/6/run/20261023T074333_leg_eval_seed1_press_ft3a_force_6998_dogrest_stand10), [ft3c](#/week/6/run/20261023T083405_leg_eval_seed1_press_ft3c_force_7997_dogrest_stand10) and
    [ft4](#/week/6/run/20261023T092458_leg_eval_seed1_press_ft4_force_6998_dogrest_stand10) 0.
- **Scope:**
  - Simulation; one seed per recipe, 600-2000 iterations from round 4.
  - Every fine-tune shares ft2a's recipe (the dog's starts, the 16 N upward cap, round 4's layer), and ft2a already
    lost palm_up presses (108 of 144). Which of its parts costs that press is not separated.
  - Leg friction at 1 N·m costs every policy most of the hand presses (round 4: 84).
- **Implication:**
  - Round 4's crouch is how it presses the dog's hands cleanly in simulation. A level base costs that press with these
    recipes.
  - For the dog, two separate things: ft4 can stand under the live program's 15° stop (a stand-only test of the
    policy on the real legs); presses are round 4's, which needs the stop at ~40° (its largest tilt p95 36°).

### F-196 — The rattle the dog shows under round 4's foot-press policy is reproduced in simulation by lighter legs and a longer command delay: on the legs the policy trained on (armature 0.01 / 0.037) its targets change by 0.002-0.008 rad a step out to 25 ms of delay, but at UniFP's armature (0.0002 / 0.0007) with 15-20 ms they chatter on the dog's three joints at the dog's size (FL / RR thighs 0.27 / 0.22 rad rms, 2.4 rad/s; the dog 0.23 / 0.18, 1.6 / 2.3), and on those legs the policy falls in 124-135 of 288 presses as it lifts the foot

- **Status:** provisional (simulation against one live run's numbers); corrected 2026-10-24: the dog's rattle after
  the latency cut is 0.23 / 0.16 rad rms on the FL / RR thighs over the whole 10 s policy stand (the other PC's
  recalculation), not the 0.13 / 0.09 quoted below from a different time window. It lies at the 0.0002-armature row,
  not between it and 0.003. Its implication held on the dog ([F-225](#f-198))
- **Week:** 6
- **Date:** 2026-10-23
- **Evidence:** [Week 6 log, 2026-10-23](week_06/notes.md), "The rattle on the dog reproduced in simulation". The
  10 s stand from the dog's rest with actions dumped:
  - trained legs: [5-15 ms](#/week/6/run/20261023T111930_leg_eval_seed1_press_r4_force_5999_stand10_actions_delay1to3), [20 ms](#/week/6/run/20261023T112137_leg_eval_seed1_press_r4_force_5999_stand10_actions_delay4to4),
    [25 ms](#/week/6/run/20261023T112220_leg_eval_seed1_press_r4_force_5999_stand10_actions_delay5to5);
  - armature 0.003: [15-20 ms](#/week/6/run/20261023T112603_leg_eval_seed1_press_r4_force_5999_stand10_arm0.003_delay3to4);
  - armature 0.0002: [5-15 ms](#/week/6/run/20261023T112519_leg_eval_seed1_press_r4_force_5999_stand10_arm0.0002_delay1to3), [15-20 ms](#/week/6/run/20261023T112646_leg_eval_seed1_press_r4_force_5999_stand10_arm0.0002_delay3to4).
  The dog's numbers are the other PC's analysis of its run (2026-10-23).
- **Scope:**
  - The dog's armature and loop delay are not measured; the match identifies a direction, not values.
  - One live run.
  - The falls on light legs are in the frozen v2 set's presses, not the dog's hands.
- **Implication:**
  - The rattle is the policy's sensitivity to legs lighter than it trained on, made worse by delay. The latency cut
    helped for that reason, and a filter on the dog would treat the symptom.
  - The fix that should hold for the press is a fine-tune with the armature drawn down to ~0.0002 and the delay out to
    ~25 ms. Score it on the 10 s stand's chatter and on the dog's hands on light legs.
  - Until then, round 4 should not press on the dog: on light legs it tips as it lifts the foot.

### F-197 — Fine-tuned on rima's legs and regularisation (leg armature 0, no command delay, unitree_rl_lab's observation noise and penalties; geared knee kept), round 4's foot-press policy stops chattering on every leg model tried (FL thigh 0.002-0.005 rad rms a step while standing, where round 4 shows 0.27 on light legs at 15-20 ms) and stops falling on light legs (the dog's hands 0 falls, against round 4's 62-83), but presses less accurately: the dog's hands 36-225 of 288 depending on legs and checkpoint, against round 4's 280 on the legs it trained on

- **Status:** provisional (simulation; one run, 1000 iterations, not converged)
- **Week:** 6
- **Date:** 2026-10-24
- **Evidence:** [Week 6 log, 2026-10-24](week_06/notes.md), "ft6 on rima's legs".
  - [The run](#/week/6/run/20261023T123207_train_seed3_press_ft6_force_s3).
  - Standing: [light legs](#/week/6/run/20261023T130733_leg_eval_seed1_press_ft6_force_6998_stand10_light), [trained legs](#/week/6/run/20261023T131152_leg_eval_seed1_press_ft6_force_6998_stand10).
  - The hands: [trained legs](#/week/6/run/20261023T131108_leg_eval_seed1_press_ft6_force_6998_hands), [light](#/week/6/run/20261023T130900_leg_eval_seed1_press_ft6_force_6998_hands_light),
    [light, at 6600](#/week/6/run/20261023T131440_leg_eval_seed1_press_ft6_force_6600_hands_light).
  - Round 4's light-leg baselines: [the hands](#/week/6/run/20261023T122815_leg_eval_seed1_press_r4_force_5999_hands_light),
    [the stand](#/week/6/run/20261023T122649_leg_eval_seed1_press_r4_force_5999_stand10_light).
- **Scope:**
  - Simulation.
  - Not converged: training's force error still falling at 10 N.
  - rima's floor friction and base mass randomisation not replicated; the knee not rima's.
  - The dog's legs are not measured (F-196).
- **Implication:**
  - Training on rima's legs and regularisation is what removes the chatter in simulation. Whether it removes it on
    the dog is the stand-only test.
  - If it does, the press needs more training on these legs, and its accuracy has to be scored on the dog's hands.

### F-225 — On the real dog the foot-press policy fine-tuned on rima's legs (ft6) stands its 10 s without the rattle: its leg targets change 0.006-0.012 rad a step on the three worst joints, where round 4's changed 0.118-0.229, its joints move at 0.14-0.34 rad/s instead of ~1.0, and it tilts at most 16.6 degrees nose up (round 4: 20.8; simulation predicted 21-22)

- **Status:** confirmed (one live run on the dog, the stand only). Numbered F-198 when written (commit 8d99415, another machine); renumbered F-225 on 2026-10-25 when merged with the robot program's F-198 to F-224, written in parallel. The same stand test as F-204, recorded twice.
- **Week:** 6
- **Date:** 2026-10-24
- **Evidence:** the other PC's run of `d1dogpress --policy checkpoints/leg_press_force_ft6_model_6998.npz --live
  --presses 0 --start_on_blend --tilt_limit_deg 40 --start_tol_rad 0.30 --start_pitch_deg -15 5` and its analysis
  (2026-10-24; the run's record not yet committed to this repository), against round 4's rattling stand recalculated
  the same way.
  - Over the 10 s policy stand, the FL thigh, RR thigh and RL calf:
    - target change a step: round 4's 0.229 / 0.160 / 0.118 rad, ft6's 0.012 / 0.006 / 0.012;
    - joint speed: round 4's 1.03 / 1.03 / 1.01 rad/s, ft6's 0.14 / 0.21 / 0.34.
  - The most nose-up tilt: 20.8° against 16.6°.
  - The stand-up, the 10 s, the lie-down and the handback to sport mode all completed. A first attempt stopped before
    anything moved: the dog was lying ("the base is 0.070 m up").
  - Simulation (F-197): ft6 0.002-0.005 rad a step on every leg model, round 4 0.27 / 0.22 on light legs at 15-20 ms.
- **Scope:**
  - One live run, the stand only.
  - The dog changes ft6's targets 2-6 times more than simulation does, but 15-25 times less than round 4's.
  - Presses not tried: in simulation ft6 presses less accurately than round 4 (F-197).
- **Implication:**
  - The rattle came from the policy having trained on legs heavier, and a loop faster, than the dog's (F-196). rima's
    legs and regularisation remove it on the dog.
  - The press has to be trained on those settings before it is tried on the dog.

### F-226 — On light legs with a long command delay the force-aware foot-press policy's own estimate of the force on its foot reads a press that is not there, pressing straight down: on the palm under the foot (5 and 8 N) the real force stays at 0-0.2 N while the estimate reads 2.9-7.4 N, so the policy holds still; on the legs it trained on the same estimate tracks the force (3.1-8.1 N against 3.5-9.1)

- **Status:** provisional (simulation; one policy, the estimate decoded offline from dumped frames). Numbered F-199 when written (commit 8d99415, another machine); renumbered F-226 on 2026-10-25 when merged with the robot program's F-198 to F-224, written in parallel.
- **Week:** 6
- **Date:** 2026-10-24
- **Evidence:** [Week 6 log, 2026-10-24](week_06/notes.md), "Round C" and "The force estimate on light legs".
  - ft10b's final on the dog's hands with the first 96 episodes' frames dumped (all on the palm under the foot):
    [trained legs](#/week/6/run/20261023T191658_leg_eval_seed1_press_ft10b_force_10595_hands_dump), [armature 0.0002 at 15-20 ms](#/week/6/run/20261023T191331_leg_eval_seed1_press_ft10b_force_10595_hands_light_dump).
  - The decoder's foot force (`leg_policy.decode_estimates`) is rebuilt from each step's 32-frame history and
    compared with the plate's normal force over the hold (medians over episodes):

    | legs | rigid 5 N | rigid 8 N | sprung 5 N | sprung 8 N |
    | --- | --- | --- | --- | --- |
    | trained | 3.5 real / 3.1 estimated | 9.0 / 8.1 | 3.9 / 3.5 | 9.1 / 7.9 |
    | light | 0.0 / 3.2 | 0.2 / 7.4 | 0.0 / 2.9 | 0.0 / 4.3 |

  - The light-leg presses of every rima-style fine-tune fail on that palm (0-9 of 144, F-197 onward), the palm in
    front succeeding (83-113).
- **Scope:**
  - One policy (ft10b's final), one palm.
  - The estimate is the decoder's, which the actor also reads through its latent; the actor's own reading is not
    separable.
  - Whether the real dog's legs are this light is not measured (F-196).
- **Implication:**
  - Pressing straight down, the force a force-aware policy believes in is the force it does not apply when its leg
    model is wrong.
  - On the dog the decoder's estimate already read 19-59 N lying (F-187). A downward press there needs either training
    on the dog's own leg corner or a measured force: the Go2's foot sensor, or the joint torques.

### F-227 — Trained on the dog's likely leg corner (armature 0-0.002, command delay 10-20 ms) with rima's regularisation and the dog's two hands oversampled, the foot-press policy (ft11a's final) presses those hands from the dog's rest on light legs without falling or rattling (149 of 288, 0 past 60 N; round 4: 10 with 83 falls and 166 past 60 N) and on zero-armature legs (108; round 4: 0, 62 falls), keeps 211 on the legs round 4 trained on (round 4: 280), and waits 12-14 degrees nose up; its weak press is the palm under the foot on light legs (27 of 144)

- **Status:** provisional (simulation; one lineage, checkpoints that swing by tens of presses). Numbered F-200 when written (commit 8d99415, another machine); renumbered F-227 on 2026-10-25 when merged with the robot program's F-198 to F-224, written in parallel.
- **Week:** 6
- **Date:** 2026-10-24
- **Evidence:** [Week 6 log, 2026-10-24](week_06/notes.md), rounds A-E and "The polish, a checkpoint sweep, and the
  candidate".
  - [The run](#/week/6/run/20261023T191133_train_seed3_press_ft11a_force_s3); its hands on [trained](#/week/6/run/20261023T202230_leg_eval_seed1_press_ft11a_force_11394_hands),
    [light](#/week/6/run/20261023T202315_leg_eval_seed1_press_ft11a_force_11394_hands_light), [zero-armature](#/week/6/run/20261023T202357_leg_eval_seed1_press_ft11a_force_11394_hands_zero)
    and [armature-0.003](#/week/6/run/20261023T230452_leg_eval_seed1_press_ft11a_force_11394_hands_mid) legs.
  - The stands: [trained](#/week/6/run/20261023T230245_leg_eval_seed1_press_ft11a_force_11394_stand10), [light](#/week/6/run/20261023T202438_leg_eval_seed1_press_ft11a_force_11394_stand10_light),
    [zero](#/week/6/run/20261023T230327_leg_eval_seed1_press_ft11a_force_11394_stand10_zero).
  - The frozen set: [classic stand](#/week/6/run/20261023T202521_leg_eval_seed1_press_ft11a_force_11394_real) (121 / 1 / 17),
    [light legs from the dog's rest](#/week/6/run/20261023T230409_leg_eval_seed1_press_ft11a_force_11394_dogrest_light) (49 / 0 / 94).
  - What did not help, from its lineage:
    - halving the joint-velocity noise, or this project's torque penalty (round A);
    - this project's `action_rate` (round B's ft9b);
    - a lower learning rate (ft13).
  - What helped: a little armature and delay in training (round B's ft9a: training force error 10 -> 6 N), the hands
    oversampled (round C), the leg corner (round D).
- **Scope:**
  - Simulation.
  - The dog's legs and loop delay are not measured: "light" is armature 0.0002 at 15-20 ms (F-196's match).
  - Friction at 1 N·m takes its light-leg hand presses to 28.
  - The choice of checkpoint departs from the rule stated before the sweep, for 140 more presses at the cost of 52
    more strikes past 60 N (on the trained and zero-armature legs, none on light).
  - Its export's `start_gate` is round 4's stands'.
- **Implication:**
  - For a press on the dog: this policy, the palm in front first. The palm under the foot needs a measured force
    (F-226).


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

### F-067 — UniFP's released B2Z1 training starts on Isaac Gym Preview 4, in under a day of setup, with two dependency pins its README does not give

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

### F-068 — The Go2+D1 retarget of UniFP trains, once the mass model is rebuilt into the URDF: the RViz drawing has no inertials, and Isaac Gym silently gives the robot almost no mass

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

### F-069 — UniFP's force commands are eight times what a D1 can produce: the port runs at ±8 N, not ±60 N

- **Status:** provisional (an arithmetic bound from published torque limits; no force has been measured
  on this arm, in simulation or on hardware)
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

### F-070 — A full-schedule UniFP run costs about 41 hours of this GPU, and its force curriculum does not begin until hour 5.5

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
  out by −2.8% — F-072.) `commands.force_start_step = 8000` gates every external force,
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

### F-071 — Resuming a UniFP run silently restarts its force curriculum, and `--max_iterations` on a resume means "train this many more", not "train up to this"

- **Status:** confirmed (both behaviours read from the source and the first measured directly)
- **Week:** 1
- **Date:** 2026-09-18
- **Evidence:** found while making the 42-hour run (F-070) survive a crash without a human present.
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

### F-072 — The Go2+D1 UniFP run finished all 60,000 iterations in 39.85 h, and the curve says most of it was wasted: learning is over by ~10,000, and a late destabilisation means the last checkpoint is not the best one

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
    25-iteration benchmark (F-070); out by −2.8%. `model_60000.pt` loads, stores `iter = 60000`,
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
     base-class value the released B2Z1 config never overrides (F-070).
  2. **"Train to the end, take the last checkpoint" is not safe here.** A run can destabilise after
     45,000 stable iterations. Reporting any UniFP-derived policy must state which checkpoint and
     how it was chosen.
  3. **The evaluator is now the blocking piece.** There is no frozen-manifest equivalent for this
     task, so neither the plateau, the checkpoint choice, nor the effect of the force curriculum can
     be settled — only observed in training reward. That, not more training, is what the next work
     on this should be.

### F-073 — The Go2 and the D1 disagree about which way is up, and one Isaac Gym flag cannot satisfy both: every rendering of this robot before 2026-09-20 was wrong, and none of the physics was

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
- **Implication:** **the 39.85-hour training run (F-072) is unaffected** — the policy never saw a
  visual mesh. What is affected is evidence made of pictures: any screenshot or video of this task
  taken before 2026-09-20 shows a robot in the wrong shape and should not be used or shown.
  More generally, a merged robot inherits a mesh convention per source, and a per-asset flag is the
  wrong shape of control for it — worth checking the first time this model is rendered in any new
  simulator, because nothing errors and the numbers all stay right.

### F-074 — First numbers off a trained Go2+D1 UniFP policy: 4.6 cm median tool-tip error undisturbed, 6.3 cm while being pushed, in a single clean rollout

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
  (F-072), which is selection on the training signal. Nothing here is a gate result.
- **Implication:** it is the first evidence that the policy does something — the training reward
  alone could not say whether 9 cm meant "tracking loosely" or "not tracking". It also shows the
  training-reward figure in F-072 (≥9.2 cm L1 lower bound) is a **population average under
  randomisation, noise and pushes**, roughly twice the error of a clean rollout, so the two numbers
  must not be quoted against each other. It does not move G1a or any other gate, and it will not
  until the task has a frozen evaluator with a zero-action reference — still the blocking piece.

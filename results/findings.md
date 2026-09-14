# Findings

One entry per conclusion, numbered in order found and never deleted. A later
result that changes a conclusion gets a new entry, and the old one is marked
`superseded` or `retracted` with a link. Status vocabulary is in the
[results README](README.md#conventions).

### F-001 — The walking policy carries the welded 3.152 kg D1 at 1 m/s forward

- **Status:** provisional
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

- **Status:** provisional
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
  randomisation values (opt-in) and a two-bus deploy manifest. Latency and D1 servo modelling remain open
  and need measurement (Thesis C system identification).

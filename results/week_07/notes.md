# Week 7 — 26 October – 1 November 2026

**Status:** in progress
**Focus:** Improve integrated repeatability and complete matched tool/force comparisons

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Complete reduced-suite P0–P4 training with seeds 42, 43, 44; evaluate held-out tool variations on validation cases.
- **Hardware and camera:** Repeat the tagged-box sequence across the declared placements; collect representative matched physical comparisons on applicable tasks.
- **Evidence and deliverable:** Keep task/perception interfaces fixed across low-level comparisons; distinguish tool diversity from generalisation and repeated trials from independent training seeds.
- **Gate focus:** G3 preparation; G7 repeatability.

## Checklist

- [ ] P0–P4 trained on the reduced suite with matched budgets and three seeds
- [ ] P3/P4 use identical applicable tool distributions and held-out variations
- [ ] Physical force-aware/position-only and tool comparisons collected where applicable
- [ ] Validation placement trials and failure analysis recorded
- [ ] Any insufficient H3 evidence labelled preliminary with explicit C follow-up

## Log

<!-- Dated entries (### YYYY-MM-DD · topic). For each: the exact command or change, the outcome in numbers,
     and what it does and does not show. Record runs with ./dashboard.py record. -->

### 2026-10-26 · Every policy that drives the legs, retrained on the dog's leg corner: the plan, the code, the queue

Lukas: "we fixed the armature and motor delay spec after trying out the foot press and seeing rattling. I fear that all
the other trained policies will also rattle, so I would like you to retrain all of them."

**The spec** is F-227's corner:
- Unitree's envelope with the geared knee (`--leg_actuator unitree_geared`);
- the leg armature drawn per environment, 0-0.002 kg·m² (knees ×3.67);
- a command delay of 10-20 ms (2-4 physics steps).

Only ft11a, the press candidate, has trained on it.

**What is retrained.** These are the checkpoints README's current policies. Each starts from its current checkpoint with
the recipe that made it, rebuilt from its recorded `arguments`, and only the legs change. The arm policies also get
today's default physical payload (lidar and wrist camera), which none of them trained with.

| policy | from | recipe | legs it trained on |
| --- | --- | --- | --- |
| reach | `..._cad333_model_58400` | week 4's c33ft (UniFP's, fingertip), 1500 iterations | UniFP's ideal PD, armature 2e-4, no delay |
| rope | hook v4 `13000` | F-133's v3, 2000 iterations, as `14600` was made (a polish of a finished rope policy erodes its recovery, F-136) | the same |
| mechanism | `..._mech_law_model_17499` | c33ft (force law, rate 1e-5), 1000 | the same |
| lever | `..._mech_lever_measured_model_21496` | c33ft (levers, high handles, off-path price), 1000 | the same |
| insertion | `..._insertion_seed3_measured_model_21400` | c33ft (compliant grip, rate 1e-5), 800 | the same |
| key, force-aware and position-only | F-179's ft045 finals (`2604`, `2613`) | ft045's, plus rima's regularisation as ft11a (observation noise, `action_rate` -0.1, torques -2e-4), 2000 | `unitree_geared`, armature 0.003-0.02, 5-15 ms |
| press, force-aware and position-only | round 4's finals (`5999`, seed 3) | ft11a's final recipe in one stage, 3000: the matched pair the light-leg work never had | the same as the key |

**Not retrained:**
- P2Dingo's walking policy: third party, trained on a bare Go2, and the dog walks on sport mode;
- the Isaac Gym reference;
- superseded checkpoints;
- ft11a.

**The code:**
- **The leg model is any task's.** `--leg_actuator`, `--leg_delay_*`, `--leg_armature*`, and the per-environment
  armature and friction draws, moved from `Go2D1LegEnv` to `Go2D1PosForceEnv`.
  - Commands without them build what they built: `swap_legs` returns the group untouched for `unifp` with no delay or
    armature.
  - Every run's `run.json` now records `legs` and `leg_joints_loaded`.
- **A frozen manifest survives a command delay.** Isaac Lab's delayed actuator draws each environment's lag from the
  global stream at reset, and the reach set's goal schedule draws from that stream too. In `eval` and `manifest` that
  reset is now forked off it (`isolate_actuator_rng`).
- **Every evaluation writes `leg_chatter.json`** (`unifp_train/leg_chatter.py`). It records each leg joint's target
  change a policy step (rms, `d1`) and the change of that change (`d2`, about twice `d1` for a target flipping every
  step).
  - It is pooled over all steps, over the standing ones (no velocity command) and, in the leg task, over the waiting
    stage.
  - It counts from each episode's 30th step, with the base upright.
  - Tests: `tests/test_leg_chatter.py` (5).
- The tests that touch this pass (117: `test_leg_chatter`, `test_leg_actuators`, `test_unifp_train`, `test_foot_press`,
  `test_leg_shadow`, `test_dog_starts`, `test_evidence`, `test_unifp_interface`). The full suite was killed by the
  kernel's OOM killer three times beside the two key runs (the test process at ~11 GB of RAM), and was not run to
  completion today.

**The checks (development runs):**
- Smoke, the reach policy for 200 steps, 16 environments: [on the corner legs](#/week/7/run/20261025T132640_play_seed1_corner_smoke_reach),
  [on its trained legs](#/week/7/run/20261025T132736_play_seed1_corner_smoke_reach_trainedlegs).
  - The corner loaded: `unitree_geared` groups for the legs, `IdealPD` for the arm, environment 0's armature drawn at
    0.0005 / 0.0018.
  - **On its own legs, too, the UniFP policy moves its leg targets 0.04-0.11 rad a step** (d2/d1 ≈ 1): that is its
    gait under play's velocity commands. Hence the `standing` scope.
- **The reach validation set on light legs keeps its schedule.** 50 episodes, 0 schedule mismatches, 0 falls:
  - [trained legs](#/week/7/run/20261025T132858_eval_seed1_base_reach58400_trained): 1.68 / 2.30 cm quiet goal error, median / p90 (F-147 on the payload robot: 1.69 / 2.33);
  - [light legs](#/week/7/run/20261025T133028_eval_seed1_base_reach58400_light): 1.89 / 2.52 cm.
  - The chatter in these two runs was counted from each episode's first step. The battery below has the final numbers.
- **The tracker against the shadow dump** (ft11a's final, the 10 s stand on light legs, 288 episodes):
  - [from the first step](#/week/7/run/20261025T133251_leg_eval_seed1_check_ft11a_stand10_light): 0.017 rad over the episode, including the press;
  - [the waiting scope](#/week/7/run/20261025T133603_leg_eval_seed1_check2_ft11a_stand10_light): 0.014, with the settling from the spawn in it;
  - [from the 30th step](#/week/7/run/20261025T133916_leg_eval_seed1_check3_ft11a_stand10_light): 0.0033 / 0.0020 rad (FL / RR thigh), against the dump's 0.0032 / 0.0019 (F-227: 0.003 / 0.002).
- Training for 2 iterations on the corner legs, small PhysX, both complete:
  [press](#/week/7/run/20261025T134209_train_seed3_corner_smoke_press_train), [rope](#/week/7/run/20261025T134231_train_seed1_corner_smoke_rope_train).

**The queue** (`logs/unifp_train/corner/`) has two lanes.
- **Training:** `runner.sh` on `spool/`: the key pair, then the five arm policies alone at 4096, then the press pair.
  Each job hands its checkpoint sweep to the second lane when it ends.
- **Evaluation:** `runner_eval.sh` on `spool_eval/`, small PhysX beside the training.
- **Conditions:** every policy is scored on three leg models. T is the legs it trained on; for the key and press that is
  H, armature 0.01 / 0.037 at 5-15 ms. L is armature 0.0002 / 0.0007 at 15-20 ms, F-196's match to the rattle. Z is
  armature 0 at 5-15 ms.
- `lib.sh` holds the conditions and each lineage's frozen evaluation, as week 4 and F-179 / F-227 scored them.
- **The baseline** (job `e010`) answers the question first: every current checkpoint on T, L and Z.

**Selection, stated before any retrained checkpoint is scored.** Each rule is applied on L, the condition F-196 matched to
the dog.
- **reach:** the lowest p90 quiet goal error with no falls.
- **rope:** the rope rule (falls, then episodes with a tug, then force error, then holds) on the dev set. Then the
  held-out set once, for the one chosen.
- **mechanism:** the most opened on the test set, then the fewest falls.
- **lever:** the most opened on levers and the test set together, then the fewest falls.
- **insertion:** the most seated on lateral and detent together, then the fewest damaged.
- **key:** no falls, then the fewest contacts past 60 N, then the most unlocks.
- **press:** F-227's rule (no falls, then the fewest past 60 N, then the most presses).

A retrained checkpoint replaces the current one only if it is at least as good on Z as well, and if its standing
chatter on L is no higher.

### 2026-10-26 · The baseline: which current policies rattle on the dog's legs

Job `e010` (`logs/unifp_train/corner/`) ran every current checkpoint on its trained legs (T), light legs (L) and zero
armature (Z). Each lineage used its own frozen evaluation, as week 4 and F-179 scored them, with the physical payload
(today's default). Small PhysX, beside the key pair's training.

| policy, set, score | trained legs (T; H for the key) | light (L) | zero armature (Z) |
| --- | --- | --- | --- |
| reach 58400, validation (50): quiet goal error median / p90 | [1.68 / 2.30 cm, 0 fell](#/week/7/run/20261025T134455_eval_seed1_b_reach58400_T) | [1.89 / 2.52 cm, 0 fell](#/week/7/run/20261025T135523_eval_seed1_b_reach58400_L) | [2.01 / 2.84 cm, 0 fell](#/week/7/run/20261025T140942_eval_seed1_b_reach58400_Z) |
| rope 14600, dev (108): mean tension at 20 / 35 / 50 N | [25.3 / 36.0 / 50.0 N, 0 fell](#/week/7/run/20261025T134616_rope_eval_seed1_b_rope14600_dev_T) | [24.3 / 35.9 / 49.1 N, 0 fell](#/week/7/run/20261025T135722_rope_eval_seed1_b_rope14600_dev_L) | [25.3 / 36.1 / 50.0 N, 0 fell](#/week/7/run/20261025T141141_rope_eval_seed1_b_rope14600_dev_Z) |
| mechanism 17499, test set: opened of 512 | [476 / 512, 3 fell](#/week/7/run/20261025T134812_mech_eval_seed1_b_mech17499_test_T) | [470 / 512, 1 fell](#/week/7/run/20261025T140002_mech_eval_seed1_b_mech17499_test_L) | [466 / 512, 1 fell](#/week/7/run/20261025T141422_mech_eval_seed1_b_mech17499_test_Z) |
| lever 21496, levers: opened of 360 | [333 / 360, 0 fell](#/week/7/run/20261025T134931_mech_eval_seed1_b_lever21496_levers_T) | [346 / 360, 0 fell](#/week/7/run/20261025T140149_mech_eval_seed1_b_lever21496_levers_L) | [340 / 360, 0 fell](#/week/7/run/20261025T141609_mech_eval_seed1_b_lever21496_levers_Z) |
| lever 21496, test set: opened of 512 | [499 / 512, 0 fell](#/week/7/run/20261025T135048_mech_eval_seed1_b_lever21496_test_T) | [502 / 512, 0 fell](#/week/7/run/20261025T140333_mech_eval_seed1_b_lever21496_test_L) | [493 / 512, 0 fell](#/week/7/run/20261025T141753_mech_eval_seed1_b_lever21496_test_Z) |
| insertion 21400, lateral: seated of 96 | [71 / 96, 3 damaged](#/week/7/run/20261025T135204_ins_eval_seed1_b_ins21400_lat_T) | [70 / 96, 2 damaged](#/week/7/run/20261025T140521_ins_eval_seed1_b_ins21400_lat_L) | [70 / 96, 4 damaged](#/week/7/run/20261025T141941_ins_eval_seed1_b_ins21400_lat_Z) |
| insertion 21400, detent: seated of 96 | [68 / 96, 2 damaged](#/week/7/run/20261025T135344_ins_eval_seed1_b_ins21400_det_T) | [81 / 96, 0 damaged](#/week/7/run/20261025T140732_ins_eval_seed1_b_ins21400_det_L) | [73 / 96, 1 damaged](#/week/7/run/20261025T142153_ins_eval_seed1_b_ins21400_det_Z) |
| key force ft045 2604, F-179 set: unlocked / fell / past 60 N, of 108 | [101 / 0 / 0](#/week/7/run/20261025T142404_leg_eval_seed1_b_keyF2604_H) | [30 / 1 / 47](#/week/7/run/20261025T143211_leg_eval_seed1_b_keyF2604_L) | [0 / 4 / 96](#/week/7/run/20261025T144032_leg_eval_seed1_b_keyF2604_Z) |
| key position ft045 2613, the same | [30 / 0 / 75](#/week/7/run/20261025T142803_leg_eval_seed1_b_keyP2613_H) | [13 / 9 / 51](#/week/7/run/20261025T143615_leg_eval_seed1_b_keyP2613_L) | [1 / 4 / 93](#/week/7/run/20261025T144459_leg_eval_seed1_b_keyP2613_Z) |

The chatter in the same runs: the worst leg joint's rms target change a step while standing (rad), its d2/d1 in
brackets (2.0 is a flip every step), and the episodes with any leg joint past 0.1 rad. For the key it is the support
thighs (FL / RR) over the whole episode instead, because its waiting stage is shorter than the 30 steps skipped and its
worst joint is the front-right leg doing the turn.

| policy, set | T (H) | L | Z |
| --- | --- | --- | --- |
| reach 58400 | 0.024 FR thigh (×1.4), 0 / 39 | 0.049 FR hip (×1.1), 2 / 39 | 0.024 FR thigh (×1.4), 0 / 39 |
| rope 14600 | 0.012 RL calf (×0.8), 0 / 108 | 0.012 RL calf (×1.0), 0 / 108 | 0.013 RL calf (×1.0), 0 / 108 |
| mechanism 17499 | 0.045 FL calf (×1.0), 41 / 512 | 0.044 FL calf (×1.0), 36 / 512 | 0.047 FL calf (×1.0), 36 / 512 |
| lever 21496, levers | 0.085 FL calf (×1.7), 74 / 360 | 0.084 FL calf (×1.5), 104 / 360 | 0.084 FL calf (×1.6), 72 / 360 |
| lever 21496, test set | 0.106 FL calf (×1.9), 216 / 512 | 0.119 RR thigh (×2.0), 292 / 512 | 0.117 FL calf (×1.9), 269 / 512 |
| insertion 21400, lateral | 0.025 FL calf (×2.0), 0 / 96 | 0.110 FL calf (×2.0), 55 / 96 | 0.077 FL calf (×2.0), 18 / 96 |
| insertion 21400, detent | 0.025 FL calf (×2.0), 0 / 96 | 0.120 FL calf (×2.0), 63 / 96 | 0.070 FL calf (×2.0), 16 / 96 |
| key force 2604, FL / RR thigh | 0.003 / 0.005 | 0.022 / 0.028 | 0.017 / 0.035 |
| key position 2613, FL / RR thigh | 0.014 / 0.008 | 0.090 / 0.081 | 0.090 / 0.090 |

The dog's rattle under round 4 was 0.16-0.23 rad on the FL and RR thighs. Still policies give 0.002-0.012 (F-196,
F-225).

**Lukas's fear holds for three of the six:**
- **Insertion rattles in the press's way.** Its FL calf flips its target every step, even on its trained legs (0.025
  rad, d2/d1 2.0). On light legs that grows to 0.11-0.12 rad, in 55-63 of 96 episodes past 0.1. In simulation the
  task does not suffer (70 and 81 seated of 96 on L, against 71 and 68).
- **The lever policy is jumpy on its own legs** (FL calf 0.085-0.106 rad, d2/d1 1.7-1.9) and a little worse on light
  legs (0.119, 292 of 512 episodes past 0.1). It opens as many.
- **The key policies collapse on light legs.** Force-aware: 101 unlocked on its trained legs, 30 on L with 47 contacts
  past 60 N, 0 on Z. Their support thighs move 5-10 times more. F-179 already found 2 of 108 at UniFP's 2e-4 armature.

**Three do not rattle:**
- **Rope:** 0.012 rad on every leg model, its tension unchanged.
- **Mechanism:** 0.044-0.047 rad, d2/d1 1.0, so its pulling motion and not a flip. It is the same on every leg model
  and opens 466-476.
- **Reach:** standing, its worst joint doubles on L (0.024 -> 0.049, 2 of 39 episodes past 0.1). It tracks 0.2-0.5 cm
  worse on L and Z.

**What this does not show:**
- Simulation only. L is F-196's match to one live run, not a measured leg.
- The arm policies' walking steps (the `all` scope, 0.12 rad for the reach) are their gait under UniFP's velocity
  commands, and are not judged here.
- On the payload robot the mechanism controller opens 476 of 512 on its own legs, where week 4 had 491 without the
  payload (the F-147 kind of cost).
- The PhysX demos are next (job `e012b`). `e012` was stopped: its demos ran out of GPU memory beside the key pair
  ([cup](#/week/7/run/20261025T145115_cup_unifp_wrist_seed1_bd_cup58400_T), [door](#/week/7/run/20261025T145127_combiner_mech_law_seed1_bd_door17499_T), [lever](#/week/7/run/20261025T145139_combiner_mech_law_seed1_bd_lever21496_T): PhysX could not allocate its
  contact buffers). The demo runners now take `--small_physx` and the leg flags, and write `leg_chatter.json`.

### 2026-10-26 · The PhysX demos of the current checkpoints on light legs

Job `e012b`: week 4's demo commands with `--small_physx` (new), on the robot the demos build (without the payload), on
the trained legs and on L. Successes, then the standing chatter (worst joint, d2/d1, episodes past 0.1 rad):

| demo | trained legs | light legs (L) |
| --- | --- | --- |
| cup, reach 58400 (16) | [10 / 16](#/week/7/run/20261025T145255_cup_unifp_wrist_seed1_bd_cup58400_T); FR hip 0.009 (×1.3), 0 | [7 / 16](#/week/7/run/20261025T151315_cup_unifp_wrist_seed1_bd_cup58400_L); FR hip 0.011 (×1.2), 0 |
| insertion, 21400, 2 mm, seed 2 (48) | [44 / 48](#/week/7/run/20261025T150806_insertion_policy_seed2_bd_ins21400_T); RL calf 0.032 (×2.0), 0 of 16 | [41 / 48](#/week/7/run/20261025T153026_insertion_policy_seed2_bd_ins21400_L); FL calf 0.056 (×2.0), 0 of 16 |
| combiner door 16 N·m, mechanism 17499 (16) | [0 / 16](#/week/7/run/20261025T145420_combiner_mech_law_seed1_bd_door17499_T); FL calf 0.043 (×2.0), 0 | [0 / 16](#/week/7/run/20261025T151512_combiner_mech_law_seed1_bd_door17499_L); RL thigh 0.027 (×2.0), 0 |
| lever demo, 21496 (16) | [0 / 16](#/week/7/run/20261025T150109_combiner_mech_law_seed1_bd_lever21496_T); FL calf 0.070 (×1.9), 0 | [0 / 16](#/week/7/run/20261025T152247_combiner_mech_law_seed1_bd_lever21496_L); FR calf 0.117 (×2.0), 13 of 16 |

- **The cup and insertion demos reproduce week 4 on the trained legs** (10 / 16 and 44 / 48, as on 2026-10-09).
  - Light legs cost the cup 3 and the insertion 3.
  - The insertion's flip shows in the demo too (×2.0).
- **The claw combiner demos open nothing on either legs, where week 4 opened 16 and 9.** The claw arrives 3.9 cm off the
  handle (week 4: 0.5 cm) and never hooks. The scene has changed since week 4: it took Lukas's measured handle (F-155),
  and F-157 found the L-lip claw does not fit the 40 mm lever.
  - So these two demos no longer score these policies.
  - **Not the small buffers:** the door demo with PhysX's default buffers
    ([run](#/week/7/run/20261025T172900_combiner_mech_law_seed1_bd_door17499_fullphysx_T), job `e013`) is the same
    0 / 16, the claw 3.908 cm off on arrival to the micrometre.
- **The lever demo's legs rattle on light legs** (13 of 16 episodes past 0.1 rad, FR calf ×2.0), as in its frozen set.

### 2026-10-26 · The key policies retrained on the dog's leg corner

Runs: [force-aware](#/week/7/run/20261025T130706_train_seed2_key_corner_force_s2) from ft045 `2604`,
[position-only](#/week/7/run/20261025T130836_train_seed2_key_corner_position_s2) from ft045 `2613`. Both trained 2000
iterations in parallel at 2048 environments (job `010`, 4.3 h).
- The recipe is ft045's: Lukas's turn, the light lock, the 30 N limit, 80% sport-mode stands.
- Changed: the corner legs (`unitree_geared`, armature 0-0.002, 10-20 ms) and rima's regularisation (observation noise,
  `action_rate` -0.1, torques -2e-4).

Job `e015` swept them on F-179's set from the classic stand. Each cell is unlocked / fell / past 60 N of 108, then the
support thighs' (FL / RR) rms target change a step over the episode, in rad:

| checkpoint | H (0.01 / 0.037, 5-15 ms) | L (0.0002 / 0.0007, 15-20 ms) | Z (0, 5-15 ms) |
| --- | --- | --- | --- |
| force ft045 2604 (original) | [101 / 0 / 0](#/week/7/run/20261025T142404_leg_eval_seed1_b_keyF2604_H), 0.003 / 0.005 | [30 / 1 / 47](#/week/7/run/20261025T143211_leg_eval_seed1_b_keyF2604_L), 0.022 / 0.028 | [0 / 4 / 96](#/week/7/run/20261025T144032_leg_eval_seed1_b_keyF2604_Z), 0.017 / 0.035 |
| force corner 3200 | [38 / 9 / 53](#/week/7/run/20261025T173437_leg_eval_seed1_r_keyF_3200_H), 0.009 / 0.021 | [77 / 0 / 22](#/week/7/run/20261025T173843_leg_eval_seed1_r_keyF_3200_L), 0.011 / 0.022 | [63 / 0 / 26](#/week/7/run/20261025T174235_leg_eval_seed1_r_keyF_3200_Z), 0.005 / 0.010 |
| force corner 3600 | [64 / 3 / 2](#/week/7/run/20261025T174619_leg_eval_seed1_r_keyF_3600_H), 0.012 / 0.032 | [103 / 0 / 5](#/week/7/run/20261025T175012_leg_eval_seed1_r_keyF_3600_L), 0.007 / 0.016 | [92 / 0 / 3](#/week/7/run/20261025T175404_leg_eval_seed1_r_keyF_3600_Z), 0.004 / 0.010 |
| force corner 4000 | [95 / 0 / 1](#/week/7/run/20261025T175752_leg_eval_seed1_r_keyF_4000_H), 0.006 / 0.013 | [107 / 0 / 0](#/week/7/run/20261025T180142_leg_eval_seed1_r_keyF_4000_L), 0.004 / 0.007 | [107 / 0 / 0](#/week/7/run/20261025T180530_leg_eval_seed1_r_keyF_4000_Z), 0.003 / 0.007 |
| force corner 4400 | [46 / 0 / 58](#/week/7/run/20261025T180919_leg_eval_seed1_r_keyF_4400_H), 0.007 / 0.013 | [64 / 0 / 44](#/week/7/run/20261025T181314_leg_eval_seed1_r_keyF_4400_L), 0.005 / 0.007 | [26 / 0 / 82](#/week/7/run/20261025T181709_leg_eval_seed1_r_keyF_4400_Z), 0.010 / 0.009 |
| force corner 4603 | [89 / 0 / 10](#/week/7/run/20261025T182117_leg_eval_seed1_r_keyF_4603_H), 0.007 / 0.012 | [104 / 0 / 4](#/week/7/run/20261025T182509_leg_eval_seed1_r_keyF_4603_L), 0.005 / 0.006 | [90 / 0 / 13](#/week/7/run/20261025T182857_leg_eval_seed1_r_keyF_4603_Z), 0.006 / 0.008 |
| position ft045 2613 (original) | [30 / 0 / 75](#/week/7/run/20261025T142803_leg_eval_seed1_b_keyP2613_H), 0.014 / 0.008 | [13 / 9 / 51](#/week/7/run/20261025T143615_leg_eval_seed1_b_keyP2613_L), 0.090 / 0.081 | [1 / 4 / 93](#/week/7/run/20261025T144459_leg_eval_seed1_b_keyP2613_Z), 0.090 / 0.090 |
| position corner 3200 | [17 / 8 / 72](#/week/7/run/20261025T183247_leg_eval_seed1_r_keyP_3200_H), 0.022 / 0.025 | [42 / 0 / 63](#/week/7/run/20261025T183655_leg_eval_seed1_r_keyP_3200_L), 0.014 / 0.010 | [16 / 0 / 62](#/week/7/run/20261025T184051_leg_eval_seed1_r_keyP_3200_Z), 0.013 / 0.013 |
| position corner 3600 | [37 / 0 / 57](#/week/7/run/20261025T184448_leg_eval_seed1_r_keyP_3600_H), 0.019 / 0.012 | [57 / 0 / 33](#/week/7/run/20261025T184849_leg_eval_seed1_r_keyP_3600_L), 0.015 / 0.010 | [1 / 0 / 94](#/week/7/run/20261025T185247_leg_eval_seed1_r_keyP_3600_Z), 0.022 / 0.016 |
| position corner 4000 | [22 / 1 / 81](#/week/7/run/20261025T185646_leg_eval_seed1_r_keyP_4000_H), 0.023 / 0.023 | [42 / 0 / 57](#/week/7/run/20261025T190056_leg_eval_seed1_r_keyP_4000_L), 0.017 / 0.013 | [7 / 0 / 93](#/week/7/run/20261025T190457_leg_eval_seed1_r_keyP_4000_Z), 0.020 / 0.024 |
| position corner 4400 | [18 / 0 / 83](#/week/7/run/20261025T190857_leg_eval_seed1_r_keyP_4400_H), 0.022 / 0.025 | [47 / 0 / 39](#/week/7/run/20261025T191305_leg_eval_seed1_r_keyP_4400_L), 0.012 / 0.013 | [13 / 0 / 86](#/week/7/run/20261025T191617_leg_eval_seed1_r_keyP_4400_Z), 0.017 / 0.023 |
| position corner 4612 | [8 / 1 / 93](#/week/7/run/20261025T192001_leg_eval_seed1_r_keyP_4612_H), 0.022 / 0.023 | [56 / 0 / 41](#/week/7/run/20261025T192348_leg_eval_seed1_r_keyP_4612_L), 0.014 / 0.013 | [8 / 0 / 85](#/week/7/run/20261025T192737_leg_eval_seed1_r_keyP_4612_Z), 0.020 / 0.026 |

- **The force-aware retrain turns the key on the dog's corner.**
  - By the rule stated before scoring, `4000` is chosen: on L no falls, none past 60 N, the most unlocks (107). It is
    better on Z too (107 against 0) and its support thighs move a fifth as much (0.004 / 0.007 rad against 0.022 /
    0.028).
  - On the old heavy legs it keeps 95 of 101.
- **Its checkpoints swing:** `4400` strikes past 60 N in 44-82 of 108 on every leg model. The last checkpoint is not
  the safe one by default, as F-227 found for the press.
- **The position-only retrain improves on L but still presses through the 60 N limit.**
  - `3600` by the rule: L 57 / 0 / 33, against its original's 13 / 9 / 51.
  - On Z it unlocks 1, as its original did, so it meets "as good on Z" only by a tie. Its later checkpoints rattle on
    Z (26-52 of 108 episodes past 0.1 rad).
- **On the dog's corner the force-aware lead holds:** 107 against 57 unlocked, 0 against 33 past 60 N (F-166 / F-179:
  190 against 43 of 216, 100-101 against 27-30 on the heavy legs).

**What this does not show:**
- The selection and the scores are on the same frozen set, as F-179's were.
- One seed.
- Simulation; L is F-196's match, not a measured leg.
- The 10 s stand before the turn is job `e016`.
- Nothing has been exported for the dog yet.

### 2026-10-26 · The reach policy retrained on the corner: better on the dog's legs, rattling on its old ones

[Run](#/week/7/run/20261025T172900_train_seed1_reach_corner_s1) (job `030`, 1.75 h): from `58400` (cad333), week 4's c33ft recipe, 1500
iterations at 4096, with the corner legs and the physical payload.

Job `e030` swept it on the validation set (50). Each cell is the quiet goal error, median / p90 in cm, then the standing
chatter: the worst joint's rms target change a step in rad, its d2/d1, and the episodes past 0.1 rad (of 39 standing):

| checkpoint | T (UniFP's PD, 2e-4, no delay) | L | Z |
| --- | --- | --- | --- |
| 58400 (original) | [1.68 / 2.30](#/week/7/run/20261025T134455_eval_seed1_b_reach58400_T); 0.024 ×1.4, 0 | [1.89 / 2.52](#/week/7/run/20261025T135523_eval_seed1_b_reach58400_L); 0.049 ×1.1, 2 | [2.01 / 2.84](#/week/7/run/20261025T140942_eval_seed1_b_reach58400_Z); 0.024 ×1.4, 0 |
| 58600 | [3.33 / 4.20](#/week/7/run/20261025T193127_eval_seed1_r_reach_58600_T); 0.024 ×1.1, 0 | [3.28 / 4.14](#/week/7/run/20261025T193242_eval_seed1_r_reach_58600_L); 0.024 ×1.1, 0 | [3.58 / 4.44](#/week/7/run/20261025T193429_eval_seed1_r_reach_58600_Z); 0.024 ×1.1, 0 |
| 58800 | [2.03 / 2.54](#/week/7/run/20261025T193615_eval_seed1_r_reach_58800_T); 0.060 ×2.0, 5 | [2.22 / 2.77](#/week/7/run/20261025T193728_eval_seed1_r_reach_58800_L); 0.022 ×1.1, 0 | [2.14 / 2.90, 1 fell](#/week/7/run/20261025T193914_eval_seed1_r_reach_58800_Z); 0.022 ×1.2, 0 |
| 59000 | [2.27 / 3.07](#/week/7/run/20261025T194102_eval_seed1_r_reach_59000_T); 0.122 ×2.0, 13 | [2.26 / 2.92](#/week/7/run/20261025T194215_eval_seed1_r_reach_59000_L); 0.024 ×1.1, 0 | [2.15 / 2.78, 1 fell](#/week/7/run/20261025T194402_eval_seed1_r_reach_59000_Z); 0.023 ×1.2, 0 |
| 59200 | [1.94 / 2.37](#/week/7/run/20261025T194549_eval_seed1_r_reach_59200_T); 0.072 ×2.0, 4 | [1.87 / 2.47](#/week/7/run/20261025T194703_eval_seed1_r_reach_59200_L); 0.022 ×1.4, 0 | [1.86 / 2.45](#/week/7/run/20261025T194849_eval_seed1_r_reach_59200_Z); 0.022 ×1.4, 0 |
| 59400 | [1.78 / 2.29](#/week/7/run/20261025T195035_eval_seed1_r_reach_59400_T); 0.171 ×2.0, 23 | [1.76 / 2.33](#/week/7/run/20261025T195149_eval_seed1_r_reach_59400_L); 0.023 ×1.2, 0 | [1.90 / 2.32](#/week/7/run/20261025T195336_eval_seed1_r_reach_59400_Z); 0.023 ×1.2, 0 |
| 59600 | [2.12 / 3.27](#/week/7/run/20261025T195523_eval_seed1_r_reach_59600_T); 0.079 ×2.0, 4 | [2.32 / 3.10](#/week/7/run/20261025T195637_eval_seed1_r_reach_59600_L); 0.022 ×1.2, 0 | [2.14 / 3.15](#/week/7/run/20261025T195824_eval_seed1_r_reach_59600_Z); 0.023 ×1.2, 0 |
| 59800 | [1.80 / 2.51](#/week/7/run/20261025T200009_eval_seed1_r_reach_59800_T); 0.177 ×2.0, 17 | [1.88 / 2.35](#/week/7/run/20261025T200124_eval_seed1_r_reach_59800_L); 0.023 ×1.2, 0 | [1.80 / 2.33](#/week/7/run/20261025T200310_eval_seed1_r_reach_59800_Z); 0.022 ×1.2, 0 |
| 59899 | [2.22 / 2.75](#/week/7/run/20261025T200457_eval_seed1_r_reach_59899_T); 0.146 ×2.0, 14 | [2.22 / 2.78](#/week/7/run/20261025T200611_eval_seed1_r_reach_59899_L); 0.023 ×1.2, 0 | [2.38 / 2.83](#/week/7/run/20261025T200759_eval_seed1_r_reach_59899_Z); 0.023 ×1.2, 0 |

- **By the rule, `59400`.** On L it has the lowest p90 with no falls: 1.76 / 2.33 cm, against the original's 1.89 /
  2.52.
  - It is better on Z (1.90 / 2.32 against 2.01 / 2.84).
  - Its standing chatter on L is half the original's (0.023 against 0.049).
- **On its old legs (T) every retrained checkpoint from 58800 on rattles,** the FR hip flipping every step (d2/d1 2.0):
  0.06-0.18 rad, in 4-23 of 39 standing episodes. The original does not (0.024). On the corner legs it is calm.
- T differs from the corner in two ways: no delay, and UniFP's ideal PD instead of `unitree_geared`. Which of the two
  the rattle rests on matters for the dog, whose delay is not measured. F-196's notes put the command 4 ms into the step
  after the latency cut.
  - Job `e017` sweeps it: light legs at a fixed 0 / 5 / 10 / 25 ms, and the ideal PD at 15-20 ms.
  - The same sweep covers the original reach, the key candidate and ft11a.
- **The cup demo** (16 placements; picked without spilling; jaw error on arrival):
  - **Without the payload** (the demos' robot until today), `59400` picks none:
    [0 / 16 on L](#/week/7/run/20261025T224504_cup_unifp_wrist_seed1_rd_cup59400_L) and [0 / 16 on Z](#/week/7/run/20261025T224645_cup_unifp_wrist_seed1_rd_cup59400_Z), the jaw 5.6 cm off on arrival.
    The original picks [10 / 16 on Z](#/week/7/run/20261025T224826_cup_unifp_wrist_seed1_bd_cup58400_Z) there. The retrain learnt its arm with the wrist camera's
    and bracket's +0.1 kg on Link6 and the lidar on the base.
  - **With the payload** (`--payload physical`, new in `demos/unifp/run_demo.py`), both reach policies are weak, and
    the retrain is no worse:
    - the original picks [4 / 16 on T](#/week/7/run/20261025T225205_cup_unifp_wrist_seed1_pd_cup58400_T), [3 / 16 on L](#/week/7/run/20261025T225318_cup_unifp_wrist_seed1_pd_cup58400_L) and [6 / 16 on Z](#/week/7/run/20261025T225456_cup_unifp_wrist_seed1_pd_cup58400_Z);
    - `59400` picks [3 / 16 on L](#/week/7/run/20261025T225550_cup_unifp_wrist_seed1_pd_cup59400_L) and [5 / 16 on Z](#/week/7/run/20261025T225732_cup_unifp_wrist_seed1_pd_cup59400_Z). It grasps 7-8 and
      spills more of them.
    - The jaw arrives 3.4-4.4 cm off for both.
  - So the cup demo on the real robot's masses is a weakness of the reach lineage, not of the retrain. The week 4
    10 / 16 was on a robot without the payload.

### 2026-10-26 · Delay sensitivity: the retrained policies rattle only with no delay at all

Job `e017`. Light legs (`unitree_geared`, 0.0002 / 0.0007) at a fixed command delay, and UniFP's ideal PD at 15-20 ms.
The cells:
- **reach:** validation quiet goal error, median / p90 in cm; then the standing chatter (worst joint rad, d2/d1,
  episodes past 0.1 rad of 39);
- **key:** after a 10 s stand, unlocked / fell / past 60 N of 108; then the waiting FL / RR thigh, in rad;
- **ft11a:** the waiting FL / RR thigh over its 10 s stand. Its presses after the stand are not a score: the 20 s
  episode leaves no time to press, and every press policy shows 0 there, as round 4's did in week 6.

| light legs at | reach retrain 59400 | reach original 58400 | key force corner 4000 | ft11a, 10 s stand |
| --- | --- | --- | --- | --- |
| 0 ms | [1.84 / 2.38](#/week/7/run/20261025T203107_eval_seed1_d_reach59400_D0); 0.141 ×2.0, 21 | [1.78 / 2.64](#/week/7/run/20261025T203255_eval_seed1_d_reach58400_D0); 0.024 ×1.4, 0 | [107 / 0 / 0](#/week/7/run/20261025T203442_leg_eval_seed1_d_keyF4000_stand10_D0); 0.003 / 0.006 | [0.003 / 0.002](#/week/7/run/20261025T203815_leg_eval_seed1_d_ft11a_stand10_D0) |
| 5 ms | [1.84 / 2.42](#/week/7/run/20261025T204028_eval_seed1_d_reach59400_D1); 0.023 ×1.2, 0 | [1.77 / 2.51](#/week/7/run/20261025T204213_eval_seed1_d_reach58400_D1); 0.024 ×1.4, 0 | [108 / 0 / 0](#/week/7/run/20261025T204401_leg_eval_seed1_d_keyF4000_stand10_D1); 0.003 / 0.006 | [0.004 / 0.002](#/week/7/run/20261025T204733_leg_eval_seed1_d_ft11a_stand10_D1) |
| 10 ms | [1.80 / 2.32](#/week/7/run/20261025T204944_eval_seed1_d_reach59400_D2); 0.022 ×1.2, 0 | [1.74 / 2.39](#/week/7/run/20261025T205129_eval_seed1_d_reach58400_D2); 0.025 ×1.3, 0 | [106 / 0 / 1](#/week/7/run/20261025T205315_leg_eval_seed1_d_keyF4000_stand10_D2); 0.003 / 0.006 | [0.004 / 0.002](#/week/7/run/20261025T205647_leg_eval_seed1_d_ft11a_stand10_D2) |
| 25 ms | [1.77 / 2.40](#/week/7/run/20261025T205857_eval_seed1_d_reach59400_D5); 0.023 ×1.2, 0 | [2.17 / 3.06](#/week/7/run/20261025T210042_eval_seed1_d_reach58400_D5); 0.067 ×1.0, 5 | [99 / 0 / 1](#/week/7/run/20261025T210229_leg_eval_seed1_d_keyF4000_stand10_D5); 0.003 / 0.006 | [0.004 / 0.002](#/week/7/run/20261025T210602_leg_eval_seed1_d_ft11a_stand10_D5) |
| ideal PD, 15-20 ms | [1.70 / 2.42](#/week/7/run/20261025T210814_eval_seed1_d_reach59400_U34); 0.023 ×1.2, 0 | [1.86 / 2.60](#/week/7/run/20261025T210945_eval_seed1_d_reach58400_U34); 0.058 ×1.1, 4 | [107 / 0 / 0](#/week/7/run/20261025T211115_leg_eval_seed1_d_keyF4000_stand10_U34); 0.003 / 0.006 | [0.003 / 0.002](#/week/7/run/20261025T211430_leg_eval_seed1_d_ft11a_stand10_U34) |

The same three on their old legs, from job `e016`:
- the key candidate after the stand: [97 / 0 / 0](#/week/7/run/20261025T202028_leg_eval_seed1_r_keyF_4000_stand10_H) on H,
  [107 / 0 / 0](#/week/7/run/20261025T200947_leg_eval_seed1_r_keyF_4000_stand10_L) on L;
- the original ft045 `2604`: [94 / 0 / 0](#/week/7/run/20261025T202737_leg_eval_seed1_b_keyF2604_stand10_H) on H,
  [28 / 0 / 29](#/week/7/run/20261025T201655_leg_eval_seed1_b_keyF2604_stand10_L) on L;
- the position candidate `3600`: [53 / 0 / 47](#/week/7/run/20261025T202405_leg_eval_seed1_r_keyP_3600_stand10_H) on H,
  [79 / 0 / 20](#/week/7/run/20261025T201321_leg_eval_seed1_r_keyP_3600_stand10_L) on L.

- **The reach retrain's rattle rests on zero delay, not on the actuator.** At 0 ms its FL hip flips every step (0.141
  rad, d2/d1 2.0, 21 of 39). At 5, 10 and 25 ms, and on the ideal PD at 15-20 ms, it is calm (0.022-0.023).
- **The original reach is the mirror image:** calm at 0-10 ms, jumpy at 25 ms and on the ideal PD at 15-20 ms
  (0.058-0.067, 4-5 of 39).
- **The key candidate and ft11a are calm at every delay from 0 to 25 ms** (support thighs 0.002-0.006 rad). The key
  unlocks 99-108 of 108 at all of them.
- **What it means for the dog:**
  - The dog's loop cannot be at 0 ms: the command leaves 4 ms into the step after the latency cut (F-196's notes),
    then crosses DDS to the motors.
  - So the zero-delay rattle is a simulation hazard: any playback or demo of a retrained arm policy without the delay
    shows it. Run retrained policies with the corner legs.
  - The dog's delay itself is still not measured.

### 2026-10-26 · The rope policy retrained on the corner: no better on the dog's legs, so `14600` stays

[Run](#/week/7/run/20261025T191424_train_seed1_rope_corner_s1) (job `040`, 2.4 h): F-133's v3 recipe from hook v4 `13000`, 2000
iterations at 4096, as `14600` (cad333) was made but on the corner legs and with the physical payload.

Job `e040` swept it on the dev set (108). Each cell is the mean tension at 20 / 35 / 50 N; the episodes with a tug in
the hold, as the rope rule counts them; and the falls after a release. Then the standing chatter (worst joint rad,
d2/d1). Checkpoints before `13600` were still climbing the curriculum (`13200` held 108 of 324):

| checkpoint | T (UniFP's PD, no delay) | L | Z |
| --- | --- | --- | --- |
| 14600 cad333 (original) | [25.3 / 36.0 / 50.0; 15; 0](#/week/7/run/20261025T134616_rope_eval_seed1_b_rope14600_dev_T); 0.012 ×0.8 | [24.3 / 35.9 / 49.1; 9; 0](#/week/7/run/20261025T135722_rope_eval_seed1_b_rope14600_dev_L); 0.012 ×1.0 | [25.3 / 36.1 / 50.0; 15; 0](#/week/7/run/20261025T141141_rope_eval_seed1_b_rope14600_dev_Z); 0.013 ×1.0 |
| 13600 | [28.1 / 37.5 / 49.7; 94; 0](#/week/7/run/20261025T215234_rope_eval_seed1_r_rope_dev_13600_T); 0.022 ×2.0 | [28.1 / 37.5 / 50.6; 84; 0](#/week/7/run/20261025T215415_rope_eval_seed1_r_rope_dev_13600_L); 0.014 ×1.6 | [28.3 / 37.9 / 51.3; 91; 0](#/week/7/run/20261025T215635_rope_eval_seed1_r_rope_dev_13600_Z); 0.016 ×1.7 |
| 13800 | [29.1 / 38.2 / 50.6; 142; 0](#/week/7/run/20261025T215855_rope_eval_seed1_r_rope_dev_13800_T); 0.142 ×2.0 | [29.0 / 35.8 / 46.0; 87; 0](#/week/7/run/20261025T220035_rope_eval_seed1_r_rope_dev_13800_L); 0.016 ×1.6 | [29.1 / 36.6 / 47.4; 120; 0](#/week/7/run/20261025T220253_rope_eval_seed1_r_rope_dev_13800_Z); 0.020 ×1.7 |
| 14000 | [27.2 / 37.5 / 51.8; 126; 0](#/week/7/run/20261025T220513_rope_eval_seed1_r_rope_dev_14000_T); 0.173 ×2.0 | [28.0 / 36.8 / 50.5; 52; 0](#/week/7/run/20261025T220654_rope_eval_seed1_r_rope_dev_14000_L); 0.019 ×1.7 | [27.9 / 37.2 / 51.6; 49; 0](#/week/7/run/20261025T220914_rope_eval_seed1_r_rope_dev_14000_Z); 0.019 ×1.7 |
| 14200 | [27.6 / 38.0 / 52.9; 172; 0](#/week/7/run/20261025T221132_rope_eval_seed1_r_rope_dev_14200_T); 0.200 ×2.0 | [28.1 / 37.0 / 51.6; 66; 0](#/week/7/run/20261025T221313_rope_eval_seed1_r_rope_dev_14200_L); 0.021 ×1.6 | [27.9 / 37.4 / 52.1; 76; 0](#/week/7/run/20261025T221532_rope_eval_seed1_r_rope_dev_14200_Z); 0.024 ×1.7 |
| 14400 | [27.6 / 39.1 / 53.5; 181; 0](#/week/7/run/20261025T221751_rope_eval_seed1_r_rope_dev_14400_T); 0.250 ×2.0 | [27.9 / 38.1 / 53.2; 70; 0](#/week/7/run/20261025T221932_rope_eval_seed1_r_rope_dev_14400_L); 0.020 ×1.6 | [27.6 / 37.9 / 52.9; 90; 0](#/week/7/run/20261025T222151_rope_eval_seed1_r_rope_dev_14400_Z); 0.027 ×1.8 |
| 14600 | [27.6 / 39.3 / 53.4; 176; 0](#/week/7/run/20261025T222412_rope_eval_seed1_r_rope_dev_14600_T); 0.248 ×2.0 | [27.5 / 37.6 / 52.9; 69; 0](#/week/7/run/20261025T222553_rope_eval_seed1_r_rope_dev_14600_L); 0.023 ×1.7 | [27.3 / 37.4 / 53.3; 67; 0](#/week/7/run/20261025T222812_rope_eval_seed1_r_rope_dev_14600_Z); 0.036 ×1.9 |
| 14800 | [27.1 / 39.8 / 54.6; 213; 0](#/week/7/run/20261025T223031_rope_eval_seed1_r_rope_dev_14800_T); 0.283 ×2.0 | [27.4 / 37.0 / 51.4; 59; 0](#/week/7/run/20261025T223211_rope_eval_seed1_r_rope_dev_14800_L); 0.022 ×1.6 | [27.2 / 37.0 / 52.1; 80; 0](#/week/7/run/20261025T223428_rope_eval_seed1_r_rope_dev_14800_Z); 0.046 ×1.9 |
| 14999 | [27.8 / 39.4 / 53.4; 208; 0](#/week/7/run/20261025T223644_rope_eval_seed1_r_rope_dev_14999_T); 0.311 ×2.0 | [27.3 / 36.0 / 49.5; 74; 0](#/week/7/run/20261025T223824_rope_eval_seed1_r_rope_dev_14999_L); 0.020 ×1.6 | [27.0 / 36.1 / 50.4; 90; 0](#/week/7/run/20261025T224045_rope_eval_seed1_r_rope_dev_14999_Z); 0.055 ×2.0 |

- **The retrain tugs and over-pulls more than the policy it would replace.**
  - On L, 52-87 of 108 episodes tug in the hold, against the original's 9.
  - It pulls 27-29 N for 20 N, against 24.3.
  - Every checkpoint from `13600` holds with no fall after a release, as the original does.
- **By the rope rule (falls, then tugged episodes, then force error) the original stays.** It was already calm on the
  dog's corner (F-228: 0.012 rad on every leg model). No retrained checkpoint replaces it.
- **The retrain develops F-230's zero-delay rattle, and it grows with training.** The RR calf flips on T: 0.022 rad at
  `13600`, 0.31 at `14999`, in 108 of 108 episodes. From `14800` it reaches Z too (0.046-0.055, ×1.9, 8-9 of 108).
  The original shows none of it.
- **The leg-task retrains, with rima's regularisation, are calm at every delay; the arm retrains, without it, are not.**
  The leg tasks trained with `action_rate` -0.1 and observation noise; the arm tasks with UniFP's -0.02 and no noise.
  That is the next thing to test (round 2, below).

### 2026-10-26 · The mechanism controller retrained on the corner: worse, so `17499` stays

[Run](#/week/7/run/20261025T214010_train_seed1_mech_corner_s1) (job `050`, 1.25 h): from `17499`, week 4's c33ft recipe (force law, rate
pinned at 1e-5, peak ceiling 60 N), 1000 iterations at 4096, with the corner legs and the physical payload.

Job `e050` swept it on the held-out test set. Each cell is opened of 512 with the button's count (of 128) and the
falls; then the standing chatter (worst joint rad, d2/d1, episodes past 0.1 rad of 512):

| checkpoint | T (UniFP's PD, no delay) | L | Z |
| --- | --- | --- | --- |
| 17499 (original) | [476, button 92; 3](#/week/7/run/20261025T134812_mech_eval_seed1_b_mech17499_test_T); 0.045 ×1.0, 41 | [470, button 86; 1](#/week/7/run/20261025T140002_mech_eval_seed1_b_mech17499_test_L); 0.044 ×1.0, 36 | [466, button 82; 1](#/week/7/run/20261025T141422_mech_eval_seed1_b_mech17499_test_Z); 0.047 ×1.0, 36 |
| 17800 | [448, button 100; 0](#/week/7/run/20261025T225912_mech_eval_seed1_r_mech_test_17800_T); 0.040 ×1.4, 11 | [429, button 79; 0](#/week/7/run/20261025T230024_mech_eval_seed1_r_mech_test_17800_L); 0.034 ×1.3, 16 | [414, button 82; 0](#/week/7/run/20261025T230157_mech_eval_seed1_r_mech_test_17800_Z); 0.037 ×1.3, 8 |
| 18000 | [371, button 86; 0](#/week/7/run/20261025T230330_mech_eval_seed1_r_mech_test_18000_T); 0.049 ×1.8, 38 | [369, button 63; 0](#/week/7/run/20261025T230440_mech_eval_seed1_r_mech_test_18000_L); 0.032 ×1.4, 2 | [354, button 73; 0](#/week/7/run/20261025T230613_mech_eval_seed1_r_mech_test_18000_Z); 0.037 ×1.5, 0 |
| 18200 | [387, button 68; 0](#/week/7/run/20261025T230746_mech_eval_seed1_r_mech_test_18200_T); 0.096 ×2.0, 148 | [374, button 54; 0](#/week/7/run/20261025T230855_mech_eval_seed1_r_mech_test_18200_L); 0.035 ×1.4, 3 | [380, button 67; 0](#/week/7/run/20261025T231030_mech_eval_seed1_r_mech_test_18200_Z); 0.041 ×1.6, 5 |
| 18400 | [390, button 64; 0](#/week/7/run/20261025T231203_mech_eval_seed1_r_mech_test_18400_T); 0.146 ×2.0, 375 | [358, button 45; 0](#/week/7/run/20261025T231312_mech_eval_seed1_r_mech_test_18400_L); 0.032 ×1.4, 0 | [371, button 54; 0](#/week/7/run/20261025T231444_mech_eval_seed1_r_mech_test_18400_Z); 0.041 ×1.6, 5 |
| 18498 | [424, button 63; 0](#/week/7/run/20261025T231618_mech_eval_seed1_r_mech_test_18498_T); 0.127 ×2.0, 227 | [405, button 43; 0](#/week/7/run/20261025T231729_mech_eval_seed1_r_mech_test_18498_L); 0.037 ×1.5, 2 | [418, button 50; 0](#/week/7/run/20261025T231901_mech_eval_seed1_r_mech_test_18498_Z); 0.046 ×1.6, 8 |

- **Every retrained checkpoint opens fewer than the original on every leg model:** 358-429 on L against 470, with the
  doors and drawers down from 128 to 93-123.
  - It is week 4's pattern again: every fine-tune of this controller opened fewer (481-443 against 498, F-127; c33ft's
    best tied it).
  - The original was already calm on the dog's corner (F-228: 0.044 rad, d2/d1 1.0).
  - **So `17499` stays.**
- The retrain develops F-230's zero-delay rattle (FR hip ×2.0 on T, in up to 375 of 512 episodes), and not on the
  corner.

### 2026-10-26 · The lever policy retrained on the corner: less rattle, but it falls on levers, so `21496` stays for now

[Run](#/week/7/run/20261025T225523_train_seed1_lever_corner_s1) (job `060`, 1 h): from `21496`, week 4's c33ft recipe (levers 30%, high
handles, the off-path price, peak ceiling 80 N), 1000 iterations at 4096, with the corner legs and the physical payload.

Job `e060` swept it. Each cell is levers opened of 360 and the falls there, then the test set opened of 512 and its
falls. After those, the test set's standing chatter: worst joint rad, d2/d1, episodes past 0.1 rad of 512.

| checkpoint | T (UniFP's PD, no delay) | L | Z |
| --- | --- | --- | --- |
| 21496 (original) | [333; 0](#/week/7/run/20261025T134931_mech_eval_seed1_b_lever21496_levers_T) / [499; 0](#/week/7/run/20261025T135048_mech_eval_seed1_b_lever21496_test_T); 0.106 ×1.9, 216 | [346; 0](#/week/7/run/20261025T140149_mech_eval_seed1_b_lever21496_levers_L) / [502; 0](#/week/7/run/20261025T140333_mech_eval_seed1_b_lever21496_test_L); 0.119 ×2.0, 292 | [340; 0](#/week/7/run/20261025T141609_mech_eval_seed1_b_lever21496_levers_Z) / [493; 0](#/week/7/run/20261025T141753_mech_eval_seed1_b_lever21496_test_Z); 0.117 ×1.9, 269 |
| 21800 | [334; 17](#/week/7/run/20261025T235435_mech_eval_seed1_r_lever_levers_21800_T) / [487; 0](#/week/7/run/20261026T001509_mech_eval_seed1_r_lever_test_21800_T); 0.186 ×2.0, 251 | [347; 34](#/week/7/run/20261025T235544_mech_eval_seed1_r_lever_levers_21800_L) / [493; 0](#/week/7/run/20261026T001618_mech_eval_seed1_r_lever_test_21800_L); 0.050 ×1.8, 13 | [333; 14](#/week/7/run/20261025T235715_mech_eval_seed1_r_lever_levers_21800_Z) / [486; 0](#/week/7/run/20261026T001749_mech_eval_seed1_r_lever_test_21800_Z); 0.053 ×1.8, 16 |
| 22000 | [333; 8](#/week/7/run/20261025T235846_mech_eval_seed1_r_lever_levers_22000_T) / [498; 0](#/week/7/run/20261026T001919_mech_eval_seed1_r_lever_test_22000_T); 0.104 ×2.0, 221 | [337; 17](#/week/7/run/20261025T235952_mech_eval_seed1_r_lever_levers_22000_L) / [507; 0](#/week/7/run/20261026T002028_mech_eval_seed1_r_lever_test_22000_L); 0.049 ×1.8, 1 | [329; 11](#/week/7/run/20261026T000122_mech_eval_seed1_r_lever_levers_22000_Z) / [494; 0](#/week/7/run/20261026T002159_mech_eval_seed1_r_lever_test_22000_Z); 0.056 ×1.8, 27 |
| 22200 | [337; 0](#/week/7/run/20261026T000252_mech_eval_seed1_r_lever_levers_22200_T) / [503; 0](#/week/7/run/20261026T002329_mech_eval_seed1_r_lever_test_22200_T); 0.137 ×2.0, 293 | [342; 4](#/week/7/run/20261026T000357_mech_eval_seed1_r_lever_levers_22200_L) / [499; 0](#/week/7/run/20261026T002439_mech_eval_seed1_r_lever_test_22200_L); 0.039 ×1.6, 3 | [341; 1](#/week/7/run/20261026T000526_mech_eval_seed1_r_lever_levers_22200_Z) / [509; 0](#/week/7/run/20261026T002609_mech_eval_seed1_r_lever_test_22200_Z); 0.043 ×1.6, 13 |
| 22400 | [358; 33](#/week/7/run/20261026T000655_mech_eval_seed1_r_lever_levers_22400_T) / [507; 0](#/week/7/run/20261026T002741_mech_eval_seed1_r_lever_test_22400_T); 0.202 ×2.0, 366 | [357; 38](#/week/7/run/20261026T000802_mech_eval_seed1_r_lever_levers_22400_L) / [508; 0](#/week/7/run/20261026T002851_mech_eval_seed1_r_lever_test_22400_L); 0.049 ×1.7, 12 | [359; 34](#/week/7/run/20261026T000932_mech_eval_seed1_r_lever_levers_22400_Z) / [506; 0](#/week/7/run/20261026T003023_mech_eval_seed1_r_lever_test_22400_Z); 0.064 ×1.8, 70 |
| 22495 | [353; 18](#/week/7/run/20261026T001102_mech_eval_seed1_r_lever_levers_22495_T) / [498; 0](#/week/7/run/20261026T003155_mech_eval_seed1_r_lever_test_22495_T); 0.093 ×1.9, 172 | [355; 15](#/week/7/run/20261026T001209_mech_eval_seed1_r_lever_levers_22495_L) / [508; 0](#/week/7/run/20261026T003304_mech_eval_seed1_r_lever_test_22495_L); 0.053 ×1.7, 10 | [355; 24](#/week/7/run/20261026T001339_mech_eval_seed1_r_lever_levers_22495_Z) / [499; 0](#/week/7/run/20261026T003436_mech_eval_seed1_r_lever_test_22495_Z); 0.061 ×1.7, 36 |

- **The retrain rattles less on the dog's corner.** On L its test-set chatter is 0.039-0.053 rad, with 1-13 of 512
  episodes past 0.1. The original has 0.119 and 292.
- **It opens as many or more** (L: 493-508 of 512 and 337-357 of 360, against 502 and 346).
- **It falls on the lever set, on every leg model:** 4-38 of 360 on L, 8-33 on T. The original falls in none.
- **The rule stated this morning picks `22400`**: the most opened on both sets on L (865 against the original's 848),
  falls only as a tie-break. It falls 38 times on L where the original falls none.
- **I have not adopted it: `21496` stays.** That departs from the rule as written. A policy that falls where the one
  it replaces stands is not safer on the dog, whatever its rattle. Falls should have come before the count, as the
  rope rule has them.
- The retrain develops F-230's zero-delay rattle (FR hip ×2.0 on T, in up to 366 of 512 episodes).
- Next: a round 2 with whichever of the reach's round-2 changes holds at every delay, scored with falls first.

### 2026-10-26 · The insertion policy retrained on the corner: no rattle at all, but 30 fewer keys seated

[Run](#/week/7/run/20261025T235415_train_seed3_ins_corner_s3) (job `070`, 1 h): from seed 3's `21400`, week 4's c33ft recipe (compliant
grip, rate pinned at 1e-5, level 0.8, promotion every 200, arm std 0.15), 800 iterations at 4096, with the corner legs
and the physical payload.

Job `e070` swept it. Each cell is lateral seated of 96 and damaged, then detent seated and damaged. After those, the
lateral set's standing chatter: worst joint rad, d2/d1, episodes past 0.1 rad of 96.

| checkpoint | T (UniFP's PD, no delay) | L | Z |
| --- | --- | --- | --- |
| 21400 (original) | [71; 3](#/week/7/run/20261025T135204_ins_eval_seed1_b_ins21400_lat_T) / [68; 2](#/week/7/run/20261025T135344_ins_eval_seed1_b_ins21400_det_T); 0.025 ×2.0, 0 | [70; 2](#/week/7/run/20261025T140521_ins_eval_seed1_b_ins21400_lat_L) / [81; 0](#/week/7/run/20261025T140732_ins_eval_seed1_b_ins21400_det_L); 0.110 ×2.0, 55 | [70; 4](#/week/7/run/20261025T141941_ins_eval_seed1_b_ins21400_lat_Z) / [73; 1](#/week/7/run/20261025T142153_ins_eval_seed1_b_ins21400_det_Z); 0.077 ×2.0, 18 |
| 21600 | [56; 2](#/week/7/run/20261026T005407_ins_eval_seed1_r_ins_lat_21600_T) / [49; 4](#/week/7/run/20261026T011740_ins_eval_seed1_r_ins_det_21600_T); 0.005 ×1.7, 0 | [68; 0](#/week/7/run/20261026T005545_ins_eval_seed1_r_ins_lat_21600_L) / [44; 4](#/week/7/run/20261026T011920_ins_eval_seed1_r_ins_det_21600_L); 0.007 ×1.3, 0 | [59; 1](#/week/7/run/20261026T005753_ins_eval_seed1_r_ins_lat_21600_Z) / [48; 4](#/week/7/run/20261026T012129_ins_eval_seed1_r_ins_det_21600_Z); 0.005 ×1.5, 0 |
| 21800 | [30; 6](#/week/7/run/20261026T010005_ins_eval_seed1_r_ins_lat_21800_T) / [41; 9](#/week/7/run/20261026T012340_ins_eval_seed1_r_ins_det_21800_T); 0.006 ×1.6, 0 | [42; 1](#/week/7/run/20261026T010147_ins_eval_seed1_r_ins_lat_21800_L) / [47; 7](#/week/7/run/20261026T012522_ins_eval_seed1_r_ins_det_21800_L); 0.006 ×1.3, 0 | [33; 3](#/week/7/run/20261026T010355_ins_eval_seed1_r_ins_lat_21800_Z) / [41; 10](#/week/7/run/20261026T012733_ins_eval_seed1_r_ins_det_21800_Z); 0.004 ×1.5, 0 |
| 22000 | [81; 0](#/week/7/run/20261026T010604_ins_eval_seed1_r_ins_lat_22000_T) / [49; 0](#/week/7/run/20261026T012944_ins_eval_seed1_r_ins_det_22000_T); 0.005 ×1.8, 0 | [74; 1](#/week/7/run/20261026T010740_ins_eval_seed1_r_ins_lat_22000_L) / [45; 0](#/week/7/run/20261026T013123_ins_eval_seed1_r_ins_det_22000_L); 0.009 ×1.9, 0 | [83; 0](#/week/7/run/20261026T010946_ins_eval_seed1_r_ins_lat_22000_Z) / [45; 0](#/week/7/run/20261026T013331_ins_eval_seed1_r_ins_det_22000_Z); 0.005 ×1.6, 0 |
| 22199 | [43; 1](#/week/7/run/20261026T011147_ins_eval_seed1_r_ins_lat_22199_T) / [31; 1](#/week/7/run/20261026T013538_ins_eval_seed1_r_ins_det_22199_T); 0.005 ×1.6, 0 | [55; 2](#/week/7/run/20261026T011320_ins_eval_seed1_r_ins_lat_22199_L) / [39; 0](#/week/7/run/20261026T013715_ins_eval_seed1_r_ins_det_22199_L); 0.007 ×1.4, 0 | [53; 1](#/week/7/run/20261026T011529_ins_eval_seed1_r_ins_lat_22199_Z) / [45; 2](#/week/7/run/20261026T013921_ins_eval_seed1_r_ins_det_22199_Z); 0.006 ×1.5, 0 |

- **The rattle is gone on every leg model, at no delay too:** 0.004-0.009 rad. The original flips at 0.11-0.12 on L
  (55-63 of 96 episodes past 0.1). Unlike the reach, rope, mechanism and lever retrains, it has no zero-delay rattle.
  Its rate was pinned at 1e-5, a tenth of theirs, and its arm noise held at 0.15.
- **It seats fewer.** The best on L is `22000`: 74 lateral and 45 detent, against the original's 70 and 81. The detent
  set loses 36.
  - It is the decline every fine-tune of this lineage has shown, on either robot (F-121; week 4's c33ft: 46-59 and
    44-50).
- **By the rule the original stays** (151 seated on L against 119). But it is the one that rattles on the dog's corner.
  - Neither is clean for the dog: one rattles, the other seats 30 fewer.
  - That trade is Lukas's to choose. Round 2 should try to keep the seats.
- **The PhysX demo on the payload robot** (`--payload physical`, new in `demos/insertion/run_demo.py`; 2 mm, seed 2;
  standing chatter) shows the same trade:
  - the original: [43 / 48 on T](#/week/7/run/20261026T014305_insertion_policy_seed2_pd_ins21400_T) and [41 / 48 on L](#/week/7/run/20261026T014811_insertion_policy_seed2_pd_ins21400_L), its RL calf flipping
    (0.034 rad on L, ×1.9);
  - `22000`: [32 / 48 on L](#/week/7/run/20261026T015442_insertion_policy_seed2_pd_ins22000_L) and [27 / 48 on Z](#/week/7/run/20261026T020111_insertion_policy_seed2_pd_ins22000_Z), calm (0.005-0.006 rad).

### 2026-10-26 · Reach round 2: the delay drawn 5-20 ms keeps it calm at every delay

[Run](#/week/7/run/20261026T005349_train_seed1_reach_r2delay_s1) (job `080`, 1.55 h): job `030` again with the command delay drawn 5-20
ms (1-4 physics steps) instead of 10-20, everything else the same. Job `e080` swept it on T, L, Z and light legs at
0 ms. Each cell is the validation quiet goal error, median / p90 in cm, then the standing chatter (worst joint rad,
d2/d1, episodes past 0.1 rad of 39):

| checkpoint | T | L | Z | 0 ms (light legs) |
| --- | --- | --- | --- | --- |
| 58400 (original) | [1.68 / 2.30](#/week/7/run/20261025T134455_eval_seed1_b_reach58400_T); 0.024 ×1.4, 0 | [1.89 / 2.52](#/week/7/run/20261025T135523_eval_seed1_b_reach58400_L); 0.049 ×1.1, 2 | [2.01 / 2.84](#/week/7/run/20261025T140942_eval_seed1_b_reach58400_Z); 0.024 ×1.4, 0 | [1.78 / 2.64](#/week/7/run/20261025T203255_eval_seed1_d_reach58400_D0); 0.024 ×1.4, 0 |
| round 1 `59400` | [1.78 / 2.29](#/week/7/run/20261025T195035_eval_seed1_r_reach_59400_T); 0.171 ×2.0, 23 | [1.76 / 2.33](#/week/7/run/20261025T195149_eval_seed1_r_reach_59400_L); 0.023 ×1.2, 0 | [1.90 / 2.32](#/week/7/run/20261025T195336_eval_seed1_r_reach_59400_Z); 0.023 ×1.2, 0 | [1.84 / 2.38](#/week/7/run/20261025T203107_eval_seed1_d_reach59400_D0); 0.141 ×2.0, 21 |
| round 2 58800 | [2.08 / 3.17](#/week/7/run/20261026T023436_eval_seed1_r2d_reach_58800_T); 0.032 ×1.9, 1 | [2.32 / 3.44](#/week/7/run/20261026T023556_eval_seed1_r2d_reach_58800_L); 0.023 ×1.1, 0 | [2.17 / 3.07](#/week/7/run/20261026T023753_eval_seed1_r2d_reach_58800_Z); 0.023 ×1.2, 0 | [2.26 / 3.18](#/week/7/run/20261026T023949_eval_seed1_r2d_reach_58800_D0); 0.024 ×1.3, 0 |
| round 2 59000 | [2.27 / 2.77](#/week/7/run/20261026T024147_eval_seed1_r2d_reach_59000_T); 0.024 ×1.2, 0 | [2.22 / 2.74](#/week/7/run/20261026T024306_eval_seed1_r2d_reach_59000_L); 0.022 ×1.1, 0 | [2.23 / 2.75](#/week/7/run/20261026T024503_eval_seed1_r2d_reach_59000_Z); 0.023 ×1.1, 0 | [2.19 / 2.65](#/week/7/run/20261026T024700_eval_seed1_r2d_reach_59000_D0); 0.023 ×1.2, 0 |
| round 2 59200 | [2.04 / 2.80](#/week/7/run/20261026T024857_eval_seed1_r2d_reach_59200_T); 0.026 ×1.8, 0 | [2.13 / 2.61](#/week/7/run/20261026T025015_eval_seed1_r2d_reach_59200_L); 0.022 ×1.2, 0 | [2.13 / 2.97](#/week/7/run/20261026T025211_eval_seed1_r2d_reach_59200_Z); 0.022 ×1.1, 0 | [2.07 / 2.87](#/week/7/run/20261026T025406_eval_seed1_r2d_reach_59200_D0); 0.025 ×1.7, 0 |
| round 2 59400 | [2.08 / 2.71](#/week/7/run/20261026T025559_eval_seed1_r2d_reach_59400_T); 0.032 ×1.8, 1 | [2.27 / 3.11](#/week/7/run/20261026T025716_eval_seed1_r2d_reach_59400_L); 0.023 ×1.2, 0 | [2.06 / 2.92](#/week/7/run/20261026T025913_eval_seed1_r2d_reach_59400_Z); 0.024 ×1.2, 0 | [2.11 / 2.65](#/week/7/run/20261026T030109_eval_seed1_r2d_reach_59400_D0); 0.025 ×1.7, 0 |
| round 2 59600 | [1.94 / 2.53](#/week/7/run/20261026T030304_eval_seed1_r2d_reach_59600_T); 0.033 ×1.9, 1 | [1.93 / 2.57](#/week/7/run/20261026T030421_eval_seed1_r2d_reach_59600_L); 0.022 ×1.4, 0 | [1.92 / 2.60](#/week/7/run/20261026T030616_eval_seed1_r2d_reach_59600_Z); 0.023 ×1.2, 0 | [1.99 / 2.46](#/week/7/run/20261026T030812_eval_seed1_r2d_reach_59600_D0); 0.037 ×1.9, 1 |
| round 2 59800 | [2.13 / 2.75](#/week/7/run/20261026T031007_eval_seed1_r2d_reach_59800_T); 0.033 ×1.9, 1 | [2.08 / 2.61](#/week/7/run/20261026T031124_eval_seed1_r2d_reach_59800_L); 0.022 ×1.4, 0 | [2.19 / 2.64](#/week/7/run/20261026T031319_eval_seed1_r2d_reach_59800_Z); 0.022 ×1.1, 0 | [2.12 / 2.57](#/week/7/run/20261026T031515_eval_seed1_r2d_reach_59800_D0); 0.026 ×1.8, 0 |
| round 2 59899 | [2.11 / 2.88](#/week/7/run/20261026T031710_eval_seed1_r2d_reach_59899_T); 0.042 ×1.9, 3 | [2.13 / 2.66](#/week/7/run/20261026T031827_eval_seed1_r2d_reach_59899_L); 0.023 ×1.2, 0 | [2.03 / 3.01](#/week/7/run/20261026T032023_eval_seed1_r2d_reach_59899_Z); 0.022 ×1.2, 0 | [2.19 / 2.68](#/week/7/run/20261026T032218_eval_seed1_r2d_reach_59899_D0); 0.050 ×1.9, 1 |

- **Calm at every delay.** At 0 ms its worst joint is 0.025-0.042 rad, with 0-3 of 39 episodes past 0.1. Round 1's
  `59400` has 0.141 and 21. On T it is the same, so the zero-delay hazard of F-230 is gone.
- **It tracks about as the original does on the dog's corner,** a little worse than round 1.
  - By the rule (the lowest p90 on L with no falls) the choice is `59600`: 1.93 / 2.57 cm, against the original's
    1.89 / 2.52 and round 1's 1.76 / 2.33.
  - On Z it is better than the original: 1.92 / 2.60 against 2.01 / 2.84.
- **Its standing chatter on L is half the original's:** 0.022 against 0.049, with no episode past 0.1.
- **In the cup demo on the payload robot it picks the most of any reach policy:**
  [6 / 16 on L](#/week/7/run/20261026T032513_cup_unifp_wrist_seed1_pd_cup_r2d59600_L) and [7 / 16 on Z](#/week/7/run/20261026T032703_cup_unifp_wrist_seed1_pd_cup_r2d59600_Z), the jaw 2.2-2.5 cm off on
  arrival. The original picks 3 and 6 (3.4-4.4 cm off), round 1's `59400` 3 and 5.
- **The rule's replacement test:** at least as good on L and on Z, no more chatter. `59600` passes on Z and chatter. On
  L it is 0.04-0.05 cm worse, inside the sweep's own checkpoint-to-checkpoint spread of 0.2-0.5 cm. Read strictly, it
  does not pass on L.
- The other round-2 arm, `action_rate` -0.1 on the 10-20 ms corner (job `081`), is still to be scored.

### 2026-10-26 · Reach round 2, the other arm: action_rate -0.1 makes it the calmest, at some cost in tracking

[Run](#/week/7/run/20261026T022733_train_seed1_reach_r2rate_s1) (job `081`, 1.5 h): job `030` again, on the 10-20 ms corner, with UniFP's
`action_rate` at -0.1, the leg tasks' value (rima's), instead of -0.02. Job `e081` swept it; the cells are as above:

| checkpoint | T | L | Z | 0 ms (light legs) |
| --- | --- | --- | --- | --- |
| 58800 | [2.75 / 3.29](#/week/7/run/20261026T040309_eval_seed1_r2r_reach_58800_T); 0.022 ×1.1, 0 | [3.02 / 3.48](#/week/7/run/20261026T040423_eval_seed1_r2r_reach_58800_L); 0.022 ×1.1, 0 | [2.63 / 3.48](#/week/7/run/20261026T040609_eval_seed1_r2r_reach_58800_Z); 0.022 ×1.1, 0 | [2.90 / 3.43](#/week/7/run/20261026T040755_eval_seed1_r2r_reach_58800_D0); 0.022 ×1.1, 0 |
| 59000 | [2.13 / 2.82](#/week/7/run/20261026T040941_eval_seed1_r2r_reach_59000_T); 0.022 ×1.1, 0 | [2.20 / 2.78](#/week/7/run/20261026T041055_eval_seed1_r2r_reach_59000_L); 0.020 ×1.1, 0 | [2.24 / 3.00](#/week/7/run/20261026T041242_eval_seed1_r2r_reach_59000_Z); 0.021 ×1.1, 0 | [2.15 / 2.86, 1 fell](#/week/7/run/20261026T041429_eval_seed1_r2r_reach_59000_D0); 0.021 ×1.1, 0 |
| 59200 | [2.47 / 3.05](#/week/7/run/20261026T041615_eval_seed1_r2r_reach_59200_T); 0.023 ×1.1, 0 | [2.42 / 3.01, 1 fell](#/week/7/run/20261026T041727_eval_seed1_r2r_reach_59200_L); 0.022 ×1.1, 0 | [2.54 / 3.25](#/week/7/run/20261026T041914_eval_seed1_r2r_reach_59200_Z); 0.022 ×1.1, 0 | [2.39 / 3.11](#/week/7/run/20261026T042057_eval_seed1_r2r_reach_59200_D0); 0.022 ×1.1, 0 |
| 59400 | [1.93 / 2.54](#/week/7/run/20261026T042239_eval_seed1_r2r_reach_59400_T); 0.021 ×1.1, 0 | [2.19 / 3.04](#/week/7/run/20261026T042348_eval_seed1_r2r_reach_59400_L); 0.020 ×1.1, 0 | [2.20 / 3.02](#/week/7/run/20261026T042534_eval_seed1_r2r_reach_59400_Z); 0.021 ×1.1, 0 | [1.98 / 2.69](#/week/7/run/20261026T042720_eval_seed1_r2r_reach_59400_D0); 0.021 ×1.1, 0 |
| 59600 | [2.05 / 2.76](#/week/7/run/20261026T042904_eval_seed1_r2r_reach_59600_T); 0.020 ×1.1, 0 | [2.07 / 2.75](#/week/7/run/20261026T043016_eval_seed1_r2r_reach_59600_L); 0.020 ×1.1, 0 | [2.10 / 2.89](#/week/7/run/20261026T043203_eval_seed1_r2r_reach_59600_Z); 0.020 ×1.1, 0 | [2.03 / 2.62](#/week/7/run/20261026T043348_eval_seed1_r2r_reach_59600_D0); 0.020 ×1.1, 0 |
| 59800 | [2.16 / 2.85](#/week/7/run/20261026T043534_eval_seed1_r2r_reach_59800_T); 0.021 ×1.1, 0 | [2.25 / 2.98](#/week/7/run/20261026T043645_eval_seed1_r2r_reach_59800_L); 0.021 ×1.1, 0 | [2.24 / 2.95](#/week/7/run/20261026T043831_eval_seed1_r2r_reach_59800_Z); 0.021 ×1.1, 0 | [2.18 / 2.78](#/week/7/run/20261026T044016_eval_seed1_r2r_reach_59800_D0); 0.020 ×1.1, 0 |
| 59899 | [2.25 / 3.26](#/week/7/run/20261026T044202_eval_seed1_r2r_reach_59899_T); 0.020 ×1.1, 0 | [2.26 / 2.91](#/week/7/run/20261026T044314_eval_seed1_r2r_reach_59899_L); 0.020 ×1.1, 0 | [2.28 / 3.09](#/week/7/run/20261026T044459_eval_seed1_r2r_reach_59899_Z); 0.020 ×1.1, 0 | - |

- **The calmest reach policy on every leg model, with no delay too.** Its standing chatter is 0.020-0.023 rad with
  d2/d1 1.1 (no flip) and no episode past 0.1. Its walking steps change less too: 0.067-0.071 rad, against 0.08-0.09
  for every other reach policy.
- **It tracks worse than the delay arm.** By the rule `59600` is its choice: 2.07 / 2.75 cm on L, against the delay
  arm's 1.93 / 2.57 and the original's 1.89 / 2.52. A few checkpoints fall once in 50.
- **So either change cures F-230's zero-delay rattle.** The delay range costs less tracking. The leg tasks' calm at every
  delay (F-229) may be their `action_rate`.
- Next (job `086`): both together, before choosing the recipe for the rest. Lever round 2 (job `085`) has the delay
  change alone.

### 2026-10-26 · Lever round 2 (delay 5-20 ms): calmer, still falling on levers

[Run](#/week/7/run/20261026T035618_train_seed1_lever_r2delay_s1) (job `085`, 1.15 h): job `060` again with the delay drawn 5-20 ms. Job
`e085` swept it; the cells are as in round 1 (levers opened; falls / test set opened; falls; the test set's standing
chatter):

| checkpoint | T | L | Z | 0 ms (light legs) |
| --- | --- | --- | --- | --- |
| 21800 | [351; 51](#/week/7/run/20261026T050547_mech_eval_seed1_r2d_lever_levers_21800_T) / [461; 0](#/week/7/run/20261026T053805_mech_eval_seed1_r2d_lever_test_21800_T); 0.067 ×1.9, 97 | [351; 31](#/week/7/run/20261026T050707_mech_eval_seed1_r2d_lever_levers_21800_L) / [472; 0](#/week/7/run/20261026T053923_mech_eval_seed1_r2d_lever_test_21800_L); 0.051 ×1.7, 16 | [351; 72](#/week/7/run/20261026T050850_mech_eval_seed1_r2d_lever_levers_21800_Z) / [466; 0](#/week/7/run/20261026T054108_mech_eval_seed1_r2d_lever_test_21800_Z); 0.051 ×1.7, 20 | [350; 27](#/week/7/run/20261026T051035_mech_eval_seed1_r2d_lever_levers_21800_D0) / [445; 0](#/week/7/run/20261026T054253_mech_eval_seed1_r2d_lever_test_21800_D0); 0.064 ×1.9, 94 |
| 22000 | [340; 10](#/week/7/run/20261026T051220_mech_eval_seed1_r2d_lever_levers_22000_T) / [491; 0](#/week/7/run/20261026T054437_mech_eval_seed1_r2d_lever_test_22000_T); 0.071 ×1.8, 92 | [336; 6](#/week/7/run/20261026T051337_mech_eval_seed1_r2d_lever_levers_22000_L) / [507; 0](#/week/7/run/20261026T054556_mech_eval_seed1_r2d_lever_test_22000_L); 0.058 ×1.8, 17 | [338; 19](#/week/7/run/20261026T051520_mech_eval_seed1_r2d_lever_levers_22000_Z) / [496; 0](#/week/7/run/20261026T054741_mech_eval_seed1_r2d_lever_test_22000_Z); 0.062 ×1.7, 58 | [340; 4](#/week/7/run/20261026T051704_mech_eval_seed1_r2d_lever_levers_22000_D0) / [496; 0](#/week/7/run/20261026T054926_mech_eval_seed1_r2d_lever_test_22000_D0); 0.070 ×1.8, 94 |
| 22200 | [337; 36](#/week/7/run/20261026T051845_mech_eval_seed1_r2d_lever_levers_22200_T) / [441; 0](#/week/7/run/20261026T055111_mech_eval_seed1_r2d_lever_test_22200_T); 0.059 ×1.7, 34 | [341; 31](#/week/7/run/20261026T052004_mech_eval_seed1_r2d_lever_levers_22200_L) / [481; 0](#/week/7/run/20261026T055228_mech_eval_seed1_r2d_lever_test_22200_L); 0.052 ×1.7, 15 | [334; 43](#/week/7/run/20261026T052148_mech_eval_seed1_r2d_lever_levers_22200_Z) / [444; 0](#/week/7/run/20261026T055413_mech_eval_seed1_r2d_lever_test_22200_Z); 0.051 ×1.6, 16 | [344; 31](#/week/7/run/20261026T052333_mech_eval_seed1_r2d_lever_levers_22200_D0) / [451; 0](#/week/7/run/20261026T055557_mech_eval_seed1_r2d_lever_test_22200_D0); 0.058 ×1.8, 34 |
| 22400 | [340; 20](#/week/7/run/20261026T052518_mech_eval_seed1_r2d_lever_levers_22400_T) / [478; 0](#/week/7/run/20261026T055742_mech_eval_seed1_r2d_lever_test_22400_T); 0.048 ×1.5, 19 | [346; 16](#/week/7/run/20261026T052634_mech_eval_seed1_r2d_lever_levers_22400_L) / [503; 0](#/week/7/run/20261026T055900_mech_eval_seed1_r2d_lever_test_22400_L); 0.044 ×1.5, 6 | [335; 19](#/week/7/run/20261026T052816_mech_eval_seed1_r2d_lever_levers_22400_Z) / [481; 0](#/week/7/run/20261026T060046_mech_eval_seed1_r2d_lever_test_22400_Z); 0.048 ×1.4, 12 | [339; 16](#/week/7/run/20261026T053000_mech_eval_seed1_r2d_lever_levers_22400_D0) / [486; 0](#/week/7/run/20261026T060231_mech_eval_seed1_r2d_lever_test_22400_D0); 0.046 ×1.5, 16 |
| 22495 | [336; 10](#/week/7/run/20261026T053143_mech_eval_seed1_r2d_lever_levers_22495_T) / [428; 0](#/week/7/run/20261026T060416_mech_eval_seed1_r2d_lever_test_22495_T); 0.054 ×1.9, 55 | [340; 5](#/week/7/run/20261026T053258_mech_eval_seed1_r2d_lever_levers_22495_L) / [438; 0](#/week/7/run/20261026T060534_mech_eval_seed1_r2d_lever_test_22495_L); 0.043 ×1.8, 3 | [330; 15](#/week/7/run/20261026T053439_mech_eval_seed1_r2d_lever_levers_22495_Z) / [424; 0](#/week/7/run/20261026T060720_mech_eval_seed1_r2d_lever_test_22495_Z); 0.047 ×1.8, 16 | [330; 9](#/week/7/run/20261026T053622_mech_eval_seed1_r2d_lever_levers_22495_D0) / [436; 0](#/week/7/run/20261026T060905_mech_eval_seed1_r2d_lever_test_22495_D0); 0.055 ×1.9, 70 |

- **It still falls on levers on every leg model:** 5-31 of 360 on L, up to 72 on Z. Round 1 fell in 4-38; the original
  falls in none.
- **Where they fall, in both rounds:** at the high force levels (20-40 N of the 5-40 N steps), most often on levers
  turned from the tip side. The lever opens and then the robot topples. The original falls at none of the levels.
- **Its legs are calmer than the original's on the dog's corner:** 0.043-0.058 rad on L's test set, against 0.119.
  But the flip is not gone (d2/d1 1.4-1.8), and 34-94 of 512 episodes pass 0.1 rad at 0 ms.
- **No checkpoint qualifies with falls first**, so `21496` stays. Round 3 adds `action_rate` -0.1 (job `087`).

### 2026-10-26 · Reach round 3 (both changes), and the reach candidates side by side

[Run](#/week/7/run/20261026T050532_train_seed1_reach_r3both_s1) (job `086`, 1.5 h): the delay drawn 5-20 ms and `action_rate` -0.1
together. Job `e086` swept it; the cells are as above (its 0 ms run of `59899` did not finish before the sweep was read):

| checkpoint | T | L | Z | 0 ms (light legs) |
| --- | --- | --- | --- | --- |
| 58800 | [2.27 / 2.79](#/week/7/run/20261026T064154_eval_seed1_r3b_reach_58800_T); 0.022 ×1.0, 0 | [2.24 / 2.99](#/week/7/run/20261026T064309_eval_seed1_r3b_reach_58800_L); 0.021 ×1.0, 0 | [2.37 / 3.13](#/week/7/run/20261026T064456_eval_seed1_r3b_reach_58800_Z); 0.022 ×1.0, 0 | [2.23 / 2.76](#/week/7/run/20261026T064642_eval_seed1_r3b_reach_58800_D0); 0.022 ×1.1, 0 |
| 59000 | [2.20 / 3.21](#/week/7/run/20261026T064830_eval_seed1_r3b_reach_59000_T); 0.022 ×1.1, 0 | [2.10 / 2.78](#/week/7/run/20261026T064942_eval_seed1_r3b_reach_59000_L); 0.022 ×1.1, 0 | [2.43 / 4.33](#/week/7/run/20261026T065129_eval_seed1_r3b_reach_59000_Z); 0.022 ×1.0, 0 | [2.09 / 2.80](#/week/7/run/20261026T065315_eval_seed1_r3b_reach_59000_D0); 0.021 ×1.1, 0 |
| 59200 | [2.12 / 2.72](#/week/7/run/20261026T065502_eval_seed1_r3b_reach_59200_T); 0.021 ×1.1, 0 | [2.16 / 2.79](#/week/7/run/20261026T065615_eval_seed1_r3b_reach_59200_L); 0.020 ×1.1, 0 | [2.32 / 2.78](#/week/7/run/20261026T065802_eval_seed1_r3b_reach_59200_Z); 0.021 ×1.0, 0 | [2.04 / 2.56](#/week/7/run/20261026T065946_eval_seed1_r3b_reach_59200_D0); 0.021 ×1.1, 0 |
| 59400 | [2.56 / 3.16](#/week/7/run/20261026T070133_eval_seed1_r3b_reach_59400_T); 0.021 ×1.1, 0 | [2.51 / 3.04](#/week/7/run/20261026T070247_eval_seed1_r3b_reach_59400_L); 0.021 ×1.1, 0 | [2.59 / 3.30](#/week/7/run/20261026T070434_eval_seed1_r3b_reach_59400_Z); 0.021 ×1.1, 0 | [2.54 / 3.18](#/week/7/run/20261026T070620_eval_seed1_r3b_reach_59400_D0); 0.020 ×1.1, 0 |
| 59600 | [2.04 / 2.96](#/week/7/run/20261026T070808_eval_seed1_r3b_reach_59600_T); 0.021 ×1.1, 0 | [2.04 / 2.75](#/week/7/run/20261026T070920_eval_seed1_r3b_reach_59600_L); 0.020 ×1.1, 0 | [2.19 / 3.03, 1 fell](#/week/7/run/20261026T071106_eval_seed1_r3b_reach_59600_Z); 0.021 ×1.1, 0 | [1.98 / 2.61](#/week/7/run/20261026T071253_eval_seed1_r3b_reach_59600_D0); 0.021 ×1.1, 0 |
| 59800 | [2.33 / 2.78](#/week/7/run/20261026T071441_eval_seed1_r3b_reach_59800_T); 0.022 ×1.1, 0 | [2.46 / 2.99](#/week/7/run/20261026T071555_eval_seed1_r3b_reach_59800_L); 0.021 ×1.1, 0 | [2.36 / 2.82](#/week/7/run/20261026T071742_eval_seed1_r3b_reach_59800_Z); 0.021 ×1.1, 0 | [2.47 / 2.84](#/week/7/run/20261026T071929_eval_seed1_r3b_reach_59800_D0); 0.021 ×1.1, 0 |
| 59899 | [2.01 / 2.68](#/week/7/run/20261026T072115_eval_seed1_r3b_reach_59899_T); 0.021 ×1.1, 0 | [2.14 / 2.51](#/week/7/run/20261026T072227_eval_seed1_r3b_reach_59899_L); 0.021 ×1.1, 0 | [2.18 / 2.97](#/week/7/run/20261026T072412_eval_seed1_r3b_reach_59899_Z); 0.021 ×1.1, 0 | - |

- **It is as calm as the `action_rate` arm** (0.020-0.023 rad, d2/d1 1.1, at 0 ms too).
- By the rule it chooses `59899`: 2.14 / 2.51 cm on L, the original's p90. It is worse on Z (2.18 / 2.97 against
  2.01 / 2.84).

**The reach candidates side by side.** Each row is the rule's checkpoint from its run. The columns:
- the validation set on L and on Z: quiet goal error, median / p90 in cm;
- the standing chatter on L, and at 0 ms where it was run: worst joint rad and d2/d1;
- the walking chatter on L (`all` scope): worst joint rad;
- the cup demo on the payload robot: picked without spilling, of 16, on L / on Z.

| policy | L | Z | standing chatter, L | standing, 0 ms | walking, L | cup on L / Z |
| --- | --- | --- | --- | --- | --- | --- |
| 58400 (original) | 1.89 / 2.52 | 2.01 / 2.84 | 0.049 ×1.1 | 0.024 ×1.4 | 0.126 | [3](#/week/7/run/20261025T225318_cup_unifp_wrist_seed1_pd_cup58400_L) / [6](#/week/7/run/20261025T225456_cup_unifp_wrist_seed1_pd_cup58400_Z) |
| round 1 `59400` (10-20 ms) | 1.76 / 2.33 | 1.90 / 2.32 | 0.023 ×1.2 | **0.141 ×2.0** | 0.081 | [3](#/week/7/run/20261025T225550_cup_unifp_wrist_seed1_pd_cup59400_L) / [5](#/week/7/run/20261025T225732_cup_unifp_wrist_seed1_pd_cup59400_Z) |
| round 2 `59600` (5-20 ms) | 1.93 / 2.57 | 1.92 / 2.60 | 0.022 ×1.4 | 0.037 ×1.9 | 0.085 | [6](#/week/7/run/20261026T032513_cup_unifp_wrist_seed1_pd_cup_r2d59600_L) / [7](#/week/7/run/20261026T032703_cup_unifp_wrist_seed1_pd_cup_r2d59600_Z) |
| round 2 `59600` (`action_rate` -0.1) | 2.07 / 2.75 | 2.10 / 2.89 | 0.020 ×1.1 | 0.020 ×1.1 | 0.068 | [5](#/week/7/run/20261026T072807_cup_unifp_wrist_seed1_pd_cup_r2r59600_L) / [10](#/week/7/run/20261026T072948_cup_unifp_wrist_seed1_pd_cup_r2r59600_Z) |
| round 3 `59899` (both) | 2.14 / 2.51 | 2.18 / 2.97 | 0.021 ×1.1 | - | 0.069 | [1](#/week/7/run/20261026T073130_cup_unifp_wrist_seed1_pd_cup_r3b59899_L) / [2](#/week/7/run/20261026T073311_cup_unifp_wrist_seed1_pd_cup_r3b59899_Z) |

- **Round 1 is out:** it rattles at 0 ms (F-230).
- **Round 3 is out:** it picks 1-2 cups of 16, though its frozen set is as good.
- **The two round-2 arms both beat the original in the cup demo and in standing chatter on the dog's corner.**
  - The delay arm tracks the frozen set as the original does.
  - The `action_rate` arm is the calmest at every delay and smoothest walking, and picks the most cups on Z.
    It tracks 0.2 cm worse.
- **The candidate: round 2's delay arm `59600`** (`logs/unifp_train/20261026T005349_train_seed1_reach_r2delay_s1/model_59600.pt`), for
  its tracking. The `action_rate` arm (`logs/unifp_train/20261026T022733_train_seed1_reach_r2rate_s1/model_59600.pt`) is the alternative
  if calm matters more than 2 mm.
  - Neither is copied into `checkpoints/` or made the demos' default yet. That is for Lukas.
- **What this does not show:** simulation; one seed of each arm; the frozen set chose each run's checkpoint.

### 2026-10-26 · Lever round 3 (delay 5-20 ms and action_rate -0.1): calm and as able, but it still falls on levers

[Run](#/week/7/run/20261026T063521_train_seed1_lever_r3both_s1) (job `087`, 1.2 h): job `085` again with `action_rate` -0.1. Job `e087`
swept it; the cells are as in rounds 1 and 2:

| checkpoint | T | L | Z | 0 ms (light legs) |
| --- | --- | --- | --- | --- |
| 21496 (original) | [333; 0](#/week/7/run/20261025T134931_mech_eval_seed1_b_lever21496_levers_T) / [499; 0](#/week/7/run/20261025T135048_mech_eval_seed1_b_lever21496_test_T); 0.106 ×1.9, 216 | [346; 0](#/week/7/run/20261025T140149_mech_eval_seed1_b_lever21496_levers_L) / [502; 0](#/week/7/run/20261025T140333_mech_eval_seed1_b_lever21496_test_L); 0.119 ×2.0, 292 | [340; 0](#/week/7/run/20261025T141609_mech_eval_seed1_b_lever21496_levers_Z) / [493; 0](#/week/7/run/20261025T141753_mech_eval_seed1_b_lever21496_test_Z); 0.117 ×1.9, 269 | - |
| 21800 | [353; 11](#/week/7/run/20261026T074714_mech_eval_seed1_r3b_lever_levers_21800_T) / [507; 0](#/week/7/run/20261026T082052_mech_eval_seed1_r3b_lever_test_21800_T); 0.041 ×1.5, 24 | [354; 15](#/week/7/run/20261026T074809_mech_eval_seed1_r3b_lever_levers_21800_L) / [506; 1](#/week/7/run/20261026T082214_mech_eval_seed1_r3b_lever_test_21800_L); 0.040 ×1.4, 15 | [349; 20](#/week/7/run/20261026T074939_mech_eval_seed1_r3b_lever_levers_21800_Z) / [508; 0](#/week/7/run/20261026T082406_mech_eval_seed1_r3b_lever_test_21800_Z); 0.043 ×1.4, 22 | [352; 3](#/week/7/run/20261026T075129_mech_eval_seed1_r3b_lever_levers_21800_D0) / [509; 0](#/week/7/run/20261026T082557_mech_eval_seed1_r3b_lever_test_21800_D0); 0.040 ×1.5, 16 |
| 22000 | [349; 20](#/week/7/run/20261026T075319_mech_eval_seed1_r3b_lever_levers_22000_T) / [507; 0](#/week/7/run/20261026T082745_mech_eval_seed1_r3b_lever_test_22000_T); 0.037 ×1.5, 7 | [341; 35](#/week/7/run/20261026T075441_mech_eval_seed1_r3b_lever_levers_22000_L) / [508; 0](#/week/7/run/20261026T082905_mech_eval_seed1_r3b_lever_test_22000_L); 0.037 ×1.4, 10 | [345; 33](#/week/7/run/20261026T075634_mech_eval_seed1_r3b_lever_levers_22000_Z) / [507; 0](#/week/7/run/20261026T083058_mech_eval_seed1_r3b_lever_test_22000_Z); 0.039 ×1.4, 8 | [343; 16](#/week/7/run/20261026T075826_mech_eval_seed1_r3b_lever_levers_22000_D0) / [507; 0](#/week/7/run/20261026T083251_mech_eval_seed1_r3b_lever_test_22000_D0); 0.038 ×1.5, 6 |
| 22200 | [356; 7](#/week/7/run/20261026T080017_mech_eval_seed1_r3b_lever_levers_22200_T) / [511; 0](#/week/7/run/20261026T083443_mech_eval_seed1_r3b_lever_test_22200_T); 0.031 ×1.5, 6 | [348; 8](#/week/7/run/20261026T080137_mech_eval_seed1_r3b_lever_levers_22200_L) / [507; 0](#/week/7/run/20261026T083605_mech_eval_seed1_r3b_lever_test_22200_L); 0.031 ×1.1, 5 | [352; 6](#/week/7/run/20261026T080327_mech_eval_seed1_r3b_lever_levers_22200_Z) / [505; 0](#/week/7/run/20261026T083758_mech_eval_seed1_r3b_lever_test_22200_Z); 0.030 ×1.4, 4 | [354; 2](#/week/7/run/20261026T080517_mech_eval_seed1_r3b_lever_levers_22200_D0) / [509; 0](#/week/7/run/20261026T083951_mech_eval_seed1_r3b_lever_test_22200_D0); 0.032 ×1.2, 8 |
| 22400 | [345; 1](#/week/7/run/20261026T080708_mech_eval_seed1_r3b_lever_levers_22400_T) / [512; 0](#/week/7/run/20261026T084143_mech_eval_seed1_r3b_lever_test_22400_T); 0.035 ×1.3, 4 | [346; 5](#/week/7/run/20261026T080826_mech_eval_seed1_r3b_lever_levers_22400_L) / [507; 0](#/week/7/run/20261026T084305_mech_eval_seed1_r3b_lever_test_22400_L); 0.034 ×1.2, 4 | [351; 10](#/week/7/run/20261026T081017_mech_eval_seed1_r3b_lever_levers_22400_Z) / [507; 0](#/week/7/run/20261026T084458_mech_eval_seed1_r3b_lever_test_22400_Z); 0.036 ×1.2, 8 | [350; 4](#/week/7/run/20261026T081207_mech_eval_seed1_r3b_lever_levers_22400_D0) / [509; 0](#/week/7/run/20261026T084650_mech_eval_seed1_r3b_lever_test_22400_D0); 0.035 ×1.3, 2 |
| 22495 | [349; 11](#/week/7/run/20261026T081358_mech_eval_seed1_r3b_lever_levers_22495_T) / [488; 0](#/week/7/run/20261026T084842_mech_eval_seed1_r3b_lever_test_22495_T); 0.041 ×1.7, 0 | [349; 20](#/week/7/run/20261026T081517_mech_eval_seed1_r3b_lever_levers_22495_L) / [486; 0](#/week/7/run/20261026T085003_mech_eval_seed1_r3b_lever_test_22495_L); 0.035 ×1.7, 1 | [347; 23](#/week/7/run/20261026T081709_mech_eval_seed1_r3b_lever_levers_22495_Z) / [490; 0](#/week/7/run/20261026T085155_mech_eval_seed1_r3b_lever_test_22495_Z); 0.040 ×1.7, 2 | [343; 14](#/week/7/run/20261026T081901_mech_eval_seed1_r3b_lever_levers_22495_D0) / [484; 0](#/week/7/run/20261026T085346_mech_eval_seed1_r3b_lever_test_22495_D0); 0.040 ×1.8, 0 |

- **The calmest lever policy.** Its test-set chatter is 0.030-0.041 rad, d2/d1 1.1-1.5, against the original's 0.106-
  0.119 at ×1.9-2.0. It is calm at 0 ms too.
- **It opens as many or more:** levers 341-356 against 333-346; the test set up to 512 against 499-502.
- **It still falls on levers:** 1-35 of 360 across checkpoints and legs, at the high force levels, as rounds 1 and 2
  did. Its best on L is `22400`: 346 opened, 5 falls; the test set 507, none.
- **Falling there is this lineage's under any fine-tune.** Week 4's lever fine-tunes, on unchanged legs without the
  payload, fell 3-12 of 360 with the 3.33 N·m arm and 0-4 with the old one (week 4 log, "Lever"). The corner, the
  payload or both add to it.
- **With falls first, `21496` stays.** That leaves the dog's lever policy as the one that rattles on its corner (0.119
  rad). The trade, `22400`'s 5 falls in 360 against a rattle 3 times larger, is Lukas's.

### 2026-10-27 · The press pair on the corner in one stage: force-aware leads on every leg model, and neither reaches ft11a

Runs (job `090`, 4.5 h, both at 2048 in parallel):
- [force-aware](#/week/7/run/20261026T074711_train_seed3_press_corner_force_s3) from round 4's `5999`;
- [position-only](#/week/7/run/20261026T074843_train_seed3_press_corner_position_s3) from round 4's position `5999`.

Both ran ft11a's final recipe in one stage for 3000 iterations: the corner legs, rima's regularisation, the dog's hands at
30%, the 16 N upward cap, seed 3.

Job `e095` ran the dog battery. Presses of 288 / fell / past 60 N, from the dog's kp-25 rest, then the waiting chatter
over the 10 s stand on L:

| checkpoint | the hands, H | the hands, L | the hands, Z | 10 s stand on L, FL / RR thigh | frozen v2 set, classic stand, H |
| --- | --- | --- | --- | --- | --- |
| force 6600 | [272 / 0 / 32](#/week/7/run/20261026T121856_leg_eval_seed1_r_pressf_hands_6600_H) | [42 / 4 / 182](#/week/7/run/20261026T122036_leg_eval_seed1_r_pressf_hands_6600_L) | [1 / 0 / 260](#/week/7/run/20261026T122330_leg_eval_seed1_r_pressf_hands_6600_Z) | [0.007 / 0.004](#/week/7/run/20261026T125758_leg_eval_seed1_r_pressf_stand10_6600_L) | [84 / 12 / 25](#/week/7/run/20261026T131031_leg_eval_seed1_r_pressf_real_6600_H) |
| force 7200 | [263 / 0 / 37](#/week/7/run/20261026T122650_leg_eval_seed1_r_pressf_hands_7200_H) | [62 / 1 / 26](#/week/7/run/20261026T122923_leg_eval_seed1_r_pressf_hands_7200_L) | [122 / 0 / 57](#/week/7/run/20261026T123157_leg_eval_seed1_r_pressf_hands_7200_Z) | [0.006 / 0.003](#/week/7/run/20261026T130031_leg_eval_seed1_r_pressf_stand10_7200_L) | [109 / 18 / 18](#/week/7/run/20261026T131300_leg_eval_seed1_r_pressf_real_7200_H) |
| force 7800 | [246 / 0 / 80](#/week/7/run/20261026T123429_leg_eval_seed1_r_pressf_hands_7800_H) | [67 / 0 / 78](#/week/7/run/20261026T123700_leg_eval_seed1_r_pressf_hands_7800_L) | [86 / 0 / 160](#/week/7/run/20261026T123931_leg_eval_seed1_r_pressf_hands_7800_Z) | [0.005 / 0.002](#/week/7/run/20261026T130301_leg_eval_seed1_r_pressf_stand10_7800_L) | [116 / 4 / 21](#/week/7/run/20261026T131533_leg_eval_seed1_r_pressf_real_7800_H) |
| force 8400 | [216 / 0 / 55](#/week/7/run/20261026T124212_leg_eval_seed1_r_pressf_hands_8400_H) | [69 / 0 / 142](#/week/7/run/20261026T124505_leg_eval_seed1_r_pressf_hands_8400_L) | [118 / 0 / 97](#/week/7/run/20261026T124747_leg_eval_seed1_r_pressf_hands_8400_Z) | [0.002 / 0.002](#/week/7/run/20261026T130529_leg_eval_seed1_r_pressf_stand10_8400_L) | [106 / 4 / 34](#/week/7/run/20261026T131800_leg_eval_seed1_r_pressf_real_8400_H) |
| force 8998 | [278 / 0 / 6](#/week/7/run/20261026T125027_leg_eval_seed1_r_pressf_hands_8998_H) | [87 / 0 / 150](#/week/7/run/20261026T125255_leg_eval_seed1_r_pressf_hands_8998_L) | [95 / 2 / 98](#/week/7/run/20261026T125524_leg_eval_seed1_r_pressf_hands_8998_Z) | [0.003 / 0.002](#/week/7/run/20261026T130801_leg_eval_seed1_r_pressf_stand10_8998_L) | [101 / 2 / 37](#/week/7/run/20261026T132030_leg_eval_seed1_r_pressf_real_8998_H) |
| position 6600 | [3 / 0 / 27](#/week/7/run/20261026T132301_leg_eval_seed1_r_pressp_hands_6600_H) | [0 / 0 / 61](#/week/7/run/20261026T132530_leg_eval_seed1_r_pressp_hands_6600_L) | [3 / 0 / 243](#/week/7/run/20261026T132758_leg_eval_seed1_r_pressp_hands_6600_Z) | [0.005 / 0.004](#/week/7/run/20261026T140421_leg_eval_seed1_r_pressp_stand10_6600_L) | [89 / 0 / 53](#/week/7/run/20261026T141655_leg_eval_seed1_r_pressp_real_6600_H) |
| position 7200 | [4 / 0 / 51](#/week/7/run/20261026T133132_leg_eval_seed1_r_pressp_hands_7200_H) | [54 / 0 / 66](#/week/7/run/20261026T133425_leg_eval_seed1_r_pressp_hands_7200_L) | [39 / 0 / 169](#/week/7/run/20261026T133656_leg_eval_seed1_r_pressp_hands_7200_Z) | [0.004 / 0.003](#/week/7/run/20261026T140656_leg_eval_seed1_r_pressp_stand10_7200_L) | [102 / 0 / 55](#/week/7/run/20261026T141930_leg_eval_seed1_r_pressp_real_7200_H) |
| position 7800 | [75 / 0 / 29](#/week/7/run/20261026T134029_leg_eval_seed1_r_pressp_hands_7800_H) | [56 / 0 / 47](#/week/7/run/20261026T134302_leg_eval_seed1_r_pressp_hands_7800_L) | [35 / 3 / 138](#/week/7/run/20261026T134529_leg_eval_seed1_r_pressp_hands_7800_Z) | [0.004 / 0.002](#/week/7/run/20261026T140928_leg_eval_seed1_r_pressp_stand10_7800_L) | [72 / 0 / 39](#/week/7/run/20261026T142204_leg_eval_seed1_r_pressp_real_7800_H) |
| position 8400 | [0 / 0 / 5](#/week/7/run/20261026T134808_leg_eval_seed1_r_pressp_hands_8400_H) | [25 / 0 / 85](#/week/7/run/20261026T135033_leg_eval_seed1_r_pressp_hands_8400_L) | [7 / 0 / 204](#/week/7/run/20261026T135300_leg_eval_seed1_r_pressp_hands_8400_Z) | [0.003 / 0.002](#/week/7/run/20261026T141155_leg_eval_seed1_r_pressp_stand10_8400_L) | [102 / 0 / 38](#/week/7/run/20261026T142434_leg_eval_seed1_r_pressp_real_8400_H) |
| position 8998 | [28 / 0 / 34](#/week/7/run/20261026T135541_leg_eval_seed1_r_pressp_hands_8998_H) | [0 / 0 / 120](#/week/7/run/20261026T135814_leg_eval_seed1_r_pressp_hands_8998_L) | [5 / 0 / 206](#/week/7/run/20261026T140043_leg_eval_seed1_r_pressp_hands_8998_Z) | [0.007 / 0.003](#/week/7/run/20261026T141423_leg_eval_seed1_r_pressp_stand10_8998_L) | [92 / 0 / 56](#/week/7/run/20261026T142706_leg_eval_seed1_r_pressp_real_8998_H) |

For comparison (week 6, the same battery):
- ft11a's final: the hands 211 / 0 / 47 (H), 149 / 0 / 0 (L), 108 / 0 / 26 (Z); the frozen set 121 / 1 / 17;
- round 4's pair on the hands: force-aware 280 / 0 / 5 (H), 10 / 83 / 166 (L), 0 / 62 / 228 (Z); position-only
  0 / 0 / 63, 0 / 207 / 94 and 0 / 224 / 74.

- **By F-227's rule** (on L: no falls, then the fewest past 60 N, then the most presses), the choice is `7800` for each.
  - Force-aware: 67 / 0 / 78 on L, 246 / 0 / 80 on H, 86 / 0 / 160 on Z.
  - Position-only: 56 / 0 / 47, 75 / 0 / 29 and 35 / 3 / 138.
- **Force-aware leads on every leg model in this matched pair:** by 11 presses on light legs, 171 on the heavy ones, 51
  on zero armature. It also strikes past 60 N more on L (78 against 47).
- **On the frozen set from the classic stand the lead nearly vanishes,** and the force-aware side falls: 84-116 with
  2-18 falls, against 72-102 with none. Round 4's pair split 196 against 100 there.
- **Neither reaches ft11a, the five-stage lineage on the same final recipe:** 67 against its 149 on light legs.
  ft11a had ~5400 iterations through its stages (ft6 to ft11a) against these 3000, so this does not separate the stages
  from the length. ft11a stays the press candidate.
- **Neither rattles:** the waiting thighs are 0.002-0.007 rad.

### 2026-10-27 · The key retrain's second seed: the force-aware result replicates, and position-only can match it on light legs alone

Job `100` repeated job `010` with seed 3 (4.3 h): [force-aware](#/week/7/run/20261026T121844_train_seed3_key_corner_force_s3),
[position-only](#/week/7/run/20261026T122015_train_seed3_key_corner_position_s3). Job `e105` swept them. The cells are as for seed 2:
unlocked / fell / past 60 N of 108, then the support thighs' rms target change a step.

| checkpoint | H | L | Z |
| --- | --- | --- | --- |
| force 3200 | [71 / 0 / 34](#/week/7/run/20261026T163641_leg_eval_seed1_r3_keyF_3200_H), 0.004 / 0.007 | [75 / 3 / 27](#/week/7/run/20261026T164030_leg_eval_seed1_r3_keyF_3200_L), 0.010 / 0.016 | [50 / 0 / 20](#/week/7/run/20261026T164422_leg_eval_seed1_r3_keyF_3200_Z), 0.005 / 0.008 |
| force 3600 | [101 / 1 / 5](#/week/7/run/20261026T164814_leg_eval_seed1_r3_keyF_3600_H), 0.004 / 0.008 | [107 / 0 / 1](#/week/7/run/20261026T165202_leg_eval_seed1_r3_keyF_3600_L), 0.003 / 0.006 | [101 / 0 / 5](#/week/7/run/20261026T165553_leg_eval_seed1_r3_keyF_3600_Z), 0.005 / 0.007 |
| force 4000 | [83 / 3 / 14](#/week/7/run/20261026T165944_leg_eval_seed1_r3_keyF_4000_H), 0.007 / 0.010 | [91 / 0 / 17](#/week/7/run/20261026T170337_leg_eval_seed1_r3_keyF_4000_L), 0.006 / 0.007 | [73 / 0 / 26](#/week/7/run/20261026T170726_leg_eval_seed1_r3_keyF_4000_Z), 0.008 / 0.008 |
| force 4400 | [44 / 3 / 47](#/week/7/run/20261026T171117_leg_eval_seed1_r3_keyF_4400_H), 0.009 / 0.014 | [85 / 0 / 23](#/week/7/run/20261026T171521_leg_eval_seed1_r3_keyF_4400_L), 0.005 / 0.005 | [64 / 0 / 44](#/week/7/run/20261026T171914_leg_eval_seed1_r3_keyF_4400_Z), 0.008 / 0.006 |
| force 4603 | [61 / 7 / 19](#/week/7/run/20261026T172305_leg_eval_seed1_r3_keyF_4603_H), 0.012 / 0.019 | [87 / 0 / 21](#/week/7/run/20261026T172659_leg_eval_seed1_r3_keyF_4603_L), 0.007 / 0.005 | [103 / 0 / 4](#/week/7/run/20261026T173050_leg_eval_seed1_r3_keyF_4603_Z), 0.012 / 0.011 |
| position 3200 | [26 / 0 / 76](#/week/7/run/20261026T173442_leg_eval_seed1_r3_keyP_3200_H), 0.016 / 0.012 | [62 / 0 / 25](#/week/7/run/20261026T173845_leg_eval_seed1_r3_keyP_3200_L), 0.011 / 0.011 | [8 / 0 / 96](#/week/7/run/20261026T174243_leg_eval_seed1_r3_keyP_3200_Z), 0.021 / 0.024 |
| position 3600 | [55 / 0 / 52](#/week/7/run/20261026T174650_leg_eval_seed1_r3_keyP_3600_H), 0.012 / 0.013 | [107 / 0 / 0](#/week/7/run/20261026T175052_leg_eval_seed1_r3_keyP_3600_L), 0.005 / 0.007 | [18 / 0 / 85](#/week/7/run/20261026T175442_leg_eval_seed1_r3_keyP_3600_Z), 0.014 / 0.024 |
| position 4000 | [14 / 0 / 94](#/week/7/run/20261026T175843_leg_eval_seed1_r3_keyP_4000_H), 0.014 / 0.016 | [52 / 40 / 16](#/week/7/run/20261026T180244_leg_eval_seed1_r3_keyP_4000_L), 0.008 / 0.010 | [11 / 7 / 59](#/week/7/run/20261026T180647_leg_eval_seed1_r3_keyP_4000_Z), 0.009 / 0.016 |
| position 4400 | [15 / 4 / 87](#/week/7/run/20261026T181044_leg_eval_seed1_r3_keyP_4400_H), 0.016 / 0.017 | [1 / 82 / 25](#/week/7/run/20261026T181451_leg_eval_seed1_r3_keyP_4400_L), 0.015 / 0.015 | [7 / 66 / 35](#/week/7/run/20261026T181911_leg_eval_seed1_r3_keyP_4400_Z), 0.015 / 0.021 |
| position 4612 | [5 / 5 / 98](#/week/7/run/20261026T182240_leg_eval_seed1_r3_keyP_4612_H), 0.018 / 0.014 | [15 / 61 / 32](#/week/7/run/20261026T182630_leg_eval_seed1_r3_keyP_4612_L), 0.013 / 0.011 | [2 / 65 / 36](#/week/7/run/20261026T183056_leg_eval_seed1_r3_keyP_4612_Z), 0.012 / 0.013 |

- **The force-aware result replicates.** By the rule (on L: no falls, the fewest past 60 N, the most unlocks), seed 3
  chooses `3600`: 107 / 0 / 1 on L, 101 / 0 / 5 on Z, 101 / 1 / 5 on H. Seed 2's `4000` scored 107 / 0 / 0, 107 / 0 / 0
  and 95 / 0 / 1. The checkpoints swing in both seeds.
- **Seed 3's position-only `3600` matches it on L: 107 / 0 / 0.** Seed 2's best there was 57 / 0 / 33. Off L it fails:
  18 / 0 / 85 on Z, 55 / 0 / 52 on H.
  - Its later checkpoints collapse: `4400` falls in 82 of 108 on L.
- **So F-229's "force-aware lead on the dog's corner" holds across leg models, not at one leg model's peak.** At their
  chosen checkpoints the force-aware policies unlock 95-107 on all three; position-only 107 or 57 on L, and 1-18 and
  37-55 on Z and H.
  - With the dog's legs unmeasured, that robustness is the advantage. On light legs alone a position-only policy can
    match it.

### 2026-10-27 · Reach round 2's delay arm, a second seed: calm again, tracking replicated on L but not on Z

Job `110` repeated job `080` (the delay drawn 5-20 ms) with seed 2: [run](#/week/7/run/20261026T163629_train_seed2_reach_r2delay_s2).
Job `e110` swept it on T, L, Z and 0 ms.
- **Calm at every delay again:** standing at 0 ms, 0.023-0.028 rad (`59200`-`59800`), with 0-1 of 39 episodes past 0.1.
- **By the rule, `59000`:** [2.10 / 2.54 cm on L](#/week/7/run/20261026T185151_eval_seed1_r2d2_reach_59000_L), the original's p90 (2.52), as seed 1's
  `59600` (2.57).
- **On Z it is [2.12 / 3.07](#/week/7/run/20261026T185356_eval_seed1_r2d2_reach_59000_Z), worse than the original's 2.01 / 2.84.** Seed 1's 1.92 / 2.60
  there does not replicate.
- So two seeds agree that the delay range removes the zero-delay rattle and keeps the tracking on light legs. Whether
  it helps on zero-armature legs is one seed against one.

### 2026-10-27 · ft11a's whole lineage again: its position-only twin, and a second force-aware seed

Job `120` (8 h, 2048 environments each, in parallel) ran the five stages of ft11a's lineage (ft6, ft7, ft9a, ft10b,
ft11a). Each stage used its recorded recipe and started from the stage before's last checkpoint.
- **A, position-only from round 4's position `5999`, seed 3 (ft11a's), the matched twin ft11a never had:** [ft6](#/week/7/run/20261026T182213_train_seed3_pchain_position_s3_ft6), [ft7](#/week/7/run/20261026T200403_train_seed3_pchain_position_s3_ft7), [ft9a](#/week/7/run/20261026T225435_train_seed3_pchain_position_s3_ft9a), [ft10b](#/week/7/run/20261027T000302_train_seed3_pchain_position_s3_ft10b), [ft11a](#/week/7/run/20261027T011103_train_seed3_pchain_position_s3_ft11a).
- **B, force-aware from round 4's `5999`, seed 4, ft11a's lineage replicated:** [ft6](#/week/7/run/20261026T182343_train_seed4_pchain_force_s4_ft6), [ft7](#/week/7/run/20261026T200639_train_seed4_pchain_force_s4_ft7), [ft9a](#/week/7/run/20261026T225815_train_seed4_pchain_force_s4_ft9a), [ft10b](#/week/7/run/20261027T000654_train_seed4_pchain_force_s4_ft10b), [ft11a](#/week/7/run/20261027T011510_train_seed4_pchain_force_s4_ft11a).

Job `e120` ran the dog battery on the last stage's checkpoints. The cells are as for the one-stage pair:

| checkpoint | the hands, H | the hands, L | the hands, Z | 10 s stand on L, FL / RR thigh | frozen v2 set, classic stand, H |
| --- | --- | --- | --- | --- | --- |
| force-aware, seed 4, 10800 | [105 / 0 / 38](#/week/7/run/20261027T030812_leg_eval_seed1_c_pressf_hands_10800_H) | [113 / 0 / 4](#/week/7/run/20261027T031041_leg_eval_seed1_c_pressf_hands_10800_L) | [84 / 0 / 42](#/week/7/run/20261027T031250_leg_eval_seed1_c_pressf_hands_10800_Z) | [0.008 / 0.005](#/week/7/run/20261027T033553_leg_eval_seed1_c_pressf_stand10_10800_L) | [168 / 0 / 11](#/week/7/run/20261027T034410_leg_eval_seed1_c_pressf_real_10800_H) |
| force-aware, seed 4, 11000 | [242 / 0 / 7](#/week/7/run/20261027T031519_leg_eval_seed1_c_pressf_hands_11000_H) | [152 / 0 / 26](#/week/7/run/20261027T031730_leg_eval_seed1_c_pressf_hands_11000_L) | [68 / 0 / 65](#/week/7/run/20261027T031941_leg_eval_seed1_c_pressf_hands_11000_Z) | [0.006 / 0.004](#/week/7/run/20261027T033758_leg_eval_seed1_c_pressf_stand10_11000_L) | [174 / 0 / 11](#/week/7/run/20261027T034614_leg_eval_seed1_c_pressf_real_11000_H) |
| force-aware, seed 4, 11200 | [254 / 0 / 45](#/week/7/run/20261027T032156_leg_eval_seed1_c_pressf_hands_11200_H) | [177 / 0 / 0](#/week/7/run/20261027T032422_leg_eval_seed1_c_pressf_hands_11200_L) | [96 / 0 / 62](#/week/7/run/20261027T032631_leg_eval_seed1_c_pressf_hands_11200_Z) | [0.007 / 0.003](#/week/7/run/20261027T033959_leg_eval_seed1_c_pressf_stand10_11200_L) | [122 / 0 / 5](#/week/7/run/20261027T034817_leg_eval_seed1_c_pressf_real_11200_H) |
| force-aware, seed 4, 11394 | [274 / 0 / 12](#/week/7/run/20261027T032915_leg_eval_seed1_c_pressf_hands_11394_H) | [142 / 0 / 0](#/week/7/run/20261027T033128_leg_eval_seed1_c_pressf_hands_11394_L) | [116 / 0 / 99](#/week/7/run/20261027T033336_leg_eval_seed1_c_pressf_hands_11394_Z) | [0.007 / 0.003](#/week/7/run/20261027T034206_leg_eval_seed1_c_pressf_stand10_11394_L) | [145 / 0 / 8](#/week/7/run/20261027T035020_leg_eval_seed1_c_pressf_real_11394_H) |
| position-only, seed 3, 10800 | [0 / 0 / 46](#/week/7/run/20261027T022118_leg_eval_seed1_c_pressp_hands_10800_H) | [32 / 0 / 72](#/week/7/run/20261027T022323_leg_eval_seed1_c_pressp_hands_10800_L) | [2 / 0 / 175](#/week/7/run/20261027T022531_leg_eval_seed1_c_pressp_hands_10800_Z) | [0.003 / 0.002](#/week/7/run/20261027T025037_leg_eval_seed1_c_pressp_stand10_10800_L) | [84 / 0 / 59](#/week/7/run/20261027T025917_leg_eval_seed1_c_pressp_real_10800_H) |
| position-only, seed 3, 11000 | [32 / 0 / 55](#/week/7/run/20261027T022817_leg_eval_seed1_c_pressp_hands_11000_H) | [22 / 0 / 63](#/week/7/run/20261027T023048_leg_eval_seed1_c_pressp_hands_11000_L) | [16 / 0 / 158](#/week/7/run/20261027T023255_leg_eval_seed1_c_pressp_hands_11000_Z) | [0.003 / 0.002](#/week/7/run/20261027T025249_leg_eval_seed1_c_pressp_stand10_11000_L) | [85 / 0 / 36](#/week/7/run/20261027T030133_leg_eval_seed1_c_pressp_real_11000_H) |
| position-only, seed 3, 11200 | [0 / 0 / 52](#/week/7/run/20261027T023554_leg_eval_seed1_c_pressp_hands_11200_H) | [25 / 0 / 70](#/week/7/run/20261027T023813_leg_eval_seed1_c_pressp_hands_11200_L) | [5 / 0 / 181](#/week/7/run/20261027T024020_leg_eval_seed1_c_pressp_hands_11200_Z) | [0.004 / 0.003](#/week/7/run/20261027T025459_leg_eval_seed1_c_pressp_stand10_11200_L) | [107 / 0 / 39](#/week/7/run/20261027T030346_leg_eval_seed1_c_pressp_real_11200_H) |
| position-only, seed 3, 11394 | [1 / 0 / 42](#/week/7/run/20261027T024307_leg_eval_seed1_c_pressp_hands_11394_H) | [31 / 0 / 71](#/week/7/run/20261027T024529_leg_eval_seed1_c_pressp_hands_11394_L) | [2 / 0 / 195](#/week/7/run/20261027T024738_leg_eval_seed1_c_pressp_hands_11394_Z) | [0.004 / 0.003](#/week/7/run/20261027T025708_leg_eval_seed1_c_pressp_stand10_11394_L) | [93 / 0 / 50](#/week/7/run/20261027T030600_leg_eval_seed1_c_pressp_real_11394_H) |

ft11a's final on the same battery (week 6): the hands 211 / 0 / 47 (H), 149 / 0 / 0 (L), 108 / 0 / 26 (Z); its frozen
set 121 / 1 / 17.

- **The lineage replicates, and its new seed presses more:** by F-227's rule, seed 4 chooses `11200`.
  - The hands: 177 / 0 / 0 on L, 254 / 0 / 45 on H, 96 / 0 / 62 on Z. Its frozen set: 122 / 0 / 5.
  - It never falls, and its 10 s stand is calm (0.007 / 0.003 rad).
  - It beats ft11a on L (177 against 149) and on H, and strikes past 60 N more often on Z (62 against 26).
- **The position-only twin presses the dog's hands 22-32 of 288 on light legs,** 0-32 on heavy ones and 2-16 on zero
  armature. Its frozen set is 84-107.
  - On the legs the dog likely has, the force-aware lineage presses 4-6 times as many (149 and 177 against ≤ 32).
  - The one-stage pair (F-234) understated it: 67 against 56.
  - The twin is calm too (0.003-0.006 rad).
- Two more seeds (position-only 4, force-aware 5) are training (job `140`).

### 2026-10-27 · Lever round 4 (round 3 with the resistance held at 40 N): it falls less, and loses the test set

[Run](#/week/7/run/20261027T022115_train_seed1_lever_r4cap_s1) (job `130`, 1.25 h): round 3's recipe with the mechanisms' peak resistance held at 40 N in
training (`--peak_ceiling 40 --peak_ceiling_max 40`; the recipe trains at 80 throughout). Every retrain had fallen at
20-40 N. Job `e130` swept it.
- **Fewer falls on levers:** `22400` falls in [2 of 360 on L](#/week/7/run/20261027T041031_mech_eval_seed1_r4c_lever_levers_22400_L) (0-3 across the legs).
  Rounds 1-3 fell in 4-38.
- **It loses the test set:** [312 of 512 on L](#/week/7/run/20261027T043858_mech_eval_seed1_r4c_lever_test_22400_L) (267-402 across checkpoints and legs),
  against the original's 502. Doors, drawers and latches drop from ~128 to 60-100 each. That set's mechanisms resist
  past 40 N; the cap took away what they need.
- **Calm** (0.035-0.046 rad, d2/d1 1.3-1.4).
- **No lever retrain is better on every count, over four rounds.** Each one that stops rattling either falls on levers
  or loses the test set. `21496` stays, and the rattle-or-falls trade of F-233 stands.

### 2026-10-27 · Insertion round 2 (delay 5-20 ms, curriculum from level 1.0): calm, and its last checkpoint catches up off light legs

[Run](#/week/7/run/20261027T033755_train_seed3_ins_r2_s3) (job `135`, 1 h): round 1's recipe with the delay drawn 5-20 ms and the
curriculum started at level 1.0. Job `e135` swept it. Each cell is lateral seated of 96 and damaged, detent seated and
damaged, then the lateral set's standing chatter:

| checkpoint | T | L | Z | 0 ms (light legs) |
| --- | --- | --- | --- | --- |
| 21400 (original) | [71; 3](#/week/7/run/20261025T135204_ins_eval_seed1_b_ins21400_lat_T) / [68; 2](#/week/7/run/20261025T135344_ins_eval_seed1_b_ins21400_det_T); 0.025 ×2.0 | [70; 2](#/week/7/run/20261025T140521_ins_eval_seed1_b_ins21400_lat_L) / [81; 0](#/week/7/run/20261025T140732_ins_eval_seed1_b_ins21400_det_L); 0.110 ×2.0 | [70; 4](#/week/7/run/20261025T141941_ins_eval_seed1_b_ins21400_lat_Z) / [73; 1](#/week/7/run/20261025T142153_ins_eval_seed1_b_ins21400_det_Z); 0.077 ×2.0 | - |
| 21600 | [68; 2](#/week/7/run/20261027T044903_ins_eval_seed1_r2_ins_lat_21600_T) / [48; 7](#/week/7/run/20261027T052226_ins_eval_seed1_r2_ins_det_21600_T); 0.004 ×1.6 | [56; 0](#/week/7/run/20261027T045045_ins_eval_seed1_r2_ins_lat_21600_L) / [40; 4](#/week/7/run/20261027T052410_ins_eval_seed1_r2_ins_det_21600_L); 0.008 ×1.3 | [65; 2](#/week/7/run/20261027T045259_ins_eval_seed1_r2_ins_lat_21600_Z) / [48; 6](#/week/7/run/20261027T052627_ins_eval_seed1_r2_ins_det_21600_Z); 0.005 ×1.6 | [74; 0](#/week/7/run/20261027T045513_ins_eval_seed1_r2_ins_lat_21600_D0) / [48; 6](#/week/7/run/20261027T052844_ins_eval_seed1_r2_ins_det_21600_D0); 0.004 ×1.5 |
| 21800 | [56; 0](#/week/7/run/20261027T045725_ins_eval_seed1_r2_ins_lat_21800_T) / [32; 4](#/week/7/run/20261027T053059_ins_eval_seed1_r2_ins_det_21800_T); 0.005 ×1.7 | [46; 0](#/week/7/run/20261027T045903_ins_eval_seed1_r2_ins_lat_21800_L) / [34; 4](#/week/7/run/20261027T053240_ins_eval_seed1_r2_ins_det_21800_L); 0.010 ×1.7 | [56; 0](#/week/7/run/20261027T050117_ins_eval_seed1_r2_ins_lat_21800_Z) / [33; 3](#/week/7/run/20261027T053454_ins_eval_seed1_r2_ins_det_21800_Z); 0.006 ×1.6 | [54; 0](#/week/7/run/20261027T050332_ins_eval_seed1_r2_ins_lat_21800_D0) / [33; 4](#/week/7/run/20261027T053710_ins_eval_seed1_r2_ins_det_21800_D0); 0.005 ×1.6 |
| 22000 | [43; 0](#/week/7/run/20261027T050545_ins_eval_seed1_r2_ins_lat_22000_T) / [46; 0](#/week/7/run/20261027T053926_ins_eval_seed1_r2_ins_det_22000_T); 0.015 ×1.9 | [38; 0](#/week/7/run/20261027T050724_ins_eval_seed1_r2_ins_lat_22000_L) / [33; 6](#/week/7/run/20261027T054105_ins_eval_seed1_r2_ins_det_22000_L); 0.009 ×1.4 | [40; 1](#/week/7/run/20261027T050939_ins_eval_seed1_r2_ins_lat_22000_Z) / [42; 2](#/week/7/run/20261027T054320_ins_eval_seed1_r2_ins_det_22000_Z); 0.010 ×1.6 | [42; 0](#/week/7/run/20261027T051153_ins_eval_seed1_r2_ins_lat_22000_D0) / [45; 0](#/week/7/run/20261027T054536_ins_eval_seed1_r2_ins_det_22000_D0); 0.015 ×1.9 |
| 22199 | [81; 0](#/week/7/run/20261027T051406_ins_eval_seed1_r2_ins_lat_22199_T) / [71; 1](#/week/7/run/20261027T054748_ins_eval_seed1_r2_ins_det_22199_T); 0.017 ×2.0 | [65; 6](#/week/7/run/20261027T051545_ins_eval_seed1_r2_ins_lat_22199_L) / [51; 5](#/week/7/run/20261027T054928_ins_eval_seed1_r2_ins_det_22199_L); 0.007 ×1.7 | [72; 0](#/week/7/run/20261027T051800_ins_eval_seed1_r2_ins_lat_22199_Z) / [61; 2](#/week/7/run/20261027T055143_ins_eval_seed1_r2_ins_det_22199_Z); 0.009 ×1.8 | [86; 0](#/week/7/run/20261027T052013_ins_eval_seed1_r2_ins_lat_22199_D0) / [68; 4](#/week/7/run/20261027T055357_ins_eval_seed1_r2_ins_det_22199_D0); 0.017 ×2.0 |

- **Starting at level 1.0 does not save the seats early on** (`21600`-`22000`: 33-48 detent on L). So round 1's loss
  was not the curriculum restart.
- **Its last checkpoint `22199` seats as many as the original off light legs:** 81 + 71 on T, 86 + 68 at 0 ms. On L it
  still seats fewer, 65 + 51 against 70 + 81, and on Z 72 + 61 against 70 + 73.
- **Calm on every leg model** (0.006-0.018 rad; the original 0.11-0.12 on L).
- It was still improving when it stopped. Job `150` continues it 800 iterations from `22199`.

### 2026-10-28 · The lineage pair's second seeds: both sides replicate

Job `140` (8 h) repeated job `120` with position-only seed 4 and force-aware seed 5:
[position, ft11a stage](#/week/7/run/20261027T113526_train_seed4_pchain_position_s4_ft11a),
[force, ft11a stage](#/week/7/run/20261027T113919_train_seed5_pchain_force_s5_ft11a). Job `e140` ran the battery:

| checkpoint | the hands, H | the hands, L | the hands, Z | 10 s stand on L | frozen v2 set, H |
| --- | --- | --- | --- | --- | --- |
| force-aware, seed 5, 10800 | [242 / 0 / 54](#/week/7/run/20261027T133012_leg_eval_seed1_c2_pressf_hands_10800_H) | [167 / 0 / 3](#/week/7/run/20261027T133228_leg_eval_seed1_c2_pressf_hands_10800_L) | [166 / 0 / 29](#/week/7/run/20261027T133430_leg_eval_seed1_c2_pressf_hands_10800_Z) | [0.003 / 0.002](#/week/7/run/20261027T135545_leg_eval_seed1_c2_pressf_stand10_10800_L) | [169 / 0 / 18](#/week/7/run/20261027T140554_leg_eval_seed1_c2_pressf_real_10800_H) |
| force-aware, seed 5, 11000 | [157 / 0 / 18](#/week/7/run/20261027T133635_leg_eval_seed1_c2_pressf_hands_11000_H) | [136 / 0 / 0](#/week/7/run/20261027T133850_leg_eval_seed1_c2_pressf_hands_11000_L) | [136 / 0 / 1](#/week/7/run/20261027T134052_leg_eval_seed1_c2_pressf_hands_11000_Z) | [0.002 / 0.002](#/week/7/run/20261027T135815_leg_eval_seed1_c2_pressf_stand10_11000_L) | [194 / 6 / 18](#/week/7/run/20261027T140829_leg_eval_seed1_c2_pressf_real_11000_H) |
| force-aware, seed 5, 11200 | [236 / 0 / 58](#/week/7/run/20261027T134255_leg_eval_seed1_c2_pressf_hands_11200_H) | [168 / 0 / 0](#/week/7/run/20261027T134515_leg_eval_seed1_c2_pressf_hands_11200_L) | [110 / 0 / 49](#/week/7/run/20261027T134720_leg_eval_seed1_c2_pressf_hands_11200_Z) | [0.003 / 0.002](#/week/7/run/20261027T140046_leg_eval_seed1_c2_pressf_stand10_11200_L) | [140 / 4 / 17](#/week/7/run/20261027T141104_leg_eval_seed1_c2_pressf_real_11200_H) |
| force-aware, seed 5, 11394 | [264 / 0 / 40](#/week/7/run/20261027T134948_leg_eval_seed1_c2_pressf_hands_11394_H) | [168 / 0 / 0](#/week/7/run/20261027T135202_leg_eval_seed1_c2_pressf_hands_11394_L) | [142 / 0 / 77](#/week/7/run/20261027T135334_leg_eval_seed1_c2_pressf_hands_11394_Z) | [0.006 / 0.005](#/week/7/run/20261027T140320_leg_eval_seed1_c2_pressf_stand10_11394_L) | [136 / 0 / 21](#/week/7/run/20261027T141341_leg_eval_seed1_c2_pressf_real_11394_H) |
| position-only, seed 4, 10800 | [2 / 0 / 33](#/week/7/run/20261027T124537_leg_eval_seed1_c2_pressp_hands_10800_H) | [32 / 0 / 106](#/week/7/run/20261027T124741_leg_eval_seed1_c2_pressp_hands_10800_L) | [29 / 0 / 153](#/week/7/run/20261027T124949_leg_eval_seed1_c2_pressp_hands_10800_Z) | [0.004 / 0.003](#/week/7/run/20261027T131312_leg_eval_seed1_c2_pressp_stand10_10800_L) | [89 / 2 / 44](#/week/7/run/20261027T132136_leg_eval_seed1_c2_pressp_real_10800_H) |
| position-only, seed 4, 11000 | [6 / 0 / 27](#/week/7/run/20261027T125226_leg_eval_seed1_c2_pressp_hands_11000_H) | [33 / 0 / 63](#/week/7/run/20261027T125431_leg_eval_seed1_c2_pressp_hands_11000_L) | [14 / 0 / 140](#/week/7/run/20261027T125636_leg_eval_seed1_c2_pressp_hands_11000_Z) | [0.003 / 0.002](#/week/7/run/20261027T131518_leg_eval_seed1_c2_pressp_stand10_11000_L) | [103 / 4 / 47](#/week/7/run/20261027T132342_leg_eval_seed1_c2_pressp_real_11000_H) |
| position-only, seed 4, 11200 | [11 / 0 / 59](#/week/7/run/20261027T125919_leg_eval_seed1_c2_pressp_hands_11200_H) | [32 / 0 / 40](#/week/7/run/20261027T130127_leg_eval_seed1_c2_pressp_hands_11200_L) | [44 / 0 / 144](#/week/7/run/20261027T130333_leg_eval_seed1_c2_pressp_hands_11200_Z) | [0.004 / 0.003](#/week/7/run/20261027T131722_leg_eval_seed1_c2_pressp_stand10_11200_L) | [51 / 2 / 68](#/week/7/run/20261027T132550_leg_eval_seed1_c2_pressp_real_11200_H) |
| position-only, seed 4, 11394 | [0 / 0 / 11](#/week/7/run/20261027T130624_leg_eval_seed1_c2_pressp_hands_11394_H) | [20 / 0 / 104](#/week/7/run/20261027T130830_leg_eval_seed1_c2_pressp_hands_11394_L) | [51 / 0 / 145](#/week/7/run/20261027T131037_leg_eval_seed1_c2_pressp_hands_11394_Z) | [0.004 / 0.004](#/week/7/run/20261027T131928_leg_eval_seed1_c2_pressp_stand10_11394_L) | [86 / 4 / 67](#/week/7/run/20261027T132802_leg_eval_seed1_c2_pressp_real_11394_H) |

- **Force-aware, three seeds of the lineage on light legs, no falls:** ft11a (seed 3) 149, seed 4's `11200` 177, seed 5
  168 (`11200` and `11394` tie). Seed 5's `11394` presses 264 on H and 142 on Z; its frozen set is 136 / 0 / 21.
- **Position-only, two seeds:** 22-32 (seed 3) and 20-33 (seed 4) on light legs; 0-11 on H.
- So the lineage's lead over its twin on the dog's likely legs is 5-8 times, on every seed so far. A third
  position-only seed and a fourth force-aware one are training (job `160`).

### 2026-10-28 · Insertion round 2 continued: closer on the frozen sets, worse in the demo

[Run](#/week/7/run/20261027T124520_train_seed3_ins_r2cont_s3) (job `150`, 1 h): round 2 continued 800 iterations from its `22199`. Job
`e150` swept it; the cells are as for round 2:

| checkpoint | T | L | Z | 0 ms (light legs) |
| --- | --- | --- | --- | --- |
| 21400 (original) | [71; 3](#/week/7/run/20261025T135204_ins_eval_seed1_b_ins21400_lat_T) / [68; 2](#/week/7/run/20261025T135344_ins_eval_seed1_b_ins21400_det_T); 0.025 ×2.0 | [70; 2](#/week/7/run/20261025T140521_ins_eval_seed1_b_ins21400_lat_L) / [81; 0](#/week/7/run/20261025T140732_ins_eval_seed1_b_ins21400_det_L); 0.110 ×2.0 | [70; 4](#/week/7/run/20261025T141941_ins_eval_seed1_b_ins21400_lat_Z) / [73; 1](#/week/7/run/20261025T142153_ins_eval_seed1_b_ins21400_det_Z); 0.077 ×2.0 | - |
| 22400 | [66; 6](#/week/7/run/20261027T141616_ins_eval_seed1_r2c_ins_lat_22400_T) / [66; 2](#/week/7/run/20261027T145006_ins_eval_seed1_r2c_ins_det_22400_T); 0.028 ×2.0 | [60; 9](#/week/7/run/20261027T141758_ins_eval_seed1_r2c_ins_lat_22400_L) / [52; 10](#/week/7/run/20261027T145146_ins_eval_seed1_r2c_ins_det_22400_L); 0.007 ×1.7 | [68; 5](#/week/7/run/20261027T142017_ins_eval_seed1_r2c_ins_lat_22400_Z) / [64; 1](#/week/7/run/20261027T145403_ins_eval_seed1_r2c_ins_det_22400_Z); 0.016 ×2.0 | [66; 7](#/week/7/run/20261027T142234_ins_eval_seed1_r2c_ins_lat_22400_D0) / [66; 3](#/week/7/run/20261027T145619_ins_eval_seed1_r2c_ins_det_22400_D0); 0.027 ×2.0 |
| 22600 | [72; 1](#/week/7/run/20261027T142452_ins_eval_seed1_r2c_ins_lat_22600_T) / [74; 0](#/week/7/run/20261027T145834_ins_eval_seed1_r2c_ins_det_22600_T); 0.019 ×2.0 | [73; 3](#/week/7/run/20261027T142631_ins_eval_seed1_r2c_ins_lat_22600_L) / [64; 2](#/week/7/run/20261027T150015_ins_eval_seed1_r2c_ins_det_22600_L); 0.008 ×1.7 | [62; 10](#/week/7/run/20261027T142848_ins_eval_seed1_r2c_ins_lat_22600_Z) / [56; 6](#/week/7/run/20261027T150230_ins_eval_seed1_r2c_ins_det_22600_Z); 0.016 ×1.9 | [74; 1](#/week/7/run/20261027T143105_ins_eval_seed1_r2c_ins_lat_22600_D0) / [74; 1](#/week/7/run/20261027T150446_ins_eval_seed1_r2c_ins_det_22600_D0); 0.020 ×2.0 |
| 22800 | [66; 0](#/week/7/run/20261027T143319_ins_eval_seed1_r2c_ins_lat_22800_T) / [55; 0](#/week/7/run/20261027T150701_ins_eval_seed1_r2c_ins_det_22800_T); 0.024 ×2.0 | [62; 2](#/week/7/run/20261027T143457_ins_eval_seed1_r2c_ins_lat_22800_L) / [52; 1](#/week/7/run/20261027T150841_ins_eval_seed1_r2c_ins_det_22800_L); 0.009 ×1.7 | [74; 2](#/week/7/run/20261027T143712_ins_eval_seed1_r2c_ins_lat_22800_Z) / [59; 3](#/week/7/run/20261027T151054_ins_eval_seed1_r2c_ins_det_22800_Z); 0.015 ×2.0 | [67; 0](#/week/7/run/20261027T143928_ins_eval_seed1_r2c_ins_lat_22800_D0) / [59; 0](#/week/7/run/20261027T151311_ins_eval_seed1_r2c_ins_det_22800_D0); 0.025 ×2.0 |
| 22998 | [67; 1](#/week/7/run/20261027T144141_ins_eval_seed1_r2c_ins_lat_22998_T) / [60; 5](#/week/7/run/20261027T151524_ins_eval_seed1_r2c_ins_det_22998_T); 0.024 ×2.0 | [75; 0](#/week/7/run/20261027T144323_ins_eval_seed1_r2c_ins_lat_22998_L) / [49; 4](#/week/7/run/20261027T151707_ins_eval_seed1_r2c_ins_det_22998_L); 0.009 ×1.9 | [64; 1](#/week/7/run/20261027T144537_ins_eval_seed1_r2c_ins_lat_22998_Z) / [59; 3](#/week/7/run/20261027T151922_ins_eval_seed1_r2c_ins_det_22998_Z); 0.018 ×2.0 | [65; 2](#/week/7/run/20261027T144751_ins_eval_seed1_r2c_ins_lat_22998_D0) / [63; 5](#/week/7/run/20261027T152138_ins_eval_seed1_r2c_ins_det_22998_D0); 0.026 ×2.0 |

- **On the frozen sets it closes most of the gap:** `22600` seats 73 + 64 on L (5 damaged), against the original's
  70 + 81. Round 1's best was 74 + 45 and round 2's 65 + 51.
- **Calm on L** (0.008 rad). A small every-step flip returns on the other legs as it trains (×2.0 at 0.016-0.028 rad,
  no episode past 0.1). The original's flip on its own legs was 0.025.
- **In the PhysX demo on the payload robot it seats fewer:** [21 / 48 on L](#/week/7/run/20261027T152540_insertion_policy_seed2_pd_ins22600_L) and
  [9 / 48 on Z](#/week/7/run/20261027T153244_insertion_policy_seed2_pd_ins22600_Z). Round 1's `22000` seated 32 and 27; the original 41 on L.
- **So no insertion retrain beats the original where it counts,** and `21400` stays, rattling (F-233).

### 2026-10-28 · The lineage pair's third seeds, and the comparison over seeds

Job `160` (8 h) ran the lineage again with position-only seed 5 and force-aware seed 6:
[position, ft11a stage](#/week/7/run/20261027T204905_train_seed5_pchain_position_s5_ft11a),
[force, ft11a stage](#/week/7/run/20261027T205338_train_seed6_pchain_force_s6_ft11a). Job `e160` ran the battery (the stands were calm:
0.005-0.007 rad on the waiting legs):

| checkpoint | the hands, H | the hands, L | the hands, Z | frozen v2 set, H |
| --- | --- | --- | --- | --- |
| force-aware, seed 6, 10800 | [254 / 0 / 28](#/week/7/run/20261027T221622_leg_eval_seed1_c3_pressf_hands_10800_H) | [95 / 0 / 7](#/week/7/run/20261027T221716_leg_eval_seed1_c3_pressf_hands_10800_L) | [71 / 0 / 27](#/week/7/run/20261027T221806_leg_eval_seed1_c3_pressf_hands_10800_Z) | [80 / 15 / 32](#/week/7/run/20261027T222957_leg_eval_seed1_c3_pressf_real_10800_H) |
| force-aware, seed 6, 11000 | [192 / 0 / 40](#/week/7/run/20261027T221859_leg_eval_seed1_c3_pressf_hands_11000_H) | [147 / 0 / 0](#/week/7/run/20261027T221951_leg_eval_seed1_c3_pressf_hands_11000_L) | [101 / 0 / 24](#/week/7/run/20261027T222042_leg_eval_seed1_c3_pressf_hands_11000_Z) | [85 / 6 / 18](#/week/7/run/20261027T223040_leg_eval_seed1_c3_pressf_real_11000_H) |
| force-aware, seed 6, 11200 | [236 / 0 / 16](#/week/7/run/20261027T222135_leg_eval_seed1_c3_pressf_hands_11200_H) | [121 / 0 / 2](#/week/7/run/20261027T222226_leg_eval_seed1_c3_pressf_hands_11200_L) | [86 / 0 / 46](#/week/7/run/20261027T222318_leg_eval_seed1_c3_pressf_hands_11200_Z) | [86 / 6 / 25](#/week/7/run/20261027T223122_leg_eval_seed1_c3_pressf_real_11200_H) |
| force-aware, seed 6, 11394 | [145 / 0 / 55](#/week/7/run/20261027T222409_leg_eval_seed1_c3_pressf_hands_11394_H) | [157 / 0 / 0](#/week/7/run/20261027T222502_leg_eval_seed1_c3_pressf_hands_11394_L) | [125 / 0 / 50](#/week/7/run/20261027T222554_leg_eval_seed1_c3_pressf_hands_11394_Z) | [157 / 7 / 22](#/week/7/run/20261027T223204_leg_eval_seed1_c3_pressf_real_11394_H) |
| position-only, seed 5, 10800 | [0 / 0 / 38](#/week/7/run/20261027T215905_leg_eval_seed1_c3_pressp_hands_10800_H) | [29 / 0 / 120](#/week/7/run/20261027T215948_leg_eval_seed1_c3_pressp_hands_10800_L) | [2 / 0 / 240](#/week/7/run/20261027T220039_leg_eval_seed1_c3_pressp_hands_10800_Z) | [80 / 1 / 48](#/week/7/run/20261027T221259_leg_eval_seed1_c3_pressp_real_10800_H) |
| position-only, seed 5, 11000 | [0 / 3 / 77](#/week/7/run/20261027T220149_leg_eval_seed1_c3_pressp_hands_11000_H) | [2 / 0 / 101](#/week/7/run/20261027T220238_leg_eval_seed1_c3_pressp_hands_11000_L) | [7 / 1 / 220](#/week/7/run/20261027T220329_leg_eval_seed1_c3_pressp_hands_11000_Z) | [104 / 5 / 35](#/week/7/run/20261027T221351_leg_eval_seed1_c3_pressp_real_11000_H) |
| position-only, seed 5, 11200 | [0 / 7 / 87](#/week/7/run/20261027T220427_leg_eval_seed1_c3_pressp_hands_11200_H) | [31 / 0 / 45](#/week/7/run/20261027T220520_leg_eval_seed1_c3_pressp_hands_11200_L) | [8 / 3 / 190](#/week/7/run/20261027T220608_leg_eval_seed1_c3_pressp_hands_11200_Z) | [111 / 2 / 50](#/week/7/run/20261027T221441_leg_eval_seed1_c3_pressp_real_11200_H) |
| position-only, seed 5, 11394 | [6 / 3 / 67](#/week/7/run/20261027T220703_leg_eval_seed1_c3_pressp_hands_11394_H) | [33 / 26 / 0](#/week/7/run/20261027T220756_leg_eval_seed1_c3_pressp_hands_11394_L) | [26 / 43 / 161](#/week/7/run/20261027T220847_leg_eval_seed1_c3_pressp_hands_11394_Z) | [83 / 3 / 53](#/week/7/run/20261027T221533_leg_eval_seed1_c3_pressp_real_11394_H) |

**The dog's two hands on light legs, by F-227's rule (no falls, the fewest past 60 N, the most presses), per seed:**

| | seed 3 | seed 4 | seed 5 | seed 6 |
| --- | --- | --- | --- | --- |
| force-aware lineage | 149 / 0 / 0 (ft11a) | 177 / 0 / 0 (`11200`) | 168 / 0 / 0 | 157 / 0 / 0 (`11394`) |
| position-only twin | 32 / 0 / 72 | 33 / 0 / 63 | 31 / 0 / 45 (`11200`) | - |

- **Over four force-aware seeds and three position-only ones, the force-aware lineage presses the dog's hands on light
  legs 149-177 of 288, and its twin 31-33,** with no falls on either side's chosen checkpoint.
- Seed 6 falls 6-15 times on the frozen set from the classic stand (on heavy legs), where seeds 4 and 5 fell 0-6. The
  candidate stays seed 4's `11200`.

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

### Every policy that drives the legs, on the dog's leg corner (2026-10-26/27)

Lukas's request: retrain every trained policy, since the armature and delay spec was fixed after the press rattled.
All results are simulation. L is light legs (armature 0.0002 / 0.0007, 15-20 ms), F-196's match to the dog.

| policy | before, on L | the dog's corner, retrained | rattle on L (standing, rad) | what to use |
| --- | --- | --- | --- | --- |
| key (force-aware) | ft045 `2604`: 30 / 1 / 47 of 108 | seed 2 `4000`: 107 / 0 / 0 (seed 3 `3600`: 107 / 0 / 1) | support thighs 0.022 / 0.028 -> 0.004 / 0.007 | **`checkpoints/leg_key_force_corner_model_4000.npz`** (F-229, F-235) |
| press (force-aware) | round 4: 10 / 83 / 166 of 288 hands | ft11a lineage, seed 4 `11200`: 177 / 0 / 0 (four seeds 149-177; position-only twin 31-33) | waiting thighs 0.27 / 0.22 (round 4) -> 0.007 / 0.003 | **`checkpoints/leg_press_force_chain_s4_model_11200.npz`** (F-236); ft11a the fallback |
| reach | `58400`: 1.89 / 2.52 cm | round 2 delay arm `59600`: 1.93 / 2.57 (seed 2 `59000`: 2.10 / 2.54) | 0.049 -> 0.022 | **round 2's `59600`** (F-232); not yet in `checkpoints/` |
| rope | `14600`: 24.3 / 35.9 / 49.1 N, 9 tugged | worse: 52-87 tugged, 27-29 N at 20 | 0.012 (no change needed) | **`14600` stays** (F-231) |
| mechanism | `17499`: 470 / 512 | worse: 358-429 | 0.044, no flip (no change needed) | **`17499` stays** (F-231) |
| lever | `21496`: 346 / 360, 502 / 512, 0 falls | round 3 `22400`: 346, 507, but 5 falls on levers (rounds 1-4: 1-72) | 0.119 ×2.0 -> 0.034 | **`21496` stays, rattling**: Lukas's trade (F-233) |
| insertion | `21400`: 70 + 81 of 192 | round 1 `22000`: 74 + 45; round 2 continued `22600`: 73 + 64, but 21 / 48 in the demo (41) | 0.11-0.12 ×2.0 -> 0.006-0.009 | **`21400` stays, rattling**: Lukas's trade (F-233) |

- **The dog's corner breaks or rattles 3 of the 6 policies that drive the legs** (F-228): the key, lever and insertion.
- **For the arm policies a 10-20 ms delay alone trains in a zero-delay rattle** (F-230). A delay drawn from 5 ms, or
  `action_rate` -0.1, prevents it (F-232).
- **Force awareness, on the dog's corner:** the press lineage presses 4-6 times its position-only twin (F-236). The
  key's lead is robustness across leg models; on light legs alone a position-only seed can match it (F-235).
- **The code** that makes this possible for every task (the leg model, `leg_chatter.json`, the demos' `--payload`,
  `--small_physx` and leg flags) is in the 2026-10-26 entry and `unifp_train/README.md`.

## Findings this week

F-228 to F-236: the corner campaign (the table above).

## Issues and risks

- **The dog's legs and loop delay are still not measured.** Every corner result rests on F-196's match to one live run.
  The measurement that would settle it: LowState's `tau_est` and joint response to a known step (`sysid/go2_knee/`),
  and the command-to-motion delay.
- **Lever and insertion have no retrain that is better on every count** (F-233). Their originals rattle on the corner.
- **The claw combiner demos no longer fit the measured handle** (0 / 16 on either legs, F-228). They cannot score the
  mechanism and lever policies.
- **The reach lineage is weak in the cup demo on the payload robot** (3-10 of 16), retrained or not. Week 4's 10 / 16
  was without the payload.
- **The full unittest suite hangs in `demos/combiner_key/tests/test_holster.py`.** Run alone it times out at 300 s;
  beside two training runs it grew to ~11 GB and was OOM-killed (2026-10-26).
  - Run module by module (2026-10-28), 45 of 50 pass, including every module this campaign touched.
  - `test_holster` hangs, `demos/combiner_key/tests/test_key_robot.py` fails 20 and errs 1, and `tests/test_payload.py`
    fails 1 ("1149 mount vertices outside both boxes").
  - All three fail identically on a clean worktree of HEAD (`5945ac9`, the other PC's commit), so they come with that
    commit, not with this campaign's changes. Not investigated.
- **Uncommitted:** the code, the two exports and this record are in the working tree, not committed (2026-10-27).

## Next week

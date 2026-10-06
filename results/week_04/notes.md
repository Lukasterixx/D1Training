# Week 4 — 5–11 October 2026

**Status:** in progress
**Focus:** Establish physical free-space control and first instrumented contact; resolve force and transfer feasibility

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Obtain repeatable simulated pressing and box phases; run P1/P3 pilots and start matched seeds once the configuration is stable.
- **Hardware and camera:** Complete G5 evidence; perform instrumented low-force contact after entry checks; compare estimated/measured forces and identify dynamics/timing mismatches.
- **Evidence and deliverable:** Week 4 scope decision: protect the tagged-box sequence; defer extra tasks/large-workspace locomotion if needed. Label any sensor-assisted diagnostic method separately.
- **Gate focus:** G5; G6 begins; G2 before substantive contact comparisons.

## Checklist

- [ ] Physical free-space pose tracking meets frozen tolerances without falls or limit violations
- [ ] Hold/abort behaviour exercised and logged
- [ ] Simulated pressing and box engagement phases pass declared task cases
- [ ] First instrumented physical contact recorded after G2/G5 and simulation evidence
- [ ] Force-estimator error, contact loss and force peaks measured independently
- [ ] Measured dynamics/timing updates versioned consistently across comparisons
- [ ] Core-scope decision and any deferrals documented

## Log

<!-- Dated entries (### YYYY-MM-DD · topic). For each: the exact command or change, the outcome in numbers,
     and what it does and does not show. Record runs with ./dashboard.py record. -->

### 2026-10-06 · The scripted scenes re-placed with the arm

**Lukas:** "Yes, update the scripted scenes", rather than re-pin the four failing tests to the new behaviour.

**The rule.** Every scene was set against the arm at the legacy mount on Unitree's base. The measured arm's shoulder
stands **(−19.0, 0, +28.2) mm** from there: the mount moved −19 mm and +5.5 mm (F-125), and the base grew 22.7 mm
(F-126). This is `mount.ARM_SHIFT_FROM_LEGACY_M`. `tests/test_workspace.py` checks it against forward kinematics:
Link6 moves by exactly that from the Week 1 legacy run.

- Scene objects move by the shift. Ones standing on the real floor in PhysX move horizontally only.
- The arm's own pose grids are defined above the mount. They rise with the taller base (+22.7 mm, `JOINT1_HEIGHT_M −
  LEGACY_JOINT1_HEIGHT_M`), so they sit where they were relative to the shoulder.

| scene | change | why |
| --- | --- | --- |
| cup pick survey (`grasp.SURVEY_PAST_THE_HEAD`) | heights 0.335/0.285/0.235 → 0.358/0.308/0.258 m above the mount | At 0.335 m the sightline past the dog's head crossed the trunk proxy, because the mount moved 19 mm back and the head did not. The planner fell back to 25 cm ahead. Raised with the shoulder, the 20 cm pose sees past the head again (clear from 0.345 m). |
| combiner search stops (`press.SearchParams`) | camera 0.30 → 0.3227, target 0.24 → 0.2627 m above the mount | Same reason. Without it, `test_a_close_look_before_a_grip_turns_the_camera_to_spare_the_wrist` saved at most 0.41 rad, not > 0.5. |
| cup pick demo (`run_pick_demo.py`) | cups 19 mm back: default (0.401, 0.03), both random regions | On the floor, so horizontal only. |
| lying combiner demo (`run_combiner_demo.py`) | boxes 19 mm back | On the floor, so horizontal only. `sample_placement` is unchanged, because the tests use it too. |
| UniFP cup table (`props.CUP_SCENE_SHIFT_M`) | the full shift: top 0.6282 m, cup x 0.496 | The heights were chosen for where the arm's hand arrives level. Raised only, the far cups' grasp leaves UniFP's goal sphere (0.587 > 0.58 m). |
| UniFP box post (`props.BOX_SCENE_SHIFT_M`) | +28.2 mm (0.4082 m), distance kept | The goal sphere is fixed to the base, not the arm. Moved 19 mm in as well, the door's pull leaves it at the side stances. `side_stance` was fixed for the shift. |
| pick tests (`test_pick_demo.py`) | `CUP_XY`, `BASE_HEIGHT` by the full shift; the survey test expects 0.358 m and 0.528 m above the floor; the random-cup tests measure from the shifted region, plus a test that it is shifted | Pure geometry, so the scenes go exactly where they were relative to the arm. |
| combiner tests (`test_turn.py`) | `box_in_base` adds the full shift | Pure geometry. |

**Tests:** 648 run, **647 pass**. All four tests that failed since `de1442c` were looked at. Three pass: survey,
pivot (9.9 mm against 10), and wall grasp (0.00 mm off the axis, from 3.47). The fourth still fails.

**The one that still fails:** `test_finds_the_tag_plans_on_the_close_look_and_pushes_through_the_arc`, at 5.18 mm
against 5.0. This is not the scene.

- The close-look camera stands where it should relative to the tag in every variant: 250 mm out, 26–33 mm above,
  6–7.5° off its normal.
- The box pose error follows the tag's estimated rotation: 1.9°, 1.1°, 0.15° gave 12.1, 7.8, 0.8 mm. The box's origin
  is about 0.35 m from the tag, so a nearly frontal planar tag's rotation noise reaches it amplified.
- Over 24 placements around the test's, the same sequence gives median 2.9 mm, p90 6.4–6.7 mm, max 8–9 mm, 19 of 24
  under 5 mm. That is the same with the box at the old arm-relative height and on the floor.
- The test's single placement passed on the old arm and lands in the tail now; small changes flip it, and the limit
  is inside the estimator's noise. Left failing for Lukas: widen the limit to what the estimator delivers, or improve
  the close look.

**The PhysX demos on the re-placed scenes:**

| demo | measured robot, scenes as they were | re-placed | old robot |
| --- | --- | --- | --- |
| lying cup pick, 11 sweep episodes (`--sweep_mode stops --seed 42`) | — | [**11/11**](#/week/4/run/20261006T015838_424641Z_pick_seed42), lift 0.117–0.119 m | 11/11 (F-069) |
| lying combiner grip-and-pull, 0.3 N·m, seeds 42 / 2026 | — | [**8/9**](#/week/4/run/20261006T013906_978543Z_combiner_seed42), [**8/8**](#/week/4/run/20261006T014641_393589Z_combiner_seed2026) | 9/9, 8/8 (F-075) |
| standing combiner, mechanism `17499`, 16 N·m door | 15/16 | [**16/16**](#/week/4/run/20261006T015458_combiner_mech_law_seed1_mb_scene_door16_17499) | 16/16 |
| lever demo, `21496`, −30° | 16/16 | [**16/16**](#/week/4/run/20261006T015603_combiner_mech_law_seed1_mb_scene_lever_21496) | 9/16 (20497) |
| insertion, `21400`, seed 2 (scene unchanged) | 46/48 | [46/48](#/week/4/run/20261006T015711_insertion_policy_seed2_mb_scene_ins_21400_s2) | 43/48 (21200) |
| UniFP cup, `57499` | 2/16 | [**7/16**](#/week/4/run/20261006T015332_cup_unifp_wrist_seed1_mb_scene_cup_57499) (10 picked, 3 spilt) | 12/16 (56000) |
| UniFP box, `57499` | 1/16 | [0/16](#/week/4/run/20261006T015358_combiner_unifp_wrist_seed1_mb_scene_box_57499) | 11/16 (56000) |

- **The lying combiner's** one miss lost the lever (169 mm slip, latch not released). Several plans note "door opened
  to 44–48 deg, where the arm stops reaching". The taller arm reaches a floor box less far, which re-placing cannot give
  back.
- **The UniFP box is not a placement problem.**
  - Probes: [moved fully](#/week/4/run/20261006T013007_combiner_unifp_wrist_seed1_mb_scene_full_combiner_57499) 3/16;
    [raised only](#/week/4/run/20261006T013233_combiner_unifp_wrist_seed1_mb_scene_zonly_combiner_57499) 3/16;
    [the CLI version](#/week/4/run/20261006T011333_combiner_unifp_wrist_seed1_mb_box_wrist_57499_shifted_probe) 6/16;
    with `56000` [0/16](#/week/4/run/20261006T013133_combiner_unifp_wrist_seed1_mb_scene_full_combiner_56000).
  - Two placements within 4 mm of each other gave 3/16 and 0/16.
  - In every case the jaws arrive within about 2 cm, the latch releases in 0–8 of 16, and the door then does not move
    (median 0.5°). The scripted pull with the position-only policy does not open the door on the measured robot. It
    needs its own diagnosis, not a placement.
- **The UniFP cup** recovers to 7/16 with its table moved with the arm (2/16 before), against 12/16. Spilling remains
  the main loss. Cup probes: [moved, 57499](#/week/4/run/20261006T012941_cup_unifp_wrist_seed1_mb_scene_full_cup_57499)
  7/16, [moved, 56000](#/week/4/run/20261006T013107_cup_unifp_wrist_seed1_mb_scene_full_cup_56000) 5/16.

**Also.** The lying cup pick needs `ultralytics`, which was not installed in `env_isaaclab` on this machine. The
[first attempt](#/week/4/run/20261006T013900_761689Z_pick_seed42) crashed on it. It was installed as the README says
(`pip install --no-deps ultralytics==8.3.228 ultralytics-thop==2.0.18`); numpy stays 1.26.0. `run_pick_demo.sh`
exited 0 on that crash, so a queue cannot tell a crash from a finished run.

What it does not show: the scenes are placed relative to the model's arm, which is as right as F-125/F-126; one seed
per demo; nothing on hardware.

### 2026-10-06 · The measured base's holes across the rail: 58 mm, as Unitree's; the mesh follows

**Lukas:** "The screws are still 58mm apart on the real model." `de1442c`'s mesh had scaled them by 1.2, to 69.6 mm
(F-126's scope: not measured). That put them 5.8 mm outboard of the rail screws.

**Change** (`d1_arm/measured_base.py`, then `python3 d1_arm/measured_base.py`):

- **What changes.** The flange and its lip (Unitree's bottom 7 mm) keep Unitree's size across the rail at the
  mounting tabs. y is scaled 1.0 beyond |x| = 51.5 mm on Unitree's base, which holds the holes whole, and 1.20 inside
  48 mm, blended smoothly. x and z are untouched.
- **Checks.**
  - No triangle flips against the plain 1.20 scaling. A 1.5 mm blend flipped four on the tab's top face, and taking
    only the bottom 5 mm sheared the lip by 3.2 mm; the version kept has neither.
  - Where the lip meets the cylinder at the tabs, the two differ by up to 1.8 mm.
  - The holes come out at **(±64.8, ±29.0) mm**: over all four rail screws, 0.0 mm across and 0.7 mm along (129.6 mm
    against the CAD's 131).
  - The holes are drawn as 5.4 × 4.5 mm ovals, and four internal bolt holes in the blend as teardrops. Neither size
    was measured.
- **Tests.** `tests/test_dock.py` gains a test that finds the holes in the mesh and checks them over the screws
  across. 10/10 pass.
- **Figures.** `d1_dock_holes.py` fits ellipses now. The bottom panel shows all four holes on the screws:

![The D1's holes against the dock's rail screws](../week_03/figures/d1_dock_holes.png)

**G0** (`python run_position_only.py verify --headless --num_envs 8`):
[25/25](#/week/4/run/20261006T005827_176584Z_verify_seed42). The resting tip, its 2.32 mm drift, the box's 17.42 cm
and Link6's 0.74 µm are identical to the digit to [yesterday's](#/week/4/run/20261005T101328_795845Z_verify_seed42).
Only the flange's shape changed: no mass, no kinematics, nothing it touches. So the fine-tuned policies and their
scores (F-127) stand and were not retrained.

What it does not show: the flange's real outline, the holes' real size (M4 in Unitree's model, M6 in a reseller's
D1-T listing), or anything about the robot that the screws' positions do not already fix.

### 2026-10-05 · The measured D1 base merged into `mechanism-demo`; G0 passes; the demos' policies to be retrained on it

**Why.** Lukas asked (2026-10-01) for the demos to be retrained with the D1's real dimensions. They were measured on
2026-09-30 and committed to `origin/main` as `de1442c`, from another machine. That is why the Claude Code sessions on
this PC did not have them (Week 3, 2026-10-03). `de1442c` sits on `e93eef7`. This branch split from there before the UniFP
merge, F-117's dock mount and every demo since, so none of its policies or demos had the measured base.

**The merge** (`git merge --no-commit origin/main`; staged, not committed):

- **Sixteen files conflicted**, nearly all the same change made two ways.
  - `mount.ARM_MOUNT_POS_M` stays the single source and is now `de1442c`'s **(−0.019, 0, 0.0855) m**: on the rails'
    tops, midway between their screws.
  - `weld.D1_MOUNT_POS` is kept as an alias, so main's launchers still import it.
  - F-117's mount is kept as `mount.DOCK_CAD_ARM_MOUNT_POS_M` (0.0025, 0, 0.0855), to name what this branch's runs
    of 2026-09-26 to 10-05 were built on.
- **From `de1442c` unchanged:**
  - Joint1 at 76.0 mm on `base_link_measured.STL` (`d1_arm/measured_base.py`);
  - `task_space`'s re-measured stance;
  - the workspace and IK test pins;
  - `go2_d1.urdf`'s mount, Joint1 and base mesh.
- **`tests/test_dock.py`** now checks that the measured base's holes (±65 mm) sit over both rail screws fore-aft, within
  1.5 mm. It used to check that Unitree's (±54 mm) rear pair was on the rails.
- **Findings renumbered.** `de1442c`'s **F-078 and F-079 are F-125 and F-126 here.** This branch's F-078 to F-093 are
  the UniFP findings (`e9eefdd`). Each carries a *Numbered* line saying so. F-125 has a merge note: on this branch it
  replaced F-117's mount, not (0, 0, 0.08). F-117 is marked superseded for its fore-aft place; the rails' height stands.
- **Tests:** `python -m unittest discover -s . -p "test_*.py"` in the Isaac env, 645 run, **641 pass, 4 fail**. The four
  are the ones `de1442c` reported on main:
  - the pick survey;
  - the pivot;
  - the wide-cup wall grasp;
  - the combiner tag pose.

  Their scenes are fixed in the base frame and were tuned for the old arm. They wait on Lukas's call (Week 3 Issues).
  Every test this branch added passes against the measured base: UniFP, mechanism, insertion, press, wipe, dock,
  pose feasibility.

**G0** (`python run_position_only.py verify --headless --num_envs 8`):
[verify on the merged tree](#/week/4/run/20261005T101328_795845Z_verify_seed42), **25/25**, `arm_mount_m` (−0.019, 0,
0.0855). Link6 matches URDF kinematics to 0.74 µm and the tool point to 1.3 µm. The resting tip is (0.3383, −0.0141,
0.7929) m, 2.3 mm from `task_space`'s recorded tip, and the target box is 17.4 cm from it. These are `de1442c`'s numbers
on main, so this branch's extra code leaves the robot the same.

**Do the measured base's holes line up with the rail screws?** This was Lukas's question of 2026-10-01, re-asked of
the new model. Script: [`d1_dock_holes.py`](../week_03/figures/d1_dock_holes.py), now two panels.

- **Fore-aft: yes.** The holes are 129.6 mm apart and land 0.7 mm from each rail screw (131.0 mm apart).
- **Across: no, in the mesh.** `measured_base.py` scaled Unitree's 58 mm by 1.2, to 69.6 mm, which nobody measured
  (F-126's scope). That puts the holes 5.8 mm outboard of the screws, on the rails' outer edges, not over their
  T-slots. If the arm bolts straight onto the rail screws, as Lukas reports, the real spacing across is 58 mm.
- **Effect:** visual only. The holes take no part in the physics, and the flange outline changes the base's collision
  shape by millimetres, at the dock, where nothing collides. Not changed; a ruler across the real holes settles it.

![The D1's holes against the dock's rail screws, before and after the measured base](../week_03/figures/d1_dock_holes.png)

**What this changes for the demos.** Every checkpoint in `checkpoints/` was trained on Unitree's 53.3 mm base. The
reach (`56000`) and mechanism (`17499`, `20497`) policies were at (0, 0, 0.08); the insertion, press and wipe runs
were at F-117's mount or older. At the real robot's shoulder that is (−19.0, 0, +28.2) mm or (−21.5, 0, +22.7) mm
away. F-118 measured that the reaching policy misses by about the whole shift. Each demo's policy is to be fine-tuned
on the measured robot and re-scored on its frozen set before its demo is re-run.

## Results

### Retraining the demos' policies on the measured robot (in progress)

Queue (one GPU job at a time): each released checkpoint is scored first on the measured robot, then fine-tuned with
its own lineage's recipe, then re-scored on the same frozen sets. Only the robot differs.

- **Reach** (`56000`): `--task unifp --tool_point fingertip`, as trained; adaptive rate, UniFP's armature.
- **Mechanism and lever** (`17499`, `20497`): `--task mechanism --force_law` with their recorded flags; adaptive rate,
  0.01 kg·m² armature.
- **Insertion, press and wipe** (`21200`): pinned 1e-5 rate, `--resume_arm_std 0.15`. The press and wipe pilots re-run
  from `21200` exactly as on 2026-09-29.

**Before retraining: the released checkpoints on the measured robot.** One seed. The frozen sets are unchanged.

| policy | frozen set | on its trained robot | on the measured robot |
| --- | --- | --- | --- |
| reach `56000` | 50-episode manifest `94e576a6…`, fingertip, settled error median / p90 | 1.51 / 2.17 cm (F-118) | [**4.35 / 4.86 cm**](#/week/4/run/20261005T101716_eval_seed20260921_mb_base_reach56000), 0 falls |
| mechanism `17499` | held-out test, law + bleed, opened of 512 | 503 (button 119) | [**498** (button 114)](#/week/4/run/20261005T101740_mech_eval_seed1_mb_base_mech_test_17499), 0 falls |
| lever `20497` | levers ≤ 0.6 m, square / spindle / tip side, of 120 each | 98 / 89 / 117 | [**89 / 79 / 112**](#/week/4/run/20261005T101808_mech_eval_seed1_mb_base_lever_20497), 0 falls |
| insertion `21200` | 96 lateral slots, seated (damaged) | 75 (1) (F-122) | [**33 (0)**](#/week/4/run/20261005T101834_ins_eval_seed1_mb_base_ins_lateral_21200), tip 10.5 mm off at the layer's start |
| insertion `21200` | 96 detent slots, seated (damaged) | 71 | [**29 (4)**](#/week/4/run/20261005T101911_ins_eval_seed1_mb_base_ins_detent_21200) |

What it shows:
- **The reach policy misses by about the shoulder's move.** That move is 3.4 cm, and the policy cannot see it,
  as F-118 predicted from translations of the mount.
- **The mechanism test hardly notices**, because it closes the grasp wherever the hand stops (F-115). The demo is
  where a reach error would show.
- **Insertion loses more than half its slots**, and the detent set now damages four. A millimetre task does not absorb
  a 2–3 cm move of the shoulder.

What it does not show: anything about the fine-tuned policies (below), or the PhysX demos.

**After fine-tuning: every saved checkpoint on the same frozen sets** (2026-10-06). Fine-tune runs:
[reach](#/week/4/run/20261005T101948_train_seed1_reach_mb), [mechanism](#/week/4/run/20261005T112628_train_seed1_mech_law_mb),
[lever](#/week/4/run/20261005T121345_train_seed1_mech_lever_mb), [insertion](#/week/4/run/20261005T130117_train_seed3_ins_mb),
[press pilot](#/week/4/run/20261005T134706_train_seed1_press_pilot_mb), [wipe pilot](#/week/4/run/20261005T140645_train_seed1_wipe_pilot_mb).

| policy | checkpoint | frozen set result | vs. trained robot / unretrained on the measured one |
| --- | --- | --- | --- |
| reach | [56400](#/week/4/run/20261005T142648_eval_seed20260921_mb_reach_model_56400) · [56800](#/week/4/run/20261005T142713_eval_seed20260921_mb_reach_model_56800) · [57200](#/week/4/run/20261005T142737_eval_seed20260921_mb_reach_model_57200) · [**57499**](#/week/4/run/20261005T142802_eval_seed20260921_mb_reach_model_57499) | median / p90: 1.89 / 2.53 · 1.64 / 2.73 · 1.77 / 2.12 · **1.70 / 2.16 cm**; 0 falls in all | 1.51 / 2.17 · 4.35 / 4.86 |
| mechanism | [18000](#/week/4/run/20261005T142826_mech_eval_seed1_mb_mech_test_model_18000) · [18400](#/week/4/run/20261005T142853_mech_eval_seed1_mb_mech_test_model_18400) · [18498](#/week/4/run/20261005T142920_mech_eval_seed1_mb_mech_test_model_18498) | held-out test, of 512: 481 · 474 · 443 (button 103 · 90 · 67) | 503 · **498** |
| lever | [21000](#/week/4/run/20261005T142947_mech_eval_seed1_mb_lever_model_21000) · [21400](#/week/4/run/20261005T143040_mech_eval_seed1_mb_lever_model_21400) · [**21496**](#/week/4/run/20261005T143134_mech_eval_seed1_mb_lever_model_21496) | levers ≤ 0.6 m, of 360: 337 (9 falls) · 332 · **343** (120 / 103 / 120) | 304 · 280 |
| lever, held-out test | [21000](#/week/4/run/20261005T143013_mech_eval_seed1_mb_lever_test_model_21000) · [21400](#/week/4/run/20261005T143107_mech_eval_seed1_mb_lever_test_model_21400) · [**21496**](#/week/4/run/20261005T143200_mech_eval_seed1_mb_lever_test_model_21496) | of 512: 435 · 473 · **505** (button 123) | 20497: 88/128 button (F-114) |
| insertion | [**21400**](#/week/4/run/20261005T143227_ins_eval_seed1_mb_ins_lateral_model_21400) · [21600](#/week/4/run/20261005T143341_ins_eval_seed1_mb_ins_lateral_model_21600) · [21800](#/week/4/run/20261005T143453_ins_eval_seed1_mb_ins_lateral_model_21800) · [21999](#/week/4/run/20261005T143606_ins_eval_seed1_mb_ins_lateral_model_21999) | lateral seated: **73** · 73 · 48 · 33 (damaged 1 · 1 · 1 · 0) | 75 · 33 |
| insertion, detent | [**21400**](#/week/4/run/20261005T143304_ins_eval_seed1_mb_ins_detent_model_21400) · [21600](#/week/4/run/20261005T143417_ins_eval_seed1_mb_ins_detent_model_21600) · [21800](#/week/4/run/20261005T143530_ins_eval_seed1_mb_ins_detent_model_21800) · [21999](#/week/4/run/20261005T143643_ins_eval_seed1_mb_ins_detent_model_21999) | seated: **68** · 57 · 46 · 32 (damaged 4 · 4 · 3 · 2) | 71 · 29 |
| press pilot | model_21599: [panel](#/week/4/run/20261005T143719_press_eval_seed1_mb_press_panel_pilot) · [force](#/week/4/run/20261005T143745_press_eval_seed1_mb_press_force_pilot) · [recessed](#/week/4/run/20261005T143810_press_eval_seed1_mb_press_recessed_pilot) | 63/96 · 19/48 · 31/72 (3 damaged in recessed) | pilot_c on the old robot: 49 · 20 · 23 |
| wipe pilot | [model_21599](#/week/4/run/20261005T143836_wipe_eval_seed1_mb_wipe_pilot) | 23/36, completion 0.94, contact 1.0, pad tilt 9.3° | pilot_c: 16/36 |

What it shows:

- **The reach policy relearns the arm in 400 iterations.** It goes from 4.35 to 1.6–1.9 cm, and the final checkpoint's
  p90 (2.16 cm) is its trained robot's (2.17). Chosen: `57499`. It is the steadiest on p90, and its median is within
  0.06 cm of the best.
- **Continuing the mechanism policy made it worse, not better.** The held-out test fell from 498 to 443, mostly the
  button. These runs cannot separate the measured robot from simply training on: there is no control continued on the
  old robot. The mechanism lineage resumes at rsl-rl's adaptive rate; the insertion lineage pins 1e-5 for exactly
  this kind of drift (F-121). As retrained, none of these checkpoints replaces `17499` on this test.
- **The lever policy improved on every set.** `21496` opens 343 of 360 levers against 304 on its trained robot. On the
  held-out mechanism test it opens 505 of 512 with the button back (123/128), against 20497's 88 and 17499's 119.
  That is the mechanism lineage's best checkpoint on both tests, but two of its three saved checkpoints were worse,
  and 21000 fell 9 times on levers. Again there is no old-robot control.
- **Insertion recovers early and then declines, as on the old robot** (F-121, F-122). `21400`, 200 iterations in,
  seats 73/96 lateral and 68/96 detent: two and three slots short of the trained robot. By 21999 it is back at the
  unretrained level. Chosen: `21400`, on the lateral set, as F-122 chose.
- **Both pilots score higher than their old-robot runs**: press panel 63 against 49, wipe 23 against 16.
  - The press gain is at 0.70 m (13/32 against 3) and 0.78 m (30 against 21); at 0.86 m it is 20 against 25.
  - The probe gives most of it: 43/48 against 32.
  - It is one seed each on two robots, so the robot and chance cannot be separated. It is not evidence that the
    taller shoulder helps pressing.

What it does not show: the PhysX demos (below); a second seed of anything; a control continued on the old robot, which
the mechanism and lever results need before "worse" or "better" can be put down to the robot.

**The PhysX demos on the measured robot** (2026-10-06): each original checkpoint against its fine-tune, with the
command its headline came from (`checkpoints/README.md`, F-096, F-110, F-114, F-122).

| demo | original checkpoint | fine-tuned | original on its own robot |
| --- | --- | --- | --- |
| insertion, compliant key, 2 mm, seeds 2 / 3 / 4 | `21200`: [39/48](#/week/4/run/20261005T144931_insertion_policy_seed2_mb_unifp_go2d1_insertion_seed3_model_21200_lat2_s2), [40/48](#/week/4/run/20261005T145044_insertion_policy_seed3_mb_unifp_go2d1_insertion_seed3_model_21200_lat2_s3), [78/96](#/week/4/run/20261005T145157_insertion_policy_seed4_mb_unifp_go2d1_insertion_seed3_model_21200_lat2_n96_s4) | `21400`: [**46/48**](#/week/4/run/20261005T145416_insertion_policy_seed2_mb_model_21400_lat2_s2), [**45/48**](#/week/4/run/20261005T145529_insertion_policy_seed3_mb_model_21400_lat2_s3), [**85/96**](#/week/4/run/20261005T145643_insertion_policy_seed4_mb_model_21400_lat2_n96_s4); no damage, no falls | 43, 43, 80 |
| lever (−30°, 1.2 N·m lever, 16 N·m door, `--correct_entry`) | `20497`: [10/16](#/week/4/run/20261005T144719_combiner_mech_law_seed1_mb_lever_stm30_h12_d16_unifp_go2d1_mech_lever_model_20497) | `21496`: [**16/16**](#/week/4/run/20261005T144823_combiner_mech_law_seed1_mb_lever_stm30_h12_d16_model_21496) | 9/16 |
| combiner, mechanism controller, 16 N·m door | `17499`: [**15/16**](#/week/4/run/20261005T144407_combiner_mech_law_seed1_mb_door16_unifp_go2d1_mech_law_model_17499) | `18000`: [12/16](#/week/4/run/20261005T144511_combiner_mech_law_seed1_mb_door16_model_18000); lever `21496`: [0/16](#/week/4/run/20261005T144616_combiner_mech_law_seed1_mb_door16_model_21496) (no `--correct_entry`, F-114) | 16/16 |
| UniFP cup pick, wrist roll | `56000`: [5/16](#/week/4/run/20261005T144117_cup_unifp_wrist_seed1_mb_cup_wrist_unifp_go2d1_isaaclab_model_56000) | `57499`: [2/16](#/week/4/run/20261005T144241_cup_unifp_wrist_seed1_mb_cup_wrist_model_57499) (7 picked, 5 spilt) | 12/16 |
| UniFP box, wrist roll | `56000`: [0/16](#/week/4/run/20261005T144143_combiner_unifp_wrist_seed1_mb_box_wrist_unifp_go2d1_isaaclab_model_56000) | `57499`: [1/16](#/week/4/run/20261005T144307_combiner_unifp_wrist_seed1_mb_box_wrist_model_57499) | 11/16 |

What it shows:

- **Insertion and the lever demo are better on the measured robot than they were on the old one.** No falls or damage
  anywhere.
- **The combiner keeps its unretrained controller.** `17499` loses one attempt (15/16). Its fine-tune loses four.
- **The UniFP cup and box demos fail with either policy, and the policy is not the reason.**
  - In the box, the fine-tune arrives closer than `56000` did on the old robot: 1.7 cm against 2.7 cm. The latch
    releases in 7 of 16, but the door then does not move: median 0.5°, against 45°.
  - In the cup, 7 of 16 are picked and 5 spilt; none spilt before.
  - Both scripts' scenes sit at fixed spots in the base frame, tuned for the old arm. That is the same issue as the
    four scripted-demo tests, and it is read from the summaries, not traced.

**Delivered.** Three checkpoints are copied into `checkpoints/` with SHA256 sums and a README section: reach `57499`,
lever `21496`, insertion `21400`. `demos/unifp/run_demo.py`'s reach default is now `57499`. `MECH_CHECKPOINT` stays
`17499`. Finding: F-127.

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-128](../findings.md): re-placed with the arm, the scripted demos come back to their old-robot results (lying
  cup pick 11/11, lying combiner 8/9 and 8/8, standing combiner 16/16); the tag test's limit is inside the estimator's
  noise; the UniFP box fails for a reason that is not its placement.

- [F-127](../findings.md): on the measured robot the released policies miss by the shoulder's move; fine-tuned there,
  reach is back to 1.70 cm, insertion (46/48, 45/48, 85/96) and the lever demo (16/16) beat their old-robot scores, the
  mechanism fine-tune got worse (keep `17499`, 15/16), and the UniFP cup and box demos fail with either policy.

## Issues and risks

- ~~Four scripted-demo tests fail on the measured robot~~ Scenes re-placed with the arm (2026-10-06, Lukas's call):
  three pass. The combiner tag-pose test still fails at 5.18 mm against 5.0, inside the tag estimator's noise (p90
  about 6.5 mm over placements). Widen the limit or improve the close look: Lukas's call.
- **The UniFP box demo (position-only policy, scripted pull) does not open the door on the measured robot** (0–6/16
  across placements, against 11/16). The latch releases and the door does not move. This is not the placement and is
  not yet diagnosed. The UniFP cup is at 7/16 against 12, losing mostly to spills.
- `run_pick_demo.sh` exits 0 when the demo crashes.
- ~~The measured base's holes across the rail are unmeasured~~ Resolved 2026-10-06: 58 mm (Lukas). The mesh has it.

## Next week

- Diagnose the UniFP box demo's door pull on the measured robot: the latch releases and the door does not move.
- The combiner tag test: widen its limit to the estimator's noise, or improve the close look (Lukas's call).
- A mechanism fine-tune at the pinned rate (F-121) before concluding `17499` cannot be adapted; and an old-robot
  control for one fine-tune, to separate the robot from the extra training.

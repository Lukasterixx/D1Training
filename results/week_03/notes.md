# Week 3 — 28 September – 4 October 2026

**Status:** in progress
**Focus:** Adapt force estimation; calibrate contact; validate measured observations and begin gated free-space transfer

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Adapt UniFP history/estimator supervision and force curriculum; finish force-sign/fixture calibration; evaluate P0 pose trajectories and required base motion.
- **Hardware and camera:** Exercise camera-to-IK reference and measured-state kinematics; test both-bus inference parity and stale-vision behaviour. Begin bounded learned free-space trials only after G4 and candidate G1 checks.
- **Evidence and deliverable:** Freeze calibrated contact metrics before contact comparisons; record estimator supervision, hardware timing and every physical trial.
- **Gate focus:** G1a; G1b for moving-base scope; G2; G4; G5 entry.

## Checklist

- [ ] Common history-based architecture and force-free P0/P2 ablation routes documented
- [ ] UniFP-derived force estimator and force curriculum pilot recorded
- [ ] Known-load contact calibration, force sign and sensor coverage checked (G2)
- [ ] P0 slow pose trajectories evaluated, including tool orientation requirements
- [ ] Camera-derived targets exercised through the IK engineering reference
- [ ] Measured-state FK, both-bus action mapping and observation normalisation validated (G4)
- [ ] Stale-vision hold/reacquisition and base-motion frame tests recorded
- [ ] Gated learned free-space robot trials recorded or entry blockers documented

## Log

<!-- Dated entries (### YYYY-MM-DD · topic). For each: the exact command or change, the outcome in numbers,
     and what it does and does not show. Record runs with ./dashboard.py record. -->

### 2026-09-29 — The thesis contact tasks: pressing and wiping built in simulation, the key task written into the plan

**The request.** Lukas: only the combiner box existed as a thesis task for the arm. He asked for the wiping and
pressing tasks to be added, and for the plan to say that the key task is included too. The key insertion was already
built and trained (`--task insertion`, F-116, F-119–F-122) but the plan never named it.

**The plan** ([thesis_b_plan.md](../../docs/thesis_b_plan.md)). Key insertion is now a core experiment, "the second
way into a box". It has its own row in the contact-task table, with a success requirement written for it (Thesis A
had none): seated through the keyed channel, no damage, no fall; time to seat, peak and lateral force, search use,
failure reason. It is declared tool-only, as the lever is. The table gains a column saying where each task is built
in simulation and how far it has got. The physical demonstration stays the lever box; changing that is Lukas's call.

**What was built.** Both tasks follow the key insertion's recipe. An analytic plant stepped at 200 Hz, and a task layer
that drives the contact from an *estimate* of the fixture through the policy's force channel. The whole-body policy
trains with that layer in the loop, and the true fixture is off the estimate by errors it never observes.

- **Pressing** (`--task press`; `unifp_train/pressing.py`, `press_cfg.py`, `press_env.py`, `press_rewards.py`,
  `press_eval.py`). The plan's "spring-loaded button with travel and spring/damping; bare end effector, then rounded
  probe".
  - The button: a cap on preload, spring, damping and a tactile click, stopping at its travel. It sits in a hole in a
    panel, and the rim of the hole keeps a wide tip out of a small recessed cap.
  - The tools: the closed jaws' end faces, modelled as a 10 mm sphere 20 mm past the jaw centre; and a 6 mm ball on a
    rod 70 mm out.
  - The layer: the tip to a standoff; a PI force law along the estimated axis, capped at 3 N until the tip stalls on
    something, then at 1.6–2.4x the datasheet's force; a steady hold at full travel for 1.5 s; a release.
  - Success is the plan's "full travel held 1 s" (the cap within 0.5 mm of its stop, no break). Every episode also
    reports the force error (applied against commanded, RMS while pushing), the physics-rate peak, the time to full
    travel, and one failure reason.
  - Three frozen sets (2026-09-29, before any policy was scored): `panel` (96: three heights, four estimate errors,
    four directions, both tools), `force` (48: 5–30 N buttons), `recessed` (72: a small cap 2 mm below the panel).
    The bare jaws were declared in advance to fail `recessed`: it is the plan's access limitation, measured.
- **Wiping** (`--task wipe`; `wiping.py`, `wipe_cfg.py`, `wipe_env.py`, `wipe_rewards.py`, `wipe_eval.py`). The plan's
  "planar fixture with known normal; slow prescribed path; rigid pad".
  - The pad: a rigid 50 mm disc 45 mm past the jaw centre, meeting the panel at nine points, so a tilted pad presses
    with its rim.
  - The paths: a straight stroke or a zigzag, resampled evenly by arc length.
  - The layer: a PI force along the estimated normal capped at the wiping force (the true panel may be nearer or
    further); once the pad has stalled on the panel, a reference point along the path at most 2 cm ahead of the pad;
    a PI force across the normal to push through friction; a lift at the end.
  - Why the force across the normal: UniFP's controller yields to what it measures, so 4 N of friction would park the
    pad 2 cm behind its goal.
  - Success is the plan's: at least 90% of the path wiped and at least 80% of the motion in contact. A sample is wiped
    when the centre of pressure passes within 2 cm of it at ≥ 1 N; a contact loss counts from 0.1 s.
  - One frozen set of 36: three heights, stroke or zigzag, the panel 8 mm nearer, on the estimate or 8 mm further,
    5 N or 10 N, tilted 3° from the estimate.
- **Launcher.** `run_unifp_train.py --task press|wipe`, modes `press_eval` and `wipe_eval`, flags `--press_set`,
  `--press_tools`, `--press_level`, `--button_fraction`, `--wipe_level`, `--wipe_fraction`. Same 76/153-wide
  observations and 18 actions as every other task, so any checkpoint resumes into them.
- **Tests** (`tests/test_pressing.py` 22, `tests/test_wiping.py` 18; they need no simulator and pass). They pin the
  preload, the spring's travel, the click, the stop, the panel beside the cap, the rim, friction, damage, the path's
  resampling, and each layer closing its loop on a stand-in hand.
  - Pressing: the stand-in presses and holds a button estimated right, 3 mm too near or too far; it never presses one
    stiffer than the cap and never touches the cap from 15 mm off.
  - Wiping: the stand-in tracks UniFP's target, `goal + (contact + command)/200`. It covers 97–100% of a 16 cm stroke
    at 10 N with ≤ 0.63 N RMS error on panels up to 8 mm off and 4° tilted from the estimate.
  - One real defect found by the tests: the wipe layer took a slow hand in free air for a touch and started the path
    before contact. It now needs the push at its cap too.

**Smoke** (zero actions, `./run_unifp_train.py smoke --task press --num_envs 64 --steps 600 --headless`, and `--task wipe
... --steps 900`). [Press smoke](#/week/3/run/20260929T011826_smoke_seed1_press_smoke_zero) and [wipe
smoke](#/week/3/run/20260929T011905_smoke_seed1_wipe_smoke_zero): shapes 2432/459/12 and joint order correct, frames
consistent, 0 resets. Nothing touches under zero actions: the press's 34.5 N peak is the drifting arm on the panel, and
the wipe records 0 N. This checks the interface only, not pressing or wiping ability.

**Baselines: the existing checkpoints on the frozen sets, untrained for these tasks.** Commands:

```
./run_unifp_train.py press_eval --task press --headless --press_set {panel,force,recessed} --checkpoint <ckpt> | --zero_actions
./run_unifp_train.py wipe_eval --task wipe --headless --checkpoint <ckpt> | --zero_actions
```

with `checkpoints/unifp_go2d1_insertion_seed3_model_21200.pt` (the key) and `unifp_go2d1_mech_law_model_17499.pt` (the
combiner's mechanism policy):

| set | zero actions | insertion 21200 | mechanism 17499 |
| --- | --- | --- | --- |
| press `panel` (96) | [0](#/week/3/run/20260929T012154_press_eval_seed1_press_panel_zero), all not started | [27](#/week/3/run/20260929T012129_press_eval_seed1_press_panel_ins21200_b): probe 27/48, bare 0/48; 47 slid off | [29](#/week/3/run/20260929T012218_press_eval_seed1_press_panel_mech17499): probe 18, bare 11; 66 slid off |
| press `force` (48) | — | [10](#/week/3/run/20260929T012244_press_eval_seed1_press_force_ins21200): 7/12 at 5 N, 3/12 at 10 N, 0/24 at 20–30 N | [7](#/week/3/run/20260929T012309_press_eval_seed1_press_force_mech17499): **25 damaged** (> 100 N), all 24 at 20–30 N among them |
| press `recessed` (72) | — | [15](#/week/3/run/20260929T012335_press_eval_seed1_press_recessed_ins21200): probe 15/36, **bare 0/36, all missed** | — |
| wipe (36) | [0](#/week/3/run/20260929T012401_wipe_eval_seed1_wipe_zero), all not started | [14](#/week/3/run/20260929T012439_wipe_eval_seed1_wipe_ins21200): contact 100% of the motion, 22 incomplete (median 86% wiped) | [9](#/week/3/run/20260929T012520_wipe_eval_seed1_wipe_mech17499): 24 incomplete, 3 never found the panel |

No falls anywhere. Three things the numbers show:

- **Pressing fails by sliding off the cap.**
  - The bare jaws' trace ([first scoring](#/week/3/run/20260929T011952_press_eval_seed1_press_panel_ins21200)): during
    the hold the tool carries 8 N across the axis against about 20 N along it. That is at μ = 0.4, so it slides onto
    the panel.
  - Both policies do worst high up: 0/32 at 0.86 m for each.
  - The mechanism policy has no force limit of its own (F-109) and breaks the stiffer buttons.
  - That first scoring filed a slide after full travel as "not held". "Slid off" now ranks above "not held" whenever
    the tool left the cap while still pushing; the `_b` run is the same trace re-scored.
- **The recessed set behaves as declared.** No bare-jaw episode touched the cap, and the probe pressed it 15 times.
  The geometry, not the controller, decides the bare jaws' result here.
- **Wiping keeps contact and misses the path.** Every insertion-policy wipe finished the path by the layer's
  kinematic measure (97%) and held contact throughout, at 2.2 N RMS force error. The diagnostics added to `wipe_eval`
  ([re-run](#/week/3/run/20260929T012649_wipe_eval_seed1_wipe_ins21200_diag)) say why coverage still falls short:
  - The pad is tilted a median 8.2° to the true panel while wiping (p90 15°), so it presses with its rim.
  - The contact is 9.6 mm off the path at the median and 21 mm at p90.
  - Incomplete episodes: tilt 9.2° against 5.6° for the successes, path error p90 21.7 mm against 13.3 mm.
  - The approach command carries only elevation, and its azimuth follows the goal's bearing as the goal moves along the
    stroke. So a rigid pad cannot be held square to a fixed panel by command. The plan's next tool, a compliant pad,
    is the obvious remedy.

What none of this shows:

- Anything a trained press or wipe policy does. See the pilots below.
- Anything on hardware.
- Any calibrated number: the button, panel, pad, friction and stiffnesses are invented.
- PhysX contact: both plants are analytic. The key is the only task with a PhysX check.

**Training pilots: do the tasks train?** Each task got 400 iterations from the insertion checkpoint with the insertion
recipe's settings, then its frozen sets. One seed, a pilot, not a trained policy.

```
./run_unifp_train.py train --task press --num_envs 4096 --iterations 400 --headless --tool_point jaw_centre \
    --resume_from checkpoints/unifp_go2d1_insertion_seed3_model_21200.pt --resume_arm_std 0.15 --entropy_coef 0.002 \
    --fixed_learning_rate 1e-5 --run_name press_pilot_c            # and --task wipe ... --run_name wipe_pilot_c
```

- **First attempt, adaptive learning rate** (the command without `--fixed_learning_rate`). I got this wrong.
  - I stopped [the first run](#/week/3/run/20260929T012752_train_seed1_press_pilot) at 58 iterations because
    `Press/missed_fraction` read 1.0. That reading alone was not a defect. The debug runs
    ([staggered starts](#/week/3/run/20260929T013255_play_seed1), [with noise](#/week/3/run/20260929T013401_play_seed1),
    [64 envs](#/week/3/run/20260929T013554_train_seed1_press_dbg64),
    [resets counted](#/week/3/run/20260929T013920_train_seed1_press_dbg_instr)) showed the first ~40 iterations' resets are
    all episodes rsl-rl started part-way (`init_at_random_ep_len`), which end before engaging.
  - The [rerun](#/week/3/run/20260929T014008_train_seed1_press_pilot_b) went the full 400 and failed for the reason
    Week 2 had already found (v7, v8; F-121). rsl-rl restarts its adaptive rate at 1e-3, and the first update's KL was
    **197**, replacing the policy.
  - [The runner's own policy](#/week/3/run/20260929T013811_train_seed1_press_dbg_runner) (identical to the eval
    loader's, 0 difference) engaged 34–40 of 64 environments with no updates. Within four updates of the adaptive rate
    no hand reached its pre-press pose.
  - The final checkpoint [scored 0/96](#/week/3/run/20260929T020127_press_eval_seed1_press_panel_pilot_b), every tool
    on the panel beside the cap. The insertion recipe recovered from the same spike (KL 243–400) in ~150 iterations;
    this did not in 400. The empty directory `…press_panel_pilot` is an eval killed with the first run before it wrote
    anything.
- **Pinned at 1e-5** (the command above): first-update KL 0.015 (pressing) and 0.016 (wiping).

| | before (insertion 21200) | pilot, 400 its at 1e-5 |
| --- | --- | --- |
| press `panel` | 27/96 (bare 0, probe 27) | [**49/96**](#/week/3/run/20260929T022152_press_eval_seed1_press_panel_pilot_c) (bare 17, probe 32); 36 slid off |
| press `force` | 10/48 | [**20/48**](#/week/3/run/20260929T022219_press_eval_seed1_press_force_pilot_c): 5 N 12/12, 10 N 8/12, 20–30 N 0/24 |
| press `recessed` | 15/72 | [**23/72**](#/week/3/run/20260929T022244_press_eval_seed1_press_recessed_pilot_c): probe 23/36 (6 damaged), bare 0/36 |
| wipe | 14/36 | [**16/36**](#/week/3/run/20260929T024352_wipe_eval_seed1_wipe_pilot_c_eval): contact 100% of motion, 20 incomplete |

No falls.

- **Pressing** ([run](#/week/3/run/20260929T020153_train_seed1_press_pilot_c)).
  - In training the share of episodes reaching full travel rose from 71% to 94% over the 400 iterations, and time
    held there roughly fourfold.
  - The logged success (a *continuous* 1 s at full travel) stayed at 0, so the curriculum never left level 0.2. Under
    exploration noise the cap flickers at its stop; the deterministic frozen sets are what count.
  - The height pattern flipped: 0.86 m went from 0/32 to 25/32 while 0.70 m fell from 13/32 to 3/32.
  - Still failing: sliding off the cap, and every 20–30 N button.
- **Wiping** ([run](#/week/3/run/20260929T022310_train_seed1_wipe_pilot_c)).
  - In training, success rose from 0 to 56% and contact during motion from 16% to 68%, and the curriculum reached
    about 0.4. The training paths are shorter and faster than the frozen set's.
  - On the frozen set, 16/36 against 14 is within noise. The pad's median tilt eased only from 7.4° to 6.8°, and all
    20 failures are still incomplete.
  - The low surfaces improved (0.70 m 10/12 from 4/12); 0.86 m is still 0/12.
  - As diagnosed above, the rigid pad's attitude is the limit, and 400 iterations of training did not move it.
- The first wipe eval aborted at Isaac Sim start-up ("double free or corruption", no run directory) the moment the
  trainer exited. It was re-run and is the linked result.

What the pilots show: both tasks train at the pinned rate from the key's policy, and pressing roughly doubles on every
frozen set in 400 iterations. What they do not show: a converged policy, a second seed, the press curriculum ever
promoting (it cannot as built), or wiping getting past the rigid pad. The checkpoints stay in `logs/` and are not
deliverables.

**Record.** Plan, `unifp_train/README.md` (two new sections), the top-level `README.md` map, G2 set to in progress in
[gates.md](../gates.md), findings [F-123 and F-124](../findings.md). Tests: `python -m unittest discover -s . -p
"test_*.py"` runs 645 and passes (5 skipped); `tests/test_evidence.py` passes on the system Python.

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-123](../findings.md): pressing built; the existing policies hold 27–29 of 96 frozen panel buttons, failing by
  sliding off the cap; the bare jaws cannot reach the recessed button, as declared. Pilot: 49/96 after 400 iterations.
- [F-124](../findings.md): wiping built; the insertion policy keeps contact throughout but wipes 90% of the path in
  14/36, because a rigid pad held ~8° off the panel wipes with its rim. Pilot: 16/36.

## Issues and risks

- **The press curriculum cannot promote as built** (2026-09-29). It promotes on a continuous 1 s hold in the training
  rollouts, which exploration noise makes flicker. Promote on the time held, or on a periodic deterministic evaluation.
- **Wiping's coverage is limited by the rigid pad's attitude**, which the command channel only half controls (elevation
  yes, azimuth no). A compliant or gimballed pad is the plan's next tool, and the natural P3/P4 tool pair for wiping.
- **Every contact number in the new tasks is invented.** Button forces, travel, click, the panel's friction, the
  pad. G2's calibration needs real buttons and panels and a force instrument.
- **Resume at a pinned rate.** Resuming another task's checkpoint at rsl-rl's adaptive rate replaces the policy on the
  first update (F-121's note; again today). Every P0–P4 variant will resume from a checkpoint.

## Next week

- Pressing: a real training run at the pinned rate with the curriculum fixed to promote on time held, evaluated every
  200 iterations on all three frozen sets; a second seed; then a PhysX check of the button, as `demos/insertion` is
  for the key.
- Wiping: a compliant pad (as the key's compliant grip), then the same pilot; declare it as the second tool.
- Measure a real button's travel and force curve and the combiner door's surface friction, to replace invented values.

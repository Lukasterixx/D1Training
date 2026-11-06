# Week 8 — 2–8 November 2026

**Status:** in progress
**Focus:** Freeze policies and demo setup; run untouched simulation and physical test manifests

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Select checkpoints from validation, freeze configurations and complete G3 before comparative claims.
- **Hardware and camera:** Freeze the tagged demonstration setup and execute the physical final-test manifest; collect synchronised sensor, force and phase traces.
- **Evidence and deliverable:** Assess G7: at least 16/20 successes across four placements × five repeats, no falls or force-limit violations, no intervention during successful sequences. Report all attempts.
- **Gate focus:** G3; G7 final evaluation.

## Checklist

- [ ] Checkpoints selected on validation and hashes/configurations frozen
- [ ] Matched budgets and at least three independent training seeds verified (G3)
- [ ] Untouched final simulation and physical manifests executed
- [ ] G7 repeated-trial criterion evaluated without changing thresholds
- [ ] Calibration, RGB-D/tags, joint states, forces, commands and phase events archived
- [ ] Post-freeze change/rerun rules recorded

## Log

<!-- Dated entries (### YYYY-MM-DD · topic). For each: the exact command or change, the outcome in numbers,
     and what it does and does not show. Record runs with ./dashboard.py record. -->

### 2026-11-06 · Report-locked commands made the standard

Lukas: "can we make lock_reports the default everywhere in this repo and the standard from now on". No run launched.

- `arm_link.py` is locked by default, so every program through it is too: the door, key and insertion programs and the holster draw. `--no_lock_reports` goes back to the clock, and `run_holster_on_dog.sh --no_lock` passes it.
- `D1Client._pace` holds every motion command out of the stall window (`stall_wait_s`: 15-88 ms after a report, at most ~38 ms held), because the `d1_ui` console, the pick demo, the `d1_hardware` CLI and its controllers, and `d1_ik_controller` send on their own timing. Release is not paced. `report_gate = False` is for measuring the stall, and `arm_command_rate_test` starts with it off.
- `--blocks gate` is built to test that gate on the arm: a 10 Hz clock through it, held and swinging. **The gate has not run on the arm.** Its 15 ms lower edge is the closest offset the records cover; 20 ms is the closest tested on purpose.
- `CLAUDE.md` carries the rule. `docs/thesis_b_plan.md` notes the measured rate. `motor_model.D1_COMMAND_HZ` is left at 10: the UniFP training session is changing it, and the position-only task and UniFP still hold 5 steps until it does.
- **Tests:** full suite 1,139, 23 failures and 2 errors, the same 25 with this work's files at `b485493d`, so none come from it. 21 are `test_key_robot` ("no arm pose in front of the key from which the whole turn solves..." and others), the rest `test_insert_robot`, `test_bezel_image`, `test_payload` and `test_remote_buttons`. Not investigated. `test_arm_timing.py` (37) passes.
- **Not covered:** P2Dingo's `arm_bridge.py` (the website's arm panel) is another repo and streams at 10 Hz on its own client. A full `d1dogdemo` has not run locked: the follow-driven phases (door, key insertion) are untested on it. The dry runs' `SimulatedLink` still steps at 10 Hz.

### 2026-11-06 · The demo's draw, all joints, with the key

Lukas: "can we test more complex motion with all the joints? it is safe to run the unholstering stage of the d1dogdemo".

The demo has no stage for the draw alone, and `holster_dog.plan` is not its motion: up to 5 degrees off at the grip, 3.4 through the draw, the
jaws shutting less, which could bind a real key in its sleeve. So `extract_demo_draw.py` took the demo's own commands from
the 29 October chain (`logs/combiner_door_robot/20261029T034118_insert_insert_door_pull_close_fast_live`) into `demos/combiner_key/robot/demo_draw_20261029.json`, and `holster_dog.py --demo_draw`
plays them with its stops and recovery. The trace holds only goals for joint moves, and streamed points peak at 80 deg/s where the link once clipped them,
so joint moves are planned to those goals and fast stretches retimed to 44 deg/s. `arm_link.py --lock_reports` puts the link on `ReportClock`; `run_holster_on_dog.sh --lock` passes it.

    demos/combiner_key/robot/run_holster_on_dog.sh --demo_draw --lock --timing          # dry
    demos/combiner_key/robot/run_holster_on_dog.sh --live --demo_draw --timing          # 10 Hz
    demos/combiner_key/robot/run_holster_on_dog.sh --live --demo_draw --timing --lock

- [Dry](#/week/8/run/20261106T023302_dry_lock): all 18 moves on the simulated arm. The ROS process aborted at exit, after the record was written ("terminate called without an active exception").
- [10 Hz](#/week/8/run/20261106T023519_live): finished. 46 skips in 390 cycles, 16.2 of 54.4 s lost, to 0.71 s; worst error 45.8 degrees; `turn` 4.22 s for 3.0. Lukas: "It worked. I did notice the stalling was happening in this run".
- [Locked](#/week/8/run/20261106T023651_live_lock): finished. 2 skips in 449 cycles, 0.42 of 50.1 s lost, 0.21 s at most; worst error 10.2 degrees; `turn` 3.01 s for 3.0. Lukas: "ok that run was perfect, no stalls. this confirms it".
- The jaws stopped at -3.5 to -4.8 units in both: the key was in them, drawn and put back.
- The two locked skips are one event at the end of `back`, as the key reached its seat, no command near the window. Not explained.
- [The error through both](figures/d1_demo_draw_10hz_vs_locked.png).

The arm ended folded (Joint1 -0.5). `test_arm_timing.py` (30) covers the replay, the link's lock and recovery on the demo's moves. `tests/test_remote_buttons.py` has one error, in the door node
(`CombinerDoorNode` has no `odom`, `node.py` line 1562), a file not changed here; not fixed.

- **Shows:** the cure on all six joints under a real load, by the record and by eye.
- **Does not show:** repeatability (one run each), walking or standing, how the key seated, or any policy. The demos are unchanged.

### 2026-11-06 · Report-locked commands on the arm

Lukas: "you can go ahead with the hardware testing. I am watching and there is nothing near the robot which is sat down with the
d1 arm on top". `d1update` (six files, no deletions), then on the dog:

    demos/combiner_door/robot/run_command_rate_test_on_dog.sh --blocks phase
    demos/combiner_door/robot/run_command_rate_test_on_dog.sh --blocks swing

- [Held](#/week/8/run/20261106T020724): 10 Hz 15 skips in 20 s; locked at 20, 50, 80 and 20 ms none (716 cycles); aimed at the window, 103 ms,
  53 of 56 cycles skipped. The arm moved 0.1 degrees. The first commanded block moved the jaws from 57.1 to the arm's limit, 50.3: the test
  commanded the reported 57.1, outside the arm's own window (F-063). Harmless, unplanned.
- [Stray start](#/week/8/run/20261106T021040): my mistake. A status check carried `--blocks swing --help`, and the wrapper only takes `--help` first, so it
  started: the test's argparse exited before any command. 2.2 s of capture, no `rate_test.json`, the echoes level with the reports (20, 21), the pose unchanged.
- [Swing, stopped](#/week/8/run/20261106T021112): 10 Hz 12 skips, 2.90 degrees rms; locked 0, 0.47. Then the test's guard stopped it: my bug, the between-blocks
  check saw the arm 3.5 degrees out, still returning. Fixed.
- [Swing, four blocks](#/week/8/run/20261106T021303): 10 Hz 17 and 14 skips, 3.35 and 3.40 degrees rms; locked 0, 0.47, then 1 skip (0.21 s), 0.98. That skip had no
  command near the window; `ReportClock` then dropped a slot on the 100 ms floor. Fixed.
- [Swing, final clock](#/week/8/run/20261106T021615): 10 Hz 15 and 15 skips, 2.89 and 3.44 degrees rms; locked 0 and 0, 0.45 and 0.44, command period at most 114 ms.
  [The motion](figures/d1_swing_10hz_vs_locked.png): flats and lurches at 10 Hz, none locked.

The arm ended at -2.4 degrees, folded, holding, jaws at 50.3. Two tests added for the two bugs; each fails without its fix (`test_arm_timing.py`, 18).
The first version of the guard test passed without the fix, so it was rewritten.

- **Shows:** the cause (F-238) by the control, and the cure on one joint (F-239).
- **Does not show:** multi-joint motion, load, a walking dog or any policy. The arm link and demos are unchanged and still pause.

### 2026-11-06 · The D1's pauses traced to command phase

Lukas asked for more solutions to the arm's 0.3-0.7 s drops, which UniFP cannot run through. Offline only: no run launched, the
arm not touched. `arm_gap_phase.py` lines each command of the 23 October records up with the arm's report cycle, which
F-200 to F-203 never did:

    python demos/combiner_door/robot/arm_gap_phase.py logs/arm_command_rate/20261023T050700 logs/arm_command_rate/20261023T051030 logs/holster_dog/<the 13 live runs>

- **What it shows:** the pauses begin when a command lands 98-106 ms after a report, 5-13 ms before the next is due
  ([figure](figures/d1_stall_vs_command_phase.png)). At 10 Hz held: 36 of 42 such cycles skip. Held, all 52 skips of 590 cycles had
  a command in 88-110 ms. Moving, 267 of 283 of 2,806 cycles, and 16 of 2,301 skipped outside it. 28% of the time is lost at 10 Hz.
- **Why 10 Hz is so bad:** 100 ms against 111 ms drifts a command into the window about once a second, and once stalled the
  next command, 100 ms on, is in the window again. At 5 Hz the same hit costs one report (0.21 s).
- **F-202 re-read:** the plain 2 Hz block never put a command in 98-106 ms, so "none at 2 Hz" is a miss, not a safe rate.
- **Does not show:** cause. It is a correlation in old records; nothing locked has been sent. The "every 5th" blocks' 8 commands in
  the window with no skip are unexplained.
- **Elsewhere:** Zeng's report ([PDF](https://eloisezeng.people.caltech.edu/documents/32338/2025_SURF_Final_Report.pdf)) edited the mode 0 execution
  time over SSH (0.04 to 0.1 s, then 0.01 s at 100 Hz) and the arm stayed shaky, then failed; it gives no user, path or file. Web
  searches found nothing else on this. The SSH route stays a last resort.

Built, not run on the arm: `d1_hardware.ReportClock` and `D1Client.servo_report_s`; `--blocks phase` and `--blocks swing` in
`arm_command_rate_test.py`, with a 103 ms block as the control; `arm_feedback_timing.py` records angles. `test_arm_timing.py`
(16 tests, a pretend arm that stalls in the window) passes, with the door suite (111) and `tests/test_d1*` (82).

### 2026-11-06 · The retrained reach policy on the dog's legs: the path, an offline check, and a first dry run

**Goal (Lukas):** the whole-body reach policy on the real dog, arm folded and sent nothing: a dry run, then a live stand.
The policy is the corner retrain `unifp_go2d1_isaaclab_corner_delay_model_59600.pt` (F-232's candidate), with
`..._corner_rate_model_59600.pt` as the alternative; both committed from the other PC (`e879afc0` on `main`).

**The change.** `press_live.py --presses 0` already stands the dog on a policy, so the reach runs through it with its stops
and handover unchanged. `export_policy.py` writes 18-output exports (`checkpoints/reach_force_corner_{delay,rate}_model_59600.npz`,
numpy against torch to 2.3e-5 / 3.4e-5); `leg_policy.py` keeps all 18 actions for the next frame and adds `reach_command` and
`VirtualArm`; `press_live.py` takes `--reach_goal` and `--reach_arm_obs`, and refuses a press with a reach export. The six
arm outputs are recorded and go nowhere: the program has no arm sender and the arm link runs `--observe`. 48 tests pass
(`tests.test_press_live`, `tests.test_leg_shadow`), two of them new.

**Offline, before the dog** (the real `59600` networks on a static settled stand, level, no closed loop; scratch script,
not kept; the leg target offset from default, max / rms, after the history fills):

| `corner_delay` sees | goal | legs, rad | arm output, max rad |
| --- | --- | --- | --- |
| the folded arm | the folded hand | 0.41 / 0.21 | 11.8 |
| the folded arm | `EE_GOAL_INIT_END` | 1.03 / 0.37 | 15.9 |
| a virtual arm following its outputs | `EE_GOAL_INIT_START` | 0.24 / 0.10 | 1.2 |

- `corner_rate` was worse under the virtual arm (0.56 / 0.17, arm 11.6 rad), and `58400` worse again (1.73 / 0.28).
- Shown the folded arm, the policy asks 12-16 rad of it for as long as it runs: a state it never trained in, ~91 degrees
  off its default on Joint2 and Joint3. A static arm inflates this in every row where the arm cannot arrive, so these rows
  rank the options and do not predict the dog.
- On the states of [the 23 Oct stand](#/week/6/run/20261023T133506_leg_press_live) (base 0.206 m, 8 degrees nose up: round 4's sit-back) every combination read 0.5-2.2 rad, so the start stance matters.

**The first dry run: [the dog was lying down](#/week/8/run/20261106T012149_reach_stand).**
`demos/combiner_key/robot/run_press_on_dog.sh --policy checkpoints/reach_force_corner_delay_model_59600.npz --presses 0 --stand_s 20 --print_s 4`
(defaults: goal `init_start`, arm `virtual`.)
- **Shows:** the path works on the dog. 1001 steps, period median 0.020 / p99 0.0205 s, inference 1.76 / 2.11 ms, 0 LowCmd
  sent. The arm link connected (99.9% measured) and read the arm within 7.2 degrees of `ARM_REST_POSE` on every joint.
- **Does not show:** anything about the policy on a stand. The base read 0.070 m with FR's calf past its limit, so the leg
  numbers (0.42 rms, 0.78 max) are a lying dog's. It has not driven the legs, in closed loop or at all.

**Dry on the standing dog** (Lukas stood it; base 0.333 m, roll -1.8, pitch -1.3 degrees; nothing sent in any run, 0 LowCmd;
the leg target's offset from default, the steady state after the first 2 s):

| run | policy sees | legs max / rms, rad | a step, rad | arm output, rad |
| --- | --- | --- | --- | --- |
| [`corner_delay`, 15 s](#/week/8/run/20261106T012425_reach_stand) | virtual arm, `init_start` | 0.35 / 0.19 | 0.002 | 1.4, reached exactly |
| [`corner_delay`, 60 s](#/week/8/run/20261106T012647_reach_stand) | virtual arm, `init_start` | 0.36 / 0.195 | | 1.4, no drift in any 20 s |
| [`corner_delay`, 15 s](#/week/8/run/20261106T012444_reach_stand) | **real folded arm**, folded goal | **25 / 19.5** | | **25** |
| [`corner_rate`, 15 s](#/week/8/run/20261106T012503_reach_stand) | virtual arm, `init_start` | 0.53 / 0.21 | 0.007 | ~18 |

- **The runaway:** past 1 rad at step 46 (0.9 s), most leg targets at the 25 rad clip after that. Live, the legs would have
  gone to their clamped limits. So F-237: `--reach_arm_obs measured` is refused with `--live`, and a new stop
  (`--reach_max_leg_off_rad`, 0.9) goes to damping without sending the target. 50 tests pass, two more new.
- **The first step:** the empty history gives 0.76 rad at step 0 on the standing dog, 0.50 by step 10. Training resets the
  same way, so it is kept.
- **The real arm** moved 0.2 degrees at most in each run (the sensor's noise).
- **The live start, offline:** the 23 Oct handover states ([one](#/week/6/run/20261023T133506_leg_press_live); base 0.29 m, pitch -4.9 to
  -5.8 degrees) held static under the virtual arm: 0.62-0.65 at the first step, 0.61-0.63 at most after, rear calves asked
  0.5-0.8 rad straighter (15-20 N·m at kp 25). Static, so overstated; it stays under the stop.
- **Does not show:** closed-loop behaviour, a rattle (F-196's shows only with the legs driven), or the handover stance on this
  policy. The dry runs held sport mode's taller stand.

Before it: `d1update` (previewed: 0 deletions, nothing newer on the dog) took the dog from its 27 Oct copy to `b485493d` plus
this work, and the 29 Oct `d1t-arm-link` container, exited 7 days, was removed with its log kept in
`logs/leg_press/20261029T034118_leftover_arm_link.log`.

**The test console and the E-stop (Lukas: a GUI "to assist in testing from now on", with a policy list and an emergency stop
"which can be triggered by pressing b on the controller too... immediately hand back to sportsmode if possible").**
[demos/robot_console/](../../demos/robot_console/README.md), started by `demos/robot_console/run_on_pc.sh`.
- **The stop:** B, the button and Esc now go damping -> LowCmd stopped -> SelectMode with no Enter. B used to wait in damping,
  so `press_live.py`'s B changed for terminal runs too. Other stops still wait. SelectMode goes again at 3.5 s.
- **Tested on the dog, dry, sending nothing** (the dog lying, base 0.071 m; `lowcmd_sent` 0 in each):
  - [the button through `docker run -i`](#/week/8/run/20261106T015341_leg_press): 25 ms HTTP, the program's `E-STOP` logged;
  - [the tunnel killed mid-run](#/week/8/run/20261106T015416_leg_press): E-stopped, the server and containers gone;
  - [`kill -9` on the server mid-run](#/week/8/run/20261106T015514_leg_press): stdin closed, E-stopped, containers gone;
  - headless Chrome's worker heartbeat at 5.0 polls a second, the status 0.12 s old at worst; `run_sport_on_dog.sh` read `mcf`.
  - [A reach dry run on the lying dog](#/week/8/run/20261106T015244_reach_stand) stopped on its first step at the new 0.9 rad bound (0.92), as it should.
- **Found by those tests:** the page was blank until its first worker tick (fixed: it draws on load); a false SIGTERM after a
  dry stop (fixed: the program's own `E-STOP` line counts); a lying dog's dry run could unlock live (fixed: 3 s standing needed).
- **Not validated:** an E-stop from a live stand; B stopping any run (its bit is confirmed, F-207); sport mode taking a sinking
  dog; a mouse click on the button in a real browser (the same POST ran from a script). 72 tests pass (`tests.test_press_live`,
  `tests.test_leg_shadow`, `demos.robot_console.tests.test_console`).
- `~/.bashrc` was not changed: adding a `d1console` helper was blocked, so the launcher is a script in the repo.

**Known gap (Lukas asked why):** the UniFP task never modelled the D1's 10 Hz commands, 9 Hz feedback or firmware planner,
which the position-only task does; the corner retrain changed the legs only. It does not limit a folded stand. It would
limit any run where the policy drives the arm, and F-097's experiment (UniFP on the measured arm) has not been run. This
PC's NVIDIA driver is down today, so nothing here was checked in simulation.

**The locked-stream reach fine-tune (reported by the UniFP session, other PC, 2026-11-06; not run or checked here).** That session
reports `62800` tracking at 4.92 / 5.73 cm in sim on the frozen validation set (light legs, its fitted locked arm model), 0 falls,
2.71 of 3.42 N, standing chatter 0.026 rad, 6% of commands clamped, against 19.86 / 22.09 cm and 89% clamped for `corner_delay` 59600
on the same condition. That replaces the 12.4 cm it first gave, which F-045's planner flattered. From a folded start, 27 closed-loop
goals had no falls and no contact on the dog; `corner_delay` struck the dog in 11 of 27. It says this does not show the F-237 runaway
is gone, only that closed-loop sim does not repeat it. It takes F-241 to F-243, so the next finding here is F-244. `62800` is uncommitted
on that PC, works only on the locked interface (15.37 / 22.63 cm on UniFP's 50 Hz soft arm), has no export yet and has not run on the arm (that session said `export_policy.py` and `leg_policy.py` do not handle 18 outputs, which is wrong for this tree, see "What exists for a reach runner" below). Its arm model rests on the Joint1 swing summary, so it asked for the
raw series: the five locked blocks went to it by message, and all ten are now in each swing run's `swing_series.json` (reports from
`feedback_timing/samples.jsonl`, which `logs/` keeps and `results/` did not). Steps under ~2.5 degrees may stop between commands, which
its model does not do: not tested on the arm.

**The UniFP session's correction (same day, later).** It withdrew the "lag 80, dead 20, age 40" arm profile after the raw series showed its fit ran about 60 ms slow. It now fits a first-order lag of 77 ms, 5 ms delay, speed-limited, and reports `62800` at 4.02 / 4.90 cm with 0 falls and `corner_delay` at 16.63 / 18.78 cm on that profile, which replaces the 4.92 / 5.73 and 19.86 / 22.09 above. Its robustness range, free-running score, folded starts and 50 Hz score came from the slow profile and are to be rerun, so the folded-start result above is not to be relied on yet. `armft3_s1` is training from `62800` and replaces it only if it beats it. It takes F-250. Reported by that session, not checked here.

**The arm model replayed on the all-joint draw.** At its request, offline, no arm motion: `python3 results/week_08/runs/20261106T023651_live_lock/replay_models.py` replays the [locked draw](#/week/8/run/20261106T023651_live_lock) (453 commands, 462 reports) through models fitted on the J1 swing, output in `replay_models.txt` beside it. Moving over 5 deg/s, bias removed, lag 63 ms, delay 20, cap 45 gives 0.26-0.30 degrees on J1-3 and 0.51-0.54 on J4-6; a cap of 36 gives 3.2 on J1 and 3.8 on J6, because the reports reach 41-45.5 deg/s. Shows the lag carries to all six joints with the key at these speeds. Does not show a final model: none tried wins on both swing and draw, and my replay of its 77/5 model gives 0.37-0.72 on the swing against its 0.23, which is unexplained until it sends its speed limit and sim step ([F-244](../findings.md#f-244)). The series went to it by message and is in `draw_series.json`.

**F-244 corrected.** The UniFP session could not match my replay and sent its code: its speed limit is the URDF's 72 deg/s, applied as a slew before the lag, where mine capped the lag's output rate. That difference in structure is about 25 ms of time and the whole mismatch; neither replay had a bug. Run its way here, `20261106T021615 swing_lock_20ms` matches its reference values to 0.01 degrees, the four clean swing blocks give 0.229 degrees with the median bias removed (it gets 0.233 and 0.231; `replay_models.txt` prints 0.239, which leaves the bias in and uses one cap for all joints), and the draw 0.23-0.57 at lag 77, delay 5 with each joint's URDF cap. Its point that the lag differs by joint also holds here, which overturns my "no sign": best lag J1-3 65-70 ms, J4-6 50-55 ms, moving error on the wrist 0.71-0.79 down to 0.29-0.43 degrees, each joint's best lag within 10 ms across the run's two halves. `replay_models.py` now has both structures. F-244 is marked superseded by [F-245](../findings.md#f-245). It also reports that steps of 0.05-0.5 degrees keep 0.90 of commanded speed under the lock (n 599), so F-199's small-step stop-start does not appear as lost speed; reported, not checked here, and 9 Hz reports cannot show stops inside a cycle.

**What exists for a reach runner (same day, later).** I told Lukas no policy runner exists and passed on that the export could not do 18 outputs. Both were wrong for this uncommitted tree. `export_policy.py` exports reach policies: rerun on CPU to a scratch file, `corner_delay` 59600 reproduces to 2.3e-5 on actions (up to 44.1) and 5.9e-6 on estimates, 64 random stacks, gate 1e-4. `press_live.py --presses 0` is a legs-only runner, dry on the dog so far. What does not exist is the arm half: nothing sends the policy's arm outputs, `ReportClock` is not used there, the measured arm's speed is passed as zeros, goals are fixed, and the start is the training pose, not the fold. The UniFP session was told this and that pushing is Lukas's call.

**Reach policies jitter (reported by the UniFP session, same day; not checked here).** It asks that no reach policy, `62800` and `armft3` included, go to the arm yet. In sim `armft3` 63800 tracks at 3.58 / 4.28 cm with no falls, but per joint its commands move 10-14 degrees rms a cycle, 42-66% of steps reverse the last and 35-59% are over 10 degrees ahead of the arm, where the goals need about 3-4 degrees. Averaging the actions between commands made tracking worse (4.73 / 5.89 cm), so it reads this as a closed-loop oscillation to fix in training. `armft4_s1` is training with a penalty on command change, with a gate fixed in advance: worst joint at most 6 degrees rms and 30% reversals, else nothing is chosen. Lukas told it to build the arm half; it has `reach_arm.py` and tests on its PC, sending nothing, waiting for this tree. Noted for it here: the arm link's feedback carries no report stamp (`age_s` is refreshed by echoes), and the fold sits on or past the training's joint limits (J2 -90.9, J3 91.7 against 89.95 degrees).

### 2026-11-06 · The reach's two `59600`s into `checkpoints/`, for a whole-body test on the dog

Lukas asked for the UniFP retrain's whole-body policies on `main`, for the agent on the other PC to test them on the
real robot. Week 7 left them gitignored under `logs/` ("That is for Lukas"). No run was launched for this.

    cp -p logs/unifp_train/20261026T005349_train_seed1_reach_r2delay_s1/model_59600.pt \
          checkpoints/unifp_go2d1_isaaclab_corner_delay_model_59600.pt
    cp -p logs/unifp_train/20261026T022733_train_seed1_reach_r2rate_s1/model_59600.pt \
          checkpoints/unifp_go2d1_isaaclab_corner_rate_model_59600.pt

- **The files:** [the delay arm's run](#/week/7/run/20261026T005349_train_seed1_reach_r2delay_s1) (`6d363be2...`) and
  [the `action_rate` arm's](#/week/7/run/20261026T022733_train_seed1_reach_r2rate_s1) (`cb6fb060...`), each sha256-equal
  to its source. `checkpoints/SHA256SUMS` has both, and all 25 of its entries verify.
- **The load check (CPU, no simulator):** through `EvaluablePolicy` each reads iteration 59600, 18 actions, and gives
  finite actions on a random 2432-wide stack.
- **What this shows:** the files are intact and load. Their scores are Week 7's (F-232, `checkpoints/README.md`), all in
  simulation, and were not re-run.
- **What this does not show:** that either can run on the dog. See "Issues and risks". The demos' default stays `58400`.
- **The key and press exports** (`leg_key_force_corner_model_4000.npz`, `leg_press_force_chain_s4_model_11200.npz`) were
  already on `main` from 2026-10-30. They are leg-only: the arm is held.

### 2026-11-06 · The reach on the real arm's interface: what 9-10 Hz costs, and what the rest does

Lukas: "the real arm takes commands at 9-10 hz, and it seems the policy expects 50hz", then "the arm also drops commands
for 0.3-0.7 seconds on occassion in real life". UniFP's task drives the arm like the legs: a 50 Hz target into an ideal
PD (kp 60 / 40), exact angles and velocities back. So the question is wider than the rate.

**New code:** `unifp_train/arm_link.py` puts the real arm between the policy and the joints, each part behind its own
flag on `run_unifp_train.py`, all of them with `--arm_real`:
- `--arm_command_hz`: one command a period, drawn 9-10 Hz per environment, at a random phase, held between. Clamped to
  the joint limits as `d1_hardware` clamps them.
- `--arm_feedback_hz`: the actor reads the arm at 9 Hz, to 0.1 deg (F-020), and its speed as the difference of reports,
  since the arm reports none. The critic and rewards keep the true state.
- `--arm_servo`: the stiff drive of `--arm_actuator d1_servo` (4000 / 400, implicit), the published torques, the measured
  speed ceilings. `run.json` reads them back from PhysX (`arm_joints_loaded`).
- `--arm_planner`: the firmware's fitted trapezoid (`core.TrapezoidTracker`, F-045).
- `--arm_stalls`: for 0.3-0.7 s no command is taken and no report comes. The arm finishes the move it has: it moved ~5
  deg through a gap asked for 15-20 (F-200), about one 4 deg command. Every 1.0-2.8 s by default, F-200's capture.

All off by default. Its draws use their own generator, so the goal schedule does not move: every run below has 0
schedule mismatches, so the rows are paired. `tests/test_arm_link.py`, 17 tests on the CPU.

The command, delay arm's `59600`, 30 s each, recorded by `logs/unifp_train/armlink/ladder.sh`:

    ./run_unifp_train.py eval --headless --small_physx --manifest results/manifests/unifp_isaaclab_validation.json \
        --tool_point fingertip --leg_actuator unitree_geared --leg_armature 0.0002 --leg_knee_armature 0.0007 \
        --leg_delay_min_steps 3 --leg_delay_steps 4 \
        --checkpoint checkpoints/unifp_go2d1_isaaclab_corner_delay_model_59600.pt <arm flags>

Light legs, quiet goal error, median / p90 cm:

| arm | flags | cm | fell | standing chatter, rad |
| --- | --- | --- | --- | --- |
| [as trained](#/week/8/run/20261106T012220_eval_seed1_al_d_base) | none | 1.93 / 2.57 | 0 | 0.022 |
| [10 Hz commands](#/week/8/run/20261106T012422_eval_seed1_al_d_cmd10) | `--arm_command_hz 10` | 3.04 / 3.67 | 0 | 0.024 |
| [9 Hz commands](#/week/8/run/20261106T012451_eval_seed1_al_d_cmd9) | `--arm_command_hz 9` | 3.31 / 4.18 | 0 | 0.023 |
| [9 Hz reports](#/week/8/run/20261106T012520_eval_seed1_al_d_fb9) | `--arm_feedback_hz 9` | 4.03 / 4.63 | 0 | 0.030 |
| [both, soft arm](#/week/8/run/20261106T012551_eval_seed1_al_d_cmdfb) | `--arm_command_hz 9 10 --arm_feedback_hz 9` | 6.36 / 7.19 | 0 | 0.031 |
| [9-10 Hz, dropped commands, soft arm](#/week/8/run/20261106T012621_eval_seed1_al_d_drop) | `--arm_command_hz 9 10 --arm_stalls` | 4.26 / 6.07 | 1 | 0.025 |
| [stiff servo](#/week/8/run/20261106T012651_eval_seed1_al_d_servo) | `--arm_servo` | 4.78 / 9.52 | 0 | 0.024 |
| [servo, planner, 50 Hz commands](#/week/8/run/20261106T012721_eval_seed1_al_d_splan) | `--arm_servo --arm_planner` | 3.50 / 11.57 | 1 | 0.023 |
| [servo, 9-10 Hz](#/week/8/run/20261106T012753_eval_seed1_al_d_scmd) | `--arm_servo --arm_command_hz 9 10` | 9.24 / 10.74 | 0 | 0.027 |
| [all, nothing dropped](#/week/8/run/20261106T012824_eval_seed1_al_d_nostall) | the four, no `--arm_stalls` | 12.03 / 14.14 | 2 | 0.031 |
| [all, dropped every 5-7 s](#/week/8/run/20261106T012857_eval_seed1_al_d_rare) | `--arm_real --arm_stall_interval_s 5 7` | 12.53 / 14.83 | 0 | 0.032 |
| [**the real arm**](#/week/8/run/20261106T012249_eval_seed1_al_d_real) | `--arm_real` | **15.13 / 21.02** | 1 | 0.035 |
| [`action_rate` arm, the real arm](#/week/8/run/20261106T012929_eval_seed1_al_r_real) | `--arm_real` | 14.83 / 23.06 | 0 | 0.032 |
| [`action_rate` arm, nothing dropped](#/week/8/run/20261106T013001_eval_seed1_al_r_nostall) | the four | 11.87 / 13.87 | 1 | 0.029 |
| [`58400`, the real arm](#/week/8/run/20261106T013033_eval_seed1_al_o_real) | `--arm_real` | 14.81 / 19.39 | 1 | 0.062 |

- **The link off reproduces Week 7 exactly:** 1.93 / 2.57 cm, the same as the run it repeats.
- **The rate alone is not the problem:** 1.1-1.4 cm. The reports alone cost more (2.1 cm).
- **They stack.** The real arm is 8 times the error, and every reach policy is the same (F-241).
- **Force goes with it:** realised along the command 0.28 N of 3.37 (as trained 3.17 of 3.42); the estimator's error 3.56 N
  (0.86); base velocity error 0.083 m/s (0.039).
- **Why the servo costs so much:** the policy uses the target as a torque. On the real arm 87% of its commands are past a
  joint limit, the first three joints' targets a median 116 / 52 / 81 deg from the arm, 96% of commands more than 10 deg
  ahead on the worst joint. On UniFP's spring that is a push; on the firmware it is a place to go at 1.25 rad/s.
- **What this shows:** in simulation, on the fitted model of the arm, the current reach policies do not track on it.
- **What this does not show:** nothing here ran on the arm. The planner is a fit (F-045, retention provisional), the
  stalls' frequency one capture, the 9-10 Hz stated, and the stiff drive stands in for a servo whose real stiffness is
  not measured. No walking comparison beyond the frozen set.
- **Also seen:** without `--arm_servo`, `velocity_limit_sim` loads as the URDF's old 1.05 / 1.73 rad/s (`arm_joints_loaded`),
  the values F-033 showed wrong both ways, so every policy so far trained with them.

### 2026-11-06 · The reach fine-tuned on the real arm: stated before any checkpoint is scored

Fine-tuning is needed. Job `armft1_s1`: job `080`'s recipe (the corner, the delay 5-20 ms, 4096 environments, seed 1) from
the delay arm's `59600`, with `--arm_real --arm_stall_interval_s 1 8` (F-200's frequent end to "on occasion"), 3000
iterations. One change to what the rewards read: with `--arm_servo` the arm's torques are left out of `torque_limits`,
because Isaac Lab estimates an implicit drive's torque as stiffness x error, which reads as the limit on any lag.

**Selection, stated now.** On the frozen validation set, light legs, `--arm_real` (stalls every 1-2.8 s, the harder end,
which the run did not train at alone): no falls, then the lowest p90 of quiet goal error. The chosen checkpoint is then
scored, not selected on, at stalls every 5-7 s, with nothing dropped, at 9 Hz and 10 Hz fixed, on zero-armature legs and
on the 50 Hz interface. **Its replacement test:** better than the delay arm's 15.13 / 21.02 on the real arm, with no more
standing chatter than its 0.035 rad.

### 2026-11-06 · The spec changes: commands locked to reports stop the drops, and the model is refitted to that stream

Lukas, relaying the session on the dog's PC: "set d1 command hz to the measured ... train against a steady 111ms stream
... about 150ms from command to report. no stall model should be needed ... locking commands to the arms reports stops the
drops." I asked that session for the evidence. Its numbers, written up there as F-238 to F-240, uncommitted on
`mechanism-demo` at the time of writing:
- **Joint1, moving:** free-running at 10 Hz 73 skipped reports in 710 cycles, 28.4 of 99.4 s lost; locked 20 ms after each
  report 1 in 894. **The causal control:** a command aimed at the bad window (98-106 ms after a report) skips 36 of 42 cycles.
- **All six joints, the holster draw with the real key:** 46 skips in 390 cycles, 29.7% of the time lost, worst joint error
  45.8 deg; locked 2 in 449, 0.8%, 10.2 deg.
- **Residual, not zero:** 3 skips in 2,059 locked cycles, each ~0.21 s, cause unknown.
- **The 150 ms is a tracking lag over ramps** (best time shift, commanded against reported), not a dead time: the first
  report of motion comes 88-90 ms after a command, so a delay of 150 ms would contradict it.
- **"6 steps" would be 120 ms.** `round(50 / 9)` is `interface_timing`'s, the position-only task's. The sender is a true
  111 ms stream; this link is on real time, so it runs one, as 100 / 120 ms on the 20 ms grid, mean 111.2.

**The first model did not match the arm.** `unifp_train/arm_link_check.py` pushes the same triangle through the link and
reads it as the dog's run was read. F-045's planner gave a 54 ms lag against 150, and report age cannot clip a peak. A fit
over servo lag, dead time and report age, refitting acceleration on the F-033 steps at each point (96 points), gave:

| | lag ms | half range deg | speed p50 deg/s | peak | first motion ms | F-033 step RMS deg |
| --- | --- | --- | --- | --- | --- | --- |
| the arm | 150 | 7.9 | 30-32 | 37-40 | 88-90 | |
| F-045, no servo lag, best report age | 106 | 8.6 | 35.5 | 38.3 | 90 | 1.05 (0.34 at its own fit) |
| **locked profile**: lag 80, dead 20, age 40 ms | 152 | 8.2 | 32.2 | 37.3 | 90 | 2.58 |
| the same on training's 20 ms grid | 134 | 8.3 | 30.0 | 40.8 | 100 | |

- **No one set fits both.** The arm answers one 30 deg step and a stream of 4 deg steps differently. F-045's note on
  streaming said so in September (4.3 deg a cycle modelled, 2.5 measured). So the locked profile is a second profile in
  `motor_model`, for streams; F-045 stays for big steps and the position-only task.
- **Dead time and report age do not separate** (only their sum shows, as that session found), and the top fits span lag
  80-100, sum 40-60. Training draws each environment's lag 60-100 and age 20-60 ms.
- **Skips** go in at 0.15% of cycles, each taking its command with it: light noise, not a fitted rate.
- `tests/test_arm_link.py` pins the profile to the measured triangle.

**On the locked stream before any retrain** (F-045's planner, the fine-tune `armft1`'s `60600`, which trained with drops):
- Locking alone does not save the old policy: the delay arm's `59600` is 12.44 / 14.21 cm, 1 fall.
- The fine-tune carries over: 5.27 / 6.51 cm, no falls, 2.01 N of 3.42.
- **Report age has no cliff:** 0, 40, 80, 120 ms give 5.27 / 6.51, 6.09 / 7.03, 7.03 / 8.26, 7.72 / 9.56 cm.
- **The command's place in the window barely matters:** 60 ms after the report 5.49 / 6.47 cm, force 1.63 N.

`armft1_s1` was stopped at `61088` as superseded: its stalls and 9-10 Hz are not what a locked runner will see. Its rule
above is withdrawn with it.

### 2026-11-06 · The reach fine-tuned on the locked stream: stated before any checkpoint is scored

Job `armft2_s1`: job `080`'s recipe from `armft1`'s `61088` (which has already learnt targets as positions), with
`--arm_locked --arm_locked_range`, 2000 iterations.

**Selection, stated now.** On the frozen validation set, light legs, `--arm_locked` at the fit's centre, skips on: no falls,
then the lowest p90 of quiet goal error. The chosen checkpoint is then scored, not selected on: at the fit's two corners
(lag 60, age 20; lag 100, age 60), on F-045's planner, with the command 60 and 80 ms after its report, on zero-armature
legs, free-running with drops (`--arm_real`, what a runner that is not locked would get), and on the 50 Hz interface.
**Its replacement test:** better than `armft1`'s `61088` on the same condition, with no more standing chatter than 0.035
rad, and force reported beside it.

### 2026-11-06 · The reach on the locked stream: `62800`

[The run](#/week/8/run/20261106T025555_train_seed1_armft2_s1) (job `armft2_s1`, 1.8 h, from [`armft1`](#/week/8/run/20261106T013202_train_seed1_armft1_s1)'s `61088`), every 200 iterations scored
beside it. By the rule stated above, **`62800`**: no falls, the lowest p90.

| checkpoint | cm | fell | force realised of 3.42 N |
| --- | --- | --- | --- |
| [delay arm `59600`](#/week/8/run/20261106T025555_eval_seed1_l2_d) | 19.86 / 22.09 | 0 | -0.16 |
| [`action_rate` arm `59600`](#/week/8/run/20261106T025802_eval_seed1_l2_r) | 19.01 / 21.48 | 2 | -0.68 |
| [`58400`](#/week/8/run/20261106T030022_eval_seed1_l2_o) | 18.73 / 22.35 | 1 | 2.35 |
| [`armft1` `61088`, the start](#/week/8/run/20261106T030234_eval_seed1_l2_f61088) | 7.26 / 8.90 | 0 | 1.42 |
| [`61400`](#/week/8/run/20261106T031658_eval_seed1_ft2_61400) | 5.27 / 6.33 | 0 | 2.41 |
| [`62400`](#/week/8/run/20261106T040924_eval_seed1_ft2_62400) | 5.09 / 6.05 | 0 | 2.44 |
| **[`62800`](#/week/8/run/20261106T043047_eval_seed1_ft2_62800)** | **4.92 / 5.73** | 0 | 2.71 |
| [`63087`, the last](#/week/8/run/20261106T044611_eval_seed1_ft2_63087) | 4.90 / 5.78 | 0 | |

- **The replacement test passes:** 4.92 / 5.73 against `61088`'s 7.26 / 8.90, standing chatter 0.026 rad against the
  0.035 allowed, 6% of commands clamped (`59600`: 89%). The delay arm's 59600 on the same condition is 19.86 / 22.09:
  F-045's planner flattered it at 12.44 / 14.21.
- **Scored, not selected on:**

| condition | cm | fell | force, N |
| --- | --- | --- | --- |
| [no skips](#/week/8/run/20261106T044648_eval_seed1_fin_noskip) | 4.85 / 5.56 | 0 | 2.63 |
| [fast corner, lag 60, age 20 ms](#/week/8/run/20261106T044721_eval_seed1_fin_fast) | 4.97 / 5.40 | 0 | 1.95 |
| [slow corner, lag 100, age 60 ms](#/week/8/run/20261106T044753_eval_seed1_fin_slow) | 4.91 / 5.95 | 0 | 2.59 |
| [F-045's planner](#/week/8/run/20261106T044826_eval_seed1_fin_f045) | 4.54 / 5.68 | 0 | 2.18 |
| [command 60 ms after its report](#/week/8/run/20261106T044858_eval_seed1_fin_off60) | 5.30 / 6.14 | 0 | 2.24 |
| [command 80 ms after its report](#/week/8/run/20261106T044931_eval_seed1_fin_off80) | 5.14 / 6.36 | 0 | 2.17 |
| [skips at 1%](#/week/8/run/20261106T045003_eval_seed1_fin_skip1) | 4.97 / 5.75 | 0 | 2.84 |
| [zero-armature legs](#/week/8/run/20261106T045036_eval_seed1_fin_zero) | 5.18 / 6.16 | 0 | 2.76 |
| [free-running, with drops](#/week/8/run/20261106T045109_eval_seed1_fin_drops) | 5.80 / 7.42 | 0 | 1.66 |
| [the 50 Hz soft-arm interface](#/week/8/run/20261106T045141_eval_seed1_fin_ideal) | 15.37 / 22.63 | 0 | 6.22 |

- **It is the real arm's policy now.** It tracks the same on everything fitted, and it no longer works on the interface it
  left, so the demos that build UniFP's soft arm cannot use it as it stands.
- **Unmodelled drops do not break it:** 5.80 / 7.42 cm and no falls free-running, though it never trained on them in this run.
- **What 5 cm is:** a hand following each goal perfectly 150 ms late scores 1.9 cm on this metric (goal speed median 0.20
  m/s, moving 64% of the time; per-episode median, pooled, 512 episodes of the task's own goal schedule). So about 3 cm is
  not the lag, and I do not know yet what it is. It was still falling at the end of the run.

**Folded starts** (`demos/unifp/folded_start_probe.py --real_arm`, 27 goals, standing, forces off, the locked model at the
fit's centre, no skips): the session on the dog's PC saw `corner_delay`, shown the true folded arm on the standing dog, run
away open loop (its F-237). Every episode trained from the zero pose, so I tested it:

| | from | within 5 cm | final error, median / max cm | contact on the dog | peak base pitch |
| --- | --- | --- | --- | --- | --- |
| [armft2 `62800`](#/week/8/run/20261106T045225_probe_seed1_fold_armft2_62800) | zero | 16 of 27 | 4.57 / 7.44 | none | 16 deg |
| | folded | 19 of 27 (median 3.0 s, slowest 7.1) | 4.52 / 6.02 | none | 28 deg |
| [delay `59600`](#/week/8/run/20261106T045243_probe_seed1_fold_delay59600) | zero | 0 | 19.8 / 24.6 | 7 of 27, to 114 N | 23 deg |
| | folded | 0 | 19.4 / 24.0 | 11 of 27, to 696 N | 30 deg |

The goal is a fixed held one, so 5 cm is close to the policy's own error here: read "within 5 cm" with the final error.
Contact is read from the dog's side, because the arm's links report none. This is closed loop in simulation, which is
not the open-loop runaway the dog showed; it says the fine-tune does not repeat that here, not that the dog would not.

- **What this shows:** on a model fitted to the arm, in simulation, a 50 Hz reach policy tracks at about 5 cm through a
  111 ms arm stream, with no falls and no strikes from the fold.
- **What this does not show:** nothing here ran on the arm or the dog. One seed. The frozen set chose the checkpoint. The
  profile rests on one Joint1 triangle and one all-joint lag, with the dog lying down; steps under ~2.5 deg (F-199) may
  still stop between commands, which the model does not do. Walking and contact on the arm are not validated, and no
  runner exists.

### 2026-11-06 · Correction: the locked profile above was fitted to the wrong number, and is replaced

**What was wrong.** I fitted `arm_link_check`'s shift to the 150 ms that session reported. That shift is against the command
*held* from each send; the 150 is against the sends as points, which adds the ~55 ms hold. Read my way the arm is 96-100 ms.
So the profile in the two entries above (lag 80, dead 20, age 40 ms) ran about 60 ms slow, and "matched" was wrong. `armft2`
trained on it, its draws (lag 60-100, age 20-60 ms) all slower than the arm.

**How it surfaced.** That session sent the raw reports of the five locked blocks (crc32 checked; `tests/data/d1_locked_swing.json`).
Replaying the commands as sent and comparing every report:

| model, driven by the commands sent | RMS deg, ~700 reports | shift ms | half range deg | speed p50 | peak |
| --- | --- | --- | --- | --- | --- |
| the arm | | 98 | 7.96 | 31.2 | 37.5 |
| **lag 77, dead 5 ms, speed-limited: the new locked profile** | **0.233** | 96 | 7.56 | 31.1 | 36.2 |
| F-045's planner with 19 ms of report age | 0.366 | 97 | 8.22 | 34.8 | 36.7 |
| the profile `armft2` trained on | 1.93 | 161 | 7.95 | 32.0 | 36.2 |

- **Held out:** fit on four blocks, score the fifth: 0.19-0.32 deg, and 0.85 on the block with the skip, which no arm model
  should fit. Acceleration, deceleration and velocity retention add nothing.
- **Also wrong above:** "F-045's planner alone cannot give it" and "40-60 ms" of dead time and age. With 19 ms of age F-045 is
  close behind, and the data wants almost no delay: 0-20 ms, lag 63-82. F-242 is superseded by F-250.
- **The check itself was the second fault.** Its synthetic triangle landed at arbitrary phases, where the real one is 10
  commands a period, locked to the stream; so it now replays the real commands, through the link on 5 ms and on 20 ms steps
  (94 ms, +-8.04, 31.9, 39.0 on training's). A test now holds the mistaken profile failing.
- **A clock bug, found on the way:** `ArmLink.command` moved its clock on before the command acted, so commands ran one step
  early: 20 ms on training's grid. Fixed, with the lock offset now the mean on any grid. All runs above carry it.

**What it did to the results.** On the corrected profile (F-250):

| checkpoint | cm | fell | clamped |
| --- | --- | --- | --- |
| [`armft2` `62800`](#/week/8/run/20261106T050701_eval_seed1_c3_62800) | 4.02 / 4.90 | 0 | 5% |
| [`armft1` `61088`](#/week/8/run/20261106T050734_eval_seed1_c3_61088) | 5.15 / 6.00 | 0 | 11% |
| [delay arm `59600`](#/week/8/run/20261106T050806_eval_seed1_c3_delay) | 16.63 / 18.78 | 0 | 86% |
| [`action_rate` arm `59600`](#/week/8/run/20261106T050839_eval_seed1_c3_rate) | 15.85 / 17.95 | 1 | 93% |
| [`58400`](#/week/8/run/20261106T050913_eval_seed1_c3_orig) | 15.75 / 18.14 | 0 | 91% |

So every conclusion stands and `62800` does better than reported: the arm is faster than the one it trained on. The numbers
in the two entries above are right for the profile they ran on, which is not the arm. Their battery and folded starts are to
be run again on the final checkpoint.

### 2026-11-06 · The corrected profile held out on the six-joint draw

That session sent the locked holster draw: all six joints, the real key in the jaws, 453 commands and 462 reports over 50.2 s, crc32
checked, `tests/data/d1_locked_draw.json`. Nothing in the profile was fitted to it. `python -m unifp_train.arm_link_check`; RMS deg,
each joint's median bias removed, the skip left out:

| model | swing | draw J1-6, all | draw J1-6, moving over 5 deg/s |
| --- | --- | --- | --- |
| **lag 77, delay 5, measured caps** | 0.233 | .35 .26 .23 .55 .51 .57 | .28 .28 .34 .84 .75 .78 |
| lag 63, delay 20, cap 72 | 0.26 | .36 .26 .23 .54 .51 .57 | .28 .31 .34 .82 .76 .78 |
| lag 77, delay 5, cap 36 | 1.09 | 2.60 .54 .60 1.08 1.70 3.06 | 3.91 1.45 1.53 1.92 2.82 4.71 |
| the profile `armft2` trained on | 1.93 | 1.30 .57 .72 1.29 1.34 1.75 | 1.93 1.56 1.90 2.34 2.23 2.68 |

- **It holds:** 0.23-0.57 deg on a task it never saw, with a load it never saw.
- **The cap:** the draw commands 40-44 deg/s and reports reach 45.5, so a cap near the swing's 36 deg/s is wrong, as that session
  found. This profile's is each joint's measured 69-74 deg/s.
- **The joints differ, which one lag hid.** Each joint's best lag at 5 ms delay: J1-3 65-70 ms, J4-6 50-55 ms; moving error .23 .25 .24
  .43 .38 .29 against .25 .26 .31 .80 .71 .72 at 75 ms. Training draws 60-90 ms, all joints alike; the wrist is at or past its fast edge.
- **Small steps do not show as lost speed under the lock.** Steady ramps in the draw by command step: 0.05-0.5 deg (599 joint-cycles)
  0.90 of commanded speed, 0.5-1 0.98, 1-2.5 1.00, 2.5-5.5 0.98. F-199's stop-start was on the free-running 10 Hz stream, so some of it
  may have been the drops. 9 Hz reports cannot show a stop inside a cycle, so this eases F-199 and does not close it.
- **A disagreement, settled the same day.** That session's replay of the swing first gave 0.72 deg for this model, against my 0.233 and a
  separate from-scratch 1 ms replay's 0.231. I sent it the replay and ten rows. The gap was its model's structure: it capped the lag's own
  output rate, where this one slews at the cap and then lags, and for a 4 deg step that slew is the ~25 ms. Run this way in its own code it
  gets 0.229 deg, the ten rows to 0.01 deg, and the draw's row to 0.01 (its F-245, which replaces its F-244). It also found each joint's lag
  stable across the run's two halves: J1 70/70, J2 75/65, J3 70/65, J4 50/50, J5 50/55, J6 55/55 ms. The halves are not independent runs,
  and neither of us knows why the wrist is quicker. It has not checked the small-step binning.
- **A note on `armft3`'s first sweep point:** its `model_62800` scored 6.36 / 7.52 cm against the installed `62800`'s 4.02 / 4.90. They are
  different weights (all 23 tensors, largest change 0.011): rsl-rl saves after the first update, and a resume's first update spikes.

### 2026-11-06 · A last stage on the corrected profile: stated before any checkpoint is scored

Job `armft3_s1`: `armft2`'s recipe from `62800`, on the corrected profile (`--arm_locked --arm_locked_range`, lag 60-90, report
age 0-20 ms), 2000 iterations. **Selection, stated now:** on `--arm_locked` (lag 77, dead 5, age 0, skips on): no falls, then the
lowest p90. **Its replacement test:** better than `62800`'s 4.02 / 4.90 on that condition, no more standing chatter than 0.035
rad. If nothing passes, `62800` stays. The chosen one gets the battery and the folded starts again.

### 2026-11-06 · The policy shakes the arm: the tracking score had been hiding it

Choosing thresholds for a runner, I read what the best checkpoint actually sends. `arm_link.json` now counts it (`command_step_rms_deg`,
`reversals`, `arm_step_rms_deg`). [`armft3` `63800`](#/week/8/run/20261106T061308_eval_seed1_j_latest), 3.58 / 4.28 cm, no falls:

| joint | command step RMS, deg | steps reversing the last | arm's own move per command, RMS deg | commands over 10 deg ahead |
| --- | --- | --- | --- | --- |
| J1 | 12.3 | 66% | 3.6 | 39% |
| J2 | 14.2 | 50% | 4.4 | 59% |
| J3 | 13.2 | 58% | 3.6 | 43% |
| J4 | 10.2 | 49% | 4.4 | 40% |
| J5 | 10.0 | 51% | 4.1 | 35% |
| J6 | 11.1 | 42% | 5.1 | 46% |

The goals need about 3-4 deg a command at their peak. So the arm is sent back and forth, and even through the simulated lag moves 3.6-5.1 deg
a command, ~35-45 deg/s, all the time. On the real arm that is a shaking arm. The frozen set's tracking does not see it, and neither do the
falls or leg chatter. `action_rate_arm` cannot either: it reads the 50 Hz output, most of which the arm never receives. **Every reach figure
above, `62800` and `armft3` included, comes from a policy doing this, so none is ready for the arm.** It may also be the unexplained error.

**It is not noise a runner can filter.** Sending the mean of the actions since the last command ([run](#/week/8/run/20261106T061855_eval_seed1_j_mean2)): 4.73 / 5.89 cm, worse,
with command steps still 9.7-14.6 deg RMS and reversals 35-46%. So it is a slow oscillation of the closed loop at the command cycle's scale.
([An earlier run of this](#/week/8/run/20261106T061519_eval_seed1_j_mean) is void: the limit clamp read the raw target and threw the mean away, so it reproduced the run above to
every digit. Its title says otherwise. No test had limits on; one does now.)

`armft3_s1` was stopped at `64109`: its tracking had plateaued (3.6-3.9 / 4.3-4.8 cm), and this is the problem now. The rule picks `63800`.

### 2026-11-06 · `armft3` `63800` on the corrected profile: the battery, for the record

Scored, not selected on; the policy jitters (above), so these are not a candidate's figures. Quiet goal error, cm; force of 3.42 N; no falls in any:

| condition | cm | force |
| --- | --- | --- |
| [locked](#/week/8/run/20261106T060527_eval_seed1_ft3_63800) | 3.58 / 4.28 | 2.41 |
| [no skips](#/week/8/run/20261106T062443_eval_seed1_f3_noskip) | 3.63 / 4.42 | 2.51 |
| [the draw's lags, 68 and wrist 52 ms](#/week/8/run/20261106T062607_eval_seed1_f3_joints) | 3.70 / 4.28 | 2.52 |
| [lag 60, age 0](#/week/8/run/20261106T062814_eval_seed1_f3_fast) | 3.85 / 4.57 | 2.31 |
| [lag 90, age 20 ms](#/week/8/run/20261106T063021_eval_seed1_f3_slow) | 3.93 / 4.57 | 2.34 |
| [every joint 50 ms](#/week/8/run/20261106T063230_eval_seed1_f3_wrist50) | 3.98 / 4.62 | 2.76 |
| [F-045's planner, age 20 ms](#/week/8/run/20261106T063440_eval_seed1_f3_f045) | 4.10 / 5.02 | 2.55 |
| [command 60 ms after its report](#/week/8/run/20261106T063649_eval_seed1_f3_off60) | 4.19 / 4.85 | 2.46 |
| [command 80 ms after its report](#/week/8/run/20261106T063858_eval_seed1_f3_off80) | 4.69 / 5.26 | 2.60 |
| [skips at 1%](#/week/8/run/20261106T064109_eval_seed1_f3_skip1) | 3.58 / 4.30 | 2.25 |
| [zero-armature legs](#/week/8/run/20261106T064351_eval_seed1_f3_zero) | 3.74 / 4.67 | 2.75 |
| [free-running, with drops](#/week/8/run/20261106T064559_eval_seed1_f3_drops) | 5.22 / 6.33 | 2.56 |
| [the 50 Hz soft arm](#/week/8/run/20261106T064809_eval_seed1_f3_ideal) | 12.44 / 17.63 | -1.64 |

Folded starts on the corrected profile, 27 goals, standing, no forces, no falls in any:

| | from | within 5 cm | final error, median / max cm | contact on the dog | peak pitch |
| --- | --- | --- | --- | --- | --- |
| [`armft3` `63800`](#/week/8/run/20261106T065058_probe_seed1_fold_armft3_63800) | zero | 26 | 3.27 / 6.41 | none | 13 deg |
| | folded | 26, median 1.5 s | 2.98 / 4.61 | none | 22 deg |
| [`armft2` `62800`](#/week/8/run/20261106T065203_probe_seed1_fold_armft2_62800c) | zero | 22 | 3.54 / 7.14 | none | 15 deg |
| | folded | 26, median 2.5 s | 3.35 / 6.56 | **1 of 27, 45 N** | 28 deg |
| [delay arm `59600`](#/week/8/run/20261106T065314_probe_seed1_fold_delay59600c) | zero | 0 | 15.7 / 20.8 | 5 of 27, to 22 N | 21 deg |
| | folded | 0 | 16.1 / 21.1 | 10 of 27, to 534 N | 29 deg |

**Correction:** "no contact from the fold" for `62800` (F-243) was on the slow profile. On the corrected one it strikes once, at 45 N.

### 2026-11-06 · The reach with a penalty on what the arm receives: stated before any checkpoint is scored

Job `armft4_s1`: `armft3`'s recipe from `63800`, adding `--arm_step_weight -10`, a reward on the squared change in each command the arm receives,
2000 iterations. Quadratic, so 12 deg on six joints costs ~20 times a real 4 deg step on two: about -0.47/s against -0.02/s, beside tracking's
+1.6/s.

**Selection, stated now.** On `--arm_locked`: no falls; then a **jitter gate**, the worst joint's command step RMS at most 6 deg and its
reversals at most 30%; then the lowest p90. Tracking may get worse; that is the trade. If nothing passes the gate, nothing is chosen, and the
closest is reported as that.

### 2026-11-06 · The penalty works, and fixing the shake fixed the tracking; no checkpoint passes the gate

[`armft4`](#/week/8/run/20261106T062448_train_seed1_armft4_s1), weight -10, from `63800`. On `--arm_locked`:

| checkpoint | cm | fell | worst command step RMS, deg | worst reversals | arm's move per command | force, N |
| --- | --- | --- | --- | --- | --- | --- |
| [`armft3` `63800`, the start](#/week/8/run/20261106T061308_eval_seed1_j_latest) | 3.58 / 4.28 | 0 | 14.2 | 66% | 5.1 | 2.41 |
| [`64000`](#/week/8/run/20261106T064100_eval_seed1_ft4_64000) | 3.76 / 4.99 | 0 | 5.3 | 44% | 3.9 | 1.80 |
| [`64400`](#/week/8/run/20261106T070553_eval_seed1_ft4_64400) | 2.41 / 3.15 | 0 | 4.6 | 48% | 3.9 | 2.49 |
| [`65000`](#/week/8/run/20261106T073721_eval_seed1_ft4_65000) | 2.13 / 2.77 | 0 | 4.1 | 41% | 3.4 | 2.23 |
| [`65400`](#/week/8/run/20261106T075810_eval_seed1_ft4_65400) | 1.86 / 2.55 | 0 | 3.9 | 39% | 3.3 | 1.68 |
| [`65799`, the last](#/week/8/run/20261106T081930_eval_seed1_ft4_65799) | 2.15 / 2.67 | 0 | 3.7 | 35% | 3.2 | 2.30 |

- **The jitter was the unexplained error.** Tracking fell from 3.58 / 4.28 to about 2.1 / 2.7 cm as it went, near the 1.9 cm a perfect follower 150 ms
  late would score and the 1.93 / 2.57 of the 50 Hz soft arm. 0% of commands now reach a limit, and leg chatter fell from 0.025 to 0.021 rad.
- **By the rule, nothing is chosen.** Every checkpoint passes on step size; none on reversals, 35-48% against 30%. The closest is `65799`.
- **What the reversals are is not yet known.** The measure counts any step of 0.5 deg or more, so dither about a held goal counts the same as shaking.

Merged `mechanism-demo`'s push (`46fcbf69`, the export, the legs' runner, the locked arm link) into this tree: only the record files conflicted.

### 2026-11-06 · A stronger penalty: stated before any checkpoint is scored

Job `armft5_s1`: `armft4`'s recipe from `65799`, `--arm_step_weight -30`, 1500 iterations. **The same rule and gate as `armft4`'s:** no falls, worst joint's command step
RMS at most 6 deg, its reversals at most 30%, then the lowest p90. If nothing passes, nothing is chosen.

### 2026-11-06 · What decides the runner: the Follower is required, and the legs have to answer

All on `armft4` `65799`, the closest to the gate, standing, no forces, on the corrected profile. (Its [folded starts](#/week/8/run/20261106T082912_probe_seed1_fold_armft4_65799): all 27 within 5 cm, and **4 strike the dog, to 347 N**, where `armft3` struck none.)

**The shake on a held goal, in degrees** (last 3 s, per joint, the probe's centre goal). `armft3` `63800`: 5-17 deg peak to peak, 23-38 deg/s rms. `65799`: 0.2-3.6 deg, 0.4-3 deg/s. So what is left of the reversals is mostly dither.

**The arm link's `Follower` costs little and filters the rest.** It is emulated in the link (`--arm_follower`), tested step for step against the real class. [Through it](#/week/8/run/20261106T082331_eval_seed1_k_follow): 2.17 / 3.34 cm against 2.15 / 2.67, no falls, 2.4 deg command step RMS, 25% reversals, 13% of commands held. That would pass the gate, which was set without it: not claimed as a pass.

**The Follower is required, not optional.** The policy starts on an empty history, as every episode does. In the first second from the default pose, [sent directly](#/week/8/run/20261106T084312_probe_seed1_fold_hand_65799), it asks for up to 64 deg in one command; [through the Follower](#/week/8/run/20261106T084414_probe_seed1_fold_hand_65799_f), 5 deg, and 7 deg of lead. Found because the chain, run end to end, stopped itself on its second tick. `ArmWatch` waits 1 s on lead and step for this, which is only safe through `follow`.

**The stops' thresholds, from [the policy's own targets](#/week/8/run/20261106T084628_probe_seed1_fold_asked_65799_f):** past the first second, through the Follower, at most 19 deg ahead of the arm (p99 11) and 8 deg a cycle, against the 25 and 20 stops. The 84-120 deg gaps at p99 in the frozen evaluation are its 8 N pushes back-driving the arm past its torque, not a jam: without them they are gone.

**With the legs dry the policy runs away.** My plan was the arm live and the legs dry first, one new thing at a time, and the code allowed only that. The fake robot's legs ignore the policy, and that chain drifted to -40 deg, so I simulated it (`--legs_held`: the legs hold their stance, the policy still reads its own leg outputs, as `press_live` keeps them). From the default pose, through the Follower:

| | within 5 cm | final error, median / max cm | contact on the dog | target ahead of the arm, max |
| --- | --- | --- | --- | --- |
| [legs on the policy](#/week/8/run/20261106T084628_probe_seed1_fold_asked_65799_f) | 27 of 27 | 1.57 / 2.81 | none | 19 deg |
| [legs held](#/week/8/run/20261106T084835_probe_seed1_fold_held_65799_f) | 10 of 27 | 27.4 / 77.2 | 8 of 27, to 79 N | 1,476 deg, the clip |

So that stage is not a safe first step, and it is the dry run's pattern on the dog (F-237). `reach_arm.SEND_CLEARED` is False and `press_live.py` refuses `--reach_arm_send`, with or without `--live`; a test holds that. Reading the arm (`--reach_arm_obs reports`) still works and writes nothing. How to go on is Lukas's call.

**Built and tested, not cleared:** `reach_arm.py` (reports, targets, the window, `ArmWatch`, `ArmSender`), its wiring in `press_live.py`, `report_s` on the arm link's feedback, `run_press_on_dog.sh` dropping `--observe` for the flag, `tests/test_reach_arm.py`, `tests/test_reach_live.py`. The other session's `export_policy.py` writes `65799` to numpy to 2.4e-5. Nothing has run on the arm.

**What these do not show:** all simulation or fakes; one seed; standing; `--legs_held` stands for sport mode's stand, which is not modelled.

### 2026-11-06 · `66200` chosen, and the whole-body run built

Lukas, on the choice of a first test: "I am on a strict timeframe so I would prefer to go straight to the whole body test". So legs and arm live together; the arm alone stays refused.

**[`armft5`](#/week/8/run/20261106T082312_train_seed1_armft5_s1), weight -30, on `--arm_locked`:** seven checkpoints pass the gate. By the rule, the lowest p90 of those:

| checkpoint | cm | fell | worst step RMS, deg | worst reversals | force, N |
| --- | --- | --- | --- | --- | --- |
| **[`66200`, chosen](#/week/8/run/20261106T085115_eval_seed1_ft5_66200)** | **2.15 / 2.83** | 0 | 3.13 | 29.9% | 2.55 |
| [`66600`](#/week/8/run/20261106T091404_eval_seed1_ft5_66600) | 2.23 / 2.96 | 0 | 2.97 | 27.5% | 2.78 |
| [`67000`](#/week/8/run/20261106T094134_eval_seed1_ft5_67000) | 2.28 / 3.07 | 0 | 3.09 | 24.5% | 2.76 |
| [`67298`, the last](#/week/8/run/20261106T095726_eval_seed1_ft5_67298) | 1.95 / 3.27 | 0 | | 24.5% | |

**Through the Follower, as the dog would run it** ([centre](#/week/8/run/20261106T091401_eval_seed1_p566200_base)): 2.24 / 3.93 cm; [the draw's lags](#/week/8/run/20261106T091647_eval_seed1_p566200_joints) 2.22 / 3.84; [lag 60](#/week/8/run/20261106T091857_eval_seed1_p566200_fast) 2.23 / 3.75; [lag 90, age 20](#/week/8/run/20261106T092108_eval_seed1_p566200_slow) 2.27 / 4.00; [F-045's planner](#/week/8/run/20261106T092318_eval_seed1_p566200_f045) 2.26 / 3.93; [skips at 1%](#/week/8/run/20261106T092529_eval_seed1_p566200_skip1) 2.24 / 3.93; [command at 60 ms](#/week/8/run/20261106T092739_eval_seed1_p566200_off60) 2.23 / 4.04; [zero-armature legs](#/week/8/run/20261106T093013_eval_seed1_p566200_zero) 2.25 / 3.12. No falls in any; command step RMS 2.3 deg, reversals 21-24%, force 2.6-3.2 of 3.42 N.

**Held goals through the Follower, from the default pose ([run](#/week/8/run/20261106T093328_probe_seed1_fold_p5_66200follower)):** 27 of 27 within 5 cm, final 1.7 / 4.1 cm, no contact, policy target at most 13 deg ahead of the arm, 5 deg a command in the first second. **With the legs held ([run](#/week/8/run/20261106T093436_probe_seed1_fold_p5_66200followerlegs_held)):** 13 of 27, 7.5 / 16.8 cm, no contact, 36 deg ahead: poor, no longer a runaway. F-252's numbers were `65799`'s.

**Stops, set from this policy.** Its legs ask 0.95 rad in the first second and 0.68 after, so `--reach_max_leg_off_rad`'s 0.9 would have ended the run the tick the legs took over: 1.2 for that second, 0.9 after. The arm's lead stop is 25 deg held 0.5 s or 60 deg at once, since a tick past 25 is no harm through the Follower and a stop puts the dog in damping with the arm out.

**The run, in `press_live.py`, tested against fakes on the deployed export with its real stops:** lie down and stand up with the arm folded; on the stand-up gains, out by `HOME_SERVO_DEG` to zero; blend; the policy, `follow` goals; hold; back by home, `tuck`, `fold`; only then lie down. A stop holds and does not fold. The fake link runs the real `check_path`, `PathPlayer` and `Follower`. The routes are the other session's: straight to the fold stalled on the lidar cage on 24 Oct, which simulation cannot see (only the dog's body reports contact), so my 0 N on that line says nothing about it.

    demos/combiner_key/robot/run_press_on_dog.sh --policy checkpoints/reach_force_locked_model_66200.npz --presses 0 --stand_s 20 \
        --reach_arm_obs reports --reach_arm_send --live --start_on_blend

**First-ever on the real arm, all at once in that run:** the arm at zero (nearest logged 25 deg off); `follow` on a locked stream; the policy on the legs with the arm out; a stop from there, on a dog going to damping with the arm held out, whose effect on balance is not known; a fold-back from a policy pose (proven only from home). The other session's `run_holster_on_dog.sh --live --zero_test --step --timing` covers the first, the second and the fold-back, with no policy.

**What this does not show:** nothing has run on the arm or the dog. All simulation or fakes, one seed, the frozen set chose the checkpoint. The goal is fixed at `init_start`. The start gate's bounds on the real stand are not known; refused, the run folds and lies down.

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-237](../findings.md#f-237): the reach retrain runs away shown the true folded arm, and holds under a virtual arm (dry).
- [F-238](../findings.md#f-238): the D1's pauses start when a command lands just before an angle report is due (confirmed on the arm by F-239).
- [F-239](../findings.md#f-239): one command per report ends the pauses: held 0 skips in 716 cycles, moving 1 in 894, tracking 2.9-3.4 down to ~0.45 degrees rms.
- [F-240](../findings.md#f-240): the same on the demo's draw, all joints with the key: 0.8% of the time lost against 29.7%, and Lukas saw no stalls.
- [F-244](../findings.md#f-244): superseded by F-245 (my replay used a different model structure).
- [F-245](../findings.md#f-245): a slew at the URDF limit then a lag fits swing and draw; the lag is 65-70 ms on J1-3, 50-55 ms on J4-6 (offline replay).
- [F-241](../findings.md#f-241): the reach policies do not track on the real arm's interface: they use arm targets as torques.
- [F-242](../findings.md#f-242): superseded by F-250 (fitted to a lag read on a different measure).
- [F-243](../findings.md#f-243): the first fine-tunes on it; they track, and shake the arm.
- [F-250](../findings.md#f-250): the D1 in a stream is a 77 ms lag behind a speed-limited target, 0.23 deg RMS on the swing and held out on the draw.

## Issues and risks

- **Every reach policy before the penalty shakes the arm** (F-251): `62800` and `armft3` move it 3.6-5.1 deg a command. None should go on it. `66200` is the candidate.
- **The two `59600`s do not track on the real arm's interface** (F-241): 16-17 cm on the locked model. They should not go on it.
- **The whole-body run has only run against fakes.** Five things are first-ever on the real arm (the log above). The goal is fixed for a run.
- **Lukas's call:** `D1_COMMAND_HZ` stays 10. At 9, `round(50 / 9)` is a hold of 6, against the 5 pinned in the three position-only manifests and
  `tests/test_sim2real.py`.
- **The other arm policies** (rope, mechanism, lever, insertion) use the same soft interface and are not scored on this one.
- The D1's pauses (F-240): the arm link is locked by default, `D1Client` gates every other sender. The gate and a full `d1dogdemo` are not yet run on the arm. P2Dingo's arm bridge still streams at 10 Hz.
- `nvidia-smi` cannot reach the driver on this PC (2026-11-06): no Isaac runs here until that is fixed.
- **No program on the dog runs a whole-body policy.** The reach has 18 outputs, and the dog's image has numpy and no
  torch. `export_policy.py` and `leg_policy.py` serve the 12-output leg policies that hold the arm, and the door and key
  demos walk on sport mode with a scripted arm. A whole-body test needs an 18-output export, a runner that sends the
  arm's targets to the D1, and a source for the goal commands. None of these exists on `main` or `mechanism-demo` as
  of 2026-11-06.
  (2026-11-06, later: on `mechanism-demo` the 18-output export and a legs-only runner did exist, see "What exists for a reach runner". The arm sender and a live goal source still do not, and no reach policy goes to the arm yet: they jitter in sim.)

## Next week

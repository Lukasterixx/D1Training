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

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-237](../findings.md#f-237): the reach retrain runs away shown the true folded arm, and holds under a virtual arm (dry).
- [F-238](../findings.md#f-238): the D1's pauses start when a command lands just before an angle report is due (confirmed on the arm by F-239).
- [F-239](../findings.md#f-239): one command per report ends the pauses: held 0 skips in 716 cycles, moving 1 in 894, tracking 2.9-3.4 down to ~0.45 degrees rms.
- [F-240](../findings.md#f-240): the same on the demo's draw, all joints with the key: 0.8% of the time lost against 29.7%, and Lukas saw no stalls.
- [F-244](../findings.md#f-244): superseded by F-245 (my replay used a different model structure).
- [F-245](../findings.md#f-245): a slew at the URDF limit then a lag fits swing and draw; the lag is 65-70 ms on J1-3, 50-55 ms on J4-6 (offline replay).

## Issues and risks

- The reach policy's arm has never been simulated at the D1's real rates (see the 2026-11-06 log). Not validated for any
  run that commands the arm.
- The D1's pauses (F-240): the arm link is locked by default, `D1Client` gates every other sender. The gate and a full `d1dogdemo` are not yet run on the arm. P2Dingo's arm bridge still streams at 10 Hz.
- `nvidia-smi` cannot reach the driver on this PC (2026-11-06): no Isaac runs here until that is fixed.
- **No program on the dog runs a whole-body policy.** The reach has 18 outputs, and the dog's image has numpy and no
  torch. `export_policy.py` and `leg_policy.py` serve the 12-output leg policies that hold the arm, and the door and key
  demos walk on sport mode with a scripted arm. A whole-body test needs an 18-output export, a runner that sends the
  arm's targets to the D1, and a source for the goal commands. None of these exists on `main` or `mechanism-demo` as
  of 2026-11-06.
  (2026-11-06, later: on `mechanism-demo` the 18-output export and a legs-only runner did exist, see "What exists for a reach runner". The arm sender and a live goal source still do not, and no reach policy goes to the arm yet: they jitter in sim.)

## Next week

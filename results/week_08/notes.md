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

## Issues and risks

- **No program on the dog runs a whole-body policy.** The reach has 18 outputs, and the dog's image has numpy and no
  torch. `export_policy.py` and `leg_policy.py` serve the 12-output leg policies that hold the arm, and the door and key
  demos walk on sport mode with a scripted arm. A whole-body test needs an 18-output export, a runner that sends the
  arm's targets to the D1, and a source for the goal commands. None of these exists on `main` or `mechanism-demo` as
  of 2026-11-06.

## Next week

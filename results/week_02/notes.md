# Week 2 — 21–27 September 2026

**Status:** not started
**Focus:** Select reusable methods; validate P0 candidate; freeze task/interface definitions and calibrate camera/tool frames

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Finish bounded UniFP and position-only reproduction attempts; select port versus retarget; validate candidate reaching and define slow pose trajectories plus pressing/box fixtures.
- **Hardware and camera:** Calibrate camera-to-base, tag-to-task and tool transforms; record real observations and replay them through robot-side inference with actuation disabled.
- **Evidence and deliverable:** Freeze sequence phases, box opening extent, tolerances, force bounds, comparison applicability, tool splits and evaluation manifests before substantive runs; calibrate contact definitions by Week 3.
- **Gate focus:** G0; candidate G1a; G4 begins. Implementation and scope decision.

## Checklist

- [ ] Reproduction levels and implementation choice recorded; legacy setup kept within its time budget
- [ ] Revised target workspace and zero-action baseline evaluated on frozen cases
- [ ] Candidate P0 evaluated with 100 frozen episodes; reaching claims distinguished from PPO diagnostics
- [ ] Task-command schema frozen: pose/force frames, units, timestamps, interpolation and tool registration
- [ ] Actual box sequence, loads, opening measurement, phase timeouts and tolerances documented
- [ ] Core pressing/box tasks and tool conditions selected; applicability and held-out cases frozen
- [ ] Camera/tag/tool calibration and recorded-state inference replay completed or blockers recorded
- [ ] Physical trial manifests and training budgets defined from pilot costs

## Log

<!-- Dated entries (### YYYY-MM-DD · topic). For each: the exact command or change, the outcome in numbers,
     and what it does and does not show. Record runs with ./dashboard.py record. -->

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

## Issues and risks

## Next week

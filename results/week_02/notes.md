# Week 2 — 21–27 September 2026

**Status:** in progress
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

### 2026-09-21 — Could crouching turn a stiffer combiner handle? A static study

Lukas asked whether the dog could turn stiffer handles by standing, gripping the lever and crouching or sitting
to pull it down with its body weight. In the grip-and-pull the jaws gave way at 0.5 N·m (F-073); the lying push
turned 0.9–1.15 N·m (F-071).

**Method** ([`figures/combiner_crouch_push_study.py`](figures/combiner_crouch_push_study.py), CPU, no
simulator):
- **Stance.** The dog stands level at its 0.274 m base height. The arm's mount is then ~0.35 m up, above the
  lever at 0.30 m, so the gripper can come straight down with the jaws straddling the bar.
- **Stroke.** The arm holds still and the body drops. A vertical push at horizontal distance d from the
  spindle gives torque F·d at every lever angle, with the contact sliding out along the bar. At d = 6.5 cm the
  contact reaches the 105 mm lever's end at 52°, after an 8.3 cm drop.
- **Ceiling.** With the arm still, its joint torques do not change through the stroke, so each stance has one
  ceiling: `press.press_capacity` (gravity plus JᵀF against the published limits). In F-071 that model's force
  matched the simulated push within 20%.
- **Placements.** 4,000 random box poses within 0.9 m, door toward the dog, kept where the box's footprint
  clears a generous dog outline (±0.40 × ±0.17 m) and the arm clears the trunk proxy.

**Result.**
- **Standing.** 439 placements solve. The ceiling is median 0.94 N·m, 90th percentile 1.17, best 1.51. That
  is a push of 14.5 N median and 23 N best.
- **Limit.** Joint2, the shoulder, limits 308 of them; Joint3, the elbow, the other 131.
- **Best stances.** The box stands beside the dog, with the lever about 0.20 m to the side of the mount.
- **Lying.** The same top-down push solves at none of 600 tries: the lever is above the mount.

**What this shows.**
- **Weight is not the constraint.** 1 N·m at 6.5 cm is 15 N, against about 180 N of dog and arm.
- **The arm is.** The force reaches the lever through the arm, so the legs doing the moving does not change
  the torque each joint must hold. What standing changes is the pose: pushing straight down with the gripper
  vertical takes the wrist and base yaw out of the load, and the push goes into the palm rather than along
  the jaws, which is the grip's weak direction.
- **Size of the gain.** About 2.5× the grip-and-pull's 0.4 N·m, but at best ~1.3× the lying push. The
  shoulder carries F × ~0.20 m, and the dog's own width keeps the lever from coming closer.

**What it does not show.**
- **Motion.** Nothing moved. The simulated scene still has the dog lying, and there is no standing or
  crouching leg controller in it.
- **Contact.** Whether the bar reaches the D1 gripper's palm between the fingers is unchecked.
- **The real arm.** Its servo limits under load are unmeasured (F-007, F-023).
- **Stance.** The dog outline and trunk proxy are coarse.
- **Sitting.** Not studied. It pitches the body, so a still arm's gripper would swing through an arc instead
  of dropping straight.

Recorded as [F-077](../findings.md#f-077) (provisional, model only). No runs.

### 2026-09-21 — Stronger arms for the dog: a spec survey (no runs)

Lukas asked what stronger arms could go on a robot dog. The survey used published specs, fetched today.
- **Rated/peak.** Torques are given as rated/peak where the maker gives both.
- **Unconfirmed.** Values with no primary source are marked so.

| Arm | Mass | Shoulder torque | Wrist torque | Payload | On a Go2? |
| --- | --- | --- | --- | --- | --- |
| Unitree D1 (current) | 2.37 kg (unitree.com D1-T page) | 3.3 | 1.7 | 0.5 kg | Unitree lists it as a Go2 module |
| ARX X5 ("ARX5") | 3.4 kg (UMI-on-Legs notes) | EC-A4310 motors, 36 N·m peak; SDK caps 30/40/30 | DM-J4310, 3/7 | 1.5 kg (2 kg in press) | yes: UMI-on-Legs (CoRL 2024), powered from the Go2 battery |
| ARX L5 | ~3.6 kg | DM-J4340, 9/27 | 3/7 | ~1.5 kg | none found |
| Galaxea A1X/A1Y | 4.2 kg | J2 20/50; J1, J3 9/27 | 5/12 | 3 kg at 0.6 m | none found |
| Trossen WidowX AI | 4 kg | 9/27 | 3/7 | 1.5 kg | none found |
| AgileX PiPER | 4.2 kg + 0.5 kg gripper | unpublished | unpublished | 1.5 kg | Go1 (HANDO, 2025) |
| AIRBOT Play | 3.5–3.8 kg | unpublished | unpublished | 1.5 kg | vendor's Go2 EDU kit |
| Unitree Z1 Air/Pro | 4.3–4.5 kg | 33 max | — | 2 / ≥3 kg | UMI-on-Legs judged it too heavy for a Go2 |

**Too heavy for a Go2.** The Go2's rated payload is ~7–8 kg. These arms weigh as much or more:
- DynaArm (8–9.2 kg, 60 N·m peak drives, ANYmal);
- Spot Arm (8 kg, Spot only);
- Kinova Gen3 (7.2–8.2 kg);
- RealMan RM65 (7.2 kg, shown on a B1).

**Heat.** UMI-on-Legs reports that with a 3.4 kg arm the Go2's front-leg motors overheated within 5–10 min
unless the controller minimised front-leg torque.

**Sources.** unitree.com (D1-T, Z1, Go2 pages); github.com/real-stanford/umi-on-legs
(`real-wbc/docs/hardware_design_choices.md`) and `arx5-sdk`; the Galaxea user guide on GitHub; Trossen, AgileX,
Discover Robotics, Duatic, Boston Dynamics, Kinova and RealMan spec pages. Several figures come from resellers
or press.

**Issue found.** Unitree's D1-T page gives the D1 as **2.37 kg**. This repo simulates **3.152 kg**, taken from
support.unitree.com, which could not be reopened today. The two disagree by 0.8 kg, and the figure sets the
arm's load in the whole-body simulation. Weighing the real arm would settle it.

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-077](../findings.md): crouching onto the lever with the arm held still would turn about 1 N·m (median 0.94, best 1.51) by the static model; the arm's shoulder, not body weight, sets the limit (provisional, model only).

## Issues and risks

- **The D1's mass is in question:** unitree.com's D1-T page says 2.37 kg, and the simulation uses 3.152 kg from support.unitree.com (log, 2026-09-21). Weigh the real arm.

## Next week

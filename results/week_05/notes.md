# Week 5 — 12–18 October 2026

**Status:** complete
**Focus:** Train tool variants and connect tagged targets to learned physical box phases

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Register tool variants and train P2/P3/P4 on matched distributions; reproduce lever engagement, actuation and opening in simulation.
- **Hardware and camera:** Validate force-controlled contact and each physical box phase; connect AprilTag task targets to the learned controller.
- **Evidence and deliverable:** Record tool transforms/mass, force-estimator accuracy and task outcomes separately; assemble the complete sequence for Week 6.
- **Gate focus:** G6; preparation for G7.

## Checklist

- [ ] Tool variants registered with measured tip transforms and mass/geometry
- [ ] P2/P3/P4 pilots use matched tasks and declared conditioning differences
- [ ] Instrumented force/contact criteria met (G6) or blockers recorded
- [ ] Box engagement, latch/lever actuation, opening and withdrawal validated separately
- [ ] Tag-derived targets drive the learned force-aware controller
- [ ] Phase transitions and opening-state observations logged with timestamps

## Log

<!-- Dated entries (### YYYY-MM-DD · topic). For each: the exact command or change, the outcome in numbers,
     and what it does and does not show. Record runs with ./dashboard.py record. -->

### 2026-10-18 · The shoulder and elbow motors identified: both a Fashion Star RX18-U100H-M (F-142)

Lukas, from his D1: "I can confirm now that I have the d1 arm that the shoulder and elbow motors at the same
RX18-U100H-M". This settles the open question in F-137 and F-140. Unitree's CAD showed identical shoulder and elbow
housings, and the model has given both 3.33 N·m since 2026-10-09, but an identical housing did not prove an identical
gearbox. They are the same part.

**The datasheet.** Fashion Star, *UART Bus Servo Actuator, 100KG Series*, V1.1.08
([PDF](https://files.seeedstudio.com/products/Fashionstar/Fashion%20Star_U100H_Datasheet_EN_V1.1.08.pdf), linked from
[Seeed's FashionStar page](https://wiki.seeedstudio.com/fashionstar_servo/)). Retailer listings agree on the 9.80 N·m
stall, 392:1 and 230 g.

| at 12 V | value |
|---|---|
| max stall torque | 9.80 N·m (100 kg·cm) |
| max continuous torque | 4.90 N·m |
| rated torque | 3.14 N·m at 35 rpm (3.67 rad/s) |
| no-load speed | 42 rpm (4.40 rad/s) |
| input voltage | 9.0–12.6 V |
| reduction | 392:1, stainless-steel gears |
| encoder | 12-bit magnetic absolute |
| mass, case | 230 g, 63 × 34 × 47 mm |

- **The overload graph** (25 °C ambient), read off by eye: about 3,000 s at 1.7 N·m, ~900 s at 3.3 N·m, ~300 s at 5.6.
  F-141's rope pull holds Joint2 and Joint3 at 3.33 N·m. That is roughly 15 minutes of margin, against holds of seconds.
- **The performance graph:** speed falls from ~38 rpm at 0.5 N·m to ~26 rpm at 7 N·m. The arm's measured ceiling of
  1.20–1.29 rad/s (F-033) is about a third of the rated speed, consistent with F-033's reading that the controller
  sets the ceiling, not the motor.
- **What it shows.** The model's 3.33 N·m for both joints (Unitree's D1-550 URDF value) sits a little above the rated
  torque and below the continuous one. Unitree's published 1.7 N·m for Joint3 is not this motor's limit. So the
  3.33 N·m robot is the one that matches the hardware, and `unifp_go2d1_rope_cad333_model_14600.pt` (F-141) is the rope
  policy for it.
- **What it does not show.** The torque the arm actually delivers. Unitree's D1 page gives a 24 V supply, and the servo
  is rated 9.0–12.6 V, so the servo's voltage is unknown, and so is any current or power limit Unitree sets in its
  configuration. Joint1 and Joint4–6 were not identified. The bench check F-137 and F-141 ask for still stands, now as
  one question: does the elbow deliver ~3.3 N·m, or does the firmware cap it? A pull to stall against a spring scale at
  a known lever arm answers it.
- **Changed:** no value. The comment on `motor_model.D1_EFFORT_LIMIT_NM` now names the part. Its label stays
  URDF-unverified, because the 3.33 is still the URDF's number, not a measurement. The hardware caveats in `README.md`,
  `checkpoints/README.md`, `unifp_train/README.md` and `docs/position_only_environment.md` now cite F-142. F-137,
  F-140 and F-141 each gained a "Followed by: F-142" line. Nothing was retrained or run.
- **A stale doc fixed on the way.** `docs/position_only_environment.md` still gave the arm's speed limits as the URDF's
  1.05/1.73 rad/s, "unverified". The code has used F-033's measured 1.21–1.29 rad/s since Week 1, and the doc now says so.

### 2026-10-24 · Weekly update 5 covers this week and Week 6

No update was written on 2026-10-16. `Weekly_Update_Lukas_5_20261024.docx` covers Weeks 5 and 6 together; how it was
put together and how Update 4's planned tasks were marked is in the [Week 6 log](../week_06/notes.md), "Weekly update 5
(Weeks 5-6), and a figure for it". This week's work in it is F-139-F-142 (the CAD lengths and F-141's rope retrain
are logged in Week 4, 2026-10-09).

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-142](../findings.md): the D1's shoulder and elbow are the same motor, a Fashion Star RX18-U100H-M (Lukas, on the
  arm). Datasheet at 12 V: 3.14 N·m rated, 4.90 continuous, 9.80 stall. The model's 3.33 N·m for both stands, and
  Unitree's 1.7 N·m for the elbow is not the motor's limit. The firmware's limit on the arm is still unmeasured.

## Issues and risks

## Next week

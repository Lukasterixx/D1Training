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

### 2026-09-30 · The arm moved onto the rails' screw positions (F-078)

**Why.** Lukas measured the real D1's mounting holes at 13 cm apart along the rail, the same spacing as the
screws that hold the Go2 EDU's two rails down. In Unitree's rail CAD (P2Dingo's
`meshes/expansion_dock_rails.stl`) those screws are 131.0 mm apart, at x = −84.5 and +46.6 mm in the Go2's
`base_link`, both rails at y = ±29.0 mm, and the rails' tops are at z = 85.5 mm. So the arm's base is centred at
their midpoint, x = −19 mm, and rests on the rail tops. The weld had it at (0, 0, 0.08), Rescue's `ARM_MOUNT_Z`,
which had no measurement behind it and left the base 5.5 mm inside the rails.

**Change.** `weld.D1_MOUNT_POS = (−0.019, 0.0, 0.0855)` is now the one mount. `sim.py`, `run_position_only.py`
and the three demo launchers take it from there. The CPU copies must match it: `position_only/workspace.MOUNT_B` and
`description/build_go2_d1_urdf.py`, whose committed `go2_d1.urdf` had only `arm_mount_joint`'s origin changed. A
full regeneration would now pull in P2Dingo's newer links; see `description/README.md`. `flat_env_cfg.ARM_MOUNT_Z`
is gone. `sim.py`'s teleop reach clamp now measures from the mount's x as well as its z. `run.json` records
`arm_mount_m` for every run from now on. Rescue carries the same change (`arm_weld.D1_MOUNT_POS`).

**Stance, re-measured.** The Week 1 method: 16 envs, zero actions, 12 s with no time limit, mean over the last
2 s ([`stance_probe.py`](figures/stance_probe.py), a scratchpad probe like Week 1's, not a run folder:
`python stance_probe.py <x> <y> <z> <out.json>` in the Isaac env). As a control it was run at the old mount
first. That run reproduced `task_space` exactly, so the two rows compare like for like:

| Mount | Base offset | Tilt | Resting pincer tip (env frame) | Peak tilt | Lowest base |
| --- | --- | --- | --- | --- | --- |
| (0, 0, 0.08), control | (−0.0555, 0, 0.2737) m | 1.03° | (0.3601, −0.0141, 0.7604) m | 5.11° | 0.2602 m |
| (−0.019, 0, 0.0855) | (−0.0571, 0, 0.2736) m | 0.78° | (0.3375, −0.0141, 0.7679) m | 4.75° | 0.2597 m |

The tip moved (−22.6, 0, +7.5) mm. That is the mount's (−19.0, 0, +5.5) plus a base that settles 1.6 mm further
back and 0.25° more level. `task_space.ZERO_ACTION_BASE_OFFSET_M` and `ZERO_ACTION_TIP_M` now hold the new values.
The first one's height also feeds the base-height reward. `TARGET_RANGES` is unchanged: the frozen manifests hold
targets in the environment frame, and their `content_sha256` does not depend on the mount.

**Checks.**
- CPU workspace model ([json](figures/d1_workspace_rail_mount.json), [png](figures/d1_workspace_rail_mount.png),
  `python -m position_only.workspace --out …`): at the settled height, 98.92% of the box is reachable, clear
  and holdable by the pincer tip. The Week 1 figure at the old mount was 99.06%. 0% of the box is within 5 cm of
  the resting tip, and the model now misses the measured tip by 1.3 cm (1.6 cm at the old mount, F-015).
- [verify, 8 envs](#/week/3/run/20260930T053155_769482Z_verify_seed42)
  (`python run_position_only.py verify --headless --num_envs 8`): **25/25**. Link6 and the tool point match URDF
  kinematics at the simulator's angles to 0.35 and 0.60 µm, which also confirms `workspace.MOUNT_B` equals the
  weld. The resting tip after the check's 5 s settle is 2.4 mm from the new `ZERO_ACTION_TIP_M`. The box is
  15.2 cm from the tip, and 0 of 8 sampled targets are within 5 cm.
- Unit tests: 383 of 387 pass (4 skipped as before). `tests/test_workspace.py` pinned the zero-pose Link6 and
  tool point in the base frame; both moved by exactly the mount's shift and are re-pinned. `test_d1_ui` pinned
  the mount itself.

**Four demo tests now fail and are left failing for Lukas to decide.** Each pins behaviour of a scene placed at a
fixed spot in the base frame, and the arm is now 19 mm further from it:
- `test_the_survey_on_the_dog_looks_past_its_head`: the survey camera now stands 0.25 m ahead of the mount, not
  0.20. The head did not move, so the arm needs the next offset to see past it.
- `test_a_pivot_turns_joint1_alone_and_puts_the_view_where_asked`: the pivoted view's height now differs by
  11.8 mm against a 10 mm tolerance. It follows from the survey standing further out on the lying dog's tilted
  axis.
- `test_a_cup_too_wide_for_the_jaws_is_taken_from_inside`: the inside-out grasp is 3.5 mm off the cup's axis. With
  the cup moved 19 mm with the arm the test passes again, so the planner is unchanged: it is a tilt, because the
  cup is further away.
- `test_finds_the_tag_plans_on_the_close_look_and_pushes_through_the_arc`: the box pose from the tag is 15.4 mm
  out against 5 mm. With the box moved 19 mm with the arm it is 6.9 mm, and the arm's extra 5.5 mm of height is
  the likely remainder.

**Does not show.** The mount is inferred from a ruler reading of the hole spacing plus the rails' CAD, not
measured on the robot. The D1 model's own base mesh has its holes 108 mm apart along the rail, not 130, so the
model's base is not the real one there. Nothing trained before today saw this mount: every earlier checkpoint
and the Week 1–2 results are at (0, 0, 0.08). The 256-env zero-action baseline (F-016) was not re-run; `verify`
samples 8 targets.

### 2026-09-30 · The arm's base, measured: 22.7 mm taller and 21 mm wider than Unitree's model (F-079)

**Measured (Lukas).** The real base's cylinder is 126 mm across, and 76 mm from the base's bottom to the rotating
top plate. The rotating plate to the shoulder pivot is right as modelled (56.3 mm).
[`screenshots/d1_base_real.jpg`](screenshots/d1_base_real.jpg), Lukas's photograph of the arm on a desk, shows
one continuous black cylinder on a square flange with cut corners. There is a screw in each corner, a flat top
plate on the cylinder, and the shoulder servo and bracket standing on it. There is no step or spacer partway up.

| | Unitree's `base_link.STL` | Real | Ratio |
| --- | --- | --- | --- |
| Cylinder diameter | 105.0 mm | 126 mm | 1.20 |
| Holes along the rail | 108 mm | 130 mm | 1.20 |
| Bottom to rotating plate (Joint1) | 53.3 mm | 76 mm | 1.43 |
| Rotating plate to shoulder (Joint2) | 56.3 mm | 56.3 mm | 1.00 |

It is the base alone. The model's chain reaches 554.9 mm from the shoulder to the wrist flange, against
Unitree's published 550 mm excluding the jaw, and 680.5 mm to the jaw tip. Every link mesh agrees with its joint
offsets.

**Change.** `d1_arm/measured_base.py` writes `meshes/base_link_measured.STL`: Unitree's base scaled 1.20 in plan,
the cylinder stretched from the 5.0 mm flange top to end at 76.0 mm, and anything above moved up 22.7 mm.
`d1.urdf` uses that mesh for visual and collision, carries the base's centre of mass the same way (z 17.1 ->
22.8 mm), and puts Joint1 at 76.0 mm. Unitree's mesh is kept beside it. `description/go2_d1.urdf` got the same
two edits, and `demos/cup/d1_ui/server.py`'s hard-coded shoulder height too. Rescue's `d1_arm/` takes the same
three files, byte for byte.

**Re-measured.** The same stance probe at the rail mount ([json](figures/stance_probe_measured_base.json)): base
(-0.0571, 0, 0.2736) m, 0.78°, unchanged to 0.1 mm. The resting pincer tip is **(0.3378, -0.0141, 0.7906) m**,
22.7 mm above the morning's value. `task_space.ZERO_ACTION_TIP_M` holds it.
- [verify, 8 envs](#/week/3/run/20260930T065528_498258Z_verify_seed42): **25/25**. Link6 and the tool point
  match URDF kinematics to 0.74 and 1.3 µm. The resting tip is 2.3 mm from the recorded value, and the box is
  17.4 cm from it, with 0 of 8 sampled targets within 5 cm.
- CPU workspace model ([json](figures/d1_workspace_measured_base.json),
  [png](figures/d1_workspace_measured_base.png)): 98.99% of the box is reachable, clear and holdable at the settled
  height, and the model misses the measured tip by 13.5 mm.
- Unit tests: 383 of 387, the same four demo failures as this morning (4 skipped). The combiner's tag pose is now
  12.1 mm out, not 15.4; the other three are unchanged. `tests/test_workspace.py` is re-pinned (Link6 and the tool
  point, +22.7 mm in z). `tests/test_d1_ik.py`'s level-attitude point [0.30, 0, 0.28] no longer solves level: the
  arm stops 22.5 mm short. It moved up with the shoulder to 0.3027, where it solves to 0.03 mm. That is a real
  loss of low level reach close in, not a solver fault.

**Does not show.** The across-rail hole spacing and the flange's shape were not measured. The mesh scales them
to 69.6 mm and 141.6 x 129.6 mm, and that is an assumption. Where the real base's mass sits was not measured
either. The arm above the plate was checked at one dimension and against the published reach, not link by link.

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-078](../findings.md): the D1 is bolted to the rails' screw positions, 19 mm further back and 5.5 mm higher
  than modelled (superseded by F-079 for its stance numbers; the mount stands).
- [F-079](../findings.md): the real base is 22.7 mm taller and 21 mm wider than Unitree's model, so the whole arm sits
  22.7 mm higher; the resting tip is (0.3378, −0.0141, 0.7906) m (provisional: hand measurements).

## Issues and risks

- Four demo tests fail at the rail mount on the measured base: the pick survey, pivot and wide-cup tests, and the
  combiner tag pose. The scenes were tuned with the arm at x = 0 on Unitree's smaller base, and each was listed in
  the 2026-09-30 logs. They need Lukas's call:
  re-place the demo scenes relative to the new mount, or accept the new behaviour and re-pin.
- Every checkpoint trained before 2026-09-30 carried the arm at (0, 0, 0.08) on Unitree's smaller base: its shoulder
  was (+19.0, 0, -28.2) mm from where the real one is. Evaluating one now runs it on a
  different robot; `run.json`'s `arm_mount_m` tells the two apart from here on.

## Next week

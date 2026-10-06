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

### 2026-10-03 — The real D1's base is not the model's: holes over the rail screws, 1.2× the diameter; its height estimated from Unitree's renders

**Lukas** (2026-10-03): on the real robot the D1's four holes line up with the four rail screws, and its base is 1.2× the
model's diameter. He did not remember the height. He had written the dimensions in a Claude chat on 2026-09-30, but
they are in none of the 17 Claude Code sessions on this PC (typed, queued, subagent or file-history records, searched
from 2026-09-27). If that chat was on claude.ai, it is not stored locally.

**What it changes.**
- The model's holes are 108 mm apart fore-aft (F-117). Over the rail screws they are 131 mm apart, a scale of
  131/108 = **1.213**, which agrees with "1.2×". In the model the fore-aft holes sit at the base's lip radius (±54 mm).
- With the holes on the screws, the D1's origin is midway between them: x = (−84.5 + 46.5)/2 = **−19.0 mm**. That is
  21.5 mm further back than the applied (0.0025, 0, 0.0855) m. This assumes the base still sits on the rail tops
  (85.5 mm), with no plate under it.
- Neither has been measured by me. The new mount has not been applied.

**Online.**
- Unitree publishes no dimensioned drawing that a script can reach: its document centre answers automated requests
  with an access-restricted page.
- Retailer listings give the D1-T's 670 × 150 × 100 mm and M6 mounts, which are not this arm's numbers.
- `yuhanliu0121/unitree_arm` on GitHub carries the same `base_link.STL` (same sha256) and the same 53.3 mm to Joint1.
  Its Go2 mount is a by-eye Blender placement, so it says nothing about the real height.

**An estimate from Unitree's product renders.** Two renders were used, the arm-upright diagram and a three-quarter
view, from the Shopify CDN of two retailers; they are not committed. The base was profiled down its centreline and
compared with the model's base sliced in height:
- The model's base: a 5 mm flange (118 × 108 mm, corner tabs), a 2 mm lip at r 54, a 105 mm cylinder to 52.5 mm,
  and Joint1 at 53.3 mm.
- The camera elevation is 20° and 22°, from each top rim's ellipse.
- The lip-to-body width ratio matches the model: 174/170 px against 108/105 mm.
- The cylinder side plus lip is 8% and 11% taller for its diameter than the model's: 51.3 and 52.7 model-mm against
  47.5. The flange matches.
- Scaled by 1.213, that puts the real rim, and Joint1 just above it, at about **69 mm**. The range is 64–73 mm: 64 if
  the renders are uniformly the model, 73 at the larger of the two readings.

This is not a measurement. The renders may not be the arm Lukas has, the pixel edges are ±2–3 px, and F-118 makes
every millimetre of shoulder height a millimetre at the tip. A ruler on the real base replaces it.

### 2026-10-01 — Before the retrain: the D1's four holes cannot all line up with the dock's four rail screws

**Why.** Lukas asked for the demos to be retrained with the D1's new dimensions ("taller base cylinder"), and before
starting: "verify that the d1 model has its screw holes aligned with the 4 go2 dock screws".

**The taller base is not in the repository yet.** `d1_arm/d1.urdf` and `d1_arm/meshes/base_link.STL` are unchanged since
2026-07-17: the base mesh is 57.8 mm tall over a 118 × 108 mm footprint, and Joint1 is 53.3 mm above its bottom face.
There is nothing newer on another branch, in the stash, in P2Dingo or in Downloads. The new numbers were asked of
Lukas. Nothing has been retrained, and no simulator was launched for this entry.

**The check.** Two independent sources, and every model of the robot:

```
python3 description/mount_from_cad.py <GO2简化-带拓展坞-0911.stp>   # re-downloaded; sha256 348cf13b…a407855, as in NOTICE.md
python3 results/week_03/figures/d1_dock_holes.py                     # the committed meshes only
```

- **The dock.** In the STEP, every vertical hole of radius 1–4 mm over the dock was listed. The only four-hole
  pattern on the rails is the M3 screws that fasten them (r 1.7 mm, heads in r 3.25 mm counterbores below the rail
  tops): x = −84.5 and +46.5 mm, y = ±29.0 mm. The committed `expansion_dock_rails.stl` gives the same centres to 0.1 mm.
- **The D1.** Four M4 clearance holes (r 2.25 mm) through a 2.9 mm flange, at (±54, ±29) mm in its base frame. The
  mesh's other 20 round holes are internal: they start 1.8 mm up and do not open on the bottom face.
- **Every model welds the D1 at the same place, square to the dog.** The place is `mount.ARM_MOUNT_POS_M` =
  (0.0025, 0, 0.0855) m, and it appears in:
  - `generated/go2_d1.usd` and both insertion USDs (read with `usd-core`, not the simulator);
  - `description/go2_d1.urdf`;
  - `position_only.workspace.MOUNT_B` and `unifp_train/pose_feasibility.py`.

  `generated/claw/go2_d1_claw.usd` still holds (0, 0, 0.08). It dates from 2026-09-25, and `demos/unifp/run_demo.py`
  rebuilds it from the default mount on every claw run, so it is stale but never used. P2Dingo has no D1.

**Result** ([d1_dock_holes.py](figures/d1_dock_holes.py)):

| | rail screws, x (mm) | D1 holes at the weld, x (mm) | D1 hole − screw |
| --- | --- | --- | --- |
| front pair | +46.5 | +56.5 | **+10.0 mm** |
| rear pair | −84.5 | −51.5 | **+33.0 mm** |
| across (both pairs) | y = ±29.0 | y = ±29.0 | 0.0 mm |

![D1 holes against the dock's rail screws](figures/d1_dock_holes.png)

What it shows:

- **No placement can line up all four.** The D1's holes are 108 mm apart fore-aft and the rail screws 131 mm apart.
  Across the rails the two patterns coincide. Fore-aft, only one pair can be put anywhere in particular.
- **The model follows Lukas's 2026-09-26 choice** (F-117): front pair 1 cm ahead of the rails' front screws. In
  Unitree's CAD that puts the front D1 holes at +56.5 mm, which is 3.5 mm past the rails' ends at +53.0. In the model,
  those two screws have no rail under them.

What it does not show: anything measured on the robot. The CAD is Unitree's "simplified" Go2, and the D1 mesh is
Unitree's model of the base this repository has always used. If the taller base has a different hole pattern, this
check has to be redone with its geometry.

### 2026-09-30 · The arm moved onto the rails' screw positions (F-125)

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

### 2026-09-30 · The arm's base, measured: 22.7 mm taller and 21 mm wider than Unitree's model (F-126)

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
- [F-125](../findings.md): the D1 is bolted to the rails' screw positions, 19 mm further back and 5.5 mm higher
  than modelled (superseded by F-126 for its stance numbers; the mount stands).
- [F-126](../findings.md): the real base is 22.7 mm taller and 21 mm wider than Unitree's model, so the whole arm sits
  22.7 mm higher; the resting tip is (0.3378, −0.0141, 0.7906) m (provisional: hand measurements).

## Issues and risks

- **The press curriculum cannot promote as built** (2026-09-29). It promotes on a continuous 1 s hold in the training
  rollouts, which exploration noise makes flicker. Promote on the time held, or on a periodic deterministic evaluation.
- **Wiping's coverage is limited by the rigid pad's attitude**, which the command channel only half controls (elevation
  yes, azimuth no). A compliant or gimballed pad is the plan's next tool, and the natural P3/P4 tool pair for wiping.
- **Every contact number in the new tasks is invented.** Button forces, travel, click, the panel's friction, the
  pad. G2's calibration needs real buttons and panels and a force instrument.
- **Resume at a pinned rate.** Resuming another task's checkpoint at rsl-rl's adaptive rate replaces the policy on the
  first update (F-121's note; again today). Every P0–P4 variant will resume from a checkpoint.
- Four demo tests fail at the rail mount on the measured base: the pick survey, pivot and wide-cup tests, and the
  combiner tag pose. The scenes were tuned with the arm at x = 0 on Unitree's smaller base, and each was listed in
  the 2026-09-30 logs. They need Lukas's call:
  re-place the demo scenes relative to the new mount, or accept the new behaviour and re-pin.
- Every checkpoint trained before 2026-09-30 carried the arm at (0, 0, 0.08) on Unitree's smaller base: its shoulder
  was (+19.0, 0, -28.2) mm from where the real one is. Evaluating one now runs it on a
  different robot; `run.json`'s `arm_mount_m` tells the two apart from here on.

## Next week

- Pressing: a real training run at the pinned rate with the curriculum fixed to promote on time held, evaluated every
  200 iterations on all three frozen sets; a second seed; then a PhysX check of the button, as `demos/insertion` is
  for the key.
- Wiping: a compliant pad (as the key's compliant grip), then the same pilot; declare it as the second tool.
- Measure a real button's travel and force curve and the combiner door's surface friction, to replace invented values.

# Week 6 — 19–25 October 2026

**Status:** in progress
**Focus:** First complete AprilTag-guided box-opening sequence on the real Go2+D1

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Diagnose transfer failures and preserve matched training/configuration records; prioritise completing the core sequence.
- **Hardware and camera:** Run locate → approach → engage → actuate → open → withdraw → verify using a learned force-aware tool policy; record every attempt.
- **Evidence and deliverable:** A successful pilot establishes integration only. G7 needs repeated trials. If the sequence is incomplete, freeze new capabilities and use Weeks 7–9 for the recorded blocking phases.
- **Gate focus:** G6 entry; G7 pilot and integration decision.

## Checklist

- [ ] Complete sequence attempted after G6 and simulated-sequence validation
- [ ] Door/lid opening verified; lever angle alone is not counted as box completion
- [ ] Every attempt, intervention, force trace and phase failure recorded
- [ ] Pilot success distinguished from G7 repeatability and G3 comparison evidence
- [ ] Week 6 integration decision logged; optional work paused if core sequence is incomplete

## Log

<!-- Dated entries (### YYYY-MM-DD · topic). For each: the exact command or change, the outcome in numbers,
     and what it does and does not show. Record runs with ./dashboard.py record. -->

### 2026-10-19 · The combiner demo on Lukas's real box

Lukas now has the real combiner box and sent two photos: the box, and a close-up of its lock. **What they show.** A
light-grey Shoals "PV ARRAY d.c ISOLATOR" enclosure standing on the floor, portrait, its door hinged on the right
edge with two hinges. Near the top, slightly left of centre, is a black rotary isolator handle with a red grip marker;
its bar points right, toward the hinge, with ON and OFF labels beside it. A yellow WARNING label sits under the bar,
and a caution sticker on the right. The printed 100 mm AprilTag (tag36h11 id 0, from `dock_tag_id0_100mm_A4.pdf`, cut
to its 125 mm border) is centred below the handle. A round quarter-turn cam lock is midway up the left edge. The
close-up shows the lock as a black plastic bezel round a chrome insert with a slotted, pinned face: it needs a key or
tool to turn. Lukas's measurements: 83 × 63 × 30 cm; the handle's centre 16 cm below the top; the tag's top edge 15 cm
below that centre; the lock's centre 6.4 cm in from the left edge, which is the side that swings open.

**Changed.**
- `demos/combiner/geometry.py` now holds the real box: 830 mm tall (the body 5 mm off the floor so the door clears the
  ground plane), 630 wide, 300 deep with the door; the handle spindle 0.67 m up on the door's centreline; the tag's
  centre 0.4575 m up; the lock at y = −0.251 m, 0.4175 m up. `hinge_side = +1` mirrors the door: hinge on +Y (the right),
  lock and free edge on −Y, the lever pointing toward the hinge. The handle's and the tag's horizontal place is assumed
  (centreline); the lever's shape, the masses and the interior are still the proxy's.
- `asset.py` draws it: no stand or glands, the cam lock, a black-and-red lever, hinges on the right, decals under the
  handle, a wider interior. Joint frames flipped so positive still opens outward and lowers the lever.
- The planners (`press.Lever`, `pull.Handle`), `turn_run`, `demos/unifp/props.py` and `mech_env` take the side from
  the geometry. `props.side_stance` keeps "positive = latch side".
- **The dog stands** (Lukas's choice, asked today): `make_resting_cfg(posture="standing")`, the position-only task's
  zero-action stance. The lying dog reaches this handle at 2 of 12 default placements (F-143).
- Search stops and the close look re-aimed for the standing dog and the bigger tag (F-144); `--settle_s` 2 → 4 s;
  default placements kept at 0.62–0.70 m. UniFP demo: the post becomes a 10 mm plinth (the handle stays 0.68 m up)
  and the box range moves out to 0.70–0.74 m. The reach console's default tag size 60 → 100 mm.
- Tests: 688 pass (15 skipped). The asset tests, which need `pxr`, were run separately with Isaac Sim's bundled USD
  (recipe in `demos/combiner/README.md`): 25 pass, including a new one pinning the measured outline, handle, tag and
  lock.

**CPU model (base level), 30 seeded default placements.** Standing: tag seen, close look solved and grip-and-pull
planned at 30/30 (the door to 20–40° in the plan, static ceilings 0.82–0.96 N·m), push at 28. Lying: pull 2/12, push
0/12. UniFP goals: inside the trained sphere at every stance but a 45° latch-side one, where the door's pull reaches a
0.61 m radius (pinned in `test_unifp_demos`). F-143.

**Three live runs** (GPU idle beforehand), each
`./demos/combiner/run_combiner_demo.sh --turn --headless --capture --no_console --ui_feed_port 0 --handle_torque_nm 0.3 --episodes 3`,
seed 42, so the same three placements:

| run | change | tag found | close-look error (mm) | peak lever (°) | grip slip max (mm) | door |
|---|---|---|---|---|---|---|
| [smoke 1](#/week/6/run/20261019T012022_709892Z_combiner_seed42) | standing; search and close look raised for the tag's height | 2/3 | 9.6, 36.9 | 34.8, 40.8 | 155, 136 | shut |
| [smoke 2](#/week/6/run/20261019T013204_701444Z_combiner_seed42) | close look aimed off-centre | 2/3 | 2.9, 2.2 | 17.0, 34.0 | 157, 141 | shut |
| [smoke 3](#/week/6/run/20261019T013846_816348Z_combiner_seed42) | search stops aimed off-centre too; base-motion split logged | 3/3 | 2.8, 3.2, 3.5 | 13.6, 37.6, 26.5 | 157, 26, 143 | shut |

- **Smoke 1.** The close-look frames show a finger of the shut gripper over the tag's lower-right corner, and the
  third placement's tag was under a finger at every stop. Fix: aim the looks so the tag sits up and to the left
  (F-144).
- **Smoke 3.** At the grip, perception is 1–4.5 mm off, but the lever has moved 17–21 mm in the base frame since the
  close look: the base tilted 1.26° and 1.47° under the arm. In the one attempt with 0.32° (5 mm), the grip held
  (26 mm slip) but the lever still stalled at 37.6°, short of the 45° release (F-145).
- The base stayed standing in every attempt (0.273 m at the end).

**What this shows.** The scripted demo runs on the real box's geometry with the dog standing. It finds the tag,
locates the box to ~3 mm, and gets onto the handle. **What it does not show.** Any door opening: 0 of 9 attempts
released the latch. The grasp record (F-075) is from the proxy and the lying dog, and the one grasp it favours that
still solves (pitch 40 roll −1) failed in every attempt here. The real handle's torque, lever shape, the door's mass
and the cam lock's behaviour are all unmeasured. The fingers' place in the image is the simulator's, not checked on
the real camera.

**Next.** Correct the plan for the base's tilt (the IMU gravity vector the sequence already reads) at the align step
and through the arcs, or re-look at the tag from the standoff. Then re-run these three placements. Lukas could also
measure the isolator's turning torque, its lever shape, and the door's weight.

### 2026-10-19 · The door pull: grab the handle and walk the door 90° open

Lukas asked for a second combiner demo: the box unlocked, no handle rotation, just grip the handle and pull while
the robot moves until the door is 90° open. Built on the UniFP demo, which already walks: UniFP's policy takes base
velocity commands, and the demo env had pinned them to zero.

**Changed.**
- `demos/unifp/run_demo.py --task door`; `script.door_phases` and `DoorTiming` (the combiner's reach, then a 7 s
  pull, a 1.5 s hold, release, back to the ready pose in front of where the robot now stands).
- `props.BoxSite.base_pose_m`: the base carried round the hinge with the door, so the handle stays put in the robot's
  frame. On the real box the hinge is on the robot's right: the base steps right while turning left, ending 0.88 m
  right, 0.25 m forward and turned 90°, 0.65 m from the hinge throughout.
- `script.base_velocity_command`, called by `env._walk_base` each policy step: path rate fed forward, measured pose
  error fed back at 1.5/s, clipped to the trained ranges (vx 0.6, vy 0.4, yaw 0.6). The env's velocity resampler
  now writes the script's command, zero for every other demo.
- `LatchController(locked=False)` / `DoorLatch(locked=False)`: an unlocked box, the lever's spring still on.
- Phases can carry orientation paths (`approach_path`, `object_axis_path`) and a base pose path (`base`).
- Tests: `test_unifp_demos.DoorPull` (6): the base orbit, goals inside the trained sphere *of the moving base* for 40
  seeds, the walk command outside the dead zones and inside the ranges, the footprint ≥ 0.10 m from the door and the
  box at every 5° (measured minimum 0.19 m), the control law's frame and clipping, the phase clock. Plus the unlocked
  latch in `test_combiner_demo`.

**Runs** (GPU idle beforehand). All `./demos/unifp/run_demo.py --task door --wrist --headless`, 16 attempts
in 16 environments unless noted:

| run | change | swung ≥ 85° | falls | notes |
|---|---|---|---|---|
| [first, 4 envs](#/week/6/run/20261019T024810_door_unifp_wrist_seed1_door_first) | `--attempts 4` | 2/4 | 0 | 91.6, 91.9° |
| [seed 1](#/week/6/run/20261019T024909_door_unifp_wrist_seed1_door16) | seed 1 | 8/16 | 0 | arrive miss median 5.9 mm |
| [goal correction](#/week/6/run/20261019T024957_door_unifp_wrist_corr_seed1_door16) | `--goal_correction` | 8/16 | 0 | arrive 4.2 mm; failures part-way, not shut |
| [lip claw](#/week/6/run/20261019T025131_door_unifp_wrist_claw_seed1_door16) | `--claw` (lips) | 2/16 | 0 | arrive 79 mm |
| [wrist pitch servoed](#/week/6/run/20261019T025238_door_unifp_wrist_seed1_door16_wrist_joints56) | `--wrist_joints 56` | 3/16 | 0 | arrive 45 mm |
| [2 s grip, seed 1](#/week/6/run/20261019T025326_door_unifp_wrist_seed1_door16_grip_s2.0) | `--grip_s 2.0` | 10/16 | 0 | |
| [seed 2](#/week/6/run/20261019T025447_door_unifp_wrist_seed2_door16_s2) | seed 2 | 7/16 | 0 | |
| [2 s grip, seed 2](#/week/6/run/20261019T025535_door_unifp_wrist_seed2_door16_s2grip_s2.0) | seed 2, `--grip_s 2.0` | 7/16 | 0 | |

- **The walk works.** No falls in 116 attempts. The base's peak pose error is 3–11 cm (median 5.5) and up to 11° of
  yaw. It ends turned 87–95°. When the grip holds, the door is at 87–105° at the end of the hold.
- **The grip is the limit.** Over seeds 1 and 2 the grip held in 15 of 32. Of the 17 failures, 4 doors barely
  moved, 12 lost the handle at 15–39°, and 1 overshot to 108° and came back to 65°. The failures' median miss at
  arrive was 12 mm, against 5 mm for the successes. But attempt 1 of seed 1 arrived 3 mm off and still lost it:
  during the 0.8 s grip the hand pitched up 30° (approach elevation 7° → 36°) and backed 24 mm off the bar,
  dragging the door 8° open before the walk began.
- Of the setting changes, none moved the rate beyond noise: the 2 s grip was 10/16 then 7/16, and outcomes flip at
  the same placement (F-098's behaviour, now on the door).
- Several doors that lost the handle ended shut. Not explained; the upper arm has no contact sensor.

**What this shows.** UniFP, as trained, can walk a door open while its hand holds the handle, driven through its own
velocity command, without falling. **What it does not show.** A reliable grasp: half the attempts lose the handle.
Nor does it show UniFP alone (this is `--wrist`), a door with any closer or a realistic mass (0.6 kg), the real
isolator handle's shape, perception, or clearance off the planned path. F-146.

**Next.** A grasp that holds a pull along the jaws: a hook behind the bar, or the lip claw run with the goal correction
and roll it was tuned with (`--mech`'s settings on this script).

### 2026-10-19 · The wrist camera's bracket is on the real arm; every demo draws it

Lukas: the bracket (F-138's design, exported 2026-10-09) is printed and fitted to the real arm, the RealSense sitting on
top of the pincer. F-138 gained a "Followed by" line. Its seating on the arm is not measured, so the mount stays the
CAD's (`wrist_mount.json`) until a hand-eye check.

**Changed.** The scripted cup pick and combiner demos already drew the camera and bracket by default. Now the rest do:
- the UniFP demos (`demos/unifp/run_demo.py`: cup, combiner, `--mech`, door) draw both on Link6 in every run
  (`--camera_body`, default on; `run.json` `camera_body` records the mount);
- the insertion demo draws them headless too (it did only in the viewer).

All are visual only: no mass, no collider, no rendering. The policies were trained without the camera's weight
(D435i ~72 g plus the bracket, on 1.7 N·m wrist joints). Neither its mass nor a collider is modelled anywhere.

**Checked.**
- [The door pull, seed 1, with it drawn](#/week/6/run/20261019T072249_door_unifp_wrist_seed1_door16_cambody): 8/16, the
  same door angles as without it to the hundredth of a degree, as visual-only geometry should give.
- Because the simulator cannot show the camera hitting the box, a new test checks it on the CPU:
  `test_turn.WristCameraClearanceTests`, 12 grasps × 2 grip radii. The camera and bracket stay at least 29.6 mm off
  the door face, and 24.9 mm off the lever bar, spindle and rosette (the proxy lever's shape; the real isolator's is
  unmeasured). The closest is pitch 80°. At the door pull's level grasp, 100 mm off the face.
- Not checked: a rendered frame showing the parts in the UniFP scene; the camera against the cup's table or the
  insertion's door.

### 2026-10-19 · Switching the reach policy on with the arm folded

Lukas is preparing to run the UniFP reach policy on the real robot and asked what it does if the arm starts
folded. Every training episode resets the arm to its zero pose (`interface.DEFAULT_DOF_POS`); the real D1 rests
folded on its stop at J1/J2 −90.9/+91.7° (F-027). So a folded switch-on is outside anything it was reset into.

**New probe:** `demos/unifp/folded_start_probe.py`. It runs each held goal twice, from the zero pose and from the
folded rest pose, and changes nothing else: `cad333_model_58400`, fingertip tool point, standing, forces off. The
grid is 27 goals (radius 0.35/0.45/0.55 m × pitch 0/20/40° × yaw −30/0/30°). The arm's links have no contact
reporting in the weld, so a strike is read on the dog's trunk, head, hips and thighs.

```bash
./demos/unifp/folded_start_probe.py --headless          # default --hold_s 1.0
```

- First launch crashed before writing a run (the contact readout assumed `Link*` sensor bodies, which don't exist).
  A 2-step launch listed the sensor bodies and also wrote no run.
- [Spawn-drop reset](#/week/6/run/20261019T075656_probe_seed1_folded_start) and
  [the same with contact detail](#/week/6/run/20261019T075729_probe_seed1_folded_start): both starts include
  training's drop from the spawn height (zero start alone pitches 18°). The hand hit the Go2's head in 13 of 27
  folded envs and 0 of 27 zero-pose envs. At the peak contact, J1/J2 were still at −69…−82° / +54…+78°, and J3 had
  already turned 29–68°. **The wrist turns before the shoulder lifts, which swings the hand into the head.**
- [**Standing 1 s hold, then handover**](#/week/6/run/20261019T075835_probe_seed1_folded_start), the main result.
  The dog stands with the arm held at its start pose for 1 s, then the policy takes over with an empty history and
  no previous action. Folded handover pose: [2.1, −90.0, 88.9, −2.3, 4.9, 2.9]°.

| 27 goals, standing | zero pose (trained) | folded |
| --- | --- | --- |
| falls | 0 | 0 |
| reached and held 5 cm for 0.5 s | 27 | 27 |
| time to 5 cm, median / max | 0.46 / 1.18 s | **1.10 / 1.68 s** |
| final error (6–8 s), median / max | 1.16 / 5.34 cm | 1.07 / 6.36 cm |
| hand on the Go2's head (`Head_lower`) > 5 N | 0 | **11**, 0.20–0.66 s after handover, up to 1.04 s in contact, peak 19–1237 N |
| peak base pitch | 3–13° | **8–26°** (centre goal +22.5°, nose down, against −4°) |
| J1 at its 3.3 N·m limit in the first 2 s, median | 40 of 100 steps | 100 of 100 |
| peak arm joint speed, median / max | 1.78 / 1.87 rad/s | 1.87 / 2.25 rad/s |

![folded start](figures/unifp_folded_start.png)

- **Which goals hit the head:** every 0° pitch goal except 0.35 m straight ahead, and all three 0.55 m / 20° goals.
  None of the 40° goals hit it, and nor did the 20° goals at 0.35 and 0.45 m. Low and far goals make the policy
  turn the hand downward while the arm is still folded over the head.
- **How it unfolds (centre goal, no contact):** J1 climbs at a steady, torque-limited rate, −89° to 0° in about
  1.6 s. J2 comes down alongside it, and the tip is within 5 cm at about 1.1 s.
- **History dependence:** at a few goals the arm settles into a different configuration from the zero-start run.
  At 0.55/0/−30, J2 ends at −39° against +4° and the error at 6.4 cm against 0.4 cm. The zero start has outliers of
  its own (5.3 cm at 0.45/20/−30), so the medians are the same.

**What this does not show.** Simulation only, one seed, standing, no forces.
- J1/J2 start 1–3° short of the real rest pose, because the simulated joints cannot be placed past the URDF limits.
- The arm here is UniFP's PD with torque limits, not the D1's firmware position servo. Its 2.25 rad/s peak is
  above the D1's measured 1.21–1.29 rad/s (F-033) and ~0.8 rad/s when streamed (F-046), so the real unfold would
  be slower and **no timing transfers** (F-097).
- The contact forces are PhysX impact values, not a prediction of the real load. What counts is which goals make
  contact, and when.
- Whether the real head sits where the simulated `Head_lower` collider does is not checked. The dock and MID-360
  are drawn as visuals only, so anything they would add to the collision is missing.

Finding: [F-149](../findings.md) (written as F-147, renumbered the same day: another session's lidar finding took F-147 at the same time).

### 2026-10-19 · The lidar and the wrist camera, in every demo and every training setup

Lukas: "I have the go2 and go2_with_mid360 in my downloads. can you add the mi360 onto the robot in all the demos? also add
both the realsense, mount and mid360 into the training setups (but dont retrain yet, I will do so on another computer)",
and: "I would like the lidar mount included".

**The geometry.** `~/Downloads/Go2 v115.stl` and `Go2_with_mid360.stl` (Lukas's Fusion model, metres, one frame). The
second adds the MID-360 and a mount on the front of the trunk, and lacks the first's head D435i and its mount.
`description/mid360_from_cad.py`:
- diffs the triangles;
- registers the STL to `base_link` by ICP of the URDF's trunk mesh. It fits to 0.34 mm median, 0.87 p90, with 0.14 deg of
  rotation, so the frames differ by a translation, (−0.0445, 0.0003, 0.1787) m. The dock alone fitted only to 6.4 mm,
  so the CAD's dock is not Unitree's;
- splits the result:
  - the lidar: 65 mm square, x 0.143–0.208, z 0.096–0.153 m;
  - the mount: a plate on the trunk at x 0.084–0.240, z 0.055–0.094, and a cage over the lidar to z 0.166;
- decimates both to 10%, giving `meshes/go2/mid360.stl` (11,642 triangles) and `mid360_mount.stl` (5,728).

**Changed.**
- `payload.py`: the four parts, each with its mass and source, centre of mass, inertia and box colliders.
  - MID-360: 0.265 kg, published.
  - Mount: 0.155 kg, estimated (124.9 cm³ × PLA).
  - D435i: 0.072 kg, published.
  - Bracket: 0.027 kg, estimated.
- `weld.py`:
  - `payload_visual` (default on) draws the lidar and mount on every robot, so every demo shows them. The demos already
    draw the wrist camera.
  - `payload_physics` adds the colliders (guide purpose, not rendered) and folds the masses into `base`
    (6.921 → 7.341 kg) and `Link6` (0.168 → 0.267 kg).
- `run_position_only.py`, `run_unifp_train.py`: `--payload physical|visual`, default physical. `run.json` records it.
  UniFP training builds `generated/go2_d1_payload.usd`, so a demo rebuilding `go2_d1.usd` never swaps it.
- `position_only/workspace.py`:
  - `clear_of_body` also keeps the arm out of the mount's plate and the lidar's cage. This check is used by every CPU
    planner and by `d1_hardware.py`'s moves.
  - `simulated_masses` / `simulated_coms` take the wrist payload.
- `position_only/verify.py`: the G0 mass check knows the wrist payload when the robot carries it.
- `grasp.proxy_margin` measures to the new boxes too. `tests/test_payload.py` (7 tests).
- Nothing retrained.

**Runs.**
- G0 verify: [first, failed](#/week/6/run/20261019T075646_286594Z_verify_seed42) failed only `arm_link_masses_match_weld_model`. Link6 was 98.7 g heavier than the CPU model, which
  did not have the camera yet. After the fix, [physical](#/week/6/run/20261019T075759_109111Z_verify_seed42) and [visual](#/week/6/run/20261019T075818_960184Z_verify_seed42) pass.
- Reach `58400` on the frozen validation set: [payload physical](#/week/6/run/20261019T075856_eval_seed1_payload_physical) 1.69 / 2.33 cm, [payload visual](#/week/6/run/20261019T075915_eval_seed1_payload_visual) 1.50 / 1.96, [no lidar drawn](#/week/6/run/20261019T080030_eval_seed1_payload_none) 1.50 / 1.96. No falls (F-147).
- [capture](#/week/6/run/20261019T080359_526678Z_combiner_seed3): the scripted combiner scene, the lidar and cage on the trunk's front ahead of the arm, the camera on the
  gripper.

**A discrepancy found on the way (F-148).** The recorded score for `58400` is 1.40 / 1.93. The Week 4 runs used an arm
URDF hashing `1509b0bc…`, which is not the committed one and is not on this machine. Find it on Victor's machine
before retraining.

**Not done / open.**
- The head D435i in `Go2 v115.stl` (and `Go2 v115_d435i_mount_only.stl`) is not added: it is not on the
  `with_mid360` model, and Lukas asked for the wrist camera.
- Weigh the mount and the bracket.
- The demos run the payload visual-only.
- The CPU torque models (pull planner's ceilings, holdability) do not carry the camera's weight; only G0's check does.
- The lidar body's CAD provenance is not recorded.

### 2026-10-19 · The lidar and its mount, redrawn from a full-resolution export

Lukas, with a viewport screenshot of the trunk's front: "the quality of the mesh is very poor. I have provided
mount_and_mid360_only.stl in my downloads". In the screenshot the cage is a set of jagged flat facets, and the
lidar's dome is faceted.

**What was wrong.** The meshes above came from the diff of two whole-robot exports. That export is coarsely
tessellated, and the diff lost geometry outright:
- the cage's top ring and struts;
- the slotted side walls;
- the plate's holes.

They were not decimated either, since `fast_simplification` is not installed in `env_isaaclab` and the script skips
the step silently. So the 11,642 and 5,728 triangles were the coarse export as it came.

**The new source.** `~/Downloads/mount_and_mid360_only.stl`: 174,238 triangles, metres, the same frame as the
whole-robot export. With the existing translation (−0.0445, 0.0003, 0.1787) m it lands exactly on the mount's
recorded bounds (x 0.0843–0.2403, top 0.1662), and the lidar's housing lands on its own to 0.3 mm.

**Changed.** `description/mid360_from_cad.py`:
- takes this single STL; the two-file diff stays available;
- decimates per connected part, as far as the part stays within 0.3 mm, sampled both ways;
- keeps a part whole where nothing does. Quadric decimation rounds the plate's and cage's edges by 1–10 mm at a
  third of their triangles, so those stay whole, and the screws come down to about a third.

Result: `mid360.stl` 95,240 triangles (of 116,432), `mid360_mount.stl` 34,574 (of 57,824). They are drawn with
per-face normals, so the dome needs that density to look round.

```bash
pip install --target <dir> fast_simplification     # not in env_isaaclab; PYTHONPATH=<dir> for this one step
python description/mid360_from_cad.py ~/Downloads/mount_and_mid360_only.stl
```

![before and after](figures/mid360_mesh_before_after.png)

**Found on the way, not changed.** The collider and clearance boxes in `payload.py` were measured on the broken
mesh, so they miss the side walls. The walls span x 0.135–0.240, y ±0.040, z 0.094–0.118 m, against the cage box's
y ±0.0327. `tests/test_payload.py` now fails on exactly this: 1149 mount vertices outside both boxes.

The mount's volume has the same problem. Its closed parts come to 141.9 cm³, against the recorded 124.9; that
includes the lidar's 1.9 cm³ connector, and the walls are an open shell, so they add an unknown amount.

Left for Lukas, because the boxes and mass feed `--payload physical` training and the arm-clearance check.

**Not checked:**
- a render in Isaac Sim; the comparison is a CPU render of the two meshes;
- the memory cost of 130k triangles per robot copy at thousands of environments.

### 2026-10-19 · Why the arm folds during the door pull's walk

Lukas sent two viewer screenshots of the door pull (environment index 1): the arm in a forward line at the grip, then
folded with the elbow up, late in the walk. His question: is that UniFP, or how we use it?

**Changed.** `demos/unifp/run_demo.py`: door traces now carry the goal as the policy receives it (`goal_radius_m`,
`goal_pitch_deg`, `goal_yaw_deg`) and the six arm joints (`arm_q0_deg`…`arm_q5_deg`).

**Runs.** Seed 1, the recorded F-146 command, 16 environments:

```bash
./demos/unifp/run_demo.py --task door --wrist --attempts 16 --headless --run_name door16_armtrace
./demos/unifp/run_demo.py --task door --attempts 16 --headless --run_name door16_nowrist_armtrace
```

- [With `--wrist`](#/week/6/run/20261019T085418_door_unifp_wrist_seed1_door16_armtrace): 8/16, the same as the
  recorded run.
  - **Successes:** none of the 8 fold. J1 stays 32–50° and J2 −22 to +3° through the 90° walk, tracking within
    0.2–2.1 cm.
  - **Goal:** radius 0.57–0.62 m at the grip, at or past the trained 0.58, and 0.50–0.58 during the walk. Pitch is
    20–26°, yaw −1 to −17°.
  - **Folding:** only in attempts that lost the handle (1, 3; 4 transiently). Attempt 1, the viewer's environment:
    - the fingers hold 2–13 N to 11.9 s, the door reaches 20° and then drifts shut, and the forearm touches at
      11.4–11.7 s;
    - the script, which never reads the door, keeps walking and commanding the handle's arc, and the tracking error
      jumps to 12–22 cm;
    - J3 swings from ~20° to ~110° and J2 from −10° to −65…−88°, and the wrist servo pins J5 at its −134.6° limit;
    - the arm stays folded to the end, 5–10 cm off a goal (r 0.56, p 22°, y −12°) that the successful attempts
      track to ~1 cm extended.
- [UniFP alone](#/week/6/run/20261019T085634_door_unifp_seed1_door16_nowrist_armtrace): 4/16.
  - The arm folds (J2 < −40°) during the walk in 13 of 16 attempts, including while still holding the handle.
  - It is folded at the end in 5, one of them a success: attempt 2, door at 90°, J2 −59°, J3 107°.
  - No arm joint is at a limit.

**Reading.** The folding is the policy's, and the wrist servo suppresses it while the grip holds.
- UniFP commands the tip position only. The orientation channels exist and are unused (`CMD_EE_ORN_*`), so the arm's
  other three degrees of freedom are whatever the policy does.
- That choice depends on history (F-149 found the same), and nothing in the reward prefers one configuration.
- Walking and the door's pull disturb it, and once it is in the folded branch it tracks within a few cm and stays.

Our use adds to it:
- the friction grip loses the handle (F-146), which is the trigger in every `--wrist` fold;
- the script carries on after the loss;
- the goal sits at the trained radius's edge;
- under `--wrist`, J5 is not the policy's, and in the folded branch the servo drives it to its limit.

What this does not show: one seed, 16 placements per condition, simulation. Which configuration a given attempt
falls into is not predictable from these numbers. The upper arm carries no contact sensor, so what the arm touches
while the door drifts shut is not known.

Finding: [F-150](../findings.md).

### 2026-10-19 · A non-UniFP baseline for the door pull: a walking policy and arm IK

Lukas: "Can you get a non-unifp method to open this door? Including moving the robot. I want to establish a baseline."

**What was built.** `--controller decoupled` for the door pull (`demos/unifp/decoupled.py`, `decoupled_arm.py`). The
controller is the only thing that changes. The scene, the 16 seed-1 placements, the script (`script.door_phases`), the
walk round the hinge (`base_velocity_command`, same gains and clips), the unlocked latch, the jaws' drive and
commands, the trace and the ≥ 85° success all come from `UniFPDemoEnv`.
- **Legs:** the walking checkpoint `sim.py` plays back, `logs/rsl_rl/unitree_go2_rough/2024-04-06_02-37-07/model_7850.pt`.
  - Provenance: dated 2024-04-06, it is the go2_omniverse rough-terrain policy, trained on a bare Go2 and converted
    from the 2024 rsl_rl format. It is not a Rescue arm-on run.
  - It gets its own 235-wide observation, rebuilt term for term (`flat_env_cfg.ObservationsCfg`), and its own
    DCMotor legs.
- **Arm:** IK on Link6 and the jaw centre (`d1_ik`, the scripted grip's convention), re-solved every 10 Hz command in
  the base's current frame. Commands go through the D1 firmware planner (F-045) into the stiff servo drive with its
  published limits.
  - The hand orientation is `pull.Handle`'s, matched to 1e-9 in `tests/test_decoupled.py`.
  - The grasp is chosen once per attempt, nearest-level first.
- **Jaws:** UniFP's drive, so the grip is the same.

**Getting the port right took four fixes; every run of the way is recorded.**
1. [Smoke, pitch-40 grasp](#/week/6/run/20261019T100636_door_decoupled_walk_ik_seed1_decoupled_smoke): 0/2, 1 fell.
   - The scripted grip's favourite grasp comes in 40° from above, but the door script enters along the level
     approach, so the fingers swept into the bar at 9–20 N before arriving.
   - That shoved the dog 7–10 cm back and left a 4.6 cm miss at arrival.
   - Fix: `GRASP_ORDER` nearest level first. Every placement now takes pitch 0.
2. [Smoke, level grasp](#/week/6/run/20261019T100907_door_decoupled_walk_ik_seed1_decoupled_smoke_level): 0/2, both
   fell.
   - The grasp was fine: arrival within 3 mm and 6–18 N on the handle, with the door at 55–82° when they fell.
   - [No box](#/week/6/run/20261019T101020_door_decoupled_walk_ik_no_object_seed1_decoupled_noobject),
     [no box, arm parked](#/week/6/run/20261019T101156_door_decoupled_walk_ik_no_object_seed1_decoupled_noobject_parked)
     and [no plinth either](#/week/6/run/20261019T101533_door_decoupled_walk_ik_no_object_free_seed1_decoupled_freespace_parked)
     all stagger the same way (1 of 2 fell in each; the last two identical to the digit).
   - So the walk itself failed, not the door.
3. [Native playback](#/week/6/run/20261019T101351_895806Z_playback_arm_-0.2_-0.3_+0.2) of the same checkpoint at the
   path's hardest command, (−0.2, −0.33, 0.2) for 6 s:
   - tracks (−0.181, −0.320, 0.234) with 7.6° tilt
     ([first try](#/week/6/run/20261019T101319_652041Z_playback_arm_-0.2_-0.3_+0.2) 7.0°; its console output was lost
     on the reach console's port, so the second was run with `--ui_feed_port 0`);
   - the port, given the same constant command (a scratchpad probe, not a run directory), tracked (−0.07, −0.12, 0.12)
     and tilted 32°.
   - Dumping the observations term by term showed identical layouts, and the height scan confirmed the sensor
     convention (−0.0855 = base 0.4145 − 0.5). Two physical differences remained:
     - **self-collision:** on (UniFP's), where the policy trained with it off. Off,
       [free space, arm parked](#/week/6/run/20261019T101630_door_decoupled_walk_ik_no_object_free_seed1_decoupled_freespace_parked_noselfcoll)
       had 0 of 2 fall, and the probe's tilt went 32° → 19°;
     - **foot friction:** UniFP's 1.0/1.0 against the trained 0.8/0.6. At 0.8/0.6 the probe tracked
       (−0.16, −0.24, 0.23).
4. Both fixed in `decoupled.py`:
   - self-collision off, with every arm command checked against the trunk proxy (none broke it in the runs below);
   - the feet alone at 0.6/0.2, which average with UniFP's 1.0 ground to the trained 0.8/0.6, so the fingers keep
     UniFP's friction.

   Probe after the fixes: (−0.166, −0.267, 0.234), 12.4° tilt. Native with the arm at zero (probe): (−0.184, −0.355,
   0.204), 10.4°. The remaining 25% of sideways speed is attributed, not tested, to the arm's drive: the force-driven
   D1 model here, the import's acceleration drives in playback (F-010). The robot configs otherwise differ only in
   spawn height.

**The baseline.** [16 placements, seed 1](#/week/6/run/20261019T102047_door_decoupled_walk_ik_seed1_door16_decoupled):

```bash
./demos/unifp/run_demo.py --task door --controller decoupled --attempts 16 --headless --run_name door16_decoupled
```

| seed 1, same 16 placements | doors ≥ 85° at end of hold | falls | arrive error | base path error (max, median) |
| --- | --- | --- | --- | --- |
| UniFP + wrist servo ([F-146](#/week/6/run/20261019T024909_door_unifp_wrist_seed1_door16), [re-run](#/week/6/run/20261019T085418_door_unifp_wrist_seed1_door16_armtrace)) | 8 | 0 | 5–6 mm | 3–11 cm |
| UniFP alone ([run](#/week/6/run/20261019T085634_door_unifp_seed1_door16_nowrist_armtrace)) | 4 | 0 | | |
| **decoupled: walking policy + arm IK** | **7** | **0** | **2.7–3.5 mm** | **0.27–0.72 m (0.39)** |

- The arm half is better than UniFP's: the jaw centre lands within 3 mm, and every attempt takes the level grasp.
- The leg half is much worse. The base trails its path by 0.39 m at worst (median attempt) and rolls up to 23°.
  [Walk-only control](#/week/6/run/20261019T103049_door_decoupled_walk_ik_no_object_free_seed1_door16_decoupled_walk_only)
  (free space, arm parked, same 16): median path error during the pull 0.146 m (0.19 m with the door), roll 26°,
  pitch 16°, no falls. So the poor walk is the walking policy's on this path, not the door's load.
- **How it fails:**
  - 14 of 16 doors pass 66°, and in 6 of the 9 failures the door was past 79° when the grip let go; it then swung
    back (to 0–58°).
  - The grip goes because the base lags out of the arm's reach: through the pull the IK finds no solution near the
    target 11–59% of the time and holds its last joints.
  - Neither half knows about the other; that is what makes it the decoupled baseline.
- Successes include doors pulled to their 110° stop. The criterion counts them (≥ 85° at the end of the hold), as it
  does for UniFP.

**What this does not show.** One seed, 16 placements, simulation, the box's pose given.
- **Not the same plant:** the legs run on DCMotor with self-collision off and the trained foot friction; the arm is
  the measured D1, which is slower than UniFP's PD (F-097). Each controller runs on the conditions it was built for.
  A same-plant comparison would need UniFP on these settings, which is not done.
- **Not a tuned decoupled stack:** a walking policy trained with the arm, or slowing the walk when the arm saturates,
  would likely do better. This is the naive baseline.
- The scratchpad probes (`port_const.py`, `native_const.py`, `cfg_diff.py`) are diagnostics with no run directory;
  their numbers are quoted above.

Finding: [F-151](../findings.md).

### 2026-10-19 · The door pull hitches: the baseline's IK search

Lukas: "the simulation hitches every half a second or so while pulling the door. my cpu is quite strong so I worry for
the jetson orin on the dog when I deploy this".

**Cause.** The decoupled baseline's arm IK (`decoupled_arm.ArmIK.solve`). On a miss, every 10 Hz command re-ran the
scripted grip's seeded search, which cost 486 ms; a warm start costs 0.2 ms, the failing warm solve 12 ms. The pull
misses 43–49% of its commands, because the walk lags the handle out of reach. The stall is in the simulator's single
loop, so the viewer froze.

**Fixed.** The loop now uses only a warm start capped at 30 iterations. A pose within 2 mm and 0.02 rad is accepted;
otherwise it solves the jaw centre alone. Worst case 7.3 ms a command, and a test pins it under 50 ms. `trace.csv` now
records each step's wall time (`step_wall_ms`).
- [One environment, final loop](#/week/6/run/20261019T113038_door_decoupled_walk_ik_seed1_timing_decoupled_final_1env):
  median 18 ms, p99 31, max 36, none over 50 ms.
- [UniFP + wrist, one environment](#/week/6/run/20261019T104640_door_unifp_wrist_seed1_timing_unifp_wrist_1env):
  median 15 ms, max 21. (An [earlier timing run](#/week/6/run/20261019T104548_door_decoupled_walk_ik_seed1_timing_decoupled_1env)
  of a bounded variant measured the same.)
- CPU cost of the controllers alone, one thread on this PC: UniFP 0.11 ms per 20 ms step, the walking policy 0.03 ms
  (a scratchpad timing, no run directory). Nothing was measured on the Orin.

**But the bound costs doors.** Every run below is 16 placements:

| arm IK loop | seed 1 | seed 2 |
| --- | --- | --- |
| [original](#/week/6/run/20261019T104918_door_decoupled_walk_ik_seed1_door16_decoupled_seededik_s1) / [s2](#/week/6/run/20261019T105837_door_decoupled_walk_ik_seed2_door16_decoupled_seededik_s2): seeded search on every miss (`--seeded_ik_loop`) | 7 (= F-151, attempt for attempt) | 8 |
| [30 its + standoff warm start](#/week/6/run/20261019T104734_door_decoupled_walk_ik_seed1_door16_decoupled_boundedik) / [s2](#/week/6/run/20261019T110930_door_decoupled_walk_ik_seed2_door16_decoupled_boundedik_s2) | 4 | 4 |
| [30 its](#/week/6/run/20261019T111059_door_decoupled_walk_ik_seed1_door16_decoupled_bounded_nostandoffseed_s1) / [s2](#/week/6/run/20261019T111204_door_decoupled_walk_ik_seed2_door16_decoupled_bounded_nostandoffseed_s2), **the final loop** ([reproduced](#/week/6/run/20261019T112933_door_decoupled_walk_ik_seed1_door16_decoupled_final_s1)) | **3** | **6** |
| [100 its](#/week/6/run/20261019T111329_door_decoupled_walk_ik_seed1_door16_decoupled_bounded100_nostandoffseed_s1) / [s2](#/week/6/run/20261019T111448_door_decoupled_walk_ik_seed2_door16_decoupled_bounded100_nostandoffseed_s2) | 3 | 7 |
| [one seed a command, 100 its](#/week/6/run/20261019T111651_door_decoupled_walk_ik_seed1_door16_decoupled_amortizedik_s1) / [s2](#/week/6/run/20261019T111809_door_decoupled_walk_ik_seed2_door16_decoupled_amortizedik_s2) | 3 | 7 |
| [full search once, at the first oriented command](#/week/6/run/20261019T112013_door_decoupled_walk_ik_seed1_door16_decoupled_plannedik_s1) / [s2](#/week/6/run/20261019T112131_door_decoupled_walk_ik_seed2_door16_decoupled_plannedik_s2) | 3 | 7 |
| [current q + one seed, 300 its](#/week/6/run/20261019T112336_door_decoupled_walk_ik_seed1_door16_decoupled_rescueik_s1) / [s2](#/week/6/run/20261019T112556_door_decoupled_walk_ik_seed2_door16_decoupled_rescueik_s2) | 3 | 7 |

- **Where they part:** the original rescues once an attempt, around 13.8 s as the walk starts to lag. Its search
  finds the pose on another branch, where the bounded loops fall back to the jaw centre; the first >1° divergence
  in joint angles is at that command.
- **The bounded variants:** none of them recovered it, and on seed 1 all gave the same attempts. Their code was
  removed, and two of the rows above were run through temporary environment-variable switches; the runs and their
  `run.json` arguments are the record.
- **What a robot would do:** run the search in a planner thread beside the 10 Hz loop. Not built.
- **Not explained:** which seed or branch does the rescuing.

Finding: [F-152](../findings.md), which supersedes F-151's headline. The baseline is now the bounded loop: 9 of 32
on two seeds, against UniFP + wrist's 15 of 32.

### 2026-10-19 · The UniFP demos' viewer: one scene, no goal markers

Lukas asked for the blue and pink spheres off the hand, and for one scene rather than 16, which ran slowly.
`run_demo.py` changes:
- with the viewer it now defaults to `--num_envs 1`, with further attempts run one after another;
- the goal overlay is off unless `--markers` is given.

Headless runs keep `--num_envs` = `--attempts`, so every recorded rate stays reproducible as it was measured. A
viewer run is a different configuration under F-098, so its outcomes are not those of the 16-environment runs. No
run was made for this change.

### 2026-10-20 · The combiner door demo: head camera, MID-360, P2Dingo's walking policy

Lukas asked for the non-UniFP door swing (grip the handle above the tag) with the RealSense on the dog's head, P2Dingo's
mount and offset, as a new demo, `combiner_door`. During the work he added:
- "the mid360 will also be involved - it can verify the angle of the door by analysing the points near the apriltag";
- that P2Dingo's docking "just uses the position of the apriltag and then the surrounding lidar points for
  orientation";
- "can you try using p2dingo's walking policy? I remember it being stable".

**Built.** `demos/combiner_door/` (README there), run through the door pull's runner (`run_demo.py --perception head`):
- **Head camera:** the RealSense on P2Dingo's head mount, the CAD-measured `base_link` (0.3392, 0.03215, 0.0927),
  straight ahead, with P2Dingo's bracket and Intel's case drawn. P2Dingo's committed mount is an older guess; the
  measured one and the bracket are in its working tree (`third_party/p2dingo/NOTICE.md`).
- **MID-360:** a `MultiMeshRayCaster` at the point-cloud origin of this robot's lidar mount, (0.1758, −0.0002, 0.1431).
- **The look:** the tag's centre for the position, the door's face in the lidar for the heading (P2Dingo's
  `dock_wall` fit, ported to numpy).
- **Door verification:** the same fit, run at 10 Hz while the door opens.
- **Walking policy:** P2Dingo's, copied in with its gait clock (`--walker`, now the decoupled default).
- **Tests:** 12 CPU tests (`demos/combiner_door/tests`), 8 for the arm (`demos/unifp/tests/test_decoupled.py`).

**What it took** (all runs recorded):
1. [First smoke](#/week/6/run/20261020T043321_door_combiner_door_seed1_combiner_door_smoke): look found the box to 4–10 mm,
   but the arm swung from its parked zero pose into the handle (21 N), shoving the dog 26 cm, and the lidar read the
   shut door at ~59°. Fixed:
   - the arm holds the home point relative to the base until the task;
   - the lidar estimate became P2Dingo's fit.
2. [Walking in](#/week/6/run/20261020T043554_door_combiner_door_seed1_combiner_door_smoke2) on the 2024 walking policy:
   - it pitched 22–33° nose-down and the stowed hand hit the handle.
   - After even a short walk, it turned in place ~5°/s and leaned 26°:
     [0.12 m](#/week/6/run/20261020T043919_door_combiner_door_seed1_cd_walk_lb12),
     [still after](#/week/6/run/20261020T044227_door_combiner_door_seed1_cd_walk_lb12_still),
     [heading held](#/week/6/run/20261020T044848_door_combiner_door_seed1_cd_lb12_heading),
     [one env](#/week/6/run/20261020T045302_door_combiner_door_seed1_cd_lb12_1env),
     [0.30 m slow](#/week/6/run/20261020T044016_door_combiner_door_seed1_cd_walk_lb30slow),
     [0.30 m heading held](#/week/6/run/20261020T044946_door_combiner_door_seed1_cd_lb30_heading).
   - Ablations ruled out the camera's visuals, the lidar and the camera:
     [no visuals](#/week/6/run/20261020T044630_door_combiner_door_seed1_cd_ablate_novis),
     [lidar on the box only](#/week/6/run/20261020T044718_door_combiner_door_seed1_cd_ablate_lidarbox),
     [no camera](#/week/6/run/20261020T045116_door_combiner_door_seed1_cd_ablate_nocam),
     [no lidar](#/week/6/run/20261020T045157_door_combiner_door_seed1_cd_ablate_nolidar).
   - [Not walking](#/week/6/run/20261020T044324_door_combiner_door_seed1_cd_walk_lb0) stood level.
   - Replaying the commands in a probe of the port (scratchpad): stopped from 0.06 m/s it stands; from 0.08, 0.10 or
     a ramp from 0.10 it turns and leans.
   - Native playback, [0.15 m/s forward](#/week/6/run/20261020T043834_835013Z_playback_arm_+0.1_+0.0_+0.0), tilted 20.6°, and
     leaned 9–14° after stopping from 0.06 (probe).
3. [The P2Dingo-style look](#/week/6/run/20261020T050047_door_combiner_door_seed1_cd_p2dingo_4): box to 1.2–1.7 mm and
   ≤ 0.31° (the tag alone, up to 10.3 mm and 1.1°). Still leaning.
   [A steady 0.06 m/s walk](#/week/6/run/20261020T050411_door_combiner_door_seed1_cd_steady_4) stood level but stopped 8–12 cm
   short, and the pull, catching up, flung the door off the hand. The lidar tracked it throughout (25.1/25.3,
   69.2/69.6, 110.0/110.3°).
4. P2Dingo's walking policy, probed: it stops from 0.15–0.30 m/s within 1.0–1.7° of level, and turns ~15° settling
   after the spawn drop, then holds. So:
   - the attempt is anchored where the robot stands after a 4 s settle;
   - it walks straight in at 0.15 m/s until odometry says it has arrived;
   - [4 attempts](#/week/6/run/20261020T051132_door_combiner_door_seed1_cd_p2dwalker_4): 4/4.

**The measurement**, the door pull's 16 placements, seeds 1 and 2:

```bash
./demos/combiner_door/run_combiner_door.py --attempts 16 --headless --seed 1      # and --seed 2
./demos/unifp/run_demo.py --task door --controller decoupled --walker p2dingo_gait --attempts 16 --headless --seed 1
```

| same placements | seed 1 | seed 2 | falls |
| --- | --- | --- | --- |
| **combiner door demo** ([s1](#/week/6/run/20261020T051305_door_combiner_door_seed1_combiner_door16_s1), [s2](#/week/6/run/20261020T051658_door_combiner_door_seed2_combiner_door16_s2)) | **15/16** | **16/16** | 0 |
| given pose, P2Dingo walker ([s1](#/week/6/run/20261020T051554_door_decoupled_walk_ik_seed1_door16_decoupled_p2dwalker_s1), [s2](#/week/6/run/20261020T051945_door_decoupled_walk_ik_seed2_door16_decoupled_p2dwalker_s2)) | 16/16 | 16/16 | 0 |
| given pose, 2024 walker (F-152) | 3/16 | 6/16 | 0 |
| UniFP + wrist (F-146) | 8/16 | 7/16 | 0 |

- **Look:** tag found 32/32. The box placed a median 2.2–2.4 mm off (max 7.4) and 0.14–0.20° off (max 0.34); the tag's
  own heading would have given max 0.50° and 8.8 mm.
- **Lidar on the door:** a reading on every grip/pull/hold frame, median error 0.46–0.49°, p90 ≤ 2.9°. Verified open
  agrees with the simulator 32/32. The one failure (door 74.5°) was read at 74.4° and not verified.
- **Arrival:** 5.9–6.0 mm.

**What this does not show.**
- **Sensors:** an ideal render, a grid lidar with 3 cm noise and no motion distortion, and the simulator's odometry.
- **The lidar's heading:** at 0.37 m in an ideal render the tag's own heading is already within 0.5°, so the gain is
  small here; P2Dingo's ~10° tag heading at a metre on the robot is where it counts.
- **Walking policy:** neither walking policy carried the arm in training.
- **Placements:** the door pull's.

The baseline's default walker is now P2Dingo's; `--walker rescue_2024` reproduces F-151 and F-152:
[re-run on seed 1 after the refactor](#/week/6/run/20261020T052129_door_decoupled_walk_ik_seed1_door16_decoupled_rescue_regression_s1), 3/16, the same attempts and door angles as F-152's.

Findings: [F-153](../findings.md), [F-154](../findings.md); F-152 superseded as the baseline's number.

Lukas asked for a script to run it as a program: `demos/combiner_door/run_combiner_door.sh`. It sets up the Isaac
environment, warns if something is already on the GPU, and forwards its arguments.
[Checked from another directory](#/week/6/run/20261020T053227_door_combiner_door_seed1_combiner_door_sh_check): one
attempt, door opened.

### 2026-10-20 · The arm starts folded, and the handle as measured

Lukas asked for the arm to begin folded in its resting position. He then measured the combiner box's handle: 14 cm
long, 4 cm tall and 1.7 cm wide, its front profile a slot with semicircular ends, and 2.7 cm clear of the door
("it looks a bit further out in simulation": it was 5.65 cm).

**Folded start** (`demos/combiner_door/env.py`, `DecoupledDoorEnv.joint_phases`).
- The arm is set to the real rest pose at every reset (`d1_hardware.REST_SERVO_DEG`, new: the F-023/F-027 pose,
  clamped inside the URDF's limits) and parked there through the landing and the look.
- After the look, an `unfold` phase moves it to the home point's IK solution: a straight line in joint space, each
  10 Hz command at most 4 deg from the measured pose, as `d1_hardware`'s approach steps the real arm. An attempt that
  finds no tag never unfolds.
- On the CPU model the rest → home path is clear of the body proxy throughout (`d1_ik.path_clearance`), as is rest →
  zero → home.
- [4-attempt smoke](#/week/6/run/20261020T060825_door_combiner_door_seed1_cd_folded_smoke): J1 from -89.7 to +8.2 deg
  in 3 s, the base tilting under 0.5 deg, no contact.

**The handle** (`demos/combiner/geometry.py`, `asset.py`).
- **The shape.** The lever is now the measured slot: a 17 mm thick plate, 40 mm tall, 140 mm long, drawn and collided
  as a box between two end cylinders, turning about its near end's centre (assumed).
- **The gap.** `handle_gap` 0.027 is measured; `handle_projection` (the centre plane, 35.5 mm) now follows from it.
- **The jaws.** They open to 63.2 mm onto it (`PullParams.jaw_open_m` 0.012 → 0.023; 11.6 mm clear above and below,
  as the bar had). They cannot close across its 17 mm thickness: shut, the CAD pads stand 17.2 mm apart.
- **The placements** move 29.5 mm closer, so the handle keeps the distance from the robot the demos were tuned at:
  - `props.BOX_DISTANCE_RANGE_M` now follows the handle (0.6705-0.7105 m, from `HANDLE_DISTANCE_RANGE_M` 0.477-0.517).
  - The turning demo's `DEFAULT_DISTANCE_M` likewise, except that its near end stays at the 0.60 m `validate_region`
    allows.
  - Without the move, UniFP's door and combiner goals left its trained sphere (`test_unifp_demos`).
- **The look** moves back as far (`LOOK_BACK_M` 0.15 → 0.18), so the head camera sees the box from where it did.
  Without it, 5 of 32 looks lost the tag.
- **The press** method's fingertips now run 15 mm past the lever's centreline, not 25 mm: 20.5 mm short of the door
  (it was 10.5).
- **Tests.**
  - The clearance checks use the slot, not the bar.
  - Two records of what changed: the wrist camera's clearance at the turning demo's 60-80 deg grasps (now 0.05-19 mm:
    F-157), and the close-look test skipping 2 of 40 nearer placements where no search stop sees the tag past the
    fingers.

**A stale box, found and fixed.** The door runner cached the box's USD by the handle's torque alone
(`generated/unifp_demos/combiner_0.400.usda`, built 2026-10-19), so it outlived every geometry change today.
- [This demo](#/week/6/run/20261020T061030_door_combiner_door_seed1_cd16_folded_slot_s1) (16/16 twice) and
  [the baseline](#/week/6/run/20261020T061656_door_decoupled_walk_ik_seed1_door16_decoupled_slot_s1) (16/16 twice)
  ran on the old bar: valid for the folded start only. Re-titled.
- With the measured gap, the scripts aimed at a lever 35.5 mm off the door while the simulated one stood at 65 mm.
  The hand stalled 10 mm short with 12-15 N on its wrist links, and the doors opened
  [6/16](#/week/6/run/20261020T070342_door_combiner_door_seed1_cd16_gap27_s1) and 7/16 (the baseline 14/16 twice;
  [the old placements](#/week/6/run/20261020T071448_door_decoupled_walk_ik_seed1_door16_decoupled_gap27_oldplace_s1),
  13/16). Recorded as INVALID.
- The cache is now keyed by the geometry's values and `asset.py`'s code (`run_demo.py`).
- The old cached box is byte-identical to one built from the committed geometry (81b9c4a): F-146 to F-154 ran on the
  box they describe.

**Results on the rebuilt box** (folded start, measured lever, moved placements, the look 0.18 m back):

```bash
./demos/combiner_door/run_combiner_door.sh --attempts 16 --num_envs 16 --headless --seed 1      # and --seed 2
./demos/unifp/run_demo.py --task door --controller decoupled --attempts 16 --num_envs 16 --headless --seed 1
```

| same placements | seed 1 | seed 2 | falls |
| --- | --- | --- | --- |
| **combiner door** ([s1](#/week/6/run/20261020T072044_door_combiner_door_seed1_cd16_gap27b_s1), [s2](#/week/6/run/20261020T072351_door_combiner_door_seed2_cd16_gap27b_s2)) | **16/16** | **16/16** | 0 |
| pose given ([s1](#/week/6/run/20261020T072654_door_decoupled_walk_ik_seed1_door16_decoupled_gap27b_s1), [s2](#/week/6/run/20261020T072754_door_decoupled_walk_ik_seed2_door16_decoupled_gap27b_s2)) | 16/16 | 16/16 | 0 |

- **Look:** tag found 32/32, the box placed a median 2.2-2.3 mm and 0.12 deg off (max 3.0 mm, 0.64 deg).
- **Lidar on the door:** a median 0.33-0.39 deg off (p90 at most 2.1); open/not open agrees 32/32.
- **The grip:**
  - the jaw centre arrives 6.2-6.5 mm from the grasp point (4.9-5.1 given the pose);
  - the fingers squeeze 14 N through the pull, about 10 N on the round bar;
  - the doors end their hold at a median 93 deg;
  - the level grasp every time.
- [4-attempt smoke](#/week/6/run/20261020T071912_door_combiner_door_seed1_cd_gap27b_smoke): 4/4.

**What this does not show.** Simulation as F-154. The lever's pivot, mass and friction are assumptions.

**Consequences elsewhere** (F-157):
- the L-lip claw's lips (20 mm) leave 37.2 mm between their tips fully open: the 40 mm lever cannot pass;
- the turning demo's 60-80 deg grasps put the wrist camera into the door.
- Neither design is changed here.
- `demos/unifp/README.md`'s claw rows describe the earlier 18 mm bar, as the runs they cite did.

Findings: [F-155](../findings.md), [F-157](../findings.md).

### 2026-10-20 · The combiner door on the real dog

Lukas asked to run the demo on the robot (reached with `sshuni`), with an RViz view through the dog's P2Dingo zenoh
bridge (like `p2dingo-dock-view-robot.sh`), the repo cloned on the dog and a bashrc helper to copy changes across.

**How it is put together** (`demos/combiner_door/robot/`, README there).
- **`plan.py`** is the simulated demo's sequence on the real robot's terms, numpy only:
  - the look's fix from the tag and the lidar (`perception`, `mid360`);
  - the stance set from the box, 0.69 m off;
  - the door script, the IK, the lidar's door reading;
  - stages that stop early, back off and fold;
  - sport mode's floors;
  - the aborts.
- **`node.py`** is the ROS 2 node, in P2Dingo's ROS image: the head camera, `/livox/lidar` and `/sportmodestate` in;
  with `--live` only, `/api/sport/request` and the arm out; RViz's topics under `/combiner_door/`.
- **`arm_link.py`** drives the D1 through `d1_hardware.D1Client`, in P2Dingo's arm image (CycloneDDS). It steps the
  arm at most 4 deg from its measured pose, holds on silence or disconnect, never releases, and refuses to start while
  P2Dingo's arm bridge is streaming.
- **Dry runs command nothing:** the plan's ghost walks and reaches in RViz.
- **Live runs wait for Enter** before each step that moves something, and `a` stops at once.
- **On the dog:**
  - `~/D1Training` is a shallow clone of main without results/checkpoints/logs; the repo is private and the dog has no
    GitHub key, so it was cloned here and copied across.
  - New `~/.bashrc` helpers: `d1update` (copy the working tree, as P2Dingo's `update`), `d1dog`, `d1view`,
    `d1fetch`.
- **In P2Dingo** (uncommitted there):
  - the `/combiner_door/` topics added to both zenoh allow-lists, the dog's copy deployed and its zenoh container
    restarted;
  - an `OPC_COMPOSE_EXTRA` hook in `p2dingo-rviz.sh`, so this view mounts this repo into rviz.

**Run on the dog** (all dry; the dog lying in front of P2Dingo's charging dock; `d1dog`, `d1view`):
- **Two start-up crashes:**
  - [a record written too early](#/week/6/run/20261020T063730_watch_dry);
  - [OpenCV without the NVIDIA runtime](#/week/6/run/20261020T063752_watch_dry). The ROS image's OpenCV needs
    `--runtime nvidia`, as P2Dingo's own service has.
- **[Watch](#/week/6/run/20261020T063824_watch_dry):**
  - the camera at 1280x720 (fx 907.1) and the lidar read;
  - the arm link read the folded arm, [-1.4, -89.7, 90.0, 2.7, 11.1, -4.0] deg, gripper 67.1 units;
  - P2Dingo's arm bridge disconnected.
- **[The tag found](#/week/6/run/20261020T063933_watch_dry), and a bug:** the walk was computed from the tag's own pose,
  tilted 5.2 deg, which moved the box's root 4 cm. It is now [computed from the levelled fix](#/week/6/run/20261020T064139_watch_dry).
  The lidar's face fits at 19 mm rms, its heading within 1.4 deg of the tag's.
- **It was the dock's tag.** RViz's camera showed the tag on the black stand of P2Dingo's charging dock (same 36h11 id
  0), not the box.
  - The face 2 cm behind the tag [did not tell them apart](#/week/6/run/20261020T064617_watch_dry).
  - The face carrying on past the door's edges [did](#/week/6/run/20261020T064744_watch_dry): 141% of its points round
    the tag, where the box gives 0.
- **RViz** drew everything; its text needed underscores (Humble draws a space four letters wide; P2Dingo found the
  same).
- **[Dry pull to 30 deg](#/week/6/run/20261020T065323_pull_dry)** against the dock's tag (`--ignore_range`):
  - all 14 phases in order;
  - the ghost walked in to within 8 mm (the gait's 75 mm carry modelled);
  - the IK held the hand's pose throughout, transit steps aside;
  - the base was commanded only at sport mode's floors;
  - the arm ended commanded to the fold;
  - the lidar refit moved the "box" 20 mm, the dock's wall being 2 cm behind its tag.

**What this does and does not show.**
- It shows the robot side plumbed end to end, safely, on the real sensors.
- **Nothing live yet:** no leg or arm command sent, and the box not in view.
- **The walking differs from the simulation.** The real dog walks on sport mode, not on the gait policy the simulated
  demo uses (that policy has only run in Isaac Sim), and the pull's base path is inside sport mode's dead zones, so it
  is walked in bursts. The pull is the step most likely to differ from simulation.

Finding: [F-156](../findings.md).

**First live runs** (Lukas, `d1dog --stage unfold --live`):
- [The first](#/week/6/run/20261020T081401_unfold_live) waited at its first Enter and was quit: nothing moved.
- A [look](#/week/6/run/20261020T081539_look_live) confirmed that Enter reaches the program through SSH and Docker. It
  also showed two bugs:
  - `q` + Enter was swallowed (KeyboardInterrupt raised in a timer callback);
  - a dropped SSH session left the program running on the dog.
- [The second unfold](#/week/6/run/20261020T081928_unfold_live) moved the arm, J1 -89.7 → -79.4 deg and J2 90.0 →
  83.3 deg in 1.3 s.
  - The arm's reports then stopped for 0.5 s while it was being commanded, and the arm link held it.
  - The arm link had streamed a command every 100 ms whatever the reports did; `d1_hardware` paces each command off a
    fresh report (~9 Hz).
  - Lukas's recovery then stalled: it planned a Cartesian back-out from the unfold, through the full grasp search.
- **Changed:**
  - the arm link commands once per fresh report and holds after 1 s without one;
  - the grasp is planned before anything moves;
  - recovery before the reach only folds;
  - the unfold and fold have 8 s;
  - one Enter starts the attempt, which then runs through and exits (`--step` restores the per-step pauses, at
    Lukas's request: "not a fan of all the enters");
  - `q` works, and a live run stops when its terminal closes.
- **Run again** ([unfold](#/week/6/run/20261020T082609_unfold_live), [unfold](#/week/6/run/20261020T082714_unfold_live)):
  the arm unfolded and folded with one Enter, and the program exited by itself.
  - **Too slow.** Each command was 4 deg past where the arm last reported itself, and the arm moves ~1 deg between
    reports, so it crept at ~8 deg/s.
  - **On the clock.** The phases ran on the clock: the unfold reached J1 -17 deg of its +8, and the fold ended at J1
    -67, J2 77 deg, short of -90/90 ("about 10 cm from fully folded", Lukas).
  - **[The walk](#/week/6/run/20261020T082827_walk_live):**
    - It refused the box 0.73 m off: a 4 cm walk is under sport mode's floor, so it asked for 0.16 m back.
    - It crashed on its way out: rclpy's own SIGINT handling had shut the context before the node's StopMove (the
      launcher's StopMove still went).
- **Changed:**
  - **The arm link** moves its command `ARM_STEP_DEG` a report from the last command (~36 deg/s), never more than 10
    deg ahead of the arm.
  - **The node sends goals.**
  - **Joint moves wait** for the arm to arrive within 2 deg (20 s, then abort).
  - **No walk is needed** when the box is 0.65-0.76 m off (the level grasp solves across it on the CPU model; wider
    than the simulated 0.67-0.71 m, which the record notes).
  - **rclpy is started without its signal handling**, so the node's own StopMove goes out on Ctrl-C.
- **With those:**
  - [the unfold](#/week/6/run/20261020T084728_unfold_live) reached home (J1 5.8 deg) and
    [folded](#/week/6/run/20261020T084154_unfold_live) right back to rest (J1 -89.8, J2 90.0 deg);
  - [the walk](#/week/6/run/20261020T084309_walk_live) walked 11 cm of its 18 in 1.7 s of commands at 0.2 m/s: the
    legs answer about a second late and stop dead.
    - It ended 8 cm short and 3.7 deg turned, and aborted on the stance check.
    - Standing to settle, the odometry slid back 1.7 cm.
- Lukas: the dog should walk in classic mode for the best accuracy.
- **Changed:**
  - **ClassicWalk once, as the attempt starts** (it was sent with a BalanceStand in front of the first Move).
  - **A zero Move between walks, StopMove only to stop for good.**
  - **The walk in is closed-loop:** it learns the carry, takes up to three corrective steps of at least 0.3 s, and
    holds its heading.
  - **Afterwards the attempt re-anchors where the dog stopped**, the lidar refit placing the box from there, and checks
    the arm can reach it (0.65-0.76 m), replacing the fixed 8 cm stance tolerance.
  - In a test with a base that answers 0.4 s late and stops dead, one corrective step finishes 1 cm past the stance.
- **[On the dog](#/week/6/run/20261020T085050_walk_live), in classic walk:**
  - The walk in stopped 6.9 cm short, having carried on nothing.
  - One corrective step ended 1.5 cm past the stance, 1.8 cm aside, 2.7 deg turned.
  - The look's lidar fit was the real box's door at 5.7 mm rms (19 mm on the dock's wall).
  - After the walk, the refit moved the box 34.5 mm along its normal.
  - **Bug: the look's tag guard ran again after the walk.** It took that correction for a tag 3 cm off its door and
    aborted. After the walk only the reach is checked now (`Fix.in_range(check_tag=False)`).
- **[The first live pull](#/week/6/run/20261020T090355_pull_live) (Lukas, 30 deg):**
  - **Walk and refit:** the walk in finished 3 mm from the stance after one corrective step; the refit moved the box
    36 mm; it re-anchored at 0.65 m and -9.6 deg (the edge of the accepted band).
  - **Reach and grip:** the arm arrived within 1.3 deg of its goal on every joint, and the jaws closed on the lever.
  - **The pull:**
    - the lidar saw the door go 0 → 16 → 24 → 36 → 40 deg in 3.5 s, and 51 deg in the hold;
    - the base moved in bursts at sport mode's floors, with stops between ("not continuous, spurts of movement",
      Lukas);
    - the arm's joints swung ±5-11 deg about their IK goals, and the door swung back and forth.
  - **The abort:**
    - in the hold, one joint 11 deg from its goal for 0.6 s, so the loaded-arm check fired;
    - Lukas quit, leaving the arm holding the lever.
- **Changed:**
  - **The pull is one continuous twist about the hinge.** It is the base and door turning as one body, constant in the
    base's frame, streamed at the gait's slowest usable speed for the whole pull. The door angle follows the measured
    turn, not the clock. In the dry model the arm's goals change at most 1.4 deg through a 30 deg pull, 4.1 through
    90. The base stands for the hold, the release and the step back.
  - **The arm link streams a smooth profile** at a steady 10 Hz: 60 deg/s^2 up to 30 deg/s, slowing into the goal,
    within 10 deg of the arm. Lukas found the arm jagged: each 4 deg step was run at the arm's top speed with a wait,
    and stalled on late reports.
  - **The unfold turns the hand to the grasp's orientation**, removing a 94 deg wrist turn at the stand.
  - **A missed pose holds the goal** instead of switching wrists. The largest step-to-step goal change through a dry
    pull is now 2.7 deg.
  - **`--release`** opens the jaws where they are and backs the hand straight out (orientation held), then goes home
    and folds; for an arm left holding the lever. Simulated from the pose the arm was left in: 97 mm straight back,
    0 mm sideways, 0 deg turned.
- None of this has run on the dog yet.
- **Lukas: faster to the grasp, less waiting between steps, and a careful ungrasp.** Letting go, the hand went home
  straight from the lever. On the way out the fingers hit the lever's 63 mm hub (the "rotating part", 63 mm across,
  photographed in the session) and pushed the door. The hub is now `GEOMETRY.handle_hub_diameter`, assumed centred on
  the spindle and filling the 27 mm gap behind the lever. The simulated box's asset is unchanged, still with its 24 mm
  spindle and 60 mm rosette. RViz draws the hub.
  - **Why it hit (CPU model):**
    - The fingertips stand 20 mm past the jaw centre, in the gap behind the lever.
    - After the pull, the door is turned 30 deg relative to the base, and the straight line home runs 11 cm along the
      door towards the hub for every 5 cm out from it.
    - In a dry pull with the lever and hub modelled, that path ran the fingers 8.2 mm into the lever and 0.7 mm into
      the hub.
  - **The ungrasp** (`plan.ungrasp_phases`, the same at the end of every stage, in a recovery and in `--release`):
    - The hand is held at the pose the arm measures itself at, the jaws open, and the hand waits for servo 6 to report
      them open.
    - It then backs straight off along its own approach by 12 cm at the enter's pace, orientation held, then goes home.
    - On the same model: 11.5 mm from the lever and 38 mm from the hub throughout, home starting 88 mm off the lever.
    - It aborts, the hand where it is, if the jaws are not reported open 2 s after the release's clock, or the IK cannot
      hold the hand's orientation for 0.5 s; it never falls back to another wrist round the lever.
  - **Bug found by its test:** a phase held past its clock froze one step short of its end. A release waiting for the
    jaws therefore never commanded them fully open; on the model they stalled at 25 units and passed as open. A held
    phase now holds at its end point.
  - **Faster.** In the live pull, from the start of the unfold to the start of the pull took 27 s; 7 s of that was the
    stand, the settle and the arrival waiting out their clocks with the arm already there.
    - **Phases that only wait for the arm end when it is there:** the stand, the settle and the arrival, the unfold and
      the fold. That is every joint within 1 deg of its goal (the fold 0.5) on a report under 0.25 s old, after at least
      0.3 s; otherwise the phase runs its clock as before. The arm's steady-state error is 0.2 deg (F-033).
    - **The other changes:**
      - the door script at the sim's own pace (`--speed_scale` 1.0, was 1.5);
      - the unfold and the fold at 40 deg/s in joint space (30);
      - the walk in ending when its steps do, not on a 5 s clock;
      - the settle after it 1 s (2 s; the walk in already ends with 1 s stood, and the refit's 10 lidar frames give
        thousands of points against the 40 it needs);
      - the unfold stage's pause and the grip stage's squeeze 1 s each (2 s).
    - **The grip waits for the jaws to stop closing** (up to 2 s) before the pull.
  - **The dry model.**
    - It runs the arm link's profile into a servo that follows as mode 0 does, ~70 deg/s and settling in about half a
      second (`arm_link.SimulatedArm`).
    - Its gripper moves at an assumed 70 units/s and stalls at 6.8 units on the lever.
    - On it, from Enter to the start of the pull takes 12.6 s, the look excluded: unfold 3.1, walk in 2.0, settle 1.0,
      stand 0.4, approach 2.0, settle 0.4, enter 2.5, arrival 0.4, grip 0.8.
    - The pull's end now takes 5 s, against 3.5 s before: release 1.0, ungrasp 2.5, home 1.5.
    - The node's dry run uses the same model.
  - `--step` now also waits before a phase that begins early.
  - Tests: `demos/combiner_door/tests/test_robot.py` 32 OK (5 new: the clearance against the old path, the dwells, an
    arm that does not get there, the jaws, `--release` straight back within 4 mm).
  - **Scope:** CPU model and one live observation (Lukas's). The gripper's speed is assumed; the ungrasp, the dwells
    and the new pace have not run on the dog. The sim's door script still goes home straight from the lever.
- **Lukas: the pull to 60 deg by default, with a shortcut.**
  - `--door_deg` now defaults to 60 (was 30). `d1dogdoor` in `~/.bashrc` is `d1dog --stage pull --door_deg 60`: still a
    dry run unless given `--live`.
  - **In the dry model at 60 deg:** the IK held the hand's pose at every step of the pull and the hold, and the twist
    turned the base 60 deg in 2.9 s. The dog ends about 0.7 m to the side, turned 60 deg. Letting go stayed 11.5 mm off
    the lever and 38 mm off the hub (90 deg the same).
  - **The hold follows the base's turn.** When the twist stops, the gait carries the base on, but the hold kept the
    door at the pull's angle. So the hand's goal swung away from the arm by 6.6 deg at 30 deg and 8.9 deg at 60 deg,
    near the loaded-arm check's 10. The hold's door now follows how far the base has actually turned (up to 10 deg
    past the pull; the door is free and the hand holds it): at most 1.3 deg at 30, 2.9 at 60, 4.5 at 90.
  - Tests: 33 OK (`test_the_arm_stays_on_its_goal_through_a_60_deg_pull_and_hold`). Not run on the dog.
### 2026-10-19/20 · The front-right foot as the end effector: a door handle at body height

Lukas's request (2026-10-19, evening): a simulation test of the Go2's front-right leg turning a lever door handle at
the Go2's body height, no door opening needed, trained position-only and force-aware as UniFP is, with a day of GPU.
Context he gave: the leg is far stronger than the D1, and the arm-mounted RealSense can be pointed at what the foot
is doing. **An exploratory side study, not a P0-P4 comparison on a declared task; nothing here touches G1-G7.**

**Built** (`unifp_train/README.md`, "The front-right-leg task"): `--task leg` in `run_unifp_train.py`. UniFP's
environment with the FR foot sphere's centre as the controlled point, a goal walk about the FR thigh joint in UniFP's
goal channels (`foot_goals.py`), 12 actions (the legs), the D1 driven to a held look pose, UniFP's two push channels on
the foot (+-35 N a side, 400 N/m virtual spring), the plan's P0/P1 split as `--leg_variant position|force`. A lever
handle plant (`handle.py`: a capsule on a sprung spindle with rest and end stops, a latch angle, the foot sphere's
penalty contact, the door's face, 8 sub-steps per physics step) and a task layer (a reference onto the bar and round
its arc past the estimated stop, hold 2 s, lift off; a PI force along the arc for the force-aware variant, capped at
1.8x the rated force). 144 frozen handles (`leg_eval.py`). `tests/test_handle.py`: 14 tests pass, CPU only. The
evaluation loader now takes the action count from the checkpoint (`unifp_train/eval.py`), so a 12-action policy loads.

**The arm's look pose.** A first pose from a CPU search that assumed a 0.30 m base put every handle above the top of
the image in Isaac Lab: the zero-action stance is 0.255-0.276 m. A second search against the handles this task draws
gave `(-0.30, -0.20, -0.60, 0, 1.30, 0)`: checked in Isaac Lab on 8 drawn handles after a 2 s settle, joints within
0.025 rad, Joint2 / Joint3 holding 0.66 / 0.95 N·m (20% / 29% of 3.33), the camera 0.89 m up and 0.6-0.77 m from the
handle, every handle point inside a 640 x 480 image (u 170-394, v 105-249). That is the CAD mount (F-138) on the
simulated arm; nothing was rendered, and the real bracket is not measured.

**Smoke** ([run](#/week/6/run/20261019T125232_smoke_seed1_leg_smoke)):
`./run_unifp_train.py smoke --task leg --leg_variant force --handle_fraction 0.5 --num_envs 16 --steps 300 --headless`:
shapes as expected (actor 32 x 76, critic 3 x 153, 12 estimates), every reward term finite, 6 resets in 300 steps (the
handle episodes truncating once their approach is missed, as zero actions must).

**Two runs that did not learn the task** (both the force-aware variant, seed 1, 4096 envs, 2.6 s an iteration):

| run | change | stopped at | what it learnt |
| --- | --- | --- | --- |
| [v1](#/week/6/run/20261019T125605_train_seed1_leg_force_s1) | as built, pushes from 300 | 328 | stands on three legs from iteration ~35 (episodes 799 of 800 steps); UniFP's foot term flat at ~1.0 of 2.0 from 80 to 328, foot ~0.2 m off its goal |
| [v2](#/week/6/run/20261019T131141_train_seed1_leg_force_v2_s1) | a 12 cm tracking term (weight 3) beside UniFP's (lowered to 1); goals' leftward yaw cut to +0.25; pushes from 800 | 290 | the same: the 12 cm term ~0.6 of 3 |

Why, from playing v2's `model_200` on 8 environments (a scratch diagnostic, not recorded): **the foot never left the
floor.** Its sphere's centre sat at 0.022 m, on the ground, with under 5 N on it while the base leant back ~15°. The
"lifted" half of `support_contact` was "under the 5 N contact threshold", so the policy was paid for unloading the
foot rather than lifting it. Separately, a kinematic sample of the FR leg with the base level puts 85% of the drawn
free-space goals within 3 cm of reach (median 4 mm); past +0.2 rad of yaw to the left, 39% are not, hence v2's cut.

**v3: the foot learns to track.** "Clear of the ground" became geometric (the sphere's centre above 5 cm and under
1 N), and a new term pays the foot's height toward its goal's (weight 1.5).
[v3](#/week/6/run/20261019T132551_train_seed1_leg_force_v3_s1), same command otherwise
(`--force_start_iteration 800`), stopped at 2014 to match the position-only run's budget; `model_2000` is the stage-1
checkpoint. Training-time medians over 40 iterations (sampled at reset, not a frozen evaluation):

| iterations | episode length (of 800) | 12 cm term (of 3) | 4 cm term (of 2) | foot error at reset |
| --- | --- | --- | --- | --- |
| 380-420 | 792 | 1.78 | 0.53 | 5.4 cm |
| 1180-1220 | 791 | 2.08 | 0.76 | 4.6 cm |
| 1580-1620 | 769 | 1.55 | 0.42 | 7.2 cm |
| 1960-2000 | 622 | 1.29 | 0.33 | 9.1 cm |

The decline from ~1500 is the pushes reaching full strength (ramped 800-1800): +-35 N a side can put ~60 N on the
raised foot, roughly 18 N·m about the edge of the three-foot support against ~14 N·m of gravity (18 kg, ~8 cm
margin). So `model_2000` falls more and tracks worse than `model_1200` did. The position-only run is trained on the
same schedule, so the two stage-1 checkpoints stay matched; the handle stage cuts the physical push to +-20 N for both
(`--foot_push_n 20`; the force-aware command channel stays +-35, being virtual).

**The handle, zero-shot and in two pilots.** None opened a latch:

| run | from | what changed | outcome |
| --- | --- | --- | --- |
| [zero-shot](#/week/6/run/20261019T141313_leg_eval_seed1_leg_eval_force_v3_1000_zeroshot), first frozen set | v3 `model_1000` | -- | 0/144: 79 missed (never settled above the bar), 42 past the 120 N damage force (p95 peak 319 N), 22 slipped; median door contact 21 N |
| [pilot 1](#/week/6/run/20261019T142125_train_seed1_leg_force_stage2_pilot) (150 it, 1024 envs) | `model_1000` | `--handle_fraction 0.5 --fixed_learning_rate 1e-4` | 92-97% of handle episodes missed, 45-69% of the rest damaged, none opened |
| [pilot 2](#/week/6/run/20261019T143608_train_seed1_leg_force_stage2_pilot2) (200 it) | `model_1200` | handles within reach (below) | 69-77% missed, engaged 39-44 mm off, 49-61% slipped, lever at most ~2° |

Each was diagnosed before the next change (scratch diagnostics, not recorded):

- **Out of reach.** The robot settles ~4 cm behind its spawn point. A kinematic sample of the FR leg with the base
  level put 48% of the first frozen set's pre-contact points more than 1 cm out of reach, and contact points at
  -0.08 m out by 4-7 cm: at hip height the foot cannot come inboard of y ~ -0.10, its leg plane being 9.55 cm outboard
  of the hip's ab/ad axis with the hip stopping at 60°. Doors 0.54-0.62 -> 0.46-0.54 m ahead, contacts -0.20..-0.06
  -> -0.22..-0.11 m; on the new frozen set nothing is more than 3 cm out of reach of a level base (worst 29 mm, at the
  end stop). **The frozen set was redefined** (2026-10-20 03:30): doors 0.47/0.50/0.53, contacts -0.20/-0.16/-0.12.
  Two pilot checkpoints had been scored on the first one; no final policy was.
- **A steady 5 cm miss.** Traced with pilot 2's checkpoint, the foot held its pre-contact goal 2.5 cm too far forward
  and 4 cm too high, the base pitched 22° nose-up, then descended ~4 cm toward the robot of the bar and passed in
  front of it without touching (0 N on the bar). It is F-118 again: the actor observes its joints, not its foot.
  Added the insertion's outer loop (`insertion.PoseCorrection`, F-120) to the handle's task layer, both variants: the
  measured miss integrated into the commanded goal, whole while approaching, only across the push direction while
  pressing.
- **The bias moves faster than the loop.** With the loop at 2/s the foot reached the pre-contact point within 1 cm,
  then drifted 2.6 cm toward the robot during a 0.5 s descent, touched the bar on its front shoulder (6 N, 1.4°) and
  was pushed off it. Loop 2 -> 4/s; descent 8-15 -> 4-7 cm/s; engage only within 1.5 cm (was 2.5), 3 s at the
  pre-contact point; the foot aimed 1.2 cm door-side of the bar's centreline rather than 0.8 cm robot-side, so an
  overshoot is caught by the door's face on the bar's door-side shoulder; a reward for the foot on the bar while
  pressing (a contact flag, so allowed to the position-only variant). Also: the door penalty is now per cm of overlap
  rather than per newton, so the position-only variant gets no force term (the plan's P0 rule).

**Matching.** Both stage-1 runs train on identical code paths: every change since v3 started touches only handle
episodes, evaluation or the viewer.

**Frames.** `leg_eval --leg_frames N --enable_cameras` saves the wrist RealSense's and a side camera's frames; the
door, rose and bar are drawn as markers, which both cameras render
([pipeline test](#/week/6/run/20261019T135610_leg_eval_seed1_leg_frames_pipeline_test), v3 `model_400`, 4 handles,
all missed). The wrist view at the look pose shows the door, the lever and the FR foot together; a white shape,
probably the D1's own closed fingers, fills the lower right of the image.

**Tests.** `tests/test_handle.py`, 14 pass. The full suite: 725 run, 1 failure, 7 skipped. The failure is
`tests/test_payload.py` `test_the_meshes_lie_inside_their_colliders` (1149 lidar-mount vertices outside both boxes):
pre-existing, from 81b9c4a's redrawn mount; nothing here touches the payload.

**Stage 2: four false starts** (force-aware, 2048 envs, each stopped within ~150 iterations, all recorded):

| start | from | change | what the training-time handle numbers showed |
| --- | --- | --- | --- |
| [a](#/week/6/run/20261019T151401_train_seed1_leg_force_stage2_s1) | `model_2000` | push +-35 N | stopped at once to cut the physical push to +-20 N |
| [b](#/week/6/run/20261019T151538_train_seed1_leg_force_stage2_s1b) | `model_2000` | +-20 N; loop 4/s; engage within 1.5 cm | ~99% of approaches never engaged |
| [c](#/week/6/run/20261019T153231_train_seed1_leg_force_stage2_s1c) | `model_2000` | loop back to 2/s; engage within 2 cm | still ~100% unengaged |
| [d](#/week/6/run/20261019T154414_train_seed1_leg_force_stage2_s1d) | `model_1200` | engage at the deadline wherever the foot is (training only) | engaged 75-77 mm off; 38-58% damaged; peak ~90-130 N |

`model_1200` replaced `model_2000` as the starting point for both variants: the full-strength pushes had cost it
tracking (above). Traced with `model_1200` after start d: on the lift the foot swung up to 11 cm ahead of its goal and
caught the bar from below (49 N, 35 N, then the damage force), and the loop wound up 3-4 cm while the goal was still
sliding. Fixed: the waypoint 12 cm back and 5 cm above the pre-contact point (was 4 and 5), the lift over 1.6 s
(was 1.2); the loop integrates only once the goal stands at the pre-contact point (and across the push while pressing),
and never while the door or the bar holds the foot; the aim back on the bar's centreline (the door-side aim put the
held foot into the door). Traced again: one of two lifts still kicked into the bar; the other engaged 1.2 cm off, came
down on the bar (22 N) and turned it 15° before sliding off -- the first lever turn by a policy that had never
trained on a handle.

**Stage 1, zero-shot on the frozen handles** (no handle in training; scored with the final task layer and the bounded
contact damping below):

| policy | latch held 1 s | engaged | opened at all | damaged (>120 N) | slipped | peak force median (p95) | door contact median |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [force-aware](#/week/6/run/20261019T171804_leg_eval_seed1_leg_eval_force_stage1_1200_h) `model_1200` | 0/144 | 106 | 3 | 74 | 101 | 121 N (221) | 33 N |
| [position-only](#/week/6/run/20261019T171934_leg_eval_seed1_leg_eval_position_stage1_1200_h) `model_1200` | 0/144 | 57 | 2 | 116 | 56 | 144 N (424) | 27 N |

`python run_unifp_train.py leg_eval --task leg --leg_variant <v> --checkpoint <model_1200.pt> --headless --small_physx`.
Neither turns a handle. The force-aware policy engages twice as often and passes the damage force less often, with a
lower peak; on the earlier contact model (unbounded damping,
[force](#/week/6/run/20261019T165935_leg_eval_seed1_leg_eval_force_stage1_1200),
[position](#/week/6/run/20261019T170105_leg_eval_seed1_leg_eval_position_stage1_1200)) the numbers were nearly the
same (78 / 121 damaged). Where the damage happens, from the traces: the position-only policy's mostly on the lift (50:
the foot kicked into the bar or the door) and at the pre-contact point against the door (27); the force-aware
policy's spread across the approach (34), the press (20), the hold (10) and the release (12). One seed each, a
pilot's numbers.

**Stage 2 recipes** (both variants from their `model_1200`, 2048 envs, `--handle_fraction 0.6 --foot_push_n 20
--fixed_learning_rate 1e-4`; from f on also `--resume_leg_std 0.2 --entropy_coef 0.002`; each stopped when its
training-time handle numbers showed the next problem; all recorded):

| recipe | change | force-aware | position-only |
| --- | --- | --- | --- |
| [e](#/week/6/run/20261019T155848_train_seed1_leg_force_stage2_e) | the approach fixes above | engaged 56-69 mm off, 45% damaged | -- |
| f ([force](#/week/6/run/20261019T160947_train_seed1_leg_force_stage2_f), [position](#/week/6/run/20261019T165037_train_seed1_leg_position_stage2_f)) | leg exploration noise 0.45 -> 0.2 (0.11 -> 0.05 rad a joint; F-119's lesson) | engaged 15-26 mm off; damage 28-66%; 96% slip, lever at most ~5° | stopped early |
| g ([force](#/week/6/run/20261019T165614_train_seed1_leg_force_stage2_g), [position](#/week/6/run/20261019T165614_train_seed1_leg_position_stage2_g)) | touch, then turn: the reference waits 0.6 s at the bar's top, the force law capped at 4 N meanwhile; the cap 60 -> 40 N | 61-71% damaged, peak ~175 N | learnt to keep the foot 8-12 cm off |
| h ([force](#/week/6/run/20261019T171618_train_seed1_leg_force_stage2_h), [position](#/week/6/run/20261019T171618_train_seed1_leg_position_stage2_h)) | contact damping bounded by the spring | 43-56% damaged, 86-95% slip | the foot kept on the floor (28-40 cm off) |

Two diagnoses behind f-h, from traces of the force-aware policy: the foot arrived at the bar's top 2.5-4 cm late and,
pushed by the layer as it arrived, was carried 3-4 cm toward the robot onto the bar's front shoulder and off it (g);
and the physics-rate peak force was a median 1.5x (p90 2.2x) the step's mean contact, the worst 406 N against a 26 N
mean, because the lever, light and lightly damped, snapped back on its spring when the foot slipped and the contact's
damping on that closing speed was unbounded (h). After h the damage terminations were teaching avoidance, so:

**Recipe i** ([force](#/week/6/run/20261019T173407_train_seed1_leg_force_stage2_i),
[position](#/week/6/run/20261019T173407_train_seed1_leg_position_stage2_i), stopped ~1420): h, with the damage
force no longer ending a training episode (recorded, and still a failure in every frozen evaluation). Both jammed:
engaged 16-26 cm off, the force-aware run 64-86% damaged with peaks of 219-288 N -- the lifting foot caught under the
bar, which its rest stop will not let turn up. Traced with `model_1200` (scratch): to rise, the FR foot has to swing
forward, and a handle at body height 0.42-0.48 m ahead sits in that swing; a straight-line goal lets it cut under.

**Recipe j** (the last; both variants, the full 1500 iterations): i, plus the lift in two legs (straight up to 14 cm,
then to a waypoint 15 cm back and 6 cm above the pre-contact point, over 2 s); a penalty on overlap with the bar or
the door before the press begins (per cm: geometry, so allowed to the position-only variant); and the contact softened
to 8,000 N/m with a 20 mm cap. Traced first with `model_1200`: the foot now reached the bar on the way down, but the
16,000 N/m contact rang, on and off the bar every step or two, 135-229 N peaks against step means under 63 N.
Training-time handle numbers at 1500 iterations (300 into stage 2): engaged 14 / 21 mm off (force / position),
damage 30 / 44%, slip ~100% for both, the lever at most 7-11°. The position-only run spent its first ~200 stage-2
iterations with the foot held 9-21 cm off before coming back.

Recipe j ran to completion: [force](#/week/6/run/20261019T175803_train_seed1_leg_force_stage2_j),
[position](#/week/6/run/20261019T175803_train_seed1_leg_position_stage2_j), 1500 iterations each, 1 h 55 min side by
side. Training-time, last 100 iterations: engaged ~11 mm off (both), latch drawn back at some point in 21% / 16% of
handle episodes (force / position), damage 24% / 48%, slip ~100%.

**The frozen evaluation** (`leg_eval ... --checkpoint <run>/model_2699.pt --headless`; final checkpoints, no
selection):

| policy | latch held 1 s | opened | damaged | slipped (any) | held at 1 / 2 / 3 / 4 N·m | peak median (p95) | held force |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [position-only](#/week/6/run/20261019T223051_leg_eval_seed1_leg_eval_position_stage2_j_2699) | **17/144** | 48 | 24 | 136 | 16 / 1 / 0 / 0 | 80 N (128) | 11 N |
| [force-aware](#/week/6/run/20261019T222952_leg_eval_seed1_leg_eval_force_stage2_j_2699) | **12/144** | 61 | 48 | 132 | 12 / 0 / 0 / 0 | 86 N (138) | 12 N |

Neither fell; both engaged every handle. The latch needs 9 / 17 / 25 / 32 N at the foot for the four springs, and
the levers almost never reached their end stops, so the force-aware layer's bound on force into a stop was never
exercised. On right-pointing levers the force-aware policy slipped (50 of 72) and on left-pointing ones it passed the
damage force (46 of 72). Figure: `figures/leg_handle_eval_stage2_seed1.png` (success and peak force by spring; one
handle, 64, traced under both).

**Diagnostic: every bar 20 mm in radius** (`--leg_bar_radius 0.02`; not the frozen set):
[position](#/week/6/run/20261019T223120_leg_eval_seed1_leg_eval_position_stage2_j_2699_bar20mm) 22 held, 108
damaged; [force](#/week/6/run/20261019T223022_leg_eval_seed1_leg_eval_force_stage2_j_2699_bar20mm) 14 held, 122
damaged. Slipping as the failure falls from 74 / 59 episodes to 2 / 3 and the peak rises to ~130 N for both: on a
thick bar the foot stays on and pushes past 120 N. Two limits, then: the 2 cm round bar (a 4.4 cm ball at μ 0.6
leaves it beyond ~1.6 cm off centre), and force once the foot is on.

**Holding** (the layer's hold with the latch mostly open; 14 force-aware, 23 position-only episodes): on the bar 0.68
/ 0.58 of the hold, contact force std/mean 0.88 / 1.10, lever std 1.4° / 2.6°, mean lever speed 28 / 34 °/s. On
handle 64 the force-aware policy held the lever at 45° on a steady ~6 N; the position-only one turned it too, its
contact bouncing 0-50 N and the lever between 30° and 42°. A small difference, one seed.

**What the wrist camera sees** ([force frames](#/week/6/run/20261019T223153_leg_eval_seed1_leg_frames_force_stage2_j),
[position frames](#/week/6/run/20261019T223252_leg_eval_seed1_leg_frames_position_stage2_j); handles 64, 66, 72, 74;
both opened handle 64 and held it). `figures/leg_handle_frames_force_handle64.png` and `..._position_handle64.png`
are contact sheets of handle 64: lift, over the bar, pressing, latch open, holding, lifting off, the wrist view beside
a side view. The side view shows the foot on the bar's top, the bar turning green once the latch draws back at 39°.
The wrist view shows the door and the rose throughout, and the FR foot at the image's lower left; but a white shape
fixed in the image -- on the camera's own link, so almost certainly the D1's closed fingers -- covers ~40% of it on
the right, and for this right-pointing lever hides the bar and most of the foot while it presses. F-138 had the
fingertips only entering the right edge; at this pose they block what the camera is there to watch.

**Second seed** (2026-10-20 09:40-11:30): recipe j stage 2 for both variants with `--seed 2`, from the same
`model_1200` ([force](#/week/6/run/20261019T223505_train_seed2_leg_force_stage2_j_s2),
[position](#/week/6/run/20261019T223505_train_seed2_leg_position_stage2_j_s2)), then the frozen set:

| policy | seed 1 held 1 s | seed 2 held 1 s | of 288 | seed 2 at 1 / 2 / 3 / 4 N·m | seed 2 damaged / slipped |
| --- | --- | --- | --- | --- | --- |
| position-only | 17 | [43](#/week/6/run/20261020T003041_leg_eval_seed1_leg_eval_position_stage2_j_s2_2699) | **60** | 28 / 11 / 4 / 0 | 59 / 31 |
| force-aware | 12 | [3](#/week/6/run/20261020T003011_leg_eval_seed1_leg_eval_force_stage2_j_s2_2699) | **15** | 3 / 0 / 0 / 0 | 6 / 126 |

Seed 2's position-only policy holds 57 latches with the foot on the bar throughout (contact std/mean 0.53, lever std
1.1°) at up to 60 N on the 4 N·m springs; its force-aware policy holds ~12 N whatever the spring and slips off in 126.
The seed-1 "steadier hold" for the force-aware policy does not reproduce. `figures/leg_handle_eval_stage2_seed2.png`.

[F-159](../findings.md), superseded by [F-160](../findings.md) the same morning. Nothing here is a P0-P4 result or
touches G1-G7.

**Watching it** (2026-10-20, Lukas asked for Isaac Sim rather than a recording): `play --task leg --leg_rows 64 66 68
107 --real_time` now loops chosen frozen handles, one per environment, the viewer fixed on environment 0 from the
front right with only the handle's markers; both seed-2 policies were left running in two windows
([position](#/week/6/run/20261020T030436_play_seed1_leg_viewer_position_s2),
[force](#/week/6/run/20261020T030544_play_seed1_leg_viewer_force_s2)). Frames of the same four handles were captured
first ([position](#/week/6/run/20261020T025831_leg_eval_seed1_leg_video_position_s2): 2 of 4 held;
[force](#/week/6/run/20261020T025938_leg_eval_seed1_leg_video_force_s2): 0 of 4). In the 144-environment evaluation the
position-only policy held all four: PhysX does not reproduce an episode across environment counts, so a handle in the
viewer is an example, not the frozen result. Both viewers were stopped at Lukas's request the same afternoon; their records still read
"running" because a stopped playback never writes its final status.

**Stage 1 zero-shot on the final plant** ([force](#/week/6/run/20261019T175951_leg_eval_seed1_leg_eval_force_stage1_1200_j),
[position](#/week/6/run/20261019T180115_leg_eval_seed1_leg_eval_position_stage1_1200_j)): both 0/144. Force-aware:
engaged 108, opened 5, damaged 48, slipped 104, peak 82 N median (p95 220). Position-only: engaged 124, opened 2,
damaged 54, slipped 111, peak 89 N (p95 290). With the lift in two legs both reach the bar; most then slip off it.

### 2026-10-21 · The key in the cam lock: inserted by the arm, turned by the front-right foot

Lukas's request of 2026-10-21 (10 h of GPU, 00:10-10:10): his lever key (`combinerbox_lever v3.stl`, a 10 mm shaft
with two wards on a bar, a flat 100 x 50 mm platform at its outer end) inserted into the box's cam lock with non-RL
methods, then turned with the front-right foot from 45 degrees up to 45 degrees down, the door unlocking past 30 down;
position-only against force-aware; the support feet kept near the Go2's normal stance; why the force-aware handle
policies never passed ~12 N; a staged demo like the combiner door's, with an option to start with the key in; the head
RealSense with its mount, and the MID-360. This entry is written as the work goes.

**Pulled first.** 9136aba (the combiner door demo, F-153 to F-158) came in under the uncommitted leg-handle work.
Its findings took F-153 to F-158, so the leg-handle findings written on 2026-10-20 as F-153 and F-154 are now
**F-159** (superseded) and **F-160**; their text is unchanged except the numbers, here, in `findings.md` and in
`unifp_train/README.md`.

**Why the handle policies' feet were out of place** (a scratch probe, `stance_probe.py`, not recorded: 64 robots, the
handle task's training mix, recipe j seed 1's final checkpoints, against zero actions). In the base's yaw frame, the
settled stance under zero actions puts the feet at FL (+0.23, +0.17), RL (-0.22, +0.17), RR (-0.22, -0.17) m. Under
both policies the right-rear foot stood 13-20 cm off that by **0.1 s** (5 policy steps), and the left-rear 12-13 cm:
the right-rear forward under the body (x -0.02 to -0.08), the left-rear back. The spawn is UniFP's 0.35 m, the
settled base 0.272 m: for the first ~0.08 s the feet are in the air, and the policies use it to place the right-rear
foot where a tripod over FL-RL-RR has room for the centre of mass (with the feet at their places the CoM sits on the
FL-RR diagonal). Once down, `support_slip` pays them to leave it there, and nothing asked for it back. So Lukas's
reading was right: it is the spawn, and the reward let it stand.

**Why the force-aware policy never passed ~12 N** (the recipe-j frozen traces, seeds 1 and 2). The command itself
rarely asked for more: the largest command in each episode had a median of 16.5 N (seed 1) and 9.1 N (seed 2), against
a cap of 19-40 N. The force law (`HandleTaskLayer`) raises the force only from the foot's lag behind its reference
along the arc, and on the 2 cm round bar the foot slid off (126 of 144 handles on seed 2) before it lagged: a slipping
foot follows its moving goal, so the lag, and the command, stay small. Where the command did reach 16-41 N, the force
delivered along the turn was 11-27 N (median), with the foot 17-50 mm (median) off its goal while pressing -- at the
400 N/m virtual spring, 7-20 N of force uncertainty. It was the bar and the law, not a limit of the leg: the
position-only policies, pushing on position error, reached ~60 N. The key's flat platform is the test of that.

**The key task** (`--task leg --leg_object key`; `unifp_train/leg_cfg.py`, "the combiner box's key"). The handle task
with:
- *the key's plant* (`handle.py`, generalised and tested: 23 tests): the key's bar and platform as two boxes the foot
  sphere touches, a start stop at -45 degrees, the end stop at +45, the unlock at +30, a lock friction that *sticks*
  below its level (0.3-2.5 N·m) plus a cam detent over the 20 degrees before the unlock (0-1 N·m), no return spring, the
  key's own weight; the task layer's path starting from -45 (onto the platform along its normal); the key on a lock
  adapter 17 mm off the door (below); the robot's head checked against the door's face as well as the foot and knee.
  The lock is at its place on the real box: 0.4175 m up, 64 mm in from the left edge, so the platform at the start
  puts the foot's centre ~0.48-0.51 m up, at the edge of what the leg reaches with the base level.
- *the stance*: every episode starts standing settled (base 0.272 m, the knees as gravity bends them, measured under
  zero actions) instead of dropping; a reward for each support foot's distance from its settled place
  (`support_stance`, exp(-(d / 6 cm)^2)); the front-right foot's footfall corrected to where it actually stands (the
  handle task's was 4.5 cm behind and 3 cm inboard of it); the D1 folded in its rest pose (the camera is on the head
  now), its jitter small.
- *after the turn* the foot goes back to the waypoint and down onto its footfall, and the episode ends a second after.
- a frozen set of 108 keys (`leg_eval.build_key_plan`: 3 distances x 3 sides x lock friction 0.5 / 1.5 / 2.5 N·m x
  detent 0 / 1 N·m x 2 heights), scored as the handles are, plus the unlock, the foot back down, and the stance.

**The key as drawn points the wrong way.** With its shaft in the lock and its platform up, the STL's lever points to the
robot's *left*, off the box past its edge; the turn Lukas described (front-right foot, clockwise, downward) needs it
pointing right, toward the box's middle, and no rotation of the part gives that with the platform up. Everything here
uses it mirrored left to right -- the same part printed handed the other way. Worth checking before printing.

**Runs so far** (each recorded): smokes of stage 1 and of key episodes, and a 3-row zero-action pass of the frozen
evaluation (all clean; the settled spawn holds the stance to 0.13 cm under zero actions). The first stage-1 pair
(4096 envs, stance weight 2) did not survive: the position-only run died of CUDA out-of-memory at startup beside the
force run (one 4096-env leg run takes 6.6 GB of the 11.6), and the force run was stopped at iteration 463 to restart the
pair matched. In its 437 iterations the median support foot stood 2.4 cm from its place, but in each robot one foot was
still ~15 cm off -- stepped there to make the tripod. The restart (`stage1b`, 2048 envs each, stance weight 4) had, at
iteration ~370, the worst support foot per robot 3.8 cm (force) and 3.1 cm (position) from its place (training-time,
sampled at reset).

    logs/unifp_train/launch_key.sh key_<v>_s1_stage1b train --task leg --leg_object key --leg_variant <v> \
        --num_envs 2048 --iterations 1300 --force_start_iteration 500 --foot_push_n 25 --headless \
        --run_name key_<v>_s1_stage1b --seed 1

**Stage 1 finished, and the position-only run came apart**
([force](#/week/6/run/20261020T140313_train_seed1_key_force_s1_stage1b),
[position](#/week/6/run/20261020T140352_train_seed1_key_position_s1_stage1b), 1300 iterations, 1 h 37 min side by
side). Training-time means per 100 iterations (sampled at reset): foot error / the worst support foot's distance from
its place / episode length of 900:

| iterations | force | position |
| --- | --- | --- |
| 400-500 | 6.9 cm / 3.6 cm / 897 | the same (identical runs until the pushes start) |
| 600-700 | 6.5 / 4.0 / 885 | 6.4 / **7.0** / 896 |
| 700-800 | 6.3 / 3.8 / 893 | 6.4 / **15.1** / 896 |
| 1000-1100 | 6.0 / 3.6 / 894 | **21.6** / 14.4 / 881 |
| 1200-1300 | 5.2 / 3.8 / 897 | **24.2** / 15.0 / 888 |

The two runs are the same seed and were identical (to the printed digit) until iteration 500, where the pushes on the
foot and the force channel begin. From ~600 the position-only policy went back to stepping a support foot out of
place, and from ~900 its foot stopped following its goals; the force-aware one held both. The pushes were still small
then (+-2.5 N at 600, +-7.5 N at 800 of the +-25 N ramp), so this is not simply strength; the position-only policy is
told nothing of a push it has to reject, and the force-aware one yields to it by design.

**Stage 2 starts from both `model_600`**: the last checkpoint before the collapse, matched, the two variants there
differing only by 100 iterations of the force channel. A first start from each run's `model_1299` (the position-only
one collapsed) was stopped at launch
([force](#/week/6/run/20261020T154059_train_seed1_key_force_s1_stage2),
[position](#/week/6/run/20261020T154159_train_seed1_key_position_s1_stage2)). This is a choice made on training-time
numbers, before any key evaluation (the frozen set, run on both `model_600`s later: 0 of 108 each).

A detached queue (`logs/unifp_train/key_queue2.sh`; it waits on process ids, never on a checkpoint's name, and takes
each run's highest-numbered checkpoint) runs stage 2 for both variants (1500 iterations asked, stopped at 1000 -- below; 60% key episodes, the
recipe-j settings: pushes +-20 N, leg noise 0.2, learning rate 1e-4), seed 1 then seed 2 from the same `model_600`s,
the frozen evaluation of every final checkpoint and of both `model_600`s zero-shot, with a window between the seeds
for the demo's camera runs (the head camera's renderer does not fit beside two training runs on the 12 GB GPU).

**Seed 2** (the same commands with `--seed 2`, stage 2 from the same `model_600`s, stopped at 1000 iterations;
[force](#/week/6/run/20261020T181513_train_seed2_key_force_s2_stage2b),
[position](#/week/6/run/20261020T181612_train_seed2_key_position_s2_stage2b); scored at `model_1605` / `model_1614`:
[force](#/week/6/run/20261020T201142_leg_eval_seed1_key_eval_force_s2_final),
[position](#/week/6/run/20261020T201252_leg_eval_seed1_key_eval_position_s2_final)):

| 108 frozen keys | force-aware | position-only |
| --- | --- | --- |
| **unlocked, foot back down, no damage, no fall** | **106** | **23** |
| by lock friction 0.5 / 1.5 / 2.5 N·m | 36 / 36 / 34 | 23 / 0 / 0 |
| past 120 N | 1 | 34 |
| peak force, median (p95) | 50 N (73) | 83 N (180) |
| time to unlock, median | 2.4 s | 2.0 s |
| worst support foot from its place, median (p90) | 8.4 cm (9.8) | 7.5 cm (9.1) |

Two seeds: **190 of 216 against 43** (F-166, superseding F-163). The seeds agree, unlike the handle's. Seed 2's
position-only run unlocked 68-78% of its training key episodes from stage-2 iteration 400 on (seed 1's: under 15%),
but most of them past 120 N, and in the frozen set it turns only the 0.5 N·m lock. `figures/key_eval_stage2_seed2.png`,
`figures/key_training_curves_seed2.png`.

**The staged demo** (`demos/combiner_key/`, README there; 9 CPU tests in `demos/combiner_key/tests`). The combiner
door demo's stack (P2Dingo's walking policy, the D1's arm IK at 10 Hz, the head RealSense and the MID-360 for the
look), with the key: look from the box's middle, unfold, walk to the insertion stance (0.17 m in, 0.33 m left), insert,
let go, back out, fold, walk 10 cm in, turn with the leg policy, stand; `--key_inserted` skips the arm. Pieces:
- the slot adjusted: a printed lock adapter over the cam lock (16 mm thick, 1 mm off the door), its keyway the key's
  outline plus 1.5 mm a side behind a 45-degree lead-in 5 mm wide, at the insertion angle (45 degrees up). The leg
  task's key geometry was moved out by the adapter's 17 mm before stage 2 read it.
- the key a free rigid body, held by a capped 6-axis spring from the jaw centre (600 N/m, 5 N·m/rad, at most 10 N and
  0.3 N·m), the arm feeling its reaction; let go, PhysX keeps it in the keyway.
- the turn run by `LegTurn`, which calls the leg environment's own approach, layer, outer loop and goal-hold methods on
  its own state, the trained plant at the key's true pose, the layer told the estimate; the box latched until the key
  passes 30 degrees.
- `--given_box` (no camera or lidar, the true placement given): for developing the rest while two training runs fill
  the GPU -- the renderer's memory does not fit beside them (two pipeline tests died of it, recorded).

**Developing the insertion** (`--given_box`, one attempt each, all recorded): the box's place known exactly, so these
test the arm, the grip and the keyway, not the perception.

| test | change | outcome |
| --- | --- | --- |
| 1 | as built: 800 N/m, 0.8 N·m/rad grip | the key twisted 48 deg in the hand on meeting the adapter; not seated |
| 2 | the IK seeded at the pre-insertion pose | the hand on the keyway's axis (1.7 deg), the key still pivoted 48 deg |
| 3 | the grip's error traced | < 1 mm and 1 deg through the walk and approach, until the shaft met the adapter |
| 4 | a stiff grip on a key made 0.15 kg (for the explicit spring's stability) | the D1 sagged 22 deg off its approach |
| 5 | the key at its real 53 g, the inertia raised instead | aligned to 1.5 deg, but the hand **16 mm low** at the pre-insertion point |
| 6 | an outer loop on the hand's measured position | saturated at its 30 mm cap; the arm could not follow |
| 7 | **the robot without the wrist payload** | **seated**: 0.16 mm out, 1.5 mm across, roll -0.8 deg, tilt 0.7 deg; still seated after letting go |

The cause of 5 and 6: the robot was the leg policy's training robot, whose payload physics puts the wrist RealSense and
its bracket (99 g) on Link6. The CPU model (`position_only.workspace.gravity_torques`, without the wrist payload) has
the chosen insertion poses at 78% of the worst joint's limit; the camera and the key on the hand, ~0.5 m out, add about
0.74 N·m at the shoulder, enough to saturate it. The camera is on the head in this task, so the demo now uses the
combiner door demo's robot (payload drawn, not simulated) -- one more way it differs from the leg policy's training
robot, which carried the lidar's and that camera's mass. With it, the scripted insertion (IK, the 10 Hz servo model,
the hand's outer loop from the arm's own encoders, the lead-in) seated the key from a perfectly known box at the
first try. That is one attempt with the true box; with the look's error, below, it did not seat.

**Seed 1: the frozen evaluation** (`leg_eval --task leg --leg_object key --leg_variant <v> --checkpoint <run>/<latest>
--headless`, run by the queue; 108 keys, one episode each). Stage 2 was stopped at 1000 iterations (the key's plant
runs at ~6.2 s an iteration, against ~4.6 for the handle's; 1500 for both seeds and the demo would not fit the day):
the queue's interrupt saved `model_1607` (force) and `model_1615` (position), and those were scored -- the last
checkpoints, no selection. [force](#/week/6/run/20261020T174654_leg_eval_seed1_key_eval_force_s1_final),
[position](#/week/6/run/20261020T174805_leg_eval_seed1_key_eval_position_s1_final); zero-shot from stage 1's `model_600`:
[force](#/week/6/run/20261020T174915_leg_eval_seed1_key_eval_force_stage1_600_zeroshot),
[position](#/week/6/run/20261020T174931_leg_eval_seed1_key_eval_position_stage1_600_zeroshot).

| 108 frozen keys | force-aware (P1) | position-only (P0) |
| --- | --- | --- |
| **unlocked, foot back down, no damage, no fall** | **84** | **20** |
| unlocked (past 30 deg down) | 87 | 27 |
| past the 120 N damage force | 14 | 79 |
| fell | 0 | 0 |
| by lock friction 0.5 / 1.5 / 2.5 N·m (36 each) | 36 / 35 / 13 | 18 / 2 / 0 |
| peak contact force, median (p95) | 82 N (136) | 142 N (218) |
| time from engaging to unlocked, median | 1.6 s | 2.5 s |
| the foot back on its footfall, median miss | 1.0 cm | 4.9 cm |
| worst support foot from its place over the episode, median (p90) | 8.9 cm (10.4) | 10.7 cm (12.5) |
| zero-shot from stage 1 | 0 (58 fell, 50 damaged) | 0 (96 damaged) |

The force law on the key's flat platform asks for more than it did on the handle's bar: the largest command in each
episode has a median of 21 N (p90 32 N; the handle: 9-16 N). The force the foot delivers along the turn is below what
it is asked for: a median 8.7 N for commands under 15 N, 15.7 N for 15-30 N, 20.4 N for 30-45 N. The impact peaks
(82 N median) are well above both: the foot meets the platform harder than the law asks. The detent costs the
force-aware policy little (44 / 40 of 54 without / with it); the lock's friction is what stops it (13 of 36 at
2.5 N·m, where the foot needs 26-37 N along the turn at the layer's 9.25 cm reach). Most episodes on both are also
flagged "slipped" (81 force, 101 position): off the platform for more than 0.1 s while the layer still pressed, most of
them after the key had reached its stop.

`figures/key_eval_stage2_seed1.png` (success by lock strength, peak force against what the lock needs, the worst
support foot over time, and key 54 traced under both); `figures/key_training_curves_seed1.png` (training-time: stage 1
dashed, stage 2 solid).

**The demo with the head camera and the MID-360**, seed 1's final leg policies, in the window between the seeds (four
robots a run, the same four box placements each run: the box 0.70 m ahead of the look +-2 cm, +-5 degrees; recorded;
the first try crashed at reset -- one robot's foot materials cannot be written alone -- and was rerun):

| run | look: the lock's estimate (depth, across), heading | insertion | turn |
| --- | --- | --- | --- |
| [insert and turn, force](#/week/6/run/20261020T175404_force_seed1_key_demo_force_s1) | 4/4 tags; -3.0 / +2.5 / +9.9 / +11.4 mm deep, < 0.6 mm across; 0.1-1.1 deg | **0/4 seated**: two on the keyway's axis (1.3-1.4 mm across) stalled 5-8 mm short, tilted 3-4 deg; two (10-11 mm deep errors) missed the lead-in | not started |
| [insert and turn, position](#/week/6/run/20261020T175841_position_seed1_key_demo_position_s1) | the same | **0/4**: 7.8 and 2.7 mm short (tilted 4.2, 2.1 deg); two missed | not started |
| [insert with the lidar's depth and 15 mm overdrive](#/week/6/run/20261020T180321_force_seed1_key_demo_force_s1_v2) | lidar's face a near-constant 1.8-2.6 mm in front of every estimate | **0/4**: the deeper target put the IK out of what it solves (it held), the hand drifted 4-14 deg | not started |
| [key in, settled handover, force](#/week/6/run/20261020T180759_force_inserted_settled_seed1_key_demo_force_s1_settled) | 1.8-9.5 mm | -- | **0/4 engaged**: the foot never settled at the pre-contact point; peaks 0-212 N |
| [key in, settled handover, position](#/week/6/run/20261020T181112_position_inserted_settled_seed1_key_demo_position_s1_settled) | 1.0-10.8 mm | -- | **4/4 unlocked**, foot back down 4/4, the foot's own measure of the turn past 30 deg 3/4; but peaks 69 / 141 / 187 / 154 N: **1/4 without passing 120 N** |

(The demo's own summary did not count the 120 N damage force when these ran; it does now.) What these show:
- The look is good across and in heading, and poor in depth: the tag's range is off by up to 11 mm along the door's
  normal. The arm, the outer loop and the lead-in put the key on the keyway's axis to 1.5 mm whenever the depth was
  within ~3 mm, and then the key stalled short: the grip's push depends on how far past the seat the IK aims, which
  the depth error changes (0.6-4 N here). The lidar's face fit does not measure that error (one trial), and aiming
  15 mm deeper broke the IK. So the non-RL insertion seats the key from a known box (the development runs) and not
  yet from the look; the missing piece is along the axis -- a push that is force-limited rather than position-aimed,
  or a depth the robot measures (the foot touching the door, the hand's own contact).
- The settled-handover diagnostic (a teleport into the leg policy's own stance, so not the demo) separates the turn
  from the handover: in this scene the **position-only policy turns the key and the force-aware one does not** -- the
  reverse of the frozen evaluation. The scene differs from training in its legs' actuator (Isaac Lab's Go2 DCMotor:
  23.5 N·m on every joint with a torque-speed curve, against UniFP's ideal PD with 45.43 N·m knees), self-collision
  (off) and the payload (drawn, not simulated); the feet's friction is set back to the robot's own for the turn, and
  the ground is UniFP's, as trained. A frozen evaluation with only the legs' actuator swapped says it is the actuator
  (`leg_eval ... --leg_actuator dcmotor`, seed 1's finals, the training environment otherwise unchanged):

  | 108 frozen keys | UniFP's ideal PD (trained) | Go2 DCMotor |
  | --- | --- | --- |
  | force-aware | 84 | **3** ([run](#/week/6/run/20261020T181728_leg_eval_seed1_key_eval_force_s1_final_dcmotor): engaged 2, 105 past 120 N) |
  | position-only | 20 | **20** ([run](#/week/6/run/20261020T182100_leg_eval_seed1_key_eval_position_s1_final_dcmotor): 28 unlocked, 83 past 120 N) |

  The force-aware policy's skill does not survive the change of actuator model; the position-only policy's does,
  such as it is. A plausible reason, not tested: the force-aware policy reads the external force off its own
  dynamics (its estimator is supervised on it), so a leg that answers torque differently looks like a push, and the
  target it tracks moves by it/400 N/m; the position-only policy has no such channel. The real Go2's knee is rated
  at 45 N·m, as UniFP's model has it, but its motors are neither model.
- The demo scene again, the settled handover with the box given (no camera), the legs on UniFP's ideal PD
  (`--legs unifp`) and nothing else changed:
  [force](#/week/6/run/20261020T182511_force_inserted_settled_unifplegs_seed1_key_demo_force_s1_settled_unifplegs_given)
  **4/4** (unlocked, its foot's measure past 30 deg, the foot back down, peaks 76-88 N);
  [position](#/week/6/run/20261020T183653_position_inserted_settled_unifplegs_seed1_key_demo_position_s1_settled_unifplegs_given)
  **0/4** turned, every one past 120 N (130-267 N). Matched actuators, the PhysX scene agrees with the frozen
  evaluation. One more thing it shows: the robot's own check -- the turn as the foot measured it at the end of the
  hold -- read "past 30 deg" on all four position-only attempts, whose keys never moved: the foot had slid down past
  the platform. The foot's position alone is not a verification of the unlock.
- And with the head camera and the MID-360 back in the loop (the look's estimate 1.1-10.3 mm off), ideal-PD legs, the
  settled handover:
  [force](#/week/6/run/20261020T201437_force_inserted_settled_unifplegs_seed1_key_demo_force_s1_settled_unifplegs_camera)
  **3/4** (peaks 70-87 N; the fourth past 120 N at 181 N, its key not turned),
  [position](#/week/6/run/20261020T201806_position_inserted_settled_unifplegs_seed1_key_demo_position_s1_settled_unifplegs_camera)
  0/4 (138-295 N). The look's depth error, which defeats the arm's insertion, is inside the +-1 cm the leg policy
  trained with.

**The handover to the leg policy** went wrong twice before the turn could be tested: switching straight from the
walking policy's stand (base 0.31 m) to the default targets dropped the robot to 0.24 m and lurched it 22 cm (now
eased over 1.5 s from the walking policy's last action, then 0.5 s held, then the policy with an empty history); and
P2Dingo's clocked gait stands inside 0.1 m/s on each axis, so a diagonal 0.15 m/s command whose forward share was
0.07 walked sideways only and stopped 19 cm short (now each axis driven at >= 0.15 m/s until within 12 mm, the walks
longer).

And with the legs matched (`--legs unifp`) the honest handover still fails
([run](#/week/6/run/20261020T184836_force_inserted_unifplegs_seed1_key_demo_force_s1_crouch_unifplegs_given), key in,
box given, force-aware, 4 attempts): P2Dingo walks on the ideal-PD legs as well (arriving within 1.2-1.7 cm), but
easing to the default targets moves the base back ~10 cm over feet that stay where it left them. The leg policy starts
with its worst support foot 15.6-15.9 cm from its place and the door 0.54-0.56 m ahead (trained: 0.40-0.47), and
engaged 0 of 4. The leg policy has only ever started from its own settled stance; taking over from another
controller's is a training problem (F-165), not one the demo can script round.

**A handover fine-tune, time-boxed** (07:30-08:30, the GPU's last hours; F-167). The stances P2Dingo leaves at the
turn's stance, recorded from [32 attempts](#/week/6/run/20261020T202942_force_inserted_unifplegs_seed3_walker_stances)
on ideal-PD legs (17 of which fell with the eased handover): the base 0.31 m up, rear thighs 0.60-0.67 rad against the
settled 0.99, nose 2.5 deg down (`demos/combiner_key/assets/walker_stances.json`). Seed 2's force-aware `model_1605`
fine-tuned 500 iterations with half its episodes starting from one of them
([run](#/week/6/run/20261020T203809_train_seed2_key_force_s2_stage3_walker), 24 min alone on the GPU):

    ./run_unifp_train.py train --task leg --leg_object key --leg_variant force --num_envs 2048 --iterations 500 \
        --handle_fraction 0.6 --foot_push_n 10 --force_start_iteration 0 --force_ramp_iterations 1 \
        --spawn_states demos/combiner_key/assets/walker_stances.json --resume_from <seed 2 model_1605.pt> \
        --resume_leg_std 0.2 --fixed_learning_rate 1e-4 --entropy_coef 0.002 --headless --seed 2

| | frozen keys, from its own stance | from the walker's stances, at rest (`--eval_spawn_states`) | the demo, direct handover |
| --- | --- | --- | --- |
| before (seed 2, stage 2) | 106 / 108 | [26 / 108](#/week/6/run/20261020T211611_leg_eval_seed1_key_eval_force_s2_stage2_walkerstarts) (45 never engaged) | -- |
| after the fine-tune | [102 / 108](#/week/6/run/20261020T210248_leg_eval_seed1_key_eval_force_s2_stage3_walker) | [**97 / 108**](#/week/6/run/20261020T211501_leg_eval_seed1_key_eval_force_s2_stage3_walkerstarts) | [0/8 box given](#/week/6/run/20261020T210357_force_inserted_unifplegs_seed1_key_demo_stage3_direct_given) (5 fell), [0/4 camera](#/week/6/run/20261020T210652_force_inserted_unifplegs_seed1_key_demo_stage3_direct_camera), [0/4 with insertion](#/week/6/run/20261020T211008_force_unifplegs_seed1_key_demo_stage3_full_camera) |

The stance was learnable; the demo still breaks it. Not by the motion at the switch: with the robot's velocities zeroed
there ([run](#/week/6/run/20261020T211833_force_inserted_unifplegs_seed1_key_demo_stage3_direct_still_given)) it is 0/8 as well, 4 fell, the foot never reaching the pre-contact point (peaks
99-390 N on the way) -- while from the settled stance the same scene turns 4/4. What in the scene breaks a start from the
walker's stances is not separated: the box's real colliders round the approach (the adapter's block is not in the
training plant), self-collision off, the payload unsimulated.

**Frames** (`leg_eval ... --enable_cameras --leg_frames 2 --leg_rows 54 30`, seed 1's finals;
[force](#/week/6/run/20261020T202459_leg_eval_seed1_key_frames_force_s1_v2),
[position](#/week/6/run/20261020T202628_leg_eval_seed1_key_frames_position_s1_v2); a first pair with the side view
behind the door's plane, unusable, recorded): `figures/key_frames_force_key54.png` -- the foot up to the key at 3 s, the
key past the unlock (green) at 5.4 s and held, lifted off at ~10 s, the foot back on its footfall by 12.6 s;
`figures/key_frames_position_key54.png` -- the key turned to about level, then the foot sliding off below it. The
analytic key is drawn as markers (the adapter dark, the key grey / orange / green). **The head RealSense sees only the
door at the turn's stance**: the key is ~10 cm ahead of the camera and ~48 degrees to its right, outside the D435's
+-34.5 degrees -- from where the robot turns the key, its head camera cannot watch the key or the foot.

**What today shows, and what it does not.**
- *Shows* (simulation): trained UniFP-style with a settled spawn and a stance reward, the front-right foot turns
  Lukas's key through 90 degrees in the training plant -- force-aware 190 of 216 frozen keys over two seeds,
  position-only 43 (F-166) -- the force-aware one at every lock friction tried and with less force; why the handle
  policies stood badly (the spawn's drop, F-162) and why the force-aware handle policies stopped at ~12 N (the bar and
  the force law, F-161); that the D1 can put the key on the keyway's axis and seat it without learning from a known
  box (F-164); and a staged demo in which every piece runs (`demos/combiner_key`).
- *Does not show*: the whole sequence. From the look's estimate the key did not seat (0/12; the tag's depth is 3-11 mm
  off and the push is position-aimed); the leg policy cannot take over from the walking policy's stance; and its
  ranking holds only on the leg actuator it trained on -- on the decoupled stack's DCMotor legs the force-aware
  policy falls from 84 to 3 of 108 (F-165). Nothing is on PhysX contact for the turn (the training's plant draws the
  key), the lock's numbers are invented, the key is simulated mirrored (the STL points the wrong way for the turn
  asked for), and nothing was run on the dog.
- *Next* (for Lukas to choose): train the leg policy on an actuator model checked against the Go2's motors and from
  the states (not only the stances, F-167) the sport mode leaves; a force-limited push (or a touch to measure depth) for the insertion; print the
  key mirrored.

### 2026-10-21 · Which leg motor model the key policies hold on, and starting from the sport mode's stand

Lukas, on F-165's "Isaac Lab's Go2 motor model (23.5 N·m knees instead of 45)": "I want to make sure the model we are
training on is as accurate as possible. Early in this thesis I cross checked with the rima repo which I know works";
then "Yes, run those tests. I want my sim to real to not fail so it would be good to know". During the runs, with a
photo of the dog in sport mode: "The classic mode gait always stands still with the feet almost directly under the
shoulders like so: 41x28cm spacings. Would it be able to transition into the policy", and "Is the force aware policy
ready to test on the real robot?"

**The leg models here** (all kp 25 / kd 0.5, action scale 0.25 -- rima's deploy node's defaults, which are Isaac Lab's
`UNITREE_GO2_CFG`):

| model | hip, thigh (N·m) | knee (N·m) | with speed | used by |
| --- | --- | --- | --- | --- |
| Isaac Lab's stock `DCMotor` | 23.5 | 23.5 | linear to 0 at 30 rad/s | rima's policies, P2Dingo's walking policy, the decoupled demos |
| unitree_rl_lab's GO2HV envelope (F-005) | 20.2 / 23.4 | the same | flat to 13.5 rad/s, 0 at 30 | the position-only task |
| UniFP's ideal PD (the URDF's limits) | 23.7 | 45.43 | none | the leg and key policies |

Unitree's URDF gives the calf 45.43 N·m at 15.70 rad/s and the hip and thigh 23.7 at 30.1, the same factor 1.917 in
torque and speed: the hip's motor behind a geared knee. Unitree's Go2 page gives ~45 N·m as the largest joint motor's
peak ([unitree.com/go2](https://unitree.com/go2)). The stock model and unitree_rl_lab give the knee the hip's limits.
rima validates the interface (gains, joint order, stance, action scale); a walking policy never asks a knee for more
than ~23 N·m, so it does not test the knee's ceiling.

**Added.** `unifp_train/leg_actuators.py` (`--leg_actuator` unifp, dcmotor, knee23, speedcurve, knee23curve,
dcmotor_armature, unitree, unitree_geared; `--leg_armature`, `--leg_knee_armature`; `--leg_delay_steps`, 0 to N
physics steps of 5 ms per environment); `motor_model.GO2_HV_KNEE` (labelled derived: GO2HV with the URDF's 1.917);
`leg_eval.TORQUE_FIELDS` (the legs' applied and demanded joint torques and speeds every policy step -- the last physics
substep, so a peak between samples can be missed -- scored as peaks, time past 23.5 N·m, time clipped); the run record's
`leg_joints_loaded` (armature, friction and limits as PhysX holds them); `demos/combiner_key/classic_stance.py`;
`sysid/go2_knee/` (below); `unifp_train/leg_actuator_plot.py`; `tests/test_leg_actuators.py` (the knee ratio read off
the URDF in torque and speed; 5 tests).

**Runs.** `run_unifp_train.py leg_eval --task leg --leg_object key --leg_variant <v> --checkpoint <model> --headless
--small_physx --leg_actuator <model> [--leg_delay_steps 2] [--leg_armature A --leg_knee_armature K] [--spawn_states
demos/combiner_key/assets/classic_stance_<h>cm.json --eval_spawn_states]`, three at a time from detached queues
(`logs/unifp_train/actuator_study/`), 09:40-11:27; seed 1's and seed 2's finals and the seed-2 walker fine-tune
(`model_2104`, F-167). 97 runs, all recorded ("Key eval, <policy>, legs: <model>", "Key eval from the classic stand at
<h> m, <policy>"); none failed. The trained model's re-runs reproduce F-166 and F-167 exactly (84, 106, 102, 20, 23).
`figures/key_leg_actuators.png`.

**Keys unlocked** (foot back down, no damage, no fall; of 108):

| legs | force s1 | force s2 | fine-tune | position s1 | position s2 |
| --- | --- | --- | --- | --- | --- |
| trained: ideal PD, armature 2e-4 | [84](#/week/6/run/20261020T225436_leg_eval_seed1_key_act_force_s1_unifp) | [106](#/week/6/run/20261020T230912_leg_eval_seed1_key_act_force_s2_unifp) | [102](#/week/6/run/20261020T232518_leg_eval_seed1_key_act_force_s2walker_unifp) | [20](#/week/6/run/20261020T230252_leg_eval_seed1_key_act_position_s1_unifp) | [23](#/week/6/run/20261020T231723_leg_eval_seed1_key_act_position_s2_unifp) |
| knee capped at 23.5 | [84](#/week/6/run/20261020T225718_leg_eval_seed1_key_act_force_s1_knee23) | 106 | | 20 | 23 |
| DCMotor speed curve, URDF peaks | [84](#/week/6/run/20261020T225723_leg_eval_seed1_key_act_force_s1_speedcurve) | 106 | | 20 | 23 |
| knee cap + speed curve | [84](#/week/6/run/20261020T232931_leg_eval_seed1_key_act_force_s1_knee23curve) | 106 | 103 | 20 | 23 |
| stock DCMotor (armature 0) | [**3**](#/week/6/run/20261020T233521_leg_eval_seed1_key_act_force_s1_dcmotor_rerun) | [104](#/week/6/run/20261020T231113_leg_eval_seed1_key_act_force_s2_dcmotor) | [94](#/week/6/run/20261020T232548_leg_eval_seed1_key_act_force_s2walker_dcmotor) | 20 | 20 |
| stock DCMotor, armature 2e-4 | [**84**](#/week/6/run/20261020T233521_leg_eval_seed1_key_act_force_s1_dcmotor_armature) | | | | |
| GO2HV envelope, knee as hip | [85](#/week/6/run/20261020T230001_leg_eval_seed1_key_act_force_s1_unitree) | 106 | 102 | 22 | 20 |
| GO2HV envelope, geared knee | [82](#/week/6/run/20261020T225436_leg_eval_seed1_key_act_force_s1_unitree_geared) | 106 | 104 | 21 | 20 |
| trained + 0-10 ms delay | [67](#/week/6/run/20261020T230012_leg_eval_seed1_key_act_force_s1_unifp_d2) | 107 | [61](#/week/6/run/20261021T002503_leg_eval_seed1_key_act_force_s2walker_unifp_d2) | 16 | 29 |
| geared + 0-10 ms delay | 66 | 107 | [80](#/week/6/run/20261021T002508_leg_eval_seed1_key_act_force_s2walker_unitree_geared_d2) | 24 | 27 |
| trained, armature 0.003 | 92 | 103 | 102 | 37 | 31 |
| trained, armature 0.01 | [96](#/week/6/run/20261020T235625_leg_eval_seed1_key_act_force_s1_unifp_arm010) | 102 | 105 | [60](#/week/6/run/20261021T001332_leg_eval_seed1_key_act_position_s1_unifp_arm010) | 60 |
| trained, armature 0.01, knees 0.037 | 81 | 88 | 106 | [69](#/week/6/run/20261021T001345_leg_eval_seed1_key_act_position_s1_unifp_arm010k037) | 67 |
| geared, armature 0.003 | 91 | 102 | 105 | 35 | 32 |
| geared, armature 0.01 | 92 | 101 | 107 | 65 | 58 |
| geared, armature 0.01, knees 0.037 | 83 | 88 | 107 | 68 | 66 |

- **F-165's 84 -> 3 was the armature.** The stock group sets no armature, so the legs keep the robot file's 0.0
  (`leg_joints_loaded`); every other model sets the trained 2e-4. The stock group with only that changed gives 84; its
  knee cap, its speed curve and both together give 84 each. PhysX's velocity limit is the URDF's (30.1 / 30.1 /
  15.70 rad/s) in every model. On the stock group seed 1's policy, which crouches the base to 0.21 m in its first
  0.4 s and then rears on three legs, sinks instead (0.197 to 0.170 m over 1.2-2.0 s, the support knees clipped at
  23.5 N·m with 24-32 demanded), and 103 of 108 end at ~2.0 s with the foot striking the key (median 184 N). Seed 2's,
  clipped 0.58 s an episode, unlocks 104. Why zero armature breaks one and not the other is not separated. F-165's
  reading (the estimator mistaking a different actuator for a push) was wrong; its status now says so.
- **Delay** (0-2 physics steps, `motor_model.GO2_LEG_DELAY_PHYSICS_STEPS`' estimate; the policies trained with none):
  seed 1 84 -> 67, the fine-tune 102 -> 61 (geared knee 104 -> 80), seed 2 none.
- **Armature.** UniFP's 2e-4 kg·m² is what they trained with. unitree_rl_lab gives the same GO2HV motor 0.01 where it
  drives the H1's joints (`UNITREE_H1_CFG`; it leaves the Go2's at the file's 0); 0.037 is 0.01 x 1.917² for a geared
  knee. Rising armature lifts position-only from 43 of 216 (both seeds) to 68, 120 and 136, while force-aware goes
  190, 195, 198, then 169; the fine-tune stays at 102-107. F-166's comparison rests on a parameter neither variant
  trained near; its status now says so.
- **What the legs carry** (the trained model; F-169): the pressing leg's joints at most 8-11 N·m (hip, thigh) and
  11-19 N·m (knee) for the force-aware policies -- with more than 40 N on the foot, seed 1's knee carries a median
  1.9 N·m, the leg pushing nearly straight. The standing legs do the work: the force-aware policies crouch to
  0.215-0.219 m, then rear 34-35° nose-up to ~0.28 m on three legs for ~11 s, the largest support knee at a median
  12-14 N·m, above 20 N·m for 6.7% (s1) and 11% (s2) of that time, peaking at 28-33 N·m. The fine-tune rears 26° to
  0.30 m (peak 23.9). The rear-up is the key's height: its platform is at ~0.42 m, above the front hips standing.

**From the classic stand** (F-170). `classic_stance.py` solves the legs for the feet's sphere centres at 41 x 28 cm,
level (its kinematics reproduce `NOMINAL_FEET_B` to 2.5 mm): the feet 4.3 cm (front) and 2.9 cm (rear) from the
trained places (44.8 x 34 cm under 0.272 m). The stand's base height is not measured here (the door demo's robot
program reports sport mode's `body_height_m`), so four; every episode starts there at rest, jittered 0.03 rad, on the
trained legs:

| policy | trained stance | 0.28 m | 0.30 m | 0.32 m | 0.34 m |
| --- | --- | --- | --- | --- | --- |
| force s1 | 84 | [73](#/week/6/run/20261020T233417_leg_eval_seed1_key_classic_force_s1_28cm) | 75 | 50 | [20](#/week/6/run/20261020T233751_leg_eval_seed1_key_classic_force_s1_34cm) |
| force s2 | 106 | 91 | 80 | 93 | 65 |
| fine-tune | 102 | [97](#/week/6/run/20261020T234136_leg_eval_seed1_key_classic_force_s2walker_28cm) | [99](#/week/6/run/20261020T234402_leg_eval_seed1_key_classic_force_s2walker_30cm) | [96](#/week/6/run/20261020T234409_leg_eval_seed1_key_classic_force_s2walker_32cm) | 74 |
| fine-tune, geared knee | 104 | | 98 | 97 | |
| position s1 | 20 | 24 | 24 | 22 | 21 |
| position s2 | 23 | 12 (50 fell) | 14 (27 fell) | 13 (20 fell) | 7 (58 fell) |

**The knee on the dog** (`sysid/go2_knee/run_on_dog.sh --mass_kg <weighed>`, not run): a read-only ROS 2 logger of
LowState in P2Dingo's image -- every joint's `tau_est`, `q`, `dq`, the IMU and foot forces to csv, the knees' peaks and
time past 23.5 N·m, and a unit check while standing still (the knee torque the weight needs from the joint angles,
pitch and an equal share per foot, against `tau_est`: ~1 if it is the joint's torque, ~0.52 if the motor's before the
knee's reduction). With the dog's weight it also checks the simulator's mass.

**Docs corrected:** `unifp_train/README.md`, `demos/combiner_key/README.md` and `run_combiner_key.py --legs` had the
DCMotor drop as the actuator's torque; they now give the armature (F-168).

**What this shows, and what it does not.**
- *Shows* (simulation): the key policies do not hinge on the knee's torque limit or the speed curve -- every torque
  model tried, at the trained armature, moves the force-aware results by at most 2 keys; they hinge on the legs'
  armature and the control loop's delay, neither of which is measured on the dog, and the force-aware lead over
  position-only shrinks at a realistic armature. The fine-tune can start from the sport mode's stand, at rest, up to a
  0.32 m base.
- *Does not show*: what the real Go2's knee, armature, delay or classic-stand height are (nothing measured); how
  policies trained on each model would rank (these are frozen policies off their training conditions); the switch
  out of sport mode (only a start at rest); anything beyond the analytic key plant.
- *Lukas's "ready for the real robot?"*: not yet. Nothing runs the leg policy on the dog (its observation from
  LowState, 50 Hz LowCmd, the task layer on the robot); it trained with no noise, a fixed mass and ground friction, no
  delay, and an armature far below the likely real one, and the delay costs the fine-tune 24-41 keys; the turn is the
  analytic plant with invented lock numbers; the rear-up on three legs with the arm on top is untested for falls; 6 of
  108 fine-tune episodes pass 120 N on the key. Proposed order: the knee logger and a weighing; the policy in shadow
  mode on the dog (real observations, nothing sent); a fine-tune with delay, noise and mass / friction / armature
  randomised on the geared model, from the classic stand; then free-space foot lifts on the dog, harnessed, with
  cut-offs; then a key on a test rig with a weak lock.

### 2026-10-21 · The leg policy's shadow mode for the dog

Lukas: "Can you build the shadow mode runner" (step 2 of the path proposed after "Is the force aware policy ready to
test on the real robot?": the policy on the dog's real observations while sport mode stands, sending nothing).

**What it is** (`demos/combiner_key/robot/`, its README has the procedure and what a good record looks like):
- `export_policy.py` (PC): a checkpoint to one `.npz` holding the encoder, actor and decoder weights and every constant
  of the interface (defaults, scales, joint names, the goal centre and footfall, the arm's rest pose, the leg
  kinematics), read from `interface`, `leg_cfg` and `observations`. It refuses to write unless numpy reproduces the
  checkpoint. Exported: the walker fine-tune (seed 2, `model_2104`, F-167) to
  `checkpoints/leg_key_force_s2walker_model_2104.npz` (6.2 MB, sha256 `783df72c…`, in `SHA256SUMS` and
  `checkpoints/README.md`); actions to 3.3e-6, estimates to 1.3e-6 on 64 random stacks.
- `leg_policy.py` (dog, numpy only): the 76-wide frame from Unitree-ordered joints, the IMU's quaternion and gyro and
  the D1's angles; the zero-started 32-frame history; the network; the targets in Unitree's order; the waiting
  stage's command (the foot on its footfall, no force); the estimates decoded to SI.
- `shadow.py` (dog, ROS 2 in P2Dingo's image): `/lowstate` in, and the arm link's feedback, listened to only. Every
  20 ms, on its own schedule rather than a ROS timer, it records the policy's action, its targets, the torque
  kp 25 / kd 0.5 would ask at the measured joints, its estimates and the timing; it prints a status each second. It
  publishes nothing.
- `run_shadow_on_dog.sh`: the arm link with `--observe` (feedback only) and the node, as the door demo's launcher.
- `check_shadow.py` (PC): `--sim` and `--dog`, below.
- `run_unifp_train.py leg_eval --leg_shadow_dump N` records the first N episodes' raw state, commands, frames,
  actions and actuator targets for `--sim`.

**Checks** (F-171):
- *A recorded Isaac rollout*: `run_unifp_train.py leg_eval --task leg --leg_object key --leg_variant force --checkpoint
  <model_2104.pt> --headless --small_physx --leg_shadow_dump 4`
  ([run](#/week/6/run/20261021T013654_leg_eval_seed1_key_shadow_dump_force_s2walker): 102 of 108, as F-168's), then
  `check_shadow.py --sim <run>/shadow_dump.npz` (its report in the run). The dump's joints are in Isaac's order,
  mapped to Unitree's by name. Over 4 episodes and 3,596 steps from each reset: frames 1.0e-7, actions 1.1e-6,
  joint targets 1.2e-7 rad, the waiting command 4.9e-7. The first check found one difference: the frame a reset
  returns has all-zero commands (the environment builds it before any step writes them), and from the second frame
  on the commands are the footfall. The node now sends zeros first, as the evaluation does.
- *The node under ROS 2* (a separate shell with `/opt/ros/humble` and `~/unitree_ros2`, not the Isaac environment;
  `ROS_LOCALHOST_ONLY=1`, domain 77) against a fake 500 Hz LowState: the classic stand at 0.30 m, a 0.01 rad sway
  and a 1° pitch sway. Two development runs in the scratchpad showed a threaded BLAS taking single steps to 30 ms
  (inference p99 29.8 ms, 555 of 600 steps); with one BLAS thread:
  [20 s run](#/week/6/run/20261021T014808_leg_shadow) -- 1001 steps, period median / p99 20.0 / 20.2 ms, state age p99
  2.2 ms, inference median / p99 0.52 / 1.11 ms. `check_shadow --dog` on it: frames against the training code's
  `interface.single_obs` (joint order by `interface.permutation_from`) 3.5e-18, actions against the checkpoint in torch
  1.2e-6, the stand from the joints 0.41 x 0.28 m under 0.30 m, the still-robot prediction within 0.012 rad RMS (the
  sway's size), the estimates as a still robot's. [With the foot asked up](#/week/6/run/20261021T014859_leg_shadow)
  (`--foot_goal 0.30 -0.17 0.12`, 10 s): 1.8e-6, 0.045 rad RMS. The FR knee's target moves 0.6 rad and ~15 N·m is asked
  at the knee. The FR foot at the targets sits 8 cm behind and ~1 cm below the held foot with either goal (a standing
  leg's targets carry its support torque), so the README no longer says the targets show the foot's direction.
- *Unit tests*: `tests/test_leg_shadow.py`, 10 tests (export of a random actor, frame against `single_obs`, target
  order, waiting command, estimates, foot kinematics, the node offline on a fake still robot through `check_shadow`,
  a source scan for anything that sends, no torch on the dog's side).
- The export's own check caught one bug while building: rsl-rl's MLP registers one ELU module between every pair of
  layers, so `module.modules()` listed it once and the layer walk refused. It now walks the Sequential in order.

**What the estimator says about a still classic stand** (the fake, which is the still-robot model): base velocity
(-0.12, -0.03, -0.16) m/s, the FR foot at (0.23, -0.18) m against the joints' (0.205, -0.14), forces under 2 N. Even
standing still, the estimator puts the base sinking and the foot near its trained footfall, as at the start of a
training episode, when the robot drops into its crouch (F-169). On the dog, the comparison is with this, not with zero.

**What this shows, and what it does not.**
- *Shows* (software): the dog-side runner is the training policy, step for step, and holds 50 Hz under ROS 2 on the
  PC without sending anything.
- *Does not show*: anything about the dog. The firmware's motor order, the IMU's frame and the quaternion's order are
  rima's node's, and nothing here has run on the robot. The dog's CPU is not timed. It is open loop: the policy's
  targets never act. The handover out of sport mode is not part of it.
- *For Lukas*: `run_shadow_on_dog.sh --policy checkpoints/leg_key_force_s2walker_model_2104.npz --seconds 90`, with
  the robot standing in sport mode and the arm's E-STOP pressed in the website (so the arm link can listen). Push the
  nose down once, press the FR shoulder once, then `d1fetch` and `check_shadow.py --dog`. The record also gives the
  classic stand's base height from the encoders.

### 2026-10-21 · The key turned from 45 to 90 degrees down

Lukas: "Is it possible to make the policy open the door assuming the key needs to turn from 45 to 90 degrees? Would
that require retraining?" -- then "I mean the key starts at 45 and goes to 90, not continues from the previous
threshold". Read as: the key starts 45 degrees below horizontal and is turned clockwise to 90 (straight down); the
door unlocks only at the end, taken as 85 degrees, 5 short of a 90-degree stop. The trained turn is 45 up to 45 down,
unlocking at 30 down.

**Added** to `run_unifp_train.py` (train and `leg_eval`, `--leg_object key`; all in the run record):
`--key_turn_deg START STOP UNLOCK` (sets `leg_cfg.KEY_THETA_*`, which the plant, the approach, the task layer and the
frozen plan read at run time), `--key_overdrive_deg` (the layer's path past the stop, `OVERDRIVE_RAD`, 6 as trained)
and `--key_hold_s` (the layer's hold at the end, `HOLD_S`, 2 s as trained).

**Runs** (`leg_eval --task leg --leg_object key --leg_variant <v> --checkpoint <final> --headless --small_physx
--key_turn_deg 45 90 85 [--key_overdrive_deg -3] [--key_hold_s 0.3]`; all 13 recorded as "Key eval, turn 45->90°
down ..."), keys unlocked, foot back down, no damage, no fall, of 108:

| policy | zero-shot | path ends 3° short of the stop | and a 0.3 s hold |
| --- | --- | --- | --- |
| force s1 | [16](#/week/6/run/20261021T021850_leg_eval_seed1_key_turn4590_force_s1) | [11](#/week/6/run/20261021T022428_leg_eval_seed1_key_turn4590_force_s1_od-3) | [9](#/week/6/run/20261021T023001_leg_eval_seed1_key_turn4590_force_s1_od-3_hold03) |
| force s2 | [1](#/week/6/run/20261021T021855_leg_eval_seed1_key_turn4590_force_s2) | [2](#/week/6/run/20261021T022433_leg_eval_seed1_key_turn4590_force_s2_od-3) | |
| walker fine-tune | [3](#/week/6/run/20261021T021900_leg_eval_seed1_key_turn4590_force_s2walker) | [17](#/week/6/run/20261021T022438_leg_eval_seed1_key_turn4590_force_s2walker_od-3) | [33](#/week/6/run/20261021T022958_leg_eval_seed1_key_turn4590_force_s2walker_od-3_hold03) |
| position s1 | [11](#/week/6/run/20261021T022132_leg_eval_seed1_key_turn4590_position_s1) | [6](#/week/6/run/20261021T022709_leg_eval_seed1_key_turn4590_position_s1_od-3) | |
| position s2 | [12](#/week/6/run/20261021T022137_leg_eval_seed1_key_turn4590_position_s2) | [10](#/week/6/run/20261021T022714_leg_eval_seed1_key_turn4590_position_s2_od-3) | [27](#/week/6/run/20261021T023004_leg_eval_seed1_key_turn4590_position_s2_od-3_hold03) |

- *The turn itself is reached*: zero-shot the walker fine-tune unlocks 100 of 108 and position s2 94. Seed 2's
  final never reaches the pad in 46 (the pre-contact point is lower and further right than any it trained at).
- *The force is what fails*: the walker fine-tune passes 120 N in 105 of 108. Every first contact past it is while
  engaged with the key at its stop (median 90.8 degrees), mostly in the layer's 2 s hold (79), with 15-20 N commanded
  and a median peak of 142 N. At 90 degrees the pad's face is vertical, and a foot trained to press down on it pushes
  it into a rigid stop (a reading, not a test). Ending the path short of the stop and shortening the hold bring it to
  33; 55 still pass 120 N and 16 stall short of 85.
- The base rears about as on the trained turn (24 degrees nose-up against 26) although the pad now starts at
  0.31-0.38 m instead of 0.45-0.52.

**Answer to Lukas (F-172):** possible, but it needs retraining: a fine-tune on the new turn from the current checkpoints
(`train ... --key_turn_deg 45 90 85 --key_hold_s 0.3 --resume_from <model.pt>`). F-167's fine-tune from the walker
stances took 500 iterations, about half an hour. Not started: it is the point to fold in F-168's delay and armature
randomisation, which is Lukas's call.

### 2026-10-21 · The key policy in shadow mode on the real dog

Lukas pulled `cbeb660` to try the combiner key on the real robot, then: "can you add the helpers and give me the
commands to run the shadow and see visualisation", and after the run: "the rviz2 window showed nothing but I pressed
on the nose and right shoulder anyway".

**Helpers** (in Lukas's `~/.bashrc`, not the repo): `d1update` now also copies `checkpoints/*.npz` (the dog's sparse
clone has no `checkpoints/`; the `.pt` files are not sent); `d1dogshadow [args]` runs
`demos/combiner_key/robot/run_shadow_on_dog.sh --policy checkpoints/leg_key_force_s2walker_model_2104.npz --seconds 90`
on the dog over SSH (later arguments override); `d1fetch` also copies `logs/leg_shadow/` back. Checked with SSH
stubbed and a dry-run `rsync` against the dog (at 192.168.1.213, its WiFi dongle; its checkout was `81b9c4a`, with no
`demos/combiner_key/` until `d1update`).

**Run** ([record](#/week/6/run/20261021T044358_leg_shadow)): `d1update`, then `d1dogshadow` with the robot standing in
sport mode, the arm folded, P2Dingo's arm bridge disconnected (the arm link's log: `"mode": "DISCONNECTED"`, the arm at
-1.0, -89.8, 89.9, 2.5, 11.2, -3.5 deg, listening only). Lukas pushed the nose down once (18.2-26.0 s) and pressed the
front-right shoulder three times (43.3-45.8, 46.8-48.3, 57.7-59.5 s). The `--foot_goal` run was not done. On the PC:
`d1fetch`; `check_shadow.py --policy checkpoints/leg_key_force_s2walker_model_2104.npz --dog
logs/leg_shadow/20261021T044358_leg_shadow --out <record>/check_shadow.json` (no `--checkpoint`: `model_2104.pt` is on
the machine that trained it, `/home/victor/D1Training/logs/...`, so the torch comparison is skipped; the export
matched numpy to torch at 3.3e-6); and a new `demos/combiner_key/robot/plot_shadow.py` for
[`figures/leg_shadow_dog_20261021.png`](figures/leg_shadow_dog_20261021.png).

RViz (`d1view`, a screenshot Lukas pasted in the chat, not in `screenshots/`) showed only the grid and "Global
Status: Error": its robot model reads `/combiner_door/robot_description`, which only the door program publishes, and
the shadow node publishes nothing. Nothing was wrong; the shadow run has no live view beyond its printed status.

**The pipeline on the dog** (F-173):
- 4,472 steps in 90.0 s, one reset (the start); period median / p99 20.0 / 20.7 ms, `/lowstate` age p99 1.6 ms,
  inference median / p99 3.1 / 4.6 ms on the dog's CPU (0.5 / 1.1 on the PC); the arm measured in every step.
  The dog's side of the leg delay, the state's age plus inference (the actor and the estimator's decoder together):
  median 3.4, p99 5.3, max 7.5 ms. `motor_model.GO2_LEG_DELAY_PHYSICS_STEPS` (0-10 ms, estimated) covers this and
  the LowCmd transport and motor driver, which are not measured; its 0 ms end is below what the dog's side alone takes.
- Frames against the training code's `interface.single_obs` from the recorded raw state: 1.4e-17. The dog's numpy
  actions against the PC's numpy on the same stacks: 2.7e-15.
- Against a still robot in the record's first-second pose: joint targets within 0.013-0.078 rad RMS (largest the
  front calves, which move with the pushes; max 0.28 rad, FR calf); the estimator as the still model's (base velocity
  (-0.11, -0.05, -0.31) against (-0.11, -0.05, -0.33) m/s, FR foot (0.216, -0.179) against (0.223, -0.181) m).

**The conventions taken from rima's node, checked by the pushes:**

| push | roll / pitch as the policy sees them | knee `tau_est`, N·m (FR, FL, RR, RL) | foot force sensors |
| --- | --- | --- | --- |
| nose down | pitch -1.7 → +3.9°, roll -2.0 → -2.9° | 4.0, 4.0, 6.3, 6.3 → 11.4, 9.1, 7.3, 10.8 | FR 36 → 64, FL 39 → 54 |
| FR shoulder (three) | roll -1.8..-2.4 → +1.6..+2.0°, pitch → +2.6..+3.5° | FR 6.3-7.1 → 12.4-13.2; the others at most 9.0, RL down to 4.9-5.6 | FR 38-45 → 77-84; RL 41-44 → 24-28 |

- Pitch is positive nose down and roll positive right side down, as UniFP's `euler_from_quat` gives them.
- FR is the first leg in both `motor_state` and `foot_force`: pressing the front-right shoulder raises the first
  knee's torque and the first foot's force most and unloads the diagonal leg (RL).
- The quaternion is w, x, y, z: the roll and pitch the policy computes from it equal the firmware's own `rpy` (0.000°
  apart in every step).
- The gyroscope is the body rate in the same frame: its x and y correlate with the rates of the policy's roll and
  pitch at 0.91 and 0.94.

**The sport mode's stand from the encoders** (the URDF's kinematics, `check_shadow`'s `stand_from_joints`, each quiet
window): feet 38.7 x 31.7 cm, unchanged by the pushes; the base 0.333 m above the feet before the first push, 0.319 m
after them. The rear-left foot sits 18.3-19.4 cm out from the centre line, the other three 13.7-16.0 cm. Lukas's tape gave
41 x 28 cm. The 2-4 cm disagreement is not resolved: the stand as sport mode left it, how the tape was laid, or a hip's
zero (the rear-left's offset would be ~0.1 rad). Level: roll -2.1°, pitch -1.7° at the start (the floor not checked).

**What the policy would ask** (open loop; the targets never act): from the stand the front-right foot's targets sit
3.7-5.5 cm behind and 0.6-1.5 cm below the held foot (the PC's fake gave 8 cm behind, ~1 cm below), but the other three
sit above their feet (FL 2.4-3.5 cm, RR 5.4-7.3 cm, RL 1.2-2.3 cm). The knee torques kp 25 / kd 0.5 would ask: FR +0.5, FL -3.0,
RR -8.5, RL -7.6 N·m at the end (-0.3, -4.3, -11.1, -7.6 at the start), where sport mode holds +6.4 to +7.7. With an
empty history the policy asks to fold three knees, which reads as the drop into its crouch at an episode's start
(F-169; the estimator also reads the base as sinking at 0.31 m/s). This is a reading of open-loop targets: what the legs
would do once the policy acts is not shown.

**What this shows, and what it does not.**
- *Shows* (on the dog): the runner holds 50 Hz on the dog's CPU beside P2Dingo's containers and builds the training
  code's observation from the real robot. The motor order, the IMU's frame, the quaternion's order and the gyroscope
  as the body rate, all of which F-171 took on trust, hold. The arm link observes the D1 without commanding it.
- *Does not show*: anything closed loop; the handover out of sport mode; the lifting regime on the real observation
  (the `--foot_goal` run is still to do); the foot sensors' units (raw); the stand's spacing against the tape.
- *Next*: the `--foot_goal 0.30 -0.17 0.12` run; tape the stand again (front-back, and each foot's distance from the
  centre line, the rear-left especially); simulate F-170's classic-stand start at the measured 0.32-0.33 m and spacing.

**The second run, the foot asked up** ([record](#/week/6/run/20261021T050842_leg_shadow);
[`figures/leg_shadow_dog_20261021_foot_goal.png`](figures/leg_shadow_dog_20261021_foot_goal.png)): `d1dogshadow
--foot_goal 0.30 -0.17 0.12`, 90 s, the robot not touched. Lukas: "this didn't move the arm". Nothing in shadow mode
moves anything: the arm link ran with `--observe` (its log: the bridge disconnected, the arm at -0.2, -89.9, 90.0 deg
throughout) and the leg policy has no arm output. The arm only takes targets from the door demo's robot program
(`run_on_dog.sh --live`); the key has no robot program yet, and its insertion by the arm is simulation only (F-164).
- 4,475 steps, one reset; period median / p99 20.0 / 20.6 ms; state age p99 0.9 ms; inference 3.1 / 4.5 ms. Frames
  against the training code 3.5e-18, numpy dog against PC 2.9e-15; against a still robot, targets 0.004-0.069 rad RMS
  and the estimates the same to the third decimal (F-174).
- The stand this time: 38.4 x 27.8 cm under 0.337 m, pitch -1.35°, roll -0.4°; the knees' `tau_est` 5.8, 8.0, 11.0,
  11.7 N·m (FR, FL, RR, RL) against 7.7, 7.1, 7.3, 6.4 at the end of the first run. The width matches the tape now,
  so the first run's 31.7 cm was mostly the stand sport mode took then. The rear-left foot is still 2.3-3.3 cm further
  out than the others (15.9 cm against 12.6-13.6); front-back is still 2.6 cm short of the tape.
- The lift goal, open loop: the FR knee's target cycles 0.42 rad peak to peak at 4.3 Hz (the would-be knee torque
  -3.5 to +6.5 N·m), growing over the first ~15 s; a still robot in the same pose cycles the same (0.40 rad), so it is
  the policy's response through its own action history to a foot that never lifts, not the dog's sensors. The PC's
  fake LowState (classic stand at 0.30 m) gave the FR knee target a median 0.22 rad off (max 0.79), 5.6 N·m. What
  the leg would do closed loop is not shown.
- *The loop stalls*: both runs have one 0.18 s gap at 30.2 s and one 0.36-0.40 s gap at 60.3-60.4 s after the first
  LowState (no other step over 30 ms). Tied to the node's start, not the clock (the runs started 25 min apart), and
  the second gap twice the first. Not diagnosed: the node's own Python (the record grows every step) or the LowState
  stream. Harmless here; a 0.4 s stall in a program that commands the legs would not be (Issues).

### 2026-10-21 · The shadow loop's stalls: the record saved inside the loop

Lukas: "can you investigate this" (the two stalls in both dog runs, F-174).

**Cause** (F-175): `shadow.py`'s loop called `shadow.save()` every 30 s, which writes the whole record so far
(`np.savez_compressed` of every step, then the summary, which runs the foot kinematics and the estimate decoding per
step in Python) before the loop can step again. Its cost grows with the record. Timed on the dog's CPU, in
`p2dingo-web` as the node runs, on the second run's real rows (`time_save.py`, kept in the
[before run's record](#/week/6/run/20261021T051927_leg_shadow)):

| steps | `np.array` | `savez_compressed` | summary | `save()` | the stall on the dog |
| --- | --- | --- | --- | --- | --- |
| 1,500 (30 s) | 9 ms | 104 ms | 39 ms | 150 ms | 178-185 ms |
| 3,000 (60 s) | 18 ms | 206 ms | 77 ms | 302 ms | 359-402 ms |
| 4,500 (90 s) | 25 ms | 307 ms | 114 ms | 449 ms | (the run ended) |

The stall is `save()` plus the wait for the next 20 ms step. On the PC `save()` takes 49 / 98 / 147 ms. With a fake
500 Hz LowState on the PC (ROS 2 Humble only, `ROS_LOCALHOST_ONLY=1`, domain 77; the 2026-10-21 fake with this PC's
paths), the unfixed node stalls 40 ms at 29.6 s and 83 ms at 59.7 s
([before](#/week/6/run/20261021T051927_leg_shadow)): it is the node, not the LowState stream.

**Fix** (`demos/combiner_key/robot/shadow.py`): nothing in the loop grows with the run. Each second, right after a
step, `flush()` writes only the steps since the last flush to `parts/<first step>.npz`, uncompressed; the final save is
unchanged (the compressed record and the summary, once, after the loop) and removes `parts/`. SIGTERM and SIGHUP now set
a stop flag (after `rclpy.init()`, which installs its own SIGTERM handler), so `docker stop` ends the loop and the final
save runs; `run_shadow_on_dog.sh` gives it 10 s instead of 2 before SIGKILL.
- On the PC, 65 s ([after](#/week/6/run/20261021T052125_leg_shadow)): 3,251 steps, period median / p99 / max
  20.0 / 20.2 / 20.4 ms, no gap; `parts/` removed.
- SIGTERM at 8 s ([run](#/week/6/run/20261021T052245_leg_shadow)): exit 0, the full record (377 steps). SIGKILL at
  5.5 s (not recorded: it leaves no `run.json`, by design): 5 parts, 252 contiguous steps to 5.02 s.
- The flush on the dog's CPU and disk (`time_flush.py`, the fixed `shadow.py` mounted over the dog's for that container
  only; the dog's checkout is unchanged): 89 flushes of 50 steps, median 2.4 ms, max 2.9 ms, the same at the end of the
  run as at the start. With inference at 3.1-4.6 ms, a step uses under 8 of its 20 ms.
- `tests/test_leg_shadow.py`: a new test that the parts are only the new steps and, concatenated, the saved record;
  29 tests pass with `test_evidence`.
- Not yet run on the dog: the fix reaches it with `d1update`; a shadow run over 60 s there is the check.

While timing on the dog, one copy failed (run from the wrong directory) and Docker created an empty root-owned
directory `/tmp/claude_time_flush.py` on the dog for the missing bind source; removed through a container straight
after. Nothing else was left on the dog.

### 2026-10-21 · The turn Lukas meant, and what his lever can take

Lukas, on the readiness of the trained turn: "lets keep the 0 to -45 degree turn from before"; "Does the policy still
need more training before I can test it on the real robot? I'm talking about the 0 to -45"; "Do we need to retrain with
the delay and armature before even the 45 to -45 is ready to deploy?"; "The key on the robot likely doesnt need that
amount of force, and I don't want to crack my 3d printed lever. Will it be safe?"; and "the lock turned when resting a
roughly half filled 1L water bottle on the lever's platform".

**The misreading.** Lukas's original sentence ("turn it 45 degrees from horixontal (pad facing up) to 45 degrees below
horizontal clockwise ... unlocks ... past the 30 degree mark") is a 45-degree turn from horizontal. It was read as
45 up to 45 down, and every key policy and evaluation since the morning (F-163 to F-172) used that longer turn. F-166's
status now says so. The code's default stays the trained turn; `--key_turn_deg 0 45 30` sets Lukas's.

**Runs** (all `leg_eval --task leg --leg_object key --leg_variant <v> --checkpoint <final> --headless --small_physx`
plus the options below; 49 recorded, titled "Key eval, turn horizontal->45° down ...", "Key eval, force-aware s2
walker fine-tune, legs: ...", "Key eval from the dog's measured stand ..."):
- Lukas's turn (`--key_turn_deg 0 45 30`) zero-shot, five policies, then the three best with `--leg_delay_steps 2`,
  `--leg_actuator unitree_geared --leg_armature 0.01 --leg_knee_armature 0.037`, both, from the classic stand
  (`--spawn_states demos/combiner_key/assets/classic_stance_30cm.json --eval_spawn_states`), and both of those together
  (F-176's table). Force s2: 108 as trained, 89 realistic, 52 from the stand with the realistic legs.
- The trained turn, the walker fine-tune: realistic legs 78, from the classic stand with them 49, a 0-5 ms delay alone
  101 (F-176). With 0-10 ms the extra damage is on the key -- 12 as the foot settles on it, 23 pressing, 7 lifting off --
  with no falls and the stance unchanged; peak median 67 -> 99 N.
- A light lock, new options `--key_eval_lock_nm 0.2 0.3 0.5 --key_eval_detent_nm 0 0.2` (the frozen set keeps its 108
  rows and estimate errors), the layer ending 5 degrees short of the stop with a 0.3 s hold (`--key_overdrive_deg -5
  --key_hold_s 0.3`) and a lower command cap (`--key_max_force_n 8 15`; new): F-177's table. Force s2 at most 37 N in
  the trained conditions; with the stand and the realistic legs 144 N p90, 342 N max. The cap changes nothing: the
  layer already asks only ~10-15 N of a light lock, and the peaks are impacts.
- The dog's measured stand (F-173: 38.7 x 31.7 cm, 0.32-0.33 m), solved symmetric into
  `demos/combiner_key/assets/dog_stand_20261021/` (F-178's table): the walker fine-tune 84-85 on the trained turn, the
  best 52 on Lukas's, 15-24 with 8-17 falls with the realistic legs.

**The lever** (Lukas's STL, measured in mm): a 15 x 20 mm bar on a 10 mm shaft; the platform a **2.5 mm** plate,
35 mm wide and 100 mm long, beside the bar (`geometry.py` and the demo's README said 2 mm: corrected). The layer aims at
~18 mm from the plate's root. Printed-PLA estimates, not a test: the plate ~70-160 N there, less at its edge; the shaft
~3-6 N·m in torsion, which a foot pressing ~0.09 m out on the end stop reaches at 35-65 N. Lukas's lock turned under a
half-full 1 L bottle on the platform: ~5 N at ~0.1 m, so its breakaway is at most ~0.5 N·m.

**Answers given.**
- *More training before the real robot?* Yes, for either turn. On the realistic legs from the sport mode's stand no
  policy is better than 49 of 108 (the trained turn) or 52 (Lukas's turn, at 0.30 m; 15-24 at the measured 0.33 m).
  The dog's measured latency (3.4 ms median before a command leaves) rules out the 0 ms the policies trained with.
  Shadow mode needed none.
- *Safe for the lever?* Not with these policies on the dog. In the trained conditions with a soft end yes (<= 37 N);
  on the dog's conditions the impacts reach 140-340 N. A stronger test print (a 4-5 mm platform is ~3x stronger, a metal
  pin through the shaft) helps whatever the policy.

**What this shows, and what it does not.** *Shows* (simulation): how far the trained policies are from Lukas's turn
on the dog's conditions, and where the force comes from. *Does not show*: the real lever's strength or the real foot's
impacts (the plant's contact stiffness sets the peaks), the motors' part of the delay, or the asymmetric real stand.

### 2026-10-21 · The fine-tune on Lukas's turn

Lukas: "the lock turned when resting a roughly half filled 1L water bottle on the lever's platform. with this
knowledge, continue the training with your proposed fine tune" (the proposal: F-176 to F-178). The bottle: ~5 N at
~0.1 m from the shaft, so the lock breaks away at most at ~0.5 N·m.

**Added** (`run_unifp_train.py`, all in the run record): `--key_lock_range_nm` / `--key_detent_range_nm` (the lock drawn
in training), `--key_damage_n` (`leg_cfg.DAMAGE_N`), `--key_press_limit_n` / `--key_press_weight` (`leg_rewards.press_excess`:
the peak contact past the limit as overlap through the plant's stiffness, cm, so the position variant gets it as it
gets `door_push`), `--leg_delay_min_steps`, `--leg_armature_range` / `--leg_knee_armature_factor` (each environment's
leg armature drawn once at start-up, `Go2D1LegEnv._draw_leg_armature`), `--spawn_states_fraction`.
`demos/combiner_key/sport_stands.py` writes `assets/sport_stands_20261021.json`: 20 sport stands (0.30-0.34 m; the
tape's and the encoders' spacings; the rear-left foot as the others or 3.5 cm out) listed twice, and the walker's 32
stops. A 2-iteration smoke ([run](#/week/6/run/20261021T053524_train_seed2_key_finetune_smoke)) checked every setting
reached the simulator (env 0's armature 0.0052 hip and thigh, 0.0191 knee; the delay 1-3 steps).

**Training** (16:36-18:31, both in parallel, 9.8 GB, ~6.5 s an iteration):

    ./run_unifp_train.py train --task leg --leg_object key --leg_variant <v> --resume_from <seed 2 final> \
        --num_envs 2048 --iterations 1000 --handle_fraction 0.6 --foot_push_n 10 --force_start_iteration 0 \
        --force_ramp_iterations 1 --spawn_states demos/combiner_key/assets/sport_stands_20261021.json \
        --spawn_states_fraction 0.8 --resume_leg_std 0.2 --fixed_learning_rate 1e-4 --entropy_coef 0.002 --headless \
        --seed 2 --key_turn_deg 0 45 30 --key_overdrive_deg -5 --key_hold_s 0.3 --key_lock_range_nm 0.15 0.6 \
        --key_detent_range_nm 0 0.2 --key_max_force_n 6 20 --key_press_limit_n 30 --key_press_weight -4 \
        --leg_actuator unitree_geared --leg_delay_min_steps 1 --leg_delay_steps 3 --leg_armature_range 0.003 0.02

[force](#/week/6/run/20261021T053626_train_seed2_key_ft045_force_s2) from `model_1605`,
[position](#/week/6/run/20261021T053727_train_seed2_key_ft045_position_s2) from `model_1614`. The penalty is small
in the episode sums (-0.006 against +0.15 for the key's progress and unlock, averaged over all episodes), yet the
force-aware run's training-time peak fell from 65-96 N to 40-44 N; the position-only run's fell to ~60 N by ~2000,
then rose to 85-98 N.

**The frozen evaluations** (18:31-18:49, the queue; then the sweep 20:17-20:41): F-179's two tables. The fine-tuned
force-aware policy: 101 / 100 from the dog's stand on the realistic legs, nothing past 60 N, max 35 N; 95-105 across
the sweep's changes within the training ranges; 65 at 20 ms; 2 with UniFP's 2e-4 armature (the whole of its 0 in
"nominal"). The position-only final 30 / 27 (its checkpoint at 2000: 86 / 94).

**Viewers** (Lukas: "can you spin up visualisation so I can see it"): the fine-tuned force-aware policy on Lukas's
turn from the dog's stand on the realistic legs. A first launch with four robots ended after `play`'s default 100 steps
([run](#/week/6/run/20261021T092014_play_seed1_key_ft045_play_force)) -- it read to Lukas as a crash ("can we only show
one at a time"); an interrupted relaunch with four robots ([run](#/week/6/run/20261021T092246_play_seed1_key_ft045_play_force))
was closed, and one robot (row 53) ran until he asked to close it ([run](#/week/6/run/20261021T092301_play_seed1_key_ft045_play_force)).

**Exported** for the dog's shadow runner: `checkpoints/leg_key_force_ft045_model_2604.npz` (numpy against the checkpoint:
actions 4.9e-6; `checkpoints/README.md`, `SHA256SUMS`). `export_policy.py` now writes the training run's own settings into
`meta.trained_on` (it had a fixed description of F-166's legs, right only for the first export).

**What this shows, and what it does not.** *Shows* (simulation): a fine-tune on the dog's conditions turns Lukas's key
gently from the dog's stand and is robust within the ranges it trained on; the force-aware variant holds its lead.
*Does not show*: the real dog -- no commanding program exists; the motors' delay; a real foot on a real PLA lever (the
plant's contact sets the peaks); the switch out of sport mode. One seed per variant.

### 2026-10-21/22 · A general foot-press policy: plates anywhere the front-right foot reaches

Lukas: "I see potential in developing this force aware foot pushing into a policy trained to go in many positions and
directions just like uniFP so it can also be used for pressing etc. You have a day of gpu to train and iterate on
this." UniFP's arm policy is general (any goal in its sphere, any force command); the key policy (F-179) has one
contact at one place. Findings: F-180 (the policy), F-181 (what its task layer needs), F-182 (a limit and a regression).

**Built** (`--task leg --leg_object press`, `unifp_train/README.md`, "Plates anywhere the foot reaches"):
`foot_press_cfg.py`, `foot_press.py` (the FR leg's kinematics, checked against `classic_stance.foot`; `PlateTable`;
`PlateSet`; `PressLayer`), `foot_press_env.py` (`Go2D1FootPressEnv`, a subclass of the leg environment: the key's stance,
robot and return), `foot_press_eval.py` (288 frozen presses), `leg_vis.PlateMarkers`; hooks in `leg_env.py`
(`_start_layer`, `_object_state`, `_draw_objects` / `_place_objects`, a per-environment lift height) with the handle and
key paths unchanged; `tests/test_foot_press.py` (18). In 70% of episodes a flat plate -- rigid (8,000 N/m) or sprung
(400-8,000 N/m over 5-30 mm, then rigid), sticking friction, the knee and head checked against it -- that the layer
approaches, finds by the foot stalling, presses with 1-3 levels of 3-35 N (some with a tangential part, some sliding
2-6 cm), releases, and the foot goes back to its footfall; in the rest UniFP's free-space goal walk with pushes. The
force-aware variant is rewarded for the applied force against the command (`press_force`, its only extra term); both
get the key's contact, slip, knee-and-head and approach terms, and a 45 N press limit.

**Recipe** (all rounds; `logs/unifp_train/press/train_one.sh`): both variants in parallel from the F-179 fine-tune's
finals (force `model_2604`, position `model_2613`), 2048 environments each, ~6.6 s an iteration:

    ./run_unifp_train.py train --task leg --leg_object press --leg_variant <v> --resume_from <checkpoint> \
        --num_envs 2048 --iterations <n> --handle_fraction 0.7 --foot_push_n 15 --force_start_iteration 0 \
        --force_ramp_iterations 1 --spawn_states demos/combiner_key/assets/sport_stands_20261021.json \
        --spawn_states_fraction 0.8 --resume_leg_std 0.2 --fixed_learning_rate 1e-4 --entropy_coef 0.002 --headless \
        --seed 3 --leg_actuator unitree_geared --leg_delay_min_steps 1 --leg_delay_steps 3 --leg_armature_range 0.003 0.02

The frozen evaluation (`logs/unifp_train/press/eval_one.sh <v> <ckpt> <name> real`): the 288 plates on the realistic
legs (geared knee, armature 0.01 / 0.037, 5-15 ms delay) from the dog's measured stand at 0.33 m, damage at 120 N.

**What went wrong first, and what changed** (each run recorded; stopped runs are titled as such):

- Smokes: [crashed](#/week/6/run/20261021T110103_smoke_seed1_press_smoke) (a mask not broadcast in the layer; a batched
  test now covers it), [then ran](#/week/6/run/20261021T110137_smoke_seed1_press_smoke); a 2-iteration
  [training smoke](#/week/6/run/20261021T110230_train_seed3_press_train_smoke).
- Round 1 ([force](#/week/6/run/20261021T110441_train_seed3_press_r1_force_s3),
  [position](#/week/6/run/20261021T110557_train_seed3_press_r1_position_s3), stopped at ~50 iterations): the layer
  called contact in the air -- training engages at the deadline wherever the foot is (12 cm off at first), and a foot
  off its line lags the reference without touching anything. A stall now counts only with the reference within 2 cm
  of the expected face and the foot within 2.5 cm of its line.
- Round 1b ([force](#/week/6/run/20261021T110920_train_seed3_press_r1b_force_s3),
  [position](#/week/6/run/20261021T111035_train_seed3_press_r1b_position_s3), stopped at ~50): held where contact was
  called, the goal makes UniFP's target `goal + (external + command) / 400 N/m` a spring, and a plate of stiffness k_s
  then settles at k_s / (k_s + 400) of the command (71% at 1,000 N/m) -- against the `press_force` term. The
  force-aware layer's goal now follows the foot along the normal while it presses (0.1 s, at most 4 cm in).
- Round 1c ([force](#/week/6/run/20261021T111219_train_seed3_press_r1c_force_s3),
  [position](#/week/6/run/20261021T111335_train_seed3_press_r1c_position_s3), stopped at ~470): its `model_3000`
  [held 1 of 288 and missed 237 approaches](#/week/6/run/20261021T114505_leg_eval_seed1_press_r1c_force_3000_real), the foot
  ~20 cm short of its pre-contact point. The plate table had been drawn from the leg's kinematic sample, uniform in joint
  space: its contact points sat at the reach's outer shell (median 0.31 m from the thigh joint, p90 0.37; the leg is
  0.426 m at full stretch). The table is now drawn uniformly in space (half of it in a box ahead of the foot), 0.13-0.30 m
  from the thigh joint (median 0.26), the waypoint checked too; the free-space goals' yaw widened to -1.2 rad for this
  task, as far round to the right as the plates go. **The frozen set was rebuilt from it (v2, 23:00)**; the runs on v1
  are titled as such and superseded.
- Round 2 ([force](#/week/6/run/20261021T115106_train_seed3_press_r2_force_s3),
  [position](#/week/6/run/20261021T115222_train_seed3_press_r2_position_s3), stopped at ~470): at `model_3000`
  [force 44](#/week/6/run/20261021T122353_leg_eval_seed1_press_r2_force_3000_real) and
  [position 40](#/week/6/run/20261021T122723_leg_eval_seed1_press_r2_position_3000_real) of 288, 55 damaged each. The
  force-aware policy applied ~63% of the command on rigid and sprung plates alike (8 / 16 / 28 N -> 5 / 10.5 / 17.5: the
  goal following works, the shortfall is the policy's); of its 83 peaks past 60 N, 22 were on the return to the footfall
  (the key's path, waypoint then straight down, cut through plates between) and 22 on the approach (the key's 14 cm
  first lift, then on a slant, met low plates); the base reared 32-40 degrees through every stage. Now: the lift goes
  straight up to the waypoint's height, the return goes out to the waypoint, across at its height and straight down, and
  the orientation weight is -4 in this task (-1 in UniFP's).

Zero-shot, the F-179 key policy holds [0 of 288 on v2](#/week/6/run/20261021T115222_leg_eval_seed1_press_r2_keyft_force_real)
(208 approaches missed; it rears ~30 degrees and drops from 0.34 to 0.22 m while it waits: the key's posture); on v1
[1](#/week/6/run/20261021T110321_leg_eval_seed1_press_eval_keyft_force_real) and
[4](#/week/6/run/20261021T111335_leg_eval_seed1_press_r1c_keyft_force_real) with the two layers.

**Round 3** ([force](#/week/6/run/20261021T123300_train_seed3_press_r3_force_s3),
[position](#/week/6/run/20261021T123415_train_seed3_press_r3_position_s3); both from their round-2 `model_3000`, 1600
iterations, 23:40 to ~02:45):

| checkpoint | held | within tolerance | missed | damaged | past 60 N | force bias / MAE (median) | tilt (median max) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| force-aware `model_3400` | [120](#/week/6/run/20261021T130547_leg_eval_seed1_press_r3_force_3400_real) | 134 | 13 | 20 | 26 | -2.3 / 3.9 N | 37 deg |
| position-only `model_3400` | [73](#/week/6/run/20261021T130912_leg_eval_seed1_press_r3_position_3400_real) | 74 | 0 | 25 | 40 | -4.2 / 5.6 N | 34 deg |
| force-aware `model_4000` | [97](#/week/6/run/20261021T135838_leg_eval_seed1_press_r3_force_4000_real) | 139 | 0 | 22 | 26 | -3.4 / 3.6 N | 36 deg |
| position-only `model_4000` | [84](#/week/6/run/20261021T140202_leg_eval_seed1_press_r3_position_4000_real) | 84 | 0 | 0 | 0 (max 38 N) | -5.9 / 6.6 N | 25 deg |

At `model_3400` the force-aware policy applies 7.5 / 15.3 / 22.5 N for 8 / 16 / 28 N (rigid and sprung within 0.5 N
of each other); the return path's peaks fell from 22 to 4. What is left: plates facing down (pushing up), where the foot
loses the plate and comes back onto it at up to 230 N (14 of the 20 damaged); brief contact losses at the start of the
ramp on plates facing up (17 of 33 slips); and the tilt. At `model_4000` the force-aware policy holds more presses
within tolerance (139) but fewer successes, because slips rose from 46 to 116 (42 of them within tolerance). That was the
layer's doing: when contact was called, the force-aware goal jumped from the reference back to the stalled foot,
pulling the target back by the foot's lag (at least 1.2 cm, ~5 N), so the foot let go. The first ramp now starts at the force the
target already asked for (the touch force plus 400 N/m times the lag), which leaves both variants' targets continuous
(`test_target_continuous_when_contact_is_called`). The position-only policy at 4000 is gentle -- nothing past 38 N, the
base at 25 degrees -- and under-presses by 6 N.

**Round 4** ([force](#/week/6/run/20261021T140237_train_seed3_press_r4_force_s3),
[position](#/week/6/run/20261021T140352_train_seed3_press_r4_position_s3); 01:05 to ~04:50; both from round 3's
`model_4000`, 2000 iterations, the ramp fix):

| checkpoint | held | within tolerance | missed | damaged | past 60 N | force bias / MAE (median) | tilt (median max) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| force-aware `model_4400` | [168](#/week/6/run/20261021T143624_leg_eval_seed1_press_r4_force_4400_real) | 168 | 0 | 4 | 5 (p95 33 N) | -2.1 / 3.0 N | 37 deg |
| position-only `model_4400` | [86](#/week/6/run/20261021T143845_leg_eval_seed1_press_r4_position_4400_real) | 87 | 0 | 7 | 12 | -1.8 / 6.5 N | 23 deg |
| force-aware `model_5000` | [208](#/week/6/run/20261021T152810_leg_eval_seed1_press_r4_force_5000_real) | 209 | 2 | 7 | 10 (p95 35 N) | -1.0 / 1.8 N | 30 deg |
| position-only `model_5000` | [67](#/week/6/run/20261021T153131_leg_eval_seed1_press_r4_position_5000_real) | 67 | 3 | 25 | 33 | +2.3 / 8.0 N | 20 deg |
| force-aware `model_5600` | [188](#/week/6/run/20261021T161957_leg_eval_seed1_press_r4_force_5600_real) | 188 | 0 | 0 | 0 (max 55 N) | -0.8 / 2.4 N | 34 deg |
| position-only `model_5600` | [106](#/week/6/run/20261021T162316_leg_eval_seed1_press_r4_position_5600_real) | 106 | 0 | 3 | 3 (max 311 N) | -2.9 / 5.1 N | 20 deg |

The ramp fix took the force-aware slips from 116 to 3 (`model_4400`). Over the hold's last 2 s its force does not
drift on sprung plates (median slope +0.05 to +0.33 N/s, as on rigid ones): the goal following does not ratchet.
At 5000 the plates facing down are the weak kind (28 of 72; the others 57-65). Between checkpoints 600 iterations
apart the counts move by 20-40 for both variants (force-aware 168 / 208 / 188, position-only 86 / 67 / 106): the
position-only policy pressed harder at 5000 (bias +2.3 N, 25 damaged) and softer again at 5600. At 5600 the force-aware
policy put nothing past 55 N on any plate.

**Round 4's finals** (`model_5999`, 03:56; in 1700 s fewer than estimated, ~5.7 s an iteration):

| final | set v2, dog's stand | set v2, settled stance | force bias / MAE | past 60 N (max) | key zero-shot, 0.33 / 0.32 m |
| --- | --- | --- | --- | --- | --- |
| force-aware | [197](#/week/6/run/20261021T165637_leg_eval_seed1_press_r4_force_5999_real) | [207](#/week/6/run/20261021T165640_leg_eval_seed1_press_r4_force_5999_real_settled) | -0.9 / 2.3 N | 5 (153 N), p95 39 N | **107 / 107** of 108, peak median 7 N, p95 12 N |
| position-only | [100](#/week/6/run/20261021T165752_leg_eval_seed1_press_r4_position_5999_real) | [86](#/week/6/run/20261021T165757_leg_eval_seed1_press_r4_position_5999_real_settled) | 0.0 / 5.6 N | 11 (153 N), p95 49 N | 72 / 69 (peak p95 39-40 N) |

Around "set v2, dog's stand", one thing changed at a time (04:25-05:02, beside the replication; the in-press outer
loop kept on, as round 4 trained):

| change | force-aware final (197) | position-only final (100) |
| --- | --- | --- |
| 0-5 ms delay | [229](#/week/6/run/20261021T172538_leg_eval_seed1_press_sweep_r4_force_5999_delay0to5) (0 past 60 N; MAE 2.0 N) | [95](#/week/6/run/20261021T174357_leg_eval_seed1_press_sweep_r4_position_5999_delay0to5) (12 past 60 N; MAE 5.5 N) |
| 15 ms delay | [166](#/week/6/run/20261021T172811_leg_eval_seed1_press_sweep_r4_force_5999_delay15) (16 past 60 N; MAE 2.9 N) | [84](#/week/6/run/20261021T174616_leg_eval_seed1_press_sweep_r4_position_5999_delay15) (1 fell, 7 past 60 N; MAE 6.4 N) |
| 20 ms delay | [143](#/week/6/run/20261021T173035_leg_eval_seed1_press_sweep_r4_force_5999_delay20) (27 missed, 4 fell, 20 past 60 N; MAE 2.8 N) | [43](#/week/6/run/20261021T174836_leg_eval_seed1_press_sweep_r4_position_5999_delay20) (33 missed, 16 fell, 11 past 60 N; MAE 8.0 N) |
| armature 0.003 / knees 0.011 | [147](#/week/6/run/20261021T173255_leg_eval_seed1_press_sweep_r4_force_5999_armlow) (20 missed, 12 past 60 N; MAE 3.0 N) | [74](#/week/6/run/20261021T175100_leg_eval_seed1_press_sweep_r4_position_5999_armlow) (4 missed, 5 fell, 14 past 60 N; MAE 6.7 N) |
| armature 0.02 / knees 0.073 | [205](#/week/6/run/20261021T173516_leg_eval_seed1_press_sweep_r4_force_5999_armhigh) (5 past 60 N; MAE 2.0 N) | [81](#/week/6/run/20261021T175324_leg_eval_seed1_press_sweep_r4_position_5999_armhigh) (7 past 60 N; MAE 6.3 N) |
| UniFP's ideal-PD legs (armature and delay kept) | [196](#/week/6/run/20261021T173734_leg_eval_seed1_press_sweep_r4_force_5999_idealpd) (4 past 60 N; MAE 2.2 N) | [84](#/week/6/run/20261021T175545_leg_eval_seed1_press_sweep_r4_position_5999_idealpd) (8 past 60 N; MAE 6.5 N) |
| GO2HV with the knee as the hip | [205](#/week/6/run/20261021T173935_leg_eval_seed1_press_sweep_r4_force_5999_unitreeknee) (11 past 60 N; MAE 2.2 N) | [46](#/week/6/run/20261021T175747_leg_eval_seed1_press_sweep_r4_position_5999_unitreeknee) (26 missed, 9 fell, 81 past 60 N; MAE 8.0 N) |
| the taped stand, 41 x 28 cm at 0.30 m | [188](#/week/6/run/20261021T174138_leg_eval_seed1_press_sweep_r4_force_5999_tape30) (6 past 60 N; MAE 2.4 N) | [84](#/week/6/run/20261021T180000_leg_eval_seed1_press_sweep_r4_position_5999_tape30) (10 past 60 N; MAE 6.7 N) |

The force-aware final holds 143-229 of 288 in every condition, the position-only one 43-95. As for the key (F-179),
the loop's total delay and a low armature are what it rests on: 15 ms costs 31 presses, 20 ms 54; the lowest trained
armature 50, mostly approaches that never settled.

`figures/foot_press_eval_r4.png`: held by plate kind; the force held against the command, rigid and sprung (the
position-only policy over-presses at 8 N and under-presses at 28 N, as an impedance does; the force-aware one is on the
line); the peak force's distribution; two plates' force traces. The key evaluation is F-179's (Lukas's turn, the light
lock, damage at 60 N, the same legs and the dog's stand): the general policy, never trained on the key, unlocks 107 of
108 with the foot back down in all 108, more gently than the fine-tuned key specialist (101, peak median 15 N, F-179).
Exported for the shadow runner: `checkpoints/leg_press_force_r4_model_5999.npz` (numpy against the checkpoint: actions
8.6e-6).

**The slide set** (`--plate_set slide`, frozen 02:45 before any policy was scored on it): the same 48 plates, rigid,
16 N, once with a tangential part (0.25 of the level, one way along the plate) and once sliding up to 4 cm toward the
plate's centre while the level is held. The rows of set v2 are unchanged by the code that added it (checked against
`model_5000`'s recorded rows). Round 4's force-aware final
[holds 56 of 96](#/week/6/run/20261021T170129_leg_eval_seed1_press_r4_force_5999_slide) (tangential rows 30, slides 26),
the normal force's error 2.5 N; the slides go their full 4 cm (median; p10 1.9 cm). But of a 4 N tangential command it
held 0.43 N. The layer did that: its outer loop kept integrating the foot's in-plane miss while the plate was pressed,
which is exactly the displacement a tangential command asks for, so it moved the goal until the push was gone. Since
04:05 the loop stops once contact is called; `--press_hold_correction` keeps the old behaviour, and round 4's plate
evaluations use it (it trained with it). The position-only final's slide-set run was cut when its queue was stopped
(an empty run directory, not recorded); re-run, it [holds 30 of 96](#/week/6/run/20261021T180258_leg_eval_seed1_press_r4_position_5999_slide_b)
(MAE 5.0 N), its tangential push 7.4 N where the force-aware layer asked 4 -- not control: its goal is displaced along
the plate and the same outer loop drives the stuck foot after it.

**The in-press loop, both ways.** Round 4's force-aware final on the slide set with the loop off once contact is called
([run](#/week/6/run/20261021T181441_leg_eval_seed1_press_r4_force_5999_slide_nohold), a diagnostic: it trained with the
loop on) holds 3.0 N of the 4 N tangential command but slides 3 mm of 4 cm: friction holds the foot, and UniFP's target
only asks for the slide less friction over 400 N/m. The loop made the slides and cancelled the tangential push. The
layer now splits the plate's directions (`--press_hold_loop hybrid`, the default since 05:25): the miss is integrated
across the normal and across a commanded tangential force, so where no force is commanded along the plate the foot is
held in position. On the same final ([run](#/week/6/run/20261021T181955_leg_eval_seed1_press_r4_force_5999_slide_hybrid)):
3.05 N of the 4, the slides their full 4 cm, 55 of 96. `--press_hold_loop full` (alias `--press_hold_correction`) is
round 4's.

**The replication.** The final recipe in one go from the F-179 finals with seed 4, 3400 iterations (round 4's lineage
had ~3400 with the layer changed three times on the way). Two starts were stopped: with the loop still fully on
([force](#/week/6/run/20261021T165913_train_seed4_press_rep_force_s4),
[position](#/week/6/run/20261021T170028_train_seed4_press_rep_position_s4), ~60 iterations), and with it off
([force](#/week/6/run/20261021T170612_train_seed4_press_rep_force_s4),
[position](#/week/6/run/20261021T170728_train_seed4_press_rep_position_s4), ~690 iterations; at `model_3200`
[force 41](#/week/6/run/20261021T180923_leg_eval_seed1_press_rep_force_3200_real),
[position 86](#/week/6/run/20261021T181147_leg_eval_seed1_press_rep_position_3200_real)); smokes with the loop
[off](#/week/6/run/20261021T170528_smoke_seed1_press_smoke_nohold) and
[hybrid](#/week/6/run/20261021T181811_smoke_seed1_press_smoke_hybrid) ran clean. The third start, with the hybrid loop
(05:26 to 10:17, `logs/unifp_train/press/queue5d.sh`; [force](#/week/6/run/20261021T182350_train_seed4_press_rep2_force_s4),
[position](#/week/6/run/20261021T182506_train_seed4_press_rep2_position_s4)), is the one reported:

| checkpoint | force-aware held (damaged, past 60 N) | position-only held (damaged, past 60 N) |
| --- | --- | --- |
| `model_3200` | [68](#/week/6/run/20261021T191308_leg_eval_seed1_press_rep2_force_3200_real) (27, 37) | [54](#/week/6/run/20261021T191536_leg_eval_seed1_press_rep2_position_3200_real) (116, 144) |
| `model_4000` | [120](#/week/6/run/20261021T202323_leg_eval_seed1_press_rep2_force_4000_real) (30, 52) | [86](#/week/6/run/20261021T202549_leg_eval_seed1_press_rep2_position_4000_real) (10, 10) |
| `model_4800` | [170](#/week/6/run/20261021T213213_leg_eval_seed1_press_rep2_force_4800_real) (2, 4) | [63](#/week/6/run/20261021T213437_leg_eval_seed1_press_rep2_position_4800_real) (62, 78) |
| `model_5600` | [194](#/week/6/run/20261021T224342_leg_eval_seed1_press_rep2_force_5600_real) (16, 18) | [77](#/week/6/run/20261021T224011_leg_eval_seed1_press_rep2_position_5600_real) (49, 65) |
| **finals** (6003 / 6012), dog's stand | [**186**](#/week/6/run/20261021T231704_leg_eval_seed1_press_rep2_force_6003_real) (20, 23), MAE 2.5 N | [**58**](#/week/6/run/20261021T231849_leg_eval_seed1_press_rep2_position_6012_real) (47, 63), MAE 6.5 N |
| finals, settled stance | [203](#/week/6/run/20261021T231707_leg_eval_seed1_press_rep2_force_6003_real_settled) (11, 12) | [63](#/week/6/run/20261021T231855_leg_eval_seed1_press_rep2_position_6012_real_settled) (34, 43) |
| finals, slide set | [50 of 96](#/week/6/run/20261021T231713_leg_eval_seed1_press_rep2_force_6003_slide): 3.1 N of a 4 N tangential command, slides 3.6 cm of 4 | [24](#/week/6/run/20261021T231900_leg_eval_seed1_press_rep2_position_6012_slide) (25 damaged) |
| finals, key zero-shot 0.33 / 0.32 m | [104](#/week/6/run/20261021T232049_leg_eval_seed1_press_rep2_force_6003_key_dog33) / [103](#/week/6/run/20261021T232257_leg_eval_seed1_press_rep2_force_6003_key_dog32) of 108, peak median 9 N | [0](#/week/6/run/20261021T232052_leg_eval_seed1_press_rep2_position_6012_key_dog33) / [1](#/week/6/run/20261021T232304_leg_eval_seed1_press_rep2_position_6012_key_dog32) (into the door's face: 51 approaches missed, 57 damaged) |

Its finals' sweep (force-aware / position-only, released from the press point as they trained): 0-5 ms delay
[218](#/week/6/run/20261021T232517_leg_eval_seed1_press_sweep_rep2_force_6003_delay0to5) / [78](#/week/6/run/20261021T232946_leg_eval_seed1_press_sweep_rep2_position_6012_delay0to5); 15 ms [179](#/week/6/run/20261021T232522_leg_eval_seed1_press_sweep_rep2_force_6003_delay15) / [63](#/week/6/run/20261021T232955_leg_eval_seed1_press_sweep_rep2_position_6012_delay15); 20 ms [126](#/week/6/run/20261021T232626_leg_eval_seed1_press_sweep_rep2_force_6003_delay20) / [19](#/week/6/run/20261021T233056_leg_eval_seed1_press_sweep_rep2_position_6012_delay20); armature 0.003 [149](#/week/6/run/20261021T232631_leg_eval_seed1_press_sweep_rep2_force_6003_armlow) / [58](#/week/6/run/20261021T233104_leg_eval_seed1_press_sweep_rep2_position_6012_armlow); armature 0.02 [194](#/week/6/run/20261021T232734_leg_eval_seed1_press_sweep_rep2_force_6003_armhigh) / [66](#/week/6/run/20261021T233205_leg_eval_seed1_press_sweep_rep2_position_6012_armhigh); ideal-PD legs [188](#/week/6/run/20261021T232741_leg_eval_seed1_press_sweep_rep2_force_6003_idealpd) / [68](#/week/6/run/20261021T233213_leg_eval_seed1_press_sweep_rep2_position_6012_idealpd); knee as the hip [186](#/week/6/run/20261021T232843_leg_eval_seed1_press_sweep_rep2_force_6003_unitreeknee) / [63](#/week/6/run/20261021T233315_leg_eval_seed1_press_sweep_rep2_position_6012_unitreeknee); taped stand 0.30 m [188](#/week/6/run/20261021T232848_leg_eval_seed1_press_sweep_rep2_force_6003_tape30) / [62](#/week/6/run/20261021T233319_leg_eval_seed1_press_sweep_rep2_position_6012_tape30).
The recipe reproduces: the second seed's force-aware final is round 4's within ~10 presses everywhere, and on the key;
its position-only final is worse than round 4's (58 against 100), so the gap between the variants is wider, not
narrower. The second seed presses harder, though: 20 of its finals' presses damaged against 5.

**Where its hard presses came from** (both seeds' finals: 17 of the replication's 23 peaks past 60 N on plates facing
down). Traced: on a strip 6.5 cm wide facing down, the foot pushing up at 28 N slid off its edge, hung beside it for the
rest of the hold, and on the way back to its footfall moved in at the slab's height. The plate had a face and nothing
else, so the foot met no edge until its centre crossed it, and then the whole of its radius at once (145 N in one
physics step) -- a contact the plant made up. Two changes, both defaults since 10:55:

- the release starts from wherever the foot is and backs straight out, never drawing it back to the press point
  (`PressLayer.release_from_foot`; `--press_release_from_press_point` is the old one): alone, on the finals, it changes
  almost nothing ([replication](#/week/6/run/20261021T233701_leg_eval_seed1_press_rep2_force_6003_real_release2) 187 with
  21 damaged; [round 4](#/week/6/run/20261021T233709_leg_eval_seed1_press_r4_force_5999_real_release2) 197 with 4). A first
  version, along the normal only, changed nothing at all (recorded, superseded the same hour);
- the plate is the slab it is drawn as, 2 cm thick: the sphere meets its nearest point, face, edge or back, as the key's
  platform does (`PlateSet.edges`; `--press_plate_face_only` is the old one), the sprung travel on the face only. With
  both: [replication](#/week/6/run/20261021T234040_leg_eval_seed1_press_rep2_force_6003_real_edges) 186 with 12 damaged
  (p95 peak 149 -> 93 N), [round 4](#/week/6/run/20261021T234043_leg_eval_seed1_press_r4_force_5999_real_edges) 196 with 2.

Neither policy trained on either. **Round 5** (10:43-12:28, `logs/unifp_train/press/queue8.sh`;
[force](#/week/6/run/20261021T234224_train_seed3_press_r5_force_s3),
[position](#/week/6/run/20261021T234340_train_seed3_press_r5_position_s3)): both variants from round 4's finals, 1200
iterations on the final layer and plant (hybrid loop, release from the foot, the slab), then every evaluation of its
finals (26, recorded). It made the force-aware policy worse and less safe:
[167](#/week/6/run/20261022T012835_leg_eval_seed1_press_r5_force_7198_real) from the dog's stand (5 fell, 17 damaged),
[190](#/week/6/run/20261022T012838_leg_eval_seed1_press_r5_force_7198_real_settled) settled (11 fell), plates facing down 7
and 9 of 72, falls in 2-86 episodes across the sweep. Every fall is on a plate facing down at 28 N, rolling ~45 degrees to
the right at 33-42 N of contact: pushing up on a plate above it, the foot is pushed down at the corner the robot is not
standing on. Round 4's policy under-presses those plates by ~6 N and stands; round 5 learned to press them fully and
tips (F-182). Its key zero-shot held (105 / 107), and the position-only twin became gentler and better on the key (98
plates, 95 / 92 keys).

**The candidate, on the final layer and plant** (round 4's finals, not retrained; 12:47-12:50):

| round 4 final | set v2, dog's stand | set v2, settled | slide set (96) |
| --- | --- | --- | --- |
| force-aware | [196](#/week/6/run/20261021T234043_leg_eval_seed1_press_r4_force_5999_real_edges) (2 damaged, 5 past 60 N) | [207](#/week/6/run/20261022T014725_leg_eval_seed1_press_r4_force_5999_final_layer_settled) (0 damaged, 3 past 60 N, max 110 N) | [56](#/week/6/run/20261022T014728_leg_eval_seed1_press_r4_force_5999_final_layer_slide): 3.05 N of 4 tangential, slides 4 cm, nothing past 34 N |
| position-only | [100](#/week/6/run/20261022T014841_leg_eval_seed1_press_r4_position_5999_final_layer_real) (4 damaged) | [86](#/week/6/run/20261022T014846_leg_eval_seed1_press_r4_position_5999_final_layer_settled) | [30](#/week/6/run/20261022T014959_leg_eval_seed1_press_r4_position_5999_final_layer_slide): 1.2 N of 4 |

(On set v2 the split loop is the full one -- no tangential commands -- so the dog's-stand row is the edges run.)

**Viewers** (Lukas: "Can I see visualisation", then "can you open it again"): round 4's force-aware final on row 86 of
set v2 (a wall ahead-right at 0.36 m, rigid, 28 N), one robot, real time, the realistic legs from the dog's stand, the
plate orange while pressed and green while the force is within tolerance:
[first](#/week/6/run/20261022T074454_play_seed1_press_play_r4_row86) (closed from the window),
[reopened](#/week/6/run/20261022T141024_play_seed1_press_play_r4_row86_b) on 2026-10-23.

**Tests:** `tests/test_foot_press.py` 23; the full suite 827, the known `test_payload` failure only (a first run during
training also failed `test_decoupled`'s 50 ms wall-clock bound at 55.7 ms under the load; it passes alone and in the second
run).

**What this shows, and what it does not.** *Shows* (simulation): one force-aware foot policy presses flat plates across
the front-right leg's reach and facing, rigid and sprung, holding the commanded force to ~2 N; the force-aware
formulation is ~2x the position-only one on plates and on the key, over two seeds and eight changes to the legs; trained
only on plates, it turns the key as well as the key specialist, more gently. What its layer has to do (F-181), and that
upward pushes past ~30 N at the lifted corner tip the robot (F-182). *Does not show*: the dog -- no task layer runs on it,
no plate or key estimate comes from a camera; real contact (the plates are penalty models on three spheres, edges only
since 10:55); anything outside the reach the table samples; tangential force beyond a 4 N test; more than one fine-tune
of the lineage.

### 2026-10-21 · The key turned by the arm (simulation)

Lukas: "I am curious if the arm itself can turn the lever. assume the turn is for 45 degrees to -45 degrees and create
a scripted unlock with visualisation. you can re-use the door detection and apriltag stuff already used for the door
opening", then "the real lever takes about 5N to rotate in the lock". Asked, he chose simulation first and the foot's
turn: 45 degrees up to 45 down, clockwise as the robot sees it.

**Built** (`demos/combiner_key/`, its README's "The arm turning the key instead"):
- `arm_turn.py` (numpy): the push. The hand comes in pitched 60 degrees down onto the platform about the key's
  direction, jaws shut, the fingers side by side along the key, their tips' lower edge on the platform 0.13 m out from
  the shaft and 35 mm off the door; the turn is that hand pose rotated rigidly about the shaft (after the isolator
  handle's push, `demos/combiner/press.py`), the edge 4 mm inside the platform. The stance: the door 0.44 m ahead, the
  lock 8 cm left of the base (the leg turn's mirrored). And the CPU scan below.
- `arm_env.py`: `ArmTurnEnv`, the key demo's environment with the key in the lock and the leg turn's slot filled by
  the arm's: the leg turn's lock plant (`LeverHandle(pad=True)`, the key's sticking friction, stops and weight) against
  a 5 mm sphere at the fingertips' edge, its force on Link6. The look is the door demo's (head-camera AprilTag, MID-360
  face fit); the walking policy keeps the legs; an overview camera takes the pictures.
- `run_arm_turn.py` (`run_arm_turn.sh`), `plot_arm_turn.py`; `env.py` gains a `_make_turn` hook (the leg turn
  unchanged). `tests/test_arm_turn.py`: 11 tests (the hand's frame, the contact sphere, the targets, the stance, the IK
  and push over the turn at 0.28-0.33 m); with the key demo's 9, 20 pass.

**On the CPU first** (`python -m demos.combiner_key.arm_turn`; the scan is kept in the
[first run's record](#/week/6/run/20261021T060043_lock0.65nm_seed1_first)): the hand held level, as the isolator push
was, does not solve at the key at any stance (the lock is ~0.11 m above the base, the wrist and fingers 0.2 m long);
pitched 40 degrees it solves at a 0.28 m base but not from 0.31; pitched 60 it solves every 5 degrees of the turn at
all 72 combinations of door 0.42-0.48 m, lock 6-10 cm left, base 0.28-0.33 m, with and without the wrist camera's 99 g,
clear of the trunk proxy. The push's quasi-static ceiling (`press.press_capacity`: the arm's weight plus J^T F within
the published limits) at its worst angle is 1.31-1.50 N·m over that spread, 1.43 at the stance (at +-45 degrees, where
the push runs sideways and Joint1 binds; Joint2 binds in between, 1.64-1.92), 10-11.5 N at the push point. The arm's
own weight takes up to 90% of one joint's limit in a corner of the grid (with the camera).

**Runs** (`run_arm_turn.sh --headless [--attempts N --no_overview] [--lock_n N]`; the head camera and lidar look,
the box spread as in the key demo; lock = the plant's friction, `--lock_n` x 0.13 m):

| lock, N at 0.13 m (N·m) | run | unlocked | furthest key angle | hand's lag in the turn | contact, median | joint at its limit |
| --- | --- | --- | --- | --- | --- | --- |
| 5 (0.65), first | [run](#/week/6/run/20261021T060043_lock0.65nm_seed1_first) | 1/1 | 45.25 | 9 mm | 4.8 N | 6.4% of push steps |
| 5, demo (frames) | [run](#/week/6/run/20261021T060916_lock0.65nm_seed1_demo_k8000) | 1/1 | 45.18 | 9 | 4.8 | |
| 5, 8 placements | [run](#/week/6/run/20261021T061331_lock0.65nm_seed1_placements) | **8/8** | 45.17-45.20 | 8-10 | 4.8 | 4-6% |
| 5, check | [run](#/week/6/run/20261021T063342_lock0.65nm_seed1_lagcheck) | 1/1 | 45.18 | 10 | 4.8 | |
| 8 (1.04) | [run](#/week/6/run/20261021T061751_lock1.04nm_seed1_sweep) | 4/4 | 44.6-44.9 | 13-14 | 7.8-7.9 | 7-11% |
| 11 (1.43) | [run](#/week/6/run/20261021T062130_lock1.43nm_seed1_sweep) | 4/4 | 42.2-42.8 | 17-20 | 10.1-11.2 | 21-35% |
| 14 (1.82) | [run](#/week/6/run/20261021T062509_lock1.82nm_seed1_sweep) | 1/4 | 40.2; 6.4, -40.6, -43.4 | 41; 96-147 | | 34-41% |
| 17 (2.21) | [run](#/week/6/run/20261021T062849_lock2.21nm_seed1_sweep) | 0/4 | -38 to -45 | 122-160 | | 30-67% |

One more ([run](#/week/6/run/20261021T060635_lock0.65nm_seed1_demo)) was stopped by me ~100 s in, before its push, to
change the contact model; no result. Figures: [`figures/arm_turn_demo_5n.png`](figures/arm_turn_demo_5n.png) (four
overview frames, the key's angle, the push, each joint against its limit) and
[`figures/arm_turn_lock_sweep.png`](figures/arm_turn_lock_sweep.png). The push as an animation:
`logs/combiner_key_arm/20261021T060916_lock0.65nm_seed1_demo_k8000/push.gif` (not committed, 3.6 MB).

- *The look and the stance*: the tag in 27 of 27; the lock's estimate 0.9-10.7 mm off (the tag's depth, as in the
  key demo). The walk ends within 5 mm of its target, then P2Dingo's gait, stepping in place, drifts the base 2-4 cm in
  the 1.5 s settle: the push started with the door 0.45-0.48 m ahead (0.44 asked) and the lock 1-6 cm left (8 asked).
  It turned the key from there; the CPU scan's spread is +-2 cm sideways.
- *The key follows the command* to a median 0.2-0.3 degrees at 5 N; the push at 5 N is 4.8 N along the key's arc.
- *The ripple*: the contact force swings 3-6 N about its mean from step to step, and the joints' torques with it. Not
  the contact: halving its stiffness (16,000 to 8,000 N/m) and damping it harder changed nothing (step-to-step change
  median 0.38 against 0.39 N). It is the D1's 10 Hz streaming as `motor_model` has it (the firmware's plan restarting
  from rest at each setpoint, F-045/F-046): with the hand held still above the key the shoulder's torque ripples at
  10.0 Hz while the base is still (roll 0.04 degrees RMS).
- *The hold*: in the first run the edge was held 4 mm inside the platform at the stop, and the arm pushed the key into
  its stop at 11 N with Joint1 at 100%. It now rests on the platform for the hold: 0 N, the shoulder at 52%.
- *Joint forces past their limits* (to 2.4x, Joint5) appear only in `arm_out`, the swing from home to the first push
  pose while the IK is still in position-only: constraint forces (a joint at its limit, or the arm against the body), not
  the drives. None in the push at 5-11 N.
- *The robot's own check*: its fingertips' edge on the estimated key reads 45 degrees at the hold on keys that never
  moved (14 and 17 N): the fingers had slid off the platform and followed the command in the air. Every key that turned
  had the hand at most 20 mm behind its command through the turn; every stalled one 96-160 mm. The runner now requires
  both (the edge past 30 degrees and the lag under `TRACK_LIMIT_MM` = 30); applied to these traces it is right on all 26
  attempts but the 14 N one that stopped at 40 degrees (41 mm), which it rejects. The table's runs predate it (their
  `verified_unlocked` is the edge alone); the check run has it.

**What this shows, and what it does not.**
- *Shows* (simulation): from the door demo's look and a walked stance, the D1 alone turns the key from 45 degrees up to
  45 down against a constant 5 N (0.65 N·m) in 11 of 11 attempts, and up to 11 N in 4 of 4; the simulated limit,
  11-14 N, is where the CPU's quasi-static ceiling (10-11.5 N) puts it. Lukas's 5 N has about twice that margin.
- *Does not show*: the real arm (the published 3.3 / 3.33 / 1.7 N·m limits; what the firmware lets the servos give is
  not measured, F-142); the hand in contact (a 5 mm sphere at the fingertips touches the plant; the rest of the hand
  passes through the key); the real cam's feel (a constant friction from one number; the unlock at 30 degrees is the
  leg model's); the wrist camera's mass (the robot here draws it; the CPU scan has it, and its weight helps a downward
  push); where the 5 N was felt (taken at 0.13 m: felt at the key's end, 0.16 m, it is 0.8 N·m, 6.2 N at the push
  point). One seed.
- *Next*, if Lukas wants it on the dog: a stage in the combiner door's robot program (dry run with the RViz ghost first),
  a cleaner swing from home to the first push pose, and the check above.

**Then: close enough is enough.** Lukas, watching it in the viewer: "it gets stuck in a loop for a minute trying to line
up perfectly in sim, but in reality it will also not line up perfectly so we should just accept a position close enough
just like in the real door pull". The walk drove each axis at 0.15 m/s until within 12 mm of the stance (P2Dingo's gait
stands inside 0.1 m/s), so it hunted round the point for the rest of its 12 s. It now stops, and its phase ends, the
first time the look's estimate puts the door 0.42-0.45 m ahead and the lock 4-12 cm left of the base, facing the door
to 3 degrees (`arm_turn.ACCEPT_*`): the near part of where the push works on the CPU (`PUSH_*`, door 0.42-0.48 m, lock
2-12 cm left, every corner solving the whole turn at 0.28-0.33 m with at least 1.31 N·m; two new tests).
- [First window](#/week/6/run/20261021T064727_lock0.65nm_seed1_window) (0.42-0.48 m): 8/8 unlocked to the stop, the walk
  over after 1.4-1.8 s; but it stopped at the far edge (0.478 m) and the gait drifted the base 1.5-4 cm back and up to
  6 cm sideways in the settle, to 0.48-0.52 m: two of those stances do not solve the whole turn on the CPU, though the
  push worked from them.
- [Near window](#/week/6/run/20261021T065244_lock0.65nm_seed1_window_near) (0.42-0.45 m, lock 4-12 cm): 8/8 unlocked to
  the stop (45.16-45.20 degrees), verified by the robot's own check (lag 7.7-9.8 mm), the walk over after 1.6-2.1 s,
  pushed from 0.46-0.49 m with the lock 1.4-8.7 cm left.

### 2026-10-21 · The key turned by the arm: ready for the dog (not yet run there)

Lukas: "can we make this ready to run on the real arm, just like the door opening did with visualisation".

**Built**: the push as a task of the combiner door's robot program, `node.py --task key` (`d1dogkey`; dry unless
`--live`), with the door's look, straight walk in, refit, arm link, aborts, keys, record and RViz view. The key's parts are
`demos/combiner_key/robot/key_plan.py` (`KeyFix`, `KeyAttempt`, `key_phases`) and `key_viz.py` (the key at the angle the
fingers measure, the push point's arc with the unlock, the walk's and the push's stance areas on the floor); the door's
`plan.py` gained hooks for them (`make_fix(walk_for, fix_class)`, the phase sets as `Attempt` attributes, `_make_phases`,
`_prepare`, `_recovery_phases`, `_solve_joint_goal`; refit and reanchor keep the fix's class), `viz.scene_markers` an
`extra` and `door_task`, `run_on_dog.sh` the run's name (`<stamp>_key_<stage>_...`). The door's 33 robot tests pass
unchanged. Stages: look, unfold, walk, reach, touch, turn. The stance: the dog in front of the lock, walking straight in
until the door is 0.435 m ahead; checked after the refit against where the push works (`arm_turn.PUSH_*`).

**What the dry runs on the CPU model found** (`demos/combiner_key/tests/test_key_robot.py`, the arm link's profile
driving a model servo, the gait's carry):
- *A wrist flip mid-turn*: the 10 Hz IK, warm-started from its own last solution, jumped Joint4 +84, Joint5 +60 and Joint6
  -94 degrees in one step halfway through the turn (and the lag abort fired, the arm 52 degrees off). The push's hand
  passes near the D1's wrist singularity there. The simulation (F-183) did not flip: its IK was seeded differently.
  Now the turn is planned before anything moves (`_plan_push`, at the attempt's first step, before the arm link has a
  command): every first pose the seeded search finds, the turn traced every 2.5 degrees with jumps subdivided, the
  branch whose Joint5 stays furthest from 0 kept (6-12 degrees on the placements tried), and the 10 Hz IK seeded from it.
- *Too fast for the arm link*: on that branch Joints 4 and 6 swing ~90 degrees each way over 20 degrees of key near 0;
  at the simulation's 15 degrees/s of key that asked 66 degrees/s, past the arm link's 30, and the arm fell 39 degrees
  behind. The turn is now retimed from the plan (the key at 10 degrees/s or the fastest joint at 15, eased: at most 22.5):
  12.7-15.5 s instead of 9; the worst joint lag in the turn 7.2-7.6 degrees (the abort is at 10).
- *Planning time*: 0.5-1.1 s on the PC at the first step; the full plan again at `key_out` cost 34-78 ms, too close to
  the arm link's 0.5 s watchdog on the dog's CPU while it streams, so `key_out` only solves its first pose again.
- Five placements dry: four turned to the stop and verified (lag 7.7-8.6 mm, the fingers' measure 45.2-45.3 degrees),
  one (the lock 12 cm left, the box 3.5 degrees off square) stopped after the walk with "move the dog left 0.05 m".
- 12 tests: the fix and range check, every stage to the fold, the walk ending where the push works, the turn verified,
  a far box off square planned without a flip, every joint under the arm link's speed, a stalled key aborting and the
  hand lifting off, RViz's key where the fingers put it. The node imports and parses under ROS Humble's Python.

**Not done**: anything on the dog -- not even a dry run with its camera and lidar. Not checked: the real key's place and
handedness against `geometry.py` (the simulation's key is the STL mirrored, pointing right; the dry run draws it to
compare), the real D1's push at its limits, the arm's swing to `key_out` on the robot. The procedure is in
`demos/combiner_key/robot/README.md`: `d1update`, `d1view`, `d1dogkey` (dry), then live `reach`, `touch`, and the turn
with `--step`.

**Then on the dog, and a walk up** (2026-10-21 evening). Lukas ran `d1dogkey` before `d1update`: the dog's old node
refused `--task` (no record but an empty run folder). After `d1update` (by me) his live `--stage reach` looked and
refused: "lock 0.44 m ahead, +0.17 m left of the stance, door -1.4 deg off square, walk in 0.38 m; move the dog left
0.09 m"; nothing moved. Lukas: "it needs a walk up stage. I don't want to position it myself". The key's `walk` stage
is now a walk up (`key_plan.KeyAttempt._walk_up`): the look sets the script frame at the push's stance (`stance_for`:
facing the door, its face 0.45 m ahead at the lock, the lock 7 cm left -- the middle of `arm_turn.PUSH_*`; `plan.make_fix`
takes a `stance_for` (x, y, yaw) now), and the walk closes on the odometry on all three axes at once, each lifted out of
sport mode's dead zone, the sideways floor raised when the dog does not move sideways (P2Dingo's docking: no side-step
at 0.25 m/s from a standstill), stopped early by the gait's carry, stepped again until it stands where the push works
with 1 cm to spare. The look accepts a stance up to 0.7 m ahead (0.2 m back), 0.4 m across and 25 degrees turned.
The first version hunted across the stance at +-0.18 m/s (every small error lifted to the floor) and stopped only on its
timeout, 5 cm off: each axis is now left alone within 2 cm / 2 degrees, and the stop is the carry's distance. Dry, on the CPU
model with the gait's carry: Lukas's look walks up in 2.6 s with one stop (the lock 0.455 m ahead, 8.4 cm left, 1.4
degrees off); from the box's middle and from 0.10 m right and 8 degrees off, one stop each; 13 key robot tests and the
door's 33 pass. Not yet walked on the dog. [Lukas's live look](#/week/6/run/20261021T073004_key_reach_live) (recorded).

**The first live walk up** ([run](#/week/6/run/20261021T073606_key_walk_live), `d1dogkey --stage walk --live`): the
stance 0.38 m away, 0.10 m across, 3 degrees turned. Lukas: "it walked up to the door fine, but the extended d1 arm kept
hitting the door, causing it to retry a few times". The first stop was 5 cm short (lock 0.508 m ahead); each of six
corrective steps moved the base ~3 cm forward and back again (base x -0.05 to -0.01 m and back): the hand, held at the
door pull's home point with its fingertips 0.42 m ahead of the base, met the door 0.42-0.48 m ahead and pushed the dog
back. It ended 0.48 m off by its odometry; the refit put the door 43 mm closer (lock 0.44 m ahead, 12 cm left, 1.9
degrees off), in range, and the arm folded. The key's phases now hold the hand at `KEY_HOME_POINT`: the fingertips 0.24
m ahead of the base (the arm's furthest point 0.20). Dry, through the walk up the fingers stay 21-22 cm off the door, the
joint move out to the first push pose 9.4 cm, the push itself 12-22 mm (the platform is 35 mm out); a test checks the
walk's clearance (>10 cm). 47 robot tests pass. Not yet walked again.

**The second live walk up** ([run](#/week/6/run/20261021T074021_key_walk_live), the arm held back): no contact, but
Lukas: "the dog still kept moving back and forth at the door. I think we need to accept a wider distance threshold".
Again six corrective steps: the first stop 5 cm short (lock 0.513 m ahead by the look's fix; the gait carried on 2-3 cm,
not the `CARRY_S` x 0.25 m/s = 9 cm the stop allowed, and settles back 1-3 cm standing), then steps at sport mode's
0.18 m/s floor that overshoot a 2-4 cm gap and settle back. And again the refit put the door 42 mm nearer than the
look's fix (43 mm the first time): the stances it was correcting were already 0.43-0.47 m off, where the push works. The
walk up now ends once the look's fix has the lock 0.42-0.52 m ahead, 2-12 cm left, the base facing to 4 degrees
(`WALK_UP_ACCEPT_*`), with at most two corrective steps; the refit's box must then be in `arm_turn.PUSH_*` (door
0.42-0.48 m), as before. Both live walks' first stops (0.508 and 0.513 m, 7-8 cm left, ~2 degrees) would have ended it
there; the refit then put them at ~0.47 m. Whether the look's 4 cm is a bias of the fix's depth (the tag's range, as in
the simulated key demo's 3-11 mm) or of the odometry over the walk is not separated.

**Two more live runs**: a walk up ([run](#/week/6/run/20261021T074548_key_walk_live)) in one stop (lock 0.518 m by the look's
fix), refit 58 mm nearer, in range; and a reach ([run](#/week/6/run/20261021T074633_key_reach_live)) that walked up in one
stop (0.489 m by the fix) and aborted at the refit: "the lidar puts the door 63 mm and 0.7 deg from where the look did",
past the door pull's 6 cm limit (RViz screenshot from Lukas, in the chat). By the refit it stood ~0.43 m off, where the
push works. Across the four walks up the refit's correction, 42, 43, 58 and 63 mm, is 12-15% of the 0.34-0.47 m walked:
an odometry scale error (the look's lidar face agreed with the tag to under 3 cm each time, the look's guard). Now the
walk up is sent 13% short on the odometry (`ODOM_WALK_SCALE` 0.87, estimated from these four) towards a stance 0.44 m
from the door, ends within 4 cm of where it was sent (a corrective step only past 9 cm: smaller ones overshoot), the refit
may move the box 10 cm, and the push is planned at the stance for the box as the look has it. Dry, with the odometry
84-90% of the walk, three placements each land in one stop at 0.443-0.478 m by the refit (in `PUSH_*`); at 92% one of
three stops 0.04 m too far and says so. The dry runs now model the odometry's error and give the refit the lidar from
where the base really is. 47 robot tests pass.

**Then** ([run](#/week/6/run/20261021T075232_key_reach_live), live reach): the 13% held -- the refit moved the box 51.6 mm
over a 0.40 m walk -- but the walk up stopped 6.9 cm short of where it was sent (under the 9 cm a corrective step then
needed) and the refit had the door 0.52 m off: "move the dog forward 0.08 m". The stop came early by `plan.CARRY_S`
(0.36 s, P2Dingo's measure in sport mode's default gait); in classic walk, which this program walks in, the dog carried on
2-3 cm from 0.18-0.25 m/s and settled back 1-1.5 cm in every live walk up. Now the walk up starts from a 0.12 s carry
(`WALK_UP_CARRY_S`) and learns it from each stop, and takes a corrective step whenever the gap is over 70% of the shortest
step the gait makes (the door pull's rule; ~6 cm). And the turn now waits for the arm: while a joint is more than 5
degrees behind its goal the turn's clock holds (3 s held in all aborts, a key not turning) -- with the box 6 degrees off
square a dry turn lagged 11 degrees, the plan being for the stance the look predicts. Dry, with the classic walk's carry
and the odometry 84-90% of the walk: four placements land in one stop at 0.443-0.468 m by the refit; turns hold 0-1.1 s
and verify. 47 robot tests pass.

**Then** ([run](#/week/6/run/20261021T080001_key_reach_live), live reach): the walk up worked -- one stop, 3.1 cm from
where it was sent, the carry 1.6 cm (0.09 s), the refit 58 mm (13% again), the lock 0.48 m ahead and 9 cm left, in range --
and `key_out` refused: "the move to the key's first pose crosses the trunk proxy". The waiting hand itself was inside it:
sent to "0.24 m above the base" by a script-frame height (`KEY_HOME_POINT`, which assumed the box's root 0.358 m below
the base), it went to 0.207, the base standing 0.391 m over the box's root by the look. The waiting point is now in the
base's own frame from its measured height (`KEY_HOME_IN_BASE`, 0.20 m ahead, 0.30 up), clear of the trunk proxy from
every IK start tried.

Checking that found two more things. *The dry tests had the box 0.65 m below the base* -- I had misread `plan.base_pose`,
which puts the base 0.32 m up in the odometry -- now fixed (the box on a floor 0.30-0.37 m below). And *at the dog's
height the planned pitch is close to the wrist's singularity*: F-183's CPU scan covered 0.28-0.33 m; at 0.353 m the
push pitched 60 degrees turned from 5 of 9 stances, Joint5 down to 0.6 degrees of 0 (the dog's plan this run had it at
5.8), and pitched 50 from 9 of 9; at 0.32 m the reverse. The pitch is now planned per attempt (`key_plan.PITCHES_DEG`,
50-70 degrees: the one whose turn keeps Joint5 furthest from 0); on the dry geometries it picks 70 (Joint5 >= 19.5
degrees at 0.32-0.37 m). Planning takes 1.4-2.3 s on the PC, several times that on the dog, after Enter and before the
unfold. The simulation (F-183) ran the push at 60 degrees on a 0.31 m walker: 70 degrees at 0.353 m is checked on the
CPU model only (IK, continuity, joint speeds), not its push force or contact. 48 robot tests pass.

**The first live turn** ([run](#/week/6/run/20261021T081629_key_turn_live), `d1dogkey --live --yes`). The plan chose
70 degrees (Joint5 >= 23.9 from 0; 60 would have been 5.5), the walk up stopped in range (refit 44.7 mm, the lock 0.48 m
ahead, 9 cm left), the hand went out, touched the platform and turned the key: the fingers' own measure followed the
command to 1-2 degrees from -45 to +14 degrees (the hand 12 mm off its path at most). Then it aborted, "the key is not
turning: the arm held 5 deg behind for 3.0 s of the turn": Joint4, the wrist joint the singularity makes fast, ran 5-7
degrees behind the arm link's command, and the turn's clock-hold (past 5 degrees) had used up its 3 s. Lukas, watching it:
"I can see its clearly turning the handle, so we should tolerate the lag". The recovery lifted the hand off and folded.
Now the clock holds only past 8 degrees of lag, and the turn aborts only when the fingers' measure of the key has not
advanced 2 degrees in 3 s; the arm-lag (10 degrees for 0.6 s) and hand-lag (30 mm) aborts stay. A dry test of an arm
running behind (every other step) turns to the stop and verifies. **The arm pushed the real key from -45 to +14
degrees**: the first contact turn on the robot, not yet past the unlock (30) or to the stop.

*Correction (same evening, recording the runs in between)*: four live runs between the reach at 08:00 and this one had
not been recorded; they are now. Two reaches, [08:08](#/week/6/run/20261021T080820_key_reach_live) and
[08:11](#/week/6/run/20261021T081113_key_reach_live), walked up in one stop, refit 44.3 and 47.1 mm, the lock 0.47 m ahead and
8-10 cm left, to above the key and back, finished. A turn at [08:09](#/week/6/run/20261021T080928_key_turn_live) was stopped
by Lukas during the walk up. And a turn at [08:12](#/week/6/run/20261021T081246_key_turn_live) was the first on the key: the
refit 57.4 mm, the fingers' measure to **+25 degrees**, then the same abort ("the arm held 6 deg behind for 3.0 s of the
turn"), the recovery lifting off and folding. So the run above was the second live turn, not the first, and the clock-hold
budget stopped both.

### 2026-10-21 · The key turned to its stop on the dog, the way home past the head camera, and on to the door pull

**The turn, live** ([run](#/week/6/run/20261021T082236_key_turn_live), `d1dogkey --live --yes`, after the lag change
above). Planned at 70 degrees (Joint5 >= 24.0 from 0); the walk up 0.34 m in one stop (1.9 cm from
where it was sent), the refit 50.0 mm and 1.2 degrees, the lock 0.46 m ahead and 8 cm left; key_out solved
there and clear of the trunk proxy. The third live turn. The turn took 12.1 s; the fingers' measure of the key went -44 -> -35 -> -15 -> +1 ->
+18 -> +35 -> **+45.6 degrees**, the hand at most 17.7 mm off its path (the limit 30), the clock held 1.3 s in all, the
worst joint 9.8 degrees behind the arm link's command (the loaded abort is 10 for 0.6 s). The robot's check: verified.
Then key_lift, key_back, home, fold, and the run finished. **The D1 turned Lukas's real key from 45 up to its stop**
(F-184).

**The camera.** Lukas: "the arm collides with the realsense camera on its way back to its resting position after a
successful open - we need to have a retract stage before which clears the camera". The head D435i and its bracket stand
4 cm past the trunk proxy's front (x 0.288-0.3435 m, z 0.066-0.105 m in `base_link`), where the arm's clearance check
does not look. The record's measured joints through forward kinematics, against that box (the proxy's 3 cm link radius):
the turn kept 28 mm, the hold 30, the lift 31; `key_back` -- 8 cm back along the hand's approach -- ended at -1, because
at +45 degrees the approach points right and down ([0.34, 0.64, -0.69] in the base), so backing off took the wrist toward
the camera; `home`, a straight line from there to the waiting point, then passed in front of the case at its height:
-30 mm, the fingertips at x 0.341-0.348, z 0.07-0.11 at t = 47.9 s (in the case). The fold stayed 142 mm off.
Now (`key_plan.key_phases`): `key_back` is gone; after `key_lift` comes **`key_clear`**, straight up from above the
platform with the hand's orientation held until the jaw centre is 0.20 m above the base (`KEY_CLEAR_ABOVE_BASE_M`), then
home. The recovery's `key_away` (back along the approach) is the same `key_clear`. Replayed from that run's measured
lift-off: 31 mm on the way up, 80 home. Dry, over four boxes and base heights 0.30-0.37 m: `key_clear` >= 45 mm, home >=
79, fold 167; the turn itself 29-45. The box is `head_camera.CLEARANCE_BOX_BASE_M`; the key attempt records its smallest
margin per phase (`camera_gap` events) from the measured joints. Not added to the arm's clearance proxy everywhere
(`position_only.workspace`), which the policies' training and the door pull also read: the door pull's own phases are
not checked against the camera.

**On to the door pull** (Lukas: "I would like an run option that chains this lever open into the door pull. ideally this
involves moving the dog from its end position from the lever task to the desired position without having to backup and
walk back in forwards"). `node.py --task key --stage turn --then_door pull` (`d1dogkeydoor`, with `--door_deg 60`): once
the robot's own check says the key turned (else the run ends there), `key_plan.DoorAfterKey` takes over with no new look.
Its fix is the key's after its refit, the script frame moved to the door pull's stance (`door_fix`: square to the door,
the box's root 0.69 m straight ahead, the middle of the simulated placements) -- ~0.07 m back and ~0.18 m right of where
the key leaves the dog. With the arm folded (the door pull's unfold puts the hand 0.40 m ahead, in the door at the key's
stance), it walks over with the key's walk up (back, sideways and turning at once), settles, refits and range-checks as the
door pull does after its walk in, unfolds, and goes on through the door pull's phases. `--then_door walk|reach|grip`
stops it short. Dry on the CPU model (the odometry 87% of the walk, the classic walk's carry): one stop 1.4 cm from the
stance, the refit 11.5 mm, the box 0.70 m off at -1.1 degrees, then unfold, reach, grip, a 30-degree pull, release and
fold; the base never more than 0.12 m behind the stance. The walk-up code is now a mixin both attempts use
(`key_plan.WalkUp`). 20 key robot tests (new: the way off the key clear of the camera for every stage, the chained pull,
no pull after an unverified turn, the door's stance from the key's), 14 arm-turn and 33 door robot tests pass. **Not run on
the dog**: neither `key_clear` nor the chain; nor is the door pull's stance after the walk over checked against a real look.

### 2026-10-21 · The key inserted by the arm on the dog: ready, not yet run there

Lukas: "can we now do the same but with the insertion? the handle will need to put the key in at a 45 degree angle".
His answers when asked: the key is handed to the gripper; the keyway is the bare cam lock's, under 0.5 mm of play (no
printed adapter, no lead-in); the robot program first; insertion only, no chaining yet.

**The reach at the dog's height.** The simulation inserts with the hand level, rolled so the key goes in at 45 degrees
up, the jaws across the bar 25 mm out from the shaft, from a walker standing 0.27-0.31 m over the box. On the CPU arm
model at the dog's 0.33-0.39 m (its looks put the base 0.353-0.391 m over the box's root), that hand solves from no
stance at all: 0.40-0.70 m from the door, 0.10 m left to 0.20 m right of the lock, approaches 25 and 60 mm out and
seated, trunk clear and holdable. The lock is only 3-6 cm above the base there. Two ways out were offered: grip the bar
160 mm out (17 stances at 0.353 m, but a 1-degree wrist twist moves the shaft ~2.8 mm), or lower the body. Lukas: "Make
the dog sit, and then insert while it is sitting down". Taken as sport mode's StandDown, which P2Dingo's "sit" sends
(the trunk lowered level over its planted feet, `P2Dingo/Isaac/go2_omniverse/posture.py`), not Unitree's `Sit` trick,
which tips the chest up. At 0.13-0.25 m over the box (lying, ~0.20 m lower) the grip at 25 mm solves from 16-46 stances,
the shoulder at 34-89% of its torque. The insertion's stance is the seated key 0.50 m ahead and the lock 0.10 m right,
where it solves from 0.13 to 0.22 m at 64-69%.

**Built** (`node.py --task insert`, `d1doginsert`; `demos/combiner_key/robot/insert_plan.py`, `insert_viz.py`):
- stages load, walk, lie, reach, touch, insert. The key is handed in at the carry pose in front of the dog: the jaws
  open, the hand already rolled as it inserts. The program always waits for Enter there and before it opens the jaws to
  hand the key back (`MUST_CONFIRM`, even with `--yes`). Jaws that close on nothing abort.
- the key turn's look and walk up (`key_plan.WalkUp`) to the insertion's stance, the refit, then StandDown. Held until
  sport mode's `body_height` is 10 cm or more down and still, then the lidar refits the door from the lowered base and
  the insertion is planned there: 60 and 25 mm out, at the touch, seated, and 4 mm off across either way, trunk and
  head camera clear and holdable. The odometry's pose keeps its standing height, so the attempt lowers its own base by
  the measured drop (`InsertAttempt.base_script`).
- **by touch, not by position** (the bare keyway's half-millimetre is past anything the robot measures). In along the
  axis at 2 mm/s until the key's encoder depth falls 3 mm further behind its command than in the first second (the sag).
  Where it stopped says where it is. Near the shaft's 10 mm out, the shaft is on the face, and an Archimedean spiral
  (0.4 mm pitch, 1.5 mm/s, out to 4 mm) searches across, pressed 2 mm into the face, until the key drops 1.2 mm. Near
  seated, it went straight in. Then in to seated, by touch again: stopped at least 8 mm past the face. The jaws open
  (the door pull's check that they did), the hand backs 6 cm straight out, home, StandUp and BalanceStand, fold. The
  hand's outer loop is the simulation's (1.5/s, 30 mm), along the axis only before the key goes in.
- the recovery after an abort: the key straight back out along the axis from where the encoders put it, home, stand
  up if lying, then the key handed back (Enter) and the fold.
- the door program: `plan.Command.posture` (sent once a change: StandDown 1005, StandUp 1004, BalanceStand 1002),
  `plan.Observation.body_height_m`, a hook on each phase change (`Attempt._on_phase_change`); a dry run lowers its own
  body height as StandDown would (0.20 m, P2Dingo's model, not measured with the D1 on). RViz draws the keyway at 45
  degrees, the key where the encoders put it, the search's spiral, and the stance window.

**Dry on the CPU model** (`demos/combiner_key/tests/test_insert_robot.py`, 10 tests). The keyway is a model: a rigid
face, no friction, 0.4 mm of play; the held key's shaft cannot pass the face anywhere else, nor its back face past
seated. The arm is put back where the key stops. The keyway is put 0.2-3.8 mm off the box's model. Results:
- one went straight in; three were found by the search at 1.6-3.6 mm out, in 13-67 s. All seated, 52-109 s from the
  look; the worst joint lag pressing was 1.3 degrees.
- a keyway 6 mm off aborts with "no keyway within 4 mm", the key backs out, the dog stands, and the key is handed back.
- every stage short of the insertion hands the key back and folds.
- jaws closing on nothing abort before the walk; a dog that does not lie down aborts.
- the plan solves lying for a dog standing 0.33-0.39 m over the box, the shoulder under 0.9.
- the arm stays off the head camera throughout.

The insertion's 10 and the key turn's 20 robot tests, the key demo's 9, the arm turn's 14 and the door's 45 pass.

**Not run on the dog, and not shown by any of it**:
- StandDown with the D1 on its back and the key in the hand; the real drop.
- whether the lidar's refit from the lying base finds the door's face.
- the real keyway's place: the seated key 17 mm off the door is the printed adapter's model, which the turn's fingers
  found good only to the platform's ~1 cm. The touch measures the depth.
- what the D1's position servos push with at 3 mm of lag.
- whether a printed key goes into a cam lock from a spiral at all, with friction, a round shaft, and the wards' 45
  degrees set by the hand to the wrist's accuracy (a degree or two at best: the wards have no play to spare either).
The keyway is a frictionless model, so the dry search's success says the logic works, not that the key will go in.

### 2026-10-21 · The insertion's first live runs: stale odometry after the hand-in; the lock measured

**Two live runs** ([1](#/week/6/run/20261021T095354_insert_insert_live),
[2](#/week/6/run/20261021T095539_insert_insert_live), `d1doginsert --live --yes`). Both looked (the stance 0.44 m
away, 0.31 m left, which the walk up accepts), unfolded, and waited for the key: 43 s each. Then the jaws closed on it,
and about a second later, at the walk's first step, both aborted: "odometry 1.41 s old", "1.80 s old". Nothing walked;
the second was quit at the prompt. The arm link's log shows the arm streamed at 10 Hz the whole time, so the node's loop
was not stuck. Its trace was lost after the pause, written buffered and never flushed on the quit.

*Cause (most likely; not seen directly)*: the insertion's hand-in phase is named `grip`, as the door pull's is. The node
starts the lidar's door-angle reading (`mid360.door_angle`) on every frame through a `grip` phase. The insertion paused
inside `grip`: the wait for the key began after its first step, since the unfold ends early on arrival. So the reading
ran back to back for 43 s. It costs 15 ms a frame on the PC over the door's points, several times that on the dog's CPU,
in a Python thread contending for the interpreter lock with the sport-mode state callback. That callback was already
held off 0.3-0.5 s once in these runs, while the look's fix was computed. The door pull's own grip lasts about a second.
Now the reading is the door pull's only (`plan.Attempt.reads_door`, False for the key and the insertion), the trace is
flushed every step, and odometry over 0.5 s old is logged once in any phase, not only aborted on in a walk.

**The lock, measured** (Lukas, callipers; his photo kept as
[`screenshots/2026-10-21_cam_lock_calipers.jpg`](screenshots/2026-10-21_cam_lock_calipers.jpg)). The photo: the
lock's chrome cylinder in a black bezel on the box's white door, the calliper's jaws across the bezel reading 27.5 mm.
Lukas: the keyway is 8.7 mm deep and the lock's face 4.5 mm proud of the door, "that puts some of it in the door". The
keyway is "exactly the same as the printed adapter" (its outline: the shaft and the two wards, at 45 degrees); it is
hard to see in the photo. I first misread it as a C-shaped ring. The insertion now seats the key's back face 5.8 mm
off the door: the key's 10 mm shaft bottoms out in the 8.7 mm keyway (`insert_plan.LOCK_FACE_OFF_DOOR_M`,
`KEYWAY_DEPTH_M`). The touch-to-seated travel is 8.7 mm, and seated by touch is 6.7 mm past the face. The model before
had the printed adapter's 17 mm and the shaft's 10. The key turn's model still has the adapter's 17 mm. Its push landed
on the platform anyway (the platform 35 mm deep), but that geometry is now known to be 11 mm off. Dry, the insertion
tests pass with it (the touch at 9.6 mm by the attempt's own model, the keyway found 0.2-3.8 mm off and seated, 41-109 s).
The play stays at Lukas's earlier "under 0.5 mm": a looser keyway only makes the search shorter.

### 2026-10-21 · The key turned and the door pulled open in one run; the insertion's third live run, and a hand-in without Enter

**Runs found unrecorded** when fetching the dog's records, now recorded:
- [the door pull alone](#/week/6/run/20261021T084730_pull_live): Lukas's "only the door swing". It aborted after a
  0.44 m walk in: "the lidar puts the door 63 mm and 0.4 deg from where the look did". Across every door pull the
  refit's correction is 12-14% of the walk in (13 mm over 0.11 m up to 63 mm over 0.44 m), the same odometry
  under-count as the key's walk up. This walk was the longest, past the door pull's 60 mm refit limit. Nothing in the
  door pull's code had changed (Lukas asked); standing the dog 15-20 cm nearer avoids it. The door pull's walk in is not
  yet sent short as the key's is.
- **[The key turned, then the door pulled open](#/week/6/run/20261021T085022_key_turn_door_pull_live)** (`d1dogkeydoor
  --live`). The key: walk up and refit 58.4 mm, turned to the stop, the fingers' measure 46.1 degrees, verified.
  `key_clear` kept the arm 29.3 mm (the proxy's margin) from the head camera on the way up and 77.6 mm home, with no
  contact. Then the door pull with no new look:
  - the walk over in one stop, 1.8 cm from the door pull's stance;
  - the refit 15.0 mm, the box 0.72 m off at -2.0 degrees;
  - unfold, reach, grip, a 3.0 s pull (commanded 64 degrees at its end), hold, release, ungrasp, home, fold.
  The lidar's door reading went past 100 degrees at its maximum, not a believable angle, so it is not evidence of how far
  the door opened. Lukas watched the run; his account of the door is not in this log.
- [The same, before it](#/week/6/run/20261021T084501_key_turn_door_pull_live): the key turned and verified (45.4 degrees),
  the walk over in one stop, the refit 16.7 mm. In the pull, the run aborted: "the arm is 22 deg from where the IK wants
  it: loaded". The lidar read the door at under 2 degrees: the door did not open, and the hand stayed on the handle.
  Why it was held is not separated: the latch, the grip or the stance (the box 0.73 m off, outside the simulated
  0.67-0.71).
- [A dry insertion on the dog](#/week/6/run/20261021T095142_insert_insert_dry): it ended at `key_in`, "the key went 5 mm
  past where it should seat without touching anything: no lock there?". A dry run has no lock to touch. A dry run now
  takes a key that goes all the way in untouched as seated (`InsertAttempt.dry`); a live run still aborts.

**The insertion's third live run** ([run](#/week/6/run/20261021T100605_insert_insert_live), after the door-reading fix
above). The grip wait now came in 5 s. The odometry during the hand-in was fresh (median 3 ms, against 0.8 s through the
second run's grip), which supports the door-reading cause. Two new failures:
- *The key*: the jaws stalled closing at -1.8 units and the grip check passed (empty was then within 3 units of shut,
  -19.8). As the walk began, servo 6 read -19.4: the jaws were shut on nothing. The key had slipped out or fallen, or
  the -1.8 stall was on Lukas's fingers holding it. This is not separated.
- *The odometry in the walk*: 0.12 s old at the median, 0.375 s at the abort ("odometry 0.37 s old"). The key and door
  walks had 3 ms. The tick kept its 10 Hz. Every callback of the node shared its one default callback group, so the
  sport-mode state waited behind the camera's tag detection and the lidar. Why this walk and not the key's is not
  known.

**Changes:**
- the odometry and the leg states, the camera and the lidar each have their own callback group (five executor
  threads). Odometry over 0.5 s old is logged in any phase.
- **the hand-in without Enter** (Lukas: "can you make it automatically grasp so I dont have to press enter while holding
  the tool in place", then "3 second countdown is plenty"):
  - `load`: the jaws open and still for 3 s, a countdown in the log, then `grip` closes them.
  - empty (within 6 units of shut) after closing, they open and count down again, three tries.
  - a key that leaves the jaws (empty for 0.5 s) before they are meant to let it go aborts "the key is no longer
    between the jaws". The recovery then folds without handing back.
  - the hand-back has a 3 s countdown before the jaws open. No Enter anywhere in the insertion now.
- dry runs' key reads -1.8 units, the dog's one stall, which may not be a hold of the bar.

Insertion tests: 13 (new: a key taken at the second grip, a key that slips out, a lock that is not there live and dry).
The key turn's 20, the door's 45 pass.

### 2026-10-21 · The insertion's fourth live run: the key held, the dog lay down, and sat back

[Run](#/week/6/run/20261021T101715_insert_insert_live), `d1doginsert --live --yes`, with the countdown hand-in and
the callback groups:
- *the hand-in*: the jaws closed on the key at 5.3 units and held it (-1.0 to -1.1 through the walk and lying), the
  first try.
- *the odometry*: through the walk 2 ms old at the median, 10 ms at worst (0.12 and 0.375 s in the run before). The
  separate callback groups fixed it.
- *the walk up*: 0.42 m in one stop, 1.5 cm from where it was sent; the refit 46.3 mm and 1.7 degrees (11% of the walk).
  The seated key 0.49 m ahead and 7 cm right, in the window.
- *StandDown*: sport mode's body height went from 0.315 to 0.067 m, **0.249 m down** (the dry runs assumed 0.20). The
  odometry saw the base turn 5.4 degrees and move 1 cm. The lidar's refit lying moved the box 26.6 mm and turned it 1.1
  degrees. After it, the seated key was 0.53 m ahead and 3 cm right, the door +2.0 degrees off square (-2.3 standing):
  the dog sat ~4 cm back and turned ~4 degrees. Lukas: "sitting down usually moves the robot abit backwards as it sits".
- *the abort*: the lying window (5-15 cm right) refused 3 cm. On the CPU model at the dog's lying height (0.09-0.13 m
  over the box's root) the insertion solves over all of 0.40-0.60 m ahead and 0.10 m left to 0.20 m right, the shoulder
  at 40-87%. The window was stricter than the arm.

Now: lying, no window; the insertion's plan from where the dog lies decides, and its refusal says where the dog is. The
walk up stops 4 cm nearer (`LIE_BACK_M`) for the dog to lie with the seated key ~0.48 m ahead, where the shoulder is
under ~60%. The turn is not allowed for, one run giving only its sign. The window after the walk is widened to
0.38-0.56 m and 0-20 cm. Dry runs drop 0.25 m, and their dog sits 4 cm back and turns 4.5 degrees (the odometry seeing
only the turn). The 13 insertion tests pass with it: keyways 0.2-3.8 mm off found and seated, 46-114 s from the look.

### 2026-10-21 · The camera on the wrist: the look from the arm, and the keyhole found by the bezel

Lukas moved the D435i from the head to the D1's wrist bracket ("do you think we could segment the keyhole reliably
(choose the most robust method) so that we can direct the arm better"). His screenshot (in the chat) is the wrist
camera's view of the box's door: the tag outlined green by the detector, the lock's black bezel left of it, the
warning label above.

**The mount.** It is the 30-degree version: `demos/cup/pick_demo/assets/mounts/wrist_mount.json` is the bracket's CAD
(2026-10-09, `bracket.py`: the camera's front 30.00 degrees below the approach axis). It is not measured; the
horizontal placement it replaced (2026-09-17) was aligned by eye. A horizontal camera, about 5 cm above the key's axis,
would have the lock below the image at every insertion distance (rows 800-1270 of 720). The 30-degree one keeps it in
view: row ~300 at 85 mm out, ~500 at 25 mm.

**The look through the wrist** (all three tasks; `node.py --camera wrist`, the default):
- each tag frame is placed with the camera's pose from the arm's measured joints (`plan.wrist_camera_base`); frames
  while the arm moves are dropped. `plan.make_fix` takes (base, detections, camera) frames, and the head's two-tuples
  still work.
- live, the arm first goes to a look pose (`plan.look_pose_servo_deg`, nearly in the arm's plane: Joint1 -3, Joint4
  -4, Joint6 +3 degrees). The tag is in view with the box's root 0.7-1.4 m ahead. A centred camera needed Joint1 +45.
  The one Enter (none with `--yes`) now comes before this move.
- a dry run looks from wherever the real arm is.
- the odometry, leg-state, camera and lidar callbacks run in their own groups (above).
- tests: the look pose solves, is clear of the trunk, and has the tag in view; a look through the wrist camera gives
  the head camera's fix for the same box.

**The keyhole** (`demos/combiner_key/keyhole.py`; the insertion's new `key_view`):
- *the method*: model-based. The lock's bezel (a 27.5 mm black disc, the keyway concentric with it) is the known circle
  on the known plane. Candidate centres within 12 mm of the model are each projected as that circle and scored by the
  image's contrast across the rim (dark inside, light outside), on a coarse grid then finer, the rim step narrowing
  2 to 1 to 0.4 mm.
- *why this, not thresholding or a learned segmenter*:
  - no thresholds to tune to the light, and no training data;
  - the circle is projected, so the camera's 30-degree view needs no special handling;
  - the key and fingers in front of the rim only take points out of the score.
- *acceptance*: at least 40% of the rim seen, and the best centre 1.5 times the score of the best one 4 mm away.
- *depth*: not searched. Seen 30 degrees off, depth and place trade off: on rendered frames a 3 mm depth error became a
  1.5 mm sideways one while the depth was searched. The touch gives the face's depth, and the camera's centre is
  carried along its ray to it (miss 0.03-0.06 mm on rendered frames).
- *self-calibration* (`logs/combiner_key_calibration/keyhole.json` on the dog; `d1fetch` brings it back): the residual
  between where the key seated and where the camera put the keyway, a running mean capped at 6 mm. It mixes the mount's
  error with how each hand-in put the key in the jaws.
- *on the CPU model*: rendered frames, from where the arm truly is with the dog sat back and turned. Keyways 3.5-9 mm
  off the model were seated with no search: the camera's offsets were within 0.3 mm of the truth in the attempt's own
  frame, and the residual kept was 0.01 mm.

**A fix found on the way.** The hand's outer loop had integrated along the key's axis while `key_pre` moved the hand
in. It wound up, and the key ran 6 mm past its command into the keyway. It now integrates only in `key_view` once the
arm settles (all axes) and across the axis while the key goes in. The dry keyway now holds a key inside it against its
sides, instead of pushing it back out onto the face.

Tests: the keyhole's 4, the insertion's 14, the door's 35 (the wrist camera's 2 new) and the key turn's 20 pass.

**Not run on the dog. Not tested by any of it:**
- the real image: the bezel's black rubber sheen, the chrome, the wrist's light, blur at 17 cm from a fixed-focus lens;
- the mount's real error;
- the look pose on the dog;
- whether the tag still resolves well from the wrist.
The keyhole's render is a flat ray-cast, with no lighting and no lens distortion.

### 2026-10-21 · The first look through the wrist camera: a finger across the tag

[Run](#/week/6/run/20261021T113412_look_live), `d1dog --stage look --live`: the arm aimed (reached the look pose,
-3.0/-6.1/59.0/-3.7/-52.5/2.5 degrees) and looked for 2 s: 26 frames, **no tag in any**. Lukas's screenshot of the
camera's view at the pose (in the chat): the door, the tag a little above the image's middle, and a dark finger rising
from the bottom of the image to the tag's centre, the jaws open. Lukas: "the gripper blocks the view: we should have the
gripper fully closed while looking". The aim had sent the jaws open (the door pull's opening).

Now the jaws are shut through the aim and the look. And the look is aimed lower, at (0.80, 0.04, 0.05) m from the base
instead of (0.80, 0.04, 0.10), so the tag is in the image's upper half: rows 50-370 of 720 with the box's root 0.9-1.3
m ahead. By the bracket's CAD mount the fingers are in the lower right of the wrist camera's view whatever the arm does.
A test checks the shut fingers stay off the tag for boxes 0.9-1.3 m ahead and 0.12 m either side.

*Mount check, rough*: by the CAD mount the open +y finger should be at columns ~420-670 of 1280; in the screenshot it is
at ~600-920 (its half-scale image doubled). Either the mount is ~3 cm or ~10 degrees off there, or the jaws were not as
far open as modelled. It is one picture, not a measurement. The keyhole's self-calibration exists for this.

Also recorded: [a dry insertion on the dog](#/week/6/run/20261021T102422_insert_insert_dry) before the camera moved,
refused at the look: "no door face round the tag in the lidar", the box 0.74 m off at -26 degrees.

### 2026-10-21 · The insertion with the wrist camera: the bezel not where the camera looked, and no touch felt

[Run](#/week/6/run/20261021T114128_insert_insert_live), `d1doginsert --live --yes` (the jaws shut through the look).
The look found the tag through the wrist camera (the stance 0.37 m ahead, 0.25 m left).
- *the hand-in*: the first grip closed on nothing (-19.4 units, opened and counted down again); the second held
  (0.8 units).
- *the walk and lie-down*: the walk up stopped in one step 1.9 cm off; refit 54.6 mm standing and 21.8 mm lying; the
  drop 0.248 m. Lying, the seated key was 0.46 m ahead and 5 cm right, the door 3.0 degrees off square. The plan
  solved, the shoulder at 48-62%.
- *`key_view`*: 6 frames, **the bezel found in none**: 0-1% of the rim seen dark inside, wherever the search put it.
- *`key_in`*: the key went in by the encoders from 26 mm out to 3.65 mm *past* seated, at 2 mm/s. The joints stayed
  0.6-1.3 degrees behind their commands the whole way, never loaded. Then the overrun abort: "the key went 5 mm past
  where it should seat without touching anything".

Lukas's RViz screenshot at the abort: the wrist camera's frame shows the lock's bezel at the image's *top* edge, cut
off, a finger from the bottom, and a white block (the key's bar, it seems) beside it. By the bracket's CAD mount the
bezel should be in the lower half at that pose (row ~600 of 720), so the camera sees it ~25-30 degrees higher than the
mount says. With the earlier look's finger ~200 px right of where the mount puts it, **the CAD mount is several
degrees and/or centimetres off**: the keyhole search's 12 mm window cannot reach a bezel that far from where the mount
projects it.

And *no touch*: by the encoders the key went 8 mm past the lock's face and 4 mm past the door's, with the arm never
loaded. So the key did not follow the hand: it slid back between the jaws (PLA pinched top and bottom, pushed along
the shaft, the palm ~4 cm behind the bar), or it missed the box. Every depth the insertion measures assumes the key
fixed in the hand. The arm's lag cannot see a key that slips.

RViz: the camera's field (the purple frustum) was still drawn from the head's mount; it is now drawn from the wrist
(the arm's feedback), and the image panel is renamed "Camera (wrist)" (the PC's RViz needs relaunching for the name).

### 2026-10-21 · The door pull through the wrist camera, and its walk in sent short

[Run](#/week/6/run/20261021T115745_pull_live), `d1dogdoor --live --yes`. The arm aimed and looked with the jaws shut:
the tag was found, the box 0.69 m from the stance at -1.2 degrees, the door -4.6 off square, a walk in of 0.36 m. That
is the first look through the wrist camera to find the box. After the walk in, the refit and re-anchor put the box
**0.63 m** off (the door -6.2 degrees off square), under the door pull's 0.65 m: "move the dog back 0.06 m". `r`
folded the arm.

That is the odometry's under-count, not the camera. Across the door pulls and key walks up the refit has put the door
12-14% of the walk nearer than the look did, and 13% of 0.36 m is 4.7 cm, plus the gait's carry. The head camera's
door pulls had walked at most 0.29 m. The door pull's walk in is now sent short as the key's is
(`plan.ODOM_WALK_SCALE` = 0.87, which `key_plan` now takes from it). The fix's stance is placed at 87% of the walk in,
so a look's "box ... from the stance" now reads 13% of the walk long; the dog walks the rest. 36 door robot tests pass
(the look's stance test now expects it).

**Then** ([run](#/week/6/run/20261021T120314_pull_live), the walk in sent short; and
[one before the change](#/week/6/run/20261021T115613_pull_live), which also walked 5 cm too far, 0.64 m). With the
walk in sent short the walk landed right: the box 0.70 m off after the refit, which moved it 43.2 mm, the 13% of the
0.33 m walk the fix now leaves for it. The refit's limit is raised to 8 cm for that. But the hand went in **too low and
missed the handle**: the pull began, the lidar saw the door stay shut ("the lidar has the door at 3 deg, the script at
50") and the run aborted. The jaw centre at `arrive` was 0.287 m above the base by the commanded joints. In the head
camera's nine door pulls it was 0.319-0.341, the same handle. So the wrist camera's look puts the box **3-5 cm low**.
About 3 degrees of pitch at the tag's 0.7-0.9 m, or centimetres of position, in the mount or the arm's pose at the
look. The lidar's refit corrects the door's depth and heading, not its height. The same error explains the
insertion's camera seeing the lock at the top of its image: the hand ~4 cm below the lock is ~20 degrees at 10 cm.

### 2026-10-22 · The look moved to the Go2's front camera; its lens fitted on one stand; Isaac's small-focal bug

Lukas: "for the insertion task, we had troubles with misalignment of the realsense. can we instead use the go2 front
camera to locate the box (the simulation tests then need to be updated with the higher fov lens etc) and then we use the
realsense on the arm just for the key insertion part?" On the lens: "use a nominal model for now; calibrate properly
later". The keyhole search was to stay as it is (no tag-based correction of the wrist mount).

**Why.** The wrist camera's look places the tag through the bracket's CAD mount and the arm's joints. The last door pull
went in 3-5 cm low and the insertion saw the bezel ~25-30 degrees off where the mount put it (above). The Go2's front
camera is in the trunk: no arm move to aim it, nothing between tag and base but a fixed mount.

**The camera on ROS.** It had no topic: the head sends H.264 over RTP multicast (230.1.1.1:1720) on the Go2 wire.
P2Dingo now runs a `p2dingo-front-camera` container that decodes it and publishes `/go2_front_camera/image_full/compressed`
(1280x720 q90, local to the dog) beside the 960 px topic for the zenoh link. On the way it was found that the dog's new
USB WiFi dongle, as its default route, had taken the multicast join off the wire: no packet reached FFmpeg, and the
website's front camera showed "No frames yet". Both now join on the wire through a relay (P2Dingo,
`maps/server/go2_rtp_relay.py`).

**What changed here.**
- `demos/combiner_door/front_camera.py`: the URDF's mount ((0.330, 0, 0.040) in `base_link`, looking ahead), the lens
  as OpenCV's fisheye model, the simulator's render.
- `apriltag.TagDetector(fisheye_distortion=...)`: corners undistorted onto a pinhole image before IPPE.
- `perception.box_in_spawn` defaults to the front camera; `plan.FRONT_CAMERA_POSE_BASE`.
- `node.py --camera front` (the new default; `wrist` and `head` kept): the look reads the front camera's JPEGs, the arm
  stays folded for it. The RealSense callback now serves only the insertion's keyhole view (wrist pose by the encoders,
  as before) and the look under `--camera wrist|head`. RViz's camera panel (renamed "Camera") shows the front camera,
  and the wrist frames through the insertion's key phases. `run_on_dog.sh` refuses if the front camera's container is
  not running.
- **Two tags at once.** The front camera's ~100 degree field held the box's tag and P2Dingo's charging dock's (both 36h11
  id 0, 100 mm) in one frame on the dog. `make_fix` averaged every detection, which would put the box between them. It
  now groups detections by where they put the box (`TAG_GROUP_M` 0.25 m), makes a fix per group and keeps the one whose
  lidar face is the box's door (`Fix.on_a_door`, the dock guard's own test); `Fix.other_tags` counts the rest.
- **Where to stand.** The front camera's field is only +-27 degrees tall, and the tag (0.46 m up the box) leaves its top
  closer in. Standing 0.30-0.34 m up and pitched +-2 degrees, the whole tag is in view from the box's root 0.95 m ahead
  (`plan.FRONT_LOOK_MIN_ROOT_M`), the door ~0.8 m off. The door pull's look band becomes 0.95-1.14 m; its no-walk band
  (0.65-0.76 m) and the key's "0.6 m from the door" cannot be looked from. A watch that sees no tag says to step back.

**The lens, fitted on one stand** (not a full calibration). A first guess, f 830 px from a rough door-width check
against the MID-360 (`front_camera.NOMINAL`), was replaced when Lukas stood the dog with the camera taped 0.61 m from the
box's front. Over 20 frames (the tag's corners scattered 0.07 px), f and k1 were searched so that the tag's plane comes
out 0.61 m off and the box's 0.63 m width, its side edges carried onto that plane, comes out 0.63 m:
**f 790 px, k1 -0.09** (`ONE_STAND`): plane 0.609 m, width 0.629 m, edges straight to 1.7 mm. f is pinned by the distance
(15 px is 1 cm of it), k1 by the width (0.03 is 7 mm). Not fitted to and agreeing: the lidar's door plane, 0.603 m from
the camera's URDF position, and its face tilt, 4.72 degrees where the camera saw the tag 4.7 off square (the dog was
lying, sport mode 0, trunk pitched 5.6 degrees). Principal point assumed at the centre, k2-k4 zero. `NOMINAL` read the
same stand 0.629 m and the door 0.605 m. The field is ~100 x 53 degrees. Two real frames are now test fixtures
(`demos/combiner_door/tests/data/`).

**In simulation** the front camera is rendered as a wide pinhole (1714 x 966) and remapped through the lens
(`front_camera.distort_render`), so the detector sees the 1280 x 720 fisheye the dog streams. The runs:
- [first run](#/week/6/run/20261022T102427_door_combiner_door_seed1_combiner_door), look 0.18 m back as for the head
  camera: the tag in none of 4 attempts, off the top of the image
  ([frame dump](#/week/6/run/20261022T102639_door_combiner_door_seed1_combiner_door)). P2Dingo's walking policy stands
  the base at 0.314 m (the trace), not the 0.40 the head camera's look-back test assumed. `perception.LOOK_BACK_M` is now
  0.40: the tag whole at all 200 placements, base 0.30-0.33 m and +-2 degrees (39 px to spare).
- [look 0.40 m back](#/week/6/run/20261022T103047_door_combiner_door_seed1_combiner_door): found 4 of 4, but the box
  **175 mm too near**, no door opened. The remap was consistent (the tag 0.433 m off in the pinhole render and the
  fisheye alike) and the frame showed the box that near; Isaac reported the intrinsics asked for
  ([dump](#/week/6/run/20261022T103414_door_combiner_door_seed1_combiner_door)). Not resolution
  ([1280 x 722 render](#/week/6/run/20261022T103637_door_combiner_door_seed1_combiner_door), 173 mm) and not aspect
  ([4:3](#/week/6/run/20261022T103925_door_combiner_door_seed1_combiner_door)). The head RealSense in the same scene put
  the box 8-15 mm off ([side by side](#/week/6/run/20261022T103755_door_combiner_door_seed1_combiner_door)), the prims
  were where configured ([transforms](#/week/6/run/20261022T104056_door_combiner_door_seed1_combiner_door)). What
  differed: Isaac Lab's `from_intrinsic_matrix` sets the horizontal aperture to 1.0, which made this wide camera's focal
  length 0.35 (the head's 0.95). **With focal 1.8 (aperture 5.2) the two agree to 5 mm**
  ([run](#/week/6/run/20261022T104222_door_combiner_door_seed1_combiner_door): look 6 mm, the door opened).
  `front_camera.USD_FOCAL_LENGTH`. Any wide simulated camera given Isaac Lab's default aperture is suspect.
- **[the door pull's 16 placements](#/week/6/run/20261022T104403_door_combiner_door_seed1_combiner_door), seed 1: 16 of 16 doors opened** and verified open by the lidar,
  no falls; the tag found in all 16 looks, the box placed 6.1 mm off (median, 7.8 max) and 0.13 degrees; the jaws at
  the lever 6.0 mm from the plan; the door 93 degrees at the end of the hold. The head RealSense's F-155: 16 of 16.

**Tests.** The door's 62 (front camera: mount, lens against `cv2.fisheye`, the render's coverage, a rendered tag measured
through the lens within 8 mm where a pinhole reading is 3-15 cm out, the look-back, the real frame, the taped stand to 1
cm; robot: the look through the front camera, the stand distance, the default; the box and the dock in one look), the
combiner's 56, the key's 63 (23 of them in the Isaac environment) pass.

**On the dog.** The look's path ran live in a throwaway container (the new code copied to `/tmp`, not the dog's
checkout): 30 frames decoded from the full-resolution topic, 61 lidar scans, but no tag. The room's lights had gone off
(the frame is black). **Not yet run there with light, not yet through `run_on_dog.sh`.**

**Not done.** A real calibration (`cv2.fisheye.calibrate` on a chessboard); the keyhole search's 12 mm window against the
wrist mount's error, which this does not touch (Lukas: leave it); the front camera's view with the arm unfolded in front
of it (the look happens folded; nothing after it needs the camera).

### 2026-10-22 · The insertion through the front camera on the dog: the first frame waited for; walk first, hand in at the stance, no lidar lying

Three live or dry insertion runs with the front camera's look, and what each changed:
- [live](#/week/6/run/20261022T115135_insert_insert_live): "no tag in the look", **0 frames**. P2Dingo's front camera node
  decodes only while subscribed and its first frame comes 3-5 s after the request (the stream joined, a keyframe
  awaited); the look began 0.2 s after this node subscribed and was over in 2 s. The look now waits for the first frame
  (`FRONT_FIRST_FRAME_TIMEOUT_S`, 20 s, then a message naming the container).
  [Dry look after it](#/week/6/run/20261022T115318_insert_look_dry): the tag in 21 of 21 frames, the dock's tag (also in
  view) set aside (21 detections, not 42), the lidar's face 4.9 mm rms, the tag's heading 0.3 deg from it; out of range by
  0.08 m. The startup line said "NOMINAL lens model" for the fitted one; it now prints the model's own label.
- [live](#/week/6/run/20261022T115429_insert_insert_live): the key handed in, the walk up 0.33 m, then **"the lidar puts the
  door 23 mm and 6.9 deg from where the look did"**: the abort is the STANDING refit after the walk (its message; the
  lying refit's says "from where the walk did"), logged the same millisecond as the lie-down's posture request. Carried
  at `KEY_CARRY_IN_BASE` (0.28 m ahead, 0.25 up: ~10 cm ahead of and above the MID-360) the hand and key shadow the door
  the fit looks at, and the leftover points tip the line. Lukas: "we cant trust the lidar after sitting down, as the tool
  obscures it"; the same holds standing with the key out in front. His choice of the fixes offered: **walk first, then
  hand in**.
- Now (`insert_plan.stage_phases`): `walk_in` and `walk_settle` (the refit, a clear view) come first, the arm held at the
  fold (`_folded_walk`, as `DoorAfterKey` walks); then `unfold`, `load`, `grip` at the stance, the key handed in beside
  the box. Lying, **no lidar refit** (`_replan_lying`): the fix rides the odometry, plus whatever part of the measured
  sit-back (`LIE_BACK_M`, 4 cm straight back, the first lie-down; its 4.5 deg of turn the odometry saw) the odometry did
  not report, applied as drift in the odometry's frame. The record's `refit_lying` event says so, with
  `odometry_back_mm` and `sit_back_added_mm`. What the prior misses of a sit-back is the keyhole view's and the touch's
  to take up.
- Tests: the insertion's 16 (two rewritten for the order: a slip now in `lie_down`, an empty grip aborting before
  `lie_down`; two new: the arm folded through the walk and its refit, and lying with the sit-back added and the key
  seated), the key's 42 on the system Python, the door's 62.
- [Dry, the new order](#/week/6/run/20261022T120615_insert_insert_dry): the dog, left facing P2Dingo's charging dock by
  the run before, saw only the dock's tag, and the look refused it ("not on a door the box's size... carries on past the
  door's edges (120%)"): the dock guard, working as meant. The new order is not yet run on the dog.

### 2026-10-23 · The keyhole found in the wrist image first, scored on two hand-held clips

Lukas moved the arm's RealSense round the box by hand with the website's arm panel and recorded two clips through
P2Dingo's website (`realsense_1792672631585.mp4`, 58 s, daylight; `realsense_1792672717000.mp4`, 40 s, dim red room
light; 1280x720 at 15 fps, the gripper's jaw always lower right, heavy motion blur in places). Asked for "a reliable
keyhole finder".
- `keyhole.find_bezel` cannot run on them (no arm pose), and on the dog it only looks 12 mm round the model's place. So:
  `demos/combiner_key/bezel_image.py` finds the bezel from the image alone: a dark disc with light all round (8 sectors
  each side of the rim, one of each allowed lost, on an image pyramid), an ellipse through its rim, five tests (rim
  support, contrast, light round it, seen, fit).
- Ground truth: the bezel tracked through each clip from seeds and every frame of the track looked at; 772 + 566 frames,
  and 66 + 25 with the lock out of view (`results/week_06/keyhole_clips/ground_truth.json`). Two of the tracker's own
  mistakes caught that way: it followed the "o" of the box's "shoals" for the first 4 s of the second clip, and a clutter
  blob through 768-868 of the first.
- What failed on the way, and what fixed it: a mean round the rim let clutter and the logo outrank a small bezel (the
  sector test); the blurred bezel's ellipse residual over the first gate (a RANSAC ellipse and a median residual); the
  gripper's rounded jaw against the door, a big dark disc, beat the bezel, and beside the bezel spoiled the light-round
  test (a fixed mask of the region dark in >85% of both clips' frames); ellipses mostly off the image won (the seen
  test); the "o" of the logo, the bezel's shape and size (the light-round test at 1.6-2.2 radii, where its letters are).
  Gates set from the labelled candidates (each about the bezel's own 5th percentile), not from looking at frames.
- Image alone, no prior (`python3 -m demos.combiner_key.robot.eval_keyhole_clips`): right 754/772 and 565/566, wrong 3/0,
  missed 15/1, found in 1/66 and 3/25 frames with the lock out of view; 85 ms a frame (PC, every size).
- `find_bezel(wide_m=0.045)`: the image's find inside the projection of 45 mm round the model's place (its size within
  0.6-1.6x the model's), its ray onto the door's plane, the rim search centred there; falls back to the model's place when
  the image finds nothing. Against a model pushed E mm off (a pose made up per frame from the truth's ellipse): at 20 mm
  the old search is right in 2/152 and 3/114 frames, the new in 146/152 and 102/114; at 40 mm 0 against 144 and 102.
  40 ms a frame on the PC, 144 ms on the dog's CPU (71 ms without; every size with no window 0.6-0.9 s there).
- `node.py` uses it on both keyhole paths (`--keyhole_wide_mm`, default 45), the gripper masked; the keyhole stage
  searches while the model's bezel is within the window of the image (it needed it inside the image).
- Tests: `demos/combiner_key/tests/test_bezel_image.py` (7 clip frames: daylight, blur, beside the gripper, far, dim; the
  logo frame refused; a prior choosing between two; the wide search on renders 25-30 mm off, agreeing with the old
  search near the model, a disc beyond the window ignored), with the keyhole, insertion and door-robot suites: 70 pass.
- Not yet on the dog. The clips are hand-held and moving; the dog's `key_view` holds still at ~17 cm, which these gates
  have a margin for (the misses are blur, far and off the image's edge). The mask is fitted to how the jaw sits now.
- Then on the dog, Lukas's live insertion (20261022T132904, the new finder synced): at `key_view` the RViz frame showed
  the held key and the jaws where the model put the bezel, the bezel nowhere; 1 frame searched in the 2.5 s (the
  image's search: "none passed", then "more than 45 mm from the model's"); the key went 5 mm past seated touching
  nothing: "no lock there?". The topic arrives at 15 Hz (measured on the dog); the node searched ~1 a second.
- Lukas: "it needs a stage where the wrist pans around", then "the search stage should happen right after the robot sits
  down while the arm is folded - tilt the wrist up and pan left and right using the base. then continue with the
  extension and receiving of the tool". Now (`insert_plan`): walk, refit, `lie_down`, `lie_settle`, all folded;
  **`lock_look`**: servo 5 (the wrist's pitch) up and servo 1 (the arm's base) round to aim the folded wrist camera at the
  model's bezel (`aim_folded`), then servo 1 +-15 and +-30 degrees; each place held until 3 frames found nothing or 5 s;
  the first with 3 finds moves the box model across the door onto the bezel and the insertion is planned again; the
  image's search 80 mm round the model; back to the fold; then `unfold`, `load`, `grip` lying, `key_go`, `key_view` (one
  place, held for 3 searched frames, not a 2.5 s clock), `key_pre` ... The node records each search's phase and time.
- Simulation (the harness's renders): the camera 0.48 m from the bezel, 25 px radius, found from the aim; 20/-15 mm off
  the model moved it 20.2/-15.3 and the key seated with `key_view` seeing nothing. Tests: the order, the aim (only servos
  1 and 5 move, the pans), the look correcting a hidden-close-up case, neither look finding it; the insertion's 22 and the
  key, keyhole, bezel and door-robot suites, 72 more, pass. Not on the dog yet; from 0.48 m a degree of the mount's
  error is 8 mm on the door.
- [Live, `lock_look`'s first run](#/week/6/run/20261022T135352_insert_insert_live): it found a bezel-sized disc from
  the aim and the +15 degree pan (the same disc 250 px apart, 72-74 mm across the door from the model); from the aim the
  rim fit refused it as "not distinct" (4 mm is a fifth of its radius at 0.48 m); the look took ~20 s (each search
  1.1-1.4 s inside the node, 0.14 s alone on the dog); the key again went 5 mm past seated touching nothing; RViz's
  camera panel flickered (front and wrist frames on one topic). Lukas: he has been handing the key with its shaft on the
  gripper's roll axis, and it "always tried inserting to the top right": the robot assumed the simulation's grip, the
  bar 25 mm out from the shaft. Changes: the robot's key on the roll axis (`KEY_IN_JAWS_LINK6`, the load prompt); a
  bezel the image found skips the rim's distinctness test; the image search cropped to its window; the find placed at the
  depth its size gives (the circle projected on the model's door orientation, used past 6% of the range) and the box
  moved along the normal by it in both looks; `key_in` up to 40 mm past seated until it touches (`IN_SLACK_M`), a touch
  past half the keyway's depth beyond seated taken as the face; the wrist frame shown through `lock_look`; each search's
  frame kept in the run's `keyhole/`. Simulation: door 40 mm further than modelled measured 39 mm and seated; 30 mm
  further with no camera found by the touch and seated. 119 tests pass.
- [Live, the next run](#/week/6/run/20261022T142556_insert_insert_live) (ended at the hand-in: the jaws closed on
  nothing): `lock_look` saw the bezel from the aim, ~140 px right of the model's place, and refused it in all 23 searches
  as "more than 80 mm from the model's" (now 0.3-0.6 s a search with the crop). Lukas: "the search needs to be widened".
  Measured in its saved frames against the tag (homography of the 100 mm square): the bezel 247-251 mm left of the tag's
  centre and 35-36 mm below; the box model has 251 and 40. The offset (~66-77 mm right, 35-39 mm down, both runs) is in
  the box's placement against the wrist camera, not the box. `LOOK_WINDOW_M` 0.08 -> 0.15; `key_view` as wide when the
  look found nothing. 120 tests pass.

### 2026-10-23 · The foot-press commit pulled; this PC's findings renumbered; the press policy on the dog

- `git pull --ff-only` brought 880f0df (the turn Lukas meant, the key fine-tune, the general foot-press policy:
  F-176 to F-182), committed elsewhere while this PC held its own uncommitted F-176 to F-179 (the arm's turn in
  simulation and on the dog, the bezel in the wrist image, the front camera). Those four are renumbered, text unchanged:
  arm turn in simulation F-176 -> **F-183**, on the dog F-177 -> **F-184**, front camera F-178 -> **F-185**, bezel
  F-179 -> **F-186**, everywhere they were cited (`findings.md`, this log, `demos/combiner_key/README.md`,
  `demos/combiner_key/robot/README.md` and `key_plan.py`, `demos/combiner_door/README.md`). Any F-176 to F-179 above
  this entry now means 880f0df's. The pre-pull files are in `git stash` ("pre-pull local edits ...").
- `d1update` put `checkpoints/leg_press_force_r4_model_5999.npz` (F-180's round-4 final) on the dog, SHA-256 matching
  `checkpoints/SHA256SUMS`. Nothing has run with it there. The dog has only the shadow runner, which commands nothing;
  no program on it drives the legs, and the press task layer (`unifp_train/foot_press.py`) is torch, not on the dog.

### 2026-10-23 · The foot-press policy driving the legs: the program, not yet run live

Lukas: "I want to prepare to deploy and test it on the dog ... I will test by using my hand as the platform wherever
the dog puts its hand out"; then "build the live runner and press logic, starting at that 5 N cap". The first program
here that commands the legs (F-187; `demos/combiner_key/robot/README.md`, "The foot-press policy driving the legs").

- `press_layer.py`: `foot_press.PressLayer` and the leg environment's press episode (`_advance_approach`,
  `_run_layer`, the hybrid `_correct_foot`, `_hold_goals`, the press return) for one robot in numpy; the force the frame
  carries one step behind the layer, as `_step_forces` writes it; the reset's frame all zeros. The dog's rules: the
  press waits for Enter at the pre-contact point (15 s, then home); no stall by the end of the travel backs out unpressed
  (the trained layer presses the air there); levels and the first ramp's start capped at 5 N; a plate facing down
  refused (F-182). Two hand plates, each a row of the trained table: `palm_up` (contact 0.331, -0.202, 0.112 m, row
  14027) and `palm_facing` (0.422, -0.198, 0.221, row 10851).
- `lowlevel.py`: LowCmd and its CRC, the remote's keys, the motion switcher (1001/1002/1003), unitree_rl_lab's FixStand
  and Passive gains. `press_live.py`: the node (see the README's table); `run_press_on_dog.sh` (refuses to start beside
  any `d1t-*` container: the door program's arm link has the same name); `fake_go2.py`, a kinematic stand-in on ROS 2.
  `~/.bashrc`: `d1dogpress`, and `d1fetch` now copies `logs/leg_press/`.
- `tests/test_press_live.py`, 29 tests, OK. The layers on closed-loop kinematic foot paths agree to 1e-9 in float64; in
  float32, torch's timer sums 75 steps of 0.02 s short of 1.5 s and ends a hold one step later. The CRC is pinned to
  Unitree's `crc32_core` compiled with gcc (279564839, 1264208000; a whole LowCmd 736096623). The program on a fake
  clock: live whole run, dry rehearsal, the confirmation word, the stops (B, tilt, `a`), sport mode already released.
- Over ROS 2 on the PC (`/opt/ros/humble`, `~/unitree_ros2/install`, `ROS_LOCALHOST_ONLY=1`, domain 77):
  `python3 demos/combiner_key/robot/fake_go2.py --hand &` then
  `python3 demos/combiner_key/robot/press_live.py --policy checkpoints/leg_press_force_r4_model_5999.npz --live
  --arm_link_port 0 --hold_s 2` with `live` and Enters piped in.
  [First](#/week/6/run/20261023T012947_leg_press_live): stopped at the lie-down, the calves held at -1.8 by the fake's
  "hand" (its bug; the program stopped as it should, nothing released, no LowCmd).
  [Second](#/week/6/run/20261023T013017_leg_press_live): the whole sequence, 12,510 LowCmd at 500 Hz, 0 failing the
  fake's CRC check, 0 while sport mode held the legs, sport mode back at the end; the press "missed" (the kinematic legs
  follow the policy's targets into the floor: no dynamics, so no meaning).
  [Third](#/week/6/run/20261023T013325_leg_press_live), the stand-in lying where the dog lies (below): the same, 12,520.
- On the dog, dry (`run_press_on_dog.sh --policy checkpoints/leg_press_force_r4_model_5999.npz --rehearse --hold_s 2`,
  Enters piped over SSH; the dog lying in sport mode, nothing sent).
  [First](#/week/6/run/20261023T013144_leg_press_rehearse): stopped at once, "FR_calf_joint past its limit". One LowState
  read: the lying calves are -2.79, -2.78, -2.805, -2.847 rad, past the URDF's -2.7227 (the quaternion's w -0.998). Live,
  that stop would have put the first stand-up into damping: it now applies only while the policy drives, and the dry
  run never stops on the pose. [Second](#/week/6/run/20261023T013422_leg_press_rehearse): the pretend foot stood on an
  endless plane at the palm's height (the footfall 9 cm up), and the decoder's force estimate (19 N lying still, 46-59 N
  as the pretend sequence ran) released the press. The rehearsal's hand is now 10 cm across and met from in front, and
  the estimate is not used in rehearsals (it is the real robot's, not the pretend foot's).
  [Third](#/week/6/run/20261023T013526_leg_press_rehearse): every stage, the stall at the palm (z 0.112), 5.0 N held,
  home onto the footfall; inference 3.0-3.7 ms; the arm link's angles read ("arm measured").
- What this does not show: anything about the policy on the real legs, the handover's ramps on the dog, this firmware's
  motion switcher, or the remote's B bit (Unitree's documented layout; a `--rehearse` with B pressed should print
  `remote: ['B']`). The decoder's foot-force estimate reads 19 N on a dog at rest: its 25 N release is a backstop, not a
  sensor. Full suite after: 943 tests, the same 1 failure (`test_payload`), 15 skipped.

### 2026-10-23 · The first live run: the dog sat back; the handover made to end where training starts

Lukas ran `d1dogpress --rehearse` (dog lying, 66 s, no Enter; the remote's keys read 0 throughout, so B is still not
seen: [run](#/week/6/run/20261023T014219_leg_press_rehearse)), then `d1dogpress --live`: "after standing up, the dog
just collapsed". [The run](#/week/6/run/20261023T014425_leg_press_live) (F-188):

- Preflight: sport mode `mcf`, its stand 0.333 m (calves -1.48 to -1.54). StandDown to calves -2.50 to -2.55,
  ReleaseMode answered, `/lowcmd` quiet. The stand-up (kp 60/80/80 to that stand) reached 0.298 m, rear calves 0.18 rad
  short, 4.5° nose up; the blend to kp 25 / kd 0.5 with the stand as target sank it to 0.273 m, 6° nose up, sinking.
- The policy: 11 steps (0.24 s). First step FL hip +0.37, RR thigh +0.75, RL thigh +0.55 rad off the joints (from
  the same pose level and still: 0.54; from the trained settled stance: 0.25). The rear knees folded -1.86 -> -2.43
  against targets ~0.9 rad more extended, base 0.273 -> 0.212 m, pitch -6 -> -20.3°: the tilt stop (20°) put every
  motor in damping and the dog sank down. `tau_est` against kp (target - q) - kd dq: slopes 0.89-1.07 (the motors do
  what is asked). Loop period median 20.8 ms, max 25.8.
- A Ctrl-C at the damping prompt killed the LowCmd writer: rclpy's SIGINT handler shuts its context (`RCLError:
  publisher's context is invalid`). Harmless there (already damping); mid-policy it would have left the last stiff
  command on the motors.

Changes (`press_live.py`): the stand-up goes to the policy's default targets, the gains are blended with them kept, and
the policy starts once every joint is under 0.05 rad/s for 0.3 s, only if within 0.2 rad and 5° of the trained settled
stance (else it lies down on the stand-up gains); the lie-down ramps from those targets. `--presses 0 --stand_s 10`:
the policy only holds the stand. Tilt stop 20 -> 15°. `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`:
Ctrl-C is a KeyboardInterrupt; a writer that cannot publish says so; SIGTERM in damping exits leaving damping. The
test stand-ins sag as gravity against kp would (the trained settled offset x 25 / kp). 31 tests OK.
Against `fake_go2.py` over ROS 2: [stand only](#/week/6/run/20261023T015053_leg_press_live) (settled to 0.000 rad of
the trained stance, 5 s held, 9,505 LowCmd, 0 bad, sport mode back); [Ctrl-C 2 s into the policy](#/week/6/run/20261023T015445_leg_press_live)
-> damping, Enter -> handback, 11,519 LowCmd, 0 bad. A first Ctrl-C test (`20261023T015134_leg_press_live`, not
recordable: killed, `parts/` only) never delivered SIGINT -- a background job in a non-interactive shell ignores it.

Not shown: that the policy holds the real dog from the trained settled stance, or that the real dog settles within
0.2 rad of it under kp 25 (the first run's numbers suggest so for the rear calves, ~-1.8 rad). Next:
`d1dogpress --live --presses 0`.

Afterwards the dog did not answer its remote: the Ctrl-C at the damping prompt had exited before the handback, so sport
mode stayed released (CheckMode `{"name": ""}`, no LowCmd, the dog lying). SelectMode `mcf` sent at Lukas's word
(answered, CheckMode `mcf` after). `press_live.py` now says on exit when it leaves sport mode released
(`left_released` in the summary), and `sport_mode.py` (`d1dogsport`, `d1dogsport mcf`) checks or selects it.

### 2026-10-23 · The key holster beside the D1's base: registered, re-angled 6° steeper, the key drawn out in simulation

Lukas: "there are 3 new d1 stl files describing a holster next to the d1 arm base for the arm to aquire the lever in the
insertion task on the combiner box. can you add it to the sim and verify that the arm can slide out the lever from the
holster". Part-way, after the first numbers: "is it possible for it to grasp the key in its usual spot using this
holster or should I redesign the angle of the holster?"; then he re-exported the holster 6° steeper.

**Where it is** (`demos/combiner_key/holster.py`; `python -m demos.combiner_key.holster <the three STLs> --write`
writes `demos/combiner_key/assets/holster/`). The exports are in the D1-550 CAD's frame, as the wrist bracket's were.
CAD to `base_link`: x = y_cad, y = -x_cad, z = z_cad + 5 mm. The base flange's holes are 58.0 x 130.0 mm, centred
within 0.05 mm, as `base_link`'s are (58 across, 130 along x); its underside is at CAD z -5.0 mm; the folded arm spans
CAD y -0.283 to 0.141 m against `d1.urdf` at rest, x -0.285 to 0.142. So the holster is on the robot's right, beside
the base. The key in it is **v3 mirrored**, the hand the simulation's insertion and turn use: its bar fits by a rotation
to 0.026 mm median (0.071 p95; 2.7 max where the export's bodies differ), and v3 as drawn fits only as a reflection.
*Correction:* earlier in the session I told Lukas the key was v3 as drawn and the holster would not take the mirrored
one. That came from a fit that was a reflection (determinant -1), and it is wrong. The same reflection is what failed
the first simulation run (below). The holster is a sleeve round the bar's far 78 mm: a cup round its rounded end, two
rails behind its back face, and near the open end a C-section (back wall, lips over and under, a hook round the front
lower edge). Its inner faces stand 0.06-0.34 mm off the bar. The key can only slide out toward its shaft, and the
simulation models the holster as nine boxes (`HOLSTER_BOXES`).

**The first export: 35° above level** (CPU arm model). The dog's grip is the jaws round the bar's root at the shaft
(`robot/insert_plan.KEY_IN_JAWS_LINK6`). Gripping there needs the elbow at 92% of its range, past the IK's 90%, and the
draw then reaches 70-75 mm before Joint1 hits its limit; the bar's far end is 75 mm inside the holster's open end.
Gripped 20-30 mm along the bar, it draws 90-105 mm. The cause: the folded grip puts the wrist pitch beside Joint1's
axis, so drawing the hand out swings Joint1 (-0.56 to -2.17 rad over 90 mm). Tilting the holster about its cup traded
the elbow for the shoulder: 4-8° steeper drew 95-115 mm from the root with every joint at 88-92% of its range; 12° or
more and Joint2 is at its limit and the grip does not solve; turning it about the vertical did not help consistently.
Recommended: about 6° steeper.

**The second export: 40.8° above level.** The holster's shape round the key is unchanged (the same bounds in the key's
frame); the shaft end is 14 mm higher and 10 mm further back. `--check`: from the root, 100 mm drawn with every joint
within 0.895 of its range, the shoulder at most 32% of its torque, ≥ 46 mm off the trunk proxy, the open fingers
≥ 11.5 mm off the bar, the key ≥ 11.6 mm off the D1's base and ≥ 15.7 mm off its links. The unfold goes Joint1 first
with the arm still folded, then the rest. Straight from rest it dips 7 mm into the trunk proxy's 3 cm margin; this way
it keeps 36 mm and ≥ 7.9 mm off the holster and key. The draw solves with `hand_rotation` roll +1; **roll -1, the
insertion's own grip (`geometry.GRASP_ROLL`), does not slide onto the bar**. So the key comes out of the holster turned
180° about its shaft from how the insertion holds it.

**In simulation** (`demos/combiner_key/holster_env.py`, `run_holster.py`). The key demo's robot stands on P2Dingo's
walking policy. The holster is kinematic, carried with the base every physics step, and its contacts with the robot
are filtered out. The key is a free body (bar, platform, shaft and wards as colliders), held in the holster's pose
until `pre`, then left to PhysX. The arm runs the IK at 10 Hz through the D1's planner and servo model. Its sequence:
`turn` and `reach` in joint space, then `pre` (jaws open 45 mm out along the axis), `on`, `close`, `draw` (100 mm),
`hold`. Once the jaws close, the key demo's spring grip holds the key where they closed on it. The fingers collide with
it. `./demos/combiner_key/run_holster.py --headless [--attempts 8 --no_overview] [--holster_error_mm N]`; the last puts
each attempt's holster and key off from where the draw is planned, uniformly within N mm per axis of the base's frame.
- [First run](#/week/6/run/20261023T014641_seed1_first): **not drawn**. The key came 50 mm out with the grip's error
  reaching 55 mm. The IK fell back to position only from its first command (55 of 56 steps), because of the reflected
  registration. With the key frame left-handed, the hand the control loop built (`hand_rotation`) was the one the CPU
  check had solved turned 180° about the approach. Fixed by fitting the mirrored key by a rotation; `grip_rotation` is
  now `hand_rotation` itself, and a test checks it.
- [One attempt, with frames](#/week/6/run/20261023T015209_seed1_roll_fixed): **drawn 99.5 mm**. The jaws closed
  0.32 mm and 0.14° from the planned grip, the grip's error peaked at 2.1 mm (0.8 mm at the end), arm torque at 60%
  (Joint3), the hand at most 6.0 mm behind its command, fingers on the key at most 1.7 N. The base tilted at most 0.9°
  and stayed 0.309 m up. The IK held the full pose throughout.
- [8 attempts, holster as planned](#/week/6/run/20261023T015351_seed1_batch8_err0mm): 8/8 drawn, alike within
  0.02 mm (the scene is deterministic; this is repeatability, not spread).
- [8 attempts, holster up to 3 mm off](#/week/6/run/20261023T015450_seed1_batch8_err3mm): 8/8. The jaws closed up to
  3.2 mm from the planned grip, the grip's error peaked at 3.65 mm, torque at most 89%.
- [8 attempts, holster up to 6 mm off](#/week/6/run/20261023T015549_seed1_batch8_err6mm): 8/8. The jaws closed up to
  4.7 mm off across the bar (their opening leaves 10.6 mm a side) and 6.5 mm along the approach; the grip's error peaked
  at 5.6 mm, the fingers pushed at most 5.9 N. In one attempt Joint3 hit its torque limit for 0.9% of the draw (~0.6 s)
  and the key still came out.

`figures/holster_draw_frames.png` (the one-attempt run's overview camera, front right of the robot and above). 4.1 s
`turn`: the arm folded, the orange key in the dark holster beside the arm's base on the robot's right. 9.8 s `reach`:
the forearm raised, the hand above the key's root, jaws open. 11.5 s `on` and 15.4 s `close`: the jaws round the bar's
root. 18.0 s `draw`: the key 45 mm out, up and forward along the holster's line. 22.3 s `hold`: the key 99.5 mm out,
clear of the holster beside the base, the arm swung round to the right and back.

What it does and does not show:
- With the holster where the CAD puts it, or up to 6 mm off, the D1 can take the key by the bar's root and slide it out
  in PhysX on boxes, on the CPU arm model's kinematics.
- Not shown: the real arm reaching the holster. Its joint zeros are unvalidated (F-023), and the grip needs 89% of
  Joint2's and 83-89% of Joint3's range.
- The grip is a spring (600 N/m, capped at 10 N). It hides misalignment that a rigid real grip would push into the
  holster with the arm's stiffness. At 4-6 mm off, the real key may bind where the simulated one slid.
- Not modelled: the cup's curved inside, the holster's mounting and flex, friction beyond 0.3, and the wrist RealSense's
  99 g (this robot has no wrist mass; with it the elbow and shoulder are nearer their limits).
- Not checked: getting from the drawn pose to the insertion's start (`insert_plan.KEY_CARRY_IN_BASE`), and the
  insertion with the hand rolled 180° (the insertion program assumes roll -1).
- Tests: `demos/combiner_key/tests/test_holster.py` (9: the registration, the boxes against the key, the draw on the
  CPU model, the insertion's roll failing). The key demo's tests, 81 in all, pass.

### 2026-10-23 · The second live run: the stand only, refused before the policy

`d1dogpress --live --presses 0` ([run](#/week/6/run/20261023T020839_leg_press_live), F-191). Lukas: "It held for a
bit, but then collapsed momentarily before resuming a stance - then sat down". The policy never ran:

- Stand-up to the default targets on kp 80: 0.303 m, 4.6° nose up, the calves ~0.2 rad more extended than the trained
  settled stance. Blend to kp 25 / kd 0.5: 0.303 -> 0.287 m in its last 0.2 s (the momentary collapse), then down to
  rest at 0.261 m, 8.1° nose up, rear thighs 0.16-0.20 rad below the trained stance; still (0.01 rad) from 12.4 s.
- The gate: "not still after 3.0 s" (velocities at rest read 0.04-0.10 rad/s; the check now uses positions: on this
  record it calls the dog still at 12.48 s) and 8.1° past its 5°: the dog lay down on the stand-up gains -- the "sat
  down". Sport mode given back on Enter (SelectMode unanswered, CheckMode `mcf` after).
- Knee torques at rest front 5.6 / 4.6, rear 11.0 / 9.1 N·m (trained stance: 7.5 / 8.3). Foot loads through the
  Jacobians: 138 N at the kp-25 rest, 242 N at the kp-80 hold, 171 N in the trained stance; not one weight's, so
  something besides gravity -- joint friction, likeliest -- holds the dog where it stops.
- The policy's first step, open loop, from each start: 0.51 rad (kp-25 rest), 0.46 (kp-80 hold), 0.62 (F-188's start),
  0.25 (trained settled). Not started from this handover again.

### 2026-10-23 · The holster draw made faster: paced by the joints, not eased along the line

Lukas, watching [the draw in the viewer](#/week/6/run/20261023T020215_seed1_viewer) (3/3 drawn): "are we able to make the
movements faster? I notice in general on the d1 arm that slow movements are choppy and fast movements are clean and
smooth". The arm model reproduces what he sees. The D1's firmware plans each 10 Hz setpoint from rest (`motor_model`:
`D1_REPLAN_VELOCITY_RETENTION` 0, fitted to the F-035 streaming sweep, F-045). A joint asked for under ~2.5° a command
reaches it inside the 100 ms and waits for the next. At the first timing (18.5 s for the whole sequence), the draw moved
its fastest joint ~2° a command, and that joint sat under a fifth of its mean speed in 28% of 20 ms samples (its
speed's spread over its mean 0.99).

`run_holster.py --speed S` scales the moves' lengths, and the joint-space unfold's step per command up to 6.5°
(1.13 rad/s, under the measured 1.21-1.29). `--joint_speed V` paces the slide onto the bar and the draw so the fastest
joint turns at V rad/s throughout (`holster.joint_paced`, planned on the CPU model), instead of a smoothstep along the
line, which peaks at 1.5x its mean. The wrist roll turns 1.8 rad through the draw and Joint1 1.6.
Each attempt now records a `smoothness` per move. 8 attempts each, holster up to 3 mm off:

| moves | sequence | drawn | draw: fastest joint near-stopped | ripple | hand's worst lag | grip's worst error | torque at limit |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [1x even](#/week/6/run/20261023T020550_seed1_speed1_err3mm) | 18.5 s | 8/8 | 28% | 0.99 | 6.2 mm | 3.6 mm | never |
| [2x even](#/week/6/run/20261023T020648_seed1_speed2_err3mm) | 11.5 s | 8/8 | 17% | 0.66 | 37 mm | 17 mm | 8.9% of steps |
| [3x even](#/week/6/run/20261023T020729_seed1_speed3_err3mm) | 9.2 s | **2/8** | 17% | 0.64 | 53 mm | 45 mm | 26% |
| [2x, paced 0.6 rad/s](#/week/6/run/20261023T020935_seed1_paced0.6_err3mm) | 12.3 s | 8/8 | 8% | 0.52 | 7.9 mm | 3.2 mm | 0.2% |
| [2x, paced 0.8](#/week/6/run/20261023T021020_seed1_paced0.8_err3mm) | 11.1 s | 8/8 | 6% | 0.52 | 9.9 mm | 3.7 mm | 0.5% |
| [2x, paced 1.0](#/week/6/run/20261023T021101_seed1_paced1.0_err3mm) | 10.4 s | 8/8 | 5% | 0.49 | 14 mm | 9.5 mm | 2.3% |

Evenly faster lags: the wrist roll and Joint1 reach their ~1.25 rad/s, the hand falls behind the line, and in
simulation the spring grip takes up what a rigid real grip would push into the holster (17 mm at 2x). Paced, the same
speed-up keeps the tracking. The default is now 2x with the moves paced at 0.6 rad/s; [shown to Lukas in the viewer](#/week/6/run/20261023T021243_seed1_viewer_paced) (3/3 drawn, 99.4 mm, near-stopped 7.7%). The unfold at 2x still sits
near-stopped in 19% of samples (49% at 1x): its joint-space moves finish before their phase ends.

What it does and does not show:
- In the model, a move fast enough per command (above ~2.5° a command on its fastest joint) and paced to stay under
  the joints' ceiling is both smoother and as accurate as the slow one.
- The stop-start is the model's fitted replanning (retention 0, provisional in `motor_model`). Lukas's observation on
  the real arm agrees with it but does not measure it.
- Not on the real arm. The other joints in a Cartesian move still step a little at a time.

### 2026-10-23 · Next: the foot-press policy fine-tuned for the real stand (to resume on the training PC)

Lukas chose to fine-tune (F-191's second way). Not started here: round 4's `model_5999.pt` (with its critic) is on the
machine that trained it, `logs/unifp_train/20261021T140237_train_seed3_press_r4_force_s3/`; this PC has only its numpy
export. Its recorded arguments are in [the run](#/week/6/run/20261021T140237_train_seed3_press_r4_force_s3)
(`run.json`: `--leg_object press --leg_variant force --leg_actuator unitree_geared --leg_delay_steps 3
--leg_delay_min_steps 1 --leg_armature_range 0.003 0.02 --handle_fraction 0.7 --num_envs 2048`, resumed from round 3).
The plan:
1. Check in Isaac whether round 4's final holds the dog from the real starts (a frozen evaluation with
   `--spawn_states` built from F-191's record: the kp-25 rest at 0.261 m, 8.1° nose up, and the kp-80 hold at 0.303 m,
   4.6°; `logs/leg_press/20261023T020839_leg_press_live/press.npz`, joints in Unitree's order, roll/pitch by
   `leg_policy.roll_pitch`). If it holds there, the gap is the legs' dynamics; if not, the starts too.
2. Fine-tune from `model_5999.pt` with those starts in the spawn library (jittered), joint friction drawn per
   environment (the training's leg model has 0.01 N·m; the dog's rising and sinking loads differ by ~100 N, F-191:
   a few N·m at the knees is the guess to bracket), and the stand-up's handover as the dog does it.
3. The frozen press set and the key set against round 4, then the export (`export_policy.py`) for the dog and
   `d1dogpress --live --presses 0` again.

### 2026-10-23 · The handover simulated, round 4's final from the dog's starts, and a first fine-tune for them that tipped

Lukas: "another agent on another pc tested the policy and determined the start stance needs fine tuning. its committed
and pushed to main. can you run the training, and once satisfied commit and push back to main so the other pc can test
it". This takes up the plan above ("Next: the foot-press policy fine-tuned for the real stand"), steps 1 and 2.

**The handover simulated** (`run_unifp_train.py leg_handover`, new: `unifp_train/leg_handover.py`). Every robot goes
through what `press_live._stand_up` does to the dog, with nothing else acting:
1. Lying in the dog's pose (the calves from the run's "lying" event, the hips and thighs from the stand-in's LowState),
   held on the stand-up gains for 1.5 s, then passive for 0.42 s.
2. The stand-up to the policy's default targets: kp 60/80/80 ramped in over 0.5 s, targets smoothstepped over 2.5 s.
3. Held for 1 s, blended to kp 25 / kd 0.5 over 1 s, then the gate's 3 s.

The legs are the realistic ones (unitree_geared, armature 0.01 / 0.037, 1-3 steps of delay), with leg joint friction
drawn per robot. That friction is `--leg_friction_range`, also new: PhysX 5.6's static and dynamic friction effort in
N·m, solved implicitly, the knees ×1.917.

    python run_unifp_train.py leg_handover --task leg --leg_object press --leg_variant force --headless --num_envs 64 \
      --small_physx --leg_actuator unitree_geared --leg_armature 0.01 --leg_knee_armature 0.037 \
      --leg_delay_min_steps 1 --leg_delay_steps 3 --leg_friction_range 0 2

| run | lying | friction (hip, thigh) | hold, kp 80 | rest, kp 25, 3 s after the blend |
| --- | --- | --- | --- | --- |
| [first](#/week/6/run/20261023T022841_leg_handover_seed1_handover_friction0to4) | slumped (kp 20: calves to the -2.72 limit, base 0.08 m) | 0-4 N·m | 0.303 m, 4.7° nose up | 0.209 m, 11.8° nose up |
| [lying held](#/week/6/run/20261023T023034_leg_handover_seed1_handover_lie_held_friction0to2) | held on the stand-up gains, then passive | 0-2 | 0.306 m, 4.9° | 0.215 m, 13.5° |
| [with velocities](#/week/6/run/20261023T023526_leg_handover_seed1_handover_moving_friction0to2) | as above; the sink's states saved with their velocities | 0-2 | 0.306 m, 4.9° | 0.215 m, 13.5° |
| the dog (F-191) | | | 0.303 m, 4.6° | 0.261 m, 8.1° |

- **The hold matches the dog's.** It is within 3 mm and 0.3°, with or without friction. Standing up from lying with
  the feet planted where the lying left them (rear hips at ±0.3-0.5 rad) is what gives the stance at kp 80.
- **At kp 25 the simulated robot sinks further than the dog:** 0.215 m against 0.261. Its joints move the way the dog's
  did. Medians of q minus the trained settled stance, in Unitree's order (FR, FL, RR, RL), dog in brackets:
  - hips +0.07, -0.06, -0.18, +0.19 (+0.069, -0.062, -0.102, +0.103);
  - thighs -0.11, -0.14, -0.27, -0.28 (-0.053, -0.074, -0.162, -0.200);
  - rear calves -0.26 (-0.094, -0.013).
- **So the dog's rest is the stand-up's stance sinking.** It is not the trained settled stance (0.272 m, level), which
  only a drop at the default targets reaches.
- **The friction model doesn't hold it up.** More friction lowers the rest and levels it: in the second run, 0.221 m
  and 14.4° at 0.4 N·m or less, 0.210 m and 12.4° at 1.7 or more. PhysX's friction slows the sink but doesn't stop it
  short the way the dog stopped. Whatever holds the dog at 0.261 m isn't this model.
- **The sink is as fast as F-188's start.** It falls at up to 0.18 m/s (median 0.035) between 1.0 and 1.6 s after the
  hold, so F-188's 0.15 m/s is inside it.

**Round 4's final from those starts.** `logs/unifp_train/press/eval_start.sh` runs the frozen v2 set on the realistic
legs, on the final layer and plant, every episode from one of the stances. The stances are in
`demos/combiner_key/assets/dog_stand_20261023/`:
- `kp25_rest.json`: the dog's rest, its base height from the export's kinematics so every foot clears by 2 mm;
- `sim_handover_hold.json`, `sim_handover_sink.json`, `sim_handover_moving.json`: the third run's states.

A moving stance starts with all its velocity in an evaluation (`leg_env._reset_idx`, new).

| start | presses of 288 | fell | force error, median |
| --- | --- | --- | --- |
| classic stand ([control today](#/week/6/run/20261023T032947_leg_eval_seed1_press_r4_force_5999_real_control)) | 196 | 0 | 2.3 N |
| [the dog's kp-25 rest](#/week/6/run/20261023T023140_leg_eval_seed1_press_r4_force_5999_dogrest) | 197 | 0 | 2.2 |
| [the simulated hold](#/week/6/run/20261023T023220_leg_eval_seed1_press_r4_force_5999_simhold) | 202 | 0 | 2.2 |
| [the sink, at rest](#/week/6/run/20261023T023301_leg_eval_seed1_press_r4_force_5999_simsink) | 197 | 0 | 2.3 |
| [the sink in motion](#/week/6/run/20261023T023608_leg_eval_seed1_press_r4_force_5999_simmoving) | 203 | 0 | 2.1 |
| [the dog's rest, leg friction 1 N·m](#/week/6/run/20261023T023342_leg_eval_seed1_press_r4_force_5999_dogrest_fric1) | 139 | 0 | 3.7 |
| [the sink in motion, friction 1 N·m](#/week/6/run/20261023T023650_leg_eval_seed1_press_r4_force_5999_simmoving_fric1) | 165 | 2 | 3.1 |
| [the dog's rest, floor μ 0.5](#/week/6/run/20261023T023732_leg_eval_seed1_press_r4_force_5999_dogrest_mu05) | 215 | 0 | 1.9 |
| [the sink in motion, μ 0.5](#/week/6/run/20261023T023814_leg_eval_seed1_press_r4_force_5999_simmoving_mu05) | 195 | 0 | 2.3 |
| [the sink in motion, μ 0.4, friction 1 N·m](#/week/6/run/20261023T023859_leg_eval_seed1_press_r4_force_5999_simmoving_mu04_fric1) | 147 | 0 | 3.7 |

What these show and don't:
- **In simulation the starts aren't the problem.** From every stance the handover gives, at rest or sinking, round 4's
  final presses as it does from the stands it trained on, and never falls.
- **Leg joint friction is what costs it.** 1 N·m (1.9 at the knees) takes 58 presses off the dog's rest, mostly
  through under-pressing (bias -2.3 N). A slipperier floor barely matters.
- **Not why the dog sat back in F-188.** Sinking starts at the same speed don't reproduce it in simulation, and nothing
  modelled here does. The friction is a guess (F-191), not a measurement.
- **The control.** Round 4's final on the classic stand under today's code scores 196, the same per plate kind as
  yesterday's [196](#/week/6/run/20261021T234043_leg_eval_seed1_press_r4_force_5999_real_edges). The new code (friction, spawn velocities, the probe)
  changes nothing it was scored on.

**The first fine-tune tipped.** [The run](#/week/6/run/20261023T023944_train_seed3_press_ft1_force_s3) trained 1000 iterations from round 4's
`model_5999.pt` on round 4's recipe and layer (`--press_hold_correction --press_release_from_press_point
--press_plate_face_only`), with leg friction 0-2 N·m. 85% of its episodes started from `ft_starts.json`:
- 1,539 stances: the simulated sink (46%); the dog's rest and the path to it from the simulated hold, ±0.06 rad (36%);
  round 4's sport stands and walker stops (19%);
- a moving stance starts with a uniform share of its velocity;
- built by `build_starts.py` beside it.

    logs/unifp_train/press/train_one.sh force logs/unifp_train/20261021T140237_train_seed3_press_r4_force_s3/model_5999.pt \
      press_ft1_force_s3 1000 --spawn_states demos/combiner_key/assets/dog_stand_20261023/ft_starts.json \
      --spawn_states_fraction 0.85 --leg_friction_range 0 2 --press_hold_correction --press_release_from_press_point \
      --press_plate_face_only
    # train_one.sh adds: --num_envs 2048 --handle_fraction 0.7 --foot_push_n 15 --resume_leg_std 0.2
    #   --fixed_learning_rate 1e-4 --entropy_coef 0.002 --seed 3 --leg_actuator unitree_geared
    #   --leg_delay_min_steps 1 --leg_delay_steps 3 --leg_armature_range 0.003 0.02

Presses of 288 / fell:

| checkpoint | classic stand | the dog's rest | the dog's rest, friction 1 N·m | sink in motion, friction 1 N·m |
| --- | --- | --- | --- | --- |
| round 4's final | 196 / 0 | 197 / 0 | 139 / 0 | 165 / 2 |
| 6200 | [158 / 0](#/week/6/run/20261023T033028_leg_eval_seed1_press_ft1_force_6200_real) | [187 / 0](#/week/6/run/20261023T033109_leg_eval_seed1_press_ft1_force_6200_dogrest) | | |
| 6400 | [186 / 5](#/week/6/run/20261023T033151_leg_eval_seed1_press_ft1_force_6400_real) | [200 / 9](#/week/6/run/20261023T033233_leg_eval_seed1_press_ft1_force_6400_dogrest) | | |
| 6600 | [130 / 21](#/week/6/run/20261023T032120_leg_eval_seed1_press_ft1_force_6600_real) | [172 / 19](#/week/6/run/20261023T032202_leg_eval_seed1_press_ft1_force_6600_dogrest) | [141 / 4](#/week/6/run/20261023T032243_leg_eval_seed1_press_ft1_force_6600_dogrest_fric1) | [130 / 11](#/week/6/run/20261023T032325_leg_eval_seed1_press_ft1_force_6600_simmoving_fric1) |
| 6998 | [130 / 23](#/week/6/run/20261023T031428_leg_eval_seed1_press_ft1_force_6998_real) | [157 / 32](#/week/6/run/20261023T031511_leg_eval_seed1_press_ft1_force_6998_dogrest) | [142 / 25](#/week/6/run/20261023T031554_leg_eval_seed1_press_ft1_force_6998_dogrest_fric1) | [136 / 30](#/week/6/run/20261023T031636_leg_eval_seed1_press_ft1_force_6998_simmoving_fric1) |

Against round 4's final elsewhere:
- settled stance: [163 / 29](#/week/6/run/20261023T032407_leg_eval_seed1_press_ft1_force_6600_real_settled) and
  [155 / 27](#/week/6/run/20261023T031719_leg_eval_seed1_press_ft1_force_6998_real_settled), against 207 / 0;
- slide set: [41](#/week/6/run/20261023T032613_leg_eval_seed1_press_ft1_force_6600_slide) and [39](#/week/6/run/20261023T031927_leg_eval_seed1_press_ft1_force_6998_slide) of 96, against 56;
- key zero-shot from the dog's 0.33 m stand: [37](#/week/6/run/20261023T032653_leg_eval_seed1_press_ft1_force_6600_key_dog33) (2 fell) and
  [90](#/week/6/run/20261023T032006_leg_eval_seed1_press_ft1_force_6998_key_dog33) of 108, against 107;
- sink in motion without friction ([168 / 16](#/week/6/run/20261023T032449_leg_eval_seed1_press_ft1_force_6600_simmoving),
  [155 / 26](#/week/6/run/20261023T031801_leg_eval_seed1_press_ft1_force_6998_simmoving)) and with μ 0.4 and friction
  ([143 / 16](#/week/6/run/20261023T032531_leg_eval_seed1_press_ft1_force_6600_simmoving_mu04_fric1),
  [101 / 33](#/week/6/run/20261023T031844_leg_eval_seed1_press_ft1_force_6998_simmoving_mu04_fric1)).

- **Every fall is on a plate facing down,** the foot pushing up. The peak force there rose from round 4's 17 N (median)
  to 67 N by 6998, and the falls began at 6400. This is round 5's tipping (F-182) again, now at 16 and 8 N too. Plates
  facing down are 1.3% of the training table and a quarter of the frozen set, and training rewards tracking up to 35 N
  in every direction.
- **On the other three kinds nothing falls,** but there are fewer presses than round 4's final in every condition
  without friction (122-165 of 216 against 174-191), and no more with it (123-142 against 130-148). The training-time
  force error stayed near 5 N, and episodes were shorter (670-737 steps against round 4's ~785).
- **Not shipped.**

### 2026-10-23 · The fine-tune with the upward force capped: no falls, no better than round 4; exported for the dog

Two fine-tunes from round 4's `model_5999.pt` trained in parallel, 600 iterations each, on the first's recipe and starts:
- **[ft2a](#/week/6/run/20261023T033438_train_seed3_press_ft2a_force_s3):** no leg friction;
- **[ft2b](#/week/6/run/20261023T033538_train_seed3_press_ft2b_force_s3):** friction 0-1 N·m, half the first's range.

Both run with `--press_up_cap_n 16` (new: `leg_env.Go2D1LegEnvCfg.press_up_cap_n`, training only). It caps the commanded
force's upward component at 16 N: on a plate facing down, the levels are scaled down to it. 16 N keeps the frozen set's
8 and 16 N upward presses in range, and leaves its 28 N ones as the refusal test F-182 says they are.

    logs/unifp_train/press/train_one.sh force <round 4 model_5999.pt> press_ft2a_force_s3 600 \
      --spawn_states demos/combiner_key/assets/dog_stand_20261023/ft_starts.json --spawn_states_fraction 0.85 \
      --press_up_cap_n 16 --press_hold_correction --press_release_from_press_point --press_plate_face_only
    # ft2b: the same and --leg_friction_range 0 1

Presses of 288 / fell, against round 4's 196 / 0, 197 / 0, 139 / 0 and 165 / 2:

| checkpoint | classic stand | the dog's rest | the dog's rest, friction 1 N·m | sink in motion, friction 1 N·m |
| --- | --- | --- | --- | --- |
| ft2a 6200 | [176 / 1](#/week/6/run/20261023T043143_leg_eval_seed1_press_ft2a_force_6200_real) | [212 / 0](#/week/6/run/20261023T043224_leg_eval_seed1_press_ft2a_force_6200_dogrest) | [146 / 0](#/week/6/run/20261023T043305_leg_eval_seed1_press_ft2a_force_6200_dogrest_fric1) | [127 / 1](#/week/6/run/20261023T043347_leg_eval_seed1_press_ft2a_force_6200_simmoving_fric1) |
| ft2a 6400 | [159 / 0](#/week/6/run/20261023T042858_leg_eval_seed1_press_ft2a_force_6400_real) | [171 / 0](#/week/6/run/20261023T042939_leg_eval_seed1_press_ft2a_force_6400_dogrest) | [118 / 0](#/week/6/run/20261023T043020_leg_eval_seed1_press_ft2a_force_6400_dogrest_fric1) | [109 / 0](#/week/6/run/20261023T043102_leg_eval_seed1_press_ft2a_force_6400_simmoving_fric1) |
| ft2a 6598 | [185 / 0](#/week/6/run/20261023T042611_leg_eval_seed1_press_ft2a_force_6598_real) | [203 / 0](#/week/6/run/20261023T042653_leg_eval_seed1_press_ft2a_force_6598_dogrest) | [148 / 0](#/week/6/run/20261023T042734_leg_eval_seed1_press_ft2a_force_6598_dogrest_fric1) | [148 / 0](#/week/6/run/20261023T042816_leg_eval_seed1_press_ft2a_force_6598_simmoving_fric1) |
| ft2b 6200 | [175 / 1](#/week/6/run/20261023T043958_leg_eval_seed1_press_ft2b_force_6200_real) | [203 / 0](#/week/6/run/20261023T044039_leg_eval_seed1_press_ft2b_force_6200_dogrest) | [160 / 0](#/week/6/run/20261023T044120_leg_eval_seed1_press_ft2b_force_6200_dogrest_fric1) | [160 / 0](#/week/6/run/20261023T044202_leg_eval_seed1_press_ft2b_force_6200_simmoving_fric1) |
| ft2b 6400 | [180 / 0](#/week/6/run/20261023T043714_leg_eval_seed1_press_ft2b_force_6400_real) | [191 / 1](#/week/6/run/20261023T043755_leg_eval_seed1_press_ft2b_force_6400_dogrest) | [154 / 1](#/week/6/run/20261023T043836_leg_eval_seed1_press_ft2b_force_6400_dogrest_fric1) | [161 / 2](#/week/6/run/20261023T043917_leg_eval_seed1_press_ft2b_force_6400_simmoving_fric1) |
| ft2b 6598 | [152 / 1](#/week/6/run/20261023T043428_leg_eval_seed1_press_ft2b_force_6598_real) | [180 / 2](#/week/6/run/20261023T043510_leg_eval_seed1_press_ft2b_force_6598_dogrest) | [144 / 1](#/week/6/run/20261023T043552_leg_eval_seed1_press_ft2b_force_6598_dogrest_fric1) | [151 / 1](#/week/6/run/20261023T043633_leg_eval_seed1_press_ft2b_force_6598_simmoving_fric1) |

The rest of the battery for the two closest to round 4. Round 4, then ft2a 6598, then ft2b 6400:

| | round 4's final | ft2a 6598 | ft2b 6400 |
| --- | --- | --- | --- |
| settled stance | 207 / 0 | [211 / 2](#/week/6/run/20261023T044513_leg_eval_seed1_press_ft2a_force_6598_real_settled) | [206 / 0](#/week/6/run/20261023T044828_leg_eval_seed1_press_ft2b_force_6400_real_settled) |
| sink in motion | 203 / 0 | [194 / 0](#/week/6/run/20261023T044554_leg_eval_seed1_press_ft2a_force_6598_simmoving) | [198 / 1](#/week/6/run/20261023T044909_leg_eval_seed1_press_ft2b_force_6400_simmoving) |
| slide set, of 96 | 56 / 1 | [53 / 0](#/week/6/run/20261023T044635_leg_eval_seed1_press_ft2a_force_6598_slide) | [52 / 0](#/week/6/run/20261023T044951_leg_eval_seed1_press_ft2b_force_6400_slide) |
| key zero-shot, of 108 | 107 | [88](#/week/6/run/20261023T044715_leg_eval_seed1_press_ft2a_force_6598_key_dog33) | [93](#/week/6/run/20261023T045030_leg_eval_seed1_press_ft2b_force_6400_key_dog33) |

- **The cap stopped most of the tipping.** ft2a's final fell only in two of the settled stance's 28 N upward presses,
  the refusal test above its cap. ft2b still tipped on upward presses at 16 and 28 N (peaks of 95-115 N) in three of
  the four conditions. Friction in training made it worse there.
- **No fine-tune presses better than round 4 where the dog presses.** On walls and plates facing up (the dog's
  `palm_facing` and `palm_up`), ft2a's final and round 4 score about the same:

  | | classic stand | the dog's rest | dog's rest, friction 1 N·m | sink in motion, friction 1 N·m |
  | --- | --- | --- | --- | --- |
  | round 4's final | 117 | 129 | 80 | 93 (2 fell, on walls) |
  | ft2a's final | 115 | 126 | 79 | 87 (none fell) |

  Out of 144. At 8 N, the level nearest the dog's 5 N, they are even without friction (59 / 58, 63 / 64 of 72) and
  round 4 is ahead with it (43 / 32, 48 / 34).
- **The high contacts.** Past 60 N, ft2a has 12 against round 4's 3-5. Its extras are all on plates facing down. The six
  on plates facing up are the same six plates for both policies, with the same 122 N p95.
- **The overall gains are on the capped upward presses,** which the cap made trackable at 8 and 16 N (20-28 of 72
  against round 4's 6-22). The dog's press layer refuses upward presses.

**The first step from the dog's rest, open loop.** Each policy's largest joint target off the measured joints, from an
empty history with zero commands, as `press_live._policy` starts. Round 4's numbers reproduce F-191's:

| start | round 4's final | ft1 6998 | ft2a 6200 | ft2a 6598 | ft2b 6200 | ft2b 6400 |
| --- | --- | --- | --- | --- | --- | --- |
| trained settled stance | 0.25 rad | 0.41 | 0.28 | 0.32 | 0.27 | 0.26 |
| the dog's kp-25 rest | 0.51 | 0.63 | 0.52 | 0.51 | 0.51 | 0.50 |
| the dog's rest, levelled | 0.45 | 0.61 | 0.48 | 0.48 | 0.48 | 0.47 |
| the simulated kp-80 hold | 0.46 | 0.55 | 0.48 | 0.50 | 0.50 | 0.42 |
| the simulated kp-25 rest | 0.80 | 0.79 | 0.72 | 0.78 | 0.70 | 0.65 |

Training from these starts doesn't change how hard the policy corrects them: the 0.5 rad is the stance correction
itself. In simulation, round 4's final makes the same 0.5 rad step from the same rest and stands (F-192). The step's
size alone is not what F-188 went wrong on.

**Exported for the dog: ft2a's final** (`checkpoints/leg_press_force_ft2a_model_6598.npz`, sha256 44bcea01...;
numpy to 7e-6 of the checkpoint):
- It is the fine-tune Lukas asked for, trained from the dog's handover starts, and falls on none of the dog's plate
  kinds in any condition.
- It is not better than round 4's final in simulation, and is worse on the key (88 of 108).
- Its meta carries a `start_gate` (new: `export_policy.start_gate`, read by `press_live._stand_up`): the stances its
  training started from, 0.402 rad (p95 of each stance's largest joint offset from the settled stance) and pitch -15.9
  to +1.0°.
- That gate admits the dog's kp-25 rest (0.20 rad, 8.1° nose up), which the default gate (0.20 rad, 5°) refused in
  the second live run. Round 4's export has no gate and keeps the default.
- Checked: the program's fake-clock tests run on this export (10 of 10) as on round 4's. The kinematic stand-in
  under twice the load stops on the RR calf's limit within 10 policy ticks with either policy: the stand-in's sag
  model, not the policy.

On the dog: `demos/combiner_key/robot/run_press_on_dog.sh --policy checkpoints/leg_press_force_ft2a_model_6598.npz
--live --presses 0` first, then `--live` with a hand. What the first run will say:
- If the dog stands from its rest under this policy, the start was the gap.
- If it sits back as in F-188, the gap is the legs' dynamics, which no start-stance fine-tune here addresses. The joint
  friction is the candidate to measure first.

### 2026-10-23 · Why the dog sat back: the policy's own crouch, in simulation too; the fine-tune turned to a level base

Lukas: "so no improvement has been made by this fine tuning? why did the dog then fall over?" Looking again at the
evaluations' per-step traces, not their fall counts, shows the answer (F-194).

Simulation sits back exactly as the dog did. Within 0.14-0.30 s of every start, still waiting on the foot-down command
(the frozen evaluation's approach starts at 0.48 s), round 4's final drops ~12 cm to 0.22 m and pitches 17-25° nose up,
with the support knees at ~21 N·m:
- from the classic stand: 0.338 -> 0.219 m, 22°;
- from the dog's rest: 0.271 -> 0.215 m, 25°;
- from the simulated hold: 0.313 -> 0.218 m, 23°.

The dog in F-188: 0.273 -> 0.212 m, 20° when the stop fired at 0.24 s, 20-27 N·m. In simulation the robot then presses
from that posture: median pitch -26 to -30° while pressing, each episode's largest tilt 30-32° (median), at most 42°.

So the live program's 15° tilt stop fired on the policy's trained posture, not on a fall. Every run of round 4's final or
ft2a's would stop the same way (ft2a crouches less, 15.6°, but still past 15° in 73% from the dog's rest).

- **The earlier entries were wrong here.** They judged the starts by falls; "nothing modelled here reproduces it"
  (F-192) and the stand-only test proposed with ft2a (F-193) are superseded, marked so in the findings.
- **The posture is what the training rewarded.** UniFP's orientation term at -4 costs a 25° crouch ~0.7 a second, against
  ~10 a second of positive reward. The base height term (-2) costs the 12 cm drop 0.01 a second. Training waited only
  0.3-1 s before each approach.

Lukas chose a fine-tune for a level base over a wider tilt stop. Its pieces, new today:
- `leg_rewards.tilt_excess`, the tilt from upright past `--tilt_excess_deg` in radians, weighted by
  `--tilt_excess_weight`;
- `--press_orientation_weight`, which overrides foot_press_cfg's -4;
- `--start_delay_steps LO HI`, the waiting drawn per episode in training;
- `--eval_start_delay_steps`: 500 is a stand-only evaluation, 10 s on the foot-down command like `--presses 0`;
- in the press evaluation, `max_tilt_waiting_deg` per episode, and in its summary `tilt_over_15` and
  `tilt_waiting_over_15`, the episodes the live program's stop would end.

### 2026-10-23 · The fine-tune for a level base: the crouch gone, the presses down by a fifth

Two fine-tunes from round 4's `model_5999`, 1000 iterations in parallel, on ft2a's recipe (the dog's handover starts,
the upward force capped at 16 N) plus two changes:
- **Long waits:** each plate episode waits 15-250 policy steps (0.3-5 s) on the foot-down command before its approach.
- **A harsher tilt penalty:**
  - [ft3a](#/week/6/run/20261023T061625_train_seed3_press_ft3a_force_s3): orientation -20 and `tilt_excess` -20 per rad past 10°, about 9 a second at
    25°;
  - [ft3b](#/week/6/run/20261023T061725_train_seed3_press_ft3b_force_s3): -40 and -60 past 8°, about 25 a second at 25°.

    logs/unifp_train/press/train_one.sh force <round 4 model_5999.pt> press_ft3a_force_s3 1000 \
      --spawn_states demos/combiner_key/assets/dog_stand_20261023/ft_starts.json --spawn_states_fraction 0.85 \
      --press_up_cap_n 16 --press_hold_correction --press_release_from_press_point --press_plate_face_only \
      --start_delay_steps 15 250 --press_orientation_weight -20 --tilt_excess_weight -20 --tilt_excess_deg 10
    # ft3b: --press_orientation_weight -40 --tilt_excess_weight -60 --tilt_excess_deg 8

In training the tilt terms fell to a mean of ~6° (ft3a) and ~5° (ft3b) within ~150 iterations. The press force error
rose to 7.9 and 9.1 N and was still coming down at the end (6.1 and 6.8; ft2a 5.3).

Presses of 288 / fell / episodes past the live program's 15° stop (for evaluations made before the metric, from their
traces):

| policy | classic stand | the dog's rest | 10 s standing from the dog's rest | the dog's rest, friction 1 N·m | the sink in motion |
| --- | --- | --- | --- | --- | --- |
| round 4's final | [196 / 0 / 288](#/week/6/run/20261023T032947_leg_eval_seed1_press_r4_force_5999_real_control) | [197 / 0 / 284](#/week/6/run/20261023T023140_leg_eval_seed1_press_r4_force_5999_dogrest) | [0 / 0 / 284](#/week/6/run/20261023T061421_leg_eval_seed1_press_r4_force_5999_dogrest_stand10) | [139 / 0 / 283](#/week/6/run/20261023T023342_leg_eval_seed1_press_r4_force_5999_dogrest_fric1) | [203 / 0 / 287](#/week/6/run/20261023T023608_leg_eval_seed1_press_r4_force_5999_simmoving) |
| ft2a's final | [185 / 0 / 287](#/week/6/run/20261023T042611_leg_eval_seed1_press_ft2a_force_6598_real) | [203 / 0 / 284](#/week/6/run/20261023T042653_leg_eval_seed1_press_ft2a_force_6598_dogrest) | [0 / 0 / 283](#/week/6/run/20261023T061541_leg_eval_seed1_press_ft2a_force_6598_dogrest_stand10) | [148 / 0 / 283](#/week/6/run/20261023T042734_leg_eval_seed1_press_ft2a_force_6598_dogrest_fric1) | [194 / 0 / 287](#/week/6/run/20261023T044554_leg_eval_seed1_press_ft2a_force_6598_simmoving) |
| ft3a 6600 | [157 / 2 / 10](#/week/6/run/20261023T074539_leg_eval_seed1_press_ft3a_force_6600_real) | [157 / 0 / 9](#/week/6/run/20261023T074621_leg_eval_seed1_press_ft3a_force_6600_dogrest) | [0 / 0 / 11](#/week/6/run/20261023T074703_leg_eval_seed1_press_ft3a_force_6600_dogrest_stand10) | [131 / 1 / 3](#/week/6/run/20261023T074744_leg_eval_seed1_press_ft3a_force_6600_dogrest_fric1) | [154 / 5 / 50](#/week/6/run/20261023T074825_leg_eval_seed1_press_ft3a_force_6600_simmoving) |
| ft3a 6998 | [154 / 8 / 4](#/week/6/run/20261023T074208_leg_eval_seed1_press_ft3a_force_6998_real) | [174 / 5 / 7](#/week/6/run/20261023T074251_leg_eval_seed1_press_ft3a_force_6998_dogrest) | [0 / 3 / 3](#/week/6/run/20261023T074333_leg_eval_seed1_press_ft3a_force_6998_dogrest_stand10) | [129 / 7 / 6](#/week/6/run/20261023T074416_leg_eval_seed1_press_ft3a_force_6998_dogrest_fric1) | [156 / 4 / 33](#/week/6/run/20261023T074457_leg_eval_seed1_press_ft3a_force_6998_simmoving) |
| ft3b 6600 | [118 / 12 / 0](#/week/6/run/20261023T075237_leg_eval_seed1_press_ft3b_force_6600_real) | [119 / 15 / 0](#/week/6/run/20261023T075319_leg_eval_seed1_press_ft3b_force_6600_dogrest) | [0 / 11 / 0](#/week/6/run/20261023T075401_leg_eval_seed1_press_ft3b_force_6600_dogrest_stand10) | [102 / 17 / 0](#/week/6/run/20261023T075442_leg_eval_seed1_press_ft3b_force_6600_dogrest_fric1) | [117 / 9 / 17](#/week/6/run/20261023T075524_leg_eval_seed1_press_ft3b_force_6600_simmoving) |
| ft3b 6998 | [97 / 14 / 15](#/week/6/run/20261023T074907_leg_eval_seed1_press_ft3b_force_6998_real) | [138 / 14 / 14](#/week/6/run/20261023T074949_leg_eval_seed1_press_ft3b_force_6998_dogrest) | [0 / 4 / 4](#/week/6/run/20261023T075032_leg_eval_seed1_press_ft3b_force_6998_dogrest_stand10) | [109 / 4 / 7](#/week/6/run/20261023T075113_leg_eval_seed1_press_ft3b_force_6998_dogrest_fric1) | [122 / 13 / 47](#/week/6/run/20261023T075155_leg_eval_seed1_press_ft3b_force_6998_simmoving) |

The 10 s stand scores no presses by construction: its episodes end before a press can finish. It is there for the
waiting.

Largest tilt while waiting, median over episodes, from the dog's rest: round 4's final 26.4° (284 of 288 past 15° in
the 10 s stand), ft2a's 15.8° (238), ft3a's 6.7° (0), ft3b's 6.7-6.8° (0).

- **The crouch is gone.** Neither ft3 checkpoint passes 15° while waiting from a start at rest. Over whole episodes
  ft3a passes it in 3-11 of 288; its episodes' largest tilt is 8-9° (median), against round 4's 30-32°.
  - From the moving sink, 29-50 episodes pass 15° while waiting, at the start: those stances are already 5-15° nose
    up and moving. The dog's handover waits until it is still.
- **Presses drop by a fifth.** On the classic stand and from the dog's rest ft3a scores 154-174, against round 4's
  196-197. From the dog's rest at 6600 it loses 60
  presses round 4 makes:
  - mostly over-pressed (force bias +4.0 N on them), on low, close plates (contact 0.17 m up and 0.32 m ahead,
    medians, against 0.19 and 0.38 for all);
  - with more slips off the plate (45-49 against round 4's 0-4);
  - plates facing up, the dog's `palm_up` kind, lose the most: 39-46 of 72 against 56-61.
- **The falls are all 28 N upward presses**, the refusal test above the training cap (2-8 for ft3a, 12-15 for ft3b);
  nothing falls on the other three kinds. The dog refuses upward presses.
- **ft3b is too harsh.** It presses 97-138 and falls more.
- **Next:** ft3a's force error was still falling, so ft3a's final trained on alone for 1000 more iterations, the same
  recipe ("ft3c").

### 2026-10-23 · ft3a trained on (ft3c), and the dog's own two hands as a plate set: the level policies are not safe on them

[ft3c](#/week/6/run/20261023T075739_train_seed3_press_ft3c_force_s3) is ft3a's final trained on alone for 1000 more iterations on the same recipe.
Presses / fell / past 15°:

| checkpoint | classic stand | the dog's rest | 10 s standing from it | the dog's rest, friction 1 N·m | the sink in motion |
| --- | --- | --- | --- | --- | --- |
| 7600 | [145 / 1 / 1](#/week/6/run/20261023T083610_leg_eval_seed1_press_ft3c_force_7600_real) | [164 / 4 / 3](#/week/6/run/20261023T083651_leg_eval_seed1_press_ft3c_force_7600_dogrest) | [0 / 0 / 2](#/week/6/run/20261023T083732_leg_eval_seed1_press_ft3c_force_7600_dogrest_stand10) | [46 / 145 / 8](#/week/6/run/20261023T083813_leg_eval_seed1_press_ft3c_force_7600_dogrest_fric1) | [157 / 3 / 26](#/week/6/run/20261023T083856_leg_eval_seed1_press_ft3c_force_7600_simmoving) |
| 7997 | [147 / 0 / 0](#/week/6/run/20261023T083241_leg_eval_seed1_press_ft3c_force_7997_real) | [216 / 3 / 5](#/week/6/run/20261023T083323_leg_eval_seed1_press_ft3c_force_7997_dogrest) | [0 / 0 / 0](#/week/6/run/20261023T083405_leg_eval_seed1_press_ft3c_force_7997_dogrest_stand10) | [145 / 5 / 10](#/week/6/run/20261023T083446_leg_eval_seed1_press_ft3c_force_7997_dogrest_fric1) | [209 / 3 / 16](#/week/6/run/20261023T083528_leg_eval_seed1_press_ft3c_force_7997_simmoving) |

- **The final looks like a recovery.** It is level (largest tilt 6-9° median, none past 15° while waiting) and presses
  216 from the dog's rest (189 on the three kinds the dog uses, round 4 191).
- **But it is fragile.**
  - At 7600 with friction it sank in 145 episodes once its foot was back down: the base level, 0.29 -> 0.14 m.
  - The final's falls are tip-overs toward the front right (roll +45°) while lifting, 1.7-3 s in. They are toward low,
    upward-facing plates at (0.17, -0.24, 0.17) and (0.30, -0.22, 0.17) m, near the dog's `palm_up` hand at
    (0.33, -0.20, 0.11).

**The dog's hands as a plate set** (`--plate_set dog`, new: `foot_press_cfg.EVAL_DOG_*`):
- the live program's two plates (`press_layer.PLATES`, rows 14027 and 10851 of the training table), rigid and sprung,
  at its 5 N cap and 8 N;
- 36 times each with their own estimate errors: 288 presses;
- v2's rows are unchanged (0 of 288 differ from yesterday's evaluation).

From the dog's rest:

| policy | presses | palm_up | palm_facing | fell | past 60 N | past 15° |
| --- | --- | --- | --- | --- | --- | --- |
| round 4's final | [280](#/week/6/run/20261023T084229_leg_eval_seed1_press_r4_force_5999_hands) | 144 / 144 | 136 / 144 | 0 | 5 | 288 |
| ft2a 6598 | [252](#/week/6/run/20261023T084311_leg_eval_seed1_press_ft2a_force_6598_hands) | 108 | 144 | 0 | 34 | 288 |
| ft3a 6998 | [75](#/week/6/run/20261023T084352_leg_eval_seed1_press_ft3a_force_6998_hands) | 67 | 8 | 0 | 87 | 0 |
| ft3c 7997 | [174](#/week/6/run/20261023T084438_leg_eval_seed1_press_ft3c_force_7997_hands) | 48 | 126 | 30, all on palm_up | 87 | 35 |
| round 4's final, friction 1 N·m | [84](#/week/6/run/20261023T084523_leg_eval_seed1_press_r4_force_5999_hands_fric1) | 51 | 33 | 0 (59 slipped) | 18 | 288 |
| ft3c 7997, friction 1 N·m | [59](#/week/6/run/20261023T084606_leg_eval_seed1_press_ft3c_force_7997_hands_fric1) | 54 | 5 | 7 | 51 | 13 |

- **Round 4's final presses the dog's hands almost perfectly** in simulation: 141 of 144 at 5 N, median error 1.2 N.
  Its only fault for the dog is the crouch.
- **The level policies are not safe there.** They strike the hand past 60 N in 87 of 288, and ft3c tips over in 30 of
  its 144 palm_up presses.
- **Leg friction costs all of them most of the hands** (84 and 59): round 4 by slipping off, ft3c by over-pressing.
- **Not shipped.**
- **What I take from it.** Penalising the tilt in every stage made the policy press a new way, and 2000 iterations did
  not relearn it. F-188's sit-back came while the dog stood. Next ("ft4"): the tilt penalised hard only while a plate
  episode stands (waiting, or the foot back down), round 4's -4 elsewhere (`--tilt_stages standing`, new).

### 2026-10-23 · Level only while standing (ft4): it stands level, and over-presses the palm under the foot

[ft4](#/week/6/run/20261023T084844_train_seed3_press_ft4_force_s3): from round 4's `model_5999`, 1000 iterations alone, on ft2a's recipe with waits of
0.3-5 s, and the tilt penalised hard (orientation -40, `tilt_excess` -60 per rad past 8°) only while a plate episode
stands (`--tilt_stages standing`). Round 4's -4 is kept everywhere.

    logs/unifp_train/press/train_one.sh force <round 4 model_5999.pt> press_ft4_force_s3 1000 \
      --spawn_states demos/combiner_key/assets/dog_stand_20261023/ft_starts.json --spawn_states_fraction 0.85 \
      --press_up_cap_n 16 --press_hold_correction --press_release_from_press_point --press_plate_face_only \
      --start_delay_steps 15 250 --tilt_stages standing --press_orientation_weight -40 --tilt_excess_weight -60 \
      --tilt_excess_deg 8

Presses / fell; then the median largest tilt while waiting and with the foot back down, and the episodes past 15° in
each:

| | round 4's final | ft4 6600 | ft4 6998 |
| --- | --- | --- | --- |
| the dog's hands | 280 / 0; 25.0°, 288; 12.3°, 79 | [180 / 0](#/week/6/run/20261023T092908_leg_eval_seed1_press_ft4_force_6600_hands); 6.8°, 0; 2.5°, 0 | [178 / 0](#/week/6/run/20261023T092414_leg_eval_seed1_press_ft4_force_6998_hands); 6.8°, 0; 1.7°, 0 |
| 10 s standing from the dog's rest | 0 fell; 26.4°, 284 | [0 fell](#/week/6/run/20261023T092951_leg_eval_seed1_press_ft4_force_6600_dogrest_stand10); 6.8°, 0 | [1 fell](#/week/6/run/20261023T092458_leg_eval_seed1_press_ft4_force_6998_dogrest_stand10); 6.8°, 0 |
| the dog's rest | 197 / 0 | [177 / 0](#/week/6/run/20261023T093032_leg_eval_seed1_press_ft4_force_6600_dogrest) | [178 / 2](#/week/6/run/20261023T092539_leg_eval_seed1_press_ft4_force_6998_dogrest) |
| classic stand | 196 / 0 | [147 / 0](#/week/6/run/20261023T093112_leg_eval_seed1_press_ft4_force_6600_real) | [146 / 5](#/week/6/run/20261023T092620_leg_eval_seed1_press_ft4_force_6998_real) |
| the dog's hands, friction 1 N·m | 84 / 0 | [70 / 0](#/week/6/run/20261023T093154_leg_eval_seed1_press_ft4_force_6600_hands_fric1) | [163 / 1](#/week/6/run/20261023T092702_leg_eval_seed1_press_ft4_force_6998_hands_fric1) |
| the dog's rest, friction 1 N·m | 139 / 0 | [139 / 0](#/week/6/run/20261023T093236_leg_eval_seed1_press_ft4_force_6600_dogrest_fric1) | [165 / 0](#/week/6/run/20261023T092745_leg_eval_seed1_press_ft4_force_6998_dogrest_fric1) |
| the sink in motion | 203 / 0 | [175 / 0](#/week/6/run/20261023T093318_leg_eval_seed1_press_ft4_force_6600_simmoving) | [171 / 7](#/week/6/run/20261023T092826_leg_eval_seed1_press_ft4_force_6998_simmoving) |

- **It stands level.** None of the 10 s stands from the dog's rest passes 15° while waiting (round 4: 284 of 288), and
  the foot back down sits at 1.7-2.7°. While pressing it still crouches (largest tilt 27° median).
- **It does not press the palm under the foot as round 4 does:** 42-64 of 144 against 144. It over-presses (bias
  +3.6 to +4.4 N on 5 and 8 N commands; press peak median 12 N), and 15-29 presses strike past 120 N, the damage limit.
  On the palm in front it scores 114-138 against 136.
- **Every fine-tune on ft2a's recipe lost that press:** ft2a 108, ft3a 67, ft3c 48, ft4 42-64. Round 4's final is
  the only policy here that presses both hands cleanly; the recipe's common changes (the dog's starts, the cap, the
  long waits) are not separated.
- **Not shipped.** Stopped here to ask Lukas how to go on.

### 2026-10-23 · Back to round 4's final

Lukas: "Ok let's go back to doing the policy that worked. Can you commit and push?"

Round 4's final (`checkpoints/leg_press_force_r4_model_5999.npz`, F-180) stays the foot-press policy. It presses the
dog's two hands 280 of 288 in simulation from the dog's rest. The fine-tunes of today are recorded and not used.

For the dog the live program has two things in its way (`demos/combiner_key/robot/README.md` says so); neither is
changed here:
- **the tilt stop:** `--tilt_limit_deg 15` ends every run on the policy's own crouch, which reaches 36° (p95) in
  simulation;
- **the settle gate:** the default 0.20 rad / 5° refuses the dog's kp-25 rest (8.1° nose up), from which round 4 presses
  as from its trained stands in simulation.
### 2026-10-23 · The holster draw's arm motion on the real dog: ready, dry on the dog, not yet run live

Lukas: "can we test this on the real dog while it is sitting? no holster is there yet but I just want to see it go
through the motions in real life". Built `demos/combiner_key/robot/holster_dog.py` and `run_holster_on_dog.sh`
(`d1dogholster` on the PC; dry unless `--live`, `--step` for an Enter before every move). It is the simulation's
default draw (F-189, F-190) as servo targets through the door program's arm link, with no perception and no legs: sit
the dog with the remote first. The moves: `turn` (Joint1 at the fold), `reach` (joint space, 40°/s), `on`, `close`,
`draw`, `hold`, `back`, `open`, `off` (streamed: the joint path planned on the CPU model, resampled every 100 ms and
paced at 0.6 rad/s on the fastest joint), then `unreach` and `fold`. Before moving it refuses unless the link answers,
the report is fresh and the arm is folded. Each joint-space move ends within 2° or holds the arm after 20 s. `a` + Enter
holds the arm where it is.

Checks on the CPU arm model, the motion as the arm link would follow it (its profile and servo model, `arm_link.SimulatedArm`):
- The link trails the streamed path by up to 10.7° (its profile plans to stop on each target: v²/2a at 34°/s and
  60°/s²). The jaw centre stays within 2.2 mm of the planned line.
- The moving links keep ≥ 37 mm from the dock, its rails, the MID-360 and its mount through the whole sequence.
- The hand comes to **9.7-11.8 mm of the D1's own base** while on, at, and off the bar's place. That margin is the
  model's; the real arm's joint zeros are not validated (F-023).
- `test_holster.DogProgram`: 4 tests (the moves start and end folded and undo each other, the streamed paths are under
  the link's 45°/s and the joints' limits, the jaws, a dry run); 13 in `test_holster.py`, OK.

On the dog (`d1update`, then `d1dogholster`, dry):
- [First dry run](#/week/6/run/20261023T031352_dry): the plan and the simulated motion went through; the real arm's
  report, through the link, read [0.5, -89.8, 88.2, -6.1, 17.2, 4.9]°, and the start check said a live run would
  refuse: J5 was 13° from the rest pose's 4.1°. The wrist rests where it was left, and `turn` sends it to the fold's
  anyway. Now 10° is allowed on Joint1-3 and 25° on the wrist.
- [Second dry run](#/week/6/run/20261023T031423_dry): the start check passed on the real arm; all 11 moves went through on
  the simulated arm.

Not run live yet: that is Lukas's, beside the dog. `d1dogholster --live --step` first.
Before the live run, [the simulated draw in the viewer again](#/week/6/run/20261023T034609_seed1_viewer_paced) (3/3 drawn, default timing).

### 2026-10-23 · The wrist through the shoulder: the grip 1 cm higher, the unfold through a point above; dry on the dog

Lukas, from the viewer (a screenshot, not kept in `screenshots/`): "the wrist motor travels through the shoulder motor.
can we assume the lever is 1cm longer so it grips it higher." The screenshot shows the forearm's wrist-pitch housing
inside the shoulder motor's at the grip. The simulation has the arm's self-collision off, and no check here compared
link against link: the arm-to-holster, key-to-base and trunk-proxy numbers of F-189 say nothing about it. Lukas judged
the result by eye ("it didnt collide on that run") and will lengthen the real lever by 1 cm.

Changes (`holster.py`): `LEVER_EXTRA_M` 0.010, the lever 10 mm longer between the shaft and the platform, its far end
still in the cup. The holster stays at the export's place (`holster_in_d1_base`); the key's frame, the grip and the
hand are 10 mm further out along the holster's axis (`key_in_d1_base`, `key_boxes`). The simulation draws the key
stretched to match. At the new grip, `SLIDE_M` 0.10 -> 0.09: it solves to 90 mm, against the 75 needed. `PRE_OUT_M`
0.045 -> 0.065: the higher root put a finger 2.7 mm from it in the unfold.
- [Viewer, 3 attempts](#/week/6/run/20261023T040014_seed1_viewer_lever10mm): 3/3 drawn 89.6 mm, grip error ≤ 1.4 mm.
  The hand was 40 mm off its point at the start of `pre`: the longer unfold had not arrived.
- [8, holster up to 3 mm off](#/week/6/run/20261023T040312_seed1_lever10mm_err3mm): 8/8. In one attempt a finger
  pressed the still-held key at up to 14.7 N during the unfold. The CPU check's unfold had been modelled with each
  joint at its own speed (`d1_ik.traversal_configs`), and it said 11.9 mm clear. But the arm link's profile and the
  simulation's joint moves go along a straight line in joint space, and on that line the unfold puts a finger 3.3 mm
  into the key. The check now models the straight line (`joint_line`). The unfold goes to the hand 80 mm above `pre`
  (`ABOVE_PRE_M`, `above_pre`), then down to `pre`: 14.0 and 16.7 mm clear, the trunk proxy ≥ 36 mm.
- [8 again, with the waypoint](#/week/6/run/20261023T040803_seed1_lever10mm_above_err3mm): 8/8 drawn 89.4-89.6 mm; no
  finger contact before the jaws close (4.3 N at most in the draw); grip error ≤ 3.0 mm; the hand ≤ 5.7 mm off its
  line in the draw. The sequence takes 14.5 s. The arm still settles into `pre` up to 26 mm off its point (free space,
  65 mm from the key). The dog program waits for each joint move to arrive instead.
- `holster_dog.py`: the moves are now `turn`, `reach` (to the point above), `lower`, `on` ... `off`, `raise`,
  `unreach`, `fold`. [Dry on the dog](#/week/6/run/20261023T040911_dry): the start check passed on the real arm; on the
  arm link's simulated trajectory the jaw centre keeps within 2.1 mm of its line, and the hand ≥ 17.9 mm from the D1's
  base (9.7 mm before). 14 tests in `test_holster.py` pass.

Still not checked: link against link. The wrist-shoulder clearance at the new grip is Lukas's look in the viewer; the
unfold through the point above has not been looked at for it. Not run live yet.

### 2026-10-23 · The holster motion live on the dog: microsteps and pauses; every move now a timed path for the arm link

Lukas ran `d1dogholster --live`: four runs at the first grip, before the wrist-shoulder problem was found
([1](#/week/6/run/20261023T034223_live), [2](#/week/6/run/20261023T034310_live), [3](#/week/6/run/20261023T034355_live),
[4](#/week/6/run/20261023T034454_live)), and [one with the grip 1 cm higher](#/week/6/run/20261023T041550_live). All went
through every move to the fold. Whether the first grip's wrist touched the shoulder on the real arm is not in the
record. Lukas: "currently it moves in microsteps, with lots of mini pauses of varying length, some up to 0.3 seconds".

From the last run's arm-link log (405 commands):
- **The commands went out at a steady 10 Hz**: intervals 0.100-0.112 s.
- **But their steps were small.** Moving, the median step was 2.40° a command; 58% were under 2.5° and 19% under 1°.
  The firmware plans each setpoint from rest (F-045), so steps that small make it stop and start. The cause was the
  program's: it sent a target 100 ms ahead every 100 ms, and the link's `Profile` slows to stop on each target it is
  given.
- **The pauses are the arm's reports.** 119 of the 405 commands saw an unchanged report, in 54 streaks of up to 6 ticks
  (0.6 s); at the program, the reports were up to 0.59 s old (p90 0.27). The link keeps its command within 10° of the
  last report, so in a gap the command stops and the arm reaches it and waits. That happened 56 times, 13 of them with
  the command not moving at all. At 34°/s, 10° is about 0.3 s.
- The arm trailed the streamed path by up to 42-54° by the node's (stale) reports.

Changes:
- `arm_link.py`: a `path` message (`PathPlayer`). The link plays a timed joint path by its clock at 10 Hz, each
  command the path's own point. Its lead on the report may grow by the path's speed times the report's age, up to
  `PATH_LEAD_MAX_DEG` 15°; past that, the path's clock waits for the arm. A path must start within 10° of the arm and
  move no joint faster than 45°/s, or it is refused and the arm held. `target` messages, which the door, key and
  insertion programs use, are unchanged.
- `holster_dog.py`: every move is one path (`move_path`). A joint-space move is a straight joint line accelerating at
  60°/s² to 40°/s and braking; a streamed move is its paced path; a wait holds a point.

Checks:
- Dry, on the simulated arm: steps median 3.44°, 20% under 2.5° (mostly the joint moves' ramps), the arm at most 3.1°
  behind its path (10.7 before), the jaw centre within 0.4 mm of its line (2.2 before), the hand ≥ 16.9 mm off the D1's base.
- The real `Link` over its socket against a fake client whose reports come in bursts (15% of gaps 0.3-0.6 s): commands
  every 0.10 s, steps median 3.45°, 19% under 2.5°, the path's clock waiting in 12 ticks of ~16 s of motion.
- Tests: `test_robot.PathPlayerTests` (4: the points by the clock with no braking, a held-back arm makes it wait, the
  lead's growth with the report's age to its cap, the refusals); the robot suites' 125 tests pass.
- [Dry on the dog](#/week/6/run/20261023T043033_dry): the start check passed on the real arm.

[Run live with the paths](#/week/6/run/20261023T043122_live): all 13 moves. Lukas: "its better, but still stutters. I
did see about 30 degrees of smooth movement in the base swivel-only step, so I know its possible". Per joint, from the
link's log:
- `turn` and `fold` move Joint1 alone, 4.0° a command, and were smooth.
- In every other move the leading joint moved 3.4-4° a command, but on a straight joint line the others move in
  proportion: 0.3-2° a command, under the ~2.5° the firmware runs through (F-045). They start and stop every 100 ms.
- The path's clock still waited 9-12 times in `reach` and `draw` at the 15° cap on stale reports.

A faster command rate would let slow joints run continuously (on F-045's model a joint is continuous at 10 Hz only
above ~22°/s), but `d1_hardware` caps motion commands at 10 Hz on purpose: an arm went inert after over-commanding
(F-032, and the Caltech account in F-024). So, within 10 Hz:
- **Joint moves**: each joint on its own at the full 40°/s (4° a command) until it arrives, ramps of 300°/s²
  (`holster.joint_move`), not a straight joint line. `holster.check` models the unfold that way: ≥ 11.9 mm from the
  holster and key, the trunk proxy ≥ 35.7 mm. Planned freshly, the way back put the hand 0.2 mm off the D1's base on the
  CPU model (`unreach`), so `raise`, `unreach` and `fold` are now the way out played backwards: the same checked clearances.
- **Moves along a line** paced at 0.75 rad/s (43°/s) on the fastest joint, from 0.6: Joint1 and Joint4 step ~3°.
  The shoulder and elbow still step ~0.5° (1-2 mm at the hand); nothing within 10 Hz changes that.
- **Lead on stale reports**: a path may ask for up to 20° (`lead_max_deg`, the link's limit); the free moves do, and the
  draw and anything with the jaws shut keeps 15°.

Checks: dry, the hand ≥ 16.9 mm off the D1's base in every move (`unreach` 72.5 mm); the real `Link` against bursty fake
reports, the largest joint step a command median 4.03° (3.45 before), 9% under 2.5° (19%); motion 20 s (26).
Tests: the robot suites' 126 pass, one new (a joint move's every moving joint at the full pace).
[Dry on the dog](#/week/6/run/20261023T044116_dry): the start check passed.
[Live](#/week/6/run/20261023T044507_live), all 13 moves: Lukas, "this works much better. the stutters are almost gone".
The link's commands: every 0.100 s (max 0.119); the largest joint step a command, moving, median 4.02°, 10% under 2.5° (the
first live version: 2.40° and 58%). In `reach` and `unreach` every moving joint stepped 4.0°; along the line the
shoulder and elbow stepped 0.4-0.5°, as expected. Not measured: the arm's own motion between its 9 Hz reports.
Lukas: "is there any way to completely eliminate the stutters?" What is left is the moves along the line (`on`,
`draw`, `back`, `off`, ~9 s): the shoulder and elbow step 0.4-0.5° a command to keep the hand on it. Played as joint
moves instead (each joint at its own pace), on the CPU model `on` strays 19.0 mm off its line (16.7 across the jaws,
whose slack is ~10 mm a side), turns the hand 22.7° and brings an open finger to 2.4 mm of the key; `draw` strays
89.9 mm. So within 10 Hz and mode 0 they stay; past that are a faster command rate (F-032: no) and the arm's on-board
command execution time (reached over SSH in the Caltech report, F-024; unexplored here, untested, Lukas's call).

Lukas: "does this explain stuttering while only the base is swivelling?" No. In [the last run](#/week/6/run/20261023T044507_live)
the swivel's commands stepped Joint1 4.0° every 0.10 s throughout, but at 1.14 s into `turn` the arm's report froze at
46.3° for ~0.7 s. When it resumed, the arm was at 51.3° with the command at 65.2°: the arm had not been acting on the
commands either. `fold` the same at 1.71 s (37.6° frozen ~0.5 s, 5° moved of 20 commanded). Across all nine live runs
([also](#/week/6/run/20261023T044227_live) [these](#/week/6/run/20261023T044408_live)), 10-15% of the program's ticks
see a report over 0.25 s old, up to 0.82 s, and as often with the arm holding still (2.5-22%) as moving: a gap in the
arm's link to the dog, both ways, unrelated to the motion. F-020 measured a steady 111 ms (stdev 0.5 ms) over 120 s with
the arm idle; where the gaps come from on the dog (the arm's network shared with the Go2 and the MID-360, the Jetson's
load, the link's Python) is not yet known.

Step 1, read-only (Lukas: "yes, run step 1"): `demos/combiner_door/robot/arm_feedback_timing.py`, DDS readers only (no
writer on `rt/arm_Command`), each sample's arrival on the dog and the arm's own source timestamp. [60 s on the
dog](#/week/6/run/20261023T045523_idle_stack_on), the arm folded, nothing commanding it, P2Dingo's stack up (front camera,
RealSense, Livox, XM125, zenoh, the arm bridge): `current_servo_angle` 540 samples at 9.00 Hz, every interval 111 ms (max
114.8 ms), no gap over 0.25 s on either clock; the status stream 9.94 Hz, max 0.20 s; the arm's NIC took ~4,180 packets/s
(~44 Mbit/s, most likely the lidar) with no errors or drops; load average 1.5. So the arm, the network and the dog's load
do not make the gaps on their own: they come only while the arm link commands the arm (holding a point too). Which side
stops -- the arm (its own timestamps would show it) or the link's process and its DDS -- is the next measurement:
`run_holster_on_dog.sh --timing` now runs the capture beside a run, in the run's record (`feedback_timing/`; [checked
dry](#/week/6/run/20261023T045750_dry)). `d1fetch` brings `logs/arm_feedback_timing/` back too.

[Live with `--timing`](#/week/6/run/20261023T045925_live) (Lukas ran it; all 13 moves): over the capture's 38.8 s the
arm's `current_servo_angle` had **19 gaps over 0.25 s (8.1 s in all, the longest 0.71 s) on the arm's own source
timestamps** as on arrival; `arm_Feedback` funcode 1 had 4 (to 0.40 s); the status stream (funcode 3) none, a steady
10 Hz. No NIC errors or drops; load at most 2.05. The gaps fall in every move, about one every 2 s, as often while the
link streams the same pose (`close`, `hold`, `open`) as while it steps 4° a command, and the commands went out on time
through them. With nothing commanding it, the same capture had none in 60 s. So it is the arm: under a 10 Hz command
stream its firmware stops reporting angles, and (from the earlier runs' commands and reports) stops executing them,
for 0.3-0.7 s at a time, while its status loop runs on. Every command carries all seven servos, the gripper's too
(`d1_hardware.set_all_joint_angles`). Whether a lower command rate, or commands that leave the gripper out, stop the
pauses is untested: both need the arm commanded, Lukas's call.

Lukas: "you can set the tests up and you have permission to move the arm. the robot is sat stationary and nothing is
nearby it". `demos/combiner_door/robot/arm_command_rate_test.py` (`run_command_rate_test_on_dog.sh`): the arm held at its
fold, commanded its own pose in 20 s blocks through `d1_hardware.D1Client` (10 Hz at most), the report timing beside it;
it stops the stream if the arm moves 3°. The arm moved at most 0.3° in either test; the jaws stayed at 7.9 units.
- [First test](#/week/6/run/20261023T050700), on the arm's own timestamps: silent 0 gaps; **10 Hz with the gripper's
  angle 12 gaps (5.0 s of 20 in gaps, to 0.71 s)**; 5 Hz 3 (1.1 s); 2 Hz 0; **10 Hz without `angle6`, 0**; 10 Hz again 10
  (4.2 s). With `angle6` in the stream `arm_Feedback` funcode 1 also comes at 13-17 Hz, not 9. But in the block without it
  the arm's status stream (funcode 3) stopped after one message, and came back with the next block's.
- [Second test](#/week/6/run/20261023T051030): 10 Hz with `angle6` in every command, 15 gaps (6.2 s); in every 5th, 1
  then 0; every 10th, 0; never, 0 -- and in all three the status stream stopped (0.1 Hz, then none) and came back
  once the commands stopped. Silent before and after: 0 gaps, status at 10 Hz.

So: the gripper's angle in the commands makes the arm stall its servo side; any command without it stops its status
reports while the stream runs. Nothing in between has both. The holster program now sends the gripper's angle only in
the 3 commands after it changes (`arm_link` path option `gripper_on_change`, `GRIPPER_REPEATS`;
`d1_hardware.set_all_joint_angles(send_gripper=False)`); a `target` message, as the door, key and insertion programs
send, still carries it every time. `--gripper_every_command` for the old way. On the fake arm, 9 of 298 commands
carried it. What stopping the status stream means inside the arm is unknown; the link watches the angle reports, not
the status, while it moves. Tests: one new (`test_d1_hardware`: the gripper left out); 181 across the hardware and
robot suites pass. [Dry on the dog](#/week/6/run/20261023T051828_dry). Not yet run live.

[Live with `--timing`](#/week/6/run/20261023T052811_live) (Lukas): "the arm did not move". The first three commands
carried `angle6` and the arm followed them, Joint1 6.4 -> 15.9°; from the fourth on, without it, the arm did not move
again, and stopped its status stream; the program held the arm after 20 s ("77.8 deg from its goal"). **The arm does not
execute funcode 2 without `angle6`.** That is also why the held-pose blocks without it had no pauses: nothing was
executed, and a held pose cannot tell an ignored command from an executed one -- a flaw of that test. F-201 retracted;
F-202 in its place. Re-read with that: the pauses go with the rate of commands the arm executes -- 10 Hz 10-15 gaps in
20 s, 5 Hz 3, every 5th of a 10 Hz stream (2 Hz executed) 1 then 0, 2 Hz and 1 Hz 0. `send_gripper` and
`gripper_on_change` are removed; every command carries `angle6` again (a test says so). The start check allows Joint1
30° off the fold (this run left it at 16°). 181 tests pass.

5 Hz on a moving arm (Lukas: "ok, lets try that"): `holster_dog.py --hz 5` sends one command per 0.2 s point, the arm
link plays a path at its own `dt` (never under 0.1 s), and `PathPlayer` now allows one path step on top of its lead (at
5 Hz an 8.6° step plus the arm's lag passed 10° and the path waited every other command; a test covers it). On the CPU
model the jaws stray ≤ 0.53 mm from the line between 5 Hz commands (0.17 at 10 Hz), more than the sleeve's 0.2-0.34 mm.
[Dry on the dog](#/week/6/run/20261023T054555_dry), then [live at 5 Hz](#/week/6/run/20261023T054616_live), run by me under
Lukas's permission (dog sitting, nothing near), with `--timing`: all 13 moves; 143 commands, one every 0.201 s, steps
median 8.0°; the arm's angle stream **2 gaps in 28 s** (0.73 s in all, max 0.41 s) against 19 in 39 s at 10 Hz. Lukas:
"we should go back to 10hz, that was very choppy". So fewer pauses, but each 8° command visibly stops before the next.
10 Hz stays the default; `--hz 5` is kept for the record, not for use. 184 tests pass.

Lukas: "is there updated firmware online that fixes this stutter?" Searched 2026-10-23: no D1 firmware update anywhere
(Unitree's download pages list the D1-T and D2, not the D1; the D1 developer page renders by script and was not read;
F-024 has Unitree saying the D1 team is gone). One community project, [`unitree-d1-control`](https://github.com/yuhanliu0121/unitree_arm/tree/main/third_party/unitree-d1-control)
(MIT, unofficial, in [yuhanliu0121/unitree_arm](https://github.com/yuhanliu0121/unitree_arm)), says what is in the arm: its
computer (192.168.123.100) runs Unitree's `marm_controller_node`, which drives all seven servos -- FashionStar UART
servos, the gripper among them -- on one serial line, `/dev/ttyS4`, one "set raw angle" per servo per command, its
feedback polled on the same line. That fits F-045/F-199 (each command a move that ends) and F-200/F-202 (the line
stalls under a 10 Hz stream of seven-servo commands). The project replaces that controller over SSH (backs it up first;
"the arm may temporarily lose holding torque while the services are being switched"), aims at timed point-to-point moves
("a high-frequency waypoint replay API is outside the supported surface"), owns the line in one thread with its reads
spread to bound blocking. It does not claim smoother streaming; untried here. Not pursued: Lukas's call.

Lukas: "for the right leg unholstering, can you make the gripper fully open when it moves into grabbing position for the
key? right now its too tight of a clearance on the real dog". `holster.JAW_OPEN_M` 0.012 -> 0.030 m a finger (the D1's
full travel: servo 6 at 50.2 units, a 77.2 mm gap round the 20 mm bar against 41.2), for everything before the jaws close
and after they open; `holster_dog.JAW_S` 1.0 -> 1.5 s (~65 units to shut). The simulation uses the same. CPU model: the
open fingers ≥ 29.6 mm off the bar (11.5 before), the unfold ≥ 12.2 mm off holster and key, trunk proxy ≥ 36 mm; on the
dry trajectory the hand ≥ 9.2 mm off the D1's base, at `open` (re-opening at the bar; 16.9 before), 13.9-14.6 mm in `on`
and `off`. Tests pass. [Dry on the dog](#/week/6/run/20261023T121755_dry). Not yet run live.


### 2026-10-23 · The rattle on the dog reproduced in simulation: light legs and a long command delay, not the policy alone

The other PC's live run (round 4's final, `--presses 0`, the stop at 40°) held the full 10 s at 20-25° nose up, as
simulation predicts (F-194). It rattled: the FL and RR thighs and the RL calf flip their targets almost every step:
- before its latency cut: rms 0.23 / 0.18 rad on the FL / RR thighs, 1.6 / 2.3 rad/s;
- after it (the command 4.0 ms after the step starts): rms 0.13 / 0.09 rad.

The same 10 s stand from the dog's rest in simulation, actions dumped (`--leg_shadow_dump 96`, the waiting from 0.6 to
10 s). The step-to-step change of the targets, rms:

| legs, command delay | FL thigh | RR thigh | RL calf | FL / RR thigh speed, rms |
| --- | --- | --- | --- | --- |
| trained (armature 0.01 / 0.037), 0 ms ([run](#/week/6/run/20261023T112012_leg_eval_seed1_press_r4_force_5999_stand10_actions_delay0to0)) | 0.003 rad | 0.002 | 0.002 | 0.34 / 0.10 rad/s |
| trained, 5-15 ms ([run](#/week/6/run/20261023T111930_leg_eval_seed1_press_r4_force_5999_stand10_actions_delay1to3)) | 0.002 | 0.001 | 0.001 | 0.09 / 0.05 |
| trained, 15 ms ([run](#/week/6/run/20261023T112054_leg_eval_seed1_press_r4_force_5999_stand10_actions_delay3to3)) | 0.002 | 0.002 | 0.001 | 0.03 / 0.06 |
| trained, 20 ms ([run](#/week/6/run/20261023T112137_leg_eval_seed1_press_r4_force_5999_stand10_actions_delay4to4)) | 0.007 | 0.006 | 0.004 | 0.07 / 0.08 |
| trained, 25 ms ([run](#/week/6/run/20261023T112220_leg_eval_seed1_press_r4_force_5999_stand10_actions_delay5to5)) | 0.008 | 0.008 | 0.005 | 0.08 / 0.11 |
| armature 0.003 / 0.011, 5-15 ms ([run](#/week/6/run/20261023T112437_leg_eval_seed1_press_r4_force_5999_stand10_arm0.003_delay1to3)) | 0.002 | 0.002 | 0.001 | 0.09 / 0.06 |
| armature 0.003 / 0.011, 15-20 ms ([run](#/week/6/run/20261023T112603_leg_eval_seed1_press_r4_force_5999_stand10_arm0.003_delay3to4)) | 0.034 | 0.028 | 0.016 | 0.32 / 0.32 |
| armature 0.0002 / 0.0007, 5-15 ms ([run](#/week/6/run/20261023T112519_leg_eval_seed1_press_r4_force_5999_stand10_arm0.0002_delay1to3)) | 0.004 | 0.005 | 0.003 | 0.16 / 0.16 |
| armature 0.0002 / 0.0007, 15-20 ms ([run](#/week/6/run/20261023T112646_leg_eval_seed1_press_r4_force_5999_stand10_arm0.0002_delay3to4)) | 0.27 | 0.22 | 0.11 | 2.4 / 2.4 |
| the dog, before / after the latency cut | 0.23 / 0.13 | 0.18 / 0.09 | | 1.6 / 2.3, then 1.0 / 1.0 |

- **On the legs it trained on, the policy does not chatter**, out to 25 ms of delay (0.002-0.008 rad).
- **On light legs with 15-20 ms it does,** on the dog's three joints (the FL and RR thighs, the RL calf) and at the
  dog's size. A quarter of those stands rattle (FL thigh rms past 0.15 rad in 23% of 96).
- **Armature 0.0002 is UniFP's original value.** unitree_rl_lab leaves the Go2's at its robot file's 0 (F-168), which
  is what rima's walking policy trained on. Round 4 trained on 0.003-0.02.
- **The dog's legs are not identified.** Its rattle after the cut lies between armature 0.003 and 0.0002 at 15-20 ms:
  lighter legs, a longer delay or both. Neither is measured.
- **Nothing falls during the stand.** On light legs the press does: 124 and 135 of 288 fell after the 10 s, while
  lifting the foot or at the pre-contact point.

### 2026-10-23 · Round 4 on light legs, and a fine-tune on rima's legs

Lukas: "yes, run the light legs fine tune", then, of the first attempt's armature 0.0002-0.012: "is this what rima trained
on? ... I want to replicate rima since I know it works".

**What rima trained on.** It didn't. rima's walking policy trained on unitree_rl_lab's Go2 velocity task
(`~/unitree_rl_lab-main`, `tasks/locomotion/robots/go2/velocity_env_cfg.py`, `assets/robots/unitree.py`):
- **Legs and loop:**
  - the GO2HV curve on all twelve joints, the knee capped at the hip's 23.4 N·m;
  - kp 25 / kd 0.5 and joint friction 0.01;
  - no armature set (the USD's: 0 for Isaac Lab's Go2, F-168);
  - no command delay (the delay buffer's default 0).
- **Regularisation:**
  - observation noise on the policy's input: joint velocity ±1.5 rad/s, joint position ±0.01 rad, angular velocity
    ±0.2 rad/s, gravity ±0.05;
  - `action_rate` -0.1 and joint torques -2e-4. Ours: -0.02 and -5e-6, and no observation noise at all.
- **Randomisation:** floor friction 0.3-1.2, base mass -1 to +3 kg, pushes.

**The first attempt stopped.** ft5a and ft5b (armature 0.0002-0.012, delay 0-25 ms; ft5b with `action_rate` -0.2) were
stopped by hand at iterations ~6137 and ~6108 for rima's setup:
[ft5a](#/week/6/run/20261023T121418_train_seed3_press_ft5a_force_s3), [ft5b](#/week/6/run/20261023T121519_train_seed3_press_ft5b_force_s3).

**Round 4's final on light legs**, from the dog's rest:

| condition | presses | fell | past 60 N |
| --- | --- | --- | --- |
| [the dog's hands, armature 0.0002, 15-20 ms](#/week/6/run/20261023T122815_leg_eval_seed1_press_r4_force_5999_hands_light) | 10 of 288 | 83 | 166 |
| [the dog's hands, armature 0, 5-15 ms](#/week/6/run/20261023T122903_leg_eval_seed1_press_r4_force_5999_hands_zero) | 0 of 288 | 62 | 228 |
| [the frozen set, armature 0.0002, 15-20 ms](#/week/6/run/20261023T122952_leg_eval_seed1_press_r4_force_5999_dogrest_light) | 5 of 288 | 171 | 85 |
| [10 s stand, armature 0.0002, 15-20 ms](#/week/6/run/20261023T122649_leg_eval_seed1_press_r4_force_5999_stand10_light) | (no presses) | 135, after the stand | |
| [10 s stand, armature 0, 5-15 ms](#/week/6/run/20261023T122732_leg_eval_seed1_press_r4_force_5999_stand10_zero) | (no presses) | 146, after the stand | |

- **On light legs round 4 cannot press:** the dog's hands 0-10 of 288, struck past 60 N in most.
- **It stands,** with nothing falling in the 10 s. It chatters at 15-20 ms (FL thigh 0.27 rad rms) but not at 5-15 ms
  (0.004).
- **Its presses on the dog rest on the legs being as heavy as it trained on** (armature 0.003-0.02), which nothing
  has measured.

**ft6, the fine-tune on rima's legs.** From round 4's final, 1000 iterations alone:
- rima's legs and regularisation: armature 0, no command delay, unitree_rl_lab's observation noise (`--obs_noise`, new:
  `leg_cfg.OBS_NOISE`), `action_rate` -0.1 and torques -2e-4 (`--action_rate_weight`, `--torques_weight`, new);
- kept from ours: the geared knee, because the dog's rear knees measured 20-27 N·m in F-188, past rima's 23.4, and
  the crouch needs ~21-24; the 16 N upward cap; round 4's stands and layer;
- not replicated: rima's floor friction and base mass randomisation (not in this environment).

A first launch never started: argparse took `--torques_weight -2e-4` for an option, so no run directory was made. Its
queue went on and made round 4's baselines above.

    logs/unifp_train/press/train_one.sh force <round 4 model_5999.pt> press_ft6_force_s3 1000 --press_up_cap_n 16 \
      --press_hold_correction --press_release_from_press_point --press_plate_face_only --leg_armature_range 0 0 \
      --leg_delay_min_steps 0 --leg_delay_steps 0 --obs_noise --action_rate_weight -0.1 --torques_weight -0.0002

### 2026-10-24 · ft6 on rima's legs: no rattle, safer but weaker presses; exported for a stand-only test

[ft6](#/week/6/run/20261023T123207_train_seed3_press_ft6_force_s3) trained 1000 iterations. In training its force error fell 15.7 -> 10.3 N and its
damage share 75% -> 21% (round 4 at the end of its own: ~4.5 N, 3-14%), still falling.

The step-to-step change of the targets while standing 10 s from the dog's rest, FL / RR thigh, rms:

| legs | round 4's final | ft6 6600 | ft6 6998 |
| --- | --- | --- | --- |
| armature 0.0002, 15-20 ms | 0.27 / 0.22 rad | [0.005 / 0.004](#/week/6/run/20261023T131315_leg_eval_seed1_press_ft6_force_6600_stand10_light) | [0.005 / 0.002](#/week/6/run/20261023T130733_leg_eval_seed1_press_ft6_force_6998_stand10_light) |
| armature 0, 5-15 ms | 0.004 / 0.005 | [0.003 / 0.002](#/week/6/run/20261023T131358_leg_eval_seed1_press_ft6_force_6600_stand10_zero) | [0.003 / 0.001](#/week/6/run/20261023T130817_leg_eval_seed1_press_ft6_force_6998_stand10_zero) |
| trained (0.01 / 0.037), 5-15 ms | 0.002 / 0.001 | [0.002 / 0.002](#/week/6/run/20261023T131730_leg_eval_seed1_press_ft6_force_6600_stand10) | [0.002 / 0.001](#/week/6/run/20261023T131152_leg_eval_seed1_press_ft6_force_6998_stand10) |

Presses of 288 / fell / past 60 N, from the dog's rest:

| | round 4's final | ft6 6600 | ft6 6998 |
| --- | --- | --- | --- |
| the dog's hands, trained legs | 280 / 0 / 5 | [225 / 0 / 23](#/week/6/run/20261023T131646_leg_eval_seed1_press_ft6_force_6600_hands) | [208 / 0 / 59](#/week/6/run/20261023T131108_leg_eval_seed1_press_ft6_force_6998_hands) |
| the dog's hands, armature 0.0002, 15-20 ms | 10 / 83 / 166 | [123 / 0 / 15](#/week/6/run/20261023T131440_leg_eval_seed1_press_ft6_force_6600_hands_light) | [36 / 0 / 0](#/week/6/run/20261023T130900_leg_eval_seed1_press_ft6_force_6998_hands_light) |
| the dog's hands, armature 0, 5-15 ms | 0 / 62 / 228 | [82 / 0 / 30](#/week/6/run/20261023T131522_leg_eval_seed1_press_ft6_force_6600_hands_zero) | [90 / 0 / 29](#/week/6/run/20261023T130942_leg_eval_seed1_press_ft6_force_6998_hands_zero) |
| frozen set, armature 0.0002, 15-20 ms | 5 / 171 / 85 | [32 / 2 / 72](#/week/6/run/20261023T131604_leg_eval_seed1_press_ft6_force_6600_dogrest_light) | [11 / 8 / 45](#/week/6/run/20261023T131026_leg_eval_seed1_press_ft6_force_6998_dogrest_light) |
| frozen set, classic stand, trained legs | 196 / 0 / 5 | [48 / 2 / 31](#/week/6/run/20261023T131812_leg_eval_seed1_press_ft6_force_6600_real) | [24 / 21 / 33](#/week/6/run/20261023T131234_leg_eval_seed1_press_ft6_force_6998_real) |

- **The rattle is gone on every leg model,** and nothing falls in the 10 s stand. Its falls (8-12 of 288 in the stand
  runs) come at ~15 s, mid-press. Its waiting tilt reaches 21-22°: every stand is past 15°, none past 30°.
- **On light legs it is safe where round 4 is not:** the hands with no falls, against 62-83.
- **It is not accurate.** Its press failures are mostly off force and under-pressing (bias -1.5 to -11.6 N). The frozen
  set's harder presses mostly fail (24-48 from the classic stand, against 196).
- **Exported for a stand-only test** of whether the dog still rattles: `checkpoints/leg_press_force_ft6_model_6998.npz`
  (sha256 78909359...; numpy to 4.0e-6). Its `start_gate`, from round 4's stands, is pitch -3.8 to +1.0°, which refuses
  the dog's 8.1° rest unless the live program's start flags override it.

### 2026-10-24 · ft6's stand-only test set up on the dog (from this PC)

Lukas: "the other pc trained a new policy on main. can you pull it and set up the same standing test for me to do to see
if it rattles". Pulled e372b5b (F-196, F-197) onto this tree's uncommitted work. This PC's findings appended after the
other PC's F-191-F-195 had taken F-191-F-196 too: renumbered F-198-F-203 (in the findings, this log and the code that
cites them; the other PC's untouched), the merge keeping both sides. `checkpoints/leg_press_force_ft6_model_6998.npz`
matches `SHA256SUMS` here and on the dog. `press_live.py`'s uncommitted latency cut and `--start_on_blend` (on the dog
already, used by the last live stand) pass `tests/test_press_live.py`. `d1update`, then the same stand as the other PC's
last live run, dry: `d1dogpress --policy checkpoints/leg_press_force_ft6_model_6998.npz --presses 0 --stand_s 10
--tilt_limit_deg 40 --start_on_blend --start_tol_rad 0.3 --start_pitch_deg -15 5` ([dry](#/week/6/run/20261023T132955_leg_press)):
the policy loaded and ran at 1.7-2.1 ms a step, the dog lying (base 0.07 m), nothing sent. The live test is Lukas's (`--live`).

Lukas ran it live: a [first try](#/week/6/run/20261023T133220_leg_press_live) refused at once ("the base is 0.070 m up:
stand the dog in sport mode first"); then [the stand](#/week/6/run/20261023T133243_leg_press_live), standing in sport
mode: stand-up, blend, 501 policy steps (10.0 s, 50 Hz, inference median 2.68 ms), lie-down, damping, sport mode `mcf`
back; 33,300 LowCmd; largest tilt 16.6°. Lukas: "ok that worked with no rattling!" Over the policy's 10 s, computed the
same way on both runs (step-to-step change of `target`, rms; `dq` rms):

| | round 4, [rattling](#/week/6/run/20261023T110725_leg_press_live) (after the latency cut) | ft6 6998 |
| --- | --- | --- |
| FL thigh / RR thigh / RL calf, target change | 0.229 / 0.160 / 0.118 rad | **0.012 / 0.006 / 0.012** |
| worst joint | 0.229 | 0.015 |
| FL thigh / RR thigh / RL calf speed | 1.03 / 1.03 / 1.01 rad/s | 0.14 / 0.21 / 0.34 |
| pitch (nose up) / roll / base height | 4.9-20.8° / 0.2-5.3° / 0.210-0.287 m | 5.8-16.6° / -1.6-3.7° / 0.201-0.287 m |

(On round 4's run this calculation gives 0.23 / 0.16, where the entry above has 0.13 / 0.09 after the cut: a
different window or definition there; the table compares like with like.) The tilt peaked under simulation's 21-22°.
Not tested: presses (ft6 presses worse than round 4 in simulation, F-197), longer than 10 s, other stances.

Lukas: "am I now able to test it putting its right foot out on my palm?" The program does it (`--presses 1`, plate
`palm_up`, 5 N cap). A [rehearsal from this PC](#/week/6/run/20261023T133729_leg_press_rehearse) (dry, a pretend foot,
nothing sent) ended before the lift: the Enters were piped, not typed. Before a hand: ft6 is exported "not for presses"
(F-197). In simulation on the dog's hands it fell 0 times on every leg model; on the light legs that reproduced the
rattle it pressed 36 of 288 and none past 60 N (mostly under-pressing), on the others 90-208 with 29-59 past 60 N. The
press releases on the decoder's force estimate past 25 N, but that estimate reads 15-20 N with nothing touched (the dry
run lying; F-187), so it is not a guard to rely on.

**Two more of Lukas's runs, found unrecorded** in `logs/leg_press/` on 2026-10-24 and recorded then:
- [the first](#/week/6/run/20261023T133428_leg_press_live) started live and ended at once: 0 LowCmd, sport mode not
  read, nothing handed back or released;
- [a second stand-only](#/week/6/run/20261023T133506_leg_press_live) (the same flags): 501 policy steps (10.0 s),
  inference median 2.71 ms, largest tilt 15.3°, sport mode `mcf` back. Computed as in the table above: target change
  FL thigh / RR thigh / RL calf 0.012 / 0.006 / 0.011 rad (worst joint 0.015), speeds 0.13 / 0.22 / 0.36 rad/s, pitch
  4.7-15.3° nose up. It repeats the first stand's numbers: two stands without the rattle.

**Lukas's photo of the palm test** ([`screenshots/2026-10-24_ft6_foot_on_palm.jpg`](screenshots/2026-10-24_ft6_foot_on_palm.jpg),
his `20261024_004827.jpg`, taken 00:48 local, after the runs above). The photo: the Go2 standing on the rug, low and
nose up, the D1 folded on its back with the wrist RealSense on top, P2Dingo's charging dock and desk behind. One front
foot is lifted forward and rests in Lukas's cupped hand; the other three feet are on the floor. It shows that the
program lifted a front foot onto a hand. **Not shown:** the force, how long it held, or how it ended. That run's record
is on the dog, not on this PC (`d1fetch` at 01:00: "no dog answered"), so it is not recorded yet.

### 2026-10-24 · Weekly update 5 (Weeks 5-6), and a figure for it

**What.** `Weekly_Update_Lukas_5_20261024.docx` (in Downloads beside Update 4) covers Weeks 5 and 6 in one update, as
Lukas asked: "the writing style and conciseness of the week 4", "not more than a few pages", and no finding numbers,
written for a reader outside the repository. It was built on Update 4's own document, so its layout carries over. Four
pages. Update 4's six planned tasks were marked as follows:
- Bench torques: **not done**. The shoulder and elbow motor identified from its datasheet instead (F-142).
- The forearm and the CAD: **done** for the model (F-139-F-141, logged in Week 4 after Update 4 went out). The forearm
  was not measured with calipers, and the update says so.
- The camera mount: **done** for the print and fit. The hand-eye calibration was not done, and the update says so.
  Camera targets now reach the real arm through the head camera's look; the wrist keyhole finder was tested only on
  hand-held clips (F-186).
- The rope pull for hardware, the roll policy and the RSS decision: **not done**.

The extras are the real box's door (F-143-F-156), the D1 turning the key on the dog (F-183, F-184, and the chained run
with the door pull), the insertion's live runs, the front-right foot's force-aware against position-only comparison
(F-160, F-161, F-166, F-168, F-180), the foot-press policy on the dog (F-173, F-188, F-194, F-196, F-197, F-204 and the
palm photo), and the holster and the arm's stutter (F-189, F-198-F-203). Next week's six items: the key turn and door
pull repeated on the dog, the insertion through the wrist camera, presses on the dog, the bench torques, the hand-eye
calibration, and the RSS decision. The rope pull's hardware set-up and the roll policy are not in it. That is for
Lukas to confirm.

**Figure** (`python3 results/week_06/figures/weekly_update_figures.py`, CPU, system Python):
- [weekly_update_foot_force_vs_position.png](figures/weekly_update_foot_force_vs_position.png): force-aware against
  position-only, as a share of held-out cases, two seeds each. The key on the trained legs: 190 against 43 of 216
  (F-166). The key at armature 0.01 (labelled "realistic leg motor inertia"): 169 against 136 (F-168). Plates: 383
  against 158 of 576 (F-180). It is drawn from those findings' counts, not from logs.
- Images in the update (cropped, crops not committed): `arm_turn_demo_5n.png`'s four frames, the calipers photo, and
  Lukas's palm photo.

**Does not show.** Nothing new. The update's scope is the findings it summarises. The leg comparison is in simulation.
The dog runs are single runs, not G7 repeatability. The door's opening in the chained key-and-door run is not
claimed: the lidar's reading there was not believable, and the update says only that the dog gripped, pulled, let go
and folded.

### 2026-10-24 · The insertion takes its key from the holster, and rolls the wrist ±10° while pushing

Lukas: "I would like to now add this to the insertion task, so that once it has the key in its grasp, it tries to insert
it. the unholstering can happen once the dog has sat down in front ot the keyhole. also, improve the keyhole insert but
rotating the wrist 10 degrees in either direction while pushing so its more likely to go in".

**What changed** (`demos/combiner_key/robot/insert_plan.py`, `node.py --key_source holster|hand`, holster the default):
- After `lie_down`, `lie_settle` and `lock_look`, the key comes from the holster, not from a hand. The phases replace
  `unfold` and `load`: `h_turn` (Joint1 turned at the fold), `h_reach` (8 cm above the waiting point), `h_lower`, `h_on`
  (the jaws fully open, 30 mm, slid onto the bar's root), `grip` (checked as the hand-in's), `h_draw` (90 mm out). The
  joint goals are `holster.draw_plan()`, the plan `holster.check` verifies (F-198, F-199). `key_go` then takes the key to
  the lock. Its move is now checked for the key's whole bar against the trunk and clearance boxes (`key_path_margin`)
  as well as for the arm. If the jaws close on nothing at the holster, the attempt aborts and backs off the way it came
  (`h_letgo`, `h_off`, `h_raise`, `h_unreach`), stands and folds. `--stage load` draws the key and hands it back.
- The holster's grip is the hand-in's turned 180° about the shaft (roll +1), so the whole attempt holds the hand that
  way. At the lock this puts Joint6 at 2.08 rad at the touch, against the IK's 0.9 soft limit of 2.12. Rolled −10° it
  needs 2.27 rad (2.271 rad at most in the plan's poses, F-205). So the holster attempt lets Joint6 alone go to 0.98 of its
  range: 2.30 rad, 2.7° inside the 2.35 hard limit that `d1_hardware` clamps to (`HOLSTER_J6_SOFT`). `d1_ik.joint_limits`
  now takes one factor per joint (`solve_link6` and `ArmIK` pass it on). The other joints keep 0.9, and the hand-in
  attempt is unchanged. The plan's rows now carry `j6_rad`.
- The wrist's search: through `key_in` and `key_search` the hand rolls about the key's shaft by
  10° · sin(2πt / 2 s). The shaft stays where it is and only the wards turn. Once the key is in, the hand holds the angle
  it went in at through the seat (`check["wrist_at_entry_deg"]`). Before anything moves, the plan solves the touch and
  the seat at ±10° and the spiral's edge at ±10°.

**Tests** (`demos/combiner_key/tests/test_insert_robot.py`, 29 tests, all pass, 51 s; 4 new in `HolsterInsertTests`):
- lying down, it draws the key, then inserts and seats it, with the phases in that order;
- the wrist rolls within 10° both ways while pushing, and holds its entry angle through the seat (the keyway 2.5 mm and
  2.0 mm off; it went in at +3.1°);
- jaws closing on nothing at the holster abort and back off;
- `--stage load` draws the key and hands it back.

The test harness's keyway model was also fixed: off centre but inside the keyway, it now stops the key at the seat as
well as pushing it across.

Full suite after: 981 tests, the same 1 failure (`test_payload`), 15 skipped.

**What it does not show.** All of this is the CPU model: a rigid keyway with no friction and no wards. Nothing in it
says whether ±10° makes a real key go in more often. Not run on the dog: it did not answer at 2026-10-24 for `d1update` and a dry `d1doginsert`; they are next. No run on the dog has had a holster yet, so the draw at the
holster is still the simulation's (F-198) and the motion-only live runs'. Joint6 at 2.27-2.30 rad has not been
commanded on the real arm.

### 2026-10-24 · First live insertion from the holster: the lying look saw the lower door, not the lock

[Lukas's live run](#/week/6/run/20261024T010004_insert_insert_live) (`d1doginsert --live`, after `d1update`). The look
found the tag (22 frames), and the walk up was 0.72 m (stance 0.60 m ahead, 0.40 m left). The standing refit moved the
box 34 mm. The dog lay 0.252 m down, with the seated key 0.49 m ahead and 0.06 m right. `lock_look` aimed at servo 5
−22.1° and servo 1 7.7° (measured −21.1° and 7.4°), and found nothing in 15 searched frames at 5 places ("only 0-24% of
the rim seen dark inside"). The run reached `h_turn` and Lukas quit there (`q`, 65 s). Lukas: "the wrist doesnt tilt up
far enough to see the actual keyhole. I assume this is because when the dog sits its model of the combiner box lowers
with it? the expected position was way off" (with an RViz screenshot at `h_turn`).

What the record shows:
- **The model does not lower with the dog.** The lying base is the standing one lowered by the measured drop, and the
  box stays where the look put it. In the dry harness the model bezel is 0.315 m above the lying base and 0.49 m ahead.
- **The aim was the same as on 2026-10-22,** when the same look saw the bezel ~35 mm below and ~70 mm right of the
  model ([20261022T142556](#/week/6/run/20261022T142556_insert_insert_live): servo 5 −21.5° commanded, −20.6° measured;
  servo 1 6.7°). Then, the frames showed the grey door with the bezel, the warning sticker and the tag. Today's
  (`keyhole/`) show the door's lower part with the "shoals" logo, and no bezel, sticker or tag at any pan. The model's
  ring sits on the box's left edge in both, as before. A white printed part at the bottom of today's frames, which
  moves with the dog's body, is probably the new holster or its key.
- **So the camera looked lower on the box than on 2026-10-22,** with the same model, the same aim and the same measured
  arm angles. The difference is physical, not in the lying model. Candidates not yet told apart: the box or its tag
  moved or raised against each other, the wrist camera's mount changed, or the dog's lying pose. Base pitch is not in
  the trace.

Not recorded as a finding until the cause is known.

Lukas then asked for the arm back at the fold. It was holding `h_turn`'s pose: Joint1 +80.7°, the rest at the fold,
the gripper at 43.3 units. `d1_hardware.py park --joints 2.1 -89.8 90.0 -3.2 4.9 3.0 --execute` was run on the dog in
the arm image (Joint1 alone back, `h_turn` reversed; 200 cycles, worst error 0.8°). Read back: [+2.2, −89.8, +90.0,
−3.2, +5.7, +3.0], the gripper still open at 43.4. The arm is held, not released. Not a recorded run: `park` writes no
record.

Lukas: "nothing about the door has changed. you may do the lidar fit again". So the box and its tag are not the
cause. Still open: the wrist camera's mount, and the dog's lying pose.

**The lidar refit lying, back on** (`insert_plan._replan_lying`):
- After the sit-back is added, `plan.refine_fix` runs on `lie_settle`'s MID-360 scans. Each is placed from its base
  lowered by the measured drop, since the odometry keeps the standing height and the face's window is in height
  around the tag. It aborts past the standing refit's 8 cm / 6°; with no face, the sit-back alone stands.
- It was off from 2026-10-22 because the key was handed in before the dog lay down. Since then the key comes after
  `lock_look`, so the arm is folded and empty through `lie_settle`.
- CPU model (`test_insert_robot`, 31 tests, all pass; with `test_key_robot`, `test_robot` and `test_holster`, 114 pass):
  - with the modelled 40 mm sit-back, the refit moves the door 0.3 mm;
  - with 80 mm of sit-back, it moves it 39.9 mm and the key seats without the camera;
  - with no lidar lying, the sit-back alone stands.
  The two camera and touch tests of a door further than modelled now run with no lidar lying, so they still test
  those fallbacks.
- **It cannot fix what Lukas saw:** the refit moves the face along its normal and turns it, keeping the box's height,
  and today's miss was vertical. `d1update` done (12:42).

### 2026-10-24 · The Go2 front camera's lens calibrated on an A3 ChArUco board

Lukas asked what to print to calibrate the front camera's intrinsics, and whether several AprilTags would do. Loose tags
would not: an intrinsic calibration needs the points' relative positions exactly, and a tag's corner is a line fit that
the lens's curvature biases near the image's edge, which is where the distortion is measured. The print is a ChArUco
board, a chessboard whose white squares carry tag36h11 markers: the markers say which corner is which, and the corners
are chessboard saddle points. `charuco_9x6_40mm_A3.pdf` (repo root): 9x6 squares of 40 mm, 30 mm markers
(`DICT_APRILTAG_36h11`, ids 0-26), A3 landscape at exact scale with 100 mm bars on both axes. The rendered board shows
all 40 corners before it is written. `python -m demos.combiner_door.calibrate_front_camera board <pdf>` rewrites it.

Lukas taped the print to the box's door and recorded two videos through P2Dingo's web app (Camera, Go2 front camera,
Record), walking the dog round the board. The second has more angles and distances. Both were copied from the p2dingo
container's `photos/web/` to `logs/front_camera_calibration/raw/`: `go2front_1792805399724.mp4` (65 s) and
`go2front_1792805630065.mp4` (61 s), 1280x720 at 14.2 fps. The fit used the nominal 40 mm squares. A scale the same on both axes changes
no intrinsic. Later the same day Lukas measured the printed squares: exactly 40 mm.

**The dock's tag hid the board.** OpenCV's `CharucoDetector` found corners in 19 of 184 sampled frames of the first clip
and 36 of 174 of the second, though markers were found in 166 and 148. Every failing frame with markers had two id 0s:
the board's and the dock tag's on its stand beside the door. `calibrate_front_camera.board_markers` keeps a marker only
when another detected marker the board puts next to it sits where the board says. With it, 255 and 233 of 306 and 289
scanned frames gave corners, and 252 and 211 foreign detections were dropped.

**The fit** ([run](#/week/6/run/20261024T124011_calibrate);
`python3 -m demos.combiner_door.calibrate_front_camera calibrate logs/front_camera_calibration/raw/*.mp4`): every third
frame with at least 12 corners, 474 views, none dropped as outliers. `cv2.fisheye.calibrate` from `ONE_STAND`:
- 0.219 px rms (per view: median 0.19, max 0.66);
- fx 804.92, fy 806.89, cx 636.00, cy 352.61 (4 px left of the image's centre and 7 px above it);
- k1-k4 −0.0736, −0.0458, +0.0624, −0.0494.

Corners were seen from u 50 to 1243 px and v 46 to 663, dense in the middle and right, thin at the image's corners
([coverage](figures/front_camera_calibration_coverage.png)).

How well it is pinned:
- Fits on 40, 80, 150 and 400 views agree on f to 0.7 px and on the principal point to 1.3 px, and their rays to 0.1° at
  the centre. At the image's corners the rays differ by 0.4-1.3°.
- Each clip held out in turn: fitted on the second alone, the rays stay within 0.03° (centre) and 0.12-0.42° (corners)
  of the full fit. Fitted on the first alone, they are 2.2-6.1° off at the image's corners. The image's corners rest on
  the second clip.
- Held-out views' error with only their pose fitted: 0.19 px through the fit, 0.28-0.29 through `ONE_STAND`. A view's
  pose absorbs much of a wrong lens, so this separates the two weakly.
- `ONE_STAND`'s rays against the fit: 0.60° at the centre (the principal point), 0.6-1.8° at the corners.

**Against the taped stand** (the 0.61 m frame `ONE_STAND` was fitted on, which the board fit never saw): the tag's plane
**0.616 m** (taped 0.61, the lidar's door plane 0.603; `ONE_STAND` 0.6096 on its own fitting frame; `NOMINAL` 0.630).
The tag reads 6.8° off square, where the lidar saw the face tilted 4.72° and `ONE_STAND` 4.6°. The tag sits 290 px above
the principal point, where pitch is most sensitive to the lens. It is one frame of a ~100 px tag, so this is open: the
lens, the tag's own pose noise, or the camera's mount pitch.

**What changed in the code:**
- `front_camera.CALIBRATED` is now `camera_model()` for the look on the dog and the simulator's render. `ONE_STAND` and
  `NOMINAL` are kept for comparison.
- k1-k4 make the lens's polynomial turn back at 65.5° off axis, past the image's 60.6° corners, so a point beyond it
  would project into the image. `FisheyeModel.project` now returns NaN past `fold_angle_rad`, and `in_view` with it.
- RViz's frustum K is fitted to two opposite corners now that the principal point is off centre.
- `plan.FRONT_LOOK_MIN_ROOT_M` goes from 0.95 to 0.98 m: through the calibrated lens the tag kept 5 px at 0.95 in the
  worst standing pose (the rule is 20), and 22.6 at 0.98. The door pull's look band is now 0.98-1.14 m. The simulated
  look from `LOOK_BACK_M` 0.40 keeps 24 px (39 through `ONE_STAND`).
- The simulator's rendered-tag test at 1.2 m now allows 15 mm. Over 20 poses within 2-3 cm of that one, the error is
  1-14 mm through the calibrated lens and 3-21 through `ONE_STAND`: corner noise on a ~50 px tag. The 8 mm it had passed
  by the luck of its one pose.

**Tests:** `demos/combiner_door/tests/test_calibrate_front_camera.py` is new (3 tests: the dock tag beside a rendered
board, a lone foreign tag, a known lens recovered from synthetic views). The combiner door tests pass (71). The full
suite runs 987 tests, 15 skipped, with one failure: `tests.test_payload.TrunkParts.test_the_meshes_lie_inside_their_colliders`
(1,149 mount vertices outside both boxes). It touches no file changed here.

**What it does not show:** F-185's simulated 16 of 16 and its 6 mm were run through `ONE_STAND` and not rerun (no
simulator launched). The look has not been run on the dog through the new lens, and the dog's copy has not been updated
(`d1update` not run). The lens's optical origin is still the URDF's. (The square size, unmeasured when this was first
written, is 40 mm as printed: Lukas measured it.) → [F-206](../findings.md#f-206)

### 2026-10-24 · Second live insertion from the holster: the lying plan stopped on the wrist's −10°; the wrist camera looks ~10-15° low

[Lukas's run](#/week/6/run/20261024T022159_insert_insert_live) (`d1doginsert --live`, 13:22). The other agent's
calibrated front-camera lens was on the dog ("lens: calibrated"). The look found the tag (22 frames, the site 0.72 m
away at a bearing of −25.9°), and the walk up was 0.38 m. The standing refit moved the box 55.5 mm and 0.7°. The dog
lay 0.247 m down. **The lying refit ran on the dog for the first time:** 13 scans, the door moved 8.6 mm and 0.35°
after 35.7 mm of sit-back was added (the odometry saw 4.3). The seated key was then 0.47 m ahead and 0.09 m right,
with the door −5.1° off square. Then it aborted: "lying down, the arm cannot hold the key touch_wrist- (9 mm out of
seated): no IK pose clear of the trunk and holdable from here". The base stood and the arm held at the fold. The record
says `running` because the program was quit at the abort prompt.

**Why (CPU model, the holster's grip):** the door's angle hardly matters. At 0° to −8° off square, Joint6 peaks at
2.27-2.28 rad. The dog's standing height over the box's root does matter. The plan solves at 0.30-0.36 m (Joint6
2.15-2.30 rad) and fails on `touch_wrist-` from 0.39 m, as on the dog. The dog stands 0.35-0.39 m.

**Change:** each side of the wrist search is now the widest of 10, 7.5, 5, 2.5 and 0° whose touch, seat and spiral
edge all solve (`WRIST_SEARCH_STEPS_DEG`, `check["wrist_search_deg"]`). The sine uses each side's span. On the model:
0.353 m standing gives ±10°, 0.37-0.39 m gives −7.5°/+10°, and 0.40-0.42 m gives −5°/+10°. A new test at 0.39 m
narrows the search and seats the key; 32 insertion tests pass.

**The wrist camera, from Lukas's screenshot of this run**
([`screenshots/2026-10-24_insert_lying_abort_rviz.png`](screenshots/2026-10-24_insert_lying_abort_rviz.png)).
- The screenshot is RViz at the abort. The wrist camera is at the fold (servos [2.3, −89.7, 90.1, −3.0, 5.4, 3.0]),
  looking down on the dog's head. It shows the MID-360's dark dome under its white printed cage and ring, the rug, a
  red bin, and the D1's grey base at the lower left. The 3D view shows the dog lying, the arm folded, and the door's
  lidar points in green.
- The camera model (`plan.wrist_camera_base`, the saved mount, fx 907) puts the lidar's top at (776, 601) and the
  cage's top at (790, 585) in the 1280x720 image. In the frame they are at about (780, 442) and (817, 333). The
  horizontal position agrees; the image shows them 160-250 px higher than the model.
- Things below the camera appear higher in the image when it pitches down, so the real camera points ~10-15° further
  down than the model. That is the size of the miss in the first run's look, where the frames showed the door's lower
  part with the same aim that saw the bezel on 2026-10-22. The dome also looks smaller than modelled (~230 px across
  against ~340), so the camera's position may be off as well.
- **Not validated:** the positions are read by eye off a screenshot, one frame. It points to the wrist camera's mount
  having moved since 2026-10-22, not the box (Lukas: "nothing about the door has changed") or the lying model.
- **Confirmed by Lukas:** "i realised the wrist camera sagged, it has been fixed now". So both of today's misses at
  `lock_look` were the camera's mount, not the model. Its saved pose (`resolve_mount`) is not re-measured.
  The next lying look shows whether the fix brought it back to 2026-10-22's view.

The first run's RViz screenshot is now [`screenshots/2026-10-24_insert_h_turn_rviz.png`](screenshots/2026-10-24_insert_h_turn_rviz.png):
the wrist frame at `h_turn` shows a wall, the box's edge and the lower door's "sh" logo. The lidar view shows the dog
lying, the arm's plan ghost turned at the fold, and the door's points in green.

### 2026-10-24 · The camera fixed: the bezel seen; the wait after lying down; every arm motion the holster draw's way

**[Lukas's run at 13:34](#/week/6/run/20261024T023431_insert_insert_live)** (`d1doginsert --live`, after he fixed the
wrist camera's sag):
- The lying refit moved the door 24.9 mm and 0.29°, with 42.2 mm of sit-back added. The plan's wrist search was
  −7.5°/+10°.
- `lock_look` found the bezel, the insertion was planned again (±10°), and the holster draw ran through `h_turn` to
  `h_draw`.
- **`key_view` saw the bezel in every frame** ("how": image; offsets 0.5-6.3 mm, range 0.163-0.167 m against the
  model's 0.163-0.169).
- In `key_in` the key went 12.5 mm past the model's seated with no touch registered. Then, 21 s in, it aborted: "the
  arm is 16 deg from where the IK wants it: loaded". Servo 6 was commanded between 101° and 121° by the wrist search's
  sine and ran up to 17° behind it; the other joints were within ~1.5°.
- It recovered: key out, home, stood, hand_back, drop, fold.
- Not shown: whether the key was in the keyway when it stopped.

**[13:38](#/week/6/run/20261024T023816_insert_insert_live)**: started live, with an empty trace and no events. It was
stopped at or before the look.

**Fixes** (`test_insert_robot` 33 tests; full suite 994 tests, the same 1 failure, `test_payload`, 15 skipped):
- **The load check leaves out Joint6 while the wrist searches** (`InsertAttempt._lag_joints`). The roll's lag is not a
  load, and the key's depth is on the roll's axis, so the touch still sees the face.
- **The wait between lying down and the look** (Lukas: "there is a long wait between sitting down and beginning the
  wrist camera search").
  - The trace shows 21.3 s with no step at the start of `lock_look`: the insertion's plan, run as `lie_settle` ends.
  - The −10° wrist search could not be reached from that stance, and each pose out of reach cost the IK all its
    seeds: 0.5 s on the PC, ~20 s on the dog.
  - The roll is Joint6 alone (the key's shaft is on its axis; checked against the IK's own: Joint6 within 0.005 rad,
    the other joints within 0.005), so the rolled poses are the unrolled ones with Joint6 turned, checked for limits,
    trunk, holding and head camera, with no IK (`_rolled`).
  - The lying replan is 0.04 s on the PC at 0.353, 0.39 and 0.42 m (0.55 s before at 0.39), with the same spans.
- **Every arm motion the holster draw's way** (Lukas: "can you also ensure all the arm motions use the same method of
  smooth commands as the unholstering does?").
  - The node sends `plan.ArmSender`'s messages, not `target`s for `arm_link.Profile`.
  - Joint moves, and goals that jump more than 6°, become one paced timed path (`plan.joint_path`, 40°/s, 300°/s²).
  - All other goals are followed (`arm_link.Follower`, a new `follow` message): each command is the goal itself, at
    most 45°/s, with a path's lead on the report.
  - `keepalive` now carries the jaws' command.
  - The clearance checks (`key_go` arm and key, the look's moves, `key_out`) check the paced path. The door, key and
    insertion harnesses drive a `SimulatedLink` through the same sender (`LinkedArm`).
  - In the dry runs: the door pull sends paths for `unfold`, `home` and `fold`; the holster insertion for `lock_look`
    (6), `h_turn`, `h_reach`, `h_lower`, `key_go`, `home` and `fold`. Everything else is followed, including `h_on`,
    `h_draw`, the `key_*` phases, release and ungrasp.
  - One counterfactual changed: the door's old ending straight home went into the lever as a straight joint line; as a
    paced move it passes 7 mm off it, still under the 10 mm the ungrasp keeps. The test now says so.
  - **Not validated on the dog:** how smooth the followed phases look there. A followed goal moving under 2.5° a step
    (the key going in at 2 mm/s) still stops and starts in the firmware.

### 2026-10-24 · The remote's A and B, the same in every live program

Lukas asked that A on the controller do what Enter (or `live` + Enter) does in every live demo, and B stop it, with A
pressed again after a stop doing what `r` + Enter does: back to the start. The live programs are the door, key and
insertion node (`d1dog`, `d1dogkey`, `d1doginsert`: `combiner_door/robot/node.py`), the foot press (`d1dogpress`:
`press_live.py`) and the holster draw (`d1dogholster`: `holster_dog.py`). The shadow runner sends nothing and is left
as it was. No run was launched.

- **One rule** (`lowlevel.RemoteButtons`): each button counts once, as it goes down. B counts with any key held (L2 + B
  is sport mode's own damping, a stop too). A counts only on its own: L1/L2 + A belong to sport mode (L2 + A stands the
  dog up or lies it down).
- **Door, key, insertion:** the presses come from `/lf/lowstate`, which the node already reads for the legs, and go
  onto its key queue. B is `a` + Enter. A is Enter, or `r` + Enter once aborted (open, back out if at the lever, fold).
  The prompts say so.
- **Foot press:** A is Enter everywhere and `live` + Enter at the confirmation. B is now `a` + Enter in every mode, not
  only while the legs are driven. After a stop the program sits in damping until Enter, so A then gives the legs back
  to sport mode, where the run started.
- **Holster draw:** it had no way back: `a` held the arm and the program exited. Now, after a stop, `r` + Enter or A
  takes the arm back to where it started (`holster_dog.recovery`). Stopped on the way out (up to `hold`), it plays the
  moves so far backwards, from the played point nearest the arm, each with the jaws as they were before it. From
  `back` on, it finishes the plan's way back. Either way the arm goes only where it has been or where the plan goes. A
  stop during the way back waits again, and the next A carries on from there. `q` + Enter or Ctrl-C quits with the arm
  held. The program reads the remote through its own ROS node on `/lf/lowstate`, dry runs included. Its launcher now
  starts it with ROS sourced and P2Dingo's `cyclonedds_go2.xml`, as `run_press_on_dog.sh` does. Without ROS it says so
  and keeps to the keyboard.
- **The bits had been read on the dog already** (F-207). This log said on 2026-10-23 that B was "still not seen". But
  [the stand](#/week/6/run/20261023T133243_leg_press_live) recorded L2, L2 + A and B in its `remote` column at 60-68 s,
  while it waited in damping. Every other recorded press run reads 0. `lowlevel.py` and the robot README are corrected.
- **Tests:** `tests/test_remote_buttons.py` (new, 10). It covers the presses and sport mode's combinations, the door
  node's keys on a bare node (A goes, B aborts, A recovers; typed text is never taken for the remote), and the holster
  way back on the simulated arm. Stopped 8 ticks into `draw`, it undoes `draw` to `turn` over played points only and
  ends within 2° of the start. Stopped in `off`, it finishes `off`, `raise`, `unreach`, `fold`. Stopped again while
  undoing `on`, a second recovery gets home. `tests/test_press_live.py` gained a whole live run on the stand-in driven
  by the remote alone: A confirms and lifts, B at `at_pre` damps, and A hands back. With `test_holster.py`: 62 tests,
  all pass. Full suite: 1005 tests, the same 1 failure (`test_payload`), 15 skipped.
- **Not validated:** none of it on the dog. The remote has never been read on `/lf/lowstate`, and B has never stopped a
  run. Not checked: whether sport mode does anything itself on A or B pressed alone. A dry run of `d1dog` or
  `d1dogholster` with the remote in hand should print `the remote's A` / `the remote's B` first. The holster way back
  starts a joint-space move at its cruising pace when stopped mid-move. It is checked on the simulated arm only.

### 2026-10-24 · Never out of range while the tag is seen: the approach

Lukas: "can you make it so that the box is never out of range as long as the dog sees the qr code". Built:
`demos/combiner_door/robot/approach.py`, in the robot program for all three tasks.

When a look finds the box out of the task's range, a live run no longer stops. It walks to a staging pose, looks again,
and carries on.
- *the staging pose*: the task's own stance (the door pull's, the key's push, or the insertion's) moved straight back
  along its facing until the box's root is 1.05 m ahead. That is inside the front camera's look band (>= 0.98 m,
  `plan.FRONT_LOOK_MIN_ROOT_M`) and the wrist's (0.75-1.3 m), and the door pull's walk in from there is 0.36 m.
- *the walk*: the key's walk up (`key_plan.WalkUp`: forward or back, sideways and turning at once), with 40 s and three
  corrective steps. It is sent short by the odometry's 13% (`plan.ODOM_WALK_SCALE`) and done within 8 cm and 6 degrees.
  The arm is left where it is: nothing is sent to it.
- *the limits*: up to three approaches, at most 3 m each.
- *not walked to*: a tag whose lidar face is not the box's door (the dock).
- *dry runs*: say how far a live run would walk, and stop.

Also from this change:
- the walk up's timeout and step count, and the walk in's wait, are now class attributes (`walk_up_timeout_s`,
  `walk_up_max_steps`, `walk_in_timeout_s`).
- the trace keeps its task's columns from the first row, so an approach before an attempt does not drop them.

Tests (`demos/combiner_door/tests/test_approach.py`, dry, the classic walk's carry, odometry at 87%):
- each task's staging pose puts the root 1.05 m ahead;
- from boxes 2.2 m ahead, 1.6 m ahead and 0.7 m to the side turned 20 degrees, 1.3 m and 0.8 m the other way turned
  15 degrees, and 0.55 m ahead (too near: it backs off), the approach stops within 12 cm and 8 degrees of the staging
  pose, and a new look from there is in range for the door pull, the key and the insertion.

A stubbed node took the approach path: it walks 1.01 m and turns 15 degrees from a box 2 m off, and looks again after.
The door's 36 and the key turn's and insertion's robot tests pass. **Not run on the dog.**

### 2026-10-24 · The v4 lever and the holster on the Go2's rails, in the holster's simulation and tests

Lukas: "there are 2 new stl files describing the new lever and holster design (the holster now slides into the go2
rails on the expansion dock). can you add it to the sim tests". The files: `~/Downloads/holster_lever.stl` (the holster
with the lever in it) and `lever_only.stl`. Registered with `python -m demos.combiner_key.holster
~/Downloads/holster_lever.stl ~/Downloads/lever_only.stl --write` (`holster.register_v4`).

**What they are:**
- *the frame*: both are in the 10-23 exports' CAD frame. The new holster's unchanged faces lie on the 10-23 holster's
  (median 0.01 mm), so `CAD_TO_D1_BASE` and the holster's frame on the D1's base carry over.
- *the holster*: one print in two bodies, the sleeve (46.0 cm^3) and a strut down to a slider for the expansion dock's
  rails (17.5 cm^3), plus the export's slivers, dropped.
- *the lever*: a plain bar 15 x 20 mm, 185 mm long with its far end rounded, no platform, 33.8 cm^3 (42 g solid PLA;
  v3 is 43.0). It lies on the holster frame's axes to 0.06 mm. Its shaft is 9.94 mm further out along the holster's
  axis than v3's: **the "1 cm longer" lever the 10-23 draw assumed** (`LEVER_EXTRA_M`), its measured frame within
  0.07 mm of that assumption. Its wards are at 45 degrees to the bar, where v3's lie along it. With the lock's keyway
  unchanged it would go in level or vertical, not at 45 degrees; this is not settled, and the insertion and turn
  simulations still use v3.
- *the sleeve's grip*, from rays out of the bar's middle against the 10-23 boxes' faces:
  - the back wall and rails unchanged;
  - the lips 0.15 mm further out (0.35 mm off the bar);
  - the front hook 0.65 mm further out (0.83 mm off);
  - the cup's end 0.14 mm further out;
  - the bottom lip and the hook now run on to the cup (from 113.5 to 151 mm along).

**Changed:**
- `holster.HOLSTER_BOXES` re-measured; `KEY_BOXES` is now v4's (bar and shaft, no platform; `V3_KEY_BOXES` keeps v3's);
- `key_in_d1_base` takes the measured v4 frame;
- `assets/holster/holster_d1_base.stl` is now v4's holster (sleeve, strut and slider); `assets/lever_v4.stl` is new;
  the registration keeps the 10-23 one under `replaced`;
- the simulation's holster key (`asset.build_holster_key_usd`) is the v4 STL at 41.9 g, where it was v3 stretched 1 cm.

**Tests** (`demos/combiner_key/tests/test_holster.py`, 21 pass, 5 new):
- the v4 lever is where the 10-23 model put the longer one, to 0.1 mm;
- its colliders are its STL's extent;
- the saved holster mesh never cuts into it, the sleeve within 0.5 mm of it;
- the arm along the draw and the unfold stays as far from the strut and slider as from the sleeve: 36 mm from any
  part of the holster, 37.8 mm folded at rest;
- the 10-23 registration is kept.

**On the CPU arm model** (`holster.check`): the draw is unchanged, joints at 89% of their range, the hand 81.6 mm from
the sleeve. Without the platform the key is 21.8 mm from the arm's links (15.5 with v3 stretched) and 14.3 from its
base (11.6).

**In simulation** (Isaac, `run_holster.py`, seed 1):
- [on the display](#/week/6/run/20261024T042418_seed1_v4_lever_rail_holster), Lukas watching, one attempt: drawn
  89.5 mm, the jaws closed 0.4/0.7/0.3 mm off the planned grip, the key 2.3 mm at most off it in the hand, the highest
  joint torque 67% of its limit, 14.5 s.
- [the holster 3 mm off](#/week/6/run/20261024T042551_seed1_v4_err3mm), 8 attempts headless: **8 of 8 drawn**,
  89.0-89.5 mm, no falls, the grasp up to 3.0 mm off where it was planned, fingers' contact at most 2.9 N, the key
  1.2-2.8 mm off in the hand. The torque topped out at 64-67% in six and at the limit in two.
- the 10-23 model (v3 stretched, the 10-23 holster), against these: 3 of 3 on the display at 123-128% torque, 8 of 8
  with the 3 mm error, at 59-100%.

So the new design draws as the old did, at less torque. That the lighter, platformless lever is why is likely, not
shown. **Not shown**: the rails' fit, the strut's stiffness, and the print. The simulation's holster is carried
kinematically with the base, and its contacts with the robot's own links are filtered out.

### 2026-10-24 · The holster grip 18 mm short of the shaft on the dog; the dog's draw taken deeper

Two live insertions from the holster with the level key (`d1doginsert --live --yes`), fetched with `d1fetch`:
[05:08 UTC](#/week/6/run/20261024T050857_insert_insert_live), stopped by Lukas before the touch; and
[05:11 UTC](#/week/6/run/20261024T051120_insert_insert_live), stopped by Lukas after the release. In the 05:11 run the
program called the key seated (the contact at 1.46 mm, the encoders' key 0.5 mm off the seated depth). Lukas's photos
show it missed: the shaft below and to the right of the keyhole. Lukas: "It is consistently grabbing the lever like
this... leading it to miss the keyhole since the key-bit is to the right. Can you add an offset so it grabs it properly
in real life". The photos show the jaws closed on the bar's free end, past the shaft, with the shaft just beyond the
fingers. That puts the shaft ~18 mm along the bar from the gripper's axis, on the platform's side. The insertion holds
the shaft on that axis (`insert_plan.KEY_IN_JAWS_LINK6`), so the shaft went in to the keyhole's right.

**Not the arm's tracking:** the encoders' forward kinematics put the jaw centre within 1.4 mm (05:11) and 0.9 mm
(05:08) of the planned grip along the bar at the end of `grip`, and 1.3-1.8 mm across it. The lever sits deeper in
the holster than the registration has it, or the arm's geometry is off along that axis; which one is not known.

**Changed:**
- `holster.DOG_KEY_DEEPER_M` = 18 mm: the dog's draw (`holster.draw_plan`, the insertion's) moves the key that much
  further into the holster along the bar, so `pre`, `grip` and `out` move with it. The simulation's draw keeps the
  model's lever. 18 mm is half a 26 mm finger plus the 10 mm shaft's radius, read off the photos, not measured.
- On the CPU model the deeper draw solves only with the shoulder past the 0.9 soft limit. Deeper by 0, 6 and 10 mm it
  solved at 0.9; by 14 and 18 mm the slide onto the bar failed near its end. So the holster attempt's IK now takes
  Joint2 to 0.95 of its range (`insert_plan.HOLSTER_J2_SOFT`; 85.5° against the 90° the hardware clamps to; Joint6
  keeps 0.98). With that limit set, at 18 mm deeper: Joint2 reaches 0.934 of its range (84°); the unfold clears by
  19.9 mm; the hand stays 63.6 mm off the holster and the key 11.5 mm off the arm's base. 24 mm deeper does not solve,
  so about 6 mm is left for tuning.
- The hand-in's prompt (`LOAD_TEXT`) said to hold the lever "up and to the right at 45 degrees". It now says level
  and to the right, platform up.
- Tests: `test_holster` and `test_insert_robot`, 56 pass (2 skipped).

**Not shown:** the deeper grip on the dog. The program's "seated" with the shaft not in the keyhole is not explained.
A likely cause, not checked in the trace: the shaft, off to the side, met the bezel or the door near the depth the
model gives the seated key, and the touch took that for seated.

**Reverted the same afternoon; the offset moved into the insertion.** On the dog, the deeper draw ran the wrist motor
into the shoulder motor (Lukas: "we have to revert to the old, as the wrise motor now collides with the shoulder motor.
can we instead add the offset the insertion phase itself"). The CPU model's draw check does not test link against
link. `DOG_KEY_DEEPER_M` and `HOLSTER_J2_SOFT` are gone, and the draw is the model's again. Instead:
- `insert_plan.HOLSTER_SHAFT_ALONG_M` = 18 mm: with the holster's grip, the insertion takes the key's shaft to be that
  far along the bar from the jaw centre (`key_in_jaws_link6`). So `jaw_for` holds the jaws 18 mm short of the keyway,
  and the encoders' key (`key_in_hand`, `key_path_margin`) is where the shaft really is. The hand-in keeps the shaft on
  the jaws' axis.
- The wrist's ±10° search used to turn Joint6 alone, which with the shaft off its axis would swing the shaft ~3 mm.
  It now turns the hand about the shaft: the jaw target moves with the rolled bar. The plan's check of the roll
  (`_rolled`) solves the IK for the holster's grip, seeded from the unrolled pose.
- `key_in_hand`'s axes are turned 180° for the holster's roll +1, so the drawn held key no longer points the wrong way.
- CPU model, holster grip, standing at 0.353 and 0.39 m: with the offset, the full ±10° search plans and the key seats.
  Against no offset, Joint6 rises 0.04-0.05 rad, to 1.60 rad at most, and the head camera stays at least 135.7 mm
  clear. With the model's shaft on the axis and the true key 18 mm off, the run aborts: "no keyway within 4 mm of where
  the look put it". The dog instead called that seated, which is still not explained.
- Tests: `test_insert_robot` 36 (two new: the offset grip seats where the unoffset one misses; the roll keeps the shaft
  within 0.5 mm), `test_holster` 22 (2 skipped), all pass.

**Not shown:** the insertion with the offset on the dog. 18 mm is still read off the photos.

**18 mm overcorrected:** on the dog the key pressed 8 mm left of the keyhole (Lukas). The default is now 10 mm, and
`node.py --shaft_along_mm` sets it per run (`InsertAttempt.shaft_along_m`, recorded as the check's `shaft_along_mm`)
for Lukas to tune by trial. Tests: `test_insert_robot` and `test_approach`, 38 pass. The roll test now expects the
offset's chord rather than a fixed 1 mm. Synced to the dog 16:49.

**Lukas's trials found 6 mm** ("I find this to be the successful offset after a few tests: `--shaft_along_mm 6`"), now
the default `HOLSTER_SHAFT_ALONG_M`. The runs themselves are on the dog and not yet fetched or recorded.

**Three more changes (synced 17:09):**
- *The offsets, renamed and a vertical one added* (Lukas: "expose a vertical offset, with positive being up. rename the
  horizontal one ... have right be positive and left negative"). `--key_right_mm` and `--key_up_mm` (default 0) move
  where the key lands at the lock, on top of the 6 mm model. Internally they move the shaft's place in the jaws the
  other way (`shaft_along`, `shaft_up`), so the wrist's search still turns about the shaft. `--shaft_along_mm` is gone.
- *The draw taken fully out* (Lukas: "the lever isnt pulled fully out of the holster before it is moved towards the
  lock"). In the model, 90 mm leaves the far end 15 mm past the holster's open end. On the dog the arm was 6 mm behind
  at the end of `h_draw`, and the grip is ~6 mm further from the far end than modelled. The dog's draw is now 100 mm
  (`DOG_SLIDE_M`), as far as the arm reaches along the holster's axis on the CPU model: 105 mm does not solve. It needs
  Joint1 and Joint5 at 0.97 of their range through the draw only (127.7° and 86.5° at its end, against 134.6° and
  90°). A new `h_out` holds the end for 1.5 s. Then the encoders must put the jaws within 4 mm of 100 mm along the
  axis (`check["drawn_mm"]`), or the attempt aborts at the holster. The model still does not test the arm's links
  against each other.
- *A 45° turn once seated* (Lukas: "after inserting, try rotating 45 degrees before backing away", and "assume the key
  is in the roll axis perfectly, so you only need to rotate the wrist"). `key_turn`: Joint6 alone, clockwise from the
  angle the key went in at, over 4 s, then held to 5 s. The jaw centre stays where it was seated. Then the jaws open
  and the hand backs away, the key left in the lock. `--turn_deg` sets it; 0 means no turn. The plan takes 5° less, down
  to 30°, if Joint6 cannot hold the seated pose turned that far past the search's widest. The lock holding the key
  shows as the arm's lag and aborts; an abort mid-turn opens the jaws where they are and backs away empty
  (`left_in_lock`).
  CPU model, holster grip, standing at 0.353 and 0.39 m: plans 45°; servo 6 moves −46°, the others under 1°; measured
  48.2° (3.1° entry + 45) and 45.1°. The hand-in's grip has no Joint6 range for it: its plan takes no turn.
- Tests: `test_insert_robot` 41 (new: the nudges, the short draw, the turn, no turn, an abort mid-turn),
  `test_holster` 22 (2 skipped), `test_approach` 2, all pass.

**Not shown:** whether the D1's wrist turns the cam lock at all; the 100 mm draw on the dog.

**The draw's abort removed.** Lukas's next live run (06:10 UTC, `d1doginsert --live --yes`) aborted after `h_out`:
"the key drawn 96 mm of 100 along the holster: the lever may still be in it". Lukas: "can you remove this abort - It
has cleared it by 2cm". So the encoders' draw length is still recorded (`check["drawn_mm"]`) but no longer stops the
run.

**The afternoon's holster insertions, fetched and recorded** (all `d1doginsert --live --yes`, the shaft model as named):
- [05:27](#/week/6/run/20261024T052724_insert_insert_live), 10 mm: the key slipped out of the jaws in `key_go` (servo 6 at −19.4 units, shut −19.8).
- [05:41](#/week/6/run/20261024T054143_insert_insert_live), 10 mm: touched 8.9 mm out of seated, searched 1.5 mm; Lukas stopped it.
- [05:50](#/week/6/run/20261024T055000_insert_insert_live), 10 mm: touched at 7.4, entered from the search at 6.1; Lukas stopped it.
- [05:54](#/week/6/run/20261024T055416_insert_insert_live) and [05:57](#/week/6/run/20261024T055717_insert_insert_live), `--shaft_along_mm 6`: **straight in**, stopped
  at 1.41 and 0.40 mm out of seated, no search. Lukas: "the successful offset".
- [06:10](#/week/6/run/20261024T061025_insert_insert_live): drawn 95.8 mm, aborted on the draw check (removed since).
- [06:14](#/week/6/run/20261024T061414_insert_insert_live): drawn 94.8 mm; **straight in** at −1.43 mm, then **turned: the encoders put the key
  at 40.3°** (planned 45). Whether the lock itself turned is not recorded.
- [06:16](#/week/6/run/20261024T061625_insert_insert_live): drawn 95.6 mm. `key_in` stopped at 5.75 mm with nothing there; the search entered at
  4.07; `key_seat` ran on to −2.6 mm and aborted ("the key did not stop at seated: 8.6 mm past the lock's face and its
  command 13.8").

**The door a cm nearer than it is?** Lukas, on the 06:16 run: "it thinks the door is about a cm closer than it actually
is ... and hence it always starts its insertion wiggle very early. can we offset this". The runs above say the
model's depth is within ~3 mm: three keys straight in stopped between −1.4 and +1.4 mm of the model's seated, and the
06:16 one stalled at −2.6. A 10 mm deeper model would take a seated key for one on the face, 10 mm short. So the
default stays (`insert_plan.LOCK_DEEPER_M` = 0). `--key_in_mm` (positive: further in) shifts the insertion's box along
the door's normal for a run (`InsertAttempt._box`; the door's fit, the stance and the look are not moved). The early
wiggle was the wrist's ±10° search running from the start of `key_in`, 25 mm out of seated, 16 mm before the face. It
now starts with the key 4 mm off the face by the model, or once it has touched (`WRIST_START_M`). What stopped
`key_in` at 5.75 mm in the air on 06:16 is not diagnosed. The key was crawling in (20.5 to 6.4 mm over 8 s), and the
touch is the encoders' lag growing.
Tests: `test_insert_robot` 43 (new: the depth nudge both ways; the search not started far from the face), all pass
with `test_holster` and `test_approach`. Synced 17:28.

### 2026-10-24 · The v5 holster with a mouth: registered, drawn and put back in simulation, and the dog's program puts the key back

Lukas: "there are 3 new lever2 stl files that modify the holster so that it has a mouth to guide the lever back
inside, and puts the lever another cm higher so the grip can properly get it without collision. can you add these to
the sim, and then make it so that the arm puts the lever back in the holster after unlocking the door". Files:
`~/Downloads/holster_with_lever2.stl`, `lever2.stl`, `d1_with_holster_and_lever2.stl`. Registered with
`python -m demos.combiner_key.holster ~/Downloads/holster_with_lever2.stl ~/Downloads/lever2.stl --write --design v5`.
v4's holster mesh is kept as `assets/holster/holster_v4_d1_base.stl`, and its registration under `replaced`.

**What they are** (`holster.py`'s v5 paragraph):
- *the lever*: v4's, the same 384 vertices, moved 15.000 mm out along the holster's axis (9.8 mm up, 11.3 forward).
  That is the "another cm higher". Lukas, asked about its platform (where the model had it, it passes through the new
  mouth's front wall): "No platform now". `KEY_BOXES` drops it; `V4_KEY_BOXES` keeps v4's.
- *the holster*: the sleeve and the strut are where v4's are. The cup is filled 15 mm, its end 0.09 mm off the bar's,
  and there is no lip over the bar's top beyond the back wall's end. The mouth is a separate body (11.4 cm^3) flaring
  from the channel 107 mm along the lever to its lip at 87 mm. It has four flat walls at 27-33 degrees, and a fifth,
  twisted face across its front-bottom corner on the D1 base's side (the base's mesh 0.4 mm off it).
- *the D1*: in `d1_with_holster_and_lever2.stl` the folded arm spans the 10-23 exports' extent, so `CAD_TO_D1_BASE`
  holds.

**The draw on the CPU model, v5:**
- From the grip 15 mm further out, the arm reaches only 76 mm along the holster's axis at 0.9 of every joint's range,
  and 86 mm with Joint1 and Joint5 at 0.97. The bar's far end needs 62 mm to leave the channel and 83 mm to leave the
  mouth.
- So the draw is now 75 mm along the axis (`SLIDE_M`), then 20 mm at 25 degrees toward the mouth's front wall
  (`EXIT_M`, `EXIT_DEG`, `draw_points`' `exit`). That ends 10 mm past the lip. Joint1 and Joint5 may go to 0.97
  through it (`draw_soft`), and Joint5 reaches 0.968.
- Tilted any other way the arm gets only 8-14 mm. The key keeps 2.2 mm off the mouth at the exit's start (the
  chamfer, near the bar's front-bottom edge) and 10.5 mm at its end.
- The unfold's first leg (Joint1 turning with the arm folded) passed the v5 key's shaft at 2.7 mm. The elbow now opens
  20 degrees as Joint1 turns (`TURN_ELBOW_RAD`): 11.4 mm, the rest pose's own clearance. The dog's insertion and
  `holster_dog.py` turn the same way.

**The simulation** (`holster_env.py`, `run_holster.py`). The mouth is now colliders: its five walls as turned box
plates, three a wall, each cut to the opening's edge at its widest. After the hold comes the way back: `enter`, `back`,
`open`, `off`, `away`. An attempt counts as put back when the key is within 3 mm of holstered along the axis, and of
where it rested before the grasp across it. All runs were `--headless --no_overview --attempts 8` unless named
otherwise.
- [8 headless](#/week/6/run/20261024T082223_seed1_v5_return): every attempt stuck in the spawn's settle. Four
  debugging runs found why:
  - [the key parked away](#/week/6/run/20261024T082615_seed1_debug_parked): still stuck, so not the key;
  - [the script's clock](#/week/6/run/20261024T082859_seed1_debug_clock): resetting every 0.28 s;
  - [no mouth plates](#/week/6/run/20261024T083008_seed1_debug_noplates): the settle ends.

  The first plates (one full-width plate a wall, no chamfer) reached 10 mm into the D1 base's convex hull, as which
  PhysX collides the base. The holster's filter against the robot did not stop it, and the robot was thrown over
  every 14 steps. Two more debugging runs, the committed `holster_env.py` against v5, crashed before writing a record,
  on the pacing API that has changed since.
- The plates as they are now are at least 0.6 mm outside that hull and 0.25 mm off the key.
  [One attempt](#/week/6/run/20261024T084322_seed1_debug_plates5): drawn and put back.
- [8](#/week/6/run/20261024T084504_seed1_v5_return_err0mm), and with the holster
  [moved 3 mm](#/week/6/run/20261024T084614_seed1_v5_return_err3mm) and
  [5 mm](#/week/6/run/20261024T084726_seed1_v5_return_err5mm): drawn 8, 3 and 2, put back 0, and 4 and 5 fell.
  - A bug of mine: an edit dropped `open`'s case, so the jaws let go as `enter` began.
  - A moved holster pushes its mouth into the D1 base. So `--holster_error_mm` now moves the plan off the holster
    instead (`plan_key`), and the holster stays where it is.
- Fixed: [8](#/week/6/run/20261024T085011_seed1_v5_return_err0mm), all drawn (93.0 mm out along the axis at the
  hold's end) and **8 of 8 put back**. They ended 0.00-0.03 mm from holstered along the axis and 2.38-2.53 mm across,
  against 2.45-2.48 at rest. On the way in the key went up to 4.8 mm across, giving up to 7.0 mm in the grip's spring;
  torque 52-54% on the way back.
- [The plan 3 mm off](#/week/6/run/20261024T085220_seed1_v5_return_err3mm) (each axis within ±3 mm) and
  [5 mm off](#/week/6/run/20261024T085332_seed1_v5_return_err5mm): **8 of 8 drawn and 8 of 8 put back each**, no
  falls.
  - Grasp misses were up to 3.4 and 5.2 mm; the key went 92-94 mm out.
  - Back to 0.00-0.03 mm along the axis.
  - On the way in, the key was moved up to 5.6 and 6.2 mm across, giving up to 7.3 and 7.7 mm in the grip. That is
    the mouth taking it in.
  - Half the plans 3 mm or more off had the unfold's waypoint above `pre` miss by 1.1 mm, so the model's waypoint was
    used for them (`record["above_pre"]`).
- [On the display](#/week/6/run/20261024T085506_seed1_v5_return_display), `run_holster.py --run_name
  v5_return_display`: drawn (93.0 mm) and put back (0.01 mm), torque 67% at most, `draw.gif`.
- Torque reached its limit at some moment in 2 of 8 at 0 mm, 2 at 3 mm and 3 at 5 mm. That is the most over all the
  draw phases, as v4's runs had it ("at the limit in two"); which phase was not looked at.

**The dog's program** (`insert_plan.py`, `node.py`):
- *The draw*: v5's. `h_draw` (75 mm), `h_exit` (20 mm out through the mouth), `h_out` (held 1.5 s). Joint1 and
  Joint5 are at 0.97 out of the holster and back (`WIDE_PHASES`). `DOG_SLIDE_M` (v4's 100 mm) is gone.
- *Putting it back* (`RETURN_PHASES`), after the turn, with the key coming out still turned (Lukas: "Pull out at
  45 deg"):
  - `key_back`: out of the lock with the jaws shut;
  - `h_return`: a joint move to where the draw ended, checked against the trunk proxy and with the held key against
    the trunk and the lidar;
  - `h_enter`, `h_back`: the exit and the draw undone, into the mouth and down the channel;
  - `h_open`, `h_off`, `h_raise`, `h_unreach`;
  - then it stands and folds.

  `key_back`, `h_enter` and `h_back` abort on the arm's lag. An abort coming out of the lock lets the key go there. In
  the channel or after it, the jaws open and slide off the bar. Before the mouth, the key is handed back.
  `--leave_key` keeps the old ending, the key let go in the lock.
- Recorded: `check["drawn_mm"]`, `returned_mm` (the jaws along the axis from the grip at the end of `h_back`) and
  `reholstered`.
- Tests: `test_insert_robot` 46, `test_holster` 24 (2 skipped; with trimesh in the Isaac env the 5 they need pass),
  `test_approach` 2 and `test_robot`: all pass. New tests cover the return, `--leave_key`, aborts in `h_back`,
  `h_return` and `key_back`, the mouth's walls and the exit. `test_key_robot` fails 21 of its tests, the standing
  key turn's unfinished v4 stance (since 15:53, untouched today; `d1dogkey` is not to be run).

**Not shown:** the v5 holster on the dog, printed and mounted. The 6 mm grip offset (`HOLSTER_SHAFT_ALONG_M`) was
found on v4's holster, so v5's may differ. The real D1 against the mouth: its hand is stiffer than the simulation's
spring grip, which took the 5-7 mm across. The simulation's arm has no self-collision, so nothing here checked link
against link (the wrist against the shoulder ended the 18 mm deeper grip on the dog). The real key's rest in the holster
on the dog is not measured.

### 2026-10-24 · ft6 on the dog: no rattle (F-225)

The other PC ran ft6's export standing, under the same flags as round 4's rattling run:

    d1dogpress --policy checkpoints/leg_press_force_ft6_model_6998.npz --live --presses 0 --start_on_blend \
      --tilt_limit_deg 40 --start_tol_rad 0.30 --start_pitch_deg -15 5

Its analysis, both columns calculated the same way over the 10 s policy stand:

| | round 4 (rattled) | ft6 |
| --- | --- | --- |
| target change a step, FL thigh / RR thigh / RL calf | 0.229 / 0.160 / 0.118 rad | 0.012 / 0.006 / 0.012 rad |
| joint speed, the same three | 1.03 / 1.03 / 1.01 rad/s | 0.14 / 0.21 / 0.34 rad/s |
| most nose-up tilt | 20.8° | 16.6° |

- **The run completed normally:** the stand-up, 10 s on the policy, the lie-down and the handback to sport mode. A
  first attempt stopped at once ("the base is 0.070 m up: stand the dog in sport mode first"), because the dog was still
  lying, as it should.
- **The rattle is gone:** 15-25 times less step-to-step change than round 4's. The tilt stayed under simulation's
  21-22°.
- **A correction to the log above** ("The rattle on the dog reproduced in simulation") and to F-196: the dog's figures
  "after the latency cut" there (0.13 / 0.09 rad) were quoted from the other session's report and came from a
  different time window. Over the whole 10 s stand that run measures 0.23 / 0.16, at the 0.0002-armature row of that
  table. F-196 is marked so.
- **What it does not show:** presses. In simulation ft6 presses less accurately than round 4 (F-197). Next, ft6
  trained on, on the same settings ("ft7").

### 2026-10-24 · ft7: ft6 trained on; the press stalls at ~10 N of error on rima's settings

[ft7](#/week/6/run/20261023T133945_train_seed3_press_ft7_force_s3): ft6's final, 2000 more iterations on the same settings. Its training force error
stayed at 10.0-10.4 N throughout (round 4: ~4.5 N), and its damage share at 14-18%.

The dog battery (presses of 288 / fell / past 60 N; chatter as the FL / RR thigh's rms target change a step in the
10 s stand on armature 0.0002 at 15-20 ms):

| checkpoint | the hands, trained legs | the hands, armature 0.0002, 15-20 ms | the hands, armature 0, 5-15 ms | chatter | frozen set, classic stand |
| --- | --- | --- | --- | --- | --- |
| round 4's final | 280 / 0 / 5 | 10 / 83 / 166 | 0 / 62 / 228 | 0.27 / 0.22 rad | 196 / 0 / 5 |
| ft6 6998 (on the dog) | 208 / 0 / 59 | 36 / 0 / 0 | 90 / 0 / 29 | 0.005 / 0.002 | 24 / 21 / 33 |
| ft7 8000 | [85 / 77 / 100](#/week/6/run/20261023T145342_leg_eval_seed1_press_ft7_force_8000_hands) | [18 / 0 / 2](#/week/6/run/20261023T145428_leg_eval_seed1_press_ft7_force_8000_hands_light) | [57 / 0 / 39](#/week/6/run/20261023T145510_leg_eval_seed1_press_ft7_force_8000_hands_zero) | [0.005 / 0.003](#/week/6/run/20261023T145554_leg_eval_seed1_press_ft7_force_8000_stand10_light) | [32 / 7 / 38](#/week/6/run/20261023T145719_leg_eval_seed1_press_ft7_force_8000_real) |
| ft7 8997 | [255 / 0 / 18](#/week/6/run/20261023T144929_leg_eval_seed1_press_ft7_force_8997_hands) | [103 / 0 / 1](#/week/6/run/20261023T145012_leg_eval_seed1_press_ft7_force_8997_hands_light) | [101 / 0 / 30](#/week/6/run/20261023T145054_leg_eval_seed1_press_ft7_force_8997_hands_zero) | [0.006 / 0.003](#/week/6/run/20261023T145136_leg_eval_seed1_press_ft7_force_8997_stand10_light) | [22 / 31 / 61](#/week/6/run/20261023T145301_leg_eval_seed1_press_ft7_force_8997_real) |

- **Checkpoints swing widely.** 8000 fell 77 times on the dog's hands on the trained legs (66 on the palm in front);
  the final none. A run's last checkpoint is not safe by default.
- **On light legs the failures are the foot's placing,** not tipping: at 8997, 78 slipped off the palm, 40 never made
  contact, 66 were off force.
- **On the trained legs the frozen set is poor:** 22-32 presses and 7-31 falls. These policies do not carry over to
  heavier legs.
- **Next: what in rima's recipe holds the press back,** tested in parallel from ft7's final (round A, 800 iterations
  each). ft8a halves the joint-velocity noise (±0.75 rad/s). ft8b keeps rima's noise with this project's torque
  penalty (-5e-6).
  - A first launch of round A resumed from `model_8998.pt`, a name ft7 never saved (its final is 8997): it failed at
    once ([recorded](#/week/6/run/20261023T145806_train_seed3_press_ft8a_force_s3)).
  - `logs/unifp_train/press/round.sh` now takes a run directory and resolves its highest checkpoint itself.

### 2026-10-24 · Round A: neither the joint-velocity noise nor the torque penalty holds the press back

From ft7's final, 800 iterations each in parallel, rima's settings otherwise:
- [ft8a](#/week/6/run/20261023T145846_train_seed3_press_ft8a_force_s3): the joint-velocity noise halved
  (`--obs_noise_dof_vel 0.75`);
- [ft8b](#/week/6/run/20261023T145945_train_seed3_press_ft8b_force_s3): this project's torque penalty (-5e-6) instead of rima's -2e-4.

Training's force error stayed at 9.8-9.9 N (ft8a) and 10.0-10.8 (ft8b); ft7's was 9.7-10.7.

Presses of 288 / fell / past 60 N; chatter as rms rad a step:

| checkpoint | the hands, trained legs | the hands, armature 0.0002, 15-20 ms | the hands, armature 0, 5-15 ms | chatter, FL / RR thigh | frozen set, classic stand |
| --- | --- | --- | --- | --- | --- |
| round 4's final | 280 / 0 / 5 | 10 / 83 / 166 | 0 / 62 / 228 | 0.271 / 0.221 | 196 / 0 / 5 |
| ft7 8997 (from) | [255 / 0 / 18](#/week/6/run/20261023T144929_leg_eval_seed1_press_ft7_force_8997_hands) | [103 / 0 / 1](#/week/6/run/20261023T145012_leg_eval_seed1_press_ft7_force_8997_hands_light) | [101 / 0 / 30](#/week/6/run/20261023T145054_leg_eval_seed1_press_ft7_force_8997_hands_zero) | [0.006 / 0.003](#/week/6/run/20261023T145136_leg_eval_seed1_press_ft7_force_8997_stand10_light) | [22 / 31 / 61](#/week/6/run/20261023T145301_leg_eval_seed1_press_ft7_force_8997_real) |
| ft8a 9200 | [218 / 0 / 18](#/week/6/run/20261023T161127_leg_eval_seed1_press_ft8a_force_9200_hands) | [114 / 0 / 0](#/week/6/run/20261023T161209_leg_eval_seed1_press_ft8a_force_9200_hands_light) | [58 / 0 / 27](#/week/6/run/20261023T161251_leg_eval_seed1_press_ft8a_force_9200_hands_zero) | [0.006 / 0.003](#/week/6/run/20261023T161334_leg_eval_seed1_press_ft8a_force_9200_stand10_light) | [35 / 7 / 52](#/week/6/run/20261023T161417_leg_eval_seed1_press_ft8a_force_9200_real) |
| ft8a 9796 | [147 / 0 / 50](#/week/6/run/20261023T160746_leg_eval_seed1_press_ft8a_force_9796_hands) | [57 / 0 / 142](#/week/6/run/20261023T160831_leg_eval_seed1_press_ft8a_force_9796_hands_light) | [88 / 0 / 33](#/week/6/run/20261023T160918_leg_eval_seed1_press_ft8a_force_9796_hands_zero) | [0.012 / 0.005](#/week/6/run/20261023T161001_leg_eval_seed1_press_ft8a_force_9796_stand10_light) | [21 / 16 / 151](#/week/6/run/20261023T161044_leg_eval_seed1_press_ft8a_force_9796_real) |
| ft8b 9200 | [161 / 1 / 29](#/week/6/run/20261023T161832_leg_eval_seed1_press_ft8b_force_9200_hands) | [147 / 0 / 0](#/week/6/run/20261023T161915_leg_eval_seed1_press_ft8b_force_9200_hands_light) | [80 / 1 / 35](#/week/6/run/20261023T161957_leg_eval_seed1_press_ft8b_force_9200_hands_zero) | [0.004 / 0.004](#/week/6/run/20261023T162040_leg_eval_seed1_press_ft8b_force_9200_stand10_light) | [13 / 18 / 63](#/week/6/run/20261023T162122_leg_eval_seed1_press_ft8b_force_9200_real) |
| ft8b 9796 | [112 / 3 / 47](#/week/6/run/20261023T161459_leg_eval_seed1_press_ft8b_force_9796_hands) | [40 / 3 / 43](#/week/6/run/20261023T161542_leg_eval_seed1_press_ft8b_force_9796_hands_light) | [91 / 0 / 37](#/week/6/run/20261023T161625_leg_eval_seed1_press_ft8b_force_9796_hands_zero) | [0.004 / 0.003](#/week/6/run/20261023T161708_leg_eval_seed1_press_ft8b_force_9796_stand10_light) | [12 / 7 / 49](#/week/6/run/20261023T161751_leg_eval_seed1_press_ft8b_force_9796_real) |

- **No arm presses better than ft7's final**, and some are worse. ft8a's final strikes the hands past 60 N in 142 of
  288 on light legs.
- **The checkpoints keep swinging** by tens of presses from one to the next.
- **Next (round B), from ft7's final:** ft9a draws the armature 0-0.006 (knees ×3.67) and the delay 0-15 ms instead of
  exactly zero; ft9b keeps rima's legs with this project's `action_rate` (-0.02 instead of -0.1).

### 2026-10-24 · Round B: a little armature and delay cut the training error; the palm under the foot still fails on light legs

From ft7's final, 800 iterations each in parallel, rima's regularisation (observation noise, `action_rate` -0.1,
torques -2e-4):
- [ft9a](#/week/6/run/20261023T162242_train_seed3_press_ft9a_force_s3): leg armature drawn 0-0.006 (knees ×3.67), command delay 0-15 ms;
- [ft9b](#/week/6/run/20261023T162342_train_seed3_press_ft9b_force_s3): rima's legs (armature 0, no delay) with `action_rate` -0.02.

Training's force error: ft9a fell to 6.0 N (8.4 at its start); ft9b stayed at 9.6-11.3, as ft7's 9.7-10.7.

| checkpoint | the hands, trained legs | the hands, armature 0.0002, 15-20 ms | the hands, armature 0, 5-15 ms | chatter, FL / RR thigh | frozen set, classic stand |
| --- | --- | --- | --- | --- | --- |
| ft7 8997 (from) | [255 / 0 / 18](#/week/6/run/20261023T144929_leg_eval_seed1_press_ft7_force_8997_hands) | [103 / 0 / 1](#/week/6/run/20261023T145012_leg_eval_seed1_press_ft7_force_8997_hands_light) | [101 / 0 / 30](#/week/6/run/20261023T145054_leg_eval_seed1_press_ft7_force_8997_hands_zero) | [0.006 / 0.003](#/week/6/run/20261023T145136_leg_eval_seed1_press_ft7_force_8997_stand10_light) | [22 / 31 / 61](#/week/6/run/20261023T145301_leg_eval_seed1_press_ft7_force_8997_real) |
| ft9a 9200 | [251 / 1 / 36](#/week/6/run/20261023T173500_leg_eval_seed1_press_ft9a_force_9200_hands) | [65 / 0 / 0](#/week/6/run/20261023T173544_leg_eval_seed1_press_ft9a_force_9200_hands_light) | [72 / 0 / 37](#/week/6/run/20261023T173626_leg_eval_seed1_press_ft9a_force_9200_hands_zero) | [0.003 / 0.003](#/week/6/run/20261023T173708_leg_eval_seed1_press_ft9a_force_9200_stand10_light) | [64 / 4 / 49](#/week/6/run/20261023T173751_leg_eval_seed1_press_ft9a_force_9200_real) |
| ft9a 9796 | [216 / 2 / 45](#/week/6/run/20261023T173126_leg_eval_seed1_press_ft9a_force_9796_hands) | [96 / 0 / 3](#/week/6/run/20261023T173210_leg_eval_seed1_press_ft9a_force_9796_hands_light) | [66 / 0 / 112](#/week/6/run/20261023T173252_leg_eval_seed1_press_ft9a_force_9796_hands_zero) | [0.004 / 0.002](#/week/6/run/20261023T173337_leg_eval_seed1_press_ft9a_force_9796_stand10_light) | [159 / 2 / 34](#/week/6/run/20261023T173419_leg_eval_seed1_press_ft9a_force_9796_real) |
| ft9b 9200 | [213 / 0 / 21](#/week/6/run/20261023T174207_leg_eval_seed1_press_ft9b_force_9200_hands) | [78 / 0 / 0](#/week/6/run/20261023T174250_leg_eval_seed1_press_ft9b_force_9200_hands_light) | [78 / 0 / 17](#/week/6/run/20261023T174332_leg_eval_seed1_press_ft9b_force_9200_hands_zero) | [0.005 / 0.002](#/week/6/run/20261023T174414_leg_eval_seed1_press_ft9b_force_9200_stand10_light) | [39 / 19 / 43](#/week/6/run/20261023T174457_leg_eval_seed1_press_ft9b_force_9200_real) |
| ft9b 9796 | [191 / 0 / 58](#/week/6/run/20261023T173831_leg_eval_seed1_press_ft9b_force_9796_hands) | [55 / 2 / 175](#/week/6/run/20261023T173915_leg_eval_seed1_press_ft9b_force_9796_hands_light) | [78 / 0 / 68](#/week/6/run/20261023T173957_leg_eval_seed1_press_ft9b_force_9796_hands_zero) | [0.005 / 0.003](#/week/6/run/20261023T174041_leg_eval_seed1_press_ft9b_force_9796_stand10_light) | [34 / 25 / 121](#/week/6/run/20261023T174125_leg_eval_seed1_press_ft9b_force_9796_real) |

- **ft9a is the first rima-style fine-tune to hold up on heavier legs:** 159 on the frozen set from the classic stand
  (ft7: 22), with no rattle (0.004 / 0.002 rad).
- **On the dog's hands on light legs it is no better** (65-96). Its zero-armature hands strike past 60 N in 112 of 288.
- **`action_rate` is not what limits the press** (ft9b).
- **The palm under the foot is where the light-leg presses fail.** On armature 0.0002 at 15-20 ms, ft9a's final
  presses 0 of its 144 and the palm in front 96 of 144:
  - on the rigid palm the foot bounces and slides off (63 slipped, the force's spread 10 N);
  - on the sprung one it does not press at all (bias -5.0 N on 5 N);
  - on armature 0 it stomps (peaks ~135 N, 93 past the damage limit).
  - The palm under the foot is a low plate (its face 0.11 m up) facing straight up: about 7% of the training table
    faces up, and few of those are low.
- **Next (round C), from ft9a's final:** 30% of plate episodes on the dog's two hands at 3-10 N
  (`--press_hand_fraction`, new). ft10a keeps ft9a's 0-15 ms delay; ft10b draws 0-20 ms.

### 2026-10-24 · Round C: the dog's hands oversampled; the palm under the foot still fails on light legs, and why

From ft9a's final, 800 iterations each, on ft9a's recipe (armature 0-0.006) with 30% of plate episodes on the dog's two
hands at 3-10 N (`--press_hand_fraction 0.3`, new: `leg_env.press_hand_fraction`; no tangential part or slide on them):
- [ft10a](#/week/6/run/20261023T174721_train_seed3_press_ft10a_force_s3): command delay 0-15 ms;
- [ft10b](#/week/6/run/20261023T174819_train_seed3_press_ft10b_force_s3): command delay 0-20 ms.

Training's force error fell to 4.7-5.0 N; the hands' low levels are now part of that average, so it is not comparable
with the earlier rounds'.

| checkpoint | the hands, trained legs | the hands, armature 0.0002, 15-20 ms | the hands, armature 0, 5-15 ms | chatter, FL / RR thigh | frozen set, classic stand |
| --- | --- | --- | --- | --- | --- |
| ft9a 9796 (from) | [216 / 2 / 45](#/week/6/run/20261023T173126_leg_eval_seed1_press_ft9a_force_9796_hands) | [96 / 0 / 3](#/week/6/run/20261023T173210_leg_eval_seed1_press_ft9a_force_9796_hands_light) | [66 / 0 / 112](#/week/6/run/20261023T173252_leg_eval_seed1_press_ft9a_force_9796_hands_zero) | [0.004 / 0.002](#/week/6/run/20261023T173337_leg_eval_seed1_press_ft9a_force_9796_stand10_light) | [159 / 2 / 34](#/week/6/run/20261023T173419_leg_eval_seed1_press_ft9a_force_9796_real) |
| ft10a 10000 | [221 / 3 / 76](#/week/6/run/20261023T185931_leg_eval_seed1_press_ft10a_force_10000_hands) | [105 / 0 / 0](#/week/6/run/20261023T190015_leg_eval_seed1_press_ft10a_force_10000_hands_light) | [91 / 0 / 56](#/week/6/run/20261023T190057_leg_eval_seed1_press_ft10a_force_10000_hands_zero) | [0.004 / 0.002](#/week/6/run/20261023T190142_leg_eval_seed1_press_ft10a_force_10000_stand10_light) | [170 / 6 / 42](#/week/6/run/20261023T190224_leg_eval_seed1_press_ft10a_force_10000_real) |
| ft10a 10595 | [236 / 0 / 43](#/week/6/run/20261023T185555_leg_eval_seed1_press_ft10a_force_10595_hands) | [83 / 0 / 0](#/week/6/run/20261023T185638_leg_eval_seed1_press_ft10a_force_10595_hands_light) | [16 / 1 / 98](#/week/6/run/20261023T185720_leg_eval_seed1_press_ft10a_force_10595_hands_zero) | [0.002 / 0.002](#/week/6/run/20261023T185807_leg_eval_seed1_press_ft10a_force_10595_stand10_light) | [146 / 2 / 21](#/week/6/run/20261023T185850_leg_eval_seed1_press_ft10a_force_10595_real) |
| ft10b 10000 | [126 / 6 / 84](#/week/6/run/20261023T190642_leg_eval_seed1_press_ft10b_force_10000_hands) | [96 / 0 / 0](#/week/6/run/20261023T190724_leg_eval_seed1_press_ft10b_force_10000_hands_light) | [54 / 0 / 94](#/week/6/run/20261023T190807_leg_eval_seed1_press_ft10b_force_10000_hands_zero) | [0.003 / 0.002](#/week/6/run/20261023T190853_leg_eval_seed1_press_ft10b_force_10000_stand10_light) | [127 / 3 / 28](#/week/6/run/20261023T190936_leg_eval_seed1_press_ft10b_force_10000_real) |
| ft10b 10595 | [186 / 0 / 85](#/week/6/run/20261023T190305_leg_eval_seed1_press_ft10b_force_10595_hands) | [122 / 0 / 1](#/week/6/run/20261023T190350_leg_eval_seed1_press_ft10b_force_10595_hands_light) | [49 / 0 / 199](#/week/6/run/20261023T190431_leg_eval_seed1_press_ft10b_force_10595_hands_zero) | [0.003 / 0.002](#/week/6/run/20261023T190519_leg_eval_seed1_press_ft10b_force_10595_stand10_light) | [171 / 1 / 19](#/week/6/run/20261023T190601_leg_eval_seed1_press_ft10b_force_10595_real) |

- **The palm under the foot on light legs is unchanged:** 0-9 of 144, against the palm in front's 83-113.
- **On zero-armature legs the hands are struck past 60 N** in 56-199 of 288.
- **The frozen set from the classic stand holds** (146-171).
- **How it fails.** Traced on ft10b's final (palm under the foot, 5 N, armature 0.0002 at 15-20 ms):
  - **Soft palm:** the layer does what it should (the foot on the palm at 4.0 s, contact called by its stall at 4.5 s,
    5 N commanded), and the force on the palm is 0 N for the whole hold: the foot rests on it without pushing.
  - **Rigid palm:** it strikes at ~20 N, lifts off to 0 N and strikes again.
  - The force-aware policy pushes until its own estimate of the foot's force (decoded from its joint history) meets
    the command. Pressing straight down is what a supporting leg does anyway, and on light legs with a long delay
    that estimate seems to read a press that is not there. It works on the palm in front, and on every palm on the
    trained legs.
- **Next (round D), from ft10b's final:** the delay drawn 10-20 ms instead of from zero. ft11a draws the armature
  0-0.002 (the dog's corner, F-196); ft11b keeps 0-0.006.

### 2026-10-24 · The force estimate on light legs (F-226)

The trace of round C's light-leg palm presses (the foot resting on the palm at 0 N) suggested the policy's force estimate
reads a press. Checked on ft10b's final:
- the dog's hands rerun with the first 96 episodes' frames dumped (all on the palm under the foot):
  [trained legs](#/week/6/run/20261023T191658_leg_eval_seed1_press_ft10b_force_10595_hands_dump), [armature 0.0002 at 15-20 ms](#/week/6/run/20261023T191331_leg_eval_seed1_press_ft10b_force_10595_hands_light_dump);
- the decoder's foot force rebuilt from each step's 32-frame history with the export (`leg_policy.LegPolicy.estimate`).

Over the hold, medians over episodes, real / estimated:

| legs | rigid 5 N | rigid 8 N | sprung 5 N | sprung 8 N |
| --- | --- | --- | --- | --- |
| trained | 3.5 / 3.1 N | 9.0 / 8.1 | 3.9 / 3.5 | 9.1 / 7.9 |
| light | 0.0 / 3.2 | 0.2 / 7.4 | 0.0 / 2.9 | 0.0 / 4.3 |

On light legs the policy believes it presses, and so it does not. Round D's ft11a (the armature drawn only 0-0.002) is
the test of whether a narrower leg range lets the estimate get it right.

### 2026-10-24 · Round D: training on the dog's leg corner gives the best candidate yet

From ft10b's final, 800 iterations each, the command delay drawn 10-20 ms, 30% of plate episodes on the dog's hands:
- [ft11a](#/week/6/run/20261023T191133_train_seed3_press_ft11a_force_s3): leg armature 0-0.002 (knees ×3.67), the corner the dog's rattle pointed to
  (F-196);
- [ft11b](#/week/6/run/20261023T191231_train_seed3_press_ft11b_force_s3): armature 0-0.006.

| checkpoint | the hands, trained legs | the hands, armature 0.0002, 15-20 ms | the hands, armature 0, 5-15 ms | chatter, FL / RR thigh | frozen set, classic stand |
| --- | --- | --- | --- | --- | --- |
| ft10b 10595 (from) | [186 / 0 / 85](#/week/6/run/20261023T190305_leg_eval_seed1_press_ft10b_force_10595_hands) | [122 / 0 / 1](#/week/6/run/20261023T190350_leg_eval_seed1_press_ft10b_force_10595_hands_light) | [49 / 0 / 199](#/week/6/run/20261023T190431_leg_eval_seed1_press_ft10b_force_10595_hands_zero) | [0.003 / 0.002](#/week/6/run/20261023T190519_leg_eval_seed1_press_ft10b_force_10595_stand10_light) | [171 / 1 / 19](#/week/6/run/20261023T190601_leg_eval_seed1_press_ft10b_force_10595_real) |
| ft11a 10800 | [200 / 0 / 86](#/week/6/run/20261023T202602_leg_eval_seed1_press_ft11a_force_10800_hands) | [144 / 0 / 2](#/week/6/run/20261023T202648_leg_eval_seed1_press_ft11a_force_10800_hands_light) | [52 / 0 / 195](#/week/6/run/20261023T202730_leg_eval_seed1_press_ft11a_force_10800_hands_zero) | [0.003 / 0.002](#/week/6/run/20261023T202815_leg_eval_seed1_press_ft11a_force_10800_stand10_light) | [120 / 0 / 19](#/week/6/run/20261023T202857_leg_eval_seed1_press_ft11a_force_10800_real) |
| ft11a 11394 | [211 / 0 / 47](#/week/6/run/20261023T202230_leg_eval_seed1_press_ft11a_force_11394_hands) | [149 / 0 / 0](#/week/6/run/20261023T202315_leg_eval_seed1_press_ft11a_force_11394_hands_light) | [108 / 0 / 26](#/week/6/run/20261023T202357_leg_eval_seed1_press_ft11a_force_11394_hands_zero) | [0.003 / 0.002](#/week/6/run/20261023T202438_leg_eval_seed1_press_ft11a_force_11394_stand10_light) | [121 / 1 / 17](#/week/6/run/20261023T202521_leg_eval_seed1_press_ft11a_force_11394_real) |
| ft11b 10800 | [228 / 0 / 37](#/week/6/run/20261023T203314_leg_eval_seed1_press_ft11b_force_10800_hands) | [148 / 0 / 0](#/week/6/run/20261023T203358_leg_eval_seed1_press_ft11b_force_10800_hands_light) | [4 / 0 / 179](#/week/6/run/20261023T203439_leg_eval_seed1_press_ft11b_force_10800_hands_zero) | [0.002 / 0.002](#/week/6/run/20261023T203527_leg_eval_seed1_press_ft11b_force_10800_stand10_light) | [73 / 0 / 19](#/week/6/run/20261023T203609_leg_eval_seed1_press_ft11b_force_10800_real) |
| ft11b 11394 | [76 / 0 / 89](#/week/6/run/20261023T202938_leg_eval_seed1_press_ft11b_force_11394_hands) | [46 / 0 / 19](#/week/6/run/20261023T203023_leg_eval_seed1_press_ft11b_force_11394_hands_light) | [45 / 0 / 161](#/week/6/run/20261023T203106_leg_eval_seed1_press_ft11b_force_11394_hands_zero) | [0.002 / 0.002](#/week/6/run/20261023T203151_leg_eval_seed1_press_ft11b_force_11394_stand10_light) | [148 / 4 / 13](#/week/6/run/20261023T203233_leg_eval_seed1_press_ft11b_force_11394_real) |

- **ft11a's final is the best rima-style candidate on every leg model:**
  - the hands 211 / 149 / 108 on trained / light / zero legs, with no falls;
  - no rattle (0.003 / 0.002 rad);
  - the frozen set 121 from the classic stand;
  - level while it waits: 7.0° median (round 4: 25°; ft7: 16°).
- **ft11b swings between its checkpoints:** its zero-armature hands are struck past 60 N in 161-179.
- **The palm under the foot on light legs** rises to 27-30 of 144 (round C: 0-9). The palm in front is at 114-122.
- **The estimate, checked on ft11a's final** (light legs, the palm under the foot, frames dumped:
  [run](#/week/6/run/20261023T203928_leg_eval_seed1_press_ft11a_force_11394_hands_light_dump)): over the hold it now reads 1.8-3.3 N against 0-0.3 N real
  (ft10b's read 2.9-7.4). The rigid palm's failures are now slips (42 of 48); the sprung palm succeeds 19 of 48.
- **Next (round E), from ft11a's final on the same corner:** ft12a with half of all plate episodes on the hands, ft12b
  with 30% (a plain continuation); 1000 iterations each.

### 2026-10-24 · Round E: more of the leg corner, and no checkpoint as safe as ft11a's final

From ft11a's final, 1000 iterations each on the same leg corner (armature 0-0.002, delay 10-20 ms):
[ft12a](#/week/6/run/20261023T203736_train_seed3_press_ft12a_force_s3) with half of the plate episodes on the dog's hands, [ft12b](#/week/6/run/20261023T203835_train_seed3_press_ft12b_force_s3)
with 30%.

| checkpoint | the hands, trained legs | the hands, armature 0.0002, 15-20 ms | the hands, armature 0, 5-15 ms | chatter, FL / RR thigh | frozen set, classic stand |
| --- | --- | --- | --- | --- | --- |
| ft11a 11394 (from) | [211 / 0 / 47](#/week/6/run/20261023T202230_leg_eval_seed1_press_ft11a_force_11394_hands) | [149 / 0 / 0](#/week/6/run/20261023T202315_leg_eval_seed1_press_ft11a_force_11394_hands_light) | [108 / 0 / 26](#/week/6/run/20261023T202357_leg_eval_seed1_press_ft11a_force_11394_hands_zero) | [0.003 / 0.002](#/week/6/run/20261023T202438_leg_eval_seed1_press_ft11a_force_11394_stand10_light) | [121 / 1 / 17](#/week/6/run/20261023T202521_leg_eval_seed1_press_ft11a_force_11394_real) |
| ft12a 11800 | [268 / 0 / 18](#/week/6/run/20261023T220756_leg_eval_seed1_press_ft12a_force_11800_hands) | [144 / 0 / 97](#/week/6/run/20261023T220838_leg_eval_seed1_press_ft12a_force_11800_hands_light) | [75 / 0 / 141](#/week/6/run/20261023T220920_leg_eval_seed1_press_ft12a_force_11800_hands_zero) | [0.005 / 0.002](#/week/6/run/20261023T221004_leg_eval_seed1_press_ft12a_force_11800_stand10_light) | [146 / 0 / 13](#/week/6/run/20261023T221046_leg_eval_seed1_press_ft12a_force_11800_real) |
| ft12a 12393 | [264 / 2 / 13](#/week/6/run/20261023T220424_leg_eval_seed1_press_ft12a_force_12393_hands) | [90 / 47 / 5](#/week/6/run/20261023T220507_leg_eval_seed1_press_ft12a_force_12393_hands_light) | [38 / 0 / 23](#/week/6/run/20261023T220550_leg_eval_seed1_press_ft12a_force_12393_hands_zero) | [0.005 / 0.003](#/week/6/run/20261023T220631_leg_eval_seed1_press_ft12a_force_12393_stand10_light) | [186 / 0 / 13](#/week/6/run/20261023T220714_leg_eval_seed1_press_ft12a_force_12393_real) |
| ft12b 11800 | [207 / 0 / 41](#/week/6/run/20261023T221458_leg_eval_seed1_press_ft12b_force_11800_hands) | [93 / 0 / 4](#/week/6/run/20261023T221541_leg_eval_seed1_press_ft12b_force_11800_hands_light) | [158 / 0 / 42](#/week/6/run/20261023T221623_leg_eval_seed1_press_ft12b_force_11800_hands_zero) | [0.003 / 0.001](#/week/6/run/20261023T221705_leg_eval_seed1_press_ft12b_force_11800_stand10_light) | [98 / 0 / 16](#/week/6/run/20261023T221748_leg_eval_seed1_press_ft12b_force_11800_real) |
| ft12b 12393 | [139 / 0 / 36](#/week/6/run/20261023T221128_leg_eval_seed1_press_ft12b_force_12393_hands) | [136 / 0 / 82](#/week/6/run/20261023T221211_leg_eval_seed1_press_ft12b_force_12393_hands_light) | [141 / 0 / 118](#/week/6/run/20261023T221252_leg_eval_seed1_press_ft12b_force_12393_hands_zero) | [0.003 / 0.002](#/week/6/run/20261023T221335_leg_eval_seed1_press_ft12b_force_12393_stand10_light) | [183 / 0 / 5](#/week/6/run/20261023T221417_leg_eval_seed1_press_ft12b_force_12393_real) |

- **The best numbers come with unsafe ones.**
  - ft12a's 11800 presses the hands 268 on the trained legs, near round 4's 280, but strikes them past 60 N in 97 and
    141 on light and zero-armature legs.
  - ft12a's final falls 47 times on light legs.
  - ft12b's final presses the frozen set 183 from the classic stand, and strikes the zero-armature hands in 118.
- **The palm under the foot on light legs** stays at 0-25 of 144.
- **No checkpoint is as safe on all three leg models as ft11a's final** (no falls, 0-47 past 60 N), and the
  checkpoints keep swinging.
- **Next:** ft11a's final polished at a lower learning rate (3e-5, 600 iterations: ft13), and the dog's hands on six
  checkpoints of rounds D and E not yet evaluated. Then a choice by safety first: no falls on any leg model, the fewest
  presses past 60 N, then the most presses on the light and zero-armature legs.

### 2026-10-24 · The polish, a checkpoint sweep, and the candidate: ft11a's final (F-227)

[ft13](#/week/6/run/20261023T221912_train_seed3_press_ft13_force_s3) polished ft11a's final at learning rate 3e-5 for 600 iterations on the same leg corner.
Six checkpoints of rounds D and E not yet evaluated got the dog's hands on all three leg models. Presses of 288 /
fell / past 60 N:

| checkpoint | the hands, trained legs | the hands, armature 0.0002, 15-20 ms | the hands, armature 0, 5-15 ms | chatter, FL / RR thigh | frozen set, classic stand |
| --- | --- | --- | --- | --- | --- |
| round 4's final | 280 / 0 / 5 | 10 / 83 / 166 | 0 / 62 / 228 | 0.271 / 0.221 | 196 / 0 / 5 |
| ft11a 11000 | [131 / 0 / 28](#/week/6/run/20261023T221912_leg_eval_seed1_press_ft11a_force_11000_hands) | [134 / 0 / 0](#/week/6/run/20261023T222102_leg_eval_seed1_press_ft11a_force_11000_hands_light) | [45 / 0 / 54](#/week/6/run/20261023T222244_leg_eval_seed1_press_ft11a_force_11000_hands_zero) | - | - |
| ft11a 11200 | [83 / 0 / 29](#/week/6/run/20261023T222433_leg_eval_seed1_press_ft11a_force_11200_hands) | [67 / 0 / 60](#/week/6/run/20261023T222630_leg_eval_seed1_press_ft11a_force_11200_hands_light) | [97 / 1 / 92](#/week/6/run/20261023T222816_leg_eval_seed1_press_ft11a_force_11200_hands_zero) | - | - |
| ft11a 11394 | [211 / 0 / 47](#/week/6/run/20261023T202230_leg_eval_seed1_press_ft11a_force_11394_hands) | [149 / 0 / 0](#/week/6/run/20261023T202315_leg_eval_seed1_press_ft11a_force_11394_hands_light) | [108 / 0 / 26](#/week/6/run/20261023T202357_leg_eval_seed1_press_ft11a_force_11394_hands_zero) | [0.003 / 0.002](#/week/6/run/20261023T202438_leg_eval_seed1_press_ft11a_force_11394_stand10_light) | [121 / 1 / 17](#/week/6/run/20261023T202521_leg_eval_seed1_press_ft11a_force_11394_real) |
| ft12a 12000 | [97 / 51 / 10](#/week/6/run/20261023T223004_leg_eval_seed1_press_ft12a_force_12000_hands) | [158 / 3 / 62](#/week/6/run/20261023T223154_leg_eval_seed1_press_ft12a_force_12000_hands_light) | [138 / 2 / 26](#/week/6/run/20261023T223341_leg_eval_seed1_press_ft12a_force_12000_hands_zero) | - | - |
| ft12a 12200 | [246 / 5 / 9](#/week/6/run/20261023T223527_leg_eval_seed1_press_ft12a_force_12200_hands) | [86 / 116 / 5](#/week/6/run/20261023T223711_leg_eval_seed1_press_ft12a_force_12200_hands_light) | [161 / 7 / 8](#/week/6/run/20261023T223900_leg_eval_seed1_press_ft12a_force_12200_hands_zero) | - | - |
| ft12b 12000 | [200 / 0 / 7](#/week/6/run/20261023T224045_leg_eval_seed1_press_ft12b_force_12000_hands) | [168 / 0 / 12](#/week/6/run/20261023T224229_leg_eval_seed1_press_ft12b_force_12000_hands_light) | [113 / 0 / 171](#/week/6/run/20261023T224415_leg_eval_seed1_press_ft12b_force_12000_hands_zero) | - | - |
| ft12b 12200 | [153 / 0 / 56](#/week/6/run/20261023T224607_leg_eval_seed1_press_ft12b_force_12200_hands) | [120 / 0 / 68](#/week/6/run/20261023T224754_leg_eval_seed1_press_ft12b_force_12200_hands_light) | [121 / 0 / 126](#/week/6/run/20261023T224949_leg_eval_seed1_press_ft12b_force_12200_hands_zero) | - | - |
| ft13 11600 | [214 / 0 / 5](#/week/6/run/20261023T225510_leg_eval_seed1_press_ft13_force_11600_hands) | [31 / 0 / 12](#/week/6/run/20261023T225553_leg_eval_seed1_press_ft13_force_11600_hands_light) | [86 / 0 / 4](#/week/6/run/20261023T225635_leg_eval_seed1_press_ft13_force_11600_hands_zero) | - | - |
| ft13 11800 | [261 / 0 / 0](#/week/6/run/20261023T225718_leg_eval_seed1_press_ft13_force_11800_hands) | [4 / 0 / 8](#/week/6/run/20261023T225759_leg_eval_seed1_press_ft13_force_11800_hands_light) | [69 / 0 / 125](#/week/6/run/20261023T225841_leg_eval_seed1_press_ft13_force_11800_hands_zero) | - | - |
| ft13 11993 | [268 / 0 / 1](#/week/6/run/20261023T225924_leg_eval_seed1_press_ft13_force_11993_hands) | [54 / 1 / 2](#/week/6/run/20261023T230005_leg_eval_seed1_press_ft13_force_11993_hands_light) | [104 / 0 / 104](#/week/6/run/20261023T230047_leg_eval_seed1_press_ft13_force_11993_hands_zero) | - | - |

- **The checkpoints swing as before;** the polish did not steady them (ft13's light-leg hands 4-54).
- **The choice: ft11a's final.** By the stated rule (no falls, then the fewest presses past 60 N, then the most on light
  and zero-armature legs) ft13's 11600 would win: 21 past 60 N over the three hand sets. But it presses the hands on
  light legs 31 of 288.
  - ft11a's final has no falls, 73 past 60 N (47 on the trained legs, 0 on light, 26 on zero) and 257 light and
    zero-armature presses: 52 more strikes for 140 more presses on the dog's likely legs.
- **The rest of its battery:**
  - the 10 s stand ([trained legs](#/week/6/run/20261023T230245_leg_eval_seed1_press_ft11a_force_11394_stand10),
    [armature 0](#/week/6/run/20261023T230327_leg_eval_seed1_press_ft11a_force_11394_stand10_zero), light legs above): 0.002-0.007 rad a step, no falls in
    any stand run, waiting 11.9-13.7° median (light legs at most 12.7°; armature 0 past 15° in 51 of 288, at most 23°);
  - the frozen set from the dog's rest on light legs: [49 / 0 / 94](#/week/6/run/20261023T230409_leg_eval_seed1_press_ft11a_force_11394_dogrest_light)
    (round 4: 5 / 171 / 85);
  - the hands at armature 0.003 with 15-20 ms: [111 / 0 / 0](#/week/6/run/20261023T230452_leg_eval_seed1_press_ft11a_force_11394_hands_mid);
  - the hands on light legs with friction 1 N·m: [28 / 0 / 4](#/week/6/run/20261023T230534_leg_eval_seed1_press_ft11a_force_11394_hands_light_fric1),
    under-pressed (bias -5 N).
- **Exported:** `checkpoints/leg_press_force_ft11a_model_11394.npz` (sha256 1ae43769...; numpy to 6.2e-6). Its
  `start_gate` is round 4's stands' (pitch -3.8 to +1.0°).
- **For the dog, in order:** the 10 s stand; then a press on the palm in front (`--plate palm_facing`), its strong
  press. The palm under the foot (`palm_up`) is its weak one on light legs (27 of 144) because of F-226.

### 2026-10-25 · v5's draw on the dog: out, but the lever's end bumped the mouth; a loop on the draw and a lift after it

[The first live run with the v5 holster](#/week/6/run/20261024T215423_insert_insert_live) (2026-10-24 21:54 UTC,
`d1doginsert --live --yes --step`; Lukas stopped it in `key_go`; the key was handed back). Lukas: "the unsheathe gets
stuck on the way out. it seems this is because the movement after the sliding out makes it collide with the mouth of
the holster", then "it still got out, but bumped the key and messed up the grip".

**The trace**, the encoders' jaw centre against the draw's plan, along the holster's axis and across it:
- *through `h_draw`*: the arm up to 18 degrees behind on its fastest joint, and 4-5 mm off the axis in `t` all through
  (the arm's sag).
- *the end of the 1.5 s `h_out`*: 88.7 mm out of the planned 93.1, and 4.2 mm off the axis. With the grip 6 mm along
  the bar (`HOLSTER_SHAFT_ALONG_M`), that puts the lever's far end at the mouth's lip, not 10 mm past it.
- *`key_go`*: its joint move began by taking the hand 7 mm back along the axis (88.7 to 81 mm) as it went across. That
  is the bump. The CPU model has the same move 7.7 mm clear of the holster with the model's grip, and 1.7 mm with the
  dog's.

**Changed** (`insert_plan.py`, `holster.py`):
- *the draw's own outer loop* (`DRAW_LOOP_PHASES`, `DRAW_LOOP_GAIN` 0.8/s, capped at 15 mm). The encoders' miss is
  integrated across the axis through `h_draw` and all of it through `h_out`, now 3 s. The return starts from its value.
  - In the dry tests, with the shoulder drooping 1.5 degrees through the draw, the jaws end `h_out` within 0.8 mm of
    the exit, against 3.0-3.6 mm without the loop.
  - At the insertion's gain, 1.5, the arm's lag made it overshoot: +1.5 to -1.3 mm.
- *a lift, `h_clear`*: 40 mm away from the mouth before `key_go` (`holster.CLEAR_M`, `clear_direction`: 20 degrees
  out along the axis, from across it 60 degrees round from the open front toward the bar's top). On the CPU model 9 of
  36 directions reach 40 mm. With the dog's grip this one keeps the key 5.5 mm or more off the holster and 8.5 off the
  D1's base. Straight toward the front it passes 0.5 mm off the base, and `holster.check` failed on that.
- *the return* now goes back to the lift's joints (`h_return`), then down to the exit (`h_unclear`), then into the
  mouth. The loop's value is carried over and integrated across the axis on the way in.
- Tests: `test_insert_robot` 47 (new: the loop against a sagging arm, the lift), `test_holster` 24 (2 skipped), and
  `test_robot`, all pass.

**Not shown:** the loop and the lift on the dog. What the sag is from (the arm's servos under the lever's 42 g, or
the IK's model) is not known. The loop corrects the encoders' miss, not the arm's real position, and the 6 mm grip
offset says those differ by about that much along the bar.

### 2026-10-25 · The whole insertion from the holster and back on the dog; the fold over the lidar, the return's abort, and a cleaner grip

Three live runs with the loop and the lift (`d1doginsert --live --yes --step`):
- [22:16 UTC](#/week/6/run/20261024T221630_insert_insert_live): drawn 92.5 mm of 93.1 (the loop; 88.7 before it). Lukas stopped it after the
  touch.
- [22:19 UTC](#/week/6/run/20261024T221928_insert_insert_live): **the whole sequence**. Drawn 92.9 mm; seated straight in (0.5 mm); turned
  40.6 degrees of 45; out of the lock and back along a path 127.5 mm clear for the key; put back, the jaws 3.7 mm
  from the grip along the axis at the end of `h_back`; folded. Lukas: "the arm sometimes collides with the lidar on
  its way back because it drags itself over the head while pointing down".
  - The trace: after `h_unreach` the arm was at the turned fold. `stand_up` and `stand_settle` then took it to the
    carry pose in front of the dog (their target, left from the hand-in's way: servo
    [11, 1, 41, -61, 17, 3] at the fold's start).
  - The fold from there swept the fingers through the lidar's box: the meshes 1 mm inside it at 118.8 s, the link
    chain's proxy 33 mm inside. Every other phase kept 41 mm or more.
- [22:24 UTC](#/week/6/run/20261024T222452_insert_insert_live): aborted in `h_back`, "the arm is 11 deg from where the IK wants it: loaded".
  Joints 1, 4, 5 and 6 were 10-24 degrees behind a moving target, the key going in: the arm's lag, as through
  `h_draw`, not a load. Lukas: "can you make it not abort when returning". The recovery opened the jaws there and slid
  off.

Lukas's photo of the grip: the fingers past the bar's end, holding its last few millimetres, as the 6 mm the jaws
close short of the shaft would put them ("It still works but it would be good to grab it more cleanly").

**Changed** (`insert_plan.py`, `holster.py`):
- *the grip*: the dog's draw takes the bar 6 mm further along (`DOG_GRIP_ALONG_M`, `draw_plan(grip_along_m=)`). The
  insertion's shaft offset is now 0 (`HOLSTER_SHAFT_ALONG_M`), the shaft on the jaws' axis as the Joint6-alone turn
  takes it. On the CPU model the draw solves 6 and 10 mm along, every joint within 0.920 and 0.887 of its range
  (0.968 at the shaft), the unfold 11.4 mm clear, the key 8.7 off the D1's base.
- *the fold*: after `h_unreach`, `h_unturn` (Joint1 home with the elbow still open) and `h_fold` (the elbow shut),
  then the stand and the last fold with the arm held folded. On the CPU model the hand keeps 63.6 mm off the
  lidar's boxes and 20.2 mm off the holstered key (straight to the fold: 11.5). An abort in them finishes the fold
  and stands.
- *the way back*: `h_enter` and `h_back` no longer abort on the arm's lag (`key_back`, out of the lock, still does).
  `h_back` takes 8 s, not 5. The draw's loop is not integrated on the way back, only its value from the draw's end
  applied.
- Tests: `test_insert_robot` 50 (new: the fold and the stand held folded, a 15-degree lag in `h_enter` and `h_back`
  not aborting, the grip 6 mm along), `test_holster` 24 (2 skipped), `test_robot`: all pass.

**Not shown:** the deeper grip, the fold and the slower `h_back` on the dog. With no lag abort on the way back,
nothing stops the arm if the key jams going in; the operator's B or `a` + Enter does. The hand-in's way (no holster)
still stands with the arm at the carry pose and folds from there.

### 2026-10-25 · The insertion chained into the door pull; the grip 3.5 mm further onto the bar

Lukas: "can you now chain this into the door handle pulling?" `node.py --task insert --stage insert --then_door pull`
(`d1doginsert --then_door pull`) does what `--task key --stage turn --then_door pull` does. The door pull follows from
where the insertion left the dog (`key_plan.DoorAfterKey`): standing, the arm folded, the key back in the holster. The
dog walks to the door pull's stance with the arm folded, the lidar refits, the arm unfolds and pulls. In the dry tests
that stance is 0.01 m behind the insertion's, 0.35 m to its right and turned 4.5 degrees. It goes only if
`InsertAttempt.unlock_check()` says so, from the robot's own measures:
- the key seated;
- the encoders' turn at least the lock's 30 degrees (`geometry.ARM_LEVER.unlock_deg`);
- the jaws empty: the key put back, or let go in the lock.

Otherwise the run ends with why (`run["insert"]`).

Then Lukas, of the grip 6 mm along: "it now grabs it like this. There is a 3.5mm gap to where the end of the pincers
are supposed to be". The fingertips stop 3.5 mm short of the bar's back face. The dog's draw now closes the jaws that
much further onto the bar along the approach (`DOG_GRIP_ONTO_M`, `draw_plan(grip_onto_m=)`). On the CPU model the draw
still solves, every joint within 0.944 of its range and the open hand 29.6 mm off the key. The insertion's model of
the key in the jaws is unchanged: the fingertips at the back face is what it has always assumed. So the key the dog
has inserted until now sat 3.5 mm further out of the jaws than modelled, which the touch's depth absorbed.

Tests: `test_insert_robot` 53 (new: the door pull after the key is put back, walking over folded, refitting and
pulling; none after a turn under 30 degrees; the grip's two offsets), `test_holster` 24 (2 skipped), `test_robot`:
all pass. **Not shown:** any of it on the dog.

### 2026-10-25 · The door pull after the insertion: aborted on the dog; now it backs up and looks again

Lukas ran the chain on the dog: `d1doginsert --live --yes --step --then_door pull` (the node's argv: `--task insert
--stage insert --live --yes --step --then_door pull`, front camera).
[The run](#/week/6/run/20261024T225859_insert_insert_door_pull_live) (F-209):
- **The insertion worked.** The key was drawn 93.4 mm, inserted, turned 49.3 degrees by the encoders (45 commanded)
  and put back, and `unlock_check` verified it.
- **The door pull aborted after its walk over** (`DoorAfterKey`, walking from the insertion's fix with no new look).
  The walk started 0.32 m left of the door pull's stance and one step got within 2.1 cm of it by the odometry, the box
  0.675 m off. The refit then moved the box 32.7 mm and turned it 5.9 degrees, which put it 0.65 m off at -3.3
  degrees, the door +2.7 degrees off square. That is outside the 0.67-0.71 m range: "move the dog back 0.05 m".
  The base stood and the arm held, folded.

Lukas's screenshot of RViz at the abort shows the status line above and the front camera's view of the tag, the dog
standing back from the box. Lukas: "I believe this is partly because it didnt use classic walk, the more accurate
walking model, but I also want to make it robust to the fact that the door swings open by almost 10 degrees so it
should back up to what is shown in the screenshot so it sees the qrcode and then commence the door pull like usual so
it rescans for the door first rather than relying on its prior knowledge".

**On the gait:** the code bears that out. `node.py` sent classic walk once, as the run started. The insertion then
lies the dog down and stands it up (StandDown at 12.4 s, StandUp at 116.0 s, BalanceStand at 118.4 s), which leaves
sport mode in its default gait, and nothing sent classic walk again before the walk over. The record does not log
sport mode's gait, so which gait the dog walked in is inferred from the code, not measured.

**The change** (`node.py` `_then_door`, `approach.door_after_key`):
- Once the key's check passes, the run becomes the door pull (`node.task`, `node.stage`), and nothing is carried over
  from the insertion's fix but where to look from.
- The dog stands 1.5 s (`approach.GAIT_SETTLE_S`, a `pause` phase) while classic walk is sent again. Then, with the arm
  folded, it backs up with the approach's walk to the door pull's staging pose: square to the door as the insertion
  found it, the box's root 1.05 m ahead, where the front camera sees the tag.
- There the node looks again, and the door pull goes on as from any look: its range check, an approach if out of
  range, its unfold, its own straight walk in and refit, then the pull.
- `key_plan.DoorAfterKey` and `door_fix` are removed. The key's `--then_door` goes the same way.

**In the dry tests** (`test_insert_robot`, the door turned open about its hinge once the key has turned):
- **0 and 5 degrees open:** one walk back, then the look is in range (walk in 0.28-0.30 m). The pull completes, the
  refit leaving the box 0.67-0.68 m off.
- **10 and 15 degrees open:** the first look sees the door 11.8 and 16.9 degrees off square, past the range check's 8,
  and asks for a turn. The node's approach walks again, square to the door as it now is, and the second look is in
  range (door 0.9 degrees off). The pull completes, the box 0.67 m off after the refit.

Tests: `test_insert_robot`, `test_approach` and `test_robot`, 105 in all, pass. `test_key_robot`'s `--then_door` test
was rewritten the same way, but it cannot pass yet: it, and most of that file's attempts, abort at the key's own
unfold with "no arm pose in front of the key from which the whole turn solves", before any of the chain runs. That
comes with the uncommitted v4-lever change, not this one.

**Not shown:**
- Any of it on the dog.
- That classic walk takes once sent after BalanceStand, and whether the 1.5 s stand is needed.
- How the lidar's door face reads on a door ajar, where the box's side may show beside it. The dry look sees only the
  door.
- The second walk an ajar door asks for costs one more look; it is not avoided.
- The door pull's own walk in still ends at 0.67 m in the dry runs, the near edge of its range: on the dog it ended
  at 0.65.

### 2026-10-25 · Three whole insertions in a row; the key lined up with the mouth before it goes back in

[22:29 UTC](#/week/6/run/20261024T222956_insert_insert_live), [22:39](#/week/6/run/20261024T223907_insert_insert_live) and [22:53](#/week/6/run/20261024T225323_insert_insert_live)
(`d1doginsert --live --yes --step`, the fold before the stand, the grip 6 mm along): **all three finished the whole
sequence**.

| run | drawn | seated, out of seated | turned (by the encoders) | jaws from the grip when put back |
| --- | --- | --- | --- | --- |
| 22:29 | 92.6 mm | 0.07 mm | 51.3 deg | 3.1 mm |
| 22:39 | 92.6 mm | 2.2 mm | 47.4 deg | 3.6 mm |
| 22:53 | 92.9 mm | −1.35 mm | 49.3 deg | 2.0 mm |

All drawn of the plan's 93.1 mm along the axis; the turn's plan was 45 deg. The 3.5 mm grip onto the bar came after
them.

Lukas: "when returning the key, theres a very close call where it begins contact with the upper angled wall of the
mouth before sliding in. this seems to be consistent". In the 22:53 trace:
- `h_enter` began with the arm still coming down from the lift: the jaws 8.5 mm toward the bar's top side (`t`) at
  `h_unclear`'s end.
- With only the draw's end value of the loop (3.2 mm in `t`) applied, the jaws reached the channel's start (76 mm out)
  3.6 mm toward its top wall. The walls are 0.4 mm off the bar there.
- Down the channel they ran within 1-2 mm of the axis.

**Changed** (`insert_plan.py`):
- `h_align`: 2 s at the exit before going in, the loop integrating all of the miss as through `h_out`.
- `h_enter`: 4 s, not 2.
- The loop now integrates across the axis through `h_enter` and `h_back`, as through `h_draw`.
- In the dry tests, with the shoulder drooping 1.5 degrees: the jaws within 0.03 mm of the exit after `h_align`,
  within 0.8 mm of the way into the mouth, and within 1.6 mm of the channel's axis down it.
- Tests: `test_insert_robot` 54, `test_holster` 24 (2 skipped), `test_robot`, `test_approach`: all pass.

**Not shown:** on the dog. A first run of the suite failed on the change to the door pull's chaining above, mid-edit
(`approach.door_after_key`); it passed once that edit was in.

### 2026-10-25 · The grip onto the bar moved the key's seat 3.5 mm: the lock model 3.5 mm deeper

[`d1doginsert --live --yes --step --then_door pull`](#/week/6/run/20261024T232734_insert_insert_door_pull_live)
(23:27 UTC, the first run with the grip 3.5 mm onto the bar). Lukas: "it kept wiggling back and forth for a minute
and didnt begin turning the lock". He stopped it; the recovery handed the key back, and there was no door pull ("the
key did not seat").
- *the trace*: drawn 92.9 mm; the bezel seen (4.1, -1.9 mm off the model). `key_in` crawled in from 25 mm to -4.61 mm
  out of the model's seated, and stopped there.
- That is past the half keyway (-4.35) that is taken as seated, so it was taken for a key on the face. The search's
  spiral then pressed at it for 47 s, reaching 3.0 mm, its ±10° wiggle with it.
- *why*: with the fingertips 3.5 mm short of the bar's back face (every earlier run), the key sat 3.5 mm further out of
  the jaws than the insertion's model of it. Seven of those runs (05:54-22:53 UTC) seated straight in at -1.4 to +2.2
  mm. With the jaws at the back face the key is where the model has it, so the lock itself is ~3.5 mm further in.
  This run's stop is -1.1 mm on that.

**Changed:** `insert_plan.LOCK_DEEPER_M` = 3.5 mm (it was 0; `--key_in_mm` still nudges it). Tests: all pass
(`test_insert_robot` 54, `test_holster`, `test_robot`, `test_approach`).

**Not shown:** on the dog. A key stopped well past the model's seated is still taken for one on the face, and the
search then runs to its 4 mm. A stop that deep is more likely seated, but that is not changed here.


### 2026-10-25 · Aborted after the walk up at 4.0 degrees off square; a tighter walk-up turn and one walk up again

[`d1doginsert --live --yes --step --then_door pull`](#/week/6/run/20261024T234121_insert_insert_door_pull_live)
(23:41 UTC) aborted before lying down: "after the walk in: the seated key 0.45 m ahead, +0.06 m right, door -4.0 deg
off square; move the dog turn right 4 deg". Lukas: "why did this happen? its never done it before?"
- *the look*: from 9 degrees off square (his other runs, 1-3).
- *the walk up*: stopped 2.0 degrees off, which the key's walk-up finish accepts (`WALK_UP_ACCEPT_YAW_DEG`, 4).
- *the lidar's refit*: turned the door 2.2 degrees further than the look had it (a look from further round is less
  sure of the door's angle). Together that came to 4.0 against the range check's 4 (`key_plan.YAW_LIMIT_DEG`), and
  the program aborted rather than turn.

**Changed** (`insert_plan.py`; a hook in `plan.Attempt`, `_out_of_range_after_walk`, which still aborts for the door
pull and the key turn):
- the insertion's walk up now finishes within 2 degrees (`INSERT_ACCEPT_YAW_DEG`), not 4;
- when the refit then puts the stance out of range, it walks up once more (`INSERT_REWALKS`). The stance is taken
  from the refitted box, the arm folded; it settles and refits again. A second miss aborts. An event, `rewalk`,
  records why.
- Tests: a look that sees the door 5 degrees off, with the range check tightened to 2.5 degrees (the harness's walk up
  ends ~1.8 degrees toward the true door, so its refit leaves ~3), walks up twice and lies down; with no walk up
  again, it aborts. All pass (`test_insert_robot` 56, `test_holster`, `test_robot`, `test_approach`).

**Not shown:** on the dog. A 2-degree finish asks sport mode for small turns near its dead zone. The walk up may then
take its corrective steps (`WALK_UP_MAX_STEPS`) and stop where it is, for the refit's range check to decide.

### 2026-10-25 · The door pull's fold onto the lidar; the dog further from the door

Lukas ran the insertion's chain into the door pull five more times (`d1doginsert --live --yes --step --then_door pull`,
the walk back and new look of the entry above). In three the key never turned, so there was no door pull:
- [23:24 UTC](#/week/6/run/20261024T232436_insert_insert_door_pull_live): the key stuck 2.3 mm short of seated.
- [23:27](#/week/6/run/20261024T232734_insert_insert_door_pull_live) and [23:41](#/week/6/run/20261024T234121_insert_insert_door_pull_live):
  in the two entries above.

**In the other two the chain went through** (F-210): [23:43](#/week/6/run/20261024T234356_insert_insert_door_pull_live) and [23:52](#/week/6/run/20261024T235219_insert_insert_door_pull_live).
- The walk back was 0.52 and 0.56 m, and the look there was in range at once.
- The door pull's walk in and refit left the box 0.68 and 0.67 m off.
- The door was pulled 60 degrees and let go, and the arm folded.

Lukas: "at the end of the door pull, when the arm is folding back up, it hits the lidar on the head of the dog. can you
prevent this from happening? the insert task had a similar problem and fixed it".

**The fold** (F-211): in both traces every joint but the shoulder reached the fold first, and the shoulder stopped
~0.5 s at -50 and -40 degrees with the elbow already shut, then went on. The arm link's path paces each joint on its
own, so the shortest moves end first. The CPU model has the fingertips' segment 29 mm into the lidar cage's box there.
The insertion had met the same thing and folds the elbow last (`h_unturn`, `h_fold`). The door pull now does too:
- `tuck`: to the fold with the elbow 30 degrees open.
- `fold`: the elbow shut (`plan.fold_phases`, `TUCK_ELBOW_DEG`). The recoveries fold the same way.

On the CPU model the arm stays 45.7 mm or more off the cage's box from all three homes the dog measured, and 55.3 mm
off the lidar's and mount's meshes themselves (straight to the fold: 6-24 mm into them). `--step` now asks before
`tuck` instead of before `fold`. The full test suite also shows `tests/test_payload.py` failing: the mount's mesh
sticks out up to 16.5 mm between its two clearance boxes. That predates this work and is left as it is.

Then Lukas: "it would be good if the dog could be further from the door while pulling it, as near the end it swings
the door into its own face. I believe this can be achieved by positioning the dog abit closer laterally to where it is
gripping the handle so the arm doesnt have to hold the door diagonally".

**The stance** (F-212):
- **What the traces show:** the base turned about the hinge as planned (57-64 degrees for 60), so the door can only
  reach the dog by swinging on past that turn. The lidar had it 8 degrees ahead at 55, and 15-29 ahead through the
  hold (with a few readings of 118-132 that look like misfits).
- **From the simulated stance:** the dog stands in front of the door's middle, 0.54 m from its face, with the grip
  7.5 cm to its right. A door 22.5 degrees past the dog's turn reaches its head there.
- **Moving the dog:** toward the hinge, the door near its face is nearer the hinge and moves less; standing back gives
  room. The arm still solves the whole pull with its full pose from 0.03 to 0.11 m back and 0.05 to 0.15 m right (the
  CPU model; from 0.12 m back it fails).
- **The new stance** (`plan.STANCE_BACK_M` 0.06, `STANCE_RIGHT_M` 0.10, `STANCE_SIDE_TOLERANCE_M` 0.05): the grip
  2.5 cm left of the dog's centre line. The door reaches the dog after 38.3 degrees, 29.9 to 46.2 across the
  tolerances.
- **What else changed:** the range check now wants the dog in line with the handle to 5 cm and says which way to
  move. The walk in stops with the root 0.75 m off, and the box may be 0.71-0.80 m off after it. The approach's and
  the walk back's staging pose is in line with the new stance. In the dry tests the approach landed 4 cm off
  sideways, hence 5 cm rather than 4.

Tests: `test_robot` (new: the fold's path off the lidar, the door's clearance at the stance; the boxes placed from the
new stance), `test_approach`, `test_insert_robot` (the ajar-door chain), 108 in all, pass.

**Not shown:**
- Any of it on the dog.
- Whether the real lidar and mount sit where their CAD meshes do.
- Why the door runs ahead of the dog.
- The stance is now outside every placement the door pull ran in simulation.


### 2026-10-25 · The door pull after the key: walked in on all three axes, to its own stance

[`d1doginsert --live --yes --step --then_door pull`](#/week/6/run/20261025T005025_insert_insert_door_pull_live)
(00:50 UTC):
- The insertion went through: the key in, turned and back in the holster, the dog stood and folded.
- The dog then backed up 0.58 m and looked again: "box 0.80 m from the stance at +5.1 deg (-3 cm left of the stance's
  line) ... walk in 0.26 m".
- After the door pull's straight walk in it aborted: "-5 cm left of the stance's line ...; move the dog right 0.05 m".

Lukas: "can we make this phase walking up to the handle more tolerant?"

The tolerance itself is the arm's: on the CPU model the pull solves with the box's root 0.05-0.15 m right of the
stance (`plan.STANCE_RIGHT_M` 0.10, `STANCE_SIDE_TOLERANCE_M` 0.05). The straight walk in cannot take out what the look
saw sideways.

**Changed:** after the key, the node's door pull is `key_plan.SteppingDoorAttempt` (the door pull alone is unchanged):
- its walk in is the key's walk up (`WalkUp`, all three axes at once);
- its target is the door pull's own stance (`approach.task_stance`), not the look's fix's straight-ahead one;
- when the refit after it still puts the stance out of range, it walks up once more (`DOOR_REWALKS`) to the refitted
  box's stance, then aborts.
- In the dry tests (the gait's carry, the odometry's under-count, the lidar from the true base), from a look 0.26 m
  behind the stance and 4-12 cm to its side, one walk lands 1-2 cm off its line. The straight walk in from 6 cm
  aborts as on the dog.
- Tests: `SteppingDoorTests` 2 (new), `test_insert_robot`, `test_holster`, `test_robot`, `test_approach` all pass.
  `test_key_robot`'s 20 failures and 1 error are the standing key turn's, as before.

**Not shown:** on the dog. Sideways steps of a few centimetres sit in sport mode's dead zone; the walk up raises its
sideways floor when a step does not move the dog (`WALK_UP_LAT_STALL_*`).

Then the door pull alone did the same:
[`d1dog --stage pull --then_close --live`](#/week/6/run/20261025T012721_pull_close_live) (01:27 UTC).
- The first look was 17 cm off, and the approach walked to a place to look again from. The second look was 4 cm left
  of the line, "walk in 0.29 m".
- After the straight walk in: "-6 cm left of the stance's line ...; move the dog right 0.06 m".

So every door pull from the node is now `SteppingDoorAttempt`, not only the one after the key. `_then_close`'s check
for the pull it follows takes it too. A full pull from 6 cm off the line runs through `pull` to the fold in the dry
test. All the suites pass as above.

### 2026-10-25 · Closing and locking: the door pushed shut, the key in turned, the door pushed onto its frame and locked

Lukas: "can you now implement the reverse: a chain of events that close the door - first by grabbing the handle again,
then pushing it closed while moving, then scanning the door again (it has a spring in the last 10 degrees of closing
that makes it stay open 5-10 degrees) then sit down in front of the keyhole, unholster the lever, insert it at -45
degrees since it is already unlocked and then push the key in while wiggling it. instead of turning it instantly, push
until the arm travels no further (door needs some force to fight the spring) and then holding this position begin
rotating from -45 to 0".

**The angle:** in the code (`geometry.Lever`) 0 is level and clockwise as the robot sees it is positive. The unlock went
0 to +45, and the key comes out turned, so the cylinder waits at +45 and takes the key only there. The lock goes in at
+45 and turns back to 0, which is what Lukas called "-45 to 0" (`--lock_from_deg`).

**The chain** (`node.py --then_close` after a door pull, `--lock` on its own; `run_on_dog.sh` names the record `_close`,
`_lock`):
1. **The close** (`plan.DoorClose`, `close_phases`), from where the pull left the dog, the arm folded:
   - the lidar's door angle over 2 s standing (the median; none: the base's turn);
   - a turn about the hinge to it if the door ran on more than 2 degrees past the base;
   - the pull's reach onto the handle at that angle (the jaws where the lidar has the handle, the hand facing as the
     base has turned);
   - the pull's twist reversed from wherever the base stands to 5 degrees, the door following the base's carry through
     the hold (not past shut);
   - then let go, `tuck`, `fold`.
2. **Looking again:** back to the insertion's staging pose and look.
3. **The lock** (`InsertAttempt.lock`): the insertion's whole sequence (walk up, lie down, the wrist camera's look, the
   holster), rolled to +45 with its wiggle about that. Then:
   - `key_close`: the seated key pushed round the door's closing arc about the hinge at `key_in`'s 2 mm/s at the lock
     (0.20 deg/s, the lock 0.566 m from the hinge), its press along the axis kept, until it stops following (the door
     on its frame; at most 15 degrees).
   - `key_turn`: back to level from wherever the wiggle had it in, the press held.
   - Back into the holster; `lock_check` in the record.

**The dry runs** (F-213):
- **The close:** re-grabs and folds from the door at the base's turn (66, no lidar) and swung on to 62-90 degrees, the
  IK's whole pose and the jaws within 0-2 mm throughout. Three fixes came out of it:
  - The push now converges base and door at 5 degrees. The align's carry had left the base 5 degrees ahead, and the
    IK fell back to position only, then jumped at the hold.
  - The hold follows the base, as the pull's does.
  - The hand faces as the base has turned.
- **The lock:**
  - A door 8 and 10 degrees open was pushed shut at 8.4 and 10.4 degrees and the key turned to 0.0.
  - A shut door stopped at 0.6.
  - One 25 degrees open aborted at the cap, and the recovery backs the key out.
  - At 1.5 deg/s the dry shut door was found only with the key 22 mm behind its command, hence `key_in`'s pace.
  - Without the camera's look, doors 4 degrees or more open missed the keyway (the odometry's short count across the
    door face).

Tests: `test_robot` (new: the close from three door angles and one too shut, the flags), `test_insert_robot` (new: the
lock at 8, 0 and 25 degrees; the harness's door turns about its hinge as the key pushes it in `key_close`),
`test_approach`: 114, all pass.

**Not shown:**
- Any of it on the dog.
- The door's spring against the touch: the dry door stays put until `key_close`.
- The lidar's angle on a door past 80 degrees.
- The node's chain itself (it needs ROS).

### 2026-10-25 · The first live close crashed at start-up: a name clash in the node

Lukas ran [`d1dog --stage pull --then_close --live`](#/week/6/run/20261025T012213_pull_close_live) (01:22 UTC). The node
crashed in its first sport-mode callback, before the look, with `AttributeError: __enter__` at `with self.lock:`. Nothing
moved, and the launcher sent StopMove.

The cause was the chain's own flag for the locking insertion, `self.lock = bool(args.lock)`, which replaced the node's
threading lock of the same name. The node's chain code is not exercised by any test without ROS, so the dry tests did
not see it. The flag is now `self.locking`; nothing else the chain added to the node shares a name. Synced to the dog
(`d1update`).

### 2026-10-25 · The close in Isaac Sim, with the holster and key drawn on the robot

Lukas: "can you open up visualisation of the closing process in isaac sim", then "the holster and key isn't in this
visualisation".

**What was added.** The Isaac door demo runs the simulated door script (`demos/unifp/run_demo.py`), not the robot
program's `plan.py`, so the close was added there as its own option:
- `run_combiner_door.sh --close` runs the robot program's close (`plan.DoorClose`) in the demo's terms
  (`demos/combiner_door/env.close_phases`): `door_look` (the simulated MID-360's readings, their median), `door_align`
  round the hinge to that angle, `re_approach` to `re_grip`, `push` round the hinge back to 5 deg, `push_hold`,
  `re_release`, `re_back`.
- The phases are named apart from the pull's, which the record scores by name.
- The record's `close` has the angle the close used, how it was found, the simulator's angle beside it, and the door
  at the end of `push_hold` and of the run.
- `--show_holster` (on with `--close`) draws Lukas's v5 holster and the v4 key in it beside the D1's base, as
  visualization markers posed from the base each policy step: no physics, out of the cameras and the lidar.

**A failure found on the way.** With the holster drawn the pull opened the door in none of 6 attempts over three runs
([viewer, 0/3](#/week/6/run/20261025T011929_door_combiner_door_seed1_combiner_door),
[as kinematic bodies, 0/1](#/week/6/run/20261025T013125_door_combiner_door_seed1_combiner_door_holster_check),
[as markers, 0/2](#/week/6/run/20261025T013427_door_combiner_door_seed1_combiner_door_holster_markers)), against 2 of 2 without it
([plain](#/week/6/run/20261025T012829_door_combiner_door_seed1_combiner_door_pull_check)).
- The jaws held the handle (13-20 N) while the door did not move from the pull's first step, and they slipped off.
- The cause was not the holster's bodies. The drawing code imported `demos/combiner_key/env.py`, and importing that
  module patches `props.BoxSite.root_m` to put every box's root on the floor (the key demo has no plinth). The door
  demo's box stands on one, so the script's grasp went to the wrong height on the handle.
- With nothing from that module imported, the pull opened the door 2 of 2 with the holster drawn
  ([holster, no import](#/week/6/run/20261025T013657_door_combiner_door_seed1_combiner_door_holster_noimport)).
- Any other demo that imports `demos/combiner_key/env.py` beside a plinth box has the same problem.

**Watched, not measured.** Two viewer runs were stopped before a batch ended, so they wrote no trace
([the first](#/week/6/run/20261025T011601_door_combiner_door_seed1_combiner_door), stopped to add the holster;
[the last](#/week/6/run/20261025T013847_door_combiner_door_seed1_combiner_door), closed by Lukas). The close in Isaac Sim has no
recorded result yet: how far it pushed, the lidar against the simulator, and the door at the end are still to be
measured (`--close --attempts 16 --headless`). The simulated door has no spring.

### 2026-10-25 · The live close reached 15 cm short of the handle; it now looks at the tag on the open door first

Lukas ran [`d1dog --stage pull --then_close --live`](#/week/6/run/20261025T013412_pull_close_live) (01:34 UTC):
"after opening, the dog reached for the handle about 15cm short of where it actually was, and also about 7cm below the
handle. this phase should use the apriltag to grab it just like on the open so it is accurate".

**The run:**
- The first look was out of range (8 cm left of the line), and the approach walked 0.23 m to look again. The second
  look was in range ("box 0.80 m from the stance at +5.0 deg, walk in 0.29 m"). The refit after the walk in moved the
  box 51.5 mm.
- The pull opened the door: the base turned 60.9 degrees about the hinge, and the lidar had the door at 75-77 after the
  release (it ran on).
- The close (the old `plan.DoorClose`) read the door at 76.3 degrees (20 of 20 lidar readings) and turned the base 15.5
  degrees about the hinge to it. It then unfolded and reached, all with the IK's whole pose.
- The jaws shut on nothing: servo 6 went to -19.3 units, the empty-jaw value (on the 40 mm lever they stall near 6.8 by
  the code's model; on the dog the pulls hold it at 22-23, corrected in the next run's entry).
  Through the push the base turned from 78.2 to 33.8 degrees while the lidar held the door at 74-83, so the door did
  not follow. Lukas pressed B 3.1 s into the push and recovered; the outcome was "the door was not pushed closed: no
  lock". The lidar's next readings (30.5, then 0.9-1.2 through the recovery) are not explained by the record and may be
  misfits.

**Why it missed** (from the trace; the 15 and 7 cm are Lukas's by eye):
- The close reached from the pull's fix through the odometry. During the pull the base turned about the hinge at (0.578,
  -0.223) in the base. A 60-degree turn about that point ends at (0.096, -0.612) in the script frame. The odometry had
  (0.013, -0.517) at 60.9 degrees, 12.6 cm short. With the jaws on the handle the real base cannot leave that arc by
  more than the arm gives, so this matches the odometry's known short count (`ODOM_WALK_SCALE`, 13%). The close's
  handle was placed that far wrong.
- The height: forward kinematics of the measured joints put the jaws at (0.542, 0.017, 0.308) m in the base at the
  pull's grip, and at (0.529, -0.194, 0.305) at the close's. So the plan sent the jaws to the same height relative to
  the base both times. A hand 7 cm under the handle means the handle stood higher against the trunk than at the pull
  (the trunk lower or pitched). Only a look from where the dog stands sees that. This is inferred, not measured: the
  door task's trace does not log the body height.

**The change** (`key_plan.DoorClose`, `plan.close_phases`, `plan.about_hinge`, `node.py _then_close`):
- After the pull, the dog backs up, folded, to the door pull's staging pose in front of the open door
  (`approach.ApproachAttempt`). That pose is the pull's box turned about the hinge to the lidar's last angle (else the
  base's turn). It looks there.
- From where the pull leaves the dog, the front camera cannot see the tag. By the calibrated lens and the mount, at both
  60 and 76 degrees the tag is 0.18-0.37 m from the lens and runs off the top of the image. Backed up 0.2-0.4 m it is
  whole with 20 px to spare. The staging pose has the open door's tag where the pull's look has the shut one's.
- The tag's enclosure pose from that look is the box whose shut door stands where the open door does. So the pull's
  range check, approach, walk up on three axes, settle and lidar refit (of the open door's face) all run on it
  unchanged, as `SteppingDoorAttempt`. The grasp is planned on the open door's handle.
- The door's angle is the look's heading against the pull's box. The box itself is the look's box turned back about the
  hinge by that angle. That box drives the arm's phases, the lidar's door reading (`Attempt.box_odom`) and RViz. The
  hinge stays put as the door turns, so the reach and the push's centre come from the look, whatever the odometry did.
  An error in the angle only moves where the push ends.
- The record has `close_look` (the look of the open door, beside the pull's `look`) and `close_from_look`: the angle,
  and `hinge_moved_mm`, how far the look moved the hinge from the pull's. That is the odometry's error over the pull
  and the walk back.
- Phases: `unfold`, `walk_in`, `walk_settle`, `stand`, `approach`, `settle`, `enter`, `arrive`, `grip`, `push`,
  `hold`, `release`, `ungrasp`, `home`, `tuck`, `fold` (`door_look` and `door_align` are gone). A door the lidar has
  under 15 degrees after the pull is not walked to; one the look puts under 15 aborts before anything moves.
- `viz.door_pose` now uses `plan.about_hinge`. The Isaac door demo's `--close` (`env.close_phases`) still models the
  old close; its `DOOR_LOOK_*` constants stay in `plan.py` for it, and its comments now say so.

**The dry runs** (`test_robot`). A 60-degree pull, then the door run on to 64, 76 and 88 degrees. The real box was 0,
(10, -7) cm and (10, -7) cm + 1.5 degrees off where the pull's odometry has it. A look from the staging pose, then the
close:
- The grip's target lands within 0.1 mm of the real handle (the test allows 12 mm). The box's hinge lands within 12 mm
  of the real one.
- `hinge_moved_mm` came out at 122 and 116 for the 122 mm offset.
- The angle comes out within 1 degree of the door's, plus the yaw offset.
- The refit fits the open door's face.
- The IK holds the whole pose through grip, push and hold, the jaws within 2-3 mm of their goal. The base ends within
  8 degrees of 5 and the arm folds.
- A door at 10 degrees aborts at once.
- The hinge turn's own test: the turned box's shut handle, lever and hinge equal the box's at the door's angle.

Tests: `test_robot` 56 (the close's three replaced; new: the hinge turn), `test_combiner_door` and `test_approach`
pass, 80 in all. The full suite: 1044 tests, 21 failures and 1 error, all standing: `test_key_robot`'s 20 and 1 (the key
turn), and `test_payload`'s mount mesh (below). 15 skipped.

**Not shown:**
- Any of it on the dog: the walk back, the look at the open door through the front camera, the open door's lidar face
  (the dry scans have only the door, not the box beside its hinge, nor the dock).
- The dry look's detections are exact poses through the head camera's model; the front camera's view of the open door
  is only the geometry above.
- The door's angle carries the odometry's yaw error over the pull, which only shifts where the push ends.
- The node's chain (`_then_close`, the approach, the look's hand-off) has no test without ROS; it was read through, and
  it compiles.

### 2026-10-25 · The grip 1 cm higher on the handle; the hold's first goal no longer jumps

Lukas: "I also noticed the dog grabs the handle 1cm too low each time causing it to rotate the handle down on a grab.
offset the handle 1cm higher".

**The change:**
- `plan.GRASP_RAISE_M = 0.01`: every door attempt's grasp point stands 1 cm above where the box's geometry puts the
  lever (`plan.RaisedGrasp`, through `Attempt.grasp_site`). That covers the pull's reach, grip, pull and hold, the
  close's (`key_plan.DoorClose`), the grasp choice before anything moves, and RViz's grasp marker.
- The key's attempts (`KeyAttempt`, `InsertAttempt`) keep 0. The simulated demo's grasp (`props.BoxSite`) is
  unchanged. The 1 cm is Lukas's eye on the dog, not a measurement.

**A blip it showed.** With the raise, `test_the_arm_stays_on_its_goal_through_a_60_deg_pull_and_hold` failed: the
largest gap from goal to arm went from 2.2 degrees to 5.1, all on joint 4, for one step at the hold's start.
- The cause was older than the raise. `Attempt.step` built the step's state before the pull's clock marked the pull
  done, so the hold's first goal used the pull's nominal 60 degrees while the base had turned 60.75. That is a 3 mm and
  0.75-degree jump in the hand's goal, and back the next step.
- The wrist there (joint 5 near -12 degrees, where joints 4 and 6 nearly line up) turned that into a 6.1-degree trade
  between joints 4 and 6. That is over `JUMP_DEG`'s 6, so the node would have sent it, and the way back, as timed joint
  moves while the hand held the door.
- The state is now rebuilt after the pull's clock. The worst gap through pull and hold is 0.15 degrees (0.13 without
  the raise), and no goal steps more than 0.22 degrees.

**A test changed.** `test_letting_go_keeps_the_fingers_off_the_lever_and_its_hub` now places its lever 1 cm up, where
the jaws close on it. Against the geometry's lever, the raised jaws cleared it by 1.6 mm, not the 10 the test asks for.

Tests: `test_robot`, `test_approach`, `test_combiner_door`, `test_insert_robot`, 137, pass. `test_key_robot` has its
standing 20 failures and 1 error.

**Not shown:** on the dog, whether 1 cm centres the jaws on the lever, and whether the hold's start was ever a
visible twitch there (the live pulls' first hold steps were not looked at for it).

### 2026-10-25 · The look at the open door refused as a tag on a stand; the close backs up 30 cm less

Lukas ran [`d1dog --stage pull --then_close --live`](#/week/6/run/20261025T022431_pull_close_live) (02:24 UTC), the
first with the close's look. Lukas: "the tag is clearly on the door here".

**The run:**
- The first look was in range ("box 0.77 m from the stance at +3.8 deg, walk in 0.09 m"). The pull opened the door:
  the base turned 62 degrees about the hinge, the lidar had the door at 79-80 after it ran on, and the jaws held the
  lever at 22-23 units through the pull and hold.
- That is not the 6.8 the code's lever model gives and that the F-214 entry told Lukas to watch for. F-214 now carries
  a correction: the grip to look for is 22-23 against the empty -19.
- The dog backed up 0.56 m (turning +17 degrees) to the staging pose in front of the open door and looked. 19 of 19
  frames had the tag, and the lidar fitted the door's face 2 cm behind it (5.1 mm rms).
- The look was refused: "the tag is not on a door the box's size: the lidar's face is -2 cm from it and carries on past
  the door's edges (42%)". Nothing moved after that, and the door was left open.
- Lukas's RViz screenshot (pasted in the chat, not in `screenshots/`): the front camera's image, labelled "LIVE | pull
  | looking | door (lidar) 79 deg", shows the open door's face filling the view with the tag outlined in green left of
  centre. The cloud on the right shows the dog with the folded arm in front of the door's face.

**Why** (F-215): the guard against P2Dingo's dock counts the lidar's face points past the door's edges. Near 90 degrees
open, the door's outer face and the box's hinge-side wall face the same way, nearly in line. With that wall added to
the dry look's lidar, the old guard finds 38% (79 degrees) and 45% (88) past the edges, as the dog's 42%.

**The change:**
- The close's look (`node.py _make_fix` while closing, `plan.make_fix(..., pulled_hinge_xy=...)`) keeps the face
  just behind the tag. In place of the edges it checks the look's hinge against where the pull's box has it:
  within `plan.OPEN_DOOR_HINGE_LIMIT_M`, 0.30 m (`Fix.on_a_door`, `Fix.hinge_off_m`).
- A refusal now reads "not on the door the pull opened" and gives the distance. The look's record carries
  `beyond_edges`, `on_a_door` and `hinge_off_m`.
- The pull's own look keeps the edges guard.

**Backing up 30 cm less.** Lukas: "can we reduce the distance the robot backs up by 30cm. it should still be able to
see the apriltag from this distance".
- The close's first staging pose is now `approach.CLOSE_NEARER_M` (0.30 m) nearer: the open door's root 0.75 m
  ahead, about the door pull's stance itself, so its walk up is ~0. `approach.staging_pose` takes the root's
  distance.
- By the calibrated lens and the mount, the tag's top is 77 px above the image there (level, 0.32 m up), and from
  0.85 m it has 22 px.
- Checked against the screenshot: the model, at the look's place as the record has it (the root ~1.18 m ahead, 13 cm
  further than planned: the odometry's error over the pull), puts the tag at columns 434-575 against ~437-580 in the
  image. Its rows come ~40 px lower in the image than the model's, so ~35 px would still be missing at 0.75.
- So a nearer look that finds no tag is not the end: the dog backs up the rest, to `STAGE_ROOT_M`, and looks again
  (`node.py _close_look_from`; the first look's record is `close_look_near`). Not validated either way.

**The dry runs:**
- `run_close` now looks from the nearer pose with the box's side in the lidar. The grip lands within 0.1 mm of the
  real handle raised by `GRASP_RAISE_M`; the test was comparing against the geometry's handle, 10.0 mm under that.
- The refit after the ~0 walk up moved the box 0.9 mm.
- `test_the_open_doors_look_is_guarded_by_its_hinge_not_its_edges`: the old guard refuses at 79 and 88 degrees, the
  new one accepts with the hinge 0.3 mm off, and refuses with the pull's hinge 0.5 m away.
- Tests: `test_robot` 57, `test_approach`, `test_combiner_door`, `test_insert_robot` (81) pass. `test_key_robot` has its
  standing 20 failures and 1 error.

**Not shown:**
- Whether the front camera sees the whole tag from the nearer pose (the lens model says not quite), and the fallback's
  walk on the dog.
- The open door's lidar with the box's side in it on the dog (the look's points are not recorded).
- The node's chain has no test without ROS.

### 2026-10-25 · The close pushed the door to 19 degrees, then out of reach; the base on the hinge's arc, the stance 5 cm nearer

Lukas ran [`d1dog --stage pull --then_close --live`](#/week/6/run/20261025T023655_pull_close_live) (02:37 UTC), and:
"the dog needs to walk closer to the door to avoid the arm extending too far".

**The run** (F-216):
- The pull opened the door to 80 degrees (lidar). The dog backed up 0.33 m (turning +15) to the nearer staging pose
  and saw the tag at once, from 30 cm nearer than before. So the nearer look works here, whatever the lens model said.
- The look's guard passed: the hinge was 122 mm from the pull's, and 38% of the face lay past the edges, which the
  old guard would have refused. The door was at 80.7 degrees.
- The walk up stopped 4.1 cm short and the refit moved the box 19.8 mm. The jaws gripped the handle (21.5 units) at
  0.582 m ahead of the base; the pull's grip was at 0.529.
- Through the push the lidar had the door follow the base, 79 to 19 degrees as the base turned 70 (lagging the
  script's angle by 3-7).
- The IK then failed: the whole pose went 8 degrees into the push, position only from 18, none at 68. The abort read
  "no IK solution for the hand" at the door's 19 (Lukas's RViz shows that line). The recovery's release aborted twice
  more on the IK before the third went through to the fold.
- Lukas's RViz screenshot (pasted in the chat): the front camera's image, labelled "LIVE | pull | push | ABORTED: no IK
  solution for the hand | door (lidar) 12 deg", shows the wall close up with the tag at the left edge. The model shows
  the arm stretched far out ahead and to the left of the turned dog, over the door's face points.

**Why:** the odometry. About the hinge at (0.623, -0.182) in the base, the base's 70-degree turn puts it 0.238 m from
where the odometry had it. The hand on the handle kept the real dog on that arc, and the door followed. But the plan
placed the base by the odometry, so the hand's goal in the base frame went from (0.581, 0.059) at the grip to
(0.589, 0.236): 18 cm sideways, out of reach. It is the same short count as the pull's (12.6 cm over 60, F-214). In the
pull it moves the goal toward the base; pushing, it moves the goal away.

**The changes:**
- `key_plan.DoorClose.base_script`: from the push's start, while the hand holds the door (push and hold), the base's
  position is where the push began, turned about the hinge by the base's own turn. The yaw is the IMU's. Once the hand
  lets go the correction is kept, so the release and the way home stay consistent. The record has
  `close.odometry_off_arc_mm`, and an `arc` event.
- `plan.CLOSE_STANCE_NEARER_M` = 0.05, as Lukas asked: the close's stance 5 cm nearer the door (`DoorClose._at_stance`).
  `plan.CloseFix` checks the range about that stance (the pull's check with the root taken 5 cm further), so the refit
  does not send it back. On the dog the 5.3 cm extra came from the walk up stopping short and the refit, not from the
  stance itself; 5 cm nearer pays for that.

**The dry runs:**
- `run_close` can count only part of the base's translation through push and hold (`walk(..., counted=)`). With 70%,
  the record has the odometry 209 mm off the arc, and the hand's goal stays within 3.1 mm of the grip's in the base
  frame, with the whole pose to the door's 5 degrees. The odometry counting all of it gives the same.
- From its own stance the close grips at 0.475 m ahead of the base, against 0.558 from the pull's. That is 8.3 cm: the
  harness's walk carries past the stance, where the dog's stopped short.
- Tests: `test_robot` 59 (new: the push with the odometry short, the nearer grip). `test_approach`, `test_combiner_door`
  and `test_insert_robot` pass. `test_key_robot` has its standing 20 failures and 1 error.

**Not shown:**
- On the dog: the push with the arc, and the reach from the nearer stance.
- That the door turns 1:1 with the base: the lidar lagged it by 3-7 degrees here, which the arm's give takes up.
- Why the recovery's release lost the IK twice: it started from the arm's out-of-reach pose, which the arc should now
  prevent.

### 2026-10-25 · The close worked; the lock's key pushed the door shut and aborted at 40 mm

Lukas ran [`d1dog --stage pull --then_close --live`](#/week/6/run/20261025T050345_pull_close_live) (05:03 UTC): "the key
insert also closes the door, so it should be allowed to press as far as it can with no abort".

**The run** (F-217):
- **The pull:** the first look was in range ("door +6.8 deg off square, walk in 0.14 m"), and the door opened to 83
  degrees (lidar).
- **The close worked end to end:**
  - From the nearer pose it saw the door at 81.6 degrees, the hinge 152 mm from the pull's.
  - It walked up, gripped, pushed the door to 5 degrees, let go and folded.
  - Through the push the arc correction (F-216) was 322 mm (`odometry_off_arc_mm`): the odometry's short count over an
    80-degree push, which the hand no longer followed.
- **The lock:**
  - The dog backed up 0.90 m, looked ("the push's stance 0.47 m ahead"), walked up and lay down. It drew the key
    (gripper -4.4 units, held; drawn 93.2 mm), and the wrist camera saw the bezel (3 of 3 frames).
  - Through `key_in`, by forward kinematics of the measured and commanded joints along the key's line, the key followed
    its command 65.9 mm with 1.5-3.1 mm of lag. `_stopped` needs 3 mm more than the first second's lag, about 4.6 mm,
    so it never fired. At 40 mm past where the look's door seats the key (`IN_SLACK_M`) it aborted, "no lock there?".
    Lukas saw the door going shut ahead of the key.
- **Record gap:** the trace had no key columns. The run began as the door task, and the trace's columns are fixed at
  its first row. The node now reserves them when `--then_close` will lock.

**The change** (`insert_plan`):
- Locking, a key commanded `IN_SLACK_M` past seated without stopping is taken as seated in a door going shut
  (`_pushing_shut`). The lock's frame turns shut about the hinge by the angle that carries the lock that far (4.0
  degrees at 40 mm), the command goes to `PRESS_M` past seated, and the wrist is held.
- `key_close` then carries the key round the door's arc as it does for a seated key, until the door stops on its frame
  (15 degrees in all at most), and the turn back to level follows.
- Up to 40 mm a touch is read as before. The phase still aborts on its clock.
- Two tries before this one failed dry:
  - Pressing straight on until the key stopped bound the key in the keyway as the door turned under it, short of the
    frame at 0.94 degrees. The 7-degree turn of the frame at once then sent the wrist's goal 30 degrees off.
  - Handing over at half the keyway past seated (4.4 mm) worked, but would take a key with the lock deeper than
    modelled as seated in free air.

**The dry runs** (`test_insert_robot`, `lock_soft`: the door turns shut under the key from `key_in` on):
- Doors 6, 8 and 10 degrees open: handed over at 39.7 mm past seated. `key_close` stopped on the frame at 6.2, 8.18 and
  10.26 degrees, the key 2.3-2.8 mm past seated. The key turned to 0.0, verified, and went back in the holster.
- 25 degrees open aborts at `key_close`'s 15-degree cap.
- The rigid door's tests (8 degrees, shut, 25) pass as before.
- Tests: `test_insert_robot`, `test_robot`, `test_approach`, `test_combiner_door`, 142, pass. `test_key_robot` has its
  standing 20 failures and 1 error.

**Not shown:**
- On the dog: the handover and the arc after it.
- Whether the key was in the keyway or on the lock's face as it pushed.
- A door under ~4 degrees open: shut by the key before 40 mm, it stops past seated and is read as the lock's face.

### 2026-10-25 · One look distance for every walk to a look; the push past shut

Lukas: "the offset the dog uses to look at the door once it opens is a good distance that I want re-used for the
initial look at the closed door, and then also the other look it does once the robot closes it after it is open. Also,
the closing stops 20 degrees from fully closed. it should attempt to close it the full 90 degrees and accept that the
spring will push it back out 5 or 10".

**One look distance** (`approach.STAGE_ROOT_M` 1.05 to 0.75 m):
- Every staging pose now has the box's root 0.75 m ahead: the approach to the closed door, the look after the key, the
  open door for the close, and the box after the close for the lock. That is the close's distance, from which the tag
  was seen in both runs. Those looks were from ~0.88-0.91 m on the dog, its walk back ending short: walk in
  0.11-0.14 m.
- For the door pull, 0.75 is its stance itself (0.7505), so the pull needs no walk in. For the insertion (its stance
  0.60) it is a 0.15 m walk up; before, 0.45.
- A look from there that finds no tag walks on back to 1.05 m (`STAGE_FAR_ROOT_M`) once and looks again. That is the
  close's earlier fallback made general: `node.py _walk_to_look`, `self.staged`, and `looks_without_tag` in the
  record.
- The lens model still says the tag's top is 77 px off the image at 0.75 (`plan.FRONT_LOOK_MIN_ROOT_M` annotated);
  0.75 itself is not validated.
- The first look is wherever Lukas stands the dog, as before.

**The push past shut** (F-218):
- The run of 05:03 UTC: the lidar had the door 16-19 degrees behind the script's angle through the push, and Lukas saw
  it stop ~20 short of shut.
- The push now aims at shut (`CLOSE_TO_DEG` 5 to 0) and 15 degrees of the base's turn past it (`CLOSE_PAST_DEG`). The
  hand turns on with the base past shut, keeping its grip in the base's frame, in the push and the hold.
- Past shut, the base stalling (`PULL_STALL_S`) or the arm's lag check is the door on its frame: the push ends there as
  done (`close.ended`), not an abort.
- The commanded door angle the lidar is compared with stops at shut.

**The dry runs:**
- From an 80-degree door the push ends at -21.2 degrees with the gait's carry.
- With the base stopped at shut (`frame_stops`), it ends "the base stopped turning" at -2.1, the whole pose held, the
  hand's goal within 2 cm of the grip.
- Tests: `test_robot` and `test_approach` 62, `test_insert_robot` and `test_combiner_door`, all pass.
  `test_key_robot` has its standing 20 failures and 1 error.
- Two tests changed with the distance: the staging test now checks 0.75 and the 1.05 fallback, and the door pull after
  the key now has little or no walk in.

**Not shown:**
- On the dog: the looks from 0.75, the fallback, and the push against the frame.
- How hard the gait's 2 s against the frame is on the arm and the handle.

### 2026-10-25 · The whole chain locked the box; the look for the lock moved to face the tag

Three runs of `d1dog --stage pull --then_close --live`. Fetching the third brought back the first two, which had not
been recorded yet.

**[05:29 UTC](#/week/6/run/20261025T052908_pull_close_live): opened, closed and locked** (F-219). This ran the code of
F-217: the push to 5 degrees, and the staging pose at 1.05 m.
- **The pull and the close:** the door opened to 77 degrees (lidar). The close's look put it at 76.6 degrees, the hinge
  112 mm from the pull's, and pushed it to 5. The arc correction was 82.8 mm.
- **Getting the key out:** the look for the lock, from in front of the lock, was in range ("the push's stance 0.47 m
  ahead"). The dog lay down (0.246 m drop) and the lying look found the bezel (3 of 3). The key was drawn (93.1 mm),
  and the wrist camera found the bezel (3 of 3).
- **The key in:** `key_in` handed over at its command's 40.2 mm past seated (`pushing_shut`, measured 37.1). `key_close`
  stopped on the frame at 5.3 degrees.
- **Locked:** the key turned back to 1.4 degrees of level and went back into the holster. "attempt finished: locked".
  This is the first open, close and lock on the dog in one run.

**[05:46 UTC](#/week/6/run/20261025T054651_pull_close_live): the door was locked.** The pull turned the base 22
degrees, the lidar's door stayed at 0, and the arm loaded ("the arm is 15 deg from where the IK wants it"). It aborted
and recovered. The run before had locked the door.

**[05:48 UTC](#/week/6/run/20261025T054805_pull_close_live): the look for the lock saw no tag** (F-220). This was the
first run with the 0.75 m staging and the push past shut.
- The pull and the close worked: the door at 81.4 degrees, the hinge 133 mm from the pull's, pushed shut, the arc
  107 mm.
- The look for the lock, from the insertion's stance moved back to 0.75 m, had no tag. That pose is 0.35 m to the box's
  side, in front of the lock.
- The fallback walked 0.30 m back to 1.05 m, and Lukas stopped it during that look. Lukas: "it laterally aligns itself
  with the key and not the center of the box where the tag is".
- The lens model agrees: 0.35 m to the side the tag is at the image's edge at 0.88 m (columns 1107-1249 of 1280) and
  off it at 0.75. At 1.05 (the 05:29 run) it is inside (1000-1128).

**The change** (`approach.staging_pose`, `LOOK_SIDE_M` 0.10):
- Every staging pose faces the box as the task's stance does, the root `STAGE_ROOT_M` ahead (or the stance's own
  distance if further). It is no more than 0.10 m to the side, on the stance's side.
- The door pull's stance is already 0.10 m to the side, so its looks are unchanged.
- The key's push and the insertion now look from 0.10 m to the side of centre, and walk up the other 0.25 m sideways
  (`key_plan.WALK_UP_SIDE_M` 0.40). Dry, the insertion's look from there is in range: "the push's stance 0.13 m ahead,
  +0.22 m left, a walk up of 0.25 m".
- Tests: `test_approach` checks that the root is within `LOOK_SIDE_M` and on the stance's side. 143 pass
  (`test_approach`, `test_robot`, `test_combiner_door`, `test_insert_robot`), and `test_key_robot` has its standing 20
  and 1.

**Not shown:** on the dog, the look for the lock from the centre and its 0.25 m sideways walk up; and the push past
shut followed by the lock.

### 2026-10-25 · The whole chain from the unlock: unlock, open, close, lock

Lukas: "ok it the sequence works! now lets chain it together with the door unlocking".

The node already chains it: `d1doginsert --then_door pull --then_close --live`, which is `--task insert --stage insert
--then_door pull --then_close`. The unlock comes first, then the walk back and look (`door_after_key`), the pull, the
close from its look, and the lock. Read through against today's changes:
- After the unlock the task is the door, so `_then_close` takes the pull.
- The lock is the insertion with `lock`, which `_then_door` skips, so the run ends on `lock_check`.
- The trace has the key columns from the first row.
- The look after the key uses the new staging pose (0.75 m, within 0.10 m of centre) and its fallback.
- Each hand-off asks for the remote's A: four in all.

**One gap fixed** (`plan.shut_reference`, `AJAR_REFERENCE_MAX_DEG` 20):
- Unlocked, the door hangs ajar on its spring (up to ~10 degrees) when the pull looks. The pull takes that door as
  shut, as in F-210's runs. Its box is then the real one turned about the hinge by the ajar angle.
- Read against it, the close's angle was short by that much. With the overshoot past shut (F-218) the push would still
  have reached the frame from 10 degrees, but the box the lock's staging used would have been 10 degrees off.
- The unlock's look saw the door shut. The close now takes its heading as shut, turned about the pull's own hinge (the
  fresher odometry), when the pull's box is 0-20 degrees open from it. Otherwise, as before. The record has
  `close_reference` with the ajar angle and whether it was used.
- The record also keeps every look in `looks`, since the chain's four looks overwrote `look`.

Dry (`test_robot`):
- A close whose pull took a door 8 degrees ajar as shut reads 68 degrees for a 76-degree door; with the unlock's look,
  76.
- `shut_reference` keeps the pull's hinge and takes the unlock's heading, and is not used for a door shut, closed past,
  or 25 degrees off.
- 145 pass (`test_robot`, `test_approach`, `test_combiner_door`, `test_insert_robot`). `test_key_robot` has its standing
  20 and 1.

**Not shown:**
- The chain on the dog in one run.
- The unlock's heading against the pull's after the insertion's lie-down and stand-up (the IMU's yaw drift between
  them is not measured).
- The node's chain has no test without ROS.

### 2026-10-25 · The whole chain: the lock's press now ends at a depth past the lidar's face

**[06:01 UTC](#/week/6/run/20261025T060125_pull_close_live), `--stage pull --then_close`: locked again.** Found when
fetching; it had not been recorded.
- The first with the centred staging pose (F-220) and the push past shut (F-218). All three looks were from 0.75 m with
  no fallback.
- The close: the door at 81.9 degrees, pushed past shut, the arc 112.9 mm.
- The lock: lying, the lidar refit moved the box 17.9 mm and turned it 0.26 degrees. The key handed over at 37.4 mm past
  seated, `key_close` stopped at 5.59 degrees of arc, and the key turned to 3.9 degrees of level: verified.

**[06:16 UTC](#/week/6/run/20261025T061629_insert_insert_door_pull_close_live),
`d1doginsert --then_door pull --then_close --live`, the whole chain** (F-221):
- **The unlock:** verified, the key turned 41.5 degrees. By the trace's key depth it seated at 1.5 mm past the model's
  seat.
- **The pull:**
  - After the walk back, the look had the door 6.9 degrees off square.
  - `close_reference`: 8.0 degrees ajar against the unlock's look, used.
- **The close:**
  - The door at 91.1 degrees.
  - Pushed past shut.
  - The arc correction was 130 mm.
- **The look for the lock:** from the centred staging pose, in range ("a walk up of 0.16 m").
- **The lock:**
  - Lying, the lidar refit moved the box 17.9 mm and turned it 0.45 degrees: the door shut.
  - The key went 35 mm past seated with no stop and handed over at 40.
  - `key_close` then held the key at its seat in a frame turning round the arc for 35 s, until "no IK solution for the
    hand".
  - Lukas: "everything worked except it kept pushing until the arm was fully outstretched, even moving the box. we need
    a better condition to start turning. make it go a certain distance beyond the front face of the box according to
    the lidar (scanned before the arm covers the lidar)", and "expose this as a launch argument for quick changing".
- With the door shut, a key 35 mm past a seat the same model put 1.5 mm right in the unlock was not in a keyway that
  bottoms: it was pushing the box. The 06:01 lock (~59 mm past seated in all) likely moved it too.

**The change** (`insert_plan`, `node.py --lock_beyond_face_mm`, default 12):
- Locking, the key is pressed straight in until its shaft's tip is `--lock_beyond_face_mm` beyond the box's front face,
  as the lidar saw it lying down before the arm came out (`_refit_lying`'s face; without one, the standing refit and the
  sit-back). It is held there for 1 s (`PRESS_DWELL_S`) and turned. Seated, the tip is 7.7 mm beyond the face, so 12 is
  a 4.3 mm press.
- A key that stops first and is in the keyway (not more than half the keyway out of seated) turns there.
- Further out it touched the lock's face, and the search follows as before.
- No abort for going on (the 40 mm "no lock there?" is the unlock's only).
- The handover and the arc push are gone (`key_close` presses straight to the depth). `lock_check`'s "the door was not
  pushed shut" is now "the key was not pressed home".
- The record has `pressed` (stopped, at depth, on the clock) and `pressed_mm_beyond_face`, and a `pressed` event.

**The dry runs** (`test_insert_robot`, the lock tests rewritten):
- The door shut: the key bottoms seated (7.7 mm beyond the face) and turns: locked.
- Nothing to stop the key: pressed to 12 mm beyond the face (20 when set) and turned.
- A soft door 8 degrees ajar, the lidar seeing it there: 12 mm leaves it at 7.5 degrees; 70 mm at 1.8; 100 mm pushes it
  onto its frame (stopped at 85.5 mm beyond) and turns.
- Tests: 143 pass (`test_insert_robot`, `test_robot`, `test_approach`, `test_combiner_door`). `test_key_robot` has its
  standing 20 and 1.

**Not shown:**
- On the dog: the press to 12 mm and the turn there.
- Whether the door is shut when the dog lies down for the lock in every run. An ajar door needs `--lock_beyond_face_mm`
  raised.

### 2026-10-25 · The jaws fully open for every handle grab

Lukas: "are the jaws open as wide as they can be for the grabs on the upper door handle?" No:
- The robot program sent the simulated door script's 63.2 mm opening (`JAW_LEVER_OPEN_M`, 33.9 units, 23 mm a finger).
- The widest a command opens the jaws is 50.2 units (`d1_hardware.GRIPPER_UNITS_OPEN`; 30 mm a finger by the URDF,
  ~77 mm between them).
- Lukas: "I want it open fully for all handle grabs".

**The change** (`plan.GRIPPER_OPEN_UNITS`, `Attempt.gripper_units`, `handle_jaws`):
- On the door's handle (the pull, the close, the release after an abort), the script's jaw range, from shut to the
  lever's opening, is sent stretched over shut to fully open. So the open phases send 50.2, and the grip's and the
  release's ramps end there too.
- The key's attempts (`KeyAttempt`, `InsertAttempt`), which grip the key and the holster with the same script opening,
  keep it as it was. The simulated demo is unchanged.
- The release's check that the jaws opened compares against what was sent.
- `JAW_WAIT_S` goes from 2 to 4 s: fully open is 43 units from the lever. The dog's jaws open at ~50 units/s (the
  06:01 release: 22.9 to 32.8 units in 0.2 s); a 12 units/s gripper needs 3.6 s.

**A blip it showed:** the dry grip took one step longer with the wider jaws, and `test_the_arm_stays_on_its_goal...`
failed (Joint4 6.7 degrees off its goal). The cause was older:
- The step the grip ended carried its leftover time into the pull on the plain clock. The pull's clock is the base's
  turn, so its first goal was a degree or two of door ahead of the base and came back the next step, 7.6 degrees on
  Joint4 by the near-aligned wrist.
- A phase going into a turn-clocked one (`pull`, `push`: `TURN_CLOCKED_PHASES`) now lands at its start. The close's
  push had the same, and its jaws' miss went back under 6 mm.

**The dry runs:**
- The pull sends 50.2 through approach, settle, enter, arrive and ungrasp, its grip and release ramps run -19.8 to
  50.2, and the key's attempts keep theirs.
- Letting go with the fingers fully open, they clear the lever by 18.5 mm (11.6 before) and its hub by 41.8.
- The slow gripper's hand leaves only once the jaws are open.
- 144 pass (`test_robot`, `test_approach`, `test_combiner_door`, `test_insert_robot`). `test_key_robot` has its standing
  20 and 1.

**Not shown:**
- On the dog: the wider jaws round the handle and the hub.
- The units-to-mm map is linear between its measured ends (F-063), so the 77 mm is the URDF's, not measured.

### 2026-10-25 · Letting go of the handle without turning it down (F-222)

Lukas: "the jaws have a habit of rotating the top handle a bit downwards when it leaves the grab position after an open
or close. can we have it carefully open its jaws while holding its position, and then back out without bumping it".

**The cause, in the recorded traces** (forward kinematics of the arm's goal and its measured joints, at the step before
each release):
- At 15 of the 16 door releases of 2026-10-24/25 (the open's and the close's), the arm's goal stood 8.5-15.0 mm
  above where it measured itself; the 01:34 run's close had 31.8. Its wrist was 1.5-2.9 degrees off. The arm sags
  under its own weight reaching out to the handle. At the home pose it sags 10 mm.
- The release's goal was the measured pose, so through its 0.8 s the hand's goal came down by the sag with the jaws
  still shut: in the [06:01 run](#/week/6/run/20261025T060125_pull_close_live)'s close, 332 to 319 mm while the hand
  stayed at 317-319. The arm pressed the lever down by its sag while the jaws held it.
- Once the jaws opened, the hand dropped 11-26 mm below where it had held the lever (41-58 in the 01:34 run; 06:01:
  319 to 302 mm, both releases), onto the lever on its way out.
- The script's jaw ramp ran from shut (-19.8 units). The jaws, at 21-22 units on the lever, began to move in its last
  0.3 s, and the hand left about 0.1 s after they opened.

**The change** (`plan.LET_GO_S`, `LET_GO_RAMP_S`, `LET_GO_SAG_MAX_M`, `Attempt._capture`), on the door's handle only
(`handle_jaws`: the pull, the close, the release after an abort):
- The hand holds, across the door, where it measured itself as the release began, as before. In height and wrist it
  holds what it was commanded on the handle, its sag kept (capped at 30 mm). The ungrasp backs out along that
  approach at that height. The `let_go` event records `sag_mm`.
- The jaws open from where servo 6 reports them on the lever to fully open over 1.5 s. The hand stays still with them
  open until 2.0 s (was 0.8), then, once servo 6 reports them open, backs out.
- The key's attempts keep their letting go.

**Dry** (`test_letting_go_of_the_handle_keeps_the_height_it_was_held_at`): a model arm sagging 1.5 degrees on
Joint2 (16 mm at the hand on the harness's handle) dropped 16.1 mm letting go, held at its measured height. Now it
stays within 0.1 mm of where it held the lever until it backs out. The jaws' commands run from the lever's reading to
50.2 in 1.5 s and the release lasts 2 s. 145 pass (`test_robot`, `test_approach`, `test_combiner_door`,
`test_insert_robot`); `test_key_robot` has its standing 20 and 1.

**Not shown:**
- On the dog: whether the handle now stays level as the hand leaves. A sag that changes once the lever's load comes
  off (the jaws open) is not modelled. The traces put the free arm's sag at 13-19 mm, about the same as holding.
- Nothing ran on the dog. Not committed. `d1update` first.

### 2026-10-25 · The holster grip 2 mm further up the lever

Lukas: "can we also make the arm grab the lever from its holster 2mm further up its axis (away from the holster).
this will affect the key insert so account for that".

**Changed** (`insert_plan.DOG_GRIP_ALONG_M` 6 → 4 mm). The dog's draw closes the jaws 2 mm back along the bar, out of
the holster (the draw's way out, `-u`). `DOG_GRIP_ONTO_M` (3.5 mm onto the bar) is unchanged. The insertion's model of
the key in the jaws follows: `HOLSTER_SHAFT_ALONG_M = 0.006 - DOG_GRIP_ALONG_M` puts the shaft 2 mm along the bar from
the jaws' axis (0 until now). So the key goes in and turns about the shaft, not about Joint6: the wrist search, the
turn (`_rolled`, the IK about the shaft) and `key_in_hand`'s measure take it 2 mm off the axis. That is the path the
6 mm offset took before 2026-10-25. `--key_right_mm` still nudges on top of it.

**Dry:**
- The CPU model's draw solves at 4 mm to the exit, every joint within 0.960 of its range (6 mm: 0.944; at the shaft:
  0.968).
- `test_insert_robot` 57 pass. Its holster insertions hold the key 2 mm off the jaws' axis as modelled, seat, turn
  and lock.

**Not shown:** on the dog, whether the jaws close 2 mm further up and the key still lands on the keyway. The 6 mm
model came from Lukas's photos (the shaft 6 mm past the jaws with the grip planned at the shaft). The 2 mm assumes the
dog's grip error is unchanged. If the key lands off, `--key_right_mm` corrects it.

### 2026-10-25 · The holster key lands 2 mm left and 2 mm up

Lukas, on the dog with the grip 4 mm along: the key "hits the bottom right and doesnt go in"; "can you make it insert
the key up 2mm and to the left 2mm". (No run directory was recorded for this.)

**Changed** (`insert_plan.HOLSTER_KEY_LEFT_M`, `HOLSTER_KEY_UP_M`, `holster_shaft`). The holster key's model of the
grip now lands the key 2 mm to the robot's left and 2 mm up. That is what `--key_right_mm -2 --key_up_mm 2` did on the
model before: the shaft modelled 4 mm along the bar from the jaws and 2 mm below their axis. `--key_right_mm` and
`--key_up_mm` still default to 0 and add to it. The handed-in key is not moved. The help text says so.

**Dry:** `test_insert_robot` 58 pass. The new test: the default landing equals those nudges on the old model. The
harness's true key follows the model's, so its insertions from the holster seat with the shaft 2 mm off the jaws'
axis in both directions. The wrist search's roll about the shaft now includes the shaft's offset up the bar.
146 pass with `test_robot`, `test_approach`, `test_combiner_door`.

**Not shown:** on the dog. Whether it is the grip or the lock's place that is 2 mm off is not known. Both give the same
correction for the holster key.

### 2026-10-25 · The landing as launch arguments; is every arm move the smooth kind? (F-223)

Lukas: "can you expose this as a launch argument so I can change it on the fly? also I recall when doing the
unholstering tests that we found a way to make the motions smooth with the exception of the occassional lag from the
arm controller. make sure that is applied to every motion the arm does in the whole sequence".

**The launch argument.** `--key_right_mm` and `--key_up_mm` now default, from the holster, to the landing above (-2
and 2: `HOLSTER_KEY_LEFT_M`, `HOLSTER_KEY_UP_M`), and handed in to 0. A value given replaces the default; it no longer
adds to it. `InsertAttempt.key_right_m` and `key_up_m` are None until resolved, and `run["insert"]` records what was
used. The dry harness's true key defaults to where the attempt models it. `test_insert_robot` 58 pass. The new test
covers the defaults, a replacement, the hand-in and the parsed arguments. 146 pass with `test_robot`, `test_approach`
and `test_combiner_door`.

**The smooth motion.** What made the holster motions smooth (F-199) was sending each move to the arm link as one
timed path, each joint at its own full pace (~4 degrees a command), with the goal followed with a lead allowing for
the report's age otherwise. Since 2026-10-24 every arm command of every task goes through it (`plan.ArmSender`; the
node sends nothing else). From the arm link's own command log of the whole chain on the dog
([06:16 UTC](#/week/6/run/20261025T061629_insert_insert_door_pull_close_live), 3360 commands; aligned to the trace's
phases by the measured joints, 0.00 degrees median mismatch):
- **Joint moves**, sent as timed paths (unfold, the look's pose, `h_turn`, `h_reach`, `h_lower`, `key_go`,
  `h_return`, `h_raise`, `h_unreach`, `h_unturn`, tuck, fold): 513 moving commands, median 4.02 degrees, 12% under
  2.5. That is the holster test's result.
- **Line moves**, the hand's goal followed along the script (approach, enter, pull, push, ungrasp, `h_draw`,
  `h_exit`, `key_in`, `key_turn`, `h_back`, `key_close` and the rest): 1604 moving commands, median 0.82 degrees, 82%
  under 2.5. That is the stop-start pattern F-199 found for near targets.
  - Under 2.5 degrees a command: the approach 100%, enter 78%, pull 68%, push 86%, ungrasp 98%, `key_in` 71% (415
    commands), `key_turn` 92%, `h_back` 81%, `key_close` 100%.
  - `h_draw` was 48% (median 2.64) and `h_on` 19% (4.51).

So the method is applied to every move, but it only makes a move smooth if the move is a joint move or fast. A line
moves each joint by the hand's speed, and at the script's paces that is ~1 degree a command. Making the line moves
smooth needs one of two things:
- Joint moves where the path's shape is free: the approach to the standoff, home after the ungrasp, the close's
  approach. These would need checking against the box and the lidar (F-211: joint-paced moves change the path).
- Faster lines.

The precise ones (onto the handle, into the keyway, the press, the holster's channel) are slow on purpose. Not
changed; for Lukas to choose.

### 2026-10-25 · `--no_wiggle`, one Enter per run, the landing back to 0

Lukas asked whether the insertion feels around. It does both, one after the other: it presses straight in along the
keyway's axis, the wrist rolling ±10 degrees about the shaft from 4 mm before the face. If the key stops on the face,
it spirals across it (0.4 mm pitch, 1.5 mm/s, out to 4 mm, pressed 2 mm in) until the key advances 1.2 mm. He then
asked for three changes.

**The changes:**
- *"can you add a launch option to remove wiggle"*: `--no_wiggle` (`InsertAttempt.wiggle`, recorded as
  `check["wiggle"]`). The plan gives the wrist's search 0 degrees each way, so the hand never rolls off the angle the
  key came to the lock at. The press and the spiral are unchanged.
- *"remove the need to confirm at the start of each stage - make it only wait for user signal once at the start"*:
  `node.started`, set by the first Enter (or A) at the start and never reset. The stages the chain goes on to
  (`_then_door`, `_then_close`, a walk to a new look, the out-of-range approach) no longer pause for it. Before, each
  of them reset `confirmed` and asked again. `--step`'s pause before each moving step is unchanged.
- *"undo the -2, 2 default, make it 0,0 for the key hole offset"*: `HOLSTER_KEY_LEFT_M` and `HOLSTER_KEY_UP_M` are 0.
  `--key_right_mm` and `--key_up_mm` still set it per run (`--key_right_mm -2 --key_up_mm 2` for the earlier landing).

**Tests:** 158 pass (`test_insert_robot`, `test_robot`, `test_approach`, `test_combiner_door`, `test_remote_buttons`).
- `test_no_wiggle_pushes_the_key_straight_in_with_the_wrist_held`: on the keyway and 1.5 mm off it (found by the
  spiral), the wrist stays at its arrival angle and the key seats.
- `test_the_start_is_confirmed_once_for_the_whole_run`.
- The landing test now expects 0 by default.

**Not shown:** any of it on the dog. Nothing commits; `d1update` first.

### 2026-10-25 · `--no_wiggle` corrected: the wiggle stays, the spiral goes

Lukas: "correction: I want no_wiggle to still wiggle but not apply the translational spiralling. no need to run the
test suite for this change I want to test it myself".

**Changed:**
- The option is now `--no_spiral`; `--no_wiggle` is an alias (`InsertAttempt.spiral`, recorded as `check["spiral"]`).
  The wrist's ±10-degree search runs as by default.
- When the key stops on the lock's face, `key_search` keeps it pressed `PRESS_M` (2 mm) into the face where it
  touched, never moved across, and the wrist keeps rolling.
- If the key goes 1.2 mm further, it is in, as before. After `PRESS_ONLY_S` (20 s, 10 of the wiggle's periods) the
  search gives up: "... never went further in 20 s of the wrist's search (no spiral)". The spiral gives up at 4 mm,
  ~85 s.
- The previous entry's version (the wrist held at 0 degrees) is undone.

**Tests:** not run, at Lukas's request. The files compile. The no-wiggle test is rewritten as
`test_no_spiral_wiggles_where_the_key_touched_and_does_not_move_across`: on the keyway it seats; 1.5 mm off it it
stays put, wiggles, and gives up. It is unrun.

### 2026-10-25 · The unlock's turn commanded 5 degrees past 45

Lukas: "the unlocking isn't rotating the full 45 degrees on the real robot, if it is commanded 45, lets command it 50
to compensate". (No run directory for the short turn has been fetched.)

**Changed** (`insert_plan.UNLOCK_OVERTURN_DEG` 5, `InsertAttempt.turn_extra_deg`, `node.py --turn_extra_deg`):
- The unlock's turn is planned and commanded at `--turn_deg` + 5, so 50 by default.
- If the arm cannot hold 50, the plan steps down 5 degrees at a time as before (45, 40, ... to `TURN_MIN_DEG` 30).
  `run["insert"]["turn_deg"]` records the turn it took and `turned_deg` the encoders' turn.
- `--turn_deg 0` still means no turn. The lock's turn back to level and `--lock_from_deg` (45, where the unlock leaves
  the cylinder) are unchanged.

**Dry:** `test_insert_robot` 59 pass, among them the previous entry's unrun no-spiral test. The holster insertion plans
and turns 50 (the encoders 50 past the entry angle, Joint6 alone).

**Not shown:** on the dog, whether 50 commanded turns the key to 45. The shortfall's cause (the arm's lag under the
lock's torque, or the key turning in the jaws) is not measured. If the plan steps down on the dog, the run's
`turn_deg` shows it.

Then Lukas: "make the default overturn 10 degrees actually, no need to rerun the tests". `UNLOCK_OVERTURN_DEG` is 10:
55 commanded. The turn test now expects 45 + `UNLOCK_OVERTURN_DEG`. It has not been rerun at 10, so whether the dry
plan holds 55 or steps down to 50 is not shown.

### 2026-10-25 · The whole chain on the dog, 06:47-08:50 UTC: locked in five runs; `d1dogdemo` (F-224)

Lukas ran the whole chain (unlock with the holster's key, pull open, push shut, lock) eleven times after the 06:16 run.
The command was `d1doginsert --then_door pull --then_close --live` with the arguments below; fetched with `d1fetch`
and recorded. The last one worked end to end. Lukas: "this command worked ... can you make them the defaults so I just
type d1dogdemo and it does all of that".

| UTC | arguments | the grip's model | unlock: contact, search | turn (encoders) | outcome |
|---|---|---|---|---|---|
| [06:47](#/week/6/run/20261025T064754_insert_insert_door_pull_close_live) | | shaft on the axis | -1.55 mm, straight in | 45 → 51.6 | opened; the close not pushed (operator stop) |
| [07:16](#/week/6/run/20261025T071613_insert_insert_door_pull_close_live) | | | | | operator stop before the draw |
| [07:17](#/week/6/run/20261025T071701_insert_insert_door_pull_close_live) | | | 5.37 out, spiral 2.06 mm | 45 → 49.4 | **locked** (2.0 deg) |
| [07:29](#/week/6/run/20261025T072959_insert_insert_door_pull_close_live) | `--lock_beyond_face_mm 40` | | 0.36, straight in | 45 → 45.1 | **locked** (4.5) |
| [07:46](#/week/6/run/20261025T074630_insert_insert_door_pull_close_live) | `55` | 2 mm along | 9.71, spiral 1.53 | | "the key stuck 2.8 mm short of seated" |
| [07:49](#/week/6/run/20261025T074911_insert_insert_door_pull_close_live) | `55` | 2 mm along | -2.22, straight in | 45 → 33.5 | **locked** (2.2) |
| [08:26](#/week/6/run/20261025T082622_insert_insert_door_pull_close_live) | `55`, landing -2/+2 | 4 along, 2 down | 9.76, spiral 0.54 | | operator stop in the search |
| [08:38](#/week/6/run/20261025T083809_insert_insert_door_pull_close_live) | `55 --no_wiggle` (the wrist held) | 2 mm along | 8.21, spiral 0.64 | | "the key did not stop at seated: 7.8 mm past the lock's face" |
| [08:42](#/week/6/run/20261025T084234_insert_insert_door_pull_close_live) | `55 --no_spiral` | 2 mm along | -2.99, straight in | 45 → 34.3 | **locked** (2.4) |
| [08:49](#/week/6/run/20261025T084956_insert_insert_door_pull_close_live) | `55 --no_spiral` | | | | operator stop before the draw |
| [08:50](#/week/6/run/20261025T085057_insert_insert_door_pull_close_live) | `55 --no_spiral` | 2 mm along | 0.87, straight in | **55 → 44.3** | **locked** (2.6) |

How to read the table:
- Contact is the key's back face out of the model's seated when it stopped. The turn is planned → measured by the
  encoders, from the entry angle. The lock's level after its turn is in brackets.
- In every locked run the close pushed the door shut from 79-92 degrees (the tag's), `odometry_off_arc_mm` 121-131.

**What the runs show:**
- The unlock's turn with the grip 2 mm off the shaft fell short: 33.5 and 34.3 for 45 commanded. With the shaft on
  the jaws' axis it was 45.1-51.6. At 55 commanded it measured 44.3, so the overturn did what it was for in the one
  run.
- The 08:38 run was the first `--no_wiggle` (the wrist held, before the correction). Its key went in after a short
  spiral and then did not stop at seated.

**`d1dogdemo`** (`~/.bashrc`, local, beside `d1doginsert`): `d1doginsert --then_door pull --then_close --live
--lock_beyond_face_mm 55 --no_spiral "$@"`. It is LIVE, with the one Enter or A at the start. Later arguments
override, since argparse keeps the last. The 10-degree overturn is the program's default now, so it needs no flag.

### 2026-10-24 · Force-aware against position-only on the dog's hands (round 4's matched pair)

Lukas: "Does this demonstrate force aware is better than position-only?" Not today's work: every light-leg fine-tune is
force-aware, with no position-only twin. Round 4's matched pair (seed 3, the same recipe but the variant) on the dog's
two hands from its rest, presses of 288 / fell / past 60 N, force error median:

| legs | force-aware | position-only |
| --- | --- | --- |
| trained (0.01 / 0.037) | 280 / 0 / 5, 1.2 N | [0 / 0 / 63, 14.1 N](#/week/6/run/20261024T031505_leg_eval_seed1_press_r4_position_5999_hands) |
| armature 0.0002, 15-20 ms | 10 / 83 / 166 | [0 / 207 / 94](#/week/6/run/20261024T031547_leg_eval_seed1_press_r4_position_5999_hands_light) |
| armature 0, 5-15 ms | 0 / 62 / 228 | [0 / 224 / 74](#/week/6/run/20261024T031634_leg_eval_seed1_press_r4_position_5999_hands_zero) |

- **On the legs they trained on,** force-aware presses the dog's hands and position-only presses none. This is
  consistent with F-180 (the frozen set: 197 against 100, a second seed 186 against 58; the sweep 126-229 against
  19-95; the key zero-shot 107 against 72).
- **The criterion is the force's accuracy** (within max(3 N, 20%)): a position-only policy is told a displacement, not a
  force, so on a plate of unknown stiffness it misses by construction. That is the argument for force awareness, and
  the comparison should be stated with it.
- **On light legs both of round 4's policies fail,** and only the force-aware side was trained on them (ft6 to
  ft11a). F-226 is a weakness particular to force awareness there, so this pair says nothing about light legs.
- **Neither variant has pressed anything on the dog.**

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

- A press candidate for light legs (F-227, simulation): ft11a's final, from five rounds of rima-style fine-tunes.
  - From the dog's rest it presses the dog's two hands 211 / 149 / 108 of 288 on trained / light / zero-armature legs,
    with no falls (round 4: 280 / 10 / 0, with 0 / 83 / 62 falls).
  - It does not rattle, and it waits 12-14° nose up.
  - Its weak press is the palm under the foot on light legs (27 of 144), where its force estimate reads a press that is
    not there (F-226).
  - `checkpoints/leg_press_force_ft11a_model_11394.npz`.
- Why the dog sat back (F-194, F-195, simulation): the foot-press policy crouches 17-25° nose up within 0.3 s of every
  start, as the dog did; its tilt stop fired on that posture.
  - On the dog's own hands (a new 288-press set at 5 and 8 N) round 4's final presses 280, palm_up 144 of 144, with no
    falls.
  - Level-base fine-tunes stop the crouch (ft4: no 10 s stand past 15°, round 4: 284 of 288) but lose the palm press
    (42-64 of 144) or tip over.
- The foot-press policy from the dog's handover (F-192, F-193, simulation, frozen v2 set of 288 presses):
  - Simulating the handover gives the dog's kp-80 hold (0.306 m, 4.9° nose up; the dog 0.303 m, 4.6°) and sinks
    further at kp 25 (0.215 m against the dog's 0.261).
  - From every stance it gives, at rest or sinking, round 4's final presses 197-203 without a fall, as from its stands
    (196). Leg joint friction at 1 N·m takes it to 139.
  - A fine-tune on those starts tipped on upward presses (19-33 fell). With the upward force capped at 16 N in
    training it stands, but presses no better than round 4 where the dog presses.
  - Exported for the dog with a start gate admitting its rest: `checkpoints/leg_press_force_ft2a_model_6598.npz`.
- The holster draw made faster (F-190, simulation, 8 attempts each): paced at 0.6 rad/s on the fastest joint and the
  unfold doubled, 12.3 s instead of 18.5, the draw's stop-start 8% instead of 28%, tracking unchanged (lag 7.9 mm, grip
  3.2 mm); evenly doubled it lagged 37 mm, tripled it drew 2 of 8.
- Lukas's key holster beside the D1's base, re-angled to 41° (F-189, simulation): the key drawn out by its root in
  25 of 25 attempts after a registration fix (holster as planned, and up to 3 and 6 mm off per axis); at the first
  export's 35° the CPU model draws it only 70-75 mm of the 75 needed. `figures/holster_draw_frames.png`.

- A general force-aware foot-press policy (F-180-F-182, simulation): on 288 frozen presses (48 plates across the
  front-right foot's reach x rigid / sprung x 8-28 N) from the dog's stand on the realistic legs, force-aware 197 (seed 3)
  and 186 (seed 4), the force's median error 2.3-2.5 N, against position-only 100 and 58; zero-shot on F-179's key
  evaluation 107 and 104 of 108 (the key specialist 101). Across eight changes to the legs and stance 126-229 against
  19-95. The candidate, round 4's force-aware final, on the final layer and plant: 196 / 207, on the slide set 56 of 96
  with nothing past 34 N. `figures/foot_press_eval_r4.png`; exported `checkpoints/leg_press_force_r4_model_5999.npz`.

- The fine-tune on Lukas's turn (F-179, simulation): from the dog's measured stand on the realistic legs, force-aware
  101 / 100 of 108 with no contact past 60 N (max 35 N); 95-105 over 0-15 ms delay and armature 0.003-0.02; the
  matched position-only final 30 / 27. Exported for shadow mode: `checkpoints/leg_key_force_ft045_model_2604.npz`.
- The key turned by the arm (F-183, simulation, 43 attempts): against Lukas's 5 N (0.65 N·m) 27/27 unlocked to the
  stop from the head camera and lidar look (16 of them with the walk stopping in a window, not on a point); 4/4 at 8 and 11 N, 1/4 at 14, 0/4 at 17; the arm's quasi-static ceiling
  10-11.5 N. `figures/arm_turn_demo_5n.png`, `figures/arm_turn_lock_sweep.png`.

- The key policy in shadow mode on the real dog (F-173, one 90 s run, open loop): 50 Hz held (period p99 20.7 ms,
  inference 3.1 ms on the dog), frames as the training code's to 1e-17, targets within 0.013-0.078 rad RMS of a still
  model's. Pushes confirm pitch + nose down, roll + right side down, FR first, the quaternion as w, x, y, z. The sport
  stand from the encoders is 38.7 x 31.7 cm under 0.32-0.33 m. `figures/leg_shadow_dog_20261021.png`. A second run with
  the foot asked up (F-174): the same agreement; 38.4 x 27.8 cm under 0.337 m; the loop stalls at 30 and 60 s.
  `figures/leg_shadow_dog_20261021_foot_goal.png`.

- The key policies on other leg models (F-168, simulation, 97 frozen-set runs): at the trained armature every torque
  model tried (Isaac Lab's DCMotor curve and 23.5 N·m knee, unitree_rl_lab's envelope, the geared knee) moves the
  force-aware results by at most 2 keys (82-85, 106, 102-104); the stock group's zero armature takes seed 1 to 3; a
  0-10 ms delay costs seed 1 17 and the fine-tune 24-41; at armature 0.01 (knees 0.037) force-aware 169 and
  position-only 136 of 216, against 190 and 43. From the sport mode's classic stand at rest (F-170), the fine-tune
  unlocks 96-99 of 108 at 0.28-0.32 m. `figures/key_leg_actuators.png`.

- The combiner key turned by the front-right foot (F-166, side study, simulation): 108 frozen keys a seed, unlocked
  with no damage and the foot back down -- force-aware 84 and 106 (190 of 216), position-only 20 and 23 (43); zero-shot
  0 each. On the decoupled demos' DCMotor legs, seed 1: 3 and 20 (F-165), from the stock group's zero leg armature,
  not its torque limits (F-168). `figures/key_eval_stage2_seed{1,2}.png`,
  `figures/key_training_curves_seed{1,2}.png`.
- The key inserted without learning (F-164): seated from a given box; 0/12 from the head camera's and lidar's look
  (3-11 mm off in depth).

- The front-right foot on a door handle (F-160, side study, simulation): after a handle stage, 60 of 288 frozen
  handles held 1 s by the position-only policy (seeds 17 and 43) against 15 by the force-aware one (12 and 3); zero-shot
  from free-space training, 0/144 each. `figures/leg_handle_eval_stage2_seed{1,2}.png`,
  `figures/leg_handle_frames_*_handle64.png`.

## Findings this week

- [F-227](../findings.md): trained on the dog's likely leg corner with rima's regularisation and the dog's hands
  oversampled, the foot-press policy presses those hands on light legs without falling or rattling (149 of 288 against
  round 4's 10 with 83 falls); the palm under the foot stays weak there (provisional, simulation).
- [F-226](../findings.md): on light legs with a long delay the force-aware policy's estimate of its foot's force reads
  a downward press that is not there (0 N real, 3-7 N estimated), so it does not press (provisional, simulation).
- [F-225](../findings.md): on the real dog the rima-legs fine-tune (ft6) stands its 10 s without the rattle: 0.006-0.012
  rad a step on the worst joints against round 4's 0.118-0.229 (confirmed, one live run).
- [F-224](../findings.md): with the holster's grip 2 mm off the shaft the unlock's 45 turned the key 33.5 and 34.3
  degrees by the encoders, 55 commanded 44.3; the whole chain locked in five of eleven runs; the unlock is now commanded
  10 past (provisional).
- [F-223](../findings.md): in the whole chain the arm's joint moves stepped a median 4.02 degrees a command, its line
  moves 0.82 (82% under 2.5): every move goes through the holster test's method, which smooths only joint moves
  (provisional, one run's command log).
- [F-222](../findings.md): the arm's goal stood 8.5-15 mm above the measured hand at the door's releases, so letting go
  pressed the lever down; the release now holds the commanded height and opens the jaws slowly (provisional, dry).
- [F-221](../findings.md): the whole chain on the dog worked but the lock's press, which pushed the box until the IK ran
  out; the press now ends `--lock_beyond_face_mm` (12) beyond the lidar's face, then turns (provisional, dry; supersedes
  F-217's handover).
- [F-220](../findings.md): the look for the lock, 0.35 m to the box's side at 0.75 m, saw no tag; staging poses now
  keep the box's root within 0.10 m of straight ahead (provisional, dry).
- [F-219](../findings.md): on the dog the program opened, closed and locked the box in one run (05:29 UTC; provisional,
  one run).
- [F-218](../findings.md): pushed to 5 degrees the door stopped ~20 short of shut, lagging the base 16-19 by the lidar;
  the push now aims 15 past shut and ends where the frame stops it (provisional, dry).
- [F-217](../findings.md): the whole close worked on the dog; the lock's key pushed the door shut ahead of it and
  aborted at 40 mm past seated; locking, a key that far past now hands over to `key_close` (provisional, dry).
- [F-216](../findings.md): the close gripped and pushed the door 80 to 19 degrees, then ran out of reach: the odometry
  24 cm short of the hinge's arc moved the hand's goal 18 cm sideways; the base is now on the arc while holding the door,
  and the close's stance 5 cm nearer (provisional).
- [F-215](../findings.md): the close's look at the open door refused by the dock guard (42% past the edges: the box's
  side in line with a door near 90 degrees); the close's look now checks its hinge against the pull's (provisional).
- [F-214](../findings.md): on the dog the close from the pull's fix reached ~15 cm short and ~7 cm under the handle and
  shut on nothing (the odometry 12.6 cm short over the pull's turn); the close now looks at the open door's tag first,
  and dry its grip lands within 0.1 mm of the real handle with the box 12 cm off the odometry (provisional). Supersedes
  F-213's close.
- [F-213](../findings.md): dry, the program closes and locks: the handle re-grabbed at 62-90 degrees with the whole pose,
  the door walked to 5; the key in at 45 pushes a door 8-10 degrees open shut and turns to level (provisional).
- [F-212](../findings.md): the door pull's stance 6 cm back and 10 cm toward the hinge: a door swinging past the dog's
  turn reaches its face after 38 degrees, not 22.5; the arm still solves the pull (provisional, geometry and CPU model).
- [F-211](../findings.md): the door pull's fold hit the lidar (the elbow shut first, the shoulder swinging the forearm back);
  folding via the elbow 30 degrees open keeps 45.7 mm off it on the CPU model (provisional).
- [F-210](../findings.md): on the dog the door pull after the insertion went through twice: backed up, looked again in range,
  pulled 60 degrees (provisional).
- [F-209](../findings.md): on the dog the door pull after the insertion, walked over from the insertion's fix, aborted
  0.65 m from the box after its refit (moved 33 mm, turned 5.9 degrees; the gait left out of classic walk by the
  postures). The chain now backs up to look again, and dry it pulls a door left 0-15 degrees open (provisional).
- [F-208](../findings.md): the v5 holster (a mouth, the lever 15 mm further out) draws the key and takes it back in
  simulation from a plan up to 5 mm off it, 24 of 24; from the higher grip the arm reaches 86 mm along the axis, so
  the draw leaves through the mouth at 25 degrees (provisional, simulation and the CPU model).
- [F-207](../findings.md): the Go2 remote's buttons reach the dog's `LowState` (L2, L2 + A and B recorded on one press
  run, the expected bits); the A/B standard rests on it. Not yet read on `/lf/lowstate` (confirmed for `/lowstate`).
- [F-205](../findings.md): the holster's grip, 180° about the shaft from the hand-in's, puts Joint6 at the lock past
  the IK's soft limit once the wrist rolls −10° (2.27 rad against 2.12); the holster attempt runs Joint6 alone at 0.98
  of its range (provisional, CPU model).
- [F-204](../findings.md): on the dog, ft6 holds the 10 s stand without rattling: its joint targets change 0.006-0.015
  rad a step against round 4's 0.12-0.23 (provisional, one live run).
- [F-197](../findings.md): fine-tuned on rima's legs, the foot-press policy stops chattering on every leg model and
  stops falling on light legs, but presses less accurately; exported for a stand-only test (provisional, simulation).
- [F-196](../findings.md): the dog's rattle is reproduced in simulation by lighter legs and a longer command delay; on
  the legs it trained on round 4 does not chatter (provisional, simulation against one live run).
- [F-195](../findings.md): fine-tuned for a level base, the foot-press policy stops the crouch (no 10 s stand past
  15°) but loses the dog's own hand presses: tilt penalised everywhere, tip-overs and strikes past 60 N; only while
  standing, over-presses the palm under the foot (42-64 of 144 against round 4's 144) (provisional, simulation).
- [F-194](../findings.md): the foot-press policy sits back 17-25° nose up within 0.3 s of every start in simulation,
  as the dog did in F-188 (0.22 m, ~21 N·m at the knees), and presses from that posture (up to 42°): the live program's
  15° stop fired on the trained posture, not on a fall; supersedes part of F-192 and F-193 (confirmed in simulation).
- [F-193](../findings.md): fine-tuned on the dog's handover starts, the foot-press policy tips on upward presses
  (19-33 of 288) unless training caps the upward force; capped at 16 N it stands, but presses no better than round 4
  where the dog presses and asks the same 0.5 rad first step; exported with a start gate that admits the dog's rest
  (provisional, simulation).
- [F-192](../findings.md): the dog's handover simulated reproduces its kp-80 hold and the direction of its kp-25 rest
  (sinking further); from every stance it gives, round 4's final presses as from its trained stands without a fall, so
  the start stance does not explain F-188 in simulation, and 1 N·m of leg friction costs more (provisional, simulation).
- [F-203](../findings.md): at 5 Hz (8° a command) the D1's pauses drop to 2 in 28 s, but the motion is very choppy;
  10 Hz with its pauses is the better of the two (provisional, one live run).
- [F-202](../findings.md): the D1 ignores funcode 2 without the gripper's angle (so F-201 is retracted); its pauses
  go with the rate of commands it executes: 10 Hz 10-15 in 20 s, 5 Hz 3, 2 Hz none (provisional).
- [F-201](../findings.md): retracted (F-202): without the gripper's angle the arm does not execute the command at all.
- [F-200](../findings.md): under a 10 Hz command stream the D1 itself pauses its angle reports (and its motion)
  0.3-0.7 s about every 2 s -- on its own timestamps, its status stream unbroken -- and not when idle; the stutter left
  in the base swivel (provisional, one run).
- [F-199](../findings.md): the real D1 moves smoothly at 10 Hz when every moving joint steps ≥ ~2.5-4° a command:
  timed paths played by the arm link's clock, joint moves each joint at its own full pace, a stale-report-aware lead;
  Lukas: "the stutters are almost gone" (provisional, one live run).
- [F-198](../findings.md): F-189's grip put the wrist through the shoulder (self-collision is off in the sim);
  1 cm higher (the lever 1 cm longer) Lukas saw none, 8/8 drawn, the unfold through a point above (provisional,
  simulation). F-189 superseded.
- [F-190](../findings.md): the D1's slow moves go stop-start in the model as Lukas sees on the arm (each 10 Hz setpoint
  planned from rest); pacing the holster draw by its fastest joint at 0.6 rad/s cuts the stop-start from 28% to 8% and
  the sequence from 18.5 to 12.3 s with no loss of tracking, where an even speed-up lags (provisional, simulation).
- [F-189](../findings.md): the arm draws the key out of Lukas's holster by the bar's root in 25 of 25 simulated
  attempts once the holster is 41° above level (it was 35°: 70-75 mm of 75 needed), up to 6 mm misplaced; the key
  in it is v3 mirrored; it comes out turned 180° about its shaft from the insertion's grip (provisional, simulation).
- [F-191](../findings.md): under the policy's default targets at kp 25 the real dog rests 8° nose up with the rear thighs
  0.16-0.20 rad below the trained stance, and the policy's first step from there is twice its trained one (one run).
- [F-188](../findings.md): the first live run sat back within 0.24 s of the policy taking over a robot
  sinking through a crouch outside its training starts; the motors delivered the asked PD torque; the handover now
  settles into the trained stance first (one run).
- [F-187](../findings.md): the program that drives the dog's legs with the foot-press policy onto a hand (at most
  5 N): its numpy layer matches training's to 1e-9, its CRC Unitree's, its live sequence runs against a kinematic
  stand-in with no bad command; dry on the dog it walks the sequence (confirmed as software; not run live).
- [F-180](../findings.md): a general force-aware foot-press policy holds 197 / 186 of 288 frozen presses (two seeds) with
  the force's error ~2.3 N, against 100 / 58 position-only, and unlocks 104-107 of 108 keys zero-shot, more than the key
  specialist (provisional, simulation).
- [F-181](../findings.md): what a UniFP-style policy's task layer needs to press -- a goal that follows the foot along the
  normal, a first ramp from the implied force, stall detection near the face, an outer loop split by direction along the
  plate (provisional, simulation).
- [F-182](../findings.md): pushing up at 28 N on a plate above the lifted front-right foot tips the robot; a fine-tune that
  learned to do it falls 5-11 times in 288; the force a layer commands needs a cap by direction (provisional, simulation).

- [F-179](../findings.md): fine-tuned on Lukas's turn, his light lock, a 30 N press limit, realistic legs and sport-mode
  starts, the force-aware key policy unlocks 100-101 of 108 from the dog's stand with nothing past 60 N (max 35 N),
  95-105 across 0-15 ms delay, armature and motor models; fails at 20 ms (65) and 2e-4 armature (2); position-only final
  27-30 (provisional, simulation).

- [F-178](../findings.md): from the dog's measured stand the trained key policies do worse; on Lukas's turn with the
  realistic legs 15-24 of 108, with falls (provisional, simulation).
- [F-177](../findings.md): on a light lock with a soft end the force-aware policy peaks at most 37 N in the trained
  conditions, but 144 N p90 / 342 N on the dog's; a force cap changes nothing (provisional, simulation).
- [F-176](../findings.md): Lukas's turn is horizontal to 45 down (unlock 30); all key policies trained on 45 up to 45
  down; on his turn force s2 unlocks 108 as trained, 52 on the dog's conditions; a fine-tune is needed (provisional).
- [F-186](../findings.md): the keyhole found in the wrist image first: on two hand-held clips (daylight, dim red light) the image alone is right in 1319 of 1338 frames with the lock in view, wrong in 3; with the model's place 20-40 mm off, `find_bezel` image-first is right in 95% of frames where the old 12 mm search is right in 0-2% (provisional, two clips, no arm pose, not on the dog).
- [F-185](../findings.md): through the Go2's front camera (a fisheye fitted on one stand) the door demo's look places the box within 6 mm and opens 16 of 16 in simulation, as the head RealSense did; Isaac renders a USD camera with a 0.35 focal length ~1.35x too large; on the dog the lens fit puts the tag 0.609 m off against a taped 0.61 (provisional, simulation + one stand).
- [F-184](../findings.md): on the real dog the D1 turned Lukas's lever key from 45 degrees up to its stop (+45.6 by its
  fingers' measure, verified by the robot's own check) after a walk up and a lidar refit, in one of three live turns;
  going home it hit the head camera, now cleared by `key_clear` (provisional, one success, dog).

- [F-183](../findings.md): in simulation the D1 alone turns the key 45 degrees up to 45 down against a 5 N lock (11/11
  from the look), up to 11 N (4/4); 1/4 at 14 N, 0/4 at 17; its fingertip measure needs a lag check (provisional,
  simulation).

- [F-175](../findings.md): the shadow loop's stalls were its own 30 s save of the whole record (150 ms per 1,500 steps
  on the dog); a per-second flush of only the new steps costs 2.9 ms at most there and the loop holds 20.4 ms max
  on the PC (confirmed on the PC and by timing on the dog's CPU; the fixed node not yet run on the dog).

- [F-174](../findings.md): the second shadow run on the dog (foot asked up): the observation again as the training
  code's and a still model's; the FR knee's target cycles at 4.3 Hz open loop, as a still model's does; the stand
  changes between sport mode stands (27.8 cm wide this time); the loop stalls 0.18 s at 30 s and 0.4 s at 60 s
  (confirmed on the dog, open loop; the stall's cause not found).

- [F-173](../findings.md): on the real dog the key policy's shadow runner holds 50 Hz and builds the training code's
  observation; pushes confirm the motor order, the IMU's frame and the quaternion's order; the sport stand from the
  encoders is 38.7 x 31.7 cm under 0.32-0.33 m, 2-4 cm off the tape (confirmed, one run on the dog, open loop).

- [F-172](../findings.md): with the key starting 45 degrees down and turned to 90, the key policies fail zero-shot
  (1-16 of 108; the walker fine-tune unlocks 100 but passes 120 N at the vertical stop in 105); layer changes reach 33;
  the new turn needs a fine-tune (provisional, simulation).

- [F-171](../findings.md): the key policy's runner for the dog (numpy) reproduces the training environment to 1e-6
  over 3,596 recorded steps and, as a ROS 2 shadow node, holds 50 Hz and publishes nothing; not run on the dog
  (confirmed, software).

- [F-170](../findings.md): from the sport mode's classic stand at rest (feet 41 x 28 cm), the walker fine-tune unlocks
  96-99 of 108 keys at 0.28-0.32 m and 74 at 0.34 m; the other policies lose more (provisional, simulation).
- [F-169](../findings.md): the force-aware key policies crouch, then rear ~35° nose-up on three legs; the standing knees
  carry 12-14 N·m median, peaks 28-33; the pressing leg at most 19 N·m; the real knee's margin turns on its unmeasured
  reduction (provisional, simulation).
- [F-168](../findings.md): F-165's DCMotor collapse was the stock group's zero leg armature, not its torque limits; the
  torque models move the force-aware results by at most 2 keys, delay and armature by far more, and a realistic
  armature narrows F-166's lead to 169 against 136 of 216 (provisional, simulation).

- [F-167](../findings.md): fine-tuned from the walking policy's recorded stances, the force-aware policy starts from
  them at rest (97 of 108, from 26) but still fails the demo's live handover (0 of 12) (provisional, simulation).
- [F-166](../findings.md): over two seeds the force-aware foot policy unlocks the key in 190 of 216 frozen keys, the
  position-only one in 43; the force-aware one at every lock friction, pushing less hard (provisional, simulation).
- [F-165](../findings.md): in the staged PhysX demo the key policies keep their ranking only on the actuator they
  trained on (DCMotor legs: force-aware 84 -> 3 of 108); neither takes over from the walking policy's stance (a support
  foot ~16 cm off) (provisional, simulation).
- [F-164](../findings.md): the D1 inserts the key without learning from a known box (on the keyway's axis to 1.5 mm,
  seated); 0/12 from the look, its depth 3-11 mm off; the STL is handed the wrong way for the turn; the D1 has no
  torque to spare at the lock's reach (provisional, simulation).
- [F-163](../findings.md): superseded by F-166. The key turned by the front-right foot, seed 1: force-aware 84,
  position-only 20 of 108 frozen keys.
- [F-162](../findings.md): the handle policies' feet were out of place because of the spawn's drop and no stance term;
  a settled spawn and a stance reward keep the worst support foot to 3-4 cm in free space; the position-only stage 1
  came apart from ~600 (provisional, simulation).
- [F-161](../findings.md): the force-aware handle policies held ~12 N because their force law asked for little more
  (corrects F-160's scope) (confirmed, a re-reading of the traces).

- [F-158](../findings.md): letting go, the door script's straight line home runs the fingers into the lever and its 63 mm hub (CPU model, as seen live); backing straight off along the hand's approach clears them (provisional, not run on the dog).
- [F-157](../findings.md): the measured lever does not fit two earlier designs: the L-lip claw's 37.2 mm entry, and the turning demo's 60-80° grasps with the wrist camera (confirmed, geometry).
- [F-156](../findings.md): the combiner door's robot program runs on the dog (sensors, arm feedback, a dry pull, RViz over zenoh); the real dog walks on sport mode, not the simulated walking policy; the dock carries the box's tag, caught by the lidar face's extent (provisional, dry runs only).
- [F-155](../findings.md): folded start and the measured handle (a slot 27 mm off the door): the combiner door demo and the given-pose baseline open 32 of 32 each; the unfold tilts the base under 0.5° (provisional, simulation).
- [F-160](../findings.md): over two seeds of the handle stage the position-only foot policy holds 60 of 288 frozen
  handles' latches against the force-aware one's 15; the force-aware policies push ~12 N and slip, the position-only
  ones push to ~60 N and pass 120 N more often; seeds spread threefold (provisional, simulation).
- [F-159](../findings.md): superseded by F-160. The Go2's FR foot, trained UniFP-style, stands on three legs and tracks
  goals at body height; seed 1 of the handle stage held 17 / 12 of 144; a 2 cm round bar and force once on it are the
  limits; the wrist camera's view of the press is blocked by the D1's own fingers.
- [F-143](../findings.md): the real box's handle is out of the lying dog's reach; standing, everything plans on the CPU model (provisional).
- [F-144](../findings.md): the shut fingers hid the bigger tag; aimed looks fixed it (provisional, simulation).
- [F-145](../findings.md): the standing base tilts 1.3–1.5° under the arm and the jaws miss the bar by ~20 mm; no door opened in 9 attempts (provisional, simulation).
- [F-146](../findings.md): UniFP walks the unlocked door 90° open round the hinge, no falls in 116 attempts; the friction grip holds in 15 of 32 (provisional, simulation).
- [F-147](../findings.md): the lidar and wrist camera, folded into the training robot, cost the reach policy 1.9 mm median, 3.7 mm p90, no falls (provisional).
- [F-148](../findings.md): the CAD-length checkpoints were scored on an arm URDF not in the repository; 58400 gives 1.50 cm on the committed one, not 1.40 (confirmed).
- [F-154](../findings.md): the combiner door demo (head RealSense + MID-360, P2Dingo's docking method and walking policy) opens 31 of 32; box to 7.4 mm and 0.34°, door read to 0.5° median, verified open agreeing 32/32 (provisional, simulation).
- [F-153](../findings.md): on P2Dingo's walking policy the decoupled door pull opens 32 of 32, against 9 on the 2024 policy (which leans after stopping) and UniFP + wrist's 15 (provisional, simulation).
- [F-152](../findings.md): superseded by F-153 as the baseline's number. The decoupled baseline with its arm IK bounded for 10 Hz opens 9 of 32 on two seeds, against 15 with an unbounded search (which caused the viewer's hitches) and UniFP + wrist's 15 (provisional, simulation).
- [F-151](../findings.md): superseded by F-152. A decoupled baseline (walking policy + arm IK) opens 7 of the door pull's 16 against UniFP's 8 (with the wrist servo) and 4 (alone); its arm is better, its walk much worse.
- [F-150](../findings.md): the door pull's arm folds because UniFP commands only the tip's position; alone it folds in 13 of 16 walks, with the wrist servo only after the handle is lost (provisional, simulation).
- [F-149](../findings.md): switched on folded, the reach policy unfolds and reaches every goal without a fall, but at low or far goals it swings the hand into the Go2's head first (11 of 27) and pitches the base up to 26° (provisional, simulation).

## Issues and risks

- `tests/test_payload.py::TrunkParts::test_the_meshes_lie_inside_their_colliders` fails on HEAD (1149 lidar-mount
  vertices outside both collider boxes) since 81b9c4a's redrawn mount; found running the full suite on 2026-10-20,
  not caused by the leg task. The other 724 tests pass (7 skipped). Still the only failure on 2026-10-21 after 9136aba
  and the key work: 788 tests, 1 failure, 7 skipped; and after the leg-model study: 793, the same 1 failure, 7 skipped. After the shadow runner: 803, the same 1 failure, 7 skipped. After the fine-tune's code (2026-10-21 evening): 804, the same 1 failure, 7 skipped. After the foot-press task
  (2026-10-22): 827, the same 1 failure, 7 skipped. After the foot-press program (2026-10-23): 943, the same 1
  failure, 15 skipped.
- The shadow runner on the dog stalls once at ~30 s (0.18 s) and once at ~60 s (0.36-0.40 s) after its first LowState,
  in both runs of 2026-10-21 (F-174). Not diagnosed. Any program that commands the legs needs this found first, and a
  watchdog for a missed step either way. **Found and fixed the same day (F-175):** the node saved its whole record
  inside the loop every 30 s; it now flushes one second's steps (2.9 ms max on the dog). To confirm on the dog with
  `d1update` and a run over 60 s. A watchdog for a missed step is still needed in any program that commands the legs.
- The leg-task side study used ~10 h of the shared GPU on 2026-10-19/20 (Lukas's allocation of a day).
- The key work used the GPU 00:47-10:10 on 2026-10-21 (Lukas's 10 h). The key's stage 2 runs at ~6.2 s an iteration
  (8 sub-steps of the two-box contact; ~10 s with a demo beside it), so both seeds' stage 2 were cut to 1000 iterations.
- The head camera's renderer does not fit beside two 2048-environment training runs on the 12 GB GPU (two demo runs
  died of it); `--given_box` develops the rest without it. One 4096-environment leg run takes 6.6 GB.
- `pgrep -f <script>` matches the shell that runs it: killing by pattern killed my own shell twice (2026-10-20 and
  -21). The queue and its helpers now write and use pid files.
- The foot-press work used the GPU 22:03 on 2026-10-21 to 12:50 on 2026-10-22 (Lukas's day): seven training starts, four
  of them stopped within an hour on a design flaw each, and ~150 frozen evaluations. Its queues ran detached on pid
  files with a watcher reporting at most every 55 min; one `pkill -f` (04:05) matched the calling shell again and cut that
  command short, without losing a run.
- **The foot-press plant (F-180-F-182):** the plates are penalty contacts on the foot sphere, the knee and the head; the
  thighs and calves pass through them. A sprung plate pressed past its travel and the 2 cm overlap cap leaves the foot's
  centre behind the slab, where its back pushes it on through (the face-only plant lost contact there instead).
- The leg-model study used the GPU 09:40-11:27 on 2026-10-21 (eval only, three at a time, ~2.4 GB). Its first queue
  read its job list on the same stdin its jobs inherited; it was replaced after three jobs by one that reads on its own
  descriptor and gives every job /dev/null (no job was lost).
- **Sim-to-real (F-168-F-170):** the leg policies trained with UniFP's leg armature (2e-4 kg·m²), no command delay, no
  observation noise and a fixed mass and ground friction; the real Go2's armature, delay and knee reduction are not
  measured. The decoupled demos' legs (and P2Dingo's walking policy in them) run the stock group's zero armature; what
  armature the walking policy trained with (Isaac Lab's own Go2 asset) was not checked.

## Next week

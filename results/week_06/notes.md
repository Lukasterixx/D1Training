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

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

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
  and the key work: 788 tests, 1 failure, 7 skipped; and after the leg-model study: 793, the same 1 failure, 7 skipped. After the shadow runner: 803, the same 1 failure, 7 skipped.
- The leg-task side study used ~10 h of the shared GPU on 2026-10-19/20 (Lukas's allocation of a day).
- The key work used the GPU 00:47-10:10 on 2026-10-21 (Lukas's 10 h). The key's stage 2 runs at ~6.2 s an iteration
  (8 sub-steps of the two-box contact; ~10 s with a demo beside it), so both seeds' stage 2 were cut to 1000 iterations.
- The head camera's renderer does not fit beside two 2048-environment training runs on the 12 GB GPU (two demo runs
  died of it); `--given_box` develops the rest without it. One 4096-environment leg run takes 6.6 GB.
- `pgrep -f <script>` matches the shell that runs it: killing by pattern killed my own shell twice (2026-10-20 and
  -21). The queue and its helpers now write and use pid files.
- The leg-model study used the GPU 09:40-11:27 on 2026-10-21 (eval only, three at a time, ~2.4 GB). Its first queue
  read its job list on the same stdin its jobs inherited; it was replaced after three jobs by one that reads on its own
  descriptor and gives every job /dev/null (no job was lost).
- **Sim-to-real (F-168-F-170):** the leg policies trained with UniFP's leg armature (2e-4 kg·m²), no command delay, no
  observation noise and a fixed mass and ground friction; the real Go2's armature, delay and knee reduction are not
  measured. The decoupled demos' legs (and P2Dingo's walking policy in them) run the stock group's zero armature; what
  armature the walking policy trained with (Isaac Lab's own Go2 asset) was not checked.

## Next week

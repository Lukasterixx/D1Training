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

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-158](../findings.md): letting go, the door script's straight line home runs the fingers into the lever and its 63 mm hub (CPU model, as seen live); backing straight off along the hand's approach clears them (provisional, not run on the dog).
- [F-157](../findings.md): the measured lever does not fit two earlier designs: the L-lip claw's 37.2 mm entry, and the turning demo's 60-80° grasps with the wrist camera (confirmed, geometry).
- [F-156](../findings.md): the combiner door's robot program runs on the dog (sensors, arm feedback, a dry pull, RViz over zenoh); the real dog walks on sport mode, not the simulated walking policy; the dock carries the box's tag, caught by the lidar face's extent (provisional, dry runs only).
- [F-155](../findings.md): folded start and the measured handle (a slot 27 mm off the door): the combiner door demo and the given-pose baseline open 32 of 32 each; the unfold tilts the base under 0.5° (provisional, simulation).
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

## Next week

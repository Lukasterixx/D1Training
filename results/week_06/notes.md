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

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- [F-143](../findings.md): the real box's handle is out of the lying dog's reach; standing, everything plans on the CPU model (provisional).
- [F-144](../findings.md): the shut fingers hid the bigger tag; aimed looks fixed it (provisional, simulation).
- [F-145](../findings.md): the standing base tilts 1.3–1.5° under the arm and the jaws miss the bar by ~20 mm; no door opened in 9 attempts (provisional, simulation).
- [F-146](../findings.md): UniFP walks the unlocked door 90° open round the hinge, no falls in 116 attempts; the friction grip holds in 15 of 32 (provisional, simulation).
- [F-147](../findings.md): the lidar and wrist camera, folded into the training robot, cost the reach policy 1.9 mm median, 3.7 mm p90, no falls (provisional).
- [F-148](../findings.md): the CAD-length checkpoints were scored on an arm URDF not in the repository; 58400 gives 1.50 cm on the committed one, not 1.40 (confirmed).
- [F-152](../findings.md): the decoupled baseline with its arm IK bounded for 10 Hz opens 9 of 32 on two seeds, against 15 with an unbounded search (which caused the viewer's hitches) and UniFP + wrist's 15 (provisional, simulation).
- [F-151](../findings.md): superseded by F-152. A decoupled baseline (walking policy + arm IK) opens 7 of the door pull's 16 against UniFP's 8 (with the wrist servo) and 4 (alone); its arm is better, its walk much worse.
- [F-150](../findings.md): the door pull's arm folds because UniFP commands only the tip's position; alone it folds in 13 of 16 walks, with the wrist servo only after the handle is lost (provisional, simulation).
- [F-149](../findings.md): switched on folded, the reach policy unfolds and reaches every goal without a fall, but at low or far goals it swings the hand into the Go2's head first (11 of 27) and pitches the base up to 26° (provisional, simulation).

## Issues and risks

## Next week

# description/

What RViz needs to draw the robot, and where the robot's mount and expansion dock come from. Nothing in here
changes the simulation's physics; the dock's meshes are also drawn in the sim, visual only (`weld.py`, `dock.py`).

| File | Role |
| --- | --- |
| `go2_d1.urdf` | the welded Go2+D1, with the expansion dock drawn, for `robot_state_publisher` and the cup console. **Generated** |
| `build_go2_d1_urdf.py` | regenerates the above from its sources |
| `meshes/go2/*.dae` | the Go2's visual meshes, copied from P2Dingo |
| `meshes/go2/expansion_dock.stl`, `expansion_dock_rails.stl` | the Go2 EDU's Jetson expansion dock and the two rails the D1 stands on, in `base_link`'s frame. **Generated** from Unitree's CAD by `dock_from_cad.py` |
| `mount_from_cad.py` | measures the arm mount (`mount.py`, F-117) from the same CAD |
| `dock_from_cad.py` | tessellates the dock and rails out of the CAD and registers them to `base_link` |
| `go2_d1.rviz` | RViz layout: robot model, L1 cloud, grid |

The arm's meshes are not duplicated here — the URDF points at the existing
`d1_arm/meshes/*.STL`, which is the same set the sim imports.

## Why the URDF is generated, not written

There is no single description of this robot to copy. The Go2 half comes from
P2Dingo (`Isaac/go2_ws/src/go2_control_cpp/config/go2.urdf`), or the same file as
Go2RemoteConnection carries it (`~/go2_ws/Go2RemoteConnection/description/go2/go2.urdf`,
whose mesh paths differ; only the file names are kept). The D1 half comes from
this repo's `d1_arm/d1.urdf`, and the joint between them is `weld.py`'s
`arm_mount_joint` at `mount.ARM_MOUNT_POS_M` (the measured mount on the expansion
dock's rails, F-117). On 2026-09-26 the file was regenerated from the
Go2RemoteConnection copy. Before the dock was added, that gave output byte-identical
to the committed file, so the two sources are the same file. Merging by hand once would
leave the sources silently drifting apart; `build_go2_d1_urdf.py` makes the merge
re-runnable and checks the result:

```bash
python3 description/build_go2_d1_urdf.py [path/to/P2Dingo | path/to/go2.urdf]   # default: the first that exists
```

It prints the movable joint names, which **must** match what the sim publishes on
`/joint_states` — `sim.py` prints the articulation's joints at startup, so the
two lists can be diffed directly. A name that differs is not an error anywhere;
the link just never moves in RViz.

Three deliberate edits along the way:

- **The D1's `base_link` is renamed `d1_base_link`.** It collides with the Go2's
  root otherwise.
- **The Go2's `radar` link is dropped.** That is the L1's mount, and the sim
  already publishes `base_link -> utlidar_lidar` (see `ros2.py`) so the cloud is
  placeable without `robot_state_publisher` running at all. Keeping the link too
  would give that transform two publishers, which is a TF authority conflict.
- **The expansion dock is added** as a link `expansion_dock` on a fixed joint
  from `base_link` at the origin (its meshes are already in that frame), with two
  visuals, the dock and its rails, and no collision or inertial. Its colours
  come from `dock.py`, which is where `weld.py` takes them from too. They are a
  choice, not the robot's.
- **All `<inertial>` blocks are dropped.** This makes the file unusable by any
  importer that reads inertials for simulation: Isaac Gym derives mass from
  collision geometry and a density default instead, and the robot NaNs on its
  first step (F-079). `unifp_go2d1/build_asset.py` rebuilds `weld.py`'s mass
  model into its own generated copy; anything else must too. `weld.py` owns the mass model; a
  second set of numbers here would drift out of step with it and read as spec.

## Mesh paths

The URDF addresses its meshes as `package://d1_training/...`, which is a
placeholder — this repo is not an ament package, so nothing resolves it.
`run_rviz.sh` rewrites the prefix to `file://<repo>/` when it starts, which is
also why the URDF it feeds `robot_state_publisher` lives under `$TMPDIR`.

## The expansion dock

The Go2 EDU's Jetson expansion dock sits in the trunk's top channel, from z = 57 mm; the D1 stands on the dock's two
T-slot rails, whose tops are at 85.5 mm (F-117). Both come from Unitree's CAD, "GO2 simplified, with expansion dock"
(a STEP file; where to get it is in `mount_from_cad.py`, provenance in `third_party/unitree_go2_cad/NOTICE.md`).
`dock_from_cad.py` regenerates the two meshes. It needs Open CASCADE's tessellator, which the Isaac environment
does not have:

```bash
python3 -m venv /tmp/cad && /tmp/cad/bin/pip install cascadio trimesh
/tmp/cad/bin/python description/dock_from_cad.py <path to GO2简化-带拓展坞-0911.stp>
python3 description/build_go2_d1_urdf.py
```

It uses the same registration as the mount (`mount_from_cad.register`, from the hip motors' axes) and tessellates to
0.5 mm and 0.5 rad, which is accurate enough for a drawing, not a measurement. It picks the dock and rails by where they are, and
fails unless the rails' tops are at the mount's height. The drawing shows what F-117 left open: in the CAD, the D1's
front screws land just past the rails' front end. `tests/test_dock.py` checks the meshes' placement and that the
URDF only draws them. `demos/cup/run_camera_body_view.sh --look_at dock` renders them in the sim.

# description/

What RViz needs to draw the robot. Nothing in here affects the simulation.

| File | Role |
| --- | --- |
| `go2_d1.urdf` | the welded Go2+D1, for `robot_state_publisher`. **Generated** |
| `build_go2_d1_urdf.py` | regenerates the above from its two sources |
| `meshes/go2/*.dae` | the Go2's visual meshes, copied from P2Dingo |
| `go2_d1.rviz` | RViz layout: robot model, L1 cloud, grid |

The arm's meshes are not duplicated here — the URDF points at the existing
`d1_arm/meshes/*.STL`, which is the same set the sim imports.

## Why the URDF is generated, not written

There is no single description of this robot to copy. The Go2 half comes from
P2Dingo (`Isaac/go2_ws/src/go2_control_cpp/config/go2.urdf`), the D1 half from
this repo's `d1_arm/d1.urdf`, and the joint between them is `weld.py`'s
`arm_mount_joint` at `ARM_MOUNT_Z`. Merging by hand once would leave three
sources silently drifting apart; `build_go2_d1_urdf.py` makes the merge
re-runnable and checks the result:

```bash
python3 description/build_go2_d1_urdf.py [path/to/P2Dingo]   # default ~/P2Dingo
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
- **All `<inertial>` blocks are dropped.** `weld.py` owns the mass model; a
  second set of numbers here would drift out of step with it and read as spec.

## Mesh paths

The URDF addresses its meshes as `package://d1_training/...`, which is a
placeholder — this repo is not an ament package, so nothing resolves it.
`run_rviz.sh` rewrites the prefix to `file://<repo>/` when it starts, which is
also why the URDF it feeds `robot_state_publisher` lives under `$TMPDIR`.

# P2Dingo

Source: Lukas's own repository, `~/P2Dingo` (a Go2 inspection robot; no licence file, same author as this one).
Brought in for the combiner door demo (`demos/combiner_door/`) and the decoupled door pull's legs (`demos/unifp/decoupled.py`).

| This repository | From P2Dingo | P2Dingo revision | sha256 |
| --- | --- | --- | --- |
| `checkpoints/p2dingo_go2_gait_v7_model_7000.pt` | `Isaac/go2_omniverse/logs/rsl_rl/go2_p2dingo_gait/2026-09-28_21-58-09_rough_v7_a1/model_7000.pt`, the walking policy its sim installs (`agent_cfg.py`, "go2_p2dingo_gait" v7) | `8b69d56` (committed) | `6f38d3185f5d1c3545bb7c549b51a7a6169e1fcc51a2facce4b25150bf30a24c` |
| `demos/unifp/p2dingo_gait_clock.py` | `Isaac/go2_omniverse/training/go2_p2dingo/gait_clock.py`, unchanged: the clock that policy observes. Any change here and the policy sees a different clock | `8b69d56` (committed) | `20f2a161eb433d14d296f271214b4b7f41031d56cddf3df03ab3a6a73b48337a` |
| `demos/combiner_door/assets/realsense_mount_grey.stl`, `realsense_mount_white.stl` | `Isaac/go2_ws/src/go2_control_cpp/meshes/`, the head RealSense bracket (Fusion "Go2 v115", exported 2026-10-16) | **working tree at `c677e54`, uncommitted there** | `bcaf6dfd…7e5929`, `1543e0c1…d92d2d` |
| `demos/combiner_door/head_camera.py` `COLOUR_SENSOR_BASE_M` | `Isaac/go2_omniverse/realsense.py` `DEFAULT_MOUNT_BASE` (0.3392, 0.03215, 0.0927) and `meshes/realsense_mount.NOTICE.md`'s measurement | **working tree at `c677e54`, uncommitted there**; the committed value is the older guess (0.32, 0.0325, 0.090) | -- |
| `demos/combiner_door/mid360.py` `face_line` | `Isaac/go2_ws/src/go2_control_cpp/include/go2_control_cpp/dock_wall.hpp` `fitOnce` and its second window, ported from C++ to numpy; the window and inlier band changed for a door and raw returns (the module's docstring) | `c677e54` | -- |

The perception method itself is P2Dingo's docking fix (`src/dock_detector_node.cpp`): the AprilTag for the position, the
lidar's line round it for the heading.

The two rows marked uncommitted cannot be reproduced from P2Dingo's history; the hashes above are what was copied on
2026-10-20.

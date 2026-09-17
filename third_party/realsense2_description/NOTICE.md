# RealSense D435 housing mesh (realsense2_description)

Intel's own CAD mesh of the D435 aluminium case, from the ROS 2 package `realsense2_description`
(<https://github.com/IntelRealSense/realsense-ros>), maintained by the LibRealSense ROS Team.
Licensed under the Apache License 2.0 (full text in `LICENCE` in this folder).

Obtained 2026-09-17 from the ROS 2 apt repository without installing it:

```
apt-get download ros-kilted-realsense2-description     # 4.58.3-1noble.20260813.061323
                                                       # sha256 3aa12a48…31083c
dpkg-deb -x ros-kilted-realsense2-description_*.deb rsdesc
# rsdesc/opt/ros/kilted/share/realsense2_description/meshes/d435.dae
#   sha256 42f3b66f47a1f8f425a2e4dc07c1d9c283183167d8441f520a15623d98f9bf78
```

The D435i shares this case: `urdf/_d435i.urdf.xacro` builds the D435i by including `_d435.urdf.xacro`
unchanged and adding the IMU frames, so `d435.dae` is the body of the camera on the bench
(D435I 238222076237).

| This repository | Source |
| --- | --- |
| `pick_demo/assets/realsense/d435_housing.ply` | `meshes/d435.dae`, converted to binary PLY (sha256 `a1a4abe2…51641`) |
| — (read, not copied) | `urdf/_d435.urdf.xacro`, the frame offsets quoted in `pick_demo/camera_body.py` |

**Changes.** The DAE's 18 sub-meshes carry no distinct materials (all default 191/191/191), so the
conversion merges them into one mesh and drops the material. Vertices and triangles are otherwise
untouched, in the DAE's own frame and metres — the bounding box is identical before and after
(`[-0.044914, -0.0125, -0.025055]` to `[0.045, 0.0125, 0.0]`, 231186 triangles, 154149 vertices).
PLY rather than DAE because reading COLLADA needs `pycollada`, which the Isaac environment does not
have, while PLY is native to `trimesh`, which it does. To regenerate:

```
python -c "import trimesh; m = trimesh.load('d435.dae').to_geometry(); m.merge_vertices(); \
           m.export('d435_housing.ply', encoding='binary')"      # needs trimesh + pycollada
```

The placement of this mesh in the camera's colour optical frame is derived in
`pick_demo/camera_body.py` from `_d435.urdf.xacro`'s own offsets, and checked against the mesh's lens
barrels in `tests/test_pick_demo.py`.

The material is provided as-is, with no warranties, as section 7 of the licence states.

"""The RealSense D435i on the Go2's head, as P2Dingo mounts it: where it is, what it sees with, and its bracket.

The combiner door demo looks for the door's AprilTag with this camera rather than the wrist one. It is fixed to the
trunk, so it needs no arm move to aim it; the robot stands square to the door and the tag is straight ahead.

**The mount** is P2Dingo's (`Isaac/go2_omniverse/realsense.py`, `DEFAULT_MOUNT_BASE`; its
`meshes/realsense_mount.NOTICE.md` has the measurement). The bracket is Lukas's Fusion design "Go2 v115"
(exported 2026-10-16), in the same design frame as D1Training's lidar mount. There the colour sensor is at
`base_link` (0.3392, 0.03215, 0.0927) m, looking straight ahead, no tilt: the case's front plate at x 0.3435, the sensor
4.3 mm behind it (the xacro's glass offsets), at the windows' height, `SCREW_TO_COLOUR_M` left of the case's middle.
P2Dingo's *committed* mount is an older guess, (0.32, 0.0325, 0.090); the CAD-measured one, used here, is in its working
tree with the bracket meshes (`third_party/p2dingo/NOTICE.md`). The two `base_link` registrations of that CAD, P2Dingo's
and this repository's lidar mount (`description/mid360_from_cad.py`), agree to 0.5 mm.

**The camera model** is this repository's measured D435i #238222076237 at 640x480 (fx 607.11 px, the calibration P2Dingo
quotes too), rendered with square pixels about the image centre as Isaac draws them (F-070): the principal point is
(320, 240), not the device's (323.0, 254.3).

Numpy at import; `camera_cfg`, `build_bracket_usd` and `build_case_usd` import Isaac or pxr when called.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ASSETS = Path(__file__).resolve().parent / "assets"
CALIBRATION = ROOT / "demos/cup/pick_demo/assets/calibration/d435i_238222076237_640x480.json"

#: From `_d435.urdf.xacro` (datasheet Rev 007): the depth origin 17.5 mm along the case from the tripod screw, the colour
#: sensor a nominal 15 mm further. P2Dingo's `SCREW_TO_COLOUR_M`.
SCREW_TO_COLOUR_M = 0.0175 + 0.015
#: The colour sensor in the Go2's `base_link`, metres: P2Dingo's `DEFAULT_MOUNT_BASE` (the screw insert at y -0.00035).
COLOUR_SENSOR_BASE_M = (0.3392, SCREW_TO_COLOUR_M - 0.00035, 0.0927)
#: The colour optical frame (x right, y down, z forward) in `base_link`, looking along +x with no tilt. P2Dingo's
#: `LINK_TO_OPTICAL_XYZW` (-0.5, 0.5, -0.5, 0.5), written w first.
OPTICAL_QUAT_BASE_WXYZ = (0.5, -0.5, 0.5, -0.5)
#: Near clip past the drawn case, as P2Dingo and the wrist camera use: a renderer's eye can sit inside the case (F-054).
NEAR_CLIP_M = 0.05
FAR_CLIP_M = 10.0
#: The bracket, drawn under the trunk in `base_link` (P2Dingo's `meshes/realsense_mount_{grey,white}.stl`), with
#: P2Dingo's colours: the design's default grey and its white parts.
BRACKET_MESHES = (("realsense_mount_grey", "realsense_mount_grey.stl", (0.61, 0.61, 0.61)),
                  ("realsense_mount_white", "realsense_mount_white.stl", (0.94, 0.94, 0.94)))
#: The case and its bracket as one box in `base_link` ((x), (y), (z) bounds, m), for keeping the arm off them: the
#: bracket's STL bounds (x 0.2877-0.3195, z 0.0657-0.0978) and the D435's 90 x 25 x 25 mm case behind its front plate
#: (x 0.3435), centred on the screw insert and the colour sensor's height. Past the trunk proxy's front (x 0.30) and
#: above its top (z 0.06): not in the arm's clearance check (`position_only.workspace.clear_of_body`), and the key's
#: way home after its turn put the fingertips through it on the dog (2026-10-21).
CLEARANCE_BOX_BASE_M = ((0.2877, 0.3435), (-0.0453, 0.0447), (0.0657, 0.1052))


def quat_to_matrix(q) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def optical_pose_base() -> np.ndarray:
    """The colour optical frame in `base_link`, 4x4."""
    pose = np.eye(4)
    pose[:3, :3] = quat_to_matrix(OPTICAL_QUAT_BASE_WXYZ)
    pose[:3, 3] = COLOUR_SENSOR_BASE_M
    return pose


def camera_model():
    """The measured D435i at 640x480, as Isaac renders it: its focal lengths, the image centre as principal point."""
    from dataclasses import replace

    from demos.cup.pick_demo.realsense import load_camera_model

    measured = load_camera_model(CALIBRATION)
    return replace(measured, cx=measured.width / 2.0, cy=measured.height / 2.0,
                   source=measured.source + "; principal point moved to the image centre, as rendered (F-070)")


def camera_cfg(model, prim_path: str = "/World/envs/env_.*/Robot/base/head_cam"):
    """A tiled RGB camera at the mount, for every environment at once."""
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import TiledCameraCfg

    return TiledCameraCfg(
        prim_path=prim_path,
        update_period=0.0,
        width=model.width, height=model.height,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
            intrinsic_matrix=model.intrinsic_matrix.flatten().tolist(), width=model.width, height=model.height,
            clipping_range=(NEAR_CLIP_M, FAR_CLIP_M)),
        offset=TiledCameraCfg.OffsetCfg(pos=COLOUR_SENSOR_BASE_M, rot=OPTICAL_QUAT_BASE_WXYZ, convention="ros"),
    )


def build_bracket_usd(out_dir) -> str:
    """Write (or reuse) the bracket as a USD in `base_link`, visual only; returns its path."""
    from pxr import Gf, Usd, UsdGeom, Vt

    from dock import read_binary_stl

    files = [ASSETS / name for _, name, _ in BRACKET_MESHES]
    key = hashlib.sha256(b"".join(f.read_bytes() for f in files) + json.dumps(BRACKET_MESHES).encode()).hexdigest()[:12]
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"head_bracket_{key}.usda"
    if path.is_file():
        return str(path)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/HeadBracket")
    stage.SetDefaultPrim(root.GetPrim())
    for name, filename, rgb in BRACKET_MESHES:
        tri = read_binary_stl(ASSETS / filename)
        points, index = np.unique(tri.reshape(-1, 3), axis=0, return_inverse=True)
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        mesh = UsdGeom.Mesh.Define(stage, f"/HeadBracket/{name}")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(tri), 3, dtype=np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(index.reshape(-1).astype(np.int32)))
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.repeat(normals, 3, axis=0).astype(np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))])
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    stage.GetRootLayer().Save()
    return str(path)


def build_case_usd(out_dir) -> str:
    """The D435 case the wrist camera draws (Intel's CAD, `camera_asset`), authored in the colour optical frame."""
    from demos.cup.pick_demo.camera_asset import build_camera_usd

    return build_camera_usd(out_dir)["usd_path"]

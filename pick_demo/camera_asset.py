"""Turn Intel's D435 case mesh into the visual USD drawn at the wrist mount.

The source is `pick_demo/assets/realsense/d435_housing.ply`: the `d435.dae` shipped in the ROS package
`realsense2_description`, converted to PLY -- provenance and licence in
`third_party/realsense2_description/NOTICE.md`. `camera_body.MESH_TO_OPTICAL` carries the transform
that puts it in the colour optical frame, derived from that package's own `_d435.urdf.xacro`.

Why rebuild rather than reference the PLY. Isaac's `UsdFileCfg` wants a USD, the mesh has to be baked
into the optical frame rather than carrying a transform an importer might reinterpret, and one
triangle group has to come out first -- see below. Baking makes the asset mean the same thing
everywhere, which is the same reason `cup_asset` rebuilds the cup.

**The lens element is removed.** A real colour sensor looks out through the lens stacked in front of
it; the simulated camera is a pinhole *at the sensor plane*, so that lens is geometry sitting on its
optical axis 0.5 mm away. Rendered, it fills the frame. `LENS_CLEARANCE_TAN` is the tangent of the
half-angle cleared around the axis: 0.90, comfortably past the widest colour frustum of any model here
(the D455 preset's 0.833 across), so one asset serves them all. It takes out under 2000 of 231186
triangles, all between 0.48 and 0.74 mm in front of the sensor -- the lens element and nothing else. The
barrel around it, the aperture in the front plate and the whole rest of the case are untouched, so the
camera still looks like a camera and `camera_body.view_obstruction` reports a clear aperture.

That is enough only when the rendered camera is where the mount puts it. F-052 says it is not -- the
renderer's eye sits about 9 mm below and 6 mm behind the optical origin, inside the case -- and no
amount of carving fixes that without destroying the model: clearing a cone wide enough for an eye
anywhere within 11 mm removes 28% of the mesh. The body is drawn where the bracket will hold the
camera; making the renderer agree is F-052's business, not this file's.

The asset is **visual only**: no collider, no mass, no rigid-body properties. It cannot change the
physics the pick was measured against.

Needs `pxr`, so it runs inside the Isaac app, after `AppLauncher`.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from . import camera_body

# The widest colour frustum of anything in `camera.CAMERAS` is the D455 preset's: 0.833 of the depth
# across, 0.625 down. Clearing 0.90 leaves every model here an open aperture with room to spare, and
# still only reaches the lens element -- the barrel around it starts 2.85 mm off the axis.
LENS_CLEARANCE_TAN = 0.90
BUILDER_VERSION = 1


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def carve_lens(points: np.ndarray, faces: np.ndarray, tan: float = LENS_CLEARANCE_TAN):
    """Drop every triangle inside the clearance cone. Returns (faces_kept, dropped_count).

    A triangle is inside if any of it lies at positive depth and its x and y spans both overlap the
    cone measured at the far end of its own depth range, which is where the cone is widest. Same
    conservative test `camera_body.view_obstruction` uses, so what this removes is exactly what that
    would report.
    """
    tri = points[faces]
    zmax = tri[:, :, 2].max(axis=1)
    half = tan * zmax
    inside = ((zmax > 0.0)
              & (tri[:, :, 0].min(axis=1) <= half) & (tri[:, :, 0].max(axis=1) >= -half)
              & (tri[:, :, 1].min(axis=1) <= half) & (tri[:, :, 1].max(axis=1) >= -half))
    return faces[~inside], int(inside.sum())


def build_camera_usd(out_dir, source=None, colour=(0.62, 0.63, 0.65), roughness: float = 0.35,
                     tan: float = LENS_CLEARANCE_TAN) -> dict:
    """Write (or reuse) the visual camera asset. Returns its path and the numbers that define it.

    The asset's frame **is** the colour optical frame: x right, y down, z forward, origin at the
    colour sensor. Spawn it with the mount's own pose and it lands where the camera is.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

    source = Path(source or camera_body.MESH_PATH).resolve()
    params = {"source_sha256": _sha256(source), "colour": list(colour), "roughness": roughness,
              "clearance_tan": tan, "transform": np.asarray(camera_body.MESH_TO_OPTICAL).round(9).tolist(),
              "builder": BUILDER_VERSION}
    key = hashlib.sha256(json.dumps(params, sort_keys=True).encode()).hexdigest()[:12]
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"d435_{key}.usd"
    info_path = out_dir / f"d435_{key}.json"
    if out_path.is_file() and info_path.is_file():
        return json.loads(info_path.read_text())

    points, faces = camera_body.mesh_optical()
    kept, dropped = carve_lens(points, faces, tan)
    lo, hi = points[np.unique(kept)].min(axis=0), points[np.unique(kept)].max(axis=0)

    if out_path.exists():
        out_path.unlink()
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/RealSenseD435i")
    stage.SetDefaultPrim(root.GetPrim())

    look = UsdShade.Material.Define(stage, "/RealSenseD435i/Looks/Aluminium")
    shader = UsdShade.Shader.Define(stage, "/RealSenseD435i/Looks/Aluminium/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.85)
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    mesh = UsdGeom.Mesh.Define(stage, "/RealSenseD435i/Visual")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(kept), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(kept.astype(np.int32).ravel()))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(
        np.array([points.min(axis=0), points.max(axis=0)], dtype=np.float32)))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(look)
    stage.GetRootLayer().Save()

    info = {
        "usd_path": str(out_path), "source": str(source), **params,
        "triangles": int(len(kept)), "triangles_removed": dropped, "vertices": int(len(points)),
        "bounds_optical_m": [[round(float(v), 6) for v in lo], [round(float(v), 6) for v in hi]],
        "frame": "colour optical: x right, y down, z forward, origin at the colour sensor",
        "note": ("Intel's d435.dae (realsense2_description, Apache-2.0), registered by "
                 "_d435.urdf.xacro's offsets, with the colour lens element removed so a pinhole "
                 "camera at the sensor plane has a clear aperture. Visual only."),
    }
    info_path.write_text(json.dumps(info, indent=2) + "\n")
    return info


__all__ = ["build_camera_usd", "carve_lens", "LENS_CLEARANCE_TAN"]

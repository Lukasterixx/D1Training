"""Turn a downloaded cup model into a graspable rigid body: metres, Z up, sized, with simple colliders.

The source is `pick_demo/assets/High-Resolution_3D_Cup_Model_FBX.usdz`: "High-Resolution 3D Cup Model (FBX)" by
fayazg1aa on Sketchfab (model 9030ed8db34a4110b6d3c508a9d57807), CC BY 4.0 -- attribution and the changes made here
are in `third_party/sketchfab_cup/NOTICE.md`. As shipped it is Y up, in centimetre
units, ~2 m across with its stage scale applied, and a mug whose height equals its diameter with the
handle along +x.

Why rebuild rather than reference it. Referencing a layer with a different metersPerUnit and up axis
leaves the correction to whichever unit-fixing extension the Kit app happens to load; baking the
transform into the points makes the asset mean the same thing everywhere. And the ~620k-point visual
mesh is no collider: the cup collides as a solid cylinder plus a box for the handle, which is what a
side-on pincer grasp touches and what PhysX handles stably. A grasp inside the cup would need the real
hollow shape.

The cup is scaled independently in diameter and height: the top-down grasp the lying robot can reach
needs a cup ~10 cm tall, and the open jaws are 7.7 cm, so the mug's own proportions do not fit both.

Needs `pxr`, so it runs inside the Isaac app, after `AppLauncher`.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_cup_usd(source: str | Path, out_dir: str | Path, diameter_m: float = 0.055, height_m: float = 0.10,
                  mass_kg: float = 0.12, colour=(0.92, 0.92, 0.90), roughness: float = 0.35,
                  static_friction: float = 0.9, dynamic_friction: float = 0.7) -> dict:
    """Write (or reuse) the rigid cup asset. Returns its path and the numbers that define it.

    The cup's frame has its origin on the axis at the bottom, z up, handle along +x.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    source = Path(source).resolve()
    params = {"source_sha256": _sha256(source), "diameter_m": diameter_m, "height_m": height_m, "mass_kg": mass_kg,
              "colour": list(colour), "roughness": roughness, "static_friction": static_friction,
              "dynamic_friction": dynamic_friction, "builder": 1}
    key = hashlib.sha256(json.dumps(params, sort_keys=True).encode()).hexdigest()[:12]
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"cup_{key}.usd"
    info_path = out_dir / f"cup_{key}.json"
    if out_path.is_file() and info_path.is_file():
        return json.loads(info_path.read_text())

    stage_in = Usd.Stage.Open(str(source))
    cache = UsdGeom.XformCache()
    meshes = []
    for prim in stage_in.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        # Gf matrices are row-major and act on row vectors: p_world = p_local * M.
        matrix = np.array(cache.GetLocalToWorldTransform(prim))
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
        world = np.c_[points, np.ones(len(points))] @ matrix
        normals = mesh.GetNormalsAttr().Get()
        normals = None if normals is None else np.asarray(normals, dtype=np.float64) @ np.linalg.inv(matrix[:3, :3]).T
        meshes.append({"name": prim.GetName(), "points": world[:, :3], "normals": normals,
                       "normals_interpolation": mesh.GetNormalsInterpolation(),
                       "counts": np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int32),
                       "indices": np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)})
    if not meshes:
        raise RuntimeError(f"No meshes in {source}")

    # Body: the z extent is the diameter (the handle sticks out along +x only); axis at its middle.
    points = np.vstack([m["points"] for m in meshes])
    lo, hi = points.min(axis=0), points.max(axis=0)
    radius_src = (hi[2] - lo[2]) / 2.0
    axis_x, axis_z = lo[0] + radius_src, (lo[2] + hi[2]) / 2.0
    s_d, s_h = diameter_m / (2.0 * radius_src), height_m / (hi[1] - lo[1])

    def to_cup(p):
        # Y up -> Z up (+90 deg about x: y -> z, z -> -y), then scale, bottom centre to the origin.
        return np.column_stack([(p[:, 0] - axis_x) * s_d, -(p[:, 2] - axis_z) * s_d, (p[:, 1] - lo[1]) * s_h])

    def normals_to_cup(n):
        # Normals take the inverse scale.
        out = np.column_stack([n[:, 0] / s_d, -n[:, 2] / s_d, n[:, 1] / s_h])
        return out / np.clip(np.linalg.norm(out, axis=1, keepdims=True), 1e-12, None)

    handle = to_cup(points[points[:, 0] > axis_x + radius_src * 1.005])
    handle_lo, handle_hi = handle.min(axis=0), handle.max(axis=0)

    if out_path.exists():
        out_path.unlink()
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Cup")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(mass_kg)

    look = UsdShade.Material.Define(stage, "/Cup/Looks/Ceramic")
    shader = UsdShade.Shader.Define(stage, "/Cup/Looks/Ceramic/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    UsdGeom.Xform.Define(stage, "/Cup/Visual")
    for i, source_mesh in enumerate(meshes):
        mesh = UsdGeom.Mesh.Define(stage, f"/Cup/Visual/mesh_{i}")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(to_cup(source_mesh["points"]).astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(source_mesh["counts"]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(source_mesh["indices"]))
        if source_mesh["normals"] is not None:
            mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals_to_cup(source_mesh["normals"]).astype(np.float32)))
            mesh.SetNormalsInterpolation(source_mesh["normals_interpolation"])
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(look)

    physics = UsdShade.Material.Define(stage, "/Cup/Looks/Grip")
    material = UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
    material.CreateStaticFrictionAttr(static_friction)
    material.CreateDynamicFrictionAttr(dynamic_friction)
    material.CreateRestitutionAttr(0.0)

    UsdGeom.Scope.Define(stage, "/Cup/Collision")
    body = UsdGeom.Cylinder.Define(stage, "/Cup/Collision/body")
    body.CreateRadiusAttr(diameter_m / 2.0)
    body.CreateHeightAttr(height_m)
    body.CreateAxisAttr(UsdGeom.Tokens.z)
    body.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height_m / 2.0))
    box = UsdGeom.Cube.Define(stage, "/Cup/Collision/handle")
    box.CreateSizeAttr(1.0)
    box.AddTranslateOp().Set(Gf.Vec3d(*((handle_lo + handle_hi) / 2.0)))
    box.AddScaleOp().Set(Gf.Vec3f(*(handle_hi - handle_lo)))
    for collider in (body, box):
        prim = collider.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        collider.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(physics, UsdShade.Tokens.weakerThanDescendants, "physics")
    stage.GetRootLayer().Save()

    info = {"usd_path": str(out_path), "source": str(source), **params,
            "source_points": int(len(points)), "source_diameter_units": float(2 * radius_src),
            "source_height_units": float(hi[1] - lo[1]), "scale_diameter": float(s_d), "scale_height": float(s_h),
            "handle_box_m": {"min": handle_lo.round(4).tolist(), "max": handle_hi.round(4).tolist()},
            "collision": "solid cylinder (body) + box (handle); the visual mesh has no collider"}
    info_path.write_text(json.dumps(info, indent=2) + "\n")
    return info


"""USD assets of the combiner key demo: the key (a rigid body, its colliders boxes, drawn from Lukas's STL mirrored) and
the lock adapter (a kinematic block with the keyway and its lead-in); and for the holster beside the D1's base
(`holster.py`), the key with the colliders the holster holds it by, and the holster. Requires USD (pxr), not a
running Kit.

The adapter is `demos/insertion/asset.build_slot_usd`'s keyed slot without its door panel (the box's own door is
there): in the slot frame -- origin at the mouth's centre on the adapter's face, +x into the door, +y along the key at
the insertion angle, +z across it -- four blocks round the channel from the lead-in's end to the floor, four face
blocks round the lead-in, its four 45-degree wedges, and the floor. Its channel is the key's outline (the wards' 15 mm by
the shaft's 10) plus the clearance: an entrance for the key's profile. The cylinder's own keyway behind it, which would
hold the wards' angle, is not modelled; the record keeps the key's roll at seating against what such a keyway takes.
Contact and rest offsets are 1 mm and 0 on every collider, as the insertion demo's (PhysX's default is centimetres).
"""
from __future__ import annotations

from pathlib import Path
import struct

from .geometry import (ADAPTER, KEY_MASS_KG, KEY_MIRRORED, KEY_SEAT_BOX, KEY_SHAFT_BOX, KEY_STL, KEY_WARD_BOX,
                       STL_AXIS_YZ, STL_BACK_FACE_X, Adapter)

ASSETS = Path(__file__).resolve().parent / "assets"
CONTACT_OFFSET_M = 0.001
#: The adapter's channel clearance a side, over the key's outline.
CHANNEL_CLEARANCE_M = 0.0015


def read_stl(path) -> list[tuple[tuple[float, float, float], ...]]:
    """Triangles of a binary or ASCII STL, metres as stored."""
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        tris, current = [], []
        for line in data.decode(errors="ignore").splitlines():
            words = line.split()
            if words and words[0] == "vertex":
                current.append(tuple(float(v) for v in words[1:4]))
                if len(current) == 3:
                    tris.append(tuple(current))
                    current = []
        return tris
    count = struct.unpack("<I", data[80:84])[0]
    tris = []
    for i in range(count):
        values = struct.unpack("<12f", data[84 + 50 * i: 84 + 50 * i + 48])
        tris.append((values[3:6], values[6:9], values[9:12]))
    return tris


def key_mesh_points() -> tuple[list, list, list]:
    """The key's drawn mesh in its prim frame (x off the door, y along the lever, z up the platform's side; origin on
    the shaft's axis at the bar's back face): points, face vertex counts, indices. Mirrored left to right as the module
    docstring of `geometry.py` says, the winding reversed with it so the faces still face out."""
    tris = read_stl(ASSETS / KEY_STL)
    points, indices = [], []
    flip = -1.0 if KEY_MIRRORED else 1.0
    for tri in tris:
        mapped = [(x - STL_BACK_FACE_X, flip * (y - STL_AXIS_YZ[0]), z - STL_AXIS_YZ[1]) for x, y, z in tri]
        if KEY_MIRRORED:
            mapped = [mapped[0], mapped[2], mapped[1]]
        base = len(points)
        points.extend(mapped)
        indices.extend((base, base + 1, base + 2))
    return points, [3] * len(tris), indices


def _stage(output, root_name: str):
    from pxr import Usd, UsdGeom, UsdPhysics

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{root_name}").GetPrim()
    stage.SetDefaultPrim(root)
    return output, stage, root


def _material(stage, root_name, name, colour, metallic=0.0, roughness=0.5):
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, f"/{root_name}/Looks/{name}")
    shader = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat, colour


def _physics_material(stage, root_name, friction):
    from pxr import UsdPhysics, UsdShade

    mat = UsdShade.Material.Define(stage, f"/{root_name}/Looks/Contact")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction)
    api.CreateRestitutionAttr(0.0)
    return mat


def _collider(prim, physics_mat):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    UsdPhysics.CollisionAPI.Apply(prim)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(physics_mat, materialPurpose="physics")
    collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.CreateContactOffsetAttr(CONTACT_OFFSET_M)
    collision.CreateRestOffsetAttr(0.0)


def _box(stage, path, lo, hi, mat=None, physics_mat=None, visible=True):
    from pxr import Gf, UsdGeom, UsdShade

    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.0)
    shape.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
    shape.AddTranslateOp().Set(Gf.Vec3d(*[(a + b) / 2 for a, b in zip(lo, hi)]))
    shape.AddScaleOp().Set(Gf.Vec3f(*[b - a for a, b in zip(lo, hi)]))
    prim = shape.GetPrim()
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat[0])
        shape.CreateDisplayColorAttr([Gf.Vec3f(*mat[1])])
    if physics_mat is not None:
        _collider(prim, physics_mat)
    if not visible:
        UsdGeom.Imageable(prim).MakeInvisible()
    return prim


def build_key_usd(output, friction: float = 0.3) -> Path:
    """The key: a rigid body of `KEY_MASS_KG`, three box colliders in its prim frame, the STL drawn over them."""
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    output, stage, root = _stage(output, "Key")
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(KEY_MASS_KG)
    root.SetCustomDataByKey("description", "Lukas's lever key (combinerbox_lever v3, mirrored); demos/combiner_key")
    plastic = _material(stage, "Key", "PrintedPLA", (0.93, 0.42, 0.12), 0.0, 0.55)
    contact = _physics_material(stage, "Key", friction)
    # (u, n, t) boxes to the prim frame (x = n, y = u, z = t).
    for name, (u, n, t) in (("Shaft", KEY_SHAFT_BOX), ("Wards", KEY_WARD_BOX), ("Seat", KEY_SEAT_BOX)):
        _box(stage, f"/Key/{name}", (n[0], u[0], t[0]), (n[1], u[1], t[1]), physics_mat=contact, visible=False)
    points, counts, indices = key_mesh_points()
    mesh = UsdGeom.Mesh.Define(stage, "/Key/Visual")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get()))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(plastic[0])
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*plastic[1])])
    stage.GetRootLayer().Save()
    return output


def build_adapter_usd(output, a: Adapter = ADAPTER) -> Path:
    """The lock adapter in the slot frame (module docstring), a rigid body the environment makes kinematic."""
    from pxr import Gf, UsdGeom, UsdPhysics

    output, stage, root = _stage(output, "Adapter")
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(0.2)
    root.SetCustomDataByKey("description", "Printed lock adapter with a keyway and lead-in; demos/combiner_key")
    face = _material(stage, "Adapter", "PrintedPETG", (0.16, 0.17, 0.19), 0.0, 0.45)
    channel = _material(stage, "Adapter", "Keyway", (0.04, 0.04, 0.04), 0.3, 0.6)
    contact = _physics_material(stage, "Adapter", a.friction)
    su = max(abs(v) for v in KEY_WARD_BOX[0]) + CHANNEL_CLEARANCE_M         # along the key: the wards' ends
    sw = max(abs(v) for v in KEY_SHAFT_BOX[2]) + CHANNEL_CLEARANCE_M        # across it: the shaft's sides
    c, d, back, m = a.chamfer, a.depth, a.thickness, a.half_size
    box = lambda name, lo, hi, mat: _box(stage, f"/Adapter/{name}", lo, hi, mat, contact)
    # The channel's walls, from the lead-in's end to the floor (x into the door).
    box("WallTop", (c, -m, sw), (back, m, m), channel)
    box("WallBottom", (c, -m, -m), (back, m, -sw), channel)
    box("WallLeft", (c, -m, -sw), (back, -su, sw), channel)
    box("WallRight", (c, su, -sw), (back, m, sw), channel)
    box("Floor", (d, -su, -sw), (back, su, sw), channel)
    # The face round the lead-in, x in [0, c].
    box("FaceTop", (0.0, -m, sw + c), (c, m, m), face)
    box("FaceBottom", (0.0, -m, -m), (c, m, -(sw + c)), face)
    box("FaceLeft", (0.0, -m, -(sw + c)), (c, -(su + c), sw + c), face)
    box("FaceRight", (0.0, su + c, -(sw + c)), (c, m, sw + c), face)
    # The lead-in: four wedges, each a triangle (face corner, channel corner, back corner) swept along its edge.
    for name, span_axis, span, (a0, a1, a2) in (
            ("LeadInTop", 1, su + c, ((0.0, sw + c), (c, sw), (c, sw + c))),
            ("LeadInBottom", 1, su + c, ((0.0, -(sw + c)), (c, -sw), (c, -(sw + c)))),
            ("LeadInLeft", 2, sw + c, ((0.0, -(su + c)), (c, -su), (c, -(su + c)))),
            ("LeadInRight", 2, sw + c, ((0.0, su + c), (c, su), (c, su + c)))):
        pts = []
        for side in (-span, span):
            for x, tt in (a0, a1, a2):
                p = [x, 0.0, 0.0]
                p[span_axis] = side
                p[3 - span_axis] = tt
                pts.append(tuple(p))
        mesh = UsdGeom.Mesh.Define(stage, f"/Adapter/{name}")
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
        mesh.CreateFaceVertexCountsAttr([3, 3, 4, 4, 4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 5, 4, 3, 0, 3, 4, 1, 1, 4, 5, 2, 2, 5, 3, 0])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get()))
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*face[1])])
        from pxr import UsdShade
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(face[0])
        _collider(mesh.GetPrim(), contact)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    stage.GetRootLayer().Save()
    return output


def _mesh(stage, path, triangles, mat) -> None:
    """A drawn triangle mesh (no collider), from (n, 3, 3) triangles."""
    from pxr import Gf, UsdGeom, UsdShade

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for tri in triangles for p in tri])
    mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
    mesh.CreateFaceVertexIndicesAttr(list(range(3 * len(triangles))))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get()))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat[0])
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*mat[1])])


def build_holster_key_usd(output, friction: float = 0.3) -> Path:
    """The key for the holster (`holster.py`): the simulation's (v3 mirrored, `key_mesh_points`), a rigid body of
    `KEY_MASS_KG`, its colliders `holster.KEY_BOXES` (the bar, the platform, the shaft and wards) so that it rests in
    the holster and slides in it."""
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    from .holster import KEY_BOXES

    output, stage, root = _stage(output, "Key")
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(KEY_MASS_KG)
    root.SetCustomDataByKey("description", "Lukas's lever key (combinerbox_lever v3, mirrored), for the holster; "
                                           "demos/combiner_key/holster.py")
    plastic = _material(stage, "Key", "PrintedPLA", (0.93, 0.42, 0.12), 0.0, 0.55)
    contact = _physics_material(stage, "Key", friction)
    for name, (lo, hi) in KEY_BOXES.items():
        _box(stage, f"/Key/{name.capitalize()}", lo, hi, physics_mat=contact, visible=False)
    points, counts, indices = key_mesh_points()
    mesh = UsdGeom.Mesh.Define(stage, "/Key/Visual")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get()))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(plastic[0])
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*plastic[1])])
    stage.GetRootLayer().Save()
    return output


def build_holster_usd(output, friction: float = 0.3) -> Path:
    """The holster (`holster.py`) in the holstered key's frame: a rigid body the environment makes kinematic and
    carries with the robot's base, its colliders `holster.HOLSTER_BOXES`, Lukas's mesh drawn over them."""
    import numpy as np
    from pxr import UsdPhysics

    from .holster import HOLSTER_BOXES, HOLSTER_MESH_PATH, key_in_d1_base

    output, stage, root = _stage(output, "Holster")
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(0.03)
    root.SetCustomDataByKey("description", "Lukas's key holster beside the D1's base (2026-10-23); "
                                           "demos/combiner_key/holster.py")
    printed = _material(stage, "Holster", "PrintedPETG", (0.16, 0.17, 0.19), 0.0, 0.45)
    contact = _physics_material(stage, "Holster", friction)
    for name, (lo, hi) in HOLSTER_BOXES.items():
        _box(stage, f"/Holster/{name.capitalize()}", lo, hi, physics_mat=contact, visible=False)
    to_key = np.linalg.inv(key_in_d1_base())
    triangles = np.asarray(read_stl(HOLSTER_MESH_PATH)) @ to_key[:3, :3].T + to_key[:3, 3]
    _mesh(stage, "/Holster/Visual", triangles, printed)
    stage.GetRootLayer().Save()
    return output

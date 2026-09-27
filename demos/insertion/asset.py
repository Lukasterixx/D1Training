"""A door panel with a keyed slot, as a USD rigid body the demo spawns kinematic.

Procedural, no meshes downloaded. In the slot frame (`geometry.py`): the lock body is four blocks around the channel
from the chamfer's end to the back plate, four face blocks around the chamfered mouth, and four wedges -- convex
triangular prisms -- whose hypotenuses are the 45° chamfer; the back plate is the seat. The door panel is a thin plate
around it all, its front face flush with the mouth. Contact and rest offsets are set small on every collider: PhysX's
default contact offset is centimetres, and the clearance here is a millimetre.

Requires USD (pxr), not a running Kit.
"""
from __future__ import annotations

from pathlib import Path

from .geometry import GEOMETRY, SlotGeometry

CONTACT_OFFSET_M = 0.001


def build_slot_usd(output, g: SlotGeometry = GEOMETRY) -> Path:
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/SlotDoor").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(5.0)
    root.SetCustomDataByKey("description", "Door panel with a keyed slot; demos/insertion/asset.py")

    def material(name, colour, metallic=0.0, roughness=0.5):
        mat = UsdShade.Material.Define(stage, f"/SlotDoor/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat, colour

    panel_mat = material("Enamel", (0.73, 0.75, 0.76), 0.25, 0.32)
    lock_mat = material("LockSteel", (0.30, 0.31, 0.33), 0.85, 0.25)
    friction = UsdShade.Material.Define(stage, "/SlotDoor/Looks/Contact")
    contact = UsdPhysics.MaterialAPI.Apply(friction.GetPrim())
    contact.CreateStaticFrictionAttr(g.friction)
    contact.CreateDynamicFrictionAttr(g.friction)
    contact.CreateRestitutionAttr(0.0)

    def finish(shape, mat):
        prim = shape.GetPrim()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat[0])
        shape.CreateDisplayColorAttr([Gf.Vec3f(*mat[1])])
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(friction, materialPurpose="physics")
        collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collision.CreateContactOffsetAttr(CONTACT_OFFSET_M)
        collision.CreateRestOffsetAttr(0.0)
        return prim

    def box(name, lo, hi, mat):
        """An axis-aligned box from corner `lo` to corner `hi`, in the slot frame."""
        shape = UsdGeom.Cube.Define(stage, f"/SlotDoor/{name}")
        shape.CreateSizeAttr(1.0)
        shape.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
        shape.AddTranslateOp().Set(Gf.Vec3d(*[(a + b) / 2 for a, b in zip(lo, hi)]))
        shape.AddScaleOp().Set(Gf.Vec3f(*[b - a for a, b in zip(lo, hi)]))
        return finish(shape, mat)

    def prism(name, points, mat):
        """A convex hull of `points` (slot frame): a chamfer wedge."""
        mesh = UsdGeom.Mesh.Define(stage, f"/SlotDoor/{name}")
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        # Two triangles and three quads: a triangular prism; the hull is what collides.
        mesh.CreateFaceVertexCountsAttr([3, 3, 4, 4, 4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 5, 4, 3, 0, 3, 4, 1, 1, 4, 5, 2, 2, 5, 3, 0])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get()))
        prim = finish(mesh, mat)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        return prim

    su, sw, c, d, m = g.half_u, g.half_w, g.chamfer, g.depth, g.margin
    back = d + g.back
    # The channel's walls, from the chamfer's end to the back plate.
    box("WallTop", (c, -(su + m), sw), (back, su + m, sw + m), lock_mat)
    box("WallBottom", (c, -(su + m), -(sw + m)), (back, su + m, -sw), lock_mat)
    box("WallLeft", (c, -(su + m), -sw), (back, -su, sw), lock_mat)
    box("WallRight", (c, su, -sw), (back, su + m, sw), lock_mat)
    box("Seat", (d, -su, -sw), (back, su, sw), lock_mat)
    # The face around the chamfered mouth, x in [0, c].
    box("FaceTop", (0.0, -(su + m), sw + c), (c, su + m, sw + m), lock_mat)
    box("FaceBottom", (0.0, -(su + m), -(sw + m)), (c, su + m, -(sw + c)), lock_mat)
    box("FaceLeft", (0.0, -(su + m), -(sw + c)), (c, -(su + c), sw + c), lock_mat)
    box("FaceRight", (0.0, su + c, -(sw + c)), (c, su + m, sw + c), lock_mat)
    # The chamfer: four wedges, each a triangle (face corner, channel corner, back corner) swept along its edge.
    for name, span_axis, span, (a0, a1, a2) in (
            ("ChamferTop", 1, su + c, ((0.0, sw + c), (c, sw), (c, sw + c))),
            ("ChamferBottom", 1, su + c, ((0.0, -(sw + c)), (c, -sw), (c, -(sw + c)))),
            ("ChamferLeft", 2, sw + c, ((0.0, -(su + c)), (c, -su), (c, -(su + c)))),
            ("ChamferRight", 2, sw + c, ((0.0, su + c), (c, su), (c, su + c)))):
        points = []
        for side in (-span, span):
            for x, t in (a0, a1, a2):
                p = [x, 0.0, 0.0]
                p[span_axis] = side
                p[3 - span_axis] = t
                points.append(tuple(p))
        prism(name, points, lock_mat)
    # The door panel around the lock, its face flush with the mouth.
    h, t = g.panel_half, g.panel_thickness
    box("PanelTop", (0.0, -h, sw + m), (t, h, h), panel_mat)
    box("PanelBottom", (0.0, -h, -h), (t, h, -(sw + m)), panel_mat)
    box("PanelLeft", (0.0, -h, -(sw + m)), (t, -(su + m), sw + m), panel_mat)
    box("PanelRight", (0.0, su + m, -(sw + m)), (t, h, sw + m), panel_mat)
    stage.GetRootLayer().Save()
    return output

"""A door panel with a keyed slot, as a USD rigid body the demo spawns kinematic; and the door it is drawn in.

Procedural, no meshes downloaded. In the slot frame (`geometry.py`): the lock body is four blocks around the channel
from the chamfer's end to the back plate, four face blocks around the chamfered mouth, and four wedges -- convex
triangular prisms -- whose hypotenuses are the 45° chamfer; the back plate is the seat. The door panel is a thin plate
around it all, its front face flush with the mouth. Contact and rest offsets are set small on every collider: PhysX's
default contact offset is centimetres, and the clearance here is a millimetre. The lock's face and chamfer are brass and
its channel dark, so that seen through the scenery's escutcheon it is a lock cylinder with its keyway.

`build_scenery_usd` is the rest of a door, for watching (`env.py` places it per attempt): a wooden door in a wall, its
casing, a round escutcheon around the keyway and a lever handle above it. `build_floor_usd` is a floor of boards under
it, and `build_key_usd` the key in the hand (`key.key_mesh`). All three are visual only -- no collider, no mass, no
rigid body -- so they cannot change what the key touches. The door is drawn upright: the slot frame turns with the
blade's roll, and a door does not. Metals are only half metallic: the scene has no environment to reflect but the
black ground, and a full metal renders black.

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
    lock_mat = material("LockBrass", (0.80, 0.63, 0.30), 0.45, 0.35)
    channel_mat = material("Keyway", (0.05, 0.05, 0.05), 0.6, 0.6)
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
    box("WallTop", (c, -(su + m), sw), (back, su + m, sw + m), channel_mat)
    box("WallBottom", (c, -(su + m), -(sw + m)), (back, su + m, -sw), channel_mat)
    box("WallLeft", (c, -(su + m), -sw), (back, -su, sw), channel_mat)
    box("WallRight", (c, su, -sw), (back, su + m, sw), channel_mat)
    box("Seat", (d, -su, -sw), (back, su, sw), channel_mat)
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


# --------------------------------------------------------------------------- the scenery

#: The scenery's sizes (m), in the door frame: origin at the mouth on the door's face, +x into the door, +z up, +y to
#: the robot's left. None is a measurement; they are an ordinary interior door.
#: The door's face stands this far proud of the lock's face, so the two never fight for the same pixels. The key's tip
#: stops on the lock's face, so a miss off the escutcheon's opening shows the tip this far into the door.
FACE_PROUD_M = 0.002
DOOR_WIDTH_M, DOOR_THICKNESS_M = 0.90, 0.040
#: The keyway's distance from the door's free edge (on the robot's right), and the door's height above it. The door's
#: foot is far enough below the mouth to be under the floor at every height the demo draws (0.70-0.86 m).
LOCK_FROM_EDGE_M = 0.075
DOOR_ABOVE_M, DOOR_BELOW_M = 1.25, 1.20
DOOR_GAP_M = 0.003
CASING_WIDTH_M, CASING_PROUD_M = 0.07, 0.016
WALL_LEFT_M, WALL_RIGHT_M, WALL_ABOVE_M, WALL_THICKNESS_M = 1.8, 1.9, 1.9, 0.12
#: The escutcheon: a ring around the keyway, its opening wide enough for the estimate's miss and the search.
ESCUTCHEON_INNER_R_M, ESCUTCHEON_OUTER_R_M, ESCUTCHEON_PROUD_M = 0.016, 0.030, 0.004
#: The lever handle: its rose and spindle 92 mm above the keyway (a common European centre), the lever to the hinges.
HANDLE_ABOVE_M = 0.092
HANDLE_ROSE_R_M, HANDLE_ROSE_PROUD_M = 0.026, 0.009
HANDLE_STANDOFF_M, HANDLE_LENGTH_M = 0.058, 0.125


def build_scenery_usd(output, room_depth_m: float = 2.1) -> Path:
    """The door the lock is drawn in, as a visual-only USD in the door frame (see the constants above): the door in its
    wall, and a side wall on the robot's left running `room_depth_m` back from it, so the room has a corner."""
    import math

    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Scenery").GetPrim()
    stage.SetDefaultPrim(root)
    root.SetCustomDataByKey("description", "The door around the keyed slot, visual only; demos/insertion/asset.py")

    def material(name, colour, metallic=0.0, roughness=0.5):
        mat = UsdShade.Material.Define(stage, f"/Scenery/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat, colour

    wall = material("Wall", (0.84, 0.82, 0.77), 0.0, 0.9)
    wood = material("DoorWood", (0.40, 0.24, 0.13), 0.0, 0.55)
    trim = material("Casing", (0.93, 0.93, 0.91), 0.0, 0.4)
    shade = material("Shadow", (0.06, 0.05, 0.05), 0.0, 0.9)
    hardware = material("MatteBlack", (0.045, 0.045, 0.05), 0.3, 0.45)

    def finish(shape, mat):
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat[0])
        shape.CreateDisplayColorAttr([Gf.Vec3f(*mat[1])])

    def box(name, lo, hi, mat):
        shape = UsdGeom.Cube.Define(stage, f"/Scenery/{name}")
        shape.CreateSizeAttr(1.0)
        shape.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
        shape.AddTranslateOp().Set(Gf.Vec3d(*[(a + b) / 2 for a, b in zip(lo, hi)]))
        shape.AddScaleOp().Set(Gf.Vec3f(*[b - a for a, b in zip(lo, hi)]))
        finish(shape, mat)

    def cylinder(name, centre, radius, height, axis, mat):
        shape = UsdGeom.Cylinder.Define(stage, f"/Scenery/{name}")
        shape.CreateRadiusAttr(radius)
        shape.CreateHeightAttr(height)
        shape.CreateAxisAttr(axis)
        r = [radius, radius, radius]
        r["XYZ".index(axis)] = height / 2
        shape.CreateExtentAttr([Gf.Vec3f(*[-v for v in r]), Gf.Vec3f(*r)])
        shape.AddTranslateOp().Set(Gf.Vec3d(*centre))
        finish(shape, mat)

    def ring(name, x0, x1, inner, outer, mat, n=48):
        """A flat ring about the x axis, from x0 to x1: the escutcheon."""
        mesh = UsdGeom.Mesh.Define(stage, f"/Scenery/{name}")
        points, counts, indices = [], [], []
        for k in range(n):
            a = 2 * math.pi * k / n
            c, s_ = math.cos(a), math.sin(a)
            points += [(x, r * c, r * s_) for x in (x0, x1) for r in (inner, outer)]   # 4 per angle
        for k in range(n):
            i, j = 4 * k, 4 * ((k + 1) % n)
            # front (x0) and back (x1) faces, the outside and the inside, each outward
            for quad in ((i, i + 1, j + 1, j), (i + 2, j + 2, j + 3, i + 3), (i + 1, i + 3, j + 3, j + 1),
                         (i, j, j + 2, i + 2)):
                counts.append(4)
                indices += quad
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get()))
        finish(mesh, mat)

    face = -FACE_PROUD_M
    # The door leaf: its free edge on the robot's right, with a square hole the escutcheon covers.
    right, left = -LOCK_FROM_EDGE_M, DOOR_WIDTH_M - LOCK_FROM_EDGE_M
    top, foot = DOOR_ABOVE_M, -DOOR_BELOW_M
    back = face + DOOR_THICKNESS_M
    hole = 0.95 * ESCUTCHEON_OUTER_R_M / math.sqrt(2)
    box("DoorAbove", (face, right, hole), (back, left, top), wood)
    box("DoorBelow", (face, right, foot), (back, left, -hole), wood)
    box("DoorRightOfLock", (face, right, -hole), (back, -hole, hole), wood)
    box("DoorLeftOfLock", (face, hole, -hole), (back, left, hole), wood)
    # The wall around the doorway, its face flush with the door's; behind the gap, the stop, in shadow.
    g = DOOR_GAP_M
    wall_back = face + WALL_THICKNESS_M
    box("WallRight", (face, -WALL_RIGHT_M, foot), (wall_back, right - g, top + WALL_ABOVE_M), wall)
    box("WallLeft", (face, left + g, foot), (wall_back, WALL_LEFT_M, top + WALL_ABOVE_M), wall)
    box("WallAbove", (face, right - g, top + g), (wall_back, left + g, top + WALL_ABOVE_M), wall)
    box("DoorStop", (back, right - g, foot), (back + 0.012, left + g, top + g), shade)
    box("WallSide", (face - room_depth_m, WALL_LEFT_M, foot), (wall_back, WALL_LEFT_M + WALL_THICKNESS_M,
                                                               top + WALL_ABOVE_M), wall)
    # The casing round the doorway.
    cf, w = face - CASING_PROUD_M, CASING_WIDTH_M
    box("CasingRight", (cf, right - g - w, foot), (face, right - g, top + g + w), trim)
    box("CasingLeft", (cf, left + g, foot), (face, left + g + w, top + g + w), trim)
    box("CasingHead", (cf, right - g, top + g), (face, left + g, top + g + w), trim)
    # Three hinges on the left edge.
    for k, z in enumerate((foot + 0.95, 0.35 * top, top - 0.20)):
        cylinder(f"Hinge{k}", (face - 0.004, left + g / 2, z), 0.007, 0.09, "Z", hardware)
    # The escutcheon round the keyway; through its opening, the lock's brass face (`build_slot_usd`).
    ring("Escutcheon", face - ESCUTCHEON_PROUD_M, face, ESCUTCHEON_INNER_R_M, ESCUTCHEON_OUTER_R_M, hardware)
    # The lever handle above it, pointing to the hinges.
    hz = HANDLE_ABOVE_M
    cylinder("HandleRose", (face - HANDLE_ROSE_PROUD_M / 2, 0.0, hz), HANDLE_ROSE_R_M, HANDLE_ROSE_PROUD_M, "X", hardware)
    cylinder("HandleNeck", (face - HANDLE_STANDOFF_M / 2, 0.0, hz), 0.009, HANDLE_STANDOFF_M, "X", hardware)
    box("HandleLever", (face - HANDLE_STANDOFF_M - 0.006, -0.012, hz - 0.009),
        (face - HANDLE_STANDOFF_M + 0.012, HANDLE_LENGTH_M, hz + 0.009), hardware)
    cylinder("HandleEnd", (face - HANDLE_STANDOFF_M + 0.003, HANDLE_LENGTH_M, hz), 0.011, 0.018, "X", hardware)
    stage.GetRootLayer().Save()
    return output


def _preview_material(stage, path: str, colour, metallic: float, roughness: float):
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def build_key_usd(output, compliant: bool = False) -> Path:
    """The key in the hand (`key.key_mesh`), brass, visual only, in the frame of the link it hangs on
    (`key.key_link`): Link6's, or a compliant key's own, whose origin is the jaw centre."""
    from pxr import Gf, Usd, UsdGeom, UsdShade

    from .key import key_link, key_mesh

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Key").GetPrim()
    stage.SetDefaultPrim(root)
    root.SetCustomDataByKey("description", "The key in the hand, visual only; demos/insertion/key.py")
    shift = key_link(compliant)[1]
    tris = key_mesh()
    mesh = UsdGeom.Mesh.Define(stage, "/Key/Visual")
    mesh.CreatePointsAttr([Gf.Vec3f(p[0], p[1], p[2] + shift) for tri in tris for p in tri])
    mesh.CreateFaceVertexCountsAttr([3] * len(tris))
    mesh.CreateFaceVertexIndicesAttr(list(range(3 * len(tris))))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get()))
    brass = _preview_material(stage, "/Key/Looks/Brass", (0.82, 0.64, 0.28), 0.45, 0.3)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(brass)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.82, 0.64, 0.28)])
    stage.GetRootLayer().Save()
    return output


#: The floor's boards (m): their width, and how far the floor reaches round the robot, in the env frame.
BOARD_WIDTH_M = 0.14
FLOOR_BEHIND_M, FLOOR_RIGHT_M, FLOOR_LEFT_M = 1.6, 1.95, 1.85
#: Its top, this far above the ground the robot stands on: clear of the ground's own surface, and less than a foot
#: would show.
FLOOR_TOP_M = 0.0005
SHADE_ORDER = (0, 1, 2, 1, 0, 2, 2, 0, 1)


def build_floor_usd(output, wall_m: float) -> Path:
    """A floor of boards running to the wall, `wall_m` ahead of the robot, visual only, in the env frame."""
    from pxr import Gf, Usd, UsdGeom, UsdShade

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Floor").GetPrim()
    stage.SetDefaultPrim(root)
    root.SetCustomDataByKey("description", "A floor of boards in front of the door, visual only; demos/insertion/asset.py")
    shades = [(0.56, 0.42, 0.29), (0.60, 0.46, 0.32), (0.52, 0.39, 0.27)]
    looks = [_preview_material(stage, f"/Floor/Looks/Board{k}", c, 0.0, 0.6) for k, c in enumerate(shades)]
    x0, x1 = -FLOOR_BEHIND_M, wall_m + 0.15
    y, k = -FLOOR_RIGHT_M, 0
    while y < FLOOR_LEFT_M - 1e-9:
        w = min(BOARD_WIDTH_M, FLOOR_LEFT_M - y)
        shape = UsdGeom.Cube.Define(stage, f"/Floor/Board{k:02d}")
        shape.CreateSizeAttr(1.0)
        shape.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
        # A hair narrower than its pitch, so the joints show.
        shape.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, y + w / 2, FLOOR_TOP_M - 0.005))
        shape.AddScaleOp().Set(Gf.Vec3f(x1 - x0, w - 0.002, 0.010))
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(looks[SHADE_ORDER[k % len(SHADE_ORDER)]])
        shape.CreateDisplayColorAttr([Gf.Vec3f(*shades[SHADE_ORDER[k % len(SHADE_ORDER)]])])
        y, k = y + w, k + 1
    stage.GetRootLayer().Save()
    return output

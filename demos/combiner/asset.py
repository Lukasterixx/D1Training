"""Build a USD combiner-box proxy with a hinged door and rotating lever handle.

Original procedural geometry based on the user's reference photo. No downloaded
meshes or textures. The lever is a separate rigid body. The demo's LatchController
couples its rotation to the door constraint. Requires USD, but not a running Kit.

The door carries a tag36h11 AprilTag above the handle (`apriltag`), drawn as flat
cells the way the label's letters are, so it renders crisply without a texture.
"""
import math
from pathlib import Path

from . import apriltag
from .geometry import DEFAULT_HANDLE_TORQUE_NM, GEOMETRY, LEVER_GRAVITY_NM


def build_combiner_usd(output, handle_torque_nm=DEFAULT_HANDLE_TORQUE_NM):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/CombinerBox").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    root.SetCustomDataByKey("description", "Solar combiner proxy; rotating lever; runtime latch in demos.combiner.latch")

    def material(name, colour, metallic=0.0, roughness=0.5):
        mat = UsdShade.Material.Define(stage, f"/CombinerBox/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

    grey = material("Enamel", (0.73, 0.75, 0.76), 0.25, 0.32)
    inner = material("Interior", (0.55, 0.57, 0.58), 0.2)
    dark = material("BlackPlastic", (0.025, 0.028, 0.032))
    steel = material("HandleSteel", (0.48, 0.50, 0.52), 0.85, 0.22)
    yellow = material("WarningYellow", (1.0, 0.76, 0.015))
    red = material("WarningRed", (0.75, 0.035, 0.02))
    friction = UsdShade.Material.Define(stage, "/CombinerBox/Looks/Contact")
    contact = UsdPhysics.MaterialAPI.Apply(friction.GetPrim())
    contact.CreateStaticFrictionAttr(0.9)
    contact.CreateDynamicFrictionAttr(0.7)
    contact.CreateRestitutionAttr(0.0)

    def finish(shape, mat, collides):
        prim = shape.GetPrim()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
        # Display colours also make the asset readable in non-RTX USD viewers.
        shader = UsdShade.Shader(stage.GetPrimAtPath(f"{mat.GetPath()}/Shader"))
        shape.CreateDisplayColorAttr([shader.GetInput("diffuseColor").Get()])
        if collides:
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdShade.MaterialBindingAPI(prim).Bind(friction, materialPurpose="physics")
        return prim

    def cube(path, pos, size, mat, collides=True):
        shape = UsdGeom.Cube.Define(stage, path)
        shape.CreateSizeAttr(1.0)
        shape.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
        shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
        shape.AddScaleOp().Set(Gf.Vec3f(*size))
        return finish(shape, mat, collides)

    def cylinder(path, pos, radius, height, mat, axis="Z", collides=True):
        shape = UsdGeom.Cylinder.Define(stage, path)
        shape.CreateRadiusAttr(radius)
        shape.CreateHeightAttr(height)
        shape.CreateAxisAttr(axis)
        bounds = [radius, radius, radius]
        bounds["XYZ".index(axis)] = height / 2
        shape.CreateExtentAttr([Gf.Vec3f(*[-v for v in bounds]), Gf.Vec3f(*bounds)])
        shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
        return finish(shape, mat, collides)

    def polygons(path, points, counts, indices, mat):
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get()))
        return finish(mesh, mat, False)

    g = GEOMETRY
    body = UsdGeom.Xform.Define(stage, "/CombinerBox/Enclosure").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    UsdPhysics.MassAPI.Apply(body).CreateMassAttr(8.0)
    body_path = str(body.GetPath())
    z = g.bottom + g.height / 2
    # Five independent walls, leaving the front open rather than a solid box collider.
    cube(f"{body_path}/Back", (-g.depth / 2 + g.wall / 2, 0, z),
         (g.wall, g.width, g.height), grey)
    for name, y in (("Left", -g.width / 2 + g.wall / 2), ("Right", g.width / 2 - g.wall / 2)):
        cube(f"{body_path}/{name}", (0, y, z), (g.depth, g.wall, g.height), grey)
    for name, height in (("Bottom", g.bottom + g.wall / 2), ("Top", g.bottom + g.height - g.wall / 2)):
        cube(f"{body_path}/{name}", (0, 0, height), (g.depth, g.width, g.wall), grey)
    # Short mounting stand holds the glands above the floor and fixes the box in space.
    for i, y in enumerate((-0.13, 0.13)):
        cube(f"{body_path}/Stand{i}", (-0.06, y, g.bottom / 2), (0.025, 0.025, g.bottom), steel)
        cube(f"{body_path}/Foot{i}", (-0.04, y, 0.0075), (0.15, 0.055, 0.015), steel)
    for i in range(6):
        y = (i - 2.5) * 0.05
        cylinder(f"{body_path}/GlandNut{i}", (0.02, y, g.bottom - 0.012), 0.016, 0.025, dark)
        cylinder(f"{body_path}/Gland{i}", (0.02, y, g.bottom - 0.039), 0.011, 0.035, dark)
    cube(f"{body_path}/MountPlate", (-0.062, 0, z), (0.008, 0.30, 0.33), inner)
    for row, height in enumerate((z - 0.08, z + 0.08)):
        cube(f"{body_path}/Rail{row}", (-0.046, 0, height), (0.018, 0.28, 0.02), steel)
        for i in range(6):
            y = (i - 2.5) * 0.043
            cube(f"{body_path}/Fuse{row}_{i}", (-0.02, y, height), (0.036, 0.03, 0.065), grey)
            cube(f"{body_path}/Switch{row}_{i}", (0.001, y, height), (0.009, 0.018, 0.022), dark)
    for i, height in enumerate((z - 0.13, z + 0.13)):
        cylinder(f"{body_path}/HingeBarrel{i}", (g.door_x, -g.width / 2, height),
                 0.009, 0.05, steel, collides=False)

    door = UsdGeom.Xform.Define(stage, "/CombinerBox/Door")
    door.AddTranslateOp().Set(Gf.Vec3d(*g.hinge))
    door.AddOrientOp().Set(Gf.Quatf(1.0))
    UsdPhysics.RigidBodyAPI.Apply(door.GetPrim())
    UsdPhysics.MassAPI.Apply(door.GetPrim()).CreateMassAttr(0.6)
    dp = str(door.GetPath())
    cube(f"{dp}/Panel", (0, g.width / 2, 0), (g.door_thickness, g.width, g.height), grey)
    # A fixed round escutcheon and an independently rotating L-shaped lever.
    spindle_in_door = tuple(v - h for v, h in zip(g.spindle, g.hinge))
    cylinder(f"{dp}/HandleRosette", (spindle_in_door[0] + 0.003, spindle_in_door[1], 0),
             0.024, 0.006, steel, axis="X")
    handle = UsdGeom.Xform.Define(stage, "/CombinerBox/Handle")
    handle.AddTranslateOp().Set(Gf.Vec3d(*g.spindle))
    handle.AddOrientOp().Set(Gf.Quatf(1.0))
    UsdPhysics.RigidBodyAPI.Apply(handle.GetPrim())
    UsdPhysics.MassAPI.Apply(handle.GetPrim()).CreateMassAttr(0.12)
    hp = str(handle.GetPath())
    cylinder(f"{hp}/Spindle", (g.handle_projection / 2, 0, 0),
             0.012, g.handle_projection, steel, axis="X")
    cylinder(f"{hp}/Lever", (g.handle_projection, -g.handle_length / 2, 0),
             g.handle_radius, g.handle_length, steel, axis="Y")
    grasp = UsdGeom.Xform.Define(stage, f"{hp}/HandleGrasp")
    grasp.AddTranslateOp().Set(Gf.Vec3d(g.handle_projection, -g.handle_grasp_offset, 0))

    # Raised vector decals: triangle, lightning bolt, and a compact readable label.
    face_x = g.door_thickness / 2 + 0.0003
    cy, cz = g.width / 2 - 0.05, 0.025
    for name, size, offset, mat in (("WarningBorder", 0.064, 0, dark),
                                     ("WarningFill", 0.056, 0.0002, yellow)):
        points = [(face_x + offset, cy, cz + size),
                  (face_x + offset, cy - size * 0.87, cz - size * 0.5),
                  (face_x + offset, cy + size * 0.87, cz - size * 0.5)]
        polygons(f"{dp}/{name}", points, [3], [0, 1, 2], mat)
    bolt = [(0.004, 0.031), (-0.017, -0.003), (-0.002, 0.001),
            (-0.009, -0.024), (0.017, 0.012), (0.002, 0.007)]
    polygons(f"{dp}/Lightning", [(face_x + 0.0005, cy + y, cz + z) for y, z in bolt],
             [3, 3, 3, 3], [0, 1, 2, 0, 2, 5, 2, 3, 4, 2, 4, 5], red)
    glyphs = {
        "P": (30, 17, 17, 30, 16, 16, 16), "V": (17, 17, 17, 17, 17, 10, 4),
        "C": (14, 17, 16, 16, 16, 17, 14), "O": (14, 17, 17, 17, 17, 17, 14),
        "M": (17, 27, 21, 21, 17, 17, 17), "B": (30, 17, 17, 30, 17, 17, 30),
        "I": (14, 4, 4, 4, 4, 4, 14), "N": (17, 25, 21, 19, 17, 17, 17),
        "E": (31, 16, 16, 30, 16, 16, 31), "R": (30, 17, 17, 30, 20, 18, 17),
        "X": (17, 17, 10, 4, 10, 17, 17), " ": (0,) * 7,
    }
    label, pixel = "PV COMBINER BOX", 0.00135
    points, indices = [], []
    for letter, char in enumerate(label):
        for row, bits in enumerate(glyphs[char]):
            for col in range(5):
                if bits & (1 << (4 - col)):
                    y = cy + (letter * 6 + col - len(label) * 3) * pixel
                    z = -0.027 - row * pixel
                    indices.extend(range(len(points), len(points) + 4))
                    points.extend((face_x + 0.0003, y + dy * pixel, z + dz * pixel)
                                  for dy, dz in ((0, 0), (1, 0), (1, -1), (0, -1)))
    polygons(f"{dp}/Label", points, [4] * (len(indices) // 4), indices, dark)

    # The AprilTag: a white square with its quiet zone, and the black cells a hair in front of it.
    white = material("TagWhite", (0.92, 0.92, 0.92), 0.0, 0.8)
    ink = material("TagBlack", (0.01, 0.01, 0.01), 0.0, 0.8)
    tag_x, tag_y, tag_z = (v - h for v, h in zip(g.tag_centre, g.hinge))
    half = g.tag_outer_size / 2
    polygons(f"{dp}/AprilTagQuietZone",
             [(tag_x - 0.0002, tag_y + dy, tag_z + dz) for dy, dz in ((-half, half), (half, half), (half, -half), (-half, -half))],
             [4], [0, 1, 2, 3], white)
    cell = g.tag_size / 8
    points = []
    for row, col in apriltag.black_cells(g.tag_id):
        y, z = tag_y + (col - 3.5) * cell, tag_z + (3.5 - row) * cell
        points.extend((tag_x, y + dy * cell / 2, z + dz * cell / 2) for dy, dz in ((-1, 1), (1, 1), (1, -1), (-1, -1)))
    polygons(f"{dp}/AprilTag", points, [4] * (len(points) // 4), list(range(len(points))), ink)

    fixed = UsdPhysics.FixedJoint.Define(stage, "/CombinerBox/WorldMount")
    # A non-rigid parent frame is a world anchor that follows the asset's spawn transform.
    fixed.CreateBody0Rel().SetTargets([root.GetPath()])
    fixed.CreateBody1Rel().SetTargets([body.GetPath()])
    hinge = UsdPhysics.RevoluteJoint.Define(stage, "/CombinerBox/DoorHinge")
    hinge.CreateBody0Rel().SetTargets([body.GetPath()])
    hinge.CreateBody1Rel().SetTargets([door.GetPath()])
    hinge.CreateLocalPos0Attr(Gf.Vec3f(*g.hinge))
    hinge.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    # Both joint frames rotate +Z onto -Z, so positive joint angles open outward.
    hinge.CreateLocalRot0Attr(Gf.Quatf(0.0, 1.0, 0.0, 0.0))
    hinge.CreateLocalRot1Attr(Gf.Quatf(0.0, 1.0, 0.0, 0.0))
    hinge.CreateAxisAttr("Z")
    hinge.CreateLowerLimitAttr(0.0)
    hinge.CreateUpperLimitAttr(g.open_limit_deg)
    hinge.CreateCollisionEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(hinge.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)  # latch.py locks/releases the door using joint limits
    drive.CreateDampingAttr(0.0)    # Isaac Lab supplies passive damping in SI units

    handle_joint = UsdPhysics.RevoluteJoint.Define(stage, "/CombinerBox/HandleJoint")
    handle_joint.CreateBody0Rel().SetTargets([door.GetPath()])
    handle_joint.CreateBody1Rel().SetTargets([handle.GetPath()])
    handle_joint.CreateLocalPos0Attr(Gf.Vec3f(*spindle_in_door))
    handle_joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    handle_joint.CreateAxisAttr("X")
    handle_joint.CreateLowerLimitAttr(0.0)
    handle_joint.CreateUpperLimitAttr(g.handle_limit_deg)
    handle_joint.CreateCollisionEnabledAttr(False)
    spring = UsdPhysics.DriveAPI.Apply(handle_joint.GetPrim(), "angular")
    spring.CreateTypeAttr("force")
    # The rest angle sits below the 0 degree stop, so the lever starts preloaded against it.
    spring.CreateTargetPositionAttr(g.spring_rest_deg)
    # USD angular gains are per degree; Isaac Lab config uses Nm/rad.
    spring.CreateStiffnessAttr(math.radians(g.spring_stiffness(handle_torque_nm)))
    spring.CreateDampingAttr(math.radians(0.08))
    spring.CreateMaxForceAttr(g.spring_effort_limit(handle_torque_nm))
    stage.GetRootLayer().Save()
    return {"usd_path": str(output), "geometry": vars(g), "hinge_joint": "DoorHinge",
            "handle_joint": "HandleJoint", "handle_frame": "Handle/HandleGrasp",
            "door_mass_kg": 0.6, "enclosure_mass_kg": 8.0, "handle_mass_kg": 0.12,
            "handle_spring": spring_record(handle_torque_nm),
            "apriltag": {"family": apriltag.FAMILY, "id": g.tag_id, "size_m": g.tag_size,
                         "centre_enclosure_m": list(g.tag_centre),
                         "pose_enclosure": apriltag.tag_pose_box(g).round(6).tolist()},
            "assumptions": "Proxy geometry/masses; spring-return lever; runtime joint-limit latch, not a bolt simulation"}


def spring_record(handle_torque_nm):
    """What run.json says about the lever's return spring."""
    g = GEOMETRY
    return {"torque_at_45_deg_nm": handle_torque_nm, "stiffness_nm_per_rad": round(g.spring_stiffness(handle_torque_nm), 5),
            "rest_deg": g.spring_rest_deg, "preload_at_stop_nm": round(g.spring_torque(handle_torque_nm, 0.0), 4),
            "torque_at_stop_60_deg_nm": round(g.spring_torque(handle_torque_nm, g.handle_limit_deg), 4),
            "drive_effort_limit_nm": round(g.spring_effort_limit(handle_torque_nm), 4),
            "lever_weight_nm_at_horizontal": LEVER_GRAVITY_NM, "latch_release_deg": g.handle_release_deg,
            "source": "assumed; lowered from 0.94 N·m at 45 deg so the D1 can turn it (Lukas, 2026-09-19)"}

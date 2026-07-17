"""Build a single Go2+D1 articulation and hand its USD path to Isaac Lab.

Rescue mounts the arm by teleporting its root onto the dog's back every physics
step. That keeps the arm in place but tells the walking policy nothing: the
teleport rewrites the arm's root pose and zeroes its velocity each tick, so no
reaction force ever reaches the quadruped. The arm is, dynamically, a ghost.

Here the arm is welded. `build_welded_robot_usd` composes a stage that
references Isaac Lab's stock `go2.usd` and the URDF-imported D1, joins them with
a `UsdPhysics.FixedJoint`, and strips the D1's ArticulationRootAPI so PhysX
folds the arm's links into the Go2's articulation instead of parsing a second
one. The result is one articulation of 20 joints whose mass matrix includes the
arm -- which is the whole point: the payload now perturbs the gait.

The cost lands in `flat_env_cfg.py`: the robot no longer has exactly the 12
joints the policy was trained on, so every observation and action term has to be
scoped back down to the legs. See LEG_JOINTS there.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


@dataclass
class WeldResult:
    """What the weld produced, and what the env cfg needs to know about it."""

    usd_path: str
    # Factor the arm's link masses were scaled by (1.0 if untouched). Reported
    # for logging only -- notably NOT applied to the drive gains or the effort
    # limits, which come from Unitree's published per-joint torques and are
    # independent of how heavy the arm is.
    arm_mass_scale: float = 1.0


def import_d1_urdf(urdf_path: str, dest_usd_path: str) -> str:
    """Convert the D1 URDF to USD. Mirrors Rescue's `generate_arm_usd()`.

    `fix_base=False` matters even though the arm ends up welded: a fixed base
    would make the importer author its own root joint to the world, and the arm
    would drag the dog to the origin. The weld is our job, not the importer's.
    """
    import omni.kit.app
    import omni.kit.commands

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    try:
        ext_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        import isaacsim.asset.importer.urdf as urdf_importer
    except ImportError:
        ext_manager.set_extension_enabled_immediate("omni.importer.urdf", True)
        import omni.importer.urdf as urdf_importer

    import_config = urdf_importer._urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.make_default_prim = True

    if os.path.exists(dest_usd_path):
        os.remove(dest_usd_path)

    print(f"[weld] Importing D1 URDF -> {dest_usd_path}")
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=dest_usd_path,
    )
    return dest_usd_path


def _set_translate(xformable: UsdGeom.Xformable, vec: Gf.Vec3d) -> None:
    """Set the prim's translate op, reusing whichever one the reference brought in.

    The URDF importer authors a full [translate, orient, scale] op order on its
    default prim, and that arrives through the reference. Calling AddTranslateOp
    on top of it raises rather than overwriting.
    """
    for op in xformable.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            op.Set(vec)
            return
    xformable.AddTranslateOp().Set(vec)


def _find_prim_named(root: Usd.Prim, name: str) -> Usd.Prim | None:
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return prim
    return None


def _rigid_body_names(root: Usd.Prim) -> list[str]:
    return [
        p.GetName()
        for p in Usd.PrimRange(root)
        if p.HasAPI(UsdPhysics.RigidBodyAPI)
    ]


def _demote_articulation_root(subtree_root: Usd.Prim) -> int:
    """Remove every ArticulationRootAPI under `subtree_root`.

    This is what fuses the two robots. PhysX discovers an articulation from a
    prim carrying ArticulationRootAPI and then walks the joint graph outward; if
    the D1 keeps its own root API, PhysX parses a second articulation and the
    fixed joint between them degrades to a soft maximal-coordinate constraint --
    the arm visibly jiggles and the force coupling is mush. Stripped, the arm's
    links are reached only by walking outward from the Go2's root, so they join
    the Go2's articulation as ordinary links.

    RemoveAPI authors a deletion into the `apiSchemas` list op in this stage's
    root layer, which is stronger than the reference that applied it.
    """
    removed = 0
    for prim in Usd.PrimRange(subtree_root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            removed += 1
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    return removed


def _rescale_arm(subtree_root: Usd.Prim, target_kg: float) -> float:
    """Scale the arm's link masses so the subtree totals `target_kg`.

    The shipped URDF's inertials are a SolidWorks export of the shells alone --
    they total 0.719 kg, where Unitree publishes 3152 g for the D1-550. Left
    alone the arm is ~5% of the Go2's mass and the gait barely notices it, which
    would make this whole testbed answer "yes it walks fine" for the wrong
    reason.

    Only mass and inertia are touched. The joint effort limits are NOT scaled
    with it: Unitree publishes the mass and the per-joint torques as separate
    facts (3.3 Nm on J0/J1, 1.7 Nm on J2-J5), so the real arm is 3.152 kg with
    3.3 Nm motors -- not 3.152 kg with motors sized in proportion to its mass.
    Scaling the two together would invent a robot that does not exist. The
    URDF's efforts already match the spec; leave them be.

    Inertia tensors are scaled by the same factor as the mass. That is only
    strictly correct if the missing mass has the shell's spatial distribution,
    which it does not -- the motors sit at the joints. It is a deliberate
    approximation: total mass and its rough placement dominate the gait
    disturbance, and getting those right is worth more than an exact tensor we
    do not have.
    """
    if target_kg <= 0.0:
        raise ValueError(f"arm mass must be positive, got {target_kg}")

    links = [p for p in Usd.PrimRange(subtree_root) if p.HasAPI(UsdPhysics.MassAPI)]
    current = 0.0
    for prim in links:
        attr = UsdPhysics.MassAPI(prim).GetMassAttr()
        current += attr.Get() or 0.0

    if current <= 0.0:
        print("[weld][WARN] Arm links report no mass; skipping rescale.")
        return 1.0

    factor = target_kg / current
    for prim in links:
        mass_api = UsdPhysics.MassAPI(prim)
        mass_attr = mass_api.GetMassAttr()
        mass_attr.Set((mass_attr.Get() or 0.0) * factor)

        diag_attr = mass_api.GetDiagonalInertiaAttr()
        diag = diag_attr.Get()
        if diag:
            diag_attr.Set(Gf.Vec3f(diag[0] * factor, diag[1] * factor, diag[2] * factor))

    print(f"[weld] Arm mass rescaled {current:.3f} kg -> {target_kg:.3f} kg (x{factor:.2f}). "
          f"Joint effort limits left at the URDF's published-spec values.")
    return factor


def build_welded_robot_usd(
    go2_usd_path: str,
    d1_urdf_path: str,
    out_usd_path: str,
    mount_pos: tuple[float, float, float] = (0.0, 0.0, 0.08),
    go2_base_link: str = "base",
    d1_base_link: str = "base_link",
    arm_mass_kg: float | None = None,
) -> WeldResult:
    """Compose go2.usd + d1.usd into one articulation.

    `mount_pos` is the arm base's offset from the Go2's base link, in metres.
    It matches Rescue's ARM_MOUNT_Z so the reach limits and IK targets ported
    from there stay meaningful.
    """
    os.makedirs(os.path.dirname(out_usd_path), exist_ok=True)
    d1_usd_path = os.path.join(os.path.dirname(out_usd_path), "d1.usd")
    import_d1_urdf(d1_urdf_path, d1_usd_path)

    if os.path.exists(out_usd_path):
        os.remove(out_usd_path)

    stage = Usd.Stage.CreateNew(out_usd_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # /Robot takes on go2.usd's default prim wholesale -- including its
    # ArticulationRootAPI, which becomes the root of the merged articulation.
    root = UsdGeom.Xform.Define(stage, "/Robot")
    root_prim = root.GetPrim()
    stage.SetDefaultPrim(root_prim)
    root_prim.GetReferences().AddReference(go2_usd_path)

    # The arm hangs under the same root so PhysX can reach it from there.
    arm_xform = UsdGeom.Xform.Define(stage, "/Robot/D1")
    arm_prim = arm_xform.GetPrim()
    arm_prim.GetReferences().AddReference(d1_usd_path)
    # Placing the Xform at the mount keeps the spawn pose consistent with the
    # joint frames below; without it the arm starts inside the dog and PhysX
    # resolves the penetration with a kick on the first step.
    _set_translate(arm_xform, Gf.Vec3d(*mount_pos))

    n_removed = _demote_articulation_root(arm_prim)
    print(f"[weld] Stripped {n_removed} ArticulationRootAPI from the arm subtree.")

    arm_mass_scale = 1.0
    if arm_mass_kg is not None:
        arm_mass_scale = _rescale_arm(arm_prim, arm_mass_kg)

    go2_base = _find_prim_named(root_prim, go2_base_link)
    arm_base = _find_prim_named(arm_prim, d1_base_link)
    if go2_base is None:
        raise RuntimeError(
            f"[weld] No prim named '{go2_base_link}' in {go2_usd_path}. "
            f"Rigid bodies found: {_rigid_body_names(root_prim)}"
        )
    if arm_base is None:
        raise RuntimeError(
            f"[weld] No prim named '{d1_base_link}' in {d1_usd_path}. "
            f"Rigid bodies found: {_rigid_body_names(arm_prim)}"
        )

    joint = UsdPhysics.FixedJoint.Define(stage, "/Robot/D1/arm_mount_joint")
    joint.CreateBody0Rel().SetTargets([go2_base.GetPath()])
    joint.CreateBody1Rel().SetTargets([arm_base.GetPath()])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*mount_pos))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    # Left default (False) on purpose: excluding it would make PhysX treat the
    # weld as a maximal joint and re-split the articulation in two.
    joint.CreateExcludeFromArticulationAttr().Set(False)

    # Duplicate body names are ambiguous to Isaac Lab's find_bodies() regexes,
    # and the failure is a silently wrong body index rather than an exception.
    names = _rigid_body_names(root_prim)
    dupes = {n for n in names if names.count(n) > 1}
    if dupes:
        raise RuntimeError(f"[weld] Duplicate rigid body names across Go2 and D1: {sorted(dupes)}")

    stage.GetRootLayer().Save()
    print(f"[weld] Welded robot written to {out_usd_path}")
    print(f"[weld] Rigid bodies ({len(names)}): {names}")
    return WeldResult(usd_path=out_usd_path, arm_mass_scale=arm_mass_scale)

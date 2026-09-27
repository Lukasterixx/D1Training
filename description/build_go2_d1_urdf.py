"""Regenerate `go2_d1.urdf` from its two sources.

The committed output is what `run_rviz.sh` uses; this script only has to run
again if one of the sources changes. The Go2 half is P2Dingo's go2.urdf, or the
same file as Go2RemoteConnection carries it (only its mesh paths differ, and only
the mesh file names are kept): pass a P2Dingo checkout or a go2.urdf as argv[1],
or let it take the first of `GO2_SOURCES` that exists.

The merge mirrors `weld.py`: the arm's base is fixed to the Go2's `base_link` at
`mount.ARM_MOUNT_POS_M`, so the URDF and the simulated articulation agree about where the D1
sits, and the Go2 EDU's expansion dock and rails are drawn under it
(`dock_from_cad.py`), as `weld.py` draws them in the sim. Everything else the
script does is about being a *drawing* rather than a second source of truth --
see description/README.md.

    python3 description/build_go2_d1_urdf.py [path/to/P2Dingo | path/to/go2.urdf]
"""
from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

P2DINGO_GO2_URDF = "Isaac/go2_ws/src/go2_control_cpp/config/go2.urdf"
# The same file in two checkouts: P2Dingo's, and Go2RemoteConnection's (2026-09-26: byte-identical output).
GO2_SOURCES = [os.path.join(os.path.expanduser("~/P2Dingo"), P2DINGO_GO2_URDF),
               os.path.expanduser("~/go2_ws/Go2RemoteConnection/description/go2/go2.urdf")]
if len(sys.argv) > 1:
    GO2_URDF = sys.argv[1] if sys.argv[1].endswith(".urdf") else os.path.join(sys.argv[1], P2DINGO_GO2_URDF)
else:
    GO2_URDF = next((p for p in GO2_SOURCES if os.path.exists(p)), GO2_SOURCES[0])
D1_URDF = os.path.join(REPO, "d1_arm", "d1.urdf")
OUT = os.path.join(HERE, "go2_d1.urdf")

# Where weld.py actually puts the arm (mount.py, F-117).
sys.path.insert(0, REPO)
from mount import ARM_MOUNT_POS_M  # noqa: E402

# One named material for the whole arm; see the comment where it is applied.
ARM_MATERIAL = "d1_arm"
ARM_MATERIAL_RGBA = "0.85 0.45 0.10 1"

# The Go2 EDU's Jetson expansion dock and the two rails the D1 stands on (`dock.py`, from Unitree's CAD by
# `dock_from_cad.py`); the meshes are in base_link's frame. (mesh, material, rgba), as weld.py draws them.
from dock import DOCK_MESHES  # noqa: E402

DOCK_LINK = "expansion_dock"
DOCK_VISUALS = tuple((stl, name, " ".join(f"{c:g}" for c in rgb) + " 1") for name, stl, rgb in DOCK_MESHES)


def main() -> None:
    if not os.path.exists(GO2_URDF):
        raise SystemExit(
            f"No Go2 description at {GO2_URDF}.\n"
            f"Pass a P2Dingo checkout or a go2.urdf: "
            f"python3 {sys.argv[0]} /path/to/P2Dingo"
        )

    go2 = ET.parse(GO2_URDF).getroot()
    go2.set("name", "go2_d1")

    # The L1's frame comes from the sim's TF (base_link -> utlidar_lidar, see
    # ros2.py), so the URDF must not declare it too: two publishers of one
    # transform is a TF authority conflict, and RViz flags it.
    for child in list(go2):
        if child.get("name") in ("radar", "radar_joint"):
            go2.remove(child)

    # `package://go2_control_cpp/meshes/base.dae` in P2Dingo's, `file:///.../go2/meshes/base.dae` in
    # Go2RemoteConnection's: the file names are what they share, and what meshes/go2/ holds.
    for mesh in go2.iter("mesh"):
        fn = mesh.get("filename")
        assert "/meshes/" in fn, fn
        mesh.set("filename", "package://d1_training/description/meshes/go2/" + fn.rsplit("/meshes/", 1)[1])

    # The dock, drawn only: no collision (the sim's is weld.py's, which has none for it either) and no inertial.
    dock = ET.SubElement(go2, "link", {"name": DOCK_LINK})
    for filename, material, _ in DOCK_VISUALS:
        visual = ET.SubElement(dock, "visual")
        ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", {"filename": f"package://d1_training/description/meshes/go2/{filename}"})
        ET.SubElement(visual, "material", {"name": material})
    dock_joint = ET.SubElement(go2, "joint", {"name": f"{DOCK_LINK}_joint", "type": "fixed"})
    ET.SubElement(dock_joint, "parent", {"link": "base_link"})
    ET.SubElement(dock_joint, "child", {"link": DOCK_LINK})
    ET.SubElement(dock_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})

    d1 = ET.parse(D1_URDF).getroot()
    for el in d1.iter():
        # The D1's root link is also called base_link, which collides.
        if el.tag == "link" and el.get("name") == "base_link":
            el.set("name", "d1_base_link")
        if el.tag in ("parent", "child") and el.get("link") == "base_link":
            el.set("link", "d1_base_link")
    for mesh in d1.iter("mesh"):
        fn = mesh.get("filename")
        assert fn.startswith("meshes/"), fn
        mesh.set("filename", f"package://d1_training/d1_arm/{fn}")

    # The D1's STLs carry no colour of their own, and the URDF's materials are
    # unnamed placeholders. Left alone, RViz falls back to its default red, which
    # reads as "this link has no material" rather than as a decision. Point every
    # arm visual at one named material instead: the arm should be obvious against
    # the Go2's white shell, since seeing where it is *is* the point of drawing it.
    for visual in d1.iter("visual"):
        for material in list(visual.findall("material")):
            visual.remove(material)
        ET.SubElement(visual, "material", {"name": ARM_MATERIAL})

    mount = ET.SubElement(go2, "joint", {"name": "arm_mount_joint", "type": "fixed"})
    ET.SubElement(mount, "parent", {"link": "base_link"})
    ET.SubElement(mount, "child", {"link": "d1_base_link"})
    ET.SubElement(mount, "origin", {"xyz": " ".join(f"{v:g}" for v in ARM_MOUNT_POS_M), "rpy": "0 0 0"})

    for child in list(d1):
        go2.append(child)

    # Defined once at robot level and referenced by name above, which is the
    # idiom urdfdom wants -- repeating the definition per visual makes it warn
    # about a non-unique material.
    arm_material = ET.SubElement(go2, "material", {"name": ARM_MATERIAL})
    ET.SubElement(arm_material, "color", {"rgba": ARM_MATERIAL_RGBA})
    for _, material, rgba in DOCK_VISUALS:
        ET.SubElement(ET.SubElement(go2, "material", {"name": material}), "color", {"rgba": rgba})

    # Unnamed materials are legal in these SolidWorks exports, and RViz logs a
    # warning for each one. The Go2's colours come from its .dae files, so the
    # placeholders carry nothing worth keeping.
    for parent in go2.iter():
        for material in list(parent.findall("material")):
            if not material.get("name"):
                parent.remove(material)

    # Inertials come out. weld.py owns the mass model; a second set of numbers
    # here would only drift out of step with it and be mistaken for spec.
    # Dropping base_link's also silences KDL's "root link has an inertia"
    # warning from robot_state_publisher.
    #
    # So this file must never be handed to an importer that reads inertials.
    # Isaac Lab does not (weld.py assembles the USD), but Isaac Gym does, and
    # with no inertials it derives mass from collision geometry times its
    # density default -- which makes the robot nearly massless and NaNs the
    # articulation on the first step (F-079). `unifp_go2d1/build_asset.py`
    # rebuilds weld.py's mass model into its own generated copy for that
    # reason; anything else reading this URDF for simulation needs to do the
    # same.
    for link in go2.findall("link"):
        for inertial in list(link.findall("inertial")):
            link.remove(inertial)

    _check(go2)

    ET.indent(go2, space="  ")
    with open(OUT, "w") as f:
        f.write(
            '<?xml version="1.0"?>\n'
            "<!-- GENERATED by description/build_go2_d1_urdf.py from P2Dingo's\n"
            "     go2.urdf, d1_arm/d1.urdf and the expansion dock's meshes.\n"
            "     Visualisation only: the physics model is weld.py's.\n"
            "     See description/README.md. -->\n"
        )
        f.write(ET.tostring(go2, encoding="unicode") + "\n")
    print(f"[urdf] wrote {OUT}")


def _check(robot: ET.Element) -> None:
    """Fail loudly on the two mistakes that would show up only in RViz.

    A dangling link reference gives 'Failed to build tree' from KDL, and a second
    root gives a robot that silently draws in the wrong place.
    """
    links = {link.get("name") for link in robot.findall("link")}
    joints = robot.findall("joint")
    for joint in joints:
        for side in ("parent", "child"):
            ref = joint.find(side).get("link")
            if ref not in links:
                raise SystemExit(f"[urdf] joint {joint.get('name')} has unknown {side} '{ref}'")

    roots = links - {joint.find("child").get("link") for joint in joints}
    if roots != {"base_link"}:
        raise SystemExit(f"[urdf] expected base_link as the only root, got {sorted(roots)}")

    movable = [j.get("name") for j in joints if j.get("type") != "fixed"]
    print(f"[urdf] {len(links)} links, {len(joints)} joints, "
          f"{len(movable)} movable: {movable}")
    print("[urdf] those names must match what the sim puts on /joint_states -- "
          "sim.py prints the articulation's joints at startup.")


if __name__ == "__main__":
    main()

"""Emit the Go2+D1 URDF that UniFP's Isaac Gym environment loads.

UniFP is legacy `legged_gym` on Isaac Gym Preview 4, so it loads a URDF from a directory
rather than the USD the Isaac Lab task uses. This takes `description/go2_d1.urdf` (the same
welded model `weld.py` builds from) and writes an Isaac-Gym-loadable copy:

  * `package://d1_training/...` mesh URIs become paths relative to the asset root, because
    Isaac Gym has no ROS package resolver and fails the whole URDF on one unresolved mesh.
  * The eight arm joints are renamed `Joint<n>` -> `d1_Joint<n>`. This is forced, not
    cosmetic: Isaac Gym orders an articulation's DOFs by walking the base link's child
    subtrees in alphabetical order of the *joint* that starts each one, so `Joint1` put the
    arm between the front and rear legs (DOFs 6-13). UniFP's environment assumes 12 legs,
    then the arm, then the gripper, and slices tensors by fixed index everywhere. The `d1_`
    prefix sorts the arm after `RR_hip_joint` and gives DOFs 0-11 legs, 12-17 arm, 18-19
    gripper. Link and body names are untouched, so `Link7_1`, `Link6` and the contact bodies
    still mean what they mean everywhere else in this repo; only joint (DOF) names differ,
    and only inside the UniFP port.

  * Inertials are put back. `description/go2_d1.urdf` is a drawing for RViz and drops every
    `<inertial>` block on purpose, because `weld.py` owns the simulated mass model (see
    description/README.md). Isaac Gym has no such second source: with no inertials it derives
    mass from collision geometry times `asset_options.density`, which at legged_gym's default
    of 0.005 gives near-massless bodies and NaNs the articulation on the first step. So this
    reassembles the same mass model `weld.py` builds for Isaac Lab:
      - Go2 links take their inertials from the Go2 description the URDF was merged from
        (15.019 kg, Unitree's published values),
      - D1 links take the SolidWorks shell inertials from `d1_arm/d1.urdf` (0.719 kg total),
      - then the D1 shells get their servo mass (`workspace.SERVO_MASS_BY_LINK`) and the
        remainder of ARM_MASS_KG goes on the arm's base link, each link's inertia scaled by
        its own mass factor -- `weld.py:_apply_d1_mass_model`, term for term.
    The arm's base is welded to the Go2 and collapses into `base_link`, so its ~2.1 kg loads
    the dog rather than any arm joint, exactly as in the Isaac Lab model.

  * The arm's VISUAL meshes are pre-rotated. `asset_options.flip_visual_attachments` is one flag
    for the whole asset, but the two halves of this robot disagree: the Go2's `.dae` meshes are
    y-up and need the flip (without it the dog renders on its side with its legs splayed), while
    the D1's SolidWorks `.STL` meshes are already z-up and must not be flipped. With the flag on,
    the dog is right and the arm lies on its side. So each arm mesh gets a `_visflip` copy rotated
    by the inverse of the flip, and only `<visual>` points at it -- `<collision>` keeps the
    original file. That is safe because the flag is visual-only: measured masses identical and
    every body within 0.0000 mm after a 2 s settle with it on and off, since the Go2's collisions
    are all primitives and the flag does not touch the arm's collision meshes either.

  * A massless `ee_gripper_link` is added at the pincer tip. UniFP indexes the controlled
    point by body name (`cfg.asset.gripper_name`) and applies its external EE force there,
    so the point has to exist as a body. It is placed at `position_only/tool_point.py`'s
    TOOL_BODY/TOOL_OFFSET_M, so the UniFP task and the Isaac Lab position-only task control
    the same physical point (the CAD pincer tip, F-013 -- CAD, not measured on the arm).
    `dont_collapse="true"` keeps it through Isaac Gym's fixed-joint collapsing, the same way
    upstream's b2z1.urdf keeps its own `ee_gripper_link`.

Nothing else about the model changes: same links, inertials, limits and joint origins.
"""

import os
import shutil
import sys
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from position_only.tool_point import TOOL_BODY, TOOL_OFFSET_M  # noqa: E402
from position_only.workspace import ARM_MASS_KG, SERVO_MASS_BY_LINK  # noqa: E402

SRC_URDF = os.path.join(REPO, "description", "go2_d1.urdf")
D1_URDF = os.path.join(REPO, "d1_arm", "d1.urdf")

# The Go2 description `description/go2_d1.urdf` was merged from, for its inertials only.
# build_go2_d1_urdf.py reads the P2Dingo copy; the Go2RemoteConnection copy is the same file.
GO2_SOURCES = [
    "~/go2_ws/Go2RemoteConnection/description/go2/go2.urdf",
    "~/P2Dingo/Isaac/go2_ws/src/go2_control_cpp/config/go2.urdf",
]

# weld.py puts everything the shells and servos do not account for on the arm's base link.
ARM_BASE_LINK = "d1_base_link"   # `base_link` in d1.urdf, renamed by build_go2_d1_urdf.py

# Where each package:// prefix lives in this repo, and where it goes under the asset root.
MESH_MAP = {
    "package://d1_training/description/meshes/go2/": ("description/meshes/go2", "meshes/go2"),
    "package://d1_training/d1_arm/meshes/": ("d1_arm/meshes", "meshes/d1"),
}


# Isaac Gym's flip converts y-up to z-up, a +90 degree rotation about X. Pre-rotating a mesh by
# the inverse cancels it, so a mesh that is already z-up survives the flip unchanged.
_VISFLIP = "_visflip"


def _rotate_stl(src, dst):
    """Write `src` (binary STL) to `dst` rotated by Rx(-90): (x, y, z) -> (x, z, -y)."""
    import struct
    data = open(src, "rb").read()
    count = struct.unpack("<I", data[80:84])[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"{src} is not a binary STL ({len(data)} bytes, {count} triangles)")
    out = bytearray(data[:84])
    for t in range(count):
        base = 84 + 50 * t
        vals = list(struct.unpack("<12f", data[base:base + 48]))
        for v in range(4):                      # one normal then three vertices
            x, y, z = vals[3 * v:3 * v + 3]
            vals[3 * v:3 * v + 3] = [x, z, -y]
        out += struct.pack("<12f", *vals) + data[base + 48:base + 50]
    open(dst, "wb").write(bytes(out))
    return count


def _inertials(urdf_path, rename=None):
    """{link name: <inertial> element} for every link in `urdf_path` that has one."""
    root = ET.parse(urdf_path).getroot()
    out = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is not None:
            name = link.get("name")
            out[(rename or {}).get(name, name)] = inertial
    return out


def _mass(inertial):
    return float(inertial.find("mass").get("value"))


def _scale(inertial, factor):
    """Scale an inertial's mass and its whole inertia tensor by `factor`, in place."""
    inertial.find("mass").set("value", f"{_mass(inertial) * factor:.9g}")
    tensor = inertial.find("inertia")
    for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        tensor.set(key, f"{float(tensor.get(key)) * factor:.9g}")


def _d1_mass_model():
    """`weld.py:_apply_d1_mass_model`, applied to URDF inertials instead of USD prims."""
    shells = _inertials(D1_URDF, rename={"base_link": ARM_BASE_LINK})
    shell_total = sum(_mass(i) for i in shells.values())
    servo_total = sum(SERVO_MASS_BY_LINK.values())
    base_extra = ARM_MASS_KG - shell_total - servo_total
    if base_extra < 0.0:
        raise ValueError(f"arm mass {ARM_MASS_KG} kg is below shells + servos "
                         f"({shell_total + servo_total:.3f} kg)")
    missing = (set(SERVO_MASS_BY_LINK) | {ARM_BASE_LINK}) - set(shells)
    if missing:
        raise RuntimeError(f"mass model does not match d1.urdf; missing {sorted(missing)}")

    for name, inertial in shells.items():
        target = _mass(inertial) + SERVO_MASS_BY_LINK.get(name, 0.0)
        if name == ARM_BASE_LINK:
            target += base_extra
        _scale(inertial, target / _mass(inertial))
    moving = ARM_MASS_KG - _mass(shells[ARM_BASE_LINK])
    print(f"  D1 mass model -> {ARM_MASS_KG:.3f} kg (shells {shell_total:.3f} "
          f"+ servos {servo_total:.3f} + base fill {base_extra:.3f})")
    print(f"    {ARM_BASE_LINK}: {_mass(shells[ARM_BASE_LINK]):.3f} kg, welded to the Go2")
    print(f"    moving arm above the base: {moving:.3f} kg")
    return shells


def _go2_inertials():
    for candidate in GO2_SOURCES:
        path = os.path.expanduser(candidate)
        if os.path.exists(path):
            found = _inertials(path)
            print(f"  Go2 inertials from {path} "
                  f"({len(found)} links, {sum(_mass(i) for i in found.values()):.3f} kg)")
            return found
    raise FileNotFoundError(
        "No Go2 description to take inertials from. Tried:\n  " + "\n  ".join(GO2_SOURCES) +
        "\nPoint one of those at the same go2.urdf description/build_go2_d1_urdf.py merges.")


def build(dest_root):
    tree = ET.parse(SRC_URDF)
    root = tree.getroot()

    # 1. Rewrite mesh URIs and copy the meshes they name.
    copied = 0
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        for prefix, (src_dir, dst_dir) in MESH_MAP.items():
            if fn.startswith(prefix):
                name = fn[len(prefix):]
                src = os.path.join(REPO, src_dir, name)
                if not os.path.exists(src):
                    raise FileNotFoundError(f"{fn} -> {src} does not exist")
                dst = os.path.join(dest_root, dst_dir, name)
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                if not os.path.exists(dst):
                    shutil.copy2(src, dst)
                    copied += 1
                mesh.set("filename", f"./{dst_dir}/{name}")
                break
        else:
            raise ValueError(f"unhandled mesh URI: {fn}")

    # 1b. Give the arm's visual meshes a pre-rotated copy (see module docstring).
    rotated = 0
    for link in root.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        mesh = visual.find("geometry/mesh")
        if mesh is None or "/d1/" not in mesh.get("filename"):
            continue
        rel = mesh.get("filename")
        stem, ext = os.path.splitext(rel)
        flipped = stem + _VISFLIP + ext
        src = os.path.join(dest_root, rel.lstrip("./"))
        dst = os.path.join(dest_root, flipped.lstrip("./"))
        _rotate_stl(src, dst)
        mesh.set("filename", flipped)
        rotated += 1
    print(f"  {rotated} arm visual meshes pre-rotated; collision meshes left untouched")

    # 2. Reorder the DOFs into UniFP's [12 legs | 6 arm | 2 gripper] layout (see module docstring).
    arm_joints = [f"Joint{n}" for n in (1, 2, 3, 4, 5, 6)] + ["Joint7_1", "Joint7_2"]
    renamed = 0
    for joint in root.findall("joint"):
        if joint.get("name") in arm_joints:
            joint.set("name", "d1_" + joint.get("name"))
            renamed += 1
    if renamed != len(arm_joints):
        raise ValueError(f"renamed {renamed} arm joints, expected {len(arm_joints)}")

    # 3. Put the mass model back (see module docstring).
    print("mass model:")
    sources = _go2_inertials()
    sources.update(_d1_mass_model())
    restored, skipped = 0, []
    for link in root.findall("link"):
        name = link.get("name")
        if link.find("inertial") is not None:
            continue
        if name in sources:
            link.insert(0, sources[name])
            restored += 1
        else:
            skipped.append(name)
    print(f"  {restored} links given inertials; {len(skipped)} left massless "
          f"(fixed frames with no mass in either source): {', '.join(skipped)}")

    # 4. Add the controlled point as a body UniFP can index and push on.
    if any(l.get("name") == "ee_gripper_link" for l in root.findall("link")):
        raise ValueError("go2_d1.urdf already defines ee_gripper_link")
    joint = ET.SubElement(root, "joint")
    joint.set("name", "ee_gripper")
    joint.set("type", "fixed")
    joint.set("dont_collapse", "true")
    ET.SubElement(joint, "origin", {"rpy": "0 0 0", "xyz": " ".join(f"{v:.6f}" for v in TOOL_OFFSET_M)})
    ET.SubElement(joint, "parent", {"link": TOOL_BODY})
    ET.SubElement(joint, "child", {"link": "ee_gripper_link"})
    link = ET.SubElement(root, "link")
    link.set("name", "ee_gripper_link")
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", {"value": "0.001"})
    ET.SubElement(inertial, "inertia", {"ixx": "1e-6", "ixy": "0", "ixz": "0",
                                        "iyy": "1e-6", "iyz": "0", "izz": "1e-6"})

    os.makedirs(dest_root, exist_ok=True)
    out = os.path.join(dest_root, "go2d1.urdf")
    tree.write(out, encoding="utf-8", xml_declaration=True)
    print(f"wrote {out} ({copied} meshes copied, {renamed} arm joints prefixed d1_)")
    print(f"ee_gripper_link at {TOOL_BODY} + {TOOL_OFFSET_M} m")
    return out


if __name__ == "__main__":
    dest = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/thesis_b_legacy/UniFP/resources/robots/go2d1")
    build(dest)

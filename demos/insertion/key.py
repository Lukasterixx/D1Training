"""The key in the hand: a copy of `d1_arm/d1.urdf` with the key's blade as a collider (and visual).

The blade's long side lies along `Link6`'s +y (the jaw axis, so the roll command turns it), its thin side along +x,
running along the approach (+z) from just past the fingertips to the tip at the jaw centre plus the key's length. A
visual-only handle between the fingers shows what they would be holding. Two ways to hold it:

  * rigid (the default): the blade is a collider on `Link6`, as the training's plant had it until 2026-09-27;
  * compliant (`compliant=True`): the blade is its own link, `key`, hung from the jaw centre on two sprung revolute
    joints across its axis, `key_rx` then `key_ry`, each limited to `insertion.GRIP_MAX_RAD` -- the PhysX form of
    `unifp_train.insertion.CompliantGrip`. The springs are the robot config's (`run_demo.py`); a policy never drives
    them (`interface.permutation_from`'s `passive`).

Numpy-free, plain `xml.etree`; like `demos/unifp/claw.py`, mesh paths are rewritten absolute because the copy does
not live beside `d1_arm/meshes/`.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from .geometry import GEOMETRY, JAW_CENTRE_Z_M, KEY_ROOT_Z_M

#: The compliant key's joints, and its blade's mass: a 10 x 3 x 55 mm steel blade is about 13 g.
KEY_JOINTS = ("key_rx", "key_ry")
KEY_MASS_KG = 0.015


def blade_box() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(centre, size) of the blade in `Link6`."""
    tip = JAW_CENTRE_Z_M + GEOMETRY.key_length
    return (0.0, 0.0, (KEY_ROOT_Z_M + tip) / 2), (2 * GEOMETRY.key_half_w, 2 * GEOMETRY.key_half_u, tip - KEY_ROOT_Z_M)


def _inertial(link: ET.Element, mass: float, size: tuple[float, float, float], centre: tuple[float, float, float]):
    """A solid box's inertial on `link`."""
    x, y, z = size
    fmt = lambda values: " ".join(f"{v:.9g}" for v in values)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": fmt(centre), "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": f"{mass:.9g}"})
    ET.SubElement(inertial, "inertia", {"ixx": f"{mass * (y * y + z * z) / 12:.9g}", "iyy": f"{mass * (x * x + z * z) / 12:.9g}",
                                        "izz": f"{mass * (x * x + y * y) / 12:.9g}", "ixy": "0", "ixz": "0", "iyz": "0"})


def write_key_urdf(source: str | Path, destination: str | Path, compliant: bool = False) -> Path:
    from unifp_train.insertion import GRIP_MAX_RAD

    source, destination = Path(source).resolve(), Path(destination)
    tree = ET.parse(source)
    root = tree.getroot()
    for mesh in root.iter("mesh"):
        name = mesh.get("filename", "")
        if not name.startswith(("/", "package://", "file://")):
            mesh.set("filename", str((source.parent / name).resolve()))
    link = next(l for l in root.iter("link") if l.get("name") == "Link6")
    centre, size = blade_box()
    fmt = lambda values: " ".join(f"{v:.6f}" for v in values)
    blade_link = link
    if compliant:
        # Link6 -> key_rx -> key_gimbal -> key_ry -> key, both joints at the jaw centre; the blade is placed in `key`'s
        # frame, which at zero tilt is Link6's moved up to the jaw centre.
        gimbal = ET.SubElement(root, "link", {"name": "key_gimbal"})
        _inertial(gimbal, 0.001, (0.004, 0.004, 0.004), (0.0, 0.0, 0.0))
        blade_link = ET.SubElement(root, "link", {"name": "key"})
        centre = (centre[0], centre[1], centre[2] - JAW_CENTRE_Z_M)
        _inertial(blade_link, KEY_MASS_KG, size, centre)
        for name, parent, child, origin, axis in ((KEY_JOINTS[0], "Link6", "key_gimbal", (0.0, 0.0, JAW_CENTRE_Z_M), "1 0 0"),
                                                  (KEY_JOINTS[1], "key_gimbal", "key", (0.0, 0.0, 0.0), "0 1 0")):
            joint = ET.SubElement(root, "joint", {"name": name, "type": "revolute"})
            ET.SubElement(joint, "origin", {"xyz": fmt(origin), "rpy": "0 0 0"})
            ET.SubElement(joint, "parent", {"link": parent})
            ET.SubElement(joint, "child", {"link": child})
            ET.SubElement(joint, "axis", {"xyz": axis})
            ET.SubElement(joint, "limit", {"lower": f"{-GRIP_MAX_RAD:.6f}", "upper": f"{GRIP_MAX_RAD:.6f}",
                                           "effort": "5", "velocity": "20"})
    for tag in ("collision", "visual"):
        element = ET.SubElement(blade_link, tag, {"name": "key_blade"})
        ET.SubElement(element, "origin", {"xyz": fmt(centre), "rpy": "0 0 0"})
        ET.SubElement(ET.SubElement(element, "geometry"), "box", {"size": fmt(size)})
        if tag == "visual":
            material = ET.SubElement(element, "material", {"name": "key_steel"})
            ET.SubElement(material, "color", {"rgba": "0.78 0.62 0.20 1"})
    # The handle the fingers hold: visual only, so it cannot fight the fingers.
    grip = ET.SubElement(link, "visual", {"name": "key_grip"})
    ET.SubElement(grip, "origin", {"xyz": fmt((0.0, 0.0, 0.103)), "rpy": "0 0 0"})
    ET.SubElement(ET.SubElement(grip, "geometry"), "box", {"size": fmt((0.012, 0.016, 0.045))})
    material = ET.SubElement(grip, "material", {"name": "key_grip"})
    ET.SubElement(material, "color", {"rgba": "0.10 0.10 0.12 1"})
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, xml_declaration=True, encoding="utf-8")
    return destination

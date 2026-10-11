"""The key in the hand: a copy of `d1_arm/d1.urdf` with the key's blade as a collider.

The blade's long side lies along `Link6`'s +y (the jaw axis, so the roll command turns it), its thin side along +x,
running along the approach (+z) from just past the fingertips to the tip at the jaw centre plus the key's length.
The URDF draws nothing of the key. What is drawn is `key_mesh`, a key in brass: a ring bow between the fingers, a neck,
and a blade with bitting cut into its +y edge and its tip ramped. `asset.build_key_usd` makes it a USD and `env.py`
hangs it on the blade's link (the importer drops a mesh visual's URDF colour). Beyond the fingertips it lies inside the
collider box, so the drawn key never goes where the collider does not. Two ways to hold it:

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


#: The drawn key, in Link6 (m). The bow is a ring between the fingers, its centre a little behind the jaw centre so a
#: compliant key's tilt about that point keeps it there; the neck joins it to the blade at the collider's root.
BOW_CENTRE_Z_M = 0.0985
BOW_OUTER_R_M = 0.0105
BOW_INNER_R_M = 0.0042
#: The bitting, from the blade's root towards its tip: how deep each cut goes into the +y edge (m), one per 5 mm, and
#: the ramp that brings the top edge down to the tip.
BITTING_M = (0.0008, 0.0022, 0.0010, 0.0026, 0.0006, 0.0018, 0.0012, 0.0024)
BITTING_PITCH_M = 0.005
TIP_RAMP_M = 0.004
TIP_HEIGHT_M = 0.0045


def _top_edge() -> list[tuple[float, float]]:
    """(z, y) of the key's top (+y) edge from the neck to the tip, in Link6: flat along the neck, V cuts along the
    blade, then the ramp down to the tip."""
    half_u = GEOMETRY.key_half_u
    tip = JAW_CENTRE_Z_M + GEOMETRY.key_length
    neck = BOW_CENTRE_Z_M + 0.5 * (BOW_INNER_R_M + BOW_OUTER_R_M)        # inside the ring, so the join is hidden
    points = [(neck, half_u), (KEY_ROOT_Z_M + 0.002, half_u)]
    z = KEY_ROOT_Z_M + 0.002
    for depth in BITTING_M:
        if z + BITTING_PITCH_M > tip - TIP_RAMP_M:
            break
        points += [(z + 0.5 * BITTING_PITCH_M, half_u - depth), (z + BITTING_PITCH_M, half_u)]
        z += BITTING_PITCH_M
    points += [(tip - TIP_RAMP_M, half_u), (tip, -half_u + TIP_HEIGHT_M)]
    return points


def key_link(compliant: bool) -> tuple[str, float]:
    """(the link the blade is on, the height along it of Link6's origin): where the drawn key hangs, so that a compliant
    key tilts as one piece, bow and all."""
    return ("key", -JAW_CENTRE_Z_M) if compliant else ("Link6", 0.0)


def key_mesh() -> list[tuple[tuple[float, float, float], ...]]:
    """The drawn key's triangles in Link6, outward-facing: the ring bow, and the neck and blade as one extruded
    profile between the straight -y edge and `_top_edge`, both `2 * key_half_w` thick along x."""
    import math

    t, half_u = GEOMETRY.key_half_w, GEOMETRY.key_half_u
    tris = []

    def quad(a, b, c, d):
        """Corners listed clockwise seen from outside."""
        tris.extend(((a, c, b), (a, d, c)))

    # The neck and blade: a strip of trapezoids between the bottom edge and the top edge, at the top edge's breaks.
    top = _top_edge()
    for (z0, y0), (z1, y1) in zip(top, top[1:]):
        quad((t, -half_u, z0), (t, -half_u, z1), (t, y1, z1), (t, y0, z0))              # +x face
        quad((-t, -half_u, z0), (-t, y0, z0), (-t, y1, z1), (-t, -half_u, z1))          # -x face
        quad((t, y0, z0), (t, y1, z1), (-t, y1, z1), (-t, y0, z0))                      # top edge
        quad((-t, -half_u, z0), (-t, -half_u, z1), (t, -half_u, z1), (t, -half_u, z0))  # bottom edge
    (za, ya), (zb, yb) = top[0], top[-1]
    quad((t, -half_u, za), (t, ya, za), (-t, ya, za), (-t, -half_u, za))                # neck end, in the bow
    quad((t, -half_u, zb), (-t, -half_u, zb), (-t, yb, zb), (t, yb, zb))                # tip end
    # The bow: a ring in the y-z plane.
    n = 32
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        ring = lambda r, a, x: (x, r * math.cos(a), BOW_CENTRE_Z_M + r * math.sin(a))
        ro, ri = BOW_OUTER_R_M, BOW_INNER_R_M
        quad(ring(ro, a0, t), ring(ri, a0, t), ring(ri, a1, t), ring(ro, a1, t))       # +x face
        quad(ring(ro, a0, -t), ring(ro, a1, -t), ring(ri, a1, -t), ring(ri, a0, -t))   # -x face
        quad(ring(ro, a0, t), ring(ro, a1, t), ring(ro, a1, -t), ring(ro, a0, -t))     # outside
        quad(ring(ri, a0, t), ring(ri, a0, -t), ring(ri, a1, -t), ring(ri, a1, t))     # the hole
    return tris


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
    collision = ET.SubElement(blade_link, "collision", {"name": "key_blade"})
    ET.SubElement(collision, "origin", {"xyz": fmt(centre), "rpy": "0 0 0"})
    ET.SubElement(ET.SubElement(collision, "geometry"), "box", {"size": fmt(size)})
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, xml_declaration=True, encoding="utf-8")
    return destination

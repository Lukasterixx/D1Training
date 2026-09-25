"""L-shaped claw fingers for the D1: each pincer gets a lip at its tip, turned 90 degrees inward.

Lukas's hardware change (2026-09-25): "turn each straight pincer into an L shape with a 90 degree lip facing
inwards by 2 cm. The pincer will still be able to close since these lips will pass by each other" -- and the
lips sit *beyond* the pincers, "so it overbites and doesn't obstruct a full close". So the lip is real geometry
here, not a spring: this module writes a copy of `d1_arm/d1.urdf` with a box collider (and a
matching visual) on the tip of each finger, and PhysX does the rest -- the bar is held only if it is inside the
loop the fingers, the lips and the palm make, it slides along the lever with nothing but friction, and it comes
off the lever's end if it slides that far.

Geometry, in the Link6 frame with the jaws shut (measured off the CAD meshes, `demos/cup/pick_demo/grasp.py`;
checked against the STL meshes by `tests/test_mech_demo.py`): the approach is +z, the jaws open along y, the
fingers are 26 mm wide in x (-12.6 to +13.4 mm), their inner faces at y = -/+8.6 mm, their end faces at z = 125.6
mm spanning |y| = 9.6-15.6 mm, the palm at z = 76 mm. Each lip is an L's foot on its fingertip:

  * a `LIP_THICKNESS_M` plate *beyond* the ends of both fingers (`LIP_OVERBITE_CLEARANCE_M` past them), so at full
    close it passes in front of the opposite finger's end -- an overbite -- and nothing stops the jaws closing;
  * from its own finger's outer edge across to `LIP_LENGTH_M` past its inner face;
  * over one half of the finger's width, `Link7_1`'s the +x half and `Link7_2`'s the -x half, with
    `LIP_CLEARANCE_M` between them, so the two pass each other. With the jaws across a bar the bar runs along x,
    so between them the two halves close the loop along the whole finger width.

So the pincers still close fully onto a bar (and grip it), and the lips close the loop in front of it. What they
cost: the bar has to pass between the lip tips to get in -- `entry_gap_m(travel)`, 37.2 mm with the jaws fully
open, 1.2 mm at the 41.2 mm the friction grip approaches with.

Plain Python and numpy (no Isaac import), so it can be checked without a simulator.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

#: How far each lip reaches inward past its finger's inner face, and how thick it is. The length is Lukas's; the
#: thickness, the clearance between the two halves and the clearance past the finger ends are assumptions.
LIP_LENGTH_M = 0.020
LIP_THICKNESS_M = 0.004
LIP_CLEARANCE_M = 0.001
LIP_OVERBITE_CLEARANCE_M = 0.0005

#: Finger geometry in Link6 at zero travel (see the module docstring).
FINGER_INNER_Y_M = 0.0086
FINGER_X_RANGE_M = (-0.0126, 0.0134)
FINGERTIP_Z_M = 0.1251
#: Where the fingers' end faces actually are (the STL's furthest point), and how far out they reach in |y|.
FINGER_END_Z_M = 0.1256
FINGER_END_OUTER_Y_M = 0.0156
PALM_Z_M = 0.076
CLOSED_GAP_M = 2 * FINGER_INNER_Y_M          # 17.2 mm
MAX_TRAVEL_M = 0.03

#: The two finger joints as the URDF places them on Link6: (xyz, rpy) of the joint origin, and which side the
#: finger is on (-1 for -y) and which half of the width its lip takes (+1 for +x).
FINGERS = {
    "Link7_1": {"xyz": (-0.0056012, -0.029636, 0.0706), "rpy": (-1.5714, -1.5708, 0.0), "side": -1, "half": +1},
    "Link7_2": {"xyz": (-0.0056388, 0.02964, 0.0706), "rpy": (1.5702, -1.5708, 0.0), "side": +1, "half": -1},
}

#: The lips' inner face: a bar is held between it and the palm.
LIP_FACE_Z_M = FINGER_END_Z_M + LIP_OVERBITE_CLEARANCE_M
#: Travel per finger below which the lips would stop the jaws. Beyond the fingertips they overbite, so none.
LIP_STOP_TRAVEL_M = 0.0


def gap_m(travel_m: float) -> float:
    """Distance between the fingers' inner faces at `travel_m` per finger."""
    return CLOSED_GAP_M + 2 * travel_m


def entry_gap_m(travel_m: float) -> float:
    """Distance between the two lip tips: what an object must pass through to get into the loop."""
    return max(0.0, gap_m(travel_m) - 2 * LIP_LENGTH_M)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF's fixed-axis roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_rpy(rot: np.ndarray) -> tuple[float, float, float]:
    """The inverse of `rpy_matrix` (away from pitch = +/-90 degrees)."""
    pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    if abs(math.cos(pitch)) < 1e-6:
        return math.atan2(-rot[1, 2], rot[1, 1]), pitch, 0.0
    return math.atan2(rot[2, 1], rot[2, 2]), pitch, math.atan2(rot[1, 0], rot[0, 0])


def lip_box_link6(finger: str) -> tuple[np.ndarray, np.ndarray]:
    """(centre, size) of `finger`'s lip in the Link6 frame, jaws shut, axes aligned with Link6's."""
    spec = FINGERS[finger]
    side, half = spec["side"], spec["half"]
    x_lo, x_hi = FINGER_X_RANGE_M
    mid = (x_lo + x_hi) / 2
    xs = (mid + LIP_CLEARANCE_M / 2, x_hi) if half > 0 else (x_lo, mid - LIP_CLEARANCE_M / 2)
    # From the finger's outer edge, across its end, to the lip's length past its inner face (toward -side).
    ys = sorted((side * FINGER_END_OUTER_Y_M, side * FINGER_INNER_Y_M - side * LIP_LENGTH_M))
    zs = (LIP_FACE_Z_M, LIP_FACE_Z_M + LIP_THICKNESS_M)
    centre = np.array([sum(xs) / 2, sum(ys) / 2, sum(zs) / 2])
    size = np.array([xs[1] - xs[0], ys[1] - ys[0], zs[1] - zs[0]])
    return centre, size


def lip_box_finger(finger: str) -> tuple[np.ndarray, tuple[float, float, float], np.ndarray]:
    """(xyz, rpy, size) of the lip in the finger link's own frame, for the URDF's collision origin."""
    spec = FINGERS[finger]
    rot = rpy_matrix(*spec["rpy"])
    centre, size = lip_box_link6(finger)
    xyz = rot.T @ (centre - np.asarray(spec["xyz"]))
    # The box's axes are Link6's, so in the finger's frame the box is rotated by R^T.
    return xyz, matrix_rpy(rot.T), size


def loop_region(travel_m: float) -> dict:
    """Where a bar is held, in Link6 at `travel_m`: between the fingers, the palm and the lips."""
    return {"half_gap_m": gap_m(travel_m) / 2, "z_range_m": (PALM_Z_M, LIP_FACE_Z_M), "x_range_m": FINGER_X_RANGE_M}


def write_claw_urdf(source: str | Path, destination: str | Path) -> Path:
    """Copy `source` (d1.urdf) to `destination` with a lip collider and visual on each finger.

    Mesh paths are rewritten absolute, since the copy does not live beside `d1_arm/meshes/`.
    """
    source, destination = Path(source).resolve(), Path(destination)
    tree = ET.parse(source)
    root = tree.getroot()
    for mesh in root.iter("mesh"):
        name = mesh.get("filename", "")
        if not name.startswith(("/", "package://", "file://")):
            mesh.set("filename", str((source.parent / name).resolve()))
    for link in root.iter("link"):
        name = link.get("name")
        if name not in FINGERS:
            continue
        xyz, rpy, size = lip_box_finger(name)
        origin = {"xyz": " ".join(f"{v:.6f}" for v in xyz), "rpy": " ".join(f"{v:.6f}" for v in rpy)}
        for tag in ("collision", "visual"):
            element = ET.SubElement(link, tag, {"name": f"{name}_lip"})
            ET.SubElement(element, "origin", origin)
            geometry = ET.SubElement(element, "geometry")
            ET.SubElement(geometry, "box", {"size": " ".join(f"{v:.6f}" for v in size)})
            if tag == "visual":
                material = ET.SubElement(element, "material", {"name": "claw_lip"})
                ET.SubElement(material, "color", {"rgba": "0.85 0.45 0.15 1"})
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, xml_declaration=True, encoding="utf-8")
    return destination

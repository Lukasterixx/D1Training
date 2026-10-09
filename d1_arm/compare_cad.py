#!/usr/bin/env python3
"""The D1's dimensions in Unitree's CAD against the URDFs: where `d1.urdf`'s arm is the wrong length.

    python3 d1_arm/compare_cad.py <D1-550 STEP> [<d1_550_description.urdf>]

The STEP is Unitree's "D1-550 STEP" (`D1舵机臂-简化B(1).stp`), and the URDF their "D1-550 URDF", both linked from the D1
developer page (support.unitree.com/home/en/developer/D1Arm_services; the archives are listed in
`third_party/unitree_d1_cad/NOTICE.md`). Neither is in this repository. Unitree's "STEP simplified model" (`D1-舵机臂.STEP`)
on the same page is the same arm in the same pose: the same parts, at the same places.

What it does, with plain regular expressions on the STEP text (`description/mount_from_cad.read_step`; the D1-550 STEP
puts every body in one frame, so no assembly transforms are needed):

  1. Finds the joint axes as lines on which many cylindrical faces are coaxial (servo housings, output flanges,
     bearings). In the CAD's pose the arm lies along -y: Joint1 vertical through the origin, the pitch joints (2, 3, 5)
     on lines along x, the rolls (4, 6) on one line along y, the forearm folded back over the upper arm.
  2. Finds the base: its bottom face (the lowest vertex), its four mounting holes (vertical cylinders of radius 2.25 mm).
  3. Finds the fingertips: the furthest vertex along the roll line, past the wrist.
  4. Computes the same quantities from each URDF's own forward kinematics at q = 0 and its meshes, and prints them side
     by side. All are independent of pose: axis-to-axis distances, the shoulder's height above the base's bottom face,
     the reach from the wrist pitch to the fingertips.
"""
from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "description"))
from mount_from_cad import read_step  # noqa: E402


# --- the CAD --------------------------------------------------------------------------------------

def coaxial(cyl: np.ndarray, axis: int, near, tol_mm: float = 1.5, rmin_mm: float = 3.0):
    """The mean line of the cylinders on `axis` (0 x, 1 y, 2 z) passing within `tol_mm` of `near` (the other two
    coordinates), and how many there are. Small cylinders (threads, pins) are left out."""
    r, p, d = cyl[:, 0], cyl[:, 1:4], cyl[:, 4:7]
    other = [i for i in range(3) if i != axis]
    on = (np.abs(np.abs(d[:, axis]) - 1.0) < 1e-4) & (r >= rmin_mm)
    close = np.hypot(p[:, other[0]] - near[0], p[:, other[1]] - near[1]) < tol_mm
    sel = on & close
    return p[sel][:, other].mean(0), int(sel.sum()), sorted({round(float(v), 2) for v in r[sel]})


def cad_dimensions(step: Path) -> dict:
    cyl, vert = read_step(step)
    r, p, d = cyl[:, 0], cyl[:, 1:4], cyl[:, 4:7]
    out = {}
    j1, n1, _ = coaxial(cyl, 2, (0.0, 0.0))
    j2, n2, r2 = coaxial(cyl, 0, (0.0, 125.8))
    j3, n3, r3 = coaxial(cyl, 0, (-270.0, 125.8))
    j5, n5, _ = coaxial(cyl, 0, (-65.3, 167.2))
    j4, n4, _ = coaxial(cyl, 1, (0.5, 167.1), tol_mm=0.3)      # the forearm's roll (Joint4)
    j6, n6, _ = coaxial(cyl, 1, (0.4, 166.5), tol_mm=0.3)      # the wrist's roll (Joint6)
    base = vert[(np.abs(vert[:, 0]) < 80) & (np.abs(vert[:, 1]) < 80) & (vert[:, 2] < 20)]
    bottom = base[:, 2].min()
    holes = p[(np.abs(d[:, 2]) > 0.999) & (np.abs(r - 2.25) < 0.01) & (p[:, 2] < 0)][:, :2]
    holes = np.unique(np.round(holes, 2), axis=0)
    hand = vert[(vert[:, 1] > 60) & (vert[:, 2] > 140) & (vert[:, 2] < 210) & (np.abs(vert[:, 0]) < 20)]
    out["counts"] = dict(J1=n1, J2=n2, J3=n3, J4=n4, J5=n5, J6=n6)
    out["shoulder and elbow servos alike"] = r2 == r3
    out["mounting holes along x, across y (mm)"] = (float(np.ptp(holes[:, 1])), float(np.ptp(holes[:, 0])))
    out["J2 axis above the base's bottom face"] = j2[1] - bottom
    out["J2 axis off the J1 axis"] = abs(j2[0] - j1[1])
    out["upper arm, J2 -> J3"] = abs(j2[0] - j3[0])
    out["elbow offset, J3 axis -> forearm roll line"] = j4[1] - j3[1]
    out["forearm, J3 -> J5 along the roll line"] = abs(j5[0] - j3[0])
    out["J5 axis off the roll line"] = abs(j5[1] - j4[1])
    out["J5 -> fingertip, jaws closed"] = hand[:, 1].max() - j5[0]
    out["wrist roll (J6) line off the forearm roll (J4) line"] = float(np.hypot(*(j6 - j4)))
    return out


# --- a URDF ---------------------------------------------------------------------------------------

def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _line_dist(p1, d1, p2, d2):
    n = np.cross(d1, d2)
    if np.linalg.norm(n) < 1e-9:
        return float(np.linalg.norm(np.cross(p2 - p1, d1)))
    return float(abs((p2 - p1) @ n) / np.linalg.norm(n))


def _stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    tri = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return tri["v"].reshape(-1, 3).astype(float)


def urdf_dimensions(urdf: Path) -> dict:
    root = ET.parse(urdf).getroot()
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    frames = {}

    def world(link):
        if link not in frames:
            if link not in by_child:
                frames[link] = np.eye(4)
            else:
                j = by_child[link]
                o = j.find("origin")
                a = np.eye(4)
                a[:3, :3] = _rpy(*map(float, o.get("rpy").split()))
                a[:3, 3] = list(map(float, o.get("xyz").split()))
                frames[link] = world(j.find("parent").get("link")) @ a
        return frames[link]

    joints = {}
    for child, j in by_child.items():
        t = world(child)
        axis = t[:3, :3] @ np.array(list(map(float, j.find("axis").get("xyz").split())))
        joints[j.get("name")] = (t[:3, 3] * 1000.0, axis / np.linalg.norm(axis), child, j.get("type"))
    meshes = {}
    for link in root.findall("link"):
        m = link.find("visual/geometry/mesh")
        if m is None:
            continue
        name = os.path.basename(m.get("filename"))
        for folder in (urdf.parent / "meshes", urdf.parent.parent / "meshes", urdf.parent):
            if (folder / name).exists():
                v = _stl_vertices(folder / name)
                meshes[link.get("name")] = (world(link.get("name")) @ np.c_[v, np.ones(len(v))].T).T[:, :3] * 1000.0
                break
    revolute = [joints[n] for n in sorted(joints) if joints[n][3] == "revolute"]
    (p1, d1, *_), (p2, d2, *_), (p3, d3, *_), (p4, d4, *_), (p5, d5, *_), (p6, d6, l6, _) = revolute
    bottom = meshes["base_link"][:, 2].min()
    foot3 = p4 + ((p3 - p4) @ d4) * d4
    foot5 = p4 + ((p5 - p4) @ d4) * d4
    out_dir = d6 if (p6 - p5) @ d6 > 0 else -d6
    fingers = [child for (_, _, child, kind) in joints.values() if kind == "prismatic"]
    out = {}
    base = meshes["base_link"]
    out["J1 frame above the base's bottom face"] = p1[2] - bottom
    out["J2 axis above the base's bottom face"] = p2[2] - bottom
    out["J2 axis off the J1 axis"] = _line_dist(p1, d1, p2, d2)
    out["upper arm, J2 -> J3"] = _line_dist(p2, d2, p3, d3)
    out["elbow offset, J3 axis -> forearm roll line"] = _line_dist(p3, d3, p4, d4)
    out["forearm, J3 -> J5 along the roll line"] = float(abs((foot5 - foot3) @ d4))
    out["J5 axis off the roll line"] = _line_dist(p4, d4, p5, d5)
    out["J5 -> J6 joint frame along the roll line"] = float(abs((p6 - p5) @ d6))
    out["J5 -> fingertip, jaws closed"] = max(float(((meshes[f] - p5) @ out_dir).max()) for f in fingers)
    out["wrist roll (J6) line off the forearm roll (J4) line"] = _line_dist(p4, d4, p6, d6)
    out["base footprint x, y (mm)"] = tuple(float(v) for v in np.round(np.ptp(base[:, :2], axis=0), 1))
    out["effort limits J1..J6 (N m)"] = tuple(float(j.find("limit").get("effort")) for j in
                                               sorted((j for j in root.findall("joint") if j.get("type") == "revolute"),
                                                      key=lambda j: j.get("name")))
    return out


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    cad = cad_dimensions(Path(sys.argv[1]))
    urdfs = {"d1.urdf (this repo)": REPO / "d1_arm/d1.urdf"}
    if len(sys.argv) > 2:
        urdfs["Unitree D1-550 URDF"] = Path(sys.argv[2])
    models = {name: urdf_dimensions(path) for name, path in urdfs.items()}
    print("coaxial faces per joint axis in the CAD:", cad.pop("counts"))
    print("shoulder (J2) and elbow (J3) servos have the same cylinders:", cad.pop("shoulder and elbow servos alike"))
    print("CAD mounting holes along, across the rail (mm):", cad.pop("mounting holes along x, across y (mm)"))
    keys = list(dict.fromkeys(list(cad) + [k for m in models.values() for k in m]))
    head = f"{'mm':52s} {'CAD':>9s}" + "".join(f" {n[:24]:>26s}" for n in models)
    print(head)
    for k in keys:
        c = cad.get(k)
        row = f"{k:52s} {('%9.2f' % c) if isinstance(c, float) else ' ' * 9}"
        for m in models.values():
            v = m.get(k)
            if isinstance(v, float):
                delta = f" ({v - c:+.2f})" if isinstance(c, float) else ""
                row += f" {('%.2f' % v) + delta:>26s}"
            elif v is not None:
                row += f" {str(v):>26s}"
            else:
                row += " " * 27
        print(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

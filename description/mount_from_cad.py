#!/usr/bin/env python3
"""Where the D1 sits on the Go2 EDU: the arm mount, measured off Unitree's CAD rather than guessed.

    python3 description/mount_from_cad.py <path to GO2简化-带拓展坞-0911.stp>

The STEP file is Unitree's "GO2 simplified, with expansion dock" (2023-09-11), from
https://oss-global-cdn.unitree.com/static/GO2%E7%AE%80%E5%8C%96-%E5%B8%A6%E6%8B%93%E5%B1%95%E5%9D%9E-0911.7z
(linked on the MYBOTSHOP forum, 2026-07-06). It is not in the repository; download and unpack it (7-zip) first.

What it does, with plain regular expressions on the STEP text (no CAD kernel):

  1. Registers the CAD frame to the URDF's `base_link`: the four thigh (hip-pitch) motors are the large cylinders on
     lateral axes, two lines that the URDF puts at x = +/-0.1934 m, z = 0; the hip-roll motors are on fore-aft axes
     at y = +/-0.0465 m. Forward is the side the body reaches furthest from the hips (the head).
  2. Finds the expansion dock's rails: the two T-slot rails on its top, and the four M3 screws that fasten them to the
     dog (vertical holes of radius 1.7 mm on the rails' centrelines).
  3. Finds the D1's four M4 mounting holes in `d1_arm/meshes/base_link.STL` (radius 2.25 mm, in its bottom face, which
     is the D1 base frame's z = 0).

and prints the mount under Lukas's description (2026-09-26): the D1 "vertically on top of these rails", each of its
screws 1 cm forward of a rail screw.
"""
from __future__ import annotations

import collections
import re
import struct
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
URDF_THIGH_X_M = 0.1934
URDF_HIP_Y_M = 0.0465
NUM = r"([-+0-9.Ee]+)"


def read_step(path: Path):
    pts, dirs, axes, cyls, verts = {}, {}, {}, [], []
    re_pt = re.compile(rf"^#(\d+)=CARTESIAN_POINT\('[^']*',\({NUM},{NUM},{NUM}\)\);")
    re_dr = re.compile(rf"^#(\d+)=DIRECTION\('[^']*',\({NUM},{NUM},{NUM}\)\);")
    re_ax = re.compile(r"^#(\d+)=AXIS2_PLACEMENT_3D\('[^']*',#(\d+),#(\d+),(?:#\d+|\$)\);")
    re_cyl = re.compile(rf"^#(\d+)=CYLINDRICAL_SURFACE\('[^']*',#(\d+),{NUM}\);")
    re_vp = re.compile(r"^#(\d+)=VERTEX_POINT\('[^']*',#(\d+)\);")
    with open(path, encoding="latin-1") as handle:
        for line in handle:
            if (m := re_pt.match(line)):
                pts[int(m[1])] = (float(m[2]), float(m[3]), float(m[4]))
            elif (m := re_dr.match(line)):
                dirs[int(m[1])] = (float(m[2]), float(m[3]), float(m[4]))
            elif (m := re_ax.match(line)):
                axes[int(m[1])] = (int(m[2]), int(m[3]))
            elif (m := re_cyl.match(line)):
                cyls.append((int(m[2]), float(m[3])))
            elif (m := re_vp.match(line)):
                verts.append(int(m[2]))
    cyl = np.array([(r, *pts[axes[a][0]], *dirs[axes[a][1]]) for a, r in cyls
                    if a in axes and axes[a][0] in pts and axes[a][1] in dirs])
    vert = np.array([pts[v] for v in verts if v in pts])
    return cyl, vert


def d1_holes() -> np.ndarray:
    data = (REPO / "d1_arm/meshes/base_link.STL").read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    tri = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    v = tri["v"].reshape(-1, 3).astype(float)
    bottom = np.unique(np.round(v[np.abs(v[:, 2]) < 1e-5][:, :2], 5), axis=0)
    holes = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            # The four exact circles of radius 2.25 mm; seed each search at its quadrant.
            near = bottom[(np.sign(bottom[:, 0]) == sx) & (np.sign(bottom[:, 1]) == sy)]
            for c in near:
                ring = near[np.abs(np.linalg.norm(near - c, axis=1) - 0.0045) < 1e-4]
                if len(ring) >= 1:
                    group = near[np.linalg.norm(near - near[np.argmin(np.linalg.norm(near - c, axis=1))], axis=1) < 0.005]
                    centre = group.mean(0)
                    radius = np.linalg.norm(group - centre, axis=1)
                    if abs(radius.mean() - 0.00225) < 1e-4 and radius.std() < 1e-5:
                        holes.append(centre)
                        break
    return np.unique(np.round(np.array(holes), 5), axis=0)


def register(cyl: np.ndarray, vert: np.ndarray) -> dict:
    """Step 1: the CAD frame in the URDF's `base_link`, from the thigh (lateral) and hip-roll (fore-aft) motor axes.

    `base_link` is the thigh axes' midpoint at their height; forward is the side the body reaches furthest (the head).
    Returns the registration and the two numbers it can be checked by against the URDF.
    """
    r, p, d = cyl[:, 0], cyl[:, 1:4], cyl[:, 4:7]
    thigh = (np.abs(d[:, 0]) > 0.999) & (r > 30) & (r < 55)
    ys = p[thigh][:, 1]
    split = (ys.min() + ys.max()) / 2
    y_a, y_b = np.median(ys[ys < split]), np.median(ys[ys >= split])
    z0 = np.median(p[thigh][:, 2])
    y0 = (y_a + y_b) / 2
    roll = (np.abs(d[:, 1]) > 0.999) & (r > 25) & (r < 55)
    hip_x = collections.Counter(np.round(np.abs(p[roll][:, 0]), 1)).most_common(1)[0][0]
    body = vert[vert[:, 2] > z0 + 50]
    forward = -1.0 if (y0 - body[:, 1].min()) > (body[:, 1].max() - y0) else 1.0
    return {"y0_mm": float(y0), "z0_mm": float(z0), "forward": forward,
            "thigh_span_mm": float(y_b - y_a), "hip_x_mm": float(hip_x)}


def base_from_cad(frame: dict) -> np.ndarray:
    """4x4: a point in the CAD's axes, metres (what a tessellation of the STEP comes out in), to `base_link`, metres."""
    fw = frame["forward"]
    out = np.eye(4)
    out[:3, :3] = [[0.0, fw, 0.0], [-fw, 0.0, 0.0], [0.0, 0.0, 1.0]]
    out[:3, 3] = [-fw * frame["y0_mm"] / 1000.0, 0.0, -frame["z0_mm"] / 1000.0]
    return out


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    cyl, vert = read_step(Path(sys.argv[1]))
    r, p, d = cyl[:, 0], cyl[:, 1:4], cyl[:, 4:7]
    # 1. The frame: thigh motors on lateral (CAD x) axes, hip-roll motors on fore-aft (CAD y) axes.
    frame = register(cyl, vert)
    z0, forward = frame["z0_mm"], frame["forward"]
    T = base_from_cad(frame)
    to_base = lambda q: T[:3, :3] @ (np.asarray(q) / 1000.0) + T[:3, 3]
    print(f"frame: thigh axes {frame['thigh_span_mm']:.1f} mm apart (URDF {2000 * URDF_THIGH_X_M:.1f}), hip-roll axes at "
          f"+/-{frame['hip_x_mm']:.1f} mm (URDF +/-{1000 * URDF_HIP_Y_M:.1f}), forward is CAD {'-' if forward < 0 else '+'}y")
    # 2. The rails and their screws.
    screws = (np.abs(d[:, 2]) > 0.999) & (np.abs(r - 1.7) < 0.05) & (np.abs(np.abs(p[:, 0]) - 29.0) < 0.5)
    rail_screws = np.unique(np.round(np.array([to_base(q) for q in p[screws]])[:, :2], 4), axis=0)
    rail = vert[(np.abs(np.abs(vert[:, 0]) - 29.0) < 7) & (vert[:, 2] > z0 + 80)]
    rail_top = (rail[:, 2].max() - z0) / 1000.0
    rail_x = sorted(to_base((0.0, y, 0.0))[0] for y in (rail[:, 1].min(), rail[:, 1].max()))
    print(f"rails: centred at y = +/-{1000 * abs(rail_screws[0, 1]):.1f} mm, from x = {1000 * rail_x[0]:.1f} to "
          f"{1000 * rail_x[1]:.1f} mm, top at z = {1000 * rail_top:.1f} mm")
    print("rail screws (base_link, mm):", [tuple(float(v) for v in np.round(1000 * s, 1)) for s in rail_screws])
    # 3. The D1's holes, and the mount.
    holes = d1_holes()
    print("D1 holes (its base frame, mm):", [tuple(float(v) for v in np.round(1000 * h, 1)) for h in holes])
    d1_half_x = np.abs(holes[:, 0]).max()
    rear, front = rail_screws[:, 0].min(), rail_screws[:, 0].max()
    for name, x in (("rear D1 screws 1 cm ahead of the rear rail screws", rear + 0.01 + d1_half_x),
                    ("front D1 screws 1 cm ahead of the front rail screws", front + 0.01 - d1_half_x)):
        ok = rail_x[0] <= x - d1_half_x and x + d1_half_x <= rail_x[1]
        print(f"  {name}: mount = ({x:+.4f}, 0.0000, {rail_top:.4f}) m"
              f"{'' if ok else '  -- a D1 screw would be off the end of the rails'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

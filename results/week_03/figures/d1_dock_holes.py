#!/usr/bin/env python3
"""Do the D1's four mounting holes line up with the four screws that fasten the dock's rails? A top view, from the
committed meshes only: Unitree's base at F-117's mount (what 2026-10-01 checked), and the measured base at the mount that
puts its holes on the rails' screws (F-125, F-126).

    python3 results/week_03/figures/d1_dock_holes.py

Reads `description/meshes/go2/expansion_dock_rails.stl` (Unitree's CAD, in the Go2's `base_link`, F-117),
`d1_arm/meshes/base_link.STL` and `base_link_measured.STL` (the D1 base, origin on its bottom face) and the mounts in
`mount.py`. Writes `d1_dock_holes.png` beside itself and prints each D1 hole's offset from the nearest
rail screw. The screw and hole centres are fitted from the meshes, not typed in.
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from matplotlib.patches import Circle, Ellipse  # noqa: E402

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO))
from dock import read_binary_stl  # noqa: E402
from mount import ARM_MOUNT_POS_M, DOCK_CAD_ARM_MOUNT_POS_M  # noqa: E402

D1_BLUE, DOCK_ORANGE = "#2a78d6", "#eb6834"  # categorical slots 1 and 2
RAIL_FILL, INK, MUTED = "#b4b3ad", "#0b0b0b", "#52514e"


def normals(tri: np.ndarray) -> np.ndarray:
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-15)


def circle_fit(xy: np.ndarray) -> tuple[np.ndarray, float]:
    a = np.c_[2 * xy, np.ones(len(xy))]
    sol = np.linalg.lstsq(a, (xy ** 2).sum(1), rcond=None)[0]
    return sol[:2], float(np.sqrt(sol[2] + sol[0] ** 2 + sol[1] ** 2))


def vertical_holes(tri: np.ndarray, radius_m, z_range_m: tuple[float, float]) -> np.ndarray:
    """Centres (x, y) of the vertical walls between `z_range_m` that are circles of `radius_m`, or axis-aligned
    ellipses of half-axes `radius_m` = (rx, ry), each to +/-0.15 mm.

    A wall is a connected patch of vertical triangles (sharing vertices). Patches that are not that shape (the rails'
    T-slots, the base's sides) fail the fit and are left out. The measured base's holes are ellipses: its flange is
    scaled 1.20 along the rail and 1.0 across it at the tabs (`d1_arm/measured_base.py`).
    """
    rx, ry = (radius_m, radius_m) if np.isscalar(radius_m) else radius_m
    wall = tri[(np.abs(normals(tri)[:, 2]) < 0.05)]
    wall = wall[(wall[:, :, 2].min(1) >= z_range_m[0] - 1e-6) & (wall[:, :, 2].max(1) <= z_range_m[1] + 1e-6)]
    parent = list(range(len(wall)))

    def root(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    owner: dict[tuple, int] = {}
    for i, t in enumerate(wall):
        for p in t:
            key = tuple(np.round(p, 6))
            if key in owner:
                parent[root(i)] = root(owner[key])
            else:
                owner[key] = i
    patches: dict[int, list[int]] = {}
    for i in range(len(wall)):
        patches.setdefault(root(i), []).append(i)
    centres = []
    for idx in patches.values():
        xy = wall[idx].reshape(-1, 3)[:, :2]
        centre, r = circle_fit(xy / [rx, ry])
        centre = centre * [rx, ry]
        if abs(r - 1.0) < 1.5e-4 / min(rx, ry) and np.abs(np.linalg.norm((xy - centre) / [rx, ry], axis=1) - 1.0).max() < 1.5e-4 / min(rx, ry):
            centres.append(centre)
    return np.unique(np.round(np.array(centres), 4), axis=0)


def load_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    tri = np.frombuffer(data[84:84 + 50 * count], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return tri["v"].reshape(-1, 3, 3).astype(float)


#: (title, D1 base mesh, its holes' radius in m, mount): what 2026-10-01 checked, then the base Lukas measured (F-126)
#: at the mount that puts its holes on the rails' screws (F-125).
PANELS = (
    ("2026-10-01: Unitree's base at F-117's mount", "d1_arm/meshes/base_link.STL", 0.00225, DOCK_CAD_ARM_MOUNT_POS_M),
    ("Since 2026-10-06: the measured base at the rail-screw mount", "d1_arm/meshes/base_link_measured.STL",
     (0.0027, 0.00225), ARM_MOUNT_POS_M),
)


def panel(ax, title: str, mesh: str, hole_r: float, mount_pos, rails: np.ndarray) -> None:
    d1 = load_stl(REPO / mesh)
    mount = np.array(mount_pos)
    # The rail screws: M3 clearance (r 1.7 mm) through the rails' feet; the D1's holes: clearance holes through its
    # bottom flange (the measured base's are Unitree's scaled 1.2 in plan), moved to where the weld puts them.
    screws = vertical_holes(rails, 0.0017, (0.070, 0.086))
    holes = vertical_holes(d1, hole_r, (-0.001, 0.004)) + mount[:2]
    rail_top = rails[:, :, 2].max()
    # The rails' upper 5.5 mm, as `description/mount_from_cad.py` measures their length (z above 80 mm).
    upper = rails[rails[:, :, 2].min(1) > 0.080]
    tops = upper[normals(upper)[:, 2] > 0.5]
    rail_front = upper[:, :, 0].max()
    bottom = d1[(normals(d1)[:, 2] < -0.99) & (np.abs(d1[:, :, 2]).max(1) < 1e-5)]

    if len(screws) != 4 or len(holes) != 4:
        raise SystemExit(f"{mesh}: expected 4 rail screws and 4 D1 holes, found {len(screws)} and {len(holes)}")
    print(f"== {title}\nmount: {np.round(1000 * mount, 1).tolist()} mm; base {mesh}; rail tops at "
          f"{1000 * rail_top:.1f} mm; rail tops end at x = {1000 * rail_front:+.1f} mm")
    print("rail screws (base_link, mm):", [tuple(np.round(1000 * s, 1).tolist()) for s in screws])
    print("D1 holes at the mount (base_link, mm):", [tuple(np.round(1000 * h, 1).tolist()) for h in holes])
    for h in holes:
        s = screws[np.argmin(np.linalg.norm(screws - h, axis=1))]
        d = 1000 * (h - s)
        print(f"  D1 hole ({1000 * h[0]:+.1f}, {1000 * h[1]:+.1f}) -> nearest rail screw ({1000 * s[0]:+.1f}, "
              f"{1000 * s[1]:+.1f}): dx {d[0]:+.1f} mm, dy {d[1]:+.1f} mm"
              f"{'  (past the rails end)' if h[0] > rail_front else ''}")
    print(f"fore-aft spacing: D1 holes {1000 * np.ptp(holes[:, 0]):.1f} mm, rail screws {1000 * np.ptp(screws[:, 0]):.1f} mm;"
          f" across: {1000 * np.ptp(holes[:, 1]):.1f} mm, {1000 * np.ptp(screws[:, 1]):.1f} mm")

    mm = 1000.0
    ax.add_collection(PolyCollection(mm * tops[:, :, :2], facecolor=RAIL_FILL, edgecolor="none"))
    ax.add_collection(PolyCollection(mm * (bottom[:, :, :2] + mount[:2]), facecolor=D1_BLUE, edgecolor="none",
                                     alpha=0.10))
    for s in screws:
        ax.add_patch(Circle(mm * s, 3.25, facecolor="white", edgecolor=DOCK_ORANGE, lw=2))
        ax.add_patch(Circle(mm * s, 1.7, facecolor=DOCK_ORANGE, edgecolor="none"))
    for h in holes:
        rx, ry = (hole_r, hole_r) if np.isscalar(hole_r) else hole_r
        ax.add_patch(Ellipse(mm * h, 2 * mm * rx, 2 * mm * ry, facecolor="none", edgecolor=D1_BLUE, lw=2))
        ax.plot(*(mm * h), marker="+", color=D1_BLUE, ms=8, mew=1.5)
    ax.axvline(mm * rail_front, color=MUTED, lw=1, ls=(0, (4, 3)))
    ax.text(mm * rail_front + 1.5, -70, f"rails end\nx = {mm * rail_front:+.1f}", color=MUTED, fontsize=8, va="bottom")
    # The two fore-aft gaps, on the +y pair.
    y = max(29.0, holes[:, 1].max() * mm) + 9.0
    for hx, sx in ((holes[:, 0].max(), screws[:, 0].max()), (holes[:, 0].min(), screws[:, 0].min())):
        ax.annotate("", xy=(mm * hx, y), xytext=(mm * sx, y),
                    arrowprops=dict(arrowstyle="<->", color=INK, lw=1, shrinkA=0, shrinkB=0))
        ax.text(mm * (hx + sx) / 2, y + 1.5, f"{mm * (hx - sx):+.1f} mm", ha="center", va="bottom", fontsize=9,
                color=INK)
    ax.text(mm * screws[:, 0].min(), -48, "rail screws (M3)\nx = {:+.1f}, {:+.1f}".format(*(mm * np.sort(screws[:, 0])[[0, -1]])),
            color=INK, fontsize=8, ha="center", va="top")
    ax.text(mm * mount[0], -48, "D1 holes at the weld\nx = {:+.1f}, {:+.1f}; y = +/-{:.1f}".format(
        *(mm * np.sort(holes[:, 0])[[0, -1]]), mm * holes[:, 1].max()), color=INK, fontsize=8, ha="center", va="top")
    ax.set_xlim(-130, 80)
    ax.set_ylim(-75, 75)
    ax.set_aspect("equal")
    ax.set_xlabel("x in the Go2's base_link, mm (forward ->)", color=MUTED)
    ax.set_ylabel("y, mm (left)", color=MUTED)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.set_title(f"{title}, weld {tuple(np.round(mm * mount, 1).tolist())} mm", fontsize=10, color=INK, loc="left")


def main() -> int:
    rails = read_binary_stl(REPO / "description/meshes/go2/expansion_dock_rails.stl")
    fig, axes = plt.subplots(2, 1, figsize=(9.0, 10.0), dpi=150)
    for ax, args in zip(axes, PANELS):
        panel(ax, *args, rails)
    axes[0].plot([], [], color=DOCK_ORANGE, marker="o", ls="none", label="dock rail screws (M3, 131 x 58 mm)")
    axes[0].plot([], [], color=D1_BLUE, marker="o", mfc="none", ls="none", label="D1 base holes")
    axes[0].fill([], [], color=RAIL_FILL, label="rail tops either side of the T-slot (z = 85.5 mm)")
    axes[0].fill([], [], color=D1_BLUE, alpha=0.10, label="D1 base footprint at the weld")
    axes[0].legend(loc="upper left", fontsize=8, frameon=False, ncol=2, bbox_to_anchor=(0.0, 1.22))
    fig.tight_layout()
    out = HERE / "d1_dock_holes.png"
    fig.savefig(out, facecolor="white")
    print(f"wrote {out.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

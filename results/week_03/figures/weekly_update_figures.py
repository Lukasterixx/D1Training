"""Figures for Weekly Update 3 (2026-10-02): the literature's body-force gains and the D1 base, model vs measured.

    python3 results/week_03/figures/weekly_update_figures.py

Writes, next to this file:
- `force_gain_literature.png`: force with the body helping divided by force without, from the papers read in the
  2026-10-01 literature check (reports/Body force amplification for weak arms.md) and F-077's static model.
- `d1_base_model_vs_measured.png`: side silhouettes of Unitree's `base_link.STL` and `base_link_measured.STL`
  (d1_arm/measured_base.py, F-126), with Joint1 and the shoulder (Joint2, 56.3 mm above Joint1) marked.

CPU only, system Python with numpy and matplotlib. No simulator.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection

HERE = Path(__file__).resolve().parent
MESHES = HERE.parents[2] / "d1_arm" / "meshes"

INK, INK2, MUTED, GRID, BASE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
BLUE, BLUE_LIGHT, ORANGE = "#2a78d6", "#cde2fb", "#eb6834"

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": BASE,
                     "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED})

# (label, without, with, unit, kind): kind "controlled" = same robot and controller, body on vs off;
# "mine" = F-077's static model; "headline" = numbers that are not a like-for-like comparison.
GAINS = [
    ("RoLoMa: ANYmal + Kinova, force gauge", 45.0, 60.0, "N", "controlled"),
    ("Thor: Unitree G1, forward pull", 103.6, 145.5, "N", "controlled"),
    ("Thor: Unitree G1, backward pull", 104.6, 167.7, "N", "controlled"),
    ("Mine: Go2 + D1, best crouch vs lying push (static model)", 1.15, 1.51, "N·m", "mine"),
    ("Portela: B1 + Z1, 90 N pull vs 36 N payload rating", 36.0, 90.0, "N", "headline"),
    ("Brace Yourself: G1 braced vs a different unbraced controller", 13.5, 60.0, "N", "headline"),
]


def force_gain_figure(out: Path) -> None:
    fig, ax = plt.subplots(figsize=(4.3, 3.1), dpi=300)
    ax.axvspan(1.3, 1.6, color="#f0efec", zorder=0)
    ax.text(1.45, len(GAINS) - 0.35, "controlled\ncomparisons", ha="center", va="bottom", fontsize=7.5, color=INK2)
    for i, (label, without, with_, unit, kind) in enumerate(GAINS):
        y = len(GAINS) - 1 - i
        ratio = with_ / without
        if kind == "headline":
            ax.plot(ratio, y, "o", ms=7, mfc="white", mec=MUTED, mew=1.5, zorder=3)
        else:
            ax.plot(ratio, y, "o", ms=7, color=ORANGE if kind == "mine" else BLUE, mec="white", mew=1.5, zorder=3)
        ax.text(1.0, y + 0.22, label, fontsize=7.5, color=INK if kind != "headline" else INK2, va="bottom")
        ax.text(ratio + 0.09, y, f"{ratio:.2f}×", fontsize=7.5, color=INK2, va="center")
    ax.axhline(1.5, color=GRID, lw=0.8)
    ax.text(4.95, 1.62, "not like-for-like", fontsize=7, color=MUTED, ha="right", va="bottom")
    ax.set_xlim(0.95, 5.0)
    ax.set_ylim(-0.6, len(GAINS) + 0.2)
    ax.set_yticks([])
    ax.set_xticks([1, 2, 3, 4, 5])
    ax.set_xticklabels(["1×", "2×", "3×", "4×", "5×"])
    ax.set_xlabel("force with the body helping ÷ force without")
    ax.grid(axis="x", color=GRID, lw=0.6, zorder=0)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.set_title("Published body-on vs body-off force gains", fontsize=9.5, color=INK, loc="left")
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)


def stl_triangles(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    record = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(data, dtype=record, count=count, offset=84)["v"].astype(np.float64) * 1000.0  # mm


def base_figure(out: Path) -> None:
    shoulder_above_joint1 = 56.3
    panels = [("Unitree's model", MESHES / "base_link.STL", 53.3, 105.0, "#d6d5cf", MUTED),
              ("Measured on the real arm", MESHES / "base_link_measured.STL", 76.0, 126.0, BLUE_LIGHT, BLUE)]
    fig, axes = plt.subplots(1, 2, figsize=(4.3, 2.9), dpi=300, sharey=True)
    for ax, (title, mesh, joint1, diameter, fill, edge) in zip(axes, panels):
        tri = stl_triangles(mesh)
        side = tri[:, :, [0, 2]]  # looking along y: x across, z up
        ax.add_collection(PolyCollection(side, facecolors=fill, edgecolors=fill, linewidths=0.4, zorder=2))
        shoulder = joint1 + shoulder_above_joint1
        ax.plot([0, 0], [joint1, shoulder], color=edge, lw=2, zorder=3)
        ax.text(3, joint1 + 18, f"{shoulder_above_joint1:.1f}", fontsize=7, color=INK2, ha="left", va="center")
        ax.plot([-8, 8], [shoulder, shoulder], color=edge, lw=2, zorder=3)
        ax.plot(0, shoulder, "o", ms=6, color=edge, mec="white", mew=1.2, zorder=4)
        ax.text(10, shoulder, f"shoulder\n{shoulder:.1f} mm", fontsize=7, color=INK2, va="center")
        ax.plot([-diameter / 2 - 4, diameter / 2 + 4], [joint1, joint1], color=edge, lw=1.2, zorder=3)
        ax.text(0, joint1 - 4, f"rotating plate\n{joint1:.1f} mm", fontsize=7, color=INK2, ha="center",
                va="top")
        ax.annotate("", xy=(-diameter / 2, -9), xytext=(diameter / 2, -9),
                    arrowprops=dict(arrowstyle="<->", color=INK2, lw=0.8))
        ax.text(0, -12, f"cylinder {diameter:.0f} mm", fontsize=7, color=INK2, ha="center", va="top")
        ax.axhline(0, color=BASE, lw=0.8, zorder=1)
        ax.set_title(title, fontsize=9, color=INK)
        ax.set_xlim(-85, 85)
        ax.set_ylim(-26, 150)
        ax.set_aspect("equal")
        ax.set_xticks([])
        for s in ("top", "right", "bottom"):
            ax.spines[s].set_visible(False)
    model_shoulder = panels[0][2] + shoulder_above_joint1
    real_shoulder = panels[1][2] + shoulder_above_joint1
    axes[1].plot([-66, -4], [model_shoulder, model_shoulder], color=MUTED, lw=0.8, zorder=1)
    axes[1].text(-66, model_shoulder - 2, "model's\nshoulder", fontsize=7, color=MUTED, ha="left", va="top")
    axes[1].annotate("", xy=(-22, real_shoulder), xytext=(-22, model_shoulder),
                     arrowprops=dict(arrowstyle="->", color=INK2, lw=0.8))
    axes[1].text(-26, (model_shoulder + real_shoulder) / 2, f"+{real_shoulder - model_shoulder:.1f} mm", fontsize=7,
                 color=INK, ha="right", va="center")
    axes[0].set_ylabel("height above the rails (mm)", fontsize=7)
    axes[0].set_yticks([0, 50, 100, 150])
    axes[1].tick_params(left=False)
    axes[1].spines["left"].set_visible(False)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)


if __name__ == "__main__":
    force_gain_figure(HERE / "force_gain_literature.png")
    base_figure(HERE / "d1_base_model_vs_measured.png")
    print("wrote", HERE / "force_gain_literature.png", HERE / "d1_base_model_vs_measured.png")

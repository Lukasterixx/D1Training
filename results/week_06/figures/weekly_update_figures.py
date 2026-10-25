#!/usr/bin/env python3
"""The figure for Weekly Update 5 (2026-10-24, Weeks 5-6), drawn for a reader outside the repository.

- `weekly_update_foot_force_vs_position.png`: the front-right foot's force-aware and position-only policies on their
  frozen evaluations, two seeds each, from the findings' own counts (no logs read):
  - the key in the cam lock on the legs they trained on (F-166: 84 + 106 against 20 + 23 of 216);
  - the same key with a realistic leg armature, labelled "motor inertia" (F-168: 169 against 136 of 216);
  - flat plates anywhere the foot reaches (F-180: 197 + 186 against 100 + 58 of 576).

    python3 results/week_06/figures/weekly_update_figures.py
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).parent
# The week's figures' convention: blue position-only (P0), orange force-aware (P1).
POSITION, FORCE = "#2a78d6", "#eb6834"
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"

# (label, force-aware successes, position-only successes, out of)
ROWS = [
    ("Turn the key\n(legs as trained)", 84 + 106, 20 + 23, 216),
    ("Turn the key\n(realistic leg\nmotor inertia)", 169, 136, 216),
    ("Press a plate\n(anywhere the\nfoot reaches)", 197 + 186, 100 + 58, 576),
]


def force_vs_position() -> Path:
    plt.rcParams.update({"font.size": 8, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(5.0, 2.5), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    ax.grid(axis="y", color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    width = 0.36
    for i, (_, force, position, total) in enumerate(ROWS):
        for dx, n, colour in ((-width / 2, position, POSITION), (width / 2, force, FORCE)):
            share = 100 * n / total
            ax.bar(i + dx, share, width, color=colour)
            ax.annotate(f"{n}/{total}", (i + dx, share), xytext=(0, 2), textcoords="offset points",
                        ha="center", va="bottom", color=INK, fontsize=7)
    ax.set_xticks(range(len(ROWS)), [r[0] for r in ROWS])
    ax.set_ylabel("succeeded (%)")
    ax.set_ylim(0, 100)
    ax.bar(0, 0, color=POSITION, label="position-only")
    ax.bar(0, 0, color=FORCE, label="force-aware")
    ax.legend(frameon=False, loc="upper right", fontsize=7, ncol=2)
    ax.set_title("Front-right foot as the end effector (simulation, held-out tests, two seeds)",
                 fontsize=8, color=INK, loc="left")
    fig.tight_layout()
    out = HERE / "weekly_update_foot_force_vs_position.png"
    fig.savefig(out, dpi=220, facecolor=SURFACE)
    plt.close(fig)
    return out


if __name__ == "__main__":
    print(force_vs_position())

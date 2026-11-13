#!/usr/bin/env python3
"""Fig. 2 of the tool paper: matched force-aware / position-only comparisons in simulation.

    python paper/rss2027_tools/figures/make_figures.py

The counts are typed in from the findings' tables, not recomputed from evaluation logs:
(a) F-284 held-out table, round 6: force-aware `8200` against position-only `8600`, keys seated of 96 frozen locks,
    counting only those inside the runner's 15 deg tilt stop ("within"); nominal from the week 9 log 2026-11-13 (69, 81).
(b) F-285: success of 108 frozen keys at the dog's stance 1 N·m, at 2 N·m, and from the settled stance, for the
    arc-following fine-tune at +0 (the parents, trained on the outcome reward), +200 and +400 iterations.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent
BLUE, ORANGE = "#2a78d6", "#eb6834"
INK, MUTED, GRID = "#1f1f1f", "#5f5f5f", "#e3e3e3"

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "Times", "DejaVu Serif"], "font.size": 8,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "legend.frameon": False, "pdf.fonttype": 42,
})

# (condition, force-aware, position-only, group)
INSERTION = [
    ("nominal", 69, 81, "nominal"),
    ("settled stance", 58, 44, "body"),
    ("leg delay 5-20 ms", 70, 71, "body"),
    ("heavy legs, 15-20 ms", 61, 2, "body"),
    ("sticky lock", 59, 76, "lock"),
    ("tight lock", 55, 70, "lock"),
    ("no edge breaks", 48, 59, "lock"),
    ("pose error x1.3-1.5", 61, 70, "lock"),
]
# iterations of the arc-following fine-tune -> (dog stance 1 N·m, 2 N·m, settled)
TURN_FA = {0: (78, 99, 106), 200: (85, 94, 106), 400: (85, 81, 107)}
TURN_PO = {0: (65, 96, 105), 200: (67, 64, 77), 400: (29, 33, 68)}


def main() -> int:
    fig, (a, b) = plt.subplots(2, 1, figsize=(3.45, 4.3), gridspec_kw={"hspace": 0.55, "height_ratios": [1.35, 1]})

    y = np.arange(len(INSERTION))[::-1].astype(float)
    y[1:4] -= 0.35
    y[4:] -= 0.7
    for yi, (name, fa, po, _) in zip(y, INSERTION):
        a.plot([fa, po], [yi, yi], color=GRID, lw=2.5, zorder=1, solid_capstyle="round")
    a.scatter([r[1] for r in INSERTION], y, s=26, color=BLUE, marker="o", edgecolor="white", linewidth=0.6,
              zorder=3, label="force-aware")
    a.scatter([r[2] for r in INSERTION], y, s=26, color=ORANGE, marker="s", edgecolor="white", linewidth=0.6,
              zorder=3, label="position-only")
    a.set_yticks(y)
    a.set_yticklabels([r[0] for r in INSERTION])
    a.set_xlim(-3, 100)
    a.set_xlabel("keys seated, of 96 frozen locks (inside the tilt stop)")
    a.grid(axis="y", visible=False)
    a.text(101, y[2], "body /\nactuation", fontsize=7, color=MUTED, va="center", ha="left")
    a.text(101, (y[5] + y[6]) / 2, "lock /\ntool", fontsize=7, color=MUTED, va="center", ha="left")
    a.legend(loc="lower left", fontsize=7, handletextpad=0.3, borderaxespad=0.1)
    a.set_title("(a) Key insertion: who wins depends on what shifts", fontsize=8, loc="left", color=INK)

    its = np.array(sorted(TURN_FA))
    for data, color, marker, label in ((TURN_FA, BLUE, "o", "force-aware"), (TURN_PO, ORANGE, "s", "position-only")):
        vals = np.array([data[i] for i in its], dtype=float)
        b.fill_between(its, vals.min(1), vals.max(1), color=color, alpha=0.13, lw=0)
        b.plot(its, vals.mean(1), color=color, lw=1.5, marker=marker, ms=4, label=label)
    b.set_xticks(its)
    b.set_xticklabels(["parent\n(outcome reward)", "+200", "+400"])
    b.set_ylim(0, 112)
    b.set_xlabel("iterations of the arc-following fine-tune")
    b.set_ylabel("keys unlocked, of 108")
    b.legend(loc="lower left", fontsize=7)
    b.set_title("(b) Key turn: paid to follow the arc, not to open", fontsize=8, loc="left", color=INK)

    for ext in ("pdf", "png"):
        fig.savefig(OUT / f"fig_matched.{ext}", bbox_inches="tight", dpi=300)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

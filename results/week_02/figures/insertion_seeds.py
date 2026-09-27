"""Three seeds of the insertion recipe on the frozen lateral slots: 400 iterations from v5, then continued at a pinned rate.

Reads the recorded `ins_eval` runs named `*_recipe_s<seed>_<iteration>` (the first 400 iterations) and
`*_recipe_s<seed>c_<iteration>` (the continuation), and v5's untrained baseline under the same task layer. Seed 1's
continuation is v9/v10 (F-121), from v7's line, and is not on this plot. Week 2 log, 2026-09-27; F-122.

    python3 results/week_02/figures/insertion_seeds.py     # -> insertion_seeds.png
"""
from __future__ import annotations

import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "runs"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
NAME = re.compile(r"_ins_eval_lateral_recipe_s(\d)(c)?_(\d+)$")
BASELINE = "20260926T203032_ins_eval_seed1_ins_eval_lateral_v5_grip_now"


def main() -> None:
    seeds: dict[int, dict[int, tuple[int, int]]] = {}
    for path in RUNS.glob("*_ins_eval_seed1_ins_eval_lateral_recipe_s*"):
        m = NAME.search(path.name)
        if m is None:
            continue
        summary = json.loads((path / "ins_eval.json").read_text())
        seeds.setdefault(int(m.group(1)), {})[int(m.group(3))] = (summary["seated"], summary["damaged"])
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(8.4, 4.4))
    for (seed, points), colour in zip(sorted(seeds.items()), SERIES):
        its = sorted(points)
        ax.plot(its, [points[i][0] for i in its], color=colour, lw=2, marker="o", ms=5, label=f"seed {seed} seated")
        ax.plot(its, [points[i][1] for i in its], color=colour, lw=1, ls=":", marker="x", ms=4)
    base = json.loads((RUNS / BASELINE / "ins_eval.json").read_text())["seated"]
    ax.axhline(base, color=MUTED, lw=1, ls="--")
    ax.annotate(f"v5, not trained for it: {base}", xy=(0.01, base + 1.5), xycoords=("axes fraction", "data"),
                color=MUTED, fontsize=7)
    ax.axvline(20397, color=GRID, lw=1)
    ax.annotate("continued at LR 1e-5 →", (20410, 90), color=MUTED, fontsize=7)
    ax.set_ylim(0, 96)
    ax.set_xlabel("iteration (resumed from v5 at 19,998)")
    ax.set_ylabel("of 96 frozen slots (solid: seated, dotted: damaged)")
    ax.grid(axis="y", color=GRID, lw=0.8)
    ax.legend(frameon=False, fontsize=8, loc="upper right")
    ax.set_title("The insertion recipe, three seeds: frozen lateral set, 1 mm clearance", fontsize=10, color=INK, loc="left")
    fig.tight_layout()
    fig.savefig(HERE / "insertion_seeds.png", dpi=160)


if __name__ == "__main__":
    main()

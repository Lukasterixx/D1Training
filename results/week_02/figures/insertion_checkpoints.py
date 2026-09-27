"""The insertion runs' checkpoints on the two frozen slot sets: each run peaks within a few hundred iterations and declines.

Reads every recorded `ins_eval` run of v7, v9, v10 and v11 whose task layer had the wider attitude clamps (roll 40°,
from `run.json`'s `insertion.outer_loops.roll_max_rad`), so every point was scored by the same controller around the
policy: the compliant grip, the outer loops, the home hold. Week 2 log, 2026-09-27; F-121.

    python3 results/week_02/figures/insertion_checkpoints.py     # -> insertion_checkpoints.png
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "runs"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
VERSIONS = ("v7", "v9", "v10", "v11")
NAME = re.compile(r"_ins_eval_(lateral|detent)_(v7|v9|v10|v11)_(\d+)(_roll40)?$")


def points() -> dict[tuple[str, str], list[tuple[int, int, int]]]:
    """(set, version) -> [(checkpoint, seated, damaged)], only runs scored with the wider clamps."""
    out: dict[tuple[str, str], dict[int, tuple[int, int]]] = {}
    for path in RUNS.glob("*_ins_eval_seed1_ins_eval_*"):
        m = NAME.search(path.name)
        if m is None:
            continue
        run = json.loads((path / "run.json").read_text())
        clamps = ((run.get("insertion") or {}).get("outer_loops") or {})
        if clamps.get("roll_max_rad", 0.0) < math.radians(39.0):
            continue
        summary = json.loads((path / "ins_eval.json").read_text())
        out.setdefault((m.group(1), m.group(2)), {})[int(m.group(3))] = (summary["seated"], summary["damaged"])
    return {k: sorted((it, s, d) for it, (s, d) in v.items()) for k, v in out.items()}


def main() -> None:
    data = points()
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 2, figsize=(12.6, 4.4), sharey=True)
    for ax, which, title in ((axes[0], "lateral", "Lateral set: estimate 0-4.5 mm off, detent 12 N"),
                             (axes[1], "detent", "Detent set: 1.5 mm off, detents 0-30 N")):
        for version, colour in zip(VERSIONS, SERIES):
            pts = data.get((which, version))
            if not pts:
                continue
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colour, lw=2, marker="o", ms=5, label=f"{version} seated")
            ax.plot([p[0] for p in pts], [p[2] for p in pts], color=colour, lw=1, ls=":", marker="x", ms=4)
        ax.set_title(title, fontsize=10, color=INK, loc="left")
        ax.set_xlabel("checkpoint (iteration)")
        ax.set_ylim(0, 96)
        ax.grid(axis="y", color=GRID, lw=0.8)
    axes[0].set_ylabel("of 96 (solid: seated, dotted: damaged)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4, frameon=False, fontsize=8)
    fig.suptitle("Insertion checkpoints on the frozen slots (1 mm clearance), compliant grip, outer loops (roll clamp "
                 "40°), home hold", fontsize=10, color=INK, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0.08, 1, 0.94))
    fig.savefig(HERE / "insertion_checkpoints.png", dpi=160)


if __name__ == "__main__":
    main()

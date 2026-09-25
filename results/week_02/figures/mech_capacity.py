"""Which mechanisms each policy opens, by resistance, and how hard it drove them.

Reads recorded `mech_eval` runs (`results/week_02/runs/<id>/mech_eval.json`), so it redraws from the
record without a simulator. Week 2 log, 2026-09-24, "The goal is the input".

    python3 results/week_02/figures/mech_capacity.py <run id>=<label> [<run id>=<label> ...] [--out NAME] [--title TEXT]

`--out` names the image (default `mech_capacity.png`): `mech_capacity_test.png` is the held-out test set. A label
may end in `@<slot>` to fix its colour slot, so a policy keeps its colour from one figure to the next.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "runs"
#: Categorical slots in fixed order (the reference palette), one per policy as given on the command line.
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
ORDER = ("drawer", "latch", "door", "button", "lid", "bolt")


def main(argv: list[str]) -> None:
    out, title, pairs = "mech_capacity.png", None, []
    args = iter(argv)
    for arg in args:
        if arg == "--out":
            out = next(args)
        elif arg == "--title":
            title = next(args)
        else:
            pairs.append(arg)
    runs, slots = [], []
    for pair in pairs:
        run_id, label = pair.split("=", 1)
        slot = len(runs)
        if "@" in label:
            label, slot = label.rsplit("@", 1)
            slot = int(slot)
        runs.append((label, json.loads((RUNS / run_id / "mech_eval.json").read_text())))
        slots.append(slot)
    colours = [SERIES[s] for s in slots]
    # The evaluation's own order; the summary's JSON is written with sorted keys.
    kinds = [k for k in ORDER if k in runs[0][1]["kinds"]]

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(2, len(kinds), figsize=(3.3 * len(kinds), 6.4), sharex=True)
    for column, kind in enumerate(kinds):
        top, bottom = axes[0, column], axes[1, column]
        for (label, summary), colour in zip(runs, colours):
            rows = summary["kinds"][kind]["by_level"]
            levels = [r["level_n"] for r in rows]
            top.plot(levels, [100.0 * r["opened"] / r["of"] for r in rows], color=colour, lw=2, marker="o", ms=4,
                     label=label)
            drive = [(r["level_n"], r["peak_drive_n"]) for r in rows if r["peak_drive_n"] is not None]
            bottom.plot([d[0] for d in drive], [d[1] for d in drive], color=colour, lw=2, marker="o", ms=4)
        top.axhline(80.0, color=MUTED, lw=1, ls=":")
        top.set_ylim(-3, 103)
        top.set_title(f"{kind}", fontsize=10, color=INK, loc="left")
        top.grid(axis="y", color=GRID, lw=0.8)
        top.set_ylabel("placements opened (%)" if column == 0 else "")
        top_levels = [r["level_n"] for r in runs[0][1]["kinds"][kind]["by_level"]]
        bottom.plot([0, max(top_levels)], [0, max(top_levels)], color=MUTED, lw=1, ls=":")
        bottom.set_xlabel("peak resistance (N)")
        bottom.set_ylabel("largest drive before opening,\nmedian (N)" if column == 0 else "")
        bottom.grid(axis="y", color=GRID, lw=0.8)
    axes[0, 0].annotate("capacity threshold: 80% of placements", (11, 70), color=MUTED, fontsize=7)
    axes[1, -1].annotate("dotted: drive = resistance", (32, 4), color=MUTED, fontsize=7)
    fig.suptitle(title or "Given only where the handle should go: which mechanisms open, and how hard the robot drives them",
                 fontsize=10, color=INK, x=0.01, ha="left")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.tight_layout(rect=(0, 0.05, 1, 0.97))
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False, fontsize=8)
    fig.savefig(HERE / out, dpi=160)


if __name__ == "__main__":
    main(sys.argv[1:])

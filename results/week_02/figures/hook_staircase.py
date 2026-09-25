"""The pull staircase, policy by policy: force applied against force commanded, and how the body made it.

Reads recorded `hook_eval` runs (`results/week_02/runs/<id>/hook_eval.json`), so it redraws from the
record without a simulator. Week 2 log, 2026-09-24; F-103.

    python3 results/week_02/figures/hook_staircase.py <run id>=<label> [<run id>=<label> ...]
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
#: F-101: a bent reach's median horizontal pull, and the best robust posture's range.
BENT_MEDIAN_N = 9.4
ROBUST_BEST_N = (43.8, 59.1)


def main(pairs: list[str]) -> None:
    runs = []
    for pair in pairs:
        run_id, label = pair.split("=", 1)
        runs.append((label, json.loads((RUNS / run_id / "hook_eval.json").read_text())))

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(13.0, 4.3))

    ax = axes[0]
    levels = [row["level_n"] for row in runs[0][1]["by_level"]]
    ax.plot([0, max(levels)], [0, max(levels)], color=MUTED, lw=1, ls=":")
    ax.axhspan(*ROBUST_BEST_N, color=GRID, lw=0)
    ax.annotate("best static posture, robust (F-101)", (1, ROBUST_BEST_N[0] + 1.0), color=MUTED, fontsize=8)
    ax.axhline(BENT_MEDIAN_N, color=MUTED, lw=1, ls="--")
    ax.annotate("bent reach, median (F-101)", (31, BENT_MEDIAN_N + 1.0), color=MUTED, fontsize=8)
    for (label, summary), colour in zip(runs, SERIES):
        rows = [r for r in summary["by_level"] if r.get("applied_n") is not None]
        ax.plot([r["level_n"] for r in rows], [r["applied_n"] for r in rows], color=colour, lw=2, marker="o", ms=4,
                label=label)
    ax.set_xlim(0, max(levels) + 2)
    ax.set_ylim(0, max(ROBUST_BEST_N) + 5)
    ax.set_xlabel("commanded pull (N)")
    ax.set_ylabel("applied along the pull, median (N)")
    ax.set_title("Force applied, 60 episodes per policy\n(last 1 s of each 2.5 s hold; dotted: applied = commanded)", fontsize=9, color=INK,
                 loc="left")
    ax.grid(axis="y", color=GRID, lw=0.8)

    ax = axes[1]
    for (label, summary), colour in zip(runs, SERIES):
        rows = [r for r in summary["by_level"]]
        ax.plot([r["level_n"] for r in rows], [100.0 * r["held"] / r["of"] for r in rows], color=colour, lw=2,
                marker="o", ms=4)
    ax.set_ylim(-3, 103)
    ax.set_xlabel("commanded pull (N)")
    ax.set_ylabel("episodes holding the level (%)")
    ax.set_title("Held: ≥80% of the level over the window,\ncontact and robot intact", fontsize=9, color=INK,
                 loc="left")
    ax.grid(axis="y", color=GRID, lw=0.8)

    ax = axes[2]
    for (label, summary), colour in zip(runs, SERIES):
        rows = [r for r in summary["by_level"] if r.get("pitch_deg") is not None]
        ax.plot([r["level_n"] for r in rows], [r["pitch_deg"] for r in rows], color=colour, lw=2, marker="o", ms=4)
    ax.axhline(0.0, color=MUTED, lw=1)
    ax.set_xlabel("commanded pull (N)")
    ax.set_ylabel("base pitch, median (deg; negative = nose up)")
    ax.set_title("How the body is used: base pitch while pulling", fontsize=9, color=INK, loc="left")
    ax.grid(axis="y", color=GRID, lw=0.8)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False, fontsize=8)
    fig.savefig(HERE / "hook_staircase.png", dpi=160)


if __name__ == "__main__":
    main(sys.argv[1:])

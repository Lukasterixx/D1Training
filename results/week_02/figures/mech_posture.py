"""How the body is used to open each mechanism: base pitch at the largest drive, by resistance.

Reads recorded `mech_eval` runs (`results/week_02/runs/<id>/mech_eval.json`). Positive pitch is nose-down (leaning
toward a handle ahead), negative nose-up (leaning back). Week 2 log, 2026-09-25; F-107, F-108.

    python3 results/week_02/figures/mech_posture.py <run id>=<label>[@slot] [...]
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
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
ORDER = ("drawer", "latch", "door", "button")


def main(pairs: list[str]) -> None:
    runs = []
    for index, pair in enumerate(pairs):
        run_id, label = pair.split("=", 1)
        slot = index
        if "@" in label:
            label, slot = label.rsplit("@", 1)
            slot = int(slot)
        runs.append((label, SERIES[slot], json.loads((RUNS / run_id / "mech_eval.json").read_text())))
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, len(ORDER), figsize=(13.2, 3.6), sharey=True)
    for ax, kind in zip(axes, ORDER):
        for label, colour, summary in runs:
            rows = [r for r in summary["kinds"][kind]["by_level"] if r["pitch_at_peak_deg"] is not None]
            ax.plot([r["level_n"] for r in rows], [r["pitch_at_peak_deg"] for r in rows], color=colour, lw=2,
                    marker="o", ms=4, label=label)
        ax.axhline(0.0, color=MUTED, lw=1)
        ax.set_title(kind + (" (push)" if kind == "button" else " (pull)"), fontsize=10, color=INK, loc="left")
        ax.set_xlabel("peak resistance (N)")
        ax.grid(axis="y", color=GRID, lw=0.8)
    axes[0].set_ylabel("base pitch at the largest drive,\nmedian (deg; + leans in, - leans back)")
    fig.suptitle("How the body is used: leaning back into pulls, forward onto the push", fontsize=10, color=INK,
                 x=0.01, ha="left")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.tight_layout(rect=(0, 0.08, 1, 0.95))
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False, fontsize=8)
    fig.savefig(HERE / "mech_posture.png", dpi=160)


if __name__ == "__main__":
    main(sys.argv[1:])

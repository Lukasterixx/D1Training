"""The standing combiner demo against a stiffening door closer: which controller opens it, how far, how hard.

Reads the recorded demo runs (`results/week_02/runs/<id>/summary.json`), so it redraws from the record without a
simulator. Week 2 log, 2026-09-25, "The standing combiner demo under the mechanism policy".

    python3 results/week_02/figures/combiner_mech_sweep.py
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "runs"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
DOORS = (0, 4, 8, 12, 16)
#: (label, run-name suffix, colour slot). Every run: 16 placements, lever 0.4 N·m hooked at 95 mm, turn to 60 degrees.
CONTROLLERS = (
    ("mechanism policy + force law", "mechlaw", 4),
    ("mechanism policy, goal only", "mechgoal", 1),
    ("UniFP + wrist servo, same claw and correction", "oldclaw", 0),
)
GRIP_N = 150.0


def summary(suffix: str, door: int) -> dict:
    matches = sorted(RUNS.glob(f"*_door{door}_{suffix}"))
    return json.loads((matches[-1] / "summary.json").read_text())["summary"]


def main() -> None:
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.9))
    for label, suffix, slot in CONTROLLERS:
        rows = [summary(suffix, door) for door in DOORS]
        colour = SERIES[slot]
        axes[0].plot(DOORS, [r["successes"] for r in rows], color=colour, lw=2, marker="o", ms=4, label=label)
        axes[1].plot(DOORS, [r["max_door_deg_median"] for r in rows], color=colour, lw=2, marker="o", ms=4)
        axes[2].plot(DOORS, [r.get("peak_claw_force_n_median") or 0.0 for r in rows], color=colour, lw=2, marker="o", ms=4)
    axes[0].set_ylim(-0.5, 16.5)
    axes[0].set_yticks((0, 4, 8, 12, 16))
    axes[0].set_ylabel("placements opened (door past 30 deg), of 16")
    axes[0].set_title("Opened", fontsize=10, color=INK, loc="left")
    axes[1].axhline(30.0, color=MUTED, lw=1, ls=":")
    axes[1].annotate("success threshold", (0.3, 31.5), color=MUTED, fontsize=7)
    axes[1].set_ylabel("largest door angle, median (deg)")
    axes[1].set_title("How far the door opened", fontsize=10, color=INK, loc="left")
    axes[2].axhline(GRIP_N, color=MUTED, lw=1, ls=":")
    axes[2].annotate("claw's grip limit", (0.3, GRIP_N - 9), color=MUTED, fontsize=7)
    axes[2].set_ylim(0, GRIP_N + 10)
    axes[2].set_ylabel("peak claw force, median (N)")
    axes[2].set_title("How hard the claw was loaded", fontsize=10, color=INK, loc="left")
    for ax in axes:
        ax.set_xlabel("door closer: torque at 45 degrees (N·m)")
        ax.set_xticks(DOORS)
        ax.grid(axis="y", color=GRID, lw=0.8)
    fig.suptitle("Standing combiner demo, 16 placements per point: a stiffer door, the same claw and script",
                 fontsize=10, color=INK, x=0.01, ha="left")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.tight_layout(rect=(0, 0.08, 1, 0.95))
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False, fontsize=8)
    fig.savefig(HERE / "combiner_mech_sweep.png", dpi=160)


if __name__ == "__main__":
    main()

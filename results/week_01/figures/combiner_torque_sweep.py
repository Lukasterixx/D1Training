"""Figure for the combiner lever torque sweep (Week 1 log, 2026-09-19): how far the arm turns the lever against
springs of increasing torque, and where on the lever it ends up pushing.

    python results/week_01/figures/combiner_torque_sweep.py

Reads the recorded runs' run.json (results/week_01/runs/), so it needs nothing from logs/.
"""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS = Path(__file__).resolve().parents[1] / "runs"
PLACEMENTS = {
    "Box straight ahead, 0.66 m": (["20260919T015634_664610Z_combiner_seed42",
                                    "20260919T020150_725196Z_combiner_seed42"], "#2a78d6", "o"),
    "Box at bearing −43°, 0.67 m": (["20260919T020325_562869Z_combiner_seed42"], "#eb6834", "s"),
}
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
PLANNED_RADIUS_MM, LEVER_LENGTH_MM = 80, 105


def attempts(run_ids):
    rows = []
    for run_id in run_ids:
        rows += json.loads((RUNS / run_id / "run.json").read_text())["episodes"]
    return sorted(rows, key=lambda e: e["handle_torque_nm"])


def main():
    fig, (angle_ax, radius_ax) = plt.subplots(1, 2, figsize=(11, 4.4))
    for label, (run_ids, colour, marker) in PLACEMENTS.items():
        rows = attempts(run_ids)
        torque = [e["handle_torque_nm"] for e in rows]
        held = [e["hold"]["handle_deg"] for e in rows]
        peak = [e["peak_handle_deg"] for e in rows]
        angle_ax.plot(torque, held, color=colour, lw=2, marker=marker, ms=8, label=f"{label}: held")
        angle_ax.plot(torque, peak, color=colour, lw=0, marker=marker, ms=8, mfc="white", mew=1.5,
                      label=f"{label}: peak")
        angle_ax.annotate(label.split(",")[0], (torque[-1], held[-1]), textcoords="offset points",
                          xytext=(-6, -14), ha="right", fontsize=8.5, color=INK)
        radius = [1000 * e["hold"]["arm_torque_on_lever_nm"] / e["hold"]["finger_force_n"] for e in rows]
        radius_ax.plot(torque, radius, color=colour, lw=2, marker=marker, ms=8, label=label)
    angle_ax.axhline(45, color=INK, lw=1, ls="--")
    angle_ax.text(0.31, 45.4, "latch releases at 45°", fontsize=8.5, color=MUTED)
    angle_ax.axhline(52, color=MUTED, lw=1, ls=":")
    angle_ax.text(1.22, 52.4, "commanded 52°", fontsize=8.5, color=MUTED, ha="right")
    angle_ax.set_xlabel("Return-spring torque needed at 45° (N·m)")
    angle_ax.set_ylabel("Lever angle (°)")
    angle_ax.set_title("Lever angle the arm reaches and holds", fontsize=10.5, color=INK)
    angle_ax.set_ylim(30, 55)
    angle_ax.legend(fontsize=7.5, loc="lower left", frameon=False)
    radius_ax.axhline(PLANNED_RADIUS_MM, color=MUTED, lw=1, ls=":")
    radius_ax.text(0.31, PLANNED_RADIUS_MM + 1, "planned contact, 80 mm", fontsize=8.5, color=MUTED)
    radius_ax.axhline(LEVER_LENGTH_MM, color=INK, lw=1, ls="--")
    radius_ax.text(0.31, LEVER_LENGTH_MM - 3.5, "lever end, 105 mm", fontsize=8.5, color=MUTED)
    radius_ax.set_xlabel("Return-spring torque needed at 45° (N·m)")
    radius_ax.set_ylabel("Arm torque on lever ÷ finger force (mm)")
    radius_ax.set_title("Effective push radius during the hold", fontsize=10.5, color=INK)
    radius_ax.set_ylim(55, 115)
    radius_ax.legend(fontsize=7.5, loc="lower right", frameon=False)
    for ax in (angle_ax, radius_ax):
        ax.grid(color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(MUTED)
        ax.tick_params(colors=MUTED, labelsize=8.5)
        ax.xaxis.label.set_color(INK)
        ax.yaxis.label.set_color(INK)
    fig.tight_layout()
    out = Path(__file__).with_suffix(".png")
    fig.savefig(out, dpi=120, facecolor="#fcfcfb")
    print(out)


if __name__ == "__main__":
    main()

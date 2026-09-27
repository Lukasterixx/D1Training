"""How much the reaching policy depends on where the arm is welded: the frozen evaluation at shifted mounts.

Reads the recorded runs (`results/week_02/runs/*_eval_seed1_mount_*`, `eval.json` and `run.json`'s `mount_pos_m`), so it
redraws without a simulator. Week 2 log, 2026-09-26, "How sensitive the policy is to the mount".

    python3 results/week_02/figures/mount_sensitivity.py     # -> mount_sensitivity.png
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "runs"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
LEGACY = (0.0, 0.0, 0.08)
MEASURED = (0.0025, 0.0, 0.0855)


def runs() -> list[tuple[tuple[float, float, float], dict]]:
    out = []
    for path in sorted(RUNS.glob("*_eval_seed1_mount_*")):
        mount = json.loads((path / "run.json").read_text())["mount_pos_m"]
        out.append((tuple(mount), json.loads((path / "eval.json").read_text())))
    return out


def near(a, b) -> bool:
    return all(abs(p - q) < 1e-6 for p, q in zip(a, b))


def main() -> None:
    data = runs()
    base = next(s for m, s in data if near(m, LEGACY))["goal_tracking_quiet_m"]["median"]
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.9), sharey=True,
                             gridspec_kw={"width_ratios": (1.0, 1.0, 0.45)})
    for ax, axis, name in ((axes[0], 0, "fore-aft (x)"), (axes[1], 2, "up-down (z)"), (axes[2], 1, "sideways (y)")):
        # The points on this axis: every other coordinate at the mount it was trained on.
        points = sorted((m[axis] - LEGACY[axis], s) for m, s in data
                        if all(abs(m[i] - LEGACY[i]) < 1e-6 for i in range(3) if i != axis))
        dx = [1000 * p for p, _ in points]
        span = max(abs(v) for v in dx)
        ref = [i * span / 50 for i in range(-50, 51)]
        # What the error would be if none of the shift were corrected and it were square to the policy's own miss.
        ax.plot(ref, [100 * math.hypot(base, r / 1000) for r in ref], color=MUTED, lw=1, ls="--",
                label="the whole shift uncorrected (added to the trained miss)")
        ax.plot(dx, [100 * s["goal_tracking_quiet_m"]["p90"] for _, s in points], color=SERIES[1], lw=2, marker="o",
                ms=5, label="90th percentile")
        ax.plot(dx, [100 * s["goal_tracking_quiet_m"]["median"] for _, s in points], color=SERIES[0], lw=2, marker="o",
                ms=5, label="median")
        ax.axvline(0.0, color=GRID, lw=1, zorder=0)
        ax.set_title(f"Arm moved {name}", fontsize=10, color=INK, loc="left")
        ax.set_xlabel("shift from the trained mount (mm)" if axis != 1 else "shift (mm)")
        ax.grid(axis="y", color=GRID, lw=0.8)
    measured = next(s for m, s in data if near(m, MEASURED))["goal_tracking_quiet_m"]["median"]
    axes[1].plot([1000 * (MEASURED[2] - LEGACY[2])], [100 * measured], color=SERIES[2], marker="*", ms=13, ls="none",
                 label="the measured mount (5.5 mm up and 2.5 mm ahead)")
    axes[1].annotate(f"measured mount: {100 * measured:.2f} cm", (1000 * (MEASURED[2] - LEGACY[2]), 100 * measured),
                     xytext=(8, -14), textcoords="offset points", color=INK, fontsize=8)
    axes[0].set_ylabel("tool-tip error, settled (cm)")
    axes[0].set_ylim(0, None)
    fig.suptitle("UniFP model_56000 (trained at the old mount), the frozen 50-episode evaluation with the arm welded "
                 "elsewhere: the whole shift shows up as error", fontsize=10, color=INK, x=0.01, ha="left")
    handles, labels = [], []
    for ax in axes:
        for h, lab in zip(*ax.get_legend_handles_labels()):
            if lab not in labels:
                handles.append(h)
                labels.append(lab)
    fig.tight_layout(rect=(0, 0.09, 1, 0.94))
    fig.legend(handles, labels, loc="lower center", ncol=4, frameon=False, fontsize=8)
    fig.savefig(HERE / "mount_sensitivity.png", dpi=160)


if __name__ == "__main__":
    main()

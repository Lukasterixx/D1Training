"""Training-time curves of the front-right-leg task, read from the runs' console logs (rsl-rl's per-iteration print).

    python -m unifp_train.leg_curves --run "position (P0)=logs/unifp_train/<stage-1 log>.out,<stage-2 log>.out" \
        --run "force (P1)=..." --out results/week_06/figures/key_training_curves.png

Training-time numbers, sampled at reset over the episodes ending in each iteration -- not a frozen evaluation. One
colour per policy (the reference palette's first two categorical slots, in a fixed order, as `leg_plot`); a 25-iteration
rolling mean. Top: free space (the foot's error to its goal, the worst support foot's distance from its settled place).
Bottom: the key episodes (the share that unlocked, the share past the damage force, the mean peak force).
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np

SERIES = ("#2a78d6", "#eb6834")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
KEYS = ("Leg/free_foot_error_m", "Leg/stance_error_max_cm", "Handle/opened_fraction", "Handle/damaged_fraction",
        "Handle/peak_n")


def read(path: Path) -> dict[str, list[tuple[int, float]]]:
    """Per-iteration values of `KEYS` from one console log: (iteration, value)."""
    out = {k: [] for k in KEYS}
    iteration = None
    pattern = re.compile(r"Learning iteration (\d+)/")
    for line in path.read_text(errors="ignore").splitlines():
        m = pattern.search(line)
        if m:
            iteration = int(m.group(1))
            continue
        if iteration is None:
            continue
        for key in KEYS:
            if key + ":" in line:
                try:
                    out[key].append((iteration, float(line.rsplit(":", 1)[1])))
                except ValueError:
                    pass
    return out


def rolling(points, window=25):
    if not points:
        return np.array([]), np.array([])
    x, y = map(np.asarray, zip(*points))
    if len(y) < window:
        return x, y
    kernel = np.ones(window) / window
    return x[window - 1:], np.convolve(y, kernel, mode="valid")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True, help="label=log[,log...] in legend order")
    parser.add_argument("--stage2_from", type=int, default=600, help="the iteration stage 2 resumed from (a marker)")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(2, 3, figsize=(15, 7), constrained_layout=True)
    panels = [(axes[0, 0], "Leg/free_foot_error_m", 100.0, "free-space foot error at reset (cm)", "The foot's tracking"),
              (axes[0, 1], "Leg/stance_error_max_cm", 1.0, "worst support foot from its place (cm)", "The stance"),
              (axes[1, 0], "Handle/opened_fraction", 100.0, "key episodes unlocked (%)", "Keys unlocked (stage 2)"),
              (axes[1, 1], "Handle/damaged_fraction", 100.0, "key episodes past 120 N (%)", "Past the damage force"),
              (axes[1, 2], "Handle/peak_n", 1.0, "mean peak contact force (N)", "How hard the foot pushes")]
    axes[0, 2].set_axis_off()
    for k, item in enumerate(args.run):
        label, _, paths = item.partition("=")
        # The first log is stage 1 (dashed, its whole length: what happened after the stage-2 checkpoint is evidence
        # too), the rest stage 2 (solid, resumed from `--stage2_from`).
        for n, path in enumerate(paths.split(",")):
            values = read(Path(path))
            for ax, key, scale, ylabel, title in panels:
                x, y = rolling(sorted(values[key]))
                if len(x):
                    ax.plot(x, y * scale, color=SERIES[k], lw=2 if n else 1.5, ls="-" if n else "--",
                            label=f"{label}, {'stage 2' if n else 'stage 1'}")
    for ax, key, scale, ylabel, title in panels:
        ax.axvline(args.stage2_from, ls=":", lw=1.2, color=MUTED, label="stage 2 from here")
        ax.set_xlabel("training iteration")
        ax.set_ylabel(ylabel)
        ax.set_title(title, loc="left", color=INK)
        ax.grid(axis="y", color=GRID, lw=0.8, zorder=0)
        ax.legend(frameon=False, fontsize=8)
    axes[0, 2].text(0.0, 0.9, "Training-time, sampled at reset;\n25-iteration rolling mean.\nNot a frozen evaluation.",
                    color=MUTED, fontsize=9, va="top", transform=axes[0, 2].transAxes)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

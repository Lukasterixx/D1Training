"""The key hold's frozen evaluations over time, one line per arm mode: the key's angle, the hand's miss from the key,
the grip's force and the body's pitch, the median over the frozen keys with the interquartile range shaded.

    python demos/combiner_key/plot_keyhold.py --runs <policy run> <ik run> <fixed run> --labels "learned arm" \
        "IK at 10 Hz" "joints fixed" --out results/week_09/figures/<name>.png

Each run is a `leg_eval` directory (`leg_trace.npz`). An episode that has ended is left out from then on, so the median
is over the keys still going; the count still held is drawn under the miss.
"""
from __future__ import annotations

import argparse
import warnings
from pathlib import Path

import numpy as np

#: The reference palette's first three categorical slots (light), the only three it validates all-pairs.
SERIES = ("#2a78d6", "#eb6834", "#1baf7a")
SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
DT = 0.02
LOST_MM = 25.0


def load(run: Path) -> dict:
    t = np.load(run / "leg_trace.npz")
    return {k: t[k] for k in t.files}


def band(x: np.ndarray):
    """Median and quartiles over episodes (columns), NaN where an episode has ended."""
    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return (np.nanmedian(x, axis=1), np.nanpercentile(x, 25, axis=1), np.nanpercentile(x, 75, axis=1))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--runs", nargs="+", required=True)
    p.add_argument("--labels", nargs="+", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--title", default="The key held while the foot turns it: frozen keys, median and quartiles")
    args = p.parse_args()
    if len(args.runs) != len(args.labels) or len(args.runs) > len(SERIES):
        p.error("one label per run, at most three runs")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "xtick.color": INK_2,
                         "ytick.color": INK_2, "axes.titlecolor": INK, "axes.titlesize": 10, "axes.titleweight": "bold"})
    fig, axes = plt.subplots(4, 1, figsize=(8.0, 9.0), sharex=True, facecolor=SURFACE)
    panels = (("theta", lambda x: np.degrees(x), "key angle, deg"),
              ("hold_err", lambda x: 1000.0 * x, "hand miss, mm"),
              ("grip_n", lambda x: x, "grip force, N"),
              ("pitch", lambda x: np.degrees(x), "pitch, deg (nose down +)"))
    ends = []
    for ax in axes:
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.6)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    for k, (run, label) in enumerate(zip(args.runs, args.labels)):
        trace = load(Path(run))
        n = trace["theta"].shape[0]
        time = np.arange(n) * DT
        for ax, (field, scale, ylabel) in zip(axes, panels):
            values = scale(trace[field].astype(float))
            if field in ("hold_err", "grip_n"):
                values = np.where(np.nan_to_num(trace["holding"]) > 0.5, values, np.nan)
            mid, lo, hi = band(values)
            ax.fill_between(time, lo, hi, color=SERIES[k], alpha=0.14, linewidth=0)
            ax.plot(time, mid, color=SERIES[k], linewidth=2.0, label=label)
            ax.set_ylabel(ylabel)
            last = np.flatnonzero(np.isfinite(mid))
            if len(last) and field == "hold_err":
                ends.append((time[last[-1]], mid[last[-1]], label))
    # Direct labels at each line's last held point in the miss panel, nudged apart where they would collide.
    placed = []
    for t_end, y_end, label in sorted(ends, key=lambda e: e[1]):
        y = y_end
        for other in placed:
            if abs(y - other) < 2.5:
                y = other + 2.5
        placed.append(y)
        axes[1].annotate(label, (t_end, y_end), xytext=(t_end, y), textcoords="data", color=INK_2, fontsize=9,
                         va="center", ha="left")
    axes[1].axhline(LOST_MM, color=INK_2, linewidth=1.0, linestyle=(0, (4, 3)))
    axes[1].annotate("key lost", (1.0, LOST_MM), xycoords=("axes fraction", "data"), xytext=(-2, 3),
                     textcoords="offset points", color=INK_2, fontsize=9, ha="right")
    axes[0].axhline(30.0, color=INK_2, linewidth=1.0, linestyle=(0, (4, 3)))
    axes[0].annotate("unlocks", (1.0, 30.0), xycoords=("axes fraction", "data"), xytext=(-2, 3), textcoords="offset points",
                     color=INK_2, fontsize=9, ha="right")
    axes[-1].set_xlabel("time from the episode's start, s")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, frameon=False, loc="upper right", ncol=len(labels), labelcolor=INK, bbox_to_anchor=(0.99, 0.965))
    right = max(ax.get_xlim()[1] for ax in axes)
    for ax in axes:
        ax.set_xlim(right=right * 1.16)
    fig.suptitle(args.title, color=INK, fontsize=11, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

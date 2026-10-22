"""Figures for the foot-press task's frozen evaluation (`foot_press_eval`).

    python -m unifp_train.foot_press_plot --run "position (P0)=<leg_eval run dir>" --run "force (P1)=<run dir>" \
        --trace_rows 40 41 --out results/week_06/figures/foot_press_eval.png

Four panels, one colour per policy (the reference palette's categorical slots, in a fixed order): presses held by plate
kind; the force held over each hold's last 2 s against the commanded level, rigid and sprung plates apart (median and
interquartile range, the tolerance band shaded); the physics-rate peak contact force per episode (its distribution);
and the normal force against time for chosen rows of the set, every policy on the same plate. Reads
`leg_eval_episodes.json` and `leg_trace.npz`.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .leg_plot import DT, GRID, INK, MUTED, SERIES

KINDS = ("tilted", "wall", "level_up", "level_down")


def load(run_dir: Path) -> tuple[list[dict], dict]:
    episodes = json.loads((run_dir / "leg_eval_episodes.json").read_text())["episodes"]
    return episodes, dict(np.load(run_dir / "leg_trace.npz"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True, help="label=run_dir, in legend order")
    parser.add_argument("--trace_rows", type=int, nargs="+", default=[40, 41])
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = []
    for item in args.run:
        label, _, path = item.partition("=")
        runs.append((label, *load(Path(path))))
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, ((ax_kind, ax_force), (ax_peak, ax_trace)) = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    width = 0.8 / len(runs)
    levels = sorted({e["level_n"] for e in runs[0][1]})
    for k, (label, episodes, trace) in enumerate(runs):
        colour = SERIES[k]
        x = np.arange(len(KINDS)) + (k - (len(runs) - 1) / 2) * width
        held = [sum(e["success"] for e in episodes if e["kind"] == kind) for kind in KINDS]
        total = [sum(e["kind"] == kind for e in episodes) for kind in KINDS]
        ax_kind.bar(x, held, width * 0.92, color=colour, label=f"{label}: {sum(held)} / {len(episodes)}")
        for xi, h in zip(x, held):
            ax_kind.text(xi, h + 1, str(h), ha="center", va="bottom", color=INK, fontsize=8)
        for s, (stiff, marker) in enumerate((("rigid", "o"), ("sprung", "s"))):
            for j, level in enumerate(levels):
                f = [e["force_mean_n"] for e in episodes if e["level_n"] == level and e["stiffness"] == stiff
                     and e["force_mean_n"] is not None]
                if not f:
                    continue
                q1, med, q3 = np.percentile(f, [25, 50, 75])
                xpos = level + (k - (len(runs) - 1) / 2) * 1.2 + (s - 0.5) * 0.5
                ax_force.errorbar(xpos, med, yerr=[[med - q1], [q3 - med]], fmt=marker, color=colour, ms=6,
                                  capsize=0, lw=1.5, mfc=colour if stiff == "rigid" else "white",
                                  label=f"{label}, {stiff}" if j == 0 else None)
        peaks = np.sort([e["peak_n"] for e in episodes])
        ax_peak.plot(peaks, np.arange(1, len(peaks) + 1) / len(peaks), color=colour, lw=2,
                     label=f"{label}: p95 {np.percentile(peaks, 95):.0f} N, max {peaks[-1]:.0f} N")
        for r, row in enumerate(args.trace_rows):
            normal = trace["normal_n"][:, row]
            level = trace["level"][:, row]
            t = np.arange(len(normal)) * DT
            ax_trace.plot(t, normal, color=colour, lw=1.6, ls="-" if r == 0 else "--",
                          label=f"{label}, row {row}" if True else None)
            if k == 0:
                ax_trace.plot(t, level, color=INK, lw=1.0, ls=":" if r else "-.", label=f"commanded, row {row}")
    ax_kind.set_xticks(np.arange(len(KINDS)), [f"{kind}\n(of {n})" for kind, n in zip(KINDS, total)])
    ax_kind.set_ylabel("presses held (force within tolerance 2 s, no slip)")
    ax_kind.set_title("Held, by plate kind", loc="left", color=INK)
    ax_kind.set_ylim(0, max(total) * 1.15)
    ax_kind.legend(frameon=False, loc="upper left", ncol=len(runs))
    grid = np.linspace(0, max(levels) * 1.15, 50)
    ax_force.plot(grid, grid, color=MUTED, lw=1)
    ax_force.fill_between(grid, grid - np.maximum(3.0, 0.2 * grid), grid + np.maximum(3.0, 0.2 * grid), color=GRID,
                          zorder=0, label="tolerance: max(3 N, 20%)")
    ax_force.set_xlabel("commanded normal force (N)")
    ax_force.set_ylabel("held over the last 2 s (N, median and IQR)")
    ax_force.set_title("Force held vs commanded (filled: rigid; open: sprung, 1,000 N/m)", loc="left", color=INK)
    ax_force.legend(frameon=False, fontsize=8, loc="upper left")
    ax_peak.set_xlim(0, 130)
    ax_peak.axvline(60.0, color=MUTED, lw=1, ls="--")
    ax_peak.text(61, 1.0, "60 N", color=MUTED, va="top")
    ax_peak.set_xlabel("peak contact force in the episode (N, physics rate; axis cut at 130)")
    ax_peak.set_ylabel("share of episodes")
    ax_peak.set_title("Peak contact force", loc="left", color=INK)
    ax_peak.legend(frameon=False, loc="lower right")
    rows = ", ".join(f"{r}: {runs[0][1][r]['kind']}, {runs[0][1][r]['stiffness']}, {runs[0][1][r]['level_n']:.0f} N"
                     for r in args.trace_rows)
    ax_trace.set_xlabel("time (s)")
    ax_trace.set_ylabel("normal force on the plate (N)")
    ax_trace.set_title(f"One plate's press ({rows})", loc="left", color=INK)
    ax_trace.legend(frameon=False, fontsize=8, loc="upper left")
    for ax in (ax_kind, ax_force, ax_peak, ax_trace):
        ax.grid(axis="y", color=GRID, lw=0.8)
        ax.set_axisbelow(True)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=130)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

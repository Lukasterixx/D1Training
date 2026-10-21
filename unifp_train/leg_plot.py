"""Figures for the front-right-leg task's frozen evaluation (`leg_eval`).

    python -m unifp_train.leg_plot --run "position (P0)=<leg_eval run dir>" --run "force (P1)=<run dir>" \
        --trace_row 30 --out results/week_06/figures/leg_handle_eval.png

Four panels, one colour per policy (the reference palette's first two categorical slots, in a fixed order):
latch held 1 s by spring strength; the physics-rate peak contact force by spring strength (median, with the
interquartile range) against the force the spring needs at the latch; and one handle's lever angle and contact force
against time, the same handle under every policy. Reads `leg_eval_episodes.json` and `leg_trace.npz`.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
DT = 0.02


def load(run_dir: Path) -> tuple[list[dict], dict]:
    episodes = json.loads((run_dir / "leg_eval_episodes.json").read_text())["episodes"]
    trace = dict(np.load(run_dir / "leg_trace.npz"))
    return episodes, trace


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True, help="label=run_dir, in legend order")
    parser.add_argument("--trace_row", type=int, default=30, help="the handle (row of the frozen set) to trace")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = []
    for item in args.run:
        label, _, path = item.partition("=")
        runs.append((label, *load(Path(path))))
    strengths = sorted({e["full_nm"] for e in runs[0][1]})

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    stance = all("stance_max_cm" in r[2] for r in runs)
    fig, axes = plt.subplots(2, 3 if stance else 2, figsize=(15 if stance else 10, 7), constrained_layout=True)
    if stance:
        (ax_success, ax_peak, ax_stance), (ax_theta, ax_force, ax_spare) = axes
        ax_spare.set_axis_off()
    else:
        (ax_success, ax_peak), (ax_theta, ax_force) = axes
    width = 0.8 / len(runs)
    for k, (label, episodes, _) in enumerate(runs):
        colour = SERIES[k]
        x = np.arange(len(strengths)) + (k - (len(runs) - 1) / 2) * width
        counts = [sum(e["success"] for e in episodes if e["full_nm"] == s) for s in strengths]
        totals = [sum(1 for e in episodes if e["full_nm"] == s) for s in strengths]
        ax_success.bar(x, counts, width * 0.92, color=colour, label=label, zorder=2)
        for xi, c, t in zip(x, counts, totals):
            ax_success.text(xi, c + 0.5, f"{c}", ha="center", va="bottom", color=INK, fontsize=8)
        medians, lows, highs = [], [], []
        for s in strengths:
            peaks = np.array([e["peak_n"] for e in episodes if e["full_nm"] == s and e["engaged"]], float)
            peaks = peaks if len(peaks) else np.array([np.nan])
            medians.append(np.nanmedian(peaks))
            lows.append(np.nanpercentile(peaks, 25))
            highs.append(np.nanpercentile(peaks, 75))
        medians, lows, highs = map(np.array, (medians, lows, highs))
        ax_peak.errorbar(x, medians, yerr=[medians - lows, highs - medians], fmt="o", ms=6, color=colour,
                         elinewidth=2, capsize=0, label=label, zorder=3)

        row = [i for i, e in enumerate(episodes) if e.get("row", i) == args.trace_row]
        trace = runs[k][2]
        if row:
            i = row[0]
            theta = np.degrees(trace["theta"][:, i])
            force = trace["contact_n"][:, i]
            t = np.arange(len(theta)) * DT
            ax_theta.plot(t, theta, color=colour, lw=2, label=label)
            ax_force.plot(t, force, color=colour, lw=2, label=label)
        if stance:
            worst = trace["stance_max_cm"]
            t_all = np.arange(worst.shape[0]) * DT
            median = np.nanmedian(worst, axis=1)
            high = np.nanpercentile(worst, 90, axis=1)
            ax_stance.plot(t_all, median, color=colour, lw=2, label=f"{label}, median")
            ax_stance.plot(t_all, high, color=colour, lw=1.5, ls="--", label=f"{label}, 90th percentile")
    group_n = [sum(1 for e in runs[0][1] if e["full_nm"] == s) for s in strengths]
    total = max(group_n)
    ax_success.set_xticks(np.arange(len(strengths)), [f"{s:g}\n(of {n})" for s, n in zip(strengths, group_n)])
    keys = bool(runs[0][1] and runs[0][1][0].get("key"))
    strength = "lock friction + cam detent (N·m)" if keys else "return spring at the end stop (N·m)"
    ax_success.set_xlabel(strength)
    ax_success.set_ylabel("success: unlocked, no damage, no fall" if keys else "latch held 1 s")
    ax_success.set_title(f"Success by {'lock' if keys else 'spring'} strength", loc="left", color=INK)
    ax_success.set_ylim(0, total * 1.12)
    required = {}
    for e in runs[0][1]:
        required.setdefault(e["full_nm"], e["required_n"])
    ax_peak.plot(np.arange(len(strengths)), [required[s] for s in strengths], ls="--", lw=1.5, color=MUTED,
                 label="needed at the latch")
    ax_peak.set_xticks(np.arange(len(strengths)), [f"{s:g}" for s in strengths])
    ax_peak.set_xlabel(strength)
    ax_peak.set_ylabel("peak contact force (N), median and IQR")
    ax_peak.set_title("How hard the foot pushes", loc="left", color=INK)
    row = runs[0][1][[e.get("row", i) for i, e in enumerate(runs[0][1])].index(args.trace_row)] \
        if any(e.get("row", i) == args.trace_row for i, e in enumerate(runs[0][1])) else None
    if row:
        ax_theta.axhline(row["latch_deg"], ls="--", lw=1.5, color=MUTED, label="unlock" if row.get("key") else "latch")
        ax_theta.axhline(row["theta_max_deg"], ls=":", lw=1.5, color=MUTED, label="end stop")
        what = (f"key {args.trace_row}: {row['friction_nm']:g} N·m lock, {row['detent_nm']:g} N·m cam" if row.get("key")
                else f"handle {args.trace_row}: {row['full_nm']:g} N·m spring, lever {'left' if row['left'] else 'right'}")
    else:
        what = f"handle {args.trace_row}"
    ax_theta.set_xlabel("time (s)")
    ax_theta.set_ylabel("lever angle (deg)")
    ax_theta.set_title(f"One {'key' if keys else 'handle'}, {what}", loc="left", color=INK)
    ax_force.set_xlabel("time (s)")
    ax_force.set_ylabel("contact force on the foot (N)")
    ax_force.set_title(f"The same {'key' if keys else 'handle'}: contact force", loc="left", color=INK)
    if stance:
        ax_stance.set_xlabel("time (s)")
        ax_stance.set_ylabel("worst support foot from its place (cm)")
        ax_stance.set_title("The stance, every episode", loc="left", color=INK)
    for ax in axes.flat:
        if not ax.axison:
            continue
        ax.grid(axis="y", color=GRID, lw=0.8, zorder=0)
        ax.legend(frameon=False, fontsize=8, loc="upper left" if stance and ax is ax_stance else "best")
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

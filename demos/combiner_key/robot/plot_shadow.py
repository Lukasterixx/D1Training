#!/usr/bin/env python3
"""A shadow run's record (`shadow.py`) as a figure: what the dog did while it stood, and what the policy would have asked.

    python demos/combiner_key/robot/plot_shadow.py logs/leg_shadow/<stamp>_leg_shadow \
        --mark 18.2 26.0 "nose pushed down" --mark 43.3 45.8 "FR shoulder" --out results/week_NN/figures/<name>.png

Top to bottom, against time from the first LowState: roll and pitch as the policy sees them (its frame's first two
values: pitch positive nose down, roll positive right side down); the knee torques sport mode reports (`tau_est`);
the foot force sensors (the firmware's raw units); and the knee torque kp 25 / kd 0.5 would ask at the policy's targets
-- open loop, never applied. Each leg keeps its colour in every panel. `--mark` shades what the person at the robot did
(start, end, label); the record does not know it. Numpy and matplotlib only.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

#: The reference palette's first four slots, one per leg (FR, FL, RR, RL), and its ink and grid.
LEGS = (("FR", "#2a78d6"), ("FL", "#eb6834"), ("RR", "#1baf7a"), ("RL", "#eda100"))
INK, MUTED, GRID, SHADE = "#0b0b0b", "#52514e", "#e4e3df", "#f1f0ec"


def spread(values, gap: float):
    """Label heights at least `gap` apart, in the values' order, as close to them as that allows."""
    order = np.argsort(values)
    out = np.array(values, dtype=float)
    for i in range(1, len(order)):
        out[order[i]] = max(out[order[i]], out[order[i - 1]] + gap)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("record", help="a shadow run's directory (logs/leg_shadow/<stamp>_leg_shadow)")
    parser.add_argument("--mark", nargs=3, action="append", default=[], metavar=("START", "END", "LABEL"),
                        help="shade START-END seconds as LABEL (repeatable)")
    parser.add_argument("--out", help="the figure (default: <record>/shadow.png)")
    args = parser.parse_args()
    record = Path(args.record)
    d = np.load(record / "shadow.npz")
    summary = json.loads((record / "summary.json").read_text())
    t = d["t"]

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    period, infer = summary["step_period_s"], summary["infer_ms"]
    goal = summary.get("foot_goal_local")
    fig.suptitle(f"{record.name}: {summary['steps']} steps over {summary['seconds']:.0f} s, nothing sent to the robot\n"
                 f"period median / p99 {1000 * period['median']:.1f} / {1000 * period['p99']:.1f} ms, inference "
                 f"{infer['median']:.1f} ms, arm measured {summary['arm_measured_share']:.0%}, foot goal "
                 f"{'the footfall' if goal is None else goal}", x=0.01, ha="left", color=INK, fontsize=10)

    for ax in axes:
        for start, end, _ in args.mark:
            ax.axvspan(float(start), float(end), color=SHADE, zorder=0, lw=0)
        ax.grid(axis="y", color=GRID, lw=0.8, zorder=0)
    top = axes[0]
    for start, end, label in args.mark:
        top.annotate(label, (0.5 * (float(start) + float(end)), 1.0), xycoords=("data", "axes fraction"),
                     ha="center", va="bottom", color=MUTED, fontsize=8)

    rp = np.degrees(d["frame"][:, 0:2])
    top.plot(t, rp[:, 1], color=INK, lw=1.6, label="pitch (+ nose down)")
    top.plot(t, rp[:, 0], color=MUTED, lw=1.6, ls="--", label="roll (+ right side down)")
    top.axhline(0.0, color=MUTED, lw=0.8)
    top.set_ylabel("degrees")
    top.set_title("Roll and pitch as the policy sees them", loc="left", color=INK, pad=14)
    top.legend(frameon=False, fontsize=8, loc="upper right", ncol=2)

    panels = ((axes[1], d["tau_est"][:, 2::3], "N·m", "Knee torque sport mode reports (tau_est)"),
              (axes[2], d["foot_force"], "sensor units", "Foot force sensors"),
              (axes[3], d["torque"][:, 2::3], "N·m",
               "Knee torque the policy's targets would ask (kp 25, kd 0.5; open loop, never applied)"))
    for ax, values, unit, title in panels:
        for i, (leg, colour) in enumerate(LEGS):
            ax.plot(t, values[:, i], color=colour, lw=1.6, label=leg)
        ends = values[-max(1, len(t) // 100):].mean(axis=0)
        span = np.ptp(values) or 1.0
        for (leg, _), y in zip(LEGS, spread(ends, 0.07 * span)):
            ax.annotate(leg, (t[-1], y), xytext=(6, 0), textcoords="offset points", va="center", color=INK, fontsize=8,
                        annotation_clip=False)
        ax.set_ylabel(unit)
        ax.set_title(title, loc="left", color=INK)
    axes[3].axhline(0.0, color=MUTED, lw=0.8)
    axes[3].set_xlabel("time from the first LowState (s)")
    axes[3].set_xlim(t[0], t[-1])
    handles, labels = axes[1].get_legend_handles_labels()
    axes[1].legend(handles, labels, frameon=False, fontsize=8, loc="upper left", ncol=4)

    out = Path(args.out) if args.out else record / "shadow.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

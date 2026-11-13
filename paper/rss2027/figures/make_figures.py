#!/usr/bin/env python3
"""Figures for the RSS 2027 Stage 1 draft, rebuilt from recorded data only.

    python paper/rss2027/figures/make_figures.py

(a) D1 report skips against where the command landed in the report cycle (F-238): the 23 October rate tests, held pose,
    the 10 Hz and 5 Hz blocks that carry the gripper angle in every command (590 cycles, as in F-238), plus the 23 October
    live holster runs whose feedback samples are still in logs/ (labelled with their own count).
(b) Real force on a scale against the told force (F-269, results/week_08/figures/force_check_tall_aligned.csv) and the
    foot-sensor force estimate per wipe stroke on the bench (F-271, force_wipe_by_stroke.csv).
"""
from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "demos/combiner_door/robot"))
import arm_gap_phase as gap  # noqa: E402

OUT = Path(__file__).resolve().parent
BLUE, ORANGE = "#2a78d6", "#eb6834"
INK, MUTED, GRID = "#1f1f1f", "#5f5f5f", "#e3e3e3"
HELD_BLOCKS = {"10hz", "5hz", "10hz_again"}
G = 9.81

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "Times", "DejaVu Serif"], "font.size": 8,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "legend.frameon": False, "pdf.fonttype": 42,
})


def phase_curve(rows: list[dict], edges: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Share of cycles whose next report skipped, by the latest command's arrival after the report."""
    hit, n = np.zeros(len(edges) - 1), np.zeros(len(edges) - 1)
    for r in rows:
        for u in r["commands_ms"]:
            k = np.searchsorted(edges, u) - 1
            if 0 <= k < len(n):
                n[k] += 1
                hit[k] += r["skipped"]
    with np.errstate(invalid="ignore", divide="ignore"):
        return (edges[:-1] + edges[1:]) / 2, np.where(n >= 5, hit / np.maximum(n, 1), np.nan)


def stall_rows() -> tuple[list[dict], list[dict], int]:
    held = []
    for run in ("20261023T050700", "20261023T051030"):
        for name, reports, commands in gap.rate_test_blocks(ROOT / "logs/arm_command_rate" / run):
            if name in HELD_BLOCKS:
                held += gap.cycle_rows(reports, commands)
    moving, runs = [], 0
    for run in sorted((ROOT / "logs/holster_dog").glob("20261023T*_live")):
        if (run / "feedback_timing/samples.jsonl").exists():
            _, reports, commands = gap.live_run(run)
            moving += gap.cycle_rows(reports, commands)
            runs += 1
    return held, moving, runs


def scale_levels() -> dict[float, list[float]]:
    by = defaultdict(list)
    with open(ROOT / "results/week_08/figures/force_check_tall_aligned.csv") as f:
        for row in csv.DictReader(f):
            told = float(row["told_n"])
            if told in (0.0, 2.0, 4.0, 6.0) and row["scale_g_by_eye"]:
                by[told].append(float(row["scale_g_by_eye"]) * G / 1000)
    return by


def stroke_estimates() -> list[tuple[float, float]]:
    with open(ROOT / "results/week_08/figures/force_wipe_by_stroke.csv") as f:
        return [(float(r["told_n"]), float(r["force_n"])) for r in csv.DictReader(f)]


def main() -> int:
    held, moving, runs = stall_rows()
    hs, ms = gap.summarise(held), gap.summarise(moving)
    print("held", hs)
    print("moving", runs, "runs", ms)

    fig, (a, b) = plt.subplots(2, 1, figsize=(3.45, 3.9), gridspec_kw={"hspace": 0.62})
    edges = np.arange(15, 126, 3.0)
    a.axvspan(98, 106, color="#ecebe6", zorder=0)
    a.axvline(111, color=MUTED, lw=0.8, ls="--")
    x, y = phase_curve(held, edges)
    a.plot(x, y, color=BLUE, lw=1.5, marker="o", ms=3, label=f"held ({hs['cycles']} cycles)")
    x, y = phase_curve(moving, edges)
    a.plot(x, y, color=ORANGE, lw=1.5, marker="s", ms=3,
           label=f"moving, {runs} runs ({ms['cycles']} cycles)")
    a.text(102, 1.07, "stall\nwindow", ha="center", va="bottom", fontsize=7, color=MUTED)
    a.text(112.5, 0.55, "next report\ndue (111 ms)", fontsize=7, color=MUTED, va="center")
    a.set_xlim(15, 125)
    a.set_ylim(-0.03, 1.25)
    a.set_yticks([0, 0.5, 1.0])
    a.set_xlabel("command arrival after the last angle report (ms)")
    a.set_ylabel("share of cycles\nwith a skipped report")
    a.legend(loc="center left", bbox_to_anchor=(0.0, 0.62), fontsize=7, handlelength=1.6)
    a.set_title("(a) Arm stalls follow command phase, not rate", fontsize=8, loc="left", color=INK)

    levels = scale_levels()
    told = np.array(sorted(levels))
    mean = np.array([np.mean(levels[t]) for t in told])
    sd = np.array([np.std(levels[t]) for t in told])
    print("scale means", dict(zip(told, mean.round(2))), "sd", sd.round(2), "n", [len(levels[t]) for t in told])
    k, c = np.polyfit(told, mean, 1)
    strokes = stroke_estimates()
    rng = np.random.default_rng(0)
    st = np.array([s[0] for s in strokes]) + rng.uniform(-0.22, 0.22, len(strokes))
    b.scatter(st, [s[1] for s in strokes], s=14, color=ORANGE, marker="s", edgecolor="white", linewidth=0.5,
              label=f"wipe strokes, foot-sensor estimate (n={len(strokes)})", zorder=3)
    b.errorbar(told, mean, yerr=sd, color=BLUE, marker="o", ms=4, lw=0, elinewidth=1, capsize=2,
               label="held on a scale, measured", zorder=4)
    xx = np.linspace(-0.3, 6.3, 10)
    b.plot(xx, k * xx + c, color=BLUE, lw=1, ls="--", zorder=2)
    b.text(4.25, k * 4.25 + c - 6.5, f"{k:.2f}$\\,\\times\\,$told + {c:.1f} N", color=BLUE, fontsize=7)
    b.set_xlim(-0.6, 6.6)
    b.set_xticks([0, 2, 4, 6])
    b.set_xlabel("force command given to the policy, 'told' (N)")
    b.set_ylabel("normal force (N)")
    b.legend(loc="upper left", fontsize=7, handlelength=1.2)
    b.set_title("(b) The trained force input on the real robot, open loop", fontsize=8, loc="left", color=INK)

    for ext in ("pdf", "png"):
        fig.savefig(OUT / f"fig_interface_force.{ext}", bbox_inches="tight", dpi=300)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

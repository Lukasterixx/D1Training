#!/usr/bin/env python3
"""The figures and table for the force check and the force wipes (F-268 to F-271), from the recorded runs alone.

    python results/week_08/figures/force_wipe_figures.py

Force is estimated from the foot sensors: 1.73 N a raw unit, fitted to the scale in the raised Force check (F-269). A wipe's zero is the
same run's lifted return, at the same goal yaw, so a stroke's estimate is good to perhaps ±9 N (F-270). Heights are calculated, and read low.
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))
from demos.combiner_key.robot import hand_error, leg_policy  # noqa: E402

RUNS = HERE.parent / "runs"
META = leg_policy.LegPolicy(str(ROOT / "checkpoints/reach_force_sticky_model_66600.npz")).meta
N_PER_RAW = 1.73
BENCH_CM = 41.7
EDGES = (-25, -15, -5, 5, 15, 25)
WIPES = (("20261107T041833", 0), ("20261107T042148", 0), ("20261107T042503", 2), ("20261107T043324", 4), ("20261107T043501", 4),
         ("20261107T042714", 6), ("20261107T042857", 6), ("20261107T043104", 6))
#: The two series colours passed the dataviz skill's checker against this surface (2026-11-07): CVD worst 24.7, normal vision 33.6.
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e1"
BLUE, ORANGE = "#2a78d6", "#eb6834"

plt.rcParams.update({"figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE, "font.size": 10, "text.color": INK,
                     "axes.labelcolor": INK2, "axes.edgecolor": GRID, "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True, "grid.color": GRID,
                     "grid.linewidth": 0.7, "axes.axisbelow": True, "axes.spines.top": False, "axes.spines.right": False, "axes.titlesize": 10.5,
                     "axes.titlelocation": "left", "legend.frameon": False, "lines.linewidth": 1.6})


def rows_of(run: str) -> dict:
    z = np.load(RUNS / f"{run}_reach_stand_live" / "policy_rows.npz")
    rows = {k: z[k] for k in z.files}
    t, sphere, off = hand_error.offsets(rows, META)
    skip = len(rows["t"]) - len(t)
    r = {k: v[skip:] for k, v in rows.items()}
    r.update(radius=sphere[:, 0], pitch=np.degrees(sphere[:, 1]), yaw=np.degrees(sphere[:, 2]), sum=r["foot_force"].sum(axis=1))
    r["goal_cm"] = (0.49 + sphere[:, 0] * np.sin(sphere[:, 1])) * 100
    r["tip_cm"] = r["goal_cm"] + off[:, 2] * 100
    return r


def strokes(r: dict) -> dict:
    """A force wipe's two right-to-left strokes and its lifted way back."""
    far = np.abs(r["radius"] - 0.54) < 0.002
    low, up = far & (np.abs(r["pitch"] + 8.5) < 0.06), far & (np.abs(r["pitch"]) < 0.06)
    turning = np.gradient(r["yaw"])
    index = np.arange(len(low))
    air = up & (turning < -0.005)
    going = low & (turning > 0.005)
    return {"one": going & (index < index[air][0]), "air": air, "two": going & (index > index[air][-1])}


def by_yaw(r: dict, key: str, where) -> np.ndarray:
    out = []
    for a, b in zip(EDGES, EDGES[1:]):
        m = where & (r["yaw"] >= a) & (r["yaw"] < b)
        out.append(r[key][m].mean() if m.sum() > 2 else np.nan)
    return np.array(out)


def wipe_table() -> list[dict]:
    table = []
    for run, told in WIPES:
        r = rows_of(run)
        s = strokes(r)
        air = by_yaw(r, "sum", s["air"])
        for name in ("one", "two"):
            force = N_PER_RAW * (air - by_yaw(r, "sum", s[name]))
            table.append({"run": run, "told_n": told, "stroke": name, "force_n": float(np.nanmean(force)), "force_low_n": float(np.nanmin(force)),
                          "force_high_n": float(np.nanmax(force)), "tip_cm": float(np.nanmean(by_yaw(r, "tip_cm", s[name])))})
    return table


def read_csv(name: str) -> dict:
    with open(HERE / name, newline="") as f:
        data = list(csv.DictReader(f))
    return {k: np.array([float(row[k]) for row in data]) for k in data[0]}


def fig_told_and_real() -> None:
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.5), sharey=True)
    for ax, name, title in ((axes[0], "force_check_tall_aligned.csv", "Goal 0.8 cm under the scale's top: real follows told"),
                            (axes[1], "force_check_aligned.csv", "Goal 5.8 cm under it: stalled, told does nothing")):
        d = read_csv(name)
        ax.step(d["s_after_A"], d["told_n"], where="mid", color=ORANGE, label="Told")
        ax.plot(d["s_after_A"], d["scale_g_by_eye"] * 0.00981, "o", ms=4.2, color=BLUE, mec=SURFACE, mew=0.6, label="Real, on the scale")
        ax.set_title(title)
        ax.set_xlabel("Seconds after the last A")
        ax.set_xlim(5, 28)
    axes[0].set_ylabel("Force, N")
    axes[0].set_ylim(0, 30)
    axes[0].legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(HERE / "force_check_told_and_real.png", dpi=200)


def fig_foot_sensors() -> None:
    fit = read_csv("force_check_tall_aligned.csv")
    keep = (fit["s_after_A"] >= 8.5) & (fit["s_after_A"] <= 24.5)
    real = fit["scale_g_by_eye"][keep] * 0.00981
    line = np.polyfit(fit["foot_sum_raw"][keep], real, 1)
    held = read_csv("force_check_aligned.csv")
    other = (held["s_after_A"] >= 8) & (held["s_after_A"] <= 23)
    fig, ax = plt.subplots(figsize=(4.8, 4.2))
    ax.plot([0, 30], [0, 30], "--", color=INK2, lw=1, label="Perfect")
    ax.plot(real, np.polyval(line, fit["foot_sum_raw"][keep]), "o", ms=5, color=BLUE, mec=SURFACE, mew=0.6, label="Fitted run")
    ax.plot(held["scale_g_by_eye"][other] * 0.00981, np.polyval(line, held["foot_sum_raw"][other]), "D", ms=4.8, color=ORANGE, mec=SURFACE, mew=0.6, label="Held-out run")
    ax.annotate("first seconds of the held-out run", xy=(26.3, 19.8), xytext=(14.5, 25.5), color=INK2, fontsize=9, arrowprops={"arrowstyle": "-", "color": INK2, "lw": 0.7})
    ax.set(xlim=(0, 30), ylim=(0, 30), xlabel="Real force on the scale, N", ylabel="Force from the foot sensors, N")
    ax.set_title("Foot sensors against the scale, one spot")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(HERE / "force_check_foot_sensors_vs_scale.png", dpi=200)


def fig_by_level(table: list[dict]) -> None:
    fig, ax = plt.subplots(figsize=(6.2, 4.0))
    told = np.array([0.0, 6.0])
    ax.plot(told, 2.06 * told + 6.4, color=INK2, lw=1.2, label="Force check, on the scale, held still")
    for name, colour, marker, shift, label in (("one", BLUE, "o", -0.09, "Stroke 1"), ("two", ORANGE, "D", 0.09, "Stroke 2")):
        rows = [t for t in table if t["stroke"] == name]
        ax.plot([t["told_n"] + shift for t in rows], [t["force_n"] for t in rows], marker, ms=6, color=colour, mec=SURFACE, mew=0.7, label=label)
    ax.axhline(0, color=INK2, lw=0.7)
    ax.set(xticks=[0, 2, 4, 6], xlabel="Told force, N", ylabel="Force from the foot sensors, N, mean over a stroke", ylim=(-5, 30), xlim=(-0.6, 6.6))
    ax.set_title("Force wipes: force rises with the level, unevenly")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(HERE / "force_wipe_force_by_level.png", dpi=200)


def fig_two_ways() -> None:
    fig, axes = plt.subplots(1, 4, figsize=(10.4, 3.5), sharex=True, sharey=True)
    centres = (np.array(EDGES[:-1]) + np.array(EDGES[1:])) / 2
    for ax, title, runs, pitch in zip(axes, ("High", "Near", "Touch", "Firm, two runs"), (("20261107T020316",), ("20261107T020619",), ("20261107T021229",), ("20261107T021520", "20261107T021951")),
                                      (-3.2, -7.5, -9.6, -11.8)):
        lines = {"Right to left": [], "Left to right": []}
        for run in runs:
            r = rows_of(run)
            low = np.abs(r["pitch"] - pitch) < 0.2
            turning = np.gradient(r["yaw"])
            lines["Right to left"].append(by_yaw(r, "tip_cm", low & (turning > 0.005)))
            lines["Left to right"].append(by_yaw(r, "tip_cm", low & (turning < -0.005)))
        ax.axhline(BENCH_CM, color=INK2, lw=0.9, ls="--")
        ax.plot(centres, np.nanmean(lines["Right to left"], axis=0), "-o", ms=4.5, color=BLUE, mec=SURFACE, mew=0.6, label="Right to left")
        ax.plot(centres, np.nanmean(lines["Left to right"], axis=0), "-D", ms=4.2, color=ORANGE, mec=SURFACE, mew=0.6, label="Left to right")
        ax.set_title(title)
    axes[0].set_ylabel("Calculated hand height, cm")
    axes[0].text(20, BENCH_CM - 0.75, "bench top", color=INK2, fontsize=8.5, ha="right")
    fig.supxlabel("Goal yaw, deg (the dog's right is negative)", fontsize=10, color=INK2)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", ncol=2, bbox_to_anchor=(0.055, 1.0))
    fig.savefig(HERE / "wipe_hand_height_two_ways.png", dpi=200)


def main() -> None:
    table = wipe_table()
    with open(HERE / "force_wipe_by_stroke.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0]))
        w.writeheader()
        w.writerows({k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()} for row in table)
    fig_told_and_real()
    fig_foot_sensors()
    fig_by_level(table)
    fig_two_ways()
    told = np.array([t["told_n"] for t in table], dtype=float)
    force = np.array([t["force_n"] for t in table])
    print("force = %.2f x told + %.1f, r %.2f, residual sd %.1f N, n %d" % (*np.polyfit(told, force, 1), np.corrcoef(told, force)[0, 1], (force - np.polyval(np.polyfit(told, force, 1), told)).std(), len(force)))
    for level in (0, 2, 4, 6):
        for name in ("one", "two"):
            v = [t for t in table if t["told_n"] == level and t["stroke"] == name]
            print(level, name, "n", len(v), "force", round(float(np.mean([t["force_n"] for t in v])), 1), "tip", round(float(np.mean([t["tip_cm"] for t in v])), 1))


if __name__ == "__main__":
    main()

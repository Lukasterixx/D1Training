#!/usr/bin/env python3
"""The key turned by the arm (`run_arm_turn.py`) as figures.

    python demos/combiner_key/plot_arm_turn.py logs/combiner_key_arm/<run> --out results/week_NN/figures/<name>.png
    python demos/combiner_key/plot_arm_turn.py --sweep logs/combiner_key_arm/<run> ... --out <name>.png

One run: four of its overview frames (on the platform, mid-turn, at the stop, lifted off), then against time through
the push -- the key's angle (the lock's plant), the robot's own measure of it (its fingertips' edge on the estimated
key) and the command; the contact force on the fingers, against what the lock takes; and each arm joint's torque as a
share of its limit. Attempt 0 of batch 0. `--sweep`: every attempt's furthest key angle against the lock's resistance
(as a push where the fingers push), with the quasi-static ceiling from `arm_turn.capacity_scan`.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

#: The reference palette's first slots: the six arm joints; the key's own series in ink.
JOINTS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300")
INK, MUTED, GRID, SHADE = "#0b0b0b", "#52514e", "#e4e3df", "#f1f0ec"
PUSH = ("arm_out", "arm_above", "arm_touch", "turn", "arm_hold", "arm_lift", "arm_back")
#: Which frame to show for each picture: the last one of the phase, or the one nearest the key angle.
PICTURES = (("on the platform", "arm_touch", None), ("mid-turn", "turn", 0.0), ("at the stop", "arm_hold", None),
            ("lifted off", "arm_lift", None))


def _style():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    return plt


def _num(v):
    return float(v) if v not in ("", None) else np.nan


def plot_run(run: Path, out: Path) -> None:
    plt = _style()
    rows = [r for r in csv.DictReader((run / "trace.csv").open()) if r["batch"] == "0" and r["env"] == "0"
            and r["phase"] in PUSH]
    report = json.loads((run / "run.json").read_text())
    attempt = json.loads((run / "summary.json").read_text())["attempts"][0]
    t = np.array([float(r["t_s"]) for r in rows])
    t0 = t[0]
    t = t - t0
    frames = sorted((run / "frames").glob("*.png"))

    fig = plt.figure(figsize=(13, 11.5), constrained_layout=True)
    grid = fig.add_gridspec(4, 4, height_ratios=(1.15, 1, 0.8, 0.8))
    lock = report["lock"]
    fig.suptitle(f"{run.name}: the D1 pushes the key from {report['push']['turn_deg'][0]:+.0f} to "
                 f"{report['push']['turn_deg'][1]:+.0f} deg against a {lock['friction_nm']:.2f} N·m lock "
                 f"({lock['as_push_n']:.1f} N where the fingers push); furthest {attempt.get('max_key_deg')} deg, "
                 f"unlocked {attempt.get('unlocked')}, the robot's own measure at the hold "
                 f"{(attempt.get('held') or {}).get('measured_deg')} deg", x=0.01, ha="left", color=INK, fontsize=10)
    from PIL import Image

    for k, (title, phase, near_deg) in enumerate(PICTURES):
        ax = fig.add_subplot(grid[0, k])
        ax.set_axis_off()
        candidates = [f for f in frames if f"_{phase}.png" in f.name]
        if near_deg is not None and candidates:
            def angle(f):
                stamp = float(f.name.split("_")[1].rstrip("s"))
                i = int(np.argmin(np.abs(t + t0 - stamp)))
                return abs(_num(rows[i]["key_deg"]) - near_deg) if rows[i]["key_deg"] != "" else 1e9
            pick = min(candidates, key=angle)
        else:
            pick = candidates[-1] if candidates else None
        if pick is not None:
            ax.imshow(Image.open(pick))
        ax.set_title(title, loc="left", color=INK)

    spans = []
    for r, tt in zip(rows, t):
        if not spans or spans[-1][0] != r["phase"]:
            spans.append([r["phase"], tt, tt])
        spans[-1][2] = tt

    def shade(ax, label=False):
        for k, (name, a, b) in enumerate(spans):
            if k % 2 == 0:
                ax.axvspan(a, b, color=SHADE, lw=0, zorder=0)
            if label:
                ax.annotate(name.replace("arm_", ""), (0.5 * (a + b), 1.0), xycoords=("data", "axes fraction"),
                            ha="center", va="bottom", color=MUTED, fontsize=8)
        ax.grid(axis="y", color=GRID, lw=0.8, zorder=0)

    ax = fig.add_subplot(grid[1, :])
    shade(ax, label=True)
    ax.plot(t, [_num(r["cmd_deg"]) for r in rows], color=MUTED, lw=1.4, ls="--", label="commanded")
    ax.plot(t, [_num(r["key_deg"]) for r in rows], color=INK, lw=2.0, label="the key (lock plant)")
    ax.plot(t, [_num(r["measured_deg"]) for r in rows], color=JOINTS[0], lw=1.4, ls=":",
            label="the robot's measure (fingertip edge on the estimated key)")
    ax.axhline(30.0, color=MUTED, lw=0.8, ls=":")
    ax.annotate("unlocks", (t[-1], 30.0), xytext=(4, 0), textcoords="offset points", va="center", color=MUTED,
                fontsize=8, annotation_clip=False)
    ax.set_ylim(-55, 55)
    ax.set_ylabel("key angle (deg, + down)")
    ax.set_title("The key's angle", loc="left", color=INK, pad=14)
    ax.legend(frameon=False, fontsize=8, loc="upper left")

    ax = fig.add_subplot(grid[2, :])
    shade(ax)
    ax.plot(t, [_num(r["contact_n"]) for r in rows], color=INK, lw=1.6, label="contact force on the fingers")
    ax.plot(t, [_num(r["drive_n"]) for r in rows], color=JOINTS[0], lw=1.2, label="its part along the key's arc")
    need = lock["friction_nm"] / report["push"]["reach_m"]
    ax.axhline(need, color=MUTED, lw=0.8, ls=":")
    ax.annotate(f"the lock: {need:.1f} N", (t[-1], need), xytext=(4, 0), textcoords="offset points", va="center",
                color=MUTED, fontsize=8, annotation_clip=False)
    ax.set_ylabel("N")
    ax.set_title("The push", loc="left", color=INK)
    ax.legend(frameon=False, fontsize=8, loc="upper left")

    ax = fig.add_subplot(grid[3, :])
    shade(ax)
    for j, colour in enumerate(JOINTS):
        ax.plot(t, [_num(r[f"use_j{j + 1}"]) for r in rows], color=colour, lw=1.3, label=f"J{j + 1}")
    ax.axhline(1.0, color=MUTED, lw=0.8)
    ax.set_ylim(0, 1.1)
    ax.set_ylabel("share of the limit")
    ax.set_xlabel(f"time from the push's start (s; {t0:.1f} s into the attempt)")
    ax.set_title("Each arm joint's torque against its limit (PhysX's joint force)", loc="left", color=INK)
    ax.legend(frameon=False, fontsize=8, loc="upper left", ncol=6)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


def plot_sweep(runs: list[Path], out: Path) -> None:
    plt = _style()
    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    reach = None
    for run in runs:
        summary = json.loads((run / "summary.json").read_text())
        report = json.loads((run / "run.json").read_text())
        reach = report["push"]["reach_m"]
        n = summary["summary"]["lock_as_push_n"]
        for k, a in enumerate(summary["attempts"]):
            y = a.get("max_key_deg")
            if y is None:
                continue
            ax.scatter(n + (k - len(summary["attempts"]) / 2) * 0.12, y, s=36, color=JOINTS[0] if a.get("unlocked")
                       else JOINTS[1], edgecolor="white", lw=1.0, zorder=3)
    scan_path = Path("logs/combiner_key_arm/capacity_scan.json")
    if scan_path.exists() and reach:
        scan = json.loads(scan_path.read_text())["scan"]
        lows = [e["min_nm"] / reach for e in scan if "min_nm" in e]
        ax.axvspan(min(lows), max(lows), color=SHADE, lw=0, zorder=0)
        ax.annotate("quasi-static ceiling over the stance's spread", (0.5 * (min(lows) + max(lows)), -50),
                    ha="center", color=MUTED, fontsize=8)
    ax.axhline(30.0, color=MUTED, lw=0.8, ls=":")
    ax.axhline(45.0, color=MUTED, lw=0.8)
    ax.annotate("unlocks", (ax.get_xlim()[1], 30.0), xytext=(-4, 3), textcoords="offset points", ha="right",
                color=MUTED, fontsize=8)
    ax.annotate("stop", (ax.get_xlim()[1], 45.0), xytext=(-4, 3), textcoords="offset points", ha="right",
                color=MUTED, fontsize=8)
    ax.set_ylim(-55, 55)
    ax.set_xlabel(f"the lock's resistance, N where the fingers push ({reach} m from the shaft)")
    ax.set_ylabel("furthest key angle (deg)")
    ax.set_title("How stiff a lock the arm turns (each dot an attempt; blue unlocked, orange not)", loc="left",
                 color=INK)
    ax.grid(axis="y", color=GRID, lw=0.8, zorder=0)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run", nargs="?", help="a run's directory (logs/combiner_key_arm/<run>)")
    parser.add_argument("--sweep", nargs="+", help="runs at different locks, for the sweep figure")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    if args.sweep:
        plot_sweep([Path(p) for p in args.sweep], Path(args.out))
    if args.run:
        plot_run(Path(args.run), Path(args.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

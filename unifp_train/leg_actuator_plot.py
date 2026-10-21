"""The key policies on each leg motor model (`leg_actuators`), and what the standing legs' knees ask of the motor.

    python -m unifp_train.leg_actuator_plot --runs logs/unifp_train --out results/week_06/figures/key_leg_actuators.png

Reads the frozen key evaluations named `key_act_<policy>_<model>` and `key_classic_<policy>_<height>cm` (2026-10-21).
Left: keys unlocked (of 108, success as the frozen evaluation scores it) per leg model and policy; force-aware and
position-only in the reference palette's first two slots, the seed by the marker. Top right: the same from the sport
mode's classic stand (feet 41 x 28 cm, at rest) by base height, against the trained stance (0.272 m). Bottom right:
under the trained model, the share of time the largest support-leg knee torque exceeds each level while the
front-right foot is off the ground (stages 1-6), against the knee caps. Torques are the last physics substep of each
policy step (50 Hz).
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np

SERIES = ("#2a78d6", "#eb6834")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
#: (run suffix, label), top to bottom.
MODELS = (("unifp", "trained: UniFP ideal PD, armature 2e-4"), ("knee23", "knee capped at 23.5 N·m"),
          ("speedcurve", "DCMotor speed curve only"), ("knee23curve", "knee cap + speed curve"),
          ("dcmotor", "Isaac Lab stock DCMotor (armature 0)"), ("dcmotor_armature", "stock DCMotor, armature 2e-4"),
          ("unitree", "Unitree envelope, knee as hip"), ("unitree_geared", "Unitree envelope, geared knee"),
          ("unifp_d2", "trained + 0-10 ms delay"), ("unitree_geared_d2", "geared + 0-10 ms delay"),
          ("unifp_arm003", "trained, armature 0.003"), ("unifp_arm010", "trained, armature 0.01"),
          ("unifp_arm010k037", "trained, armature 0.01, knees 0.037"), ("unitree_geared_arm003", "geared, armature 0.003"),
          ("unitree_geared_arm010", "geared, armature 0.01"), ("unitree_geared_arm010k037", "geared, armature 0.01, knees 0.037"))
#: Classic-stance starts (`demos/combiner_key/classic_stance.py`): base heights, cm.
CLASSIC_CM = (28, 30, 32, 34)
#: (run prefix, label, colour index, marker).
POLICIES = (("force_s1", "force-aware, seed 1", 0, "o"), ("force_s2", "force-aware, seed 2", 0, "s"),
            ("force_s2walker", "force-aware, seed 2 walker fine-tune", 0, "D"),
            ("position_s1", "position-only, seed 1", 1, "o"), ("position_s2", "position-only, seed 2", 1, "s"))
CAPS = ((20.2, "Unitree driving 20.2"), (23.5, "stock cap 23.5"), (38.7, "geared driving 38.7"))


def run_dir(root: Path, name: str) -> Path | None:
    hits = sorted(glob.glob(str(root / f"*_leg_eval_seed1_key_act_{name}")))
    return Path(hits[-1]) if hits and (Path(hits[-1]) / "leg_eval.json").exists() else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", default="logs/unifp_train")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    root = Path(args.runs)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(15, 8.5), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, width_ratios=(1.3, 1))
    left, top, right = fig.add_subplot(grid[:, 0]), fig.add_subplot(grid[0, 1]), fig.add_subplot(grid[1, 1])
    table = {}
    for y, (model, label) in enumerate(MODELS):
        left.axhline(y, color=GRID, lw=0.8, zorder=0)
        for prefix, plabel, colour, marker in POLICIES:
            d = run_dir(root, f"{prefix}_{model}")
            if d is None:
                continue
            success = json.loads((d / "leg_eval.json").read_text())["success"]
            table[(prefix, model)] = success
            offset = {"force_s1": -0.18, "force_s2": -0.06, "force_s2walker": 0.06, "position_s1": 0.12,
                      "position_s2": 0.24}[prefix]
            left.scatter(success, y + offset, s=42, color=SERIES[colour], marker=marker, edgecolor="white", lw=1.2,
                         zorder=3, label=plabel if y == 0 else None)
    left.set_yticks(range(len(MODELS)), [m[1] for m in MODELS])
    left.invert_yaxis()
    left.set_xlim(-3, 111)
    left.set_xlabel("keys unlocked, foot back down, no damage, no fall (of 108)")
    left.set_title("The same policies on each leg motor model", loc="left", color=INK)
    handles, labels = left.get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=len(labels), frameon=False, fontsize=9)

    for prefix, plabel, colour, marker in POLICIES:
        trained = run_dir(root, f"{prefix}_unifp")
        xs, ys = [], []
        for cm in CLASSIC_CM:
            hits = sorted(glob.glob(str(root / f"*_leg_eval_seed1_key_classic_{prefix}_{cm}cm")))
            if hits and (Path(hits[-1]) / "leg_eval.json").exists():
                xs.append(cm / 100.0)
                ys.append(json.loads((Path(hits[-1]) / "leg_eval.json").read_text())["success"])
        if xs:
            top.plot(xs, ys, color=SERIES[colour], lw=2, marker=marker, ms=6, mec="white", mew=1.2, label=plabel)
        if trained is not None:
            top.scatter(0.272, json.loads((trained / "leg_eval.json").read_text())["success"], s=42, color=SERIES[colour],
                        marker=marker, facecolor="white", lw=1.5, zorder=3)
    top.text(0.2745, 45, "hollow: from the\ntrained stance", color=MUTED, fontsize=8)
    top.set_xlim(0.265, 0.345)
    top.set_ylim(-3, 111)
    top.set_xlabel("base height of the classic stand (m); feet 41 x 28 cm, at rest")
    top.set_ylabel("keys unlocked (of 108)")
    top.set_title("Starting from the sport mode's classic stand (trained model)", loc="left", color=INK)
    top.grid(axis="y", color=GRID, lw=0.8, zorder=0)

    levels = np.linspace(0, 40, 161)
    for prefix, plabel, colour, marker in POLICIES:
        if prefix not in ("force_s1", "position_s1"):
            continue
        d = run_dir(root, f"{prefix}_unifp")
        if d is None:
            continue
        t = np.load(d / "leg_trace.npz")
        lifted = (t["stage"] >= 1) & (t["stage"] <= 6)
        knee = t["tau_sup_calf"][lifted]
        share = [(knee > v).mean() * 100.0 for v in levels]
        right.plot(levels, share, color=SERIES[colour], lw=2, label=plabel)
    for value, label in CAPS:
        right.axvline(value, color=MUTED, ls=":", lw=1.2)
        right.text(value + 0.3, 92, label, color=MUTED, fontsize=8, rotation=90, va="top")
    right.set_yscale("log")
    right.set_ylim(0.01, 100)
    right.set_xlabel("largest support-leg knee torque (N·m)")
    right.set_ylabel("time above it, FR foot lifted (%)")
    right.set_title("What the standing knees ask for (trained model)", loc="left", color=INK)
    right.grid(axis="y", color=GRID, lw=0.8, zorder=0)
    right.legend(frameon=False, fontsize=8, loc="lower left")
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(f"wrote {args.out}")
    for (prefix, model), success in sorted(table.items()):
        print(f"  {prefix:16s} {model:18s} {success}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

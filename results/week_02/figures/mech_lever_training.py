"""The mechanism policy before and after training on levers: the frozen lever test, and the combiner demo.

Reads the recorded runs (`results/week_02/runs/<id>/`), so it redraws from the record without a simulator. Week 2 log,
2026-09-25, "Training the mechanism policy on levers".

    python3 results/week_02/figures/mech_lever_training.py     # -> mech_lever_eval.png, combiner_lever_policy.png
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "runs"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
#: (label, run-name suffix, colour slot) for the two policies.
POLICIES = (("v0: model_17499", "mech_lever_17499", 1), ("v1: model_18998 (levers)", "mech_lever_18998", 3),
            ("v2: model_20497 (high levers, off-path price)", "mech_lever_20497", 0))
KINDS = (("lever", "Square to the lever"), ("lever_spindle_side", "30° to the spindle's side (late turn a pull)"),
         ("lever_tip_side", "30° to the tip's side (late turn a push)"))


def style() -> None:
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})


def latest(pattern: str, exclude: tuple[str, ...] = ()) -> Path | None:
    matches = sorted(path for path in RUNS.glob(pattern) if not any(word in path.name for word in exclude))
    return matches[-1] if matches else None


def eval_figure() -> None:
    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.7), sharey=True)
    for label, suffix, slot in POLICIES:
        summary = json.loads((latest(f"*_mech_eval_seed1_{suffix}") / "mech_eval.json").read_text())
        kinds = summary.get("summary", summary)["kinds"]
        for ax, (kind, title) in zip(axes, KINDS):
            rows = kinds[kind]["by_level"]
            ax.plot([r["level_n"] for r in rows], [r["opened"] for r in rows], color=SERIES[slot], lw=2, marker="o",
                    ms=4, label=label)
            ax.set_title(title, fontsize=10, color=INK, loc="left")
    for ax in axes:
        ax.set_xlabel("lever's peak force at the handle (N)")
        ax.set_ylim(-0.5, 15.5)
        ax.set_yticks((0, 5, 10, 15))
        ax.grid(axis="y", color=GRID, lw=0.8)
    axes[0].set_ylabel("placements opened, of 15")
    fig.suptitle("Frozen lever test (the combiner's handle: 8 cm, 60° down, 15 placements per point), law with 1 s bleed",
                 fontsize=10, color=INK, x=0.01, ha="left")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.tight_layout(rect=(0, 0.08, 1, 0.95))
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False, fontsize=8)
    fig.savefig(HERE / "mech_lever_eval.png", dpi=160)


#: The demo, lever stiffness × stance, both claws, for the three policies: v0 the stance sweep's runs, v1 (`lvr_`) and
#: v2 (`v2_`, with the goal corrected through the entry, which fixes v1's and v2's grasp and leaves v0 as it was).
DEMO_STANCES = ((-30, "m30", "−30° (hinge side)"), (0, "p0", "square"), (15, "p15", "+15°"), (30, "p30", "+30° (latch side)"))
DEMO_TORQUES = ("0.4", "0.8", "1.2", "1.6", "2.4")
CLAWS = (("lips", "L-lip claw"), ("spring", "spring claw"))
#: v0's runs share the pattern with every later one: the others' prefixes, the corrected-entry checks and the raised box.
V0_EXCLUDE = ("_lvr_", "_v2", "_ce_", "high_", "both_")
DEMO_POLICIES = (("v0: model_17499", "", V0_EXCLUDE, 1), ("v1: model_18998 (levers), no entry correction", "lvr_", (), 3),
                 ("v2: model_20497 (high levers, off-path price), entry corrected", "v2_", (), 0))


def demo_figure() -> None:
    fig, axes = plt.subplots(2, 4, figsize=(13.4, 6.2), sharex=True, sharey=True)
    for row, (claw, name) in enumerate(CLAWS):
        for col, (stance, tag, title) in enumerate(DEMO_STANCES):
            ax = axes[row, col]
            for label, prefix, exclude, slot in DEMO_POLICIES:
                points = []
                for torque in DEMO_TORQUES:
                    path = latest(f"*_{prefix}st{tag}_lev{torque}_{claw}", exclude=exclude)
                    if path is not None:
                        points.append((float(torque), json.loads((path / "summary.json").read_text())["summary"]["successes"]))
                if points:
                    ax.plot(*zip(*points), color=SERIES[slot], lw=2, marker="o", ms=4, label=label)
            ax.set_title(f"{name}, {title}", fontsize=9, color=INK, loc="left")
            ax.set_ylim(-0.5, 16.5)
            ax.set_yticks((0, 4, 8, 12, 16))
            ax.set_xticks([float(t) for t in DEMO_TORQUES])
            ax.grid(axis="y", color=GRID, lw=0.8)
            if row == 1:
                ax.set_xlabel("lever spring at 45° (N·m)")
        axes[row, 0].set_ylabel("opened, of 16")
    fig.suptitle("Standing combiner demo, free door, 16 placements per point: the mechanism policy before and after "
                 "lever training", fontsize=10, color=INK, x=0.01, ha="left")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.tight_layout(rect=(0, 0.06, 1, 0.95))
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False, fontsize=8)
    fig.savefig(HERE / "combiner_lever_policy.png", dpi=160)


def main() -> None:
    style()
    eval_figure()
    if latest("*_lvr_stp0_lev0.4_spring") is not None:
        demo_figure()


if __name__ == "__main__":
    main()

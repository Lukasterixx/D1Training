"""Where the robot stands against the combiner box: the lever turn from the door's normal and from either side.

Reads the recorded demo runs (`results/week_02/runs/<id>/summary.json`), so it redraws from the record
without a simulator. Week 2 log, 2026-09-25, "Standing to the side of the box".

    python3 results/week_02/figures/combiner_stance_sweep.py     # -> combiner_stance_sweep.png, combiner_stance_door.png
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
TORQUES = ("0.4", "0.8", "1.2", "1.6")
#: (stance in degrees, run-name tag, colour slot). Positive: the latch side, where the lever's turn draws the
#: handle toward the robot; negative: the hinge side.
STANCES = ((-30, "m30", 0), (0, "p0", 1), (15, "p15", 4), (30, "p30", 2), (45, "p45", 3))
CLAWS = (("spring", "spring claw (holds in every direction)"), ("lips", "L-lip claw (PhysX contact)"))
#: The lever spring (demos/combiner/geometry.py): linear from a rest 15 degrees below the stop, sized by its torque
#: at 45 degrees; the claw's hook on the lever for each claw model (run_demo.py MECH_LEVER_GRASP_*).
REST_DEG, REFERENCE_DEG, STOP_DEG = -15.0, 45.0, 60.0
HOOK_M = {"spring": 0.095, "lips": 0.080}


def run_dir(tag: str, torque: str, claw: str) -> Path | None:
    matches = sorted(RUNS.glob(f"*_st{tag}_lev{torque}_{claw}"))
    return matches[-1] if matches else None


def summary(path: Path) -> dict:
    return json.loads((path / "summary.json").read_text())


def lever_force_n(torque_45: float, handle_deg: float, claw: str) -> float:
    """Force along the lever's arc at the hook that holds the spring at `handle_deg` (the lever's own 0.03 N·m ignored)."""
    angle = min(handle_deg, STOP_DEG)
    return torque_45 * (angle - REST_DEG) / (REFERENCE_DEG - REST_DEG) / HOOK_M[claw]


#: The stiff-door check: lip claw, lever 0.4 N·m, a door closer, by stance. Square is the overbite sweep's runs.
DOOR_STANCES = ((-30, "stm30_door{door}_lips"), (-15, "stm15_door{door}_lips"), (0, "seed1_door{door}_olaw"),
                (15, "stp15_door{door}_lips"), (30, "stp30_door{door}_lips"))
DOORS = ((12, 0), (16, 3))


def door_figure() -> None:
    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.6))
    for door, slot in DOORS:
        rows = [summary(sorted(RUNS.glob("*_" + pattern.format(door=door)))[-1])["summary"]
                for _, pattern in DOOR_STANCES]
        stances = [stance for stance, _ in DOOR_STANCES]
        style = {"color": SERIES[slot], "lw": 2, "marker": "o", "ms": 4}
        axes[0].plot(stances, [r["successes"] for r in rows], label=f"door closer {door} N·m", **style)
        axes[1].plot(stances, [r["max_door_deg_median"] for r in rows], **style)
        axes[2].plot(stances, [r["claw_lost"] for r in rows], **style)
    for ax, title, label in ((axes[0], "Opened", "placements opened (door past 30 deg), of 16"),
                             (axes[1], "How far the door opened", "largest door angle, median (deg)"),
                             (axes[2], "How often the claw lost the lever", "attempts where the bar left the claw, of 16")):
        ax.set_title(title, fontsize=10, color=INK, loc="left")
        ax.set_ylabel(label)
        ax.set_xlabel("stance (deg): − hinge side, + latch side")
        ax.set_xticks([stance for stance, _ in DOOR_STANCES])
        ax.grid(axis="y", color=GRID, lw=0.8)
    for ax in (axes[0], axes[2]):
        ax.set_ylim(-0.5, 16.5)
        ax.set_yticks((0, 4, 8, 12, 16))
    axes[1].axhline(30.0, color=MUTED, lw=1, ls=":")
    fig.suptitle("Stiff doors by stance: L-lip claw, mechanism policy + force law, lever 0.4 N·m, 16 placements per "
                 "point", fontsize=10, color=INK, x=0.01, ha="left")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.tight_layout(rect=(0, 0.08, 1, 0.95))
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False, fontsize=8)
    fig.savefig(HERE / "combiner_stance_door.png", dpi=160)


def main() -> None:
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.9))
    x = [float(t) for t in TORQUES]
    for panel, (claw, title) in enumerate(CLAWS):
        ax = axes[panel]
        for stance, tag, slot in STANCES:
            points = [(float(t), summary(path)["summary"]["latch_released"])
                      for t in TORQUES if (path := run_dir(tag, t, claw))]
            ax.plot(*zip(*points), color=SERIES[slot], lw=2, marker="o", ms=4, label="square to the door" if stance == 0 else f"{stance:+d}° ({'latch' if stance > 0 else 'hinge'} side)")
        ax.set_ylim(-0.5, 16.5)
        ax.set_yticks((0, 4, 8, 12, 16))
        ax.set_ylabel("placements where the latch released, of 16")
        ax.set_title(title, fontsize=10, color=INK, loc="left")
        ax.set_xlabel("lever spring: torque at 45 degrees (N·m)")
        ax.set_xticks(x)
        ax.grid(axis="y", color=GRID, lw=0.8)
    # The force the policy held along the lever's arc: the spring's torque at the angle reached, per attempt.
    ax = axes[2]
    for stance, tag, slot in STANCES:
        points = []
        for t in TORQUES:
            if (path := run_dir(tag, t, "spring")) is None:
                continue
            angles = sorted(a.get("max_handle_deg", 0.0) for a in summary(path)["attempts"])
            points.append((float(t), lever_force_n(float(t), angles[len(angles) // 2], "spring")))
        ax.plot(*zip(*points), color=SERIES[slot], lw=2, marker="o", ms=4)
    ax.set_ylabel("force held along the lever's arc, median (N)")
    ax.set_title("Spring claw: force held on the lever", fontsize=10, color=INK, loc="left")
    ax.annotate("at 0.4 N·m every stance but +45° reaches the stop: a floor", (0.4, 0.4), color=MUTED, fontsize=7)
    ax.set_xlabel("lever spring: torque at 45 degrees (N·m)")
    ax.set_xticks(x)
    ax.set_ylim(bottom=0)
    ax.grid(axis="y", color=GRID, lw=0.8)
    fig.suptitle("Standing combiner demo, mechanism policy + force law, free door, 16 placements per point: the robot "
                 "square to the door or turned to one side", fontsize=10, color=INK, x=0.01, ha="left")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.tight_layout(rect=(0, 0.08, 1, 0.95))
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False, fontsize=8)
    fig.savefig(HERE / "combiner_stance_sweep.png", dpi=160)
    door_figure()


if __name__ == "__main__":
    main()

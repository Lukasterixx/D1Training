"""Week 1 smoke posture traces, from the recorded runs' smoke_trace.csv.

    python results/week_01/figures/smoke_posture.py
"""
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS = Path(__file__).resolve().parents[1] / "runs"
LEGS = [("20260915T093730_338089Z_smoke_seed42", "Unitree envelope (default)", "#2a78d6", "-"),
        ("20260915T093743_674698Z_smoke_seed42", "stock DCMotor", "#eb6834", "--"),
        ("20260915T093756_424913Z_smoke_seed42", "pre-port (DCMotor, no latency)", "#1a1a1a", ":")]
ARM = [("20260915T093730_338089Z_smoke_seed42", "acceleration drives (URDF import)", "#eb6834"),
       ("20260915T095815_165623Z_smoke_seed42", "force drives (d1_servo fix)", "#2a78d6")]
STEP_S = 0.02


def trace(run):
    with (RUNS / run / "smoke_trace.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    return {key: [float(row[key]) for row in rows] for key in rows[0]}


def main():
    fig, (legs, arm) = plt.subplots(1, 2, figsize=(11, 3.8))
    for run, label, color, style in LEGS:
        data = trace(run)
        legs.plot([s * STEP_S for s in data["step"]], data["base_height_m"], color=color, ls=style, lw=1.6, label=label)
    legs.set(xlabel="time (s)", ylabel="base height (m)", xlim=(0, 12),
             title="Zero actions: base height by leg motor model")
    legs.annotate("time-limit reset at 10 s", (10.0, 0.40), xytext=(5.2, 0.40), fontsize=8,
                  arrowprops={"arrowstyle": "->", "color": "#5f6570"})
    legs.legend(fontsize=8, frameon=False, loc="lower center")
    for run, label, color in ARM:
        data = trace(run)
        arm.plot([s * STEP_S for s in data["step"]], data["arm_joint_dev_rad"], color=color, lw=1.6, label=label)
    arm.set(xlabel="time (s)", ylabel="largest arm joint deviation (rad)", xlim=(0, 10),
            title="Zero actions: arm hold by drive type")
    arm.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    out = Path(__file__).with_suffix(".png")
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()

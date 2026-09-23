"""The UniFP policy in the two simulators: Isaac Gym (where it trained) against Isaac Lab.

Reads the recorded runs' trace.csv, which both stacks write with the same leading columns
(`unifp_go2d1/play_policy.py --out` and `unifp_isaaclab/rollout.py`).

    python results/week_01/figures/unifp_sim2sim.py

The two runs are not the same episode -- each stack generates its own end-effector goal
trajectory from its own RNG -- so read the level and the spread, not any individual crossing.
"""
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS = Path(__file__).resolve().parents[1] / "runs"
STEP_S = 0.02

#: Plot 20 s, not the full 30 s of the rollouts. The Isaac Gym side resets at 20 s
#: (legged_gym's `env.episode_length_s`, which `play_policy.py` does not disable), so past that
#: its trace is a second episode starting from a randomised pose -- the violent excursions it
#: shows there are the reset, not the controller. The Isaac Lab port has no episode limit.
LAST_STEP = 1000

GYM = "#eb6834"
LAB = "#2a78d6"


def trace(run):
    with (RUNS / run / "trace.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    out = {}
    for key in rows[0]:
        try:
            out[key] = [float(row[key]) for row in rows]
        except ValueError:
            continue
    return out


def main(gym_policy, lab_policy, gym_zero, lab_zero, dest):
    series = {name: trace(run) for name, run in
              (("gym_policy", gym_policy), ("lab_policy", lab_policy),
               ("gym_zero", gym_zero), ("lab_zero", lab_zero))}

    fig, (height, error) = plt.subplots(1, 2, figsize=(11, 3.8))
    for key, label, color, style in (
            ("gym_policy", "Isaac Gym, model_48800", GYM, "-"),
            ("lab_policy", "Isaac Lab, model_48800", LAB, "-"),
            ("gym_zero", "Isaac Gym, zero actions", GYM, ":"),
            ("lab_zero", "Isaac Lab, zero actions", LAB, ":")):
        data = series[key]
        base = data["base_z_m"][:LAST_STEP]
        tip = data["tip_err_l1_m"][:LAST_STEP]
        time = [i * STEP_S for i in range(len(base))]
        height.plot(time, [v * 100 for v in base], color=color, ls=style, lw=1.2, label=label)
        error.plot(time, [v * 100 for v in tip], color=color, ls=style, lw=1.2, label=label)

    height.axhline(30.0, color="#888888", lw=0.8, ls="--")
    height.annotate("base height target, 30 cm", (10.5, 30.4), fontsize=7, color="#666666")
    height.axhline(15.0, color="#cc2222", lw=0.8, ls="--")
    height.annotate("fallen, 15 cm", (0.5, 15.4), fontsize=7, color="#cc2222")
    height.set_ylabel("base height (cm)")
    height.set_ylim(12, 38)
    height.set_title("The robot stays up, or does not")

    error.set_ylabel("tool-tip error, L1 (cm)")
    error.set_title("Tracking the commanded goal")
    error.set_yscale("log")

    for axis in (height, error):
        axis.set_xlabel("time (s)")
        axis.grid(alpha=0.3)
        axis.legend(fontsize=7)

    fig.suptitle("UniFP model_48800, standing command, flat ground, no external forces "
                 "(first 20 s; the Isaac Gym episode resets there)", fontsize=10)
    fig.tight_layout()
    fig.savefig(dest, dpi=160)
    print(f"wrote {dest}")


if __name__ == "__main__":
    import sys
    main(*sys.argv[1:5], dest=sys.argv[5] if len(sys.argv) > 5
         else str(Path(__file__).resolve().parent / "unifp_sim2sim.png"))

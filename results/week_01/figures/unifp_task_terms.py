"""All 27 reward terms, Isaac Gym against the Isaac Lab port of the same task.

    python results/week_01/figures/unifp_task_terms.py

The Isaac Gym side is `tests/data/unifp_task_48800.npz`, 160 policy steps recorded out of the
running training environment by `unifp_go2d1/dump_interface.py`, one environment. The Isaac Lab
side is the recorded `play` run's `run.json`, 16 environments over 400 steps. Both are
`model_48800` on flat ground with no external forces.

**They are not the same episode.** Each stack draws its own velocity commands and its own goal
trajectory, so what this compares is the level of each term, not a trajectory. That is the point:
a single total can agree while two terms are wrong in opposite directions, and the terms that
disagree here are the ones a reader should be told about rather than the ones that cancel.
"""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / "tests/data/unifp_task_48800.npz"
GYM = "#eb6834"
LAB = "#2a78d6"


def main(lab_run, dest):
    lab = json.loads((Path(lab_run) / "run.json").read_text())["reward_terms_per_step"]
    data = np.load(FIXTURE, allow_pickle=True)
    # Step 0 is short by whatever the environment accumulated before the recording loop started,
    # which is why every comparison against this fixture drops it.
    gym = {name: float(data[f"reward_{name}"][1:].mean()) for name in lab}

    order = sorted(lab, key=lambda n: -abs(gym[n]))
    positions = np.arange(len(order))
    gym_values = np.array([gym[n] for n in order])
    lab_values = np.array([lab[n] for n in order])

    fig, (big, small) = plt.subplots(2, 1, figsize=(11, 8),
                                     gridspec_kw={"height_ratios": [1, 1.4]})

    # The six terms that carry the reward, on their own axis: on one shared scale the other
    # twenty-one are invisible.
    top = 6
    width = 0.38
    big.bar(positions[:top] - width / 2, gym_values[:top], width, color=GYM, label="Isaac Gym")
    big.bar(positions[:top] + width / 2, lab_values[:top], width, color=LAB, label="Isaac Lab")
    big.set_xticks(positions[:top])
    big.set_xticklabels(order[:top], rotation=20, ha="right", fontsize=8)
    big.set_ylabel("reward per step")
    big.set_title("The six terms that carry the reward")
    big.legend(fontsize=8)
    big.grid(alpha=0.3, axis="y")

    rest = positions[top:] - top
    small.bar(rest - width / 2, gym_values[top:], width, color=GYM, label="Isaac Gym")
    small.bar(rest + width / 2, lab_values[top:], width, color=LAB, label="Isaac Lab")
    small.set_xticks(rest)
    small.set_xticklabels(order[top:], rotation=60, ha="right", fontsize=7)
    small.set_ylabel("reward per step")
    small.set_title("The remaining twenty-one, mostly regularisation")
    small.legend(fontsize=8)
    small.grid(alpha=0.3, axis="y")

    total_gym, total_lab = gym_values.sum(), lab_values.sum()
    fig.suptitle(
        f"UniFP model_48800, flat ground, no external forces.  "
        f"Total {total_gym:.5f} (Isaac Gym) against {total_lab:.5f} (Isaac Lab port), "
        f"{100 * (total_lab - total_gym) / total_gym:+.1f}%", fontsize=10)
    fig.tight_layout()
    fig.savefig(dest, dpi=160)
    print(f"wrote {dest}")

    worst = sorted(order, key=lambda n: -abs(lab[n] - gym[n]))[:5]
    for name in worst:
        print(f"  {name:28s} gym {gym[name]:+.5f}  lab {lab[name]:+.5f}  "
              f"diff {lab[name] - gym[name]:+.5f}")


if __name__ == "__main__":
    import sys

    default_run = ROOT / "results/week_01/runs/20260920T041756_play_seed42_baseline_48800"
    main(sys.argv[1] if len(sys.argv) > 1 else default_run,
         sys.argv[2] if len(sys.argv) > 2
         else str(Path(__file__).resolve().parent / "unifp_task_terms.png"))

"""The insertion task's fifth training run: what the policy did with the slot over 2,500 iterations.

Reads the recorded run's `scalars.csv` (training-time `Insertion/*` logs, sampled at resets: not the frozen evaluation).
Week 2 log, 2026-09-26, "Force-aware insertion of a handle into a key slot".

    python3 results/week_02/figures/insertion_training.py     # -> insertion_training.png
"""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
RUN = HERE.parent / "runs" / "20260925T164048_train_seed1_ins_v5"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
WINDOW = 50


def smooth(xs, ys):
    out_x, out_y = [], []
    for i in range(0, len(xs) - WINDOW + 1, WINDOW // 2):
        chunk = [y for y in ys[i:i + WINDOW] if y == y]
        if chunk:
            out_x.append(xs[i + WINDOW // 2])
            out_y.append(sum(chunk) / len(chunk))
    return out_x, out_y


def main() -> None:
    rows = list(csv.DictReader(open(RUN / "scalars.csv")))
    it = [int(r["step"]) for r in rows]
    get = lambda key: [float(r[key]) if r.get(key) not in (None, "") else float("nan") for r in rows]
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.7))
    for key, label, slot in (("Insertion/missed_fraction", "missed the pre-insertion pose", 0),
                             ("Insertion/inserted_fraction", "in past the face (of those started)", 2),
                             ("Insertion/damaged_fraction", "damaged (of those started)", 1),
                             ("Insertion/seated_fraction", "seated (of those started)", 3)):
        axes[0].plot(*smooth(it, get(key)), color=SERIES[slot], lw=2, label=label)
    axes[0].set_ylim(-0.03, 1.03)
    axes[0].set_title("What the insertion episodes did", fontsize=10, color=INK, loc="left")
    axes[0].set_ylabel("fraction of episodes ending")
    axes[1].plot(*smooth(it, get("Insertion/peak_contact_n")), color=SERIES[1], lw=2)
    axes[1].axhline(150.0, color=MUTED, lw=1, ls=":")
    axes[1].annotate("damage", (it[0], 152), color=MUTED, fontsize=7)
    axes[1].set_title("Peak contact on the key (started episodes)", fontsize=10, color=INK, loc="left")
    axes[1].set_ylabel("N")
    for key, label, slot in (("Episode_Reward/ins_reach", "jaw centre's precision (ins_reach)", 0),
                             ("Episode_Reward/tracking_ee_orn_approach", "approach (tracking_ee_orn_approach)", 4)):
        axes[2].plot(*smooth(it, get(key)), color=SERIES[slot], lw=2, label=label)
    axes[2].set_title("Two of the terms it was paid on", fontsize=10, color=INK, loc="left")
    axes[2].set_ylabel("episode reward")
    for ax in axes:
        ax.set_xlabel("iteration")
        ax.grid(axis="y", color=GRID, lw=0.8)
    axes[0].legend(frameon=False, fontsize=7, loc="center right")
    axes[2].legend(frameon=False, fontsize=7, loc="lower right")
    fig.suptitle("Insertion training v5 (from model_17499, 4,096 environments), training-time logs smoothed over 50 "
                 "iterations: not a frozen evaluation", fontsize=10, color=INK, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(HERE / "insertion_training.png", dpi=160)


if __name__ == "__main__":
    main()

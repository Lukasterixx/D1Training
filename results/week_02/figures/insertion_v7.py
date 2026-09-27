"""The insertion task once it could be learned: v7's training and its checkpoints on the frozen slots.

Reads the recorded runs: v7's `scalars.csv` (training-time `Insertion/*` logs, sampled at resets, on the curriculum's
easier slots below level 1) and every `*_ins_eval_seed1_ins_eval_lateral_v7_*` run's `ins_eval.json` (the frozen 96
slots at the real 1 mm clearance, with the compliant grip, the outer loops and the home hold). Week 2 log, 2026-09-27.

    python3 results/week_02/figures/insertion_v7.py     # -> insertion_v7.png
"""
from __future__ import annotations

import csv
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "runs"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
WINDOW = 25
#: v5 `model_19998` on the same frozen slots, as the starting point.
BASELINES = {"v5, rigid key, no loops": "20260926T140026_ins_eval_seed1_ins_eval_lateral_v5_noloops_newmount",
             "v5 + grip + loops": "20260926T142228_ins_eval_seed1_ins_eval_lateral_v5_grip"}


def smooth(xs, ys):
    out_x, out_y = [], []
    for i in range(0, len(xs) - WINDOW + 1, max(1, WINDOW // 2)):
        chunk = [y for y in ys[i:i + WINDOW] if y == y]
        if chunk:
            out_x.append(xs[i + WINDOW // 2])
            out_y.append(sum(chunk) / len(chunk))
    return out_x, out_y


def main() -> None:
    train = next(p for p in sorted(RUNS.glob("*_train_seed1_ins_v7")))
    rows = list(csv.DictReader(open(train / "scalars.csv")))
    it = [int(r["step"]) for r in rows]
    get = lambda key: [float(r[key]) if r.get(key) not in (None, "") else float("nan") for r in rows]
    evals = []
    for path in sorted(RUNS.glob("*_ins_eval_seed1_ins_eval_lateral_v7_*")):
        m = re.search(r"_v7_(\d+)(_hold)?$", path.name)
        if m is None:
            continue                                    # traces and ablations, not the checkpoint series
        summary = json.loads((path / "ins_eval.json").read_text())
        evals.append((int(m.group(1)), summary))
    by_it = {}
    for step, summary in evals:
        by_it[step] = summary                       # a re-run of the same checkpoint replaces it (deterministic)
    steps = sorted(by_it)

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 2, figsize=(12.6, 4.4))
    ax = axes[0]
    for key, label, slot in (("Insertion/seated_fraction", "seated", 0), ("Insertion/damaged_fraction", "damaged", 1)):
        ax.plot(*smooth(it, get(key)), color=SERIES[slot], lw=2, label=label)
    ax.plot(it, get("Insertion/curriculum_level"), color=MUTED, lw=1.2, ls="--", label="curriculum level (1 = the real slot)")
    ax.set_ylim(-0.03, 1.03)
    ax.set_title("In training (of episodes with a slot started)", fontsize=10, color=INK, loc="left")
    ax.set_xlabel("iteration")
    ax.legend(frameon=False, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3)
    ax = axes[1]
    ax.plot(steps, [by_it[s]["seated"] for s in steps], color=SERIES[0], lw=2, marker="o", ms=5, label="seated (frozen metric)")
    ax.plot(steps, [by_it[s].get("ever_seated", float("nan")) for s in steps], color=SERIES[2], lw=1.5, marker="o", ms=4,
            label="reached the seat at some point")
    ax.plot(steps, [by_it[s]["damaged"] for s in steps], color=SERIES[1], lw=2, marker="o", ms=5, label="damaged")
    for (label, run), style in zip(BASELINES.items(), (":", "-.")):
        path = RUNS / run / "ins_eval.json"
        if path.exists():
            ax.axhline(json.loads(path.read_text())["seated"], color=MUTED, lw=1, ls=style)
            ax.annotate(f"{label}: {json.loads(path.read_text())['seated']}", (steps[0] if steps else 0,
                        json.loads(path.read_text())["seated"] + 1.5), color=MUTED, fontsize=7)
    ax.set_ylim(0, 96)
    ax.set_title("Frozen evaluation: 96 slots at 1 mm clearance, estimate 0-4.5 mm off", fontsize=10, color=INK, loc="left")
    ax.set_xlabel("checkpoint (iteration)")
    ax.set_ylabel("of 96")
    ax.legend(frameon=False, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3)
    for a in axes:
        a.grid(axis="y", color=GRID, lw=0.8)
    fig.suptitle("Insertion v7: compliant grip, outer loops, arm noise 0.15, from v5 model_19998 (4,096 environments); "
                 "training logs smoothed over 25 iterations", fontsize=10, color=INK, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.subplots_adjust(bottom=0.25)
    fig.savefig(HERE / "insertion_v7.png", dpi=160)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""The figure for Weekly Update 4 (2026-10-09), drawn for a reader outside the repository.

- `weekly_update_rope_pull.png`: the rope pull before and after training, redrawn from `rope_pull.json` (the extract
  `rope_pull.py` wrote beside `rope_pull.png`) with plain labels and sized for the update's 3.7-inch column. The
  extract is at the policy rate (50 Hz, each point the mean of four physics substeps), so the tie-on spikes are
  softer than in `rope_pull.png`, which was drawn from the 200 Hz traces in `logs/` on the training machine.

    python3 results/week_04/figures/weekly_update_figures.py
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).parent
# Same reference-palette slots as rope_pull.py (validated as a pair, light mode).
BEFORE, AFTER = "#eb6834", "#2a78d6"
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def rope_pull() -> Path:
    extract = json.loads((HERE / "rope_pull.json").read_text())
    before, after = extract["series"]
    plt.rcParams.update({"font.size": 8, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(5.0, 4.1), sharex=True,
                                      gridspec_kw={"height_ratios": [3, 2]}, facecolor=SURFACE)
    for ax in (top, bottom):
        ax.set_facecolor(SURFACE)
        ax.grid(axis="y", color=GRID, linewidth=0.6)
        ax.axvline(0.0, color=MUTED, linewidth=0.8, linestyle=":")

    top.plot(after["t_from_release"], after["cmd_n"], color=MUTED, linewidth=1.0, linestyle="--",
             label="force asked for")
    for s, colour, label in ((before, BEFORE, "before: Week 2's pull policy"),
                             (after, AFTER, "after: trained on the rope")):
        top.plot(s["t_from_release"], s["tension_mean_n"], color=colour, linewidth=1.5, label=label)
        bottom.plot(s["t_from_release"], s["pitch_deg"], color=colour, linewidth=1.5)

    top.set_ylabel("rope tension (N)")
    top.set_ylim(-2, 68)
    top.legend(frameon=False, loc="upper left", fontsize=7)
    top.annotate("rope snaps", (0.0, 64), xytext=(4, 0), textcoords="offset points", color=MUTED, fontsize=7)

    bottom.set_ylabel("body pitch (°)\n+ = nose down")
    bottom.set_xlabel("seconds from the rope snapping")
    bottom.set_xlim(*extract["window_from_release_s"])
    fall_t, fall_p = min(zip(before["t_from_release"], before["pitch_deg"]), key=lambda tp: tp[1])
    bottom.annotate("falls over backwards", (fall_t, fall_p), xytext=(8, 0), textcoords="offset points",
                    ha="left", va="center", color=INK, fontsize=7)
    bottom.annotate("leans into the rope", (-9.0, 21.5), xytext=(0, 3), textcoords="offset points",
                    ha="center", va="bottom", color=INK, fontsize=7)
    bottom.annotate("recovers", (2.0, 9.0), xytext=(0, 4), textcoords="offset points",
                    ha="center", va="bottom", color=INK, fontsize=7)
    bottom.set_ylim(-62, 34)

    fig.suptitle("Pulling a rope at 20, 35 then 50 N until it snaps (simulation, one episode)", fontsize=8.5,
                 color=INK, x=0.02, ha="left")
    fig.tight_layout()
    out = HERE / "weekly_update_rope_pull.png"
    fig.savefig(out, dpi=220, facecolor=SURFACE)
    plt.close(fig)
    return out


if __name__ == "__main__":
    print("wrote", rope_pull())

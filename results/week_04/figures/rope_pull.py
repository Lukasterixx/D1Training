#!/usr/bin/env python3
"""Figure for the rope-pull task (Week 4, 2026-10-07): one frozen evaluation episode under two policies.

Top: the rope's tension at the physics rate (200 Hz) against the command, through the 20 / 35 / 50 N holds and the
release at the end of the last. Bottom: the base's pitch over the same time (positive nose-down), which is where a
fall shows. Each policy's trace is its own `rope_eval` run (`logs/unifp_train/*_rope_eval_*/rope_trace.npz`, not
committed); the episode drawn is the same frozen row in both, and the extract plotted is written beside the figure
(`rope_pull.json`) so the figure can be checked without the logs.

    python3 results/week_04/figures/rope_pull.py --runs <baseline run dir> <trained run dir> \
        --labels "hook v4 13000" "rope v1" --env 27 --out results/week_04/figures/rope_pull.png
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

POLICY_DT = 0.02
# The reference palette's categorical slots 1 and 2 (validated as a set in both modes) and its muted ink.
COLOURS = ("#eb6834", "#2a78d6", "#1baf7a")
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def load(run: Path, env: int) -> dict:
    trace = np.load(run / "rope_trace.npz")
    episodes = json.loads((run / "rope_eval_episodes.json").read_text())["episodes"]
    tension = trace["tension_sub"][:, env, :].reshape(-1)
    steps = trace["stage"].shape[0]
    return {"tension": tension, "t_sub": np.arange(len(tension)) * POLICY_DT / trace["tension_sub"].shape[2],
            "cmd": trace["cmd"][:, env], "t": np.arange(steps) * POLICY_DT,
            "pitch_deg": np.degrees(trace["pitch"][:, env]), "released": trace["released"][:, env],
            "episode": episodes[env]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runs", nargs="+", required=True, type=Path)
    parser.add_argument("--labels", nargs="+", required=True)
    parser.add_argument("--env", type=int, default=27, help="Frozen row to draw (27: 0.40 m grip straight ahead, 4,000 N/m, snap noticed).")
    parser.add_argument("--out", type=Path, default=Path(__file__).with_suffix(".png"))
    args = parser.parse_args()
    series = [load(run, args.env) for run in args.runs]

    # Each policy ties its rope when its own tool settles, so the episodes are drawn against the time of their own
    # release: the staircase before it is the same length in every episode.
    for s in series:
        released = np.flatnonzero(s["released"] > 0.5)
        s["release_s"] = float(released[0]) * POLICY_DT if len(released) else None
        shift = s["release_s"] if s["release_s"] is not None else 0.0
        s["x"], s["x_sub"] = s["t"] - shift, s["t_sub"] - shift
    start, stop = -13.5, 4.0

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(8.0, 5.2), sharex=True, gridspec_kw={"height_ratios": [3, 2]},
                                      facecolor=SURFACE)
    for ax in (top, bottom):
        ax.set_facecolor(SURFACE)
        ax.grid(axis="y", color=GRID, linewidth=0.6)
        ax.axvline(0.0, color=MUTED, linewidth=0.8, linestyle=":")
    for i, (s, label, colour) in enumerate(zip(series, args.labels, COLOURS)):
        keep = (s["x"] >= start) & (s["x"] <= stop)
        top.plot(s["x"][keep], s["cmd"][keep], color=MUTED, linewidth=1.1, linestyle="--",
                 label="command" if i == 0 else None)
    for s, label, colour in zip(series, args.labels, COLOURS):
        keep = (s["x_sub"] >= start) & (s["x_sub"] <= stop)
        top.plot(s["x_sub"][keep], s["tension"][keep], color=colour, linewidth=1.5, label=label)
        keep = (s["x"] >= start) & (s["x"] <= stop)
        bottom.plot(s["x"][keep], s["pitch_deg"][keep], color=colour, linewidth=1.5, label=label)
    top.set_ylabel("rope tension (N)")
    bottom.set_ylabel("base pitch (deg, + nose-down)")
    bottom.set_xlabel("time from the anchor letting go (s)")
    top.set_xlim(start, stop)
    top.annotate("anchor lets go", (0.0, top.get_ylim()[1] * 0.95), xytext=(4, 0), textcoords="offset points",
                 color=MUTED, fontsize=8)
    top.legend(frameon=False, loc="upper left", fontsize=8)
    row = series[0]["episode"]
    fig.suptitle(f"Rope pull, frozen episode {args.env}: grip {row.get('grip_height_m', float('nan')):.2f} m high, "
                 f"{row['stiffness']:.0f} N/m rope, release '{row['release']}'", fontsize=10, color=INK, x=0.02,
                 ha="left")
    fig.tight_layout()
    fig.savefig(args.out, dpi=160, facecolor=SURFACE)

    extract = {"env": args.env, "runs": [str(r) for r in args.runs], "labels": args.labels,
               "window_from_release_s": [start, stop], "series": []}
    for s, label in zip(series, args.labels):
        keep = (s["x"] >= start) & (s["x"] <= stop)
        extract["series"].append({"label": label, "release_s": s["release_s"],
                                  "t_from_release": np.round(s["x"][keep], 3).tolist(),
                                  "tension_mean_n": np.round(np.nanmean(
                                      s["tension"].reshape(-1, 4)[: len(s["t"])][keep], axis=1), 2).tolist(),
                                  "cmd_n": np.round(s["cmd"][keep], 2).tolist(),
                                  "pitch_deg": np.round(s["pitch_deg"][keep], 2).tolist(),
                                  "episode": s["episode"]})
    args.out.with_suffix(".json").write_text(json.dumps(extract, allow_nan=True))
    print(f"wrote {args.out} and {args.out.with_suffix('.json')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

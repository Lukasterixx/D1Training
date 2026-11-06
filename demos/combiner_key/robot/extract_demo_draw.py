#!/usr/bin/env python3
"""The demo's own holster draw, out of one of its records, for `holster_dog.py --demo_draw`. PC only; commands nothing.

    python demos/combiner_key/robot/extract_demo_draw.py logs/combiner_door_robot/<a live chain> --out <json>

`holster_dog.plan` is the plan before the dog's tuning: at the grip it is up to 5 degrees off what the demo commands, up
to 3.4 through the draw (the sag loop), and the jaws shut less. A real key sits in a sleeve with under 0.4 mm to spare, so a
test of the draw on the dog plays the demo's. Its first chain's phases, skipping the lock between `h_clear` and `h_unclear`:

- `goals`: where the joint-space moves end (the trace holds only their goals; the link planned the way there);
- `paths`: the streamed phases as commanded every 100 ms, retimed where they ran faster than `MAX_SPEED_DEG_S`, the
  arm link's limit. The link used to clip those steps as it followed them, so the path is the same and only slower there.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

PERIOD_S = 0.1
#: Under the link's 45 deg/s.
MAX_SPEED_DEG_S = 44.0
GOALS = {"reach": "h_reach"}
#: The streamed phases, this program's name for each.
PATHS = {"on": "h_on", "draw": "h_draw", "exit": "h_exit", "hold": "h_out", "clear": "h_clear", "unclear": "h_unclear",
         "align": "h_align", "enter": "h_enter", "back": "h_back", "off": "h_off"}


def first_phases(rows: list[dict]) -> dict[str, list[dict]]:
    """Each phase's rows, from its first appearance only (the record runs the chain and its way back)."""
    out, current = {}, None
    for row in rows:
        if row["phase"] != current:
            current = row["phase"]
            taking = current not in out
            if taking:
                out[current] = []
        if taking:
            out[current].append(row)
    return out


def commands(rows: list[dict]) -> np.ndarray:
    return np.array([[float(r[f"arm_cmd{i}_deg"]) for i in range(6)] for r in rows])


def retimed(points: np.ndarray, period_s: float = PERIOD_S, max_speed: float = MAX_SPEED_DEG_S) -> np.ndarray:
    """The points, `period_s` apart in the record, resampled `period_s` apart with no joint above `max_speed`: each
    segment takes its own time or as long as its largest step needs, whichever is longer."""
    steps = np.abs(np.diff(points, axis=0)).max(axis=1)
    times = np.concatenate([[0.0], np.cumsum(np.maximum(period_s, steps / max_speed))])
    grid = np.append(np.arange(0.0, times[-1], period_s), times[-1])
    return np.stack([np.interp(grid, times, points[:, j]) for j in range(points.shape[1])], axis=1)


def extract(run: Path) -> dict:
    with (run / "trace.csv").open() as handle:
        phases = first_phases(list(csv.DictReader(handle)))
    out = {"source": run.name, "turn": commands(phases["h_turn"])[-1].tolist(),
           "goals": {name: commands(phases[phase])[-1].tolist() for name, phase in GOALS.items()},
           "paths": {name: np.round(retimed(commands(phases[phase])), 3).tolist() for name, phase in PATHS.items()},
           "grip_units": float(phases["grip"][-1]["gripper_cmd"]), "open_units": float(phases["h_on"][-1]["gripper_cmd"])}
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("run", type=Path)
    p.add_argument("--out", type=Path, required=True)
    args = p.parse_args(argv)
    args.out.write_text(json.dumps(extract(args.run)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

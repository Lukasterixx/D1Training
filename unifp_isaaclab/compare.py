"""Put an Isaac Gym trace and an Isaac Lab trace side by side.

Both stacks write a `trace.csv` whose first columns are the same by construction
(`unifp_go2d1/play_policy.py --out` and `unifp_isaaclab/rollout.py`), so the sim-to-sim question
comes down to comparing distributions of the same few columns over the same window.

Distributions, not steps. The two runs are not the same episode: each side generates its own
end-effector goal trajectory from its own RNG, so step 400 on one is not step 400 on the other.
What is shared is the goal *distribution* and the command, which makes a median over a long
window meaningful and a step-by-step difference meaningless.

    python -m unifp_isaaclab.compare --gym <gym run>/trace.csv --lab <lab run>/trace.csv

Standard library only, so it runs from the system Python, the Isaac environment or the legacy one.
"""
from __future__ import annotations

import argparse
import csv
from statistics import median

#: Steps to drop from the front. Both sides spawn above standing height and spend the first
#: moments dropping and settling, and the 32-frame observation history is still filling with
#: zeros -- neither is the behaviour being compared.
SETTLE_STEPS = 100

#: Last step to include. **The Isaac Gym side resets at 20 s** -- legged_gym's
#: `env.episode_length_s` is 20 and `play_policy.py` does not disable it, so the robot is
#: teleported back to a randomised start pose partway through a 1500-step rollout and the trace
#: after that is a *second* episode. The Isaac Lab port has no episode limit and runs straight
#: through. Comparing full traces therefore averages one continuous run against one-and-a-half
#: episodes, which quietly flatters or punishes whichever side happens to be settled; 1000 steps
#: is 20 s, the longest window both sides cover exactly once.
LAST_STEP = 1000

COLUMNS = (
    ("tip_err_l1_m", "tool-tip error, L1", 100.0, "cm"),
    ("base_z_m", "base height", 100.0, "cm"),
    ("force_est_n", "force estimate, |F|", 1.0, "N"),
)


def read(path: str, settle: int = SETTLE_STEPS, last: int = LAST_STEP) -> dict[str, list[float]]:
    with open(path) as handle:
        rows = list(csv.DictReader(handle))[settle:last]
    if not rows:
        raise SystemExit(f"{path}: no rows in window [{settle}, {last})")
    out = {}
    for name in rows[0]:
        try:
            out[name] = [float(row[name]) for row in rows]
        except (TypeError, ValueError):
            continue
    return out


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gym", required=True, help="trace.csv from the Isaac Gym (training) stack")
    parser.add_argument("--lab", required=True, help="trace.csv from the Isaac Lab port")
    parser.add_argument("--settle", type=int, default=SETTLE_STEPS)
    parser.add_argument("--last", type=int, default=LAST_STEP,
                        help="last step to include; the default stops at the Isaac Gym episode reset")
    args = parser.parse_args()

    gym, lab = read(args.gym, args.settle, args.last), read(args.lab, args.settle, args.last)
    print(f"steps {args.settle}-{args.last} ({(args.last - args.settle) * 0.02:.1f} s), "
          f"{len(gym['step'])} vs {len(lab['step'])} rows\n")
    print(f"{'':30s} {'Isaac Gym':>22s} {'Isaac Lab':>22s}")
    print(f"{'':30s} {'median':>10s} {'p90':>11s} {'median':>10s} {'p90':>11s}")
    for key, label, scale, unit in COLUMNS:
        if key not in gym or key not in lab:
            continue
        row = f"{label + ' (' + unit + ')':30s}"
        for data in (gym, lab):
            row += f" {median(data[key]) * scale:10.2f} {percentile(data[key], 0.9) * scale:11.2f}"
        print(row)

    # A fall is the one outcome where a median is misleading: a robot on its back can sit at a
    # modest tip error while having failed completely.
    for name, data in (("Isaac Gym", gym), ("Isaac Lab", lab)):
        if "base_z_m" in data:
            low = sum(1 for value in data["base_z_m"] if value < 0.15)
            print(f"{name}: {low} of {len(data['base_z_m'])} steps below 0.15 m "
                  f"({100.0 * low / len(data['base_z_m']):.1f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

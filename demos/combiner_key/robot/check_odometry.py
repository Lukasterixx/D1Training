#!/usr/bin/env python3
"""Item 1b (2026-11-19): how far the base moves during a simulated insertion, and how well the dog's leg odometry
(`base_odometry.LegOdometry`) follows it, from `--insert_trace_body` runs (`insert_trace.npz`). CPU only.

    python demos/combiner_key/robot/check_odometry.py logs/unifp_train/<run> [<run> ...] [--contact_n 10]

Per run: the base's displacement from `--start_s` into each episode to its end (x, the robot's forward at the start and
so about the lock's axis; y; z), how far the true feet moved, and the odometry's error against the simulator's base over
the same steps, started from the true pose then, with the feet in contact (foot force over `--contact_n`). The joints are UniFP's order in the trace and the feet `gait.FEET`'s (FL, FR, RL, RR): both are put in
Unitree's order (FR, FL, RR, RL) as the dog gives them.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import base_odometry as bo  # noqa: E402

#: UniFP's leg joint order (FL, FR, RL, RR) to Unitree's (FR, FL, RR, RL), joint by joint; the feet likewise.
TO_UNITREE_Q = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
TO_UNITREE_FEET = np.array([1, 0, 3, 2])


def run(path: Path, meta: dict, contact_n: float, start_s: float) -> dict:
    t = dict(np.load(path / "insert_trace.npz"))
    if "base_quat" not in t:
        raise SystemExit(f"{path}: no body fields (run with --insert_trace_body)")
    alive = t["alive"] > 0.5
    n = alive.shape[1]
    moves, errs, errs_xy, used, slid = [], [], [], [], []
    for i in range(n):
        # The last alive step is the one the episode ended on, recorded after the reset placed the robot anew (its base and
        # feet jump), so it is left out.
        steps = int(alive[:, i].sum()) - 1
        s0 = int(round(start_s / 0.02))
        if steps < s0 + 10:
            continue
        sl = slice(s0, steps)
        base = np.stack((t["base_x"][sl, i], t["base_y"][sl, i], t["base_z"][sl, i]), -1).astype(float)
        quat = t["base_quat"][sl, i].astype(float)
        q = t["leg_q"][sl, i][:, TO_UNITREE_Q].astype(float)
        contact = t["foot_fz"][sl, i][:, TO_UNITREE_FEET] > contact_n
        feet = t["feet_w"][sl, i].astype(float)
        slid.append(float(np.abs(feet[:, :, :2] - feet[0, :, :2]).max()) * 1000)
        steps = steps - s0
        odo = bo.LegOdometry(meta)
        odo.reset(base[0], quat[0], q[0], contact[0])
        est = np.stack([base[0]] + [odo.step(quat[k], q[k], contact[k]) for k in range(1, steps)])
        used.append(odo.used)
        moves.append(np.abs(base - base[0]).max(axis=0))
        errs.append(np.abs(est - base).max(axis=0))
        errs_xy.append(float(np.linalg.norm((est - base)[:, :2], axis=-1).max()))
    moves, errs = np.array(moves) * 1000, np.array(errs) * 1000
    q = lambda a: [round(float(v), 2) for v in np.quantile(a, [0.5, 0.9, 1.0])]
    return {"run": path.name, "episodes": len(moves),
            "base_moved_mm_x_y_z_median_p90_max": [q(moves[:, k]) for k in range(3)],
            "odometry_error_mm_x_y_z_median_p90_max": [q(errs[:, k]) for k in range(3)],
            "odometry_error_xy_mm_median_p90_max": q(np.array(errs_xy) * 1000),
            "true_feet_moved_xy_mm_median_p90_max": q(np.array(slid)),
            "from_s": start_s, "feet_used_at_end_median": float(np.median(used))}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--policy", default=str(REPO / "checkpoints/insert_force_ki6_8200.npz"))
    parser.add_argument("--contact_n", type=float, default=10.0)
    parser.add_argument("--start_s", type=float, default=1.0,
                        help="start the odometry this far into each episode: the spawned stance settles in its first "
                             "0.5 s (the rear feet slide back up to ~66 mm), which a dog already standing does not")
    args = parser.parse_args()
    meta = json.loads(str(np.load(args.policy)["meta"]))
    for r in args.runs:
        print(json.dumps(run(Path(r), meta, args.contact_n, args.start_s)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

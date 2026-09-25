#!/usr/bin/env python3
"""How much of UniFP's goal sphere admits an *orientation* as well as a position?

    ./unifp_train/pose_feasibility.py
    ./unifp_train/pose_feasibility.py --orientations 16 --radius 0.35 0.45 0.55

Before an orientation objective is added to the task, the question is what fraction of the goals
the task actually draws can be reached with the hand pointed a given way. A reward for something
the arm cannot do is not a harder task, it is a corrupted one: the goal generator would keep
drawing targets that no action attains, and the term would be noise on the return.

Numpy and the repository's own damped-least-squares solver (`d1_ik`), no simulator, so this runs
on the system Python in a couple of minutes.

Four constraints are compared over the same grid of `interface`'s goal sphere:

  * **position only** — what the task does today, and the redundancy is left free;
  * **position + roll** — one rotational degree of freedom, reported analytically rather than
    solved, for the reason in `roll_is_always_reachable`;
  * **position + level** — `d1_ik`'s two-degree-of-freedom attitude: approach horizontal, no roll,
    heading free;
  * **position + a freely chosen orientation** — the full six-degree-of-freedom pose, sampled with
    uniformly random rotations.

**What this does and does not measure.** The arm alone, with its base fixed: UniFP has the whole
body, and a policy that may pitch and shift the trunk has more to work with, so the full-pose
figure here is a lower bound for the whole-body system. And a damped-least-squares failure is not
a proof of infeasibility — the solver's own staging of the level case lifts it from 24% to 51%
(see `d1_ik.solve`), so these are solver-dependent numbers, not a kinematic certificate.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import d1_ik  # noqa: E402
from position_only.workspace import SOFT_LIMIT_FACTOR, load_urdf  # noqa: E402

#: The goal sphere's centre in the *arm's* base frame. `interface.EE_GOAL_CENTER_OFFSET` puts it
#: 0.49 m above the ground under the robot; the arm is mounted 0.08 m above a base that stands at
#: about 0.30 m, so it sits this far below the arm's own origin.
ARM_MOUNT_HEIGHT_M = 0.38
GOAL_CENTRE_Z_M = 0.49

#: `Joint6`'s travel, from `d1_arm/d1.urdf`.
JOINT6_RANGE_RAD = 2.0 * 2.35


def roll_is_always_reachable(soft: float = SOFT_LIMIT_FACTOR) -> tuple[bool, float]:
    """Whether every commanded roll is reachable from every position solution, and by how much.

    A grasp's jaw axis is an *axis*, not a direction — the two fingers are interchangeable — so a
    commanded jaw direction repeats every 180 degrees. An interval wider than 180 degrees
    therefore contains a representative of any commanded roll, wherever in it the arm happens to
    be sitting. That makes roll the one rotational degree of freedom that costs no workspace,
    which is the whole argument for constraining it and nothing else.
    """
    span = math.degrees(JOINT6_RANGE_RAD * soft)
    return span > 180.0, span


def sphere_point(radius: float, pitch: float, yaw: float) -> np.ndarray:
    centre = np.array([0.0, 0.0, GOAL_CENTRE_Z_M - ARM_MOUNT_HEIGHT_M])
    return centre + np.array([radius * math.cos(pitch) * math.cos(yaw),
                              radius * math.cos(pitch) * math.sin(yaw),
                              radius * math.sin(pitch)])


def random_rotation(rng) -> np.ndarray:
    """A uniformly random rotation matrix, from a normalised Gaussian quaternion."""
    quaternion = rng.normal(size=4)
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--radius", type=float, nargs="+", default=[0.35, 0.45, 0.55])
    parser.add_argument("--pitch_steps", type=int, default=7)
    parser.add_argument("--yaw_steps", type=int, default=7)
    parser.add_argument("--orientations", type=int, default=8,
                        help="Random orientations tried per goal for the full-pose case.")
    parser.add_argument("--max_iterations", type=int, default=300)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output", default=str(ROOT / "logs/unifp_demos"))
    parser.add_argument("--run_name", default="pose_feasibility")
    args = parser.parse_args()

    joints, _links = load_urdf(str(ROOT / "d1_arm/d1.urdf"))
    rng = np.random.default_rng(args.seed)

    pitches = np.linspace(-math.pi / 4, math.pi / 3, args.pitch_steps)
    yaws = np.linspace(-2 * math.pi / 5, 2 * math.pi / 5, args.yaw_steps)
    goals = [(r, float(p), float(y)) for r in args.radius for p in pitches for y in yaws]

    records, position_ok, level_ok, full_ok, full_tried = [], 0, 0, 0, 0
    for radius, pitch, yaw in goals:
        target = sphere_point(radius, pitch, yaw)
        position = d1_ik.solve(joints, target, max_iterations=args.max_iterations).converged
        level = d1_ik.solve(joints, target, level=True, max_iterations=args.max_iterations).converged
        hits = sum(bool(d1_ik.solve(joints, target, random_rotation(rng),
                                    max_iterations=args.max_iterations).converged)
                   for _ in range(args.orientations))
        position_ok += bool(position)
        level_ok += bool(level)
        full_ok += hits
        full_tried += args.orientations
        records.append({"radius": radius, "pitch_deg": math.degrees(pitch),
                        "yaw_deg": math.degrees(yaw), "target_arm_frame_m": target.tolist(),
                        "position": bool(position), "level": bool(level),
                        "orientations_reached": hits, "orientations_tried": args.orientations})

    always, span = roll_is_always_reachable()
    total = len(goals)
    summary = {
        "mode": "pose_feasibility", "status": "finished", "seed": args.seed,
        "goals": total, "orientations_per_goal": args.orientations,
        "radii": args.radius, "pitch_steps": args.pitch_steps, "yaw_steps": args.yaw_steps,
        "position_only_fraction": round(position_ok / total, 4),
        "level_fraction": round(level_ok / total, 4),
        "full_pose_fraction": round(full_ok / full_tried, 4),
        "goals_with_no_orientation": sum(1 for r in records if r["orientations_reached"] == 0),
        "goals_with_every_orientation": sum(
            1 for r in records if r["orientations_reached"] == args.orientations),
        "roll_always_reachable": bool(always),
        "joint6_soft_span_deg": round(span, 1),
        "solver": "damped least squares (d1_ik), fixed base, arm alone",
        "command": " ".join(sys.argv),
    }

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    run_dir = Path(args.output).resolve() / f"{stamp}_ik_seed{args.seed}_{args.run_name}"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "pose_feasibility.json").write_text(json.dumps({"records": records}, indent=2))
    (run_dir / "run.json").write_text(json.dumps({**summary, "records_file": "pose_feasibility.json"},
                                                 indent=2))

    print(f"\n{total} goals over UniFP's trained sphere, {args.orientations} orientations each\n")
    print(f"  position only             {position_ok:4d}/{total}  {100*position_ok/total:5.1f}%")
    print(f"  position + roll (1 DOF)   {position_ok:4d}/{total}  {100*position_ok/total:5.1f}%"
          f"   (analytic: Joint6 spans {span:.0f} deg against a 180 deg period)")
    print(f"  position + level (2 DOF)  {level_ok:4d}/{total}  {100*level_ok/total:5.1f}%")
    print(f"  position + full pose      {full_ok:4d}/{full_tried}  {100*full_ok/full_tried:5.1f}%")
    print(f"\n  goals accepting no orientation tried  : {summary['goals_with_no_orientation']}/{total}")
    print(f"  goals accepting every orientation tried: {summary['goals_with_every_orientation']}/{total}")
    print(f"\nwritten to {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

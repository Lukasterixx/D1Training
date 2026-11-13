#!/usr/bin/env python3
"""Which hand poses the Go2+D1 can really take: the analytic successor to `pose_feasibility.py` (F-099).

    python unifp_train/pose_reach.py --run_name reach_map

F-099 tried damped least squares from one seed on uniformly random orientations, with the arm alone on a fixed base, and
concluded 8.2% of goals admit a freely chosen orientation. Its own scope called that a lower bound. `arm_kin` solves the
pose exactly through the spherical wrist (every elbow seed, both wrist branches), so what is left is the arm and not the
solver, and three questions F-099 could not ask:

  * **Which orientations?** Uniform SO(3) counts hands pointing back at the robot. The jaw centre sits 18.6 cm past the
    wrist, so those need the wrist 18.6 cm further out than the goal. Here each orientation is binned by the angle between
    its approach and the outward line from the shoulder through the goal.
  * **How much does the body add?** A standing dog can pitch, roll, crouch and shift. Each pose is tried with the base at
    a grid of those, feet fixed, within what `66600` already does (`--base_*`).
  * **Is the frame right?** `pose_feasibility.sphere_point` builds goals about the arm's mount and passes them to
    `d1_ik.solve`, whose frame has the mount in it already. `--legacy_frame` keeps that, to size what it cost.
  * **Where is a hand free?** Two jaws repeat every half turn, and a task picks its hand. So the band table takes a hand
    level (and one pitched 45 deg down), any heading within +/-60 deg and every roll, and reports what fraction of them each
    goal admits, arm alone and with the body.

Soft limits (0.9), as F-099. Arm against the body is checked with `d1_ik.clear_of_body` where `--check_body` asks. No
simulator.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from unifp_train import arm_kin  # noqa: E402

GOAL_CENTRE_Z_M = 0.49
BASE_HEIGHT_M = 0.30
#: Joint2's axis at the zero pose, the arm's shoulder, in the base frame: the outward line starts here.
SHOULDER_B = (-0.019, 0.0, 0.0855 + 0.076 + 0.0548)
#: The bins of the approach's angle from outward, degrees.
BINS = (0, 30, 60, 90, 120, 180)


def goal_grid(radii, pitch_steps, yaw_steps):
    pitches = np.linspace(-math.pi / 4, math.pi / 3, pitch_steps)
    yaws = np.linspace(-2 * math.pi / 5, 2 * math.pi / 5, yaw_steps)
    return [(r, float(p), float(y)) for r in radii for p in pitches for y in yaws]


def sphere_to_base(goals, base_height, legacy_frame):
    """(r, pitch, yaw) -> the goal in the Go2 base frame at a level `base_height`, as `_goal_world` puts it."""
    g = torch.as_tensor(goals, dtype=torch.float64)
    r, p, y = g.unbind(-1)
    local = torch.stack((r * p.cos() * y.cos(), r * p.cos() * y.sin(), r * p.sin()), dim=-1)
    centre = torch.tensor([0.0, 0.0, GOAL_CENTRE_Z_M - base_height], dtype=torch.float64)
    if legacy_frame:
        centre = torch.tensor([0.019, 0.0, GOAL_CENTRE_Z_M - base_height - 0.0855], dtype=torch.float64)
    return local + centre


def random_rotations(count, gen):
    q = torch.randn(count, 4, generator=gen, dtype=torch.float64)
    q = q / q.norm(dim=-1, keepdim=True)
    w, x, y, z = q.unbind(-1)
    return torch.stack((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
                        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)), dim=-1).view(count, 3, 3)


def rotation_rpy(roll, pitch):
    """Base attitude as a matrix: pitch about y, roll about x."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = torch.tensor([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=torch.float64)
    ry = torch.tensor([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=torch.float64)
    return ry @ rx


def solvable(kin, position, rotation, seeds, joints=None, base_height=None):
    """(N,) bool: some seed and wrist branch reaches this pose inside the soft limits (and clear of the body if asked)."""
    n = position.shape[0]
    done = torch.zeros(n, dtype=torch.bool)
    wrist_goal = position - rotation @ kin.tool_from_wrist
    lower, upper = kin.soft_lower, kin.soft_upper
    for seed in seeds:
        idx = (~done).nonzero().squeeze(-1)
        if len(idx) == 0:
            break
        q3, miss = kin.solve_wrist(wrist_goal[idx], seed.expand(len(idx), 3), iterations=10)
        ok3 = (miss < 2e-3) & ((q3 >= lower[:3]) & (q3 <= upper[:3])).all(dim=-1)
        if not ok3.any():
            continue
        sub = idx[ok3]
        for q456 in kin.solve_hand(q3[ok3], rotation[sub]):
            ok = ((q456 >= lower[3:]) & (q456 <= upper[3:])).all(dim=-1)
            if joints is not None and ok.any():
                import d1_ik
                q = torch.cat((q3[ok3], q456), dim=-1).numpy()
                clear = np.array([bool(d1_ik.clear_of_body(joints, q[i:i + 1], base_height))
                                  if ok[i] else False for i in range(len(q))])
                ok = ok & torch.as_tensor(clear)
            done[sub[ok]] = True
    return done


def hand_rotations(heading, elevation, roll):
    """Link6 rotations for an approach at `heading` and `elevation` (rad, tensors), turned `roll` about it."""
    approach = torch.stack((elevation.cos() * heading.cos(), elevation.cos() * heading.sin(), elevation.sin()), dim=-1)
    up = torch.tensor([0.0, 0.0, 1.0], dtype=torch.float64).expand_as(approach)
    jaw = torch.cross(up, approach, dim=-1)
    jaw = jaw / jaw.norm(dim=-1, keepdim=True)
    frame = torch.stack((torch.cross(jaw, approach, dim=-1), jaw, approach), dim=-1)
    c, s = roll.cos(), roll.sin()
    turn = torch.zeros(len(roll), 3, 3, dtype=torch.float64)
    turn[:, 0, 0], turn[:, 0, 1], turn[:, 1, 0], turn[:, 1, 1], turn[:, 2, 2] = c, -s, s, c, 1.0
    return frame @ turn


def band_table(kin, seeds, variants, radii, pitches_deg, hand_elevation_deg):
    """Per (goal pitch, radius): the fraction of headings and rolls a hand at this elevation admits, arm alone and with the body."""
    heading, roll = torch.meshgrid(torch.linspace(-60, 60, 9, dtype=torch.float64).deg2rad(),
                                   torch.linspace(0, math.pi, 9, dtype=torch.float64)[:-1], indexing="ij")
    heading, roll = heading.flatten(), roll.flatten()
    elevation = torch.full_like(heading, math.radians(hand_elevation_deg))
    half = torch.diag(torch.tensor([-1.0, -1.0, 1.0], dtype=torch.float64))
    rotation = hand_rotations(heading, elevation, roll)
    rows = []
    for pitch in pitches_deg:
        row = {"goal_pitch_deg": pitch, "arm": [], "body": []}
        for radius in radii:
            p = math.radians(pitch)
            position = torch.stack((radius * math.cos(p) * heading.cos(), radius * math.cos(p) * heading.sin(),
                                    torch.full_like(heading, radius * math.sin(p) + GOAL_CENTRE_Z_M - BASE_HEIGHT_M)), dim=-1)
            arm = solvable(kin, position, rotation, seeds) | solvable(kin, position, rotation @ half, seeds)
            body = arm.clone()
            for pitch_b, roll_b, height, shift in variants:
                idx = (~body).nonzero().squeeze(-1)
                if len(idx) == 0:
                    break
                attitude = rotation_rpy(math.radians(roll_b), math.radians(pitch_b))
                offset = torch.tensor([shift, 0.0, height - BASE_HEIGHT_M], dtype=torch.float64)
                local_p, local_r = (position[idx] - offset) @ attitude, attitude.T @ rotation[idx]
                body[idx[solvable(kin, local_p, local_r, seeds) | solvable(kin, local_p, local_r @ half, seeds)]] = True
            row["arm"].append(round(float(arm.double().mean()), 3))
            row["body"].append(round(float(body.double().mean()), 3))
        rows.append(row)
    return rows


def angle_from_outward(position_b, rotation):
    """The approach's angle from the outward line through the shoulder, degrees."""
    outward = position_b - torch.tensor(SHOULDER_B, dtype=torch.float64)
    outward = outward / outward.norm(dim=-1, keepdim=True)
    approach = rotation[..., :, 2]
    return torch.rad2deg(torch.acos((outward * approach).sum(-1).clamp(-1, 1)))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--radius", type=float, nargs="+", default=[0.35, 0.45, 0.55])
    parser.add_argument("--pitch_steps", type=int, default=7)
    parser.add_argument("--yaw_steps", type=int, default=7)
    parser.add_argument("--orientations", type=int, default=400)
    parser.add_argument("--base_pitch_deg", type=float, nargs="+", default=[-12.0, -6.0, 0.0, 6.0, 12.0])
    parser.add_argument("--base_roll_deg", type=float, nargs="+", default=[-8.0, 0.0, 8.0])
    parser.add_argument("--base_height_m", type=float, nargs="+", default=[0.26, 0.30, 0.33])
    parser.add_argument("--base_shift_m", type=float, nargs="+", default=[-0.04, 0.0, 0.04])
    parser.add_argument("--legacy_frame", action="store_true")
    parser.add_argument("--check_body", action="store_true")
    parser.add_argument("--band_radius", type=float, nargs="+", default=[0.40, 0.45, 0.50, 0.55, 0.60, 0.65])
    parser.add_argument("--band_pitch_deg", type=float, nargs="+", default=[-20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0])
    parser.add_argument("--threads", type=int, default=6, help="CPU threads: this runs beside training.")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output", default=str(ROOT / "logs/unifp_demos"))
    parser.add_argument("--run_name", default="pose_reach")
    args = parser.parse_args()
    torch.set_num_threads(args.threads)

    kin = arm_kin.ArmKinematics(dtype=torch.float64)
    gen = torch.Generator().manual_seed(args.seed)
    goals = goal_grid(args.radius, args.pitch_steps, args.yaw_steps)
    seeds = torch.tensor(list(itertools.product((0.0, -0.8, 0.8, -1.6, 1.6), (0.0, -0.9, 0.9), (0.0, -0.9, 0.9))),
                         dtype=torch.float64)
    per_goal = args.orientations
    position = sphere_to_base(goals, BASE_HEIGHT_M, args.legacy_frame)
    n_goals = len(goals)
    rotations = random_rotations(n_goals * per_goal, gen)
    positions = position.repeat_interleave(per_goal, dim=0)
    goal_of = torch.arange(n_goals).repeat_interleave(per_goal)
    joints = None
    if args.check_body:
        from position_only.workspace import load_urdf
        joints, _ = load_urdf(str(ROOT / "d1_arm/d1.urdf"))

    half_turn = torch.diag(torch.tensor([-1.0, -1.0, 1.0], dtype=torch.float64))
    arm_alone = solvable(kin, positions, rotations, seeds, joints, BASE_HEIGHT_M)
    arm_either_way = arm_alone | solvable(kin, positions, rotations @ half_turn, seeds, joints, BASE_HEIGHT_M)
    print(f"arm alone, level base at {BASE_HEIGHT_M} m: {100 * arm_alone.double().mean():.1f}% of {len(arm_alone)} poses")

    whole = arm_alone.clone()
    variants = list(itertools.product(args.base_pitch_deg, args.base_roll_deg, args.base_height_m, args.base_shift_m))
    for i, (pitch, roll, height, shift) in enumerate(variants):
        idx = (~whole).nonzero().squeeze(-1)
        if len(idx) == 0:
            break
        attitude = rotation_rpy(math.radians(roll), math.radians(pitch))
        # Goals are given level in the yaw frame at 0.30 m; the base now sits elsewhere, so see them from it.
        offset = torch.tensor([shift, 0.0, height - BASE_HEIGHT_M], dtype=torch.float64)
        local_p = (positions[idx] - offset) @ attitude
        local_r = attitude.T @ rotations[idx]
        whole[idx[solvable(kin, local_p, local_r, seeds)]] = True
        if i % 20 == 0:
            print(f"  base variant {i + 1}/{len(variants)}: {100 * whole.double().mean():.1f}%", flush=True)

    bands = {str(elevation): band_table(kin, seeds, variants, args.band_radius, args.band_pitch_deg, elevation)
             for elevation in (0.0, -45.0)}
    angle = angle_from_outward(positions, rotations)
    radius = torch.as_tensor(goals, dtype=torch.float64)[goal_of, 0]
    table = []
    for lo, hi in zip(BINS[:-1], BINS[1:]):
        mask = (angle >= lo) & (angle < hi)
        row = {"approach_from_outward_deg": [lo, hi], "poses": int(mask.sum()),
               "arm_alone": round(float(arm_alone[mask].double().mean()), 4),
               "whole_body": round(float(whole[mask].double().mean()), 4)}
        for r in args.radius:
            m = mask & (radius == r)
            row[f"arm_r{r}"] = round(float(arm_alone[m].double().mean()), 4) if m.any() else None
            row[f"whole_r{r}"] = round(float(whole[m].double().mean()), 4) if m.any() else None
        table.append(row)

    summary = {
        "mode": "pose_reach", "status": "finished", "seed": args.seed, "goals": n_goals, "orientations_per_goal": per_goal,
        "radii": args.radius, "legacy_frame": args.legacy_frame, "check_body": args.check_body,
        "base_variants": len(variants),
        "base_ranges": {"pitch_deg": args.base_pitch_deg, "roll_deg": args.base_roll_deg,
                        "height_m": args.base_height_m, "shift_m": args.base_shift_m},
        "arm_alone_fraction": round(float(arm_alone.double().mean()), 4),
        "arm_alone_jaw_symmetric_fraction": round(float(arm_either_way.double().mean()), 4),
        "goals_with_no_orientation_arm": int((arm_either_way.view(-1, per_goal).sum(1) == 0).sum()),
        "band_radii": args.band_radius, "bands_by_hand_elevation_deg": bands,
        "whole_body_fraction": round(float(whole.double().mean()), 4),
        "by_approach": table,
        "solver": "arm_kin: analytic through the spherical wrist, 45 elbow seeds, both wrist branches, soft limits",
        "command": " ".join(sys.argv),
    }
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    run_dir = Path(args.output).resolve() / f"{stamp}_ik_seed{args.seed}_{args.run_name}"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "run.json").write_text(json.dumps(summary, indent=2))
    print(f"\nuniform SO(3): arm alone {100 * summary['arm_alone_fraction']:.1f}%, whole body {100 * summary['whole_body_fraction']:.1f}%")
    print(f"{'approach from outward':>22} {'poses':>7} {'arm':>6} {'body':>6}   " + "  ".join(f"r={r}: arm/body" for r in args.radius))
    for row in table:
        cells = "  ".join(f"{100 * (row[f'arm_r{r}'] or 0):5.1f}/{100 * (row[f'whole_r{r}'] or 0):5.1f}   " for r in args.radius)
        print(f"{row['approach_from_outward_deg'][0]:>10}-{row['approach_from_outward_deg'][1]:<3} deg {row['poses']:>10} "
              f"{100 * row['arm_alone']:5.1f}% {100 * row['whole_body']:5.1f}%   {cells}")
    for elevation, rows in bands.items():
        print(f"\nhand at {elevation} deg elevation, every roll, heading +/-60: % admitted, arm / with the body; columns r = {args.band_radius}")
        for row in rows:
            print(f"  goal pitch {row['goal_pitch_deg']:+5.0f}: " + "  ".join(
                f"{100 * a:3.0f}/{100 * b:3.0f}" for a, b in zip(row["arm"], row["body"])))
    print(f"written to {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

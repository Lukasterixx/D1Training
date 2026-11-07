#!/usr/bin/env python3
"""The hand against its goal, from a run's `policy_rows.npz`: `python3 demos/combiner_key/robot/hand_error.py <run>/policy_rows.npz [--policy <export>]`.

The controlled point is the CAD pincer tip by forward kinematics from the arm's reports, the dog's tilt from its IMU and its height from the three support feet: calculated, not
measured on the arm, and blind to any arm calibration error. The goal is the one the policy was shown on that row, in the frame's commands, so a sliding goal is scored as it slides.

`d1_ik.tool_pose` is in the Go2 base frame, with the arm's mount already in it. Adding the mount again put the hand 8.4 cm too high on 2026-11-07 (`tests/test_reach_arm.py` holds
this to a number the simulator recorded). Not numpy-only: it needs the URDF, so it runs on the PC, not the dog.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import d1_ik  # noqa: E402
from demos.combiner_key.robot import leg_policy  # noqa: E402
from position_only.workspace import load_urdf  # noqa: E402

FOOT_RADIUS_M = 0.022                               # press_layer.FOOT_RADIUS_M
GOAL_CENTRE_M = (0.0, 0.0, 0.49)                    # interface.EE_GOAL_CENTER_OFFSET, over the ground under the base
LEGS, SUPPORT = ("FR", "FL", "RR", "RL"), (1, 2, 3)
FRAME_RADIUS = 64                                   # the frame's goal: radius, pitch, yaw, each scaled by commands_scale[3:6]


def tilt_rotation(roll: float, pitch: float) -> np.ndarray:
    """The base in the yaw frame, roll then pitch; pitch negative is nose up."""
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def goal_point(sphere) -> np.ndarray:
    """UniFP's (radius, pitch, yaw) to the yaw frame, from the ground under the base."""
    r, pitch, yaw = (float(v) for v in sphere)
    return np.array(GOAL_CENTRE_M) + r * np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)])


def hand_point(joints, arm_q, base_m: float, roll: float, pitch: float) -> np.ndarray:
    """The tip in the yaw frame, from the ground under the base. The mount is in `tool_pose`."""
    return np.array([0.0, 0.0, base_m]) + tilt_rotation(roll, pitch) @ d1_ik.tool_pose(joints, arm_q)[0]


def base_height(q_unitree, roll: float, pitch: float, meta: dict) -> float:
    rot = tilt_rotation(roll, pitch)
    z = [(rot @ leg_policy.foot_local(leg, np.asarray(q_unitree)[3 * k:3 * k + 3], meta))[2] for k, leg in enumerate(LEGS)]
    return FOOT_RADIUS_M - float(np.mean([z[k] for k in SUPPORT]))


def offsets(rows, meta: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(t (N,), goal spheres (N, 3), hand minus goal (N, 3)) in metres, for the rows that had a goal."""
    joints = load_urdf()[0]
    scale = np.asarray(meta["commands_scale"], dtype=float)[3:6]
    t, frame = np.asarray(rows["t"], dtype=float), np.asarray(rows["frame"], dtype=float)
    sphere = frame[:, FRAME_RADIUS:FRAME_RADIUS + 3] / scale
    keep = np.abs(sphere[:, 0]) > 1e-6                  # the reset's frame carries no goal
    out = []
    for i in np.nonzero(keep)[0]:
        roll, pitch = rows["rpy"][i, :2]
        hand = hand_point(joints, rows["arm_obs"][i], base_height(rows["q"][i], roll, pitch, meta), roll, pitch)
        out.append(hand - goal_point(sphere[i]))
    return t[keep] - t[0], sphere[keep], np.array(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("rows")
    parser.add_argument("--policy", default=str(ROOT / "checkpoints/reach_force_sticky_model_66600.npz"))
    parser.add_argument("--every_s", type=float, default=1.0)
    args = parser.parse_args()
    meta = leg_policy.LegPolicy(args.policy).meta
    rows = dict(np.load(args.rows))
    t, sphere, off = offsets(rows, meta)
    err = np.linalg.norm(off, axis=1) * 100
    print("    t | goal r, pitch, yaw      | off x, y, z cm        | cm")
    for x in np.arange(0.0, t[-1] + 1e-6, args.every_s):
        i = int(np.argmin(np.abs(t - x)))
        print(f"{t[i]:5.2f} | {sphere[i, 0]:.2f} {math.degrees(sphere[i, 1]):+6.1f} {math.degrees(sphere[i, 2]):+6.1f} | {off[i, 0] * 100:+6.1f} {off[i, 1] * 100:+6.1f} {off[i, 2] * 100:+6.1f} | {err[i]:5.1f}")
    # A goal held for at least a second: what the hand settled to, its last half.
    moved = np.abs(np.diff(sphere, axis=0)).max(axis=1) > 1e-6
    start = 0
    for edge in list(np.nonzero(moved)[0] + 1) + [len(t)]:
        if t[edge - 1] - t[start] >= 1.0:
            half = slice(start + (edge - start) // 2, edge)
            print(f"held to {t[edge - 1]:5.2f} s at r {sphere[start, 0]:.2f} pitch {math.degrees(sphere[start, 1]):+.0f} yaw {math.degrees(sphere[start, 2]):+.0f}: "
                  f"{err[half].mean():.1f} cm, off {np.round(off[half].mean(axis=0) * 100, 1).tolist()}")
        start = edge
    return 0


if __name__ == "__main__":
    sys.exit(main())

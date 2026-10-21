#!/usr/bin/env python3
"""Stances like the Go2 sport mode's classic stand, for starting the key policy's frozen episodes from (2026-10-21).

Lukas measured the classic gait standing still with the feet almost under the shoulders, 41 cm front to back and 28 cm
side to side. This solves the 12 leg angles that put the feet's sphere centres there, level, under a given base height,
and writes one json per height in `leg_env`'s spawn-state format (`--spawn_states <json> --eval_spawn_states`). The base
height of the classic stand is not measured here (`sportmodestate.body_height` on the dog has it), so several are
written. CPU only; the kinematics are `description/go2_d1.urdf`'s (checked against `leg_cfg.NOMINAL_FEET_B`).

    python demos/combiner_key/classic_stance.py --heights 0.28 0.30 0.32
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from unifp_train import leg_cfg  # noqa: E402

#: UniFP's leg order: FL, FR, RL, RR, each (hip, thigh, calf).
LEGS = ("FL", "FR", "RL", "RR")
HIP_X, HIP_Y, THIGH_Y, LINK_M = 0.1934, 0.0465, 0.0955, 0.213
FOOT_RADIUS_M, FOOT_SPHERE_X = 0.022, -0.002


def foot(leg: str, q: np.ndarray) -> np.ndarray:
    """The foot sphere's centre in the base frame for one leg's (hip, thigh, calf)."""
    sx = 1.0 if leg[0] == "F" else -1.0
    sy = 1.0 if leg[1] == "L" else -1.0
    hip, thigh, calf = q
    # In the thigh's frame (after the hip's roll), the leg swings in its x-z plane.
    x = -LINK_M * math.sin(thigh) - LINK_M * math.sin(thigh + calf) + FOOT_SPHERE_X * math.cos(thigh + calf)
    z = -LINK_M * math.cos(thigh) - LINK_M * math.cos(thigh + calf) + FOOT_SPHERE_X * math.sin(thigh + calf)
    y = sy * THIGH_Y
    # The hip's roll about x.
    c, s = math.cos(hip), math.sin(hip)
    return np.array([sx * HIP_X + x, sy * HIP_Y + c * y - s * z, s * y + c * z])


def solve(leg: str, target: np.ndarray, start: np.ndarray) -> np.ndarray:
    fit = least_squares(lambda q: foot(leg, q) - target, start, xtol=1e-12, ftol=1e-12)
    if np.linalg.norm(fit.fun) > 1e-6:
        raise ValueError(f"{leg}: no solution to {target} ({np.linalg.norm(fit.fun):.2e} m off)")
    return fit.x


def stance(front_back: float, left_right: float, height: float) -> dict:
    legs = []
    settled = np.array(leg_cfg.SETTLED_LEG_POS).reshape(4, 3)
    for k, leg in enumerate(LEGS):
        sx = 1.0 if leg[0] == "F" else -1.0
        sy = 1.0 if leg[1] == "L" else -1.0
        target = np.array([sx * front_back / 2, sy * left_right / 2, -(height - FOOT_RADIUS_M)])
        legs += solve(leg, target, settled[k]).tolist()
    return {"legs": [round(v, 5) for v in legs], "base_z": height, "roll": 0.0, "pitch": 0.0,
            "feet_b_m": {"front_back": front_back, "left_right": left_right}}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--front_back", type=float, default=0.41)
    parser.add_argument("--left_right", type=float, default=0.28)
    parser.add_argument("--heights", type=float, nargs="+", default=[0.28, 0.30, 0.32])
    parser.add_argument("--out_dir", default=str(Path(__file__).resolve().parent / "assets"))
    args = parser.parse_args()

    # The kinematics against the trained stance's recorded foot places (`leg_cfg.NOMINAL_FEET_B`, in the base's yaw frame).
    settled = np.array(leg_cfg.SETTLED_LEG_POS).reshape(4, 3)
    for k, leg in enumerate(LEGS):
        p = foot(leg, settled[k])
        print(f"trained stance {leg}: foot at x {p[0]:+.4f} y {p[1]:+.4f} z {p[2]:+.4f} m (recorded x {leg_cfg.NOMINAL_FEET_B[k][0]:+.4f} "
              f"y {leg_cfg.NOMINAL_FEET_B[k][1]:+.4f}; base {leg_cfg.SETTLED_BASE_HEIGHT_M} m up)")
    for h in args.heights:
        s = stance(args.front_back, args.left_right, h)
        path = Path(args.out_dir) / f"classic_stance_{round(h * 100)}cm.json"
        path.write_text(json.dumps([s], indent=1))
        q = np.array(s["legs"]).reshape(4, 3)
        moved = [math.hypot(foot(leg, q[k])[0] - leg_cfg.NOMINAL_FEET_B[k][0], foot(leg, q[k])[1] - leg_cfg.NOMINAL_FEET_B[k][1])
                 for k, leg in enumerate(LEGS)]
        print(f"{path.name}: hips {np.round(q[:, 0], 3).tolist()}, thighs {np.round(q[:, 1], 3).tolist()}, calves "
              f"{np.round(q[:, 2], 3).tolist()}; feet {', '.join(f'{100 * m:.1f}' for m in moved)} cm from the trained places")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

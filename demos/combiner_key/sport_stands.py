#!/usr/bin/env python3
"""A library of sport-mode stands to start the key policy's episodes from (`--spawn_states`), 2026-10-21.

The real dog hands over from Unitree's sport mode, standing still in its classic stand. Lukas taped it at 41 x 28 cm
(front-back x left-right); the shadow run's encoders put it at 38.7 x 31.7 cm under a 0.32-0.33 m base, the rear-left
foot 3-4 cm further out than the others. This writes stands over those spreads -- base heights 0.30-0.34 m, both
spacings, the rear-left foot as the others or 3.5 cm out -- level and at rest, each solved by `classic_stance.solve`
(the URDF's legs), then the walking policy's 32 recorded stops (`assets/walker_stances.json`).

    python demos/combiner_key/sport_stands.py --out demos/combiner_key/assets/sport_stands_20261021.json
"""
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import numpy as np

from demos.combiner_key import classic_stance
from unifp_train import leg_cfg

HERE = Path(__file__).resolve().parent
HEIGHTS_M = (0.30, 0.31, 0.32, 0.33, 0.34)
SPACINGS_M = ((0.41, 0.28), (0.387, 0.317))       # the tape, the encoders
RL_OUT_M = (0.0, 0.035)


def stand(front_back: float, left_right: float, height: float, rl_out: float) -> dict:
    settled = np.array(leg_cfg.SETTLED_LEG_POS).reshape(4, 3)
    legs = []
    for k, leg in enumerate(classic_stance.LEGS):
        sx = 1.0 if leg[0] == "F" else -1.0
        sy = 1.0 if leg[1] == "L" else -1.0
        y = sy * left_right / 2 + (rl_out if leg == "RL" else 0.0)
        target = np.array([sx * front_back / 2, y, -(height - classic_stance.FOOT_RADIUS_M)])
        legs += classic_stance.solve(leg, target, settled[k]).tolist()
    return {"legs": [round(v, 5) for v in legs], "base_z": height, "roll": 0.0, "pitch": 0.0,
            "source": f"sport stand {front_back:.3f} x {left_right:.3f} m, base {height:.2f} m, RL +{rl_out:.3f} m"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--sport_weight", type=int, default=2,
                        help="how many times each sport stand is listed (the library is drawn from uniformly)")
    args = parser.parse_args()
    sport = [stand(fb, lr, h, rl) for h, (fb, lr), rl in itertools.product(HEIGHTS_M, SPACINGS_M, RL_OUT_M)]
    walker = json.loads((HERE / "assets" / "walker_stances.json").read_text())
    for s in walker:
        s.setdefault("source", "P2Dingo walking policy stop (F-167)")
    library = sport * args.sport_weight + walker
    Path(args.out).write_text(json.dumps(library, indent=1))
    print(f"wrote {args.out}: {len(sport)} sport stands x{args.sport_weight} + {len(walker)} walker stops = {len(library)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

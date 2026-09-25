"""How much the D1 alone holds in each direction the mechanism task asks for: up, down, back, away, sideways.

The goal-commanded policies open pulls toward the robot and fail at lifting (7% of training lifts opened,
0 of 120 lids; week 2 log, 2026-09-25). Is lifting weak because the policy never learned it, or because the
arm has little to give upward? The same static model as F-101 (`force_transmission_study.py`: published
joint limits, the weld's masses, gravity + J^T F, base level), over the same reachable postures in UniFP's
goal shell, asked along each direction: the median and the best, and which joint binds. Arm only: the legs
can add to a push or a pull by leaning, and to a lift only by standing taller.

    PYTHONPATH=. python3 results/week_02/figures/direction_capacity.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import force_transmission_study as study  # noqa: E402

DIRECTIONS = {
    "back toward the robot (-x)": (-1.0, 0.0, 0.0),
    "away from the robot (+x)": (1.0, 0.0, 0.0),
    "up (+z)": (0.0, 0.0, 1.0),
    "down (-z)": (0.0, 0.0, -1.0),
    "sideways (+y)": (0.0, 1.0, 0.0),
}


def main() -> None:
    rng = np.random.default_rng(7)
    q = study.sample_configs(study.JOINTS, 300_000, rng)
    q = q[study.clear_of_body(study.JOINTS, q, 0.30)]
    tip, jac, hold = study.kinematics(q, study.GRAVITY_W)
    centre = np.array([0.0, 0.0, 0.49 - 0.30])
    radius = np.linalg.norm(tip - centre, axis=1)
    keep = (radius > 0.30) & (radius < 0.58) & (tip[:, 0] > 0.0)
    jac, hold = jac[keep], hold[keep]
    out = {"postures": int(keep.sum()), "directions": {}}
    for name, u in DIRECTIONS.items():
        force, joint = study.capacity(jac, hold, np.array(u))
        binding = np.bincount(joint[force > 0], minlength=6)
        out["directions"][name] = {
            **study.pct(force), "p99": round(float(np.percentile(force, 99)), 2),
            "max": round(float(force.max()), 2),
            "binding_joint_share": {f"Joint{j + 1}": round(float(binding[j] / max(1, binding.sum())), 3) for j in range(6)},
        }
        print(f"{name:28s} p10 {out['directions'][name]['p10']:6.1f}  p50 {out['directions'][name]['p50']:6.1f}  "
              f"p90 {out['directions'][name]['p90']:6.1f}  p99 {out['directions'][name]['p99']:6.1f} N   "
              f"binds most: Joint{int(binding.argmax()) + 1}", flush=True)
    (HERE / "direction_capacity.json").write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    main()

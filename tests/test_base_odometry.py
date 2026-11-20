"""`demos/combiner_key/robot/base_odometry.py`: the base from the legs, on made-up stances (no simulator)."""
import json
import math
import unittest
from pathlib import Path

import numpy as np

from demos.combiner_key.robot import base_odometry as bo

ROOT = Path(__file__).resolve().parents[1]
META = json.loads(str(np.load(ROOT / "checkpoints/insert_force_ki6_8200.npz")["meta"]))
#: A standing pose (Unitree's order): hip 0, thigh 0.8, calf -1.5.
STAND = np.tile([0.0, 0.8, -1.5], 4)


def _quat(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr, cp, sp, cy, sy = (f(a / 2) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


def _legs_for(base, quat, feet_w):
    """Joint angles that put each foot at `feet_w` (4, 3) from this base pose: Newton on the leg's own kinematics."""
    rot = bo.quat_matrix(quat)
    q = STAND.copy()
    for k, leg in enumerate(bo.LEG_ORDER):
        want = rot.T @ (feet_w[k] - base)
        qk = q[3 * k:3 * k + 3].copy()
        for _ in range(30):
            f = bo.leg_policy.foot_local(leg, qk, META)
            jac = np.stack([(bo.leg_policy.foot_local(leg, qk + 1e-6 * e, META) - f) / 1e-6 for e in np.eye(3)], 1)
            qk = qk + np.linalg.solve(jac, want - f)
        q[3 * k:3 * k + 3] = qk
    return q


class LegOdometryTests(unittest.TestCase):
    def setUp(self):
        self.base0 = np.array([0.0, 0.0, 0.30])
        self.quat0 = _quat()
        self.feet = self.base0 + bo.feet_in_base(STAND, META) @ bo.quat_matrix(self.quat0).T

    def test_a_base_that_moves_over_planted_feet_is_followed(self):
        odo = bo.LegOdometry(META)
        odo.reset(self.base0, self.quat0, STAND, np.ones(4, bool))
        # Leaning 2 cm forward and 1 cm down, pitched 3 deg nose down, yawed 1 deg: the feet stay put.
        for frac in np.linspace(0.0, 1.0, 20):
            base = self.base0 + frac * np.array([0.02, -0.004, -0.01])
            quat = _quat(pitch=math.radians(3.0) * frac, yaw=math.radians(1.0) * frac)
            est = odo.step(quat, _legs_for(base, quat, self.feet), np.ones(4, bool))
        self.assertLess(float(np.abs(est - base).max()), 1e-6)

    def test_a_lifted_foot_drops_out_and_is_anchored_where_it_lands(self):
        odo = bo.LegOdometry(META)
        odo.reset(self.base0, self.quat0, STAND, np.ones(4, bool))
        quat = self.quat0
        feet = self.feet.copy()
        up = np.array([True, False, True, True])            # FL up (Unitree's order: FR, FL, RR, RL)
        moved = feet.copy()
        moved[1] = moved[1] + np.array([0.0, 0.0, 0.05])     # wherever FL goes in the air, it is not used
        base = self.base0 + np.array([0.005, 0.0, 0.0])
        est = odo.step(quat, _legs_for(base, quat, moved), up)
        self.assertEqual(odo.used, 3)
        self.assertLess(float(np.abs(est - base).max()), 1e-6)
        moved[1] = feet[1] + np.array([0.03, 0.0, 0.0])      # FL comes down 3 cm further on
        est = odo.step(quat, _legs_for(base, quat, moved), np.ones(4, bool))
        base2 = base + np.array([0.004, 0.0, 0.0])
        est = odo.step(quat, _legs_for(base2, quat, moved), np.ones(4, bool))
        self.assertEqual(odo.used, 4)
        self.assertLess(float(np.abs(est - base2).max()), 1e-6)

    def test_no_foot_holds_the_last_estimate(self):
        odo = bo.LegOdometry(META)
        odo.reset(self.base0, self.quat0, STAND, np.ones(4, bool))
        est = odo.step(self.quat0, STAND, np.zeros(4, bool))
        self.assertEqual(odo.used, 0)
        np.testing.assert_allclose(est, self.base0)


if __name__ == "__main__":
    unittest.main()

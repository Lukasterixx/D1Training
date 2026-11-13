"""The dog runner's stops as the key-hold evaluation reads them (`unifp_train/leg_eval.py`, `STOP_FIELDS`): the arm's lead
from 1 s on, the legs' targets in the first second and after, and the count of episodes inside every bound. No simulator."""
from __future__ import annotations

import math
import unittest

import numpy as np

from unifp_train import leg_eval


def _trace(steps: int = 200, ended_at: int = 150) -> tuple[dict, np.ndarray]:
    col = {k: np.zeros(steps) for k in leg_eval.STOP_FIELDS}
    alive = np.ones(steps, bool)
    alive[ended_at:] = False
    col["lead_j15"][:] = math.radians(5.0)
    col["lead_j4"][:] = math.radians(3.0)
    col["lead_j6"][:] = math.radians(12.0)
    col["leg_target_fr"][:] = 0.3
    col["leg_target_sup"][:] = 0.2
    return col, alive


class StopExtrasTest(unittest.TestCase):
    def test_the_lead_counts_from_one_second_and_only_while_alive(self):
        col, alive = _trace()
        one_s = int(round(1.0 / leg_eval.interface.POLICY_DT))
        col["lead_j15"][one_s - 1] = math.radians(40.0)       # inside the first second: the runner's grace
        col["lead_j15"][one_s + 5] = math.radians(18.0)
        col["lead_j15"][160] = math.radians(90.0)             # after the episode ended
        e = leg_eval._stop_extras(col, alive)
        self.assertAlmostEqual(e["lead_deg_after_1s"], 18.0, places=3)
        self.assertAlmostEqual(e["lead_j4_deg_after_1s"], 3.0, places=3)
        self.assertAlmostEqual(e["lead_j6_deg_after_1s"], 12.0, places=3)

    def test_the_legs_first_second_and_after_are_apart_and_nan_is_skipped(self):
        col, alive = _trace()
        col["leg_target_fr"][10] = 1.0
        col["leg_target_fr"][100] = 1.1
        col["leg_target_sup"][60] = np.nan
        e = leg_eval._stop_extras(col, alive)
        self.assertAlmostEqual(e["leg_fr_rad_first_s"], 1.0)
        self.assertAlmostEqual(e["leg_fr_rad_after"], 1.1)
        self.assertAlmostEqual(e["leg_sup_rad_after"], 0.2)


class StopSummaryTest(unittest.TestCase):
    def test_counts_against_each_bound_and_within_all(self):
        base = {"lead_deg_after_1s": 5.0, "lead_j4_deg_after_1s": 3.0, "lead_j6_deg_after_1s": 12.0,
                "leg_fr_rad_first_s": 0.5, "leg_fr_rad_after": 0.5, "leg_sup_rad_first_s": 0.2, "leg_sup_rad_after": 0.2,
                "max_tilt_deg": 10.0, "success": True, "lost": False, "foot_down": True}
        episodes = [dict(base),
                    dict(base, lead_deg_after_1s=18.0),                   # past 15, not 25
                    dict(base, leg_fr_rad_after=1.1),                     # the lifting leg past 0.9
                    dict(base, max_tilt_deg=19.0),                        # the tilt stop
                    dict(base, lost=True)]                                # within every bound, the key lost
        s = leg_eval._stop_summary(episodes)
        over = s["episodes_over"]
        self.assertEqual(over["lead_15_deg"], 1)
        self.assertEqual(over["lead_25_deg"], 0)
        self.assertEqual(over["leg_fr_after_0.9_rad"], 1)
        self.assertEqual(over["leg_sup_after_0.9_rad"], 0)
        self.assertEqual(over["tilt_15_deg"], 1)
        self.assertEqual(s["episodes_within_all"], 2)
        self.assertEqual(s["all_four_within_all"], 1)
        self.assertEqual(s["max_tilt_deg"]["max"], 19.0)


if __name__ == "__main__":
    unittest.main()

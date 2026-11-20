"""`check_armwatch.py`: the dog runner's `ArmWatch` over a made-up insertion trace (no simulator)."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from demos.combiner_key.robot import check_armwatch


def trace(lead_deg: float, lead_from_s: float, lead_for_s: float, steps: int = 200, ramp: int = 1) -> dict:
    n = 1
    t = {k: np.zeros((steps, n), np.float32) for k in ("alive", "phase", "normal")}
    t["alive"][:] = 1.0
    t["phase"][:] = 2                                   # keylock.SEAT
    t["normal"][:] = 5.0                                # pressing the key the whole time
    goal = np.zeros((steps, n, 6), np.float32)
    rep = np.zeros((steps, n, 6), np.float32)
    k0, k1 = int(lead_from_s / 0.02), int((lead_from_s + lead_for_s) / 0.02)
    goal[k0:k1, :, 1] = np.radians(lead_deg) * np.minimum(1.0, (np.arange(k0, k1) - k0 + 1) / ramp)[:, None]  # J2 ahead of its report
    t["arm_goal"], t["arm_reported"] = goal, rep
    return t


class ArmWatchOverATrace(unittest.TestCase):
    def check(self, t: dict) -> dict:
        with tempfile.TemporaryDirectory() as d:
            np.savez(Path(d) / "insert_trace.npz", **t)
            (Path(d) / "insert_eval_episodes.json").write_text(json.dumps({"episodes": [{"success": True, "max_tilt_deg": 3.0}]}))
            return check_armwatch.run(Path(d), contact_n=1.0)

    def test_a_small_lead_in_contact_goes_on(self):
        r = self.check(trace(10.0, 1.5, 2.0))
        self.assertEqual(r["would_stop"], 0)
        self.assertAlmostEqual(r["lead_contact_max_deg"]["max"], 10.0, places=3)
        self.assertEqual(r["lead_contact_joint"], 2)

    def test_a_lead_held_past_the_limit_stops_in_contact(self):
        r = self.check(trace(30.0, 1.5, 1.0, ramp=5))        # 6 deg a tick in: no step stop
        self.assertEqual(r["would_stop"], 1)
        self.assertEqual(r["stops_by"], {"lead held, seat, in contact": 1})

    def test_a_short_lead_past_the_limit_goes_on(self):
        r = self.check(trace(30.0, 1.5, 0.3, ramp=5))        # under LEAD_HOLD_S, and back out in one 30 deg tick: a step stop
        self.assertEqual(r["stops_by"], {"step, seat, in contact": 1})

    def test_a_jump_is_a_step_stop(self):
        r = self.check(trace(25.0, 1.5, 0.2))
        self.assertEqual(r["stops_by"], {"step, seat, in contact": 1})

    def test_inside_the_grace_nothing_stops(self):
        r = self.check(trace(70.0, 0.1, 0.5))
        self.assertEqual(r["would_stop"], 0)


if __name__ == "__main__":
    unittest.main()

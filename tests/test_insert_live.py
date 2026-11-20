"""`insert_live.py`: the learned insertion's run on the dog, against the fake robot and the fake arm link (no ROS, no
simulator). What reaches the arm and the legs, in what order, and what is refused."""
import math
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

import d1_hardware
from demos.combiner_key.robot import insert_live, leg_policy, press_live, reach_arm
from tests.test_press_live import Clock, FakeOperator, FakeRobot, args
from tests.test_press_live import REACH_POLICY
from tests.test_reach_live import FakeLink

#: The dog's insertion (F-303, 2026-11-19); `INSERT_LIVE_POLICY` runs these on another export (`ki6_force_8200`'s passes too).
POLICY = Path(os.environ.get("INSERT_LIVE_POLICY", REACH_POLICY.parent / "insert_force_ki7_s62_9200.npz"))
FOLD = reach_arm.from_servo_deg(d1_hardware.REST_SERVO_DEG)
#: The middle of what the policy trained on: the mouth 4.5 mm proud of a door 0.51 m ahead, 0.11 m to the right, 0.45 m up.
LOCK = (0.5055, -0.11, 0.45, 0.0)


@unittest.skipUnless(POLICY.exists(), "no exported insertion policy")
class Rig(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.saved = press_live.time, insert_live.time
        press_live.time = insert_live.time = self.clock
        self.meta = leg_policy.LegPolicy(POLICY).meta

    def tearDown(self):
        press_live.time, insert_live.time = self.saved
        self.tmp.cleanup()

    def make(self, link=None, *, live=True, send=True, **kw):
        base = dict(policy=str(POLICY), live=live, presses=0, policy_s=3.0, run_dir=self.tmp.name, lock=LOCK, air_touch_mm=15.0,
                    lock_outside_training=False, insert_arm_send=send, max_press_n=20.0, contact_raw=None, back_out_m=0.04,
                    reach_max_leg_off_rad=0.9, reach_soft_tilt_deg=12.0, start_on_blend=True, start_tol_rad=1.0,
                    start_pitch_deg=(-90.0, 90.0))
        base.update(kw)
        self.robot = FakeRobot(self.meta)
        return insert_live.InsertRun(args(**base), self.robot if live else None, FakeOperator(), arm_link=link)

    def drive(self, run, link=None, *, press_a=True, steps=8000):
        for k in range(steps):
            self.robot.step()
            run.on_state(self.robot.msg())
            if press_a and run.key_phase == "hand_in" and not run.go:
                run.go = True
            if link is not None:
                link.step(0.02)
            if k % 30 == 0 and run.mode in ("preflight", "damping"):
                run.operator.lines.append("live" if run.confirming else "")
            run.tick()
            self.clock.now += 0.02
            if run.mode == "done":
                return k
        self.fail(f"never ended: mode {run.mode}, key {run.key_phase}, arm {run.arm_phase}")


class Dry(Rig):
    def test_the_policy_runs_on_the_virtual_arm_and_nothing_is_sent(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, live=False, send=False)
        self.drive(run, link)
        self.assertIsNone(run.abort_reason)
        self.assertEqual(run.end_reason, "--policy_s 3.0 s")
        self.assertEqual([m for m in link.lines if m["type"] != "keepalive"], [])
        self.assertEqual(run.layer_phases[0], "done")               # the reset's frame; the layer starts after START_STEPS
        self.assertIn("approach", run.layer_phases)
        self.assertGreater(run.policy_ticks, 100)
        self.assertLess(float(np.abs(run.virtual.q - run.q_start).max()), 0.5)

    def test_a_lock_outside_training_runs_dry_only_when_asked(self):
        far = (0.60, -0.11, 0.45, 0.0)
        with self.assertRaises(SystemExit):
            self.make(None, live=False, send=False, lock=far)
        run = self.make(None, live=False, send=False, lock=far, lock_outside_training=True)
        self.assertIsNotNone(run)


class Live(Rig):
    def test_the_whole_run_in_order(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        self.drive(run, link)
        self.assertIsNone(run.abort_reason)
        self.assertEqual(run.end_reason, "--policy_s 3.0 s")        # no face here: the layer never seats
        events = [e["event"] for e in run.events]
        order = ["start solved", "the jaws closing on the key", "arm: at the insertion's start",
                 "the arm's goals are going to the link", "the jaws open: the key stays where it is (in the lock if the layer seated it)",
                 "the hand is out along the axis", "arm: folded"]
        idx = [next(i for i, e in enumerate(events) if e.startswith(o)) for o in order]
        self.assertEqual(idx, sorted(idx), events)
        kinds = [k for k in link.kinds() if k != "keepalive"]
        first_follow = kinds.index("follow")
        # Out to the hand-in pose, to the start, and once or twice to the start from the settled stand (it sinks 3 cm).
        self.assertEqual(set(kinds[:first_follow]), {"path"})
        self.assertIn(first_follow, (3, 4))
        self.assertEqual(kinds[-3:], ["hold", "path", "path"])      # held, backed out, folded
        self.assertEqual(run.arm_phase, "folded_again")
        self.assertLess(np.degrees(np.abs(link.q - reach_arm.from_servo_deg(reach_arm.FOLD_SERVO_DEG)).max()), 3.5)

    def test_no_key_no_policy(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        self.drive(run, link, press_a=False, steps=12000)
        self.assertIn("no key", run.abort_reason or "")
        self.assertEqual(run.policy_ticks, 0)
        self.assertNotIn("follow", link.kinds())
        self.assertEqual(run.arm_phase, "folded_again")             # folded and lay down; not dropped in damping

    def test_a_press_past_the_cap_stops_and_folds(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, max_press_n=-1.0)                      # any press at all is past it
        self.drive(run, link)
        self.assertIn("press", run.abort_reason or "")
        self.assertEqual(run.arm_phase, "folded_again")

    def test_live_is_the_whole_body_or_nothing(self):
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock, q=FOLD), live=True, send=False)
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock, q=FOLD), live=False, send=True)
        with self.assertRaises(SystemExit):
            self.make(None, live=True, send=True)                    # no arm link

    def test_an_observing_link_is_refused(self):
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock, q=FOLD, observe=True))


class Frames(unittest.TestCase):
    def test_the_lock_axis_and_bar(self):
        lk = insert_live.lock_estimate(0.5, -0.1, 0.45, 0.0)
        np.testing.assert_allclose(lk.axis, [1.0, 0.0, 0.0])
        np.testing.assert_allclose(lk.u, [0.0, -1.0, 0.0])            # the bar to the robot's right
        lk = insert_live.lock_estimate(0.5, -0.1, 0.45, 90.0)
        np.testing.assert_allclose(lk.axis, [0.0, 1.0, 0.0], atol=1e-12)

    def test_the_heading_comes_off(self):
        yaw = math.radians(37.0)
        q = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
        np.testing.assert_allclose(insert_live.without_heading(q, yaw), [1.0, 0.0, 0.0, 0.0], atol=1e-12)

    def test_an_air_touch(self):
        self.assertTrue(insert_live.air_touch(-0.070, 15.0))      # F-298's weak shoulder: 70 mm short
        self.assertFalse(insert_live.air_touch(-0.006, 15.0))     # inside the estimate's error
        self.assertFalse(insert_live.air_touch(0.004, 15.0))      # past the face: in the keyway

    def test_the_trained_range(self):
        self.assertEqual(insert_live.outside_training(*LOCK), [])
        self.assertEqual(len(insert_live.outside_training(0.5055, -0.11, 0.40, 0.0)), 1)    # the box not raised: 0.4175


if __name__ == "__main__":
    unittest.main()

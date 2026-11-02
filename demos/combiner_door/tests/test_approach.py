"""The approach (`robot/approach.py`): a box the look sees out of a task's range walked to, then looked at again. The
key's dry walk (the classic walk's carry, the odometry counting 87% of the walk) from boxes far, near, to the side and
turned; from where it stops, a new look must be in range for the door pull, the key and the insertion. numpy only."""
from __future__ import annotations

import math
import unittest

import numpy as np

import d1_hardware
from demos.combiner_door.robot import approach, plan
from demos.combiner_door.tests.test_robot import box_odom, look
from demos.combiner_key.robot import insert_plan, key_plan
from demos.combiner_key.tests.test_key_robot import ODOM_SCALE, walk_classic

FIXES = {"door": plan.make_fix, "key": key_plan.make_key_fix, "insert": insert_plan.make_insert_fix}


class ApproachTests(unittest.TestCase):
    def walk_to_stage(self, task, box, start=(0.0, 0.0, 0.0)):
        base = plan.base_pose(*start)
        fix = FIXES[task](base, *look(base, box))
        self.assertIsNotNone(fix)
        staging = approach.staging_pose(task, fix.box_odom)
        target = approach.odometry_target(plan.planar(base), staging)
        attempt = approach.ApproachAttempt(approach.staging_fix(fix, target), "walk")
        true_base, moving = base, (0.0, 0.0, 0.0)
        arm = np.asarray(d1_hardware.REST_SERVO_DEG, dtype=float)
        for step in range(900):
            command = attempt.step(plan.Observation(0.1 * step, base, 0.0, arm, 0.0))
            self.assertEqual(command.abort, "", command.phase)
            self.assertIsNone(command.arm_servo_deg)
            if command.finished:
                break
            before = plan.planar(true_base)
            true_base, moving = walk_classic(true_base, command.base if command.walking else None, moving)
            after = plan.planar(true_base)
            x, y, yaw = plan.planar(base)
            base = plan.base_pose(x + ODOM_SCALE * (after[0] - before[0]), y + ODOM_SCALE * (after[1] - before[1]),
                                  yaw + (after[2] - before[2]))
        else:
            self.fail("the approach did not finish")
        return fix, staging, true_base

    def test_each_tasks_staging_pose_and_its_fallback_put_the_box_root_where_they_should(self):
        """The staging pose at `STAGE_ROOT_M` (where the dog's front camera saw the tag, 2026-10-25), its fallback
        further back in the lens model's band; the box's root no more than `LOOK_SIDE_M` to the side (the look after the
        close, in front of the lock, saw no tag on 2026-10-25 05:49 UTC)."""
        box = box_odom(2.0, 0.6, 200.0)
        for task in FIXES:
            sx, sy, _ = approach.task_stance(task, box)
            for root_m in (approach.STAGE_ROOT_M, approach.STAGE_FAR_ROOT_M):
                x, y, yaw = approach.staging_pose(task, box, root_m)
                facing = np.array([math.cos(yaw), math.sin(yaw)])
                left = np.array([-facing[1], facing[0]])
                ahead = float((box[:2, 3] - np.array([x, y])) @ facing)
                stance = float((box[:2, 3] - np.array([sx, sy])) @ facing)
                stance_side = float((box[:2, 3] - np.array([sx, sy])) @ left)
                self.assertAlmostEqual(ahead, max(root_m, stance), places=6, msg=task)   # never nearer than the stance
                # The tag in view (the root no more than `LOOK_SIDE_M` to the side), on the stance's side.
                side = float((box[:2, 3] - np.array([x, y])) @ left)
                self.assertLessEqual(abs(side), approach.LOOK_SIDE_M + 1e-9, msg=task)
                self.assertGreater(side * stance_side, 0.0, msg=task)
        self.assertGreaterEqual(approach.STAGE_FAR_ROOT_M, plan.FRONT_LOOK_MIN_ROOT_M)

    def test_a_box_out_of_range_is_walked_to_and_in_range_from_there(self):
        """Far, too near (it backs off), off to the side and turned: from where the approach stops, a new look is in the
        task's range."""
        for task in FIXES:
            for ahead, left, yaw_deg in ((2.2, 0.0, 180.0), (1.6, 0.7, 200.0), (1.3, -0.8, 165.0), (0.55, 0.1, 180.0)):
                with self.subTest(task=task, box=(ahead, left, yaw_deg)):
                    box = box_odom(ahead, left, yaw_deg)
                    ok, text = FIXES[task](plan.base_pose(0.0, 0.0, 0.0), *look(plan.base_pose(0.0, 0.0, 0.0),
                                                                                   box)).in_range()
                    if ok:
                        continue                                    # the task's own walk reaches it from here
                    fix, staging, true_base = self.walk_to_stage(task, box)
                    sx, sy, syaw = staging
                    tx, ty, tyaw = plan.planar(true_base)
                    self.assertLess(math.hypot(tx - sx, ty - sy), 0.12, (task, ahead, left))
                    self.assertLess(abs(math.degrees(plan.wrap(tyaw - syaw))), 8.0)
                    again = FIXES[task](true_base, *look(true_base, box))
                    ok, text = again.in_range()
                    self.assertTrue(ok, f"{task}: {text}")

    def test_the_walk_home_ends_where_the_run_began_whatever_the_odometry_drifted(self):
        """`node.py`'s walk back at a run's end (`approach.walk_home`): the start kept against the box from the first
        look, the box from the last look. The odometry here has drifted 15 cm and 3 deg by the end of the run (the dog's
        unseen sit-backs are 4 cm each); walked back on the box, the dog stands where it began, where the odometry's own
        start would be the drift away."""
        box = box_odom(1.45, 0.15, 182.0)                 # the d1dogdemo start: the box ~1.5 m ahead, square on
        start = plan.base_pose(0.0, 0.0, 0.0)
        first = insert_plan.make_insert_fix(start, *look(start, box))
        home_in_box = approach.home_against_box(first.box_odom, start)
        drift = plan.pose(plan.rot_z(np.radians(3.0)), (0.12, -0.09, 0.0))      # odom = drift @ true by the end
        true_base = plan.base_pose(*approach.staging_pose("insert", box))      # where the run ends: the lock's look
        base = drift @ true_base
        last = insert_plan.make_insert_fix(base, *look(base, drift @ box))
        attempt, home = approach.walk_home(last, plan.planar(base), home_in_box, last.box_odom)
        self.assertEqual(attempt.record_key, "return_home_events")
        truth = plan.planar(drift @ start)                 # the start where the drifted odometry now has it
        self.assertLess(math.hypot(home[0] - truth[0], home[1] - truth[1]), 0.02)
        naive = plan.planar(start)                         # the start as the odometry counted it then
        self.assertGreater(math.hypot(home[0] - naive[0], home[1] - naive[1]), 0.1)
        arm = np.asarray(d1_hardware.REST_SERVO_DEG, dtype=float)
        moving = (0.0, 0.0, 0.0)
        for step in range(900):
            command = attempt.step(plan.Observation(0.1 * step, base, 0.0, arm, 0.0))
            self.assertEqual(command.abort, "", command.phase)
            self.assertIsNone(command.arm_servo_deg)        # the arm left where it is (folded)
            if command.finished:
                break
            before = plan.planar(true_base)
            true_base, moving = walk_classic(true_base, command.base if command.walking else None, moving)
            after = plan.planar(true_base)
            c, s = math.cos(np.radians(3.0)), math.sin(np.radians(3.0))
            dx, dy = after[0] - before[0], after[1] - before[1]
            x, y, yaw = plan.planar(base)
            base = plan.base_pose(x + ODOM_SCALE * (c * dx - s * dy), y + ODOM_SCALE * (s * dx + c * dy),
                                  yaw + (after[2] - before[2]))
        else:
            self.fail("the walk home did not finish")
        tx, ty, tyaw = plan.planar(true_base)
        self.assertLess(math.hypot(tx, ty), 0.12)
        self.assertLess(abs(math.degrees(plan.wrap(tyaw))), 8.0)


class ReturnHomeNodeTests(unittest.TestCase):
    """The node's end of a run (`node.py` `_finish`, `_return_home`, `_returned`, `_end`) without ROS: the node built
    without its constructor, its sport mode and exit stubbed."""

    def node(self, argv, base=(0.9, 0.35, 0.2), standing=True, lock_verified=True):
        import tempfile
        from unittest import mock
        from demos.combiner_door.robot import node as node_module

        args = node_module.parse_args(argv)
        n = object.__new__(node_module.CombinerDoorNode)
        box = box_odom(1.45, 0.15, 182.0)
        start = plan.base_pose(0.0, 0.0, 0.0)
        fix = insert_plan.make_insert_fix(start, *look(start, box))
        n.args, n.task, n.closing, n.t0 = args, "insert", False, 0.0
        n.run = {"status": "running", "lock": {"verified": lock_verified}, "return_home": None}
        n.run_dir = __import__("pathlib").Path(tempfile.mkdtemp())
        n.attempt, n.fix, n.staged, n.state = insert_plan.InsertAttempt(fix, "insert", dry=True), fix, None, "attempt"
        n.odom = (0.0, plan.base_pose(*base))
        n.home_in_box = approach.home_against_box(fix.box_odom, start)
        n.home_box, n.returning = fix.box_odom, None
        n.sport_state = type("S", (), {"body_height": 0.32 if standing else 0.07})()
        self.calls = []
        for name in ("_classic_walk", "_stop_motion", "_drive"):
            setattr(n, name, lambda *a, name=name: self.calls.append(name))
        patcher = mock.patch.object(node_module.os, "kill", lambda *a: self.calls.append("exit"))   # `_end`'s exit
        patcher.start()
        self.addCleanup(patcher.stop)
        return n

    def test_a_finished_run_walks_back_then_ends_repeatable_only_when_the_chain_locked(self):
        chain = ["--task", "insert", "--stage", "insert", "--then_door", "pull", "--then_close", "--live"]
        for argv, locked, repeatable in ((chain, True, True), (chain, False, False),
                                         (["--task", "insert", "--stage", "insert", "--live"], True, False)):
            with self.subTest(argv=argv, locked=locked):
                n = self.node(argv, lock_verified=locked)
                n._finish("attempt finished: locked")
                self.assertEqual(n.state, "attempt")                       # walking back, not ended
                self.assertIsInstance(n.attempt, approach.ApproachAttempt)
                self.assertTrue(n.run["return_home"]["walked"])
                self.assertIn("_classic_walk", self.calls)
                self.assertNotIn("exit", self.calls)
                self.assertGreater(n.run["return_home"]["distance_m"], 0.9)
                n._returned(n.attempt)
                self.assertEqual(n.state, "waiting")                       # then a look from there
                self.assertNotIn("exit", self.calls)
                n.odom = (0.0, plan.base_pose(0.01, 0.0, 0.0))
                n._home_looked(self.look_fix(n.odom[1]), 10)                # facing the box as at the start
                self.assertEqual(n.state, "finished")
                self.assertEqual(n.run["outcome"], "attempt finished: locked; back where it started, facing the box as "
                                                   "it did")
                self.assertIs(n.run["repeatable"], repeatable)
                self.assertIn("exit", self.calls)

    def look_fix(self, base):
        return insert_plan.make_insert_fix(base, *look(base, box_odom(1.45, 0.15, 182.0)))

    def test_back_at_the_start_a_look_turns_the_dog_to_face_the_box_as_it_did(self):
        """The walk back by the odometry and the lock's look arrives turned (Lukas, 2026-10-28: "the yaw is off"):
        the look from there finds it 5 deg turned and the dog turns, within `HOME_ACCEPT_YAW_DEG`, then looks again."""
        chain = ["--task", "insert", "--stage", "insert", "--then_door", "pull", "--then_close", "--live"]
        n = self.node(chain)
        n._finish("attempt finished: locked")
        n._returned(n.attempt)
        n.odom = (0.0, plan.base_pose(0.02, 0.01, np.radians(5.0)))
        n._home_looked(self.look_fix(n.odom[1]), 10)
        self.assertEqual(n.state, "attempt")
        self.assertIsInstance(n.attempt, approach.ApproachAttempt)
        self.assertEqual(n.attempt.accept_yaw_deg, approach.HOME_ACCEPT_YAW_DEG)
        self.assertEqual(n.attempt.settle_s, 0.0)
        correction = n.run["return_home"]["corrections"][-1]
        self.assertAlmostEqual(correction["turn_deg"], -5.0, delta=1.0)
        self.assertTrue(correction["walked"])
        n._returned(n.attempt)                                          # turned: the look again
        n.odom = (0.0, plan.base_pose(0.0, 0.0, np.radians(0.5)))
        n._home_looked(self.look_fix(n.odom[1]), 10)
        self.assertEqual(n.state, "finished")
        self.assertTrue(n.run["repeatable"])
        self.assertEqual(len(n.run["return_home"]["corrections"]), 2)

    def test_no_walk_back_with_no_return_lying_down_or_already_there(self):
        chain = ["--task", "insert", "--stage", "insert", "--then_door", "pull", "--then_close", "--live"]
        for argv, kw, why_not in ((chain + ["--no_return"], {}, None), (chain, {"standing": False}, "the dog is not standing"),
                                  (chain, {"base": (0.03, -0.02, 0.02)}, None)):
            with self.subTest(argv=argv, kw=kw):
                n = self.node(argv, **kw)
                n._finish("attempt finished: locked")
                self.assertEqual(n.state, "finished")
                self.assertNotIn("_classic_walk", self.calls)
                record = n.run["return_home"]
                if "--no_return" in argv:
                    self.assertIsNone(record)
                    self.assertFalse(n.run["repeatable"])
                elif why_not:
                    self.assertEqual(record["why_not"], why_not)
                    self.assertFalse(n.run["repeatable"])
                else:
                    self.assertTrue(record["arrived"])                   # already where it began: repeatable
                    self.assertTrue(n.run["repeatable"])


if __name__ == "__main__":
    unittest.main()

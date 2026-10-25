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


if __name__ == "__main__":
    unittest.main()

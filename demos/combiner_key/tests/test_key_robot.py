"""The key turned by the D1 on the robot program, without the robot (`robot/key_plan.py`): the look's fix and the key's
range check from synthetic tag poses and lidar returns, a dry attempt through every stage with the arm link's profile
driving a model servo and the gait's carry, the robot's own check of the turn, the stall abort and the lift-off. numpy
only (no ROS, no simulator)."""
from __future__ import annotations

import math
import unittest

import numpy as np

import d1_hardware
from demos.combiner_door.robot import plan
from demos.combiner_door.robot.arm_link import SimulatedArm
from demos.combiner_door.tests.test_robot import LEVER_STALL_UNITS, box_odom, face_scan, look, walk
from demos.combiner_key import arm_turn as at
from demos.combiner_key import geometry as kg
from demos.combiner_door import perception
from demos.combiner_key.robot import key_plan as kp


#: The base's height above the floor the box stands on, in these dry runs: `plan.base_pose` puts the base at
#: `plan.BASE_HEIGHT_M` (0.32) in the odometry and the door pull's tests the box's root at 0. The dog stood 0.353 m over
#: the box's root (2026-10-21).
BASE_ABOVE_FLOOR_M = plan.BASE_HEIGHT_M


def box_for_lock(ahead_m: float, left_m: float, yaw_deg: float = 180.0, base_above_floor_m: float = BASE_ABOVE_FLOOR_M):
    """The box on a floor `base_above_floor_m` below a base at the origin looking along +x, placed so its lock's point on
    the door is `ahead_m` ahead and `left_m` left (the door square to it at 180 degrees)."""
    box = box_odom(0.0, 0.0, yaw_deg)
    lock = plan.transform(box, [kg.LOCK])[0]
    box = box_odom(ahead_m - lock[0], left_m - lock[1], yaw_deg)
    box[2, 3] = plan.BASE_HEIGHT_M - base_above_floor_m
    return box


class KeyFixTests(unittest.TestCase):
    def test_a_dog_in_front_of_the_lock_walks_straight_to_the_push(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = kp.make_key_fix(base, *look(base, box_for_lock(0.80, 0.08)))
        ok, text = fix.in_range()
        self.assertTrue(ok, text)
        # Sent short by the odometry's error: the dog walks the whole of it (`ODOM_WALK_SCALE`).
        self.assertAlmostEqual(fix.walk_in_m, kp.ODOM_WALK_SCALE * (0.80 - kp.WALK_TO_DOOR_M), delta=0.01)
        x, y, _ = kp.push_stance(perception.site_pose(fix.site))
        self.assertAlmostEqual(x, 0.80 - kp.WALK_TO_DOOR_M - fix.walk_in_m, delta=0.01)
        ahead, left, _ = kp.lock_from(fix, (x, y, 0.0))
        self.assertAlmostEqual(ahead, kp.WALK_TO_DOOR_M, delta=0.01)
        self.assertAlmostEqual(left, kp.STANCE_LOCK_LEFT_M, delta=0.01)

    def test_a_dog_in_front_of_the_boxs_middle_walks_up_and_one_too_far_off_is_told_to_move(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        ok, text = kp.make_key_fix(base, *look(base, box_for_lock(0.80, -kg.LOCK[1], 186.0))).in_range()
        self.assertTrue(ok, text)
        ok, text = kp.make_key_fix(base, *look(base, box_for_lock(0.80, 0.60))).in_range()
        self.assertFalse(ok)
        self.assertIn("move the dog left", text)

    def test_too_close_is_told_to_back_off_and_turned_too_far_is_told_to_turn(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        ok, text = kp.make_key_fix(base, *look(base, box_for_lock(0.15, 0.08))).in_range()
        self.assertFalse(ok)
        self.assertIn("back", text)
        ok, text = kp.make_key_fix(base, *look(base, box_for_lock(0.80, 0.08, 212.0))).in_range()
        self.assertFalse(ok)
        self.assertIn("turn", text)

    def test_after_the_walk_anywhere_the_push_works_is_close_enough(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        for ahead, left, ok_expected in ((0.46, 0.05, True), (0.47, 0.11, True), (0.50, 0.08, False), (0.44, 0.0, False)):
            fix = plan.reanchor(kp.make_key_fix(base, *look(base, box_for_lock(ahead, left))), base)
            self.assertIsInstance(fix, kp.KeyFix)
            self.assertEqual(fix.in_range(check_tag=False)[0], ok_expected, (ahead, left))


def walk_classic(base, command, moving, dt=0.1):
    """`test_robot.walk` with the classic walk's shorter carry on the dog (`key_plan.WALK_UP_CARRY_S`)."""
    if command is None:
        decay = max(0.0, 1.0 - dt / kp.WALK_UP_CARRY_S)
        command = moving = tuple(v * decay for v in moving)
    else:
        moving = tuple(command)
    x, y, yaw = plan.planar(base)
    vx, vy, wz = command
    return plan.base_pose(x + dt * (math.cos(yaw) * vx - math.sin(yaw) * vy),
                          y + dt * (math.sin(yaw) * vx + math.cos(yaw) * vy), yaw + dt * wz), moving


#: The odometry's share of the walk in the dry runs: `key_plan.ODOM_WALK_SCALE`, sport mode's on the dog's four live
#: walks up.
ODOM_SCALE = kp.ODOM_WALK_SCALE


class KeyAttemptTests(unittest.TestCase):
    def run_attempt(self, stage, max_steps=2000, stall_in_turn=False, expect_abort=False, lock=(0.78, 0.08, 180.0),
                    watch=None, odom_scale=ODOM_SCALE, box=None, slow_in_turn=False, then_door=None):
        """A dry key attempt: the arm link's profile driving a model servo, the base walking with the gait's carry, its
        odometry reporting `odom_scale` of the walk (sport mode's, as the dog's walks up suggest), and the lidar's frames
        through the settle from where the base really stands, so the refit takes the odometry's error out.
        `stall_in_turn`: the arm stops following once the turn is a second in (a key that does not turn). `then_door`:
        on into the door pull to that stage once the turn is verified (`DoorAfterKey`, as `node.py --then_door`), the
        jaws stalling on the lever while they hold it; that attempt is left in `self.door`."""
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = box_for_lock(*lock) if box is None else box
        fix = kp.make_key_fix(base, *look(base, box))
        true_base, rng = base, np.random.default_rng(3)
        attempt = kp.KeyAttempt(fix, stage)
        arm = SimulatedArm(d1_hardware.REST_SERVO_DEG)
        phases, moving, turn_steps = [], (0.0, 0.0, 0.0), 0
        self.door = None
        for step in range(max_steps):
            scan = (base, face_scan(true_base, box, rng)) if attempt.phase == "walk_settle" else None
            if self.door is not None:
                arm.stall_units = LEVER_STALL_UNITS if attempt.phase in plan.HOLDING_PHASES else None
            obs = plan.Observation(0.1 * step, base, 0.0, arm.q.copy(), 0.0, scan=scan, gripper_units=arm.gripper)
            command = attempt.step(obs)
            if command.finished and then_door and self.door is None and attempt.turn_check().get("verified"):
                key, attempt = attempt, kp.DoorAfterKey(kp.door_fix(attempt.fix), then_door, plan.scaled_timing(1.0, 30.0))
                self.door = attempt
                continue
            if command.finished:
                if self.door is not None:
                    attempt = key
                break
            if command.abort:
                if expect_abort:
                    return attempt, command.abort, arm, base
                self.fail(f"{stage}: {command.phase}: {command.abort}")
            if not phases or phases[-1] != command.phase:
                phases.append(command.phase)
            turn_steps += command.phase == "turn"
            stalled = stall_in_turn and command.phase == "turn" and turn_steps > 10
            stalled = stalled or (slow_in_turn and command.phase == "turn" and step % 2 == 1)
            if command.arm_servo_deg is not None and not stalled:
                arm.step(command.arm_servo_deg, command.gripper_units, command.speed_deg_s)
                attempt.sent(command)
            if watch is not None:
                watch(attempt, command, arm, base)
            before = plan.planar(true_base)
            true_base, moving = walk_classic(true_base, command.base if command.walking else None, moving)
            after = plan.planar(true_base)
            x, y, yaw = plan.planar(base)
            base = plan.base_pose(x + odom_scale * (after[0] - before[0]), y + odom_scale * (after[1] - before[1]),
                                  yaw + (after[2] - before[2]))
        else:
            self.fail(f"{stage} did not finish")
        if expect_abort:
            self.fail(f"{stage} did not abort")
        return attempt, phases, arm, base

    def test_every_stage_runs_dry_to_the_fold(self):
        for stage in kp.STAGES[2:]:
            with self.subTest(stage=stage):
                attempt, phases, arm, base = self.run_attempt(stage)
                self.assertEqual(phases, [p.name for p in kp.key_phases(stage)])
                np.testing.assert_allclose(arm.q, plan.FOLD_SERVO_DEG, atol=plan.FOLD_ARRIVED_DEG + 0.1)

    def test_the_walk_ends_where_the_push_works(self):
        attempt, _, _, _ = self.run_attempt("walk")
        ahead, left, _ = kp.lock_from_base(attempt.fix)
        self.assertTrue(at.PUSH_DOOR_AHEAD_M[0] <= ahead <= at.PUSH_DOOR_AHEAD_M[1], ahead)
        self.assertTrue(at.PUSH_LOCK_LEFT_M[0] <= left <= at.PUSH_LOCK_LEFT_M[1], left)

    def test_the_arm_stays_off_the_door_through_the_walk_up(self):
        """Lukas's first live walk up hit the door with the door pull's home point (0.42 m ahead of the base)."""
        import d1_ik
        from demos.combiner_door import perception
        from demos.cup.pick_demo.grasp import FINGERTIP_Z_M

        closest = []

        def watch(attempt, command, arm, base):
            if command.phase not in ("walk_in", "walk_settle"):
                return
            q = d1_ik.from_servo_deg(arm.q)[0]
            pos, rot = d1_ik.tool_pose(attempt.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
            tip = plan.transform(attempt.base_script(base), [pos + rot @ np.array([0.0, 0.0, FINGERTIP_Z_M])])[0]
            box = perception.site_pose(attempt.fix.site)
            lock = plan.transform(box, [kg.LOCK])[0]
            closest.append(float((tip - lock) @ (box[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX))))

        self.run_attempt("walk", lock=(0.82, -kg.LOCK[1], 181.4), watch=watch)
        self.assertGreater(min(closest), 0.10)

    def test_the_waiting_arm_is_clear_of_the_trunk_however_high_the_base_stands(self):
        """On the dog the base stood 0.353 m over the box's root, not the script frame's assumed 0.32, and the waiting
        hand, given a script-frame height, went low enough that the trunk proxy refused the move out to the key."""
        import d1_ik
        from demos.cup.pick_demo.grasp import arm_clear

        for height in (0.30, 0.32, 0.353, 0.37):
            clear = []

            def watch(attempt, command, arm, base):
                if command.phase in ("walk_in", "walk_settle", "home"):
                    clear.append(bool(arm_clear(attempt.arm.joints, d1_ik.from_servo_deg(arm.q)[0].reshape(1, 6))[0]))

            with self.subTest(height=height):
                self.run_attempt("reach", box=box_for_lock(0.80, 0.08, 180.0, height), watch=watch)
                self.assertTrue(clear and all(clear))

    def test_the_walk_up_brings_a_dog_from_the_boxs_middle_turned_to_the_push(self):
        """Lukas's first look (2026-10-21): the lock 0.17 m left of where a straight walk would leave it."""
        for lock in ((0.82, -kg.LOCK[1], 180.0), (0.75, -0.05, 174.0), (0.40, 0.20, 186.0)):
            with self.subTest(lock=lock):
                attempt, phases, _, _ = self.run_attempt("turn", lock=lock)
                self.assertTrue(attempt.turn_check()["verified"], attempt.turn_check())
                ahead, left, off = kp.lock_from_base(attempt.fix)
                self.assertTrue(at.PUSH_DOOR_AHEAD_M[0] <= ahead <= at.PUSH_DOOR_AHEAD_M[1], ahead)
                self.assertTrue(at.PUSH_LOCK_LEFT_M[0] <= left <= at.PUSH_LOCK_LEFT_M[1], left)

    def test_the_turn_reaches_the_stop_and_the_robots_check_says_so(self):
        attempt, phases, _, _ = self.run_attempt("turn")
        check = attempt.turn_check()
        self.assertTrue(check["verified"], check)
        self.assertAlmostEqual(check["measured_deg"], at.END_DEG, delta=3.0)
        self.assertLess(check["lag_mm"], at.TRACK_LIMIT_MM)
        out = [e for e in attempt.events if e.get("phase") == "key_out" and "path_clear" in e]
        self.assertTrue(out and out[0]["path_clear"] and out[0]["solved_here"], out)
        planned = [e for e in attempt.events if e.get("phase") == "plan_push"][0]
        self.assertTrue(planned["turns"], planned)

    def test_a_far_box_off_square_still_finds_a_turn_without_a_wrist_flip(self):
        """J5 passes within ~6 deg of the wrist's singularity there: fast but continuous (`ARC_SUBSTEPS`), not a flip."""
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = kp.make_key_fix(base, *look(base, box_for_lock(0.85, 0.11, 177.0)))
        attempt = kp.KeyAttempt(fix, "turn")
        stance = attempt.base_script(base).copy()
        stance[0, 3] = stance[1, 3] = 0.0
        attempt._plan_push(stance)
        self.assertEqual(attempt._blocked, "")
        times = attempt._retime_turn()
        self.assertLess(times, 30.0)

    def test_the_planned_turn_keeps_every_joint_under_the_arm_links_speed(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        attempt = kp.KeyAttempt(kp.make_key_fix(base, *look(base, box_for_lock(0.78, 0.08))), "turn")
        stance = attempt.base_script(base).copy()
        stance[0, 3] = stance[1, 3] = 0.0
        attempt._plan_push(stance)
        seconds = attempt._retime_turn()
        us = np.linspace(0.0, 1.0, 2001)
        degs = [attempt.turn_deg(u) for u in us]
        arc_degs = np.array([d for d, _ in attempt._arc])
        qs = np.degrees(np.array([q for _, q in attempt._arc]))
        joint = np.array([[np.interp(d, arc_degs, qs[:, j]) for j in range(6)] for d in degs])
        rate = np.max(np.abs(np.diff(joint, axis=0)), axis=1) / (seconds / (len(us) - 1))
        self.assertLess(float(rate.max()), plan.ARM_SPEED_DEG_S)

    def test_an_arm_that_runs_behind_but_turns_the_key_is_not_stopped(self):
        """Lukas's first live turn: the key followed, Joint4 ran 5-7 degrees behind, and it aborted as not turning."""
        attempt, phases, _, _ = self.run_attempt("turn", slow_in_turn=True)
        self.assertIn("key_hold", phases)
        self.assertTrue(attempt.turn_check()["verified"], attempt.turn_check())

    def test_a_key_that_does_not_turn_aborts_and_the_hand_lifts_off(self):
        attempt, why, arm, base = self.run_attempt("turn", stall_in_turn=True, expect_abort=True)
        self.assertTrue("not turning" in why or "loaded" in why, why)
        attempt.recover()
        names = [p.name for p in attempt.script.phases]
        self.assertEqual(names, ["key_up", "key_clear", "home", "fold"])
        held = attempt._held_pose()
        up = np.asarray(held["up"]) - np.asarray(held["hand"])
        self.assertGreater(float(up[2]), 0.01)                   # up off the platform
        st = attempt.state(base)
        clear = np.asarray(kp.cleared(st, held["up"]))
        np.testing.assert_allclose(clear[:2], np.asarray(held["up"])[:2])      # then straight up, not back at the camera
        # (Stopped near -45 the hand is already above the camera; at +45 it is 9 cm lower, the platform turned down.)
        self.assertAlmostEqual(float(clear[2]), max(held["up"][2], st["base_z"] + kp.KEY_CLEAR_ABOVE_BASE_M), places=9)
        moving = (0.0, 0.0, 0.0)
        for step in range(600):
            obs = plan.Observation(300.0 + 0.1 * step, base, 0.0, arm.q.copy(), 0.0, gripper_units=arm.gripper)
            command = attempt.step(obs)
            if command.finished:
                break
            self.assertEqual(command.abort, "", command.phase)
            if command.arm_servo_deg is not None:
                arm.step(command.arm_servo_deg, command.gripper_units, command.speed_deg_s)
                attempt.sent(command)
            base, moving = walk(base, command.base if command.walking else None, moving)
        else:
            self.fail("the recovery did not finish")
        self.assertGreater(min(attempt.camera_gaps_mm.values()), 0.0, attempt.camera_gaps_mm)

    def test_the_way_off_the_key_keeps_the_arm_off_the_head_camera(self):
        """After Lukas's first full turn on the dog (2026-10-21) the fingertips went through the head camera's case on
        the way home: backing off along the approach at +45 degrees took the wrist towards it. The clearance proxy's
        margin from the case and its bracket (`camera_margin`), from the model servo's joints, through every phase."""
        for stage, lock, height in (("turn", (0.78, 0.08, 180.0), 0.353), ("turn", (0.85, 0.11, 177.0), 0.37),
                                    ("touch", (0.78, 0.08, 180.0), 0.32), ("reach", (0.75, -0.05, 174.0), 0.30)):
            with self.subTest(stage=stage, lock=lock, height=height):
                attempt, phases, _, _ = self.run_attempt(stage, box=box_for_lock(*lock, height))
                self.assertIn("key_clear", phases)
                gaps = attempt.camera_gaps_mm
                self.assertGreater(min(gaps.values()), 0.0, gaps)
                self.assertGreater(min(gaps["key_clear"], gaps["home"], gaps["fold"]), 30.0, gaps)

    def test_the_door_pull_follows_the_turn_walking_over_without_backing_off(self):
        """`node.py --then_door pull`: from where the key left it, the dog walks over to the door pull's stance (back
        ~0.07 m and right ~0.18 m) with the arm folded, refits there, unfolds and pulls the door."""
        folded, xs = [], []

        def watch(attempt, command, arm, base):
            if isinstance(attempt, kp.DoorAfterKey) and command.phase in ("walk_in", "walk_settle"):
                folded.append(float(np.max(np.abs(np.asarray(arm.q) - plan.FOLD_SERVO_DEG))))
                xs.append(plan.planar(attempt.base_script(base))[0])

        attempt, phases, arm, _ = self.run_attempt("turn", lock=(0.80, 0.07, 180.0), watch=watch, then_door="pull",
                                                   max_steps=4000)
        door = self.door
        self.assertIsNotNone(door, "no door pull after the turn")
        self.assertTrue(attempt.turn_check()["verified"])
        names = [p.name for p in plan.attempt_phases("pull", plan.scaled_timing(1.0, 30.0))]
        order = [n for n in names if n in ("walk_in", "walk_settle")] + [n for n in names if n not in ("walk_in", "walk_settle")]
        self.assertEqual(phases[-len(order):], order)
        self.assertLess(max(folded), 1.0)                         # the arm folded until the dog is over
        self.assertGreater(min(xs), -0.12)                        # no backing off to walk in straight
        reanchor = [e for e in door.events if e.get("phase") == "reanchor"]
        self.assertTrue(reanchor and reanchor[0]["ok"], reanchor)
        self.assertTrue(door._pull is not None and door._pull["done"])
        np.testing.assert_allclose(arm.q, plan.FOLD_SERVO_DEG, atol=plan.FOLD_ARRIVED_DEG + 0.1)

    def test_no_door_pull_after_a_turn_that_was_not_verified(self):
        attempt, why, _, _ = self.run_attempt("turn", stall_in_turn=True, expect_abort=True, then_door="pull")
        self.assertIsNone(self.door)

    def test_the_door_pulls_stance_from_the_keys(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.reanchor(kp.make_key_fix(base, *look(base, box_for_lock(0.46, 0.07))), base)
        door = kp.door_fix(fix)
        self.assertIs(type(door), plan.Fix)
        self.assertAlmostEqual(door.site.distance_m, plan.NOMINAL_DISTANCE_M, places=6)
        self.assertAlmostEqual(door.site.bearing_deg, 0.0, places=6)
        self.assertAlmostEqual(door.site.yaw_deg, 180.0, places=6)
        self.assertTrue(door.in_range(check_tag=False)[0])
        x, y, _ = plan.planar(plan.invert(fix.script_odom) @ door.script_odom)
        self.assertLess(x, 0.0)                                   # back from the key's stance
        self.assertLess(y, -0.10)                                 # and to its right: the handle is right of the lock

    def test_confirmations_name_real_phases(self):
        names = {p.name for p in kp.key_phases("turn")}
        self.assertTrue(set(kp.CONFIRM_BEFORE) <= names, set(kp.CONFIRM_BEFORE) - names)


class _Stub:
    """A ROS message stand-in: any attribute, nested."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        value = _Stub()
        setattr(self, name, value)
        return value


class _Marker(_Stub):
    CUBE, SPHERE, ARROW, LINE_STRIP, LINE_LIST, CYLINDER, TEXT_VIEW_FACING, ADD = range(8)


class KeyVizTests(unittest.TestCase):
    def setUp(self):
        import sys
        import types

        if "geometry_msgs.msg" not in sys.modules:
            pkg, msg = types.ModuleType("geometry_msgs"), types.ModuleType("geometry_msgs.msg")
            msg.Point = lambda x, y, z: (x, y, z)
            pkg.msg = msg
            sys.modules["geometry_msgs"], sys.modules["geometry_msgs.msg"] = pkg, msg
            self.addCleanup(lambda: (sys.modules.pop("geometry_msgs"), sys.modules.pop("geometry_msgs.msg")))

    def test_the_scene_draws_the_key_where_the_fingers_put_it_and_no_door_pull(self):
        from demos.combiner_door import mid360
        from demos.combiner_door import perception
        from demos.combiner_door.robot import viz
        from demos.combiner_key.robot import key_viz

        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = kp.make_key_fix(base, *look(base, box_for_lock(0.78, 0.08)))
        attempt = kp.KeyAttempt(fix, "turn")
        attempt._turn["measured_deg"] = 12.0
        items = viz.scene_markers(_Marker, 0, fix, attempt, "status", "note", mid360.NO_READING, door_task=False,
                                  extra=key_viz.extra_for(attempt))
        spaces = {m.ns for m in items}
        self.assertTrue({"key", "key_arc", "key_stance"} <= spaces, spaces)
        self.assertFalse({"base_path", "grasp"} & spaces)
        bar = [m for m in items if m.ns == "key"][0]
        want = key_viz.key_pose(perception.site_pose(fix.site), 12.0) @ plan.pose(
            None, [0.5 * (lo + hi) for lo, hi in at.KEY_BAR_BOX])
        np.testing.assert_allclose([bar.pose.position.x, bar.pose.position.y, bar.pose.position.z], want[:3, 3],
                                   atol=1e-9)
        self.assertIn("key +12 deg", key_viz.status_text(attempt))


if __name__ == "__main__":
    unittest.main()

"""`node.py --fast` without the robot: the quicker attempts do what the default ones do (the door pulled and closed,
the key drawn, inserted, turned, put back and the lock locked), in less time, and nothing changes without it
(`plan.FAST_*`, `insert_plan.FAST_*`). numpy only."""
from __future__ import annotations

import types
import unittest

import numpy as np

import d1_hardware
from demos.combiner_door.robot import arm_link, plan
from demos.combiner_door.tests import test_robot as door_tests
from demos.combiner_door.tests.test_robot import LinkedArm, door_box, look
from demos.combiner_key.robot import insert_plan as ip
from demos.combiner_key.tests import test_insert_robot as insert_tests

# The harnesses are used through their modules, so that this module's run does not collect their tests again.


class FastArmTests(unittest.TestCase):
    def test_fast_joint_moves_stay_under_the_arm_links_limit_and_are_quicker(self):
        start, goal = np.asarray(plan.FOLD_SERVO_DEG), np.array([5.2, 12.8, 21.7, 20.0, -34.4, 72.8])  # a door unfold
        slow = plan.ArmSender().message(goal, None, True, start)
        fast = plan.ArmSender(speed_deg_s=plan.FAST_JOINT_SPEED_DEG_S).message(goal, None, True, start)
        for msg in (slow, fast):
            self.assertIsNone(arm_link.check_path(msg["q"], msg["dt"], start))
        self.assertLess(plan.FAST_JOINT_SPEED_DEG_S, arm_link.MAX_SPEED_DEG_S)
        self.assertLess(len(fast["q"]), len(slow["q"]))
        np.testing.assert_allclose(fast["q"][-1], slow["q"][-1])

    def test_fast_door_phases_walk_in_before_the_unfold_and_the_default_is_unchanged(self):
        timing = plan.scaled_timing(1.0, 60.0)
        default = [p.name for p in plan.attempt_phases("pull", timing)]
        fast = [p.name for p in plan.attempt_phases("pull", timing, fast=True)]
        self.assertEqual(default[:3], ["unfold", "walk_in", "walk_settle"])
        self.assertEqual(fast[:3], ["walk_in", "unfold", "walk_settle"])
        self.assertEqual(sorted(default), sorted(fast))
        for f in (False, True):
            close = [p.name for p in plan.close_phases(types.SimpleNamespace(timing=timing, fast=f))]
            self.assertEqual(close[:2], ["walk_in", "unfold"] if f else ["unfold", "walk_in"])
        # Stages that do not walk keep their unfold first.
        self.assertEqual([p.name for p in plan.attempt_phases("unfold", timing, fast=True)][0], "unfold")


class FastDoorTests(unittest.TestCase):
    run_attempt = door_tests.AttemptTests.run_attempt

    def test_the_fast_pull_unfolds_while_it_walks_in_and_grips_where_the_default_does(self):
        seen = {False: [], True: []}
        results = {}
        for fast in (False, True):
            watch = lambda a, c, arm, base, fast=fast: seen[fast].append((c.phase, c.walking, arm.q.copy()))
            results[fast] = self.run_attempt("pull", timing=plan.scaled_timing(1.0, 30.0), watch=watch, fast=fast)
        _, phases, q, at_arrive = results[True]
        self.assertEqual(phases, [p.name for p in plan.attempt_phases("pull", plan.scaled_timing(1.0, 30.0), fast=True)])
        # The arm moves while the dog walks in, and the unfold after it has little left to do.
        walking = [arm for ph, w, arm in seen[True] if ph == "walk_in" and w]
        self.assertGreater(np.max(np.abs(walking[-1] - walking[0])), 20.0)
        self.assertLess(sum(ph == "unfold" for ph, _, _ in seen[True]), sum(ph == "unfold" for ph, _, _ in seen[False]))
        self.assertLess(len(seen[True]), len(seen[False]))
        # The same arrival at the lever, and folded at the end.
        for fast in (False, True):
            base_s, jaw_b, grasp_s = results[fast][3]
            grasp_b = plan.transform(plan.invert(base_s), [grasp_s])[0]
            self.assertLess(np.linalg.norm(jaw_b - grasp_b), 0.02, fast)
        self.assertLess(np.max(np.abs(q - np.asarray(plan.FOLD_SERVO_DEG))), plan.FAST_FOLD_ARRIVED_DEG + 0.05)

    def test_a_fold_that_settles_off_by_more_than_half_a_degree_ends_at_once_only_with_fast(self):
        """The dog's arm settles 0.3-0.6 deg off the fold (Joint5): the default's fold then runs its whole clock."""
        lengths = {}
        for fast in (False, True):
            base = plan.base_pose(0.0, 0.0, 0.0)
            fix = plan.make_fix(base, *look(base, door_box(0.92, 0.0, 181.0)))
            attempt = plan.Attempt(fix, "unfold", fast=fast)
            arm = LinkedArm(d1_hardware.REST_SERVO_DEG)
            folding = 0
            for step in range(600):
                q = arm.q.copy()
                if attempt.phase == "fold":
                    q = np.asarray(plan.FOLD_SERVO_DEG) + np.array([0.0, 0.3, 0.0, 0.2, 0.7, 0.1])
                command = attempt.step(plan.Observation(0.1 * step, base, 0.0, q, 0.0, gripper_units=arm.gripper))
                if command.finished:
                    break
                self.assertEqual(command.abort, "")
                folding += command.phase == "fold"
                if command.arm_servo_deg is not None:
                    arm.drive(command, attempt)
                    attempt.sent(command)
            lengths[fast] = 0.1 * folding
        self.assertGreaterEqual(lengths[False], plan.FOLD_S - 0.2)
        self.assertLess(lengths[True], 1.0)


class FastInsertTests(unittest.TestCase):
    run_attempt, finish = insert_tests.InsertAttemptTests.run_attempt, insert_tests.InsertAttemptTests.finish
    _keyway = staticmethod(insert_tests.InsertAttemptTests._keyway)
    _look_at_keyhole = staticmethod(insert_tests.InsertAttemptTests._look_at_keyhole)

    def _steps(self, **kw):
        counted = []
        watch = lambda a, c, arm: counted.append(c.phase)
        attempt = []
        a, phases, arm, _ = self.run_attempt("insert", watch=watch, attempt_out=attempt, **kw)
        return attempt[0], phases, arm, counted

    def test_the_fast_unlock_draws_inserts_turns_and_puts_the_key_back_sooner(self):
        default = self._steps(key_source="holster")
        fast = self._steps(key_source="holster", fast=True)
        for a, phases, arm, _ in (default, fast):
            check = a.unlock_check()
            self.assertTrue(check["verified"], check)
            self.assertTrue(check["reholstered"])
        a, phases, arm, counted = fast
        self.assertEqual([p for p in default[1] if p != "lock_look"], [p for p in phases if p != "lock_look"])
        self.assertLess(len(counted), len(default[3]) - 30)            # at least 3 s sooner
        np.testing.assert_allclose(arm.q, plan.FOLD_SERVO_DEG, atol=plan.FAST_FOLD_ARRIVED_DEG + 0.1)
        self.assertFalse(any(e.get("phase") == "key_in_pace" for e in default[0].events))

    def test_fast_key_in_goes_quickly_through_the_air_and_slowly_from_before_the_lock(self):
        seen = []
        watch = lambda a, c, arm: seen.append((c.phase, a._ins["off"], a._in_pace, a._lag0))
        attempt = []
        self.run_attempt("insert", keyway_mm=(-2.5, 2.0), key_source="holster", fast=True, watch=watch,
                         attempt_out=attempt)
        a = attempt[0]
        paces = [e for e in a.events if e.get("phase") == "key_in_pace"]
        self.assertEqual([e["pace"] for e in paces], ["slow"])
        self.assertLessEqual(paces[0]["commanded_off_mm"], 1000 * ip.FAST_IN_UNTIL_M + 0.1)
        key_in = [s for s in seen if s[0] == "key_in"]
        fast_steps = [s for s in key_in if s[2] == "fast"]
        step = np.diff([s[1] for s in fast_steps])
        np.testing.assert_allclose(step[1:], -ip.FAST_IN_SPEED_M_S * 0.1, atol=1e-9)
        # The contact's baseline taken again at the slow pace, a second after it began, from at least 13 mm out.
        slow = [s for s in key_in if s[2] == "slow"]
        baseline = next(s for s in slow if s[3] is not None)
        self.assertGreater(baseline[1], ip.TRAVEL_M + ip.WRIST_START_M)
        touched = [e for e in a.events if e.get("phase") == "touched"]
        self.assertTrue(touched)
        self.assertTrue(a.check["seated"], a.check)

    def test_the_fast_lock_look_goes_straight_on_to_the_holster(self):
        seen = []
        watch = lambda a, c, arm: seen.append((c.phase, arm.q.copy()))
        attempt = []
        self.run_attempt("insert", key_source="holster", fast=True, watch=watch, attempt_out=attempt)
        a = attempt[0]
        on = [e for e in a.events if e.get("phase") == "lock_look" and "straight_to_h_turn" in e]
        self.assertEqual(len(on), 1)
        self.assertTrue(on[0]["straight_to_h_turn"], on[0])
        self.assertGreaterEqual(on[0]["camera_mm"], 1000 * ip.LOOK_MARGIN_M)
        # Never back at the fold between the look's place and the holster's turn.
        between = [q for ph, q in seen if ph == "lock_look"][-3:]
        self.assertTrue(all(np.max(np.abs(q - np.asarray(plan.FOLD_SERVO_DEG))) > ip.LOOK_ARRIVED_DEG for q in between))

    def test_fast_locking_pushes_a_door_ajar_shut_quicker_and_locks(self):
        """`test_insert_robot`'s door 8 deg ajar on a spring weaker than the press, pressed up to 100 mm beyond its
        face: with `--fast` the key pushes it at `FAST_PUSH_SPEED_M_S` once it is past seated, and still locks."""
        results = {}
        for fast in (False, True):
            attempt, counted = [], []
            watch = lambda a, c, arm, counted=counted: counted.append(c.phase)
            self.run_attempt("insert", key_source="holster", lock_ajar_deg=8.0, lock_soft=True, vision=True,
                             max_steps=20000, lock_beyond_face_mm=100.0, fast=fast, watch=watch, attempt_out=attempt)
            results[fast] = (attempt[0], counted, self.door_deg)
        a, counted, door_deg = results[True]
        check = a.lock_check()
        self.assertTrue(check["verified"], check)
        self.assertLess(door_deg, 1.0)
        self.assertIn("push", [e["pace"] for e in a.events if e.get("phase") == "key_in_pace"])
        self.assertLess(counted.count("key_in") + counted.count("key_close"),
                        results[False][1].count("key_in") + results[False][1].count("key_close") - 30)


class TodaysDogRunsTests(unittest.TestCase):
    """What Lukas's runs of 2026-10-28 01:37 and 01:42 UTC aborted on, both modes unless named."""

    run_attempt, finish = insert_tests.InsertAttemptTests.run_attempt, insert_tests.InsertAttemptTests.finish
    _keyway = staticmethod(insert_tests.InsertAttemptTests._keyway)
    _look_at_keyhole = staticmethod(insert_tests.InsertAttemptTests._look_at_keyhole)

    def test_the_turn_goes_on_with_joint6_well_behind_and_stops_only_when_the_key_does_not_turn(self):
        def held_back(lag_deg):
            def watch(a, command, arm):
                if command.phase == "key_turn" and command.arm_servo_deg is not None and a.script.elapsed_s > 0.5:
                    q = arm.q.copy()
                    q[5] = command.arm_servo_deg[5] - lag_deg       # the lock holding the key back
                    arm.q = q
            return watch
        for fast in (False, True):
            with self.subTest(fast=fast):
                attempt = []
                _, phases, _, why = self.run_attempt("insert", key_source="holster", watch=held_back(19.7),
                                                     attempt_out=attempt, fast=fast)
                self.assertEqual(why, "")
                self.assertIn("h_return", phases)                     # turned and on to the holster
                a, _, _, why = self.run_attempt("insert", key_source="holster", watch=held_back(35.0),
                                                expect_abort=True, fast=fast)
                self.assertIn("the key is not turning", why)
                self.assertEqual(a.phase, "key_turn")

    def test_locking_goes_on_with_joint6_held_by_the_keyway_through_the_seat_and_the_press(self):
        """00:37: locking, the keyway held the key 12.5-16.9 deg from where the wrist's search left its command, and
        `key_close` aborted on it."""
        def watch(a, command, arm):
            if command.phase in ("key_seat", "key_close") and command.arm_servo_deg is not None:
                q = arm.q.copy()
                q[5] = command.arm_servo_deg[5] - 16.9
                arm.q = q
        a, phases, _, why = self.run_attempt("insert", key_source="holster", lock_ajar_deg=0.0, vision=True,
                                             max_steps=14000, watch=watch)
        self.assertEqual(why, "")
        self.assertIn("key_turn", phases)

    def test_fast_key_in_touches_a_lock_much_nearer_than_the_model_without_pushing_on(self):
        """01:37: the lock ~30 mm nearer than the model (no lidar refit), met while the key still caught up from
        `key_pre`. With `--fast` the touch is taken from the lag (`FAST_LAG_MAX_M`), the command no further than that
        ahead of the stopped key. (A key stopped that far out of the model's seated is then searched for on the lock's
        face, in either mode, and the search gives up: the depth is not this test's.)"""
        seen = []
        watch = lambda a, c, arm: seen.append((c.phase, a._measured_key, a._ins["off"], a.script.elapsed_s))
        attempt = []
        _, _, _, why = self.run_attempt("insert", keyway_mm=(0.0, 0.0), key_source="holster", fast=True, watch=watch,
                                        true_deeper_m=ip.LOCK_DEEPER_M - 0.027, attempt_out=attempt,
                                        expect_abort=True)
        self.assertNotIn("tilted", why)
        self.assertNotIn("loaded", why)
        a = attempt[0]
        touched = [e for e in a.events if e.get("phase") == "touched"]
        self.assertTrue(touched)
        self.assertGreater(touched[0]["off_mm"], 25.0)                  # on the box, far out of the model's seated
        ahead = [key[0] - off for ph, key, off, el in seen if ph == "key_in" and key is not None and el >= 1.0]
        self.assertLess(max(ahead), ip.FAST_LAG_MAX_M + 0.006)

    def test_a_recovery_from_a_tilt_runs_while_the_base_is_no_more_tilted(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.make_fix(base, *look(base, door_box(0.92, 0.0, 181.0)))
        attempt = plan.Attempt(fix, "unfold")
        tilted = plan.base_pose(0.0, 0.0, 0.0, 0.0, np.radians(13.0))
        obs = plan.Observation(0.0, tilted, 0.0, np.asarray(plan.FOLD_SERVO_DEG), 0.0)
        self.assertIn("tilted", attempt._abort(obs, "unfold"))
        attempt.tilt_limit_deg = min(plan.RECOVER_TILT_MAX_DEG, 13.0 + plan.RECOVER_TILT_MARGIN_DEG)   # node's `r`
        self.assertEqual(attempt._abort(obs, "unfold"), "")
        worse = plan.base_pose(0.0, 0.0, 0.0, 0.0, np.radians(18.0))
        self.assertIn("tilted", attempt._abort(plan.Observation(0.0, worse, 0.0, np.asarray(plan.FOLD_SERVO_DEG), 0.0),
                                                "unfold"))


if __name__ == "__main__":
    unittest.main()

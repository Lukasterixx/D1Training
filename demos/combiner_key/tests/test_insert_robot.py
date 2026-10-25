"""The key inserted by the D1 on the robot program, without the robot (`robot/insert_plan.py`): a dry attempt through
every stage with the arm link's profile driving a model servo, the gait's carry and the odometry's short count, sport
mode's StandDown lowering the base, and a tight keyway (the bare cam lock's, under 0.5 mm of play) that stops the key's
shaft on the lock's face anywhere else; the touch, the search, the seat, the aborts and the hand-back. numpy only."""
from __future__ import annotations
import math
import unittest
import numpy as np
import d1_hardware
import d1_ik
from demos.combiner_door.robot import approach, plan, viz
from demos.combiner_door.tests.test_robot import LEVER_STALL_UNITS, LinkedArm, face_scan, look
from demos.combiner_key.robot import insert_plan as ip
from demos.combiner_key.tests.test_key_robot import DOOR_AJAR_DEG, ODOM_SCALE, after_walk_back, box_for_lock, walk_classic

#: The keyway's play round the key in these runs: under Lukas's "< 0.5 mm".
CLEAR_M = 0.0004
#: The wrist camera's colour intrinsics at 1280 x 720 (the dog's `camera_info`).
K = np.array([[907.1, 0.0, 640.0], [0.0, 906.0, 360.0], [0.0, 0.0, 1.0]])
#: How StandDown moved the dog on its first lie-down (2026-10-21): back, unseen by the odometry, and turned, seen.
SIT_BACK_M = 0.04
SIT_TURN_DEG = -4.5
#: Sport mode's body height standing, and how far StandDown lowers it here (`insert_plan.LIE_DROP_EST_M`), over 2 s.
STAND_BODY_M = 0.32
BAR_UNITS = ip.BAR_UNITS


#: How far the draw takes the jaws out along the holster's axis: along it, then the exit's part along it.
DRAWN_PLAN_MM = 1000 * (ip.holster.SLIDE_M + ip.holster.EXIT_M * math.cos(math.radians(ip.holster.EXIT_DEG)))


def lock_ahead(walk_m: float = 0.30, right_m: float = ip.INSERT_LOCK_RIGHT_M):
    """(ahead, left) of the lock's point on the door from the look for a walk up of `walk_m` to the insertion's stance."""
    return ip.INSERT_SEATED_AHEAD_M - ip.SEATED_OFF_DOOR_M + walk_m, -right_m


class InsertAttemptTests(unittest.TestCase):
    def run_attempt(self, stage, keyway_mm=(1.5, -1.0), height=0.353, lock=None, key=True, max_steps=6000,
                    drop_m=ip.LIE_DROP_EST_M, expect_abort=False, watch=None, attempt_out=None, slip_in=None,
                    lock_there=True, dry=False, vision=False, calibration_path=None, hidden=(), key_source="hand",
                    lidar_lying=True, true_along_m=None, true_up_m=None, key_right_mm=None, key_up_mm=None, turn_deg=None, true_deeper_m=None,
                    key_in_mm=0.0, reholster=True, then_door=None, ajar_deg=DOOR_AJAR_DEG, look_turn_deg=0.0,
                    lock_ajar_deg=None, lock_soft=False, lock_beyond_face_mm=None, spiral=True):
        """A dry insertion attempt. The keyway is `keyway_mm` (along the key, up its top side) off where the box's model
        has it; the arm's key stops on the lock's face unless its shaft is within `CLEAR_M` of it. `key`: a key between
        the jaws (an int: from that try of the grip on). `slip_in`: the key falls out in that phase. `vision`: the wrist
        camera's frames through `key_view`, rendered from where the arm truly holds it, the bezel round the true keyway;
        `hidden`: the looks (`lock_look`, `key_view`) that do not see it (the held key and jaws in the way), or "all".
        `true_along_m`, `true_up_m`: where the key's shaft truly is along the bar and up its top side from the jaws (by
        default: where the attempt's model of the grip and its nudges have them); `key_right_mm`,
        `key_up_mm`, `key_in_mm`: the attempt's nudges. `reholster`: from the holster, the key put back after the turn.
        `then_door`: on into the door pull to that stage if the insertion unlocked it, as `node.py --then_door`: the door
        swings `ajar_deg` open, the dog backs up and looks again (`test_key_robot.after_walk_back`); the door attempt is
        `self.door`, the walks `self.walks`, the looks' range texts `self.looks`. `true_deeper_m`: the lock truly that far further into the door than the
        box's model (default `LOCK_DEEPER_M`, as the attempt's model has it). `turn_deg`: the attempt's turn once seated (None: its default).
        `lock_ajar_deg`: lock instead (`InsertAttempt.lock`), the door held that far open by its spring: the look and the
        lidar see it there, rigid through the touch and the seat (the spring stiffer than the touch), and through
        `key_close` it turns shut about its hinge as the seated key pushes it, to its frame at 0 (`self.door_deg`).
        `lock_soft`: the spring weaker than the touch, as on the dog (2026-10-25 05:07 UTC): the door turns shut under the
        key from `key_in` on. `lock_beyond_face_mm`: the attempt's press (`InsertAttempt.lock_beyond_face_m`). Returns (attempt, phases, arm, abort reason or "")."""
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = box_for_lock(*(lock or lock_ahead()), 180.0, height)
        closed_box, self.door_deg = box, float(lock_ajar_deg or 0.0)
        if lock_ajar_deg:
            box = viz.door_pose(closed_box, self.door_deg)        # the door, the lock on it, held open
        # `look_turn_deg`: the look sees the box turned this much from where it is (the lidar does not).
        seen = box if not look_turn_deg else box_for_lock(*(lock or lock_ahead()), 180.0 + look_turn_deg, height)
        fix = ip.make_insert_fix(base, *look(base, seen))
        ok, text = fix.in_range()
        self.assertTrue(ok, text)
        attempt = ip.InsertAttempt(fix, stage, dry=dry, calibration_path=calibration_path, key_source=key_source,
                                   key_right_m=None if key_right_mm is None else key_right_mm / 1000.0,
                                   key_up_m=None if key_up_mm is None else key_up_mm / 1000.0,
                                   key_in_m=key_in_mm / 1000.0, reholster=reholster, lock=lock_ajar_deg is not None,
                                   spiral=spiral,
                                   **({} if lock_beyond_face_mm is None else
                                      {"lock_beyond_face_m": lock_beyond_face_mm / 1000.0}),
                                   **({} if turn_deg is None else {"turn_deg": turn_deg}))
        if attempt_out is not None:
            attempt_out.append(attempt)
        arm = LinkedArm(d1_hardware.REST_SERVO_DEG)
        joints = attempt.arm.joints
        true_base, rng, moving = base, np.random.default_rng(5), (0.0, 0.0, 0.0)
        body, posture, phases, released = STAND_BODY_M, "", [], False
        a0, b0 = (v / 1000.0 for v in keyway_mm)
        self.door, self.walks, self.looks = None, [], []
        deeper = ip.LOCK_DEEPER_M if true_deeper_m is None else true_deeper_m
        lock_box = box.copy()
        lock_box[:3, 3] -= deeper * box[:3, 0]
        true_along = attempt.shaft_along if true_along_m is None else true_along_m
        true_up_m = attempt.shaft_up if true_up_m is None else true_up_m
        for step in range(max_steps):
            dropped = STAND_BODY_M - body
            lowered = true_base.copy()
            lowered[2, 3] -= dropped
            sensed = base.copy()
            scan = None
            if attempt.phase == "walk_settle" or (attempt.phase == "lie_settle" and lidar_lying):
                scan = (base, face_scan(lowered, box, rng))      # the lidar sees from where the base really is
            slipped = slip_in is not None and slip_in in phases
            there = not self.walks and bool(key) and attempt._grips + 1 >= int(key) and not slipped
            arm.stall_units = BAR_UNITS if (there and not released) else None
            if self.door is not None:
                arm.stall_units = LEVER_STALL_UNITS if attempt.phase in plan.HOLDING_PHASES else None
            obs = plan.Observation(0.1 * step, sensed, 0.0, arm.q.copy(), 0.0, scan=scan, gripper_units=arm.gripper,
                                   body_height_m=body)
            command = attempt.step(obs)
            if command.finished and then_door and not self.walks and attempt.unlock_check()["verified"]:
                insert, attempt = attempt, approach.door_after_key(attempt.fix, plan.planar(base))
                self.walks.append(attempt)
                released = True
                box = viz.door_pose(box, ajar_deg)              # unlocked, the door swings open
                continue
            if command.finished and isinstance(attempt, approach.ApproachAttempt):
                self.assertLessEqual(len(self.walks), approach.MAX_APPROACHES, self.looks)
                attempt, text = after_walk_back(base, true_base, box, then_door)
                self.looks.append(text)
                if isinstance(attempt, approach.ApproachAttempt):
                    self.walks.append(attempt)
                else:
                    self.door = attempt
                continue
            if command.finished:
                if self.walks:
                    attempt = insert
                break
            if command.abort:
                if expect_abort:
                    return attempt, phases, arm, command.abort
                self.fail(f"{stage}: {command.phase}: {command.abort}")
            if not phases or phases[-1] != command.phase:
                phases.append(command.phase)
            if command.posture == "lie" and posture != "lie":
                # StandDown sets the dog down further back and turned (the dog's first lie-down: ~4 cm back, which the
                # odometry missed, and ~4-5 degrees, which it saw).
                tx, ty, tyaw = plan.planar(true_base)
                c, sn = math.cos(tyaw), math.sin(tyaw)
                true_base = plan.base_pose(tx - SIT_BACK_M * c, ty - SIT_BACK_M * sn, tyaw + math.radians(SIT_TURN_DEG))
                x, y, yaw = plan.planar(base)
                base = plan.base_pose(x, y, yaw + math.radians(SIT_TURN_DEG))
            posture = command.posture or posture
            body += float(np.clip((STAND_BODY_M - (drop_m if posture == "lie" else 0.0)) - body, -0.01, 0.01))
            if command.arm_servo_deg is not None:
                arm.drive(command, attempt)
                attempt.sent(command)
            released = released or command.phase in ("release", "h_open")
            if there and not released and lock_there and lock_ajar_deg \
                    and (attempt.phase == "key_close" or (lock_soft and attempt.phase == "key_in")) \
                    and getattr(arm, "in_keyway", False):
                # The seated key pushing past seated turns the door shut instead, as far as its frame.
                origin, _ = ip.key_in_hand(joints, arm.q, lowered, true_along, attempt.grip_roll, true_up_m)
                seated, out, _, _ = ip._frames(lock_box)
                past = -float((origin - seated) @ out)
                if past > 0.0 and self.door_deg > 0.0:
                    r = abs(ip.kg.LOCK[1] - viz.GEOMETRY.hinge[1])
                    self.door_deg = max(0.0, self.door_deg - math.degrees(past / r))
                    box = viz.door_pose(closed_box, self.door_deg)
                    lock_box = box.copy()
                    lock_box[:3, 3] -= deeper * box[:3, 0]
            if there and not released and lock_there:
                self._keyway(joints, arm, lowered, lock_box, a0, b0, true_along, attempt.grip_roll, true_up_m)
            if vision and not self.walks and attempt.wants_keyhole(0.1 * step):
                hide = hidden == "all" or attempt.phase in hidden
                attempt.add_keyhole(self._look_at_keyhole(attempt, arm, lowered, base, lock_box, a0, b0, hide))
            if watch is not None:
                watch(attempt, command, arm)
            before = plan.planar(true_base)
            true_base, moving = walk_classic(true_base, command.base if command.walking else None, moving)
            after = plan.planar(true_base)
            x, y, yaw = plan.planar(base)
            base = plan.base_pose(x + ODOM_SCALE * (after[0] - before[0]), y + ODOM_SCALE * (after[1] - before[1]),
                                  yaw + (after[2] - before[2]))
        else:
            self.fail(f"{stage} did not finish")
        if expect_abort:
            self.fail(f"{stage} did not abort")
        return attempt, phases, arm, ""

    @staticmethod
    def _look_at_keyhole(attempt, arm, base_true, base_sensed, box, a0, b0, hide=False):
        """A frame of the wrist camera: rendered from the arm's true pose (the base where it really is, lowered) with
        the bezel round the true keyway (`hide`: a plain door, the bezel hidden behind the key and jaws); then
        `keyhole.find_bezel` as the node runs it (the image's search first), from where the attempt believes the camera
        is."""
        from demos.combiner_key import geometry as kg, keyhole

        cam_base = plan.wrist_camera_base(attempt.arm.joints, arm.q)
        _, out, u, t = ip._frames(box)
        true_centre = (box @ np.array([kg.LOCK[0] + ip.LOCK_FACE_OFF_DOOR_M, kg.LOCK[1], kg.LOCK[2], 1.0]))[:3]
        image = keyhole.render(K, base_true @ cam_base, (1280, 720), true_centre + a0 * u + b0 * t, out, u, t,
                               occluders=[(900, 560, 1100, 720, 90)] + ([(0, 0, 1280, 720, 190)] if hide else []))
        centre, out_s, u_s, t_s = attempt.keyhole_prediction()
        return keyhole.find_bezel(image, K, attempt.base_script(base_sensed) @ cam_base, centre, out_s, u_s, t_s,
                                  wide_m=attempt.keyhole_window_m(keyhole.WIDE_SEARCH_M))

    @staticmethod
    def _keyway(joints, arm, base_true, box, a0, b0, along_m=0.0, roll=-1, up_m=0.0) -> None:
        """The lock's face and its tight keyway: the held key's shaft cannot pass the face more than `CLEAR_M` off the
        keyway, nor its back face pass seated; once in, the keyway's sides hold it within `CLEAR_M` across. The arm is
        put back where the key stops (rigid, frictionless)."""
        origin, _ = ip.key_in_hand(joints, arm.q, base_true, along_m, roll, up_m)
        seated, out, u, t = ip._frames(box)
        rel = origin - seated
        off = float(rel @ out)
        across = np.array([float(rel @ u) - a0, float(rel @ t) - b0])
        miss = float(np.linalg.norm(across))
        # In the keyway only if it went in through it (the arm can move the key more than the face's depth in a step).
        if off >= ip.TRAVEL_M:
            arm.in_keyway = False
        elif miss <= CLEAR_M:
            arm.in_keyway = True
        inside = getattr(arm, "in_keyway", False)
        if inside:
            # Back against the keyway's side, and no deeper than seated wherever it is across (the keyway's floor stops
            # a key that has gone in, off its centre or not; the wrist's search leaves it a few tenths off).
            push = np.zeros(3)
            if miss > CLEAR_M:
                push = push - (1.0 - CLEAR_M / miss) * (across[0] * u + across[1] * t)
            if off < 0.0:
                push = push - off * out
            if not push.any():
                return
        elif off < (0.0 if miss <= CLEAR_M else ip.TRAVEL_M):
            push = ((0.0 if miss <= CLEAR_M else ip.TRAVEL_M) - off) * out          # on the face, or seated
        else:
            return
        push = plan.invert(base_true)[:3, :3] @ push                                  # base frame
        q = d1_ik.from_servo_deg(arm.q)[0]
        pos, rot = d1_ik.tool_pose(joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
        result = d1_ik.solve(joints, pos + push, rot, q0=q, body="Link6", offset=(0.0, 0.0, 0.0), max_iterations=100)
        arm.q = np.asarray(d1_ik.to_servo_deg(result.q), dtype=float)

    def test_every_stage_short_of_the_insertion_hands_the_key_back_and_folds(self):
        for stage in ("load", "walk", "lie", "reach", "touch"):
            with self.subTest(stage=stage):
                attempt, phases, arm, _ = self.run_attempt(stage)
                self.assertEqual(phases, [p.name for p in ip.stage_phases(stage, attempt)])
                self.assertEqual(phases[-4:], ["unload", "hand_back", "drop", "fold"])
                np.testing.assert_allclose(arm.q, plan.FOLD_SERVO_DEG, atol=plan.FOLD_ARRIVED_DEG + 0.1)
                self.assertTrue(attempt.check["held"])

    def test_the_touch_finds_the_lock_face_where_the_shaft_meets_it(self):
        attempt, phases, _, _ = self.run_attempt("touch")
        self.assertIn("key_in", phases)
        self.assertAlmostEqual(attempt.check["contact_off_mm"], 1000 * ip.TRAVEL_M, delta=1.5)
        self.assertFalse(attempt.check["seated"])

    def test_the_search_finds_a_keyway_a_few_mm_off_and_seats_the_key(self):
        for keyway in ((1.5, -1.0), (-2.5, 2.0), (0.2, 0.1)):
            with self.subTest(keyway=keyway):
                attempt, phases, arm, _ = self.run_attempt("insert", keyway_mm=keyway)
                check = attempt.check
                self.assertTrue(check["seated"], check)
                self.assertLessEqual(check["measured_off_mm"], 1000 * ip.SEATED_TOL_M + 1.0)
                self.assertEqual(phases[-5:], ["key_back", "home", "stand_up", "stand_settle", "fold"])
                self.assertIn("release", phases)
                np.testing.assert_allclose(arm.q, plan.FOLD_SERVO_DEG, atol=plan.FOLD_ARRIVED_DEG + 0.1)
                if check["entered_at_mm"] is not None:      # found by the search, not straight in
                    self.assertLess(math.dist(check["entered_at_mm"], keyway), 1000 * (CLEAR_M + ip.SEARCH_PITCH_M) + 1.0)

    def test_the_camera_puts_the_key_on_a_keyway_mm_off_and_the_touch_calibrates_it(self):
        """The wrist camera's bezel (`keyhole.py`) in rendered frames from where the arm truly is, the dog sat back and
        turned as on the dog: the key goes in on (or a step from) the keyway the camera saw, and the residual is kept."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            path = f"{tmp}/keyhole.json"
            for keyway in ((3.5, -1.5), (-5.0, 3.0)):
                with self.subTest(keyway=keyway):
                    attempt, phases, _, _ = self.run_attempt("insert", keyway_mm=keyway, vision=True,
                                                             calibration_path=path)
                    vision = attempt.check["vision"]
                    self.assertGreaterEqual(vision["found"], ip.VIEW_MIN_FRAMES, vision)
                    self.assertTrue(attempt.check["seated"], attempt.check)
                    self.assertLess(attempt.check["searched_r_mm"], 1.5, attempt.check)      # on it, or next to it
                    # The first look moves the model onto it from the fold; `key_view`, close, the rest.
                    moved = attempt.check["look"]["moved_mm"]
                    total = (moved[0] + vision["offset_mm"][0], moved[1] + vision["offset_mm"][1])
                    self.assertLess(math.dist(total, keyway), 3.0, (attempt.check["look"], vision))
            saved = ip._load_calibration(path)
            self.assertEqual(len(saved["runs"]), 2)

    def test_it_lies_down_folded_looks_for_the_lock_then_takes_the_key(self):
        """Lukas's order (2026-10-23): the walk, lying down with the arm folded and empty, the wrist camera's look for
        the lock (tilted up, panned by the arm's base), then the arm unfolds and the key is handed in."""
        _, phases, _, _ = self.run_attempt("insert", vision=True)
        order = [phases.index(p) for p in ("walk_settle", "lie_down", "lie_settle", "lock_look", "unfold", "load",
                                           "grip", "key_go", "key_view", "key_pre")]
        self.assertEqual(order, sorted(order), phases)

    def test_the_first_look_moves_the_model_onto_the_lock_when_the_key_hides_it_close_up(self):
        """The bezel 25 mm off the model (past the rim search's 12 mm round it), and hidden from `key_view` by the
        held key and jaws as on the dog: `lock_look` finds it from the fold, the box model moves onto it, and the key
        goes in there."""
        # 25 mm off: what was (20, -15) along v3's key and up its top side at 45 degrees, the key now level (v4).
        offset = (24.7, 3.5)
        attempt, _, _, _ = self.run_attempt("insert", keyway_mm=offset, vision=True, hidden=("key_view",))
        look, vision = attempt.check["look"], attempt.check["vision"]
        self.assertGreaterEqual(look["found"], ip.VIEW_MIN_FRAMES, look)
        self.assertLess(math.dist(look["moved_mm"], offset), 4.0, look)
        self.assertEqual(vision["found"], 0, vision)
        self.assertTrue(attempt.check["seated"], attempt.check)

    def test_the_first_look_reaches_the_offset_seen_on_the_dog(self):
        """The bezel where the dog's two runs on 2026-10-22/23 saw it from the fold: 19 mm along the key and 74 mm down
        its top side from the model's place (~66 mm right and 39 mm down), past the first look's old 80 mm window. From
        the fold, that far to the side, the look alone leaves a few mm (its depth is the model's), more than the touch's
        4 mm spiral was found to take up here; `key_view`, close, the rest."""
        # ~66 mm right and 39 mm down: along the level key (v4, 2026-10-24) and up its top side.
        offset = (65.8, -38.9)
        attempt, _, _, _ = self.run_attempt("insert", keyway_mm=offset, vision=True)
        look = attempt.check["look"]
        self.assertGreaterEqual(look["found"], ip.VIEW_MIN_FRAMES, look)
        self.assertLess(math.dist(look["moved_mm"], offset), 5.0, look)
        self.assertTrue(attempt.check["seated"], attempt.check)

    def test_the_folded_looks_aim_at_the_models_bezel_and_pan_with_the_arms_base(self):
        attempt, _, _, _ = self.run_attempt("insert", vision=True)
        plan_look = next(e for e in attempt.events if e.get("phase") == "plan_look")
        self.assertEqual(plan_look["places"], len(ip.LOOK_PAN_DEG), plan_look)
        aim = np.asarray(plan_look["aim_servo_deg"])
        fold = np.asarray(plan.FOLD_SERVO_DEG)
        self.assertLess(aim[4], fold[4] - 10.0, "the wrist tilted up from the fold")
        self.assertTrue(np.allclose(np.delete(aim, [0, 4]), np.delete(fold, [0, 4])), "only servos 1 and 5 move")
        pans = [p[0] - attempt._look.places[0][0] for p in attempt._look.places]
        self.assertTrue(np.allclose(pans, ip.LOOK_PAN_DEG), pans)

    def test_no_bezel_in_either_look_goes_in_on_the_model(self):
        attempt, phases, _, _ = self.run_attempt("insert", keyway_mm=(1.0, 0.0), vision=True, hidden="all")
        look = attempt.check["look"]
        self.assertEqual(look["found"], 0, look)
        self.assertEqual(look["places_tried"], look["places"], look)
        self.assertEqual(len([e for e in attempt.events if e.get("phase") == "lock_look" and "place" in e]),
                         look["places"])
        self.assertTrue(attempt.check["seated"], attempt.check)

    def test_a_door_further_than_the_model_is_measured_close_up_and_the_key_seats(self):
        """The dog sits back 80 mm where the model allows 40 (`LIE_BACK_M`), so the door is 40 mm further than modelled,
        as both live runs on 2026-10-22 seem to have had it (the key 5 mm past seated touching nothing): the looks' finds
        put the bezel that far out by its size, the box moves, and the key seats."""
        from unittest import mock

        with mock.patch(f"{__name__}.SIT_BACK_M", 0.08):
            attempt, _, _, _ = self.run_attempt("insert", keyway_mm=(1.0, -0.5), vision=True, lidar_lying=False)
        # The first look moves it out by its size from 0.48 m (2-3% long: ~1 cm too far), `key_view` close up the rest.
        moved = attempt.check["look"].get("out_mm", 0.0) + attempt.check["vision"].get("out_mm", 0.0)
        self.assertAlmostEqual(moved, -40.0, delta=4.0, msg=(attempt.check["look"], attempt.check["vision"]))
        self.assertTrue(attempt.check["seated"], attempt.check)

    def test_without_the_camera_a_door_further_than_the_model_is_found_by_the_touch(self):
        """30 mm further and nothing seen: the key goes on past the model's seated (`IN_SLACK_M`), meets the lock's
        face, takes it as the face (past seated), and searches and seats from there."""
        from unittest import mock

        with mock.patch(f"{__name__}.SIT_BACK_M", 0.07):
            attempt, _, _, _ = self.run_attempt("insert", keyway_mm=(1.5, -1.0), lidar_lying=False)
        touched = next(e for e in attempt.events if e.get("phase") == "touched")
        self.assertFalse(touched["seated"], touched)
        self.assertLess(touched["off_mm"], -1000 * ip.TRAVEL_M / 2, touched)
        self.assertTrue(attempt.check["seated"], attempt.check)

    def test_a_keyway_beyond_the_search_aborts_and_the_key_comes_back_out(self):
        attempt, phases, arm, why = self.run_attempt("insert", keyway_mm=(6.0, 0.0), expect_abort=True)
        self.assertIn("no keyway", why)
        attempt.recover()
        names = [p.name for p in attempt.script.phases]
        self.assertEqual(names, ["key_out", "home", "stand_up", "stand_settle", "unload", "hand_back", "drop", "fold"])
        self.finish(attempt, arm)

    def finish(self, attempt, arm):
        body = STAND_BODY_M - ip.LIE_DROP_EST_M
        base = plan.base_pose(0.0, 0.0, 0.0)
        for step in range(800):
            obs = plan.Observation(900.0 + 0.1 * step, base, 0.0, arm.q.copy(), 0.0, gripper_units=arm.gripper,
                                   body_height_m=body)
            command = attempt.step(obs)
            if command.finished:
                break
            self.assertEqual(command.abort, "", command.phase)
            if command.posture in ("stand", "balance"):
                body = min(STAND_BODY_M, body + 0.01)
            if command.arm_servo_deg is not None:
                arm.drive(command, attempt)
                attempt.sent(command)
        else:
            self.fail("the recovery did not finish")
        np.testing.assert_allclose(arm.q, plan.FOLD_SERVO_DEG, atol=plan.FOLD_ARRIVED_DEG + 0.1)

    def test_a_key_not_there_at_the_first_grip_is_taken_at_the_next(self):
        attempt, phases, _, _ = self.run_attempt("load", key=2)
        self.assertEqual(phases[:5], ["unfold", "load", "grip", "load", "grip"])
        grips = [e for e in attempt.events if e.get("phase") == "grip_check"]
        self.assertEqual([g["held"] for g in grips], [False, True])

    def test_a_key_that_slips_out_aborts_and_the_arm_folds_without_a_hand_back(self):
        attempt, phases, arm, why = self.run_attempt("reach", slip_in="key_go", expect_abort=True)
        self.assertIn("no longer between the jaws", why)
        attempt.recover()
        self.assertEqual([p.name for p in attempt.script.phases][-1], "fold")
        self.assertNotIn("hand_back", [p.name for p in attempt.script.phases])

    def test_with_no_lock_a_live_run_aborts_and_a_dry_one_takes_the_key_as_seated(self):
        attempt, phases, _, why = self.run_attempt("insert", lock_there=False, expect_abort=True)
        self.assertIn("no lock there", why)
        attempt, phases, _, _ = self.run_attempt("insert", lock_there=False, dry=True)
        self.assertTrue(attempt.check["seated"])
        self.assertEqual(phases[-1], "fold")

    def test_jaws_that_close_on_nothing_abort_before_the_key_goes_to_the_lock(self):
        """The hand-in is lying now, after the first look (2026-10-23); an empty grip still stops it there."""
        attempt, phases, _, why = self.run_attempt("insert", key=False, expect_abort=True)
        self.assertIn("closed on nothing", why)
        self.assertEqual(phases[:6], ["walk_in", "walk_settle", "lie_down", "lie_settle", "lock_look", "unfold"])
        self.assertNotIn("key_go", phases)

    def test_the_wrist_frame_draws_the_model_the_search_and_what_it_found(self):
        """RViz's camera panel through the key phases (2026-10-22): the bezel where the model has it, the search's area
        and the search's own circle, projected through the wrist camera; on a rendered view of a bezel 3 mm off."""
        from demos.combiner_door.robot import viz
        from demos.combiner_key import keyhole
        from demos.combiner_key.robot import insert_viz

        attempt, _, _, _ = self.run_attempt("lie")
        centre, normal, u, t = (np.asarray(v, dtype=float) for v in attempt.keyhole_prediction())
        eye = centre + 0.17 * normal
        z = (centre - eye) / np.linalg.norm(centre - eye)
        x = np.cross(z, t)
        x /= np.linalg.norm(x)
        camera = np.eye(4)
        camera[:3, :3] = np.column_stack((x, np.cross(z, x), z))
        camera[:3, 3] = eye
        true_centre = centre + 0.003 * u
        image = keyhole.render(K, camera, (1280, 720), true_centre, normal, u, t)
        bezel = keyhole.find_bezel(image, K, camera, centre, normal, u, t)
        self.assertTrue(bezel.ok, bezel.why)
        outlines, text = insert_viz.keyhole_outlines(attempt.keyhole_prediction(), K, camera, bezel)
        self.assertEqual([o[3] for o in outlines], ["search", "model", "found"])
        for points, _, _, _ in outlines:
            self.assertTrue(np.isfinite(points).all())
            self.assertTrue(((points[:, 0] > 0) & (points[:, 0] < 1280) & (points[:, 1] > 0) & (points[:, 1] < 720)).all())
        self.assertIn("FOUND +3.0", text)
        self.assertGreater(len(viz.debug_jpeg(np.dstack([image] * 3), [], text, outlines=outlines)), 1000)

    def test_the_keyhole_search_runs_alone_dry_only(self):
        """`--task insert --stage keyhole`: the look, then the bezel searched on every wrist frame, the arm moved by hand
        (2026-10-22). Dry only: it commands nothing, and the arm link observes."""
        import contextlib
        import io

        from demos.combiner_door.robot import node

        self.assertEqual(node.parse_args(["--task", "insert", "--stage", "keyhole"]).stage, "keyhole")
        for argv in (["--task", "insert", "--stage", "keyhole", "--live"], ["--task", "door", "--stage", "keyhole"]):
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                node.parse_args(argv)

    def test_the_walk_up_and_its_refit_happen_with_the_arm_folded(self):
        """Carried out in front, the key and hand shadow the door from the lidar: the refit after the walk turned it 6.9
        deg and aborted (Lukas's live run, 2026-10-22). Now the arm holds the fold until the walk and its refit are over."""
        sent = {}

        def watch(attempt, command, arm):
            if command.phase in ("walk_in", "walk_settle") and command.arm_servo_deg is not None:
                sent.setdefault(command.phase, []).append(np.asarray(command.arm_servo_deg))

        attempt, phases, _, _ = self.run_attempt("lie", watch=watch)
        self.assertEqual(phases[:6], ["walk_in", "walk_settle", "lie_down", "lie_settle", "lock_look", "unfold"])
        for phase in ("walk_in", "walk_settle"):
            self.assertTrue(sent.get(phase), phase)
            for q in sent[phase]:
                np.testing.assert_allclose(q, plan.FOLD_SERVO_DEG, atol=1e-9)
        self.assertTrue(any(e.get("phase") == "refit" and e.get("used") for e in attempt.events))

    def test_lying_down_adds_the_measured_sit_back_and_the_lidar_confirms_it(self):
        """The 4 cm of sit-back the odometry did not see is added, and the lidar's refit lying (the arm folded and
        empty through `lie_settle`) finds the door where that put it."""
        attempt, phases, _, _ = self.run_attempt("insert")
        lying = [e for e in attempt.events if e.get("phase") == "refit_lying"]
        self.assertEqual(len(lying), 1)
        self.assertAlmostEqual(lying[0]["sit_back_added_mm"], 1000 * SIT_BACK_M, delta=1.0)
        self.assertTrue(lying[0]["used"], lying[0])
        self.assertLess(lying[0]["moved_mm"], 15.0)
        self.assertTrue(attempt.check["seated"])

    def test_lying_the_lidar_measures_a_sit_back_the_model_does_not_allow_for(self):
        """The dog sits back 80 mm where the model allows 40: the lying refit moves the door the other 40 mm, before
        the look is aimed and the insertion planned, and the key seats without the camera."""
        from unittest import mock

        with mock.patch(f"{__name__}.SIT_BACK_M", 0.08):
            attempt, _, _, _ = self.run_attempt("insert", keyway_mm=(1.0, -0.5))
        lying = next(e for e in attempt.events if e.get("phase") == "refit_lying")
        self.assertTrue(lying["used"], lying)
        self.assertAlmostEqual(lying["moved_mm"], 40.0, delta=5.0)
        self.assertTrue(attempt.check["seated"], attempt.check)

    def test_with_no_face_lying_the_sit_back_alone_stands(self):
        attempt, _, _, _ = self.run_attempt("insert", lidar_lying=False)
        lying = next(e for e in attempt.events if e.get("phase") == "refit_lying")
        self.assertFalse(lying["used"])
        self.assertIn("sit-back alone", lying["why"])
        self.assertTrue(attempt.check["seated"])

    def test_a_dog_that_does_not_lie_down_aborts(self):
        attempt, phases, _, why = self.run_attempt("lie", drop_m=0.02, expect_abort=True)
        self.assertIn("did not lie down", why)

    def test_lying_down_the_insertion_is_planned_within_reach_however_high_the_dog_stood(self):
        """The base 0.33-0.39 m over the box's root standing (the dog's 0.353-0.391 by its looks), 0.20 m lower lying."""
        for height in (0.33, 0.353, 0.39):
            with self.subTest(height=height):
                attempt, _, _, _ = self.run_attempt("lie", height=height)
                rows = attempt.check["plan"]
                self.assertTrue(rows, attempt.events)
                self.assertLess(max(r["shoulder_share"] for r in rows), 0.9)
                self.assertGreater(min(r["camera_mm"] for r in rows), 0.0)
                self.assertAlmostEqual(attempt.check["lie_drop_m"], ip.LIE_DROP_EST_M, delta=0.01)

    def test_the_arm_keeps_off_the_head_camera_throughout(self):
        attempt, _, _, _ = self.run_attempt("insert")
        self.assertGreater(min(attempt.camera_gaps_mm.values()), 0.0, attempt.camera_gaps_mm)

    def test_confirmations_name_real_phases(self):
        names = set()
        for source in ip.KEY_SOURCES:
            attempt = []
            self.run_attempt("load", attempt_out=attempt, key_source=source)
            names |= {p.name for p in ip.stage_phases("insert", attempt[0])}
        self.assertTrue(set(ip.CONFIRM_BEFORE) <= names, set(ip.CONFIRM_BEFORE) - names)
        self.assertTrue({"load", "grip"} <= names)
        self.assertTrue(set(ip.HOLSTER_PHASES) <= names)



class HolsterInsertTests(unittest.TestCase):
    """The key drawn out of the holster beside the arm's base once the dog lies in front of the lock
    (`key_source="holster"`), then inserted; and the wrist's ±10 deg search while it is pushed in."""

    run_attempt, finish = InsertAttemptTests.run_attempt, InsertAttemptTests.finish
    _keyway, _look_at_keyhole = staticmethod(InsertAttemptTests._keyway), staticmethod(InsertAttemptTests._look_at_keyhole)

    def test_lying_down_it_draws_the_key_from_the_holster_then_inserts_and_seats_it(self):
        attempt = []
        a, phases, arm, _ = self.run_attempt("insert", attempt_out=attempt, key_source="holster")
        a = attempt[0]
        order = [p for p in ("lie_down", "lock_look", "h_turn", "h_reach", "h_lower", "h_on", "grip", "h_draw", "h_out",
                             "key_go", "key_in") if p in phases]
        self.assertEqual(order, ["lie_down", "lock_look", "h_turn", "h_reach", "h_lower", "h_on", "grip", "h_draw",
                                 "h_out", "key_go", "key_in"])
        # Drawn the dog's full length before the key goes to the lock, with the base yaw and wrist back to 0.9 after.
        self.assertGreaterEqual(a.check["drawn_mm"], DRAWN_PLAN_MM - 4.0)
        self.assertEqual(a.arm.soft, a._soft)
        self.assertNotIn("load", phases)
        self.assertTrue(a.check["held"])
        self.assertTrue(a.check["seated"], a.check)
        self.assertEqual(a.check["key_source"], "holster")
        self.assertEqual(a.arm.grasp, (0.0, 1))
        go = [e for e in a.events if e.get("phase") == "key_go"][0]
        self.assertTrue(go["path_clear"])
        self.assertGreater(go["key_margin_mm"], 0.0)
        np.testing.assert_allclose(arm.q, plan.FOLD_SERVO_DEG, atol=plan.FOLD_ARRIVED_DEG + 0.1)

    def test_the_wrist_s_search_begins_near_the_lock_s_face_not_where_the_key_starts_in(self):
        """Lukas, 2026-10-24: "it always starts its insertion wiggle very early": the wrist holds still until the key is
        `WRIST_START_M` off the face."""
        seen = []
        watch = lambda a, c, arm: seen.append((c.phase, a._measured_key, a._wrist["theta"]))
        self.run_attempt("insert", keyway_mm=(-2.5, 2.0), watch=watch, key_source="holster")
        far = [th for ph, key, th in seen if ph == "key_in" and key is not None
               and key[0] > ip.TRAVEL_M + ip.WRIST_START_M + 0.001]
        self.assertTrue(far)
        self.assertEqual(max(abs(th) for th in far), 0.0)
        self.assertGreater(max(abs(th) for ph, _, th in seen if ph in ip.WRIST_SEARCH_PHASES), 5.0)

    def test_the_wrist_rolls_within_10_degrees_while_pushing_and_holds_the_angle_it_went_in_at(self):
        seen = []
        watch = lambda a, c, arm: seen.append((c.phase, a._wrist["theta"], a._wrist["held"]))
        attempt = []
        self.run_attempt("insert", keyway_mm=(-2.5, 2.0), watch=watch, attempt_out=attempt, key_source="holster")
        rolling = [th for ph, th, held in seen if ph in ip.WRIST_SEARCH_PHASES and not held]
        self.assertLessEqual(max(abs(th) for th in rolling), ip.WRIST_SEARCH_DEG + 1e-9)
        self.assertGreater(max(rolling) - min(rolling), ip.WRIST_SEARCH_DEG)       # it swings both ways
        held = {round(th, 6) for ph, th, h in seen if ph == "key_seat"}
        self.assertEqual(len(held), 1)
        self.assertAlmostEqual(next(iter(held)), attempt[0].check["wrist_at_entry_deg"], places=1)
        for pose in ("touch_wrist+", "touch_wrist-", "seated_wrist+", "seated_wrist-"):
            self.assertIn(pose, [r["pose"] for r in attempt[0].check["plan"]])
        self.assertEqual(attempt[0].check["wrist_search_deg"], [-ip.WRIST_SEARCH_DEG, ip.WRIST_SEARCH_DEG])

    def test_standing_high_the_wrist_search_narrows_to_what_the_arm_can_hold_and_the_key_still_seats(self):
        """Lukas's live run (2026-10-24 13:22) aborted planning lying: -10 deg at the touch needed Joint6 past
        `HOLSTER_J6_SOFT`. On the model that happens from a dog standing 0.39 m over the box's root; now that side of
        the search narrows instead."""
        seen, attempt = [], []
        watch = lambda a, c, arm: seen.append((c.phase, a._wrist["theta"], a._wrist["held"]))
        self.run_attempt("insert", height=0.39, watch=watch, attempt_out=attempt, key_source="holster")
        a = attempt[0]
        low, high = a.check["wrist_search_deg"]
        self.assertEqual(high, ip.WRIST_SEARCH_DEG)
        # Rolled 45 degrees (v3) it narrowed here; holding v4 level the arm holds the whole search at 0.39 m too.
        self.assertGreaterEqual(low, -ip.WRIST_SEARCH_DEG)
        if low > -ip.WRIST_SEARCH_DEG:
            self.assertIn(-low, ip.WRIST_SEARCH_STEPS_DEG)
        rolling = [th for ph, th, held in seen if ph in ip.WRIST_SEARCH_PHASES and not held]
        self.assertGreaterEqual(min(rolling), low - 1e-9)
        self.assertLessEqual(max(rolling), high + 1e-9)
        self.assertTrue(a.check["seated"], a.check)

    def test_the_wrist_s_search_lag_is_not_taken_for_a_load(self):
        """Joint6 runs behind the search's sine on the dog (17 deg, Lukas's run of 2026-10-24 13:34, which aborted as
        "loaded"): the load check leaves it out while searching, and keeps it otherwise."""
        attempt = []
        self.run_attempt("lie", attempt_out=attempt, key_source="holster")
        a = attempt[0]
        self.assertEqual(a._lag_joints("key_in"), (0, 1, 2, 3, 4))
        self.assertEqual(a._lag_joints("key_search"), (0, 1, 2, 3, 4))
        self.assertEqual(a._lag_joints("key_pre"), tuple(range(6)))
        a._wrist["held"] = True
        self.assertEqual(a._lag_joints("key_search"), tuple(range(6)))

    def test_the_holster_grip_holds_the_jaws_short_of_the_shaft_and_the_key_still_seats(self):
        """The dog's holster grip closes `HOLSTER_SHAFT_ALONG_M` short of the shaft (Lukas's photos, 2026-10-24): the
        insertion holds the jaws that far along the bar from the keyway and seats the key; a model with the shaft on
        the jaws' axis puts it that far to the keyway's side, past the spiral."""
        from unittest import mock

        a, _, _, _ = self.run_attempt("insert", keyway_mm=(0.0, 0.0), key_source="holster")
        self.assertEqual((a.shaft_along, a.shaft_up),
                         (ip.HOLSTER_SHAFT_ALONG_M + ip.HOLSTER_KEY_LEFT_M, -ip.HOLSTER_KEY_UP_M))
        self.assertTrue(a.check["seated"])
        with mock.patch.object(ip, "HOLSTER_SHAFT_ALONG_M", 0.0):
            a, _, _, why = self.run_attempt("insert", keyway_mm=(0.0, 0.0), key_source="holster", expect_abort=True,
                                            true_along_m=0.018)
        self.assertFalse(a.check["seated"])

    def test_the_launch_nudges_move_the_key_right_and_up_onto_a_keyway_the_grip_s_model_misses(self):
        """The shaft truly 5 mm nearer the jaws along the bar and 4 mm below them: on the grip's model the key presses
        5 mm left of and 4 mm below the keyway, past the spiral; nudged 5 mm right and 4 mm up it goes straight in."""
        along, up = ip.HOLSTER_SHAFT_ALONG_M - 0.005, -0.004
        a, _, _, _ = self.run_attempt("insert", keyway_mm=(0.0, 0.0), key_source="holster", expect_abort=True,
                                      true_along_m=along, true_up_m=up)
        self.assertFalse(a.check["seated"])
        a, _, _, _ = self.run_attempt("insert", keyway_mm=(0.0, 0.0), key_source="holster", true_along_m=along,
                                      true_up_m=up, key_right_mm=5.0, key_up_mm=4.0)
        self.assertTrue(a.check["seated"])
        self.assertEqual(a.check["searched_r_mm"], 0.0)
        self.assertEqual((a.check["key_right_mm"], a.check["key_up_mm"]), (5.0, 4.0))
        self.assertAlmostEqual(a.shaft_along, along)
        self.assertAlmostEqual(a.shaft_up, up)

    def test_the_holster_key_s_landing_defaults_to_0_and_the_launch_sets_it(self):
        """Lukas, 2026-10-25: "make it insert the key up 2mm and to the left 2mm", "expose this as a launch argument",
        then "undo the -2, 2 default, make it 0,0": from the holster the nudges default to `HOLSTER_KEY_LEFT_M` and
        `HOLSTER_KEY_UP_M` (0); `--key_right_mm -2 --key_up_mm 2` lands it 2 mm left and up; handed in, 0."""
        def made(**kw):
            attempt = []
            self.run_attempt("load", attempt_out=attempt, **kw)
            a = attempt[0]
            return (round(1000 * a.key_right_m, 9), round(1000 * a.key_up_m, 9),
                    round(1000 * a.shaft_along, 9), round(1000 * a.shaft_up, 9))

        along = 1000 * ip.HOLSTER_SHAFT_ALONG_M
        self.assertEqual((ip.HOLSTER_KEY_LEFT_M, ip.HOLSTER_KEY_UP_M), (0.0, 0.0))
        self.assertEqual(made(key_source="holster"), (0.0, 0.0, along, 0.0))
        self.assertEqual(made(key_source="holster", key_right_mm=-2.0, key_up_mm=2.0), (-2.0, 2.0, along + 2.0, -2.0))
        self.assertEqual(made(key_source="holster", key_right_mm=1.0), (1.0, 0.0, along - 1.0, 0.0))
        self.assertEqual(made(key_source="hand"), (0.0, 0.0, 0.0, 0.0))
        from demos.combiner_door.robot import node

        args = node.parse_args(["--task", "insert", "--stage", "insert"])
        self.assertIsNone(args.key_right_mm)
        self.assertIsNone(args.key_up_mm)

    def test_no_spiral_wiggles_where_the_key_touched_and_does_not_move_across(self):
        """`node.py --no_spiral` (alias `--no_wiggle`; Lukas, 2026-10-25: "I want no_wiggle to still wiggle but not apply
        the translational spiralling"): on the keyway the key goes straight in with the wrist's search as before; 1.5 mm
        off it the key stays pressed where it touched, wiggling, never moved across, and the search gives up after
        `PRESS_ONLY_S`."""
        from demos.combiner_door.robot import node

        a, _, _, _ = self.run_attempt("insert", keyway_mm=(0.0, 0.0), key_source="holster", spiral=False)
        self.assertTrue(a.check["seated"])
        self.assertFalse(a.check["spiral"])
        across, thetas = [], []

        def watch(a, command, arm):
            if command.phase == "key_search":
                across.append((a._ins["a"], a._ins["b"]))
                thetas.append(a._wrist["theta"] - a._roll0)

        a, _, _, why = self.run_attempt("insert", keyway_mm=(1.5, -1.0), key_source="holster", spiral=False,
                                        watch=watch, expect_abort=True)
        self.assertFalse(a.check["seated"])
        self.assertIn("no spiral", why)
        self.assertEqual(len(set(across)), 1)                                 # never moved across the face
        self.assertGreater(max(abs(t) for t in thetas), 5.0)                  # still wiggling
        self.assertEqual(a.search_path, [])
        self.assertTrue(node.parse_args(["--task", "insert", "--stage", "insert", "--no_wiggle"]).no_spiral)
        self.assertTrue(node.parse_args(["--task", "insert", "--stage", "insert", "--no_spiral"]).no_spiral)
        self.assertFalse(node.parse_args(["--task", "insert", "--stage", "insert"]).no_spiral)

    def test_the_depth_nudge_puts_the_touch_on_the_lock_s_face_where_the_model_expects_it(self):
        """The lock truly 10 mm further in than the model, or 4 mm nearer: with `key_in_mm` +10 and -4 the key, on the
        keyway, goes straight in and stops where the model has it seated."""
        for true_deeper, nudge_mm in ((ip.LOCK_DEEPER_M + 0.010, 10.0), (ip.LOCK_DEEPER_M - 0.004, -4.0)):
            a, _, _, _ = self.run_attempt("insert", keyway_mm=(0.0, 0.0), key_source="holster", true_deeper_m=true_deeper,
                                          key_in_mm=nudge_mm)
            self.assertAlmostEqual(a.check["contact_off_mm"], 0.0, delta=1.5)
            self.assertTrue(a.check["seated"])
            self.assertAlmostEqual(a.lock_deeper, true_deeper)

    def test_seated_from_the_holster_the_wrist_alone_turns_the_key_45_degrees_commanded_past_it(self):
        """Lukas, 2026-10-24: "after inserting, try rotating 45 degrees before backing away", with the key taken to be
        on the roll's axis: Joint6 alone turns, clockwise, from the angle the key went in at. Then, 2026-10-25: the dog's
        45 fell short, "if it is commanded 45, lets command it 50 to compensate", then 10 past (`UNLOCK_OVERTURN_DEG`)."""
        seen = {}
        watch = lambda a, c, arm: seen.setdefault(c.phase, []).append(np.array(arm.q))
        a, phases, _, _ = self.run_attempt("insert", key_source="holster", watch=watch)
        commanded = ip.kg.ARM_LEVER.end_deg + ip.UNLOCK_OVERTURN_DEG
        self.assertEqual(a.check["turn_deg"], commanded)
        self.assertAlmostEqual(a.check["turned_deg"], a.check["wrist_at_entry_deg"] + commanded, delta=1.5)
        moved = np.abs(seen["key_turn"][-1] - seen["key_seat"][-1])
        self.assertGreater(moved[5], commanded - 5.0)
        self.assertLess(moved[:5].max(), 2.0)                     # the wrist alone
        self.assertFalse(a.check["left_in_lock"])

    def test_after_the_turn_the_key_comes_out_turned_and_goes_back_in_the_holster(self):
        """Lukas, 2026-10-24: "make it so that the arm puts the lever back in the holster after unlocking the door"
        ("Pull out at 45 deg"): out of the lock in the shut jaws still turned, back to where the draw ended in joint
        space, the draw undone into the mouth and down the channel to the grip, the jaws open, off the bar and folded;
        nothing handed back."""
        seen, angle = {}, {}

        def watch(a, c, arm):
            seen.setdefault(c.phase, []).append(np.array(arm.q))
            if c.phase in ("key_turn", "key_back") and a._base_s is not None:
                # The key's turn by the encoders, as the attempt measures it (`check["turned_deg"]`).
                _, axes = ip.key_in_hand(a.arm.joints, arm.q, a._base_s, a.shaft_along, a.grip_roll, a.shaft_up)
                _, _, u, t = ip._frames(a._box())
                angle[c.phase] = math.degrees(math.atan2(-(axes[:, 0] @ t), axes[:, 0] @ u))

        a, phases, _, _ = self.run_attempt("insert", key_source="holster", watch=watch)
        after = phases[phases.index("key_turn"):]
        self.assertEqual([p for p in after if p in ("key_turn", "key_back", *ip.RETURN_PHASES, "release")],
                         ["key_turn", "key_back", *ip.RETURN_PHASES])
        self.assertEqual(phases[-2:], ["stand_settle", "fold"] if phases[-1] != "done" else phases[-2:])
        self.assertNotIn("hand_back", phases)
        self.assertTrue(a.check["reholstered"])
        self.assertTrue(a.check["released"])
        self.assertLess(abs(a.check["returned_mm"]), 3.0)              # the jaws back at the grip, along the axis
        # Out of the lock still turned: the key as it was at the end of the turn, all through `key_back`.
        self.assertGreater(angle["key_turn"], 40.0)
        self.assertLess(abs(angle["key_back"] - angle["key_turn"]), 2.0)
        # Back to the lift at the end of the draw in joint space, then the IK the other way.
        np.testing.assert_allclose(seen["h_return"][-1], seen["h_clear"][-1], atol=3.0)

    def test_the_draw_s_outer_loop_takes_out_a_sagging_arm_s_miss_at_the_exit(self):
        """Lukas's run of 2026-10-24 21:54 UTC: the encoders put the jaws 4.4 mm short of the exit along the holster's
        axis and 4.2 mm off it after the hold. With the shoulder drooping 1.5 degrees through the draw, the loop still
        has them within 1 mm of the plan's exit at the end of `h_out`, and the lift takes the key away from there."""
        from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6

        ends = {}
        sag = np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0])

        def watch(a, command, arm):
            if command.phase in ip.DRAW_LOOP_PHASES:
                arm.q = arm.q + sag
                q = d1_ik.from_servo_deg(arm.q)[0]
                pos, rot = d1_ik.tool_pose(a.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
                ends[command.phase] = pos + rot @ np.asarray(JAW_CENTRE_LINK6)

        a, phases, _, _ = self.run_attempt("lie", key_source="holster", watch=watch)
        self.assertLess(np.linalg.norm(ends["h_out"] - a._draw["exit"]), 0.001)
        self.assertGreater(np.linalg.norm(a.check["draw_loop_mm"]), 2.0)          # it did have something to take out
        self.assertIn("h_clear", phases)
        self.assertLess(phases.index("h_clear"), phases.index("unload"))
        self.assertGreater(np.linalg.norm(ends["h_clear"] - ends["h_out"]), 0.030)

    def test_after_putting_the_key_back_it_folds_then_stands_with_the_arm_held_folded(self):
        """The run of 2026-10-24 22:19 UTC: the stand took the arm to the carry pose over the head, and the fold from
        there dragged the fingers through the lidar's box. Now Joint1 comes home with the elbow open, the elbow
        shuts, and the stand and the last fold hold the fold."""
        sent = {}
        watch = lambda a, c, arm: sent.setdefault(c.phase, []).append(
            None if c.arm_servo_deg is None else np.array(c.arm_servo_deg))
        a, phases, arm, _ = self.run_attempt("insert", key_source="holster", watch=watch)
        self.assertLess(phases.index("h_unreach"), phases.index("h_unturn"))
        self.assertLess(phases.index("h_fold"), phases.index("stand_up"))
        fold = np.asarray(plan.FOLD_SERVO_DEG)
        for phase in ("stand_up", "stand_settle", "fold"):
            for goal in sent[phase]:
                if goal is not None:
                    np.testing.assert_allclose(goal, fold, atol=0.5, err_msg=phase)
        np.testing.assert_allclose(arm.q, fold, atol=2.0)

    def test_a_sagging_arm_is_lined_up_with_the_mouth_before_the_key_goes_in(self):
        """Lukas's run of 2026-10-24 22:53 UTC: the key reached the channel 3.6 mm toward the mouth's top wall. With
        the shoulder drooping 1.5 degrees on the way back, the hold at the exit (`h_align`) has the jaws within 1 mm of
        it, and the loop keeps them within 1 mm of the way into the mouth and 2 mm of the channel's axis."""
        from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6

        seen = {}
        sag = np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0])

        def watch(a, command, arm):
            if command.phase in ip.DRAW_LOOP_PHASES + ip.RETURN_LOOP_PHASES:
                arm.q = arm.q + sag
                q = d1_ik.from_servo_deg(arm.q)[0]
                pos, rot = d1_ik.tool_pose(a.arm.joints, q, body="Link6", offset=(0.0, 0.0, 0.0))
                seen.setdefault(command.phase, []).append(pos + rot @ np.asarray(JAW_CENTRE_LINK6))

        a, phases, _, _ = self.run_attempt("insert", key_source="holster", watch=watch)
        self.assertTrue(a.check["reholstered"])
        self.assertLess(np.linalg.norm(seen["h_align"][-1] - a._draw["exit"]), 0.001)
        d = a._draw

        out = d["axes"]["out"]

        def off_entry(p):
            """How far `p` is off the way into the mouth, the exit to the axis."""
            f = float(np.clip((p - d["exit"]) @ (d["out"] - d["exit"]) / np.sum((d["out"] - d["exit"]) ** 2), 0.0, 1.0))
            return float(np.linalg.norm(p - d["exit"] - f * (d["out"] - d["exit"])))

        def across(p):
            """How far `p` is across the channel's axis (along it is the holster's business: the cup stops the key)."""
            r = p - d["grip"]
            return float(np.linalg.norm(r - (r @ out) * out))

        self.assertLess(max(off_entry(p) for p in seen["h_enter"][len(seen["h_enter"]) // 2:]), 0.001)
        # Down the channel the loop follows the sag as it changes along the slide, within 2 mm (the channel's walls are
        # 0.4 mm off the bar).
        self.assertLess(max(across(p) for p in seen["h_back"]), 0.002)

    def test_a_lagging_arm_on_the_way_back_does_not_abort(self):
        """The run of 2026-10-24 22:24 UTC aborted in `h_back` on Joint6 20-24 degrees behind its moving target."""
        def lag(a, command, arm):
            if command.phase in ("h_enter", "h_back"):
                arm.q = arm.q + np.array([0.0, 0.0, 0.0, 0.0, 0.0, 15.0])

        a, phases, _, _ = self.run_attempt("insert", key_source="holster", watch=lag)
        self.assertTrue(a.check["reholstered"])
        self.assertNotIn("h_letgo", phases)

    def test_the_dog_s_draw_takes_the_bar_4_mm_along_so_the_shaft_is_2_mm_off_the_jaws_axis(self):
        """Lukas's photos of 2026-10-25: the fingers past the bar's end, the jaws closing 6 mm short of the shaft; then
        the fingertips 3.5 mm short of the bar's back face. The draw's grip is `DOG_GRIP_ALONG_M` along the bar and
        `DOG_GRIP_ONTO_M` further onto it. Then Lukas: the grip "2mm further up its axis (away from the holster)": 4 mm
        along, and the insertion holds the shaft 2 mm along the bar from the jaws' axis."""
        attempt = []
        self.run_attempt("load", key_source="holster", attempt_out=attempt)
        a = attempt[0]
        model = ip.holster.draw_plan()
        np.testing.assert_allclose(a._draw["grip"] - model["grip"], ip.DOG_GRIP_ALONG_M * model["axes"]["u"]
                                   - ip.DOG_GRIP_ONTO_M * model["axes"]["n"], atol=1e-9)
        np.testing.assert_allclose(model["axes"]["out"], -model["axes"]["u"], atol=1e-9)  # up the axis: back along u
        self.assertAlmostEqual(ip.DOG_GRIP_ALONG_M, 0.004)
        self.assertAlmostEqual(ip.HOLSTER_SHAFT_ALONG_M, 0.002)

    def test_the_door_pull_follows_the_key_put_back_from_a_new_look_after_backing_up(self):
        """`node.py --task insert --stage insert --then_door pull` (Lukas, 2026-10-25: "can you now chain this into the
        door handle pulling?"; then, the door swinging ~10 degrees open once unlocked: "it should back up to what is
        shown in the screenshot so it sees the qrcode and then commence the door pull like usual so it rescans for the
        door first"): the key in, turned and back in the holster, the dog stood and folded, it backs up folded to where
        the box's root is `approach.STAGE_ROOT_M` ahead, looks again, and pulls the door from that look."""
        folded = []

        def watch(a, command, arm):
            if isinstance(a, approach.ApproachAttempt):
                folded.append(float(np.max(np.abs(np.asarray(arm.q) - plan.FOLD_SERVO_DEG))))

        a, phases, arm, _ = self.run_attempt("insert", key_source="holster", watch=watch, then_door="pull",
                                             max_steps=14000)
        self.assertIsNotNone(self.door, f"no door pull after the insertion: {self.looks}")
        check = a.unlock_check()
        self.assertTrue(check["verified"], check)
        names = [p.name for p in plan.attempt_phases("pull", plan.scaled_timing(1.0, 30.0))]
        self.assertEqual(phases[-len(names):], names)
        self.assertLess(phases.index("h_fold"), len(phases) - len(names))     # the key back before the walk back
        self.assertEqual(self.walks[0].script.phases[0].name, "pause")       # stands first, for classic walk
        self.assertLess(max(folded), 1.0)                                    # folded through the walks
        # The staging pose is the door pull's stance itself since 2026-10-25 (`approach.STAGE_ROOT_M`): little or no
        # walk in from the look.
        self.assertLessEqual(self.door.look_fix.walk_in_m, plan.WALK_IN_RANGE_M[1])
        reanchor = [e for e in self.door.events if e.get("phase") == "reanchor"]
        self.assertTrue(reanchor and reanchor[0]["ok"], reanchor)
        self.assertTrue(self.door._pull is not None and self.door._pull["done"])

    def test_locking_the_key_goes_in_turned_seats_and_turns_back(self):
        """Lukas, 2026-10-25: "insert it at -45 degrees since it is already unlocked and then push the key in while
        wiggling it ... then holding this position begin rotating from -45 to 0". The door shut: the key in at the unlock's
        45 (clockwise, `geometry.Lever`), the wiggle about it, bottoms seated (its command pressing on to the press's
        depth), and turns back to level there; then into the holster."""
        a, phases, arm, _ = self.run_attempt("insert", key_source="holster", lock_ajar_deg=0.0, vision=True,
                                             max_steps=14000)
        check = a.lock_check()
        self.assertTrue(check["verified"], check)
        self.assertEqual(check["lock_from_deg"], 45.0)
        self.assertLessEqual(abs(check["wrist_at_entry_deg"] - 45.0), 10.0 + 1e-6)   # in at 45, within the wiggle
        self.assertIn(check["pressed"], ("stopped", "at depth"))             # bottomed in the keyway, pressing on
        self.assertAlmostEqual(check["pressed_mm_beyond_face"], 7.7, delta=2.0)     # seated: 7.7 mm beyond the face
        self.assertLess(abs(check["turned_deg"]), 2.0)                        # level: locked
        self.assertTrue(check["reholstered"])
        np.testing.assert_allclose(arm.q, plan.FOLD_SERVO_DEG, atol=plan.FOLD_ARRIVED_DEG + 0.1)

    def test_locking_a_key_that_nothing_stops_is_pressed_to_its_depth_beyond_the_face_and_no_further(self):
        """Lukas, 2026-10-25: "it kept pushing until the arm was fully outstretched, even moving the box. we need a better
        condition to start turning. make it go a certain distance beyond the front face of the box according to the
        lidar". Nothing stops the key here (no lock): it goes to `LOCK_BEYOND_FACE_M` beyond the box's front face, holds
        there and turns; at `--lock_beyond_face_mm` 20, 20 mm."""
        for beyond_mm in (None, 20.0):
            with self.subTest(beyond_mm=beyond_mm):
                a, phases, _, _ = self.run_attempt("insert", key_source="holster", lock_ajar_deg=0.0, lock_there=False,
                                                   vision=True, max_steps=14000, lock_beyond_face_mm=beyond_mm)
                check = a.lock_check()
                want = 1000 * ip.LOCK_BEYOND_FACE_M if beyond_mm is None else beyond_mm
                self.assertEqual(check["pressed"], "at depth")
                self.assertAlmostEqual(check["pressed_mm_beyond_face"], want, delta=3.0)   # the arm's lag along it
                self.assertNotIn("pushing_shut", [e.get("phase") for e in a.events])
                self.assertIn("key_turn", phases)

    def test_locking_a_door_ajar_presses_beyond_the_face_the_lidar_saw(self):
        """The door 8 deg ajar on its spring, the spring weaker than the press (`lock_soft`): the lidar's face is the
        ajar door's, and the key goes `LOCK_BEYOND_FACE_M` beyond it (the door 0.5 deg nearer shut) and turns; pressed up
        to 100 mm beyond, it pushes the door onto its frame (85 mm, the lock's 8 deg of arc) and turns there."""
        a, _, _, _ = self.run_attempt("insert", key_source="holster", lock_ajar_deg=8.0, lock_soft=True, vision=True,
                                      max_steps=20000)
        check = a.lock_check()
        self.assertEqual(check["pressed"], "at depth")
        self.assertGreater(self.door_deg, 7.0)                                # not pushed shut by a 12 mm press
        a, _, _, _ = self.run_attempt("insert", key_source="holster", lock_ajar_deg=8.0, lock_soft=True, vision=True,
                                      max_steps=20000, lock_beyond_face_mm=100.0)
        check = a.lock_check()
        self.assertTrue(check["verified"], check)
        self.assertEqual(check["pressed"], "stopped")
        self.assertLess(self.door_deg, 1.0)                                   # on its frame, or as good as

    def test_no_door_pull_when_the_key_did_not_turn_far_enough(self):
        a, _, _, _ = self.run_attempt("insert", key_source="holster", turn_deg=0.0, then_door="pull")
        self.assertIsNone(self.door)
        check = a.unlock_check()
        self.assertFalse(check["verified"])
        self.assertIn("under the 30", check["why"])

    def test_a_door_the_look_saw_turned_is_walked_square_again_after_the_refit(self):
        """Lukas's run of 2026-10-24 23:41 UTC: the refit after the walk up turned the door 2.2 degrees more than the look,
        and the range check aborted at 4.0 off square. Here the look sees the door 5 degrees off and the range check's
        limit is 2.5 (the harness's walk up ends ~1.8 toward the true door, so its refit leaves ~3 off): one more walk
        up to the refitted stance, then on; with no walk up again, the abort."""
        from unittest import mock

        with mock.patch.object(ip.kp, "YAW_LIMIT_DEG", 2.5):
            a, phases, _, _ = self.run_attempt("lie", look_turn_deg=-5.0)
            rewalk = [e for e in a.events if e.get("phase") == "rewalk"]
            self.assertEqual(len(rewalk), 1, [e for e in a.events if e.get("phase") in ("refit", "reanchor", "rewalk")])
            self.assertIn("deg off square", rewalk[0]["why"])
            self.assertEqual(phases.count("walk_in"), 2)
            self.assertLess(phases.index("walk_settle"), phases.index("lie_down"))
            self.assertTrue([e for e in a.events if e.get("phase") == "reanchor"][-1]["ok"])
            with mock.patch.object(ip, "INSERT_REWALKS", 0):
                _, _, _, why = self.run_attempt("lie", look_turn_deg=-5.0, expect_abort=True)
        self.assertIn("after the walk in", why)

    def test_told_to_leave_the_key_it_lets_go_in_the_lock_and_backs_away(self):
        a, phases, _, _ = self.run_attempt("insert", key_source="holster", reholster=False)
        self.assertEqual([p for p in phases if p in ("key_seat", "key_turn", "release", "key_back")],
                         ["key_seat", "key_turn", "release", "key_back"])
        self.assertFalse(set(ip.RETURN_PHASES) & set(phases))
        self.assertIsNone(a.check.get("reholstered"))

    def test_an_abort_in_the_channel_lets_go_there_and_one_before_it_hands_the_key_back(self):
        """Stopped with the key going down the channel: the jaws open where they are and slide off the bar. Stopped
        before the mouth: the key is still in the hand, handed back. Stopped coming out of the lock: let go in it."""
        for phase, first, handed_back in (("h_back", "h_letgo", False), ("h_return", None, True),
                                          ("key_back", "key_letgo", False)):
            def stop(a, command, arm, phase=phase):
                if command.phase == phase and a.script.elapsed_s > 0.5:
                    a._blocked = f"test: stopped in {phase}"

            a, _, _, why = self.run_attempt("insert", key_source="holster", expect_abort=True, watch=stop)
            self.assertEqual(why, f"test: stopped in {phase}")
            a._blocked = ""
            a.recover()
            names = [p.name for p in a.script.phases]
            if first is not None:
                self.assertEqual(names[0], first, phase)
            self.assertEqual("hand_back" in names, handed_back, (phase, names))

    def test_no_turn_asked_releases_the_key_where_it_went_in(self):
        seen = {}
        watch = lambda a, c, arm: seen.setdefault(c.phase, []).append(np.array(arm.q))
        a, _, _, _ = self.run_attempt("insert", key_source="holster", watch=watch, turn_deg=0.0)
        self.assertEqual(a.check["turn_deg"], 0.0)
        self.assertIsNone(a.check["turned_deg"])
        self.assertTrue(a.check["seated"])
        self.assertLess(np.abs(seen["key_back"][0] - seen["key_seat"][-1]).max(), 2.0)

    def test_an_abort_while_turning_lets_go_and_leaves_the_key_in_the_lock(self):
        """Turned, the key cannot come out: the recovery opens the jaws where they are and backs away empty, with no
        hand-back."""
        def stop(a, command, arm):
            if command.phase == "key_turn" and a.script.elapsed_s > 1.0:
                a._blocked = "test: the lock held"

        a, phases, _, why = self.run_attempt("insert", key_source="holster", expect_abort=True, watch=stop)
        self.assertEqual(why, "test: the lock held")
        a._blocked = ""
        a.recover()
        names = [p.name for p in a.script.phases]
        self.assertEqual(names[:2], ["key_letgo", "key_out"])
        self.assertNotIn("hand_back", names)
        self.assertTrue(a.check["left_in_lock"])

    def test_a_draw_that_falls_short_is_recorded_and_the_key_goes_on_to_the_lock(self):
        """The encoders short of the dog's draw after `h_out` (Lukas's run of 2026-10-24 06:10 UTC: 96 mm of 100, the
        lever 2 cm clear): recorded, no abort."""
        def short(a, command, arm):
            if command.phase == "h_out" and "short" not in a._draw:
                a._draw = dict(a._draw, grip=a._draw["grip"] + 0.02 * a._draw["axes"]["out"], short=True)

        a, phases, _, _ = self.run_attempt("insert", key_source="holster", watch=short)
        self.assertLess(a.check["drawn_mm"], DRAWN_PLAN_MM - 15.0)
        self.assertIn("key_go", phases)

    def test_with_the_shaft_off_the_jaws_axis_the_wrist_rolls_about_the_shaft(self):
        """The holster's grip: the wrist's search turns the hand about the key's shaft, not Joint6's axis, so the shaft
        stays on the keyway through the roll."""
        import math

        attempt = []
        self.run_attempt("lie", attempt_out=attempt, key_source="holster")
        a = attempt[0]
        base_s = a._look_base
        to_base = plan.invert(base_s)
        into, bar = a.key_axes()
        box = a._box()
        from demos.cup.pick_demo.grasp import _seeds, solve_link6
        from demos.unifp import decoupled_arm

        point = plan.transform(to_base, [ip.jaw_for(box, ip.TRAVEL_M, 0.0, 0.0, a.shaft_along, None, a.shaft_up)])[0]
        q0 = solve_link6(a.arm.joints, a.arm.links, point, decoupled_arm.hand_rotation(
            to_base[:3, :3] @ into, to_base[:3, :3] @ bar, 0.0, a.grip_roll), _seeds(point, [a._q_pre]),
            a.arm.offset, a.arm.soft).q
        shaft = lambda q: ip.key_in_hand(a.arm.joints, d1_ik.to_servo_deg(q), base_s, a.shaft_along, a.grip_roll,
                                         a.shaft_up)[0]
        for deg in (5.0, -5.0):
            row = a._rolled(q0, deg, "touch", (ip.TRAVEL_M, 0.0, 0.0), base_s)
            self.assertIsNotNone(row)
            jaw_only = np.asarray(q0, dtype=float).copy()
            jaw_only[5] -= math.radians(deg)
            # Joint6 alone swings the shaft round the jaws' axis, a chord of the offset
            self.assertAlmostEqual(float(np.linalg.norm(shaft(jaw_only) - shaft(q0))),
                                   2 * math.hypot(a.shaft_along, a.shaft_up) * math.sin(math.radians(abs(deg)) / 2),
                                   delta=1e-4)
        a._wrist["theta"] = 5.0
        a._ins.update(off=ip.TRAVEL_M, a=0.0, b=0.0)
        rot = decoupled_arm.hand_rotation(to_base[:3, :3] @ into, to_base[:3, :3] @ a.key_bar(), 0.0, a.grip_roll)
        target = plan.transform(to_base, [a._target()])[0]
        rolled = solve_link6(a.arm.joints, a.arm.links, target, rot, [q0], a.arm.offset, a.arm.soft).q
        self.assertLess(np.linalg.norm(shaft(rolled) - shaft(q0)), 0.5e-3)

    def test_the_wrist_s_roll_is_joint6_alone_as_the_ik_has_it(self):
        """`_rolled` puts the hand where the IK does for the rolled key, without the IK (whose failing seeds cost the
        dog ~20 s of planning lying down)."""
        import math

        from demos.cup.pick_demo.grasp import _seeds, solve_link6
        from demos.unifp import decoupled_arm

        attempt = []
        self.run_attempt("lie", attempt_out=attempt, key_source="holster")
        a = attempt[0]
        base_s = a._look_base
        to_base = plan.invert(base_s)
        into, bar = a.key_axes()
        box = ip.perception.site_pose(a.fix.site)
        point = plan.transform(to_base, [ip.jaw_for(box, ip.TRAVEL_M, 0.0, 0.0)])[0]
        for deg in (10.0, -5.0):
            rot = decoupled_arm.hand_rotation(to_base[:3, :3] @ into, to_base[:3, :3] @ (
                decoupled_arm.axis_angle(into, math.radians(deg)) @ bar), 0.0, a.grip_roll)
            ik = solve_link6(a.arm.joints, a.arm.links, point, rot, _seeds(point, [a._q_pre]), a.arm.offset, a.arm.soft)
            unrolled = solve_link6(a.arm.joints, a.arm.links, point, decoupled_arm.hand_rotation(
                to_base[:3, :3] @ into, to_base[:3, :3] @ bar, 0.0, a.grip_roll), _seeds(point, [a._q_pre]),
                a.arm.offset, a.arm.soft)
            row = a._rolled(unrolled.q, deg, "touch")
            self.assertIsNotNone(row)
            self.assertAlmostEqual(row["j6_rad"], float(ik.q[5]), delta=0.005)
            np.testing.assert_allclose(unrolled.q[:5], ik.q[:5], atol=0.005)

    def test_jaws_closing_on_nothing_at_the_holster_abort_and_back_off_the_way_they_came(self):
        attempt = []
        a, phases, arm, why = self.run_attempt("insert", key=False, expect_abort=True, attempt_out=attempt,
                                               key_source="holster")
        self.assertIn("at the holster", why)
        a = attempt[0]
        a.recover()
        self.assertEqual([p.name for p in a.script.phases][:4], ["h_letgo", "h_off", "h_raise", "h_unreach"])
        self.assertIn("fold", [p.name for p in a.script.phases])

    def test_the_load_stage_draws_the_key_and_hands_it_back(self):
        a, phases, arm, _ = self.run_attempt("load", key_source="holster")
        self.assertIn("h_draw", phases)
        self.assertIn("hand_back", phases)
        self.assertNotIn("key_go", phases)


class InsertVizTests(unittest.TestCase):
    def test_the_scene_draws_the_keyway_the_held_key_and_the_search(self):
        from demos.combiner_door import mid360
        from demos.combiner_door.robot import viz
        from demos.combiner_key.robot import insert_viz
        from demos.combiner_key.tests.test_key_robot import KeyVizTests, _Marker

        KeyVizTests.setUp(self)
        attempts = []
        InsertAttemptTests("test_confirmations_name_real_phases").run_attempt("insert", attempt_out=attempts)
        attempt = attempts[0]
        attempt.check["released"] = False                      # draw it as if still held
        items = viz.scene_markers(_Marker, 0, attempt.fix, attempt, "status", "note", mid360.NO_READING,
                                  door_task=False, extra=insert_viz.extra_for(attempt))
        spaces = {m.ns for m in items}
        self.assertTrue({"keyway", "key_held", "key_search", "key_stance"} <= spaces, spaces)
        self.assertIn("SEATED", insert_viz.status_text(attempt))


if __name__ == "__main__":
    unittest.main()

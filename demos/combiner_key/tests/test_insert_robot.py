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
from demos.combiner_door.robot import plan
from demos.combiner_door.robot.arm_link import SimulatedArm
from demos.combiner_door.tests.test_robot import face_scan, look
from demos.combiner_key.robot import insert_plan as ip
from demos.combiner_key.robot import key_plan as kp
from demos.combiner_key.tests.test_key_robot import ODOM_SCALE, box_for_lock, walk_classic

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


def lock_ahead(walk_m: float = 0.30, right_m: float = ip.INSERT_LOCK_RIGHT_M):
    """(ahead, left) of the lock's point on the door from the look for a walk up of `walk_m` to the insertion's stance."""
    return ip.INSERT_SEATED_AHEAD_M - ip.SEATED_OFF_DOOR_M + walk_m, -right_m


class InsertAttemptTests(unittest.TestCase):
    def run_attempt(self, stage, keyway_mm=(1.5, -1.0), height=0.353, lock=None, key=True, max_steps=6000,
                    drop_m=ip.LIE_DROP_EST_M, expect_abort=False, watch=None, attempt_out=None, slip_in=None,
                    lock_there=True, dry=False, vision=False, calibration_path=None, hidden=()):
        """A dry insertion attempt. The keyway is `keyway_mm` (along the key, up its top side) off where the box's model
        has it; the arm's key stops on the lock's face unless its shaft is within `CLEAR_M` of it. `key`: a key between
        the jaws (an int: from that try of the grip on). `slip_in`: the key falls out in that phase. `vision`: the wrist
        camera's frames through `key_view`, rendered from where the arm truly holds it, the bezel round the true keyway;
        `hidden`: the looks (`lock_look`, `key_view`) that do not see it (the held key and jaws in the way), or "all".
        Returns (attempt, phases, arm, abort reason or "")."""
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = box_for_lock(*(lock or lock_ahead()), 180.0, height)
        fix = ip.make_insert_fix(base, *look(base, box))
        ok, text = fix.in_range()
        self.assertTrue(ok, text)
        attempt = ip.InsertAttempt(fix, stage, dry=dry, calibration_path=calibration_path)
        if attempt_out is not None:
            attempt_out.append(attempt)
        arm = SimulatedArm(d1_hardware.REST_SERVO_DEG)
        joints = attempt.arm.joints
        true_base, rng, moving = base, np.random.default_rng(5), (0.0, 0.0, 0.0)
        body, posture, phases, released = STAND_BODY_M, "", [], False
        a0, b0 = (v / 1000.0 for v in keyway_mm)
        for step in range(max_steps):
            dropped = STAND_BODY_M - body
            lowered = true_base.copy()
            lowered[2, 3] -= dropped
            sensed = base.copy()
            scan = None
            if attempt.phase in ("walk_settle", "lie_settle"):
                scan = (base, face_scan(lowered, box, rng))      # the lidar sees from where the base really is
            slipped = slip_in is not None and slip_in in phases
            there = bool(key) and attempt._grips + 1 >= int(key) and not slipped
            arm.stall_units = BAR_UNITS if (there and not released) else None
            obs = plan.Observation(0.1 * step, sensed, 0.0, arm.q.copy(), 0.0, scan=scan, gripper_units=arm.gripper,
                                   body_height_m=body)
            command = attempt.step(obs)
            if command.finished:
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
                arm.step(command.arm_servo_deg, command.gripper_units, command.speed_deg_s)
                attempt.sent(command)
            released = released or command.phase == "release"
            if there and not released and lock_there:
                self._keyway(joints, arm, lowered, box, a0, b0)
            if vision and attempt.wants_keyhole(0.1 * step):
                hide = hidden == "all" or attempt.phase in hidden
                attempt.add_keyhole(self._look_at_keyhole(attempt, arm, lowered, base, box, a0, b0, hide))
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
    def _keyway(joints, arm, base_true, box, a0, b0) -> None:
        """The lock's face and its tight keyway: the held key's shaft cannot pass the face more than `CLEAR_M` off the
        keyway, nor its back face pass seated; once in, the keyway's sides hold it within `CLEAR_M` across. The arm is
        put back where the key stops (rigid, frictionless)."""
        origin, _ = ip.key_in_hand(joints, arm.q, base_true)
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
        if inside and miss > CLEAR_M:
            push = -(1.0 - CLEAR_M / miss) * (across[0] * u + across[1] * t)      # back against the keyway's side
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
        attempt, _, _, _ = self.run_attempt("insert", keyway_mm=(20.0, -15.0), vision=True, hidden=("key_view",))
        look, vision = attempt.check["look"], attempt.check["vision"]
        self.assertGreaterEqual(look["found"], ip.VIEW_MIN_FRAMES, look)
        self.assertLess(math.dist(look["moved_mm"], (20.0, -15.0)), 4.0, look)
        self.assertEqual(vision["found"], 0, vision)
        self.assertTrue(attempt.check["seated"], attempt.check)

    def test_the_first_look_reaches_the_offset_seen_on_the_dog(self):
        """The bezel where the dog's two runs on 2026-10-22/23 saw it from the fold: 19 mm along the key and 74 mm down
        its top side from the model's place (~66 mm right and 39 mm down), past the first look's old 80 mm window. From
        the fold, that far to the side, the look alone leaves a few mm (its depth is the model's), more than the touch's
        4 mm spiral was found to take up here; `key_view`, close, the rest."""
        attempt, _, _, _ = self.run_attempt("insert", keyway_mm=(19.0, -74.0), vision=True)
        look = attempt.check["look"]
        self.assertGreaterEqual(look["found"], ip.VIEW_MIN_FRAMES, look)
        self.assertLess(math.dist(look["moved_mm"], (19.0, -74.0)), 5.0, look)
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
            attempt, _, _, _ = self.run_attempt("insert", keyway_mm=(1.0, -0.5), vision=True)
        # The first look moves it out by its size from 0.48 m (2-3% long: ~1 cm too far), `key_view` close up the rest.
        moved = attempt.check["look"].get("out_mm", 0.0) + attempt.check["vision"].get("out_mm", 0.0)
        self.assertAlmostEqual(moved, -40.0, delta=4.0, msg=(attempt.check["look"], attempt.check["vision"]))
        self.assertTrue(attempt.check["seated"], attempt.check)

    def test_without_the_camera_a_door_further_than_the_model_is_found_by_the_touch(self):
        """30 mm further and nothing seen: the key goes on past the model's seated (`IN_SLACK_M`), meets the lock's
        face, takes it as the face (past seated), and searches and seats from there."""
        from unittest import mock

        with mock.patch(f"{__name__}.SIT_BACK_M", 0.07):
            attempt, _, _, _ = self.run_attempt("insert", keyway_mm=(1.5, -1.0))
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
                arm.step(command.arm_servo_deg, command.gripper_units, command.speed_deg_s)
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

    def test_lying_down_uses_the_measured_sit_back_not_the_lidar(self):
        """Lying, the held key shadows the door (Lukas: "we cant trust the lidar after sitting down"): no lidar refit;
        the 4 cm of sit-back the odometry did not see is added, and the key still seats."""
        attempt, phases, _, _ = self.run_attempt("insert")
        lying = [e for e in attempt.events if e.get("phase") == "refit_lying"]
        self.assertEqual(len(lying), 1)
        self.assertFalse(lying[0]["used"])
        self.assertAlmostEqual(lying[0]["sit_back_added_mm"], 1000 * SIT_BACK_M, delta=1.0)
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
        attempt = []
        self.run_attempt("load", attempt_out=attempt)
        names = {p.name for p in ip.stage_phases("insert", attempt[0])}
        self.assertTrue(set(ip.CONFIRM_BEFORE) <= names, set(ip.CONFIRM_BEFORE) - names)
        self.assertTrue({"load", "grip"} <= names)



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

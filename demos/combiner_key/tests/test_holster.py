"""Lukas's key holster, without a simulator: the saved registration (where the holster and the key in it are on the
D1's base), the colliders against the key, and the draw on the CPU arm model (`holster.check`)."""
from __future__ import annotations

import math
import unittest

import numpy as np

from demos.combiner_key import geometry as kg
from demos.combiner_key import holster
from demos.unifp.decoupled_arm import hand_rotation


class Registration(unittest.TestCase):
    def setUp(self):
        self.record = holster.load()
        self.key = holster.key_in_d1_base(self.record)

    def test_the_key_frame_is_a_rotation(self):
        rot = self.key[:3, :3]
        self.assertTrue(np.allclose(rot.T @ rot, np.eye(3), atol=1e-6))
        self.assertAlmostEqual(float(np.linalg.det(rot)), 1.0, places=5)

    def test_the_key_is_the_mirrored_one_the_simulation_uses(self):
        self.assertTrue(self.record["key_mirrored"])
        self.assertTrue(kg.KEY_MIRRORED)
        # The mirrored key's (u, n, t) is left-handed, as `geometry.KEY_AXES_LINK6` has it.
        axes = holster.key_axes(self.key)
        rows = np.asarray(kg.KEY_AXES_LINK6)
        self.assertAlmostEqual(float(np.cross(axes["u"], axes["n"]) @ axes["t"]),
                               float(np.cross(rows[0], rows[1]) @ rows[2]), places=5)
        self.assertLess(self.record["bar_fit_mm"]["median"], 0.1)

    def test_the_holster_is_on_the_right_beside_the_base_and_the_key_slides_out_forward_and_up(self):
        axes = holster.key_axes(self.key)
        self.assertLess(axes["origin"][1], -0.06)                      # the robot's right of the arm's axis
        out = axes["out"]
        self.assertGreater(out[0], 0.7)
        self.assertAlmostEqual(math.degrees(math.asin(float(out[2]))), 40.8, delta=0.5)


class V5(unittest.TestCase):
    """Lukas's v5 holster (2026-10-24, `holster.register_v4` on `holster_with_lever2.stl` and `lever2.stl`): v4's on the
    Go2's rails with a mouth, the lever (v4's, no platform) 15 mm further out along the axis."""

    def setUp(self):
        self.record = holster.load()

    def test_the_lever_sits_15_mm_further_out_than_v4_s_on_the_holster_s_axes(self):
        self.assertEqual(self.record["design"], "v5")
        out = holster.holster_in_d1_base(self.record)[:3, :3] @ np.asarray(holster.SLIDE_OUT_KEY)
        v4 = np.asarray(self.record["replaced"]["lever_in_d1_base"])
        measured = holster.key_in_d1_base(self.record)
        np.testing.assert_allclose(measured[:3, 3] - v4[:3, 3], 0.015 * out, atol=1e-4)
        np.testing.assert_allclose(measured[:3, :3], v4[:3, :3], atol=1e-6)
        self.assertLess(self.record["lever_measured"]["bar_faces_off_planes_mm"], 0.1)
        self.assertEqual(len(self.record["lever_measured"]["holster_volumes_cm3"]), 3)     # sleeve, strut, mouth

    def test_its_colliders_are_its_stl(self):
        """`KEY_BOXES`'s bar and shaft against `lever_v4.stl`'s extent, to 0.1 mm (the bar's rounded end, its square
        collider's corner, and the wards' square stand-in aside)."""
        from demos.combiner_key.asset import read_stl

        tris = np.asarray(read_stl(holster.LEVER_V4_PATH)).reshape(-1, 3)
        lo, hi = holster.KEY_BOXES["bar"]
        bar = tris[tris[:, 0] > -0.0005]
        np.testing.assert_allclose(bar.min(0), lo, atol=1e-4)
        np.testing.assert_allclose(bar.max(0), hi, atol=1e-4)
        shaft = tris[tris[:, 0] < -0.0005]
        self.assertAlmostEqual(float(shaft[:, 0].min()), holster.KEY_BOXES["shaft"][0][0], delta=1e-4)
        # No platform since v5 (Lukas: "No platform now"); v4's was v3's 10 mm further out.
        self.assertNotIn("platform", holster.KEY_BOXES)
        v3, v4 = holster.V3_KEY_BOXES, holster.V4_KEY_BOXES
        self.assertAlmostEqual(v4["bar"][1][1] - v4["platform"][1][1], v3["bar"][1][1] - v3["platform"][1][1], places=9)

    def test_the_holster_mesh_holds_the_lever_without_touching_it(self):
        """The holster's mesh (the sleeve, the strut, the rail slider, the mouth) against the lever's, both as saved: no
        point of the lever inside it, and the sleeve within half a millimetre of it all along the bar."""
        try:
            import trimesh
        except ImportError:
            self.skipTest("trimesh")
        from demos.combiner_key.asset import read_stl

        tris = np.asarray(read_stl(holster.HOLSTER_MESH_PATH))
        mesh = trimesh.Trimesh(tris.reshape(-1, 3), np.arange(len(tris) * 3).reshape(-1, 3), process=True)
        lever = np.asarray(read_stl(holster.LEVER_V4_PATH)).reshape(-1, 3)
        to_base = holster.key_in_d1_base(self.record)
        points = lever @ to_base[:3, :3].T + to_base[:3, 3]
        depth = trimesh.proximity.signed_distance(mesh, points)
        self.assertLess(float(depth.max()), 1e-4)                   # positive is inside the holster
        self.assertLess(float(np.min(np.abs(depth))), 5e-4)

    def test_v4_s_registration_and_the_10_23_one_are_kept(self):
        self.assertIn("lever_in_d1_base", self.record["replaced"])
        self.assertNotIn("lever_in_d1_base", self.record["replaced"]["replaced"])


class MeshClearance(unittest.TestCase):
    """The arm through the draw and the unfold against the whole holster mesh, its strut and its rail slider as well as
    the sleeve and the mouth (`check` measures their boxes and plates only)."""

    def test_the_arm_stays_as_far_from_the_strut_and_rail_as_from_the_sleeve(self):
        try:
            import trimesh
        except ImportError:
            self.skipTest("trimesh")
        from demos.combiner_key.asset import read_stl
        from position_only.workspace import MOUNT_B, load_urdf

        joints, _ = load_urdf()
        tris = np.asarray(read_stl(holster.HOLSTER_MESH_PATH))
        mesh = trimesh.Trimesh(tris.reshape(-1, 3), np.arange(len(tris) * 3).reshape(-1, 3), process=True)
        mesh.apply_translation(MOUNT_B)
        out = holster.check()
        goals = holster.unfold_stages(out["rest"], out["q_pre"], out["q_above"])
        path = np.concatenate([holster.joint_move(a, b) for a, b in zip([out["rest"], *goals[:-1]], goals)])
        qs = [np.asarray(r["q"]) for r in out["rows"] if r["solved"]] + list(path[::3])
        worst = 1.0
        for q in qs:
            arm = holster._arm_points(joints, q, holster.JAW_OPEN_M)
            points = np.vstack([v for k, v in arm.items() if k != "base_link"])
            worst = min(worst, float(np.abs(trimesh.proximity.signed_distance(mesh, points)).min()))
        sleeve = min(r["arm_to_holster_mm"] for r in out["rows"] if r["solved"]) / 1000.0
        self.assertGreater(worst, min(sleeve, 0.030) - 0.002)


class Colliders(unittest.TestCase):
    """In the holster's frame, the key where it sits: v5's frame there (24.93 mm out along the axis from v3's)."""

    def setUp(self):
        self.key = np.linalg.inv(holster.holster_in_d1_base()) @ holster.key_in_d1_base()
        self.holster_boxes = holster.boxes_in(np.eye(4), holster.HOLSTER_BOXES)

    def _grid(self, lo, hi, n):
        return np.stack(np.meshgrid(*[np.linspace(a, b, n) for a, b in zip(lo, hi)], indexing="ij"), -1).reshape(-1, 3)

    def test_the_holster_s_boxes_stand_off_the_key_s_by_less_than_half_a_millimetre_and_never_overlap_it(self):
        key = holster.boxes_in(self.key, holster.key_boxes())
        gaps = []
        for name, (lo, hi) in holster.HOLSTER_BOXES.items():
            gap = float(holster.box_distance(self._grid(lo, hi, 9), key).min())
            self.assertGreaterEqual(gap, 0.0, name)
            gaps.append(gap)
        self.assertLess(min(gaps), 0.0005)

    def test_the_mouth_s_walls_flare_from_the_channel_s_faces_and_clear_the_key(self):
        """At the channel the opening is the channel's (its four faces, the chamfer ending in its front-bottom corner);
        at the lip it is ~40 mm across; every wall opens at more than 25 degrees; no plate touches the key."""
        key = holster.boxes_in(self.key, holster.key_boxes())
        boxes = holster.HOLSTER_BOXES
        mid_n = holster.BAR_MIDDLE_NT[0]
        faces = {"back": mid_n - boxes["back_wall"][1][0], "front": boxes["front_hook"][0][0] - mid_n,
                 "bottom": -boxes["bottom_lip"][1][2], "top": boxes["top_lip"][0][2]}
        u0, u1 = holster.MOUTH_U_M
        walls_in, walls_lip = holster.mouth_walls(u1), holster.mouth_walls(u0)
        for name, d in faces.items():
            self.assertAlmostEqual(walls_in[name][1], d, delta=0.0006, msg=name)
            slope = math.degrees(math.atan2(walls_lip[name][1] - walls_in[name][1], u1 - u0))
            self.assertGreater(slope, 25.0, name)
        corner = np.array([walls_in["front"][1], -walls_in["bottom"][1]])
        self.assertAlmostEqual(float(corner @ walls_in["chamfer"][0]), walls_in["chamfer"][1], delta=1e-6)
        lip = holster.mouth_section(u0)
        self.assertGreater(lip["top"][1][0] - lip["top"][0][0], 0.035)
        plates = holster.mouth_plates()
        self.assertEqual(len(plates), 5 * holster.MOUTH_SEGMENTS)
        for name, (centre, axes, half) in plates.items():
            gap = float(holster.box_distance(holster.box_points((centre, axes, half)), key).min())
            self.assertGreaterEqual(gap, 0.0002, name)

    def test_the_longer_bar_still_ends_in_the_cup(self):
        far = self.key[:3, :3] @ np.array([0.0075, holster.key_boxes()["bar"][1][1], 0.0]) + self.key[:3, 3]
        cup_end = holster.HOLSTER_BOXES["cup_end"][0][1]
        self.assertTrue(cup_end - 0.001 < far[1] < cup_end, (far[1], cup_end))

    def test_the_key_can_only_slide_out_toward_its_shaft(self):
        lo, hi = holster.key_boxes()["bar"]
        bar = self._grid(lo, hi, 15) @ self.key[:3, :3].T + self.key[:3, 3]
        out = np.asarray(holster.SLIDE_OUT_KEY)
        for direction, should_hit in ((out, False), (-out, True), (np.array([1.0, 0, 0]), True),
                                      (np.array([-1.0, 0, 0]), True), (np.array([0, 0, 1.0]), True),
                                      (np.array([0, 0, -1.0]), True)):
            inside = holster.box_distance(bar + 0.002 * direction, self.holster_boxes) < 0.0
            self.assertEqual(bool(inside.any()), should_hit, direction)


class Draw(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out = holster.check()

    def test_the_grip_is_what_the_control_loop_builds(self):
        axes = holster.key_axes(holster.key_in_d1_base())
        self.assertTrue(np.allclose(holster.grip_rotation(axes), hand_rotation(-axes["n"], axes["u"], 0.0,
                                                                                 holster.GRIP_ROLL), atol=1e-9))

    def test_the_draw_solves_within_the_loop_s_limits_from_the_root_of_the_bar(self):
        """Onto the bar within the loop's 0.9 of every joint's range; out along the axis and through the mouth with
        Joint1 and Joint5 to `DRAW_SOFT_J1_J5`."""
        self.assertTrue(self.out["solved"], self.out["rows"][-1])
        rows = self.out["rows"]
        self.assertEqual(rows[-1]["move"], "clear")
        self.assertAlmostEqual(rows[-1]["f"], 1.0)
        self.assertLessEqual(max(r["range_use_max"] for r in rows if r["move"] in ("pre", "on")), holster.IK_SOFT + 1e-6)
        self.assertLessEqual(max(r["range_use_max"] for r in rows), holster.DRAW_SOFT_J1_J5 + 1e-6)
        self.assertLess(max(r["gravity_use_max"] for r in rows), 0.5)

    def test_the_exit_takes_the_bar_s_far_end_out_past_the_mouth_s_lip_clear_of_its_walls(self):
        rows = self.out["rows"]
        self.assertGreaterEqual(min(r["key_to_mouth_mm"] for r in rows), 0.0)
        exits = [r for r in rows if r["move"] == "exit"]
        # Nearest at its start, the chamfer ~2 mm off the bar's front-bottom edge where the draw along the axis ends.
        self.assertGreater(min(r["key_to_mouth_mm"] for r in exits), 1.0)
        self.assertGreater(exits[-1]["key_to_mouth_mm"], 5.0)
        pts = self.out["points"]
        key = holster.key_in_d1_base()
        moved = key[:3, :3].T @ (pts["exit"] - pts["grip"])                       # the key's move, in its frame
        far_end = holster.key_boxes()["bar"][1][1] + moved[1]                    # along u
        lip = holster.MOUTH_U_M[0] - (np.linalg.inv(holster.holster_in_d1_base()) @ key)[1, 3]
        self.assertLess(far_end, lip - 0.005)

    def test_nothing_touches_on_the_way(self):
        rows = self.out["rows"]
        self.assertGreater(min(r["trunk_proxy_margin_m"] for r in rows), 0.0)
        self.assertGreater(min(r["open_hand_to_key_mm"] for r in rows if r["open_hand_to_key_mm"] is not None), 5.0)
        self.assertGreater(min(r["arm_to_holster_mm"] for r in rows), 5.0)
        self.assertGreater(min(r["key_to_arm_base_mm"] for r in rows), 5.0)
        self.assertGreater(min(r["key_to_arm_links_mm"] for r in rows), 5.0)
        self.assertGreater(self.out["unfold_clearance_mm"], 5.0)
        self.assertGreater(self.out["unfold_trunk_proxy_margin_m"], 0.0)

    def test_the_insertion_s_own_roll_does_not_draw_it(self):
        self.assertFalse(holster.check(roll=-holster.GRIP_ROLL)["solved"])


if __name__ == "__main__":
    unittest.main()


class DogProgram(unittest.TestCase):
    """`robot/holster_dog.py`: the moves it would send the real arm, and its dry run on the simulated one."""

    @classmethod
    def setUpClass(cls):
        from demos.combiner_key.robot import holster_dog

        cls.dog = holster_dog
        cls.moves = holster_dog.plan()

    def test_it_starts_and_ends_folded_and_undoes_what_it_does(self):
        names = [m["name"] for m in self.moves]
        self.assertEqual(names, ["turn", "reach", "lower", "on", "close", "draw", "exit", "hold", "enter", "back", "open",
                                 "off", "raise", "unreach", "fold"])
        fold = np.asarray(self.dog.FOLD_SERVO_DEG)
        self.assertEqual(self.moves[-1]["reverse_of"], "turn")                 # back to where the arm started
        # Joint1 turns at the fold, the elbow opening as it does (`holster.TURN_ELBOW_RAD`)
        self.assertTrue(np.allclose(self.moves[0]["goal"][[1, 3, 4, 5]], fold[[1, 3, 4, 5]], atol=0.05))
        self.assertGreater(abs(self.moves[0]["goal"][2] - fold[2]), 15.0)
        by = {m["name"]: m for m in self.moves}
        for back, out in (("enter", "exit"), ("back", "draw"), ("off", "on"), ("raise", "lower"), ("unreach", "reach")):
            self.assertTrue(np.allclose(by[back]["path"], by[out]["path"][::-1]), back)
        self.assertTrue(np.allclose(by["lower"]["path"][-1], by["on"]["path"][0]))
        self.assertTrue(np.allclose(by["reach"]["path"][0], by["turn"]["goal"]))

    def test_a_joint_move_runs_every_moving_joint_at_the_full_pace(self):
        path = self.dog.move_path({"goal": np.array([60.0, -60.0, 70.0, -40.0, 60.0, 90.0])},
                                  np.asarray(self.dog.FOLD_SERVO_DEG))
        steps = np.abs(np.diff(path, axis=0))
        for j in range(6):
            moving = steps[:, j] > 0.05
            # each joint mostly steps the full 4 degrees a command (its ramps are a command or two)
            self.assertGreaterEqual(np.median(steps[moving, j]), 3.5, j)

    def test_the_streamed_paths_stay_within_the_link_s_speed_and_the_joint_limits(self):
        for m in self.moves:
            if "path" not in m:
                continue
            step = np.abs(np.diff(m["path"], axis=0)).max()
            self.assertLess(step / self.dog.PERIOD_S, 45.0, m["name"])           # arm_link.MAX_SPEED_DEG_S
            self.assertLessEqual(np.abs(m["path"]).max(), 135.0)

    def test_the_jaws_open_round_the_bar_and_shut_to_it(self):
        by = {m["name"]: m for m in self.moves}
        self.assertGreater(by["on"]["gripper"], by["close"]["gripper"])

    def test_at_5_hz_the_steps_double_and_the_moves_take_as_long(self):
        import importlib

        dog = importlib.reload(self.dog)
        try:
            dog.PERIOD_S = 0.2
            slow = {m["name"]: m for m in dog.plan()}
        finally:
            dog = importlib.reload(self.dog)
        fast = {m["name"]: m for m in self.moves}
        for name in ("reach", "draw"):
            step = lambda m: float(np.median(np.abs(np.diff(m["path"], axis=0)).max(axis=1)))
            self.assertAlmostEqual(step(slow[name]) / step(fast[name]), 2.0, delta=0.15, msg=name)
            self.assertAlmostEqual((len(slow[name]["path"]) - 1) * 0.2, (len(fast[name]["path"]) - 1) * 0.1, delta=0.21)
            self.assertLess(step(slow[name]) / 0.2, 45.0)

    def test_a_dry_run_goes_through_every_move(self):
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(self.dog.main(["--run_dir", tmp]), 0)

    def test_a_dry_run_at_5_hz_goes_through_every_move(self):
        import importlib
        import tempfile

        try:
            with tempfile.TemporaryDirectory() as tmp:
                self.assertEqual(self.dog.main(["--run_dir", tmp, "--hz", "5"]), 0)
        finally:
            self.dog.PERIOD_S = 0.1
            importlib.reload(self.dog)

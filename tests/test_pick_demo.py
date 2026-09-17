"""Tests for the scripted cup pick's numpy half: camera model, cup geometry, grasp planning and the sequence.

Numpy only (no Isaac, no ultralytics), so these run on the system Python.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import d1_ik
from pick_demo import camera, grasp, perception, sequence
from position_only.workspace import clear_of_body, forward, load_urdf, tip_offsets

UP = np.array([0.0, 0.0, 1.0])
BASE_HEIGHT = 0.12
CUP_XY = np.array([0.42, 0.03])
CUP_RADIUS, CUP_HEIGHT = 0.0275, 0.10


def cup_top():
    return np.array([CUP_XY[0], CUP_XY[1], -BASE_HEIGHT + CUP_HEIGHT])


def cup_surface(camera_pos, rng, noise_m=0.001):
    """Points a camera above and in front would see on a mug: rim, near outer wall, far inner wall, handle."""
    centre = np.array([CUP_XY[0], CUP_XY[1], 0.0])
    top, bottom = -BASE_HEIGHT + CUP_HEIGHT, -BASE_HEIGHT
    toward = camera_pos[:2] - CUP_XY
    toward /= np.linalg.norm(toward)
    angles = rng.uniform(0, 2 * np.pi, 3000)
    ring = np.column_stack([np.cos(angles), np.sin(angles)])
    near = ring @ toward > 0.0
    rim = np.column_stack([CUP_XY + CUP_RADIUS * ring, np.full(len(ring), top - rng.uniform(0, 0.002, len(ring)))])
    wall = np.column_stack([CUP_XY + CUP_RADIUS * ring[near], rng.uniform(bottom, top, near.sum())])
    inner = np.column_stack([CUP_XY + (CUP_RADIUS - 0.004) * ring[~near], rng.uniform(top - 0.03, top, (~near).sum())])
    handle = np.column_stack([rng.uniform(CUP_XY[0] + CUP_RADIUS, CUP_XY[0] + CUP_RADIUS + 0.022, 300),
                              rng.uniform(CUP_XY[1] - 0.006, CUP_XY[1] + 0.006, 300),
                              rng.uniform(bottom + 0.008, top - 0.007, 300)])
    points = np.vstack([rim, wall, inner, handle])
    return points + rng.normal(0.0, noise_m, points.shape)


class CameraTests(unittest.TestCase):
    def test_presets_follow_the_datasheet_geometry(self):
        d435 = camera.CAMERAS["d435"]
        self.assertAlmostEqual(math.degrees(2 * math.atan(320 / d435.fx)), 54.9, delta=0.5)
        self.assertAlmostEqual(d435.min_depth_m, 0.177, delta=0.005)
        self.assertEqual(camera.CAMERAS["d405"].min_depth_m, 0.07)
        self.assertGreater(camera.CAMERAS["d455"].min_depth_m, d435.min_depth_m)

    def test_project_inverts_deproject(self):
        model = camera.CAMERAS["d435"]
        u, v, z = np.array([0, 100, 639]), np.array([0, 240, 479]), np.array([0.3, 0.5, 1.2])
        pixels = camera.project(model, camera.deproject(model, u, v, z))
        np.testing.assert_allclose(pixels, np.column_stack([u, v]), atol=1e-9)

    def test_mount_is_a_rotation_and_sees_the_fingertips(self):
        mount = camera.WristMount()
        rot = mount.rotation
        np.testing.assert_allclose(rot.T @ rot, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(rot), 1.0, places=12)
        joints, _ = load_urdf()
        tip = np.r_[tip_offsets(joints)["fingertip_centre_m"], 1.0]
        in_cam = camera.invert(mount.pose) @ tip
        self.assertGreater(in_cam[2], 0.05)
        u, v = camera.project(camera.CAMERAS["d435"], in_cam[:3])[0]
        self.assertTrue(0 <= u < 640 and 240 < v < 480, (u, v))

    def test_quaternion_matches_the_matrix(self):
        mount = camera.WristMount(pitch_deg=33.0)
        w, x, y, z = mount.quat_wxyz()
        matrix = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                           [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                           [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(matrix, mount.rotation, atol=1e-9)

    def test_depth_has_range_limits_and_the_modelled_noise(self):
        model = camera.CAMERAS["d435"]
        out = camera.realsense_depth(model, np.array([0.1, 0.3, 5.0, np.inf]), np.random.default_rng(0))
        self.assertEqual(out[0], 0.0)
        self.assertAlmostEqual(float(out[1]), 0.3, delta=0.005)
        self.assertEqual(out[2], 0.0)
        self.assertEqual(out[3], 0.0)
        samples = camera.realsense_depth(model, np.full(40000, 0.4), np.random.default_rng(1))
        self.assertAlmostEqual(float(samples.std()), float(model.depth_noise_std_m(0.4)), delta=0.1 * model.depth_noise_std_m(0.4))


class PerceptionTests(unittest.TestCase):
    def test_rim_fit_finds_the_axis_despite_wall_inside_and_handle(self):
        rng = np.random.default_rng(3)
        camera_pos = np.array([0.25, 0.02, 0.24])
        estimate = perception.estimate_cup(cup_surface(camera_pos, rng), UP, camera_pos)
        self.assertEqual(estimate.method, "rim_circle")
        self.assertLess(np.linalg.norm(estimate.top_centre_b[:2] - CUP_XY), 0.002)
        self.assertAlmostEqual(estimate.radius_m, CUP_RADIUS, delta=0.002)
        self.assertAlmostEqual(estimate.top_height_m, -BASE_HEIGHT + CUP_HEIGHT, delta=0.003)
        # Few points reach the base from above, so the bottom reads high; nothing uses it to grasp.
        self.assertAlmostEqual(estimate.bottom_height_m, -BASE_HEIGHT, delta=0.01)

    def test_too_few_points_is_no_estimate(self):
        self.assertIsNone(perception.estimate_cup(np.zeros((10, 3)), UP, np.zeros(3)))

    def test_ray_to_height_recovers_a_point_from_its_pixel(self):
        model = camera.CAMERAS["d435"]
        pose = camera.transform(grasp.optical_rotation(np.array([0.3, 0.0, -1.0]), UP), [0.35, 0.0, 0.1])
        point = np.array([0.41, 0.05, -0.02])
        pixel = camera.project(model, (camera.invert(pose) @ np.r_[point, 1.0])[:3])[0]
        hit = perception.ray_to_height(model, pose, pixel[0], pixel[1], UP, point[2])
        np.testing.assert_allclose(hit, point, atol=1e-9)

    def test_reobserve_casts_the_mask_when_depth_is_too_close(self):
        joints, _ = load_urdf()
        model, mount = camera.CAMERAS["d435"], camera.WristMount()
        cup = perception.CupEstimate(cup_top(), CUP_RADIUS, cup_top()[2], -BASE_HEIGHT, 100, "prior")
        plan = grasp.plan_top_down_grasp(joints, _links(), cup, UP, np.zeros(6), BASE_HEIGHT)
        pose = camera.camera_pose(joints, plan.q_pregrasp, mount)
        moved = cup_top() + np.array([0.006, -0.004, 0.0])
        u, v = camera.project(model, (camera.invert(pose) @ np.r_[moved, 1.0])[:3])[0]
        mask = np.zeros((480, 640), bool)
        mask[int(round(v)) - 20:int(round(v)) + 21, int(round(u)) - 20:int(round(u)) + 21] = True

        class Detector:
            def detect(self, rgb, labels):
                return [perception.Detection("bowl", 0.6, (u - 20, v - 20, u + 20, v + 20), mask)]

        seen = perception.CupPerception(Detector(), model, joints, mount).reobserve(
            perception.Frame(np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640), np.float32), plan.q_pregrasp, UP, 0.0), cup)
        self.assertEqual(seen.estimate.method, "ray_to_height")
        self.assertLess(np.linalg.norm(seen.estimate.top_centre_b - moved), 0.001)


_LINKS = None


def _links():
    global _LINKS
    if _LINKS is None:
        _LINKS = load_urdf()[1]
    return _LINKS


class GraspTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.joints, cls.links = load_urdf()
        cls.cup = perception.CupEstimate(cup_top(), CUP_RADIUS, cup_top()[2], -BASE_HEIGHT, 100, "truth")

    def test_ground_clearance_follows_gravity_not_the_base(self):
        # A pose reaching 4 cm above a floor that a 7 deg nose-up base puts 5.6 cm lower than level would.
        pitch = math.radians(7.0)
        up = np.array([math.sin(pitch), 0.0, math.cos(pitch)])
        cup = perception.CupEstimate(np.array([0.44, 0.03, -0.047]), CUP_RADIUS, -0.047, -0.14, 100, "tilted")
        plan = grasp.plan_top_down_grasp(self.joints, self.links, cup, up, np.zeros(6), 0.085)
        self.assertTrue(grasp.arm_clear(self.joints, plan.descend[-1], 0.085, up)[0])
        self.assertFalse(clear_of_body(self.joints, plan.descend[-1][None], 0.085)[0])
        level = grasp.arm_clear(self.joints, plan.descend[-1], 0.085, UP)[0]
        self.assertEqual(bool(level), bool(clear_of_body(self.joints, plan.descend[-1][None], 0.085)[0]))

    def test_top_down_rotation_points_down_with_a_level_jaw(self):
        for tilt in (0.0, 20.0):
            rot = grasp.top_down_rotation(np.array([0.8, 0.6, 0.0]), UP, tilt)
            np.testing.assert_allclose(rot.T @ rot, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(rot), 1.0, places=12)
            self.assertAlmostEqual(rot[2, 1], 0.0, places=12)      # jaw axis horizontal
            self.assertAlmostEqual(rot[:, 1] @ np.array([0.8, 0.6, 0.0]), 0.0, places=12)
            self.assertAlmostEqual(rot[2, 2], -math.cos(math.radians(tilt)), places=12)

    def test_default_scene_plans_a_straight_clear_grasp(self):
        view = grasp.plan_observation(self.joints, self.links, camera.CAMERAS["d435"], camera.WristMount(),
                                      grasp.floor_point(0.42, 0.03, UP, BASE_HEIGHT), UP, np.zeros(6), BASE_HEIGHT)
        pose = camera.camera_pose(self.joints, view.q, camera.WristMount())
        np.testing.assert_allclose(pose, view.camera_pose_b, atol=2e-4)
        self.assertGreaterEqual(view.distance_m, camera.CAMERAS["d435"].min_depth_m)
        # The look point lands in the upper image, clear of the gripper, not at the centre.
        u, v = camera.project(camera.CAMERAS["d435"], (camera.invert(pose) @ np.r_[view.look_at_b, 1.0])[:3])[0]
        self.assertAlmostEqual(u, 319.5, delta=3.0)
        self.assertAlmostEqual(v, 0.3 * 480 - 0.5, delta=3.0)

        plan = grasp.plan_top_down_grasp(self.joints, self.links, self.cup, UP, view.q, BASE_HEIGHT)
        np.testing.assert_allclose(grasp.jaw_positions(self.joints, plan.descend[-1:])[0], plan.jaw_grasp_b, atol=1e-3)
        previous = view.q
        for q in plan.approach + plan.descend + plan.lift:
            self.assertTrue(d1_ik.path_clearance(self.joints, previous, q, BASE_HEIGHT)[0])
            previous = q
        frames, _, _ = forward(self.joints, plan.descend[-1][None])
        fingertip = frames["Link6"][1][0] + frames["Link6"][0][0] @ np.array([0.0, 0.0, grasp.FINGERTIP_Z_M])
        params = grasp.GraspParams()
        overlap = min(params.finger_overlap_m, grasp.shell_height_above_tips(plan.tilt_deg) - params.rim_clearance_m)
        self.assertAlmostEqual(fingertip[2], cup_top()[2] - overlap, delta=0.002)
        # The wrist shell's lowest corner stays the wanted clearance above the rim.
        corners = [frames["Link6"][1][0] + frames["Link6"][0][0] @ np.array([x, 0.0, grasp.PALM_Z_M])
                   for x in grasp.PALM_X_RANGE_M]
        self.assertGreaterEqual(min(c[2] for c in corners) - cup_top()[2], params.rim_clearance_m - 0.002)
        # Steps down are no longer than the cap.
        self.assertGreaterEqual(len(plan.descend), math.ceil(params.pregrasp_clearance_m / params.descend_step_m))
        path = [plan.q_pregrasp] + plan.descend
        worst = max(grasp.line_deviation(grasp.jaw_positions(self.joints, d1_ik.traversal_configs(a, b)),
                                         plan.jaw_pregrasp_b, plan.jaw_grasp_b).max() for a, b in zip(path, path[1:]))
        self.assertLessEqual(worst, grasp.GraspParams().line_tolerance_m + 1e-9)

    def test_a_cup_wider_than_the_jaws_is_refused(self):
        wide = perception.CupEstimate(cup_top(), 0.04, cup_top()[2], -BASE_HEIGHT, 100, "truth")
        with self.assertRaises(grasp.PlanningError):
            grasp.plan_top_down_grasp(self.joints, self.links, wide, UP, np.zeros(6), BASE_HEIGHT)

    def test_an_out_of_reach_cup_is_refused(self):
        far = perception.CupEstimate(np.array([1.0, 0.0, cup_top()[2]]), CUP_RADIUS, cup_top()[2], -BASE_HEIGHT, 100, "x")
        with self.assertRaises(grasp.PlanningError):
            grasp.plan_top_down_grasp(self.joints, self.links, far, UP, np.zeros(6), BASE_HEIGHT)


class SequenceTests(unittest.TestCase):
    def test_runs_to_done_with_a_refinement_against_a_simple_arm(self):
        joints, links = load_urdf()
        model, mount = camera.CAMERAS["d435"], camera.WristMount()
        truth = cup_top()
        detection = perception.Detection("cup", 0.9, (0, 0, 1, 1), None)

        class Perception:
            refined = 0

            def observe(self, frame):
                estimate = perception.CupEstimate(truth + np.array([-0.008, 0.005, 0.0]), CUP_RADIUS, truth[2],
                                                  -BASE_HEIGHT, 500, "rim_circle", 0.001, 300.0)
                return perception.CupObservation(detection, estimate, np.eye(4), 1.0)

            def reobserve(self, frame, prior):
                self.refined += 1
                estimate = perception.CupEstimate(truth.copy(), CUP_RADIUS, truth[2], -BASE_HEIGHT, 50, "ray_to_height")
                return perception.CupObservation(detection, estimate, np.eye(4), 0.0)

        fake = Perception()
        pick = sequence.PickSequence(joints, links, model, mount, fake, base_height_m=BASE_HEIGHT,
                                     timing=sequence.Timing(settle_s=0.2))
        q, dt, t, gripper_log, target = np.zeros(6), 0.02, 0.0, [], None
        while not pick.done and t < 90.0:
            command = pick.update(t, q, UP, lambda: object())
            if command.q is not None:
                target = command.q
            if target is not None:
                q = q + np.clip(target - q, -1.2 * dt, 1.2 * dt)
            gripper_log.append((command.state, command.gripper_m))
            t += dt
        self.assertEqual(pick.state, "done", pick.failure)
        self.assertEqual(fake.refined, 2)       # moved once, then confirmed
        np.testing.assert_allclose(pick.cup.top_centre_b, truth, atol=1e-9)
        np.testing.assert_allclose(grasp.jaw_positions(joints, q)[0], pick.plan.jaw_lift_b, atol=0.002)
        states = [s for s, _ in gripper_log]
        close_at = states.index("close")
        self.assertEqual(gripper_log[close_at - 1][1], grasp.GRIPPER_OPEN_M)
        self.assertAlmostEqual(gripper_log[-1][1], (2 * CUP_RADIUS - 0.004 - grasp.CLOSED_GAP_M) / 2, places=9)

    def test_grip_closes_just_inside_the_cup_and_stays_in_range(self):
        self.assertAlmostEqual(2 * grasp.grip_travel_m(0.055, 0.004) + grasp.CLOSED_GAP_M, 0.051, places=9)
        self.assertEqual(grasp.grip_travel_m(0.010, 0.004), grasp.GRIPPER_CLOSED_M)
        self.assertEqual(grasp.grip_travel_m(0.200, 0.004), grasp.GRIPPER_OPEN_M)


class RandomCupTests(unittest.TestCase):
    def test_restart_positions_stay_in_the_picked_region_with_the_handle_off_the_jaw_axis(self):
        import run_pick_demo  # stdlib-only at import; Isaac is imported inside run()

        rng = np.random.default_rng(0)
        for _ in range(500):
            (x, y), yaw = run_pick_demo.random_cup(rng)
            self.assertTrue(run_pick_demo.RANDOM_CUP_X[0] <= x <= run_pick_demo.RANDOM_CUP_X[1])
            self.assertTrue(run_pick_demo.RANDOM_CUP_Y[0] <= y <= run_pick_demo.RANDOM_CUP_Y[1])
            # Handle relative to pointing straight away from the robot: within the band of 0 or of 180 deg.
            rel = (yaw - math.degrees(math.atan2(y, x)) + 180.0) % 360.0 - 180.0
            off_axis = min(abs(rel), 180.0 - abs(rel))
            self.assertLessEqual(off_axis, run_pick_demo.RANDOM_HANDLE_BAND_DEG + 1e-9)


if __name__ == "__main__":
    unittest.main()

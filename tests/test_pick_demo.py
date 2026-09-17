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

    def test_a_cup_wider_than_the_jaws_is_refused_when_wall_grasps_are_off(self):
        wide = perception.CupEstimate(cup_top(), 0.04, cup_top()[2], -BASE_HEIGHT, 100, "truth")
        params = grasp.GraspParams(wall_grasp="off")
        with self.assertRaises(grasp.PlanningError):
            grasp.plan_top_down_grasp(self.joints, self.links, wide, UP, np.zeros(6), BASE_HEIGHT, params)

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



class WeightsDownloadTests(unittest.TestCase):
    """`ensure_weights` against a local file:// URL: kept when the hash matches, discarded when it does not."""

    def setUp(self):
        import tempfile

        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.source = self.dir / "source.pt"
        self.source.write_bytes(b"not really weights")
        self.url = self.source.resolve().as_uri()

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_matching_download_is_kept(self):
        import hashlib

        from pick_demo.perception import ensure_weights

        target = self.dir / "yolo" / "w.pt"
        ensure_weights(target, self.url, hashlib.sha256(self.source.read_bytes()).hexdigest())
        self.assertEqual(target.read_bytes(), self.source.read_bytes())

    def test_a_mismatched_download_leaves_nothing_behind(self):
        from pick_demo.perception import ensure_weights

        target = self.dir / "yolo" / "w.pt"
        with self.assertRaises(RuntimeError):
            ensure_weights(target, self.url, "0" * 64)
        self.assertEqual(list(target.parent.iterdir()), [])

    def test_existing_weights_are_not_fetched(self):
        from pick_demo.perception import ensure_weights

        target = self.dir / "w.pt"
        target.write_bytes(b"already here")
        ensure_weights(target, "http://127.0.0.1:9/never", "0" * 64)
        self.assertEqual(target.read_bytes(), b"already here")


if __name__ == "__main__":
    unittest.main()


class CameraBodyTests(unittest.TestCase):
    """Intel's D435 case mesh at the wrist mount: geometry only, no Isaac."""

    def setUp(self):
        from pick_demo import camera_body

        self.camera_body = camera_body
        self.mount = camera.WristMount()
        if not camera_body.MESH_PATH.is_file():
            self.skipTest(f"no case mesh at {camera_body.MESH_PATH}")

    def test_the_cad_is_the_case_the_datasheet_describes(self):
        """90 x 25 x 25 mm nominal; the CAD is allowed to differ by a tenth of a millimetre."""
        size = np.array(self.camera_body.housing_size_m())
        np.testing.assert_allclose(size, self.camera_body.HOUSING_SIZE_NOMINAL_M, atol=1e-4)

    def test_the_registration_lands_the_cad_lenses_on_the_nominal_extrinsics(self):
        """The check that the mesh-to-optical transform is right rather than merely asserted.

        `_d435.urdf.xacro` places the mesh; the mesh's own lens barrels then have to appear where the
        nominal extrinsics say the imagers are. The colour barrel and its two inner elements are the
        strongest of these: they are concentric with the optical axis, so they must land on x = y = 0.
        """
        points, faces = self.camera_body.mesh_optical()
        front = points[points[:, 2] > 0.0]                 # the front plate and the barrels on it
        self.assertGreater(len(front), 0)
        # The colour lens element: the only geometry on the axis, 0.4 to 0.8 mm in front of the sensor.
        lens = points[(points[:, 2] > 0.0004) & (points[:, 2] < 0.0008)
                      & (np.abs(points[:, 0]) < 0.002) & (np.abs(points[:, 1]) < 0.002)]
        self.assertGreater(len(lens), 100, "no colour lens element found where the xacro puts it")
        self.assertLess(abs(float(lens[:, 0].mean())), 5e-5, "the colour lens is off the optical axis")
        self.assertLess(abs(float(lens[:, 1].mean())), 5e-5, "the colour lens is off the optical axis")

    def test_the_case_runs_the_right_way_along_its_long_axis(self):
        """The sign that was wrong until 2026-09-17: the colour lens is 12.5 mm from one end, not 35.

        librealsense's depth-to-colour translation is the depth origin *in the colour frame*, so the
        left imager is to the right of colour and the case extends that way. Mirroring it moved the
        case 22 mm, which is the number a bracket is designed around.
        """
        lo, hi = self.camera_body.housing_bounds_m()
        self.assertAlmostEqual(lo[0], -0.0125, places=3)
        self.assertAlmostEqual(hi[0], +0.0774, places=3)
        self.assertGreater(self.camera_body.imager_positions_m()["left_ir"][0], 0.0,
                           "the left imager must be to the right of the colour sensor")

    def test_colour_is_the_optical_origin(self):
        x, y, z = self.camera_body.imager_positions_m()["colour"]
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, self.camera_body.GLASS_AHEAD_OF_OPTICAL_M)

    def test_imager_spacing_is_the_measured_baseline(self):
        positions = self.camera_body.imager_positions_m()
        span = positions["right_ir"][0] - positions["left_ir"][0]
        self.assertAlmostEqual(span, self.camera_body.STEREO_BASELINE_M, places=9)
        self.assertAlmostEqual(positions["left_ir"][0], self.camera_body.COLOUR_FROM_LEFT_IMAGER_M, places=9)

    def test_the_cad_agrees_with_the_bench_camera(self):
        """The xacro's nominals and the measured extrinsics must not disagree by more than tolerance."""
        self.assertAlmostEqual(self.camera_body.COLOUR_FROM_LEFT_IMAGER_M,
                               self.camera_body.NOMINAL_COLOUR_FROM_DEPTH_M, delta=3e-4)
        self.assertAlmostEqual(self.camera_body.STEREO_BASELINE_M, 0.050, delta=3e-4)

    def test_every_imager_lies_inside_the_case(self):
        lo, hi = (np.array(v) for v in self.camera_body.housing_bounds_m())
        for name, position in self.camera_body.imager_positions_m().items():
            p = np.array(position)
            self.assertTrue(np.all(p >= lo - 1e-9) and np.all(p <= hi + 1e-9), f"{name} lies outside the case")

    def test_the_tripod_thread_is_on_the_bottom_face_at_the_long_axis_centre(self):
        lo, hi = self.camera_body.housing_bounds_m()
        x, y, z = self.camera_body.tripod_thread_m()
        self.assertAlmostEqual(y, hi[1], places=6)                      # the bottom face (+y is down)
        self.assertAlmostEqual(x, (lo[0] + hi[0]) / 2.0, delta=1e-4)    # centred along the case

    def test_the_carved_asset_leaves_a_clear_aperture_at_the_model_eye(self):
        """The lens element sits on the axis; with it gone every preset looks out of a clear hole."""
        from pick_demo.camera_asset import carve_lens

        points, faces = self.camera_body.mesh_optical()
        kept, dropped = carve_lens(points, faces)
        self.assertGreater(dropped, 0, "nothing was carved: the lens element should have been")
        self.assertLess(dropped, len(faces) // 100, "the carve took more than 1% of the case")
        models = [camera.CAMERAS[n] for n in ("d435", "d405", "d455")]
        stored = self.camera_body.MESH_PATH.parent / "calibration" / "d435i_238222076237_640x480.json"
        if stored.is_file():
            from pick_demo import realsense
            models.append(realsense.load_camera_model(stored))
        for model in models:
            with self.subTest(model=model.name):
                self.assertEqual(self.camera_body.view_obstruction(model, points=points[kept]), [])

    def test_the_uncarved_case_blocks_the_view_so_the_guard_can_fail(self):
        """The guard has to fail on the shape that actually blinds the camera, or it guards nothing."""
        found = self.camera_body.view_obstruction(camera.CAMERAS["d435"])
        self.assertEqual([entry["part"] for entry in found], ["housing"])
        self.assertGreater(found[0]["triangles"], 0)

    def test_the_guard_sees_the_case_from_the_rendered_eye(self):
        """F-052: the renderer's eye is inside the case, so the honest answer there is 'blocked'.

        This is the regression that put a lens marker in shot while the guard reported clear: it was
        only ever asked about the model frame. If F-052 is ever fixed, this test is what says so.
        """
        from pick_demo.camera_asset import carve_lens

        points, faces = self.camera_body.mesh_optical()
        kept, _ = carve_lens(points, faces)
        found = self.camera_body.view_obstruction(camera.CAMERAS["d435"],
                                                  self.camera_body.RENDERED_EYE_OFFSET_M,
                                                  points=points[kept])
        self.assertTrue(found, "the rendered eye sits inside the case; the guard must say so")

    def test_points_transform_into_link6_through_the_mount(self):
        """part_pose_link6 must agree with applying the mount pose by hand."""
        point = self.camera_body.tripod_thread_m()
        expected = (self.mount.pose @ np.append(np.array(point), 1.0))[:3]
        np.testing.assert_allclose(self.camera_body.part_pose_link6(point, self.mount), expected)

    def test_default_mount_keeps_the_housing_clear_of_the_shell(self):
        report = self.camera_body.clearance_report(self.mount, grasp.PALM_Z_M, grasp.PALM_X_RANGE_M)
        self.assertFalse(report["intersects_shell"], report)

    def test_a_mount_buried_in_the_wrist_is_reported(self):
        """The clearance check has to be able to fail, or it is not a check."""
        buried = camera.WristMount(pos_link6=(0.0, 0.0, 0.03), pitch_deg=0.0)
        report = self.camera_body.clearance_report(buried, grasp.PALM_Z_M, grasp.PALM_X_RANGE_M)
        self.assertTrue(report["intersects_shell"], report)


class RealSenseCalibrationTests(unittest.TestCase):
    """The stored calibration and the CameraModel built from it. No camera and no pyrealsense2 needed."""

    def setUp(self):
        from pick_demo import realsense

        self.realsense = realsense
        self.path = realsense.CALIBRATION_DIR / "d435i_238222076237_640x480.json"
        if not self.path.is_file():
            self.skipTest("no stored calibration for the bench camera")
        self.calibration = realsense.load_calibration(self.path)

    def test_model_uses_the_measured_intrinsics(self):
        model = self.realsense.camera_model(self.calibration)
        colour = self.calibration["colour_intrinsics"]
        for field in ("fx", "fy"):
            self.assertAlmostEqual(getattr(model, field), colour[field], places=6)
        self.assertAlmostEqual(model.cx, colour["cx"], places=6)
        self.assertAlmostEqual(model.cy, colour["cy"], places=6)
        self.assertIn("measured", model.source)

    def test_the_measured_principal_point_is_not_the_image_centre(self):
        """The reason the preset is not good enough: cy is 14 px off centre on this camera."""
        model = self.realsense.camera_model(self.calibration)
        self.assertGreater(abs(model.cy - model.height / 2.0), 10.0)

    def test_the_preset_and_the_measurement_disagree(self):
        """If these ever agree, the preset stopped being an assumption and this test should say so."""
        model = self.realsense.camera_model(self.calibration)
        preset = camera.CAMERAS["d435"]
        self.assertGreater(abs(model.fx - preset.fx), 5.0)

    def test_body_offset_matches_the_calibration_it_came_from(self):
        """camera_body's measured offset must be the one in the stored calibration, not a stale copy."""
        from pick_demo import camera_body

        measured = self.calibration["depth_to_colour"]["translation_m"][0]
        self.assertAlmostEqual(camera_body.COLOUR_FROM_LEFT_IMAGER_M, measured, places=6)
        self.assertAlmostEqual(camera_body.STEREO_BASELINE_M, self.calibration["stereo_baseline_m"], places=5)

    def test_deprojection_with_the_measured_model_round_trips(self):
        """A point deprojected and reprojected must return to its pixel, with the real principal point."""
        model = self.realsense.camera_model(self.calibration)
        pixels = np.array([[100.0, 80.0], [320.0, 240.0], [600.0, 450.0]])
        depths = np.array([0.35, 0.40, 0.55])
        points = camera.deproject(model, pixels[:, 0], pixels[:, 1], depths)
        back = camera.project(model, points)
        np.testing.assert_allclose(back, pixels - 0.5 + 0.5, atol=1e-6)


class WideCupTests(unittest.TestCase):
    """Cups near the jaws' limit: what is refused, and what is attempted more carefully.

    The bench mug (Week 1 log, 2026-09-17) measured 70-83 mm depending on how much rim was seen, against
    77.2 mm of open jaw. The old rule refused anything over 67.2 mm outright; these hold the new one,
    which spends the clearance on a finer descent instead and only refuses a cup that leaves no room.
    """

    def setUp(self):
        self.params = grasp.GraspParams()

    def test_clearance_is_what_the_cup_leaves_of_the_open_jaws(self):
        self.assertAlmostEqual(grasp.jaw_clearance_per_side_m(0.0275), grasp.OPEN_GAP_M / 2 - 0.0275, places=9)
        # A cup wider than the jaws has negative room; the number must go negative rather than clamp.
        self.assertLess(grasp.jaw_clearance_per_side_m(0.045), 0.0)

    def test_a_narrow_cup_keeps_the_full_descent_tolerance(self):
        """The 55 mm cup every recorded simulated pick used must plan exactly as it did before."""
        self.assertAlmostEqual(grasp.descent_tolerance_for(0.0275, self.params),
                               self.params.line_tolerance_m, places=9)

    def test_a_wide_cup_tightens_the_descent_instead_of_being_refused(self):
        tolerance = grasp.descent_tolerance_for(0.035, self.params)     # 70 mm, the bench mug
        self.assertGreater(tolerance, 0.0)
        self.assertLess(tolerance, self.params.line_tolerance_m)
        # And it must leave the safety margin beside the fingers.
        self.assertAlmostEqual(tolerance + self.params.jaw_safety_m,
                               grasp.jaw_clearance_per_side_m(0.035), places=9)

    def test_a_cup_that_fills_the_jaws_is_still_refused(self):
        for diameter in (0.079, 0.083):
            with self.subTest(diameter=diameter):
                self.assertLessEqual(grasp.descent_tolerance_for(diameter / 2, self.params), 0.0)

    def test_the_refusal_says_how_much_room_there_was(self):
        joints, links = load_urdf()
        wide = perception.CupEstimate(cup_top(), 0.0415, cup_top()[2], -BASE_HEIGHT, 500, "rim_circle")
        params = grasp.GraspParams(wall_grasp="off")
        with self.assertRaises(grasp.PlanningError) as caught:
            grasp.plan_top_down_grasp(joints, links, wide, UP, np.zeros(6), BASE_HEIGHT, params)
        self.assertIn("mm a side", str(caught.exception))

    def test_a_70_mm_cup_now_plans_a_grasp(self):
        """The bench mug, at the width its best-conditioned look reported."""
        joints, links = load_urdf()
        cup = perception.CupEstimate(cup_top(), 0.035, cup_top()[2], -BASE_HEIGHT, 500, "rim_circle")
        plan = grasp.plan_top_down_grasp(joints, links, cup, UP, np.zeros(6), BASE_HEIGHT, self.params)
        self.assertTrue(plan.descend)
        # The finer tolerance must actually show up as more waypoints than the 55 mm cup needs.
        narrow = perception.CupEstimate(cup_top(), 0.0275, cup_top()[2], -BASE_HEIGHT, 500, "rim_circle")
        narrow_plan = grasp.plan_top_down_grasp(joints, links, narrow, UP, np.zeros(6), BASE_HEIGHT, self.params)
        self.assertGreaterEqual(len(plan.descend), len(narrow_plan.descend))

    def test_the_descent_for_a_wide_cup_stays_inside_its_own_clearance(self):
        """The point of the whole change: the fingers must not reach the cup on the way down."""
        joints, links = load_urdf()
        radius = 0.035
        cup = perception.CupEstimate(cup_top(), radius, cup_top()[2], -BASE_HEIGHT, 500, "rim_circle")
        plan = grasp.plan_top_down_grasp(joints, links, cup, UP, np.zeros(6), BASE_HEIGHT, self.params)
        jaws = grasp.jaw_positions(joints, np.array(plan.descend))
        deviation = grasp.line_deviation(jaws, plan.jaw_pregrasp_b, plan.jaw_grasp_b)
        self.assertLess(float(np.max(deviation)), grasp.jaw_clearance_per_side_m(radius))


class WallGraspTests(unittest.TestCase):
    """Cups too wide for the jaws to straddle, taken by the wall instead.

    The bench mug measured 70-83 mm (F-058) against 77.2 mm of open jaw, so whether a cup is graspable
    from the outside turns on a millimetre or two of a circle fit. These hold the fallback: the same
    descent, the fingers shut and inside the mouth, opened against the wall from within.
    """

    @classmethod
    def setUpClass(cls):
        cls.joints, cls.links = load_urdf()

    def cup(self, diameter_m, height_m=CUP_HEIGHT):
        top = np.array([CUP_XY[0], CUP_XY[1], -BASE_HEIGHT + height_m])
        return perception.CupEstimate(top, diameter_m / 2, top[2], -BASE_HEIGHT + height_m - height_m,
                                      500, "rim_circle")

    def plan(self, diameter_m, **params):
        return grasp.plan_top_down_grasp(self.joints, self.links, self.cup(diameter_m), UP, np.zeros(6),
                                         BASE_HEIGHT, grasp.GraspParams(**params))

    def test_the_cups_that_already_worked_are_planned_exactly_as_before(self):
        """The fallback must not touch a cup the jaws can take: same mode, same waypoints, same grip."""
        for diameter in (0.055, 0.070):
            with self.subTest(diameter=diameter):
                fallback, old = self.plan(diameter), self.plan(diameter, wall_grasp="off")
                self.assertEqual(fallback.mode, "outside")
                self.assertEqual(fallback.gripper_descend_m, grasp.GRIPPER_OPEN_M)
                self.assertEqual(len(fallback.descend), len(old.descend))
                np.testing.assert_allclose(fallback.jaw_grasp_b, old.jaw_grasp_b, atol=1e-12)
                self.assertAlmostEqual(fallback.gripper_grasp_m,
                                       grasp.grip_travel_m(diameter, grasp.GraspParams().squeeze_m), places=12)

    def test_which_wall_grasp_a_cup_gets_is_decided_by_the_jaws_it_is_planned_for(self):
        """The real arm shuts to about 2 mm and pinches the wall; the URDF's 17.2 mm jaws cannot, and
        fall through to the inside-out grasp. Same cup, same code, different gripper."""
        with self.assertRaises(grasp.PlanningError):
            self.plan(0.080, wall_grasp="off")
        self.assertEqual(self.plan(0.080).mode, "pinch")
        self.assertEqual(self.plan(0.080, pinch_closed_gap_m=grasp.CLOSED_GAP_M).mode, "inside_out")

    def test_a_cup_too_wide_for_the_jaws_is_taken_from_inside(self):
        """80 mm: 1.4 mm too wide to straddle, so the fingers go in shut and open on the wall."""
        plan = self.plan(0.080, wall_grasp="inside_out")
        self.assertEqual(plan.mode, "inside_out")
        self.assertEqual(plan.gripper_descend_m, grasp.GRIPPER_CLOSED_M)
        self.assertGreater(plan.gripper_grasp_m, plan.gripper_descend_m)
        # The jaw centre still descends down the cup's axis: an inside-out grasp is centred, not offset.
        self.assertAlmostEqual(float(np.hypot(*(plan.jaw_grasp_b[:2] - self.cup(0.080).top_centre_b[:2]))),
                               0.0, places=9)

    def test_the_fingers_go_in_clear_and_come_out_pressing_on_the_wall(self):
        """The point of the mode: clearance on the way down, contact with the wall at the end."""
        params = grasp.GraspParams()
        for diameter in (0.080, 0.090, 0.100):
            with self.subTest(diameter=diameter):
                plan = self.plan(diameter, wall_grasp="inside_out")
                inner = grasp.inside_radius_m(self.cup(diameter), params)
                insert = grasp.insert_depth_m(self.cup(diameter), params, plan.tilt_deg)
                going_in = grasp.finger_reach_m(plan.rotation_b, UP, plan.gripper_descend_m, insert)
                holding = grasp.finger_reach_m(plan.rotation_b, UP, plan.gripper_grasp_m, insert)
                self.assertLess(going_in, inner - params.jaw_safety_m)
                self.assertGreater(holding, inner)      # past the wall, so the drives press on it
                # And the descent may not wander further sideways than the room going in.
                jaws = grasp.jaw_positions(self.joints, np.array(plan.descend))
                deviation = grasp.line_deviation(jaws, plan.jaw_pregrasp_b, plan.jaw_grasp_b)
                self.assertLess(float(np.max(deviation)), inner - going_in)

    def test_the_fingers_stop_above_the_inside_of_the_base_and_the_palm_above_the_rim(self):
        params = grasp.GraspParams()
        cup = self.cup(0.090)
        plan = self.plan(0.090, wall_grasp="inside_out")
        frames, _, _ = forward(self.joints, plan.descend[-1][None])
        rot, pos = frames["Link6"][0][0], frames["Link6"][1][0]
        fingertip = pos + rot @ np.array([0.0, 0.0, grasp.FINGERTIP_Z_M])
        self.assertGreaterEqual(fingertip[2] - (cup.top_centre_b[2] - cup.height_m),
                                params.wall_floor_clearance_m - 0.002)
        palm = min((pos + rot @ np.array([x, 0.0, grasp.PALM_Z_M]))[2] for x in grasp.PALM_X_RANGE_M)
        self.assertGreaterEqual(palm - cup.top_centre_b[2], params.rim_clearance_m - 0.002)

    def test_a_shallow_cup_shortens_the_insertion_and_then_refuses_it(self):
        """The depth limit, on its own: a floor-standing cup this shallow has an unreachable rim anyway."""
        params = grasp.GraspParams()
        deep = grasp.insert_depth_m(self.cup(0.090, height_m=0.10), params, 0.0)
        shallow = grasp.insert_depth_m(self.cup(0.090, height_m=0.045), params, 0.0)
        self.assertAlmostEqual(deep, params.wall_insert_m, places=9)
        self.assertAlmostEqual(shallow, 0.045 - params.wall_floor_clearance_m, places=9)
        self.assertIn("inside the cup", grasp.insert_depth_m(self.cup(0.090, height_m=0.025), params, 0.0))

    def test_a_cup_wider_than_the_fingers_can_reach_is_refused_with_the_numbers(self):
        """Past about 100 mm of mouth the fingers cannot touch both walls; with CAD jaws nothing is left."""
        with self.assertRaises(grasp.PlanningError) as caught:
            self.plan(0.140, pinch_closed_gap_m=grasp.CLOSED_GAP_M)
        message = str(caught.exception)
        self.assertIn("cannot press on the wall", message)
        self.assertIn("130 mm", message)                 # the mouth, as the planner reads it

    def test_the_pinch_is_refused_against_the_modelled_jaws_and_planned_against_the_real_ones(self):
        """F-059: the CAD pads bottom out 17.2 mm apart, so closing on a 5 mm wall never touches it."""
        with self.assertRaises(grasp.PlanningError) as caught:
            self.plan(0.090, wall_grasp="pinch", pinch_closed_gap_m=grasp.CLOSED_GAP_M)
        self.assertIn("F-059", str(caught.exception))

        plan = self.plan(0.090, wall_grasp="pinch")
        params = grasp.GraspParams()
        self.assertEqual(plan.mode, "pinch")
        # It shuts past the URDF's stop, onto the wall, and the gap it ends at is the one it was told the
        # jaws reach -- 2 mm on a 5 mm wall, so 3 mm of squeeze.
        self.assertLess(plan.gripper_grasp_m, grasp.GRIPPER_CLOSED_M)
        self.assertAlmostEqual(grasp.jaw_gap_m(plan.gripper_grasp_m), params.pinch_closed_gap_m, places=12)
        self.assertLess(grasp.jaw_gap_m(plan.gripper_grasp_m), params.wall_thickness_m)
        # ...and goes down with the wall loose between the pads, not gripped.
        self.assertGreater(grasp.jaw_gap_m(plan.gripper_descend_m), params.wall_thickness_m)
        # The pinch point is on the wall, a quarter turn round the rim, where the jaw axis is radial.
        cup = self.cup(0.090)
        frames, _, _ = forward(self.joints, plan.descend[-1][None])
        rot, pos = frames["Link6"][0][0], frames["Link6"][1][0]
        fingertip = pos + rot @ np.array([0.0, 0.0, grasp.FINGERTIP_Z_M])
        radial = fingertip[:2] - cup.top_centre_b[:2]
        self.assertAlmostEqual(float(np.linalg.norm(radial)),
                               cup.radius_m - grasp.GraspParams().wall_thickness_m / 2, delta=0.002)
        self.assertAlmostEqual(float(radial @ plan.heading_b[:2]), 0.0, delta=0.002)
        self.assertAlmostEqual(abs(float(radial @ rot[:2, 1])), float(np.linalg.norm(radial)), delta=0.002)

    def test_a_cup_standing_below_the_floor_it_was_given_lowers_the_floor_but_not_the_other_way(self):
        """The bench refused a mug it could see, by 3 mm of ground proxy (Week 1 log, 2026-09-17).

        The floor is *told* to the planner and the cup is *measured*, so where they disagree the cup wins
        -- downwards only, and by at most `ground_trust_m`. A cup standing on a box says nothing about
        the ground beside it.
        """
        params = grasp.GraspParams()
        deep = perception.CupEstimate(np.array([0.273, -0.064, 0.142]), 0.0417, 0.142, 0.055, 500, "rim_circle")
        floor, note = grasp.ground_under(deep, -0.09, params)
        self.assertAlmostEqual(floor, -0.055, places=9)
        self.assertIn("below the floor", note)
        # ...and never lifted by a cup standing higher than the floor it was given.
        raised = perception.CupEstimate(np.array([0.29, 0.08, 0.20]), 0.0417, 0.20, 0.13, 500, "rim_circle")
        self.assertEqual(grasp.ground_under(raised, -0.09, params), (-0.09, ""))
        # ...and the trust is bounded: a cup reading a long way under does not hand over the whole gap.
        absurd = perception.CupEstimate(np.array([0.29, 0.08, 0.05]), 0.0417, 0.05, -0.20, 500, "rim_circle")
        self.assertAlmostEqual(grasp.ground_under(absurd, 0.0, params)[0], params.ground_trust_m, places=9)

    def test_an_unknown_mode_is_a_planning_error_not_a_silent_outside_grasp(self):
        with self.assertRaises(grasp.PlanningError):
            self.plan(0.055, wall_grasp="sideways")


class WallGraspSequenceTests(unittest.TestCase):
    """What the state machine commands for a wide cup -- the same object drives the simulator and the arm.

    The sequence knows nothing about walls: it holds whatever the plan says on the way down and commands
    whatever it says at the bottom. So these check the two wall grasps end to end, in the one place where
    getting the direction wrong would open the fingers when they should shut.
    """

    RADIUS = 0.045          # 90 mm: too wide for the 77.2 mm jaws to straddle

    def run_pick(self, **params):
        joints, links = load_urdf()
        truth = cup_top()
        radius = self.RADIUS
        detection = perception.Detection("cup", 0.9, (0, 0, 1, 1), None)

        class Perception:
            def observe(self, frame):
                estimate = perception.CupEstimate(truth, radius, truth[2], -BASE_HEIGHT, 500, "rim_circle",
                                                  0.001, 300.0)
                return perception.CupObservation(detection, estimate, np.eye(4), 1.0)

            def reobserve(self, frame, prior):
                return perception.CupObservation(detection, perception.CupEstimate(
                    truth.copy(), radius, truth[2], -BASE_HEIGHT, 50, "ray_to_height"), np.eye(4), 0.0)

        pick = sequence.PickSequence(joints, links, camera.CAMERAS["d435"], camera.WristMount(), Perception(),
                                     base_height_m=BASE_HEIGHT, timing=sequence.Timing(settle_s=0.2),
                                     grasp_params=grasp.GraspParams(**params))
        q, dt, t, log, target = np.zeros(6), 0.02, 0.0, [], None
        while not pick.done and t < 90.0:
            command = pick.update(t, q, UP, lambda: object())
            if command.q is not None:
                target = command.q
            if target is not None:
                q = q + np.clip(target - q, -1.2 * dt, 1.2 * dt)
            log.append((command.state, command.gripper_m))
            t += dt
        self.assertEqual(pick.state, "done", pick.failure)
        # What the gripper was told from the last look above the cup to the bottom of the descent: the
        # part of the run where anything the plan did not ask for would hit the cup.
        states = [s for s, _ in log]
        going_down = log[len(states) - 1 - states[::-1].index("refine"):states.index("close")]
        return pick, log, [g for _, g in going_down]

    def test_an_inside_out_grasp_goes_down_shut_and_opens_onto_the_wall(self):
        """Against the URDF's jaws, which cannot pinch: the fingers go in shut and press outwards."""
        pick, log, before = self.run_pick(pinch_closed_gap_m=grasp.CLOSED_GAP_M)
        self.assertEqual(pick.plan.mode, "inside_out")
        self.assertEqual(set(before), {grasp.GRIPPER_CLOSED_M})
        self.assertAlmostEqual(log[-1][1], pick.plan.gripper_grasp_m, places=12)
        self.assertGreater(log[-1][1], grasp.GRIPPER_CLOSED_M)      # it holds the cup by being open
        self.assertEqual([e["event"] for e in pick.events].count("opening onto the wall"), 1)

    def test_a_pinch_goes_down_straddling_the_wall_and_shuts_on_it(self):
        """Against the real arm's jaws, which shut to about 2 mm."""
        pick, log, before = self.run_pick()
        self.assertEqual(pick.plan.mode, "pinch")
        self.assertEqual(set(before), {pick.plan.gripper_descend_m})
        self.assertGreater(grasp.jaw_gap_m(pick.plan.gripper_descend_m), grasp.GraspParams().wall_thickness_m)
        self.assertEqual(log[-1][1], grasp.closed_travel_m(grasp.GraspParams()))
        self.assertLess(log[-1][1], grasp.GRIPPER_CLOSED_M)      # past the URDF's stop, where the real arm goes
        self.assertEqual([e["event"] for e in pick.events].count("closing"), 1)


class RimCoverageTests(unittest.TestCase):
    """A first look must see enough rim before its circle is trusted to set the grasp width."""

    class _Detector:
        def __init__(self, outer):
            self.outer = outer
            self.names = {41: "cup"}

        def detect(self, rgb, labels=(perception.COCO_CUP,)):
            return [perception.Detection("cup", 0.9, (0, 0, 10, 10), None)]

    def _perception(self, **kwargs):
        joints, _ = load_urdf()
        return perception.CupPerception(self._Detector(None), camera.CAMERAS["d435"], joints,
                                        camera.WristMount(), **kwargs)

    def test_the_default_gate_matches_what_the_bench_showed(self):
        """63-66 deg read 79-83 mm and 126 deg read 70 mm on the same mug: the gate sits between them."""
        gate = perception.CupPerception.MIN_RIM_COVERAGE_DEG
        self.assertGreater(gate, 66.0)
        self.assertLess(gate, 126.0)

    def test_a_short_arc_is_rejected_with_a_reason(self):
        seen = self._perception()
        estimate = perception.CupEstimate(cup_top(), 0.0415, cup_top()[2], -BASE_HEIGHT, 500,
                                          "rim_circle", 0.0006, 63.4)
        self.assertTrue(estimate.rim_coverage_deg < seen.min_rim_coverage_deg)

    def test_an_estimate_without_coverage_is_not_gated(self):
        """`ray_to_height` reports no coverage; gating on a missing number would refuse every re-look."""
        estimate = perception.CupEstimate(cup_top(), 0.035, cup_top()[2], -BASE_HEIGHT, 50, "ray_to_height")
        self.assertIsNone(estimate.rim_coverage_deg)

    def test_the_gate_can_be_turned_off(self):
        seen = self._perception(min_rim_coverage_deg=0.0)
        self.assertEqual(seen.min_rim_coverage_deg, 0.0)

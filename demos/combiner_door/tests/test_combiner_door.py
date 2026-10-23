"""CPU tests for the combiner door demo: the front and head cameras' mounts, the front camera's lens and its render, the
lidar's origin, the look and the door's angle.

No simulator. Run from the repository root:

    python -m unittest demos.combiner_door.tests.test_combiner_door -v
"""
import math
from pathlib import Path
import unittest

import numpy as np

from demos.combiner.apriltag import TagDetection, TagDetector, invert, tag_image, tag_pose_box
from demos.combiner.geometry import GEOMETRY
from demos.combiner_door import front_camera, head_camera, mid360, perception
from demos.unifp import props

DATA = Path(__file__).resolve().parent / "data"


def _rot_z(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _rot_y(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


class HeadCamera(unittest.TestCase):
    def test_the_mount_is_p2dingos_looking_straight_ahead(self):
        pose = head_camera.optical_pose_base()
        np.testing.assert_allclose(pose[:3, 3], (0.3392, 0.03215, 0.0927), atol=1e-9)
        np.testing.assert_allclose(pose[:3, 2], (1.0, 0.0, 0.0), atol=1e-12)    # optical z: forward
        np.testing.assert_allclose(pose[:3, 0], (0.0, -1.0, 0.0), atol=1e-12)   # x: image right is the robot's right
        np.testing.assert_allclose(pose[:3, 1], (0.0, 0.0, -1.0), atol=1e-12)   # y: image down

    def test_the_model_is_the_measured_d435i_rendered_about_the_centre(self):
        model = head_camera.camera_model()
        self.assertEqual((model.width, model.height), (640, 480))
        self.assertAlmostEqual(model.fx, 607.11, places=2)
        self.assertEqual((model.cx, model.cy), (320.0, 240.0))

    def test_the_bracket_is_behind_the_lens_and_clear_of_the_lidar_mount(self):
        from dock import read_binary_stl
        from payload import CLEARANCE_BOXES_B

        for _, name, _ in head_camera.BRACKET_MESHES:
            points = read_binary_stl(head_camera.ASSETS / name).reshape(-1, 3)
            self.assertLessEqual(points[:, 0].max(), head_camera.COLOUR_SENSOR_BASE_M[0])
            for box in CLEARANCE_BOXES_B:
                low, high = np.array(box)[:, 0], np.array(box)[:, 1]
                inside = np.all((points >= low) & (points <= high), axis=1)
                self.assertFalse(inside.any(), f"{name} inside the lidar mount's box {box}")

    def test_the_camera_sits_below_the_lidars_field(self):
        """The case's top front edge (front plate x 0.3435, top z 0.1052, P2Dingo's CAD) under the MID-360's -7 deg."""
        origin = np.array(mid360.ORIGIN_BASE_M)
        for y in (-0.0457, 0.0450):
            corner = np.array([0.3435, y, 0.1052]) - origin
            elevation = math.degrees(math.atan2(corner[2], math.hypot(corner[0], corner[1])))
            self.assertLess(elevation, mid360.VERTICAL_FOV_DEG[0])


def _render_tag(rotation, position, model, pixels_per_cell=40):
    """The tag (quiet zone included) at `position`/`rotation` in the front camera's optical frame, as the simulator draws
    it: an exact pinhole projection of the plane into `front_camera.render_size`'s render, on a grey background, then
    remapped through the lens (`front_camera.distort_render`)."""
    import cv2

    width, height, f = front_camera.render_size(model)
    render_k = np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])
    cell = GEOMETRY.tag_outer_size / 10
    texture_to_tag = np.array([[cell / pixels_per_cell, 0.0, -5 * cell], [0.0, -cell / pixels_per_cell, 5 * cell],
                               [0.0, 0.0, 1.0]])
    homography = render_k @ np.column_stack((rotation[:, 0], rotation[:, 1], position)) @ texture_to_tag
    grey = cv2.warpPerspective(tag_image(pixels_per_cell=pixels_per_cell), homography, (width, height),
                               flags=cv2.INTER_LINEAR, borderValue=150)
    return front_camera.distort_render(np.dstack([grey] * 3), model)


class FrontCamera(unittest.TestCase):
    """The Go2's front camera, the look's since 2026-10-22: the URDF's mount and a NOMINAL fisheye lens."""

    def test_the_mount_is_the_urdfs_looking_straight_ahead(self):
        pose = front_camera.optical_pose_base()
        np.testing.assert_allclose(pose[:3, 3], (0.330, 0.0, 0.040), atol=1e-9)
        np.testing.assert_allclose(pose[:3, 2], (1.0, 0.0, 0.0), atol=1e-12)    # optical z: forward
        np.testing.assert_allclose(pose[:3, 0], (0.0, -1.0, 0.0), atol=1e-12)   # x: image right is the robot's right
        np.testing.assert_allclose(pose[:3, 1], (0.0, 0.0, -1.0), atol=1e-12)   # y: image down

    def test_the_lens_is_labelled_as_no_full_calibration(self):
        model = front_camera.camera_model()
        self.assertEqual((model.width, model.height), (1280, 720))
        self.assertFalse(model.calibrated)
        self.assertIn("NOT a full calibration", model.source)

    def test_the_fitted_lens_puts_the_box_where_the_tape_did(self):
        """The frame `ONE_STAND` was fitted on (the camera taped 0.61 m from the box's front, square to it): the tag's
        plane 0.61 m off within a centimetre, where the first guess (`NOMINAL`) read 2 cm long."""
        import cv2

        rgb = cv2.imread(str(DATA / "go2_front_camera_box_0p61m.jpg"))[..., ::-1]
        for model, tolerance in ((front_camera.ONE_STAND, 0.01), (front_camera.NOMINAL, 0.03)):
            found = [d for d in TagDetector(model.intrinsic_matrix, fisheye_distortion=model.distortion).detect(rgb)
                     if d.corners_px[:, 0].mean() < 900]          # not the charging dock's tag (same id) on the right
            self.assertEqual(len(found), 1)
            pose = found[0].pose_cam
            plane = abs(pose[:3, 2] @ pose[:3, 3])
            self.assertAlmostEqual(plane, 0.61, delta=tolerance)
        self.assertGreater(abs(plane - 0.61), 0.01)              # NOMINAL's: the fit mattered

    def test_the_lens_model_is_opencvs_fisheye_and_inverts(self):
        import cv2

        model = front_camera.camera_model()
        rng = np.random.default_rng(3)
        points = rng.normal(size=(400, 3))
        points[:, 2] = np.abs(points[:, 2]) + 0.3
        uv = model.project(points)
        cv_uv, _ = cv2.fisheye.projectPoints(points.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                                             model.intrinsic_matrix, np.array(model.distortion).reshape(4, 1))
        np.testing.assert_allclose(uv, cv_uv.reshape(-1, 2), atol=1e-6)
        rays = model.rays(uv)
        np.testing.assert_allclose(rays, points / np.linalg.norm(points, axis=1, keepdims=True), atol=1e-9)

    def test_the_render_holds_the_whole_fisheye_field(self):
        width, height, _ = front_camera.render_size()
        map_x, map_y = front_camera.distort_maps()
        self.assertEqual(map_x.shape, (720, 1280))
        self.assertGreaterEqual(map_x.min(), 0.0)
        self.assertLessEqual(map_x.max(), width - 1.0)
        self.assertGreaterEqual(map_y.min(), 0.0)
        self.assertLessEqual(map_y.max(), height - 1.0)

    def test_rviz_draws_the_field_through_the_corner_rays(self):
        model = front_camera.camera_model()
        k = front_camera.frustum_matrix(model)
        for u, v in ((0.0, 0.0), (model.width, model.height)):
            ray = model.rays([[u, v]])[0]
            np.testing.assert_allclose(((u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1]),
                                       (ray[0] / ray[2], ray[1] / ray[2]), atol=1e-9)

    def test_a_rendered_tag_is_measured_through_the_lens(self):
        """The simulator's path end to end: rendered, remapped, found, solved with the lens undistorted. Off the image's
        centre, a detector that took the lens for a pinhole would be 6-15 cm out."""
        model = front_camera.camera_model()
        fisheye = TagDetector(model.intrinsic_matrix, fisheye_distortion=model.distortion)
        pinhole = TagDetector(model.intrinsic_matrix)
        facing = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])    # tag x right, y up, z to the camera
        for position, yaw_deg in (((0.30, -0.20, 0.85), 15.0), ((0.55, -0.25, 0.75), -20.0), ((-0.45, 0.10, 1.2), 10.0)):
            rotation = _rot_y(yaw_deg) @ facing
            image = _render_tag(rotation, np.array(position), model)
            found = fisheye.detect(image)
            self.assertEqual(len(found), 1, position)
            self.assertLess(np.linalg.norm(found[0].pose_cam[:3, 3] - position), 0.008, position)
            naive = pinhole.detect(image)[0]
            self.assertGreater(np.linalg.norm(naive.pose_cam[:3, 3] - position), 0.03, position)

    def test_the_tag_is_whole_in_the_picture_from_where_the_robot_looks(self):
        """At every door-pull placement from `LOOK_BACK_M` back, P2Dingo's walking policy standing the base 0.30-0.33 m
        up (0.314 in the trace) and pitched +-2 deg; from the stance itself the tag runs off the top of the image."""
        import random

        model = front_camera.camera_model()
        rng = random.Random(1)
        half = GEOMETRY.tag_outer_size / 2

        def in_frame(site, back_m, z=0.314, pitch_deg=0.0):
            base = np.eye(4)
            base[:3, :3] = _rot_y(pitch_deg)
            base[:3, 3] = (-back_m, 0.0, z)
            cam = base @ front_camera.optical_pose_base()
            tag = perception.site_pose(site) @ tag_pose_box()
            corners = [(invert(cam) @ tag @ np.array([dx, dy, 0.0, 1.0]))[:3]
                       for dx, dy in ((-half, half), (half, half), (half, -half), (-half, -half))]
            return bool(front_camera.in_view(model, np.array(corners), margin_px=20.0).all())

        sites = [props.sample_door_site(rng) for _ in range(200)]
        for z, pitch in ((0.314, 0.0), (0.30, 2.0), (0.33, -2.0)):
            self.assertTrue(all(in_frame(site, perception.LOOK_BACK_M, z, pitch) for site in sites), (z, pitch))
        self.assertEqual(sum(in_frame(site, 0.0) for site in sites), 0)

    def test_the_tag_is_found_in_a_real_frame(self):
        """A frame of the real camera (P2Dingo's stream, 1280x720, the box's tag 0.4-0.5 m ahead, 2026-10-22). Shows
        the decoder finds it through the real lens; the range is only checked for plausibility."""
        import cv2

        model = front_camera.camera_model()
        rgb = cv2.imread(str(DATA / "go2_front_camera_tag.jpg"))[..., ::-1]
        found = TagDetector(model.intrinsic_matrix, fisheye_distortion=model.distortion).detect(rgb)
        self.assertEqual([d.tag_id for d in found], [GEOMETRY.tag_id])
        self.assertTrue(0.25 < found[0].range_m < 0.9, found[0].range_m)


class LidarOrigin(unittest.TestCase):
    def test_on_the_housings_axis_47_mm_above_its_bottom(self):
        from dock import read_binary_stl
        from payload import MESH_DIR

        points = read_binary_stl(MESH_DIR / "mid360.stl").reshape(-1, 3)
        low, high = points.min(axis=0), points.max(axis=0)
        expected = ((low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2] + 0.047)
        np.testing.assert_allclose(mid360.ORIGIN_BASE_M, expected, atol=0.001)


class Look(unittest.TestCase):
    def test_a_detection_gives_back_the_placement(self):
        truth = props.BoxSite(distance_m=0.72, bearing_deg=3.5, yaw_deg=184.0)
        # The base a little forward, left, up and turned, and pitched as a standing dog's trunk is.
        base = np.eye(4)
        base[:3, :3] = _rot_z(1.2) @ _rot_y(1.5)
        base[:3, 3] = (0.02, -0.01, 0.40)
        box_base = invert(base) @ perception.site_pose(truth)
        tag_cam = invert(front_camera.optical_pose_base()) @ box_base @ tag_pose_box()   # the look's camera
        detection = TagDetection(GEOMETRY.tag_id, np.zeros((4, 2)), tag_cam, 0.0, 100.0)
        site = perception.site_from_pose(perception.box_in_spawn(detection, base))
        self.assertAlmostEqual(site.distance_m, truth.distance_m, places=9)
        self.assertAlmostEqual(site.bearing_deg, truth.bearing_deg, places=9)
        self.assertAlmostEqual(site.yaw_deg - 360.0 * round((site.yaw_deg - truth.yaw_deg) / 360.0), truth.yaw_deg,
                               places=9)

    @staticmethod
    def _in_frame(site, back_m):
        """Whether all four of the tag's corners are in the head camera's picture, `back_m` behind the stance, standing
        at the walking policy's 0.40 m."""
        model = head_camera.camera_model()
        base = np.eye(4)
        base[:3, 3] = (-back_m, 0.0, 0.40)
        cam = base @ head_camera.optical_pose_base()
        tag = perception.site_pose(site) @ tag_pose_box()
        half = GEOMETRY.tag_outer_size / 2
        for dx, dy in ((-half, half), (half, half), (half, -half), (-half, -half)):
            corner = invert(cam) @ tag @ np.array([dx, dy, 0.0, 1.0])
            u = model.fx * corner[0] / corner[2] + model.cx
            v = model.fy * corner[1] / corner[2] + model.cy
            if not (corner[2] > 0.1 and 0 <= u < model.width and 0 <= v < model.height):
                return False
        return True

    def test_the_tag_is_whole_in_the_picture_from_where_the_robot_looks(self):
        """At every door-pull placement from `LOOK_BACK_M` back; from the stance itself it is not (why it looks back)."""
        import random

        rng = random.Random(1)
        sites = [props.sample_door_site(rng) for _ in range(200)]
        self.assertTrue(all(self._in_frame(site, perception.LOOK_BACK_M) for site in sites))
        self.assertLess(sum(self._in_frame(site, 0.0) for site in sites), 180)


def _rotate_about_hinge(xy, angle_deg):
    g = GEOMETRY
    a = math.radians(g.hinge_axis_z * angle_deg)
    x, y = xy[:, 0] - g.hinge[0], xy[:, 1] - g.hinge[1]
    return np.stack((g.hinge[0] + math.cos(a) * x - math.sin(a) * y, g.hinge[1] + math.sin(a) * x + math.cos(a) * y),
                    axis=1)


def _door_points(angle_deg, rng, noise_m=mid360.RANGE_NOISE_M, spacing=0.01, standoff=0.39):
    """Returns on the door's outer face (whole door), as a lidar `standoff` m off it would see them, with range noise
    along each ray; the robot turned round the hinge with the door, as the door pull walks it. Enclosure frame."""
    g = GEOMETRY
    face = g.door_x + g.door_thickness / 2
    ys = np.arange(-g.width / 2 + 0.01, g.width / 2 - 0.01, spacing)
    zs = np.arange(g.bottom + 0.01, g.top - 0.01, spacing)
    yy, zz = np.meshgrid(ys, zs)
    points = np.stack((np.full(yy.size, face), yy.ravel(), zz.ravel()), axis=1)
    lidar = np.array([face + standoff, 0.0, 0.505])
    rotate = lambda p: np.column_stack((_rotate_about_hinge(p[:, :2], angle_deg), p[:, 2]))
    points, lidar = rotate(points), rotate(lidar[None])[0]
    rel = points - lidar
    points = points[np.degrees(np.arctan2(rel[:, 2], np.hypot(rel[:, 0], rel[:, 1]))) >= -7.0]   # the field's edge
    rays = points - lidar
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    return points + rays * rng.normal(0.0, noise_m, (len(points), 1))


def _interior_points(rng):
    """The box's insides behind the door: its mounting plate and back wall, which an open door uncovers."""
    g = GEOMETRY
    ys = rng.uniform(-g.width / 2 + 0.04, g.width / 2 - 0.04, 3000)
    zs = rng.uniform(g.bottom + 0.06, g.top - 0.06, 3000)
    xs = rng.choice([-0.10, -g.depth / 2 + g.wall], 3000)
    return np.column_stack((xs, ys, zs))


class FaceLine(unittest.TestCase):
    def test_the_shut_doors_face_gives_the_boxs_heading(self):
        """P2Dingo's wall fit on the door: the heading to well under a degree, from a window seeded 8 deg off."""
        rng = np.random.default_rng(4)
        fit = mid360.face_line(_door_points(0.0, rng, standoff=0.51), mid360.tag_at(0.0),
                               mid360.facing(0.0) + math.radians(8.0))
        self.assertTrue(fit.ok, fit)
        self.assertLess(abs(math.degrees(mid360._wrap(fit.heading - math.pi))), 0.5)
        self.assertLess(abs(fit.depth), 0.01)        # the face where the tag says, give or take its standoff

    def test_the_look_takes_the_tags_position_and_the_lidars_heading(self):
        truth = props.BoxSite(distance_m=0.72, bearing_deg=-2.0, yaw_deg=183.0)
        box = perception.site_pose(truth)
        rng = np.random.default_rng(5)
        cloud = _door_points(0.0, rng, standoff=0.51)
        cloud = (box[:3, :3] @ cloud.T).T + box[:3, 3]
        # A tag pose 8 deg off in yaw about its own centre: the heading error a 10 cm tag has at range on the robot.
        turn = np.eye(4)
        turn[:3, :3] = _rot_z(8.0)
        centre = perception.tag_centre(box)
        off = np.eye(4)
        off[:3, 3] = centre
        back = np.eye(4)
        back[:3, 3] = -centre
        tag_pose = off @ turn @ back @ box
        site, pose, fit = perception.lidar_estimate([tag_pose], cloud)
        self.assertTrue(fit.ok)
        error = perception.compare(pose, site, truth)
        self.assertLess(error["site_yaw_deg"], 0.5)
        self.assertLess(error["site_position_mm"], 8.0)
        # Without the lidar, the tag's heading would have stood.
        self.assertGreater(perception.compare(tag_pose, perception.site_from_pose(tag_pose), truth)["site_yaw_deg"], 7.0)


class DoorAngle(unittest.TestCase):
    def test_reads_the_door_at_every_angle_of_the_swing(self):
        rng = np.random.default_rng(1)
        for angle in (0.0, 15.0, 45.0, 70.0, 90.0, 105.0):
            points = np.vstack((_door_points(angle, rng), _interior_points(rng)))
            # The door's travel is 0-110 deg (it rests on its stop shut), so the last reading is never outside it.
            for prior in (angle, max(0.0, angle - 10.0), min(110.0, angle + 30.0)):
                reading = mid360.door_angle(points, prior)
                self.assertTrue(reading.ok, (angle, prior))
                self.assertLess(abs(reading.angle_deg - angle), 1.0, (angle, prior, reading))

    def test_no_door_in_view_is_no_reading(self):
        rng = np.random.default_rng(2)
        self.assertFalse(mid360.door_angle(_interior_points(rng), 30.0).ok)
        self.assertFalse(mid360.door_angle(np.zeros((0, 3)), 30.0).ok)

    def test_a_box_estimate_off_by_a_centimetre_still_reads_the_door(self):
        """The look's estimate of the box carries into the frame the points are read in."""
        rng = np.random.default_rng(3)
        points = _door_points(60.0, rng) + np.array([0.01, -0.01, 0.005])
        reading = mid360.door_angle(points, 55.0)
        self.assertLess(abs(reading.angle_deg - 60.0), 1.5, reading)


if __name__ == "__main__":
    unittest.main()

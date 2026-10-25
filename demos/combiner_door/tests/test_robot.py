"""The combiner door's robot program without the robot: the look's fix from synthetic tag poses and lidar returns, the
range check, the refit after the walk in, a dry attempt through every stage, sport mode's command shaping, and the
RViz plumbing. numpy only (no ROS, no simulator)."""
from __future__ import annotations

import math
from types import SimpleNamespace
import unittest

import numpy as np

import d1_hardware
import d1_ik
from demos.combiner.apriltag import TagDetection, tag_pose_box
from demos.combiner.geometry import GEOMETRY
from demos.combiner_door import mid360, perception
from demos.combiner_door.robot import plan, viz
from demos.combiner_door.robot.arm_link import SimulatedArm, SimulatedLink
from demos.cup.pick_demo import grasp
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6
from demos.unifp import script as demo_script

#: Servo 6 with the 40 mm lever between the pads: where the jaws stall closing on it (`plan.GRIPPER_SHUT_UNITS`' note).
LEVER_STALL_UNITS = 6.8


def box_odom(x, y, yaw_deg):
    """The enclosure on the floor at (x, y), its door facing `yaw_deg` (180: facing a robot looking along +x)."""
    return plan.pose(plan.rot_z(math.radians(yaw_deg)), (x, y, 0.0))


def door_box(x, y, yaw_deg):
    """`box_odom` for the door pull: (x, y) as from the simulated demo's stance, moved to where the dog stands
    (`plan.STANCE_BACK_M` further from the door, `plan.STANCE_RIGHT_M` toward the hinge)."""
    return box_odom(x + plan.STANCE_BACK_M, y + plan.STANCE_RIGHT_M, yaw_deg)


def detection(base, box):
    pose_cam = plan.invert(plan.CAMERA_POSE_BASE) @ plan.invert(base) @ box @ tag_pose_box(GEOMETRY)
    return TagDetection(GEOMETRY.tag_id, np.zeros((4, 2)), pose_cam, 0.1, 120.0)


def face_scan(base, box, rng, noise_m=0.01, door_deg=0.0, half_width=0.30, above=0.36):
    """Lidar returns (base frame) on the door's outer face round the tag (to `half_width` either side and `above` it:
    the door's edges by default), as `door_deg` open, with range noise."""
    g = GEOMETRY
    ys, zs = np.meshgrid(np.arange(-half_width, half_width, 0.012),
                         np.arange(g.tag_centre[2] - 0.10, g.tag_centre[2] + above, 0.012))
    face = np.stack([np.full(ys.size, g.door_x + g.door_thickness / 2), ys.ravel(), zs.ravel()], axis=-1)
    door = viz.door_pose(box, door_deg)
    world = plan.transform(door, face)
    points = plan.transform(plan.invert(base), world)
    return points + rng.normal(0.0, noise_m, points.shape)


def side_scan(base, box, rng, noise_m=0.01):
    """Lidar returns (base frame) on the box's hinge-side wall at the tag's heights: near 90 deg open, the door's outer
    face stands nearly in line with it (the dog's look of 2026-10-25 02:26 UTC)."""
    g = GEOMETRY
    xs, zs = np.meshgrid(np.arange(-g.depth / 2, g.depth / 2, 0.012),
                         np.arange(g.tag_centre[2] - 0.10, g.tag_centre[2] + 0.36, 0.012))
    wall = np.stack([xs.ravel(), np.full(xs.size, g.hinge_side * g.width / 2), zs.ravel()], axis=-1)
    points = plan.transform(plan.invert(base) @ box, wall)
    return points + rng.normal(0.0, noise_m, points.shape)


def walk(base, command, moving, dt=0.1, counted=1.0):
    """A dry run's base: walks the command; stopped, it carries on as the real gait does (`plan.CARRY_S` of the speed it
    had, over that long). `counted`: the share of the step's translation the odometry counts (its yaw whole). Returns the
    base and the speed it is going at."""
    if command is None:
        decay = max(0.0, 1.0 - dt / plan.CARRY_S)
        command = moving = tuple(v * decay for v in moving)
    else:
        moving = tuple(command)
    x, y, yaw = plan.planar(base)
    vx, vy, wz = command
    return plan.base_pose(x + counted * dt * (math.cos(yaw) * vx - math.sin(yaw) * vy),
                          y + counted * dt * (math.sin(yaw) * vx + math.cos(yaw) * vy), yaw + dt * wz), moving


def look(base, box, frames=20, scans=10, seed=0):
    rng = np.random.default_rng(seed)
    return ([(base, [detection(base, box)])] * frames, [(base, face_scan(base, box, rng)) for _ in range(scans)])


def finger_points(q_servo, base_s, travel=d1_hardware.GRIPPER_FULL_TRAVEL_M):
    """Points through both fingers' volume (`grasp.FINGER_*`, each finger `travel` open), script frame."""
    joints, _ = d1_ik.load_urdf()
    pos, rot = d1_ik.tool_pose(joints, d1_ik.from_servo_deg(q_servo)[0], body="Link6", offset=(0.0, 0.0, 0.0))
    grid = [(x, side * y, z) for side in (1, -1)
            for y in np.linspace(grasp.FINGER_INNER_Y_M + travel, grasp.FINGER_OUTER_Y_M + travel, 3)
            for x in np.linspace(*grasp.FINGER_X_RANGE_M, 5)
            for z in np.linspace(grasp.FINGER_SHOULDER_Z_M, grasp.FINGERTIP_Z_M, 6)]
    return plan.transform(base_s, pos + np.asarray(grid) @ rot.T)


def lever_and_hub_clearance(points_box):
    """Signed distances (m, negative inside) of points in the closed enclosure's frame from the measured slot lever and
    from the 63 mm hub behind it, which fills the gap to the door (`GEOMETRY.handle_hub_diameter`)."""
    g = GEOMETRY
    rel = np.asarray(points_box) - np.asarray(g.spindle)
    out, along, up = rel[:, 0], g.hinge_side * rel[:, 1], rel[:, 2]

    def solid(radial, slab):
        return np.where((radial > 0) & (slab > 0), np.hypot(radial, slab), np.maximum(radial, slab))

    lever = solid(np.hypot(along - np.clip(along, 0.0, g.handle_span), up) - g.handle_radius,
                  np.maximum(g.handle_gap - out, out - g.handle_gap - g.handle_thickness))
    hub = solid(np.hypot(along, up) - g.handle_hub_diameter / 2, np.maximum(-out, out - g.handle_gap))
    return lever, hub


class FixTests(unittest.TestCase):
    def setUp(self):
        # The odometry's frame is not the room's: the dog stands somewhere in it, turned.
        self.base = plan.base_pose(1.3, -0.4, 0.3)

    def box_ahead(self, forward, left, yaw_off_deg):
        return self.base @ plan.pose(None, (0, 0, -plan.BASE_HEIGHT_M)) @ door_box(forward, left, 180.0 + yaw_off_deg)

    def test_a_look_puts_the_stance_at_its_distance(self):
        box = self.box_ahead(0.95, 0.03, 3.0)
        frames, scans = look(self.base, box)
        fix = plan.make_fix(self.base, frames, scans)
        left = 0.03 + plan.STANCE_RIGHT_M
        walk = 0.95 + plan.STANCE_BACK_M - math.sqrt(plan.STANCE_DISTANCE_M ** 2 - left ** 2)
        # Sent short by the odometry's under-count (`plan.ODOM_WALK_SCALE`): the dog walks the whole of it.
        self.assertAlmostEqual(fix.walk_in_m, plan.ODOM_WALK_SCALE * walk, places=3)
        self.assertAlmostEqual(fix.site.root_m[0], math.sqrt(plan.STANCE_DISTANCE_M ** 2 - left ** 2)
                               + (1 - plan.ODOM_WALK_SCALE) * walk, places=3)
        self.assertTrue(fix.fit.ok, fix.fit.why)
        self.assertAlmostEqual(fix.site.yaw_deg % 360.0, 183.0, delta=0.5)
        ok, text = fix.in_range()
        self.assertTrue(ok, text)
        # The box's root is where it was put, in odom, and the script frame's z puts it at the plinth's height.
        self.assertLess(np.linalg.norm(fix.box_odom[:2, 3] - box[:2, 3]), 0.005)

    def test_the_stance_keeps_the_dog_clear_of_a_door_swinging_past_its_turn(self):
        """Lukas, 2026-10-25: near the end of the pull the door swung into the dog's face. The base turns about the
        hinge with the door, so the door reaches the dog only by swinging on past that turn (15-30 deg in both live
        pulls after the insertion). From the door pull's stance (`plan.STANCE_*`) it touches the dog's front only past
        35 deg; from the simulated demo's, under 25."""
        front = np.array([(0.37, 0.05), (0.37, -0.05), (0.30, 0.13), (0.30, -0.13)])     # the head, the trunk's corners

        def touches_after(box):
            for deg in np.arange(0.0, 90.0, 0.5):
                door = viz.door_pose(box, deg)
                face = door[:2, :2] @ np.array([GEOMETRY.door_x + GEOMETRY.door_thickness / 2, 0.0]) + door[:2, 3]
                along = door[:2, 1]
                s = np.clip((front - face) @ along, -GEOMETRY.width / 2, GEOMETRY.width / 2)
                if np.linalg.norm(front - (face + s[:, None] * along), axis=1).min() < 0.01:
                    return deg
            return 90.0

        base = plan.base_pose(0.0, 0.0, 0.0)
        stance = plan.make_fix(base, *look(base, door_box(plan.NOMINAL_DISTANCE_M, 0.0, 180.0)))
        self.assertEqual(stance.walk_in_m, 0.0)
        self.assertGreater(touches_after(perception.site_pose(stance.site)), 35.0)
        self.assertLess(touches_after(box_odom(plan.NOMINAL_DISTANCE_M, 0.0, 180.0)), 25.0)
        self.assertLess(abs(stance.site.grasp_point_m(0.0, 0.0)[1]), 0.03)     # the grip about straight ahead

    def test_no_tag_no_fix(self):
        self.assertIsNone(plan.make_fix(self.base, [(self.base, [])] * 5, []))

    def test_out_of_range_says_how_to_move_the_dog(self):
        n = plan.NOMINAL_DISTANCE_M
        for forward, left, yaw, word in ((n + 0.68, 0.0, 0.0, "forward"), (n + 0.23, 0.0, 15.0, "turn left"),
                                         (n + 0.23, 0.22, 0.0, "left"), (n - 0.08, 0.0, 0.0, "back")):
            frames, scans = look(self.base, self.box_ahead(forward, left, yaw))
            ok, text = plan.make_fix(self.base, frames, scans).in_range()
            with self.subTest(forward=forward, left=left, yaw=yaw):
                self.assertFalse(ok, text)
                self.assertIn(word, text)

    def test_with_the_dock_in_view_too_the_look_keeps_the_boxs_tag(self):
        """The front camera's ~100 degree field holds the box's tag and P2Dingo's dock's (also 36h11 id 0, 100 mm) at
        once (2026-10-22). Averaged, they put the box between them; grouped, the one on a door is kept, whichever was
        seen more often."""
        box = self.box_ahead(1.00, 0.03, 0.0)
        dock = self.box_ahead(1.40, -0.90, 10.0)             # its tag on a stand 2 cm in front of a wall
        rng = np.random.default_rng(6)
        frames = [(self.base, [detection(self.base, dock), detection(self.base, box)])] * 12 \
            + [(self.base, [detection(self.base, dock)])] * 12
        scans = [(self.base, np.concatenate([face_scan(self.base, box, rng),
                                             face_scan(self.base, dock @ plan.pose(None, (-0.02, 0.0, 0.0)), rng,
                                                       half_width=0.9, above=0.8)])) for _ in range(10)]
        fix = plan.make_fix(self.base, frames, scans)
        self.assertEqual(fix.other_tags, 1)
        self.assertTrue(fix.on_a_door())
        self.assertLess(np.linalg.norm(fix.box_odom[:2, 3] - box[:2, 3]), 0.02)
        ok, text = fix.in_range()
        self.assertTrue(ok, text)

    def test_a_tag_on_a_stand_is_not_the_box(self):
        # P2Dingo's dock carries the same tag on a stand, 2 cm in front of the wall the lidar sees (as on the dog,
        # 2026-10-20): the depth alone does not tell them apart, the wall's extent does.
        box = self.box_ahead(0.95, 0.0, 0.0)
        wall = box @ plan.pose(None, (-0.02, 0.0, 0.0))
        rng = np.random.default_rng(5)
        frames = [(self.base, [detection(self.base, box)])] * 20
        scans = [(self.base, face_scan(self.base, wall, rng, half_width=0.9, above=0.8)) for _ in range(10)]
        fix = plan.make_fix(self.base, frames, scans)
        ok, text = fix.in_range()
        self.assertFalse(ok)
        self.assertIn("stand", text)
        self.assertGreater(fix.beyond_edges, 0.5)
        # The box itself: nothing on the face's line past its edges.
        frames, scans = look(self.base, box)
        self.assertLess(plan.make_fix(self.base, frames, scans).beyond_edges, 0.05)

    def test_already_at_the_stance_no_walk(self):
        frames, scans = look(self.base, self.box_ahead(plan.NOMINAL_DISTANCE_M, 0.0, 0.0))
        fix = plan.make_fix(self.base, frames, scans)
        self.assertEqual(fix.walk_in_m, 0.0)
        self.assertTrue(fix.in_range()[0])

    def test_the_refit_takes_out_the_walks_odometry_error(self):
        box = self.box_ahead(0.95, 0.0, 2.0)
        frames, scans = look(self.base, box)
        fix = plan.make_fix(self.base, frames, scans)
        # The odometry has the base at the stance; it is really 4 cm short of it, and turned 1.5 deg.
        believed = fix.script_odom @ plan.pose(None, (0.0, 0.0, plan.BASE_HEIGHT_M - fix.script_odom[2, 3]))
        believed = plan.base_pose(*plan.planar(believed))
        actual = believed @ plan.pose(plan.rot_z(math.radians(1.5)), (-0.04, 0.0, 0.0))
        rng = np.random.default_rng(3)
        settle = [(believed, face_scan(actual, box, rng)) for _ in range(20)]
        refined, fit = plan.refine_fix(fix, settle)
        self.assertTrue(fit.ok, fit.why)
        # Where the arm will reach for the box, relative to the base, against where the box really is.
        true_in_base = plan.invert(actual) @ box
        before = plan.invert(believed) @ fix.box_odom
        after = plan.invert(believed) @ refined.box_odom
        self.assertGreater(np.linalg.norm(before[:2, 3] - true_in_base[:2, 3]), 0.03)
        normal = true_in_base[:2, 0]
        # Along the door's normal (all the lidar sees) the error is gone; along the face it stays.
        self.assertLess(abs((after[:2, 3] - true_in_base[:2, 3]) @ normal), 0.01)
        moved, turned = plan.fix_change(fix, refined)
        self.assertLess(moved, plan.REFIT_LIMIT_M)
        self.assertAlmostEqual(turned, 1.5, delta=0.8)


class WristCameraTests(unittest.TestCase):
    """The RealSense on the D1's wrist (2026-10-21): the look from the arm's look pose."""

    def test_the_look_pose_aims_the_wrist_camera_at_a_door_ahead(self):
        joints, links = d1_ik.load_urdf()
        q = plan.look_pose_servo_deg(joints, links)
        self.assertIsNotNone(q)
        self.assertTrue(np.all(np.abs(q) <= 90.0))
        camera = plan.wrist_camera_base(joints, q)
        np.testing.assert_allclose(camera[:3, 3], plan.LOOK_CAMERA_IN_BASE_M, atol=2e-3)
        self.assertTrue(bool(grasp.arm_clear(joints, d1_ik.from_servo_deg(q)[0].reshape(1, 6))[0]))
        # The tag in the image with the box's root 0.75-1.3 m ahead, where the looks are (the D435i's colour field, 1280 x
        # 720 at fx 907).
        for ahead in (0.75, 0.92, 1.30):
            box = box_odom(ahead, 0.0, 180.0)
            tag = (plan.invert(camera) @ plan.invert(plan.base_pose(0.0, 0.0, 0.0)) @ box @ tag_pose_box(GEOMETRY))[:3, 3]
            u, v = 907.0 * tag[0] / tag[2] + 640.0, 906.0 * tag[1] / tag[2] + 360.0
            self.assertTrue(0 < u < 1280 and 0 < v < 720, (ahead, u, v))

    def test_the_shut_fingers_stay_off_the_tag_in_the_look(self):
        """The first live look had an open finger across the tag (0 of 26 frames); the jaws are now shut while it looks,
        and the tag is aimed into the image's upper half. By the bracket's CAD mount, not measured."""
        joints, links = d1_ik.load_urdf()
        camera = plan.wrist_camera_base(joints, plan.look_pose_servo_deg(joints, links))
        to_camera = plan.invert(plan.wrist_mount())
        fingers = []
        for side in (1, -1):
            for z in np.linspace(grasp.FINGER_SHOULDER_Z_M, grasp.FINGERTIP_Z_M, 5):
                for y in np.linspace(grasp.FINGER_INNER_Y_M, grasp.FINGER_OUTER_Y_M, 3):
                    for x in np.linspace(*grasp.FINGER_X_RANGE_M, 3):
                        p = to_camera @ np.array([x, side * y, z, 1.0])
                        fingers.append((907.0 * p[0] / p[2] + 640.0, 906.0 * p[1] / p[2] + 360.0))
        fingers = np.array(fingers)
        for ahead, left in ((0.90, 0.0), (1.10, 0.0), (1.30, 0.0), (0.95, -0.12), (0.95, 0.12)):
            tag = (plan.invert(camera) @ plan.invert(plan.base_pose(0.0, 0.0, 0.0)) @ box_odom(ahead, left, 180.0)
                   @ tag_pose_box(GEOMETRY))[:3, 3]
            u, v, half = 907.0 * tag[0] / tag[2] + 640.0, 906.0 * tag[1] / tag[2] + 360.0, 906.0 * 0.05 / tag[2]
            overlap = ((fingers[:, 0] > u - half) & (fingers[:, 0] < u + half) & (fingers[:, 1] > v - half)
                       & (fingers[:, 1] < v + half))
            self.assertFalse(overlap.any(), (ahead, left, u, v))
            self.assertTrue(0 < v - half and v + half < 720, (ahead, left, v))

    def test_a_look_through_the_wrist_camera_finds_the_box_as_the_head_camera_does(self):
        joints, links = d1_ik.load_urdf()
        camera = plan.wrist_camera_base(joints, plan.look_pose_servo_deg(joints, links))
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = box_odom(0.92, 0.04, 182.0)
        pose_cam = plan.invert(camera) @ plan.invert(base) @ box @ tag_pose_box(GEOMETRY)
        found = TagDetection(GEOMETRY.tag_id, np.zeros((4, 2)), pose_cam, 0.1, 120.0)
        frames, scans = look(base, box)
        head = plan.make_fix(base, frames, scans)
        wrist = plan.make_fix(base, [(base, [found], camera)] * len(frames), scans)
        self.assertAlmostEqual(wrist.site.distance_m, head.site.distance_m, places=6)
        self.assertAlmostEqual(wrist.site.bearing_deg, head.site.bearing_deg, places=4)
        self.assertAlmostEqual(wrist.site.yaw_deg, head.site.yaw_deg, places=4)


class FrontCameraTests(unittest.TestCase):
    """The Go2's front camera as the look's camera (2026-10-22): fixed in the trunk, its fisheye lens calibrated
    (2026-10-24)."""

    @staticmethod
    def _margin_px(ahead, left, yaw_deg, base_z=plan.BASE_HEIGHT_M, pitch_deg=0.0):
        from demos.combiner_door import front_camera

        model = front_camera.camera_model()
        base = plan.base_pose(0.0, 0.0, 0.0, 0.0, math.radians(pitch_deg))
        base[2, 3] = base_z
        tag = (plan.invert(base @ plan.FRONT_CAMERA_POSE_BASE) @ box_odom(ahead, left, yaw_deg)
               @ tag_pose_box(GEOMETRY))
        half = GEOMETRY.tag_outer_size / 2
        corners = np.array([(tag @ np.array([dx, dy, 0.0, 1.0]))[:3]
                            for dx, dy in ((-half, half), (half, half), (half, -half), (-half, -half))])
        uv = model.project(corners)
        return float(min(uv[:, 0].min(), model.width - uv[:, 0].max(), uv[:, 1].min(), model.height - uv[:, 1].max()))

    def test_its_mount_is_the_front_camera_modules(self):
        from demos.combiner_door import front_camera

        np.testing.assert_allclose(plan.FRONT_CAMERA_POSE_BASE, front_camera.optical_pose_base())

    def test_a_look_through_the_front_camera_finds_the_box_as_the_head_camera_does(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = box_odom(0.95, 0.04, 182.0)
        pose_cam = plan.invert(plan.FRONT_CAMERA_POSE_BASE) @ plan.invert(base) @ box @ tag_pose_box(GEOMETRY)
        found = TagDetection(GEOMETRY.tag_id, np.zeros((4, 2)), pose_cam, 0.1, 150.0)
        frames, scans = look(base, box)
        head = plan.make_fix(base, frames, scans)
        front = plan.make_fix(base, [(base, [found], plan.FRONT_CAMERA_POSE_BASE)] * len(frames), scans)
        self.assertAlmostEqual(front.site.distance_m, head.site.distance_m, places=6)
        self.assertAlmostEqual(front.site.bearing_deg, head.site.bearing_deg, places=4)
        self.assertAlmostEqual(front.site.yaw_deg, head.site.yaw_deg, places=4)

    def test_the_tag_is_whole_from_the_nearest_look_back(self):
        """`FRONT_LOOK_MIN_ROOT_M` and further, within 0.25 m of straight ahead and 15 deg of square, standing 0.30-0.34 m
        up and pitched +-2 deg; and not from where the door pull used to look without walking (0.65-0.76 m)."""
        for ahead in (plan.FRONT_LOOK_MIN_ROOT_M, 1.14, 1.5):
            for left in (-0.25, 0.0, 0.25):
                for yaw in (165.0, 180.0, 195.0):
                    for z in (0.30, 0.34):
                        for pitch in (-2.0, 2.0):
                            self.assertGreaterEqual(self._margin_px(ahead, left, yaw, z, pitch), 20.0,
                                                    (ahead, left, yaw, z, pitch))
        for ahead in (0.65, 0.76):
            self.assertLess(self._margin_px(ahead, 0.0, 180.0), 0.0, ahead)

    def test_the_look_is_the_front_cameras_by_default(self):
        from demos.combiner_door.robot import node

        args = node.parse_args([])
        self.assertEqual(args.camera, "front")
        self.assertEqual(node.parse_args(["--camera", "wrist"]).camera, "wrist")

    def test_the_close_and_the_lock_follow_only_what_they_can(self):
        from demos.combiner_door.robot import node

        self.assertTrue(node.parse_args(["--stage", "pull", "--then_close"]).then_close)
        args = node.parse_args(["--task", "insert", "--stage", "insert", "--then_door", "pull", "--then_close"])
        self.assertTrue(args.then_close)
        self.assertEqual(node.parse_args(["--task", "insert", "--stage", "insert", "--lock"]).lock_from_deg, 45.0)
        for argv in (["--stage", "grip", "--then_close"], ["--task", "insert", "--stage", "insert", "--then_close"],
                     ["--stage", "pull", "--lock"], ["--task", "insert", "--stage", "insert", "--lock", "--then_door",
                                                     "pull"]):
            with self.subTest(argv=argv), self.assertRaises(SystemExit):
                node.parse_args(argv)


def approach_staging(box):
    """The close's staging pose in front of `box`'s open door (`approach.STAGE_ROOT_M`)."""
    from demos.combiner_door.robot import approach

    return approach.staging_pose("door", box)


class LinkedArm:
    """A dry run's arm as the node drives it (`node.py`): each command's goal through `plan.ArmSender` (joint moves as
    timed paths, the rest followed) to `arm_link.SimulatedLink`, the link's player on a model servo. `q`, `gripper`,
    `stall_units` and `gripper_rate` are the servo's; setting `q` moves the arm under the link's command (a lock face
    pushing it back)."""

    def __init__(self, start, gripper: float = 40.8, gripper_rate: float = 70.0):
        self.link = SimulatedLink(start, gripper, gripper_rate)
        self.sender = plan.ArmSender()

    def drive(self, command, attempt) -> None:
        joint = command.phase in attempt.joint_phases or command.phase == "fold"
        self.link.handle(self.sender.message(command.arm_servo_deg, command.gripper_units, joint, self.link.arm.q))

    @property
    def q(self):
        return self.link.arm.q

    @q.setter
    def q(self, value):
        self.link.arm.q = np.asarray(value, dtype=float).copy()

    @property
    def gripper(self):
        return self.link.arm.gripper

    @property
    def stall_units(self):
        return self.link.arm.stall_units

    @stall_units.setter
    def stall_units(self, value):
        self.link.arm.stall_units = value

    @property
    def gripper_rate(self):
        return self.link.arm.gripper_rate

    @gripper_rate.setter
    def gripper_rate(self, value):
        self.link.arm.gripper_rate = value


class AttemptTests(unittest.TestCase):
    def run_attempt(self, stage, timing=None, max_steps=1200, arm=None, watch=None, expect_abort=False):
        """A dry attempt with the arm link's profile driving a model servo (`SimulatedArm`), the jaws stalling on the
        lever while the hand holds it. `watch(attempt, command, arm, base)` is called each step. Returns the attempt,
        the phases in order, the arm's last joints and (base, jaw centre, grasp point) at the arrival; with
        `expect_abort`, the attempt and the abort's reason."""
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = door_box(0.92, 0.0, 181.0)
        frames, scans = look(base, box)
        fix = plan.make_fix(base, frames, scans)
        attempt = plan.Attempt(fix, stage, timing or plan.scaled_timing(1.0, 30.0))
        arm = arm or LinkedArm(d1_hardware.REST_SERVO_DEG)
        joints, _ = d1_ik.load_urdf()
        phases, at_arrive, moving = [], None, (0.0, 0.0, 0.0)
        for step in range(max_steps):
            arm.stall_units = LEVER_STALL_UNITS if attempt.phase in plan.HOLDING_PHASES else None
            obs = plan.Observation(0.1 * step, base, 0.0, arm.q.copy(), 0.0, gripper_units=arm.gripper)
            command = attempt.step(obs)
            if command.finished:
                break
            if expect_abort and command.abort:
                return attempt, command.abort
            self.assertEqual(command.abort, "", (stage, command.phase))
            if not phases or phases[-1] != command.phase:
                phases.append(command.phase)
            if command.arm_servo_deg is not None:
                arm.drive(command, attempt)
                attempt.sent(command)
            if watch is not None:
                watch(attempt, command, arm, base)
            if command.walking:
                self.assertLessEqual(abs(command.base[0]), plan.WALK_CAP[0] + 1e-9)
            base, moving = walk(base, command.base if command.walking else None, moving)
            if command.phase == "arrive":
                pos, _ = d1_ik.tool_pose(joints, d1_ik.from_servo_deg(arm.q)[0], body="Link6", offset=JAW_CENTRE_LINK6)
                at_arrive = (attempt.base_script(base), pos, attempt.fix.site.grasp_point_m())
        else:
            self.fail(f"{stage} did not finish")
        if expect_abort:
            self.fail(f"{stage} did not abort")
        return attempt, phases, arm.q, at_arrive

    def test_every_stage_runs_dry_to_the_fold(self):
        for stage in plan.STAGES[2:]:
            with self.subTest(stage=stage):
                attempt, phases, q, at_arrive = self.run_attempt(stage)
                expected = [p.name for p in plan.attempt_phases(stage, plan.scaled_timing(1.0, 30.0))]
                self.assertEqual(phases, expected)
                self.assertLess(np.max(np.abs(q - np.asarray(plan.FOLD_SERVO_DEG))), plan.FOLD_ARRIVED_DEG + 0.05)
                if at_arrive is not None:
                    base_s, jaw_b, grasp_s = at_arrive
                    jaw_s = plan.transform(base_s, jaw_b)[0]
                    self.assertLess(np.linalg.norm(jaw_s - np.asarray(grasp_s)), 0.015, stage)

    def test_a_slow_starting_walk_is_finished_by_corrective_steps(self):
        # As in classic walk on the dog (2026-10-20): the legs answer a Move only after 0.4 s of them and stop dead,
        # so the first walk in stopped 8 cm short. Corrective steps take it within the tolerance; the attempt then
        # re-anchors where the dog stopped.
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.make_fix(base, *look(base, door_box(0.92, 0.0, 180.0)))
        attempt = plan.Attempt(fix, "walk")
        q, on = np.asarray(d1_hardware.REST_SERVO_DEG, dtype=float), 0.0
        for step in range(3000):
            command = attempt.step(plan.Observation(0.1 * step, base, 0.0, q, 0.0))
            self.assertEqual(command.abort, "", command.phase)
            if command.finished:
                break
            attempt.sent(command)
            q = np.asarray(plan.step_towards(q, command.arm_servo_deg))
            on = on + 0.1 if command.walking else 0.0
            if command.walking and on > 0.4:
                x, y, yaw = plan.planar(base)
                base = plan.base_pose(x + 0.1 * command.base[0] * math.cos(yaw), y + 0.1 * command.base[0] * math.sin(yaw),
                                      yaw + 0.1 * command.base[2])
        steps = [e for e in attempt.events if e.get("phase") == "walk_in"]
        self.assertGreaterEqual(len(steps), 2)                                  # it stopped short and stepped again
        self.assertLessEqual(abs(steps[-1]["short_m"]), plan.WALK_IN_TOLERANCE_M + 0.02)
        anchored = [e for e in attempt.events if e.get("phase") == "reanchor"]
        self.assertTrue(anchored and anchored[0]["ok"], anchored)

    def test_after_a_refit_the_reach_check_ignores_the_tag_guard(self):
        # On the dog (2026-10-20) the walk ended 1.5 cm past the stance and the refit moved the box 3.5 cm along its
        # normal; the look's tag guard then read that 3.5 cm as a tag 3 cm off its door, and aborted.
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = door_box(0.85, 0.0, 180.0)
        fix = plan.make_fix(base, *look(base, box))
        stance = fix.script_odom @ plan.pose(None, (0.015, 0.0, plan.BASE_HEIGHT_M - fix.script_odom[2, 3]))
        actual_box = box @ plan.pose(None, (-0.035, 0.0, 0.0))        # 3.5 cm further than the look had it
        rng = np.random.default_rng(9)
        refined, fit = plan.refine_fix(fix, [(stance, face_scan(stance, actual_box, rng)) for _ in range(10)])
        self.assertTrue(fit.ok)
        self.assertGreater(abs(fit.depth), plan.FACE_BEHIND_TAG_LIMIT_M)
        moved = plan.reanchor(refined, stance)
        self.assertFalse(moved.in_range()[0])
        ok, text = moved.in_range(check_tag=False)
        self.assertTrue(ok, text)

    def test_reanchoring_keeps_the_box_where_it_is(self):
        base = plan.base_pose(0.3, -0.2, 0.4)
        box = base @ plan.pose(None, (0, 0, -plan.BASE_HEIGHT_M)) @ door_box(0.92, 0.0, 182.0)
        fix = plan.make_fix(base, *look(base, box))
        stopped = base @ plan.pose(plan.rot_z(math.radians(3.0)), (0.15, 0.01, 0.0))
        moved = plan.reanchor(fix, stopped)
        self.assertLess(np.linalg.norm(moved.box_odom[:2, 3] - fix.box_odom[:2, 3]), 1e-6)
        self.assertAlmostEqual(moved.site.yaw_deg % 360, (fix.site.yaw_deg - 3.0) % 360, places=4)
        self.assertAlmostEqual(moved.site.distance_m, math.hypot(0.92 + plan.STANCE_BACK_M - 0.15,
                                                                 plan.STANCE_RIGHT_M - 0.01), delta=0.01)

    def test_the_walk_in_ends_short_by_the_carry(self):
        attempt, _, _, _ = self.run_attempt("walk")
        walked = [e for e in attempt.events if e.get("phase") == "walk_settle"]
        self.assertTrue(walked)

    def test_a_loaded_arm_aborts_the_pull(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = door_box(0.92, 0.0, 180.0)
        fix = plan.make_fix(base, *look(base, box))
        attempt = plan.Attempt(fix, "grip", plan.scaled_timing(1.5, 30.0))
        q = np.asarray(d1_hardware.REST_SERVO_DEG, dtype=float)
        stuck, why, moving = None, "", (0.0, 0.0, 0.0)
        for step in range(1500):
            command = attempt.step(plan.Observation(0.1 * step, base, 0.0, q if stuck is None else stuck, 0.0))
            if command.abort or command.finished:
                why = command.abort
                break
            attempt.sent(command)
            q = np.asarray(plan.step_towards(q, command.arm_servo_deg))
            base, moving = walk(base, command.base if command.walking else None, moving)
            if command.phase == "squeeze" and stuck is None:
                # The arm stops following: 15 deg off on the shoulder from here on.
                stuck = q + np.array([0.0, 15.0, 0.0, 0.0, 0.0, 0.0])
        self.assertIn("from where the IK wants it", why)

    def test_stale_feedback_and_tilt_abort(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = door_box(0.92, 0.0, 180.0)
        fix = plan.make_fix(base, *look(base, box))
        q = np.asarray(d1_hardware.REST_SERVO_DEG, dtype=float)
        self.assertIn("arm feedback", plan.Attempt(fix, "unfold").step(plan.Observation(0.0, base, 0.0, q, 2.0)).abort)
        tilted = base @ plan.pose(plan.rpy_matrix(0.0, math.radians(20.0), 0.0))
        self.assertIn("tilted", plan.Attempt(fix, "unfold").step(plan.Observation(0.0, tilted, 0.0, q, 0.0)).abort)
        self.assertIn("joysticks",
                      plan.Attempt(fix, "unfold").step(plan.Observation(0.0, base, 0.0, q, 0.0, web_teleop=True)).abort)

    def test_recovery_opens_backs_out_and_folds(self):
        attempt, _, _, _ = self.run_attempt("grip")
        attempt.last_command = plan.Command("squeeze", hand_target_script=attempt.fix.site.grasp_point_m(),
                                            approach_script=attempt.fix.site.approach_unit(0.0),
                                            object_axis_script=attempt.fix.site.lever_axis_unit(0.0, 0.0))
        attempt.recover()
        self.assertEqual([p.name for p in attempt.script.phases], ["release", "ungrasp", "home", "tuck", "fold"])

    def test_letting_go_keeps_the_fingers_off_the_lever_and_its_hub(self):
        # 2026-10-20, live: after the pull the hand went straight from the lever to the home point and the fingers,
        # still behind the lever, pushed the door by its 63 mm hub. On the same model the script's straight "back"
        # runs the fingers into the lever and the hub; the ungrasp backs them straight off.
        def closest(phases_from=None):
            worst = {}

            def watch(attempt, command, arm, base):
                if attempt._pull is None or command.phase not in ("release", "ungrasp", "home", "back"):
                    return
                door = viz.door_pose(perception.site_pose(attempt.fix.site), attempt.commanded_door_deg())
                points = plan.transform(plan.invert(door), finger_points(arm.q, attempt.base_script(base)))
                points[:, 2] -= plan.GRASP_RAISE_M      # the lever where the jaws close on it (the dog's, Lukas)
                lever, hub = lever_and_hub_clearance(points)
                w = worst.setdefault(command.phase, [math.inf, math.inf])
                w[:] = [min(w[0], float(lever.min())), min(w[1], float(hub.min()))]

            original = plan.attempt_phases
            if phases_from is not None:
                plan.attempt_phases = phases_from
            try:
                self.run_attempt("pull", watch=watch)
            finally:
                plan.attempt_phases = original
            return worst

        new = closest()
        self.assertEqual(set(new), {"release", "ungrasp", "home"})
        for phase, (lever, hub) in new.items():
            with self.subTest(phase=phase):
                self.assertGreater(lever, 0.010)        # the open jaws clear the lever by 11.6 mm a side
                self.assertGreater(hub, 0.030)

        original = plan.attempt_phases

        def script_ending(stage, timing=None, fold=True):
            """The previous ending: the door script's release, then straight to the home point, then the fold."""
            phases = original(stage, timing, fold)
            names = [p.name for p in phases]
            back = demo_script.Phase("back", timing.back_s, path=lambda st, u: plan.home_at(st["base"]),
                                     jaw_m=demo_script.JAW_LEVER_OPEN_M)
            return phases[:names.index("release")] + [demo_script.door_phases(timing)[-2], back, phases[-1]]

        old = closest(script_ending)
        # Into the lever or the hub on the way home as the link's profile drove it (a straight line in joint space);
        # sent as the holster draw's paced joint move (`plan.ArmSender`, 2026-10-24) it passes 7 mm off the lever,
        # still short of the 10 mm the ungrasp keeps.
        self.assertLess(min(old["back"]), 0.010)

    def test_the_jaws_open_fully_for_every_handle_grab_and_the_key_keeps_its_own(self):
        """Lukas, 2026-10-25: "I want it open fully for all handle grabs": open on the handle is the widest a command
        opens the jaws (`d1_hardware.GRIPPER_UNITS_OPEN`), the pull's reach and its letting go; the key's attempts
        keep the script's opening for the key and the holster."""
        sent = {}

        def watch(attempt, command, arm, base):
            if command.gripper_units is not None:
                sent.setdefault(command.phase, set()).add(round(command.gripper_units, 2))

        self.run_attempt("pull", timing=plan.scaled_timing(1.0, 60.0), watch=watch)
        for phase in ("approach", "settle", "enter", "arrive", "ungrasp"):
            self.assertEqual(sent[phase], {d1_hardware.GRIPPER_UNITS_OPEN}, phase)
        self.assertEqual(sent["pull"], {plan.GRIPPER_SHUT_UNITS})
        self.assertEqual(min(sent["grip"]), plan.GRIPPER_SHUT_UNITS)       # the script's ramp, fully open to shut
        self.assertEqual(max(sent["grip"]), d1_hardware.GRIPPER_UNITS_OPEN)
        self.assertEqual(min(sent["release"]), LEVER_STALL_UNITS)           # from where they hold the lever
        self.assertEqual(max(sent["release"]), d1_hardware.GRIPPER_UNITS_OPEN)
        from demos.combiner_key.robot import insert_plan, key_plan

        for cls in (insert_plan.InsertAttempt, key_plan.KeyAttempt):
            self.assertFalse(cls.handle_jaws, cls)
        self.assertTrue(key_plan.DoorClose.handle_jaws)

    def test_the_arm_stays_on_its_goal_through_a_60_deg_pull_and_hold(self):
        # The base carries on turning after the pull's twist stops; held at the pull's angle, the hand's goal swung 8.9
        # deg from the arm at the hold's start (60 deg), near the loaded-arm check's 10. The hold follows the turn.
        worst = [0.0]

        def watch(attempt, command, arm, base):
            if command.phase in ("pull", "hold"):
                self.assertEqual(command.ik_status, "pose")
                worst[0] = max(worst[0], float(np.max(np.abs(np.asarray(command.arm_servo_deg) - arm.q))))

        attempt, _, _, _ = self.run_attempt("pull", timing=plan.scaled_timing(1.0, 60.0), watch=watch)
        self.assertLess(worst[0], plan.ARM_LAG_DEG / 2)
        self.assertGreaterEqual(attempt._pull["followed"], 60.0)
        self.assertLessEqual(attempt._pull["followed"], 60.0 + plan.HOLD_FOLLOW_DEG)

    def test_dwells_end_when_the_arm_is_there(self):
        # The stand, the settle and the arrival waited out their clocks: 7 s of the live pull's reach (2026-10-20).
        steps = {}
        self.run_attempt("reach", watch=lambda attempt, command, arm, base: steps.update(
            {command.phase: steps.get(command.phase, 0) + 1}))
        took = {phase: 0.1 * n for phase, n in steps.items()}
        timing = plan.scaled_timing(1.0, 30.0)
        for phase, clock in (("stand", timing.stand_s), ("settle", timing.settle_s), ("arrive", timing.arrive_s)):
            with self.subTest(phase=phase):
                self.assertLess(took[phase], clock - 0.3)
                self.assertGreaterEqual(took[phase], plan.DWELL_MIN_S - 1e-9)
        self.assertLess(took["unfold"], plan.UNFOLD_S - 1.0)            # 40 deg/s in joint space
        self.assertLess(took["fold"], plan.FOLD_S - 1.0)

    def test_an_arm_that_does_not_get_there_waits_out_the_clock(self):
        # 3 deg off on the wrist for good: the settle runs its whole clock, as every phase did before.
        import dataclasses

        stuck, phase, steps = LinkedArm(d1_hardware.REST_SERVO_DEG), [""], {}
        real_drive = stuck.drive

        def drive(command, attempt):
            off = np.array([0.0, 0.0, 0.0, 0.0, 3.0, 0.0]) if phase[0] == "settle" else 0.0
            real_drive(dataclasses.replace(command, arm_servo_deg=list(np.asarray(command.arm_servo_deg) + off)),
                       attempt)

        def watch(attempt, command, arm, base):
            phase[0] = attempt.phase
            steps[command.phase] = steps.get(command.phase, 0) + 1

        stuck.drive = drive
        self.run_attempt("reach", arm=stuck, watch=watch)
        self.assertAlmostEqual(0.1 * steps["settle"], plan.scaled_timing(1.0, 30.0).settle_s, delta=0.15)

    def test_the_hand_leaves_the_lever_only_once_the_jaws_are_open(self):
        slow = LinkedArm(d1_hardware.REST_SERVO_DEG, gripper_rate=12.0)
        opened_at = []

        def watch(attempt, command, arm, base):
            if command.phase == "ungrasp" and not opened_at:
                opened_at.append(arm.gripper)

        self.run_attempt("grip", arm=slow, watch=watch)
        self.assertGreaterEqual(opened_at[0], plan.GRIPPER_OPEN_UNITS - plan.JAW_OPEN_TOLERANCE_UNITS - 1.5)

        def jammed(attempt, command, arm, base):
            if command.phase == "release":
                arm.gripper_rate = 0.0                   # the jaws do not open
        attempt, why = self.run_attempt("grip", watch=jammed, expect_abort=True)
        self.assertIn("the jaws did not open", why)
        self.assertEqual(attempt.phase, "release")

    def test_letting_go_of_the_handle_keeps_the_height_it_was_held_at(self):
        """Lukas, 2026-10-25: the jaws turned the handle down as the hand left it. The arm sags 11-15 mm below its goal
        on the handle (every release of 2026-10-24/25); the release's goal at the hand's measured height pressed the
        lever down by that much while the jaws held it, and the hand dropped onto it once they opened. A sagging arm
        (Joint2 1.5 deg below its servo, 12 mm at the jaw centre, reported where it is) now stays at the height it
        held the lever at while the jaws open, slowly and from where they grip it, and stays there with them open
        before it backs out."""
        from unittest import mock

        sag = np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0])

        class Sagging(LinkedArm):
            @property
            def q(self):
                return self.link.arm.q + sag

            @q.setter
            def q(self, value):
                self.link.arm.q = np.asarray(value, dtype=float) - sag

        joints, _ = d1_ik.load_urdf()
        height = lambda q: d1_ik.tool_pose(joints, d1_ik.from_servo_deg(q)[0], body="Link6",
                                           offset=JAW_CENTRE_LINK6)[0][2]

        def let_go():
            seen = {"z": [], "units": [], "release_steps": 0}

            def watch(attempt, command, arm, base):
                if command.phase in ("release", "ungrasp"):
                    seen.setdefault("z0", height(arm.q))
                    seen["z"].append(height(arm.q))
                if command.phase == "release":
                    seen["release_steps"] += 1
                    seen["units"].append(command.gripper_units)
                    seen["opened"] = attempt._jaw_from
            attempt, _, _, _ = self.run_attempt("pull", timing=plan.scaled_timing(1.0, 60.0), arm=Sagging(
                d1_hardware.REST_SERVO_DEG), watch=watch)
            return attempt, seen

        attempt, seen = let_go()
        let_go_event = next(e for e in attempt.events if e.get("phase") == "let_go")
        self.assertTrue(10.0 < let_go_event["sag_mm"] < 20.0, let_go_event)   # the sag, and the hold's tracking lag
        drop = 1000 * (seen["z0"] - min(seen["z"]))
        self.assertLess(drop, 3.0)
        # The jaws: from where they gripped the lever to fully open in `LET_GO_RAMP_S`, then still with the hand.
        units = seen["units"]
        self.assertAlmostEqual(units[0], LEVER_STALL_UNITS, delta=0.1)
        self.assertTrue(all(b >= a for a, b in zip(units, units[1:])))
        ramp = round(plan.LET_GO_RAMP_S / 0.1)
        self.assertAlmostEqual(units[ramp], d1_hardware.GRIPPER_UNITS_OPEN, delta=0.01)
        self.assertLess(units[ramp - 2], d1_hardware.GRIPPER_UNITS_OPEN - 5.0)
        self.assertGreaterEqual(0.1 * seen["release_steps"], plan.LET_GO_S - 0.1)

        with mock.patch.object(plan, "LET_GO_SAG_MAX_M", 0.0):              # held at its measured height
            _, before = let_go()
        self.assertGreater(1000 * (before["z0"] - min(before["z"])), 10.0)

    def test_the_release_lets_go_from_where_the_arm_is(self):
        # `--release` from the pose the arm was left holding the lever in (2026-10-20).
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.make_fix(base, *look(base, door_box(0.92, 0.0, 180.0)))
        held = [29.8, 27.9, 12.3, 38.2, -50.7, 67.3]
        attempt = plan.release(fix, base, held)
        self.assertEqual([p.name for p in attempt.script.phases], ["release", "ungrasp", "home", "tuck", "fold"])
        arm = LinkedArm(held, gripper=LEVER_STALL_UNITS)
        joints, _ = d1_ik.load_urdf()
        start = np.asarray(attempt._held["hand"])
        approach = np.asarray(attempt._held["approach"])
        for step in range(600):
            command = attempt.step(plan.Observation(0.1 * step, base, 0.0, arm.q.copy(), 0.0,
                                                    gripper_units=arm.gripper))
            self.assertEqual(command.abort, "", command.phase)
            if command.finished:
                break
            arm.drive(command, attempt)
            attempt.sent(command)
            if command.phase in ("release", "ungrasp"):
                jaw, _ = d1_ik.tool_pose(joints, d1_ik.from_servo_deg(arm.q)[0], body="Link6", offset=JAW_CENTRE_LINK6)
                off = plan.transform(attempt.base_script(base), [jaw])[0] - start
                sideways = off - (off @ approach) * approach
                self.assertLess(np.linalg.norm(sideways), 0.004, command.phase)   # straight back along the approach
        else:
            self.fail("the release did not finish")
        self.assertLess(np.max(np.abs(arm.q - np.asarray(plan.FOLD_SERVO_DEG))), plan.ARRIVED_DEG + 0.5)

    def test_recovery_before_the_reach_only_folds(self):
        # Aborted while unfolding (2026-10-20, the arm's reports went quiet): the hand is nowhere near the lever, so
        # the recovery is the fold alone, not a Cartesian open-and-back-out.
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.make_fix(base, *look(base, door_box(0.92, 0.0, 180.0)))
        attempt = plan.Attempt(fix, "grip", plan.scaled_timing(1.5, 30.0))
        self.assertEqual(attempt.arm.grasp_note, "pitch 0 deg, roll +1")       # planned before anything moves
        q = np.asarray(d1_hardware.REST_SERVO_DEG, dtype=float)
        for step in range(5):
            command = attempt.step(plan.Observation(0.1 * step, base, 0.0, q, 0.0))
            q = np.asarray(plan.step_towards(q, command.arm_servo_deg))
        self.assertEqual(command.phase, "unfold")
        attempt.recover()
        self.assertEqual([p.name for p in attempt.script.phases], ["tuck", "fold"])

    def test_the_fold_keeps_the_arm_off_the_lidar(self):
        """From the homes the dog measured as its two live pulls began to fold (2026-10-24), the arm link's paths to
        the fold (`plan.joint_path`) stay off the trunk and the lidar's boxes through `tuck`; straight to the fold they
        cross the cage's (both stalled there on the dog, the shoulder at -40 to -50 deg)."""
        joints, _ = d1_ik.load_urdf()

        def worst(legs):
            margin, q0 = math.inf, legs[0]
            for goal in legs[1:]:
                for q in plan.joint_path(q0, goal):
                    margin = min(margin, grasp.proxy_margin(joints, d1_ik.from_servo_deg(q)[0]))
                q0 = goal
            return margin

        fold = np.asarray(plan.FOLD_SERVO_DEG)
        for home in ([14, 21, 23, 43, -65, 69], [13, 21, 24, 41, -65, 69], [40, 20, 37, 36, -57, 70]):
            home = np.asarray(home, dtype=float)
            self.assertLess(worst([home, fold]), 0.0, home)
            self.assertGreater(worst([home, plan.tuck_servo_deg(), fold]), 0.04, home)

    def test_the_fold_waits_for_the_arm(self):
        # An arm slower than the clock: the fold phase runs on until it is folded (2026-10-20: on the clock alone it
        # ended 23 deg short), and gives up after JOINT_TIMEOUT_S.
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.make_fix(base, *look(base, door_box(0.92, 0.0, 180.0)))
        for slow, stuck in ((1.0, False), (4.0, True)):
            attempt = plan.Attempt(fix, "unfold")
            q, why = np.asarray(d1_hardware.REST_SERVO_DEG, dtype=float), ""
            for step in range(3000):
                command = attempt.step(plan.Observation(0.1 * step, base, 0.0, q, 0.0))
                if command.finished or command.abort:
                    why = command.abort
                    break
                attempt.sent(command)
                frozen = stuck and command.phase == "fold"
                q = q if frozen else np.asarray(plan.step_towards(q, command.arm_servo_deg, slow))
            with self.subTest(stuck=stuck):
                if stuck:
                    self.assertIn("did not reach the fold", why)
                else:
                    self.assertEqual(why, "")
                    self.assertLess(np.max(np.abs(q - np.asarray(plan.FOLD_SERVO_DEG))), plan.JOINT_ARRIVED_DEG)

    def test_a_box_a_little_far_needs_no_walk(self):
        # 2026-10-20: 0.73 m off, a walk of 4 cm (under what sport mode can do) was refused with "back 0.16 m". Now
        # 2.5 cm past the stance (`plan.STANCE_*`), short of the arm's reach.
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.make_fix(base, *look(base, door_box(plan.NOMINAL_DISTANCE_M + 0.025, 0.0, 180.0)))
        ok, text = fix.in_range()
        self.assertEqual(fix.walk_in_m, 0.0)
        self.assertTrue(ok, text)

    def run_close(self, door_deg=76.0, drift=(0.0, 0.0, 0.0), expect_abort=False, push_counted=1.0, frame_stops=False,
                  pulled_ajar_deg=0.0, shut_from_unlock=False):
        """A dry 60 deg pull, the door run on to `door_deg`, then closed from a look at its tag (`key_plan.DoorClose`)
        from the door pull's staging pose in front of the open door, the real box `drift` (x m, y m, yaw deg) from where
        the pull's odometry has it (the odometry's error over the pull). The lidar reads the door at the script's angle
        through the close. Returns the close's attempt, its phases, the arm's last joints, the base (script frame), the
        IK's statuses, the jaws' worst miss of their goal through grip, push and hold, the grip's goal against the real
        handle (m) and the real box (odom), and the hand's goal's furthest from the grip's in the base frame through push
        and hold (m), and the grip's goal in the base frame; with `expect_abort`, the attempt and the reason. `push_counted`: the share of the base's translation
        the odometry counts through push and hold (the dog's fell 24 cm short of the hinge's arc over 70 deg).
        `frame_stops`: the base stands once its turn has the door shut (the door on its frame, the hand on it).
        `pulled_ajar_deg`: the pull took the door that far ajar as shut (after an unlock); `shut_from_unlock`: the close's
        shut from the unlock's look of the shut door (`plan.shut_reference`), as `node.py` takes it."""
        from demos.combiner_door.robot import approach
        from demos.combiner_key.robot import key_plan

        timing = plan.scaled_timing(1.0, 60.0)
        pull, _, q, _ = self.run_attempt("pull", timing=timing)
        dx, dy, dyaw = drift
        box = plan.pose(plan.rot_z(math.radians(dyaw)), (dx, dy, 0.0)) @ door_box(0.92, 0.0, 181.0)
        opened = box @ plan.about_hinge(door_deg)          # the box whose shut door stands where the open one does
        # The node's look, from the staging pose (`approach.STAGE_ROOT_M`), the box's side in the lidar too.
        base = plan.base_pose(*approach.staging_pose("door", opened))
        frames, scans = look(base, opened)
        rng = np.random.default_rng(2)
        scans = [(b, np.concatenate([pts, side_scan(b, box, rng)])) for b, pts in scans]
        hinge = pull.box_odom() @ np.append(GEOMETRY.hinge, 1.0)
        fix = plan.make_fix(base, frames, scans, pulled_hinge_xy=(hinge[0], hinge[1]))
        ok, text = fix.in_range()
        self.assertTrue(ok, text)
        pulled = pull.box_odom() @ plan.about_hinge(pulled_ajar_deg)
        if shut_from_unlock:
            pulled, _, _ = plan.shut_reference(pulled, pull.box_odom())
        attempt = key_plan.DoorClose(fix, "pull", timing, pulled_box_odom=pulled)
        joints, _ = d1_ik.load_urdf()
        moving, arm, rng = (0.0, 0.0, 0.0), LinkedArm(q), np.random.default_rng(1)
        phases, statuses, miss, grip_off, grip_b, goal_drift = [], set(), 0.0, None, None, 0.0
        for step in range(4000):
            arm.stall_units = LEVER_STALL_UNITS if attempt.phase in attempt.holding_phases else None
            reading = mid360.DoorReading(attempt.commanded_door_deg(), 100, 0.005, 0.0)
            # The settle's lidar frames, of the open door, for the refit after the walk up.
            scan = (base, face_scan(base, opened, rng)) if attempt.phase == "walk_settle" else None
            command = attempt.step(plan.Observation(0.1 * step, base, 0.0, arm.q.copy(), 0.0, scan=scan,
                                                    door_reading=reading, gripper_units=arm.gripper))
            if command.finished:
                break
            if expect_abort and command.abort:
                return attempt, command.abort
            self.assertEqual(command.abort, "", command.phase)
            if not phases or phases[-1] != command.phase:
                phases.append(command.phase)
            if command.phase == "grip" and grip_off is None:
                handle = np.asarray(perception.site_from_pose(plan.invert(attempt.fix.script_odom) @ opened)
                                    .grasp_point_m()) + (0.0, 0.0, plan.GRASP_RAISE_M)       # where the jaws close on it
                grip_off = float(np.linalg.norm(np.asarray(command.hand_target_script) - handle))
            goal_b = plan.transform(plan.invert(attempt.base_script(base)), command.hand_target_script)[0] \
                if command.hand_target_script is not None else None
            if command.phase == "grip":
                grip_b = goal_b
            if command.phase in ("push", "hold"):
                goal_drift = max(goal_drift, float(np.linalg.norm(goal_b - grip_b)))
            if command.phase in ("grip", "push", "hold"):
                statuses.add(command.ik_status)
                pos, _ = d1_ik.tool_pose(joints, d1_ik.from_servo_deg(arm.q)[0], body="Link6", offset=JAW_CENTRE_LINK6)
                jaw = plan.transform(attempt.base_script(base), pos)[0]
                miss = max(miss, float(np.linalg.norm(jaw - np.asarray(command.hand_target_script))))
            if command.arm_servo_deg is not None:
                arm.drive(command, attempt)
                attempt.sent(command)
            walking = command.walking and not (frame_stops and command.phase in ("push", "hold") and
                                               attempt._door_from_base(plan.planar(attempt.base_script(base))) <= 0.0)
            if not walking and frame_stops and command.phase == "push":
                moving = (0.0, 0.0, 0.0)                               # stopped by the frame: no carry
            base, moving = walk(base, command.base if walking else None, moving,
                                counted=push_counted if command.phase in ("push", "hold") else 1.0)
        else:
            self.fail("the close did not finish")
        if expect_abort:
            self.fail("the close did not abort")
        return (attempt, phases, arm.q, plan.planar(attempt.base_script(base)), statuses, miss, grip_off, box, goal_drift,
                grip_b)

    def test_the_door_is_closed_from_a_look_at_its_tag(self):
        """Lukas, 2026-10-25: "close the door - first by grabbing the handle again, then pushing it closed while
        moving", and "this phase should use the apriltag to grab it just like on the open" (the close from the pull's
        fix had reached 15 cm short of the handle). From a look at the open door's tag, swung on past the pull's 60 deg
        (15-30 on the dog) and the box off where the pull's odometry has it (12 cm, as the dog's under-counted the
        pull's turn round the hinge): the walk up to the stance in front of it, the handle grabbed where it is with the
        hand's pose held, the door walked to `CLOSE_TO_DEG` round the hinge, let go, folded."""
        from demos.combiner_key.robot import key_plan

        hinge = np.append(GEOMETRY.hinge, 1.0)
        for door_deg, drift in ((64.0, (0.0, 0.0, 0.0)), (76.0, (0.10, -0.07, 0.0)), (88.0, (0.10, -0.07, 1.5))):
            with self.subTest(door_deg=door_deg, drift=drift):
                attempt, phases, q, base, statuses, miss, grip_off, box, _, _ = self.run_close(door_deg, drift)
                self.assertEqual(phases, [p.name for p in plan.close_phases(attempt)])
                check = attempt.close_check()
                self.assertTrue(check["closed"], check)
                self.assertLess(abs(check["from_deg"] - door_deg), 1.0 + abs(drift[2]))
                self.assertTrue(attempt.refit.ok, attempt.refit.why)          # the refit on the open door's face
                self.assertAlmostEqual(check["hinge_moved_mm"], 1000 * math.hypot(*drift[:2]), delta=12.0)
                self.assertLess(grip_off, 0.003)                    # the jaws sent to the real handle
                self.assertLess(np.linalg.norm((attempt.box_odom() @ hinge - box @ hinge)[:2]), 0.012)
                self.assertEqual(statuses, {"pose"})
                self.assertLess(miss, 0.006)
                # Walked back round the hinge to its end, past shut (`CLOSE_PAST_DEG`).
                self.assertLess(abs(attempt._door_from_base(base) - (plan.CLOSE_TO_DEG - plan.CLOSE_PAST_DEG)), 8.0)
                self.assertLess(np.max(np.abs(q - np.asarray(plan.FOLD_SERVO_DEG))), plan.FOLD_ARRIVED_DEG + 0.05)

    def test_the_open_doors_look_is_guarded_by_its_hinge_not_its_edges(self):
        """2026-10-25 02:26 UTC on the dog: the tag on the door 79 deg open, refused as a tag on a stand (42% of the face
        past the door's edges: the box's side, in line with the door near 90 deg). The close's look checks its hinge
        against the pull's instead; a tag elsewhere (the dock's) is still refused."""
        rng = np.random.default_rng(3)
        box = door_box(0.92, 0.0, 181.0)
        hinge = box @ np.append(GEOMETRY.hinge, 1.0)
        for door_deg in (79.0, 88.0):
            with self.subTest(door_deg=door_deg):
                opened = box @ plan.about_hinge(door_deg)
                base = plan.base_pose(*approach_staging(opened))
                frames, scans = look(base, opened)
                scans = [(b, np.concatenate([pts, side_scan(b, box, rng)])) for b, pts in scans]
                plain = plan.make_fix(base, frames, scans)
                self.assertFalse(plain.on_a_door())                         # as on the dog
                self.assertGreater(plain.beyond_edges, plan.BEYOND_LIMIT)
                fix = plan.make_fix(base, frames, scans, pulled_hinge_xy=(hinge[0], hinge[1]))
                self.assertTrue(fix.on_a_door())
                self.assertLess(fix.hinge_off_m(), 0.02)
                # The same look with the pull's hinge 0.5 m away: not the door the pull opened.
                far = plan.make_fix(base, frames, scans, pulled_hinge_xy=(hinge[0] + 0.5, hinge[1]))
                ok, text = far.in_range()
                self.assertFalse(ok)
                self.assertIn("not on the door the pull opened", text)

    def test_the_push_keeps_the_hand_where_it_gripped_whatever_the_odometry_counts(self):
        """2026-10-25 02:37 UTC on the dog: through the push the odometry fell 24 cm short of the hinge's arc, the hand's
        goal went 18 cm sideways after it and the IK ran out at 19 deg. Holding the door, the base is on the arc by its
        turn: with the odometry counting 70% of the walk, the goal stays where it gripped, the whole pose throughout."""
        for counted in (1.0, 0.7):
            with self.subTest(counted=counted):
                attempt, _, _, base, statuses, miss, _, _, drift, _ = self.run_close(76.0, push_counted=counted)
                check = attempt.close_check()
                self.assertTrue(check["closed"], check)
                self.assertEqual(statuses, {"pose"})
                self.assertLess(drift, 0.02)
                self.assertLess(miss, 0.006)
                if counted < 1.0:
                    self.assertGreater(check["odometry_off_arc_mm"], 100.0)

    def test_the_push_goes_past_shut_and_ends_where_the_frame_stops_the_door(self):
        """Lukas, 2026-10-25: "the closing stops 20 degrees from fully closed. it should attempt to close it the full 90
        degrees and accept that the spring will push it back out 5 or 10" (the door lagged the base 16-19 deg through
        the push). The push aims `CLOSE_PAST_DEG` past shut; a frame that stops the base at shut ends it there as done,
        not as an abort, the hand's whole pose held."""
        attempt, _, _, base, statuses, miss, _, _, drift, _ = self.run_close(80.0, frame_stops=True)
        check = attempt.close_check()
        self.assertTrue(check["closed"], check)
        self.assertEqual(check["ended"], "the base stopped turning")
        self.assertLess(abs(attempt._door_from_base(base)), 3.0)               # stopped at shut
        self.assertEqual(statuses, {"pose"})
        self.assertLess(drift, 0.02)
        self.assertEqual(attempt.commanded_door_deg(), plan.CLOSE_TO_DEG)

    def test_after_an_unlock_the_close_takes_the_shut_door_from_the_unlocks_look(self):
        """Lukas, 2026-10-25: "now lets chain it together with the door unlocking". Unlocked, the door hangs ajar on its
        spring when the pull looks, and the pull takes it as shut: the close's angle read against the pull's box is
        short by that much; against the unlock's look of the shut door (`plan.shut_reference`), it is the door's."""
        attempt, *_ = self.run_close(76.0, pulled_ajar_deg=8.0)
        self.assertAlmostEqual(attempt.close["from_deg"], 68.0, delta=1.0)
        attempt, *_ = self.run_close(76.0, pulled_ajar_deg=8.0, shut_from_unlock=True)
        self.assertAlmostEqual(attempt.close["from_deg"], 76.0, delta=1.0)
        self.assertTrue(attempt.close_check()["closed"])

    def test_the_shut_reference_turns_the_pulls_box_about_its_hinge_only_when_ajar(self):
        hinge = np.append(GEOMETRY.hinge, 1.0)
        shut = door_box(0.92, 0.0, 181.0)
        pulled = plan.pose(None, (0.10, -0.06, 0.0)) @ shut @ plan.about_hinge(9.0)   # ajar, the odometry moved on
        box, ajar, used = plan.shut_reference(pulled, shut)
        self.assertTrue(used)
        self.assertAlmostEqual(ajar, 9.0, places=6)
        np.testing.assert_allclose(box @ hinge, pulled @ hinge, atol=1e-9)        # the pull's hinge
        np.testing.assert_allclose(box[:3, :3], shut[:3, :3], atol=1e-9)          # the unlock's heading
        for off in (0.0, -3.0, 25.0):                                             # shut, closed past, too far: not used
            with self.subTest(off=off):
                box, ajar, used = plan.shut_reference(shut @ plan.about_hinge(off), shut)
                self.assertFalse(used)
                np.testing.assert_allclose(box, shut @ plan.about_hinge(off), atol=1e-9)

    def test_the_close_grips_nearer_than_from_the_pulls_stance(self):
        """Lukas, 2026-10-25: "the dog needs to walk closer to the door to avoid the arm extending too far". On the dog
        the close gripped 0.582 m ahead of the base, the pull 0.529: its walk up had stopped 4 cm short and the refit
        put the box 2 cm further. From its own stance, `CLOSE_STANCE_NEARER_M` nearer, the dry close grips that much
        nearer than from the pull's at least (0.475 against 0.558 m: from the nearer staging pose the pull's stance
        needs no walk, and the walk to the close's carries on past it)."""
        from unittest import mock

        with mock.patch.object(plan, "CLOSE_STANCE_NEARER_M", 0.0):
            *_, from_pulls = self.run_close(76.0)
        attempt, *_, grip_b = self.run_close(76.0)
        reanchor = [e for e in attempt.events if e.get("phase") == "reanchor"]
        self.assertTrue(reanchor and reanchor[0]["ok"], reanchor)
        self.assertGreater(from_pulls[0] - grip_b[0], plan.CLOSE_STANCE_NEARER_M - 0.01)

    def test_a_door_nearly_shut_is_not_pushed(self):
        attempt, why = self.run_close(10.0, expect_abort=True)
        self.assertIn("nothing to push closed", why)
        self.assertEqual(attempt.phase, "unfold")

    def test_the_box_turned_about_the_hinge_has_its_shut_door_where_the_door_is(self):
        site = plan.props.BoxSite(0.8, 4.0, 178.0)
        for door_deg in (0.0, 30.0, 76.0):
            turned = perception.site_from_pose(perception.site_pose(site) @ plan.about_hinge(door_deg))
            np.testing.assert_allclose(turned.grasp_point_m(), site.grasp_point_m(0.0, door_deg), atol=1e-9)
            np.testing.assert_allclose(turned.lever_axis_unit(0.0, 0.0), site.lever_axis_unit(0.0, door_deg), atol=1e-9)
            np.testing.assert_allclose(turned.hinge_xy_m(), site.hinge_xy_m(), atol=1e-9)

    def test_confirmations_name_real_phases(self):
        names = {p.name for stage in plan.STAGES[2:] for p in plan.attempt_phases(stage)}
        self.assertTrue(set(plan.CONFIRM_BEFORE) <= names)
        with self.assertRaises(ValueError):
            plan.attempt_phases("watch")


class ProfileTests(unittest.TestCase):
    """The arm link's command profile (`arm_link.Profile`): smooth, bounded, and never far ahead of the arm."""

    def test_the_unfold_ramps_up_cruises_and_stops_on_the_goal(self):
        from demos.combiner_door.robot.arm_link import ACCEL_DEG_S2, Profile

        start = np.asarray(d1_hardware.REST_SERVO_DEG, dtype=float)
        goal = np.array([0.6, 7.3, 13.3, 2.6, -31.4, 3.1])
        profile, q, speeds = Profile(start), start, []
        for _ in range(100):
            command = np.asarray(profile.step(goal, q, 0.1, 30.0))
            speeds.append(float(np.max(np.abs(command - q))) / 0.1)
            q = command                                       # the arm follows its command
        self.assertLess(np.max(np.abs(q - goal)), 1e-6)       # arrives, no overshoot
        self.assertLessEqual(max(speeds), 30.0 + 1e-6)
        steps = np.diff([0.0] + speeds)
        # No jumps in speed: at most one step of acceleration, and a little more on the last, part-length step.
        self.assertLessEqual(float(np.max(np.abs(steps))), 1.25 * ACCEL_DEG_S2 * 0.1)
        moving = [v for v in speeds if v > 1e-6]
        self.assertGreater(len(moving), 30)                  # ~100 deg at 30 deg/s: a few seconds, not one lurch

    def test_an_arm_held_back_holds_the_command_to_it(self):
        from demos.combiner_door.robot.arm_link import LEAD_DEG, Profile

        stuck = np.zeros(6)
        profile = Profile(stuck)
        for _ in range(50):
            command = np.asarray(profile.step(np.full(6, 80.0), stuck, 0.1, 30.0))
        self.assertLessEqual(float(np.max(np.abs(command - stuck))), LEAD_DEG + 1e-9)


class FollowAndSendTests(unittest.TestCase):
    """Every arm motion the holster draw's way (Lukas, 2026-10-24): `arm_link.Follower` for a goal that moves each
    step, and `plan.ArmSender`, which sends joint moves as paced timed paths."""

    def test_a_moving_goal_is_commanded_as_it_moves_where_the_profile_brakes_on_each(self):
        from demos.combiner_door.robot.arm_link import Follower, Profile

        goals = [np.array([3.0 * i, 0, 0, 0, 0, 0]) for i in range(1, 31)]      # 30 deg/s, a step a command
        follower, profile, q_f, q_p, f_steps, p_steps = Follower(np.zeros(6)), Profile(np.zeros(6)), \
            np.zeros(6), np.zeros(6), [], []
        for goal in goals:
            follower.goal = goal
            c = np.asarray(follower.step(q_f, 0.0, 0.1))
            f_steps.append(float(c[0] - q_f[0]))
            q_f = c
            c = np.asarray(profile.step(goal, q_p, 0.1, 45.0))
            p_steps.append(float(c[0] - q_p[0]))
            q_p = c
        self.assertTrue(np.allclose(f_steps, 3.0))                             # each command the goal itself
        self.assertGreaterEqual(sum(st < 2.5 for st in p_steps), 3)           # the profile's under the firmware's 2.5
        self.assertGreater(goals[-1][0] - q_p[0], 1.0)                        # and behind the goal at the end

    def test_a_far_goal_is_approached_at_the_link_s_top_speed_and_an_arm_held_back_makes_it_wait(self):
        from demos.combiner_door.robot.arm_link import LEAD_DEG, MAX_SPEED_DEG_S, Follower

        follower = Follower(np.zeros(6))
        follower.goal = np.array([90.0, 0, 0, 0, 0, 0])
        c = follower.step(np.zeros(6), 0.0, 0.1)
        self.assertAlmostEqual(c[0], MAX_SPEED_DEG_S * 0.1)
        for _ in range(10):
            c = follower.step(np.zeros(6), 0.0, 0.1)                         # the arm does not move
        self.assertLessEqual(c[0], LEAD_DEG + MAX_SPEED_DEG_S * 0.1 + 1e-9)
        self.assertGreater(follower.waited, 0)

    def test_the_sender_sends_a_joint_move_once_as_a_paced_path_and_follows_the_rest(self):
        sender, start = plan.ArmSender(), np.zeros(6)
        goal = np.array([60.0, -20.0, 5.0, 0.0, 30.0, 0.0])
        first = sender.message(goal, 10.0, True, start)
        self.assertEqual(first["type"], "path")
        path = np.asarray(first["q"])
        np.testing.assert_allclose(path[0], start)
        np.testing.assert_allclose(path[-1], goal)
        self.assertLessEqual(float(np.abs(np.diff(path, axis=0)).max()), plan.JOINT_SPEED_DEG_S * 0.1 + 1e-6)
        moving = np.abs(np.diff(path[:, 1]))                                   # a joint that moves 20 deg of 60
        self.assertGreaterEqual(float(np.median(moving[moving > 0.05])), 3.5)  # at the full pace, not a third of it
        again = sender.message(goal, 12.0, True, path[3])
        self.assertEqual(again, {"type": "keepalive", "gripper": 12.0})
        near = sender.message(goal + 2.0, 12.0, False, goal)
        self.assertEqual(near["type"], "follow")
        jump = sender.message(goal + 20.0, 12.0, False, goal)
        self.assertEqual(jump["type"], "path")                               # a jump is a joint move too

    def test_the_simulated_link_plays_what_the_sender_sends(self):
        from demos.combiner_door.robot.arm_link import SimulatedLink

        link, sender = SimulatedLink(np.zeros(6)), plan.ArmSender()
        goal = np.array([40.0, 0, 0, 0, 0, 0])
        for _ in range(40):
            link.handle(sender.message(goal, None, True, link.q))
        self.assertEqual(link.paths, 1)
        self.assertLess(float(np.max(np.abs(link.q - goal))), 0.5)
        for i in range(1, 11):
            link.handle(sender.message(goal + i, None, False, link.q))
        self.assertIsNotNone(link.follower)
        self.assertLess(float(np.max(np.abs(link.q - (goal + 10)))), 1.5)


class PathPlayerTests(unittest.TestCase):
    """The arm link's timed paths (`arm_link.PathPlayer`, `check_path`): the path's own points by the clock, waiting for
    an arm held back, the lead allowing for the report's age."""

    def _path(self, n=31, step=3.4):
        return np.array([[i * step, 0, 0, 0, 0, 0] for i in range(n)], dtype=float)

    def test_it_plays_each_point_at_its_time_without_slowing_between_them(self):
        from demos.combiner_door.robot.arm_link import PathPlayer

        path = self._path()
        player, commands = PathPlayer(path, 0.1), []
        q = path[0]
        while not player.done:
            commands.append(player.step(q, 0.0, 0.1))
            q = np.asarray(commands[-1])                       # the arm follows its command
        steps = np.abs(np.diff([path[0][0]] + [c[0] for c in commands]))
        self.assertTrue(np.allclose(steps, 3.4))              # every command a full step: no braking between points
        self.assertTrue(np.allclose(commands[-1], path[-1]))

    def test_an_arm_held_back_makes_the_path_wait(self):
        from demos.combiner_door.robot.arm_link import LEAD_DEG, PathPlayer

        player = PathPlayer(self._path(), 0.1)
        stuck = np.zeros(6)
        for _ in range(40):
            command = np.asarray(player.step(stuck, 0.0, 0.1))
        self.assertLessEqual(float(np.max(np.abs(command - stuck))), LEAD_DEG + 3.4 + 1e-9)
        self.assertFalse(player.done)
        self.assertGreater(player.waited, 30)

    def test_a_stale_report_allows_more_lead_up_to_its_cap(self):
        from demos.combiner_door.robot.arm_link import LEAD_DEG, PATH_LEAD_MAX_DEG, PathPlayer

        # One step (3.4 deg at 0.1 s) is allowed on top of the lead, and the path's speed times the report's age.
        for age, cap in ((0.0, LEAD_DEG + 3.4), (0.05, LEAD_DEG + 5.1), (0.5, PATH_LEAD_MAX_DEG)):
            player = PathPlayer(self._path(), 0.1)
            stuck = np.zeros(6)
            for _ in range(20):
                command = np.asarray(player.step(stuck, age, 0.1))
            lead = float(np.max(np.abs(command - stuck)))
            self.assertLessEqual(lead, cap + 1e-9)
            self.assertGreater(lead, cap - 3.4 - 1e-9)

    def test_at_5_hz_a_lagging_arm_does_not_make_it_wait_every_other_step(self):
        from demos.combiner_door.robot.arm_link import PathPlayer, SimulatedArm

        path = self._path(n=16, step=8.6)
        player, arm = PathPlayer(path, 0.2), SimulatedArm(path[0])
        while not player.done:
            arm.step_command(player.step(arm.q, 0.0, 0.2), None, 0.2)
        self.assertEqual(player.waited, 0)

    def test_a_path_is_refused_away_from_the_arm_or_too_fast(self):
        from demos.combiner_door.robot.arm_link import check_path

        path = self._path()
        self.assertIsNone(check_path(path, 0.1, path[0]))
        self.assertIn("start", check_path(path, 0.1, path[0] + 15.0))
        self.assertIn("faster", check_path(self._path(step=5.0), 0.1, np.zeros(6)))
        self.assertIsNotNone(check_path(path[:1], 0.1, path[0]))


class ShapingTests(unittest.TestCase):
    def test_sport_command_stands_lifts_and_caps(self):
        self.assertEqual(plan.sport_command((0.05, 0.0, 0.0), 0.01, 0.0), (0.0, 0.0, 0.0))
        lifted = plan.sport_command((0.05, 0.0, 0.02), 0.10, 0.0)
        self.assertAlmostEqual(lifted[0], plan.WALK_FLOOR[0])
        self.assertAlmostEqual(lifted[2], 0.02 * plan.WALK_FLOOR[0] / 0.05)
        capped = plan.sport_command((0.9, -0.9, 2.0), 1.0, 1.0)
        self.assertEqual(capped, (plan.WALK_CAP[0], -plan.WALK_CAP[1], plan.WALK_CAP[2]))

    def test_step_towards_caps_the_longest_joint(self):
        step = plan.step_towards([0, 0, 0, 0, 0, 0], [40, -20, 0, 0, 0, 10])
        self.assertAlmostEqual(max(abs(v) for v in step), plan.ARM_STEP_DEG)
        self.assertAlmostEqual(step[1] / step[0], -0.5)

    def test_the_fold_is_the_rest_pose_inside_the_commandable_limits(self):
        self.assertEqual(plan.FOLD_SERVO_DEG[1], -90.0)
        self.assertEqual(plan.FOLD_SERVO_DEG[2], 90.0)
        self.assertGreater(plan.GRIPPER_OPEN_UNITS, d1_hardware.finger_travel_to_gripper_units((0.040 - 0.0172) / 2))


class VizTests(unittest.TestCase):
    def test_quaternions_round_trip(self):
        rng = np.random.default_rng(1)
        for _ in range(50):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            rot = viz.quat_to_matrix(q)
            back = np.asarray(viz.matrix_to_quat(rot))
            self.assertLess(min(np.linalg.norm(back - q), np.linalg.norm(back + q)), 1e-9)

    def test_the_urdf_chain_reaches_the_jaw_as_the_ik_does(self):
        from pathlib import Path

        root = Path(__file__).resolve().parents[3]
        model = viz.UrdfModel(root / "description/go2_d1.urdf")
        servo = [10.0, -30.0, 40.0, 5.0, -20.0, 15.0]
        values = viz.joint_values(["FR_hip_joint"], [0.0], servo, 20.0)
        frames = {"base_link": np.eye(4)}
        pending = model.transforms(values)
        while pending:
            rest = []
            for parent, child, p in pending:
                if parent in frames:
                    frames[child] = frames[parent] @ p
                else:
                    rest.append((parent, child, p))
            if len(rest) == len(pending):
                break
            pending = rest
        joints, _ = d1_ik.load_urdf()
        pos, _ = d1_ik.tool_pose(joints, d1_ik.from_servo_deg(servo)[0], body="Link6", offset=JAW_CENTRE_LINK6)
        drawn = plan.transform(frames["Link6"], [JAW_CENTRE_LINK6])[0]
        self.assertLess(np.linalg.norm(drawn - pos), 0.002)

    def test_the_description_points_at_the_pc_mount(self):
        from pathlib import Path

        text = viz.description_for_pc(Path(__file__).resolve().parents[3] / "description/go2_d1.urdf",
                                      "file:///root/d1training/")
        self.assertNotIn("package://d1_training/", text)
        self.assertIn("file:///root/d1training/description/meshes/", text)

    def test_cloud_xyz_reads_the_livox_layout_and_drops_no_returns(self):
        # x y z intensity float32, tag line uint8, timestamp float64: 26 bytes, as /livox/lidar on the dog.
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("tag", "u1"), ("line", "u1"),
                          ("t", "<f8")])
        data = np.zeros(3, dtype=dtype)
        data["x"], data["y"], data["z"] = [1.0, 0.0, 2.0], [0.5, 0.0, -1.0], [0.1, 0.0, 0.3]
        fields = [SimpleNamespace(name=n, offset=o) for n, o in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))]
        msg = SimpleNamespace(fields=fields, point_step=26, width=3, height=1, data=data.tobytes())
        points = viz.cloud_xyz(msg)
        np.testing.assert_allclose(points, [[1.0, 0.5, 0.1], [2.0, -1.0, 0.3]], atol=1e-6)


if __name__ == "__main__":
    unittest.main()

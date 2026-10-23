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
from demos.combiner_door import perception
from demos.combiner_door.robot import plan, viz
from demos.combiner_door.robot.arm_link import SimulatedArm
from demos.cup.pick_demo import grasp
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6
from demos.unifp import script as demo_script

#: Servo 6 with the 40 mm lever between the pads: where the jaws stall closing on it (`plan.GRIPPER_SHUT_UNITS`' note).
LEVER_STALL_UNITS = 6.8


def box_odom(x, y, yaw_deg):
    """The enclosure on the floor at (x, y), its door facing `yaw_deg` (180: facing a robot looking along +x)."""
    return plan.pose(plan.rot_z(math.radians(yaw_deg)), (x, y, 0.0))


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


def walk(base, command, moving, dt=0.1):
    """A dry run's base: walks the command; stopped, it carries on as the real gait does (`plan.CARRY_S` of the speed it
    had, over that long). Returns the base and the speed it is going at."""
    if command is None:
        decay = max(0.0, 1.0 - dt / plan.CARRY_S)
        command = moving = tuple(v * decay for v in moving)
    else:
        moving = tuple(command)
    x, y, yaw = plan.planar(base)
    vx, vy, wz = command
    return plan.base_pose(x + dt * (math.cos(yaw) * vx - math.sin(yaw) * vy),
                          y + dt * (math.sin(yaw) * vx + math.cos(yaw) * vy), yaw + dt * wz), moving


def look(base, box, frames=20, scans=10, seed=0):
    rng = np.random.default_rng(seed)
    return ([(base, [detection(base, box)])] * frames, [(base, face_scan(base, box, rng)) for _ in range(scans)])


def finger_points(q_servo, base_s, travel=demo_script.JAW_LEVER_OPEN_M):
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
        return self.base @ plan.pose(None, (0, 0, -plan.BASE_HEIGHT_M)) @ box_odom(forward, left, 180.0 + yaw_off_deg)

    def test_a_look_puts_the_stance_at_the_nominal_distance(self):
        box = self.box_ahead(0.95, 0.03, 3.0)
        frames, scans = look(self.base, box)
        fix = plan.make_fix(self.base, frames, scans)
        walk = 0.95 - math.sqrt(plan.NOMINAL_DISTANCE_M ** 2 - 0.03 ** 2)
        # Sent short by the odometry's under-count (`plan.ODOM_WALK_SCALE`): the dog walks the whole of it.
        self.assertAlmostEqual(fix.walk_in_m, plan.ODOM_WALK_SCALE * walk, places=3)
        self.assertAlmostEqual(fix.site.root_m[0], plan.NOMINAL_DISTANCE_M * math.cos(math.atan2(0.03, plan.NOMINAL_DISTANCE_M))
                               + (1 - plan.ODOM_WALK_SCALE) * walk, places=3)
        self.assertTrue(fix.fit.ok, fix.fit.why)
        self.assertAlmostEqual(fix.site.yaw_deg % 360.0, 183.0, delta=0.5)
        ok, text = fix.in_range()
        self.assertTrue(ok, text)
        # The box's root is where it was put, in odom, and the script frame's z puts it at the plinth's height.
        self.assertLess(np.linalg.norm(fix.box_odom[:2, 3] - box[:2, 3]), 0.005)

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
        box = self.box_ahead(1.00, 0.10, 0.0)
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
    """The Go2's front camera as the look's camera (2026-10-22): fixed in the trunk, a NOMINAL fisheye lens."""

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


class AttemptTests(unittest.TestCase):
    def run_attempt(self, stage, timing=None, max_steps=1200, arm=None, watch=None, expect_abort=False):
        """A dry attempt with the arm link's profile driving a model servo (`SimulatedArm`), the jaws stalling on the
        lever while the hand holds it. `watch(attempt, command, arm, base)` is called each step. Returns the attempt,
        the phases in order, the arm's last joints and (base, jaw centre, grasp point) at the arrival; with
        `expect_abort`, the attempt and the abort's reason."""
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = box_odom(0.92, 0.0, 181.0)
        frames, scans = look(base, box)
        fix = plan.make_fix(base, frames, scans)
        attempt = plan.Attempt(fix, stage, timing or plan.scaled_timing(1.0, 30.0))
        arm = arm or SimulatedArm(d1_hardware.REST_SERVO_DEG)
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
                arm.step(command.arm_servo_deg, command.gripper_units, command.speed_deg_s)
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
        fix = plan.make_fix(base, *look(base, box_odom(0.92, 0.0, 180.0)))
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
        box = box_odom(0.95, 0.0, 180.0)
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
        box = base @ plan.pose(None, (0, 0, -plan.BASE_HEIGHT_M)) @ box_odom(0.92, 0.0, 182.0)
        fix = plan.make_fix(base, *look(base, box))
        stopped = base @ plan.pose(plan.rot_z(math.radians(3.0)), (0.15, 0.01, 0.0))
        moved = plan.reanchor(fix, stopped)
        self.assertLess(np.linalg.norm(moved.box_odom[:2, 3] - fix.box_odom[:2, 3]), 1e-6)
        self.assertAlmostEqual(moved.site.yaw_deg % 360, (fix.site.yaw_deg - 3.0) % 360, places=4)
        self.assertAlmostEqual(moved.site.distance_m, math.hypot(0.92 - 0.15 * math.cos(0) - 0.0, 0.01) , delta=0.01)

    def test_the_walk_in_ends_short_by_the_carry(self):
        attempt, _, _, _ = self.run_attempt("walk")
        walked = [e for e in attempt.events if e.get("phase") == "walk_settle"]
        self.assertTrue(walked)

    def test_a_loaded_arm_aborts_the_pull(self):
        base = plan.base_pose(0.0, 0.0, 0.0)
        box = box_odom(0.92, 0.0, 180.0)
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
        box = box_odom(0.92, 0.0, 180.0)
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
        self.assertEqual([p.name for p in attempt.script.phases], ["release", "ungrasp", "home", "fold"])

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
        self.assertLess(min(old["back"]), 0.0)          # into the lever or the hub on the way home

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
        stuck, phase, steps = SimulatedArm(d1_hardware.REST_SERVO_DEG), [""], {}
        real_step = stuck.step

        def step(target, gripper=None, speed=30.0, dt=0.1):
            off = np.array([0.0, 0.0, 0.0, 0.0, 3.0, 0.0]) if phase[0] == "settle" else 0.0
            real_step(np.asarray(target) + off, gripper, speed, dt)

        def watch(attempt, command, arm, base):
            phase[0] = attempt.phase
            steps[command.phase] = steps.get(command.phase, 0) + 1

        stuck.step = step
        self.run_attempt("reach", arm=stuck, watch=watch)
        self.assertAlmostEqual(0.1 * steps["settle"], plan.scaled_timing(1.0, 30.0).settle_s, delta=0.15)

    def test_the_hand_leaves_the_lever_only_once_the_jaws_are_open(self):
        slow = SimulatedArm(d1_hardware.REST_SERVO_DEG, gripper_rate=12.0)
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

    def test_the_release_lets_go_from_where_the_arm_is(self):
        # `--release` from the pose the arm was left holding the lever in (2026-10-20).
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.make_fix(base, *look(base, box_odom(0.92, 0.0, 180.0)))
        held = [29.8, 27.9, 12.3, 38.2, -50.7, 67.3]
        attempt = plan.release(fix, base, held)
        self.assertEqual([p.name for p in attempt.script.phases], ["release", "ungrasp", "home", "fold"])
        arm = SimulatedArm(held, gripper=LEVER_STALL_UNITS)
        joints, _ = d1_ik.load_urdf()
        start = np.asarray(attempt._held["hand"])
        approach = np.asarray(attempt._held["approach"])
        for step in range(600):
            command = attempt.step(plan.Observation(0.1 * step, base, 0.0, arm.q.copy(), 0.0,
                                                    gripper_units=arm.gripper))
            self.assertEqual(command.abort, "", command.phase)
            if command.finished:
                break
            arm.step(command.arm_servo_deg, command.gripper_units, command.speed_deg_s)
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
        fix = plan.make_fix(base, *look(base, box_odom(0.92, 0.0, 180.0)))
        attempt = plan.Attempt(fix, "grip", plan.scaled_timing(1.5, 30.0))
        self.assertEqual(attempt.arm.grasp_note, "pitch 0 deg, roll +1")       # planned before anything moves
        q = np.asarray(d1_hardware.REST_SERVO_DEG, dtype=float)
        for step in range(5):
            command = attempt.step(plan.Observation(0.1 * step, base, 0.0, q, 0.0))
            q = np.asarray(plan.step_towards(q, command.arm_servo_deg))
        self.assertEqual(command.phase, "unfold")
        attempt.recover()
        self.assertEqual([p.name for p in attempt.script.phases], ["fold"])

    def test_the_fold_waits_for_the_arm(self):
        # An arm slower than the clock: the fold phase runs on until it is folded (2026-10-20: on the clock alone it
        # ended 23 deg short), and gives up after JOINT_TIMEOUT_S.
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.make_fix(base, *look(base, box_odom(0.92, 0.0, 180.0)))
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
        # 2026-10-20: 0.73 m off, a walk of 4 cm (under what sport mode can do) was refused with "back 0.16 m".
        base = plan.base_pose(0.0, 0.0, 0.0)
        fix = plan.make_fix(base, *look(base, box_odom(0.73, 0.0, 180.0)))
        ok, text = fix.in_range()
        self.assertEqual(fix.walk_in_m, 0.0)
        self.assertTrue(ok, text)
        self.assertIn("outside the simulated", text)

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

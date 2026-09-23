"""The door's AprilTag, the lever press plan and the turn sequence, on the CPU (numpy + OpenCV, no Isaac)."""
import math
import random
import unittest

import numpy as np

from demos.combiner.apriltag import (TAG_BITS, TagDetector, black_cells, box_pose, corner_points, invert,
                                     mean_pose, pose_error, tag_image, tag_pose_box)
from demos.combiner.geometry import DEFAULT_HANDLE_TORQUE_NM, GEOMETRY, Placement, sample_placement
from demos.combiner.press import (Lever, PressParams, SearchParams, plan_close_look, plan_press, plan_search,
                                  press_capacity)
from demos.combiner.pull import Handle, PullParams, arc_times, is_proven, plan_pull
from demos.cup.pick_demo.camera import CAMERAS, camera_pose, link6_pose, project, resolve_mount, transform
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6, optical_rotation
from position_only.workspace import load_urdf

try:
    import cv2
except ImportError:
    cv2 = None

JOINTS, LINKS = load_urdf()
MODEL = CAMERAS["d435"]
MOUNT = resolve_mount(None)[0]   # the demo's default: the pick's saved wrist mount, else the assumed one
# The lying Go2 as the pick measured it: base 0.0851 m up, 7.3 deg nose-up (demos/cup/pick_demo/grasp.py).
_PITCH = math.radians(-7.3)
BASE_W = np.eye(4)
BASE_W[:3, :3] = [[math.cos(_PITCH), 0, math.sin(_PITCH)], [0, 1, 0], [-math.sin(_PITCH), 0, math.cos(_PITCH)]]
BASE_W[:3, 3] = (0.0, 0.0, 0.0851)
UP_B = BASE_W[:3, :3].T @ np.array([0.0, 0.0, 1.0])


def box_in_base(placement) -> np.ndarray:
    pose = np.eye(4)
    yaw = math.radians(placement.yaw_deg)
    pose[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    pose[:3, 3] = placement.position
    return invert(BASE_W) @ pose


def render(camera_b, box_b, pixels_per_cell=24):
    """The wrist camera's view of the door's tag alone, on grey: a homography of the printed tag."""
    tag_cam = invert(camera_b) @ box_b @ tag_pose_box()
    half = GEOMETRY.tag_outer_size / 2
    corners = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
    points = (tag_cam[:3, :3] @ corners.T).T + tag_cam[:3, 3]
    if (points[:, 2] < 0.05).any():
        return np.full((MODEL.height, MODEL.width, 3), 128, np.uint8)
    side = 10 * pixels_per_cell
    src = np.float32([[0, 0], [side, 0], [side, side], [0, side]])
    homography = cv2.getPerspectiveTransform(src, np.float32(project(MODEL, points)))
    grey = cv2.warpPerspective(tag_image(GEOMETRY.tag_id, pixels_per_cell), homography,
                               (MODEL.width, MODEL.height), flags=cv2.INTER_AREA, borderValue=128)
    return np.dstack([grey] * 3)


class TagTests(unittest.TestCase):
    @unittest.skipUnless(cv2 is not None, "needs OpenCV")
    def test_pattern_is_opencvs_tag36h11_id0(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, GEOMETRY.tag_id, 8)
        self.assertTrue((np.array(TAG_BITS[GEOMETRY.tag_id]) == (marker[1:7, 1:7] > 0)).all())
        # The border ring is black all round.
        self.assertEqual(sum(1 for r, c in black_cells() if r in (0, 7) or c in (0, 7)), 28)

    @unittest.skipUnless(cv2 is not None, "needs OpenCV")
    def test_oblique_view_recovers_the_tag_pose(self):
        detector = TagDetector(MODEL.intrinsic_matrix)
        box = box_in_base(Placement((0.66, 0.1, 0.0), 195.0))
        tag_b = box @ tag_pose_box()
        for yaw_deg, pitch_deg, range_m in ((0, 10, 0.25), (35, 25, 0.4), (-40, 5, 0.5)):
            yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
            out = tag_b[:3, 2] * math.cos(yaw) + tag_b[:3, 0] * math.sin(yaw)
            camera_pos = tag_b[:3, 3] + range_m * (math.cos(pitch) * out + math.sin(pitch) * UP_B)
            camera = transform(optical_rotation(tag_b[:3, 3] - camera_pos, UP_B), camera_pos)
            found = detector.detect(render(camera, box))
            self.assertEqual(len(found), 1, (yaw_deg, pitch_deg, range_m))
            error = pose_error(box_pose(camera, found[0]), box)
            self.assertLess(error["position_mm"], 3.0 * range_m / 0.25, (yaw_deg, error))
            self.assertLess(error["rotation_deg"], 2.0, (yaw_deg, error))

    def test_tag_frame_is_the_door_face_and_inverse_is_exact(self):
        pose = tag_pose_box()
        self.assertTrue(np.allclose(pose[:3, 2], [1, 0, 0]))   # out of the door
        self.assertTrue(np.allclose(pose[:3, 1], [0, 0, 1]))   # printed upright
        self.assertAlmostEqual(np.linalg.det(pose[:3, :3]), 1.0)
        self.assertTrue(np.allclose(invert(pose) @ pose, np.eye(4)))
        self.assertTrue(np.allclose(mean_pose([pose, pose]), pose))
        self.assertTrue(np.allclose(corner_points(0.06)[0], [-0.03, 0.03, 0]))

    def test_tag_is_on_the_door_and_clear_of_the_decals_and_the_lever(self):
        g = GEOMETRY
        half = g.tag_outer_size / 2
        y0, y1, z0, z1 = g.tag_centre_y - half, g.tag_centre_y + half, g.tag_centre_z - half, g.tag_centre_z + half
        self.assertGreater(y0, -g.width / 2 + 0.01)
        self.assertLess(y1, g.width / 2 - 0.01)
        self.assertLess(z1, g.bottom + g.height - 0.01)
        # The warning triangle spans door-local y 0.074-0.186 (enclosure -0.106 to 0.006).
        self.assertGreater(y0, 0.006 + 0.01)
        # The rosette and the lever are at the spindle's height and below it, at any angle.
        self.assertGreater(z0, g.spindle[2] + 0.024 + 0.02)


class SpringTests(unittest.TestCase):
    def test_torque_is_specified_at_45_degrees_with_a_preload_at_the_stop(self):
        g = GEOMETRY
        self.assertEqual(g.handle_release_deg, 45.0)
        for torque in (0.1, DEFAULT_HANDLE_TORQUE_NM, 0.94):
            self.assertAlmostEqual(g.spring_torque(torque, 45.0), torque)
            self.assertAlmostEqual(g.spring_torque(torque, 0.0), torque / 4)
            # The drive limit never clips the spring anywhere in the lever's travel or while H holds it.
            self.assertGreater(g.spring_effort_limit(torque), g.spring_torque(torque, g.handle_limit_deg))
            self.assertGreater(g.spring_effort_limit(torque),
                               g.spring_stiffness(torque) * math.radians(g.handle_hold_deg))
        self.assertLess(DEFAULT_HANDLE_TORQUE_NM, 0.94)
        for bad in (0.0, -1.0, math.nan):
            with self.assertRaises(ValueError):
                g.spring_stiffness(bad)


class PressPlanTests(unittest.TestCase):
    def test_arc_carries_the_fingers_along_the_turning_lever(self):
        box = box_in_base(Placement((0.64, 0.0, 0.0), 180.0))
        params = PressParams()
        plan = plan_press(JOINTS, LINKS, box, np.zeros(6), params)
        lever = Lever(box)
        offset = params.contact_link6()
        self.assertAlmostEqual(plan.arc[-1][0], params.final_deg)
        for deg, q in plan.arc:
            pose = link6_pose(JOINTS, q)
            contact = pose[:3, :3] @ offset + pose[:3, 3]
            target = lever.top(deg, params.radius_m) - params.press_depth_m * lever.normal(deg)
            self.assertLess(np.linalg.norm(contact - target), 0.002, deg)
            cos = (np.trace(pose[:3, :3].T @ lever.tool_rotation(deg)) - 1) / 2
            self.assertLess(math.degrees(math.acos(min(1.0, cos))), 2.0, deg)
        # The fingers stay on the lever: both finger centres are inside its length.
        self.assertLess(params.radius_m + 0.0141 + 0.005, GEOMETRY.handle_length)
        # The fingertips stop short of the door.
        self.assertLess(params.reach_past_m, GEOMETRY.handle_projection - 0.02)

    def test_tool_frame_is_right_handed_and_level_at_rest(self):
        lever = Lever(box_in_base(Placement((0.64, 0.0, 0.0), 180.0)))
        rot = lever.tool_rotation(0.0)
        self.assertAlmostEqual(np.linalg.det(rot), 1.0, places=6)
        self.assertGreater(-rot[:, 0] @ UP_B, 0.95)      # +x (the finger undersides) points down
        self.assertLess(abs(rot[:, 2] @ UP_B), 0.15)     # approach level

    def test_capacity_counts_the_arms_weight_and_names_the_joint(self):
        box = box_in_base(Placement((0.64, 0.0, 0.0), 180.0))
        plan = plan_press(JOINTS, LINKS, box, np.zeros(6))
        lever = Lever(box)
        q = plan.arc[0][1]
        down = press_capacity(JOINTS, LINKS, q, lever.top(0, 0.08), -lever.normal(0))
        up = press_capacity(JOINTS, LINKS, q, lever.top(0, 0.08), lever.normal(0))
        self.assertGreater(down["force_n"], up["force_n"])   # its weight helps a push down, hinders a lift
        self.assertIn(down["limiting_joint"], [f"Joint{i}" for i in range(1, 7)])
        self.assertTrue(0.4 < plan.predicted_torque_nm() < 1.2)

    def test_default_placements_are_found_looked_at_and_pressed(self):
        rng = random.Random(3)
        stops = plan_search(JOINTS, LINKS, MOUNT, UP_B, np.zeros(6))
        self.assertEqual([h for h, _ in stops], list(SearchParams().headings_deg))
        for _ in range(12):
            box = box_in_base(sample_placement(rng))
            seen = [q for _, q in stops if self._tag_in_view(q, box)]
            self.assertTrue(seen)
            q_close = plan_close_look(JOINTS, LINKS, MOUNT, box, UP_B, seen[0])
            self.assertIsNotNone(q_close)
            self.assertTrue(self._tag_in_view(q_close, box))
            plan_press(JOINTS, LINKS, box, q_close)

    def test_a_close_look_before_a_grip_turns_the_camera_to_spare_the_wrist(self):
        from demos.combiner.pull import standoff_guess

        rng = random.Random(5)
        stops = plan_search(JOINTS, LINKS, MOUNT, UP_B, np.zeros(6))
        saved = []
        for _ in range(8):
            box = box_in_base(sample_placement(rng))
            q_look = next(q for _, q in stops if self._tag_in_view(q, box))
            toward = standoff_guess(JOINTS, LINKS, box, q_look)
            if toward is None:          # the grasp that has held does not reach here; the look stays upright
                continue
            upright = plan_close_look(JOINTS, LINKS, MOUNT, box, UP_B, q_look)
            turned = plan_close_look(JOINTS, LINKS, MOUNT, box, UP_B, q_look, toward=toward)
            self.assertTrue(self._tag_in_view(turned, box))
            travel = lambda q: np.max(np.abs(q - q_look)) + np.max(np.abs(toward - q))
            self.assertLessEqual(travel(turned), travel(upright) + 1e-9)
            saved.append(travel(upright) - travel(turned))
        self.assertGreaterEqual(len(saved), 4)
        self.assertGreater(max(saved), 0.5, saved)

    @staticmethod
    def _tag_in_view(q, box, margin=20):
        tag = invert(camera_pose(JOINTS, q, MOUNT)) @ box @ tag_pose_box()
        points = (tag[:3, :3] @ corner_points(GEOMETRY.tag_outer_size).T).T + tag[:3, 3]
        if (points[:, 2] < 0.05).any():
            return False
        uv = project(MODEL, points)
        return bool((uv[:, 0] > margin).all() and (uv[:, 0] < MODEL.width - margin).all()
                    and (uv[:, 1] > margin).all() and (uv[:, 1] < MODEL.height - margin).all())


class PullPlanTests(unittest.TestCase):
    BOX = box_in_base(Placement((0.66, 0.0, 0.0), 180.0))

    @classmethod
    def setUpClass(cls):
        cls.plan = plan_pull(JOINTS, LINKS, cls.BOX, np.zeros(6))

    def jaw(self, q):
        pose = link6_pose(JOINTS, q)
        return pose[:3, :3] @ np.asarray(JAW_CENTRE_LINK6) + pose[:3, 3], pose[:3, :3]

    def test_the_jaws_follow_the_handle_through_turn_crack_unturn_and_pull(self):
        plan, handle = self.plan, Handle(self.BOX, self.plan.params)
        p = plan.params
        paths = ([(d, 0.0, q) for d, q in plan.turn] + [(p.turn_deg, d, q) for d, q in plan.crack]
                 + [(d, p.crack_deg, q) for d, q in plan.unturn] + [(0.0, d, q) for d, q in plan.pull])
        for lever, door, q in paths:
            pos, rot = handle.grasp(lever, door)
            jaw, link6 = self.jaw(q)
            self.assertLess(np.linalg.norm(jaw - pos), 0.002, (lever, door))
            cos = (np.trace(link6.T @ rot) - 1) / 2
            self.assertLess(math.degrees(math.acos(min(1.0, cos))), 2.0, (lever, door))
        self.assertGreaterEqual(plan.door_final_deg, p.door_min_deg)
        self.assertEqual([d for d, _ in plan.turn][-1], p.turn_deg)
        self.assertEqual([d for d, _ in plan.unturn][-1], 0.0)

    def test_the_jaws_close_across_the_bar_and_meet_it_square(self):
        handle = Handle(self.BOX, self.plan.params)
        _, rot = handle.grasp(0.0, 0.0)
        lever = self.BOX[:3, :3] @ np.array([0.0, -1.0, 0.0])
        self.assertLess(abs(rot[:, 1] @ lever), 1e-9)          # jaw axis across the lever
        self.assertLess(abs(rot[:, 2] @ lever), 1e-9)          # approach square to it
        self.assertAlmostEqual(np.linalg.det(rot), 1.0)
        # The grasp point is on the lever's centreline, `radius_m` out from the spindle axis.
        pos, _ = handle.grasp(0.0, 0.0)
        axis_point = self.BOX[:3, :3] @ np.array(GEOMETRY.lever_axis_point) + self.BOX[:3, 3]
        self.assertAlmostEqual(float(np.linalg.norm(pos - axis_point)), self.plan.params.radius_m)

    def test_the_door_swings_the_handle_outward_about_the_hinge(self):
        handle = Handle(self.BOX)
        hinge = self.BOX[:3, :3] @ np.array(GEOMETRY.hinge) + self.BOX[:3, 3]
        out = self.BOX[:3, :3] @ np.array([1.0, 0.0, 0.0])
        closed, _ = handle.grasp(0.0, 0.0)
        opened, _ = handle.grasp(0.0, 30.0)
        # Same distance from the hinge line, measured in the enclosure's own horizontal plane (the base is pitched).
        in_box = lambda v: self.BOX[:3, :3].T @ v
        self.assertAlmostEqual(np.linalg.norm(in_box(closed - hinge)[:2]), np.linalg.norm(in_box(opened - hinge)[:2]),
                               places=6)
        self.assertGreater((opened - closed) @ out, 0.05)       # towards the robot, out of the box
        # The same swing as the asset's own geometry (`BoxGeometry.handle_at_angle`) at the lever's centreline.
        g = GEOMETRY
        grasp = (g.spindle[0] + g.handle_projection, g.spindle[1] - handle.p.radius_m, g.spindle[2])
        angle = math.radians(-30.0)
        x, y = grasp[0] - g.hinge[0], grasp[1] - g.hinge[1]
        expected = (g.hinge[0] + math.cos(angle) * x - math.sin(angle) * y,
                    g.hinge[1] + math.sin(angle) * x + math.cos(angle) * y, grasp[2])
        self.assertTrue(np.allclose(opened, self.BOX[:3, :3] @ np.array(expected) + self.BOX[:3, 3]))

    def test_a_grasp_that_has_held_goes_first_then_the_furthest_door(self):
        from demos.combiner.pull import plan_pull_candidates

        self.assertIn("choice 1 of", self.plan.notes[-1])
        ranked = plan_pull_candidates(JOINTS, LINKS, self.BOX, np.zeros(6))
        self.assertGreaterEqual(len(ranked), 2)
        from demos.combiner.pull import PROVEN_GRASPS, proven_rank

        proven = [proven_rank(plan.params) for plan in ranked]
        self.assertEqual(proven[0], len(PROVEN_GRASPS), [(plan.params.pitch_deg, plan.params.roll) for plan in ranked])
        self.assertEqual(proven, sorted(proven, reverse=True))
        rest = [plan.door_final_deg for plan in ranked if not is_proven(plan.params)]
        self.assertEqual(rest, sorted(rest, reverse=True))
        self.assertTrue(0.5 < self.plan.predicted_torque_nm() < 1.3)

    def test_arcs_are_paced_by_the_joints_up_to_the_angle_ceiling(self):
        p = self.plan.params
        for path, ceiling in ((self.plan.turn, p.turn_speed_deg_s), (self.plan.pull, p.door_speed_deg_s)):
            times = arc_times(path, ceiling, p.joint_speed_rad_s)
            for (a0, q0), (a1, q1), t0, t1 in zip(path, path[1:], times, times[1:]):
                self.assertLessEqual(float(np.max(np.abs(q1 - q0))) / (t1 - t0), p.joint_speed_rad_s + 1e-9)
                self.assertLessEqual(abs(a1 - a0) / (t1 - t0), ceiling + 1e-9)
            # Some step is joint-bound, or the whole arc runs at the ceiling.
            fastest = abs(path[-1][0] - path[0][0]) / ceiling
            self.assertGreaterEqual(times[-1], fastest - 1e-9)
        self.assertEqual(set(self.plan.arc_seconds()), {"turn", "crack", "unturn", "pull"})
        # A step that barely moves the arm goes at the ceiling; one that swings a joint waits for it.
        path = [(0.0, np.zeros(6)), (2.0, np.zeros(6)), (4.0, np.array([0, 0, 0, 0.3, 0, 0.0]))]
        self.assertTrue(np.allclose(arc_times(path, 40.0, 0.6), [0.0, 0.05, 0.05 + 0.3 / 0.6]))

    def test_a_known_handle_torque_picks_a_grasp_strong_enough_for_it(self):
        from dataclasses import replace

        weak = plan_pull(JOINTS, LINKS, self.BOX, np.zeros(6), replace(PullParams(), handle_torque_nm=0.3))
        strong = plan_pull(JOINTS, LINKS, self.BOX, np.zeros(6), replace(PullParams(), handle_torque_nm=0.7))
        self.assertGreaterEqual(strong.predicted_torque_nm(), 1.1 * 0.7)
        self.assertGreaterEqual(weak.door_final_deg, strong.door_final_deg)
        impossible = plan_pull(JOINTS, LINKS, self.BOX, np.zeros(6), replace(PullParams(), handle_torque_nm=5.0))
        self.assertIn("took the strongest", " ".join(impossible.notes))

    def test_every_waypoint_leaves_torque_spare_for_the_handle(self):
        from demos.combiner.pull import _gravity_fraction

        for _, q in self.plan.turn + self.plan.crack + self.plan.unturn + self.plan.pull:
            self.assertLessEqual(_gravity_fraction(JOINTS, LINKS, q), self.plan.params.hold_fraction + 1e-9)


@unittest.skipUnless(cv2 is not None, "needs OpenCV")
class SequenceTests(unittest.TestCase):
    """The whole sequence against rendered tags, with feedback that follows each command after a lag."""

    def run_sequence(self, box, visible=True, max_t=90.0, method="press", blocked=lambda sequence: False):
        from demos.combiner.sequence import TurnSequence
        from demos.cup.pick_demo.perception import Frame
        from demos.cup.pick_demo.sequence import Timing

        sequence = TurnSequence(JOINTS, LINKS, MOUNT, TagDetector(MODEL.intrinsic_matrix), method=method,
                                timing=Timing(settle_s=0.5))
        q, t, dt, states = np.zeros(6), 0.0, 0.02, []
        self.grippers = {}

        def frame_source():
            rgb = render(camera_pose(JOINTS, q, MOUNT), box) if visible else np.full((480, 640, 3), 128, np.uint8)
            return Frame(rgb, None, q.copy(), UP_B.copy(), t)

        while not sequence.done and t < max_t:
            command = sequence.update(t, q, UP_B, frame_source)
            if command.q is not None and not blocked(sequence):
                q = q + np.clip(command.q - q, -1.2 * dt, 1.2 * dt)   # the arm's speed, no dynamics
            if not states or states[-1] != sequence.state:
                states.append(sequence.state)
            self.grippers.setdefault(sequence.state, command.gripper_m)
            t += dt
        return sequence, states

    def test_finds_the_tag_plans_on_the_close_look_and_pushes_through_the_arc(self):
        box = box_in_base(Placement((0.66, -0.2, 0.0), 163.0))
        sequence, states = self.run_sequence(box)
        self.assertEqual(sequence.state, "done", sequence.failure)
        for state in ("detect", "refine", "press", "hold", "release"):
            self.assertIn(state, states)
        self.assertLess(states.index("refine"), states.index("press"))
        self.assertLess(pose_error(sequence.box, box)["position_mm"], 5.0)
        self.assertLess(pose_error(sequence.box, box)["rotation_deg"], 2.0)
        events = [e["event"] for e in sequence.events]
        self.assertIn("press planned", events)
        self.assertGreater(sequence.phase_times["hold"] - sequence.phase_times["press"],
                           0.9 * PressParams().final_deg / PressParams().speed_deg_s)

    def test_grips_the_lever_turns_it_and_pulls_the_door_open(self):
        from demos.combiner.sequence import GRIP_SHUT_M
        from demos.cup.pick_demo.grasp import GRIPPER_OPEN_M

        box = box_in_base(Placement((0.66, 0.0, 0.0), 180.0))
        sequence, states = self.run_sequence(box, method="pull", max_t=120.0)
        self.assertEqual(sequence.state, "done", sequence.failure)
        order = ["refine", "align", "grip", "turn", "turn_hold", "crack", "unturn", "pull", "open_hold", "let_go"]
        self.assertEqual([s for s in states if s in order], order)
        # Jaws: open onto the lever, shut to the real jaws' 2 mm (past the URDF's stop) to grip, open to let go.
        from demos.cup.pick_demo.grasp import CLOSED_GAP_M

        p = sequence.plan.params
        self.assertEqual(self.grippers["at_handle"], p.jaw_open_m)
        self.assertLess(p.jaw_open_m, GRIPPER_OPEN_M)
        self.assertGreater(CLOSED_GAP_M + 2 * p.jaw_open_m, 2 * GEOMETRY.handle_radius + 0.02)  # 10 mm clear a side
        self.assertEqual(self.grippers["turn"], GRIP_SHUT_M)
        self.assertAlmostEqual(CLOSED_GAP_M + 2 * GRIP_SHUT_M, 0.002)
        self.assertEqual(self.grippers["let_go"], p.jaw_open_m)
        # Each arc takes the time its joints allow, to within a control step.
        arcs, times = sequence.plan.arc_seconds(), sequence.phase_times
        for arc, after in (("turn", "turn_hold"), ("crack", "unturn"), ("unturn", "pull"), ("pull", "open_hold")):
            self.assertAlmostEqual(times[after] - times[arc], arcs[arc], delta=0.05, msg=arc)
        self.assertGreater(arcs["turn"], 0.9 * p.turn_deg / p.turn_speed_deg_s)

    def test_a_blocked_approach_backs_off_and_takes_the_next_grasp(self):
        box = box_in_base(Placement((0.66, 0.0, 0.0), 180.0))
        first = []

        def blocked(sequence):
            # The arm stops dead partway onto the lever, the first time only.
            moving_in = sequence.state == "moving" and sequence.motion.name.startswith("onto the lever")
            if moving_in and sequence.grasps_tried == 1:
                first.append(sequence.plan.params)
                return sequence.motion.index >= 1
            return False

        sequence, states = self.run_sequence(box, method="pull", max_t=160.0, blocked=blocked)
        self.assertEqual(sequence.state, "done", sequence.failure)
        self.assertEqual(sequence.grasps_tried, 2)
        events = [e["event"] for e in sequence.events]
        self.assertIn("cannot get onto the lever; backing off for the next grasp", events)
        self.assertNotEqual((first[0].pitch_deg, first[0].roll), (sequence.plan.params.pitch_deg, sequence.plan.params.roll))

    def test_no_tag_fails_after_every_stop(self):
        sequence, _ = self.run_sequence(box_in_base(Placement((0.66, 0.0, 0.0), 180.0)), visible=False)
        self.assertEqual(sequence.state, "failed")
        self.assertIn("not seen from any", sequence.failure)
        self.assertEqual(sequence.stop_index, len(SearchParams().headings_deg))


if __name__ == "__main__":
    unittest.main()

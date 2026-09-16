"""Tests for the D1 IK solver. Numpy only, so these run on the system Python."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import d1_ik
from position_only.tool_point import TOOL_BODY
from position_only.workspace import clear_of_body, forward, sample_configs, tool_position


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.joints, _ = d1_ik.load_urdf()

    def test_linear_jacobian_matches_finite_differences(self):
        """The geometric Jacobian is analytic; a numerical derivative must agree."""
        rng = np.random.default_rng(0)
        q = sample_configs(self.joints, 4, rng)
        frames, axes, origins = forward(self.joints, q)
        point = tool_position(frames)
        jac = d1_ik.jacobian(axes, origins, point)

        eps = 1e-6
        for i in range(6):
            step = np.zeros((q.shape[0], 6))
            step[:, i] = eps
            plus, _, _ = forward(self.joints, q + step)
            minus, _, _ = forward(self.joints, q - step)
            numeric = (tool_position(plus) - tool_position(minus)) / (2 * eps)
            np.testing.assert_allclose(jac[:, :3, i], numeric, atol=1e-6)

    def test_angular_jacobian_matches_finite_differences(self):
        rng = np.random.default_rng(1)
        q = sample_configs(self.joints, 4, rng)
        frames, axes, origins = forward(self.joints, q)
        jac = d1_ik.jacobian(axes, origins, tool_position(frames))

        eps = 1e-6
        for i in range(6):
            step = np.zeros((q.shape[0], 6))
            step[:, i] = eps
            plus, _, _ = forward(self.joints, q + step)
            minus, _, _ = forward(self.joints, q - step)
            numeric = d1_ik.rotation_error(minus[TOOL_BODY][0], plus[TOOL_BODY][0]) / (2 * eps)
            np.testing.assert_allclose(jac[:, 3:, i], numeric, atol=1e-5)

    def test_rotation_error_recovers_a_known_rotation(self):
        from position_only.workspace import axis_angle
        for angle in (1e-9, 0.3, 1.5, math.pi - 1e-3):
            axis = np.array([0.0, 0.0, 1.0])
            rot = axis_angle(axis, np.array([angle]))
            err = d1_ik.rotation_error(np.broadcast_to(np.eye(3), (1, 3, 3)).copy(), rot)
            np.testing.assert_allclose(err[0], axis * angle, atol=1e-6)

    def test_rotation_error_is_zero_for_identical_frames(self):
        rng = np.random.default_rng(2)
        q = sample_configs(self.joints, 3, rng)
        frames, _, _ = forward(self.joints, q)
        err = d1_ik.rotation_error(frames["Link6"][0], frames["Link6"][0])
        np.testing.assert_allclose(err, 0.0, atol=1e-9)

    def test_servo_conversion_round_trips(self):
        q = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
        deg = d1_ik.to_servo_deg(q)
        np.testing.assert_allclose(d1_ik.from_servo_deg(deg)[0], q, atol=1e-12)

    def test_servo_conversion_applies_the_measured_sign_flips(self):
        """J0 and J3 are inverted on the wire: each measured on hardware (F-030, F-034)."""
        self.assertEqual(list(d1_ik.SERVO_SIGN), [-1.0, 1.0, 1.0, -1.0, 1.0, 1.0])
        q = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
        deg = d1_ik.to_servo_deg(q)
        for i in (0, 3):
            self.assertAlmostEqual(deg[i], -q[i] * 180.0 / math.pi, places=9)
        for i in (1, 2, 4, 5):
            self.assertAlmostEqual(deg[i], q[i] * 180.0 / math.pi, places=9)

    def test_j3_and_j5_roll_the_same_way_on_the_wire(self):
        """The observation that caught J3: the model had them counter-rotating.

        With the measured signs, equal positive servo commands on J3 and J5 must
        produce opposite-signed URDF angles about their (opposed) axes, i.e. the
        same physical roll direction.
        """
        urdf = d1_ik.from_servo_deg([0, 0, 0, 30, 0, 30])[0]
        self.assertLess(urdf[3], 0.0, "servo J3 +30 must map to a negative URDF Joint4")
        self.assertGreater(urdf[5], 0.0, "servo J5 +30 must map to a positive URDF Joint6")

    def test_a_left_target_commands_a_negative_j0(self):
        """The regression this fixes: a +y (left) target must not swing the arm right.

        Before F-030 the solver's +37.5 deg J0 went out verbatim and the arm moved
        to the dog's right. The commanded servo angle must now have the opposite
        sign to the URDF joint angle.
        """
        joints, _ = d1_ik.load_urdf()
        left = d1_ik.solve(joints, [0.22, 0.16, 0.36], q0=np.zeros(6))
        self.assertTrue(left.converged)
        self.assertGreater(left.q[0], 0.0, "URDF Joint1 should be positive for +y")
        self.assertLess(left.servo_deg[0], 0.0, "servo 0 must be commanded negative for +y")

    def test_servo_limits_are_ordered_after_the_sign_flip(self):
        joints, _ = d1_ik.load_urdf()
        lows, highs = d1_ik.servo_limits_deg(joints)
        self.assertTrue(np.all(lows < highs))


class SolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.joints, _ = d1_ik.load_urdf()

    def test_position_only_solve_reaches_a_reachable_point(self):
        """Every target is the FK of a known configuration, so all are reachable."""
        rng = np.random.default_rng(3)
        goals = sample_configs(self.joints, 25, rng)
        solved = 0
        for q_goal in goals:
            target, _ = d1_ik.tool_pose(self.joints, q_goal)
            res = d1_ik.solve(self.joints, target, q0=np.zeros(6))
            if res.converged:
                solved += 1
                self.assertLess(res.position_error_m, 1e-4)
                reached, _ = d1_ik.tool_pose(self.joints, res.q)
                np.testing.assert_allclose(reached, target, atol=1e-4)
        # A 6-DOF arm from a single fixed seed will miss a few; most must land.
        self.assertGreaterEqual(solved, 20, f"only {solved}/25 converged")

    def test_full_pose_solve_recovers_position_and_orientation(self):
        rng = np.random.default_rng(4)
        solved = 0
        for q_goal in sample_configs(self.joints, 15, rng):
            target_pos, target_rot = d1_ik.tool_pose(self.joints, q_goal)
            res = d1_ik.solve(self.joints, target_pos, target_rot,
                              q0=q_goal + rng.normal(0.0, 0.05, 6), max_iterations=400)
            if res.converged:
                solved += 1
                self.assertLess(res.position_error_m, 1e-4)
                self.assertLess(res.rotation_error_rad, 1e-3)
        self.assertGreaterEqual(solved, 12, f"only {solved}/15 converged")

    def test_solution_always_respects_joint_limits(self):
        lows, highs = d1_ik.joint_limits(self.joints)
        rng = np.random.default_rng(5)
        # Includes targets far outside the workspace, where the solver must still
        # return something commandable rather than diverging.
        for target in rng.uniform(-1.5, 1.5, size=(30, 3)):
            res = d1_ik.solve(self.joints, target, max_iterations=60)
            self.assertTrue(np.all(np.isfinite(res.q)))
            self.assertTrue(np.all(res.q >= lows - 1e-9), f"{res.q} below {lows}")
            self.assertTrue(np.all(res.q <= highs + 1e-9), f"{res.q} above {highs}")

    def test_unreachable_target_reports_failure_without_nan(self):
        res = d1_ik.solve(self.joints, [5.0, 5.0, 5.0], max_iterations=50)
        self.assertFalse(res.converged)
        self.assertTrue(np.all(np.isfinite(res.q)))
        self.assertGreater(res.position_error_m, 1.0)

    def test_seeding_at_the_goal_converges_immediately(self):
        q_goal = np.array([0.2, -0.3, 0.4, 0.1, -0.2, 0.3])
        target, _ = d1_ik.tool_pose(self.joints, q_goal)
        res = d1_ik.solve(self.joints, target, q0=q_goal)
        self.assertTrue(res.converged)
        self.assertEqual(res.iterations, 1)
        np.testing.assert_allclose(res.q, q_goal, atol=1e-6)

    def test_collision_proxy_flags_a_pose_folded_into_the_body(self):
        """base_height engages the trunk/ground proxy; a target under the robot is not clear."""
        res = d1_ik.solve(self.joints, [0.0, 0.0, -0.10], base_height=0.30,
                          max_iterations=120)
        self.assertFalse(res.clear_of_body)
        far = d1_ik.solve(self.joints, [0.40, 0.0, 0.50], base_height=0.30)
        self.assertTrue(far.converged)
        self.assertTrue(far.clear_of_body)


if __name__ == "__main__":
    unittest.main()


class PathClearanceTests(unittest.TestCase):
    """Endpoint clearance is not path clearance (F-036)."""

    @classmethod
    def setUpClass(cls):
        cls.joints, _ = d1_ik.load_urdf()

    def test_traversal_bends_because_short_joints_finish_first(self):
        """Not a straight line: a joint with less to do stops while others continue."""
        a = np.zeros(6)
        b = np.array([1.0, 0.1, 0.0, 0.0, 0.0, 0.0])
        path = d1_ik.traversal_configs(a, b, samples=21)
        np.testing.assert_allclose(path[0], a, atol=1e-12)
        np.testing.assert_allclose(path[-1], b, atol=1e-12)
        # J1 (0.1 rad) finishes at a tenth of the way through and then holds.
        self.assertAlmostEqual(path[-1][1], 0.1, places=9)
        self.assertAlmostEqual(path[len(path) // 2][1], 0.1, places=9)
        # J0 (1.0 rad) is only halfway at the halfway point.
        self.assertAlmostEqual(path[len(path) // 2][0], 0.5, places=6)
        straight = a + (b - a) * 0.5
        self.assertGreater(abs(path[len(path) // 2][1] - straight[1]), 0.04)

    def test_catches_a_colliding_path_between_two_clear_poses(self):
        """The worst pair from a 600-pair search: both ends clear, 58% of the path is not."""
        a = d1_ik.from_servo_deg([-48.1, 61.3, 15.1, 15.7, -68.0, -97.2])[0]
        b = d1_ik.from_servo_deg([49.0, 53.5, 44.6, 115.5, 28.9, -68.0])[0]
        self.assertTrue(clear_of_body(self.joints, a[None, :], 0.15)[0])
        self.assertTrue(clear_of_body(self.joints, b[None, :], 0.15)[0])
        clear, frac = d1_ik.path_clearance(self.joints, a, b, 0.15)
        self.assertFalse(clear, "a path sweeping through the body was reported clear")
        self.assertIsNotNone(frac)
        self.assertTrue(0.0 < frac < 1.0)

    def test_passes_a_genuinely_clear_path(self):
        a = d1_ik.from_servo_deg([-80, -20, 40, 0, 0, 0])[0]
        b = d1_ik.from_servo_deg([80, -20, 40, 0, 0, 0])[0]
        clear, frac = d1_ik.path_clearance(self.joints, a, b, 0.15)
        self.assertTrue(clear)
        self.assertIsNone(frac)

    def test_zero_length_path_is_clear(self):
        a = d1_ik.from_servo_deg([0, -45, 55, 0, 0, 0])[0]
        self.assertTrue(d1_ik.path_clearance(self.joints, a, a, 0.15)[0])


class LevelAttitudeTests(unittest.TestCase):
    """'Level' = approach axis horizontal, no roll, heading free."""

    @classmethod
    def setUpClass(cls):
        cls.joints, _ = d1_ik.load_urdf()
        cls.seed = d1_ik.from_servo_deg([0, -45, 55, 0, 0, 0])[0]

    def test_tool_triad_is_orthonormal_and_right_handed(self):
        m = np.column_stack([d1_ik.TOOL_APPROACH_LOCAL, d1_ik.TOOL_JAW_LOCAL, d1_ik.TOOL_UP_LOCAL])
        np.testing.assert_allclose(m @ m.T, np.eye(3), atol=1e-9)
        self.assertAlmostEqual(float(np.linalg.det(m)), 1.0, places=9)

    def test_level_rotation_is_a_rotation_and_is_level(self):
        _, rot = d1_ik.tool_pose(self.joints, self.seed)
        levelled = d1_ik.level_rotation(rot[None, ...])
        np.testing.assert_allclose(levelled[0] @ levelled[0].T, np.eye(3), atol=1e-9)
        self.assertAlmostEqual(float(np.linalg.det(levelled[0])), 1.0, places=9)
        elevation, roll = d1_ik.tool_attitude_deg(levelled)
        self.assertAlmostEqual(float(elevation[0]), 0.0, places=6)
        self.assertAlmostEqual(float(roll[0]), 0.0, places=6)

    def test_level_rotation_preserves_heading(self):
        """It removes pitch and roll only; which way the gripper faces is untouched."""
        _, rot = d1_ik.tool_pose(self.joints, self.seed)
        before = d1_ik.tool_axes(rot[None, ...])[0][0]
        after = d1_ik.tool_axes(d1_ik.level_rotation(rot[None, ...]))[0][0]
        b = np.arctan2(before[1], before[0])
        a = np.arctan2(after[1], after[0])
        self.assertAlmostEqual(a, b, places=6)

    def test_already_level_frame_is_unchanged(self):
        _, rot = d1_ik.tool_pose(self.joints, self.seed)
        once = d1_ik.level_rotation(rot[None, ...])
        twice = d1_ik.level_rotation(once)
        np.testing.assert_allclose(once, twice, atol=1e-9)

    def test_vertical_approach_does_not_produce_nan(self):
        """Pointing straight up leaves the heading undefined; a fallback must cover it."""
        straight_up = np.column_stack([d1_ik.WORLD_UP, [1.0, 0, 0], [0, 1.0, 0]]) @ \
            np.column_stack([d1_ik.TOOL_APPROACH_LOCAL, d1_ik.TOOL_JAW_LOCAL, d1_ik.TOOL_UP_LOCAL]).T
        out = d1_ik.level_rotation(straight_up[None, ...], fallback_heading=[0.3, 0.0, 0.9])
        self.assertTrue(np.all(np.isfinite(out)))
        np.testing.assert_allclose(out[0] @ out[0].T, np.eye(3), atol=1e-9)

    def test_solve_level_reaches_the_point_and_finishes_level(self):
        for target in ([0.25, 0.10, 0.35], [0.20, 0.18, 0.30], [0.30, 0.0, 0.28]):
            r = d1_ik.solve(self.joints, target, q0=self.seed, level=True, max_iterations=400)
            self.assertTrue(r.converged, f"{target} did not converge")
            self.assertLess(r.position_error_m, 1e-3)
            self.assertLess(abs(r.elevation_deg), 0.5, f"{target} not level: {r.elevation_deg}")
            self.assertLess(abs(r.roll_deg), 0.5, f"{target} rolled: {r.roll_deg}")

    def test_position_only_solutions_are_generally_not_level(self):
        """Without the constraint the gripper points downward; the constraint is doing work."""
        r = d1_ik.solve(self.joints, [0.20, 0.18, 0.30], q0=self.seed)
        self.assertTrue(r.converged)
        self.assertGreater(abs(r.elevation_deg), 5.0)

    def test_level_and_target_rot_together_is_rejected(self):
        _, rot = d1_ik.tool_pose(self.joints, self.seed)
        with self.assertRaises(ValueError):
            d1_ik.solve(self.joints, [0.25, 0.1, 0.35], rot, level=True)

"""Tests for the hardware Cartesian mover.

`d1_hardware` imports CycloneDDS only inside `D1Client.__init__` and
`_message_types`, so the planning and step-limiting logic is testable on the
system Python with no arm and no DDS. A stub stands in for the client.
"""
import math
import sys
import threading
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import d1_hardware
import d1_ik


def bare_client(angles_deg=None, record=None):
    """A D1Client with no DDS, wired for tests.

    Built with __new__ because __init__ opens a CycloneDDS participant. Every
    attribute the non-transport methods touch is set here, so adding one to the
    class shows up as a single failure rather than a dozen.
    """
    c = d1_hardware.D1Client.__new__(d1_hardware.D1Client)
    c._lock = threading.Lock()
    c._seq = 0
    c._min_command_interval_s = 1.0 / d1_hardware.MAX_COMMAND_HZ
    c._last_motion_tx = 0.0
    c._angles_deg = (list(angles_deg) + [0.0]) if angles_deg is not None else None
    c._status = None
    c._last_rx = 0.0
    if record is not None:
        c._send = lambda funcode, data=None: record.append((funcode, data)) or "stub"
    return c


class StubClient:
    """The parts of D1Client the mover uses. Records what was commanded."""

    def __init__(self, angles_deg=None):
        self.angles = list(angles_deg if angles_deg is not None else [0.0] * 6)
        self.commanded = []
        self.feedback_age_s = 0.0

    def wait_for_feedback(self, timeout_s=3.0):
        return list(self.angles) + [0.0]

    def hold_here(self):
        """Controlled stop: command the measured pose. Never a release."""
        return self.set_all_joint_angles(self.get_joint_angles())

    def poll(self, timeout_s=0.0):
        return True

    def get_joint_angles(self):
        return list(self.angles)

    def get_gripper_units(self):
        return 0.0

    def set_all_joint_angles(self, angles_deg, mode=1):
        self.commanded.append(list(angles_deg))
        self.angles = list(angles_deg)      # a perfectly obedient arm
        return "stub"


class MoverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.joints, _ = d1_ik.load_urdf()

    def test_step_towards_caps_the_largest_joint_move(self):
        client = StubClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=5.0)
        goal = [60.0, -60.0, 30.0, 0.0, 0.0, 0.0]          # servo degrees: far away
        step = np.asarray(mover.step_towards(goal))
        self.assertLessEqual(np.max(np.abs(step - np.asarray(client.get_joint_angles()))), 5.0 + 1e-9)

    def test_step_towards_does_not_overshoot_a_near_goal(self):
        client = StubClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=5.0)
        step = np.asarray(mover.step_towards([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        self.assertAlmostEqual(step[0], 1.0, places=6)

    def test_step_towards_works_in_servo_space_not_urdf_space(self):
        """The unit bug SERVO_SIGN would otherwise have introduced.

        A solver result is URDF radians; the measured pose is servo degrees. Mixing
        them mirrors J0. `run()` must convert, so a +y target has to command a
        negative J0 step from zero.
        """
        client = StubClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=5.0,
                                           feedback_period_s=0.0)
        plan = mover.plan([0.22, 0.16, 0.36])
        self.assertGreater(plan.q[0], 0.0)
        commanded = mover.step_towards(d1_ik.to_servo_deg(plan.q))
        self.assertLess(commanded[0], 0.0, "a left target must step J0 negative on the wire")

    def test_dry_run_publishes_nothing(self):
        client = StubClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints)
        target, _ = d1_ik.tool_pose(self.joints, np.array([0.2, -0.3, 0.3, 0.0, 0.1, 0.0]))
        mover.run(target, execute=False, max_cycles=40)
        self.assertEqual(client.commanded, [], "dry run must not command the arm")

    def test_dry_run_converges_on_a_reachable_target(self):
        client = StubClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=5.0)
        target, _ = d1_ik.tool_pose(self.joints, np.array([0.2, -0.3, 0.3, 0.0, 0.1, 0.0]))
        steps = mover.run(target, execute=False, max_cycles=300, tolerance_m=0.005)
        self.assertLess(steps[-1].target_error_m, 0.005)

    def test_every_commanded_step_respects_the_cap(self):
        client = StubClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=3.0)
        target, _ = d1_ik.tool_pose(self.joints, np.array([0.4, -0.5, 0.6, 0.2, -0.3, 0.1]))
        steps = mover.run(target, execute=False, max_cycles=300)
        previous = np.zeros(6)
        for step in steps:
            delta = np.abs(np.asarray(step.commanded_deg) - previous)
            self.assertLessEqual(np.max(delta), 3.0 + 0.02, f"step {step.index} jumped {delta}")
            previous = np.asarray(step.commanded_deg)

    def test_execute_commands_the_arm_and_reaches_the_target(self):
        client = StubClient([0.0] * 6)
        # feedback_period_s=0 because the stub answers instantly; the default is
        # the arm's measured 111 ms.
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=5.0,
                                           feedback_period_s=0.0)
        target, _ = d1_ik.tool_pose(self.joints, np.array([0.2, -0.3, 0.3, 0.0, 0.1, 0.0]))
        steps = mover.run(target, execute=True, max_cycles=300, tolerance_m=0.005)
        self.assertTrue(client.commanded)
        self.assertLess(steps[-1].target_error_m, 0.005)

    def test_clearance_proxy_refuses_to_move(self):
        """A target under the robot must raise rather than command anything."""
        client = StubClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, base_height_m=0.30,
                                           feedback_period_s=0.0)
        with self.assertRaises(RuntimeError):
            mover.run([0.0, 0.0, -0.12], execute=True, max_cycles=5)
        self.assertEqual(client.commanded, [])


class ProtocolTests(unittest.TestCase):
    def test_measured_protocol_constants(self):
        """These encode hardware measurements (F-020); drift here is a real change."""
        self.assertAlmostEqual(d1_hardware.FEEDBACK_PERIOD_S, 0.111, places=3)
        self.assertEqual(d1_hardware.ARM_SERVOS, 6)


if __name__ == "__main__":
    unittest.main()


class WireClampTests(unittest.TestCase):
    """The wire clamp must protect the protocol, not re-impose the solver's margin."""

    def setUp(self):
        self.joints, _ = d1_ik.load_urdf()
        self.sent = []
        self.client = bare_client(record=self.sent)
        self.client.get_gripper_units = lambda: 0.0

    def test_clamps_to_hard_limits_not_soft(self):
        soft_low, _ = d1_ik.joint_limits(self.joints)
        hard_low, _ = d1_ik.joint_limits(self.joints, soft=1.0)
        # -88 deg is outside J1's soft limit (-81) but inside its hard limit (-90).
        self.client.set_all_joint_angles([0.0, -88.0, 0.0, 0.0, 0.0, 0.0])
        sent = self.sent[-1][1]
        self.assertAlmostEqual(sent["angle1"], -88.0, places=3,
                               msg="a valid angle was clamped by the solver's soft margin")
        # -88 must sit outside the soft band but inside the hard one, or this
        # test proves nothing about which limit the clamp used.
        self.assertGreater(soft_low[1] * d1_ik.RAD2DEG, -88.0)
        self.assertLess(hard_low[1] * d1_ik.RAD2DEG, -88.0)

    def test_still_clamps_beyond_the_hard_limit(self):
        self.client.set_all_joint_angles([0.0, -200.0, 0.0, 0.0, 0.0, 0.0])
        sent = self.sent[-1][1]
        hard_low, _ = d1_ik.joint_limits(self.joints, soft=1.0)
        self.assertAlmostEqual(sent["angle1"], hard_low[1] * d1_ik.RAD2DEG, places=3)

    def test_resting_pose_is_commandable_without_a_jump(self):
        """From the measured rest pose, a 3 deg step must survive the clamp."""
        self.client.set_all_joint_angles([2.1, -87.9, 88.7, -3.1, 4.1, 3.1])
        sent = self.sent[-1][1]
        self.assertAlmostEqual(sent["angle1"], -87.9, places=3)


class StallTests(unittest.TestCase):
    """A plant that cannot quite reach its command must end the approach, not spin."""

    class OffsetClient(StubClient):
        """An arm that lands a fixed offset short of every command, like the real one."""

        def __init__(self, angles_deg=None, offset_deg=0.7):
            super().__init__(angles_deg)
            self.offset = offset_deg

        def set_all_joint_angles(self, angles_deg, mode=1):
            self.commanded.append(list(angles_deg))
            self.angles = [a + self.offset for a in angles_deg]
            return "stub"

    def setUp(self):
        self.joints, _ = d1_ik.load_urdf()
        self.target, _ = d1_ik.tool_pose(self.joints, np.array([0.2, -0.3, 0.3, 0.0, 0.1, 0.0]))

    def test_stalls_instead_of_running_to_max_cycles(self):
        client = self.OffsetClient([0.0] * 6, offset_deg=0.7)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=3.0,
                                           feedback_period_s=0.0)
        steps = mover.run(self.target, execute=True, max_cycles=400,
                          tolerance_m=0.0001, stall_patience=12)
        self.assertEqual(mover.last_stop_reason, "stalled")
        self.assertLess(len(steps), 400, "should have given up well before max_cycles")

    def test_a_perfect_arm_still_reaches(self):
        client = self.OffsetClient([0.0] * 6, offset_deg=0.0)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=5.0,
                                           feedback_period_s=0.0)
        steps = mover.run(self.target, execute=True, max_cycles=400, tolerance_m=0.005)
        self.assertEqual(mover.last_stop_reason, "reached")
        self.assertLess(steps[-1].target_error_m, 0.005)

    def test_stall_does_not_fire_while_progress_continues(self):
        """A slow approach must not be mistaken for a stall."""
        client = self.OffsetClient([0.0] * 6, offset_deg=0.0)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=0.5,
                                           feedback_period_s=0.0)
        steps = mover.run(self.target, execute=True, max_cycles=1000, tolerance_m=0.005,
                          stall_patience=12)
        self.assertEqual(mover.last_stop_reason, "reached")
        self.assertGreater(len(steps), 12, "this approach needs more cycles than the patience")


class ParkTests(unittest.TestCase):
    """Joint-space approach: what returns the arm to a pose IK cannot express."""

    def setUp(self):
        self.joints, _ = d1_ik.load_urdf()

    def test_reaches_a_joint_target_under_the_step_cap(self):
        client = StubClient([3.3, -49.9, 68.9, -2.9, -6.8, 3.2])
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=3.0,
                                           feedback_period_s=0.0)
        target = [2.1, -89.9, 89.9, -3.1, 4.1, 3.1]
        steps = mover.approach_joints(target, execute=True, tolerance_deg=0.5)
        self.assertEqual(mover.last_stop_reason, "reached")
        np.testing.assert_allclose(client.get_joint_angles(), target, atol=0.5)
        previous = np.array([3.3, -49.9, 68.9, -2.9, -6.8, 3.2])
        for step in steps:
            self.assertLessEqual(np.max(np.abs(np.asarray(step.commanded_deg) - previous)), 3.0 + 0.02)
            previous = np.asarray(step.commanded_deg)

    def test_dry_run_parks_nothing(self):
        client = StubClient([3.3, -49.9, 68.9, -2.9, -6.8, 3.2])
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        mover.approach_joints([0.0] * 6, execute=False, max_cycles=20)
        self.assertEqual(client.commanded, [])


class SettleTests(unittest.TestCase):
    """settle() must observe only -- it is the honest final measurement."""

    def setUp(self):
        self.joints, _ = d1_ik.load_urdf()

    def test_settle_publishes_nothing(self):
        client = StubClient([2.0, -50.0, 60.0, -3.0, -7.0, 3.0])
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        target, _ = d1_ik.tool_pose(self.joints, d1_ik.from_servo_deg(client.angles)[0])
        err, measured, _ = mover.settle(target, seconds=0.05)
        self.assertEqual(client.commanded, [], "settle must not command the arm")
        self.assertLess(err, 1e-9)
        self.assertEqual(measured, client.angles)

    def test_settle_reports_the_error_at_the_measured_pose(self):
        client = StubClient([2.0, -50.0, 60.0, -3.0, -7.0, 3.0])
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        here, _ = d1_ik.tool_pose(self.joints, d1_ik.from_servo_deg(client.angles)[0])
        target = here + np.array([0.01, 0.0, 0.0])
        err, _, _ = mover.settle(target, seconds=0.05)
        self.assertAlmostEqual(err, 0.01, places=6)


class ReleaseSafetyTests(unittest.TestCase):
    """Release removes torque and the arm falls (F-028). It must never be implicit."""

    def _client(self, angles):
        return bare_client(angles)

    def test_folded_pose_is_safe_to_release(self):
        c = self._client([2.5, -90.8, 91.6, -3.2, 4.5, 2.8])   # measured rest pose
        self.assertTrue(c.is_safe_to_release())

    def test_extended_pose_is_not_safe_to_release(self):
        # The pose the arm was actually released from when it fell 60.8 deg.
        c = self._client([19.6, -30.1, 57.7, -4.3, -12.6, 2.8])
        self.assertFalse(c.is_safe_to_release())

    def test_partially_folded_pose_is_not_safe(self):
        c = self._client([3.3, -49.9, 68.9, -2.9, -6.8, 3.2])
        self.assertFalse(c.is_safe_to_release())


class SweepAnalysisTests(unittest.TestCase):
    """analyse_sweep is pure, so it is checked against synthetic traces."""

    @staticmethod
    def _trace(joint, start, target, t_cmd, latency, rate_deg_s, offset=0.0,
               period=0.111, duration=4.0):
        """A ramp starting `latency` after the command, settling `offset` short."""
        samples, t = [], 0.0
        final = target - offset
        while t < duration:
            if t < t_cmd + latency:
                v = start
            else:
                travelled = rate_deg_s * (t - t_cmd - latency)
                v = start + math.copysign(min(abs(final - start), travelled), final - start)
            angles = [0.0] * 6
            angles[joint] = round(v, 1)          # the arm's 0.1 deg quantisation
            samples.append((t, angles))
            t += period
        return samples

    def test_recovers_latency_rate_and_offset(self):
        samples = self._trace(2, 10.0, 30.0, t_cmd=1.0, latency=0.13,
                              rate_deg_s=60.0, offset=0.7)
        r = d1_hardware.analyse_sweep(samples, 1.0, 2, 10.0, 30.0)
        # Latency is quantised to the feedback period, so allow one sample.
        self.assertLessEqual(r["latency_s"], 0.13 + 0.111 + 1e-9)
        self.assertGreater(r["latency_s"], 0.0)
        self.assertAlmostEqual(r["peak_rate_deg_s"], 60.0, delta=8.0)
        self.assertAlmostEqual(r["steady_state_error_deg"], -0.7, delta=0.15)

    def test_reports_no_motion_when_the_joint_never_moves(self):
        samples = [(i * 0.111, [0.0] * 6) for i in range(40)]
        r = d1_hardware.analyse_sweep(samples, 1.0, 3, 0.0, 20.0)
        self.assertIsNone(r["latency_s"])
        self.assertEqual(r["peak_rate_deg_s"], 0.0)

    def test_perfect_tracking_has_no_steady_state_error(self):
        samples = self._trace(0, 0.0, 15.0, t_cmd=0.5, latency=0.11,
                              rate_deg_s=90.0, offset=0.0)
        r = d1_hardware.analyse_sweep(samples, 0.5, 0, 0.0, 15.0)
        self.assertAlmostEqual(r["steady_state_error_deg"], 0.0, delta=0.06)
        self.assertIsNotNone(r["settle_s"])

    def test_empty_trace_does_not_raise(self):
        self.assertEqual(d1_hardware.analyse_sweep([], 0.0, 0, 0.0, 1.0)["samples"], 0)

    def test_out_window_stops_at_the_return_command(self):
        """The bug this fixes: an unbounded window reads the return as the result."""
        out = self._trace(1, 0.0, 20.0, t_cmd=0.5, latency=0.11, rate_deg_s=60.0)
        back = self._trace(1, 20.0, 0.0, t_cmd=0.5, latency=0.11, rate_deg_s=60.0)
        t_back = out[-1][0] + 0.111
        samples = out + [(t_back + t, a) for t, a in back]
        unbounded = d1_hardware.analyse_sweep(samples, 0.5, 1, 0.0, 20.0)
        bounded = d1_hardware.analyse_sweep(samples, 0.5, 1, 0.0, 20.0, t_end=t_back)
        self.assertAlmostEqual(bounded["steady_state_error_deg"], 0.0, delta=0.1)
        self.assertLess(bounded["hold_band_deg"], 0.2)
        # Unbounded sees the joint back at the start, i.e. the full amplitude out.
        self.assertLess(unbounded["steady_state_error_deg"], -5.0)


class CommandModeTests(unittest.TestCase):
    """Mode 0 is the streaming mode; mode 1 is a slow interpolator (F-031)."""

    def setUp(self):
        self.sent = []
        self.client = bare_client(record=self.sent)
        self.client.get_gripper_units = lambda: 0.0

    def test_all_joint_command_defaults_to_mode_0(self):
        self.client.set_all_joint_angles([0.0] * 6)
        funcode, data = self.sent[-1]
        self.assertEqual(funcode, 2)
        self.assertEqual(data["mode"], 0, "mode 1 is 5x slower and 25x less accurate")

    def test_mode_1_is_still_reachable_for_comparison(self):
        self.client.set_all_joint_angles([0.0] * 6, mode=1)
        self.assertEqual(self.sent[-1][1]["mode"], 1)


class CommandPacingTests(unittest.TestCase):
    """Motion commands must never out-run the arm's 10 Hz control cycle (F-032)."""

    def _client(self):
        c = bare_client(record=[])
        c.get_gripper_units = lambda: 0.0
        return c

    def test_burst_of_commands_is_paced(self):
        import time as _t
        c = self._client()
        start = _t.monotonic()
        for _ in range(5):
            c.set_all_joint_angles([0.0] * 6)
        elapsed = _t.monotonic() - start
        # 5 commands at 10 Hz cannot be delivered faster than 4 intervals.
        self.assertGreaterEqual(elapsed, 4 * (1.0 / d1_hardware.MAX_COMMAND_HZ) - 0.02)

    def test_release_is_never_delayed(self):
        """Release is the abort: pacing must not sit in front of it."""
        import time as _t
        c = self._client()
        c.set_all_joint_angles([0.0] * 6)
        start = _t.monotonic()
        c.release()
        self.assertLess(_t.monotonic() - start, 0.02)


class HoldMeasurementTests(unittest.TestCase):
    """measure_hold reports millimetres. A metres-labelled-mm slip is invisible by eye."""

    class JitterClient(StubClient):
        """Reports a pose that dithers by a fixed amount on one joint."""

        def __init__(self, angles, joint, amplitude_deg):
            super().__init__(angles)
            self._base = list(angles)
            self._joint, self._amp, self._n = joint, amplitude_deg, 0

        def poll(self, timeout_s=0.0):
            self._n += 1
            self.angles = list(self._base)
            self.angles[self._joint] += self._amp if self._n % 2 else 0.0
            return True

        def wait_for_feedback(self, timeout_s=3.0):
            return list(self.angles) + [0.0]

    def test_tool_ptp_is_in_millimetres(self):
        joints, _ = d1_ik.load_urdf()
        base = [-0.2, -45.3, 55.4, -0.1, 0.4, 0.3]
        client = self.JitterClient(base, joint=2, amplitude_deg=0.4)
        r = d1_hardware.measure_hold(client, joints, seconds=0.3)
        # A 0.4 deg swing on J2 moves the tool ~2.8 mm in z at this pose.
        self.assertGreater(max(r["tool_ptp_mm"]), 1.0,
                           "tool_ptp_mm looks like metres, not millimetres")
        self.assertLess(max(r["tool_ptp_mm"]), 20.0)
        self.assertGreaterEqual(max(r["tool_ptp_mm"]), r["tool_max_excursion_mm"],
                                "peak-to-peak cannot be smaller than the max excursion")

    def test_a_perfectly_still_arm_reports_no_jitter(self):
        joints, _ = d1_ik.load_urdf()
        client = self.JitterClient([0.0] * 6, joint=0, amplitude_deg=0.0)
        r = d1_hardware.measure_hold(client, joints, seconds=0.2)
        self.assertAlmostEqual(max(r["tool_ptp_mm"]), 0.0, places=6)
        self.assertAlmostEqual(r["tool_max_excursion_mm"], 0.0, places=6)


class CancelTests(unittest.TestCase):
    """A cancelled approach stops commanding and reports it; it never releases."""

    def setUp(self):
        self.joints, _ = d1_ik.load_urdf()

    def test_run_stops_when_asked_and_leaves_the_arm_holding(self):
        client = StubClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=2.0,
                                           feedback_period_s=0.0)
        target, _ = d1_ik.tool_pose(self.joints, np.array([0.4, -0.5, 0.6, 0.2, -0.3, 0.1]))
        calls = []
        steps = mover.run(target, execute=True, max_cycles=300,
                          should_stop=lambda: calls.append(1) or len(calls) > 3)
        self.assertEqual(mover.last_stop_reason, "cancelled")
        self.assertEqual(len(steps), 3)
        self.assertLess(len(client.commanded), 300)

    def test_approach_joints_honours_cancel(self):
        client = StubClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, max_joint_step_deg=1.0,
                                           feedback_period_s=0.0)
        steps = mover.approach_joints([40.0, 0, 0, 0, 0, 0], execute=True,
                                      should_stop=lambda: len(client.commanded) >= 5)
        self.assertEqual(mover.last_stop_reason, "cancelled")
        self.assertEqual(len(client.commanded), 5)


class MotionProfileTests(unittest.TestCase):
    """analyse_motion distinguishes 'arrives and waits' from 'never stops'."""

    @staticmethod
    def _cycles(step_deg, rate_deg_s, period=0.111, n=12):
        """An arm that moves at `rate` and is capped at `step_deg` per cycle."""
        t, a, pos = 0.0, [], 0.0
        for _ in range(n):
            pos += min(step_deg, rate_deg_s * period)
            a.append((t, [pos] + [0.0] * 5))
            t += period
        return a

    def test_small_cap_reads_as_stepping(self):
        # 5 deg cap while the arm could do 70*0.111 = 7.8 deg: it arrives early.
        r = d1_hardware.analyse_motion(self._cycles(5.0, 70.0), step_cap_deg=5.0)
        self.assertFalse(r["continuous"])
        self.assertLess(r["duty"], 0.85)
        self.assertAlmostEqual(r["displacement_per_cycle_deg"], 5.0, delta=0.2)

    def test_large_cap_reads_as_continuous(self):
        r = d1_hardware.analyse_motion(self._cycles(16.0, 70.0), step_cap_deg=16.0)
        self.assertTrue(r["continuous"])
        self.assertGreaterEqual(r["duty"], 0.85)
        # Capped by the arm's speed, not by the requested step.
        self.assertAlmostEqual(r["displacement_per_cycle_deg"], 70.0 * 0.111, delta=0.3)

    def test_short_trace_is_reported_not_guessed(self):
        self.assertEqual(d1_hardware.analyse_motion([(0.0, [0] * 6)], 5.0)["cycles"], 1)


class OneShotTests(unittest.TestCase):
    """One waypoint per pass: the arm drives its own profile instead of being nudged."""

    class SettlingClient(StubClient):
        """Reports the commanded pose only after a few polls, like a real slew."""

        def __init__(self, angles, lag=3, offset=0.0):
            super().__init__(angles)
            self._goal = list(angles)
            self._lag, self._n, self._offset = lag, 0, offset

        def set_all_joint_angles(self, angles_deg, mode=0):
            self.commanded.append(list(angles_deg))
            self._goal = [a + self._offset for a in angles_deg]
            self._n = 0
            return "stub"

        def poll(self, timeout_s=0.0):
            # The real arm reacts after 60-130 ms, so the first polls show no
            # movement at all. A settle detector must not read that as arrival.
            self._n += 1
            if self._n >= self._lag:
                self.angles = list(self._goal)
            return True

    def setUp(self):
        self.joints, _ = d1_ik.load_urdf()
        self.target, _ = d1_ik.tool_pose(self.joints, np.array([0.2, -0.3, 0.3, 0.0, 0.1, 0.0]))

    class StaleReadClient(StubClient):
        """Feedback arrives every 3rd poll; the others re-read the same cached angles.

        Reproduces the real client, whose poll returns False when nothing new has
        arrived. A settle detector that counts stale reads as stillness stops mid-slew.
        """

        def __init__(self, angles, steps=6):
            super().__init__(angles)
            self._goal, self._i, self._steps = list(angles), 0, steps

        def set_all_joint_angles(self, angles_deg, mode=0):
            self.commanded.append(list(angles_deg))
            self._goal, self._i = [float(a) for a in angles_deg], 0
            return "stub"

        def poll(self, timeout_s=0.0):
            self._i += 1
            if self._i % 3:
                return False                      # nothing new this time
            k = min(1.0, (self._i / 3) / self._steps)
            start = self.angles
            self.angles = [s + (g - s) * k for s, g in zip(start, self._goal)] \
                if k < 1.0 else list(self._goal)
            return True

    def test_stale_reads_are_not_mistaken_for_arrival(self):
        """The regression: repeats of a cached sample must not end the wait."""
        client = self.StaleReadClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        steps = mover.run_oneshot(self.target, execute=True, passes=1)
        np.testing.assert_allclose(client.get_joint_angles(), client.commanded[-1], atol=1e-6)
        self.assertLess(steps[-1].target_error_m, 1e-3, "returned while the arm was still moving")

    def test_waits_through_the_command_latency_before_declaring_arrival(self):
        """The regression: three stationary polls after a command are latency, not arrival."""
        client = self.SettlingClient([0.0] * 6, lag=6)
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        steps = mover.run_oneshot(self.target, execute=True, passes=1)
        self.assertLess(steps[-1].target_error_m, 1e-3,
                        "returned before the arm started moving")

    def test_sends_one_command_per_pass_not_one_per_cycle(self):
        """An arm that lands short needs a second waypoint -- but only one more."""
        client = self.SettlingClient([0.0] * 6, offset=2.0)
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        steps = mover.run_oneshot(self.target, execute=True, passes=2)
        self.assertEqual(len(client.commanded), 2, "one waypoint per pass, not per cycle")
        self.assertEqual(len(steps), 2)

    def test_a_perfect_arm_needs_only_one_waypoint(self):
        """A sub-floor correction is skipped: commanding it moves nothing (F-031)."""
        client = self.SettlingClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        steps = mover.run_oneshot(self.target, execute=True, passes=3)
        self.assertEqual(len(client.commanded), 1)
        self.assertEqual(len(steps), 1)
        self.assertEqual(mover.last_stop_reason, "at the arm's resolution")
        self.assertLess(steps[-1].target_error_m, 1e-3)

    def test_refuses_an_excursion_beyond_the_limit(self):
        client = self.SettlingClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        with self.assertRaises(RuntimeError) as ctx:
            mover.run_oneshot(self.target, execute=True, max_excursion_deg=2.0)
        self.assertIn("excursion", str(ctx.exception))
        self.assertEqual(client.commanded, [], "nothing may be sent when the guard trips")

    def test_extra_passes_correct_a_tracking_offset(self):
        """A plant that lands short should be closer after the second pass."""
        client = self.SettlingClient([0.0] * 6, offset=2.0)
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        steps = mover.run_oneshot(self.target, execute=True, passes=3)
        self.assertEqual(len(steps), 3)
        self.assertLessEqual(steps[-1].target_error_m, steps[0].target_error_m)

    def test_dry_run_sends_nothing(self):
        client = self.SettlingClient([0.0] * 6)
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        mover.run_oneshot(self.target, execute=False, passes=2)
        self.assertEqual(client.commanded, [])

    def test_cancel_holds_where_it_is_and_does_not_release(self):
        client = self.SettlingClient([0.0] * 6, lag=99)     # never settles on its own
        client.release = lambda: (_ for _ in ()).throw(AssertionError("must not release"))
        mover = d1_hardware.CartesianMover(client, self.joints, feedback_period_s=0.0)
        calls = []
        mover.run_oneshot(self.target, execute=True, passes=1,
                          should_stop=lambda: calls.append(1) or len(calls) > 2)
        self.assertEqual(mover.last_stop_reason, "cancelled")
        # The last command is a hold at the measured pose, not a release.
        self.assertEqual(client.commanded[-1], client.get_joint_angles())

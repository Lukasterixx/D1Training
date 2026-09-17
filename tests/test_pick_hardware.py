"""The scripted pick driven against the hardware seam, with a fake arm and a fake camera.

No CycloneDDS, no RealSense, no Isaac: `pick_demo.hardware` only ever touches its client through the
handful of methods `D1Client` and `DirectD1` both have, so a stand-in that implements those exercises
the real control path. What these tests are really for is the safety behaviour -- that a dry run sends
nothing, that a stop holds rather than releases, and that an oversized step is refused -- because those
are the parts whose first real trial involves a moving arm.
"""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import d1_hardware
import d1_ik
from pick_demo import camera, grasp, hardware, perception
from position_only.workspace import load_urdf

UP = np.array([0.0, 0.0, 1.0])
BASE_HEIGHT = 0.12
CUP_XY = np.array([0.42, 0.03])
CUP_RADIUS = 0.0275


def cup_top():
    return np.array([CUP_XY[0], CUP_XY[1], -BASE_HEIGHT + 0.10])


class FakeArm:
    """A D1Client-shaped arm that follows its commands at the measured slew rate.

    `set_all_joint_angles` records rather than transmits, so a test can assert on exactly what a run
    would have put on the wire.
    """

    def __init__(self, rate_deg_s: float = 70.0, tick_s: float = 0.1):
        self.servo = list(d1_ik.to_servo_deg(np.zeros(6)))
        self.gripper_units = 40.0
        self.commands = []
        self.gripper_commands = []
        self.holds = 0
        self.releases = 0
        self.rate, self.tick = rate_deg_s, tick_s

    def poll(self, timeout_s: float = 0.0) -> bool:
        return True

    def get_joint_angles(self):
        return list(self.servo)

    def get_gripper_units(self):
        return self.gripper_units

    def set_all_joint_angles(self, angles_deg, mode: int = 0, gripper=None):
        angles = [float(v) for v in angles_deg]
        self.commands.append(angles)
        self.gripper_commands.append(gripper)
        if gripper is not None:
            self.gripper_units = float(gripper)
        step = self.rate * self.tick
        self.servo = [s + max(-step, min(step, a - s)) for s, a in zip(self.servo, angles)]
        return "sent"

    def hold_here(self):
        self.holds += 1
        return "hold"

    def release(self):
        self.releases += 1
        return "release"


class FakeCamera:
    """A camera pipeline handing out one frame per call, with depth unless told otherwise."""

    def __init__(self, with_depth: bool = True):
        self.with_depth = with_depth
        self.seq = 0

    def wait_frame(self, after_seq: int, timeout_s: float = 2.0):
        self.seq += 1
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 0.4, dtype=np.float32) if self.with_depth else None
        return self.seq, rgb, depth, float(self.seq)


class FakePerception:
    """Always finds the cup where it actually is, so the tests are about control, not vision."""

    def __init__(self):
        self.detection = perception.Detection("cup", 0.9, (0, 0, 1, 1), None)
        self.observed = 0

    def _estimate(self):
        truth = cup_top()
        return perception.CupEstimate(truth, CUP_RADIUS, truth[2], -BASE_HEIGHT, 500, "rim_circle", 0.001, 300.0)

    def observe(self, frame):
        self.observed += 1
        return perception.CupObservation(self.detection, self._estimate(), np.eye(4), 1.0)

    def reobserve(self, frame, prior, gate_m: float = 0.04):
        return perception.CupObservation(self.detection, self._estimate(), np.eye(4), 1.0)


class FakeClock:
    """Time that advances one command cycle per read, so a whole pick runs without any real waiting."""

    def __init__(self, step_s: float = hardware.COMMAND_PERIOD_S):
        self.now, self.step = 0.0, step_s

    def __call__(self) -> float:
        value = self.now
        self.now += self.step
        return value


def run(**kwargs):
    joints, links = load_urdf()
    defaults = dict(client=FakeArm(), joints=joints, links=links,
                    camera_model=camera.CAMERAS["d435"], mount=camera.WristMount(),
                    perception=FakePerception(), pipeline=FakeCamera(),
                    base_height_m=BASE_HEIGHT, up_b=UP, execute=False, max_time_s=60.0,
                    clock=FakeClock(), sleep=lambda _s: None)
    defaults.update(kwargs)
    return defaults["client"], hardware.run_pick(**defaults)


class DryRunTests(unittest.TestCase):
    def test_a_dry_run_plans_the_whole_pick_and_sends_nothing(self):
        arm, result = run(execute=False)
        self.assertEqual(arm.commands, [], "a dry run must not write to the arm")
        self.assertEqual(result.commands_sent, 0)
        self.assertFalse(result.executed)
        # It still did the work: real frames, real perception, a located cup and a full run.
        self.assertIsNotNone(result.cup)
        self.assertGreater(result.frames_looked_at, 0)
        self.assertTrue(result.ok, result.reason)

    def test_a_dry_run_says_its_arm_pose_was_imagined(self):
        """The result must never let a dry run read as evidence the arm can follow the plan."""
        _, result = run(execute=False)
        self.assertEqual(result.feedback, "virtual (dry run)")
        self.assertIn("virtual", result.as_dict()["interpretation"])
        _, live = run(execute=True)
        self.assertEqual(live.feedback, "arm")

    def test_the_virtual_arm_starts_from_where_the_real_arm_is(self):
        """A dry run must not pretend the arm is somewhere it is not, or its plans start from a lie."""
        arm = FakeArm()
        arm.servo = list(d1_ik.to_servo_deg(np.array([0.2, 0.3, 0.4, 0.0, 0.1, 0.0])))
        virtual = hardware._VirtualArm(arm.get_joint_angles())
        np.testing.assert_allclose(virtual.advance(0.0), arm.get_joint_angles(), atol=1e-9)

    def test_a_dry_run_never_holds_or_releases_the_arm(self):
        arm, _ = run(execute=False)
        self.assertEqual((arm.holds, arm.releases), (0, 0))


class GripperTests(unittest.TestCase):
    def test_the_gripper_rides_along_with_the_arm_pose(self):
        """One message carries both, so the jaw and the pose never arrive a cycle apart (F-032)."""
        arm, result = run(execute=True, grip_gripper=True)
        self.assertTrue(result.gripper_intent["commanded"])
        self.assertEqual(len(arm.gripper_commands), len(arm.commands))
        self.assertTrue(all(g is not None for g in arm.gripper_commands))
        for command in arm.commands:
            self.assertEqual(len(command), 6, "arm angles and gripper travel on the same message")

    def test_the_commanded_units_follow_the_sequence_and_stay_in_range(self):
        arm, _ = run(execute=True, grip_gripper=True)
        low, high = d1_hardware.GRIPPER_UNITS_RANGE
        for units in arm.gripper_commands:
            self.assertGreaterEqual(units, low)
            self.assertLessEqual(units, high)
        # The sequence opens the jaws before the descent and closes them on the cup, so the run must
        # contain more than one distinct width -- a constant gripper would mean it never opened.
        self.assertGreater(len(set(arm.gripper_commands)), 1)

    def test_open_travel_asks_for_the_open_end_of_the_scale(self):
        """Servo 6 runs open-to-closed and the planner's travel runs closed-to-open (F-062).

        This is the test that would have caught the inversion: with the two senses the same way round,
        a pick opens the jaw at the moment it means to close on the cup.
        """
        self.assertAlmostEqual(d1_hardware.finger_travel_to_gripper_units(grasp.GRIPPER_OPEN_M),
                               d1_hardware.GRIPPER_UNITS_OPEN, places=6)
        self.assertAlmostEqual(d1_hardware.finger_travel_to_gripper_units(grasp.GRIPPER_CLOSED_M),
                               d1_hardware.GRIPPER_UNITS_CLOSED, places=6)

    def test_closing_on_a_cup_asks_for_fewer_units_than_holding_the_jaws_open(self):
        """The direction stated as behaviour rather than as endpoints.

        Closing is the *negative* direction on this arm (F-063), which is the half the vendor's
        advertised 0-65 does not cover and which nothing had commanded until it was probed.
        """
        wide = d1_hardware.finger_travel_to_gripper_units(grasp.GRIPPER_OPEN_M)
        on_cup = d1_hardware.finger_travel_to_gripper_units(grasp.grip_travel_m(0.070, 0.004))
        touching = d1_hardware.finger_travel_to_gripper_units(grasp.GRIPPER_CLOSED_M)
        self.assertLess(on_cup, wide)
        self.assertLess(touching, on_cup)

    def test_a_jaw_shut_past_the_cad_stop_asks_for_the_closed_end(self):
        """A wall pinch commands negative travel, and negative has to mean shut.

        The CAD pads stop 17.2 mm apart, the arm's meet (F-063), so `grasp.closed_travel_m` is negative
        and the pinch commands it. Taking `abs` of it sent -7.6 mm to -2.07 units -- a quarter open --
        which would have opened the jaws on the wall they were told to close on.
        """
        shut = grasp.closed_travel_m(grasp.GraspParams())
        self.assertLess(shut, 0.0)
        self.assertAlmostEqual(d1_hardware.finger_travel_to_gripper_units(shut),
                               d1_hardware.GRIPPER_UNITS_CLOSED, places=6)
        self.assertLess(d1_hardware.finger_travel_to_gripper_units(shut),
                        d1_hardware.finger_travel_to_gripper_units(grasp.GRIPPER_CLOSED_M) + 1e-9)

    def test_a_pinch_uses_only_the_two_commands_that_are_calibrated(self):
        """Every intermediate opening is a guess on this arm; the endpoints are measured (F-063).

        So a wall pinch goes down at the widest the command path reaches and shuts at the pads touching,
        and asks for nothing in between.
        """
        joints, links = d1_ik.load_urdf()
        top = np.array([0.436, 0.030, -0.085 + 0.10])
        cup = perception.CupEstimate(top, 0.0419, top[2], -0.085, 500, "rim_circle")
        plan = grasp.plan_top_down_grasp(joints, links, cup, np.array([0.0, 0.0, 1.0]), np.zeros(6), 0.085,
                                         grasp.GraspParams())
        self.assertEqual(plan.mode, "pinch")
        self.assertAlmostEqual(d1_hardware.finger_travel_to_gripper_units(plan.gripper_descend_m),
                               d1_hardware.GRIPPER_UNITS_OPEN, places=6)
        self.assertAlmostEqual(d1_hardware.finger_travel_to_gripper_units(plan.gripper_grasp_m),
                               d1_hardware.GRIPPER_UNITS_CLOSED, places=6)

    def test_the_measured_span_is_inside_what_the_protocol_accepts(self):
        """The jog control must still be able to command past the span, to find where the arm stops."""
        self.assertGreaterEqual(d1_hardware.GRIPPER_UNITS_SPAN[0], d1_hardware.GRIPPER_UNITS_RANGE[0])
        self.assertLess(d1_hardware.GRIPPER_UNITS_SPAN[1], d1_hardware.GRIPPER_UNITS_RANGE[1])

    def test_units_and_travel_round_trip(self):
        for travel in (0.0, 0.012, 0.0244, 0.03):
            units = d1_hardware.finger_travel_to_gripper_units(travel)
            self.assertAlmostEqual(d1_hardware.gripper_units_to_finger_travel(units), travel, places=9)

    def test_out_of_range_travel_is_clamped_not_sent_raw(self):
        """A wild width must not reach the wire: the arm's response to one is unknown.

        The negative case changed on 2026-09-17 and the change is the point of it. It used to assert
        that -0.05 m of travel came back as the *open* end, because the conversion took `abs` of its
        argument. Travel below zero now means a jaw shut past the CAD's stop -- what a wall pinch asks
        for (`grasp.closed_travel_m`) -- so a wild negative clamps to the closed end instead. Both ends
        still clamp; only the sense of the negative one moved.
        """
        # Travel beyond the stroke is as open as the scale goes, never past it.
        self.assertEqual(d1_hardware.finger_travel_to_gripper_units(1.0), d1_hardware.GRIPPER_UNITS_OPEN)
        self.assertEqual(d1_hardware.finger_travel_to_gripper_units(-0.05), d1_hardware.GRIPPER_UNITS_CLOSED)
        # Units past either end of the jaw span come back as that end's travel, never as a negative one.
        self.assertEqual(d1_hardware.gripper_units_to_finger_travel(-500.0), 0.0)
        self.assertEqual(d1_hardware.gripper_units_to_finger_travel(500.0), d1_hardware.GRIPPER_FULL_TRAVEL_M)
        for units in (-30.0, -19.8, -10.0, 0.0, 40.0):
            travel = d1_hardware.gripper_units_to_finger_travel(units)
            self.assertGreaterEqual(travel, 0.0)
            self.assertLessEqual(travel, d1_hardware.GRIPPER_FULL_TRAVEL_M)

    def test_it_can_still_be_turned_off(self):
        """The reach-only mode the first hardware runs used, kept for when the jaw must not move."""
        arm, result = run(execute=True, grip_gripper=False)
        self.assertFalse(result.gripper_intent["commanded"])
        self.assertTrue(all(g is None for g in arm.gripper_commands))

    def test_a_dry_run_never_sends_a_gripper_command(self):
        arm, result = run(execute=False, grip_gripper=True)
        self.assertEqual(arm.gripper_commands, [])
        self.assertFalse(result.gripper_intent["commanded"])

    def test_the_record_does_not_claim_millimetres(self):
        """The scale is unmeasured; a run must say what it asked for, not what the jaw did."""
        _, result = run(execute=True, grip_gripper=True)
        intent = result.gripper_intent
        self.assertIn("units_asked", intent)
        self.assertIn("unverified", intent["why"])
        self.assertIn("nobody has measured", result.as_dict()["interpretation"].replace("scale nobody has measured", "nobody has measured"))


class SafetyTests(unittest.TestCase):
    def test_stopping_holds_the_arm_rather_than_releasing_it(self):
        """Release drops the arm (F-028), so it must never be the abort path."""
        calls = {"n": 0}

        def should_stop():
            calls["n"] += 1
            return calls["n"] > 5

        arm, result = run(execute=True, should_stop=should_stop)
        self.assertEqual(arm.releases, 0, "the pick must never release the arm")
        self.assertEqual(arm.holds, 1)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "stopped")

    def test_a_dry_run_that_is_stopped_still_sends_nothing(self):
        arm, _ = run(execute=False, should_stop=lambda: True)
        self.assertEqual((arm.commands, arm.holds, arm.releases), ([], 0, 0))

    def test_an_oversized_joint_step_is_refused_before_it_is_sent(self):
        """The sanity bound on a wild target. A large step is normal (F-035), so this forces one open."""
        arm, result = run(execute=True, max_step_deg=0.001)
        self.assertEqual(arm.commands, [])
        self.assertIn("refused", result.reason)
        self.assertFalse(result.ok)

    def test_every_commanded_step_stays_under_the_cap(self):
        arm = FakeArm()
        _, result = run(client=arm, execute=True, max_step_deg=hardware.MAX_STEP_DEG)
        reached = [list(d1_ik.to_servo_deg(np.zeros(6)))]
        for command in arm.commands:
            step = max(abs(c - r) for c, r in zip(command, reached[-1]))
            self.assertLessEqual(step, hardware.MAX_STEP_DEG + 1e-6)
            reached.append(command)

    def test_it_gives_up_rather_than_running_forever(self):
        _, result = run(execute=False, max_time_s=0.0)
        self.assertFalse(result.ok)
        self.assertIn("gave up", result.reason)


class CameraTests(unittest.TestCase):
    def test_a_camera_without_depth_yields_no_cup_rather_than_a_wrong_one(self):
        """Colour alone cannot place a cup; the pick must fail to find it, not invent a position."""
        _, result = run(pipeline=FakeCamera(with_depth=False), execute=False)
        self.assertIsNone(result.cup)
        self.assertFalse(result.ok)

    def test_frames_carry_the_arm_pose_read_at_capture(self):
        arm = FakeArm()
        arm.servo = list(d1_ik.to_servo_deg(np.array([0.1, 0.2, 0.3, 0.0, 0.1, 0.0])))
        frames = hardware.CameraPipelineFrames(FakeCamera(), arm, UP)
        frame = frames()
        np.testing.assert_allclose(frame.q, d1_ik.from_servo_deg(arm.get_joint_angles())[0], atol=1e-9)
        np.testing.assert_allclose(frame.up_b, UP)
        self.assertEqual(frame.depth.shape, frame.rgb.shape[:2])

    def test_each_frame_is_read_only_once(self):
        """Re-reading one image would let a stale look pass as a fresh detection."""
        pipeline = FakeCamera()
        frames = hardware.CameraPipelineFrames(pipeline, FakeArm(), UP)
        seqs = []
        for _ in range(3):
            frames()
            seqs.append(frames.last_seq)
        self.assertEqual(seqs, sorted(set(seqs)))


class MetadataTests(unittest.TestCase):
    def test_the_record_states_what_was_assumed(self):
        meta = hardware.run_metadata(camera_model=camera.CAMERAS["d435"], mount=camera.WristMount(),
                                     base_height_m=0.02, up_b=UP, execute=False)
        self.assertEqual(meta["frame"]["base_height_m"], 0.02)
        self.assertIn("assumed", meta["mount"]["source"])
        # The gripper is commanded by default now, and the record must say the scale is unverified
        # rather than imply the jaw went to a known width.
        self.assertTrue(meta["gripper"]["commanded"])
        self.assertIn("unverified", meta["gripper"]["why"])
        self.assertIn("unmeasured scale", meta["scope"])

    def test_the_record_says_so_when_the_gripper_is_left_out(self):
        meta = hardware.run_metadata(camera_model=camera.CAMERAS["d435"], mount=camera.WristMount(),
                                     base_height_m=0.02, up_b=UP, execute=False, grip_gripper=False)
        self.assertFalse(meta["gripper"]["commanded"])
        self.assertIn("not a grasp", meta["scope"])


if __name__ == "__main__":
    unittest.main()

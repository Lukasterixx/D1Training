"""CPU tests for the UniFP demos: placement geometry, the phase machine, the wrist servo.

Nothing here starts a simulator. What it checks is the half of the demo that can be wrong
silently -- a grasp point outside the trained workspace, a phase list whose clock does not add up,
a servo that rolls the wrist the long way round -- because those show up in a run as a worse
success rate rather than as an error.

    python -m unittest discover -s demos/unifp/tests -v
"""
import math
import random
import unittest

from demos.combiner.geometry import GEOMETRY
from demos.unifp import props, script

# UniFP's trained goal ranges, written out rather than imported: `unifp_isaaclab.interface` pulls
# in torch, and these tests are meant to run on the system Python as `test_evidence.py` does.
RADIUS_RANGE = (0.30, 0.58)
PITCH_RANGE = (-math.pi / 4, math.pi / 3)
YAW_RANGE = (-2 * math.pi / 5, 2 * math.pi / 5)
GOAL_CENTRE_Z = 0.49


def sphere(point):
    dz = point[2] - GOAL_CENTRE_Z
    radius = math.sqrt(point[0] ** 2 + point[1] ** 2 + dz ** 2)
    return radius, math.asin(dz / radius), math.atan2(point[1], point[0])


def in_workspace(point):
    radius, pitch, yaw = sphere(point)
    return (RADIUS_RANGE[0] <= radius <= RADIUS_RANGE[1]
            and PITCH_RANGE[0] <= pitch <= PITCH_RANGE[1]
            and YAW_RANGE[0] <= yaw <= YAW_RANGE[1])


def unit(vector):
    norm = math.sqrt(sum(v * v for v in vector))
    return tuple(v / norm for v in vector)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


class PlacementGeometry(unittest.TestCase):
    def test_cup_grasp_sits_on_the_cup(self):
        site = props.CupSite(cup_xy_m=(0.515, 0.0), table_centre_x_m=0.62)
        grasp = site.grasp_point_m
        self.assertEqual(grasp[:2], site.cup_xy_m)
        self.assertAlmostEqual(grasp[2], props.TABLE_TOP_M + props.CUP_GRASP_UP_M)
        # Below the rim, or the jaws close on air above the cup.
        self.assertLess(props.CUP_GRASP_UP_M, props.CUP_HEIGHT_M)

    def test_cup_lift_comes_in_as_it_rises(self):
        site = props.CupSite(cup_xy_m=(0.515, 0.0), table_centre_x_m=0.62)
        grasp, lift = site.grasp_point_m, site.lift_point_m
        self.assertAlmostEqual(lift[2] - grasp[2], props.CUP_LIFT_UP_M)
        self.assertLess(math.hypot(*lift[:2]), math.hypot(*grasp[:2]))

    def test_cup_clears_the_table_edge(self):
        for seed in range(50):
            site = props.sample_cup_site(random.Random(seed))
            near_face = site.table_centre_x_m - props.TABLE_SIZE_M[0] / 2
            self.assertGreater(site.cup_xy_m[0] - props.CUP_DIAMETER_M / 2, near_face,
                               "cup overhangs the near edge of the table")
            self.assertLess(abs(site.cup_xy_m[1]) + props.CUP_DIAMETER_M / 2,
                            props.TABLE_SIZE_M[1] / 2, "cup overhangs the side of the table")

    def test_lever_grasp_is_on_the_bar(self):
        site = props.BoxSite(distance_m=0.66, bearing_deg=0.0, yaw_deg=180.0)
        grasp = site.grasp_point_m()
        self.assertAlmostEqual(grasp[2], props.POST_TOP_M + GEOMETRY.bottom + GEOMETRY.height / 2)
        # Between the spindle and the end of the lever.
        self.assertLess(props.LEVER_GRASP_OFFSET_M, GEOMETRY.handle_length)
        self.assertGreater(props.LEVER_GRASP_OFFSET_M, 0.0)

    def test_turning_the_lever_moves_the_grasp_down_and_in(self):
        site = props.BoxSite(distance_m=0.66, bearing_deg=0.0, yaw_deg=180.0)
        flat, turned = site.grasp_point_m(0.0), site.grasp_point_m(props.LEVER_TURN_DEG)
        self.assertLess(turned[2], flat[2], "a turned lever's grasp point must drop")
        drop = flat[2] - turned[2]
        self.assertAlmostEqual(
            drop, props.LEVER_GRASP_OFFSET_M * math.sin(math.radians(props.LEVER_TURN_DEG)),
            places=6)

    def test_lever_axis_is_across_the_approach(self):
        for seed in range(20):
            site = props.sample_box_site(random.Random(seed))
            axis = site.lever_axis_unit()
            approach = site.approach_unit()
            self.assertAlmostEqual(dot(unit(axis), unit(approach)), 0.0, places=6)

    def test_turn_angle_clears_the_latch(self):
        self.assertTrue(script.latch_would_release(props.LEVER_TURN_DEG))
        self.assertFalse(script.latch_would_release(GEOMETRY.handle_release_deg - 1))


class CommandedGoalsStayInsideTheTrainedWorkspace(unittest.TestCase):
    """The invariant the whole demo rests on.

    UniFP's goal generator draws from a bounded sphere and the policy has never been asked for a
    goal outside it. A placement whose grasp point or door arc leaves that sphere is not a hard
    failure -- the policy simply does something unmeasured -- so it has to be caught here rather
    than read off a worse success rate later.
    """

    def _sweep(self, phases, key, sampler, seeds=200):
        worst = []
        for seed in range(seeds):
            state = {key: sampler(random.Random(seed))}
            demo = script.DemoScript(list(phases))
            demo.reset(script.HOME_POINT, script.JAW_WIDE_M)
            elapsed = 0.0
            while not demo.finished and elapsed < 60.0:
                command = demo.update(0.02, state)
                elapsed += 0.02
                if not in_workspace(command.point):
                    worst.append((seed, command.phase, command.point, sphere(command.point)))
        return worst

    def test_cup_phases(self):
        bad = self._sweep(script.cup_phases(), "cup_site", props.sample_cup_site)
        self.assertEqual(bad[:3], [], f"{len(bad)} cup goals outside the trained sphere")

    def test_combiner_phases(self):
        bad = self._sweep(script.combiner_phases(), "box_site", props.sample_box_site)
        self.assertEqual(bad[:3], [], f"{len(bad)} combiner goals outside the trained sphere")

    def test_home_point_is_reachable(self):
        self.assertTrue(in_workspace(script.HOME_POINT))


class PhaseMachine(unittest.TestCase):
    def setUp(self):
        self.state = {"cup_site": props.sample_cup_site(random.Random(3)),
                      "box_site": props.sample_box_site(random.Random(4))}

    def _run(self, phases):
        demo = script.DemoScript(list(phases))
        demo.reset(script.HOME_POINT, script.JAW_WIDE_M)
        seen, elapsed = [], 0.0
        while not demo.finished and elapsed < 60.0:
            command = demo.update(0.02, self.state)
            elapsed += 0.02
            if not seen or seen[-1][0] != command.phase:
                seen.append((command.phase, round(elapsed, 3)))
        return demo, seen, elapsed

    def test_phases_run_in_order_and_finish_on_the_clock(self):
        phases = script.cup_phases()
        demo, seen, elapsed = self._run(phases)
        # The last `update` of a run is the one that finds the script finished, and it reports
        # "done" while holding the final point.
        self.assertEqual([name for name, _ in seen],
                         [phase.name for phase in phases] + ["done"])
        self.assertAlmostEqual(elapsed, demo.total_s, delta=0.05)

    def test_a_finished_script_holds_its_last_point(self):
        phases = script.cup_phases()
        demo, _, _ = self._run(phases)
        first = demo.update(0.02, self.state)
        second = demo.update(0.02, self.state)
        self.assertTrue(first.finished and second.finished)
        self.assertEqual(first.point, second.point)
        self.assertEqual(first.phase, "done")

    def test_a_phase_shorter_than_one_step_still_issues_its_endpoint(self):
        phases = [script.Phase("a", 0.005, target=lambda st: (0.4, 0.0, 0.7)),
                  script.Phase("b", 0.005, target=lambda st: (0.45, 0.0, 0.7)),
                  script.Phase("c", 1.0, target=lambda st: (0.5, 0.0, 0.7))]
        demo = script.DemoScript(phases)
        demo.reset((0.35, 0.0, 0.7), script.JAW_WIDE_M)
        command = demo.update(0.02, self.state)
        self.assertEqual(command.phase, "c")
        self.assertEqual([entry["phase"] for entry in demo.log], ["a", "b"])

    def test_the_jaws_ramp_rather_than_step(self):
        phases = script.cup_phases()
        demo = script.DemoScript(list(phases))
        demo.reset(script.HOME_POINT, script.JAW_WIDE_M)
        jaws, elapsed = [], 0.0
        while not demo.finished and elapsed < 60.0:
            jaws.append(demo.update(0.02, self.state).jaw_m)
            elapsed += 0.02
        self.assertLess(max(abs(b - a) for a, b in zip(jaws, jaws[1:])), 0.002,
                        "a jaw target that steps closes the fingers as fast as the drive allows")
        self.assertAlmostEqual(min(jaws), script.JAW_CUP_SHUT_M, places=6)
        self.assertAlmostEqual(max(jaws), script.JAW_WIDE_M, places=6)

    def test_the_servo_is_off_while_the_arm_is_still_travelling(self):
        for phases in (script.cup_phases(), script.combiner_phases()):
            by_name = {phase.name: phase for phase in phases}
            self.assertFalse(by_name["stand"].servo)
            self.assertFalse(by_name["approach"].servo)
            self.assertTrue(by_name["settle"].servo, "the hand must be turned before it goes in")
            self.assertTrue(by_name["arrive"].servo)

    def test_the_combiner_holds_the_lever_past_the_release_while_it_cracks_the_door(self):
        timing = script.CombinerTiming()
        self.assertGreater(timing.turn_deg, GEOMETRY.handle_release_deg)
        self.assertLess(timing.ease_deg, GEOMETRY.handle_reset_deg)
        phases = {phase.name: phase for phase in script.combiner_phases(timing)}
        # The crack phase must still be commanding the turned lever, or the latch re-engages.
        point = phases["crack"].path(self.state, 0.0)
        site = self.state["box_site"]
        self.assertEqual(point, site.grasp_point_m(timing.turn_deg, 0.0))


class WristServo(unittest.TestCase):
    """Algebra only -- the servo's behaviour on a robot is measured by a run, not asserted here."""

    def setUp(self):
        try:
            import torch  # noqa: F401
        except ImportError:  # pragma: no cover - the system Python has no torch
            self.skipTest("torch is not importable outside the Isaac environment")
        from demos.unifp import wrist

        self.wrist = wrist
        self.torch = __import__("torch")

    def test_desired_jaw_axis_is_across_both(self):
        torch, wrist = self.torch, self.wrist
        approach = torch.tensor([[1.0, 0.0, 0.0], [0.7, 0.7, 0.0]])
        obj = torch.tensor([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
        jaw = wrist.desired_jaw_axis(wrist.unit(approach), obj)
        self.assertTrue(torch.allclose((jaw * approach).sum(-1), torch.zeros(2), atol=1e-6))
        self.assertTrue(torch.allclose((jaw * obj).sum(-1), torch.zeros(2), atol=1e-6))
        self.assertTrue(torch.allclose(jaw.norm(dim=-1), torch.ones(2), atol=1e-6))

    def test_desired_jaw_axis_survives_a_parallel_object(self):
        torch, wrist = self.torch, self.wrist
        approach = torch.tensor([[0.0, 0.0, 1.0]])
        jaw = wrist.desired_jaw_axis(approach, torch.tensor([[0.0, 0.0, 1.0]]))
        self.assertTrue(torch.isfinite(jaw).all())
        self.assertAlmostEqual(float(jaw.norm()), 1.0, places=5)

    def test_the_jaw_axis_is_an_axis_not_a_direction(self):
        """A desired axis pointing the other way must not ask for a 180 degree roll."""
        torch, wrist = self.torch, self.wrist
        jaw_cur = torch.tensor([[0.0, 1.0, 0.0]])
        approach = torch.tensor([[1.0, 0.0, 0.0]])
        near = wrist.orientation_error(approach, jaw_cur, torch.tensor([[0.0, 1.0, 0.0]]))
        flipped = wrist.orientation_error(approach, jaw_cur, torch.tensor([[0.0, -1.0, 0.0]]))
        self.assertTrue(torch.allclose(near, flipped, atol=1e-6))
        self.assertLess(float(near.norm()), 1e-6)

    def test_a_correction_about_an_available_axis_is_solved_exactly(self):
        torch, wrist = self.torch, self.wrist
        error = torch.tensor([[0.0, 0.0, 0.3]])
        axes = torch.tensor([[[0.0], [0.0], [1.0]]])          # one joint, about z
        steps = wrist.joint_correction(error, axes, gain=1.0, damping=0.0)
        self.assertAlmostEqual(float(steps[0, 0]), 0.3, places=5)

    def test_a_correction_the_axes_cannot_make_stays_bounded(self):
        torch, wrist = self.torch, self.wrist
        error = torch.tensor([[0.5, 0.0, 0.0]])
        axes = torch.tensor([[[0.0], [0.0], [1.0]]])          # cannot roll about x
        steps = wrist.joint_correction(error, axes)
        self.assertLess(abs(float(steps[0, 0])), 1e-6)

    def test_actions_respect_the_joint_limits(self):
        torch, wrist = self.torch, self.wrist
        limits = torch.tensor([[-2.35, 2.35]])
        current = torch.tensor([[2.3]])
        actions = wrist.joint_actions(current, torch.tensor([[0.6]]), limits, 0.25)
        self.assertAlmostEqual(float(actions[0, 0]), 2.35 / 0.25, places=5)

    def test_the_step_is_bounded(self):
        torch, wrist = self.torch, self.wrist
        error = torch.tensor([[0.0, 0.0, 50.0]])
        axes = torch.tensor([[[0.0], [0.0], [1.0]]])
        steps = wrist.joint_correction(error, axes)
        self.assertLessEqual(float(steps.abs().max()), wrist.MAX_STEP_RAD + 1e-6)


if __name__ == "__main__":
    unittest.main()


class TheDemosControlThePointTheirPolicyWasTrainedOn(unittest.TestCase):
    """The demo config inherits the training config, and the training task moved its tool point.

    These demos run the released checkpoint, which was trained to put the Link7_1 fingertip on the
    commanded goal. Inheriting the new jaw-centre default would move the commanded point 2.4 cm for
    a policy that knows nothing about it, and would do it silently -- the demos would still run,
    still report, and quietly mean something else than the numbers already on the record.
    """

    def test_the_demo_pins_the_fingertip(self):
        try:
            from unifp_isaaclab import interface
        except ImportError:  # pragma: no cover - torch is absent on the system Python
            self.skipTest("unifp_isaaclab needs torch")
        import ast
        import pathlib

        source = pathlib.Path(__file__).resolve().parents[1] / "env.py"
        tree = ast.parse(source.read_text())
        pinned = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "UniFPDemoEnvCfg":
                for item in node.body:
                    if isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name):
                        if item.target.id in ("tool_body", "tool_offset_m"):
                            pinned.add(item.target.id)
        self.assertEqual(pinned, {"tool_body", "tool_offset_m"},
                         "UniFPDemoEnvCfg must pin the tool point, not inherit the task's")

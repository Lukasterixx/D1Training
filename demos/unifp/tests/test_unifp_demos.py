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
        self.assertAlmostEqual(grasp[2], props.POST_TOP_M + GEOMETRY.spindle[2])
        # Between the spindle and the end of the lever.
        self.assertLess(props.LEVER_GRASP_OFFSET_M, GEOMETRY.handle_span)
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


class SideStance(unittest.TestCase):
    """`props.side_stance`: the robot off the door's normal, the reach unchanged."""

    @staticmethod
    def toward_robot(before, after):
        """How much of the grasp point's motion from `before` to `after` comes horizontally toward the robot."""
        step = unit(tuple(b - a for a, b in zip(before, after)))
        back = unit((-before[0], -before[1], 0.0))
        return dot(step, back)

    def test_the_grasp_point_stays_where_it_was(self):
        for seed in range(20):
            site = props.sample_box_site(random.Random(seed))
            for stance in (-30.0, 15.0, 45.0):
                turned = props.side_stance(site, stance)
                for a, b in zip(site.grasp_point_m(), turned.grasp_point_m()):
                    self.assertAlmostEqual(a, b, places=9)
                self.assertAlmostEqual(turned.yaw_deg, site.yaw_deg + GEOMETRY.hinge_side * stance)
        self.assertIs(props.side_stance(site, 0.0), site)

    def test_the_latch_side_turns_the_lever_toward_the_robot(self):
        site = props.BoxSite(distance_m=0.66, bearing_deg=0.0, yaw_deg=180.0)
        pull = {stance: self.toward_robot(props.side_stance(site, stance).grasp_point_m(45.0),
                                          props.side_stance(site, stance).grasp_point_m(46.0))
                for stance in (-30.0, 0.0, 30.0)}
        self.assertLess(pull[-30.0], -0.2)
        self.assertLess(abs(pull[0.0]), 0.15)
        self.assertGreater(pull[30.0], 0.2)

    def test_the_hinge_side_brings_the_door_toward_the_robot(self):
        site = props.BoxSite(distance_m=0.66, bearing_deg=0.0, yaw_deg=180.0)
        pull = {stance: self.toward_robot(props.side_stance(site, stance).grasp_point_m(60.0, 40.0),
                                          props.side_stance(site, stance).grasp_point_m(60.0, 41.0))
                for stance in (-30.0, 0.0, 30.0)}
        # 0.82 on the real box (0.9+ on the earlier proxy, whose handle sat nearer its hinge).
        self.assertGreater(pull[-30.0], 0.75)
        self.assertGreater(pull[-30.0], pull[0.0])
        self.assertGreater(pull[0.0], pull[30.0])

    def test_stances_the_demo_measures_keep_the_goals_in_the_sphere(self):
        import dataclasses
        saved = props.LEVER_GRASP_OFFSET_M
        props.LEVER_GRASP_OFFSET_M = 0.080          # the lip claw's hook, as `--mech` sets it
        try:
            timing = dataclasses.replace(script.CombinerTiming(), turn_deg=60.0, ease_deg=60.0)
            sweep = lambda stance: CommandedGoalsStayInsideTheTrainedWorkspace._sweep(
                self, script.combiner_phases(timing), "box_site",
                lambda rng: props.side_stance(props.sample_box_site(rng), stance), seeds=40)
            for stance in (-15.0, 15.0, 30.0):
                bad = sweep(stance)
                self.assertEqual(bad[:3], [], f"stance {stance}: {len(bad)} goals outside the trained sphere")
            # The real box (2026-10-19): 45 deg on the latch side, which the earlier proxy kept inside, now carries
            # the door's pull out to a 0.61 m radius, past the trained 0.58. Pinned so a change that fixes or
            # widens it is noticed.
            outside = sweep(45.0)
            self.assertTrue(outside)
            self.assertTrue(all(phase in ("pull", "release", "back") for _, phase, _, _ in outside))
        finally:
            props.LEVER_GRASP_OFFSET_M = saved


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


class DoorPull(unittest.TestCase):
    """`script.door_phases`: grip the unlocked box's handle and walk the door open round the hinge."""

    #: The Go2's footprint about its base origin, legs included (0.70 x 0.31 m), and the clearance asked of it.
    HALF_LENGTH_M, HALF_WIDTH_M, CLEARANCE_M = 0.35, 0.16, 0.10

    @staticmethod
    def in_base(pose, point):
        """A spawn-frame point in a base pose's own yaw frame (z unchanged)."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        x, y = point[0] - pose[0], point[1] - pose[1]
        return (c * x + s * y, -s * x + c * y, point[2])

    def footprint_gap(self, pose, point):
        """How far a spawn-frame point is outside the robot's footprint at `pose` (negative inside)."""
        x, y, _ = self.in_base(pose, (point[0], point[1], 0.0))
        dx, dy = abs(x) - self.HALF_LENGTH_M, abs(y) - self.HALF_WIDTH_M
        return math.hypot(max(dx, 0.0), max(dy, 0.0)) if dx > 0 or dy > 0 else max(dx, dy)

    def test_the_base_turns_round_the_hinge_with_the_door(self):
        for seed in range(20):
            site = props.sample_door_site(random.Random(seed))
            hinge = site.hinge_xy_m()
            start = site.base_pose_m(0.0)
            self.assertEqual(start, (0.0, 0.0, 0.0))
            radius = math.hypot(hinge[0], hinge[1])
            for door in (0.0, 30.0, 60.0, 90.0):
                pose = site.base_pose_m(door)
                self.assertAlmostEqual(math.hypot(pose[0] - hinge[0], pose[1] - hinge[1]), radius, places=9)
                self.assertAlmostEqual(math.degrees(pose[2]), GEOMETRY.hinge_axis_z * door, places=9)
                # The handle does not move in the robot's frame: the grasp the reach made is the grasp it keeps.
                for a, b in zip(self.in_base(pose, site.grasp_point_m(0.0, door)), site.grasp_point_m(0.0, 0.0)):
                    self.assertAlmostEqual(a, b, places=9)

    def test_goals_stay_in_the_trained_sphere_of_the_walking_base(self):
        bad = []
        for seed in range(40):
            state = {"box_site": props.sample_door_site(random.Random(seed))}
            demo = script.DemoScript(script.door_phases())
            demo.reset(script.HOME_POINT, script.JAW_WIDE_M)
            while not demo.finished:
                command = demo.update(0.02, state)
                pose = command.base_pose or (0.0, 0.0, 0.0)
                local = self.in_base(pose, command.point)
                if not in_workspace(local):
                    bad.append((seed, command.phase, sphere(local)))
        self.assertEqual(bad[:3], [], f"{len(bad)} door-pull goals outside the trained sphere")

    def test_the_walk_is_commanded_outside_the_dead_zones_and_inside_the_trained_ranges(self):
        state = {"box_site": props.sample_door_site(random.Random(2))}
        demo = script.DemoScript(script.door_phases())
        demo.reset(script.HOME_POINT, script.JAW_WIDE_M)
        previous, walking = None, []
        while not demo.finished:
            command = demo.update(0.02, state)
            if command.phase == "pull" and previous is not None:
                walking.append(script.base_velocity_command(command.base_pose, previous, command.base_pose, 0.02))
            previous = command.base_pose
        self.assertGreater(len(walking), 300)
        for vx, vy, wz in walking:
            # UniFP's dead zones (interface: 0.1 m/s, 0.1 m/s, 0.2 rad/s): inside all three it is told to stand.
            self.assertTrue(abs(vx) > 0.1 or abs(vy) > 0.1 or abs(wz) > 0.2)
            self.assertLessEqual(abs(vx), 0.6)
            self.assertLessEqual(abs(vy), 0.4)
            self.assertLessEqual(abs(wz), 0.6)
            # Turning left round a hinge on the right: stepping right, turning left.
            self.assertLess(vy, 0.0)
            self.assertGreater(wz, 0.0)

    def test_the_robot_keeps_clear_of_the_door_and_the_box(self):
        g = GEOMETRY
        front = g.door_x + g.door_thickness / 2
        outline = [(x, y, 0.0) for x in (-g.depth / 2, front) for y in [v / 10 * g.width - g.width / 2 for v in range(11)]]
        outline += [(-g.depth / 2 + v / 10 * (front + g.depth / 2), y, 0.0) for v in range(11) for y in (-g.width / 2, g.width / 2)]
        for seed in range(20):
            site = props.sample_door_site(random.Random(seed))
            for door in range(0, 91, 5):
                pose = site.base_pose_m(float(door))
                angle = math.radians(g.hinge_axis_z * door)
                edge = (0.0, -g.hinge_side * g.width)
                door_points = []
                for t in [v / 20 for v in range(21)]:
                    ex, ey = t * edge[0], t * edge[1]
                    door_points.append(site.world((g.hinge[0] + math.cos(angle) * ex - math.sin(angle) * ey,
                                                   g.hinge[1] + math.sin(angle) * ex + math.cos(angle) * ey, 0.0)))
                box_points = [site.world(point) for point in outline]
                gap = min(self.footprint_gap(pose, point) for point in door_points + box_points)
                self.assertGreater(gap, self.CLEARANCE_M, f"seed {seed}, door {door} deg: {gap:.3f} m")

    def test_feedback_turns_the_pose_error_into_the_base_frame_and_clips(self):
        # No path motion; the base 0.1 m behind its goal, facing +y: forward in the world is its right.
        vx, vy, wz = script.base_velocity_command((0.1, 0.0, math.pi / 2), (0.1, 0.0, math.pi / 2),
                                                  (0.0, 0.0, math.pi / 2), 0.02)
        self.assertAlmostEqual(vx, 0.0, places=9)
        self.assertAlmostEqual(vy, -0.15, places=9)
        self.assertAlmostEqual(wz, 0.0, places=9)
        vx, vy, wz = script.base_velocity_command((5.0, 5.0, 3.0), None, (0.0, 0.0, 0.0), 0.02)
        self.assertEqual((vx, vy, wz), script.BASE_COMMAND_LIMITS)
        # Wrapped: a yaw error just past pi is a short turn the other way.
        _, _, wz = script.base_velocity_command((0.0, 0.0, math.pi - 0.05), None, (0.0, 0.0, -math.pi + 0.05), 0.02)
        self.assertLess(wz, 0.0)

    def test_the_phase_clock_and_the_end_pose(self):
        timing = script.DoorTiming()
        phases = script.door_phases(timing)
        self.assertEqual([p.name for p in phases],
                         ["stand", "approach", "settle", "enter", "arrive", "grip", "pull", "hold", "release", "back"])
        self.assertTrue(all(p.base is None for p in phases[:6]))
        self.assertTrue(all(p.base is not None for p in phases[6:]))
        site = props.sample_door_site(random.Random(0))
        # Back to the ready pose in front of the robot where it ends, which has turned 90 degrees.
        back = phases[-1].target({"box_site": site})
        local = self.in_base(site.base_pose_m(timing.door_deg), back)
        for a, b in zip(local, script.HOME_POINT):
            self.assertAlmostEqual(a, b, places=9)


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

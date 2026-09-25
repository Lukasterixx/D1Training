"""The roll objective and the moved controlled point: the parts that can be checked on a CPU.

Three things this guards, each of which would otherwise fail silently rather than loudly:

  * **The port's fidelity record stays separate from our additions.** `task_cfg.REWARD_WEIGHTS` is
    asserted elsewhere to equal, term for term, the reward set of a recorded Isaac Gym rollout. If
    the roll term ever leaks into it, that assertion stops meaning "this port reproduces upstream".
  * **The roll angle means what the docstring says**, including at the two places angle code goes
    wrong: the half-turn wrap (a jaw axis is an axis, so +90 and -90 degrees are one grasp) and the
    degenerate frame (a hand pointing straight down has no horizontal reference).
  * **The controlled point is on the roll axis and does not move with the jaws**, which is the
    entire reason for moving it (F-094, F-096).
"""
import math
import unittest

import numpy as np
import torch

from unifp_isaaclab import interface, task
from unifp_train import rewards, task_cfg


def state(approach, jaw, forward=(1.0, 0.0, 0.0), commanded_roll=0.0):
    """A `TaskState` carrying only the fields the roll term reads."""
    blank = rewards.TaskState.__new__(rewards.TaskState)
    unit = lambda v: torch.tensor([v], dtype=torch.float) / np.linalg.norm(v)
    blank.ee_approach_w = unit(approach)
    blank.ee_jaw_w = unit(jaw)
    blank.base_forward_w = unit(forward)
    commands = torch.zeros(1, interface.NUM_COMMANDS)
    commands[:, interface.CMD_EE_ORN_R:interface.CMD_EE_ORN_P + 1] = rewards.encode_roll(
        torch.tensor([commanded_roll], dtype=torch.float))
    blank.commands = commands
    return blank


class TheExtensionStaysOutOfThePortsRecord(unittest.TestCase):
    def test_roll_is_not_in_the_upstream_weights(self):
        self.assertNotIn("tracking_ee_orn_roll", task_cfg.REWARD_WEIGHTS)
        self.assertNotIn("tracking_ee_orn_roll", rewards.TERMS)
        self.assertEqual(len(task_cfg.REWARD_WEIGHTS), 27)

    def test_the_extension_is_declared_and_implemented(self):
        self.assertEqual(set(task_cfg.EXTENSION_WEIGHTS), set(rewards.EXTENSION_TERMS))
        for name, weight in task_cfg.scaled_extension_weights().items():
            self.assertAlmostEqual(weight, task_cfg.EXTENSION_WEIGHTS[name] * interface.POLICY_DT)

    def test_upstreams_dead_names_are_not_revived(self):
        """`tracking_ee_orn` and `tracking_ee_orn_ry` are declared upstream and never implemented.

        Adopting either name would suggest this term is upstream's, which it is not, and would
        invite someone to set a weight on a function that does not exist.
        """
        for dead in ("tracking_ee_orn", "tracking_ee_orn_ry"):
            self.assertNotIn(dead, task_cfg.EXTENSION_WEIGHTS)
            self.assertNotIn(dead, rewards.EXTENSION_TERMS)


class TheRollAngle(unittest.TestCase):
    def test_level_jaws_read_zero(self):
        self.assertAlmostEqual(float(rewards.tool_roll(state((1, 0, 0), (0, 1, 0)))), 0.0, places=5)

    def test_a_jaw_axis_is_an_axis(self):
        """Flipping the jaw axis is the same grasp and must read the same angle."""
        for approach, jaw in (((1, 0, 0), (0, 1, 0)), ((1, 0, 0), (0, 0.7, 0.7)),
                              ((0.6, 0.8, 0), (0, 0, 1))):
            flipped = tuple(-v for v in jaw)
            self.assertAlmostEqual(float(rewards.tool_roll(state(approach, jaw))),
                                   float(rewards.tool_roll(state(approach, flipped))), places=5)

    def test_upright_jaws_read_a_quarter_turn(self):
        angle = abs(float(rewards.tool_roll(state((1, 0, 0), (0, 0, 1)))))
        self.assertAlmostEqual(angle, math.pi / 2, places=5)

    def test_the_angle_stays_inside_a_half_turn(self):
        rng = np.random.default_rng(0)
        for _ in range(200):
            approach = rng.normal(size=3)
            jaw = np.cross(approach, rng.normal(size=3))   # any axis perpendicular to it
            if np.linalg.norm(jaw) < 1e-6:
                continue
            angle = float(rewards.tool_roll(state(tuple(approach), tuple(jaw))))
            self.assertGreaterEqual(angle, -math.pi / 2 - 1e-6)
            self.assertLessEqual(angle, math.pi / 2 + 1e-6)

    def test_a_hand_pointing_straight_down_still_has_a_roll(self):
        """The horizontal reference does not exist there; the base's heading stands in for it."""
        angle = rewards.tool_roll(state((0, 0, -1), (0, 1, 0)))
        self.assertTrue(torch.isfinite(angle).all())

    def test_the_error_wraps_the_short_way(self):
        """Commanding +90 degrees and measuring -90 is zero error, not a half turn of it."""
        reward = rewards.tracking_ee_orn_roll(
            state((1, 0, 0), (0, 0, 1), commanded_roll=math.pi / 2))
        self.assertAlmostEqual(float(reward), 1.0, places=5)

    def test_the_reward_falls_off_with_error(self):
        on_target = float(rewards.tracking_ee_orn_roll(state((1, 0, 0), (0, 1, 0))))
        off = float(rewards.tracking_ee_orn_roll(
            state((1, 0, 0), (0, math.cos(0.5), math.sin(0.5)))))
        worst = float(rewards.tracking_ee_orn_roll(state((1, 0, 0), (0, 0, 1))))
        self.assertAlmostEqual(on_target, 1.0, places=5)
        self.assertLess(off, on_target)
        self.assertLess(worst, off)
        self.assertGreater(worst, 0.0)


class TheCommandedRoll(unittest.TestCase):
    def test_the_generator_is_silent_unless_asked(self):
        """A run without the objective must behave exactly as upstream's generator does."""
        goals = task.EeGoalTrajectory(4, device="cpu")
        for _ in range(400):
            goals.step()
        self.assertEqual(float(goals.current_roll.abs().max()), 0.0)

    def test_the_commanded_roll_stays_in_range(self):
        goals = task.EeGoalTrajectory(8, device="cpu", roll_range=task_cfg.EE_ROLL_RANGE_RAD)
        low, high = task_cfg.EE_ROLL_RANGE_RAD
        for _ in range(2000):
            goals.step()
            self.assertGreaterEqual(float(goals.current_roll.min()), low - 1e-5)
            self.assertLessEqual(float(goals.current_roll.max()), high + 1e-5)

    def test_what_the_policy_sees_is_continuous(self):
        """The angle wraps; the encoding the observation carries must not.

        This is the check that would have caught feeding the raw angle: a command sliding past
        90 degrees jumps to -90, and the policy sees a step change in what it is asked for while
        the hand is meant to keep turning smoothly.
        """
        goals = task.EeGoalTrajectory(8, device="cpu", roll_range=task_cfg.EE_ROLL_RANGE_RAD)
        previous = rewards.encode_roll(goals.current_roll)
        biggest = 0.0
        for _ in range(4000):
            goals.step()
            encoded = rewards.encode_roll(goals.current_roll)
            biggest = max(biggest, float((encoded - previous).abs().max()))
            previous = encoded
        # The fastest legitimate slide is a half turn over the shortest trajectory time, and the
        # encoding moves at twice the angle's rate.
        fastest = 2.0 * (math.pi / 2) / (min(interface.EE_GOAL_TRAJ_TIME_S) / interface.POLICY_DT)
        self.assertLess(biggest, fastest * 1.5)

    def test_the_encoding_round_trips(self):
        angles = torch.linspace(-math.pi / 2, math.pi / 2, 101)
        commands = torch.zeros(len(angles), interface.NUM_COMMANDS)
        commands[:, interface.CMD_EE_ORN_R:interface.CMD_EE_ORN_P + 1] = rewards.encode_roll(angles)
        decoded = rewards.commanded_roll(commands)
        self.assertLess(float(rewards.wrap_half_turn(decoded - angles).abs().max()), 1e-5)

    def test_the_roll_takes_the_short_way_round(self):
        """+80 to -80 degrees is a 20 degree turn for an axis, not a 160 degree one."""
        goals = task.EeGoalTrajectory(1, device="cpu", roll_range=task_cfg.EE_ROLL_RANGE_RAD)
        goals.start_roll[:] = math.radians(80.0)
        goals.goal_roll[:] = math.radians(-80.0)
        goals.timer[:] = 0.0
        goals.traj_steps[:] = 50.0
        goals.total_steps[:] = 1000.0
        # Accumulated through the wrap: the stored angle is folded into a half turn, so a raw
        # difference would count crossing the fold as 180 degrees of travel that never happened.
        travelled = 0.0
        previous = goals.start_roll.clone()
        for _ in range(50):
            goals.step()
            travelled += abs(float(rewards.wrap_half_turn(goals.current_roll - previous)))
            previous = goals.current_roll.clone()
        self.assertLess(math.degrees(travelled), 40.0)


class TheControlledPoint(unittest.TestCase):
    #: Link6-frame geometry from `d1_arm/d1.urdf`; `Joint6` rolls about Link6's own z.
    FINGER_ORIGINS = ((-0.0056012, -0.029636, 0.0706), (-0.0056388, 0.029640, 0.0706))

    def test_the_new_point_is_on_the_roll_axis(self):
        x, y, _ = task_cfg.TOOL_OFFSET_M
        self.assertAlmostEqual(math.hypot(x, y), 0.0, places=9,
                               msg="a point off the roll axis makes roll cost position")

    def test_the_old_point_was_not(self):
        x, y, _ = interface.TOOL_OFFSET_M
        self.assertGreater(math.hypot(x, y), 0.005)

    def test_the_new_point_sits_between_the_pads(self):
        from demos.cup.pick_demo.grasp import (FINGER_INNER_Y_M, FINGER_SHOULDER_Z_M,
                                               FINGER_X_RANGE_M, FINGERTIP_Z_M, JAW_CENTRE_LINK6)
        x, y, z = task_cfg.TOOL_OFFSET_M
        for ours, theirs in zip(task_cfg.TOOL_OFFSET_M, JAW_CENTRE_LINK6):
            self.assertAlmostEqual(ours, theirs, places=9)
        self.assertTrue(FINGER_X_RANGE_M[0] < x < FINGER_X_RANGE_M[1])
        self.assertLess(abs(y), FINGER_INNER_Y_M)
        self.assertTrue(FINGER_SHOULDER_Z_M < z < FINGERTIP_Z_M)

    def test_the_new_point_does_not_move_when_the_jaws_do(self):
        """The two fingers take opposite joint coordinates, so their midpoint is invariant."""
        left, right = (np.array(o) for o in self.FINGER_ORIGINS)
        slide = np.array([-0.0006, -1.0, 0.0])          # both slide along this per unit coordinate
        for travel in (0.0, 0.012, 0.03):
            midpoint = 0.5 * ((left + slide * travel) + (right - slide * travel))
            np.testing.assert_allclose(midpoint[:2], 0.5 * (left + right)[:2], atol=1e-9)

    def test_the_old_point_did(self):
        tip_shut = np.array([0.0004, -0.0126, 0.1253])   # measured from the URDF, jaws shut
        tip_open = tip_shut + np.array([-0.0006, -1.0, 0.0]) * 0.03
        self.assertGreater(float(np.linalg.norm(tip_open - tip_shut)), 0.029)

    def test_the_interface_still_records_what_the_checkpoints_trained_on(self):
        """`interface` is the released contract and must not follow the task's new choice."""
        self.assertEqual(interface.TOOL_BODY, "Link7_1")
        self.assertNotEqual(task_cfg.TOOL_BODY, interface.TOOL_BODY)


if __name__ == "__main__":
    unittest.main()


class ResumingARun(unittest.TestCase):
    """F-082: the force curriculum lives on the environment and restarts at every launch.

    `launch_training.py` fixes this for the Isaac Gym side; `run_unifp_train.py` never did, and a
    resume past the gate would quietly train position-only again for another 8,000 iterations. The
    arithmetic is checkable here even though the effect is not.
    """

    def _runner(self, iteration):
        return type("Runner", (), {"current_learning_iteration": iteration})()

    def test_the_iteration_comes_from_the_runner(self):
        import run_unifp_train

        self.assertEqual(run_unifp_train.resumed_iteration(self._runner(20000), "model_1.pt"),
                         20000)

    def test_it_falls_back_to_the_filename(self):
        import run_unifp_train

        self.assertEqual(
            run_unifp_train.resumed_iteration(self._runner(0), "/logs/x/model_11000.pt"), 11000)
        self.assertEqual(run_unifp_train.resumed_iteration(object(), "model_800.pt"), 800)

    def test_an_unreadable_name_resumes_at_zero_rather_than_guessing(self):
        import run_unifp_train

        self.assertEqual(run_unifp_train.resumed_iteration(object(), "latest.pt"), 0)

    def test_the_restored_counter_puts_a_resume_on_the_right_side_of_the_gate(self):
        gate = task_cfg.FORCE_START_ITERATION * task_cfg.NUM_STEPS_PER_ENV
        before = 7000 * task_cfg.NUM_STEPS_PER_ENV
        after = 11000 * task_cfg.NUM_STEPS_PER_ENV
        self.assertLess(before, gate, "a resume below the gate must still be position-only")
        self.assertGreater(after, gate, "a resume above it must have forces from the first step")

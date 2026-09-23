"""The Isaac Lab port of the UniFP policy, checked against the stack that trained it.

`tests/data/unifp_interface_48800.npz` is 128 policy steps recorded out of the running Isaac Gym
environment by `unifp_go2d1/dump_interface.py`, for `Sep18_11-10-15_/model_48800.pt`: the raw
state each step, the 76-wide observation UniFP built from it, and the 18 actions the policy
produced. That makes the two halves of the port separately checkable with no simulator running:

  * `interface.py` describes the contract correctly -- same constants, same observation, from the
    same raw state; and
  * `policy.py` rebuilds the network correctly -- same actions, from the same observation.

A failure here means the port no longer matches the checkpoint, which is a different and much
more serious thing than the two simulators disagreeing about physics.

The policy tests need the checkpoint itself (24 MB, not in the repository) and skip without it.
Everything else runs from the fixture alone.
"""
from __future__ import annotations

import os
from pathlib import Path
import unittest

import numpy as np
import torch

from unifp_isaaclab import interface
from unifp_isaaclab.policy import ObsHistory, UniFPPolicy

FIXTURE = Path(__file__).resolve().parent / "data/unifp_interface_48800.npz"
CHECKPOINT = Path(os.environ.get(
    "UNIFP_CHECKPOINT",
    os.path.expanduser("~/thesis_b_legacy/UniFP/logs/go2d1_pos_force/Sep18_11-10-15_/model_48800.pt")))

#: float32 rounding. The recorded values came off a GPU and are rebuilt here on the CPU, so
#: "identical" means identical to the last bit or two, not bitwise.
TOLERANCE = 1e-5


def load():
    return np.load(FIXTURE, allow_pickle=True)


class TestConstants(unittest.TestCase):
    """What `interface.py` asserts about the environment, against what the environment had.

    These are not redundant with reading the config: several are derived there (the torque limits
    come from the URDF, the gains from name-keyed dicts resolved per joint, the DOF order from
    Isaac Gym's own asset sort), so the config cannot be trusted to state them.
    """

    def setUp(self):
        self.data = load()

    def test_dof_order(self):
        self.assertEqual(list(self.data["dof_names"]), list(interface.DOF_NAMES))

    def test_isaaclab_names_drop_the_d1_prefix(self):
        # The prefix exists only to force Isaac Gym's DOF sort; Isaac Lab's model has the D1
        # URDF's own names. Getting this wrong makes `permutation_from` raise, which is the
        # intended failure -- but check the translation itself too.
        self.assertEqual(interface.ISAACLAB_NAMES[12], "Joint1")
        self.assertEqual(interface.ISAACLAB_NAMES[0], "FL_hip_joint")
        self.assertEqual(len(set(interface.ISAACLAB_NAMES)), interface.NUM_DOF)

    def test_control_constants(self):
        for name, recorded, ours in (
            ("default_dof_pos", self.data["default_dof_pos"], interface.DEFAULT_DOF_POS),
            ("p_gains", self.data["p_gains"], interface.P_GAINS),
            ("d_gains", self.data["d_gains"], interface.D_GAINS),
            ("torque_limits", self.data["torque_limits"], interface.TORQUE_LIMITS),
            ("commands_scale", self.data["commands_scale"], interface.COMMANDS_SCALE),
        ):
            with self.subTest(constant=name):
                np.testing.assert_allclose(recorded, np.asarray(ours), atol=1e-5)

    def test_timing(self):
        self.assertAlmostEqual(float(self.data["dt"]), interface.POLICY_DT, places=6)
        self.assertAlmostEqual(float(self.data["sim_dt"]), interface.SIM_DT, places=6)
        self.assertEqual(int(self.data["decimation"]), interface.DECIMATION)
        self.assertAlmostEqual(float(self.data["cycle_time"]), interface.CYCLE_TIME, places=6)
        self.assertAlmostEqual(float(self.data["action_scale"]), interface.ACTION_SCALE, places=6)

    def test_shapes(self):
        self.assertEqual(int(self.data["num_single_obs"]), interface.NUM_SINGLE_OBS)
        self.assertEqual(int(self.data["frame_stack"]), interface.FRAME_STACK)
        self.assertEqual(int(self.data["num_actions"]), interface.NUM_ACTIONS)
        self.assertEqual(self.data["obs_single"].shape[1], interface.NUM_SINGLE_OBS)

    def test_motor_strength_is_off_in_playback(self):
        # The port applies no motor-strength randomisation, so `joint_targets` is UniFP's torque
        # law exactly. That holds only while the recorded run had it disabled.
        np.testing.assert_allclose(self.data["motor_strength"], 1.0, atol=1e-6)

    def test_goal_sphere_centre(self):
        np.testing.assert_allclose(self.data["ee_goal_center_offset"],
                                   np.asarray(interface.EE_GOAL_CENTER_OFFSET), atol=1e-6)


class TestObservation(unittest.TestCase):
    """The observation builder, against UniFP's, from the same raw state."""

    def setUp(self):
        self.data = load()
        quat_xyzw = self.data["base_quat"]
        # Isaac Gym stores quaternions w-last; Isaac Lab (and `interface`) w-first.
        self.quat_wxyz = torch.as_tensor(
            np.concatenate([quat_xyzw[:, 3:4], quat_xyzw[:, 0:3]], axis=1), dtype=torch.float32)
        self.built = interface.single_obs(
            self.quat_wxyz,
            torch.as_tensor(self.data["base_ang_vel"]),
            torch.as_tensor(self.data["dof_pos"]),
            torch.as_tensor(self.data["dof_vel"]),
            torch.as_tensor(self.data["prev_actions"]),
            torch.as_tensor(self.data["gait_indices"]),
            torch.as_tensor(self.data["commands"]),
        )
        self.recorded = torch.as_tensor(self.data["obs_single"])

    def test_whole_observation(self):
        self.assertLess(float((self.built - self.recorded).abs().max()), TOLERANCE)

    def test_each_block(self):
        # Reported per block, so a failure names the term that drifted rather than "the
        # observation changed".
        for name, (start, end) in interface.OBS_SLICES.items():
            with self.subTest(block=name):
                worst = float((self.built[:, start:end] - self.recorded[:, start:end]).abs().max())
                self.assertLess(worst, TOLERANCE, f"{name} differs by {worst}")

    def test_roll_pitch_matches_the_gym_euler_convention(self):
        roll_pitch = interface.body_roll_pitch(self.quat_wxyz)
        self.assertLess(float((roll_pitch - self.recorded[:, 0:2]).abs().max()), TOLERANCE)

    def test_permutation_round_trips_isaac_lab_ordering(self):
        # Isaac Lab groups joints by level (all hips, then all thighs); UniFP orders leg by leg.
        # A permutation built from that ordering must put the names back into UniFP's order.
        isaaclab_order = ["FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                          "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
                          "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
                          "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6",
                          "Joint7_1", "Joint7_2"]
        gather = interface.permutation_from(isaaclab_order)
        reordered = [isaaclab_order[i] for i in gather]
        self.assertEqual(reordered, list(interface.ISAACLAB_NAMES))

    def test_permutation_rejects_a_missing_joint(self):
        with self.assertRaises(KeyError):
            interface.permutation_from(["FL_hip_joint"] * interface.NUM_DOF)

    def test_gait_phase_is_pinned_while_standing(self):
        standing = torch.zeros(1, interface.NUM_COMMANDS)
        phase = interface.gait_step(torch.tensor([0.4]), standing)
        self.assertEqual(float(phase[0]), 0.0)

    def test_gait_phase_advances_while_walking(self):
        walking = torch.zeros(1, interface.NUM_COMMANDS)
        walking[0, interface.CMD_LIN_VEL_X] = 0.5
        phase = interface.gait_step(torch.tensor([0.0]), walking)
        self.assertAlmostEqual(float(phase[0]), interface.POLICY_DT / interface.CYCLE_TIME, places=6)


class TestHistory(unittest.TestCase):
    """The 32-frame ring, against the environment's own stacking of the same frames."""

    def test_stack_matches_the_environment(self):
        data = load()
        frames = torch.as_tensor(data["obs_single"])
        history = ObsHistory(1)
        stacked = None
        for index in range(len(frames)):
            stacked = history.append(frames[index:index + 1])
        at = int(data["obs_stacked_at"])
        self.assertEqual(at, len(frames) - 1, "fixture records the stack at its last step")
        recorded = torch.as_tensor(data["obs_stacked"]).unsqueeze(0)
        self.assertLess(float((stacked - recorded).abs().max()), TOLERANCE)

    def test_newest_frame_is_last(self):
        history = ObsHistory(1)
        history.append(torch.full((1, interface.NUM_SINGLE_OBS), 1.0))
        stacked = history.append(torch.full((1, interface.NUM_SINGLE_OBS), 2.0))
        self.assertEqual(float(stacked[0, -1]), 2.0)
        self.assertEqual(float(stacked[0, -interface.NUM_SINGLE_OBS - 1]), 1.0)
        self.assertEqual(float(stacked[0, 0]), 0.0)

    def test_reset_zeroes_the_ring(self):
        history = ObsHistory(2)
        history.append(torch.ones(2, interface.NUM_SINGLE_OBS))
        history.reset()
        self.assertEqual(float(history.buffer.abs().max()), 0.0)

    def test_reset_does_not_alter_an_observation_already_returned(self):
        """A reset must not reach back into a stack the caller is still holding.

        `DirectRLEnv` resets terminated environments after the policy has acted on the
        observation but before the rollout storage copies it, so an in-place reset blanks a
        sample that has already been used -- which is what pinned this port's learning-rate
        schedule to its floor for 26,000 iterations (week 2 log, 2026-09-21).
        """
        history = ObsHistory(4)
        stacked = history.append(torch.ones(4, interface.NUM_SINGLE_OBS))
        before = stacked.clone()
        history.reset(torch.tensor([1, 3]))
        self.assertTrue(torch.equal(stacked, before),
                        "resetting environments 1 and 3 modified a stack already handed out")
        # and the reset still did its job for the next caller
        self.assertEqual(float(history.buffer[1].abs().max()), 0.0)
        self.assertEqual(float(history.buffer[0].abs().max()), 1.0)

    def test_reset_all_does_not_alter_an_observation_already_returned(self):
        history = ObsHistory(2)
        stacked = history.append(torch.ones(2, interface.NUM_SINGLE_OBS))
        before = stacked.clone()
        history.reset()
        self.assertTrue(torch.equal(stacked, before),
                        "a full reset modified a stack already handed out")


@unittest.skipUnless(CHECKPOINT.is_file(), f"checkpoint not present at {CHECKPOINT}")
class TestPolicy(unittest.TestCase):
    """The rebuilt network, against the actions UniFP's own runner produced."""

    @classmethod
    def setUpClass(cls):
        cls.data = load()
        cls.policy = UniFPPolicy(CHECKPOINT)

    def test_iteration_comes_from_the_filename(self):
        # The `iter` field inside the payload is 0 in these runs, so the filename is the only
        # honest source -- and the fixture records which checkpoint produced it.
        self.assertEqual(self.policy.iteration, int(self.data["checkpoint"]))

    def test_shapes_match_the_interface(self):
        self.assertEqual(self.policy.num_obs, interface.NUM_OBS)
        self.assertEqual(self.policy.num_single_obs, interface.NUM_SINGLE_OBS)
        self.assertEqual(self.policy.num_actions, interface.NUM_ACTIONS)
        self.assertEqual(self.policy.num_latent, interface.NUM_LATENT)

    def test_actions_match_the_training_stack(self):
        """Same observations in, same actions out, to float32 rounding.

        The history is rebuilt here from the recorded single observations rather than replayed
        from a stored stack -- `TestHistory` is what earns the right to do that. Step 0 is skipped:
        a freshly constructed UniFP environment hands its play loop an all-zero observation on the
        first control step, which is an artefact of upstream's play path and not the contract
        (see `policy.ObsHistory`).
        """
        frames = torch.as_tensor(self.data["obs_single"])
        recorded = torch.as_tensor(self.data["action"])
        history = ObsHistory(1)
        worst = 0.0
        for index in range(len(frames)):
            stacked = history.append(frames[index:index + 1])
            if index == 0:
                continue
            action = self.policy.act(stacked)
            worst = max(worst, float((action - recorded[index:index + 1]).abs().max()))
        self.assertLess(worst, TOLERANCE, f"actions differ by up to {worst}")

    def test_estimates_are_unscaled_into_si(self):
        self.policy.act(torch.zeros(1, interface.NUM_OBS))
        estimates = self.policy.estimates()
        for field in ("base_lin_vel", "ee_pos_sphere", "ee_force", "base_force"):
            with self.subTest(field=field):
                self.assertEqual(tuple(getattr(estimates, field).shape), (1, 3))


if __name__ == "__main__":
    unittest.main()

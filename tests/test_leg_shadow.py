"""The dog's numpy leg policy (`demos/combiner_key/robot/`) against the training code, without a simulator.

A recorded Isaac rollout is checked by `check_shadow.py --sim` (the Week 6 log has the run); these check the pieces on
synthetic inputs, and the shadow node offline on a fake robot.
"""
import math
import re
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np
import torch

from demos.combiner_key import classic_stance
from demos.combiner_key.robot import check_shadow, export_policy, leg_policy
from unifp_isaaclab import interface
from unifp_train import foot_goals, leg_cfg, observations

ROBOT = Path(__file__).resolve().parents[1] / "demos" / "combiner_key" / "robot"


def fake_checkpoint(path: Path, seed: int = 3) -> None:
    """An rsl-rl checkpoint of a randomly initialised 12-action UniFP actor."""
    from tensordict import TensorDict

    from unifp_train.models import UniFPActor

    torch.manual_seed(seed)
    probe = TensorDict({"policy": torch.zeros(1, interface.NUM_OBS),
                        "critic": torch.zeros(1, observations.NUM_CRITIC_OBS)}, batch_size=[1])
    actor = UniFPActor(probe, {"actor": ["policy"], "critic": ["critic"]}, "actor", 12,
                       distribution_cfg={"class_name": "GaussianDistribution", "init_std": 1.0, "std_type": "scalar"})
    torch.save({"actor_state_dict": actor.state_dict(), "iter": 7}, path)


class Export(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.ckpt = Path(cls.tmp.name) / "model_7.pt"
        fake_checkpoint(cls.ckpt)
        cls.result = export_policy.export(cls.ckpt, "force", Path(cls.tmp.name) / "p.npz")
        cls.policy = leg_policy.LegPolicy(cls.result["out"])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_numpy_reproduces_the_checkpoint(self):
        self.assertLess(self.result["action_max_abs_err"], 1e-5)
        self.assertLess(self.result["estimate_max_abs_err"], 1e-5)

    def test_the_export_carries_the_training_constants(self):
        m = self.policy.meta
        self.assertEqual(m["dof_names"], list(interface.DOF_NAMES))
        self.assertEqual(m["default_dof_pos"], list(interface.DEFAULT_DOF_POS))
        self.assertEqual(m["key_down_goal"], list(leg_cfg.KEY_DOWN_GOAL))
        self.assertEqual(m["num_leg_actions"], 12)
        self.assertEqual(m["layers"], {"encoder": 4, "actor": 4, "decoder": 3})

    def test_frame_matches_single_obs(self):
        """The dog's frame from Unitree-ordered legs against `interface.single_obs` with the training permutation."""
        rng = np.random.default_rng(1)
        obs = leg_policy.LegObservation(self.policy.meta)
        names = list(leg_policy.UNITREE_LEGS) + list(interface.ISAACLAB_NAMES[12:])
        perm = interface.permutation_from(names)
        for _ in range(20):
            quat = rng.normal(size=4)
            quat /= np.linalg.norm(quat)
            gyro, q, dq = rng.normal(size=3), rng.normal(size=20), rng.normal(size=20)
            commands, prev = rng.normal(size=15), rng.normal(size=12)
            obs.prev_actions = np.concatenate((prev, np.zeros(6)))
            mine = obs.frame(quat, gyro, q[:12], dq[:12], q[12:18], dq[12:18], commands)
            f = lambda x: torch.as_tensor(np.asarray(x), dtype=torch.float64).unsqueeze(0)
            ref = interface.single_obs(f(quat), f(gyro), f(q)[:, perm], f(dq)[:, perm], f(obs.prev_actions),
                                       torch.zeros(1, dtype=torch.float64), f(commands))[0].numpy()
            np.testing.assert_allclose(mine, ref, atol=1e-12)

    def test_targets_are_in_unitree_order(self):
        obs = leg_policy.LegObservation(self.policy.meta)
        action = np.arange(12, dtype=float)      # UniFP order: FL hip 0, FL thigh 1, ..., FR hip 3, ...
        targets = obs.leg_targets_unitree(action)
        # Unitree's first motor is FR_hip: UniFP index 3.
        self.assertAlmostEqual(targets[0], 3 * interface.ACTION_SCALE + interface.DEFAULT_DOF_POS[3])
        # Unitree's FL_calf (index 5) is UniFP's 2.
        self.assertAlmostEqual(targets[5], 2 * interface.ACTION_SCALE + interface.DEFAULT_DOF_POS[2])

    def test_waiting_command_is_the_footfall(self):
        mine = leg_policy.waiting_command(self.policy.meta)
        ref = foot_goals.local_to_sphere(torch.as_tensor(leg_cfg.KEY_DOWN_GOAL, dtype=torch.float64)).numpy()
        np.testing.assert_allclose(mine[3:6], ref, atol=1e-12)
        self.assertEqual(np.count_nonzero(np.delete(mine, [3, 4, 5])), 0)

    def test_estimates_decode(self):
        v, foot, ff, bf = np.array([0.3, -0.1, 0.05]), np.array([0.3, -0.2, 0.15]), np.array([5., -3., 20.]), np.array([1., 2., 3.])
        sphere = foot_goals.local_to_sphere(torch.as_tensor(foot)).numpy() * np.array(observations.EE_SPHERE_SCALE)
        encoded = np.concatenate((v * interface.COMMANDS_SCALE[0], sphere, ff * observations.OBS_SCALE_EE_FORCE,
                                  bf * observations.OBS_SCALE_BASE_FORCE))
        d = leg_policy.decode_estimates(self.policy.meta, encoded)
        for got, want in ((d["base_lin_vel_b"], v), (d["foot_local"], foot), (d["foot_force"], ff), (d["base_force"], bf)):
            np.testing.assert_allclose(got, want, atol=1e-9)

    def test_foot_kinematics_match_classic_stance(self):
        q = np.array(leg_cfg.SETTLED_LEG_POS).reshape(4, 3)
        for k, leg in enumerate(classic_stance.LEGS):
            np.testing.assert_allclose(leg_policy.foot_local(leg, q[k], self.policy.meta), classic_stance.foot(leg, q[k]),
                                       atol=1e-12)

    def test_the_shadow_node_offline_on_a_still_robot(self):
        """`Shadow` on fake LowStates of a still robot in the classic stand, then `check_shadow --dog` on its record."""
        from demos.combiner_key.robot import shadow

        stance = np.array(classic_stance.stance(0.41, 0.28, 0.30)["legs"])
        q_unitree = stance[leg_policy.LegObservation(self.policy.meta).to_unitree]
        args = types.SimpleNamespace(policy=self.result["out"], foot_goal=None, arm_link_port=0, print_s=100.0,
                                     run_dir=self.tmp.name, topic="/lowstate")
        node = shadow.Shadow(args)
        motor = lambda q: types.SimpleNamespace(q=q, dq=0.0, tau_est=1.0)
        msg = types.SimpleNamespace(motor_state=[motor(v) for v in q_unitree] + [motor(0.0)] * 8,
                                    imu_state=types.SimpleNamespace(quaternion=[1.0, 0, 0, 0], gyroscope=[0.0, 0, 0],
                                                                    rpy=[0.0, 0, 0]), foot_force=[20, 20, 20, 20])
        for _ in range(60):
            node.on_state(msg)
            node.tick()
        record = node.save()
        report = check_shadow.check_dog(self.policy, record, str(self.ckpt))
        self.assertEqual(report["steps"], 60)
        self.assertEqual(report["resets"], [0])
        self.assertLess(report["frame_vs_training_code_max_abs_err"], 1e-9)
        self.assertLess(report["dog_vs_torch_action_max_abs_err"], 1e-5)
        # A still robot: the record is the still-robot prediction itself.
        self.assertLess(max(report["still_robot_prediction"]["max_rad"].values()), 1e-9)
        self.assertEqual(int(np.load(record / "shadow.npz")["commands"][0].any()), 0)

    def test_the_flushed_parts_are_the_record(self):
        """The loop's flush writes only new steps (its cost cannot grow with the run, F-175); the parts concatenated are
        the record, and the final save removes them."""
        from demos.combiner_key.robot import shadow

        stance = np.array(classic_stance.stance(0.41, 0.28, 0.30)["legs"])
        q_unitree = stance[leg_policy.LegObservation(self.policy.meta).to_unitree]
        args = types.SimpleNamespace(policy=self.result["out"], foot_goal=None, arm_link_port=0, print_s=100.0,
                                     run_dir=self.tmp.name, topic="/lowstate")
        node = shadow.Shadow(args)
        for i in range(45):
            motor = lambda q: types.SimpleNamespace(q=q + 0.001 * i, dq=0.0, tau_est=1.0)
            node.on_state(types.SimpleNamespace(
                motor_state=[motor(v) for v in q_unitree] + [motor(0.0)] * 8, foot_force=[20, 20, 20, 20],
                imu_state=types.SimpleNamespace(quaternion=[1.0, 0, 0, 0], gyroscope=[0.0, 0, 0], rpy=[0.0, 0, 0])))
            node.tick()
            if i in (9, 29, 30):
                node.flush()
        node.flush()
        parts = sorted(node.parts.glob("*.npz"))
        self.assertEqual([len(np.load(p)["t"]) for p in parts], [10, 20, 1, 14])
        joined = {k: np.concatenate([np.load(p)[k] for p in parts]) for k in np.load(parts[0]).files}
        record = node.save()
        saved = np.load(record / "shadow.npz")
        self.assertEqual(sorted(joined), sorted(saved.files))
        for key in saved.files:
            np.testing.assert_array_equal(joined[key], saved[key], key)
        self.assertFalse(node.parts.exists())


class ReadOnly(unittest.TestCase):
    def test_the_shadow_node_sends_nothing(self):
        source = (ROBOT / "shadow.py").read_text()
        code = "\n".join(line for line in source.splitlines() if not line.strip().startswith("#"))
        for forbidden in ("create_publisher", "LowCmd", "Request", "sendall", ".send(", "create_client", "SportClient"):
            self.assertNotIn(forbidden, code, forbidden)
        self.assertTrue(re.search(r"from unitree_go\.msg import LowState\n", source))

    def test_the_dog_side_needs_no_torch(self):
        for name in ("leg_policy.py", "shadow.py"):
            source = (ROBOT / name).read_text()
            self.assertNotIn("import torch", source, name)
            self.assertNotIn("unifp_isaaclab", source.split('"""', 2)[2], name)


if __name__ == "__main__":
    unittest.main()

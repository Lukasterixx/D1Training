"""Exercise the actual PPO API on synthetic CPU data, without a simulator.

Widths match position_only.env_cfg POLICY_OBS_DIM / CRITIC_OBS_DIM (that module needs Isaac Sim to import)."""
from copy import deepcopy
from contextlib import redirect_stdout
import io
from types import SimpleNamespace
import unittest

import torch
from tensordict import TensorDict
from rsl_rl.algorithms import PPO

from position_only.agent import make_agent_cfg


class PositionAgentTests(unittest.TestCase):
    def test_stochastic_actor_can_complete_a_finite_ppo_update(self):
        threads = torch.get_num_threads()
        torch.set_num_threads(1)
        try:
            with torch.random.fork_rng(devices=[]):
                torch.manual_seed(42)
                cfg = make_agent_cfg(device="cpu", iterations=1)
                cfg["multi_gpu"] = None
                # Only shape metadata is needed to construct the algorithm.
                env = SimpleNamespace(num_envs=4, num_actions=18)
                obs = TensorDict({"policy": torch.randn(4, 66), "critic": torch.randn(4, 87)}, batch_size=[4])
                with redirect_stdout(io.StringIO()):
                    algorithm = PPO.construct_algorithm(obs, env, deepcopy(cfg), "cpu")
                # The critic must be built from the privileged group, not the actor's inputs.
                first_layer = lambda model: next(m for m in model.modules() if isinstance(m, torch.nn.Linear))
                self.assertEqual(first_layer(algorithm.actor).in_features, 66)
                self.assertEqual(first_layer(algorithm.critic).in_features, 87)
                before = [p.detach().clone() for p in algorithm.actor.parameters()]
                self.assertIsNotNone(algorithm.actor.distribution)
                for _ in range(cfg["num_steps_per_env"]):
                    with torch.inference_mode():
                        actions = algorithm.act(obs)
                        self.assertEqual(tuple(actions.shape), (4, 18))
                        obs = TensorDict({"policy": torch.randn(4, 66), "critic": torch.randn(4, 87)}, batch_size=[4])
                        algorithm.process_env_step(
                            obs, -actions.square().mean(dim=-1), torch.zeros(4, dtype=torch.bool), {}
                        )
                with torch.inference_mode():
                    algorithm.compute_returns(obs)
                losses = algorithm.update()
                self.assertTrue(all(torch.isfinite(torch.tensor(v)) for v in losses.values()))
                self.assertTrue(all(torch.isfinite(p).all() for p in algorithm.actor.parameters()))
                self.assertTrue(any(not torch.equal(a, b) for a, b in zip(before, algorithm.actor.parameters())))
        finally:
            torch.set_num_threads(threads)


if __name__ == "__main__":
    unittest.main()

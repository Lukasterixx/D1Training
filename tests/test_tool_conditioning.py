"""S1's tool-conditioned network (`models.UniFPActor`'s tool group, `run_unifp_train.widen_tool_checkpoint`,
`eval.EvaluablePolicy`): a parent widened with zero columns computes exactly what it did, whatever the tool input, so P3
and P4 start from the same policy and only the information they are given differs. Needs rsl-rl (the Isaac env)."""
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import torch

try:
    from tensordict import TensorDict
    from rsl_rl.models import MLPModel

    from unifp_isaaclab import interface
    from unifp_train import keylock, observations
    from unifp_train.models import UniFPActor
    HAVE_RSL = True
except ImportError:                     # the system Python
    HAVE_RSL = False

N_TOOL = 10


def _dist():
    return {"class_name": "GaussianDistribution", "init_std": 1.0, "std_type": "scalar"}


@unittest.skipUnless(HAVE_RSL, "needs rsl-rl")
class WidenTests(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(0)
        self.obs = TensorDict({"policy": torch.randn(5, interface.NUM_OBS),
                               "critic": torch.randn(5, observations.NUM_CRITIC_OBS),
                               "tool": torch.randn(5, N_TOOL), "tool_true": torch.randn(5, N_TOOL)}, batch_size=[5])
        self.parent = UniFPActor(self.obs, {"actor": ["policy"], "critic": ["critic"]}, "actor", 18,
                                 distribution_cfg=_dist())
        self.critic = MLPModel(self.obs, {"actor": ["policy"], "critic": ["critic"]}, "critic", 1,
                               hidden_dims=[512, 256, 128])
        # Not the initial weights' zero output head: perturb so the comparison means something.
        with torch.no_grad():
            for p in list(self.parent.parameters()) + list(self.critic.parameters()):
                p.add_(0.05 * torch.randn_like(p))
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "parent.pt"
        torch.save({"actor_state_dict": self.parent.state_dict(), "critic_state_dict": self.critic.state_dict(),
                    "iter": 8200, "infos": None}, self.path)

    def tearDown(self):
        self.tmp.cleanup()

    def test_widened_actor_and_critic_compute_the_parent(self):
        import run_unifp_train

        out = run_unifp_train.widen_tool_checkpoint(str(self.path), Path(self.tmp.name) / "wide.pt", N_TOOL, N_TOOL)
        ck = torch.load(out, weights_only=False)
        self.assertEqual(ck["iter"], 8200)
        groups = {"actor": ["policy", "tool"], "critic": ["critic", "tool_true"]}
        wide = UniFPActor(self.obs, groups, "actor", 18, distribution_cfg=_dist(), num_tool_obs=N_TOOL)
        wide.load_state_dict(ck["actor_state_dict"], strict=True)
        wide_critic = MLPModel(self.obs, groups, "critic", 1, hidden_dims=[512, 256, 128])
        wide_critic.load_state_dict(ck["critic_state_dict"], strict=True)
        with torch.no_grad():
            self.assertLess(float((wide(self.obs) - self.parent(self.obs)).abs().max()), 1e-6)
            self.assertLess(float((wide.estimate(self.obs) - self.parent.estimate(self.obs)).abs().max()), 1e-6)
            self.assertLess(float((wide_critic(self.obs) - self.critic(self.obs)).abs().max()), 1e-5)
        # Already wide: nothing to do.
        self.assertIsNone(run_unifp_train.widen_tool_checkpoint(str(out), Path(self.tmp.name) / "x.pt", N_TOOL, N_TOOL))

    def test_the_tool_input_reaches_the_action_once_trained(self):
        """The new columns are live: with them non-zero the tool input changes the action (not a dead input)."""
        groups = {"actor": ["policy", "tool"], "critic": ["critic", "tool_true"]}
        wide = UniFPActor(self.obs, groups, "actor", 18, distribution_cfg=_dist(), num_tool_obs=N_TOOL)
        other = self.obs.clone()
        other["tool"] = other["tool"] + 1.0
        with torch.no_grad():
            self.assertGreater(float((wide(self.obs) - wide(other)).abs().max()), 1e-4)

    def test_evaluable_policy_reads_the_tool_width_from_the_checkpoint(self):
        import run_unifp_train
        from unifp_train.eval import EvaluablePolicy

        out = run_unifp_train.widen_tool_checkpoint(str(self.path), Path(self.tmp.name) / "wide.pt", N_TOOL, N_TOOL)
        torch.save({"actor_state_dict": torch.load(out, weights_only=False)["actor_state_dict"]}, out)
        policy = EvaluablePolicy(str(out), device="cpu")
        self.assertEqual(policy.num_tool_obs, N_TOOL)
        narrow = EvaluablePolicy(str(self.path), device="cpu")
        self.assertEqual(narrow.num_tool_obs, 0)
        with torch.no_grad():
            a = policy.act(self.obs["policy"])
            b = policy.act(self.obs["policy"], torch.zeros(5, N_TOOL))
            c = narrow.act(self.obs["policy"], self.obs["tool"])          # ignored by a tool-free network
        self.assertLess(float((a - b).abs().max()), 1e-7)
        self.assertLess(float((a - c).abs().max()), 1e-6)

    def test_descriptor_width_matches(self):
        self.assertEqual(keylock.NUM_DESCRIPTOR, N_TOOL)


if __name__ == "__main__":
    unittest.main()

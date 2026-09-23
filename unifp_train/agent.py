"""UniFP's PPO configuration, in the shape rsl-rl 5.x wants it.

Every hyperparameter is upstream's, from `Go2D1PosForceRoughCfgPPO` and the `LeggedRobotCfgPPO`
base it inherits. Nothing is tuned here; where the two libraries disagree about a name the value
is carried across unchanged, and where rsl-rl 5.x has an option legged_gym did not, the default is
whichever setting reproduces legged_gym's behaviour.

The two lines that matter most:

  * `actor.class_name` points at `models.UniFPActor`, which is an encoder and a decoder around the
    actor MLP rather than a plain network. rsl-rl 5.x resolves model and algorithm classes by
    dotted path, so this needs no fork of the library.
  * `obs_normalization` is **off** for both models. UniFP feeds raw scaled observations clipped at
    +/-100; the running normalizer that Isaac Lab's own tasks use would train a different policy.

`max_iterations` is 60,000 upstream. The run that produced `model_48800` reached 60,000 in 39.85 h
on this machine's GPU and most of that was wasted (see the week 1 log) -- the number here is the
config's, and what a run actually asks for is a command-line argument.
"""
from __future__ import annotations

from . import task_cfg


def make_agent_cfg(seed: int = 1, device: str = "cuda:0", iterations: int = 60000,
                   experiment_name: str = "unifp_go2d1", run_name: str = "") -> dict:
    """The agent config as a plain dict, matching this repository's other runners."""
    return {
        "seed": seed,
        "device": device,
        # 24 x 4096 = 98,304 transitions per iteration, upstream's.
        "num_steps_per_env": task_cfg.NUM_STEPS_PER_ENV,
        "max_iterations": iterations,
        "empirical_normalization": False,
        # "estimates" is deliberately absent: it is carried by the rollout storage for the
        # adaptation loss, and no model reads it. Adding it to either set would hand the actor the
        # privileged quantities it is supposed to be inferring.
        "obs_groups": {"actor": ["policy"], "critic": ["critic"]},
        "actor": {
            "class_name": "unifp_train.models.UniFPActor",
            "hidden_dims": [512, 256, 128],
            "activation": "elu",
            "obs_normalization": False,
            # legged_gym's ActorCritic learns one std per action, initialised at 1.0, and stores it
            # unlogged. "scalar" is that parameterisation.
            "distribution_cfg": {"class_name": "GaussianDistribution", "init_std": 1.0,
                                 "std_type": "scalar"},
        },
        "critic": {
            "class_name": "MLPModel",
            "hidden_dims": [512, 256, 128],
            "activation": "elu",
            "obs_normalization": False,
        },
        "algorithm": {
            "class_name": "unifp_train.algorithm.UniFPPPO",
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "learning_rate": 1.0e-3,
            "schedule": "adaptive",
            "gamma": 0.99,
            "lam": 0.95,
            "desired_kl": 0.01,
            "max_grad_norm": 1.0,
            "normalize_advantage_per_mini_batch": False,
            "rnd_cfg": None,
            "symmetry_cfg": None,
        },
        "save_interval": 200,
        "experiment_name": experiment_name,
        "run_name": run_name,
        "logger": "tensorboard",
        "check_for_nan": True,
        "resume": False,
        "load_run": ".*",
        "load_checkpoint": "model_.*.pt",
    }

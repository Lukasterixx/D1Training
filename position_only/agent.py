"""Fresh PPO configuration for the installed RSL-RL 5.x API."""


def make_agent_cfg(seed=42, device="cuda:0", iterations=100):
    return {
        "seed": seed, "device": device,
        "num_steps_per_env": 24, "max_iterations": iterations,
        # Asymmetric actor-critic: the critic also sees base linear velocity and joint torques.
        "obs_groups": {"actor": ["policy"], "critic": ["critic"]},
        "actor": {
            "class_name": "MLPModel", "hidden_dims": [256, 128, 128], "activation": "elu",
            "obs_normalization": True,
            "distribution_cfg": {"class_name": "GaussianDistribution", "init_std": 0.5, "std_type": "log"},
        },
        "critic": {
            "class_name": "MLPModel", "hidden_dims": [256, 128, 128], "activation": "elu",
            "obs_normalization": True,
        },
        "algorithm": {
            "class_name": "PPO", "value_loss_coef": 1.0, "use_clipped_value_loss": True,
            "clip_param": 0.2, "entropy_coef": 0.01, "num_learning_epochs": 5,
            "num_mini_batches": 4, "learning_rate": 0.0003, "schedule": "adaptive",
            "gamma": 0.99, "lam": 0.95, "desired_kl": 0.01, "max_grad_norm": 1.0,
            "rnd_cfg": None, "symmetry_cfg": None,
        },
        "save_interval": 50, "experiment_name": "position_only_reach",
        "run_name": "", "logger": "tensorboard", "check_for_nan": True,
    }

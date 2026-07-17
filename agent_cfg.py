"""RSL-RL config for the Go2 walking policy.

Carried over verbatim from Rescue so the checkpoint in `logs/` loads against the
same network shape. Only the Go2 entry is kept -- there is no G1 here.

The training hyperparameters below are inert during playback; `sim.py` only uses
`experiment_name`, `load_run`, `load_checkpoint` and `device`. They are kept
intact because this repo is named D1Training and will presumably want them.
"""

unitree_go2_agent_cfg = {
    'seed': 42,
    'device': 'cuda',
    'num_steps_per_env': 24,
    'max_iterations': 15000,
    'empirical_normalization': False,
    'obs_groups': {
        'actor': ['policy'],
        'critic': ['policy'],
    },
    'actor': {
        'class_name': 'MLPModel',
        'hidden_dims': [512, 256, 128],
        'activation': 'elu',
    },
    'critic': {
        'class_name': 'MLPModel',
        'hidden_dims': [512, 256, 128],
        'activation': 'elu',
    },
    'algorithm': {
        'class_name': 'PPO',
        'value_loss_coef': 1.0,
        'use_clipped_value_loss': True,
        'clip_param': 0.2,
        'entropy_coef': 0.01,
        'num_learning_epochs': 5,
        'num_mini_batches': 4,
        'learning_rate': 0.001,
        'schedule': 'adaptive',
        'gamma': 0.99,
        'lam': 0.95,
        'desired_kl': 0.01,
        'max_grad_norm': 1.0,
    },
    'save_interval': 50,
    'experiment_name': 'unitree_go2_rough',
    'run_name': '',
    'logger': 'tensorboard',
    'neptune_project': 'isaaclab',
    'wandb_project': 'isaaclab',
    'resume': False,
    'load_run': '.*',
    'load_checkpoint': 'model_.*.pt',
}

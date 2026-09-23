"""UniFP's position/force task, being rebuilt to train natively in Isaac Lab.

`unifp_isaaclab/` runs a checkpoint trained elsewhere; this trains one here. The motivation is
F-088: the policy trained on the legacy Isaac Gym stack holds a stance on the Isaac Lab model but
falls within half a second when told to walk, and the reproduction is therefore not portable in
the form it exists in. Training the same task on the stack it will be evaluated on removes the
question rather than answering it.

Modules, in the order they build on each other:

  * `task_cfg` -- reward weights, command ranges, force and curriculum constants
  * `gait`, `rewards`, `observations` -- the task itself, each checked against a recorded
    Isaac Gym rollout
  * `forces` -- the external-force schedule, the half that makes this position *and force*
  * `env_cfg`, `env` -- the `DirectRLEnv` those add up to
  * `models`, `algorithm`, `agent` -- the adaptation-module actor-critic, its extra PPO loss,
    and the hyperparameters

`models` and `algorithm` import `rsl_rl` and `env` imports `isaaclab`; everything else is plain
torch and can be exercised without a simulator, which is how the task was verified.

See README.md for what is checked against the training stack and what is not.
"""

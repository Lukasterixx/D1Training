# unitree_rl_lab

Source: <https://github.com/unitreerobotics/unitree_rl_lab>, as vendored in the MaiRo RL Lab
container (`~/mairo-rl-lab-rinam`, origin `github.com/vick-l1m/mairo-rl-lab-rinam`, commit `a179aa0`).
Licensed under the Apache License 2.0 (`LICENCE` in this folder).

Adapted into this repository, with changes described in each file's docstring:

| This repository | Adapted from |
| --- | --- |
| `motor_model.py`, `unitree_actuators.py` | `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree_actuators.py` |
| `position_only/deploy.py` | `source/unitree_rl_lab/unitree_rl_lab/utils/export_deploy_cfg.py` |
| Randomisation and observation-noise values in `position_only/env_cfg.py` | `tasks/locomotion/robots/go2/velocity_env_cfg.py` |

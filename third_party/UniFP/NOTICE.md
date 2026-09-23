# UniFP

Source: <https://github.com/unified-force/UniFP>, commit
[`68847a070f88`](https://github.com/unified-force/UniFP/commit/68847a070f88d731058c3d8476929bc3b205f5bd)
(the revision pinned in [docs/codebase_investigation.md](../../docs/codebase_investigation.md)).
Licensed under the BSD 3-Clause License (`LICENCE` in this folder).

UniFP is legacy `legged_gym` on Isaac Gym Preview 4 and Python 3.8, which is a different stack
from this repository's Isaac Lab installation. It is therefore **not vendored into this
repository**: it is cloned separately (`~/thesis_b_legacy/UniFP`) and left unmodified except
for the port below, which is generated into the clone by scripts kept here. Nothing in
`~/thesis_b_legacy` is committed; `unifp_go2d1/` is the committed source of truth.

Adapted into this repository, with changes described in each file's docstring:

| This repository | Adapted from |
| --- | --- |
| `unifp_go2d1/go2d1_pos_force_config.py` | `legged_gym/envs/b2/b2z1_pos_force_config.py` |
| `unifp_go2d1/port_env.py` (generates `legged_gym/envs/go2d1/legged_robot_go2d1_pos_force.py`) | `legged_gym/envs/b2/legged_robot_b2z1_pos_force.py` |
| `unifp_go2d1/build_asset.py` (generates `resources/robots/go2d1/go2d1.urdf`) | asset layout of `resources/robots/b2z1/` |
| `unifp_go2d1/launch_training.py` | `legged_gym/scripts/train_b2z1posforce.py` |

The PPO implementation, history encoder, estimator supervision, force curriculum and reward
terms are UniFP's and are used unchanged; the port changes the robot, not the method. See
[unifp_go2d1/README.md](../../unifp_go2d1/README.md) for what differs and what is not validated.

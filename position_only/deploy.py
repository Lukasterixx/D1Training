"""Write the sim-to-real contract for a run: what a robot-side controller must reproduce.

The idea is unitree_rl_lab's `export_deploy_cfg.py` (Apache-2.0; see
third_party/unitree_rl_lab/NOTICE.md): every run records the joint-to-motor mapping, PD gains,
action scaling and limits, and the exact observation layout. unitree_rl_lab's C++ controller
reads that file on the robot, so the deployed policy sees the same processing it trained with.

Changes for this robot: two buses (Go2 legs over unitree_sdk2, D1 arm over its own SDK) instead
of one; this task's raw-action clip and soft-limit clamp; each actuator's model and envelope,
so a run states which motor model it trained against. The D1 arm's 4000/400 gains are
simulation drive gains standing in for its internal servo loop, not values to send to it.
`build_manifest` is plain Python; `export_deploy_cfg` reads a live Isaac Lab environment.
"""
from __future__ import annotations

from pathlib import Path

FORMAT = "d1training-deploy/1"

# unitree_sdk2 LowCmd motor order for the Go2.
GO2_SDK_JOINT_ORDER = [f"{leg}_{part}_joint" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")]
# D1 protocol id 0 is URDF Joint1 (see README, "The arm's mass").
D1_SDK_JOINT_ORDER = [f"Joint{i}" for i in range(1, 7)]
BUSES = {
    "go2_legs": {"interface": "unitree_sdk2 LowCmd", "sdk_joint_order": GO2_SDK_JOINT_ORDER},
    "d1_arm": {"interface": "D1 arm SDK", "sdk_joint_order": D1_SDK_JOINT_ORDER},
}


def _bus_of(joint: str) -> tuple[str, int]:
    matches = [(bus, spec["sdk_joint_order"].index(joint)) for bus, spec in BUSES.items()
               if joint in spec["sdk_joint_order"]]
    if len(matches) != 1:
        raise ValueError(f"Joint {joint!r} is not on exactly one motor bus.")
    return matches[0]


def build_manifest(step_dt: float, actions: list[dict], observations: list[dict], actuators: list[dict]) -> dict:
    """Assemble and check the contract.

    actions: in policy-output order; each {name, joints, scale, offset, raw_clip, target_limits}.
    observations: the actor group in concatenation order; each {name, dim, scale, clip, history_length}.
    actuators: each {name, model, joints, stiffness, damping, effort_limit, velocity_limit, envelope}.
    """
    action_terms, index = {}, 0
    for term in actions:
        joints = list(term["joints"])
        for key in ("scale", "offset", "target_limits"):
            if len(term[key]) != len(joints):
                raise ValueError(f"Action term {term['name']!r}: {key} has {len(term[key])} values for {len(joints)} joints.")
        placed = [_bus_of(joint) for joint in joints]
        buses = {bus for bus, _ in placed}
        if len(buses) != 1:
            raise ValueError(f"Action term {term['name']!r} spans motor buses {sorted(buses)}.")
        action_terms[term["name"]] = {
            "policy_output_indices": [index, index + len(joints)],
            "bus": buses.pop(), "joints": joints, "motor_ids": [motor for _, motor in placed],
            "raw_clip": list(term["raw_clip"]), "scale": list(term["scale"]), "offset": list(term["offset"]),
            "target_limits": [list(pair) for pair in term["target_limits"]],
            "target": "offset + scale * clip(raw_action, raw_clip), then clamped to target_limits",
        }
        index += len(joints)

    obs_terms, width = {}, 0
    for term in observations:
        span = term["dim"] * term["history_length"]
        obs_terms[term["name"]] = {"input_indices": [width, width + span], **{k: v for k, v in term.items() if k != "name"}}
        width += span

    return {
        "format": FORMAT, "step_dt": step_dt, "buses": BUSES,
        "policy": {"input_width": width, "output_width": index},
        "actions": action_terms, "observations": obs_terms,
        "actuators": {term["name"]: {k: v for k, v in term.items() if k != "name"} for term in actuators},
    }


def _row(values) -> list[float]:
    """The first environment's row of a (num_envs, n) tensor, as floats."""
    return [float(v) for v in values[0].detach().cpu().reshape(-1).tolist()]


def _per_joint(value, count: int) -> list[float]:
    """A scalar broadcast to `count` joints, or the first environment's row of a tensor."""
    return [float(value)] * count if isinstance(value, (int, float)) else _row(value)


def export_deploy_cfg(env, path: str | Path, raw_clip: tuple[float, float] = (-1.0, 1.0)) -> dict:
    """Build the manifest from a live `ManagerBasedRLEnv` and write it as YAML."""
    import yaml

    robot = env.scene["robot"]
    actions = []
    for name, term in env.action_manager._terms.items():
        joints = list(term._joint_names)
        actions.append({
            "name": name, "joints": joints, "raw_clip": list(raw_clip),
            "scale": _per_joint(term._scale, len(joints)), "offset": _per_joint(term._offset, len(joints)),
            "target_limits": robot.data.soft_joint_pos_limits[0, term._joint_ids].detach().cpu().tolist(),
        })

    manager = env.observation_manager
    observations = []
    for name, dims, cfg in zip(manager.active_terms["policy"], manager.group_obs_term_dim["policy"],
                               manager._group_obs_term_cfgs["policy"]):
        scale = cfg.scale  # The manager turns a configured scale into a tensor.
        observations.append({
            "name": name, "dim": int(dims[-1]), "history_length": max(1, int(cfg.history_length or 0)),
            "scale": None if scale is None else [float(v) for v in scale.detach().cpu().reshape(-1).tolist()],
            "clip": None if cfg.clip is None else list(cfg.clip),
        })

    actuators = []
    for name, actuator in robot.actuators.items():
        envelope = {key: _row(getattr(actuator, attr)) for key, attr in
                    (("Y1", "_effort_y1"), ("Y2", "_effort_y2"), ("X1", "_velocity_x1"), ("X2", "_velocity_x2"))
                    if hasattr(actuator, attr)}
        actuators.append({
            "name": name, "model": type(actuator).__name__, "joints": list(actuator.joint_names),
            "stiffness": _row(actuator.stiffness), "damping": _row(actuator.damping),
            "effort_limit": _row(actuator.effort_limit), "velocity_limit": _row(actuator.velocity_limit),
            "envelope": envelope or None,
        })

    manifest = build_manifest(float(env.step_dt), actions, observations, actuators)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None, width=120))
    return manifest

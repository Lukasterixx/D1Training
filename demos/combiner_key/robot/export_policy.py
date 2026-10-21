#!/usr/bin/env python3
"""Export a key-turning leg policy for the dog: its weights and its interface's constants in one `.npz` (on the PC).

    python demos/combiner_key/robot/export_policy.py --checkpoint <model.pt> --variant force --out checkpoints/<name>.npz

The dog runs it with `leg_policy.LegPolicy` (numpy only). The export reads every constant from the training code
(`unifp_isaaclab.interface`, `unifp_train.leg_cfg`, `unifp_train.observations`), so the dog's side has none of its own,
and refuses to write unless the numpy network reproduces the checkpoint's own actions and estimates
(`unifp_train.eval.EvaluablePolicy`) on random stacks to `--tolerance`.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))


def linear_layers(module) -> list[tuple[np.ndarray, np.ndarray]]:
    """The (weight, bias) of an MLP's linear layers, checking it is Linear (ELU Linear)* as `leg_policy` runs it."""
    import torch.nn as nn

    # In order and with repeats: rsl-rl's MLP registers one activation module between every pair of layers, which
    # `module.modules()` would list once.
    def flat(m):
        kids = list(m._modules.values())
        return [m] if not kids else [leaf for kid in kids for leaf in flat(kid)]

    children = flat(module)
    kinds = ["L" if isinstance(m, nn.Linear) else "E" if isinstance(m, nn.ELU) else type(m).__name__ for m in children]
    pattern = "".join(kinds)
    if not pattern or pattern != "LE" * (pattern.count("L") - 1) + "L":
        raise ValueError(f"unexpected MLP layout {kinds}; leg_policy runs Linear (ELU Linear)*")
    return [(m.weight.detach().cpu().numpy(), m.bias.detach().cpu().numpy()) for m in children if isinstance(m, nn.Linear)]


def meta_for(variant: str, checkpoint: Path, iteration, num_leg_actions: int, layers: dict) -> dict:
    from demos.combiner_key import classic_stance
    from unifp_isaaclab import interface
    from unifp_train import leg_cfg, observations

    return {
        "format": "d1training-leg-policy-1",
        "variant": variant,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "checkpoint_iteration": iteration,
        "layers": layers,
        "dof_names": list(interface.DOF_NAMES),
        "num_actions": interface.NUM_ACTIONS,
        "num_leg_actions": num_leg_actions,
        "default_dof_pos": list(interface.DEFAULT_DOF_POS),
        "obs_scale_ang_vel": interface.OBS_SCALE_ANG_VEL,
        "obs_scale_dof_pos": interface.OBS_SCALE_DOF_POS,
        "obs_scale_dof_vel": interface.OBS_SCALE_DOF_VEL,
        "commands_scale": list(interface.COMMANDS_SCALE),
        "num_commands": interface.NUM_COMMANDS,
        "cmd_ee_radius": interface.CMD_EE_RADIUS,
        "frame_stack": interface.FRAME_STACK,
        "num_single_obs": interface.NUM_SINGLE_OBS,
        "clip_observations": interface.CLIP_OBSERVATIONS,
        "clip_actions": interface.CLIP_ACTIONS,
        "action_scale": interface.ACTION_SCALE,
        "policy_dt": interface.POLICY_DT,
        "leg_kp": interface.P_GAINS[0],
        "leg_kd": interface.D_GAINS[0],
        "leg_torque_limits": list(interface.TORQUE_LIMITS[:12]),
        "goal_centre": list(leg_cfg.GOAL_CENTRE),
        "key_down_goal": list(leg_cfg.KEY_DOWN_GOAL),
        "arm_rest_pose": list(leg_cfg.ARM_REST_POSE),
        "settled_leg_pos": list(leg_cfg.SETTLED_LEG_POS),
        "settled_base_height_m": leg_cfg.SETTLED_BASE_HEIGHT_M,
        "ee_sphere_scale": list(observations.EE_SPHERE_SCALE),
        "obs_scale_ee_force": observations.OBS_SCALE_EE_FORCE,
        "obs_scale_base_force": observations.OBS_SCALE_BASE_FORCE,
        "kinematics": {"hip_x": classic_stance.HIP_X, "hip_y": classic_stance.HIP_Y, "thigh_y": classic_stance.THIGH_Y,
                       "link_m": classic_stance.LINK_M, "foot_sphere_x": classic_stance.FOOT_SPHERE_X,
                       "foot_radius_m": classic_stance.FOOT_RADIUS_M},
        "trained_on": {"legs": "UniFP ideal PD, kp 25 / kd 0.5, URDF limits, armature 2e-4 (F-168)",
                       "start": "the settled stance (leg_cfg.SETTLED_LEG_POS, base 0.272 m) or recorded walker stops",
                       "noise": "none", "delay": "none"},
    }


def export(checkpoint: str | Path, variant: str, out: str | Path, tolerance: float = 1e-4) -> dict:
    """Write `out` from `checkpoint`; refuse (raise) unless numpy reproduces the checkpoint's actions and estimates."""
    import torch
    from tensordict import TensorDict

    from demos.combiner_key.robot import leg_policy
    from unifp_isaaclab import interface
    from unifp_train.eval import EvaluablePolicy

    checkpoint = Path(checkpoint).resolve()
    policy = EvaluablePolicy(str(checkpoint), device="cpu")
    actor = policy.actor
    parts = {"encoder": linear_layers(actor.encoder), "actor": linear_layers(actor.mlp),
             "decoder": linear_layers(actor.decoder)}
    arrays = {}
    for name, layers in parts.items():
        for k, (w, b) in enumerate(layers):
            arrays[f"{name}_w{k}"], arrays[f"{name}_b{k}"] = w.astype(np.float32), b.astype(np.float32)
    meta = meta_for(variant, checkpoint, policy.iteration, policy.num_actions,
                    {name: len(layers) for name, layers in parts.items()})
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp.npz")
    np.savez_compressed(tmp, meta=np.array(json.dumps(meta)), **arrays)

    # The numpy network against the checkpoint's own, on stacks at the observation's scale and wider.
    exported = leg_policy.LegPolicy(tmp)
    rng = np.random.default_rng(20261021)
    stacks = np.concatenate((rng.normal(0.0, 0.5, (48, interface.NUM_OBS)), rng.normal(0.0, 2.0, (16, interface.NUM_OBS))))
    with torch.inference_mode():
        batch = torch.as_tensor(stacks, dtype=torch.float32)
        ref_act = policy.act(batch).numpy()
        ref_est = actor.estimate(TensorDict({"policy": batch}, batch_size=[len(stacks)])).numpy()
    result = {"checkpoint": str(checkpoint), "variant": variant, "iteration": policy.iteration, "stacks": len(stacks),
              "action_max_abs_err": float(np.abs(exported.act(stacks) - ref_act).max()),
              "estimate_max_abs_err": float(np.abs(exported.estimate(stacks) - ref_est).max()),
              "action_max_abs": float(np.abs(ref_act).max())}
    if result["action_max_abs_err"] > tolerance or result["estimate_max_abs_err"] > tolerance:
        tmp.unlink()
        raise ValueError(f"numpy does not reproduce the checkpoint to {tolerance}: {result}")
    tmp.replace(out)
    result.update(out=str(out), bytes=out.stat().st_size, sha256=hashlib.sha256(out.read_bytes()).hexdigest())
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--variant", choices=("force", "position"), required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--tolerance", type=float, default=1e-4, help="largest |numpy - torch| allowed")
    args = parser.parse_args()
    try:
        r = export(args.checkpoint, args.variant, args.out, args.tolerance)
    except ValueError as error:
        print(f"REFUSED: {error}", file=sys.stderr)
        return 1
    print(f"numpy against torch on {r['stacks']} random stacks: actions {r['action_max_abs_err']:.2e}, estimates "
          f"{r['estimate_max_abs_err']:.2e} (|a| up to {r['action_max_abs']:.2f})")
    print(f"wrote {r['out']} ({r['bytes'] / 1e6:.1f} MB, sha256 {r['sha256'][:16]}...) from "
          f"{Path(r['checkpoint']).name}, {r['variant']}, iteration {r['iteration']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

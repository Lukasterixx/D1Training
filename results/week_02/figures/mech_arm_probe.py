"""Is a saturated arm joint carrying load, or oscillating between its limits? Physics-rate, under a policy.

The mechanism evaluation reports Joints 1, 2, 3 and 5 at their torque limits for the whole push
(`mech_eval` `joint_load_p95`), and |torque| cannot tell a joint holding its limit from one swinging
between +limit and -limit. This records the *signed* applied torque of the six arm joints at every
5 ms physics step, for one evaluation mechanism at every placement, while the robot holds the handle.
A joint carrying load has a mean signed load near its mean absolute load; one oscillating has a mean
near zero and flips sign often.

    python results/week_02/figures/mech_arm_probe.py --headless --checkpoint <model.pt> --kind drawer --level 60 --out <name>.json
    ... --free        # the same policy reaching in free space, no mechanism

Week 2 log, 2026-09-24, "The goal is the input".
"""
import argparse, json, sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", required=True)
parser.add_argument("--kind", default="drawer")
parser.add_argument("--level", type=float, default=60.0)
parser.add_argument("--free", action="store_true", help="Free-space reaching instead of a mechanism.")
parser.add_argument("--steps", type=int, default=600, help="Policy steps (50 Hz).")
parser.add_argument("--out", default="mech_arm_probe.json")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch
from unifp_isaaclab import interface, robot as robot_mod
from unifp_train import eval as evaluation, hook_cfg, mech_cfg, mech_eval
from unifp_train.mech_env import Go2D1MechEnv, Go2D1MechEnvCfg

cfg = Go2D1MechEnvCfg()
placements = len(hook_cfg.eval_handle_spheres())
cfg.scene.num_envs = placements
cfg.episode_length_s = mech_cfg.EVAL_EPISODE_S
armature = {name: (hook_cfg.ARM_ARMATURE_KG_M2 if name in interface.ISAACLAB_NAMES[12:18] else robot_mod.ARM_ARMATURE)
            for name in interface.ISAACLAB_NAMES}
cfg.robot = robot_mod.make_robot_cfg(str(ROOT / "generated/go2_d1.usd"), prim_path="/World/envs/env_.*/Robot",
                                     armature=armature)
if args.free:
    cfg.mechanism_fraction = 0.0
env = Go2D1MechEnv(cfg)
if not args.free:
    env.set_evaluation(*mech_eval.build_plan([args.kind], [args.level], env.device))
obs, _ = env.reset()
policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))

arm = env._order[12:18]
limits = torch.tensor(interface.TORQUE_LIMITS[12:18], device=env.device)
rows, held = [], []
original = env._apply_action


def traced():
    original()
    rows.append((env._robot.data.applied_torque[:, arm] / limits).cpu().numpy().copy())
    held.append(env._mech.grasped.cpu().numpy().copy())


env._apply_action = traced
with torch.inference_mode():
    for _ in range(args.steps):
        obs, *_ = env.step(policy.act(obs["policy"]))
tau = np.array(rows)                     # (physics steps, envs, 6)
mask = np.array(held) if not args.free else np.ones(tau.shape[:2], dtype=bool)
mask[:200] = False                       # skip the first second
flip = (np.sign(tau[1:]) != np.sign(tau[:-1])) & mask[1:, :, None] & mask[:-1, :, None]
sel = lambda a: a[mask]                  # (samples, 6)
out = {
    "checkpoint": str(Path(args.checkpoint).resolve()), "kind": None if args.free else args.kind,
    "level_n": None if args.free else args.level, "free": bool(args.free),
    "joints": [f"Joint{i}" for i in range(1, 7)], "samples": int(mask.sum()),
    "mean_abs_load": np.abs(sel(tau)).mean(axis=0).round(3).tolist(),
    "mean_signed_load": sel(tau).mean(axis=0).round(3).tolist(),
    "saturated_fraction": (np.abs(sel(tau)) > 0.99).mean(axis=0).round(3).tolist(),
    "sign_flip_fraction": (flip.sum(axis=(0, 1)) / max(1, (mask[1:] & mask[:-1]).sum())).round(3).tolist(),
    # Per environment: |mean signed| / mean |.|, 1 for a steady load, ~0 for a symmetric oscillation.
    "steadiness_median": np.median(np.stack([
        np.abs(tau[mask[:, e], e].mean(axis=0)) / np.abs(tau[mask[:, e], e]).mean(axis=0).clip(min=1e-6)
        for e in range(tau.shape[1]) if mask[:, e].sum() > 50]), axis=0).round(3).tolist(),
}
print(json.dumps(out, indent=1), flush=True)
Path(__file__).with_name(args.out).write_text(json.dumps(out, indent=1) + "\n")
import os; os._exit(0)

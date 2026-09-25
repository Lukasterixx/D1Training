"""Does the UniFP port's arm hold still, or chatter at its torque limits? Physics-rate trace, zero actions.

UniFP's arm gains (kp 60/40 N·m/rad, kd 1.0/0.8 N·m·s/rad) are applied as an explicit PD every 5 ms
physics step (`unifp_isaaclab/robot.py`, IdealPDActuator). The D1's wrist links weigh grams, so with
the 2e-4 kg·m² armature the port adds, kd·dt/I is far above the ~2 an explicit damper tolerates. This
records applied torque and joint velocity at every physics step for one standing robot, zero actions,
to see whether the torque sign alternates step to step (a numerical limit cycle) or holds.

    ./run_isaac.sh results/week_02/figures/arm_chatter_probe.py      # see the week 2 log for the exact call
Week 2 log, 2026-09-24; F-102.
"""
import argparse, json, sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--armature", type=float, default=None)
parser.add_argument("--out", default="arm_chatter_probe.json")
parser.add_argument("--arm_only", action="store_true", help="Apply --armature to the six arm joints only.")
parser.add_argument("--legs", action="store_true", help="Report the 12 leg joints instead of the arm.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from weld import build_welded_robot_usd
from unifp_isaaclab import interface, robot as robot_mod
from unifp_train.env import Go2D1PosForceEnv
from unifp_train.env_cfg import Go2D1PosForceEnvCfg

usd = build_welded_robot_usd(go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
                             d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"), out_usd_path=str(ROOT / "generated/go2_d1.usd"),
                             mount_pos=(0.0, 0.0, 0.08), arm_mass_kg=3.152).usd_path
cfg = Go2D1PosForceEnvCfg()
cfg.scene.num_envs = 1
kwargs = {} if args.armature is None else {"armature": args.armature}
if args.armature is not None and args.arm_only:
    kwargs = {"armature": {name: (args.armature if name.startswith("Joint") and name[5:].isdigit() else robot_mod.ARM_ARMATURE)
                           for name in interface.ISAACLAB_NAMES}}
cfg.robot = robot_mod.make_robot_cfg(usd, prim_path="/World/envs/env_.*/Robot", **kwargs)
cfg.force_start_step = 10 ** 9
env = Go2D1PosForceEnv(cfg)
env.reset()
span = slice(0, 12) if args.legs else slice(12, 18)
arm = env._order[span]
limits = torch.tensor(interface.TORQUE_LIMITS[span], device=env.device)
rows = []
original = env._apply_action

def traced():
    original()
    rows.append((env._robot.data.applied_torque[0, arm] / limits).cpu().numpy().copy())
    rows.append(None) if False else None

env._apply_action = traced
vel = []
for _ in range(150):
    env.step(torch.zeros(1, interface.NUM_ACTIONS, device=env.device))
    vel.append(env._robot.data.joint_vel[0, arm].cpu().numpy().copy())
tau = np.array(rows[40:])                    # physics steps, after the first ~0.2 s settle
flips = (np.sign(tau[1:]) != np.sign(tau[:-1])).mean(axis=0)
out = {
    "armature": args.armature if args.armature is not None else robot_mod.ARM_ARMATURE,
    "armature_arm_only": bool(args.arm_only), "joints": [interface.ISAACLAB_NAMES[i] for i in range(20)][span],
    "physics_steps": int(len(tau)),
    "mean_abs_load": np.abs(tau).mean(axis=0).round(3).tolist(),
    "mean_signed_load": tau.mean(axis=0).round(3).tolist(),
    "sign_flip_fraction": flips.round(3).tolist(),
    "saturated_fraction": (np.abs(tau) > 0.99).mean(axis=0).round(3).tolist(),
    "joint_speed_rms_rad_s": np.sqrt((np.array(vel[10:]) ** 2).mean(axis=0)).round(3).tolist(),
    "first_steps": tau[:8].round(3).tolist(),
}
print(json.dumps(out, indent=1), flush=True)
Path(__file__).with_name(args.out).write_text(json.dumps(out, indent=1) + "\n")
import os; os._exit(0)

"""Zero-action stance probe: the Week 1 method (16 envs, 12 s, no time limit, mean over the last 2 s).

    python stance_probe.py <mount x> <mount y> <mount z> <out.json>

Measures, relative to each environment's origin, where the Go2 base settles and where the controlled point
(the pincer tip, task_space.ZERO_ACTION_TIP_M's quantity) rests, with the arm welded at the given mount.
Everything else is run_position_only's default configuration.
"""
import json
import sys

sys.path.insert(0, "/home/lukas/D1Training")
mount = tuple(float(v) for v in sys.argv[1:4])
out_path = sys.argv[4]

from isaaclab.app import AppLauncher  # noqa: E402

app = AppLauncher(headless=True, device="cuda:0").app

import math  # noqa: E402
from pathlib import Path  # noqa: E402

import torch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # noqa: E402

from position_only.env_cfg import make_cfg  # noqa: E402
from weld import build_welded_robot_usd  # noqa: E402

NUM_ENVS, STEPS, TAIL = 16, 600, 100   # 12 s at 50 Hz; the last 2 s
assets = Path(out_path).with_suffix("") / "assets"
robot_usd = build_welded_robot_usd(
    go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
    d1_urdf_path="/home/lukas/D1Training/d1_arm/d1.urdf",
    out_usd_path=str(assets / "go2_d1.usd"), mount_pos=mount, arm_mass_kg=3.152).usd_path
# run_position_only's argparse defaults, passed explicitly.
cfg = make_cfg(robot_usd, NUM_ENVS, 42, "cuda:0", leg_actuator="unitree", robustness="none", self_collisions=True,
               latency="estimated", arm_actuator="d1_servo", arm_trajectory="measured")
cfg.terminations.time_out = None     # no time limit
cfg.episode_length_s = 1000.0
env = ManagerBasedRLEnv(cfg)
robot, command = env.scene["robot"], env.command_manager.get_term("ee_position")
action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
env.reset()
base, tip, tilt, resets = [], [], [], 0
for _ in range(STEPS):
    _, _, terminated, truncated, _ = env.step(action)
    resets += int((terminated | truncated).sum())
    base.append((robot.data.root_pos_w - env.scene.env_origins).clone())
    tip.append((command.tip_w - env.scene.env_origins).clone())
    tilt.append(torch.rad2deg(torch.acos((-robot.data.projected_gravity_b[:, 2]).clamp(-1, 1))).clone())
base, tip, tilt = torch.stack(base), torch.stack(tip), torch.stack(tilt)   # (steps, envs, ...)
tail_base, tail_tip = base[-TAIL:].mean(dim=0), tip[-TAIL:].mean(dim=0)    # per env
final_x = tail_base[:, 0].mean()
settled = (base[:, :, 0].mean(dim=1) - final_x).abs() > 0.001
settled_step = int(settled.nonzero().max()) + 1 if settled.any() else 0
report = {
    "mount_m": list(mount), "num_envs": NUM_ENVS, "steps": STEPS, "tail_steps": TAIL, "resets": resets,
    "base_offset_m": [round(float(v), 4) for v in tail_base.mean(dim=0)],
    "base_x_spread_between_envs_cm_sd": round(float(tail_base[:, 0].std()) * 100, 4),
    "tilt_deg_tail": round(float(tilt[-TAIL:].mean()), 2),
    "tip_m": [round(float(v), 4) for v in tail_tip.mean(dim=0)],
    "tip_spread_between_envs_m": [round(float(v), 4) for v in (tail_tip.amax(dim=0) - tail_tip.amin(dim=0))],
    "settled_by_s": round(settled_step * 0.02, 2),
    "peak_tilt_deg": round(float(tilt.max()), 2),
    "lowest_base_m": round(float(base[:, :, 2].min()), 4),
}
print("[probe]", json.dumps(report), flush=True)
Path(out_path).write_text(json.dumps(report, indent=2) + "\n")
env.close()
app.close()

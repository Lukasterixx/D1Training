"""The hardware step protocol, run in the simulator: each arm joint 30 deg out and back.

Mirrors the F-033 sweeps that `arm_response.py` fits (one joint at a time, 30 deg, a hold, and back),
so the simulated arm can be fitted with the same code and set beside the D1. Environment k steps joint
k; every other joint holds its default and the legs get zero actions, after the base has settled.

The command time of each leg is the policy step on which the arm's *setpoint* changed -- the step the
firmware would receive the message -- which under the 10 Hz command hold is up to 100 ms after the
action changed, at a random phase per environment, as on the real interface. Joint angles are recorded
every policy step (50 Hz), finer than the D1's 9 Hz feedback.
"""
from __future__ import annotations

import math

import torch

STEP_DEG = 30.0


def run_arm_steps(env, settle_s=4.0, hold_s=2.0, step_deg=STEP_DEG):
    from .env_cfg import ARM_NAMES

    if env.num_envs < len(ARM_NAMES):
        raise ValueError(f"arm_steps needs one environment per arm joint ({len(ARM_NAMES)}).")
    robot = env.scene["robot"]
    arm = env.action_manager.get_term("arm")
    arm_ids = [robot.joint_names.index(name) for name in ARM_NAMES]
    dt = env.step_dt
    settle, hold = int(round(settle_s / dt)), int(round(hold_s / dt))
    amplitude = math.radians(step_deg)

    env.reset()
    action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
    record = {"setpoint": [], "planned": [], "joint": [], "done": []}
    for step in range(settle + 2 * hold):
        action[:, 12:] = 0.0
        if settle <= step < settle + hold:
            for k in range(len(ARM_NAMES)):
                action[k, 12 + k] = amplitude
        with torch.inference_mode():
            _, _, terminated, truncated, _ = env.step(action)
        default = robot.data.default_joint_pos[:, arm_ids]
        record["setpoint"].append((arm.processed_actions - default).clone())
        planned = arm.planned_targets
        record["planned"].append(None if planned is None else (planned - default).clone())
        record["joint"].append((robot.data.joint_pos[:, arm_ids] - default).clone())
        record["done"].append(terminated | truncated)

    setpoint = torch.stack(record["setpoint"]).cpu()
    joint = torch.stack(record["joint"]).cpu()
    planned = None if record["planned"][0] is None else torch.stack(record["planned"]).cpu()
    done = torch.stack(record["done"]).cpu()
    legs = []
    for k, name in enumerate(ARM_NAMES):
        series = setpoint[:, k, k]
        changes = [i for i in range(1, len(series)) if abs(float(series[i] - series[i - 1])) > 1e-6]
        legs.append({
            "env": k, "joint": name, "servo_index": k,
            "command_steps": changes[:2],
            "setpoint_rad": series.tolist(),
            "joint_rad": joint[:, k, k].tolist(),
            "planned_rad": None if planned is None else planned[:, k, k].tolist(),
            "other_joints_max_abs_rad": float(joint[:, k, [j for j in range(len(ARM_NAMES)) if j != k]].abs().max()),
            "reset_during_trace": bool(done[:, k].any()),
        })
    return {"step_dt_s": dt, "settle_steps": settle, "hold_steps": hold, "step_deg": step_deg,
            "planner_active": planned is not None, "legs": legs}

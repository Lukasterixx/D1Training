#!/usr/bin/env python3
"""What does the reaching policy do when it is switched on with the arm folded?

    ./demos/unifp/folded_start_probe.py --headless

Every training episode resets the arm to its zero pose (`interface.DEFAULT_DOF_POS`, all six arm joints
at 0) and nowhere else. The real D1 rests folded against its mechanical stop, J1/J2 at about -90.9/+91.7
degrees (F-027, F-028), so a policy switched on there starts from a pose it has never been reset into.
This measures what it does about it.

Paired environments: each held goal is run twice, once from the trained zero pose and once from the
folded rest pose, everything else identical -- same goal, standing (velocity command zeroed every step),
forces off. Both stand for `--hold_s` with the arm held where it starts, then hand over with the buffers of
a fresh episode (no previous action, empty history), as a switch-on on the robot would. J1/J2 are put
just inside the URDF's +/-1.57 rad limits, 1-2 degrees short of where the real arm rests, because the
simulated arm cannot be placed past them. J0 and J3-J5 take the measured rest values.

Per environment, over the first `--steps` policy steps: tool-tip error against time, the fastest arm
joint, the share of steps an arm joint sits at its torque limit, contact force on the dog's trunk, head,
hips and thighs (the arm's links report none, so a strike is read on what it strikes),
base roll/pitch and drift, falls. Two time series (the centre goal, both starts) are saved for a figure.

Simulation of UniFP's port, which drives the arm with UniFP's own PD -- not the D1's firmware position
servo, and faster than it (F-097). Timings here do not transfer to the bench; the direction and order of
the motion, and whether it hits anything, are what this can say.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent

#: The folded rest pose the real arm reported with the dog sitting (F-027 era, `results/findings.md`),
#: J0-J5 in degrees. J1/J2 lie past the URDF's +/-90 degree limits and are clamped below.
FOLDED_REST_DEG = (2.1, -90.9, 91.7, -3.1, 4.1, 3.1)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", default="checkpoints/unifp_go2d1_isaaclab_cad333_model_58400.pt")
    parser.add_argument("--radius", type=float, nargs="+", default=[0.35, 0.45, 0.55])
    parser.add_argument("--pitch_deg", type=float, nargs="+", default=[0.0, 20.0, 40.0])
    parser.add_argument("--yaw_deg", type=float, nargs="+", default=[-30.0, 0.0, 30.0])
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--hold_s", type=float, default=1.0,
                        help="seconds standing with the arm held at its start pose before the policy takes over")
    parser.add_argument("--tool_point", choices=("jaw_centre", "fingertip"), default="fingertip",
                        help="fingertip is what the reach checkpoints are scored on (checkpoints/README.md)")
    parser.add_argument("--run_name", default="folded_start")
    parser.add_argument("--output", default=str(ROOT / "logs/unifp_demos"))
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    sys.path.insert(0, str(ROOT))
    import torch  # noqa: E402
    from datetime import datetime, timezone  # noqa: E402

    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # noqa: E402
    from weld import build_welded_robot_usd  # noqa: E402
    from unifp_isaaclab import interface, robot as robot_mod  # noqa: E402
    from unifp_train import task_cfg  # noqa: E402
    from unifp_train.env import Go2D1PosForceEnv  # noqa: E402
    from unifp_train.env_cfg import Go2D1PosForceEnvCfg  # noqa: E402
    from unifp_train.eval import EvaluablePolicy  # noqa: E402

    goals = [(r, math.radians(p), math.radians(y))
             for r in args.radius for p in args.pitch_deg for y in args.yaw_deg]
    count = len(goals)
    starts = ("zero", "folded")

    robot_usd = build_welded_robot_usd(
        go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
        d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
        out_usd_path=str(ROOT / "generated/go2_d1.usd"),
        arm_mass_kg=3.152).usd_path

    cfg = Go2D1PosForceEnvCfg()
    cfg.scene.num_envs = 2 * count
    cfg.seed = args.seed
    cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot")
    cfg.robot_usd = robot_usd
    cfg.force_start_step = 10 ** 9
    if args.tool_point == "fingertip":
        cfg.tool_body, cfg.tool_offset_m = interface.TOOL_BODY, interface.TOOL_OFFSET_M
    env = Go2D1PosForceEnv(cfg)
    env.reset()
    device = env.device

    policy = EvaluablePolicy(args.checkpoint, device=str(device))
    held = torch.tensor(goals + goals, dtype=torch.float32, device=device)
    env._goals.step = lambda: held          # type: ignore[assignment]
    for buffer in ("current", "start", "goal"):
        setattr(env._goals, buffer, held.clone())

    # Place the arm. Envs [0, count) keep the zero pose, [count, 2*count) are folded.
    arm_ids = env._order[12:interface.NUM_ACTIONS]
    limits = torch.tensor(task_cfg.JOINT_POS_LIMITS[12:18], device=device)
    folded = torch.deg2rad(torch.tensor(FOLDED_REST_DEG, device=device))
    folded = torch.maximum(torch.minimum(folded, limits[:, 1] - 0.005), limits[:, 0] + 0.005)
    joint_pos = env._robot.data.default_joint_pos.clone()
    joint_pos[count:, arm_ids] = folded
    env._robot.write_joint_state_to_sim(joint_pos, torch.zeros_like(joint_pos))
    env._robot.set_joint_position_target(joint_pos)

    # On the robot the dog is already standing when the policy is switched on. Training's reset drops it
    # from the spawn height instead, so without this both starts would include that landing. Hold every
    # joint at its start pose through the env's own control law for `--hold_s`, then hand over to the
    # policy with the buffers a fresh episode has: no previous action, an empty history.
    hold = torch.zeros(2 * count, interface.NUM_ACTIONS, device=device)
    hold[count:, 12:18] = folded / interface.ACTION_SCALE
    with torch.no_grad():
        for _ in range(int(args.hold_s / interface.POLICY_DT)):
            env.step(hold)
            env._commands[:, :3] = 0.0
    env._actions.zero_()
    env._last_actions.zero_()
    env._last_dof_vel[:] = env._dof()[1]
    env._actor_history.reset(None)
    env._critic_history.reset(None)
    obs = env._get_observations()
    held_arm_deg = torch.rad2deg(env._dof()[0][:, 12:18])

    names = list(env._robot.body_names)
    sensor_names = list(env._contact_sensor.body_names)
    # The weld gives the arm's links no contact reporting, so the arm striking the dog is read from the
    # dog's side: the trunk, the head and the hips and thighs, none of which touch the ground while standing.
    body_sensor = [i for i, n in enumerate(sensor_names)
                  if n in ("base", "Head_upper", "Head_lower") or n.endswith(("_hip", "_thigh"))]
    trunk_sensor = [i for i, n in enumerate(sensor_names) if n == "base"]
    torque_limit = torch.tensor(interface.TORQUE_LIMITS[12:18], device=device)
    spawn_xy = env._robot.data.root_pos_w[:, :2].clone()

    T = args.steps
    err = torch.zeros(T, 2 * count, device=device)
    q_arm = torch.zeros(T, 2 * count, 6, device=device)
    qd_arm = torch.zeros(T, 2 * count, 6, device=device)
    saturated = torch.zeros(T, 2 * count, 6, dtype=torch.bool, device=device)
    body_contact = torch.zeros(T, 2 * count, device=device)
    body_which = torch.zeros(T, 2 * count, dtype=torch.long, device=device)
    trunk_contact = torch.zeros(T, 2 * count, device=device)
    tilt = torch.zeros(T, 2 * count, 2, device=device)
    drift = torch.zeros(T, 2 * count, device=device)
    tip_z = torch.zeros(T, 2 * count, device=device)
    first_actions = None
    fell = torch.zeros(2 * count, dtype=torch.bool, device=device)
    with torch.no_grad():
        for step in range(T):
            if not simulation_app.is_running():
                break
            action = policy.act(obs["policy"])
            if first_actions is None:
                first_actions = action[:, 12:18].clone()
            obs, _, terminated, _, _ = env.step(action)
            fell |= terminated
            env._commands[:, :3] = 0.0
            tip = env._tip_pos()
            err[step] = torch.norm(tip - env._goal_world(), dim=1)
            tip_z[step] = tip[:, 2]
            pos, vel = env._dof()
            q_arm[step] = pos[:, 12:18]
            qd_arm[step] = vel[:, 12:18]
            torque = env._robot.data.applied_torque[:, arm_ids]
            saturated[step] = torque.abs() >= 0.98 * torque_limit
            forces = env._contact_sensor.data.net_forces_w.norm(dim=-1)
            body_contact[step], which = forces[:, body_sensor].max(dim=1)
            body_which[step] = which
            trunk_contact[step] = forces[:, trunk_sensor].max(dim=1).values if trunk_sensor else 0.0
            roll_pitch = interface.body_roll_pitch(env._robot.data.root_quat_w)
            tilt[step] = torch.rad2deg(roll_pitch)
            drift[step] = torch.norm(env._robot.data.root_pos_w[:, :2] - spawn_xy, dim=1)

    dt = interface.POLICY_DT

    def first_below(threshold):
        """Seconds until the error first drops below `threshold` and stays there for 0.5 s; NaN if never."""
        below = (err < threshold).float()
        hold = int(0.5 / dt)
        out = []
        for e in range(2 * count):
            t = float("nan")
            for s in range(T - hold):
                if below[s:s + hold, e].all():
                    t = s * dt
                    break
            out.append(t)
        return out

    reach5, reach3 = first_below(0.05), first_below(0.03)
    settle = slice(max(0, T - 100), T)
    records = []
    for e in range(2 * count):
        r, p, y = goals[e % count]
        sat = saturated[:, e]
        records.append({
            "start": starts[e // count], "radius": r, "pitch_deg": math.degrees(p), "yaw_deg": math.degrees(y),
            "time_to_5cm_s": reach5[e], "time_to_3cm_s": reach3[e],
            "error_final_m": float(err[settle, e].median()),
            "peak_arm_speed_rad_s": float(qd_arm[:, e].abs().max()),
            "peak_speed_by_joint_rad_s": qd_arm[:, e].abs().max(dim=0).values.tolist(),
            "saturated_share_by_joint": sat.float().mean(dim=0).tolist(),
            "saturated_steps_first_2s_by_joint": sat[:int(2.0 / dt)].sum(dim=0).tolist(),
            "peak_body_contact_n": float(body_contact[:, e].max()),
            "peak_body_contact_s": float(body_contact[:, e].argmax()) * dt,
            "peak_body_contact_body": sensor_names[body_sensor[int(body_which[body_contact[:, e].argmax(), e])]],
            "body_contact_steps_over_5n": int((body_contact[:, e] > 5.0).sum()),
            "body_contact_bodies_over_5n": sorted({sensor_names[body_sensor[int(i)]]
                                                   for i in body_which[body_contact[:, e] > 5.0, e].tolist()}),
            "arm_q_at_peak_contact_deg": torch.rad2deg(q_arm[body_contact[:, e].argmax(), e]).tolist(),
            "peak_trunk_contact_n": float(trunk_contact[:, e].max()),
            "peak_roll_deg": float(tilt[:, e, 0].abs().max()),
            "peak_pitch_deg": float(tilt[:, e, 1].abs().max()),
            "base_drift_m": float(drift[:, e].max()),
            "tip_z_min_m": float(tip_z[:, e].min()),
            "first_arm_action": first_actions[e].tolist(),
            "arm_q_at_handover_deg": held_arm_deg[e].tolist(),
            "arm_q_final_deg": torch.rad2deg(q_arm[settle, e].median(dim=0).values).tolist(),
            "fell": bool(fell[e]),
        })

    def pooled(start, key, fn):
        vals = [r[key] for r in records if r["start"] == start and not math.isnan(r[key])]
        return fn(vals) if vals else None

    med = lambda v: sorted(v)[len(v) // 2]
    summary = {}
    for start in starts:
        rs = [r for r in records if r["start"] == start]
        summary[start] = {
            "envs": len(rs),
            "falls": sum(r["fell"] for r in rs),
            "reached_5cm": sum(not math.isnan(r["time_to_5cm_s"]) for r in rs),
            "time_to_5cm_s_median": pooled(start, "time_to_5cm_s", med),
            "time_to_5cm_s_max": pooled(start, "time_to_5cm_s", max),
            "time_to_3cm_s_median": pooled(start, "time_to_3cm_s", med),
            "error_final_cm_median": 100 * med([r["error_final_m"] for r in rs]),
            "error_final_cm_max": 100 * max(r["error_final_m"] for r in rs),
            "peak_arm_speed_rad_s_median": med([r["peak_arm_speed_rad_s"] for r in rs]),
            "peak_arm_speed_rad_s_max": max(r["peak_arm_speed_rad_s"] for r in rs),
            "peak_J1_speed_rad_s_max": max(r["peak_speed_by_joint_rad_s"][1] for r in rs),
            "peak_J2_speed_rad_s_max": max(r["peak_speed_by_joint_rad_s"][2] for r in rs),
            "saturated_steps_first_2s_J1_median": med([r["saturated_steps_first_2s_by_joint"][1] for r in rs]),
            "saturated_steps_first_2s_J2_median": med([r["saturated_steps_first_2s_by_joint"][2] for r in rs]),
            "peak_body_contact_n_max": max(r["peak_body_contact_n"] for r in rs),
            "envs_body_contact_over_5n": sum(r["peak_body_contact_n"] > 5.0 for r in rs),
            "peak_trunk_contact_n_max": max(r["peak_trunk_contact_n"] for r in rs),
            "peak_pitch_deg_max": max(r["peak_pitch_deg"] for r in rs),
            "peak_roll_deg_max": max(r["peak_roll_deg"] for r in rs),
            "base_drift_m_max": max(r["base_drift_m"] for r in rs),
            "first_arm_action_median": [med([r["first_arm_action"][j] for r in rs]) for j in range(6)],
        }

    centre = min(range(count), key=lambda i: abs(goals[i][0] - 0.45) + abs(goals[i][1] - math.radians(20))
                 + abs(goals[i][2]))
    series = {
        "dt": dt, "goal": list(goals[centre]),
        **{f"{s}_{k}": v[:, centre + count * n].tolist()
           for n, s in enumerate(starts)
           for k, v in (("error_m", err), ("arm_q_rad", q_arm), ("arm_qd_rad_s", qd_arm),
                        ("body_contact_n", body_contact), ("pitch_deg", tilt[:, :, 1]))},
    }

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    run_dir = Path(args.output).resolve() / f"{stamp}_probe_seed{args.seed}_{args.run_name}"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "folded_start.json").write_text(json.dumps({"records": records, "series": series}, indent=2))
    run_json = {
        "mode": "folded_start_probe", "status": "finished", "seed": args.seed,
        "checkpoint": os.path.abspath(args.checkpoint),
        "checkpoint_source": policy.source, "checkpoint_iteration": policy.iteration,
        "steps": T, "hold_s": args.hold_s, "goals": count, "tool_point": args.tool_point,
        "forces": "off", "base_command": "zero (standing)",
        "folded_pose_rad": folded.tolist(), "folded_rest_measured_deg": list(FOLDED_REST_DEG),
        "body_contact_bodies": [sensor_names[i] for i in body_sensor],
        "summary": summary,
        "records_file": "folded_start.json",
        "command": " ".join(sys.argv),
    }
    (run_dir / "run.json").write_text(json.dumps(run_json, indent=2))
    print(json.dumps(run_json, indent=2))
    print(f"\nwritten to {run_dir}")
    env.close()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

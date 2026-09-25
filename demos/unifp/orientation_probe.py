#!/usr/bin/env python3
"""Where do the jaws point when UniFP settles on a goal?

    ./demos/unifp/orientation_probe.py --checkpoint checkpoints/unifp_go2d1_isaaclab_model_56000.pt --headless

UniFP's command carries a tool-tip **position** and nothing else: `CMD_EE_ORN_R/P/Y` are declared
in the command vector and never written by the task (`unifp_isaaclab/interface.py`). The arm has
six joints and the goal constrains three numbers, so the remaining three are whatever the policy
learnt -- and a grasp is decided by exactly those three. `workspace_map.py` measures how close the
tip gets; this measures which way the hand is facing when it gets there.

Same method as the workspace map: one environment per goal, the goal held still for the whole run,
standing (velocity command zeroed every step), forces off, read after the arm has settled. What is
added is the hand frame, taken from body poses rather than from joint angles so it needs no
forward-kinematics of its own:

  * **approach** -- unit vector from Link6's origin to the controlled tool point. Which way the
    hand is pointed.
  * **jaw** -- unit vector from Link7_1's origin to Link7_2's, the direction the fingers open
    along. A pair of jaws closes *across* this axis.

Both are reported in the base's yaw-only frame, the frame the goal itself is commanded in, so a
number here does not depend on which way the robot happens to be facing. Two derived angles say
whether a grasp is on:

  * `approach_elevation_deg` -- above (+) or below (-) horizontal. A top-down grasp wants about -90.
  * `jaw_tilt_deg` -- the jaw axis's angle from horizontal. A top-down grasp onto an upright cup,
    or a grip across a horizontal lever bar, wants about 0.

Nothing here is a demo and nothing is grasped: it is the feasibility measurement that comes before
one. A steady hand frame that happens to suit the object means a scripted grasp can be built on
this policy; a hand frame that wanders, or points the wrong way everywhere, means it cannot.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--radius", type=float, nargs="+", default=[0.35, 0.45, 0.55])
    parser.add_argument("--pitch_deg", type=float, nargs="+",
                        default=[-20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0])
    parser.add_argument("--yaw_deg", type=float, nargs="+", default=[-30.0, -15.0, 0.0, 15.0, 30.0])
    parser.add_argument("--steps", type=int, default=600)
    parser.add_argument("--settle", type=int, default=400)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--robot_usd", default=None)
    parser.add_argument("--run_name", default="orientation_probe")
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
    from unifp_train.env import Go2D1PosForceEnv  # noqa: E402
    from unifp_train.env_cfg import Go2D1PosForceEnvCfg  # noqa: E402
    from unifp_train.eval import EvaluablePolicy  # noqa: E402

    points = [(r, math.radians(p), math.radians(y))
              for r in args.radius for p in args.pitch_deg for y in args.yaw_deg]

    robot_usd = args.robot_usd or build_welded_robot_usd(
        go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
        d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
        out_usd_path=str(ROOT / "generated/go2_d1.usd"),
        mount_pos=(0.0, 0.0, 0.08), arm_mass_kg=3.152).usd_path

    cfg = Go2D1PosForceEnvCfg()
    cfg.scene.num_envs = len(points)
    cfg.seed = args.seed
    cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot")
    cfg.robot_usd = robot_usd
    cfg.force_start_step = 10 ** 9          # position tracking alone, as the workspace map does
    env = Go2D1PosForceEnv(cfg)
    obs, _ = env.reset()

    policy = EvaluablePolicy(args.checkpoint, device=str(env.device))
    held = torch.tensor(points, dtype=torch.float32, device=env.device)
    env._goals.step = lambda: held          # type: ignore[assignment]
    for buffer in ("current", "start", "goal"):
        setattr(env._goals, buffer, held.clone())

    names = list(env._robot.body_names)
    wrist, finger_a, finger_b = (names.index(n) for n in ("Link6", "Link7_1", "Link7_2"))

    errors, approaches, jaws, tips = [], [], [], []
    fell = torch.zeros(len(points), dtype=torch.bool, device=env.device)
    with torch.no_grad():
        for step in range(args.steps):
            if not simulation_app.is_running():
                break
            obs, _, terminated, _, _ = env.step(policy.act(obs["policy"]))
            fell |= terminated
            env._commands[:, :3] = 0.0      # stand still
            if step < args.settle:
                continue
            tip = env._tip_pos()
            errors.append(torch.norm(tip - env._goal_world(), dim=1).clone())
            pos = env._robot.data.body_pos_w
            yaw = env._base_yaw_quat()
            # Both directions in the yaw-only base frame: the frame the goal is commanded in.
            approach = tip + env.scene.env_origins - pos[:, wrist]
            jaw = pos[:, finger_b] - pos[:, finger_a]
            unit = lambda v: v / v.norm(dim=-1, keepdim=True).clamp(min=1e-9)
            approaches.append(unit(interface.quat_rotate_inverse(yaw, approach)).clone())
            jaws.append(unit(interface.quat_rotate_inverse(yaw, jaw)).clone())
            tips.append(interface.quat_rotate_inverse(yaw, tip).clone())

    def median(stack):
        return torch.stack(stack).median(dim=0).values.cpu()

    error_med = median(errors)
    error_sd = torch.stack(errors).std(dim=0).cpu()
    approach_med, jaw_med = median(approaches), median(jaws)
    # Spread of a direction over time, as the largest angle from its own median.
    def wobble(stack, med):
        s = torch.stack(stack)
        cos = (s * med.to(s.device)).sum(-1).clamp(-1.0, 1.0)
        return torch.rad2deg(torch.acos(cos)).max(dim=0).values.cpu()

    approach_wobble = wobble(approaches, approach_med)
    jaw_wobble = wobble(jaws, jaw_med)
    tip_med = median(tips)
    fell_cpu = fell.cpu()

    records = []
    for i, (r, p, y) in enumerate(points):
        a, j = approach_med[i], jaw_med[i]
        records.append({
            "radius": r, "pitch_deg": math.degrees(p), "yaw_deg": math.degrees(y),
            "goal_base_m": interface.sphere2cart(torch.tensor([r, p, y])).tolist(),
            "tip_base_m": tip_med[i].tolist(),
            "error_m": float(error_med[i]), "error_sd_m": float(error_sd[i]),
            "approach_unit": a.tolist(), "jaw_unit": j.tolist(),
            "approach_elevation_deg": math.degrees(math.asin(max(-1.0, min(1.0, float(a[2]))))),
            "approach_azimuth_deg": math.degrees(math.atan2(float(a[1]), float(a[0]))),
            "jaw_tilt_deg": math.degrees(math.asin(abs(max(-1.0, min(1.0, float(j[2])))))),
            "jaw_azimuth_deg": math.degrees(math.atan2(float(j[1]), float(j[0]))),
            "approach_wobble_deg": float(approach_wobble[i]),
            "jaw_wobble_deg": float(jaw_wobble[i]),
            "fell": bool(fell_cpu[i]),
        })

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    run_dir = Path(args.output).resolve() / f"{stamp}_probe_seed{args.seed}_{args.run_name}"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "orientation_probe.json").write_text(json.dumps({"records": records}, indent=2))

    med = lambda vals: sorted(vals)[len(vals) // 2] if vals else None
    run_json = {
        "mode": "orientation_probe", "status": "finished", "seed": args.seed,
        "checkpoint": os.path.abspath(args.checkpoint),
        "checkpoint_source": policy.source, "checkpoint_iteration": policy.iteration,
        "steps": args.steps, "settle": args.settle, "grid_points": len(points),
        "forces": "off", "base_command": "zero (standing)",
        "error_m_median": med([r["error_m"] for r in records]),
        "approach_elevation_deg_median": med([r["approach_elevation_deg"] for r in records]),
        "approach_elevation_deg_min": min(r["approach_elevation_deg"] for r in records),
        "approach_elevation_deg_max": max(r["approach_elevation_deg"] for r in records),
        "jaw_tilt_deg_median": med([r["jaw_tilt_deg"] for r in records]),
        "jaw_tilt_deg_min": min(r["jaw_tilt_deg"] for r in records),
        "jaw_tilt_deg_max": max(r["jaw_tilt_deg"] for r in records),
        "approach_wobble_deg_max": max(r["approach_wobble_deg"] for r in records),
        "jaw_wobble_deg_max": max(r["jaw_wobble_deg"] for r in records),
        "falls": int(fell_cpu.sum()),
        "records_file": "orientation_probe.json",
        "command": " ".join(sys.argv),
    }
    (run_dir / "run.json").write_text(json.dumps(run_json, indent=2))

    print(f"\n{len(points)} goals, measured over steps {args.settle}-{args.steps}\n")
    for r in args.radius:
        print(f"radius {r:.2f} m -- approach elevation (deg, -90 = straight down) / jaw tilt from "
              f"horizontal (deg); rows = goal pitch, columns = goal yaw")
        print(f"{'pitch':>7}" + "".join(f"{y:>14.0f}" for y in args.yaw_deg))
        for p in args.pitch_deg:
            cells = ""
            for y in args.yaw_deg:
                rec = records[points.index((r, math.radians(p), math.radians(y)))]
                cells += f"{rec['approach_elevation_deg']:>7.0f}/{rec['jaw_tilt_deg']:<6.0f}"
            print(f"{p:>7.0f}{cells}")
        print()
    print(json.dumps({k: v for k, v in run_json.items() if k != "records_file"}, indent=2))
    print(f"\nwritten to {run_dir}")
    env.close()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

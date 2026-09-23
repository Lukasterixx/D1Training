#!/usr/bin/env python3
"""Tool-tip tracking error as a function of *where* the goal is.

    ./unifp_train/workspace_map.py --checkpoint <ckpt> --headless
    ./unifp_train/workspace_map.py --checkpoint <ckpt> --headless --radius 0.35 0.45 0.55

The frozen-manifest evaluation reports one median per episode, and an episode sweeps the goal all
over the workspace, so a region the policy cannot reach is averaged in with regions it handles
perfectly and never appears. This holds the goal still instead: one environment per grid point,
the same goal for the whole run, and the steady-state error read off after the arm has settled.

The goal is spherical about a centre that rides with the base (`EE_GOAL_CENTER_OFFSET`, 0.49 m up)
and **pitch is elevation**, so the bottom of the pitch range is a goal low and in front of the
robot -- at radius 0.5 and pitch -45 degrees, 35 cm forward and 14 cm off the ground.

Grid points the task's own goal generator would refuse -- below `EE_GOAL_UNDERGROUND_LIMIT`, or
inside the keep-out box around the body -- are marked and excluded from the summary, because
failing to reach a goal that is never commanded is not a fault.

The base is held at a zero velocity command and external forces are off, so what is measured is
arm tracking from a standing robot and nothing else.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def grid(radii, pitch_steps, yaw_steps):
    """Every (radius, pitch, yaw) in the sweep, as a list, coarsest axis first."""
    from unifp_isaaclab import interface

    pitch_low, pitch_high = interface.EE_GOAL_PITCH_RANGE
    yaw_low, yaw_high = interface.EE_GOAL_YAW_RANGE
    pitches = [pitch_low + (pitch_high - pitch_low) * i / (pitch_steps - 1) for i in range(pitch_steps)]
    yaws = [yaw_low + (yaw_high - yaw_low) * i / (yaw_steps - 1) for i in range(yaw_steps)]
    return [(r, p, y) for r in radii for p in pitches for y in yaws], pitches, yaws


def reachable(points):
    """True where the task's own generator would be willing to place this goal."""
    import torch

    from unifp_isaaclab import interface

    sphere = torch.tensor(points, dtype=torch.float32)
    cart = interface.sphere2cart(sphere)
    upper = torch.tensor(interface.EE_GOAL_COLLISION_UPPER)
    lower = torch.tensor(interface.EE_GOAL_COLLISION_LOWER)
    inside_box = ((cart > lower) & (cart < upper)).all(dim=-1)
    underground = cart[:, 2] < interface.EE_GOAL_UNDERGROUND_LIMIT
    return (~inside_box & ~underground), cart


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--radius", type=float, nargs="+", default=[0.35, 0.45, 0.55])
    parser.add_argument("--pitch_steps", type=int, default=9)
    parser.add_argument("--yaw_steps", type=int, default=9)
    parser.add_argument("--steps", type=int, default=600, help="Under the 1000-step episode limit.")
    parser.add_argument("--settle", type=int, default=400, help="Measure after this many steps.")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--robot_usd", default=None)
    parser.add_argument("--out", default=None)
    parser.add_argument("--run_name", default="workspace_map")
    parser.add_argument("--output", default=str(ROOT / "logs/unifp_train"))

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

    points, pitches, yaws = grid(args.radius, args.pitch_steps, args.yaw_steps)
    ok, cart = reachable(points)

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
    # Forces off: this isolates position tracking. A goal the arm cannot reach unforced is not
    # going to be reached when something is also pushing on the gripper.
    cfg.force_start_step = 10 ** 9
    env = Go2D1PosForceEnv(cfg)
    obs, _ = env.reset()

    policy = EvaluablePolicy(args.checkpoint, device=str(env.device))
    held = torch.tensor(points, dtype=torch.float32, device=env.device)

    # Hold the goal by replacing the generator's step rather than by writing its buffers: the
    # generator also runs timers and resamples, and a held goal must survive all of that.
    env._goals.step = lambda: held  # type: ignore[assignment]
    env._goals.current = held.clone()
    env._goals.start = held.clone()
    env._goals.goal = held.clone()

    errors: list[torch.Tensor] = []
    offsets: list[torch.Tensor] = []
    saturation: list[torch.Tensor] = []
    limits = torch.tensor(__import__("unifp_train.task_cfg", fromlist=["x"]).JOINT_POS_LIMITS,
                          device=env.device)
    arm_low = limits[12:interface.NUM_ACTIONS, 0]
    arm_high = limits[12:interface.NUM_ACTIONS, 1]
    fell = torch.zeros(len(points), dtype=torch.bool, device=env.device)
    with torch.no_grad():
        for step in range(args.steps):
            if not simulation_app.is_running():
                break
            action = policy.act(obs["policy"])
            obs, _, terminated, truncated, _ = env.step(action)
            fell |= terminated
            # Stand still: the velocity command is zeroed every step so the base does not walk
            # away from the goal it is being scored against.
            env._commands[:, :3] = 0.0
            if step >= args.settle:
                tip = env._tip_pos()
                errors.append(torch.norm(tip - env._goal_world(), dim=1).clone())
                # Where the tip sits relative to the goal, in the base's yaw frame: a policy that
                # simply cannot reach shows a steady offset, one that flops shows a large spread.
                offsets.append(interface.quat_rotate_inverse(
                    env._base_yaw_quat(), tip - env._goal_world()).clone())
                # How close the arm joints are to their travel. "Cannot reach" and "will not
                # reach" look identical in the error alone and completely different here.
                arm = env._dof()[0][:, 12:interface.NUM_ACTIONS]
                span = (arm_high - arm_low).clamp(min=1e-6)
                saturation.append((2.0 * (arm - arm_low) / span - 1.0).abs().max(dim=1).values.clone())

    stacked = torch.stack(errors)                     # (measured steps, N)
    median = stacked.median(dim=0).values.cpu()
    spread = stacked.std(dim=0).cpu()
    offset = torch.stack(offsets).median(dim=0).values.cpu()
    sat = torch.stack(saturation).median(dim=0).values.cpu()
    fell_cpu = fell.cpu()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    run_dir = Path(args.output).resolve() / f"{stamp}_map_seed{args.seed}_{args.run_name}"
    run_dir.mkdir(parents=True, exist_ok=True)

    records = []
    for i, (r, p, y) in enumerate(points):
        records.append({"radius": r, "pitch": p, "yaw": y,
                        "pitch_deg": math.degrees(p), "yaw_deg": math.degrees(y),
                        "cart": cart[i].tolist(), "generator_would_command": bool(ok[i]),
                        "error_m": float(median[i]), "error_sd_m": float(spread[i]),
                        "offset_base_m": offset[i].tolist(),
                        "arm_joint_saturation": float(sat[i]),
                        "fell": bool(fell_cpu[i])})
    report = {"checkpoint": os.path.abspath(args.checkpoint),
              "checkpoint_source": policy.source, "checkpoint_iteration": policy.iteration,
              "steps": args.steps, "settle": args.settle, "seed": args.seed,
              "radii": args.radius, "pitch_steps": args.pitch_steps, "yaw_steps": args.yaw_steps,
              "forces": "off", "base_command": "zero (standing)",
              "grid_points": len(points), "records": records}
    (run_dir / "workspace_map.json").write_text(json.dumps(report, indent=2))

    # `./dashboard.py record` keys off run.json, so every run this repository launches writes one.
    live = [rec for rec in records if rec["generator_would_command"]]
    def median(values):
        ordered = sorted(values)
        return ordered[len(ordered) // 2] if ordered else None
    low = [rec for rec in live if rec["pitch_deg"] < -40]
    rest = [rec for rec in live if rec["pitch_deg"] >= -40]
    run_json = {k: v for k, v in report.items() if k != "records"}
    run_json.update({
        "mode": "workspace_map", "status": "finished",
        "commandable_goals": len(live),
        "error_m_median": median([r["error_m"] for r in live]),
        "error_m_max": max((r["error_m"] for r in live), default=None),
        "error_sd_m_median": median([r["error_sd_m"] for r in live]),
        "lowest_pitch_error_m_median": median([r["error_m"] for r in low]),
        "above_lowest_pitch_error_m_median": median([r["error_m"] for r in rest]),
        "falls": sum(1 for r in live if r["fell"]),
        "records_file": "workspace_map.json",
    })
    (run_dir / "run.json").write_text(json.dumps(run_json, indent=2))

    print(f"\ngrid {len(points)} points; measured over steps {args.settle}-{args.steps}")
    for r in args.radius:
        print(f"\nradius {r:.2f} m -- median tool-tip error in cm, rows = pitch (elevation), "
              f"columns = yaw; '.' = the generator would never command this goal")
        head = "".join(f"{math.degrees(y):>7.0f}" for y in yaws)
        print(f"{'pitch':>7}{head}")
        for p in pitches:
            cells = ""
            for y in yaws:
                i = points.index((r, p, y))
                cells += "      ." if not ok[i] else f"{median[i] * 100:>7.1f}"
            print(f"{math.degrees(p):>7.0f}{cells}")
    worst = sorted(live, key=lambda rec: -rec["error_m"])[:6]
    print("\nworst commandable goals:")
    for rec in worst:
        dx, dy, dz = (c * 100 for c in rec["offset_base_m"])
        print(f"  radius {rec['radius']:.2f}  pitch {rec['pitch_deg']:>5.0f}d  "
              f"yaw {rec['yaw_deg']:>5.0f}d  ->  {rec['error_m'] * 100:6.1f} cm"
              f"  sd {rec['error_sd_m'] * 100:5.1f}  offset ({dx:+.1f},{dy:+.1f},{dz:+.1f})"
              f"  joint sat {rec['arm_joint_saturation']:.2f}"
              f"{'   FELL' if rec['fell'] else ''}")
    print(f"\nwritten to {run_dir / 'workspace_map.json'}")
    env.close()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Launch the Thesis B starter task. Use `check` before `smoke` or `train`."""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def dependencies():
    packages = {}
    for name in ("torch", "isaacsim", "isaaclab", "isaaclab-rl", "rsl-rl-lib"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    return packages


def preflight():
    report = {"python": sys.version.split()[0], "executable": sys.executable, "packages": dependencies()}
    try:
        import torch
        report["cuda_available"] = torch.cuda.is_available()
        report["gpu_count"] = torch.cuda.device_count()
    except ImportError:
        report["cuda_available"] = False
        report["gpu_count"] = 0
    report["d1_urdf_exists"] = (ROOT / "d1_arm/d1.urdf").is_file()
    # The fresh actor/critic configuration deliberately targets the installed 5.x API.
    report["rsl_rl_5_api"] = (report["packages"]["rsl-rl-lib"] or "").startswith("5.")
    report["ready_for_gpu_smoke"] = bool(
        all(report["packages"].values()) and report["cuda_available"]
        and report["d1_urdf_exists"] and report["rsl_rl_5_api"]
    )
    print(json.dumps(report, indent=2))
    return report


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def git_output(*args):
    result = subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None


def snapshot_sources(run_dir):
    hashes = {}
    sources = list((ROOT / "position_only").glob("*.py"))
    sources += [ROOT / name for name in ("run_position_only.py", "flat_env_cfg.py", "weld.py", "d1_arm/d1.urdf",
                                         "motor_model.py", "unitree_actuators.py")]
    for source in sources:
        relative = source.relative_to(ROOT)
        dest = run_dir / "source" / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
        hashes[str(relative)] = hashlib.sha256(source.read_bytes()).hexdigest()
    hashes["d1_meshes"] = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT / "d1_arm/meshes").glob("*"))
    }
    return hashes


def run(args, report):
    # Isaac modules must only be imported after AppLauncher has started the app.
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=args.headless, device=args.device).app
    env = None
    try:
        import torch
        from isaaclab.envs import ManagerBasedRLEnv
        from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
        from isaaclab.utils.io import dump_yaml
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
        from rsl_rl.runners import OnPolicyRunner

        from position_only.agent import make_agent_cfg
        from position_only.deploy import export_deploy_cfg
        from position_only.env_cfg import CONTROLLED_NAMES, CRITIC_OBS_DIM, POLICY_OBS_DIM, make_cfg
        from weld import build_welded_robot_usd

        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        run_dir = Path(args.output).resolve() / f"{stamp}_{args.mode}_seed{args.seed}"
        run_dir.mkdir(parents=True, exist_ok=False)
        metadata = {
            "status": "initializing", "mode": args.mode, "seed": args.seed,
            "preflight": report, "arguments": vars(args),
            "git_commit": git_output("rev-parse", "HEAD"), "git_status": git_output("status", "--short"),
            "source_sha256": snapshot_sources(run_dir), "action_joint_order": CONTROLLED_NAMES,
            "task": "free_space_stance_reach", "force_inputs": False,
            "controlled_point": {"body": "Link6", "offset_m": args.tip_offset},
            "policy_hz": 50, "physics_hz": 200,
            "sim2real": {"leg_actuator": args.leg_actuator, "robustness": args.robustness,
                         "self_collisions": args.self_collisions, "solver_iterations": [8, 4],
                         "actor_observations": "no base linear velocity", "critic_observations": "privileged"},
        }
        write_json(run_dir / "run.json", metadata)
        try:
            if args.robot_usd:
                robot_path = str(Path(args.robot_usd).resolve())
            else:
                robot_path = build_welded_robot_usd(
                    go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
                    d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
                    out_usd_path=str(run_dir / "assets/go2_d1.usd"),
                    mount_pos=(0.0, 0.0, 0.08), arm_mass_kg=3.152,
                ).usd_path
            metadata["robot_usd"] = robot_path
            metadata["robot_usd_sha256"] = hashlib.sha256(Path(robot_path).read_bytes()).hexdigest()
            cfg = make_cfg(robot_path, args.num_envs, args.seed, args.device, args.tip_offset,
                           leg_actuator=args.leg_actuator, robustness=args.robustness,
                           self_collisions=args.self_collisions)
            cfg.commands.ee_position.debug_vis = not args.headless
            cfg.log_dir = str(run_dir)
            agent_cfg = make_agent_cfg(args.seed, args.device, args.iterations)
            dump_yaml(str(run_dir / "env.yaml"), cfg)
            write_json(run_dir / "agent.json", agent_cfg)
            env = RslRlVecEnvWrapper(ManagerBasedRLEnv(cfg=cfg), clip_actions=1.0)
            robot = env.unwrapped.scene["robot"]
            if env.num_actions != 18 or len(robot.joint_names) != 20:
                raise RuntimeError("Expected 18 actions and one welded articulation with 20 joints.")
            resolved = env.unwrapped.action_manager.get_term("legs")._joint_names
            resolved = resolved + env.unwrapped.action_manager.get_term("arm")._joint_names
            if resolved != CONTROLLED_NAMES:
                raise RuntimeError(f"Unexpected action order: {resolved}")
            arm_bodies = env.unwrapped.scene["arm_contact"].body_names
            if not {"Link6", "Link7_1", "Link7_2"}.issubset(set(arm_bodies)):
                raise RuntimeError(f"Arm sensor did not resolve the end effector: {arm_bodies}")
            obs = env.get_observations()
            for group, width in (("policy", POLICY_OBS_DIM), ("critic", CRITIC_OBS_DIM)):
                if tuple(obs[group].shape) != (args.num_envs, width):
                    raise RuntimeError(f"Unexpected {group} observation shape: {obs[group].shape}")
            metadata["observed_joint_names"] = list(robot.joint_names)
            metadata["arm_sensor_bodies"] = list(arm_bodies)
            metadata["articulation_mass_kg"] = float(robot.root_physx_view.get_masses()[0].sum())
            metadata["observation_width"] = {"policy": POLICY_OBS_DIM, "critic": CRITIC_OBS_DIM}
            deploy = export_deploy_cfg(env.unwrapped, run_dir / "params" / "deploy.yaml")
            metadata["deploy_manifest"] = {"path": "params/deploy.yaml", "format": deploy["format"]}
            metadata["checkpoint"] = str(Path(args.checkpoint).resolve()) if args.checkpoint else None
            if args.checkpoint:
                metadata["checkpoint_sha256"] = hashlib.sha256(Path(args.checkpoint).read_bytes()).hexdigest()
            runner = None
            if args.mode == "train" or args.checkpoint:
                runner = OnPolicyRunner(env, deepcopy(agent_cfg), log_dir=str(run_dir), device=args.device)
                if args.checkpoint:
                    runner.load(args.checkpoint)
            if args.mode == "train":
                runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=False)
                metadata["status"] = "training_finished"
            else:
                policy = runner.get_inference_policy(device=args.device) if runner else None
                terminated = truncated = 0
                for step in range(args.steps):
                    if not app.is_running():
                        raise RuntimeError("Simulator closed before the requested smoke steps completed.")
                    with torch.inference_mode():
                        action = policy(obs) if policy else torch.zeros(args.num_envs, 18, device=args.device)
                        if not torch.isfinite(action).all():
                            raise RuntimeError(f"Non-finite action at step {step}.")
                        obs, reward, done, extras = env.step(action)
                        if not torch.isfinite(obs["policy"]).all() or not torch.isfinite(reward).all():
                            raise RuntimeError(f"Non-finite observation/reward at step {step}.")
                        terminated += int(env.unwrapped.reset_terminated.sum())
                        truncated += int(env.unwrapped.reset_time_outs.sum())
                diagnostic = {
                    "steps": args.steps, "transitions": args.steps * args.num_envs,
                    "failure_resets": terminated, "time_limit_resets": truncated,
                    "controller": "checkpoint" if policy else "zero_actions_default_joint_targets",
                    "final_position_error_m": env.unwrapped.command_manager.get_term("ee_position").error_m.cpu().tolist(),
                    "interpretation": "Interface smoke diagnostics only; not a baseline evaluation or success rate.",
                }
                write_json(run_dir / "smoke.json", diagnostic)
                metadata["status"] = "smoke_finished"
            print(f"[position_only] {metadata['status']}: {run_dir}")
        except Exception as exc:
            metadata["status"] = "failed"
            metadata["error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            write_json(run_dir / "run.json", metadata)
    finally:
        if env is not None:
            env.close()
        app.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("check", "smoke", "train"))
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--num_envs", type=int, default=64)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--iterations", type=int, default=100, help="Additional PPO iterations; starts fresh unless --checkpoint is set.")
    parser.add_argument("--steps", type=int, default=600, help="Number of policy steps for smoke mode.")
    parser.add_argument("--tip_offset", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "Z"))
    parser.add_argument("--leg_actuator", choices=("unitree", "dc_motor"), default="unitree",
                        help="Go2 leg motor model: unitree_rl_lab's measured envelope, or Isaac Lab's stock DCMotor.")
    parser.add_argument("--robustness", choices=("none", "unitree"), default="none",
                        help="'unitree' adds unitree_rl_lab's observation noise and randomisation.")
    parser.add_argument("--self_collisions", action="store_true",
                        help="Let the arm collide with the Go2 body. Check the weld for overlaps first.")
    parser.add_argument("--checkpoint", help="Explicit checkpoint from this task, for resuming training or smoke playback.")
    parser.add_argument("--robot_usd", help="Existing local welded USD; otherwise rebuild inside the new run directory.")
    parser.add_argument("--output", default=str(ROOT / "logs/position_only"))
    args = parser.parse_args()
    if min(args.num_envs, args.iterations, args.steps) <= 0:
        parser.error("Environment count, iterations and steps must be positive.")
    for name in ("checkpoint", "robot_usd"):
        value = getattr(args, name)
        if value and not Path(value).is_file():
            parser.error(f"--{name} must point to an existing local file.")
    report = preflight()
    if args.mode == "check":
        return 0 if report["ready_for_gpu_smoke"] else 1
    if not report["ready_for_gpu_smoke"]:
        parser.exit(1, "GPU simulation prerequisites are unavailable. Activate env_isaaclab and check NVIDIA device access.\n")
    if not args.device.startswith("cuda"):
        parser.error("This starter launch path targets GPU simulation; use --device cuda:0.")
    run(args, report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

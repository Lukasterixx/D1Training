"""Launch the Thesis B starter task. Use `check` before `smoke` or `train`.

Modes: `check` (prerequisites, no simulator), `manifest` (write a frozen evaluation manifest, no
simulator), `smoke`, `verify`, `view`, `train`, and `eval` (run a frozen manifest under one
controller and report the G1a measurement; without `--checkpoint` it evaluates zero actions, which
is the baseline every reach number is read against).
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

from position_only.task_space import SPAWN_HEIGHT_M
from position_only.tool_point import TOOL_BODY, TOOL_OFFSET_M

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
    print(json.dumps(report, indent=2), flush=True)
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


POSTURE_FIELDS = ("base_height_m", "tilt_deg", "base_drift_m", "leg_joint_dev_rad", "arm_joint_dev_rad",
                  "leg_torque_nm", "arm_contact_n", "base_x_m", "base_y_m", "base_yaw_deg")
SETTLE_STEPS = 50  # 1 s at 50 Hz: posture statistics skip the drop onto the feet.


def posture_sample(env, robot, leg_ids, arm_ids):
    """Per-environment stance quantities after one policy step, as (num_envs, len(POSTURE_FIELDS))."""
    import torch

    data = robot.data
    offset = data.root_pos_w - env.scene.env_origins
    # Largest net contact force on any D1 link: ground or, with --self_collisions, the Go2's own body.
    arm_contact = torch.linalg.vector_norm(env.scene["arm_contact"].data.net_forces_w, dim=-1).amax(dim=-1)
    joint_dev = (data.joint_pos - data.default_joint_pos).abs()
    return torch.stack([
        offset[:, 2],
        torch.rad2deg(torch.acos((-data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))),
        torch.linalg.vector_norm(offset[:, :2], dim=-1),
        joint_dev[:, leg_ids].amax(dim=-1),
        joint_dev[:, arm_ids].amax(dim=-1),
        data.applied_torque[:, leg_ids].abs().amax(dim=-1),
        arm_contact,
        offset[:, 0],
        offset[:, 1],
        torch.rad2deg(2.0 * torch.atan2(data.root_quat_w[:, 3], data.root_quat_w[:, 0])),  # yaw, for a near-level base
    ], dim=-1)


def write_posture(run_dir, samples):
    """Write the env-mean trace per step and return min/mean/max per field after settling."""
    import csv

    import torch

    samples = torch.stack(samples).cpu()  # (steps, envs, fields)
    with (run_dir / "smoke_trace.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("step", *POSTURE_FIELDS))
        for step, row in enumerate(samples.mean(dim=1).tolist()):
            writer.writerow((step, *(f"{value:.6g}" for value in row)))
    settled = samples[min(SETTLE_STEPS, len(samples) - 1):]
    return {
        name: {"min": float(settled[..., i].min()), "mean": float(settled[..., i].mean()),
               "max": float(settled[..., i].max())}
        for i, name in enumerate(POSTURE_FIELDS)
    }


EPISODE_CSV_FIELDS = (
    "index", "success", "survived", "fell", "truncated", "recorded_s", "max_dwell_s", "time_to_reach_s",
    "final_error_m", "rms_error_m", "p95_error_m", "min_base_height_m", "max_tilt_deg",
    "final_base_x_from_spawn_m", "final_base_y_from_spawn_m", "max_base_horizontal_travel_m",
    "joint_limit_steps_frac", "arm_commanded_effort_at_limit_frac", "peak_arm_joint_torque_nm",
    "leg_effort_saturated_frac", "peak_leg_torque_nm",
)


def _write_episode_csv(path, records):
    import csv

    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(EPISODE_CSV_FIELDS + ("target_x_m", "target_y_m", "target_z_m",
                                              "transient_mean_m", "final_2s_mean_m"))
        for record in records:
            row = [record[name] for name in EPISODE_CSV_FIELDS]
            row += list(record["target_env_frame_m"])
            row += [(record["transient_error"] or {}).get("mean_m"),
                    (record["final_2s_error"] or {}).get("mean_m")]
            writer.writerow("" if value is None else value for value in row)


def _planner_condition(arm_term):
    """The firmware planner a running arm action uses, in the manifest's form (F-039: values, not labels)."""
    plan = arm_term.cfg.trajectory
    if plan is None or arm_term.planned_targets is None:
        return None
    return {key: [round(float(v), 6) for v in plan[key]] if key == "velocity_limits_rad_s" else round(float(plan[key]), 6)
            for key in ("dead_time_s", "accel_rad_s2", "decel_rad_s2", "replan_velocity_retention",
                        "velocity_limits_rad_s")}


def view(args, app, env, runner, obs, metadata, run_dir):
    """Run episodes in real time in the viewer until the window closes, printing each episode's reach state."""
    import torch

    unwrapped = env.unwrapped
    command = unwrapped.command_manager.get_term("ee_position")
    robot = unwrapped.scene["robot"]
    policy = runner.get_inference_policy(device=args.device) if runner else None

    # Replaying one recorded episode rather than watching fresh samples. The command term resamples
    # its target on every reset, so the manifest's target is re-pinned each step rather than once.
    pinned = None
    if args.manifest and args.episode is not None:
        from position_only.manifest import load as load_manifest

        replay = load_manifest(args.manifest)
        if not 0 <= args.episode < len(replay["episodes"]):
            raise SystemExit(f"--episode {args.episode} is outside the manifest's 0..{len(replay['episodes']) - 1}")
        target = replay["episodes"][args.episode]["target_env_frame_m"]
        pinned = torch.tensor(target, dtype=torch.float32, device=args.device)
        metadata["view_replay"] = {"manifest": str(Path(args.manifest).resolve()),
                                   "manifest_sha256": replay["content_sha256"],
                                   "role": replay["role"], "episode": args.episode,
                                   "target_env_frame_m": list(target)}
        print(f"\n[view] Replaying {replay['role']} episode {args.episode}, target "
              f"({target[0]:.3f}, {target[1]:+.3f}, {target[2]:.3f}) m from the environment origin. "
              f"Every episode below uses this one target.", flush=True)
    snapshot_step = int(round(9.0 / unwrapped.step_dt))  # 9 s: settled, before the 10 s time limit
    episodes = []
    metadata["status"] = "viewing"
    write_json(run_dir / "run.json", metadata)
    print("\n[view] Green target: the pincer tip is already within 5 cm. Red: farther. Blue dot: pincer tip. "
          "Orange wireframe: target box. Grey plate: where the robot is spawned.\n"
          "[view] Close the window or press Ctrl+C to stop.\n", flush=True)
    try:
        while app.is_running():
            started = time.perf_counter()
            with torch.inference_mode():
                if pinned is not None:
                    command.target_w[:] = unwrapped.scene.env_origins + pinned
                action = policy(obs) if policy else torch.zeros(args.num_envs, 18, device=args.device)
                obs, _, _, _ = env.step(action)
                if int(unwrapped.episode_length_buf[0]) == snapshot_step:
                    error = command.error_m
                    slide = robot.data.root_pos_w[:, 0] - unwrapped.scene.env_origins[:, 0]
                    episode = {"episode": len(episodes) + 1, "within_5cm": int((error <= 0.05).sum()),
                               "envs": args.num_envs, "mean_error_m": float(error.mean()),
                               "mean_base_x_from_spawn_m": float(slide.mean())}
                    episodes.append(episode)
                    # Saved as they happen, so a forced close keeps every finished episode.
                    metadata["view"] = {"controller": "checkpoint" if policy else "zero_actions_default_joint_targets",
                                        "episodes": episodes}
                    write_json(run_dir / "run.json", metadata)
                    print(f"[view] episode {episode['episode']}: {episode['within_5cm']}/{args.num_envs} targets already "
                          f"within 5 cm at 9 s; mean error {100 * episode['mean_error_m']:.1f} cm; base "
                          f"{-100 * episode['mean_base_x_from_spawn_m']:.1f} cm behind its spawn point", flush=True)
            time.sleep(max(0.0, unwrapped.step_dt - (time.perf_counter() - started)))
    except KeyboardInterrupt:
        pass
    metadata["status"] = "view_closed"
    metadata["view"] = {"controller": "checkpoint" if policy else "zero_actions_default_joint_targets",
                        "episodes": episodes}


def run(args, report):
    # Isaac modules must only be imported after AppLauncher has started the app.
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=args.headless, device=args.device).app
    env = None
    failed = False
    try:
        import torch
        from isaaclab.envs import ManagerBasedRLEnv
        from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
        from isaaclab.utils.io import dump_yaml
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
        from rsl_rl.runners import OnPolicyRunner

        from position_only.agent import make_agent_cfg
        from position_only.deploy import export_deploy_cfg
        from position_only.env_cfg import (ARM_NAMES, CONTROLLED_NAMES, CRITIC_OBS_DIM, LEG_NAMES, POLICY_OBS_DIM,
                                           make_cfg)
        from motor_model import interface_timing
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
            "controlled_point": {"body": args.tip_body, "offset_m": list(args.tip_offset)},
            "policy_hz": 50, "physics_hz": 200,
            "sim2real": {"leg_actuator": args.leg_actuator, "arm_actuator": args.arm_actuator,
                         "latency": args.latency, "robustness": args.robustness,
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
            spawn_kwargs = {} if args.spawn_height is None else {"spawn_height": args.spawn_height}
            cfg = make_cfg(robot_path, args.num_envs, args.seed, args.device, args.tip_offset, args.tip_body,
                           leg_actuator=args.leg_actuator, robustness=args.robustness,
                           self_collisions=args.self_collisions, latency=args.latency,
                           arm_actuator=args.arm_actuator, arm_trajectory=args.arm_trajectory, **spawn_kwargs)
            metadata["reset"] = {"spawn_height_m": cfg.scene.robot.init_state.pos[2]}
            metadata["target_box_env_frame_m"] = [list(axis) for axis in cfg.commands.ee_position.ranges]
            timing = interface_timing(args.latency, 1.0 / (cfg.sim.dt * cfg.decimation), args.leg_actuator)
            metadata["sim2real"]["timing"] = timing
            # The firmware planner as the action term was actually configured, not as the flag names it.
            metadata["sim2real"]["arm_trajectory"] = {"profile": args.arm_trajectory,
                                                      "resolved": cfg.actions.arm.trajectory}
            cfg.commands.ee_position.debug_vis = not args.headless
            if args.mode == "view":
                # Side-on to environment 0, from the robot's right: base, arm and target box in one frame.
                cfg.viewer.origin_type, cfg.viewer.env_index = "env", 0
                cfg.viewer.eye, cfg.viewer.lookat = (0.25, -1.7, 0.75), (0.15, 0.0, 0.5)
            cfg.log_dir = str(run_dir)
            agent_cfg = make_agent_cfg(args.seed, args.device, args.iterations)
            # Reward weight overrides: applied before env.yaml is written, so the dump records what trained.
            overrides = {}
            for term, weight in args.reward_weight:
                if not hasattr(cfg.rewards, term):
                    raise ValueError(f"--reward_weight: no reward term {term!r}")
                overrides[term] = {"default": getattr(cfg.rewards, term).weight, "used": weight}
                getattr(cfg.rewards, term).weight = weight
            metadata["reward_overrides"] = overrides
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
            # The limits PhysX actually applies, whatever the URDF import or actuator cfg intended.
            # Explicit leg actuators leave torque to their own model (PhysX gets a huge cap) but
            # inherit the USD's joint speed limits.
            arm_ids = [robot.joint_names.index(name) for name in ARM_NAMES]
            leg_ids = [robot.joint_names.index(name) for name in LEG_NAMES]
            for key, ids in (("arm_limits_in_physx", arm_ids), ("leg_limits_in_physx", leg_ids)):
                metadata["sim2real"][key] = {
                    "effort_nm": robot.data.joint_effort_limits[0, ids].tolist(),
                    "velocity_rad_s": robot.data.joint_vel_limits[0, ids].tolist(),
                }
            # Drive type as authored on the stage: acceleration drives scale the gains by joint inertia.
            from pxr import UsdPhysics
            stage = env.unwrapped.sim.stage
            metadata["sim2real"]["drive_type_in_usd"] = {
                str(prim.GetName()): UsdPhysics.DriveAPI(prim, "angular").GetTypeAttr().Get()
                for prim in stage.Traverse()
                if str(prim.GetPath()).startswith("/World/envs/env_0/Robot/")
                and prim.GetName() in ARM_NAMES + LEG_NAMES[:1] and prim.HasAPI(UsdPhysics.DriveAPI, "angular")
            }
            deploy = export_deploy_cfg(env.unwrapped, run_dir / "params" / "deploy.yaml", timing=timing)
            metadata["deploy_manifest"] = {"path": "params/deploy.yaml", "format": deploy["format"]}
            metadata["checkpoint"] = str(Path(args.checkpoint).resolve()) if args.checkpoint else None
            if args.checkpoint:
                metadata["checkpoint_sha256"] = hashlib.sha256(Path(args.checkpoint).read_bytes()).hexdigest()
            runner = None
            # eval with a checkpoint needs the runner too; without one it evaluates zero actions.
            if args.mode == "train" or args.checkpoint:
                runner = OnPolicyRunner(env, deepcopy(agent_cfg), log_dir=str(run_dir), device=args.device)
                if args.checkpoint:
                    runner.load(args.checkpoint)
            if args.mode == "train":
                runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=False)
                metadata["status"] = "training_finished"
            elif args.mode == "view":
                view(args, app, env, runner, obs, metadata, run_dir)
            elif args.mode == "verify":
                from position_only.verify import run_checks

                result = run_checks(env.unwrapped)
                write_json(run_dir / "verify.json", result)
                metadata["status"] = "verify_passed" if result["all_passed"] else "verify_failed"
                metadata["verify_failed"] = result["failed"]
            elif args.mode == "arm_steps":
                from position_only.arm_steps import run_arm_steps

                result = run_arm_steps(env.unwrapped)
                write_json(run_dir / "arm_steps.json", result)
                metadata["status"] = "arm_steps_finished"
                metadata["arm_steps"] = {"planner_active": result["planner_active"],
                                         "legs_with_two_commands": sum(len(l["command_steps"]) == 2 for l in result["legs"]),
                                         "resets": sum(l["reset_during_trace"] for l in result["legs"])}
            elif args.mode == "eval":
                from position_only.evaluate import run_episodes, summarise
                from position_only.manifest import load as load_manifest

                manifest = load_manifest(args.manifest)
                policy = runner.get_inference_policy(device=args.device) if runner else None
                # What this run actually ran under, for the manifest to object to.
                conditions = {
                    "spawn_height_m": cfg.scene.robot.init_state.pos[2],
                    "target_box_env_frame_m": [list(a) for a in cfg.commands.ee_position.ranges],
                    "robustness": args.robustness, "latency": args.latency,
                    "leg_actuator": args.leg_actuator, "arm_actuator": args.arm_actuator,
                    "self_collisions": args.self_collisions,
                    "tool_body": args.tip_body, "tool_offset_m": list(args.tip_offset),
                    "episode_length_s": cfg.episode_length_s,
                    # Read from the live articulation and the resolved config, not re-derived from
                    # motor_model: the manifest froze that source's values, so asking it again would
                    # compare it with itself. These are what PhysX and the env actually got (F-039).
                    "policy_hz": round(1.0 / (cfg.sim.dt * cfg.decimation), 6),
                    "arm_velocity_limits_rad_s": [round(v, 6) for v in
                                                  metadata["sim2real"]["arm_limits_in_physx"]["velocity_rad_s"]],
                    "arm_effort_limits_nm": [round(v, 6) for v in
                                             metadata["sim2real"]["arm_limits_in_physx"]["effort_nm"]],
                    "leg_delay_physics_steps": list(metadata["sim2real"]["timing"]["leg_delay_physics_steps"]),
                    "arm_command_hold_steps": metadata["sim2real"]["timing"]["arm_command_hold_steps"],
                    "arm_feedback_period_steps": metadata["sim2real"]["timing"]["arm_feedback_period_steps"],
                    # From the live action term: the planner the arm is actually running, or None.
                    "arm_trajectory": _planner_condition(env.unwrapped.action_manager.get_term("arm")),
                }
                started = time.monotonic()
                records = run_episodes(
                    env, manifest, policy=policy, device=args.device,
                    progress=lambda done, total: print(f"[position_only] evaluated {done}/{total} episodes",
                                                       flush=True))
                result = summarise(
                    records, manifest,
                    controller="checkpoint" if policy else "zero_actions_default_joint_targets",
                    conditions=conditions)
                result["wall_time_s"] = round(time.monotonic() - started, 2)
                result["checkpoint"] = metadata.get("checkpoint")
                result["checkpoint_sha256"] = metadata.get("checkpoint_sha256")
                result["manifest_path"] = str(Path(args.manifest).resolve())
                write_json(run_dir / "eval.json", result)
                write_json(run_dir / "eval_episodes.json", records)
                _write_episode_csv(run_dir / "eval_episodes.csv", records)
                if result["condition_mismatches"]:
                    print("[position_only] WARNING: run does not match the manifest's conditions:", flush=True)
                    for line in result["condition_mismatches"]:
                        print(f"  {line}", flush=True)
                metadata["status"] = "eval_finished"
                metadata["eval"] = {
                    "manifest_role": manifest["role"], "episodes": result["episodes"],
                    "success_rate": result["success"]["rate"], "fall_rate": result["falls"]["rate"],
                    "g1a_passed": result["g1a"]["passed"], "controller": result["controller"],
                    "condition_mismatches": result["condition_mismatches"],
                }
            else:
                policy = runner.get_inference_policy(device=args.device) if runner else None
                terminated = truncated = 0
                samples = []
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
                        samples.append(posture_sample(env.unwrapped, robot, leg_ids, arm_ids))
                diagnostic = {
                    "steps": args.steps, "transitions": args.steps * args.num_envs,
                    "failure_resets": terminated, "time_limit_resets": truncated,
                    # Sampled after each policy step, so a reset step shows the reset state.
                    "posture_after_settling": write_posture(run_dir, samples),
                    "settle_steps": SETTLE_STEPS,
                    "controller": "checkpoint" if policy else "zero_actions_default_joint_targets",
                    "final_position_error_m": env.unwrapped.command_manager.get_term("ee_position").error_m.cpu().tolist(),
                    "interpretation": "Interface smoke diagnostics only; not a baseline evaluation or success rate.",
                }
                write_json(run_dir / "smoke.json", diagnostic)
                metadata["status"] = "smoke_finished"
            print(f"[position_only] {metadata['status']}: {run_dir}", flush=True)
        except Exception as exc:
            metadata["status"] = "failed"
            metadata["error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            write_json(run_dir / "run.json", metadata)
    except BaseException:
        failed = True
        traceback.print_exc()
        raise
    finally:
        if env is not None:
            env.close()
        sys.stdout.flush()
        sys.stderr.flush()
        # SimulationApp.close() ends the process with status 0 before Python can report an
        # exception, so a failed run would look like a success. Exit nonzero ourselves instead.
        if failed:
            os._exit(1)
        app.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("check", "smoke", "verify", "view", "train", "eval", "manifest", "arm_steps"),
                        help="verify: deliberate interface-timing, termination and partial-reset checks (>= 6 envs). "
                             "view: real-time episodes in the viewer with target, tip and box markers, until closed.")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--num_envs", type=int, default=64)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--iterations", type=int, default=100, help="Additional PPO iterations; starts fresh unless --checkpoint is set.")
    parser.add_argument("--steps", type=int, default=600, help="Number of policy steps for smoke mode.")
    parser.add_argument("--tip_body", default=TOOL_BODY,
                        help="Body carrying the controlled point (default: the Link7_1 pincer). Link6 is the wrist.")
    parser.add_argument("--tip_offset", type=float, nargs=3, default=TOOL_OFFSET_M, metavar=("X", "Y", "Z"),
                        help="Controlled point in --tip_body's link frame, metres (default: the pincer tip).")
    parser.add_argument("--leg_actuator", choices=("unitree", "dc_motor"), default="unitree",
                        help="Go2 leg motor model: unitree_rl_lab's measured envelope, or Isaac Lab's stock DCMotor.")
    parser.add_argument("--arm_actuator", choices=("d1_servo", "implicit"), default="d1_servo",
                        help="'d1_servo': force drives with the D1's published torque and URDF speed limits; 'implicit': the URDF import's acceleration drives.")
    parser.add_argument("--arm_trajectory", choices=("measured", "none"), default="measured",
                        help="'measured': setpoints pass through the D1 firmware's fitted motion planner "
                             "(10 ms dead time, trapezoid, replan from rest; F-045). 'none': straight to the drive.")
    parser.add_argument("--latency", choices=("estimated", "none"), default="estimated",
                        help="'estimated': 0-10 ms leg command delay; D1 commands and feedback at 10 Hz.")
    parser.add_argument("--robustness", choices=("none", "unitree"), default="none",
                        help="'unitree' adds unitree_rl_lab's observation noise and randomisation.")
    parser.add_argument("--self_collisions", action=argparse.BooleanOptionalAction, default=True,
                        help="Let the arm collide with the Go2 body (default). Off, the arm passes through the trunk.")
    parser.add_argument("--checkpoint", help="Explicit checkpoint from this task, for resuming training or smoke playback.")
    parser.add_argument("--reward_weight", action="append", default=[], metavar="TERM=WEIGHT",
                        type=lambda s: (s.split("=", 1)[0], float(s.split("=", 1)[1])),
                        help="Override one reward term's weight, e.g. action_rate=-0.1. Repeatable; recorded in run.json.")
    parser.add_argument("--manifest", help="Evaluation manifest JSON (eval mode), or its output path (manifest mode).")
    parser.add_argument("--role", choices=("development", "validation", "test"), default="development",
                        help="Manifest role to build (manifest mode).")
    parser.add_argument("--episodes", type=int, default=100, help="Episodes in a new manifest (manifest mode).")
    parser.add_argument("--episode", type=int, default=None,
                        help="View mode: replay this episode index from --manifest instead of fresh targets.")
    parser.add_argument("--spawn_height", type=float, default=None,
                        help="Base height at reset (m). Default: the task's standing spawn, env_cfg.SPAWN_HEIGHT_M.")
    parser.add_argument("--robot_usd", help="Existing local welded USD; otherwise rebuild inside the new run directory.")
    parser.add_argument("--output", default=str(ROOT / "logs/position_only"))
    args = parser.parse_args()
    if min(args.num_envs, args.iterations, args.steps) <= 0:
        parser.error("Environment count, iterations and steps must be positive.")
    if args.mode == "view" and args.headless:
        parser.error("view needs the viewer; drop --headless.")
    if args.mode == "arm_steps" and (args.num_envs != 6 or args.checkpoint):
        parser.error("arm_steps steps one joint per environment: --num_envs 6 and no checkpoint.")
    if args.mode == "verify" and (args.num_envs < 6 or args.checkpoint):
        parser.error("verify needs --num_envs 6 or more (one per induced termination plus a control) and no checkpoint.")
    if args.episode is not None:
        if args.mode != "view":
            parser.error("--episode replays one manifest episode in view mode; it does nothing elsewhere.")
        if not args.manifest:
            parser.error("--episode needs --manifest to take the episode from.")
    if args.mode == "view" and args.manifest and not Path(args.manifest).is_file():
        parser.error(f"--manifest must point to an existing file: {args.manifest}")
    if args.mode == "eval":
        if not args.manifest:
            parser.error("eval needs --manifest pointing at a frozen manifest.")
        if not Path(args.manifest).is_file():
            parser.error(f"--manifest must point to an existing file: {args.manifest}")
    for name in ("checkpoint", "robot_usd"):
        value = getattr(args, name)
        if value and not Path(value).is_file():
            parser.error(f"--{name} must point to an existing local file.")
    if args.mode == "manifest":
        from position_only.manifest import build, write

        if not args.manifest:
            parser.error("manifest mode needs --manifest to say where to write the file.")
        path = Path(args.manifest)
        if path.exists():
            parser.error(f"{path} already exists. A frozen manifest is not regenerated in place; "
                         "delete it deliberately if you really mean to replace it.")
        spawn = SPAWN_HEIGHT_M if args.spawn_height is None else args.spawn_height
        manifest = build(args.role, episodes=args.episodes, spawn_height=spawn,
                         robustness=args.robustness, latency=args.latency,
                         leg_actuator=args.leg_actuator, arm_actuator=args.arm_actuator,
                         self_collisions=args.self_collisions)
        write(manifest, path)
        print(f"[position_only] wrote {manifest['role']} manifest: {path} "
              f"({manifest['episode_count']} episodes, sha256 {manifest['content_sha256'][:12]})", flush=True)
        return 0

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

"""Run a UniFP checkpoint on the Isaac Lab welded Go2+D1.

One policy step is, in order:

    1. assemble the 15 commands (velocity, the moving end-effector goal, the force commands)
    2. advance the gait phase -- pinned to 0 while the velocity command is inside its dead zone
    3. read the robot, build the 76-wide observation, push it onto the 32-frame history
    4. run the policy on the 2432-wide stack
    5. hold the resulting joint targets for four 200 Hz physics steps, recomputing torque each one

which is the order UniFP's `step()` and `post_physics_step()` produce, with the caveat recorded in
`policy.ObsHistory`: upstream's play loop feeds an all-zero observation on its very first control
step and this does not.

Everything imported below needs Isaac Sim to be running already, which is why this module is
imported from `run_unifp_isaaclab.py` after `AppLauncher` and not at the top of it.
"""
from __future__ import annotations

import json
import math
import os
import time

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from . import interface, robot as robot_mod, task
from .policy import ObsHistory, UniFPPolicy

TRACE_COLUMNS = (
    "step", "time_s",
    # The same first columns as unifp_go2d1/play_policy.py's trace, so the two are directly
    # comparable without a translation step.
    "tip_err_l1_m", "tip_err_x_m", "tip_err_y_m", "tip_err_z_m", "base_z_m",
    "force_cmd_n", "force_est_n",
    # Extra, because this side can afford them and they are what a sim-to-sim disagreement shows up in.
    "base_vx_mps", "base_vy_mps", "base_wz_radps", "base_roll_rad", "base_pitch_rad",
    "goal_radius_m", "goal_pitch_rad", "goal_yaw_rad",
    "tip_x_m", "tip_y_m", "tip_z_m", "action_absmax", "torque_absmax_nm",
)


@configclass
class UniFPSceneCfg(InteractiveSceneCfg):
    """Flat ground, one light, and the welded robot. No terrain, no sensors."""

    ground = AssetBaseCfg(prim_path="/World/ground", spawn=robot_mod.ground_cfg())
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9)),
    )
    robot = None  # filled in by build_scene


def build_scene(num_envs: int, robot_usd: str, spawn_height: float, env_spacing: float = 2.5):
    """The simulation context and scene, with UniFP's timestep."""
    sim_cfg = sim_utils.SimulationCfg(
        dt=interface.SIM_DT,
        render_interval=interface.DECIMATION,
        device="cuda:0" if torch.cuda.is_available() else "cpu",
    )
    sim = sim_utils.SimulationContext(sim_cfg)
    scene_cfg = UniFPSceneCfg(num_envs=num_envs, env_spacing=env_spacing)
    scene_cfg.robot = robot_mod.make_robot_cfg(
        robot_usd, prim_path="{ENV_REGEX_NS}/Robot", spawn_height=spawn_height)
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    return sim, scene


def reset_to_default(robot: Articulation, scene: InteractiveScene) -> None:
    """Put the robot in UniFP's reset pose and push it into PhysX.

    Spawning an articulation does not place its joints: the prim arrives at `init_state.pos` but
    the joints hold whatever the USD author left in them, which for `go2.usd` is all zeros -- legs
    straight. The robot then lands on straight legs, stands ~8 cm too tall, and the policy is
    driving a configuration it has never seen from the first step. It reads as the policy failing
    to transfer, which is why this is written out rather than left to the spawn.
    """
    root_state = robot.data.default_root_state.clone()
    root_state[:, :3] += scene.env_origins
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])
    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(),
                                   robot.data.default_joint_vel.clone())
    robot.reset()
    scene.write_data_to_sim()


def tip_position_w(robot: Articulation, body_index: int) -> torch.Tensor:
    """World position of the controlled point: the CAD pincer tip on `Link7_1`.

    UniFP indexes a dedicated `ee_gripper_link` body that `build_asset.py` welds onto `Link7_1`
    at `TOOL_OFFSET_M` with zero rotation, so this is the same point expressed as an offset
    rather than as a body. CAD, not measured on the arm (F-013).
    """
    pos = robot.data.body_pos_w[:, body_index]
    quat = robot.data.body_quat_w[:, body_index]
    offset = torch.as_tensor(interface.TOOL_OFFSET_M, dtype=pos.dtype, device=pos.device).expand_as(pos)
    return pos + interface.quat_apply(quat, offset)


def run(args, simulation_app) -> dict:
    """Roll the policy out and return a summary. Writes `trace.csv` and `run.json` under `--out`."""
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(args.seed)
    generator = torch.Generator(device=device).manual_seed(args.seed)

    sim, scene = build_scene(args.num_envs, args.robot_usd, args.spawn_height)
    robot: Articulation = scene["robot"]

    model = robot_mod.report_model(robot)
    model["control"] = {
        "action_scale": interface.ACTION_SCALE,
        "policy_rate_hz": round(1.0 / interface.POLICY_DT, 3),
        "physics_rate_hz": round(1.0 / interface.SIM_DT, 3),
        "decimation": interface.DECIMATION,
        "actuator": "IdealPD (explicit torque, UniFP's law)",
        # Recorded because it is the one deliberate departure from the training stack's numbers.
        "armature_kg_m2": robot_mod.ARM_ARMATURE,
        "self_collisions": True,
        "solver_iterations": list(robot_mod.SOLVER_ITERATIONS),
        "ground": {"static_friction": robot_mod.GROUND_STATIC_FRICTION,
                   "dynamic_friction": robot_mod.GROUND_DYNAMIC_FRICTION,
                   "restitution": robot_mod.GROUND_RESTITUTION,
                   "shape": "flat plane"},
        "spawn_height_m": args.spawn_height,
    }
    print(f"[unifp] {model['num_joints']} joints, {model['num_bodies']} bodies, "
          f"{model['total_mass_kg']:.3f} kg total ({model['arm_mass_kg']:.3f} kg arm)", flush=True)

    # The one thing that must not be wrong: UniFP's DOF order against this articulation's.
    order = torch.as_tensor(interface.permutation_from(list(robot.joint_names)), device=device)
    action_joint_ids = order[:interface.NUM_ACTIONS]
    jaw_joint_ids = order[interface.NUM_ACTIONS:]
    tip_body_index = robot.body_names.index(interface.TOOL_BODY)
    print(f"[unifp] DOF permutation (UniFP order -> articulation index): {order.tolist()}", flush=True)

    reset_to_default(robot, scene)
    print(f"[unifp] reset: base height {float(robot.data.root_pos_w[0, 2] - scene.env_origins[0, 2]):.3f} m, "
          f"joints at default", flush=True)

    policy = UniFPPolicy(args.checkpoint, device=device)
    history = ObsHistory(args.num_envs, device=device)
    goals = task.EeGoalTrajectory(args.num_envs, device=device, generator=generator)
    print(f"[unifp] loaded {os.path.basename(args.checkpoint)} (iteration {policy.iteration})", flush=True)

    default_pos = torch.as_tensor(interface.DEFAULT_DOF_POS, device=device)
    jaw_targets = default_pos[interface.NUM_ACTIONS:].expand(args.num_envs, -1).clone()
    gait_phase = torch.zeros(args.num_envs, device=device)
    prev_actions = torch.zeros(args.num_envs, interface.NUM_ACTIONS, device=device)
    commands = torch.zeros(args.num_envs, interface.NUM_COMMANDS, device=device)
    commands[:, :3] = torch.as_tensor(args.command, device=device)
    commands[:, interface.CMD_EE_FORCE] = torch.as_tensor(args.ee_force_cmd, device=device)

    external_force = torch.as_tensor(args.ee_force_ext, device=device).expand(args.num_envs, 3).clone()
    applying_external = bool(external_force.abs().sum() > 0)
    if applying_external:
        print(f"[unifp] applying an external force of {args.ee_force_ext} N at {interface.TOOL_BODY}",
              flush=True)

    trace = None
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        trace = open(os.path.join(args.out, "trace.csv"), "w")
        trace.write(",".join(TRACE_COLUMNS) + "\n")

    falls = 0
    tip_errors: list[float] = []
    started = time.time()

    for step in range(args.steps):
        if not simulation_app.is_running():
            break

        # 1. commands. The velocity and force commands are held; the goal walks.
        commands[:, interface.CMD_EE_RADIUS:interface.CMD_EE_YAW + 1] = goals.step()

        # 2. gait phase, advanced before the observation that reports it.
        gait_phase = interface.gait_step(gait_phase, commands)

        # 3. observation, in UniFP's DOF order.
        joint_pos = robot.data.joint_pos[:, order]
        joint_vel = robot.data.joint_vel[:, order]
        obs = interface.single_obs(
            robot.data.root_quat_w, robot.data.root_ang_vel_b,
            joint_pos, joint_vel, prev_actions, gait_phase, commands)
        obs = torch.clip(obs, -interface.CLIP_OBSERVATIONS, interface.CLIP_OBSERVATIONS)
        stacked = history.append(obs)

        # 4. policy. `--zero_actions` runs the same loop with the policy's output discarded,
        # which is the baseline every reach number in this repository is read against: it says
        # whether the robot stands up at all under UniFP's gains and default pose.
        actions = torch.clip(policy.act(stacked), -interface.CLIP_ACTIONS, interface.CLIP_ACTIONS)
        if args.zero_actions:
            actions = torch.zeros_like(actions)
        prev_actions = actions
        targets = interface.joint_targets(actions)

        robot.set_joint_position_target(targets, joint_ids=action_joint_ids)
        robot.set_joint_position_target(jaw_targets, joint_ids=jaw_joint_ids)

        # 5. four physics steps, torque recomputed at 200 Hz by the PD actuator.
        for _ in range(interface.DECIMATION):
            if applying_external:
                robot.set_external_force_and_torque(
                    forces=external_force.unsqueeze(1),
                    torques=torch.zeros_like(external_force).unsqueeze(1),
                    body_ids=[tip_body_index])
            scene.write_data_to_sim()
            sim.step()
            scene.update(interface.SIM_DT)

        # --- measurement, against the goal this step was actually run against ---
        tip = tip_position_w(robot, tip_body_index)
        yaw_q = interface.yaw_quat(interface.yaw_from_quat(robot.data.root_quat_w))
        goal_w = task.sphere_goal_to_world(goals.current, robot.data.root_pos_w, yaw_q)
        error = (tip - goal_w).abs()
        tip_errors.append(float(error.sum(dim=-1).mean()))
        base_z = robot.data.root_pos_w[:, 2] - scene.env_origins[:, 2]
        falls += int((base_z < 0.15).sum())

        if trace is not None or (args.report_every and step % args.report_every == 0):
            estimates = policy.estimates()
            roll_pitch = interface.body_roll_pitch(robot.data.root_quat_w)
            force_cmd = commands[0, interface.CMD_EE_FORCE].norm()
            force_est = estimates.ee_force[0].norm()

        if trace is not None:
            row = (
                step, step * interface.POLICY_DT,
                float(error[0].sum()), float(error[0, 0]), float(error[0, 1]), float(error[0, 2]),
                float(base_z[0]), float(force_cmd), float(force_est),
                float(robot.data.root_lin_vel_b[0, 0]), float(robot.data.root_lin_vel_b[0, 1]),
                float(robot.data.root_ang_vel_b[0, 2]),
                float(roll_pitch[0, 0]), float(roll_pitch[0, 1]),
                float(goals.current[0, 0]), float(goals.current[0, 1]), float(goals.current[0, 2]),
                float(tip[0, 0]), float(tip[0, 1]), float(tip[0, 2]),
                float(actions[0].abs().max()),
                float(robot.data.applied_torque[0].abs().max()),
            )
            trace.write(",".join(f"{v:.6g}" for v in row) + "\n")

        if args.report_every and step % args.report_every == 0:
            print(f"[{step:6d}] tip err L1 {float(error[0].sum()) * 100:5.1f} cm "
                  f"(xyz {float(error[0,0])*100:4.1f} {float(error[0,1])*100:4.1f} {float(error[0,2])*100:4.1f})"
                  f"  base z {float(base_z[0]):.3f} m"
                  f"  |F| cmd {float(force_cmd):4.1f} est {float(force_est):4.1f} N", flush=True)

    elapsed = time.time() - started
    if trace is not None:
        trace.close()

    # Per-joint final state. Cheap, twenty lines, and it is what actually localises a
    # disagreement between the two stacks: `torque_absmax_nm` in the trace is a maximum over all
    # twenty joints, so a jaw pressed against its 15 N stop hides whatever the legs are doing.
    final = []
    for position, index in enumerate(order.tolist()):
        final.append({
            "joint": interface.DOF_NAMES[position],
            "pos": round(float(robot.data.joint_pos[0, index]), 4),
            "target": round(float(robot.data.joint_pos_target[0, index]), 4),
            "default": round(interface.DEFAULT_DOF_POS[position], 4),
            "torque": round(float(robot.data.applied_torque[0, index]), 4),
            "effort_limit": round(interface.TORQUE_LIMITS[position], 3),
        })
    print("[unifp] final joint state (UniFP order):", flush=True)
    print(f"  {'joint':16s} {'pos':>9} {'target':>9} {'default':>9} {'torque':>9} {'limit':>8}", flush=True)
    for row in final:
        saturated = " SATURATED" if abs(row["torque"]) >= 0.999 * row["effort_limit"] else ""
        print(f"  {row['joint']:16s} {row['pos']:9.4f} {row['target']:9.4f} {row['default']:9.4f} "
              f"{row['torque']:9.4f} {row['effort_limit']:8.2f}{saturated}", flush=True)

    settled = tip_errors[len(tip_errors) // 2:] or tip_errors
    summary = {
        "steps": len(tip_errors),
        "wall_clock_s": round(elapsed, 2),
        "tip_err_l1_mean_m": round(sum(tip_errors) / len(tip_errors), 5) if tip_errors else None,
        "tip_err_l1_second_half_m": round(sum(settled) / len(settled), 5) if settled else None,
        "fall_steps": falls,
        "checkpoint": os.path.abspath(args.checkpoint),
        "checkpoint_iteration": policy.iteration,
        "command": list(args.command),
        "ee_force_cmd_n": list(args.ee_force_cmd),
        "ee_force_ext_n": list(args.ee_force_ext),
        "zero_actions": bool(args.zero_actions),
        "model": model,
        "final_joint_state": final,
    }
    if args.out:
        with open(os.path.join(args.out, "summary.json"), "w") as handle:
            json.dump(summary, handle, indent=2)
    return summary

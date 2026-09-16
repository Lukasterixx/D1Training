"""Deliberate G0 checks inside a live environment (`run_position_only.py verify`).

Each check induces one condition and states what it expects. A pass is evidence about that
mechanism only; none of it says anything about standing, stepping or reaching ability.

- Interface timing: arm targets and arm feedback change only on their sample steps, leg feedback
  every step, and arm velocity is the difference of consecutive angle samples.
- Terminations and partial resets: time limit, low base, tilt, base contact and workspace exit are
  each induced in their own environment. Only those environments reset, to the default state with a
  new target, while control environments carry on.
- Arm model: the simulator's arm link masses, Link6 and controlled-point kinematics and rest posture
  against the CPU model in `workspace.py`.
- Target box: measured against where the controlled point actually rests, so no target can be met
  without moving the arm.
"""
from __future__ import annotations

import math

import torch

from .env_cfg import ARM_NAMES
from .mdp import SUCCESS_RADIUS_M
from .task_space import ZERO_ACTION_TIP_M

ROLES = ("control", "time_limit", "low_base", "tilt", "base_contact", "workspace_exit")
EXPECTED_TERMS = {
    "control": set(), "time_limit": {"time_out"}, "low_base": {"low_base"}, "tilt": {"bad_orientation"},
    # On flat ground the base can only touch the ground when it is also below the 0.15 m height limit.
    "base_contact": {"base_contact", "low_base"}, "workspace_exit": {"outside_workspace"},
}
MIN_ENVS = len(ROLES)


def _result(name, passed, expected, observed):
    return {"name": name, "passed": bool(passed), "expected": expected, "observed": observed}


def _policy_spans(env):
    manager, spans, start = env.observation_manager, {}, 0
    for name, dims in zip(manager.active_terms["policy"], manager.group_obs_term_dim["policy"]):
        spans[name] = slice(start, start + dims[-1])
        start += dims[-1]
    return spans


def _change_steps(trace):
    """Steps k >= 1 at which any value differs from step k-1, per environment."""
    changed = (trace[1:] != trace[:-1]).any(dim=-1)  # (steps-1, envs)
    return [(changed[:, e].nonzero().squeeze(-1) + 1).tolist() for e in range(trace.shape[1])]


def _regular(steps, period):
    return len(steps) >= 2 and all(b - a == period for a, b in zip(steps, steps[1:])) and steps[0] <= period


def interface_timing(env, steps=31, amplitude=0.5, seed=0, settle_steps=60):
    robot = env.scene["robot"]
    arm = env.action_manager.get_term("arm")
    arm_ids = [robot.joint_names.index(name) for name in ARM_NAMES]
    spans = _policy_spans(env)
    policy_cfgs = dict(zip(env.observation_manager.active_terms["policy"],
                           env.observation_manager._group_obs_term_cfgs["policy"]))
    hold, period = int(arm.cfg.hold_steps), int(policy_cfgs["arm_joint_pos"].params["period_steps"])

    env.reset()
    action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
    # Settle onto the feet first: in the drop after a reset the legs can hold bit-identical angles.
    for _ in range(settle_steps):
        env.step(action)
    generator = torch.Generator(device=env.device).manual_seed(seed)
    record = {key: [] for key in ("target", "arm_pos", "arm_vel", "leg_pos", "true_arm", "done")}
    for _ in range(steps):
        # Legs hold their defaults; the arm gets a fresh random target every policy step.
        action[:, 12:] = amplitude * (2.0 * torch.rand(env.num_envs, 6, device=env.device, generator=generator) - 1.0)
        obs, _, terminated, truncated, _ = env.step(action)
        policy = obs["policy"]
        record["target"].append(arm.processed_actions.clone())
        record["arm_pos"].append(policy[:, spans["arm_joint_pos"]].clone())
        record["arm_vel"].append(policy[:, spans["arm_joint_vel"]].clone())
        record["leg_pos"].append(policy[:, spans["leg_joint_pos"]].clone())
        record["true_arm"].append(robot.data.joint_pos[:, arm_ids] - robot.data.default_joint_pos[:, arm_ids])
        record["done"].append(terminated | truncated)
    trace = {key: torch.stack(values).cpu() for key, values in record.items()}
    valid = [e for e in range(env.num_envs) if not trace["done"][:, e].any()]

    target_steps = _change_steps(trace["target"])
    feedback_steps = _change_steps(trace["arm_pos"])
    leg_steps = _change_steps(trace["leg_pos"])
    sample_error, velocity_error, velocity_held = 0.0, 0.0, True
    for e in valid:
        samples = feedback_steps[e]
        for k in samples:
            sample_error = max(sample_error, float((trace["arm_pos"][k, e] - trace["true_arm"][k, e]).abs().max()))
        for previous, k in zip(samples, samples[1:]):
            expected = (trace["arm_pos"][k, e] - trace["arm_pos"][previous, e]) / (period * env.step_dt)
            velocity_error = max(velocity_error, float((trace["arm_vel"][k, e] - expected).abs().max()))
            velocity_held &= bool((trace["arm_vel"][k + 1:k + period, e] == trace["arm_vel"][k, e]).all())

    checks = [
        _result("arm_command_hold", all(_regular(target_steps[e], hold) for e in valid) if hold > 1
                else all(target_steps[e] == list(range(1, steps)) for e in valid),
                f"arm targets change exactly every {hold} policy step(s)",
                {"change_steps_per_env": {e: target_steps[e] for e in valid}}),
        _result("arm_feedback_sampling", all(_regular(feedback_steps[e], period) for e in valid) if period > 1
                else all(feedback_steps[e] == list(range(1, steps)) for e in valid),
                f"arm angle feedback changes exactly every {period} policy step(s)",
                {"change_steps_per_env": {e: feedback_steps[e] for e in valid}}),
        _result("arm_feedback_is_current_at_sample", sample_error < 1e-5,
                "a fresh arm sample equals the simulator's joint angle at that step (< 1e-5 rad)",
                {"max_abs_error_rad": sample_error}),
        _result("arm_velocity_from_samples", velocity_error < 1e-3 and velocity_held,
                "arm velocity = (sample - previous sample) / sample interval, held between samples (< 1e-3 rad/s)",
                {"max_abs_error_rad_s": velocity_error, "held_between_samples": velocity_held}),
        _result("leg_feedback_every_step", all(leg_steps[e] == list(range(1, steps)) for e in valid),
                "leg joint angles change every policy step", {"envs_checked": valid}),
        _result("timing_trace_uninterrupted", len(valid) == env.num_envs,
                "no environment resets during the timing trace", {"envs_without_reset": valid}),
    ]
    # The first step each environment's interface updated on, modulo its period.
    phases = {"command": [target_steps[e][0] % hold if target_steps[e] else None for e in valid],
              "feedback": [feedback_steps[e][0] % period if feedback_steps[e] else None for e in valid]}

    actuator = robot.actuators["base_legs"]
    if hasattr(actuator, "positions_delay_buffer"):
        lags = actuator.positions_delay_buffer.time_lags.cpu().tolist()
        low, high = actuator.cfg.min_delay, actuator.cfg.max_delay
        checks.append(_result("leg_delay_in_range", all(low <= lag <= high for lag in lags),
                              f"per-env leg command delay within [{low}, {high}] physics steps", {"lags": lags}))
    return checks, {"hold_steps": hold, "feedback_period_steps": period, "phases": phases}


def _quat_about_x(angle):
    return (math.cos(angle / 2.0), math.sin(angle / 2.0), 0.0, 0.0)


def terminations_and_resets(env, settle_steps=60, after_steps=5):
    """Induce each termination in its own environment and check the partial reset that follows."""
    if env.num_envs < MIN_ENVS:
        raise ValueError(f"The termination check needs at least {MIN_ENVS} environments.")
    robot, command = env.scene["robot"], env.command_manager.get_term("ee_position")
    arm, device = env.action_manager.get_term("arm"), env.device
    roles = {e: (ROLES[e] if e < len(ROLES) else "control") for e in range(env.num_envs)}
    origins = env.scene.env_origins
    zero = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=device)

    env.reset()
    for _ in range(settle_steps):
        env.step(zero)
    before = {"length": env.episode_length_buf.clone(), "target": command.target_w.clone(),
              "root": robot.data.root_pos_w.clone()}
    settled_height = (robot.data.root_pos_w[:, 2] - origins[:, 2]).cpu().tolist()

    def place(env_id, offset, quat=(1.0, 0.0, 0.0, 0.0)):
        ids = torch.tensor([env_id], device=device)
        pose = torch.tensor([[*offset, *quat]], device=device)
        pose[:, :3] += origins[ids]
        robot.write_root_pose_to_sim(pose, env_ids=ids)
        robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=device), env_ids=ids)

    induced = {}
    for e, role in roles.items():
        height = settled_height[e]
        if role == "time_limit":
            env.episode_length_buf[e] = env.max_episode_length - 1
            induced[e] = f"episode_length_buf set to {env.max_episode_length - 1}"
        elif role == "low_base":
            place(e, (0.0, 0.0, 0.10))
            induced[e] = "base placed at 0.10 m, level, at rest"
        elif role == "tilt":
            place(e, (0.0, 0.0, height), _quat_about_x(math.radians(60.0)))
            induced[e] = f"base rolled 60 deg at the settled height {height:.3f} m"
        elif role == "base_contact":
            # Sunk well in: depenetration is capped at 1 m/s, so the contact outlasts the sensor's
            # three-substep history (a shallow 7 mm overlap cleared within the first substep).
            place(e, (0.0, 0.0, 0.0))
            induced[e] = "base centre placed at ground level, level, so the base is half in the ground"
        elif role == "workspace_exit":
            place(e, (0.80, 0.0, height))
            induced[e] = f"base moved 0.80 m along x at the settled height {height:.3f} m"

    env.step(zero)
    manager = env.termination_manager
    fired = {e: sorted(name for name in manager.active_terms if bool(manager.get_term(name)[e])) for e in roles}
    reset = env.reset_buf.cpu().tolist()
    length = env.episode_length_buf.cpu().tolist()
    default_root = robot.data.default_root_state[:, :7].clone()
    default_root[:, :3] += origins
    root_error = (robot.data.root_pose_w[:, :7] - default_root).abs().amax(dim=-1).cpu().tolist()
    # Resets clamp the default pose to the soft limits: the gripper's default 0 sits outside its
    # 0.9-scaled [0, 0.03] m range and resets to 1.5 mm open.
    limits = robot.data.soft_joint_pos_limits
    reset_pose = robot.data.default_joint_pos.clamp(limits[..., 0], limits[..., 1])
    joint_error = (robot.data.joint_pos - reset_pose).abs().amax(dim=-1).cpu().tolist()
    target_changed = (command.target_w != before["target"]).any(dim=-1).cpu().tolist()
    target_local = command.target_w - origins
    in_box = torch.ones(env.num_envs, dtype=torch.bool, device=device)
    for axis, (low, high) in enumerate(command.cfg.ranges):
        in_box &= (target_local[:, axis] >= low) & (target_local[:, axis] <= high)
    hold_default = (arm._hold.value == arm._offset).all(dim=-1).cpu().tolist() if hasattr(arm, "_hold") else None
    drift = torch.linalg.vector_norm(robot.data.root_pos_w[:, :2] - before["root"][:, :2], dim=-1).cpu().tolist()

    rows, checks = [], []
    for e, role in roles.items():
        should_reset = role != "control"
        row = {
            "env": e, "role": role, "induced": induced.get(e, "nothing"), "terms_fired": fired[e],
            "reset": bool(reset[e]), "episode_length_after": int(length[e]),
            "target_resampled": bool(target_changed[e]), "target_in_box": bool(in_box[e]),
        }
        if should_reset:
            row.update({"root_pose_error_vs_default": root_error[e], "joint_error_vs_soft_clamped_default": joint_error[e],
                        "arm_hold_at_default": None if hold_default is None else bool(hold_default[e])})
            hold_ok = hold_default is None or bool(hold_default[e])
            passed = (set(fired[e]) == EXPECTED_TERMS[role] and row["reset"] and length[e] == 0
                      and row["target_resampled"] and row["target_in_box"]
                      and root_error[e] < 1e-4 and joint_error[e] < 1e-4 and hold_ok)
            expected = (f"exactly {sorted(EXPECTED_TERMS[role])} fire; the env resets to the default root pose and the "
                        "soft-limit-clamped default joints (< 1e-4), episode length 0, a new target inside the box, arm command hold at default")
        else:
            row["base_drift_during_check_m"] = drift[e]
            passed = (not fired[e] and not row["reset"] and length[e] == int(before["length"][e]) + 1
                      and not row["target_resampled"] and drift[e] < 0.02)
            expected = "no term fires, no reset, episode length continues, same target, base moves < 0.02 m"
        rows.append(row)
        checks.append(_result(f"termination_{role}_env{e}", passed, expected, row))

    re_terminated = torch.zeros(env.num_envs, dtype=torch.bool, device=device)
    for _ in range(after_steps):
        env.step(zero)
        re_terminated |= env.reset_buf
    checks.append(_result("no_termination_right_after_reset", not bool(re_terminated.any()),
                          f"no environment terminates in the {after_steps} steps after the induced resets",
                          {"envs_terminated": re_terminated.nonzero().squeeze(-1).cpu().tolist()}))
    return checks, {"settle_steps": settle_steps, "settled_base_height_m": settled_height, "table": rows}


def arm_model(env, settle_steps=150, seed=0):
    """Compare the simulator's arm with the CPU model in `workspace.py` (URDF kinematics, weld mass model).

    1. Standing with zero actions, every arm joint should hold its zero-pose target: the model's static
       gravity torque there is below every joint's effort limit, and a 4000 N·m/rad drive would then
       deviate by well under 0.01 rad.
    2. Link masses and Link6's position in the base frame at random arm angles should match the model.
    """
    import numpy as np

    from . import workspace
    from .core import world_to_body

    robot, device = env.scene["robot"], env.device
    arm_ids = [robot.joint_names.index(name) for name in ARM_NAMES]
    link6 = robot.find_bodies("Link6")[0][0]
    joints, links = workspace.load_urdf()
    zero = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=device)

    def arm_deviation():
        return (robot.data.joint_pos[:, arm_ids] - robot.data.default_joint_pos[:, arm_ids]).abs()

    env.reset()
    trace = []
    for step in range(settle_steps):
        env.step(zero)
        if step % 10 == 0:
            trace.append([step, *arm_deviation().mean(dim=0).cpu().round(decimals=4).tolist()])
    q_rest = robot.data.joint_pos[:, arm_ids].cpu().numpy()
    deviation = np.abs(q_rest - robot.data.default_joint_pos[:, arm_ids].cpu().numpy())
    model_torque = workspace.gravity_torques(joints, links, q_rest)
    rest = {
        "settle_trace_step_then_mean_deviation_rad": trace,
        "joint_friction_coeff": robot.data.joint_friction_coeff[0, arm_ids].cpu().tolist(),
        "joint_dynamic_friction_coeff": robot.data.joint_dynamic_friction_coeff[0, arm_ids].cpu().tolist(),
        "joint_viscous_friction_coeff": robot.data.joint_viscous_friction_coeff[0, arm_ids].cpu().tolist(),
        "joint_armature": robot.data.joint_armature[0, arm_ids].cpu().tolist(),
        "joint_damping": robot.data.joint_damping[0, arm_ids].cpu().tolist(),
        "joint_stiffness": robot.data.joint_stiffness[0, arm_ids].cpu().tolist(),
        "arm_joint_deviation_rad_mean_over_envs": deviation.mean(axis=0).round(4).tolist(),
        "arm_joint_deviation_rad_max_over_envs": deviation.max(axis=0).round(4).tolist(),
        "model_gravity_torque_at_rest_nm": model_torque.mean(axis=0).round(3).tolist(),
        "computed_torque_nm_env0": robot.data.computed_torque[0, arm_ids].cpu().round(decimals=3).tolist(),
        "applied_torque_nm_env0": robot.data.applied_torque[0, arm_ids].cpu().round(decimals=3).tolist(),
        "effort_limit_nm": robot.data.joint_effort_limits[0, arm_ids].cpu().tolist(),
    }
    try:
        forces = robot.root_physx_view.get_dof_projected_joint_forces()
        rest["physx_projected_joint_force_env0"] = forces[0, arm_ids].cpu().round(decimals=3).tolist()
    except Exception as exc:  # Not every PhysX tensor API build exposes it.
        rest["physx_projected_joint_force_env0"] = f"unavailable: {type(exc).__name__}"

    # Reseat the arm exactly on its targets while the robot already stands, then hold: if it stays,
    # the rest deviation came from the landing, not from a load the drives cannot carry.
    positions = robot.data.joint_pos.clone()
    positions[:, arm_ids] = robot.data.default_joint_pos[:, arm_ids]
    velocities = robot.data.joint_vel.clone()
    velocities[:, arm_ids] = 0.0
    robot.write_joint_state_to_sim(positions, velocities)
    reseat_trace = []
    for step in range(100):
        env.step(zero)
        if step % 10 == 0 or step == 99:
            reseat_trace.append([step, *arm_deviation().mean(dim=0).cpu().round(decimals=4).tolist()])
    rest["reseated_on_target_then_held_trace"] = reseat_trace
    rest["reseated_max_deviation_after_100_steps_rad"] = float(arm_deviation().max())

    names = robot.body_names
    masses = robot.root_physx_view.get_masses()[0].cpu().numpy()
    coms = robot.root_physx_view.get_coms()[0].cpu().numpy()
    expected_mass = workspace.simulated_masses(links)
    mass_rows = {name: {"sim_kg": round(float(masses[names.index(name)]), 4), "model_kg": round(expected_mass[name], 4),
                        "sim_com_m": coms[names.index(name), :3].round(4).tolist(),
                        "urdf_com_m": links[name]["com"].round(4).tolist()}
                 for name in expected_mass if name in names}
    mass_error = max(abs(r["sim_kg"] - r["model_kg"]) for r in mass_rows.values())
    com_error = max(float(np.abs(np.array(r["sim_com_m"]) - np.array(r["urdf_com_m"])).max()) for r in mass_rows.values())

    # Random arm angles inside the soft limits, clear of the body, written straight into the joint state.
    # Teleporting the arm into the trunk with self-collisions on (602 N in one physics step) shifted a
    # pincer 1.1 mm from its joint reading, which is contact, not kinematics.
    generator = np.random.default_rng(seed)
    candidates = workspace.sample_configs(joints, 50 * env.num_envs, generator)
    candidates = candidates[workspace.clear_of_body(joints, candidates, workspace.ZERO_ACTION_BASE_HEIGHT_M)]
    q_target = candidates[:env.num_envs]
    positions = robot.data.joint_pos.clone()
    positions[:, arm_ids] = torch.tensor(q_target, dtype=positions.dtype, device=device)
    robot.write_joint_state_to_sim(positions, torch.zeros_like(positions))
    robot.set_joint_position_target(positions)
    env.sim.step(render=False)
    env.scene.update(dt=env.physics_dt)
    q_now = robot.data.joint_pos[:, arm_ids].cpu().numpy()
    sim_b = world_to_body(robot.data.body_link_pos_w[:, link6], robot.data.root_pos_w, robot.data.root_quat_w)
    fk_b = workspace.forward(joints, q_now)[0]["Link6"][1]
    fk_error = float(np.linalg.norm(sim_b.cpu().numpy() - fk_b, axis=1).max())
    # The task's controlled point (the pincer tip), as the command term computes it, at the actual jaw opening.
    command = env.command_manager.get_term("ee_position")
    jaw = robot.data.joint_pos[:, [robot.joint_names.index(n) for n in ("Joint7_1", "Joint7_2")]].cpu().numpy()
    tool_sim_b = world_to_body(command.tip_w, robot.data.root_pos_w, robot.data.root_quat_w).cpu().numpy()
    tool_fk_b = workspace.tool_position(workspace.forward(joints, q_now, jaw)[0], command.cfg.body_name,
                                        command.cfg.tip_offset)
    tool_errors = np.linalg.norm(tool_sim_b - tool_fk_b, axis=1)
    tool_error = float(tool_errors.max())
    env.reset()

    # What the sag costs at the controlled point: Link6 at the rest angles against the zero pose.
    droop = np.linalg.norm(workspace.forward(joints, q_rest)[0]["Link6"][1]
                           - workspace.forward(joints, np.zeros_like(q_rest))[0]["Link6"][1], axis=1)
    rest["link6_droop_m_max_over_envs"] = float(droop.max())
    checks = [
        _result("arm_holds_zero_pose_at_rest", float(droop.max()) < 0.005,
                "standing with zero actions, the arm holds its zero pose: Link6 within 5 mm of where the target "
                "angles put it (model gravity torque is below each effort limit)", rest),
        _result("arm_link_masses_match_weld_model", mass_error < 1e-3 and com_error < 1e-3,
                "each arm link's PhysX mass and centre of mass match the weld mass model and URDF (< 1 g, < 1 mm)",
                {"max_mass_error_kg": mass_error, "max_com_error_m": com_error, "links": mass_rows}),
        _result("link6_matches_urdf_kinematics", fk_error < 1e-3,
                "Link6's position in the base frame equals URDF forward kinematics at the simulator's joint angles (< 1 mm)",
                {"max_error_m": fk_error, "configs": env.num_envs}),
        _result("tool_point_matches_urdf_kinematics", tool_error < 1e-3,
                "the command term's controlled point equals URDF forward kinematics of its body and offset, "
                "at the simulator's arm and jaw positions (< 1 mm)",
                {"max_error_m": tool_error, "body": command.cfg.body_name, "offset_m": list(command.cfg.tip_offset),
                 "error_m_per_env": tool_errors.round(6).tolist(), "jaw_m_per_env": jaw.round(5).tolist(),
                 "arm_angles_rad_per_env": q_now.round(3).tolist(),
                 "arm_contact_n_per_env": torch.linalg.vector_norm(env.scene["arm_contact"].data.net_forces_w, dim=-1)
                 .amax(dim=-1).cpu().round(decimals=2).tolist()}),
    ]
    return checks, {"rest": rest}


# Arm angles that bury the fingertips 7 cm below the Go2's top surface, over the trunk (workspace.py FK:
# Link6 at (0.19, 0.00, 0.05) m and fingertip at (0.18, 0.00, -0.07) m in the base frame). Within the ±1 rad actions.
POSE_INTO_BODY = (0.0, 1.0, 0.88, 0.0, -0.16, 0.0)


def self_collision_control(env, settle_steps=60, steps=150):
    """Positive control for self-collision: command the arm into the Go2 body.

    With self-collisions on, the arm must be stopped by the body (contact on the arm links, or the
    base-contact termination the touch sets off). With them off, it must pass through untouched. A
    zero arm contact force at rest means "no overlap" only if this control shows the sensor sees contacts.
    """
    robot, sensor = env.scene["robot"], env.scene["arm_contact"]
    enabled = bool(env.cfg.scene.robot.spawn.articulation_props.enabled_self_collisions)
    arm_ids = [robot.joint_names.index(name) for name in ARM_NAMES]
    action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
    env.reset()
    for _ in range(settle_steps):
        env.step(action)
    action[:, 12:] = torch.tensor(POSE_INTO_BODY, device=env.device)
    peak = torch.zeros(env.num_envs, device=env.device)
    base_contact = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    resets = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    for _ in range(steps):
        env.step(action)
        # Read before anything else resets: a reset environment's sensor history is cleared.
        peak = torch.maximum(peak, torch.linalg.vector_norm(sensor.data.net_forces_w, dim=-1).amax(dim=-1))
        base_contact |= env.termination_manager.get_term("base_contact")
        resets |= env.reset_buf
    joint_error = (robot.data.joint_pos[:, arm_ids] - torch.tensor(POSE_INTO_BODY, device=env.device)).abs().amax(dim=-1)
    observed = {
        "self_collisions_enabled": enabled, "commanded_arm_angles_rad": POSE_INTO_BODY,
        "peak_arm_contact_n_per_env": peak.cpu().round(decimals=2).tolist(),
        "base_contact_termination_per_env": base_contact.cpu().tolist(),
        "any_reset_per_env": resets.cpu().tolist(),
        "final_arm_joint_error_rad_per_env": joint_error.cpu().round(decimals=3).tolist(),
    }
    blocked = (peak > 1.0) | base_contact
    if enabled:
        passed, expected = bool(blocked.all()), "every env: the body stops the arm (arm contact > 1 N or a base-contact termination)"
    else:
        passed = bool((~blocked).all() and (joint_error[~resets] < 0.05).all())
        expected = "every env: no arm contact and no base-contact termination; the arm reaches the pose (< 0.05 rad)"
    env.reset()
    return [_result("self_collision_positive_control", passed, expected, observed)]


def target_box_needs_arm_motion(env, settle_steps=250):
    """Where the controlled point rests with zero actions, against the target box.

    F-013: the old box surrounded the resting pincer tip, so 13% of targets were met without the arm
    moving at all and the reach metric could not be told apart from doing nothing. This measures the
    resting tip in the simulator (the CPU model puts it 1.6 cm out, having neither the arm's residual
    sag nor the base's resting tilt) and requires every point of the box, not just the sampled
    targets, to be further than G1a's success radius from it.
    """
    command = env.command_manager.get_term("ee_position")
    action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
    env.reset()
    for _ in range(settle_steps):
        env.step(action)
    tip = (command.tip_w - env.scene.env_origins)
    ranges = torch.tensor([list(axis) for axis in command.cfg.ranges], device=env.device)  # (3, 2)
    # Nearest point of the axis-aligned box to each resting tip, so this covers the whole box
    # rather than only the targets this seed happened to draw.
    nearest = tip.clamp(min=ranges[:, 0], max=ranges[:, 1])
    box_distance = torch.linalg.vector_norm(tip - nearest, dim=-1)
    target_distance = command.error_m
    reference = torch.tensor(ZERO_ACTION_TIP_M, device=env.device)
    drift = torch.linalg.vector_norm(tip - reference, dim=-1)
    observed = {
        "settle_steps": settle_steps,
        "resting_tip_env_frame_m": tip.mean(dim=0).cpu().round(decimals=4).tolist(),
        "resting_tip_spread_m": (tip.amax(dim=0) - tip.amin(dim=0)).cpu().round(decimals=4).tolist(),
        "target_box_env_frame_m": [list(axis) for axis in command.cfg.ranges],
        "min_distance_to_box_m": float(box_distance.min()),
        "distance_to_sampled_target_m": {"min": float(target_distance.min()), "max": float(target_distance.max())},
        "sampled_targets_within_success_radius": int((target_distance <= SUCCESS_RADIUS_M).sum()),
        "drift_from_recorded_resting_tip_m": float(drift.max()),
        "recorded_resting_tip_m": list(ZERO_ACTION_TIP_M),
    }
    passed = bool((box_distance > SUCCESS_RADIUS_M).all() and (target_distance > SUCCESS_RADIUS_M).all())
    return [_result("target_box_needs_arm_motion", passed,
                    f"with zero actions the controlled point settles further than G1a's "
                    f"{SUCCESS_RADIUS_M * 100:.0f} cm from every point of the target box, so no target "
                    f"can be met without moving the arm", observed),
            _result("resting_tip_matches_recorded_stance", float(drift.max()) < 0.02,
                    "the resting controlled point is within 2 cm of the stance the target box was placed "
                    "against (task_space.ZERO_ACTION_TIP_M)", observed)]


def arm_planner(env, settle_steps=60, move_steps=60, step_rad=0.3):
    """The D1 firmware planner (F-045), when configured: bounded plan, a drive that follows it.

    Every arm joint is commanded `step_rad` from default at once. The planned drive target must respect
    each joint's speed ceiling and the fitted acceleration and deceleration, arrive at the setpoint, and
    the simulated joint must follow the plan rather than trail it -- without velocity feedforward the
    drive's damping put it 97 ms behind (Week 1, 2026-09-17). Sampled at the policy rate, so speed and
    acceleration are 20 ms averages, which a correct plan cannot exceed.
    """
    arm = env.action_manager.get_term("arm")
    plan = arm.cfg.trajectory
    if plan is None:
        return [_result("arm_planner", True, "arm setpoints go straight to the drive (--arm_trajectory none)",
                        {"configured": False})]
    from .env_cfg import ARM_NAMES

    robot = env.scene["robot"]
    arm_ids = [robot.joint_names.index(name) for name in ARM_NAMES]
    env.reset()
    action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
    for _ in range(settle_steps):
        env.step(action)
    action[:, 12:] = step_rad
    planned, joint, setpoint = [], [], []
    for _ in range(move_steps):
        env.step(action)
        default = robot.data.default_joint_pos[:, arm_ids]
        planned.append((arm.planned_targets - default).clone())
        joint.append((robot.data.joint_pos[:, arm_ids] - default).clone())
        setpoint.append((arm.processed_actions - default).clone())
    planned, joint, setpoint = (torch.stack(x).cpu() for x in (planned, joint, setpoint))
    dt = env.step_dt
    speed = (planned[1:] - planned[:-1]).abs() / dt
    change = (speed[1:] - speed[:-1]) / dt
    ceilings = torch.tensor(plan["velocity_limits_rad_s"])
    speed_excess = float((speed - ceilings).max())
    # Only speeding up is bounded. Each command re-send restarts the plan from rest by design (retention
    # 0, fitted to F-035), so speed can fall faster than the deceleration at those instants.
    accel_excess = float((change - plan["accel_rad_s2"]).max())
    largest_drop = float(-change.min())
    arrival_error = float((planned[-1] - setpoint[-1]).abs().max())
    tracking_rms = math.degrees(float(torch.sqrt(torch.mean((joint - planned) ** 2))))
    return [
        _result("arm_planner_bounded", speed_excess <= 1e-3 and accel_excess <= 0.5,
                "planned drive targets stay within each joint's speed ceiling and never speed up faster than "
                "the fitted acceleration (20 ms averages)",
                {"max_speed_over_ceiling_rad_s": speed_excess, "max_speedup_over_accel_rad_s2": accel_excess,
                 "accel_rad_s2": plan["accel_rad_s2"], "replan_retention": plan["replan_velocity_retention"],
                 "largest_slowdown_rad_s2": largest_drop,
                 "note": "slowdowns beyond the deceleration are the per-message restarts the model fits (F-045)"}),
        _result("arm_planner_arrives_and_is_followed", arrival_error < 1e-4 and tracking_rms < 1.0,
                f"the plan reaches the setpoint and the joint follows it (RMS < 1 deg)",
                {"final_plan_to_setpoint_rad": arrival_error, "joint_vs_plan_rms_deg": tracking_rms,
                 "step_rad": step_rad, "steps": move_steps}),
    ]


def run_checks(env):
    timing_checks, timing = interface_timing(env)
    reset_checks, resets = terminations_and_resets(env)
    model_checks, model = arm_model(env)
    checks = (timing_checks + reset_checks + model_checks + self_collision_control(env)
              + target_box_needs_arm_motion(env) + arm_planner(env))
    return {
        "all_passed": all(check["passed"] for check in checks),
        "failed": [check["name"] for check in checks if not check["passed"]],
        "checks": checks, "timing": timing, "resets": resets, "arm_model": model,
        "interpretation": "Deliberate mechanism checks only; not standing, stepping or reaching ability.",
    }

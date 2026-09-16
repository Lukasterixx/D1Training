"""Frozen-manifest episode evaluation: the measurement G1a is decided on.

Runs every episode of a manifest under one controller, with deterministic actions, and reports what
the plan's measurement section asks for: reach success with a continuous dwell, survival, falls,
error norm, RMS, 95th percentile, time to reach, base tilt and height, and joint-limit and effort
saturation, with transient and final-two-second tracking kept apart.

Three things this is careful about, because each of them can silently flatter a result:

- **Failed episodes stay in the denominator.** A policy that falls in 40% of episodes cannot report
  the error of the 60% that survived as if it were the result.
- **Truncated traces are not filled.** An episode that ends early is recorded as truncated, with the
  samples it actually produced; the missing tail is not replaced with the last good value.
- **Terminal samples are taken before the reset.** Isaac Lab resets a terminated environment inside
  `step()`, so the state read after that step belongs to the *next* episode. The sample from a
  terminating step is discarded, and the episode's last valid sample is the one before it.

Torque reporting needs care, because the arm and the legs mean different things by it. The legs use
an explicit actuator, so `applied_torque` is the model's own clipped output: a real number to report.
The arm is an `ImplicitActuator`, where PhysX runs the PD itself and `applied_torque` is only Isaac
Lab's Python-side estimate (`stiffness * error + damping * error_vel`) clipped to the effort limit --
the class calls them "approximate torques ... since PhysX does not expose this quantity". With the
arm's 4000 N·m/rad standing in for the D1's servo loop against 1.7-3.3 N·m limits, that estimate
saturates on four of six arm joints while the robot is merely standing still, whereas PhysX's own
joint reaction torques are about 1.1 N·m (Week 1, 2026-09-16). So the commanded figure is reported as
a commanded figure, and the arm's measured reaction torque is reported beside it. That reaction torque
is the whole load carried through the joint, gravity and inertia included, so it is not the drive's
torque either and must not be read against the effort limit.

`Metrics/ee_position/position_error_m` from training is sampled at reset and is not this.
"""
from __future__ import annotations

import numpy as np
import torch

from .manifest import DWELL_S, mismatches, targets as manifest_targets

FALL_TERMS = ("low_base", "bad_orientation", "base_contact")
JOINT_LIMIT_MARGIN_RAD = 1e-3
EFFORT_SATURATION_FRACTION = 0.99
FINAL_WINDOW_S = 2.0


def wilson(successes, total, z=1.96):
    """Wilson score interval: usable at 0% and 100%, where the normal approximation is not."""
    if total == 0:
        return (float("nan"), float("nan"))
    p = successes / total
    denominator = 1.0 + z * z / total
    centre = (p + z * z / (2 * total)) / denominator
    half = z * np.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denominator
    return (float(max(0.0, centre - half)), float(min(1.0, centre + half)))


def oscillation(values, step_dt):
    """Peak-to-peak amplitude and frequency of a signal about its own mean.

    Lukas watching a replay described the body and arm "oscillating back and forth / up and down"
    while the tool point tracked accurately (Week 1, 2026-09-16). That is a limit cycle, and RMS
    magnitude does not describe one: a slow large sway and a fast small shake can share an RMS. The
    frequency comes from mean crossings -- two crossings per cycle -- which needs no windowing choice
    and is robust on the ~100 samples a 2 s window holds at 50 Hz.

    Returns (peak-to-peak, Hz). Frequency is None when the signal does not cross its mean at all.
    """
    values = np.asarray(values, dtype=float)
    if len(values) < 2:
        return (0.0, None)
    centred = values - values.mean()
    crossings = int(np.count_nonzero(np.diff(np.signbit(centred))))
    duration = len(values) * step_dt
    return (float(values.max() - values.min()),
            float(crossings / (2.0 * duration)) if crossings else None)


def longest_run(flags):
    """Longest run of consecutive True values."""
    best = run = 0
    for flag in flags:
        run = run + 1 if flag else 0
        best = max(best, run)
    return best


def _leg_effort_limit(applied, joint_vel):
    from motor_model import GO2_HV, unitree_effort_limit

    return unitree_effort_limit(applied, joint_vel, GO2_HV.Y1, GO2_HV.Y2, GO2_HV.X1, GO2_HV.X2)


def run_episodes(env, manifest, policy=None, device="cuda:0", progress=None):
    """Run every manifest episode, in batches of `env.num_envs`, and return per-episode records.

    `policy` is a deterministic callable from observations to actions; `None` means zero actions,
    which is the baseline every reach number is reported against.
    """
    # Imported here, not at module scope: the metric rules below are pure and are tested on CPU,
    # while env_cfg pulls in Isaac Lab.
    from .env_cfg import ARM_NAMES, LEG_NAMES

    arm_link_names = [f"Link{i}" for i in range(1, 7)]
    unwrapped = env.unwrapped
    robot = unwrapped.scene["robot"]
    command = unwrapped.command_manager.get_term("ee_position")
    conditions = manifest["conditions"]
    step_dt = unwrapped.step_dt
    total_steps = int(round(conditions["episode_length_s"] / step_dt))
    # From the manifest, not the module default: a frozen manifest that declared a different dwell
    # was silently evaluated at 1.0 s before, which is the manifest saying one thing and the code
    # doing another -- the same family of defect as F-039.
    dwell_steps = int(round(conditions.get("dwell_s", DWELL_S) / step_dt))
    final_steps = int(round(FINAL_WINDOW_S / step_dt))
    radius = conditions["success_radius_m"]

    arm_ids = [robot.joint_names.index(name) for name in ARM_NAMES]
    leg_ids = [robot.joint_names.index(name) for name in LEG_NAMES]
    controlled = leg_ids + arm_ids
    soft_limits = robot.data.soft_joint_pos_limits[:, controlled]
    arm_effort_limit = robot.data.joint_effort_limits[:, arm_ids]
    arm_link_ids = [robot.body_names.index(name) for name in arm_link_names if name in robot.body_names]

    goals = manifest_targets(manifest)
    batch = env.num_envs
    records = []
    for start in range(0, len(goals), batch):
        chunk = goals[start:start + batch]
        active_count = len(chunk)
        padded = np.zeros((batch, 3))
        padded[:active_count] = chunk

        # Inference mode throughout: once the first batch has stepped, the command term's buffers
        # are inference tensors and writing to them outside inference mode raises.
        with torch.inference_mode():
            env.reset()
            # Overwrite the sampled targets with the manifest's, in world frame. The command term
            # resamples on reset, so this must come after it.
            command.target_w[:] = unwrapped.scene.env_origins + torch.tensor(
                padded, dtype=command.target_w.dtype, device=command.target_w.device)
            obs = env.get_observations()

        alive = torch.ones(batch, dtype=torch.bool, device=device)
        ended_step = torch.full((batch,), -1, dtype=torch.long, device=device)
        fell = torch.zeros(batch, dtype=torch.bool, device=device)
        timed_out = torch.zeros(batch, dtype=torch.bool, device=device)
        samples = []
        for step in range(total_steps):
            with torch.inference_mode():
                action = policy(obs) if policy else torch.zeros(batch, 18, device=device)
                if not torch.isfinite(action).all():
                    raise RuntimeError(f"Non-finite action at step {step}.")
                obs, _, _, _ = env.step(action)

            # Read before recording: these flags describe the step just taken, while the
            # robot state of a done environment has already been reset.
            terminated = unwrapped.reset_terminated.clone()
            truncated = unwrapped.reset_time_outs.clone()
            data = robot.data
            joint_pos = data.joint_pos[:, controlled]
            at_limit = ((joint_pos <= soft_limits[..., 0] + JOINT_LIMIT_MARGIN_RAD)
                        | (joint_pos >= soft_limits[..., 1] - JOINT_LIMIT_MARGIN_RAD)).any(dim=-1)
            # Commanded, not measured: an ImplicitActuator's applied_torque is Isaac Lab's PD
            # estimate clipped to the limit (see the module docstring).
            arm_commanded = data.applied_torque[:, arm_ids].abs()
            arm_at_limit = (arm_commanded >= EFFORT_SATURATION_FRACTION * arm_effort_limit).any(dim=-1)
            # Measured: PhysX's reaction torque at each arm joint.
            reaction = robot.root_physx_view.get_link_incoming_joint_force()[:, arm_link_ids, 3:]
            arm_reaction = torch.linalg.vector_norm(reaction, dim=-1).amax(dim=-1)
            # The legs use an explicit actuator, so this is the model's own clipped output.
            leg_torque = data.applied_torque[:, leg_ids]
            leg_limit = _leg_effort_limit(leg_torque, data.joint_vel[:, leg_ids])
            leg_saturated = (leg_torque.abs() >= EFFORT_SATURATION_FRACTION * leg_limit).any(dim=-1)
            offset = data.root_pos_w - unwrapped.scene.env_origins
            samples.append(torch.stack([
                command.error_m,
                offset[:, 2],
                torch.rad2deg(torch.acos((-data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))),
                at_limit.float(),
                arm_at_limit.float(),
                leg_saturated.float(),
                leg_torque.abs().amax(dim=-1),
                arm_reaction,
                # Base translation from the spawn point. Recorded because it was not: the evaluator
                # measured base *height* and *tilt* but never how far the robot walked, so a policy
                # that meets its target by stepping 20 cm forward scored identically to one that
                # stood and reached (Week 1, 2026-09-16). Height caught the squat and tilt caught the
                # rotation; nothing was watching translation.
                offset[:, 0],
                offset[:, 1],
                # Vibration. Lukas watching a replay saw the body and arm shaking while the tool
                # point tracked accurately (Week 1, 2026-09-16), which no recorded metric showed.
                # Reported separately for arm and legs because they shake for different reasons: the
                # arm is torque-limited rather than PD-tracked (F-017) and its commands are held at
                # 10 Hz, while the legs run every policy step.
                torch.sqrt(torch.mean(torch.square(data.joint_vel[:, arm_ids]), dim=-1)),
                torch.sqrt(torch.mean(torch.square(data.joint_vel[:, leg_ids]), dim=-1)),
            ], dim=-1).cpu())

            done = terminated | truncated
            newly = done & alive
            if newly.any():
                ended_step[newly] = step
                fell |= newly & terminated
                timed_out |= newly & truncated
                alive &= ~newly
            if not alive.any() and step + 1 < total_steps:
                # Everything has finished; the rest of the batch would only record reset states.
                break

        trace = torch.stack(samples).numpy()  # (steps, envs, fields)
        for i in range(active_count):
            end = int(ended_step[i])
            # The terminating step's sample is post-reset, so the last valid sample precedes it.
            valid = trace[:end, i] if end >= 0 else trace[:, i]
            records.append(_episode_record(
                index=start + i, target=chunk[i], trace=valid,
                ended_step=end, total_steps=total_steps, step_dt=step_dt,
                fell=bool(fell[i]), timed_out=bool(timed_out[i]),
                radius=radius, dwell_steps=dwell_steps, final_steps=final_steps))
        if progress:
            progress(min(start + batch, len(goals)), len(goals))
    return records


def _hold_oscillation(height, base_x, error, steps, final_steps, step_dt):
    """Amplitude and frequency of base height, base travel and tool error over the parked tail."""
    if steps < final_steps:
        return {k: None for k in ("hold_base_z_p2p_m", "hold_base_z_hz", "hold_base_x_p2p_m",
                                  "hold_base_x_hz", "hold_error_p2p_m", "hold_error_hz")}
    z_p2p, z_hz = oscillation(height[-final_steps:], step_dt)
    x_p2p, x_hz = oscillation(base_x[-final_steps:], step_dt)
    e_p2p, e_hz = oscillation(error[-final_steps:], step_dt)
    return {"hold_base_z_p2p_m": z_p2p, "hold_base_z_hz": z_hz,
            "hold_base_x_p2p_m": x_p2p, "hold_base_x_hz": x_hz,
            "hold_error_p2p_m": e_p2p, "hold_error_hz": e_hz}


def _episode_record(index, target, trace, ended_step, total_steps, step_dt, fell, timed_out,
                    radius, dwell_steps, final_steps):
    (error, height, tilt, at_limit, arm_at_limit, leg_sat, leg_torque, arm_reaction,
     base_x, base_y, arm_vel_rms, leg_vel_rms) = (trace[:, i] for i in range(12))
    steps = len(error)
    within = error <= radius
    dwell = longest_run(within)
    reached = np.flatnonzero(within)
    # G1a: the episode has to last to its end, so an early termination cannot succeed however
    # close the tool point got before it.
    survived = bool(timed_out and not fell)
    truncated = not survived
    final = error[-final_steps:] if steps >= final_steps else np.array([])
    transient = error[:-final_steps] if steps > final_steps else error

    def stats(values):
        if len(values) == 0:
            return None
        return {"mean_m": float(values.mean()), "max_m": float(values.max())}

    return {
        "index": index,
        "target_env_frame_m": [float(v) for v in target],
        "survived": survived,
        "fell": bool(fell),
        "truncated": truncated,
        "ended_step": int(ended_step) if ended_step >= 0 else None,
        "recorded_steps": steps,
        "recorded_s": round(steps * step_dt, 4),
        "success": bool(survived and dwell >= dwell_steps),
        "max_dwell_s": round(dwell * step_dt, 4),
        "time_to_reach_s": round(float(reached[0]) * step_dt, 4) if len(reached) else None,
        "final_error_m": float(error[-1]) if steps else None,
        "rms_error_m": float(np.sqrt(np.mean(error ** 2))) if steps else None,
        "p95_error_m": float(np.percentile(error, 95)) if steps else None,
        "transient_error": stats(transient),
        "final_2s_error": stats(final),
        "min_base_height_m": float(height.min()) if steps else None,
        # Signed, so the sign carries meaning: negative is the backward settle every episode starts
        # with (F-014, about -5.6 cm under zero actions), positive is the robot walking forward.
        "final_base_x_from_spawn_m": float(base_x[-1]) if steps else None,
        "final_base_y_from_spawn_m": float(base_y[-1]) if steps else None,
        "max_base_x_from_spawn_m": float(base_x.max()) if steps else None,
        "max_base_horizontal_travel_m": float(np.max(np.hypot(base_x, base_y))) if steps else None,
        # Over the final two seconds the tool point is parked, so anything left here is vibration
        # rather than travel. Reported beside the whole-episode figure it is measured against.
        "arm_joint_vel_rms_rad_s": float(arm_vel_rms.mean()) if steps else None,
        "leg_joint_vel_rms_rad_s": float(leg_vel_rms.mean()) if steps else None,
        "arm_joint_vel_rms_holding_rad_s": float(arm_vel_rms[-final_steps:].mean()) if steps >= final_steps else None,
        "leg_joint_vel_rms_holding_rad_s": float(leg_vel_rms[-final_steps:].mean()) if steps >= final_steps else None,
        # The final two seconds, by which the tool point is parked: whatever moves here is a limit
        # cycle, not progress toward the target. Amplitude and frequency, because the two together
        # are what distinguishes a slow sway from a fast shake.
        **_hold_oscillation(height, base_x, error, steps, final_steps, step_dt),
        "max_tilt_deg": float(tilt.max()) if steps else None,
        "joint_limit_steps_frac": float(at_limit.mean()) if steps else None,
        "arm_commanded_effort_at_limit_frac": float(arm_at_limit.mean()) if steps else None,
        "peak_arm_joint_torque_nm": float(arm_reaction.max()) if steps else None,
        "leg_effort_saturated_frac": float(leg_sat.mean()) if steps else None,
        "peak_leg_torque_nm": float(leg_torque.max()) if steps else None,
    }


def summarise(records, manifest, controller, conditions=None):
    """Aggregate per-episode records into the reported result, with G1a checked explicitly."""
    total = len(records)
    successes = sum(r["success"] for r in records)
    falls = sum(r["fell"] for r in records)
    survived = [r for r in records if r["survived"]]
    truncated = [r for r in records if r["truncated"]]
    criteria = manifest["conditions"]

    def pooled(key, source):
        values = [r[key] for r in source if r[key] is not None]
        return float(np.mean(values)) if values else None

    def quantiles(source):
        values = [r["p95_error_m"] for r in source if r["p95_error_m"] is not None]
        return float(np.mean(values)) if values else None

    reached = [r["time_to_reach_s"] for r in records if r["time_to_reach_s"] is not None]
    # G1a's thresholds, checked here rather than left to the reader.
    success_rate = successes / total if total else float("nan")
    fall_rate = falls / total if total else float("nan")
    gate = {
        "criterion": "≥90% reach within "
                     f"{criteria['success_radius_m'] * 100:.0f} cm for ≥{criteria['dwell_s']:.0f} s continuous, "
                     "surviving to episode end, with ≤1% falls",
        "success_rate_ok": bool(success_rate >= 0.90),
        "fall_rate_ok": bool(fall_rate <= 0.01),
    }
    gate["passed"] = bool(gate["success_rate_ok"] and gate["fall_rate_ok"])
    return {
        "controller": controller,
        "manifest": {"role": manifest["role"], "episode_count": manifest["episode_count"],
                     "seed": manifest["seed"], "content_sha256": manifest["content_sha256"]},
        "condition_mismatches": mismatches(manifest, conditions or {}),
        "episodes": total,
        "success": {"count": successes, "rate": success_rate, "wilson95": wilson(successes, total)},
        "falls": {"count": falls, "rate": fall_rate, "wilson95": wilson(falls, total)},
        "survived": {"count": len(survived), "rate": len(survived) / total if total else float("nan")},
        "truncated": {"count": len(truncated),
                      "note": "ended before the time limit; their traces stop at the last pre-reset sample "
                              "and the missing tail is not filled"},
        # Denominator is every episode, including the ones that failed.
        "error_all_episodes": {
            "rms_m": pooled("rms_error_m", records),
            "p95_m": quantiles(records),
            "final_m": pooled("final_error_m", records),
            "transient_mean_m": pooled_nested(records, "transient_error", "mean_m"),
            "final_2s_mean_m": pooled_nested(records, "final_2s_error", "mean_m"),
        },
        "error_surviving_episodes_only": {
            "note": "for diagnosis only; the reported result is error_all_episodes",
            "count": len(survived),
            "rms_m": pooled("rms_error_m", survived),
            "final_2s_mean_m": pooled_nested(survived, "final_2s_error", "mean_m"),
        },
        "time_to_reach_s": {"count": len(reached), "mean": float(np.mean(reached)) if reached else None,
                            "of_episodes": total},
        "max_dwell_s": {"mean": pooled("max_dwell_s", records)},
        "posture": {"min_base_height_m": min((r["min_base_height_m"] for r in records
                                              if r["min_base_height_m"] is not None), default=None),
                    "max_tilt_deg": max((r["max_tilt_deg"] for r in records
                                         if r["max_tilt_deg"] is not None), default=None)},
        "base_travel": {
            "mean_final_x_from_spawn_m": pooled("final_base_x_from_spawn_m", records),
            "max_final_x_from_spawn_m": max((r["final_base_x_from_spawn_m"] for r in records
                                             if r["final_base_x_from_spawn_m"] is not None), default=None),
            "max_horizontal_travel_m": max((r["max_base_horizontal_travel_m"] for r in records
                                            if r["max_base_horizontal_travel_m"] is not None), default=None),
        },
        "vibration": {
            "arm_joint_vel_rms_rad_s": pooled("arm_joint_vel_rms_rad_s", records),
            "leg_joint_vel_rms_rad_s": pooled("leg_joint_vel_rms_rad_s", records),
            "arm_joint_vel_rms_holding_rad_s": pooled("arm_joint_vel_rms_holding_rad_s", records),
            "leg_joint_vel_rms_holding_rad_s": pooled("leg_joint_vel_rms_holding_rad_s", records),
            "hold_base_z_p2p_m": pooled("hold_base_z_p2p_m", records),
            "hold_base_z_hz": pooled("hold_base_z_hz", records),
            "hold_base_x_p2p_m": pooled("hold_base_x_p2p_m", records),
            "hold_base_x_hz": pooled("hold_base_x_hz", records),
            "hold_error_p2p_m": pooled("hold_error_p2p_m", records),
            "hold_error_hz": pooled("hold_error_hz", records),
            "note": "RMS joint velocity. The *_holding figures cover the final two seconds, by which "
                    "the tool point is parked, so they measure shaking rather than motion. The D1 "
                    "publishes at 9 Hz, answers a step in ~127 ms and needs ~220 ms to reach cruise "
                    "(F-020, F-021, F-035), so arm chatter above a few rad/s is commanding motion the "
                    "hardware cannot execute.",
        },
        "_base_travel_note": {
            "note": "Displacement from the spawn point, signed in x: negative is the backward settle "
                    "every episode starts with (about -5.6 cm under zero actions, F-014), positive is "
                    "the robot walking forward. The task is a stance-and-reach and the workspace "
                    "analysis says the box is reachable from the settled stance (F-016), so large "
                    "positive travel means the reach is partly locomotion.",
        },
        "saturation": {
            "joint_limit_steps_frac": pooled("joint_limit_steps_frac", records),
            "arm_commanded_effort_at_limit_frac": pooled("arm_commanded_effort_at_limit_frac", records),
            "peak_arm_joint_torque_nm": max((r["peak_arm_joint_torque_nm"] for r in records
                                             if r["peak_arm_joint_torque_nm"] is not None), default=None),
            "leg_effort_saturated_frac": pooled("leg_effort_saturated_frac", records),
            "peak_leg_torque_nm": max((r["peak_leg_torque_nm"] for r in records
                                       if r["peak_leg_torque_nm"] is not None), default=None),
            "note": "arm_commanded_* is Isaac Lab's PD estimate clipped to the effort limit, not a PhysX "
                    "measurement, and it saturates even standing still because the arm's 4000 N·m/rad gains "
                    "far exceed its 1.7-3.3 N·m limits. peak_arm_joint_torque_nm is PhysX's reaction torque: "
                    "the total load carried through the joint, including gravity and inertia from the links "
                    "beyond it, not the drive's own torque. It is not comparable to the 1.7-3.3 N·m effort "
                    "limits and may exceed them without any limit being violated. "
                    "The legs use an explicit actuator, so their figures are the model's own clipped output.",
        },
        "g1a": gate,
        "interpretation": "One controller on one frozen manifest, one simulator seed. A zero-action run of "
                          "the same manifest is the baseline this must be read against.",
    }


def pooled_nested(records, outer, inner):
    values = [r[outer][inner] for r in records if r.get(outer) and r[outer].get(inner) is not None]
    return float(np.mean(values)) if values else None

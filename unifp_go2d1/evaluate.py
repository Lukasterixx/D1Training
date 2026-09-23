"""Run a Go2+D1 UniFP policy against a frozen manifest and summarise what it did.

What this measures, and what each number is not:

  * **Goal tracking** -- distance from the tool tip to the commanded end-effector goal, measured
    only on steps where no force is commanded and none is applied. This is the closest thing the
    task has to a reach error, and it is the number to quote for reaching.
  * **Unified tracking** -- distance from the tip to `goal + (applied + commanded force)/k`, which
    is what the task actually rewards (`tracking_ee_force_world`). Under force this is the
    objective; with no force it is the same as goal tracking.
  * **Force realisation** -- how much of a commanded force the policy actually produces, read as
    the displacement it achieves along the commanded direction times the virtual stiffness `k`.
    It is a *simulated* force through UniFP's admittance formulation, not a measured contact
    force, and there is no force sensor anywhere in this loop.
  * **Estimator error** -- the policy's own predicted end-effector force against the force
    actually applied. This is UniFP's estimator supervision, evaluated rather than trained.
  * **Base velocity tracking** and **falls**.

Every one is reported beside the zero-action baseline on the same manifest, because without it a
tracking number says nothing: the resting tool tip is already somewhere, and some goals are near it.
"""

import hashlib
import math
import statistics as st

import torch
from isaacgym.torch_utils import quat_apply


def _finite(values):
    return [v for v in values if v is not None and math.isfinite(v)]


def _stats(values):
    vals = _finite(values)
    if not vals:
        return None
    vals_sorted = sorted(vals)
    return {
        "n": len(vals),
        "mean": st.mean(vals),
        "median": st.median(vals),
        "p90": vals_sorted[min(int(0.9 * len(vals)), len(vals) - 1)],
        "max": max(vals),
    }


def run_episodes(env, manifest, policy=None, device="cuda:0", progress=None):
    """One record per manifest episode. `policy=None` runs the zero-action baseline.

    All episodes run **at once**, one per environment, for exactly the full episode length, with
    termination recorded but not acted upon. Both halves of that matter:

      * *In parallel*, because the environment draws its random schedule in batches across
        environments -- so a fixed seed and a fixed environment count give every episode the same
        schedule every time, while stepping episodes one at a time is both slower and no more
        reproducible.
      * *Without resetting on a fall*, because a reset mid-batch would redraw commands for that one
        environment and shift the random stream for every episode after it. It also means the
        schedule is always realised to full length, so the digest covers the whole episode rather
        than whatever prefix a particular policy survived. A fallen robot flops around and its
        tracking numbers stop meaning anything, so metrics are cut at the step it fell -- but the
        schedule generator does not care, and that is the point.
    """
    episodes = manifest["episodes"]
    if env.num_envs != len(episodes):
        raise ValueError(f"manifest has {len(episodes)} episodes but env has {env.num_envs} "
                         f"environments; they must match (the schedule depends on the batch)")

    n = env.num_envs
    max_steps = int(env.max_episode_length)

    # Record terminations instead of acting on them.
    real_termination = torch.zeros(n, dtype=torch.bool, device=env.device)
    original_check = env.check_termination

    def check_without_reset():
        original_check()
        real_termination[:] = env.reset_buf & ~env.time_out_buf
        env.reset_buf[:] = False

    torch.manual_seed(manifest["seed"])
    env.reset()
    env.episode_length_buf[:] = 0
    # `reset_idx` clears the force-push state but NOT the push intervals, which are only redrawn
    # when a push cycle completes. Redrawing them here, from the seeded stream, keeps a run from
    # inheriting whatever the environment happened to be holding.
    for buf, lo, hi in (
        ("push_interval_gripper_cmd", "push_interval_gripper_cmd_min", "push_interval_gripper_cmd_max"),
        ("push_interval_gripper_ext", "push_interval_gripper_ext_min", "push_interval_gripper_ext_max"),
        ("push_interval_base_cmd", "push_interval_base_cmd_min", "push_interval_base_cmd_max"),
        ("push_interval_base_ext", "push_interval_base_ext_min", "push_interval_base_ext_max"),
    ):
        getattr(env, buf)[:, 0] = torch.randint(
            int(getattr(env, lo)), int(getattr(env, hi)), (n,), device=env.device)

    env.check_termination = check_without_reset

    fell_at = [None] * n
    schedules = [hashlib.sha256() for _ in range(n)]
    trace = {k: [[] for _ in range(n)] for k in
             ("goal_err", "unified_err", "force_cmd", "force_applied", "force_along_cmd",
              "est_err", "base_vel_err", "base_z")}
    info = {}

    try:
        with torch.no_grad():
            for step in range(max_steps):
                obs = env.get_observations()
                action = (policy(obs, info) if policy is not None
                          else torch.zeros(n, env.num_actions, device=device))
                env.step(action.detach())

                tip = env.ee_pos
                goal = env.curr_ee_goal_cart_world
                goal_sphere = env.curr_ee_goal_sphere
                f_applied = env.forces[:, env.gripper_idx, 0:3]
                f_cmd_local = env.current_Fxyz_gripper_cmd
                vel_cmd = env.commands[:, :3]

                f_cmd_world = quat_apply(env.base_yaw_quat, f_cmd_local)
                offset = (f_applied + f_cmd_world) / env.gripper_force_kps
                unified_target = goal + offset

                goal_err = torch.norm(tip - goal, dim=1)
                unified_err = torch.norm(tip - unified_target, dim=1)
                cmd_mag = torch.norm(f_cmd_world, dim=1)
                applied_mag = torch.norm(f_applied, dim=1)
                # Force realised along the command: the displacement achieved from the bare goal,
                # projected on the commanded direction, times the virtual stiffness.
                safe_dir = f_cmd_world / cmd_mag.clamp(min=1e-6).unsqueeze(1)
                realised = (((tip - goal) * safe_dir).sum(dim=1) * env.gripper_force_kps[:, 0])
                vel_err = torch.norm(env.base_lin_vel[:, :2] - vel_cmd[:, :2], dim=1)
                base_z = env.root_states[:, 2]

                latents = info.get("latents")
                if latents is not None:
                    est = latents if torch.is_tensor(latents) else torch.tensor(
                        latents, device=tip.device)
                    est_mag = torch.norm(est[:, 6:9] / env.obs_scales.ee_force, dim=1)
                    est_err = (est_mag - applied_mag).abs()
                else:
                    est_err = None

                newly = (real_termination & torch.tensor(
                    [f is None for f in fell_at], device=env.device)).nonzero().flatten().tolist()
                for i in newly:
                    fell_at[i] = step

                sph, fcl, vcm = goal_sphere.tolist(), f_cmd_local.tolist(), vel_cmd.tolist()
                ge, ue = goal_err.tolist(), unified_err.tolist()
                cm, am, rl = cmd_mag.tolist(), applied_mag.tolist(), realised.tolist()
                ve, bz = vel_err.tolist(), base_z.tolist()
                ee = est_err.tolist() if est_err is not None else [None] * n
                for i in range(n):
                    # The schedule is hashed in the policy-independent frames: the goal as
                    # arm-frame spherical coordinates (what the observation carries and what the
                    # trajectory is generated in, not the world position that follows the base),
                    # and the force command in the base-yaw frame.
                    schedules[i].update(",".join(
                        f"{v:.4f}" for v in (sph[i] + fcl[i] + vcm[i])).encode())
                    if fell_at[i] is None:
                        trace["goal_err"][i].append(ge[i])
                        trace["unified_err"][i].append(ue[i])
                        trace["force_cmd"][i].append(cm[i])
                        trace["force_applied"][i].append(am[i])
                        trace["force_along_cmd"][i].append(rl[i] if cm[i] > 1e-6 else None)
                        trace["est_err"][i].append(ee[i])
                        trace["base_vel_err"][i].append(ve[i])
                        trace["base_z"][i].append(bz[i])
    finally:
        env.check_termination = original_check

    records = []
    for i, spec in enumerate(episodes):
        quiet = {j for j, (c, a) in enumerate(zip(trace["force_cmd"][i], trace["force_applied"][i]))
                 if c < 0.1 and a < 0.1}
        pushed = [j for j in range(len(trace["force_cmd"][i])) if j not in quiet]
        records.append({
            "index": spec["index"],
            "steps": fell_at[i] if fell_at[i] is not None else max_steps,
            "fell": fell_at[i] is not None,
            "timed_out": fell_at[i] is None,
            "schedule_sha256": schedules[i].hexdigest(),
            "goal_tracking_quiet_m": _stats([trace["goal_err"][i][j] for j in sorted(quiet)]),
            "unified_tracking_m": _stats(trace["unified_err"][i]),
            "unified_tracking_pushed_m": _stats([trace["unified_err"][i][j] for j in pushed]),
            "force_cmd_n": _stats([trace["force_cmd"][i][j] for j in pushed]),
            "force_realised_n": _stats([trace["force_along_cmd"][i][j] for j in pushed]),
            "estimator_err_n": _stats(trace["est_err"][i]),
            "base_vel_err_m_s": _stats(trace["base_vel_err"][i]),
            "base_z_m": _stats(trace["base_z"][i]),
        })
        if progress:
            progress(records[-1])
    return records


def _pool(records, key, field="median"):
    """Pool one per-episode statistic across episodes."""
    return _stats([r[key][field] for r in records if r.get(key)])


def summarise(records, manifest, controller, conditions=None):
    episodes = len(records)
    falls = sum(1 for r in records if r["fell"])
    survived = sum(1 for r in records if r["timed_out"])

    # Every episode must have produced the schedule its seed implies.
    expected = {e["index"]: e.get("schedule_sha256") for e in manifest["episodes"]}
    drifted = [r["index"] for r in records
               if expected.get(r["index"]) and expected[r["index"]] != r["schedule_sha256"]]

    return {
        "controller": controller,
        "manifest_role": manifest["role"],
        "manifest_sha256": manifest["content_sha256"],
        "episodes": episodes,
        "falls": {"count": falls, "rate": falls / episodes if episodes else None},
        "survived_to_time_limit": survived,
        "schedule_mismatches": drifted,
        "goal_tracking_quiet_m": _pool(records, "goal_tracking_quiet_m"),
        "unified_tracking_m": _pool(records, "unified_tracking_m"),
        "unified_tracking_pushed_m": _pool(records, "unified_tracking_pushed_m"),
        "force_cmd_n": _pool(records, "force_cmd_n", "mean"),
        "force_realised_n": _pool(records, "force_realised_n", "mean"),
        "estimator_err_n": _pool(records, "estimator_err_n"),
        "base_vel_err_m_s": _pool(records, "base_vel_err_m_s"),
        "base_z_m": _pool(records, "base_z_m"),
        "condition_mismatches": conditions or [],
    }

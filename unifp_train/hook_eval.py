"""Scoring a policy on the force-transmission task: how hard can it pull, and where.

Every episode is the same experiment: the goal goes to one of `hook_cfg.eval_handle_spheres()`,
the tool engages there with the axis straight back toward the robot (straight away for a press),
and the force command climbs `hook_cfg.EVAL_LEVELS_N`, each level ramped to and held for
`EVAL_HOLD_S`. Nothing is drawn, so two policies see identical demands; the only randomness left is
UniFP's per-robot goal cadence, which depends on the seed and the environment count, both recorded.

A level is **held** when the force applied along the axis, averaged over the last `EVAL_WINDOW_S` of
its hold, reaches `EVAL_HELD_FRACTION` of it and the contact and the robot both survive the hold.
An episode's **sustained force** is the highest level held with every level below it held too --
a policy that holds 40 N but dropped 30 N on the way up is credited with 20.

Also recorded per level, because the thesis claim is about *how* the force is made, not only how
much: the arm's worst joint load as a fraction of its limit, the base pitch and height, and the
adaptation module's own estimate of the contact force against the true one.

The recording and the scoring are separate so the scoring can be tested without a simulator.
"""
from __future__ import annotations

import math

import numpy as np
import torch

from unifp_isaaclab import interface

from . import hook_cfg, observations

#: Per-step quantities recorded for each environment. All (steps, envs).
FIELDS = ("engaged", "lost", "lost_how", "fallen", "stage", "level", "applied_along", "applied_err",
          "arm_load", "arm_loads", "arm_limit_gap", "arm_contact", "body_contact", "slide_up", "slide_across",
          "pitch", "height",
          "estimate_along", "estimate_err")


def build_handles(repeats: int, device) -> torch.Tensor:
    """(15 * repeats, 3) goal-sphere placements: every placement, `repeats` times, in blocks."""
    base = torch.tensor(hook_cfg.eval_handle_spheres(), dtype=torch.float32, device=device)
    return base.repeat(repeats, 1)


def record_step(env, policy_estimate: torch.Tensor | None, done: torch.Tensor,
                ended: torch.Tensor) -> dict:
    """One step's measurements, as numpy arrays over environments.

    Call after `env.step`. An environment that terminated on this step has already been reset by
    the time this runs, so its state is the next episode's: `ended` marks those, and they are
    recorded only as how they ended (`env._term_lost` / `env._term_fallen`), with every other field
    NaN. `done` marks episodes that ended earlier, which are all NaN.
    """
    fx = env._fixture
    applied = fx.applied_by_robot()
    command = env._fixture_command_w()
    limits = torch.as_tensor(interface.TORQUE_LIMITS[12:interface.NUM_ACTIONS], device=env.device)
    arm_loads = env._robot.data.applied_torque[:, env._order][:, 12:interface.NUM_ACTIONS].abs() / limits
    arm_load = arm_loads.max(dim=1).values
    roll_pitch = interface.body_roll_pitch(env._robot.data.root_quat_w)
    height = env._robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    out = {
        "engaged": fx.engaged.float(), "lost": torch.zeros_like(arm_load), "fallen": torch.zeros_like(arm_load),
        "stage": env._levels.stage.float(), "level": env._levels.level,
        "applied_along": (applied * fx.axis).sum(-1), "applied_err": (applied - command).norm(dim=-1),
        "arm_load": arm_load, "pitch": roll_pitch[:, 1], "height": height,
        # How far the contact point has slid from where it landed, split into up/down and the
        # horizontal direction across the axis: which way a pad or claw is coming off.
        "slide_up": (fx.anchor - fx.origin)[:, 2],
        "slide_across": ((fx.anchor - fx.origin)[:, :2]).norm(dim=-1),
    }
    if policy_estimate is not None:
        # The decoder's force block is the reaction *on the tool*, x0.01, in the yaw frame; the
        # force the robot applies is its negative.
        start = observations.offsets()["ee_force"][0]
        on_tool_yaw = policy_estimate[:, start:start + 3] / observations.OBS_SCALE_EE_FORCE
        estimate = -interface.quat_apply(env._base_yaw_quat(), on_tool_yaw)
        out["estimate_along"] = (estimate * fx.axis).sum(-1)
        out["estimate_err"] = (estimate - applied).norm(dim=-1)
    else:
        out["estimate_along"] = torch.full_like(arm_load, float("nan"))
        out["estimate_err"] = torch.full_like(arm_load, float("nan"))
    blank = (done | ended).cpu().numpy()
    result = {k: np.where(blank, np.nan, v.detach().float().cpu().numpy()) for k, v in out.items()}
    # Per joint, (envs, 6): which joint is carrying the load, not only how much the worst one is.
    result["arm_loads"] = np.where(blank[:, None], np.nan, arm_loads.detach().float().cpu().numpy())
    # And how far each arm joint is from its nearest position limit, rad: a joint on its limit passes
    # load through the limit constraint rather than its motor, which PhysX allows and a real servo
    # with only soft limits would not.
    arm_ids = env._order[12:interface.NUM_ACTIONS]
    q = env._robot.data.joint_pos[:, arm_ids]
    limits = env._robot.data.joint_pos_limits[:, arm_ids]
    gap = torch.minimum(q - limits[..., 0], limits[..., 1] - q)
    result["arm_limit_gap"] = np.where(blank[:, None], np.nan, gap.detach().float().cpu().numpy())
    # Contact on the arm's own links, and on the trunk, N: a load path around the motors -- the arm
    # braced against the robot's own body or the ground -- shows up here and nowhere else.
    arm_forces = env._arm_contact.data.net_forces_w.norm(dim=-1)
    result["arm_contact"] = np.where(blank, np.nan, arm_forces.max(dim=1).values.cpu().numpy())
    names = env._contact_sensor.body_names
    trunk = [i for i, n in enumerate(names) if n == "base" or "Head" in n]
    trunk_forces = env._contact_sensor.data.net_forces_w[:, trunk].norm(dim=-1)
    result["body_contact"] = np.where(blank, np.nan, trunk_forces.max(dim=1).values.cpu().numpy())
    ended_np = ended.cpu().numpy()
    result["lost"] = np.where(ended_np, env._term_lost.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    result["fallen"] = np.where(ended_np, env._term_fallen.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    # How it was lost (fixture.ContactFixture.LOST_*), on the step it ended; 0 otherwise.
    result["lost_how"] = np.where(ended_np, env._term_lost_how.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    return result


def score(trace: dict, levels=hook_cfg.EVAL_LEVELS_N, hold_steps: int | None = None,
          window_steps: int | None = None, handles_per_repeat: int | None = None) -> dict:
    """Per-episode and per-level results from a recorded trace (dict of (steps, envs) arrays)."""
    hold_steps = hold_steps or math.ceil(hook_cfg.EVAL_HOLD_S / interface.POLICY_DT)
    window_steps = window_steps or math.ceil(hook_cfg.EVAL_WINDOW_S / interface.POLICY_DT)
    placements = hook_cfg.eval_handle_spheres()
    handles_per_repeat = handles_per_repeat or len(placements)
    stage = trace["stage"]
    num_envs = stage.shape[1]
    episodes = []
    for env in range(num_envs):
        engaged = np.nan_to_num(trace["engaged"][:, env]) > 0.5
        lost_at = np.flatnonzero(np.nan_to_num(trace["lost"][:, env]) > 0.5)
        fell_at = np.flatnonzero(np.nan_to_num(trace["fallen"][:, env]) > 0.5)
        per_level, sustained, broken = [], 0.0, False
        for k, level in enumerate(levels):
            steps = np.flatnonzero((stage[:, env] == k + 1) & engaged)
            # The hold survived if the stage ran its full length while engaged.
            complete = len(steps) >= hold_steps - 1
            window = steps[-window_steps:] if complete else steps[:0]
            mean_along = float(np.nanmean(trace["applied_along"][window, env])) if len(window) else float("nan")
            held = bool(complete and mean_along >= hook_cfg.EVAL_HELD_FRACTION * level)
            row = {"level_n": level, "reached": bool(len(steps) > 0), "complete": complete, "held": held,
                   "applied_n": round(mean_along, 2) if len(window) else None}
            if len(window):
                row.update({
                    "error_n": round(float(np.nanmean(trace["applied_err"][window, env])), 2),
                    "arm_load_max": round(float(np.nanmax(trace["arm_load"][window, env])), 3),
                    "pitch_deg": round(math.degrees(float(np.nanmean(trace["pitch"][window, env]))), 2),
                    "height_m": round(float(np.nanmean(trace["height"][window, env])), 4),
                })
                est = trace["estimate_along"][window, env]
                if np.isfinite(est).any():
                    row["estimate_n"] = round(float(np.nanmean(est)), 2)
                    row["estimate_err_n"] = round(float(np.nanmean(trace["estimate_err"][window, env])), 2)
            per_level.append(row)
            if held and not broken:
                sustained = level
            elif not held:
                broken = True
        placement = placements[env % handles_per_repeat]
        height = hook_cfg.GOAL_CENTRE_HEIGHT_M + placement[0] * math.sin(placement[1])
        how = trace["lost_how"][lost_at[0], env] if len(lost_at) and "lost_how" in trace else 0
        episodes.append({
            "env": env, "handle_height_m": round(height, 3),
            "lost_how": {1: "backed off", 2: "slid off", 3: "lifted off"}.get(int(how)) if len(lost_at) else None,
            "handle_bearing_rad": placement[2], "engaged": bool(engaged.any()),
            "lost_step": int(lost_at[0]) if len(lost_at) else None,
            "fell_step": int(fell_at[0]) if len(fell_at) else None,
            "sustained_n": sustained, "levels": per_level,
        })

    sustained = np.array([e["sustained_n"] for e in episodes])
    summary = {
        "episodes": num_envs,
        "engaged": int(sum(e["engaged"] for e in episodes)),
        "lost": int(sum(e["lost_step"] is not None for e in episodes)),
        "fell": int(sum(e["fell_step"] is not None for e in episodes)),
        "sustained_n": {"median": float(np.median(sustained)), "p10": float(np.percentile(sustained, 10)),
                        "p90": float(np.percentile(sustained, 90)), "max": float(sustained.max())},
        "by_level": [], "by_height": [],
    }
    for k, level in enumerate(levels):
        rows = [e["levels"][k] for e in episodes]
        got = [r for r in rows if r["applied_n"] is not None]
        entry = {"level_n": level, "held": int(sum(r["held"] for r in rows)),
                 "of": num_envs}
        if got:
            for key in ("applied_n", "error_n", "arm_load_max", "pitch_deg", "height_m", "estimate_n", "estimate_err_n"):
                values = [r[key] for r in got if r.get(key) is not None]
                if values:
                    entry[key] = round(float(np.median(values)), 3)
        summary["by_level"].append(entry)
    for h in sorted({e["handle_height_m"] for e in episodes}):
        values = [e["sustained_n"] for e in episodes if e["handle_height_m"] == h]
        summary["by_height"].append({"handle_height_m": h, "sustained_n_median": float(np.median(values)),
                                     "episodes": len(values)})
    return {"summary": summary, "episodes": episodes}

"""Scoring a policy on the goal-commanded task: which mechanisms does it open, how, and at what cost.

Every episode is one fixed experiment: the goal goes to one of the pull task's fifteen handle
placements (`hook_cfg.eval_handle_spheres()`), the tool grasps the handle there, and a mechanism of
one of `mech_cfg.EVAL_KINDS` is built around it with its peak resistance at one of
`mech_cfg.EVAL_LEVELS_N`. The reference then slides to fully open at `EVAL_SPEED_M_S` and stays.
Nothing is drawn, so two policies meet identical mechanisms; the remaining randomness is UniFP's
per-robot goal cadence (seed and environment count, both recorded).

An episode **opened** when the handle reached `EVAL_OPENED_FRACTION` of its travel while still in the
claw and before any fall. A kind's **capacity** is the highest level at which at least
`CAPACITY_FRACTION` of the placements opened, with every lower level doing so too.

Also per episode, because the claim is about *how*: the largest force the robot drove the handle
with before it opened, the arm's worst joint load, the base's pitch at that moment and how far the
base moved, the handle's peak speed (the lunge when a latch lets go), and the force estimator's
reading against the true drive.

The recording and the scoring are separate so the scoring can be tested without a simulator.
"""
from __future__ import annotations

import math

import numpy as np
import torch

from unifp_isaaclab import interface

from . import hook_cfg, mech_cfg, observations

#: Per-step quantities recorded for each environment. All (steps, envs) except `arm_loads` (steps, envs, 6).
FIELDS = ("grasped", "torn", "fallen", "released", "s", "v", "ref", "travel", "drive", "force",
          "arm_load", "arm_loads", "pitch", "height", "base_x", "base_y", "estimate_drive",
          "arm_contact", "body_contact")
CAPACITY_FRACTION = 0.8


def build_plan(kinds: list[str], levels, device,
               placements: list | None = None) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """(handles, kind indices, levels), one row per episode: kind slowest, then level, then placement.

    `placements` defaults to the development grid (`hook_cfg.eval_handle_spheres()`); the held-out test set
    passes `mech_cfg.test_handle_spheres()`."""
    names = list(mech_cfg.EVAL_KINDS)
    placements = hook_cfg.eval_handle_spheres() if placements is None else placements
    handles, kind_ids, level_values = [], [], []
    for kind in kinds:
        for level in levels:
            for placement in placements:
                handles.append(placement)
                kind_ids.append(names.index(kind))
                level_values.append(float(level))
    return (torch.tensor(handles, dtype=torch.float32, device=device),
            torch.tensor(kind_ids, dtype=torch.long, device=device),
            torch.tensor(level_values, dtype=torch.float32, device=device))


def record_step(env, policy_estimate: torch.Tensor | None, done: torch.Tensor, ended: torch.Tensor) -> dict:
    """One step's measurements, as numpy arrays over environments. Same conventions as
    `hook_eval.record_step`: an environment that ended on this step is recorded only by how it
    ended; one that ended earlier is all NaN."""
    m = env._mech
    limits = torch.as_tensor(interface.TORQUE_LIMITS[12:interface.NUM_ACTIONS], device=env.device)
    arm_loads = env._robot.data.applied_torque[:, env._order][:, 12:interface.NUM_ACTIONS].abs() / limits
    roll_pitch = interface.body_roll_pitch(env._robot.data.root_quat_w)
    base = env._robot.data.root_pos_w - env.scene.env_origins
    tangent = m.tangent()
    out = {
        "grasped": m.grasped.float(), "released": m.released.float(),
        "s": m.s, "v": m.v, "ref": env._travel_goal.ref, "travel": m.travel,
        "drive": m.drive_n, "force": m.force_on_tool.norm(dim=-1),
        "arm_load": arm_loads.max(dim=1).values, "pitch": roll_pitch[:, 1], "height": base[:, 2],
        "base_x": base[:, 0], "base_y": base[:, 1],
    }
    if policy_estimate is not None:
        start = observations.offsets()["ee_force"][0]
        on_tool_yaw = policy_estimate[:, start:start + 3] / observations.OBS_SCALE_EE_FORCE
        applied = -interface.quat_apply(env._base_yaw_quat(), on_tool_yaw)
        out["estimate_drive"] = (applied * tangent).sum(-1)
    else:
        out["estimate_drive"] = torch.full_like(m.s, float("nan"))
    blank = (done | ended).cpu().numpy()
    result = {k: np.where(blank, np.nan, v.detach().float().cpu().numpy()) for k, v in out.items()}
    result["arm_loads"] = np.where(blank[:, None], np.nan, arm_loads.detach().float().cpu().numpy())
    result["arm_contact"] = np.where(blank, np.nan, env._arm_contact.data.net_forces_w.norm(dim=-1).max(dim=1).values.cpu().numpy())
    names = env._contact_sensor.body_names
    trunk = [i for i, n in enumerate(names) if n == "base" or "Head" in n]
    result["body_contact"] = np.where(blank, np.nan, env._contact_sensor.data.net_forces_w[:, trunk].norm(dim=-1).max(dim=1).values.cpu().numpy())
    ended_np = ended.cpu().numpy()
    result["torn"] = np.where(ended_np, env._term_torn.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    result["fallen"] = np.where(ended_np, env._term_fallen.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    return result


def _first(mask: np.ndarray) -> int | None:
    hits = np.flatnonzero(mask)
    return int(hits[0]) if len(hits) else None


def score_episode(trace: dict, env: int, dt: float = interface.POLICY_DT) -> dict:
    """One episode's result from a recorded trace."""
    grasped = np.nan_to_num(trace["grasped"][:, env]) > 0.5
    s, travel = trace["s"][:, env], trace["travel"][:, env]
    grasp_step = _first(grasped)
    fell_step = _first(np.nan_to_num(trace["fallen"][:, env]) > 0.5)
    torn_step = _first(np.nan_to_num(trace["torn"][:, env]) > 0.5)
    out = {"grasp_step": grasp_step, "fell_step": fell_step, "torn_step": torn_step, "opened": False,
           "open_time_s": None}
    if grasp_step is None:
        return out
    full = float(np.nanmax(travel[grasped]))
    out["travel_m"] = round(full, 4)
    open_step = _first(grasped & (np.nan_to_num(s) >= mech_cfg.EVAL_OPENED_FRACTION * full))
    out["opened"] = open_step is not None
    if open_step is not None:
        out["open_time_s"] = round((open_step - grasp_step) * dt, 3)
    finite = np.flatnonzero(grasped)
    last = finite[-1]
    out["final_fraction"] = round(float(s[last] / full), 3) if full > 0 else None
    # The push phase: from the grasp until it opened, or until the end if it never did.
    stop = open_step if open_step is not None else last
    window = np.arange(grasp_step, stop + 1)
    drive = trace["drive"][window, env]
    peak_at = window[int(np.nanargmax(drive))]
    out["peak_drive_n"] = round(float(np.nanmax(drive)), 2)
    out["pitch_at_peak_deg"] = round(math.degrees(float(trace["pitch"][peak_at, env])), 2)
    out["arm_load_p95"] = round(float(np.nanpercentile(trace["arm_load"][window, env], 95)), 3)
    out["arm_saturated_fraction"] = round(float(np.nanmean(trace["arm_load"][window, env] >= 0.99)), 3)
    out["joint_load_p95"] = [round(float(v), 3) for v in np.nanpercentile(trace["arm_loads"][window, env], 95, axis=0)]
    moved = np.hypot(trace["base_x"][peak_at, env] - trace["base_x"][grasp_step, env],
                     trace["base_y"][peak_at, env] - trace["base_y"][grasp_step, env])
    out["base_moved_at_peak_m"] = round(float(moved), 4)
    estimate = trace["estimate_drive"][peak_at, env]
    out["estimate_at_peak_n"] = None if not np.isfinite(estimate) else round(float(estimate), 2)
    released = _first(np.nan_to_num(trace["released"][:, env]) > 0.5)
    out["released_step"] = released
    after = np.arange(grasp_step, last + 1)
    out["peak_speed_m_s"] = round(float(np.nanmax(np.abs(trace["v"][after, env]))), 3)
    out["arm_contact_max_n"] = round(float(np.nanmax(trace["arm_contact"][after, env])), 2)
    out["body_contact_max_n"] = round(float(np.nanmax(trace["body_contact"][after, env])), 2)
    return out


def score(trace: dict, kinds: list[str], levels, placements_per_level: int | None = None,
          placements: list | None = None) -> dict:
    """Per-episode results and a per-kind, per-level summary. The plan is `build_plan(kinds, levels, placements)`."""
    placements = hook_cfg.eval_handle_spheres() if placements is None else placements
    per_level = placements_per_level or len(placements)
    episodes = []
    index = 0
    for kind in kinds:
        for level in levels:
            for p in range(per_level):
                result = score_episode(trace, index)
                placement = placements[p % len(placements)]
                result.update({"env": index, "kind": kind, "level_n": float(level),
                               "handle_height_m": round(hook_cfg.GOAL_CENTRE_HEIGHT_M
                                                        + placement[0] * math.sin(placement[1]), 3),
                               "handle_bearing_rad": placement[2]})
                episodes.append(result)
                index += 1

    summary = {"episodes": len(episodes), "kinds": {}}
    median = lambda values: round(float(np.median(values)), 3) if values else None
    for kind in kinds:
        rows, capacity, broken = [], 0.0, False
        for level in levels:
            cell = [e for e in episodes if e["kind"] == kind and e["level_n"] == float(level)]
            opened = [e for e in cell if e["opened"]]
            row = {"level_n": float(level), "opened": len(opened), "of": len(cell),
                   "fell": sum(e["fell_step"] is not None for e in cell),
                   "torn": sum(e["torn_step"] is not None for e in cell),
                   "open_time_s": median([e["open_time_s"] for e in opened]),
                   "peak_drive_n": median([e["peak_drive_n"] for e in cell if "peak_drive_n" in e]),
                   "arm_load_p95": median([e["arm_load_p95"] for e in cell if "arm_load_p95" in e]),
                   "pitch_at_peak_deg": median([e["pitch_at_peak_deg"] for e in cell if "pitch_at_peak_deg" in e]),
                   "base_moved_at_peak_m": median([e["base_moved_at_peak_m"] for e in cell if "base_moved_at_peak_m" in e]),
                   "peak_speed_m_s": median([e["peak_speed_m_s"] for e in cell if "peak_speed_m_s" in e]),
                   "estimate_at_peak_n": median([e["estimate_at_peak_n"] for e in cell
                                                 if e.get("estimate_at_peak_n") is not None])}
            rows.append(row)
            ok = len(cell) > 0 and len(opened) >= CAPACITY_FRACTION * len(cell)
            if ok and not broken:
                capacity = float(level)
            elif not ok:
                broken = True
        summary["kinds"][kind] = {"capacity_n": capacity, "by_level": rows,
                                  "opened": sum(r["opened"] for r in rows),
                                  "of": sum(r["of"] for r in rows),
                                  "fell": sum(r["fell"] for r in rows), "torn": sum(r["torn"] for r in rows)}
    return {"summary": summary, "episodes": episodes}

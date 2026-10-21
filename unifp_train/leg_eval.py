"""The front-right-leg task's frozen evaluation: 144 door handles, each turned once, and how it went.

The set (`build_plan`): 3 door distances x 3 sideways placements of the contact point x 2 lever directions x 4 spring
strengths (1-4 N·m at the end stop) x 2 spindle heights, every other property at its training mid-range, and the
task layer's estimate wrong by errors drawn once from `leg_cfg.EVAL_SEED` at the training ranges (the door's yaw too).
Frozen on 2026-10-20 at 03:30, replacing a first set (doors 0.55-0.61 m, contacts -0.18 to -0.08 m) on which two pilot
checkpoints had been scored and whose pre-contact points were out of the FR leg's reach in 48% of handles. No final
policy was scored on the first set.

Each episode is scored on the plan's terms (`docs/thesis_b_plan.md`): **success** is the latch held drawn back for
`leg_cfg.SUCCESS_OPEN_S` without a break, no damage (`leg_cfg.DAMAGE_N` at the physics rate), no fall. Also reported:
the time from the layer's start to the latch opening, the physics-rate peak force, the force held while the latch is
open, the force the foot puts into the end stop, the force along the turn against the force-aware layer's command,
slips off the bar, the door's face, and one failure reason: fell, damaged, missed (the foot never settled above the
bar), slipped (off the bar and never opened), stalled (on the bar and never opened), not held (opened, not for 1 s).
"""
from __future__ import annotations

import math

import numpy as np
import torch

from unifp_isaaclab import interface

from . import handle, leg_cfg

FIELDS = ("stage", "phase", "theta", "theta_ref", "latch_open", "on_bar", "touched", "peak_n", "contact_n", "door_n",
          "drive_n", "cmd_n", "foot_err", "base_z", "roll", "pitch", "fallen", "damaged", "stance_max_cm", "stance_fr_cm")
#: The legs' joint torques and speeds (2026-10-21, `leg_actuators`): the front-right leg's three joints, applied (after the
#: actuator's limits) and demanded (the PD law's, before them), and their speeds; the three support legs' largest of each
#: joint kind; whether any leg joint's demand was clipped. The last physics substep of each policy step (50 Hz), so a
#: peak between samples can be missed. Older traces lack them; `score_episode` scores them when present.
JOINTS = ("hip", "thigh", "calf")
TORQUE_FIELDS = (tuple(f"{k}_fr_{j}" for k in ("tau", "dem", "qd") for j in JOINTS)
                 + tuple(f"{k}_sup_{j}" for k in ("tau", "dem", "qd") for j in JOINTS) + ("sat_fr", "sat_legs"))
#: A joint is clipped when its demand exceeds what was applied by more than this (N·m).
SATURATED_NM = 0.5
#: Isaac Lab's stock Go2 knee cap and where the geared knee's torque starts to fall (`leg_actuators`).
STOCK_KNEE_NM = 23.5
GEARED_KNEE_FALL_RAD_S = 7.04


def build_key_plan(device) -> tuple[dict, list[dict]]:
    """The key's frozen set (2026-10-21, `leg_cfg.KEY_EVAL_*`): 108 keys, each turned once from 45 degrees up."""
    c = leg_cfg
    rows = []
    for ahead in c.KEY_EVAL_DOOR_AHEAD_M:
        for side in c.KEY_EVAL_AXIS_SIDE_M:
            for friction in c.KEY_EVAL_FRICTION_NM:
                for detent in c.KEY_EVAL_DETENT_NM:
                    for height in c.KEY_EVAL_AXIS_HEIGHT_M:
                        rows.append({"door_ahead": ahead, "axis_side": side, "friction_nm": friction,
                                     "detent_nm": detent, "height": height})
    n = len(rows)
    g = torch.Generator().manual_seed(c.KEY_EVAL_SEED)
    sym = lambda a: ((2 * torch.rand(n, generator=g) - 1) * a)
    mid = lambda r: torch.full((n,), 0.5 * (r[0] + r[1]))
    col = lambda key: torch.tensor([r[key] for r in rows], dtype=torch.float32)
    keys = {"door_ahead": col("door_ahead"), "door_yaw": sym(0.12), "height": col("height"),
            "axis_side": col("axis_side"), "inertia": mid(c.KEY_INERTIA_KG_M2), "friction": col("friction_nm"),
            "detent": col("detent_nm"), "damping": mid(c.KEY_DAMPING), "mu": mid(c.KEY_MU), "reach": mid(c.KEY_REACH_M),
            "err_side": sym(c.KEY_EST_ERROR_SIDE_M), "err_up": sym(c.KEY_EST_ERROR_UP_M),
            "err_depth": sym(c.KEY_EST_ERROR_DEPTH_M), "err_rating": sym(c.KEY_EST_ERROR_RATING),
            "descent": mid(c.KEY_DESCENT_M), "descend_speed": mid(c.DESCEND_SPEED_M_S), "turn_speed": mid(c.TURN_SPEED_RAD_S)}
    reach = float(keys["reach"][0])
    for i, row in enumerate(rows):
        row.update({k: round(float(keys[k][i]), 5) for k in ("door_yaw", "err_side", "err_up", "err_depth", "err_rating")})
        row["theta_max_deg"] = round(math.degrees(c.KEY_THETA_END_RAD), 3)
        row["latch_deg"] = round(math.degrees(c.KEY_THETA_UNLOCK_RAD), 3)
        row["full_nm"] = round(row["friction_nm"] + row["detent_nm"], 3)       # what turning it takes, at its hardest
        row["required_n"] = round((row["full_nm"] - c.KEY_GRAVITY_NM * math.cos(math.radians(10.0))) / reach, 2)
        row["key"] = True
    return {"handles": {k: v.to(device) for k, v in keys.items()}}, rows


def build_plan(device) -> tuple[dict, list[dict]]:
    rows = []
    for ahead in leg_cfg.EVAL_DOOR_AHEAD_M:
        for side in leg_cfg.EVAL_CONTACT_SIDE_M:
            for left in leg_cfg.EVAL_LEVER_LEFT:
                for full in leg_cfg.EVAL_FULL_NM:
                    for height in leg_cfg.EVAL_SPINDLE_HEIGHT_M:
                        rows.append({"door_ahead": ahead, "contact_side": side, "left": left, "full_nm": full,
                                     "height": height})
    n = len(rows)
    g = torch.Generator().manual_seed(leg_cfg.EVAL_SEED)
    sym = lambda a: ((2 * torch.rand(n, generator=g) - 1) * a)
    mid = lambda r: torch.full((n,), 0.5 * (r[0] + r[1]))
    col = lambda key, dtype=torch.float32: torch.tensor([r[key] for r in rows], dtype=dtype)
    handles = {
        "door_ahead": col("door_ahead"), "door_yaw": sym(0.20), "height": col("height"),
        "contact_side": col("contact_side"), "left": col("left", torch.bool),
        "length": mid(leg_cfg.LEVER_LENGTH_M), "radius": mid(leg_cfg.LEVER_RADIUS_M),
        "standoff": mid(leg_cfg.LEVER_STANDOFF_M), "inertia": mid(leg_cfg.LEVER_INERTIA_KG_M2),
        "rest_nm": torch.full((n,), 0.5), "full_nm": col("full_nm"), "damping": mid(leg_cfg.SPINDLE_DAMPING),
        "friction": mid(leg_cfg.SPINDLE_FRICTION_NM), "theta_max": mid(leg_cfg.THETA_MAX_RAD),
        "latch_fraction": mid(leg_cfg.LATCH_FRACTION), "mu": mid(leg_cfg.FOOT_MU),
        "err_side": sym(leg_cfg.EST_ERROR_SIDE_M), "err_up": sym(leg_cfg.EST_ERROR_UP_M),
        "err_depth": sym(leg_cfg.EST_ERROR_DEPTH_M), "err_length": sym(leg_cfg.EST_ERROR_LENGTH),
        "err_theta": sym(leg_cfg.EST_ERROR_THETA), "err_rating": sym(leg_cfg.EST_ERROR_RATING),
        "descent": mid(leg_cfg.DESCENT_M), "descend_speed": mid(leg_cfg.DESCEND_SPEED_M_S),
        "turn_speed": mid(leg_cfg.TURN_SPEED_RAD_S),
    }
    for i, row in enumerate(rows):
        row.update({k: round(float(handles[k][i]), 5) for k in ("door_yaw", "err_side", "err_up", "err_depth",
                                                                 "err_length", "err_theta", "err_rating")})
        row["theta_max_deg"] = round(math.degrees(float(handles["theta_max"][i])), 3)
        row["latch_deg"] = round(math.degrees(float(handles["theta_max"][i] * handles["latch_fraction"][i])), 3)
    plan = {"handles": {k: v.to(device) for k, v in handles.items()}}
    return plan, rows


def record_step(env, done: torch.Tensor, ended: torch.Tensor) -> dict:
    """One step's measurements, numpy over environments; an environment that ended earlier (or now) is NaN."""
    hd, layer = env._handle, env._layer
    foot, _ = env._foot_state()
    rp = interface.body_roll_pitch(env._robot.data.root_quat_w)
    out = {"stage": env._stage.float(), "phase": layer.phase.float(), "theta": hd.theta,
           "theta_ref": layer.theta_ref, "latch_open": hd.latch_open().float(), "on_bar": hd.on_bar.float(),
           "touched": hd.touched.float(), "peak_n": env._step_peak, "contact_n": hd.force_on_foot.norm(dim=-1),
           "door_n": hd.door_n + hd.knee_door_n, "drive_n": hd.drive_n, "cmd_n": layer.force_cmd.norm(dim=-1),
           "foot_err": (foot - env._goal_world()).norm(dim=-1),
           "base_z": env._robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2], "roll": rp[:, 0], "pitch": rp[:, 1]}
    from . import leg_rewards
    stance = leg_rewards.stance_error(
        env._robot.data.body_pos_w[:, env._feet_bodies] - env.scene.env_origins.unsqueeze(1),
        env._robot.data.root_pos_w - env.scene.env_origins, env._base_yaw_quat())
    out["stance_max_cm"] = 100.0 * stance[:, list(leg_cfg.SUPPORT_FEET)].max(dim=1).values
    out["stance_fr_cm"] = 100.0 * stance[:, leg_cfg.FR_FOOT_INDEX]
    legs = env._order[:12]
    tau, dem = env._robot.data.applied_torque[:, legs], env._robot.data.computed_torque[:, legs]
    qd = env._robot.data.joint_vel[:, legs]
    support = [3 * f for f in leg_cfg.SUPPORT_FEET]
    for k, j in enumerate(JOINTS):
        fr = leg_cfg.FR_JOINTS[k]
        for name, value in (("tau", tau), ("dem", dem), ("qd", qd)):
            out[f"{name}_fr_{j}"] = value[:, fr]
            out[f"{name}_sup_{j}"] = value[:, [s + k for s in support]].abs().max(dim=1).values
    clipped = dem.abs() - tau.abs() > SATURATED_NM
    out["sat_fr"] = clipped[:, list(leg_cfg.FR_JOINTS)].any(dim=1).float()
    out["sat_legs"] = clipped.any(dim=1).float()
    blank = (done | ended).cpu().numpy()
    result = {k: np.where(blank, np.nan, v.detach().float().cpu().numpy()) for k, v in out.items()}
    ended_np = ended.cpu().numpy()
    result["fallen"] = np.where(ended_np, env._term_fallen.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    result["damaged"] = np.where(ended_np, env._term_damage.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    return result


def _first(mask: np.ndarray) -> int | None:
    hits = np.flatnonzero(mask)
    return int(hits[0]) if len(hits) else None


def _longest_run(mask: np.ndarray) -> int:
    best = run = 0
    for value in mask:
        run = run + 1 if value else 0
        best = max(best, run)
    return best


def _r(value, digits=3):
    return None if value is None or (isinstance(value, float) and not math.isfinite(value)) else round(float(value), digits)


def score_episode(trace: dict, env: int, row: dict, dt: float = interface.POLICY_DT) -> dict:
    col = {k: trace[k][:, env] for k in FIELDS}
    alive = np.isfinite(col["stage"])
    stage = np.where(alive, col["stage"], -1)
    phase = np.where(alive, col["phase"], -1)
    engaged = stage == 4
    engage_step = _first(engaged)
    opened = np.nan_to_num(col["latch_open"]) > 0.5
    pushing = engaged & ((phase == handle.PRESSING) | (phase == handle.HOLDING))
    on_bar = np.nan_to_num(col["on_bar"]) > 0.5
    touched = np.nan_to_num(col["touched"]) > 0.5
    fell = bool(np.nansum(col["fallen"]) > 0)
    damaged = bool(np.nansum(col["damaged"]) > 0)
    longest = _longest_run(opened) * dt
    success = longest >= leg_cfg.SUCCESS_OPEN_S - 1e-9 and not fell and not damaged
    first_open = _first(opened)
    off = pushing & touched & ~on_bar
    slipped = _longest_run(off) * dt >= leg_cfg.SLIP_S
    # NaN once the episode has ended. Not filled with 0: a key starts at -45 degrees, so 0 would read as a turn.
    theta = np.where(np.isfinite(col["theta"]), col["theta"], -np.inf)
    at_stop = theta >= math.radians(row["theta_max_deg"] - 1.0)
    contact = np.nan_to_num(col["contact_n"])
    cmd = np.nan_to_num(col["cmd_n"])
    drive = np.nan_to_num(col["drive_n"])
    tracked = pushing & on_bar & (cmd > 0.0)
    if success:
        reason = "ok"
    elif fell:
        reason = "fell"
    elif damaged:
        reason = "damaged"
    elif engage_step is None:
        reason = "missed"
    elif first_open is None:
        reason = "slipped" if slipped else "stalled"
    else:
        reason = "not held"
    roll, pitch = np.nan_to_num(col["roll"]), np.nan_to_num(col["pitch"])
    stance = col["stance_max_cm"]
    down = stage == 7                                                     # leg_env.FOOT_DOWN (keys)
    extra = {"stance_median_cm": _r(float(np.nanmedian(stance)), 2) if np.isfinite(stance).any() else None,
             "stance_max_cm": _r(float(np.nanmax(stance)), 2) if np.isfinite(stance).any() else None}
    if all(k in trace for k in TORQUE_FIELDS):
        extra.update(_torque_extras({k: trace[k][:, env] for k in TORQUE_FIELDS}, alive, pushing, dt))
    if row.get("key"):
        last = np.flatnonzero(alive)
        extra.update({
            "foot_down": bool(down.any()),
            "fr_footfall_cm": _r(float(col["stance_fr_cm"][last[-1]]), 2) if down.any() and len(last) else None,
            "unlocked": first_open is not None,
            "time_to_unlock_s": _r((first_open - engage_step) * dt) if first_open is not None and engage_step is not None else None})
    return extra | {
        "success": bool(success), "reason": reason, "fell": fell, "damaged": damaged,
        "engaged": engage_step is not None, "opened": first_open is not None, "slipped": bool(slipped),
        "longest_open_s": _r(longest),
        "time_to_open_s": _r((first_open - engage_step) * dt) if first_open is not None and engage_step is not None else None,
        "max_theta_deg": _r(math.degrees(float(theta.max())), 2) if np.isfinite(theta.max()) else None,
        "peak_n": _r(float(np.nanmax(col["peak_n"])) if np.isfinite(col["peak_n"]).any() else 0.0, 2),
        "peak_at_stop_n": _r(float(contact[at_stop].max()) if at_stop.any() else 0.0, 2),
        "hold_force_n": _r(float(contact[opened].mean()) if opened.any() else None, 2),
        "required_n": _r(row["required_n"], 2),
        "force_rmse_n": _r(float(np.sqrt(np.mean(np.square(drive[tracked] - cmd[tracked])))) if tracked.any() else None, 2),
        "cmd_max_n": _r(float(cmd.max()), 2),
        "door_peak_n": _r(float(np.nanmax(col["door_n"])) if np.isfinite(col["door_n"]).any() else 0.0, 2),
        "max_tilt_deg": _r(math.degrees(float(np.max(np.abs(np.concatenate((roll, pitch)))))), 2),
        "foot_err_engaged_mm": _r(1000.0 * float(np.nanmedian(col["foot_err"][pushing])) if pushing.any() else None, 1),
    }


def _torque_extras(col: dict, alive: np.ndarray, pushing: np.ndarray, dt: float) -> dict:
    """One episode's joint torques and speeds (`TORQUE_FIELDS`): peaks (|N·m|, |rad/s|) and seconds past the limits."""
    def peak(x, mask=None):
        x = np.abs(x[alive if mask is None else (mask & alive)])
        return _r(float(np.nanmax(x)), 2) if x.size and np.isfinite(x).any() else None

    def seconds(mask):
        return _r(float(np.count_nonzero(mask & alive)) * dt, 2)

    out = {}
    for j in JOINTS:
        out[f"fr_{j}_peak_nm"] = peak(col[f"tau_fr_{j}"])
        out[f"fr_{j}_demand_peak_nm"] = peak(col[f"dem_fr_{j}"])
        out[f"fr_{j}_speed_peak"] = peak(col[f"qd_fr_{j}"])
        out[f"sup_{j}_peak_nm"] = peak(col[f"tau_sup_{j}"])
        out[f"sup_{j}_demand_peak_nm"] = peak(col[f"dem_sup_{j}"])
        out[f"sup_{j}_speed_peak"] = peak(col[f"qd_sup_{j}"])
    # While the foot presses the key round (the layer's PRESSING or HOLDING, engaged).
    out["fr_calf_turn_peak_nm"] = peak(col["tau_fr_calf"], pushing)
    out["fr_thigh_turn_peak_nm"] = peak(col["tau_fr_thigh"], pushing)
    over = lambda x: np.nan_to_num(np.abs(x)) > STOCK_KNEE_NM
    out["fr_calf_over_stock_s"] = seconds(over(col["tau_fr_calf"]))
    out["fr_calf_demand_over_stock_s"] = seconds(over(col["dem_fr_calf"]))
    out["sup_calf_over_stock_s"] = seconds(over(col["tau_sup_calf"]))
    out["fr_calf_fast_s"] = seconds(np.nan_to_num(np.abs(col["qd_fr_calf"])) > GEARED_KNEE_FALL_RAD_S)
    out["fr_saturated_s"] = seconds(np.nan_to_num(col["sat_fr"]) > 0.5)
    out["legs_saturated_s"] = seconds(np.nan_to_num(col["sat_legs"]) > 0.5)
    return out


def _torque_summary(episodes: list[dict]) -> dict:
    keys = [k for k in episodes[0] if k.startswith(("fr_", "sup_", "legs_")) and k.endswith(("_nm", "_s", "_speed_peak"))
            and k != "fr_footfall_cm"]
    out = {k: {"median": _median([e[k] for e in episodes]), "p90": _quantile([e[k] for e in episodes], 0.9),
               "max": _r(max((e[k] for e in episodes if e[k] is not None), default=float("nan")), 2)} for k in keys}
    out["episodes_fr_calf_over_stock"] = sum((e["fr_calf_peak_nm"] or 0.0) > STOCK_KNEE_NM for e in episodes)
    out["episodes_sup_calf_over_stock"] = sum((e["sup_calf_peak_nm"] or 0.0) > STOCK_KNEE_NM for e in episodes)
    ok = [e for e in episodes if e["success"]]
    out["successes"] = {k: _median([e[k] for e in ok]) for k in ("fr_calf_peak_nm", "fr_calf_turn_peak_nm",
                                                                 "fr_thigh_peak_nm", "sup_calf_peak_nm")} if ok else None
    return out


def score(trace: dict, rows: list[dict]) -> dict:
    episodes = []
    for i, row in enumerate(rows):
        row = dict(row)
        if not row.get("key"):
            # The force a foot at the layer's reach needs to hold the lever at the latch, from the true spring.
            theta_latch = math.radians(row["latch_deg"])
            theta_max = math.radians(row["theta_max_deg"])
            stiffness = (row["full_nm"] - 0.5) / theta_max
            reach = leg_cfg.REACH_FRACTION * 0.5 * sum(leg_cfg.LEVER_LENGTH_M)
            row["required_n"] = (0.5 + stiffness * theta_latch) / reach
        episodes.append({**row, **score_episode(trace, i, row)})

    def group(key):
        out = []
        for value in sorted({e[key] for e in episodes}, key=lambda v: (str(type(v)), v)):
            sub = [e for e in episodes if e[key] == value]
            out.append({key: value, "success": sum(e["success"] for e in sub), "of": len(sub),
                        "peak_n_median": _median([e["peak_n"] for e in sub]),
                        "peak_at_stop_n_median": _median([e["peak_at_stop_n"] for e in sub]),
                        "hold_force_n_median": _median([e["hold_force_n"] for e in sub]),
                        "time_to_open_s_median": _median([e["time_to_open_s"] for e in sub]),
                        "reasons": _reasons(sub)})
        return out

    summary = {
        "episodes": len(episodes), "success": sum(e["success"] for e in episodes),
        "engaged": sum(e["engaged"] for e in episodes), "opened": sum(e["opened"] for e in episodes),
        "fell": sum(e["fell"] for e in episodes), "damaged": sum(e["damaged"] for e in episodes),
        "slipped": sum(e["slipped"] for e in episodes), "reasons": _reasons(episodes),
        "peak_n_median": _median([e["peak_n"] for e in episodes]),
        "peak_n_p95": _quantile([e["peak_n"] for e in episodes], 0.95),
        "peak_at_stop_n_median": _median([e["peak_at_stop_n"] for e in episodes]),
        "hold_force_n_median": _median([e["hold_force_n"] for e in episodes]),
        "time_to_open_s_median": _median([e["time_to_open_s"] for e in episodes]),
        "force_rmse_n_median": _median([e["force_rmse_n"] for e in episodes]),
        "door_peak_n_median": _median([e["door_peak_n"] for e in episodes]),
        "max_tilt_deg_median": _median([e["max_tilt_deg"] for e in episodes]),
        "stance_median_cm": _median([e["stance_median_cm"] for e in episodes]),
        "stance_max_cm_median": _median([e["stance_max_cm"] for e in episodes]),
        "stance_max_cm_p90": _quantile([e["stance_max_cm"] for e in episodes], 0.9),
        "by_full_nm": group("full_nm"), "by_door_ahead": group("door_ahead"), "by_height": group("height"),
    }
    if rows and rows[0].get("key"):
        summary.update({"unlocked": sum(e["unlocked"] for e in episodes), "foot_down": sum(e["foot_down"] for e in episodes),
                        "success_and_foot_down": sum(e["success"] and e["foot_down"] for e in episodes),
                        "time_to_unlock_s_median": _median([e["time_to_unlock_s"] for e in episodes]),
                        "fr_footfall_cm_median": _median([e["fr_footfall_cm"] for e in episodes]),
                        "by_friction_nm": group("friction_nm"), "by_detent_nm": group("detent_nm"),
                        "by_axis_side": group("axis_side")})
    else:
        summary.update({"by_contact_side": group("contact_side"), "by_left": group("left")})
    if episodes and "fr_calf_peak_nm" in episodes[0]:
        summary["torques"] = _torque_summary(episodes)
    return {"summary": summary, "episodes": episodes}


def _median(values):
    v = [x for x in values if x is not None]
    return _r(float(np.median(v)), 3) if v else None


def _quantile(values, q):
    v = [x for x in values if x is not None]
    return _r(float(np.quantile(v, q)), 3) if v else None


def _reasons(episodes):
    out = {}
    for e in episodes:
        out[e["reason"]] = out.get(e["reason"], 0) + 1
    return out

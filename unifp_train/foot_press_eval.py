"""The foot-press task's frozen evaluation: 288 plates pressed once each, and how well the force was held.

The set (`build_plan`, `foot_press_cfg.EVAL_*`): 48 plates -- 12 of each kind, tilted, wall, level facing up, level facing
down -- drawn once from a plate table built with its own seed (`EVAL_TABLE_SEED`, not the training table's), x rigid or
sprung (1,000 N/m over 25 mm, then rigid) x a commanded level of 8, 16 or 28 N, one segment each (ramped over 0.5 s and
held 3 s), no tangential part, no slide. The layer's estimate is wrong by errors drawn once per plate at the training
ranges; friction, standoff and approach speed are mid-range.

This is the set's second version, frozen on 2026-10-21 at 23:00. The first (22:03-22:47) was drawn from the first plate
table, whose contact points sat at the outer shell of the leg's reach (`foot_press.PlateTable`); it scored the F-179
key policy twice and round 1c's `model_3000`, and is superseded -- those runs are recorded as on "plate set v1".

Each episode is scored on the force over the hold's last 2 s (`EVAL_SCORE_S`): its mean, bias and mean absolute error
against the level, the peak while pressing; **success** is the profile run to its end with the mean absolute error
within max(3 N, 20% of the level), no slip off the plate, no fall, no damage (`leg_cfg.DAMAGE_N` at the physics rate,
which ends a frozen episode). Also: whether the contact was found by the foot stalling, where on the plate the foot
pressed against where it was sent, the foot back down, the stance, the joints' torques. The position-only policy is
scored against the same levels: its layer asks for them as a goal `level / 400 N/m` past the contact.

The **slide** set (`--plate_set slide`, frozen 2026-10-22 02:45): the same 48 plates, rigid, 16 N, each pressed twice --
with a tangential part of 0.25 of the level one way along the plate (scored also on the tangential force held), and
sliding up to 4 cm toward the plate's centre at 2.5 cm/s while the level is held (scored also on how far the foot moved).
"""
from __future__ import annotations

import math

import numpy as np
import torch

from unifp_isaaclab import interface

from . import foot_press, foot_press_cfg as pc, handle, leg_cfg, leg_eval

FIELDS = ("stage", "phase", "normal_n", "level", "applied_n", "on_plate", "touched", "stalled", "peak_n", "knee_head_n",
          "foot_err", "plane_err", "engage_miss", "tangent_n", "tangent_cmd", "slide_s", "foot_along", "base_z", "roll", "pitch", "fallen", "damaged", "stance_max_cm", "stance_fr_cm")
TORQUE_FIELDS = leg_eval.TORQUE_FIELDS


def build_plan(device, plate_set: str = "v2") -> tuple[dict, list[dict]]:
    """`v2`: the 288 presses (module docstring); `slide`: the same 48 plates, rigid, 16 N, once with a tangential part and
    once sliding along the plate (`foot_press_cfg.EVAL_SLIDE_*`)."""
    if plate_set not in ("v2", "slide"):
        raise ValueError(f"plate set {plate_set!r}")
    table = foot_press.PlateTable(size=4000, seed=pc.EVAL_TABLE_SEED)
    cols = table.columns
    g = np.random.default_rng(pc.EVAL_SEED)
    picks = []
    for kind in range(len(pc.KINDS)):
        idx = np.flatnonzero(cols["kind"] == kind)
        picks += sorted(g.choice(idx, size=pc.EVAL_PLATES_PER_KIND, replace=False).tolist())
    m = len(picks)
    err_plane = g.uniform(-pc.EST_ERROR_PLANE_M, pc.EST_ERROR_PLANE_M, (m, 2))
    err_normal = g.uniform(-pc.EST_ERROR_NORMAL_M, pc.EST_ERROR_NORMAL_M, m)
    err_tilt = g.uniform(-pc.EST_ERROR_TILT_RAD, pc.EST_ERROR_TILT_RAD, (m, 2))
    way = g.uniform(0.0, 2 * math.pi, m)                   # drawn after the errors, so v2's draws are unchanged
    rows, sel = [], []
    conditions = ([(k, t, lv, "press") for k, t in pc.EVAL_STIFFNESS for lv in pc.EVAL_FORCES_N] if plate_set == "v2" else
                  [(pc.EVAL_STIFFNESS[0][0], 0.0, pc.EVAL_SLIDE_LEVEL_N, mode) for mode in ("tangent", "slide")])
    for j, i in enumerate(picks):
        for k_soft, travel, level, mode in conditions:
            if True:
                rows.append({"plate": j, "table_row": int(i), "kind": pc.KINDS[int(cols["kind"][i])],
                             "k_soft": k_soft, "travel_m": travel, "stiffness": "rigid" if travel == 0.0 else "sprung",
                             "level_n": level,
                             "contact_m": [round(float(v), 4) for v in cols["contact"][i]],
                             "normal": [round(float(v), 4) for v in cols["n"][i]],
                             "half_m": [round(float(v), 4) for v in cols["half"][i]],
                             "err_plane_m": [round(float(v), 4) for v in err_plane[j]],
                             "err_normal_m": round(float(err_normal[j]), 4),
                             "err_tilt_deg": [round(math.degrees(float(v)), 3) for v in err_tilt[j]], "mode": mode})
                sel.append((i, j, k_soft, travel, level, mode))
    n = len(rows)
    col = lambda name: torch.as_tensor(np.stack([cols[name][i] for i, *_ in sel]),
                                       dtype=torch.long if name == "kind" else torch.float32)
    full = lambda v, *shape: torch.full((n,) + shape, float(v))
    k = foot_press.MAX_SEGMENTS
    level = torch.tensor([s[4] for s in sel], dtype=torch.float32)
    t1 = col("t1")
    plan = {"contact": col("contact"), "face": col("face"), "n": col("n"), "t1": t1, "t2": col("t2"),
            "half": col("half"), "kind": col("kind"),
            "k_soft": torch.tensor([s[2] for s in sel], dtype=torch.float32),
            "travel": torch.tensor([s[3] for s in sel], dtype=torch.float32),
            "mu": full(0.5 * sum(pc.PLATE_MU)),
            "err_plane": torch.as_tensor(np.stack([err_plane[s[1]] for s in sel]), dtype=torch.float32),
            "err_normal": torch.as_tensor(np.array([err_normal[s[1]] for s in sel]), dtype=torch.float32),
            "err_tilt": torch.as_tensor(np.stack([err_tilt[s[1]] for s in sel]), dtype=torch.float32),
            "standoff": full(0.5 * sum(pc.STANDOFF_M)), "speed": full(0.5 * sum(pc.APPROACH_SPEED_M_S)),
            "levels": level.unsqueeze(-1).expand(n, k).clone(), "ramps": full(pc.EVAL_RAMP_S, k),
            "holds": full(pc.EVAL_HOLD_S, k), "segments": torch.ones(n, dtype=torch.long),
            "tan_ratio": full(0.0, k), "tan_dir": t1.clone(), "slide": torch.zeros(n, dtype=torch.bool),
            "slide_dir": t1.clone(), "slide_speed": full(0.0), "slide_dist": full(0.0)}
    if plate_set == "slide":
        t2 = plan["t2"]
        angle = torch.as_tensor(np.array([way[s[1]] for s in sel]), dtype=torch.float32).unsqueeze(-1)
        direction = torch.cos(angle) * t1 + torch.sin(angle) * t2
        tangent = torch.tensor([s[5] == "tangent" for s in sel])
        slide = torch.tensor([s[5] == "slide" for s in sel])
        plan["tan_ratio"] = torch.where(tangent.unsqueeze(-1), full(pc.EVAL_TANGENT_RATIO, k), full(0.0, k))
        plan["tan_dir"] = direction.clone()
        # The slide heads for the plate's centre and stops 1 cm short of the edge if the plate is too small for 4 cm.
        rel = plan["contact"] - plan["face"]
        u, v = (rel * t1).sum(-1), (rel * t2).sum(-1)
        off = torch.stack((u, v), -1)
        d = torch.where(off.norm(dim=-1, keepdim=True) > 0.01, -off / off.norm(dim=-1, keepdim=True).clamp(min=1e-9),
                        torch.tensor([[1.0, 0.0]]))
        half = plan["half"]
        room = torch.where(d.abs() > 1e-6, (half * torch.sign(d) - off) / torch.where(d.abs() > 1e-6, d, torch.ones_like(d)),
                           torch.full_like(d, 1e9)).min(dim=-1).values
        plan["slide_dir"] = d[:, :1] * t1 + d[:, 1:] * t2
        plan["slide"] = slide
        plan["slide_speed"] = full(pc.EVAL_SLIDE_SPEED_M_S)
        plan["slide_dist"] = torch.clamp(room - 0.01, min=0.01, max=pc.EVAL_SLIDE_M)
        for r, s_m, is_slide in zip(rows, plan["slide_dist"].tolist(), slide.tolist()):
            if is_slide:
                r["slide_m"] = round(s_m, 4)
    return {"handles": {key: v.to(device) for key, v in plan.items()}}, rows


def record_step(env, done: torch.Tensor, ended: torch.Tensor) -> dict:
    """One step's measurements, numpy over environments; an environment that ended earlier (or now) is NaN."""
    hd, layer = env._handle, env._layer
    foot, _ = env._foot_state()
    applied = -hd.force_on_foot
    rp = interface.body_roll_pitch(env._robot.data.root_quat_w)
    rel = foot - env._plate_true
    plane = torch.stack(((rel * hd.t1).sum(-1), (rel * hd.t2).sum(-1)), dim=-1).norm(dim=-1)
    out = {"stage": env._stage.float(), "phase": layer.phase.float(), "normal_n": (applied * -layer.n).sum(-1),
           "level": layer.normal_level, "applied_n": applied.norm(dim=-1), "on_plate": hd.on_bar.float(),
           "touched": hd.touched.float(), "stalled": layer.stalled.float(), "peak_n": env._step_peak,
           "knee_head_n": hd.knee_door_n, "foot_err": (foot - env._goal_world()).norm(dim=-1), "plane_err": plane,
           "engage_miss": env._engage_miss,
           "tangent_n": (applied * layer.tan_dir).sum(-1),
           "tangent_cmd": layer.normal_level * layer._segment(layer.tan_ratio),
           "slide_s": layer.slide_s, "foot_along": (foot * layer.slide_dir).sum(-1),
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
    for k, j in enumerate(leg_eval.JOINTS):
        fr = leg_cfg.FR_JOINTS[k]
        for name, value in (("tau", tau), ("dem", dem), ("qd", qd)):
            out[f"{name}_fr_{j}"] = value[:, fr]
            out[f"{name}_sup_{j}"] = value[:, [s + k for s in support]].abs().max(dim=1).values
    clipped = dem.abs() - tau.abs() > leg_eval.SATURATED_NM
    out["sat_fr"] = clipped[:, list(leg_cfg.FR_JOINTS)].any(dim=1).float()
    out["sat_legs"] = clipped.any(dim=1).float()
    blank = (done | ended).cpu().numpy()
    result = {k: np.where(blank, np.nan, v.detach().float().cpu().numpy()) for k, v in out.items()}
    ended_np = ended.cpu().numpy()
    result["fallen"] = np.where(ended_np, env._term_fallen.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    result["damaged"] = np.where(ended_np, env._term_damage.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    return result


_r, _first, _longest_run, _median, _quantile, _reasons = (leg_eval._r, leg_eval._first, leg_eval._longest_run,
                                                           leg_eval._median, leg_eval._quantile, leg_eval._reasons)


def score_episode(trace: dict, env: int, row: dict, dt: float = interface.POLICY_DT) -> dict:
    col = {k: trace[k][:, env] for k in FIELDS}
    alive = np.isfinite(col["stage"])
    stage = np.where(alive, col["stage"], -1)
    phase = np.where(alive, col["phase"], -1)
    engaged = stage == 4
    pressing = engaged & (phase == handle.HOLDING)
    on_plate = np.nan_to_num(col["on_plate"]) > 0.5
    touched = np.nan_to_num(col["touched"]) > 0.5
    fell = bool(np.nansum(col["fallen"]) > 0)
    damaged = bool(np.nansum(col["damaged"]) > 0)
    released = bool(((phase == handle.RELEASING) | (phase == handle.DONE)).any() & engaged.any())
    slipped = _longest_run(pressing & touched & ~on_plate) * dt >= leg_cfg.SLIP_S
    normal = np.nan_to_num(col["normal_n"])
    level = row["level_n"]
    steps = np.flatnonzero(pressing)
    window = steps[-round(pc.EVAL_SCORE_S / dt):] if released else np.array([], dtype=int)
    tol = max(pc.EVAL_TOL_N, pc.EVAL_TOL_SHARE * level)
    if len(window):
        mean = float(normal[window].mean())
        mae = float(np.abs(normal[window] - level).mean())
        std = float(normal[window].std())
    else:
        mean = mae = std = None
    within = mae is not None and mae <= tol
    success = released and within and not slipped and not fell and not damaged
    if success:
        reason = "ok"
    elif fell:
        reason = "fell"
    elif damaged:
        reason = "damaged"
    elif _first(engaged) is None:
        reason = "missed"
    elif not released:
        reason = "not finished"
    elif slipped:
        reason = "slipped"
    elif not touched.any():
        reason = "no contact"
    else:
        reason = "off force"
    first_press = _first(pressing)
    down = stage == 7
    # The slide set's measures: the tangential force against its command over the window; how far the foot moved along
    # the slide while it was commanded (the foot's own displacement, which a sticking foot does not make).
    tangential = {}
    if row.get("mode") == "tangent" and len(window):
        t_cmd = np.nan_to_num(col["tangent_cmd"])[window]
        t_act = np.nan_to_num(col["tangent_n"])[window]
        tangential = {"tangent_cmd_n": _r(float(t_cmd.mean()), 2), "tangent_mean_n": _r(float(t_act.mean()), 2),
                      "tangent_mae_n": _r(float(np.abs(t_act - t_cmd).mean()), 2)}
    if row.get("mode") == "slide" and first_press is not None:
        along = col["foot_along"]
        idx = np.flatnonzero(pressing)
        tangential = {"slide_cmd_m": _r(float(np.nanmax(col["slide_s"][idx])), 4),
                      "slide_foot_m": _r(float(along[idx[-1]] - along[idx[0]]), 4)}
    stance = col["stance_max_cm"]
    roll, pitch = np.nan_to_num(col["roll"]), np.nan_to_num(col["pitch"])
    extra = {}
    if all(k in trace for k in TORQUE_FIELDS):
        extra.update(leg_eval._torque_extras({k: trace[k][:, env] for k in TORQUE_FIELDS}, alive, pressing, dt))
    return extra | tangential | {
        "success": bool(success), "reason": reason, "fell": fell, "damaged": damaged,
        "engaged": bool(engaged.any()), "pressed": first_press is not None, "released": released,
        "stalled": bool(np.nansum(col["stalled"]) > 0), "touched": bool(touched.any()), "slipped": bool(slipped),
        "within": bool(within), "tolerance_n": _r(tol, 2),
        "force_mean_n": _r(mean, 2), "force_bias_n": _r(None if mean is None else mean - level, 2),
        "force_rel_bias": _r(None if mean is None else (mean - level) / level, 3),
        "force_mae_n": _r(mae, 2), "force_std_n": _r(std, 2),
        "press_peak_n": _r(float(normal[pressing].max()) if pressing.any() else None, 2),
        "peak_n": _r(float(np.nanmax(col["peak_n"])) if np.isfinite(col["peak_n"]).any() else 0.0, 2),
        "knee_head_peak_n": _r(float(np.nanmax(col["knee_head_n"])) if np.isfinite(col["knee_head_n"]).any() else 0.0, 2),
        "plane_err_mm": _r(1000.0 * float(col["plane_err"][first_press]) if first_press is not None else None, 1),
        # How far the foot was from the pre-contact point when the layer started (UniFP's goal reaching, held).
        "engage_miss_mm": _r(1000.0 * float(np.nanmax(col["engage_miss"])) if np.isfinite(col["engage_miss"]).any()
                             else None, 1),
        "foot_down": bool(down.any()),
        "stance_max_cm": _r(float(np.nanmax(stance)), 2) if np.isfinite(stance).any() else None,
        "max_tilt_deg": _r(math.degrees(float(np.max(np.abs(np.concatenate((roll, pitch)))))), 2),
    }


def score(trace: dict, rows: list[dict]) -> dict:
    episodes = [{**row, **score_episode(trace, i, row)} for i, row in enumerate(rows)]

    def group(key):
        out = []
        for value in sorted({e[key] for e in episodes}, key=lambda v: (str(type(v)), v)):
            sub = [e for e in episodes if e[key] == value]
            out.append({key: value, "success": sum(e["success"] for e in sub), "of": len(sub),
                        "force_bias_n_median": _median([e["force_bias_n"] for e in sub]),
                        "force_mae_n_median": _median([e["force_mae_n"] for e in sub]),
                        "force_rel_bias_median": _median([e["force_rel_bias"] for e in sub]),
                        "press_peak_n_median": _median([e["press_peak_n"] for e in sub]),
                        "peak_n_max": _r(max((e["peak_n"] for e in sub), default=0.0), 2),
                        "reasons": _reasons(sub)})
        return out

    summary = {
        "episodes": len(episodes), "success": sum(e["success"] for e in episodes),
        "within": sum(e["within"] for e in episodes), "engaged": sum(e["engaged"] for e in episodes),
        "released": sum(e["released"] for e in episodes), "stalled": sum(e["stalled"] for e in episodes),
        "touched": sum(e["touched"] for e in episodes), "slipped": sum(e["slipped"] for e in episodes),
        "fell": sum(e["fell"] for e in episodes), "damaged": sum(e["damaged"] for e in episodes),
        "foot_down": sum(e["foot_down"] for e in episodes), "reasons": _reasons(episodes),
        "force_bias_n_median": _median([e["force_bias_n"] for e in episodes]),
        "force_mae_n_median": _median([e["force_mae_n"] for e in episodes]),
        "force_mae_n_p90": _quantile([e["force_mae_n"] for e in episodes], 0.9),
        "force_rel_bias_median": _median([e["force_rel_bias"] for e in episodes]),
        "peak_n_median": _median([e["peak_n"] for e in episodes]),
        "peak_n_p95": _quantile([e["peak_n"] for e in episodes], 0.95),
        "peak_n_max": _r(max((e["peak_n"] for e in episodes), default=0.0), 2),
        "over_60n": sum((e["peak_n"] or 0.0) > 60.0 for e in episodes),
        "plane_err_mm_median": _median([e["plane_err_mm"] for e in episodes]),
        "engage_miss_mm_median": _median([e["engage_miss_mm"] for e in episodes]),
        "engage_miss_mm_p90": _quantile([e["engage_miss_mm"] for e in episodes], 0.9),
        "stance_max_cm_median": _median([e["stance_max_cm"] for e in episodes]),
        "max_tilt_deg_median": _median([e["max_tilt_deg"] for e in episodes]),
        "by_kind": group("kind"), "by_stiffness": group("stiffness"), "by_level_n": group("level_n"),
        "by_mode": group("mode"),
    }
    if any(e["mode"] == "tangent" for e in episodes):
        t = [e for e in episodes if e["mode"] == "tangent"]
        s = [e for e in episodes if e["mode"] == "slide"]
        summary["tangent"] = {"cmd_n_median": _median([e.get("tangent_cmd_n") for e in t]),
                              "mean_n_median": _median([e.get("tangent_mean_n") for e in t]),
                              "mae_n_median": _median([e.get("tangent_mae_n") for e in t])}
        summary["slide"] = {"cmd_m_median": _median([e.get("slide_cmd_m") for e in s]),
                            "foot_m_median": _median([e.get("slide_foot_m") for e in s]),
                            "foot_m_p10": _quantile([e.get("slide_foot_m") for e in s], 0.1)}
    if episodes and "fr_calf_peak_nm" in episodes[0]:
        summary["torques"] = leg_eval._torque_summary(episodes)
    return {"summary": summary, "episodes": episodes}

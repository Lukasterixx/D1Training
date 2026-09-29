"""Scoring a policy on the pressing task: which buttons it holds at full travel for a second, and why the rest failed.

Every episode is one fixed experiment (`press_cfg.EVAL_*`, frozen 2026-09-29 before any policy was scored): a button
straight ahead at one of three heights, pressed with the bare jaws or the probe, the estimate wrong by a set amount
across the axis, along it, in tilt and in the datasheet's force. Nothing is drawn, so two policies meet identical
buttons. Three sets (`build_plan`): `panel` (precision), `force` (the calibration's force levels), `recessed` (a small
recessed cap only the probe can reach: the bare jaws are expected to fail it, and that is an access limitation).

An episode **succeeded** when the cap was held at its full travel for `press_cfg.SUCCESS_HOLD_S` without a break,
with the robot upright and nothing damaged -- the plan's "full button travel and engaged for 1 s". Also per episode,
as the plan asks: the force error (the force applied along the axis against the task layer's command, RMS while it
pushes), the peak force at the physics rate, the time from the layer's start to full travel, and the failure reason:

    fell > damaged > not started (the hand never reached its pre-press pose) > missed (never touched the cap) >
    slid off (was on the cap, then left it while still pushing, pressed or not) > short (stayed on the cap, never at
    full travel) > not held (stayed on the cap and reached full travel, never for the full second)

Recording and scoring are separate so the scoring can be tested without a simulator.
"""
from __future__ import annotations

import itertools
import math

import numpy as np
import torch

from unifp_isaaclab import interface

from . import press_cfg, pressing

FIELDS = ("engaged", "pressed", "on_cap", "over_cap", "touched_cap", "travel_fraction", "peak_n", "applied_n",
          "push_n", "lateral_n", "phase", "approach_err", "engage_tip_miss", "engage_elevation", "fallen", "damaged")
REASONS = ("success", "fell", "damaged", "not started", "missed", "slid off", "short", "not held")


def _button(set_name: str, force_n: float | None) -> dict:
    if set_name == "recessed":
        return dict(press_cfg.EVAL_RECESSED_BUTTON)
    button = dict(press_cfg.EVAL_BUTTON)
    if force_n is not None:
        # The same cap and travel at another full-travel force: 30% of it preload, the rest the spring, a click a fifth.
        button["preload_n"] = 0.3 * force_n
        button["stiffness_n_per_m"] = 0.7 * force_n / button["travel_m"]
        button["snap_n"] = 0.2 * force_n
    return button


def build_plan(kind: str, device) -> tuple[dict, list[dict]]:
    """(the plan `Go2D1PressEnv.set_evaluation` takes, one descriptive row per episode)."""
    tools = pressing.TOOL_NAMES
    if kind == "panel":
        grid = itertools.product(press_cfg.EVAL_HEIGHTS_M, press_cfg.EVAL_LATERAL_M, press_cfg.EVAL_DIRECTIONS_RAD,
                                 tools, (None,))
    elif kind == "force":
        grid = itertools.product(press_cfg.EVAL_HEIGHTS_M, (press_cfg.EVAL_FORCE_LATERAL_M,), (0.0, math.pi), tools,
                                 press_cfg.EVAL_FORCE_LEVELS_N)
    elif kind == "recessed":
        grid = itertools.product(press_cfg.EVAL_HEIGHTS_M, press_cfg.EVAL_RECESSED_LATERAL_M,
                                 press_cfg.EVAL_DIRECTIONS_RAD, tools, (None,))
    else:
        raise ValueError(f"unknown pressing evaluation {kind!r}")
    rows, buttons = [], []
    for height, lateral, direction, tool, force in grid:
        button = _button(kind, force)
        full = button["preload_n"] + button["stiffness_n_per_m"] * button["travel_m"]
        rows.append({"set": kind, "height_m": height, "lateral_m": lateral, "direction_rad": direction, "tool": tool,
                     "force_n": round(full, 3), "cap_radius_m": button["cap_radius_m"]})
        buttons.append(button)
    n = len(rows)
    t = lambda values: torch.tensor(values, dtype=torch.float32, device=device)
    sign = [1.0 if i % 2 == 0 else -1.0 for i in range(n)]
    plan = {
        "face": t([[press_cfg.EVAL_AHEAD_M, 0.0, r["height_m"]] for r in rows]),
        "elevation": torch.zeros(n, device=device),
        "tool": torch.tensor([pressing.TOOL_NAMES.index(r["tool"]) for r in rows], dtype=torch.long, device=device),
        "lateral": t([[r["lateral_m"] * math.cos(r["direction_rad"]), r["lateral_m"] * math.sin(r["direction_rad"])]
                      for r in rows]),
        # The tilt turns with the lateral error's direction, so the directions are distinct cases; the depth and
        # rating errors alternate in sign.
        "tilt": t([[press_cfg.EVAL_TILT_RAD * math.cos(r["direction_rad"] + math.pi / 4),
                    press_cfg.EVAL_TILT_RAD * math.sin(r["direction_rad"] + math.pi / 4)] for r in rows]),
        "depth_error": t([press_cfg.EVAL_DEPTH_ERROR_M * s for s in sign]),
        "rating": t([1.0 + press_cfg.EVAL_RATING_ERROR * s for s in sign]),
        "travel": t([b["travel_m"] for b in buttons]), "cap_radius": t([b["cap_radius_m"] for b in buttons]),
        "proud": t([b["proud_m"] for b in buttons]), "preload": t([b["preload_n"] for b in buttons]),
        "stiffness": t([b["stiffness_n_per_m"] for b in buttons]), "damping": t([b["damping"] for b in buttons]),
        "snap_n": t([b["snap_n"] for b in buttons]), "snap_at": t([b["snap_at"] for b in buttons]),
        "mu": t([b["mu"] for b in buttons]),
        "conditions": dict(press_cfg.EVAL_CONDITIONS),
    }
    return plan, rows


def record_step(env, done: torch.Tensor, ended: torch.Tensor) -> dict:
    """One step's measurements, numpy over environments; an environment that ended earlier is NaN."""
    b, layer = env._button, env._layer
    state = env._hand_frame_terms()
    cos = (state["ee_approach_w"] * env._approach_des_w).sum(-1).clamp(-1.0, 1.0)
    miss = env._engage_miss
    out = {"engaged": env._engaged.float(), "pressed": (b.pressed() & env._engaged).float(),
           "on_cap": b.on_cap.float(), "over_cap": (b.active & (b.off_axis_m < b.cap_radius)).float(),
           "touched_cap": b.touched_cap.float(), "travel_fraction": b.s / b.travel.clamp(min=1e-6),
           "peak_n": env._step_peak, "applied_n": b.axial_n, "push_n": layer.push, "lateral_n": b.lateral_n,
           "phase": layer.phase.float(), "approach_err": torch.acos(cos), "engage_tip_miss": miss[:, 0],
           "engage_elevation": miss[:, 1]}
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


def score_episode(trace: dict, env: int, dt: float = interface.POLICY_DT) -> dict:
    col = lambda key: trace[key][:, env]
    flag = lambda key: np.nan_to_num(col(key)) > 0.5
    engaged = flag("engaged")
    start = _first(engaged)
    fell = _first(flag("fallen")) is not None
    damaged = _first(flag("damaged")) is not None
    out = {"engage_step": start, "fell": fell, "damaged": damaged, "success": False, "time_to_press_s": None,
           "longest_pressed_s": 0.0}
    if start is not None:
        after = np.arange(start, len(engaged))
        pressed = flag("pressed")[after]
        on_cap, over_cap = flag("on_cap")[after], flag("over_cap")[after]
        pushing = np.nan_to_num(col("phase")[after], nan=9.0) <= pressing.HOLDING
        first_pressed = _first(pressed)
        out["longest_pressed_s"] = round(_longest_run(pressed) * dt, 3)
        out["touched_cap"] = bool(on_cap.any())
        if first_pressed is not None:
            out["time_to_press_s"] = round(first_pressed * dt, 3)
        first_on = _first(on_cap)
        out["slid_off"] = bool(first_on is not None and (~over_cap[first_on:] & pushing[first_on:]).any())
        out["peak_contact_n"] = round(float(np.nanmax(col("peak_n")[after])), 2)
        out["peak_lateral_n"] = round(float(np.nanmax(col("lateral_n")[after])), 2)
        applied, push = col("applied_n")[after], col("push_n")[after]
        live = pushing & ~np.isnan(applied)
        # The plan's force error: what went along the axis against what the task layer commanded, while it pushed.
        out["force_rmse_n"] = round(float(np.sqrt(np.mean((applied[live] - push[live]) ** 2))), 3) if live.any() else None
        holding = (np.nan_to_num(col("phase")[after], nan=9.0) == pressing.HOLDING) & ~np.isnan(applied)
        out["hold_force_error_n"] = (round(float(np.mean(applied[holding] - push[holding])), 3)
                                     if holding.any() else None)
        first = np.arange(start, min(start + 10, len(engaged)))
        out["approach_err_start_deg"] = round(math.degrees(float(np.nanmean(col("approach_err")[first]))), 2)
        out["engage_tip_miss_mm"] = round(1000 * float(col("engage_tip_miss")[start]), 2)
        out["engage_elevation_deg"] = round(math.degrees(float(col("engage_elevation")[start])), 2)
        last = np.flatnonzero(~np.isnan(col("travel_fraction")))
        out["released"] = bool(len(last) and col("travel_fraction")[last[-1]] < 0.1
                               and col("phase")[last[-1]] >= pressing.RELEASING)
    out["success"] = (out["longest_pressed_s"] >= press_cfg.SUCCESS_HOLD_S - 1e-9) and not fell and not damaged
    if out["success"]:
        reason = "success"
    elif fell:
        reason = "fell"
    elif damaged:
        reason = "damaged"
    elif start is None:
        reason = "not started"
    elif not out["touched_cap"]:
        reason = "missed"
    elif out["slid_off"]:
        reason = "slid off"
    elif out["time_to_press_s"] is None:
        reason = "short"
    else:
        reason = "not held"
    out["reason"] = reason
    return out


def score(trace: dict, rows: list[dict]) -> dict:
    episodes = []
    for index, row in enumerate(rows):
        result = score_episode(trace, index)
        result.update({"env": index, **row})
        episodes.append(result)
    median = lambda values: round(float(np.median(values)), 3) if values else None

    def group(key):
        out = []
        for value in sorted({e[key] for e in episodes}, key=str):
            cell = [e for e in episodes if e[key] == value]
            won = [e for e in cell if e["success"]]
            out.append({key: value, "success": len(won), "of": len(cell),
                        "reasons": {r: sum(e["reason"] == r for e in cell) for r in REASONS if any(e["reason"] == r for e in cell)},
                        "time_to_press_s": median([e["time_to_press_s"] for e in won]),
                        "peak_contact_n": median([e["peak_contact_n"] for e in cell if "peak_contact_n" in e]),
                        "force_rmse_n": median([e["force_rmse_n"] for e in cell if e.get("force_rmse_n") is not None]),
                        "approach_err_start_deg": median([e["approach_err_start_deg"] for e in cell
                                                          if "approach_err_start_deg" in e])})
        return out

    started = [e for e in episodes if e["engage_step"] is not None]
    summary = {"episodes": len(episodes), "success": sum(e["success"] for e in episodes), "started": len(started),
               "reasons": {r: sum(e["reason"] == r for e in episodes) for r in REASONS},
               "fell": sum(e["fell"] for e in episodes), "damaged": sum(e["damaged"] for e in episodes),
               "released": sum(e.get("released", False) for e in episodes),
               "time_to_press_s": median([e["time_to_press_s"] for e in episodes if e["success"]]),
               "peak_contact_n": median([e["peak_contact_n"] for e in started if "peak_contact_n" in e]),
               "force_rmse_n": median([e["force_rmse_n"] for e in started if e.get("force_rmse_n") is not None]),
               "engage_tip_miss_mm": median([abs(e["engage_tip_miss_mm"]) for e in started
                                             if not math.isnan(e.get("engage_tip_miss_mm", float("nan")))]),
               "by_tool": group("tool"), "by_lateral": group("lateral_m"), "by_height": group("height_m"),
               "by_force": group("force_n")}
    return {"summary": summary, "episodes": episodes}

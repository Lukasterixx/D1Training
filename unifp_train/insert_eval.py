"""Scoring a policy on the insertion task: which keys it seats, how fast, how hard it pushes, whether it searched.

Every episode is one fixed experiment (`insert_cfg.EVAL_*`, frozen 2026-09-26): a slot straight ahead at one of three
heights, the estimate wrong across the axis by one of `EVAL_LATERAL_M` in one of four directions, a small tilt and
roll error, a detent; a level blade and an upright one. Nothing is drawn, so two policies meet identical slots.

An episode **seated** when the key reached its seat with the robot upright and nothing damaged. Also per episode:
time from the task layer's start to seated, whether it searched (the spiral ran), the largest contact and lateral
force, the hand's approach error at the start, and the key's final depth.

Recording and scoring are separate so the scoring can be tested without a simulator.
"""
from __future__ import annotations

import itertools
import math

import numpy as np
import torch

from unifp_isaaclab import interface

from . import insert_cfg

FIELDS = ("engaged", "seated", "entered", "searching", "depth", "contact_n", "lateral_n", "approach_err",
          "engage_tip_miss", "engage_elevation", "engage_azimuth", "engage_roll", "fallen", "damaged")


def build_plan(kind: str, device) -> tuple[dict, list[dict]]:
    """(the plan `Go2D1InsertEnv.set_evaluation` takes, one descriptive row per episode).

    `kind` "lateral": every height x lateral error x direction x roll at `EVAL_DETENT_N`. "detent": every height x
    direction x roll at 1.5 mm off and each of `EVAL_DETENT_LEVELS_N`."""
    rows = []
    if kind == "lateral":
        grid = itertools.product(insert_cfg.EVAL_HEIGHTS_M, insert_cfg.EVAL_LATERAL_M, insert_cfg.EVAL_DIRECTIONS_RAD,
                                 insert_cfg.EVAL_ROLLS_RAD, (insert_cfg.EVAL_DETENT_N,))
    elif kind == "detent":
        grid = itertools.product(insert_cfg.EVAL_HEIGHTS_M, (0.0015,), insert_cfg.EVAL_DIRECTIONS_RAD,
                                 insert_cfg.EVAL_ROLLS_RAD, insert_cfg.EVAL_DETENT_LEVELS_N)
    else:
        raise ValueError(f"unknown insertion evaluation {kind!r}")
    for height, lateral, direction, roll, detent in grid:
        rows.append({"height_m": height, "lateral_m": lateral, "direction_rad": direction, "roll_rad": roll,
                     "detent_n": detent})
    n = len(rows)
    t = lambda values: torch.tensor(values, dtype=torch.float32, device=device)
    mouth = t([[insert_cfg.EVAL_AHEAD_M, 0.0, r["height_m"]] for r in rows])
    lateral = t([[r["lateral_m"] * math.cos(r["direction_rad"]), r["lateral_m"] * math.sin(r["direction_rad"])]
                 for r in rows])
    # The tilt and roll errors turn with the lateral error's direction, so the four directions are four real cases.
    tilt = t([[insert_cfg.EVAL_TILT_RAD * math.cos(r["direction_rad"] + math.pi / 4),
               insert_cfg.EVAL_TILT_RAD * math.sin(r["direction_rad"] + math.pi / 4)] for r in rows])
    roll_error = t([insert_cfg.EVAL_ROLL_ERROR_RAD * (1.0 if i % 2 == 0 else -1.0) for i in range(n)])
    plan = {"mouth": mouth, "elevation": torch.zeros(n, device=device), "roll": t([r["roll_rad"] for r in rows]),
            "lateral": lateral, "tilt": tilt, "roll_error": roll_error, "detent_n": t([r["detent_n"] for r in rows]),
            "conditions": dict(insert_cfg.EVAL_CONDITIONS)}
    return plan, rows


def record_step(env, done: torch.Tensor, ended: torch.Tensor) -> dict:
    """One step's measurements, numpy over environments; an environment that ended earlier is NaN."""
    slot, layer = env._slot, env._layer
    force = slot.force_on_tip
    lateral = force - (force * layer.n).sum(-1, keepdim=True) * layer.n
    state = env._hand_frame_terms()
    cos = (state["ee_approach_w"] * env._approach_des_w).sum(-1).clamp(-1.0, 1.0)
    miss = env._engage_miss   # at the layer's start: tip (m), elevation, azimuth, roll (rad); NaN before
    out = {"engaged": env._engaged.float(), "seated": slot.seated().float(), "entered": slot.entered.float(),
           "searching": (layer.stall > layer.stall_s).float(), "depth": slot.tip_depth,
           "contact_n": slot.normal_sum_n, "lateral_n": lateral.norm(dim=-1), "approach_err": torch.acos(cos),
           "engage_tip_miss": miss[:, 0], "engage_elevation": miss[:, 1], "engage_azimuth": miss[:, 2],
           "engage_roll": miss[:, 3]}
    blank = (done | ended).cpu().numpy()
    result = {k: np.where(blank, np.nan, v.detach().float().cpu().numpy()) for k, v in out.items()}
    ended_np = ended.cpu().numpy()
    result["fallen"] = np.where(ended_np, env._term_fallen.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    result["damaged"] = np.where(ended_np, env._term_damage.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    return result


def _first(mask: np.ndarray) -> int | None:
    hits = np.flatnonzero(mask)
    return int(hits[0]) if len(hits) else None


def score_episode(trace: dict, env: int, dt: float = interface.POLICY_DT) -> dict:
    engaged = np.nan_to_num(trace["engaged"][:, env]) > 0.5
    start = _first(engaged)
    fell = _first(np.nan_to_num(trace["fallen"][:, env]) > 0.5)
    damaged = _first(np.nan_to_num(trace["damaged"][:, env]) > 0.5)
    out = {"engage_step": start, "fell": fell is not None, "damaged": damaged is not None, "seated": False,
           "time_to_seat_s": None}
    if start is None:
        return out
    seated_at = _first(np.nan_to_num(trace["seated"][:, env]) > 0.5)
    out["seated"] = seated_at is not None and fell is None and damaged is None
    #: Reached the seat at some point, whatever came after -- not the frozen metric, which also asks for no damage.
    out["ever_seated"] = seated_at is not None
    if seated_at is not None:
        out["time_to_seat_s"] = round((seated_at - start) * dt, 3)
    after = np.arange(start, len(trace["engaged"]))
    out["entered"] = bool(np.nanmax(np.nan_to_num(trace["entered"][after, env])) > 0.5)
    out["searched"] = bool(np.nanmax(np.nan_to_num(trace["searching"][after, env])) > 0.5)
    out["peak_contact_n"] = round(float(np.nanmax(trace["contact_n"][after, env])), 2)
    out["peak_lateral_n"] = round(float(np.nanmax(trace["lateral_n"][after, env])), 2)
    first = np.arange(start, min(start + 10, len(trace["engaged"])))
    out["approach_err_start_deg"] = round(math.degrees(float(np.nanmean(trace["approach_err"][first, env]))), 2)
    if "engage_tip_miss" in trace:
        out["engage_tip_miss_mm"] = round(1000 * float(trace["engage_tip_miss"][start, env]), 2)
        for name in ("elevation", "azimuth", "roll"):
            out[f"engage_{name}_deg"] = round(math.degrees(float(trace[f"engage_{name}"][start, env])), 2)
    out["final_depth_mm"] = round(1000 * float(trace["depth"][np.flatnonzero(~np.isnan(trace["depth"][:, env]))[-1], env]), 2)
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
        for value in sorted({e[key] for e in episodes}):
            cell = [e for e in episodes if e[key] == value]
            seated = [e for e in cell if e["seated"]]
            out.append({key: value, "seated": len(seated), "of": len(cell),
                        "searched": sum(e.get("searched", False) for e in cell),
                        "fell": sum(e["fell"] for e in cell), "damaged": sum(e["damaged"] for e in cell),
                        "time_to_seat_s": median([e["time_to_seat_s"] for e in seated]),
                        "peak_contact_n": median([e["peak_contact_n"] for e in cell if "peak_contact_n" in e]),
                        "peak_lateral_n": median([e["peak_lateral_n"] for e in cell if "peak_lateral_n" in e]),
                        "approach_err_start_deg": median([e["approach_err_start_deg"] for e in cell
                                                          if "approach_err_start_deg" in e])})
        return out

    started = [e for e in episodes if e["engage_step"] is not None]
    at_start = {name: median([abs(e[f"engage_{name}"]) for e in started if e.get(f"engage_{name}") is not None])
                for name in ("tip_miss_mm", "elevation_deg", "azimuth_deg", "roll_deg")}
    summary = {"episodes": len(episodes), "seated": sum(e["seated"] for e in episodes),
               "started": len(started), "entered": sum(e.get("entered", False) for e in started),
               "ever_seated": sum(e.get("ever_seated", False) for e in episodes),
               "at_start_median_abs": at_start,
               "fell": sum(e["fell"] for e in episodes), "damaged": sum(e["damaged"] for e in episodes),
               "by_lateral": group("lateral_m"), "by_detent": group("detent_n"), "by_height": group("height_m"),
               "by_roll": group("roll_rad")}
    return {"summary": summary, "episodes": episodes}

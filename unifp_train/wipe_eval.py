"""Scoring a policy on the wiping task: how much of the path it wiped, how much of the motion it kept contact, at what
force, and why the rest failed.

Every episode is one fixed experiment (`wipe_cfg.EVAL_*`, frozen 2026-09-29 before any policy was scored): a panel
straight ahead, the path centred at one of three heights, a straight stroke or a zigzag, the surface nearer, where or
further than estimated and tilted from it, one of two wiping forces. Nothing is drawn, so two policies meet identical
surfaces.

An episode **succeeded** when at least `wipe_cfg.SUCCESS_COMPLETION` of the path was wiped and the pad kept contact
for at least `wipe_cfg.SUCCESS_CONTACT` of the motion, with the robot upright and nothing damaged -- the plan's
"at least 90% path completion and at least 80% contact during motion". Also per episode, as the plan asks for contact
tasks: the normal force's RMS error against the task layer's command and its physics-rate peak, the friction carried,
contact-loss events (a loss shorter than `wipe_cfg.LOSS_DEBOUNCE_S` is not one), the time from the layer's start to the
path's end, and the failure reason:

    fell > damaged > not started (the hand never reached its pre-contact pose) > no contact (never pressed the
    surface) > incomplete (less than 90% wiped) > contact lost (less than 80% of the motion in contact)

"The motion" is from the moment the layer found the surface to the path's end (or the episode's). Recording and scoring
are separate so the scoring can be tested without a simulator.
"""
from __future__ import annotations

import itertools
import math

import numpy as np
import torch

from unifp_isaaclab import interface

from . import wipe_cfg, wiping

FIELDS = ("engaged", "phase", "in_contact", "normal_n", "push_n", "peak_n", "tangential_n", "coverage", "progress",
          "path_error", "pad_tilt", "approach_err", "engage_tip_miss", "engage_elevation", "fallen", "damaged")
REASONS = ("success", "fell", "damaged", "not started", "no contact", "incomplete", "contact lost")


def build_plan(device) -> tuple[dict, list[dict]]:
    """(the plan `Go2D1WipeEnv.set_evaluation` takes, one descriptive row per episode)."""
    rows = []
    for height, kind, offset, force in itertools.product(wipe_cfg.EVAL_HEIGHTS_M, wipe_cfg.KINDS,
                                                         wipe_cfg.EVAL_OFFSETS_M, wipe_cfg.EVAL_FORCES_N):
        rows.append({"height_m": height, "kind": kind, "offset_m": offset, "force_n": force})
    n = len(rows)
    t = lambda values: torch.tensor(values, dtype=torch.float32, device=device)
    line = wiping.resample(wiping.line_path(torch.full((n,), wipe_cfg.EVAL_LINE_LENGTH_M, device=device),
                                            torch.zeros(n, device=device)))
    zigzag = wiping.resample(wiping.zigzag_path(torch.full((n,), wipe_cfg.EVAL_ZIGZAG_LENGTH_M, device=device),
                                                torch.full((n,), wipe_cfg.EVAL_ZIGZAG_SPACING_M, device=device)))
    is_line = torch.tensor([r["kind"] == "line" for r in rows], device=device)
    # The tilt alternates between the surface's horizontal and vertical axis, and its sign with the next pair.
    tilt = [[wipe_cfg.EVAL_TILT_RAD * (1.0 if (i // 2) % 2 == 0 else -1.0), 0.0] if i % 2 == 0 else
            [0.0, wipe_cfg.EVAL_TILT_RAD * (1.0 if (i // 2) % 2 == 0 else -1.0)] for i in range(n)]
    plan = {"centre": t([[wipe_cfg.EVAL_AHEAD_M, 0.0, r["height_m"]] for r in rows]),
            "elevation": torch.zeros(n, device=device),
            "points": torch.where(is_line[:, None, None], line[0], zigzag[0]),
            "arc": torch.where(is_line[:, None], line[1], zigzag[1]),
            "kind": (~is_line).long(),
            "offset": t([r["offset_m"] for r in rows]), "tilt": t(tilt), "force": t([r["force_n"] for r in rows]),
            "conditions": dict(wipe_cfg.EVAL_CONDITIONS)}
    for row, tilt_row in zip(rows, tilt):
        row["tilt_deg"] = [round(math.degrees(v), 2) for v in tilt_row]
    return plan, rows


def record_step(env, done: torch.Tensor, ended: torch.Tensor) -> dict:
    """One step's measurements, numpy over environments; an environment that ended earlier is NaN."""
    s, layer = env._surface, env._layer
    state = env._hand_frame_terms()
    cos = (state["ee_approach_w"] * env._approach_des_w).sum(-1).clamp(-1.0, 1.0)
    miss = env._engage_miss
    # Diagnostics of coverage: how far the contact is from the path (in the estimated plane), and how far the pad's
    # face is tilted from the true surface (a tilted rigid pad presses with its rim, off its centre).
    contact_ab = layer.in_plane(s.pressure_centre)
    path_error = (layer.points - contact_ab.unsqueeze(1)).norm(dim=-1).min(dim=-1).values
    tilt = torch.acos((state["ee_approach_w"] * -s.normal).sum(-1).clamp(-1.0, 1.0))
    out = {"engaged": env._engaged.float(), "phase": layer.phase.float(), "in_contact": s.in_contact.float(),
           "normal_n": s.normal_n, "push_n": layer.push, "peak_n": env._step_peak, "tangential_n": s.tangential_n,
           "coverage": env.coverage(), "progress": layer.progress / layer.total.clamp(min=1e-6),
           "path_error": torch.where(s.in_contact, path_error, torch.full_like(path_error, float("nan"))),
           "pad_tilt": torch.where(s.active, tilt, torch.full_like(tilt, float("nan"))),
           "approach_err": torch.acos(cos), "engage_tip_miss": miss[:, 0], "engage_elevation": miss[:, 1]}
    blank = (done | ended).cpu().numpy()
    result = {k: np.where(blank, np.nan, v.detach().float().cpu().numpy()) for k, v in out.items()}
    ended_np = ended.cpu().numpy()
    result["fallen"] = np.where(ended_np, env._term_fallen.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    result["damaged"] = np.where(ended_np, env._term_damage.float().cpu().numpy(), np.where(blank, np.nan, 0.0))
    return result


def _first(mask: np.ndarray) -> int | None:
    hits = np.flatnonzero(mask)
    return int(hits[0]) if len(hits) else None


def loss_events(contact: np.ndarray, debounce_steps: int) -> int:
    """Losses of contact lasting at least `debounce_steps`, after the first contact."""
    events, run, seen = 0, 0, False
    for value in contact:
        if value:
            if seen and run >= debounce_steps:
                events += 1
            seen, run = True, 0
        elif seen:
            run += 1
    if seen and run >= debounce_steps:
        events += 1
    return events


def score_episode(trace: dict, env: int, dt: float = interface.POLICY_DT) -> dict:
    col = lambda key: trace[key][:, env]
    flag = lambda key: np.nan_to_num(col(key)) > 0.5
    start = _first(flag("engaged"))
    fell = _first(flag("fallen")) is not None
    damaged = _first(flag("damaged")) is not None
    out = {"engage_step": start, "fell": fell, "damaged": damaged, "success": False, "completion": 0.0,
           "contact_fraction": 0.0, "finished": False, "time_to_finish_s": None}
    if start is not None:
        phase = np.nan_to_num(col("phase"), nan=-1.0)
        motion = phase == wiping.WIPING
        contact = flag("in_contact")
        valid = ~np.isnan(col("coverage"))
        out["touched"] = bool(motion.any())
        out["completion"] = round(float(np.nanmax(col("coverage"))), 3) if valid.any() else 0.0
        if motion.any():
            out["contact_fraction"] = round(float(contact[motion].mean()), 3)
            normal, push = col("normal_n")[motion], col("push_n")[motion]
            out["force_rmse_n"] = round(float(np.sqrt(np.mean((normal - push) ** 2))), 3)
            out["force_mean_n"] = round(float(np.mean(normal[contact[motion]])), 3) if contact[motion].any() else 0.0
            out["tangential_mean_n"] = round(float(np.mean(col("tangential_n")[motion])), 3)
            out["loss_events"] = loss_events(contact[motion], max(1, round(wipe_cfg.LOSS_DEBOUNCE_S / dt)))
            if "path_error" in trace:
                err = col("path_error")[motion]
                out["path_error_p90_mm"] = (round(1000 * float(np.nanpercentile(err, 90)), 2)
                                            if np.isfinite(err).any() else None)
                out["pad_tilt_median_deg"] = round(math.degrees(float(np.nanmedian(col("pad_tilt")[motion]))), 2)
        finished = _first(phase >= wiping.LIFTING)
        out["finished"] = finished is not None
        if finished is not None:
            out["time_to_finish_s"] = round((finished - start) * dt, 3)
        after = np.arange(start, len(phase))
        out["peak_contact_n"] = round(float(np.nanmax(col("peak_n")[after])), 2)
        first = np.arange(start, min(start + 10, len(phase)))
        out["approach_err_start_deg"] = round(math.degrees(float(np.nanmean(col("approach_err")[first]))), 2)
        out["engage_tip_miss_mm"] = round(1000 * float(col("engage_tip_miss")[start]), 2)
        out["engage_elevation_deg"] = round(math.degrees(float(col("engage_elevation")[start])), 2)
    out["success"] = (out["completion"] >= wipe_cfg.SUCCESS_COMPLETION
                      and out["contact_fraction"] >= wipe_cfg.SUCCESS_CONTACT and not fell and not damaged)
    if out["success"]:
        reason = "success"
    elif fell:
        reason = "fell"
    elif damaged:
        reason = "damaged"
    elif start is None:
        reason = "not started"
    elif not out.get("touched", False):
        reason = "no contact"
    elif out["completion"] < wipe_cfg.SUCCESS_COMPLETION:
        reason = "incomplete"
    else:
        reason = "contact lost"
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
            out.append({key: value, "success": sum(e["success"] for e in cell), "of": len(cell),
                        "reasons": {r: sum(e["reason"] == r for e in cell) for r in REASONS
                                    if any(e["reason"] == r for e in cell)},
                        "completion": median([e["completion"] for e in cell]),
                        "contact_fraction": median([e["contact_fraction"] for e in cell]),
                        "force_rmse_n": median([e["force_rmse_n"] for e in cell if "force_rmse_n" in e]),
                        "peak_contact_n": median([e["peak_contact_n"] for e in cell if "peak_contact_n" in e])})
        return out

    started = [e for e in episodes if e["engage_step"] is not None]
    summary = {"episodes": len(episodes), "success": sum(e["success"] for e in episodes), "started": len(started),
               "touched": sum(e.get("touched", False) for e in episodes),
               "finished": sum(e["finished"] for e in episodes),
               "reasons": {r: sum(e["reason"] == r for e in episodes) for r in REASONS},
               "fell": sum(e["fell"] for e in episodes), "damaged": sum(e["damaged"] for e in episodes),
               "completion": median([e["completion"] for e in episodes]),
               "contact_fraction": median([e["contact_fraction"] for e in started]),
               "force_rmse_n": median([e["force_rmse_n"] for e in started if "force_rmse_n" in e]),
               "peak_contact_n": median([e["peak_contact_n"] for e in started if "peak_contact_n" in e]),
               "loss_events": median([e["loss_events"] for e in started if "loss_events" in e]),
               "path_error_p90_mm": median([e["path_error_p90_mm"] for e in started
                                            if e.get("path_error_p90_mm") is not None]),
               "pad_tilt_median_deg": median([e["pad_tilt_median_deg"] for e in started if "pad_tilt_median_deg" in e]),
               "engage_tip_miss_mm": median([abs(e["engage_tip_miss_mm"]) for e in started
                                             if not math.isnan(e.get("engage_tip_miss_mm", float("nan")))]),
               "by_kind": group("kind"), "by_offset": group("offset_m"), "by_force": group("force_n"),
               "by_height": group("height_m")}
    return {"summary": summary, "episodes": episodes}

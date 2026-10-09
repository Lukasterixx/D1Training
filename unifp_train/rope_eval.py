"""Scoring a policy on the rope-pull task: how steady its pull is, how it makes it, and how it survives the rope going.

Every episode is the same experiment (`rope_cfg.eval_plan()`, frozen 2026-10-07): the goal to one of nine grip
placements, a 1 m rope tied there and pulled straight back at one of two stiffnesses, the command climbing 20, 35 and
50 N (each held 4 s), then the anchor letting go at the end of the 50 N hold in one of three ways -- snapping with the
task layer noticing 0.3 s later, snapping unnoticed, or slipping to a 20% drag, noticed. Nothing is drawn, so every
policy meets identical demands.

**Steadiness** is read off the tension at the physics rate (200 Hz) over the last 2 s of each hold:

  * mean and error against the level; *held* when the mean reaches 80% of it and the robot is up;
  * `std_n` and `p2p_n`, the spread; `hf_rms_n`, what is left after a 0.2 s running mean (shake and tugs, not drift);
  * `slack_fraction`, the share of samples with the rope slack;
  * `tugs_per_s`: surge-and-release cycles, peaks standing 25% of the command (at least 3 N) above the troughs
    either side -- a tug as a rope feels it -- counted also over each whole hold (`hold_tugs`), tie-on included;
  * `dominant_hz` and its amplitude, from the spectrum of the detrended tension.

**How the force is made**, over the same windows: base pitch and height, each arm joint's load against its limit,
the front feet's share of the vertical load, how far any foot slid, and the angle between the arm's line (shoulder to
tool) and the rope's -- 0 when the arm is drawn straight along the rope, in pure tension.

**Recovery**, over the `EVAL_RECOVERY_S` after the release: a strict fall (`rope_cfg.EVAL_FALL_*`), the base's largest
horizontal excursion and speed, the largest change of pitch, the tool's lunge (its largest move relative to the body),
the feet lifted, and the time to settle (the base still for 0.5 s).

The recording and the scoring are separate so the scoring is tested without a simulator (`tests/test_rope.py`).
"""
from __future__ import annotations

import math

import numpy as np
import torch

from unifp_isaaclab import interface

from . import observations, rope, rope_cfg

#: Per-step quantities recorded per environment: (steps, envs) unless noted.
FIELDS = ("engaged", "released", "detected", "stage", "level", "cmd", "tension_sub", "along", "pitch", "roll",
          "height", "base_xy", "base_speed", "base_rate", "tool_b", "tool_speed", "arm_loads", "feet_fz", "feet_xy",
          "align_deg", "elbow", "trunk_contact", "arm_contact", "fallen", "estimate_along")


def record_step(env, policy_estimate: torch.Tensor | None, done: torch.Tensor, ended: torch.Tensor) -> dict:
    """One step's measurements, as numpy arrays. Call after `env.step`.

    An environment that terminated on this step has already been reset, so its state is the next episode's: `ended`
    marks those, recorded only as having fallen or not (`env._term_fallen`), every other field NaN; `done` marks
    episodes that ended earlier, all NaN.
    """
    rp = env._rope
    data = env._robot.data
    origins = env.scene.env_origins
    yaw = env._base_yaw_quat()
    root = data.root_pos_w - origins
    tip = env._tip_pos()
    limits = torch.as_tensor(interface.TORQUE_LIMITS[12:interface.NUM_ACTIONS], device=env.device)
    arm_loads = data.applied_torque[:, env._order][:, 12:interface.NUM_ACTIONS].abs() / limits
    roll_pitch = interface.body_roll_pitch(data.root_quat_w)
    feet = env._contact_forces(env._feet_contacts)
    feet_xy = data.body_pos_w[:, env._feet_bodies, :2] - origins[:, None, :2]
    shoulder = data.body_pos_w[:, env._shoulder_body] - origins
    arm_line = tip - shoulder
    rope_line = -rp.direction
    cos = (arm_line * rope_line).sum(-1) / (arm_line.norm(dim=-1) * rope_line.norm(dim=-1)).clamp(min=1e-9)
    dof_pos, _ = env._dof()
    trunk = env._contact_forces(env._trunk_contacts).norm(dim=-1).max(dim=1).values
    out = {
        "engaged": rp.engaged.float(), "released": rp.released.float(), "detected": env._detected.float(),
        "stage": env._levels.stage.float(), "level": env._levels.level, "cmd": env._cmd_level,
        "along": (rp.applied_by_robot() * env._pull_dir).sum(-1),
        "pitch": roll_pitch[:, 1], "roll": roll_pitch[:, 0], "height": root[:, 2],
        "base_speed": data.root_lin_vel_w[:, :2].norm(dim=-1), "base_rate": data.root_ang_vel_b[:, :2].norm(dim=-1),
        "tool_speed": env._tool_vel_w().norm(dim=-1),
        "align_deg": torch.rad2deg(torch.acos(cos.clamp(-1.0, 1.0))) * rp.engaged,
        # Joint3, the D1's elbow, in UniFP's order (12 legs, then Joint1..6).
        "elbow": dof_pos[:, 14],
        "trunk_contact": trunk,
        "arm_contact": env._arm_contact.data.net_forces_w.norm(dim=-1).max(dim=1).values,
        "fallen": torch.zeros_like(trunk),
    }
    if policy_estimate is not None:
        # The decoder's force block is the reaction on the tool, x0.01, yaw frame; the robot applies its negative.
        start = observations.offsets()["ee_force"][0]
        on_tool_yaw = policy_estimate[:, start:start + 3] / observations.OBS_SCALE_EE_FORCE
        estimate = -interface.quat_apply(yaw, on_tool_yaw)
        out["estimate_along"] = (estimate * env._pull_dir).sum(-1)
    else:
        out["estimate_along"] = torch.full_like(trunk, float("nan"))
    blank = (done | ended).cpu().numpy()
    np_ = lambda t: t.detach().float().cpu().numpy()
    result = {k: np.where(blank, np.nan, np_(v)) for k, v in out.items()}
    for key, value in (("tension_sub", env._tension_sub), ("base_xy", root[:, :2]),
                       ("tool_b", interface.quat_rotate_inverse(yaw, tip - root)), ("arm_loads", arm_loads),
                       ("feet_fz", feet[:, :, 2])):
        result[key] = np.where(blank[:, None], np.nan, np_(value))
    result["feet_xy"] = np.where(blank[:, None, None], np.nan, np_(feet_xy))
    ended_np = ended.cpu().numpy()
    result["fallen"] = np.where(ended_np, np_(env._term_fallen.float()), np.where(blank, np.nan, 0.0))
    return result


# --- scoring --------------------------------------------------------------------------------------

def count_tugs(tension: np.ndarray, level) -> int:
    """Surge-and-release cycles: tension peaks standing at least `TUG_PROMINENCE_FRACTION` of the command (and at
    least `TUG_MIN_PROMINENCE_N`) above the troughs either side of them.

    `level` is a number, or an array as long as `tension` (the command as it ramps). A peak counts only where at least
    `rope_cfg.TUG_MIN_COMMAND_N` is commanded. A steady pull with a newton of ripple has none; a rope snatched taut,
    or let slack and snatched again, has one per snatch.
    """
    from scipy.signal import find_peaks

    if len(tension) < 3:
        return 0
    level = np.broadcast_to(np.asarray(level, dtype=float), tension.shape)
    peaks, props = find_peaks(tension, prominence=rope_cfg.TUG_MIN_PROMINENCE_N)
    if len(peaks) == 0:
        return 0
    target = level[peaks]
    keep = (target >= rope_cfg.TUG_MIN_COMMAND_N) & (
        props["prominences"] >= np.maximum(rope_cfg.TUG_PROMINENCE_FRACTION * target, rope_cfg.TUG_MIN_PROMINENCE_N))
    return int(keep.sum())


def running_mean(x: np.ndarray, width: int) -> np.ndarray:
    if width <= 1 or len(x) < width:
        return np.full_like(x, x.mean())
    kernel = np.ones(width) / width
    padded = np.pad(x, (width // 2, width - 1 - width // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def steadiness(tension: np.ndarray, level: float, rate_hz: float) -> dict:
    """Spread, shake, slack, tugs and the dominant oscillation of a tension trace sampled at `rate_hz`."""
    mean = float(tension.mean())
    detrended = tension - mean
    spectrum = np.abs(np.fft.rfft(detrended * np.hanning(len(detrended))))
    freqs = np.fft.rfftfreq(len(detrended), 1.0 / rate_hz)
    peak = 1 + int(np.argmax(spectrum[1:])) if len(spectrum) > 1 else 0
    # Amplitude of a sinusoid at the peak, corrected for the Hann window's coherent gain (0.5).
    amplitude = float(2.0 * spectrum[peak] / (0.5 * len(detrended))) if len(detrended) else 0.0
    window_s = len(tension) / rate_hz
    shake = tension - running_mean(tension, max(1, round(0.2 * rate_hz)))
    return {
        "mean_n": mean, "error_n": mean - level, "std_n": float(tension.std()),
        "p2p_n": float(tension.max() - tension.min()), "cv": float(tension.std() / max(mean, 1e-6)),
        "hf_rms_n": float(np.sqrt(np.mean(np.square(shake)))),
        "slack_fraction": float(np.mean(tension < rope.SLACK_N)),
        "tugs": count_tugs(tension, level), "tugs_per_s": count_tugs(tension, level) / window_s,
        "dominant_hz": float(freqs[peak]), "dominant_amp_n": amplitude,
    }


def settle_time(base_speed, base_rate, dt: float) -> float | None:
    """Seconds until the base is still -- horizontal speed and roll/pitch rate under their limits -- for
    `EVAL_SETTLE_HOLD_S`, or None if never within the trace.

    The base only: once the task layer notices a release the goal slides back and UniFP's free-space goals resume, so
    the tool is meant to move; its lunge is scored separately. (Until 2026-10-07 this also required the tool still, and
    so read "never settled" for nearly every noticed release.)
    """
    still = ((np.nan_to_num(base_speed, nan=9.0) < rope_cfg.EVAL_SETTLE_BASE_M_S)
             & (np.nan_to_num(base_rate, nan=9.0) < rope_cfg.EVAL_SETTLE_ANG_RAD_S))
    need = max(1, round(rope_cfg.EVAL_SETTLE_HOLD_S / dt))
    run = 0
    for i, ok in enumerate(still):
        run = run + 1 if ok else 0
        if run >= need:
            return (i + 1 - need) * dt
    return None


def _nanmean(values) -> float:
    """`np.nanmean` without its warning: NaN when nothing is finite (a zero-action run has no force estimate)."""
    values = np.asarray(values, dtype=float)
    return float(np.mean(values[np.isfinite(values)])) if np.isfinite(values).any() else float("nan")


def _round(value, places=3):
    return None if value is None or not np.isfinite(value) else round(float(value), places)


def score(trace: dict, rows: list[dict], levels=rope_cfg.EVAL_LEVELS_N, hold_steps: int | None = None,
          window_steps: int | None = None, recovery_steps: int | None = None) -> dict:
    """Per-episode and summary results from a recorded trace, `rows` being the evaluation plan."""
    dt = interface.POLICY_DT
    substeps = trace["tension_sub"].shape[2]
    rate = substeps / dt
    hold_steps = hold_steps or round(rope_cfg.EVAL_HOLD_S / dt)
    window_steps = window_steps or round(rope_cfg.EVAL_WINDOW_S / dt)
    recovery_steps = recovery_steps or round(rope_cfg.EVAL_RECOVERY_S / dt)
    num_envs = trace["stage"].shape[1]
    episodes = []
    for env in range(num_envs):
        row = rows[env]
        engaged = np.nan_to_num(trace["engaged"][:, env]) > 0.5
        released = np.nan_to_num(trace["released"][:, env]) > 0.5
        fell_at = np.flatnonzero(np.nan_to_num(trace["fallen"][:, env]) > 0.5)
        release_at = np.flatnonzero(released)
        r = int(release_at[0]) if len(release_at) else None
        per_level = []
        for k, level in enumerate(levels):
            steps = np.flatnonzero((trace["stage"][:, env] == k + 1) & engaged & ~released)
            complete = len(steps) >= hold_steps - 1
            window = steps[-window_steps:] if complete else steps[:0]
            entry = {"level_n": level, "complete": bool(complete)}
            if len(window):
                tension = trace["tension_sub"][window, env].reshape(-1)
                tension = tension[np.isfinite(tension)]
                stats = steadiness(tension, level, rate)
                entry.update({key: _round(value) for key, value in stats.items()})
                fz = trace["feet_fz"][window, env]
                total = np.nansum(fz, axis=1)
                feet = trace["feet_xy"][window, env]
                slide = np.nanmax(np.linalg.norm(feet - feet[:1], axis=-1)) if len(feet) else float("nan")
                entry.update({
                    "held": bool(stats["mean_n"] >= rope_cfg.EVAL_HELD_FRACTION * level
                                 and not (len(fell_at) and fell_at[0] <= window[-1])),
                    "pitch_deg": _round(math.degrees(np.nanmean(trace["pitch"][window, env])), 2),
                    "height_m": _round(np.nanmean(trace["height"][window, env]), 4),
                    "arm_loads": [_round(v) for v in np.nanmean(trace["arm_loads"][window, env], axis=0)],
                    "arm_load_max": _round(np.nanmax(trace["arm_loads"][window, env])),
                    "front_share": _round(np.nanmean((fz[:, 0] + fz[:, 1]) / np.where(total > 1.0, total, np.nan))),
                    "feet_slide_mm": _round(1000.0 * slide, 1),
                    "align_deg": _round(np.nanmean(trace["align_deg"][window, env]), 2),
                    "elbow_rad": _round(np.nanmean(trace["elbow"][window, env])),
                    "tool_speed_m_s": _round(np.nanmean(trace["tool_speed"][window, env])),
                    "estimate_n": _round(_nanmean(trace["estimate_along"][window, env]), 2),
                })
            else:
                entry["held"] = False
            # The whole hold, from the step the level was commanded -- the ramp, and for the first level the rope
            # being tied -- where the tail window above sees only the settled end. Added 2026-10-07 after the
            # baseline's trace showed 10-35 N swings in the first second of the first hold that the window misses;
            # the episodes are unchanged.
            if len(steps):
                tension = trace["tension_sub"][steps, env].reshape(-1)
                command = np.repeat(trace["cmd"][steps, env], substeps)
                finite = np.isfinite(tension) & np.isfinite(command)
                tension, command = tension[finite], command[finite]
                pulling = command >= rope_cfg.SLACK_COMMAND_N
                entry.update({
                    "hold_tugs": count_tugs(tension, command),
                    "hold_peak_n": _round(tension.max() if len(tension) else np.nan, 2),
                    "hold_overshoot_n": _round((tension - command).max() if len(tension) else np.nan, 2),
                    "hold_slack_fraction": _round(np.mean(tension[pulling] < rope.SLACK_N) if pulling.any() else np.nan),
                })
            per_level.append(entry)
        recovery = None
        if r is not None:
            span = np.arange(r, min(r + recovery_steps, trace["stage"].shape[0]))
            pitch, roll = trace["pitch"][span, env], trace["roll"][span, env]
            height, trunk = trace["height"][span, env], trace["trunk_contact"][span, env]
            fell_term = bool(len(fell_at) and r <= fell_at[0] <= span[-1])
            strict = bool(np.nanmax(np.abs(roll), initial=0.0) > rope_cfg.EVAL_FALL_TILT_RAD
                          or np.nanmax(np.abs(pitch), initial=0.0) > rope_cfg.EVAL_FALL_TILT_RAD
                          or np.nanmin(height, initial=9.0) < rope_cfg.EVAL_FALL_HEIGHT_M
                          or np.nanmax(trunk, initial=0.0) > rope_cfg.TRUNK_CONTACT_N)
            base = trace["base_xy"][span, env]
            tool = trace["tool_b"][span, env]
            fz = trace["feet_fz"][span, env]
            down = np.nan_to_num(fz, nan=0.0) > 5.0
            liftoffs = int(np.sum(down[:-1] & ~down[1:])) if len(down) > 1 else 0
            settled = settle_time(trace["base_speed"][span, env], trace["base_rate"][span, env], dt)
            after = span[(span >= r + 25) & (span < r + 50)]
            recovery = {
                "fell": fell_term or strict, "fell_terminated": fell_term,
                "pitch_change_max_deg": _round(math.degrees(np.nanmax(np.abs(pitch - pitch[0]))), 2),
                "roll_max_deg": _round(math.degrees(np.nanmax(np.abs(roll))), 2),
                "height_min_m": _round(np.nanmin(height), 4),
                "base_speed_max_m_s": _round(np.nanmax(trace["base_speed"][span, env])),
                "base_excursion_max_m": _round(np.nanmax(np.linalg.norm(base - base[0], axis=-1))),
                "base_excursion_end_m": _round(np.linalg.norm(base[-1] - base[0]) if np.isfinite(base[-1]).all() else np.nan),
                "tool_lunge_max_m": _round(np.nanmax(np.linalg.norm(tool - tool[0], axis=-1))),
                "tool_speed_max_m_s": _round(np.nanmax(trace["tool_speed"][span, env])),
                "foot_liftoffs": liftoffs,
                "settle_s": _round(settled, 2),
                "tension_max_after_n": _round(np.nanmax(trace["tension_sub"][span[1:], env]) if len(span) > 1 else np.nan, 2),
                "estimate_after_n": _round(_nanmean(trace["estimate_along"][after, env]) if len(after) else np.nan, 2),
                "tension_at_release_n": _round(np.nanmean(trace["tension_sub"][r - 1, env]) if r > 0 else np.nan, 2),
            }
        episodes.append({"env": env, "release": row["release"], "stiffness": row["stiffness"],
                         "grip_sphere": list(row["sphere"]),
                         "grip_height_m": round(rope_cfg.GOAL_CENTRE_HEIGHT_M + row["sphere"][0] * math.sin(row["sphere"][1]), 3),
                         "engaged": bool(engaged.any()),
                         "fell_before_release": bool(len(fell_at) and (r is None or fell_at[0] < r)),
                         "levels": per_level, "recovery": recovery})
    return {"summary": summarise(episodes, levels), "episodes": episodes}


def _stats(values) -> dict | None:
    values = [v for v in values if v is not None and np.isfinite(v)]
    if not values:
        return None
    return {"median": round(float(np.median(values)), 3), "p90": round(float(np.percentile(values, 90)), 3),
            "max": round(float(np.max(values)), 3)}


def summarise(episodes: list[dict], levels) -> dict:
    summary = {"episodes": len(episodes), "engaged": sum(e["engaged"] for e in episodes),
               "fell_before_release": sum(e["fell_before_release"] for e in episodes),
               "by_level": [], "by_level_stiffness": [], "recovery": []}
    keys = ("mean_n", "error_n", "std_n", "p2p_n", "cv", "hf_rms_n", "slack_fraction", "tugs_per_s", "dominant_hz",
            "dominant_amp_n", "pitch_deg", "height_m", "arm_load_max", "front_share", "feet_slide_mm", "align_deg",
            "elbow_rad", "tool_speed_m_s", "estimate_n", "hold_tugs", "hold_peak_n", "hold_overshoot_n",
            "hold_slack_fraction")
    for k, level in enumerate(levels):
        rows = [e["levels"][k] for e in episodes]
        entry = {"level_n": level, "held": sum(r["held"] for r in rows), "of": len(rows),
                 "with_tugs": sum(1 for r in rows if (r.get("tugs") or 0) > 0),
                 "with_hold_tugs": sum(1 for r in rows if (r.get("hold_tugs") or 0) > 0),
                 "hold_tugs_total": sum(r.get("hold_tugs") or 0 for r in rows)}
        for key in keys:
            entry[key] = _stats([r.get(key) for r in rows])
        loads = [r["arm_loads"] for r in rows if r.get("arm_loads")]
        if loads:
            entry["arm_loads_median"] = [round(float(np.median([l[j] for l in loads if l[j] is not None])), 3)
                                         for j in range(6)]
        summary["by_level"].append(entry)
        for stiffness in sorted({e["stiffness"] for e in episodes}):
            sub = [e["levels"][k] for e in episodes if e["stiffness"] == stiffness]
            summary["by_level_stiffness"].append({
                "level_n": level, "stiffness": stiffness, "held": sum(r["held"] for r in sub), "of": len(sub),
                "std_n": _stats([r.get("std_n") for r in sub]), "hf_rms_n": _stats([r.get("hf_rms_n") for r in sub]),
                "tugs_per_s": _stats([r.get("tugs_per_s") for r in sub]),
                "slack_fraction": _stats([r.get("slack_fraction") for r in sub])})
    for name in dict.fromkeys(e["release"] for e in episodes):
        sub = [e for e in episodes if e["release"] == name]
        rec = [e["recovery"] for e in sub if e["recovery"] is not None]
        entry = {"release": name, "episodes": len(sub), "released": len(rec), "fell": sum(r["fell"] for r in rec),
                 "fell_terminated": sum(r["fell_terminated"] for r in rec),
                 "never_settled": sum(1 for r in rec if r["settle_s"] is None)}
        for key in ("pitch_change_max_deg", "roll_max_deg", "height_min_m", "base_speed_max_m_s",
                    "base_excursion_max_m", "base_excursion_end_m", "tool_lunge_max_m", "tool_speed_max_m_s",
                    "foot_liftoffs", "settle_s", "tension_max_after_n", "estimate_after_n", "tension_at_release_n"):
            entry[key] = _stats([r.get(key) for r in rec])
        summary["recovery"].append(entry)
    return summary


def rescore(run_dir) -> dict:
    """Score a recorded `rope_eval` run again with the scoring as it is now, from its saved trace and `run.json`."""
    import json
    from pathlib import Path

    run_dir = Path(run_dir)
    report = json.loads((run_dir / "run.json").read_text())
    arguments = report["arguments"]
    rows = rope_cfg.test_plan() if arguments.get("rope_set") == "test" else rope_cfg.eval_plan()
    if arguments.get("rope_stiffness"):
        rows = [dict(r, stiffness=k) for k in arguments["rope_stiffness"]
                for r in rows if r["stiffness"] == rope_cfg.EVAL_STIFFNESS[0]]
    rows = rows * int(arguments.get("repeats") or 1)
    trace = dict(np.load(run_dir / "rope_trace.npz"))
    levels = tuple(arguments.get("eval_levels")
                   or (rope_cfg.TEST_LEVELS_N if arguments.get("rope_set") == "test" else rope_cfg.EVAL_LEVELS_N))
    result = score(trace, rows, levels=levels)
    (run_dir / "rope_eval.json").write_text(json.dumps(result["summary"], indent=2))
    (run_dir / "rope_eval_episodes.json").write_text(json.dumps({"episodes": result["episodes"]}, indent=2))
    report["summary"] = result["summary"]
    (run_dir / "run.json").write_text(json.dumps(report, indent=2))
    return result["summary"]


if __name__ == "__main__":
    import sys

    for path in sys.argv[1:]:
        summary = rescore(path)
        print(path)
        for row in summary["by_level"]:
            med = lambda key: (row.get(key) or {}).get("median")
            print(f"  {row['level_n']:4.0f} N: held {row['held']}/{row['of']}  tail tugs in {row['with_tugs']}  "
                  f"hold tugs in {row['with_hold_tugs']} ({row['hold_tugs_total']} total)  hold peak {med('hold_peak_n')}  "
                  f"overshoot {med('hold_overshoot_n')}  hold slack {med('hold_slack_fraction')}")

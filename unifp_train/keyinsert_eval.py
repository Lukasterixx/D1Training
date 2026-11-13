"""The key insertion's frozen evaluation (`keyinsert_env.Go2D1KeyInsertEnv`, `run_unifp_train.py leg_eval --key_insert`).

96 locks: the estimate off the true keyway across the axis by 0.5, 1.5, 2.5 and 3.5 mm in four directions, along it by
-6, 0 and +6 mm (negative: the face nearer than estimated), the keyway rolled -3 or +3 degrees; the box placed as the key
hold's frozen set places it (cycled over its 108). The lock's own draws (play, edge breaks, friction, the grip's slip)
are the training ranges' middles, the force-aware press `keylock.EVAL_PRESS_N`.

**Success** is the key truly seated -- its tip within 0.6 mm of the keyway's floor, its axis within the play of the
keyway's -- when the layer has finished its hold, with nothing damaged (`keylock.DAMAGE_N` at the physics rate), the key
not slid out of the jaws and the robot standing. The layer's own belief is scored apart: `believed` (the layer finished)
and `false_seat` (it finished with the key not in).
"""
from __future__ import annotations

import math

import numpy as np
import torch

from unifp_isaaclab import interface

from . import keylock, leg_cfg, leg_eval

LATERAL_MM = (0.5, 1.5, 2.5, 3.5)
DIRECTIONS_DEG = (45.0, 135.0, 225.0, 315.0)
DEPTH_MM = (-6.0, 0.0, 6.0)
ROLL_DEG = (-3.0, 3.0)
SEED = 20261113
#: The dog runner's tilt stop, deg.
TILT_STOP_DEG = 15.0
FIELDS = ("alive", "ended", "phase", "ref", "loop", "fk_true_depth", "rep_err_deg", "ik_fail", "tip_depth", "tip_lateral", "seated", "inside", "force", "normal", "peak", "slip",
          "lost", "damaged", "roll_cmd", "press_cmd", "meas_depth", "touch", "base_z", "rp", "hold_err")


#: What `--insert_eval_set name=value` may replace in every frozen lock, and the unit its value is given in.
OVERRIDES = {"mu": 1.0, "play_mm": 1e-3, "lock_chamfer_mm": 1e-3, "key_chamfer_mm": 1e-3, "grip_slip_n": 1.0,
             "press_n": 1.0, "lateral_scale": 1.0, "depth_scale": 1.0, "roll_scale": 1.0}
_PLAN_KEY = {"mu": "mu", "play_mm": "play", "lock_chamfer_mm": "lock_chamfer", "key_chamfer_mm": "key_chamfer",
             "grip_slip_n": "grip_slip", "press_n": "press"}


def build_plan(device, overrides: dict | None = None) -> tuple[dict, list[dict]]:
    """The frozen set; `overrides` (`OVERRIDES`' names) replace a column for every lock, or scale the estimate's errors."""
    overrides = overrides or {}
    for name in overrides:
        if name not in OVERRIDES:
            raise ValueError(f"--insert_eval_set: {name!r} is not one of {sorted(OVERRIDES)}")
    key_plan, key_rows = leg_eval.build_key_plan("cpu")
    rows = []
    for lat in LATERAL_MM:
        for direction in DIRECTIONS_DEG:
            for depth in DEPTH_MM:
                for roll in ROLL_DEG:
                    rows.append({"lateral_mm": lat, "direction_deg": direction, "depth_mm": depth, "roll_deg": roll})
    n = len(rows)
    pick = torch.arange(n) % len(key_rows)
    handles = {k: v[pick] for k, v in key_plan["handles"].items()}
    for i, row in enumerate(rows):
        kr = key_rows[int(pick[i])]
        row.update({"door_ahead": kr["door_ahead"], "axis_side": kr["axis_side"], "height": kr["height"],
                    "door_yaw": kr["door_yaw"], "key_row": int(pick[i])})
    col = lambda key: torch.tensor([r[key] for r in rows], dtype=torch.float32)
    mid = lambda r: torch.full((n,), 0.5 * (r[0] + r[1]))
    g = torch.Generator().manual_seed(SEED)
    insert = {"lat_r": col("lateral_mm") / 1000.0, "lat_dir": torch.deg2rad(col("direction_deg")),
              "depth": col("depth_mm") / 1000.0, "roll": torch.deg2rad(col("roll_deg")),
              "tilt_u": (2 * torch.rand(n, generator=g) - 1) * math.radians(0.5),
              "tilt_t": (2 * torch.rand(n, generator=g) - 1) * math.radians(0.5),
              "play": mid(keylock.PLAY_M), "lock_chamfer": mid(keylock.LOCK_CHAMFER_M),
              "key_chamfer": mid(keylock.KEY_CHAMFER_M), "mu": mid(keylock.MU), "grip_slip": mid(keylock.GRIP_SLIP_N),
              "start_off": torch.full((n,), 0.0175), "press": torch.full((n,), keylock.EVAL_PRESS_N)}
    for name, value in overrides.items():
        if name in _PLAN_KEY:
            insert[_PLAN_KEY[name]] = torch.full((n,), float(value) * OVERRIDES[name])
        else:
            column = {"lateral_scale": "lat_r", "depth_scale": "depth", "roll_scale": "roll"}[name]
            insert[column] = insert[column] * float(value)
            unit = {"lateral_scale": "lateral_mm", "depth_scale": "depth_mm", "roll_scale": "roll_deg"}[name]
            for row in rows:
                row[unit] = round(row[unit] * float(value), 4)
    for row in rows:
        row["overrides"] = dict(overrides)
    return {"handles": {k: v.to(device) for k, v in handles.items()},
            "insert": {k: v.to(device) for k, v in insert.items()}}, rows


def record_step(env, done: torch.Tensor, ended: torch.Tensor) -> dict:
    lk, ly = env._lock, env._layer_ins
    alive = (~done).float()
    rp = interface.body_roll_pitch(env._robot.data.root_quat_w)
    out = {"alive": alive, "ended": ended.float(), "phase": ly.phase.float(), "ref": ly.ref_depth,
           "loop": env._loop.norm(dim=-1), "ik_fail": env._ik_fail / env._ik_calls.clamp(min=1.0), "fk_true_depth": env._fk_depth(true_joints=True),
           "rep_err_deg": torch.rad2deg((env._arm_link.reported - env._dof()[0][:, 12:18]).abs().amax(-1))
           if env._arm_link is not None else torch.zeros(env.num_envs, device=env.device), "tip_depth": lk.tip_depth,
           "tip_lateral": lk.tip_lateral.norm(dim=-1), "seated": lk.seated.float(), "inside": lk.tip_inside.float(),
           "force": lk.force_w.norm(dim=-1), "normal": lk.normal_n, "peak": lk.peak_n, "slip": lk.slip_m,
           "lost": lk.lost.float(), "damaged": env._damaged.float(), "roll_cmd": ly.roll, "press_cmd": ly.force_cmd,
           "meas_depth": env._meas_depth, "touch": ly.touch_depth,
           "base_z": env._robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2], "rp": rp.abs().amax(dim=-1),
           "hold_err": env._hold_err}
    return {k: v.detach().float().cpu().numpy() for k, v in out.items()}


def score_episode(col: dict, row: dict, dt: float = interface.POLICY_DT) -> dict:
    alive = col["alive"] > 0.5
    steps = int(alive.sum())
    phase = col["phase"][:steps]
    # The layer believed it seated the key: it reached its hold (DONE alone is also the reset's and the wait's).
    believed = bool((phase == keylock.HOLD).any())
    end = steps - 1
    # Judged where the layer finishes its hold (the dog's next step is the turn), else at the episode's end.
    hold_steps = np.nonzero(phase == keylock.HOLD)[0]
    judge = int(hold_steps[-1]) if len(hold_steps) else end
    seated_end = bool(col["seated"][judge] > 0.5)
    seated_ever = bool((col["seated"][:steps] > 0.5).any())
    first_seat = np.nonzero(col["seated"][:steps] > 0.5)[0]
    touched = np.nonzero((phase >= keylock.SEARCH) & (phase <= keylock.HOLD))[0]
    fell = bool(col["ended"][end] > 0.5) and float(col["rp"][end]) > 0.8
    damaged = bool(col["damaged"][:steps].max() > 0.5)
    lost = bool(col["lost"][:steps].max() > 0.5)
    pressing = (phase >= keylock.SEARCH) & (phase <= keylock.SEAT)
    out = {**row,
           "success": bool(believed and seated_end and not damaged and not lost and not fell),
           "believed": believed, "seated_end": seated_end, "seated_ever": seated_ever,
           "false_seat": bool(believed and not seated_end),
           "touched": bool(len(touched)), "damaged": damaged, "lost": lost, "fell": fell,
           "seat_s": round(float(first_seat[0]) * dt, 2) if len(first_seat) else None,
           "touch_s": round(float(touched[0]) * dt, 2) if len(touched) else None,
           "peak_n": round(float(col["peak"][:steps].max()), 2),
           "press_n_median": round(float(np.median(col["normal"][:steps][pressing])), 2) if pressing.any() else None,
           "over_20n": bool(col["force"][:steps].max() > keylock.PRESS_LIMIT_N),
           "slip_mm": round(1000.0 * float(col["slip"][:steps].max()), 2),
           "max_tilt_deg": round(math.degrees(float(col["rp"][:steps].max())), 1),
           "end_depth_mm": round(1000.0 * float(col["tip_depth"][judge]), 2),
           "end_lateral_mm": round(1000.0 * float(col["tip_lateral"][judge]), 2),
           "seated_at_20s": bool(col["seated"][end] > 0.5),
           "ik_fail_fraction": round(float(col["ik_fail"][end]), 3), "steps": steps}
    if len(touched):
        # Where the layer felt the face, against where the true face was along the estimated axis.
        out["touch_err_mm"] = round(1000.0 * (float(col["touch"][touched[0]]) - row["depth_mm"] / 1000.0), 2)
    reason = ("success" if out["success"] else "fell" if fell else "damaged" if damaged else "lost" if lost
              else "false_seat" if out["false_seat"] else "seated_no_finish" if seated_end
              else "searching" if len(touched) else "no_touch")
    out["reason"] = reason
    return out


def score(trace: dict, rows: list[dict]) -> dict:
    episodes = [score_episode({k: v[:, i] for k, v in trace.items()}, rows[i]) for i in range(len(rows))]
    n = len(episodes)
    count = lambda key: int(sum(1 for e in episodes if e[key]))
    vals = lambda key: [e[key] for e in episodes if e.get(key) is not None]
    med = lambda key: round(float(np.median(vals(key))), 2) if vals(key) else None
    q = lambda key, p: round(float(np.quantile(vals(key), p)), 2) if vals(key) else None
    reasons = {}
    for e in episodes:
        reasons[e["reason"]] = reasons.get(e["reason"], 0) + 1

    def group(key):
        out = []
        for value in sorted({e[key] for e in episodes}):
            sub = [e for e in episodes if e[key] == value]
            out.append({key: value, "success": sum(e["success"] for e in sub), "of": len(sub),
                        "peak_n_median": round(float(np.median([e["peak_n"] for e in sub])), 1),
                        "seat_s_median": (round(float(np.median([e["seat_s"] for e in sub if e["seat_s"] is not None])), 2)
                                          if any(e["seat_s"] is not None for e in sub) else None)})
        return out

    # Inside the dog runner's tilt stop (15 deg, `leg_eval.STOP_FIELDS`'s; a whole-body run past it is stopped).
    within = sum(1 for e in episodes if e["success"] and e["max_tilt_deg"] <= TILT_STOP_DEG)
    summary = {"episodes": n, "success": count("success"), "success_tilt15": within, "believed": count("believed"),
               "seated_end": count("seated_end"), "seated_ever": count("seated_ever"), "false_seat": count("false_seat"),
               "touched": count("touched"), "damaged": count("damaged"), "lost": count("lost"), "fell": count("fell"),
               "over_20n": count("over_20n"), "peak_n_median": med("peak_n"), "peak_n_p90": q("peak_n", 0.9),
               "peak_n_max": round(max(e["peak_n"] for e in episodes), 2), "press_n_median": med("press_n_median"),
               "seat_s_median": med("seat_s"), "seat_s_p90": q("seat_s", 0.9), "touch_err_mm_median": med("touch_err_mm"),
               "max_tilt_deg_median": med("max_tilt_deg"), "ik_fail_fraction_median": med("ik_fail_fraction"),
               "reasons": reasons,
               "by_lateral_mm": group("lateral_mm"), "by_depth_mm": group("depth_mm"), "by_roll_deg": group("roll_deg")}
    return {"summary": summary, "episodes": episodes}


def evaluate(args, env, obs, report, run_dir, rows, write_json) -> int:
    from . import eval as evaluation

    run_dir.mkdir(parents=True, exist_ok=True)
    policy = None
    if not args.zero_actions:
        policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = str(args.checkpoint)
        report["checkpoint_source"] = policy.source
        report["checkpoint_iteration"] = policy.iteration
    controller = "zero actions" if policy is None else str(args.checkpoint).split("/")[-1]
    report["controller"] = controller
    report["insert_rows"] = rows
    done = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    trace = {k: [] for k in FIELDS}
    with torch.inference_mode():
        for _ in range(int(env.max_episode_length) - 1):
            actions = (torch.zeros(env.num_envs, int(env.cfg.action_space), device=env.device)
                       if policy is None else policy.act(obs["policy"]))
            obs, _, terminated, truncated, _ = env.step(actions)
            ended = (terminated | truncated) & ~done
            # Recorded before the reset's values replace the ended episodes' (the env resets inside step): the lock's
            # buffers are the step's as computed, the reset touches only the layer's phase (DONE) and the counters.
            step = record_step(env, done, ended)
            for k in FIELDS:
                trace[k].append(step[k])
            done |= ended
            if bool(done.all()):
                break
    trace = {k: np.stack(v) for k, v in trace.items()}
    np.savez_compressed(run_dir / "insert_trace.npz", **trace)
    result = score(trace, rows)
    write_json(run_dir / "insert_eval.json", result["summary"])
    write_json(run_dir / "insert_eval_episodes.json", {"episodes": result["episodes"]})
    report["summary"] = result["summary"]
    report["status"] = "evaluated"
    s = result["summary"]
    print(f"\n[insert_eval {env.cfg.leg_variant}] {controller}  success {s['success']}/{s['episodes']} "
          f"(within the 15 deg tilt stop {s['success_tilt15']})  "
          f"believed {s['believed']}  seated {s['seated_end']} (ever {s['seated_ever']})  false seat {s['false_seat']}  "
          f"touched {s['touched']}  damaged {s['damaged']}  lost {s['lost']}  fell {s['fell']}  past 20 N {s['over_20n']}\n"
          f"  peak median {s['peak_n_median']} N (p90 {s['peak_n_p90']}, max {s['peak_n_max']}), press median "
          f"{s['press_n_median']} N, seated at {s['seat_s_median']} s (p90 {s['seat_s_p90']}), touch error "
          f"{s['touch_err_mm_median']} mm, tilt {s['max_tilt_deg_median']} deg  {s['reasons']}", flush=True)
    for key in ("by_lateral_mm", "by_depth_mm", "by_roll_deg"):
        print("  " + "  ".join(f"{key[3:]} {list(r.values())[0]}: {r['success']}/{r['of']} (peak {r['peak_n_median']})"
                               for r in s[key]), flush=True)
    return 0


if __name__ == "__main__":
    # Rescore a recorded evaluation: python -m unifp_train.keyinsert_eval <run_dir>
    import json
    import sys
    from pathlib import Path

    run = Path(sys.argv[1])
    trace = dict(np.load(run / "insert_trace.npz"))
    args = json.loads((run / "run.json").read_text()).get("arguments", {}) if (run / "run.json").exists() else {}
    sets = dict(item.split("=", 1) for item in (args.get("insert_eval_set") or []))
    result = score(trace, build_plan("cpu", sets)[1])
    (run / "insert_eval.json").write_text(json.dumps(result["summary"], indent=2))
    (run / "insert_eval_episodes.json").write_text(json.dumps({"episodes": result["episodes"]}, indent=2))
    s = result["summary"]
    print({k: s[k] for k in ("episodes", "success", "success_tilt15", "believed", "seated_end", "seated_ever", "false_seat", "touched",
                             "damaged", "lost", "fell", "over_20n", "peak_n_median", "peak_n_p90", "press_n_median",
                             "seat_s_median", "touch_err_mm_median", "reasons")})
    for key in ("by_lateral_mm", "by_depth_mm", "by_roll_deg"):
        print("  " + "  ".join(f"{list(r.values())[0]}: {r['success']}/{r['of']}" for r in s[key]))

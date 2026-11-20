#!/usr/bin/env python3
"""Item 3 (2026-11-19): the box sequence's handover, from a learned insertion to the learned key hold and turn.

    python demos/combiner_key/insert_handover.py logs/unifp_train/<insertion run> --out <dir>

From an insertion frozen evaluation run with `--insert_trace_body` (its `insert_trace.npz` and episodes), for each episode
the layer seated inside the dog runner's tilt stop: the robot where the insertion left it, at the layer's last HOLD
step -- the legs (UniFP's order), the base's height, roll and pitch as a spawn state -- and that episode's box re-expressed
from the base's pose then (the base moved 18 mm forward and 43 mm sideways at the median during the insertion, F-296),
so a key-hold episode starting at the origin meets the box where the robot really is against it. Its friction, detent and
the estimate's errors are the episode's key row's (`leg_eval.build_key_plan` with `--key_hold`'s overrides, as the
insertion used; for a tool-set run, which has none, row lock mod the plan's, as the frozen 96 take them). Writes `<out>/handover_rows.json` (`--key_eval_rows`) and `<out>/handover_spawn.json`
(`--spawn_states ... --eval_spawn_states --spawn_states_ordered`), the rows in the spawn states' order.

What it does not carry over: the joints' velocities (the robot is near still in HOLD), the arm (the hold re-solves it from
the base's pose with the key in the lock, as every hold episode starts), and the key's give in the jaws.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

HOLD = 3                    # keylock.HOLD
TILT_STOP_DEG = 15.0


def rpy(quat_wxyz) -> tuple[float, float, float]:
    w, x, y, z = (float(v) for v in quat_wxyz)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def door_point(ahead: float, side: float, height: float, yaw: float) -> np.ndarray:
    """Where the key's axis meets the door, as `leg_env._place_keys` places it (`insert_gate.lock_estimate`)."""
    along = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    return np.array([ahead, 0.0, height]) + (side / along[1]) * along


def box_from(base_xy, base_yaw: float, ahead: float, side: float, height: float, door_yaw: float) -> dict:
    """The box's (door_ahead, axis_side, height, door_yaw) for a robot at the origin facing +x, from one standing at
    `base_xy` heading `base_yaw` in the box's frame."""
    p = door_point(ahead, side, height, door_yaw)
    c, s = math.cos(-base_yaw), math.sin(-base_yaw)
    rel = p[:2] - np.asarray(base_xy, dtype=float)
    x, y = c * rel[0] - s * rel[1], s * rel[0] + c * rel[1]
    psi = door_yaw - base_yaw
    return {"door_ahead": x + y * math.tan(psi), "axis_side": y, "height": height, "door_yaw": psi}


def key_rows() -> list[dict]:
    """The frozen key plan's rows as `--key_hold` sets them (`run_unifp_train.py`'s overrides of `leg_cfg`)."""
    from unifp_train import leg_cfg as c
    from unifp_train import leg_eval

    c.KEY_EVAL_DOOR_AHEAD_M, c.KEY_EVAL_AXIS_SIDE_M = c.KEYHOLD_EVAL_DOOR_AHEAD_M, c.KEYHOLD_EVAL_AXIS_SIDE_M
    c.KEY_EVAL_FRICTION_NM, c.KEY_EVAL_DETENT_NM = c.KEYHOLD_EVAL_FRICTION_NM, c.KEYHOLD_EVAL_DETENT_NM
    c.KEY_EVAL_AXIS_HEIGHT_M, c.KEY_EVAL_DOOR_YAW_RAD = c.KEYHOLD_EVAL_AXIS_HEIGHT_M, c.KEYHOLD_EVAL_DOOR_YAW_RAD
    return leg_eval.build_key_plan("cpu")[1]


def handover(run: Path) -> tuple[list[dict], list[dict], dict]:
    t = dict(np.load(run / "insert_trace.npz"))
    if "base_quat" not in t:
        raise SystemExit(f"{run}: no body fields (an insertion run with --insert_trace_body)")
    episodes = json.loads((run / "insert_eval_episodes.json").read_text())["episodes"]
    keys = key_rows()
    rows, spawn, moved = [], [], []
    for i, e in enumerate(episodes):
        if not (e["success"] and e["max_tilt_deg"] <= TILT_STOP_DEG):
            continue
        hold = np.nonzero(t["phase"][:, i] == HOLD)[0]
        k = int(hold[-1])
        roll, pitch, yaw = rpy(t["base_quat"][k, i])
        bx, by = float(t["base_x"][k, i]), float(t["base_y"][k, i])
        box = box_from((bx, by), yaw, e["door_ahead"], e["axis_side"], e["height"], e["door_yaw"])
        # The frozen 96 carry their key row (`keyinsert_eval.build_plan`: lock i takes row i mod the plan's). A tool-set run
        # (`keytools.build_plan`, the chain's sets) gives every lock the plan's first row, its foot never turning: its
        # handovers take row lock mod the plan's, as the frozen 96 do.
        row_i = int(e["key_row"]) if "key_row" in e else int(e["lock"]) % len(keys)
        kr = keys[row_i]
        rows.append({**{key: round(float(v), 6) for key, v in box.items()},
                     **{key: kr[key] for key in ("friction_nm", "detent_nm", "err_side", "err_up", "err_depth", "err_rating")},
                     "insert_episode": i, "key_row": row_i,
                     "insert_box": {key: e[key] for key in ("door_ahead", "axis_side", "height", "door_yaw")}})
        spawn.append({"legs": [round(float(v), 5) for v in t["leg_q"][k, i]], "base_z": round(float(t["base_z"][k, i]), 5),
                      "roll": round(roll, 6), "pitch": round(pitch, 6),
                      "source": f"{run.name} episode {i}, the layer's last HOLD step ({k})"})
        moved.append((bx, by, math.degrees(yaw)))
    m = np.array(moved) if moved else np.zeros((0, 3))
    info = {"run": run.name, "episodes": len(episodes), "handed_over": len(rows),
            "base_at_handover_mm_deg_median": [round(float(v), 2) for v in (np.median(m, 0) * [1000, 1000, 1])] if len(m) else None}
    return rows, spawn, info


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    rows, spawn, info = handover(Path(args.run))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "handover_rows.json").write_text(json.dumps({"source": info, "rows": rows}, indent=1) + "\n")
    (out / "handover_spawn.json").write_text(json.dumps(spawn, indent=1) + "\n")
    print(json.dumps(info))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

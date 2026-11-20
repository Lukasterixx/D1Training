#!/usr/bin/env python3
"""Check the dog's numpy insertion runner (`insert_policy.py`) against a recorded Isaac rollout (on the PC).

    # the rollout (`run_unifp_train.py leg_eval --key_insert ... --insert_shadow_dump N`):
    python demos/combiner_key/robot/check_insert.py --policy checkpoints/insert_force_ki6_8200.npz \
        --sim <run dir>/insert_shadow_dump.npz [--out report.json]

For each recorded episode, from its reset: the runner is reset with the episode's estimated lock, believed key, start
depth and press and the base's pose, and solves the arm's start pose itself; then each step it is given only the raw
state the environment had (the joints in Isaac's order, mapped to Unitree's by name; the IMU; the base's pose in the env
frame; the arm as the D1's link reported it) and runs the layer, the outer loop, the IK and the frame on its own state.
The numpy network acts on the runner's own history. Its targets are computed from the action the checkpoint took in the
rollout (as `check_shadow.py --sim` does), so the replay stays on the recorded trajectory. Every quantity is compared
with what the environment computed: the largest absolute difference over every alive step of every episode.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import insert_policy, leg_policy  # noqa: E402
from demos.combiner_key.robot.check_shadow import block_errors  # noqa: E402

#: The layer's buffers compared each step (the dump's `layer_<name>`).
LAYER_COMPARED = ("ref_depth", "ref_lateral", "force_cmd", "roll", "touch_depth", "phase_for", "lag_for", "held_for",
                  "still_for", "still_any", "jiggle_for", "spiral_s", "entered_by", "seat_end", "face_depth",
                  "settle_for")
LAYER_EXACT = ("phase", "locate_stage", "refused", "touched")
#: Buffers the batched torch layer updates inside `if any environment is searching / seating:` blocks, unmasked, so one
#: environment's are zeroed while another searches or seats (`insert_policy`'s docstring): compared in the phases that
#: read them; outside those, reported apart as `<name>_outside_read_phases`.
READ_IN = {"held_for": (insert_policy.APPROACH, insert_policy.SEARCH), "still_for": (insert_policy.SEAT,),
           "still_any": (insert_policy.SEAT,)}


class MaxErr:
    def __init__(self):
        self.err: dict[str, float] = {}

    def add(self, key: str, mine, ref) -> float:
        e = float(np.max(np.abs(np.asarray(mine, dtype=float) - np.asarray(ref, dtype=float)))) if np.size(ref) else 0.0
        self.err[key] = max(self.err.get(key, 0.0), e)
        return e


def check_sim(policy: leg_policy.LegPolicy, dump_path: Path) -> dict:
    with np.load(dump_path) as raw:
        d = {k: raw[k] for k in raw.files}
    meta = policy.meta
    names = [str(n) for n in d["joint_names"]]
    unitree = [names.index(n) for n in leg_policy.UNITREE_LEGS]
    if str(d["hold_arm"]) != "policy" or bool(d["leg_answer"]):
        raise ValueError("the runner is the policy-held arm's without --leg_answer_range")
    # A fresh runner per episode, as each recorded episode is its environment's first: the layer's buffers that `start`
    # clears hold the previous episode's values until then, in the environment and here alike.
    fresh = lambda: insert_policy.InsertRunner(meta, seat_test=str(d["seat_test"]), light_probe=bool(d["light_probe"]))
    runner = fresh()
    if bool(d["sag_on"]) != runner.sag_on:
        raise ValueError(f"the rollout's sag compensation is {bool(d['sag_on'])}, the export's {runner.sag_on}")
    to_unitree = runner.obs.to_unitree
    err, frame_diffs = MaxErr(), []
    episodes = []
    for e in range(d["frame"].shape[0]):
        alive = np.flatnonzero(d["alive"][e] > 0.5)
        g = lambda key, t: d[key][e, t]
        lock = insert_policy.LockEstimate(g("est_o", 0), g("est_a", 0), g("est_u", 0), g("est_t", 0))
        runner = fresh()
        q0, ok = runner.reset(lock, g("root_pos", 0), g("root_quat_w", 0), float(g("start_depth", 0)),
                              float(g("press_draw", 0)), float(g("tool_reach", 0)), g("tool_offset", 0),
                              arm_pose=g("arm_pose", 0))
        ep = {"episode": e, "steps": int(len(alive)), "start_solved": ok,
              "start_solved_isaac": bool(g("ins_active", 0) > 0.5)}
        err.add("reset_ins_ref_rad", runner.ins_ref, g("ins_ref", 0))
        err.add("reset_goal_jaw_m", runner.goal_jaw, g("goal_jaw", 0))
        phase_mismatch, first_mismatch = 0, None
        act_ep, tgt_ep = 0.0, 0.0
        for t in alive:
            if t > 0:
                runner.update(g("root_pos", t), g("root_quat_w", t))
                ly = runner.layer
                for key in LAYER_EXACT:
                    if int(np.asarray(getattr(ly, key))[0]) != int(round(float(g(f"layer_{key}", t)))):
                        if key == "phase":
                            phase_mismatch += 1
                            first_mismatch = int(t) if first_mismatch is None else first_mismatch
                        err.err[f"layer_{key}_mismatched_steps"] = err.err.get(f"layer_{key}_mismatched_steps", 0) + 1
                for key in LAYER_COMPARED:
                    if f"layer_{key}" in d:
                        read = key not in READ_IN or int(ly.phase[0]) in READ_IN[key]
                        err.add(f"layer_{key}" if read else f"layer_{key}_outside_read_phases", getattr(ly, key)[0],
                                g(f"layer_{key}", t))
                err.add("meas_depth_m", runner.meas_depth[0], g("meas_depth", t))
                err.add("meas_lateral_m", runner.meas_lateral[0], g("meas_lateral", t))
                err.add("goal_jaw_m", runner.goal_jaw, g("goal_jaw", t))
                err.add("loop_m", runner.loop, g("loop", t))
                err.add("miss_b_m", runner.miss_b, g("miss_b", t))
                err.add("ins_ref_rad", runner.ins_ref, g("ins_ref", t))
                err.add("ins_geo_rad", runner.ins_geo, g("ins_geo", t))
                err.add("done_for_s", runner.done_for[0], g("done_for", t))
                err.add("since_reset", runner.since_reset, g("since_reset", t))
                err.add("layer_started", float(runner.layer_started), g("layer_started", t))
            q, dq = g("joint_pos", t), g("joint_vel", t)
            stack = runner.frame(g("root_quat_w", t), g("root_ang_vel_b", t), q[unitree], dq[unitree],
                                 g("arm_reported", t), g("arm_rate", t))
            frame_diffs.append(runner.last_frame - g("frame", t))
            err.add("commands_foot_goal", runner.commands[3:6], g("commands", t)[3:6])
            err.add("commands_miss", runner.commands[6:9], g("commands", t)[6:9])
            err.add("commands_press_n", runner.commands[9:12], g("commands", t)[9:12])
            err.add("commands_other", np.r_[runner.commands[:3], runner.commands[12:]],
                    np.r_[g("commands", t)[:3], g("commands", t)[12:]])
            act_ep = max(act_ep, err.add("action", policy.act(stack)[0], g("action", t)))
            legs, arm_t = runner.targets(g("action", t))
            target = g("joint_targets", t)
            tgt_ep = max(tgt_ep, err.add("targets_legs_rad", legs, target[:12][to_unitree]),
                         err.add("targets_arm_rad", arm_t, target[12:18]))
        phases = d["layer_phase"][e, alive].round().astype(int)
        ep.update({"phase_mismatched_steps": phase_mismatch, "first_phase_mismatch_step": first_mismatch,
                   "action_max_abs_err": act_ep, "target_max_abs_err_rad": tgt_ep,
                   "isaac_phases": _transitions(phases), "runner_events": runner.events,
                   "ik_calls": runner.ik_calls, "ik_fail": runner.ik_fail,
                   "ik_fail_isaac": int(d["ik_fail"][e, alive[-1]]), "seat_test": runner.layer.seat_test})
        episodes.append(ep)
    frame_diffs = np.array(frame_diffs)
    return {"dump": str(dump_path), "episodes": len(episodes), "steps": int(sum(ep["steps"] for ep in episodes)),
            "frame_max_abs_err": float(np.abs(frame_diffs).max()), "frame_err_by_block": block_errors(frame_diffs),
            "action_scale": float(np.abs(d["action"][d["alive"] > 0.5]).max()),
            "max_abs_err": dict(sorted(err.err.items())), "by_episode": episodes}


def _transitions(phases: np.ndarray) -> list[list]:
    """(step, phase name) at each change of the recorded layer's phase."""
    out, last = [], None
    for t, p in enumerate(phases):
        if p != last:
            out.append([t, insert_policy.PHASE_NAMES[p]])
            last = p
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", required=True, help="the exported .npz (export_policy.py --task insert)")
    parser.add_argument("--sim", required=True, help="a leg_eval --key_insert run's insert_shadow_dump.npz")
    parser.add_argument("--out", help="write the report here as json")
    parser.add_argument("--brief", action="store_true", help="print the summary without the per-episode logs")
    args = parser.parse_args()
    policy = leg_policy.LegPolicy(args.policy)
    report = {"policy": args.policy, "policy_meta": {k: policy.meta[k] for k in ("task", "variant", "checkpoint",
                                                                                  "checkpoint_iteration")},
              "sim": check_sim(policy, Path(args.sim))}
    text = json.dumps(report, indent=2)
    if args.out:
        Path(args.out).write_text(text)
    if args.brief:
        sim = dict(report["sim"])
        sim["by_episode"] = [{k: v for k, v in ep.items() if k not in ("isaac_phases", "runner_events")}
                             for ep in sim["by_episode"]]
        text = json.dumps({**report, "sim": sim}, indent=2)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Check the dog's numpy leg policy (`leg_policy.py`) against the training code (on the PC).

    # a recorded Isaac rollout through the dog's pipeline (`run_unifp_train.py leg_eval ... --leg_shadow_dump 4`):
    python demos/combiner_key/robot/check_shadow.py --policy checkpoints/<name>.npz --sim <run dir>/shadow_dump.npz
    # a shadow run's record from the dog (`shadow.py`):
    python demos/combiner_key/robot/check_shadow.py --policy checkpoints/<name>.npz --checkpoint <model.pt> \
        --dog logs/leg_shadow/<stamp>

`--sim`: for each recorded episode, from its reset, the dog's `LegObservation` builds every frame from the raw state
(joints in Isaac's order, mapped to Unitree's by name -- independently of the environment's own permutation), the dog's
`LegPolicy` acts on its own history, and both are compared with what the environment built and the checkpoint did; the
joint targets the actuators were given are compared with `leg_targets_unitree`, and the waiting stage's commands with
`waiting_command`.

`--dog`: the record's frames rebuilt by the training code's `interface.single_obs` (torch) from the recorded raw state,
with `interface.permutation_from` doing the joint order; the record's actions recomputed by the checkpoint in torch from
the record's own frames; and the actions the policy gives a *still* robot in the record's first-second pose, open loop
from reset as the shadow run is, against what it gave on the dog -- the part only the dog can check: the IMU's frame
and the joints' signs and offsets as the firmware reports them.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import leg_policy  # noqa: E402

#: The 76-wide frame's blocks (`interface.OBS_SLICES`), for saying where a difference is.
BLOCKS = (("roll_pitch", 0, 2), ("ang_vel", 2, 5), ("dof_pos", 5, 23), ("dof_vel", 23, 41), ("actions", 41, 59),
          ("gait_phase", 59, 61), ("commands", 61, 76))


def block_errors(diff: np.ndarray) -> dict:
    return {name: float(np.abs(diff[..., a:b]).max()) if diff.size else 0.0 for name, a, b in BLOCKS}


def check_sim(policy: leg_policy.LegPolicy, dump_path: Path) -> dict:
    d = np.load(dump_path)
    names = [str(n) for n in d["joint_names"]]
    meta = policy.meta
    unitree = [names.index(n) for n in leg_policy.UNITREE_LEGS]
    arm = [names.index(n[3:] if n.startswith("d1_") else n) for n in meta["dof_names"][12:18]]
    builder = leg_policy.LegObservation(meta)
    waiting = leg_policy.waiting_command(meta)
    frame_err, act_err, target_err, cmd_err, steps, waiting_steps = [], [], [], [], 0, 0
    episodes = d["frame"].shape[0]
    for e in range(episodes):
        builder.reset()
        alive = d["alive"][e] > 0.5
        for t in np.flatnonzero(alive):
            q, dq = d["joint_pos"][e, t], d["joint_vel"][e, t]
            frame = builder.frame(d["root_quat_w"][e, t], d["root_ang_vel_b"][e, t], q[unitree], dq[unitree], q[arm], dq[arm],
                                  d["commands"][e, t])
            frame_err.append(frame - d["frame"][e, t])
            stack = builder.step(frame)
            mine = policy.act(stack)[0]
            act_err.append(float(np.abs(mine - d["action"][e, t]).max()))
            legs = builder.record_action(d["action"][e, t])
            target_err.append(float(np.abs(builder.leg_targets_unitree(legs) - d["joint_pos_target"][e, t][unitree]).max()))
            if d["stage"][e, t] == 0:
                # The reset's own frame is built before any step writes the commands: zero, as `shadow.py` sends it.
                expected = np.zeros_like(waiting) if t == 0 else waiting
                cmd_err.append(float(np.abs(expected - d["commands"][e, t]).max()))
                waiting_steps += 1
            steps += 1
    frame_err = np.array(frame_err)
    return {"dump": str(dump_path), "episodes": int(episodes), "steps": steps, "waiting_steps": waiting_steps,
            "frame_max_abs_err": float(np.abs(frame_err).max()), "frame_err_by_block": block_errors(frame_err),
            "action_max_abs_err": float(max(act_err)), "action_scale": float(np.abs(d["action"]).max()),
            "target_max_abs_err_rad": float(max(target_err)),
            "waiting_command_max_abs_err": float(max(cmd_err)) if cmd_err else None}


def stand_from_joints(meta: dict, q_unitree: np.ndarray) -> dict:
    """Where the measured joints put the feet (base frame, `leg_policy.foot_local`): the stand's spacing and the base's
    height above them -- against Lukas's tape (41 x 28 cm), a check of the joints' signs and zeros as the firmware
    reports them, and the classic stand's base height, which the evaluation could only guess (F-170)."""
    order = leg_policy.UNITREE_LEGS
    feet = {}
    for leg in ("FR", "FL", "RR", "RL"):
        k = order.index(f"{leg}_hip_joint")
        feet[leg] = np.median([leg_policy.foot_local(leg, q[k:k + 3], meta) for q in q_unitree], axis=0)
    front_back = 0.5 * ((feet["FR"][0] - feet["RR"][0]) + (feet["FL"][0] - feet["RL"][0]))
    left_right = 0.5 * ((feet["FL"][1] - feet["FR"][1]) + (feet["RL"][1] - feet["RR"][1]))
    height = -np.mean([f[2] for f in feet.values()]) + meta["kinematics"].get("foot_radius_m", 0.022)
    return {"feet_base_frame_m": {k: np.round(v, 4).tolist() for k, v in feet.items()},
            "front_back_m": round(float(front_back), 4), "left_right_m": round(float(left_right), 4),
            "base_height_above_feet_m": round(float(height), 4)}


def static_prediction(policy: leg_policy.LegPolicy, rec: dict, seconds: float = 1.0, estimates: bool = False):
    """The actions a still robot in the record's first-second mean pose gets, open loop from reset."""
    n = max(1, int(round(seconds / policy.meta["policy_dt"])))
    quat = rec["quat"][:n].mean(axis=0)
    quat /= np.linalg.norm(quat)
    q, arm = rec["q"][:n].mean(axis=0), rec["arm_q"][:n].mean(axis=0)
    builder = leg_policy.LegObservation(policy.meta)
    out, est = [], []
    for t in range(len(rec["t"])):
        frame = builder.frame(quat, np.zeros(3), q, np.zeros(12), arm, np.zeros(6), rec["commands"][t])
        stack = builder.step(frame)
        out.append(builder.record_action(policy.act(stack)[0]))
        est.append(policy.estimate(stack)[0])
    return (np.array(out), np.array(est)) if estimates else np.array(out)


def median_estimates(meta: dict, estimates: np.ndarray) -> dict:
    decoded = [leg_policy.decode_estimates(meta, e) for e in estimates]
    return {k: np.round(np.median([d[k] for d in decoded], axis=0), 3).tolist() for k in decoded[0]}


def check_dog(policy: leg_policy.LegPolicy, record_dir: Path, checkpoint: str | None) -> dict:
    import torch

    from unifp_isaaclab import interface

    rec = dict(np.load(record_dir / "shadow.npz"))
    meta = policy.meta
    T = len(rec["t"])
    resets = np.flatnonzero(rec["reset"] > 0.5)
    # The training code's frame from the record's raw state: joint order by `interface.permutation_from` over the names
    # the record's columns carry (Unitree's legs, the D1's joints, the jaws at 0).
    names = list(leg_policy.UNITREE_LEGS) + [n[3:] if n.startswith("d1_") else n for n in meta["dof_names"][12:18]] \
        + [n for n in interface.ISAACLAB_NAMES[18:]]
    perm = interface.permutation_from(names)
    q_all = np.concatenate((rec["q"], rec["arm_q"], np.zeros((T, 2))), axis=1)
    dq_all = np.concatenate((rec["dq"], np.zeros((T, 8))), axis=1)
    prev = np.zeros((T, interface.NUM_ACTIONS))
    for t in range(1, T):
        if rec["reset"][t] < 0.5:
            prev[t, :12] = np.clip(rec["action"][t - 1], -interface.CLIP_ACTIONS, interface.CLIP_ACTIONS)
    with torch.inference_mode():
        f = lambda x: torch.as_tensor(np.asarray(x), dtype=torch.float64)
        ref = interface.single_obs(f(rec["quat"]), f(rec["gyro"]), f(q_all)[:, perm], f(dq_all)[:, perm], f(prev),
                                   torch.zeros(T, dtype=torch.float64), f(rec["commands"])).numpy()
    ref = np.clip(ref, -interface.CLIP_OBSERVATIONS, interface.CLIP_OBSERVATIONS)
    frame_diff = rec["frame"] - ref
    # The record's stacks, rebuilt from its frames from each reset.
    stacks = np.zeros((T, interface.NUM_OBS))
    hist = np.zeros((interface.FRAME_STACK, interface.NUM_SINGLE_OBS))
    for t in range(T):
        if rec["reset"][t] > 0.5:
            hist[:] = 0.0
        hist = np.roll(hist, -1, axis=0)
        hist[-1] = rec["frame"][t]
        stacks[t] = hist.reshape(-1)
    out = {"record": str(record_dir), "steps": T, "resets": resets.tolist(),
           "frame_vs_training_code_max_abs_err": float(np.abs(frame_diff).max()),
           "frame_vs_training_code_by_block": block_errors(frame_diff),
           "numpy_dog_vs_numpy_pc_action_max_abs_err": float(np.abs(policy.act(stacks) - rec["action"]).max())}
    if checkpoint:
        from unifp_train.eval import EvaluablePolicy

        torch_policy = EvaluablePolicy(checkpoint, device="cpu")
        with torch.inference_mode():
            torch_act = torch_policy.act(torch.as_tensor(stacks, dtype=torch.float32)).numpy()
        out["dog_vs_torch_action_max_abs_err"] = float(np.abs(torch_act - rec["action"]).max())
    out["stand_from_joints"] = stand_from_joints(meta, rec["q"])
    out["rpy_deg_median"] = np.round(np.degrees(np.median(rec["rpy"], axis=0)), 2).tolist()
    out["roll_pitch_as_the_policy_sees_deg_median"] = np.round(np.degrees(np.median(rec["frame"][:, 0:2], axis=0)), 2).tolist()
    out["arm_measured_share"] = round(float(rec["arm_measured"].mean()), 3)
    if len(resets) == 1:
        predicted, predicted_est = static_prediction(policy, rec, estimates=True)
        clipped = np.clip(rec["action"], -interface.CLIP_ACTIONS, interface.CLIP_ACTIONS)
        diff = (clipped - predicted) * meta["action_scale"]
        order = leg_policy.LegObservation(meta).to_unitree
        out["still_robot_prediction"] = {
            "what": "the dog's joint targets against a still robot's in the record's first-second pose (rad, Unitree order)",
            "rms_rad": dict(zip(leg_policy.UNITREE_LEGS, np.sqrt(np.mean(diff[:, order] ** 2, axis=0)).round(4).tolist())),
            "max_rad": dict(zip(leg_policy.UNITREE_LEGS, np.abs(diff[:, order]).max(axis=0).round(4).tolist())),
            "estimates_dog_median": median_estimates(meta, rec["estimate"]),
            "estimates_still_robot_median": median_estimates(meta, predicted_est)}
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", required=True, help="the exported .npz (export_policy.py)")
    parser.add_argument("--sim", help="a leg_eval run's shadow_dump.npz")
    parser.add_argument("--dog", help="a shadow run's record directory (logs/leg_shadow/<stamp>)")
    parser.add_argument("--checkpoint", help="--dog: the .pt the .npz came from, to recompute the actions in torch")
    parser.add_argument("--out", help="write the report here as json")
    args = parser.parse_args()
    policy = leg_policy.LegPolicy(args.policy)
    report = {"policy": args.policy, "policy_meta": {k: policy.meta[k] for k in ("variant", "checkpoint", "checkpoint_iteration")}}
    if args.sim:
        report["sim"] = check_sim(policy, Path(args.sim))
    if args.dog:
        report["dog"] = check_dog(policy, Path(args.dog), args.checkpoint)
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        Path(args.out).write_text(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

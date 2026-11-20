#!/usr/bin/env python3
"""S5's replay gate for the insertion policy (on the PC, CPU only): the numpy runner (`insert_policy.InsertRunner`) on
`66600`'s four recorded stands, the arm closed as `logs/unifp_train/armlink/gate4.py` closes it.

    python demos/combiner_key/robot/insert_gate.py --policy checkpoints/insert_force_ki6_8200.npz --out logs/unifp_train/s5gate

The thresholds are the week 10 log's (2026-11-18, "S5: ki6_force_8200 exported ...", stated before any scoring), not
chosen here. Pass needs all four on every recorded stand and every placement:
  1. J1-J5's target lead over the report <= 15 deg from 1 s on (F-256's);
  2. leg targets <= 1.2 rad from the stand in the first second, <= 0.9 rad after (F-256's; measured as gate4 does,
     |leg action| x 0.25, the target's offset from the policy's default stance);
  3. the IK reference solves on every call (the start included);
  4. the layer feels no touch while its reference is short of the estimated face (no face is modelled: any touch
     before the reference reaches the face is felt in free air, on the dog's own body motion, F-292's failure).
Reported without a pass mark: the hand's lag along the axis, J2's gravity torque as a share of its 3.33 N·m, the
layer's decisions.

What the replay is, and is not:
- **Body and legs:** each recorded frame's legs, roll, pitch and body rate as the dog's pipeline built them
  (`policy_rows.npz`, 501 steps, 10 s). The legs are open loop, as in gate4: the insertion policy's leg targets are
  scored, never applied; the body moves as it moved under `66600`'s reach policy.
- **Base pose:** the frames carry no position. The base is held at x = y = 0 (where the frozen set's robot resets), its
  height from the support feet (FL, RR, RL) as `handreplay.hand` computes it, its attitude the recorded roll and pitch
  and, by default, the recorded yaw less the first frame's (`--yaw zero` holds it at 0, as `handreplay` does).
- **Arm:** closed as gate4 closes it: reports at 9 Hz quantised to 0.1 deg (`reach_arm.ArmReports`), the real
  `Follower` (`demos/combiner_door/robot/arm_link.py`) stepped at each report, the joints following the wire with a
  77 ms first-order lag. It starts at the runner's solved start pose, as the evaluation's arm does.
- **Lock estimate:** where the frozen 96's box puts it (`keyinsert_eval.build_plan`, with `run_unifp_train.py`'s key
  hold overrides of `leg_cfg`), estimate error zero, the v4 key, the evaluation's start (17.5 mm short) and press (8 N).
- **No contact:** nothing stops the key. The reference passes the estimated face about 3.5 s in.
"""
from __future__ import annotations

import os

for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import sys  # noqa: E402
from multiprocessing import Pool  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

import d1_ik  # noqa: E402
from demos.combiner_door.robot import arm_link  # noqa: E402
from demos.combiner_key.robot import insert_policy, leg_policy, reach_arm  # noqa: E402

#: `66600`'s four whole-body stands on the dog (F-257), the frames gate4 replays.
RUNS = tuple(f"results/week_08/runs/20261106T1{s}_reach_stand_live/policy_rows.npz"
             for s in ("40642", "41120", "41455", "41945"))
#: The frozen 96's rows whose box placements the gate uses, and the median placement (each field's median over the 96).
ROWS = (0, 24, 48, 72)
#: The thresholds, as the week 10 log states them (2026-11-18). Not to be changed here.
LEAD_MAX_DEG = 15.0
LEG_FIRST_S_MAX_RAD = 1.2
LEG_AFTER_MAX_RAD = 0.9
FIRST_S = 1.0
#: gate4's arm: the joints' first-order lag behind the wire (s), the report rate (Hz) and quantum (deg).
ARM_LAG_S, REPORT_HZ, REPORT_QUANTUM_DEG = 0.077, 9.0, 0.1
#: `handreplay`'s foot sphere radius (m) and its leg order (Unitree's): the height is from the last three.
FOOT_M = 0.022
LEGS = ("FR", "FL", "RR", "RL")
#: The evaluation's start (`keyinsert_eval.build_plan`: start_off 17.5 mm) and press.
START_DEPTH_M, PRESS_N = -0.0175, 8.0
#: J2's effort limit (N·m, `interface.TORQUE_LIMITS`, `motor_model.D1_EFFORT_LIMIT_NM`).
J2_LIMIT_NM = 3.33


# --- the placements --------------------------------------------------------------------------------------------------

def key_hold_plan():
    """`keyinsert_eval.build_plan` as `leg_eval --key_hold --key_insert` builds it: `run_unifp_train.py` points the key
    task's evaluation draws at the raised box (`leg_cfg.KEYHOLD_*`) before building the plan. Returns the plan's boxes
    (door ahead, axis side, height, door yaw; one row per lock)."""
    from unifp_train import keyinsert_eval, leg_cfg as c

    # `run_unifp_train.py` for `--key_hold` (the raised box) and the evaluation's `--key_turn_deg 0 45 30` (iev.sh's EK).
    override = {"KEY_AXIS_HEIGHT_M": c.KEYHOLD_AXIS_HEIGHT_M, "KEY_AXIS_HEIGHT_JITTER_M": c.KEYHOLD_AXIS_HEIGHT_JITTER_M,
                "KEY_DOOR_AHEAD_M": c.KEYHOLD_DOOR_AHEAD_M, "KEY_AXIS_SIDE_M": c.KEYHOLD_AXIS_SIDE_M,
                "KEY_DOOR_YAW_RAD": c.KEYHOLD_DOOR_YAW_RAD, "KEY_EVAL_DOOR_AHEAD_M": c.KEYHOLD_EVAL_DOOR_AHEAD_M,
                "KEY_EVAL_AXIS_SIDE_M": c.KEYHOLD_EVAL_AXIS_SIDE_M, "KEY_EVAL_FRICTION_NM": c.KEYHOLD_EVAL_FRICTION_NM,
                "KEY_EVAL_DETENT_NM": c.KEYHOLD_EVAL_DETENT_NM, "KEY_EVAL_AXIS_HEIGHT_M": c.KEYHOLD_EVAL_AXIS_HEIGHT_M,
                "KEY_EVAL_DOOR_YAW_RAD": c.KEYHOLD_EVAL_DOOR_YAW_RAD, "KEY_REACH_M": c.KEYHOLD_REACH_M,
                "KEY_THETA_START_RAD": 0.0, "KEY_THETA_END_RAD": math.radians(45.0),
                "KEY_THETA_UNLOCK_RAD": math.radians(30.0)}
    missing = object()
    saved = {k: getattr(c, k, missing) for k in override}
    try:
        for k, v in override.items():
            setattr(c, k, v)
        plan, _ = keyinsert_eval.build_plan("cpu")
    finally:
        for k, v in saved.items():
            if v is missing:
                delattr(c, k)
            else:
                setattr(c, k, v)
    h = plan["handles"]
    return {k: h[k].double().numpy() for k in ("door_ahead", "axis_side", "height", "door_yaw")}


def placements() -> list[dict]:
    """The gate's boxes: `ROWS` of the frozen 96, and each field's median over them."""
    boxes = key_hold_plan()
    out = [{"name": f"row {i}", **{k: float(v[i]) for k, v in boxes.items()}} for i in ROWS]
    out.append({"name": "median", **{k: float(np.median(v)) for k, v in boxes.items()}})
    return out


def lock_estimate(box: dict, face_off_door_m: float) -> insert_policy.LockEstimate:
    """The estimated lock for a box, as `leg_env._place_keys` and `keyinsert_env._place_insert` place it (the robot at
    the origin facing +x): the key's axis on the door `axis_side` to the side at `height`, into the door; the bar level
    to the robot's right; the face `face_off_door_m` proud of the door."""
    psi = box["door_yaw"]
    normal = np.array([-math.cos(psi), -math.sin(psi), 0.0])           # out of the door, toward the robot
    along = np.array([-math.sin(psi), math.cos(psi), 0.0])             # the robot's left along the door
    door = np.array([box["door_ahead"], 0.0, box["height"]]) + (box["axis_side"] / along[1]) * along
    return insert_policy.LockEstimate(door + face_off_door_m * normal, -normal, -along)


# --- the body ----------------------------------------------------------------------------------------------------------

def quat_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """w-first quaternion of Rz(yaw) Ry(pitch) Rx(roll) (UniFP's `euler_from_quat` reads roll and pitch back)."""
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cy * cp * cr + sy * sp * sr, cy * cp * sr - sy * sp * cr, cy * sp * cr + sy * cp * sr,
                     sy * cp * cr - cy * sp * sr])


def base_height(meta: dict, q_unitree, roll: float, pitch: float) -> float:
    """`handreplay.hand`'s base height: the support feet (FL, RR, RL) under the base, the body tilted as recorded."""
    c, s = math.cos(pitch), math.sin(pitch)
    ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    c, s = math.cos(roll), math.sin(roll)
    rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    rot = ry @ rx
    z = [(rot @ leg_policy.foot_local(leg, q_unitree[3 * n:3 * n + 3], meta))[2] for n, leg in enumerate(LEGS)]
    return FOOT_M - float(np.mean(z[1:]))


def frames_state(meta: dict, rows: dict, yaw: str = "recorded") -> dict:
    """The raw inputs a recorded stand gives each step: the IMU's quaternion (roll, pitch) and rate, the legs in Unitree's
    order, and the base's pose (`yaw`: "recorded", less the first frame's, or "zero")."""
    obs = leg_policy.LegObservation(meta)
    f = np.asarray(rows["frame"], dtype=float)
    default = np.asarray(meta["default_dof_pos"], dtype=float)[:12]
    q = (f[:, 5:17] + default)[:, obs.to_unitree]
    dq = (f[:, 23:35] / meta["obs_scale_dof_vel"])[:, obs.to_unitree]
    gyro = f[:, 2:5] / meta["obs_scale_ang_vel"]
    yaws = np.asarray(rows["rpy"], dtype=float)[:, 2]
    yaws = (yaws - yaws[0]) if yaw == "recorded" else np.zeros_like(yaws)
    imu = [quat_rpy(r, p, 0.0) for r, p in f[:, 0:2]]
    base_quat = [quat_rpy(r, p, y) for (r, p), y in zip(f[:, 0:2], yaws)]
    height = np.array([base_height(meta, qi, r, p) for qi, (r, p) in zip(q, f[:, 0:2])])
    base_pos = np.stack((np.zeros_like(height), np.zeros_like(height), height), -1)
    return {"imu": np.array(imu), "gyro": gyro, "q": q, "dq": dq, "base_pos": base_pos, "base_quat": np.array(base_quat),
            "frame": f, "yaw": yaws}


# --- the replay ---------------------------------------------------------------------------------------------------------

def replay(policy, state: dict, box: dict, steps: int | None = None) -> dict:
    """One recorded stand with one placement: the runner drives the arm through gate4's closed loop, the legs open loop."""
    meta = policy.meta
    runner = insert_policy.InsertRunner(meta, seat_test="old")
    lock = lock_estimate(box, meta["insert"]["lock"]["face_off_door_m"])
    q_start, solved = runner.reset(lock, state["base_pos"][0], state["base_quat"][0], START_DEPTH_M, PRESS_N)
    dt = float(meta["policy_dt"])
    n = len(state["frame"]) if steps is None else min(steps, len(state["frame"]))
    # gate4's arm, from the start pose.
    fol = arm_link.Follower(np.array(d1_ik.to_servo_deg(q_start)))
    arm, wire = q_start.copy(), q_start.copy()
    rep = reach_arm.ArmReports(q_start)
    nxt, clock = 0.0, 0.0
    gravity = runner.gravity
    rec = {k: [] for k in ("target", "report", "legs", "phase", "ref", "ref_before", "meas", "true_depth", "j2_share",
                           "frame_err")}
    for k in range(n):
        if clock >= nxt:
            quantum = math.radians(REPORT_QUANTUM_DEG)
            rep.report(clock, np.round(arm / quantum) * quantum)
            nxt += 1.0 / REPORT_HZ
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0,
                                                                   1.0 / REPORT_HZ))))
        rec["ref_before"].append(float(runner.layer.ref_depth[0]))
        if k > 0:
            runner.update(state["base_pos"][k], state["base_quat"][k])
        stack = runner.frame(state["imu"][k], state["gyro"][k], state["q"][k], state["dq"][k], rep.q, rep.dq)
        # The legs and the IMU as the dog's pipeline built them (a check of this replay's own plumbing).
        rec["frame_err"].append(float(max(np.abs(runner.last_frame[:17] - state["frame"][k, :17]).max(),
                                          np.abs(runner.last_frame[23:35] - state["frame"][k, 23:35]).max())))
        action = policy.act(stack)[0]
        legs, arm_t = runner.targets(action)
        fol.goal = np.array(d1_ik.to_servo_deg(arm_t))
        arm = arm + (wire - arm) * (1.0 - math.exp(-dt / ARM_LAG_S))
        clock += dt
        # Records: the target and the report it leads, the legs' offsets, the layer, the hand along the axis.
        rec["target"].append(arm_t.copy())
        rec["report"].append(rep.q.copy())
        rec["legs"].append(float(np.abs(runner.obs.prev_actions[:12]).max() * meta["action_scale"]))
        rec["phase"].append(int(runner.layer.phase[0]))
        rec["ref"].append(float(runner.layer.ref_depth[0]))
        rec["meas"].append(float(runner.meas_depth[0]))
        rot, jaw = runner.kin.tool(arm[None])
        tip = runner.base_pos + insert_policy.quat_apply(runner.base_quat, jaw[0] + rot[0] @ runner.tip_off)
        rec["true_depth"].append(float((tip - lock.origin) @ lock.axis))
        g_b = insert_policy.quat_rotate_inverse(runner.base_quat, np.array([0.0, 0.0, -9.81]))
        rec["j2_share"].append(abs(float(gravity.hold(rep.q[None], g_b)[0, 1])) / J2_LIMIT_NM)
    return score(rec, runner, solved, dt, state, box)


def score(rec: dict, runner, solved: bool, dt: float, state: dict, box: dict) -> dict:
    t = np.arange(len(rec["phase"])) * dt
    lead = np.degrees(np.abs(np.array(rec["target"]) - np.array(rec["report"])))[:, :5]
    first = t < FIRST_S
    legs = np.array(rec["legs"])
    phase, ref, ref_before = np.array(rec["phase"]), np.array(rec["ref"]), np.array(rec["ref_before"])
    meas, true_depth = np.array(rec["meas"]), np.array(rec["true_depth"])
    # A touch: the layer leaves APPROACH for SEARCH (the old seated test; ki6's). Short of the face: its reference, before
    # the touch reset it, still short of the estimated face (depth 0).
    touched = np.flatnonzero((phase[1:] == insert_policy.SEARCH) & (phase[:-1] == insert_policy.APPROACH)) + 1
    early = [int(k) for k in touched if ref_before[k] < 0.0]
    approaching = (phase == insert_policy.APPROACH) & (ref < 0.0)
    passed = np.flatnonzero(ref >= 0.0)
    lag = ref - meas
    lag_true = ref - true_depth
    checks = {
        "lead_deg_max_1s_on": float(lead[~first].max()),
        "legs_rad_max_first_s": float(legs[first].max()),
        "legs_rad_max_after": float(legs[~first].max()),
        "ik_calls": int(runner.ik_calls) + 1, "ik_fail": int(runner.ik_fail) + (0 if solved else 1),
        "touches_short_of_face": early,
    }
    passes = {"lead": checks["lead_deg_max_1s_on"] <= LEAD_MAX_DEG,
              "legs": checks["legs_rad_max_first_s"] <= LEG_FIRST_S_MAX_RAD and checks["legs_rad_max_after"] <= LEG_AFTER_MAX_RAD,
              "ik": checks["ik_fail"] == 0, "no_free_air_touch": not early}
    return {
        "box": box, "steps": int(len(phase)), "passes": passes, "pass": all(passes.values()), **checks,
        "lead_deg_max_by_joint_1s_on": [round(float(v), 2) for v in lead[~first].max(axis=0)],
        "ref_passes_face_s": float(t[passed[0]]) if len(passed) else None,
        "touches_s": [round(float(t[k]), 2) for k in touched],
        "hand_lag_mm_reported": _stats(1e3 * lag[approaching]),
        "hand_lag_mm_true": _stats(1e3 * lag_true[approaching]),
        "j2_gravity_share": _stats(np.array(rec["j2_share"])),
        "decisions": runner.events,
        "final_phase": insert_policy.PHASE_NAMES[int(phase[-1])],
        "base_height_m": [round(float(state["base_pos"][:, 2].min()), 4), round(float(state["base_pos"][:, 2].max()), 4)],
        "yaw_deg": [round(math.degrees(float(state["yaw"].min())), 2), round(math.degrees(float(state["yaw"].max())), 2)],
        "replay_frame_err": float(max(rec["frame_err"])),
    }


def _stats(x: np.ndarray) -> dict | None:
    if not len(x):
        return None
    return {"median": round(float(np.median(x)), 3), "p95": round(float(np.percentile(x, 95)), 3),
            "max": round(float(np.max(x)), 3)}


def _one(job):
    policy_path, run, box, yaw = job
    policy = leg_policy.LegPolicy(policy_path)
    rows = dict(np.load(REPO / run))
    out = replay(policy, frames_state(policy.meta, rows, yaw), box)
    return {"run": Path(run).parent.name.split("_")[0], "placement": box["name"], "yaw": yaw, **out}


def table(results: list[dict]) -> str:
    head = (f"{'stand':16s} {'placement':9s} {'yaw':8s} | lead 1 s+ | legs 1st s, after | IK fail/calls | touch short "
            f"| ref at face | lag mm med/max | J2 share max | pass")
    lines = [head, "-" * len(head)]
    for r in results:
        lag = r["hand_lag_mm_reported"] or {"median": float("nan"), "max": float("nan")}
        lines.append(f"{r['run']:16s} {r['placement']:9s} {r['yaw']:8s} | {r['lead_deg_max_1s_on']:7.2f}   | "
                     f"{r['legs_rad_max_first_s']:5.2f} {r['legs_rad_max_after']:5.2f}       | "
                     f"{r['ik_fail']:3d}/{r['ik_calls']:<4d}     | {len(r['touches_short_of_face']):5d}      | "
                     f"{r['ref_passes_face_s'] if r['ref_passes_face_s'] is not None else float('nan'):6.2f} s   | "
                     f"{lag['median']:5.2f} {lag['max']:5.2f}    | {r['j2_gravity_share']['max']:6.3f}       | "
                     f"{'PASS' if r['pass'] else 'fail'}")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", default="checkpoints/insert_force_ki6_8200.npz")
    parser.add_argument("--out", default="logs/unifp_train/s5gate")
    parser.add_argument("--yaw", choices=("recorded", "zero", "both"), default="both",
                        help="the base's yaw: the recorded IMU yaw less the first frame's (the gate), zero (as handreplay "
                             "has it; a sensitivity, no pass mark), or both")
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args()
    boxes = placements()
    yaws = ("recorded", "zero") if args.yaw == "both" else (args.yaw,)
    jobs = [(str(args.policy), run, box, yaw) for yaw in yaws for run in RUNS for box in boxes]
    with Pool(args.workers) as pool:
        results = pool.map(_one, jobs)
    gate = [r for r in results if r["yaw"] == "recorded"]
    verdict = bool(gate) and all(r["pass"] for r in gate)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    report = {"policy": args.policy, "runs": list(RUNS), "placements": boxes,
              "thresholds": {"lead_deg": LEAD_MAX_DEG, "legs_first_s_rad": LEG_FIRST_S_MAX_RAD,
                             "legs_after_rad": LEG_AFTER_MAX_RAD, "ik": "every call solves",
                             "touch": "none while the reference is short of the estimated face",
                             "source": "results/week_10/notes.md, 2026-11-18, stated before scoring"},
              "assumptions": ["base x, y held at 0 (the frames carry no position); height from the support feet "
                              "(handreplay.hand's); roll, pitch as recorded; yaw as recorded less the first frame's (the "
                              "gate) or 0 (sensitivity)",
                              "legs open loop: the recorded legs, the policy's leg targets scored only",
                              "arm closed as gate4: 9 Hz reports quantised to 0.1 deg, the real Follower, 77 ms lag, "
                              "from the runner's start pose",
                              "lock estimate exact (no estimate error), no face, v4 key, start 17.5 mm short, press 8 N, "
                              "the old seated test"],
              "gate_pass": verdict, "results": results}
    (out / "insert_gate.json").write_text(json.dumps(report, indent=2))
    text = table(results) + f"\n\nGATE (yaw as recorded, all stands and placements): {'PASS' if verdict else 'FAIL'}\n"
    (out / "insert_gate.txt").write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

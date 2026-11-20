#!/usr/bin/env python3
"""S4b's replay gate for a held-mechanism policy (on the PC, CPU only): the numpy runner (`mech_layer.MechRunner`) on
`66600`'s four recorded stands, the arm closed as `insert_gate.py` (gate4) closes it, in free space.

    python demos/combiner_key/robot/mech_gate.py --policy <export>.npz --out logs/unifp_train/s4bgate [--workers 6]

**The thresholds here are PROPOSED** (2026-11-19, with the S4b dog-side build), not yet stated: they are to be stated in
the week 10 log before this is run on any candidate, and changed here only by that statement. The script marks each
check against them; it decides nothing.

**What a free-space replay can check for a mechanism.** No handle is modelled: nothing resists the hand and nothing
moves when it pulls. So it can check the approach -- UniFP's goal sliding from its opening goal to the handle, the arm
following it through the link, the grasp's rule firing on the reports -- and the hold of the goal on the handle while
the law asks for nothing. It cannot check the law: with nothing moving, the travel read correctly stays 0 while the
reference runs on, so the integral winds up to the cap within about a second and the policy is asked for UniFP's
target, the goal plus force / 200 N/m along the path (0.4 m at 80 N), which free space does not hold back. That is the
`stuck` mode's report, with no pass mark. Nor can it check the opening, the force, the latch's release, a torn grasp,
the jaws on a real handle, or the legs answering this policy (they replay `66600`'s).

Modes, each on every stand and placement:
  - `hold` (marked): the approach over `approach_s`, the grasp by the rule on the tool speed the reports give
    (`mech_layer.tool_speed_estimate`) and, for an export with S4c's arrival gate (`approach.grasp_within_m`), on the
    reports' hand (the forward kinematics of the reported joints placed by the replayed base, `MechRunner.hand`) within
    that radius of the layer's goal, as the dog would gate it; then the handle held where it is: the plan's fraction 0 (no travel asked) and the
    travel read as 0, so the law's force is 0 and the goal is the handle, in the goal sphere of the base now. One path
    per placement (the kind does not matter when nothing moves).
  - `free` (reported): the plan runs (the whole travel at the evaluation's 0.10 m/s) and the travel is read as the dog
    would read it (`MechRunner.measure`: the hand by the reports, the replayed base, projected on the path) -- a
    mechanism with no resistance and no mass, closed through the hand. Each of the four kinds.
  - `stuck` (reported): the plan runs and the travel is read as 0 (a handle that does not move): the wind-up above.
    Each of the four kinds.

Proposed pass marks (`hold`; every stand and every placement):
  1. J1-J5's target lead over the report <= 15 deg from 1 s on (F-256's, S5's);
  2. leg targets <= 1.2 rad from the stand in the first second, <= 0.9 rad after (F-256's; |leg action| x 0.25, as
     gate4 measures it);
  3. the grasp comes by the settle rule (the reports' tool speed under 0.10 m/s), not the 50-step fallback (with the
     arrival gate a grasp after the 50 steps counts as the settle rule's only if the speed was under 0.10 m/s on its
     step: the gate holds the fallback back until the hand arrives, so the step count alone no longer tells them apart);
  4. at the grasp, the hand (the forward kinematics of the reports, the replayed base) within `GRASP_MAX_M` of the
     handle;
  5. from `HOLD_FROM_S` after the grasp to the end, the hand within `HOLD_MAX_M` of the handle.
Reported without a mark: the hand's distance from its goal (the policy's tracking) beside its distance from the handle
(which adds the base's motion since the reset: the approach's goal is fixed in the goal sphere, as in training); the
dog's reading of the travel on the held hand and the force the law would ask on it at fraction 0 (open loop); J2's
gravity torque as a share of its 3.33 N·m; in `free`, the reading's lag behind the reference, the hand's distance off
the path and the law's peak; in `stuck`, when the law reaches its cap, how far past the handle the hand goes, and the
lead and legs.

What the replay is, as `insert_gate.py`'s: the recorded legs, roll, pitch and body rate (`policy_rows.npz`, 501 steps,
10 s), the legs open loop (this policy's leg targets scored, never applied); the base at x = y = 0, its height from the
support feet, its attitude as recorded, yaw less the first frame's; the arm through the real `Follower`, 9 Hz reports
quantised to 0.1 deg and a 77 ms lag, from the policy's default pose (every training episode's start). The handles are
S4b's validation placements (`results/manifests/mech_t1_v3.json`), each a world point fixed where the placement is
against the base at the reset; the paths are the evaluation's (`eval_path`, `mech_env._draw_evaluation` without noise).
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
from demos.combiner_key.robot import insert_gate, insert_policy, leg_policy, mech_layer, reach_arm  # noqa: E402

RUNS = insert_gate.RUNS
#: S4b's validation placements (radius, pitch, yaw about the goal sphere's centre), read from the frozen manifest.
MANIFEST = "results/manifests/mech_t1_v3.json"
#: PROPOSED (2026-11-19), to be stated in the week 10 log before any candidate is run. Not to be changed after.
LEAD_MAX_DEG = 15.0
LEG_FIRST_S_MAX_RAD = 1.2
LEG_AFTER_MAX_RAD = 0.9
FIRST_S = 1.0
GRASP_MAX_M = 0.02
HOLD_MAX_M = 0.03
HOLD_FROM_S = 1.0
THRESHOLD_STATUS = "PROPOSED 2026-11-19 with the S4b dog-side build; to be stated in the week 10 log before any run"
ARM_LAG_S, REPORT_HZ, REPORT_QUANTUM_DEG = insert_gate.ARM_LAG_S, insert_gate.REPORT_HZ, insert_gate.REPORT_QUANTUM_DEG
J2_LIMIT_NM = insert_gate.J2_LIMIT_NM
MODES = ("hold", "free", "stuck")


# --- the evaluation's paths ------------------------------------------------------------------------------------------

def hinge_axis(opening: np.ndarray, vertical: bool, flip: bool) -> np.ndarray:
    """`mechanism.hinge_axes` for one opening: vertical (a door) or level (a lid), flipped to the other side."""
    up = np.array([0.0, 0.0, 1.0])
    upright = up - (up @ opening) * opening
    level = np.cross(opening, up)
    if np.linalg.norm(level) < 1e-6:
        level = np.array([0.0, 1.0, 0.0])
    axis = upright if vertical and np.linalg.norm(upright) > 0.3 else level
    axis = axis / np.linalg.norm(axis)
    return -axis if flip else axis


def eval_path(spec: dict, point, base_pos, bearing: float) -> mech_layer.MechPath:
    """The path `mech_env._draw_evaluation` builds for a kind `spec` (`mech_cfg.EVAL_KINDS`, the export's
    `evaluation.kinds`) grasped at `point` with the base at `base_pos`, `bearing` the placement's yaw (which side a door
    swings and a bolt slides): `_directions` without noise, the kind's travel, `hinge_axes`, `Mechanism.grasp`."""
    if spec.get("axis") == "lever":
        raise ValueError("levers are not in the gate")
    point, base = np.asarray(point, dtype=float), np.asarray(base_pos, dtype=float)
    heading = math.atan2(base[1] - point[1], base[0] - point[0])
    back = np.array([math.cos(heading), math.sin(heading), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    side = np.cross(back, up) * (1.0 if bearing >= 0.0 else -1.0)
    opening = {"pull": back, "push": -back, "side": side, "up": up, "down": -up, "back": back}[spec["opening"]]
    hinge = bool(spec["hinge"])
    vertical = (not hinge) or spec.get("axis") == "vertical"
    outward = (not hinge) or spec.get("hinge_side") == "outward"
    radius = float(spec["radius_m"]) if hinge else 1.0
    travel = float(spec["radius_m"]) * float(spec["angle_rad"]) if hinge else float(spec["travel_m"])
    axis = hinge_axis(opening, vertical, outward and bearing >= 0.0)
    return mech_layer.MechPath.from_grasp(hinge, point, opening, axis, radius, travel)


def placements(path: str | Path = MANIFEST) -> list[list[float]]:
    return json.loads((REPO / path).read_text())["sets"]["validation"]["placements"]


# --- the replay ---------------------------------------------------------------------------------------------------------

def replay(policy, state: dict, placement, kind: str, mode: str, approach_s: float | None = None,
           steps: int | None = None) -> dict:
    """One recorded stand, one placement and kind, one mode: the runner drives the arm through gate4's closed loop, the
    legs open loop."""
    meta = policy.meta
    mech = meta["mechanism"]
    dt = float(meta["policy_dt"])
    slide_s = float(mech["approach"]["default_slide_s"] if approach_s is None else approach_s)
    runner = mech_layer.MechRunner(meta, plan_fraction=0.0 if mode == "hold" else None)
    base_pos0, base_quat0 = state["base_pos"][0], state["base_quat"][0]
    handle = mech_layer.sphere_to_world(placement, base_pos0, base_quat0, mech["goal_sphere"]["centre_offset"])
    path = eval_path(mech["evaluation"]["kinds"][kind], handle, base_pos0, float(placement[2]))
    runner.reset(placement, base_pos0, base_quat0, traj_steps=slide_s / dt)
    n = len(state["frame"]) if steps is None else min(steps, len(state["frame"]))
    q_start = np.asarray(meta["default_dof_pos"], dtype=float)[12:18]
    fol = arm_link.Follower(np.array(d1_ik.to_servo_deg(q_start)))
    arm, wire = q_start.copy(), q_start.copy()
    rep = reach_arm.ArmReports(q_start)
    nxt, clock = 0.0, 0.0
    gravity = insert_policy.ArmGravity(runner.kin, mech["gravity"])
    rec = {k: [] for k in ("target", "report", "legs", "grasped", "ref", "s", "force", "hand_rep", "hand_true", "goal_w",
                           "s_dog", "off_dog", "j2_share", "frame_err", "speed", "tool_to_goal")}
    for k in range(n):
        if clock >= nxt:
            quantum = math.radians(REPORT_QUANTUM_DEG)
            rep.report(clock, np.round(arm / quantum) * quantum)
            nxt += 1.0 / REPORT_HZ
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0,
                                                                   1.0 / REPORT_HZ))))
        base_pos, base_quat = state["base_pos"][k], state["base_quat"][k]
        speed = mech_layer.tool_speed_estimate(runner.kin, rep.q, rep.dq, base_quat, runner.tool_offset)
        s_dog, off_dog = runner.measure(path, rep.q, base_pos, base_quat)
        if k > 0:
            s_read = s_dog if mode == "free" else 0.0
            # The arrival gate (if the export has one) on the dog's own reading of the tool: the reports' forward
            # kinematics placed by the base.
            runner.update(base_pos, base_quat, speed, s=s_read, path=path, tool_pos=runner.hand(rep.q, base_pos, base_quat))
        stack = runner.frame(state["imu"][k], state["gyro"][k], state["q"][k], state["dq"][k], rep.q, rep.dq)
        rec["frame_err"].append(float(max(np.abs(runner.last_frame[:17] - state["frame"][k, :17]).max(),
                                          np.abs(runner.last_frame[23:35] - state["frame"][k, 23:35]).max())))
        action = policy.act(stack)[0]
        legs, arm_t = runner.targets(action)
        fol.goal = np.array(d1_ik.to_servo_deg(arm_t))
        arm = arm + (wire - arm) * (1.0 - math.exp(-dt / ARM_LAG_S))
        clock += dt
        ly = runner.layer
        rec["target"].append(arm_t.copy())
        rec["report"].append(rep.q.copy())
        rec["legs"].append(float(np.abs(runner.obs.prev_actions[:12]).max() * meta["action_scale"]))
        rec["grasped"].append(bool(ly.grasped))
        rec["ref"].append(float(ly.plan.ref))
        rec["s"].append(float(ly.s))
        rec["force"].append(float(ly.law.force))
        rec["hand_rep"].append(runner.hand(rep.q, base_pos, base_quat))
        rec["hand_true"].append(runner.hand(arm, base_pos, base_quat))
        rec["goal_w"].append(mech_layer.sphere_to_world(ly.goal, base_pos, base_quat, mech["goal_sphere"]["centre_offset"]))
        rec["s_dog"].append(s_dog)
        rec["off_dog"].append(off_dog)
        rec["speed"].append(speed)
        rec["tool_to_goal"].append(np.nan if k == 0 or ly.tool_to_goal_m is None else ly.tool_to_goal_m)
        g_b = insert_policy.quat_rotate_inverse(base_quat, np.array([0.0, 0.0, -9.81]))
        rec["j2_share"].append(abs(float(gravity.hold(rep.q[None], g_b)[0, 1])) / J2_LIMIT_NM)
    return score(rec, runner, path, handle, dt, mode, kind, placement, state, meta)


def score(rec: dict, runner, path, handle, dt: float, mode: str, kind: str, placement, state, meta) -> dict:
    t = np.arange(len(rec["grasped"])) * dt
    lead = np.degrees(np.abs(np.array(rec["target"]) - np.array(rec["report"])))[:, :5]
    first = t < FIRST_S
    legs = np.array(rec["legs"])
    grasped = np.array(rec["grasped"])
    hand_rep, goal_w = np.array(rec["hand_rep"]), np.array(rec["goal_w"])
    to_handle = np.linalg.norm(hand_rep - handle, axis=-1)
    to_goal = np.linalg.norm(hand_rep - goal_w, axis=-1)
    grasp_events = [ev for ev in runner.events if ev["event"] == "grasp"]
    g = grasp_events[0]["step"] if grasp_events else None
    ly = runner.layer
    gated = ly.grasp_within is not None
    by_rule = bool(grasp_events) and ((g - ly.engage_step) < ly.max_wait
                                      or (gated and np.float32(rec["speed"][g]) < ly.settle_speed))
    # The gate's distance (the reports' hand from the goal each update read), from the engage step to the grasp.
    reach = np.array(rec["tool_to_goal"])[ly.engage_step:(len(t) if g is None else g + 1)]
    out = {"stand": None, "placement": [round(float(v), 6) for v in placement], "kind": kind, "mode": mode,
           "steps": int(len(t)), "grasp_step": g, "engage_step": ly.engage_step,
           "grasp_by_settle_rule": by_rule,
           "grasp_within_m": None if not gated else round(float(ly.grasp_within), 6),
           "tool_speed_at_grasp_m_s": round(float(rec["speed"][g]), 4) if g is not None else None,
           "tool_to_goal_at_grasp_m": round(float(rec["tool_to_goal"][g]), 4) if g is not None else None,
           "tool_to_goal_min_from_engage_m": round(float(np.nanmin(reach)), 4) if np.isfinite(reach).any() else None,
           "lead_deg_max_1s_on": float(lead[~first].max()), "lead_deg_max_by_joint_1s_on":
               [round(float(v), 2) for v in lead[~first].max(axis=0)],
           "legs_rad_max_first_s": float(legs[first].max()), "legs_rad_max_after": float(legs[~first].max()),
           "hand_to_handle_at_grasp_m": round(float(to_handle[g]), 4) if g is not None else None,
           "hand_to_goal_at_grasp_m": round(float(to_goal[g]), 4) if g is not None else None,
           "j2_gravity_share": _stats(np.array(rec["j2_share"])),
           "replay_frame_err": float(max(rec["frame_err"])), "decisions": runner.events}
    if g is not None:
        hold = t >= t[g] + HOLD_FROM_S
        out["hand_to_handle_held_m"] = _stats(to_handle[hold]) if hold.any() else None
        out["hand_to_goal_held_m"] = _stats(to_goal[hold]) if hold.any() else None
        after = np.arange(len(t)) >= g
        s_dog, force = np.array(rec["s_dog"]), np.array(rec["force"])
        out["dog_reading_held_m"] = _stats(s_dog[after])
        out["off_path_held_m"] = _stats(np.array(rec["off_dog"])[after])
        out["law_peak_n"] = round(float(np.abs(force).max()), 3)
        if mode == "hold":
            # The law the dog would run on its own reading of the held hand, open loop, the reference at 0.
            law = mech_layer.ForceLaw(ly.law_cfg["kp"], ly.law_cfg["ki"], ly.law_cfg["max_n"], ly.law_cfg["bleed_s"],
                                      meta["mechanism"]["law"]["evaluation"]["arrived_m"], dt)
            open_loop = np.array([float(law.step(0.0, s, True)) for s in s_dog[after]])
            out["law_on_dog_reading_open_loop_n"] = _stats(open_loop)
        if mode == "free":
            ref = np.array(rec["ref"])
            out["reading_behind_ref_m"] = _stats((ref - s_dog)[after], signed=True)
            out["final_fraction"] = round(float(s_dog[-1] / path.travel), 3)
        if mode == "stuck":
            cap = np.flatnonzero(after & (np.abs(force) >= ly.law.max - 1e-3))
            out["law_cap_after_grasp_s"] = round(float(t[cap[0]] - t[g]), 3) if len(cap) else None
            beyond = (hand_rep - handle) @ path.tangent(0.0)
            out["hand_past_handle_along_path_m"] = _stats(beyond[after], signed=True)
    if mode == "hold":
        held = out.get("hand_to_handle_held_m")
        marks = {"lead": out["lead_deg_max_1s_on"] <= LEAD_MAX_DEG,
                 "legs": out["legs_rad_max_first_s"] <= LEG_FIRST_S_MAX_RAD and out["legs_rad_max_after"] <= LEG_AFTER_MAX_RAD,
                 "grasp_by_settle_rule": bool(by_rule),
                 "grasp_on_handle": g is not None and out["hand_to_handle_at_grasp_m"] <= GRASP_MAX_M,
                 "held_on_handle": held is not None and held["max"] <= HOLD_MAX_M}
        out["marks_under_proposed_thresholds"] = marks
        out["all_marks"] = all(marks.values())
    out["base_height_m"] = [round(float(state["base_pos"][:, 2].min()), 4), round(float(state["base_pos"][:, 2].max()), 4)]
    out["yaw_deg"] = [round(math.degrees(float(state["yaw"].min())), 2), round(math.degrees(float(state["yaw"].max())), 2)]
    return out


def _stats(x: np.ndarray, signed: bool = False) -> dict | None:
    if not len(x):
        return None
    out = {"median": round(float(np.median(x)), 4), "p95": round(float(np.percentile(x, 95)), 4),
           "max": round(float(np.max(x)), 4)}
    if signed:
        out["min"] = round(float(np.min(x)), 4)
    return out


def _one(job):
    policy_path, run, placement, kind, mode, approach_s, yaw = job
    policy = leg_policy.LegPolicy(policy_path)
    rows = dict(np.load(REPO / run))
    out = replay(policy, insert_gate.frames_state(policy.meta, rows, yaw), placement, kind, mode, approach_s)
    out["stand"], out["yaw"] = Path(run).parent.name.split("_")[0], yaw
    return out


def jobs_for(policy_path: str, modes, approach_s: float | None, rows=None, yaw: str = "recorded") -> list[tuple]:
    places = placements()
    if rows is not None:
        places = [places[i] for i in rows]
    kinds = ("drawer", "latch", "door", "button")
    jobs = []
    for mode in modes:
        for run in RUNS:
            for p in places:
                for kind in (("drawer",) if mode == "hold" else kinds):
                    jobs.append((str(policy_path), run, p, kind, mode, approach_s, yaw))
    return jobs


def table(results: list[dict]) -> str:
    lines = [f"{'stand':16s} {'mode':5s} {'kind':6s} {'placement (r, pitch, yaw)':27s} | lead 1s+ | legs 1st/after | "
             f"grasp step rule | hand-handle grasp / held max mm | law peak N | marks"]
    lines.append("-" * len(lines[0]))
    for r in results:
        held = r.get("hand_to_handle_held_m") or {}
        grasp_mm = 1e3 * r["hand_to_handle_at_grasp_m"] if r["hand_to_handle_at_grasp_m"] is not None else float("nan")
        mark = ("all" if r.get("all_marks") else "not all") if r["mode"] == "hold" else "-"
        lines.append(f"{r['stand']:16s} {r['mode']:5s} {r['kind']:6s} {str(r['placement']):27s} | {r['lead_deg_max_1s_on']:7.2f} | "
                     f"{r['legs_rad_max_first_s']:4.2f} {r['legs_rad_max_after']:4.2f}      | {str(r['grasp_step']):>4s} "
                     f"{'yes' if r['grasp_by_settle_rule'] else 'no ':3s}   | {grasp_mm:6.1f} / "
                     f"{1e3 * held.get('max', float('nan')):6.1f}            | {r.get('law_peak_n', float('nan')):7.2f}  | {mark}")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", required=True, help="an export_policy.py --task mechanism .npz")
    parser.add_argument("--out", default="logs/unifp_train/s4bgate")
    parser.add_argument("--modes", nargs="+", choices=MODES, default=list(MODES))
    parser.add_argument("--approach_s", type=float, default=None,
                        help="the approach's slide, s (default the export's default_slide_s, 2.0: training draws 1-3)")
    parser.add_argument("--rows", type=int, nargs="+", default=None, help="only these validation placements")
    parser.add_argument("--yaw", choices=("recorded", "zero"), default="recorded",
                        help="the base's yaw: as recorded less the first frame's (the gate), or held at 0 (a sensitivity)")
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args()
    jobs = jobs_for(args.policy, args.modes, args.approach_s, args.rows, args.yaw)
    with Pool(args.workers) as pool:
        results = pool.map(_one, jobs)
    held = [r for r in results if r["mode"] == "hold"]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    report = {"policy": args.policy, "runs": list(RUNS), "manifest": MANIFEST, "placements": placements(), "yaw": args.yaw,
              "thresholds": {"status": THRESHOLD_STATUS, "lead_deg": LEAD_MAX_DEG, "legs_first_s_rad": LEG_FIRST_S_MAX_RAD,
                             "legs_after_rad": LEG_AFTER_MAX_RAD, "grasp_by": "the settle rule, not the 50-step fallback",
                             "grasp_max_m": GRASP_MAX_M, "hold_max_m": HOLD_MAX_M, "hold_from_s": HOLD_FROM_S},
              "assumptions": ["base x, y held at 0; height from the support feet; roll, pitch as recorded; yaw as "
                              "recorded less the first frame's (insert_gate.frames_state)",
                              "legs open loop: the recorded legs, the policy's leg targets scored only",
                              "arm closed as gate4: 9 Hz reports quantised to 0.1 deg, the real Follower, 77 ms lag, from "
                              "the policy's default pose",
                              "handles at S4b's validation placements against the reset's base, fixed in the world; the "
                              "evaluation's paths; no handle modelled: nothing resists or moves",
                              "the grasp by the rule on the reports' tool speed and, with the export's arrival gate "
                              "(approach.grasp_within_m), the reports' hand on the replayed base within it of the "
                              "layer's goal; the jaws not modelled"],
              "all_hold_marks": bool(held) and all(r["all_marks"] for r in held), "results": results}
    (out / "mech_gate.json").write_text(json.dumps(report, indent=2))
    text = (table(results) + f"\n\nThresholds: {THRESHOLD_STATUS}.\n"
            f"hold, every stand and placement, all marks under the proposed thresholds: "
            f"{'yes' if report['all_hold_marks'] else 'no'}\n")
    (out / "mech_gate.txt").write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

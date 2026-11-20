#!/usr/bin/env python3
"""Check the dog's numpy mechanism runner (`mech_layer.py`) against a recorded Isaac rollout (on the PC, CPU only), and
measure on it what the dog's own reading of the travel would have been.

    # the rollout (`run_unifp_train.py mech_eval --task mechanism --force_law ... --mech_shadow_dump N`):
    python demos/combiner_key/robot/check_mech.py --policy <export>.npz --sim <run dir>/mech_shadow_dump.npz \
        [--out report.json] [--brief] [--grasp_within_m R]

**Parity.** For each recorded episode, from its reset: the runner is reset with the episode's handle placement, the
approach's slide (the environment's per-robot draw, `traj_steps`) and the base's pose; each step it is given only what
the environment had -- the base's pose; the tool's true speed; the mechanism's true travel `s` and, at the grasp, its
path; the joints in Isaac's order, mapped to Unitree's by name; the IMU; the arm as the D1's link reported it; and, for
S4c's arrival gate, the tool's distance from its goal as `_engage_due` measured it (`env_goal_world`) -- and it runs the
grasp's rule, the plan, the goal and the law on its own state. The gate's radius is the export's
(`approach.grasp_within_m`; `--grasp_within_m` for an export without it), refused if the rollout recorded another. The grasp is made on the environment's step
(`grasp_now`); whether the runner's own rule fired on that same step is counted apart. The numpy network acts on the
runner's own history; the targets are computed from the action the checkpoint took in the rollout, so the replay stays
on the recorded trajectory. Each quantity is compared with what the environment computed: the largest absolute
difference over every alive step of every episode (the law's buffers after the step only where the episode did not end
on it: those are its reset's). An episode whose UniFP goal moved on before the grasp (its hold ended before the tool
settled; this layer holds the handle instead) has its goal, roll, frame and action compared apart.

**The gate on the layer's own goal** (`grasp_gate`). The rule again with the distance from the layer's goal (as on the
dog) in place of the environment's: its mismatches, apart once UniFP's goal has moved on (the environment's gate then
measures the tool against the moved goal, not the handle); the tool's distance from the handle's placement at each of
the environment's grasps, those after the goal moved on apart; and how near the radius the distance came on a step the
settle rule allowed, the margin a float difference would need to flip a step.

**The dog's reading** (`--no_dog` skips it). On every grasped step, the travel the dog would read -- a point projected on
the path (`MechPath.project`) -- from four points, each adding one thing the dog would add:
  - `tip`: the simulator's own controlled point (the grasp spring's stretch between the tool and the handle alone);
  - `fk_true`: the forward kinematics of the true joints placed by the true base (adds the kinematic model);
  - `fk_reported`: of the joints as the link reported them (adds the reports' rate, lag and quantum);
  - `fk_reported_odometry`: and the base by `LegOdometry` from the reset's true pose, the feet in contact above
    `--contact_n`, the attitude the simulator's (adds the odometry; not the IMU's drift).
Each against the true `s`, and the law replayed open loop on the recorded reference with that reading in place of the
true one (open loop: the policy saw the true one, so this is not what the dog's loop would have done). And when the
grasp's rule would have fired on the tool speed the reports give (`tool_speed_estimate`) and, with the gate, the
reported hand on the odometry's base, against the environment's.
None of it is validated on the dog.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import base_odometry, insert_policy, leg_policy, mech_layer  # noqa: E402
from demos.combiner_key.robot.check_shadow import block_errors  # noqa: E402

F32 = np.float32
#: The feet in the dump are `gait.FEET`'s (FL, FR, RL, RR); the dog's order is Unitree's (FR, FL, RR, RL).
TO_UNITREE_FEET = np.array([1, 0, 3, 2])
READINGS = ("tip", "fk_true", "fk_reported", "fk_reported_odometry")


class MaxErr:
    def __init__(self):
        self.err: dict[str, float] = {}

    def add(self, key: str, mine, ref) -> float:
        e = float(np.max(np.abs(np.asarray(mine, dtype=float) - np.asarray(ref, dtype=float)))) if np.size(ref) else 0.0
        self.err[key] = max(self.err.get(key, 0.0), e)
        return e


def path_at(d: dict, e: int, t: int) -> mech_layer.MechPath:
    """The environment's mechanism geometry at step `t` (set at the grasp), as a `MechPath`."""
    g = lambda key: np.asarray(d[key][e, t], dtype=float)
    return mech_layer.MechPath(bool(d["mech_hinge"][e, t] > 0.5), g("mech_start"), g("mech_direction"), g("mech_axis"),
                               g("mech_radial"), g("mech_centre"), float(d["mech_radius"][e, t]),
                               float(d["mech_travel"][e, t]))


def _speed(v) -> np.float32:
    v = np.asarray(v, dtype=F32)
    return F32(np.sqrt(F32((v * v).sum(dtype=F32))))


def load(dump_path: Path) -> dict:
    with np.load(dump_path) as raw:
        return {k: raw[k] for k in raw.files}


def runner_for(meta: dict, d: dict, e: int, grasp_within_m: float | None | str = "export") -> mech_layer.MechRunner:
    """A runner set to the rollout's law, plan speed and delay (`mech_eval`'s, recorded in the dump) and the grasp's
    arrival gate (`grasp_within_m`: the export's, or the one given)."""
    law = json.loads(str(d["force_law"]))
    conditions = json.loads(str(d["conditions"]))
    return mech_layer.MechRunner(meta, speed_m_s=conditions["speed_m_s"], gains=(law["kp"], law["ki"], law["max"]),
                                 bleed_s=law.get("bleed_s"), engage_delay_steps=int(d["engage_delay_steps"]),
                                 grasp_within_m=grasp_within_m)


def rollout_gate(d: dict, dump_path: Path) -> tuple[float | None, int | None, str | None]:
    """The arrival gate's radius and the fallback's steps the rollout ran with: the dump's meta (`mech_shadow.py` since
    S4c), else the evaluation's run.json beside the dump (its `--mech_grasp_within_m`, `--mech_engage_max_wait_s`), else
    unknown (None, None, None)."""
    if "grasp_within_m" in d:
        within = json.loads(str(d["grasp_within_m"]))
        wait = int(d["engage_max_wait_steps"]) if "engage_max_wait_steps" in d else None
        return (None if within is None else float(within)), wait, "dump"
    record = Path(dump_path).parent / "run.json"
    if record.exists():
        args = json.loads(record.read_text()).get("arguments") or {}
        if "mech_grasp_within_m" in args:
            within, wait_s = args.get("mech_grasp_within_m"), args.get("mech_engage_max_wait_s")
            wait = None if wait_s is None else int(math.ceil(float(wait_s) / float(d["policy_dt"])))
            return (None if within is None else float(within)), wait, "run.json"
    return None, None, None


def env_goal_world(d: dict, e: int, t: int) -> np.ndarray:
    """`_goal_world()` as `_engage_due` read it on step `t`: the goal the frame before was built with
    (`goal_current[t - 1]`; UniFP's goal steps after the rule) placed by the goal sphere of the base after the step
    (`goal_centre[t]`, the base's yaw at `t`). Reproduces the environment's `grasp_error` at every grasp."""
    g = lambda key, k: np.asarray(d[key][e, k], dtype=float)
    yaw = insert_policy.yaw_quat(g("root_quat_w", t))
    return g("goal_centre", t) + insert_policy.quat_apply(yaw, leg_policy.sphere_to_cart(*g("goal_current", t - 1)))


def check_sim(policy, dump_path: Path, dog: bool = True, contact_n: float = 10.0,
              grasp_within_m: float | None | str = "export") -> dict:
    """`grasp_within_m`: the arrival gate's radius the layer runs, m (None: off); "export" takes the export's
    (`approach.grasp_within_m`). Refused when the rollout recorded another (`rollout_gate`)."""
    d = load(dump_path)
    meta = policy.meta
    if json.loads(str(d["force_law"])) is None:
        raise ValueError("the rollout ran without the law (mech_eval --force_law)")
    if bool(d["leg_answer"]):
        raise ValueError("the runner has no --leg_answer_range")
    mech = meta["mechanism"]
    if bool(d["roll_objective"]) != bool(mech["commands"]["roll_objective"]):
        raise ValueError(f"the rollout's roll objective is {bool(d['roll_objective'])}, the export's "
                         f"{mech['commands']['roll_objective']}")
    if not np.allclose(d["tool_offset"], mech["tool"]["offset_m"]) or str(d["tool_body"]) != mech["tool"]["body"]:
        raise ValueError(f"the rollout's tool is {d['tool_body']} {d['tool_offset']}, the export's {mech['tool']}")
    law = json.loads(str(d["force_law"]))
    if not np.allclose(d["law_max"], law["max"]):
        raise ValueError("the rollout's law cap differs between environments (a drawn cap): the runner has one")
    within = mech["approach"].get("grasp_within_m") if grasp_within_m == "export" else grasp_within_m
    within = None if within is None else float(within)
    within_from = "export" if grasp_within_m == "export" else "--grasp_within_m"
    ran_within, ran_wait, ran_source = rollout_gate(d, dump_path)
    if ran_source is not None and ((ran_within is None) != (within is None)
                                   or (within is not None and F32(ran_within) != F32(within))):
        raise ValueError(f"the rollout's grasp gate is {ran_within} m ({ran_source}), the layer's {within} ({within_from}): "
                         "re-export the policy, or pass --grasp_within_m")
    if ran_wait is not None and ran_wait != int(mech["approach"]["max_wait_steps"]):
        raise ValueError(f"the rollout's settle fallback is {ran_wait} steps ({ran_source}), the export's "
                         f"{mech['approach']['max_wait_steps']}")
    names = [str(n) for n in d["joint_names"]]
    unitree = [names.index(n) for n in leg_policy.UNITREE_LEGS]
    arm_idx = [names.index(n[3:] if n.startswith("d1_") else n) for n in meta["dof_names"][12:18]]
    kind_names = [str(k) for k in d["kind_names"]]
    err, frame_diffs, resampled_frames, episodes = MaxErr(), [], [], []
    rule_mismatch = 0
    # The rule with the layer's own goal (the handle, as on the dog) in place of the environment's distance: apart once
    # UniFP's goal has moved on, which the layer does not reproduce.
    own_mismatch = {"goal_as_env": 0, "after_goal_moved_on": 0}
    margin, held_back = None, 0
    for e in range(d["frame"].shape[0]):
        alive = np.flatnonzero(d["alive"][e] > 0.5)
        g = lambda key, t: d[key][e, t]
        runner = runner_for(meta, d, e, within)
        goals_before = {}
        to_unitree = runner.obs.to_unitree
        runner.reset(g("handle_sphere", 0), g("root_pos", 0), g("root_quat_w", 0), traj_steps=float(d["traj_steps"][e]))
        err.add("engage_step", runner.layer.engage_step, g("engage_step", 0))
        grasped = d["mech_grasped"][e] > 0.5
        grasp_sim = int(np.flatnonzero(grasped)[0]) if grasped.any() else None
        ep = {"episode": e, "env": int(d["env_ids"][e]), "kind": kind_names[int(d["kinds"][e])],
              "level_n": float(d["levels_n"][e]), "steps": int(len(alive)), "grasp_step_isaac": grasp_sim,
              "grasp_error_m": round(float(d["grasp_error"][e, grasp_sim]), 4) if grasp_sim is not None else None}
        ep_err = MaxErr()
        resampled = None
        # UniFP's goal moved on before the grasp (its hold ended first): the layer does not reproduce that, and the roll
        # drawn with the new goal stays in the commands after the grasp (`hold_roll`), so such an episode's goal, roll,
        # frame and action are compared apart (`*_resampled_episodes`).
        pre = alive[~grasped[alive]]
        # A goal held on the handle before the grasp (`mech_env._hold_handle_until_grasp`, S4c with the gate) has not
        # moved on, whatever UniFP's generator does underneath it.
        unheld = d["goal_hold_mask"][e] < 0.5
        moved = bool(len(pre)) and bool(((np.abs(d["goal_goal"][e, pre] - d["handle_sphere"][e, pre]).max(-1) > 1e-6)
                                         & unheld[pre]).any())
        tag = "_resampled_episodes" if moved else ""
        for t in alive:
            last = bool(d["ended"][e, t] > 0.5)
            if t > 0:
                now = bool(grasped[t] and not grasped[t - 1])
                was = runner.layer.ever_grasped
                goals_before[int(t)] = runner.layer.goal.copy()
                env_dist = own_dist = None
                if not was:
                    # The arrival gate's distance: the environment's (`env_goal_world`) is the layer's input, as the
                    # tool's true speed is; the layer's own goal's beside it.
                    tip = np.asarray(g("tip_pos", t), dtype=float)
                    env_dist = float(np.linalg.norm(tip - env_goal_world(d, e, t)))
                    own_dist = runner.layer.tool_to_goal(tip, g("root_pos", t), g("root_quat_w", t))
                    goal_moved = resampled is not None
                runner.update(g("root_pos", t), g("root_quat_w", t), _speed(g("tool_vel", t)),
                              s=g("mech_s", t) if grasped[t] else None, path=path_at(d, e, t) if now else None,
                              grasp_now=now, tool_to_goal_m=env_dist)
                if not was and runner.layer.rule_due != now:
                    rule_mismatch += 1
                    ep.setdefault("rule_mismatch_steps", []).append(int(t))
                if not was:
                    err.add("tool_to_goal_m" + ("_after_goal_moved_on" if goal_moved else ""), own_dist, env_dist)
                    if (runner.layer.settle_due and runner.layer.within_reach(own_dist)) != now:
                        own_mismatch["after_goal_moved_on" if goal_moved else "goal_as_env"] += 1
                    if within is not None and runner.layer.settle_due:
                        gap = abs(float(F32(env_dist)) - float(F32(within)))
                        margin = gap if margin is None else min(margin, gap)
                        held_back += int(not runner.layer.rule_due)
                if (resampled is None and not grasped[t] and unheld[t]
                        and np.abs(g("goal_goal", t) - g("handle_sphere", t)).max() > 1e-6):
                    resampled = int(t)
                err.add("since_reset", runner.layer.since_reset, g("since_reset", t))
                err.add("grasped", float(runner.layer.grasped), float(grasped[t]))
                err.add("ref_m", runner.layer.plan.ref, g("tg_ref", t))
                err.add("target_m", runner.layer.plan.target, g("tg_target", t))
                err.add("goal_centre_m", mech_layer.goal_sphere_centre(g("root_pos", t), g("root_quat_w", t),
                                                                       runner.layer.centre_offset), g("goal_centre", t))
            # The goal: held on the handle once grasped, the approach's before; after UniFP's resample (which this layer
            # does not reproduce) apart, and left out of the frame's comparison.
            moved_on = resampled is not None and not grasped[t]
            stack = runner.frame(g("root_quat_w", t), g("root_ang_vel_b", t), g("joint_pos", t)[unitree],
                                 g("joint_vel", t)[unitree], g("arm_reported", t), g("arm_rate", t))
            ref_cmd = g("commands", t)
            goal_key = "commands_goal_after_resample" if moved_on else (
                "commands_goal_held" if grasped[t] else "commands_goal_approach" + tag)
            for store in (err, ep_err):
                store.add(goal_key, runner.commands[3:6], ref_cmd[3:6])
            frame_diff = runner.last_frame - g("frame", t)
            (resampled_frames if moved else frame_diffs).append(frame_diff)
            err.add("commands_velocity", runner.commands[0:3], ref_cmd[0:3])
            err.add("commands_roll" + tag, runner.commands[6:9], ref_cmd[6:9])
            err.add("commands_force_n", runner.commands[9:12], ref_cmd[9:12])
            err.add("commands_base_force", runner.commands[12:15], ref_cmd[12:15])
            act = policy.act(stack)[0]
            ep_err.add("action", act, g("action", t))
            err.add("action" + tag, act, g("action", t))
            legs, arm = runner.targets(g("action", t))
            target = g("joint_targets", t)
            err.add("targets_raw_rad", runner.raw_targets, target)
            err.add("targets_legs_rad", legs, target[:12][to_unitree])
            err.add("targets_arm_clamped_vs_raw_rad", arm, target[12:18])
            if not last:
                err.add("law_integral_m_s", runner.layer.law.integral, g("law_integral_after", t))
                err.add("law_force_w_n", runner.layer.force_w, g("law_command_w_after", t))
                err.add("law_force_b_n", runner.layer.force_b, g("force_cmd_after", t))
        final = alive[-1]
        if grasp_sim is not None:
            # Where the tool was against the handle's placement (in the goal sphere of the base then) when the environment
            # grasped: the gate's distance had UniFP's goal stayed on the handle.
            place = mech_layer.sphere_to_world(g("handle_sphere", grasp_sim), g("root_pos", grasp_sim),
                                               g("root_quat_w", grasp_sim), runner.layer.centre_offset)
            ep["tool_to_placement_at_grasp_m"] = round(float(np.linalg.norm(g("tip_pos", grasp_sim) - place)), 4)
        ep.update({"rule_grasp_step": next((ev["step"] for ev in runner.events if ev["event"] == "grasp"), None),
                   "goal_resampled_before_grasp_step": resampled, "compared_apart": moved,
                   "final_s_m": round(float(d["mech_s"][e, final]), 4), "travel_m": round(float(d["mech_travel"][e, final]), 4),
                   "ended_by": _ended_by(d, e, final),
                   "max_abs_err": {k: round(v, 9) for k, v in ep_err.err.items()}, "runner_events": runner.events})
        if dog and grasp_sim is not None:
            ep["dog_reading"] = dog_reading(d, e, alive, runner, meta, arm_idx, unitree, contact_n, goals_before)
        episodes.append(ep)
    frame_diffs = np.array(frame_diffs).reshape(-1, 76)
    resampled_frames = np.array(resampled_frames).reshape(-1, 76)
    out = {"dump": str(dump_path), "episodes": len(episodes), "steps": int(sum(ep["steps"] for ep in episodes)),
           "episodes_resampled_before_grasp": sum(bool(ep["compared_apart"]) for ep in episodes),
           "frame_max_abs_err": float(np.abs(frame_diffs).max()) if frame_diffs.size else 0.0,
           "frame_err_by_block": block_errors(frame_diffs),
           "frame_err_by_block_resampled_episodes": block_errors(resampled_frames),
           "action_scale": float(np.abs(d["action"][d["alive"] > 0.5]).max()),
           "rule_mismatched_steps": rule_mismatch,
           "grasp_gate": gate_summary(episodes, within, within_from, ran_within, ran_wait, ran_source, own_mismatch, margin,
                                      held_back),
           "max_abs_err": dict(sorted(err.err.items())), "by_episode": episodes}
    if dog:
        out["dog_reading_summary"] = summarise_dog(episodes)
    return out


def gate_summary(episodes: list[dict], within, within_from, ran_within, ran_wait, ran_source, own_mismatch: dict,
                 margin, held_back: int) -> dict:
    """The arrival gate: the radius the layer ran and the rollout's; the steps the settle rule allowed and the gate held
    back (the tool outside the radius); the rule's mismatches with the layer's own goal in place of the environment's
    distance; how near the threshold the closest call came; and the environment's grasps that closed after UniFP's goal
    had moved on, with how far the tool was from the handle's placement at them."""
    grasped = [ep for ep in episodes if ep["grasp_step_isaac"] is not None]
    moved = [ep for ep in grasped if ep["compared_apart"]]
    held = [ep for ep in grasped if not ep["compared_apart"]]
    return {"grasp_within_m": within, "from": within_from,
            "rollout": {"grasp_within_m": ran_within, "max_wait_steps": ran_wait, "source": ran_source},
            "settled_steps_held_back": held_back if within is not None else None,
            "rule_mismatched_steps_own_goal": own_mismatch,
            "settled_steps_min_gap_to_radius_m": None if margin is None else round(margin, 9),
            "grasps": len(grasped), "grasps_after_goal_moved_on": len(moved),
            "tool_to_placement_at_those_grasps_m": sorted(ep["tool_to_placement_at_grasp_m"] for ep in moved),
            "tool_to_placement_at_other_grasps_max_m": max((ep["tool_to_placement_at_grasp_m"] for ep in held),
                                                           default=None)}


def _ended_by(d: dict, e: int, t: int) -> str:
    if d["ended"][e, t] < 0.5:
        return "running"
    for name in ("torn", "fallen", "tilt"):
        if f"term_{name}" in d and d[f"term_{name}"][e, t] > 0.5:
            return name
    return "time"


def dog_reading(d: dict, e: int, alive: np.ndarray, runner: mech_layer.MechRunner, meta: dict, arm_idx, unitree,
                contact_n: float, goals_before: dict | None = None) -> dict:
    """The travel the dog would read on this episode's grasped steps, from each of `READINGS`, against the true `s`; the
    law open loop on each; and the grasp's rule on the reports' tool speed and, with the arrival gate, the reports' hand
    (`fk_reported_odometry`'s point) within the radius of the layer's goal (`goals_before`: the layer's goal each step
    read, placed by the odometry's base)."""
    kin, offset = runner.kin, runner.tool_offset
    g = lambda key, t: np.asarray(d[key][e, t], dtype=float)
    odo = base_odometry.LegOdometry(meta)
    contact = lambda t: d["feet_fz"][e, t][TO_UNITREE_FEET] > contact_n
    odo.reset(g("root_pos", 0), g("root_quat_w", 0), g("joint_pos", 0)[unitree], contact(0))
    base_est = {0: g("root_pos", 0)}
    for t in alive[1:]:
        base_est[int(t)] = odo.step(g("root_quat_w", t), g("joint_pos", t)[unitree], contact(t))
    law = json.loads(str(d["force_law"]))
    laws = {name: mech_layer.ForceLaw(law["kp"], law["ki"], law["max"], law.get("bleed_s"),
                                      meta["mechanism"]["law"]["evaluation"]["arrived_m"], float(d["policy_dt"]))
            for name in ("true",) + READINGS}
    diffs = {name: [] for name in READINGS}
    offs = {name: [] for name in READINGS}
    forces = {name: [] for name in ("true",) + READINGS}
    stretch, base_err, fk_tip = [], [], []
    for t in alive:
        held = bool(d["mech_grasped"][e, t] > 0.5)
        s_true = float(d["mech_s"][e, t])
        reading = {"true": s_true}
        if held:
            path = path_at(d, e, t)
            q_true = g("joint_pos", t)[arm_idx]
            points = {"tip": g("tip_pos", t),
                      "fk_true": mech_layer.hand_point(kin, q_true, g("root_pos", t), g("root_quat_w", t), offset),
                      "fk_reported": mech_layer.hand_point(kin, g("arm_reported", t), g("root_pos", t),
                                                           g("root_quat_w", t), offset),
                      "fk_reported_odometry": mech_layer.hand_point(kin, g("arm_reported", t), base_est[int(t)],
                                                                    g("root_quat_w", t), offset)}
            for name, p in points.items():
                s, off, _ = path.project(p)
                reading[name] = s
                diffs[name].append(s - s_true)
                offs[name].append(off)
            handle = path.position(s_true)
            stretch.append(float((g("tip_pos", t) - handle) @ path.tangent(s_true)))
            base_err.append(float(np.linalg.norm(base_est[int(t)] - g("root_pos", t))))
            fk_tip.append(float(np.linalg.norm(points["fk_true"] - g("tip_pos", t))))
        for name, lw in laws.items():
            forces[name].append(float(lw.step(g("tg_ref", t), reading.get(name, s_true), held)))
    true_force = np.array(forces["true"])
    out = {"steps": len(stretch),
           "law_true_vs_isaac_n": float(np.abs(true_force[:-1] - _isaac_force(d, e, alive)[:-1]).max())
           if len(alive) > 1 else 0.0,
           "stretch_along_path_m": _stats(np.array(stretch)), "odometry_base_err_m": _stats(np.array(base_err)),
           "fk_true_vs_tip_m": _stats(np.array(fk_tip))}
    for name in READINGS:
        f = np.array(forces[name])
        out[name] = {"s_minus_true_m": _stats(np.array(diffs[name]), signed=True), "off_path_m": _stats(np.array(offs[name])),
                     "law_force_diff_n": _stats(f - true_force, signed=True)}
    # The grasp's rule on the reports' tool speed: the first step it would fire, against the environment's grasp.
    engage = int(d["engage_step"][e, 0])
    est_step = None
    within = runner.layer.grasp_within
    for t in alive[1:]:
        waited = int(d["since_reset"][e, t]) - engage
        speed = mech_layer.tool_speed_estimate(kin, g("arm_reported", t), g("arm_rate", t), g("root_quat_w", t), offset)
        due = waited >= 0 and (F32(speed) < F32(meta["mechanism"]["approach"]["settle_speed_m_s"])
                               or waited >= int(meta["mechanism"]["approach"]["max_wait_steps"]))
        if due and within is not None:
            base = base_est[int(t)]
            hand = mech_layer.hand_point(kin, g("arm_reported", t), base, g("root_quat_w", t), offset)
            goal = mech_layer.sphere_to_world(goals_before[int(t)], base, g("root_quat_w", t), runner.layer.centre_offset)
            due = bool(F32(np.linalg.norm(hand - goal)) <= within)
        if due:
            est_step = int(t)
            break
    grasped = np.flatnonzero(d["mech_grasped"][e] > 0.5)
    out["grasp_step_by_reported_speed"] = est_step
    out["grasp_step_isaac"] = int(grasped[0]) if len(grasped) else None
    return out


def _isaac_force(d: dict, e: int, alive: np.ndarray) -> np.ndarray:
    """The environment's law output along the tangent on each alive step (`law_command_w_after` . tangent)."""
    w = d["law_command_w_after"][e, alive].astype(float)
    tangent = d["mech_tangent"][e, alive].astype(float)
    return (w * tangent).sum(-1)


def _stats(x: np.ndarray, signed: bool = False) -> dict | None:
    if not len(x):
        return None
    a = np.abs(x)
    out = {"median_abs": round(float(np.median(a)), 6), "p95_abs": round(float(np.percentile(a, 95)), 6),
           "max_abs": round(float(a.max()), 6)}
    if signed:
        out["median"] = round(float(np.median(x)), 6)
    return out


def summarise_dog(episodes: list[dict]) -> dict:
    """Per kind, the worst of each episode's reading statistics (median of medians, max of maxes)."""
    out = {}
    for kind in sorted({ep["kind"] for ep in episodes if "dog_reading" in ep}):
        eps = [ep["dog_reading"] for ep in episodes if ep["kind"] == kind and "dog_reading" in ep]
        row = {"episodes": len(eps)}
        for name in READINGS:
            s = [r[name]["s_minus_true_m"] for r in eps if r[name]["s_minus_true_m"]]
            f = [r[name]["law_force_diff_n"] for r in eps if r[name]["law_force_diff_n"]]
            row[name] = {"s_minus_true_median_m": round(float(np.median([x["median"] for x in s])), 5) if s else None,
                         "s_minus_true_max_abs_m": round(float(max(x["max_abs"] for x in s)), 5) if s else None,
                         "force_diff_median_n": round(float(np.median([x["median"] for x in f])), 3) if f else None,
                         "force_diff_max_abs_n": round(float(max(x["max_abs"] for x in f)), 3) if f else None}
        row["grasp_step_by_reported_speed_minus_isaac"] = [
            (r["grasp_step_by_reported_speed"] - r["grasp_step_isaac"])
            if r["grasp_step_by_reported_speed"] is not None and r["grasp_step_isaac"] is not None else None for r in eps]
        out[kind] = row
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", required=True, help="the exported .npz (export_policy.py --task mechanism)")
    parser.add_argument("--sim", required=True, help="a mech_eval --mech_shadow_dump run's mech_shadow_dump.npz")
    parser.add_argument("--out", help="write the report here as json")
    parser.add_argument("--brief", action="store_true", help="print the summary without the per-episode logs")
    parser.add_argument("--no_dog", action="store_true", help="parity only, no dog's reading")
    parser.add_argument("--contact_n", type=float, default=10.0, help="a foot in contact above this force, N (odometry)")
    parser.add_argument("--grasp_within_m", type=float, default=None,
                        help="the grasp's arrival gate the layer runs, m (S4c's --mech_grasp_within_m), for an export "
                             "made before the exporter recorded it; default the export's (approach.grasp_within_m); "
                             "a negative value turns it off")
    args = parser.parse_args()
    policy = leg_policy.LegPolicy(args.policy)
    report = {"policy": args.policy, "policy_meta": {k: policy.meta[k] for k in ("task", "variant", "checkpoint",
                                                                                  "checkpoint_iteration")},
              "sim": check_sim(policy, Path(args.sim), dog=not args.no_dog, contact_n=args.contact_n,
                               grasp_within_m="export" if args.grasp_within_m is None
                               else (None if args.grasp_within_m < 0 else args.grasp_within_m))}
    text = json.dumps(report, indent=2)
    if args.out:
        Path(args.out).write_text(text)
    if args.brief:
        sim = dict(report["sim"])
        sim["by_episode"] = [{k: v for k, v in ep.items() if k not in ("runner_events", "dog_reading", "max_abs_err")}
                             for ep in sim["by_episode"]]
        text = json.dumps({**report, "sim": sim}, indent=2)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

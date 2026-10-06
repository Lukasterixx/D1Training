#!/usr/bin/env python3
"""Measure the walking gait of the UniFP Go2+D1 policy (Isaac Lab port) under held velocity commands.

    ./demos/unifp/gait_probe.py --headless
    ./dashboard.py record logs/unifp_gait_probe/<run> --title "..."

Written 2026-09-27 to give P2Dingo's Go2 walking policy a target: the numbers the UniFP gait walks with
(stance width, foot placement against the thighs, stride timing, swing height, ride height). The welded USD
is built at the LEGACY mount the checkpoint was trained on, into the run directory; the run directory holds
`gait.json` and a `run.json` for the record.

Per held command (stand, walk 0.3, walk 0.6, turn 0.5): reset all envs, settle, then record the robot
at the PHYSICS rate (200 Hz, sampled in `_apply_action`) and report foot placement, hip abduction, base
height/attitude, contact timing and achieved velocity.

Frames: "yaw frame" = origin at the base link, x = heading, y = left, z = world up (roll/pitch removed).
Sign folds: side = +1 for left legs (FL, RL), -1 for right legs; "outward" quantities are multiplied by it.
Hip abduction: Go2 hip joints rotate about +x, so +q moves the foot to +y; outward angle = side * q.
"""
from __future__ import annotations

import sys

sys.dont_write_bytecode = True   # do not drop __pycache__ into the repo

import argparse
import json
import math
import os
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

COMMANDS = (
    ("stand", (0.0, 0.0, 0.0)),
    ("walk_0.3", (0.3, 0.0, 0.0)),
    ("walk_0.6", (0.6, 0.0, 0.0)),
    ("turn_0.5", (0.0, 0.0, 0.5)),
)
LEGS = ("FL", "FR", "RL", "RR")
SIDE = (1.0, -1.0, 1.0, -1.0)

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--checkpoint", default=str(REPO / "checkpoints/unifp_go2d1_isaaclab_model_56000.pt"))
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--settle_s", type=float, default=2.0)
parser.add_argument("--measure_s", type=float, default=8.0)
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--contact_n", type=float, default=5.0, help="foot contact = vertical force above this (the env's own 5 N)")
parser.add_argument("--min_run_s", type=float, default=0.015, help="debounce: contact/swing runs shorter than this are merged")
parser.add_argument("--mount_pos", type=float, nargs=3, default=None,
                    help="arm weld offset, m (default mount.LEGACY_ARM_MOUNT_POS_M, what model_56000 was trained on)")
parser.add_argument("--output", default=str(REPO / "logs/unifp_gait_probe"),
                    help="parent of the run directory (<UTC stamp>_gait_probe/)")

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

sys.path.insert(0, str(REPO))
import numpy as np  # noqa: E402
import torch  # noqa: E402
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # noqa: E402

from mount import LEGACY_ARM_MOUNT_POS_M  # noqa: E402
from weld import build_welded_robot_usd  # noqa: E402
from unifp_isaaclab import interface, robot as robot_mod  # noqa: E402
from unifp_train import gait  # noqa: E402
from unifp_train.env import Go2D1PosForceEnv  # noqa: E402
from unifp_train.env_cfg import Go2D1PosForceEnvCfg  # noqa: E402
from unifp_train.eval import EvaluablePolicy  # noqa: E402


def main() -> int:
    mount_pos = tuple(args.mount_pos) if args.mount_pos is not None else tuple(LEGACY_ARM_MOUNT_POS_M)
    started = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
    run_dir = Path(args.output) / f"{started}_gait_probe"
    run_dir.mkdir(parents=True, exist_ok=True)
    usd_dir = run_dir / "usd"
    robot_usd = build_welded_robot_usd(
        go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
        d1_urdf_path=str(REPO / "d1_arm/d1.urdf"),
        out_usd_path=str(usd_dir / "go2_d1_probe.usd"),
        mount_pos=mount_pos, arm_mass_kg=3.152, dock_visual=False,
    ).usd_path

    cfg = Go2D1PosForceEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.seed = args.seed
    cfg.roll_objective = False
    cfg.tool_body, cfg.tool_offset_m = interface.TOOL_BODY, interface.TOOL_OFFSET_M   # what 56000 trained on
    cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot")  # armature 2e-4, as trained
    cfg.robot_usd = robot_usd
    cfg.force_start_step = 10 ** 12        # no gripper pushes at all
    env = Go2D1PosForceEnv(cfg)
    dev = env.device
    n = env.num_envs
    robot = env._robot

    policy = EvaluablePolicy(args.checkpoint, device=str(dev))

    # --- held velocity command (replaces the sampler on the timer and on reset, as `eval --hold_command`) ---
    held = torch.zeros(3, device=dev)

    def hold(env_ids):
        env._commands[env_ids, :3] = held

    env._resample_velocity_commands = hold

    # --- arm goal: the task's opening sweep to its home (r 0.50 m, pitch 0, yaw 0), done in 1 s, never resampled ---
    env._goals.traj_steps[:] = 1.0 / interface.POLICY_DT
    env._goals.total_steps[:] = float("inf")

    # --- no resets mid-run; falls recorded (as unifp_train.eval.run_episodes) ---
    fell = torch.zeros(n, dtype=torch.bool, device=dev)
    original_dones = env._get_dones

    def dones_without_reset():
        terminated, truncated = original_dones()
        fell.logical_or_(terminated)
        return torch.zeros_like(terminated), torch.zeros_like(truncated)

    env._get_dones = dones_without_reset

    # --- bodies ---
    names = list(robot.body_names)
    feet = [names.index(f"{leg}_foot") for leg in LEGS]
    thighs = [names.index(f"{leg}_thigh") for leg in LEGS]
    feet_c = env._feet_contacts
    hip_joint_ids = env._order[[0, 3, 6, 9]]
    leg_joint_ids = env._order[:12]
    origins = env.scene.env_origins

    # --- recording at the physics rate: `_apply_action` runs once per 5 ms substep, after the previous update ---
    rec: list | None = None
    original_apply = env._apply_action

    def apply_and_record():
        if rec is not None:
            rec.append({
                "root": (robot.data.root_pos_w - origins).clone(),
                "quat": robot.data.root_quat_w.clone(),
                "lin_w": robot.data.root_lin_vel_w.clone(),
                "ang_w": robot.data.root_ang_vel_w.clone(),
                "feet": (robot.data.body_pos_w[:, feet] - origins.unsqueeze(1)).clone(),
                "thigh": (robot.data.body_pos_w[:, thighs] - origins.unsqueeze(1)).clone(),
                "fz": env._contact_sensor.data.net_forces_w[:, feet_c, 2].clone(),
                "q_hip": robot.data.joint_pos[:, hip_joint_ids].clone(),
                "q_leg": robot.data.joint_pos[:, leg_joint_ids].clone(),
                "phase": env._gait_phase.clone(),
            })
        original_apply()

    env._apply_action = apply_and_record

    settle_steps = round(args.settle_s / interface.POLICY_DT)
    measure_steps = round(args.measure_s / interface.POLICY_DT)
    dt = interface.SIM_DT
    results = {}
    t0 = time.time()
    for label, command in COMMANDS:
        held[:] = torch.tensor(command, device=dev)
        torch.manual_seed(args.seed)
        fell.zero_()
        rec = None
        obs, _ = env.reset()
        env.episode_length_buf[:] = 0
        with torch.no_grad():
            for step in range(settle_steps + measure_steps):
                if step == settle_steps:
                    rec = []
                obs, _, _, _, _ = env.step(policy.act(obs["policy"]))
        data = {k: torch.stack([r[k] for r in rec]) for k in rec[0]}     # (T, N, ...)
        rec = None
        cmd_seen = env._commands[:, :3].mean(0).tolist()
        results[label] = analyse(label, command, cmd_seen, data, fell.cpu().numpy(), dt)
        print(f"[probe] {label}: done ({time.time() - t0:.0f} s), falls {int(fell.sum())}/{n}", flush=True)

    report = {
        "checkpoint": os.path.abspath(args.checkpoint),
        "policy_source": policy.source, "policy_iteration": policy.iteration,
        "num_envs": n, "settle_s": args.settle_s, "measure_s": args.measure_s, "seed": args.seed,
        "sample_rate_hz": 1.0 / dt, "contact_threshold_n": args.contact_n,
        "mount_pos_m": list(mount_pos), "robot_usd": robot_usd,
        "arm_goal": "opening sweep to (r 0.50 m, pitch 0, yaw 0) in 1 s, then held; no external forces",
        "gait_clock_period_s": interface.CYCLE_TIME,
        "results": results,
    }
    (run_dir / "gait.json").write_text(json.dumps(report, indent=2) + "\n")
    commit = os.popen(f"git -C {REPO} rev-parse HEAD 2>/dev/null").read().strip() or None
    dirty = os.popen(f"git -C {REPO} status --porcelain 2>/dev/null").read().strip()
    (run_dir / "run.json").write_text(json.dumps({
        "mode": "unifp_gait_probe", "status": "finished", "seed": args.seed, "error": None,
        "started_utc": started, "git_commit": commit, "git_status": dirty,
        "arguments": {"num_envs": n, "checkpoint": os.path.abspath(args.checkpoint),
                      "settle_s": args.settle_s, "measure_s": args.measure_s, "mount_pos": list(mount_pos)},
    }, indent=2) + "\n")
    print_tables(report)
    print(f"[probe] wrote {run_dir}", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------------------

def runs(sig: np.ndarray):
    """Run-length encode a 1-D bool array: [(value, start, length)]."""
    if len(sig) == 0:
        return []
    change = np.flatnonzero(np.diff(sig.astype(np.int8))) + 1
    starts = np.concatenate(([0], change))
    ends = np.concatenate((change, [len(sig)]))
    return [(bool(sig[s]), int(s), int(e - s)) for s, e in zip(starts, ends)]


def debounce(sig: np.ndarray, min_len: int) -> np.ndarray:
    """Fill interior swing gaps shorter than min_len, then drop interior contact blips shorter than it."""
    sig = sig.copy()
    for val in (False, True):
        for v, s, length in runs(sig):
            if v == val and length < min_len and s > 0 and s + length < len(sig):
                sig[s:s + length] = not val
    return sig


def to_np(t: torch.Tensor) -> np.ndarray:
    return t.detach().cpu().double().numpy()


def analyse(label, command, cmd_seen, d, fell, dt) -> dict:
    T, N = d["root"].shape[:2]
    yaw = interface.yaw_from_quat(d["quat"])                              # (T, N)
    yq = interface.yaw_quat(yaw)                                          # (T, N, 4)
    yq4 = yq.unsqueeze(2).expand(T, N, 4, 4)
    root = d["root"].unsqueeze(2)
    feet_y = interface.quat_rotate_inverse(yq4, d["feet"] - root)         # yaw frame (T, N, 4, 3)
    thigh_y = interface.quat_rotate_inverse(yq4, d["thigh"] - root)
    q4 = d["quat"].unsqueeze(2).expand(T, N, 4, 4)
    feet_b = interface.quat_rotate_inverse(q4, d["feet"] - root)          # full body frame
    lin_yaw = interface.quat_rotate_inverse(yq, d["lin_w"])
    rp = interface.body_roll_pitch(d["quat"])

    feet_y, thigh_y, feet_b = map(to_np, (feet_y, thigh_y, feet_b))
    lin_yaw, rp = to_np(lin_yaw), to_np(rp)
    ang_z = to_np(d["ang_w"][..., 2])
    base_z = to_np(d["root"][..., 2])
    feet_z = to_np(d["feet"][..., 2])
    fz = to_np(d["fz"])
    q_hip = to_np(d["q_hip"])
    phase = d["phase"]
    clock_stance = to_np(gait.stance_mask(phase.reshape(-1)).reshape(T, N, 4)).astype(bool)

    ok = ~fell
    side = np.array(SIDE)
    min_len = max(1, round(args.min_run_s / dt))

    contact = np.zeros((T, N, 4), dtype=bool)
    for e in range(N):
        for f in range(4):
            contact[:, e, f] = debounce(fz[:, e, f] > args.contact_n, min_len)
    raw_contact = fz > args.contact_n

    def stance_mean(x):   # x (T, N, 4) -> per-env per-foot mean over stance samples (N, 4)
        w = contact.astype(float)
        with np.errstate(invalid="ignore", divide="ignore"):
            return (x * w).sum(0) / w.sum(0)

    def agg(per_env):     # (N,) or (N, k) -> mean and sd across non-fallen envs
        v = per_env[ok]
        return np.nanmean(v, 0), np.nanstd(v, 0)

    out = {"command": list(command), "command_in_env": cmd_seen, "falls": int(fell.sum()),
           "envs_used": int(ok.sum()), "samples_per_env": T}

    # 1. lateral
    abs_y = stance_mean(np.abs(feet_y[..., 1]))
    signed_y = stance_mean(feet_y[..., 1])
    abs_y_body = stance_mean(np.abs(feet_b[..., 1]))
    rel_thigh_out = stance_mean((feet_y[..., 1] - thigh_y[..., 1]) * side)
    thigh_out = stance_mean(thigh_y[..., 1] * side)
    width_front = signed_y[:, 0] - signed_y[:, 1]
    width_rear = signed_y[:, 2] - signed_y[:, 3]
    # 2. fore-aft
    x_rel_thigh = stance_mean(feet_y[..., 0] - thigh_y[..., 0])
    foot_x = stance_mean(feet_y[..., 0])
    thigh_x = stance_mean(thigh_y[..., 0])
    spread_x = foot_x[:, :2].mean(1) - foot_x[:, 2:].mean(1)
    # 3. hip abduction (outward +), all samples and stance only
    abd = np.degrees(q_hip * side)
    abd_all = abd.mean(0)
    abd_stance = stance_mean(abd)
    abd_sd = abd.std(0)

    # 5. contact timing, touchdown/liftoff placement, swing apex
    duty = contact.mean(0)
    clock_agree = (contact == clock_stance).mean(0)
    per_foot_freq, per_foot_rate, per_foot_apex, per_foot_td, per_foot_lo, per_foot_swing_s = ([] for _ in range(6))
    per_foot_td_y, raw_td_rate = [], []
    stance_z = stance_mean(feet_z)
    for f in range(4):
        periods, apexes, tds, los, tdy, swings = [], [], [], [], [], []
        n_td, n_raw = 0, 0
        for e in np.flatnonzero(ok):
            r = runs(contact[:, e, f])
            n_raw += len([1 for v, s, _ in runs(raw_contact[:, e, f]) if v and s > 0])
            td_idx = [s for v, s, _ in r if v and s > 0]
            n_td += len(td_idx)
            periods += list(np.diff(td_idx) * dt)
            for s in td_idx:
                tds.append(feet_y[s, e, f, 0] - thigh_y[s, e, f, 0])
                tdy.append((feet_y[s, e, f, 1] - thigh_y[s, e, f, 1]) * side[f])
            for v, s, length in r:
                if v and s + length < T:
                    k = s + length - 1
                    los.append(feet_y[k, e, f, 0] - thigh_y[k, e, f, 0])
                if (not v) and s > 0 and s + length < T:
                    apexes.append(feet_z[s:s + length, e, f].max() - stance_z[e, f])
                    swings.append(length * dt)
        per_foot_freq.append(1.0 / np.median(periods) if periods else float("nan"))
        per_foot_rate.append(n_td / (ok.sum() * T * dt))
        raw_td_rate.append(n_raw / (ok.sum() * T * dt))
        per_foot_apex.append(float(np.mean(apexes)) if apexes else float("nan"))
        per_foot_td.append(float(np.mean(tds)) if tds else float("nan"))
        per_foot_lo.append(float(np.mean(los)) if los else float("nan"))
        per_foot_td_y.append(float(np.mean(tdy)) if tdy else float("nan"))
        per_foot_swing_s.append(float(np.mean(swings)) if swings else float("nan"))

    cm = lambda a: [round(100 * float(x), 2) for x in np.atleast_1d(a)]
    r2 = lambda a: [round(float(x), 3) for x in np.atleast_1d(a)]
    m = lambda per_env: agg(per_env)[0]
    s = lambda per_env: agg(per_env)[1]

    out["per_foot"] = {
        "legs": list(LEGS),
        "stance_abs_y_yawframe_cm": cm(m(abs_y)),
        "stance_abs_y_yawframe_sd_over_envs_cm": cm(s(abs_y)),
        "stance_signed_y_yawframe_cm": cm(m(signed_y)),
        "stance_abs_y_bodyframe_cm": cm(m(abs_y_body)),
        "stance_y_outward_of_thigh_cm": cm(m(rel_thigh_out)),
        "thigh_origin_outward_y_cm": cm(m(thigh_out)),
        "stance_x_rel_thigh_cm": cm(m(x_rel_thigh)),
        "stance_x_yawframe_cm": cm(m(foot_x)),
        "thigh_origin_x_cm": cm(m(thigh_x)),
        "touchdown_x_rel_thigh_cm": cm(per_foot_td),
        "liftoff_x_rel_thigh_cm": cm(per_foot_lo),
        "touchdown_y_outward_of_thigh_cm": cm(per_foot_td_y),
        "hip_abduction_outward_deg_all": r2(m(abd_all)),
        "hip_abduction_outward_deg_stance": r2(m(abd_stance)),
        "hip_abduction_sd_within_env_deg": r2(m(abd_sd)),
        "duty_factor": r2(m(duty)),
        "stride_freq_hz_median_td_interval": r2(per_foot_freq),
        "touchdowns_per_s": r2(per_foot_rate),
        "raw_undebounced_touchdowns_per_s": r2(raw_td_rate),
        "mean_swing_duration_s": r2(per_foot_swing_s),
        "swing_apex_above_stance_cm": cm(per_foot_apex),
        "stance_foot_origin_z_cm": cm(m(stance_z)),
        "contact_matches_gait_clock_fraction": r2(m(clock_agree)),
    }
    out["stance_width_cm"] = {
        "front_FL_minus_FR": cm(m(width_front))[0], "front_sd_over_envs": cm(s(width_front))[0],
        "rear_RL_minus_RR": cm(m(width_rear))[0], "rear_sd_over_envs": cm(s(width_rear))[0]}
    out["front_to_rear_foot_x_spread_cm"] = cm(m(spread_x))[0]
    out["base"] = {
        "height_m_mean": round(float(base_z[:, ok].mean()), 4),
        "height_m_sd": round(float(base_z[:, ok].std()), 4),
        "roll_deg_mean": round(math.degrees(float(rp[:, ok, 0].mean())), 2),
        "roll_deg_sd": round(math.degrees(float(rp[:, ok, 0].std())), 2),
        "pitch_deg_mean": round(math.degrees(float(rp[:, ok, 1].mean())), 2),
        "pitch_deg_sd": round(math.degrees(float(rp[:, ok, 1].std())), 2),
    }
    out["velocity"] = {
        "cmd_vx_vy_wz": list(command),
        "achieved_vx_m_s": round(float(lin_yaw[:, ok, 0].mean()), 3),
        "achieved_vy_m_s": round(float(lin_yaw[:, ok, 1].mean()), 3),
        "achieved_wz_rad_s": round(float(ang_z[:, ok].mean()), 3),
        "vx_sd_over_envs": round(float(lin_yaw[:, ok, 0].mean(0).std()), 3),
        "wz_sd_over_envs": round(float(ang_z[:, ok].mean(0).std()), 3),
    }
    return out


def print_tables(report):
    for label, r in report["results"].items():
        pf = r["per_foot"]
        print(f"\n=== {label}  cmd {r['command']}  falls {r['falls']}/{report['num_envs']} ===")
        rows = (("|y| stance, yaw frame (cm)", "stance_abs_y_yawframe_cm"),
                ("|y| stance, body frame (cm)", "stance_abs_y_bodyframe_cm"),
                ("y outward of thigh, stance (cm)", "stance_y_outward_of_thigh_cm"),
                ("thigh origin outward y (cm)", "thigh_origin_outward_y_cm"),
                ("x - thigh x, stance mean (cm)", "stance_x_rel_thigh_cm"),
                ("x - thigh x, touchdown (cm)", "touchdown_x_rel_thigh_cm"),
                ("x - thigh x, liftoff (cm)", "liftoff_x_rel_thigh_cm"),
                ("hip abd outward, all (deg)", "hip_abduction_outward_deg_all"),
                ("hip abd outward, stance (deg)", "hip_abduction_outward_deg_stance"),
                ("duty factor", "duty_factor"),
                ("stride freq, median TD int (Hz)", "stride_freq_hz_median_td_interval"),
                ("touchdowns per s", "touchdowns_per_s"),
                ("swing apex above stance (cm)", "swing_apex_above_stance_cm"),
                ("contact == gait clock", "contact_matches_gait_clock_fraction"))
        print(f"{'':34s}" + "".join(f"{leg:>9s}" for leg in pf["legs"]))
        for name, key in rows:
            print(f"{name:34s}" + "".join(f"{v:9.2f}" for v in pf[key]))
        print(f"stance width front/rear (cm): {r['stance_width_cm']}")
        print(f"front-rear foot x spread (cm): {r['front_to_rear_foot_x_spread_cm']}")
        print(f"base: {r['base']}")
        print(f"velocity: {r['velocity']}")


if __name__ == "__main__":
    status = 1
    try:
        status = main()
    except Exception:
        import traceback

        traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)   # Isaac Sim 5.1 hangs in teardown (run_unifp_train.py does the same)

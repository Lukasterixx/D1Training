#!/usr/bin/env python3
"""The key turned by the arm: find the box with the head RealSense and the MID-360, walk to the push's stance, and push
Lukas's lever key from 45 degrees up to 45 down with the D1's fingers, scripted (`arm_turn`, `arm_env`). The key starts
in the lock; the legs stay with the walking policy.

    ./demos/combiner_key/run_arm_turn.py                                   # watch one, pictures in frames/
    ./demos/combiner_key/run_arm_turn.py --lock_n 10                       # a stiffer lock (10 N where the fingers push)
    ./demos/combiner_key/run_arm_turn.py --attempts 8 --headless --no_overview

Writes `logs/combiner_key_arm/<stamp>_lock<N>_seed<S>[_<name>]/` (`run.json`, `summary.json`, `trace.csv`, `frames/`,
`push.gif`); then `python demos/combiner_key/plot_arm_turn.py <dir>` and `./dashboard.py record <dir> --title "..."`.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--attempts", type=int, default=1)
    p.add_argument("--num_envs", type=int, default=None, help="Default: the attempts when headless, else 1.")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--settle_s", type=float, default=4.0, help="The spawn's settle before the look.")
    p.add_argument("--lock_n", type=float, default=5.0,
                   help="The lock's resistance as a push where the fingers push (arm_turn.PUSH_REACH_M), N: Lukas's 5 N.")
    p.add_argument("--lock_friction_nm", type=float, default=None, help="The lock's sticking friction, N·m (overrides "
                                                                         "--lock_n).")
    p.add_argument("--lock_detent_nm", type=float, default=0.0, help="A cam detent before the unlock, N·m (none felt).")
    p.add_argument("--walker", default="p2dingo_gait")
    p.add_argument("--given_box", action="store_true",
                   help="No head camera or lidar: the look is handed the box's true placement. Not the demo.")
    p.add_argument("--no_overview", action="store_true", help="No overview camera (no frames): for batches.")
    p.add_argument("--run_name", default="")
    return p


def git(*args) -> str:
    try:
        return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception:  # noqa: BLE001
        return ""


def main() -> int:
    parser = build_parser()
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.num_envs = args.num_envs or (args.attempts if args.headless else 1)
    args.enable_cameras = not (args.given_box and args.no_overview)
    app = AppLauncher(args).app

    import numpy as np
    import torch

    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
    from weld import build_welded_robot_usd
    from mount import ARM_MOUNT_POS_M
    from unifp_isaaclab import interface
    from demos.combiner.asset import build_combiner_usd
    from demos.combiner.geometry import DEFAULT_HANDLE_TORQUE_NM, GEOMETRY
    from demos.unifp import decoupled, env as demo_env, props, script as demo_script
    from demos.combiner_key import arm_env, arm_turn as at, env as key_env, geometry as kg
    from unifp_train import leg_cfg

    friction = args.lock_friction_nm if args.lock_friction_nm is not None else at.lock_torque_nm(args.lock_n)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    name = f"{stamp}_lock{friction:.2f}nm_seed{args.seed}" + (f"_{args.run_name}" if args.run_name else "")
    out_dir = ROOT / "logs/combiner_key_arm" / name
    (out_dir / "frames").mkdir(parents=True, exist_ok=True)
    early = {"mode": "combiner_key_arm_turn", "status": "running", "seed": args.seed,
             "arguments": {k: v for k, v in vars(args).items() if not k.startswith("_")},
             "command": " ".join(sys.argv), "git_commit": git("rev-parse", "HEAD"), "git_status": git("status", "--short")}
    (out_dir / "run.json").write_text(json.dumps(early, indent=2))

    # The combiner door demo's robot (as the key demo's): the payload drawn, not simulated.
    robot_usd = build_welded_robot_usd(
        go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd", d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
        out_usd_path=str(ROOT / "generated/go2_d1.usd"), mount_pos=ARM_MOUNT_POS_M, arm_mass_kg=3.152).usd_path
    phases = arm_env.arm_phases()
    total_s = sum(p.duration_s for p in phases)

    cfg = demo_env.UniFPDemoEnvCfg()
    cfg.task = "combiner"
    cfg.seed = args.seed
    cfg.scene.num_envs = args.num_envs
    cfg.scene.env_spacing = 3.0
    cfg.robot = decoupled.make_robot_cfg(robot_usd)
    px = cfg.sim.physx
    px.gpu_max_rigid_contact_count, px.gpu_max_rigid_patch_count = 2**16, 2**14
    px.gpu_found_lost_pairs_capacity, px.gpu_found_lost_aggregate_pairs_capacity = 2**16, 2**18
    px.gpu_total_aggregate_pairs_capacity, px.gpu_collision_stack_size = 2**16, 2**24
    px.gpu_heap_capacity, px.gpu_temp_buffer_capacity = 2**24, 2**22
    cfg.walker = args.walker
    cfg.robot_usd = robot_usd
    torque = DEFAULT_HANDLE_TORQUE_NM
    cfg.handle_torque_nm, cfg.door_torque_nm = torque, 0.0
    cfg.episode_length_s = args.settle_s + total_s + 5.0
    import demos.combiner.asset as box_asset

    built_from = json.dumps(dataclasses.asdict(GEOMETRY), sort_keys=True).encode() + Path(box_asset.__file__).read_bytes()
    box_usd = ROOT / "generated/unifp_demos" / f"combiner_{torque:.3f}_{hashlib.sha256(built_from).hexdigest()[:12]}.usda"
    if not box_usd.exists():
        build_combiner_usd(box_usd, handle_torque_nm=torque)
    cfg.box_usd = str(box_usd)
    cfg.post = demo_env.post_cfg()
    cfg.combiner = demo_env.combiner_cfg(cfg.box_usd, torque, 0.0)
    cfg.door_deg = 90.0
    cfg.given_box = args.given_box
    cfg.overview = not args.no_overview
    cfg.lock_friction_nm, cfg.lock_detent_nm = friction, args.lock_detent_nm
    if not args.headless:
        # From the robot's front left, onto the key's arc and the hand.
        cfg.viewer.origin_type = "env"
        cfg.viewer.env_index = 0
        cfg.viewer.eye = (0.10, 0.95, 0.80)
        cfg.viewer.lookat = (0.62, 0.15, 0.40)

    env = arm_env.ArmTurnEnv(cfg)
    env.attach_latches(locked=True)
    env.hide_object = False
    env.hide_furniture = True
    env.command_tip = False
    env.jaw_override = None
    env.jaw_compensation = True
    env.scripts = [demo_script.DemoScript(list(phases)) for _ in range(env.num_envs)]
    walker = decoupled.WalkingPolicy(decoupled.walker_checkpoint(env.walker), env.device)
    rng = np.random.default_rng(args.seed)

    def sample_site() -> props.BoxSite:
        """The box squarely ahead of the look, as the key demo places it."""
        ahead = kg.LOOK_DOOR_AHEAD_M + GEOMETRY.lock[0] + rng.uniform(-1, 1) * kg.PLACEMENT_AHEAD_JITTER_M
        side = rng.uniform(-1, 1) * kg.PLACEMENT_SIDE_JITTER_M
        yaw = 180.0 + rng.uniform(-1, 1) * kg.PLACEMENT_YAW_JITTER_DEG
        return props.BoxSite(distance_m=math.hypot(ahead, side), bearing_deg=math.degrees(math.atan2(side, ahead)),
                             yaw_deg=yaw)

    report = {**early, "robot_usd": robot_usd, "box_usd": str(box_usd), "phases": [p.name for p in phases],
              "push": {"reach_m": at.PUSH_REACH_M, "contact_off_door_m": at.CONTACT_OFF_DOOR_M,
                       "hand_pitch_deg": at.HAND_PITCH_DEG, "press_depth_m": at.PRESS_DEPTH_M,
                       "turn_deg": [at.START_DEG, at.END_DEG], "turn_s": at.turn_seconds(),
                       "stance": {"door_ahead_m": at.ARM_DOOR_AHEAD_M, "lock_left_m": at.LOCK_LEFT_OF_BASE_M}},
              "lock": {"plant": "unifp_train.handle.LeverHandle(pad=True) against a sphere at the fingertips' edge",
                       "friction_nm": friction, "as_push_n": round(friction / at.PUSH_REACH_M, 2),
                       "detent_nm": args.lock_detent_nm, "unlock_deg": at.UNLOCK_DEG,
                       "contact_k_n_m": arm_env.CONTACT_K_N_PER_M, "contact_mu": arm_env.CONTACT_MU,
                       "contact_radius_m": at.CONTACT_RADIUS_M},
              "robot": "the combiner door demo's: the lidar and the RealSenses drawn, not simulated (no wrist mass)"}
    attempts, rows, frames = [], [], []
    remaining, batch = args.attempts, 0
    status = "finished"
    steps = int(round((args.settle_s + total_s) / interface.POLICY_DT)) + 2
    frame_every = max(1, round(arm_env.FRAME_EVERY_S / interface.POLICY_DT))
    while remaining > 0 and app.is_running():
        count = min(remaining, env.num_envs)
        env.place([sample_site() for _ in range(env.num_envs)])
        for demo in env.scripts:
            demo.phases = [demo_script.Phase("spawn", args.settle_s, target=lambda st: demo_script.HOME_POINT,
                                             jaw_m=arm_env.JAW_SHUT_M)] + list(phases)
        env.reset()
        worst = {i: {"min_base_z": 9.0, "max_tilt_deg": 0.0} for i in range(env.num_envs)}
        push = {i: {"use_max": np.zeros(6), "steps": 0, "saturated_steps": 0, "turn_contact_n": [], "lag_mm": 0.0}
                for i in range(env.num_envs)}
        last_phase = [""] * env.num_envs
        with torch.no_grad():
            for step in range(steps):
                if not app.is_running():
                    status = "aborted"
                    break
                started = time.perf_counter()
                env.step(env.action_from_legs(walker.act(env.walking_observation())))
                wall_ms = 1000.0 * (time.perf_counter() - started)
                data = env._robot.data
                base = (data.root_pos_w - env.scene.env_origins).cpu().numpy()
                rp = interface.body_roll_pitch(data.root_quat_w).cpu().numpy()
                use = env.arm_torque_use().cpu().numpy()
                measured = env.leg.measured_deg().cpu().numpy()
                centre, _, _ = env._hand_frame()
                for i in range(count):
                    command = env.commands[i]
                    phase = command.phase if command is not None else ""
                    if phase != last_phase[i]:
                        last_phase[i] = phase
                        if env._overview is not None and phase == "unfold":
                            env.aim_overview()          # the box is placed again at the look's anchor
                    w = worst[i]
                    w["min_base_z"] = min(w["min_base_z"], float(base[i, 2]))
                    w["max_tilt_deg"] = max(w["max_tilt_deg"], math.degrees(float(np.abs(rp[i]).max())))
                    pushing = phase in arm_env.PUSH_PHASES
                    if pushing:
                        p = push[i]
                        p["use_max"] = np.maximum(p["use_max"], use[i])
                        p["steps"] += 1
                        p["saturated_steps"] += int(use[i].max() >= 0.99)
                        if phase == "turn":
                            p["turn_contact_n"].append(float(env.leg.force_w[i].norm()))
                    if pushing or step % 5 == 0:
                        active = bool(env.leg.active[i])
                        cmd_deg = (at.turn_angle(command.progress) if phase == "turn"
                                   else at.START_DEG if phase in ("arm_out", "arm_above", "arm_touch")
                                   else at.END_DEG if phase in ("arm_hold", "arm_lift", "arm_back") else "")
                        spawn_rot = key_env.door_env.head_camera.quat_to_matrix(env._spawn_yaw_quat[i].cpu().numpy())
                        sent = (env._spawn_pos[i].cpu().numpy() + spawn_rot @ np.asarray(command.point)
                                if command is not None else None)
                        hand_mm = (1000 * float(np.linalg.norm(sent - (centre[i] - env.scene.env_origins[i]).cpu().numpy()))
                                   if sent is not None and pushing else "")
                        if phase == "turn" and hand_mm != "":
                            push[i]["lag_mm"] = max(push[i]["lag_mm"], hand_mm)
                        rows.append({"batch": batch, "env": i, "t_s": round(step * interface.POLICY_DT, 2), "phase": phase,
                                     "base_x": round(float(base[i, 0]), 4), "base_y": round(float(base[i, 1]), 4),
                                     "base_z": round(float(base[i, 2]), 4),
                                     "roll_deg": round(math.degrees(float(rp[i, 0])), 2),
                                     "pitch_deg": round(math.degrees(float(rp[i, 1])), 2),
                                     "cmd_deg": round(cmd_deg, 2) if cmd_deg != "" else "",
                                     "key_deg": round(math.degrees(float(env.leg._handle.theta[i])), 2) if active else "",
                                     "measured_deg": round(float(measured[i]), 2) if active else "",
                                     "contact_n": round(float(env.leg.force_w[i].norm()), 2),
                                     "drive_n": round(float(env.leg._handle.drive_n[i]), 2) if active else "",
                                     "hand_miss_mm": round(hand_mm, 1) if hand_mm != "" else "",
                                     **{f"use_j{j + 1}": round(float(use[i, j]), 3) for j in range(6)},
                                     "ik": env.ik_status[i], "latched": int(env._latches[i].latch.locked),
                                     "wall_ms": round(wall_ms, 2)})
                    if (i == 0 and batch == 0 and env._overview is not None
                            and ((pushing and step % frame_every == 0) or (phase in ("settle_arm", "stand")
                                                                         and step % (8 * frame_every) == 0))):
                        frames.append(_save_frame(env, out_dir, len(frames), step * interface.POLICY_DT, phase,
                                                  rows[-1] if rows else {}))
        for i in range(count):
            record = dict(env.records[i])
            record.pop("phase", None)
            look = dict(env.look_records[i])
            p = push[i]
            fell = worst[i]["min_base_z"] < 0.15 or worst[i]["max_tilt_deg"] > 45.0
            max_deg = math.degrees(float(env.leg.max_theta[i])) if record.get("after") else None
            held = record.get("held") or {}
            attempt = {"batch": batch, "env": i, "site": dataclasses.asdict(env.truth_sites[i]),
                       "look": {"found": look.get("found"), "detections": look.get("detections"),
                                "lidar": look.get("lidar_heading"), "tag_only": look.get("tag_only")},
                       **record, **worst[i], "fell": fell,
                       "max_key_deg": round(max_deg, 2) if max_deg is not None else None,
                       "unlocked": max_deg is not None and max_deg >= at.UNLOCK_DEG,
                       "reached_stop": max_deg is not None and max_deg >= at.END_DEG - 1.0,
                       "turn_hand_lag_max_mm": round(p["lag_mm"], 1),
                       "verified_unlocked": ((held.get("measured_deg") or -90.0) >= at.UNLOCK_DEG
                                             and p["lag_mm"] < arm_env.TRACK_LIMIT_MM),
                       "peak_contact_n": round(float(env.leg.peak_n[i]), 2),
                       "turn_contact_n_median": round(float(np.median(p["turn_contact_n"])), 2) if p["turn_contact_n"] else None,
                       "torque_use_max": [round(float(v), 3) for v in p["use_max"]],
                       "saturated_share": round(p["saturated_steps"] / max(1, p["steps"]), 3)}
            attempt["damaged"] = attempt["peak_contact_n"] > leg_cfg.DAMAGE_N
            attempt["success"] = bool(attempt["unlocked"] and not fell and not attempt["damaged"])
            attempts.append(attempt)
        remaining -= count
        batch += 1

    def share(key):
        return sum(1 for a in attempts if a.get(key))
    summary = {"attempts": len(attempts), "lock_friction_nm": friction, "lock_as_push_n": round(friction / at.PUSH_REACH_M, 2),
               "tag_found": sum(1 for a in attempts if a["look"].get("found")), "unlocked": share("unlocked"),
               "reached_stop": share("reached_stop"), "verified_unlocked": share("verified_unlocked"),
               "fell": share("fell"), "damaged": share("damaged"), "success": share("success"),
               "max_key_deg": [a.get("max_key_deg") for a in attempts]}
    report.update({"status": status, "summary": summary, "frames": len(frames)})
    (out_dir / "run.json").write_text(json.dumps(report, indent=2, default=str))
    (out_dir / "summary.json").write_text(json.dumps({"summary": summary, "attempts": attempts}, indent=2, default=str))
    if rows:
        with (out_dir / "trace.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    if frames:
        _gif(out_dir, frames)
    print(json.dumps({"out_dir": str(out_dir), "summary": summary}, indent=2), flush=True)
    for a in attempts:
        print(json.dumps({k: a.get(k) for k in ("env", "unlocked", "reached_stop", "verified_unlocked", "max_key_deg",
                                                "fell", "at_stance", "touched", "held", "after", "estimate",
                                                "torque_use_max", "saturated_share", "turn_contact_n_median")},
                         default=str), flush=True)
    sys.stdout.flush()
    env.close()
    os._exit(0)


def _save_frame(env, out_dir: Path, n: int, t: float, phase: str, row: dict) -> str:
    """The overview camera's picture, captioned: time, phase, the key's angle and the push."""
    import cv2

    rgb = env.overview_rgb(0)
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    key = row.get("key_deg", "")
    text = f"{t:5.1f} s  {phase}" + (f"  key {key:+.1f} deg  push {row.get('contact_n', 0):.1f} N" if key != "" else "")
    cv2.rectangle(image, (0, 0), (image.shape[1], 34), (255, 255, 255), -1)
    cv2.putText(image, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 2, cv2.LINE_AA)
    name = f"frames/{n:03d}_{t:05.1f}s_{phase}.png"
    cv2.imwrite(str(out_dir / name), image)
    return name


def _gif(out_dir: Path, frames: list[str]) -> None:
    """The push as an animated GIF at a third of the frames' size (`push.gif`)."""
    try:
        from PIL import Image
    except ImportError:
        return
    images = [Image.open(out_dir / f) for f in frames]
    images = [im.resize((im.width // 3, im.height // 3)) for im in images]
    images[0].save(out_dir / "push.gif", save_all=True, append_images=images[1:], duration=120, loop=0)


if __name__ == "__main__":
    raise SystemExit(main())

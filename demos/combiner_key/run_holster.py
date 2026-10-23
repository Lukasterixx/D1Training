#!/usr/bin/env python3
"""The key drawn out of Lukas's holster beside the D1's base (`holster.py`, `holster_env.py`): the robot stands on the
walking policy, the arm unfolds to the holster, slides its open jaws onto the bar's root, closes, and draws the key out
along the holster's axis.

    ./demos/combiner_key/run_holster.py                                    # watch one, pictures in frames/
    ./demos/combiner_key/run_holster.py --speed 1 --joint_speed 0          # the first, slower moves (stop-start on the D1)
    ./demos/combiner_key/run_holster.py --attempts 8 --headless --no_overview
    ./demos/combiner_key/run_holster.py --attempts 8 --headless --no_overview --holster_error_mm 5   # a misplaced holster

Writes `logs/combiner_key_holster/<stamp>_seed<S>[_<name>]/` (`run.json`, `summary.json`, `trace.csv`, `frames/`,
`draw.gif`); then `./dashboard.py record <dir> --title "..."`.

An attempt **draws the key** when, at the end of the hold, the key is out along the holster's axis by more than its
bar's far end started inside the holster's open end (`holster_env.OUT_MM`), still in the hand (the grip's spring within
`GRIP_HELD_MM`), and the robot is standing.
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

#: The key still in the hand: its pose within this of where the grip holds it.
GRIP_HELD_MM = 10.0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--attempts", type=int, default=1)
    p.add_argument("--num_envs", type=int, default=None, help="Default: the attempts when headless, else 1.")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--settle_s", type=float, default=4.0, help="The spawn's settle before the unfold.")
    p.add_argument("--walker", default="p2dingo_gait")
    p.add_argument("--speed", type=float, default=2.0,
                   help="The moves (turn, reach; and onto the bar and the draw unless paced) this many times as fast "
                        "as holster_env's phase lengths (holster_env.SPEED).")
    p.add_argument("--joint_speed", type=float, default=0.6,
                   help="Pace the slide onto the bar and the draw so the fastest joint turns at this many rad/s "
                        "(holster.joint_paced; holster_env.JOINT_SPEED_RAD_S); 0 eases them evenly along the line.")
    p.add_argument("--holster_error_mm", type=float, default=0.0,
                   help="Put each attempt's holster (and the key in it) off from where the draw is planned, uniformly "
                        "within this on each axis of the base's frame, mm. The plan never sees it.")
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
    args.enable_cameras = not args.no_overview
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
    from demos.combiner_key import holster, holster_env

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    name = f"{stamp}_seed{args.seed}" + (f"_{args.run_name}" if args.run_name else "")
    out_dir = ROOT / "logs/combiner_key_holster" / name
    (out_dir / "frames").mkdir(parents=True, exist_ok=True)
    registration = holster.load()
    early = {"mode": "combiner_key_holster", "status": "running", "seed": args.seed,
             "arguments": {k: v for k, v in vars(args).items() if not k.startswith("_")},
             "command": " ".join(sys.argv), "git_commit": git("rev-parse", "HEAD"), "git_status": git("status", "--short"),
             "holster": {"registration": registration["inputs"], "key_mirrored": registration["key_mirrored"],
                         "grip_along_m": holster.GRIP_ALONG_M, "grip_roll": holster.GRIP_ROLL,
                         "ik_soft": holster.IK_SOFT, "pre_out_m": holster.PRE_OUT_M, "slide_m": holster.SLIDE_M,
                         "speed": args.speed, "joint_speed": args.joint_speed,
                         "phase_s": {p.name: round(p.duration_s, 3) for p in holster_env.holster_phases(args.speed, args.joint_speed)},
                         "jaw_open_m": holster.JAW_OPEN_M, "jaw_hold_m": holster.JAW_HOLD_M}}
    (out_dir / "run.json").write_text(json.dumps(early, indent=2))

    robot_usd = build_welded_robot_usd(
        go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd", d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
        out_usd_path=str(ROOT / "generated/go2_d1.usd"), mount_pos=ARM_MOUNT_POS_M, arm_mass_kg=3.152).usd_path
    args.joint_speed = args.joint_speed or None
    phases = holster_env.holster_phases(args.speed, args.joint_speed)
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
    cfg.overview = not args.no_overview
    cfg.leg_variant, cfg.leg_checkpoint = "none", ""
    cfg.holster_speed = args.speed
    if not args.headless:
        # From the robot's front right, onto the holster.
        cfg.viewer.origin_type = "env"
        cfg.viewer.env_index = 0
        cfg.viewer.eye = (0.55, -0.60, 0.65)
        cfg.viewer.lookat = (0.0, -0.08, 0.40)

    env = holster_env.HolsterEnv(cfg)
    env.attach_latches(locked=True)
    env.hide_object = False
    env.hide_furniture = True
    env.command_tip = False
    env.jaw_override = None
    env.jaw_compensation = True
    env.scripts = [demo_script.DemoScript(list(phases)) for _ in range(env.num_envs)]
    walker = decoupled.WalkingPolicy(decoupled.walker_checkpoint(env.walker), env.device)

    report = {**early, "robot_usd": robot_usd, "box_usd": str(box_usd), "phases": [p.name for p in phases],
              "robot": "the combiner door demo's: the lidar and the RealSenses drawn, not simulated (no wrist mass)",
              "box": "the combiner box 3 m behind the robot, out of the way"}
    attempts, rows, frames = [], [], []
    remaining, batch = args.attempts, 0
    status = "finished"
    steps = int(round((args.settle_s + total_s) / interface.POLICY_DT)) + 2
    frame_every = max(1, round(holster_env.FRAME_EVERY_S / interface.POLICY_DT))
    while remaining > 0 and app.is_running():
        count = min(remaining, env.num_envs)
        env.place([props.BoxSite(distance_m=3.0, bearing_deg=180.0, yaw_deg=0.0) for _ in range(env.num_envs)])
        for demo in env.scripts:
            demo.phases = [demo_script.Phase("spawn", args.settle_s, target=lambda st: demo_script.HOME_POINT,
                                             jaw_m=holster.JAW_OPEN_M)] + list(phases)
        env.reset()
        shift_rng = np.random.default_rng(args.seed * 1000 + batch)
        shifts = shift_rng.uniform(-1.0, 1.0, (env.num_envs, 3)) * args.holster_error_mm / 1000.0
        env.holster_shift[:] = torch.as_tensor(shifts, dtype=torch.float32, device=env.device)
        worst = {i: {"min_base_z": 9.0, "max_tilt_deg": 0.0} for i in range(env.num_envs)}
        draw = {i: {"use_max": np.zeros(6), "steps": 0, "saturated_steps": 0, "hand_miss_max_mm": 0.0,
                    "draw_hand_miss_max_mm": 0.0, "grip_err_max_mm": 0.0, "finger_contact_max_n": 0.0,
                    "key_across_max_mm": 0.0} for i in range(env.num_envs)}
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
                centre, _, _ = env._hand_frame()
                contact = env.contact_state()
                fingers = contact["finger_n"].cpu().numpy()
                for i in range(count):
                    command = env.commands[i]
                    phase = command.phase if command is not None else ""
                    if phase != last_phase[i]:
                        last_phase[i] = phase
                        if env._overview is not None and phase == "turn":
                            env.aim_overview()
                    w = worst[i]
                    w["min_base_z"] = min(w["min_base_z"], float(base[i, 2]))
                    w["max_tilt_deg"] = max(w["max_tilt_deg"], math.degrees(float(np.abs(rp[i]).max())))
                    drawing = phase in holster_env.DRAW_PHASES
                    out_mm, across_mm, tilt_deg = env.key_out_mm(i)
                    d = draw[i]
                    spawn_rot = holster_env.door_env.head_camera.quat_to_matrix(env._spawn_yaw_quat[i].cpu().numpy())
                    sent = (env._spawn_pos[i].cpu().numpy() + spawn_rot @ np.asarray(command.point)
                            if command is not None else None)
                    hand_mm = (1000 * float(np.linalg.norm(sent - (centre[i] - env.scene.env_origins[i]).cpu().numpy()))
                               if sent is not None and phase in ("pre", "on", "close", "draw", "hold") else None)
                    finger_n = float(fingers[i]) if fingers is not None else None
                    if drawing:
                        d["use_max"] = np.maximum(d["use_max"], use[i])
                        d["steps"] += 1
                        d["saturated_steps"] += int(use[i].max() >= 0.99)
                        if hand_mm is not None:
                            d["hand_miss_max_mm"] = max(d["hand_miss_max_mm"], hand_mm)
                            if phase == "draw":
                                d["draw_hand_miss_max_mm"] = max(d["draw_hand_miss_max_mm"], hand_mm)
                        if phase in ("draw", "hold"):
                            d["grip_err_max_mm"] = max(d["grip_err_max_mm"], float(env.grip_err_mm[i]))
                        if finger_n is not None:
                            d["finger_contact_max_n"] = max(d["finger_contact_max_n"], finger_n)
                        if phase in ("pre", "on", "close", "draw") and out_mm < holster_env.OUT_MM:
                            d["key_across_max_mm"] = max(d["key_across_max_mm"], across_mm)
                    if drawing or step % 5 == 0:
                        rows.append({"batch": batch, "env": i, "t_s": round(step * interface.POLICY_DT, 2), "phase": phase,
                                     "base_z": round(float(base[i, 2]), 4),
                                     "roll_deg": round(math.degrees(float(rp[i, 0])), 2),
                                     "pitch_deg": round(math.degrees(float(rp[i, 1])), 2),
                                     "key_out_mm": round(out_mm, 2), "key_across_mm": round(across_mm, 2),
                                     "key_tilt_deg": round(tilt_deg, 2),
                                     "grip_err_mm": round(float(env.grip_err_mm[i]), 2),
                                     "hand_miss_mm": round(hand_mm, 1) if hand_mm is not None else "",
                                     "finger_contact_n": round(finger_n, 2) if finger_n is not None else "",
                                     **{f"q{j + 1}": round(float(v), 3) for j, v in
                                        enumerate(data.joint_pos[i, env._arm_ids].cpu().numpy())},
                                     **{f"use_j{j + 1}": round(float(use[i, j]), 3) for j in range(6)},
                                     "ik": env.ik_status[i], "key_mode": env.key_mode[i], "wall_ms": round(wall_ms, 2)})
                    if (i == 0 and batch == 0 and env._overview is not None and drawing and step % frame_every == 0):
                        frames.append(_save_frame(env, out_dir, len(frames), step * interface.POLICY_DT, phase,
                                                  rows[-1]))
        for i in range(count):
            record = dict(env.records[i])
            record.pop("phase", None)
            d = draw[i]
            end = env._key_record(i)
            fell = worst[i]["min_base_z"] < 0.15 or worst[i]["max_tilt_deg"] > 45.0
            attempt = {"batch": batch, "env": i, "holster_shift_mm": [round(1000 * float(v), 2) for v in shifts[i]],
                       **record, "end": end, **worst[i], "fell": fell,
                       "torque_use_max": [round(float(v), 3) for v in d["use_max"]],
                       "saturated_share": round(d["saturated_steps"] / max(1, d["steps"]), 3),
                       "hand_miss_max_mm": round(d["hand_miss_max_mm"], 1),
                       "draw_hand_miss_max_mm": round(d["draw_hand_miss_max_mm"], 1),
                       "grip_err_max_mm": round(d["grip_err_max_mm"], 2),
                       "finger_contact_max_n": round(d["finger_contact_max_n"], 2),
                       "key_across_in_holster_max_mm": round(d["key_across_max_mm"], 2)}
            attempt["smoothness"] = _smoothness([r for r in rows if r["batch"] == batch and r["env"] == i])
            attempt["drawn"] = bool(end["key_out_mm"] > holster_env.OUT_MM and end["grip_err_mm"] < GRIP_HELD_MM
                                    and env.key_mode[i] == "held" and not fell)
            attempts.append(attempt)
        remaining -= count
        batch += 1

    summary = {"attempts": len(attempts), "drawn": sum(1 for a in attempts if a["drawn"]),
               "fell": sum(1 for a in attempts if a["fell"]),
               "holster_error_mm": args.holster_error_mm, "speed": args.speed, "joint_speed": args.joint_speed,
               "draw_s": round(sum(p.duration_s for p in phases), 2),
               "draw_near_stopped": [a["smoothness"].get("draw", {}).get("near_stopped") for a in attempts],
               "key_out_mm": [a["end"]["key_out_mm"] for a in attempts],
               "grasp_miss_mm": [a.get("grasp", {}).get("grasp_miss_mm") for a in attempts],
               "grip_err_max_mm": [a["grip_err_max_mm"] for a in attempts],
               "torque_use_max": [max(a["torque_use_max"]) for a in attempts]}
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
        print(json.dumps(a, default=str), flush=True)
    sys.stdout.flush()
    env.close()
    os._exit(0)


def _smoothness(rows: list[dict]) -> dict:
    """Per move, how stop-go the arm was: for the joint that moves most in it, the share of 20 ms samples at under a
    fifth of its mean speed (`near_stopped`), its speed's spread over its mean (`ripple`), its mean speed and its mean
    step per 10 Hz command. The D1's firmware stops between setpoints when a step is small (`holster_env`)."""
    import numpy as np

    out = {}
    for phase in ("reach", "on", "draw"):
        part = [r for r in rows if r["phase"] == phase]
        if len(part) < 3:
            continue
        t = np.array([r["t_s"] for r in part])
        q = np.array([[r[f"q{j}"] for j in range(1, 7)] for r in part])
        speed = np.abs(np.diff(q, axis=0)) / np.maximum(np.diff(t), 1e-9)[:, None]
        joint = int(np.argmax(speed.mean(axis=0)))
        s = speed[:, joint]
        out[phase] = {"joint": joint + 1, "mean_rad_s": round(float(s.mean()), 3),
                      "step_deg": round(float(np.degrees(s.mean() * 0.1)), 2),
                      "near_stopped": round(float((s < 0.2 * s.mean()).mean()), 3),
                      "ripple": round(float(s.std() / max(s.mean(), 1e-9)), 3),
                      "peak_rad_s": round(float(speed.max()), 3)}
    return out


def _save_frame(env, out_dir: Path, n: int, t: float, phase: str, row: dict) -> str:
    """The overview camera's picture, captioned: time, phase, how far the key is out."""
    import cv2

    image = cv2.cvtColor(env.overview_rgb(0), cv2.COLOR_RGB2BGR)
    text = f"{t:5.1f} s  {phase}  key out {row.get('key_out_mm', 0):+.1f} mm"
    cv2.rectangle(image, (0, 0), (image.shape[1], 34), (255, 255, 255), -1)
    cv2.putText(image, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 2, cv2.LINE_AA)
    name = f"frames/{n:03d}_{t:05.1f}s_{phase}.png"
    cv2.imwrite(str(out_dir / name), image)
    return name


def _gif(out_dir: Path, frames: list[str]) -> None:
    """The draw as an animated GIF at half the frames' size (`draw.gif`)."""
    try:
        from PIL import Image
    except ImportError:
        return
    images = [Image.open(out_dir / f) for f in frames]
    images = [im.resize((im.width // 2, im.height // 2)) for im in images]
    images[0].save(out_dir / "draw.gif", save_all=True, append_images=images[1:], duration=120, loop=0)


if __name__ == "__main__":
    raise SystemExit(main())

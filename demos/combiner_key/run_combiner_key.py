#!/usr/bin/env python3
"""The combiner key demo: find the box with the head RealSense and the MID-360, insert Lukas's lever key into its cam
lock with the arm (inverse kinematics, no learning), walk in, and turn the key with the front-right foot (the
key-trained leg policy) until the door unlocks. See README.md.

    ./demos/combiner_key/run_combiner_key.py --leg_checkpoint <key stage-2 model.pt> --leg_variant position   # watch one
    ./demos/combiner_key/run_combiner_key.py --leg_checkpoint <...> --leg_variant force --attempts 8 --headless
    ./demos/combiner_key/run_combiner_key.py ... --key_inserted      # start with the key already in the lock

Writes `logs/combiner_key/<stamp>_<variant>[_inserted]_seed<N>[_<name>]/` (`run.json`, `summary.json`, `trace.csv`);
then `./dashboard.py record <dir> --title "..."`.
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
    p.add_argument("--leg_checkpoint", required=True, help="A key-trained leg policy (`--task leg --leg_object key`).")
    p.add_argument("--leg_variant", choices=("position", "force"), required=True,
                   help="Which the checkpoint is: the plan's P0 (position) or P1 (force).")
    p.add_argument("--key_inserted", action="store_true",
                   help="Start with the key seated in the lock: look, walk to the turn's stance, turn. No insertion.")
    p.add_argument("--attempts", type=int, default=1)
    p.add_argument("--num_envs", type=int, default=None, help="Default: the attempts when headless, else 1.")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--settle_s", type=float, default=4.0, help="The spawn's settle before the look.")
    p.add_argument("--lock_friction_nm", type=float, default=1.5, help="The lock's sticking friction, N·m.")
    p.add_argument("--lock_detent_nm", type=float, default=0.5, help="The cam's detent before the unlock, N·m.")
    p.add_argument("--walker", default="p2dingo_gait")
    p.add_argument("--handover", choices=("crouch", "settled", "direct", "direct_still"), default="crouch",
                   help="How the legs pass to the leg policy: 'crouch' (the demo) eases them from the walking policy's "
                        "last action to the default targets; 'settled' (a diagnostic) puts the robot, where it stands, in "
                        "the stance the leg policy trained from; 'direct' switches at once, the policy starting from the "
                        "walking policy's stance (for a policy trained from such stances); 'direct_still' the same with the "
                        "robot's velocities zeroed at the switch (a diagnostic).")
    p.add_argument("--legs", choices=("dcmotor", "unifp"), default="dcmotor",
                   help="The legs' actuator: 'dcmotor', the decoupled stack's (Isaac Lab's Go2 DCMotor, 23.5 N·m, what the "
                        "walking policy trained on); 'unifp', UniFP's ideal PD (the same 25 / 0.5 gains, knees 45.43 N·m), "
                        "what the leg policy trained on. The leg policy's frozen evaluation on DCMotor legs drops from 84 "
                        "to 3 of 108 (force-aware, seed 1) and stays at 20 (position-only): the stock group leaves the legs' "
                        "armature at 0 against the trained 2e-4; with 2e-4 it is 84 (F-168).")
    p.add_argument("--given_box", action="store_true",
                   help="No head camera or lidar: the look is handed the box's true placement. For development on a GPU "
                        "with no room for the renderer; not the demo.")
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
    args.enable_cameras = not args.given_box
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
    from demos.combiner_key import env as key_env, geometry as kg

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    name = (f"{stamp}_{args.leg_variant}{'_inserted' if args.key_inserted else ''}"
            f"{'_settled' if args.handover == 'settled' else ''}{'_unifplegs' if args.legs == 'unifp' else ''}"
            f"_seed{args.seed}") + (
        f"_{args.run_name}" if args.run_name else "")
    out_dir = ROOT / "logs/combiner_key" / name
    out_dir.mkdir(parents=True, exist_ok=True)
    early = {"mode": "combiner_key_demo", "status": "running", "seed": args.seed,
             "arguments": {k: v for k, v in vars(args).items() if not k.startswith("_")},
             "command": " ".join(sys.argv), "git_commit": git("rev-parse", "HEAD"), "git_status": git("status", "--short")}
    (out_dir / "run.json").write_text(json.dumps(early, indent=2))

    # The combiner door demo's robot: the lidar and its mount drawn, not simulated, and no wrist camera (the RealSense is
    # on the head). Not the leg policy's training robot, whose payload physics puts the wrist RealSense and its bracket
    # (99 g) on Link6: with the key, the D1 reaching the lock then needs ~100% of its shoulder's 3.33 N·m and sags ~16 mm
    # (2026-10-21, `--given_box` trials); without it, 78-84%.
    robot_usd = build_welded_robot_usd(
        go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd", d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
        out_usd_path=str(ROOT / "generated/go2_d1.usd"), mount_pos=ARM_MOUNT_POS_M, arm_mass_kg=3.152).usd_path
    phases = key_env.key_phases(args.key_inserted)
    total_s = sum(p.duration_s for p in phases)

    cfg = demo_env.UniFPDemoEnvCfg()
    cfg.task = "combiner"
    cfg.seed = args.seed
    cfg.scene.num_envs = args.num_envs
    cfg.scene.env_spacing = 3.0
    cfg.robot = decoupled.make_robot_cfg(robot_usd)
    if args.legs == "unifp":
        from unifp_isaaclab import robot as robot_mod
        unifp_group = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot").actuators["unifp"]
        legs = list(interface.ISAACLAB_NAMES[:12])
        keep = lambda d: {k: v for k, v in d.items() if k in legs} if isinstance(d, dict) else d
        cfg.robot.actuators["base_legs"] = unifp_group.replace(
            joint_names_expr=legs, stiffness=keep(unifp_group.stiffness), damping=keep(unifp_group.damping),
            effort_limit=keep(unifp_group.effort_limit), effort_limit_sim=keep(unifp_group.effort_limit_sim),
            armature=keep(unifp_group.armature), friction=keep(unifp_group.friction))
    # PhysX's GPU buffers sized for a few robots, not thousands: the demo runs beside training on the shared GPU.
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
    cfg.key_inserted = args.key_inserted
    cfg.given_box = args.given_box
    cfg.handover = args.handover
    cfg.legs = args.legs
    cfg.leg_variant, cfg.leg_checkpoint = args.leg_variant, str(Path(args.leg_checkpoint).resolve())
    cfg.lock_friction_nm, cfg.lock_detent_nm = args.lock_friction_nm, args.lock_detent_nm
    if not args.headless:
        # From the robot's front right, onto the lock, the key and the right foot.
        cfg.viewer.origin_type = "env"
        cfg.viewer.env_index = 0
        cfg.viewer.eye = (0.15, -1.25, 0.85)
        cfg.viewer.lookat = (0.55, 0.25, 0.38)

    env = key_env.CombinerKeyEnv(cfg)
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
        """The box squarely ahead of the look, its door `LOOK_DOOR_AHEAD_M` off, spread a little."""
        ahead = kg.LOOK_DOOR_AHEAD_M + GEOMETRY.lock[0] + rng.uniform(-1, 1) * kg.PLACEMENT_AHEAD_JITTER_M
        side = rng.uniform(-1, 1) * kg.PLACEMENT_SIDE_JITTER_M
        yaw = 180.0 + rng.uniform(-1, 1) * kg.PLACEMENT_YAW_JITTER_DEG
        return props.BoxSite(distance_m=math.hypot(ahead, side), bearing_deg=math.degrees(math.atan2(side, ahead)),
                             yaw_deg=yaw)

    report = {**early, "robot_usd": robot_usd, "box_usd": str(box_usd), "phases": [p.name for p in phases],
              "leg": {"checkpoint": cfg.leg_checkpoint, "variant": args.leg_variant,
                      "plant": "unifp_train.handle.LeverHandle(pad=True), as trained (leg_cfg.KEY_*)",
                      "lock_friction_nm": args.lock_friction_nm, "lock_detent_nm": args.lock_detent_nm,
                      "actuator": "the decoupled stack's DCMotor (trained on UniFP's ideal PD, the same gains)",
                      "robot": "the combiner door demo's: payload drawn, not simulated (trained with the lidar's and the "
                               "wrist camera's mass and colliders)"},
              "key": {"stl": str(ROOT / "demos/combiner_key/assets" / kg.KEY_STL), "mirrored": kg.KEY_MIRRORED,
                      "grip": {"k_n_m": key_env.GRIP_K_N_M, "k_nm_rad": key_env.GRIP_K_NM_RAD},
                      "adapter": dataclasses.asdict(kg.ADAPTER)},
              "stances": {"insert_from_look": kg.INSERT_STANCE, "turn_from_look": kg.TURN_STANCE}}
    attempts, rows = [], []
    remaining, batch = args.attempts, 0
    status = "finished"
    steps = int(round((args.settle_s + total_s) / interface.POLICY_DT)) + 2
    while remaining > 0 and app.is_running():
        count = min(remaining, env.num_envs)
        env.place([sample_site() for _ in range(env.num_envs)])
        for demo in env.scripts:
            demo.phases = [demo_script.Phase("spawn", args.settle_s, target=lambda st: demo_script.HOME_POINT,
                                             jaw_m=key_env.JAW_HOLD_M)] + list(phases)
        env.reset()
        worst = {i: {"min_base_z": 9.0, "max_tilt_deg": 0.0, "max_stance_cm": 0.0} for i in range(env.num_envs)}
        with torch.no_grad():
            for step in range(steps):
                if not app.is_running():
                    status = "aborted"
                    break
                started = time.perf_counter()
                walk = env.action_from_legs(walker.act(env.walking_observation()))
                legs = env.leg_action(walk)
                action = torch.where(env.leg.active.unsqueeze(-1), legs, walk)
                env.step(action)
                wall_ms = 1000.0 * (time.perf_counter() - started)
                data = env._robot.data
                base = (data.root_pos_w - env.scene.env_origins).cpu().numpy()
                rp = interface.body_roll_pitch(data.root_quat_w).cpu().numpy()
                from unifp_train import leg_rewards
                stance = 100.0 * leg_rewards.stance_error(
                    data.body_pos_w[:, env._feet_bodies] - env.scene.env_origins.unsqueeze(1),
                    data.root_pos_w - env.scene.env_origins, env._base_yaw_quat()).cpu().numpy()
                for i in range(count):
                    w = worst[i]
                    command = env.commands[i]
                    phase = command.phase if command is not None else ""
                    w["min_base_z"] = min(w["min_base_z"], float(base[i, 2]))
                    w["max_tilt_deg"] = max(w["max_tilt_deg"], math.degrees(float(np.abs(rp[i]).max())))
                    if phase not in ("spawn",):
                        support = stance[i, [0, 2, 3]] if bool(env.leg.active[i]) else stance[i]
                        w["max_stance_cm"] = max(w["max_stance_cm"], float(support.max()))
                    if step % 5 == 0 or phase in ("pre", "align", "insert", "seat"):
                        leg = env.leg
                        rows.append({"batch": batch, "env": i, "t_s": round(step * interface.POLICY_DT, 2),
                                     "phase": phase, "legs": "leg_policy" if bool(leg.active[i]) else "walker",
                                     "base_x": round(float(base[i, 0]), 4), "base_y": round(float(base[i, 1]), 4),
                                     "base_z": round(float(base[i, 2]), 4),
                                     "key_deg": round(math.degrees(float(leg._handle.theta[i])), 2) if bool(leg.active[i]) else "",
                                     "foot_n": round(float(leg.force_w[i].norm()), 2),
                                     "cmd_n": round(float(leg._layer.force_cmd[i].norm()), 2),
                                     "stage": int(leg._stage[i]) if bool(leg.active[i]) else "",
                                     "key_mode": env.key_mode[i], "stance_max_cm": round(float(stance[i].max()), 2),
                                     "grip_err_mm": round(float(env.grip_err_mm[i]), 2),
                                     "grip_err_deg": round(float(env.grip_err_deg[i]), 2),
                                     "ik": env.ik_status[i],
                                     "wall_ms": round(wall_ms, 2)})
        for i in range(count):
            record = dict(env.records[i])
            record.pop("phase", None)
            look = dict(env.look_records[i])
            turn = record.get("turn") or {}
            seat = record.get("after_release") or {}
            fell = worst[i]["min_base_z"] < 0.15 or worst[i]["max_tilt_deg"] > 45.0
            attempt = {"batch": batch, "env": i, "site": dataclasses.asdict(env.truth_sites[i]),
                       "look": {"found": look.get("found"), "detections": look.get("detections"),
                                "lidar": look.get("lidar_heading"), "lidar_depth_shift_mm": look.get("lidar_depth_shift_mm"),
                                "tag_only": look.get("tag_only")},
                       **record, **worst[i], "fell": fell,
                       "inserted": True if args.key_inserted else bool(seat.get("seated")),
                       "unlocked": bool(turn.get("unlocked")), "verified_unlocked": bool(turn.get("verified_unlocked")),
                       "foot_down": bool(turn.get("foot_down"))}
            # Damage as the frozen evaluation counts it: any contact force on the foot, knee or head past 120 N.
            from unifp_train import leg_cfg as _leg_cfg
            attempt["damaged"] = (turn.get("peak_n") or 0.0) > _leg_cfg.DAMAGE_N
            attempt["success"] = (attempt["inserted"] and attempt["unlocked"] and attempt["foot_down"] and not fell
                                  and not attempt["damaged"])
            attempts.append(attempt)
        remaining -= count
        batch += 1

    def share(key):
        return sum(1 for a in attempts if a.get(key))
    summary = {"attempts": len(attempts), "tag_found": sum(1 for a in attempts if a["look"].get("found")),
               "inserted": share("inserted"), "turn_started": share("turn_started"), "unlocked": share("unlocked"),
               "verified_unlocked": share("verified_unlocked"), "foot_down": share("foot_down"), "fell": share("fell"),
               "damaged": share("damaged"),
               "success": share("success"),
               "max_stance_cm_median": float(np.median([a["max_stance_cm"] for a in attempts])) if attempts else None}
    report.update({"status": status, "summary": summary})
    (out_dir / "run.json").write_text(json.dumps(report, indent=2, default=str))
    (out_dir / "summary.json").write_text(json.dumps({"summary": summary, "attempts": attempts}, indent=2, default=str))
    if env.handover_states:
        (out_dir / "handover_states.json").write_text(json.dumps(env.handover_states))
    if rows:
        with (out_dir / "trace.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps({"out_dir": str(out_dir), "summary": summary}, indent=2), flush=True)
    for a in attempts:
        print(json.dumps({k: a.get(k) for k in ("env", "inserted", "unlocked", "verified_unlocked", "foot_down", "fell",
                                                "success", "aligned", "seat", "after_release", "turn", "estimate",
                                                "max_stance_cm")}, default=str), flush=True)
    sys.stdout.flush()
    env.close()
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""The insertion demo: the standing robot pushes the key in its hand into a keyed slot on a door, in PhysX.

    source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
    unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

    # watch one attempt, the estimate 2 mm off
    ./demos/insertion/run_demo.py --num_envs 1 --attempts 1 --lateral_mm 2
    # measure: sixteen attempts, headless
    ./demos/insertion/run_demo.py --attempts 16 --headless --lateral_mm 3
    # watch attempt after attempt until the window is closed (what demos/insertion/run_demo.sh runs)
    ./demos/insertion/run_demo.py --num_envs 1 --loop --hold_s 3

Then, as for every run: `./dashboard.py record <out dir> --title "..."`.

**What runs.** The training's task (`unifp_train.insert_env`) with a PhysX door and key (`env.py`): the task layer
takes the slot's pose from an *estimate* -- the true pose moved by `--lateral_mm` across the axis in a random direction
per attempt, tilted by `--tilt_deg` and rolled by `--roll_error_deg` -- sends the hand to the pre-insertion pose with
the approach along the estimated axis and the roll of the blade, then pushes in with a PI force law, lightly while
the key is on the door's face (and spirals across it if it stalls), harder once it is in. The key is rigid in the hand.

**Seated** is the tip reaching the seat's depth in the true slot. **What this does not show**: a real key's grip, which
can slip or tilt; the detent the training's plant has (the PhysX lock has none); a lock turned after insertion; any
estimate other than a known error added to the truth; anything on hardware.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

DEFAULT_CHECKPOINT = None
BLADES = {"level": 0.0, "upright": math.pi / 2}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT, help="The policy (an insertion-trained one).")
    parser.add_argument("--zero_actions", action="store_true", help="Discard the policy's actions: the baseline.")
    parser.add_argument("--attempts", type=int, default=16)
    parser.add_argument("--num_envs", type=int, default=None)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--lateral_mm", type=float, default=2.0,
                        help="How far the estimate misses the slot across its axis, mm; direction random per attempt.")
    parser.add_argument("--tilt_deg", type=float, default=1.5, help="How far the estimated axis misses the true one.")
    parser.add_argument("--roll_error_deg", type=float, default=3.0, help="How far the estimated blade roll misses.")
    parser.add_argument("--height_m", type=float, nargs=2, default=(0.70, 0.86), metavar=("LOW", "HIGH"),
                        help="The mouth's height above the ground, drawn per attempt.")
    parser.add_argument("--side_m", type=float, nargs=2, default=(-0.04, 0.04), metavar=("LEFT", "RIGHT"))
    parser.add_argument("--ahead_m", type=float, default=0.52, help="The mouth's distance ahead of the robot.")
    parser.add_argument("--blade", choices=("level", "upright", "random"), default="random")
    parser.add_argument("--elevation_deg", type=float, default=0.0, help="The slot axis's elevation (a leaning door).")
    parser.add_argument("--speed_m_s", type=float, default=0.02, help="The task layer's reference speed.")
    parser.add_argument("--search_n", type=float, default=6.0, help="The push cap while the key is on the face.")
    parser.add_argument("--max_n", type=float, default=30.0, help="The push cap once it is in.")
    parser.add_argument("--episode_s", type=float, default=16.0)
    parser.add_argument("--tip_correction", type=float, default=None, metavar="GAIN_PER_S",
                        help="The task layer's tip loop (insertion.PoseCorrection): the jaw goal placed so the key's tip "
                             "lands on its goal given the hand's measured approach, plus the integral of the tip's miss "
                             "at this gain. Default insert_cfg.TIP_CORRECTION_GAIN; 0: off (the runs before 2026-09-26's v6).")
    parser.add_argument("--roll_max_deg", type=float, default=None,
                        help="The roll loop's clamp, degrees (default insert_cfg.ROLL_CORRECTION_MAX_RAD).")
    parser.add_argument("--elevation_max_deg", type=float, default=None,
                        help="The elevation loop's clamp, degrees (default insert_cfg.ELEVATION_CORRECTION_MAX_RAD).")
    parser.add_argument("--compliant_key", action="store_true",
                        help="Hang the blade on two sprung joints at the jaw centre (key.py; the PhysX form of the "
                             "training's insertion.CompliantGrip) instead of fixing it to Link6.")
    parser.add_argument("--attitude_correction", type=float, default=None, metavar="GAIN_PER_S",
                        help="The approach-elevation and blade-roll loops' gain. Default insert_cfg's; 0: off.")
    parser.add_argument("--loop", action="store_true",
                        help="Keep drawing new attempts until the window is closed; --attempts is ignored. The results "
                             "are written after every batch.")
    parser.add_argument("--hold_s", type=float, default=None,
                        help="End an attempt this long after its key is seated, instead of at --episode_s: for watching. "
                             "Its peak force and final depth then stop there.")
    parser.add_argument("--realtime", action=argparse.BooleanOptionalAction, default=None,
                        help="Hold each policy step to its 20 ms of wall time (default: in the viewer, not headless).")
    parser.add_argument("--scenery", action=argparse.BooleanOptionalAction, default=None,
                        help="Draw the door the lock is in: a wooden door in a wall, its casing, an escutcheon and a "
                             "lever handle (asset.build_scenery_usd), and a floor of boards; the viewer then looks at it "
                             "from behind the robot. Visual only. Default: in the viewer, not headless.")
    parser.add_argument("--camera_body", action=argparse.BooleanOptionalAction, default=None,
                        help="Draw the wrist RealSense and its bracket at the saved mount (demos/cup/pick_demo). Visual "
                             "only, with no mass and no camera rendering. Default: in the viewer, not headless.")
    parser.add_argument("--run_name", default=None)
    parser.add_argument("--output", default=str(ROOT / "logs/insertion_demo"))
    return parser


def draw_plan(args, count: int, rng: random.Random) -> list[dict]:
    rows = []
    for _ in range(count):
        blade = rng.choice(list(BLADES.values())) if args.blade == "random" else BLADES[args.blade]
        direction = rng.uniform(0.0, 2 * math.pi)
        rows.append({"height_m": rng.uniform(*args.height_m), "side_m": rng.uniform(*args.side_m),
                     "roll_rad": blade, "direction_rad": direction,
                     "tilt_direction_rad": rng.uniform(0.0, 2 * math.pi),
                     "roll_error_sign": rng.choice((-1.0, 1.0))})
    return rows


def median(values):
    present = sorted(v for v in values if v is not None)
    return round(present[len(present) // 2], 3) if present else None


def summarise(attempts: list[dict]) -> dict:
    return {"attempts": len(attempts), "successes": sum(a["success"] for a in attempts),
            "falls": sum(a["fell"] for a in attempts), "damaged": sum(a["damaged"] for a in attempts),
            "engaged": sum(a["engage_step"] is not None for a in attempts),
            "searched": sum(a["searched"] for a in attempts),
            "time_to_seat_s_median": median([a["time_to_seat_s"] for a in attempts]),
            "peak_force_n_median": median([a["peak_force_n"] for a in attempts]),
            "peak_lateral_n_median": median([a["peak_lateral_n"] for a in attempts]),
            "approach_err_deg_median": median([a["approach_err_deg"] for a in attempts]),
            "roll_err_deg_median": median([a.get("roll_err_deg") for a in attempts]),
            "final_depth_mm_median": median([a["final_depth_mm"] for a in attempts])}


def plan_tensors(args, rows, device, geometry) -> dict:
    import torch

    t = lambda values: torch.tensor(values, dtype=torch.float32, device=device)
    lateral, tilt = args.lateral_mm / 1000.0, math.radians(args.tilt_deg)
    return {"mouth": t([[args.ahead_m, r["side_m"], r["height_m"]] for r in rows]),
            "elevation": t([math.radians(args.elevation_deg)] * len(rows)),
            "roll": t([r["roll_rad"] for r in rows]),
            "lateral": t([[lateral * math.cos(r["direction_rad"]), lateral * math.sin(r["direction_rad"])] for r in rows]),
            "tilt": t([[tilt * math.cos(r["tilt_direction_rad"]), tilt * math.sin(r["tilt_direction_rad"])] for r in rows]),
            "roll_error": t([math.radians(args.roll_error_deg) * r["roll_error_sign"] for r in rows]),
            "detent_n": t([0.0] * len(rows)),
            "conditions": {"clearance_m": geometry.clearance, "chamfer_m": geometry.chamfer, "depth_m": geometry.depth,
                           "detent_fraction": 0.65, "mu": geometry.friction, "standoff_m": 0.012,
                           "speed_m_s": args.speed_m_s, "search_n": args.search_n, "max_n": args.max_n}}


def main() -> int:
    parser = build_parser()
    if len(sys.argv) > 1 and ("-h" in sys.argv or "--help" in sys.argv):
        parser.parse_args()
        return 0
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.checkpoint is None and not args.zero_actions:
        parser.error("--checkpoint is required (or --zero_actions)")
    for name in ("realtime", "scenery", "camera_body"):
        if getattr(args, name) is None:
            setattr(args, name, not args.headless)
    simulation_app = AppLauncher(args).app

    import torch

    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObjectCfg
    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

    from demos.insertion.asset import FLOOR_BEHIND_M, build_floor_usd, build_key_usd, build_scenery_usd, build_slot_usd
    from demos.insertion.env import InsertionDemoEnv, InsertionDemoEnvCfg
    from demos.insertion.geometry import GEOMETRY
    from demos.insertion.key import write_key_urdf
    from unifp_isaaclab import interface, robot as robot_mod
    from types import SimpleNamespace

    from unifp_train import hook_cfg, insertion, rewards as reward_mod
    from unifp_train.eval import EvaluablePolicy
    from weld import build_welded_robot_usd

    variant = "_compliant" if args.compliant_key else ""
    key_urdf = write_key_urdf(ROOT / "d1_arm/d1.urdf", ROOT / f"generated/insertion/d1_key{variant}.urdf",
                              compliant=args.compliant_key)
    robot_usd = build_welded_robot_usd(
        go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd", d1_urdf_path=str(key_urdf),
        out_usd_path=str(ROOT / f"generated/insertion/go2_d1_key{variant}.usd"),
        arm_mass_kg=3.152).usd_path
    slot_usd = build_slot_usd(ROOT / "generated/insertion/slot_door.usd")
    drawn = {"key_usd": str(build_key_usd(ROOT / f"generated/insertion/key{variant}.usd", compliant=args.compliant_key)),
             "scenery_usd": (str(build_scenery_usd(ROOT / "generated/insertion/scenery.usd",
                                                   room_depth_m=args.ahead_m + FLOOR_BEHIND_M)) if args.scenery else None),
             "floor_usd": (str(build_floor_usd(ROOT / "generated/insertion/floor.usd", wall_m=args.ahead_m))
                           if args.scenery else None),
             "camera_body": None}
    if args.camera_body:
        from demos.cup.pick_demo.camera import resolve_mount
        from demos.cup.pick_demo.camera_asset import build_bracket_usd, build_camera_usd

        mount, mount_source, mount_file = resolve_mount()
        body, bracket = build_camera_usd(ROOT / "generated/pick_demo"), build_bracket_usd(ROOT / "generated/pick_demo")
        drawn["camera_body"] = {"mount_source": mount_source, "mount_file": mount_file,
                                "pos_link6_m": [float(v) for v in mount.pos_link6],
                                "quat_wxyz": [float(v) for v in mount.quat_wxyz()],
                                "body_usd": body["usd_path"], "bracket_usd": bracket and bracket["usd_path"]}
        print(f"[insertion demo] wrist RealSense drawn at {mount_source}", flush=True)

    num_envs = args.num_envs or args.attempts
    cfg = InsertionDemoEnvCfg()
    if args.tip_correction is not None:
        cfg.tip_correction_gain = args.tip_correction
    if args.attitude_correction is not None:
        cfg.elevation_correction_gain = cfg.roll_correction_gain = args.attitude_correction
    if args.roll_max_deg is not None:
        cfg.roll_correction_max_rad = math.radians(args.roll_max_deg)
    if args.elevation_max_deg is not None:
        cfg.elevation_correction_max_rad = math.radians(args.elevation_max_deg)
    cfg.scene.num_envs = num_envs
    cfg.key_usd, cfg.scenery_usd, cfg.floor_usd = drawn["key_usd"], drawn["scenery_usd"], drawn["floor_usd"]
    if args.scenery and num_envs > 1:
        cfg.scene.env_spacing = max(cfg.scene.env_spacing, 4.0)      # room for each robot's wall and floor
    if drawn["camera_body"]:
        cfg.camera_body_usd = drawn["camera_body"]["body_usd"]
        cfg.camera_body_pose = (tuple(drawn["camera_body"]["pos_link6_m"]), tuple(drawn["camera_body"]["quat_wxyz"]))
        cfg.bracket_usd = drawn["camera_body"]["bracket_usd"]
    cfg.episode_length_s = args.episode_s
    armature = {name: (hook_cfg.ARM_ARMATURE_KG_M2 if name in interface.ISAACLAB_NAMES[12:18] else robot_mod.ARM_ARMATURE)
                for name in interface.ISAACLAB_NAMES}
    cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot", armature=armature)
    cfg.robot_usd = robot_usd
    if args.compliant_key:
        from isaaclab.actuators import ImplicitActuatorCfg

        from demos.insertion.key import KEY_JOINTS

        # The grip's springs, as the training's `insertion.CompliantGrip` has them; a little armature so a 15 g blade
        # on a 2 N·m/rad spring does not ring at the physics rate.
        cfg.robot.actuators["key"] = ImplicitActuatorCfg(
            joint_names_expr=list(KEY_JOINTS), stiffness=insertion.GRIP_STIFFNESS_NM_PER_RAD,
            damping=insertion.GRIP_DAMPING_NMS_PER_RAD, effort_limit_sim=5.0, armature=1e-4)
        cfg.robot.init_state.joint_pos.update({name: 0.0 for name in KEY_JOINTS})
        cfg.compliant_key = True
        cfg.passive_joints = KEY_JOINTS
    cfg.door = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Door",
        spawn=sim_utils.UsdFileCfg(usd_path=str(slot_usd),
                                   rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.0, 0.7)))
    if not args.headless:
        cfg.viewer.env_index = 0
        if args.scenery:
            # From behind the robot's right, on the door: fixed in environment 0, so it can be moved with the mouse.
            mouth = (args.ahead_m, sum(args.side_m) / 2, sum(args.height_m) / 2)
            cfg.viewer.origin_type = "env"
            cfg.viewer.eye = (mouth[0] - 0.95, mouth[1] - 1.25, mouth[2] + 0.35)
            cfg.viewer.lookat = (mouth[0] - 0.25, mouth[1], mouth[2] - 0.15)
        else:
            cfg.viewer.origin_type = "asset_root"
            cfg.viewer.asset_name = "robot"
            cfg.viewer.eye = (0.35, -1.0, 0.95)
            cfg.viewer.lookat = (0.45, 0.0, 0.7)
    env = InsertionDemoEnv(cfg)
    policy = None if args.zero_actions else EvaluablePolicy(args.checkpoint, device=str(env.device))
    controller = "zero actions" if policy is None else os.path.basename(args.checkpoint)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    tag = "zero" if policy is None else "policy"
    out_dir = Path(args.output) / f"{stamp}_insertion_{tag}_seed{args.seed}{'_' + args.run_name if args.run_name else ''}"
    out_dir.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)
    report = {"mode": "insertion_demo", "status": "running", "seed": args.seed, "controller": controller,
              "checkpoint": os.path.abspath(args.checkpoint) if policy else None,
              "checkpoint_iteration": policy.iteration if policy else None,
              "arguments": {k: v for k, v in vars(args).items() if not k.startswith("_")},
              "geometry": {k: getattr(GEOMETRY, k) for k in ("key_half_u", "key_half_w", "key_length", "clearance",
                                                             "chamfer", "depth", "friction")},
              "num_envs": num_envs, "attempts": args.attempts, "robot_usd": robot_usd, "slot_usd": str(slot_usd),
              "drawn_only": drawn,
              "outer_loops": {"tip_gain": cfg.tip_correction_gain, "elevation_gain": cfg.elevation_correction_gain,
                              "roll_gain": cfg.roll_correction_gain, "tip_max_m": cfg.tip_correction_max_m,
                              "elevation_max_rad": cfg.elevation_correction_max_rad,
                              "roll_max_rad": cfg.roll_correction_max_rad},
              "compliant_key": ({"stiffness_nm_per_rad": insertion.GRIP_STIFFNESS_NM_PER_RAD,
                                 "damping_nms_per_rad": insertion.GRIP_DAMPING_NMS_PER_RAD,
                                 "max_rad": insertion.GRIP_MAX_RAD} if args.compliant_key else None),
              "command": " ".join(sys.argv), "out_dir": str(out_dir)}
    print(json.dumps({"out_dir": str(out_dir)}), flush=True)
    (out_dir / "run.json").write_text(json.dumps(report, indent=2))

    fields = ["attempt", "step", "time_s", "engaged", "inserted", "searching", "theta", "s_mm", "ref_mm", "depth_mm",
              "across_u_mm", "across_w_mm", "force_cmd_n", "key_force_n", "key_axial_n", "key_lateral_n",
              "approach_err_deg", "roll_err_deg", "base_pitch_deg", "fell"]
    attempts = []
    hold_steps = None if args.hold_s is None else max(1, round(args.hold_s / env.step_dt))
    trace_file = open(out_dir / "trace.csv", "w", newline="")
    trace = csv.DictWriter(trace_file, fieldnames=fields)
    trace.writeheader()
    remaining, batch, closed = args.attempts, 0, False
    while args.loop or remaining > 0:
        if not simulation_app.is_running():
            closed = True
            break
        count = num_envs if args.loop else min(remaining, num_envs)
        rows_out = []
        rows = draw_plan(args, num_envs, rng)
        env.set_evaluation(plan_tensors(args, rows, env.device, GEOMETRY))
        # Inside inference mode, as the steps are: a second batch's reset writes into buffers the first batch's steps
        # created there, which torch refuses outside it.
        with torch.inference_mode():
            obs, _ = env.reset()
        done = torch.zeros(num_envs, dtype=torch.bool, device=env.device)
        deadline = time.perf_counter()
        best = [{"engage_step": None, "seated_step": None, "searched": False, "inserted": False, "peak_force_n": 0.0,
                 "peak_lateral_n": 0.0, "approach_err_deg": None, "fell": False, "damaged": False, "final_depth_mm": None}
                for _ in range(num_envs)]
        with torch.inference_mode():
            for step in range(int(env.max_episode_length) - 1):
                if not simulation_app.is_running():
                    closed = True
                    break
                actions = (torch.zeros(num_envs, interface.NUM_ACTIONS, device=env.device) if policy is None
                           else policy.act(obs["policy"]))
                obs, _, terminated, truncated, _ = env.step(actions)
                ended = (terminated | truncated) & ~done
                slot, layer = env._slot, env._layer
                key = env.key_state()
                rel = key["tip"] - slot.mouth
                across_u, across_w = (rel * slot.u).sum(-1), (rel * slot.w).sum(-1)
                force = env._key_force_w
                axial = (force * layer.n).sum(-1)
                lateral = (force - axial.unsqueeze(-1) * layer.n).norm(dim=-1)
                frame = env._hand_frame_terms()
                approach_err = torch.rad2deg(torch.acos((frame["ee_approach_w"] * env._approach_des_w).sum(-1).clamp(-1, 1)))
                state = SimpleNamespace(**frame)
                roll_err = torch.rad2deg(insertion.wrap_half_turn(reward_mod.tool_roll(state) - env._roll)).abs()
                pitch = torch.rad2deg(interface.body_roll_pitch(env._robot.data.root_quat_w)[:, 1])
                seated = slot.active & (slot.tip_depth >= slot.depth - 0.001)
                for i in range(count):
                    if bool(done[i]):
                        continue
                    b = best[i]
                    if bool(env._engaged[i]) and b["engage_step"] is None:
                        b["engage_step"] = step
                        b["approach_err_deg"] = round(float(approach_err[i]), 2)
                        b["roll_err_deg"] = round(float(roll_err[i]), 2)
                    if bool(seated[i]) and b["seated_step"] is None:
                        b["seated_step"] = step
                    b["searched"] |= bool(layer.theta[i] > 0.0)
                    b["inserted"] |= bool(layer.inserted[i])
                    if bool(env._engaged[i]):
                        b["peak_force_n"] = max(b["peak_force_n"], float(force[i].norm()))
                        b["peak_lateral_n"] = max(b["peak_lateral_n"], float(lateral[i]))
                        b["final_depth_mm"] = round(1000 * float(slot.tip_depth[i]), 2)
                    if bool(ended[i]):
                        b["fell"] = bool(env._term_fallen[i])
                        b["damaged"] = bool(env._term_damage[i])
                    rows_out.append({"attempt": batch * num_envs + i, "step": step,
                                     "time_s": round(step * interface.POLICY_DT, 3),
                                     "engaged": int(bool(env._engaged[i])), "inserted": int(bool(layer.inserted[i])),
                                     "searching": int(bool(layer.stall[i] > layer.stall_s)),
                                     "theta": round(float(layer.theta[i]), 3),
                                     "s_mm": round(1000 * float(layer.s[i]), 2), "ref_mm": round(1000 * float(layer.ref[i]), 2),
                                     "depth_mm": round(1000 * float(slot.tip_depth[i]), 2) if bool(slot.active[i]) else "",
                                     "across_u_mm": round(1000 * float(across_u[i]), 2),
                                     "across_w_mm": round(1000 * float(across_w[i]), 2),
                                     "force_cmd_n": round(float(layer.force_cmd[i].norm()), 2),
                                     "key_force_n": round(float(force[i].norm()), 2), "key_axial_n": round(float(axial[i]), 2),
                                     "key_lateral_n": round(float(lateral[i]), 2),
                                     "approach_err_deg": round(float(approach_err[i]), 2),
                                     "roll_err_deg": round(float(roll_err[i]), 2),
                                     "base_pitch_deg": round(float(pitch[i]), 2), "fell": int(bool(ended[i] and env._term_fallen[i]))})
                done |= ended
                if hold_steps is not None:
                    for i in range(count):
                        if best[i]["seated_step"] is not None and step - best[i]["seated_step"] >= hold_steps:
                            done[i] = True
                if bool(done[:count].all()):
                    break
                if args.realtime:
                    # Never a backlog: a slow stretch is not made up by running fast afterwards.
                    deadline = max(deadline + env.step_dt, time.perf_counter())
                    time.sleep(max(0.0, deadline - time.perf_counter()))
        trace.writerows(rows_out)
        trace_file.flush()
        if closed:
            # The window closed mid-batch: its attempts did not finish and are left out of the summary.
            break
        for i in range(count):
            b = best[i]
            row = rows[i]
            seated = b["seated_step"] is not None and not b["fell"] and not b["damaged"]
            attempts.append({"attempt": batch * num_envs + i, "success": seated, **row, **b,
                             "time_to_seat_s": (round((b["seated_step"] - b["engage_step"]) * interface.POLICY_DT, 3)
                                                if b["seated_step"] is not None and b["engage_step"] is not None else None)})
            a = attempts[-1]
            outcome = (f"seated in {a['time_to_seat_s']} s" if a["success"]
                       else "fell" if a["fell"] else "damaged" if a["damaged"]
                       else f"not seated, the tip {a['final_depth_mm']} mm past the face"
                       if a["engage_step"] is not None else "never engaged")
            print(f"[insertion demo] attempt {a['attempt']}: {outcome}  (running: seated "
                  f"{sum(x['success'] for x in attempts)}/{len(attempts)})", flush=True)
        remaining -= count
        batch += 1
        # After every batch, so a loop closed from the window still leaves its finished attempts.
        summary = summarise(attempts)
        (out_dir / "summary.json").write_text(json.dumps({"summary": summary, "attempts": attempts}, indent=2))
        report["summary"] = summary
        (out_dir / "run.json").write_text(json.dumps(report, indent=2))

    trace_file.close()
    summary = summarise(attempts)
    (out_dir / "summary.json").write_text(json.dumps({"summary": summary, "attempts": attempts}, indent=2))
    report["status"] = "closed" if closed else "finished"
    report["summary"] = summary
    (out_dir / "run.json").write_text(json.dumps(report, indent=2))
    print(f"\n[insertion demo] {controller}: seated {summary['successes']}/{summary['attempts']}  "
          f"searched {summary['searched']}  falls {summary['falls']}  damaged {summary['damaged']}  "
          f"time {summary['time_to_seat_s_median']} s  peak force {summary['peak_force_n_median']} N  "
          f"approach {summary['approach_err_deg_median']} deg  roll {summary['roll_err_deg_median']} deg", flush=True)
    env.close()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

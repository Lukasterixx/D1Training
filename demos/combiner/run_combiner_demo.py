"""Resting Go2 + D1 beside a randomized solar combiner box in Isaac Sim.

R: next attempt. H: depress/release handle. O/C: pull door open/closed. F: release inspection drives.
Without --turn the arm holds its resting pose. With --turn it finds the door's AprilTag with the wrist
camera, grips the lever in its jaws, turns it through 45 degrees and pulls the door open
(`sequence.TurnSequence`; `--method press` only pushes the lever down, as F-071's runs did). Several
--handle_torque_nm values try each placement once per spring, which measures the most it can turn.
The wrist carries the cup pick's RealSense, published for the reach console as the pick does (`wrist_camera`).
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import random
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from demos.combiner import wrist_camera
from demos.combiner.geometry import DEFAULT_HANDLE_TORQUE_NM, GEOMETRY, sample_placement, validate_region


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


class SceneKeys:
    def __init__(self):
        import carb.input
        import omni.appwindow

        self.events = []
        self.interface = carb.input.acquire_input_interface()
        self.keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self.subscription = self.interface.subscribe_to_keyboard_events(self.keyboard, self.on_key)

    def on_key(self, event, *_):
        import carb.input

        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input.name in ("R", "H", "O", "C", "F"):
            self.events.append(event.input.name)
        return True

    def take(self):
        events, self.events = self.events, []
        return events

    def close(self):
        self.interface.unsubscribe_to_keyboard_events(self.keyboard, self.subscription)


def run(args):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    run_dir = Path(args.output).resolve() / f"{stamp}_combiner_seed{args.seed}"
    run_dir.mkdir(parents=True)
    metadata = {"mode": "combiner_turn" if args.turn else "combiner_scene", "status": "initializing",
                "seed": args.seed, "arguments": vars(args), "episodes": [],
                "scope": (("AprilTag-guided grasp of the lever, 45 degree turn and door pull" if args.method == "pull"
                           else "AprilTag-guided lever push; door opening not attempted")
                          + " in simulation, lying dog, arm only (IK reference); the latch releases at 45 degrees"
                          if args.turn else "Scene setup and manual door inspection; no autonomous opening controller")}
    write_json(run_dir / "run.json", metadata)
    app = env = keys = latch = feed = None
    latch_callback_registered = False
    try:
        from isaaclab.app import AppLauncher

        app = AppLauncher(headless=args.headless, device=args.device, enable_cameras=True).app
        import torch
        from isaaclab.envs import ManagerBasedRLEnv
        from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
        from isaaclab.utils.io import dump_yaml

        from demos.combiner.asset import build_combiner_usd
        from demos.combiner.latch import LatchController
        from demos.combiner.scene import make_combiner_cfg, overview_view
        from demos.common.resting import LYING_LEG_POSE, LYING_SPAWN_HEIGHT_M
        from weld import build_welded_robot_usd

        rng = random.Random(args.seed)

        def next_placement():
            return sample_placement(rng, tuple(args.box_range), args.sweep_deg, args.yaw_jitter_deg)

        placement = next_placement()
        torques = list(args.handle_torque_nm)
        asset = build_combiner_usd(run_dir / "assets/combiner_box.usda", torques[0])
        robot_usd = str(Path(args.robot_usd).resolve()) if args.robot_usd else build_welded_robot_usd(
            go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
            d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"), out_usd_path=str(run_dir / "assets/go2_d1.usd"),
            mount_pos=(0.0, 0.0, 0.08), arm_mass_kg=3.152).usd_path
        metadata.update(asset=asset, robot_usd=robot_usd,
                        posture={"legs": LYING_LEG_POSE, "spawn_height_m": LYING_SPAWN_HEIGHT_M},
                        arm_model={"actuator": "d1_servo", "trajectory": "measured", "latency": "estimated"})
        camera, mount, metadata["camera"] = wrist_camera.resolve(args)
        print(f"[combiner] wrist camera: {metadata['camera']['model_source']}; mount: "
              f"{metadata['camera']['mount_source']}", flush=True)
        camera_usd = wrist_camera.build_body(metadata["camera"]) if args.camera_body else None
        cfg = make_combiner_cfg(robot_usd, asset["usd_path"], placement, args.seed, args.device,
                                door_angle_deg=args.door_angle_deg, handle_angle_deg=args.handle_angle_deg,
                                capture=args.capture, camera=camera, mount=mount, camera_usd=camera_usd,
                                handle_torque_nm=torques[0])
        dump_yaml(str(run_dir / "env.yaml"), cfg)
        env = ManagerBasedRLEnv(cfg=cfg)
        robot, box = env.scene["robot"], env.scene["combiner"]
        if not box.is_fixed_base or set(box.joint_names) != {"DoorHinge", "HandleJoint"}:
            raise RuntimeError(f"Expected fixed combiner with DoorHinge and HandleJoint; got {box.joint_names}")
        latch = LatchController(box)
        env.sim.add_physics_callback("combiner_latch", latch.on_physics_step)
        latch_callback_registered = True
        origin = env.scene.env_origins[0]
        wrist = env.scene["wrist_cam"]
        wrist_camera.note_rendered(metadata["camera"], camera, wrist)
        feed = wrist_camera.open_feed(args, camera, wrist.data.intrinsic_matrices[0].cpu().numpy())
        metadata["ui_feed"] = feed.url if feed else None
        if args.capture:
            overview = env.scene["overview_cam"]
        if args.turn:
            from demos.combiner.apriltag import TagDetector
            from demos.combiner.asset import spring_record
            from demos.combiner.pull import PullParams
            from demos.combiner.turn_run import TurnAttempt
            from position_only.workspace import load_urdf

            urdf_joints, urdf_links = load_urdf()
            # The intrinsics the renderer used, which is what the image obeys (F-070).
            detector = TagDetector(wrist.data.intrinsic_matrices[0].cpu().numpy())
            metadata["turn"] = {"detector": "OpenCV ArUco DICT_APRILTAG_36h11 + solvePnP IPPE_SQUARE, rendered intrinsics",
                                "method": args.method, "door_target_deg": args.door_deg,
                                "mount_calibration": args.mount_calibration, "handle_torques_nm": torques,
                                "springs": [spring_record(v) for v in torques]}
        keys = None if args.headless else SceneKeys()
        actions = torch.zeros((1, env.action_manager.total_action_dim), device=env.device)
        print(f"[combiner] logs: {run_dir}", flush=True)
        print("[combiner] R: reset | H: turn/release handle | O/C: pull open/closed | F: release drives", flush=True)
        metadata["status"] = "running"
        episode = 0
        steps_per_episode = max(1, math.ceil(args.episode_s / env.step_dt))
        with torch.inference_mode():
            while app.is_running() and (args.episodes == 0 or episode < args.episodes):
                for attempt, torque in enumerate(torques):
                    if not app.is_running():
                        break
                    latch.enabled = False
                    obs, _ = env.reset()
                    pose = torch.tensor([[*placement.position, *placement.quaternion]], device=env.device)
                    pose[:, :3] += origin
                    box.write_root_pose_to_sim(pose)
                    latch.set_spring(torque)
                    latch.reset(args.door_angle_deg, args.handle_angle_deg)
                    actions.zero_()
                    eye, target = overview_view(placement)
                    eye_w = torch.tensor([eye], device=env.device) + origin
                    target_w = torch.tensor([target], device=env.device) + origin
                    if not args.headless:
                        env.sim.set_camera_view(eye=eye_w[0].cpu().tolist(), target=target_w[0].cpu().tolist())
                    if args.capture:
                        overview.set_world_poses_from_view(eye_w, target_w)
                    row = {"episode": episode, "attempt": attempt, "handle_torque_nm": torque, **asdict(placement),
                           "handle_position_env_m": placement.world_point(
                               GEOMETRY.handle_at_angle(args.door_angle_deg, args.handle_angle_deg)),
                           "initial_door_angle_deg": args.door_angle_deg,
                           "initial_handle_angle_deg": args.handle_angle_deg,
                           "latch_transitions": latch.transitions, "inspection_events": []}
                    metadata["episodes"].append(row)
                    write_json(run_dir / "run.json", metadata)
                    print(f"[combiner] episode {episode}.{attempt}: box ({placement.position[0]:.3f}, "
                          f"{placement.position[1]:.3f}) m, yaw {placement.yaw_deg:.1f} deg, handle spring "
                          f"{torque:.3f} N·m at 45 deg", flush=True)
                    if args.turn:
                        step = 0
                        # The dog settles onto its folded legs before the arm starts, as in the pick.
                        for _ in range(math.ceil(args.settle_s / env.step_dt)):
                            obs, *_ = env.step(actions)
                            step += 1
                        attempt_dir = run_dir / f"attempt_{len(metadata['episodes']) - 1:03d}"
                        turn = TurnAttempt(env, latch, urdf_joints, urdf_links, mount, detector, attempt_dir, torque,
                                           mount_calibration=args.mount_calibration,
                                           overview=overview if args.capture else None, method=args.method,
                                           pull=PullParams(door_deg=args.door_deg, handle_torque_nm=torque,
                                                           **grasp_params(args)))

                        pace = {"deadline": time.perf_counter()}

                        def on_step(state):
                            wrist_camera.publish(feed, robot, wrist, origin, env.common_step_counter * env.step_dt,
                                                 f"turn: {state}")
                            if args.realtime:
                                pace["deadline"] += env.step_dt
                                time.sleep(max(0.0, pace["deadline"] - time.perf_counter()))

                        summary = turn.run(app, obs, keys=keys, on_step=on_step, max_s=args.attempt_s)
                        row.update(summary)
                        row["directory"] = attempt_dir.name
                        step += int(round(summary["simulated_s"] / env.step_dt))
                        if args.capture:
                            import cv2

                            rgb = overview.data.output["rgb"][0, ..., :3].cpu().numpy()
                            cv2.imwrite(str(attempt_dir / "overview_end.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                        door = (f", door held open at {summary.get('door_held_open_deg')} deg of "
                                f"{summary.get('door_planned_deg')} planned, grip slip max "
                                f"{summary.get('grip', {}).get('slip_max_mm')} mm" if args.method == "pull" else "")
                        print(f"[combiner] attempt {len(metadata['episodes']) - 1}: {summary['outcome']}, peak lever "
                              f"{summary.get('peak_handle_deg')} deg, held {summary.get('hold', {}).get('handle_deg')} "
                              f"deg, latch released {summary['latch_released']}{door}", flush=True)
                    else:
                        step = 0
                        deadline = time.perf_counter()
                        captured = False
                        while app.is_running():
                            events = keys.take() if keys else []
                            if "R" in events:
                                break
                            for key in events:
                                accepted = latch.command(key)
                                if not accepted:
                                    print("[combiner] Door is latched. Press H and let the handle turn before O.", flush=True)
                                row["inspection_events"].append({"time_s": step * env.step_dt, "key": key,
                                                                 "accepted": accepted, "latched": latch.latch.latched})
                            _, _, terminated, truncated, _ = env.step(actions)
                            if bool(terminated.any() or truncated.any()):
                                raise RuntimeError("Unexpected automatic reset in scene-only demo")
                            step += 1
                            wrist_camera.publish(feed, robot, wrist, origin, env.common_step_counter * env.step_dt,
                                                 f"combiner scene, episode {episode}")
                            if args.capture and not captured and step >= min(steps_per_episode, math.ceil(2 / env.step_dt)):
                                import cv2

                                rgb = overview.data.output["rgb"][0, ..., :3].cpu().numpy()
                                frame = run_dir / f"scene_{episode:03d}.png"
                                if not cv2.imwrite(str(frame), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                                    raise RuntimeError(f"Could not save {frame}")
                                row["overview"] = frame.name
                                wrist_camera.save_frame(wrist, run_dir / f"wrist_{episode:03d}.png")
                                row["wrist_view"] = f"wrist_{episode:03d}.png"
                                captured = True
                            if args.episodes and step >= steps_per_episode:
                                break
                            if args.realtime:
                                deadline += env.step_dt
                                time.sleep(max(0.0, deadline - time.perf_counter()))
                    for state in (robot.data.root_state_w, robot.data.joint_pos, box.data.root_state_w, box.data.joint_pos):
                        if not bool(torch.isfinite(state).all()):
                            raise RuntimeError("Non-finite simulation state")
                    door_deg, handle_deg = latch.angles()
                    row.update(simulated_s=step * env.step_dt,
                               final_door_angle_deg=door_deg, final_handle_angle_deg=handle_deg,
                               final_latched=latch.latch.latched,
                               robot_root_position_w_m=robot.data.root_pos_w[0].cpu().tolist())
                    write_json(run_dir / "run.json", metadata)
                episode += 1
                placement = next_placement()
        metadata["status"] = "completed" if args.episodes and episode >= args.episodes else "closed"
        return 0
    except Exception:
        metadata["status"] = "failed"
        metadata["error"] = traceback.format_exc()
        raise
    finally:
        write_json(run_dir / "run.json", metadata)
        if keys is not None:
            keys.close()
        if feed is not None:
            feed.close()
        if latch_callback_registered:
            latch.enabled = False
            env.sim.remove_physics_callback("combiner_latch")
        if env is not None:
            env.close()
        if app is not None:
            app.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--box_range", type=float, nargs=2, default=(0.62, 0.70), metavar=("MIN", "MAX"),
                        help="Enclosure centre distance from the dog spawn in metres (minimum 0.60).")
    parser.add_argument("--sweep_deg", type=float, default=45.0,
                        help="Placement sector either side of forward; 180 samples all around the dog.")
    parser.add_argument("--yaw_jitter_deg", type=float, default=10.0,
                        help="Yaw variation either side of facing the dog (0–30 degrees).")
    parser.add_argument("--door_angle_deg", type=float, default=0.0, help="Initial opening angle (0–110).")
    parser.add_argument("--handle_angle_deg", type=float, default=0.0,
                        help="Initial lever depression (0–60); the return spring then acts.")
    parser.add_argument("--episodes", type=int, default=None,
                        help="Timed placements; default 1 headless, unlimited manual resets in the viewer.")
    parser.add_argument("--episode_s", type=float, default=5.0, help="Seconds per timed placement.")
    parser.add_argument("--capture", action="store_true",
                        help="Save an overview and a wrist-camera PNG for each placement.")
    parser.add_argument("--realtime", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--robot_usd", help="Reuse an existing welded Go2+D1 USD instead of rebuilding it.")
    parser.add_argument("--output", default=str(ROOT / "logs/combiner_demo"))
    turn = parser.add_argument_group("turning the lever (--turn)")
    turn.add_argument("--turn", action="store_true",
                      help="Find the door's AprilTag with the wrist camera and work the handle (--method).")
    turn.add_argument("--method", choices=("pull", "press"), default="pull",
                      help="'pull': grip the lever, turn it 45 deg and pull the door open. 'press': push the lever "
                           "down with the fingers and let it back up, door shut (F-071's runs).")
    turn.add_argument("--door_deg", type=float, default=60.0,
                      help="How far to pull the door open (deg); the plan stops where the arm stops reaching.")
    turn.add_argument("--handle_torque_nm", type=float, nargs="+", default=[DEFAULT_HANDLE_TORQUE_NM],
                      metavar="NM", help="Return-spring torque at 45 deg (N·m). Several values try each placement "
                                         "once per spring, in order: a sweep for the most the arm can turn.")
    turn.add_argument("--grasp", type=float, nargs=2, default=None, metavar=("PITCH_DEG", "ROLL"),
                      help="Take this one grasp instead of choosing (pitch 0-90 deg above level, wrist roll +1 or "
                           "-1; see demos/combiner/pull.py): for finding out which grasps the jaws hold.")
    turn.add_argument("--mount_calibration", choices=("sim", "none"), default="sim",
                      help="'sim': locate the tag with the camera pose the renderer used (a perfect hand-eye "
                           "calibration), so the torque test is not a perception test. 'none': the requested mount.")
    turn.add_argument("--attempt_s", type=float, default=90.0, help="Longest a --turn attempt may run (s).")
    turn.add_argument("--settle_s", type=float, default=2.0, help="Settling before a --turn attempt starts (s).")
    wrist_camera.add_arguments(parser)
    args = parser.parse_args(argv)
    if args.episodes is None:
        args.episodes = 1 if args.headless else 0
    if args.realtime is None:
        args.realtime = not args.headless
    try:
        if not 0 <= args.seed < 2**32:
            raise ValueError("seed must be between 0 and 2**32 - 1")
        validate_region(tuple(args.box_range), args.sweep_deg, args.yaw_jitter_deg)
        if not 0 <= args.door_angle_deg <= GEOMETRY.open_limit_deg:
            raise ValueError("door_angle_deg must be between 0 and 110")
        if not 0 <= args.handle_angle_deg <= GEOMETRY.handle_limit_deg:
            raise ValueError("handle_angle_deg must be between 0 and 60")
        if not math.isfinite(args.episode_s) or args.episode_s <= 0:
            raise ValueError("episode_s must be finite and positive")
        if args.episodes < 0 or (args.headless and args.episodes == 0):
            raise ValueError("episodes must be positive in headless mode, or zero for manual viewer resets")
        if not all(math.isfinite(v) and 0 < v <= 5.0 for v in args.handle_torque_nm):
            raise ValueError("handle_torque_nm values must be finite, positive and at most 5 N·m")
        if not 20.0 <= args.door_deg <= GEOMETRY.open_limit_deg:
            raise ValueError(f"door_deg must be between 20 and {GEOMETRY.open_limit_deg:.0f}")
        if args.grasp is not None and not (0.0 <= args.grasp[0] <= 90.0 and args.grasp[1] in (1.0, -1.0)):
            raise ValueError("grasp is a pitch of 0-90 deg and a roll of 1 or -1")
        for name in ("attempt_s", "settle_s"):
            if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        wrist_camera.validate(args)
    except ValueError as error:
        parser.error(str(error))
    return args


def grasp_params(args) -> dict:
    """`PullParams` fields for `--grasp`, or none to let the planner choose."""
    return {} if args.grasp is None else {"pitch_deg": float(args.grasp[0]), "roll": int(args.grasp[1])}


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))

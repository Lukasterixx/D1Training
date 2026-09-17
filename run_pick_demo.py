"""Simulated cup pick: Go2 lying down, the D1 with its measured motion model, a wrist RealSense and stock YOLO.

    python run_pick_demo.py --headless
    python run_pick_demo.py                            # in the viewer, paced to real time; R: new cup position

No learning anywhere: YOLO (COCO weights) finds the cup, depth and forward kinematics place it, `d1_ik`
plans a top-down grasp and the scripted `pick_demo.sequence` drives the arm through the same interface
the position-only task uses -- 10 Hz setpoints into the fitted firmware planner (F-045), 9 Hz joint
feedback. See `pick_demo/` for what each piece assumes.

Writes logs/pick_demo/<UTC stamp>_pick_seed<seed>/: run.json, pick.json (outcome, and every estimate
against the simulator's ground truth), events.json, trace.csv, env.yaml, pick.mp4 (annotated wrist view
beside an overview) and frames/ (each frame the pick looked at, annotated). In the viewer, R resets the robot
and puts the cup somewhere new (`random_cup`); each restart writes the same files into episode_NN/, and
episodes.csv lists every episode.

While it runs, the joints and the wrist camera are published on localhost:8765 for the reach console's sim
mode (`./run_ui.sh`, `d1_ui/sim_feed.py`); nothing is copied unless the console is reading.

Scope: a success here shows the pipeline closes in simulation with ideal rendering, a CAD gripper, a
primitive cup collider and a camera mount that does not exist yet. It is not evidence the real arm
can do it.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parent
# CC BY 4.0, by fayazg1aa on Sketchfab: see third_party/sketchfab_cup/NOTICE.md.
DEFAULT_CUP = ROOT / "pick_demo/assets/High-Resolution_3D_Cup_Model_FBX.usdz"
DEFAULT_WEIGHTS = ROOT / "generated/yolo/yolo11s-seg.pt"


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def git_output(*args):
    result = subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None


def sha256(path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def snapshot_sources(run_dir):
    sources = sorted((ROOT / "pick_demo").glob("*.py")) + sorted((ROOT / "position_only").glob("*.py"))
    sources += [ROOT / name for name in ("run_pick_demo.py", "d1_ik.py", "d1_ui/sim_feed.py", "flat_env_cfg.py", "weld.py",
                                         "motor_model.py", "unitree_actuators.py", "d1_arm/d1.urdf")]
    hashes = {}
    for source in sources:
        relative = source.relative_to(ROOT)
        dest = run_dir / "source" / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
        hashes[str(relative)] = sha256(source)
    return hashes


def quat_to_matrix(q):
    import numpy as np

    w, x, y, z = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# Where R puts the cup, relative to the robot's spawn point: inside the positions the final set picked from
# (x 0.36-0.44, |y| <= 0.12; 0.48 was marginal), with the handle within 45 deg of pointing straight away from or
# straight at the robot. Across the jaw axis the handle blocks the descent (F-049), so that band is excluded.
RANDOM_CUP_X = (0.36, 0.44)
RANDOM_CUP_Y = (-0.10, 0.10)
RANDOM_HANDLE_BAND_DEG = 45.0


def random_cup(rng):
    """(x, y) and a handle yaw in degrees for the next episode."""
    x, y = float(rng.uniform(*RANDOM_CUP_X)), float(rng.uniform(*RANDOM_CUP_Y))
    away = math.degrees(math.atan2(y, x))
    offset = float(rng.uniform(-RANDOM_HANDLE_BAND_DEG, RANDOM_HANDLE_BAND_DEG))
    return (x, y), away + offset + (180.0 if rng.random() < 0.5 else 0.0)


class RestartKey:
    """R in the viewer asks for a restart. Kit delivers keys on its own thread, so only a flag crosses over."""

    def __init__(self):
        import carb.input
        import omni.appwindow

        self.requested = False
        self._iface = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._subscription = self._iface.subscribe_to_keyboard_events(self._keyboard, self._on_key)

    def _on_key(self, event, *args, **kwargs):
        import carb.input

        # Presses only: a held key's repeats would otherwise skip episodes.
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input.name == "R":
            self.requested = True
            print("[pick] R pressed: restarting with the cup somewhere new", flush=True)
        return True

    def take(self) -> bool:
        requested, self.requested = self.requested, False
        return requested

    def close(self):
        self._iface.unsubscribe_to_keyboard_events(self._keyboard, self._subscription)


def annotate(frame_rgb, state, t, perception, observation, model, camera_pose_b, cup_estimate):
    """BGR image with the detections, the fitted rim (magenta) and the state."""
    import cv2
    import numpy as np

    from pick_demo.camera import invert, project

    image = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    for detection in perception.last_detections if perception is not None else []:
        colour = (60, 200, 60) if detection.label == "cup" else (0, 200, 230)
        if detection.mask is not None:
            contours, _ = cv2.findContours(detection.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, colour, 1)
        x1, y1, x2, y2 = (int(round(c)) for c in detection.box)
        cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(image, f"{detection.label} {detection.confidence:.2f}", (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
    estimate = observation.estimate if observation is not None else cup_estimate
    if estimate is not None and camera_pose_b is not None:
        angles = np.linspace(0.0, 2 * np.pi, 48)
        up = np.array([0.0, 0.0, 1.0])
        e1 = np.array([1.0, 0.0, 0.0])
        e2 = np.cross(up, e1)
        rim = estimate.top_centre_b + estimate.radius_m * (np.cos(angles)[:, None] * e1 + np.sin(angles)[:, None] * e2)
        cam = (invert(camera_pose_b) @ np.c_[rim, np.ones(len(rim))].T).T[:, :3]
        if (cam[:, 2] > 0.02).all():
            pixels = np.round(project(model, cam)).astype(np.int32)
            cv2.polylines(image, [pixels], True, (220, 0, 220), 2)
    cv2.putText(image, f"{t:5.1f}s  {state}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return image


def run(args):
    from isaaclab.app import AppLauncher

    app = AppLauncher(headless=args.headless, device=args.device, enable_cameras=True).app
    env = None
    failed = False
    video = None
    feed = None
    try:
        import cv2
        import numpy as np
        import torch
        from isaaclab.envs import ManagerBasedRLEnv
        from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
        from isaaclab.utils.io import dump_yaml

        from pick_demo.camera import (CAMERAS, MeasuredMount, WristMount, camera_pose, invert, link6_pose,
                                      mount_to_dict, realsense_depth, transform)
        from pick_demo.cup_asset import build_cup_usd
        from pick_demo.grasp import CLOSED_GAP_M, JAW_CENTRE_LINK6, GraspParams
        from pick_demo.perception import CupPerception, Frame, YoloDetector
        from pick_demo.scene import LYING_LEG_POSE, OVERVIEW_EYE, OVERVIEW_TARGET, make_pick_cfg
        from pick_demo.sequence import PickSequence, Timing
        from position_only.env_cfg import ARM_NAMES
        from position_only.workspace import load_urdf
        from weld import build_welded_robot_usd

        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        run_dir = Path(args.output).resolve() / f"{stamp}_pick_seed{args.seed}"
        (run_dir / "frames").mkdir(parents=True, exist_ok=False)
        # A calibration file describes the camera actually on the bench; the presets are the datasheet's
        # idea of one. Measured on D435I 238222076237 the two differ enough to matter (F-053), so say in
        # the record which was used rather than leaving "d435" to mean either.
        if args.calibration:
            from pick_demo.realsense import load_camera_model

            model = load_camera_model(args.calibration)
            camera_source = f"calibration file {Path(args.calibration).resolve()}"
        else:
            model = CAMERAS[args.camera]
            camera_source = f"preset {args.camera} (datasheet-derived, not a calibration)"
        # A saved mount file (the console's editor, or pick_demo.camera.save_mount) is six degrees of
        # freedom and carries its own provenance; --mount_pos/--mount_pitch_deg are the four-number
        # placeholder for a bracket nobody has measured. The file wins when given.
        from pick_demo.camera import resolve_mount

        mount, mount_source, mount_file = resolve_mount(
            args.mount, fallback=WristMount(tuple(args.mount_pos), args.mount_pitch_deg))
        print(f"[pick] wrist mount: {mount_source}", flush=True)
        packages = {}
        for name in ("isaacsim", "isaaclab", "torch", "ultralytics", "numpy", "opencv-python-headless"):
            try:
                packages[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                packages[name] = None
        metadata = {
            "status": "initializing", "mode": "pick", "seed": args.seed, "arguments": vars(args),
            "git_commit": git_output("rev-parse", "HEAD"), "git_status": git_output("status", "--short"),
            "source_sha256": snapshot_sources(run_dir), "packages": packages,
            "task": "scripted_top_down_cup_pick", "learning": None,
            "camera": {**{k: v for k, v in model.__dict__.items()},
                       "mount": mount_to_dict(mount, mount_source), "mount_source": mount_source,
                       "mount_file": mount_file,
                       "model_source": camera_source, "body_drawn": bool(args.camera_body),
                       "depth_noise": args.depth_noise},
            "posture": {"legs": LYING_LEG_POSE, "source": "unitree_ros2 go2_stand_example.cpp target_pos_1"},
            "detector": {"weights": str(Path(args.weights).resolve()), "weights_sha256": sha256(args.weights),
                         "classes": "COCO, no fine-tuning"},
        }
        write_json(run_dir / "run.json", metadata)
        try:
            robot_usd = str(Path(args.robot_usd).resolve()) if args.robot_usd else build_welded_robot_usd(
                go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
                d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"), out_usd_path=str(run_dir / "assets/go2_d1.usd"),
                mount_pos=(0.0, 0.0, 0.08), arm_mass_kg=3.152).usd_path
            metadata["robot_usd"], metadata["robot_usd_sha256"] = robot_usd, sha256(robot_usd)
            cup_info = build_cup_usd(args.cup_usdz, ROOT / "generated/pick_demo", diameter_m=args.cup_diameter,
                                     height_m=args.cup_height, mass_kg=args.cup_mass, wall_m=args.cup_wall)
            metadata["cup"] = {**cup_info, "spawn_xy_env_m": list(args.cup_xy), "yaw_deg": args.cup_yaw_deg}

            camera_usd = None
            if args.camera_body:
                from pick_demo import camera_body as camera_body_geom
                from pick_demo.camera_asset import build_camera_usd, carve_lens
                from pick_demo.grasp import PALM_X_RANGE_M, PALM_Z_M

                body_info = build_camera_usd(ROOT / "generated/pick_demo")
                camera_usd = body_info["usd_path"]
                metadata["camera"]["body_asset"] = body_info
                for line in camera_body_geom.describe(mount):
                    print(f"[pick] {line}", flush=True)
                clearance = camera_body_geom.clearance_report(mount, PALM_Z_M, PALM_X_RANGE_M)
                metadata["camera"]["body_clearance"] = clearance
                if clearance["intersects_shell"]:
                    print("[pick] WARNING: the camera housing overlaps the Link6 shell at this mount", flush=True)
                # The guard, asked twice: where the model puts the eye, and where F-052 says the
                # renderer puts it. The second is what the wrist images will actually show.
                _points, _faces = camera_body_geom.mesh_optical()
                _spawned = _points[carve_lens(_points, _faces)[0]]
                obstruction = {
                    "model_eye": camera_body_geom.view_obstruction(model, points=_spawned),
                    "rendered_eye_f052": camera_body_geom.view_obstruction(
                        model, camera_body_geom.RENDERED_EYE_OFFSET_M, points=_spawned)}
                metadata["camera"]["view_obstruction"] = obstruction
                if obstruction["rendered_eye_f052"]:
                    print("[pick] WARNING: at the F-052 rendered eye the case is in shot; the wrist "
                          "view is obstructed. Use --no_camera_body for a pick that needs it.", flush=True)

            cfg = make_pick_cfg(robot_usd, cup_info["usd_path"], model, mount, tuple(args.cup_xy), args.cup_yaw_deg,
                                args.seed, args.device, episode_s=args.max_time + (3600.0 if args.linger else 30.0),
                                show_camera_body=args.camera_body, camera_usd=camera_usd)
            grasp_params = GraspParams(wall_grasp=args.wall_grasp, pinch_closed_gap_m=args.pinch_closed_gap)
            metadata["grasp"] = {"wall_grasp": grasp_params.wall_grasp,
                                 "pinch_closed_gap_m": grasp_params.pinch_closed_gap_m,
                                 "wall_thickness_m": grasp_params.wall_thickness_m,
                                 "why": ("a cup too wide for the 77.2 mm jaws is grasped by its wall: pinched "
                                         "if the jaws shut below the wall, else from inside the mouth")}
            metadata["sim2real"] = {
                "leg_actuator": "unitree", "arm_actuator": "d1_servo", "latency": "estimated",
                "arm_trajectory": cfg.actions.arm.trajectory, "arm_command_hold_steps": cfg.actions.arm.hold_steps,
                "arm_feedback_period_steps": cfg.observations.policy.arm_joint_pos.params["period_steps"],
                "arm_action_scale_rad": cfg.actions.arm.scale, "policy_hz": 50, "physics_hz": 200,
                "gripper": "implicit drive, URDF 15 N effort and 0.02 m/s limits; gripper timing not measured on the D1",
            }
            dump_yaml(str(run_dir / "env.yaml"), cfg)
            env = ManagerBasedRLEnv(cfg=cfg)
            scene = env.scene
            robot, cup, wrist, overview = scene["robot"], scene["cup"], scene["wrist_cam"], scene["overview_cam"]
            origin = scene.env_origins[0]
            overview.set_world_poses_from_view(torch.tensor([OVERVIEW_EYE], device=env.device) + origin,
                                               torch.tensor([OVERVIEW_TARGET], device=env.device) + origin)
            arm_ids = [robot.joint_names.index(n) for n in ARM_NAMES]
            grip_ids = [robot.joint_names.index(n) for n in ("Joint7_1", "Joint7_2")]
            # Let the modelled fingers shut as far as the real ones do. The URDF stops each at zero
            # travel, where the CAD pads are still 17.2 mm apart; the arm on the bench closes until they
            # touch (F-063), and a pinch of a cup wall lives entirely in the travel between the two. So a
            # run told the jaws shut to less than the CAD's gap widens the joint limits to match, and
            # says so -- this is the one place where the simulated robot is deliberately not the URDF's.
            shut_travel = grasp_params.pinch_closed_gap_m
            shut_travel = min(0.0, (shut_travel - CLOSED_GAP_M) / 2.0)
            metadata["sim2real"]["gripper_shut_travel_m"] = round(float(shut_travel), 5)
            if shut_travel < 0.0:
                limits = robot.data.joint_pos_limits[:, grip_ids].clone()
                # Targets are clamped to the *soft* limits, which sit a factor inside the hard ones, so
                # the hard limit has to be opened further than the travel actually wanted or the pinch
                # would stop short of the wall by exactly the margin nobody could see.
                factor = float(robot.cfg.soft_joint_pos_limit_factor)
                span = float(limits[0, 0, 1] - limits[0, 0, 0])
                reach = (2 * (shut_travel - 0.0005) - span * (1 - factor)) / (1 + factor)
                limits[..., 0] = torch.minimum(limits[..., 0], torch.full_like(limits[..., 0], reach))
                limits[..., 1] = torch.maximum(limits[..., 1], torch.full_like(limits[..., 1], -reach))
                robot.write_joint_position_limit_to_sim(limits, joint_ids=grip_ids)
                soft = robot.data.soft_joint_pos_limits[0, grip_ids].cpu().numpy()
                print(f"[pick] gripper: fingers allowed {1000 * -shut_travel:.1f} mm of travel past the "
                      f"URDF's stop, so the pads can shut to "
                      f"{1000 * grasp_params.pinch_closed_gap_m:.1f} mm as the real arm does (F-063); "
                      f"soft limits now {1000 * soft[0, 0]:.1f} to {1000 * soft[0, 1]:.1f} mm",
                      flush=True)
                if soft[0, 0] > shut_travel:
                    print(f"[pick] WARNING: the soft limit stops the fingers at {1000 * soft[0, 0]:.1f} mm, "
                          f"short of the {1000 * shut_travel:.1f} mm a shut jaw needs; a pinch will not "
                          f"close on the wall.", flush=True)
                metadata["sim2real"]["gripper"] = (
                    f"implicit drive, URDF 15 N effort and 0.02 m/s limits; finger limits widened by "
                    f"{1000 * -shut_travel:.1f} mm past the URDF's stop so the pads shut to "
                    f"{1000 * grasp_params.pinch_closed_gap_m:.1f} mm (F-063), which is not the imported "
                    f"model; gripper timing not measured on the D1")
            link6 = robot.body_names.index("Link6")
            default_arm = robot.data.default_joint_pos[0, arm_ids].cpu().numpy()
            terms = env.observation_manager.active_terms["policy"]
            dims = env.observation_manager.group_obs_term_dim["policy"]
            offsets = np.cumsum([0] + [int(np.prod(d)) for d in dims])
            slices = {name: slice(int(offsets[i]), int(offsets[i + 1])) for i, name in enumerate(terms)}

            if args.ui_feed_port:
                from d1_ui.sim_feed import SimFeed

                feed = SimFeed("pick_demo", camera=True, port=args.ui_feed_port,
                               camera_info={"model": model.name, "width": model.width, "height": model.height})
                metadata["ui_feed"] = feed.url

            joints, links = load_urdf()
            detector = YoloDetector(str(args.weights), device=args.device)
            perception = CupPerception(detector, model, joints, mount)
            rng = np.random.default_rng(args.seed) if args.depth_noise else None
            step_dt = env.step_dt

            def base_pose():
                return robot.data.root_pos_w[0].cpu().numpy(), quat_to_matrix(robot.data.root_quat_w[0].cpu().numpy())

            def to_base(p_w):
                pos, rot = base_pose()
                return rot.T @ (np.asarray(p_w) - pos)

            def cup_truth():
                pos = cup.data.root_pos_w[0].cpu().numpy()
                rot = quat_to_matrix(cup.data.root_quat_w[0].cpu().numpy())
                top_w = pos + rot @ np.array([0.0, 0.0, args.cup_height])
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))
                return pos, to_base(top_w), tilt

            def jaw_truth_b():
                pose = robot.data.body_link_pose_w[0, link6].cpu().numpy()
                return to_base(pose[:3] + quat_to_matrix(pose[3:]) @ np.asarray(JAW_CENTRE_LINK6))

            actions = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)
            restart = RestartKey() if not args.headless else None
            cup_rng = np.random.default_rng(args.seed + 1)
            episodes = []
            episode, cup_xy, cup_yaw = 0, tuple(args.cup_xy), args.cup_yaw_deg

            def paced(step_start):
                if args.realtime:
                    time.sleep(max(0.0, step_dt - (time.perf_counter() - step_start)))

            def publish_ui(status):
                """The reach console's sim mode: only copies off the GPU while the console is reading."""
                if feed is None:
                    return
                sim_time = env.common_step_counter * step_dt
                if feed.wants_state():
                    feed.publish_joints(robot.joint_names, robot.data.joint_pos[0].cpu().numpy(), sim_time_s=sim_time,
                                        base_height_m=float(robot.data.root_pos_w[0, 2] - origin[2]), status=status)
                if feed.wants_frame():
                    feed.publish_frame(wrist.data.output["rgb"][0, ..., :3].cpu().numpy(), sim_time_s=sim_time)

            while app.is_running():
                out_dir = run_dir if episode == 0 else run_dir / f"episode_{episode:02d}"
                (out_dir / "frames").mkdir(parents=True, exist_ok=True)
                # The simulator's tensors were made under inference mode, so resetting them must be too.
                with torch.inference_mode():
                    obs, _ = env.reset()
                    if episode > 0:
                        yaw = math.radians(cup_yaw) / 2.0
                        pose = torch.tensor([[cup_xy[0], cup_xy[1], 0.001, math.cos(yaw), 0.0, 0.0, math.sin(yaw)]],
                                            device=env.device)
                        pose[:, :3] += origin
                        cup.write_root_pose_to_sim(pose)
                        cup.write_root_velocity_to_sim(torch.zeros(1, 6, device=env.device))
                    actions.zero_()
                    robot.set_joint_position_target(torch.zeros(1, 2, device=env.device), joint_ids=grip_ids)
                if restart is not None:
                    restart.take()
                print(f"[pick] episode {episode}: cup at ({cup_xy[0]:.3f}, {cup_xy[1]:.3f}) m, handle yaw "
                      f"{cup_yaw:.0f} deg", flush=True)
                with torch.inference_mode():
                    for _ in range(int(round(args.settle / step_dt))):
                        if not app.is_running():
                            break
                        step_start = time.perf_counter()
                        obs, *_ = env.step(actions)
                        publish_ui("settling")
                        paced(step_start)
                if not app.is_running():
                    break
                base_height = float(robot.data.root_pos_w[0, 2] - origin[2])
                cup_start_w, _, _ = cup_truth()
                settled = {"base_height_m": round(base_height, 4),
                           "base_xy_env_m": [round(float(v), 4) for v in (robot.data.root_pos_w[0, :2] - origin[:2])],
                           "cup_start_env_m": [round(float(v), 4) for v in cup_start_w - origin.cpu().numpy()],
                           "cup_yaw_deg": round(cup_yaw, 2), "settle_s": args.settle}
                if episode == 0:
                    metadata["settled"] = settled
                print(f"[pick] settled lying down: base {100 * base_height:.1f} cm above the floor", flush=True)
                write_json(run_dir / "run.json", metadata)

                sequence = PickSequence(joints, links, model, mount, perception, base_height_m=base_height,
                                        grasp_params=grasp_params, timing=Timing(settle_s=0.5))
                captured = {}

                def frame_source():
                    rgb = wrist.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
                    depth = wrist.data.output["distance_to_image_plane"][0, ..., 0].cpu().numpy()
                    frame = Frame(rgb, realsense_depth(model, depth, rng), captured["q_fb"].copy(),
                                  captured["up"].copy(), captured["t"])
                    captured["frame"] = frame
                    captured["state"] = sequence.state
                    captured["truth_top_b"] = cup_truth()[1]
                    # The rendered camera against the model the estimate uses: FK of the feedback angles and the mount.
                    sim_pos_b = to_base(wrist.data.pos_w[0].cpu().numpy())
                    _, base_rot = base_pose()
                    sim_rot_b = base_rot.T @ quat_to_matrix(wrist.data.quat_w_ros[0].cpu().numpy())
                    if args.mount_calibration == "sim" and "calibrated_mount" not in metadata["camera"]:
                        # Where the renderer really put the camera, relative to Link6 by forward kinematics: the
                        # simulated equivalent of a perfect hand-eye calibration. Constant in Link6 (F-052).
                        measured = invert(link6_pose(joints, frame.q)) @ transform(sim_rot_b, sim_pos_b)
                        perception.mount = MeasuredMount.from_pose(measured, "simulator render pose at the first look")
                        metadata["camera"]["calibrated_mount"] = {
                            "pose_link6": np.round(measured, 6).tolist(),
                            "offset_from_requested_mm": np.round(1000 * (measured[:3, 3] - np.asarray(mount.pos_link6)), 2).tolist()}
                    model_pose = camera_pose(joints, frame.q, perception.mount)
                    cos = np.clip((np.trace(model_pose[:3, :3].T @ sim_rot_b) - 1.0) / 2.0, -1.0, 1.0)
                    captured["camera_check"] = {
                        "position_error_mm": [round(1000 * float(v), 2) for v in model_pose[:3, 3] - sim_pos_b],
                        "rotation_error_deg": round(math.degrees(math.acos(cos)), 3),
                        "intrinsics_sim": [round(float(v), 2) for v in wrist.data.intrinsic_matrices[0].flatten().tolist()],
                    }
                    return frame

                if not args.no_video:
                    video = cv2.VideoWriter(str(out_dir / "pick.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                                            1.0 / (args.video_every * step_dt), (model.width + 640, model.height))
                trace_file = (out_dir / "trace.csv").open("w", newline="")
                trace = csv.writer(trace_file)
                trace.writerow(["t", "state", *[f"q_cmd_{i}" for i in range(6)], *[f"q_fb_{i}" for i in range(6)],
                                *[f"q_true_{i}" for i in range(6)], "gripper_cmd_m", "finger_1_m", "finger_2_m",
                                "finger_1_target_m", "finger_2_target_m", "cup_x_b", "cup_y_b",
                                "cup_top_z_b", "cup_tilt_deg", "jaw_x_b", "jaw_y_b", "jaw_z_b", "base_height_m"])
                looks = []
                t = 0.0
                aborted = False
                wall_start = time.perf_counter()
                last_image = np.zeros((model.height, model.width, 3), np.uint8)
                with torch.inference_mode():
                    while app.is_running() and not sequence.done and t < args.max_time:
                        if restart is not None and restart.requested:
                            aborted = True
                            break
                        step_start = time.perf_counter()
                        policy = obs["policy"][0]
                        q_fb = policy[slices["arm_joint_pos"]].cpu().numpy() + default_arm
                        up = -policy[slices["projected_gravity"]].cpu().numpy()
                        captured.update(q_fb=q_fb, up=up, t=t, frame=None)
                        command = sequence.update(t, q_fb, up, frame_source)
                        if command.q is not None:
                            actions[0, 12:] = torch.as_tensor((command.q - default_arm) / cfg.actions.arm.scale,
                                                              device=env.device, dtype=torch.float32)
                        robot.set_joint_position_target(
                            torch.tensor([[command.gripper_m, -command.gripper_m]], device=env.device), joint_ids=grip_ids)

                        if captured["frame"] is not None:
                            frame = captured["frame"]
                            observation = sequence.last_observation
                            pose = camera_pose(joints, frame.q, perception.mount)
                            image = annotate(frame.rgb, command.state, t, perception, observation, model, pose, sequence.cup)
                            last_image = image
                            name = f"{len(looks):03d}_{t:05.1f}s_{captured['state']}.png"
                            cv2.imwrite(str(out_dir / "frames" / name), image)
                            truth = captured["truth_top_b"]
                            entry = {"t": round(t, 3), "state": captured["state"], "image": f"frames/{name}",
                                     "camera_model_vs_sim": captured["camera_check"],
                                     "detections": [{"label": d.label, "confidence": round(d.confidence, 3)}
                                                    for d in perception.last_detections],
                                     "truth_top_centre_b": [round(float(v), 4) for v in truth]}
                            if observation is not None:
                                error = observation.estimate.top_centre_b - truth
                                entry.update(estimate=observation.estimate.as_dict(),
                                             horizontal_error_mm=round(1000 * float(np.hypot(error[0], error[1])), 2),
                                             height_error_mm=round(1000 * float(error[2]), 2))
                            looks.append(entry)

                        obs, _, terminated, truncated, _ = env.step(actions)
                        if bool(terminated[0] or truncated[0]):
                            raise RuntimeError(f"environment reset during the pick at t={t:.2f} s")
                        t += step_dt
                        step = int(round(t / step_dt))
                        publish_ui(sequence.state)

                        cup_w, cup_top_b, tilt = cup_truth()
                        jaw_b = jaw_truth_b()
                        q_true = robot.data.joint_pos[0, arm_ids].cpu().numpy()
                        trace.writerow([f"{t:.3f}", command.state,
                                        *[f"{v:.5f}" for v in (command.q if command.q is not None else [math.nan] * 6)],
                                        *[f"{v:.5f}" for v in q_fb], *[f"{v:.5f}" for v in q_true],
                                        f"{command.gripper_m:.4f}",
                                        *[f"{float(v):.4f}" for v in robot.data.joint_pos[0, grip_ids]],
                                        *[f"{float(v):.4f}" for v in robot.data.joint_pos_target[0, grip_ids]],
                                        *[f"{v:.4f}" for v in cup_top_b[:2]], f"{cup_top_b[2]:.4f}", f"{tilt:.2f}",
                                        *[f"{v:.4f}" for v in jaw_b], f"{float(robot.data.root_pos_w[0, 2] - origin[2]):.4f}"])
                        if video is not None and step % args.video_every == 0:
                            side = overview.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
                            side = cv2.resize(cv2.cvtColor(side, cv2.COLOR_RGB2BGR), (640, model.height))
                            live = wrist.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
                            live = cv2.cvtColor(live, cv2.COLOR_RGB2BGR)
                            cv2.putText(live, f"{t:5.1f}s  {sequence.state}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                        (255, 255, 255), 2, cv2.LINE_AA)
                            video.write(np.hstack([live if sequence.state not in ("detect", "refine") else last_image, side]))
                        paced(step_start)
                trace_file.close()
                if video is not None:
                    video.release()
                    video = None

                cup_end_w, cup_top_b, tilt = cup_truth()
                jaw_b = jaw_truth_b()
                lift = float(cup_end_w[2] - cup_start_w[2])
                axis_gap = float(np.hypot(*(cup_top_b[:2] - jaw_b[:2])))
                # How far the jaw centre was *meant* to sit from the cup's axis. Zero for the grasps that
                # straddle the cup or go inside it, and a radius for a wall pinch, which holds the cup by
                # one wall on purpose. Scoring the pinch against a zero offset failed a run that lifted
                # the cup 11.9 cm (Week 1 log, 2026-09-17), so what is checked is the departure from the
                # plan, not the distance from the axis.
                planned_offset = 0.0
                if sequence.plan is not None and sequence.cup is not None:
                    planned_offset = float(np.hypot(*(sequence.plan.jaw_grasp_b[:2] - sequence.cup.top_centre_b[:2])))
                axis_error = abs(axis_gap - planned_offset)
                success = sequence.state == "done" and lift >= 0.05 and axis_error <= 0.03
                first = [l for l in looks if "estimate" in l and l["state"] == "detect"]
                refine = [l for l in looks if "estimate" in l and l["state"] == "refine"]
                result = {
                    "success": success,
                    "criterion": ("sequence finished, cup base raised >= 5 cm, and the cup axis within 3 cm "
                                  "of where the plan put the jaw centre relative to it (on the axis for an "
                                  "outside or inside-out grasp, a wall's radius away for a pinch)"),
                    "episode": episode, "settled": settled,
                    "final_state": "aborted" if aborted else sequence.state,
                    "failure": "restarted with R before the pick finished" if aborted else sequence.failure,
                    "sim_time_s": round(t, 2), "wall_time_s": round(time.perf_counter() - wall_start, 1),
                    "cup_lift_m": round(lift, 4), "cup_axis_to_jaw_centre_m": round(axis_gap, 4),
                    "planned_jaw_offset_from_axis_m": round(planned_offset, 4),
                    "axis_error_m": round(axis_error, 4),
                    "cup_tilt_deg_end": round(tilt, 2), "base_height_m": round(base_height, 4),
                    "first_look_errors_mm": [{"horizontal": l["horizontal_error_mm"], "height": l["height_error_mm"],
                                              "method": l["estimate"]["method"]} for l in first],
                    "refine_errors_mm": [{"horizontal": l["horizontal_error_mm"], "height": l["height_error_mm"],
                                          "method": l["estimate"]["method"]} for l in refine],
                    "frames_looked_at": len(looks),
                    "frames_with_cup_detection": sum(1 for l in looks if l["detections"]),
                    "grasp_plan": sequence.plan.as_dict() if sequence.plan is not None else None,
                    "cup_estimate_used": sequence.cup.as_dict() if sequence.cup is not None else None,
                    "first_cup_estimate": sequence.first_cup.as_dict() if sequence.first_cup is not None else None,
                    "interpretation": "Simulation only: ideal rendering, depth with range limits and best-case stereo "
                                      "noise, CAD gripper, primitive cup collider, assumed camera mount. Not evidence "
                                      "about the real arm."
                                      + ("" if sequence.plan is None or sequence.plan.mode == "outside" else
                                         f" The grasp was a {sequence.plan.mode} wall grasp against "
                                         f"{cup_info['collision']}, and the CAD gripper's travel limits: what "
                                         f"the real jaws do at the same command is unmeasured (F-059)."),
                }
                write_json(out_dir / "pick.json", result)
                write_json(out_dir / "events.json", {"events": sequence.events, "looks": looks})
                episodes.append({"episode": episode, "dir": "." if episode == 0 else out_dir.name,
                                 "cup_x_env_m": round(cup_xy[0], 4), "cup_y_env_m": round(cup_xy[1], 4),
                                 "cup_yaw_deg": round(cup_yaw, 1),
                                 "grasp_mode": sequence.plan.mode if sequence.plan is not None else "",
                                 "final_state": result["final_state"],
                                 "success": success, "cup_lift_m": result["cup_lift_m"],
                                 "cup_axis_to_jaw_centre_mm": round(1000 * axis_gap, 1),
                                 "axis_error_mm": round(1000 * axis_error, 1), "sim_time_s": result["sim_time_s"],
                                 "failure": result["failure"] or ""})
                with (run_dir / "episodes.csv").open("w", newline="") as handle:
                    writer = csv.DictWriter(handle, fieldnames=list(episodes[0]))
                    writer.writeheader()
                    writer.writerows(episodes)
                completed = [e for e in episodes if e["final_state"] != "aborted"]
                wins = sum(e["success"] for e in completed)
                if len(episodes) == 1:
                    metadata["status"] = "pick_succeeded" if success else "pick_failed"
                else:
                    metadata["status"] = f"picks_succeeded_{wins}_of_{len(completed)}"
                metadata["pick"] = {k: result[k] for k in ("success", "final_state", "failure", "cup_lift_m",
                                                          "cup_axis_to_jaw_centre_m", "sim_time_s")}
                metadata["episodes"] = {"count": len(episodes), "completed": len(completed), "succeeded": wins,
                                        "random_cup_x_env_m": list(RANDOM_CUP_X), "random_cup_y_env_m": list(RANDOM_CUP_Y),
                                        "random_handle_band_deg": RANDOM_HANDLE_BAND_DEG}
                write_json(run_dir / "run.json", metadata)
                outcome = "aborted" if aborted else ("succeeded" if success else "failed")
                print(f"[pick] episode {episode} {outcome}: {sequence.plan.mode if sequence.plan else 'no'} grasp, "
                      f"lift {100 * lift:.1f} cm, axis gap {1000 * axis_gap:.0f} mm "
                      f"({1000 * axis_error:.0f} mm off the plan), {t:.1f} s simulated -> {out_dir}", flush=True)

                if not aborted and episode + 1 >= args.episodes:
                    if not (args.linger and app.is_running()):
                        break
                    # Results are written; hold the last commands so the viewer can be looked around.
                    print("[pick] holding the final pose; press R for a new cup position, or close the window.",
                          flush=True)
                    with torch.inference_mode():
                        while app.is_running() and not (restart is not None and restart.requested):
                            step_start = time.perf_counter()
                            env.step(actions)
                            publish_ui("holding (R: new cup)")
                            paced(step_start)
                if not app.is_running():
                    break
                episode += 1
                cup_xy, cup_yaw = random_cup(cup_rng)
            if restart is not None:
                restart.close()
        except Exception as exc:
            metadata["status"] = "failed"
            metadata["error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            write_json(run_dir / "run.json", metadata)
    except BaseException:
        failed = True
        traceback.print_exc()
        raise
    finally:
        if video is not None:
            video.release()
        if feed is not None:
            feed.close()
        if env is not None:
            env.close()
        sys.stdout.flush()
        sys.stderr.flush()
        # SimulationApp.close() exits with status 0 before Python reports an exception; exit nonzero ourselves.
        if failed:
            os._exit(1)
        app.close()


def main():
    from pick_demo.grasp import GraspParams

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--seed", type=int, default=42, help="Seeds the depth noise and the command/feedback phases.")
    parser.add_argument("--camera", choices=("d435", "d405", "d455"), default="d435",
                        help="RealSense preset: intrinsics and depth range (pick_demo/camera.py).")
    parser.add_argument("--mount_pos", type=float, nargs=3, default=(-0.055, 0.0, 0.035), metavar=("X", "Y", "Z"),
                        help="Camera position in the Link6 frame (m). Assumed, not measured.")
    parser.add_argument("--mount_pitch_deg", type=float, default=20.0, help="Camera tilt towards the approach axis.")
    parser.add_argument("--mount", default=None, metavar="FILE",
                        help="A saved wrist-mount file (six degrees of freedom, with provenance). "
                             "Default: pick_demo/assets/mounts/wrist_mount.json when it exists. "
                             "'none' forces --mount_pos/--mount_pitch_deg instead.")
    parser.add_argument("--calibration", default=None,
                        help="A calibration from pick_demo.realsense, used instead of --camera's datasheet preset. "
                             "This is the camera on the bench rather than the datasheet's idea of one.")
    parser.add_argument("--camera_body", action=argparse.BooleanOptionalAction, default=True,
                        help="Draw the RealSense housing at the mount, to check it against the real bracket. "
                             "Visual only: no collider and no mass, but it does obstruct the wrist camera "
                             "while F-052 stands, so a pick that needs the wrist view wants it off.")
    # `BooleanOptionalAction` spells the negative `--no-camera_body`, while run_camera_body_view.py takes
    # `--no_camera_body`, and F-057 and scene.py both name the underscore form. Rather than leave a flag
    # that errors out for anyone following the findings, accept both spellings here.
    parser.add_argument("--no_camera_body", dest="camera_body", action="store_false",
                        help=argparse.SUPPRESS)
    parser.add_argument("--mount_calibration", choices=("none", "sim"), default="none",
                        help="'sim': estimate with the camera pose the renderer actually used (a perfect hand-eye "
                             "calibration). 'none': with the requested mount.")
    parser.add_argument("--depth_noise", action=argparse.BooleanOptionalAction, default=True,
                        help="Stereo depth noise (Intel's RMS model). Range limits apply either way.")
    parser.add_argument("--cup_usdz", default=str(DEFAULT_CUP), help="The cup model to import.")
    parser.add_argument("--cup_diameter", type=float, default=0.055)
    parser.add_argument("--cup_height", type=float, default=0.10)
    parser.add_argument("--cup_mass", type=float, default=0.12)
    parser.add_argument("--cup_wall", type=float, default=0.0, metavar="M",
                        help="Wall thickness of a hollow cup collider, in metres. 0 (the default) keeps the "
                             "solid cylinder every recorded pick used; a wall grasp reaches inside the cup "
                             "and needs an inside to reach into.")
    parser.add_argument("--wall_grasp", choices=("auto", "off", "outside", "wall", "inside_out", "pinch"),
                        default="auto",
                        help="What to do with a cup too wide for the jaws (grasp.GraspParams.wall_grasp): "
                             "auto falls back to an inside-out wall grasp, off refuses as before.")
    parser.add_argument("--pinch_closed_gap", type=float, default=GraspParams().pinch_closed_gap_m,
                        metavar="M",
                        help="What the jaws shut to, in metres. It decides both whether a wall pinch is "
                             "planned and how far the simulated fingers may travel: the default is the real "
                             "arm's (F-063), so the modelled pads close past the URDF's stop to match it. "
                             "Pass 0.0172 to hold the simulator to the URDF's own limit, where no pinch is "
                             "possible and a wide cup is taken from inside instead.")
    parser.add_argument("--cup_xy", type=float, nargs=2, default=(0.42, 0.03), metavar=("X", "Y"),
                        help="Cup position on the floor relative to the robot's spawn point (m).")
    parser.add_argument("--cup_yaw_deg", type=float, default=0.0,
                        help="Cup yaw; 0 points the handle along +x, away from the robot.")
    parser.add_argument("--weights", default=str(DEFAULT_WEIGHTS), help="Ultralytics segmentation weights.")
    parser.add_argument("--settle", type=float, default=3.0, help="Seconds lying still before the pick starts.")
    parser.add_argument("--max_time", type=float, default=90.0, help="Simulated seconds before giving up.")
    parser.add_argument("--video_every", type=int, default=3, help="Policy steps per video frame.")
    parser.add_argument("--episodes", type=int, default=1,
                        help="Picks to run back to back; every one after the first puts the cup somewhere random, "
                             "as R does in the viewer.")
    parser.add_argument("--no_video", action="store_true")
    parser.add_argument("--linger", action=argparse.BooleanOptionalAction, default=None,
                        help="After the pick, keep simulating the final pose until the window closes "
                             "(default: on with the viewer, off headless).")
    parser.add_argument("--realtime", action=argparse.BooleanOptionalAction, default=None,
                        help="Pace to real time (default: on with the viewer, off headless).")
    parser.add_argument("--ui_feed_port", type=int, default=8765,
                        help="Publish joints and the wrist camera here for the reach console's sim mode; 0 turns it off.")
    parser.add_argument("--robot_usd", help="Existing welded robot USD; otherwise rebuilt in the run directory.")
    parser.add_argument("--output", default=str(ROOT / "logs/pick_demo"))
    args = parser.parse_args()
    if args.realtime is None:
        args.realtime = not args.headless
    if args.linger is None:
        args.linger = not args.headless
    if Path(args.weights) == DEFAULT_WEIGHTS and not DEFAULT_WEIGHTS.is_file():
        from pick_demo.perception import ensure_weights

        ensure_weights(DEFAULT_WEIGHTS)     # not committed; fetched once, pinned by hash
    for name in ("cup_usdz", "weights"):
        if not Path(getattr(args, name)).expanduser().is_file():
            parser.error(f"--{name} must point to an existing file: {getattr(args, name)}")
        setattr(args, name, str(Path(getattr(args, name)).expanduser().resolve()))
    run(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Close-up renders of the wrist, to check the simulated RealSense against the bracket on the bench.

The pick's camera is a pose, not an object: `WristMount` places an optical frame on Link6 and the
geometry follows from it. With a bracket now built, the question is whether that assumed pose is where
the real camera actually sits -- and that is a question you answer by looking. This draws the camera
body (`pick_demo.camera_body`) on the wrist and photographs it from several directions, with the arm
held in the pose the pick observes from.

    ./run_camera_body_view.sh                        # four views into logs/camera_body/<stamp>/
    ./run_camera_body_view.sh --mount_pos -0.05 0.0 0.04 --mount_pitch_deg 25
    ./run_camera_body_view.sh --pose observe         # the arm where the pick looks from (default)
    ./run_camera_body_view.sh --pose zero            # every joint at zero, easiest to measure against

Each view is saved under its own name, and `run.json` records the mount that produced them, so a render
can be compared with a later one after the mount is corrected -- and so `./dashboard.py record` takes it
like any other run.

Nothing here moves a real arm and nothing is trained: it is a still life of the wrist.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def git_output(*args):
    import subprocess

    result = subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None

# Where to stand relative to Link6, in the arm's own base frame: (name, offset, up hint). The distances
# are close enough that the 90 mm housing fills the frame.
VIEWS = (
    ("front", (0.16, 0.0, 0.05)),
    ("side", (0.0, -0.18, 0.04)),
    ("above", (0.02, 0.0, 0.20)),
    ("three_quarter", (0.13, -0.13, 0.10)),
)

OBSERVE_POSE_RAD = (0.0, 0.10, 0.83, 0.0, 0.16, 0.0)   # the pick's first viewpoint, from a recorded run


def run(args) -> int:
    from isaaclab.app import AppLauncher

    app = AppLauncher(headless=True, device=args.device, enable_cameras=True).app
    env = None
    try:
        import numpy as np
        import torch
        from isaaclab.envs import ManagerBasedRLEnv

        from pick_demo import camera_body
        from pick_demo.camera import CAMERAS, WristMount, camera_pose, link6_pose, mount_to_dict
        from pick_demo.camera_asset import build_camera_usd
        from pick_demo.cup_asset import build_cup_usd
        from pick_demo.grasp import PALM_X_RANGE_M, PALM_Z_M
        from pick_demo.scene import make_pick_cfg
        from position_only.workspace import load_urdf
        from weld import build_welded_robot_usd
        from isaaclab_assets import ISAACLAB_NUCLEUS_DIR
        import imageio.v3 as iio

        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        out_dir = Path(args.output).resolve() / stamp
        out_dir.mkdir(parents=True, exist_ok=True)

        if args.calibration:
            from pick_demo.realsense import load_camera_model

            model = load_camera_model(args.calibration)
        else:
            model = CAMERAS[args.camera]
        # A saved mount file (the console's editor, or pick_demo.camera.save_mount) is six degrees of
        # freedom and carries its own provenance; --mount_pos/--mount_pitch_deg are the four-number
        # placeholder for a bracket nobody has measured. The file wins when given.
        from pick_demo.camera import resolve_mount

        mount, mount_source, mount_file = resolve_mount(
            args.mount, fallback=WristMount(tuple(args.mount_pos), args.mount_pitch_deg))
        print(f"[view] wrist mount: {mount_source}", flush=True)
        joints, _ = load_urdf()

        for line in camera_body.describe(mount):
            print(f"[view] {line}", flush=True)
        clearance = camera_body.clearance_report(mount, PALM_Z_M, PALM_X_RANGE_M)
        # Ask about the asset that is actually spawned -- the mesh with the lens element carved out --
        # not the raw CAD, or the guard reports a lens that is not in the scene.
        from pick_demo.camera_asset import carve_lens

        _points, _faces = camera_body.mesh_optical()
        _kept, _ = carve_lens(_points, _faces)
        spawned = _points[_kept]
        obstruction = {"model_eye": camera_body.view_obstruction(model, points=spawned),
                       "rendered_eye_f052": camera_body.view_obstruction(
                           model, camera_body.RENDERED_EYE_OFFSET_M, points=spawned)}
        print(f"[view] housing vs the Link6 shell: "
              f"{'OVERLAPS' if clearance['intersects_shell'] else 'clear'}", flush=True)
        for where, found in obstruction.items():
            print(f"[view] aperture at the {where}: {'BLOCKED ' + str(found) if found else 'clear'}", flush=True)

        robot_usd = str(Path(args.robot_usd).resolve()) if args.robot_usd else build_welded_robot_usd(
            go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
            d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"), out_usd_path=str(out_dir / "assets/go2_d1.usd"),
            mount_pos=(0.0, 0.0, 0.08), arm_mass_kg=3.152).usd_path
        cup = build_cup_usd(args.cup_usdz, ROOT / "generated/pick_demo")
        body_info = None if args.no_camera_body else build_camera_usd(ROOT / "generated/pick_demo")
        cfg = make_pick_cfg(robot_usd, cup["usd_path"], model, mount, (0.42, 0.03), 0.0,
                            args.seed, args.device, episode_s=600.0, overview_size=(args.width, args.height),
                            show_camera_body=not args.no_camera_body,
                            camera_usd=None if body_info is None else body_info["usd_path"])
        env = ManagerBasedRLEnv(cfg=cfg)
        env.reset()

        robot = env.scene["robot"]
        arm_ids = [robot.find_joints(f"Joint{i}")[0][0] for i in range(1, 7)]
        default_arm = robot.data.default_joint_pos[0, arm_ids].cpu().numpy()

        # The pose has to be asked for through the action manager, not written to the articulation:
        # the arm action term rewrites the joint targets every step, so a `set_joint_position_target`
        # here is overwritten before the next render (it left the first run's arm at zero). Arm actions
        # are the last six, offset from the default and divided by the term's scale, as the pick does.
        actions = torch.zeros((1, env.action_manager.total_action_dim), device=env.device)
        goal = np.zeros(6) if args.pose == "zero" else np.asarray(OBSERVE_POSE_RAD, dtype=float)
        actions[0, 12:] = torch.as_tensor((goal - default_arm) / cfg.actions.arm.scale,
                                          device=env.device, dtype=actions.dtype)
        for _ in range(args.settle_steps):
            env.step(actions)

        q = np.array([float(robot.data.joint_pos[0, i]) for i in arm_ids])
        origin = env.scene.env_origins[0].detach().cpu().numpy()
        base_pos = robot.data.root_pos_w[0].detach().cpu().numpy() - origin

        link6 = link6_pose(joints, q)
        optical = camera_pose(joints, q, mount)
        print(f"[view] arm at {np.round(q, 4).tolist()} rad", flush=True)
        print(f"[view] Link6 origin in the base frame: {np.round(link6[:3, 3], 4).tolist()} m", flush=True)
        print(f"[view] camera optical origin:          {np.round(optical[:3, 3], 4).tolist()} m", flush=True)

        overview = env.scene["overview_cam"]
        centre_w = origin + base_pos + link6[:3, 3]
        saved = []
        for name, offset in VIEWS:
            eye = centre_w + np.asarray(offset, dtype=float)
            overview.set_world_poses_from_view(
                torch.tensor([eye], device=env.device, dtype=torch.float32),
                torch.tensor([centre_w], device=env.device, dtype=torch.float32))
            for _ in range(args.frames_per_view):
                env.step(actions)
            rgb = overview.data.output["rgb"][0, ..., :3].detach().cpu().numpy().astype("uint8")
            path = out_dir / f"{name}.png"
            iio.imwrite(path, rgb)
            saved.append({"view": name, "eye_offset_m": list(offset), "file": path.name})
            print(f"[view] {path}", flush=True)

        # run.json, not view.json: it is what ./dashboard.py record reads, so a render joins the
        # experimental record the same way a training or pick run does.
        (out_dir / "run.json").write_text(json.dumps({
            "status": "complete", "mode": "camera_body_view", "seed": args.seed,
            "arguments": vars(args),
            "git_commit": git_output("rev-parse", "HEAD"), "git_status": git_output("status", "--short"),
            "task": "wrist camera body render", "learning": None,
            "captured_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "mount": mount_to_dict(mount, mount_source),
            "mount_file": mount_file,
            "camera_model": {k: v for k, v in model.__dict__.items()},
            "arm_pose_rad": [round(float(v), 5) for v in q],
            "pose": args.pose,
            "link6_origin_base_m": [round(float(v), 5) for v in link6[:3, 3]],
            "optical_origin_base_m": [round(float(v), 5) for v in optical[:3, 3]],
            "body_clearance": clearance,
            "view_obstruction": obstruction,
            "body": {
                "asset": body_info,
                "size_m": [round(float(v), 6) for v in camera_body.housing_size_m()],
                "bounds_optical_m": [[round(float(v), 6) for v in corner]
                                     for corner in camera_body.housing_bounds_m()],
                "imagers_optical_m": {k: [round(float(v), 6) for v in p]
                                      for k, p in camera_body.imager_positions_m().items()},
                "tripod_optical_m": [round(float(v), 6) for v in camera_body.tripod_thread_m()],
                "source": "Intel d435.dae (realsense2_description, Apache-2.0), registered by _d435.urdf.xacro",
            },
            "views": saved,
        }, indent=2) + "\n")
        print(f"[view] wrote {out_dir}", flush=True)
        return 0
    finally:
        if env is not None:
            env.close()
        app.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--camera", choices=("d435", "d405", "d455"), default="d435")
    parser.add_argument("--calibration", default=None, help="Use a measured calibration for the camera model.")
    parser.add_argument("--mount_pos", type=float, nargs=3, default=(-0.055, 0.0, 0.035), metavar=("X", "Y", "Z"),
                        help="Camera optical origin in the Link6 frame.")
    parser.add_argument("--mount_pitch_deg", type=float, default=20.0)
    parser.add_argument("--mount", default=None, metavar="FILE",
                        help="A saved wrist-mount file (six degrees of freedom, with provenance). "
                             "Default: pick_demo/assets/mounts/wrist_mount.json when it exists. "
                             "'none' forces --mount_pos/--mount_pitch_deg instead.")
    parser.add_argument("--no_camera_body", action="store_true", help="Render the wrist without the camera drawn.")
    parser.add_argument("--pose", choices=("observe", "zero"), default="observe")
    parser.add_argument("--settle_steps", type=int, default=120)
    parser.add_argument("--frames_per_view", type=int, default=4)
    parser.add_argument("--width", type=int, default=900)
    parser.add_argument("--height", type=int, default=700)
    parser.add_argument("--cup_usdz", default=str(ROOT / "pick_demo/assets/High-Resolution_3D_Cup_Model_FBX.usdz"))
    parser.add_argument("--robot_usd", default=None)
    parser.add_argument("--output", default=str(ROOT / "logs/camera_body"))
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())

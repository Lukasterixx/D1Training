"""The cup pick's wrist RealSense on the combiner scene's arm, and the feed that shows it in the reach console.

The camera is the pick's own (`demos/cup/pick_demo`): the same preset or calibration, the same saved wrist
mount, the same D435 housing drawn at it, and `scene.wrist_camera_cfg`'s render settings. The reach console
(`demos/cup/d1_ui/server.py --mode sim`, started by `run_combiner_demo.sh`) follows this run's feed exactly
as it follows the pick's: the arm and legs from the joints, the wrist view from the camera. Nothing here
acts on the image; it is for looking at the box from the wrist.

Stdlib at import, so the runner's argument checks run without Isaac; the rest is imported on use.
"""
from __future__ import annotations

import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# The pick's housing asset is named by its content hash, so the two demos share one file.
BODY_DIR = ROOT / "generated/pick_demo"
FEED_SOURCE = "combiner_demo"


def add_arguments(parser):
    group = parser.add_argument_group("wrist RealSense and reach console (as in the cup pick)")
    group.add_argument("--camera", choices=("d435", "d405", "d455"), default="d435",
                       help="RealSense preset: intrinsics and depth range (demos/cup/pick_demo/camera.py).")
    group.add_argument("--calibration", default=None, metavar="FILE",
                       help="A calibration from demos.cup.pick_demo.realsense, used instead of --camera's "
                            "datasheet preset.")
    group.add_argument("--mount", default=None, metavar="FILE",
                       help="A saved wrist-mount file. Default: the pick's, "
                            "demos/cup/pick_demo/assets/mounts/wrist_mount.json, when it exists; "
                            "'none' forces the assumed placeholder mount.")
    group.add_argument("--camera_body", action=argparse.BooleanOptionalAction, default=True,
                       help="Draw the RealSense housing at the mount. Visual only: no collider and no mass.")
    group.add_argument("--no_camera_body", dest="camera_body", action="store_false", help=argparse.SUPPRESS)
    group.add_argument("--ui_feed_port", type=int, default=8765,
                       help="Publish joints and the wrist camera here for the reach console's sim mode; "
                            "0 turns it off.")


def validate(args):
    """Raise ValueError for what would otherwise fail after the simulator has spent minutes loading."""
    for name in ("calibration", "mount"):
        value = getattr(args, name)
        if not value or (name == "mount" and value.lower() == "none"):
            continue
        path = Path(value).expanduser()
        if not path.is_file():
            raise ValueError(f"--{name} must point to an existing file: {value}")
        setattr(args, name, str(path.resolve()))
    if not 0 <= args.ui_feed_port <= 65535:
        raise ValueError("ui_feed_port must be between 0 and 65535")


def resolve(args):
    """(model, mount, record): the camera model and wrist mount in force, and what run.json says of them."""
    from demos.cup.pick_demo.camera import CAMERAS, mount_to_dict, resolve_mount

    if args.calibration:
        from demos.cup.pick_demo.realsense import load_camera_model

        model = load_camera_model(args.calibration)
        model_source = f"calibration file {args.calibration}"
    else:
        model = CAMERAS[args.camera]
        model_source = f"preset {args.camera} (datasheet-derived, not a calibration)"
    mount, mount_source, mount_file = resolve_mount(args.mount)
    record = {**model.__dict__, "model_source": model_source, "mount": mount_to_dict(mount, mount_source),
              "mount_source": mount_source, "mount_file": mount_file, "body_drawn": bool(args.camera_body)}
    return model, mount, record


def build_body(record):
    """Write (or reuse) the D435 housing USD and return its path. Needs the Isaac app."""
    from demos.cup.pick_demo.camera_asset import build_camera_usd

    record["body_asset"] = build_camera_usd(BODY_DIR)
    return record["body_asset"]["usd_path"]


def note_rendered(record, model, wrist):
    """The intrinsics the renderer used beside the model's: they differ for a calibration (F-070)."""
    record["intrinsics_model"] = [round(float(v), 2) for v in model.intrinsic_matrix.flatten()]
    record["intrinsics_rendered"] = [round(float(v), 2) for v in wrist.data.intrinsic_matrices[0].flatten().tolist()]


def open_feed(args, model, rendered_k=None):
    """The console's feed. `rendered_k`, the intrinsics the renderer used (F-070), lets the console range
    the door's AprilTag in the frames it is sent (`camera_feed.CameraPipeline`)."""
    if not args.ui_feed_port:
        return None
    from demos.cup.d1_ui.sim_feed import SimFeed

    info = {"model": model.name, "width": model.width, "height": model.height}
    if rendered_k is not None:
        info["K"] = [round(float(v), 4) for v in rendered_k.flatten()]
    return SimFeed(FEED_SOURCE, camera=True, port=args.ui_feed_port, camera_info=info)


def publish(feed, robot, wrist, origin, sim_time_s, status):
    """The console's view of this step. Copies nothing off the GPU unless the console is reading."""
    if feed is None:
        return
    if feed.wants_state():
        feed.publish_joints(robot.joint_names, robot.data.joint_pos[0].cpu().numpy(), sim_time_s=sim_time_s,
                            base_height_m=float(robot.data.root_pos_w[0, 2] - origin[2]), status=status)
    if feed.wants_frame():
        feed.publish_frame(wrist.data.output["rgb"][0, ..., :3].cpu().numpy(), sim_time_s=sim_time_s)


def save_frame(wrist, path):
    import cv2

    rgb = wrist.data.output["rgb"][0, ..., :3].cpu().numpy()
    if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"Could not save {path}")
